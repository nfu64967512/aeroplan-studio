"""3D 畫面 60 FPS（2026-10-17「希望 3D 畫面可以像 X-Plane 一樣順暢」）—— 值層級的鎖。

量到的兩個根因（tools/cesium_fps_probe.py；runs/artifacts/PERF_2026-10-17/fps_probe.json）：
  1. Windows 上 Qt 視窗端每次 present 等 vsync，和 QtWebEngine 出圖的來回串在一起 ⇒ 任何會動的網頁都只有 ~22 FPS。
     修法：QApplication 建立前把 QSurfaceFormat 預設 swapInterval 設 0（ui/render_setup.py；main.py 呼叫）。
  2. vtol.glb／copter.glb 是 CAD 匯出：1192／383 個 primitive ⇒ 每架每幀上千次 draw call，6 架 JS 33 ms。
     修法：ui/widgets/glb_merge.py 把節點變換烘進頂點、依外觀簽名合併（幾何／材質不變），快取在 data/cache/models/。
另外：追機鏡頭偏航加一階阻尼（main.html ``_chaseSmoothYaw``），與幀率無關、走最短角、τ=0 ⇒ 舊的剛性鎖機頭。
"""
from __future__ import annotations

import json
import math
import os
import pathlib
import re
import struct
import time

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO = pathlib.Path(__file__).resolve().parents[1]
MAIN_HTML = pathlib.Path(os.environ.get("RENDER_SMOOTH_MAIN_HTML") or REPO / "ui" / "widgets" / "cesium_templates" / "main.html")
HTML = MAIN_HTML.read_text(encoding="utf-8")

from ui.widgets import glb_merge as gm  # noqa: E402
from ui import render_setup as rs  # noqa: E402
from tests.test_survey_map_layers import _pw, _launch  # noqa: E402,F401  （_pw 是 module 級 fixture）


# ═══════════════════════════════════════════════════════════════════════
#  合成 GLB 的小工具
# ═══════════════════════════════════════════════════════════════════════
class _Glb:
    def __init__(self):
        self.blob = bytearray()
        self.views: list[dict] = []
        self.acc: list[dict] = []

    def _view(self, raw: bytes, stride: int | None = None) -> int:
        while len(self.blob) % 4:
            self.blob.append(0)
        v = {"buffer": 0, "byteOffset": len(self.blob), "byteLength": len(raw)}
        if stride:
            v["byteStride"] = stride
        self.blob.extend(raw)
        self.views.append(v)
        return len(self.views) - 1

    def vec3(self, a) -> int:
        a = np.asarray(a, dtype=np.float32)
        self.acc.append({"bufferView": self._view(a.tobytes()), "componentType": 5126, "count": len(a), "type": "VEC3",
                         "min": a.min(0).tolist(), "max": a.max(0).tolist()})
        return len(self.acc) - 1

    def interleaved(self, pos, nrm) -> tuple[int, int]:
        """POSITION＋NORMAL 交錯在同一個 bufferView（byteStride 24）。"""
        pos = np.asarray(pos, dtype=np.float32)
        nrm = np.asarray(nrm, dtype=np.float32)
        raw = np.hstack([pos, nrm]).astype(np.float32).tobytes()
        v = self._view(raw, stride=24)
        self.acc.append({"bufferView": v, "byteOffset": 0, "componentType": 5126, "count": len(pos), "type": "VEC3",
                         "min": pos.min(0).tolist(), "max": pos.max(0).tolist()})
        self.acc.append({"bufferView": v, "byteOffset": 12, "componentType": 5126, "count": len(pos), "type": "VEC3"})
        return len(self.acc) - 2, len(self.acc) - 1

    def idx(self, a, dtype=np.uint16) -> int:
        a = np.asarray(a, dtype=dtype).reshape(-1)
        self.acc.append({"bufferView": self._view(a.tobytes()), "componentType": 5123 if dtype == np.uint16 else 5125,
                         "count": len(a), "type": "SCALAR"})
        return len(self.acc) - 1

    def build(self, js: dict) -> bytes:
        while len(self.blob) % 4:
            self.blob.append(0)
        js = dict(js, asset={"version": "2.0", "generator": "test"}, accessors=self.acc, bufferViews=self.views,
                  buffers=[{"byteLength": len(self.blob)}])
        jb = json.dumps(js).encode()
        jb += b" " * ((4 - len(jb) % 4) % 4)
        return (struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(jb) + 8 + len(self.blob))
                + struct.pack("<II", len(jb), 0x4E4F534A) + jb + struct.pack("<II", len(self.blob), 0x004E4942)
                + bytes(self.blob))


def _parse(data: bytes):
    js, bin_ = gm._parse_glb(data)
    return js, bin_


def _read(js, bin_, i):
    return gm._read_accessor(js, bin_, i)


# 一個單位四面體（外向法向量、逆時針為正面），供各種變換測試
_TET_P = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float64)
_TET_I = np.array([[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]])


def _unwelded(p, tris):
    """每個三角形三個獨立頂點＋平面法向量（CAD 匯出就是這樣：三角形不共用頂點）。"""
    P, N = [], []
    for t in tris:
        a, b, c = p[t]
        n = np.cross(b - a, c - a)
        n /= np.linalg.norm(n)
        P += [a, b, c]
        N += [n, n, n]
    return np.array(P), np.array(N)


def _scene_glb(extra_prims_mesh0: int = 0) -> tuple[bytes, dict]:
    """mesh0：兩個 primitive 共用同一份 POSITION/NORMAL（交錯），材質 m0／m1 名字不同內容相同；
    mesh1：一個 primitive、材質 m2（不同顏色）、沒有索引。
    節點：root(平移 10,0,0) → A(mesh0, 縮放 2) ; root → B(mesh1, 繞 Z 轉 90°) ; C(mesh0, 縮放 -1,1,1 ⇒ 鏡射)。"""
    g = _Glb()
    P, N = _unwelded(_TET_P, _TET_I)                       # 12 頂點、4 三角形
    pa, na = g.interleaved(P, N)
    i_first = g.idx(np.arange(6))                           # 三角形 0,1
    i_second = g.idx(np.arange(6, 12))                      # 三角形 2,3
    extra = [g.idx(np.arange(12)) for _ in range(extra_prims_mesh0)]
    p1 = g.vec3(P)
    n1 = g.vec3(N)
    mat = {"pbrMetallicRoughness": {"metallicFactor": 0, "roughnessFactor": 1}, "doubleSided": True}
    js = {
        "scene": 0, "scenes": [{"nodes": [0, 3]}],
        "nodes": [
            {"name": "root", "translation": [10, 0, 0], "children": [1, 2]},
            {"name": "A", "mesh": 0, "scale": [2, 2, 2]},
            {"name": "B", "mesh": 1, "rotation": [0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)]},
            {"name": "C", "mesh": 0, "scale": [-1, 1, 1]},
        ],
        "meshes": [
            {"primitives": [{"attributes": {"POSITION": pa, "NORMAL": na}, "indices": i_first, "material": 0},
                            {"attributes": {"POSITION": pa, "NORMAL": na}, "indices": i_second, "material": 1}]
             + [{"attributes": {"POSITION": pa, "NORMAL": na}, "indices": e, "material": 0} for e in extra]},
            {"primitives": [{"attributes": {"POSITION": p1, "NORMAL": n1}, "material": 2}]},
        ],
        "materials": [dict(mat, name="Opaque(0,102,64)"), dict(mat, name="Opaque(138,102,0)"),
                      dict(mat, name="red", pbrMetallicRoughness={"baseColorFactor": [1, 0, 0, 1],
                                                                  "metallicFactor": 0, "roughnessFactor": 1})],
    }
    return g.build(js), js


def _world_tris(js, bin_):
    """把輸出（或輸入）GLB 的每個三角形取成世界座標 (n,3,3) 與其頂點法向量 (n,3,3)。只看 scene 根下的 mesh 節點。"""
    out_p, out_n = [], []
    for pr in js["meshes"][0]["primitives"]:
        pos = _read(js, bin_, pr["attributes"]["POSITION"]).astype(np.float64)
        nrm = _read(js, bin_, pr["attributes"]["NORMAL"]).astype(np.float64)
        idx = _read(js, bin_, pr["indices"]).reshape(-1, 3)
        out_p.append(pos[idx])
        out_n.append(nrm[idx])
    return np.concatenate(out_p), np.concatenate(out_n)


# ═══════════════════════════════════════════════════════════════════════
#  GLB 合併
# ═══════════════════════════════════════════════════════════════════════
def test_merge_bakes_transforms_groups_by_appearance_and_keeps_every_triangle():
    data, _ = _scene_glb()
    out, st = gm.merge_glb_bytes(data)
    js, bin_ = _parse(out)
    assert st["primitives_in"] == 2 + 1 + 2 and st["triangles_in"] == 4 + 4 + 4
    assert st["triangles_out"] == st["triangles_in"]
    prims = js["meshes"][0]["primitives"]
    assert len(prims) == 2, "m0/m1 只有名字不同 ⇒ 同一組；m2 顏色不同 ⇒ 自己一組"
    assert len(js["nodes"]) == 1 and "children" not in js["nodes"][0] and not any(
        k in js["nodes"][0] for k in ("matrix", "translation", "rotation", "scale")), "變換全烘進頂點"
    colors = sorted(json.dumps(js["materials"][p["material"]].get("pbrMetallicRoughness", {}).get("baseColorFactor"))
                    for p in prims)
    assert colors == ["[1, 0, 0, 1]", "null"]
    for p in prims:
        assert set(p["attributes"]) == {"POSITION", "NORMAL"}, "沒有貼圖 ⇒ TEXCOORD 不留"

    tri_p, tri_n = _world_tris(js, bin_)
    # 期望的世界座標：A = root∘scale2、C = scale(-1,1,1)、B = root∘rotZ90
    P, _N = _unwelded(_TET_P, _TET_I)
    A = P * 2 + [10, 0, 0]
    C = P * [-1, 1, 1]
    rot = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    B = P @ rot.T + [10, 0, 0]
    want = np.concatenate([A, C, B]).reshape(-1, 3, 3)
    key = lambda t: tuple(np.round(np.sort(t.reshape(-1)), 5))       # noqa: E731
    assert sorted(map(key, tri_p)) == sorted(map(key, want)), "每個三角形的世界座標都要對得上（不多不少）"


def test_mirrored_instance_flips_winding_so_faces_still_point_along_their_normals():
    data, _ = _scene_glb()
    js, bin_ = _parse(gm.merge_glb_bytes(data)[0])
    tri_p, tri_n = _world_tris(js, bin_)
    face = np.cross(tri_p[:, 1] - tri_p[:, 0], tri_p[:, 2] - tri_p[:, 0])
    dots = np.einsum("ij,ij->i", face, tri_n.mean(axis=1))
    assert (dots > 0).all(), f"鏡射（行列式 < 0）的實例要反轉三角形頂點順序：{dots}"
    assert np.allclose(np.linalg.norm(tri_n, axis=2), 1.0, atol=1e-5), "法向量用逆轉置變換後要重新正規化"


def test_primitives_sharing_one_vertex_buffer_store_it_once_per_node_instance():
    data, _ = _scene_glb(extra_prims_mesh0=5)              # mesh0 共 7 個 primitive 都指向同一份 12 個頂點
    js, bin_ = _parse(gm.merge_glb_bytes(data)[0])
    plain = [p for p in js["meshes"][0]["primitives"]
             if js["materials"][p["material"]]["pbrMetallicRoughness"].get("baseColorFactor") is None][0]
    assert js["accessors"][plain["attributes"]["POSITION"]]["count"] == 12 * 2, \
        "mesh0 被 A、C 兩個節點引用 ⇒ 兩份；同一節點裡的 7 個 primitive 不可以各複製一份（vtol.glb 曾膨脹成 416 MB）"
    assert js["accessors"][plain["indices"]]["count"] == (6 + 6 + 5 * 12) * 2


def test_output_is_a_well_formed_glb_with_exact_position_bounds_and_in_range_indices():
    data, _ = _scene_glb()
    out = gm.merge_glb_bytes(data)[0]
    magic, ver, length = struct.unpack_from("<III", out, 0)
    assert (magic, ver, length) == (0x46546C67, 2, len(out))
    jlen, jtype = struct.unpack_from("<II", out, 12)
    assert jtype == 0x4E4F534A and jlen % 4 == 0
    blen, btype = struct.unpack_from("<II", out, 20 + jlen)
    assert btype == 0x004E4942 and blen % 4 == 0 and 20 + jlen + 8 + blen == len(out)
    js, bin_ = _parse(out)
    for v in js["bufferViews"]:
        assert v["byteOffset"] % 4 == 0
    for p in js["meshes"][0]["primitives"]:
        a = js["accessors"][p["attributes"]["POSITION"]]
        pos = _read(js, bin_, p["attributes"]["POSITION"])
        assert np.allclose(a["min"], pos.min(0)) and np.allclose(a["max"], pos.max(0)), "POSITION 的 min/max 是規格必填"
        idx = _read(js, bin_, p["indices"])
        assert idx.max() < a["count"] and p["mode"] == 4
        assert js["accessors"][p["indices"]]["componentType"] == (5123 if a["count"] <= 65535 else 5125)


def test_large_groups_switch_to_32_bit_indices():
    g = _Glb()
    n = 70000 * 3
    P = np.random.default_rng(0).random((n, 3))
    pa = g.vec3(P)
    js = {"scene": 0, "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}],
          "meshes": [{"primitives": [{"attributes": {"POSITION": pa}}]}]}
    out, st = gm.merge_glb_bytes(g.build(js))
    oj, ob = _parse(out)
    p = oj["meshes"][0]["primitives"][0]
    assert oj["accessors"][p["indices"]]["componentType"] == 5125 and st["triangles_out"] == 70000
    assert "NORMAL" not in p["attributes"] and "materials" not in oj, "沒法向量／沒材質的原樣保持"
    assert np.array_equal(_read(oj, ob, p["indices"]).reshape(-1), np.arange(n))


@pytest.mark.parametrize("breaker", ["texture", "lines", "required_ext", "sparse", "external_buffer", "garbage", "anim"])
def test_anything_unsupported_returns_none_so_the_caller_keeps_the_original(breaker):
    data, js = _scene_glb()
    if breaker == "garbage":
        assert gm.merge_glb_bytes(b"not a glb at all, definitely not") is None
        return
    head = struct.unpack_from("<II", data, 12)[0]
    js = json.loads(data[20:20 + head])
    binb = data[20 + head + 8:]
    if breaker == "texture":
        js["materials"][0]["pbrMetallicRoughness"]["baseColorTexture"] = {"index": 0}
    elif breaker == "lines":
        js["meshes"][1]["primitives"][0]["mode"] = 1
    elif breaker == "required_ext":
        js["extensionsRequired"] = ["KHR_draco_mesh_compression"]
    elif breaker == "sparse":
        js["accessors"][0]["sparse"] = {"count": 1}
    elif breaker == "external_buffer":
        js["buffers"][0]["uri"] = "x.bin"
    elif breaker == "anim":
        js["animations"] = [{"channels": [], "samplers": []}]
    jb = json.dumps(js).encode()
    jb += b" " * ((4 - len(jb) % 4) % 4)
    bad = (struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(jb) + 8 + len(binb)) + struct.pack("<II", len(jb), 0x4E4F534A)
           + jb + struct.pack("<II", len(binb), 0x004E4942) + binb)
    assert gm.merge_glb_bytes(bad) is None


def test_out_of_range_indices_are_refused_not_silently_read():
    g = _Glb()
    pa = g.vec3(_TET_P)
    ia = g.idx([0, 1, 9])
    js = {"scene": 0, "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}],
          "meshes": [{"primitives": [{"attributes": {"POSITION": pa}, "indices": ia}]}]}
    assert gm.merge_glb_bytes(g.build(js)) is None


# ── 快取 ──
def _big_glb(tmp: pathlib.Path) -> pathlib.Path:
    data, _ = _scene_glb(extra_prims_mesh0=gm.MIN_PRIMITIVES_TO_MERGE)
    p = tmp / "vtol.glb"
    p.write_bytes(data)
    return p


def test_cache_is_built_once_reused_and_rebuilt_when_the_source_changes(tmp_path, monkeypatch):
    monkeypatch.setenv("AEROPLAN_GLB_MERGE", "1")
    src = _big_glb(tmp_path)
    cache = tmp_path / "cache"
    first = gm.optimized_model_path(src, cache)
    assert first.parent == cache and first.name.startswith("vtol.merged-") and first.suffix == ".glb"
    assert gm.merge_glb_bytes(src.read_bytes())[0] == first.read_bytes(), "快取內容就是合併結果（決定性）"
    m1 = first.stat().st_mtime_ns
    time.sleep(0.02)
    assert gm.optimized_model_path(src, cache) == first and first.stat().st_mtime_ns == m1, "原檔沒變 ⇒ 不重做"
    os.utime(src, ns=(src.stat().st_atime_ns, src.stat().st_mtime_ns + 5_000_000_000))
    second = gm.optimized_model_path(src, cache)
    assert second != first and second.exists() and not first.exists(), "原檔換了 ⇒ 新鍵、舊快取刪掉"
    assert [p.name for p in cache.iterdir()] == [second.name], "不留暫存檔"


def test_small_files_env_off_unsupported_and_missing_all_return_the_original(tmp_path, monkeypatch):
    cache = tmp_path / "cache"
    monkeypatch.setenv("AEROPLAN_GLB_MERGE", "1")
    small, _ = _scene_glb()                                  # 3 個 primitive < MIN_PRIMITIVES_TO_MERGE
    sp = tmp_path / "small.glb"
    sp.write_bytes(small)
    assert gm.optimized_model_path(sp, cache) == sp
    big = _big_glb(tmp_path)
    monkeypatch.setenv("AEROPLAN_GLB_MERGE", "0")
    assert gm.optimized_model_path(big, cache) == big
    monkeypatch.setenv("AEROPLAN_GLB_MERGE", "1")
    head = struct.unpack_from("<II", big.read_bytes(), 12)[0]
    js = json.loads(big.read_bytes()[20:20 + head])
    js["materials"][0]["pbrMetallicRoughness"]["baseColorTexture"] = {"index": 0}
    jb = json.dumps(js).encode()
    jb += b" " * ((4 - len(jb) % 4) % 4)
    binb = big.read_bytes()[20 + head + 8:]
    tex = tmp_path / "tex.glb"
    tex.write_bytes(struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(jb) + 8 + len(binb)) + struct.pack("<II", len(jb), 0x4E4F534A)
                    + jb + struct.pack("<II", len(binb), 0x004E4942) + binb)
    assert gm.optimized_model_path(tex, cache) == tex
    junk = tmp_path / "junk.glb"
    junk.write_bytes(b"glTF" + b"\x00" * 100)
    assert gm.optimized_model_path(junk, cache) == junk
    assert gm.optimized_model_path(tmp_path / "nope.glb", cache) == tmp_path / "nope.glb"
    assert not cache.exists() or list(cache.iterdir()) == [], "不合併的情況不可以留下任何快取檔"


def test_count_primitives_reads_only_the_json_chunk(tmp_path):
    data, _ = _scene_glb(extra_prims_mesh0=2)
    p = tmp_path / "x.glb"
    p.write_bytes(data)
    assert gm.count_primitives(p) == 5
    (tmp_path / "bad.glb").write_bytes(b"abc")
    assert gm.count_primitives(tmp_path / "bad.glb") == 0


@pytest.mark.parametrize("name", ["vtol.glb", "copter.glb"])
def test_real_cad_models_merge_to_a_handful_of_draw_calls_with_identical_bounds(name):
    src = REPO / "ui" / "resources" / name
    if not src.exists():
        pytest.skip(f"{name} 是 gitignored 的本機資產")
    data = src.read_bytes()
    res = gm.merge_glb_bytes(data)
    assert res is not None
    out, st = res
    assert st["primitives_in"] >= 300 and st["primitives_out"] <= 4, st
    assert st["triangles_out"] == st["triangles_in"] and st["bytes_out"] < st["bytes_in"], st
    js, _ = _parse(data)
    # 原檔沒有節點變換（下一行檢查）⇒ 直接比各 POSITION accessor 的 min/max
    pos_ids = {p["attributes"]["POSITION"] for m in js["meshes"] for p in m["primitives"]}
    assert all(not any(k in n for k in ("matrix", "translation", "rotation", "scale")) for n in js["nodes"])
    lo = np.min([js["accessors"][i]["min"] for i in pos_ids], axis=0)
    hi = np.max([js["accessors"][i]["max"] for i in pos_ids], axis=0)
    oj, _ = _parse(out)
    olo = np.min([oj["accessors"][p["attributes"]["POSITION"]]["min"] for p in oj["meshes"][0]["primitives"]], axis=0)
    ohi = np.max([oj["accessors"][p["attributes"]["POSITION"]]["max"] for p in oj["meshes"][0]["primitives"]], axis=0)
    assert np.allclose(lo, olo, atol=1e-3) and np.allclose(hi, ohi, atol=1e-3), "包圍盒（＝ main.html 的比例／偏移校正基準）不變"


# ── 合併：多層節點、非均勻縮放、鏡射、matrix 節點 —— 對照獨立的參考算法（scipy 四元數）──
# （2026-10-17 審查：只有單一變換的節點時，TRS 順序／matrix 轉置／法向量矩陣／四元數公式錯了測試都抓不到）
def _ref_world(nodes, path):
    from scipy.spatial.transform import Rotation
    M = np.eye(4)
    for i in path:
        n = nodes[i]
        if "matrix" in n:
            L = np.array(n["matrix"], dtype=float).reshape(4, 4, order="F")
        else:
            T = np.eye(4)
            T[:3, 3] = n.get("translation", [0, 0, 0])
            R = np.eye(4)
            R[:3, :3] = Rotation.from_quat(n.get("rotation", [0, 0, 0, 1])).as_matrix()
            S = np.diag(list(n.get("scale", [1, 1, 1])) + [1])
            L = T @ R @ S
        M = M @ L
    return M


def _deep_scene():
    from scipy.spatial.transform import Rotation
    g = _Glb()
    P, N = _unwelded(_TET_P, _TET_I)
    pa, na = g.interleaved(P, N)
    ia = g.idx(np.arange(12))
    p1 = g.vec3(P + [0.3, 0, 0])
    n1 = g.vec3(N)
    root_m = np.eye(4)
    root_m[:3, :3] = Rotation.from_euler("x", 30, degrees=True).as_matrix()
    root_m[:3, 3] = [5, -3, 2]
    axis = np.array([1, 1, 0]) / math.sqrt(2)
    nodes = [
        {"matrix": root_m.reshape(-1, order="F").tolist(), "children": [1, 3]},
        {"translation": [1, 2, 3], "rotation": Rotation.from_rotvec(np.radians(40) * axis).as_quat().tolist(),
         "scale": [2, 0.5, 1.5], "mesh": 0, "children": [2]},
        {"rotation": Rotation.from_euler("z", 90, degrees=True).as_quat().tolist(), "scale": [-1, 1, 1], "mesh": 0},
        {"scale": [1, 1, -2], "translation": [0, 4, 0], "mesh": 1},
    ]
    mat = {"pbrMetallicRoughness": {"metallicFactor": 0, "roughnessFactor": 1}, "doubleSided": True}
    js = {"scene": 0, "scenes": [{"nodes": [0]}], "nodes": nodes,
          "meshes": [{"primitives": [{"attributes": {"POSITION": pa, "NORMAL": na}, "indices": ia, "material": 0}]},
                     {"primitives": [{"attributes": {"POSITION": p1, "NORMAL": n1}, "material": 0}]}],
          "materials": [mat]}
    paths = {(0, 1): P, (0, 1, 2): P, (0, 3): P + [0.3, 0, 0]}
    return g.build(js), nodes, paths


def test_deep_hierarchy_positions_match_an_independent_reference():
    data, nodes, paths = _deep_scene()
    js, bin_ = _parse(gm.merge_glb_bytes(data)[0])
    tri_p, _ = _world_tris(js, bin_)
    want = []
    for path, P in paths.items():
        M = _ref_world(nodes, path)
        want.append((P @ M[:3, :3].T + M[:3, 3]).reshape(-1, 3, 3))
    want = np.concatenate(want)
    key = lambda t: tuple(np.round(np.sort(t.reshape(-1)), 5))       # noqa: E731
    assert sorted(map(key, tri_p)) == sorted(map(key, want)), "TRS 順序／column-major／四元數／父子相乘任一錯都會對不上"


def test_normals_stay_perpendicular_to_faces_under_non_uniform_scale_and_mirroring():
    data, _nodes, _paths = _deep_scene()
    js, bin_ = _parse(gm.merge_glb_bytes(data)[0])
    tri_p, tri_n = _world_tris(js, bin_)
    face = np.cross(tri_p[:, 1] - tri_p[:, 0], tri_p[:, 2] - tri_p[:, 0])
    face /= np.linalg.norm(face, axis=1, keepdims=True)
    for k in range(3):
        n = tri_n[:, k]
        assert np.allclose(np.linalg.norm(n, axis=1), 1.0, atol=1e-5)
        assert np.allclose(np.cross(face, n), 0, atol=1e-5), "法向量要用逆轉置（非均勻縮放下用原矩陣會歪）"
        assert (np.einsum("ij,ij->i", face, n) > 0.999).all(), "鏡射（含父節點鏡射）要反轉頂點順序"


def test_shared_positions_with_different_normals_keep_both_normal_sets():
    g = _Glb()
    P, N = _unwelded(_TET_P, _TET_I)
    pa = g.vec3(P)
    na, nb = g.vec3(N), g.vec3(-N)
    ia = g.idx(np.arange(12))
    js = {"scene": 0, "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}],
          "meshes": [{"primitives": [{"attributes": {"POSITION": pa, "NORMAL": na}, "indices": ia, "material": 0},
                                     {"attributes": {"POSITION": pa, "NORMAL": nb}, "indices": ia, "material": 0}]}],
          "materials": [{"doubleSided": True}]}
    oj, ob = _parse(gm.merge_glb_bytes(g.build(js))[0])
    _, tri_n = _world_tris(oj, ob)
    signs = np.sign(np.einsum("ij,ij->i", tri_n[:, 0], np.tile(N[::3], (2, 1))))
    assert sorted(signs.tolist()) == [-1.0] * 4 + [1.0] * 4, "同一份 POSITION、不同 NORMAL ⇒ 不可以共用頂點（去重鍵要含 NORMAL）"


def test_primitives_with_and_without_normals_are_not_merged_together():
    g = _Glb()
    P, N = _unwelded(_TET_P, _TET_I)
    pa, na = g.vec3(P), g.vec3(N)
    pb = g.vec3(P + 5)
    js = {"scene": 0, "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}],
          "meshes": [{"primitives": [{"attributes": {"POSITION": pa, "NORMAL": na}, "material": 0},
                                     {"attributes": {"POSITION": pb}, "material": 0}]}],
          "materials": [{"doubleSided": True}]}
    oj, _ = _parse(gm.merge_glb_bytes(g.build(js))[0])
    prims = oj["meshes"][0]["primitives"]
    assert sorted("NORMAL" in p["attributes"] for p in prims) == [False, True]


def test_each_appearance_group_keeps_only_the_vertices_it_uses():
    g = _Glb()
    P, N = _unwelded(_TET_P, _TET_I)
    pa, na = g.interleaved(P, N)
    i1, i2 = g.idx(np.arange(3)), g.idx(np.arange(3, 12))
    js = {"scene": 0, "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}],
          "meshes": [{"primitives": [{"attributes": {"POSITION": pa, "NORMAL": na}, "indices": i1, "material": 0},
                                     {"attributes": {"POSITION": pa, "NORMAL": na}, "indices": i2, "material": 1}]}],
          "materials": [{"pbrMetallicRoughness": {"baseColorFactor": [1, 0, 0, 1]}},
                        {"pbrMetallicRoughness": {"baseColorFactor": [0, 1, 0, 1]}}]}
    oj, ob = _parse(gm.merge_glb_bytes(g.build(js))[0])
    counts = sorted(oj["accessors"][p["attributes"]["POSITION"]]["count"] for p in oj["meshes"][0]["primitives"])
    assert counts == [3, 9], counts
    tri_p, _ = _world_tris(oj, ob)
    key = lambda t: tuple(np.round(np.sort(t.reshape(-1)), 5))       # noqa: E731
    assert sorted(map(key, tri_p)) == sorted(map(key, P.reshape(-1, 3, 3)))


def _patched(mutate) -> bytes:
    data, _ = _scene_glb()
    head = struct.unpack_from("<II", data, 12)[0]
    js = json.loads(data[20:20 + head])
    binb = data[20 + head + 8:]
    mutate(js)
    jb = json.dumps(js).encode()
    jb += b" " * ((4 - len(jb) % 4) % 4)
    return (struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(jb) + 8 + len(binb)) + struct.pack("<II", len(jb), 0x4E4F534A)
            + jb + struct.pack("<II", len(binb), 0x004E4942) + binb)


def _set_attr(name):
    def f(js):
        js["meshes"][1]["primitives"][0]["attributes"][name] = 0
    return f


def _instancing(js):
    js["extensionsUsed"] = ["EXT_mesh_gpu_instancing"]
    js["nodes"][1]["extensions"] = {"EXT_mesh_gpu_instancing": {"attributes": {}}}


def _node_ext(js):
    js["nodes"][1]["extensions"] = {"X_foo": {}}


def _prim_ext(js):
    js["meshes"][0]["primitives"][0]["extensions"] = {"KHR_materials_variants": {}}


def _cycle(js):
    js["nodes"][1]["children"] = [0]


@pytest.mark.parametrize("name,mutate", [
    ("COLOR_0", _set_attr("COLOR_0")), ("TANGENT", _set_attr("TANGENT")), ("instancing", _instancing),
    ("node_ext", _node_ext), ("prim_ext", _prim_ext), ("cycle", _cycle),
])
def test_content_that_would_be_lost_or_loop_is_refused(name, mutate):
    assert gm.merge_glb_bytes(_patched(mutate)) is None, name


def test_material_extensions_are_kept_and_declared():
    def add_ext(js):
        js["materials"][2]["extensions"] = {"KHR_materials_emissive_strength": {"emissiveStrength": 2.0}}
        js["extensionsUsed"] = ["KHR_materials_emissive_strength"]
    oj, _ = _parse(gm.merge_glb_bytes(_patched(add_ext))[0])
    assert oj["extensionsUsed"] == ["KHR_materials_emissive_strength"]
    assert any((m.get("extensions") or {}).get("KHR_materials_emissive_strength") for m in oj["materials"])


def test_without_scenes_only_true_roots_are_walked_so_children_are_not_duplicated():
    def drop(js):
        del js["scenes"]
        del js["scene"]
    a = gm.merge_glb_bytes(_scene_glb()[0])[1]
    b = gm.merge_glb_bytes(_patched(drop))[1]
    assert a["triangles_out"] == b["triangles_out"] == 12, (a, b)


@pytest.mark.parametrize("off", ["0", "false", "OFF", " no "])
def test_merge_env_off_values(tmp_path, monkeypatch, off):
    src = _big_glb(tmp_path)
    monkeypatch.setenv("AEROPLAN_GLB_MERGE", off)
    assert gm.optimized_model_path(src, tmp_path / "c") == src


def test_a_failed_cache_rename_leaves_no_temp_file_and_uses_a_cache_another_process_wrote(tmp_path, monkeypatch):
    monkeypatch.setenv("AEROPLAN_GLB_MERGE", "1")
    src = _big_glb(tmp_path)
    cache = tmp_path / "cache"

    def deny(a, b):
        raise PermissionError(5, "Access is denied")
    monkeypatch.setattr(gm.os, "replace", deny)
    assert gm.optimized_model_path(src, cache) == src
    assert list(cache.iterdir()) == [], "暫存檔要清掉（審查重現：WinError 5 留下 ~90 MB 的 .tmp）"

    def race(a, b):                                          # 另一個行程搶先寫好同一個快取
        pathlib.Path(b).write_bytes(pathlib.Path(a).read_bytes())
        raise PermissionError(5, "Access is denied")
    monkeypatch.setattr(gm.os, "replace", race)
    got = gm.optimized_model_path(src, cache)
    assert got != src and got.exists() and [x.name for x in cache.iterdir()] == [got.name]


# ── 接線：CesiumMapWidget 三個模型都走合併快取 ──
def test_widget_resolves_all_three_models_through_the_merge_cache(monkeypatch):
    try:
        from ui.widgets.cesium_map_widget import CesiumMapWidget
        import config.settings as cs
    except Exception as e:                                   # pragma: no cover
        pytest.skip(f"Qt 匯入失敗：{e}")
    seen = []

    def fake(p, cache):
        seen.append((pathlib.Path(p).name, pathlib.Path(cache)))
        return pathlib.Path(p).with_name(pathlib.Path(p).stem + ".MERGED.glb")
    monkeypatch.setattr(gm, "optimized_model_path", fake)
    monkeypatch.setattr(cs, "get_settings", lambda: (_ for _ in ()).throw(RuntimeError("不讀設定檔")))
    monkeypatch.setattr(CesiumMapWidget, "_resolve_token", lambda self: "")

    class _W(CesiumMapWidget):
        def __del__(self):
            pass
    vals = _W.__new__(_W)._cesium_template_values()
    present = {n for n in ("barracuda500.glb", "copter.glb", "vtol.glb", "hero.glb", "pioneer.glb")
               if (REPO / "ui" / "resources" / n).exists()}
    assert {n for n, _ in seen} == present
    assert all(c == REPO / "data" / "cache" / "models" for _, c in seen)
    for key, n in (("__PLANE_MODEL_URI__", "barracuda500"), ("__COPTER_MODEL_URI__", "copter"), ("__VTOL_MODEL_URI__", "vtol")):
        if f"{n}.glb" in present:
            assert vals[key].endswith(f"{n}.MERGED.glb"), (key, vals[key])
        else:
            assert vals[key] == ""
    af = vals["/*__AIRFRAME_MODELS__*/"]                    # 機體模型換進的是一行 JS（各 URI＋機體預設＋逐架）
    for name, var in (("hero", "_HERO_MODEL_URI"), ("pioneer", "_PIONEER_MODEL_URI")):
        if f"{name}.glb" in present:
            assert f'{var} = "' in af and f'{name}.MERGED.glb"' in af, af
        else:
            assert f'{var} = "";' in af, af


def test_test_session_does_not_write_model_caches_by_default():
    assert os.environ.get("AEROPLAN_GLB_MERGE") == "0", "conftest 要預設關掉，否則建 widget 的測試會寫 180 MB 進 data/cache"


# ═══════════════════════════════════════════════════════════════════════
#  Qt 呈現：swapInterval 0
# ═══════════════════════════════════════════════════════════════════════
@pytest.mark.parametrize("platform,env,want", [
    ("win32", None, 0), ("linux", None, None), ("darwin", None, None),
    ("win32", "1", 1), ("linux", "0", 0), ("win32", " 0 ", 0), ("win32", "-1", None),
    ("win32", "default", None), ("win32", "", None),
])
def test_desired_swap_interval(monkeypatch, platform, env, want):
    monkeypatch.setattr(rs.sys, "platform", platform)
    if env is None:
        monkeypatch.delenv(rs.ENV_SWAP_INTERVAL, raising=False)
    else:
        monkeypatch.setenv(rs.ENV_SWAP_INTERVAL, env)
    assert rs.desired_swap_interval() == want


def test_configure_sets_the_default_surface_format_and_leaves_it_alone_when_asked(monkeypatch):
    from PyQt6.QtGui import QSurfaceFormat
    saved = QSurfaceFormat.defaultFormat()
    logs = []

    class _L:
        def info(self, m): logs.append(m)
        def warning(self, m): logs.append("W:" + m)
    try:
        f = QSurfaceFormat.defaultFormat(); f.setSwapInterval(1); QSurfaceFormat.setDefaultFormat(f)
        monkeypatch.setenv(rs.ENV_SWAP_INTERVAL, "0")
        assert rs.configure_qt_presentation(_L()) == 0
        assert QSurfaceFormat.defaultFormat().swapInterval() == 0
        monkeypatch.setenv(rs.ENV_SWAP_INTERVAL, "default")
        f = QSurfaceFormat.defaultFormat(); f.setSwapInterval(1); QSurfaceFormat.setDefaultFormat(f)
        assert rs.configure_qt_presentation(_L()) is None
        assert QSurfaceFormat.defaultFormat().swapInterval() == 1, "不動 Qt 預設"
        assert not any(m.startswith("W:") for m in logs)
    finally:
        QSurfaceFormat.setDefaultFormat(saved)


def test_main_configures_presentation_before_creating_the_application():
    src = (REPO / "main.py").read_text(encoding="utf-8")
    body = src[src.index("def run_gui_mode"):]
    i_conf = body.index("configure_qt_presentation(logger)")
    i_app = body.index("app = QApplication(sys.argv)")
    assert i_conf < i_app, "QSurfaceFormat 預設格式必須在 QApplication 建立前設定"


# ═══════════════════════════════════════════════════════════════════════
#  追機鏡頭偏航阻尼（main.html）
# ═══════════════════════════════════════════════════════════════════════
def _func(name: str) -> str:
    m = re.search(r"function " + re.escape(name) + r"\(.*?\n\}\n", HTML.replace("\r\n", "\n"), flags=re.S)
    assert m, f"main.html 裡找不到 {name}"
    return m.group(0)


def test_chase_camera_uses_the_damped_yaw_and_resets_it_on_every_restart():
    chase = _func("setChaseCamera")
    assert "_chaseYaw = _chaseSmoothYaw(_chaseYaw, hdgRad, dtSec, _chaseConfig.smoothing * 2.0);" in chase
    assert "new Cesium.HeadingPitchRoll(_chaseYaw, 0, 0)" in chase and "HeadingPitchRoll(hdgRad" not in chase
    head = chase[:chase.index("_chasePreRender = function")]
    assert "_chaseYaw = null;" in head and "_chaseLastMs = 0;" in head, "換機／關閉時要從機頭重新對齊"
    reset = _func("resetChaseView")
    assert "_chaseYaw = null;" in reset and "_chaseLastMs = 0;" in reset
    assert "_chaseLastMs = nowMs;" in chase and "performance.now()" in chase


def _py_smooth(prev, target, dt, tau):
    """_chaseSmoothYaw 的 Python 對照實作（用來比對瀏覽器跑出來的值）。"""
    if prev is None or not math.isfinite(prev) or not (tau > 0):
        return target
    if not math.isfinite(target):
        return prev
    d = math.atan2(math.sin(target - prev), math.cos(target - prev))
    if abs(d) < 1e-5:
        return target
    a = 1 - math.exp(-max(0.0, min(dt, 0.5)) / tau)
    y = prev + d * a
    return math.atan2(math.sin(y), math.cos(y))


@pytest.fixture(scope="module")
def jspage(_pw):
    """真的 JS 引擎（無頭 Chrome）跑 main.html 裡抽出來的 _chaseSmoothYaw。safe runner 要設 SAFE_ALLOW_BROWSER=1。"""
    try:
        b = _launch(_pw, [])
    except Exception as e:                                   # pragma: no cover
        pytest.skip(f"瀏覽器起不來：{e}")
    pg = b.new_page()
    snap = re.search(r"const CHASE_YAW_SNAP_RAD = [^;]+;", HTML)
    assert snap, "main.html 裡找不到 CHASE_YAW_SNAP_RAD"
    pg.set_content("<html><body><script>" + snap.group(0) + "\n" + _func("_chaseSmoothYaw") + "</script></body></html>")
    yield pg
    b.close()


def test_chase_yaw_smoothing_values_in_a_real_js_engine(jspage):
    ev = lambda *a: jspage.evaluate("([p,t,d,u]) => _chaseSmoothYaw(p,t,d,u)", list(a))   # noqa: E731
    assert ev(None, 1.0, 0.016, 0.36) == 1.0, "首幀直接對齊"
    assert ev(0.5, 1.0, 0.016, 0) == 1.0, "τ=0 ⇒ 剛性鎖機頭（舊行為）"
    # 最短角：179° → -179° 走 +2°（不是 -358°），結果包回 (-180°, 180°]；dt 夾在 0.5 s
    want = 179 + 2 * (1 - math.exp(-0.5 / 0.36)) - 360                  # ≈ -179.4987°
    y = ev(math.radians(179), math.radians(-179), 10.0, 0.36)
    assert abs(math.degrees(y) - want) < 1e-9, (math.degrees(y), want)
    y = ev(math.radians(179), math.radians(-179), 0.05, 0.36)
    want = 179 + 2 * (1 - math.exp(-0.05 / 0.36))                     # ≈ 179.259°，還沒跨過 180
    assert abs(math.degrees(y) - want) < 1e-9, (math.degrees(y), want)
    # 差距 < 1e-5 rad 直接對齊（停止改相機）；剛好大於就照指數走
    assert ev(0.3, 0.3 + 5e-6, 0.016, 0.36) == 0.3 + 5e-6
    assert ev(0.3, 0.3 + 2e-5, 0.016, 0.36) < 0.3 + 2e-5
    for args in [(0.3, 2.9, 0.016, 0.36), (-3.0, 3.0, 0.033, 0.2), (1.0, 1.2, 2.0, 0.36), (0.0, float("nan"), 0.016, 0.36)]:
        got = ev(*args)
        want = _py_smooth(*args)
        assert abs(got - want) < 1e-9, (args, got, want)
        assert -math.pi <= got <= math.pi
    assert ev(0.0, 1.0, -1.0, 0.36) == 0.0, "負 dt 當 0"


def test_chase_yaw_is_frame_rate_independent_and_lags_a_steady_turn_by_rate_times_tau(jspage):
    run = """([fps, secs, tau, rate]) => { let y = null, t = 0; const dt = 1 / fps;
      for (let i = 0; i <= fps * secs; i++) { t = i * dt; y = _chaseSmoothYaw(y, Math.atan2(Math.sin(rate*t), Math.cos(rate*t)), i ? dt : 0, tau); }
      const tgt = Math.atan2(Math.sin(rate*t), Math.cos(rate*t)); return Math.atan2(Math.sin(tgt - y), Math.cos(tgt - y)); }"""
    rate = math.radians(17.0)                                 # 驗收探針的急轉 ≈ 17°/s
    lag60 = jspage.evaluate(run, [60, 8, 0.36, rate])
    lag30 = jspage.evaluate(run, [30, 8, 0.36, rate])
    lag144 = jspage.evaluate(run, [144, 8, 0.36, rate])
    assert abs(math.degrees(lag60) - 17 * 0.36) < 0.4, math.degrees(lag60)
    assert abs(lag60 - lag30) < math.radians(0.3) and abs(lag60 - lag144) < math.radians(0.2), \
        (math.degrees(lag30), math.degrees(lag60), math.degrees(lag144))
    # 步進：機頭瞬間轉 90°，τ 秒後追上 63%、3τ 後 95%，不會過衝
    step = """([fps, tau]) => { let y = _chaseSmoothYaw(null, 0, 0, tau); const out = []; const dt = 1 / fps;
      for (let i = 1; i <= fps * 3 * tau + 1e-9; i++) { y = _chaseSmoothYaw(y, Math.PI / 2, dt, tau); out.push(y); } return out; }"""
    ys = jspage.evaluate(step, [60, 0.36])
    assert all(b >= a for a, b in zip(ys, ys[1:])) and max(ys) <= math.pi / 2 + 1e-12
    assert abs(ys[round(60 * 0.36) - 1] / (math.pi / 2) - 0.632) < 0.02
    assert abs(ys[-1] / (math.pi / 2) - 0.950) < 0.02


# ═══════════════════════════════════════════════════════════════════════
#  追機鏡頭：真的 Cesium（本機 assets/cesium；沒有就 skip；safe runner 要設 SAFE_ALLOW_BROWSER=1）
#  （2026-10-17 審查：只靠字串比對的話「_chaseYaw = hdgRad」「重設搬進 if」「resetChaseView 的重設變註解」都活得下來）
# ═══════════════════════════════════════════════════════════════════════
from tests.test_cesium_uav_batch import page3d, LAT0 as _B_LAT0, LON0 as _B_LON0  # noqa: E402,F401

_CHASE_JS = """
window.CH_ = {
  fly: null, samples: [], renders: 0, sid: 21,
  // 繞圈：R=60 m、V=18 m/s ⇒ 17.2°/s 右轉；每 50 ms 一筆（20 Hz），pt/at = 現在
  start(lat0, lon0) {
    const mLat = 111319.49, mLon = mLat * Math.cos(lat0 * Math.PI / 180), R = 60, V = 18, W = V / R, t0 = Date.now();
    const push = () => { const now = Date.now(), th = W * (now - t0) / 1000;
      const hdg = ((th * 180 / Math.PI) + 90) % 360;
      updateUAVPositions([{sysid: CH_.sid, lat: lat0 + R * Math.cos(th) / mLat, lon: lon0 + R * Math.sin(th) / mLon, alt: 100,
        hdg: hdg, crs: hdg, spd: V, mode: 'AUTO', armed: true, vt: 'VTOL', pitch: 0, roll: 0, pt: now, at: now}]); };
    push(); CH_.fly = setInterval(push, 50);
  },
  stop() { if (CH_.fly) clearInterval(CH_.fly); CH_.fly = null; },
  wrap(d) { return Math.atan2(Math.sin(d), Math.cos(d)); },
  sample() {
    const st = _UAV_STATE[CH_.sid]; if (!st) return null;
    const ac = _uavSmoothPos(st), hdg = _uavSmoothHpr(st).heading;
    return {t: performance.now(), lagDeg: Cesium.Math.toDegrees(CH_.wrap(hdg - viewer.camera.heading)),
            range: Cesium.Cartesian3.distance(viewer.camera.positionWC, ac),
            dh: Cesium.Cartographic.fromCartesian(viewer.camera.positionWC).height - Cesium.Cartographic.fromCartesian(ac).height};
  },
  hook() { if (CH_._h) return; CH_._h = () => { CH_.renders++; const s = CH_.sample(); if (s) CH_.samples.push(s); };
           viewer.scene.postRender.addEventListener(CH_._h); },
  window(ms) { const t1 = performance.now() - ms; return CH_.samples.filter(s => s.t >= t1); },
  mean(a, k) { return a.reduce((s, x) => s + x[k], 0) / Math.max(1, a.length); },
  pump(ms) { return new Promise(r => { const until = performance.now() + ms;
    (function f() { _requestRender(); if (performance.now() < until) requestAnimationFrame(f); else r(); })(); }); },
  sleep(ms) { return new Promise(r => setTimeout(r, ms)); },
};
"""


@pytest.fixture
def chase(page3d):
    page3d.evaluate("clearUAV(); _RENDER_IDLE_MS = 2000;")
    page3d.add_script_tag(content=_CHASE_JS)
    page3d.evaluate(f"CH_.hook(); CH_.start({_B_LAT0}, {_B_LON0});")
    page3d.wait_for_function("!!_UAV_STATE[CH_.sid] && !!uavEntities[CH_.sid]", timeout=10000)
    yield page3d
    page3d.evaluate("CH_.stop(); setChaseCamera(0); setChaseCameraParams(80, 20, -10, 0.18); clearUAV(); _RENDER_IDLE_MS = 2000;")
    assert not page3d._errs, page3d._errs


def test_real_chase_lags_a_steady_turn_by_rate_times_tau_and_holds_range(chase):
    r = chase.evaluate("""async () => { setChaseCamera(CH_.sid); await CH_.pump(4000);
      const w = CH_.window(1500);
      return {n: w.length, lag: CH_.mean(w, 'lagDeg'), rmin: Math.min(...w.map(s => s.range)), rmax: Math.max(...w.map(s => s.range)),
              dh: CH_.mean(w, 'dh')}; }""")
    assert r["n"] > 30, r
    assert 4.5 < r["lag"] < 8.0, f"17.2°/s × τ 0.36 s ≈ 6.2°（剛性鎖機頭 ≈ 0°）：{r}"
    want = math.hypot(80, 80 * math.tan(math.radians(10)))          # 俯角 -10° ⇒ 高度 = 80·tan10°
    assert want - 6 < r["rmin"] and r["rmax"] < want + 6, (want, r)
    assert abs(r["dh"] - 80 * math.tan(math.radians(10))) < 2.0, f"俯角 -10° ⇒ 高 14.1 m（以前忽略俯角、用 height 20 m）：{r}"


def test_real_chase_rigid_when_smoothing_is_zero(chase):
    r = chase.evaluate("""async () => { setChaseCameraParams(80, 20, -10, 0); setChaseCamera(CH_.sid); await CH_.pump(2500);
      const w = CH_.window(1000); return {n: w.length, lag: CH_.mean(w, 'lagDeg')}; }""")
    assert r["n"] > 20 and abs(r["lag"]) < 1.0, r


@pytest.mark.parametrize("how", ["setChaseCamera", "resetChaseView"])
def test_real_chase_realigns_with_the_nose_on_restart_and_reset(chase, how):
    r = chase.evaluate(f"""async () => {{ setChaseCamera(CH_.sid); await CH_.pump(3000);
      const before = CH_.mean(CH_.window(500), 'lagDeg');
      {'setChaseCamera(CH_.sid);' if how == 'setChaseCamera' else 'resetChaseView();'}
      const n0 = CH_.samples.length; await CH_.pump(120);
      const first = CH_.samples.slice(n0, n0 + 2);
      return {{before, first: first.map(s => s.lagDeg)}}; }}""")
    assert r["before"] > 4.0, r
    assert r["first"] and abs(r["first"][0]) < 1.0, f"{how} 之後第一幀要直接對齊機頭：{r}"


@pytest.mark.parametrize("call", ["flyToPos({lat}, {lon}, 0, 600)", "setViewPos({lat}, {lon}, 0, 600)"])
def test_real_chase_survives_a_camera_jump_instead_of_parking_6000_km_away(chase, call):
    js_call = call.format(lat=_B_LAT0, lon=_B_LON0)
    r = chase.evaluate(f"""async () => {{ setChaseCamera(CH_.sid); await CH_.pump(1500);
      const n0 = CH_.samples.length; {js_call}; await CH_.pump(4000);
      const after = CH_.samples.slice(n0); return {{n: after.length, rmax: Math.max(...after.map(s => s.range)),
        rlast: after[after.length - 1].range}}; }}""")
    assert r["n"] > 30, r
    assert r["rmax"] < 300 and r["rlast"] < 100, f"舊版：鏡頭跑到 ~6,375 km 外而且回不來：{r}"


def test_real_chase_stops_touching_the_camera_once_the_aircraft_stops(chase):
    """閒置時相機要完全不動，Cesium 的依需求算圖才停得下來（審查實測：舊版剛性鎖每幀浮點抖一個 ULP、
    阻尼版指數逼近永不收斂 ⇒ 每幀都算「相機動了」）。這裡直接比相機矩陣與位置逐位元不變、且沒再呼叫 lookAtTransform。
    （不量算圖次數：測試環境沒網路、圖磚一直重試，Cesium 自己每幀要求算圖，與追機無關。）"""
    r = chase.evaluate("""async () => { setChaseCamera(CH_.sid); await CH_.pump(2500); CH_.stop();
      await CH_.pump(5000);                           // 阻尼收斂（snap ≈ 3 s）
      let calls = 0; const orig = viewer.camera.lookAtTransform;
      viewer.camera.lookAtTransform = function(a, b) { calls++; return orig.call(viewer.camera, a, b); };
      const snap = () => Array.from(Cesium.Matrix4.toArray(viewer.camera.transform)).concat(
        [viewer.camera.position.x, viewer.camera.position.y, viewer.camera.position.z]);
      let prev = snap(), frames = 0, changed = 0;
      const h = () => { frames++; const cur = snap(); if (cur.some((v, k) => v !== prev[k])) changed++; prev = cur; };
      viewer.scene.postRender.addEventListener(h);
      await CH_.pump(1500);
      viewer.scene.postRender.removeEventListener(h); viewer.camera.lookAtTransform = orig;
      const last = CH_.samples[CH_.samples.length - 1];
      return {frames, changed, calls, range: last.range, lag: last.lagDeg}; }""")
    assert r["frames"] > 30, r
    assert r["changed"] == 0 and r["calls"] == 0, f"飛機停了、鏡頭也收斂了，就不該再動相機：{r}"
    assert abs(r["lag"]) < 0.01 and 70 < r["range"] < 95, r
