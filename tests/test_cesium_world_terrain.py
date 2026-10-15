"""Cesium World Terrain 取樣器與規劃器接線（2026-10-09 使用者：taiwan_dem.tif 解析度太低，改用 World Terrain）。

分三層，全部**不連網**（假 HTTP session / 假取樣器），真連線的對照放在最後一個 live 測試（預設跳過）：

1. 圖磚幾何與 quantized-mesh 編解碼（用規格自己寫一個編碼器來回驗）
2. 取樣器：優先層級、404 退回上一層、快取、授權標頭不洩漏 token、錯誤不靜默、**不寫任何檔案**
3. 規劃器接線（真的 HARVEST 計畫，假地形）：地形真的進了規劃器的 DEM、notes、NO_GO 提示、失敗不靜默、開關
"""
from __future__ import annotations

import ast
import math
import os
import pathlib
import re
import struct
import types

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import core.terrain.cesium_world_terrain as cwt  # noqa: E402
from core.terrain import CesiumWorldTerrain, WorldTerrainError  # noqa: E402

USER_TOKEN = "user.token.secret"
EP_TOKEN = "endpoint.token.other"


# ═══════════════════════════ 1. 幾何與編解碼 ═══════════════════════════
def test_tile_addressing_matches_the_real_tile_ids():
    """這組座標在真實服務上取的圖磚就是 13/13708/5237（2026-10-09 實測的網路請求）。"""
    assert cwt.tile_xy(25.071956, 121.211472, 13) == (13708, 5237)


def test_level0_has_two_tiles_and_edges_are_clamped():
    assert cwt.tile_xy(10.0, -10.0, 0) == (0, 0)
    assert cwt.tile_xy(10.0, 10.0, 0) == (1, 0)
    n = 1 << 5
    assert cwt.tile_xy(90.0, 180.0, 5) == (2 * n - 1, n - 1)         # 極值夾在邊界內
    assert cwt.tile_xy(-90.0, -180.0, 5) == (0, 0)


@pytest.mark.parametrize("lat,lon,level", [(25.07, 121.21, 13), (-33.9, 151.2, 9), (0.0, 0.0, 3)])
def test_tile_bounds_contain_the_point(lat, lon, level):
    x, y = cwt.tile_xy(lat, lon, level)
    w, s, e, n = cwt.tile_bounds(level, x, y)
    assert w <= lon < e + 1e-12 and s <= lat < n + 1e-12
    assert e - w == pytest.approx(180.0 / (1 << level))


def _zz(vals):
    out, prev = [], 0
    for x in vals:
        d = int(x) - prev
        prev = int(x)
        out.append((d << 1) if d >= 0 else (((-d) << 1) - 1))
    return out


def encode_qm(u, v, hq, tris, min_h, max_h):
    """規格寫的 quantized-mesh 編碼器（測試用）。頂點須按首次被三角形參照的順序排列。"""
    vc = len(u)
    b = struct.pack("<3d2f4d3d", 0, 0, 0, min_h, max_h, 0, 0, 0, 0, 0, 0, 0)
    b += struct.pack("<I", vc)
    for arr in (u, v, hq):
        b += struct.pack(f"<{vc}H", *_zz(arr))
    wide = vc > 65536
    align = 4 if wide else 2
    b += b"\0" * ((-len(b)) % align)
    flat, hw, codes = [i for t in tris for i in t], 0, []
    for i in flat:
        codes.append(hw - i)
        if i == hw:
            hw += 1
    b += struct.pack("<I", len(tris))
    b += struct.pack(f"<{len(codes)}{'I' if wide else 'H'}", *codes)
    b += struct.pack("<4I", 0, 0, 0, 0)                          # 邊界索引（解碼器不讀）
    return b


def plane_mesh(k=8, h0=10.0, du=50.0, dv=20.0):
    """(k+1)² 個頂點的平面網格：h = h0 + du·fu + dv·fv；頂點依首次使用順序重排。"""
    pts = [(i, j) for j in range(k + 1) for i in range(k + 1)]
    tri = []
    for j in range(k):
        for i in range(k):
            a, b, c, d = (j * (k + 1) + i, j * (k + 1) + i + 1, (j + 1) * (k + 1) + i, (j + 1) * (k + 1) + i + 1)
            tri += [(a, b, c), (b, d, c)]
    order, seen = [], {}
    for t in tri:
        for p in t:
            if p not in seen:
                seen[p] = len(order)
                order.append(p)
    tri = [tuple(seen[p] for p in t) for t in tri]
    u = [round(pts[p][0] / k * 32767) for p in order]
    v = [round(pts[p][1] / k * 32767) for p in order]
    h = [h0 + du * (pts[p][0] / k) + dv * (pts[p][1] / k) for p in order]
    mn, mx = min(h), max(h)
    hq = [round((x - mn) / (mx - mn) * 32767) for x in h]
    return encode_qm(u, v, hq, tri, mn, mx), (u, v, h, tri)


def test_decode_roundtrip_recovers_vertices_and_triangles():
    data, (u, v, h, tri) = plane_mesh()
    m = cwt.decode_quantized_mesh(data)
    assert m.u.tolist() == u and m.v.tolist() == v
    assert np.allclose(m.h, h, atol=0.01)                        # 16 位元量化誤差 ≤ 70 m/32767
    assert m.tri.tolist() == [list(t) for t in tri]


def test_decode_handles_the_32bit_index_layout():
    """頂點數 > 65536 ⇒ 索引改 32 位元、對齊 4 bytes。"""
    vc = 65538
    u = [0, 32767, 0, 32767] + [100] * (vc - 4)
    v = [0, 0, 32767, 32767] + [100] * (vc - 4)
    hq = [0, 10, 20, 32767] + [5] * (vc - 4)
    data = encode_qm(u, v, hq, [(0, 1, 2), (1, 3, 2)], 0.0, 100.0)
    m = cwt.decode_quantized_mesh(data)
    assert len(m.h) == vc and m.tri.tolist() == [[0, 1, 2], [1, 3, 2]]
    assert m.h[3] == pytest.approx(100.0)


@pytest.mark.parametrize("bad", [b"", b"\0" * 50, b"\0" * 92])
def test_garbage_tiles_raise_instead_of_returning_nonsense(bad):
    with pytest.raises(WorldTerrainError):
        cwt.decode_quantized_mesh(bad)


def test_rasterised_plane_is_exact_at_the_grid_points():
    data, _ = plane_mesh(k=8, h0=10.0, du=50.0, dv=20.0)
    r = cwt.rasterize_mesh(cwt.decode_quantized_mesh(data))
    n = cwt.RASTER_N
    assert r.shape == (n, n) and np.isfinite(r).all()
    i, j = np.meshgrid(np.arange(n), np.arange(n))
    expect = 10.0 + 50.0 * i / (n - 1) + 20.0 * j / (n - 1)
    assert np.abs(r - expect).max() < 0.02


def test_fast_rasteriser_equals_the_reference_on_a_random_mesh():
    from scipy.spatial import Delaunay
    rng = np.random.default_rng(7)
    pts = np.vstack([[0, 0], [1, 0], [0, 1], [1, 1], rng.random((300, 2))])
    tri = Delaunay(pts).simplices
    order, seen = [], {}
    for t in tri:
        for p in t:
            if int(p) not in seen:
                seen[int(p)] = len(order)
                order.append(int(p))
    tris = [tuple(seen[int(p)] for p in t) for t in tri]
    u = [round(pts[p][0] * 32767) for p in order]
    v = [round(pts[p][1] * 32767) for p in order]
    h = rng.random(len(order)) * 200
    hq = [round(x / 200 * 32767) for x in h]
    m = cwt.decode_quantized_mesh(encode_qm(u, v, hq, tris, 0.0, 200.0))
    a, ref = cwt.rasterize_mesh(m), cwt.rasterize_mesh_reference(m)
    both = np.isfinite(a) & np.isfinite(ref)
    assert both.sum() > 0.97 * a.size
    assert np.abs(a[both] - ref[both]).max() < 0.05


# ═══════════════════════════ 2. 取樣器（假 HTTP）═══════════════════════════
class _Resp:
    def __init__(self, status=200, content=b"", js=None):
        self.status_code, self.content, self._js = status, content, js

    def json(self):
        return self._js


class FakeSession:
    """假的 ion：endpoint / layer.json / 圖磚。tiles = {(z,x,y): bytes}，沒有的回 404。"""

    def __init__(self, tiles, levels=5, *, endpoint_status=200, tile_status=None, boom=None):
        self.tiles, self.levels = tiles, levels
        self.endpoint_status, self.tile_status, self.boom = endpoint_status, tile_status or {}, boom
        self.calls = []

    def get(self, url, headers=None, timeout=None):
        self.calls.append((url, dict(headers or {})))
        if self.boom:
            raise self.boom
        if url.endswith("/v1/assets/1/endpoint"):
            if self.endpoint_status != 200:
                return _Resp(self.endpoint_status)
            return _Resp(200, js={"url": "https://assets.test/depot/1/CWT/v1.2/", "accessToken": EP_TOKEN,
                                  "attributions": [{"html": "<a href='x'>Data © Foo Survey</a>"},
                                                   {"html": "<b>Upgrade for commercial use.</b>"}]})
        if url.endswith("/layer.json"):
            av = [[{"startX": 0, "endX": (2 << z) - 1, "startY": 0, "endY": (1 << z) - 1}] for z in range(self.levels)]
            return _Resp(200, js={"scheme": "tms", "projection": "EPSG:4326", "version": "1.2.0",
                                  "tiles": ["{z}/{x}/{y}.terrain?v={version}"], "available": av})
        m = re.search(r"/(\d+)/(\d+)/(\d+)\.terrain", url)
        key = tuple(int(g) for g in m.groups())
        if key in self.tile_status:
            return _Resp(self.tile_status[key])
        if key in self.tiles:
            return _Resp(200, self.tiles[key])
        return _Resp(404)


LAT, LON = 25.071956, 121.211472
L4 = cwt.tile_xy(LAT, LON, 4)


def _frac(level, lat=LAT, lon=LON):
    x, y = cwt.tile_xy(lat, lon, level)
    w, s, e, n = cwt.tile_bounds(level, x, y)
    return (lon - w) / (e - w), (lat - s) / (n - s)


def _expected(h0, du, dv, level):
    fu, fv = _frac(level)
    return h0 + du * fu + dv * fv


def test_samples_the_deepest_available_level_and_matches_the_plane():
    d4, _ = plane_mesh(h0=100.0, du=40.0, dv=10.0)
    s = FakeSession({(4, *L4): d4})
    w = CesiumWorldTerrain(USER_TOKEN, session=s)
    assert w.max_level_at(LAT, LON) == 4
    assert w.get_elevation(LAT, LON) == pytest.approx(_expected(100.0, 40.0, 10.0, 4), abs=0.1)


def test_falls_back_to_the_parent_level_when_the_deepest_tile_is_missing():
    d3, _ = plane_mesh(h0=200.0, du=30.0, dv=0.0)
    L3 = cwt.tile_xy(LAT, LON, 3)
    s = FakeSession({(3, *L3): d3})                  # level 4 的圖磚 404
    w = CesiumWorldTerrain(USER_TOKEN, session=s)
    assert w.get_elevation(LAT, LON) == pytest.approx(_expected(200.0, 30.0, 0.0, 3), abs=0.1)
    assert w.stats["not_found"] == 1 and w.failures == []


def test_level_lookup_distinguishes_neighbouring_tiles():
    """level 4 只在「這一列」有資料：鄰列的點要退回 level 3；記憶不可把兩者混為一談（兩種查詢順序都測）。"""
    class OneRow(FakeSession):
        def get(self, url, headers=None, timeout=None):
            r = super().get(url, headers, timeout)
            if url.endswith("/layer.json"):
                av = r._js["available"]
                x4, y4 = L4
                av[4] = [{"startX": x4, "endX": x4, "startY": y4, "endY": y4}]
            return r

    for first, second in (((LAT, LON), (LAT + 12.0, LON)), ((LAT + 12.0, LON), (LAT, LON))):
        w = CesiumWorldTerrain(USER_TOKEN, session=OneRow({}))
        assert w.max_level_at(*first) == (4 if first == (LAT, LON) else 3)
        assert w.max_level_at(*second) == (4 if second == (LAT, LON) else 3)


def test_repeated_samples_come_from_memory():
    d4, _ = plane_mesh()
    s = FakeSession({(4, *L4): d4})
    w = CesiumWorldTerrain(USER_TOKEN, session=s)
    w.get_elevation(LAT, LON)
    n = len(s.calls)
    for _ in range(50):
        w.get_elevation(LAT + 1e-5, LON + 1e-5)
    assert len(s.calls) == n, "第二次起不該再連線"
    assert w.stats["cache_hits"] >= 50


def test_the_user_token_only_goes_to_the_endpoint_and_never_into_a_url():
    d4, _ = plane_mesh()
    s = FakeSession({(4, *L4): d4})
    CesiumWorldTerrain(USER_TOKEN, session=s).get_elevation(LAT, LON)
    for url, hdr in s.calls:
        assert USER_TOKEN not in url and EP_TOKEN not in url, f"token 出現在網址：{url}"
    first = s.calls[0]
    assert first[1].get("Authorization") == f"Bearer {USER_TOKEN}"
    for url, hdr in s.calls[1:]:
        assert hdr.get("Authorization") == f"Bearer {EP_TOKEN}", "圖磚請求要用端點給的短效 token，不是使用者的 token"


def test_attribution_is_exposed_without_html():
    d4, _ = plane_mesh()
    w = CesiumWorldTerrain(USER_TOKEN, session=FakeSession({(4, *L4): d4}))
    w.get_elevation(LAT, LON)
    assert "Data © Foo Survey" in w.attribution_text and "<" not in w.attribution_text
    assert "Upgrade for commercial use" in w.attribution_text


def test_rejected_token_raises_a_clear_error_that_does_not_leak_the_token():
    w = CesiumWorldTerrain(USER_TOKEN, session=FakeSession({}, endpoint_status=401))
    with pytest.raises(WorldTerrainError) as e:
        w.prefetch((25.0, 121.0, 25.01, 121.01))
    assert "401" in str(e.value) and "拒絕" in str(e.value) and USER_TOKEN not in str(e.value)


def test_network_failure_raises_without_the_exception_text_that_may_carry_a_url():
    w = CesiumWorldTerrain(USER_TOKEN, session=FakeSession({}, boom=ConnectionError(f"https://x/?access_token={USER_TOKEN}")))
    with pytest.raises(WorldTerrainError) as e:
        w.prefetch((25.0, 121.0, 25.01, 121.01))
    assert "連線失敗" in str(e.value) and USER_TOKEN not in str(e.value)


@pytest.mark.parametrize("tok", ["", "   ", None])
def test_no_token_is_refused_up_front(tok):
    with pytest.raises(WorldTerrainError):
        CesiumWorldTerrain(tok)


def test_prefetch_loads_every_tile_and_fails_loudly_on_a_server_error():
    d4, _ = plane_mesh()
    b = (LAT - 0.001, LON - 0.001, LAT + 0.001, LON + 0.001)
    tiles = {(4, x, y): d4 for x in range(0, 32) for y in range(0, 16)}
    w = CesiumWorldTerrain(USER_TOKEN, session=FakeSession(tiles))
    st = w.prefetch(b)
    assert st["level"] == 4 and st["tiles"] >= 1 and st["fetched"] == st["tiles"]
    # 同樣範圍、某片 500 ⇒ 直接丟錯，不是靜默少一片
    bad = CesiumWorldTerrain(USER_TOKEN, session=FakeSession(tiles, tile_status={(4, *L4): 500}))
    with pytest.raises(WorldTerrainError) as e:
        bad.prefetch(b)
    assert "500" in str(e.value)


def test_prefetch_refuses_an_absurdly_large_region():
    w = CesiumWorldTerrain(USER_TOKEN, session=FakeSession({}, levels=12))
    with pytest.raises(WorldTerrainError) as e:
        w.prefetch((20.0, 115.0, 30.0, 125.0), max_tiles=50)
    assert "太大" in str(e.value)


def test_sampling_failure_outside_the_prefetch_is_recorded_not_swallowed():
    w = CesiumWorldTerrain(USER_TOKEN, session=FakeSession({}, tile_status={(4, *L4): 500}))
    assert w.get_elevation(LAT, LON) is None
    assert w.failures and "500" in w.failures[0]


def test_lru_is_bounded():
    d4, _ = plane_mesh()
    tiles = {(4, x, y): d4 for x in range(32) for y in range(16)}
    w = CesiumWorldTerrain(USER_TOKEN, session=FakeSession(tiles), max_tiles=3)
    for k in range(10):
        w.get_elevation(LAT, 100.0 + k * 3.0)
    assert len(w._tiles) <= 3


def test_nothing_is_ever_written_to_disk(tmp_path, monkeypatch):
    """授權：条款禁止儲存供離線使用 ⇒ 取樣器不得寫任何檔案（行為 + 原始碼雙重檢查）。"""
    monkeypatch.chdir(tmp_path)
    d4, _ = plane_mesh()
    w = CesiumWorldTerrain(USER_TOKEN, session=FakeSession({(4, *L4): d4}))
    w.prefetch((LAT - 0.001, LON - 0.001, LAT + 0.001, LON + 0.001))
    w.get_elevation(LAT, LON)
    assert list(tmp_path.iterdir()) == []
    tree = ast.parse(pathlib.Path(cwt.__file__).read_text(encoding="utf-8"))
    bad = []
    for n in ast.walk(tree):
        if isinstance(n, ast.Call):
            f = n.func
            name = f.id if isinstance(f, ast.Name) else (f.attr if isinstance(f, ast.Attribute) else "")
            if name in {"open", "write_bytes", "write_text", "save", "savez", "savez_compressed", "tofile",
                        "dump", "dumps_to_file", "to_file", "NamedTemporaryFile", "mkdtemp"}:
                bad.append(f"{name}@{n.lineno}")
    assert not bad, f"取樣器原始碼出現寫檔呼叫：{bad}"


def test_relief_sampling_density_follows_the_region_size_not_a_fixed_grid():
    """固定 40×40 在大區域會漏掉極值 ⇒ 起伏被低估 ⇒ NO_GO 提示給出「不夠」的 h_ceil（實例：66 m 量成 62 m）。"""
    calls = []
    w = CesiumWorldTerrain(USER_TOKEN, session=FakeSession({}))
    w.get_elevation = lambda la, lo: (calls.append((la, lo)) or 10.0 + la)
    w.relief_m((25.0, 121.0, 25.02, 121.02))                    # ~2.2 km × 2.0 km
    las = sorted({c[0] for c in calls})
    assert len(las) > 60, "2 km 的區域用 30 m 間距應有 60+ 列，不是固定 40"
    assert max(b - a for a, b in zip(las, las[1:])) * 111320.0 < 35.0
    calls.clear()
    w.relief_m((20.0, 115.0, 30.0, 125.0))                      # 1000 km 級：每軸封頂 300，不能爆
    assert len(calls) <= 300 * 300
    calls.clear()
    w.relief_m((25.0, 121.0, 25.02, 121.02), n=7)               # 明確指定 n 仍可固定格數
    assert len(calls) == 49


def test_relief_finds_a_narrow_peak_a_coarse_grid_would_miss():
    w = CesiumWorldTerrain(USER_TOKEN, session=FakeSession({}))
    lat0, lon0 = 25.0100, 121.0100                              # 區域正中央一座 ~60 m 寬的尖峰
    w.get_elevation = lambda la, lo: 90.0 if (abs(la - lat0) < 0.0003 and abs(lo - lon0) < 0.0003) else 20.0
    lo, hi = w.relief_m((25.0, 121.0, 25.02, 121.02))
    assert (lo, hi) == (20.0, 90.0)
    lo, hi = w.relief_m((25.0, 121.0, 25.02, 121.02), n=40)    # 對照：固定 40×40 的格距 ~55 m，會錯過或勉強擦到
    assert hi - lo <= 70.0


# ═══════════════════════════ 3. 規劃器接線 ═══════════════════════════
from mission.swarm_coordinator import SwarmCoordinator  # noqa: E402
from utils.math_utils import spawn_positions  # noqa: E402

hv = pytest.importorskip("hv_dccpp.integration.aeroplan_adapter")

HOME = (23.95, 120.90)
_M_LAT = 111132.0
_M_LON = 111412.0 * math.cos(math.radians(HOME[0]))


def _ll(e, n):
    return (HOME[0] + n / _M_LAT, HOME[1] + e / _M_LON)


AREAS = [{"area_id": 1, "priority": 1.0, "scan_angle_deg": 0.0,
          "polygon": [_ll(3000, -450), _ll(4600, -450), _ll(4600, 450), _ll(3000, 450)]}]
CLASSES = {1: "A", 2: "B"}


class FakeWT:
    """假的 World Terrain：1% 的東西向坡。記下被呼叫的情況。"""
    instances: list = []
    relief = (0.0, 46.0)
    failures_to_report: list = []

    def __init__(self, token, **kw):
        self.token, self.kw = token, kw
        self.prefetched, self.calls = [], 0
        self.failures = list(FakeWT.failures_to_report)
        self.stats = {}
        self.attribution_text = "Fake data © Foo Survey. Upgrade for commercial use."
        FakeWT.instances.append(self)

    def prefetch(self, bounds, **kw):
        self.prefetched.append(bounds)
        return {"level": 13, "tiles": 4, "fetched": 4, "bytes": 4000}

    def relief_m(self, bounds, n=40):
        return FakeWT.relief

    def get_elevation(self, lat, lon):
        self.calls += 1
        return 30.0 + (lon - HOME[1]) * _M_LON * 0.01


@pytest.fixture
def fake_wt(monkeypatch):
    FakeWT.instances, FakeWT.relief, FakeWT.failures_to_report = [], (0.0, 46.0), []
    monkeypatch.setattr("core.terrain.CesiumWorldTerrain", FakeWT)
    monkeypatch.setattr("utils.cesium_ion.configured_ion_token", lambda *a, **k: ("fake.tok.en", "測試"))
    return FakeWT


def _coordinator(over=None, *, use_harvest=True):
    c = SwarmCoordinator()
    c.use_harvest = use_harvest
    c.hv_class_of = lambda d: CLASSES.get(int(d["drone_id"]), "A")
    c.hv_home = HOME
    c.harvest_params = _params(over)
    c.hv_on_report = lambda r: None
    return c


def _params(over=None):
    return {"min_separation_m": 100.0, "alt_offset_m": 50.0, "hv_waiver_max_agl_m": 200.0,
            "hv_world_terrain": True, **(over or {})}


def _run(c, *, terrain_func=None):
    pts = spawn_positions(HOME[0], HOME[1], 2, 0.0, 60.0)
    drones = [{"drone_id": i + 1, "name": f"UAV-{i + 1}", "position": pts[i], "speed": 20.0, "heading": 0.0,
               "turn_radius": 150.0, "vehicle_type": "vtol", "fov_width": 110.0} for i in range(2)]
    reports = []
    c.hv_on_report = reports.append
    res = c.plan_coverage_dccpp(areas=AREAS, drones=drones, coverage_width_m=110.0, overlap_rate=0.2,
                                coverage_altitude=120.0, terrain_func=terrain_func)
    return res, reports


def _plan(over=None, *, terrain_func=None, use_harvest=True):
    c = _coordinator(over, use_harvest=use_harvest)
    res, reports = _run(c, terrain_func=terrain_func)
    return c, res, reports


def _notes(c, reports):
    return [str(n) for r in reports for n in (r.get("validation_notes") or [])] + [str(n) for n in c.hv_coordinator.notes]


def test_world_terrain_really_reaches_the_planners_dem(fake_wt):
    c, res, reports = _plan()
    assert res["assembled_paths"], "前提：規劃成功"
    w = fake_wt.instances[0]
    assert w.token == "fake.tok.en" and len(w.prefetched) == 1 and w.calls > 1000
    dem = c.hv_coordinator.last_inputs.dem
    z = np.asarray(dem.z_msl)
    assert float(z.max() - z.min()) > 40.0, "規劃器的 DEM 是平的 ⇒ 地形沒有進去"
    # DEM 裡的值就是取樣器給的：東邊比西邊高（1% 坡）
    assert z[:, -1].mean() > z[:, 0].mean() + 40.0


def test_prefetch_covers_the_area_plus_the_dem_margin(fake_wt):
    _plan()
    (la0, lo0, la1, lo1), = fake_wt.instances[0].prefetched
    ring = [pt for a in AREAS for pt in a["polygon"]] + [HOME]
    assert la0 < min(p[0] for p in ring) - 0.9 * 3000 / 111320 and la1 > max(p[0] for p in ring) + 0.9 * 3000 / 111320
    assert lo0 < min(p[1] for p in ring) - 0.9 * 3000 / 111320 and lo1 > max(p[1] for p in ring) + 0.9 * 3000 / 111320


def test_notes_say_where_the_terrain_came_from_and_the_licence(fake_wt):
    c, _, reports = _plan()
    notes = _notes(c, reports)
    src = [n for n in notes if n.startswith("terrain_source(")]
    assert src, notes
    assert "Cesium World Terrain" in src[0] and "46 m" in src[0] and "非商用" in src[0]
    attr = [n for n in notes if n.startswith("terrain_attribution(")]
    assert attr and "Foo Survey" in attr[0], notes


def test_the_operator_will_actually_see_the_terrain_notes():
    """notes 不在 `_HV_LOUD_NOTES` 裡就只會進 log 檔；授權與「起伏吃掉高度帶」要讓操作員看見。"""
    from ui.main_window import MainWindow
    for prefix in ("terrain_source", "terrain_relief_exceeds_band", "landing_not_at_requested_site"):
        assert prefix in MainWindow._HV_LOUD_NOTES, prefix
    assert "terrain_attribution" not in MainWindow._HV_LOUD_NOTES      # 太長，只進 log


def test_unticked_means_the_provider_is_never_built(fake_wt):
    _plan({"hv_world_terrain": False})
    assert fake_wt.instances == []


def test_an_explicit_terrain_func_wins_over_world_terrain(fake_wt):
    _plan(terrain_func=lambda la, lo: 12.0)
    assert fake_wt.instances == []


def test_legacy_planner_ignores_the_flag(fake_wt):
    from contextlib import suppress
    with suppress(Exception):
        _plan(use_harvest=False)
    assert fake_wt.instances == []


def test_missing_token_stops_the_plan_with_an_actionable_message(fake_wt, monkeypatch):
    monkeypatch.setattr("utils.cesium_ion.configured_ion_token", lambda *a, **k: ("", ""))
    with pytest.raises(WorldTerrainError) as e:
        _plan()
    msg = str(e.value)
    assert "settings.json" in msg and "CESIUM_ION_TOKEN" in msg and "取消勾選" in msg
    assert fake_wt.instances == []


def test_sampling_failures_are_not_silently_planned_as_flat_ground(fake_wt):
    fake_wt.failures_to_report = ["圖磚 13/1/1 失敗（HTTP 500）"]
    with pytest.raises(WorldTerrainError) as e:
        _plan()
    assert "不可信" in str(e.value) and "500" in str(e.value)


def test_no_go_message_says_how_high_the_ceiling_must_be(fake_wt):
    """120 m 上限（不給豁免）+ 77 m 起伏 ⇒ NO_GO；訊息要算出需要的 h_ceil（= 最低作業高度 + 下限 + 起伏）。"""
    fake_wt.relief = (4.0, 81.0)
    with pytest.raises(hv.AeroPlanAdapterError) as e:
        _plan({"hv_waiver_max_agl_m": None, "hv_min_fw_agl_m": {"A": 100.0, "B": 100.0}, "hv_dh_band_min_m": 15.0})
    msg = str(e.value)
    assert "no_coverable_area" in msg and "提示" in msg
    assert "77 m" in msg and "h_ceil ≥ 192 m" in msg, msg
    assert "豁免" in msg


def test_hint_rounds_the_required_ceiling_up(fake_wt):
    """192.4 m 不能顯示成 192（使用者照著設會剛好不夠）⇒ 無條件進位到 193。"""
    fake_wt.relief = (4.0, 81.4)
    with pytest.raises(hv.AeroPlanAdapterError) as e:
        _plan({"hv_waiver_max_agl_m": None, "hv_min_fw_agl_m": {"A": 100.0, "B": 100.0}, "hv_dh_band_min_m": 15.0})
    assert "h_ceil ≥ 193 m" in str(e.value) and "h_ceil ≥ 192 m" not in str(e.value)


def test_hint_is_not_carried_over_to_the_next_plan_on_the_same_coordinator(fake_wt):
    """GUI 的 coordinator 是長壽的、連續規劃多次：上一次 NO_GO 的提示不可殘留到下一次。"""
    fake_wt.relief = (4.0, 81.0)
    tight = {"hv_waiver_max_agl_m": None, "hv_min_fw_agl_m": {"A": 100.0, "B": 100.0}, "hv_dh_band_min_m": 15.0}
    c = _coordinator(tight)
    with pytest.raises(hv.AeroPlanAdapterError):
        _run(c)
    assert c._terrain_hint, "前提：第一次 NO_GO 要留下提示"
    # 同一個 coordinator，改成給 200 m 豁免（需要 100+15+77 = 192 ≤ 200）⇒ 成功，而且不能還帶著上一次的提示
    c.harvest_params = _params({"hv_min_fw_agl_m": {"A": 100.0, "B": 100.0}, "hv_dh_band_min_m": 15.0})
    res, _ = _run(c)
    assert res["assembled_paths"] and c._terrain_hint == ""


# ═══════════════════════════ 面板 ═══════════════════════════
try:
    from ui.main_window import MainWindow  # noqa: F401  先載入 QtWebEngine
    from ui.widgets.vtol_recon_panel import VTOLReconPanel
    from PyQt6.QtWidgets import QApplication
    _QT_OK = True
except Exception as _e:                              # pragma: no cover
    _QT_OK = False
    _QT_ERR = _e


@pytest.fixture
def qt_app():
    return QApplication.instance() or QApplication([])


@pytest.mark.skipif(not _QT_OK, reason="Qt 匯入失敗")
def test_panel_checkbox_follows_the_token(qt_app, monkeypatch):
    monkeypatch.setattr("utils.cesium_ion.configured_ion_token", lambda *a, **k: ("a.b.c", "測試"))
    p = VTOLReconPanel()
    assert p.hv_world_terrain_chk.isEnabled() and p.hv_world_terrain_chk.isChecked()
    assert p.get_params()["hv_world_terrain"] is True
    p.hv_world_terrain_chk.setChecked(False)
    assert p.get_params()["hv_world_terrain"] is False

    monkeypatch.setattr("utils.cesium_ion.configured_ion_token", lambda *a, **k: ("", ""))
    q = VTOLReconPanel()
    assert not q.hv_world_terrain_chk.isEnabled() and not q.hv_world_terrain_chk.isChecked()
    q.hv_world_terrain_chk.setChecked(True)          # 就算被強制勾上，停用狀態下也不能送出
    assert q.get_params()["hv_world_terrain"] is False
    assert "免費" in q.hv_world_terrain_chk.toolTip() and "不存檔" in q.hv_world_terrain_chk.toolTip()


# ═══════════════════════════ 與真實服務對照（預設跳過）═══════════════════════════
def _live_token():
    if os.environ.get("AEROPLAN_LIVE_ION") != "1":
        return ""
    from utils.cesium_ion import configured_ion_token
    return configured_ion_token()[0]


@pytest.mark.skipif(not _live_token(), reason="設 AEROPLAN_LIVE_ION=1 且有 ion token 才連真實服務")
def test_live_matches_cesiumjs_sample_terrain_most_detailed():
    """CesiumJS `sampleTerrainMostDetailed` 在同一組點的結果（2026-10-09 無頭 Chrome 實測）。"""
    truth = [(25.071956, 121.211472, 36.445), (25.0777, 121.2325, 47.437), (24.9894, 121.3139, 124.2),
             (23.5100, 120.8057, 2288.7)]
    w = CesiumWorldTerrain(_live_token())
    for lat, lon, js in truth:
        assert w.get_elevation(lat, lon) == pytest.approx(js, abs=1.5), (lat, lon)
    assert w.failures == [] and w.attribution_text
