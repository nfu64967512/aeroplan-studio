# -*- coding: utf-8 -*-
"""glb_merge.py — 把 CAD 匯出的 GLB 合併成「每種外觀一個繪製單元」，讓 3D 多機可以順跑。

為什麼（2026-10-17 量測，tools/cesium_fps_probe.py + CDP CPU profile）：
  ui/resources/vtol.glb 是 THREE.GLTFExporter 從 CAD 匯出的：819 個節點、1192 個 primitive、103 萬三角形、
  51 個材質但**全都沒有 baseColorFactor**（名字寫 Opaque(r,g,b) 但顏色沒匯出，畫出來全是同一種白）。
  Cesium 每個 primitive 一次 draw call ⇒ 每架每幀 1192 次 draw＋uniform 設定；6 架 ≈ 7000 次，
  JS 每幀 33 ms ⇒ 最多 28 FPS。copter.glb 同樣是 383 個 primitive。
  GPU 畫 100 萬三角形不是問題（RTX 4070 空場景 1200 FPS），問題是 draw call 數。

做法：把所有節點的變換烘進頂點（這兩個檔其實沒有節點變換，但照規格處理 matrix/TRS 與負行列式），
依「外觀簽名」（材質去掉 name/extras 後的內容）分組合併，每組一個 primitive，uint32 索引。
幾何與外觀完全不變：同樣的頂點座標、法向量、三角形、材質參數；只丟掉沒有材質用到的 TEXCOORD_0。

任何不支援的情況都回 None ⇒ 呼叫端用原檔：有貼圖、非三角形、sparse、外部 buffer、必要擴充（Draco/量化…）、
POSITION/NORMAL/TEXCOORD_n 以外的頂點屬性（例如 COLOR_0 頂點色，合併後會遺失）、材質類（KHR_materials_*）以外的
可選擴充（例如 EXT_mesh_gpu_instancing，合併後實例會不見）、節點／網格／primitive 層的擴充、骨架／動畫、節點圖有環。
"""
from __future__ import annotations

import hashlib
import json
import logging
import os
import pathlib
import struct
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)

MERGE_VERSION = 2                 # 改演算法就 +1，快取檔名帶著它，舊快取自動失效（v2：屬性白名單、頂點壓實、輸出宣告擴充）
MIN_PRIMITIVES_TO_MERGE = 8       # primitive 少於這個數的檔（例如 barracuda500.glb 只有 3 個）不值得動
MAX_NODE_VISITS = 200_000         # 節點走訪上限（實例展開爆量時放棄，避免在 GUI 執行緒卡住）
_MERGEABLE_ATTRS = ('POSITION', 'NORMAL')   # 另外 TEXCOORD_n 允許（沒有貼圖 ⇒ 用不到，合併時丟掉）
_OFF_VALUES = ('0', 'false', 'off', 'no')

_GLB_MAGIC = 0x46546C67           # b'glTF'
_CHUNK_JSON = 0x4E4F534A
_CHUNK_BIN = 0x004E4942
_COMP_DTYPE = {5120: np.int8, 5121: np.uint8, 5122: np.int16, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
_TYPE_N = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT2': 4, 'MAT3': 9, 'MAT4': 16}


class _Unsupported(Exception):
    pass


def _parse_glb(data: bytes) -> tuple[dict, memoryview]:
    if len(data) < 20:
        raise _Unsupported('檔案太短')
    magic, version, length = struct.unpack_from('<III', data, 0)
    if magic != _GLB_MAGIC or version != 2:
        raise _Unsupported('不是 glTF 2.0 GLB')
    off, js, bin_ = 12, None, None
    while off + 8 <= min(length, len(data)):
        clen, ctype = struct.unpack_from('<II', data, off)
        body = memoryview(data)[off + 8: off + 8 + clen]
        if ctype == _CHUNK_JSON and js is None:
            js = json.loads(bytes(body).decode('utf-8'))
        elif ctype == _CHUNK_BIN and bin_ is None:
            bin_ = body
        off += 8 + clen
    if js is None or bin_ is None:
        raise _Unsupported('缺 JSON 或 BIN chunk')
    return js, bin_


def _read_accessor(js: dict, bin_: memoryview, idx: int) -> np.ndarray:
    a = js['accessors'][idx]
    if 'sparse' in a or 'bufferView' not in a:
        raise _Unsupported('sparse／無 bufferView 的 accessor')
    bv = js['bufferViews'][a['bufferView']]
    if bv.get('buffer', 0) != 0 or 'uri' in js['buffers'][bv.get('buffer', 0)]:
        raise _Unsupported('外部 buffer')
    dt = np.dtype(_COMP_DTYPE[a['componentType']])
    n = _TYPE_N[a['type']]
    count = int(a['count'])
    item = dt.itemsize * n
    stride = int(bv.get('byteStride') or item)
    off = int(bv.get('byteOffset', 0)) + int(a.get('byteOffset', 0))
    if count == 0:
        return np.zeros((0, n), dtype=dt)
    end = off + stride * (count - 1) + item
    if end > int(bv.get('byteOffset', 0)) + int(bv['byteLength']) or end > len(bin_):
        raise _Unsupported('accessor 超出 bufferView')
    arr = np.ndarray(shape=(count, n), dtype=dt, buffer=bin_, offset=off, strides=(stride, dt.itemsize)).copy()
    if a.get('normalized'):
        info = np.iinfo(dt)
        arr = np.maximum(arr.astype(np.float64) / info.max, -1.0).astype(np.float32)
    return arr


def _node_matrix(node: dict) -> np.ndarray:
    if 'matrix' in node:
        return np.array(node['matrix'], dtype=np.float64).reshape(4, 4).T     # glTF 是 column-major
    m = np.eye(4)
    if 'scale' in node:
        m = np.diag([*node['scale'], 1.0]) @ m
    if 'rotation' in node:
        x, y, z, w = node['rotation']
        r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        rm = np.eye(4); rm[:3, :3] = r
        m = rm @ m
    if 'translation' in node:
        t = np.eye(4); t[:3, 3] = node['translation']
        m = t @ m
    return m


def _uses_texture(obj: Any) -> bool:
    if isinstance(obj, dict):
        return any((k.endswith('Texture') and isinstance(v, dict)) or _uses_texture(v) for k, v in obj.items())
    if isinstance(obj, list):
        return any(_uses_texture(v) for v in obj)
    return False


def _material_signature(js: dict, mat_idx: int | None) -> str:
    if mat_idx is None:
        return '__default__'
    m = {k: v for k, v in js['materials'][mat_idx].items() if k not in ('name', 'extras')}
    return json.dumps(m, sort_keys=True)


def merge_glb_bytes(data: bytes) -> tuple[bytes, dict] | None:
    """合併成每種外觀一個 primitive。回 (新 GLB bytes, 統計)；不支援就回 None。"""
    try:
        return _merge(data)
    except _Unsupported as e:
        logger.info(f'[GLB] 不合併：{e}')
        return None


def _merge(data: bytes) -> tuple[bytes, dict]:
    js, bin_ = _parse_glb(data)
    if js.get('extensionsRequired'):
        raise _Unsupported(f'必要擴充 {js["extensionsRequired"]}')
    if js.get('skins') or js.get('animations'):
        raise _Unsupported('有骨架或動畫')
    if any(_uses_texture(m) for m in js.get('materials', [])):
        raise _Unsupported('材質有貼圖')
    # 只認得「材質類」的可選擴充（合併時材質原樣複製，輸出照樣宣告）；其他可選擴充（例如 EXT_mesh_gpu_instancing
    # 會讓 Cesium 畫出多個實例）合併後會靜靜不見 ⇒ 不合併（2026-10-17 審查）
    used = list(js.get('extensionsUsed') or [])
    odd = [e for e in used if not e.startswith('KHR_materials_')]
    if odd:
        raise _Unsupported(f'可選擴充 {odd}')
    nodes = js.get('nodes', [])
    meshes = js.get('meshes', [])
    for what, objs in (('node', nodes), ('mesh', meshes)):
        if any(o.get('extensions') for o in objs):
            raise _Unsupported(f'{what} 層有擴充')
    for m in meshes:
        for pr in m.get('primitives', []):
            if pr.get('extensions'):
                raise _Unsupported('primitive 層有擴充')
            bad = [a for a in pr.get('attributes', {}) if a not in _MERGEABLE_ATTRS and not a.startswith('TEXCOORD_')]
            if bad:
                raise _Unsupported(f'primitive 屬性 {bad} 合併後會遺失（例如 COLOR_0 頂點色）')
    if js.get('scenes'):
        scene = js['scenes'][js.get('scene', 0)]
    else:                                   # 沒有 scenes：根 = 沒被任何節點當成 child 的節點（不能每個節點都當根，子節點會重複）
        kids = {c for n in nodes for c in n.get('children', [])}
        scene = {'nodes': [i for i in range(len(nodes)) if i not in kids]}

    groups: dict[str, dict] = {}            # 簽名 → {'mat': 原材質 idx, 'pos': [], 'nrm': [], 'idx': [], 'nv': int}
    n_prims_in = 0
    tris_in = 0
    stack = [(i, np.eye(4), frozenset()) for i in scene.get('nodes', [])]
    seen_guard = 0
    while stack:
        ni, parent, path = stack.pop()
        seen_guard += 1
        visit = seen_guard                      # 這一次節點走訪的編號（同一 mesh 被多個節點引用時各自烘一份）
        if ni in path:
            raise _Unsupported('節點圖有環')
        if seen_guard > MAX_NODE_VISITS:
            raise _Unsupported(f'節點走訪超過 {MAX_NODE_VISITS} 次（實例爆量）')
        node = nodes[ni]
        world = parent @ _node_matrix(node)
        for c in node.get('children', []):
            stack.append((c, world, path | {ni}))
        if 'mesh' not in node:
            continue
        lin = world[:3, :3]
        flip = np.linalg.det(lin) < 0
        try:
            nrm_m = np.linalg.inv(lin).T
        except np.linalg.LinAlgError:
            raise _Unsupported('節點矩陣不可逆') from None
        for pr in js['meshes'][node['mesh']]['primitives']:
            n_prims_in += 1
            if pr.get('mode', 4) != 4:
                raise _Unsupported(f'primitive mode {pr.get("mode")}（只處理三角形）')
            if pr.get('targets'):
                raise _Unsupported('morph targets')
            attrs = pr['attributes']
            has_n = 'NORMAL' in attrs
            sig = _material_signature(js, pr.get('material')) + ('|N' if has_n else '|noN')
            g = groups.setdefault(sig, {'mat': pr.get('material'), 'pos': [], 'nrm': [], 'idx': [], 'nv': 0,
                                        'has_n': has_n, 'base': {}})
            # 同一個 mesh 的多個 primitive 常共用同一份 POSITION/NORMAL（CAD 匯出：一份頂點、每個材質一組索引）：
            # 同一次節點走訪、同一組頂點 accessor、同一外觀組 ⇒ 頂點只放一次，索引指回去（否則檔案膨脹 4 倍）
            vkey = (visit, attrs['POSITION'], attrs.get('NORMAL'))
            nverts = int(js['accessors'][attrs['POSITION']]['count'])
            if 'indices' in pr:
                idx = _read_accessor(js, bin_, pr['indices']).reshape(-1).astype(np.int64)
            else:
                idx = np.arange(nverts, dtype=np.int64)
            if len(idx) % 3:
                raise _Unsupported('索引數不是 3 的倍數')
            if len(idx) and (idx.min() < 0 or idx.max() >= nverts):
                raise _Unsupported('索引越界')
            tris_in += len(idx) // 3
            if flip:
                idx = idx.reshape(-1, 3)[:, [0, 2, 1]].reshape(-1)
            if vkey not in g['base']:
                pos = _read_accessor(js, bin_, attrs['POSITION']).astype(np.float64)
                if pos.shape[1] != 3:
                    raise _Unsupported('POSITION 不是 VEC3')
                g['pos'].append((pos @ lin.T + world[:3, 3]).astype(np.float32))
                if has_n:
                    nrm = _read_accessor(js, bin_, attrs['NORMAL']).astype(np.float64) @ nrm_m.T
                    ln = np.linalg.norm(nrm, axis=1, keepdims=True)
                    nrm = np.divide(nrm, ln, out=np.zeros_like(nrm), where=ln > 1e-12)
                    g['nrm'].append(nrm.astype(np.float32))
                g['base'][vkey] = g['nv']
                g['nv'] += nverts
            g['idx'].append(idx + g['base'][vkey])

    if not groups:
        raise _Unsupported('沒有任何三角形')

    # ── 寫出新 GLB ──
    blob = bytearray()
    buffer_views: list[dict] = []
    accessors: list[dict] = []
    materials: list[dict] = []
    mat_remap: dict[Any, int] = {}
    prims_out = []

    def add_view(arr: np.ndarray, target: int) -> int:
        while len(blob) % 4:
            blob.append(0)
        buffer_views.append({'buffer': 0, 'byteOffset': len(blob), 'byteLength': arr.nbytes, 'target': target})
        blob.extend(arr.tobytes())
        return len(buffer_views) - 1

    tris_out = 0
    for g in groups.values():
        pos = np.concatenate(g['pos'])
        idx = np.concatenate(g['idx'])
        nrm_all = np.concatenate(g['nrm']) if g['has_n'] else None
        # 壓實：一份頂點被不同外觀組的 primitive 共用時，每組只留自己用到的頂點（否則有色材質時檔案反而變大）
        used, idx = np.unique(idx, return_inverse=True)
        idx = idx.reshape(-1)
        if len(used) < len(pos):
            pos = pos[used]
            if nrm_all is not None:
                nrm_all = nrm_all[used]
        tris_out += len(idx) // 3
        idx_dt, idx_ct = (np.uint16, 5123) if len(pos) <= 65535 else (np.uint32, 5125)
        a_pos = len(accessors)
        accessors.append({'bufferView': add_view(pos, 34962), 'componentType': 5126, 'count': len(pos), 'type': 'VEC3',
                          'min': [float(x) for x in pos.min(axis=0)], 'max': [float(x) for x in pos.max(axis=0)]})
        attrs = {'POSITION': a_pos}
        if nrm_all is not None:
            attrs['NORMAL'] = len(accessors)
            accessors.append({'bufferView': add_view(nrm_all, 34962), 'componentType': 5126, 'count': len(nrm_all),
                              'type': 'VEC3'})
        a_idx = len(accessors)
        accessors.append({'bufferView': add_view(idx.astype(idx_dt), 34963), 'componentType': idx_ct,
                          'count': len(idx), 'type': 'SCALAR'})
        prim = {'attributes': attrs, 'indices': a_idx, 'mode': 4}
        if g['mat'] is not None:
            if g['mat'] not in mat_remap:
                mat_remap[g['mat']] = len(materials)
                materials.append(js['materials'][g['mat']])
            prim['material'] = mat_remap[g['mat']]
        prims_out.append(prim)
    while len(blob) % 4:
        blob.append(0)

    gen = (js.get('asset') or {}).get('generator', '?')
    out_js = {
        'asset': {'version': '2.0', 'generator': f'AeroPlan glb_merge v{MERGE_VERSION} (from {gen})'},
        'scene': 0, 'scenes': [{'nodes': [0]}],
        'nodes': [{'mesh': 0, 'name': 'merged'}],
        'meshes': [{'name': 'merged', 'primitives': prims_out}],
        'accessors': accessors, 'bufferViews': buffer_views, 'buffers': [{'byteLength': len(blob)}],
    }
    if materials:
        out_js['materials'] = materials
        mat_ext = sorted({e for m in materials for e in (m.get('extensions') or {})})
        if mat_ext:                              # 材質擴充原樣保留 ⇒ 也要宣告，否則是不合規格的 glTF
            out_js['extensionsUsed'] = mat_ext
    jb = json.dumps(out_js, separators=(',', ':'), ensure_ascii=False).encode('utf-8')
    jb += b' ' * ((4 - len(jb) % 4) % 4)
    total = 12 + 8 + len(jb) + 8 + len(blob)
    out = struct.pack('<III', _GLB_MAGIC, 2, total) + struct.pack('<II', len(jb), _CHUNK_JSON) + jb \
        + struct.pack('<II', len(blob), _CHUNK_BIN) + bytes(blob)
    stats = {'primitives_in': n_prims_in, 'primitives_out': len(prims_out), 'triangles_in': tris_in,
             'triangles_out': tris_out, 'bytes_in': len(data), 'bytes_out': len(out)}
    return out, stats


def count_primitives(path: pathlib.Path) -> int:
    """只讀 JSON chunk 數 primitive（不讀整個 BIN）。讀不了回 0。"""
    try:
        with open(path, 'rb') as f:
            head = f.read(20)
            magic, _ver, _len = struct.unpack_from('<III', head, 0)
            clen, ctype = struct.unpack_from('<II', head, 12)
            if magic != _GLB_MAGIC or ctype != _CHUNK_JSON:
                return 0
            js = json.loads(f.read(clen).decode('utf-8'))
        return sum(len(m.get('primitives', [])) for m in js.get('meshes', []))
    except Exception:                                              # noqa: BLE001
        return 0


def optimized_model_path(src: pathlib.Path, cache_dir: pathlib.Path) -> pathlib.Path:
    """回傳合併後的快取檔（必要時現做）；不需要／做不成就回原檔。

    快取鍵 = 原檔大小 + mtime_ns + MERGE_VERSION（原檔換了就重做）。原子寫入（暫存檔 + os.replace）。
    環境變數 AEROPLAN_GLB_MERGE=0／false／off／no ⇒ 一律用原檔（對照用）。
    """
    src = pathlib.Path(src)
    if os.environ.get('AEROPLAN_GLB_MERGE', '1').strip().lower() in _OFF_VALUES or not src.exists():
        return src
    tmp = None
    try:
        if count_primitives(src) < MIN_PRIMITIVES_TO_MERGE:
            return src
        st = src.stat()
        key = hashlib.sha1(f'{src.name}|{st.st_size}|{st.st_mtime_ns}|v{MERGE_VERSION}'.encode()).hexdigest()[:12]
        dst = cache_dir / f'{src.stem}.merged-{key}.glb'
        if dst.exists() and dst.stat().st_size > 0:
            return dst
        res = merge_glb_bytes(src.read_bytes())
        if res is None:
            return src
        out, stats = res
        cache_dir.mkdir(parents=True, exist_ok=True)
        tmp = dst.with_suffix(f'.tmp{os.getpid()}')
        tmp.write_bytes(out)
        try:
            os.replace(tmp, dst)
        except OSError:
            # Windows：另一個 app 實例剛好寫好同一個檔、而且頁面正開著它（WinError 5）⇒ 用它的
            if dst.exists() and dst.stat().st_size > 0:
                logger.info(f'[GLB] {dst.name} 已由另一個行程寫好，直接使用')
                return dst
            raise
        finally:
            try:
                if tmp.exists():
                    tmp.unlink()
            except OSError:
                pass
        for old in cache_dir.glob(f'{src.stem}.merged-*.glb'):      # 同一原檔的舊快取
            if old != dst:
                try:
                    old.unlink()
                except OSError:
                    pass
        logger.info(f'[GLB] {src.name} 合併：primitive {stats["primitives_in"]}→{stats["primitives_out"]}、'
                    f'三角形 {stats["triangles_in"]}→{stats["triangles_out"]}、'
                    f'{stats["bytes_in"] / 1e6:.1f}→{stats["bytes_out"] / 1e6:.1f} MB ⇒ {dst.name}')
        return dst
    except Exception as e:                                         # noqa: BLE001
        logger.warning(f'[GLB] {src.name} 合併失敗，用原檔：{e}')
        try:
            if tmp is not None and tmp.exists():
                tmp.unlink()
        except OSError:
            pass
        return src


if __name__ == '__main__':                                         # python -m ui.widgets.glb_merge 檔.glb [輸出.glb]
    import sys
    import time
    logging.basicConfig(level=logging.INFO)
    t0 = time.perf_counter()
    r = merge_glb_bytes(pathlib.Path(sys.argv[1]).read_bytes())
    if r is None:
        print('不支援，未輸出'); sys.exit(1)
    if len(sys.argv) > 2:
        pathlib.Path(sys.argv[2]).write_bytes(r[0])
    print(json.dumps(r[1], ensure_ascii=False), f'{time.perf_counter() - t0:.2f} s')
