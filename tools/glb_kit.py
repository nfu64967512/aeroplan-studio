# -*- coding: utf-8 -*-
"""glb_kit.py — 由原廠 STEP＋參數化零件組出 CesiumJS 用 GLB 的共用工具（tools/build_hero_glb.py、build_pioneer_glb.py）。

座標慣例：建模用 CAD 座標（mm；x 往後、y 往右翼、z 往上），匯出 glTF：公尺、+X 機鼻、+Y 上、+Z 右翼（與 barracuda500.glb
同慣例 ⇒ main.html 校正 0/0/0、scale 1），原點＝重心。這個轉換是真旋轉（行列式 +1）⇒ 三角形繞行不變、左右不會對調。

輸出：一個 mesh、每種材質一個 primitive，只有 POSITION／NORMAL＋baseColorFactor（沒有貼圖／擴充 ⇒ glb_merge 不必合併）。
同樣輸入逐位元組相同；STEP 三角化快取以 STEP 的 git blob sha＋cascadio 版本＋容差為鍵（換了來源或工具不會吃到舊網格）。
只有建模時需要 cascadio、trimesh、shapely、numpy；app 執行期不需要。
"""
from __future__ import annotations

import base64
import hashlib
import json
import math
import os
import pathlib
import struct
import subprocess
import sys
import tempfile
import urllib.request

import numpy as np


# ═════════════════════════════════════════════════════════════════════════════
#  來源（gitee blob API；大檔 raw 連結要登入）
# ═════════════════════════════════════════════════════════════════════════════
def git_blob_sha1(data: bytes) -> str:
    return hashlib.sha1(b'blob %d\0' % len(data) + data).hexdigest()


def fetch_blobs(sources: dict, repo: str, dst: pathlib.Path) -> None:
    """sources：英文檔名 → (原廠檔名, git blob sha1)。存成 dst/<英文檔名>.stp（OpenCASCADE 讀不了中文路徑），下載後驗 sha1。"""
    dst.mkdir(parents=True, exist_ok=True)
    for key, (name, sha) in sources.items():
        p = dst / f'{key}.stp'
        if p.exists() and git_blob_sha1(p.read_bytes()) == sha:
            print(f'  ok   {key}')
            continue
        url = f'https://gitee.com/api/v5/repos/{repo}/git/blobs/{sha}'
        with urllib.request.urlopen(url, timeout=300) as r:
            data = base64.b64decode(json.loads(r.read())['content'])
        if git_blob_sha1(data) != sha:
            raise RuntimeError(f'{key}: 下載內容的 sha1 對不上 {sha}（{name}）')
        p.write_bytes(data)
        print(f'  get  {key}  {len(data):,} B')


def tool_versions() -> dict:
    out = {'python': sys.version.split()[0]}
    for mod in ('cascadio', 'trimesh', 'numpy', 'shapely'):
        try:
            out[mod] = getattr(__import__(mod), '__version__', '?')
        except Exception:                       # noqa: BLE001 — 只是記錄
            out[mod] = None
    return out


def load_step(src: pathlib.Path, key: str, sha: str, cache: pathlib.Path, tol=(0.6, 0.5)):
    """STEP → [(V mm, F, 原檔顏色 rgba)]，每個 glTF 幾何一組（已套節點變換）。
    快取檔名含 STEP sha、cascadio 版本、容差 ⇒ 換了任何一個都重做；先寫暫存檔再改名（中斷不會留下半個檔）。"""
    import cascadio
    import trimesh
    stp = src / f'{key}.stp'
    if not stp.exists():
        raise FileNotFoundError(f'{stp} 不存在 ⇒ 先跑 fetch')
    if git_blob_sha1(stp.read_bytes()) != sha:
        raise RuntimeError(f'{stp} 的 sha1 不是釘住的版本 ⇒ 重跑 fetch')
    cache.mkdir(parents=True, exist_ok=True)
    ver = getattr(cascadio, '__version__', 'unknown')
    glb = cache / f'{key}.{sha[:12]}.cascadio{ver}.{tol[0]}_{tol[1]}.glb'
    if not glb.exists():
        tmp = glb.with_suffix('.tmp.glb')
        cascadio.step_to_glb(str(stp), str(tmp), tol_linear=tol[0], tol_angular=tol[1])
        os.replace(tmp, glb)
    scene = trimesh.load(str(glb), process=False)
    out = []
    for node in sorted(scene.graph.nodes_geometry):
        T, gname = scene.graph[node]
        g = scene.geometry[gname]
        V = np.asarray(g.vertices, dtype=np.float64) @ T[:3, :3].T + T[:3, 3]
        mat = getattr(g.visual, 'material', None)
        col = tuple(int(c) for c in getattr(mat, 'baseColorFactor', (255, 255, 255, 255)))
        out.append((V * 1000.0, np.asarray(g.faces, dtype=np.int64), col))
    return out


# ═════════════════════════════════════════════════════════════════════════════
#  網格小工具（V: (n,3) float mm、F: (m,3) int）
# ═════════════════════════════════════════════════════════════════════════════
class Part:
    """一塊同材質網格。smooth=True ⇒ 匯出前依角度切開銳邊（參數化零件共用頂點）；
    False ⇒ 頂點照 STEP（OCCT 每個 B-rep 面各自三角化、不共用頂點 ⇒ 面與面之間本來就是銳邊）。"""

    def __init__(self, name, mat, V, F, smooth):
        self.name, self.mat, self.smooth = name, mat, smooth
        self.V = np.asarray(V, dtype=np.float64)
        self.F = np.asarray(F, dtype=np.int64)


def keep_faces(V, F, mask):
    F = F[mask]
    used, inv = np.unique(F.ravel(), return_inverse=True)
    return V[used], inv.reshape(-1, 3)


def face_centroids(V, F):
    return V[F].mean(axis=1)


def slice_keep(V, F, origin, normal):
    """用平面切開、留 (v − origin)·normal ≥ 0 那一側（三角形沿平面切齊，不留鋸齒；切口不封）。
    以面心挑三角形會在邊界留下一排凸出去的三角形，和相鄰零件的同一個外表面打架（z-fighting 鋸齒）。"""
    import trimesh
    m = trimesh.intersections.slice_mesh_plane(trimesh.Trimesh(V, F, process=False), np.asarray(normal, float),
                                               np.asarray(origin, float), cap=False)
    return np.asarray(m.vertices, np.float64), np.asarray(m.faces, np.int64)


def mirror_y(V, F):
    V = V.copy()
    V[:, 1] *= -1
    return V, F[:, ::-1].copy()                              # 鏡像會翻轉繞行方向 ⇒ 反過來


def shifted(V, d):
    return V + np.asarray(d, float)


def merged(bodies):
    V = np.vstack([b[0] for b in bodies])
    off = np.cumsum([0] + [len(b[0]) for b in bodies[:-1]])
    F = np.vstack([b[1] + o for b, o in zip(bodies, off)])
    return V, F


def section_segments(V, F, origin, normal):
    import trimesh
    m = trimesh.Trimesh(V, F, process=True)                    # 剖面要接得起來 ⇒ 這裡合併頂點
    return trimesh.intersections.mesh_plane(m, np.asarray(normal, float), np.asarray(origin, float))


def section_outline(V, F, origin, normal, axes, expect_full=False):
    """平面剖面的外輪廓（包絡面積最大的那一圈），回傳 (k,2)：axes 指定取哪兩個座標。
    自相交（無效）的環也收進來（make_valid 後聯集），否則外皮那圈一有毛病，polygonize 會默默只剩內圈
    （2026-10-23 審查：Pioneer 機頭接頭框因此縮進機身 15–36 mm、側面看得穿）。
    expect_full=True ⇒ 回傳的輪廓沒涵蓋整個剖面（任一方向差 > 1 mm）就丟例外，不靜默給內圈。"""
    import shapely
    from shapely.geometry import Polygon
    from shapely.ops import polygonize_full, unary_union
    seg = section_segments(V, F, origin, normal)
    lines = [tuple(map(tuple, np.round(s[:, axes], 6))) for s in seg]
    polys, _cuts, _dangles, invalid = polygonize_full(lines)
    shells = [Polygon(p.exterior) for p in polys.geoms]
    for r in invalid.geoms:
        if len(r.coords) >= 4:
            shells.append(shapely.make_valid(Polygon(r.coords)))
    pieces = []

    def collect(g):
        if g.geom_type == 'Polygon' and not g.is_empty:
            pieces.append(g)
        for sub in getattr(g, 'geoms', ()):
            collect(sub)
    collect(unary_union(shells) if shells else Polygon())
    if not pieces:
        raise RuntimeError(f'剖面 {origin} 沒有封閉輪廓')
    out = np.asarray(max(pieces, key=lambda p: p.area).exterior.coords)[:-1]
    if expect_full:
        P = seg[:, :, axes].reshape(-1, 2)
        gap = max((out.min(axis=0) - P.min(axis=0)).max(), (P.max(axis=0) - out.max(axis=0)).max())
        if gap > 1.0:
            raise RuntimeError(f'剖面 {origin} 的外輪廓沒涵蓋整個剖面（差 {gap:.1f} mm）⇒ 外皮沒閉合，改用 outer_hull')
    return out


def densify(P, step=0.5, closed=True):
    """折線加密到每段 ≤ step（mm）：極角取樣／包絡在長直邊上不會往外鼓、不會讓內部結構的頂點搶走空的方位。"""
    P = np.asarray(P, float)
    Q = np.vstack([P, P[:1]]) if closed else P
    out = []
    for a, b in zip(Q[:-1], Q[1:]):
        k = max(1, int(math.ceil(np.linalg.norm(b - a) / step)))
        out.append(a + (b - a) * np.linspace(0, 1, k, endpoint=False)[:, None])
    if not closed:
        out.append(Q[-1:])
    return np.vstack(out)


def densify_segments(seg, step=0.5):
    """剖面線段 (n,2,d) → 沿線段每 ≤ step 一點。"""
    return np.vstack([densify(np.asarray(s, float), step, closed=False) for s in seg])


def outer_hull(P, step=2.0):
    """點雲（剖面點）的凸包外圈、每 ≤ step 一點：外皮剖面不閉合（有開口、內部結構）時的外輪廓。"""
    import shapely
    from shapely.geometry import MultiPoint
    hull = MultiPoint([tuple(p) for p in np.asarray(P, float)]).convex_hull
    return np.asarray(shapely.segmentize(hull, step).exterior.coords)[:-1]


# ═════════════════════════════════════════════════════════════════════════════
#  參數化零件
# ═════════════════════════════════════════════════════════════════════════════
def naca4(code, n=60):
    """NACA 4 位數翼型，弦長 1、前緣在 0；封閉折線（後緣→上表面→前緣→下表面）。"""
    m, p, t = int(code[0]) / 100.0, int(code[1]) / 10.0, int(code[2:]) / 100.0
    x = (1 - np.cos(np.linspace(0, math.pi, n))) / 2
    yt = 5 * t * (0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * x ** 2 + 0.2843 * x ** 3 - 0.1036 * x ** 4)
    if m > 0:
        yc = np.where(x < p, m / p ** 2 * (2 * p * x - x ** 2), m / (1 - p) ** 2 * ((1 - 2 * p) + 2 * p * x - x ** 2))
        dy = np.where(x < p, 2 * m / p ** 2 * (p - x), 2 * m / (1 - p) ** 2 * (p - x))
    else:
        yc, dy = np.zeros_like(x), np.zeros_like(x)
    th = np.arctan(dy)
    up = np.c_[x - yt * np.sin(th), yc + yt * np.cos(th)][::-1]
    lo = np.c_[x + yt * np.sin(th), yc - yt * np.cos(th)][1:-1]
    return np.vstack([up, lo])


def resample_profile(poly, n):
    """封閉翼剖面 (x, z) → n 點：從後緣（x 最大）出發先走上表面到前緣（x 最小）再走下表面；
    兩半各自依弧長、兩端加密 ⇒ 形狀不同的兩個剖面也是前緣對前緣，放樣不會扭。"""
    poly = np.asarray(poly, float)
    poly = np.roll(poly, -int(np.argmax(poly[:, 0])), axis=0)

    def halves(p):
        i = int(np.argmin(p[:, 0]))
        return p[:i + 1], np.vstack([p[i:], p[:1]])
    a, b = halves(poly)
    if a[len(a) // 2, 1] < b[len(b) // 2, 1]:                  # a 要是上表面
        poly = np.roll(poly[::-1], 1, axis=0)
        a, b = halves(poly)

    def rs(seg, k):
        d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(seg, axis=0), axis=1))]
        s = (1 - np.cos(np.linspace(0, math.pi, k))) / 2 * d[-1]
        return np.c_[np.interp(s, d, seg[:, 0]), np.interp(s, d, seg[:, 1])]
    h = n // 2
    return np.vstack([rs(a, h + 1)[:-1], rs(b, n - h + 1)[:-1]])


def offset_profile(prof, dist, n=None):
    """封閉剖面往外推 dist（mm，束帶這種貼在外面的件用），再重新取樣成同樣點數。"""
    from shapely.geometry import Polygon
    poly = Polygon(prof).buffer(dist, join_style=2, mitre_limit=3.0)
    return resample_profile(np.asarray(poly.exterior.coords)[:-1], n or len(prof))


def resample_loop_polar(loop, n, center=None):
    """機身剖面這種星形封閉曲線 → 依極角等分 n 點（從 -π 起逆時針）。
    先把輪廓加密到每段 ≤0.5 mm：稀疏頂點之間對半徑做線性內插，長直邊會往外鼓（審查 F5）。中心用原頂點算（呼叫端不變）。"""
    loop = np.asarray(loop, float)
    c = loop.mean(axis=0) if center is None else np.asarray(center, float)
    loop = densify(loop, 0.5)
    ang = np.arctan2(loop[:, 1] - c[1], loop[:, 0] - c[0])
    r = np.linalg.norm(loop - c, axis=1)
    o = np.argsort(ang)
    a, r = ang[o], r[o]
    a = np.r_[a[-1] - 2 * math.pi, a, a[0] + 2 * math.pi]
    r = np.r_[r[-1], r, r[0]]
    t = np.linspace(-math.pi, math.pi, n, endpoint=False)
    rr = np.interp(t, a, r)
    return np.c_[c[0] + rr * np.cos(t), c[1] + rr * np.sin(t)]


def loft(sections, cap=True):
    """同點數的剖面串（沿展向排）→ 封閉網格；端面用扇形封。"""
    S, k = len(sections), len(sections[0])
    V = [np.vstack(sections)]
    F = []
    i = np.arange(k)
    j = (i + 1) % k
    for s in range(S - 1):
        o0, o1 = s * k, (s + 1) * k
        F.append(np.c_[o0 + i, o0 + j, o1 + j])
        F.append(np.c_[o0 + i, o1 + j, o1 + i])
    if cap:
        nv = S * k
        for s, flip in ((0, True), (S - 1, False)):
            V.append(sections[s].mean(axis=0, keepdims=True))
            o = s * k
            F.append(np.c_[np.full(k, nv), o + j, o + i] if flip else np.c_[np.full(k, nv), o + i, o + j])
            nv += 1
    V, F = np.vstack(V), np.vstack(F)
    return orient_outward(V, F)


def orient_outward(V, F):
    """封閉網格的有號體積 < 0 ⇒ 整個翻面（放樣方向不同時不用逐個想繞行）。"""
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    vol = np.einsum('ij,ij->i', a, np.cross(b, c)).sum()
    return (V, F[:, ::-1].copy()) if vol < 0 else (V, F)


def frame_from_axis(axis, radial=None):
    """回傳 3×3 [r t a]：a＝軸、r＝指定的徑向（正交化）、t＝a×r。"""
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    r = np.array([1.0, 0, 0]) if radial is None else np.asarray(radial, float)
    if abs(np.dot(r, a)) > 0.99:
        r = np.array([0, 1.0, 0])
    r = r - np.dot(r, a) * a
    r /= np.linalg.norm(r)
    return np.c_[r, np.cross(a, r), a]


def cylinder(p0, p1, r, seg=24, r1=None):
    """p0→p1 的圓柱（r1 給了就是圓錐台；r1=0 ⇒ 錐）。"""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    R = frame_from_axis(p1 - p0)
    t = np.linspace(0, 2 * math.pi, seg, endpoint=False)
    ring = np.c_[np.cos(t), np.sin(t), np.zeros(seg)]
    r1 = r if r1 is None else r1
    s0 = p0 + (ring * r) @ R.T
    s1 = p1 + (ring * max(r1, 1e-3)) @ R.T
    return loft([s0, s1])


def rounded_box(center, size, r, seg=6):
    """xy 平面圓角矩形（四角半徑 r）沿 z 擠出：size=(lx, ly, lz)。"""
    cx, cy, cz = center
    lx, ly, lz = size
    pts = []
    for qx, qy, a0 in ((1, 1, 0), (-1, 1, 90), (-1, -1, 180), (1, -1, 270)):
        ox, oy = cx + qx * (lx / 2 - r), cy + qy * (ly / 2 - r)
        for a in np.linspace(math.radians(a0), math.radians(a0 + 90), seg):
            pts.append((ox + r * math.cos(a), oy + r * math.sin(a)))
    pts = np.array(pts)
    return loft([np.c_[pts, np.full(len(pts), cz - lz / 2)], np.c_[pts, np.full(len(pts), cz + lz / 2)]])


def box(lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c = (lo + hi) / 2
    return rounded_box(c, hi - lo, min(1.0, *(hi - lo) / 4), seg=2)


def propeller(hub, axis, radial, diameter, handed=1, chord_root=30.0, chord_tip=12.0, thick=3.2, hub_d=None):
    """靜止的二葉槳：葉片沿 ±radial、漸縮、帶扭角，加一個槳轂。
    handed=+1 ⇒ 從 +axis 那一側看逆時針轉時推力朝 +axis（APC「E」／正槳）；-1 ⇒ 順時針（「EP」／反槳）。
    兩片葉用同一個扭角正負（第二片是第一片繞軸轉 180°）；葉根多一段伸進槳轂（葉片和槳轂接在一起）。"""
    R = frame_from_axis(axis, radial)
    rad = diameter / 2.0
    hd = diameter * 0.06 if hub_d is None else hub_d
    r0 = rad * 0.10
    rr = np.r_[hd / 2 - 1.0, np.linspace(r0, rad, 12)]
    blades_V, blades_F = [], []
    nv = 0
    for side in (1, -1):
        secs = []
        for k, ri in enumerate(rr):
            f = max(0.0, (ri - r0) / (rad - r0))
            c = chord_root + (chord_tip - chord_root) * f ** 0.9
            if k == 0:
                c = min(c, 0.9 * hd)                                    # 葉根伸進槳轂：不要從側面凸出去
            if f > 0.94:                                                # 圓弧葉尖
                c *= math.sqrt(max(0.05, 1 - ((f - 0.94) / 0.06) ** 2))
            tw = math.radians(handed * (24 - 14 * f))                   # 扭角：根 24°、尖 10°
            u = np.linspace(-0.5, 0.5, 7) * c
            hz = thick * (1 - (2 * u / c) ** 2) * (0.5 - 0.3 * f)
            sec = np.vstack([np.c_[u, hz], np.c_[u[::-1][1:-1], -hz[::-1][1:-1]]])
            t = sec[:, 0] * math.cos(tw) - sec[:, 1] * math.sin(tw)
            a = sec[:, 0] * math.sin(tw) + sec[:, 1] * math.cos(tw)
            secs.append(np.c_[np.full(len(t), side * ri), side * t, a])
        V, F = loft(secs)
        blades_V.append(V); blades_F.append(F + nv); nv += len(V)
    V, F = np.vstack(blades_V), np.vstack(blades_F)
    V = V @ R.T + np.asarray(hub, float)
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    hv, hf = cylinder(np.asarray(hub) - a * 6, np.asarray(hub) + a * 6, hd / 2, 20)
    return np.vstack([V, hv]), np.vstack([F, hf + len(V)])


# ═════════════════════════════════════════════════════════════════════════════
#  法向、座標轉換、GLB
# ═════════════════════════════════════════════════════════════════════════════
def vertex_normals(V, F):
    fn = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])     # 面積加權
    N = np.zeros_like(V)
    for k in range(3):
        np.add.at(N, F[:, k], fn)
    ln = np.linalg.norm(N, axis=1)
    bad = ln < 1e-12
    N[bad] = (0.0, 0.0, 1.0)
    ln[bad] = 1.0
    return N / ln[:, None]


def split_sharp(V, F, angle_deg=35.0):
    import trimesh
    m = trimesh.graph.smooth_shade(trimesh.Trimesh(V, F, process=False), angle=math.radians(angle_deg))
    return np.asarray(m.vertices, np.float64), np.asarray(m.faces, np.int64)


def to_gltf(V, cg):
    """CAD（mm；x 後、y 右翼、z 上）→ glTF（m；+X 機鼻、+Y 上、+Z 右翼），原點移到重心。真旋轉（det=+1）⇒ 繞行不變。"""
    d = np.asarray(V, float) - np.asarray(cg, float)
    return np.c_[-d[:, 0], d[:, 2], d[:, 1]] / 1000.0


def build_groups(parts, materials, cg):
    groups = {m[0]: ([], [], []) for m in materials}
    for p in parts:
        V, F = (split_sharp(p.V, p.F) if p.smooth else (p.V, p.F))
        keep = np.linalg.norm(np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]), axis=1) > 1e-9
        V, F = keep_faces(V, F, keep)
        N = vertex_normals(V, F)
        Vg, Ng = to_gltf(V, cg), np.c_[-N[:, 0], N[:, 2], N[:, 1]]
        gv, gn, gf = groups[p.mat]
        off = sum(len(v) for v in gv)
        gv.append(Vg); gn.append(Ng); gf.append(F + off)
    out = []
    for name, *_ in materials:
        gv, gn, gf = groups[name]
        if gv:
            out.append((name, np.vstack(gv).astype(np.float32), np.vstack(gn).astype(np.float32),
                        np.vstack(gf).astype(np.uint32)))
    return out


def write_glb(groups, path: pathlib.Path, extras: dict, materials, name='Model') -> bytes:
    bin_parts, views, accessors, prims = [], [], [], []
    offset = 0

    def push(arr: np.ndarray, target):
        nonlocal offset
        data = arr.tobytes()
        views.append({'buffer': 0, 'byteOffset': offset, 'byteLength': len(data), 'target': target})
        bin_parts.append(data)
        offset += len(data)
        pad = (-offset) % 4
        if pad:
            bin_parts.append(b'\0' * pad); offset += pad
        return len(views) - 1

    index = {m[0]: m for m in materials}
    used = []
    for gname, V, N, F in groups:
        iv = push(V, 34962)
        accessors.append({'bufferView': iv, 'componentType': 5126, 'count': len(V), 'type': 'VEC3',
                          'min': [float(x) for x in V.min(axis=0)], 'max': [float(x) for x in V.max(axis=0)]})
        a_pos = len(accessors) - 1
        accessors.append({'bufferView': push(N, 34962), 'componentType': 5126, 'count': len(N), 'type': 'VEC3'})
        a_nrm = len(accessors) - 1
        accessors.append({'bufferView': push(F.ravel(), 34963), 'componentType': 5125, 'count': int(F.size),
                          'type': 'SCALAR'})
        used.append(gname)
        prims.append({'attributes': {'POSITION': a_pos, 'NORMAL': a_nrm}, 'indices': len(accessors) - 1,
                      'material': len(used) - 1, 'mode': 4})
    mats = []
    for gname in used:
        _, rgba, metal, rough = index[gname]
        mats.append({'name': gname, 'pbrMetallicRoughness': {'baseColorFactor': rgba, 'metallicFactor': metal,
                                                             'roughnessFactor': rough}})
    gltf = {
        'asset': {'version': '2.0', 'generator': 'AeroPlan tools/glb_kit.py', 'extras': extras},
        'scene': 0, 'scenes': [{'nodes': [0]}], 'nodes': [{'name': name, 'mesh': 0}],
        'meshes': [{'name': name, 'primitives': prims}], 'materials': mats,
        'accessors': accessors, 'bufferViews': views, 'buffers': [{'byteLength': offset}],
    }
    js = json.dumps(gltf, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
    js += b' ' * ((-len(js)) % 4)
    binb = b''.join(bin_parts)
    out = (struct.pack('<III', 0x46546C67, 2, 12 + 8 + len(js) + 8 + len(binb))
           + struct.pack('<II', len(js), 0x4E4F534A) + js + struct.pack('<II', len(binb), 0x004E4942) + binb)
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(out)
    return out


def report_of(data: bytes, groups, out: pathlib.Path, n_parts: int, **info) -> dict:
    allV = np.vstack([g[1] for g in groups])
    lo, hi = allV.min(axis=0), allV.max(axis=0)
    return {
        'out': str(out), 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest(),
        'primitives': len(groups),
        'triangles': {g[0]: int(len(g[3])) for g in groups},
        'triangles_total': int(sum(len(g[3]) for g in groups)),
        'bounds_m': {'min': [round(float(v), 4) for v in lo], 'max': [round(float(v), 4) for v in hi]},
        'span_m': round(float(hi[2] - lo[2]), 4), 'length_m': round(float(hi[0] - lo[0]), 4),
        'height_m': round(float(hi[1] - lo[1]), 4), 'parts': n_parts, 'tools': tool_versions(), **info,
    }


def check_rebuild_identical(script: pathlib.Path, data: bytes, extra_args) -> bool:
    """決定性檢查：另開一個行程、用**全新的空快取**（重新三角化 STEP）再建一次，逐位元組比對。"""
    with tempfile.TemporaryDirectory() as td:
        out = pathlib.Path(td) / 'check.glb'
        cmd = [sys.executable, '-I', str(script), 'build', '--out', str(out), '--cache', str(pathlib.Path(td) / 'mesh'),
               *extra_args]
        r = subprocess.run(cmd, capture_output=True, text=True, encoding='utf-8', errors='replace')
        if r.returncode != 0 or not out.exists():
            print(r.stdout[-2000:], r.stderr[-2000:], file=sys.stderr)
            return False
        return out.read_bytes() == data
