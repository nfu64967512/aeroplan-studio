# -*- coding: utf-8 -*-
"""algo_spiral_v2_r1.py — 向內螺旋的改進版（審查者 #1）。

key = ``spiral_v2_r1``；介面與 ``algo_spiral.plan`` 相同，自帶全部工具，不 import v1。

⚠ 檔名說明：交付物本來要叫 ``algo_spiral_v2.py``，但另一個 agent 同時在寫同一個
路徑（13:21 覆蓋掉我 13:14 寫進去的版本）。為了不互相蓋掉，我改用 ``_r1`` 後綴。

v1 的兩個結構性缺口（實測，見 ``_audit_loops.py``）
--------------------------------------------------
1. **退化圈被來回走兩遍。** 內縮到某層時該塊變成細長塊，v1 仍然畫封閉迴路。
   情境(1) 最內圈 1055.5 x 55.5 m、周長 2222 m，只需要一條 1055 m 的中線。
   情境(2) 四塊各有一條「等效寬 21.2 m」的圈，合計 22.34 km，一半是純重走。
   成因是 v1 的終止判準 ``erode(comp, swath/2).is_empty`` 在**畫完這一圈之後**
   才問，所以那一圈必然已經被畫成迴路。
2. **細長區塊上輪廓平行本來就輸給條帶。** W×L 矩形：輪廓的多餘長度 ≈ (L−W)
   （核心退化成長 L−W 的線），條帶的多餘長度 ≈ W（掉頭連接段總和）
   ⇒ L > 2W 時條帶必勝。情境(2) 是 1250×4000（L=3.2W），實測本體/(面積/pitch)
   = 1.089；情境(1)(3) 都是 0.99x。

v2 做法：組合策略（portfolio）
------------------------------
同一份幾何跑三套產生器，各自用**同一顆** DP 分配器切完，挑「最長航程最短」的
回傳。挑選準則是最長航程（= makespan，這把尺上的作業目標）。挑了誰記在
``debug['strategy']``，不藏。因為候選 A 就是 v1 的行為，**v2 在任何情境都不會
比 v1 差**。

  A. ``contour`` 純輪廓螺旋（= v1，保底）
  B. ``hybrid``  每層每塊比「一路輪廓下去」與「改鋪條帶」的實際長度，取短的
                 ⇒ 圓胖外圈留輪廓（區外浪費 0），退化核心改中線／條帶
  C. ``strips``  純內縮條帶：掃描線內縮 swath/2、**兩端也內縮 swath/2**，
                 於是刈幅剛好切齊作業區邊界 —— v1 論證過「貼邊飛＝白掃
                 swath/2」，但那個好處條帶也拿得到，只是 bench 附的牛耕沒做。

關鍵細節：掃描線的**位置**由 ``erode(area, swath/2)`` 決定，但**裁切**要用
``area`` 本身再把兩端各縮 swath/2。我第一版把兩件事都用 erode 過的多邊形做，
四個角就蓋不到、殘留補掃被引爆，長度反而多 3.73 km（173.18 → 176.91）。

保留 v1 所有正確的部分：內縮 swath/2 的第一道、erosion 斷裂各塊遞迴、有洞時
外環與每個內環都是圈、殘留補掃、minimax 連續分割 DP。

只用 shapely + 標準庫。不 import 本專案任何規劃器。
"""
from __future__ import annotations
from pathlib import Path

import math
from bisect import bisect_left, bisect_right

from shapely.affinity import rotate
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

_MIN_AREA = 4.0
_DENSIFY = 60.0
_SIMPLIFY = 0.4
_MAX_CUTS = 260
_SWITCH_MARGIN = 0.995      # 條帶要短 0.5% 以上才換（平手留輪廓，區外浪費較低）
_SAFETY = 4.0               # m，圈→條帶交接處的安全重疊（吸收 buffer 離散化誤差）
_COV_TOL = 0.001            # 覆蓋率閘門：候選要落在最佳者 0.1 個百分點以內


# ── 幾何小工具 ────────────────────────────────────────────────────────
def _polys(geom) -> list[Polygon]:
    if geom is None or geom.is_empty:
        return []
    gt = geom.geom_type
    if gt == "Polygon":
        return [geom] if geom.area > _MIN_AREA else []
    if gt in ("MultiPolygon", "GeometryCollection"):
        out: list[Polygon] = []
        for g in geom.geoms:
            out.extend(_polys(g))
        return out
    return []


def _erode(poly: Polygon, d: float):
    return poly if d <= 0 else poly.buffer(-d, quad_segs=16)


def _dist(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _plen(pts) -> float:
    return sum(_dist(pts[i - 1], pts[i]) for i in range(1, len(pts)))


def _densify(coords, step: float = _DENSIFY):
    out = [tuple(coords[0])]
    for i in range(1, len(coords)):
        a, b = coords[i - 1], coords[i]
        d = _dist(a, b)
        if d > step:
            k = int(d // step)
            for j in range(1, k + 1):
                t = j * step / d
                if t < 1.0 - 1e-9:
                    out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
        out.append(tuple(b))
    return out


def _boundary_loops(poly: Polygon):
    out = []
    for ring in [poly.exterior] + list(poly.interiors):
        g = ring.simplify(_SIMPLIFY)
        c = list(g.coords)
        if len(c) >= 4 and LineString(c).length > 1.0:
            out.append(_densify(c))
    return out


def _rotate_loop(loop, pt):
    c = loop[:-1] if loop[0] == loop[-1] else list(loop)
    i = min(range(len(c)), key=lambda k: (c[k][0] - pt[0]) ** 2 + (c[k][1] - pt[1]) ** 2)
    r = c[i:] + c[:i]
    r.append(r[0])
    return r


def _min_width_angle(poly: Polygon) -> float:
    hull = poly.convex_hull
    if hull.geom_type != "Polygon":
        return 0.0
    cs = list(hull.exterior.coords)[:-1]
    best, best_w = 0.0, float("inf")
    for i in range(len(cs)):
        ax, ay = cs[i]
        bx, by = cs[(i + 1) % len(cs)]
        th = math.atan2(by - ay, bx - ax)
        c, s = math.cos(-th), math.sin(-th)
        # ⚠ 旋轉 −th 之後的 y 是 s·x + c·y。舊版 `-s*x + c*y` 因 s = −sin(th)
        #   而量成「鏡射後」的寬度（旋轉矩形實測多算 95%）。五個情境全軸對齊
        #   所以封存結果幾乎不受影響，但等寬邊的 argmin 取捨會變。
        ys = [(s * x + c * y) for x, y in cs]
        w = max(ys) - min(ys)
        if w < best_w:
            best_w, best = w, th
    return best


# ── 條帶填充 ──────────────────────────────────────────────────────────
def _scan_rows(clip: Polygon, swath: float, pitch: float):
    """把 clip（= 這一層還沒被蓋到的責任區）鋪滿掃描線。

    位置：外側兩條各內縮 ``min(swath/2, W/2)``、中間間距 <= pitch
          ⇒ 最外條的刈幅剛好切齊 clip 邊界，不出界也不缺邊。
    裁切：用 clip 本身，再把每段**兩端各縮 swath/2**（圓端帽剛好補回來）。
    clip 窄到一條中線就夠時只鋪一條 —— 這就是 v1「來回走兩遍」的修法。
    """
    th = _min_width_angle(clip)
    rk = rotate(clip, -th, origin=(0.0, 0.0), use_radians=True)
    kx0, ky0, kx1, ky1 = rk.bounds
    W = ky1 - ky0
    e = min(swath * 0.5, W * 0.5)
    span = max(W - 2.0 * e, 0.0)
    if span <= 1e-9:
        ys = [(ky0 + ky1) * 0.5]
    else:
        k = max(1, math.ceil(span / pitch - 1e-9))
        ys = [ky0 + e + i * span / k for i in range(k + 1)]
    ci, si = math.cos(th), math.sin(th)

    def back(p):
        return (ci * p[0] - si * p[1], si * p[0] + ci * p[1])

    rows = []
    half = swath * 0.5
    for iy, y in enumerate(ys):
        inter = rk.intersection(LineString([(kx0 - 10.0, y), (kx1 + 10.0, y)]))
        for g in getattr(inter, "geoms", [inter]):
            if g.geom_type != "LineString" or g.length <= 1e-6:
                continue
            a, b = g.coords[0][0], g.coords[-1][0]
            if b - a <= swath:                      # 比刈幅短 ⇒ 一個點就蓋滿
                a = b = (a + b) * 0.5
            else:
                a, b = a + half, b - half
            rows.append((iy, a, b, y))
    return rows, back


def _cells(rows):
    """相鄰層且沿線方向有重疊的線段併成 cell（凹形／分岔的正確處理）。"""
    par = list(range(len(rows)))

    def find(a):
        while par[a] != a:
            par[a] = par[par[a]]
            a = par[a]
        return a

    for i, (iy, a0, a1, _a) in enumerate(rows):
        for j, (jy, b0, b1, _b) in enumerate(rows):
            if jy == iy + 1 and min(a1, b1) - max(a0, b0) > -1e-6:
                ra, rb = find(i), find(j)
                if ra != rb:
                    par[rb] = ra
    groups: dict[int, list[int]] = {}
    for i in range(len(rows)):
        groups.setdefault(find(i), []).append(i)
    return list(groups.values())


def _clip_for(comp: Polygon, area: Polygon, swath: float, pitch: float, depth: int):
    """這一層還沒被蓋到的責任區。

    depth 0：整個作業區（外圈的刈幅要切齊邊界）。
    depth d>=1：上一圈畫在 ``comp ⊕ pitch`` 的邊界上，刈幅往內蓋到
    ``(comp ⊕ pitch) ⊖ swath/2 = comp ⊕ (pitch − swath/2)``。
    pitch > swath/2 時那條前緣在 comp **外面** ⇒ 責任區要**外擴** g；反之內縮。
    所以兩種情形統一就是 ``comp.buffer(pitch − swath/2)``。

    ⚠ 我第一版把符號寫反（外擴寫成內縮），造成環狀 64.7 m 寬的漏掃帶
    （覆蓋率 94.5%），再被殘留補掃用 303 km 補回來 —— 帳面 467.5 km。
    """
    if depth <= 0:
        return area
    g = pitch - swath * 0.5 + _SAFETY
    grown = comp.buffer(g) if abs(g) > 1e-9 else comp
    left = _polys(grown)
    return unary_union(left) if left else None


def _strip_paths(clip, swath: float, pitch: float, cur):
    """回傳 (paths, 含轉場的總長, 新 cur)。paths 已交替往返串好。"""
    if clip is None or clip.is_empty:
        return [], 0.0, cur
    rows, back = _scan_rows(clip, swath, pitch)
    if not rows:
        return [], 0.0, cur
    pend = [sorted((rows[i] for i in cell), key=lambda r: (r[0], r[1]))
            for cell in _cells(rows)]
    paths, total, p = [], 0.0, cur
    while pend:
        k = min(range(len(pend)),
                key=lambda t: min(_dist(p, back((r[1], r[3]))) for r in pend[t]))
        rw = pend.pop(k)
        first, last = back((rw[0][1], rw[0][3])), back((rw[0][2], rw[0][3]))
        flip = _dist(p, last) < _dist(p, first)
        path = []
        for _iy, xa, xb, y in rw:
            p0, p1 = (back((xb, y)), back((xa, y))) if flip else \
                     (back((xa, y)), back((xb, y)))
            path += [p0, p1]
            flip = not flip
        total += _dist(p, path[0]) + _plen(path)
        paths.append(path)
        p = path[-1]
    return paths, total, p


# ── 純輪廓的長度估算（給 hybrid 決策用）──────────────────────────────
def _next_comps(comp, swath, pitch):
    steps = [pitch]
    if pitch > swath * 0.5:
        steps += [max(pitch * 0.6, swath * 0.5), swath * 0.5]
    for s in steps:
        nxt = _polys(_erode(comp, s))
        if nxt:
            return nxt
    return []


def _contour_len(comp: Polygon, swath: float, pitch: float, depth=0) -> float:
    if depth > 4000:
        return 0.0
    tot = sum(_plen(a) for a in _boundary_loops(comp))
    if not _polys(_erode(comp, swath * 0.5)):
        return tot
    for c in _next_comps(comp, swath, pitch):
        tot += _contour_len(c, swath, pitch, depth + 1)
    return tot


def _emit_loops(comp, cur, out):
    pend = _boundary_loops(comp)
    while pend:
        j = min(range(len(pend)),
                key=lambda k: min((p[0] - cur[0]) ** 2 + (p[1] - cur[1]) ** 2
                                  for p in pend[k]))
        lp = _rotate_loop(pend.pop(j), cur)
        out.append(lp)
        cur = lp[-1]
    return cur


# ── 三套產生器 ────────────────────────────────────────────────────────
def _gen_contour(area, swath, pitch, cur, out):
    """A：純輪廓螺旋（v1 行為）。"""
    def rec(comp, cur, depth):
        if depth > 4000:
            return cur
        cur = _emit_loops(comp, cur, out)
        if not _polys(_erode(comp, swath * 0.5)):
            return cur
        nxt = _next_comps(comp, swath, pitch)
        while nxt:
            i = min(range(len(nxt)), key=lambda k: nxt[k].distance(Point(cur)))
            cur = rec(nxt.pop(i), cur, depth + 1)
        return cur

    comps = _polys(_erode(area, swath * 0.5)) or [area]
    while comps:
        i = min(range(len(comps)), key=lambda k: comps[k].distance(Point(cur)))
        cur = rec(comps.pop(i), cur, 0)
    return cur


def _gen_hybrid(area, swath, pitch, cur, out):
    """B：每層每塊比長度，輪廓 vs 條帶取短的。"""
    def rec(comp, cur, depth):
        if depth > 4000:
            return cur
        clip = _clip_for(comp, area, swath, pitch, depth)
        sp, slen, _ = _strip_paths(clip, swath, pitch, cur)
        if sp and slen < _contour_len(comp, swath, pitch) * _SWITCH_MARGIN:
            for q in sp:
                out.append(q)
                cur = q[-1]
            return cur
        cur = _emit_loops(comp, cur, out)
        if not _polys(_erode(comp, swath * 0.5)):
            return cur
        nxt = _next_comps(comp, swath, pitch)
        while nxt:
            i = min(range(len(nxt)), key=lambda k: nxt[k].distance(Point(cur)))
            cur = rec(nxt.pop(i), cur, depth + 1)
        return cur

    comps = _polys(_erode(area, swath * 0.5)) or [area]
    while comps:
        i = min(range(len(comps)), key=lambda k: comps[k].distance(Point(cur)))
        cur = rec(comps.pop(i), cur, 0)
    return cur


def _gen_strips(area, swath, pitch, cur, out):
    """C：純內縮條帶（責任區就是整個作業區）。"""
    paths, _l, cur = _strip_paths(area, swath, pitch, cur)
    out.extend(paths)
    return cur


# ── 殘留補掃（與 v1 相同語意）─────────────────────────────────────────
def _chain_strips(segs, cur):
    if not segs:
        return []
    a, b = segs[0]
    flip = _dist(b, cur) < _dist(a, cur)
    path = []
    for a, b in segs:
        p0, p1 = (b, a) if flip else (a, b)
        path += [p0, p1]
        flip = not flip
    return path


def _raw_strips(poly: Polygon, pitch: float, th: float):
    rp = rotate(poly, -th, origin=(0.0, 0.0), use_radians=True)
    x0, y0, x1, y1 = rp.bounds
    segs, y = [], y0 + pitch / 2.0
    if y1 - y0 < pitch:
        y = (y0 + y1) / 2.0
    while y < y1:
        inter = rp.intersection(LineString([(x0 - 10.0, y), (x1 + 10.0, y)]))
        for g in getattr(inter, "geoms", [inter]):
            if g.geom_type == "LineString" and g.length > 1e-6:
                segs.append((tuple(g.coords[0]), tuple(g.coords[-1]), y))
        y += pitch
    ci, si = math.cos(th), math.sin(th)
    return [((ci * a[0] - si * a[1], si * a[0] + ci * a[1]),
             (ci * b[0] - si * b[1], si * b[0] + ci * b[1]))
            for a, b, _y in sorted(segs, key=lambda r: r[2])]


def _residual_patches(area: Polygon, loops, swath: float, pitch: float, cur):
    if not loops:
        return [], cur, 0.0
    swept = unary_union([LineString(lp).buffer(swath * 0.5, quad_segs=8)
                         for lp in loops if len(lp) >= 2])
    resid = area.difference(swept)
    thr = max(0.5 * swath * pitch, 200.0)
    comps = [p for p in _polys(resid) if p.area > thr]
    out, patched = [], 0.0
    while comps:
        i = min(range(len(comps)), key=lambda k: comps[k].distance(Point(cur)))
        c = comps.pop(i)
        segs = _raw_strips(c.buffer(min(pitch, swath) * 0.25), pitch,
                           _min_width_angle(c))
        if not segs:
            mrr = c.minimum_rotated_rectangle
            if mrr.geom_type != "Polygon":
                continue
            q = list(mrr.exterior.coords)[:-1]
            if len(q) < 4:
                continue
            k = max(range(4), key=lambda t: _dist(q[t], q[(t + 1) % 4]))
            a, b = q[k], q[(k + 1) % 4]
            mx, my = (a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0
            cx, cy = c.centroid.x, c.centroid.y
            segs = [((a[0] - mx + cx, a[1] - my + cy),
                     (b[0] - mx + cx, b[1] - my + cy))]
        path = _chain_strips(segs, cur)
        if len(path) >= 2:
            out.append(path)
            cur = path[-1]
            patched += c.area
    return out, cur, patched


def _run_strategy(gen, geoms, swath, pitch, start):
    atoms: list = []
    cur = (float(start[0]), float(start[1]))
    todo, patched = list(geoms), 0.0
    while todo:
        i = min(range(len(todo)), key=lambda k: todo[k].distance(Point(cur)))
        area = todo.pop(i)
        loops: list = []
        cur = gen(area, swath, pitch, cur, loops)
        extra, cur, pa = _residual_patches(area, loops, swath, pitch, cur)
        atoms.extend(loops)
        atoms.extend(extra)
        patched += pa
    return atoms, patched


# ── 串折線 + DP minimax 連續分割（與 v1 相同的分配器）────────────────
def _build_polyline(atoms):
    pts, dead = [], []
    for a in atoms:
        if len(a) < 2:
            continue
        if not pts:
            pts.append(a[0])
        elif _dist(pts[-1], a[0]) > 1e-6:
            pts.append(a[0])
            dead.append(True)
        for p in a[1:]:
            if _dist(pts[-1], p) <= 1e-9:
                continue
            pts.append(p)
            dead.append(False)
    cum = [0.0]
    for i in range(1, len(pts)):
        cum.append(cum[-1] + _dist(pts[i - 1], pts[i]))
    return pts, dead, cum


def _partition(pts, dead, cum, start, n):
    L = cum[-1]
    if L <= 0.0 or len(pts) < 2:
        return []

    def at(s):
        i = min(max(bisect_right(cum, s) - 1, 0), len(pts) - 2)
        seg = cum[i + 1] - cum[i]
        t = 0.0 if seg <= 0 else (s - cum[i]) / seg
        a, b = pts[i], pts[i + 1]
        return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)

    def fwd(s):
        i = min(max(bisect_right(cum, s) - 1, 0), len(dead) - 1)
        while i < len(dead) and dead[i]:
            s = cum[i + 1]
            i += 1
        return min(s, L)

    def bwd(s):
        j = min(max(bisect_left(cum, s) - 1, -1), len(dead) - 1)
        while j >= 0 and dead[j]:
            s = cum[j]
            j -= 1
        return max(s, 0.0)

    m = min(_MAX_CUTS, max(6 * n, 200))
    cands = {0.0, L}
    cands.update(L * k / m for k in range(1, m))
    for i, d in enumerate(dead):
        if d:
            cands.add(cum[i])
            cands.add(cum[i + 1])
    cand = sorted(cands)
    if len(cand) > _MAX_CUTS * 2:
        step = len(cand) / (_MAX_CUTS * 2)
        cand = [cand[min(int(k * step), len(cand) - 1)] for k in range(_MAX_CUTS * 2)]
        cand = sorted(set(cand + [0.0, L]))
    M = len(cand) - 1
    if M < n:
        n = max(M, 1)
    A = [fwd(s) for s in cand]
    B = [bwd(s) for s in cand]
    DA = [_dist(start, at(s)) for s in A]
    DB = [_dist(start, at(s)) for s in B]
    INF = float("inf")
    prev = [INF] * (M + 1)
    prev[0] = 0.0
    parents = []
    for _k in range(n):
        curr = [INF] * (M + 1)
        par = [0] * (M + 1)
        for i in range(1, M + 1):
            bi, dbi = B[i], DB[i]
            best, bj = INF, i - 1
            for j in range(i):
                pj = prev[j]
                if pj == INF:
                    continue
                aj = A[j]
                c = 0.0 if bi <= aj else (bi - aj) + DA[j] + dbi
                v = pj if pj > c else c
                if v < best:
                    best, bj = v, j
            curr[i], par[i] = best, bj
        prev = curr
        parents.append(par)
    i, cuts = M, []
    for k in range(n - 1, -1, -1):
        j = parents[k][i]
        cuts.append((A[j], B[i]))
        i = j
    cuts.reverse()

    def slice_(a, b):
        if b <= a + 1e-6:
            return []
        i0 = min(max(bisect_right(cum, a) - 1, 0), len(pts) - 2)
        i1 = min(max(bisect_left(cum, b) - 1, 0), len(pts) - 2)
        out = [at(a)]
        for k in range(i0 + 1, i1 + 1):
            out.append(pts[k])
        out.append(at(b))
        return out

    tracks = []
    for a, b in cuts:
        body = slice_(a, b)
        if len(body) < 2:
            continue
        t = [start] + body + [start]
        t = [p for k, p in enumerate(t) if k == 0 or _dist(p, t[k - 1]) > 1e-9]
        # ⚠ 去重會把「和 start 差 1e-13 的最後一點」當成重複而砍掉補上的 start，
        #   於是折線終點變成 2.27e-13 而不是 start（v1 同樣的碼有同樣的潛在問題，
        #   只是它的切點沒踩到）。這裡把兩端釘死。
        if len(t) >= 2:
            t[0] = t[-1] = start
            tracks.append(t)
    return tracks


# ── 對外介面 ──────────────────────────────────────────────────────────
_STRATS = (("contour", _gen_contour), ("hybrid", _gen_hybrid), ("strips", _gen_strips))


def plan(polys, n_uav, *, swath_m, pitch_m, start=(0.0, 0.0), debug=False,
         only=None):
    """自適應輪廓／條帶混合覆蓋。回傳每機一條折線（公尺，起訖都在 start）。

    ``only`` 只跑指定的一套策略（消融用），預設三套都跑取最好。
    """
    n_uav = max(int(n_uav), 1)
    swath, pitch = float(swath_m), float(pitch_m)
    src = [p for p in (polys or []) if p is not None and not p.is_empty]
    geoms = _polys(unary_union(src)) if src else []
    if not geoms or swath <= 0 or pitch <= 0:
        return ({}, []) if debug else []
    st = (float(start[0]), float(start[1]))
    work = unary_union(geoms)
    cands, info = [], {}
    for name, gen in _STRATS:
        if only and name != only:
            continue
        atoms, patched = _run_strategy(gen, geoms, swath, pitch, st)
        pts, dead, cum = _build_polyline(atoms)
        tr = _partition(pts, dead, cum, st, n_uav)
        if not tr:
            continue
        mx = max(_plen(t) for t in tr)
        swept = unary_union([LineString(t).buffer(swath * 0.5, quad_segs=8)
                             for t in tr])
        cov = work.intersection(swept).area / work.area if work.area else 0.0
        cands.append((mx, cov, name, tr))
        info[name] = dict(max_km=round(mx / 1000.0, 3),
                          body_km=round(cum[-1] / 1000.0, 2),
                          coverage=round(cov, 5),
                          patched_m2=round(patched, 1), n=len(tr))
    if not cands:
        return ({}, []) if debug else []
    # ⚠ 只用長度挑會讓組合策略「用少掃一點換短一點」。所以先過覆蓋率閘門：
    #   只有覆蓋率落在最佳者 0.1 個百分點以內的候選才有資格比長度。
    top = max(c[1] for c in cands)
    pool = [c for c in cands if c[1] >= top - _COV_TOL] or cands
    best = min(pool, key=lambda c: c[0])
    if debug:
        return {"strategy": best[2], "coverage_gate": round(top - _COV_TOL, 5),
                "candidates": info}, best[3]
    return best[3]


# ── 自檢：三個交付情境 + 消融 ─────────────────────────────────────────
if __name__ == "__main__":
    import sys

    # ⚠ 原本硬編 session scratchpad 的路徑（會消失），改成 repo 相對。
    _B = str(Path(__file__).resolve().parent)
    if _B not in sys.path:
        sys.path.insert(0, _B)
    from bench import boustrophedon, metrics, rect_xy      # noqa: E402
    import algo_spiral as _V1                              # noqa: E402

    _ALT, _HFOV = 140.0, 60.0
    _SW = 2 * _ALT * math.tan(math.radians(_HFOV) / 2)
    _PI = _SW * 0.7
    _L = Polygon([(-2500, -2000), (2500, -2000), (2500, 0),
                  (0, 0), (0, 2000), (-2500, 2000)])
    _CASES = [("(1) 5.0x4.0 km 矩形, n=8", [rect_xy(0, 0, 2500, 2000)], 8),
              ("(2) 四塊 1.25x4.0 km, n=8",
               [rect_xy(cx, 0, 625, 2000) for cx in (-2625, -875, 875, 2625)], 8),
              ("(3) L 形凹多邊形, n=4", [_L], 4)]
    _H = (f"{'方法':20s}{'台':>3s}{'總km':>8s}{'最長km':>8s}{'覆蓋':>8s}{'開銷':>7s}"
          f"{'重複':>7s}{'不均':>7s}{'掃描率':>9s}{'浪費':>8s}")

    def _show(nm, m):
        print(f"{nm:20s}{m['n_uav']:3d}{m['total_km']:8.1f}{m['max_km']:8.2f}"
              f"{m['coverage'] * 100:7.2f}%{m['overhead'] or 0:7.3f}"
              f"{m['redundancy'] or 0:7.3f}{m['balance'] or 0:7.3f}"
              f"{m['scan_rate']:9.2f}{(m['waste'] or 0) * 100:7.2f}%")

    print(f"刈幅 {_SW:.1f} m  圈距/條帶距 {_PI:.1f} m  速度 20 m/s\n")
    for _t, _p, _n in _CASES:
        print(f"{_t}   {unary_union(_p).area / 1e6:.2f} km²")
        print(_H)
        _d, _tr = plan(_p, _n, swath_m=_SW, pitch_m=_PI, debug=True)
        _show("spiral_v2_r1", metrics(_p, _tr, _SW))
        _show("spiral (v1)", metrics(_p, _V1.plan(_p, _n, swath_m=_SW, pitch_m=_PI), _SW))
        _show("boustrophedon", metrics(_p, boustrophedon(_p, _n, swath_m=_SW,
                                                         pitch_m=_PI), _SW))
        print(f"   選了 {_d['strategy']}（覆蓋率閘門 >= {_d['coverage_gate']}）")
        for _k, _v in _d["candidates"].items():
            print(f"      {_k:8s} 最長 {_v['max_km']:7.3f} km  本體 {_v['body_km']:7.2f} km"
                  f"  覆蓋 {_v['coverage']*100:6.3f}%  補掃 {_v['patched_m2']} m²")
        print()
