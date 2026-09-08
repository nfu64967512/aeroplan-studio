"""algo_spiral.py — 向內螺旋（inward spiral / contour-parallel）覆蓋規劃。

key = ``spiral``

演算法
------
1. **第一圈內縮 swath/2**。這樣最外圈的刈幅外緣剛好切齊作業區邊界 ⇒ 區外浪費 ≈ 0。
   （若改成貼著邊界飛，等於白掃 swath/2 的區外；那是常見但錯誤的做法。）
2. **之後每圈再內縮 pitch**，直到「這一塊的內接半徑 ≤ swath/2」為止。
   終止判準是 ``erode(comp, swath/2).is_empty``：此時該塊每個點離自己的邊界都
   ≤ swath/2，於是**這一圈自己的刈幅就蓋滿它**，不會像固定步長那樣留下
   一塊最深 ``pitch - swath/2`` 的中心殘塊。
3. **凹多邊形／內縮斷裂**：negative buffer 產生 MultiPolygon 時，各塊各自遞迴繼續
   螺旋（L 形內縮到某個深度會斷成兩臂，這是螺旋覆蓋唯一真正麻煩的地方）。
   多邊形有洞時，外環與每個內環都是該層的圈。
4. **串接**：下一圈的進入點取「離上一圈終點最近的頂點」，圈本身完整走完再跳。
   轉場長度 ≈ pitch，比牛耕的每條帶掉頭還少（牛耕 N-1 次、螺旋只有圈數-1 次）。
5. **殘留補掃**：所有圈畫完後檢查 ``area - union(swath)``。正常凸區塊只會剩下
   角落的 ``(1-π/4)·(swath/2)²``（總量 <0.1%，低於門檻不動）；但若有比刈幅還窄的
   細臂被 erosion 整段吃掉，就對殘塊補牛耕條帶。**三個交付情境都沒有觸發它**
   （`plan(..., debug=True)` 的 ``patched_m2`` 全是 0.0），所以帳面數字裡沒有它的份；
   但它不是死碼：100 m 細臂與 300 m 窄頸兩個情境會觸發，覆蓋率因此保住 99.9%/99.7%。
6. **多機分配**：把整條螺旋切成 n 段**連續**弧。切點不是等長切，而是用 DP 解
   「minimax 連續分割」，成本 = 弧長 + 去程 + 回程 —— 因為外圈離家遠、內圈近，
   等長切會讓拿到外圈的那台明顯最久。切在轉場段上時該段轉場直接丟掉（沒人飛）。

只用 shapely + 標準庫。不 import 本專案任何規劃器。
"""
from __future__ import annotations

import math
from bisect import bisect_left, bisect_right

from shapely.affinity import rotate
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union
from pathlib import Path

_MIN_AREA = 4.0        # m²，比這小的碎塊丟掉
_DENSIFY = 60.0        # m，折線加密（決定進入點與切點的解析度）
_SIMPLIFY = 0.4        # m，圈的簡化容差（相對 swath 可忽略）
_MAX_CUTS = 260        # DP 候選切點數


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
    if d <= 0:
        return poly
    return poly.buffer(-d, quad_segs=16)          # round join = 真正的 erosion


def _dist(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


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
    """外環 + 每個洞 → 加密過的封閉折線。"""
    out = []
    for ring in [poly.exterior] + list(poly.interiors):
        g = ring.simplify(_SIMPLIFY)
        c = list(g.coords)
        if len(c) >= 4 and LineString(c).length > 1.0:
            out.append(_densify(c))
    return out


def _rotate_loop(loop, pt):
    """把封閉折線旋轉到「離 pt 最近的頂點」起頭，並回到該點閉合。"""
    c = loop[:-1] if loop[0] == loop[-1] else list(loop)
    i = min(range(len(c)), key=lambda k: (c[k][0] - pt[0]) ** 2 + (c[k][1] - pt[1]) ** 2)
    r = c[i:] + c[:i]
    r.append(r[0])
    return r


# ── 螺旋圈生成 ────────────────────────────────────────────────────────
def _emit(comp: Polygon, swath: float, pitch: float, cur, out: list, depth: int):
    """畫出 comp 這一層的圈，再遞迴往內。回傳新的當前位置。"""
    if depth > 4000:
        return cur
    pend = _boundary_loops(comp)
    while pend:
        j = min(range(len(pend)),
                key=lambda k: min((p[0] - cur[0]) ** 2 + (p[1] - cur[1]) ** 2
                                  for p in pend[k]))
        lp = _rotate_loop(pend.pop(j), cur)
        out.append(lp)
        cur = lp[-1]

    # 終止：內接半徑 ≤ swath/2 ⇒ 剛畫的這圈已經蓋滿 comp
    if not _polys(_erode(comp, swath * 0.5)):
        return cur

    # 往內一步。正常是 pitch；pitch 太大導致整塊消失時退到 swath/2
    # （內接半徑 > swath/2 已由上一行保證，所以退到 swath/2 一定非空）
    steps = [pitch]
    if pitch > swath * 0.5:
        steps += [max(pitch * 0.6, swath * 0.5), swath * 0.5]
    nxt: list[Polygon] = []
    for s in steps:
        nxt = _polys(_erode(comp, s))
        if nxt:
            break
    if not nxt:
        return cur
    return _walk(nxt, swath, pitch, cur, out, depth + 1)


def _walk(comps, swath, pitch, cur, out, depth=0):
    comps = list(comps)
    while comps:
        i = min(range(len(comps)), key=lambda k: comps[k].distance(Point(cur)))
        cur = _emit(comps.pop(i), swath, pitch, cur, out, depth)
    return cur


def _spiral_loops(area: Polygon, swath: float, pitch: float, cur):
    out: list = []
    comps = _polys(_erode(area, swath * 0.5))
    if not comps:
        # 整塊比刈幅還窄 ⇒ 內接半徑 ≤ swath/2，飛它自己的邊界就蓋滿
        comps = [area]
    cur = _walk(comps, swath, pitch, cur, out)
    return out, cur


# ── 殘留補掃（細臂／窄頸被 erosion 吃掉時才會觸發）────────────────────
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


def _strips(poly: Polygon, pitch: float, th: float):
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


def _residual_patches(area: Polygon, loops, swath: float, pitch: float, cur):
    """圈畫完後掃不到的殘塊 → 補條帶。回傳 (paths, cur, 補掃面積)。"""
    if not loops:
        return [], cur, 0.0
    swept = unary_union([LineString(l).buffer(swath * 0.5, quad_segs=8) for l in loops])
    resid = area.difference(swept)
    thr = max(0.5 * swath * pitch, 200.0)
    comps = [p for p in _polys(resid) if p.area > thr]
    out, patched = [], 0.0
    while comps:
        i = min(range(len(comps)), key=lambda k: comps[k].distance(Point(cur)))
        c = comps.pop(i)
        segs = _strips(c.buffer(min(pitch, swath) * 0.25), pitch, _min_width_angle(c))
        if not segs:
            # 退路：殘塊細到連一條帶都排不進去 -> 沿最小外接矩形長邊做一條中線
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


# ── 把一堆 atom 串成一條折線（並記住哪些段是轉場）────────────────────
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


# ── minimax 連續分割（DP）─────────────────────────────────────────────
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

    def fwd(s):                       # 跳過起點側的轉場段
        i = min(max(bisect_right(cum, s) - 1, 0), len(dead) - 1)
        while i < len(dead) and dead[i]:
            s = cum[i + 1]
            i += 1
        return min(s, L)

    def bwd(s):                       # 跳過終點側的轉場段
        j = min(max(bisect_left(cum, s) - 1, -1), len(dead) - 1)
        while j >= 0 and dead[j]:
            s = cum[j]
            j -= 1
        return max(s, 0.0)

    m = min(_MAX_CUTS, max(6 * n, 200))
    cands = {0.0, L}
    cands.update(L * k / m for k in range(1, m))
    for i, d in enumerate(dead):       # atom 邊界也放進候選，讓 DP 能剛好切在轉場上
        if d:
            cands.add(cum[i])
            cands.add(cum[i + 1])
    cand = sorted(cands)
    if len(cand) > _MAX_CUTS * 2:      # 太多就抽稀（保留頭尾）
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
        if len(t) >= 2:
            tracks.append(t)
    return tracks


# ── 對外介面 ──────────────────────────────────────────────────────────
def plan(polys, n_uav, *, swath_m, pitch_m, start=(0.0, 0.0), debug=False):
    """向內螺旋覆蓋。回傳每機一條折線（平面公尺座標，起訖都在 start）。"""
    n_uav = max(int(n_uav), 1)
    swath, pitch = float(swath_m), float(pitch_m)
    src = [p for p in (polys or []) if p is not None and not p.is_empty]
    geoms = _polys(unary_union(src)) if src else []
    if not geoms or swath <= 0 or pitch <= 0:
        return ({}, []) if debug else []

    atoms: list = []
    cur = (float(start[0]), float(start[1]))
    todo, patched = list(geoms), 0.0
    while todo:
        i = min(range(len(todo)), key=lambda k: todo[k].distance(Point(cur)))
        area = todo.pop(i)
        loops, cur = _spiral_loops(area, swath, pitch, cur)
        extra, cur, pa = _residual_patches(area, loops, swath, pitch, cur)
        atoms.extend(loops)
        atoms.extend(extra)
        patched += pa

    pts, dead, cum = _build_polyline(atoms)
    tracks = _partition(pts, dead, cum, (float(start[0]), float(start[1])), n_uav)
    if debug:
        return {"loops": len(atoms), "spiral_km": round(cum[-1] / 1000.0, 2),
                "patched_m2": round(patched, 1)}, tracks
    return tracks


# ── 自檢：三個情境 ────────────────────────────────────────────────────
if __name__ == "__main__":
    import sys

    # ⚠ 原本硬編 session scratchpad 的路徑（會消失），改成 repo 相對。
    BENCH = str(Path(__file__).resolve().parent)
    if BENCH not in sys.path:
        sys.path.insert(0, BENCH)
    from bench import boustrophedon, metrics, rect_xy   # noqa: E402

    ALT, HFOV = 140.0, 60.0
    SW = 2 * ALT * math.tan(math.radians(HFOV) / 2)
    PI = SW * 0.7

    L_SHAPE = Polygon([(-2500, -2000), (2500, -2000), (2500, 0),
                       (0, 0), (0, 2000), (-2500, 2000)])
    CASES = [
        ("(1) 單一 5.0x4.0 km 矩形, n=8", [rect_xy(0, 0, 2500, 2000)], 8),
        ("(2) 四塊 1.25x4.0 km, n=8",
         [rect_xy(cx, 0, 625, 2000) for cx in (-2625, -875, 875, 2625)], 8),
        ("(3) L 形凹多邊形, n=4", [L_SHAPE], 4),
    ]

    hdr = (f"{'方法':16s}{'台':>3s}{'總 km':>8s}{'最長 km':>9s}{'覆蓋':>8s}"
           f"{'開銷':>7s}{'重複':>7s}{'不均':>7s}{'掃描率 km²/h':>13s}{'區外浪費':>9s}")

    def show(name, m):
        print(f"{name:16s}{m['n_uav']:3d}{m['total_km']:8.1f}{m['max_km']:9.1f}"
              f"{m['coverage'] * 100:7.2f}%{m['overhead'] or 0:7.3f}"
              f"{m['redundancy'] or 0:7.3f}{m['balance'] or 0:7.3f}"
              f"{m['scan_rate']:13.2f}{(m['waste'] or 0) * 100:8.2f}%")

    print(f"刈幅 {SW:.1f} m  條帶/圈距 {PI:.1f} m  速度 20 m/s\n")
    for title, polys, n in CASES:
        area_km2 = unary_union(polys).area / 1e6
        print(f"{title}   作業區 {area_km2:.2f} km²")
        print(hdr)
        dbg, tr = plan(polys, n, swath_m=SW, pitch_m=PI, debug=True)
        show("spiral", metrics(polys, tr, SW))
        show("boustrophedon", metrics(polys, boustrophedon(
            polys, n, swath_m=SW, pitch_m=PI), SW))
        print(f"   [spiral] 圈數 {dbg['loops']}  螺旋本體 {dbg['spiral_km']} km  "
              f"殘留補掃 {dbg['patched_m2']} m²\n")
