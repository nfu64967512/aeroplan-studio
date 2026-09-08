"""algo_spiral_v2.py — 向內螺旋 v2（審查者改進版）。

key = ``spiral_v2``。介面與 ``algo_spiral.plan`` 相同。

相對 v1 的三項改動（每一項都獨立可關，__main__ 有逐項消融）
------------------------------------------------------------
A. **退化核心改鋪條帶**（``core_fill``）
   v1 一路剝到最內圈，最後一圈變成 1055.5 x 55.5 m 的髮夾（周長 2222 m），
   卻只需要蓋掉 1054 x 54 m 的殘塊 —— 輪廓平行在核心是**結構性退化**。
   判準不是調出來的常數，而是幾何敘述：``erode(comp, 2*pitch)`` 為空
   ⇒「最多只剩兩層圈」⇒ 這一塊已經退化，此時才允許拿條帶跟圈比長度，
   短的贏。外圈完全不受影響（閘門為假），所以不會退化成純牛耕。

B. **精確 minimax 連續分割**（``exact_cut``）
   v1 的 DP 候選切點寫死 ``m = min(_MAX_CUTS, max(6*n, 200))`` = 200 個，
   **``_MAX_CUTS`` 是惰性參數**（我把它調到 1200 數字一動也不動）。
   200 個候選 ⇒ 176 km 上的切點解析度 880 m，實測 8 段弧長全是 880.04 的整數倍。
   v2 改成：對 minimax 值 T 二分搜尋 + 貪婪可行性檢查。
   可行性檢查為什麼是對的：成本 ``c(a,b) = (b'-a) + d(home,a') + d(home,b')``
   對 b **單調不減**（因為 b' 對 b 不減，而 ``d`` 對 b' 的變化率 |·| <= 1，
   兩者相加斜率 >= 0）⇒ 可行的 b 構成一個區間 ⇒ 貪婪取最大 b 是最佳的。
   收斂後再做「相鄰兩段等化」掃描（只接受不抬高全域最大值的移動）壓低 balance。

C. **多區塊走訪順序**（``area_tour``）
   v1 用「離目前位置最近」的貪婪，四塊長條會走成 Z 字。v2 改 NN + 2-opt，
   區塊間轉場（飛在作業區外、直接變成 waste）因此變短。

只用 shapely + 標準庫。不 import 本專案任何規劃器。
"""
from __future__ import annotations

import math
from bisect import bisect_left, bisect_right

from shapely.affinity import rotate
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union
from pathlib import Path

_MIN_AREA = 4.0
_DENSIFY = 60.0
_SIMPLIFY = 0.4
_EPS = 1e-9


# ── 幾何小工具（與 v1 相同語意）──────────────────────────────────────
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
    return poly.buffer(-d, quad_segs=16)


def _dist(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _plen(p) -> float:
    return sum(_dist(p[i - 1], p[i]) for i in range(1, len(p)))


def _densify(coords, step: float = _DENSIFY):
    out = [tuple(coords[0])]
    for i in range(1, len(coords)):
        a, b = coords[i - 1], coords[i]
        d = _dist(a, b)
        if d > step:
            k = int(d // step)
            for j in range(1, k + 1):
                t = j * step / d
                if t < 1.0 - _EPS:
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


# ── 條帶填充（改動 A 與殘留補掃共用）────────────────────────────────
def _fill_lines(target: Polygon, swath: float, pitch: float, cur):
    """用直條帶蓋滿 target。回傳 (atoms, end_point)。

    外側兩條的內縮取 min(swath/2, W/2)：這樣最外條的刈幅剛好切齊 target 邊界
    （條帶數最少且不留邊）。中間間距 <= pitch。
    """
    th = _min_width_angle(target)
    rp = rotate(target, -th, origin=(0.0, 0.0), use_radians=True)
    x0, y0, x1, y1 = rp.bounds
    W = y1 - y0
    e = min(swath * 0.5, W * 0.5)
    span = max(W - 2.0 * e, 0.0)
    n = 1 if span <= _EPS else int(math.ceil(span / pitch - 1e-9)) + 1
    ys = [(y0 + y1) * 0.5] if n <= 1 else [y0 + e + i * (span / (n - 1)) for i in range(n)]

    ci, si = math.cos(th), math.sin(th)
    back = lambda p: (ci * p[0] - si * p[1], si * p[0] + ci * p[1])   # noqa: E731

    rows = []
    for y in ys:
        inter = rp.intersection(LineString([(x0 - 10.0, y), (x1 + 10.0, y)]))
        segs = [(tuple(g.coords[0]), tuple(g.coords[-1]))
                for g in getattr(inter, "geoms", [inter])
                if g.geom_type == "LineString" and g.length > 1e-6]
        if segs:
            rows.append(sorted(segs, key=lambda s: s[0][0]))
    if not rows:
        return [], cur

    # 交替往返；同一列多段時沿掃描方向依序取，連接線若跑出 target 就斷成新 atom
    guard = target.buffer(max(pitch, swath) * 0.5 + 1.0)
    atoms, path = [], []
    flip = False
    prev = None
    for segs in rows:
        order = list(reversed(segs)) if flip else segs
        for a, b in order:
            p0, p1 = (b, a) if flip else (a, b)
            P0, P1 = back(p0), back(p1)
            if prev is not None and not guard.contains(LineString([prev, P0])):
                if len(path) >= 2:
                    atoms.append(path)
                path = []
            path += [P0, P1]
            prev = P1
        flip = not flip
    if len(path) >= 2:
        atoms.append(path)
    return atoms, (atoms[-1][-1] if atoms else cur)


# ── 螺旋圈生成（改動 A 在 _emit 內）──────────────────────────────────
def _ring_atoms(comp: Polygon, cur):
    pend = _boundary_loops(comp)
    out = []
    while pend:
        j = min(range(len(pend)),
                key=lambda k: min((p[0] - cur[0]) ** 2 + (p[1] - cur[1]) ** 2
                                  for p in pend[k]))
        lp = _rotate_loop(pend.pop(j), cur)
        out.append(lp)
        cur = lp[-1]
    return out, cur


def _chain_cost(atoms, cur):
    """atoms 串起來的總長（含 atom 之間的轉場）。"""
    tot, p = 0.0, cur
    for a in atoms:
        if len(a) < 2:
            continue
        tot += _dist(p, a[0]) + _plen(a)
        p = a[-1]
    return tot


def _emit(comp: Polygon, swath: float, pitch: float, cur, depth: int,
          core_fill: bool = True):
    """畫出 comp 這一層（含往內遞迴）。回傳 (atoms, end_point)。"""
    if depth > 4000:
        return [], cur
    rings, rcur = _ring_atoms(comp, cur)
    if not _polys(_erode(comp, swath * 0.5)):        # 這一圈自己就蓋滿 comp
        return rings, rcur

    # 改動 A 的閘門：depth 0 不准換（那一層的條帶會伸出作業區 ⇒ waste），
    # 且只在「最多剩兩層圈」時才准把圈換成條帶。閘門存在的理由是
    # 「短的贏」是**局部**判斷：換成條帶會改變離開點，可能反而拉長下游轉場
    # （實測 offset-home 情境全開閘門時掃描率 44.45 -> 43.38）。
    # 敏感度：閘門取 2/3/4/6*pitch 或全開，11 個情境的掃描率差 <= 1.3%。
    degenerate = core_fill and depth >= 1 and not _polys(_erode(comp, 2.0 * pitch))

    steps = [pitch]
    if pitch > swath * 0.5:
        steps += [max(pitch * 0.6, swath * 0.5), swath * 0.5]
    nxt: list[Polygon] = []
    for s in steps:
        nxt = _polys(_erode(comp, s))
        if nxt:
            break
    ring_branch = list(rings)
    ring_cur = rcur
    if nxt:
        sub, ring_cur = _walk(nxt, swath, pitch, rcur, depth + 1, core_fill)
        ring_branch += sub

    if not degenerate:
        return ring_branch, ring_cur

    # 條帶分支：祖先那一圈的刈幅只到 comp 外緣 (pitch - swath/2)，所以
    # 還沒被蓋到的是 comp 往外 buffer 這麼多的那一塊。
    grow = max(pitch - swath * 0.5, 0.0)
    tgt = comp.buffer(grow) if grow > 0 else comp
    fill, fcur = _fill_lines(tgt, swath, pitch, cur)
    if not fill:
        return ring_branch, ring_cur
    if _chain_cost(fill, cur) < _chain_cost(ring_branch, cur):
        return fill, fcur
    return ring_branch, ring_cur


def _walk(comps, swath, pitch, cur, depth=0, core_fill=True):
    comps = list(comps)
    out = []
    while comps:
        i = min(range(len(comps)), key=lambda k: comps[k].distance(Point(cur)))
        a, cur = _emit(comps.pop(i), swath, pitch, cur, depth, core_fill)
        out += a
    return out, cur


def _spiral_loops(area: Polygon, swath, pitch, cur, core_fill=True):
    comps = _polys(_erode(area, swath * 0.5)) or [area]
    return _walk(comps, swath, pitch, cur, 0, core_fill)


# ── 殘留補掃（與 v1 同角色，改用 _fill_lines）─────────────────────────
def _residual_patches(area: Polygon, loops, swath: float, pitch: float, cur):
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
        fill, cur2 = _fill_lines(c.buffer(min(pitch, swath) * 0.25), swath, pitch, cur)
        if not fill:
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
            fill = [[(a[0] - mx + cx, a[1] - my + cy), (b[0] - mx + cx, b[1] - my + cy)]]
            cur2 = fill[-1][-1]
        out += fill
        cur = cur2
        patched += c.area
    return out, cur, patched


# ── 串成單一折線 ─────────────────────────────────────────────────────
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
            if _dist(pts[-1], p) <= _EPS:
                continue
            pts.append(p)
            dead.append(False)
    cum = [0.0]
    for i in range(1, len(pts)):
        cum.append(cum[-1] + _dist(pts[i - 1], pts[i]))
    return pts, dead, cum


# ── 改動 B：精確 minimax 連續分割 ────────────────────────────────────
class _Curve:
    def __init__(self, pts, dead, cum, start):
        self.pts, self.dead, self.cum, self.start = pts, dead, cum, start
        self.L = cum[-1]

    def at(self, s):
        i = min(max(bisect_right(self.cum, s) - 1, 0), len(self.pts) - 2)
        seg = self.cum[i + 1] - self.cum[i]
        t = 0.0 if seg <= 0 else (s - self.cum[i]) / seg
        a, b = self.pts[i], self.pts[i + 1]
        return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)

    def fwd(self, s):                         # 跳過起點側的轉場段
        i = min(max(bisect_right(self.cum, s) - 1, 0), len(self.dead) - 1)
        while i < len(self.dead) and self.dead[i]:
            s = self.cum[i + 1]
            i += 1
        return min(s, self.L)

    def bwd(self, s):                         # 跳過終點側的轉場段
        j = min(max(bisect_left(self.cum, s) - 1, -1), len(self.dead) - 1)
        while j >= 0 and self.dead[j]:
            s = self.cum[j]
            j -= 1
        return max(s, 0.0)

    def d0(self, s):
        return _dist(self.start, self.at(s))

    def cost(self, a, b):
        """一台飛 [a,b] 的總航程（去程 + 弧 + 回程），轉場端點已剝掉。"""
        aa = self.fwd(a)
        bb = max(self.bwd(b), aa)
        if bb <= aa + 1e-6:
            return 0.0
        return (bb - aa) + self.d0(aa) + self.d0(bb)


def _greedy(cv: _Curve, n: int, T: float):
    """在 minimax 上限 T 下貪婪吃最長的弧。回傳切點或 None（不可行）。"""
    cuts, s = [], 0.0
    for _ in range(n):
        if s >= cv.L - 1e-6:
            cuts.append((s, s))
            continue
        a = cv.fwd(s)
        if a >= cv.L - 1e-6:
            cuts.append((s, cv.L))
            s = cv.L
            continue
        if cv.cost(a, cv.L) <= T:
            cuts.append((s, cv.L))
            s = cv.L
            continue
        lo, hi = a, cv.L                      # cost(a, ·) 對 b 單調不減
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if cv.cost(a, mid) <= T:
                lo = mid
            else:
                hi = mid
        if lo <= s + 1e-6:
            return None
        cuts.append((s, lo))
        s = lo
    return cuts if s >= cv.L - 1e-3 else None


def _equalize(cv: _Curve, cuts, rounds: int = 400):
    """相鄰兩段等化：只接受不抬高兩段中較大者的移動。"""
    b = [c[1] for c in cuts]
    for _ in range(rounds):
        moved = False
        for i in range(len(b) - 1):
            lo = b[i - 1] if i > 0 else 0.0
            hi = b[i + 1]
            c0 = cv.cost(lo, b[i])
            c1 = cv.cost(b[i], hi)
            before = max(c0, c1)
            x0, x1 = lo, hi
            for _ in range(40):               # cost(lo,·) 增、cost(·,hi) 減
                m = 0.5 * (x0 + x1)
                if cv.cost(lo, m) < cv.cost(m, hi):
                    x0 = m
                else:
                    x1 = m
            m = 0.5 * (x0 + x1)
            if max(cv.cost(lo, m), cv.cost(m, hi)) < before - 1e-6:
                b[i] = m
                moved = True
        if not moved:
            break
    out, prev = [], 0.0
    for x in b:
        out.append((prev, x))
        prev = x
    return out


def _slice(cv: _Curve, a, b):
    aa = cv.fwd(a)
    bb = max(cv.bwd(b), aa)
    if bb <= aa + 1e-6:
        return []
    i0 = min(max(bisect_right(cv.cum, aa) - 1, 0), len(cv.pts) - 2)
    i1 = min(max(bisect_left(cv.cum, bb) - 1, 0), len(cv.pts) - 2)
    out = [cv.at(aa)]
    for k in range(i0 + 1, i1 + 1):
        out.append(cv.pts[k])
    out.append(cv.at(bb))
    return out


def exact_partition(pts, dead, cum, start, n):
    if cum[-1] <= 0.0 or len(pts) < 2:
        return []
    cv = _Curve(pts, dead, cum, start)
    lo, hi = 0.0, cv.cost(0.0, cv.L)
    best = None
    for _ in range(60):                        # 二分 minimax 值
        mid = 0.5 * (lo + hi)
        g = _greedy(cv, n, mid)
        if g is None:
            lo = mid
        else:
            best, hi = g, mid
    if best is None:
        best = [(cv.L * i / n, cv.L * (i + 1) / n) for i in range(n)]
    best = _equalize(cv, best)
    tracks = []
    for a, b in best:
        body = _slice(cv, a, b)
        if len(body) < 2:
            continue
        t = [start] + body + [start]
        t = [p for k, p in enumerate(t) if k == 0 or _dist(p, t[k - 1]) > _EPS]
        if len(t) >= 2:
            tracks.append(t)
    return tracks


# ── 改動 C：多區塊走訪順序（NN + 2-opt）──────────────────────────────
def _order_areas(geoms, start):
    if len(geoms) <= 2:
        return list(geoms)
    reps = [g.representative_point() for g in geoms]
    idx, cur, left = [], Point(start), list(range(len(geoms)))
    while left:
        j = min(left, key=lambda k: geoms[k].distance(cur))
        idx.append(j)
        left.remove(j)
        cur = reps[j]

    def d(i, j):
        return geoms[i].distance(reps[j])

    def tour_len(o):
        t = geoms[o[0]].distance(Point(start))
        for i in range(len(o) - 1):
            t += d(o[i + 1], o[i])
        return t

    improved, best = True, idx
    while improved:
        improved = False
        for i in range(len(best) - 1):
            for j in range(i + 2, len(best)):
                cand = best[:i + 1] + best[i + 1:j + 1][::-1] + best[j + 1:]
                if tour_len(cand) < tour_len(best) - 1e-6:
                    best, improved = cand, True
    return [geoms[k] for k in best]


# ── 對外介面 ─────────────────────────────────────────────────────────
def plan(polys, n_uav, *, swath_m, pitch_m, start=(0.0, 0.0), debug=False,
         core_fill=True, exact_cut=True, area_tour=True):
    n_uav = max(int(n_uav), 1)
    swath, pitch = float(swath_m), float(pitch_m)
    src = [p for p in (polys or []) if p is not None and not p.is_empty]
    geoms = _polys(unary_union(src)) if src else []
    if not geoms or swath <= 0 or pitch <= 0:
        return ({}, []) if debug else []

    start = (float(start[0]), float(start[1]))
    atoms: list = []
    cur = start
    patched = 0.0
    if area_tour:
        todo = _order_areas(geoms, start)
        for a in todo:
            loops, cur = _spiral_loops(a, swath, pitch, cur, core_fill)
            extra, cur, pa = _residual_patches(a, loops, swath, pitch, cur)
            atoms += loops + extra
            patched += pa
    else:
        todo = list(geoms)
        while todo:
            i = min(range(len(todo)), key=lambda k: todo[k].distance(Point(cur)))
            a = todo.pop(i)
            loops, cur = _spiral_loops(a, swath, pitch, cur, core_fill)
            extra, cur, pa = _residual_patches(a, loops, swath, pitch, cur)
            atoms += loops + extra
            patched += pa

    pts, dead, cum = _build_polyline(atoms)
    if exact_cut:
        tracks = exact_partition(pts, dead, cum, start, n_uav)
    else:
        import algo_spiral as _v1
        tracks = _v1._partition(pts, dead, cum, start, n_uav)
    if debug:
        return {"loops": len(atoms), "spiral_km": round(cum[-1] / 1000.0, 2),
                "patched_m2": round(patched, 1)}, tracks
    return tracks


# ── 自檢 / 消融 ──────────────────────────────────────────────────────
if __name__ == "__main__":
    import sys
    import time

    # ⚠ 原本硬編 session scratchpad 的路徑（會消失），改成 repo 相對。
    BENCH = str(Path(__file__).resolve().parent)
    if BENCH not in sys.path:
        sys.path.insert(0, BENCH)
    import algo_spiral as v1                                   # noqa: E402
    from bench import boustrophedon, metrics, rect_xy          # noqa: E402

    ALT, HFOV = 140.0, 60.0
    SW = 2 * ALT * math.tan(math.radians(HFOV) / 2)
    PI = SW * 0.7

    L_SHAPE = Polygon([(-2500, -2000), (2500, -2000), (2500, 0),
                       (0, 0), (0, 2000), (-2500, 2000)])
    CASES = [
        ("(1) 5.0x4.0 km 矩形, n=8", [rect_xy(0, 0, 2500, 2000)], 8),
        ("(2) 四塊 1.25x4.0 km, n=8",
         [rect_xy(cx, 0, 625, 2000) for cx in (-2625, -875, 875, 2625)], 8),
        ("(3) L 形凹多邊形, n=4", [L_SHAPE], 4),
    ]
    hdr = (f"{'方法':22s}{'台':>3s}{'總km':>8s}{'最長km':>9s}{'覆蓋':>8s}"
           f"{'開銷':>7s}{'重複':>7s}{'不均':>7s}{'掃描率':>9s}{'區外浪費':>9s}")

    def show(name, m, t=None):
        print(f"{name:22s}{m['n_uav']:3d}{m['total_km']:8.1f}{m['max_km']:9.2f}"
              f"{m['coverage'] * 100:7.2f}%{m['overhead'] or 0:7.3f}"
              f"{m['redundancy'] or 0:7.3f}{m['balance'] or 0:7.3f}"
              f"{m['scan_rate']:9.2f}{(m['waste'] or 0) * 100:8.2f}%"
              + (f"  {t * 1000:.0f} ms" if t else ""))

    print(f"刈幅 {SW:.1f} m  圈距/條帶距 {PI:.1f} m  速度 20 m/s\n")
    for title, polys, n in CASES:
        print(f"{title}   {unary_union(polys).area / 1e6:.2f} km²")
        print(hdr)
        t0 = time.perf_counter()
        d2, tr2 = plan(polys, n, swath_m=SW, pitch_m=PI, debug=True)
        t2 = time.perf_counter() - t0
        t0 = time.perf_counter()
        d1, tr1 = v1.plan(polys, n, swath_m=SW, pitch_m=PI, debug=True)
        t1 = time.perf_counter() - t0
        show("spiral_v2", metrics(polys, tr2, SW), t2)
        show("spiral (v1)", metrics(polys, tr1, SW), t1)
        show("boustrophedon(bench)", metrics(polys, boustrophedon(
            polys, n, swath_m=SW, pitch_m=PI), SW))
        # 逐項消融
        for flag in ("core_fill", "exact_cut", "area_tour"):
            kw = {flag: False}
            show(f"  v2 關掉 {flag}", metrics(
                polys, plan(polys, n, swath_m=SW, pitch_m=PI, **kw), SW))
        print(f"   v2 atoms {d2['loops']} 本體 {d2['spiral_km']} km 補掃 "
              f"{d2['patched_m2']} m² | v1 atoms {d1['loops']} 本體 "
              f"{d1['spiral_km']} km 補掃 {d1['patched_m2']} m²\n")
