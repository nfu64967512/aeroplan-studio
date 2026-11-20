"""牛耕式 v2 —— key = ``bous_variants_v2``（審查者版，對 ``algo_bous_variants`` 的修正）

介面與 v1 完全相同：``plan(polys, n_uav, *, swath_m, pitch_m, start)``。

v1 已經做對的（**不動**，直接沿用同樣的做法）：掃描方向以最小寬度為起點、條帶交替
往返、以刈幅內縮均分佈點（兩端與刈幅切齊）、牛耕式單元分解、DP 精確 min-max、
子段切點、代價矩陣用真實折線長。這三個修正只針對實測抓到的三個缺口。

修正 1（**主要**）：**條帶端點取整條刈幅帶的跨距，不是中心線的跨距**
    v1 用「中心線 ∩ 多邊形」定條帶端點。斜邊上中心線構不到、但刈幅帶構得到的那一
    小塊因此變成漏掃碎屑，`_rows` 的驗證補線再為每一塊插一條**橫跨整個 bounding
    box** 的掃描線。實測（梯形 6.0x(0.8~2.4) km、θ=7.6°、n=4）：名目漏掃只有
    **2,246 m²（面積的 0.023%）**，v1 為它多鋪了 **43 km 條帶**，而且分成
    **14 個孤立碎片** ⇒ cell 串接得專程飛過去，轉場暴增到 **53.7 km**。
    v2 把 `_cut` 改成與 ``box(y-swath/2, y+swath/2)`` 相交後取 x 範圍，碎屑
    **從源頭就不會產生**：條帶數 35 → 21，總航程 142.1 → 103.3 km，
    而覆蓋率**兩者都是 100.0000%**（不是拿覆蓋率換的）。

修正 2：**補線退回成保險絲**
    修正 1 之後補線幾乎不再觸發；門檻維持 v1 的 `2e-4`（覆蓋率標準完全不放寬），
    補線本身則改成只補漏掃塊自己的 x 窗，避免再出現「一塊碎屑換一條全寬條帶」。

修正 3：**方向搜尋不再只看寬度**
    v1 候選 = 凸包邊方向且寬度在最小值 +3% 內。寬度只決定**條帶數**，決定不了
    航跡長度：十字形 θ=0（寬 5400）比 θ=45（寬 5091，v1 的唯一候選群）好 7.7%，
    但 5400/5091 = 1.061 > 1.03 ⇒ 結構上看不到。v2 改成兩階段：
      A 階段 粗掃全角度（含凸包邊方向與 ±180 相位），用**單機巡迴長度**代理排序
             —— 這個代理不用做 DP，實測 0.02 ms，是完整評估的三千分之一；
      B 階段 前 K 名才做完整的 DP 分配 + 真實目標函數決勝。
    多連通塊仍走逐塊座標下降（對區塊數線性）。

修正 4（工程）：代價矩陣與 min-max DP 改用 numpy 對角線推進，
    O(S²) 的部分從 36 ms + 31 ms 降到個位數 ms，修正 3 的加寬搜尋才付得起。

沒有改的（v1 的已知限制照舊，見審查報告）：單元分解仍是「全域掃描線格網 + 分組」
不是正統臨界點分解；cell / 區塊排序仍是貪婪最近鄰沒有 2-opt；min-max DP 仍只在
「固定條帶序列的連續切分」子空間內精確。
"""
from __future__ import annotations
from pathlib import Path

import math

import numpy as np
from shapely import affinity
from shapely.geometry import LineString, MultiPolygon, Polygon, box
from shapely.ops import unary_union

__all__ = ["plan", "plan_contiguous", "plan_interleaved", "plan_axis_aligned",
           "min_width_angle"]

_EPS = 1e-9
_TARGET_PIECES = 300
_MIN_PIECE_M = 250.0
_MAX_REPAIR = 3
_COV_MIN = 0.999                 # 可行性閘門（與 v1 相同）
_COV_TOL = 2e-4                  # 補線目標：與 v1 **完全相同**的覆蓋率標準
_SLIVER_TOL = 1e-6
_BAND_SPAN = True         # 修正 1：條帶端點取整條刈幅帶的 x 範圍
_REPAIR_MAX_M_PER_M2 = float('inf')   # 不用成本上限換覆蓋率（見說明）
_COARSE_STEP_DEG = 15.0          # 修正 3：A 階段粗掃步長（0..360，相位有意義）
_REFINE_DEG = (5.0, 1.5)         # A 階段在最佳點附近的局部細化
_TOPK = 3                        # B 階段完整評估的候選數
_MAX_DIR_EVALS = 24              # 多區塊座標下降的完整評估上限（同 v1）


# ── 幾何小工具（與 v1 相同）────────────────────────────────────────────
def _parts(polys) -> list[Polygon]:
    if isinstance(polys, (Polygon, MultiPolygon)):
        polys = [polys]
    geoms = [g for g in polys if g is not None and not g.is_empty]
    if not geoms:
        return []
    u = unary_union([g.buffer(0) for g in geoms])
    return [g for g in getattr(u, "geoms", [u])
            if g.geom_type == "Polygon" and g.area > 0]


def _edge_widths(poly: Polygon) -> list[tuple[float, float]]:
    cs = list(poly.convex_hull.exterior.coords)[:-1]
    out = []
    for i in range(len(cs)):
        ax, ay = cs[i]
        bx, by = cs[(i + 1) % len(cs)]
        if math.hypot(bx - ax, by - ay) < _EPS:
            continue
        th = math.atan2(by - ay, bx - ax)
        c, s = math.cos(-th), math.sin(-th)
        ys = [(s * x + c * y) for x, y in cs]
        out.append((max(ys) - min(ys), th))
    return out or [(0.0, 0.0)]


def _merge(spans, tol=1e-6):
    """把同一列的區間合併（刈幅帶與補線都可能重疊）。"""
    spans = sorted(spans)
    out = []
    for a, b in spans:
        if out and a <= out[-1][1] + tol:
            out[-1] = (out[-1][0], max(out[-1][1], b))
        else:
            out.append((a, b))
    return [(a, b) for a, b in out if b - a > 1e-6]


def _span_needed(g, y, half):
    """條帶要涵蓋整塊 ``g``（一條刈幅帶內的多邊形）所需的跨距 = 該塊的 x 範圍。

    試過更精細的版本：扣掉圓端帽已涵蓋的部分
    （``a = min_d[xmin(d)+sqrt(half²-d²)]``）。實測**結果幾乎一模一樣**
    （梯形 +40.86% vs +40.83%、壓測 39 勝 18 敗兩者相同），但每列多做 11 次
    線段相交 ⇒ 中位時間 297 ms → 823 ms，而且覆蓋率反而從 100.000% 掉到 99.990%。
    所以留用這個便宜版本，把那次嘗試記在這裡以免有人重做。
    """
    gx0, _gy0, gx1, _gy1 = g.bounds
    return (gx0, gx1) if gx1 - gx0 > 1e-9 else None


def _cut(rp: Polygon, ys, x0: float, x1: float, half: float = 0.0):
    """在旋轉後的座標系用一組 y 切出條帶。

    ``half > 0``（修正 1）時端點由 `_span_needed` 決定：以這條線負責的刈幅帶
    ``[y-half, y+half]`` 為準、扣掉圓端帽已經涵蓋的部分。斜邊碎屑因此
    **從源頭就不會產生**，不必靠補線收拾。
    """
    out = []
    for y in ys:
        if half > 0.0:
            inter = rp.intersection(box(x0 - 10.0, y - half, x1 + 10.0, y + half))
            segs = []
            for g in getattr(inter, "geoms", [inter]):
                if g.is_empty or g.geom_type not in ("Polygon", "MultiPolygon"):
                    continue
                sp = _span_needed(g, y, half)
                if sp is not None:
                    segs.append(sp)
        else:
            inter = rp.intersection(LineString([(x0 - 10.0, y), (x1 + 10.0, y)]))
            segs = []
            for g in getattr(inter, "geoms", [inter]):
                if g.geom_type == "LineString" and g.length > 1e-6:
                    xs = [c[0] for c in g.coords]
                    segs.append((min(xs), max(xs)))
        segs = [(a, b) for a, b in segs if b - a > 1e-9]
        if segs:
            out.append((y, _merge(segs)))
    return out


# ── 修正 1 + 2：刈幅帶跨距 + 局部補線 ────────────────────────────────
def _rows(poly: Polygon, pitch: float, swath: float, theta: float, band: bool = True):
    """旋轉到條帶水平後掃描 + **局部**驗證補線。回傳 [(y, [(a, b), ...]), ...]。"""
    rp = affinity.rotate(poly, -theta, origin=(0.0, 0.0), use_radians=True)
    x0, y0, x1, y1 = rp.bounds
    ext = y1 - y0
    if ext <= swath or pitch <= 0:
        ys = [(y0 + y1) / 2.0]
    else:
        k = int(math.ceil((ext - swath) / pitch - 1e-9)) + 1
        step = (ext - swath) / (k - 1)
        ys = [y0 + swath / 2.0 + i * step for i in range(k)]

    half = swath / 2.0
    table: dict[float, list[tuple[float, float]]] = {}
    for y, segs in _cut(rp, ys, x0, x1, half if band else 0.0):
        table[y] = list(segs)

    area = rp.area
    for _ in range(_MAX_REPAIR):
        segs = [LineString([(a, y), (b, y)]) for y, ss in table.items() for a, b in ss]
        if not segs:
            break
        miss = rp.difference(unary_union([s.buffer(half, quad_segs=8) for s in segs]))
        if miss.is_empty or miss.area <= _COV_TOL * area:
            break
        added_m, added_area, new = 0.0, 0.0, []
        for g in getattr(miss, "geoms", [miss]):
            if g.area <= _SLIVER_TOL * area:
                continue
            gx0, gy0, gx1, gy1 = g.bounds
            lo, hi = gx0 - half, gx1 + half        # ← 修正 1：夾在漏掃塊自己的 x 窗內
            kk = max(1, int(math.ceil((gy1 - gy0) / pitch - 1e-9)))
            for t in range(kk):
                y = gy0 + (gy1 - gy0) * (t + 0.5) / kk
                inter = rp.intersection(LineString([(x0 - 10.0, y), (x1 + 10.0, y)]))
                for h in getattr(inter, "geoms", [inter]):
                    if h.geom_type != "LineString" or h.length <= 1e-6:
                        continue
                    xs = [c[0] for c in h.coords]
                    a, b = max(min(xs), lo), min(max(xs), hi)
                    if b - a > 1e-6:
                        new.append((y, a, b))
                        added_m += b - a
            added_area += g.area
        # 修正 2：邊際成本不合理就不補（讓目標函數去淘汰這個方向）
        if not new or (added_area > 0 and added_m / added_area > _REPAIR_MAX_M_PER_M2):
            break
        for y, a, b in new:
            table.setdefault(y, []).append((a, b))
        for y in table:
            table[y] = _merge(table[y])
    return [(y, table[y]) for y in sorted(table) if table[y]]


# ── 單元分解 / 串接 / 定向（與 v1 相同）──────────────────────────────
def _cells(rows) -> list[list[tuple[int, int]]]:
    nodes = [(i, k) for i, (_y, segs) in enumerate(rows) for k in range(len(segs))]
    idx = {nd: t for t, nd in enumerate(nodes)}
    parent = list(range(len(nodes)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for i in range(len(rows) - 1):
        up, dn = rows[i][1], rows[i + 1][1]
        down_of = {k: [] for k in range(len(up))}
        up_of = {m: [] for m in range(len(dn))}
        for k, (a1, b1) in enumerate(up):
            for m, (a2, b2) in enumerate(dn):
                if min(b1, b2) - max(a1, a2) > _EPS:
                    down_of[k].append(m)
                    up_of[m].append(k)
        for k, ms in down_of.items():
            if len(ms) == 1 and len(up_of[ms[0]]) == 1:
                ra, rb = find(idx[(i, k)]), find(idx[(i + 1, ms[0])])
                if ra != rb:
                    parent[rb] = ra

    groups: dict[int, list[tuple[int, int]]] = {}
    for nd in nodes:
        groups.setdefault(find(idx[nd]), []).append(nd)
    return [sorted(g) for g in groups.values()]


def _strips_of_cell(cell, rows, theta) -> list[tuple]:
    c, s = math.cos(theta), math.sin(theta)
    out = []
    for i, k in cell:
        y = rows[i][0]
        a, b = rows[i][1][k]
        out.append(((c * a - s * y, s * a + c * y), (c * b - s * y, s * b + c * y)))
    return out


def _mid(strip):
    (x0, y0), (x1, y1) = strip
    return ((x0 + x1) / 2.0, (y0 + y1) / 2.0)


def _orient(strips, start):
    pos, out = start, []
    for p, q in strips:
        a, b = (p, q) if math.dist(pos, p) <= math.dist(pos, q) else (q, p)
        out.append((a, b))
        pos = b
    return out


def _subdivide(strips, target=_TARGET_PIECES, min_piece=_MIN_PIECE_M):
    tot = sum(math.dist(p, q) for p, q in strips)
    if tot <= 0 or not strips:
        return list(strips)
    g = max(min_piece, tot / max(target, 1))
    out = []
    for p, q in strips:
        L = math.dist(p, q)
        k = max(1, int(math.ceil(L / g - 1e-9)))
        if k == 1:
            out.append((p, q))
            continue
        dx, dy = (q[0] - p[0]) / k, (q[1] - p[1]) / k
        for t in range(k):
            out.append(((p[0] + dx * t, p[1] + dy * t),
                        (p[0] + dx * (t + 1), p[1] + dy * (t + 1))))
    return out


def _chain(cells, pos):
    order, left = [], list(range(len(cells)))
    while left:
        best = None
        for t in left:
            for rev in (False, True):
                seq = cells[t][::-1] if rev else cells[t]
                d = math.dist(pos, _mid(seq[0]))
                if best is None or d < best[0]:
                    best = (d, t, rev)
        _d, t, rev = best
        seq = cells[t][::-1] if rev else cells[t]
        order += seq
        pos = _mid(seq[-1])
        left.remove(t)
    return order, pos


def _part_cells(poly, pitch, swath, theta, band=True):
    rows = _rows(poly, pitch, swath, theta, band)
    if not rows:
        return []
    return [s for s in (_strips_of_cell(c, rows, theta) for c in _cells(rows)) if s]


def _ordered_strips_for(parts, angles, pitch, swath, start, cache=None,
                        band=True) -> list[tuple]:
    cells_all: list[list[tuple]] = []
    for i, (poly, theta) in enumerate(zip(parts, angles)):
        if cache is None:
            cells_all += _part_cells(poly, pitch, swath, theta, band)
            continue
        k = (i, round(theta, 9), band)
        if k not in cache:
            cache[k] = _part_cells(poly, pitch, swath, theta, band)
        cells_all += cache[k]
    if not cells_all:
        return []
    order, _end = _chain(cells_all, start)
    return _orient(order, start)


# ── 航跡建構與代價 ────────────────────────────────────────────────────
def _build(strips, start) -> list[tuple]:
    pos, pts = start, [start]
    for p, q in strips:
        a, b = (p, q) if math.dist(pos, p) <= math.dist(pos, q) else (q, p)
        pts += [a, b]
        pos = b
    pts.append(start)
    return pts


def _plen(t):
    return sum(math.dist(t[i], t[i - 1]) for i in range(1, len(t)))


def _cost_matrix(strips, start):
    """修正 4：與 v1 語意**逐位元相同**，改用 numpy 沿對角線推進。

    cm[j][i] = 飛 strips[j:i] 的最短航跡長（正序 / 逆序取小）；rv 記是否逆序。
    """
    S = len(strips)
    inf = float("inf")
    cm = np.full((S + 1, S + 1), inf)
    rv = np.zeros((S + 1, S + 1), dtype=bool)
    if S == 0:
        return cm, rv
    P = np.array([s[0] for s in strips], dtype=float)
    Q = np.array([s[1] for s in strips], dtype=float)
    Ln = np.hypot(Q[:, 0] - P[:, 0], Q[:, 1] - P[:, 1])
    sx, sy = float(start[0]), float(start[1])

    for order in (0, 1):                       # 0 = 正序（頭固定 j）, 1 = 逆序
        pos = np.empty((S, 2))
        pos[:, 0], pos[:, 1] = sx, sy
        acc = np.zeros(S)
        alive = np.arange(S)                   # order 0: j ；order 1: i-1
        for m in range(1, S + 1):
            if order == 0:
                a = alive[:S - m + 1]          # j = 0..S-m
                sidx = a + (m - 1)
            else:
                a = alive[m - 1:]              # i-1 = m-1..S-1
                sidx = a - (m - 1)
            p, q = P[sidx], Q[sidx]
            px, py = pos[a, 0], pos[a, 1]
            dp = np.hypot(px - p[:, 0], py - p[:, 1])
            dq = np.hypot(px - q[:, 0], py - q[:, 1])
            fwd = dp <= dq
            acc[a] += np.where(fwd, dp, dq) + Ln[sidx]
            pos[a, 0] = np.where(fwd, q[:, 0], p[:, 0])
            pos[a, 1] = np.where(fwd, q[:, 1], p[:, 1])
            tot = acc[a] + np.hypot(pos[a, 0] - sx, pos[a, 1] - sy)
            if order == 0:
                cm[a, sidx + 1] = tot
            else:
                jj, ii = sidx, a + 1
                better = tot < cm[jj, ii]
                cm[jj[better], ii[better]] = tot[better]
                rv[jj[better], ii[better]] = True
    return cm, rv


def _minmax_partition(cm, S, n):
    """連續切成恰好 n 段，精確 min-max；同 max 之下再取最小總長（同 v1 語意）。"""
    n = max(1, min(n, S))
    inf = float("inf")
    dp = np.full((n + 1, S + 1), inf)
    dp[0, 0] = 0.0
    for k in range(1, n + 1):
        prev = dp[k - 1]
        for i in range(k, S + 1):
            v = np.maximum(prev[k - 1:i], cm[k - 1:i, i])
            dp[k, i] = v.min() if v.size else inf
    mk = dp[n, S]
    if not np.isfinite(mk):
        return None
    cap = mk * (1.0 + 1e-9) + 1e-6

    tt = np.full((n + 1, S + 1), inf)
    par = np.full((n + 1, S + 1), -1, dtype=int)
    tt[0, 0] = 0.0
    for k in range(1, n + 1):
        prev = tt[k - 1]
        for i in range(k, S + 1):
            c = cm[k - 1:i, i]
            v = np.where(c <= cap, prev[k - 1:i] + c, inf)
            if v.size:
                t = int(np.argmin(v))
                if np.isfinite(v[t]):
                    tt[k, i], par[k, i] = v[t], k - 1 + t
    cuts, i = [S], S
    for k in range(n, 0, -1):
        j = int(par[k, i])
        if j < 0:
            return None
        cuts.append(j)
        i = j
    cuts.reverse()
    return cuts


def _alloc_contiguous(strips, n_uav, start, granular):
    if granular and n_uav > 1:
        strips = _subdivide(strips)
    cm, rv = _cost_matrix(strips, start)
    cuts = _minmax_partition(cm, len(strips), n_uav)
    if cuts is None:
        return [_build(strips, start)]
    out = []
    for a, b in zip(cuts[:-1], cuts[1:]):
        if b <= a:
            continue
        seg = strips[a:b]
        out.append(_build(seg[::-1] if rv[a][b] else seg, start))
    return out


def _alloc_interleaved(strips, n_uav, start, _granular):
    out = []
    for k in range(min(n_uav, len(strips))):
        b = strips[k::n_uav]
        if not b:
            continue
        fwd, bwd = _build(b, start), _build(b[::-1], start)
        out.append(fwd if _plen(fwd) <= _plen(bwd) else bwd)
    return out


def _strip_coverage(parts, strips, swath):
    area = unary_union(parts)
    if area.area <= 0 or not strips:
        return 0.0
    swept = unary_union([LineString([a, b]).buffer(swath / 2.0, quad_segs=8)
                         for a, b in strips])
    return area.intersection(swept).area / area.area


def _score(tracks, cov):
    lens = [_plen(t) for t in tracks]
    if not lens:
        return (1, float("inf"), float("inf"))
    return (0 if cov >= _COV_MIN else 1, max(lens), sum(lens))


# ── 修正 3：兩階段方向搜尋 ────────────────────────────────────────────
def min_width_angle(poly: Polygon) -> float:
    return min(_edge_widths(poly))[1]


def _v1_cand_angles(poly: Polygon, tol: float = 0.03) -> list[float]:
    """v1 的方向候選集：凸包邊方向且寬度在最小值 (1+tol) 內。

    **一律**併進 B 階段候選，所以 v2 的候選集是 v1 的**超集**：代理排序只負責
    「多找幾個方向」，不負責淘汰 v1 找得到的那些。
    """
    cand = sorted(_edge_widths(poly))
    wmin = cand[0][0]
    out, seen = [], []
    for w, th in cand:
        if w > wmin * (1.0 + tol) + 1e-9:
            break
        t = th % (2.0 * math.pi)
        if any(abs(t - u) < 1e-9 for u in seen):
            continue
        seen.append(t)
        out.append(th)
    return out or [0.0]


def _angle_grid(poly: Polygon) -> list[float]:
    """A 階段候選：全角度粗掃（相位有意義 ⇒ 0..360）+ 凸包邊方向及其 +180。"""
    out = [math.radians(d) for d in
           np.arange(0.0, 360.0, _COARSE_STEP_DEG)]
    for _w, th in _edge_widths(poly):
        out += [th, th + math.pi]
    return out


def _proxy(parts, angles, pitch, swath, start, cache, band=True):
    """A 階段代理成本：**單機**巡迴長度（不做 DP，實測 0.02 ms）。

    n 機 min-max 的分母幾乎與總航程同向（實測 balance 1.00-1.04），所以單機巡迴
    是很好的排序代理；真正決勝仍由 B 階段的完整目標函數做。
    """
    strips = _ordered_strips_for(parts, angles, pitch, swath, start, cache, band)
    if not strips:
        return float("inf"), None
    return _plen(_build(strips, start)), strips


def _best_angles_single(poly, pitch, swath, start, cache, part_idx, band=True):
    """單一連通塊：粗掃 → 局部細化 → 回傳前 _TOPK 個方向（依代理成本）。"""
    scored = []
    for th in _angle_grid(poly):
        c, _s = _proxy([poly], [th], pitch, swath, start, cache, band)
        scored.append((c, th))
    scored.sort(key=lambda r: r[0])
    seeds = [th for _c, th in scored[:_TOPK]]
    for step in _REFINE_DEG:                       # 在種子附近細化
        more = []
        for th in seeds:
            for d in (-step, step):
                t2 = th + math.radians(d)
                c, _s = _proxy([poly], [t2], pitch, swath, start, cache, band)
                more.append((c, t2))
        scored += more
        scored.sort(key=lambda r: r[0])
        seeds = [th for _c, th in scored[:_TOPK]]
    # 一律把 v1 的選擇（最小寬度方向與其 +180 相位）留在 B 階段候選裡，
    # 這樣 v2 在方向這一軸上**結構上不可能輸給 v1**：代理排序只負責「多找幾個」，
    # 不負責淘汰。（沒有這一條時 L 形 n=4 實測退步 0.70%。）
    for th in _v1_cand_angles(poly):
        for t in (th, th + math.pi):
            if not any(abs(t - s) < 1e-9 for s in seeds):
                seeds.append(t)
    return seeds


def _plan_one(parts, n_uav, swath_m, pitch_m, start, axis_aligned, granular, alloc,
              band, cache):
    memo: dict = {}

    def run(angles):
        key = tuple(round(a, 9) for a in angles)
        if key not in memo:
            strips = _ordered_strips_for(parts, angles, pitch_m, swath_m, start, cache,
                                         band)
            if strips:
                cov = _strip_coverage(parts, strips, swath_m)
                tracks = alloc(strips, n_uav, start, granular)
                memo[key] = (tracks, _score(tracks, cov))
            else:
                memo[key] = ([], (1, float("inf"), float("inf")))
        return memo[key]

    if axis_aligned:
        return run([0.0] * len(parts))

    per = [_best_angles_single(p, pitch_m, swath_m, start, cache, i, band)
           for i, p in enumerate(parts)]
    cur = [c[0] for c in per]
    best, best_key = run(cur)
    budget = _MAX_DIR_EVALS - 1
    improved = True
    while improved and budget > 0:
        improved = False
        for i, cands in enumerate(per):
            for th in cands:
                if budget <= 0:
                    break
                if th == cur[i]:
                    continue
                trial = list(cur)
                trial[i] = th
                budget -= 1
                tracks, key = run(trial)
                if key < best_key:
                    best, best_key, cur, improved = tracks, key, trial, True
    return best or [], best_key


def _plan(polys, n_uav, swath_m, pitch_m, start, axis_aligned, granular, alloc):
    """條帶端點的**兩種**造法都當候選（刈幅帶跨距 / v1 的中心線跨距），由同一個
    目標函數決勝 ⇒ v2 的搜尋空間在「方向」與「端點造法」兩軸上都是 v1 的超集。"""
    parts = _parts(polys)
    if not parts or n_uav < 1:
        return []
    best, best_key = [], None
    for band in (True, False):
        cache: dict = {}
        tr, key = _plan_one(parts, n_uav, swath_m, pitch_m, start, axis_aligned,
                            granular, alloc, band, cache)
        if tr and (best_key is None or key < best_key):
            best, best_key = tr, key
    return best


# ── 三個變體（與 v1 同名同義）─────────────────────────────────────────
def plan_contiguous(polys, n_uav, *, swath_m, pitch_m, start=(0.0, 0.0),
                    granular=True):
    return _plan(polys, n_uav, swath_m, pitch_m, start, False, granular,
                 _alloc_contiguous)


def plan_axis_aligned(polys, n_uav, *, swath_m, pitch_m, start=(0.0, 0.0),
                      granular=True):
    return _plan(polys, n_uav, swath_m, pitch_m, start, True, granular,
                 _alloc_contiguous)


def plan_interleaved(polys, n_uav, *, swath_m, pitch_m, start=(0.0, 0.0)):
    return _plan(polys, n_uav, swath_m, pitch_m, start, False, False,
                 _alloc_interleaved)


plan = plan_contiguous


# ── 自檢 ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import sys
    import time

    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:                                     # noqa: BLE001
        pass
    # ⚠ 原本硬編 session scratchpad 的路徑（會消失），改成 repo 相對。
    HERE = str(Path(__file__).resolve().parent)
    if HERE not in sys.path:
        sys.path.insert(0, HERE)
    from shapely import affinity                          # noqa: E402
    import algo_bous_variants as V1                       # noqa: E402
    from bench import boustrophedon, metrics, rect_xy     # noqa: E402

    SWATH = 2 * 140.0 * math.tan(math.radians(60.0) / 2)
    PITCH = SWATH * 0.7

    def L_shape(a=4000.0, b=2000.0):
        return Polygon([(-a / 2, -a / 2), (a / 2, -a / 2), (a / 2, -a / 2 + b),
                        (-a / 2 + b, -a / 2 + b), (-a / 2 + b, a / 2), (-a / 2, a / 2)])

    SC = [
        ("(1) rect 5.0x4.0 km n=8", [rect_xy(0, 0, 2500, 2000)], 8),
        ("(2) four 1.25x4.0 blocks n=8",
         [rect_xy(-2400, 0, 625, 2000), rect_xy(-800, 0, 625, 2000),
          rect_xy(800, 0, 625, 2000), rect_xy(2400, 0, 625, 2000)], 8),
        ("(3) L concave 12 km2 n=4", [L_shape()], 4),
        ("(4) trapezoid wedge n=4",
         [Polygon([(-3000, -400), (3000, -1200), (3000, 1200), (-3000, 400)])], 4),
        ("(5) plus / cross n=6",
         [unary_union([rect_xy(0, 0, 2700, 900), rect_xy(0, 0, 900, 2700)])], 6),
        ("(6) L rotated 20deg n=6", [affinity.rotate(L_shape(), 20, origin=(0, 0))], 6),
    ]
    print(f"swath {SWATH:.1f} m   pitch {PITCH:.1f} m   speed 20 m/s"
          "   (指標全部由 bench.metrics 計算)\n")
    HDR = (f"{'scenario':30s}{'method':10s}{'uav':>4s}{'tot km':>8s}{'max km':>8s}"
           f"{'cover':>9s}{'ovhd':>7s}{'redun':>7s}{'bal':>7s}{'scan':>8s}"
           f"{'waste':>8s}{'ms':>7s}")
    print(HDR)
    for title, polys, n in SC:
        for i, (nm, fn) in enumerate((
                ("bench", lambda: boustrophedon(polys, n, swath_m=SWATH, pitch_m=PITCH)),
                ("v1", lambda: V1.plan(polys, n, swath_m=SWATH, pitch_m=PITCH)),
                ("v2", lambda: plan(polys, n, swath_m=SWATH, pitch_m=PITCH)))):
            t0 = time.perf_counter()
            tracks = fn()
            dt = (time.perf_counter() - t0) * 1000
            m = metrics(polys, tracks, SWATH)
            print(f"{title if i == 0 else '':30s}{nm:10s}{m['n_uav']:4d}"
                  f"{m['total_km']:8.1f}{m['max_km']:8.2f}{m['coverage'] * 100:8.3f}%"
                  f"{m['overhead'] or 0:7.3f}{m['redundancy'] or 0:7.3f}"
                  f"{m['balance'] or 0:7.3f}{m['scan_rate']:8.2f}"
                  f"{(m['waste'] or 0) * 100:7.1f}%{dt:7.0f}")
        print()
