"""牛耕式（boustrophedon）三變體 —— key = ``bous_variants``

共用管線（三個變體只差「方向」與「分配」兩步）:

  1. `unary_union(polys)` 去重疊 → 逐個連通塊處理
  2. 掃描方向 θ：`contiguous` / `interleaved` 以**最小寬度**（rotating calipers，
     條帶數最少）為起點，把寬度在 +3% 內的方向都當候選，逐塊座標下降、用**真實
     目標函數**決勝（見 `_cand_angles`）；`axis_aligned` 固定 θ=0（東西向）
     ＝「沒做方向優化」的對照
  3. 條帶佈點：以刈幅內縮 `swath/2` 後把 k-1 個間隔**均分**，
     `step = (ext - swath) / (k - 1) <= pitch`
     ⇒ 兩端剛好與刈幅邊緣切齊，不留邊帶也不多伸出去。
     （原 `bench.boustrophedon` 從 `pitch/2` 起跳、固定步進，尾端會留下最多
       `pitch - swath/2 ≈ 32 m` 的漏帶 ⇒ 覆蓋率 99.6%）
  4. 牛耕式單元分解（cell decomposition）：凹多邊形一條掃描線會切出多段，
     依「上下鄰居互為唯一」的鏈接規則把段落聚成 cell，cell 內按列連續、
     cell 之間貪婪最近鄰排序 ⇒ 凹形不會在兩葉之間來回跳。
     排完之後把每條條帶**定向**成蛇形（頭尾交替），下游都吃這個方向。
  5. **分配**（三個變體的分歧點）：
       `plan_contiguous`   連續切塊 + DP 精確 min-max（同 max 之下再取最小總航程）
       `plan_interleaved`  條帶輪流發（k, k+n, k+2n, ...）；方向選擇與 contiguous
                           **完全相同** ⇒ 兩者相減就是「分配方式」的差
       `plan_axis_aligned` 分配與 contiguous **完全相同**，只有 θ 鎖 0
                           ⇒ 兩者相減就是「方向優化值多少」
  6. 航跡建構：每條條帶挑離目前位置較近的那一端進入（自動蛇形），
     整段序列的正序 / 逆序兩種都算，取短的。

`granular=True`（contiguous / axis_aligned 的預設）會把條帶預先切成小段，
讓 DP 的切點可以落在條帶**中間**——多機分一個長條帶時不必整條給一台，
負載不均因此掉到 ~1.00。切中間要付轉場，DP 用精確代價自己決定值不值得。

所有代價（含往返 `start` 的轉場）都由 `_cost_matrix` 算真實折線長度，
DP 直接吃它 ⇒「哪一台拿哪一段」是對真實航跡最佳化，不是按條帶數硬分。
"""
from __future__ import annotations

import math

from shapely import affinity
from shapely.geometry import LineString, MultiPolygon, Polygon
from pathlib import Path
from shapely.ops import unary_union

__all__ = ["plan", "plan_contiguous", "plan_interleaved", "plan_axis_aligned",
           "min_width_angle"]

_EPS = 1e-9
_TARGET_PIECES = 300        # granular 模式下的子段目標數（控 DP 的 O(n·S²)）
_MIN_PIECE_M = 250.0        # 子段不切得比這更短
_WIDTH_TOL = 0.03           # 寬度在最小值 +3% 內的方向都是候選，再用真實目標函數決勝
_MAX_DIR_EVALS = 24         # 方向座標下降的求解次數上限
_MAX_REPAIR = 3             # 驗證補線的重複上限
_COV_TOL = 2e-4             # 漏揃面積低於這個比例就不再補線
_SLIVER_TOL = 1e-6          # 比這個還小的漏揃塊不理（斜邊碎屑）
_COV_MIN = 0.999            # 方向候選的可行性閘門


# ── 幾何小工具 ────────────────────────────────────────────────────────
def _parts(polys) -> list[Polygon]:
    """正規化成互不重疊的 Polygon list（重疊區只掃一次）。"""
    if isinstance(polys, (Polygon, MultiPolygon)):
        polys = [polys]
    geoms = [g for g in polys if g is not None and not g.is_empty]
    if not geoms:
        return []
    u = unary_union([g.buffer(0) for g in geoms])
    return [g for g in getattr(u, "geoms", [u])
            if g.geom_type == "Polygon" and g.area > 0]


def _edge_widths(poly: Polygon) -> list[tuple[float, float]]:
    """凸包每條邊的方向與對應寬度 [(width, theta), ...]（rotating calipers）。"""
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


def min_width_angle(poly: Polygon) -> float:
    """最小寬度方向（rad）。條帶沿此方向 ⇒ 條帶數最少（rotating calipers）。"""
    return min(_edge_widths(poly))[1]


def _cand_angles(poly: Polygon, tol: float = _WIDTH_TOL) -> list[float]:
    """寬度在最小值 (1+tol) 內的所有方向，最小寬度排第一。

    兩件事讓「取最小寬度」不足以定案：

    * 最小寬度只保證**條帶數**最少，不保證**航跡**最短。凹形上換個方向可能少切
      一個 cell，省下的 cell 間轉場比多出來的一兩條條帶還多。
    * θ 與 θ+180° 條帶**方向相同但相位不同**（`_rows` 一律從旋轉後的下緣起算，
      翻 180° 等於改從另一邊起算）。凹形的缺口被切成幾段會跟著變 ——
      L 形轉 50° 實測 θ=140° 得 35 條、θ=-40° 得 36 條，差一整條。
      所以**不可**把兩者視為同一個候選（早期版本 mod π 去重，害這裡少 1.3%）。

    去重只去真正重複的（mod 2π）。最後由 `_plan` 用真實目標函數決勝。
    """
    cand = sorted(_edge_widths(poly))
    wmin = cand[0][0]
    out, seen = [], []
    for w, th in cand:
        if w > wmin * (1.0 + tol) + 1e-9:
            break
        t = th % (2.0 * math.pi)
        if any(abs(t - s) < 1e-9 for s in seen):
            continue
        seen.append(t)
        out.append(th)
    return out or [0.0]


def _cut(rp: Polygon, ys, x0: float, x1: float):
    """在旋轉後的座標系用一組 y 切出條帶。回傳 [(y, [(a, b), ...]), ...]。"""
    out = []
    for y in ys:
        inter = rp.intersection(LineString([(x0 - 10.0, y), (x1 + 10.0, y)]))
        segs = []
        for g in getattr(inter, "geoms", [inter]):
            if g.geom_type == "LineString" and g.length > 1e-6:
                xs = [c[0] for c in g.coords]
                segs.append((min(xs), max(xs)))
        if segs:
            out.append((y, sorted(segs)))
    return out


def _rows(poly: Polygon, pitch: float, swath: float, theta: float):
    """旋轉到條帶水平後掃描 + **驗證補線**。回傳 [(y, [(a, b), ...]), ...]，列由下往上。

    均勻格線在「截面寬度會跳變」的地方會漏：L 形沿長邊掃時條帶長度從 4000 直接掉到
    2000，階梯下方那條 `pitch - swath/2 ≈ 32 m` 高、2000 m 長的帶子上下兩條掃描線
    都搆不到 ⇒ 實測漏 61,613 m²（覆蓋率 99.5%）。純幾何門檻（平行邊、截面跳變量）
    都要調參數又擋不住斜邊，所以這裡直接**用刈幅緩衝區驗證**：算出沒被蓋到的區塊，
    在它自己的 y 範圍內補線，最多重複 `_MAX_REPAIR` 次。
    """
    rp = affinity.rotate(poly, -theta, origin=(0.0, 0.0), use_radians=True)
    x0, y0, x1, y1 = rp.bounds
    ext = y1 - y0
    if ext <= swath or pitch <= 0:
        ys = [(y0 + y1) / 2.0]
    else:
        k = int(math.ceil((ext - swath) / pitch - 1e-9)) + 1       # 條帶數
        step = (ext - swath) / (k - 1)                             # <= pitch
        ys = [y0 + swath / 2.0 + i * step for i in range(k)]

    rows = _cut(rp, ys, x0, x1)
    half, area = swath / 2.0, rp.area
    for _ in range(_MAX_REPAIR):
        segs = [LineString([(a, y), (b, y)]) for y, ss in rows for a, b in ss]
        if not segs:
            break
        miss = rp.difference(unary_union([s.buffer(half, quad_segs=8) for s in segs]))
        if miss.is_empty or miss.area <= _COV_TOL * area:
            break
        add = []
        for g in getattr(miss, "geoms", [miss]):
            if g.area <= _SLIVER_TOL * area:       # 斜邊上的碎屑，圓端帽本來就吃掉了
                continue
            gy0, gy1 = g.bounds[1], g.bounds[3]
            k = max(1, int(math.ceil((gy1 - gy0) / pitch - 1e-9)))
            add += [gy0 + (gy1 - gy0) * (t + 0.5) / k for t in range(k)]
        if not add:
            break
        ys = sorted(set(ys) | set(add))
        rows = _cut(rp, ys, x0, x1)
    return rows


def _cells(rows) -> list[list[tuple[int, int]]]:
    """牛耕式單元分解：把 (列, 段) 節點鏈成 cell。

    標準規則：節點 u 與下一列的 v 同 cell，當且僅當 u 往下只有 v 一個鄰居、
    且 v 往上只有 u 一個鄰居（分裂 / 合併事件都切斷 cell）。
    """
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
    """cell → 由下而上的條帶（回到原座標系的兩個端點）。"""
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
    """把條帶定向成蛇形：每條的頭端 = 離前一條尾端較近者。"""
    pos, out = start, []
    for p, q in strips:
        a, b = (p, q) if math.dist(pos, p) <= math.dist(pos, q) else (q, p)
        out.append((a, b))
        pos = b
    return out


def _subdivide(strips, target=_TARGET_PIECES, min_piece=_MIN_PIECE_M):
    """把已定向的條帶等分成子段（順著方向），讓 DP 的切點能落在條帶中間。

    子段共線且首尾相接 ⇒ 同一台飛連續子段時轉場為 0，代價與整條完全相同；
    只有在**跨機切點**落在中間時才會產生額外轉場，而那筆由 DP 精確計入。
    """
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
    """cell 之間貪婪最近鄰（每個 cell 正反兩向都試）→ (攤平的條帶序列, 結束位置)。"""
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


def _part_cells(poly, pitch, swath, theta):
    rows = _rows(poly, pitch, swath, theta)
    if not rows:
        return []
    return [s for s in (_strips_of_cell(c, rows, theta) for c in _cells(rows)) if s]


def _ordered_strips_for(parts, angles, pitch, swath, start, cache=None) -> list[tuple]:
    """給定每個連通塊的掃描方向 → 一條地理上連續、已定向成蛇形的條帶序列。

    `cache` 讓方向座標下降不必重算同一個 (區塊, θ) 的條帶（_rows 含驗證補線，不便宜）。
    """
    cells_all: list[list[tuple]] = []
    for i, (poly, theta) in enumerate(zip(parts, angles)):
        if cache is None:
            cells_all += _part_cells(poly, pitch, swath, theta)
            continue
        k = (i, theta)
        if k not in cache:
            cache[k] = _part_cells(poly, pitch, swath, theta)
        cells_all += cache[k]
    if not cells_all:
        return []
    order, _end = _chain(cells_all, start)
    return _orient(order, start)


# ── 航跡建構與代價 ────────────────────────────────────────────────────
def _build(strips, start) -> list[tuple]:
    """依序飛過 strips，每條挑近端進入 ⇒ 自動蛇形。首尾都在 start。"""
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
    """cm[j][i] = 飛 strips[j:i] 的最短航跡長（正序/逆序取小）；rv 記是否逆序。"""
    S = len(strips)
    inf = float("inf")
    cm = [[inf] * (S + 1) for _ in range(S + 1)]
    rv = [[False] * (S + 1) for _ in range(S + 1)]
    dist = math.dist

    for j in range(S):                                   # 正序：固定頭往後長
        pos, acc, row = start, 0.0, cm[j]
        for i in range(j, S):
            p, q = strips[i]
            dp_, dq = dist(pos, p), dist(pos, q)
            if dp_ <= dq:
                acc += dp_ + dist(p, q)
                pos = q
            else:
                acc += dq + dist(q, p)
                pos = p
            row[i + 1] = acc + dist(pos, start)

    for i in range(1, S + 1):                            # 逆序：固定尾往前長
        pos, acc = start, 0.0
        for j in range(i - 1, -1, -1):
            p, q = strips[j]
            dp_, dq = dist(pos, p), dist(pos, q)
            if dp_ <= dq:
                acc += dp_ + dist(p, q)
                pos = q
            else:
                acc += dq + dist(q, p)
                pos = p
            c = acc + dist(pos, start)
            if c < cm[j][i]:
                cm[j][i] = c
                rv[j][i] = True
    return cm, rv


def _minmax_partition(cm, S, n):
    """連續切成恰好 n 段（n <= S），精確 min-max；同 max 之下再取最小總長。"""
    n = max(1, min(n, S))
    inf = float("inf")
    dp = [[inf] * (S + 1) for _ in range(n + 1)]
    dp[0][0] = 0.0
    for k in range(1, n + 1):
        prev, cur = dp[k - 1], dp[k]
        for i in range(k, S + 1):
            best = inf
            for j in range(k - 1, i):
                pv = prev[j]
                if pv == inf:
                    continue
                c = cm[j][i]
                v = pv if pv > c else c
                if v < best:
                    best = v
            cur[i] = best
    mk = dp[n][S]
    if mk == inf:
        return None
    cap = mk * (1.0 + 1e-9) + 1e-6

    tt = [[inf] * (S + 1) for _ in range(n + 1)]
    par = [[-1] * (S + 1) for _ in range(n + 1)]
    tt[0][0] = 0.0
    for k in range(1, n + 1):
        prev, cur, pk = tt[k - 1], tt[k], par[k]
        for i in range(k, S + 1):
            best, bj = inf, -1
            for j in range(k - 1, i):
                pv = prev[j]
                if pv == inf:
                    continue
                c = cm[j][i]
                if c > cap:
                    continue
                v = pv + c
                if v < best:
                    best, bj = v, j
            cur[i], pk[i] = best, bj

    cuts, i = [S], S
    for k in range(n, 0, -1):
        j = par[k][i]
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
    """條帶（不含轉場）蓋到作業區的比例 —— 方向搜尋的**可行性閘門**。

    沒有這一關，方向搜尋會偏好「少掃一點所以比較短」的方向：L 形轉 20° 實測
    就選出過覆蓋率 99.5% 的方向，因為它航程最短。目標函數只看長度就會獎勵漏掃。
    """
    area = unary_union(parts)
    if area.area <= 0 or not strips:
        return 0.0
    swept = unary_union([LineString([a, b]).buffer(swath / 2.0, quad_segs=8)
                         for a, b in strips])
    return area.intersection(swept).area / area.area


def _score(tracks, cov):
    """規劃目標：**先可行**（覆蓋率過關），再最小化最長航程（＝掃描率的分母），
    最後才是總航程。"""
    lens = [_plen(t) for t in tracks]
    if not lens:
        return (1, float("inf"), float("inf"))
    return (0 if cov >= _COV_MIN else 1, max(lens), sum(lens))


def _plan(polys, n_uav, swath_m, pitch_m, start, axis_aligned, granular, alloc):
    parts = _parts(polys)
    if not parts or n_uav < 1:
        return []
    memo: dict = {}          # 方向組合 → (tracks, key)
    cache: dict = {}         # (區塊, θ) → cells，避免座標下降重算 _rows

    def run(angles):
        key = tuple(angles)
        if key not in memo:
            strips = _ordered_strips_for(parts, angles, pitch_m, swath_m, start,
                                         cache)
            if strips:
                cov = _strip_coverage(parts, strips, swath_m)
                tracks = alloc(strips, n_uav, start, granular)
                memo[key] = (tracks, _score(tracks, cov))
            else:
                memo[key] = ([], (1, float("inf"), float("inf")))
        return memo[key]

    if axis_aligned:
        return run([0.0] * len(parts))[0]

    per = [_cand_angles(p) for p in parts]
    cur = [c[0] for c in per]                       # 起點 = 逐塊最小寬度
    best, best_key = run(cur)
    budget = _MAX_DIR_EVALS - 1

    # 逐塊座標下降：候選數對區塊數是**線性**的，不是笛卡兒積。
    # （方向選擇在區塊之間幾乎不耦合，唯一的耦合是區塊間的轉場順序）
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
    return best or []


# ── 三個變體 ──────────────────────────────────────────────────────────
def plan_contiguous(polys, n_uav, *, swath_m, pitch_m, start=(0.0, 0.0),
                    granular=True):
    """變體 3：最小寬度方向 + **連續**切塊 + DP 精確 min-max。

    順帶修掉 `bench.boustrophedon` 只用 7 台的瑕疵：那裡是
    `if acc >= share and len(tracks) < n_uav - 1` 的貪婪切法，
    每切一刀都無條件**進位**（35 條 / 8 台 ⇒ share = 4.375 條，貪婪切成 5 條一段
    ⇒ 7 段就切完，第 8 台永遠拿不到）。這裡改成恰好 n 段的 DP，結構上不可能少台。
    """
    return _plan(polys, n_uav, swath_m, pitch_m, start, False, granular,
                 _alloc_contiguous)


def plan_axis_aligned(polys, n_uav, *, swath_m, pitch_m, start=(0.0, 0.0),
                      granular=True):
    """變體 2：條帶固定東西向（θ=0）。分配邏輯與 contiguous 一字不差
    ⇒ 兩者相減 = 方向優化的價值，沒有其他變因。"""
    return _plan(polys, n_uav, swath_m, pitch_m, start, True, granular,
                 _alloc_contiguous)


def plan_interleaved(polys, n_uav, *, swath_m, pitch_m, start=(0.0, 0.0)):
    """變體 1：條帶輪流發給各機（k, k+n, k+2n, ...）。

    負載按**條帶數**幾乎完全平均，代價是每機相鄰條帶隔了 n·pitch ⇒ 轉場暴增。
    方向選擇與 contiguous 相同（同一個目標函數），每機自己那組仍用正序/逆序取小
    + 就近端點進入 ⇒ 與 contiguous 的差就是**分配方式**的差，沒有其他變因。
    """
    return _plan(polys, n_uav, swath_m, pitch_m, start, False, False,
                 _alloc_interleaved)


plan = plan_contiguous          # 預設 = 我認為最好的那個


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
    from bench import boustrophedon, metrics, rect_xy      # noqa: E402

    ALT, HFOV = 140.0, 60.0
    SWATH = 2 * ALT * math.tan(math.radians(HFOV) / 2)
    PITCH = SWATH * 0.7

    def L_shape(a=4000.0, b=2000.0):
        return Polygon([(-a / 2, -a / 2), (a / 2, -a / 2), (a / 2, -a / 2 + b),
                        (-a / 2 + b, -a / 2 + b), (-a / 2 + b, a / 2), (-a / 2, a / 2)])

    SC = [
        ("(1) single 5.0x4.0 km rect, n=8", [rect_xy(0, 0, 2500, 2000)], 8),
        ("(2) four 1.25x4.0 km blocks, n=8",
         [rect_xy(-2400, 0, 625, 2000), rect_xy(-800, 0, 625, 2000),
          rect_xy(800, 0, 625, 2000), rect_xy(2400, 0, 625, 2000)], 8),
        ("(3) L-shaped concave 12 km2, n=4", [L_shape()], 4),
        ("(4) 5.0x4.0 km rect rotated 30deg, n=8",
         [affinity.rotate(rect_xy(0, 0, 2500, 2000), 30, origin=(0, 0))], 8),
        ("(5) L-shape rotated 20deg, n=6",
         [affinity.rotate(L_shape(), 20, origin=(0, 0))], 6),
    ]

    HDR = (f"{'method':22s}{'uav':>4s}{'tot km':>8s}{'max km':>8s}{'cover':>8s}"
           f"{'ovhd':>7s}{'redun':>7s}{'bal':>7s}{'scan km2/h':>12s}{'waste':>8s}{'s':>6s}")

    def show(name, polys, fn):
        t0 = time.perf_counter()
        tracks = fn()
        dt = time.perf_counter() - t0
        m = metrics(polys, tracks, SWATH)
        print(f"{name:22s}{m['n_uav']:4d}{m['total_km']:8.1f}{m['max_km']:8.1f}"
              f"{m['coverage'] * 100:7.1f}%{m['overhead'] or 0:7.3f}"
              f"{m['redundancy'] or 0:7.3f}{m['balance'] or 0:7.3f}"
              f"{m['scan_rate']:12.2f}{(m['waste'] or 0) * 100:7.1f}%{dt:6.1f}")
        return m

    print(f"swath {SWATH:.1f} m   pitch {PITCH:.1f} m   speed 20 m/s\n")
    res = {}
    for title, polys, n in SC:
        print(title)
        print(HDR)
        kw = dict(swath_m=SWATH, pitch_m=PITCH)
        r = {
            "bench_baseline": show("bench_baseline", polys,
                                   lambda: boustrophedon(polys, n, **kw)),
            "contiguous": show("contiguous", polys,
                               lambda: plan_contiguous(polys, n, **kw)),
            "contiguous(whole)": show("contiguous(whole)", polys,
                                      lambda: plan_contiguous(polys, n, granular=False, **kw)),
            "interleaved": show("interleaved", polys,
                                lambda: plan_interleaved(polys, n, **kw)),
            "axis_aligned": show("axis_aligned", polys,
                                 lambda: plan_axis_aligned(polys, n, **kw)),
        }
        res[title] = r
        print()

    print("value of scan-direction optimisation  (contiguous - axis_aligned, "
          "identical allocator)")
    print(f"{'scenario':40s}{'d tot km':>10s}{'d max km':>10s}{'d scan':>9s}{'ratio':>8s}")
    for title, r in res.items():
        a, b = r["contiguous"], r["axis_aligned"]
        print(f"{title[:40]:40s}{a['total_km'] - b['total_km']:+10.1f}"
              f"{a['max_km'] - b['max_km']:+10.1f}"
              f"{a['scan_rate'] - b['scan_rate']:+9.2f}"
              f"{a['scan_rate'] / b['scan_rate']:8.3f}")
