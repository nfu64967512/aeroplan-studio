"""（審查者 2 的副本，內容與 algo_darp_v2.py 相同；同目錄有別的 agent 在寫，
留一份自己的檔名以免被覆蓋。審查者 1 的版本是 algo_darp_v2_r1.py。）

algo_darp 的改進版：把「分割」這一層補強，幾何/航跡層原封不動沿用。

審查結論先講：algo_darp 的**航跡層已經接近最佳**——我用 2-opt（列序 + 每列方向
自由，不強制交替）重跑它產出的 8 條航跡，總長只從 202.4 改善到 202.3 km
（8 條裡 4 條 0 改善，最大單條 −53 m）。所以 v2 一行都沒動 ``_Field.route``。

真正還有空間的是**分割**與**掃描方向**。v2 加四件事：

1. ``_cell_refine``：**單格**邊界搬移（保連通），讓子區可以長成階梯狀。
   單一 5×4 km 區 n=8：最長航程 26.02 → 25.53 km（掃描率 55.34 → 56.41）。
   而且它便宜——整輪 0.9 s。
2. ``_refine_full``：整列/整行搬移與單格搬移**交替**做到不動為止。兩種鄰域互補：
   整列搬移能一次翻轉「條帶數奇偶」（決定出口在哪一側），單格搬移能磨邊界。
3. ``_chunks_balanced`` / ``_alloc_measured``：多區時，跨區切割的切點改用**實測航程**
   做局部搜尋（原版是等工作量切），配置改用實測 cost(m) 表做交換改良
   （原版是估計式貪婪）。五塊散置 n=6：24.66 → 27.99 km²/h（+13.5%），
   增益**全部**來自切點再平衡，配置那一項到目前為止從沒改變過答案。
4. ``_cand_angles``：掃描方向不再只取最小寬度，凸包邊方向與其垂線都量一次。
   「最小寬度」最小化的是**條帶數**（＝迴轉次數），但這把尺上迴轉只值一個
   pitch＝113 m，而空程動輒好幾 km ⇒ 那不是要最小化的量。實測某個凹形 n=2：
   最小寬度方向 12.45 km²/h，掃過 12 個方向後最好的是 13.71（+10.1%），而那
   一格正是原版唯一輸給 boustrophedon 的案例。矩形 n=3 也 +4.7%。

⚠ 一個對原作者自評的訂正：他寫「情境2 放鬆成分數配置解析解 ~26.7 km，
還有 14% 沒拿到」。那個估計**漏了跨塊轉場**。實測：外側塊 2 台 = 31.01 km，
其中 7.9 km 是空程；讓一台橫跨內外兩塊，它的空程會從 3.4 km 變成 ~5.9 km，
把搬過去的工作量吃掉大半。把「每台可跨塊」的切割真的做出來並用實測長度平衡，
只到 30.3–30.6 km（−2%），不是 26.7。這一項不是實作偷懶，是解析式高估。

介面與 algo_darp 相同：``plan(polys, n_uav, *, swath_m, pitch_m, start)``。
"""
from __future__ import annotations

import contextlib
import math

import numpy as np
from shapely.geometry import MultiPolygon, Polygon

import algo_darp
from shapely.ops import unary_union

from algo_darp import (_allocate, _blocks, _cell_order, _chain, _components,
                       _darp, _Field,
                       _geo_order, _plan_chunks, _plan_field, _plen, _refine,
                       _repair,
                       _routes_of, _score)

__all__ = ["plan"]

_TOPK = 5          # 每輪從最長的幾個子區找捐出者
_CELL_ROUNDS = 500
_FULL_K = 4        # 只有 _refine 排名前幾名的佈局才送進昂貴的交替精修
_GATE = 1.06       # 而且要落在領先者 6% 以內
_DIR_K = 3         # 掃描方向候選最多完整精修幾個
_DIR_GATE = 1.03   # 且要落在最好方向 3% 以內


# -- 單格邊界搬移 -----------------------------------------------------
def _cell_refine(field: _Field, lab: np.ndarray, m: int, start, lens: np.ndarray,
                 rounds: int = _CELL_ROUNDS):
    """把**單一格子**在相鄰子區之間搬，字典序壓 (最長, 總長)。

    ``_refine`` 只搬整列/整行，子區永遠是矩形塊；但最佳分割的邊界是階梯狀的
    ——離 home 遠的那幾列該窄一點，近的該寬一點，而「寬多少」是逐列不同的。
    每次移動只重算受影響的兩個子區的航跡，所以一輪只要 O(邊界格數) 次 route()。
    """
    lab = lab.copy()
    lens = lens.copy()
    memb = {i: set(np.flatnonzero(lab == i).tolist()) for i in range(m)}
    if any(not memb[i] for i in range(m)):
        return lab, lens

    def rl(idxs):
        if not idxs:
            return float("inf")
        t = field.route(list(idxs), start)
        return _plen(t) if len(t) >= 2 else float("inf")

    for _ in range(rounds):
        cur = (round(float(lens.max()), 3), round(float(lens.sum()), 3))
        best = None
        for i in np.argsort(-lens)[:_TOPK]:
            i = int(i)
            if len(memb[i]) < 2:
                continue
            # 只看**邊界格**（有鄰居屬於別區的格子）。內部格搬不動，逐一試它們
            # 是純粹浪費：矩形子區裡邊界格只佔兩三成。
            for t in tuple(memb[i]):
                nb = {int(lab[u]) for u in field.adj[t] if lab[u] != i}
                if not nb:
                    continue
                rest = memb[i] - {t}
                li = rl(rest)
                if not math.isfinite(li):
                    continue
                for j in nb:
                    lj = rl(memb[j] | {t})
                    if not math.isfinite(lj):
                        continue
                    l2 = lens.copy()
                    l2[i], l2[j] = li, lj
                    key = (round(float(l2.max()), 3), round(float(l2.sum()), 3))
                    # 連通性檢查排在**改善判定之後**：它是 O(子區格數) 而改善的
                    # 候選很少，先檢查等於對每個邊界格都付一次全區 BFS。
                    if key < cur and (best is None or key < best[0]) \
                            and len(_components(field, list(rest))) == 1:
                        best = (key, i, j, t, l2)
        if best is None:
            break
        _k, i, j, t, lens = best
        memb[i].discard(t)
        memb[j].add(t)
        lab[t] = j
    return lab, lens


def _refine_full(field: _Field, lab: np.ndarray, m: int, start, passes: int = 4,
                 pre: tuple | None = None):
    """整列/整行搬移 <-> 單格搬移，交替到不動為止。

    兩種鄰域互補：整列一次翻轉條帶數奇偶（＝出口換一側，可省 2 km 級的回程
    空程），單格只能一格一格磨。單獨跑任一種都會早停。
    ``pre`` 是已經跑過 ``_refine`` 的結果，用來省掉重跑。
    """
    if pre is None:
        lab, trks, lens = _refine(field, lab, m, start)
    else:
        lab, trks, lens = pre
    if any(len(t) < 2 for t in trks):
        return lab, trks, lens
    prev = None
    for _ in range(passes):
        lab, lens = _cell_refine(field, lab, m, start, lens)
        lab, trks, lens = _refine(field, lab, m, start)
        key = (round(float(lens.max()), 3), round(float(lens.sum()), 3))
        if key == prev:
            break
        prev = key
    trks, lens = _routes_of(field, lab, m, start)
    return lab, trks, lens


def _plan_field2(field: _Field, m: int, start):
    if field.n == 0 or m <= 0:
        return []
    if m == 1:
        t = field.route(list(range(field.n)), start)
        return [t] if len(t) >= 2 else []
    m = min(m, field.n)
    cands = []
    for b in range(1, m + 1):
        lab0 = _blocks(field, m, b)
        if lab0 is None:
            continue
        lab0 = _repair(field, lab0, m)
        trks, lens = _routes_of(field, lab0, m, start)
        pick = [(_score(trks, lens), lab0)]
        seeds = np.array([field.xy[lab0 == i].mean(axis=0) if np.any(lab0 == i)
                          else field.xy.mean(axis=0) for i in range(m)])
        lab1, s1 = _darp(field, m, seeds, start)
        if lab1 is not None:
            pick.append((s1, lab1))
        pick = [p for p in pick if math.isfinite(p[0])]
        if pick:
            cands.append(min(pick, key=lambda p: p[0]))
    if not cands:
        t = field.route(list(range(field.n)), start)
        return [t] if len(t) >= 2 else []
    # 先用便宜的 ``_refine`` 把每種佈局都精修一次（原版就是做到這裡），再只把
    # **前 _FULL_K 名**送進昂貴的交替精修。實測（單一 5x4 km, n=8）：8 種佈局
    # 全跑交替精修 42 s / 25.53 km，只跑前四名 5 s / 25.53 km，同一個答案。
    scored = []
    for _s, lab in cands:
        pre = _refine(field, lab, m, start)
        scored.append((_score(pre[1], pre[2]), pre))
    scored.sort(key=lambda r: r[0])
    best, best_s = None, float("inf")
    for s0, pre in scored:
        if math.isfinite(s0) and s0 < best_s:
            best_s, best = s0, pre[1]
    gate = scored[0][0] * _GATE                # 落後領先者 6% 以上就不必再磨
    for _s0, pre in scored[:_FULL_K]:
        if _s0 > gate:
            break
        _l, trks, lens = _refine_full(field, None, m, start, pre=pre)
        s = _score(trks, lens)
        if s < best_s:
            best_s, best = s, trks
    if best is None:
        t = field.route(list(range(field.n)), start)
        return [t] if len(t) >= 2 else []
    return [t for t in best if len(t) >= 2]


# -- 掃描方向：不要只信「最小寬度」 ------------------------------------
@contextlib.contextmanager
def _forced_angle(th: float):
    """暫時把 algo_darp 的最小寬度函式換掉，讓 _Field 用指定的掃描方向建格。"""
    orig = algo_darp._min_width_angle
    algo_darp._min_width_angle = lambda _p: th
    try:
        yield
    finally:
        algo_darp._min_width_angle = orig


def _cand_angles(poly: Polygon, cap: int = 6) -> list[float]:
    """候選掃描方向：凸包每條邊的方向 + 各自的垂線，去重後取樣。

    最小寬度方向排第一（它最小化**條帶數**＝迴轉次數，教科書答案），但在這個
    測試台裡迴轉只值一個 pitch（113 m）而空程動輒好幾 km，所以「條帶數最少」
    根本不是要最小化的量。實測（壓力測試 #13，凹形 n=2，home 在形心附近）：
    最小寬度方向 29.1 度 → 掃描率 12.45；掃過 12 個方向，120 度 → **13.71**，
    差 10.1%。而且那一格正好是 darp 唯一輸給 boustrophedon 的案例。
    """
    hull = poly.convex_hull
    if hull.geom_type != "Polygon":
        return [0.0]
    cs = list(hull.exterior.coords)[:-1]
    th0 = algo_darp._min_width_angle(poly)
    raw = [th0, (th0 + math.pi / 2) % math.pi]
    for i in range(len(cs)):
        ax, ay = cs[i]
        bx, by = cs[(i + 1) % len(cs)]
        if math.hypot(bx - ax, by - ay) < 1e-9:
            continue
        a = math.atan2(by - ay, bx - ax) % math.pi
        raw += [a, (a + math.pi / 2) % math.pi]
    out: list[float] = []
    for a in raw:                                   # 10 度以內視為同一個方向
        if all(min(abs(a - b), math.pi - abs(a - b)) > math.radians(10) for b in out):
            out.append(a)
    if len(out) <= cap:
        return out
    keep = out[:2]                                  # th0 與其垂線一定留
    rest = out[2:]
    step = len(rest) / (cap - 2)
    keep += [rest[int(i * step)] for i in range(cap - 2)]
    return keep


def _plan_field_dirs(poly: Polygon, swath, pitch, m, start, f_default: _Field):
    """對每個候選方向都規劃一次；先用便宜管線排名，只有前段才做完整精修。"""
    cands = []
    for th in _cand_angles(poly):
        with _forced_angle(th):
            f = _Field(poly, swath, pitch)
            cw = f.cw
            while f.n < m and cw > 2.0:
                cw /= 2.0
                f = _Field(poly, swath, pitch, cw=cw)
        if f.n == 0:
            continue
        tr = _plan_field(f, m, start)               # 便宜管線（= 原版）
        if tr:
            cands.append((max(_plen(t) for t in tr), th, f, tr))
    if not cands:
        return _plan_field2(f_default, m, start), f_default
    cands.sort(key=lambda r: r[0])
    gate = cands[0][0] * _DIR_GATE
    best, best_s, best_f = cands[0][3], cands[0][0], cands[0][2]
    for L0, _th, f, _tr in cands[:_DIR_K]:
        if L0 > gate:
            break
        tr = _plan_field2(f, m, start)              # 完整精修
        if tr:
            s = max(_plen(t) for t in tr)
            if s < best_s:
                best_s, best, best_f = s, tr, f
    return best, best_f


# -- 多區：跨區切割的切點用實測長度再平衡 ------------------------------
def _chunk_seq(fields):
    order = _geo_order(fields)
    seq = [(fi, ci) for fi in order for ci in _cell_order(fields[fi])]
    return order, seq


def _chunks_from_cuts(fields, order, seq, cuts, start):
    out = []
    for i in range(len(cuts) - 1):
        by: dict[int, list[int]] = {}
        for fi, ci in seq[cuts[i]:cuts[i + 1]]:
            by.setdefault(fi, []).append(ci)
        out.append(_chain(fields, order, by, start))
    return out


def _chunks_balanced(fields, n, start, rounds: int = 200):
    """跨區連續切割 + 切點局部搜尋（目標函數是**實測**折線長度，不是工作量）。"""
    order, seq = _chunk_seq(fields)
    if not seq or n < 1:
        return []
    n = min(n, len(seq))
    w = np.array([fields[fi].cells[ci].w for fi, ci in seq], dtype=float)
    cum = np.concatenate([[0.0], np.cumsum(w)])
    tot = cum[-1]
    cuts = [0] + [int(np.searchsorted(cum, tot * (i + 1) / n)) for i in range(n - 1)] \
        + [len(seq)]
    for q in range(1, n):
        cuts[q] = max(cuts[q], cuts[q - 1] + 1)
    for q in range(n - 1, 0, -1):
        cuts[q] = min(cuts[q], cuts[q + 1] - 1)
    if any(cuts[q] <= cuts[q - 1] for q in range(1, n + 1)):
        return []

    def key(cs):
        ts = _chunks_from_cuts(fields, order, seq, cs, start)
        if len(ts) != n or any(len(t) < 2 for t in ts):
            return (float("inf"), float("inf")), ts
        L = [_plen(t) for t in ts]
        return (round(max(L), 3), round(sum(L), 3)), ts

    best_k, best_t = key(cuts)
    best_c = list(cuts)
    if not math.isfinite(best_k[0]):
        return []
    # 最佳改良（每輪掃完所有 (切點, 步長) 才動），不是先到先贏：first-improvement
    # 的軌跡對步長清單的順序敏感——實測同一個問題只是把 ±16 加進清單，答案就從
    # 30.64 變成 31.62 km。這種「參數順序決定答案」的搜尋不能拿來做基準。
    for _ in range(rounds):
        best = None
        for q in range(1, n):
            for d in (-32, -16, -8, -4, -2, -1, 1, 2, 4, 8, 16, 32):
                cs = list(best_c)
                cs[q] += d
                if not (cs[q - 1] < cs[q] < cs[q + 1]):
                    continue
                k2, t2 = key(cs)
                if k2 < best_k and (best is None or k2 < best[0]):
                    best = (k2, t2, cs)
        if best is None:
            break
        best_k, best_t, best_c = best
    return best_t


# -- 多區：配置用實測 cost(m) 表 ---------------------------------------
def _alloc_measured(fields, n, start, cache):
    """每區先 1 台，之後把下一台給「邊際收益最大」的區——但成本是**實測**的。

    原版 ``_allocate`` 用 (條帶長+迴轉)/m + 2·離場距離 的估計式；估計式看不到
    奇偶效應也看不到子區形狀。實測表只多花 O(區數 × n) 次 _plan_field。
    """
    k = len(fields)
    m = [1] * k

    def cost(i, mi):
        key = (i, mi)
        if key not in cache:
            tr = _plan_field(fields[i], mi, start)
            cache[key] = max(_plen(t) for t in tr) if tr else float("inf")
        return cache[key]

    for _ in range(n - k):
        j = max(range(k), key=lambda i: cost(i, m[i]) - cost(i, m[i] + 1))
        m[j] += 1
    for _ in range(2 * k):                     # 交換改良
        cur = max(cost(i, m[i]) for i in range(k))
        hot = max(range(k), key=lambda i: cost(i, m[i]))
        best = None
        for i in range(k):
            if i == hot or m[i] <= 1:
                continue
            v = max([cost(i, m[i] - 1), cost(hot, m[hot] + 1)]
                    + [cost(q, m[q]) for q in range(k) if q not in (i, hot)])
            if v < cur - 1e-6 and (best is None or v < best[0]):
                best = (v, i)
        if best is None:
            break
        m[best[1]] -= 1
        m[hot] += 1
    # 原版的估計式配置也算一次分數，取較好的那個。實測到目前為止兩者從沒分歧
    # （四塊 [2,2,2,2]、五塊散置 [1,2,1,1,1] 都相同），所以這一項是**尚未證實
    # 有用**的改動；留著只是因為比較的代價是零，而且保證不會比原版差。
    alt = _allocate(fields, n, start)
    if alt != m and max(cost(i, alt[i]) for i in range(k)) < \
            max(cost(i, m[i]) for i in range(k)) - 1e-6:
        return alt
    return m


def plan(polys, n_uav, *, swath_m, pitch_m, start=(0.0, 0.0)):
    """DARP 式覆蓋規劃（改進版）。回傳每機一條折線，頭尾都在 start。"""
    n_uav = int(n_uav)
    if n_uav < 1 or not polys or swath_m <= 0 or pitch_m <= 0:
        return []
    geoms = []
    for p in polys:
        g = p if isinstance(p, (Polygon, MultiPolygon)) else Polygon(p)
        if g.is_empty:
            continue
        g = g.buffer(0)
        if not g.is_empty:
            geoms.append(g)
    if not geoms:
        return []
    u = unary_union(geoms)
    parts = [g for g in (u.geoms if u.geom_type == "MultiPolygon" else [u])
             if g.geom_type == "Polygon" and g.area > 0]
    if not parts:
        return []
    fields = [f for f in (_Field(p, swath_m, pitch_m) for p in parts) if f.n > 0]
    if not fields:
        return []

    if n_uav < len(fields):
        # 兩個起始切法（等工作量 vs 原版 _plan_chunks 的累積規則）都跑，取較小的
        # 最長航程。改良版**不准比原版差**，這一行就是那道保險。
        opts = [t for t in (_chunks_balanced(fields, n_uav, start),
                            _plan_chunks(fields, n_uav, start)) if t]
        if not opts:
            return []
        return min(opts, key=lambda ts: max(_plen(t) for t in ts))

    cache: dict = {}
    alloc = _alloc_measured(fields, n_uav, start, cache)
    for i, (f, mm) in enumerate(zip(fields, alloc)):
        cw = f.cw
        while fields[i].n < mm and cw > 2.0:      # 格子比飛機少 ⇒ 細分行寬
            cw /= 2.0
            fields[i] = _Field(f.poly, swath_m, pitch_m, cw=cw)
    tracks = []
    for i, (f, mm) in enumerate(zip(fields, alloc)):
        tr, fbest = _plan_field_dirs(f.poly, swath_m, pitch_m, mm, start, f)
        fields[i] = fbest
        tracks += tr

    if len(fields) > 1:
        mx_t = max(_plen(t) for t in tracks) if tracks else float("inf")
        for alt in (_chunks_balanced(fields, n_uav, start),
                    _plan_chunks(fields, n_uav, start)):
            if alt and len(alt) >= len(tracks) and max(_plen(t) for t in alt) < mx_t:
                tracks, mx_t = alt, max(_plen(t) for t in alt)
    return tracks
