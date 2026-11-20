"""牛耕式 v2（審查者 1 的改進版）—— key = ``bous_variants_v2_rev1``

⚠ 檔名不是 `algo_bous_variants_v2.py`：另一個 agent 在同一輪把那個路徑寫成了
   他自己的版本（同一時間兩個 session 寫同一個檔）。為了不互相覆蓋，這裡另開檔名。

介面與 v1 相同::

    plan(polys, n_uav, *, swath_m, pitch_m, start=(0.0, 0.0)) -> list[list[tuple]]

只依賴 shapely + 標準庫。不 import 本專案的規劃器；沿用 `algo_bous_variants`（v1）
已驗證的幾何管線（`_parts` / `_rows` 含驗證補線 / `_cells` / `_orient` /
`_subdivide` / `_cost_matrix` / `_minmax_partition`），只換掉**方向**與**分配**
兩步，所以「v2 − v1」就是這兩步的價值。

改動一：方向搜尋改成全角度兩階段搜尋
------------------------------------
v1 的候選只有「凸包邊方向、且寬度在最小寬度 +3% 內」。寬度只決定**條帶數**，
決定不了航跡長度 —— 方向同時決定轉場幾何。用 v1 自己的管線逐角度暴力掃描
（`_rv1_dir.py`，2° 步長）量到 v1 選到的方向比最佳值差：

    rect 5x4 n=8 +0.45% | L n=4 0.00% | L rot20 n=6 +1.70%
    star n=6     +5.78% | tri n=5    +2.98%      （max_km，越低越好）

矩形的候選角只有 [0°, 180°]（凸包邊就那兩個），但 38° 更好。

v2 改成：A 階段全角度粗掃（`_COARSE` 度）用**單機巡迴長度**排序（不做 DP、
不算緩衝區，~1.5 ms/角）；B 階段前 K 名 **∪ v1 的候選角**才做完整評估。
把 v1 的候選角無條件放進 B 階段是刻意的：這讓 v2 在方向這一軸上
**結構上不可能輸給 v1**（我第一版沒這麼做，L rot20 就退步 3.0%）。

改動二：領地從「全寬橫帶」改成二維方塊
--------------------------------------
v1 把全域蛇形條帶序列做連續切分，所以每台的領地必然是**跨越整個寬度的橫帶**
（實測 n=8 矩形，8 台的 bbox x 全是 [-2500, 2500]）。home 在區域中心時這很貴：
橫帶只能從 x=±2500 的條帶端點進入。

v2 增加一個候選：在**同一組全域掃描線**上（所以沒有接縫重複掃描），把領地切成
`r` 個列群 × 每列群 `c_g` 個 x 區段的方塊。r=n、c=1 就退化成 v1 的橫帶 ⇒ 嚴格推廣。
切點用「等航程初始化 + 座標局部搜尋」，目標 min-max，代價是真實折線長。

改動三：cell 串接加 2-opt / or-opt（v1 只有貪婪最近鄰）。

v2 永遠同時評估「v1 的一維 granular DP」與「方塊分配」兩個候選並取好的，
所以在同一個方向下也不會退步。
"""
from __future__ import annotations
from pathlib import Path

import math

from shapely.geometry import LineString
from shapely.ops import unary_union

import algo_bous_variants as V1

__all__ = ["plan", "plan_v2", "plan_v2_dir_only", "plan_v2_grid_only",
           "plan_v2_chain_only"]

_COARSE = 6.0            # A 階段粗掃步長（度），0..360（相位有意義，不 mod 180）
_REFINE = 1.5            # 粗掃最佳角附近的細化步長
_TOPK = 5                # B 階段完整評估的角度數（另外無條件加上 v1 的候選角）
_COV_MIN = 0.999         # 與 v1 相同的可行性閘門
_LS_ROUNDS = 6
_MAX_PARTS_GRID = 8


def _plen(t):
    return sum(math.dist(t[i], t[i - 1]) for i in range(1, len(t)))


def _fwd(th):
    c, s = math.cos(th), math.sin(th)
    return lambda p: (c * p[0] + s * p[1], -s * p[0] + c * p[1])


def _bak(th):
    c, s = math.cos(th), math.sin(th)
    return lambda p: (c * p[0] - s * p[1], s * p[0] + c * p[1])


# ── 方塊領地（旋轉座標系）─────────────────────────────────────────────
def _terr_track(rows, i0, i1, x0, x1, start_r, flip):
    """列 [i0,i1)、x 夾在 [x0,x1] 的領地 → 航跡點列（首尾都在 start_r）。"""
    idx = range(i1 - 1, i0 - 1, -1) if flip else range(i0, i1)
    pos, pts = start_r, [start_r]
    for i in idx:
        y, segs = rows[i]
        cl = []
        for a, b in segs:
            aa = a if a > x0 else x0
            bb = b if b < x1 else x1
            if bb - aa > 1e-6:
                cl.append((aa, bb))
        if not cl:
            continue
        if math.dist(pos, (cl[0][0], y)) > math.dist(pos, (cl[-1][1], y)):
            cl = cl[::-1]
        for aa, bb in cl:
            p, q = (aa, y), (bb, y)
            if math.dist(pos, p) > math.dist(pos, q):
                p, q = q, p
            pts += [p, q]
            pos = q
    if len(pts) < 2:
        return None
    pts.append(start_r)
    return pts


def _terr(rows, i0, i1, x0, x1, start_r):
    best = None
    for flip in (False, True):
        t = _terr_track(rows, i0, i1, x0, x1, start_r, flip)
        if t is None:
            continue
        L = _plen(t)
        if best is None or L < best[0]:
            best = (L, t)
    return best


def _row_len(rows, i, x0, x1):
    s = 0.0
    for a, b in rows[i][1]:
        aa = a if a > x0 else x0
        bb = b if b < x1 else x1
        if bb > aa:
            s += bb - aa
    return s


def _col_split(rows, i0, i1, m, start_r, lo, hi):
    """列群 [i0,i1) 的 x 範圍切成 m 個連續欄，min-max（等分初始化 + 局部搜尋）。"""
    if m <= 1:
        r = _terr(rows, i0, i1, lo, hi, start_r)
        return None if r is None else (r[0], [r[1]])
    span = hi - lo
    if span <= 0:
        return None
    # 等航程初始化比等寬好（凹形的列長差很多）
    tot = sum(_row_len(rows, i, lo, hi) for i in range(i0, i1))
    cuts = [lo]
    if tot > 0:
        probe = 400
        acc, k = 0.0, 1
        for t in range(probe):
            a = lo + span * t / probe
            b = lo + span * (t + 1) / probe
            acc += sum(_row_len(rows, i, a, b) for i in range(i0, i1))
            while k < m and acc >= tot * k / m:
                cuts.append(b)
                k += 1
    while len(cuts) < m:
        cuts.append(lo + span * len(cuts) / m)
    cuts.append(hi)
    cuts = sorted(cuts)

    def ev(cs):
        out, mx = [], 0.0
        for k in range(m):
            r = _terr(rows, i0, i1, cs[k], cs[k + 1], start_r)
            if r is None:
                return None
            out.append(r[1])
            if r[0] > mx:
                mx = r[0]
        return mx, out

    cur = ev(cuts)
    if cur is None:
        return None
    step = span / (2.0 * m)
    for _ in range(_LS_ROUNDS):
        moved = False
        for k in range(1, m):
            for d in (-step, step):
                trial = list(cuts)
                trial[k] = min(max(trial[k] + d, trial[k - 1] + 1e-3),
                               trial[k + 1] - 1e-3)
                r = ev(trial)
                if r is not None and r[0] < cur[0] - 1e-6:
                    cuts, cur, moved = trial, r, True
        if not moved:
            step /= 2.0
            if step < 1.0:
                break
    return cur


def _grid_part(rows, m, start_r):
    """一個連通塊 + m 台 ⇒ 列群 × 欄 的方塊分配。r=m/c=1 就是 v1 的全寬橫帶。"""
    R = len(rows)
    if R == 0 or m < 1:
        return None
    lo = min(a for _y, segs in rows for a, _b in segs)
    hi = max(b for _y, segs in rows for _a, b in segs)
    rl = [_row_len(rows, i, lo, hi) for i in range(R)]
    tot = sum(rl)
    if tot <= 0:
        return None

    best = None
    for r in range(1, min(m, R) + 1):
        cnt = [m // r + (1 if k < m % r else 0) for k in range(r)]
        tgt, acc, cuts, k = 0.0, 0.0, [0], 0
        want = [tot * c / m for c in cnt]
        for i in range(R):
            acc += rl[i]
            while k < r - 1 and acc >= sum(want[:k + 1]) - 1e-9:
                cuts.append(i + 1)
                k += 1
        cuts.append(R)
        cuts = sorted(set(cuts))
        if len(cuts) != r + 1:                    # 退回等列數
            cuts = [round(R * t / r) for t in range(r + 1)]
            cuts = sorted(set(cuts))
            if len(cuts) != r + 1:
                continue

        def ev(cs):
            outs, mx = [], 0.0
            for g in range(r):
                a, b = cs[g], cs[g + 1]
                if b <= a:
                    return None
                res = _col_split(rows, a, b, cnt[g], start_r, lo, hi)
                if res is None:
                    return None
                if res[0] > mx:
                    mx = res[0]
                outs += res[1]
            return mx, outs

        cur = ev(cuts)
        if cur is None:
            continue
        for _ in range(_LS_ROUNDS):
            moved = False
            for k in range(1, r):
                for d in (-1, 1):
                    trial = list(cuts)
                    trial[k] += d
                    if trial[k] <= trial[k - 1] or trial[k] >= trial[k + 1]:
                        continue
                    res = ev(trial)
                    if res is not None and res[0] < cur[0] - 1e-6:
                        cuts, cur, moved = trial, res, True
            if not moved:
                break
        if best is None or cur[0] < best[0]:
            best = cur
    return best


# ── cell 串接：貪婪最近鄰 + 2-opt / or-opt（v1 只有貪婪）───────────────
def _chain2(cells, pos):
    n = len(cells)
    if n <= 1:
        return V1._chain(cells, pos)
    ends = [(V1._mid(c[0]), V1._mid(c[-1])) for c in cells]

    def cost(order):
        p, s = pos, 0.0
        for t, rev in order:
            a, b = ends[t]
            if rev:
                a, b = b, a
            s += math.dist(p, a)
            p = b
        return s

    order, left, p = [], list(range(n)), pos
    while left:
        _d, t, rv = min(((math.dist(p, ends[t][1] if rv else ends[t][0]), t, rv)
                         for t in left for rv in (False, True)), key=lambda z: z[0])
        order.append((t, rv))
        p = ends[t][0] if rv else ends[t][1]
        left.remove(t)

    cur = cost(order)
    for _ in range(_LS_ROUNDS):
        improved = False
        for i in range(n):
            trial = list(order)
            trial[i] = (trial[i][0], not trial[i][1])
            c = cost(trial)
            if c < cur - 1e-6:
                order, cur, improved = trial, c, True
        for i in range(n - 1):
            for j in range(i + 1, n):
                trial = (order[:i] + [(t, not rr) for t, rr in reversed(order[i:j + 1])]
                         + order[j + 1:])
                c = cost(trial)
                if c < cur - 1e-6:
                    order, cur, improved = trial, c, True
        for i in range(n):
            for j in range(n):
                if i == j:
                    continue
                rest = order[:i] + order[i + 1:]
                trial = rest[:j] + [order[i]] + rest[j:]
                c = cost(trial)
                if c < cur - 1e-6:
                    order, cur, improved = trial, c, True
        if not improved:
            break

    flat, p = [], pos
    for t, rv in order:
        seq = cells[t][::-1] if rv else cells[t]
        flat += seq
        p = V1._mid(seq[-1])
    return flat, p


# ── 快取 ──────────────────────────────────────────────────────────────
class _Ctx:
    def __init__(self, parts, pitch, swath, start):
        self.parts, self.pitch, self.swath, self.start = parts, pitch, swath, start
        self.rows: dict = {}
        self.cells: dict = {}
        self.tour: dict = {}

    def rows_of(self, i, th):
        k = (i, round(th, 9))
        if k not in self.rows:
            self.rows[k] = V1._rows(self.parts[i], self.pitch, self.swath, th)
        return self.rows[k]

    def cells_of(self, i, th):
        k = (i, round(th, 9))
        if k not in self.cells:
            rw = self.rows_of(i, th)
            self.cells[k] = ([s for s in (V1._strips_of_cell(c, rw, th)
                                          for c in V1._cells(rw)) if s] if rw else [])
        return self.cells[k]

    def tour_len(self, i, th):
        """A 階段代理：單機把這個區塊飛完的巡迴長度（不做 DP、不算緩衝區）。"""
        k = (i, round(th, 9))
        if k not in self.tour:
            cs = self.cells_of(i, th)
            if not cs:
                self.tour[k] = float("inf")
            else:
                best = float("inf")
                for chain in (V1._chain, _chain2):
                    order, _e = chain(cs, self.start)
                    best = min(best, _plen(V1._build(
                        V1._orient(order, self.start), self.start)))
                self.tour[k] = best
        return self.tour[k]


def _ordered_strips(ctx, angles):
    """回傳**兩個**條帶序列候選：v1 的貪婪串接，與加了 2-opt/or-opt 的串接。

    2-opt 讓「單機巡迴」變短，但 min-max 分配之後**不保證**比較好 —— 隨機母體
    實測純換串接會輸 8/25 趟、最差 -12.6%。所以這裡不是取代而是**並列候選**，
    交給同一個目標函數決勝，v2 才在這一軸上也不會退步。
    """
    cells_all = []
    for i, th in enumerate(angles):
        cells_all += ctx.cells_of(i, th)
    if not cells_all:
        return []
    out = []
    for chain in (V1._chain, _chain2):
        order, _e = chain(cells_all, ctx.start)
        out.append(V1._orient(order, ctx.start))
    return out


def _alloc_grid(ctx, angles, n_uav):
    P = len(ctx.parts)
    if P > n_uav or P > _MAX_PARTS_GRID:
        return None
    rows_r, starts_r, baks = [], [], []
    for i, th in enumerate(angles):
        rw = ctx.rows_of(i, th)
        if not rw:
            return None
        rows_r.append(rw)
        starts_r.append(_fwd(th)(ctx.start))
        baks.append(_bak(th))

    memo: dict = {}

    def gp(i, m):
        if (i, m) not in memo:
            memo[(i, m)] = _grid_part(rows_r[i], m, starts_r[i])
        return memo[(i, m)]

    wl = [sum(_row_len(rows_r[i], j, -1e18, 1e18) for j in range(len(rows_r[i])))
          for i in range(P)]
    tw = sum(wl) or 1.0
    m = [max(1, int(n_uav * w / tw)) for w in wl]
    while sum(m) > n_uav:
        j = max(range(P), key=lambda i: (m[i], -wl[i]))
        if m[j] <= 1:
            break
        m[j] -= 1
    while sum(m) < n_uav:
        res = [gp(i, m[i]) for i in range(P)]
        if any(x is None for x in res):
            return None
        m[max(range(P), key=lambda i: res[i][0])] += 1

    for _ in range(P * 2):
        res = [gp(i, m[i]) for i in range(P)]
        if any(x is None for x in res):
            return None
        hi = max(range(P), key=lambda i: res[i][0])
        cands = [i for i in range(P) if i != hi and m[i] > 1]
        if not cands:
            break
        lo = min(cands, key=lambda i: res[i][0])
        trial = list(m)
        trial[hi] += 1
        trial[lo] -= 1
        rh, rl = gp(hi, trial[hi]), gp(lo, trial[lo])
        if rh is None or rl is None:
            break
        nm = max([res[i][0] for i in range(P) if i not in (hi, lo)] + [rh[0], rl[0]])
        if nm < res[hi][0] - 1e-6:
            m = trial
        else:
            break

    tracks = []
    for i in range(P):
        r = gp(i, m[i])
        if r is None:
            return None
        bk = baks[i]
        tracks += [[bk(p) for p in t] for t in r[1]]
    return tracks


def _coverage(parts, tracks, swath):
    area = unary_union(parts)
    if area.area <= 0 or not tracks:
        return 0.0
    sw = unary_union([LineString(t).buffer(swath / 2.0, quad_segs=8)
                      for t in tracks if len(t) >= 2])
    return area.intersection(sw).area / area.area


def _score(tracks, cov):
    lens = [_plen(t) for t in tracks]
    if not lens:
        return (1, float("inf"), float("inf"))
    return (0 if cov >= _COV_MIN else 1, max(lens), sum(lens))


def _full_eval(ctx, angles, n_uav, use_grid, use_1d):
    """完整評估：一維 granular DP（= v1）與方塊分配都算，取好的。"""
    best = None
    if use_1d:
        for strips in _ordered_strips(ctx, angles):
            if not strips:
                continue
            tr = V1._alloc_contiguous(strips, n_uav, ctx.start, True)
            sc = _score(tr, _coverage(ctx.parts, tr, ctx.swath))
            if best is None or sc < best[0]:
                best = (sc, tr)
    if use_grid:
        tr = _alloc_grid(ctx, angles, n_uav)
        if tr:
            sc = _score(tr, _coverage(ctx.parts, tr, ctx.swath))
            if best is None or sc < best[0]:
                best = (sc, tr)
    return best


def _frange(a, b, s):
    out, x = [], a
    while x < b - 1e-9:
        out.append(x)
        x += s
    return out


def _plan(polys, n_uav, swath_m, pitch_m, start, use_grid, use_dir):
    parts = V1._parts(polys)
    if not parts or n_uav < 1:
        return []
    ctx = _Ctx(parts, pitch_m, swath_m, start)
    P = len(parts)
    v1_cand = [V1._cand_angles(p) for p in parts]
    cur = [c[0] for c in v1_cand]

    if use_dir:
        for i in range(P):
            # A 階段：全角度粗掃 + 局部細化，用單機巡迴長度排序（便宜）
            pool = [math.radians(d) for d in _frange(0.0, 360.0, _COARSE)]
            ranked = sorted(pool, key=lambda th: ctx.tour_len(i, th))
            top = ranked[0]
            pool += [top + math.radians(d)
                     for d in _frange(-_COARSE, _COARSE + 1e-9, _REFINE)]
            ranked = sorted(set(pool), key=lambda th: ctx.tour_len(i, th))
            # B 階段：前 K 名 ∪ v1 的候選角（後者保證 v2 不會輸給 v1）
            short = list(dict.fromkeys(ranked[:_TOPK] + list(v1_cand[i]) + [cur[i]]))
            bf, bth = None, cur[i]
            for th in short:
                trial = list(cur)
                trial[i] = th
                r = _full_eval(ctx, trial, n_uav, use_grid, True)
                if r is not None and (bf is None or r[0] < bf[0]):
                    bf, bth = r, th
            cur[i] = bth

    r = _full_eval(ctx, cur, n_uav, use_grid, True)
    return r[1] if r else []


# ── 對外 ──────────────────────────────────────────────────────────────
def plan_v2(polys, n_uav, *, swath_m, pitch_m, start=(0.0, 0.0)):
    return _plan(polys, n_uav, swath_m, pitch_m, start, True, True)


def plan_v2_dir_only(polys, n_uav, *, swath_m, pitch_m, start=(0.0, 0.0)):
    """只加全角度方向搜尋（分配仍是 v1 的一維 DP）⇒ 隔離「方向」的價值。"""
    return _plan(polys, n_uav, swath_m, pitch_m, start, False, True)


def plan_v2_grid_only(polys, n_uav, *, swath_m, pitch_m, start=(0.0, 0.0)):
    """只加方塊分配（方向仍用 v1 的候選角）⇒ 隔離「分配」的價值。"""
    return _plan(polys, n_uav, swath_m, pitch_m, start, True, False)


def plan_v2_chain_only(polys, n_uav, *, swath_m, pitch_m, start=(0.0, 0.0)):
    """只加 cell 串接候選（方向與分配都用 v1 的）⇒ 隔離「串接」的價值。"""
    return _plan(polys, n_uav, swath_m, pitch_m, start, False, False)


plan = plan_v2


# ── 自檢 ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import sys
    import time

    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:                                    # noqa: BLE001
        pass
    # ⚠ 原本硬編 session scratchpad 的路徑（會消失），改成 repo 相對。
    HERE = str(Path(__file__).resolve().parent)
    if HERE not in sys.path:
        sys.path.insert(0, HERE)
    from shapely import affinity
    from shapely.geometry import Polygon

    from bench import boustrophedon, metrics, rect_xy          # noqa: E402

    ALT, HFOV = 140.0, 60.0
    SWATH = 2 * ALT * math.tan(math.radians(HFOV) / 2)
    PITCH = SWATH * 0.7

    def L_shape(a=4000.0, b=2000.0):
        return Polygon([(-a / 2, -a / 2), (a / 2, -a / 2), (a / 2, -a / 2 + b),
                        (-a / 2 + b, -a / 2 + b), (-a / 2 + b, a / 2), (-a / 2, a / 2)])

    def star(k=5, r0=2600, r1=1100):
        return Polygon([((r0 if i % 2 == 0 else r1) * math.cos(math.pi * i / k),
                         (r0 if i % 2 == 0 else r1) * math.sin(math.pi * i / k))
                        for i in range(2 * k)])

    SC = [
        ("(1) rect 5.0x4.0 km n=8", [rect_xy(0, 0, 2500, 2000)], 8),
        ("(2) four 1.25x4.0 km blocks n=8",
         [rect_xy(-2400, 0, 625, 2000), rect_xy(-800, 0, 625, 2000),
          rect_xy(800, 0, 625, 2000), rect_xy(2400, 0, 625, 2000)], 8),
        ("(3) L concave 12 km2 n=4", [L_shape()], 4),
        ("(4) rect rot30 n=8",
         [affinity.rotate(rect_xy(0, 0, 2500, 2000), 30, origin=(0, 0))], 8),
        ("(5) L rot20 n=6", [affinity.rotate(L_shape(), 20, origin=(0, 0))], 6),
        ("(6) star concave n=6", [star()], 6),
        ("(7) triangle n=5", [Polygon([(-2600, -1600), (2800, -1400), (-200, 2400)])], 5),
    ]
    HDR = (f"{'method':22s}{'uav':>4s}{'tot km':>8s}{'max km':>8s}{'cover':>8s}"
           f"{'ovhd':>7s}{'redun':>7s}{'bal':>7s}{'scan km2/h':>12s}{'waste':>8s}{'s':>7s}")

    def show(name, polys, fn):
        t0 = time.perf_counter()
        tr = fn()
        dt = time.perf_counter() - t0
        m = metrics(polys, tr, SWATH)
        print(f"{name:22s}{m['n_uav']:4d}{m['total_km']:8.1f}{m['max_km']:8.1f}"
              f"{m['coverage'] * 100:7.1f}%{m['overhead'] or 0:7.3f}"
              f"{m['redundancy'] or 0:7.3f}{m['balance'] or 0:7.3f}"
              f"{m['scan_rate']:12.2f}{(m['waste'] or 0) * 100:7.1f}%{dt:7.2f}")
        return m

    print(f"swath {SWATH:.1f} m   pitch {PITCH:.1f} m   speed 20 m/s\n")
    gains = []
    for title, polys, n in SC:
        print(title)
        print(HDR)
        kw = dict(swath_m=SWATH, pitch_m=PITCH)
        show("bench baseline", polys, lambda: boustrophedon(polys, n, **kw))
        a = show("v1 contiguous", polys, lambda: V1.plan_contiguous(polys, n, **kw))
        b = show("v2_rev1", polys, lambda: plan_v2(polys, n, **kw))
        gains.append((title, a["scan_rate"], b["scan_rate"]))
        print()
    print(f"{'scenario':34s}{'v1':>8s}{'v2':>8s}{'gain%':>8s}")
    for t, a, b in gains:
        print(f"{t:34s}{a:8.2f}{b:8.2f}{(b - a) / a * 100:+8.2f}")
