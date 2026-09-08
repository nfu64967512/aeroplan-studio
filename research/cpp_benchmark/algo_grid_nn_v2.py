"""grid_nn_v2 —— grid_nn 的改進版（審查者第 1 位提出）。

與 v1 相同的骨架：網格分解 → 容量受限 k-means（兩種容量政策）→ 以 makespan
為目標的再平衡 → NN + 2-opt + Or-opt 巡迴。v1 的**求解器端已經幾乎收斂**
（場內航程只比「格數 × 格距」的粗下界高 2.0%；八重啟只再省 0～1.1%），
所以 v2 動的是**模型端**，不是求解器端。

v2 的三項改動
-------------
1. **視點內縮（主要收益）**。v1 把每格的**幾何中心**當必訪點，於是每一列的兩端
   都伸出多邊形外約半格；矩形情境實測每列多飛 95 m × 36 列 = 3.4 km。
   v2 把視點沿「鄰格質心方向」往場內拉，拉到不能再拉為止，上限是一條
   **與 v1 等價的覆蓋緊度不變式**：

       max_{v ∈ 裁切後的格} |v − 視點|  ≤  cell·√2/2

   （= 滿格用中心時的最遠角距離）。所以 v2 的每一格覆蓋都**不比 v1 鬆**，
   覆蓋率只可能持平或變好（三情境 + 十個健全性案例實測全是 1.000000）。
   滿格的中心恰好落在等號上 ⇒ 二分搜尋不會動它；只有**碎格**會往內縮。

2. **再平衡多一層單格搬移**。v1 只從最長那台搬「離目標群質心最近的 k 格」給
   前三短的群（k = gap/(4·cell) 起手再折半），折半到 1 還不行整輪就停。
   v2 在那之後補一輪：把最長那台的邊界格試搬給**任何**較短的群，還是沒改善才停。

3. **輸出合併共線點**。v1 每格一個航點（情境(1) 1636 點）；把共線且同向的中間點
   拿掉不改變折線幾何、不改變 bench 的任何指標，只是把航點數砍半。

⚠ 量過但**沒有**採用的兩件事（留紀錄免得別人重跑）
  * 格網角度掃描（目標 = 最小化格數）：矩形 / 長條 / L / 旋轉 37° 的 L，最佳角
    **就是**最小寬度角；三角形也只省 0.34 km / 37 km。維持 v1 作法。
  * k-means 多重啟（8 個種子）：情境(1) +0.00%、(2) +0.27%、(3) +1.1%，時間 ×8。
    不划算，維持單一固定種子（也保住決定性）。

⚠⚠ v1 自陳的最大弱點（航跡不可飛）v2 **沒有解決，而且略為惡化**。
  情境(1) >60° 轉折 v1 754 → v2 930：內縮讓視點離開完美格點，巡迴更容易走成
  階梯狀。實測「純列蛇形」巡迴在 k-means 的不規則群上長度反而 +14%、
  >60° 轉折仍有 653 個 ⇒ 那不是出路；真正的出路是改成條帶狀分群，
  那已經是另一個演算法而不是這一支的改良。
  **所以：只要比較加上轉彎半徑 / Dubins 成本，v2 的優勢會比 v1 縮水得更快。**

⚠ 代價：時間。情境(1) v1 1.1 s → v2 6.6 s；80 km²/n=12 是 13 s → 67 s。
  幾乎全部來自新增的單格再平衡（每次搬移都要重解兩條巡迴）。離線基準可接受。
  `_REBAL_CAND` 調小可換回時間，但情境(1) 掃描率會從 58.94 掉到 58.74（≈1% 的
  平手打破噪音，不是系統性效應）。

只用 shapely + numpy + 標準庫。

三情境實測（全部由 bench.metrics 算，重跑 `PYTHONUTF8=1 python -u run_grid_nn_v2.py`）
    情境                 v1 掃描率 → v2 掃描率
    (1) rect 5x4 n=8        58.17 → 58.94   (+1.33%)
    (2) four 1.25x4 n=8     48.36 → 51.84   (+7.20%)
    (3) L 形 n=4            28.69 → 29.52   (+2.88%)
    (4) 含洞方環 n=5         35.85 → 37.41   (+4.34%)
    (5) 三角形 n=6           27.24 → 29.82   (+9.50%)
    (6) L 旋轉 37° n=4       28.61 → 29.38   (+2.72%)
    (7) rect n=16           93.74 → 98.09   (+4.65%)
    (8) rect n=1             7.83 →  8.09   (+3.31%)
"""
from __future__ import annotations

import math
import random
from collections import deque

import numpy as np
from shapely.affinity import rotate as _rotate
from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union

KEY = "grid_nn_v2"

_SEED = 12345
_NEIGH_K = 12
_EPS = 1e-7
_ORO_MAX = 3
_REBAL_ROUNDS = 40
_REBAL_SINGLE = 40     # v2 新增的「單格搬移」最多做幾次（純粹是時間護欄）
_REBAL_CAND = 64       # 單格搬移每輪只試最短的這幾個群
_PULL_R = 1.6          # 內縮目標 = 這個半徑（單位 cell）內鄰格的質心
_PULL_ITER = 24        # 內縮的二分搜尋次數


# ── 幾何小工具（與 v1 相同）────────────────────────────────────────────
def _geoms(poly):
    return list(getattr(poly, "geoms", [poly]))


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


def _cell_size(pitch_m: float, swath_m: float) -> float:
    return max(1e-3, min(pitch_m, swath_m / math.sqrt(2.0)))


def _build_cells(polys, cell: float):
    centers, clipped, frac = [], [], []
    for poly in polys:
        for g in _geoms(poly):
            if g.is_empty or g.area <= 0:
                continue
            th = _min_width_angle(g)
            deg = math.degrees(th)
            loc = _rotate(g, -deg, origin=(0.0, 0.0), use_radians=False)
            x0, y0, x1, y1 = loc.bounds
            nx = max(1, int(math.ceil((x1 - x0) / cell)))
            ny = max(1, int(math.ceil((y1 - y0) / cell)))
            ox = x0 - (nx * cell - (x1 - x0)) / 2.0
            oy = y0 - (ny * cell - (y1 - y0)) / 2.0
            for iy in range(ny):
                for ix in range(nx):
                    cx, cy = ox + (ix + 0.5) * cell, oy + (iy + 0.5) * cell
                    b = box(cx - cell / 2, cy - cell / 2, cx + cell / 2, cy + cell / 2)
                    inter = loc.intersection(b)
                    if inter.is_empty or inter.area <= 1e-9:
                        continue
                    pt = _rotate(Point(cx, cy), deg, origin=(0.0, 0.0))
                    centers.append((pt.x, pt.y))
                    clipped.append(_rotate(inter, deg, origin=(0.0, 0.0)))
                    frac.append(inter.area / (cell * cell))
    return centers, clipped, frac


def _prune(centers, clipped, frac, swath_m: float, cell: float):
    n = len(centers)
    if n <= 1:
        return list(range(n))
    P = np.asarray(centers, float)
    r = 0.99 * swath_m / 2.0
    reach = r + cell * math.sqrt(2.0)
    keep = [True] * n
    for i in sorted(range(n), key=lambda i: frac[i]):
        if frac[i] >= 0.35:
            break
        d = np.hypot(P[:, 0] - P[i, 0], P[:, 1] - P[i, 1])
        cand = [j for j in np.nonzero(d < reach)[0].tolist() if j != i and keep[j]]
        if not cand:
            continue
        cover = unary_union([Point(centers[j]).buffer(r, quad_segs=32) for j in cand])
        if clipped[i].difference(cover).area <= 1e-6 * max(clipped[i].area, 1.0):
            keep[i] = False
    return [i for i in range(n) if keep[i]]


# ── v2 新增①：視點內縮 ────────────────────────────────────────────────
def _verts(g):
    if g.is_empty:
        return []
    out = []
    for p in _geoms(g):
        if p.geom_type == "Polygon":
            out += list(p.exterior.coords)
            for r in p.interiors:
                out += list(r.coords)
        else:
            out += list(getattr(p, "coords", []))
    return out


def _pull_in(centers, clipped, cell: float):
    """把視點往場內拉；上限 = 與 v1 等價的覆蓋緊度（滿格中心到角的距離）。"""
    n = len(centers)
    if n <= 1:
        return list(centers), 0
    P = np.asarray(centers, float)
    r2 = (cell * math.sqrt(2.0) / 2.0) ** 2 + 1e-9
    rad = _PULL_R * cell
    out, moved = [], 0
    for i in range(n):
        ci = centers[i]
        vs = _verts(clipped[i])
        if not vs:
            out.append(ci)
            continue
        if max((ci[0] - x) ** 2 + (ci[1] - y) ** 2 for x, y in vs) > r2:
            out.append(ci)                      # 連原中心都不滿足 ⇒ 保守不動
            continue
        d2 = (P[:, 0] - ci[0]) ** 2 + (P[:, 1] - ci[1]) ** 2
        nb = np.nonzero((d2 < rad * rad) & (d2 > 1e-12))[0]
        if nb.size == 0:
            out.append(ci)
            continue
        tx, ty = P[nb].mean(0)
        dx, dy = tx - ci[0], ty - ci[1]
        if dx * dx + dy * dy < 1e-12:
            out.append(ci)
            continue
        lo, hi = 0.0, 1.0
        for _ in range(_PULL_ITER):
            mid = 0.5 * (lo + hi)
            px, py = ci[0] + mid * dx, ci[1] + mid * dy
            if max((px - x) ** 2 + (py - y) ** 2 for x, y in vs) <= r2:
                lo = mid
            else:
                hi = mid
        p = (ci[0] + lo * dx, ci[1] + lo * dy)
        if (p[0] - ci[0]) ** 2 + (p[1] - ci[1]) ** 2 > 1e-12:
            moved += 1
        out.append(p)
    return out, moved


# ── 分群（與 v1 相同）─────────────────────────────────────────────────
def _kmeanspp(P, k, rng):
    n = len(P)
    idx = [rng.randrange(n)]
    d2 = ((P - P[idx[0]]) ** 2).sum(1)
    for _ in range(k - 1):
        tot = float(d2.sum())
        if tot <= 0:
            idx.append(rng.randrange(n))
        else:
            t, acc, pick = rng.random() * tot, 0.0, n - 1
            for j in range(n):
                acc += float(d2[j])
                if acc >= t:
                    pick = j
                    break
            idx.append(pick)
        d2 = np.minimum(d2, ((P - P[idx[-1]]) ** 2).sum(1))
    return P[idx].copy()


def _caps(P, cent, depot, cell):
    n, k = len(P), len(cent)
    d = np.hypot(cent[:, 0] - depot[0], cent[:, 1] - depot[1])
    base = n / k
    want = np.clip(base + 2.0 * (d.mean() - d) / max(cell, 1e-6),
                   max(1.0, 0.55 * base), 1.6 * base)
    caps = np.maximum(1, np.round(want)).astype(int)
    while caps.sum() < n:
        caps[int(np.argmin(caps - want))] += 1
    while caps.sum() > n:
        j = int(np.argmax(caps - want))
        if caps[j] <= 1:
            j = int(np.argmax(caps))
        caps[j] -= 1
    return caps.tolist()


def _caps_equal(n, k):
    caps = [n // k] * k
    for i in range(n - sum(caps)):
        caps[i] += 1
    return caps


def _assign(P, cent, caps):
    n, k = len(P), len(cent)
    D = np.sqrt(((P[:, None, :] - cent[None, :, :]) ** 2).sum(-1))
    order_pref = np.argsort(D, axis=1)
    if k >= 2:
        srt = np.sort(D, axis=1)
        regret = srt[:, 1] - srt[:, 0]
    else:
        regret = np.zeros(n)
    lab = np.full(n, -1, int)
    left = list(caps)
    for i in np.argsort(-regret):
        i = int(i)
        for c in order_pref[i]:
            c = int(c)
            if left[c] > 0:
                lab[i], left[c] = c, left[c] - 1
                break
        if lab[i] < 0:
            lab[i] = int(order_pref[i][0])
    return lab


def _cluster(P, k, depot, cell, *, depot_caps=True):
    n = len(P)
    if k <= 1 or n <= 1:
        return [list(range(n))]
    if n <= k:
        return [[i] for i in range(n)]
    rng = random.Random(_SEED)
    cent = _kmeanspp(P, k, rng)
    lab = None
    for _ in range(20):
        caps = _caps(P, cent, depot, cell) if depot_caps else _caps_equal(n, k)
        new = _assign(P, cent, caps)
        if lab is not None and np.array_equal(new, lab):
            break
        lab = new
        for c in range(k):
            m = lab == c
            if m.any():
                cent[c] = P[m].mean(0)
    return [np.nonzero(lab == c)[0].tolist() for c in range(k)]


# ── 巡迴（與 v1 相同）─────────────────────────────────────────────────
def _mat(P):
    d = np.sqrt(((P[:, None, :] - P[None, :, :]) ** 2).sum(-1))
    K = min(_NEIGH_K, len(P) - 1)
    idx = np.argsort(d, axis=1)
    # ⚠ 不可寫成 [:, 1:K+1]：有重合點時 argsort 第 0 欄不一定是自己（v1 的註解）
    neigh = [[int(j) for j in row if int(j) != i][:K] for i, row in enumerate(idx)]
    return (d.tolist() if len(P) <= 1500 else d), neigh


def _nn_tour(D, m):
    unv = set(range(1, m))
    tour, cur = [0], 0
    while unv:
        row = D[cur]
        nxt = min(unv, key=lambda j: row[j])
        tour.append(nxt)
        unv.discard(nxt)
        cur = nxt
    return tour


def _tour_len(tour, D):
    m = len(tour)
    return sum(D[tour[i]][tour[(i + 1) % m]] for i in range(m))


def _rev(tour, pos, i, j):
    m = len(tour)
    n_in = (j - i) % m + 1
    if 2 * n_in > m:
        i, j = (j + 1) % m, (i - 1) % m
        n_in = m - n_in
    for _ in range(n_in // 2):
        a, b = tour[i], tour[j]
        tour[i], tour[j] = b, a
        pos[a], pos[b] = j, i
        i = (i + 1) % m
        j = (j - 1) % m


def _two_opt(tour, D, neigh, pos):
    m = len(tour)
    q = deque(tour)
    inq = [True] * m
    any_move = False
    while q:
        a = q.popleft()
        inq[a] = False
        Da = D[a]
        for direction in (1, -1):
            b = tour[(pos[a] + direction) % m]
            dab = Da[b]
            moved = False
            for c in neigh[a]:
                dac = Da[c]
                if dac >= dab:
                    break
                e = tour[(pos[c] + direction) % m]
                if e == a or c == b:
                    continue
                if dab + D[c][e] - dac - D[b][e] > _EPS:
                    if direction == 1:
                        _rev(tour, pos, pos[b], pos[c])
                    else:
                        _rev(tour, pos, pos[c], pos[b])
                    for x in (a, b, c, e):
                        if not inq[x]:
                            q.append(x)
                            inq[x] = True
                    moved = any_move = True
                    break
            if moved:
                break
    return any_move


def _or_opt(tour, D, neigh, pos):
    m = len(tour)
    if m < 5:
        return False
    any_move = False
    q = deque(tour)
    inq = [True] * m
    while q:
        s1 = q.popleft()
        inq[s1] = False
        for L in range(1, min(_ORO_MAX, m - 3) + 1):
            i = pos[s1]
            seg = [tour[(i + t) % m] for t in range(L)]
            sL = seg[-1]
            p = tour[(i - 1) % m]
            nx = tour[(i + L) % m]
            if nx == p:
                continue
            g_rem = D[p][s1] + D[sL][nx] - D[p][nx]
            if g_rem <= _EPS:
                continue
            inseg = set(seg)
            best = None
            for flip, (head, tail) in enumerate(((s1, sL), (sL, s1))):
                for c in neigh[head]:
                    if c in inseg or c == p:
                        continue
                    e = tour[(pos[c] + 1) % m]
                    if e in inseg:
                        continue
                    add = D[c][head] + D[tail][e] - D[c][e]
                    if g_rem - add > _EPS and (best is None or add < best[0]):
                        best = (add, c, bool(flip))
            if best is None:
                continue
            _, c, rev = best
            keep = [x for x in tour if x not in inseg]
            put = seg[::-1] if rev else seg
            k = keep.index(c)
            tour[:] = keep[:k + 1] + put + keep[k + 1:]
            for t, x in enumerate(tour):
                pos[x] = t
            for x in (p, nx, c, s1, sL):
                if not inq[x]:
                    q.append(x)
                    inq[x] = True
            any_move = True
            break
    return any_move


def _dedupe(pts):
    out, seen = [], set()
    for p in pts:
        k = (round(p[0], 6), round(p[1], 6))
        if k in seen:
            continue
        seen.add(k)
        out.append((float(p[0]), float(p[1])))
    return out


def _solve(pts):
    pts = _dedupe(pts)
    m = len(pts)
    if m <= 2:
        return list(pts), (2.0 * math.dist(pts[0], pts[1]) if m == 2 else 0.0)
    D, neigh = _mat(np.asarray(pts, float))
    tour = _nn_tour(D, m)
    pos = [0] * m
    for t, c in enumerate(tour):
        pos[c] = t
    for _ in range(30):
        a = _two_opt(tour, D, neigh, pos)
        b = _or_opt(tour, D, neigh, pos)
        if not (a or b):
            break
    L = _tour_len(tour, D)
    k = tour.index(0)
    seq = tour[k:] + tour[:k]
    return [pts[i] for i in seq], L


# ── v2 新增②：再平衡多一層單格搬移 ─────────────────────────────────────
def _rebalance(groups, centers, start, cell):
    P = np.asarray(centers, float)
    live = [g for g in groups if g]
    if len(live) < 2:
        return groups
    cache = {}

    def solve(g):
        k = tuple(sorted(g))
        if k not in cache:
            cache[k] = _solve([start] + [centers[i] for i in g])[1]
        return cache[k]

    lens = [solve(g) for g in live]
    singles = 0
    for _ in range(_REBAL_ROUNDS):
        hi = int(np.argmax(lens))
        cent = {i: P[live[i]].mean(0) for i in range(len(live))}
        moved = False
        # (a) v1 的區塊搬移
        for lo in sorted((i for i in range(len(live))
                          if i != hi and lens[i] < lens[hi] - 2 * cell),
                         key=lambda i: lens[i])[:3]:
            k = max(1, min(len(live[hi]) - 1,
                           int((lens[hi] - lens[lo]) / (4.0 * cell))))
            while k >= 1 and not moved:
                tgt = cent[lo]
                mv = set(sorted(live[hi], key=lambda i: (P[i][0] - tgt[0]) ** 2
                                + (P[i][1] - tgt[1]) ** 2)[:k])
                nh = [i for i in live[hi] if i not in mv]
                nl = live[lo] + sorted(mv)
                lh, ll_ = solve(nh), solve(nl)
                if max(lh, ll_) < lens[hi] - 1e-6:
                    live[hi], live[lo] = nh, nl
                    lens[hi], lens[lo] = lh, ll_
                    moved = True
                k //= 2
            if moved:
                break
        # (b) v2 新增：單格搬給較短的群（v1 只搬區塊，且只試前三短）
        if not moved and len(live[hi]) > 1 and singles < _REBAL_SINGLE:
            best = None
            for lo in sorted((i for i in range(len(live))
                              if i != hi and lens[i] < lens[hi] - 1e-6),
                             key=lambda i: lens[i])[:_REBAL_CAND]:
                tgt = cent[lo]
                for i in sorted(live[hi], key=lambda i: (P[i][0] - tgt[0]) ** 2
                                + (P[i][1] - tgt[1]) ** 2)[:3]:
                    nh = [j for j in live[hi] if j != i]
                    nl = live[lo] + [i]
                    v = max(solve(nh), solve(nl))
                    if v < lens[hi] - 1e-6 and (best is None or v < best[0]):
                        best = (v, lo, nh, nl)
            if best is not None:
                _, lo, nh, nl = best
                live[hi], live[lo] = nh, nl
                lens[hi], lens[lo] = solve(nh), solve(nl)
                moved = True
                singles += 1
        if not moved:
            break
    return live


# ── v2 新增③：輸出合併共線點（不改幾何、不改任何指標）──────────────────
def _merge_collinear(t, tol=1e-6):
    if len(t) < 3:
        return t
    out = [t[0]]
    for i in range(1, len(t) - 1):
        ax, ay = t[i][0] - out[-1][0], t[i][1] - out[-1][1]
        bx, by = t[i + 1][0] - t[i][0], t[i + 1][1] - t[i][1]
        na, nb = math.hypot(ax, ay), math.hypot(bx, by)
        if na > 1e-9 and nb > 1e-9 and ax * bx + ay * by > 0 \
                and abs(ax * by - ay * bx) <= tol * na * nb:
            continue
        out.append(t[i])
    out.append(t[-1])
    return out


# ── 對外介面 ──────────────────────────────────────────────────────────
def plan(polys, n_uav, *, swath_m, pitch_m, start=(0.0, 0.0)):
    """回傳每機一條折線（平面座標、公尺），起點與終點都在 start。"""
    n_uav = max(1, int(n_uav))
    cell = _cell_size(float(pitch_m), float(swath_m))
    centers, clipped, frac = _build_cells(polys, cell)
    if not centers:
        return []
    keep = _prune(centers, clipped, frac, float(swath_m), cell)
    centers = [centers[i] for i in keep]
    clipped = [clipped[i] for i in keep]
    centers, _moved = _pull_in(centers, clipped, cell)
    P = np.asarray(centers, float)
    best = None
    for dc in (True, False):
        groups = _rebalance(_cluster(P, n_uav, start, cell, depot_caps=dc),
                            centers, start, cell)
        tr, mk = [], 0.0
        for g in groups:
            if not g:
                continue
            seq, L = _solve([tuple(start)] + [centers[i] for i in g])
            if len(seq) < 2:
                continue
            tr.append(_merge_collinear(list(seq) + [tuple(start)]))
            mk = max(mk, L)
        if tr and (best is None or mk < best[0] - 1e-9):
            best = (mk, tr)
        if n_uav <= 1:
            break
    return best[1] if best else []
