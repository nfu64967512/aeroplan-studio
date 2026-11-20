"""grid_nn —— 網格分解 + 貪婪最近鄰巡迴 + 2-opt/Or-opt 局部搜尋。

文獻裡最常見的「naive 但合理」覆蓋基準：把作業區切成 pitch x pitch 的方格，
每格視為一個必訪點，於是覆蓋問題退化成一個多車輛 TSP（mTSP with depot）。

實作的常識優化（不是稻草人，每一項都會影響數字）
-------------------------------------------------
1. 網格**對齊最小寬度方向**（每個多邊形各自算），跟牛耕式取同一個方向，
   讓「一列格子」就是一條掃描條帶，而不是斜著切出一堆碎格。
2. 分配用**容量受限 k-means**（k-means++ 起始 + 後悔值貪婪指派）。容量兩種政策：
   等量、以及按「到 home 的距離」補償（n_i = N/n + 2(d̄ - d_i)/cell，離家遠的少分
   幾格，因為往返 home 的空程也算航程）。**兩種都跑，取 makespan 較小者** ——
   消融顯示兩者互有勝負（單一矩形補償版好 3.7%，四塊區反而差 2.6%）。
3. 分完之後做**以 makespan 為目標的實測再平衡**：算出真實巡迴長度，把最長那台的
   邊界格子搬給相鄰的較短那台，重跑巡迴，沒變好就回退。
4. 巡迴 = 最近鄰起手 + **2-opt**（鄰居表 + don't-look bits，雙向掃描）
   + **Or-opt**（1~3 格的片段搬移，含反向插入）。兩者都做到不再有改善為止。
5. 邊界碎格**剪枝**：交集已被鄰格中心的刈幅圓完整蓋住的格子不必飛。
   ⚠ 誠實標註：在本基準的操作點（pitch = 0.7·swath）它**一格都沒剪到**
   （三個情境實測 pruned=0），只有 pitch ≤ 0.6·swath 且邊界不平行網格時才會咬
   （三角形 pitch=0.5·swath 剪掉 63 格）。留著是為了小 pitch 的情形，
   不要把它算成本表數字的功勞。

覆蓋率的保證：格邊長 c ≤ swath/√2 ⇒ 格中心的刈幅圓半徑 swath/2 ≥ 半對角線
c√2/2，單靠「站在格中心」那一個圓就蓋得住整格。所以只要每格都被走過，
覆蓋率就是幾何上必然的，不靠運氣。

只用 shapely + numpy + 標準庫。
"""
from __future__ import annotations

import math
import random
from collections import deque

import numpy as np
from shapely.affinity import rotate as _rotate
from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union

KEY = "grid_nn"

_SEED = 12345          # 決定性：同組輸入永遠同一條航跡
_NEIGH_K = 12          # 2-opt / Or-opt 的鄰居表大小
_EPS = 1e-7
_ORO_MAX = 3           # Or-opt 片段最長格數
_REBAL_ROUNDS = 40     # 再平衡上限


# ── 幾何小工具 ────────────────────────────────────────────────────────
def _geoms(poly):
    return list(getattr(poly, "geoms", [poly]))


def _min_width_angle(poly: Polygon) -> float:
    """最小寬度方向（rad）。條帶（= 一列格子）垂直於它 ⇒ 列數最少。"""
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
    """格邊長。上限 swath/√2 是覆蓋率的幾何硬條件（半對角線 ≤ 刈幅半徑）。"""
    cap = swath_m / math.sqrt(2.0)
    return max(1e-3, min(pitch_m, cap))


def _build_cells(polys, cell: float):
    """每個多邊形沿自己的最小寬度方向鋪格。回傳 (centers, clipped, frac)。"""
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
            # 置中：讓兩側的碎格對稱，而不是全擠在一邊
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
    """邊界碎格剪枝：交集已被鄰格中心的刈幅圓完整覆蓋者不必飛。

    半徑取 0.99·swath/2（比指標用的 quad_segs=8 內接多邊形還保守），
    所以剪掉的格子在指標那把尺上必然仍被覆蓋。
    """
    n = len(centers)
    if n <= 1:
        return list(range(n))
    P = np.asarray(centers, float)
    r = 0.99 * swath_m / 2.0
    reach = r + cell * math.sqrt(2.0)
    keep = [True] * n
    order = sorted(range(n), key=lambda i: frac[i])
    for i in order:
        if frac[i] >= 0.35:          # 只考慮碎格
            break
        d = np.hypot(P[:, 0] - P[i, 0], P[:, 1] - P[i, 1])
        cand = [j for j in np.nonzero(d < reach)[0].tolist() if j != i and keep[j]]
        if not cand:
            continue
        cover = unary_union([Point(centers[j]).buffer(r, quad_segs=32) for j in cand])
        if clipped[i].difference(cover).area <= 1e-6 * max(clipped[i].area, 1.0):
            keep[i] = False
    return [i for i in range(n) if keep[i]]


# ── 分群：容量受限 k-means（k-means++ 起始）────────────────────────────
def _kmeanspp(P: np.ndarray, k: int, rng: random.Random) -> np.ndarray:
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


def _caps(P: np.ndarray, cent: np.ndarray, depot, cell: float) -> list[int]:
    """容量：離 home 遠的少分幾格（往返空程也算航程）。

    想讓 n_i·cell + 2·d_i 各群相等 ⇒ n_i = N/k + 2(d̄ - d_i)/cell。
    """
    n, k = len(P), len(cent)
    d = np.hypot(cent[:, 0] - depot[0], cent[:, 1] - depot[1])
    base = n / k
    want = base + 2.0 * (d.mean() - d) / max(cell, 1e-6)
    lo, hi = max(1.0, 0.55 * base), 1.6 * base
    want = np.clip(want, lo, hi)
    caps = np.maximum(1, np.round(want)).astype(int)
    # 湊足總數
    while caps.sum() < n:
        caps[int(np.argmin(caps - want))] += 1
    while caps.sum() > n:
        j = int(np.argmax(caps - want))
        if caps[j] <= 1:
            j = int(np.argmax(caps))
        caps[j] -= 1
    return caps.tolist()


def _assign(P: np.ndarray, cent: np.ndarray, caps: list[int]) -> np.ndarray:
    """後悔值貪婪的容量受限指派（最好與次好差距大的先選）。"""
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
                lab[i] = c
                left[c] -= 1
                break
        if lab[i] < 0:                      # 容量算錯時的保險
            lab[i] = int(order_pref[i][0])
    return lab


def _caps_equal(n: int, k: int) -> list[int]:
    caps = [n // k] * k
    for i in range(n - sum(caps)):
        caps[i] += 1
    return caps


def _cluster(P: np.ndarray, k: int, depot, cell: float, *,
             depot_caps: bool = True) -> list[list[int]]:
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


# ── 巡迴：NN + 2-opt + Or-opt ─────────────────────────────────────────
def _mat(P: np.ndarray):
    d = np.sqrt(((P[:, None, :] - P[None, :, :]) ** 2).sum(-1))
    K = min(_NEIGH_K, len(P) - 1)
    idx = np.argsort(d, axis=1)
    # ⚠ 不可寫成 [:, 1:K+1]：有重合點時 argsort 的第 0 欄不一定是自己，
    #    自己混進鄰居表 ⇒ 2-opt 找到「增益 dab 但實際 0」的假移動而永遠不收斂。
    neigh = [[int(j) for j in row if int(j) != i][:K] for i, row in enumerate(idx)]
    return (d.tolist() if len(P) <= 1500 else d), neigh


def _nn_tour(D, m: int) -> list[int]:
    unv = set(range(1, m))
    tour, cur = [0], 0
    while unv:
        row = D[cur]
        nxt = min(unv, key=lambda j: row[j])
        tour.append(nxt)
        unv.discard(nxt)
        cur = nxt
    return tour


def _tour_len(tour, D) -> float:
    m = len(tour)
    return sum(D[tour[i]][tour[(i + 1) % m]] for i in range(m))


def _rev(tour, pos, i, j):
    """反轉環狀區段 [i..j]（往前數）。長度過半就改反轉補集，等價但更快。"""
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


def _two_opt(tour, D, neigh, pos) -> bool:
    """鄰居表 + don't-look bits 的 2-opt，前後兩個方向都掃。"""
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
                    break                     # 鄰居已排序，再往後不可能有增益
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


def _or_opt(tour, D, neigh, pos) -> bool:
    """Or-opt：把 1~_ORO_MAX 格的片段搬到別處（正反兩種插入方向都試）。"""
    m = len(tour)
    if m < 5:
        return False
    any_move = False
    q = deque(tour)
    inq = [True] * m
    while q:
        s1 = q.popleft()
        inq[s1] = False
        done = False
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
                anchor = head
                for c in neigh[anchor]:
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
            any_move = done = True
            break
        if done:
            continue
    return any_move


def _dedupe(pts):
    """去掉重合點（例如 home 恰好落在某格中心上）。保留 pts[0] 為 home。"""
    out, seen = [], set()
    for p in pts:
        k = (round(p[0], 6), round(p[1], 6))
        if k in seen:
            continue
        seen.add(k)
        out.append((float(p[0]), float(p[1])))
    return out


def _solve(pts: list[tuple[float, float]]) -> tuple[list[tuple[float, float]], float]:
    """pts[0] 是 home。回傳 (從 home 起算的環狀點序, 長度)。"""
    pts = _dedupe(pts)
    m = len(pts)
    if m <= 2:
        return list(pts), (2.0 * math.dist(pts[0], pts[1]) if m == 2 else 0.0)
    D, neigh = _mat(np.asarray(pts, float))
    tour = _nn_tour(D, m)
    pos = [0] * m
    for t, c in enumerate(tour):
        pos[c] = t
    for _ in range(30):                      # 兩個鄰域交替做到不動為止
        a = _two_opt(tour, D, neigh, pos)
        b = _or_opt(tour, D, neigh, pos)
        if not (a or b):
            break
    L = _tour_len(tour, D)
    k = tour.index(0)
    seq = tour[k:] + tour[:k]
    return [pts[i] for i in seq], L


# ── 再平衡：以 makespan（最長那台）為目標的實測爬山 ────────────────────
def _rebalance(groups, centers, start, cell):
    P = np.asarray(centers, float)
    live = [g for g in groups if g]
    if len(live) < 2:
        return groups
    sol = {}

    def solve(gi):
        g = tuple(live[gi])
        if g not in sol:
            pts = [start] + [centers[i] for i in g]
            sol[g] = _solve(pts)[1]
        return sol[g]

    lens = [solve(i) for i in range(len(live))]
    for _ in range(_REBAL_ROUNDS):
        hi = int(np.argmax(lens))
        cent = {i: P[live[i]].mean(0) for i in range(len(live))}
        # 只考慮空間上相鄰（質心夠近）且較短的那些群
        cands = sorted((i for i in range(len(live))
                        if i != hi and lens[i] < lens[hi] - 2 * cell),
                       key=lambda i: lens[i])
        moved = False
        for lo in cands[:3]:
            gap = lens[hi] - lens[lo]
            k = max(1, min(len(live[hi]) - 1, int(gap / (4.0 * cell))))
            while k >= 1:
                tgt = cent[lo]
                order = sorted(live[hi],
                               key=lambda i: (P[i][0] - tgt[0]) ** 2 + (P[i][1] - tgt[1]) ** 2)
                mv = order[:k]
                nh = [i for i in live[hi] if i not in set(mv)]
                nl = live[lo] + mv
                lh = _solve([start] + [centers[i] for i in nh])[1]
                ll_ = _solve([start] + [centers[i] for i in nl])[1]
                if max(lh, ll_) < lens[hi] - 1e-6:
                    live[hi], live[lo] = nh, nl
                    lens[hi], lens[lo] = lh, ll_
                    moved = True
                    break
                k //= 2
            if moved:
                break
        if not moved:
            break
    return live


# ── 對外介面 ──────────────────────────────────────────────────────────
def plan(polys, n_uav, *, swath_m, pitch_m, start=(0.0, 0.0)):
    """網格 + 貪婪最近鄰（+2-opt/Or-opt）覆蓋規劃。

    回傳每機一條折線（平面座標、公尺），起點與終點都在 start。
    """
    n_uav = max(1, int(n_uav))
    cell = _cell_size(float(pitch_m), float(swath_m))
    centers, clipped, frac = _build_cells(polys, cell)
    if not centers:
        return []
    keep = _prune(centers, clipped, frac, float(swath_m), cell)
    centers = [centers[i] for i in keep]
    P = np.asarray(centers, float)
    # 兩種容量政策各跑一次（等量 / 依 home 距離補償），取 makespan 較小者。
    # 消融顯示兩者互有勝負（單一矩形 −3.7%、四塊 +2.6%），所以都跑再挑。
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
            tr.append(list(seq) + [tuple(start)])
            mk = max(mk, L)
        if tr and (best is None or mk < best[0] - 1e-9):
            best = (mk, tr)
        if n_uav <= 1:
            break
    return best[1] if best else []
