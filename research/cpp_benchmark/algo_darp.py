"""DARP 式覆蓋規劃（Divide Areas based on Robot initial Positions）。

論文核心是「把作業區依各機起點切成 n 塊**面積相等且連通**的子區」：在格點上算
每格到各機起點的評價值 E_i，乘上一組**修正係數 c_i**後取 argmin 指派，再依各機
分到的量迭代調整 c_i 直到均衡；最後對每塊子區各跑一次牛耕。

本檔在原演算法之上做的事（都寫在明處，也都附了實測數字）：

1. **起點退化的處理**。測試台所有機的起點都是同一個 home ⇒ E_i 全機相同、
   DARP 的指派完全退化。這裡給一組**虛擬起點**：在掃描方向的旋轉座標系裡排成
   (a 欄 × b 列) 的格狀種子，並且**把 b = 1…n 的每種拆法都跑一遍**，用實測航程
   挑贏家。這步是最大的單一槓桿：同一塊 5×4 km 區 n=8，切成 8 條全寬橫帶
   （題目提示的「沿最小寬度方向等面積切條」）量到最長 33.41 km / 總 243.9 km；
   換成 2 欄 × 4 列是 26.02 / 202.4。差別純粹來自種子佈局。
2. **均衡的量是實測航程，不是面積**。c_i 的迭代拿「該機這一輪真正飛出來的折線
   長度」去調（含來回 home 的空程）。面積相等 ≠ 工作量相等：離 home 遠的子區
   空程多，就該少分幾條條帶。
3. **格子的列 = 條帶**。格點列高取 pitch 且與條帶對齊，子區邊界在垂直方向落在
   條帶之間、水平方向落在格子邊界 ⇒ 不會把條帶切碎成一堆要額外來回的片段。
   條帶相位**置中**（margin = (H−(K−1)p)/2），上下邊緣不留縫。
4. **航跡順序不只蛇形**。蛇形收尾一定停在離 home 最遠那一側，回程空程可到
   2.5 km；加入「梳形」（偶數條上去、奇數條下來）後由實測長度自己挑。
5. **邊界局部搜尋**。整列/整行在相鄰子區之間搬移，字典序壓 (最長, 總長)。
   它抓得到公式看不出來的東西——例如條帶數奇偶決定出口在哪一側——實測把
   n=8 的最長航程從 27.15 壓到 26.02 km。
6. **凹形補洞**。內凹邊旁邊會留下一條掃不到的細長條（L 形基準線 99.5% 就是
   這個）。把已掃區減掉，殘餘超過 0.3% 面積就補一條**只跨殘餘 u 範圍**的條帶。
7. **跨區連續切割**當第二條路徑：機比區少、或機數不是區數倍數時，整塊分配會
   爆掉（4 塊 3 台最佳整塊切法 2/1/1 ⇒ 98.5 km；允許拆塊 ⇒ 70.4 km）。

介面：``plan(polys, n_uav, *, swath_m, pitch_m, start) -> list[list[(x, y)]]``
只回傳折線，長度與覆蓋率一律由 bench.metrics 算。
"""
from __future__ import annotations

import math

import numpy as np
from shapely.geometry import LineString, MultiPolygon, Point, Polygon
from shapely.ops import unary_union

__all__ = ["plan"]

_ITERS = 26          # DARP 修正係數迴路的迭代數
_GAIN = 0.35         # c_i 的調整增益
_C_LO, _C_HI = 0.25, 4.0


# ── 幾何小工具 ────────────────────────────────────────────────────────
def _min_width_angle(poly: Polygon) -> float:
    """最小寬度方向（rad）：條帶平行於它 ⇒ 條帶數最少、迴轉最少。

    旋轉卡尺定理：最小寬度必定「垂直於凸包的某條邊」，所以只要掃過凸包的邊。
    """
    hull = poly.convex_hull
    if hull.geom_type != "Polygon":
        return 0.0
    cs = list(hull.exterior.coords)[:-1]
    if len(cs) < 2:
        return 0.0
    best_th, best_w = 0.0, float("inf")
    for i in range(len(cs)):
        ax, ay = cs[i]
        bx, by = cs[(i + 1) % len(cs)]
        if math.hypot(bx - ax, by - ay) < 1e-9:
            continue
        th = math.atan2(by - ay, bx - ax)
        s, c = math.sin(th), math.cos(th)
        vs = [(-s * x + c * y) for x, y in cs]      # 垂直於 th 的投影
        w = max(vs) - min(vs)
        if w < best_w - 1e-9:
            best_w, best_th = w, th
    return best_th


def _merge(ivs: list[tuple[float, float]], tol: float = 1e-6):
    if not ivs:
        return []
    ivs = sorted(ivs)
    out = [list(ivs[0])]
    for lo, hi in ivs[1:]:
        if lo <= out[-1][1] + tol:
            out[-1][1] = max(out[-1][1], hi)
        else:
            out.append([lo, hi])
    return [(a, b) for a, b in out]


def _overlap(ivs, lo, hi):
    out = []
    for a, b in ivs:
        x, y = max(a, lo), min(b, hi)
        if y - x > 1e-7:
            out.append((x, y))
    return out


def _plen(pts) -> float:
    return sum(math.dist(pts[i], pts[i - 1]) for i in range(1, len(pts)))


# ── 條帶場：一個多邊形 + 它的掃描方向 + 對齊條帶的格點 ────────────────
class _Cell:
    __slots__ = ("k", "c", "ivs", "w", "cu", "cv")

    def __init__(self, k, c, ivs, w, cu, cv):
        self.k, self.c, self.ivs, self.w, self.cu, self.cv = k, c, ivs, w, cu, cv


class _Field:
    """單一多邊形上的條帶格點。列 (k) = 掃描條帶，行 (c) = 沿條帶方向切格。"""

    def __init__(self, poly: Polygon, swath: float, pitch: float, cw: float = 0.0):
        self.poly = poly
        self.pitch = pitch
        self.th = _min_width_angle(poly)
        self.ct, self.st = math.cos(self.th), math.sin(self.th)
        rot = self._rot_poly(poly)
        umin, vmin, umax, vmax = rot.bounds
        H = vmax - vmin
        # 條帶列數：確保 (K-1)*pitch + swath >= H ⇒ 上下邊緣不留縫；相位**置中**
        # （常見寫法是從 vmin+p/2 起跳，那會在遠端留下最多半個 pitch 的縫，
        #   也就是基準線那 99.6% 的來源；置中是零成本的修正。）
        K = 1 if H <= swath else int(math.ceil((H - swath) / pitch)) + 1
        margin = (H - (K - 1) * pitch) / 2.0
        rows = [(vmin + margin + k * pitch, umin - 1.0, umax + 1.0) for k in range(K)]
        rows = self._patch(rot, rows, swath, pitch, umin, umax)
        rows.sort(key=lambda r: r[0])
        self.v = [r[0] for r in rows]
        self.K = len(rows)
        self.umin, self.umax = umin, umax
        w = cw if cw > 0 else pitch
        self.cw = w
        self.C = max(1, int(math.ceil((umax - umin) / w)))

        self.cells: list[_Cell] = []
        self.at: dict[tuple[int, int], int] = {}
        for k, (v, ulo, uhi) in enumerate(rows):
            row_ivs = _overlap(self._cut(rot, v, umin, umax), ulo, uhi)
            if not row_ivs:
                continue
            for c in range(self.C):
                lo, hi = umin + c * w, umin + (c + 1) * w
                ivs = _overlap(row_ivs, lo, hi)
                if not ivs:
                    continue
                wt = sum(b - a for a, b in ivs)
                cu = 0.5 * (ivs[0][0] + ivs[-1][1])
                self.at[(k, c)] = len(self.cells)
                self.cells.append(_Cell(k, c, ivs, wt, cu, v))

        self.n = len(self.cells)
        self.wt = np.array([cl.w for cl in self.cells], dtype=float)
        self.xy = np.array([self.to_xy(cl.cu, cl.cv) for cl in self.cells],
                           dtype=float).reshape(-1, 2)
        self.adj = [[] for _ in range(self.n)]
        for (k, c), i in self.at.items():
            for dk, dc in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                j = self.at.get((k + dk, c + dc))
                if j is not None:
                    self.adj[i].append(j)
        self.work = float(self.wt.sum())          # 純條帶長度
        self.turn = max(0, self.K - 1) * pitch    # 單機掃完全區的迴轉估計

    # 旋轉座標：u 沿掃描方向、v 垂直於它
    def to_uv(self, x, y):
        return (self.ct * x + self.st * y, -self.st * x + self.ct * y)

    def to_xy(self, u, v):
        return (self.ct * u - self.st * v, self.st * u + self.ct * v)

    def _rot_poly(self, poly: Polygon) -> Polygon:
        shell = [self.to_uv(*p) for p in poly.exterior.coords]
        holes = [[self.to_uv(*p) for p in r.coords] for r in poly.interiors]
        return Polygon(shell, holes)

    @staticmethod
    def _cut(rot: Polygon, v: float, umin: float, umax: float):
        seg = rot.intersection(LineString([(umin - 1.0, v), (umax + 1.0, v)]))
        out = []
        for g in getattr(seg, "geoms", [seg]):
            if g.geom_type != "LineString" or g.length <= 1e-6:
                continue
            us = [p[0] for p in g.coords]
            out.append((min(us), max(us)))
        return _merge(out)

    def _patch(self, rot, rows, swath, pitch, umin, umax):
        """凹形補洞：均勻格線在**內凹邊**旁邊會留下一條掃不到的細長條。

        L 形就是典型：短臂的最後一條條帶落在離內凹邊 pitch 處，中間 30 幾公尺
        永遠沒人飛過（基準線的 99.5% 就是這個）。做法是把已掃區域減掉，殘餘塊
        超過門檻就補一條條帶，而且**只補在殘餘塊的 u 範圍內**（L 形只要 2 km，
        不是整條 4 km）。門檻設在 0.3% 面積：再小就不值得為它多飛一趟。
        """
        tol = max(0.003 * rot.area, swath * pitch)
        cap = len(rows) // 4 + 2
        added = 0
        for _ in range(3):
            bufs = []
            for v, ulo, uhi in rows:
                for a, b in _overlap(self._cut(rot, v, umin, umax), ulo, uhi):
                    bufs.append(LineString([(a, v), (b, v)]).buffer(swath / 2.0,
                                                                    quad_segs=8))
            if not bufs:
                break
            resid = rot.difference(unary_union(bufs))
            parts = [g for g in getattr(resid, "geoms", [resid])
                     if g.geom_type == "Polygon" and g.area > tol]
            if not parts or added >= cap:
                break
            grew = False
            for g in parts:
                gu0, gv0, gu1, gv1 = g.bounds
                for t in range(max(1, int(math.ceil((gv1 - gv0) / swath)))):
                    if added >= cap:
                        break
                    vv = gv0 + (gv1 - gv0) * (t + 0.5) / max(
                        1, int(math.ceil((gv1 - gv0) / swath)))
                    rows.append((vv, gu0 - 0.25 * swath, gu1 + 0.25 * swath))
                    added += 1
                    grew = True
            if not grew:
                break
        return rows

    # ── 由一組格子做出牛耕航跡 ──────────────────────────────────────
    @staticmethod
    def _orders(ks: list[int]) -> list[list[int]]:
        """條帶的走訪順序候選。

        純蛇形（0,1,2,…）掃完會停在**最遠**那條，回 home 的空程往往比整段迴轉
        還貴。所以加上「梳形」：先走偶數條上去、再走奇數條下來，出口就回到入口
        附近。代價是每次迴轉變成 2·pitch（多約 (R-1)·pitch），但省下的空程通常
        是它的兩三倍。哪個划算不用猜，四個順序全部量長度取最短。
        """
        out = []
        for base in (ks, ks[::-1]):
            out.append(base)
            if len(base) > 2:
                out.append(base[0::2] + base[1::2][::-1])
        return out

    def route(self, idxs, start, entry=None, exit_to=None) -> list[tuple[float, float]]:
        """回傳 start → 覆蓋 → start 的折線（順序 × 起始方向全試，取最短）。

        entry / exit_to 給「一台機連掃好幾個小區」用：進場點與離場點都不是 home。
        """
        if len(idxs) == 0:
            return []
        rows: dict[int, list] = {}
        for i in idxs:
            cl = self.cells[i]
            rows.setdefault(cl.k, []).extend(cl.ivs)
        ks = sorted(rows)
        for k in ks:
            rows[k] = _merge(rows[k])
        home = start if entry is None else entry
        tail = start if exit_to is None else exit_to
        best, best_L = None, float("inf")
        for order in self._orders(ks):
            for d0 in (1, -1):
                pts, d = [], d0
                for k in order:
                    ivs = rows[k]
                    v = self.v[k]
                    seq = ivs if d > 0 else [(b, a) for a, b in reversed(ivs)]
                    for a, b in seq:
                        pts.append(self.to_xy(a, v))
                        pts.append(self.to_xy(b, v))
                    d = -d
                trk = [home] + pts + [tail]
                L = _plen(trk)
                if L < best_L:
                    best_L, best = L, trk
        return best


# ── DARP：格點指派 + 連通性修復 + 修正係數迴路 ────────────────────────
def _components(field: _Field, members: list[int]) -> list[list[int]]:
    inside = set(members)
    seen, comps = set(), []
    for s in members:
        if s in seen:
            continue
        stack, comp = [s], []
        seen.add(s)
        while stack:
            i = stack.pop()
            comp.append(i)
            for j in field.adj[i]:
                if j in inside and j not in seen:
                    seen.add(j)
                    stack.append(j)
        comps.append(comp)
    return comps


def _repair(field: _Field, lab: np.ndarray, m: int) -> np.ndarray:
    """每區只留最大連通塊，其餘零碎塊併給**目前最輕**的相鄰區（DARP 的連通性步）。"""
    lab = lab.copy()
    for _ in range(2):
        load = np.zeros(m)
        np.add.at(load, lab, field.wt)
        changed = False
        for i in range(m):
            members = np.flatnonzero(lab == i).tolist()
            if len(members) < 2:
                continue
            comps = _components(field, members)
            if len(comps) <= 1:
                continue
            comps.sort(key=lambda cp: sum(field.wt[j] for j in cp), reverse=True)
            for comp in comps[1:]:
                nb = set()
                for j in comp:
                    for t in field.adj[j]:
                        if lab[t] != i:
                            nb.add(int(lab[t]))
                if nb:
                    tgt = min(nb, key=lambda L: load[L])
                else:                                   # 孤島：給質心最近的區
                    cen = field.xy[comp].mean(axis=0)
                    d = [np.hypot(*(field.xy[np.flatnonzero(lab == q)].mean(axis=0) - cen))
                         if np.any(lab == q) else 1e18 for q in range(m)]
                    d[i] = 1e18
                    tgt = int(np.argmin(d))
                wsum = sum(field.wt[j] for j in comp)
                for j in comp:
                    lab[j] = tgt
                load[i] -= wsum
                load[tgt] += wsum
                changed = True
        if not changed:
            break
    return lab


def _routes_of(field: _Field, lab: np.ndarray, m: int, start):
    trks, lens = [], []
    for i in range(m):
        idxs = np.flatnonzero(lab == i).tolist()
        t = field.route(idxs, start)
        trks.append(t)
        lens.append(_plen(t) if len(t) >= 2 else 0.0)
    return trks, np.array(lens, dtype=float)


def _score(trks, lens) -> float:
    if any(len(t) < 2 for t in trks):
        return float("inf")
    return float(lens.max())


def _darp(field: _Field, m: int, seeds: np.ndarray, start):
    """DARP 主迴路：argmin_i c_i·‖cell − seed_i‖，用實測航程調 c_i。"""
    d = np.linalg.norm(field.xy[:, None, :] - seeds[None, :, :], axis=2)
    d = np.maximum(d, 1.0)
    c = np.ones(m)
    best, best_s = None, float("inf")
    for _ in range(_ITERS):
        lab = np.argmin(d * c[None, :], axis=1)
        lab = _repair(field, lab, m)
        trks, lens = _routes_of(field, lab, m, start)
        s = _score(trks, lens)
        if s < best_s:
            best_s, best = s, lab
        ref = lens[lens > 0].mean() if np.any(lens > 0) else 1.0
        c = c * (1.0 + _GAIN * (lens - ref) / ref)
        c = np.clip(c, _C_LO, _C_HI)
        c = c / c.mean()
    return best, best_s


# ── 邊界局部搜尋：拿實測航程直接壓 max ────────────────────────────────
def _connected(field: _Field, lab: np.ndarray, i: int) -> bool:
    members = np.flatnonzero(lab == i).tolist()
    return len(members) > 0 and len(_components(field, members)) == 1


def _donations(field: _Field, lab: np.ndarray, i: int):
    """區 i 可以捐出去的單位：它最外面的那一列 / 那一行（連同收受的鄰居）。

    只動最外層可以讓「捐完還連通」幾乎自動成立，也把候選數壓到個位數。
    """
    idxs = np.flatnonzero(lab == i)
    if idxs.size == 0:
        return []
    rows: dict[int, list] = {}
    cols: dict[int, list] = {}
    for t in idxs:
        cl = field.cells[t]
        rows.setdefault(cl.k, []).append(int(t))
        cols.setdefault(cl.c, []).append(int(t))
    out = []
    for grp in (rows, cols):
        keys = sorted(grp)
        for key in {keys[0], keys[-1]}:
            group = grp[key]
            if len(group) >= idxs.size:
                continue
            nb = set()
            for t in group:
                for u in field.adj[t]:
                    if lab[u] != i:
                        nb.add(int(lab[u]))
            for j in nb:
                out.append((group, j))
    return out


def _refine(field: _Field, lab: np.ndarray, m: int, start, budget: int = 60):
    """把整列/整行在相鄰區之間搬移，字典序壓 (最長航程, 總航程)。

    這一步不是裝飾：牛耕的**出口在哪一側由條帶數的奇偶決定**。條帶數是奇數時
    收尾一定停在離 home 最遠的那一側，回程空程可以到 2.5 km；搬一列過去就翻轉
    奇偶，同時砍掉工作量又砍掉空程。這種增益沒辦法用面積或成本公式看出來，
    只能真的把折線量出來，所以這裡直接用實測長度當目標函數。
    """
    lab = lab.copy()
    trks, lens = _routes_of(field, lab, m, start)
    if any(len(t) < 2 for t in trks):
        return lab, trks, lens
    for _ in range(budget):
        cur = (round(float(lens.max()), 3), round(float(lens.sum()), 3))
        best = None
        for i in np.argsort(-lens)[:4]:
            i = int(i)
            for group, j in _donations(field, lab, i):
                lab2 = lab.copy()
                lab2[group] = j
                if not (_connected(field, lab2, i) and _connected(field, lab2, j)):
                    continue
                ti = field.route(np.flatnonzero(lab2 == i).tolist(), start)
                tj = field.route(np.flatnonzero(lab2 == j).tolist(), start)
                if len(ti) < 2 or len(tj) < 2:
                    continue
                l2 = lens.copy()
                l2[i], l2[j] = _plen(ti), _plen(tj)
                key = (round(float(l2.max()), 3), round(float(l2.sum()), 3))
                if key < cur and (best is None or key < best[0]):
                    best = (key, lab2, l2, i, j, ti, tj)
        if best is None:
            break
        _, lab, lens, i, j, ti, tj = best
        trks = list(trks)
        trks[i], trks[j] = ti, tj
    return lab, trks, lens


# ── 結構化種子 / 初始分割（起點退化時的「幾何上合理的切法」）──────────
def _split_by_weight(keys: list, w: dict, parts: int) -> list[list]:
    """把有序的 keys 依權重切成 parts 段連續區塊（每段至少一個）。"""
    parts = max(1, min(parts, len(keys)))
    tot = sum(w[k] for k in keys)
    if tot <= 0:
        parts = min(parts, len(keys))
        step = len(keys) / parts
        return [keys[int(i * step):int((i + 1) * step)] or [keys[min(i, len(keys) - 1)]]
                for i in range(parts)]
    out, acc, cur, done = [], 0.0, [], 0
    share = tot / parts
    for idx, k in enumerate(keys):
        cur.append(k)
        acc += w[k]
        left = len(keys) - idx - 1
        if done < parts - 1 and acc >= share * (done + 1) - 1e-9 and left >= parts - done - 1:
            out.append(cur)
            cur, done = [], done + 1
    if cur:
        out.append(cur)
    while len(out) < parts:                      # 保底：不讓任何一區空掉
        big = max(range(len(out)), key=lambda i: len(out[i]))
        if len(out[big]) < 2:
            break
        h = len(out[big]) // 2
        out.insert(big + 1, out[big][h:])
        out[big] = out[big][:h]
    return out


def _blocks(field: _Field, m: int, b: int) -> np.ndarray | None:
    """(a 欄 × b 列) 結構化切割：先沿 v 切 b 條帶群，每群再沿 u 切 a_j 塊。"""
    if b > field.K:
        return None
    per = [m // b + (1 if j < m % b else 0) for j in range(b)]
    rw = {k: 0.0 for k in range(field.K)}
    for cl in field.cells:
        rw[cl.k] += cl.w
    rows = [k for k in range(field.K) if rw[k] > 0]
    if len(rows) < b:
        return None
    bands = _split_by_weight(rows, rw, b)
    if len(bands) != b:
        return None
    lab = np.full(field.n, -1, dtype=int)
    nxt = 0
    for band, a in zip(bands, per):
        rs = set(band)
        idxs = [i for i, cl in enumerate(field.cells) if cl.k in rs]
        cols = sorted({field.cells[i].c for i in idxs})
        cwt = {c: 0.0 for c in cols}
        for i in idxs:
            cwt[field.cells[i].c] += field.cells[i].w
        groups = _split_by_weight(cols, cwt, a)
        if len(groups) < a:
            return None
        gof = {}
        for gi, g in enumerate(groups):
            for c in g:
                gof[c] = gi
        for i in idxs:
            lab[i] = nxt + gof[field.cells[i].c]
        nxt += a
    if nxt != m or np.any(lab < 0):
        return None
    return lab


def _plan_field(field: _Field, m: int, start) -> list[list[tuple[float, float]]]:
    if field.n == 0 or m <= 0:
        return []
    if m == 1:
        t = field.route(list(range(field.n)), start)
        return [t] if len(t) >= 2 else []
    m = min(m, field.n)
    cands: list[tuple[float, np.ndarray]] = []
    for b in range(1, m + 1):
        lab0 = _blocks(field, m, b)
        if lab0 is None:
            continue
        lab0 = _repair(field, lab0, m)
        trks, lens = _routes_of(field, lab0, m, start)
        pick = [(_score(trks, lens), lab0)]                   # A：結構化等量切割
        lab1, s1 = _darp(field, m, np.array(
            [field.xy[lab0 == i].mean(axis=0) if np.any(lab0 == i)
             else field.xy.mean(axis=0) for i in range(m)]), start)
        if lab1 is not None:
            pick.append((s1, lab1))                           # B：DARP 修正係數
        pick = [p for p in pick if math.isfinite(p[0])]
        if pick:
            cands.append(min(pick, key=lambda p: p[0]))       # 每種佈局留一個
    if not cands:
        t = field.route(list(range(field.n)), start)
        return [t] if len(t) >= 2 else []
    # 每種佈局都要精修過才比。局部搜尋常常翻轉排名（實測：某佈局精修前 33.6
    # 排最後、精修後 31.0 反而最好），只精修「精修前領先」的候選會直接漏掉它。
    best, best_s = None, float("inf")
    for _s, lab in cands:                                     # C：邊界局部搜尋
        _lab, trks, lens = _refine(field, lab, m, start)
        s = _score(trks, lens)
        if s < best_s:
            best_s, best = s, trks
    return [t for t in best if len(t) >= 2]


# ── 多個多邊形：先把機隊分給各區，再各自 DARP ─────────────────────────
def _field_cost(f: _Field, m: int, start) -> float:
    trans = Point(start).distance(f.poly)
    return (f.work + f.turn) / max(m, 1) + 2.0 * trans


def _allocate(fields: list[_Field], n: int, start) -> list[int]:
    m = [1] * len(fields)
    for _ in range(n - len(fields)):
        j = max(range(len(fields)), key=lambda i: _field_cost(fields[i], m[i], start))
        m[j] += 1
    return m


def _cell_order(field: _Field) -> list[int]:
    """單一 field 內的牛耕走訪序（列升冪、方向交替）。連續的一段必定連通。"""
    rows: dict[int, list[int]] = {}
    for i, cl in enumerate(field.cells):
        rows.setdefault(cl.k, []).append(i)
    out, d = [], 1
    for k in sorted(rows):
        ids = sorted(rows[k], key=lambda i: field.cells[i].c)
        out += ids if d > 0 else ids[::-1]
        d = -d
    return out


def _chain(fields: list[_Field], order: list[int], by: dict[int, list[int]], start):
    """一台機把好幾個（部分）小區串起來飛：離場點瞄下一個區，最後才回 home。"""
    pts, cur = [start], start
    keys = [fi for fi in order if by.get(fi)]
    for t, fi in enumerate(keys):
        nxt = keys[t + 1] if t + 1 < len(keys) else None
        aim = (fields[nxt].poly.centroid.x, fields[nxt].poly.centroid.y) \
            if nxt is not None else start
        seg = fields[fi].route(by[fi], start, entry=cur, exit_to=aim)
        body = seg[1:-1] if len(seg) >= 3 else []
        pts += body
        if body:
            cur = body[-1]
    pts.append(start)
    return pts


def _geo_order(fields: list[_Field]) -> list[int]:
    cen = np.array([[f.poly.centroid.x, f.poly.centroid.y] for f in fields])
    if len(fields) < 2:
        return [0]
    d = cen - cen.mean(axis=0)
    axis = np.linalg.svd(d, full_matrices=False)[2][0]
    return [int(i) for i in np.argsort(d @ axis)]


def _plan_chunks(fields: list[_Field], n: int, start):
    """跨區的連續等量切割：把所有格子照地理序 + 牛耕序串成一條，切成 n 段。

    這條路徑存在的理由是「整塊分配」在 n 不是區數的倍數時會爆掉：4 塊 3 台的
    最佳整塊切法是 2/1/1，最長那台要飛兩整塊（實測 98.5 km），允許把一塊拆給
    兩台就能壓到 ~70 km。切點落在牛耕序上 ⇒ 每一段自動連通。
    """
    order = _geo_order(fields)
    seq = [(fi, ci, fields[fi].cells[ci].w) for fi in order
           for ci in _cell_order(fields[fi])]
    if not seq:
        return []
    tot = sum(s[2] for s in seq)
    share = tot / max(n, 1)
    chunks, cur, acc = [], [], 0.0
    for s in seq:
        cur.append(s)
        acc += s[2]
        if len(chunks) < n - 1 and acc >= share:
            chunks.append(cur)
            cur, acc = [], 0.0
    chunks.append(cur)
    out = []
    for ch in chunks:
        by: dict[int, list[int]] = {}
        for fi, ci, _w in ch:
            by.setdefault(fi, []).append(ci)
        t = _chain(fields, order, by, start)
        if len(t) >= 2:
            out.append(t)
    return out


def _multi_route(fields: list[_Field], start) -> list[tuple[float, float]]:
    """一台機掃好幾個小區：最近鄰序，逐區牛耕，離場點瞄準**下一個區**（不是 home）。"""
    left, cur, pts = list(fields), start, [start]
    while left:
        f = min(left, key=lambda g: Point(cur).distance(g.poly))
        left.remove(f)
        nxt = min(left, key=lambda g: Point(cur).distance(g.poly)) if left else None
        aim = (nxt.poly.centroid.x, nxt.poly.centroid.y) if nxt is not None else start
        seg = f.route(list(range(f.n)), start, entry=cur, exit_to=aim)
        body = seg[1:-1] if len(seg) >= 3 else []
        pts += body
        cur = body[-1] if body else cur
    pts.append(start)
    return pts


def plan(polys, n_uav, *, swath_m, pitch_m, start=(0.0, 0.0)):
    """DARP 式覆蓋規劃。回傳每機一條折線（平面公尺座標），頭尾都在 start。"""
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
        return _plan_chunks(fields, n_uav, start)

    alloc = _allocate(fields, n_uav, start)
    for i, (f, m) in enumerate(zip(fields, alloc)):
        # 格子比飛機還少 ⇒ 有人分不到工作。把行寬對切到夠分為止（例如作業區
        # 本身比刈幅還小的退化情形）。
        cw = f.cw
        while fields[i].n < m and cw > 2.0:
            cw /= 2.0
            fields[i] = _Field(f.poly, swath_m, pitch_m, cw=cw)
    tracks = []
    for f, m in zip(fields, alloc):
        tracks += _plan_field(f, m, start)

    if len(fields) > 1:                    # 多區時把「跨區連續切割」也拿來比一次
        alt = _plan_chunks(fields, n_uav, start)
        if alt and len(alt) >= len(tracks):
            mx = lambda ts: max(_plen(t) for t in ts) if ts else float("inf")  # noqa: E731
            if mx(alt) < mx(tracks):
                return alt
    return tracks
