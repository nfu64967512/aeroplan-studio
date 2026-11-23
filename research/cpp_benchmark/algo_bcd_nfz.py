"""禁航區的胞分解覆蓋（Boustrophedon Cellular Decomposition + 貪婪排序）。

這就是「把 NFZ 當切割線，先掃乾淨的區、再掃貼著 NFZ 的區」那個想法。
它在文獻裡的名字是 **Choset 的 Boustrophedon Cellular Decomposition (BCD)**：
用障礙物的臨界點把自由空間切成胞，各胞內牛耕，胞之間排一條巡迴。

四個步驟
--------
1. **自由空間** = 作業區 − NFZ.buffer(inset)。
   ⚠⚠ `inset` 是關鍵，也是整個方法的取捨所在：
   航跡繞開 NFZ **不等於**相機沒拍到裡面。刈幅 w 的相機沿著 NFZ 邊界飛，
   會拍進去 w/2。要「一格都不拍到」就得 `inset = w/2`，代價是 NFZ 周圍
   多出一圈寬 w/2 的**掃不到的環**。本模組把 inset 開成參數，好把這條
   取捨曲線量出來（見 `sweep_inset()`）。
2. **切胞**：沿掃描方向的垂直軸，在每個障礙物的臨界點（x 的極值）切一刀。
   這是 BCD 的原始定義。切完用 shapely 取聯集差集，並**驗證面積守恆**
   （`hv_dccpp.geometry.convex_partition` 就是在這一步靜默丟掉 25.8% 面積的）。
3. **胞內牛耕**：每個胞用自己的最小寬度方向鋪條帶。
4. **排序與分配**：
   * 順序：`order='greedy'` 最近鄰 + 2-opt；`order='clean_first'` 先掃不與 NFZ
     相鄰的胞（使用者要的那個），`order='area'` 大胞優先。
   * 分配：LPT（longest-processing-time-first）貪婪，這是 makespan 的標準貪婪法，
     近似比 4/3 − 1/(3m)。不是隨便切。
"""
from __future__ import annotations

from pathlib import Path
import math
import sys

# ⚠ 原本硬編 session scratchpad 的路徑（會消失），改成 repo 相對。
BENCH = str(Path(__file__).resolve().parent)
if BENCH not in sys.path:
    sys.path.insert(0, BENCH)

from shapely.geometry import LineString, Polygon, box  # noqa: E402
from shapely.ops import unary_union  # noqa: E402

from bench import _strips, min_width_angle  # noqa: E402


# ── 1+2. 自由空間與切胞 ───────────────────────────────────────────────
def free_space(polys, nfz_polys, inset: float):
    area = unary_union(polys)
    if not nfz_polys:
        return area
    grow = unary_union([p.buffer(inset, join_style=2) for p in nfz_polys]) \
        if inset > 0 else unary_union(nfz_polys)
    return area.difference(grow)


def bcd_cells(free, nfz_polys, inset: float, theta: float = 0.0) -> list[Polygon]:
    """沿 theta 的垂直軸在障礙物臨界點切胞。回傳的胞聯集面積必須等於 free。"""
    c, s = math.cos(-theta), math.sin(-theta)
    rot = lambda x, y: (c * x - s * y, s * x + c * y)            # noqa: E731
    inv = lambda x, y: (c * x + s * y, -s * x + c * y)           # noqa: E731

    def rot_geom(g):
        return Polygon([rot(x, y) for x, y in g.exterior.coords[:-1]],
                       [[rot(x, y) for x, y in r.coords[:-1]] for r in g.interiors])

    parts = [rot_geom(g) for g in getattr(free, "geoms", [free]) if not g.is_empty]
    if not parts:
        return []
    rfree = unary_union(parts)
    x0, y0, x1, y1 = rfree.bounds
    cuts = set()
    for z in nfz_polys:
        zz = z.buffer(inset, join_style=2) if inset > 0 else z
        rz = Polygon([rot(x, y) for x, y in zz.exterior.coords[:-1]])
        cuts.add(rz.bounds[0])
        cuts.add(rz.bounds[2])
    xs = [x0 - 1.0] + sorted(x for x in cuts if x0 < x < x1) + [x1 + 1.0]
    cells = []
    for a, b in zip(xs, xs[1:]):
        sl = rfree.intersection(box(a, y0 - 1.0, b, y1 + 1.0))
        for g in getattr(sl, "geoms", [sl]):
            if g.geom_type == "Polygon" and g.area > 1.0:
                cells.append(Polygon([inv(x, y) for x, y in g.exterior.coords[:-1]],
                                     [[inv(x, y) for x, y in r.coords[:-1]]
                                      for r in g.interiors]))
    # ★ 面積守恆：切胞絕不可以靜默吃掉面積
    lost = free.area - sum(c_.area for c_ in cells)
    assert abs(lost) < max(1.0, free.area * 1e-6), f"切胞掉了 {lost:.1f} m²"
    return cells


# ── 3+4. 胞內牛耕、排序、分配 ────────────────────────────────────────
def _cell_path(cell: Polygon, pitch: float) -> list[tuple]:
    segs = _strips(cell, pitch, min_width_angle(cell))
    pts, flip = [], False
    for a, b in segs:
        pts += [b, a] if flip else [a, b]
        flip = not flip
    return pts


def _order(cells, paths, start, nfz_polys, inset, how: str):
    """胞的走訪順序。"""
    idx = list(range(len(cells)))
    if how == "area":
        return sorted(idx, key=lambda i: -cells[i].area)

    def greedy(sub):
        rest, cur, out = set(sub), start, []
        while rest:
            i = min(rest, key=lambda j: math.dist(cur, paths[j][0]))
            out.append(i)
            rest.discard(i)
            cur = paths[i][-1]
        for _ in range(60):                                  # 2-opt
            improved = False
            for a in range(len(out) - 1):
                for b in range(a + 2, len(out)):
                    cand = out[:a + 1] + out[a + 1:b + 1][::-1] + out[b + 1:]
                    if _tour_len(cand, paths, start) < _tour_len(out, paths, start) - 1e-6:
                        out, improved = cand, True
            if not improved:
                break
        return out

    if how == "clean_first":
        # 使用者要的：先掃不貼著 NFZ 的胞，最後才掃貼著的
        grow = (unary_union([p.buffer(inset + 1.0, join_style=2) for p in nfz_polys])
                if nfz_polys else None)
        dirty = {i for i in idx if grow is not None and cells[i].intersects(grow)}
        return greedy([i for i in idx if i not in dirty]) + greedy(sorted(dirty))
    return greedy(idx)


def _tour_len(order, paths, start):
    cur, d = start, 0.0
    for i in order:
        if not paths[i]:
            continue
        d += math.dist(cur, paths[i][0])
        d += sum(math.dist(paths[i][k], paths[i][k - 1]) for k in range(1, len(paths[i])))
        cur = paths[i][-1]
    return d + math.dist(cur, start)


def plan(polys, n_uav, *, swath_m, pitch_m, start=(0.0, 0.0), nfz_polys=(),
         inset=None, order="greedy", reroute=True):
    """BCD 胞分解 → 貪婪排胞 → **依條帶**平均切給 n 台 → 轉場繞開 NFZ。

    ⚠ 前一版有三個缺陷，都修了，記在這裡免得再犯：
      1. **只用了 4 台**：原本以「胞」為分配單位，胞只有 4 個就只餵得飽 4 台。
         改成把排好序的**條帶序列**依累積長度切成 n 段 —— 這樣胞數與機數解耦。
      2. `clean_first` 回空清單：原本那串巢狀 comprehension 寫錯了，改直白寫法。
      3. **胞間轉場還是穿過 NFZ**（實測 2.533 km 在裡面）：胞內條帶不進 NFZ，
         但兩個胞之間的直線會。最後對每條航跡套 `nfz_correct`（可視圖 + Dijkstra），
         它處理得了轉場 —— 這正是事後繞行唯一真正有效的場合。
    """
    nfz_polys = list(nfz_polys)
    if inset is None:
        inset = swath_m / 2.0
    free = free_space(polys, nfz_polys, inset)
    if free.is_empty:
        return []
    cells = bcd_cells(free, nfz_polys, inset)
    paths = [_cell_path(c, pitch_m) for c in cells]
    keep = [i for i, p in enumerate(paths) if len(p) >= 2]
    cells, paths = [cells[i] for i in keep], [paths[i] for i in keep]
    if not cells:
        return []
    seq = _order(cells, paths, start, nfz_polys, inset, order)

    # 排好序的條帶序列（一條 = 一對端點），再依累積長度平均切成 n 段。
    segs = []
    for i in seq:
        p = paths[i]
        segs += [(p[k - 1], p[k]) for k in range(1, len(p), 2)]
    lens = [math.dist(a, b) for a, b in segs]
    share = sum(lens) / max(n_uav, 1)
    tracks, cur, acc, k = [], [start], 0.0, 1
    for (a, b), L in zip(segs, lens):
        cur += [a, b]
        acc += L
        if acc >= share * k and len(tracks) < n_uav - 1:
            cur.append(start)
            tracks.append(cur)
            cur, k = [start], k + 1
    cur.append(start)
    tracks.append(cur)
    tracks = [t for t in tracks if len(t) >= 2]
    if reroute and nfz_polys:
        from bench import nfz_correct
        tracks, _added, _rr = nfz_correct(tracks, nfz_polys)
    return tracks


# ── 取捨曲線：inset 給多少 ───────────────────────────────────────────
def sweep_inset(polys, nfz_polys, n_uav, swath_m, pitch_m):
    from bench import nfz_metrics
    rows = []
    for f in (0.0, 0.25, 0.5, 0.75, 1.0):
        ins = swath_m * f
        tr = plan(polys, n_uav, swath_m=swath_m, pitch_m=pitch_m, nfz_polys=nfz_polys,
                  inset=ins)
        m = nfz_metrics(polys, tr, swath_m, nfz_polys)
        rows.append((f, ins, m))
    return rows


if __name__ == "__main__":
    from bench import boustrophedon, nfz_correct, nfz_metrics, rect_xy, run_planner_nfz
    polys = [rect_xy(0, 0, 2500, 2000)]
    nfz = [rect_xy(1200, 800, 700, 600)]
    alt = 140.0
    sw = 2 * alt * math.tan(math.radians(60.0) / 2)
    pitch = sw * 0.7
    print(f"作業區 20 km²、NFZ 1.68 km²、8 台、刈幅 {sw:.1f} m\n")
    hdr = (f"{'方法':34s}{'台':>3s}{'總km':>8s}{'覆蓋':>8s}{'開銷':>7s}"
           f"{'掃描率':>8s}{'NFZ內航跡km':>12s}{'NFZ內拍到km²':>13s}")
    print(hdr)

    def row(name, tr):
        m = nfz_metrics(polys, tr, sw, nfz)
        print(f"{name:34s}{m['n_uav']:3d}{m['total_km']:8.1f}{m['coverage'] * 100:7.1f}%"
              f"{(m['overhead'] or 0):7.3f}{m['scan_rate']:8.2f}"
              f"{m['nfz_track_km']:12.3f}{m['nfz_swept_km2']:13.4f}")

    b = boustrophedon(polys, 8, swath_m=sw, pitch_m=pitch)
    bc, _a, _r = nfz_correct(b, nfz)
    row("牛耕式＋事後繞行", bc)
    t, _e, _a, _r = run_planner_nfz(False, polys, 8, 0, alt=alt, pitch=pitch,
                                    overlap=0.0, nfz_polys=nfz)
    row("DCCPP＋GUI 事後繞行", t)
    t, _e, _a, _r = run_planner_nfz(True, polys, 8, 0, alt=alt, pitch=pitch,
                                    overlap=0.0, nfz_polys=nfz)
    row("HARVEST 約束內規劃", t)
    for o in ("greedy", "clean_first", "area"):
        row(f"BCD 胞分解（排序 {o}）",
            plan(polys, 8, swath_m=sw, pitch_m=pitch, nfz_polys=nfz, order=o))

    print(f"\ninset 取捨（inset = 刈幅 × f；f=0.5 才保證相機一格都不拍進 NFZ）")
    print(f"{'f':>5s}{'inset m':>9s}{'覆蓋':>8s}{'總km':>8s}{'NFZ內拍到km²':>13s}")
    for f, ins, m in sweep_inset(polys, nfz, 8, sw, pitch):
        print(f"{f:5.2f}{ins:9.1f}{m['coverage'] * 100:7.1f}%{m['total_km']:8.1f}"
              f"{m['nfz_swept_km2']:13.4f}")
