"""乾淨條帶優先（clean-strip-first）：完整區先一次掃完，NFZ 旁的殘段最後補。

設計原則：**越少重複路徑越好**。所謂重複路徑就是「不在測繪條帶上的那些公尺」
—— 轉場、掉頭、往返。所以本模組多量一個指標：

    transit_ratio = (總航程 − 條帶長度) / 總航程

它直接就是「白飛的比例」。條帶長度是覆蓋所必需、無法再省的下界。

與 BCD 的差別（這是本方法存在的理由）
------------------------------------
BCD 沿障礙物臨界點把**整個作業區**切成胞，所以連離 NFZ 幾公里、根本不受影響
的左半邊也被切成獨立的胞，胞之間要飛轉場。實測那一刀讓左邊 11.68 km² 的完整
區塊被迫與右邊分家。

本方法改成**按條帶分類**，不切區：
  * **乾淨條帶**：被自由空間裁完仍是「一整條」的 → 完全不受 NFZ 影響。
    它們在掃描軸上是連續的兩段（NFZ 上方、NFZ 下方），各自一次牛耕掃完，
    轉場只有掉頭，沒有任何額外飛行。
  * **髒條帶**：被 NFZ 切成兩段以上的。最後才處理，而且**同一側的殘段收在一起**
    （左半邊全部掃完再掃右半邊），避免每一條都左右來回穿越 NFZ。

掃描方向
--------
`direction='min_width'` 用作業區最小寬度（教科書預設，條帶數最少）。
`direction='fewest_dirty'` 改成**讓髒條帶最少**的方向 —— 沿 NFZ 長軸掃，
被切斷的條帶就少。兩者哪個好由幾何決定，所以兩個都跑。
"""
from __future__ import annotations

from pathlib import Path
import math
import sys

# ⚠ 原本硬編 session scratchpad 的路徑（會消失），改成 repo 相對。
BENCH = str(Path(__file__).resolve().parent)
if BENCH not in sys.path:
    sys.path.insert(0, BENCH)

from shapely.geometry import LineString, Polygon  # noqa: E402
from shapely.ops import unary_union  # noqa: E402

from bench import min_width_angle  # noqa: E402


def _rot(p, th):
    c, s = math.cos(th), math.sin(th)
    return (c * p[0] - s * p[1], s * p[0] + c * p[1])


def _clip_strips(free, area, pitch: float, theta: float):
    """回傳 [(y, [段, ...], 原始全長)]，段已轉回原座標，依 y 由小到大。"""
    rot_area = Polygon([_rot(p, -theta) for p in area.exterior.coords[:-1]])
    x0, y0, x1, y1 = rot_area.bounds
    rfree = unary_union([Polygon([_rot(p, -theta) for p in g.exterior.coords[:-1]],
                                 [[_rot(p, -theta) for p in r.coords[:-1]]
                                  for r in g.interiors])
                         for g in getattr(free, "geoms", [free]) if not g.is_empty])
    out, y = [], y0 + pitch / 2.0
    while y < y1:
        line = LineString([(x0 - 10, y), (x1 + 10, y)])
        full = rot_area.intersection(line)
        cut = rfree.intersection(line)
        segs = [g for g in getattr(cut, "geoms", [cut])
                if g.geom_type == "LineString" and g.length > 1.0]
        if segs:
            out.append((y, [[_rot(c, theta) for c in g.coords] for g in segs],
                        full.length if hasattr(full, "length") else 0.0))
        y += pitch
    return out


def _dirty_count(free, area, pitch, theta) -> int:
    return sum(1 for _y, segs, _L in _clip_strips(free, area, pitch, theta) if len(segs) > 1)


def _chain(segs_in_order, start):
    """把有序的段串成一條折線，交替往返（不製造多餘回頭）。"""
    pts, flip = [start], False
    for seg in segs_in_order:
        a, b = seg[0], seg[-1]
        if flip:
            a, b = b, a
        pts += [a, b]
        flip = not flip
    pts.append(start)
    return pts


def plan(polys, n_uav, *, swath_m, pitch_m, start=(0.0, 0.0), nfz_polys=(),
         inset=None, direction="min_width", reroute=True):
    nfz_polys = list(nfz_polys)
    if inset is None:
        inset = swath_m / 2.0
    area = unary_union(polys)
    free = area
    if nfz_polys:
        grow = unary_union([p.buffer(inset, join_style=2) for p in nfz_polys]) \
            if inset > 0 else unary_union(nfz_polys)
        free = area.difference(grow)
    if free.is_empty:
        return []

    th = min_width_angle(area if area.geom_type == "Polygon" else area.convex_hull)
    if direction == "fewest_dirty" and nfz_polys:
        th = min((th + k * math.pi / 24.0 for k in range(24)),
                 key=lambda a: (_dirty_count(free, area, pitch_m, a),
                                len(_clip_strips(free, area, pitch_m, a))))

    strips = _clip_strips(free, area, pitch_m, th)
    if not strips:
        return []

    # 乾淨 = 只有一段（NFZ 沒把它切開）
    clean = [(y, segs[0]) for y, segs, _L in strips if len(segs) == 1]
    dirty = [(y, segs) for y, segs, _L in strips if len(segs) > 1]

    ordered = [s for _y, s in clean]                       # 完整區先一次掃完
    if dirty:
        # 髒條帶：同一側的殘段收在一起，掃完一側再換另一側
        sides = {}
        for y, segs in dirty:
            for s in segs:
                key = round(sum(_rot(c, -th)[0] for c in s) / len(s) / 1000.0)
                sides.setdefault(key, []).append((y, s))
        for key in sorted(sides):
            ordered += [s for _y, s in sorted(sides[key], key=lambda t: t[0])]

    lens = [LineString(s).length for s in ordered]
    share = sum(lens) / max(n_uav, 1)
    tracks, cur, acc, k = [], [], 0.0, 1
    for s, L in zip(ordered, lens):
        cur.append(s)
        acc += L
        if acc >= share * k and len(tracks) < n_uav - 1:
            tracks.append(_chain(cur, start))
            cur, k = [], k + 1
    if cur:
        tracks.append(_chain(cur, start))
    tracks = [t for t in tracks if len(t) >= 2]
    if reroute and nfz_polys:
        from bench import nfz_correct
        tracks, _a, _r = nfz_correct(tracks, nfz_polys)
    return tracks


def transit_ratio(polys, tracks, nfz_polys, swath_m, pitch_m, inset=None):
    """白飛比例 = (總航程 − 條帶長度) / 總航程。條帶長度是覆蓋的下界。"""
    if inset is None:
        inset = swath_m / 2.0
    area = unary_union(polys)
    free = area
    if nfz_polys:
        free = area.difference(unary_union([p.buffer(inset, join_style=2)
                                            for p in nfz_polys]))
    th = min_width_angle(area if area.geom_type == "Polygon" else area.convex_hull)
    strip_len = sum(LineString(s).length
                    for _y, segs, _L in _clip_strips(free, area, pitch_m, th) for s in segs)
    total = sum(LineString(t).length for t in tracks if len(t) >= 2)
    return (total - strip_len) / total if total else None


if __name__ == "__main__":
    import algo_bcd_nfz as bcd
    from bench import boustrophedon, nfz_correct, nfz_metrics, rect_xy, run_planner_nfz
    polys = [rect_xy(0, 0, 2500, 2000)]
    nfz = [rect_xy(1200, 800, 700, 600)]
    alt = 140.0
    sw = 2 * alt * math.tan(math.radians(60.0) / 2)
    pitch = sw * 0.7
    print(f"作業區 20 km²、NFZ 1.68 km²、8 台、刈幅 {sw:.1f} m、內縮 {sw/2:.1f} m\n")
    print(f"{'方法':32s}{'台':>3s}{'總km':>8s}{'覆蓋':>8s}{'開銷':>7s}"
          f"{'掃描率':>8s}{'白飛%':>7s}{'NFZ內拍到km²':>13s}")

    def row(name, tr):
        m = nfz_metrics(polys, tr, sw, nfz)
        tr_ratio = transit_ratio(polys, tr, nfz, sw, pitch)
        print(f"{name:32s}{m['n_uav']:3d}{m['total_km']:8.1f}{m['coverage'] * 100:7.1f}%"
              f"{(m['overhead'] or 0):7.3f}{m['scan_rate']:8.2f}"
              f"{(tr_ratio or 0) * 100:6.1f}%{m['nfz_swept_km2']:13.4f}")

    b, _a, _r = nfz_correct(boustrophedon(polys, 8, swath_m=sw, pitch_m=pitch), nfz)
    row("牛耕＋事後繞行", b)
    t, _e, _a, _r = run_planner_nfz(False, polys, 8, 0, alt=alt, pitch=pitch,
                                    overlap=0.0, nfz_polys=nfz)
    row("DCCPP＋GUI 事後繞行", t)
    t, _e, _a, _r = run_planner_nfz(True, polys, 8, 0, alt=alt, pitch=pitch,
                                    overlap=0.0, nfz_polys=nfz)
    row("HARVEST 約束內規劃", t)
    row("BCD 胞分解", bcd.plan(polys, 8, swath_m=sw, pitch_m=pitch, nfz_polys=nfz))
    row("乾淨條帶優先（最小寬度）",
        plan(polys, 8, swath_m=sw, pitch_m=pitch, nfz_polys=nfz))
    row("乾淨條帶優先（髒條帶最少）",
        plan(polys, 8, swath_m=sw, pitch_m=pitch, nfz_polys=nfz,
             direction="fewest_dirty"))
