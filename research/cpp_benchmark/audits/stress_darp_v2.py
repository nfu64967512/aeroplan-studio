from pathlib import Path
"""隨機形狀壓力測試：作者自己挑的情境之外，darp / darp_v2 會不會輸給基準線或破功。

隨機生成凸/凹/多塊/帶洞的作業區與隨機 home，全部由 bench.metrics 評分。
輸出：覆蓋率不足、回傳條數不對、或掃描率輸給 boustrophedon 的案例。
"""
import math
import random
import sys
import time

# ⚠ 原本硬編 session scratchpad 的路徑（會消失），改成 repo 相對。
BENCH = str(Path(__file__).resolve().parent.parent)
if BENCH not in sys.path:
    sys.path.insert(0, BENCH)

from shapely import affinity                                   # noqa: E402
from shapely.geometry import Point, Polygon                    # noqa: E402
from shapely.ops import unary_union                            # noqa: E402

from bench import boustrophedon, metrics, rect_xy              # noqa: E402
from algo_darp import plan as darp                             # noqa: E402
from algo_darp_v2 import plan as darp2                         # noqa: E402

SWATH = 2 * 140.0 * math.tan(math.radians(60.0) / 2)
PITCH = SWATH * 0.7


def make(rng, kind):
    if kind == "convex":
        pts = [(rng.uniform(-3000, 3000), rng.uniform(-3000, 3000)) for _ in range(7)]
        return [unary_union([Point(p).buffer(1) for p in pts]).convex_hull]
    if kind == "concave":
        base = rect_xy(0, 0, rng.uniform(1500, 3000), rng.uniform(1200, 2500))
        cut = rect_xy(rng.uniform(-1500, 1500), rng.uniform(-1200, 1200),
                      rng.uniform(400, 1200), rng.uniform(400, 1200))
        g = base.difference(cut)
        g = g.geoms[0] if g.geom_type == "MultiPolygon" else g
        return [affinity.rotate(g, rng.uniform(0, 180), origin=(0, 0))]
    if kind == "holed":
        base = rect_xy(0, 0, rng.uniform(1800, 3000), rng.uniform(1500, 2500))
        hole = rect_xy(rng.uniform(-800, 800), rng.uniform(-700, 700), 500, 400)
        return [base.difference(hole)]
    k = rng.randint(2, 4)
    return [rect_xy(rng.uniform(-4500, 4500), rng.uniform(-3500, 3500),
                    rng.uniform(400, 1100), rng.uniform(400, 1400)) for _ in range(k)]


def run(trials=28, seed=7):
    rng = random.Random(seed)
    bad, lose1, lose2 = [], [], []
    rows = []
    for t in range(trials):
        kind = ("convex", "concave", "holed", "multi")[t % 4]
        polys = make(rng, kind)
        if unary_union(polys).area < 1.0e6:
            continue
        n = rng.randint(2, 9)
        start = (rng.uniform(-3000, 3000), rng.uniform(-3000, 3000)) if t % 3 == 0 \
            else (0.0, 0.0)
        mb = metrics(polys, boustrophedon(polys, n, swath_m=SWATH, pitch_m=PITCH,
                                          start=start), SWATH)
        t0 = time.perf_counter()
        d1 = darp(polys, n, swath_m=SWATH, pitch_m=PITCH, start=start)
        t1 = time.perf_counter()
        d2 = darp2(polys, n, swath_m=SWATH, pitch_m=PITCH, start=start)
        t2 = time.perf_counter()
        m1, m2 = metrics(polys, d1, SWATH), metrics(polys, d2, SWATH)
        for tag, tr, mm in (("v1", d1, m1), ("v2", d2, m2)):
            if mm["coverage"] < 0.99:
                bad.append(f"#{t} {kind} n={n} {tag} cov={mm['coverage']:.4f}")
            if len(tr) != n:
                bad.append(f"#{t} {kind} n={n} {tag} 回傳 {len(tr)} 條")
            for i, trk in enumerate(tr):
                if tuple(trk[0]) != tuple(start) or tuple(trk[-1]) != tuple(start):
                    bad.append(f"#{t} {kind} {tag} track{i} 頭尾不在 start")
        if m1["scan_rate"] < mb["scan_rate"] - 1e-9:
            lose1.append(f"#{t} {kind} n={n} v1 {m1['scan_rate']:.2f} < bous {mb['scan_rate']:.2f}")
        if m2["scan_rate"] < mb["scan_rate"] - 1e-9:
            lose2.append(f"#{t} {kind} n={n} v2 {m2['scan_rate']:.2f} < bous {mb['scan_rate']:.2f}")
        rows.append((t, kind, n, mb["scan_rate"], m1["scan_rate"], m2["scan_rate"],
                     mb["coverage"], m1["coverage"], m2["coverage"], t1 - t0, t2 - t1))
        print(f"#{t:2d} {kind:8s} n={n} home={'off' if start != (0.0, 0.0) else 'orig'}  "
              f"bous {mb['scan_rate']:6.2f}/{mb['coverage'] * 100:6.2f}%  "
              f"v1 {m1['scan_rate']:6.2f}/{m1['coverage'] * 100:6.2f}%({t1 - t0:4.1f}s)  "
              f"v2 {m2['scan_rate']:6.2f}/{m2['coverage'] * 100:6.2f}%({t2 - t1:4.1f}s)  "
              f"v2/v1 {(m2['scan_rate'] / m1['scan_rate'] - 1) * 100:+6.2f}%")
    g1 = [r[4] / r[3] for r in rows if r[3] > 0]
    g2 = [r[5] / r[3] for r in rows if r[3] > 0]
    gv = [r[5] / r[4] for r in rows if r[4] > 0]
    print(f"\n{len(rows)} 例：v1/bous 中位 {sorted(g1)[len(g1) // 2]:.3f} "
          f"最差 {min(g1):.3f}  |  v2/bous 中位 {sorted(g2)[len(g2) // 2]:.3f} "
          f"最差 {min(g2):.3f}  |  v2/v1 中位 {sorted(gv)[len(gv) // 2]:.4f} "
          f"最差 {min(gv):.4f} 最好 {max(gv):.4f}")
    print("契約/覆蓋率問題:", bad or "無")
    print("v1 輸給基準線:", lose1 or "無")
    print("v2 輸給基準線:", lose2 or "無")


if __name__ == "__main__":
    run(int(sys.argv[1]) if len(sys.argv) > 1 else 28)
