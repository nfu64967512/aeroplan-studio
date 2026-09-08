from pathlib import Path
"""darp vs darp_v2 vs boustrophedon vs DCCPP vs HARVEST，全部由 bench.metrics 評分。"""
import math
import sys
import time

# ⚠ 原本硬編 session scratchpad 的路徑（會消失），改成 repo 相對。
BENCH = str(Path(__file__).resolve().parent.parent)
if BENCH not in sys.path:
    sys.path.insert(0, BENCH)

from shapely import affinity                                    # noqa: E402
from shapely.geometry import Polygon                            # noqa: E402

from bench import boustrophedon, metrics, rect_xy, run_planner  # noqa: E402
from algo_darp import plan as darp                              # noqa: E402
from algo_darp_v2 import plan as darp2                          # noqa: E402

ALT, HFOV = 140.0, 60.0
SWATH = 2 * ALT * math.tan(math.radians(HFOV) / 2)
PITCH = SWATH * 0.7
HDR = (f"{'method':16s}{'n':>3s}{'total km':>10s}{'max km':>9s}{'cov':>8s}"
       f"{'ovh':>7s}{'red':>7s}{'bal':>7s}{'scan km2/h':>12s}{'waste':>8s}{'s':>7s}")


def row(name, polys, tracks, dt=0.0):
    m = metrics(polys, tracks, SWATH)
    print(f"{name:16s}{m['n_uav']:3d}{m['total_km']:10.1f}{m['max_km']:9.2f}"
          f"{m['coverage'] * 100:7.2f}%{m['overhead'] or 0:7.3f}"
          f"{m['redundancy'] or 0:7.3f}{m['balance'] or 0:7.3f}"
          f"{m['scan_rate']:12.2f}{(m['waste'] or 0) * 100:7.1f}%{dt:7.1f}")
    return m


def timed(fn, *a, **k):
    t0 = time.perf_counter()
    out = fn(*a, **k)
    return out, time.perf_counter() - t0


def block(title, polys, n, planners=True):
    print(f"\n=== {title} ===  {sum(p.area for p in polys) / 1e6:.2f} km2  n={n}")
    print(HDR)
    row("boustrophedon", polys, boustrophedon(polys, n, swath_m=SWATH, pitch_m=PITCH))
    if planners:
        t, _ms, err = run_planner(False, polys, n, 0, alt=ALT, pitch=PITCH, overlap=0.0)
        row("DCCPP", polys, t) if not err else print("DCCPP  ", err)
        t, _ms, err = run_planner(True, polys, n, 0, alt=ALT, pitch=PITCH, overlap=0.0)
        row("HARVEST", polys, t) if not err else print("HARVEST", err)
    t, dt = timed(darp, polys, n, swath_m=SWATH, pitch_m=PITCH)
    a = row("darp", polys, t, dt)
    t, dt = timed(darp2, polys, n, swath_m=SWATH, pitch_m=PITCH)
    b = row("darp_v2", polys, t, dt)
    if a["scan_rate"]:
        print(f"{'':16s}v2 vs v1 scan_rate {(b['scan_rate'] / a['scan_rate'] - 1) * 100:+.2f}%"
              f"   max {(b['max_km'] / a['max_km'] - 1) * 100:+.2f}%")


L = Polygon([(-2000, -2000), (2000, -2000), (2000, 0),
             (0, 0), (0, 2000), (-2000, 2000)])
QUAD = [rect_xy(cx, 0, 625, 2000) for cx in (-3750, -1250, 1250, 3750)]
PLUS = Polygon([(-600, -2000), (600, -2000), (600, -600), (2000, -600), (2000, 600),
                (600, 600), (600, 2000), (-600, 2000), (-600, 600), (-2000, 600),
                (-2000, -600), (-600, -600)])
DONUT = Polygon([(-2000, -2000), (2000, -2000), (2000, 2000), (-2000, 2000)],
                [[(-700, -700), (700, -700), (700, 700), (-700, 700)]])
SCATTER = [rect_xy(-4000, 3000, 500, 500), rect_xy(3500, -2500, 900, 700),
           rect_xy(0, 0, 1500, 400), rect_xy(-1000, -3500, 300, 1200),
           rect_xy(4200, 3600, 700, 400)]
TILT = [affinity.rotate(rect_xy(0, 0, 2500, 2000), 37.0, origin=(0, 0))]

if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "main"
    if what in ("main", "all"):
        block("S1 single 5.0x4.0 km rect", [rect_xy(0, 0, 2500, 2000)], 8)
        block("S2 four 1.25x4.0 km blocks", QUAD, 8)
        block("S3 L-shape concave", [L], 4)
    if what in ("more", "all"):
        block("S4 tilted 37 deg", TILT, 8, planners=False)
        block("S5 L-shape n=8", [L], 8, planners=False)
        block("S6 plus/cross n=4", [PLUS], 4, planners=False)
        block("S7 donut n=4", [DONUT], 4, planners=False)
        block("S8 scatter 5 blocks n=6", SCATTER, 6, planners=False)
        block("S9 four blocks n=3", QUAD, 3, planners=False)
        block("S10 thin strip 6000x90 n=4", [rect_xy(0, 0, 3000, 45)], 4, planners=False)
    if what in ("sweep", "all"):
        print("\n=== n sweep, single 5x4 km rect (scan_rate km2/h) ===")
        rect = [rect_xy(0, 0, 2500, 2000)]
        print(f"{'n':>3s}{'bous':>9s}{'darp':>9s}{'darp_v2':>9s}{'v2 gain':>10s}{'s':>7s}")
        for n in (1, 2, 3, 4, 5, 6, 7, 8, 11, 16):
            mb = metrics(rect, boustrophedon(rect, n, swath_m=SWATH, pitch_m=PITCH), SWATH)
            t1, _ = timed(darp, rect, n, swath_m=SWATH, pitch_m=PITCH)
            m1 = metrics(rect, t1, SWATH)
            t2, dt = timed(darp2, rect, n, swath_m=SWATH, pitch_m=PITCH)
            m2 = metrics(rect, t2, SWATH)
            print(f"{n:3d}{mb['scan_rate']:9.2f}{m1['scan_rate']:9.2f}{m2['scan_rate']:9.2f}"
                  f"{(m2['scan_rate'] / m1['scan_rate'] - 1) * 100:+9.2f}%{dt:7.1f}")
