from pathlib import Path
"""darp vs boustrophedon vs DCCPP vs HARVEST，全部由 bench.metrics 評分。"""
import math
import sys

# ⚠ 原本硬編 session scratchpad 的路徑（會消失），改成 repo 相對。
BENCH = str(Path(__file__).resolve().parent.parent)
if BENCH not in sys.path:
    sys.path.insert(0, BENCH)

from shapely.geometry import Polygon                          # noqa: E402

from bench import boustrophedon, metrics, rect_xy, run_planner  # noqa: E402
from algo_darp import plan as darp                            # noqa: E402

ALT, HFOV = 140.0, 60.0
SWATH = 2 * ALT * math.tan(math.radians(HFOV) / 2)
PITCH = SWATH * 0.7
HDR = (f"{'方法':16s}{'台':>3s}{'總 km':>8s}{'最長 km':>9s}{'覆蓋':>8s}"
       f"{'開銷':>7s}{'重複':>7s}{'不均':>7s}{'掃描率 km²/h':>13s}{'區外浪費':>9s}")


def row(name, polys, tracks):
    m = metrics(polys, tracks, SWATH)
    print(f"{name:16s}{m['n_uav']:3d}{m['total_km']:8.1f}{m['max_km']:9.1f}"
          f"{m['coverage'] * 100:7.1f}%{m['overhead'] or 0:7.3f}"
          f"{m['redundancy'] or 0:7.3f}{m['balance'] or 0:7.3f}"
          f"{m['scan_rate']:13.2f}{(m['waste'] or 0) * 100:8.1f}%")


def block(title, polys, n, planners=True):
    print(f"\n=== {title} ===  {sum(p.area for p in polys) / 1e6:.2f} km²  n={n}  "
          f"刈幅 {SWATH:.1f} m  間距 {PITCH:.1f} m")
    print(HDR)
    row("boustrophedon", polys, boustrophedon(polys, n, swath_m=SWATH, pitch_m=PITCH))
    if planners:
        t, _ms, err = run_planner(False, polys, n, 0, alt=ALT, pitch=PITCH, overlap=0.0)
        row("DCCPP", polys, t) if not err else print("DCCPP  ", err)
        t, _ms, err = run_planner(True, polys, n, 0, alt=ALT, pitch=PITCH, overlap=0.0)
        row("HARVEST", polys, t) if not err else print("HARVEST", err)
    row("darp", polys, darp(polys, n, swath_m=SWATH, pitch_m=PITCH))


if __name__ == "__main__":
    block("情境1 單一 5.0x4.0 km 矩形", [rect_xy(0, 0, 2500, 2000)], 8)
    quad = [rect_xy(cx, 0, 625, 2000) for cx in (-3750, -1250, 1250, 3750)]
    block("情境2 四塊 1.25x4.0 km（橫排、home 在中間空隙）", quad, 8)
    L = Polygon([(-2000, -2000), (2000, -2000), (2000, 0),
                 (0, 0), (0, 2000), (-2000, 2000)])
    block("情境3 L 形凹多邊形（4x4 挖掉右上 2x2），home 在凹角", [L], 4)
