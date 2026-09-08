from pathlib import Path
"""三情境 + n 掃描：boustrophedon / DCCPP / HARVEST / darp / darp_v2。"""
import math, sys, time
# ⚠ 原本硬編 session scratchpad 的路徑（會消失），改成 repo 相對。
BENCH = str(Path(__file__).resolve().parent.parent)
sys.path.insert(0, BENCH)
from shapely.geometry import Polygon
from bench import boustrophedon, metrics, rect_xy, run_planner
from algo_darp import plan as darp
from algo_darp_v2 import plan as darp2
ALT, HFOV = 140.0, 60.0
SWATH = 2*ALT*math.tan(math.radians(HFOV)/2); PITCH = SWATH*0.7
HDR = f"{'method':16s}{'n':>3s}{'total km':>10s}{'max km':>9s}{'cov':>8s}{'ovh':>7s}{'red':>7s}{'bal':>7s}{'scan km2/h':>12s}{'waste':>8s}{'sec':>7s}"
def row(name, polys, tracks, dt=0.0):
    m = metrics(polys, tracks, SWATH)
    print(f"{name:16s}{m['n_uav']:3d}{m['total_km']:10.1f}{m['max_km']:9.2f}"
          f"{m['coverage']*100:7.2f}%{m['overhead'] or 0:7.3f}{m['redundancy'] or 0:7.3f}"
          f"{m['balance'] or 0:7.3f}{m['scan_rate']:12.2f}{(m['waste'] or 0)*100:7.1f}%{dt:7.1f}")
def timed(fn, *a, **kw):
    t0=time.perf_counter(); r=fn(*a,**kw); return r, time.perf_counter()-t0
def block(title, polys, n, planners=True):
    print(f"\n=== {title} ===  {sum(p.area for p in polys)/1e6:.2f} km2  n={n}")
    print(HDR)
    t,dt = timed(boustrophedon, polys, n, swath_m=SWATH, pitch_m=PITCH); row("boustrophedon", polys, t, dt)
    if planners:
        t,_ms,err = run_planner(False, polys, n, 0, alt=ALT, pitch=PITCH, overlap=0.0)
        row("DCCPP", polys, t) if not err else print("DCCPP", err)
        t,_ms,err = run_planner(True, polys, n, 0, alt=ALT, pitch=PITCH, overlap=0.0)
        row("HARVEST", polys, t) if not err else print("HARVEST", err)
    t,dt = timed(darp, polys, n, swath_m=SWATH, pitch_m=PITCH); row("darp", polys, t, dt)
    t,dt = timed(darp2, polys, n, swath_m=SWATH, pitch_m=PITCH); row("darp_v2", polys, t, dt)
if __name__ == "__main__":
    block("S1 single 5.0x4.0 km rect", [rect_xy(0,0,2500,2000)], 8)
    quad = [rect_xy(cx,0,625,2000) for cx in (-3750,-1250,1250,3750)]
    block("S2 four 1.25x4.0 km blocks", quad, 8)
    L = Polygon([(-2000,-2000),(2000,-2000),(2000,0),(0,0),(0,2000),(-2000,2000)])
    block("S3 L-shape concave", [L], 4)
