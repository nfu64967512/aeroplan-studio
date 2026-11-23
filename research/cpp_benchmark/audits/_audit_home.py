"""Fairness audit: does every method's track actually start and end at home (0,0)?
If the two project planners skip the home leg, their overhead is understated."""
import math, os, sys
sys.path.insert(0, os.getcwd())
from matrix import METHODS, SCEN, START
import matrix

def audit(method, scen, n):
    polys, _ = SCEN[scen]
    _fam, _fl, fn = METHODS[method]
    tracks, _ms = fn(polys, n)
    if not tracks:
        return None
    d0 = [math.dist(t[0], START) for t in tracks]
    d1 = [math.dist(t[-1], START) for t in tracks]
    return max(d0), max(d1), len(tracks)

for m in ["boustrophedon", "bous_v2", "spiral_v2", "grid_nn_v2b", "darp_v2",
          "DCCPP", "HARVEST"]:
    r = audit(m, "S1", 4)
    print(f"{m:16s} max|first-home|={r[0]:9.1f} m   max|last-home|={r[1]:9.1f} m   tracks={r[2]}"
          if r else f"{m:16s} EMPTY")
