#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D1 test B: is the 'gain' statistic itself the artifact?

stot_lagcheck.m::rate_gain computes  gain = (x'y)/(x'x)  where
    x = (gam_cmd - gam)/lead        <- INSTANTANEOUS, unfiltered
    y = local_slope(gam, +-0.5 s)   <- a 1.0 s least-squares differentiator
No mean removal (regression through the origin), and the two signals do not
live in the same bandwidth.  Both of those attenuate a ratio-of-magnitudes.
"""
from __future__ import annotations
import math, statistics as st
import numpy as np
import d1_replicate as R


def movavg(t, x, half=0.5):
    """Same support as local_slope: mean of x over |t-t_k| <= half."""
    n = len(t); o = np.empty(n)
    for k in range(n):
        m = np.abs(t - t[k]) <= half
        o[k] = x[m].mean()
    return o


def variants(t, gam, gcmd, lead=1.0):
    _, iu = np.unique(t, return_index=True); iu = np.sort(iu)
    t, gam, gcmd = t[iu], gam[iu], gcmd[iu]
    y, _ = R.local_slope(t, gam, 0.5)
    x = (gcmd - gam) / lead
    xs = movavg(t, x, 0.5)                      # bandwidth-matched demand
    out = {}
    def thru(a, b): return float(a @ b / max(a @ a, 1e-12))
    def ols(a, b):
        ac, bc = a - a.mean(), b - b.mean()
        return float(ac @ bc / max(ac @ ac, 1e-12))
    def cc(a, b):
        ac, bc = a - a.mean(), b - b.mean()
        return float(ac @ bc / max(math.sqrt((ac@ac)*(bc@bc)), 1e-12))
    out["gate (thru-origin, raw x)"] = thru(x, y)
    out["OLS slope (mean-removed)"] = ols(x, y)
    out["thru-origin, bandwidth-matched x"] = thru(xs, y)
    out["OLS slope, bandwidth-matched x"] = ols(xs, y)
    out["reverse OLS  1/slope(y->x)"] = 1.0 / max(ols(y, x), 1e-12)
    out["ratio of medians |y|/|x|"] = float(np.median(np.abs(y)) / max(np.median(np.abs(x)), 1e-12))
    out["ratio of rms  rms(y)/rms(x)"] = float(np.sqrt((y@y)/(x@x)))
    out["corr"] = cc(x, y)
    out["corr bandwidth-matched"] = cc(xs, y)
    # what fraction of S_xx sits in ticks whose demand exceeds the +-7 deg/s cap
    V = None
    return out, x, xs, y


if __name__ == "__main__":
    keys = None; agg = {}
    Sxx_hi = Sxx_all = 0.0
    for tag in R.TAGS:
        d = R.load(tag)
        if len(d["t"]) < 60: continue
        gc, _, _ = R.replay_gamcmd(d["t"], d["r"], d["z"], d["gam"], d["gs"])
        v, x, xs, y = variants(d["t"], d["gam"], gc)
        if keys is None:
            keys = list(v); agg = {k: [] for k in keys}
        for k in keys: agg[k].append(v[k])
        cap = np.radians(7.0)
        hi = np.abs(x) > cap
        Sxx_hi += float(x[hi] @ x[hi]); Sxx_all += float(x @ x)
    print("=== the same 32 flights, only the ESTIMATOR changes ===")
    for k in keys:
        vv = [z for z in agg[k] if np.isfinite(z)]
        print(f"  {k:<38s} median {st.median(vv):+.4f}   "
              f"IQR {np.percentile(vv,25):+.4f}..{np.percentile(vv,75):+.4f}")
    print(f"\n  share of Sxx held by ticks whose demand exceeds the +-7 deg/s "
          f"physical cap: {100*Sxx_hi/Sxx_all:.1f}%")
