#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D1 test C -- the decisive one.

Run the gate's OWN pipeline (replay_gamcmd -> rate_gain -> best_lag) on a
SYNTHETIC vehicle whose tracking behaviour is known by construction:

    gdot_act[k] = gdot_act[k-1] + (clip(demand[k-1], cap) - gdot_act[k-1])*dt/tau
    gam_syn[k+1] = gam_syn[k] + gdot_act[k]*dt

tau  = the true first-order tracking lag        (ground truth)
cap  = the true authority limit                 (ground truth)

The demand sequence is the one REPLAYED FROM THE REAL FLIGHT, so the synthetic
vehicle sees exactly the command spectrum the real one saw.  If the estimator
reports ~0.06 for a vehicle whose true gain is 1.0 and whose true lag is 0.05 s,
then 0.063 and 8.00 s measure the command, not the vehicle.
"""
from __future__ import annotations
import math, statistics as st
import numpy as np
import d1_replicate as R

CAP = np.radians(7.0)


def synth(t, x, tau, cap, gam0):
    """Open-loop: drive a known plant with the flight's own demand sequence."""
    n = len(t); act = 0.0; g = np.empty(n); g[0] = gam0
    rate = np.empty(n)
    for k in range(n):
        xc = x[k] if cap is None else float(np.clip(x[k], -cap, cap))
        rate[k] = act
        if k + 1 < n:
            dt = t[k + 1] - t[k]
            if not (0.0 < dt <= 0.6):
                dt = 0.05
            act += (xc - act) * min(dt / tau, 1.0)
            g[k + 1] = g[k] + act * dt
    return g, rate


def thru(a, b): return float(a @ b / max(a @ a, 1e-12))
def cc(a, b):
    ac, bc = a - a.mean(), b - b.mean()
    return float(ac @ bc / max(math.sqrt((ac @ ac) * (bc @ bc)), 1e-12))


if __name__ == "__main__":
    lags = np.arange(0, 8.0 + 1e-9, 0.05)
    cases = [("perfect,   no cap ", 0.05, None), ("tau=0.25s, no cap ", 0.25, None),
             ("tau=1.0s,  no cap ", 1.00, None), ("tau=3.0s,  no cap ", 3.00, None),
             ("perfect,   +-7deg/s", 0.05, CAP), ("tau=0.25s, +-7deg/s", 0.25, CAP),
             ("tau=1.0s,  +-7deg/s", 1.00, CAP), ("tau=3.0s,  +-7deg/s", 3.00, CAP)]
    res = {c[0]: dict(g=[], c=[], l=[], ceil=0) for c in cases}
    real = dict(g=[], c=[], l=[], ceil=0)

    for tag in R.TAGS:
        d = R.load(tag)
        if len(d["t"]) < 60: continue
        t, gam = d["t"], d["gam"]
        gc, _, _ = R.replay_gamcmd(t, d["r"], d["z"], gam, d["gs"])
        _, iu = np.unique(t, return_index=True); iu = np.sort(iu)
        tu, gamu, gcu = t[iu], gam[iu], gc[iu]
        x = (gcu - gamu) / 1.0
        y, _ = R.local_slope(tu, gamu, 0.5)
        real["g"].append(thru(x, y)); real["c"].append(cc(x, y))
        l, _, _ = R.best_lag(tu, gcu, tu, gamu, lags)
        real["l"].append(l); real["ceil"] += (l >= 8.0 - 1e-9)
        for name, tau, cap in cases:
            gs_, _ = synth(tu, x, tau, cap, gamu[0])
            ys, _ = R.local_slope(tu, gs_, 0.5)
            res[name]["g"].append(thru(x, ys)); res[name]["c"].append(cc(x, ys))
            ls_, _, _ = R.best_lag(tu, gcu, tu, gs_, lags)
            res[name]["l"].append(ls_); res[name]["ceil"] += (ls_ >= 8.0 - 1e-9)

    print("=== the gate's estimator applied to vehicles of KNOWN tracking quality ===")
    print(f"{'synthetic vehicle':<22s}{'true gain':>10s}{'true lag':>10s}"
          f"{'REPORTED':>10s}{'REPORTED':>10s}{'REPORTED':>10s}{'at 8s':>8s}")
    print(f"{'':<22s}{'':>10s}{'':>10s}{'gain':>10s}{'corr':>10s}{'lag s':>10s}{'ceiling':>8s}")
    for name, tau, cap in cases:
        r = res[name]
        tg = "1.000" if cap is None else "1.000*"
        print(f"{name:<22s}{tg:>10s}{tau:>10.2f}{st.median(r['g']):>10.3f}"
              f"{st.median(r['c']):>+10.3f}{st.median(r['l']):>10.2f}"
              f"{r['ceil']:>5d}/32")
    print(f"{'REAL AIRCRAFT':<22s}{'?':>10s}{'?':>10s}{st.median(real['g']):>10.3f}"
          f"{st.median(real['c']):>+10.3f}{st.median(real['l']):>10.2f}"
          f"{real['ceil']:>5d}/32")
    print("\n  * true gain is 1.000 up to the +-7 deg/s cap; the cap is the only")
    print("    thing limiting it, and it is the SAME physical cap the briefing quotes.")
