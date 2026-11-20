#!/usr/bin/env python3
"""M2 pass 4 -- cross-validations that the strong claims depend on.

  1. n_z from the accelerometer  vs  n_z from kinematics (cos g + V*gdot/g).
     Two independent routes; if they agree the -2.3 g claim stands.
  2. where in the PN window the over-ceiling / saturated ticks sit.
  3. what the LOS floor actually demands at the instant it engages.
"""
import numpy as np, csv, statistics as st

CACHE = "/home/windows-wsl/gammadiag_cache"
REPO = "/mnt/c/Users/joy46/Documents/Projects/aeroplan-studio"
TRIALS = [f"RRD{i}" for i in range(1, 9)] + [f"RRE{i}" for i in range(1, 9)]
G = 9.80665

agree, prof, losprof = [], [], []
for tr in TRIALS:
    for s in (1, 2):
        z = np.load(f"{CACHE}/dump_{tr}_U{s}.npz")
        tA, gam, gd, V, nzm = z["tA"], z["gam"], z["gamdot"], z["V"], z["nz_meas"]
        roll = np.interp(tA, z["tl"], z["r_roll"])
        keep = np.isfinite(gd) & (tA <= tA[-1] - 1.0)
        # kinematic load factor in the vertical plane, de-banked
        nzk = (np.cos(np.radians(gam)) + V * np.radians(gd) / G) / np.maximum(
            np.cos(np.radians(roll)), 0.2)
        a, b = nzm[keep], nzk[keep]
        agree.append(dict(trial=tr, sysid=s,
                          corr=float(np.corrcoef(a, b)[0, 1]),
                          bias=float(np.median(a - b)),
                          absmed=float(np.median(np.abs(a - b))),
                          nzm_min=float(a.min()), nzk_min=float(b.min()),
                          nzm_p01=float(np.percentile(a, 1)),
                          nzk_p01=float(np.percentile(b, 1))))

        # --- where do the over-ceiling ticks sit? (fraction of PN window)
        tl = z["tl"]
        dem = z["gam_cmd"] - z["gamr"]
        cu = np.interp(tl, tA, z["ceil_up"]); cd = np.interp(tl, tA, z["ceil_dn"])
        over = (dem > cu) | (dem < cd)
        u = (tl - tl[0]) / (tl[-1] - tl[0])
        for lo, hi in [(0, .2), (.2, .4), (.4, .6), (.6, .8), (.8, 1.0)]:
            m = (u >= lo) & (u < hi)
            prof.append((lo, float(over[m].mean())))
        # --- LOS floor engagement profile
        los = z["gam_cmd"] < z["gam_cmd_pn"] - 1e-12
        for lo, hi in [(0, .2), (.2, .4), (.4, .6), (.6, .8), (.8, 1.0)]:
            m = (u >= lo) & (u < hi)
            losprof.append((lo, float(los[m].mean())))

print("=" * 74)
print("1. n_z cross-validation: accelerometer vs kinematics (independent)")
print("=" * 74)
for k in ["corr", "bias", "absmed", "nzm_p01", "nzk_p01", "nzm_min", "nzk_min"]:
    v = sorted(r[k] for r in agree)
    print(f"  {k:9s} median {st.median(v):+7.3f}   min {v[0]:+7.3f}  max {v[-1]:+7.3f}")
print()
print("=" * 74)
print("2. WHEN are the over-ceiling demands issued?  (fifths of the PN window)")
print("=" * 74)
print(f"  {'window':>12s} {'demand>ceiling':>16s} {'LOS floor active':>18s}")
for lo in [0, .2, .4, .6, .8]:
    o = st.median([p[1] for p in prof if abs(p[0] - lo) < 1e-9])
    l = st.median([p[1] for p in losprof if abs(p[0] - lo) < 1e-9])
    print(f"  {lo:.0%}-{lo+.2:.0%}  {o:15.1%} {l:17.1%}")
