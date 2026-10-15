#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D1 test G: the discriminant, redone with the firmware's own roll.

The briefing's ceiling  gdot_max = (nz - cos gam)*g/V  assumes the load factor
acts wholly in the vertical plane.  With bank phi the vertical-plane part is
nz*cos(phi), so the true ceiling is  (nz*cos phi - cos gam)*g/V.  Bank is one of
the reconstructed quantities, so this check uses ATT.Roll from the log on the two
sorties whose ground track matches to ~1 m.
"""
from __future__ import annotations
import csv, math, os
import numpy as np
import d1_replicate as R

SCR = ("C:/Users/joy46/AppData/Local/Temp/claude/"
       "C--Users-joy46-Documents-Projects-aeroplan-studio/"
       "460cd350-a1ff-48ba-b6f5-703543b5032c/scratchpad")
G = 9.80665
MATCH = {"RRE7_U1": ("u1_419.npz", 1.000, 114.15),
         "RRE7_U2": ("u2_407.npz", 1.000, 114.15)}

for tag, (fn, a, b) in MATCH.items():
    d = np.load(os.path.join(SCR, fn))
    at = d["ATT_TimeUS"] * 1e-6
    ct = d["CTUN_TimeUS"] * 1e-6
    tr = R.load(tag)
    tl = a * tr["t"] + b
    roll = np.interp(tl, at, d["ATT_Roll"])
    desp = np.interp(tl, at, d["ATT_DesPitch"])
    pit = np.interp(tl, at, d["ATT_Pitch"])
    navp = np.interp(tl, ct, d["CTUN_NavPitch"])
    gam, gs, z = tr["gam"], tr["gs"], tr["z"]
    gdot, _ = R.local_slope(tr["t"], gam, 0.5)
    V = gs / np.maximum(np.cos(gam), 1e-6)
    cphi = np.cos(np.radians(roll))
    m = z > 50

    cap_flat_up = (3.0 - np.cos(gam)) * G / V
    cap_flat_dn = (-1.0 - np.cos(gam)) * G / V
    cap_roll_up = (3.0 * cphi - np.cos(gam)) * G / V
    cap_roll_dn = (-1.0 * cphi - np.cos(gam)) * G / V
    occ_flat = np.where(gdot >= 0, gdot / cap_flat_up, gdot / cap_flat_dn)
    occ_roll = np.where(gdot >= 0, gdot / np.maximum(cap_roll_up, 1e-6),
                        gdot / np.minimum(cap_roll_dn, -1e-6))
    floor = -80.0 * np.abs(cphi)
    clamp = np.abs(desp - floor) < 0.25

    print(f"\n===== {tag} =====   n = {m.sum()} dive ticks (z > 50 m)")
    print(f"  ATT.DesPitch vs CTUN.NavPitch: |diff| med "
          f"{np.median(np.abs(desp-navp)[m]):.4f} deg  max "
          f"{np.max(np.abs(desp-navp)[m]):.4f}  -> DesPitch is the post-clamp value")
    print(f"  |ATT.Roll|  med {np.median(np.abs(roll[m])):5.1f}  p90 "
          f"{np.percentile(np.abs(roll[m]),90):5.1f}  max {np.max(np.abs(roll[m])):5.1f} deg")
    print(f"  ceiling +  flat-wing model  med {np.degrees(np.median(cap_flat_up[m])):+6.2f} deg/s")
    print(f"  ceiling +  with real bank   med {np.degrees(np.median(cap_roll_up[m])):+6.2f}"
          f"   p10 {np.degrees(np.percentile(cap_roll_up[m],10)):+6.2f} deg/s")
    print(f"  |gdot| med {np.degrees(np.median(np.abs(gdot[m]))):.3f}  p90 "
          f"{np.degrees(np.percentile(np.abs(gdot[m]),90)):.3f} deg/s")
    print(f"  occupancy |gdot|/ceiling:  flat-wing med {np.median(np.abs(occ_flat[m])):.3f}"
          f"   with real bank med {np.median(np.abs(occ_roll[m])):.3f}")
    print(f"  ticks at >=90% of ceiling: flat-wing {100*(np.abs(occ_flat[m])>=0.9).mean():.1f}%"
          f"   with real bank {100*(np.abs(occ_roll[m])>=0.9).mean():.1f}%")
    print(f"  firmware pitch floor -80*|cos roll|: med {np.median(floor[m]):.1f} deg,"
          f"  least-negative {np.max(floor[m]):.1f} deg")
    print(f"  DesPitch pinned to that floor: {100*clamp[m].mean():.2f} %  "
          f"({clamp[m].sum()} ticks)   [briefing / M2 / M3 all say 0.00 %]")
    if clamp[m].sum():
        i = np.where(m & clamp)[0]
        print(f"     first pinned tick: t_csv {tr['t'][i[0]]:.2f}  roll "
              f"{roll[i[0]]:+.1f}  floor {floor[i[0]]:.1f}  DesPitch "
              f"{desp[i[0]]:.1f}  gamma {math.degrees(gam[i[0]]):+.1f}  z {z[i[0]]:.0f}")

# outcome of these two sorties, for context
with open(os.path.join("..", "data", "truth32.csv"), encoding="utf-8") as f:
    for row in csv.DictReader(f):
        if row["tag"] in ("RRE7-U1", "RRE7-U2"):
            print(f"\n  {row['tag']}: err_touch {row['err_touch_m']} m, "
                  f"z_res {row['z_res_m']} m, status {row['status']}, "
                  f"nz_clip_frac {row['nz_clip_frac']}")
