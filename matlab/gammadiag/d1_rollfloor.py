#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D1 test F: does the firmware pitch floor -80*|cos(ahrs.roll)| really never bite?

The briefing (and the two prior measurements) carry this node as 0.00% triggered,
on the strength of a reconstructed roll that is near zero.  Here we (a) read the
firmware's own ATT.Roll and ATT.DesPitch on the two sorties whose ground track
matches a log to ~1 m, and (b) build a roll estimate from the FLOWN track
curvature -- which needs no command reconstruction -- so the question can be
asked on all 32 sorties.
"""
from __future__ import annotations
import math, os
import numpy as np
import d1_replicate as R

SCR = ("C:/Users/joy46/AppData/Local/Temp/claude/"
       "C--Users-joy46-Documents-Projects-aeroplan-studio/"
       "460cd350-a1ff-48ba-b6f5-703543b5032c/scratchpad")
EARTH = 111320.0
G = 9.80665
MATCH = {"RRE7_U1": ("u1_419.npz", 1.000, 114.15),
         "RRE7_U2": ("u2_407.npz", 1.000, 114.15)}


def mlon(lat): return EARTH * np.cos(np.radians(lat))


def roll_from_track(tr):
    """bank from flown curvature:  tan(phi) = V_h * chi_dot / (g * cos gam)."""
    t, lat, lon, gam, gs = tr["t"], tr["lat"], tr["lon"], tr["gam"], tr["gs"]
    n = len(t); chi = np.full(n, np.nan)
    h = 2
    for k in range(h, n - h):
        dt = t[k + h] - t[k - h]
        if not (0.05 <= dt <= 0.6):
            continue
        dn = (lat[k + h] - lat[k - h]) * EARTH
        de = (lon[k + h] - lon[k - h]) * mlon(lat[k])
        if math.hypot(dn, de) < 1.0:
            continue
        chi[k] = math.atan2(de, dn)
    chid = np.full(n, np.nan)
    for k in range(h, n - h):
        dt = t[k + h] - t[k - h]
        if not (0.05 <= dt <= 0.6) or not (np.isfinite(chi[k+h]) and np.isfinite(chi[k-h])):
            continue
        dchi = (chi[k + h] - chi[k - h] + math.pi) % (2 * math.pi) - math.pi
        chid[k] = dchi / dt
    return np.degrees(np.arctan2(gs * chid, G * np.maximum(np.cos(gam), 1e-3))), chid


if __name__ == "__main__":
    print("=== A. firmware ground truth on the two exactly-matched sorties ===")
    for tag, (fn, a, b) in MATCH.items():
        d = np.load(os.path.join(SCR, fn))
        at, aroll = d["ATT_TimeUS"] * 1e-6, d["ATT_Roll"]
        adesp, apitch = d["ATT_DesPitch"], d["ATT_Pitch"]
        tr = R.load(tag)
        tl = a * tr["t"] + b
        roll = np.interp(tl, at, aroll)
        desp = np.interp(tl, at, adesp)
        pit = np.interp(tl, at, apitch)
        m = tr["z"] > 50
        floor = -80.0 * np.abs(np.cos(np.radians(roll)))
        clamped = np.abs(desp - floor) < 0.25
        rec, _ = roll_from_track(tr)
        mm = m & np.isfinite(rec)
        print(f"\n  {tag}  (n={m.sum()} dive ticks, z>50 m)")
        print(f"    |ATT.Roll|        med {np.median(np.abs(roll[m])):6.2f}  p90 "
              f"{np.percentile(np.abs(roll[m]),90):6.2f}  max "
              f"{np.max(np.abs(roll[m])):6.2f} deg")
        print(f"    firmware floor    med {np.median(floor[m]):6.1f}  p90(least neg) "
              f"{np.percentile(floor[m],90):6.1f}  worst {np.max(floor[m]):6.1f} deg")
        print(f"    ATT.DesPitch      med {np.median(desp[m]):6.1f}  min "
              f"{np.min(desp[m]):6.1f} deg")
        print(f"    ticks with DesPitch pinned to the floor: "
              f"{100*clamped[m].mean():.2f} %   ({clamped[m].sum()} ticks)")
        print(f"    track-curvature roll estimate vs ATT.Roll: corr "
              f"{np.corrcoef(rec[mm], roll[mm])[0,1]:+.4f}  bias "
              f"{np.median(rec[mm]-roll[mm]):+.2f}  |err| med "
              f"{np.median(np.abs(rec[mm]-roll[mm])):.2f} deg")

    print("\n=== B. track-curvature roll across all 32 sorties (no log needed) ===")
    print(f"{'sortie':<11s}{'|roll| med':>11s}{'p90':>8s}{'max':>8s}"
          f"{'floor med':>11s}{'floor worst':>13s}{'>|45| frac':>11s}")
    agg = []
    for tag in R.TAGS:
        tr = R.load(tag)
        rec, _ = roll_from_track(tr)
        m = (tr["z"] > 50) & np.isfinite(rec)
        if m.sum() < 100:
            continue
        rr = np.abs(rec[m])
        fl = -80.0 * np.cos(np.radians(np.minimum(rr, 89.0)))
        agg.append((tag, np.median(rr), np.percentile(rr, 90), rr.max(),
                    np.median(fl), fl.max(), (rr > 45).mean()))
    for a_ in agg:
        print(f"{a_[0]:<11s}{a_[1]:>11.2f}{a_[2]:>8.2f}{a_[3]:>8.2f}"
              f"{a_[4]:>11.1f}{a_[5]:>13.1f}{100*a_[6]:>10.1f}%")
    arr = np.array([[x[1], x[2], x[3], x[6]] for x in agg])
    print(f"\n  across-sortie medians: |roll| med {np.median(arr[:,0]):.2f} deg, "
          f"p90 {np.median(arr[:,1]):.2f}, max {np.median(arr[:,2]):.2f}, "
          f"frac>45deg {100*np.median(arr[:,3]):.1f}%")
