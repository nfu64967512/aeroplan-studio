#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D1 test H: a roll estimator that does not assume nz=1/cos(phi).

    nz_body * cos(phi) = cos(gam) + V*gdot/g        (vertical-plane component)
    nz_body * sin(phi) = V_h * chi_dot / g          (horizontal turning component)
    => tan(phi) = (V_h*chi_dot/g) / (cos gam + V*gdot/g)

Validated against ATT.Roll on the two exactly-matched sorties, then applied to
all 32 to ask how common the high-bank regime actually is.
"""
from __future__ import annotations
import csv, math, os
import numpy as np
import d1_replicate as R

SCR = ("C:/Users/joy46/AppData/Local/Temp/claude/"
       "C--Users-joy46-Documents-Projects-aeroplan-studio/"
       "460cd350-a1ff-48ba-b6f5-703543b5032c/scratchpad")
EARTH = 111320.0; G = 9.80665
MATCH = {"RRE7_U1": ("u1_419.npz", 1.000, 114.15),
         "RRE7_U2": ("u2_407.npz", 1.000, 114.15)}


def mlon(lat): return EARTH * np.cos(np.radians(lat))


def roll_proxy(tr):
    t, lat, lon, gam, gs = tr["t"], tr["lat"], tr["lon"], tr["gam"], tr["gs"]
    n = len(t); h = 2
    chi = np.full(n, np.nan)
    for k in range(h, n - h):
        dn = (lat[k + h] - lat[k - h]) * EARTH
        de = (lon[k + h] - lon[k - h]) * mlon(lat[k])
        if math.hypot(dn, de) > 1.0:
            chi[k] = math.atan2(de, dn)
    chid = np.full(n, np.nan)
    for k in range(h, n - h):
        dt = t[k + h] - t[k - h]
        if not (0.05 <= dt <= 0.6):
            continue
        if not (np.isfinite(chi[k + h]) and np.isfinite(chi[k - h])):
            continue
        chid[k] = ((chi[k + h] - chi[k - h] + math.pi) % (2 * math.pi) - math.pi) / dt
    gdot, _ = R.local_slope(t, gam, 0.5)
    V = gs / np.maximum(np.cos(gam), 1e-6)
    nz_vert = np.cos(gam) + V * gdot / G
    nz_horz = gs * chid / G
    return np.degrees(np.arctan2(nz_horz, nz_vert)), nz_vert, nz_horz


print("=== validation against ATT.Roll on the two 1 m matches ===")
for tag, (fn, a, b) in MATCH.items():
    d = np.load(os.path.join(SCR, fn))
    tr = R.load(tag)
    roll = np.interp(a * tr["t"] + b, d["ATT_TimeUS"] * 1e-6, d["ATT_Roll"])
    est, _, _ = roll_proxy(tr)
    m = (tr["z"] > 50) & np.isfinite(est)
    e = est[m] - roll[m]
    print(f"  {tag}: corr {np.corrcoef(est[m], roll[m])[0,1]:+.4f}  bias "
          f"{np.median(e):+.2f}  |err| med {np.median(np.abs(e)):.2f}  p90 "
          f"{np.percentile(np.abs(e),90):.2f} deg   "
          f"(est |roll| med {np.median(np.abs(est[m])):.1f} vs true "
          f"{np.median(np.abs(roll[m])):.1f})")

print("\n=== all 32 sorties: bank, roll-corrected ceiling, floor headroom ===")
truth = {}
with open(os.path.join("..", "data", "truth32.csv"), encoding="utf-8") as f:
    for row in csv.DictReader(f):
        truth[row["tag"].replace("-", "_")] = row
print(f"{'sortie':<10s}{'|roll|med':>10s}{'p90':>7s}{'ceil med':>10s}{'ceil p10':>9s}"
      f"{'occ med':>9s}{'floor<cmd?':>11s}{'err_touch':>11s}")
rows = []
for tag in R.TAGS:
    tr = R.load(tag)
    est, nzv, _ = roll_proxy(tr)
    gam, gs = tr["gam"], tr["gs"]
    gdot, _ = R.local_slope(tr["t"], gam, 0.5)
    V = gs / np.maximum(np.cos(gam), 1e-6)
    cphi = np.cos(np.radians(np.clip(np.abs(est), 0, 85)))
    m = (tr["z"] > 50) & np.isfinite(est)
    cap = (3.0 * cphi - np.cos(gam)) * G / V
    capd = (-1.0 * cphi - np.cos(gam)) * G / V
    occ = np.where(gdot >= 0, gdot / np.maximum(cap, 1e-6), gdot / np.minimum(capd, -1e-6))
    floor = -80.0 * cphi
    # the node's own pitch command floor proxy: gamma + alpha ~ gamma - 4.4 deg
    pcmd = np.degrees(gam) - 4.4
    bites = (floor > pcmd)
    e = truth.get(tag, {}).get("err_touch_m", "")
    rows.append((tag, np.median(np.abs(est[m])), np.percentile(np.abs(est[m]), 90),
                 np.degrees(np.median(cap[m])), np.degrees(np.percentile(cap[m], 10)),
                 np.median(np.abs(occ[m])), 100 * bites[m].mean(), e))
for r_ in rows:
    print(f"{r_[0]:<10s}{r_[1]:>10.1f}{r_[2]:>7.1f}{r_[3]:>+10.2f}{r_[4]:>+9.2f}"
          f"{r_[5]:>9.3f}{r_[6]:>10.1f}%{str(r_[7])[:9]:>11s}")
A = np.array([[r_[1], r_[3], r_[4], r_[5], r_[6]] for r_ in rows])
print(f"\n  medians across 32: |roll| {np.median(A[:,0]):.1f} deg,  ceiling "
      f"{np.median(A[:,1]):+.2f} deg/s (p10 {np.median(A[:,2]):+.2f}),  occupancy "
      f"{np.median(A[:,3]):.3f},  floor-above-command {np.median(A[:,4]):.1f}%")
hi = [r_ for r_ in rows if r_[1] > 15]
print(f"  sorties with |roll| median > 15 deg: {len(hi)}/32 -> "
      f"{', '.join(x[0] for x in hi)}")
