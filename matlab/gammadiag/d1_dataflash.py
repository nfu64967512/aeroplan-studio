#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D1 test E: spot-check the RECONSTRUCTED nodes against the firmware's own log.

Nodes checked (all three are reconstructions in the briefing's decay table):
  nz    : cos(gam) + V*gamdot/g            vs  -IMU.AccZ/g * cos(ATT.Roll)
  roll  : atan2(a_h, g*max(cos gam,0.25))  vs  ATT.DesRoll / ATT.Roll
  pitch : firmware floor -80*|cos(roll)|   vs  ATT.DesPitch actually reached

The BIN<->sortie mapping is established here by ground-track matching against all
16 candidate U1 (or U2) sorties, not taken on trust from file mtime.  ATT.* in a
dataflash log are DEGREES; IMU.AccZ is m/s^2 body-z (down positive is negative).
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
PN = R.PN


def mlon(lat): return EARTH * np.cos(np.radians(lat))


def load_bin(path):
    z = np.load(path)
    d = {k: z[k] for k in z.files}
    for pre in ("ATT", "IMU", "SIM", "POS", "CTUN", "XKF1"):
        d[pre + "_t"] = d[pre + "_TimeUS"] * 1e-6
    return d


def track_match(bin_d, tag):
    tr = R.load(tag)
    lat, lon, t = tr["lat"], tr["lon"], tr["t"]
    bl, bo, bt = bin_d["SIM_Lat"], bin_d["SIM_Lng"], bin_d["SIM_t"]
    best = (1.0, 0.0, 1e18)

    def ev(a, b):
        tt = a * t + b
        m = (tt >= bt[0]) & (tt <= bt[-1])
        if m.sum() < 200:
            return 1e18
        li = np.interp(tt[m], bt, bl); oi = np.interp(tt[m], bt, bo)
        dn = (li - lat[m]) * EARTH; de = (oi - lon[m]) * mlon(lat[m])
        return float(np.sqrt(np.mean(dn * dn + de * de)))

    for a in np.arange(0.82, 1.041, 0.01):
        for b in np.arange(-100, 500, 1.0):
            r = ev(a, b)
            if r < best[2]:
                best = (float(a), float(b), r)
    a0, b0 = best[0], best[1]
    for a in np.arange(a0 - 0.012, a0 + 0.0121, 0.001):
        for b in np.arange(b0 - 2, b0 + 2.01, 0.05):
            r = ev(a, b)
            if r < best[2]:
                best = (float(a), float(b), r)
    return best


def report(num, side, claimed, cands):
    p = os.path.join(SCR, f"{side}_{num}.npz")
    if not os.path.exists(p):
        print("missing", p); return
    d = load_bin(p)
    print(f"\n########## {side.upper()} BIN {num}   (mtime claim: {claimed}) ##########")
    sc = sorted(((track_match(d, tg), tg) for tg in cands),
                key=lambda e: e[0][2])
    for (a, b, rms), tg in sc[:3]:
        fl = "   <== mtime claim" if tg == claimed else ""
        print(f"    {tg:<10s} track rms {rms:8.1f} m   a={a:.3f}  b={b:+.2f}{fl}")
    (a, b, rms), tag = sc[0]
    if rms > 40:
        print(f"    !! best match {rms:.0f} m is not decisive -- node checks skipped")
        return None
    tr = R.load(tag)
    t, gam, gs, r_, z = tr["t"], tr["gam"], tr["gs"], tr["r"], tr["z"]
    tl = a * t + b
    gdot, _ = R.local_slope(t, gam, 0.5)
    V = gs / np.maximum(np.cos(gam), 1e-6)
    nz_recon = np.cos(gam) + V * gdot / G

    accz = np.interp(tl, d["IMU_t"], d["IMU_AccZ"])
    roll_m = np.interp(tl, d["ATT_t"], d["ATT_Roll"])          # DEGREES
    desroll = np.interp(tl, d["ATT_t"], d["ATT_DesRoll"])      # DEGREES
    pitch_m = np.interp(tl, d["ATT_t"], d["ATT_Pitch"])        # DEGREES
    despitch = np.interp(tl, d["ATT_t"], d["ATT_DesPitch"])    # DEGREES
    nz_meas = (-accz / G) * np.cos(np.radians(roll_m))

    m = (z > 50) & np.isfinite(nz_recon) & np.isfinite(nz_meas)
    e = nz_recon[m] - nz_meas[m]
    print(f"  [node nz]    corr {np.corrcoef(nz_recon[m], nz_meas[m])[0,1]:+.4f}"
          f"   bias {np.median(e):+.3f} g   |err| med {np.median(np.abs(e)):.3f}"
          f"   p95 {np.percentile(np.abs(e),95):.3f}")
    print(f"      recon  p01 {np.percentile(nz_recon[m],1):+.2f}  med "
          f"{np.median(nz_recon[m]):+.2f}  p99 {np.percentile(nz_recon[m],99):+.2f}"
          f"  max {nz_recon[m].max():+.2f}")
    print(f"      IMU    p01 {np.percentile(nz_meas[m],1):+.2f}  med "
          f"{np.median(nz_meas[m]):+.2f}  p99 {np.percentile(nz_meas[m],99):+.2f}"
          f"  max {nz_meas[m].max():+.2f}")

    # --- roll reconstruction from the PN lateral channel ---
    lat, lon = tr["lat"], tr["lon"]
    tlat = lat + 0.0; tlon = lon + 0.0     # target recovered below
    import csv as _csv
    trial = tag.split("_")[0]
    with open(os.path.join("..", "..", "runs", "artifacts", trial,
                           f"tot_{trial}_geom.csv"), encoding="utf-8") as f:
        for row in _csv.DictReader(f):
            if row["kind"] == "target":
                TLAT, TLON = float(row["lat"]), float(row["lon"]); break
    dn = (TLAT - lat) * EARTH; de = (TLON - lon) * mlon(lat)
    dd = np.maximum(np.hypot(dn, de), 1e-3)
    vn = np.gradient(lat) * EARTH / np.maximum(np.gradient(t), 1e-9)
    ve = np.gradient(lon) * mlon(lat) / np.maximum(np.gradient(t), 1e-9)
    ok = np.abs(np.gradient(t)) > 1e-6
    lam_dot = (de * vn - dn * ve) / (dd * dd)
    vh = gs
    a_h = 4.0 * vh * lam_dot
    roll_recon = np.degrees(np.arctan2(a_h, G * np.maximum(np.cos(gam), 0.25)))
    roll_recon = np.clip(roll_recon, -80, 80)
    mm = m & ok & np.isfinite(roll_recon)
    er = roll_recon[mm] - desroll[mm]
    print(f"  [node roll]  recon vs ATT.DesRoll: corr "
          f"{np.corrcoef(roll_recon[mm], desroll[mm])[0,1]:+.4f}  bias "
          f"{np.median(er):+.2f} deg  |err| med {np.median(np.abs(er)):.2f}"
          f"  p95 {np.percentile(np.abs(er),95):.2f}")
    print(f"      |ATT.Roll| med {np.median(np.abs(roll_m[mm])):.2f} deg  p90 "
          f"{np.percentile(np.abs(roll_m[mm]),90):.2f}  max "
          f"{np.max(np.abs(roll_m[mm])):.2f}")
    print(f"  [node pitch] |ATT.Pitch-ATT.DesPitch| med "
          f"{np.median(np.abs(pitch_m-despitch)[mm]):.2f} deg;  DesPitch min "
          f"{despitch[mm].min():.1f} deg;  firmware floor -80*|cos roll| worst "
          f"{(-80*np.abs(np.cos(np.radians(roll_m))))[mm].max():.1f} deg")
    return dict(tag=tag, a=a, b=b, rms=rms)


if __name__ == "__main__":
    u1 = [t for t in R.TAGS if t.endswith("_U1")]
    u2 = [t for t in R.TAGS if t.endswith("_U2")]
    for num, claim in [(413, "RRE1_U1"), (417, "RRE5_U1"), (419, "RRE7_U1")]:
        report(num, "u1", claim, u1)
    for num, claim in [(401, "RRE1_U2"), (405, "RRE5_U2"), (407, "RRE7_U2")]:
        report(num, "u2", claim, u2)
