#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D1: byte-faithful Python port of matlab/stot_lagcheck.m (measured half only).

Purpose: reproduce 0.063 / +0.217 / 8.00 s / 7.43 deg from the SAME inputs the
MATLAB gate used, so that every subsequent perturbation is a controlled change
of ONE thing.  Nothing in the repo is modified.
"""
from __future__ import annotations
import csv, glob, math, os, statistics as st
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TRAJ = os.path.abspath(os.path.join(HERE, "..", "data", "traj"))

PN = dict(n_vert=5.0, lead=1.0, nz_min=-1.0, nz_max=3.0, los_floor=1.0, g=9.80665)


def load(tag):
    with open(os.path.join(TRAJ, tag + ".csv"), encoding="utf-8") as f:
        rows = [r for r in csv.DictReader(f) if r["post_touch"] == "0"]
    g = lambda k: np.array([float(r[k]) for r in rows])
    return dict(t=g("t_dq"), t_raw=g("t"), r=g("r"), z=g("z"),
                gam=np.radians(g("gamma_deg")), gs=g("gs"),
                lat=g("lat"), lon=g("lon"))


def local_slope(t, y, half=0.5):
    """Exact port of stot_lagcheck.m::local_slope, INCLUDING the s(k)=0 fill."""
    n = len(t); s = np.zeros(n); nz = 0
    for k in range(n):
        m = np.abs(t - t[k]) <= half
        if m.sum() < 3:
            s[k] = 0.0; nz += 1; continue
        tt = t[m] - t[k]
        A = np.column_stack([tt, np.ones(m.sum())])
        s[k] = np.linalg.lstsq(A, y[m], rcond=None)[0][0]
    return s, nz


def replay_gamcmd(t, r, z, gam, gs, pn=PN):
    hdot = gs * np.tan(gam)
    V = np.maximum(1e-3, np.hypot(gs, hdot))
    ddot, _ = local_slope(t, r, 0.5)
    d = np.maximum(1e-3, r); h = z
    R = np.maximum(1e-3, np.hypot(d, h))
    sig_dot = (hdot * d - h * ddot) / (R * R)
    gdot = -pn["n_vert"] * sig_dot
    nz_raw = np.cos(gam) + V * gdot / pn["g"]
    nz = np.clip(nz_raw, pn["nz_min"], pn["nz_max"])
    gdot = (nz - np.cos(gam)) * pn["g"] / V
    gc = gam + gdot * pn["lead"]
    if pn["los_floor"] > 0:
        gc = np.minimum(gc, -np.arctan2(h, np.maximum(d, 1e-3)) * pn["los_floor"])
    return gc, nz_raw, V


def rate_gain(t, gam, gcmd, lead=1.0):
    """Exact port, including unique(t,'stable') row dropping."""
    _, iu = np.unique(t, return_index=True)
    iu = np.sort(iu)                      # 'stable' == first occurrence, in order
    tu, gamu, gcu = t[iu], gam[iu], gcmd[iu]
    ndrop = len(t) - len(tu)
    if len(tu) < 30:
        return None
    gd_meas, nzero = local_slope(tu, gamu, 0.5)
    gd_cmd = (gcu - gamu) / lead
    x, y = gd_cmd, gd_meas
    gain = float(x @ y / max(x @ x, 1e-12))
    xc, yc = x - x.mean(), y - y.mean()
    cc = float(xc @ yc / max(math.sqrt((xc @ xc) * (yc @ yc)), 1e-12))
    return dict(gain=gain, corr=cc, x=x, y=y, t=tu, gam=gamu, gcmd=gcu,
                ndrop=ndrop, nzero=nzero, n=len(tu))


def best_lag(tc, gcmd, tg, gam, lags):
    ok = np.isfinite(gcmd); tk, gk = tc[ok], gcmd[ok]
    _, iu = np.unique(tg, return_index=True); iu = np.sort(iu)
    tgu, gamu = tg[iu], gam[iu]
    o = np.argsort(tgu); tgu, gamu = tgu[o], gamu[o]
    out = []
    for L in lags:
        gi = np.interp(tk + L, tgu, gamu, left=np.nan, right=np.nan)
        m = np.isfinite(gi)
        if m.sum() < 20:
            out.append(np.nan); continue
        out.append(float(np.sqrt(np.mean((gi[m] - gk[m]) ** 2))))
    out = np.array(out)
    if np.all(np.isnan(out)):
        return np.nan, np.nan, out
    i = int(np.nanargmin(out))
    return float(lags[i]), float(out[i]), out


TAGS = sorted(os.path.basename(p)[:-4] for p in glob.glob(os.path.join(TRAJ, "*.csv")))

if __name__ == "__main__":
    lags = np.arange(0, 8.0 + 1e-9, 0.05)
    lags20 = np.arange(0, 20.0 + 1e-9, 0.05)
    G, C, L8, L20, RMS, ND, NZ, N = [], [], [], [], [], [], [], []
    for tag in TAGS:
        d = load(tag)
        if len(d["t"]) < 30:
            continue
        gc, _, _ = replay_gamcmd(d["t"], d["r"], d["z"], d["gam"], d["gs"])
        rg = rate_gain(d["t"], d["gam"], gc)
        l8, r8, _ = best_lag(d["t"], gc, d["t"], d["gam"], lags)
        l20, _, _ = best_lag(d["t"], gc, d["t"], d["gam"], lags20)
        G.append(rg["gain"]); C.append(rg["corr"]); L8.append(l8); L20.append(l20)
        RMS.append(math.degrees(r8)); ND.append(rg["ndrop"]); NZ.append(rg["nzero"])
        N.append(rg["n"])
        print(f"{tag:<10s} gain {rg['gain']:+.4f}  corr {rg['corr']:+.4f}  "
              f"lag8 {l8:5.2f}  lag20 {l20:5.2f}  rms {math.degrees(r8):6.2f}  "
              f"n {rg['n']:5d} drop {rg['ndrop']:4d} zerofill {rg['nzero']:4d}")
    print("\n=== D1 replication of stot_lagcheck (measured half) ===")
    print(f"n flights            {len(G)}")
    print(f"rate_gain  median    {st.median(G):+.4f}   (gate reported 0.063)")
    print(f"   IQR               {np.percentile(G,25):+.4f} .. {np.percentile(G,75):+.4f}")
    print(f"rate_corr  median    {st.median(C):+.4f}   (gate reported +0.217)")
    print(f"best lag (max 8)     median {st.median(L8):.2f}   at ceiling "
          f"{sum(1 for v in L8 if v >= 8.0-1e-9)}/{len(L8)}   (gate: 8.00, 26/32)")
    print(f"best lag (max 20)    median {st.median(L20):.2f}  at ceiling "
          f"{sum(1 for v in L20 if v >= 20.0-1e-9)}/{len(L20)}")
    print(f"residual rms median  {st.median(RMS):.2f} deg   (gate reported 7.43)")
    print(f"rows dropped by unique(): total {sum(ND)}  median/flight {st.median(ND):.0f}")
    print(f"local_slope zero-fill ticks: total {sum(NZ)}  median/flight {st.median(NZ):.0f}")
