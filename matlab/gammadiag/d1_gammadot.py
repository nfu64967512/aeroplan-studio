#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D1 test A: is d(gam_deg)/dt a systematic UNDER-estimate of the true gamma rate?

Three estimators of the same quantity, on the same ticks:
  (1) local_slope(gam_deg, +-0.5 s)      <- what the gate used
  (2) central difference of gam_deg over +-h rows, h in {1,2,3,5}
  (3) gamma reconstructed from POSITION (lat/lon/z) then differenced
      -- fully independent of the gam_deg column, which comes from MAVLink
         vn/ve/vd rather than from the logged position.
"""
from __future__ import annotations
import math, os, statistics as st
import numpy as np
import d1_replicate as R

EARTH = 111320.0

def mlon(lat): return EARTH * math.cos(math.radians(lat))

def gamma_from_position(d, half_rows=2):
    """gamma from finite-differenced lat/lon/z.  Returns rad, NaN where unusable."""
    t, lat, lon, z = d["t"], d["lat"], d["lon"], d["z"]
    n = len(t); g = np.full(n, np.nan); vh = np.full(n, np.nan)
    for k in range(half_rows, n - half_rows):
        a, b = k - half_rows, k + half_rows
        dt = t[b] - t[a]
        if not (0.05 <= dt <= 0.6):      # skip freeze gaps + duplicate stamps
            continue
        dn = (lat[b] - lat[a]) * EARTH
        de = (lon[b] - lon[a]) * mlon(lat[k])
        dz = z[b] - z[a]
        vhk = math.hypot(dn, de) / dt
        if vhk < 1.0:
            continue
        vh[k] = vhk
        g[k] = math.atan2(dz / dt, vhk)
    return g, vh


def slope_nan(t, y, half=0.5):
    n = len(t); s = np.full(n, np.nan)
    for k in range(n):
        m = (np.abs(t - t[k]) <= half) & np.isfinite(y)
        if m.sum() < 3:
            continue
        tt = t[m] - t[k]
        A = np.column_stack([tt, np.ones(m.sum())])
        s[k] = np.linalg.lstsq(A, y[m], rcond=None)[0][0]
    return s


def central(t, y, h):
    n = len(t); s = np.full(n, np.nan)
    for k in range(h, n - h):
        dt = t[k + h] - t[k - h]
        if not (0.05 <= dt <= 0.6):
            continue
        s[k] = (y[k + h] - y[k - h]) / dt
    return s


def q(v, p): return float(np.nanpercentile(v, p))

if __name__ == "__main__":
    acc = {k: [] for k in ["ls", "c1", "c2", "c3", "c5", "pos_ls", "pos_c2",
                           "gam_csv", "gam_pos", "gs_csv", "vh_pos"]}
    per = []
    for tag in R.TAGS:
        d = R.load(tag)
        if len(d["t"]) < 60:
            continue
        t, gam = d["t"], d["gam"]
        ls, _ = R.local_slope(t, gam, 0.5)
        gp, vhp = gamma_from_position(d, 2)
        rec = dict(tag=tag)
        rows = dict(
            ls=ls,
            c1=central(t, gam, 1), c2=central(t, gam, 2),
            c3=central(t, gam, 3), c5=central(t, gam, 5),
            pos_ls=slope_nan(t, gp, 0.5), pos_c2=central(t, gp, 2),
            gam_csv=gam, gam_pos=gp, gs_csv=d["gs"], vh_pos=vhp)
        for k, v in rows.items():
            acc[k].append(np.asarray(v, float))
        # per-flight |rate| medians
        rec["ls"] = float(np.nanmedian(np.abs(ls)))
        rec["pos_ls"] = float(np.nanmedian(np.abs(rows["pos_ls"])))
        rec["c2"] = float(np.nanmedian(np.abs(rows["c2"])))
        per.append(rec)
    A = {k: np.concatenate(v) for k, v in acc.items()}
    deg = np.degrees

    print("=== gamma itself: CSV column vs position-derived (deg) ===")
    m = np.isfinite(A["gam_pos"])
    dg = deg(A["gam_csv"][m] - A["gam_pos"][m])
    print(f"  n={m.sum()}  gam_csv - gam_pos : median {np.median(dg):+.3f}  "
          f"p05 {np.percentile(dg,5):+.3f}  p95 {np.percentile(dg,95):+.3f}  "
          f"rms {np.sqrt(np.mean(dg**2)):.3f}")
    print(f"  corr(gam_csv, gam_pos) = "
          f"{np.corrcoef(A['gam_csv'][m], A['gam_pos'][m])[0,1]:+.5f}")
    mv = np.isfinite(A["vh_pos"])
    dv = A["gs_csv"][mv] - A["vh_pos"][mv]
    print(f"  gs_csv - |v_h| from position: median {np.median(dv):+.3f} m/s  "
          f"p05 {np.percentile(dv,5):+.2f} p95 {np.percentile(dv,95):+.2f}")

    print("\n=== |gamma-dot| distributions, deg/s (all ticks pooled) ===")
    print(f"{'estimator':<26s}{'n':>7s}{'med|.|':>9s}{'p90|.|':>9s}"
          f"{'p99|.|':>9s}{'max|.|':>9s}{'sd':>9s}")
    for k, lab in [("ls", "local_slope +-0.5s (GATE)"), ("c5", "central +-5 rows"),
                   ("c3", "central +-3 rows"), ("c2", "central +-2 rows"),
                   ("c1", "central +-1 row"),
                   ("pos_ls", "POSITION local_slope"), ("pos_c2", "POSITION +-2 rows")]:
        v = deg(A[k]); v = v[np.isfinite(v)]
        print(f"{lab:<26s}{len(v):>7d}{np.median(np.abs(v)):>9.3f}"
              f"{np.percentile(np.abs(v),90):>9.3f}{np.percentile(np.abs(v),99):>9.3f}"
              f"{np.max(np.abs(v)):>9.2f}{np.std(v):>9.3f}")

    print("\n=== pairwise regression of one estimator on another (slope, corr) ===")
    def reg(a, b):
        m = np.isfinite(a) & np.isfinite(b)
        x, y = a[m], b[m]
        s = float(np.polyfit(x, y, 1)[0])
        return s, float(np.corrcoef(x, y)[0, 1]), m.sum()
    for (ka, kb) in [("ls", "c2"), ("ls", "pos_ls"), ("c2", "pos_c2"),
                     ("pos_ls", "ls"), ("c2", "ls")]:
        s, c, n = reg(A[ka], A[kb])
        print(f"  y={kb:<8s} on x={ka:<8s}: slope {s:+.4f}  corr {c:+.4f}  n {n}")
