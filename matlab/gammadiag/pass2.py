#!/usr/bin/env python3
"""M2 pass 2 -- decompose the 0.063 tracking gain and test the demand side.

Runs in WSL against the per-sortie dumps written by chain.py.

Everything here is on the node's own 20 Hz tick grid, expressed in sim time.
gamma-dot demanded is (gam_cmd - gamma)/pn_lead, matching how the node itself
turns a rate into an angle command.
"""
import csv, math
import numpy as np

import os
CACHE = "/home/windows-wsl/gammadiag_cache"
PFX = os.environ.get("M2_PFX", "dump")
REPO = "/mnt/c/Users/joy46/Documents/Projects/aeroplan-studio"
TRIALS = [f"RRD{i}" for i in range(1, 9)] + [f"RRE{i}" for i in range(1, 9)]
G = 9.80665
LEAD = 1.0


def q(x, p):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(np.percentile(x, p)) if len(x) else float("nan")


def reg(x, y):
    ok = np.isfinite(x) & np.isfinite(y)
    x, y = x[ok], y[ok]
    if len(x) < 30 or x.std() == 0 or y.std() == 0:
        return float("nan"), float("nan"), float("nan")
    r = float(np.corrcoef(x, y)[0, 1])
    slope = float(np.polyfit(x, y, 1)[0])
    return slope, r, float(y.std() / x.std())


out = []
for tr in TRIALS:
    for s in (1, 2):
        z = np.load(f"{CACHE}/{PFX}_{tr}_U{s}.npz")
        tw, tl = z["tw"], z["tl"]
        gam_t = z["gamr"]                      # gamma on the tick grid  [M]
        cmd_pre = z["gam_cmd_pn"]              # after node nz clamp     [R]
        cmd = z["gam_cmd"]                     # after LOS floor         [R]
        # achieved gamma-dot resampled onto the tick grid  [M]
        act = np.interp(tl, z["tA"], z["gamdot"])
        V = np.interp(tl, z["tA"], z["V"])
        ceil_up = np.interp(tl, z["tA"], z["ceil_up"])
        ceil_dn = np.interp(tl, z["tA"], z["ceil_dn"])
        dem = (cmd - gam_t) / LEAD             # deg/s demanded (post LOS)
        dem_pre = (cmd_pre - gam_t) / LEAD     # deg/s demanded (pre LOS)

        # -- lag sweep: shift the demand forward in time and re-correlate ----
        dt = float(np.median(np.diff(tl)))
        best = (None, -9)
        curve = []
        for lag_s in np.arange(0.0, 10.01, 0.1):
            k = int(round(lag_s / dt))
            if k >= len(dem) - 60:
                break
            a = dem[:len(dem) - k] if k else dem
            b = act[k:] if k else act
            _, r, _ = reg(a, b)
            curve.append((lag_s, r))
            if r > best[1]:
                best = (lag_s, r)
        lag_best, r_best = best
        kb = int(round(lag_best / dt))
        sl_b, r_b, sd_b = reg(dem[:len(dem) - kb] if kb else dem, act[kb:] if kb else act)
        sl0, r0, sd0 = reg(dem, act)
        sl0p, r0p, sd0p = reg(dem_pre, act)

        # -- angle-domain tracking: does gamma reach the commanded angle? ----
        ang_err0 = gam_t - cmd
        ang_errL = (gam_t[kb:] - cmd[:len(cmd) - kb]) if kb else ang_err0

        # -- demand vs the physical ceiling ----------------------------------
        over = (dem > ceil_up) | (dem < ceil_dn)
        over_pre = (dem_pre > ceil_up) | (dem_pre < ceil_dn)
        ratio = np.where(dem >= 0, dem / np.maximum(ceil_up, 1e-6),
                         dem / np.minimum(ceil_dn, -1e-6))

        # -- node stalls: ticks where the node produced nothing --------------
        gaps = np.diff(tw)
        stall = float(np.sum(gaps[gaps > 0.3]))

        out.append(dict(
            trial=tr, sysid=s, n=len(tw),
            gain0=sl0, corr0=r0, sdratio0=sd0,
            gain0_preLOS=sl0p, corr0_preLOS=r0p,
            lag_best=lag_best, corr_best=r_b, gain_best=sl_b, sdratio_best=sd_b,
            dem_absmed=q(np.abs(dem), 50), dem_p90=q(np.abs(dem), 90),
            dem_p99=q(np.abs(dem), 99), dem_max=float(np.nanmax(np.abs(dem))),
            act_absmed=q(np.abs(act), 50), act_p90=q(np.abs(act), 90),
            frac_dem_over_ceil=float(np.mean(over)),
            frac_dempre_over_ceil=float(np.mean(over_pre)),
            dem_over_ceil_ratio_p90=q(np.abs(ratio), 90),
            dem_over_ceil_ratio_p99=q(np.abs(ratio), 99),
            dem_over_ceil_ratio_max=float(np.nanmax(np.abs(ratio))),
            ang_err_med=q(np.abs(ang_err0), 50), ang_err_p90=q(np.abs(ang_err0), 90),
            ang_errL_med=q(np.abs(ang_errL), 50), ang_errL_p90=q(np.abs(ang_errL), 90),
            stall_s=stall, dur_s=float(tw[-1] - tw[0]),
            V_med=q(V, 50),
        ))

keys = list(out[0])
with open(f"{REPO}/matlab/gammadiag/pass2_{PFX}.csv", "w", newline="") as f:
    w = csv.DictWriter(f, keys)
    w.writeheader()
    for r in out:
        w.writerow(r)
print("wrote", len(out))
