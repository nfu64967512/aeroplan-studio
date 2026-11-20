#!/usr/bin/env python3
"""M2 pass 3 -- conditional structure, plant identification, node 6/7.

Answers, on the PN-engaged window only:
  (a) how long is the pre-PN stretch that phase=='DIVE' wrongly includes
  (b) conditional on demand/ceiling, is the achieved rate saturating or lagging
  (c) node 6  : DesPitch -> Pitch     (attitude loop)
  (d) node 7  : Pitch    -> gamma     (alpha, i.e. the airframe)
  (e) measured n_z envelope with the touchdown spike excluded
"""
import csv, math, os
import numpy as np

CACHE = "/home/windows-wsl/gammadiag_cache"
REPO = "/mnt/c/Users/joy46/Documents/Projects/aeroplan-studio"
TRIALS = [f"RRD{i}" for i in range(1, 9)] + [f"RRE{i}" for i in range(1, 9)]
LEAD = 1.0


def q(x, p):
    x = np.asarray(x, float); x = x[np.isfinite(x)]
    return float(np.percentile(x, p)) if len(x) else float("nan")


def lagfit(x, y, dt, lo=0.0, hi=6.0):
    """best (lag, corr, slope) of y(t) against x(t-lag)."""
    best = (0.0, -9, float("nan"))
    for L in np.arange(lo, hi + 1e-9, 0.05):
        k = int(round(L / dt))
        if k >= len(x) - 60:
            break
        a = x[:len(x) - k] if k else x
        b = y[k:] if k else y
        ok = np.isfinite(a) & np.isfinite(b)
        if ok.sum() < 60 or a[ok].std() == 0 or b[ok].std() == 0:
            continue
        r = float(np.corrcoef(a[ok], b[ok])[0, 1])
        if r > best[1]:
            best = (L, r, float(np.polyfit(a[ok], b[ok], 1)[0]))
    return best


# --- bins of |demand| / ceiling, pooled over all 32 sorties
BINS = [(0, .25), (.25, .5), (.5, 1.0), (1.0, 2.0), (2.0, 99)]
pool = {b: {"dem": [], "act": [], "sat": []} for b in BINS}
rows = []

for tr in TRIALS:
    for s in (1, 2):
        zp = np.load(f"{CACHE}/phase_{tr}_U{s}.npz")
        z = np.load(f"{CACHE}/dump_{tr}_U{s}.npz")
        pre_s = float(z["tw"][0] - zp["tw"][0])          # pre-PN stretch [R]

        tl = z["tl"]
        dt = float(np.median(np.diff(tl)))
        gam_t = z["gamr"]
        dem = (z["gam_cmd"] - gam_t) / LEAD
        act = np.interp(tl, z["tA"], z["gamdot"])
        cu = np.interp(tl, z["tA"], z["ceil_up"])
        cd = np.interp(tl, z["tA"], z["ceil_dn"])
        ratio = np.where(dem >= 0, dem / np.maximum(cu, 1e-6), dem / np.minimum(cd, -1e-6))
        satfrac = (act > 0.8 * cu) | (act < 0.8 * cd)
        for b in BINS:
            m = (np.abs(ratio) >= b[0]) & (np.abs(ratio) < b[1]) & np.isfinite(act)
            if m.sum():
                pool[b]["dem"].append(np.abs(dem[m]))
                pool[b]["act"].append(np.abs(act[m]))
                pool[b]["sat"].append(satfrac[m])

        # -- node 6: commanded pitch -> achieved pitch (both MEASURED) -------
        tA = z["tA"]
        dtA = float(np.median(np.diff(tA)))
        L6, r6, g6 = lagfit(z["despitch_fw"], z["pitch_m"], dtA)
        e6 = z["pitch_m"] - z["despitch_fw"]
        k6 = int(round(L6 / dtA))
        e6L = (z["pitch_m"][k6:] - z["despitch_fw"][:len(z["despitch_fw"]) - k6]) if k6 else e6

        # -- node 7: achieved pitch -> gamma (alpha is the whole node) -------
        alpha = z["alpha_true"]
        L7, r7, g7 = lagfit(z["pitch_m"], z["gam"], dtA)

        # -- n_z envelope, last 1.0 s dropped (touchdown spike) --------------
        nz = z["nz_meas"]
        keep = tA <= tA[-1] - 1.0
        nzk = nz[keep]

        rows.append(dict(
            trial=tr, sysid=s,
            pre_pn_s=pre_s, pn_s=float(z["tw"][-1] - z["tw"][0]),
            n6_lag=L6, n6_corr=r6, n6_gain=g6,
            n6_bias=q(e6, 50), n6_absmed=q(np.abs(e6), 50),
            n6_absmedL=q(np.abs(e6L), 50), n6_absp90L=q(np.abs(e6L), 90),
            n7_lag=L7, n7_corr=r7, n7_gain=g7,
            alpha_med=q(alpha, 50), alpha_p05=q(alpha, 5), alpha_p95=q(alpha, 95),
            aoa_fw_med=q(z["aoa_fw"], 50),
            nz_p01=q(nzk, 1), nz_p50=q(nzk, 50), nz_p99=q(nzk, 99),
            nz_min=float(np.nanmin(nzk)), nz_max=float(np.nanmax(nzk)),
            nz_below_m1=float(np.mean(nzk < -1.0)),
            nz_above_3=float(np.mean(nzk > 3.0)),
        ))

with open(f"{REPO}/matlab/gammadiag/pass3.csv", "w", newline="") as f:
    w = csv.DictWriter(f, list(rows[0])); w.writeheader()
    for r in rows: w.writerow(r)

import statistics as st
def M(k): return st.median([r[k] for r in rows])

print("=" * 76)
print("(a) how much of phase=='DIVE' is NOT under PN")
print("=" * 76)
print(f"  pre-PN stretch inside phase==DIVE : median {M('pre_pn_s'):.1f} s"
      f"   (min {min(r['pre_pn_s'] for r in rows):.1f}, max {max(r['pre_pn_s'] for r in rows):.1f})")
print(f"  PN-engaged stretch                : median {M('pn_s'):.1f} s")
print(f"  => fraction of the DIVE phase with no PN command at all: "
      f"{M('pre_pn_s')/(M('pre_pn_s')+M('pn_s')):.1%}")
print()
print("=" * 76)
print("(b) conditional on |demand|/ceiling -- saturation or lag?  (pooled ticks)")
print("=" * 76)
print(f"{'|dem|/ceiling':>16s} {'ticks':>8s} {'%ticks':>7s} {'med|dem|':>9s} "
      f"{'med|act|':>9s} {'act/dem':>8s} {'|act|>0.8ceil':>14s}")
tot = sum(sum(len(a) for a in pool[b]['dem']) for b in BINS)
for b in BINS:
    d = np.concatenate(pool[b]["dem"]) if pool[b]["dem"] else np.array([])
    a = np.concatenate(pool[b]["act"]) if pool[b]["act"] else np.array([])
    sfr = np.concatenate(pool[b]["sat"]) if pool[b]["sat"] else np.array([])
    if not len(d): continue
    lbl = f"{b[0]:.2f}-{b[1]:.2f}" if b[1] < 90 else f">{b[0]:.2f}"
    print(f"{lbl:>16s} {len(d):8d} {100*len(d)/tot:6.1f}% {np.median(d):9.2f} "
          f"{np.median(a):9.2f} {np.median(a)/np.median(d):8.3f} {sfr.mean():13.1%}")
print()
print("=" * 76)
print("(c) node 6  commanded pitch (ATT.DesPitch) -> achieved pitch (ATT.Pitch)")
print("=" * 76)
print(f"  best lag        : {M('n6_lag'):.2f} s      corr {M('n6_corr'):.3f}   gain {M('n6_gain'):.3f}")
print(f"  |error| median  : {M('n6_absmed'):.2f} deg   (lag-removed {M('n6_absmedL'):.2f} deg,"
      f" p90 {M('n6_absp90L'):.2f} deg)")
print(f"  signed bias     : {M('n6_bias'):+.2f} deg")
print()
print("=" * 76)
print("(d) node 7  achieved pitch -> gamma   (this node IS alpha)")
print("=" * 76)
print(f"  best lag {M('n7_lag'):.2f} s   corr {M('n7_corr'):.3f}   gain {M('n7_gain'):.3f}")
print(f"  alpha_true = pitch - gamma : median {M('alpha_med'):+.2f} deg "
      f"[p05 {M('alpha_p05'):+.2f}, p95 {M('alpha_p95'):+.2f}]")
print(f"  firmware AOA message       : median {M('aoa_fw_med'):+.2f} deg")
print()
print("=" * 76)
print("(e) MEASURED n_z envelope (-IMU.AccZ/g), last 1 s dropped")
print("=" * 76)
print(f"  p01 {M('nz_p01'):+.2f}   p50 {M('nz_p50'):+.2f}   p99 {M('nz_p99'):+.2f}"
      f"   min {M('nz_min'):+.2f}   max {M('nz_max'):+.2f}")
print(f"  fraction of ticks below the model's nz_min = -1.0 : {M('nz_below_m1'):.1%}")
print(f"  fraction of ticks above the model's nz_max = +3.0 : {M('nz_above_3'):.1%}")
