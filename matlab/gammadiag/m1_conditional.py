#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M1b: the conditional test that actually decides (A) vs (B).

Splits every dive tick by whether the demand that reaches the attitude
command (post-LOS-floor) exceeds the nz ceiling, then asks separately in
each half: is the achieved gamma-dot at the ceiling, and what is the gain?

Also reports nz_meas, LOS-floor duty cycle, and HALF sensitivity.
"""
from __future__ import annotations
import math, os, statistics as st, sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import m1_gamma_ceiling as M  # noqa: E402
import truth32  # noqa: E402


def collect():
    acs = []
    for run in truth32.RUNS:
        tlat, tlon = truth32._read_target(run)
        for sysid, rows in sorted(truth32._rows(run).items()):
            o = M.per_ac(run, sysid, rows, tlat, tlon)
            if o:
                acs.append(o)
    return acs


def stats(name, rs):
    if not rs:
        print("  %-34s n=0" % name)
        return
    gm = [r["gdot_meas"] for r in rs]
    occ = []
    for r in rs:
        c = r["cap_up"] if r["gdot_meas"] >= 0 else r["cap_dn"]
        occ.append(abs(r["gdot_meas"]) / max(1e-9, abs(c)))
    los = [r["gdot_los"] for r in rs]
    sxy = sum(a * b for a, b in zip(los, gm))
    sxx = sum(a * a for a in los)
    kk = sxy / sxx if sxx > 0 else float("nan")
    try:
        cc = st.correlation(los, gm)
    except Exception:
        cc = float("nan")
    print("  %-34s n=%6d  occ_med %.3f  occ>=0.9 %5.1f%%  occ>=1.0 %5.1f%%  "
          "|meas|med %5.2f  gain %6.3f  corr %+.3f"
          % (name, len(rs), st.median(occ),
             sum(1 for x in occ if x >= 0.9) / len(rs) * 100,
             sum(1 for x in occ if x >= 1.0) / len(rs) * 100,
             math.degrees(st.median([abs(x) for x in gm])), kk, cc))


def main():
    acs = collect()
    allr = [r for a in acs for r in a["recs"] if r["gdot_meas"] == r["gdot_meas"]]
    N = len(allr)

    # --- LOS floor duty cycle -------------------------------------------
    los_active = []
    for r in allr:
        gam_los = -math.atan2(r["h"], r["d"])
        cmd_nz = r["gam"] + r["gdot_nz"] * M.PN_LEAD
        los_active.append(gam_los < cmd_nz - 1e-9)
    print("LOS floor active (gam_los is the binding command): %.1f%% of %d ticks"
          % (sum(los_active) / N * 100, N))

    def dem_over_cap(r, key):
        c = r["cap_up"] if r[key] >= 0 else r["cap_dn"]
        return abs(r[key]) / max(1e-9, abs(c))

    print("\n=== SPLIT BY: does the command that reaches the FC exceed the nz ceiling?")
    over = [r for r in allr if dem_over_cap(r, "gdot_los") > 1.0]
    under = [r for r in allr if dem_over_cap(r, "gdot_los") <= 1.0]
    print("  post-LOS demand > ceiling : %6d ticks (%.1f%%)" % (len(over), len(over) / N * 100))
    print("  post-LOS demand <= ceiling: %6d ticks (%.1f%%)" % (len(under), len(under) / N * 100))
    stats("  [demand OVER ceiling]", over)
    stats("  [demand UNDER ceiling]", under)

    print("\n=== SPLIT BY: does the PN-only demand (post-nz-clamp, pre-LOS) exceed ceiling?")
    o2 = [r for r in allr if dem_over_cap(r, "gdot_raw") > 1.0]
    u2 = [r for r in allr if dem_over_cap(r, "gdot_raw") <= 1.0]
    print("  pre-clamp demand > ceiling : %6d (%.1f%%)" % (len(o2), len(o2) / N * 100))
    stats("  [PN demand OVER ceiling]", o2)
    stats("  [PN demand UNDER ceiling]", u2)

    print("\n=== gain against POST-CLAMP demand (i.e. LOS floor removed from the demand)")
    for nm, rs in (("all", allr),
                   ("LOS floor inactive", [r for r, a in zip(allr, los_active) if not a]),
                   ("LOS floor active", [r for r, a in zip(allr, los_active) if a])):
        gm = [r["gdot_meas"] for r in rs]
        dd = [r["gdot_nz"] for r in rs]
        if not rs:
            continue
        sxy = sum(a * b for a, b in zip(dd, gm))
        sxx = sum(a * a for a in dd)
        try:
            cc = st.correlation(dd, gm)
        except Exception:
            cc = float("nan")
        print("  %-22s n=%6d  gain %.3f  corr %+.3f" % (nm, len(rs), sxy / sxx, cc))

    # --- achieved load factor ------------------------------------------
    nzm = sorted(r["nz_meas"] for r in allr)
    def qq(v, p):
        return v[min(len(v) - 1, max(0, int(round(p * (len(v) - 1)))))]
    print("\n=== achieved load factor nz_meas = cos(gam) + V*gdot_meas/g")
    print("  p01 %+.2f  p05 %+.2f  med %+.2f  p95 %+.2f  p99 %+.2f  max %+.2f  min %+.2f"
          % (qq(nzm, .01), qq(nzm, .05), qq(nzm, .50), qq(nzm, .95),
             qq(nzm, .99), nzm[-1], nzm[0]))
    print("  frac nz_meas > 3.0 : %.2f%%     frac nz_meas < -1.0 : %.2f%%"
          % (sum(1 for x in nzm if x > 3.0) / N * 100,
             sum(1 for x in nzm if x < -1.0) / N * 100))

    # --- where does the aircraft actually saturate? ---------------------
    print("\n=== ticks at >=90%% of ceiling, by direction")
    up = [r for r in allr if r["gdot_meas"] > 0
          and r["gdot_meas"] / r["cap_up"] >= 0.9]
    dn = [r for r in allr if r["gdot_meas"] < 0
          and r["gdot_meas"] / r["cap_dn"] >= 0.9]
    print("  pull-up  saturated: %5d (%.1f%%)   push-over saturated: %5d (%.1f%%)"
          % (len(up), len(up) / N * 100, len(dn), len(dn) / N * 100))
    if dn:
        print("  push-over saturated ticks: frac-of-dive med %.2f  gam med %.1f deg"
              % (st.median([r["frac"] for r in dn]),
                 math.degrees(st.median([r["gam"] for r in dn]))))
    if up:
        print("  pull-up  saturated ticks: frac-of-dive med %.2f  gam med %.1f deg"
              % (st.median([r["frac"] for r in up]),
                 math.degrees(st.median([r["gam"] for r in up]))))


if __name__ == "__main__":
    main()
