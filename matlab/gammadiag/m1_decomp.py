#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M1c: exact decomposition of the 0.063 gain + reproduction of the gate number
+ HALF (differencing window) sensitivity.

gain k = sum(dem*meas) / sum(dem^2).  Split the two sums by sub-population to
see which population *makes* the number.
"""
from __future__ import annotations
import math, os, statistics as st, sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..", "scripts", "research", "ditcg")))
import truth32  # noqa: E402


def run_once(half, span_lo, span_hi):
    os.environ["HALF"] = str(half)
    os.environ["SPAN_LO"] = str(span_lo)
    os.environ["SPAN_HI"] = str(span_hi)
    import importlib
    import m1_gamma_ceiling as M
    importlib.reload(M)
    acs = []
    for run in truth32.RUNS:
        tlat, tlon = truth32._read_target(run)
        for sysid, rows in sorted(truth32._rows(run).items()):
            o = M.per_ac(run, sysid, rows, tlat, tlon)
            if o:
                acs.append(o)
    return M, acs


def gain(dd, gm):
    sxx = sum(a * a for a in dd)
    sxy = sum(a * b for a, b in zip(dd, gm))
    return (sxy / sxx if sxx else float("nan")), sxx, sxy


def main():
    print("=== HALF (central-difference window) sensitivity, demand = post-LOS-floor")
    print("  HALF  span(s)      n  gain    corr    |meas|med  occ_med  occ>=0.9%")
    for half, lo, hi in ((2, 0.15, 0.45), (3, 0.15, 0.60), (5, 0.20, 0.80),
                         (10, 0.60, 1.40), (20, 1.40, 2.60)):
        M, acs = run_once(half, lo, hi)
        rs = [r for a in acs for r in a["recs"] if r["gdot_meas"] == r["gdot_meas"]]
        gm = [r["gdot_meas"] for r in rs]
        dd = [r["gdot_los"] for r in rs]
        k, _, _ = gain(dd, gm)
        try:
            cc = st.correlation(dd, gm)
        except Exception:
            cc = float("nan")
        occ = []
        for r in rs:
            c = r["cap_up"] if r["gdot_meas"] >= 0 else r["cap_dn"]
            occ.append(abs(r["gdot_meas"]) / max(1e-9, abs(c)))
        print("  %4d  %.2f-%.2f  %6d  %.3f  %+.3f  %8.2f  %8.3f  %8.1f"
              % (half, lo, hi, len(rs), k, cc,
                 math.degrees(st.median([abs(x) for x in gm])),
                 st.median(occ), sum(1 for x in occ if x >= 0.9) / len(rs) * 100))

    # ---- decomposition at the reference window --------------------------
    M, acs = run_once(5, 0.20, 0.80)
    rs = [r for a in acs for r in a["recs"] if r["gdot_meas"] == r["gdot_meas"]]
    gm = [r["gdot_meas"] for r in rs]
    dd = [r["gdot_los"] for r in rs]
    k_all, sxx_all, sxy_all = gain(dd, gm)
    print("\n=== decomposition of gain = S_xy / S_xx  (demand = post-LOS-floor, HALF=5)")
    print("  ALL          n=%6d  S_xx=%10.2f  S_xy=%9.2f  gain=%.4f"
          % (len(rs), sxx_all, sxy_all, k_all))

    def cap_of(r, key):
        return r["cap_up"] if r[key] >= 0 else r["cap_dn"]

    groups = {
        "demand > ceiling": [r for r in rs if abs(r["gdot_los"]) > abs(cap_of(r, "gdot_los"))],
        "demand <= ceiling": [r for r in rs if abs(r["gdot_los"]) <= abs(cap_of(r, "gdot_los"))],
    }
    for nm, g in groups.items():
        gm2 = [r["gdot_meas"] for r in g]
        dd2 = [r["gdot_los"] for r in g]
        k2, sxx2, sxy2 = gain(dd2, gm2)
        print("  %-18s n=%6d (%4.1f%%)  S_xx=%10.2f (%4.1f%% of total)  "
              "S_xy=%9.2f (%5.1f%%)  within-gain=%.4f"
              % (nm, len(g), len(g) / len(rs) * 100, sxx2, sxx2 / sxx_all * 100,
                 sxy2, sxy2 / sxy_all * 100, k2))

    print("\n=== counterfactual gains (same measured data, different demand definition)")
    for nm, key in (("PN raw (pre nz clamp)", "gdot_raw"),
                    ("PN after nz clamp", "gdot_nz"),
                    ("after LOS floor (what is sent)", "gdot_los")):
        d2 = [r[key] for r in rs]
        k2, _, _ = gain(d2, gm)
        try:
            cc = st.correlation(d2, gm)
        except Exception:
            cc = float("nan")
        print("  %-32s gain %.3f  corr %+.3f" % (nm, k2, cc))
    # demand truncated at the physical ceiling = "what a perfect aircraft could do"
    d3 = []
    for r in rs:
        c = cap_of(r, "gdot_los")
        d3.append(max(-abs(r["cap_dn"]), min(abs(r["cap_up"]), r["gdot_los"]))
                  if True else r["gdot_los"])
    k3, _, _ = gain(d3, gm)
    try:
        cc3 = st.correlation(d3, gm)
    except Exception:
        cc3 = float("nan")
    print("  %-32s gain %.3f  corr %+.3f"
          % ("post-LOS, TRUNCATED at ceiling", k3, cc3))

    # ---- how much gamma does the LOS step actually ask for? -------------
    print("\n=== size of the LOS-floor step (gam_cmd - gam), deg, when the floor binds")
    step = []
    for r in rs:
        gam_los = -math.atan2(r["h"], r["d"])
        cmd_nz = r["gam"] + r["gdot_nz"] * M.PN_LEAD
        if gam_los < cmd_nz - 1e-9:
            step.append(math.degrees(gam_los - r["gam"]))
    step.sort()

    def qq(v, p):
        return v[min(len(v) - 1, max(0, int(round(p * (len(v) - 1)))))]
    print("  n=%d  med %.2f  p10 %.2f  p90 %.2f  min %.2f"
          % (len(step), qq(step, .5), qq(step, .1), qq(step, .9), step[0]))
    print("  -> demanded over pn_lead=1.0 s  => that many deg/s")

    # ---- attainable gamma change in 1 s at the ceiling ------------------
    print("\n=== what the ceiling allows in the same 1.0 s window")
    cd = sorted(math.degrees(r["cap_dn"]) for r in rs)
    print("  push-over ceiling deg/s: med %.2f  p10 %.2f  p90 %.2f"
          % (qq(cd, .5), qq(cd, .1), qq(cd, .9)))


if __name__ == "__main__":
    main()
