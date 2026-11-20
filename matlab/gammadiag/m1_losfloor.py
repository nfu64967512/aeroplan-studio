#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M1d: is the "demand" a real rate demand, or an angle command divided by pn_lead?

The LOS floor sets gam_cmd = gam_los (an ANGLE). Dividing (gam_cmd - gam) by
pn_lead = 1.0 s manufactures a rate demand of up to -61 deg/s. Test: does gamma
converge onto gam_los, and how long does it take?
"""
from __future__ import annotations
import math, os, statistics as st, sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..",
                                                "scripts", "research", "ditcg")))
import truth32  # noqa: E402
import m1_gamma_ceiling as M  # noqa: E402


def qq(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, max(0, int(round(p * (len(v) - 1)))))]


def main():
    acs = []
    for run in truth32.RUNS:
        tlat, tlon = truth32._read_target(run)
        for sysid, rows in sorted(truth32._rows(run).items()):
            o = M.per_ac(run, sysid, rows, tlat, tlon)
            if o:
                acs.append(o)

    print("=== gamma vs LOS angle over the dive (deg).  err = gam - gam_los")
    print("    (err < 0 = steeper than LOS = floor NOT binding)")
    print("  seg        n     err_med   err_p10   err_p90   |err|<2deg%")
    for lo, hi, nm in ((0.0, 1 / 3, "EARLY"), (1 / 3, 2 / 3, "MID"),
                       (2 / 3, 1.01, "LATE"), (0.0, 1.01, "ALL")):
        e = []
        for a in acs:
            for r in a["recs"]:
                if lo <= r["frac"] < hi:
                    e.append(math.degrees(r["gam"] + math.atan2(r["h"], r["d"])))
        print("  %-6s %6d  %+8.2f  %+8.2f  %+8.2f  %8.1f"
              % (nm, len(e), qq(e, .5), qq(e, .1), qq(e, .9),
                 sum(1 for x in e if abs(x) < 2.0) / len(e) * 100))

    print("\n=== convergence: time from release until |gam - gam_los| first < 2 deg")
    tc, never = [], 0
    for a in acs:
        t0 = a["recs"][0]["t"]
        hit = None
        for r in a["recs"]:
            if abs(math.degrees(r["gam"] + math.atan2(r["h"], r["d"]))) < 2.0:
                hit = r["t"] - t0
                break
        if hit is None:
            never += 1
        else:
            tc.append(hit)
    print("  converged %d/%d  med %.1f s  p90 %.1f s  max %.1f s  (dive T med %.1f s)"
          % (len(tc), len(acs), qq(tc, .5), qq(tc, .9), max(tc),
             st.median([a["t_dive"] for a in acs])))

    print("\n=== fraction of dive spent with the LOS floor binding, per aircraft")
    fr = []
    for a in acs:
        n = b = 0
        for r in a["recs"]:
            gam_los = -math.atan2(r["h"], r["d"])
            cmd_nz = r["gam"] + r["gdot_nz"] * M.PN_LEAD
            n += 1
            if gam_los < cmd_nz - 1e-9:
                b += 1
        fr.append(b / max(1, n))
    print("  med %.2f  p10 %.2f  p90 %.2f" % (qq(fr, .5), qq(fr, .1), qq(fr, .9)))

    print("\n=== the same test done properly: treat gam_cmd as an ANGLE command.")
    print("    tracking error gam_cmd - gam (deg) and how fast it shrinks")
    allr = [r for a in acs for r in a["recs"] if r["gdot_meas"] == r["gdot_meas"]]
    err = []
    for r in allr:
        gam_los = -math.atan2(r["h"], r["d"])
        cmd = min(r["gam"] + r["gdot_nz"] * M.PN_LEAD, gam_los)
        err.append(math.degrees(cmd - r["gam"]))
    print("  angle error: med %+.2f  p10 %+.2f  p90 %+.2f" % (qq(err, .5), qq(err, .1), qq(err, .9)))
    # first-order closure rate: gdot_meas vs angle error -> implied time constant
    e_rad = [math.radians(x) for x in err]
    gm = [r["gdot_meas"] for r in allr]
    sxx = sum(x * x for x in e_rad)
    sxy = sum(x * y for x, y in zip(e_rad, gm))
    k = sxy / sxx
    try:
        cc = st.correlation(e_rad, gm)
    except Exception:
        cc = float("nan")
    print("  gdot_meas = k * (gam_cmd - gam):  k = %.4f 1/s  => tau = %.2f s   corr %+.3f"
          % (k, 1.0 / k if k else float("inf"), cc))
    # restrict to ticks where the ceiling is not binding
    sub = [(e, g, r) for e, g, r in zip(e_rad, gm, allr)
           if abs(e) / M.PN_LEAD <= abs(r["cap_dn"] if e < 0 else r["cap_up"])]
    if sub:
        ee = [x[0] for x in sub]
        gg = [x[1] for x in sub]
        k2 = sum(a * b for a, b in zip(ee, gg)) / sum(a * a for a in ee)
        try:
            c2 = st.correlation(ee, gg)
        except Exception:
            c2 = float("nan")
        print("  same, ceiling-feasible ticks only (n=%d): k = %.4f 1/s => tau = %.2f s  corr %+.3f"
              % (len(sub), k2, 1.0 / k2, c2))


if __name__ == "__main__":
    main()
