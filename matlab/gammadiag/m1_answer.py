#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M1e: the final cross-tab that answers "A or B", per third of the dive.

Rows are classified on two independent axes:
  FEASIBLE  : is the command the FC receives (post-LOS-floor) inside the
              nz-clamp ceiling gdot_max = (nz_lim - cos gam)*g/V ?
  SATURATED : is the ACHIEVED gdot at >= 90% of that same ceiling ?

(A) = SATURATED           -> airframe is doing all it can
(B) = not SATURATED and demand > achieved  -> tracking deficit
(C) = demand <= achieved  -> nobody is asking

Writes m1_ticks.csv (one row per usable tick) so every number can be recomputed.
"""
from __future__ import annotations
import csv, math, os, statistics as st, sys

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
                o["_run"] = run
                o["_sid"] = sysid
                acs.append(o)

    out = os.path.join(HERE, "m1_ticks.csv")
    with open(out, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["tag", "t", "frac", "gam_deg", "V_ms", "h_m", "d_m",
                    "gdot_meas_dps", "gdot_raw_dps", "gdot_nz_dps", "gdot_los_dps",
                    "cap_up_dps", "cap_dn_dps", "nz_raw", "nz_meas", "occ",
                    "feasible", "saturated", "cls"])
        rows_all = []
        for a in acs:
            for r in a["recs"]:
                if r["gdot_meas"] != r["gdot_meas"]:
                    continue
                cap_m = r["cap_up"] if r["gdot_meas"] >= 0 else r["cap_dn"]
                cap_d = r["cap_up"] if r["gdot_los"] >= 0 else r["cap_dn"]
                occ = abs(r["gdot_meas"]) / abs(cap_m)
                feas = abs(r["gdot_los"]) <= abs(cap_d)
                sat = occ >= 0.90
                cls = "A" if sat else ("B" if abs(r["gdot_raw"]) > abs(r["gdot_meas"]) else "C")
                rec = dict(r, tag=a["tag"], occ=occ, feasible=feas, sat=sat, cls=cls)
                rows_all.append(rec)
                w.writerow([a["tag"], "%.2f" % r["t"], "%.4f" % r["frac"],
                            "%.3f" % math.degrees(r["gam"]), "%.2f" % r["V"],
                            "%.1f" % r["h"], "%.1f" % r["d"],
                            "%.4f" % math.degrees(r["gdot_meas"]),
                            "%.4f" % math.degrees(r["gdot_raw"]),
                            "%.4f" % math.degrees(r["gdot_nz"]),
                            "%.4f" % math.degrees(r["gdot_los"]),
                            "%.4f" % math.degrees(r["cap_up"]),
                            "%.4f" % math.degrees(r["cap_dn"]),
                            "%.4f" % r["nz_raw"], "%.4f" % r["nz_meas"],
                            "%.4f" % occ, int(feas), int(sat), cls])
    print("wrote %s  (%d ticks)" % (out, len(rows_all)))

    def tab(name, rs):
        n = len(rs)
        if not n:
            return
        ff = [r for r in rs if r["feasible"]]
        ii = [r for r in rs if not r["feasible"]]
        print("\n== %s   n=%d" % (name, n))
        print("   demand FEASIBLE  : %5.1f%% of ticks   of those, ACHIEVED>=90%%ceiling: %5.1f%%   occ med %.3f"
              % (len(ff) / n * 100,
                 (sum(1 for r in ff if r["sat"]) / len(ff) * 100) if ff else 0.0,
                 st.median([r["occ"] for r in ff]) if ff else float("nan")))
        print("   demand INFEASIBLE: %5.1f%% of ticks   of those, ACHIEVED>=90%%ceiling: %5.1f%%   occ med %.3f"
              % (len(ii) / n * 100,
                 (sum(1 for r in ii if r["sat"]) / len(ii) * 100) if ii else 0.0,
                 st.median([r["occ"] for r in ii]) if ii else float("nan")))
        cA = sum(1 for r in rs if r["cls"] == "A")
        cB = sum(1 for r in rs if r["cls"] == "B")
        cC = sum(1 for r in rs if r["cls"] == "C")
        print("   A %5.1f%%   B %5.1f%%   C %5.1f%%" % (cA / n * 100, cB / n * 100, cC / n * 100))
        nzm = [r["nz_meas"] for r in rs]
        print("   nz achieved  p01 %+.2f  med %+.2f  p99 %+.2f  max %+.2f | >3.0 %.2f%%  <-1.0 %.2f%%"
              % (qq(nzm, .01), qq(nzm, .5), qq(nzm, .99), max(nzm),
                 sum(1 for x in nzm if x > 3) / n * 100,
                 sum(1 for x in nzm if x < -1) / n * 100))
        dr = [abs(r["gdot_raw"]) / abs(r["cap_up"] if r["gdot_raw"] >= 0 else r["cap_dn"]) for r in rs]
        dl = [abs(r["gdot_los"]) / abs(r["cap_up"] if r["gdot_los"] >= 0 else r["cap_dn"]) for r in rs]
        print("   demand/ceiling  PN-only med %.2f p90 %.2f | as-sent(post-LOS) med %.2f p90 %.2f"
              % (qq(dr, .5), qq(dr, .9), qq(dl, .5), qq(dl, .9)))

    tab("ALL", rows_all)
    for lo, hi, nm in ((0.0, 1 / 3, "EARLY 0-33%"), (1 / 3, 2 / 3, "MID 33-67%"),
                       (2 / 3, 1.01, "LATE 67-100%")):
        tab(nm, [r for r in rows_all if lo <= r["frac"] < hi])

    print("\n== demand gdot distribution (deg/s, |.|) as the task asks in item 4")
    for nm, key in (("PN raw (pre nz clamp)", "gdot_raw"),
                    ("PN after nz clamp", "gdot_nz"),
                    ("as sent (post LOS floor)", "gdot_los")):
        v = [abs(math.degrees(r[key])) for r in rows_all]
        print("   %-26s med %7.2f  p90 %8.2f  max %10.2f"
              % (nm, qq(v, .5), qq(v, .9), max(v)))
    cap = [abs(math.degrees(r["cap_dn"])) for r in rows_all]
    print("   push-over ceiling |.|      med %7.2f  p10 %8.2f" % (qq(cap, .5), qq(cap, .1)))
    for nm, key in (("PN raw", "gdot_raw"), ("as sent", "gdot_los")):
        v = [abs(r[key]) / abs(r["cap_up"] if r[key] >= 0 else r["cap_dn"]) for r in rows_all]
        print("   N = demand/ceiling  %-10s med %.2f  p90 %.2f  p99 %.2f"
              % (nm, qq(v, .5), qq(v, .9), qq(v, .99)))


if __name__ == "__main__":
    main()
