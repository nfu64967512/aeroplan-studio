#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""R1 gates: the FOUR pre-registered pass/fail criteria, evaluated verbatim.

    (1) feasibility     : >= 95% of ticks have the NEW law's demanded n_z
                          inside the reachable envelope
                          (baseline: 35.5% of as-sent demands are outside)
    (2) phantom rate    : NO tick's implied gamma-dot demand exceeds the
                          physical ceiling by 2x  (baseline p90 4.94, p99 7.80)
    (3) actually use it : LATE-third demanded n_z median > 1.5
                          (baseline achieved 0.94, reachable ~3.0)
    (4) switch off      : bit-identical to the pre-B1 baseline, all 32 sorties

Nothing here is tuned after the fact.  Read-only.
"""
from __future__ import annotations

import math
import statistics as st

import r1_replay as RP

G = RP.G
NZ_MIN, NZ_MAX = RP.NZ_MIN, RP.NZ_MAX


def q(v, p):
    return RP.q(v, p)


def cap_of(gd, cg, V):
    return ((NZ_MAX if gd >= 0 else NZ_MIN) - cg) * G / V


def gdot_of(r):
    return (r["cmd_nz"] - math.cos(r["gam"])) * G / r["V"]


def ratio_of(r):
    gd = gdot_of(r)
    return abs(gd) / max(1e-9, abs(cap_of(gd, math.cos(r["gam"]), r["V"])))


def infeas(r):
    """demanded n_z outside the [pn_nz_min, nz_max_g] envelope."""
    return not (NZ_MIN - 1e-12 <= r["cmd_nz"] <= NZ_MAX + 1e-12)


def main():
    acs, out, aero = RP.main()
    n = len(out["OFF"])
    late = [k for k in range(n) if out["OFF"][k]["frac"] >= 2.0 / 3.0]
    meas = [k for k in range(n) if out["OFF"][k]["nz_meas"] == out["OFF"][k]["nz_meas"]]
    print("\nticks total %d   with measured nz (M1-comparable subset) %d   LATE third %d"
          % (n, len(meas), len(late)))

    # ── 0. does the OFF replay reproduce the published baseline numbers? ──
    print("\n" + "=" * 78)
    print("BASELINE REPRODUCTION (OFF config, M1-comparable subset)")
    print("=" * 78)
    sub = [out["OFF"][k] for k in meas]
    r_off = [ratio_of(r) for r in sub]
    print("  as-sent demand OUTSIDE envelope : %6.2f %%   (published 35.5 %%)"
          % (sum(1 for r in sub if infeas(r)) / len(sub) * 100))
    print("  implied gdot / ceiling           : med %.3f  p90 %.3f  p99 %.3f  max %.2f"
          % (q(r_off, .5), q(r_off, .9), q(r_off, .99), max(r_off)))
    print("                                     (published p90 4.94 / p99 7.80)")
    lm = [out["OFF"][k]["nz_meas"] for k in late
          if out["OFF"][k]["nz_meas"] == out["OFF"][k]["nz_meas"]]
    print("  LATE achieved n_z median         : %.3f   (published 0.94)"
          % st.median(lm))
    print("  implied gdot/ceiling > 1  (same test as the 'feasible' column)"
          " : %6.2f %%" % (sum(1 for x in r_off if x > 1.0) / len(sub) * 100))

    verdict = {}

    # ── GATE 4 : switch off == bit-identical ────────────────────────────
    print("\n" + "=" * 78)
    print("GATE (4)  switches off  ==>  BIT-IDENTICAL to pre-B1 baseline")
    print("=" * 78)
    bad = []
    for k in range(n):
        a, b = out["BASE"][k], out["OFF"][k]
        for f in ("roll", "pitch", "thr", "R", "V"):
            if a[f] != b[f]:
                bad.append((k, f, a[f], b[f]))
    print("  compared %d ticks x 5 fields (roll, pitch, thr, R, V)" % n)
    print("  mismatching ticks: %d" % len({x[0] for x in bad}))
    for x in bad[:5]:
        print("    tick %d %s  base=%r  off=%r" % x)
    verdict["4"] = (not bad)
    print("  ==> %s" % ("PASS" if not bad else "FAIL"))

    # ── GATE 1 : demand feasibility ─────────────────────────────────────
    print("\n" + "=" * 78)
    print("GATE (1)  NEW law demanded n_z inside reachable envelope >= 95 %")
    print("=" * 78)
    for cfg in ("OFF", "CLAMP", "CL", "NEW"):
        rs = out[cfg]
        ok = sum(1 for r in rs if not infeas(r))
        okm = sum(1 for k in meas if not infeas(rs[k]))
        print("  %-6s  inside envelope %6.2f %% (all ticks)   %6.2f %% (M1 subset)"
              % (cfg, ok / n * 100, okm / len(meas) * 100))
    frac1 = sum(1 for r in out["NEW"] if not infeas(r)) / n * 100
    verdict["1"] = frac1 >= 95.0
    print("  ==> NEW = %.2f %%   %s" % (frac1, "PASS" if verdict["1"] else "FAIL"))

    # also against the TRUE aero envelope (C_Lmax*q*S/W), for the record
    hard = sum(1 for r in out["NEW"]
               if r["cmd_nz"] > r["nz_max_true"] + 1e-9)
    nzmt = [r["nz_max_true"] for r in out["NEW"]]
    print("  (aero stall ceiling n_z_max_true: med %.1f g, p05 %.1f g;"
          " NEW demands above it: %d ticks)" % (q(nzmt, .5), q(nzmt, .05), hard))

    # ── GATE 2 : phantom gamma-dot demand ───────────────────────────────
    print("\n" + "=" * 78)
    print("GATE (2)  NO tick with implied gamma-dot demand > 2x ceiling")
    print("=" * 78)
    for cfg in ("OFF", "CLAMP", "CL", "NEW"):
        rr = [ratio_of(r) for r in out[cfg]]
        print("  %-6s  med %.3f  p90 %6.3f  p99 %6.3f  max %7.3f   frac>2x %5.2f %%"
              % (cfg, q(rr, .5), q(rr, .9), q(rr, .99), max(rr),
                 sum(1 for x in rr if x > 2.0) / n * 100))
    rn = [ratio_of(r) for r in out["NEW"]]
    verdict["2"] = max(rn) <= 2.0
    print("  ==> NEW max = %.4f   %s" % (max(rn), "PASS" if verdict["2"] else "FAIL"))

    # ── GATE 3 : late-dive demanded n_z median > 1.5 ────────────────────
    print("\n" + "=" * 78)
    print("GATE (3)  LATE third: demanded n_z median > 1.5")
    print("=" * 78)
    print("  %-6s %-10s %8s %8s %8s %8s" % ("cfg", "quantity", "med", "p75", "p90", "max"))
    for cfg in ("OFF", "NEW"):
        for key in ("req_nz", "cmd_nz"):
            v = [out[cfg][k][key] for k in late]
            print("  %-6s %-10s %8.3f %8.3f %8.3f %8.3f"
                  % (cfg, key, st.median(v), q(v, .75), q(v, .9), max(v)))
    v = [out["OFF"][k]["nz_meas"] for k in late
         if out["OFF"][k]["nz_meas"] == out["OFF"][k]["nz_meas"]]
    print("  %-6s %-10s %8.3f %8.3f %8.3f %8.3f"
          % ("meas", "nz_meas", st.median(v), q(v, .75), q(v, .9), max(v)))
    med3 = st.median([out["NEW"][k]["cmd_nz"] for k in late])
    verdict["3"] = med3 > 1.5
    print("  ==> NEW cmd_nz LATE median = %.3f   %s"
          % (med3, "PASS" if verdict["3"] else "FAIL"))

    # thirds, for the record
    print("\n  per-third demanded n_z median (NEW cmd_nz / NEW req_nz / achieved):")
    for lo, hi, nm in ((0, 1 / 3, "EARLY"), (1 / 3, 2 / 3, "MID"), (2 / 3, 1.01, "LATE")):
        ks = [k for k in range(n) if lo <= out["OFF"][k]["frac"] < hi]
        km = [k for k in ks if out["OFF"][k]["nz_meas"] == out["OFF"][k]["nz_meas"]]
        print("    %-6s n=%5d   cmd %+.3f   req %+.3f   achieved %+.3f"
              % (nm, len(ks),
                 st.median([out["NEW"][k]["cmd_nz"] for k in ks]),
                 st.median([out["NEW"][k]["req_nz"] for k in ks]),
                 st.median([out["OFF"][k]["nz_meas"] for k in km]) if km else float("nan")))

    print("\n" + "=" * 78)
    print("PRE-REGISTERED VERDICT")
    print("=" * 78)
    for k in sorted(verdict):
        print("  gate (%s) : %s" % (k, "PASS" if verdict[k] else "FAIL"))
    print("  overall (gates 1-4) : %s"
          % ("PASS" if all(verdict.values()) else "FAIL"))
    return acs, out, aero, late, meas


if __name__ == "__main__":
    main()
