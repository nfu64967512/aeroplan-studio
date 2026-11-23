# -*- coding: utf-8 -*-
"""Validation: does the replay reproduce the values the NODE ITSELF logged?"""
from __future__ import annotations
import math, sys, pathlib, statistics as st
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import b1_full as F, b1_band as B
D = math.degrees
acs = F.load()

print("=== replay validity: replayed vs LOGGED req_nz / cmd_nz, post-handover, NZC only")
print("   (RRE has no such columns; NZC logs them from the real handover onward)")
rows = []
for a in acs:
    if not a["run"].startswith("NZC"):
        continue
    dr, dc, n = [], [], 0
    for r in a["pn_flown"]:
        raw = a["seg"][r["k"]]
        lr, lc = B._f(raw, "req_nz"), B._f(raw, "cmd_nz")
        if lr != lr or lc != lc:
            continue
        n += 1
        dr.append(r["req_nz"] - lr)
        dc.append(r["cmd_nz"] - lc)
    if not n:
        continue
    rows.append((a["tag"], n, st.median([abs(x) for x in dr]),
                 F.q([abs(x) for x in dr], .95),
                 st.median([abs(x) for x in dc]), F.q([abs(x) for x in dc], .95)))
h = "%-9s %7s %10s %10s %10s %10s"
print(h % ("tag", "n", "|dreq|med", "|dreq|p95", "|dcmd|med", "|dcmd|p95"))
for r in rows:
    print(h % (r[0], r[1], "%.4f" % r[2], "%.4f" % r[3], "%.4f" % r[4], "%.4f" % r[5]))
alln = sum(r[1] for r in rows)
print("  pooled n=%d ; median |dcmd_nz| across airframes = %.4f g"
      % (alln, st.median([r[4] for r in rows])))

print("\n=== tracking-scaled slew (slope 0.336 from Q3d2 applied to the clampON rate)")
G = 9.80665
SLOPE = 0.3361
h2 = "%-9s %s %8s %8s %9s %9s %9s %9s"
print(h2 % ("tag", "F", "band_s", "gam_ent", "rate_cmd", "rate_eff", "t_to_0", "gam_end"))
for a in acs:
    if a["tag"] not in B.FAIL:
        continue
    b = a["band_clampON"]
    ge = D(b[0]["gam"])
    rate = st.median([D((-1.0 - math.cos(r["gam"])) * G / r["V"]) for r in b])
    eff = rate * SLOPE
    bs = b[-1]["t"] - b[0]["t"]
    t0 = (ge / abs(eff)) if ge > 0 else 0.0
    print(h2 % (a["tag"], "*", "%.1f" % bs, "%+.2f" % ge, "%.2f" % rate,
                "%.2f" % eff, ("%.1f" % t0) if t0 <= bs else ">band",
                "%+.1f" % (ge + eff * bs)))
