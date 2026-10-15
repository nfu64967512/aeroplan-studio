# -*- coding: utf-8 -*-
from __future__ import annotations
import math, sys, pathlib, statistics as st
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import b1_full as F, b1_band as B
D = math.degrees
acs = F.load()

print("=== Q1  band = release .. PN handover;  20 Hz rows;  k1 == first logged cmd_nz in 16/16 NZC")
print("dh = measured altitude change over the band (+ = CLIMBED).  gcmd = commanded gamma.")
print()
h = ("%-9s %s %4s %5s %7s %7s %7s | %8s %8s %6s | %8s %8s %6s"
     % ("tag", "F", "n", "band_s", "dh_m", "gam_med", "gam_max",
        "OFFmed", "OFFp90", "OFFdsc", "ONmed", "ONp90", "ONdsc"))
print(h); print("-" * len(h))
for a in acs:
    bo, bn = a["band_clampOFF"], a["band_clampON"]
    gam = [D(r["gam"]) for r in bo]
    dh = bo[-1]["h"] - bo[0]["h"]
    go = [D(r["gam_cmd"]) for r in bo]
    gn = [D(r["gam_cmd"]) for r in bn]
    print("%-9s %s %4d %5.1f %7.1f %7.2f %7.2f | %8.2f %8.2f %5.1f%% | %8.2f %8.2f %5.1f%%"
          % (a["tag"], "*" if a["tag"] in B.FAIL else " ", len(bo),
             bo[-1]["t"] - bo[0]["t"], dh, st.median(gam), max(gam),
             st.median(go), F.q(go, .90), 100 * sum(1 for v in go if v < 0) / len(go),
             st.median(gn), F.q(gn, .90), 100 * sum(1 for v in gn if v < 0) / len(gn)))

print("\n=== Q1 payoff: THE SIX FAILURES -- would _pn_cmd have commanded a DESCENT?")
hh = ("%-9s %6s %7s %7s | %8s %8s %8s %6s | %8s %8s %8s %6s %8s"
      % ("tag", "dh_m", "gam_ent", "gam_max", "OFF_ent", "OFF_med", "OFF_max",
         "dsc%", "ON_ent", "ON_med", "ON_max", "dsc%", "ON_1st<0"))
print(hh); print("-" * len(hh))
for a in acs:
    if a["tag"] not in B.FAIL: continue
    bo, bn = a["band_clampOFF"], a["band_clampON"]
    go = [D(r["gam_cmd"]) for r in bo]; gn = [D(r["gam_cmd"]) for r in bn]
    gam = [D(r["gam"]) for r in bo]
    first = next((i for i, v in enumerate(gn) if v < 0), None)
    tfirst = ("%.2fs" % (bn[first]["t"] - bn[0]["t"])) if first is not None else "never"
    print("%-9s %6.1f %7.2f %7.2f | %8.2f %8.2f %8.2f %5.1f%% | %8.2f %8.2f %8.2f %5.1f%% %8s"
          % (a["tag"], bo[-1]["h"] - bo[0]["h"], gam[0], max(gam),
             go[0], st.median(go), max(go), 100 * sum(1 for v in go if v < 0) / len(go),
             gn[0], st.median(gn), max(gn), 100 * sum(1 for v in gn if v < 0) / len(gn),
             tfirst))

print("\n=== climb/descend split as MEASURED in the band (dh>0 = climber)")
cl = [a for a in acs if a["band_clampOFF"][-1]["h"] - a["band_clampOFF"][0]["h"] > 0]
de = [a for a in acs if a not in cl]
print("  climbers %d/32 : %s" % (len(cl), " ".join(a["tag"] for a in cl)))
print("  of the 6 failures, climbers by this measure: %d"
      % sum(1 for a in cl if a["tag"] in B.FAIL))
for nm, grp in (("climb", cl), ("descend", de)):
    dh = [a["band_clampOFF"][-1]["h"] - a["band_clampOFF"][0]["h"] for a in grp]
    dv = [a["band_clampOFF"][-1]["vh"] - a["band_clampOFF"][0]["vh"] for a in grp]
    print("   %-7s n=%2d  dh med %+7.1f m   d(gs) med %+6.1f m/s"
          % (nm, len(grp), st.median(dh), st.median(dv)))
