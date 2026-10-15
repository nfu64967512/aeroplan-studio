# -*- coding: utf-8 -*-
from __future__ import annotations
import math, sys, pathlib, statistics as st
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import b1_full as F, b1_band as B
D = math.degrees
G = 9.80665
acs = F.load()

print("=== Q2  LOS floor  gam_los = -atan2(h,d)  (pn_los_floor = 1.0)")
print("floor_binds = the floor, not the PN nz path, is what sets the command.")
allb = [r for a in acs for r in a["band_clampOFF"]]
print("  band ticks n=%d ; floor binds on %d (%.2f%%) ; airframes with 100%% bind: %d/32"
      % (len(allb), sum(1 for r in allb if r["floor_binds"]),
         100.0 * sum(1 for r in allb if r["floor_binds"]) / len(allb),
         sum(1 for a in acs if all(r["floor_binds"] for r in a["band_clampOFF"]))))
gp = [D(r["gam_pre"]) for r in allb]
gl = [D(r["gam_los"]) for r in allb]
print("  gam_pre  (PN nz path, post [-1,3] clamp, pre floor): min %.2f p10 %.2f med %.2f p90 %.2f max %.2f"
      % (min(gp), F.q(gp, .1), st.median(gp), F.q(gp, .9), max(gp)))
print("  gam_los  (the floor itself)                        : min %.2f p10 %.2f med %.2f p90 %.2f max %.2f"
      % (min(gl), F.q(gl, .1), st.median(gl), F.q(gl, .9), max(gl)))
print()
h = ("%-9s %s | %7s %7s %8s | %7s %7s %8s | %8s %8s | %7s %7s"
     % ("tag", "F", "d_ent", "h_ent", "los_ent", "gam_ent", "gam_med", "step_ent",
        "nz_step", "nz_p95", "los_ho", "gam_ho"))
print(h); print("-" * len(h))
rows = []
for a in acs:
    b = a["band_clampOFF"]
    e = b[0]
    step = D(e["gam_los"] - e["gam"])
    nz_need = [math.cos(r["gam"]) + r["V"] * (r["gam_los"] - r["gam"]) / 1.0 / G
               for r in b]
    los_ho = D(-math.atan2(b[-1]["h"], b[-1]["d"]))
    rows.append((a["tag"], step, nz_need, los_ho))
    print("%-9s %s | %7.0f %7.0f %8.2f | %7.2f %7.2f %8.2f | %8.2f %8.2f | %7.2f %7.2f"
          % (a["tag"], "*" if a["tag"] in B.FAIL else " ", e["d"], e["h"],
             D(e["gam_los"]), D(e["gam"]), st.median([D(r["gam"]) for r in b]),
             step, nz_need[0], F.q(nz_need, .95), los_ho, D(b[-1]["gam"])))

steps = [r[1] for r in rows]
nz0 = [r[2][0] for r in rows]
nzw = [min(r[2]) for r in rows]
print("\n  step at band ENTRY (los - measured gam), deg : min %.2f med %.2f max %.2f"
      % (min(steps), st.median(steps), max(steps)))
print("  n_z needed to fly that step in pn_lead=1 s  : entry med %.2f  worst %.2f  (contract floor -1.0)"
      % (st.median(nz0), min(nzw)))
print("  ticks needing n_z < -1.0 : %d/%d (%.1f%%) ; airframes with any: %d/32"
      % (sum(1 for a in acs for r in a["band_clampOFF"]
             if math.cos(r["gam"]) + r["V"] * (r["gam_los"] - r["gam"]) / G < -1.0),
         len(allb),
         100.0 * sum(1 for a in acs for r in a["band_clampOFF"]
                     if math.cos(r["gam"]) + r["V"] * (r["gam_los"] - r["gam"]) / G < -1.0) / len(allb),
         sum(1 for a in acs if any(
             math.cos(r["gam"]) + r["V"] * (r["gam_los"] - r["gam"]) / G < -1.0
             for r in a["band_clampOFF"]))))

print("\n=== Q2b  cost of NOT descending in the band: geometry required at handover")
print("  If the airframe holds its measured profile through the band, the LOS")
print("  depression it hands PN is atan(h/d) at d=4700.  predictor.PN_GAMMA_CAP_DEG")
print("  = 36.0 is the MEASURED ceiling of what PN then delivers.")
ho = [(a["tag"], -D(math.atan2(a["band_clampOFF"][-1]["h"], a["band_clampOFF"][-1]["d"])),
       a["tag"] in B.FAIL) for a in acs]
over = [x for x in ho if -x[1] > 36.0]
print("  LOS depression at handover: min %.2f med %.2f max %.2f deg"
      % (min(-x[1] for x in ho), st.median([-x[1] for x in ho]), max(-x[1] for x in ho)))
print("  airframes handing over ABOVE the 36 deg measured ceiling: %d/32  %s"
      % (len(over), " ".join("%s(%.1f)" % (x[0], -x[1]) for x in over)))
print("  of the six failures: %s"
      % " ".join("%s=%.1f%s" % (x[0], -x[1], "!" if -x[1] > 36 else "")
                 for x in ho if x[2]))
