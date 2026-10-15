# -*- coding: utf-8 -*-
from __future__ import annotations
import math, sys, pathlib, statistics as st
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import b1_full as F, b1_band as B
D = math.degrees
acs = F.load()


def s5(v):
    return (min(v), F.q(v, .05), st.median(v), F.q(v, .95), max(v))


print("=== pitch actually SENT (SET_ATTITUDE_TARGET), band, both arms")
for nm, key in (("clampOFF", "band_clampOFF"), ("clampON", "band_clampON")):
    p = [D(r["pitch"]) for a in acs for r in a[key]]
    al = [D(r["alpha"]) for a in acs for r in a[key]]
    print("  %-9s pitch  min %7.2f p05 %7.2f med %7.2f p95 %7.2f max %7.2f  "
          "| at -80 clip: %d  at +45 clip: %d"
          % ((nm,) + s5(p) + (sum(1 for x in p if x <= -79.999),
             sum(1 for x in p if x >= 44.999))))
    print("            alpha  min %7.2f p05 %7.2f med %7.2f p95 %7.2f max %7.2f"
          % s5(al))
print("  (pitch = gam_cmd + alpha/max(cos roll,0.5); alpha reconstructed from the")
print("   measured n_z through the identified aero model -- it is a MODEL quantity,")
print("   the CSVs carry no AOA column for these sorties.)")

print("\n=== roll / lateral coupling in the band (context only, not the question)")
ro = [D(r["roll"]) for a in acs for r in a["band_clampOFF"]]
print("  roll  min %7.2f p05 %7.2f med %7.2f p95 %7.2f max %7.2f deg" % s5(ro))

print("\n=== Q4b  what the throttle SETPOINT v_req actually is")
vr = [r["v_req"] for a in acs for r in a["band_clampOFF"]]
V = [r["V"] for a in acs for r in a["band_clampOFF"]]
gap = [a - b for a, b in zip(vr, V)]
print("  v_req - V   min %7.1f p05 %7.1f med %7.1f p95 %7.1f max %7.1f m/s" % s5(gap))
print("  ticks with v_req < 95 m/s (below the FC AIRSPEED_MIN): %d/%d (%.2f%%)"
      % (sum(1 for x in vr if x < 95.0), len(vr),
         100.0 * sum(1 for x in vr if x < 95.0) / len(vr)))
print("  ticks with v_req < V (PN asking the aircraft to SLOW DOWN): %d (%.2f%%)"
      % (sum(1 for g in gap if g < 0), 100.0 * sum(1 for g in gap if g < 0) / len(gap)))
print("  throttle-law speed ceiling: thr=0 when V >= v_req + 12 m/s;")
print("  thr=1 when V <= v_req - 21.3 m/s  =>  proportional band is only 33.3 m/s wide.")
