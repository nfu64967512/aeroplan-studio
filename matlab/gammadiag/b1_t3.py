# -*- coding: utf-8 -*-
from __future__ import annotations
import math, sys, pathlib, statistics as st
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import b1_full as F, b1_band as B
D = math.degrees
G = 9.80665
acs = F.load()


def stats(v):
    return (min(v), F.q(v, .05), F.q(v, .25), st.median(v), F.q(v, .75),
            F.q(v, .95), max(v))


def _smooth(acs, key):
    out = []
    for a in acs:
        seq = a[key]
        for i in range(len(seq) - 20):
            dt = seq[i + 20]["t"] - seq[i]["t"]
            if 0.5 < dt < 2.0:
                out.append(D(seq[i + 20]["gam"] - seq[i]["gam"]) / dt)
    return out or [float("nan")]


print("=== Q3  pre-clamp req_nz  (nz_req = cos(gam) + V*gdot/g,  gdot = -n_vert*sig_dot)")
print("    req_nz does NOT depend on nz_clamp_after_floor -- it is upstream of both clamps.")
band = [r["req_nz"] for a in acs for r in a["band_clampOFF"]]
pn = [r["req_nz"] for a in acs for r in a["pn_flown"]]
pn_t = [r["req_nz"] for a in acs if a["touched"] for r in a["pn_flown"]]
hdr = "%-26s %6s %9s %8s %8s %8s %8s %8s %9s %8s %8s"
print(hdr % ("segment", "n", "min", "p05", "p25", "med", "p75", "p95", "max",
             "out[-1,3]", ">3 %"))
for nm, v in (("BAND (release->handover)", band),
              ("POST-HANDOVER dive (32)", pn),
              ("POST-HANDOVER dive (30 td)", pn_t)):
    s = stats(v)
    out = 100.0 * sum(1 for x in v if x < -1.0 or x > 3.0) / len(v)
    hi = 100.0 * sum(1 for x in v if x > 3.0) / len(v)
    print(hdr % (nm, len(v), "%.3f" % s[0], "%.3f" % s[1], "%.3f" % s[2],
                 "%.3f" % s[3], "%.3f" % s[4], "%.3f" % s[5], "%.2f" % s[6],
                 "%.2f%%" % out, "%.2f%%" % hi))
lo = 100.0 * sum(1 for x in band if x < -1.0) / len(band)
print("  BAND: below -1.0 %.2f%% ; above 3.0 %.2f%%"
      % (lo, 100.0 * sum(1 for x in band if x > 3.0) / len(band)))

print("\n  per-airframe req_nz in the band")
h2 = "%-9s %s %6s %8s %8s %8s %8s %8s"
print(h2 % ("tag", "F", "n", "min", "p05", "med", "p95", "max"))
for a in acs:
    v = [r["req_nz"] for r in a["band_clampOFF"]]
    s = stats(v)
    print(h2 % (a["tag"], "*" if a["tag"] in B.FAIL else " ", len(v),
                "%.3f" % s[0], "%.3f" % s[1], "%.3f" % s[3], "%.3f" % s[5],
                "%.3f" % s[6]))

print("\n=== Q3b  the aero ceiling nz_max_true = CL_max*q*S/W in the band")
nmt = [r["nz_max_true"] for a in acs for r in a["band_clampOFF"]]
s = stats(nmt)
print("  band  min %.1f  p05 %.1f  med %.1f  p95 %.1f  max %.1f g   (contract cap nz_max_g = 3.0)"
      % (s[0], s[1], s[3], s[5], s[6]))

print("\n=== Q3c  MEASURED n_z the airframe was actually pulling in the band")
nzm = [r["nz_meas"] for a in acs for r in a["band_clampOFF"] if r["nz_meas"] == r["nz_meas"]]
s = stats(nzm)
print("  n=%d (central-difference gam-dot, +-0.25 s window)  min %.2f p05 %.2f med %.2f p95 %.2f max %.2f"
      % (len(nzm), s[0], s[1], s[3], s[5], s[6]))

print("\n=== Q3d  achieved gamma-rate: BAND (L1/TECS flying) vs POST-HANDOVER (PN flying)")
for nm, key in (("band", "band_clampOFF"), ("post-handover", "pn_flown")):
    v = []
    for a in acs:
        seq = a[key]
        for i in range(len(seq) - 1):
            dt = seq[i + 1]["t"] - seq[i]["t"]
            if 0.01 < dt < 0.3:
                v.append(D(seq[i + 1]["gam"] - seq[i]["gam"]) / dt)
    s = stats(v)
    print("  %-14s n=%6d  min %8.2f  p05 %7.2f  med %6.2f  p95 %6.2f  max %7.2f deg/s"
          % (nm, len(v), s[0], s[1], s[3], s[5], s[6]))
    sm = _smooth(acs, key)
    print("                   1 s-window rate: p05 %7.2f  med %6.2f  p95 %6.2f deg/s"
          % (F.q(sm, .05), st.median(sm), F.q(sm, .95)))


