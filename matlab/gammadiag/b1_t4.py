# -*- coding: utf-8 -*-
from __future__ import annotations
import math, sys, pathlib, statistics as st
HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "scripts" / "wsl" / "sar_swarm_patches"))
import b1_full as F, b1_band as B                                  # noqa: E402
import predictor as P                                              # noqa: E402
D = math.degrees
G = 9.80665
acs = F.load()


def stats(v):
    return (min(v), F.q(v, .05), st.median(v), F.q(v, .95), max(v))


print("=== Q3-addendum: post-handover req_nz outside the R->0 singularity")
for lim in (0.0, 100.0, 300.0):
    v = [r["req_nz"] for a in acs for r in a["pn_flown"] if r["d"] >= lim]
    s = stats(v)
    print("  d >= %4.0f m  n=%6d  min %12.3f  p05 %8.3f  med %6.3f  p95 %7.3f  max %12.2f  out[-1,3] %6.2f%%"
          % (lim, len(v), s[0], s[1], s[2], s[3], s[4],
             100.0 * sum(1 for x in v if x < -1 or x > 3) / len(v)))

print("\n=== Q3d2  tracking: post-handover, PN IS in command.  gdot_cmd vs achieved")
pairs = []
for a in acs:
    seq = a["pn_flown"]
    for i in range(len(seq)):
        j = i
        while j < len(seq) - 1 and seq[j]["t"] - seq[i]["t"] < 1.0:
            j += 1
        dt = seq[j]["t"] - seq[i]["t"]
        if not (0.8 <= dt <= 1.3):
            continue
        cmd = D(seq[i]["gam_cmd"] - seq[i]["gam"]) / 1.0
        ach = D(seq[j]["gam"] - seq[i]["gam"]) / dt
        if abs(cmd) > 0.5:
            pairs.append((cmd, ach, a["tag"]))
cm = [p[0] for p in pairs]; ac = [p[1] for p in pairs]
mx, my = st.mean(cm), st.mean(ac)
sxy = sum((x - mx) * (y - my) for x, y in zip(cm, ac))
sxx = sum((x - mx) ** 2 for x in cm)
syy = sum((y - my) ** 2 for y in ac)
print("  n=%d ticks (|cmd rate| > 0.5 deg/s, 1 s achieved window)" % len(pairs))
print("  slope achieved/commanded = %.4f    pearson r = %.4f" % (sxy / sxx, sxy / math.sqrt(sxx * syy)))
print("  commanded rate  med %7.2f  p05 %7.2f deg/s" % (st.median(cm), F.q(cm, .05)))
print("  achieved  rate  med %7.2f  p05 %7.2f deg/s" % (st.median(ac), F.q(ac, .05)))

print("\n=== Q4  ENERGY.  PN throttle law: thr = clip(0,1, 0.36 + 0.030*(v_req - V))")
print("    v_req = max(45, min(pn_vmax=210, R/t_rem)),  t_rem = dive_T - now")
print("    t_left reconstructed as secs_to_target + early_by_s (the _row identity).")
thr = [r["thr"] for a in acs for r in a["band_clampOFF"]]
vr = [r["v_req"] for a in acs for r in a["band_clampOFF"]]
V = [r["V"] for a in acs for r in a["band_clampOFF"]]
tl = [r["t_left"] for a in acs for r in a["band_clampOFF"]]
for nm, v in (("thr", thr), ("v_req m/s", vr), ("V (3D) m/s", V), ("t_left s", tl)):
    s = stats(v)
    print("  %-11s min %8.2f  p05 %8.2f  med %8.2f  p95 %8.2f  max %8.2f" % ((nm,) + s))
print("  ticks with thr == 0.0 : %d/%d (%.2f%%)  |  ticks thr >= 0.36 : %d (%.2f%%)"
      % (sum(1 for x in thr if x <= 0.0), len(thr),
         100.0 * sum(1 for x in thr if x <= 0.0) / len(thr),
         sum(1 for x in thr if x >= 0.36),
         100.0 * sum(1 for x in thr if x >= 0.36) / len(thr)))
print("  airframes whose ENTIRE band is thr==0 : %d/32"
      % sum(1 for a in acs if all(r["thr"] <= 0.0 for r in a["band_clampOFF"])))
print("  ticks flagged `late` (t_left<=1 -> v_req=pn_vmax) : %d"
      % sum(1 for a in acs for r in a["band_clampOFF"] if r["late"]))


def dvdt(v, h_amsl, dive_deg, thr_cmd):
    """point-mass, predictor.py physics (CD0=0.0150 calibrated n=1617).
    predictor's gam is POSITIVE-DOWN; tot_node's gam is positive-up."""
    gam = math.radians(dive_deg)
    T = min(max(0.0, min(1.0, thr_cmd)), 1.0) * P.THRUST_FULL_N   # THR_MAX=100
    return (T + P.WEIGHT_N * math.sin(gam) - P.drag_n(v, h_amsl, gam)) / P.MASS_KG


print("\n  scenario integration over each band (Euler on the measured tick grid)")
print("  A: PN throttle + the MEASURED gamma profile   (isolates the throttle channel)")
print("  B: PN throttle + the PN-commanded gamma (clampOFF ~ the LOS floor)")
print("  C: MEASURED speed change (what L1/TECS actually delivered)")
h = "%-9s %s %8s %8s %8s %8s %8s %8s"
print(h % ("tag", "F", "V_ent", "V_exit", "C_meas", "A_pred", "B_pred", "gam_eq"))
A, Bv, C = [], [], []
for a in acs:
    b = a["band_clampOFF"]
    va = vb = b[0]["V"]
    for i in range(len(b) - 1):
        dt = b[i + 1]["t"] - b[i]["t"]
        if not (0.0 < dt < 0.3):
            continue
        va += dvdt(va, b[i]["h"], -D(b[i]["gam"]), b[i]["thr"]) * dt
        vb += dvdt(vb, b[i]["h"], -D(b[i]["gam_cmd"]), b[i]["thr"]) * dt
    dc = b[-1]["V"] - b[0]["V"]
    # dive angle at which thr=0 holds speed at the entry V
    v0, h0 = b[0]["V"], b[0]["h"]
    dg = 0.0
    for g_ in [x * 0.1 for x in range(0, 900)]:
        if dvdt(v0, h0, g_, 0.0) >= 0.0:
            dg = g_; break
    A.append(va - b[0]["V"]); Bv.append(vb - b[0]["V"]); C.append(dc)
    print(h % (a["tag"], "*" if a["tag"] in B.FAIL else " ",
               "%.1f" % b[0]["V"], "%.1f" % b[-1]["V"], "%+.1f" % dc,
               "%+.1f" % (va - b[0]["V"]), "%+.1f" % (vb - b[0]["V"]),
               "%.1f" % dg))
print("\n  median over 32:  C_meas %+.1f   A_pred %+.1f   B_pred %+.1f  m/s"
      % (st.median(C), st.median(A), st.median(Bv)))
print("  A worst %+.1f   B worst %+.1f   (worst = most negative)" % (min(A), min(Bv)))
