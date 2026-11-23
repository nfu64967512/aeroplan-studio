# -*- coding: utf-8 -*-
"""B1 forward sim -- ⚠️ NOT a replay.  A vertical-plane point-mass EXTRAPOLATION
of what the band would look like if `_pn_cmd` flew it.  Physics = predictor.py
(CD0 = 0.01500, calibrated n=1617, PN-segment speed RMS 5.4 m/s).  Lateral
channel, wind and roll-induced lift loss are NOT modelled.  Two tracking
assumptions bracket the answer:
   K=1.00  the FC delivers the commanded gamma-rate exactly       (upper bound)
   K=0.40  the slope measured post-handover on the 16 NZC sorties (lower bound)
"""
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
NV, LEAD, NZMIN, NZMAX, VMAX = 5.0, 1.0, -1.0, 3.0, 210.0
acs = F.load()


def step_cmd(h, d, V, gam, clamp_after):
    R = max(1e-3, math.hypot(d, h))
    vh, hdot = V * math.cos(gam), V * math.sin(gam)
    sig_dot = (hdot * d + h * vh) / (R * R)
    gdot = -NV * sig_dot
    nz_req = math.cos(gam) + V * gdot / G
    nz = max(NZMIN, min(NZMAX, nz_req))
    gc = gam + ((nz - math.cos(gam)) * G / V) * LEAD
    gc = min(gc, -math.atan2(h, max(d, 1e-3)))
    if clamp_after:
        cz = math.cos(gam) + V * ((gc - gam) / LEAD) / G
        cz = max(NZMIN, min(NZMAX, cz))
        gc = gam + ((cz - math.cos(gam)) * G / V) * LEAD
    return gc, R


def sim(h, d, V, gam, t_left, clamp_after, K, dt=0.05, tmax=40.0):
    t = 0.0
    while d > 4700.0 and t < tmax and h > 0.0:
        gc, R = step_cmd(h, d, V, gam, clamp_after)
        t_rem = max(1e-3, t_left - t)
        v_req = VMAX if t_rem <= 1.0 else max(45.0, min(VMAX, R / t_rem))
        thr = max(0.0, min(1.0, 0.36 + 0.030 * (v_req - V)))
        dive = -gam
        V += (thr * P.THRUST_FULL_N + P.WEIGHT_N * math.sin(dive)
              - P.drag_n(V, h, dive)) / P.MASS_KG * dt
        V = max(20.0, V)
        gam += K * ((gc - gam) / LEAD) * dt
        h += V * math.sin(gam) * dt
        d -= V * math.cos(gam) * dt
        t += dt
    return dict(t=t, h=h, d=d, V=V, gam=gam,
                los=D(math.atan2(max(h, 0.0), max(d, 1.0))))


print("forward sim from each airframe's MEASURED band-entry state to d=4700 m")
print("MEAS = what actually happened (L1/TECS).  OFF/ON = nz_clamp_after_floor.")
h = "%-9s %s | %6s %6s %6s | %6s %6s %6s %5s | %6s %6s %6s %5s"
print(h % ("tag", "F", "h_HO", "LOS", "V_HO", "h_K1", "LOS", "V", "t",
           "h_K.4", "LOS", "V", "t"))
print("-" * 108)
agg = {}
for a in acs:
    b = a["band_clampOFF"]
    e, x = b[0], b[-1]
    tl = e["t_left"]
    r1 = sim(e["h"], e["d"], e["V"], e["gam"], tl, False, 1.00)
    r4 = sim(e["h"], e["d"], e["V"], e["gam"], tl, True, 0.40)
    los_m = D(math.atan2(x["h"], x["d"]))
    agg.setdefault("m", []).append(los_m)
    agg.setdefault("1", []).append(r1["los"]); agg.setdefault("4", []).append(r4["los"])
    agg.setdefault("mh", []).append(x["h"]); agg.setdefault("1h", []).append(r1["h"])
    agg.setdefault("4h", []).append(r4["h"])
    agg.setdefault("mv", []).append(x["V"]); agg.setdefault("1v", []).append(r1["V"])
    agg.setdefault("4v", []).append(r4["V"])
    print(h % (a["tag"], "*" if a["tag"] in B.FAIL else " ",
               "%.0f" % x["h"], "%.1f" % los_m, "%.0f" % x["V"],
               "%.0f" % r1["h"], "%.1f" % r1["los"], "%.0f" % r1["V"], "%.1f" % r1["t"],
               "%.0f" % r4["h"], "%.1f" % r4["los"], "%.0f" % r4["V"], "%.1f" % r4["t"]))
print("\nmedians   h_handover  LOS_handover  V_handover")
for k, nm in (("m", "MEASURED (L1/TECS)"), ("1", "PN clampOFF K=1.00"),
              ("4", "PN clampON  K=0.40")):
    print("  %-22s %8.0f m %10.1f deg %8.0f m/s"
          % (nm, st.median(agg[k + "h"] if k != "m" else agg["mh"]),
             st.median(agg[k]), st.median(agg[k + "v"] if k != "m" else agg["mv"])))
