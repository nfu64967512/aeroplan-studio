# -*- coding: utf-8 -*-
"""C7 evidence probe 3 -- airspeed vs groundspeed actually flown in HV4SITL16.

READ-ONLY. Pure python. Writes probe_flown.out.txt (UTF-8).
Proves (a) the SITL flew with zero wind (Vg == Va to telemetry resolution),
(b) there is no east/west groundspeed asymmetry, i.e. the wind triangle was
never exercised, and (c) sizes what a 5 m/s west wind would have done.
"""
from __future__ import annotations
import csv, io, math, os, sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..")))
import numpy as np
from hv_dccpp.wind import wind_vector, ground_speed

buf = io.StringIO()


def P(*a):
    print(*a, file=buf)


R_EARTH = 6378137.0
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
RUN = os.path.join(ROOT, "artifacts", "HV4SITL16")
VA = {"A1": 21.5, "B2": 19.0, "B3": 19.0}

wxy5 = np.asarray(wind_vector(5.0, 270.0), dtype=float).reshape(-1)

P("== airspeed vs groundspeed flown, runs/artifacts/HV4SITL16/<ac>/flight.csv ==")
P("   cruise sample filter: armed, mode AUTO, rel_alt >= 90 m, airspeed >= 12 m/s")
P("")
for ac in ["A1", "B2", "B3"]:
    p = os.path.join(RUN, ac, "flight.csv")
    if not os.path.isfile(p):
        P("  %s: no flight.csv" % ac)
        continue
    rows = list(csv.DictReader(open(p, encoding="utf-8")))
    prev = None
    samples = []      # (t, asp, gsp, track_deg)
    for r in rows:
        try:
            t = float(r["t_s"]); la = float(r["lat"]); lo = float(r["lon"])
            alt = float(r["rel_alt_m"]); asp = float(r["airspeed"]); gsp = float(r["groundspeed"])
        except (ValueError, KeyError):
            continue
        trk = None
        if prev is not None:
            dt = t - prev[0]
            if dt > 0:
                dx = math.radians(lo - prev[2]) * R_EARTH * math.cos(math.radians(la))
                dy = math.radians(la - prev[1]) * R_EARTH
                if math.hypot(dx, dy) > 5.0:
                    trk = math.degrees(math.atan2(dx, dy)) % 360.0
        prev = (t, la, lo)
        if r["armed"].strip().lower() in ("true", "1") and r["mode"].strip() == "AUTO" \
                and alt >= 90.0 and asp >= 12.0:
            samples.append((t, asp, gsp, trk))
    if not samples:
        P("  %s: no cruise samples" % ac)
        continue
    asp = np.array([s[1] for s in samples])
    gsp = np.array([s[2] for s in samples])
    d = gsp - asp
    P("  -- %s : %d cruise samples, t %.1f..%.1f s --" % (ac, len(samples), samples[0][0], samples[-1][0]))
    P("     airspeed    mean %.4f  sd %.4f  min %.2f max %.2f   (commanded AIRSPEED_CRUISE = %.1f)"
      % (asp.mean(), asp.std(), asp.min(), asp.max(), VA[ac]))
    P("     groundspeed mean %.4f  sd %.4f  min %.2f max %.2f" % (gsp.mean(), gsp.std(), gsp.min(), gsp.max()))
    P("     Vg - Va     mean %+.4f  sd %.4f  |max| %.4f   -> zero-wind signature"
      % (d.mean(), d.std(), np.abs(d).max()))
    # east vs west
    e = [s[2] for s in samples if s[3] is not None and 45.0 <= s[3] < 135.0]
    w = [s[2] for s in samples if s[3] is not None and 225.0 <= s[3] < 315.0]
    if e and w:
        P("     groundspeed on EAST tracks  n=%3d mean %.4f" % (len(e), float(np.mean(e))))
        P("     groundspeed on WEST tracks  n=%3d mean %.4f" % (len(w), float(np.mean(w))))
        P("     east-west groundspeed difference = %.4f m/s" % (float(np.mean(e)) - float(np.mean(w))))
        vge = float(ground_speed(math.radians(90), VA[ac], wxy5)[0])
        vgw = float(ground_speed(math.radians(270), VA[ac], wxy5)[0])
        P("     a real 5 m/s west wind would have made that difference %.1f m/s (%.1f vs %.1f)"
          % (vge - vgw, vge, vgw))
    P("")

P("== makespan arithmetic (paper Table 2 vs the planner wind budget) ==")
P("   paper Table 2: planned makespan 1260.4 s, SITL 1268.0 s  -> gap +7.6 s (+0.60%)")
P("   A1 planned t_takeoff 656.0 s, t_land 1260.4 s -> A1 sortie 604.4 s (release_schedule.json)")
P("   probe_legs.py: A1 cruise-leg time 506.24 s at 5 m/s W wind vs 481.00 s calm -> wind budget 25.24 s")
P("   => the wind term the planner charged A1 (25.2 s) is %.1fx the plan-vs-SITL makespan gap (7.6 s)"
  % (25.24 / 7.6))
P("   => the reported agreement is NOT evidence that the wind cost model is right; it is")
P("      consistent with a +25 s wind budget cancelling a similar amount of un-modelled")
P("      SITL-side cost (transition, climb/descent rate, waypoint capture, pad queueing).")
P("      [this attribution is an ESTIMATE: only the two summed quantities are measured]")
P("")

sys.stdout.reconfigure(encoding="utf-8")
out = buf.getvalue()
print(out)
with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "probe_flown.out.txt"),
          "w", encoding="utf-8") as fh:
    fh.write(out)
