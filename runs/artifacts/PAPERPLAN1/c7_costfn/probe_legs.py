# -*- coding: utf-8 -*-
"""C7 evidence probe 2 -- per-leg wind-triangle time on the ACTUAL exported HV4SITL16 mission.

READ-ONLY. Pure python. Writes probe_legs.out.txt (UTF-8).
Reconstructs each cruise leg from the exported .waypoints, computes:
  * leg bearing (track), length
  * planner ground speed Vg(theta) under the scenario 5 m/s / 270 deg wind
  * calm ground speed (= Va, what the no-wind SITL actually flew)
and sums the per-aircraft time both ways, to size the wind-model error the
table note ("SITL 未設風") admits to.
"""
from __future__ import annotations
import io, glob, json, math, os, sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..")))
import numpy as np
from hv_dccpp.wind import wind_vector, ground_speed

buf = io.StringIO()


def P(*a):
    print(*a, file=buf)


R_EARTH = 6378137.0
NAV_CMDS = {16, 21, 22, 84, 85, 31, 18, 17, 19}


def enu(lat, lon, lat0, lon0):
    x = math.radians(lon - lon0) * R_EARTH * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * R_EARTH
    return x, y


def load_nav(path):
    pts = []
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            if line.startswith("QGC") or line.startswith("#"):
                continue
            f = line.rstrip("\n").split("\t")
            if len(f) < 12:
                continue
            cmd = int(f[3])
            lat, lon, alt = float(f[8]), float(f[9]), float(f[10])
            if cmd in NAV_CMDS and (abs(lat) > 1e-7 or abs(lon) > 1e-7):
                pts.append((int(f[0]), cmd, lat, lon, alt))
    return pts


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
RUN = os.path.join(ROOT, "artifacts", "HV4SITL16")

W_SPEED, W_DIR = 5.0, 270.0
wxy = np.asarray(wind_vector(W_SPEED, W_DIR), dtype=float).reshape(-1)
wxy8 = np.asarray(wind_vector(8.0, 270.0), dtype=float).reshape(-1)
VA = {"A1": 21.5, "B2": 19.0, "B3": 19.0}

P("== per-leg wind-triangle audit of runs/artifacts/HV4SITL16/plan/*.waypoints ==")
P("   wind used by the planner: %.1f m/s FROM %.0f deg -> to-vector (E,N) = (%.3f, %.3f)"
  % (W_SPEED, W_DIR, wxy[0], wxy[1]))
P("   NOTE: the SITL that flew these missions had NO wind (no SIM_WIND anywhere in scripts/).")
P("")

totals = {}
for name in ["A1", "B2", "B3"]:
    wp = os.path.join(RUN, "plan", "%s.waypoints" % name)
    pts = load_nav(wp)
    if len(pts) < 2:
        continue
    lat0, lon0 = pts[0][2], pts[0][3]
    va = VA[name]
    L_tot = t_wind = t_calm = t_w8 = 0.0
    bearings = []
    for i in range(len(pts) - 1):
        _, _, la, lo, _ = pts[i]
        _, _, lb, lob, _ = pts[i + 1]
        x0, y0 = enu(la, lo, lat0, lon0)
        x1, y1 = enu(lb, lob, lat0, lon0)
        dx, dy = x1 - x0, y1 - y0
        L = math.hypot(dx, dy)
        if L < 1.0:
            continue
        th = math.atan2(dx, dy)
        vg = float(ground_speed(th, va, wxy)[0])
        vg8 = float(ground_speed(th, va, wxy8)[0])
        L_tot += L
        t_wind += L / vg
        t_calm += L / va
        t_w8 += L / vg8 if vg8 > 0 else float("inf")
        bearings.append((math.degrees(th) % 360.0, L))
    totals[name] = (L_tot, t_wind, t_calm, t_w8, len(bearings))
    P("  -- %s (Va=%.1f) : %d legs, straight-line length %.1f m --" % (name, va, len(bearings), L_tot))
    P("     sum L/Vg  with 5 m/s W wind : %9.2f s" % t_wind)
    P("     sum L/Va  calm (what SITL flew): %9.2f s   -> planner is %+.2f%% SLOWER than calm"
      % (t_calm, 100 * (t_wind - t_calm) / t_calm))
    P("     sum L/Vg  with 8 m/s W wind : %9.2f s   -> %+.2f%% vs calm"
      % (t_w8, 100 * (t_w8 - t_calm) / t_calm))
    # bearing histogram, length-weighted, 8 sectors
    hist = {}
    for b, L in bearings:
        sec = int(((b + 22.5) % 360) // 45)
        hist[sec] = hist.get(sec, 0.0) + L
    lbl = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    P("     length-weighted track histogram: " +
      "  ".join("%s=%.0fm" % (lbl[k], v) for k, v in sorted(hist.items())))
    ew = hist.get(2, 0.0) + hist.get(6, 0.0)
    ns = hist.get(0, 0.0) + hist.get(4, 0.0)
    P("     E+W legs = %.0f m (%.1f%%), N+S legs = %.0f m (%.1f%%)  -> cancellation applies to the E/W share"
      % (ew, 100 * ew / L_tot, ns, 100 * ns / L_tot))
    P("")

P("== totals ==")
LT = sum(v[0] for v in totals.values())
TW = sum(v[1] for v in totals.values())
TC = sum(v[2] for v in totals.values())
T8 = sum(v[3] for v in totals.values())
P("   fleet straight-line length  %.1f m" % LT)
P("   fleet sum of leg times: 5 m/s wind %.1f s | calm %.1f s (%+.2f%%) | 8 m/s wind %.1f s (%+.2f%%)"
  % (TW, TC, 100 * (TW - TC) / TC, T8, 100 * (T8 - TC) / TC))
P("   (leg-sum only: excludes VTOL takeoff/landing, transition, loiter and 4D hold delays)")
P("")

# ---- what the run actually reported
for fn in ["validation.json", "report.json", "release_schedule.json"]:
    p = os.path.join(RUN, fn)
    if not os.path.isfile(p):
        continue
    try:
        d = json.load(open(p, encoding="utf-8"))
    except Exception as e:
        P("  (%s unreadable: %s)" % (fn, e))
        continue
    P("== %s : keys with numeric/str leaves ==" % fn)

    def walk(o, pre="", depth=0):
        if depth > 3:
            return
        if isinstance(o, dict):
            for k, v in o.items():
                if isinstance(v, (int, float, str, bool)) or v is None:
                    P("   %s%s = %r" % (pre, k, v))
                elif isinstance(v, list) and v and isinstance(v[0], (int, float, str)):
                    P("   %s%s = %r" % (pre, k, v[:8]))
                else:
                    P("   %s%s:" % (pre, k))
                    walk(v, pre + "  ", depth + 1)
        elif isinstance(o, list):
            for i, v in enumerate(o[:4]):
                walk(v, pre + "[%d]." % i, depth + 1)

    walk(d)
    P("")

sys.stdout.reconfigure(encoding="utf-8")
out = buf.getvalue()
print(out)
with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "probe_legs.out.txt"),
          "w", encoding="utf-8") as fh:
    fh.write(out)
