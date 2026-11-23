# -*- coding: utf-8 -*-
"""C7 evidence probe -- what the HARVEST time cost function actually computes.

READ-ONLY on the repo. Pure python, no SITL. Writes probe_costfn.out.txt (UTF-8).
Run:  python runs/artifacts/PAPERPLAN1/c7_costfn/probe_costfn.py
"""
from __future__ import annotations
import io, glob, math, sys, os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..")))
import numpy as np

buf = io.StringIO()


def P(*a):
    print(*a, file=buf)


from hv_dccpp import platform as plat
from hv_dccpp.wind import (wind_vector, ground_speed, heading_from_track, r_eff,
                           circle_time_factor, proxy_turn_time)

# ------------------------------------------------------------------ specs
SPECS = {}
for name in dir(plat):
    o = getattr(plat, name)
    if isinstance(o, plat.PlatformSpec):
        SPECS[name] = o
P("== PlatformSpec objects found in hv_dccpp.platform ==")
for n, s in SPECS.items():
    P("  %s: cls=%r Va=%s R_theo=%s R_plan=%s phi_max=%s v_min_mission=%.3f v_max=%s"
      % (n, getattr(s, "cls", None), s.cruise_airspeed_mps, s.turn_radius_theoretical_m,
         s.turn_radius_planning_m, s.max_bank_angle_deg, s.v_min_mission(), s.max_airspeed_mps))
P("")

A = next(s for s in SPECS.values() if abs(s.cruise_airspeed_mps - 21.5) < 1e-6)
B = next(s for s in SPECS.values() if abs(s.cruise_airspeed_mps - 19.0) < 1e-6)

P("== 1. R_eff(W) = max{R_plan, R_theo*(1+W/Va)^2}   (hv_dccpp/platform.py:296-300) ==")
P("%7s | %9s %9s %8s | %9s %9s %8s" % ("W[m/s]", "A R_eff", "A raw", "A bound", "B R_eff", "B raw", "B bound"))
for W in [0.0, 2.0, 4.0, 5.0, 5.03, 5.17, 6.0, 8.0, 10.0]:
    ra_raw = A.turn_radius_theoretical_m * (1.0 + W / A.cruise_airspeed_mps) ** 2
    rb_raw = B.turn_radius_theoretical_m * (1.0 + W / B.cruise_airspeed_mps) ** 2
    ra, rb = A.r_eff(W), B.r_eff(W)
    P("%7.2f | %9.2f %9.2f %8s | %9.2f %9.2f %8s"
      % (W, ra, ra_raw, "R_plan" if ra_raw <= A.turn_radius_planning_m else "wind",
         rb, rb_raw, "R_plan" if rb_raw <= B.turn_radius_planning_m else "wind"))
P("  W* (wind absorbed by the R_plan margin, platform.py:302-304): A = %.4f m/s, B = %.4f m/s"
  % (A.w_star(), B.w_star()))
P("  => at the paper assumed W = 5 m/s BOTH configurations sit on the R_plan floor:")
P("     A: raw %.2f m < R_plan %s m" % (A.turn_radius_theoretical_m * (1 + 5 / A.cruise_airspeed_mps) ** 2,
                                        A.turn_radius_planning_m))
P("     B: raw %.2f m < R_plan %s m" % (B.turn_radius_theoretical_m * (1 + 5 / B.cruise_airspeed_mps) ** 2,
                                        B.turn_radius_planning_m))
P("     i.e. R_eff(5) == R_eff(0) == R_plan.  The wind term in R_eff is INERT at 5 m/s.")
P("")

# ------------------------------------------------------------------ span coefficient
P("== 2. span coefficient k = ceil(2 R_eff / s) at the paper s = 97.0 m ==")
s_strip = 97.0
for W in [0.0, 5.0, 8.0, 10.0, 12.0]:
    ka = math.ceil(2 * A.r_eff(W) / s_strip)
    kb = math.ceil(2 * B.r_eff(W) / s_strip)
    P("  W=%4.1f: A R_eff=%7.2f k=%d   B R_eff=%7.2f k=%d" % (W, A.r_eff(W), ka, B.r_eff(W), kb))
P("  paper claims Pioneer k=5, HERO k=3 -> reproduced, but only because R_eff == R_plan.")
P("")

# ------------------------------------------------------------------ wind triangle
P("== 3. time-cost ground speed  Vg = sqrt(Va^2 - Wperp^2) + Wpar   (hv_dccpp/wind.py:278-291) ==")
P("   theta = TRACK bearing, ENU, atan2(dx,dy) => 0 deg = north(+y), 90 deg = east(+x)")
W_SPEED, W_DIR_FROM = 5.0, 270.0   # scripts/hv_sitl_scenarios.py:66
wxy = np.asarray(wind_vector(W_SPEED, W_DIR_FROM), dtype=float).reshape(-1)
P("   scenario wind = %s m/s FROM %s deg -> blowing-to vector (x_east, y_north) = (%.4f, %.4f)"
  % (W_SPEED, W_DIR_FROM, wxy[0], wxy[1]))
P("")
P("%7s %13s | %7s %7s | %7s %7s | %10s %9s | %11s"
  % ("track", "dirn", "Wpar", "Wperp", "Vg A", "Vg B", "t/1000m A", "vs calm", "crab A[deg]"))
for th_deg, lbl in [(0, "N"), (45, "NE"), (90, "E downwind"), (135, "SE"), (180, "S"),
                    (225, "SW"), (270, "W upwind"), (315, "NW")]:
    th = math.radians(th_deg)
    st, ct = math.sin(th), math.cos(th)
    wpar = wxy[0] * st + wxy[1] * ct
    wperp = wxy[0] * ct - wxy[1] * st
    vga = float(ground_speed(th, A.cruise_airspeed_mps, wxy)[0])
    vgb = float(ground_speed(th, B.cruise_airspeed_mps, wxy)[0])
    ta = 1000.0 / vga
    t0 = 1000.0 / A.cruise_airspeed_mps
    psi = float(heading_from_track(th, A.cruise_airspeed_mps, wxy)[0])
    P("%7.0f %13s | %7.3f %7.3f | %7.3f %7.3f | %10.3f %+8.2f%% | %+11.3f"
      % (th_deg, lbl, wpar, wperp, vga, vgb, ta, 100 * (ta - t0) / t0, math.degrees(psi - th)))
P("")
P("   -> BOTH components enter: Wpar shifts Vg linearly, Wperp costs sqrt(Va^2 - Wperp^2).")
P("      A pure-headwind model would give Vg = Va - W on every leg. That is NOT what runs.")
P("")

# ------------------------------------------------------------------ cancellation
P("== 4. east-west cancellation (why a no-wind SITL still agreed with a 5 m/s plan) ==")
for spec, nm in ((A, "A/Pioneer Va=21.5"), (B, "B/HERO    Va=19.0")):
    va = spec.cruise_airspeed_mps
    vg_e = float(ground_speed(math.radians(90), va, wxy)[0])
    vg_w = float(ground_speed(math.radians(270), va, wxy)[0])
    L = 1560.0
    t_pair_wind = L / vg_e + L / vg_w
    t_pair_calm = 2 * L / va
    P("  %s: E leg Vg=%.3f  W leg Vg=%.3f | one E+W pair over L=%.0f m: wind %.3f s vs calm %.3f s = %+.3f%%"
      % (nm, vg_e, vg_w, L, t_pair_wind, t_pair_calm, 100 * (t_pair_wind - t_pair_calm) / t_pair_calm))
    P("      single E leg alone: %+.2f%% , single W leg alone: %+.2f%%  (they nearly cancel in a pair)"
      % (100 * ((L / vg_e) - (L / va)) / (L / va), 100 * ((L / vg_w) - (L / va)) / (L / va)))
P("  -> residual is second order: 1/(Va-W) + 1/(Va+W) = 2Va/(Va^2-W^2) > 2/Va")
P("")
P("   contrast: N-S strips under the same 5 m/s west wind (pure crosswind, no cancellation):")
for spec, nm in ((A, "A"), (B, "B")):
    va = spec.cruise_airspeed_mps
    vg_n = float(ground_speed(0.0, va, wxy)[0])
    P("     %s: Vg=%.4f vs Va=%s => every leg %+.3f%% slower" % (nm, vg_n, va, 100 * ((1 / vg_n) - (1 / va)) / (1 / va)))
P("")
P("   8 m/s west wind, same geometry (advisor Case 3):")
wxy8 = np.asarray(wind_vector(8.0, 270.0), dtype=float).reshape(-1)
for spec, nm in ((A, "A"), (B, "B")):
    va = spec.cruise_airspeed_mps
    vg_e = float(ground_speed(math.radians(90), va, wxy8)[0])
    vg_w = float(ground_speed(math.radians(270), va, wxy8)[0])
    vg_n = float(ground_speed(0.0, va, wxy8)[0])
    L = 1560.0
    P("     %s: E Vg=%.3f W Vg=%.3f N Vg=%.3f | E+W pair %+.3f%% vs calm | R_eff=%.2f (raw %.2f)"
      % (nm, vg_e, vg_w, vg_n,
         100 * ((L / vg_e + L / vg_w) - 2 * L / va) / (2 * L / va),
         spec.r_eff(8.0), spec.turn_radius_theoretical_m * (1 + 8.0 / va) ** 2))
P("")

# ------------------------------------------------------------------ turn factors
P("== 5. turn & orbit time factors (hv_dccpp/wind.py:424-449) ==")
for W in [0.0, 5.0, 8.0]:
    P("  W=%4.1f: circle_time_factor A=%.6f B=%.6f | proxy 1+(W/Va)^2 A=%.6f B=%.6f"
      % (W, circle_time_factor(A.cruise_airspeed_mps, W), circle_time_factor(B.cruise_airspeed_mps, W),
         1 + (W / A.cruise_airspeed_mps) ** 2, 1 + (W / B.cruise_airspeed_mps) ** 2))
P("")

# ------------------------------------------------------------------ export check
P("== 6. what is actually commanded to the flight controller (DO_CHANGE_SPEED, cmd 178) ==")
root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
for run in ["HV4SITL16", "HV4SITL15", "HV4SITL13", "HV4SITL11"]:
    d = os.path.join(root, "artifacts", run, "plan")
    if not os.path.isdir(d):
        continue
    P("  -- %s --" % run)
    for wp in sorted(glob.glob(os.path.join(d, "*.waypoints"))):
        rows = []
        with open(wp, encoding="utf-8", errors="replace") as fh:
            for line in fh:
                if line.startswith("QGC") or line.startswith("#"):
                    continue
                f = line.rstrip("\n").split("\t")
                if len(f) >= 6 and f[3] == "178":
                    rows.append((f[0], f[4], f[5]))
        P("     %-16s DO_CHANGE_SPEED x%d: %s"
          % (os.path.basename(wp), len(rows),
             ", ".join("seq%s param1=%s(0=AIRSPEED,1=GROUNDSPEED) param2=%s" % r for r in rows)))
P("")

sys.stdout.reconfigure(encoding="utf-8")
out = buf.getvalue()
print(out)
with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "probe_costfn.out.txt"),
          "w", encoding="utf-8") as fh:
    fh.write(out)
