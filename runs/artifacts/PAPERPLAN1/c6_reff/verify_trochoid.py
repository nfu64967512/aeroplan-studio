# -*- coding: utf-8 -*-
"""C6 evidence 2: numerically verify R_eff = R_theo(1+W/Va)^2 is the MAX radius of curvature
of the ground track (trochoid) of a constant-bank turn in uniform wind; plus W=8 consequences."""
import math, sys, io, json
sys.path.insert(0, r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
import numpy as np
from hv_dccpp.platform import PLATFORM_PIONEER, PLATFORM_HERO
from hv_dccpp.dubins import skip_k
from hv_dccpp.decompose import d_turn_from_r

def trochoid_max_curv_radius(Va, R, Wx, Wy, n=200001):
    """Air-frame circle of radius R at airspeed Va + uniform wind (Wx,Wy).
    Ground track r(t) = R*(cos wt, sin wt)*... ; return max |v|^3/|v x a| over one air-lap."""
    w = Va / R                       # turn rate [rad/s]
    t = np.linspace(0.0, 2*math.pi/w, n)
    ph = w*t
    # air velocity (left turn): Va*(cos ph, sin ph); accel = Va*w*(-sin ph, cos ph)
    vx = Va*np.cos(ph) + Wx
    vy = Va*np.sin(ph) + Wy
    ax = -Va*w*np.sin(ph)
    ay =  Va*w*np.cos(ph)
    cross = vx*ay - vy*ax
    sp = np.hypot(vx, vy)
    with np.errstate(divide="ignore", invalid="ignore"):
        rho = sp**3/np.abs(cross)
    rho = rho[np.isfinite(rho)]
    return float(np.max(rho)), float(np.min(rho))

print("=== numeric check: max radius of curvature of the wind-blown ground track ===")
print(" (air circle radius R = R_theo; wind direction swept; analytic claim = R*(1+W/Va)^2)")
res = {}
for spec in (PLATFORM_PIONEER, PLATFORM_HERO):
    Va, R = spec.cruise_airspeed_mps, spec.turn_radius_theoretical_m
    for Wsp in (5.0, 8.0, 12.0):
        worst = -1.0; worst_dir = None; worst_min = None
        for deg in range(0, 360, 5):      # wind blowing-toward direction
            Wx, Wy = Wsp*math.cos(math.radians(deg)), Wsp*math.sin(math.radians(deg))
            mx, mn = trochoid_max_curv_radius(Va, R, Wx, Wy)
            if mx > worst: worst, worst_dir, worst_min = mx, deg, mn
        analytic = R*(1.0+Wsp/Va)**2
        print(f"  [{spec.cls}] Va={Va} R_theo={R}  W={Wsp:4.1f}: numeric max rho={worst:8.3f} m "
              f"(min rho={worst_min:7.3f}) vs analytic R(1+W/Va)^2={analytic:8.3f} m  "
              f"rel.err={100*(worst-analytic)/analytic:+.4f}%")
        res[f"{spec.cls}_W{Wsp:g}"] = {"numeric_max_rho_m": round(worst,3), "analytic_m": round(analytic,3),
                                       "numeric_min_rho_m": round(worst_min,3)}

print("\n=== W=8 m/s consequences (this is where R_eff stops being R_plan) ===")
s = 97.0
e_ext = 0.0
for spec in (PLATFORM_PIONEER, PLATFORM_HERO):
    for Wsp in (5.0, 8.0):
        R = spec.r_eff(Wsp)
        k = skip_k(s, R)
        dt = d_turn_from_r(R, s, e_ext)
        print(f"  [{spec.cls}] W={Wsp:.0f} -> R_eff={R:7.2f} m  skip_k(s={s})={k}  "
              f"d_turn(e_ext=0)={dt:8.2f} m  lateral(R-s/2)={max(0.0,R-s/2):7.2f} m")

with open(r"C:/Users/joy46/Documents/Projects/aeroplan-studio/runs/artifacts/PAPERPLAN1/c6_reff/trochoid_check.json","w",encoding="utf-8") as f:
    json.dump(res, f, ensure_ascii=False, indent=2)
print("\nwrote trochoid_check.json")
