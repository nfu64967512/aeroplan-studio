# -*- coding: utf-8 -*-
"""C6 evidence 3: is the SPEC CLAIM true? -- "a ground-frame path of radius >= R_eff needs bank <= phi_max
at every phase".  Test: fly a ground CIRCLE of radius R_g in uniform wind, compute the required bank at
every track angle, find the smallest feasible R_g, compare with R_eff = R_theo(1+W/Va)^2."""
import math, sys, io, json
sys.path.insert(0, r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
import numpy as np
from hv_dccpp.platform import PLATFORM_PIONEER, PLATFORM_HERO, G_MPS2

def required_Rg_over_Rtheo(Va, W, n=400001, turn="left"):
    """min R_g / R_theo such that required |a| <= Va^2/R_theo everywhere on a ground circle.
    Wind blowing toward +x, WLOG.  Returns (factor, theta_worst_deg)."""
    th = np.linspace(0.0, 2*math.pi, n, endpoint=False)
    tx, ty = np.cos(th), np.sin(th)
    sgn = 1.0 if turn == "left" else -1.0
    nx, ny = -sgn*np.sin(th), sgn*np.cos(th)          # inward normal of the ground circle
    wx, wy = W, 0.0
    wpar = wx*tx + wy*ty
    wperp = wx*nx + wy*ny
    rad = Va*Va - wperp*wperp
    if np.any(rad <= 0):                               # |W_perp| >= Va -> infeasible track direction
        return math.inf, float("nan")
    Vg = np.sqrt(rad) + wpar
    if np.any(Vg <= 0):
        return math.inf, float("nan")
    dth = th[1]-th[0]
    dVg = np.gradient(Vg, dth, edge_order=2)
    f = Vg*np.sqrt(dVg*dVg + Vg*Vg)/(Va*Va)            # R_g/R_theo needed at this phase
    i = int(np.argmax(f))
    return float(f[i]), float(math.degrees(th[i]))

print("=== spec claim check: min ground-circle radius so that bank <= phi_max, vs R_theo(1+W/Va)^2 ===")
rows = {}
for spec in (PLATFORM_PIONEER, PLATFORM_HERO):
    Va, Rth, phimax = spec.cruise_airspeed_mps, spec.turn_radius_theoretical_m, spec.max_bank_angle_deg
    for W in (0.0, 5.0, 8.0, 12.0):
        for turn in ("left", "right"):
            fac, thw = required_Rg_over_Rtheo(Va, W, turn=turn)
            claim = (1.0+W/Va)**2
            Rg_req = fac*Rth
            Reff = spec.r_eff(W)
            # actual worst bank if we fly the ground circle at R_g = R_eff
            a_over_g = (Va*Va/Rth)*(fac*Rth/Reff)/G_MPS2 if math.isfinite(fac) else float("inf")
            phi_at_Reff = math.degrees(math.atan(a_over_g))
            print(f"  [{spec.cls}] W={W:4.1f} {turn:5s}: need R_g/R_theo={fac:7.4f} (worst track {thw:6.1f} deg) ; "
                  f"claim (1+W/Va)^2={claim:7.4f} ; diff={100*(fac-claim)/max(claim,1e-9):+7.4f}% ; "
                  f"R_g,req={Rg_req:7.2f} m vs R_eff={Reff:7.2f} m ; bank on R_eff circle={phi_at_Reff:5.2f} deg "
                  f"(limit {phimax:.0f})")
            rows[f"{spec.cls}_W{W:g}_{turn}"] = {"need_factor": round(fac,6), "claim": round(claim,6),
                                                 "R_g_req_m": round(Rg_req,3), "R_eff_m": round(Reff,3),
                                                 "bank_deg_on_R_eff": round(phi_at_Reff,3),
                                                 "phi_max_deg": phimax}
with open(r"C:/Users/joy46/Documents/Projects/aeroplan-studio/runs/artifacts/PAPERPLAN1/c6_reff/ground_circle_bank.json","w",encoding="utf-8") as f:
    json.dump(rows, f, ensure_ascii=False, indent=2)
print("\nwrote ground_circle_bank.json")
