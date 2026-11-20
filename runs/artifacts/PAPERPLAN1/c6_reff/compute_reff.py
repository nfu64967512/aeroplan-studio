# -*- coding: utf-8 -*-
"""C6 evidence: R_eff numbers straight out of hv_dccpp (read-only, no SITL)."""
import math, sys, json, io
sys.path.insert(0, r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")

from hv_dccpp.platform import PLATFORM_PIONEER, PLATFORM_HERO, G_MPS2
from hv_dccpp import wind as W
from hv_dccpp.dubins import skip_k, omega_length, uturn_length

out = {}
for spec in (PLATFORM_PIONEER, PLATFORM_HERO):
    c = spec.cls
    va = spec.cruise_airspeed_mps
    rth = spec.turn_radius_theoretical_m
    rpl = spec.turn_radius_planning_m
    phi = spec.max_bank_angle_deg
    rth_formula = va*va/(G_MPS2*math.tan(math.radians(phi)))
    row = {
        "name": spec.name, "cls": c, "V_a_mps": va, "phi_max_deg": phi,
        "R_theo_table_m": rth, "R_theo_formula_Va2_over_g_tanphi_m": round(rth_formula, 2),
        "R_theo_rel_dev_pct": round(100.0*(rth_formula-rth)/rth, 2),
        "R_plan_m": rpl, "W_star_mps": round(spec.w_star(), 4),
        "R_eff_at_W": {},
        "R_theo_term_at_W": {},
        "binding_branch_at_W": {},
    }
    for w in (0.0, 2.0, 4.0, 5.0, 5.03, 5.17, 6.0, 8.0, 10.0, 12.0, 15.0, 19.0, 21.5, 25.0, 40.0):
        r = spec.r_eff(w)
        term = rth*(1.0+w/va)**2
        row["R_eff_at_W"][f"{w:g}"] = round(r, 3)
        row["R_theo_term_at_W"][f"{w:g}"] = round(term, 3)
        row["binding_branch_at_W"][f"{w:g}"] = "R_plan" if rpl >= term else "trochoid"
        assert abs(r - W.r_eff(spec, w)) < 1e-9
    out[c] = row

# skip-k at the scenario strip spacing
print("=== R_eff table (from hv_dccpp.platform.PlatformSpec.r_eff) ===")
for c, row in out.items():
    print(f"\n[{c}] {row['name']}  V_a={row['V_a_mps']} m/s  phi_max={row['phi_max_deg']} deg")
    print(f"  R_theo table={row['R_theo_table_m']} m ; Va^2/(g tan phi)={row['R_theo_formula_Va2_over_g_tanphi_m']} m "
          f"(dev {row['R_theo_rel_dev_pct']:+.2f}%)")
    print(f"  R_plan={row['R_plan_m']} m ; W*={row['W_star_mps']:.4f} m/s")
    print("   W[m/s]  R_theo(1+W/Va)^2   R_eff    binding")
    for k in row["R_eff_at_W"]:
        print(f"   {k:>6}  {row['R_theo_term_at_W'][k]:>14.2f}  {row['R_eff_at_W'][k]:>8.2f}   {row['binding_branch_at_W'][k]}")

print("\n=== skip-k = ceil(2 R_eff / s), s = strip spacing ===")
for s in (97.0, 100.0, 168.05, 270.0):
    ks = {c: skip_k(s, out[c]['R_eff_at_W']['5']) for c in out}
    print(f"  s={s:7.2f} m -> k_A(R_eff={out['A']['R_eff_at_W']['5']})={ks['A']}, "
          f"k_B(R_eff={out['B']['R_eff_at_W']['5']})={ks['B']}")

print("\n=== turn lengths at s=97.0 m, W=5 (Omega vs skip-k U) ===")
for c in out:
    R = out[c]["R_eff_at_W"]["5"]
    s = 97.0
    k = skip_k(s, R)
    print(f"  [{c}] R_eff={R:.1f}  Omega(s)={omega_length(s,R):.2f} m   U(k*s={k*s:.1f})={uturn_length(k*s,R):.2f} m   "
          f"min-> {'U(skip-k)' if uturn_length(k*s,R) < omega_length(s,R) else 'Omega'}")

# measured-in-flight comparison (from memory note fc-turns-tighter-than-planner; values are logged flight data)
print("\n=== cross-check vs 2026-09-05 FLYBOTH measured turn radius (source: memory note, not recomputed here) ===")
meas = {"A": 112.0, "B": 68.0}
for c in out:
    R = out[c]["R_eff_at_W"]["5"]
    print(f"  [{c}] R_eff(W=5)={R:.0f} m ; R_theo={out[c]['R_theo_table_m']:.0f} m ; measured={meas[c]:.0f} m ; "
          f"R_eff/measured={R/meas[c]:.2f}x ; measured/R_theo={meas[c]/out[c]['R_theo_table_m']:.3f}")

with open(r"C:/Users/joy46/Documents/Projects/aeroplan-studio/runs/artifacts/PAPERPLAN1/c6_reff/reff_numbers.json","w",encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=2)
print("\nwrote reff_numbers.json")
