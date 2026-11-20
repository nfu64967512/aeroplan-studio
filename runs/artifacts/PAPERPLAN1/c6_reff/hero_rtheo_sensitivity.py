# -*- coding: utf-8 -*-
"""C6 evidence 5: HERO's tabulated R_theo=75 m vs the geometrically consistent Va^2/(g tan phi)=78.92 m,
and the W -> R_eff -> skip-k sensitivity table for the paper."""
import math, sys, io, json
sys.path.insert(0, r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
from hv_dccpp.platform import PLATFORM_PIONEER, PLATFORM_HERO, G_MPS2
from hv_dccpp.dubins import skip_k

def reff(rplan, rtheo, va, w):  return max(rplan, rtheo*(1.0+w/va)**2)
def bank_on_circle(Rg, va, w):  # tailwind phase: |a| = (Va+W)^2 / Rg
    return math.degrees(math.atan(((va+w)**2/Rg)/G_MPS2))

print("=== HERO R_theo: table value vs geometrically consistent value ===")
h = PLATFORM_HERO
va, phi, rpl = h.cruise_airspeed_mps, h.max_bank_angle_deg, h.turn_radius_planning_m
r_tab, r_con = h.turn_radius_theoretical_m, va*va/(G_MPS2*math.tan(math.radians(phi)))
print(f"  V_a={va} m/s, ROLL_LIMIT_DEG=phi_max={phi} deg")
print(f"  table R_theo          = {r_tab:.2f} m  -> implied phi = {math.degrees(math.atan(va*va/(G_MPS2*r_tab))):.2f} deg  (ABOVE the {phi:.0f} deg limit)")
print(f"  consistent Va^2/(g tan phi) = {r_con:.2f} m  (table is {100*(r_tab-r_con)/r_con:+.2f}% -> passes the +/-10% validator at platform.py:249-253)")
for w in (0.0, 5.0, 8.0):
    Rt = reff(rpl, r_tab, va, w); Rc = reff(rpl, r_con, va, w)
    print(f"   W={w:4.1f}: R_eff(table R_theo)={Rt:7.2f} m (bank needed on that ground circle {bank_on_circle(Rt,va,w):5.2f} deg) | "
          f"R_eff(consistent)={Rc:7.2f} m (bank {bank_on_circle(Rc,va,w):5.2f} deg) | "
          f"W*={va*(math.sqrt(rpl/r_tab)-1):.3f} vs {va*(math.sqrt(rpl/r_con)-1):.3f}")
print(f"  -> at the paper's W=5 m/s the CONSISTENT R_theo would give R_eff={reff(rpl,r_con,va,5.0):.2f} m > R_plan={rpl:.0f} m,")
print(f"     i.e. the trochoid branch WOULD bind for HERO; skip_k(s=97) stays {skip_k(97.0, reff(rpl,r_con,va,5.0))} either way.")

print("\n=== paper sensitivity table: W -> R_eff -> skip-k (s = 97.0 m) ===")
print("   W[m/s] |  A R_eff  k_A  |  B R_eff  k_B")
rows = {}
for w in (0.0, 3.0, 5.0, 6.0, 8.0, 10.0, 12.0):
    a = PLATFORM_PIONEER.r_eff(w); b = PLATFORM_HERO.r_eff(w)
    ka, kb = skip_k(97.0, a), skip_k(97.0, b)
    print(f"   {w:5.1f}  | {a:8.2f}  {ka:3d}  | {b:8.2f}  {kb:3d}")
    rows[f"W{w:g}"] = {"A_R_eff_m": round(a,2), "k_A": ka, "B_R_eff_m": round(b,2), "k_B": kb}
with open(r"C:/Users/joy46/Documents/Projects/aeroplan-studio/runs/artifacts/PAPERPLAN1/c6_reff/hero_rtheo_sensitivity.json","w",encoding="utf-8") as f:
    json.dump({"rows": rows, "hero_R_theo_table_m": r_tab, "hero_R_theo_consistent_m": round(r_con,3)}, f, ensure_ascii=False, indent=2)
print("\nwrote hero_rtheo_sensitivity.json")
