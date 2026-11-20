# -*- coding: utf-8 -*-
"""C6 evidence 4: which wind speed W_R actually feeds R_eff in the paper scenario
(single layer 0-3000 m AGL, 5 m/s from 270 deg -- see aeroplan_adapter.py:961 and
scripts/hv_sitl_scenarios.py:66 / runs/artifacts/HV4SITL*/scenario.json)."""
import sys, io, json, math
sys.path.insert(0, r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
from hv_dccpp.mission import WindLayer
from hv_dccpp.wind import WindField, r_eff_wind_speed, ground_speed, wind_vector
from hv_dccpp.platform import PLATFORM_PIONEER, PLATFORM_HERO

speed, dir_from = 5.0, 270.0
layers = (WindLayer(0.0, 3000.0, speed, dir_from, gust_mps=speed*1.3, z_ref="agl"),)
wf = WindField(layers, None, 0.0)
print(f"scenario wind: {speed} m/s from {dir_from} deg (west wind), single layer 0-3000 m AGL, gust {speed*1.3} m/s")
print(f"WindField.max_speed() = {wf.max_speed():.4f} m/s   (this is what ardupilot.export/recommended_params uses)")
out = {}
z_site = 0.0
for spec in (PLATFORM_PIONEER, PLATFORM_HERO):
    z_top = max(120.0, spec.min_fw_operation_agl_m)   # strips at 120 m AGL; floor = min_fw_operation_agl
    for pol in ("max_layer", "op_layer"):
        WR = r_eff_wind_speed(wf, spec, z_site, z_top, pol)
        R = spec.r_eff(WR)
        print(f"  [{spec.cls}] policy={pol:9s} z in [{z_site + spec.transition_cmd().h_tc_clear_m:.1f}, {z_top:.1f}] "
              f"-> W_R={WR:.4f} m/s -> R_eff={R:.2f} m  (R_plan={spec.turn_radius_planning_m:.0f}, "
              f"W*={spec.w_star():.3f} -> {'R_plan branch (wind correction INERT)' if R <= spec.turn_radius_planning_m + 1e-9 else 'trochoid branch ACTIVE'})")
        out[f"{spec.cls}_{pol}"] = {"W_R_mps": round(WR,4), "R_eff_m": round(R,3),
                                    "R_plan_m": spec.turn_radius_planning_m, "inert": R <= spec.turn_radius_planning_m + 1e-9}

print("\n=== ground speed along the survey headings (5 m/s from 270 = blowing toward +x / east) ===")
wxy = wind_vector(speed, dir_from)[0]
print(f"  wind blowing-toward vector = ({wxy[0]:+.3f}, {wxy[1]:+.3f}) m/s (ENU east,north)")
for spec in (PLATFORM_PIONEER, PLATFORM_HERO):
    va = spec.cruise_airspeed_mps
    for name, th_deg in (("east (tailwind)", 90.0), ("west (headwind)", 270.0), ("north (cross)", 0.0), ("south (cross)", 180.0)):
        vg = float(ground_speed(math.radians(th_deg), va, wxy)[0])
        print(f"  [{spec.cls}] V_a={va:.1f} track {name:16s} -> V_g={vg:6.3f} m/s")

with open(r"C:/Users/joy46/Documents/Projects/aeroplan-studio/runs/artifacts/PAPERPLAN1/c6_reff/wind_policy_readout.json","w",encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=2)
print("\nwrote wind_policy_readout.json")
