"""Side-evidence: R_eff(W) = max{R_plan, R_theo (1+W/Va)^2}
   hv_dccpp/platform.py:296-301 (PlatformSpec.r_eff), wind.py:312-315 (thin wrapper),
   W* = Va(sqrt(R_plan/R_theo)-1)  platform.py:303-305
Shows the wind term is INERT at the paper's own 5 m/s and only bites above W*.
"""
import sys
from pathlib import Path
sys.path.insert(0, r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
from hv_dccpp.platform import PLATFORM_PIONEER as A, PLATFORM_HERO as B  # noqa: E402

print(f"{'W m/s':>6} | {'A R_eff':>9} {'A raw':>8} {'clamped':>7} | {'B R_eff':>9} {'B raw':>8} {'clamped':>7}")
for w in (0, 2, 4, 5, 5.03, 5.17, 6, 8, 10, 12):
    ra = A.turn_radius_theoretical_m * (1 + w / A.cruise_airspeed_mps) ** 2
    rb = B.turn_radius_theoretical_m * (1 + w / B.cruise_airspeed_mps) ** 2
    print(f"{w:6.2f} | {A.r_eff(w):9.1f} {ra:8.1f} {'YES' if ra < A.turn_radius_planning_m else 'no':>7} "
          f"| {B.r_eff(w):9.1f} {rb:8.1f} {'YES' if rb < B.turn_radius_planning_m else 'no':>7}")
print()
print(f"W* Pioneer = {A.w_star():.3f} m/s   (R_plan {A.turn_radius_planning_m}, R_theo {A.turn_radius_theoretical_m}, Va {A.cruise_airspeed_mps})")
print(f"W* HERO    = {B.w_star():.3f} m/s   (R_plan {B.turn_radius_planning_m}, R_theo {B.turn_radius_theoretical_m}, Va {B.cruise_airspeed_mps})")
print()
print("=> at W = 5 m/s (the paper's planning wind) R_eff == R_plan for BOTH classes:")
print(f"   A: R_eff(5)={A.r_eff(5.0):.1f} == R_plan {A.turn_radius_planning_m}  (raw {A.turn_radius_theoretical_m*(1+5/A.cruise_airspeed_mps)**2:.1f})")
print(f"   B: R_eff(5)={B.r_eff(5.0):.1f} == R_plan {B.turn_radius_planning_m}  (raw {B.turn_radius_theoretical_m*(1+5/B.cruise_airspeed_mps)**2:.1f})")
print(f"   A: R_eff(8)={A.r_eff(8.0):.1f}   B: R_eff(8)={B.r_eff(8.0):.1f}  <- 8 m/s DOES move it")
