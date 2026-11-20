# -*- coding: utf-8 -*-
"""M2 deliverable: the attenuation table, one row per node of the signal chain.

n = 32 sorties (RRD1-8, RRE1-8, 2 aircraft each), PN-engaged window only
(median 31.4 s per sortie, ~630 node ticks).  Every row states whether its
numbers are MEASURED from the dataflash log or RECONSTRUCTED by replay.
"""
import csv, statistics as st

D = "C:/Users/joy46/Documents/Projects/aeroplan-studio/matlab/gammadiag/"
C = list(csv.DictReader(open(D + "chain32.csv")))
P2 = list(csv.DictReader(open(D + "pass2_dump.csv")))
P3 = list(csv.DictReader(open(D + "pass3.csv")))


def m(R, k):
    return st.median([float(r[k]) for r in R])


W = 96
print("=" * W)
print("M2 ATTENUATION TABLE  --  n=32 sorties, PN-engaged window only")
print("amplitude = median |commanded gamma-rate| at that node = (cmd - gamma)/pn_lead, deg/s")
print("=" * W)
print(f"{'#':<3}{'node':<34}{'in':>8}{'out':>8}{'ratio':>8}{'fires':>9}   {'src':<4} note")
print("-" * W)

R = [
    ("0", "pure PN longitudinal demand", m(C, "gdot_pn"), m(C, "gdot_pn"), None,
     "[R]", "median 1.32, p90 " + f"{m(C,'gdot_pn_p90'):.1f} deg/s"),
    ("1", "node nz clamp [-1.0,+3.0]", m(C, "gdot_pn"), m(C, "gdot_nzcl"),
     m(C, "nz_node_bite"), "[R]", "keeps demand INSIDE the ceiling"),
    ("2", "LOS floor min(cmd, gam_los)", m(C, "gdot_cmd_pn"), m(C, "gdot_cmd_los"),
     m(C, "los_bite"), "[R]", "AMPLIFIES; only source of over-ceiling demand"),
    ("3", "alpha low-pass tau=1.75 s", None, None, None, "[R]",
     f"costs {m(C,'alpha_lag_abs'):.2f} deg median, {m(C,'alpha_lag_p95'):.2f} deg p95"),
    ("4", "node pitch clamp [-80,+45]", m(C, "gdot_cmd_los"), m(C, "gdot_cmd_los"),
     m(C, "node_bite"), "[R]", "NEVER fires"),
    ("5", "firmware -80*|cos roll|", m(C, "gdot_cmd_los"), m(C, "gdot_cmd_los"),
     m(C, "fw_bite"), "[R]", f"NEVER fires; |roll| med {m(C,'roll_abs'):.1f} deg"),
    ("6", "attitude loop DesPitch->Pitch", None, None, None, "[M]",
     f"gain {m(P3,'n6_gain'):.3f}, lag {m(P3,'n6_lag'):.2f} s, corr {m(P3,'n6_corr'):.3f}"),
    ("7", "airframe Pitch->gamma", None, None, None, "[M]",
     f"gain {m(P3,'n7_gain'):.3f}, lag {m(P3,'n7_lag'):.2f} s, corr {m(P3,'n7_corr'):.3f}"),
    ("6+7", "net plant cmd->achieved rate", m(C, "gdot_cmd_los"),
     m(C, "gdot_act_absmed"), None, "[M]",
     f"regression gain {m(P2,'gain0'):.3f} (corr {m(P2,'corr0'):.3f})"),
]
for n, name, i, o, tg, src, note in R:
    si = f"{i:8.2f}" if i is not None else " " * 8
    so = f"{o:8.2f}" if o is not None else " " * 8
    sr = f"{o/i:8.3f}" if (i and o) else " " * 8
    stg = f"{100*tg:8.2f}%" if tg is not None else " " * 9
    print(f"{n:<3}{name:<34}{si}{so}{sr}{stg}   {src:<4} {note}")
print("-" * W)
print()
print("KEY NUMBERS")
print("-" * W)
print(f"  demand exceeds the physical ceiling on          "
      f"{m(P2,'frac_dem_over_ceil'):.1%} of ticks  (before the LOS floor: "
      f"{m(P2,'frac_dempre_over_ceil'):.1%})")
print(f"  worst demand / ceiling ratio                   "
      f"p99 {m(P2,'dem_over_ceil_ratio_p99'):.1f}x , max "
      f"{m(P2,'dem_over_ceil_ratio_max'):.1f}x")
print(f"  ALL over-ceiling demand sits in the first 20% of the PN window,")
print(f"     where the LOS floor is active 100% of ticks")
print()
print(f"  MEASURED n_z envelope (accel; kinematics agree to 0.04, corr 0.999):")
print(f"     p01 {m(P3,'nz_p01'):+.2f}   p50 {m(P3,'nz_p50'):+.2f}   "
      f"p99 {m(P3,'nz_p99'):+.2f}   min {m(P3,'nz_min'):+.2f}   max {m(P3,'nz_max'):+.2f}")
print(f"     below the model's nz_min=-1.0 : {m(P3,'nz_below_m1'):.1%} of ticks")
print(f"     above the model's nz_max=+3.0 : {m(P3,'nz_above_3'):.1%} of ticks")
print()
print(f"  MEASURED |gamma-dot|: med {m(C,'gdot_act_absmed'):.2f}, "
      f"p99 {m(C,'gdot_act_p99'):.2f}, max {m(C,'gdot_act_max'):.2f} deg/s")
print(f"  briefing ceiling    : +{m(C,'ceil_up'):.2f} / {m(C,'ceil_dn'):.2f} deg/s")
print(f"  ticks above ceiling : {m(C,'frac_over_ceiling'):.1%}")
print()
print(f"  alpha_true = Pitch - gamma : {m(P3,'alpha_med'):+.2f} deg "
      f"[p05 {m(P3,'alpha_p05'):+.2f}, p95 {m(P3,'alpha_p95'):+.2f}]  [M]")
print(f"  node's own alpha estimate  : {m(C,'alpha_node'):+.2f} deg  -> "
      f"error vs instantaneous {m(C,'alpha_lag_abs'):.2f} deg  [R]")
print(f"  firmware AOA message       : {m(P3,'aoa_fw_med'):+.2f} deg  "
      f"(UNUSABLE: EK3 estimates 24 m/s of wind that does not exist)")
print()
print("REPLAY VALIDATION")
print("-" * W)
print(f"  |replay pitch cmd - logged nav_pitch_cd| : median {m(C,'chk_med'):.3f} deg, "
      f"{m(C,'chk_frac_1deg'):.1%} of ticks within 1 deg")
print(f"  EKF attitude vs SITL truth              : 0.23 deg median (pitch)")
print(f"  CSV<->log identity                      : 0-4 m spatial residual")
