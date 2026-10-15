# -*- coding: utf-8 -*-
"""Print the M2 attenuation table from chain32.csv (n=32 sorties)."""
import csv, statistics as st, sys

R = list(csv.DictReader(open(
    "C:/Users/joy46/Documents/Projects/aeroplan-studio/matlab/gammadiag/chain32.csv")))


def c(k):
    return [float(r[k]) for r in R]


def m(k):
    return st.median(c(k))


def iqr(k):
    v = sorted(c(k))
    n = len(v)
    return v[n // 4], v[3 * n // 4]


def line(k, f="{:.3f}"):
    lo, hi = iqr(k)
    return f"{f.format(m(k))}  [IQR {f.format(lo)}-{f.format(hi)}]"


print("=" * 78)
print("A. DISCRIMINANT  -- is the achieved gamma-dot sitting on the physical wall?")
print("=" * 78)
print(f"  measured |gamma-dot|  median      : {line('gdot_act_absmed','{:.2f}')} deg/s")
print(f"  measured |gamma-dot|  p90         : {line('gdot_act_p90','{:.2f}')} deg/s")
print(f"  measured |gamma-dot|  p99         : {line('gdot_act_p99','{:.2f}')} deg/s")
print(f"  measured |gamma-dot|  max         : {line('gdot_act_max','{:.2f}')} deg/s")
print(f"  ceiling  +(nz=3.0)                : {line('ceil_up','{:.2f}')} deg/s")
print(f"  ceiling  -(nz=-1.0)               : {line('ceil_dn','{:.2f}')} deg/s")
print(f"  frac of ticks |gdot| > 0.8*ceiling: {line('frac_at_ceiling')}")
print(f"  frac of ticks |gdot| >     ceiling: {line('frac_over_ceiling')}")
print()
print(f"  MEASURED n_z (= -IMU.AccZ/g)  median : {line('nz_meas_med','{:.2f}')}")
print(f"  MEASURED n_z  p99                    : {line('nz_meas_p99','{:.2f}')}")
print(f"  MEASURED n_z  p01                    : {line('nz_meas_p01','{:.2f}')}")
print(f"  MEASURED n_z  max                    : {line('nz_meas_max','{:.2f}')}")
print(f"  MEASURED n_z  min                    : {line('nz_meas_min','{:.2f}')}")
print(f"  frac of ticks n_z outside [-1,3]     : {line('frac_nz_out')}")
print()
print("=" * 78)
print("B. ATTENUATION TABLE -- one row per node of the signal chain")
print("   'amplitude' = median |commanded gamma-rate| implied at that node,")
print("   i.e. (command - current gamma)/pn_lead, in deg/s, so nodes compare.")
print("=" * 78)
hdr = f"{'node':38s} {'in':>9s} {'out':>9s} {'ratio':>7s} {'trigger%':>9s}"
print(hdr)
print("-" * 78)


def row(name, kin, kout, ktrig=None, fin=None, fout=None):
    vin = m(kin) if isinstance(kin, str) else kin
    vout = m(kout) if isinstance(kout, str) else kout
    tr = f"{100*m(ktrig):8.2f}%" if ktrig else "       --"
    print(f"{name:38s} {vin:9.2f} {vout:9.2f} {vout/vin if vin else float('nan'):7.3f} {tr:>9s}")


row("1  pure PN longitudinal demand", "gdot_pn", "gdot_pn")
row("1b node nz clamp  [-1.0, 3.0]", "gdot_pn", "gdot_nzcl", "nz_node_bite")
row("2  LOS floor  min(cmd, gam_los)", "gdot_cmd_pn", "gdot_cmd_los", "los_bite")
row("4  node pitch clamp [-80,+45]", "gdot_cmd_los", "gdot_cmd_los", "node_bite")
row("5  firmware clamp -80*|cos roll|", "gdot_cmd_los", "gdot_cmd_los", "fw_bite")
row("6+7 plant: cmd -> achieved gdot", "gdot_cmd_los", "gdot_act_absmed")
print("-" * 78)
print()
print("  node nz clamp  low side (nz<-1) : {}".format(line('nz_node_lo')))
print("  node nz clamp high side (nz> 3) : {}".format(line('nz_node_hi')))
print("  LOS floor extra steepness when it bites: {} deg".format(line('los_extra_deg', '{:.2f}')))
print("  node pitch clamp eats: {} deg".format(line('node_eat_deg', '{:.2f}')))
print("  firmware clamp eats : {} deg".format(line('fw_eat_deg', '{:.2f}')))
print("  firmware pitch floor -80*|cos roll| : median {} deg, p95 {} deg".format(
    f"{m('fw_limmin'):.1f}", f"{m('fw_limmin_p95'):.1f}"))
print("  |roll| measured: median {} deg, p90 {} deg".format(
    f"{m('roll_abs'):.1f}", f"{m('roll_abs_p90'):.1f}"))
print()
print("=" * 78)
print("C. NODE 3 -- alpha: node's low-pass estimate vs the true instantaneous value")
print("=" * 78)
print(f"  alpha_node (tau=1.75 s low-pass) median : {line('alpha_node','{:.2f}')} deg")
print(f"  alpha_true = pitch - gamma       median : {line('alpha_true_med','{:.2f}')} deg")
print(f"  firmware's own AOA estimate      median : {line('aoa_fw_med','{:.2f}')} deg")
print(f"  |alpha_node - alpha_raw| median          : {line('alpha_lag_abs','{:.2f}')} deg")
print(f"  |alpha_node - alpha_raw| p95             : {line('alpha_lag_p95','{:.2f}')} deg")
print()
print("=" * 78)
print("D. NODE 6 -- attitude loop tracking (measured, both sides)")
print("=" * 78)
print(f"  commanded pitch (ATT.DesPitch)  median : {line('despitch','{:.2f}')} deg")
print(f"  achieved  pitch (ATT.Pitch)     median : {line('pitch_meas','{:.2f}')} deg")
print(f"  |DesPitch - Pitch|              median : {line('track_err_deg','{:.2f}')} deg")
print(f"  |DesPitch - Pitch|              p90    : {line('track_err_p90','{:.2f}')} deg")
print(f"  pitch-rate loop |target| median        : {line('pid_tar_med','{:.2f}')} deg/s")
print(f"  pitch-rate loop |target| p95           : {line('pid_tar_p95','{:.2f}')} deg/s")
print(f"  pitch-rate loop |achieved| median      : {line('pid_act_med','{:.2f}')} deg/s")
print(f"  frac ticks rate target at RMAX(90 d/s) : {line('pid_sat')}")
print(f"  frac ticks elevator saturated          : {line('elev_sat')}")
print()
print("=" * 78)
print("E. context")
print("=" * 78)
print(f"  gamma median {line('gam_med','{:.1f}')} deg,  gamma p02 {line('gam_min','{:.1f}')} deg")
print(f"  V median {line('V_med','{:.1f}')} m/s")
print(f"  terminal-freeze fraction of PN window {line('froze_frac')}")
print(f"  replay self-check |replay - logged| med {line('chk_med','{:.3f}')} deg")
print(f"  ticks per sortie {line('n_tick','{:.0f}')}")
