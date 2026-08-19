#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A7: the decisive numbers for plan 3."""
from __future__ import annotations
import math, os, pickle, sys, json, statistics as st
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ditcg"))
import truth32                                        # noqa: E402
G = 9.80665; M = 22.0; W = M*G; TFULL = M*G/0.7
S = 0.45; B = 1.88; AR = B*B/S; EO = 0.9; PIEAR = math.pi*EO*AR; CD0 = 0.01500
def rho(h): return 1.225*(1.0-2.25577e-5*max(h, 0.0))**4.2586

print("== (i) is there ANY drag-modulation lever?  induced vs parasite drag ==")
for V, h in ((190, 1500), (150, 3000), (215, 300)):
    qS = 0.5*rho(h)*S*V*V
    for n in (1.0, 2.0, 3.0):
        CL = n*W/qS
        cdi = CL*CL/PIEAR
        print(f"   V={V:3d} h={h:4d} n={n:.0f}:  qS={qS:8.0f} N  CL={CL:.4f}  "
              f"CDi={cdi:.6f}  CDi/CD0={cdi/CD0*100:6.2f}%  "
              f"D={qS*(CD0+cdi):7.1f} N  (+{qS*cdi/(qS*CD0)*100:5.2f}% over parasite)")

print("\n== (ii) energy margin at release: lowest terminal speed physically reachable ==")
print("   E_td_min = E0 - integral(D/(m g cos gam)) over d0, evaluated at idle on the")
print("   straight LOS line with the polar re-validated in a4b.")
def min_vtd(d0, h0, V0, ds=5.0):
    L = math.hypot(d0, h0); gam = math.atan2(h0, d0)
    V, s = V0, 0.0
    while s < L:
        h = h0*(1.0-s/L)
        qS = 0.5*rho(h)*S*V*V
        CL = W*math.cos(gam)/max(qS, 1e-6)
        D = qS*(CD0+CL*CL/PIEAR)
        acc = (-D)/M + G*math.sin(gam)
        V = math.sqrt(max(900.0, V*V+2*acc*ds)); s += ds
    return V
recs = truth32.load()
a1 = {r["tag"]: r for r in json.load(open(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "a1_rows.json")))}
mrg = []
print("   tag         V0     Vtd_meas  Vtd_min(idle)   margin[m/s]  E_margin[m]")
for d in recs:
    vm = min_vtd(d["r_m"], d["z_m"], d["V_ms"])
    meas = a1[d["tag"]]["Vtd"]
    em = (meas**2-vm**2)/(2*G)
    mrg.append((meas-vm, em))
    print(f"   {d['tag']:<11s}{d['V_ms']:7.1f} {meas:11.1f} {vm:14.1f} "
          f"{meas-vm:12.1f} {em:12.0f}")
print(f"   speed margin: med {st.median([m[0] for m in mrg]):+.1f} m/s   "
      f"energy margin: med {st.median([m[1] for m in mrg]):+.0f} m  "
      f"(p10 {sorted(m[1] for m in mrg)[3]:+.0f}, p90 {sorted(m[1] for m in mrg)[-4]:+.0f})")

print("\n== (iii) hurry authority: straight-in, constant throttle ==")
def fly_const(thr, d0=6140.0, h0=3100.0, V0=186.0, ds=5.0):
    L = math.hypot(d0, h0); gam = math.atan2(h0, d0); V, s, t = V0, 0.0, 0.0
    while s < L:
        h = h0*(1.0-s/L)
        qS = 0.5*rho(h)*S*V*V
        CL = W*math.cos(gam)/max(qS, 1e-6)
        D = qS*(CD0+CL*CL/PIEAR)
        acc = (thr*TFULL-D)/M + G*math.sin(gam)
        Vn = math.sqrt(max(900.0, V*V+2*acc*ds))
        t += ds/(0.5*(V+Vn)); V = Vn; s += ds
    return t, V
base_t, base_v = fly_const(0.0)
print(f"   thr   t[s]    dt vs idle   V_end[m/s]")
for thr in (0.0, 0.05, 0.10, 0.15, 0.20, 0.30, 0.50, 0.75, 1.0):
    t, v = fly_const(thr)
    print(f"   {thr:4.2f} {t:7.2f} {t-base_t:11.2f} {v:12.1f}")

print("\n== (iv) RRE-only spread (the batch the 7.6 s figure comes from) ==")
by_run = {}
for d in recs: by_run.setdefault(d["run"], []).append(d)
for grp, runs in (("RRD", [f"RRD{i}" for i in range(1,9)]),
                  ("RRE", [f"RRE{i}" for i in range(1,9)]),
                  ("all", sorted(by_run))):
    dr = [abs(by_run[r][0]["t_release_s"]-by_run[r][1]["t_release_s"]) for r in runs]
    dt = [abs(by_run[r][0]["t_touch_s"]-by_run[r][1]["t_touch_s"]) for r in runs]
    dd = [abs(by_run[r][0]["t_wall_s"]-by_run[r][1]["t_wall_s"]) for r in runs]
    print(f"   {grp}: dt_release med {st.median(dr):6.2f} max {max(dr):6.2f} | "
          f"dt_touch med {st.median(dt):6.2f} max {max(dt):6.2f} | "
          f"dt_divetime med {st.median(dd):6.2f} max {max(dd):6.2f}")

print("\n== (v) what the longitudinal envelope can absorb ==")
DT_DELAY = {36: 3.30, 40: 5.08, 45: 7.48, 52: 11.02}    # from a5 (idle, level-then-dive)
for gmax, dly in DT_DELAY.items():
    for vcap, hurry in ((210.0, None),):
        pass
print("   delay (level-then-dive at gamma_max, idle):")
for gmax, dly in DT_DELAY.items():
    print(f"      gamma_max {gmax}: +{dly:5.2f} s")
print("   hurry (throttle up on the straight line, capped by terminal speed):")
for vcap in (190.0, 210.0, 230.0, 260.0):
    best = None
    for i in range(0, 101):
        thr = i/100.0
        t, v = fly_const(thr)
        if v <= vcap: best = (thr, t, v)
    print(f"      V_end <= {vcap:5.1f}:  thr {best[0]:4.2f}  t {best[1]:6.2f} s  "
          f"(-{base_t-best[1]:5.2f} s)  V_end {best[2]:6.1f}")
