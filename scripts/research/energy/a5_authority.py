#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A5: longitudinal TIME authority, and how much release-time spread it can absorb.

Quadrature along a PRESCRIBED vertical-plane profile (ground track held straight).
The guidance loop is NOT simulated -- gamma(d) is imposed -- so the fidelity gate
that killed the closed-loop 3-DOF integrator (ditcg/README.md) does not apply.
Polar = scripts/research/predictor/sim.py:5-13, re-validated on RRD/RRE in a4b.
"""
from __future__ import annotations
import math, json, os, sys, statistics as st
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ditcg"))
import truth32                                       # noqa: E402

G = 9.80665; M = 22.0; W = M*G; TFULL = M*G/0.7
S = 0.45; B = 1.88; AR = B*B/S; E = 0.9; PIEAR = math.pi*E*AR
CD0 = 0.01500
PN_VMAX = 210.0

def rho(h): return 1.225*(1.0-2.25577e-5*max(h, 0.0))**4.2586

def drag(V, h, n=1.0):
    qS = 0.5*rho(h)*S*V*V
    CL = n*W/max(qS, 1e-6)
    return qS*(CD0 + CL*CL/PIEAR)

def fly(gam_of, thr_of, d0=6140.0, h0=3100.0, V0=186.0, dt=0.02, vfloor=90.0,
        nz_min=-1.0, nz_max=3.0):
    """March forward with an IMPOSED gamma schedule (deg, +down), rate-limited by nz."""
    h, d, V, t, path = h0, d0, V0, 0.0, 0.0
    gam = 0.0
    while h > 0.0 and t < 200.0 and d > -500.0:
        gtgt = math.radians(gam_of(d, h))
        # nz-limited gamma rate:  gdot = (nz - cos gam)*g/V   (gam +down => cos gam)
        gd_max = (nz_max - math.cos(gam))*G/V      # pull-up (less dive) .. sign care
        gd_min = (nz_min - math.cos(gam))*G/V
        want = (gtgt - gam)/dt
        # gam is +down, so increasing gam = pushing over = negative nz direction
        gdot = max(-gd_max, min(-gd_min, want))
        gam += gdot*dt
        thr = thr_of(V, d, h)
        n = abs(math.cos(gam) - V*gdot/G)
        V += ((thr*TFULL - drag(V, h, max(n, 0.05)))/M + G*math.sin(gam))*dt
        V = max(vfloor, V)
        h -= V*math.sin(gam)*dt
        d -= V*math.cos(gam)*dt
        path += V*dt
        t += dt
    return dict(t=t, V=V, d=d, path=path)

def pn_thr(V, d, h):
    R = math.hypot(d, h)
    return max(0.0, min(1.0, 0.36 + 0.030*(min(PN_VMAX, R/max(R/max(V,1e-3),1e-3)) - V)))

straight = lambda d, h: math.degrees(math.atan2(3100.0, 6140.0))
def two_seg(gmax_deg):
    gmax = math.radians(gmax_deg)
    def f(d, h):
        d_break = h/math.tan(gmax)
        return 0.0 if d > d_break else gmax_deg
    return f

print("== baseline sanity: straight-in from d0=6140 h0=3100 V0=186 ==")
print("   measured dive time: t_wall med 41.20 s, t_sim med 37.90 s (truth32)")
for nm, th in (("thr=1.0 (max hurry)", lambda V,d,h: 1.0),
               ("thr=0.36 (trim)",     lambda V,d,h: 0.36),
               ("thr=0.0  (idle)",     lambda V,d,h: 0.0)):
    r = fly(straight, th)
    print(f"   straight-in {nm:22s} t={r['t']:6.2f} s  Vend={r['V']:6.1f}  path={r['path']:6.0f}")

print("\n== LATE authority: level-then-dive at gamma_max, idle throttle ==")
base = fly(straight, lambda V,d,h: 0.0)
print(f"   reference (straight-in, idle) t = {base['t']:.2f} s, path {base['path']:.0f} m")
for gmax in (30, 36, 40, 45, 52, 60, 70):
    r = fly(two_seg(gmax), lambda V,d,h: 0.0)
    print(f"   gamma_max {gmax:3d} deg  t={r['t']:6.2f} s  (+{r['t']-base['t']:5.2f} s)  "
          f"Vend={r['V']:6.1f}  path={r['path']:6.0f} (+{r['path']-base['path']:5.0f} m)")

print("\n   -- with a speed floor enforced during the level leg (idle, vfloor) --")
for vf in (150.0, 120.0, 95.0):
    for gmax in (40, 52, 60):
        r = fly(two_seg(gmax), lambda V,d,h: 0.0, vfloor=vf)
        print(f"   vfloor {vf:5.1f}  gamma_max {gmax:3d}  t={r['t']:6.2f} s "
              f"(+{r['t']-base['t']:5.2f} s)  Vend={r['V']:6.1f}")

print("\n== ideal (nz-unlimited) bang-bang path bound  L_max = d0 + h0*tan(gmax/2) ==")
R0 = math.hypot(6140.0, 3100.0)
for gmax in (36, 40, 45, 52, 60, 70):
    Lm = 6140.0 + 3100.0*math.tan(math.radians(gmax)/2.0)
    print(f"   gamma_max {gmax:3d}: L_max {Lm:6.0f} m  vs straight {R0:6.0f} m  "
          f"(+{Lm-R0:5.0f} m = +{(Lm-R0)/186.0:5.2f} s at 186 m/s)")

# ---------- release-time / touchdown-time spread, per sortie pair ----------
print("\n== spread bookkeeping (each run = 2 aircraft) ==")
recs = truth32.load()
by_run = {}
for d in recs:
    by_run.setdefault(d["run"], []).append(d)
drel, dtd, ddive = [], [], []
print("   run    dt_release  dt_touch  dt_divetime(wall)  dt_divetime(sim)")
for run in sorted(by_run):
    a, b = by_run[run]
    dr = abs(a["t_release_s"]-b["t_release_s"])
    dtt = abs(a["t_touch_s"]-b["t_touch_s"])
    dd = abs(a["t_wall_s"]-b["t_wall_s"])
    ds = abs(a["t_sim_s"]-b["t_sim_s"])
    drel.append(dr); dtd.append(dtt); ddive.append(dd)
    print(f"   {run:<6s} {dr:9.2f} {dtt:9.2f} {dd:15.2f} {ds:16.2f}")
print(f"   median  dt_release {st.median(drel):.2f} s | dt_touch {st.median(dtd):.2f} s "
      f"| dt_divetime {st.median(ddive):.2f} s")
print(f"   max     dt_release {max(drel):.2f} s | dt_touch {max(dtd):.2f} s "
      f"| dt_divetime {max(ddive):.2f} s")

def pe(a, b):
    ma, mb = st.mean(a), st.mean(b)
    num = sum((x-ma)*(y-mb) for x, y in zip(a, b))
    den = math.sqrt(sum((x-ma)**2 for x in a)*sum((y-mb)**2 for y in b))
    return num/den if den else float("nan")
print(f"   corr(dt_release, dt_touch) = {pe(drel, dtd):+.3f}   n={len(drel)}")

# signed decomposition:  dt_touch = dt_release + dt_divetime  (exactly)
sd_rel = [a["t_release_s"]-b["t_release_s"] for a, b in
          (by_run[r] for r in sorted(by_run))]
sd_dv = [a["t_wall_s"]-b["t_wall_s"] for a, b in (by_run[r] for r in sorted(by_run))]
sd_td = [a["t_touch_s"]-b["t_touch_s"] for a, b in (by_run[r] for r in sorted(by_run))]
print(f"   signed:  sd(dt_release) {st.pstdev(sd_rel):6.2f}  sd(dt_divetime) "
      f"{st.pstdev(sd_dv):6.2f}  sd(dt_touch) {st.pstdev(sd_td):6.2f}")
print(f"   corr(signed release, signed dive-time) = {pe(sd_rel, sd_dv):+.3f}  "
      f"(negative = dive time already partially compensating)")

# ---------- how well does the throttle channel track its own time command? ----------
print("\n== the existing time channel: commanded dive duration t_rem0 vs achieved T ==")
print("   tag         t_rem0   T_wall   T_sim   err_wall(T-t_rem0)")
er = []
for d in sorted(recs, key=lambda x: x["t_rem0_s"]):
    e = d["t_wall_s"] - d["t_rem0_s"]
    er.append(e)
    print(f"   {d['tag']:<11s} {d['t_rem0_s']:7.2f} {d['t_wall_s']:8.2f} {d['t_sim_s']:7.2f} "
          f"{e:+11.2f}")
print(f"   commanded range {min(x['t_rem0_s'] for x in recs):.2f} .. "
      f"{max(x['t_rem0_s'] for x in recs):.2f} s  (span "
      f"{max(x['t_rem0_s'] for x in recs)-min(x['t_rem0_s'] for x in recs):.2f} s)")
print(f"   achieved  range {min(x['t_wall_s'] for x in recs):.2f} .. "
      f"{max(x['t_wall_s'] for x in recs):.2f} s  (span "
      f"{max(x['t_wall_s'] for x in recs)-min(x['t_wall_s'] for x in recs):.2f} s)")
print(f"   tracking err: med {st.median(er):+.2f}  min {min(er):+.2f}  max {max(er):+.2f}"
      f"  sd {st.pstdev(er):.2f}")
lo = [d for d in recs if d["t_rem0_s"] < 35]
hi = [d for d in recs if d["t_rem0_s"] > 50]
print(f"   asked to HURRY (t_rem0<35, n={len(lo)}): achieved med "
      f"{st.median([d['t_wall_s'] for d in lo]):.2f} s, shortfall med "
      f"{st.median([d['t_wall_s']-d['t_rem0_s'] for d in lo]):+.2f} s")
print(f"   asked to WAIT  (t_rem0>50, n={len(hi)}): achieved med "
      f"{st.median([d['t_wall_s'] for d in hi]):.2f} s, shortfall med "
      f"{st.median([d['t_wall_s']-d['t_rem0_s'] for d in hi]):+.2f} s")

# is dive_T common within a pair?  t_rem0_A - t_rem0_B should = -(t_relA - t_relB)
print("\n== consistency: is dive_T shared within a pair? ==")
for run in sorted(by_run):
    a, b = by_run[run]
    lhs = a["t_rem0_s"] - b["t_rem0_s"]
    rhs = -(a["t_release_s"] - b["t_release_s"])
    print(f"   {run:<6s} dt_rem0 {lhs:+8.2f}   -dt_release {rhs:+8.2f}   "
          f"resid {lhs-rhs:+7.2f}")
