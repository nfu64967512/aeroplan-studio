#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A6: (1) per-aircraft dispersion of the specific-energy profile E(d)
       (2) glide gradient: geometric requirement h/d vs achieved tan|gam|
       (3) ENERGY glide gradient: required (E-E*)/d vs available at idle D/(m g cos gam)
"""
from __future__ import annotations
import math, os, pickle, sys, json, statistics as st
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
G = 9.80665; M = 22.0; W = M*G
S = 0.45; B = 1.88; AR = B*B/S; E_OSW = 0.9; PIEAR = math.pi*E_OSW*AR
CD0 = 0.01500
VSTAR = 185.0                      # accuracy-safe terminal speed (median of the 28 clean)
ESTAR = VSTAR**2/(2*G)

def rho(h): return 1.225*(1.0-2.25577e-5*max(h, 0.0))**4.2586
def drag(V, h, n=1.0):
    qS = 0.5*rho(h)*S*V*V
    CL = n*W/max(qS, 1e-6)
    return qS*(CD0 + CL*CL/PIEAR)

ticks = pickle.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                      "a3_ticks.pkl"), "rb"))
a1 = {r["tag"]: r for r in json.load(open(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "a1_rows.json")))}
FAIL = {"RRD1-U2", "RRD4-U1", "RRD2-U1", "RRD7-U1"}

GRID = [6000, 5000, 4000, 3000, 2000, 1500, 1000, 500, 200]
prof = {g: [] for g in GRID}
prof_f = {g: [] for g in GRID}
rows = []
for tag, tk in ticks:
    live = [x for x in tk if not x["frozen"]]
    ser = []
    for x in live:
        Ei = x["h"] + x["V"]**2/(2*G)
        gm = math.radians(x["gam"])
        req_geo = x["h"]/max(x["d"], 1.0)
        ach_geo = math.tan(max(0.0, -gm))
        req_E = (Ei - ESTAR)/max(x["d"], 1.0)
        Dv = drag(x["V"], x["h"], 1.0)
        avail_E = Dv/(M*G*max(math.cos(gm), 0.3))
        ser.append((x["d"], Ei, req_geo, ach_geo, req_E, avail_E, x["V"], x["h"],
                    x["bind"]))
    for g in GRID:
        c = min(ser, key=lambda s: abs(s[0]-g))
        if abs(c[0]-g) < 250:
            (prof_f if tag in FAIL else prof)[g].append(c)
    # whole-dive margins
    mE = [s[5]-s[4] for s in ser if s[0] > 300]
    mG = [s[3]-s[2] for s in ser if s[0] > 300]
    rows.append(dict(tag=tag, mE_med=st.median(mE), mE_min=min(mE),
                     mG_med=st.median(mG), mG_min=min(mG),
                     frac_infeasible=sum(1 for m in mE if m < 0)/len(mE),
                     zres=a1[tag]["zres"], r_td=a1[tag]["r_td"]))

print("== E(d) across the fleet (clean 28) : specific energy at fixed horizontal range ==")
print("    d[m]    n   E_med    E_p10    E_p90   E_max-E_min   V_med   h_med   bindF")
def pct(v, q):
    v = sorted(v); i = q*(len(v)-1); lo = int(i)
    return v[lo] if lo >= len(v)-1 else v[lo]+(i-lo)*(v[lo+1]-v[lo])
for g in GRID:
    c = prof[g]
    if not c: continue
    Ev = [x[1] for x in c]
    print(f"   {g:5d} {len(c):4d} {st.median(Ev):7.0f} {pct(Ev,0.1):8.0f} {pct(Ev,0.9):8.0f}"
          f"   {max(Ev)-min(Ev):10.0f}  {st.median([x[6] for x in c]):6.1f}"
          f"  {st.median([x[7] for x in c]):6.0f}"
          f"   {st.mean([1.0 if x[8] else 0.0 for x in c]):5.2f}")
print("   -- the 4 residual-altitude failures --")
for g in GRID:
    c = prof_f[g]
    if not c: continue
    Ev = [x[1] for x in c]
    print(f"   {g:5d} {len(c):4d} {st.median(Ev):7.0f} {min(Ev):8.0f} {max(Ev):8.0f}"
          f"   {max(Ev)-min(Ev):10.0f}  {st.median([x[6] for x in c]):6.1f}"
          f"  {st.median([x[7] for x in c]):6.0f}"
          f"   {st.mean([1.0 if x[8] else 0.0 for x in c]):5.2f}")

print("\n== glide gradients at fixed range (median over the clean 28 | the 4 failures) ==")
print("    d[m]   req_geo(h/d)  ach_geo(tan|g|)   margin |  req_E((E-E*)/d)  avail_E(D/mg cosg)  margin")
for g in GRID:
    c, f = prof[g], prof_f[g]
    if not c: continue
    def m(v, i): return st.median([x[i] for x in v])
    line = (f"   {g:5d}   {m(c,2):10.3f} {m(c,3):14.3f} {m(c,3)-m(c,2):9.3f} | "
            f"{m(c,4):14.3f} {m(c,5):18.3f} {m(c,5)-m(c,4):9.3f}")
    if f:
        line += (f"   ||F: {m(f,2):.3f} {m(f,3):.3f} {m(f,3)-m(f,2):+.3f} | "
                 f"{m(f,4):.3f} {m(f,5):.3f} {m(f,5)-m(f,4):+.3f}")
    print(line)

print("\n== per-aircraft margins over the dive (d>300 m) ==")
print("   tag        mE_med  mE_min  frac_E_infeasible   mG_med  mG_min   zres   r_td")
for r in sorted(rows, key=lambda r: r["mE_med"]):
    star = " *FAIL" if r["tag"] in FAIL else ""
    print(f"   {r['tag']:<11s}{r['mE_med']:7.3f} {r['mE_min']:7.3f} "
          f"{r['frac_infeasible']:16.3f}   {r['mG_med']:7.3f} {r['mG_min']:7.3f} "
          f"{r['zres']:6.1f} {r['r_td']:7.1f}{star}")
for k in ("mE_med","mE_min","frac_infeasible","mG_med","mG_min"):
    v=[r[k] for r in rows]
    print(f"   {k:<16s} med {st.median(v):8.3f}  min {min(v):8.3f}  max {max(v):8.3f}")
