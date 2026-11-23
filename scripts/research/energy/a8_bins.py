#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A8: range-binned profile of the longitudinal channel + unused path budget."""
from __future__ import annotations
import math, os, pickle, sys, json, statistics as st
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
G = 9.80665
ticks = pickle.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                      "a3_ticks.pkl"), "rb"))
a1 = {r["tag"]: r for r in json.load(open(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "a1_rows.json")))}
FAIL = {"RRD1-U2", "RRD4-U1", "RRD2-U1", "RRD7-U1"}
BINS = [(6200,5500),(5500,5000),(5000,4500),(4500,4000),(4000,3500),(3500,3000),
        (3000,2500),(2500,2000),(2000,1500),(1500,1000),(1000,500),(500,150)]

def table(sel, name):
    print(f"\n-- {name} (n={len(sel)} aircraft) --")
    print("   d_bin        thr_med  thr=0frac  V_med   gam_med  gamcmd_on  gamcmd_off "
          " bindF  dgam_bind  dE/dt   glide_ach  glide_req")
    for hi, lo in BINS:
        acc = []
        for tag, tk in sel:
            b = [x for x in tk if lo <= x["d"] < hi and not x["frozen"]]
            if b: acc.extend(b)
        if not acc: continue
        bd = [x for x in acc if x["bind"]]
        de = [x["dEdt"] for x in acc if x["dEdt"] == x["dEdt"]]
        print(f"   {hi:5d}-{lo:<5d} {st.median([x['thr'] for x in acc]):7.3f}"
              f" {sum(1 for x in acc if x['thr']<=0.001)/len(acc):9.3f}"
              f" {st.median([x['V'] for x in acc]):7.1f}"
              f" {st.median([x['gam'] for x in acc]):8.1f}"
              f" {st.median([x['gc_on'] for x in acc]):10.1f}"
              f" {st.median([x['gc_off'] for x in acc]):11.1f}"
              f" {len(bd)/len(acc):6.2f}"
              f" {(st.median([x['gc_off']-x['gc_on'] for x in bd]) if bd else 0.0):9.1f}"
              f" {(st.median(de) if de else float('nan')):7.1f}"
              f" {st.median([-math.tan(math.radians(x['gam'])) for x in acc]):10.3f}"
              f" {st.median([x['h']/max(x['d'],1) for x in acc]):10.3f}")

clean = [(t, k) for t, k in ticks if t not in FAIL and a1[t]["r_td"] < 100]
fail = [(t, k) for t, k in ticks if t in FAIL]
table(clean, "clean 26 (r_td<100, not a residual-alt failure)")
table(fail, "the 4 residual-altitude failures")

print("\n== unused vertical-plane path budget, per aircraft ==")
print("   L_max(gmax) = d0 + h0*tan(gmax/2)  (ideal bang-bang, nz-unlimited upper bound)")
print("   tag         d0    h0   R0_slant  path_meas  L_max(own gmax)  unused[m]  unused[s]"
      "   L_max(52deg) unused52[s]")
u1, u2 = [], []
for tag in sorted(a1):
    r = a1[tag]
    gm = math.radians(abs(r["gam_p05"]))
    Lm = r["d0"] + r["h0"]*math.tan(gm/2.0)
    L52 = r["d0"] + r["h0"]*math.tan(math.radians(52)/2.0)
    Vav = r["path"]/r["T"]
    un, un52 = Lm-r["path"], L52-r["path"]
    u1.append(un/Vav); u2.append(un52/Vav)
    print(f"   {tag:<11s}{r['d0']:5.0f} {r['h0']:5.0f} {r['R0']:9.0f} {r['path']:10.0f}"
          f" {Lm:16.0f} {un:10.0f} {un/Vav:10.2f} {L52:14.0f} {un52/Vav:11.2f}")
print(f"   unused (own gamma_max): med {st.median(u1):+.2f} s  "
      f"[{min(u1):+.2f}, {max(u1):+.2f}]")
print(f"   unused (gamma_max=52) : med {st.median(u2):+.2f} s  "
      f"[{min(u2):+.2f}, {max(u2):+.2f}]")
