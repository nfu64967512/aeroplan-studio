#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A4b: validate the ALREADY-CALIBRATED drag polar (scripts/research/predictor/sim.py
lines 5-13, calibrated on RPK/RPL) against the RRD/RRE dives, WITH the measured
load factor in the induced-drag term, then use it to bound longitudinal time authority.

Constants copied verbatim from scripts/research/predictor/sim.py:5-8:
    G=9.80665; M=22.0; W=M*G; TFULL=M*G/0.7
    S=0.45; B=1.88; AR=B*B/S; E=0.9; PIEAR=pi*E*AR; CD0=0.01500
"""
from __future__ import annotations
import math, os, pickle, sys, statistics as st
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

G = 9.80665; M = 22.0; W = M*G; TFULL = M*G/0.7
S = 0.45; B = 1.88; AR = B*B/S; E = 0.9; PIEAR = math.pi*E*AR
CD0 = 0.01500

def rho(h): return 1.225*(1.0-2.25577e-5*max(h, 0.0))**4.2586

def drag(V, h, n):
    qS = 0.5*rho(h)*S*V*V
    CL = n*W/max(qS, 1e-6)
    return qS*(CD0 + CL*CL/PIEAR)

ticks = pickle.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                      "a3_ticks.pkl"), "rb"))

def smooth_rate(tk, k, key, hw=10):
    a, b = max(0, k-hw), min(len(tk)-1, k+hw)
    dt = tk[b]["t"]-tk[a]["t"]
    return (tk[b][key]-tk[a][key])/dt if dt > 1e-6 else 0.0

print("== validation on RRD/RRE: integrate dV/dt with measured gamma & load factor ==")
print("   n_load from measured gamma-rate only (vertical plane): n = cos(gam) + V*gdot/g")
rows = []
for tag, tk in ticks:
    live = [x for x in tk if not x["frozen"] and x["h"] > 100]
    if len(live) < 200:
        continue
    V = live[0]["V"]; errs = []; nlist = []
    for k in range(len(live)-1):
        x = live[k]; dt = live[k+1]["t"]-x["t"]
        if not (0.01 < dt <= 0.3):
            continue
        gdot = math.radians(smooth_rate(live, k, "gam"))
        gam = math.radians(x["gam"])
        n = math.cos(gam) + x["V"]*gdot/G
        nlist.append(n)
        T = x["thr"]*TFULL
        V += ((T - drag(V, x["h"], abs(n)))/M - G*math.sin(gam))*dt
        errs.append(V - live[k+1]["V"])
    rms = math.sqrt(sum(e*e for e in errs)/len(errs))
    rows.append((tag, live[0]["V"], live[-1]["V"], V, rms,
                 st.median(nlist), max(nlist), min(nlist)))
    print(f"   {tag:<11s} V0 {live[0]['V']:6.1f}  Vend_meas {live[-1]['V']:6.1f}  "
          f"Vend_sim {V:6.1f}  err {V-live[-1]['V']:+7.1f}  rms {rms:6.1f}   "
          f"n_med {st.median(nlist):5.2f} [{min(nlist):5.2f},{max(nlist):5.2f}]")
d = [r[3]-r[2] for r in rows]
print(f"   Vend err: med {st.median(d):+.2f}  mean {st.mean(d):+.2f}  sd {st.pstdev(d):.2f}"
      f"   pooled rms {st.mean([r[4] for r in rows]):.2f} m/s   n={len(rows)}")
