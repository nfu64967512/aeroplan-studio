#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A4: fit specific thrust/drag from measured energy rate, then bound the
longitudinal TIME authority.

Model (per unit mass, specific energy E = h + V^2/2g):
    dE/dt = (a*thr - b*sigma*V^2) * V / g
      a  [m/s^2] specific thrust at thr=1     sigma = rho/rho0 (ISA)
      b  [1/m]   specific drag coefficient
Fit is linear least squares on  y = (dE/dt)*g/V  vs  (thr, sigma*V^2).
"""
from __future__ import annotations
import math, os, pickle, sys, statistics as st
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
G = 9.80665

def sigma(h):
    h = max(0.0, min(11000.0, h))
    return (1.0 - 2.25577e-5 * h) ** 4.2559

ticks = pickle.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                      "a3_ticks.pkl"), "rb"))

X, Y, W = [], [], []
for tag, tk in ticks:
    for x in tk:
        if x["frozen"] or not (x["dEdt"] == x["dEdt"]):
            continue
        if x["V"] < 50 or x["h"] < 50:
            continue
        y = x["dEdt"] * G / x["V"]
        X.append((x["thr"], sigma(x["h"]) * x["V"] ** 2))
        Y.append(y)
print(f"fit samples n = {len(X)}")

# normal equations for y = a*x1 - b*x2  (2x2)
s11 = sum(x[0]*x[0] for x in X); s12 = sum(x[0]*x[1] for x in X)
s22 = sum(x[1]*x[1] for x in X)
t1  = sum(x[0]*y for x, y in zip(X, Y)); t2 = sum(x[1]*y for x, y in zip(X, Y))
det = s11*s22 - s12*s12
a = ( t1*s22 - t2*s12) / det
c = ( t2*s11 - t1*s12) / det      # c = -b
b = -c
res = [y - (a*x[0] + c*x[1]) for x, y in zip(X, Y)]
print(f"  a (specific thrust @thr=1) = {a:8.4f} m/s^2   ({a/G:.3f} g)")
print(f"  b (specific drag coeff)    = {b:.6e} 1/m")
print(f"  residual rms = {math.sqrt(sum(r*r for r in res)/len(res)):.4f} m/s^2"
      f"   |  sd(y) = {st.pstdev(Y):.4f}")
ybar = st.mean(Y)
r2 = 1 - sum(r*r for r in res)/sum((y-ybar)**2 for y in Y)
print(f"  R^2 = {r2:.4f}")
print(f"  drag decel at V=190, h=1500: {b*sigma(1500)*190**2:.3f} m/s^2 "
      f"({b*sigma(1500)*190**2/G:.3f} g)")

# ---- equilibrium speed on a straight path of angle gam (deg) ----
def v_eq(gam_deg, thr, h):
    """dV/dt = a*thr - b*sigma*V^2 + g*sin(|gam|) = 0"""
    acc = a*thr + G*math.sin(math.radians(abs(gam_deg)))
    return math.sqrt(acc/(b*sigma(h)))

print("\n-- equilibrium speed on a straight descent (h = 1500 m) --")
for gd in (0, 10, 20, 26.8, 35, 45, 52, 60):
    print(f"   gam {gd:5.1f} deg   thr=0: {v_eq(gd,0.0,1500):7.1f} m/s   "
          f"thr=1: {v_eq(gd,1.0,1500):7.1f} m/s")

# ---- quadrature along a PRESCRIBED geometric path (no guidance loop) ----
def fly(d0, h0, V0, thr, gam_prof=None, ds=5.0):
    """March along the straight release->target line (or a supplied profile).
    Returns (time, V_end). 1-D quadrature; guidance loop NOT simulated."""
    L = math.hypot(d0, h0)
    gam = math.degrees(math.atan2(h0, d0))
    V, t, s = V0, 0.0, 0.0
    while s < L:
        h = h0 * (1.0 - s / L)
        acc = a*thr - b*sigma(h)*V*V + G*math.sin(math.radians(gam))
        V = max(30.0, math.sqrt(max(900.0, V*V + 2*acc*ds)))
        t += ds / V
        s += ds
    return t, V

print("\n-- straight-line quadrature, d0=6140 h0=3100, V0=186 --")
for thr in (0.0, 0.36, 1.0):
    t, ve = fly(6140, 3100, 186.0, thr)
    print(f"   thr={thr:4.2f}  t = {t:6.2f} s   V_end = {ve:6.1f} m/s")

# ---- VALIDATION: predict each sortie's dive time along its OWN measured
#      ground path with its OWN reconstructed throttle profile ----
print("\n-- validation: model vs measured dive time (own path, own throttle) --")
errs = []
import json
a1 = {r["tag"]: r for r in json.load(open(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "a1_rows.json")))}
for tag, tk in ticks:
    live = [x for x in tk if not x["frozen"]]
    if len(live) < 50:
        continue
    # integrate V forward using the model along measured dt, using measured gam
    V = live[0]["V"]; t = 0.0
    for x, nx in zip(live, live[1:]):
        dt = nx["t"] - x["t"]
        if not (0.01 < dt <= 0.3):
            continue
        acc = a*x["thr"] - b*sigma(x["h"])*V*V - G*math.sin(math.radians(x["gam"]))
        V = max(30.0, V + acc*dt)
        t += dt
    meas_end = live[-1]["V"]
    errs.append((tag, V, meas_end, V-meas_end))
for tag, vp, vm, de in errs:
    print(f"   {tag:<11s} V_pred {vp:7.1f}  V_meas {vm:7.1f}  err {de:+7.1f}")
d = [e[3] for e in errs]
print(f"   speed-prediction err: med {st.median(d):+.2f}  "
      f"mean {st.mean(d):+.2f}  sd {st.pstdev(d):.2f}  "
      f"|err| med {st.median([abs(x) for x in d]):.2f} m/s   n={len(d)}")
