#!/usr/bin/env python3
"""The predictor treats gamma as an instantaneous input. What does that cost?

Validation replayed the MEASURED gamma(t), so gamma dynamics were never tested.
Measure (a) the achievable gamma rate from the nz limits, (b) the path/altitude
consumed by a gamma change, (c) how wrong the predicted terminal state gets if
the profile is tracked with a first-order lag instead of instantly.
"""
import math, pickle, os, statistics as st

G = 9.80665; M = 22.0; W = M*G; TFULL = M*G/0.7
S = 0.45; B = 1.88; AR = B*B/S; E = 0.9; PIEAR = math.pi*E*AR
CD0 = 0.01500

print("="*78)
print("A) gamma rate available from the normal-load limits")
print("="*78)
print("   gamma_dot = (nz - cos gamma) * g / V     [rad/s]")
print(f"  {'V':>6s} {'gam':>5s} | " + " ".join(f"nz={nz:<5.2f}" for nz in (-1.0, 0.05, 1.0, 2.0, 3.0)))
for V in (157.0, 190.0, 210.0):
    for gam in (0.0, 30.0, 45.0):
        row = []
        for nz in (-1.0, 0.05, 1.0, 2.0, 3.0):
            gd = (nz-math.cos(math.radians(gam)))*G/V
            row.append(f"{math.degrees(gd):+7.2f}")
        print(f"  {V:6.0f} {gam:5.0f} | " + " ".join(row) + "   deg/s")
print("   (pn_nz_min = 0.05 on the interactive path, -1.0 on the batch path)")

print()
print("="*78)
print("B) cost of a gamma change: time, path, altitude")
print("="*78)
def pushover(V, gam0, gam1, nz):
    """time/path/alt to rotate from gam0 to gam1 at constant nz and speed V."""
    g0, g1 = math.radians(gam0), math.radians(gam1)
    dt = 0.01; t = 0.0; g = g0; path = 0.0; dh = 0.0
    while (g1-g)*(1 if g1 > g0 else -1) > 0 and t < 120.0:
        gd = (nz-math.cos(g))*G/V
        if (g1 > g0 and gd >= 0) or (g1 < g0 and gd <= 0): return None
        g += gd*dt; path += V*dt; dh += V*math.sin(g)*dt; t += dt
    return t, path, dh
print(f"  {'from':>5s} {'to':>4s} {'nz':>6s} {'V':>5s} | {'time':>7s} {'path':>8s} "
      f"{'alt lost':>9s} {'horiz':>8s}")
for gam0, gam1 in ((0, 30), (0, 45), (30, 45), (33, 50), (0, 60)):
    for nz in (0.05, -1.0):
        for V in (157.0, 210.0):
            r = pushover(V, gam0, gam1, nz)
            if r is None:
                print(f"  {gam0:5d} {gam1:4d} {nz:6.2f} {V:5.0f} |  IMPOSSIBLE (nz too high to push over)")
                continue
            t, path, dh = r
            print(f"  {gam0:5d} {gam1:4d} {nz:6.2f} {V:5.0f} | {t:7.2f}s {path:8.0f}m "
                  f"{dh:9.0f}m {math.sqrt(max(path*path-dh*dh,0)):8.0f}m")

print()
print("="*78)
print("C) predicted terminal state: instant gamma vs first-order tracked gamma")
print("="*78)
def fly(h0, d0, V0, w0, w1, tau=0.0, nz_min=0.05, dt=0.02, thr_pct=100.0, v_req=210.0):
    R = math.hypot(d0, h0); R0 = R; sig = math.atan2(h0, d0)
    V = V0; t = 0.0; gam = math.atan2(h0, d0)   # start on the LOS
    while t < 300.0:
        h = R*math.sin(sig)
        if h <= 0.0 or R <= 1.0: break
        gc = math.radians(max(0.0, min(85.0, w0+w1*(1.0-R/R0))))
        if tau <= 0.0:
            gam = gc
        else:
            gd = (gc-gam)/tau
            gdmax = (3.0-math.cos(gam))*G/V           # pull-down limit
            gdmin = (nz_min-math.cos(gam))*G/V        # push-over limit
            gam += max(gdmin, min(gdmax, gd))*dt
        rh = 1.225*(1.0-2.25577e-5*h)**4.2586
        qS = 0.5*rh*S*V*V
        CL = W*math.cos(gam)/max(qS, 1e-6)
        D = qS*(CD0+CL*CL/PIEAR)
        thr = min(max(0.0, min(1.0, 0.36+0.030*(v_req-V))), thr_pct*0.01)
        V = max(20.0, V+(thr*TFULL+W*math.sin(gam)-D)/M*dt)
        hprev = h
        R += -V*math.cos(gam-sig)*dt
        sig += V*math.sin(gam-sig)/max(R, 1.0)*dt
        t += dt
        hn = R*math.sin(sig)
        if hn <= 0.0 < hprev:
            f = hprev/(hprev-hn); t -= dt*(1.0-f); break
    return t, V, math.degrees(gam), R*math.cos(sig)
print(f"  {'profile':>14s} | {'instant gamma':>26s} | {'tau=1s, nz>=0.05':>26s} | "
      f"{'tau=1s, nz>=-1.0':>26s}")
print(f"  {'w0/w1':>14s} | {'t_f':>7s} {'V':>6s} {'x_miss':>10s} | "
      f"{'t_f':>7s} {'V':>6s} {'x_miss':>10s} | {'t_f':>7s} {'V':>6s} {'x_miss':>10s}")
for w0, w1 in ((33, 0), (25, 15), (20, 25), (45, -12), (33, 17)):
    o = []
    for tau, nzm in ((0.0, 0.05), (1.0, 0.05), (1.0, -1.0)):
        t, V, ge, xm = fly(3100.0, 4800.0, 157.0, float(w0), float(w1), tau=tau, nz_min=nzm)
        o.append((t, V, xm))
    print(f"  {w0:6d}/{w1:<7d} | " + " | ".join(
        f"{a:7.2f} {b:6.1f} {c:10.1f}" for a, b, c in o))
print("  x_miss = horizontal distance still to run when h reaches 0 (+ = short of target)")
