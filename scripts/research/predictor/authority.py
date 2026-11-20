#!/usr/bin/env python3
"""How much TIME can the glide slope actually buy, subject to hitting the target?

The PINN-PCG scheme needs, per aircraft, a feasible arrival window [t_min, t_max]
and then t_c = 0.5*(max_i t_min_i + min_i t_max_i).  That only exists if the
windows OVERLAP.  Measure the window width from a real handover state, and check
overlap across the measured handover dispersion.
"""
import math

G = 9.80665; M = 22.0; W = M*G; TFULL = M*G/0.7
S = 0.45; B = 1.88; AR = B*B/S; E = 0.9; PIEAR = math.pi*E*AR
CD0 = 0.01500

def predict(h0, d0, V0, w0, w1, dt=0.05, thr_pct=100.0, v_req=210.0, thr_fix=None):
    R = math.hypot(d0, h0); R0 = R; sig = math.atan2(h0, d0)
    V = V0; t = 0.0; Sx = 0.0
    while t < 300.0:
        h = R*math.sin(sig)
        if h <= 0.0 or R <= 1.0: break
        gam = math.radians(max(0.0, min(85.0, w0+w1*(1.0-R/R0))))
        rh = 1.225*(1.0-2.25577e-5*h)**4.2586
        qS = 0.5*rh*S*V*V
        CL = W*math.cos(gam)/max(qS, 1e-6)
        D = qS*(CD0+CL*CL/PIEAR)
        thr = thr_fix if thr_fix is not None else min(max(0.0, min(1.0, 0.36+0.030*(v_req-V))), thr_pct*0.01)
        thr = min(thr, thr_pct*0.01)
        Vd = (thr*TFULL+W*math.sin(gam)-D)/M
        hprev = h
        V = max(20.0, V+Vd*dt)
        R += -V*math.cos(gam-sig)*dt
        sig += V*math.sin(gam-sig)/max(R, 1.0)*dt
        Sx += V*math.cos(gam)*dt; t += dt
        hn = R*math.sin(sig)
        if hn <= 0.0 < hprev:
            f = hprev/(hprev-hn); t -= dt*(1.0-f); break
    return t, Sx, V, math.degrees(math.atan2(max(R*math.sin(sig), 0.0), max(R*math.cos(sig), 1e-6)))

def solve_w1(h0, d0, V0, w0, S_req, **kw):
    """given the initial slope w0, find the ramp w1 that lands exactly at S_req."""
    lo, hi = -w0, 80.0-w0
    f_lo = predict(h0, d0, V0, w0, lo, **kw)[1]-S_req
    f_hi = predict(h0, d0, V0, w0, hi, **kw)[1]-S_req
    if f_lo*f_hi > 0: return None
    for _ in range(50):
        mid = 0.5*(lo+hi)
        if (predict(h0, d0, V0, w0, mid, **kw)[1]-S_req)*f_lo > 0: lo = mid
        else: hi = mid
    return 0.5*(lo+hi)

H0, D0, V0 = 3100.0, 4800.0, 157.0
print("="*78)
print("TIME AUTHORITY of the glide slope, SUBJECT TO landing on the target")
print("="*78)
print(f"  handover h0={H0:.0f} d0={D0:.0f} V0={V0:.0f}, THR_MAX=100, PN throttle law")
print(f"  {'w0':>6s} {'w1*':>8s} {'t_f':>7s} {'V_end':>7s} {'gs_end':>7s} {'gam_end':>8s}")
ts = []
for w0 in (5, 10, 15, 20, 25, 30, 33, 35, 40, 45, 50, 55, 60):
    w1 = solve_w1(H0, D0, V0, float(w0), D0)
    if w1 is None:
        print(f"  {w0:6d}   (no w1 lands on target)")
        continue
    t, Sx, V, ge = predict(H0, D0, V0, float(w0), w1)
    ge2 = w0+w1
    ts.append(t)
    print(f"  {w0:6d} {w1:8.2f} {t:7.3f} {V:7.1f} "
          f"{V*math.cos(math.radians(ge2)):7.1f} {ge2:8.1f}")
print(f"\n  >>> time window from glide-slope shaping alone: "
      f"{min(ts):.2f} .. {max(ts):.2f} s  = {max(ts)-min(ts):.2f} s WIDE")

print()
print("="*78)
print("Does the window widen with the OTHER available actuators?")
print("="*78)
for nm, kw in (("THR_MAX=100 (as flown)", dict(thr_pct=100.0)),
               ("THR_MAX=25 (design)", dict(thr_pct=25.0)),
               ("throttle forced 0.0 (idle glide)", dict(thr_fix=0.0)),
               ("throttle forced 1.0 (full)", dict(thr_fix=1.0))):
    ts2 = []
    for w0 in (5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60):
        w1 = solve_w1(H0, D0, V0, float(w0), D0, **kw)
        if w1 is None: continue
        t, Sx, V, ge = predict(H0, D0, V0, float(w0), w1, **kw)
        ts2.append((t, w0, V*math.cos(math.radians(w0+w1))))
    if not ts2: continue
    lo = min(ts2); hi = max(ts2)
    print(f"  {nm:34s} t_f {lo[0]:6.2f}..{hi[0]:6.2f} s  (width {hi[0]-lo[0]:5.2f} s)  "
          f"gs_end {min(x[2] for x in ts2):5.1f}..{max(x[2] for x in ts2):5.1f}")

print()
print("="*78)
print("Do the per-aircraft windows OVERLAP across the measured handover dispersion?")
print("="*78)
print("  RPQ handover dispersion (CTX #3): 245 m of altitude, gamma spread 1.6 deg")
print(f"  {'h_hand':>8s} {'t_min':>8s} {'t_max':>8s} {'width':>7s}")
wins = []
for h0 in (3100-245, 3100-123, 3100, 3100+123, 3100+245):
    ts3 = []
    for w0 in (5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60):
        w1 = solve_w1(float(h0), D0, V0, float(w0), D0)
        if w1 is None: continue
        ts3.append(predict(float(h0), D0, V0, float(w0), w1)[0])
    if not ts3: continue
    wins.append((h0, min(ts3), max(ts3)))
    print(f"  {h0:8.0f} {min(ts3):8.3f} {max(ts3):8.3f} {max(ts3)-min(ts3):7.3f}")
if wins:
    t_lo = max(w[1] for w in wins); t_hi = min(w[2] for w in wins)
    print(f"\n  intersection = [{t_lo:.3f}, {t_hi:.3f}] -> "
          f"{'EMPTY (no common arrival time)' if t_hi < t_lo else f'width {t_hi-t_lo:.3f} s'}")
    print(f"  t_c = 0.5*(max t_min + min t_max) = {0.5*(t_lo+t_hi):.3f} s")
    print(f"  spread if every aircraft flies its OWN extreme: "
          f"{max(w[2] for w in wins)-min(w[1] for w in wins):.2f} s  (KPI4 allows 3.0)")

print()
print("="*78)
print("Sanity: what DOES have time authority? (vary the handover range = gate_r)")
print("="*78)
print(f"  {'d0':>7s} {'w0*':>7s} {'t_f':>7s} {'gs_end':>7s}   (h0=3100, straight-in w1=0)")
for d0 in (3500, 4200, 4800, 5400, 6000, 7000, 8000):
    w0 = math.degrees(math.atan2(H0, d0))
    t, Sx, V, ge = predict(H0, float(d0), V0, w0, 0.0)
    print(f"  {d0:7d} {w0:7.2f} {t:7.2f} {V*math.cos(math.radians(w0)):7.1f}")
