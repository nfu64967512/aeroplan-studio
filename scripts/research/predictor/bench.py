#!/usr/bin/env python3
"""Execution budget for the online predictor inside tot_node (pure python, math only)."""
import math, time

G = 9.80665; M = 22.0; W = M*G; TFULL = M*G/0.7
S = 0.45; B = 1.88; AR = B*B/S; E = 0.9; PIEAR = math.pi*E*AR
CD0 = 0.01500

def deriv(R, sig, V, gam, h, thr_pct, v_req):
    qS = 0.5*1.225*(1.0-2.25577e-5*(h if h > 0 else 0.0))**4.2586*S*V*V
    CL = W*math.cos(gam)/(qS if qS > 1e-6 else 1e-6)
    D = qS*(CD0+CL*CL/PIEAR)
    thr = 0.36+0.030*(v_req-V)
    thr = 0.0 if thr < 0.0 else (1.0 if thr > 1.0 else thr)
    lim = thr_pct*0.01
    T = (thr if thr < lim else lim)*TFULL
    return ((T+W*math.sin(gam)-D)/M, -V*math.cos(gam-sig), V*math.sin(gam-sig)/R)

def predict(h0, d0, V0, gam_cmd, dt=0.05, method="heun", thr_pct=100.0, v_req=210.0):
    """forward-integrate to R<=0; return (V_end, gam_end, t_f, nsteps)."""
    R = math.hypot(d0, h0); sig = math.atan2(h0, d0); V = V0; t = 0.0; nst = 0
    while R > 1.0 and t < 200.0:
        h = R*math.sin(sig)
        if h <= 0.0: break
        gam = gam_cmd(R, sig, V, t)
        Vd, Rd, sd = deriv(R, sig, V, gam, h, thr_pct, v_req)
        if method == "euler":
            V += Vd*dt; R += Rd*dt; sig += sd*dt
        else:  # heun / RK2
            V2 = V+Vd*dt; R2 = R+Rd*dt; sig2 = sig+sd*dt
            h2 = R2*math.sin(sig2)
            g2 = gam_cmd(R2, sig2, V2, t+dt)
            Vd2, Rd2, sd2 = deriv(R2 if R2 > 1e-6 else 1e-6, sig2, V2, g2,
                                  h2 if h2 > 0 else 0.0, thr_pct, v_req)
            V += 0.5*(Vd+Vd2)*dt; R += 0.5*(Rd+Rd2)*dt; sig += 0.5*(sd+sd2)*dt
        t += dt; nst += 1
    return V, math.degrees(sig), t, nst

# glide profile parameterised by w=(w0,w1): gamma = w0 + w1*(1 - R/R0)
def make_gam(w0, w1, R0):
    def f(R, sig, V, t):
        return math.radians(w0+w1*(1.0-R/R0))
    return f

H0, D0, V0 = 3100.0, 4800.0, 157.0
R0 = math.hypot(H0, D0)

print("== accuracy vs step size (reference: Heun dt=0.005) ==")
ref = predict(H0, D0, V0, make_gam(33.0, 12.0, R0), dt=0.005, method="heun")
print(f"  {'method':6s} {'dt':>6s} {'steps':>6s} {'V_end':>8s} {'dV':>7s} {'t_f':>7s} "
      f"{'dt_f':>7s} {'usec/call':>10s}")
rows = []
for method in ("euler", "heun"):
    for dt in (0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0):
        f = make_gam(33.0, 12.0, R0)
        V, g, tf, nst = predict(H0, D0, V0, f, dt=dt, method=method)
        N = max(3, int(0.30/max(1e-9, (nst*2e-6))))
        t0 = time.perf_counter()
        for _ in range(N): predict(H0, D0, V0, f, dt=dt, method=method)
        us = (time.perf_counter()-t0)/N*1e6
        rows.append((method, dt, nst, V, tf, us))
        print(f"  {method:6s} {dt:6.3f} {nst:6d} {V:8.2f} {V-ref[0]:+7.3f} {tf:7.2f} "
              f"{tf-ref[2]:+7.3f} {us:10.1f}")

print()
print("== Gauss-Newton budget (2 params, forward-difference Jacobian) ==")
print("   per iteration: 1 baseline + 2 perturbed = 3 integrations (central diff: 5)")
for method, dt in (("heun", 0.05), ("heun", 0.1), ("heun", 0.2), ("euler", 0.1), ("euler", 0.2)):
    us = [r[5] for r in rows if r[0] == method and abs(r[1]-dt) < 1e-9][0]
    for iters in (5, 10, 30):
        per_ac_fwd = 3*iters*us/1000.0
        per_ac_ctr = 5*iters*us/1000.0
        print(f"  {method:6s} dt={dt:4.2f}  {iters:2d} iters -> "
              f"{per_ac_fwd:7.2f} ms/aircraft (fwd-diff) | {per_ac_ctr:7.2f} ms (central)"
              f"   n=6 swarm: {6*per_ac_fwd:8.1f} ms")

print()
print("== what fits in the tot_node tick? (single process, single thread) ==")
print("   tot_node: create_timer(0.05) = 20 Hz = 50 ms budget for ALL n aircraft,")
print("   shared with _pump() MAVLink drain and the per-UAV command sends.")
for hz in (20.0, 10.0, 5.0, 2.0, 1.0):
    budget = 1000.0/hz*0.30   # allow 30% of the period for the solver
    us = [r[5] for r in rows if r[0] == "heun" and abs(r[1]-0.1) < 1e-9][0]
    n_int = budget/(us/1000.0)
    print(f"  at {hz:5.1f} Hz, 30% duty = {budget:6.1f} ms -> {n_int:6.0f} integrations/cycle "
          f"= {n_int/6:5.0f} per aircraft (n=6) = {n_int/6/3:5.1f} GN iterations")
