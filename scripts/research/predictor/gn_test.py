#!/usr/bin/env python3
"""Real 2-parameter Gauss-Newton on the STOT residual: how many iterations,
   how well conditioned, and does warm-starting collapse the cost?"""
import math, time

G = 9.80665; M = 22.0; W = M*G; TFULL = M*G/0.7
S = 0.45; B = 1.88; AR = B*B/S; E = 0.9; PIEAR = math.pi*E*AR
CD0 = 0.01500

def predict(h0, d0, V0, w0, w1, dt=0.1, thr_pct=100.0, v_req=210.0):
    """glide profile gamma(R) = w0 + w1*(1 - R/R0) [deg]; integrate to h<=0.
       returns (t_f, S_flown_horizontal, V_end, gam_end)."""
    R = math.hypot(d0, h0); R0 = R; sig = math.atan2(h0, d0)
    V = V0; t = 0.0; Sx = 0.0
    while t < 200.0:
        h = R*math.sin(sig)
        if h <= 0.0 or R <= 1.0: break
        gam = math.radians(w0+w1*(1.0-R/R0))
        rh = 1.225*(1.0-2.25577e-5*h)**4.2586
        qS = 0.5*rh*S*V*V
        CL = W*math.cos(gam)/(qS if qS > 1e-6 else 1e-6)
        D = qS*(CD0+CL*CL/PIEAR)
        thr = min(max(0.0, min(1.0, 0.36+0.030*(v_req-V))), thr_pct*0.01)
        Vd = (thr*TFULL+W*math.sin(gam)-D)/M
        Rd = -V*math.cos(gam-sig); sd = V*math.sin(gam-sig)/R
        # Heun
        V2 = V+Vd*dt; R2 = max(1e-6, R+Rd*dt); sig2 = sig+sd*dt
        h2 = max(0.0, R2*math.sin(sig2))
        gam2 = math.radians(w0+w1*(1.0-R2/R0))
        rh2 = 1.225*(1.0-2.25577e-5*h2)**4.2586
        qS2 = 0.5*rh2*S*V2*V2
        CL2 = W*math.cos(gam2)/(qS2 if qS2 > 1e-6 else 1e-6)
        D2 = qS2*(CD0+CL2*CL2/PIEAR)
        thr2 = min(max(0.0, min(1.0, 0.36+0.030*(v_req-V2))), thr_pct*0.01)
        Vd2 = (thr2*TFULL+W*math.sin(gam2)-D2)/M
        Rd2 = -V2*math.cos(gam2-sig2); sd2 = V2*math.sin(gam2-sig2)/R2
        hprev = h
        V += 0.5*(Vd+Vd2)*dt; R += 0.5*(Rd+Rd2)*dt; sig += 0.5*(sd+sd2)*dt
        Sx += V*math.cos(gam)*dt; t += dt
        hn = R*math.sin(sig)
        if hn <= 0.0 and hprev > 0.0:          # interpolate the last partial step
            f = hprev/(hprev-hn)
            t -= dt*(1.0-f)
            break
    return t, Sx, V, math.degrees(sig)

def residual(h0, d0, V0, w, S_req, T_cmd, dt=0.1):
    t, Sx, V, g = predict(h0, d0, V0, w[0], w[1], dt=dt)
    return [Sx-S_req, t-T_cmd], (t, Sx, V, g)

def gauss_newton(h0, d0, V0, S_req, T_cmd, w=None, mu=1e-3, dt=0.1,
                 tol=(1.0, 0.02), itmax=40, eps=(0.05, 0.05), verbose=False):
    w = list(w if w else [30.0, 5.0]); nint = 0
    hist = []
    for it in range(itmax):
        r, info = residual(h0, d0, V0, w, S_req, T_cmd, dt); nint += 1
        hist.append((it, w[0], w[1], r[0], r[1]))
        if verbose:
            print(f"    it{it:2d} w=({w[0]:7.3f},{w[1]:7.3f}) "
                  f"r=({r[0]:+9.2f} m, {r[1]:+7.3f} s)  V_end={info[2]:.1f}")
        if abs(r[0]) < tol[0] and abs(r[1]) < tol[1]:
            return w, it, nint, r, info, hist, True
        J = [[0.0, 0.0], [0.0, 0.0]]
        for j in range(2):
            wp = list(w); wp[j] += eps[j]
            rp, _ = residual(h0, d0, V0, wp, S_req, T_cmd, dt); nint += 1
            J[0][j] = (rp[0]-r[0])/eps[j]; J[1][j] = (rp[1]-r[1])/eps[j]
        # (J^T J + mu I) dw = -J^T r
        a = J[0][0]*J[0][0]+J[1][0]*J[1][0]+mu
        b = J[0][0]*J[0][1]+J[1][0]*J[1][1]
        c = b
        d_ = J[0][1]*J[0][1]+J[1][1]*J[1][1]+mu
        g0 = -(J[0][0]*r[0]+J[1][0]*r[1])
        g1 = -(J[0][1]*r[0]+J[1][1]*r[1])
        det = a*d_-b*c
        if abs(det) < 1e-14: break
        dw = [(d_*g0-b*g1)/det, (a*g1-c*g0)/det]
        step = max(abs(dw[0]), abs(dw[1]))
        if step > 5.0:                       # trust region
            dw = [x*5.0/step for x in dw]
        w = [w[0]+dw[0], w[1]+dw[1]]
        w[0] = max(0.0, min(80.0, w[0])); w[1] = max(-40.0, min(60.0, w[1]))
    return w, it, nint, r, info, hist, False

H0, D0, V0 = 3100.0, 4800.0, 157.0
print("== cold-start solve: hit the target 4800 m away in a commanded time ==")
S_req = 4800.0
for T_cmd in (26.0, 28.0, 30.0, 33.0, 36.0):
    t0 = time.perf_counter()
    w, it, nint, r, info, hist, ok = gauss_newton(H0, D0, V0, S_req, T_cmd)
    ms = (time.perf_counter()-t0)*1000.0
    print(f"  T_cmd={T_cmd:5.1f}s -> w=({w[0]:6.2f},{w[1]:6.2f}) iters={it:2d} "
          f"integrations={nint:3d} {ms:7.2f} ms  conv={ok}  "
          f"r=({r[0]:+7.2f} m,{r[1]:+6.3f} s)  V_end={info[2]:6.1f} "
          f"gs_end={info[2]*math.cos(math.radians(w[0]+w[1])):6.1f}")

print()
print("  verbose trace for T_cmd=30:")
gauss_newton(H0, D0, V0, S_req, 30.0, verbose=True)

print()
print("== WARM start: state advances one guidance cycle, re-solve from last w ==")
w = [30.0, 5.0]
w, _, _, _, _, _, _ = gauss_newton(H0, D0, V0, S_req, 30.0)
print(f"  converged cold to w=({w[0]:.3f},{w[1]:.3f})")
h, dd, V = H0, D0, V0
tot_ms = 0.0; tot_int = 0
for cyc in range(8):
    gam = math.radians(w[0])
    V += 0.0
    h -= V*math.sin(gam)*0.5; dd -= V*math.cos(gam)*0.5   # advance 0.5 s
    t0 = time.perf_counter()
    w, it, nint, r, info, hist, ok = gauss_newton(h, dd, V, dd, 30.0-0.5*(cyc+1), w=w)
    ms = (time.perf_counter()-t0)*1000.0
    tot_ms += ms; tot_int += nint
    print(f"  cycle {cyc}: h={h:6.0f} d={dd:6.0f} -> iters={it} integrations={nint:2d} "
          f"{ms:6.2f} ms  r=({r[0]:+6.2f} m,{r[1]:+6.3f} s)")
print(f"  warm-start mean: {tot_int/8:.1f} integrations, {tot_ms/8:.2f} ms per aircraft per solve")

print()
print("== Jacobian conditioning (are the 2 params independent?) ==")
w = [33.0, 8.0]
r, _ = residual(H0, D0, V0, w, S_req, 30.0)
J = [[0.0, 0.0], [0.0, 0.0]]
for j in range(2):
    wp = list(w); wp[j] += 0.05
    rp, _ = residual(H0, D0, V0, wp, S_req, 30.0)
    J[0][j] = (rp[0]-r[0])/0.05; J[1][j] = (rp[1]-r[1])/0.05
print(f"  J = [[{J[0][0]:9.3f} {J[0][1]:9.3f}]   d(range)/d(w0,w1)   m/deg")
print(f"       [{J[1][0]:9.4f} {J[1][1]:9.4f}]]  d(time)/d(w0,w1)    s/deg")
# singular values of 2x2
a, b = J[0]; c, dd_ = J[1]
tr = a*a+b*b+c*c+dd_*dd_; det = abs(a*dd_-b*c)
s1 = math.sqrt(max(0.0, (tr+math.sqrt(max(0.0, tr*tr-4*det*det)))/2))
s2 = math.sqrt(max(0.0, (tr-math.sqrt(max(0.0, tr*tr-4*det*det)))/2))
print(f"  singular values {s1:.3f} / {s2:.4f}  -> condition number {s1/max(s2,1e-12):.1f}")
print(f"  (units differ: range in m, time in s. Scale the residual before trusting this.)")
# scaled: normalise range residual by 10 m tolerance, time by 0.1 s
Js = [[J[0][0]/10.0, J[0][1]/10.0], [J[1][0]/0.1, J[1][1]/0.1]]
a, b = Js[0]; c, dd_ = Js[1]
tr = a*a+b*b+c*c+dd_*dd_; det = abs(a*dd_-b*c)
s1 = math.sqrt(max(0.0, (tr+math.sqrt(max(0.0, tr*tr-4*det*det)))/2))
s2 = math.sqrt(max(0.0, (tr-math.sqrt(max(0.0, tr*tr-4*det*det)))/2))
print(f"  tolerance-scaled (10 m, 0.1 s): sv {s1:.2f} / {s2:.3f} -> cond {s1/max(s2,1e-12):.1f}")

print()
print("== time authority: how much time can the glide slope actually buy? ==")
print(f"  {'w0':>6s} {'w1':>6s} {'t_f':>7s} {'S_x':>8s} {'V_end':>7s} {'gs_end':>7s} {'gam_end':>8s}")
for w0, w1 in ((20, 0), (25, 0), (30, 0), (33, 0), (40, 0), (50, 0),
               (25, 10), (30, 10), (25, 20), (30, 20), (20, 25)):
    t, Sx, V, g = predict(H0, D0, V0, float(w0), float(w1))
    ge = w0+w1
    print(f"  {w0:6d} {w1:6d} {t:7.2f} {Sx:8.0f} {V:7.1f} "
          f"{V*math.cos(math.radians(ge)):7.1f} {ge:8.1f}")
