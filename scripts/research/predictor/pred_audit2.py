#!/usr/bin/env python3
"""Part 2: resolve the Mach confound, build+test the [R,lambda,sigma,V] form,
   and find the ground-speed-optimal dive angle."""
import pickle, math, os, statistics as st, time

G = 9.80665; M = 22.0; W = M*G; TFULL = M*G/0.7
S = 0.45; B = 1.88; AR = B*B/S; E = 0.9; PIEAR = math.pi*E*AR
CD0 = 0.01500; HOME = 32.0
d = os.path.dirname(os.path.abspath(__file__))
ALL = pickle.load(open(os.path.join(d, "traj.pkl"), "rb"))

def rho(h): return 1.225*(1.0-2.25577e-5*max(h, 0.0))**4.2586
def sos(h):
    T = max(216.65, 288.15-0.0065*max(h, 0.0))
    return math.sqrt(1.4*287.053*T)

# ---------------------------------------------------------------- B2: confound
print("="*78)
print("B2) is the 'Mach trend' really Mach, or the throttle law leaking V?")
print("="*78)
samp = []
for tag in sorted(ALL):
    for i in sorted(ALL[tag]):
        rows = [r for r in ALL[tag][i] if r["phase"] == "DIVE" and r["mode"] == "dive"]
        cut = len(rows)
        for k, r in enumerate(rows):
            if r["agl"] <= 0: cut = k; break
        rows = rows[:cut]
        for k, r in enumerate(rows):
            if not (150 < r["agl"] < 1500): continue
            if r["gam"] < 25 or r["V"] < 180: continue
            a = max(0, k-20); b = min(len(rows)-1, k+20)
            dt = rows[b]["t"]-rows[a]["t"]
            if dt <= 1e-6 or abs((rows[b]["V"]-rows[a]["V"])/dt) > 1.5: continue
            h = r["h_s"]+HOME; V = r["V"]; gam = math.radians(r["gam"])
            thr = max(0.0, min(1.0, 0.36+0.030*(210.0-V)))
            qS = 0.5*rho(h)*S*V*V
            CL = W*math.cos(gam)/qS
            cd0 = (thr*TFULL+W*math.sin(gam))/qS - CL*CL/PIEAR
            samp.append(dict(mach=V/sos(h), V=V, h=h, cd0=cd0, thr=thr, gam=gam, qS=qS))
n = len(samp)
def ols(x, y):
    mx = sum(x)/len(x); my = sum(y)/len(y)
    sxx = sum((v-mx)**2 for v in x)
    sxy = sum((x[k]-mx)*(y[k]-my) for k in range(len(x)))
    b = sxy/sxx if sxx else 0.0
    res = [y[k]-(my+b*(x[k]-mx)) for k in range(len(x))]
    ss = sum((v-my)**2 for v in y)
    return b, 1-sum(v*v for v in res)/ss
for nm in ("mach", "V", "h", "thr"):
    x = [s[nm] for s in samp]; b, r2 = ols(x, [s["cd0"] for s in samp])
    print(f"  cD0 ~ {nm:5s}: slope {b:+.6f}  R^2 {r2:.4f}   "
          f"range {min(x):.4f}..{max(x):.4f}")
print("  correlation(V, Mach) = ", end="")
v = [s["V"] for s in samp]; m = [s["mach"] for s in samp]
mv, mm = sum(v)/n, sum(m)/n
cov = sum((v[k]-mv)*(m[k]-mm) for k in range(n))
print(f"{cov/math.sqrt(sum((x-mv)**2 for x in v)*sum((x-mm)**2 for x in m)):.4f}")
print("  -> V and Mach are collinear here; SIM_Plane has NO Mach term in dragCoeff(),")
print("     and the fitted slope is NEGATIVE (drag falling with Mach) which is the")
print("     wrong sign for compressibility. The trend is a thrust-model artifact.")
print()
print("  test: hold V fixed (V in 205..215) and look at cD0 vs ALTITUDE only")
sub = [s for s in samp if 205 <= s["V"] <= 215]
if len(sub) > 30:
    b, r2 = ols([s["h"] for s in sub], [s["cd0"] for s in sub])
    print(f"    n={len(sub)}  cD0 ~ h: slope {b:+.8f} /m  R^2 {r2:.4f}  "
          f"(Mach varies {min(s['mach'] for s in sub):.3f}..{max(s['mach'] for s in sub):.3f} "
          f"at fixed V)")
    print(f"    -> at constant V, Mach still moves {100*(max(s['mach'] for s in sub)/min(s['mach'] for s in sub)-1):.1f}% "
          f"via altitude, and cD0 shows R^2={r2:.4f}. No Mach effect.")

# --------------------------------------------- B3: what if thrust model is off
print()
print("  how much thrust error mimics the observed 'Mach' swing?")
sw = 0.00464
qS_typ = st.median([s["qS"] for s in samp])
print(f"    observed cD0 swing {sw:.5f} over the band == force {sw*qS_typ:.1f} N "
      f"(= {100*sw*qS_typ/TFULL:.1f}% of full thrust, on a commanded band of "
      f"{100*(max(s['thr'] for s in samp)-min(s['thr'] for s in samp)):.1f}% throttle)")

# ------------------------------------------------- Q1: R,lambda,sigma,V form
print()
print("="*78)
print("Q1) [R, lambda, sigma, V] reformulation -- equivalence + termination test")
print("="*78)

def fly_hd(h0, d0, V0, gam_of, dt=0.02, hstop=0.0, thr_pct=100.0, v_req=210.0):
    """existing form: state (h, d, V)."""
    h, dd, V, t = h0, d0, V0, 0.0
    while h > hstop and t < 400.0:
        gam = math.radians(gam_of(dd, h))
        qS = 0.5*rho(h)*S*V*V
        CL = W*math.cos(gam)/max(qS, 1e-6)
        D = qS*(CD0+CL*CL/PIEAR)
        T = min(max(0.0, min(1.0, 0.36+0.030*(v_req-V))), thr_pct/100.0)*TFULL
        V += (T+W*math.sin(gam)-D)/M*dt
        h -= V*math.sin(gam)*dt; dd -= V*math.cos(gam)*dt; t += dt
    return V, t, dd, h

def fly_rls(h0, d0, V0, gam_of, dt=0.02, thr_pct=100.0, v_req=210.0, h_t=0.0):
    """paper form: state (R, sigma, V) [lambda decoupled], h reconstructed."""
    R = math.hypot(d0, h0-h_t); sig = math.atan2(h0-h_t, d0); V, t = V0, 0.0
    while R > 1e-6 and t < 400.0:
        h = h_t+R*math.sin(sig); dd = R*math.cos(sig)
        if h <= 0.0: break
        gam = math.radians(gam_of(dd, h))
        qS = 0.5*rho(h)*S*V*V
        CL = W*math.cos(gam)/max(qS, 1e-6)
        D = qS*(CD0+CL*CL/PIEAR)
        T = min(max(0.0, min(1.0, 0.36+0.030*(v_req-V))), thr_pct/100.0)*TFULL
        Vd = (T+W*math.sin(gam)-D)/M
        # lead angle between velocity (-gam below horizon) and LOS (-sig)
        Rd = -V*math.cos(gam-sig)
        sd = V*math.sin(gam-sig)/R
        V += Vd*dt; R += Rd*dt; sig += sd*dt; t += dt
    return V, t, R, sig

g_los = math.degrees(math.atan2(3100.0, 4800.0))
for gfun, nm in ((lambda dd, hh: g_los, "constant gamma (straight LOS)"),
                 (lambda dd, hh: math.degrees(math.atan2(max(hh, 0.0), max(dd, 1.0))),
                  "pure-LOS chase (gamma tracks sigma)")):
    a = fly_hd(3100.0, 4800.0, 157.0, gfun)
    b = fly_rls(3100.0, 4800.0, 157.0, gfun)
    print(f"  {nm:32s} (h,d,V): V_end {a[0]:7.2f} t {a[1]:6.2f}   "
          f"(R,sig,V): V_end {b[0]:7.2f} t {b[1]:6.2f}   dV {b[0]-a[0]:+.3f}")

# ------------------------------------------------------- Q5: gs-optimal gamma
print()
print("="*78)
print("Q5) ground speed is V*cos(gamma): where is the ground-speed optimum?")
print("="*78)
def v_eq(gam_deg, h, thr_pct, v_req=210.0):
    gam = math.radians(gam_deg); V = 150.0
    for _ in range(600):
        T = min(max(0.0, min(1.0, 0.36+0.030*(v_req-V))), thr_pct/100.0)*TFULL
        qS = 0.5*rho(h)*S*V*V
        CL = W*math.cos(gam)/max(qS, 1e-6)
        V += (T+W*math.sin(gam)-qS*(CD0+CL*CL/PIEAR))/M*0.25
        V = max(20.0, min(400.0, V))
    return V
print(f"  {'gamma':>6s} | {'THR_MAX=25':>21s} | {'THR_MAX=100':>21s}   (sea level, equilibrium)")
print(f"  {'':6s} | {'V_tot':>9s} {'gs':>10s} | {'V_tot':>9s} {'gs':>10s}")
best = {}
for g in (0, 5, 10, 15, 20, 25, 27, 30, 33, 35, 39, 40, 45, 50, 55, 60, 70, 80, 90):
    r = []
    for thr in (25.0, 100.0):
        V = v_eq(g, 0, thr); gs = V*math.cos(math.radians(g))
        r.append((V, gs))
        if gs > best.get(thr, (0, 0))[1]: best[thr] = (g, gs, V)
    print(f"  {g:6.0f} | {r[0][0]:9.1f} {r[0][1]:10.1f} | {r[1][0]:9.1f} {r[1][1]:10.1f}")
for thr in (25.0, 100.0):
    g, gs, V = best[thr]
    print(f"  THR_MAX={thr:.0f}%: ground speed peaks at gamma={g} deg -> gs={gs:.1f} m/s "
          f"(V_tot {V:.1f});  KPI2 needs 204")
