#!/usr/bin/env python3
"""Audit the point-mass predictor for the Leblebici hybrid-shaping port.

A) how big is the induced-drag error from CL = W cos(gam)/qS  (nz==cos gam)?
B) does the inverted cD0 carry Mach / CL structure (i.e. would a table help)?
C) sensitivity of the predicted TOUCHDOWN speed to cD0 mis-specification
E) ground speed vs total speed at touchdown (KPI is ground speed)
"""
import pickle, math, os, statistics as st

G = 9.80665; M = 22.0; W = M*G; TFULL = M*G/0.7
S = 0.45; B = 1.88; AR = B*B/S; E = 0.9; PIEAR = math.pi*E*AR
CD0 = 0.01500; HOME = 32.0
d = os.path.dirname(os.path.abspath(__file__))
ALL = pickle.load(open(os.path.join(d, "traj.pkl"), "rb"))

def rho(h): return 1.225*(1.0-2.25577e-5*max(h, 0.0))**4.2586
def sos(h):  # ISA speed of sound
    T = max(216.65, 288.15 - 0.0065*max(h, 0.0))
    return math.sqrt(1.4*287.053*T)

def dive_rows(tag, i, agl_lo=150.0, agl_hi=1e9):
    rows = [r for r in ALL[tag][i] if r["phase"] == "DIVE" and r["mode"] == "dive"]
    cut = len(rows)
    for k, r in enumerate(rows):
        if r["agl"] <= 0: cut = k; break
    return [r for r in rows[:cut] if agl_lo < r["agl"] < agl_hi]

def smooth(xs, n):
    return [sum(xs[max(0,k-n):min(len(xs),k+n+1)])/len(xs[max(0,k-n):min(len(xs),k+n+1)])
            for k in range(len(xs))]

print("="*78)
print("A) nz != cos(gam):  how wrong is the predictor's CL during the dive?")
print("="*78)
print(f"  {'run/uav':10s} {'n':>5s} {'nz med':>7s} {'nz p05':>7s} {'nz p95':>7s} "
      f"{'dCDi med':>9s} {'dCDi p95':>9s} {'=dcD0 %':>8s}")
pool_nz, pool_dcdi = [], []
for tag in sorted(ALL):
    for i in sorted(ALL[tag]):
        rows = dive_rows(tag, i)
        if len(rows) < 200: continue
        t = [r["t"] for r in rows]
        gam = smooth([math.radians(r["gam"]) for r in rows], 10)
        nzs, dcdi = [], []
        for k in range(len(rows)):
            a = max(0, k-10); b = min(len(rows)-1, k+10)
            dt = t[b]-t[a]
            if dt <= 1e-6: continue
            gdot = (gam[b]-gam[a])/dt          # rad/s, +down positive
            V = rows[k]["V"]; h = rows[k]["h_s"]+HOME
            # measured trajectory curvature -> real normal load factor
            nz = math.cos(gam[k]) - V*gdot/G   # gam here is +down, so sign flips
            qS = 0.5*rho(h)*S*V*V
            CL_pred = W*math.cos(gam[k])/qS    # what the predictor assumes
            CL_true = W*nz/qS                  # what the airframe actually pulls
            dcdi.append((CL_true**2 - CL_pred**2)/PIEAR)
            nzs.append(nz)
        if not nzs: continue
        nzs_s = sorted(nzs); dc_s = sorted(dcdi, key=abs)
        med = st.median(nzs); p05 = nzs_s[int(.05*len(nzs_s))]; p95 = nzs_s[int(.95*len(nzs_s))]
        dmed = st.median(dcdi); dp95 = dc_s[int(.95*len(dc_s))]
        pool_nz += nzs; pool_dcdi += dcdi
        print(f"  {tag}/{i:<8d} {len(nzs):5d} {med:7.3f} {p05:7.3f} {p95:7.3f} "
              f"{dmed:9.5f} {dp95:9.5f} {100*abs(dp95)/CD0:8.1f}")
ps = sorted(pool_dcdi, key=abs)
print(f"\n  POOLED n={len(pool_nz)}  nz median={st.median(pool_nz):.3f} "
      f"(p05 {sorted(pool_nz)[int(.05*len(pool_nz))]:.3f} / "
      f"p95 {sorted(pool_nz)[int(.95*len(pool_nz))]:.3f})")
print(f"  induced-drag error |dCDi|: median {abs(st.median(pool_dcdi)):.5f}  "
      f"p95 {abs(ps[int(.95*len(ps))]):.5f}  max {abs(ps[-1]):.5f}")
print(f"  as a fraction of cD0={CD0}: median {100*abs(st.median(pool_dcdi))/CD0:.1f}% "
      f"p95 {100*abs(ps[int(.95*len(ps))])/CD0:.1f}%  max {100*abs(ps[-1])/CD0:.1f}%")

print()
print("="*78)
print("B) does the inverted cD0 carry Mach / CL structure?")
print("="*78)
samp = []
for tag in sorted(ALL):
    for i in sorted(ALL[tag]):
        rows = dive_rows(tag, i, 150.0, 1500.0)
        for k, r in enumerate(rows):
            if r["gam"] < 25 or r["V"] < 180: continue
            a = max(0, k-20); b = min(len(rows)-1, k+20)
            dt = rows[b]["t"]-rows[a]["t"]
            if dt <= 1e-6: continue
            if abs((rows[b]["V"]-rows[a]["V"])/dt) > 1.5: continue
            h = r["h_s"]+HOME; V = r["V"]; gam = math.radians(r["gam"])
            thr = max(0.0, min(1.0, 0.36+0.030*(210.0-V)))
            qS = 0.5*rho(h)*S*V*V
            CL = W*math.cos(gam)/qS
            cd0 = (thr*TFULL + W*math.sin(gam))/qS - CL*CL/PIEAR
            samp.append((V/sos(h), CL, cd0, V, h))
print(f"  usable quasi-equilibrium samples n={len(samp)}")
mach = [s[0] for s in samp]; cls = [s[1] for s in samp]; cds = [s[2] for s in samp]
print(f"  Mach range {min(mach):.3f}..{max(mach):.3f}   CL range {min(cls):.4f}..{max(cls):.4f}")
print(f"  cD0 pooled: median {st.median(cds):.5f}  sd {st.pstdev(cds):.5f}")

def ols(x, y):
    n = len(x); mx = sum(x)/n; my = sum(y)/n
    sxx = sum((v-mx)**2 for v in x); sxy = sum((x[k]-mx)*(y[k]-my) for k in range(n))
    b = sxy/sxx if sxx else 0.0; a = my-b*mx
    res = [y[k]-(a+b*x[k]) for k in range(n)]
    ss = sum((v-my)**2 for v in y); rss = sum(v*v for v in res)
    return a, b, (1-rss/ss if ss else 0.0), math.sqrt(rss/n)

for name, x in (("Mach", mach), ("CL", cls)):
    a, b, r2, rmse = ols(x, cds)
    span = (max(x)-min(x))*b
    print(f"  cD0 ~ {name:5s}: slope {b:+.5f} /unit  R^2 {r2:.4f}  "
          f"resid sd {rmse:.5f}  swing over observed range {span:+.5f} "
          f"({100*abs(span)/CD0:.1f}% of cD0)")
# binned medians
print("\n  binned medians (is there any real trend?):")
for name, idx in (("Mach", 0), ("CL", 1)):
    xs = sorted(samp, key=lambda s: s[idx])
    nb = 6; per = len(xs)//nb
    row = []
    for bnum in range(nb):
        chunk = xs[bnum*per:(bnum+1)*per if bnum < nb-1 else len(xs)]
        row.append(f"{st.median([c[idx] for c in chunk]):.3f}:{st.median([c[2] for c in chunk]):.5f}")
    print(f"    {name:5s} " + "  ".join(row))

print()
print("="*78)
print("C) sensitivity of TOUCHDOWN speed to cD0 (and to the nz error)")
print("="*78)
import sim
base = sim.CD0
print(f"  {'cD0':>8s} {'d(cD0)':>8s} {'V_touch':>8s} {'dV':>7s} {'gs_touch':>9s} "
      f"  (h0=3100, d0=4800, V0=157, THR_MAX=100, straight LOS)")
g_los = math.degrees(math.atan2(3100.0, 4800.0))
ref = None
for f in (0.80, 0.90, 0.9613, 1.0, 1.0387, 1.10, 1.20):
    sim.CD0 = base*f
    out = sim.fly(3100.0, 4800.0, 157.0, lambda dd, hh: g_los, 100.0)
    if ref is None and abs(f-1.0) < 1e-9: pass
    if abs(f-1.0) < 1e-9: ref = out["V"]
for f in (0.80, 0.90, 0.9613, 1.0, 1.0387, 1.10, 1.20):
    sim.CD0 = base*f
    out = sim.fly(3100.0, 4800.0, 157.0, lambda dd, hh: g_los, 100.0)
    tag = "  <- 1 sd of the fit" if abs(f-0.9613) < 1e-4 or abs(f-1.0387) < 1e-4 else ""
    print(f"  {sim.CD0:8.5f} {100*(f-1):+7.1f}% {out['V']:8.1f} {out['V']-ref:+7.2f} "
          f"{out['V']*math.cos(math.radians(g_los)):9.1f}{tag}")
sim.CD0 = base

print()
print("="*78)
print("E) ground speed vs total speed at touchdown  (KPI2 = 204 m/s GROUND speed)")
print("="*78)
print(f"  {'run/uav':10s} {'gam_td':>7s} {'V_tot':>7s} {'gs':>7s} {'hdot':>7s} "
      f"{'gs/V':>6s} {'V req for gs=204':>17s}")
gaps = []
for tag in sorted(ALL):
    for i in sorted(ALL[tag]):
        rows = [r for r in ALL[tag][i] if r["phase"] == "DIVE" and r["mode"] == "dive"]
        cut = len(rows)
        for k, r in enumerate(rows):
            if r["agl"] <= 0: cut = k; break
        rows = rows[:cut]
        if len(rows) < 100: continue
        r = rows[-1]
        c = math.cos(math.radians(r["gam"]))
        gaps.append((r["gam"], r["V"], r["gs_s"], c))
        print(f"  {tag}/{i:<8d} {r['gam']:7.1f} {r['V']:7.1f} {r['gs_s']:7.1f} "
              f"{r['hdot']:7.1f} {c:6.3f} {204.0/max(c,1e-3):17.1f}")
print(f"\n  mean gam_touchdown {st.mean([g[0] for g in gaps]):.1f} deg  =>  "
      f"gs = {st.mean([g[3] for g in gaps]):.3f} * V_total")
print(f"  mean V_total {st.mean([g[1] for g in gaps]):.1f} m/s  ->  "
      f"mean gs {st.mean([g[2] for g in gaps]):.1f} m/s  "
      f"(deficit vs KPI 204: {204-st.mean([g[2] for g in gaps]):+.1f} m/s)")
