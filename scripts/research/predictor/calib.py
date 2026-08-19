#!/usr/bin/env python3
"""Calibrate c_D0_eff from the terminal quasi-equilibrium of the measured dives.

In the terminal PN segment the throttle law (tot_node._pn_cmd) is
    thr = clip(0,1, 0.36 + 0.030*(v_req - V)),  v_req = 210 (pn_vmax; late=>full)
and THR_MAX=100 in RPK/RPL (rep_interactive.sh unsets JET_THRMAX) so
    T = thr * TFULL,  TFULL = m*g/0.7 = 308.2 N.

Where dV/dt ~ 0 and gamma ~ const (the 1500..200 m AGL band, see profile.py)
    T + W sin(gam) = 0.5*rho*S*(cD0 + CL^2/(pi e AR))*V^2
=> invert for cD0.
"""
import pickle, math, os, statistics
G=9.80665; M=22.0; W=M*G; TFULL=M*G/0.7
S=0.45; B=1.88; AR=B*B/S; E=0.9; HOME=32.0
PIEAR=math.pi*E*AR
d=os.path.dirname(__file__)
ALL=pickle.load(open(os.path.join(d,"traj.pkl"),"rb"))

def rho(h): return 1.225*(1.0-2.25577e-5*max(h,0.0))**4.2586

def dvdt(rows,k,win=20):
    a=max(0,k-win); b=min(len(rows)-1,k+win)
    dt=rows[b]["t"]-rows[a]["t"]
    return (rows[b]["V"]-rows[a]["V"])/dt if dt>1e-6 else 0.0

print("== c_D0 inverted from terminal quasi-equilibrium (1500..150 m AGL) ==")
print("   filter: |dV/dt| < 1.5 m/s^2, gam > 25 deg, V > 180")
allc=[]
for tag in sorted(ALL):
    for i in sorted(ALL[tag]):
        rows=[r for r in ALL[tag][i] if r["phase"]=="DIVE" and r["mode"]=="dive"]
        cut=len(rows)
        for k,r in enumerate(rows):
            if r["agl"]<=0: cut=k; break
        rows=rows[:cut]
        loc=[]
        for k,r in enumerate(rows):
            if not (150 < r["agl"] < 1500): continue
            if r["gam"]<25 or r["V"]<180: continue
            a=dvdt(rows,k)
            if abs(a)>1.5: continue
            h=r["h_s"]+HOME; V=r["V"]; gam=math.radians(r["gam"])
            thr=max(0.0,min(1.0,0.36+0.030*(210.0-V)))
            T=thr*TFULL
            qS=0.5*rho(h)*S*V*V
            CL=W*math.cos(gam)/qS
            cd0=(T+W*math.sin(gam))/qS - CL*CL/PIEAR
            loc.append(cd0); allc.append(cd0)
        if loc:
            print(f"  {tag}/{i}: n={len(loc):4d}  cD0 = {statistics.median(loc):.5f}")
allc.sort()
print(f"\nPOOLED n={len(allc)}  median={statistics.median(allc):.5f}  "
      f"mean={statistics.mean(allc):.5f}  sd={statistics.pstdev(allc):.5f}")
print("  p10/p25/p50/p75/p90 = " + " ".join(f"{allc[int(p*len(allc))]:.5f}" for p in (.1,.25,.5,.75,.9)))
print(f"\n  JET_CDP written into jet_model.json         = 0.01642")
print(f"  jet_design.py raw solve() value p           = 0.01511  (0.01642/1.087)")
print(f"  -> measured effective cD0 matches the RAW value, i.e. SIM_Plane really")
print(f"     flies ~ (0.01642/0.01511)^0.5 = {math.sqrt(0.01642/0.01511):.3f}x faster than the")
print(f"     point-mass formula predicts for the coefficient it is given.")

CD0=statistics.median(allc)
print()
print("== derived airframe performance, cD0 = %.5f ==" % CD0)
LDmax=0.5*math.sqrt(PIEAR/CD0)
CLopt=math.sqrt(CD0*PIEAR)
print(f"  AR={AR:.3f}  pi*e*AR={PIEAR:.2f}")
print(f"  (L/D)max = {LDmax:.1f} at CL={CLopt:.3f}  "
      f"=> V_bestglide(SL) = {math.sqrt(2*W/(1.225*S*CLopt)):.1f} m/s  (irrelevant: we fly 5x that)")
print()
print(f"  {'V(m/s)':>7s} {'CL':>8s} {'CD':>8s} {'L/D':>7s} {'glide gam':>10s}   (sea level, 1g)")
for V in (100,135,157,180,200,204,216,230):
    qS=0.5*1.225*S*V*V
    CL=W/qS; CD=CD0+CL*CL/PIEAR
    ld=CL/CD
    print(f"  {V:7.0f} {CL:8.4f} {CD:8.5f} {ld:7.2f} {math.degrees(math.atan(1/ld)):10.1f}")
