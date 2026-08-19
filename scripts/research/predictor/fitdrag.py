#!/usr/bin/env python3
"""Calibrate the effective drag polar of the SITL airframe from measured dives.

Known-thrust window: in DIVE the tot_node throttle law is
    thr = clip(0,1, 0.36 + 0.030*(v_req - V)),  v_req = min(pn_vmax=210, R/t_rem)
so thr saturates at 1.0 whenever V <= v_req - 21.33.  With THR_MAX=100 (RPK/RPL
config, handoff_2026-08-11 line 168) servo 1.0 => full thrust.

Newton along the flight path:
    m dV/dt = T + W sin(gam) - D           (gam +down)
    =>  D = T + W sin(gam) - m dV/dt
    c_drag_p_eff = D/(q S) - CL^2/(pi e AR),  CL = W cos(gam)/(q S)
"""
import pickle, math, os, statistics
G=9.80665; M=22.0; W=M*G; HOVER=0.7; TFULL=M*G/HOVER
S=0.45; B=1.88; AR=B*B/S; E=0.9; HOME=32.0
d=os.path.dirname(__file__)
ALL=pickle.load(open(os.path.join(d,"traj.pkl"),"rb"))

def rho(h):
    return 1.225*(1.0-2.25577e-5*max(h,0.0))**4.2586

def dvdt(rows,k,win=10):
    a=max(0,k-win); b=min(len(rows)-1,k+win)
    dt=rows[b]["t"]-rows[a]["t"]
    return (rows[b]["V"]-rows[a]["V"])/dt if dt>1e-6 else 0.0

print("== drag fit, full-throttle window (V<=185, mode=dive, gam>5deg) ==")
samples=[]
for tag in ALL:
    for i in ALL[tag]:
        rows=[r for r in ALL[tag][i] if r["phase"]=="DIVE" and r["mode"]=="dive"]
        cut=len(rows)
        for k,r in enumerate(rows):
            if r["agl"]<=0: cut=k; break
        rows=rows[:cut]
        loc=[]
        for k,r in enumerate(rows):
            if not (60 < r["V"] <= 185): continue
            if r["gam"] < 5: continue
            if k<15 or k>len(rows)-15: continue
            a=dvdt(rows,k)
            h=r["h_s"]+HOME
            q=0.5*rho(h)*r["V"]**2
            qS=q*S
            gam=math.radians(r["gam"])
            D=TFULL + W*math.sin(gam) - M*a
            CL=W*math.cos(gam)/qS
            cdp=D/qS - CL*CL/(math.pi*E*AR)
            loc.append(cdp); samples.append((cdp,r["V"],h,r["gam"]))
        if loc:
            print(f"  {tag}/{i}: n={len(loc):4d}  cdp_eff med={statistics.median(loc):.5f} "
                  f"mean={statistics.mean(loc):.5f}")
cd=[s[0] for s in samples]
print(f"POOLED n={len(cd)} median={statistics.median(cd):.5f} mean={statistics.mean(cd):.5f} "
      f"sd={statistics.pstdev(cd):.5f}")
cd_s=sorted(cd)
print("  p10/p25/p50/p75/p90 = " + " ".join(f"{cd_s[int(p*len(cd_s))]:.5f}" for p in (.1,.25,.5,.75,.9)))
print(f"  config value JET_CDP = 0.01642")

# ---- cruise check: level segments at ~3100 m ----
print()
print("== level-flight speeds (PERCH/ALIGN, |hdot|<2 m/s, h>2800 m) ==")
for tag in ALL:
    for i in ALL[tag]:
        rows=[r for r in ALL[tag][i] if r["phase"] in ("ALIGN","PERCH")]
        v=[r["gs_s"] for r in rows if abs(r["hdot"])<2 and r["h_s"]>2800]
        if len(v)>50:
            h=[r["h_s"] for r in rows if abs(r["hdot"])<2 and r["h_s"]>2800]
            hm=statistics.median(h)
            e2t=math.sqrt(1.225/rho(hm+HOME))
            print(f"  {tag}/{i}: n={len(v):5d} TAS med={statistics.median(v):6.1f} "
                  f"max={max(v):6.1f} (h={hm:.0f}) => EAS med={statistics.median(v)/e2t:6.1f}")
