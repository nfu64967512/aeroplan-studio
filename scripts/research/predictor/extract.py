#!/usr/bin/env python3
"""Extract h-V trajectories from RPK/RPL artifacts; measure real L/D and decay."""
import csv, math, os, statistics, sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
TAGS = ["RPK1","RPK2","RPK3","RPL1","RPL2","RPL3"]
G = 9.80665
HOME_AMSL = 32.0   # from standby logs: HOME GPS alt ~ +32 m AMSL, target pinned 0 AMSL

def load(tag):
    p = os.path.join(ROOT, "runs", "artifacts", tag, f"tot_{tag}.csv")
    per = {}
    with open(p) as f:
        for row in csv.DictReader(f):
            i = int(row["sysid"])
            per.setdefault(i, []).append(dict(
                t=float(row["t"]), lat=float(row["lat"]), lon=float(row["lon"]),
                alt=float(row["alt"]), gs=float(row["gs"]), d=float(row["d_slot"]),
                mode=row["mode"], vcmd=float(row["v_cmd"]), phase=row["phase"],
                agl=float(row["agl"])))
    return per

def smooth(xs, n):
    out=[]
    for k in range(len(xs)):
        a=max(0,k-n); b=min(len(xs),k+n+1)
        out.append(sum(xs[a:b])/(b-a))
    return out

def derive(rows, win=10):
    """add hdot (positive down), V, gamma, specific energy"""
    t=[r["t"] for r in rows]
    h=[r["alt"] for r in rows]
    hs=smooth(h,win)
    gs=smooth([r["gs"] for r in rows],win)
    hdot=[]
    for k in range(len(rows)):
        a=max(0,k-win); b=min(len(rows)-1,k+win)
        dt=t[b]-t[a]
        hdot.append(-(hs[b]-hs[a])/dt if dt>1e-6 else 0.0)   # +down
    for k,r in enumerate(rows):
        r["h_s"]=hs[k]; r["gs_s"]=gs[k]; r["hdot"]=hdot[k]
        r["V"]=math.hypot(gs[k], hdot[k])
        r["gam"]=math.degrees(math.atan2(hdot[k], max(gs[k],1e-6)))
        r["E"]=r["V"]**2/(2*G)+ (hs[k]+HOME_AMSL)   # energy height (m)
    return rows

def rho(h_amsl):
    return 1.225*(1.0-2.25577e-5*max(h_amsl,0.0))**4.2586

print("=== per-sortie DIVE summary ===")
print(f"{'run':5s} {'uav':3s} {'t_go':>6s} {'h_go':>6s} {'V_go':>6s} {'gam_go':>7s} "
      f"{'h_min':>6s} {'V@h=200':>8s} {'V_end':>6s} {'gs_end':>6s} {'gam_end':>7s} {'dur':>5s}")
ALL={}
for tag in TAGS:
    per=load(tag)
    ALL[tag]={}
    for i in sorted(per):
        rows=derive(per[i])
        ALL[tag][i]=rows
        dv=[r for r in rows if r["phase"]=="DIVE" and r["mode"] in ("dive","expended")]
        if not dv:
            print(tag,i,"no dive"); continue
        # first sample where mode=='dive'
        st=[r for r in dv if r["mode"]=="dive"]
        if not st: continue
        g0=st[0]
        # terminal: last sample with agl>0 before first agl<=0
        term=None
        for r in dv:
            if r["agl"]<=0: term=r; break
        if term is None: term=dv[-1]
        # V at 200m agl
        v200=None
        for r in dv:
            if r["agl"]<=200 and v200 is None: v200=r
        hmin=min(r["agl"] for r in dv)
        print(f"{tag:5s} {i:3d} {g0['t']:6.1f} {g0['agl']:6.0f} {g0['V']:6.1f} {g0['gam']:7.1f} "
              f"{hmin:6.1f} {(v200['V'] if v200 else float('nan')):8.1f} "
              f"{term['V']:6.1f} {term['gs']:6.1f} {term['gam']:7.1f} {term['t']-g0['t']:5.1f}")

import pickle
with open(os.path.join(os.path.dirname(__file__),"traj.pkl"),"wb") as f:
    pickle.dump(ALL,f)
