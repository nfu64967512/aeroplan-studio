#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""N1b: WHERE in the dive does production command its huge negative n_z,
and does the ordering fix land on the sorties that already fail?"""
from __future__ import annotations
import math, sys, pathlib, statistics as st
HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(ROOT/"scripts"/"research"/"ditcg"))
import truth32, r1_harness as H, r1_replay as R                  # noqa: E402
AFTER = ROOT/"scripts"/"wsl"/"sar_swarm_patches"/"tot_node.py"

def q(v,p):
    v=sorted(v); return v[min(len(v)-1,max(0,int(round(p*(len(v)-1)))))]

bind = H.Bind(AFTER.read_text(encoding="utf-8"), "worktree")
acs=[]
for run in truth32.RUNS:
    tlat,tlon = truth32._read_target(run)
    for sysid,rows in sorted(truth32._rows(run).items()):
        s = R.states(run,sysid,rows,tlat,tlon)
        if s: acs.append(dict(tag=f"{run}-U{sysid}",st=s,tlat=tlat,tlon=tlon))

A0 = R.run_config(bind, dict(nz_clamp_after_floor=False, pn_nz_min=-1.0), acs, bind.ns)
A1 = R.run_config(bind, dict(nz_clamp_after_floor=True,  pn_nz_min=-1.0), acs, bind.ns)

# --- 1. where in the dive is the demand unbounded? -----------------------
print("A0 cmd_nz < -2.0  : where in the dive (frac = normalised dive time)")
fr = [x["frac"] for x in A0 if x["cmd_nz"] < -2.0]
print(f"  n = {len(fr)}/{len(A0)}   frac p10 {q(fr,.10):.3f}  med {q(fr,.50):.3f}  p90 {q(fr,.90):.3f}")
for lo,hi in ((0.0,.25),(.25,.50),(.50,.75),(.75,1.01)):
    sub=[x for x in A0 if lo<=x["frac"]<hi]
    bad=[x for x in sub if x["cmd_nz"]<-2.0]
    print(f"  frac [{lo:.2f},{hi:.2f})  {100*len(bad)/max(1,len(sub)):6.2f}%  "
          f"cmd_nz min {min([x['cmd_nz'] for x in sub],default=0):9.3f}")

# --- 2. is production sitting on the -80 deg pitch rail? -----------------
rail = sum(1 for x in A0 if math.degrees(x["pitch"]) <= -79.999)
rail1= sum(1 for x in A1 if math.degrees(x["pitch"]) <= -79.999)
print(f"\npitch railed at -80deg :  A0 {rail}/{len(A0)} = {100*rail/len(A0):.2f}%"
      f"    A1 {rail1}/{len(A1)} = {100*rail1/len(A1):.2f}%")
per={}
for x in A0:
    if math.degrees(x["pitch"])<=-79.999: per[x["tag"]]=per.get(x["tag"],0)+1
print(f"sorties touching the rail: {len(per)}/32")

# --- 3. does the change land on the known failures? ----------------------
try:
    import importlib; t32 = truth32.load()
    err = {r["tag"]: r.get("r_touch_m") for r in t32}
except Exception as e:                                           # noqa: BLE001
    err = {}; print("(no err column:", e, ")")
chg={}
for x,y in zip(A0,A1):
    chg[x["tag"]] = max(chg.get(x["tag"],0.0), abs(math.degrees(x["pitch"]-y["pitch"])))
rows=[(k,v,err.get(k)) for k,v in chg.items()]
rows.sort(key=lambda r:-r[1])
print(f"\n{'sortie':<12}{'max |dpitch| A0->A1':>21}{'r_touch_m':>18}")
for k,v,e in rows[:10]:
    print(f"{k:<12}{v:21.3f}{(f'{e:.2f}' if isinstance(e,float) else str(e)):>18}")
ok=[(v,e) for _,v,e in rows if isinstance(e,float)]
if len(ok)>4:
    import statistics
    xs=[v for v,_ in ok]; ys=[e for _,e in ok]
    rx=sorted(range(len(xs)),key=lambda i:xs[i]); ry=sorted(range(len(ys)),key=lambda i:ys[i])
    ra=[0]*len(xs); rb=[0]*len(ys)
    for r,i in enumerate(rx): ra[i]=r
    for r,i in enumerate(ry): rb[i]=r
    mx,my=statistics.mean(ra),statistics.mean(rb)
    num=sum((a-mx)*(b-my) for a,b in zip(ra,rb))
    den=math.sqrt(sum((a-mx)**2 for a in ra)*sum((b-my)**2 for b in rb))
    print(f"\nspearman(max |dpitch|, touchdown err) = {num/den:+.3f}   n={len(ok)}")
