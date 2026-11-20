#!/usr/bin/env python3
"""Empirical sensitivity of terminal speed to handover altitude, vs the naive energy model."""
import pickle, math, os, statistics
from validate2 import HAND
ALL=pickle.load(open(os.path.join(os.path.dirname(__file__),"traj.pkl"),"rb"))
G=9.80665

pts=[]
for (tag,i),(dh,hh,gh,ha) in HAND.items():
    rows=[r for r in ALL[tag][i] if r["phase"]=="DIVE"]
    cut=len(rows)
    for k,r in enumerate(rows):
        if r["agl"]<=0: cut=k;break
    rows=rows[:cut]
    v=None
    for r in rows:
        if r["agl"]<=500: v=r["V"]; break
    if v is None: continue
    # energy height at handover and at 500 m
    Eh=None
    for r in rows:
        if r["d"]<=dh: Eh=r["E"]; break
    E5=None
    for r in rows:
        if r["agl"]<=500: E5=r["E"]; break
    pts.append((hh,gh,v,Eh,E5,tag,i))

def lin(xs,ys):
    n=len(xs); mx=sum(xs)/n; my=sum(ys)/n
    sxy=sum((x-mx)*(y-my) for x,y in zip(xs,ys)); sxx=sum((x-mx)**2 for x in xs)
    b=sxy/sxx; a=my-b*mx
    syy=sum((y-my)**2 for y in ys)
    r2=(sxy*sxy)/(sxx*syy) if syy>0 else 0.0
    return a,b,r2

h=[p[0] for p in pts]; v=[p[2] for p in pts]; g=[p[1] for p in pts]
a,b,r2=lin(h,v)
print("== measured: terminal speed (V at 500 m AGL) vs handover altitude, n=%d ==" % len(pts))
for p in sorted(pts):
    print(f"   {p[5]}/{p[6]}  h_hand={p[0]:5d}  gam={p[1]:5.1f}  V@500={p[2]:6.1f}  "
          f"E_hand={p[3]:6.0f}  E@500={p[4]:6.0f}  dE={p[4]-p[3]:+7.0f}")
print(f"\n   fit  V500 = {a:.1f} + {b:.5f} * h_hand   (R2={r2:.3f})")
print(f"   measured sensitivity  dV/dh_hand = {b*1000:.1f} m/s per 1000 m of handover altitude")
vm=statistics.mean(v)
print(f"   naive energy-conservation sensitivity dV/dh = g/V = {G/vm*1000:.1f} m/s per 1000 m")
print(f"   => the real budget is {b/(G/vm)*100:.0f}% of the naive one")
print(f"   V@500 spread over a {max(h)-min(h)} m spread in handover altitude: "
      f"{min(v):.1f} - {max(v):.1f} m/s (range {max(v)-min(v):.1f})")
a2,b2,r22=lin(g,v)
print(f"\n   vs handover gamma: V500 = {a2:.1f} + {b2:.3f}*gamma  (R2={r22:.3f}), "
      f"{b2:.2f} m/s per degree")

print()
print("== how much of the potential energy actually survives as kinetic energy? ==")
lost=[p[3]-p[4] for p in pts]; drop=[p[0]-500 for p in pts]
print(f"   mean energy-height LOST between handover and 500 m AGL : {statistics.mean(lost):6.0f} m")
print(f"   mean altitude dropped over the same stretch             : {statistics.mean(drop):6.0f} m")
print(f"   => {statistics.mean(lost)/statistics.mean(drop)*100:.0f}% of the potential energy released")
print(f"      is eaten by drag; the kinetic-energy store is already full and cannot take it.")
