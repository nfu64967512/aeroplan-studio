#!/usr/bin/env python3
"""V(h) profiles through the dive + energy-height rate -> effective drag."""
import pickle, math, os, statistics
G=9.80665
d=os.path.dirname(__file__)
ALL=pickle.load(open(os.path.join(d,"traj.pkl"),"rb"))

LEV=[3000,2500,2000,1500,1000,700,500,300,200,100,50]
print("=== V(total, m/s) at AGL levels, DIVE phase, descending pass ===")
print("run/uav  " + " ".join(f"{l:>5d}" for l in LEV))
for tag in ALL:
    for i in ALL[tag]:
        rows=[r for r in ALL[tag][i] if r["phase"]=="DIVE"]
        # cut at first agl<=0 (impact)
        cut=len(rows)
        for k,r in enumerate(rows):
            if r["agl"]<=0: cut=k; break
        rows=rows[:cut]
        out=[]
        for L in LEV:
            best=None
            for r in rows:
                if r["agl"]<=L:
                    best=r; break
            out.append(f"{best['V']:5.0f}" if best else "    -")
        print(f"{tag}/{i}   "+" ".join(out))

print()
print("=== gamma (deg, +down) at AGL levels ===")
print("run/uav  " + " ".join(f"{l:>5d}" for l in LEV))
for tag in ALL:
    for i in ALL[tag]:
        rows=[r for r in ALL[tag][i] if r["phase"]=="DIVE"]
        cut=len(rows)
        for k,r in enumerate(rows):
            if r["agl"]<=0: cut=k; break
        rows=rows[:cut]
        out=[]
        for L in LEV:
            best=None
            for r in rows:
                if r["agl"]<=L: best=r; break
            out.append(f"{best['gam']:5.1f}" if best else "    -")
        print(f"{tag}/{i}   "+" ".join(out))

print()
print("=== energy height E=V^2/2g + h (m) at AGL levels ===")
print("run/uav  " + " ".join(f"{l:>5d}" for l in LEV))
for tag in ALL:
    for i in ALL[tag]:
        rows=[r for r in ALL[tag][i] if r["phase"]=="DIVE"]
        cut=len(rows)
        for k,r in enumerate(rows):
            if r["agl"]<=0: cut=k; break
        rows=rows[:cut]
        out=[]
        for L in LEV:
            best=None
            for r in rows:
                if r["agl"]<=L: best=r; break
            out.append(f"{best['E']:5.0f}" if best else "    -")
        print(f"{tag}/{i}   "+" ".join(out))
