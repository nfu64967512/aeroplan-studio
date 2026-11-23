#!/usr/bin/env python3
"""Re-validate the integrator on the PN segment only (handover -> ground)."""
import pickle, math, os, statistics, sim

HAND = {   # from tot_standby.log "交接終端 PN @": (d_m, h_m, gamma_deg, hit_alt_m)
 ("RPK1",2):(4779,3203,33.8,428), ("RPK1",1):(4787,3158,33.4,48),
 ("RPK2",2):(4779,2829,30.6,519), ("RPK2",1):(4780,2662,29.1,20),
 ("RPK3",1):(4785,3510,36.3,1532),("RPK3",2):(4775,2286,25.6,12),
 ("RPL1",2):(4781,2897,31.2,390), ("RPL1",1):(4765,3277,34.5,877),
 ("RPL2",1):(4785,3250,34.2,648), ("RPL2",2):(4779,3091,32.9,2433),
 ("RPL3",2):(4779,2607,28.6,21),  ("RPL3",1):(4776,2887,31.2,23),
}
ALL=pickle.load(open(os.path.join(os.path.dirname(__file__),"traj.pkl"),"rb"))
print("== VALIDATION on the PN segment only (measured gamma(t) replayed) ==")
print(f"  {'run/uav':10s} {'V_hand':>7s} {'V_end_meas':>10s} {'V_end_sim':>9s} {'RMS':>6s} {'n':>5s}")
errs=[];ends=[]
for (tag,i),(dh,hh,gh,ha) in sorted(HAND.items()):
    rows=[r for r in ALL[tag][i] if r["phase"]=="DIVE" and r["mode"]=="dive"]
    cut=len(rows)
    for k,r in enumerate(rows):
        if r["agl"]<=0: cut=k;break
    rows=rows[:cut]
    st=None
    for k,r in enumerate(rows):
        if r["d"]<=dh: st=k;break
    if st is None: continue
    rows=[r for r in rows[st:] if r["agl"]>120]
    if len(rows)<100: continue
    V=rows[0]["V"]; e=[]
    for k in range(1,len(rows)):
        dt=rows[k]["t"]-rows[k-1]["t"]
        if dt<=0 or dt>0.5: continue
        gam=math.radians(rows[k-1]["gam"]); h=rows[k-1]["h_s"]+32
        V+=(sim.thrust(V,100.0)+sim.W*math.sin(gam)-sim.drag(V,h,gam))/sim.M*dt
        e.append(V-rows[k]["V"])
    rms=math.sqrt(sum(x*x for x in e)/len(e)); errs.append(rms)
    ends.append(V-rows[-1]["V"])
    print(f"  {tag}/{i:<8d} {rows[0]['V']:7.1f} {rows[-1]['V']:10.1f} {V:9.1f} {rms:6.1f} {len(rows):5d}")
print(f"  pooled RMS = {statistics.mean(errs):.1f} m/s ; end-point bias = "
      f"{statistics.mean(ends):+.1f} m/s (sd {statistics.pstdev(ends):.1f})")

print()
print("== measured terminal speed vs handover state (the KPI is 204 m/s) ==")
print(f"  {'run/uav':10s} {'h_hand':>7s} {'gam_hand':>9s} {'V@500m':>8s} {'V@200m':>8s} {'hit_alt':>8s}")
for (tag,i),(dh,hh,gh,ha) in sorted(HAND.items(), key=lambda kv: kv[1][2]):
    rows=[r for r in ALL[tag][i] if r["phase"]=="DIVE"]
    cut=len(rows)
    for k,r in enumerate(rows):
        if r["agl"]<=0: cut=k;break
    rows=rows[:cut]
    def at(L):
        for r in rows:
            if r["agl"]<=L: return r["V"]
        return float("nan")
    print(f"  {tag}/{i:<8d} {hh:7d} {gh:9.1f} {at(500):8.1f} {at(200):8.1f} {ha:8d}")
