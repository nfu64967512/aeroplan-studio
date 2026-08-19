#!/usr/bin/env python3
"""Energy budget: does descending earlier cost touchdown speed?"""
import math, sim
G=sim.G; W=sim.W; M=sim.M

H0=3132.0        # perch 3100 rel HOME + 32 m HOME AMSL = height above the sea target
D_RING=6147.0    # ring radius actually flown (tot_standby.log, terrain-corrected)
D_GATE=4800.0    # gate_r actually flown

def const_gamma(g): return lambda d,h: g

print("="*78)
print("Q3a  CONSTANT-GAMMA DESCENT FROM 3132 m -- touchdown speed vs entry speed")
print("     (touchdown = h reaches 0; PN throttle law v_req=210)")
for thr in (25.0,100.0):
    print(f"\n  --- THR_MAX = {thr:.0f} %  (T_max = {thr/100*sim.TFULL:.0f} N, T/W = {thr/100*sim.TFULL/W:.2f}) ---")
    print(f"    {'gamma':>6s} {'path(m)':>8s} " + " ".join(f"v0={v:<5.0f}" for v in (100,135,157,180,210)))
    for g in (10,15,20,25,26,27,28,30,35,40,45):
        row=[]
        for v0 in (100,135,157,180,210):
            r=sim.fly(H0,H0/math.tan(math.radians(g)),v0,const_gamma(g),thr)
            row.append(f"{r['V']:7.1f}")
        print(f"    {g:6.0f} {H0/math.sin(math.radians(g)):8.0f} " + " ".join(row))

print()
print("="*78)
print("Q3b  EARLY vs LATE, same start (ring d=6147 m, h=3132 m), same target")
print("     LATE : hold 3132 m to the gate, then straight-in  (what the system does now)")
print("     EARLY: shed altitude on the run-in so the handover LOS is shallower")
print()
for thr in (25.0,100.0):
    print(f"  --- THR_MAX = {thr:.0f} % ---")
    print(f"    {'d_start':>8s} {'gam_run':>8s} {'h_gate':>7s} {'gam_hand':>9s} "
          f"{'V_hand':>7s} {'V_touch':>8s} {'t_total':>8s}")
    for d_start,h_gate in [(D_RING,3132),          # LATE: no run-in descent
                           (D_RING,2900),(D_RING,2770),(D_RING,2446),
                           (8000,2770),(8000,2446),(8000,2100),
                           (12000,2770),(12000,2446),(12000,2100),(12000,1600)]:
        # phase 1: cruise level from d_start to the point where the run-in glide begins,
        #          here we glide the whole d_start->D_GATE leg at constant angle
        g1=math.degrees(math.atan2(H0-h_gate, d_start-D_GATE)) if d_start>D_GATE else 0.0
        r1=sim.fly(H0, d_start, 157.0, const_gamma(g1), thr, hstop=h_gate) if g1>0 else \
           dict(V=157.0,t=(d_start-D_GATE)/157.0,h=H0)
        gh=math.degrees(math.atan2(h_gate, D_GATE))
        r2=sim.fly(h_gate, D_GATE, r1["V"], const_gamma(gh), thr)
        print(f"    {d_start:8.0f} {g1:8.1f} {h_gate:7.0f} {gh:9.1f} "
              f"{r1['V']:7.1f} {r2['V']:8.1f} {r1['t']+r2['t']:8.1f}")
    print()

print("="*78)
print("Q4  Does a profile exist with gam_handover <= 30 deg AND touchdown >= 204 m/s ?")
print("    handover fixed at d = 4800 m; sweep handover altitude; PN then chases the LOS")
for thr in (25.0,100.0):
    print(f"\n  --- THR_MAX = {thr:.0f} % ---")
    print(f"    {'h_hand':>7s} {'gam_hand':>9s} " + " ".join(f"V_hand={v:<4.0f}" for v in (140,157,180,200)))
    for hh in (3132,2900,2770,2600,2446,2240,2000,1750,1500,1250,1000):
        gh=math.degrees(math.atan2(hh,D_GATE))
        row=[]
        for vh in (140,157,180,200):
            r=sim.fly(hh,D_GATE,vh,const_gamma(gh),thr)
            ok="*" if r["V"]>=204 else " "
            row.append(f"{r['V']:9.1f}{ok}")
        print(f"    {hh:7.0f} {gh:9.1f} " + " ".join(row))
print("\n    ( * = meets the 204 m/s touchdown KPI )")

print()
print("="*78)
print("Relaxation: how far does the aircraft need to converge on the equilibrium?")
for thr in (25.0,100.0):
    print(f"  THR_MAX={thr:.0f}%")
    for g in (25,27,30,35):
        ve=sim.v_eq(g,0,thr)
        k=0.5*1.225*sim.S*sim.CD0
        tau=M/(2*k*ve)
        print(f"    gamma={g:2.0f}  V_eq(SL)={ve:6.1f}  tau={tau:5.1f} s  "
              f"path/tau={ve*tau:6.0f} m  altitude/tau={ve*tau*math.sin(math.radians(g)):6.0f} m")
