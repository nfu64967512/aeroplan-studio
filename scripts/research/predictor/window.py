#!/usr/bin/env python3
"""Exact feasible window: gam_handover <= 30 deg AND touchdown >= 204 m/s."""
import math, sim
W=sim.W; K_SL=0.5*1.225*sim.S*sim.CD0

def gam_for_Veq(V, thr_pct, law="pn", v_req=210.0):
    """flight-path angle whose sea-level equilibrium speed is exactly V"""
    T=sim.thrust(V, thr_pct, law, v_req)
    need=K_SL*V*V - T
    s=need/W
    return math.degrees(math.asin(s)) if -1<=s<=1 else float('nan')

print("== equilibrium requirement (sea level, steady state) ==")
for thr in (25.0,100.0):
    print(f"  THR_MAX={thr:.0f}%  PN law  -> gamma needed for 204 m/s = "
          f"{gam_for_Veq(204,thr):.1f} deg")
    print(f"  THR_MAX={thr:.0f}%  full thr-> gamma needed for 204 m/s = "
          f"{gam_for_Veq(204,thr,'full'):.1f} deg")

print()
print("== finite-path boundary: handover at d=4800 m, PN flies the straight LOS ==")
for thr in (25.0,100.0):
    print(f"  --- THR_MAX={thr:.0f}% ---")
    for vh in (140,157,175,190):
        lo,hi=800.0,3400.0
        for _ in range(60):
            mid=0.5*(lo+hi)
            g=math.degrees(math.atan2(mid,4800.0))
            v=sim.fly(mid,4800.0,vh,lambda d,h: g,thr)["V"]
            if v<204: lo=mid
            else: hi=mid
        g=math.degrees(math.atan2(hi,4800.0))
        print(f"    V_hand={vh:3.0f} -> minimum handover altitude {hi:6.0f} m "
              f"(gamma_hand {g:4.1f} deg) for touchdown = 204 m/s")

print()
print("== the feasible window at d_gate = 4800 m (design thrust THR_MAX=25) ==")
h30=4800*math.tan(math.radians(30))
print(f"  upper bound  gamma_hand <= 30.0 deg  ->  h_hand <= {h30:.0f} m")
lo,hi=800.0,3400.0
for _ in range(60):
    mid=0.5*(lo+hi); g=math.degrees(math.atan2(mid,4800.0))
    v=sim.fly(mid,4800.0,157.0,lambda d,h: g,25.0)["V"]
    if v<204: lo=mid
    else: hi=mid
print(f"  lower bound  touchdown >= 204 m/s   ->  h_hand >= {hi:.0f} m "
      f"(gamma_hand {math.degrees(math.atan2(hi,4800)):.1f} deg)")
print(f"  => window width = {h30-hi:.0f} m of altitude "
      f"({math.degrees(math.atan2(h30,4800))-math.degrees(math.atan2(hi,4800)):.1f} deg of gamma)")
print(f"  today's flown value: h_hand = 3132 m nominal (measured 2286-3510), "
      f"gamma 25.6-36.3 deg")

print()
print("== equivalent: lower gamma by moving the gate OUT instead of descending ==")
for dg in (4800,5400,5600,6000,6500,7000):
    g=math.degrees(math.atan2(3132.0,dg))
    v25=sim.fly(3132.0,dg,157.0,lambda d,h,gg=g: gg,25.0)["V"]
    v100=sim.fly(3132.0,dg,157.0,lambda d,h,gg=g: gg,100.0)["V"]
    print(f"  gate_r={dg:5d}  h=3132 -> gamma_hand {g:5.1f} deg  "
          f"V_touch(THR25)={v25:6.1f}  V_touch(THR100)={v100:6.1f}")

print()
print("== run-in descent angle needed to arrive at the gate low enough ==")
print(f"  {'target h_gate':>13s} {'gam_hand':>9s} " +
      " ".join(f"from d={d}" for d in (6147,8000,10000,12000,16000)))
for hg in (2770,2600,2446,2374):
    row=[]
    for ds in (6147,8000,10000,12000,16000):
        g1=math.degrees(math.atan2(3132.0-hg, ds-4800.0))
        row.append(f"{g1:9.1f}")
    print(f"  {hg:13.0f} {math.degrees(math.atan2(hg,4800)):9.1f} " + " ".join(row))
print("  (PTCH_LIM_MIN_DEG / TECS_PITCH_MIN = -20 deg in cruise  => gam_run <= ~20 deg;")
print("   idle-thrust equilibrium at 157 TAS / 3100 m is gamma = "
      f"{math.degrees(math.asin(0.5*sim.rho(3100)*sim.S*sim.CD0*157**2/W)):.1f} deg,")
print("   so drag is NOT the limit on the run-in descent - TECS h_dem rate limiting is.)")
