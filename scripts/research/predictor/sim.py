#!/usr/bin/env python3
"""Point-mass longitudinal integrator, calibrated on RPK/RPL, + validation replay."""
import pickle, math, os, statistics
G=9.80665; M=22.0; W=M*G; TFULL=M*G/0.7      # 308.21 N at servo=1.0
S=0.45; B=1.88; AR=B*B/S; E=0.9; PIEAR=math.pi*E*AR
CD0=0.01500                                   # measured, calib.py (n=1617, sd 0.00058)
HOME=32.0

def rho(h): return 1.225*(1.0-2.25577e-5*max(h,0.0))**4.2586

def drag(V,h,gam):
    qS=0.5*rho(h)*S*V*V
    CL=W*math.cos(gam)/max(qS,1e-6)
    return qS*(CD0+CL*CL/PIEAR)

def thrust(V, thr_max_pct, law="pn", v_req=210.0):
    if law=="pn":  cmd=max(0.0,min(1.0,0.36+0.030*(v_req-V)))
    elif law=="full": cmd=1.0
    else: cmd=float(law)
    return min(cmd, thr_max_pct/100.0)*TFULL

def v_eq(gam_deg, h, thr_max_pct, law="pn", v_req=210.0):
    """steady-state speed on a constant-gamma path at altitude h."""
    gam=math.radians(gam_deg); V=150.0
    for _ in range(400):
        T=thrust(V,thr_max_pct,law,v_req)
        f=T+W*math.sin(gam)-drag(V,h,gam)
        V+= f/M*0.25
        V=max(20.0,min(400.0,V))
    return V

# ---------------- validation: replay measured gamma(t), compare V(t) -------------
def validate():
    d=os.path.dirname(__file__)
    ALL=pickle.load(open(os.path.join(d,"traj.pkl"),"rb"))
    print("== VALIDATION: integrate dV/dt with the MEASURED gamma(t), compare V ==")
    print("   (thrust from tot_node PN law, THR_MAX=100 as flown)")
    print(f"   {'run/uav':9s} {'n':>5s} {'V0':>6s} {'Vend_meas':>9s} {'Vend_sim':>9s} {'rms err':>8s}")
    errs=[]
    for tag in sorted(ALL):
        for i in sorted(ALL[tag]):
            rows=[r for r in ALL[tag][i] if r["phase"]=="DIVE" and r["mode"]=="dive"]
            cut=len(rows)
            for k,r in enumerate(rows):
                if r["agl"]<=0: cut=k; break
            rows=[r for r in rows[:cut] if r["agl"]>100]
            if len(rows)<300: continue
            V=rows[0]["V"]; e=[]
            for k in range(1,len(rows)):
                dt=rows[k]["t"]-rows[k-1]["t"]
                if dt<=0 or dt>0.5: continue
                gam=math.radians(rows[k-1]["gam"]); h=rows[k-1]["h_s"]+HOME
                T=thrust(V,100.0)
                V+= (T+W*math.sin(gam)-drag(V,h,gam))/M*dt
                e.append(V-rows[k]["V"])
            rms=math.sqrt(sum(x*x for x in e)/len(e))
            errs.append(rms)
            print(f"   {tag}/{i:<6d} {len(rows):5d} {rows[0]['V']:6.1f} "
                  f"{rows[-1]['V']:9.1f} {V:9.1f} {rms:8.1f}")
    print(f"   pooled RMS of V error = {statistics.mean(errs):.1f} m/s "
          f"(median {statistics.median(errs):.1f}) over a 90-220 m/s range")

# ---------------- profile flight ------------------------------------------------
def fly(h0, d0, v0, gam_of_d, thr_max_pct, law="pn", v_req=210.0, dt=0.02, hstop=0.0):
    """Fly from (h0 above target, d0 horizontal) to h=hstop. gam_of_d(d,h)->deg."""
    h=h0; d=d0; V=v0; t=0.0; path=0.0; Emin=None
    while h>hstop and t<400.0 and d>-2000:
        gam=math.radians(max(0.0,min(85.0,gam_of_d(d,h))))
        T=thrust(V,thr_max_pct,law,v_req)
        V+= (T+W*math.sin(gam)-drag(V,h,gam))/M*dt
        V=max(20.0,V)
        h-= V*math.sin(gam)*dt
        d-= V*math.cos(gam)*dt
        path+=V*dt; t+=dt
    return dict(V=V, t=t, d=d, path=path, h=h)

def gam_two_seg(d_gate, h_gate, h0, d0):
    """level-ish seg1 from d0 to d_gate shedding (h0-h_gate), then straight-in."""
    g1=math.degrees(math.atan2(h0-h_gate, max(d0-d_gate,1.0)))
    def f(d,h):
        if d>d_gate: return g1
        return math.degrees(math.atan2(max(h,0.0), max(d,1.0)))   # pure LOS chase
    return f, g1

if __name__=="__main__":
    validate()
    print()
    print("== steady-state (terminal) speed on a constant-gamma path, sea level ==")
    print(f"   {'gamma':>6s} {'THR_MAX=25':>11s} {'THR_MAX=100':>12s}   (m/s; PN throttle law, v_req=210)")
    for g in (10,15,20,25,27,30,35,40,45,50,60):
        print(f"   {g:6.0f} {v_eq(g,0,25):11.1f} {v_eq(g,0,100):12.1f}")
    print()
    print("== same, at 3100 m (where handover happens) ==")
    print(f"   {'gamma':>6s} {'THR_MAX=25':>11s} {'THR_MAX=100':>12s}")
    for g in (0,5,10,15,20,25,27,30,35,40):
        print(f"   {g:6.0f} {v_eq(g,3100,25):11.1f} {v_eq(g,3100,100):12.1f}")
