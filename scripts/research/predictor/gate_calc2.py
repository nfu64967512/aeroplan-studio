import math
G0 = 9.80665

def raw_gate(h, V, gam_deg=0.0, th_deg=35.0, phi=0.80, nz=0.05):
    gam = math.radians(gam_deg)
    r_turn = V*V/(G0*(math.cos(gam)-nz))
    th = math.radians(th_deg)
    h_arc = r_turn*(1-math.cos(th))
    if h > h_arc:
        d_min = r_turn*math.sin(th) + (h-h_arc)/math.tan(th)
    else:
        d_min = math.sqrt(max(h*(2*r_turn-h), 0.0))
    return d_min/phi, r_turn

print("== A. dgate/dh = 1/(phi*tan(th))  (unclamped slope) ==")
for th in (25,28,30,33,35):
    s = 1.0/(0.80*math.tan(math.radians(th)))
    print(f"  th={th}: slope={s:.3f} m gate per m alt ; dh=1224m -> dgate={s*1224:.0f} m")

print()
print("== B. asymptotic handover LOS angle under an UNCLAMPED dynamic gate ==")
print("   gamma_eff -> atan(phi*tan(th)) as h grows (offset term pulls it lower)")
for th in (25,28,30,33,35):
    a = math.degrees(math.atan(0.80*math.tan(math.radians(th))))
    print(f"  th={th}: gamma_asym={a:.1f} deg", end="  |  ")
    for V in (120.0, 157.0, 200.0):
        g,_ = raw_gate(3100.0, V, 0.0, th)
        print(f"V={V:.0f}: gamma={math.degrees(math.atan2(3100.0,g)):.1f}", end=" ")
    print()

print()
print("== C. offset constant C/phi = r_turn*(sin th - (1-cos th)/tan th)/phi ==")
for V in (111.0,120.0,157.0,185.0,216.0):
    r = V*V/(G0*0.95)
    for th in (30,35):
        t = math.radians(th)
        C = r*(math.sin(t)-(1-math.cos(t))/math.tan(t))
        print(f"  V={V:5.0f} r_turn={r:6.0f} th={th}: C/phi={C/0.8:7.0f} m")

print()
print("== D. h_crit: largest handover altitude that still stays UNDER a clamp ==")
for cap,lab in ((6000.0,"gate_r_max=6000"),(5780.0,"0.95*ring nominal 6084"),
                (5840.0,"0.95*ring reground 6147"),(6084.0,"ring nominal"),(6147.0,"ring reground")):
    print(f"  -- cap {lab} --")
    for th in (25,28,30,33,35):
        row=[]
        for V in (120.0,157.0,200.0):
            r = V*V/(G0*0.95); t=math.radians(th)
            C = r*(math.sin(t)-(1-math.cos(t))/math.tan(t))
            h_arc = r*(1-math.cos(t))
            hc = (cap*0.8 - C)*math.tan(t)
            row.append(f"V={V:.0f}:{hc:6.0f}")
        print(f"     th={th}: h_crit " + "  ".join(row))

print()
print("== E. max dive_angle so that raw gate(h=3510,V) <= 0.95*ring(perch/tan) ==")
for perch in (3100.0,):
    for V in (120.0,157.0,200.0):
        for th in (30,33,35):
            lo,hi = 5.0, 45.0
            best=None
            for _ in range(80):
                mid=(lo+hi)/2
                ring = perch/math.tan(math.radians(mid))
                g,_ = raw_gate(3510.0, V, 0.0, th)
                if g <= 0.95*ring: lo=mid; best=mid
                else: hi=mid
            ring=perch/math.tan(math.radians(best)) if best else float('nan')
            g,_=raw_gate(3510.0,V,0.0,th)
            print(f"  perch={perch:.0f} V={V:.0f} th={th}: raw_gate={g:6.0f} "
                  f"-> dive_angle <= {best:5.2f} deg (ring {ring:6.0f} m)")

print()
print("== F. same, but with h=2900 (if handover altitude spread were cut in half) ==")
for V in (157.0,):
    for h in (3510.0,3100.0,2700.0,2300.0,1900.0,1500.0):
        for th in (35,):
            g,_=raw_gate(h,V,0.0,th)
            need = math.degrees(math.atan(3100.0/(g/0.95)))
            print(f"  h={h:5.0f} th={th} V={V:.0f}: raw_gate={g:6.0f}  "
                  f"needs dive_angle<= {need:5.2f} deg ; fits in current ring 6147? "
                  f"{'YES' if g<=0.95*6147 else 'no'}")

print()
print("== G. what the flight actually gets: effective gate & gamma, current geometry ==")
print("   clamps: min(raw, 6000, ring 6147), floor 2500")
meas=[("RPK1",1,3162,124.0,0.0),("RPK1",2,3206,176.1,21.0),("RPK2",1,2668,185.6,0.0),
      ("RPK2",2,2830,154.9,0.0),("RPK3",1,3509,127.2,0.0),("RPK3",2,2293,215.9,0.0),
      ("RPL1",1,3278,201.4,0.0),("RPL1",2,2901,136.7,0.0),("RPL2",1,3250,121.5,0.0),
      ("RPL2",2,3087,118.0,0.0),("RPL3",1,2894,180.5,39.4),("RPL3",2,2611,143.4,0.0)]
for th in (30,35):
    print(f"  -- th={th} --")
    n_cl=0; gams=[]
    for (t,s,h,gs,vz) in meas:
        V=math.hypot(gs,vz); gam=math.degrees(math.atan2(vz,max(gs,1e-6)))
        g,_=raw_gate(h,V,gam,th)
        eff=max(2500.0,min(g,6000.0,6147.0))
        cl = eff<g-0.5
        n_cl+=1 if cl else 0
        gg=math.degrees(math.atan2(h,eff)); gams.append(gg)
        print(f"    {t} U{s}: h={h:4d} V={V:5.1f} raw={g:6.0f} eff={eff:5.0f} "
              f"{'CLAMPED' if cl else 'live   '} gamma={gg:5.1f}")
    print(f"    clamped {n_cl}/12 ; gamma span {min(gams):.1f}-{max(gams):.1f} "
          f"(spread {max(gams)-min(gams):.1f} deg)")
print()
print("  -- baseline: fixed gate 4800 / 5600 --")
for fixed in (4800.0,5600.0,6000.0):
    gams=[math.degrees(math.atan2(h,fixed)) for (_,_,h,_,_) in meas]
    bad=sum(1 for g in gams if g>=33.8); good=sum(1 for g in gams if g<=29.1)
    print(f"    gate={fixed:.0f}: gamma {min(gams):.1f}-{max(gams):.1f} "
          f"(spread {max(gams)-min(gams):.1f}) ; <=29.1 pass-band {good}/12 ; >=33.8 fail-band {bad}/12")
