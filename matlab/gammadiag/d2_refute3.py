# -*- coding: utf-8 -*-
import csv,math,os,statistics as st,collections,json
H=os.path.dirname(os.path.abspath(__file__))
def rd(p): return list(csv.DictReader(open(p,encoding='utf-8')))
def ff(r,k):
    try: return float(r[k])
    except: return float('nan')
T=rd(os.path.join(H,'m1_ticks.csv'))
for r in T:
    for k in r:
        if k not in ('tag','cls'): r[k]=ff(r,k)
by=collections.defaultdict(list)
for r in T: by[r["tag"]].append(r)
for k in list(by): by[k]=sorted(by[k],key=lambda r:r["t"])
TR={r['tag']:r for r in rd(os.path.join(H,'..','data','truth32.csv'))}
GATE={}
for tr in set(k.split('-')[0] for k in by):
    p=os.path.join(H,'..','..','runs','artifacts',tr,'contract_%s.json'%tr)
    GATE[tr]=float(json.load(open(p,encoding='utf-8'))['values']['gate_r']) if os.path.exists(p) else 4700.0

def gain(rows,xk):
    sxx=sum(r[xk]**2 for r in rows); sxy=sum(r[xk]*r['gdot_meas_dps'] for r in rows)
    syy=sum(r['gdot_meas_dps']**2 for r in rows)
    return (sxy/sxx if sxx>0 else float('nan')), (sxy/math.sqrt(sxx*syy) if sxx*syy>0 else float('nan'))

print("=== [9] how much of M1's window is really pre-PN?  (PN latches at d<=gate_r) ===")
pre=post=0; frs=[]
for k,v in by.items():
    g=GATE[k.split('-')[0]]
    a=sum(1 for r in v if r['d_m']>g); pre+=a; post+=len(v)-a
    frs.append(a/len(v))
print("  ticks with d > gate_r (PN not yet latched): %d / %d = %.1f%%   per-sortie med %.1f%%"%(
    pre,pre+post,100*pre/(pre+post),100*st.median(frs)))
print("  -> M2's '85.2%% of the window has no PN command' does NOT apply to M1's window.")
sub=[r for r in T if r['d_m']<=GATE[r['tag'].split('-')[0]]]
for lbl,key in (("LOS-floor demand","gdot_los_dps"),("nz-clamped PN","gdot_nz_dps")):
    g0,c0=gain(T,key); g1,c1=gain(sub,key)
    print("  gain vs %-17s full window %.4f (r%+.3f)   d<=gate_r only %.4f (r%+.3f)"%(lbl,g0,c0,g1,c1))

print()
print("=== [10] terminal geometry: is it pursuit?  (last 5 s, gam vs gam_LOS) ===")
rows=[]
for k,v in by.items():
    t1=v[-1]['t']
    tail=[r for r in v if r['t']>=t1-5.0]
    dg=[r['gdot_los_dps']*1.0 for r in tail]              # = gam_cmd - gam (pn_lead=1)
    los=[abs(x) for x in dg]
    rows.append((k,st.median(los),st.median([abs(r['gdot_meas_dps']) for r in tail]),
                 st.median([abs(r['gdot_nz_dps']) for r in tail]),
                 ff(TR[k],'err_touch_m') if k in TR else float('nan')))
hit=[r for r in rows if r[4]<=10]; mis=[r for r in rows if r[4]>10]
for lbl,g in (("HIT<=10m (n=%d)"%len(hit),hit),("MISS>10m (n=%d)"%len(mis),mis)):
    print("  %-16s |gam-gam_LOS| med %6.2f deg | |gdot_act| %5.2f | |PN demand| %5.2f deg/s"%(
        lbl,st.median([r[1] for r in g]),st.median([r[2] for r in g]),st.median([r[3] for r in g])))

print()
print("=== [11] the ceiling is speed-dependent: throttle policy shrinks it ===")
allv=sorted(r['V_ms'] for r in T)
n=len(allv)
print("  V p05 %.0f  p50 %.0f  p95 %.0f m/s"%(allv[int(.05*n)],allv[n//2],allv[int(.95*n)]))
for V in (130,160,190,215):
    print("    V=%3d -> pull-up ceiling (3.0-cos33)*g/V = %+5.2f deg/s ; push ceiling %+5.2f"%(
        V,math.degrees((3.0-0.84)*9.80665/V),math.degrees((-1.0-0.84)*9.80665/V)))
# correlate terminal V with err
tv=[];te=[]
for k,v in by.items():
    if k in TR:
        tv.append(st.median([r['V_ms'] for r in v[-60:]])); te.append(ff(TR[k],'err_touch_m'))
def spear(x,y):
    def rk(a):
        o=sorted(range(len(a)),key=lambda i:a[i]); r=[0]*len(a)
        for p,i in enumerate(o): r[i]=p
        return r
    rx,ry=rk(x),rk(y); mx,my=st.mean(rx),st.mean(ry)
    num=sum((rx[i]-mx)*(ry[i]-my) for i in range(len(x)))
    den=math.sqrt(sum((v-mx)**2 for v in rx)*sum((v-my)**2 for v in ry))
    return num/den if den else float('nan')
print("  rho(terminal V, err_touch) = %+.3f"%spear(tv,te))

print()
print("=== [12] sensitivity of every headline number to n_vert (unverified = 5.0) ===")
for nv in (3.0,5.0,7.0):
    s=nv/5.0
    ok=[r for r in T if abs(r['gdot_raw_dps'])<1e4]
    over=sum(1 for r in ok if s*abs(r['gdot_raw_dps'])>max(r['cap_up_dps'],abs(r['cap_dn_dps'])))/len(ok)
    I=sum(abs(r['gdot_meas_dps']) for r in ok)/sum(s*abs(r['gdot_raw_dps']) for r in ok)
    print("  n_vert=%.0f : raw-PN demand>ceiling %5.2f%% of ticks ; delivery int|act|/int|dem| %.3f"%(nv,100*over,I))
