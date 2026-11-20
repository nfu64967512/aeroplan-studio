# -*- coding: utf-8 -*-
import csv,math,os,statistics as st,collections,json
H=os.path.dirname(os.path.abspath(__file__)); G=9.80665
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
    GATE[tr]=float(json.load(open(p,encoding='utf-8'))['values']['gate_r'])
def gain(rows,xk):
    sxx=sum(r[xk]**2 for r in rows); sxy=sum(r[xk]*r['gdot_meas_dps'] for r in rows)
    syy=sum(r['gdot_meas_dps']**2 for r in rows)
    return (sxy/sxx if sxx>0 else float('nan')),(sxy/math.sqrt(sxx*syy) if sxx*syy>0 else float('nan'))
sub=[r for r in T if r['d_m']<=GATE[r['tag'].split('-')[0]]]
print("=== [13] the 15.1x gap, decomposed (each step changes ONE thing) ===")
g0,c0=gain(T,'gdot_los_dps'); g1,c1=gain(sub,'gdot_los_dps'); g2,c2=gain(sub,'gdot_nz_dps')
print("  A M1 as published (mode=='dive' window, LOS-floor demand) : gain %.4f  r %+.3f  -> 1/g = %.1f"%(g0,c0,1/g0))
print("  B fix WINDOW only (d<=gate_r, i.e. PN actually running)   : gain %.4f  r %+.3f   [x%.2f]"%(g1,c1,g1/g0))
print("  C fix DEMAND too (PN law's own output, before LOS floor)  : gain %.4f  r %+.3f   [x%.2f]"%(g2,c2,g2/g1))
print("  D residual = real closed-loop lag                          : x%.2f  (1/%.4f)"%(1/g2,g2))
print("  -> 15.1x = %.1fx window  x  %.1fx demand-definition  x  %.1fx real lag"%(g1/g0,g2/g1,1/g2))
print()
print("=== [14] airframe nz envelope vs the guidance clamp -- which is the real ceiling? ===")
for k in ('nz_meas',):
    a=sorted(r[k] for r in T if not math.isnan(r[k]))
    n=len(a)
    print("  CSV-diff nz  : p01 %.2f p50 %.2f p99 %.2f  min %.2f max %.2f"%(a[int(.01*n)],a[n//2],a[int(.99*n)],a[0],a[-1]))
print("  M2 accelerometer nz (dataflash) : min -2.29  max +1.17")
print("  M3 accelerometer nz (dataflash) : max +4.71 (RRD2/1); alpha=0 already gives 3.1-4.8 g")
print("  sorties whose measured nz left the guidance clamp [-1,+3]: lo %d/32   hi %d/32"%(
  sum(1 for k in by if min(r['nz_meas'] for r in by[k])<-1.0),
  sum(1 for k in by if max(r['nz_meas'] for r in by[k])>3.0)))
ok=[r for r in T if abs(r['gdot_raw_dps'])<1e4]
for lo,hi,lbl in ((-1.0,3.0,"guidance clamp [-1,+3] (brief)"),(-2.3,4.7,"MEASURED airframe envelope [-2.3,+4.7]")):
    o=0
    for r in ok:
        cg=math.cos(math.radians(r['gam_deg']))
        cu=math.degrees((hi-cg)*G/r['V_ms']); cd=math.degrees((lo-cg)*G/r['V_ms'])
        if r['gdot_raw_dps']>cu or r['gdot_raw_dps']<cd: o+=1
    print("  raw PN demand exceeds %-40s : %.2f%% of ticks"%(lbl,100*o/len(ok)))
print()
print("=== [15] RECONCILIATION: how do '16x impossible' and 25/32 hits coexist? ===")
rows=[]
for k,v in by.items():
    if k not in TR: continue
    t1=v[-1]['t']; tail=[r for r in v if r['t']>=t1-3.0]
    dt=0.05
    rows.append(dict(tag=k,err=ff(TR[k],'err_touch_m'),zres=abs(ff(TR[k],'z_res_m')),
      align=st.median([abs(r['gdot_los_dps']) for r in tail]),
      dem=st.median([abs(r['gdot_nz_dps']) for r in tail]),
      act=st.median([abs(r['gdot_meas_dps']) for r in tail]),
      occ=st.median([r['occ'] for r in tail]),
      deliv=sum(abs(r['gdot_meas_dps'])*dt for r in v)/max(1e-9,sum(abs(r['gdot_nz_dps'])*dt for r in v))))
h=[r for r in rows if r['err']<=10]; m=[r for r in rows if r['err']>10]
print("  last 3 s        |gam-gam_LOS|  PN dem   actual   occupancy  full-dive delivery  err")
for lbl,g in (("HIT  n=%d"%len(h),h),("MISS n=%d"%len(m),m)):
    print("   %-9s   %8.2f deg %7.2f  %7.2f  %8.3f   %10.3f   %8.2f m"%(lbl,
      st.median([r['align'] for r in g]),st.median([r['dem'] for r in g]),
      st.median([r['act'] for r in g]),st.median([r['occ'] for r in g]),
      st.median([r['deliv'] for r in g]),st.median([r['err'] for r in g])))
print()
print("  per-sortie delivery int|act|/int|nz-clamped PN dem| : med %.3f  (pooled over all ticks %.3f)"%(
  st.median([r['deliv'] for r in rows]),
  sum(abs(r['gdot_meas_dps']) for r in T)/sum(abs(r['gdot_nz_dps']) for r in T)))
