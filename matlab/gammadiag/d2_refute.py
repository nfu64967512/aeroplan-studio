# -*- coding: utf-8 -*-
"""D2: refutation pass. Reads m1_ticks.csv + chain32.csv + truth32.csv only."""
import csv, math, os, statistics as st, collections
H=os.path.dirname(os.path.abspath(__file__))
def rd(p): return list(csv.DictReader(open(p,encoding='utf-8')))
def f(r,k):
    try: return float(r[k])
    except: return float('nan')
T=rd(os.path.join(H,'m1_ticks.csv'))
for r in T:
    for k in r:
        if k!='tag' and k!='cls': r[k]=f(r,k)
by=collections.defaultdict(list)
for r in T: by[r['tag']].append(r)

def gain(rows,xk,yk='gdot_meas_dps'):
    sxx=sum(r[xk]**2 for r in rows); sxy=sum(r[xk]*r[yk] for r in rows)
    syy=sum(r[yk]**2 for r in rows)
    return sxy/sxx if sxx>0 else float('nan'), sxy/math.sqrt(sxx*syy) if sxx*syy>0 else float('nan')

print("=== [1] window-truncation sweep (M2 claims 85%% of M1 window has no PN cmd) ===")
print("M1 window = mode=='dive' -> agl<=0 ; median 40.7 s (NOT phase=='DIVE' 212 s)")
print(" keep frac>=  n      gain   corr   | dur kept (med s)")
for cut in (0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7):
    sub=[r for r in T if r['frac']>=cut]
    g,c=gain(sub,'gdot_los_dps')
    durs=[]
    for k,v in by.items():
        s=[r for r in v if r['frac']>=cut]
        if s: durs.append(max(x['t'] for x in s)-min(x['t'] for x in s))
    print("  %.1f  %6d  %.4f  %+.3f  |  %.1f"%(cut,len(sub),g,c,st.median(durs)))

print()
print("=== [2] INTEGRAL TEST: if demand were a real rate demand, int(demand)dt = 16x int(actual)dt ===")
rows=[]
for k,v in sorted(by.items()):
    v=sorted(v,key=lambda r:r['t'])
    dt=0.05
    I_los=sum(abs(r['gdot_los_dps'])*dt for r in v)
    I_raw=sum(abs(r['gdot_raw_dps'])*dt for r in v if abs(r['gdot_raw_dps'])<1e4)
    I_nz =sum(abs(r['gdot_nz_dps'])*dt for r in v)
    I_act=sum(abs(r['gdot_meas_dps'])*dt for r in v)
    dgam=v[-1]['gam_deg']-v[0]['gam_deg']
    rows.append((k,I_los,I_nz,I_raw,I_act,dgam,v[0]['gam_deg'],v[-1]['gam_deg']))
for lbl,ix in (("LOS-floor demand",1),("nz-clamped PN demand",2),("raw PN demand",3),("ACTUAL |dgam|",4)):
    a=[r[ix] for r in rows]
    print("  int|.|dt  %-22s med %8.1f deg   (p10 %7.1f  p90 %8.1f)"%(lbl,st.median(a),sorted(a)[3],sorted(a)[-4]))
net=[abs(r[5]) for r in rows]
print("  net |gam_end - gam_start|            med %8.1f deg"%st.median(net))
print("  ratio int(LOS demand)/int(actual)    med %8.2f"%st.median([r[1]/r[4] for r in rows]))
print("  ratio int(rawPN  )/int(actual)       med %8.2f"%st.median([r[3]/r[4] for r in rows]))

print()
print("=== [3] where does the raw-PN demand blow up? (sig_dot ~ 1/R^2) ===")
allr=[r for r in T if abs(r['gdot_raw_dps'])<1e6]
for lo,hi in ((0,200),(200,500),(500,1000),(1000,2000),(2000,5000),(5000,1e9)):
    s=[r for r in allr if lo<=math.hypot(r['h_m'],r['d_m'])<hi]
    if not s: continue
    a=sorted(abs(r['gdot_raw_dps']) for r in s)
    n=len(a)
    print("  R %5d-%-7d n=%5d (%4.1f%%)  |raw| med %7.2f p99 %9.2f max %10.1f"%(
        lo,hi,n,100*n/len(allr),a[n//2],a[int(.99*n)],a[-1]))

print()
print("=== [4] n_z actually achieved vs the [-1,3] guidance clamp ===")
nz=sorted(r['nz_meas'] for r in T if not math.isnan(r['nz_meas']))
n=len(nz)
print("  nz_meas p01 %.2f p50 %.2f p95 %.2f p99 %.2f max %.2f"%(nz[int(.01*n)],nz[n//2],nz[int(.95*n)],nz[int(.99*n)],nz[-1]))
print("  frac nz>2.7 %.4f   frac nz>3.0 %.4f   frac nz<-1 %.4f"%(
    sum(1 for x in nz if x>2.7)/n, sum(1 for x in nz if x>3.0)/n, sum(1 for x in nz if x<-1)/n))
