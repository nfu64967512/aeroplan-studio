# -*- coding: utf-8 -*-
import csv,math,os,statistics as st,collections
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
for r in T: by[r['tag']].append(r)
TR={r['tag']:r for r in rd(os.path.join(H,'..','data','truth32.csv'))}

def spear(x,y):
    def rk(a):
        o=sorted(range(len(a)),key=lambda i:a[i]); r=[0]*len(a)
        for p,i in enumerate(o): r[i]=p
        return r
    rx,ry=rk(x),rk(y); n=len(x)
    mx,my=st.mean(rx),st.mean(ry)
    num=sum((rx[i]-mx)*(ry[i]-my) for i in range(n))
    den=math.sqrt(sum((v-mx)**2 for v in rx)*sum((v-my)**2 for v in ry))
    return num/den if den else float('nan')

per={}
for k,v in by.items():
    v=sorted(v,key=lambda r:r['t']); dt=0.05
    ok=[r for r in v if abs(r['gdot_raw_dps'])<1e4]
    I_act=sum(abs(r['gdot_meas_dps'])*dt for r in v)
    per[k]=dict(
      over_los=sum(1 for r in v if abs(r['gdot_los_dps'])>max(r['cap_up_dps'],abs(r['cap_dn_dps'])))/len(v),
      over_raw=sum(1 for r in ok if abs(r['gdot_raw_dps'])>max(r['cap_up_dps'],abs(r['cap_dn_dps'])))/len(ok),
      occ_med=st.median([r['occ'] for r in v]),
      sat=sum(1 for r in v if r['saturated'])/len(v),
      deliv_raw=sum(abs(r['gdot_raw_dps'])*dt for r in ok)and I_act/sum(abs(r['gdot_raw_dps'])*dt for r in ok),
      deliv_nz=I_act/max(1e-9,sum(abs(r['gdot_nz_dps'])*dt for r in v)),
      deliv_los=I_act/max(1e-9,sum(abs(r['gdot_los_dps'])*dt for r in v)),
      nzmax=max(r['nz_meas'] for r in v), nzmin=min(r['nz_meas'] for r in v),
    )
tags=[k for k in sorted(per) if k in TR]
err=[ff(TR[k],'err_touch_m') for k in tags]
zres=[abs(ff(TR[k],'z_res_m')) for k in tags]
nzc=[ff(TR[k],'nz_clip_frac') for k in tags]
nzlo=[ff(TR[k],'nz_lo_frac') for k in tags]
nzhi=[ff(TR[k],'nz_hi_frac') for k in tags]
hit=[int(ff(TR[k],'hit_10m')) for k in tags]
print("n=%d  hits(10m)=%d  err med %.2f m  p90 %.2f  max %.2f"%(len(tags),sum(hit),
    st.median(err),sorted(err)[-4],max(err)))
print()
print("=== [5] does 'demand exceeds physics' predict the miss?  (spearman vs err_touch_m) ===")
for lbl,key in (("frac ticks: LOS demand > ceiling","over_los"),
                ("frac ticks: raw PN demand > ceiling","over_raw"),
                ("frac ticks: actual gdot at ceiling","sat"),
                ("median occupancy |gdot|/ceiling","occ_med"),
                ("DELIVERY int|act|/int|rawPN dem|","deliv_raw"),
                ("DELIVERY int|act|/int|nzclamped|","deliv_nz"),
                ("DELIVERY int|act|/int|LOSfloor |","deliv_los")):
    x=[per[k][key] for k in tags]
    print("  %-34s med %7.3f   rho(err) %+.3f   rho(|z_res|) %+.3f"%(lbl,st.median(x),spear(x,err),spear(x,zres)))
print("  %-34s med %7.3f   rho(err) %+.3f   rho(|z_res|) %+.3f"%("truth32 nz_clip_frac",st.median(nzc),spear(nzc,err),spear(nzc,zres)))
print("  %-34s med %7.3f   rho(err) %+.3f   rho(|z_res|) %+.3f"%("truth32 nz_lo_frac (push-over)",st.median(nzlo),spear(nzlo,err),spear(nzlo,zres)))
print("  %-34s med %7.3f   rho(err) %+.3f   rho(|z_res|) %+.3f"%("truth32 nz_hi_frac (pull-up)",st.median(nzhi),spear(nzhi,err),spear(nzhi,zres)))
print("  %-34s med %7.3f   rho(err) %+.3f"%("|z_res_m| residual altitude",st.median(zres),spear(zres,err)))
print()
print("=== [6] delivery ratio per sortie (int|actual| / int|demand|), all 32 ===")
d=[per[k]['deliv_nz'] for k in tags]
print("  vs nz-clamped PN demand: med %.3f  min %.3f  max %.3f  n>0.5: %d/32"%(
    st.median(d),min(d),max(d),sum(1 for x in d if x>0.5)))
d2=[per[k]['deliv_los'] for k in tags]
print("  vs LOS-floor demand    : med %.3f  min %.3f  max %.3f"%(st.median(d2),min(d2),max(d2)))
print()
print("=== [7] nz clamp bites which side? (truth32, node-side saturation flags) ===")
print("  nz_lo_frac (hit -1.0 push-over floor) med %.4f  p90 %.4f  max %.4f"%(st.median(nzlo),sorted(nzlo)[-4],max(nzlo)))
print("  nz_hi_frac (hit +3.0 pull-up  ceil ) med %.4f  p90 %.4f  max %.4f"%(st.median(nzhi),sorted(nzhi)[-4],max(nzhi)))
print("  sorties with nz_hi_frac > nz_lo_frac: %d/32"%sum(1 for i in range(len(tags)) if nzhi[i]>nzlo[i]))
print()
print("=== [8] measured nz exceeds the guidance clamp itself ===")
print("  sorties whose measured nz went below -1.0 : %d/32"%sum(1 for k in tags if per[k]['nzmin']<-1.0))
print("  min measured nz over all sorties: %.2f"%min(per[k]['nzmin'] for k in tags))
print("  sorties whose measured nz went above +3.0: %d/32"%sum(1 for k in tags if per[k]['nzmax']>3.0))
