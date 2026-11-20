import numpy as np, sys
sys.path.insert(0, "/mnt/c/Users/joy46/Documents/Projects/aeroplan-studio/matlab/gammadiag")
from align import load_csv, load_log, EARTH, UAV1, UAV2

def best_tau(ts, las, los, z):
    lt = z["POS.TimeUS"]/1e6; lla=z["POS.Lat"]; llo=z["POS.Lng"]
    ml = np.cos(np.radians(las.mean()))
    def rms(tau):
        a=np.interp(ts+tau, lt, lla, left=np.nan, right=np.nan)
        b=np.interp(ts+tau, lt, llo, left=np.nan, right=np.nan)
        d=np.hypot((a-las)*EARTH,(b-los)*EARTH*ml)
        return np.nanmean(d**2)**.5 if np.isfinite(d).sum()>50 else 1e9
    c=min(np.arange(-300,301,1.0), key=rms)
    c=min(np.arange(c-1.5,c+1.5,0.02), key=rms)
    return c, rms(c)

trial=sys.argv[1]; sysid=int(sys.argv[2])
t, rows = load_csv(trial, sysid)
dive=np.array([r["phase"]=="DIVE" for r in rows])
lat=np.array([float(r["lat"]) for r in rows]); lon=np.array([float(r["lon"]) for r in rows])
cands = range(405,421) if sysid==1 else range(393,409)
out=[]
for n in cands:
    try: z=load_log(sysid,n)
    except Exception: continue
    tau,r = best_tau(t[dive],lat[dive],lon[dive],z)
    out.append((r,n,tau))
out.sort()
for r,n,tau in out[:5]: print(f"  log {n}  tau {tau:8.2f}  rms {r:10.2f}")
