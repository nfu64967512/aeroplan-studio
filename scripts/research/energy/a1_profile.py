#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A1: specific-energy profile + glide ratio, 32 sorties."""
from __future__ import annotations
import math, statistics as st
from energy_core import load_all, G

def pct(v, q):
    v = sorted(v); i = q * (len(v) - 1); lo = int(i)
    return v[lo] if lo >= len(v)-1 else v[lo] + (i-lo)*(v[lo+1]-v[lo])

rows = []
for tag, run, sysid, seg, tlat, tlon in load_all():
    r0, rN = seg[0], seg[-2] if len(seg) > 1 else seg[0]
    # residual altitude = agl at closest horizontal approach
    rmin, zres = r0["d"], r0["h"]
    for s in seg[1:]:
        if s["d"] < rmin:
            rmin, zres = s["d"], s["h"]
    # touchdown miss = horizontal range at first agl<=0
    r_td = seg[-1]["d"]
    # path length (3D)
    path = 0.0
    for a, b in zip(seg, seg[1:]):
        dh = math.hypot((b["lat"]-a["lat"])*111320.0,
                        (b["lon"]-a["lon"])*111320.0*math.cos(math.radians(a["lat"])))
        path += math.hypot(dh, b["h"]-a["h"])
    R0 = r0["R"]
    T = seg[-1]["t"] - seg[0]["t"]
    frz = sum(max(0.0, b["t"]-a["t"]-0.0) for a, b in zip(seg, seg[1:])
              if b["t"]-a["t"] > 0.3)
    # energy
    E0, Etd = r0["E"], rN["E"]
    # dE/dr : energy consumed per metre of ground closed
    dE_dr = (Etd - E0) / max(1e-6, (r0["d"] - rN["d"]))
    # max/min gamma over the segment
    gams = [math.degrees(s["gam"]) for s in seg[:-1]]
    Vs = [s["V"] for s in seg[:-1]]
    # sustained steepest: 5th percentile of gamma (deepest 5%)
    rows.append(dict(tag=tag, run=run, sysid=sysid,
        t_rel=r0["t"], t_td=seg[-1]["t"], T=T, frz=frz, Tsim=T-frz,
        d0=r0["d"], h0=r0["h"], R0=R0, V0=r0["V"], gam0=math.degrees(r0["gam"]),
        E0=E0, Etd=Etd, dE=Etd-E0, dEdt=(Etd-E0)/max(T,1e-6), dE_dr=dE_dr,
        Vtd=rN["V"], gstd=rN["gs"], gamtd=math.degrees(rN["gam"]),
        zres=zres, rmin=rmin, r_td=r_td, path=path, ratio=path/R0,
        gam_min=min(gams), gam_p05=pct(gams,0.05), gam_med=st.median(gams),
        Vmax=max(Vs), Vmin=min(Vs), Vmed=st.median(Vs),
        n=len(seg)))

hdr = "tag        d0    h0    R0   V0   gam0     E0    Etd     dE  dE/dt  dE/dr  Vtd  gam_p05 gam_min  zres    r_td  path  ratio    T"
print(hdr)
for r in rows:
    print(f"{r['tag']:<10s}{r['d0']:5.0f} {r['h0']:5.0f} {r['R0']:5.0f} {r['V0']:5.1f} "
          f"{r['gam0']:6.1f} {r['E0']:6.0f} {r['Etd']:6.0f} {r['dE']:6.0f} {r['dEdt']:6.1f} "
          f"{r['dE_dr']:6.3f} {r['Vtd']:5.1f} {r['gam_p05']:7.1f} {r['gam_min']:7.1f} "
          f"{r['zres']:6.1f} {r['r_td']:7.1f} {r['path']:5.0f} {r['ratio']:5.3f} {r['T']:5.1f}")

print()
for k in ("E0","Etd","dE","dEdt","dE_dr","V0","Vtd","Vmax","Vmed","gam_p05","gam_min",
          "zres","r_td","path","ratio","T","Tsim","h0"):
    v=[r[k] for r in rows]
    print(f"  {k:<8s} med {st.median(v):9.2f}  p10 {pct(v,0.10):9.2f}  p90 {pct(v,0.90):9.2f}  "
          f"min {min(v):9.2f}  max {max(v):9.2f}  sd {st.pstdev(v):7.2f}")

# --- four failure aircraft by residual altitude ---
print("\n-- sorted by residual altitude (zres) --")
for r in sorted(rows, key=lambda r: -r["zres"])[:8]:
    print(f"  {r['tag']:<10s} zres {r['zres']:7.2f}  r_td {r['r_td']:8.1f}  "
          f"ratio {r['ratio']:.3f}  gam_p05 {r['gam_p05']:6.1f}  Vtd {r['Vtd']:6.1f}  dE/dr {r['dE_dr']:6.3f}")

import json, os
json.dump(rows, open(os.path.join(os.path.dirname(__file__), "a1_rows.json"), "w"))
