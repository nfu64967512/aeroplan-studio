#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A3: per-tick replay of the LONGITUDINAL channel of tot_node._pn_cmd (2617-2645)
     + reconstruction of the throttle law (2688-2703).

Replay = recompute the command from the MEASURED state, tick by tick.
No dynamics integration (the fidelity gate in ditcg/README.md rejected that).
"""
from __future__ import annotations
import math, os, sys, json, statistics as st
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from energy_core import load_all, course, G          # noqa: E402

N_VERT, PN_LEAD = 5.0, 1.0
NZ_MIN, NZ_MAX = -1.0, 3.0
PN_VMAX, THR_TRIM, THR_GAIN = 210.0, 0.36, 0.030


def gam_cmd(h, d, R, V, gam, hdot, ddot, floor_on, nz_min=NZ_MIN):
    sig_dot = (hdot * d - h * ddot) / (R * R)
    gdot = -N_VERT * sig_dot
    nz = math.cos(gam) + V * gdot / G
    nz_c = max(nz_min, min(NZ_MAX, nz))
    gdot_c = (nz_c - math.cos(gam)) * G / V
    gc = gam + gdot_c * PN_LEAD
    gl = -math.atan2(h, max(d, 1e-3))
    if floor_on:
        gc = min(gc, gl)
    return gc, gl, nz, (nz != nz_c)


out = []
per_tick_dump = []
for tag, run, sysid, seg, tlat, tlon in load_all():
    n = len(seg)
    ticks = []
    for k in range(n - 1):
        s = seg[k]
        chi = course(seg, k)
        vn, ve = s["gs"] * math.cos(chi), s["gs"] * math.sin(chi)
        ddot = -(s["dn"] * vn + s["de"] * ve) / s["d"]
        tgo = s["R"] / max(s["V"], 1e-3)
        frozen = (tgo <= 1.5) or (s["R"] <= 100.0)
        gc_on, gl, nz, nzclip = gam_cmd(s["h"], s["d"], s["R"], s["V"], s["gam"],
                                        s["hdot"], ddot, True)
        gc_off, _, _, _ = gam_cmd(s["h"], s["d"], s["R"], s["V"], s["gam"],
                                  s["hdot"], ddot, False)
        bind = gc_off > gl + 1e-12          # PN wanted shallower than LOS
        # throttle reconstruction
        t_left = s["early"] + s["secs"]
        late = t_left <= 1.0
        t_rem = t_left if not late else max(1.0, s["R"] / max(s["V"], 1e-3))
        v_req = max(45.0, min(PN_VMAX, s["R"] / t_rem))
        if late:
            v_req = PN_VMAX
        thr = max(0.0, min(1.0, THR_TRIM + THR_GAIN * (v_req - s["V"])))
        thr_raw = THR_TRIM + THR_GAIN * (v_req - s["V"])
        dt = seg[k + 1]["t"] - s["t"]
        dEdt = ((seg[k + 1]["E"] - s["E"]) / dt) if 0.01 < dt <= 0.3 else float("nan")
        ticks.append(dict(k=k, t=s["t"], dt=dt, h=s["h"], d=s["d"], R=s["R"], V=s["V"],
                          gam=math.degrees(s["gam"]), gc_on=math.degrees(gc_on),
                          gc_off=math.degrees(gc_off), gl=math.degrees(gl),
                          bind=bind, nz=nz, nzclip=nzclip, frozen=frozen,
                          thr=thr, thr_raw=thr_raw, v_req=v_req, late=late,
                          dEdt=dEdt, E=s["E"]))
    live = [x for x in ticks if not x["frozen"]]
    if not live:
        continue
    dg = [x["gc_off"] - x["gc_on"] for x in live]
    dgb = [x for x in dg if x > 1e-9]
    thrv = [x["thr"] for x in live]
    out.append(dict(tag=tag,
        n_live=len(live),
        bind_frac=sum(1 for x in live if x["bind"]) / len(live),
        nzclip_frac=sum(1 for x in live if x["nzclip"]) / len(live),
        dgam_med=st.median(dg), dgam_max=max(dg),
        dgam_bind_med=(st.median(dgb) if dgb else 0.0),
        thr_med=st.median(thrv),
        thr_sat_hi=sum(1 for x in live if x["thr"] >= 0.999) / len(live),
        thr_sat_lo=sum(1 for x in live if x["thr"] <= 0.001) / len(live),
        late_frac=sum(1 for x in live if x["late"]) / len(live),
        vreq_med=st.median([x["v_req"] for x in live]),
    ))
    per_tick_dump.append((tag, ticks))

print("tag         n_live  bindF  nzclipF  dgam_med dgam_bindmed dgam_max   thr_med thrSatHi thrSatLo lateF vreq_med")
for r in out:
    print(f"{r['tag']:<11s}{r['n_live']:6d}  {r['bind_frac']:5.3f}  {r['nzclip_frac']:6.3f}  "
          f"{r['dgam_med']:8.3f} {r['dgam_bind_med']:11.3f} {r['dgam_max']:8.2f}   "
          f"{r['thr_med']:6.3f} {r['thr_sat_hi']:7.3f} {r['thr_sat_lo']:8.3f} "
          f"{r['late_frac']:5.3f} {r['vreq_med']:7.1f}")
print()
for k in ("bind_frac","nzclip_frac","dgam_med","dgam_bind_med","dgam_max",
          "thr_med","thr_sat_hi","thr_sat_lo","late_frac","vreq_med"):
    v=[r[k] for r in out]
    print(f"  {k:<14s} med {st.median(v):8.3f}  min {min(v):8.3f}  max {max(v):8.3f}")

json.dump(out, open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "a3_rows.json"), "w"))
import pickle
pickle.dump(per_tick_dump, open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                             "a3_ticks.pkl"), "wb"))
