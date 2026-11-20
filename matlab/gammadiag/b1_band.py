# -*- coding: utf-8 -*-
"""B1: the L1/TECS band  (release ring -> PN handover)  for NZC1-8 + RRE1-8.

Own loader: truth32.RUNS is RRD+RRE, and R.states() drops any airframe that
never logs agl<=0 (that silently deletes NZC1-U2 and NZC4-U2, 2 of the 6
failures).  Nothing here requires touchdown.
"""
from __future__ import annotations
import csv, math, os, pathlib, sys

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[1]
ART = ROOT / "runs" / "artifacts"
EARTH = 111320.0
GAP_S = 0.3
FD_HALF = 4
GATE_R = 4700.0
G = 9.80665

RUNS = [f"NZC{i}" for i in range(1, 9)] + [f"RRE{i}" for i in range(1, 9)]
FAIL = {"NZC1-U2", "NZC3-U1", "NZC4-U2", "NZC5-U1", "NZC6-U2", "RRE7-U2"}


def mlon(lat): return EARTH * math.cos(math.radians(lat))


def _f(r, k, d=float("nan")):
    try: return float(r.get(k, ""))
    except (TypeError, ValueError): return d


def target(run):
    p = ART / run / f"tot_{run}_geom.csv"
    for row in csv.DictReader(open(p, newline="", encoding="utf-8", errors="replace")):
        if row["kind"] == "target":
            return float(row["lat"]), float(row["lon"])
    raise RuntimeError(run)


def rows_by_sys(run):
    p = ART / run / f"tot_{run}.csv"
    out = {}
    for row in csv.DictReader(open(p, newline="", encoding="utf-8", errors="replace")):
        out.setdefault(int(row["sysid"]), []).append(row)
    return out


def course(rows, k):
    n = len(rows)
    lo, hi = max(0, k - FD_HALF), min(n - 1, k + FD_HALF)
    while lo < k and _f(rows[lo + 1], "t") - _f(rows[lo], "t") > GAP_S: lo += 1
    while hi > k and _f(rows[hi], "t") - _f(rows[hi - 1], "t") > GAP_S: hi -= 1
    if hi <= lo: lo, hi = max(0, k - 1), min(n - 1, k + 1)
    la0, lo0 = _f(rows[lo], "lat"), _f(rows[lo], "lon")
    la1, lo1 = _f(rows[hi], "lat"), _f(rows[hi], "lon")
    dn = (la1 - la0) * EARTH
    de = (lo1 - lo0) * mlon(0.5 * (la0 + la1))
    return math.atan2(de, dn)


def dequant(seg):
    out, run_t, k = [], None, 0
    for r in seg:
        t = _f(r, "t")
        if run_t is not None and abs(t - run_t) < 1e-9: k += 1
        else: run_t, k = t, 0
        out.append(t + 0.05 * k)
    return out


def airframes():
    """-> list of dicts, one per airframe, with the full post-release row slice."""
    acs = []
    for run in RUNS:
        tlat, tlon = target(run)
        for sysid, rows in sorted(rows_by_sys(run).items()):
            i_rel = next((k for k, r in enumerate(rows) if r.get("mode") == "dive"), None)
            if i_rel is None:
                continue
            seg = rows[i_rel:]
            tq = dequant(seg)
            d = [math.hypot((tlat - _f(r, "lat")) * EARTH,
                            (tlon - _f(r, "lon")) * mlon(_f(r, "lat"))) for r in seg]
            # first row carrying a logged cmd_nz  (= empirical PN handover, NZC only)
            i_cmd = next((k for k, r in enumerate(seg)
                          if (r.get("cmd_nz") or "").strip() != ""), None)
            # gate:  first k with d<=GATE_R  (hold_n=3 node ticks ~ 0.15 s, sub-row)
            i_gate = next((k for k, x in enumerate(d) if x <= GATE_R), None)
            acs.append(dict(tag=f"{run}-U{sysid}", run=run, sysid=sysid,
                            tlat=tlat, tlon=tlon, seg=seg, tq=tq, d=d,
                            i_cmd=i_cmd, i_gate=i_gate))
    return acs


def states(a, k0, k1):
    """rows [k0,k1) -> replay states (same reconstruction as r1_replay.states)."""
    seg, tq, n = a["seg"], a["tq"], len(a["seg"])
    traw = [_f(r, "t") for r in seg]
    out = []
    for k in range(k0, k1):
        r = seg[k]
        gam = math.radians(_f(r, "gam_deg"))
        vh = _f(r, "gs"); h = _f(r, "agl")
        lat, lon = _f(r, "lat"), _f(r, "lon")
        if any(x != x for x in (gam, vh, h, lat, lon)): continue
        cg = math.cos(gam)
        if cg < 0.05: continue
        V = max(1e-3, vh / cg)
        hdot = vh * math.tan(gam)
        chi = course(seg, k)
        vn, ve = vh * math.cos(chi), vh * math.sin(chi)
        gm = float("nan")
        lo, hi = k - 5, k + 5
        if lo >= 0 and hi < n:
            ok = all(traw[j + 1] - traw[j] <= GAP_S for j in range(lo, hi))
            span = tq[hi] - tq[lo]
            if ok and 0.20 <= span <= 0.80:
                gm = math.radians(_f(seg[hi], "gam_deg") - _f(seg[lo], "gam_deg")) / span
        out.append(dict(k=k, t=tq[k], lat=lat, lon=lon, h=h, vn=vn, ve=ve, vd=-hdot,
                        gam=gam, V=V, vh=vh, d=a["d"][k],
                        nz_meas=(cg + V * gm / G) if gm == gm else float("nan"),
                        sec=_f(r, "secs_to_target")))
    return out
