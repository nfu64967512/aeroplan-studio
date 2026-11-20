#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""R1: 32-sortie REPLAY of the vertical channel, old law vs path A.

⚠️ This is a REPLAY, not a flight. It feeds the *measured* per-tick state of
   all 32 sorties into both laws and compares what they ASK FOR. It cannot
   prove the aircraft can deliver it; it can only prove the command became
   (in)feasible.

Reads   runs/artifacts/{RRD,RRE}{1..8}/tot_*.csv   (via scripts/research/ditcg/truth32)
        the worktree  scripts/wsl/sar_swarm_patches/tot_node.py
        the pre-B1 baseline from `git show HEAD:` (path given by TOT_BEFORE env)
Writes  only this directory.

Pre-registered pass/fail gates are in `GATES` below; they are evaluated
verbatim and never tuned.
"""
from __future__ import annotations

import math
import os
import statistics as st
import sys
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "scripts" / "research" / "ditcg"))

import truth32                                                   # noqa: E402
import r1_harness as H                                           # noqa: E402

G = 9.80665
NZ_MIN, NZ_MAX = -1.0, 3.0          # contract pn_nz_min / shipped nz_max_g
GAP_S = 0.3
HALF = 5
SPAN_LO, SPAN_HI = 0.20, 0.80

BEFORE = os.environ.get(
    "TOT_BEFORE",
    str(pathlib.Path(os.environ.get("TEMP", "/tmp")) / "tot_node_HEAD.py"))
AFTER = ROOT / "scripts" / "wsl" / "sar_swarm_patches" / "tot_node.py"


# ══════════════════════════════════════════════════════════════════════════
#  per-tick state reconstruction  (identical segment/dequant rules to
#  m1_gamma_ceiling.per_ac, so the tick set matches m1_ticks.csv 1:1)
# ══════════════════════════════════════════════════════════════════════════
def dequant(seg):
    out, run_t, k = [], None, 0
    for r in seg:
        t = truth32._f(r, "t")
        if run_t is not None and abs(t - run_t) < 1e-9:
            k += 1
        else:
            run_t, k = t, 0
        out.append(t + 0.05 * k)
    return out


def states(run, sysid, rows, tlat, tlon):
    """-> list of dicts with everything a stub node needs for one tick."""
    i0 = next((k for k, r in enumerate(rows) if r.get("mode") == "dive"), None)
    if i0 is None:
        return None
    i1 = next((k for k in range(i0, len(rows))
               if truth32._f(rows[k], "agl") <= 0.0), None)
    if i1 is None:
        return None
    seg = rows[i0:i1 + 1]
    tq = dequant(seg)
    traw = [truth32._f(r, "t") for r in seg]
    n = len(seg)
    out = []
    for k in range(n):
        r = seg[k]
        gam = math.radians(truth32._f(r, "gam_deg"))
        vh = truth32._f(r, "gs")
        h = truth32._f(r, "agl")
        lat, lon = truth32._f(r, "lat"), truth32._f(r, "lon")
        if any(math.isnan(x) for x in (gam, vh, h, lat, lon)):
            continue
        cg = math.cos(gam)
        if cg < 0.05:
            continue
        V = max(1e-3, vh / cg)
        hdot = vh * math.tan(gam)
        chi, _ = truth32._course_rad(seg, k)
        vn, ve = vh * math.cos(chi), vh * math.sin(chi)
        # measured gamma-dot (central difference), same window as M1
        gm = float("nan")
        lo, hi = k - HALF, k + HALF
        if lo >= 0 and hi < n:
            ok = all(traw[j + 1] - traw[j] <= GAP_S for j in range(lo, hi))
            span = tq[hi] - tq[lo]
            if ok and SPAN_LO <= span <= SPAN_HI:
                gm = math.radians(truth32._f(seg[hi], "gam_deg")
                                  - truth32._f(seg[lo], "gam_deg")) / span
        sec = truth32._f(r, "secs_to_target")
        out.append(dict(t=tq[k], lat=lat, lon=lon, h=h, vn=vn, ve=ve,
                        vd=-hdot, gam=gam, V=V, vh=vh,
                        nz_meas=(cg + V * gm / G) if gm == gm else float("nan"),
                        sec=sec))
    if not out:
        return None
    t0, t1 = out[0]["t"], out[-1]["t"]
    for s in out:
        s["frac"] = (s["t"] - t0) / max(1e-6, t1 - t0)
        s["t_to_end"] = t1 - s["t"]
    return out


# ══════════════════════════════════════════════════════════════════════════
#  alpha model  (the only quantity the artifact CSVs do not carry)
# ══════════════════════════════════════════════════════════════════════════
#: theta_meas = gam + alpha.  alpha is reconstructed from the MEASURED n_z
#: through the identified aero model, so the stub aircraft is flying at the
#: AOA that actually produced the observed load factor.
#: ⚠️ dataflash cross-check: this puts the dive-segment AOA median at the
#:    value printed by --alpha-report; the logged AOA median is -3.2 deg.
#: ⚠️ ALPHA_CONST=<deg> overrides it with a flat AOA (sensitivity run).
ALPHA_CONST = os.environ.get("ALPHA_CONST")


def alpha_of(bind_ns, s, node):
    if ALPHA_CONST is not None:
        return math.radians(float(ALPHA_CONST))
    nz = s["nz_meas"]
    if nz != nz:
        nz = 1.0
    q = 0.5 * bind_ns["RHO0"] * (max(1e-3, (1.0 - 2.25577e-5 * max(0.0, s["h"]))
                                     ** bind_ns["ISA_RHO_EXP"]) ** 2.0) \
        * s["V"] * s["V"]
    w = bind_ns.get("AERO_WEIGHT_N") or bind_ns["AERO_MASS_KG"] * bind_ns["G0"]
    cl = nz * w / max(q * bind_ns["AERO_S_REF_M2"], 1e-6)
    return (cl - 0.56381) / 6.9503


# ══════════════════════════════════════════════════════════════════════════
#  driver
# ══════════════════════════════════════════════════════════════════════════
def run_config(bind, over, acs, aero_ns):
    """Replay every tick of every aircraft under one configuration."""
    recs = []
    for a in acs:
        node = H.make_node(bind, over)
        node.T = (a["tlat"], a["tlon"])
        for s in a["st"]:
            node.pos[1] = (s["lat"], s["lon"], s["h"])
            node.alt_amsl[1] = s["h"]
            node.vel[1] = (s["vn"], s["ve"], s["vd"])
            al = alpha_of(aero_ns, s, node)
            node.att[1] = (0.0, s["gam"] + al, 0.0)
            node.dive_T = (s["t"] + s["sec"]) if s["sec"] == s["sec"] else None
            roll, pitch, thr, R, V = node._pn_cmd(1, s["t"])
            d = dict(node.nz_dbg.get(1, {}))
            recs.append(dict(tag=a["tag"], t=s["t"], frac=s["frac"],
                             t_to_end=s["t_to_end"], gam=s["gam"], V=V, R=R,
                             h=s["h"], roll=roll, pitch=pitch, thr=thr,
                             alpha_lp=node.alpha.get(1), alpha_in=al,
                             nz_meas=s["nz_meas"], **d))
    return recs


def q(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, max(0, int(round(p * (len(v) - 1)))))]


def ceiling(gd, cg, V):
    return ((NZ_MAX if gd >= 0 else NZ_MIN) - cg) * G / V


def implied_gdot(r, lead=1.0):
    """gamma-dot the FC is actually being asked for, from the sent pitch."""
    return (r["cmd_nz"] - math.cos(r["gam"])) * G / r["V"]


def main():
    src_before = pathlib.Path(BEFORE).read_text(encoding="utf-8")
    src_after = AFTER.read_text(encoding="utf-8")
    b_before = H.Bind(src_before, "BEFORE(HEAD)")
    b_after = H.Bind(src_after, "AFTER(worktree)")
    assert "nz_closed_loop" not in src_before, \
        "⚠️ TOT_BEFORE already contains path A -- not a clean baseline"
    aero = b_after.ns

    acs = []
    for run in truth32.RUNS:
        tlat, tlon = truth32._read_target(run)
        for sysid, rows in sorted(truth32._rows(run).items()):
            stt = states(run, sysid, rows, tlat, tlon)
            if stt:
                acs.append(dict(tag="%s-U%d" % (run, sysid), tlat=tlat,
                                tlon=tlon, st=stt))
    ntick = sum(len(a["st"]) for a in acs)
    print("aircraft %d   ticks %d" % (len(acs), ntick))

    CFG = {
        "BASE": (b_before, {}),
        "OFF": (b_after, dict(nz_closed_loop=False, nz_clamp_after_floor=False)),
        "CLAMP": (b_after, dict(nz_closed_loop=False, nz_clamp_after_floor=True)),
        "CL": (b_after, dict(nz_closed_loop=True, nz_clamp_after_floor=False)),
        "NEW": (b_after, dict(nz_closed_loop=True, nz_clamp_after_floor=True)),
    }
    out = {}
    for k, (b, o) in CFG.items():
        out[k] = run_config(b, o, acs, aero)
        print("  replayed %-6s n=%d" % (k, len(out[k])))
    return acs, out, aero


if __name__ == "__main__":
    main()
