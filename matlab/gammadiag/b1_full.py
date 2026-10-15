# -*- coding: utf-8 -*-
"""B1: is the LONGITUDINAL channel of `_pn_cmd` flyable in the L1/TECS band?

OPEN-LOOP REPLAY.  Measured per-tick state in, command out.  It shows what the
law WOULD HAVE COMMANDED at states the aircraft actually occupied.  It cannot
show what the aircraft would then have done -- the moment the command changes
the trajectory, the replayed states stop being the states that would occur.
"""
from __future__ import annotations
import math, sys, pathlib, statistics as st
HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "scripts" / "research" / "ditcg"))
sys.path.insert(0, str(ROOT / "scripts" / "wsl" / "sar_swarm_patches"))
import b1_band as B, r1_harness as H, r1_replay as R                # noqa: E402
import predictor as P                                               # noqa: E402

G = 9.80665
D = math.degrees
AFTER = ROOT / "scripts" / "wsl" / "sar_swarm_patches" / "tot_node.py"
BIND = H.Bind(AFTER.read_text(encoding="utf-8"), "worktree")
HOLD = 3
ARMS = (("clampOFF", False), ("clampON", True))


def replay(a, k0, k1, clamp):
    node = H.make_node(BIND, dict(nz_clamp_after_floor=clamp))
    node.T = (a["tlat"], a["tlon"])
    out = []
    for s in B.states(a, k0, k1):
        r = a["seg"][s["k"]]
        eb = B._f(r, "early_by_s")
        node.pos[1] = (s["lat"], s["lon"], s["h"])
        node.alt_amsl[1] = s["h"]
        node.vel[1] = (s["vn"], s["ve"], s["vd"])
        al = R.alpha_of(BIND.ns, s, node)
        node.att[1] = (0.0, s["gam"] + al, 0.0)
        # ⚠️ true dive_T-now = secs_to_target + early_by_s  (_row: early_by_s =
        #    (dive_T-now) - secs_to_target).  r1_replay.py uses `sec` alone,
        #    which understates t_left by up to +31 s and therefore OVERSTATES
        #    the throttle.  Only the throttle channel depends on it (itcg_on=0).
        tl = s["sec"] + eb if (s["sec"] == s["sec"] and eb == eb) else float("nan")
        node.dive_T = (s["t"] + tl) if tl == tl else None
        roll, pitch, thr, Rr, V = node._pn_cmd(1, s["t"])
        dbg = dict(node.nz_dbg.get(1, {}))
        gam = s["gam"]
        gam_cmd = gam + (dbg["cmd_nz"] - math.cos(gam)) * G * node.pn_lead / V
        nz_cl = max(node.pn_nz_min, min(node.nz_max_g, dbg["req_nz"]))
        pre = gam + ((nz_cl - math.cos(gam)) * G / V) * node.pn_lead
        gam_los = -math.atan2(s["h"], max(s["d"], 1e-3))
        t_left = 0.0 if node.dive_T is None else tl
        late = t_left <= 1.0
        t_rem = t_left if not late else max(1.0, Rr / max(V, 1e-3))
        v_req = node.pn_vmax if late else max(45.0, min(node.pn_vmax, Rr / t_rem))
        out.append(dict(k=s["k"], t=s["t"], d=s["d"], h=s["h"], gam=gam, V=V,
                        vh=s["vh"], R=Rr, roll=roll, pitch=pitch, thr=thr,
                        v_req=v_req, t_left=t_left, late=late, alpha=al,
                        nz_meas=s["nz_meas"], req_nz=dbg["req_nz"],
                        cmd_nz=dbg["cmd_nz"], nz_max_true=dbg["nz_max_true"],
                        gam_cmd=gam_cmd, gam_pre=pre, gam_los=gam_los,
                        floor_binds=(gam_los < pre - 1e-12)))
    return out


def load():
    acs = B.airframes()
    for a in acs:
        k1 = a["i_gate"] + HOLD - 1
        if a["i_cmd"] is not None:
            assert k1 == a["i_cmd"], (a["tag"], k1, a["i_cmd"])
        a["k1"] = k1
        a["clamp_flown"] = a["run"].startswith("NZC")
        # post-handover PN segment: k1 .. first agl<=0 (else end of file, flagged)
        td = next((k for k in range(k1, len(a["seg"]))
                   if B._f(a["seg"][k], "agl") <= 0.0), None)
        a["touched"] = td is not None
        a["k2"] = (td + 1) if td is not None else len(a["seg"])
        for nm, cl in ARMS:
            a["band_" + nm] = replay(a, 0, k1, cl)
        a["pn_flown"] = replay(a, k1, a["k2"], a["clamp_flown"])
    return acs


def q(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, max(0, int(round(p * (len(v) - 1)))))]
