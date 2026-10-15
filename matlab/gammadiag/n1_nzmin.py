#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""N1: does `pn_nz_min` buy anything?  32-sortie per-tick REPLAY, 4 arms.

⚠️ REPLAY, not flight.  It feeds the *measured* per-tick state of all 32
   sorties into the worktree law under four configurations and compares what
   each one ASKS FOR.  It cannot prove the aircraft delivers it.  What it CAN
   settle, exactly, is whether the arms are even distinguishable -- see
   [[itcg-lateral-authority-ceiling]] trap #1: "flipped the switch, track
   unchanged" is NOT evidence the law did not fire.

Arms (single-variable pairs by construction):
  A0  clamp_after_floor=0  pn_nz_min=-1.0   <- RRE production, the baseline
  A1  clamp_after_floor=1  pn_nz_min=-1.0   <- ordering fix at contract value
  A2  clamp_after_floor=1  pn_nz_min=-2.0   <- ordering fix at contract floor
  A3  clamp_after_floor=0  pn_nz_min=-2.0   <- widen ONLY; tests inertness

  A0 vs A3 : is pn_nz_min live with the clamp OFF?
  A0 vs A1 : what does the ordering fix cost/buy at the same nz_min?
  A1 vs A2 : does widening the floor buy anything once the clamp is live?
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
import r1_replay as R                                            # noqa: E402

G = 9.80665
AFTER = ROOT / "scripts" / "wsl" / "sar_swarm_patches" / "tot_node.py"

ARMS = [
    ("A0", dict(nz_clamp_after_floor=False, pn_nz_min=-1.0)),
    ("A1", dict(nz_clamp_after_floor=True,  pn_nz_min=-1.0)),
    ("A2", dict(nz_clamp_after_floor=True,  pn_nz_min=-2.0)),
    ("A3", dict(nz_clamp_after_floor=False, pn_nz_min=-2.0)),
]


def q(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, max(0, int(round(p * (len(v) - 1)))))]


def main():
    bind = H.Bind(AFTER.read_text(encoding="utf-8"), "worktree")
    acs = []
    for run in truth32.RUNS:
        tlat, tlon = truth32._read_target(run)
        for sysid, rows in sorted(truth32._rows(run).items()):
            stt = R.states(run, sysid, rows, tlat, tlon)
            if stt:
                acs.append(dict(tag=f"{run}-U{sysid}", st=stt,
                                tlat=tlat, tlon=tlon))
    print(f"aircraft = {len(acs)}   ticks = {sum(len(a['st']) for a in acs)}")

    res = {}
    for name, over in ARMS:
        res[name] = R.run_config(bind, over, acs, bind.ns)

    n = len(res["A0"])
    print(f"\n{'arm':<4} {'cmd_nz med':>11} {'cmd_nz min':>11} "
          f"{'<-1.0 %':>9} {'<-2.0 %':>9} {'pitch p10':>10} {'pitch min':>10}")
    for name, _ in ARMS:
        r = res[name]
        cz = [x["cmd_nz"] for x in r if x.get("cmd_nz") == x.get("cmd_nz")]
        pit = [math.degrees(x["pitch"]) for x in r]
        print(f"{name:<4} {st.median(cz):11.4f} {min(cz):11.4f} "
              f"{100*sum(1 for v in cz if v < -1.0)/len(cz):9.3f} "
              f"{100*sum(1 for v in cz if v < -2.0)/len(cz):9.3f} "
              f"{q(pit,0.10):10.3f} {min(pit):10.3f}")

    print(f"\n{'pair':<10} {'pitch differs':>14} {'max |dpitch| deg':>17} "
          f"{'med |dpitch| (differing)':>26}")
    for a, b in (("A0", "A3"), ("A0", "A1"), ("A1", "A2"), ("A0", "A2")):
        dif = [abs(math.degrees(x["pitch"] - y["pitch"]))
               for x, y in zip(res[a], res[b])]
        nz = [d for d in dif if d > 1e-12]
        print(f"{a}->{b:<6} {len(nz):>9}/{n:<5} {max(dif):17.6f} "
              f"{(st.median(nz) if nz else 0.0):26.6f}")

    # per-aircraft worst-case, so a single sortie cannot hide inside a pool
    print(f"\n{'pair':<10} {'sorties changed':>16} {'worst sortie':>14} "
          f"{'its max |dpitch|':>17}")
    for a, b in (("A0", "A3"), ("A0", "A1"), ("A1", "A2")):
        per = {}
        for x, y in zip(res[a], res[b]):
            d = abs(math.degrees(x["pitch"] - y["pitch"]))
            per[x["tag"]] = max(per.get(x["tag"], 0.0), d)
        ch = {k: v for k, v in per.items() if v > 1e-12}
        w = max(per, key=per.get)
        print(f"{a}->{b:<6} {len(ch):>11}/{len(per):<4} {w:>14} {per[w]:17.6f}")


if __name__ == "__main__":
    main()
