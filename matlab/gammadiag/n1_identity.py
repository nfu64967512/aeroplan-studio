#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""N1c: is the worktree law bit-identical to the RRE-era law with switches off?

If yes, the already-flown RRE batch is a valid control arm and we only need to
fly the treatment.  If no, both arms must be flown.
"""
from __future__ import annotations
import math, os, sys, pathlib
HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(ROOT/"scripts"/"research"/"ditcg"))
import truth32, r1_harness as H, r1_replay as R                  # noqa: E402

AFTER = ROOT/"scripts"/"wsl"/"sar_swarm_patches"/"tot_node.py"
BEFORE = pathlib.Path(os.environ["TOT_BEFORE"])

acs=[]
for run in truth32.RUNS:
    tlat,tlon = truth32._read_target(run)
    for sysid,rows in sorted(truth32._rows(run).items()):
        s = R.states(run,sysid,rows,tlat,tlon)
        if s: acs.append(dict(tag=f"{run}-U{sysid}",st=s,tlat=tlat,tlon=tlon))

OFF = dict(nz_clamp_after_floor=False, nz_closed_loop=False,
           pn_nz_min=-1.0, nz_max_g=3.0, mpng_on=False, itcg_on=False)
bA = H.Bind(AFTER.read_text(encoding="utf-8"), "worktree")
bB = H.Bind(BEFORE.read_text(encoding="utf-8"), "RRE-era")
rA = R.run_config(bA, OFF, acs, bA.ns)
rB = R.run_config(bB, OFF, acs, bA.ns)   # alpha is plant state: same input to both

n = len(rA)
assert n == len(rB), (n, len(rB))
worst = {"roll":0.0, "pitch":0.0, "thr":0.0}
ndiff = 0
for x, y in zip(rA, rB):
    d = False
    for k in worst:
        e = abs(x[k] - y[k])
        if e > worst[k]: worst[k] = e
        if e != 0.0: d = True
    if d: ndiff += 1
print(f"ticks compared      : {n}")
print(f"ticks differing     : {ndiff}")
for k, v in worst.items():
    print(f"  max |d{k:<5}|      : {v:.17g}")
print("\nVERDICT:", "BIT-IDENTICAL -- RRE is a valid control arm"
      if ndiff == 0 else "DIFFERS -- both arms must be flown")
