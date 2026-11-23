#!/usr/bin/env python3
"""M2 pass 5 -- do the ~3.6 s node stalls land inside the PN window?

If they do, the FC holds the last SET_ATTITUDE_TARGET for that whole stretch,
which is a genuine chain node.  Measured directly from CSV timestamp gaps.
"""
import csv, math, json
import numpy as np

REPO = "/mnt/c/Users/joy46/Documents/Projects/aeroplan-studio"
EARTH = 111319.4917
TRIALS = [f"RRD{i}" for i in range(1, 9)] + [f"RRE{i}" for i in range(1, 9)]

tot_gap, tot_dur, n_ev, worst = [], [], [], []
for tr in TRIALS:
    gr = float(json.load(open(f"{REPO}/runs/artifacts/{tr}/contract_{tr}.json"))["values"]["gate_r"])
    tgt = [r for r in csv.DictReader(open(f"{REPO}/runs/artifacts/{tr}/tot_{tr}_geom.csv"))
           if r["kind"] == "target"][0]
    TL, TO = float(tgt["lat"]), float(tgt["lon"])
    for s in (1, 2):
        rows = [r for r in csv.DictReader(open(f"{REPO}/runs/artifacts/{tr}/tot_{tr}.csv"))
                if r["sysid"] == str(s)]
        t = np.array([float(r["t"]) for r in rows])
        ph = np.array([r["phase"] for r in rows])
        la = np.array([float(r["lat"]) for r in rows])
        lo = np.array([float(r["lon"]) for r in rows])
        d = np.hypot((TL - la) * EARTH,
                     (TO - lo) * EARTH * math.cos(math.radians(float(la.mean()))))
        hits, i0 = 0, None
        for i in np.where(ph == "DIVE")[0]:
            hits = hits + 1 if d[i] <= gr else 0
            if hits >= 3:
                i0 = i
                break
        if i0 is None:
            continue
        td = None
        for r in csv.DictReader(open(f"{REPO}/matlab/data/truth32.csv")):
            if r["trial"] == tr and r["sysid"] == str(s):
                td = float(r["t_touch_row_s"])
        i1 = int(np.searchsorted(t, td)) if td else len(t)
        tt = t[i0:max(i1, i0 + 2)]
        g = np.diff(tt)
        big = g[g > 0.3]
        tot_gap.append(float(big.sum()))
        tot_dur.append(float(tt[-1] - tt[0]))
        n_ev.append(int(len(big)))
        worst.append(float(big.max()) if len(big) else 0.0)

import statistics as st
print("node stalls inside the PN-engaged window (n=32 sorties)")
print(f"  PN window duration      median {st.median(tot_dur):.1f} s")
print(f"  stall events per sortie median {st.median(n_ev):.1f}  (max {max(n_ev)})")
print(f"  total stalled time      median {st.median(tot_gap):.2f} s  (max {max(tot_gap):.2f} s)")
print(f"  longest single stall    median {st.median(worst):.2f} s  (max {max(worst):.2f} s)")
print(f"  stalled fraction of PN  median "
      f"{st.median([a/b for a, b in zip(tot_gap, tot_dur)]):.2%}")
print(f"  sorties with zero stalls: {sum(1 for x in n_ev if x == 0)}/32")
