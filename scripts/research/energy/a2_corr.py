#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A2: correlations between energy variables and the two failure metrics."""
from __future__ import annotations
import json, os, math, sys, statistics as st
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "ditcg"))
import truth32                                   # noqa: E402

rows = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   "a1_rows.json")))
tr = {d["tag"]: d for d in truth32.load()}
for r in rows:
    t = tr[r["tag"]]
    r["t_rem0"] = t["t_rem0_s"]
    r["eps0"] = abs(t["eps_deg"])
    r["Esurp0"] = r["E0"] - r["h0"] - 0.0          # = V0^2/2g
    # required glide gradient at release, and the "hold-current-gamma" residual
    r["req_grad"] = r["h0"] / r["d0"]
    r["diss_ratio"] = abs(r["dE_dr"]) / r["req_grad"]

def rank(v):
    o = sorted(range(len(v)), key=lambda i: v[i]); rk = [0.0]*len(v); i = 0
    while i < len(o):
        j = i
        while j+1 < len(o) and v[o[j+1]] == v[o[i]]: j += 1
        for k in range(i, j+1): rk[o[k]] = 0.5*(i+j)+1.0
        i = j+1
    return rk

def sp(a, b):
    ra, rb = rank(a), rank(b)
    ma, mb = st.mean(ra), st.mean(rb)
    num = sum((x-ma)*(y-mb) for x, y in zip(ra, rb))
    den = math.sqrt(sum((x-ma)**2 for x in ra)*sum((y-mb)**2 for y in rb))
    return num/den if den else float("nan")

def pe(a, b):
    ma, mb = st.mean(a), st.mean(b)
    num = sum((x-ma)*(y-mb) for x, y in zip(a, b))
    den = math.sqrt(sum((x-ma)**2 for x in a)*sum((y-mb)**2 for y in b))
    return num/den if den else float("nan")

main = [r for r in rows if r["r_td"] < 100.0]   # drop the 2 gross escapes
print(f"n_all = {len(rows)}   n_main(r_td<100) = {len(main)}\n")
targets = [("zres", "residual alt"), ("r_td", "touchdown err"), ("T", "dive time")]
preds = ["t_rem0","V0","Vtd","E0","Etd","dE","dE_dr","diss_ratio","h0","gam0",
         "eps0","ratio","path","gam_p05"]
for tk, tn in targets:
    print(f"== target {tk} ({tn}) ==   [all32 | main30]")
    for p in preds:
        s_all = sp([r[p] for r in rows], [r[tk] for r in rows])
        s_m   = sp([r[p] for r in main], [r[tk] for r in main])
        print(f"   sp({p:<10s}) = {s_all:+.3f} | {s_m:+.3f}")
    print()

print("== four residual-altitude failures vs the other 28 ==")
fail = sorted(rows, key=lambda r: -r["zres"])[:4]
rest = sorted(rows, key=lambda r: -r["zres"])[4:]
for k in ("t_rem0","V0","Vtd","E0","Etd","dE_dr","diss_ratio","req_grad","h0",
          "gam_p05","ratio","T"):
    fv = [r[k] for r in fail]; rv = [r[k] for r in rest]
    print(f"  {k:<11s} fail med {st.median(fv):8.3f} [{min(fv):8.3f},{max(fv):8.3f}]   "
          f"rest med {st.median(rv):8.3f} [{min(rv):8.3f},{max(rv):8.3f}]")
