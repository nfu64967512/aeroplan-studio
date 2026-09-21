# -*- coding: utf-8 -*-
"""prep_baseline_control step D -- strict field-by-field diff of the two arms,
and a strict diff of the uncoordinated arm against c5_baseline/out/step5_kpi_scan.json
(the reproduction check).

usage: python stepD_arm_diff.py
"""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
OUT = HERE / "out"
C5 = HERE.parent / "c5_baseline" / "out" / "step5_kpi_scan.json"

ctl = json.loads((OUT / "stepC_kpi_scan_control.json").read_text(encoding="utf-8"))
old = json.loads(C5.read_text(encoding="utf-8"))

KEYS = ["coverage", "max_km", "total_km", "makespan_s", "sep_conflicts",
        "pad_conflicts", "n_endurance_over", "n_agl_over_120"]


def key(c):
    return (c["scenario"], c["n"], bool(c["mix"]))


unc = {key(c): c for c in ctl["arms"]["uncoordinated"] if "error" not in c}
crd = {key(c): c for c in ctl["arms"]["coordinated"] if "error" not in c}
prev = {key(c): c for c in old["cells"] if "error" not in c}

print("=" * 100)
print("D1. uncoordinated (this session) vs c5_baseline step5 (previous session) -- REPRODUCTION")
print("=" * 100)
n_diff = 0
for k in sorted(unc):
    a, b = unc[k], prev.get(k)
    if b is None:
        print(f"  {k} MISSING in previous run"); n_diff += 1; continue
    for f in KEYS:
        if a.get(f) != b.get(f):
            print(f"  {k} {f}: now={a.get(f)!r} prev={b.get(f)!r}"); n_diff += 1
print(f"  -> {len(unc)} cells compared on {len(KEYS)} fields, {n_diff} differences")
print(f"  -> VERDICT: {'REPRODUCED bit-for-bit' if n_diff == 0 else 'DOES NOT REPRODUCE'}")

print()
print("=" * 100)
print("D2. coordinated vs uncoordinated -- THE FAIR-FIGHT CONTROL")
print("=" * 100)
hdr = f"{'cell':>9} {'fleet':>5} | {'pad unc':>7} {'pad crd':>7} {'d':>4} | {'4D unc':>7} {'4D crd':>7} {'d':>4} | {'maxkm unc':>9} {'maxkm crd':>9} | {'wp unc':>6} {'wp crd':>6}"
print(hdr)
print("-" * len(hdr))
tot = dict(pad_u=0, pad_c=0, sep_u=0, sep_c=0, moved=0)
rows = []
for k in sorted(unc):
    a, b = unc[k], crd[k]
    pu, pc = a.get("pad_conflicts", -1), b.get("pad_conflicts", -1)
    su, sc = a.get("sep_conflicts", -1), b.get("sep_conflicts", -1)
    tot["pad_u"] += pu; tot["pad_c"] += pc; tot["sep_u"] += su; tot["sep_c"] += sc
    moved = [f for f in KEYS + ["n_wp_total"] if a.get(f) != b.get(f)]
    if moved:
        tot["moved"] += 1
    rows.append(dict(scenario=k[0], n=k[1], mix=k[2], pad_uncoord=pu, pad_coord=pc,
                     sep_uncoord=su, sep_coord=sc, max_km_uncoord=a["max_km"],
                     max_km_coord=b["max_km"], n_wp_uncoord=a["n_wp_total"],
                     n_wp_coord=b["n_wp_total"], fields_that_moved=moved,
                     coord_calls_coord_arm=b["coord_calls"]))
    print(f"{k[0]+'/'+str(k[1]):>9} {'mix' if k[2] else 'homo':>5} | {pu:7} {pc:7} {pc-pu:4} | "
          f"{su:7} {sc:7} {sc-su:4} | {a['max_km']:9} {b['max_km']:9} | "
          f"{a['n_wp_total']:6} {b['n_wp_total']:6}")
print("-" * len(hdr))
print(f"{'TOTAL':>9} {'':>5} | {tot['pad_u']:7} {tot['pad_c']:7} {tot['pad_c']-tot['pad_u']:4} | "
      f"{tot['sep_u']:7} {tot['sep_c']:7} {tot['sep_c']-tot['sep_u']:4} |")
print()
print(f"cells where ANY of {KEYS + ['n_wp_total']} moved: {tot['moved']} / {len(unc)}")
print()
print("instrumentation (both arms, 48 planner runs total):")
print("  CoordinatedPlanner.__init__          called", ctl["coord_call_totals"]["init"], "times")
print("  CoordinatedPlanner.synchronize       called", ctl["coord_call_totals"]["synchronize"], "times")
print("  CoordinatedPlanner._insert_holding_  called", ctl["coord_call_totals"]["insert_holding"], "times")
print()
print("=> the object is CONSTRUCTED once per coordinated run (swarm_coordinator.py:2290)")
print("   and then NEVER USED: the DCCPP v2 straight-through pipeline (line 2296) returns")
print("   its own dict at line ~2443, and the only two consumers of coord_planner are the")
print("   legacy DCCPPSolver constructions at lines 2463 and 2473, which are downstream of")
print("   that return. coordination_mode is therefore INERT on the benchmark path.")

rep = dict(reproduction_differences=n_diff,
           reproduction_verdict=("REPRODUCED" if n_diff == 0 else "MISMATCH"),
           n_cells=len(unc), keys_compared=KEYS,
           cells_with_any_change_between_arms=tot["moved"],
           pad_total_uncoordinated=tot["pad_u"], pad_total_coordinated=tot["pad_c"],
           sep_total_uncoordinated=tot["sep_u"], sep_total_coordinated=tot["sep_c"],
           coord_call_totals=ctl["coord_call_totals"], per_cell=rows)
f = OUT / "stepD_arm_diff.json"
f.write_text(json.dumps(rep, indent=2, ensure_ascii=False), encoding="utf-8")
print("")
print("wrote", f)
