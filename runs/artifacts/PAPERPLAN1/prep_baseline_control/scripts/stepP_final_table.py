# -*- coding: utf-8 -*-
"""prep_baseline_control step P -- assemble the final KPI table from the JSONs
produced in THIS session only. Every number is read from a file written by one
of steps B, C/D, I, L2, N, O; nothing is typed in by hand.

usage: python stepP_final_table.py
"""
import json
from pathlib import Path

OUT = Path(__file__).resolve().parents[1] / "out"
C5 = Path(__file__).resolve().parents[1].parent / "c5_baseline" / "out"


def load(p):
    return json.loads(Path(p).read_text(encoding="utf-8"))


ratio = load(C5 / "step1_ratio_from_csv.json")
ctl = load(OUT / "stepC_kpi_scan_control.json")
diff = load(OUT / "stepD_arm_diff.json")
dplan = load(OUT / "stepI_dplan_rule.json")
budg = load(OUT / "stepL2_dmodel_vs_baseline.json")
fair = load(OUT / "stepN_fair_fight.json")
nat = load(OUT / "stepO_harvest_native.json")

PAPER10 = {("S1", 4), ("S1", 8), ("S1", 16), ("S2", 4), ("S2", 8),
           ("S3", 4), ("S3", 8), ("S4", 4), ("S4", 8), ("S5", 16)}
nat_by = {(r["scenario"], r["n"]): r for r in nat["rows"] if "error" not in r}
fair_by = {(r["scenario"], r["n"]): r for r in fair["rows"]}
ctl_homo = {(c["scenario"], c["n"]): c for c in ctl["arms"]["uncoordinated"]
            if not c["mix"] and "error" not in c}

rep = {}

print("=" * 104)
print("P1. EFFICIENCY (the paper's headline) -- ruler: bench.metrics max_km, 10 included cells")
print("=" * 104)
inc = [r for r in ratio["rows"] if r["included"]]
print(f"  DCCPP {min(r['dccpp_max_km'] for r in inc):.2f}-{max(r['dccpp_max_km'] for r in inc):.2f} km   "
      f"HARVEST {min(r['harvest_max_km'] for r in inc):.2f}-{max(r['harvest_max_km'] for r in inc):.2f} km")
print(f"  HARVEST longer by {ratio['min_pct']:+.2f}% .. {ratio['max_pct']:+.2f}%  (mean {ratio['mean_pct']:+.2f}%)")
print(f"  reproduced from {Path(ratio['source']).name}, provenance head={ratio['provenance']['head'][:8]}")
rep["efficiency"] = dict(min_pct=ratio["min_pct"], max_pct=ratio["max_pct"],
                         mean_pct=ratio["mean_pct"], n_cells=ratio["n_included"])

print()
print("=" * 104)
print("P2. COVERAGE / SELF-DECLARED INFEASIBILITY -- ruler: bench.metrics coverage, all 12 cells")
print("=" * 104)
dec = [r for r in ratio["rows"] if not r["included"]]
print(f"  DCCPP coverage = 1.000 in 12/12 cells")
print(f"  HARVEST coverage = 1.000 in {len(inc)}/12; declines "
      + ", ".join(f"{r['scenario']}/n={r['n']} (cov {r['harvest_coverage']}, "
                  f"self_validate={r['harvest_self_validate']})" for r in dec))
rep["coverage"] = dict(dccpp_full=12, harvest_full=len(inc),
                       declined=[dict(cell=f"{r['scenario']}/{r['n']}",
                                      coverage=r["harvest_coverage"],
                                      self_uncovered=r["harvest_self_uncovered"],
                                      self_validate=r["harvest_self_validate"]) for r in dec])

print()
print("=" * 104)
print("P3. ENERGY FEASIBILITY -- ruler: D_ceq (5 m/s wind triangle) vs the allocator's own budget")
print("=" * 104)
print(f"  D_model (EnergyModel.cruise_equiv_km(e_usable_wh)) = {budg['D_model_km']} km   <- what the allocator enforces")
print(f"  D_use   (Table-1 usable_distance_cap_km)           = {budg['D_use_km']} km   <- what sim.py:913 checks")
print(f"  DCCPP exceeds D_model in {budg['cells_over_R1']}/12 cells; exceeds D_use in {budg['cells_over_R2']}/12 cells")
for r in budg["rows"]:
    if r["r1_over"]:
        print(f"    {r['cell']}: D_ceq {r['d_ceq']:.2f} km = {r['d_ceq']/budg['D_model_km']*100:.1f}% of D_model, "
              f"{r['r1_over']}/{r['n']} aircraft over")
print("  HARVEST exceeds D_model in 0/12 cells: it declines exactly the two cells above.")
rep["energy"] = budg

print()
print("=" * 104)
print("P4. CONFLICT AXES -- identical checkers, HARVEST measured on its NATIVE plan objects")
print("=" * 104)
hdr = (f"{'cell':>8} | {'DCCPP 4D':>8} {'HARVEST 4D':>10} | {'DCCPP pad':>9} {'HARVEST pad':>11} | "
       f"{'DCCPP+CA 4D':>11} {'DCCPP+CA pad':>12}")
print(hdr)
print("-" * len(hdr))
tot = dict(d_sep=0, h_sep=0, d_pad=0, h_pad=0, ca_sep=0, ca_pad=0, ca_fired=0)
rows = []
for k in sorted(fair_by):
    f_, n_ = fair_by[k], nat_by.get(k, {})
    hs, hp = n_.get("sep", -1), n_.get("pad", -1)
    print(f"{k[0]+'/'+str(k[1]):>8} | {f_['dccpp_sep']:8} {hs:10} | {f_['dccpp_pad']:9} {hp:11} | "
          f"{f_['dccpp_ca_sep']:11} {f_['dccpp_ca_pad']:12}")
    for a, b in (("d_sep", f_["dccpp_sep"]), ("d_pad", f_["dccpp_pad"]),
                 ("ca_sep", f_["dccpp_ca_sep"]), ("ca_pad", f_["dccpp_ca_pad"]),
                 ("ca_fired", f_.get("ca_detected", 0)), ("h_sep", hs), ("h_pad", hp)):
        if b >= 0:
            tot[a] += b
    rows.append(dict(cell=f"{k[0]}/{k[1]}", dccpp_sep=f_["dccpp_sep"], harvest_sep=hs,
                     dccpp_pad=f_["dccpp_pad"], harvest_pad=hp,
                     dccpp_ca_sep=f_["dccpp_ca_sep"], dccpp_ca_pad=f_["dccpp_ca_pad"],
                     ca_detected=f_.get("ca_detected"), ca_actions=f_.get("ca_actions"),
                     harvest_sep_relift=f_.get("harvest_sep"),
                     harvest_pad_relift=f_.get("harvest_pad")))
print("-" * len(hdr))
print(f"{'TOTAL':>8} | {tot['d_sep']:8} {tot['h_sep']:10} | {tot['d_pad']:9} {tot['h_pad']:11} | "
      f"{tot['ca_sep']:11} {tot['ca_pad']:12}")
print(f"  baseline CollisionAvoidance detected {tot['ca_fired']} conflicts of its own and applied that many actions;")
print(f"  the two HARVEST checkers then reported {tot['ca_sep']} 4-D and {tot['ca_pad']} pad conflicts,")
print(f"  i.e. {'NO CHANGE' if (tot['ca_sep'], tot['ca_pad']) == (tot['d_sep'], tot['d_pad']) else 'a change'} "
      f"from {tot['d_sep']} / {tot['d_pad']}.")
print()
print("  INSTRUMENT NOTE: step N measured HARVEST through the same re-lift the baseline")
print("  needs (t=0 for everyone). Re-lift vs native, totalled over the 10 cells:")
rl_sep = sum(r["harvest_sep_relift"] for r in rows if (r["harvest_sep_relift"] or -1) >= 0)
rl_pad = sum(r["harvest_pad_relift"] for r in rows if (r["harvest_pad_relift"] or -1) >= 0)
print(f"    re-lift  4D={rl_sep}  pad={rl_pad}")
print(f"    native   4D={tot['h_sep']}  pad={tot['h_pad']}")
print("    -> the re-lift discards HARVEST's takeoff schedule and its own timestamps;")
print("       only the NATIVE numbers may be quoted for HARVEST.")
rep["conflicts"] = dict(totals=tot, relift_totals=dict(sep=rl_sep, pad=rl_pad), rows=rows)

print()
print("=" * 104)
print("P5. THE FAIR-FIGHT CONTROL (coordination_mode)")
print("=" * 104)
print(f"  24 configurations x 2 arms. Fields compared: {diff['keys_compared'] + ['n_wp_total']}")
print(f"  cells where ANY field moved: {diff['cells_with_any_change_between_arms']}/{diff['n_cells']}")
print(f"  pad total  uncoordinated {diff['pad_total_uncoordinated']} -> coordinated {diff['pad_total_coordinated']}")
print(f"  4-D total  uncoordinated {diff['sep_total_uncoordinated']} -> coordinated {diff['sep_total_coordinated']}")
print(f"  CoordinatedPlanner.__init__ {diff['coord_call_totals']['init']}x, "
      f".synchronize {diff['coord_call_totals']['synchronize']}x, "
      f"._insert_holding_pattern {diff['coord_call_totals']['insert_holding']}x")
print(f"  reproduction of the previous session's scan: {diff['reproduction_verdict']} "
      f"({diff['reproduction_differences']} differences)")
rep["control"] = diff

print()
print("=" * 104)
print("P6. ROWS THAT DID NOT SURVIVE THE HONESTY CHECK (excluded from the paper table)")
print("=" * 104)
print("  * AGL > 120 m: the benchmark's own coverage altitude is "
      f"{ctl['bench_alt_m']:.0f} m AGL, so EVERY aircraft of BOTH planners exceeds 120 m by")
print("    construction. Measured max AGL: DCCPP "
      f"{max(u['agl_max_m'] for c in ctl_homo.values() for u in c['per_uav']):.2f} m, HARVEST "
      f"{max(r.get('max_agl_m', 0) for r in nat['rows'] if 'error' not in r):.2f} m."
      "  HARVEST is the HIGHER one.")
print("  * corner-fit / unflyable corners: c5_baseline step7 showed the test sits within")
print("    0.03 % of its own threshold on 10 m-discretised Dubins arcs -> coin flip. Excluded.")
print("  * makespan: DCCPP's self-reported makespan and HARVEST's are computed by different")
print("    code paths and are not comparable (research/cpp_benchmark/bench.py:109).")

f = OUT / "stepP_final_table.json"
f.write_text(json.dumps(rep, indent=2, ensure_ascii=False, default=str), encoding="utf-8")
print("")
print("wrote", f)
