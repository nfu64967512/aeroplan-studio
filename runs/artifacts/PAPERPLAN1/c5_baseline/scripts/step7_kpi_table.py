# -*- coding: utf-8 -*-
"""C5 step 7 -- roll step5 up into the KPI table, and sanity-check the one
axis I do not trust (corner-fit), so the table says which numbers are solid.
"""
import json
import math
import sys
from pathlib import Path

OUT = Path(__file__).resolve().parents[1] / "out"
scan = json.loads((OUT / "step5_kpi_scan.json").read_text(encoding="utf-8"))
cells = [c for c in scan["cells"] if "error" not in c]

# ---- the 10 cells the paper's 0.9%-22.9% claim is computed over -------------
PAPER10 = {("S1", 4), ("S1", 8), ("S1", 16), ("S2", 4), ("S2", 8),
           ("S3", 4), ("S3", 8), ("S4", 4), ("S4", 8), ("S5", 16)}

print("=" * 96)
print("KPI TABLE -- DCCPP baseline output, measured by HARVEST's own checkers")
print(f"D_use (km): {scan['d_use_km']}   R_plan (m): {scan['r_plan_m']}   "
      f"bench altitude: {scan['bench_alt_m']} m AGL   civil limit: {scan['civil_agl_m']} m")
print("=" * 96)
hdr = f"{'cell':>8} {'fleet':>5} | {'endur>D_use':>11} {'4D conflicts':>12} {'pad conflicts':>13} {'AGL>120m':>9} {'unflyable/corners':>18}"
print(hdr)
print("-" * len(hdr))
agg = {}
for c in cells:
    key = "homo" if not c["mix"] else "mix"
    star = "*" if (c["scenario"], c["n"]) in PAPER10 else " "
    print(f"{star}{c['scenario']}/{c['n']:<4} {key:>5} | {c['n_endurance_over']:11} "
          f"{c.get('sep_conflicts', -1):12} {c.get('pad_conflicts', -1):13} "
          f"{c['n_agl_over_120']:9} {str(c['n_corners_unflyable'])+'/'+str(c['n_corners']):>18}")
    a = agg.setdefault(key, dict(n=0, end=0, sep=0, pad=0, agl=0, cells_sep=0, cells_pad=0,
                                 cells_end=0, cells_agl=0))
    a["n"] += 1
    a["end"] += c["n_endurance_over"]
    a["sep"] += c.get("sep_conflicts", 0)
    a["pad"] += c.get("pad_conflicts", 0)
    a["agl"] += c["n_agl_over_120"]
    a["cells_sep"] += 1 if c.get("sep_conflicts", 0) else 0
    a["cells_pad"] += 1 if c.get("pad_conflicts", 0) else 0
    a["cells_end"] += 1 if c["n_endurance_over"] else 0
    a["cells_agl"] += 1 if c["n_agl_over_120"] else 0
print("-" * len(hdr))
print("* = one of the 10 cells the paper's 0.9%-22.9% claim is computed over")
print()
for k, a in agg.items():
    print(f"{k}: {a['n']} cells | endurance-over in {a['cells_end']} cells ({a['end']} aircraft) "
          f"| 4-D conflicts in {a['cells_sep']} cells ({a['sep']} total) "
          f"| pad conflicts in {a['cells_pad']} cells ({a['pad']} total) "
          f"| AGL>120 in {a['cells_agl']} cells ({a['agl']} aircraft)")

# ---- sanity check on the corner-fit axis -----------------------------------
print()
print("=" * 96)
print("SANITY CHECK on the corner-fit axis (the one number I do NOT trust)")
print("=" * 96)
print("""A DCCPP path is a Dubins curve already DISCRETISED at step_size = 10 m
(core/dccpp/dccpp_path_builder: DCCPPPathBuilder(step_size=10.0)).  On such a
path each vertex is a sample of an arc that is ALREADY of radius R, not a corner
needing a fly-by.  The corner-fit rule 'need = R*tan(delta/2) <= min(leg)/2'
therefore sits exactly on its own threshold for arc samples:""")
R, step = 200.0, 10.0
delta = 2.0 * math.asin(min(1.0, step / (2 * R)))     # deflection between arc samples
need = R * math.tan(delta / 2.0)
print(f"  R = {R} m, arc sample spacing = {step} m -> deflection = {math.degrees(delta):.3f} deg")
print(f"  need = R*tan(delta/2) = {need:.4f} m   vs   min(leg)/2 = {step/2:.4f} m")
print(f"  ratio need/(leg/2) = {need/(step/2):.6f}  <- within {abs(need/(step/2)-1)*100:.4f}% of the threshold")
print("""
=> the corner-fit count is a COIN FLIP on arc samples and must NOT go in the
   paper. The trustworthy turn-radius instrument is the library's own
   circumradius test, hv_dccpp.sim.check_kinematics ('curvature_below_R_eff'),
   which on a 10 m-step arc of radius 200 m correctly returns R = 200 m.
   step4 proved that test fires (synthetic 90-deg corner -> R=141.4 < 200)
   and that on the real DCCPP S4/n=8 path it reports ZERO violations --
   but examined only 5 waypoint triples out of 30 waypoints.""")

rep = dict(paper10=[list(x) for x in sorted(PAPER10)], aggregate=agg,
           corner_fit_verdict="UNRELIABLE on Dubins-discretised arcs; excluded from the paper table",
           corner_fit_need_over_halfleg=need / (step / 2))
(OUT / "step7_kpi_table.json").write_text(json.dumps(rep, indent=2, ensure_ascii=False),
                                          encoding="utf-8")
print("\nwrote", OUT / "step7_kpi_table.json")
