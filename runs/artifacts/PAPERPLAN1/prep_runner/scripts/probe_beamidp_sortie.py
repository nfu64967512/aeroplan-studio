# -*- coding: utf-8 -*-
"""D3 pre-check: is the paper's own scenario actually interlaced, and what would
an interlaced sortie cost?

PLANNING ONLY. No SITL / ArduPilot / MAVProxy process is started.
Writes: runs/artifacts/PAPERPLAN1/prep_runner/beamidp_sortie.json
"""
import dataclasses
import json
import math
import os
import sys
import time
from pathlib import Path

ROOT = Path(r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
OUT = ROOT / "runs/artifacts/PAPERPLAN1/prep_runner"
os.chdir(ROOT)
sys.path.insert(0, str(ROOT))
for _k in [k for k in os.environ if k.startswith("HV_")]:
    os.environ.pop(_k)

import hv_dccpp as hvlib                                    # noqa: E402
import scripts.hv_sitl_run as R                             # noqa: E402
from hv_dccpp.dubins import skip_k                          # noqa: E402
from scripts.hv_sitl_scenarios import SCENARIOS, rect       # noqa: E402

CEIL = float(os.environ.get("PROBE_CEIL", "119.5"))


def census(hv):
    """Per-cell n / s / R_eff / k / does skip-k differ from sequential?"""
    plan, ctx = hv.last_plan, hv.last_ctx
    rows = []
    cells = getattr(ctx, "cells", None) or {}
    for cid, cell in sorted(cells.items(), key=lambda kv: str(kv[0])):
        strips = list(getattr(cell, "strips", ()) or ())
        n = len(strips)
        cls = str(getattr(cell, "cls", "") or "")
        s = float(getattr(cell, "spacing_m", 0.0) or 0.0)
        if not (n and cls and s > 0):
            continue
        w = ctx.wind.max_speed() if ctx.wind is not None else 0.0
        r = float(ctx.specs[cls].r_eff(w))
        k = int(skip_k(s, r))
        rows.append({"cell": str(cid), "cls": cls, "n_strips": n,
                     "spacing_m": round(s, 3), "R_eff_m": r, "k": k,
                     "interlaces": bool(n > k)})
    return rows


def run(tag, sc):
    t0 = time.perf_counter()
    hv, rep = R._plan_gui(sc)
    plan, ctx, cfg = hv.last_plan, hv.last_ctx, hv.cfg
    cache = hvlib.build_validation_cache(plan, ctx, budget_only=False)
    v = hvlib.validate(plan, ctx, cache, cfg)
    dt = time.perf_counter() - t0
    rows = census(hv)
    out = {"tag": tag, "plan_s": round(dt, 1), "status": rep.get("status"),
           "makespan_s": round(plan.makespan_s, 2),
           "validate_passed": bool(v.passed), "conflicts": len(v.conflicts),
           "uncovered": len(plan.uncovered),
           "min_sep_h_m": round(v.min_sep_h_m, 3),
           "total_distance_m": round(sum(p.distance_m for p in plan.plans.values()), 1),
           "wall_clock_min_at_speedup_5": round(plan.makespan_s / 5.0 / 60.0, 1),
           "per_uav": {u: {"cls": p.cls, "strips": len(p.strips),
                           "distance_km": round(p.distance_m / 1000.0, 3),
                           "energy_used_wh": round(p.energy_used_wh, 1),
                           "energy_margin_wh": round(p.energy_margin_wh, 1),
                           "t_takeoff_s": round(p.t_takeoff_s, 1),
                           "t_land_s": round(p.t_land_s, 1)}
                       for u, p in sorted(plan.plans.items())},
           "cells": rows,
           "n_cells_interlacing": sum(r["interlaces"] for r in rows),
           "n_cells": len(rows)}
    print(json.dumps(out, ensure_ascii=False, indent=1), flush=True)
    return out


base = SCENARIOS["gui_legal_120_1a2b"]
res = {}
# (1) the paper's own cell, at the D2 ceiling -- does anything interlace?
paper = dataclasses.replace(base, waiver_max_agl_m=CEIL, coverage_altitude_m=CEIL)
res["paper_cell_1560x800"] = run("paper_cell_1560x800", paper)
# (2) the geometry c3 measured as triggering: 3000 x 3000 m, same fleet/airspace
big = dataclasses.replace(base, waiver_max_agl_m=CEIL, coverage_altitude_m=CEIL,
                          areas=[{"area_id": 1, "priority": 1.0,
                                  "polygon": rect(0, 0, 1500, 1500)}])
res["interlaced_3000x3000"] = run("interlaced_3000x3000", big)
res["_note"] = ("PLANNING ONLY. CEIL=%.1f m applied as waiver_max_agl_m and "
                "coverage_altitude_m, the same pair HV_PLAN_CEIL_M sets." % CEIL)
(OUT / "beamidp_sortie.json").write_text(json.dumps(res, ensure_ascii=False, indent=1),
                                         encoding="utf-8")
print("wrote", OUT / "beamidp_sortie.json")
