# -*- coding: utf-8 -*-
"""Option (i) cost: what changes if the planning ceiling is lowered below 120 m?
Pure-Python planner runs only (no SITL, nothing written outside this folder)."""
import os, sys, math, json, dataclasses, time
ROOT = r"C:/Users/joy46/Documents/Projects/aeroplan-studio"
sys.path.insert(0, ROOT); os.chdir(ROOT)
OUT = os.path.join(ROOT, "runs/artifacts/PAPERPLAN1/c1_altitude")
import numpy as np
import hv_dccpp as hvlib
from scripts.hv_sitl_scenarios import SCENARIOS
import scripts.hv_sitl_run as R
from hv_dccpp.ardupilot import reduce_waypoints, accept_radius_m, _home_msl, _wgs

base = SCENARIOS["gui_legal_120_1a2b"]
rows = {}
for cap in (120.0, 119.7, 119.5, 119.0):
    sc = dataclasses.replace(base, coverage_altitude_m=cap,
                             waiver_max_agl_m=(None if cap == 120.0 else cap))
    t0 = time.perf_counter()
    hv, rep = R._plan_gui(sc)
    plan, ctx, cfg = hv.last_plan, hv.last_ctx, hv.cfg
    cache = hvlib.build_validation_cache(plan, ctx, budget_only=False)
    v = hvlib.validate(plan, ctx, cache, cfg)
    home_msl = _home_msl(ctx)
    per = {}
    for u, p in sorted(plan.plans.items()):
        spec = ctx.specs[p.cls]
        w_ref = ctx.wind.max_speed() if ctx.wind is not None else 0.0
        pts = reduce_waypoints(p.waypoints, spec, cfg, accept_radius_m(spec.r_eff(w_ref), None))
        xy = np.array([[q.x, q.y] for q in pts], dtype=float)
        rel = _wgs(ctx, xy, np.array([q.z for q in pts]))[:, 2] - home_msl
        per[u] = dict(cls=p.cls, strips=len(p.strips), n_items=len(pts),
                      dist_km=round(p.distance_m / 1000.0, 4),
                      max_export_rel_m=round(float(rel.max()), 4))
    # swath geometry
    w = 2.0 * cap * math.tan(math.radians(base.hfov_deg) / 2.0)
    rows[f"{cap:g}"] = dict(cap_m=cap, swath_m=round(w, 3),
                            spacing_m=round(w * (1 - base.overlap_rate), 3),
                            makespan_s=round(plan.makespan_s, 2),
                            min_sep_h_m=round(v.min_sep_h_m, 3),
                            passed=bool(v.passed), conflicts=len(v.conflicts),
                            uncovered=len(plan.uncovered), status=rep.get("status"),
                            plan_s=round(time.perf_counter() - t0, 1), uavs=per)
    p = rows[f"{cap:g}"]
    print("=" * 100)
    print(f"cap={cap:g} m   swath={p['swath_m']:.2f}  spacing={p['spacing_m']:.2f}  "
          f"makespan={p['makespan_s']:.1f}s  min_sep_h={p['min_sep_h_m']:.2f} m  "
          f"uncovered={p['uncovered']}  ({p['plan_s']}s)")
    for u, d in per.items():
        print(f"    {u} ({d['cls']}) strips={d['strips']:>2} items={d['n_items']:>3} "
              f"dist={d['dist_km']:7.4f} km   max exported rel alt = {d['max_export_rel_m']:.4f} m "
              f"(excess {d['max_export_rel_m']-120.0:+.4f} vs the 120 m cap)")
json.dump(rows, open(os.path.join(OUT, "ablation_cap.json"), "w"), indent=1)
print("\nwrote ablation_cap.json")
