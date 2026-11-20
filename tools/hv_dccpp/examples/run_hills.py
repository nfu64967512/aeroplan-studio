"""run_hills.py — 「山丘」情境（8 機／3 區／障礙／禁飛／分層風）完整流程 + 基線比較。

    python examples/run_hills.py

build_hills_scenario → build_context → preflight → plan_mission → sim.validate → viz.save_all →
ardupilot.export_all → sim.baselines（hetero vs all_A vs all_B；legacy 另列）。
輸出：figures/hills/*.png、figures/ardupilot/hills/*、figures/hills_baselines.json。
"""
from __future__ import annotations

import dataclasses
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from examples._compat import ARDU_DIR, FIG_DIR, export_ardupilot, export_summary_lines, fmt_table, plan_summary, step, validation_summary  # noqa: E402

from hv_dccpp import sim, viz  # noqa: E402
from hv_dccpp.context import build_context  # noqa: E402
from hv_dccpp.planner import plan_mission  # noqa: E402
from hv_dccpp.preflight import preflight  # noqa: E402
from hv_dccpp.scenario import build_hills_scenario  # noqa: E402

DEM_STEP_M = 50.0
TERRAIN_MARGIN_M = 1500.0
BASELINE_VARIANTS = ("hetero", "all_A", "all_B", "legacy")


def main() -> int:
    t_all = time.perf_counter()
    out_dir = os.path.join(FIG_DIR, "hills")
    ardu_dir = os.path.join(ARDU_DIR, "hills")
    os.makedirs(out_dir, exist_ok=True)

    inputs, cfg = build_hills_scenario()
    cfg = dataclasses.replace(cfg, dem_step_m=DEM_STEP_M)
    print(f"hills scenario: areas {[(a.area_id, a.sensor_req, a.priority, a.time_window_s) for a in inputs.areas]}")
    print(f"  fleet {[u.uav_id + '(' + u.cls + ')' for u in inputs.fleet]}; alternates {[s.site_id for s in inputs.alternates]}; "
          f"obstacles {[o.obs_id for o in inputs.obstacles]}; no_fly {[z.zone_id for z in inputs.airspace.no_fly]}; "
          f"wind {[(w.z_bottom_m, w.z_top_m, w.speed_mps, w.dir_from_deg) for w in inputs.wind]}")

    ctx = step("build_context", lambda: build_context(inputs, cfg, terrain_margin_m=TERRAIN_MARGIN_M))
    if ctx is None:
        return 1
    print(f"h_clr={ctx.h_clr_m}, h_ceil={ctx.h_ceil_m} ({ctx.h_ceil_source}), "
          f"keepouts={None if ctx.keepouts is None else list(ctx.keepouts.ids)}, sensors={sorted(ctx.sensors)}")

    rep = step("preflight", lambda: preflight(inputs, cfg, ctx))
    if rep is not None:
        print(f"preflight status={rep.status}; errors={rep.errors}; n_warnings={len(rep.warnings)}")
        for w in rep.warnings[:12]:
            print(f"  warn: {w}")
        for aid, pa in rep.per_area.items():
            cl = {c: (d.get('feasible'), d.get('reason'), round(d.get('h_op_m') or 0, 1)) for c, d in pa.get("classes", {}).items()}
            print(f"  per_area[{aid}]: feasible_classes={pa.get('feasible_classes')}, coverable={pa.get('coverable')}, "
                  f"dist_home={pa.get('dist_home_m', 0) / 1000:.1f} km, classes={cl}")

    plan = step("plan_mission", lambda: plan_mission(inputs, cfg, ctx=ctx))
    if plan is None:
        return 1
    print(plan_summary(plan, ctx))
    cell_rows = [[c.cell_id, c.area_id, c.sensor_req, {k: v for k, v in c.n_strips.items()},
                  {k: round(math.degrees(v)) for k, v in c.alpha_rad.items() if c.feasible.get(k)},
                  {k: round(v, 1) for k, v in c.h_op_m.items()}, c.parent_cell_id or ""] for c in plan.cells]
    print(fmt_table(cell_rows, ["cell", "area", "req", "n_strips", "alpha_deg", "h_op", "parent"]))
    for uid, p in sorted(plan.plans.items()):
        print(f"  {uid}: {[s for s, _d in p.strips]}")
        if p.warnings:
            print(f"  {uid} warnings({len(p.warnings)}): {p.warnings[:4]}{' ...' if len(p.warnings) > 4 else ''}")

    cache = step("build_validation_cache", lambda: sim.build_validation_cache(plan, ctx, budget_only=True))
    vrep = step("sim.validate", lambda: sim.validate(plan, ctx, cache, cfg)) if cache is not None else None
    if vrep is not None:
        print(validation_summary(vrep))

    paths = step("viz.save_all", lambda: viz.save_all(plan, ctx, vrep, out_dir))
    if paths:
        print("figures: " + ", ".join(os.path.relpath(p, FIG_DIR) for p in paths))

    exported = step("ardupilot.export_all", lambda: export_ardupilot(plan, ctx, cfg, ardu_dir))
    if exported:
        name, summary = exported
        print(f"  exporter = {name}; out_dir = {os.path.relpath(ardu_dir, FIG_DIR)}")
        for uid, d in summary.get("uavs", {}).items():
            print(f"  {uid}: n_items={d.get('n_items')} n_nav={d.get('n_nav')}")

    # ---- 基線比較（規格 §17.1 (6)）--------------------------------------------
    base = step("sim.baselines", lambda: sim.baselines(inputs, cfg, terrain_margin_m=TERRAIN_MARGIN_M,
                                                      variants=BASELINE_VARIANTS))
    if base:
        rows = []
        for v in BASELINE_VARIANTS:
            mk = base.get(v, math.inf)
            rows.append([v, f"{mk:.0f}" if math.isfinite(mk) else "inf (incomplete/NO_GO)",
                         f"{mk / 60:.1f}" if math.isfinite(mk) else "-",
                         f"{base.get(v + '_distance_m', math.inf) / 1000:.1f}", f"{base.get(v + '_uncovered', math.inf):.0f}",
                         f"{base.get(v + '_coverage_min', float('nan')):.3f}",
                         f"{base.get(v + '_compute_ms', 0) / 1000:.1f}"])
        print(fmt_table(rows, ["variant", "makespan_s", "min", "dist_km", "uncovered", "cov_min", "compute_s"]))
        h = base.get("hetero", math.inf)
        if math.isfinite(h):
            for v in [x for x in BASELINE_VARIANTS if x != "hetero"]:
                mk = base.get(v, math.inf)
                if math.isfinite(mk):
                    print(f"hetero vs {v}: {100 * (mk - h) / h:+.1f} %  (positive = hetero faster)")
                else:
                    print(f"hetero vs {v}: n/a - {v} is not a complete mission "
                          f"(uncovered={base.get(v + '_uncovered', float('nan')):.0f}, "
                          f"min coverage={base.get(v + '_coverage_min', float('nan')):.3f})")
        with open(os.path.join(FIG_DIR, "hills_baselines.json"), "w", encoding="utf-8") as f:
            json.dump(base, f, indent=1, default=lambda x: None if isinstance(x, float) and not math.isfinite(x) else x)

    print(f"\nTOTAL {time.perf_counter() - t_all:.1f} s; validation {'PASS' if vrep is not None and vrep.passed else 'FAIL'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
