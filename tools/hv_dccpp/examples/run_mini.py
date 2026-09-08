"""run_mini.py — 契約 §W mini example 完整流程。

    python examples/run_mini.py

build_mini_example → build_context → preflight → plan_mission → sim.validate → viz.save_all →
ardupilot.export_all（`hv_dccpp.ardupilot` 缺席時用 examples/_compat.py 之後備匯出）。
輸出：figures/mini/*.png、figures/ardupilot/mini/*.waypoints|.param|.json、figures/mini_plan.json。
"""
from __future__ import annotations

import dataclasses
import json
import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from examples._compat import ARDU_DIR, FIG_DIR, export_ardupilot, export_summary_lines, plan_summary, step, validation_summary  # noqa: E402

from hv_dccpp import sim, viz  # noqa: E402
from hv_dccpp.context import build_context  # noqa: E402
from hv_dccpp.planner import plan_mission  # noqa: E402
from hv_dccpp.preflight import preflight  # noqa: E402
from hv_dccpp.scenario import build_mini_example  # noqa: E402

DEM_STEP_M = 50.0          # 測試等級解析度（預設 10 m 需 ~1.7 s 重取樣）
TERRAIN_MARGIN_M = 1500.0  # 起飛／進入迴圈可達 home 上風側 ~R_eff 外（altitude 註記）


def main() -> int:
    t_all = time.perf_counter()
    out_dir = os.path.join(FIG_DIR, "mini")
    ardu_dir = os.path.join(ARDU_DIR, "mini")
    os.makedirs(out_dir, exist_ok=True)

    inputs, cfg = build_mini_example()
    cfg = dataclasses.replace(cfg, dem_step_m=DEM_STEP_M)
    print(f"mini example: home {inputs.home.latlon}, areas {[a.area_id for a in inputs.areas]}, "
          f"fleet {[u.uav_id + '(' + u.cls + ')' for u in inputs.fleet]}, alternates {[s.site_id for s in inputs.alternates]}")

    ctx = step("build_context", lambda: build_context(inputs, cfg, terrain_margin_m=TERRAIN_MARGIN_M))
    if ctx is None:
        return 1
    print(f"h_clr={ctx.h_clr_m}, h_ceil={ctx.h_ceil_m} ({ctx.h_ceil_source}), keepouts="
          f"{None if ctx.keepouts is None else len(ctx.keepouts.ids)}, warnings={len(ctx.warnings)}")

    rep = step("preflight", lambda: preflight(inputs, cfg, ctx))
    if rep is not None:
        print(f"preflight status={rep.status}; errors={rep.errors}; warnings({len(rep.warnings)})={rep.warnings[:6]}")
        for cls, d in rep.derived.items():
            print(f"  derived[{cls}]: R_eff={d.get('R_eff')}, h_op={d.get('h_op')}, w={d.get('w')}, s={d.get('s')}, "
                  f"e_ext={d.get('e_ext')}, E_use_rep={d.get('E_use_rep')}, t_takeoff_s={d.get('t_takeoff_s')}, "
                  f"t_landing_s={d.get('t_landing_s')}, usable_alternates={d.get('usable_alternates')}")
        for aid, pa in rep.per_area.items():
            print(f"  per_area[{aid}]: feasible_classes={pa.get('feasible_classes')}, coverable={pa.get('coverable')}, "
                  f"T_oper_lb_s={pa.get('T_oper_lb_s')}")

    plan = step("plan_mission", lambda: plan_mission(inputs, cfg, ctx=ctx))
    if plan is None:
        return 1
    print(plan_summary(plan, ctx))
    for uid, p in plan.plans.items():
        print(f"  {uid} strips: {p.strips}")
        if p.warnings:
            print(f"  {uid} warnings: {p.warnings[:5]}{' ...' if len(p.warnings) > 5 else ''}")

    cache = step("build_validation_cache", lambda: sim.build_validation_cache(plan, ctx, budget_only=True))
    vrep = step("sim.validate", lambda: sim.validate(plan, ctx, cache, cfg)) if cache is not None else None
    if vrep is not None:
        print(validation_summary(vrep))

    def _track():
        out = {}
        for uid, p in plan.plans.items():
            tr = sim.track_simulate(p, ctx.specs[p.cls], ctx.wind, ctx.models[uid], cfg, dt_s=0.5)
            out[uid] = tr
            print(f"  track_simulate {uid}: max_xt={tr.max_cross_track_m:.1f} m, energy_dev={tr.energy_dev_frac * 100:.2f} %, "
                  f"t_land={tr.t_land_s:.0f} s, completed={tr.completed}")
        return out
    step("sim.track_simulate", _track)

    paths = step("viz.save_all", lambda: viz.save_all(plan, ctx, vrep, out_dir))
    if paths:
        print("figures: " + ", ".join(os.path.relpath(p, FIG_DIR) for p in paths))

    exported = step("ardupilot.export_all", lambda: export_ardupilot(plan, ctx, cfg, ardu_dir))
    if exported:
        name, summary = exported
        print(f"  exporter = {name}")
        for ln in export_summary_lines(summary, FIG_DIR):
            print(ln)
        # 以 pymavlink mavwp 讀回驗證（ardupilot_notes §1）
        def _verify():
            os.environ.setdefault("MAVLINK20", "1")
            from pymavlink import mavwp
            for uid in plan.plans:
                p = os.path.join(ardu_dir, f"{uid}.waypoints")
                if os.path.exists(p):
                    loader = mavwp.MAVWPLoader()
                    n = loader.load(p)
                    cmds = sorted({int(w.command) for w in loader.wpoints})
                    print(f"  mavwp loaded {uid}.waypoints: {n} items, commands {cmds}")
        step("mavwp read-back", _verify)

    def _dump():
        p = os.path.join(FIG_DIR, "mini_plan.json")
        with open(p, "w", encoding="utf-8") as f:
            f.write(plan.to_json())
        return p
    p = step("plan.to_json", _dump)
    if p:
        print("plan json:", os.path.relpath(p, FIG_DIR), f"({os.path.getsize(p) / 1e6:.1f} MB)")

    print(f"\nTOTAL {time.perf_counter() - t_all:.1f} s; validation {'PASS' if vrep is not None and vrep.passed else 'FAIL'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
