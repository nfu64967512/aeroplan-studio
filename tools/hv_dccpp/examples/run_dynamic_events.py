"""run_dynamic_events.py — mini example 之動態事件重規劃示範（規格 §12）。

    python examples/run_dynamic_events.py

plan_mission → 自計畫在 t=600 s 內插遙測 → 依序觸發 UAV_EXIT、NEW_AREA、WIND_UPDATE、TARGET_CUE、ENERGY_UPDATE：
(1) 隔離模式：每個事件皆自原始計畫出發（各自重建 ctx/cache），列出變更機、耗時與備註；
(2) 串接模式：res.plan 作為下一事件之輸入（時間遞增 600→840 s），UAV_EXIT(A1) 後之事件以 B1 為主體。
輸出：figures/dynamic/before/*.png、figures/dynamic/after/*.png、figures/dynamic/chain_<i>_<event>.png、
figures/dynamic_events.json。
"""
from __future__ import annotations

import dataclasses
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from examples._compat import FIG_DIR, fmt_table, plan_summary, step, telemetry_from_plan  # noqa: E402

import numpy as np  # noqa: E402

from hv_dccpp import dynamic as dyn  # noqa: E402
from hv_dccpp import sim, viz  # noqa: E402
from hv_dccpp.context import build_context  # noqa: E402
from hv_dccpp.mission import AreaSpec, Phase, WindLayer  # noqa: E402
from hv_dccpp.planner import plan_mission  # noqa: E402
from hv_dccpp.scenario import build_mini_example  # noqa: E402

DEM_STEP_M = 50.0
TERRAIN_MARGIN_M = 1500.0
T_TELEMETRY_S = 600.0
E_TEL_DROP_WH = {"A": 400.0, "B": 110.0}   # 事件注入之遙測電量（明顯低於計畫預測 → κ 上修／DIVERT）


def _new_area(ctx) -> AreaSpec:
    """Z1 北側 800×500 m 之 rgb 新區（ENU (4500..5300, 1200..1700)）。"""
    ring = ctx.frame.to_wgs84(np.array([[4500.0, 1200.0, 0.0], [5300.0, 1200.0, 0.0], [5300.0, 1700.0, 0.0],
                                        [4500.0, 1700.0, 0.0]]))
    return AreaSpec("NEW1", tuple((float(a), float(b)) for a, b, _ in ring), sensor_req="rgb", priority=1.0,
                    payload_req_id="default_rgb")


def _events(ctx, plan, t0: float, *, survivor_only: bool = False):
    """事件清單 [(name, factory(t) -> event)]；`survivor_only` 時以 B1 為主體（A1 已退出）。"""
    cue_ll = ctx.frame.to_wgs84(np.array([[5500.0, -200.0, 0.0]]))[0]
    src = "B1" if survivor_only else "A1"
    e_uav = "B1" if survivor_only else "A1"
    return [
        ("UAV_EXIT", lambda t: dyn.UAVExit("A1", t, "motor_failure")),
        ("NEW_AREA", lambda t: dyn.NewArea(_new_area(ctx), t)),
        ("WIND_UPDATE", lambda t: dyn.WindUpdate((WindLayer(0.0, 3000.0, 6.5, 250.0, 9.0, "agl"),), t, None)),
        ("TARGET_CUE", lambda t: dyn.TargetCue((float(cue_ll[0]), float(cue_ll[1])), src, 0.9, t)),
        ("ENERGY_UPDATE", lambda t: dyn.EnergyUpdate(e_uav, t, E_TEL_DROP_WH[plan.plans[e_uav].cls]
                                                     if e_uav in plan.plans else 100.0)),
    ]


def _describe(res: dyn.ReplanResult, plan_before, *, max_notes: int = 6) -> list[str]:
    lines = [f"changed={res.changed_uavs}  placeholder={res.placeholder_used}  "
             f"timing_ms={{{', '.join(f'{k}={v:.0f}' for k, v in res.timing_ms.items())}}}  "
             f"makespan {plan_before.makespan_s:.0f} -> {res.plan.makespan_s:.0f} s  uncovered={res.plan.uncovered}"]
    for n in res.notes[:max_notes]:
        lines.append(f"  note: {n}")
    if len(res.notes) > max_notes:
        lines.append(f"  (+{len(res.notes) - max_notes} notes)")
    for uid, p in sorted(res.plan.plans.items()):
        old = plan_before.plans.get(uid)
        same = old is p
        k = len(p.waypoints) - len(res.diff.get(uid, []))
        lines.append(f"  {uid}: {'unchanged (same object)' if same else 'CHANGED'} strips={[s for s, _ in p.strips]} "
                     f"n_wp={len(p.waypoints)} diff_from_wp={k if not same else '-'} t_land={p.t_land_s:.0f} "
                     f"E_margin={p.energy_margin_wh:.0f} landing={p.landing_site} warn={p.warnings[-2:]}")
    for uid in plan_before.plans:
        if uid not in res.plan.plans:
            lines.append(f"  {uid}: REMOVED from plan")
    return lines


def main() -> int:
    t_all = time.perf_counter()
    out_dir = os.path.join(FIG_DIR, "dynamic")
    os.makedirs(out_dir, exist_ok=True)
    inputs, cfg = build_mini_example()
    cfg = dataclasses.replace(cfg, dem_step_m=DEM_STEP_M)

    def _build():
        ctx = build_context(inputs, cfg, terrain_margin_m=TERRAIN_MARGIN_M)
        return ctx
    ctx0 = step("build_context", _build)
    plan0 = step("plan_mission", lambda: plan_mission(inputs, cfg, ctx=ctx0))
    if ctx0 is None or plan0 is None:
        return 1
    print(plan_summary(plan0, ctx0))
    step("viz.save_all(before)", lambda: viz.save_all(plan0, ctx0, None, os.path.join(out_dir, "before")))

    tel0 = telemetry_from_plan(plan0, T_TELEMETRY_S, ctx0)
    print(f"\ntelemetry @ t={T_TELEMETRY_S:.0f} s (interpolated from plan):")
    for uid, t in tel0.items():
        print(f"  {uid}: xy=({t.x_m:.0f},{t.y_m:.0f}) z={t.z_m:.1f} theta={math.degrees(t.theta_rad):.0f} deg "
              f"Va={t.v_air_mps} E_tel={t.e_tel_wh:.0f} strip={t.current_strip_id} progress={t.strip_progress:.2f}")

    results: dict[str, dict] = {"isolated": {}, "chain": {}}

    # ---- (1) 隔離模式 -------------------------------------------------------------
    print("\n########## (1) isolated: each event from the original plan at t=600 s ##########")
    for name, factory in _events(ctx0, plan0, T_TELEMETRY_S):
        def _run(name=name, factory=factory):
            ctx = build_context(inputs, cfg, terrain_margin_m=TERRAIN_MARGIN_M)   # 隔離：replan 會就地改 ctx/cache
            cache = dyn.cache_from_plan(plan0, ctx)
            tel = telemetry_from_plan(plan0, T_TELEMETRY_S, ctx)
            ev = factory(T_TELEMETRY_S)
            t0 = time.perf_counter()
            res = dyn.replan(plan0, ev, tel, ctx, cache, cfg, [])
            dt = (time.perf_counter() - t0) * 1e3
            print(f"replan wall {dt:.0f} ms")
            for ln in _describe(res, plan0):
                print(ln)
            results["isolated"][name] = {"changed": res.changed_uavs, "wall_ms": dt, "notes": res.notes,
                                         "makespan_s": res.plan.makespan_s, "uncovered": res.plan.uncovered,
                                         "n_strips": {u: len(p.strips) for u, p in res.plan.plans.items()}}
            return res
        step(f"isolated {name}", _run)

    # ---- (2) 串接模式 -------------------------------------------------------------
    print("\n########## (2) chained: res.plan feeds the next event (t = 600, 660, 720, 780, 840 s) ##########")
    ctx = step("rebuild ctx + cache", lambda: build_context(inputs, cfg, terrain_margin_m=TERRAIN_MARGIN_M))
    cache = dyn.cache_from_plan(plan0, ctx)
    plan = plan0
    ghosts: list = []
    t = T_TELEMETRY_S
    chain_rows = []
    t_sync_hold = [0.0]           # 最後一次重規劃之 t_now（sim.validate 之 t_sync_s）
    for i, (name, factory) in enumerate(_events(ctx, plan0, T_TELEMETRY_S, survivor_only=True)):
        def _run(i=i, name=name, factory=factory, t=t):
            nonlocal plan, ghosts
            tel = telemetry_from_plan(plan, t, ctx)
            ev = factory(t)
            t0 = time.perf_counter()
            res = dyn.replan(plan, ev, tel, ctx, cache, cfg, ghosts)
            dt = (time.perf_counter() - t0) * 1e3
            print(f"t={t:.0f} s, replan wall {dt:.0f} ms")
            for ln in _describe(res, plan):
                print(ln)
            chain_rows.append([i, name, f"{t:.0f}", ",".join(res.changed_uavs) or "-", f"{dt:.0f}",
                               f"{res.plan.makespan_s:.0f}", len(res.plan.uncovered), len(res.plan.strips)])
            results["chain"][f"{i}_{name}"] = {"t_s": t, "changed": res.changed_uavs, "wall_ms": dt, "notes": res.notes,
                                               "makespan_s": res.plan.makespan_s, "uncovered": res.plan.uncovered}
            p = os.path.join(out_dir, f"chain_{i}_{name}.png")
            viz.plot_2d(res.plan, ctx, p)
            plan, ghosts = res.plan, res.ghosts
            t_sync_hold[0] = float(getattr(res, "t_sync_s", 0.0))
            return res
        step(f"chain {i} {name}", _run)
        t += 60.0
    print("\n" + fmt_table(chain_rows, ["#", "event", "t_s", "changed", "wall_ms", "makespan_s", "uncovered", "strips"]))
    print("\nfinal chained plan:")
    print(plan_summary(plan, ctx))

    def _validate():
        vcache = sim.build_validation_cache(plan, ctx, budget_only=True)
        # 任務中途重規劃之計畫必須以事件時刻為 t_sync 驗證（規格 §13.3：σ_t = k_t (t − t_sync)），
        # 否則 σ_t 會被 k_t·t_now 灌大，validate 會回報重規劃器自己判定不存在的衝突（審查 R-12）。
        rep = sim.validate(plan, ctx, vcache, cfg, t_sync_s=t_sync_hold[0])
        print(f"validate(after): t_sync={t_sync_hold[0]:.0f} s passed={rep.passed} coverage_ok={rep.coverage_ok} "
              f"conflicts={len(rep.conflicts)} "
              f"energy_ok={rep.energy_ok} agl_ok={rep.agl_ok} airspace_ok={rep.airspace_ok}")
        for c in rep.coverage:
            print(f"  coverage {c.area_id}: " + ", ".join(f"{k}={v:.3f}" for k, v in sorted(c.coverage_by_cls.items())))
        return rep
    vrep = step("sim.validate(after)", _validate)
    step("viz.save_all(after)", lambda: viz.save_all(plan, ctx, vrep, os.path.join(out_dir, "after")))

    with open(os.path.join(FIG_DIR, "dynamic_events.json"), "w", encoding="utf-8") as f:
        json.dump(results, f, indent=1, ensure_ascii=False,
                  default=lambda x: None if isinstance(x, float) and not math.isfinite(x) else str(x))
    print(f"\nTOTAL {time.perf_counter() - t_all:.1f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
