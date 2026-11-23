"""run_paper_table2.py — 論文 Table 2 情境之 legacy 退化重現（Wang et al. 2025）。

    python examples/run_paper_table2.py

build_paper_table2_scenario（3 機 U1–U3、V=230 m/s、R=4000 m、h=1200 m、平地）→ plan_mission(legacy_mode) →
每機路徑長，與論文比較：協同 620.9 km（總路徑）／非協同 648.13、606.06、591.27 km；另呼叫 sim.paper_table2_check。
輸出：figures/paper_table2/plan_2d.png 等、figures/paper_table2.json。
"""
from __future__ import annotations

import dataclasses
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from examples._compat import FIG_DIR, fmt_table, plan_summary, step  # noqa: E402

from hv_dccpp import sim, viz  # noqa: E402
from hv_dccpp.context import build_context  # noqa: E402
from hv_dccpp.planner import plan_mission  # noqa: E402
from hv_dccpp.preflight import preflight  # noqa: E402
from hv_dccpp.scenario import PAPER_TABLE2_UAV_START, build_paper_table2_scenario  # noqa: E402

PAPER_COORDINATED_TOTAL_KM = 620.9
PAPER_UNCOORDINATED_KM = {"U1": 648.13, "U2": 606.06, "U3": 591.27}
D_D_M = 200.0              # 70 km 區域：25 m 取樣過細（sim.paper_table2_check 同樣放寬）
TERRAIN_MARGIN_M = 5000.0


def main() -> int:
    t_all = time.perf_counter()
    out_dir = os.path.join(FIG_DIR, "paper_table2")
    os.makedirs(out_dir, exist_ok=True)
    inputs, cfg = build_paper_table2_scenario()
    cfg = dataclasses.replace(cfg, d_d_m=max(cfg.d_d_m, D_D_M))
    print(f"paper Table 2 scenario: legacy_mode={cfg.legacy_mode}, dem_step={cfg.dem_step_m}, d_d={cfg.d_d_m}, "
          f"transfer_types={cfg.transfer_types}, idp={cfg.idp_branching}, K={cfg.K}, B={cfg.B}, metric={cfg.metric}")
    print(f"  UAV starts (paper): {PAPER_TABLE2_UAV_START}")
    spec = inputs.fleet[0].spec
    print(f"  spec: V={spec.cruise_airspeed_mps} m/s, R_plan={spec.turn_radius_planning_m} m, sensors={inputs.fleet[0].sensors}")

    ctx = step("build_context", lambda: build_context(inputs, cfg, terrain_margin_m=TERRAIN_MARGIN_M))
    if ctx is None:
        return 1
    area = ctx.areas_enu["Z1"]
    print(f"  Z1 area = {area.area / 1e6:.1f} km2 (paper 2604.34), bounds = {[round(b / 1000, 1) for b in area.bounds]} km")
    rep = step("preflight", lambda: preflight(inputs, cfg, ctx))
    if rep is not None:
        print(f"  preflight {rep.status}; errors={rep.errors}; warnings={rep.warnings[:5]}")
        for cls, d in rep.derived.items():
            print(f"  derived[{cls}]: R_eff={d.get('R_eff')}, h_op={d.get('h_op')}, w={d.get('w')}, s={d.get('s')}, e_ext={d.get('e_ext')}")

    plan = step("plan_mission (legacy)", lambda: plan_mission(inputs, cfg, ctx=ctx))
    if plan is None:
        return 1
    print(plan_summary(plan, ctx))
    for c in plan.cells:
        print(f"  cell {c.cell_id}: n_strips={c.n_strips}, alpha={ {k: round(math.degrees(v), 1) for k, v in c.alpha_rad.items()} }, "
              f"h_op={c.h_op_m}")

    rows = []
    total = 0.0
    for uid in sorted(plan.plans):
        p = plan.plans[uid]
        km = p.distance_m / 1000.0
        total += km
        oper = sum(math.hypot(b.x_m - a.x_m, b.y_m - a.y_m) for a, b in zip(p.waypoints[:-1], p.waypoints[1:])
                   if a.sensor_on) / 1000.0
        ref = PAPER_UNCOORDINATED_KM.get(uid, math.nan)
        rows.append([uid, len(p.strips), f"{km:.1f}", f"{oper:.1f}", f"{km - oper:.1f}", f"{ref:.2f}",
                     f"{100 * (km - ref) / ref:+.1f} %", ctx.start_site_id(uid), f"{p.t_takeoff_s:.0f}",
                     f"{p.t_land_s / 3600:.2f}"])
    print(fmt_table(rows, ["uav", "strips", "path_km", "oper_km", "ferry_km", "paper_uncoord_km", "diff",
                           "start_site", "t_to_s", "t_land_h"]))
    print(f"total path = {total:.1f} km  vs paper coordinated total {PAPER_COORDINATED_TOTAL_KM} km "
          f"({100 * (total - PAPER_COORDINATED_TOTAL_KM) / PAPER_COORDINATED_TOTAL_KM:+.1f} %); "
          f"paper uncoordinated sum = {sum(PAPER_UNCOORDINATED_KM.values()):.2f} km")
    print(f"  start sites (UAVInstance.start_site_id): {inputs.start_sites}; recovery = {ctx.recovery_site_id} (D40)")
    print("  note: each UAV now takes off from its own Table 2 ground station, but D40 recovers the whole fleet at "
          "home (U1), so ferry_km (takeoff/landing corridors + cross-field return) is included in path_km and the "
          "paper's per-UAV figures are not. Compare oper_km with the paper's numbers; reproducing the paper's "
          "per-UAV path length exactly would need per-UAV recovery sites as well.")

    chk = step("sim.paper_table2_check", lambda: sim.paper_table2_check(cfg, d_d_m=D_D_M, terrain_margin_m=TERRAIN_MARGIN_M))
    if chk:
        keys = ("total_path_km", "max_uav_path_km", "mean_uav_path_km", "makespan_s", "uncovered", "in_range_total",
                "in_range_max_uav", "expected_lo_km", "expected_hi_km", "compute_ms")
        print("  " + ", ".join(f"{k}={chk.get(k):.1f}" for k in keys if isinstance(chk.get(k), (int, float))))
        print("  per-UAV: " + ", ".join(f"{k}={v:.1f}" for k, v in chk.items() if k.startswith("path_km_")))

    paths = step("viz.save_all", lambda: viz.save_all(plan, ctx, None, out_dir))
    if paths:
        print("figures: " + ", ".join(os.path.relpath(p, FIG_DIR) for p in paths))

    with open(os.path.join(FIG_DIR, "paper_table2.json"), "w", encoding="utf-8") as f:
        json.dump({"per_uav_km": {u: plan.plans[u].distance_m / 1000 for u in plan.plans}, "total_km": total,
                   "paper_coordinated_total_km": PAPER_COORDINATED_TOTAL_KM, "paper_uncoordinated_km": PAPER_UNCOORDINATED_KM,
                   "makespan_s": plan.makespan_s, "check": chk or {}}, f, indent=1,
                  default=lambda x: None if isinstance(x, float) and not math.isfinite(x) else x)
    print(f"\nTOTAL {time.perf_counter() - t_all:.1f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
