"""quickstart.py — README 快速上手：任務 JSON → 規劃 → 驗證 → ArduPilot 檔案。

    python examples/quickstart.py [mission.json] [out_dir]

輸入預設 `examples/mission_min.json`（平地、1 架 Pioneer PNP + 1 架 HERO PNP、3 km × 1.2 km 區域），
輸出預設 `figures/ardupilot/quickstart/`（*.waypoints / *_rally.txt / *_fence.txt / *.param / *.plan）。
"""
from __future__ import annotations

import dataclasses
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import hv_dccpp as hv  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    mission = argv[0] if argv else os.path.join(ROOT, "examples", "mission_min.json")
    out_dir = argv[1] if len(argv) > 1 else os.path.join(ROOT, "figures", "ardupilot", "quickstart")

    inputs = hv.load_mission(mission)                               # 表 2 任務輸入（JSON / dict / 路徑）
    cfg = dataclasses.replace(hv.PlannerConfig(), dem_step_m=50.0)  # 地形網格 50 m（平地夠用；預設 10 m）
    ctx = hv.build_context(inputs, cfg)                             # ENU 框架／地形／風場／能量模型／感測器

    rep = hv.preflight(inputs, cfg, ctx)                            # 起飛前檢查（GO / GO_WITH_WARNINGS / NO_GO）
    print("preflight:", rep.status, "errors:", rep.errors[:3], "warnings:", len(rep.warnings))
    if not rep.ok:
        return 1

    plan = hv.plan_mission(inputs, cfg, ctx=ctx)                    # 分解 → 配置 → Beam-IDP → 高度剖面 → 去衝突
    print(f"makespan = {plan.makespan_s / 60:.1f} min, cells={len(plan.cells)}, strips={len(plan.strips)}")
    for uid, p in plan.plans.items():
        print(f"  {uid}({p.cls}) 條帶 {len(p.strips)} 距離 {p.distance_m / 1000:.1f} km "
              f"落地 {p.t_land_s:.0f} s 能量餘裕 {p.energy_margin_wh:.0f} Wh")

    cache = hv.build_validation_cache(plan, ctx, budget_only=True)  # 驗證用能量預算表
    vrep = hv.validate(plan, ctx, cache, cfg)                       # 覆蓋／能量／坡度／AGL／空域／4D 分離
    print("validate:", vrep.passed, "coverage:", {c.area_id: round(c.coverage, 3) for c in vrep.coverage},
          "conflicts:", len(vrep.conflicts))

    summary = hv.export_all(plan, ctx, cfg, out_dir)                # QGC WPL + rally + fence + .param + .plan
    for uid, s in summary["uavs"].items():
        print(f"  {uid}: {s['n_items']} 個任務項目 → {s['files']['waypoints']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
