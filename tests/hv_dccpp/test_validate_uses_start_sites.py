"""`validate()` 必須用**與 planner 同一組**起飛站登記走廊。

⚠⚠⚠ 為什麼有這個檔（2026-09-09）：`hv_dccpp/sim.py` 的 `validate()` 原本寫

    home = ctx.inputs.home.site_id
    for p in plan.plans.values():
        sched.reserve_plan(p, home)

**每一架都登記在 home**，完全不看 `UAVInstance.start_site_id`。而 planner 早就
逐機登記（`planner.py:1550-1552` 用 `_start_site`）⇒ 兩條管線用不同的站點。
只要有人從別的場坪起飛，`validate` 就會報**不存在的**走廊衝突。

實測：B4 起飛站 ALT4、`t_takeoff=588.0`，validate 回報一筆
``site=H, res_kinds=('landing','takeoff'), t=588.0``；四組接了逐機起飛站的組態
**全部** `validate_failed(conflicts=2)`，而基準組全部 passed。

⚠⚠ 而且這是「承諾了但不存在的守衛」：planner 側有
`test_per_uav_start_site_used_for_takeoff_corridor` 鎖住「起飛走廊真的用該站」，
但**沒有任何測試**斷言 `validate()` 也用同一組站點 ——
`grep -rln start_site tests/` 當時完全不列出 `test_sim.py`。
"""

from __future__ import annotations

import dataclasses

import pytest

pytest.importorskip("shapely")
pytest.importorskip("scipy")

import hv_dccpp.planner as P                                    # noqa: E402
from hv_dccpp import build_validation_cache, validate           # noqa: E402
from hv_dccpp.context import build_context                      # noqa: E402
from hv_dccpp.deconflict import CorridorSchedule                # noqa: E402
from hv_dccpp.scenario import build_mini_example                # noqa: E402


def _plan_with_start_sites(mapping):
    """把 `mapping`（{uav_id: site_id}）套上去規劃；回傳 (plan, inputs, cfg)。"""
    inputs, cfg = build_mini_example()
    fleet = tuple(dataclasses.replace(u, start_site_id=mapping.get(u.uav_id))
                  for u in inputs.fleet)
    ins = dataclasses.replace(inputs, fleet=fleet)
    return P.plan_mission(ins, cfg), ins, cfg


def test_baseline_all_at_home_still_validates():
    """前置斷言：不設起飛站時本來就該過。少了它，下面那條可能是因為情境本來就壞。"""
    plan, ins, cfg = _plan_with_start_sites({})
    v = validate(plan, build_context(ins, cfg), build_validation_cache(plan, build_context(ins, cfg),
                                                                       budget_only=False), cfg)
    assert v.passed, [f"{c.kind} {c.uav_a}-{c.uav_b} t={c.t_s:.0f}" for c in v.conflicts]


def test_per_uav_start_site_does_not_create_phantom_conflicts():
    """⭐ 有人從別的場坪起飛時，validate 不可以報出不存在的走廊衝突。

    ⚠ 修好之前這條會紅：走廊衝突的 `site` 恆為 H，而其中一方的起飛站根本不是 H。
    """
    plan, ins, cfg = _plan_with_start_sites({"B1": "ALT1"})
    ctx = build_context(ins, cfg)

    # 前置斷言：這個設定**真的**讓起飛站分開了，否則下面測的是空的
    assert ctx.start_site_id("B1") == "ALT1"
    assert ctx.start_site_id("A1") == ins.home.site_id

    v = validate(plan, ctx, build_validation_cache(plan, ctx, budget_only=False), cfg)
    bogus = [c for c in v.conflicts if c.kind == "corridor"]
    assert not bogus, [f"{c.kind} site={getattr(c, 'site_id', None)} "
                       f"{c.uav_a}-{c.uav_b} t={c.t_s:.0f} kinds={getattr(c, 'res_kinds', None)}"
                       for c in bogus]
    assert v.passed


def test_validate_reserves_at_the_same_site_the_planner_used():
    """值層級：validate 建的走廊預約，站點必須逐機等於 `planner._start_site`。

    ⚠ 這條直接鎖「兩條管線用同一組站點」，而不是鎖「那一行長什麼樣」
    —— 換寫法、換存取器都免疫，只要值一致就綠。
    """
    plan, ins, cfg = _plan_with_start_sites({"B1": "ALT1"})
    ctx = build_context(ins, cfg)

    sched = CorridorSchedule(cfg)
    for p in plan.plans.values():
        sched.reserve_plan(p, P._start_site(ctx, p.uav_id))
    expect = {(r.uav_id, r.kind): r.site_id for r in sched.reservations if r.kind == "takeoff"}

    # validate 自己建的那一份：用同一組計畫重跑，比對起飛預約的站點
    got = {}
    real = CorridorSchedule.reserve_plan

    def spy(self, plan_, site_id):
        got[(plan_.uav_id, "takeoff")] = site_id
        return real(self, plan_, site_id)

    CorridorSchedule.reserve_plan = spy                          # type: ignore[method-assign]
    try:
        validate(plan, ctx, build_validation_cache(plan, ctx, budget_only=False), cfg)
    finally:
        CorridorSchedule.reserve_plan = real                     # type: ignore[method-assign]

    assert got, "validate 沒有建任何走廊預約 —— 這條測試量不到東西"
    assert got == expect, (got, expect)
    assert got[("B1", "takeoff")] == "ALT1"                      # 反不可能觸發：真的分開了
