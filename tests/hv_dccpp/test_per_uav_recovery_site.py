"""vendored 修改 #4：逐機回收站 `UAVInstance.recovery_site_id`。

⚠⚠⚠ 為什麼需要：HV4SITL9 的 makespan 有 **70.3%** 是起飛錯開，而驅動它的 6 筆
衝突**全部**是同一個場坪 H 的**降落**走廊排隊（d_h = d_v = 0.0），沒有一筆是空間
衝突。而上游只有全域 `recovery_site_id`，加上 `planner._site_order` 把它無條件排
第一、`append_rtb` 取第一個能量可行者 ⇒ **給 8 個備降點，四機仍 100% 落 H**。

⚠ 這個欄位只提供**表達能力**，不含指派規則。自動指派與它的安全守衛是另一件事。

本檔遵守專案規矩：每條「設了會怎樣」都配一條「沒設要逐位元不變」，而且每條都
先斷言「這個設定真的讓 X 發生」，避免測到一個不可能觸發的儀表。
"""

from __future__ import annotations

import dataclasses

import pytest

pytest.importorskip("shapely")
pytest.importorskip("scipy")

import hv_dccpp.planner as P                                    # noqa: E402
from hv_dccpp.context import build_context                      # noqa: E402
from hv_dccpp.mission import MissionInputError                  # noqa: E402
from hv_dccpp.scenario import build_mini_example                # noqa: E402


def _plan(mapping):
    ins, cfg = build_mini_example()
    fleet = tuple(dataclasses.replace(u, recovery_site_id=mapping.get(u.uav_id))
                  for u in ins.fleet)
    ins = dataclasses.replace(ins, fleet=fleet)
    return P.plan_mission(ins, cfg), ins, cfg


def _shape(plan):
    """拿來比「逐位元相同」的可比較快照。"""
    return {u: (p.landing_site, round(float(p.t_land_s), 6), round(float(p.distance_m), 6),
                tuple(p.strips)) for u, p in sorted(plan.plans.items())}


# ── 退回保證：沒設就要逐位元等於上游 ──────────────────────────────────────

def test_unset_is_bit_identical_and_deterministic():
    a, _i, _c = _plan({})
    b, _i2, _c2 = _plan({})
    assert _shape(a) == _shape(b), "同一份輸入跑兩次就不一樣 ⇒ 後面每條比較都不可信"
    assert all(p.landing_site == a.plans[u].landing_site for u, p in b.plans.items())
    assert not a.uncovered


def test_unset_lands_everyone_at_the_global_recovery_site():
    """前置斷言：預設行為就是全員落全域 recovery。少了它，下面那條可能本來就成立。"""
    plan, ins, cfg = _plan({})
    rec = ins.recovery_site.site_id
    assert {p.landing_site for p in plan.plans.values()} == {rec}


# ── 表達能力：設了要真的改變產物 ────────────────────────────────────────

def test_assignment_actually_changes_the_landing_site():
    """⭐ 這是本功能唯一該證明的事：`landing_site` 真的變了。

    ⚠ 不可以只斷言「參數有傳進去」—— 上游的老問題正是「宣告了 8 個備降點、
    參數有送到、log 有印，而四機照樣全落 H」。
    """
    base, _i, _c = _plan({})
    plan, ins, _cfg = _plan({"B1": "ALT1"})

    assert base.plans["B1"].landing_site == ins.recovery_site.site_id   # 前置斷言
    assert plan.plans["B1"].landing_site == "ALT1"
    assert plan.plans["A1"].landing_site == base.plans["A1"].landing_site   # 別人不受影響
    assert not plan.uncovered


def test_relocated_aircraft_gets_its_rtb_recomputed():
    """換站必須重算返航段，不是只改一個字串欄位。"""
    base, _i, _c = _plan({})
    plan, _i2, _c2 = _plan({"B1": "ALT1"})
    assert plan.plans["B1"].distance_m != pytest.approx(base.plans["B1"].distance_m, abs=1.0)
    assert plan.plans["A1"].distance_m == pytest.approx(base.plans["A1"].distance_m, abs=1e-6)


def test_context_exposes_the_per_uav_lookup():
    ins, cfg = build_mini_example()
    fleet = tuple(dataclasses.replace(u, recovery_site_id=("ALT1" if u.uav_id == "B1" else None))
                  for u in ins.fleet)
    ctx = build_context(dataclasses.replace(ins, fleet=fleet), cfg)
    assert ctx.recovery_site_id_of("B1") == "ALT1"
    assert ctx.recovery_site_id_of("A1") == ctx.recovery_site_id
    assert any("per_uav_recovery_sites" in w for w in ctx.warnings), ctx.warnings


# ── 不可把任務打死：偏好站吃不下時要退回 ────────────────────────────────

def test_site_order_puts_preference_first_without_dropping_others():
    """`_site_order` 的單元行為：偏好在前、其餘一個都不能少。"""
    import numpy as np

    class _Cache:
        rtb = {("A", "H"): np.zeros((1, 2)), ("A", "ALT1"): np.ones((1, 2)),
               ("A", "ALT2"): np.full((1, 2), 2.0)}

    base = P._site_order(_Cache(), "A", 0, 0, "H")
    assert base[0] == "H" and set(base) == {"H", "ALT1", "ALT2"}

    pref = P._site_order(_Cache(), "A", 0, 0, "H", preferred="ALT2")
    assert pref[0] == "ALT2", pref
    assert set(pref) == set(base), "偏好站不可以把其他候選擠掉 —— 那會讓能量不可行時無路可退"

    # 反向臂：不存在的偏好站要被忽略而不是炸掉或清空候選
    assert P._site_order(_Cache(), "A", 0, 0, "H", preferred="NOPE") == base


# ── 輸入驗證 ────────────────────────────────────────────────────────────

def test_unknown_recovery_site_is_rejected_loudly():
    ins, _cfg = build_mini_example()
    fleet = tuple(dataclasses.replace(u, recovery_site_id=("NOPE" if u.uav_id == "B1" else None))
                  for u in ins.fleet)
    with pytest.raises(MissionInputError, match="recovery_site_id"):
        dataclasses.replace(ins, fleet=fleet)
