"""`recovery_sites` 從 AeroPlan 一路接到 `UAVInstance.recovery_site_id` 的接線測試。

⚠⚠⚠ 為什麼需要：這條路上最容易出的錯**不是**算錯，而是「參數有送到、log 有印，
產物卻沒變」。上游的老症狀正是如此 —— 宣告 8 個備降點、參數一路傳進去，
四機照樣 100% 落 H。所以這裡每一條都斷言**產物**（`UAVInstance.recovery_site_id`
或 `UAVPlan.landing_site`），不斷言「有沒有呼叫」。

接線鏈（2026-09-09，vendored 修改 #4）::

    SwarmCoordinator.hv_recovery_sites
      → HvDccppCoordinator(recovery_sites=…)
      → to_mission_inputs(recovery_sites=…)
      → UAVInstance.recovery_site_id
      → context.recovery_sites / recovery_site_id_of
      → planner._site_order(preferred=…) → UAVPlan.landing_site
"""

from __future__ import annotations

import pytest

pytest.importorskip("shapely")

from hv_dccpp.integration.aeroplan_adapter import (AeroPlanAdapterError,      # noqa: E402
                                                   to_mission_inputs)

HOME = (23.95, 120.90)
ALT = (23.9554, 120.90)                       # HOME 正北約 600 m
AREAS = [{"area_id": 1, "priority": 1.0,
          "polygon": [(23.9470, 120.8930), (23.9470, 120.9070),
                      (23.9530, 120.9070), (23.9530, 120.8930)]}]
DRONES = [{"drone_id": 1, "name": "A1", "hv_class": "A", "position": HOME,
           "speed": 21.5, "turn_radius": 200.0, "vehicle_type": "vtol"},
          {"drone_id": 2, "name": "B2", "hv_class": "B", "position": HOME,
           "speed": 19.0, "turn_radius": 120.0, "vehicle_type": "vtol"}]


def _inputs(recovery_sites=None, warnings=None):
    return to_mission_inputs(
        AREAS, DRONES, coverage_width_m=150.0, overlap_rate=0.30,
        coverage_altitude=120.0, home=HOME, alternates=[ALT],
        hfov_deg=60.0, vfov_deg=45.0, recovery_sites=recovery_sites,
        warnings=warnings)


def _by_uid(inputs):
    return {u.uav_id: u.recovery_site_id for u in inputs.fleet}


# ── 前置斷言：不給時必須全是 None（＝走全域回收點）────────────────────────

def test_unset_leaves_every_aircraft_on_the_global_recovery_site():
    ins, id_map = _inputs()
    assert _by_uid(ins) == {"A1": None, "B2": None}
    assert set(id_map.values()) == {"A1", "B2"}          # 前置：uav_id 真的是這兩個


# ── 產物斷言：給了要真的落到 UAVInstance 上 ─────────────────────────────

def test_uav_id_key_reaches_the_fleet():
    ins, _m = _inputs({"B2": "ALT1"})
    assert _by_uid(ins) == {"A1": None, "B2": "ALT1"}


def test_drone_id_key_also_works():
    """面板那一側習慣用 `drone_id`（整數），兩種鍵都要收。"""
    ins, _m = _inputs({2: "ALT1"})
    assert _by_uid(ins) == {"A1": None, "B2": "ALT1"}
    ins2, _m2 = _inputs({"2": "ALT1"})                   # 字串形式的 drone_id
    assert _by_uid(ins2) == {"A1": None, "B2": "ALT1"}


def test_home_is_a_valid_target():
    ins, _m = _inputs({"A1": "H", "B2": "ALT1"})
    assert _by_uid(ins) == {"A1": "H", "B2": "ALT1"}


def test_a_note_is_emitted_so_the_operator_can_see_it():
    """⚠ 靜靜生效與靜靜失效一樣糟：換了落點必須留下可讀回的痕跡。"""
    warns: list[str] = []
    _inputs({"B2": "ALT1"}, warnings=warns)
    assert any("per_uav_recovery_sites" in w and "B2@ALT1" in w for w in warns), warns


# ── 大聲拒絕：認不得的鍵不可以默默忽略 ──────────────────────────────────

def test_unknown_aircraft_key_is_rejected_loudly():
    """⚠⚠ 默默忽略會讓「指派了但沒生效」看起來跟「沒指派」一模一樣。"""
    with pytest.raises(AeroPlanAdapterError, match="不是機隊裡的載具"):
        _inputs({"C9": "ALT1"})
    with pytest.raises(AeroPlanAdapterError, match="不是機隊裡的載具"):
        _inputs({99: "ALT1"})


def test_unknown_site_is_rejected_loudly():
    with pytest.raises(AeroPlanAdapterError, match="不是已知站點"):
        _inputs({"B2": "NOWHERE"})


# ── 對照組必須乾淨：只宣告場坪、不指派 ⇒ 機隊逐位元不變 ──────────────────

def test_declaring_an_alternate_alone_changes_nothing_on_the_fleet():
    """⚠ 沒有這條，任何 A/B 對照都可能被「多宣告一個站」本身污染。"""
    a, _m = to_mission_inputs(AREAS, DRONES, coverage_width_m=150.0, overlap_rate=0.30,
                              coverage_altitude=120.0, home=HOME, alternates=None,
                              hfov_deg=60.0, vfov_deg=45.0)
    b, _m2 = _inputs()
    assert _by_uid(a) == _by_uid(b) == {"A1": None, "B2": None}


# ── SwarmCoordinator 這一端真的有那個屬性且預設是 None ────────────────────

def test_swarm_coordinator_exposes_the_knob_defaulting_to_none():
    from mission.swarm_coordinator import SwarmCoordinator
    c = SwarmCoordinator()
    assert hasattr(c, "hv_recovery_sites")
    assert c.hv_recovery_sites is None, "預設必須是 None ⇒ 逐位元等於改動前"
