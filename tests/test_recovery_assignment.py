"""多場坪的自動指派與回歸守衛（`mission.swarm_coordinator`）。

兩個受測物：

* :func:`propose_recovery_assignment` —— 由基準計畫推出「誰去哪個場坪」。
* :func:`recovery_regression_guard` —— 不准拿間距或能量餘裕換時間。

⚠⚠⚠ 守衛不是可選的。生產路徑實測：自動指派在 **2 個場坪**下給出 makespan
922.3 s（−36.4%），但最小水平間距從基準的 148.4 m 掉到 **51.5 m**；同一份幾何在
**3 個場坪**下給出 855.3 s（−41.0%）而間距 **188.7 m（比基準還好）**。
兩者的 makespan 只差 67 s ⇒ 沒有守衛的話演算法會停在那個 51.5 m 的解上。
"""

from __future__ import annotations

from dataclasses import dataclass

import pytest

from mission.swarm_coordinator import (propose_recovery_assignment,
                                       recovery_regression_guard)


# ── 最小替身：只需要 propose 用到的那幾個欄位 ────────────────────────────

@dataclass
class _P:
    t_takeoff_s: float
    t_land_s: float
    landing_site: str = "H"


@dataclass
class _Plan:
    plans: dict


class _Cfg:
    t_sep_ldg_s = 90.0


def _fleet(**durations):
    """{uav: 架次時長} → 計畫替身（起飛時刻用實測 HV4SITL13 的錯開值不影響結果）。"""
    return _Plan({u: _P(0.0, d) for u, d in durations.items()})


# ── 指派：訊號必須是**架次時長**，不是去衝突後的降落窗 ────────────────────

def test_single_site_proposes_nothing():
    """只有一個場坪時不可以有任何指派 ⇒ 逐位元等於今天。"""
    plan = _fleet(A1=534.8, B2=521.3, B3=533.9, B4=534.2)
    assert propose_recovery_assignment(plan, _Cfg(), ["H"]) == {}
    assert propose_recovery_assignment(plan, _Cfg(), []) == {}
    assert propose_recovery_assignment(plan, _Cfg(), None) == {}


def test_two_sites_split_the_fleet_in_half():
    """n 台 k 坪 ⇒ 每坪 ⌈n/k⌉ 台。這是 makespan 閉式的直接推論。"""
    plan = _fleet(A1=534.8, B2=521.3, B3=533.9, B4=534.2)
    got = propose_recovery_assignment(plan, _Cfg(), ["H", "ALT2"])
    # 四台分兩坪 ⇒ 兩台留 H、兩台改 ALT2 ⇒ 回傳兩筆（只含要改的）
    assert len(got) == 2, got
    assert set(got.values()) == {"ALT2"}


def test_three_sites_spread_further():
    plan = _fleet(A1=534.8, B2=521.3, B3=533.9, B4=534.2)
    got = propose_recovery_assignment(plan, _Cfg(), ["H", "ALT1", "ALT2"])
    assert set(got.values()) <= {"ALT1", "ALT2"}
    assert len(got) == 2, got            # 4 台 3 坪 ⇒ H 兩台、其餘各一

def test_longest_sortie_stays_on_the_first_site():
    """⭐ 依架次時長排序輪流分配 ⇒ 最長的那台落在第一個坪（通常是 home）。

    ⚠ 這條同時是反不可能觸發：把時長順序倒過來，指派必須跟著變。
    """
    a = propose_recovery_assignment(_fleet(X=900.0, Y=100.0), _Cfg(), ["H", "ALT2"])
    b = propose_recovery_assignment(_fleet(X=100.0, Y=900.0), _Cfg(), ["H", "ALT2"])
    assert a == {"Y": "ALT2"}, a
    assert b == {"X": "ALT2"}, b
    assert a != b, "把時長順序倒過來卻得到同樣的指派 ⇒ 這個函式沒有在讀時長"


def test_aircraft_already_on_the_right_site_is_not_listed():
    """只回傳**要改的**機；已經在對的坪上就不該出現在字典裡。"""
    plan = _Plan({"A1": _P(0.0, 900.0, "H"), "B2": _P(0.0, 100.0, "ALT2")})
    assert propose_recovery_assignment(plan, _Cfg(), ["H", "ALT2"]) == {}


def test_single_aircraft_proposes_nothing():
    assert propose_recovery_assignment(_fleet(A1=534.8), _Cfg(), ["H", "ALT2"]) == {}


# ── 守衛：三個實測案例 ──────────────────────────────────────────────────

_BASE = dict(passed=True, conflicts=0, uncovered=0, min_sep_h_m=148.4,
             energy_wh={"A1": 1177.0, "B2": 321.3, "B3": 320.5, "B4": 320.4}, n_cross_flown=0)


def test_guard_blocks_the_fast_but_unsafe_candidate():
    """⭐ 實測 2 坪那組：makespan −36.4% 但間距 148.4 → 51.5 m。必須擋。"""
    cand = dict(_BASE, min_sep_h_m=51.5)
    ok, why = recovery_regression_guard(_BASE, cand)
    assert not ok
    assert "間距" in why, why


def test_guard_accepts_the_faster_and_safer_candidate():
    """⭐ 實測 3 坪那組：makespan −41.0% 且間距 148.4 → 188.7 m。必須放行。

    ⚠ 少了這條反向臂，守衛可能是「一律擋下」的壞掉版本而測試照樣綠。
    """
    cand = dict(_BASE, min_sep_h_m=188.7,
                energy_wh={"A1": 1177.0, "B2": 321.3, "B3": 319.3, "B4": 317.1})
    ok, why = recovery_regression_guard(_BASE, cand)
    assert ok, why


def test_guard_blocks_energy_regression():
    """`energy_safe_check` 只保證 ≥ 保留量，所以換站可以把餘裕吃到剩 5 Wh 仍全綠。"""
    cand = dict(_BASE, min_sep_h_m=200.0,
                energy_wh={"A1": 1177.0, "B2": 321.3, "B3": 5.0, "B4": 320.4})
    ok, why = recovery_regression_guard(_BASE, cand)
    assert not ok
    assert "餘電" in why and "B3" in why, why


def test_guard_blocks_failed_validation_and_new_uncovered():
    ok, why = recovery_regression_guard(_BASE, dict(_BASE, passed=False, conflicts=1))
    assert not ok and "validate" in why
    ok, why = recovery_regression_guard(_BASE, dict(_BASE, uncovered=2))
    assert not ok and "未覆蓋" in why


def test_guard_tolerances_are_actually_applied():
    """容差要真的生效：間距 1 m 內、能量 5% 內的噪音不該被當成回歸。"""
    ok, _w = recovery_regression_guard(_BASE, dict(_BASE, min_sep_h_m=148.0))
    assert ok, "0.4 m 的浮點噪音不該擋"
    ok, _w = recovery_regression_guard(
        _BASE, dict(_BASE, energy_wh=dict(_BASE["energy_wh"], B2=321.3 * 0.97)))
    assert ok, "3% 的能量變化在 5% 容差內"
    ok, why = recovery_regression_guard(
        _BASE, dict(_BASE, energy_wh=dict(_BASE["energy_wh"], B2=321.3 * 0.90)))
    assert not ok, "10% 的能量回歸必須擋"


def test_guard_rejects_a_candidate_missing_an_aircraft():
    """候選裡少了一台的能量讀數 ⇒ 不可以當成「沒有回歸」放行。"""
    cand = dict(_BASE, energy_wh={"A1": 1177.0})
    ok, why = recovery_regression_guard(_BASE, cand)
    assert not ok and "沒有能量讀數" in why, why


@pytest.mark.parametrize("sep", [float("inf"), float("nan")])
def test_guard_survives_degenerate_baseline_separation(sep):
    """基準只有一台在空中時 min_sep 是 inf/nan ⇒ 不可以炸掉，也不可以誤擋。"""
    base = dict(_BASE, min_sep_h_m=sep)
    ok, _why = recovery_regression_guard(base, dict(_BASE, min_sep_h_m=200.0))
    assert ok


# ── 守衛：交叉航帶（建圖模式；2026-10-15 審查 L-e）────────────────────────

def test_guard_blocks_a_candidate_that_flies_fewer_cross_strips():
    """⭐ 少飛交叉帶的候選 makespan 自然較短 ⇒ 沒有這條會被「makespan 有變好」主動選中。

    候選其餘每一項都更好（間距 188.7 m、能量不變），只有交叉帶 4 → 3 ⇒ 仍必須擋。"""
    base = dict(_BASE, n_cross_flown=4)
    better_but_fewer = dict(_BASE, min_sep_h_m=188.7, n_cross_flown=3)
    ok, why = recovery_regression_guard(base, better_but_fewer)
    assert not ok and "交叉航帶變少" in why and "4 → 3" in why, why
    # 反向臂：一樣多或更多 ⇒ 放行（不是「有交叉帶就一律擋」）
    for n in (4, 5):
        ok, why = recovery_regression_guard(base, dict(better_but_fewer, n_cross_flown=n))
        assert ok, (n, why)
    ok, why = recovery_regression_guard(dict(_BASE, n_cross_flown=0), dict(_BASE, n_cross_flown=0))
    assert ok, why


@pytest.mark.parametrize("side", ["base", "cand"])
def test_guard_refuses_when_the_cross_strip_count_is_missing(side):
    """缺讀數 ⇒ 無從比較 ⇒ 擋（不默默當 0：當 0 的話基準缺讀數時任何候選都「沒變少」）。"""
    base, cand = dict(_BASE, n_cross_flown=4), dict(_BASE, n_cross_flown=4)
    (base if side == "base" else cand).pop("n_cross_flown")
    ok, why = recovery_regression_guard(base, cand)
    assert not ok and "n_cross_flown" in why, why


def test_measure_harvest_plan_counts_flown_cross_strips(monkeypatch):
    """量測字典的 ``n_cross_flown`` ＝ 各機 ``UAVPlan.strips``（(strip_id, 方向)）裡的交叉帶條數，含殘段 ``#r``。"""
    import types

    import hv_dccpp
    from hv_dccpp.geometry import cross_strip_id
    from mission.swarm_coordinator import measure_harvest_plan

    v = types.SimpleNamespace(passed=True, conflicts=[])
    monkeypatch.setattr(hv_dccpp, "validate", lambda *a, **k: v)
    monkeypatch.setattr(hv_dccpp, "build_validation_cache", lambda *a, **k: None)
    monkeypatch.setattr(hv_dccpp.sim, "min_separation", lambda plans, cfg: (150.0, None))
    wp = [types.SimpleNamespace(energy_remaining_wh=300.0)]
    x1, x2 = cross_strip_id("Z1_1", "B", "B2", 1), cross_strip_id("Z1_1", "B", "B2", 2)
    plans = {
        "B2": types.SimpleNamespace(strips=[("Z1_1_B_k10", 1), (x1, 0), (x2 + "#r", 1)], waypoints=wp,
                                    t_land_s=900.0, landing_site="H"),
        "B3": types.SimpleNamespace(strips=[("Z1_2_B_k3", 0)], waypoints=wp, t_land_s=950.0, landing_site="ALT1"),
    }
    hv = types.SimpleNamespace(last_plan=types.SimpleNamespace(plans=plans, uncovered=[]), last_ctx=None, cfg=None)
    m = measure_harvest_plan(hv)
    assert m["n_cross_flown"] == 2 and m["makespan_s"] == 950.0 and m["landing"] == {"B2": "H", "B3": "ALT1"}
    plans["B2"].strips = [("Z1_1_B_k10", 1), (x1, 0)]                    # 少一條 ⇒ 量測跟著變、守衛擋下
    m2 = measure_harvest_plan(hv)
    assert m2["n_cross_flown"] == 1
    ok, why = recovery_regression_guard(m, m2)
    assert not ok and "交叉航帶變少" in why, why
