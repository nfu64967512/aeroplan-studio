"""同時命中可行區間的幾何。

鎖住的是 Dubins「先轉後直」那段算術 —— 它決定了 t_min,而 t_min 決定了
「3 秒散布做不做得到」這個判定。算錯的話整個判定都是錯的,而且會很有說服力。
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts", "wsl"))
from impact_feasibility import (dubins_len_to_point, t_min_impact,  # noqa: E402
                                turn_radius)


# ── 轉彎半徑 ──────────────────────────────────────────────────────────
def test_turn_radius_matches_coordinated_turn():
    # 157 m/s、45° 傾斜:ρ = V²/(g·tan45) = V²/g
    assert turn_radius(157.0, 45.0) == pytest.approx(157.0 ** 2 / 9.80665, rel=1e-6)


def test_steeper_bank_tightens_turn():
    assert turn_radius(157.0, 60.0) < turn_radius(157.0, 45.0) < turn_radius(157.0, 30.0)


def test_zero_bank_cannot_turn():
    assert math.isinf(turn_radius(157.0, 0.0))


# ── Dubins ────────────────────────────────────────────────────────────
def test_target_dead_ahead_is_straight_line():
    """正前方的目標不該需要轉彎 —— 切點分支取錯的話這裡會變成轉快 180°。"""
    L, dpsi = dubins_len_to_point(0.0, 10000.0, 0.0, 2500.0)   # 航向北,目標正北
    assert L == pytest.approx(10000.0, rel=1e-9)
    assert dpsi == pytest.approx(0.0, abs=1e-9)


def test_path_never_shorter_than_straight_line():
    rho = 2500.0
    for brg in [0.0, 0.3, 1.0, 2.0, 3.0, 4.5, 6.0]:
        d = 30000.0
        dx, dy = d * math.sin(brg), d * math.cos(brg)
        L, _ = dubins_len_to_point(dx, dy, 0.0, rho)
        assert L >= d - 1e-6


def test_heading_error_costs_distance():
    """航向偏差越大,路徑越長 —— 這是「早到的機體吸收不了延遲」的物理根據。"""
    rho, d = 2500.0, 30000.0
    prev = None
    for brg in [0.0, 0.5, 1.0, 1.5]:
        L, _ = dubins_len_to_point(d * math.sin(brg), d * math.cos(brg), 0.0, rho)
        if prev is not None:
            assert L > prev
        prev = L


def test_target_inside_turning_circle_is_infeasible():
    """目標落在最小轉彎圓內 = 切不進去。必須明確回不可行,不能給一個大數字。"""
    rho = 2500.0
    L, dpsi = dubins_len_to_point(0.0, 1000.0, math.radians(90.0), rho)
    assert math.isinf(L)
    assert math.isnan(dpsi)


def test_symmetric_left_right():
    rho, d = 2500.0, 30000.0
    Ll, _ = dubins_len_to_point(d * math.sin(0.8), d * math.cos(0.8), 0.0, rho)
    Lr, _ = dubins_len_to_point(d * math.sin(-0.8), d * math.cos(-0.8), 0.0, rho)
    assert Ll == pytest.approx(Lr, rel=1e-9)


# ── t_min ─────────────────────────────────────────────────────────────
def test_t_min_straight_in_matches_hand_calculation():
    """正對目標、30 km、3100 m 高、45° 俯衝、157 m/s。

    環半徑 = 3100/tan45 = 3100 m;水平段 = 30000 − 3100 = 26900 m
    俯衝段 = 3100/sin45 = 4384 m;t = (26900 + 4384)/157 = 199.3 s
    """
    t, info = t_min_impact(30000.0, 0.0, 0.0, 157.0, 3100.0, 45.0, 45.0)
    assert info["r_ring"] == pytest.approx(3100.0, rel=1e-9)
    assert t == pytest.approx((26900.0 + 3100.0 / math.sin(math.radians(45))) / 157.0,
                              rel=1e-6)


def test_shallower_dive_pushes_ring_further_out():
    """俯衝角越淺,環半徑越大 —— 27° 的環比 45° 遠將近一倍。"""
    _, i45 = t_min_impact(30000.0, 0.0, 0.0, 157.0, 3100.0, 45.0, 45.0)
    _, i27 = t_min_impact(30000.0, 0.0, 0.0, 157.0, 3100.0, 27.0, 45.0)
    assert i27["r_ring"] > i45["r_ring"] * 1.9


def test_inside_the_ring_is_infeasible():
    """距離比環半徑還近 = 該高度需要更陡的俯衝角,不是慢慢修就能到。"""
    t, info = t_min_impact(1000.0, 0.0, 0.0, 157.0, 3100.0, 45.0, 45.0)
    assert math.isinf(t)
    assert "環內" in info["reason"]


def test_heading_error_delays_impact():
    t0, _ = t_min_impact(30000.0, 0.0, 0.0, 157.0, 3100.0, 45.0, 45.0)
    t90, _ = t_min_impact(30000.0, math.radians(90), 0.0, 157.0, 3100.0, 45.0, 45.0)
    assert t90 > t0


def test_zero_dive_angle_rejected():
    with pytest.raises(ValueError):
        t_min_impact(30000.0, 0.0, 0.0, 157.0, 3100.0, 0.0, 45.0)
