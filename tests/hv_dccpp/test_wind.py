"""tests/test_wind.py — 契約 §4 不變量與規格 §17.2（ground_speed_triangle、crosswind_infeasible、
arc_time_jensen、r_eff_wind_policy_max_layer）＋ dubins.path_time 一致性。"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import pytest

from hv_dccpp import dubins
from hv_dccpp import wind as W
from hv_dccpp.geo import GeoFrame
from hv_dccpp.platform import PLATFORM_HERO, PLATFORM_PIONEER, MissionInputError

A = PLATFORM_PIONEER
B = PLATFORM_HERO
VA_A = A.cruise_airspeed_mps      # 21.5
VA_B = B.cruise_airspeed_mps      # 19.0
FRAME = GeoFrame(23.95, 120.9, 100.0)


@dataclass(frozen=True)
class Layer:
    """mission.WindLayer 之鴨子型別替身（欄位同契約 §3）。"""
    z_bottom_m: float
    z_top_m: float
    speed_mps: float
    dir_from_deg: float
    gust_mps: float = 0.0
    z_ref: str = "msl"


def uniform_field(speed: float = 5.0, dir_from_deg: float = 270.0, gust: float = 7.0) -> W.WindField:
    """單層 AGL 0–1000 m 風場（home 地面 z=0）。"""
    return W.WindField([Layer(0.0, 1000.0, speed, dir_from_deg, gust, "agl")], FRAME, 0.0)


# ---------------------------------------------------------------------------
# 風向量與風三角
# ---------------------------------------------------------------------------
def test_wind_vector_convention():
    """w = −W(sin β, cos β)：from 270 吹向東、from 0 吹向南。"""
    assert np.allclose(W.wind_vector(5.0, 270.0), [[5.0, 0.0]])
    assert np.allclose(W.wind_vector(5.0, 0.0), [[0.0, -5.0]])
    assert np.allclose(W.wind_vector(5.0, 90.0), [[-5.0, 0.0]])
    assert np.allclose(W.wind_vector(5.0, 180.0), [[0.0, 5.0]])
    out = W.wind_vector([1.0, 2.0], [270.0, 0.0])
    assert out.shape == (2, 2)


@pytest.mark.parametrize("beta_deg", [0.0, 37.0, 90.0, 123.4, 270.0, 359.0])
@pytest.mark.parametrize("wind", [0.0, 3.0, 8.0])
def test_ground_speed_headwind_tailwind_identities(beta_deg, wind):
    """ground_speed(β)=Va−W（逆風）、ground_speed(β+π)=Va+W（順風）、側風 √(Va²−W²)。"""
    beta = math.radians(beta_deg)
    w = W.wind_vector(wind, beta_deg)[0]
    assert W.ground_speed(beta, VA_A, w)[0] == pytest.approx(VA_A - wind, abs=1e-9)
    assert W.ground_speed(beta + math.pi, VA_A, w)[0] == pytest.approx(VA_A + wind, abs=1e-9)
    for side in (beta + math.pi / 2, beta - math.pi / 2):
        assert W.ground_speed(side, VA_A, w)[0] == pytest.approx(math.sqrt(VA_A ** 2 - wind ** 2), abs=1e-9)


def test_ground_speed_contract_numbers_W2():
    """契約 §W.2：A 東 26.5／西 16.5／南北 20.91；B 24.0／14.0／18.33（西風 5 m/s）。"""
    w = W.wind_vector(5.0, 270.0)[0]
    th = np.radians([90.0, 270.0, 0.0, 180.0])
    vg_a = W.ground_speed(th, VA_A, w)
    vg_b = W.ground_speed(th, VA_B, w)
    assert vg_a == pytest.approx([26.5, 16.5, 20.91, 20.91], abs=0.01)
    assert vg_b == pytest.approx([24.0, 14.0, 18.33, 18.33], abs=0.01)
    assert vg_a.shape == (4,)


def test_ground_speed_triangle_broadcast():
    """(2,) 廣播與 (N,2) 逐點；純量 theta 回 (1,)。"""
    rng = np.random.default_rng(0)
    th = rng.uniform(0, 2 * math.pi, 50)
    winds = rng.uniform(-6, 6, (50, 2))
    per_point = W.ground_speed(th, VA_A, winds)
    assert per_point.shape == (50,)
    for i in range(50):
        assert per_point[i] == pytest.approx(W.ground_speed(th[i], VA_A, winds[i])[0], abs=1e-12)
    bcast = W.ground_speed(th, VA_A, winds[0])
    assert bcast.shape == (50,)
    assert bcast[0] == per_point[0]
    assert W.ground_speed(1.0, VA_A, winds[0]).shape == (1,)
    with pytest.raises(ValueError):
        W.ground_speed(th, VA_A, np.zeros(3))
    with pytest.raises(ValueError):
        W.ground_speed(th, 0.0, winds[0])


def test_crosswind_infeasible():
    """|W⊥| ≥ Va → nan（ground_speed 與 heading_from_track 皆然）；恰等亦 nan。"""
    th = math.pi / 2                                 # 向東
    w_strong = W.wind_vector(25.0, 0.0)[0]          # 北風 25 → 純側風
    assert math.isnan(W.ground_speed(th, VA_A, w_strong)[0])
    assert math.isnan(W.heading_from_track(th, VA_A, w_strong)[0])
    w_equal = W.wind_vector(VA_A, 0.0)[0]
    assert math.isnan(W.ground_speed(th, VA_A, w_equal)[0])
    w_ok = W.wind_vector(VA_A - 1e-6, 0.0)[0]
    assert np.isfinite(W.ground_speed(th, VA_A, w_ok)[0])
    # 混合陣列：只有超出者為 nan
    out = W.ground_speed(np.array([th, th]), VA_A, np.vstack([w_strong, W.wind_vector(5, 0)[0]]))
    assert math.isnan(out[0]) and np.isfinite(out[1])


def test_heading_from_track_identity_and_sign():
    """Va(sin ψ, cos ψ) + w = Vg (sin θ, cos θ)；風向右 ⇒ ψ < θ（機頭向左偏）。"""
    rng = np.random.default_rng(1)
    th = rng.uniform(0, 2 * math.pi, 40)
    winds = rng.uniform(-7, 7, (40, 2))
    psi = W.heading_from_track(th, VA_A, winds)
    vg = W.ground_speed(th, VA_A, winds)
    lhs = VA_A * np.column_stack([np.sin(psi), np.cos(psi)]) + winds
    rhs = vg[:, None] * np.column_stack([np.sin(th), np.cos(th)])
    assert np.allclose(lhs, rhs, atol=1e-9)
    # 純逆風／順風：ψ = θ
    w = W.wind_vector(5.0, 270.0)[0]
    assert W.heading_from_track(math.radians(90), VA_A, w)[0] == pytest.approx(math.radians(90))
    # 向北飛、西風（吹向東 = 右側）→ 機頭向左（ψ < θ）
    assert W.heading_from_track(0.0, VA_A, w)[0] < 0.0
    assert W.heading_from_track(0.0, VA_A, w)[0] == pytest.approx(-math.asin(5.0 / VA_A))


# ---------------------------------------------------------------------------
# WindField
# ---------------------------------------------------------------------------
def test_windfield_msl_to_enu_and_sorting():
    """MSL 層界於 home 以 msl_to_enu_z 換算（h0=100 → z = msl − 100）；層依 z 排序。"""
    layers = [Layer(300.0, 500.0, 8.0, 180.0, 10.0), Layer(100.0, 300.0, 5.0, 270.0, 7.0)]
    wf = W.WindField(layers, FRAME, 0.0)
    assert wf.z_bounds_enu_m.shape == (2, 2)
    assert np.allclose(wf.z_bounds_enu_m, [[0.0, 200.0], [200.0, 400.0]], atol=1e-3)
    assert wf.layers[0] is layers[1] and wf.layers[1] is layers[0]
    assert wf.speeds_mps.tolist() == [5.0, 8.0]
    assert wf.version == 0
    assert any("msl" in n for n in wf.notes)
    assert wf.speed_at(50.0) == 5.0 and wf.speed_at(250.0) == 8.0
    assert wf.gust_at(50.0) == 7.0 and wf.gust_at(250.0) == 10.0
    assert wf.dir_from_at(50.0) == pytest.approx(math.radians(270.0))
    assert wf.dir_from_at(250.0) == pytest.approx(math.pi)


def test_windfield_agl_layer_uses_home_ground():
    """z_ref='agl' 層以 home 地面 ENU z 換算；notes 記錄假設。"""
    wf = W.WindField([Layer(0.0, 200.0, 4.0, 90.0, z_ref="agl")], FRAME, 12.5)
    assert np.allclose(wf.z_bounds_enu_m, [[12.5, 212.5]])
    assert wf.z_ground_home_m == 12.5
    assert any("agl" in n for n in wf.notes)
    # agl 層不需要 frame
    wf2 = W.WindField([Layer(0.0, 200.0, 4.0, 90.0, z_ref="agl")], None, 0.0)
    assert wf2.speed_at(100.0) == 4.0
    with pytest.raises(MissionInputError):
        W.WindField([Layer(0.0, 200.0, 4.0, 90.0, z_ref="msl")], None, 0.0)


def test_windfield_overlap_lower_and_outside_nearest():
    """重疊取較低層；範圍外／縫隙取最近層（並列取較低層）。"""
    wf = W.WindField([Layer(0.0, 200.0, 5.0, 270.0, z_ref="agl"),
                      Layer(150.0, 400.0, 8.0, 0.0, z_ref="agl")], FRAME, 0.0)
    assert wf.speed_at(170.0) == 5.0            # 重疊 → 較低層
    assert wf.speed_at(200.0) == 5.0            # 邊界含於較低層
    assert wf.speed_at(300.0) == 8.0
    assert wf.speed_at(-50.0) == 5.0            # 下方 → 最近（最低）層
    assert wf.speed_at(1000.0) == 8.0           # 上方 → 最近（最高）層
    gap = W.WindField([Layer(0.0, 100.0, 5.0, 270.0, z_ref="agl"),
                       Layer(300.0, 400.0, 8.0, 0.0, z_ref="agl")], FRAME, 0.0)
    assert gap.speed_at(150.0) == 5.0           # 縫隙：離下層近
    assert gap.speed_at(250.0) == 8.0           # 離上層近
    assert gap.speed_at(200.0) == 5.0           # 等距 → 較低層
    assert gap.layer_index([150.0, 250.0]).tolist() == [0, 1]


def test_windfield_wind_vec_shape_direction_nan():
    wf = uniform_field(5.0, 270.0)
    assert wf.wind_vec(100.0).shape == (1, 2)
    assert np.allclose(wf.wind_vec(100.0), [[5.0, 0.0]])
    out = wf.wind_vec(np.array([10.0, 500.0, 5000.0]))
    assert out.shape == (3, 2) and np.allclose(out, [[5.0, 0.0]] * 3)
    assert np.isnan(wf.wind_vec(np.nan)).all()
    assert wf.max_speed() == 5.0
    assert wf.n_layers == 1


def test_windfield_empty_is_calm():
    wf = W.WindField([], FRAME, 0.0)
    assert wf.n_layers == 0
    assert np.allclose(wf.wind_vec([0.0, 100.0]), 0.0)
    assert wf.speed_at(100.0) == 0.0 and wf.gust_at(100.0) == 0.0 and wf.max_speed() == 0.0
    assert wf.max_speed_between(0.0, 1000.0) == 0.0
    assert "wind_layers_empty_assumed_calm" in wf.notes
    assert dubins.ground_speed_uniform(0.3, VA_A, wf.wind_vec(10.0)[0]) == pytest.approx(VA_A)


@pytest.mark.parametrize("bad", [
    Layer(200.0, 100.0, 5.0, 270.0),                 # top ≤ bottom
    Layer(0.0, 100.0, -1.0, 270.0),                  # 負風速
    Layer(0.0, 100.0, 5.0, 270.0, gust_mps=-2.0),    # 負陣風
    Layer(0.0, 100.0, 5.0, 270.0, z_ref="ellipsoid"),
])
def test_windfield_bad_layer_raises(bad):
    with pytest.raises(MissionInputError):
        W.WindField([bad], FRAME, 0.0)


def test_max_speed_between_matches_dense_sampling():
    """max_speed_between 與 speed_at 之密集取樣最大值一致（含重疊、縫隙、範圍外）。"""
    wf = W.WindField([Layer(0.0, 100.0, 3.0, 270.0, z_ref="agl"),
                      Layer(80.0, 160.0, 9.0, 0.0, z_ref="agl"),        # 重疊，部分被遮蔽
                      Layer(300.0, 400.0, 6.0, 90.0, z_ref="agl"),      # 縫隙 160–300
                      Layer(400.0, 500.0, 4.0, 90.0, z_ref="agl")], FRAME, 0.0)
    ranges = [(0, 500), (-100, 50), (50, 120), (100, 100), (161, 229), (231, 299), (150, 260),
              (350, 700), (700, 900), (500, 0), (-1e9, 1e9), (-math.inf, math.inf), (450, math.inf)]
    for lo, hi in ranges:
        lo_c = max(min(lo, hi), -200.0)
        hi_c = min(max(lo, hi), 800.0)
        dense = np.arange(lo_c, hi_c + 1e-9, 0.25)
        expected = float(wf.speeds_at(dense).max())
        got = wf.max_speed_between(lo, hi)
        assert got == pytest.approx(expected), (lo, hi)
    assert wf.max_speed_between(161.0, 229.0) == 9.0        # 縫隙下半：最近層為第 2 層
    assert wf.max_speed_between(231.0, 299.0) == 6.0        # 縫隙上半
    assert wf.max_speed_between(0.0, 79.0) == 3.0
    assert wf.max_speed() == 9.0


# ---------------------------------------------------------------------------
# R_eff、W_R 政策、地面坡度、最壞地速
# ---------------------------------------------------------------------------
def test_r_eff_wrapper():
    assert W.r_eff(A, 0.0) == 200.0 and W.r_eff(B, 0.0) == 120.0
    assert W.r_eff(A, 8.0) == pytest.approx(245.0, abs=1.0)
    assert W.r_eff(B, 8.0) == pytest.approx(152.0, abs=1.0)
    assert W.r_eff(A, 5.0) == 200.0                 # W=5 < W*=5.17 → R_plan
    assert W.r_eff(A, 8.0) == A.r_eff(8.0)


def test_r_eff_wind_policy_max_layer():
    """max_layer ≥ 範圍內任一層值（含 op_layer）；隨 z_op_max 擴大單調不減；政策名錯誤 → ValueError。"""
    wf = W.WindField([Layer(0.0, 150.0, 3.0, 270.0, z_ref="agl"),
                      Layer(150.0, 250.0, 9.0, 270.0, z_ref="agl"),
                      Layer(250.0, 600.0, 6.0, 270.0, z_ref="agl")], FRAME, 0.0)
    z_site = 0.0
    h_tc_lo = A.transition_cmd().h_tc_clear_m       # 80
    assert h_tc_lo == 80.0
    assert W.r_eff_wind_speed(wf, A, z_site, 100.0, "max_layer") == 3.0
    assert W.r_eff_wind_speed(wf, A, z_site, 200.0, "max_layer") == 9.0
    assert W.r_eff_wind_speed(wf, A, z_site, 400.0, "max_layer") == 9.0
    assert W.r_eff_wind_speed(wf, A, z_site, 400.0, "op_layer") == 6.0
    assert W.r_eff_wind_speed(wf, A, z_site, 200.0, "op_layer") == 9.0
    # z_op_max 低於 z_site + H_tc：範圍退化為單點
    assert W.r_eff_wind_speed(wf, A, z_site, 10.0, "max_layer") == wf.speed_at(z_site + h_tc_lo)
    prev = -1.0
    for z_op in np.linspace(50.0, 700.0, 27):
        v = W.r_eff_wind_speed(wf, A, z_site, float(z_op), "max_layer")
        assert v >= prev                                                       # 單調不減
        assert v >= W.r_eff_wind_speed(wf, A, z_site, float(z_op), "op_layer")
        for z in np.linspace(z_site + h_tc_lo, max(z_op, z_site + h_tc_lo), 15):
            assert v >= wf.speed_at(float(z))
        prev = v
    # 站點抬高 → 範圍上移
    assert W.r_eff_wind_speed(wf, A, 100.0, 200.0, "max_layer") == 9.0
    assert W.r_eff_wind_speed(wf, A, 200.0, 290.0, "max_layer") == 6.0
    with pytest.raises(ValueError):
        W.r_eff_wind_speed(wf, A, 0.0, 200.0, "mean_layer")


def test_tan_gamma_ground_wrapper():
    """A、W=8 順風：tanγ_c^g = 21.5 sin3°/29.5 = 0.0381（規格 §3.2）；不可行 → inf；弧段 × cos φ_max。"""
    w = W.wind_vector(8.0, 270.0)[0]                 # 吹向東
    tail = W.tan_gamma_ground(A, "climb", math.radians(90), w, A.reference_auw_kg, arc=False)
    assert tail == pytest.approx(0.0381, abs=5e-4)
    head = W.tan_gamma_ground(A, "climb", math.radians(270), w, A.reference_auw_kg, arc=False)
    assert head > tail
    assert head == pytest.approx(VA_A * math.sin(math.radians(3)) / (VA_A - 8.0), rel=1e-9)
    arc = W.tan_gamma_ground(A, "climb", math.radians(90), w, A.reference_auw_kg, arc=True)
    assert arc == pytest.approx(tail * math.cos(math.radians(A.max_bank_angle_deg)), rel=1e-9)
    desc = W.tan_gamma_ground(A, "descent", math.radians(90), w, A.reference_auw_kg, arc=False)
    assert desc == pytest.approx(VA_A * math.sin(math.radians(4)) / 29.5, rel=1e-9)
    assert W.tan_gamma_ground(A, "climb", 0.0, W.wind_vector(25.0, 270.0)[0], 24.6, False) == math.inf
    # 等於 spec 薄包裝
    vg = W.ground_speed(math.radians(90), VA_A, w)[0]
    assert tail == A.tan_gamma_ground("climb", vg, A.reference_auw_kg, False)


def test_worst_case_v_g():
    assert W.worst_case_v_g(A, 8.0) == pytest.approx(13.5)
    assert W.worst_case_v_g(A, 0.0) == VA_A
    assert W.worst_case_v_g(A, 25.0) == 3.0
    assert W.worst_case_v_g(A, 25.0, v_g_min_mps=5.0) == 5.0
    assert W.worst_case_v_g(B, 8.0) == pytest.approx(11.0)


# ---------------------------------------------------------------------------
# 風中時間
# ---------------------------------------------------------------------------
def test_circle_time_factor_and_arc_time_jensen():
    """circle_time_factor(21.5, 5) ≈ 1.043；≥ 1（Jensen）；≤ 代理 1+(W/Va)²；W=0 → 1；W ≥ Va → inf。"""
    assert W.circle_time_factor(VA_A, 5.0) == pytest.approx(1.043, abs=1e-3)
    assert W.circle_time_factor(VA_A, 0.0) == pytest.approx(1.0, abs=1e-12)
    prev = 0.0
    for wind in np.linspace(0.0, 21.0, 43):
        f = W.circle_time_factor(VA_A, wind)
        assert f >= 1.0                                   # Jensen：圓周平均 1/Vg ≥ 1/Va
        assert f >= prev                                  # 隨 W 單調增
        prev = f
    # 代理 1+(W/Va)² 在作業風速範圍（W ≤ W_go = 8 m/s，D7）內為保守上界；W→Va 時精確值發散、代理不再保守
    for wind in np.linspace(0.0, 8.0, 17):
        assert W.circle_time_factor(VA_A, wind) <= 1.0 + (wind / VA_A) ** 2 + 1e-12
        assert W.circle_time_factor(VA_B, wind) <= 1.0 + (wind / VA_B) ** 2 + 1e-12
    assert W.circle_time_factor(VA_A, 12.0) > 1.0 + (12.0 / VA_A) ** 2
    assert W.circle_time_factor(VA_A, VA_A) == math.inf
    assert W.circle_time_factor(VA_A, -5.0) == W.circle_time_factor(VA_A, 5.0)
    # 與 time_arc 整圈 Riemann 和（極細步長）一致
    wf = uniform_field(5.0, 270.0)
    t_circle = W.time_arc(np.zeros(2), 200.0, 0.3, 2 * math.pi, VA_A, wf, 100.0, step_rad=1e-3)
    assert t_circle / (2 * math.pi * 200.0 / VA_A) == pytest.approx(W.circle_time_factor(VA_A, 5.0), abs=1e-6)
    # 規格 §8.1：A、W=5 半圓（東→南→西）30.5 s，L/Va=29.2 s 低估 4.3%
    t_half = W.time_arc(np.zeros(2), 200.0, math.pi / 2, math.pi, VA_A, wf, 100.0)
    assert t_half == pytest.approx(30.5, abs=0.1)
    assert math.pi * 200.0 / VA_A == pytest.approx(29.2, abs=0.05)


def test_time_arc_is_riemann_sum_and_converges():
    """time_arc = Σ R δ / Vg(θ_j)（中點；n = ⌈|Δθ|/step⌉）；step→0 收斂至解析積分。"""
    wf = uniform_field(6.0, 200.0)
    R, th0, dth = 150.0, 0.7, -2.1
    step = abs(dth) / 12
    n = 12
    delta = abs(dth) / n
    th = th0 - delta * (np.arange(n) + 0.5)
    manual = float(np.sum(R * delta / W.ground_speed(th, VA_A, wf.wind_vec(50.0)[0])))
    assert W.time_arc(np.array([1.0, 2.0]), R, th0, dth, VA_A, wf, 50.0, step_rad=step) == pytest.approx(manual, abs=1e-12)
    # 收斂：誤差隨步長縮小且 5° 之相對誤差 < 1e-3
    ref = W.time_arc(np.zeros(2), R, th0, dth, VA_A, wf, 50.0, step_rad=1e-4)
    errs = [abs(W.time_arc(np.zeros(2), R, th0, dth, VA_A, wf, 50.0, step_rad=math.radians(s)) - ref)
            for s in (30.0, 10.0, 5.0, 1.0, 0.1)]
    assert all(errs[i] >= errs[i + 1] for i in range(len(errs) - 1))
    assert errs[2] / ref < 1e-3
    assert errs[-1] / ref < 1e-7
    # 零弧、方向對稱、不可行
    assert W.time_arc(np.zeros(2), R, th0, 0.0, VA_A, wf, 50.0) == 0.0
    assert W.time_arc(np.zeros(2), R, 0.0, 2 * math.pi, VA_A, wf, 50.0) == pytest.approx(
        W.time_arc(np.zeros(2), R, 0.0, -2 * math.pi, VA_A, wf, 50.0), rel=1e-9)
    strong = uniform_field(30.0, 270.0)
    assert W.time_arc(np.zeros(2), R, 0.0, math.pi, VA_A, strong, 50.0) == math.inf
    with pytest.raises(ValueError):
        W.time_arc(np.zeros(2), R, 0.0, 1.0, VA_A, wf, 50.0, step_rad=0.0)
    with pytest.raises(ValueError):
        W.time_arc(np.zeros(2), 0.0, 0.0, 1.0, VA_A, wf, 50.0)


def test_time_line_identities():
    wf = uniform_field(5.0, 270.0)
    p0 = np.array([0.0, 0.0])
    assert W.time_line(p0, np.array([265.0, 0.0]), VA_A, wf, 100.0) == pytest.approx(10.0)       # 順風 26.5
    assert W.time_line(p0, np.array([-165.0, 0.0]), VA_A, wf, 100.0) == pytest.approx(10.0)      # 逆風 16.5
    assert W.time_line(p0, np.array([0.0, 100.0]), VA_A, wf, 100.0) == pytest.approx(100.0 / math.sqrt(VA_A ** 2 - 25.0))
    assert W.time_line(p0, p0, VA_A, wf, 100.0) == 0.0
    assert W.time_line(p0, np.array([-100.0, 0.0]), VA_A, uniform_field(21.5, 270.0), 100.0) == math.inf
    assert W.time_line(p0, np.array([0.0, 100.0]), VA_A, uniform_field(25.0, 270.0), 100.0) == math.inf
    # 也接受 (2,) 風向量與 None
    assert W.time_line(p0, np.array([265.0, 0.0]), VA_A, np.array([5.0, 0.0]), 0.0) == pytest.approx(10.0)
    assert W.time_line(p0, np.array([215.0, 0.0]), VA_A, None, 0.0) == pytest.approx(10.0)
    # 分層：不同 z 取不同層
    wf2 = W.WindField([Layer(0.0, 100.0, 5.0, 270.0, z_ref="agl"), Layer(100.0, 300.0, 5.0, 90.0, z_ref="agl")], FRAME, 0.0)
    assert W.time_line(p0, np.array([265.0, 0.0]), VA_A, wf2, 50.0) == pytest.approx(10.0)
    assert W.time_line(p0, np.array([165.0, 0.0]), VA_A, wf2, 200.0) == pytest.approx(10.0)


@pytest.mark.parametrize("start,end,R", [
    ((7068.0, 503.0, math.pi / 2), (7068.0, -1.2, 3 * math.pi / 2), 200.0),      # §W.5 skip-3 U 型（RSR）
    ((0.0, 0.0, 0.0), (900.0, 400.0, math.radians(200)), 200.0),
    ((100.0, -50.0, math.radians(300)), (-300.0, 250.0, math.radians(45)), 120.0),
    ((0.0, 0.0, 0.0), (150.0, 100.0, math.pi), 200.0),                          # Ω / 短距
])
def test_path_time_agrees_with_dubins(start, end, R):
    """Σ time_line（S）+ time_arc（L/R）== dubins.path_time（同一取點規則；<1e-9 s）。"""
    wf = uniform_field(5.0, 270.0)
    z = 260.0
    path = dubins.dubins_path(dubins.Pose(*start), dubins.Pose(*end), R)
    total = 0.0
    for seg in path.segments:
        if seg.length_m <= 0.0:
            continue
        if seg.kind == "S":
            total += W.time_line(np.array([seg.start.x_m, seg.start.y_m]), np.array([seg.end.x_m, seg.end.y_m]),
                                 VA_A, wf, z)
        else:
            assert seg.center is not None
            total += W.time_arc(np.asarray(seg.center), path.R_m, seg.start.theta_rad, seg.dtheta_rad, VA_A, wf, z)
    ref = dubins.path_time(path, VA_A, wf, z)
    assert total == pytest.approx(ref, abs=1e-9)
    assert total >= path.length_m / (VA_A + 5.0) - 1e-9 and total <= path.length_m / (VA_A - 5.0) + 1e-9


def test_W5_transfer_time_35p5():
    """契約 §W.5：A 條帶 1 → 4（skip-3 U 型）732.5 m、35.5 s；代理 ≥ 精確值。"""
    wf = uniform_field(5.0, 270.0)
    path = dubins.dubins_path(dubins.Pose(7068.0, 503.0, math.pi / 2), dubins.Pose(7068.0, -1.2, 3 * math.pi / 2), 200.0)
    assert path.length_m == pytest.approx(732.5, abs=0.5)
    t = sum((W.time_line(np.array([s.start.x_m, s.start.y_m]), np.array([s.end.x_m, s.end.y_m]), VA_A, wf, 260.0)
             if s.kind == "S" else
             W.time_arc(np.asarray(s.center), path.R_m, s.start.theta_rad, s.dtheta_rad, VA_A, wf, 260.0))
            for s in path.segments if s.length_m > 0.0)
    assert t == pytest.approx(35.5, abs=0.1)
    proxy = W.proxy_turn_time(path.length_m, VA_A, 5.0)
    assert proxy >= t
    assert proxy == pytest.approx(path.length_m / VA_A * (1 + (5.0 / VA_A) ** 2))
    assert W.proxy_turn_time(1000.0, VA_A, 0.0) == pytest.approx(1000.0 / VA_A)
