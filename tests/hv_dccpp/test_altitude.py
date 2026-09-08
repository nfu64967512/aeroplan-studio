"""tests/test_altitude.py — 契約 §11 `altitude.py` 不變量與規格 §17.2 清單。

情境：`scenario.build_mini_example()`（平地 100 m MSL、西風 5 m/s、A1/B1）以 `dem_step_m=50`、`terrain_margin_m=1500`
建構（起飛進入弧西側達 x≈−806）；山丘反例以就地修改 `ctx.terrain.z` 造 30 m 小丘。
"""
from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

from hv_dccpp import altitude as ALT
from hv_dccpp.context import build_context
from hv_dccpp.dubins import DubinsPath, Pose, dubins_path, discretize_step_rad
from hv_dccpp.geometry import Cell, Strip, lay_strips, sweep_origin
from hv_dccpp.mission import FW_PHASES, Mode, Obstacle, Phase, PlannerConfig, Site, mode_phase_violation
from hv_dccpp.platform import PLATFORM_HERO, PLATFORM_PIONEER, sensor_model
from hv_dccpp.routing import avoid
from hv_dccpp.scenario import build_mini_example, enu_to_latlon, site_from_enu
from hv_dccpp.wind import WindField

A = PLATFORM_PIONEER
B = PLATFORM_HERO


# ---------------------------------------------------------------------------
# fixtures
# ---------------------------------------------------------------------------
@pytest.fixture(scope="module")
def mini():
    inputs, cfg = build_mini_example()
    ctx = build_context(inputs, dataclasses.replace(cfg, dem_step_m=50.0), terrain_margin_m=1500.0)
    return inputs, cfg, ctx


@pytest.fixture(scope="module")
def mini_calm():
    inputs, cfg = build_mini_example()
    inputs = dataclasses.replace(inputs, wind=())
    ctx = build_context(inputs, dataclasses.replace(cfg, dem_step_m=50.0), terrain_margin_m=1500.0)
    return inputs, cfg, ctx


def _strip_a1() -> Strip:
    """§W.3 A 條帶 1（u∈[3932,7068]、v 中心 112 → y = 615 − 112 = 503）。"""
    return Strip("Z1_A_1", "Z1", 1, "A", np.array([3932.0, 503.0]), np.array([7068.0, 503.0]), math.pi / 2,
                 np.array([3985.0, 615.0]), 224.07, 168.05, 160.0, 160.0, 160.0, -112.0, 112.0, 3136.0, 160.0)


def _route_segments(ctx, uav, strip: Strip, z_op: float, h_tc: float = 100.0):
    """起飛走廊 + 進入 + 作業 + 返航 + 降落走廊（單條帶）。"""
    va = ctx.specs[uav.cls].cruise_airspeed_mps
    r = ctx.specs[uav.cls].r_eff(5.0)
    s_entry = strip.start_state(-1)
    to = ALT.takeoff_corridor("H", uav, ctx, s_entry, z_op, 0.0)
    s_last = strip.end_state(-1)
    ld = ALT.landing_corridor("H", uav, ctx, s_last, z_op)
    ex = to.exit_state
    entry = avoid(Pose(ex.x_m, ex.y_m, ex.theta_rad), s_entry, r, ctx.keepouts, z_op, ctx.wind, va, 50.0, {})
    rtb = avoid(s_last, ld.app_state, r, ctx.keepouts, z_op, ctx.wind, va, 50.0, {})
    segs = list(to.segments) + [
        ALT.PathSegment2D(Phase.ENTRY, Mode.CRUISE, entry, None, None, va, False, ex.z_m, z_op),
        ALT.PathSegment2D(Phase.OPERATE, Mode.CRUISE, np.vstack([strip.pL, strip.pR]), strip.strip_id, -1, va, True, z_op, z_op),
        ALT.PathSegment2D(Phase.RTB, Mode.CRUISE, rtb, None, None, va, False, z_op, h_tc),
    ] + list(ld.segments)
    return segs, to, ld


# ---------------------------------------------------------------------------
# ENVELOPE（規格 §11.2；D2）
# ---------------------------------------------------------------------------
def _brute_envelope(z_lo, tan_c, tan_d, ds):
    n = z_lo.size
    D = np.concatenate([[0.0], np.cumsum(ds * tan_d[:-1])])
    C = np.concatenate([[0.0], np.cumsum(ds * tan_c[:-1])])
    z = np.empty(n)
    for j in range(n):
        a = max(z_lo[k] - (D[j] - D[k]) for k in range(j + 1))
        b = max(z_lo[k] - (C[k] - C[j]) for k in range(j, n))
        z[j] = max(a, b)
    return z


def test_envelope_forward_descent_backward_climb():
    """ds 陣列；前向用下降角、後向用爬升角：單一尖峰前坡 = tan_c、後坡 = tan_d。"""
    n = 41
    z_lo = np.zeros(n)
    z_lo[20] = 30.0
    ds = np.full(n - 1, 25.0)
    ds[::2] = 20.0
    tan_c = np.full(n, 0.05)
    tan_d = np.full(n, 0.07)
    env = ALT.envelope(z_lo, np.full(n, 1e3), tan_c, tan_d, ds)
    assert env.feasible
    dz = np.diff(env.z_star)
    # 峰前：以爬升角上升（左斜率 = +tan_c）；峰後：以下降角下降（−tan_d）
    assert np.allclose(dz[15:20], ds[15:20] * 0.05)
    assert np.allclose(dz[20:25], -ds[20:25] * 0.07)
    assert env.z_star[20] == pytest.approx(30.0, abs=1e-9)


def test_envelope_is_pointwise_infimum_and_slopes_within():
    rng = np.random.default_rng(3)
    n = 60
    z_lo = np.cumsum(rng.normal(0, 4, n)) + rng.uniform(0, 30, n)
    ds = rng.uniform(15, 30, n - 1)
    tan_c = rng.uniform(0.03, 0.06, n)
    tan_d = rng.uniform(0.05, 0.09, n)
    env = ALT.envelope(z_lo, np.full(n, 1e4), tan_c, tan_d, ds)
    assert env.feasible
    assert np.all(env.z_star >= z_lo - 1e-9)
    dz = np.diff(env.z_star)
    assert np.all(dz <= ds * tan_c[:-1] + 1e-9)
    assert np.all(dz >= -ds * tan_d[:-1] - 1e-9)
    assert np.allclose(env.z_star, _brute_envelope(z_lo, tan_c, tan_d, ds))
    # 任意其他可行剖面 ≥ z*：以 z* + 隨機非負抬升再包絡仍 ≥ z*
    other = ALT.envelope(env.z_star + rng.uniform(0, 5, n), np.full(n, 1e4), tan_c, tan_d, ds).z_star
    assert np.all(other >= env.z_star - 1e-9)
    # 純量 ds = 全等特例
    e2 = ALT.envelope(z_lo, 1e4, tan_c, tan_d, 25.0)
    e3 = ALT.envelope(z_lo, 1e4, tan_c, tan_d, np.full(n - 1, 25.0))
    assert np.allclose(e2.z_star, e3.z_star)


def test_envelope_pins_and_pin_end_upper_bound():
    n = 30
    ds = 25.0
    tan_c, tan_d = 0.05, 0.07
    z_lo = np.zeros(n)
    env = ALT.envelope(z_lo, np.full(n, 500.0), tan_c, tan_d, ds, pin_start=100.0, pin_end=120.0)
    assert env.feasible
    assert env.z_star[0] == 100.0 and env.z_star[-1] == 120.0
    assert env.u[0] == 100.0 and env.u[-1] == 120.0
    back = np.arange(n)[::-1] * ds * tan_d
    assert np.all(env.u <= 120.0 + back + 1e-9)
    fwd = np.arange(n) * ds * tan_c
    assert np.all(env.u <= 100.0 + fwd + 1e-9)
    # 釘住終點過低 → 不可行（首個違反索引與正缺口）
    bad = ALT.envelope(z_lo + 50.0, np.full(n, 500.0), tan_c, tan_d, ds, pin_start=100.0, pin_end=0.0)
    assert not bad.feasible and bad.first_violation is not None and bad.deficit_m > 0


def test_ridge_gt_3deg_flag():
    """稜線坡度 > tan_c 且空域上限緊 → z* > u，回報 first_violation。"""
    n = 21
    s = np.arange(n) * 25.0
    z_lo = np.where(s < 250, 0.0, (s - 250) * 0.10)       # 5.7° 上坡
    z_hi = z_lo + 8.0
    env = ALT.envelope(z_lo, z_hi, 0.052, 0.07, 25.0)
    assert not env.feasible
    # 後向遞迴使 z* 在坡腳前即已抬升（提前爬升）→ 首個違反點在坡腳或其前、且 z* 高於該處 z_lo
    j = env.first_violation
    assert j is not None and j <= 10 and env.z_star[j] > z_lo[j]
    assert env.deficit_m > 0


# ---------------------------------------------------------------------------
# 取樣（規格 §11.1）
# ---------------------------------------------------------------------------
def test_arc_sampling_min_dd():
    path = dubins_path(Pose(0, 0, 0), Pose(600, 600, math.pi / 2), 200.0)
    seg = ALT.PathSegment2D(Phase.ENTRY, Mode.CRUISE, path, None, None, 21.5, False, 100.0, 160.0)
    cfg = PlannerConfig()
    samples = ALT.resample_segments([seg], cfg.d_d_m, cfg.e_max_m, cfg.psi_max_rad)
    ds = np.array([p.ds_m for p in samples])
    assert np.all(ds[:-1] <= cfg.d_d_m + 1e-9) and samples[-1].ds_m == 0.0
    d_arc = min(cfg.d_d_m, 200.0 * discretize_step_rad(200.0, cfg.e_max_m, cfg.psi_max_rad))
    arc_ds = ds[[p.arc for p in samples]]
    assert np.all(arc_ds[arc_ds > 0] <= d_arc + 1e-9)
    assert abs(samples[-1].s_m - path.length_m) < 1e-6
    assert (samples[0].x_m, samples[0].y_m) == (0.0, 0.0) and abs(samples[-1].x_m - 600) < 1e-6
    assert all(p.arc_param is not None for p in samples if p.arc)
    # 接點：後段屬性取代前段終點
    seg2 = ALT.PathSegment2D(Phase.OPERATE, Mode.CRUISE, np.array([[600.0, 600.0], [600.0, 900.0]]), "s1", -1, 20.0, True, 160.0, 160.0)
    both = ALT.resample_segments([seg, seg2], cfg.d_d_m, cfg.e_max_m, cfg.psi_max_rad)
    j = [i for i, p in enumerate(both) if p.phase is Phase.OPERATE][0]
    assert (both[j].x_m, both[j].y_m) == (600.0, 600.0) and both[j - 1].phase is Phase.ENTRY
    assert both[-1].s_m == pytest.approx(path.length_m + 300.0)


# ---------------------------------------------------------------------------
# 局部包絡（規格 §11.3）
# ---------------------------------------------------------------------------
def test_local_envelope_h_min_ge_clr_minfw(mini):
    inputs, cfg, ctx = mini
    sensor = ctx.sensors[("A", "default")]
    n = 60
    xy = np.column_stack([np.linspace(4000, 7000, n), np.full(n, 503.0)])
    theta = np.full(n, math.pi / 2)
    mask = np.ones(n, dtype=bool)
    mask[:5] = mask[-5:] = False
    for h_op in (160.0, 30.0):                    # h_op 低於 h_minfw 時剖面被 z* 抬到 h_minfw
        prof = ALT.local_envelope(xy, theta, mask, "A", A, sensor, ctx.terrain, 112.0, 0.0, ctx.h_clr_m, 200.0, h_op,
                                  ctx.wind, 24.6, ctx.cfg)
        assert prof.feasible
        assert prof.h_min_m >= max(ctx.h_clr_m, A.min_fw_operation_agl_m) - 1e-9
        assert prof.z.shape == (n,)
    prof = ALT.local_envelope(xy, theta, mask, "A", A, sensor, ctx.terrain, 112.0, 0.0, ctx.h_clr_m, 200.0, 160.0,
                              ctx.wind, 24.6, ctx.cfg)
    assert prof.h_min_m == pytest.approx(160.0, abs=1e-6) and prof.agl_max_m == pytest.approx(160.0, abs=0.5)
    # h_ceil 低於 h_minfw → 不可行
    bad = ALT.local_envelope(xy, theta, mask, "A", A, sensor, ctx.terrain, 112.0, 0.0, ctx.h_clr_m, 100.0, 160.0,
                             ctx.wind, 24.6, ctx.cfg)
    assert not bad.feasible


def test_local_envelope_fn_with_lay_strips(mini):
    """閉包接 geometry.lay_strips：§W.3 A@160 → 7 條、h_min = 160、z_op_m = 160（ENU）。"""
    inputs, cfg, ctx = mini
    sensor = ctx.sensors[("A", "default")]
    cell = ctx.areas_enu["Z1"]
    alpha = math.pi / 2
    origin = sweep_origin(cell, alpha)
    fn = ALT.make_local_envelope_fn("A", A, sensor, ctx.terrain, ctx.h_clr_m, 200.0, 160.0, ctx.wind, 24.6, ctx.cfg,
                                    origin, alpha, 200.0)
    strips = lay_strips(cell, "Z1", "A", alpha, sensor, 160.0, 53.0, fn)
    assert len(strips) == 7
    assert all(s.feasible for s in strips)
    assert all(abs(s.h_min_m - 160.0) < 1e-6 for s in strips)
    zg = {s.strip_id: float(ctx.terrain.corridor_max(np.vstack([s.pL, s.pR]), np.full(2, alpha), s.width_m / 2, 25.0).max())
          for s in strips}
    assert all(abs(s.z_op_m - (zg[s.strip_id] + 160.0)) < 0.5 for s in strips)     # ENU z（含曲率下垂 ≈ −2 m）
    assert all(abs(s.width_m - sensor.swath_m(160.0)) < 1e-6 for s in strips)


# ---------------------------------------------------------------------------
# 盤旋、起飛／降落走廊（規格 §11.6、§11.7）
# ---------------------------------------------------------------------------
def test_orbit_gain_cos_phi(mini):
    inputs, cfg, ctx = mini
    calm = WindField([], ctx.frame, 0.0)
    dh, t = ALT.orbit_gain(A, calm, 100.0, 24.6, 200.0, "climb")
    assert dh == pytest.approx(61.8, abs=0.1) and t == pytest.approx(58.4, abs=0.1)
    dh, t = ALT.orbit_gain(B, calm, 100.0, 6.65, 120.0, "climb")
    assert dh == pytest.approx(35.8, abs=0.1) and t == pytest.approx(39.7, abs=0.1)
    dh, t = ALT.orbit_gain(B, ctx.wind, 100.0, 6.65, 120.0, "climb")
    assert dh == pytest.approx(37.8, abs=0.1) and t == pytest.approx(41.9, abs=0.1)
    dh_d, _ = ALT.orbit_gain(A, ctx.wind, 100.0, 24.6, 200.0, "descent")
    assert dh_d == pytest.approx(86.0, abs=0.5)            # 規格 §11.7：A ≈ 86 m/圈 @W=5
    assert dh == pytest.approx(19.0 * math.sin(math.radians(3)) * math.cos(math.radians(25)) * t)


def test_accel_segment_and_takeoff_exit_state(mini):
    """§W.6：P_tc=(−468,0)、S_acc=((−606,0),270°)；exit_state = P_tc + d_acc·t̂（無盤旋）；Δh=60 不需盤旋。"""
    inputs, cfg, ctx = mini
    a1 = ctx.uav("A1")
    tc = A.transition_cmd()
    res = ALT.takeoff_corridor("H", a1, ctx, Pose(3932, 503, math.pi / 2), 160.0, 0.0)
    assert res.n_laps == 0 and res.issues == []
    assert [s.phase for s in res.segments] == [Phase.TAKEOFF, Phase.TRANS_OUT, Phase.ACCEL]
    assert [s.mode for s in res.segments] == [Mode.HOVER, Mode.TRANSITION, Mode.CRUISE]
    ex = res.exit_state
    th = 3 * math.pi / 2
    p_tc = np.array([-tc.d_tr_m, 0.0])
    assert np.allclose(res.segments[1].path[-1], p_tc, atol=1e-6)
    assert np.allclose([ex.x_m, ex.y_m], p_tc + tc.d_acc_m * np.array([math.sin(th), math.cos(th)]), atol=1e-6)
    assert ex.theta_rad == pytest.approx(th) and ex.z_m == pytest.approx(100.0) and ex.r_eff_m == pytest.approx(200.0)
    assert res.t_s == pytest.approx(30 + 20 + 7) and ex.t_s == pytest.approx(57.0)
    acc = res.segments[2]
    assert acc.z_hint_start_m == acc.z_hint_end_m == pytest.approx(100.0)
    assert np.linalg.norm(acc.path[1] - acc.path[0]) == pytest.approx(tc.d_acc_m)
    assert res.e_wh == pytest.approx(ctx.models["A1"].e_takeoff_wh())


def test_transition_rotor_assisted_slope(mini):
    inputs, cfg, ctx = mini
    tc = A.transition_cmd()
    assert math.degrees(tc.slope_tr_rad) == pytest.approx(4.89, abs=0.05)
    assert math.degrees(tc.slope_tr_rad) > A.planning_climb_angle_deg
    assert tc.slope_tr_rad <= math.radians(A.gamma_tr_max_deg)
    res = ALT.takeoff_corridor("H", ctx.uav("A1"), ctx, Pose(3932, 503, math.pi / 2), 160.0, 0.0)
    assert not any("transition_slope" in s for s in res.issues)
    seg2 = res.segments[1]
    assert (seg2.z_hint_end_m - seg2.z_hint_start_m) / tc.d_tr_m == pytest.approx(math.tan(tc.slope_tr_rad), rel=1e-6)


def test_takeoff_orbit_count_by_envelope(mini_calm):
    """Δh=100、直線進入 500 m → 盤旋圈數 = ⌈(b[0]−z_tc)/Δh_lap⌉；1200 m 全弧進入（κ=cos φ）Δh=61：直線判準
    1200·tan_c=62.8 ≥ 61 誤判無需盤旋，包絡實得 59 m → 1 圈。"""
    inputs, cfg, ctx = mini_calm
    a1 = ctx.uav("A1")
    tc = A.transition_cmd()
    th = 0.0                                                     # 無風：β = 0 → 向北起飛
    p_acc = np.array([0.0, tc.d_tr_m + tc.d_acc_m])
    s_acc = Pose(p_acc[0], p_acc[1], th)
    straight = dubins_path(s_acc, Pose(p_acc[0], p_acc[1] + 500.0, th), 200.0)
    assert straight.length_m == pytest.approx(500.0, abs=1e-6)
    res = ALT.takeoff_corridor("H", a1, ctx, straight.end, 200.0, 0.0, entry_path=straight)
    tan_c = A.tan_gamma_ground("climb", 21.5, 24.6, arc=False)
    dh_lap, _ = ALT.orbit_gain(A, ctx.wind, 150.0, 24.6, 200.0, "climb")
    expect = math.ceil((100.0 - 500.0 * tan_c) / dh_lap)
    assert res.n_laps == expect >= 1
    assert res.orbit_center is not None and any(s.phase is Phase.ORBIT for s in res.segments)
    orb = [s for s in res.segments if s.phase is Phase.ORBIT][-1]
    assert orb.n_laps == res.n_laps and orb.orbit is not None and orb.orbit[2] == pytest.approx(200.0)
    ex = res.exit_state
    assert ex.z_m >= 100.0 + res.n_laps * dh_lap - 1e-6
    assert abs(math.hypot(ex.x_m - orb.orbit[0], ex.y_m - orb.orbit[1]) - 200.0) < 1e-6
    assert res.t_s > 57.0 and res.e_wh > ctx.models["A1"].e_takeoff_wh()
    # 1200 m 全弧
    center = p_acc + 200.0 * np.array([math.cos(th), -math.sin(th)])
    arc = ALT._orbit_path(s_acc, center, 200.0, 1, 1200.0 / 200.0)
    assert arc.length_m == pytest.approx(1200.0)
    assert 1200.0 * tan_c >= 61.0 > 1200.0 * tan_c * math.cos(math.radians(A.max_bank_angle_deg))
    res2 = ALT.takeoff_corridor("H", a1, ctx, arc.end, 161.0, 0.0, entry_path=arc)
    assert res2.n_laps == 1
    res3 = ALT.takeoff_corridor("H", a1, ctx, arc.end, 155.0, 0.0, entry_path=arc)
    assert res3.n_laps == 0


def test_landing_ptc_downwind_and_descent_dist(mini):
    """D19：P_tc = T − d_tr·t̂(β)、S^app = P_tc − d_acc·t̂；Δh=60 需 1060 m（順風保守）→ 短返航加下降盤旋。"""
    inputs, cfg, ctx = mini
    a1 = ctx.uav("A1")
    tc = A.transition_cmd()
    beta = 3 * math.pi / 2
    t_hat = np.array([math.sin(beta), math.cos(beta)])
    res = ALT.landing_corridor("H", a1, ctx, Pose(7068, 166.9, math.pi / 2), 160.0)
    assert res.issues == [] and res.n_laps == 0
    p_tc = -tc.d_tr_m * t_hat
    assert np.allclose(p_tc, [468.0, 0.0], atol=0.5)
    app = res.app_state
    assert np.allclose([app.x_m, app.y_m], p_tc - tc.d_acc_m * t_hat, atol=1e-6)
    assert app.theta_rad == pytest.approx(beta)
    assert [s.phase for s in res.segments] == [Phase.DECEL, Phase.TRANS_IN, Phase.LAND]
    assert np.allclose(res.segments[0].path[-1], p_tc, atol=1e-6)
    assert res.t_s == pytest.approx(7 + 25 + 70 + 15) and res.e_wh == pytest.approx(ctx.models["A1"].e_landing_wh())
    assert A.descent_dist_m(50.0, 26.5) == pytest.approx(883, abs=2)
    assert A.descent_dist_m(60.0, 26.5) == pytest.approx(1060, abs=2)
    # 短返航（700 m < 1060 m）：即使實際逆風可更陡亦不利用 → 盤旋 ≥ 1 圈
    short = ALT.landing_corridor("H", a1, ctx, Pose(app.x_m + 700.0, app.y_m, beta), 160.0)
    assert short.n_laps >= 1 and short.segments[0].phase is Phase.ORBIT
    dh_lap, t_lap = ALT.orbit_gain(A, ctx.wind, 100.0, 24.6, 200.0, "descent")
    assert short.t_s == pytest.approx(res.t_s + short.n_laps * t_lap, rel=0.02)
    assert short.orbit_center is not None and abs(np.linalg.norm(short.orbit_center - [app.x_m, app.y_m]) - 200.0) < 1e-6


def test_choose_orbit_center(mini):
    inputs, cfg, ctx = mini
    c = ALT.choose_orbit_center(np.array([-606.0, 0.0]), 3 * math.pi / 2, 200.0, ctx.keepouts, ctx.link_poly["A"], 150.0, ctx.cfg)
    assert c is not None and np.allclose(c, [-806.0, 0.0])
    import shapely
    tiny = shapely.Point(0, 0).buffer(100.0)
    assert ALT.choose_orbit_center(np.zeros(2), 0.0, 200.0, None, tiny, 150.0, ctx.cfg) is None


# ---------------------------------------------------------------------------
# BUILD_3D（規格 §11.4；D35/D36）
# ---------------------------------------------------------------------------
def test_climb_phase_below_minfw_feasible(mini):
    """平地、pin_start = z_H + 100、h_minfw = 120 → 可行；CLIMB 段 ≈ 471 m（自 P_acc；25 m 取樣）；
    §W 時間：進入 201.8、作業 118.3、返航 422.3 s。"""
    inputs, cfg, ctx = mini
    a1 = ctx.uav("A1")
    st = _strip_a1()
    segs, to, ld = _route_segments(ctx, a1, st, 160.0)
    wps, stats = ALT.build_3d(segs, a1, None, ctx, {st.strip_id: st}, 0.0, 1350.0, 100.0, 100.0)
    assert stats.infeasible_at is None and stats.violations == []
    assert wps[0].phase is Phase.TAKEOFF and wps[-1].phase is Phase.LAND
    samples = ALT.resample_segments(segs, ctx.cfg.d_d_m, ctx.cfg.e_max_m, ctx.cfg.psi_max_rad)
    j_c, j_d = ALT.phase_boundaries(samples, "A", ctx, 100.0, 100.0, 24.6)
    s_acc = 468.0 + 138.25
    climb_len = samples[j_c].s_m - s_acc
    assert 471.0 - 1.0 <= climb_len <= 471.0 + 26.0
    assert all(samples[j].phase is Phase.CLIMB for j in range(j_c - 3, j_c))
    assert samples[j_c].phase is Phase.ENTRY
    i_dec = [i for i, p in enumerate(samples) if p.phase is Phase.DECEL][0]
    desc_len = samples[i_dec].s_m - samples[j_d].s_m
    tan_d_cons = A.tan_gamma_ground("descent", 26.5, 24.6, arc=False)
    assert 20.0 / tan_d_cons - 1.0 <= desc_len <= 20.0 / tan_d_cons + 26.0
    # 時間戳（§W.6/W.7）
    t_of = {}
    for w in wps:
        t_of.setdefault(w.phase, w.t_s)
    assert t_of[Phase.CLIMB] == pytest.approx(57.0)
    assert t_of[Phase.OPERATE] - t_of[Phase.CLIMB] == pytest.approx(201.8, abs=1.0)
    assert t_of[Phase.RTB] - t_of[Phase.OPERATE] == pytest.approx(118.3, abs=0.5)
    assert t_of[Phase.DECEL] - t_of[Phase.RTB] == pytest.approx(422.3, abs=1.0)
    assert t_of[Phase.LAND] - t_of[Phase.DECEL] == pytest.approx(7 + 25)
    assert wps[-1].t_s - t_of[Phase.LAND] == pytest.approx(70.0)
    # 高度：CLIMB 段 AGL < 120 允許、其後固定翼段 ≥ 120；OPER = 160；RTB 以 4° 降至 P_tc = 100
    z_by = {}
    for w in wps:
        z_by.setdefault(w.phase, []).append(w.z_m)
    assert z_by[Phase.CLIMB][0] == pytest.approx(100.0) and min(z_by[Phase.CLIMB]) < 120.0 - 5.0    # CLIMB 允許 < h_minfw
    agl = {}
    for w in wps:
        agl.setdefault(w.phase, []).append(w.agl_m)
    assert min(agl[Phase.ENTRY]) >= 120.0 - 1e-6 and min(agl[Phase.RTB]) >= 120.0 - 1e-6
    assert min(agl[Phase.OPERATE]) >= 160.0 - 1e-6 and max(agl[Phase.OPERATE]) <= 170.0 + 1e-6   # 平滑只向上偏離（D36）
    assert z_by[Phase.DECEL][0] == pytest.approx(100.0) and z_by[Phase.DECEL][-1] == pytest.approx(100.0)
    assert stats.agl_min_by_strip[st.strip_id] >= 160.0 - 1e-6
    assert stats.slope_max_rad <= math.atan(A.tan_gamma_ground("climb", 16.5, 24.6, False)) + 1e-6
    assert stats.slope_min_rad >= -math.atan(A.tan_gamma_ground("descent", 16.5, 24.6, False)) - 1e-6
    # 航點欄位
    for w in wps:
        assert mode_phase_violation(w.mode, w.phase, w.airspeed_mps, A.v_min_mission()) is None
        assert w.lost_link_action in ("RTB", "CONTINUE") and w.nearest_alternate in ("H", "ALT1")
        assert w.alt_reachable
        assert abs(w.alt_msl_m - ctx.frame.enu_z_to_msl(np.array([[w.x_m, w.y_m]]), w.z_m)[0]) < 1e-3   # D42
    assert any(w.lost_link_action == "CONTINUE" for w in wps if w.phase is Phase.OPERATE)     # A: CONTINUE_THEN_RTB
    assert all(w.lost_link_action == "RTB" for w in wps if w.phase in (Phase.RTB, Phase.DESCENT, Phase.LAND))
    assert all(w.sensor_on for w in wps if w.phase is Phase.OPERATE and w.strip_id == st.strip_id)
    assert any(w.arc is not None for w in wps if w.phase in (Phase.ENTRY, Phase.RTB))
    assert wps[-1].energy_remaining_wh < 1350.0 - ctx.models["A1"].e_takeoff_wh()
    assert stats.n_points == len(wps) < len(samples)


def test_no_hover_outside_vtol_phases_and_energy(mini):
    inputs, cfg, ctx = mini
    b1 = ctx.uav("B1")
    st = Strip("Z1_B_1", "Z1", 1, "B", np.array([3937.0, -337.8]), np.array([7063.0, -337.8]), math.pi / 2,
               np.array([3985.0, 615.0]), 146.875, 102.81, 100.0, 100.0, 100.0, 879.4, 1026.3, 3126.0, 100.0)
    segs, to, ld = _route_segments(ctx, b1, st, 100.0, h_tc=90.0)
    wps, stats = ALT.build_3d(segs, b1, None, ctx, {st.strip_id: st}, 60.0, 380.0, 90.0, 90.0)
    assert stats.violations == [] and stats.infeasible_at is None
    assert wps[0].t_s == pytest.approx(60.0)
    assert all(w.mode is not Mode.HOVER for w in wps if w.phase not in (Phase.TAKEOFF, Phase.LAND))
    assert all(w.lost_link_action == "RTB" for w in wps)                 # B 預設 RTB
    e = np.array([w.energy_remaining_wh for w in wps])
    assert np.all(np.diff(e) <= 1e-9)
    assert e[0] == pytest.approx(380.0)
    a = [w.agl_m for w in wps if w.phase is Phase.OPERATE]
    assert min(a) >= 100.0 - 1e-6 and max(a) <= 101.0


def _add_hill(ctx, cx: float, cy: float, height: float, sigma: float) -> None:
    T = ctx.terrain
    xs = T.x0_m + T.step_m * np.arange(T.nx)
    ys = T.y0_m + T.step_m * np.arange(T.ny)
    X, Y = np.meshgrid(xs, ys)
    bump = height * np.exp(-((X - cx) ** 2 + (Y - cy) ** 2) / (2 * sigma ** 2))
    T.z[:] = T.z + bump.astype(T.z.dtype)
    T.z_raw[:] = T.z_raw + bump.astype(T.z_raw.dtype)


def test_oper_lower_bound_includes_hmin_hill_counterexample():
    """走廊上 30 m 小丘（規格 §11.4 反例）：D36 使 z* 於丘頂 ≥ z_g^max + h_min，平滑無法低於；OPER 樣本 AGL ≥ h_min。"""
    inputs, cfg = build_mini_example()
    ctx = build_context(inputs, dataclasses.replace(cfg, dem_step_m=25.0), terrain_margin_m=1500.0)
    _add_hill(ctx, 5500.0, 503.0, 30.0, 45.0)
    a1 = ctx.uav("A1")
    st = _strip_a1()
    st.h_min_m = 150.0
    st.h_op_m = 150.0
    segs, to, ld = _route_segments(ctx, a1, st, 150.0)
    wps, stats = ALT.build_3d(segs, a1, None, ctx, {st.strip_id: st}, 0.0, 1350.0, 100.0, 100.0)
    assert stats.infeasible_at is None and stats.violations == []
    assert stats.agl_min_by_strip[st.strip_id] >= 150.0 - 1e-6
    samples = ALT.resample_segments(segs, ctx.cfg.d_d_m, ctx.cfg.e_max_m, ctx.cfg.psi_max_rad)
    j_c, j_d = ALT.phase_boundaries(samples, "A", ctx, 100.0, 100.0, 24.6)
    z_lo, z_hi, z_des, tan_c, tan_d, ds = ALT.sample_bounds(samples, "A", ctx, {st.strip_id: st}, 24.6, j_c, j_d)
    op = np.array([p.phase is Phase.OPERATE for p in samples])
    xy = np.array([[p.x_m, p.y_m] for p in samples])
    zg = ctx.terrain.corridor_max(xy, np.array([p.theta_rad for p in samples]), 112.0, 12.5)
    assert zg[op].max() > 25.0                      # 小丘確實在走廊內
    assert np.all(z_lo[op] >= zg[op] + 150.0 - 1e-9)
    # 丘頂附近航點：z ≥ 地形 + 150（平滑後仍成立）
    ops = [w for w in wps if w.phase is Phase.OPERATE]
    assert max(w.z_m for w in ops) > 150.0 + 20.0
    # 相對：若 OPER 下界不含 h_min（舊版），平滑會把丘頂壓低 — 以直接呼叫 smooth_projected 驗證夾持
    zz = ALT.smooth_projected(z_des[op], z_des[op], z_lo[op] - 40.0, z_hi[op], ctx.cfg)
    assert zz.min() < (zg[op] + 150.0).max() - 5.0


def test_smoothing_then_clamp():
    rng = np.random.default_rng(1)
    n = 80
    z_star = np.cumsum(rng.normal(0, 1.5, n)) + 100.0
    u = z_star + 25.0
    z_des = z_star + rng.uniform(0, 25, n)
    z = np.clip(z_des, z_star, u)
    out = ALT.smooth_projected(z, z_des, z_star, u, PlannerConfig())
    assert np.all(out >= z_star - 1e-9) and np.all(out <= u + 1e-9)
    assert out[0] == z[0] and out[-1] == z[-1]
    assert np.abs(np.diff(out, 2)).sum() < np.abs(np.diff(z, 2)).sum()      # 更平滑
    env = ALT.envelope(out, u, 0.05, 0.07, 25.0)
    assert np.all(env.z_star >= out - 1e-9)


def test_reduce_points():
    cfg = PlannerConfig()
    n = 100
    pts = [ALT.Sample2D(25.0 * i, 0.0, 0.0, 25.0 * i, 25.0, Phase.RTB, Mode.CRUISE, False, None, False, 21.5, 0) for i in range(n)]
    z = np.full(n, 150.0)
    keep = np.zeros(n, dtype=bool)
    keep[50] = True
    idx = ALT.reduce_points(pts, z, keep, cfg)
    assert idx[0] == 0 and idx[-1] == n - 1 and 50 in idx
    gaps = np.diff(idx) * 25.0
    assert np.all(gaps <= cfg.L_max_m + 1e-9) and len(idx) <= 8
    # 折點保留
    z2 = z.copy()
    z2[30:] += np.arange(70) * 0.5
    idx2 = ALT.reduce_points(pts, z2, keep, cfg)
    assert 30 in idx2 or 29 in idx2


# ---------------------------------------------------------------------------
# 站點檢查、分層（規格 §11.8、§11.9）
# ---------------------------------------------------------------------------
def test_site_checks(mini):
    inputs, cfg, ctx = mini
    assert ALT.site_check("H", A, ctx, ctx.obstacle_pairs()) == []
    assert ALT.site_check("ALT1", B, ctx, ctx.obstacle_pairs()) == []
    assert ALT.site_check("nope", A, ctx, []) == ["site_unknown(nope)"]
    # CLIMB 走廊內 150 m 高塔（P_tc 西 430 m；高度線 80 + 430·0.0425 ≈ 98 m）→ climb_corridor_obstacle
    frame = ctx.frame
    lat, lon = enu_to_latlon(frame, -900.0, 0.0)
    tower = Obstacle("T1", "point", ((lat, lon),), None, 150.0, category="tower")
    inp2 = dataclasses.replace(inputs, obstacles=(tower,))
    ctx2 = build_context(inp2, dataclasses.replace(cfg, dem_step_m=50.0), terrain_margin_m=1500.0)
    iss = ALT.site_check("H", A, ctx2, ctx2.obstacle_pairs())
    assert any("climb_corridor_obstacle" in s for s in iss)
    # 起飛走廊本身亦回報（淨空／盤旋）而不崩潰
    res = ALT.takeoff_corridor("H", ctx2.uav("A1"), ctx2, Pose(3932, 503, math.pi / 2), 160.0, 0.0)
    assert isinstance(res.issues, list)
    # 淨空半徑內障礙 + 站點過小之淨空圓
    lat, lon = enu_to_latlon(frame, 2004.0, 1500.0)
    near = Obstacle("T2", "point", ((lat, lon),), None, 20.0, category="tower")
    inp3 = dataclasses.replace(inputs, obstacles=(near,))
    ctx3 = build_context(inp3, dataclasses.replace(cfg, dem_step_m=50.0), terrain_margin_m=1500.0)
    iss3 = ALT.site_check("ALT1", B, ctx3, ctx3.obstacle_pairs())
    assert any("obstacle_in_clear_radius" in s for s in iss3)
    small = dataclasses.replace(inputs, home=dataclasses.replace(inputs.home, clear_radius_m=2.0))
    ctx4 = build_context(small, dataclasses.replace(cfg, dem_step_m=50.0), terrain_margin_m=1500.0)
    assert any("clear_radius_too_small" in s for s in ALT.site_check("H", A, ctx4, []))


@dataclasses.dataclass
class _Band:
    cls: str
    h_op_m: float
    h_lo_m: float
    h_hi_m: float
    dh_band_m: float = 0.0
    feasible: bool = True
    reason: str = ""


def _cell(h_a: float | None, h_b: float | None) -> Cell:
    import shapely
    feas = {c: h is not None for c, h in (("A", h_a), ("B", h_b))}
    hop = {c: h for c, h in (("A", h_a), ("B", h_b)) if h is not None}
    return Cell("c1", "Z1", shapely.box(0, 0, 100, 100), "both", "default", 1.0, None, 1e4, 100.0, 0.0, 0.0,
                {}, hop, feas, {}, {}, {}, {}, {}, {}, {}, {})


def test_layering_margin_delta_prof():
    cfg = PlannerConfig()
    specs = {"A": A, "B": B}
    bands = {"c1": {"A": _Band("A", 150.0, 120.0, 200.0), "B": _Band("B", 100.0, 100.0, 180.0)}}
    out = ALT.layering([_cell(150.0, 100.0)], specs, bands, cfg)
    assert out["c1"] == {"A": 160.0, "B": 100.0}
    # 已滿足 → 不變
    assert ALT.layering([_cell(170.0, 100.0)], specs, bands, cfg)["c1"] == {"A": 170.0, "B": 100.0}
    # A 無法再高（h_hi 155）→ 降 B（h_lo 80）
    bands2 = {"c1": {"A": _Band("A", 150.0, 120.0, 155.0), "B": _Band("B", 100.0, 80.0, 180.0)}}
    assert ALT.layering([_cell(150.0, 100.0)], specs, bands2, cfg)["c1"] == {"A": 150.0, "B": 90.0}
    # 反向：B 在上
    bands3 = {"c1": {"A": _Band("A", 130.0, 120.0, 135.0), "B": _Band("B", 100.0, 100.0, 200.0)}}
    out3 = ALT.layering([_cell(130.0, 100.0)], specs, bands3, cfg)["c1"]
    assert out3["B"] - out3["A"] >= 60.0 - 1e-9 and out3["B"] <= 200.0
    # 兩層皆無空間 → 時間分離
    bands4 = {"c1": {"A": _Band("A", 130.0, 120.0, 135.0), "B": _Band("B", 100.0, 95.0, 110.0)}}
    assert ALT.layering([_cell(130.0, 100.0)], specs, bands4, cfg)["c1"] == {"mode": "time_separation"}
    # 單構型胞原樣
    assert ALT.layering([_cell(150.0, None)], specs, bands, cfg)["c1"] == {"A": 150.0}


# ---------------------------------------------------------------------------
# §11.1 / §11.8 CLIMB／DESCENT 走廊淨空長度（R-03；規格書 v1.1 的「B 480 m」是錯的）
# ---------------------------------------------------------------------------
def test_climb_corridor_length_pioneer_942_hero_724():
    """L = (h_minfw − H_tc,lo)/tanγ^g，H_tc,lo 取 transition_complete_agl_m 下緣（D29）。

    平台參數表：A 80–100 / 120 → Δh = 40；B 70–90 / 100 → Δh = 30。順風 5 m/s：A 942 m、B 724 m。
    規格書 §11.1、§11.8 寫的「A 940 m、B 480 m」中 480 m 對應 Δh = 20 m（誤用 Pioneer 的下緣 80 m），
    設計文件的 724 m 才對；此測試把正確值釘住。
    """
    assert A.transition_cmd().h_tc_clear_m == 80.0 and A.min_fw_operation_agl_m == 120.0
    assert B.transition_cmd().h_tc_clear_m == 70.0 and B.min_fw_operation_agl_m == 100.0
    assert ALT.climb_corridor_length_m(A, 5.0) == pytest.approx(942.0, abs=1.0)
    assert ALT.climb_corridor_length_m(B, 5.0) == pytest.approx(724.0, abs=1.0)
    assert ALT.climb_corridor_length_m(B, 5.0) > 700.0          # 絕不是 480 m
    # 無風
    assert ALT.climb_corridor_length_m(A, 0.0) == pytest.approx(764.3, abs=1.0)
    assert ALT.climb_corridor_length_m(B, 0.0) == pytest.approx(573.2, abs=1.0)
    # 下降側（γ_d）較短
    assert ALT.climb_corridor_length_m(A, 5.0, side="descent") == pytest.approx(706.8, abs=1.0)
    assert ALT.climb_corridor_length_m(B, 5.0, side="descent") == pytest.approx(543.2, abs=1.0)


def test_climb_corridor_issues_uses_full_length(mini):
    """_climb_corridor_issues 之取樣長度 = climb_corridor_length_m：把地形抬到 600 m 處（> 規格書誤植的 480 m、
    < 正確的 724 m）超過爬升線時，HERO 必須抓到 climb_corridor_terrain。"""
    _inputs, _cfg, ctx = mini
    spec = B
    tc = spec.transition_cmd()
    p_tc = np.array([0.0, 0.0])
    z_site = float(ctx.z_ground_home_m)
    w = ALT._wind_speed_at(ctx.wind, z_site + tc.h_tc_clear_m)
    L = ALT.climb_corridor_length_m(spec, w, spec.reference_auw_kg)
    assert 700.0 < L < 740.0
    tan_g = spec.tan_gamma_ground("climb", spec.cruise_airspeed_mps + w, spec.reference_auw_kg, arc=False)
    line_z = z_site + tc.h_tc_clear_m + 600.0 * tan_g
    terr = ctx.terrain
    z_before = terr.z.copy()
    raw_before = terr.z_raw.copy() if getattr(terr, "z_raw", None) is not None else None
    try:
        xy = terr.grid_xy() if hasattr(terr, "grid_xy") else None
        # 600 m 處（沿 +y，theta = 0）抬高一塊地形到爬升線之上
        gx, gy = np.meshgrid(np.arange(terr.z.shape[1]), np.arange(terr.z.shape[0]))
        px = terr.x0_m + gx * terr.step_m
        py = terr.y0_m + gy * terr.step_m
        sel = (np.abs(px) <= 120.0) & (np.abs(py - 600.0) <= 120.0)
        bump = float(line_z - ctx.h_clr_m + 30.0)
        terr.z[sel] = np.maximum(terr.z[sel], bump)
        if raw_before is not None:
            terr.z_raw[sel] = np.maximum(terr.z_raw[sel], bump)
        if hasattr(terr, "_raw_view"):
            terr._raw_view = None
        issues = ALT._climb_corridor_issues(ctx, spec, "H", p_tc, 0.0, z_site, tc, spec.reference_auw_kg, "climb")
        assert any(i.startswith("climb_corridor_terrain") for i in issues), issues
    finally:
        terr.z[...] = z_before
        if raw_before is not None:
            terr.z_raw[...] = raw_before
        if hasattr(terr, "_raw_view"):
            terr._raw_view = None
