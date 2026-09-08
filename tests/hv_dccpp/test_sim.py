"""tests/test_sim.py — 契約 §14 `sim.py` 不變量與規格 §17.2 清單（example_coverage_ge_99p5、all_checks_pass、
runtime_under_1s、legacy_reproduces_paper_table2、hetero_le_homogeneous_baseline、mini_example_numbers）。

情境：`scenario.build_mini_example()`（平地、西風 5 m/s；`dem_step_m=50`、`terrain_margin_m=1500`）經 `planner.plan_mission`
得完整 `MissionPlan`，再以 `sim.build_validation_cache(budget_only=True)` 重建能量預算表。違規偵測以合成／變造之航點驗證。
"""
from __future__ import annotations

import dataclasses
import math
import time
from types import SimpleNamespace

import numpy as np
import pytest
from shapely.geometry import Polygon, box

from hv_dccpp import sim
from hv_dccpp.context import build_context
from hv_dccpp.costs import CostCache
from hv_dccpp.mission import FW_PHASES, Mode, Phase, UAVPlan, Waypoint
from hv_dccpp.planner import plan_mission
from hv_dccpp.scenario import build_mini_example

E_MAX_PLUS_E_XT = 3.0 + 5.0      # 規格 §17.1 (9)：橫向偏差 ≤ e_max + e_xt


# ---------------------------------------------------------------------------
# fixtures
# ---------------------------------------------------------------------------
@pytest.fixture(scope="module")
def mini():
    inputs, cfg = build_mini_example()
    cfg = dataclasses.replace(cfg, dem_step_m=50.0)
    ctx = build_context(inputs, cfg, terrain_margin_m=1500.0)
    t0 = time.perf_counter()
    mp = plan_mission(inputs, cfg, ctx=ctx)
    t_plan = time.perf_counter() - t0
    cache = sim.build_validation_cache(mp, ctx, budget_only=True)
    return SimpleNamespace(inputs=inputs, cfg=cfg, ctx=ctx, mp=mp, cache=cache, t_plan=t_plan)


@pytest.fixture(scope="module")
def report(mini):
    return sim.validate(mini.mp, mini.ctx, mini.cache, mini.cfg)


def _wp(x: float, y: float, z: float, t: float, phase: Phase, *, mode: Mode = Mode.CRUISE, strip_id: str | None = None,
        cell_id: str | None = None, d: int | None = None, sensor_on: bool = False, va: float = 21.5, vg: float = 21.5,
        track: float = 90.0, e: float = 1000.0) -> Waypoint:
    return Waypoint(x_m=x, y_m=y, z_m=z, lat_deg=0.0, lon_deg=0.0, alt_msl_m=z, agl_m=z, t_s=t, mode=mode, phase=phase,
                    sensor_on=sensor_on, strip_id=strip_id, cell_id=cell_id, dir=d, airspeed_mps=va, ground_speed_mps=vg,
                    track_deg=track, heading_deg=track, energy_remaining_wh=e, nearest_alternate="H", alt_reachable=True,
                    lost_link_action="RTB")


def _oper_plan(uav_id: str, cls: str, strips: list, *, vg: float = 20.0, ds: float = 25.0) -> UAVPlan:
    """只含 OPER 航段之合成計畫：每條帶 pL → pR（dir −1）於 z_op，取樣 ds。"""
    wps: list[Waypoint] = []
    t = 0.0
    for s in strips:
        pl, pr = np.asarray(s.pL, float), np.asarray(s.pR, float)
        L = float(np.hypot(*(pr - pl)))
        n = max(1, int(math.ceil(L / ds)))
        for k in range(n + 1):
            f = k / n
            p = pl + f * (pr - pl)
            wps.append(_wp(p[0], p[1], float(s.z_op_m), t, Phase.OPERATE, strip_id=s.strip_id, cell_id=s.cell_id, d=-1,
                           sensor_on=(k < n), vg=vg, track=math.degrees(s.theta_LR_rad)))
            t += (L / n) / vg
        t += 60.0
    return UAVPlan(uav_id=uav_id, cls=cls, waypoints=wps, strips=[(s.strip_id, -1) for s in strips], t_takeoff_s=0.0,
                   t_arr_ptc_s=t, t_land_s=t, landing_site="H", energy_used_wh=0.0, energy_margin_wh=0.0,
                   distance_m=0.0, cruise_equiv_km=0.0, orbits_inserted=0, warnings=[])


def _first_index(plan: UAVPlan, phase: Phase) -> int:
    return next(j for j, w in enumerate(plan.waypoints) if w.phase is phase)


# ---------------------------------------------------------------------------
# mini example：全部檢查通過（規格 §17.2 all_checks_pass / example_coverage_ge_99p5）
# ---------------------------------------------------------------------------
def test_mini_plan_sane(mini):
    mp = mini.mp
    assert mp.feasibility.status != "NO_GO"
    assert set(mp.plans) == {"A1", "B1"}
    assert mp.uncovered == []


def test_example_coverage_ge_99p5(mini):
    reps = sim.coverage_raster(mini.mp, mini.ctx, mini.cfg)
    assert [r.area_id for r in reps] == ["Z1"]
    r = reps[0]
    assert r.cls_required == ["all"]                       # sensor_req="any"
    assert r.raster_m == mini.cfg.cov_raster_m == 5.0
    assert r.coverage_by_cls["all"] >= 0.995 and r.passed
    assert r.coverage >= 0.995
    # A 5 條（v ∈ [0, 896]，區域 y ∈ [−600, 600] → 880/1200）、B 殘帶 3 條（352.5 m）；union 涵蓋全區（契約 §W.10）
    assert r.coverage_by_cls["A"] == pytest.approx(0.733, abs=0.01)
    assert r.coverage_by_cls["B"] == pytest.approx(0.28, abs=0.02)
    assert r.coverage_by_cls["lidar"] == r.coverage_by_cls["A"] and r.coverage_by_cls["rgb"] == r.coverage_by_cls["B"]
    assert r.uncovered_cells_xy.shape == (0, 2)


def test_mini_kinematics_ok(mini):
    for uid, p in mini.mp.plans.items():
        out = sim.check_kinematics(p, mini.ctx.specs[p.cls], mini.ctx, mini.cfg, strips=mini.mp.strips)
        assert out == [], (uid, out)


def test_mini_energy_ok(mini):
    for uid, p in mini.mp.plans.items():
        out = sim.check_energy(p, mini.ctx.models[uid], mini.ctx, mini.cache)
        assert out == [], (uid, out)


def test_mini_link_airspace_ok(mini):
    cells = {c.cell_id: c for c in mini.mp.cells}
    for uid, p in mini.mp.plans.items():
        assert sim.check_link_airspace(p, mini.ctx, cells=cells) == []


def test_all_checks_pass(mini, report):
    rep = report
    assert rep.passed
    assert rep.coverage_ok and all(c.passed for c in rep.coverage)
    assert rep.slope_ok and rep.agl_ok and rep.curvature_ok and rep.energy_ok and rep.link_ok and rep.airspace_ok \
        and rep.time_window_ok
    assert rep.conflicts == []
    assert rep.min_sep_h_m > mini.cfg.d_sep_m               # A/B 分層 60 m 且空間錯開（契約 §W.9）
    assert math.isinf(rep.min_sep_v_m)                      # 從未水平接近 < d_sep
    assert rep.makespan_s == pytest.approx(mini.mp.makespan_s)
    assert rep.total_distance_m == pytest.approx(sum(p.distance_m for p in mini.mp.plans.values()))
    assert rep.compute_ms > 0
    assert set(rep.details) >= {"A1", "B1", "separation", "coverage"}
    assert rep.details["A1"] == [] and rep.details["B1"] == []
    assert any("PASS" in s for s in rep.details["coverage"])


def test_mini_example_numbers(mini):
    """契約 §W.10 `mini_example_numbers`（本模組可觸及之項目；其餘於 test_costs／test_altitude）。"""
    mp, ctx = mini.mp, mini.ctx
    parent = next(c for c in mp.cells if c.parent_cell_id is None)
    assert len(parent.strips["A"]) == 7 and len(parent.strips["B"]) == 12
    subs = [c for c in mp.cells if c.parent_cell_id is not None]
    k_a = {c.k_A for c in subs}
    assert len(k_a) == 1 and next(iter(k_a)) in {5, 6}
    k = next(iter(k_a))
    n_b_res = len(mp.plans["B1"].strips)
    assert n_b_res == {5: 3, 6: 2}[k]
    assert len(mp.plans["A1"].strips) == k
    spec_a = ctx.specs["A"]
    assert spec_a.transition_cmd().d_tr_m == 468.0
    assert spec_a.climb_dist_m(dh_m=60.0, v_g_max_mps=26.5, auw_kg=24.6) == pytest.approx(1413.0, abs=5.0)
    # makespan_final ≈ 29.3 ± 0.5 min（含起飛間隔與降落排序）
    assert mp.makespan_s / 60.0 == pytest.approx(29.3, abs=0.5)
    # 最遠點能量（A1 進入 RTB 時）：契約 §W.8 1114 Wh 為 3 條轉場 + 爬升之手算；實作含真實剖面、Ω 轉場 → 1078
    # （見 CONTRACT_DEVIATIONS）；仍須遠高於 r·E_nom + min_rtb（603.6）。
    p = mp.plans["A1"]
    j = _first_index(p, Phase.RTB)
    e_far = p.waypoints[j].energy_remaining_wh
    assert 1000.0 <= e_far <= 1150.0
    need = ctx.models["A1"].reserve_wh() + min(mini.cache.budget.point_wh("A", np.array([p.waypoints[j].x_m, p.waypoints[j].y_m]),
                                                                          p.waypoints[j].z_m, s) for s in mini.cache.budget.site_ids)
    # 契約 §W.8：自條帶 3 右端 RTB 需求 450 + 153.6 = 603.6；IDP 可自左端進入（§W.7 不斷言進入側）→ 自左端 RTB 較短（≈ 545）
    assert ctx.models["A1"].reserve_wh() == 450.0
    assert 450.0 + 80.0 <= need <= 620.0
    assert e_far >= need
    e_land = p.waypoints[-1].energy_remaining_wh - ctx.models["A1"].segment_wh(Mode.HOVER, spec_a.t_hover_reserve_s)
    assert e_land >= ctx.models["A1"].reserve_wh()


# ---------------------------------------------------------------------------
# coverage_raster：合成計畫
# ---------------------------------------------------------------------------
def test_coverage_synthetic_full_and_gap(mini):
    mp0, ctx, cfg = mini.mp, mini.ctx, mini.cfg
    parent = next(c for c in mp0.cells if c.parent_cell_id is None)
    a_strips, b_strips = list(parent.strips["A"]), list(parent.strips["B"])          # 母胞 7 + 12 條（未全入快取）
    assert len(a_strips) == 7 and len(b_strips) == 12
    mp = dataclasses.replace(mp0, strips={**mp0.strips, **{s.strip_id: s for s in a_strips + b_strips}})
    full = dataclasses.replace(mp, plans={"A1": _oper_plan("A1", "A", a_strips), "B1": _oper_plan("B1", "B", b_strips)})
    r = sim.coverage_raster(full, ctx, cfg)[0]
    assert r.coverage_by_cls["lidar"] >= 0.995 and r.coverage_by_cls["rgb"] >= 0.995 and r.coverage_by_cls["all"] >= 0.995
    assert r.uncovered_cells_xy.shape == (0, 2)
    # both 需求：兩感測器各自 ≥ 0.995
    r2 = sim.CoverageReport("Z1", ["lidar", "rgb"], dict(r.coverage_by_cls), r.uncovered_cells_xy, r.raster_m)
    assert r2.passed and r2.coverage >= 0.995
    # 缺一條 A（第 4 條，y ≈ −0.6）→ lidar 出現缺口，未覆蓋格落在該條帶帶內
    gap = dataclasses.replace(mp, plans={"A1": _oper_plan("A1", "A", a_strips[:3] + a_strips[4:])})
    rg = sim.coverage_raster(gap, ctx, cfg)[0]
    assert rg.coverage_by_cls["A"] < 0.995
    exp_gap = (a_strips[3].width_m - 2 * (a_strips[3].spacing_m - 0.5 * a_strips[3].width_m) * 0) / 1200.0
    assert 1.0 - rg.coverage_by_cls["A"] < exp_gap + 0.01
    assert rg.uncovered_cells_xy.shape[0] > 0
    ys = rg.uncovered_cells_xy[:, 1]
    assert ys.min() > a_strips[4].pL[1] + 0.5 * a_strips[4].width_m - 6.0
    assert ys.max() < a_strips[2].pL[1] - 0.5 * a_strips[2].width_m + 6.0
    # 分母排除禁飛區：全區禁飛 → 覆蓋率 1（無可覆蓋格）
    ctx_nf = dataclasses.replace(ctx, nofly_buffered=[ctx.areas_enu["Z1"].buffer(10.0)])
    assert sim.coverage_raster(gap, ctx_nf, cfg)[0].coverage_by_cls["all"] == 1.0


def test_coverage_actual_agl_and_camera_exposure(mini):
    """足跡寬度依實際 AGL：條帶抬高 → 足跡變寬（一條 B 即覆蓋更多）；相機曝光間距 > 前向足跡 → 只計曝光矩形。"""
    mp0, ctx, cfg = mini.mp, mini.ctx, mini.cfg
    parent = next(c for c in mp0.cells if c.parent_cell_id is None)
    s = parent.strips["B"][5]
    mp = dataclasses.replace(mp0, strips={**mp0.strips, s.strip_id: s})
    base = sim.coverage_raster(dataclasses.replace(mp, plans={"B1": _oper_plan("B1", "B", [s])}), ctx, cfg)[0]
    hi_plan = _oper_plan("B1", "B", [s])
    hi_plan.waypoints = [dataclasses.replace(w, z_m=w.z_m + 40.0) for w in hi_plan.waypoints]
    hi = sim.coverage_raster(dataclasses.replace(mp, plans={"B1": hi_plan}), ctx, cfg)[0]
    sensor = ctx.sensors[("B", "default_rgb")]
    ratio = sensor.swath_m(140.0) / sensor.swath_m(100.0)
    assert hi.coverage_by_cls["rgb"] / base.coverage_by_cls["rgb"] == pytest.approx(ratio, rel=0.03)
    fast = _oper_plan("B1", "B", [s], vg=400.0)             # d_exp = 400 m ≫ l(100) = 97.5 m
    fr = sim.coverage_raster(dataclasses.replace(mp, plans={"B1": fast}), ctx, cfg)[0]
    l_fp = sensor.footprint_len_m(100.0)
    assert fr.coverage_by_cls["rgb"] < base.coverage_by_cls["rgb"] * 0.5
    assert fr.coverage_by_cls["rgb"] == pytest.approx(base.coverage_by_cls["rgb"] * min(1.0, l_fp / 400.0), rel=0.15)


def test_raster_helpers_consistent():
    q = np.array([[0.0, 0.0], [100.0, 0.0], [100.0, 20.0], [0.0, 20.0]])
    a = sim._raster_quad(q, -10.0, -10.0, 5.0, 40, 10)
    b = sim._raster_geom(Polygon(q), -10.0, -10.0, 5.0, 40, 10)
    c = sim._raster_run([q], -10.0, -10.0, 5.0, 40, 10)
    assert a is not None and b is not None and c is not None
    A = np.zeros((10, 40), bool); A[a[0], a[1]] = a[2]
    B = np.zeros((10, 40), bool); B[b[0], b[1]] = b[2]
    C = np.zeros((10, 40), bool); C[c[0], c[1]] = c[2]
    assert A.sum() == 20 * 4 and np.array_equal(A, B) and np.array_equal(A, C)
    # 共線鏈與四邊形逐一相同
    chain = [np.array([[x, -5.0], [x + 50.0, -5.0], [x + 50.0, 5.0], [x, 5.0]])[[0, 1, 2, 3]] for x in (0.0, 50.0, 100.0)]
    chain = [np.array([[c_[0][0], 5.0], [c_[1][0], 5.0], [c_[2][0], -5.0], [c_[3][0], -5.0]]) for c_ in chain]
    r1 = sim._raster_ribbon(chain, -10.0, -10.0, 1.0, 200, 20)
    assert r1 is not None
    R1 = np.zeros((20, 200), bool); R1[r1[0], r1[1]] = r1[2]
    R2 = np.zeros((20, 200), bool)
    for c_ in chain:
        h = sim._raster_quad(c_, -10.0, -10.0, 1.0, 200, 20)
        R2[h[0], h[1]] |= h[2]
    assert np.array_equal(R1, R2) and R1.sum() == 150 * 10


# ---------------------------------------------------------------------------
# check_kinematics：違規偵測
# ---------------------------------------------------------------------------
def _with_wps(plan: UAVPlan, wps: list[Waypoint]) -> UAVPlan:
    return dataclasses.replace(plan, waypoints=wps)


def _kinds(out: list[str]) -> set[str]:
    return {s.split(":")[0] for s in out}


def test_kinematics_mode_phase_and_time(mini):
    p = mini.mp.plans["A1"]
    spec, ctx, cfg = mini.ctx.specs["A"], mini.ctx, mini.cfg
    j = _first_index(p, Phase.OPERATE) + 3
    wps = list(p.waypoints)
    wps[j] = dataclasses.replace(wps[j], mode=Mode.HOVER)
    out = sim.check_kinematics(_with_wps(p, wps), spec, ctx, cfg, strips=mini.mp.strips)
    assert "mode_phase" in _kinds(out) and any(f"wp={j}" in s for s in out)
    wps = list(p.waypoints)
    wps[j] = dataclasses.replace(wps[j], airspeed_mps=spec.v_min_mission() - 1.0)
    assert "mode_phase" in _kinds(sim.check_kinematics(_with_wps(p, wps), spec, ctx, cfg))
    wps = list(p.waypoints)
    wps[j] = dataclasses.replace(wps[j], t_s=wps[j + 1].t_s + 5.0)
    assert "time_not_monotone" in _kinds(sim.check_kinematics(_with_wps(p, wps), spec, ctx, cfg))
    assert sim.check_kinematics(_with_wps(p, []), spec, ctx, cfg) == []


def test_kinematics_slope_agl_ceiling(mini):
    p = mini.mp.plans["A1"]
    spec, ctx, cfg = mini.ctx.specs["A"], mini.ctx, mini.cfg
    j = _first_index(p, Phase.OPERATE) + 2
    wps = list(p.waypoints)
    wps[j] = dataclasses.replace(wps[j], z_m=wps[j].z_m + 200.0)     # 25 m 腿 +200 m → 坡度遠超 3.5°/5°；OPER AGL 360 > h_max 331.8
    out = sim.check_kinematics(_with_wps(p, wps), spec, ctx, cfg, strips=mini.mp.strips)
    k = _kinds(out)
    assert "slope_exceeded" in k and "oper_agl_above_h_max" in k
    assert "above_ceiling" in k                                     # AGL 360 > h_ceil 200（waiver）
    wps = list(p.waypoints)
    je = _first_index(p, Phase.ENTRY) + 5
    wps[je] = dataclasses.replace(wps[je], z_m=ctx.terrain.z_at(np.array([[wps[je].x_m, wps[je].y_m]]))[0] + 10.0)
    k = _kinds(sim.check_kinematics(_with_wps(p, wps), spec, ctx, cfg, strips=mini.mp.strips))
    assert "agl_below_h_clr" in k and "agl_below_h_minfw" in k
    # OPER 段低於 h_min,l（條帶寬度不變量 D36）
    wps = list(p.waypoints)
    for jj in range(j, j + 4):
        wps[jj] = dataclasses.replace(wps[jj], z_m=wps[jj].z_m - 30.0)
    k = _kinds(sim.check_kinematics(_with_wps(p, wps), spec, ctx, cfg, strips=mini.mp.strips))
    assert "oper_agl_below_h_min" in k
    # 轉換段坡度以 γ_tr,max 驗
    jt = _first_index(p, Phase.TRANS_OUT)
    wps = list(p.waypoints)
    wps[jt + 1] = dataclasses.replace(wps[jt + 1], z_m=wps[jt + 1].z_m + 40.0)
    assert "transition_slope_exceeded" in _kinds(sim.check_kinematics(_with_wps(p, wps), spec, ctx, cfg))


def test_kinematics_wind_corrected_slope(mini):
    """地面坡度以 V_a sinγ/V_g 驗：逆風 16.5 m/s 之 3.5° 空氣坡對應地面 4.7°，順風 26.5 m/s 對應 2.9°。"""
    ctx, cfg, spec = mini.ctx, mini.cfg, mini.ctx.specs["A"]
    z0 = float(ctx.terrain.z_at(np.array([[500.0, 0.0]]))[0]) + 160.0
    va = spec.cruise_airspeed_mps

    def plan_with(track_deg: float, slope_deg: float) -> UAVPlan:
        th = math.radians(track_deg)
        wps = []
        for k in range(6):
            s_ = 100.0 * k
            wps.append(_wp(500.0 + s_ * math.sin(th), s_ * math.cos(th), z0 + s_ * math.tan(math.radians(slope_deg)),
                           10.0 * k, Phase.ENTRY, va=va, vg=va, track=track_deg, e=1200.0))
        return dataclasses.replace(_oper_plan("A1", "A", []), waypoints=wps)

    # 逆風（向西 270°：V_g = 16.5）：地面 4.2° 仍在限內（空氣 3.2°）；順風（向東：V_g = 26.5）：地面 4.2° 違規（空氣 5.2°）
    assert "slope_exceeded" not in _kinds(sim.check_kinematics(plan_with(270.0, 4.2), spec, ctx, cfg))
    assert "slope_exceeded" in _kinds(sim.check_kinematics(plan_with(90.0, 4.2), spec, ctx, cfg))
    # 下降：順風地面 −3.5° 合法（空氣 −4.3°）、逆風地面 −6.5°（空氣 −5.0° 邊界外）違規
    assert "slope_exceeded" not in _kinds(sim.check_kinematics(plan_with(90.0, -3.5), spec, ctx, cfg))
    assert "slope_exceeded" in _kinds(sim.check_kinematics(plan_with(270.0, -6.6), spec, ctx, cfg))


def test_kinematics_curvature(mini):
    ctx, cfg, spec = mini.ctx, mini.cfg, mini.ctx.specs["A"]
    z0 = float(ctx.terrain.z_at(np.array([[500.0, 0.0]]))[0]) + 160.0
    # 之字形（每 100 m 轉 60°）→ 外接圓半徑 ≈ 100 m < R_eff 200
    pts = [(500.0, 0.0)]
    th = 0.0
    for k in range(8):
        th += math.radians(60.0 if k % 2 == 0 else -60.0)
        pts.append((pts[-1][0] + 100.0 * math.sin(th), pts[-1][1] + 100.0 * math.cos(th)))
    wps = [_wp(x, y, z0, 5.0 * k, Phase.ENTRY, e=1200.0) for k, (x, y) in enumerate(pts)]
    out = sim.check_kinematics(dataclasses.replace(_oper_plan("A1", "A", []), waypoints=wps), spec, ctx, cfg)
    assert "curvature_below_R_eff" in _kinds(out)
    # 外切多邊形之 Dubins 弧（R = R_eff）通過：以 mini 計畫之真實轉場為證
    p = mini.mp.plans["A1"]
    assert "curvature_below_R_eff" not in _kinds(sim.check_kinematics(p, spec, ctx, cfg))
    assert sim._r_eff_for_plan(spec, ctx, p.waypoints) == pytest.approx(200.0)
    assert sim._r_eff_for_plan(ctx.specs["B"], ctx, mini.mp.plans["B1"].waypoints) == pytest.approx(120.0)


# ---------------------------------------------------------------------------
# check_energy：違規偵測（規格 §5.5、§17.1 (4)）
# ---------------------------------------------------------------------------
def test_energy_detects_deficits(mini):
    p = mini.mp.plans["A1"]
    model, ctx, cache = mini.ctx.models["A1"], mini.ctx, mini.cache
    wps = [dataclasses.replace(w, energy_remaining_wh=w.energy_remaining_wh - 900.0) for w in p.waypoints]
    out = sim.check_energy(_with_wps(p, wps), model, ctx, cache)
    k = _kinds(out)
    assert {"energy_alt_unreachable", "landing_reserve", "missed_approach_energy"} <= k
    # 只在 P_tc 之後不足 → 只有 landing_reserve
    j_tc = _first_index(p, Phase.TRANS_IN)
    wps = list(p.waypoints)
    for j in range(j_tc + 1, len(wps)):
        wps[j] = dataclasses.replace(wps[j], energy_remaining_wh=model.reserve_wh() - 5.0)
    k = _kinds(sim.check_energy(_with_wps(p, wps), model, ctx, cache))
    assert "landing_reserve" in k and "energy_alt_unreachable" not in k
    # 落點非回收點且無 planned_landing_at_alternate 警告
    k = _kinds(sim.check_energy(dataclasses.replace(p, landing_site="ALT1"), model, ctx, cache))
    assert "landing_site_not_recovery" in k
    k = _kinds(sim.check_energy(dataclasses.replace(p, landing_site="ALT1", warnings=["planned_landing_at_alternate(ALT1)"]),
                                model, ctx, cache))
    assert "landing_site_not_recovery" not in k
    # D_plan > D_use
    k = _kinds(sim.check_energy(dataclasses.replace(p, energy_used_wh=1e5), model, ctx, cache))
    assert "usable_distance_exceeded" in k
    # 無預算表 → energy_budget_unavailable；legacy（enforce_energy=False）→ 空
    assert "energy_budget_unavailable" in _kinds(sim.check_energy(p, model, ctx, SimpleNamespace()))
    ctx_off = dataclasses.replace(ctx, cfg=dataclasses.replace(ctx.cfg, enforce_energy=False))
    assert sim.check_energy(_with_wps(p, [dataclasses.replace(w, energy_remaining_wh=0.0) for w in p.waypoints]),
                            model, ctx_off, cache) == []


def test_energy_coarse_to_fine_equals_pointwise(mini, monkeypatch):
    """粗取樣 + Lipschitz 細化須與全逐航點檢查給出相同違規集合（E 單調遞減之計畫；整體下移 E 使後段開始不足）。

    這個不變量講的是**取樣**的等價性，基準也是逐點的 `point_wh`；R-06 之後 `check_energy` 會在餘裕 < slack 時
    改用 `point_wh_exact` 定案（那是另一層、只會移除粗篩的假陽性），故此處把精算關掉以隔離取樣這一層。
    帶著精算的行為由 `test_energy_exact_refinement_only_removes_false_positives` 驗。
    """
    p = mini.mp.plans["A1"]
    model, ctx, cache = mini.ctx.models["A1"], mini.ctx, mini.cache
    monkeypatch.setattr(type(cache.budget), "point_wh_exact",
                        lambda self, cls, xy, z_m, th, site: math.inf)      # 關掉精算層
    reserve = model.reserve_wh()
    j_fw, j_tc = _first_index(p, Phase.ACCEL), _first_index(p, Phase.TRANS_IN)
    for shift in (560.0, 600.0, 640.0):
        wps = [dataclasses.replace(w, energy_remaining_wh=w.energy_remaining_wh - shift) for w in p.waypoints]
        brute = [jj for jj in range(j_fw, j_tc)
                 if wps[jj].energy_remaining_wh < reserve + min(cache.budget.point_wh("A", np.array([wps[jj].x_m, wps[jj].y_m]),
                                                                                       wps[jj].z_m, s_) for s_ in cache.budget.site_ids)]
        out = [o for o in sim.check_energy(_with_wps(p, wps), model, ctx, cache) if o.startswith("energy_alt_unreachable")]
        listed = [int(o.split("wp=")[1].split()[0]) for o in out if "wp=" in o]
        total = len(listed)
        for o in out:
            if "total n=" in o:
                total = int(o.split("total n=")[1].rstrip(")"))
        assert total == len(brute), (shift, total, len(brute))
        if brute:
            assert listed[0] == min(brute) and listed == brute[:len(listed)]
    assert brute, "最大下移量應產生違規"


# ---------------------------------------------------------------------------
# check_link_airspace：違規偵測（規格 §17.1 (5)）
# ---------------------------------------------------------------------------
def test_link_airspace_detects(mini):
    p, ctx = mini.mp.plans["A1"], mini.ctx
    cells = {c.cell_id: c for c in mini.mp.cells}
    ctx2 = dataclasses.replace(ctx, link_poly={**ctx.link_poly, "A": box(-500.0, -500.0, 500.0, 500.0)})
    assert "outside_link" in _kinds(sim.check_link_airspace(p, ctx2, cells=cells))
    ctx3 = dataclasses.replace(ctx, rho_sym_m={**ctx.rho_sym_m, "A": 3000.0})
    assert "beyond_rho_sym" in _kinds(sim.check_link_airspace(p, ctx3, cells=cells))
    inp = ctx.inputs
    ctx4 = dataclasses.replace(ctx, inputs=dataclasses.replace(inp, airspace=dataclasses.replace(inp.airspace,
                                                                                                    time_window_s=(0.0, 900.0))))
    assert "outside_airspace_window" in _kinds(sim.check_link_airspace(p, ctx4, cells=cells))
    cell = cells[p.waypoints[_first_index(p, Phase.OPERATE)].cell_id]
    cells_w = dict(cells)
    cells_w[cell.cell_id] = dataclasses.replace(cell, time_window_s=(0.0, 300.0))
    assert "outside_area_window" in _kinds(sim.check_link_airspace(p, ctx, cells=cells_w))
    # 禁飛區（2-D）位於作業路徑上
    ctx5 = dataclasses.replace(ctx, keepouts=None, nofly_buffered=[box(5400.0, 300.0, 5600.0, 380.0)])
    assert "nofly_intersect" in _kinds(sim.check_link_airspace(p, ctx5, cells=cells))
    # 鏈路限制關閉 → 不檢查
    ctx6 = dataclasses.replace(ctx2, cfg=dataclasses.replace(ctx.cfg, enforce_link=False))
    assert "outside_link" not in _kinds(sim.check_link_airspace(p, ctx6, cells=cells))
    assert sim.check_link_airspace(_with_wps(p, p.waypoints[:1]), ctx) == []


def test_link_airspace_keepout_chunks(mini):
    """ctx.keepouts 路徑：以 KeepoutSet 建一個禁飛區橫跨 A1 作業條帶 → keepout_intersect 並定位到該腿。"""
    from hv_dccpp.mission import NoFlyZone
    from hv_dccpp.routing import KeepoutSet
    ctx = mini.ctx
    p = mini.mp.plans["A1"]
    ring_enu = np.array([[5400.0, 300.0, 0.0], [5600.0, 300.0, 0.0], [5600.0, 380.0, 0.0], [5400.0, 380.0, 0.0]])
    ll = ctx.frame.to_wgs84(ring_enu)
    nf = NoFlyZone("NF_T", tuple((float(a), float(b)) for a, b in ll[:, :2]))
    ko = KeepoutSet.build([], [nf], ctx.frame, ctx.terrain.copy(), ctx.cfg, None, burn=False)
    ctx_k = dataclasses.replace(ctx, keepouts=ko)
    out = sim.check_link_airspace(p, ctx_k)
    assert "keepout_intersect" in _kinds(out)
    hit = [s for s in out if s.startswith("keepout_intersect") and "wp=" in s]
    j = int(hit[0].split("wp=")[1].split()[0])
    w = p.waypoints[j]
    assert w.phase is Phase.OPERATE and abs(w.y_m - 335.5) < 1.0 and 5100.0 <= w.x_m <= 5700.0   # 該腿（≤ L_max）跨越禁區
    assert sim.check_link_airspace(mini.mp.plans["B1"], ctx_k) == []


# ---------------------------------------------------------------------------
# validate：分離與失敗旗標
# ---------------------------------------------------------------------------
def test_validate_detects_conflicts_and_flags(mini):
    mp, ctx, cache, cfg = mini.mp, mini.ctx, mini.cache, mini.cfg
    a1 = mp.plans["A1"]
    ghost = dataclasses.replace(a1, uav_id="A2")             # 同一航跡 → 4D 衝突與走廊重疊
    mp2 = dataclasses.replace(mp, plans={"A1": a1, "A2": ghost})
    rep = sim.validate(mp2, ctx, cache, cfg)
    assert not rep.passed
    assert rep.conflicts and any(c.kind == "4d" for c in rep.conflicts) and any(c.kind == "corridor" for c in rep.conflicts)
    assert rep.min_sep_h_m == pytest.approx(0.0, abs=1e-6) and rep.min_sep_v_m == pytest.approx(0.0, abs=1e-6)
    assert rep.slope_ok and rep.energy_ok                      # 其餘檢查仍通過
    h, v, t, pair = sim.min_separation({"A1": a1, "B1": mp.plans["B1"]}, cfg)
    assert h > cfg.d_sep_m and math.isinf(v) and pair == ("A1", "B1") and t > 0
    assert sim.min_separation({"A1": a1}, cfg)[3] is None
    # 旗標對應
    wps = list(a1.waypoints)
    j = _first_index(a1, Phase.OPERATE) + 2
    wps[j] = dataclasses.replace(wps[j], z_m=wps[j].z_m + 200.0)
    rep2 = sim.validate(dataclasses.replace(mp, plans={"A1": _with_wps(a1, wps), "B1": mp.plans["B1"]}), ctx, cache, cfg)
    assert not rep2.passed and not rep2.slope_ok and not rep2.agl_ok and not rep2.airspace_ok
    assert rep2.energy_ok and rep2.link_ok and rep2.time_window_ok and rep2.curvature_ok
    ctx_nf = dataclasses.replace(ctx, keepouts=None, nofly_buffered=[box(5400.0, 300.0, 5600.0, 380.0)])
    rep3 = sim.validate(mp, ctx_nf, cache, cfg)
    assert not rep3.airspace_ok and not rep3.passed
    empty = sim.validate(dataclasses.replace(mp, plans={}), ctx, cache, cfg)
    assert empty.conflicts == [] and math.isinf(empty.min_sep_h_m) and not empty.passed   # 覆蓋 0


# ---------------------------------------------------------------------------
# track_simulate（規格 §17.1 (9)）
# ---------------------------------------------------------------------------
def test_track_simulate_mini(mini):
    ctx, cfg = mini.ctx, mini.cfg
    for uid, p in mini.mp.plans.items():
        tr = sim.track_simulate(p, ctx.specs[p.cls], ctx.wind, ctx.models[uid], cfg)
        assert tr.completed and tr.n_steps > 0
        assert tr.max_cross_track_m <= E_MAX_PLUS_E_XT, (uid, tr)
        assert tr.energy_dev_frac < 0.05
        assert tr.energy_plan_wh == pytest.approx(p.energy_used_wh)
        assert abs(tr.t_land_s - p.t_land_s) <= 0.02 * p.t_land_s
        assert 0 <= tr.wp_at_max_xt < len(p.waypoints)
    # 無風 → 更小偏差；只有 VTOL 航點 → 零偏差
    p = mini.mp.plans["A1"]
    tr0 = sim.track_simulate(p, ctx.specs["A"], None, ctx.models["A1"], cfg)
    assert tr0.completed and tr0.max_cross_track_m <= E_MAX_PLUS_E_XT
    trv = sim.track_simulate(_with_wps(p, p.waypoints[:1]), ctx.specs["A"], ctx.wind, ctx.models["A1"], cfg)
    assert trv.max_cross_track_m == 0.0 and trv.energy_dev_frac == 0.0
    with pytest.raises(ValueError):
        sim.track_simulate(p, ctx.specs["A"], ctx.wind, ctx.models["A1"], cfg, dt_s=0.0)


# ---------------------------------------------------------------------------
# build_validation_cache
# ---------------------------------------------------------------------------
def test_build_validation_cache_full(mini):
    cache = sim.build_validation_cache(mini.mp, mini.ctx)
    assert isinstance(cache, CostCache)
    assert set(cache.idx) == {"A", "B"}
    assert set(cache.idx["A"]) == {s for s, st in mini.mp.strips.items() if st.cls == "A"}
    assert cache.budget.site_ids and cache.budget.site_ids[0] == mini.ctx.recovery_site_id
    assert cache.R_eff["A"] == pytest.approx(200.0) and cache.R_eff["B"] == pytest.approx(120.0)
    assert mini.cache.budget.site_ids == cache.budget.site_ids


# ---------------------------------------------------------------------------
# 效能（規格 §17.2 runtime_under_1s：8 機／3 區／80 條帶，排除 DEM 初始化）
# ---------------------------------------------------------------------------
def test_runtime_under_1s(mini):
    mp, ctx, cache, cfg = mini.mp, mini.ctx, mini.cache, mini.cfg
    plans = {}
    for i in range(4):                                       # 8 機（4 × A1/B1 副本，時間錯開 → 無衝突計算負擔差異）
        for uid, p in mp.plans.items():
            nid = f"{uid[0]}{i + 1}"
            wps = [dataclasses.replace(w, t_s=w.t_s + 2000.0 * i) for w in p.waypoints]
            plans[nid] = dataclasses.replace(p, uav_id=nid, waypoints=wps, t_takeoff_s=p.t_takeoff_s + 2000.0 * i,
                                             t_arr_ptc_s=p.t_arr_ptc_s + 2000.0 * i, t_land_s=p.t_land_s + 2000.0 * i)
    big = dataclasses.replace(mp, plans=plans)
    sim.validate(big, ctx, cache, cfg)                        # 暖身
    t0 = time.perf_counter()
    rep = sim.validate(big, ctx, cache, cfg)
    dt = time.perf_counter() - t0
    # 8 機 + "separation" + "coverage" + "integrity"（此測試蓄意把同一組條帶複製 4 份 → 每條被飛 4 次，
    # R-14 的 strip_flown_twice 正確地報出來；這裡只驗鍵數與執行時間）
    assert len(rep.details) == 8 + 3
    assert not rep.integrity_ok and any(x.startswith("strip_flown_twice") for x in rep.details["integrity"])
    assert dt < 1.0, dt
    assert rep.compute_ms < 1000.0


# ---------------------------------------------------------------------------
# 基線比較與論文情境（規格 §17.1 (6)(7)）
# ---------------------------------------------------------------------------
def test_hetero_le_homogeneous_baseline(mini):
    res = sim.baselines(mini.inputs, mini.cfg, terrain_margin_m=1500.0)
    assert set(res) >= {"hetero", "all_A", "all_B", "legacy"}
    for k in ("hetero", "all_A", "all_B", "legacy"):
        assert math.isfinite(res[k]) and res[k] > 0
        assert res[f"{k}_uncovered"] == 0.0 and math.isfinite(res[f"{k}_distance_m"])
    assert res["hetero"] == pytest.approx(mini.mp.makespan_s, rel=1e-6)
    # 異構（1A+1B）≤ 最差同構基線（全 B）；全 A（2 架 A）在此平地小區更快（見 CONTRACT_DEVIATIONS）
    assert res["hetero"] <= max(res["all_A"], res["all_B"])
    assert res["all_B"] > res["hetero"]
    assert res["legacy"] <= res["all_A"] * 1.05              # 退化設定（無能量／鏈路限制、e_ext=0）不會更慢
    # 同構機隊建構：uav_id 不變、構型一致
    all_b = sim.homogeneous_fleet(mini.inputs, "B")
    assert [u.uav_id for u in all_b.fleet] == [u.uav_id for u in mini.inputs.fleet]
    assert {u.cls for u in all_b.fleet} == {"B"}
    all_a = sim.homogeneous_fleet(mini.inputs, "A")
    assert {u.cls for u in all_a.fleet} == {"A"} and all(any(h.kind == "lidar" for h in u.sensors) for u in all_a.fleet)
    with pytest.raises(ValueError):
        sim.baselines(mini.inputs, mini.cfg, terrain_margin_m=1500.0, variants=("nope",))


def test_legacy_reproduces_paper_table2():
    _inputs, cfg = build_mini_example()
    res = sim.paper_table2_check(cfg)
    assert res["n_uav"] == 3.0 and res["error"] == 0.0 and res["status_ok"] == 1.0 and res["uncovered"] == 0.0
    assert (res["expected_lo_km"], res["expected_hi_km"]) == sim.PAPER_PATH_RANGE_KM == (590.0, 650.0)
    per =[v for k, v in res.items() if k.startswith("path_km_")]
    assert len(per) == 3 and all(math.isfinite(v) and v > 0 for v in per)
    assert res["max_uav_path_km"] == pytest.approx(max(per)) and res["total_path_km"] == pytest.approx(sum(per))
    # 論文 Table 2 之每機路徑長 590–650 km；本實作三機皆自 U1 起飛且含起降走廊（見 CONTRACT_DEVIATIONS），
    # 只要求同量級（0.8×590 ≤ 每機 ≤ 1.5×650）並回報實際值 in_range_*。
    assert all(0.8 * 590.0 <= v <= 1.5 * 650.0 for v in per), per
    assert res["in_range_total"] in (0.0, 1.0) and res["in_range_max_uav"] in (0.0, 1.0)
    assert res["compute_ms"] > 0 and math.isfinite(res["makespan_s"])


# ---------------------------------------------------------------------------
# R-02：柵格中心全部落在區域外時，不得以「無樣本 ⇒ 全覆蓋」放行
# ---------------------------------------------------------------------------
def test_coverage_no_grid_sample_is_not_full_coverage(mini):
    """3000 m × 2 m 的細長區 + cov_raster_m = 5 → 唯一一列柵格中心落在多邊形外（n_in = 0）。
    舊碼在此回報 coverage = 1.0、passed = True（零架機、零條帶也「通過」）；正確答案是 0.0 / False。"""
    from hv_dccpp.mission import AreaSpec
    from hv_dccpp.scenario import enu_ring_to_wgs84, scenario_frame
    fr = scenario_frame(mini.inputs)
    ring = [(4000.0, -1.0), (7000.0, -1.0), (7000.0, 1.0), (4000.0, 1.0)]
    inp = dataclasses.replace(mini.inputs, areas=(AreaSpec("Z1", enu_ring_to_wgs84(fr, ring), sensor_req="lidar",
                                                           priority=1.0, payload_req_id="default"),))
    ctx = build_context(inp, mini.cfg, terrain_margin_m=1500.0)
    empty = dataclasses.replace(mini.mp, plans={}, strips={}, cells=[])
    for raster in (5.0, 2.0, 1.0):
        cfg = dataclasses.replace(mini.cfg, cov_raster_m=raster)
        for rep in sim.coverage_raster(empty, ctx, cfg):
            assert rep.coverage_by_cls["lidar"] == 0.0, (raster, rep.coverage_by_cls)
            assert not rep.passed
            assert rep.uncovered_cells_xy.shape[0] >= 1


# ---------------------------------------------------------------------------
# R-14：任務完整性（OPER 必須 sensor_on；每條條帶恰好飛一次）
# ---------------------------------------------------------------------------
def test_mission_integrity_clean_plan(mini, report):
    assert sim.check_mission_integrity(mini.mp) == []
    assert report.integrity_ok and report.separation_ok


def test_mission_integrity_catches_sensor_off_and_duplicates(mini):
    mp = mini.mp
    uid = next(u for u, p in mp.plans.items() if any(w.phase is Phase.OPERATE for w in p.waypoints))
    p = mp.plans[uid]
    j = next(j for j, w in enumerate(p.waypoints) if w.phase is Phase.OPERATE)
    wps = list(p.waypoints)
    wps[j] = dataclasses.replace(wps[j], sensor_on=False)
    bad = dataclasses.replace(mp, plans={**mp.plans, uid: dataclasses.replace(p, waypoints=wps)})
    out = sim.check_mission_integrity(bad)
    assert any(x.startswith(f"oper_sensor_off({uid}@{j})") for x in out), out
    # 條帶完全沒飛
    dropped = dataclasses.replace(mp, plans={u: q for u, q in mp.plans.items() if u != uid})
    out2 = sim.check_mission_integrity(dropped)
    assert any(x.startswith("strip_not_flown(") for x in out2), out2
    # 同一條帶被兩架飛
    other = next(u for u in mp.plans if u != uid)
    dup = dataclasses.replace(mp, plans={**mp.plans, other: dataclasses.replace(mp.plans[other], waypoints=list(p.waypoints))})
    out3 = sim.check_mission_integrity(dup)
    assert any(x.startswith("strip_flown_twice(") for x in out3), out3


def test_validate_fails_when_separation_cannot_run(mini):
    """R-13：check_separation 丟例外時，separation_ok 必須為 False 且 passed 為 False
    （舊碼只把訊息塞進 details["separation"]，conflicts 仍為空 → `not conflicts` 被滿足）。"""
    mp = mini.mp
    uid = next(iter(mp.plans))
    p = mp.plans[uid]
    wps = list(p.waypoints)
    wps[0] = dataclasses.replace(wps[0], t_s=wps[5].t_s + 500.0)          # 時間倒退
    bad = dataclasses.replace(mp, plans={**mp.plans, uid: dataclasses.replace(p, waypoints=wps)})
    rep = sim.validate(bad, mini.ctx, mini.cache, mini.cfg)
    assert not rep.separation_ok and not rep.passed
    assert any(x.startswith("check_separation_failed") for x in rep.details["separation"])


def test_validate_accepts_t_sync(mini, report):
    """R-12：validate 可指定 σ_t 的基準時刻；初始計畫 t_sync=0 與 t_sync=t_takeoff 都不應改變乾淨計畫的結論。"""
    rep = sim.validate(mini.mp, mini.ctx, mini.cache, mini.cfg, t_sync_s=0.0)
    assert rep.passed == report.passed
    rep2 = sim.validate(mini.mp, mini.ctx, mini.cache, mini.cfg, t_sync_s=600.0)
    assert rep2.conflicts == [] and rep2.passed


def test_energy_exact_refinement_only_removes_false_positives(mini):
    """R-06：`point_wh` 是粗篩、最壞低估量由 `point_wh_slack_wh` 界定；`check_energy` 在餘裕 < slack 時改以
    `point_wh_exact`（AVOID+ENVELOPE）定案。精算只會移除粗篩的假陽性，不會漏掉粗篩已判定的真違規之外的東西。"""
    p = mini.mp.plans["A1"]
    model, ctx, cache = mini.ctx.models["A1"], mini.ctx, mini.cache
    bud = cache.budget
    assert bud.point_wh_slack_wh("A") > 0.0
    j_fw, j_tc = _first_index(p, Phase.ACCEL), _first_index(p, Phase.TRANS_IN)
    reserve = model.reserve_wh()
    shift = 600.0
    wps = [dataclasses.replace(w, energy_remaining_wh=w.energy_remaining_wh - shift) for w in p.waypoints]
    screen = {jj for jj in range(j_fw, j_tc)
              if wps[jj].energy_remaining_wh < reserve + min(bud.point_wh("A", np.array([wps[jj].x_m, wps[jj].y_m]),
                                                                         wps[jj].z_m, s_) for s_ in bud.site_ids)}
    out = [o for o in sim.check_energy(_with_wps(p, wps), model, ctx, cache) if o.startswith("energy_alt_unreachable")]
    listed = {int(o.split("wp=")[1].split()[0]) for o in out if "wp=" in o}
    assert listed <= screen, sorted(listed - screen)


# ---------------------------------------------------------------------------
# 審查後新增（R-04）：both 區由「同機雙感測」單一構型服務時，兩種感測需求都必須被真的覆蓋
# ---------------------------------------------------------------------------
def test_both_area_single_dual_sensor_class_covers_lidar_and_rgb():
    """規格 §6.2「取較窄者鋪一套」：同時掛 LiDAR + RGB 的同構機隊飛 ``both`` 區時，

    (a) 條帶以較窄之間距鋪設（`platform.CombinedSensor`），(b) `coverage_raster` 對 lidar 與 rgb **各**算一套
    足跡並雙雙通過，(c) 不再出現 ``#sensor2 → both_second_sensor_unserved``。
    這是審查 R-04 blocker（all_A 基線 Z2 的 rgb 覆蓋率 0.000 卻被當成完整任務）的正面測試。
    """
    from hv_dccpp.platform import CombinedSensor

    inputs, cfg = build_mini_example()
    inputs = dataclasses.replace(inputs, areas=tuple(dataclasses.replace(a, sensor_req="both") for a in inputs.areas))
    inputs = sim.homogeneous_fleet(inputs, "A")            # 8 架同構 A，皆掛 LiDAR + 相機
    ctx = build_context(inputs, cfg)
    assert isinstance(ctx.sensors[("A", "default")], CombinedSensor)

    plan = plan_mission(inputs, cfg)
    assert plan.uncovered == []
    assert not any("sensor2" in str(k) for k, _r in plan.uncovered)
    reps = sim.coverage_raster(plan, ctx, cfg)
    assert reps and all(r.passed for r in reps)
    for r in reps:
        assert sorted(r.cls_required) == ["lidar", "rgb"]
        assert r.coverage_by_cls["lidar"] >= sim.COVERAGE_PASS
        assert r.coverage_by_cls["rgb"] >= sim.COVERAGE_PASS


def test_subsensors_helper():
    """`sim.subsensors`：CombinedSensor → 兩個子感測器；一般感測器 → 自身。"""
    inputs, cfg = build_mini_example()
    ctx = build_context(inputs, cfg)
    single = ctx.sensors[("A", "default")]
    assert sim.subsensors(single) == (single,)
    dual = sim.homogeneous_fleet(inputs, "A")
    ctx2 = build_context(dual, cfg)
    combo = ctx2.sensors[("A", "default")]
    assert len(sim.subsensors(combo)) == 2
    assert {getattr(s, "kind", None) for s in sim.subsensors(combo)} == {"lidar", "rgb"}
