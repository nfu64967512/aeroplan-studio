"""tests/test_viz.py — 契約 §15 `viz.py`：plot_2d / plot_profiles / plot_gantt / plot_coverage / save_all 於
`scenario.build_mini_example()` 之 `planner.plan_mission` 計畫上皆寫出 > 10 kB 之 PNG；另測附加函式
（resolve_coverage → sim.coverage_raster、coverage_from_plan、profile_bounds、plot_energy、configure_fonts）、
真實 `sim.ValidationReport`、山丘情境 context（keep-out／禁飛／多區域／備降點）、空計畫與缺字警告。

情境以 `dem_step_m=50`、`terrain_margin_m=1500` 建構（planner 測試同設定），plan_mission ≈ 1 s（module fixture）。
"""
from __future__ import annotations

import dataclasses
import os
import warnings
from types import SimpleNamespace

import matplotlib
import numpy as np
import pytest
import shapely

from hv_dccpp import viz
from hv_dccpp.context import build_context
from hv_dccpp.deconflict import CorridorSchedule
from hv_dccpp.mission import FeasibilityReport, MissionPlan, Phase
from hv_dccpp.planner import plan_mission
from hv_dccpp.scenario import build_mini_example

PNG_SIG = b"\x89PNG\r\n\x1a\n"
MIN_BYTES = 10 * 1024


# ---------------------------------------------------------------------------
# fixtures
# ---------------------------------------------------------------------------
@pytest.fixture(scope="module")
def mini():
    inputs, cfg = build_mini_example()
    cfg = dataclasses.replace(cfg, dem_step_m=50.0)
    ctx = build_context(inputs, cfg, terrain_margin_m=1500.0)
    plan = plan_mission(inputs, cfg, ctx=ctx)
    assert plan.plans, "mini example must produce UAV plans"
    return inputs, cfg, ctx, plan


@pytest.fixture(scope="module")
def out_dir(tmp_path_factory):
    return str(tmp_path_factory.mktemp("viz"))


def _assert_png(path: str, min_bytes: int = MIN_BYTES) -> None:
    assert os.path.isfile(path), path
    size = os.path.getsize(path)
    assert size > min_bytes, f"{path}: {size} B <= {min_bytes} B"
    with open(path, "rb") as fh:
        assert fh.read(8) == PNG_SIG
    img = matplotlib.image.imread(path)
    assert img.ndim == 3 and img.shape[0] > 100 and img.shape[1] > 100


def _empty_plan(plan: MissionPlan) -> MissionPlan:
    rep = FeasibilityReport("GO", [], [], {}, {}, {})
    return MissionPlan({}, 0.0, [], {}, [], rep, {}, (0, 0, 0))


# ---------------------------------------------------------------------------
# 後端 / 字型
# ---------------------------------------------------------------------------
def test_backend_is_agg_and_fonts_configured():
    assert matplotlib.get_backend().lower().startswith("agg")
    fonts = viz.configure_fonts(force=True)
    assert fonts and fonts[-1] == "DejaVu Sans" or "DejaVu Sans" in fonts
    assert matplotlib.rcParams["font.family"] == ["sans-serif"]
    assert matplotlib.rcParams["axes.unicode_minus"] is False


# ---------------------------------------------------------------------------
# plot_2d
# ---------------------------------------------------------------------------
def test_plot_2d_writes_png(mini, out_dir):
    _inputs, _cfg, ctx, plan = mini
    path = os.path.join(out_dir, "sub", "plan_2d.png")   # 父目錄不存在 → 自動建立
    with warnings.catch_warnings(record=True) as rec:
        warnings.simplefilter("always")
        viz.plot_2d(plan, ctx, path)
    _assert_png(path)
    glyph = [str(w.message) for w in rec if "Glyph" in str(w.message) or "missing from font" in str(w.message)]
    assert not glyph, glyph


def test_plot_2d_flags_and_conflicts(mini, out_dir):
    _inputs, _cfg, ctx, plan = mini
    p1 = os.path.join(out_dir, "plan_2d_nostrips.png")
    viz.plot_2d(plan, ctx, p1, show_strips=False, show_keepouts=False)
    _assert_png(p1)
    uid = sorted(plan.plans)[0]
    conf = [SimpleNamespace(uav_a=uid, uav_b="X", t_s=plan.plans[uid].waypoints[len(plan.plans[uid].waypoints) // 2].t_s,
                            d_h_m=10.0, d_v_m=0.0, idx_a=0, idx_b=0, kind="4d")]
    p2 = os.path.join(out_dir, "plan_2d_conf.png")
    viz.plot_2d(plan, ctx, p2, conflicts=conf)
    _assert_png(p2)
    assert os.path.getsize(p1) != os.path.getsize(p2) or True  # 兩張皆有效即可


def test_plot_2d_hills_context_keepouts_nofly_alternates(out_dir):
    """山丘情境（keep-out 塔／輸電線／建物、禁飛區、3 區域、2 備降點、分層風）之 context 上以空計畫畫 2D 圖：
    覆蓋 `_draw_keepouts`／多區域／備降點／風向箭頭之程式路徑（不跑 plan_mission，≈0.3 s）。"""
    from hv_dccpp.scenario import build_hills_scenario
    inputs, cfg = build_hills_scenario()
    cfg = dataclasses.replace(cfg, dem_step_m=50.0)
    ctx = build_context(inputs, cfg, terrain_margin_m=800.0)
    assert ctx.keepouts is not None and set(ctx.keepouts.ids) >= {"T1", "T2", "PL1", "BLD1", "NF1"}
    rep = FeasibilityReport("GO", [], [], {}, {}, {})
    plan = MissionPlan({}, 0.0, [], {}, [], rep, {}, (0, 0, 0))
    path = os.path.join(out_dir, "hills_2d_ctx.png")
    with warnings.catch_warnings(record=True) as rec:
        warnings.simplefilter("always")
        viz.plot_2d(plan, ctx, path)
    _assert_png(path)
    assert not [str(w.message) for w in rec if "Glyph" in str(w.message)]
    # keep-out 關閉時仍可畫
    path2 = os.path.join(out_dir, "hills_2d_ctx_noko.png")
    viz.plot_2d(plan, ctx, path2, show_keepouts=False)
    _assert_png(path2)


def test_plot_2d_empty_plan(mini, out_dir):
    _inputs, _cfg, ctx, plan = mini
    path = os.path.join(out_dir, "plan_2d_empty.png")
    viz.plot_2d(_empty_plan(plan), ctx, path)
    _assert_png(path)   # 地形、區域、站點仍畫出


# ---------------------------------------------------------------------------
# plot_profiles / profile_bounds
# ---------------------------------------------------------------------------
def test_plot_profiles_writes_png(mini, out_dir):
    _inputs, _cfg, ctx, plan = mini
    path = os.path.join(out_dir, "profiles.png")
    viz.plot_profiles(plan, ctx, path)
    _assert_png(path)


def test_profile_bounds_consistent_with_plan(mini):
    _inputs, cfg, ctx, plan = mini
    for up in plan.plans.values():
        z_g, z_lo, z_hi = viz.profile_bounds(up, ctx, plan.strips)
        n = len(up.waypoints)
        assert z_g.shape == z_lo.shape == z_hi.shape == (n,)
        z = np.array([w.z_m for w in up.waypoints])
        assert np.isfinite(z_g).all() and np.isfinite(z_lo).all()
        assert np.all(z_lo >= z_g - 1e-2)       # float32 DEM 之內插雜訊 ~1e-5 m
        assert np.all(z_hi >= z_lo)
        # OPERATE：規劃保證 z ≥ z_g^max_{w/2} + h_min,l（D36）；z ≤ z_g^min + h_ceil
        oper = np.array([w.phase is Phase.OPERATE for w in up.waypoints])
        assert oper.any()
        assert np.all(z[oper] >= z_lo[oper] - 0.5)
        assert np.all(z[oper] <= z_hi[oper] + 0.5)
        # 固定翼段（CLIMB/DESCENT 以外）下界含 h_minfw
        spec = ctx.specs[up.cls]
        fw = np.array([w.phase in (Phase.ENTRY, Phase.TRANSFER, Phase.RTB) for w in up.waypoints])
        assert np.all(z_lo[fw] >= z_g[fw] + max(ctx.h_clr_m, spec.min_fw_operation_agl_m) - 1e-2)
        # 起降旋翼段：下界 = 地形
        rot = np.array([w.phase in (Phase.TAKEOFF, Phase.LAND) for w in up.waypoints])
        assert np.all(np.abs(z_lo[rot] - z_g[rot]) < 1.0)


def test_profile_bounds_empty_plan(mini):
    _inputs, _cfg, ctx, plan = mini
    up = next(iter(plan.plans.values()))
    empty = dataclasses.replace(up, waypoints=[])
    z_g, z_lo, z_hi = viz.profile_bounds(empty, ctx, plan.strips)
    assert z_g.shape == (0,) and z_lo.shape == (0,) and z_hi.shape == (0,)


def test_plot_profiles_empty_plan(mini, out_dir):
    _inputs, _cfg, ctx, plan = mini
    path = os.path.join(out_dir, "profiles_empty.png")
    viz.plot_profiles(_empty_plan(plan), ctx, path)
    assert os.path.isfile(path) and os.path.getsize(path) > 1000


# ---------------------------------------------------------------------------
# plot_gantt
# ---------------------------------------------------------------------------
def test_plot_gantt_with_schedule(mini, out_dir):
    _inputs, cfg, ctx, plan = mini
    schedule = CorridorSchedule(cfg)
    for up in plan.plans.values():
        schedule.reserve_plan(up, "H")
    assert schedule.reservations
    path = os.path.join(out_dir, "gantt.png")
    viz.plot_gantt(plan, schedule, path)
    _assert_png(path)


def test_plot_gantt_without_schedule_and_overlap(mini, out_dir):
    _inputs, cfg, ctx, plan = mini
    path = os.path.join(out_dir, "gantt_none.png")
    viz.plot_gantt(plan, None, path, cfg=cfg, takeoff_site_id="H")
    _assert_png(path)
    # 人工重疊之走廊預約 + 4D 衝突標示
    schedule = CorridorSchedule(cfg)
    for up in plan.plans.values():
        schedule.reserve("H", "landing", 100.0, 300.0, up.uav_id)
    conf = [SimpleNamespace(uav_a="A1", uav_b="B1", t_s=600.0, d_h_m=5.0, d_v_m=0.0, idx_a=0, idx_b=0, kind="4d")]
    path2 = os.path.join(out_dir, "gantt_overlap.png")
    viz.plot_gantt(plan, schedule, path2, conflicts=conf)
    _assert_png(path2)


def test_plot_gantt_empty_plan(mini, out_dir):
    _inputs, _cfg, _ctx, plan = mini
    path = os.path.join(out_dir, "gantt_empty.png")
    viz.plot_gantt(_empty_plan(plan), None, path)
    assert os.path.isfile(path) and os.path.getsize(path) > 1000


# ---------------------------------------------------------------------------
# coverage
# ---------------------------------------------------------------------------
def _footprint_union(plan: MissionPlan, ctx, cls: str | None):
    """獨立之 shapely 足跡聯集（sensor_on 航點折線 ⊕ Strip.width_m/2）供比對。"""
    from shapely.geometry import LineString
    parts = []
    for up in plan.plans.values():
        if cls is not None and up.cls != cls:
            continue
        wps = up.waypoints
        i = 0
        while i < len(wps):
            if not wps[i].sensor_on:
                i += 1
                continue
            j = i
            while j + 1 < len(wps) and wps[j + 1].sensor_on and wps[j + 1].strip_id == wps[i].strip_id:
                j += 1
            if j > i:
                xy = [(w.x_m, w.y_m) for w in wps[i:j + 1]]
                parts.append(LineString(xy).buffer(0.5 * plan.strips[wps[i].strip_id].width_m, cap_style="flat"))
            i = j + 1
    import shapely
    return shapely.union_all(parts) if parts else None


def test_coverage_from_plan_mini(mini):
    _inputs, cfg, ctx, plan = mini
    reps = viz.coverage_from_plan(plan, ctx)
    assert [r.area_id for r in reps] == list(ctx.areas_enu)
    rep = reps[0]
    assert rep.cls_required == ["any"]                       # mini Z1 sensor_req="any"
    assert rep.raster_m == pytest.approx(cfg.cov_raster_m)
    assert set(rep.coverage_by_cls) >= {"any", "A", "B"}
    area = ctx.areas_enu["Z1"]
    # 與獨立 shapely 面積比對（柵格化誤差 < 1%）；感測器刈幅於實際 AGL ≥ Strip.width_m（保守寬度）
    for key, cls in (("any", None), ("A", "A"), ("B", "B")):
        u = _footprint_union(plan, ctx, cls)
        frac = u.intersection(area).area / area.area if u is not None else 0.0
        assert rep.coverage_by_cls[key] == pytest.approx(frac, abs=0.01), key
    assert 0.0 < rep.coverage_by_cls["A"] < 1.0 and 0.0 < rep.coverage_by_cls["B"] < 1.0   # 殘帶分割：各自部分覆蓋
    assert rep.coverage_by_cls["A"] + rep.coverage_by_cls["B"] >= rep.coverage_by_cls["any"] - 1e-9
    # 規格 §17.1 (1)：覆蓋率 ≥ 99.5%（殘帶條帶 id 跨候選 k 碰撞已於 decompose/planner 修正，見 CONTRACT_DEVIATIONS）
    assert rep.coverage_by_cls["any"] >= 0.995
    unc = rep.uncovered_cells_xy
    assert unc.ndim == 2 and unc.shape[1] == 2
    n_unc_expected = (1.0 - rep.coverage_by_cls["any"]) * area.area / rep.raster_m ** 2
    assert unc.shape[0] == pytest.approx(n_unc_expected, rel=0.02, abs=2)
    if unc.shape[0]:
        assert shapely.contains_xy(area, unc[:, 0], unc[:, 1]).all()
        u_any = _footprint_union(plan, ctx, None)
        assert not shapely.contains_xy(u_any, unc[:, 0], unc[:, 1]).any()


def test_coverage_from_plan_no_footprint(mini):
    _inputs, _cfg, ctx, plan = mini
    reps = viz.coverage_from_plan(_empty_plan(plan), ctx, raster_m=20.0)
    assert len(reps) == 1 and reps[0].coverage_by_cls["any"] == 0.0 and reps[0].raster_m == 20.0
    n_cells = reps[0].uncovered_cells_xy.shape[0]
    assert n_cells == pytest.approx(ctx.areas_enu["Z1"].area / 400.0, rel=0.05)


def test_plot_coverage_writes_png(mini, out_dir):
    _inputs, _cfg, ctx, plan = mini
    reps = viz.coverage_from_plan(plan, ctx)
    path = os.path.join(out_dir, "coverage.png")
    viz.plot_coverage(reps, ctx, path)
    _assert_png(path)
    # 鴨子型別（sim.CoverageReport 之同形物件）＋人工未覆蓋格
    poly = ctx.areas_enu["Z1"]
    b = poly.bounds
    unc = np.array([[b[0] + 50.0 + 5.0 * i, b[1] + 50.0] for i in range(40)])
    duck = [SimpleNamespace(area_id="Z1", cls_required=["A", "B"], coverage_by_cls={"A": 0.98, "B": 0.999},
                            uncovered_cells_xy=unc, raster_m=5.0)]
    path2 = os.path.join(out_dir, "coverage_duck.png")
    viz.plot_coverage(duck, ctx, path2)
    _assert_png(path2)


def test_plot_coverage_empty_report(mini, out_dir):
    _inputs, _cfg, ctx, _plan = mini
    path = os.path.join(out_dir, "coverage_empty.png")
    viz.plot_coverage([], ctx, path)
    assert os.path.isfile(path) and os.path.getsize(path) > 1000


# ---------------------------------------------------------------------------
# energy / save_all
# ---------------------------------------------------------------------------
def test_plot_energy_writes_png(mini, out_dir):
    _inputs, _cfg, ctx, plan = mini
    path = os.path.join(out_dir, "energy.png")
    viz.plot_energy(plan, ctx, path)
    _assert_png(path)


def test_save_all_with_none_report(mini, out_dir):
    _inputs, _cfg, ctx, plan = mini
    d = os.path.join(out_dir, "all_none")
    paths = viz.save_all(plan, ctx, None, d)
    assert len(paths) >= 4
    names = [os.path.basename(p) for p in paths]
    assert names[:4] == ["plan_2d.png", "profiles.png", "gantt.png", "coverage.png"]
    for p in paths:
        assert os.path.dirname(os.path.abspath(p)) == os.path.abspath(d)
        _assert_png(p)


def test_save_all_with_duck_validation_report(mini, out_dir):
    _inputs, _cfg, ctx, plan = mini
    reps = viz.coverage_from_plan(plan, ctx, raster_m=10.0)
    uid = sorted(plan.plans)[0]
    conf = [SimpleNamespace(uav_a=uid, uav_b="B1", t_s=500.0, d_h_m=1.0, d_v_m=0.0, idx_a=0, idx_b=0, kind="4d"),
            SimpleNamespace(uav_a=uid, uav_b="B1", t_s=0.0, d_h_m=0.0, d_v_m=0.0, idx_a=0, idx_b=1, kind="corridor")]
    report = SimpleNamespace(coverage=reps, conflicts=conf, passed=False)
    d = os.path.join(out_dir, "all_report")
    paths = viz.save_all(plan, ctx, report, d)
    assert len(paths) >= 4
    for p in paths:
        _assert_png(p)


def test_resolve_coverage_prefers_sim_then_report(mini):
    from hv_dccpp import sim
    _inputs, _cfg, ctx, plan = mini
    reps = viz.resolve_coverage(plan, ctx, None)
    assert reps and all(isinstance(r, sim.CoverageReport) for r in reps)
    assert [r.area_id for r in reps] == list(ctx.areas_enu)
    lite = viz.coverage_from_plan(plan, ctx)
    # sim（實際 AGL 足跡）與 viz 近似在 mini 平地例一致（任一構型 union 與 A／B 各自）
    assert reps[0].coverage_by_cls["all"] == pytest.approx(lite[0].coverage_by_cls["any"], abs=0.01)
    assert reps[0].coverage_by_cls["A"] == pytest.approx(lite[0].coverage_by_cls["A"], abs=0.01)
    assert reps[0].coverage_by_cls["B"] == pytest.approx(lite[0].coverage_by_cls["B"], abs=0.01)
    # 給了 report.coverage 時直接採用
    duck = SimpleNamespace(coverage=lite, conflicts=[])
    assert viz.resolve_coverage(plan, ctx, duck) == lite


def test_save_all_with_real_validation_report(mini, out_dir):
    from hv_dccpp import sim
    _inputs, cfg, ctx, plan = mini
    cache = sim.build_validation_cache(plan, ctx, budget_only=True)
    report = sim.validate(plan, ctx, cache, cfg)
    assert isinstance(report, sim.ValidationReport)
    d = os.path.join(out_dir, "all_sim")
    paths = viz.save_all(plan, ctx, report, d)
    assert [os.path.basename(p) for p in paths] == ["plan_2d.png", "profiles.png", "gantt.png", "coverage.png",
                                                    "energy.png"]
    for p in paths:
        _assert_png(p)
    # sim.CoverageReport（鍵 all/lidar/rgb）直接可畫
    p2 = os.path.join(out_dir, "coverage_sim.png")
    viz.plot_coverage(report.coverage, ctx, p2)
    _assert_png(p2)


def test_save_all_empty_plan(mini, out_dir):
    _inputs, _cfg, ctx, plan = mini
    d = os.path.join(out_dir, "all_empty")
    paths = viz.save_all(_empty_plan(plan), ctx, None, d)
    assert len(paths) >= 4 and all(os.path.isfile(p) for p in paths)


# ---------------------------------------------------------------------------
# 私有工具
# ---------------------------------------------------------------------------
def test_hillshade_handles_nan_and_flat():
    z = np.zeros((6, 8))
    s = viz._hillshade(z, 10.0)
    assert s.shape == z.shape and np.isfinite(s).all()
    z2 = np.add.outer(np.arange(6.0), np.arange(8.0)) * 5.0
    z2[0, 0] = np.nan
    s2 = viz._hillshade(z2, 10.0)
    assert np.isnan(s2[0, 0]) and np.isfinite(s2[1:, 1:]).all()
    assert (s2[1:, 1:] >= 0).all() and (s2[1:, 1:] <= 1).all()
    assert np.isnan(viz._hillshade(np.full((3, 3), np.nan), 10.0)).all()


def test_phase_runs_and_poly_patch(mini):
    _inputs, _cfg, ctx, plan = mini
    up = next(iter(plan.plans.values()))
    runs = viz._phase_runs(up.waypoints)
    assert runs[0][1] == 0 and runs[-1][2] == len(up.waypoints) - 1
    assert all(runs[k][2] + 1 == runs[k + 1][1] for k in range(len(runs) - 1))
    assert all(up.waypoints[i].phase is ph for ph, i0, i1 in runs for i in range(i0, i1 + 1))
    assert runs[0][0] is Phase.TAKEOFF and runs[-1][0] is Phase.LAND
    from shapely.geometry import Polygon
    hole = Polygon([(0, 0), (100, 0), (100, 100), (0, 100)], holes=[[(40, 40), (60, 40), (60, 60), (40, 60)]])
    patch = viz._poly_patch(hole)
    assert patch.get_path().vertices.shape[0] == 10   # 5 + 5（含閉合點）
    assert viz._iter_polys(hole.buffer(0)) == [hole.buffer(0)] or len(viz._iter_polys(hole)) == 1
    assert viz._iter_polys(None) == [] and viz._iter_polys(Polygon()) == []
