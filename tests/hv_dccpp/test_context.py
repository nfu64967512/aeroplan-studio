"""tests/test_context.py — context.build_context / MissionContext / DecomposeContext（契約 §6A）。

情境（inline，不依賴 scenario.py）：平地 100 m MSL、home (23.95°N, 120.90°E)、備降點 ENU (2000, 1500)、
矩形區域 ENU (4000,−600)–(7000,600)、電塔（點，40 m AGL）＋輸電線（線，頂 130 m MSL）、單層西風 5 m/s、
A1（PIONEER + LiDAR）＋ B1（HERO + 相機）、C2 25 km、空域 120/豁免 200、淨空 36。
"""
from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest
import shapely
from shapely.geometry import Point

from hv_dccpp import context as C
from hv_dccpp.context import (
    DecomposeContext,
    MissionContext,
    ObstacleEnu,
    apply_platform_overrides,
    build_context,
)
from hv_dccpp.geo import DemGrid, GeoFrame
from hv_dccpp.mission import (
    Airspace,
    AreaSpec,
    C2Link,
    MissionInputError,
    MissionInputs,
    NoFlyZone,
    Obstacle,
    PayloadRequirement,
    PlannerConfig,
    Site,
    SortieEnergy,
    UAVInstance,
    WeatherObs,
    WeatherThresholds,
    WindLayer,
)
from hv_dccpp.platform import PLATFORM_HERO, PLATFORM_PIONEER, CameraHardware, LidarHardware

A = PLATFORM_PIONEER
B = PLATFORM_HERO

LAT0, LON0, Z0 = 23.95, 120.90, 100.0
REF = GeoFrame(LAT0, LON0, Z0)          # 與 build_context 之框架相同（home 由 DEM 取 100 m）
CFG = PlannerConfig(dem_step_m=50.0)


def _wgs(x: float, y: float) -> tuple[float, float]:
    lat, lon, _ = REF.to_wgs84(np.array([x, y, 0.0]))[0]
    return (float(lat), float(lon))


def _ring(*xy: tuple[float, float]) -> tuple[tuple[float, float], ...]:
    return tuple(_wgs(x, y) for x, y in xy)


def _dem(sigma: float | None = 2.0, is_dsm: bool = False, n: int = 25) -> DemGrid:
    return DemGrid(23.85, 120.78, 0.01, 0.01, np.full((n, n), Z0, dtype=np.float32), sigma_z_m=sigma, is_dsm=is_dsm)


AREA_RING = _ring((4000, -600), (7000, -600), (7000, 600), (4000, 600))
NOFLY_RING = _ring((800, -1200), (1200, -1200), (1000, -800))
HOME = Site("H", LAT0, LON0, kind="home")
ALT1 = Site("ALT1", *_wgs(2000, 1500), approach_heading_deg=270.0)
TOWER = Obstacle("T1", "point", (_wgs(2500, 300),), None, 40.0, category="tower")
LINE = Obstacle("PL1", "line", (_wgs(3000, -1000), _wgs(3000, 1000)), 130.0, None, category="powerline",
                keepout_buffer_m=60.0)


def _fleet() -> tuple[UAVInstance, ...]:
    a1 = UAVInstance("A1", A, (LidarHardware(),),
                     SortieEnergy("A1", 24.6, payload_kg=5.0, available_wh=1350, pack_nominal_wh=1500))
    b1 = UAVInstance("B1", B, (CameraHardware(),),
                     SortieEnergy("B1", 6.65, payload_kg=0.6, available_wh=380, pack_nominal_wh=420, battery_temp_c=22.0))
    return (a1, b1)


def make_inputs(**kw) -> MissionInputs:
    d = dict(
        home=HOME, alternates=(ALT1,),
        areas=(AreaSpec("Z1", AREA_RING, sensor_req="any", payload_req_id="default"),),
        dem=_dem(), obstacles=(TOWER, LINE),
        wind=(WindLayer(0.0, 1000.0, 5.0, 270.0, gust_mps=7.0),),
        fleet=_fleet(),
        payload_reqs={
            "default": PayloadRequirement("default", "lidar", density_pt_m2=20, side_overlap=0.25, h_target_agl_m=160),
            "default_rgb": PayloadRequirement("default_rgb", "rgb", gsd_required_m=0.05, side_overlap=0.30,
                                              forward_overlap=0.70, h_target_agl_m=100),
        },
        c2=C2Link(25.0, None, HOME, {}),
        airspace=Airspace(max_agl_m=120.0, waiver_max_agl_m=200.0, terrain_clearance_m=36.0, time_window_s=(0.0, 7200.0),
                          no_fly=(NoFlyZone("NF1", NOFLY_RING, active_window_s=(0.0, 600.0)),)),
        weather_thresholds=WeatherThresholds(), weather_obs=WeatherObs(5, 7, 270, 8000, 0, 25),
    )
    d.update(kw)
    return MissionInputs(**d)


@pytest.fixture(scope="module")
def ctx() -> MissionContext:
    return build_context(make_inputs(), CFG)


# ---------------------------------------------------------------------------
# 1. 基本組裝、座標框架、站點 ENU
# ---------------------------------------------------------------------------

def test_build_context_types_and_frame(ctx: MissionContext):
    assert isinstance(ctx, MissionContext)
    assert ctx.cfg is CFG and ctx.inputs.home is HOME
    assert ctx.frame.lat0_deg == LAT0 and ctx.frame.lon0_deg == LON0 and ctx.frame.h0_msl_m == pytest.approx(Z0)
    assert set(ctx.specs) == {"A", "B"} and ctx.specs["A"].cls == "A" and ctx.specs["B"].cls == "B"
    assert ctx.recovery_site_id == "H" and ctx.t_window == (0.0, 7200.0)


def test_sites_enu_exact(ctx: MissionContext):
    xy_h, z_h = ctx.sites_enu["H"]
    assert np.allclose(xy_h, 0.0, atol=1e-6) and abs(z_h) < 1e-6
    assert np.allclose(ctx.home_xy, 0.0, atol=1e-6) and ctx.z_ground_home_m == pytest.approx(0.0, abs=1e-6)
    xy_a, z_a = ctx.sites_enu["ALT1"]
    assert np.allclose(xy_a, [2000.0, 1500.0], atol=0.05)
    # 平地在 2.5 km 處之曲率下垂 ≈ d²/(2R) ≈ 0.49 m
    assert -1.0 < z_a < 0.0 and z_a == pytest.approx(-2500.0**2 / (2 * 6.37e6), abs=0.1)
    assert ctx.terrain.z_at(xy_a.reshape(1, 2))[0] == pytest.approx(z_a, abs=0.05)
    assert np.allclose(ctx.ground_station_xy, 0.0, atol=1e-6)


def test_site_agl_and_msl_altitude():
    alt_agl = dataclasses.replace(ALT1, alt_agl_m=5.0)
    alt_msl = Site("ALT2", *_wgs(-1500, 500), alt_msl_m=120.0)
    ctx2 = build_context(make_inputs(alternates=(alt_agl, alt_msl)), CFG)
    xy, z = ctx2.sites_enu["ALT1"]
    z_ground = ctx2.terrain.z_at(xy.reshape(1, 2))[0]
    assert z == pytest.approx(z_ground + 5.0, abs=0.05)
    xy2, z2 = ctx2.sites_enu["ALT2"]
    assert z2 == pytest.approx(ctx2.frame.msl_to_enu_z(xy2.reshape(1, 2), 120.0)[0], abs=1e-6)
    assert 19.0 < z2 < 20.0                      # 120 − 100 − 曲率


def test_home_altitude_from_agl_sets_frame_origin():
    home = Site("H", LAT0, LON0, alt_agl_m=10.0, kind="home")
    ctx2 = build_context(make_inputs(home=home), CFG)
    assert ctx2.frame.h0_msl_m == pytest.approx(110.0)
    assert ctx2.sites_enu["H"][1] == pytest.approx(0.0, abs=1e-6)
    assert ctx2.terrain.z_at(np.zeros((1, 2)))[0] == pytest.approx(-10.0, abs=0.05)   # 地面在起降墊下方 10 m
    assert ctx2.wind.z_ground_home_m == pytest.approx(0.0, abs=1e-6)


def test_site_outside_dem_is_error():
    far = Site("FAR", 25.5, 122.0)
    with pytest.raises(MissionInputError):
        build_context(make_inputs(alternates=(far,)), CFG)


# ---------------------------------------------------------------------------
# 2. 地形、風場、區域
# ---------------------------------------------------------------------------

def test_terrain_bounds_cover_everything_with_margin(ctx: MissionContext):
    xmin, ymin, xmax, ymax = ctx.terrain.bounds()
    tol = 1e-3                                                   # WGS84 往返之浮點誤差
    assert xmin <= -3000.0 + tol and ymin <= -1200.0 - 3000.0 + tol
    assert xmax >= 7000.0 + 3000.0 - tol and ymax >= 1500.0 + 3000.0 - tol
    assert ctx.terrain.step_m == 50.0 and ctx.terrain.sigma_z_m == 2.0
    assert np.isfinite(ctx.terrain.z_at(np.array([[5500.0, 0.0], [0.0, 0.0], [2000.0, 1500.0]]))).all()


def test_terrain_margin_keyword():
    ctx2 = build_context(make_inputs(), CFG, terrain_margin_m=500.0)
    xmin, ymin, xmax, ymax = ctx2.terrain.bounds()
    assert -600.0 <= xmin <= -500.0 and xmax >= 7500.0 and ctx2.terrain.nx < build_context(make_inputs(), CFG).terrain.nx


def test_wind_field(ctx: MissionContext):
    assert ctx.wind.speed_at(150.0) == pytest.approx(5.0)
    assert np.allclose(ctx.wind.wind_vec(150.0)[0], [5.0, 0.0], atol=1e-9)
    assert ctx.wind.gust_at(150.0) == pytest.approx(7.0)
    assert any(w.startswith("wind: ") for w in ctx.warnings)


def test_areas_enu_polygon(ctx: MissionContext):
    poly = ctx.areas_enu["Z1"]
    assert isinstance(poly, shapely.Polygon) and poly.is_valid
    assert poly.area == pytest.approx(3000.0 * 1200.0, rel=2e-3)
    minx, miny, maxx, maxy = poly.bounds
    assert (minx, miny, maxx, maxy) == pytest.approx((4000.0, -600.0, 7000.0, 600.0), abs=1.0)


def test_area_with_hole_and_invalid_area():
    hole = _ring((5000, -200), (5400, -200), (5400, 200), (5000, 200))
    inp = make_inputs(areas=(AreaSpec("Z1", AREA_RING, holes_wgs84=(hole,)),))
    ctx2 = build_context(inp, CFG)
    assert ctx2.areas_enu["Z1"].area == pytest.approx(3000.0 * 1200.0 - 400.0 * 400.0, rel=2e-3)
    assert len(ctx2.areas_enu["Z1"].interiors) == 1
    degenerate = _ring((4000, 0), (7000, 0), (5500, 0))
    with pytest.raises(MissionInputError):
        build_context(make_inputs(areas=(AreaSpec("Z1", degenerate),)), CFG)


# ---------------------------------------------------------------------------
# 3. platform_overrides（規格 §3.5）
# ---------------------------------------------------------------------------

def test_override_min_fw_only_raise():
    with pytest.raises(MissionInputError):
        build_context(make_inputs(platform_overrides={"A": {"min_fw_operation_agl_m": 100.0}}), CFG)
    ctx2 = build_context(make_inputs(platform_overrides={"A": {"min_fw_operation_agl_m": 150.0,
                                                               "terrain_clearance_m": 40.0}}), CFG)
    sa = ctx2.specs["A"]
    assert sa.min_fw_operation_agl_m == 150.0 and sa.terrain_clearance_m == 40.0
    assert "terrain_clearance_m" not in sa.provisional
    assert ctx2.specs["B"] is B                                   # 未覆寫構型維持原物件
    assert any(w.startswith("overridden(A.min_fw_operation_agl_m, 120.0→150.0") for w in ctx2.warnings)
    assert any(w.startswith("overridden(A.terrain_clearance_m") for w in ctx2.warnings)
    assert A.min_fw_operation_agl_m == 120.0                      # 常數不變


def test_override_disallowed_key_and_inconsistent_value():
    with pytest.raises(MissionInputError):
        apply_platform_overrides({"A": A}, {"A": {"cruise_airspeed_mps": 25.0}})
    with pytest.raises(MissionInputError):                        # lo > hi → PlatformSpecError → MissionInputError
        build_context(make_inputs(platform_overrides={"A": {"transition_complete_agl_m": [100.0, 80.0]}}), CFG)
    specs, warns = apply_platform_overrides({"A": A, "B": B}, {"B": {"transition_ground_run_m": 300.0,
                                                                     "payload_max_kg": 0.9}})
    assert specs["B"].transition_cmd().d_tr_m == 300.0 and specs["B"].payload_max_kg == 0.9
    assert "payload_max_kg" not in specs["B"].provisional and "transition_ground_run_m" not in specs["B"].provisional_supplementary
    assert specs["A"] is A and len(warns) == 2
    _, warns2 = apply_platform_overrides({"A": A}, {"B": {"payload_max_kg": 0.9}})
    assert warns2 == ["override_ignored_no_fleet(B)"]


# ---------------------------------------------------------------------------
# 4. h_clr（D11）與 h_ceil（§4.6）
# ---------------------------------------------------------------------------

def test_h_clr_resolution(ctx: MissionContext):
    assert ctx.h_clr_m == 36.0                                    # 3σ+30 = 36 恰好不抬高
    assert not any(w.startswith("terrain_clearance_raised") for w in ctx.warnings)
    # Airspace None → spec 覆寫槽
    inp = make_inputs(airspace=dataclasses.replace(make_inputs().airspace, terrain_clearance_m=None),
                      platform_overrides={"A": {"terrain_clearance_m": 40.0}})
    assert build_context(inp, CFG).h_clr_m == 40.0
    # 兩層皆 None → MissionInputError（列鍵名）
    with pytest.raises(MissionInputError, match="terrain_clearance_m"):
        build_context(make_inputs(airspace=dataclasses.replace(make_inputs().airspace, terrain_clearance_m=None)), CFG)
    # 低於 3σ+30 → 抬高並警告
    ctx3 = build_context(make_inputs(airspace=dataclasses.replace(make_inputs().airspace, terrain_clearance_m=20.0)), CFG)
    assert ctx3.h_clr_m == 36.0 and any(w.startswith("terrain_clearance_raised_to_3sigma_plus_30(20") for w in ctx3.warnings)
    # σ 未知 → 採輸入值並警告
    ctx4 = build_context(make_inputs(dem=_dem(sigma=None),
                                     airspace=dataclasses.replace(make_inputs().airspace, terrain_clearance_m=20.0)), CFG)
    assert ctx4.h_clr_m == 20.0 and "terrain_clearance_sigma_unknown" in ctx4.warnings


def test_h_ceil_resolution(ctx: MissionContext):
    assert ctx.h_ceil_m == {"A": 200.0, "B": 200.0} and ctx.h_ceil_source == "waiver_max_agl_m"
    base = make_inputs().airspace
    c1 = build_context(make_inputs(airspace=dataclasses.replace(base, waiver_max_agl_m=None)), CFG)
    assert c1.h_ceil_m["A"] == 120.0 and c1.h_ceil_source == "max_agl_m"
    c2 = build_context(make_inputs(airspace=dataclasses.replace(base, waiver_max_agl_m=None, max_agl_m=None)), CFG)
    assert c2.h_ceil_m == {"A": 120.0, "B": 120.0} and c2.h_ceil_source == "default_120"
    assert "airspace_ceiling_defaulted_120" in c2.warnings
    assert "airspace_ceiling_defaulted_120" not in ctx.warnings
    c3 = build_context(make_inputs(airspace=dataclasses.replace(base, waiver_max_agl_m=None, max_agl_m=None,
                                                                 max_msl_m=400.0)), CFG)
    assert c3.h_ceil_m["A"] == pytest.approx(300.0, abs=0.5) and c3.h_ceil_source == "max_msl_m"
    c4 = build_context(make_inputs(airspace=dataclasses.replace(base, waiver_max_agl_m=None, max_agl_m=150.0,
                                                                 max_msl_m=220.0)), CFG)
    assert c4.h_ceil_m["B"] == pytest.approx(120.0, abs=0.5) and c4.h_ceil_source.startswith("max_msl_m")


def test_legacy_cfg_ceiling_inf_and_link_unbounded():
    cfg = PlannerConfig(legacy_mode=True, dem_step_m=50.0)
    c = build_context(make_inputs(c2=C2Link(None, None, HOME, {})), cfg)      # 無鏈路資訊亦可建（enforce_link=False）
    assert c.h_ceil_m["A"] == math.inf and c.h_ceil_source == "cfg.h_ceil_override_m"
    assert c.rho_sym_m == {"A": math.inf, "B": math.inf}
    assert c.link_poly["A"].contains(Point(1e6, 1e6))


# ---------------------------------------------------------------------------
# 5. 鏈路（§3.1、D11）
# ---------------------------------------------------------------------------

def test_link_poly_disc_and_rho_sym(ctx: MissionContext):
    for cls in ("A", "B"):
        poly = ctx.link_poly[cls]
        assert poly.area == pytest.approx(math.pi * 25000.0**2, rel=1e-2)
        assert poly.contains(ctx.areas_enu["Z1"]) and poly.contains(Point(ctx.sites_enu["ALT1"][0]))
        assert not poly.contains(Point(26000.0, 0.0))
    assert ctx.rho_sym_m == {"A": 66000.0, "B": 39000.0}


def test_link_poly_per_class_coverage_and_missing():
    c = build_context(make_inputs(c2=C2Link({"A": 25.0, "B": 15.0}, None, HOME, {})), CFG)
    assert c.link_poly["B"].area == pytest.approx(math.pi * 15000.0**2, rel=1e-2)
    cov = _ring((-2000, -3000), (9000, -3000), (9000, 3000), (-2000, 3000))
    c2 = build_context(make_inputs(c2=C2Link(None, cov, HOME, {})), CFG)
    assert c2.link_poly["A"].area == pytest.approx(11000.0 * 6000.0, rel=2e-3)
    c3 = build_context(make_inputs(c2=C2Link(25.0, cov, HOME, {})), CFG)      # 兩者皆給 → 交集
    assert c3.link_poly["A"].area <= min(c2.link_poly["A"].area, c.link_poly["A"].area) + 1.0
    with pytest.raises(MissionInputError, match="c2"):
        build_context(make_inputs(c2=C2Link(None, None, HOME, {})), CFG)
    # spec 覆寫槽
    c4 = build_context(make_inputs(c2=C2Link(None, None, HOME, {}),
                                   platform_overrides={"A": {"c2_link_range_km": 10.0}, "B": {"c2_link_range_km": 8.0}}), CFG)
    assert c4.link_poly["A"].area == pytest.approx(math.pi * 10000.0**2, rel=1e-2)
    assert c4.link_poly["B"].area == pytest.approx(math.pi * 8000.0**2, rel=1e-2)


def test_ground_station_elsewhere_moves_disc():
    gs = Site("GS", *_wgs(1000.0, 0.0))
    c = build_context(make_inputs(c2=C2Link(25.0, None, gs, {})), CFG)
    assert np.allclose(c.ground_station_xy, [1000.0, 0.0], atol=0.05)
    assert c.link_poly["A"].centroid.x == pytest.approx(1000.0, abs=1.0)


# ---------------------------------------------------------------------------
# 6. 感測器（D43）與能量模型（§5）
# ---------------------------------------------------------------------------

def test_sensors_per_cls_req(ctx: MissionContext):
    assert set(ctx.sensors) == {("A", "default"), ("B", "default_rgb")}
    assert ctx.sensors[("A", "default")].swath_m(160.0) == pytest.approx(224.07, abs=0.05)
    assert ctx.sensors[("B", "default_rgb")].swath_m(100.0) == pytest.approx(146.875, abs=0.01)
    assert ctx.available_kinds == {"A": frozenset({"lidar"}), "B": frozenset({"rgb"})}
    assert "sensor_spec_default_example(A)" in ctx.warnings and "sensor_spec_default_example(B)" in ctx.warnings


def test_sensors_A_with_rgb_and_shutter_error():
    fleet = _fleet()
    a_rgb = dataclasses.replace(fleet[0], sensors=(LidarHardware(), CameraHardware(is_default_example=False)))
    c = build_context(make_inputs(fleet=(a_rgb, fleet[1])), CFG)
    assert ("A", "default_rgb") in c.sensors and c.available_kinds["A"] == frozenset({"lidar", "rgb"})
    bad = dict(make_inputs().payload_reqs)
    bad["default_rgb"] = PayloadRequirement("default_rgb", "rgb", gsd_required_m=0.05, shutter_rate_hz=5.0)
    with pytest.raises(MissionInputError):
        build_context(make_inputs(payload_reqs=bad), CFG)


def test_models_per_uav_numbers(ctx: MissionContext):
    assert set(ctx.models) == {"A1", "B1"}
    ma, mb = ctx.models["A1"], ctx.models["B1"]
    assert ma.spec is ctx.specs["A"] and ma.auw_kg == 24.6 and ma.available_wh == 1350.0
    assert ma.nominal_wh == 1500.0 and ma.reference_wh == 1500.0 and not ma.base_minute_mode
    assert ma.k_unc == pytest.approx(1.2075) and ma.k_bm == CFG.k_bm and ma.k_as == CFG.k_as
    assert ma.e_usable_wh() == pytest.approx(779.6, abs=0.5)
    assert mb.e_usable_wh() == pytest.approx(214.3, abs=0.5)
    assert ma.battery_temp_c == 25.0 and mb.battery_temp_c == 22.0      # A1 取 WeatherObs.temp_c
    assert "battery_temp_assumed(A1)" in ctx.warnings and "battery_temp_assumed(B1)" not in ctx.warnings
    assert "reference_battery_assumed(A1)" in ctx.warnings
    assert not any(w.startswith("battery_nominal_example") for w in ctx.warnings)


def test_models_rep_conservative():
    fleet = _fleet()
    a2 = UAVInstance("A2", A, (LidarHardware(),), SortieEnergy("A2", 24.0, payload_kg=4.4, available_wh=1200,
                                                                pack_nominal_wh=1400, battery_temp_c=5.0))
    c = build_context(make_inputs(fleet=(fleet[0], a2, fleet[1])), CFG)
    rep = c.models_rep["A"]
    assert rep.auw_kg == 24.6 and rep.available_wh == 1200.0 and rep.nominal_wh == 1500.0 and rep.battery_temp_c == 5.0
    assert c.auw_kg == {"A": 24.6, "B": 6.65}
    assert c.models_rep["B"].e_usable_wh() == pytest.approx(c.models["B1"].e_usable_wh())


def test_base_minute_mode_and_missing_nominal():
    fleet = _fleet()
    a_soc = dataclasses.replace(fleet[0], sortie=SortieEnergy("A1", 24.6, payload_kg=5.0, soc_fraction=0.9))
    c = build_context(make_inputs(fleet=(a_soc, fleet[1])), CFG)
    m = c.models["A1"]
    assert m.base_minute_mode and m.available_wh == pytest.approx(0.9 * 176.0) and m.p_cruise_ref_w() == pytest.approx(1 / 60)
    assert "base_minute_mode(A1)" in c.warnings and c.models_rep["A"].base_minute_mode
    a_wh = dataclasses.replace(fleet[0], sortie=SortieEnergy("A1", 24.6, payload_kg=5.0, available_wh=1350))
    with pytest.raises(MissionInputError, match="pack_nominal_wh"):
        build_context(make_inputs(fleet=(a_wh, fleet[1])), CFG)
    # spec 示例標稱（覆寫）→ battery_nominal_example
    c2 = build_context(make_inputs(fleet=(a_wh, fleet[1]), platform_overrides={"A": {"battery_nominal_wh": 1500.0}}), CFG)
    assert c2.models["A1"].nominal_wh == 1500.0 and "battery_nominal_example(A1)" in c2.warnings


def test_validated_flags_force_unity_k():
    spec_v = dataclasses.replace(A, battery_monitor_validated=True, airspeed_sensor_validated=True)
    fleet = _fleet()
    a_v = dataclasses.replace(fleet[0], spec=spec_v)
    c = build_context(make_inputs(fleet=(a_v, fleet[1])), CFG)
    assert c.models["A1"].k_unc == 1.0 and c.models["B1"].k_unc == pytest.approx(1.2075)


# ---------------------------------------------------------------------------
# 7. 禁區（D22/D41）、障礙（D24）、keep-out
# ---------------------------------------------------------------------------

def test_nofly_buffered_active_window(ctx: MissionContext):
    assert len(ctx.nofly_buffered) == 1
    raw = C.polygon_enu(ctx.frame, ctx.inputs.dem, NOFLY_RING)
    buf = ctx.nofly_buffered[0]
    assert buf.contains(raw) and buf.area > raw.area
    assert buf.buffer(1e-6).contains(raw.buffer(CFG.b_trk_m - 0.5))
    base = make_inputs().airspace
    inactive = dataclasses.replace(base, no_fly=(NoFlyZone("NF1", NOFLY_RING, active_window_s=(8000.0, 9000.0)),))
    assert build_context(make_inputs(airspace=inactive), CFG).nofly_buffered == []
    # 時間窗關閉 → 一律有效
    cfg_nowin = PlannerConfig(dem_step_m=50.0, enforce_time_windows=False)
    c = build_context(make_inputs(airspace=inactive), cfg_nowin)
    assert len(c.nofly_buffered) == 1 and c.t_window is None


def test_obstacles_enu_and_burn(ctx: MissionContext):
    assert [o.obs_id for o in ctx.obstacles_enu] == ["T1", "PL1"]
    t1, pl1 = ctx.obstacles_enu
    assert isinstance(t1, ObstacleEnu) and isinstance(t1.geom, Point) and t1.buffer_m == CFG.b_h_m and pl1.buffer_m == 60.0
    xy_t = np.array([t1.geom.x, t1.geom.y])
    assert np.allclose(xy_t, [2500.0, 300.0], atol=0.05)
    z_ground = ctx.frame.msl_to_enu_z(xy_t.reshape(1, 2), Z0)[0]
    assert t1.top_z_m == pytest.approx(z_ground + 40.0, abs=0.1)
    assert pl1.top_z_m == pytest.approx(ctx.frame.msl_to_enu_z(np.array([[3000.0, 1000.0]]), 130.0)[0], abs=0.05)
    assert ctx.obstacle_pairs()[0][1] == t1.top_z_m
    # 燒入（D24）：塔頂 + b_v；z_raw 不變
    z_b = ctx.terrain.z_at(xy_t.reshape(1, 2))[0]
    assert z_b >= t1.top_z_m + CFG.b_v_m - 1e-3
    zr = ctx.terrain.z_raw[int(round((300 - ctx.terrain.y0_m) / 50)), int(round((2500 - ctx.terrain.x0_m) / 50))]
    assert abs(zr - z_ground) < 0.1
    z_line = ctx.terrain.z_at(np.array([[3000.0, 0.0]]))[0]
    assert z_line >= pl1.top_z_m + CFG.b_v_m - 1e-3
    assert ctx.terrain.z_at(np.array([[5500.0, 0.0]]))[0] < 1.0        # 區域內未受影響


def test_dsm_skips_tree_building_burn():
    tree = dataclasses.replace(TOWER, obs_id="TR1", category="tree")
    bld = Obstacle("BLD", "polygon", _ring((1500, 800), (1600, 800), (1600, 900), (1500, 900)), None, 30.0, category="building")
    c = build_context(make_inputs(dem=_dem(is_dsm=True), obstacles=(tree, bld, LINE)), CFG)
    assert c.terrain.is_dsm
    xy = np.array([[2500.0, 300.0], [1550.0, 850.0]])
    assert np.allclose(c.terrain.z_at(xy), c.frame.msl_to_enu_z(xy, Z0), atol=0.1)     # 未燒入
    assert c.terrain.z_at(np.array([[3000.0, 0.0]]))[0] >= 130.0 - 100.0 + CFG.b_v_m - 1.0   # powerline 仍燒入
    assert [o.obs_id for o in c.obstacles_enu] == ["TR1", "BLD", "PL1"]


def test_keepouts_lazy_import(ctx: MissionContext):
    if ctx.keepouts is None:
        assert "routing_unavailable_keepouts_none" in ctx.warnings
    else:
        assert type(ctx.keepouts).__name__ == "KeepoutSet"


# ---------------------------------------------------------------------------
# 8. DecomposeContext、其他
# ---------------------------------------------------------------------------

def test_decompose_ctx(ctx: MissionContext):
    d = ctx.decompose_ctx()
    assert isinstance(d, DecomposeContext)
    assert d.frame is ctx.frame and d.terrain is ctx.terrain and d.wind is ctx.wind and d.cfg is ctx.cfg
    assert d.airspace is ctx.inputs.airspace and d.specs is ctx.specs and d.sensors is ctx.sensors
    assert d.models_rep is ctx.models_rep and d.nofly_buffered is ctx.nofly_buffered
    assert d.available_kinds == ctx.available_kinds and d.auw_kg == ctx.auw_kg
    assert np.allclose(d.home_xy, 0.0, atol=1e-6) and d.h_clr_m == ctx.h_clr_m and d.h_ceil_m == ctx.h_ceil_m


def test_recovery_site_and_helpers():
    c = build_context(make_inputs(recovery_site_id="ALT1"), CFG)
    assert c.recovery_site_id == "ALT1"
    assert c.spec_of("B1") is c.specs["B"] and c.uav("A1").uav_id == "A1"


def test_dataclasses_slots():
    assert "__slots__" in DecomposeContext.__dict__ and "__slots__" in MissionContext.__dict__


def test_build_is_fast():
    import time
    t0 = time.perf_counter()
    build_context(make_inputs(), CFG)
    assert time.perf_counter() - t0 < 3.0


# ---------------------------------------------------------------------------
# 12. 逐機起飛站點（UAVInstance.start_site_id → ctx.start_sites / start_xy）
# ---------------------------------------------------------------------------

def test_start_sites_default_to_home(ctx: MissionContext):
    """未設定 start_site_id → 每機起飛站點皆為 home，start_xy == home_xy（既有情境不變）。"""
    assert ctx.start_sites == {"A1": "H", "B1": "H"}
    for uid in ("A1", "B1"):
        assert ctx.start_site_id(uid) == "H"
        assert np.allclose(ctx.start_xy(uid), ctx.home_xy)
        assert ctx.z_ground_start_m(uid) == pytest.approx(ctx.z_ground_home_m)
    assert not any(w.startswith("per_uav_start_sites") for w in ctx.warnings)


def test_start_sites_per_uav_resolved_to_site_enu():
    """A1 自 ALT1 起飛 → start_xy 為 ALT1 之 ENU (2000, 1500)；B1 仍為 home；回收點不變（D40）。"""
    a1, b1 = _fleet()
    inputs = make_inputs(fleet=(dataclasses.replace(a1, start_site_id="ALT1"), b1))
    c = build_context(inputs, CFG)
    assert c.start_sites == {"A1": "ALT1", "B1": "H"}
    assert np.allclose(c.start_xy("A1"), c.sites_enu["ALT1"][0])
    assert np.allclose(c.start_xy("A1"), (2000.0, 1500.0), atol=1.0)
    assert np.allclose(c.start_xy("B1"), c.home_xy)
    assert c.z_ground_start_m("A1") == pytest.approx(c.sites_enu["ALT1"][1])
    assert c.recovery_site_id == "H"                                 # 起飛點改變不動回收點
    assert any(w == "per_uav_start_sites(A1@ALT1)" for w in c.warnings)


def test_start_site_id_falls_back_to_inputs_when_map_empty(ctx: MissionContext):
    """start_sites 缺鍵（例如手工建構之 MissionContext）→ 由 inputs 解析，不丟例外。"""
    c = dataclasses.replace(ctx, start_sites={})
    assert c.start_site_id("A1") == "H" and np.allclose(c.start_xy("A1"), c.home_xy)


# ---------------------------------------------------------------------------
# 審查後新增（R-04 的 decompose/context 半邊）：both 區之「同機雙感測 → 取較窄者鋪一套」（規格 §6.2）
# ---------------------------------------------------------------------------
def test_build_sensors_combines_dual_carry_into_narrower_single_pass():
    """同一構型同時掛載 LiDAR 與 RGB 時，(cls, req_id) 應回傳 `CombinedSensor`：

    間距／掃幅／地速上限取兩者之嚴格值（min）、高度窗取交集、端延伸取 max，且 `kinds` 同時含兩種。
    這是規格 §6.2 之「取較窄者鋪一套」；此前只鋪第一個相符 Sensor 的那一套，第二個感測需求無聲消失。
    """
    from hv_dccpp.platform import CombinedSensor

    inp = make_inputs()
    a_dual = dataclasses.replace(inp.fleet[0], sensors=(LidarHardware(), CameraHardware()))
    sensors, kinds, warns = C.build_sensors((a_dual, inp.fleet[1]), inp.payload_reqs)
    combined = sensors[("A", "default")]
    assert isinstance(combined, CombinedSensor)
    assert set(combined.kinds) == {"lidar", "rgb"}
    assert kinds["A"] == frozenset({"lidar", "rgb"})
    assert any(w.startswith("dual_sensor_single_pass(A, default)") for w in warns)

    lid, cam = combined.sensors
    for h in (100.0, 150.0, 200.0):
        assert combined.strip_spacing_m(h) == pytest.approx(min(lid.strip_spacing_m(h), cam.strip_spacing_m(h)))
        assert combined.swath_m(h) == pytest.approx(min(lid.swath_m(h), cam.swath_m(h)))
        assert combined.max_ground_speed_mps(h) == pytest.approx(min(lid.max_ground_speed_mps(h),
                                                                     cam.max_ground_speed_mps(h)))
        assert combined.end_extension_m(20.0, 2.0, h) == pytest.approx(max(lid.end_extension_m(20.0, 2.0, h),
                                                                          cam.end_extension_m(20.0, 2.0, h)))
    lo = max(lid.agl_window()[0], cam.agl_window()[0])
    hi = min(lid.agl_window()[1], cam.agl_window()[1])
    assert combined.agl_window() == pytest.approx((lo, hi))
    # (cls, req_id + "_rgb") 仍保留原始 RGB 模型（子感測器可單獨取用）
    assert sensors[("A", "default_rgb")].kind == "rgb"


def test_build_sensors_single_kind_is_not_combined():
    """只掛一種感測器（本情境之 A1／B1）時不得產生 CombinedSensor，也不得發出 dual_sensor 警告。"""
    from hv_dccpp.platform import CombinedSensor

    inp = make_inputs()
    sensors, _kinds, warns = C.build_sensors(inp.fleet, inp.payload_reqs)
    assert not any(isinstance(s, CombinedSensor) for s in sensors.values())
    assert not any(w.startswith("dual_sensor_single_pass") for w in warns)
    assert sorted(sensors) == [("A", "default"), ("B", "default_rgb")]
