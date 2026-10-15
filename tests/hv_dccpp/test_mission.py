"""tests/test_mission.py — 模組契約 §3（mission.py 純資料型別、load_mission、序列化）與規格 §17.2 之資料層測試。

preflight 相關（precheck_*、A_needs_waiver_flag、dsm_skips_tree_building_burn、airspace_ceiling_default_warning）屬
preflight.py，此處只測其資料層前提（旗標／解析順序／缺鍵例外）。
"""
from __future__ import annotations

import dataclasses
import json
import math
from pathlib import Path

import numpy as np
import pytest
import shapely

from hv_dccpp import mission as M
from hv_dccpp import platform as P
from hv_dccpp.geo import DemGrid
from hv_dccpp.mission import (
    FW_PHASES,
    K_INF,
    TRANSITION_PHASES,
    VTOL_PHASES,
    Airspace,
    AreaSpec,
    C2Link,
    FeasibilityReport,
    MissionInputError,
    MissionInputs,
    MissionPlan,
    Mode,
    NoFlyZone,
    Obstacle,
    PayloadRequirement,
    Phase,
    PlannerConfig,
    Site,
    SortieEnergy,
    UAVInstance,
    UAVPlan,
    UAVState,
    Waypoint,
    WeatherObs,
    WeatherThresholds,
    WindLayer,
    load_mission,
    mode_phase_violation,
    resolve_weather_go,
    to_jsonable,
)
from hv_dccpp.platform import PLATFORM_HERO, PLATFORM_PIONEER, CameraHardware, LidarHardware, TrapezoidFovSpec, sensor_model

A = PLATFORM_PIONEER
B = PLATFORM_HERO

# ---------------------------------------------------------------------------
# fixtures
# ---------------------------------------------------------------------------

SQUARE = [[23.945, 120.94], [23.945, 120.97], [23.955, 120.97], [23.955, 120.94]]


def _dem() -> DemGrid:
    return DemGrid(23.9, 120.85, 0.01, 0.01, np.full((6, 8), 100.0, dtype=np.float32), sigma_z_m=2.0)


def mission_dict() -> dict:
    """§W mini example 之 JSON 形式（平地、A1+B1、單層西風 5 m/s）。"""
    return {
        "name": "mini",
        "home": {"site_id": "H", "lat_deg": 23.95, "lon_deg": 120.9},
        "alternates": [{"site_id": "ALT1", "lat_deg": 23.963, "lon_deg": 120.92, "alt_agl_m": 0.0, "approach_heading_deg": 270.0}],
        "areas": [{"area_id": "Z1", "polygon_wgs84": SQUARE, "sensor_req": "any", "priority": 1.0,
                   "payload_req_id": "default", "time_window_s": [0, 5400]}],
        "dem": {"lat0_deg": 23.9, "lon0_deg": 120.85, "dlat_deg": 0.01, "dlon_deg": 0.01,
                "z_msl": np.full((6, 8), 100.0).tolist(), "sigma_z_m": 2.0},
        "obstacles": [{"obs_id": "T1", "geom_type": "point", "coords_wgs84": [[23.95, 120.93]], "height_agl_m": 40.0,
                       "category": "tower", "keepout_buffer_m": 60.0},
                      {"obs_id": "PL1", "geom_type": "line", "coords_wgs84": [[23.94, 120.92], [23.94, 120.93]],
                       "top_msl_m": 130.0, "category": "powerline"}],
        "wind": [{"z_bottom_m": 0, "z_top_m": 1000, "speed_mps": 5, "dir_from_deg": 270, "gust_mps": 7},
                 {"z_bottom_m": 0, "z_top_m": 300, "speed_mps": 4, "dir_from_deg": 270, "z_ref": "agl"}],
        "fleet": [
            {"uav_id": "A1", "cls": "A", "sensors": ["lidar"],
             "sortie": {"auw_kg": 24.6, "payload_kg": 5.0, "available_wh": 1350, "pack_nominal_wh": 1500}},
            {"uav_id": "B1", "spec": "HERO", "sensors": [{"kind": "rgb"}],
             "sortie": {"auw_kg": 6.65, "payload_kg": 0.6, "available_wh": 380, "pack_nominal_wh": 420, "battery_temp_c": 22.0}},
        ],
        "payload_reqs": {
            "default": {"kind": "lidar", "density_pt_m2": 20, "side_overlap": 0.25, "h_target_agl_m": 160},
            "default_rgb": {"kind": "rgb", "gsd_required_m": 0.05, "side_overlap": 0.30, "forward_overlap": 0.70,
                            "h_target_agl_m": 100},
        },
        "c2": {"range_km": 25, "lost_link_policy": {"B1": "RTB"}, "t_ll_s": 60.0},
        "airspace": {"max_agl_m": 120, "waiver_max_agl_m": 200, "terrain_clearance_m": 36, "time_window_s": [0, 7200],
                     "no_fly": [{"zone_id": "NF1", "polygon_wgs84": [[23.94, 120.90], [23.94, 120.91], [23.945, 120.91]],
                                 "z_ceiling_msl_m": 500.0, "active_window_s": [0, 600]}]},
        "weather_thresholds": {"go_gust_mps": 12.0},
        "weather_obs": {"mean_wind_mps": 5, "gust_mps": 7, "wind_dir_from_deg": 270, "visibility_m": 8000,
                        "precip_mmh": 0, "temp_c": 25},
        "platform_overrides": {"A": {"terrain_clearance_m": 36.0}},
        "recovery_site_id": "H",
    }


@pytest.fixture
def mission() -> MissionInputs:
    return load_mission(mission_dict())


def _sortie(**kw) -> SortieEnergy:
    base = dict(uav_id="A1", auw_kg=24.6)
    base.update(kw)
    return SortieEnergy(**base)


# ---------------------------------------------------------------------------
# 1. dataclass 建構、frozen/slots、表 2 欄位
# ---------------------------------------------------------------------------

def test_mission_input_error_is_platform_error():
    assert MissionInputError is P.MissionInputError
    assert issubclass(MissionInputError, ValueError)


@pytest.mark.parametrize("cls", [Site, AreaSpec, Obstacle, WindLayer, SortieEnergy, PayloadRequirement, C2Link,
                                 NoFlyZone, Airspace, WeatherThresholds, WeatherObs, UAVInstance, MissionInputs, PlannerConfig])
def test_input_types_frozen_and_slots(cls):
    params = cls.__dataclass_params__
    assert params.frozen and params.slots
    assert "__slots__" in cls.__dict__


@pytest.mark.parametrize("cls", [UAVState, Waypoint, UAVPlan, FeasibilityReport, MissionPlan])
def test_output_types_slots_mutable(cls):
    params = cls.__dataclass_params__
    assert params.slots and not params.frozen


def test_frozen_instance_rejects_assignment():
    s = Site("H", 23.95, 120.9, kind="home")
    with pytest.raises(dataclasses.FrozenInstanceError):
        s.lat_deg = 1.0          # type: ignore[misc]
    assert not hasattr(s, "__dict__")
    assert s.latlon == (23.95, 120.9)


def test_table2_fields_present():
    names = [f.name for f in dataclasses.fields(MissionInputs)]
    assert names == ["home", "alternates", "areas", "dem", "obstacles", "wind", "fleet", "payload_reqs", "c2",
                     "airspace", "weather_thresholds", "weather_obs", "t0_epoch_s", "platform_overrides",
                     "recovery_site_id"]
    assert [f.name for f in dataclasses.fields(Site)] == ["site_id", "lat_deg", "lon_deg", "alt_msl_m", "alt_agl_m",
                                                          "kind", "clear_radius_m", "approach_heading_deg"]
    assert [f.name for f in dataclasses.fields(SortieEnergy)] == ["uav_id", "auw_kg", "payload_kg", "available_wh",
                                                                  "available_mah", "pack_voltage_nom_v", "soc_fraction",
                                                                  "pack_nominal_wh", "battery_temp_c"]
    assert [f.name for f in dataclasses.fields(Waypoint)] == [
        "x_m", "y_m", "z_m", "lat_deg", "lon_deg", "alt_msl_m", "agl_m", "t_s", "mode", "phase", "sensor_on", "strip_id",
        "cell_id", "dir", "airspeed_mps", "ground_speed_mps", "track_deg", "heading_deg", "energy_remaining_wh",
        "nearest_alternate", "alt_reachable", "lost_link_action", "arc", "accept_radius_m"]


def test_site_agl_altitude_exclusive():
    assert Site("S", 23.0, 120.0, alt_agl_m=0.0).alt_agl_m == 0.0
    assert Site("S", 23.0, 120.0, alt_msl_m=100.0).alt_msl_m == 100.0
    assert Site("S", 23.0, 120.0).alt_msl_m is None and Site("S", 23.0, 120.0).alt_agl_m is None   # 皆 None → DEM
    with pytest.raises(MissionInputError):
        Site("S", 23.0, 120.0, alt_msl_m=100.0, alt_agl_m=0.0)
    with pytest.raises(MissionInputError):
        Site("S", 95.0, 120.0)
    with pytest.raises(MissionInputError):
        Site("S", 23.0, 120.0, kind="depot")   # type: ignore[arg-type]
    with pytest.raises(MissionInputError):
        Site("S", 23.0, 120.0, clear_radius_m=0.0)


def test_area_spec_normalises_and_validates():
    a = AreaSpec("Z", SQUARE + [SQUARE[0]], holes_wgs84=[[[23.948, 120.95], [23.948, 120.955], [23.952, 120.955]]])
    assert a.polygon_wgs84 == tuple(tuple(p) for p in SQUARE)          # 閉合頂點去除、list → tuple
    assert isinstance(a.holes_wgs84[0], tuple) and len(a.holes_wgs84[0]) == 3
    assert a.sensor_req == "any" and a.priority == 1.0 and a.payload_req_id == "default" and a.time_window_s is None
    with pytest.raises(MissionInputError):
        AreaSpec("Z", SQUARE[:2])
    with pytest.raises(MissionInputError):
        AreaSpec("Z", SQUARE, sensor_req="thermal")   # type: ignore[arg-type]
    with pytest.raises(MissionInputError):
        AreaSpec("Z", SQUARE, time_window_s=(10.0, 5.0))
    with pytest.raises(MissionInputError):
        AreaSpec("", SQUARE)


def test_obstacle_height_exactly_one():
    o = Obstacle("T", "point", [[23.95, 120.93]], None, 40.0, category="tower")
    assert o.coords_wgs84 == ((23.95, 120.93),) and o.keepout_buffer_m is None
    with pytest.raises(MissionInputError):
        Obstacle("T", "point", [[23.95, 120.93]], None, None)
    with pytest.raises(MissionInputError):
        Obstacle("T", "point", [[23.95, 120.93]], 150.0, 40.0)
    with pytest.raises(MissionInputError):
        Obstacle("L", "line", [[23.95, 120.93]], 150.0, None)              # line 需 ≥2 點
    with pytest.raises(MissionInputError):
        Obstacle("P", "polygon", [[23.95, 120.93], [23.96, 120.93]], 150.0, None)
    with pytest.raises(MissionInputError):
        Obstacle("T", "point", [[23.95, 120.93]], None, 40.0, category="bridge")   # type: ignore[arg-type]


def test_wind_layer_agl_ref_and_vector():
    w = WindLayer(0.0, 300.0, 5.0, 270.0, z_ref="agl")
    assert w.z_ref == "agl" and w.gust_mps == 0.0
    np.testing.assert_allclose(w.wind_vec(), [5.0, 0.0], atol=1e-12)     # 西風 → 吹向東（契約 §0）
    np.testing.assert_allclose(WindLayer(0, 100, 3.0, 0.0).wind_vec(), [0.0, -3.0], atol=1e-12)   # 北風 → 吹向南
    assert WindLayer(0.0, math.inf, 5.0, 270.0).z_top_m == math.inf
    with pytest.raises(MissionInputError):
        WindLayer(100.0, 100.0, 5.0, 270.0)
    with pytest.raises(MissionInputError):
        WindLayer(0.0, 100.0, -1.0, 270.0)
    with pytest.raises(MissionInputError):
        WindLayer(0.0, 100.0, 5.0, 270.0, z_ref="ellipsoid")   # type: ignore[arg-type]


def test_payload_requirement_defaults_d25_via_sensor_model():
    req = PayloadRequirement("default", "lidar")
    assert req.forward_overlap == 0.70 and req.side_overlap is None
    sen = sensor_model(LidarHardware(), req)
    assert "side_overlap" in sen.defaults_applied            # D25：lidar 側向 0.25 由 sensor_model 補
    assert sen.hw.is_default_example
    cam = sensor_model(CameraHardware(), PayloadRequirement("rgb", "rgb", gsd_required_m=0.05))
    assert cam.kind == "rgb"
    with pytest.raises(MissionInputError):
        PayloadRequirement("x", "thermal")   # type: ignore[arg-type]
    with pytest.raises(MissionInputError):
        PayloadRequirement("x", "rgb", side_overlap=1.0)
    with pytest.raises(MissionInputError):
        sensor_model(CameraHardware(), PayloadRequirement("x", "rgb", shutter_rate_hz=5.0))   # > 硬體上限 1 Hz


def test_uav_instance_available_kinds():
    u = UAVInstance("A1", A, (LidarHardware(), CameraHardware()), _sortie(available_wh=1350, pack_nominal_wh=1500))
    assert u.available_kinds == frozenset({"lidar", "rgb"}) and u.cls == "A"
    assert isinstance(u.primary_sensor, LidarHardware)
    u2 = UAVInstance("A1", A, LidarHardware(), _sortie())      # 單一硬體自動包成 tuple
    assert u2.sensors == (LidarHardware(),)
    with pytest.raises(MissionInputError):
        UAVInstance("A2", A, (LidarHardware(),), _sortie())        # sortie.uav_id 不符
    with pytest.raises(MissionInputError):
        UAVInstance("A1", A, (), _sortie())
    with pytest.raises(MissionInputError):
        UAVInstance("A1", A, (TrapezoidFovSpec("lidar", 45.0, 30.0, 60.0), "lidar"), _sortie())   # type: ignore[arg-type]


# ---------------------------------------------------------------------------
# 2. SortieEnergy 解析順序（規格 §5.7）
# ---------------------------------------------------------------------------

def test_sortie_nominal_resolution_order():
    spec_nom = dataclasses.replace(A, battery_nominal_wh=1500.0)
    assert _sortie(pack_nominal_wh=1400.0).nominal_wh_resolved(spec_nom) == 1400.0     # pack → spec
    assert _sortie().nominal_wh_resolved(spec_nom) == 1500.0
    assert _sortie().nominal_wh_resolved(A) is None
    assert _sortie(pack_nominal_wh=1400.0).nominal_wh_resolved(A) == 1400.0


def test_sortie_available_wh_order_wh_mah_soc():
    s = _sortie(available_wh=1350.0, available_mah=20000.0, pack_voltage_nom_v=44.4, soc_fraction=0.5, pack_nominal_wh=1500.0)
    assert s.available_wh_resolved(1500.0) == 1350.0                           # Wh 優先
    s = _sortie(available_mah=20000.0, pack_voltage_nom_v=44.4, soc_fraction=0.5)
    assert s.available_wh_resolved(1500.0) == pytest.approx(888.0)             # mAh×V/1000 次之
    s = _sortie(soc_fraction=0.8)
    assert s.available_wh_resolved(1500.0) == pytest.approx(1200.0)            # SoC×nominal
    assert s.available_wh_resolved(None) is None                               # 僅 soc 且 nominal None → 基準分鐘模式
    assert s.is_base_minute_candidate(A) and not s.is_base_minute_candidate(dataclasses.replace(A, battery_nominal_wh=1500.0))


def test_sortie_available_wh_error_cases():
    with pytest.raises(MissionInputError, match="pack_nominal_wh"):
        _sortie(available_wh=1350.0).available_wh_resolved(None)               # Wh 但 nominal None → 鍵名
    with pytest.raises(MissionInputError, match="battery_nominal_wh"):
        _sortie(available_mah=20000.0, pack_voltage_nom_v=44.4).available_wh_resolved(None)
    with pytest.raises(MissionInputError, match="pack_voltage_nom_v"):
        _sortie(available_mah=20000.0).available_wh_resolved(1500.0)          # mAh 缺電壓
    with pytest.raises(MissionInputError, match="available_wh/available_mah/soc_fraction"):
        _sortie().available_wh_resolved(1500.0)                                # 三者皆缺
    with pytest.raises(MissionInputError):
        _sortie(soc_fraction=1.2)
    with pytest.raises(MissionInputError):
        _sortie(auw_kg=0.0)
    with pytest.raises(MissionInputError):
        _sortie(available_wh=-5.0)


def test_sortie_payload_and_battery_temp_fallback():
    pl, est = _sortie().payload_kg_resolved(A)
    assert est and pl == pytest.approx(24.6 - (24.6 - 5.0))                    # auw − empty_mass = 5.0，警告 payload_estimated
    assert _sortie(payload_kg=4.0).payload_kg_resolved(A) == (4.0, False)
    obs = WeatherObs(5, 7, 270, 8000, 0, 25)
    assert _sortie().battery_temp_resolved(obs) == (25.0, True)                # None → obs.temp_c 並警告 battery_temp_assumed
    assert _sortie(battery_temp_c=15.0).battery_temp_resolved(obs) == (15.0, False)
    assert _sortie().battery_temp_resolved(None) == (20.0, True)


# ---------------------------------------------------------------------------
# 3. C2Link.range_km_for 優先序、失鏈策略缺省（D26）
# ---------------------------------------------------------------------------

def test_c2_range_km_for_precedence():
    gs = Site("GS", 23.95, 120.9, kind="home")
    spec_override = dataclasses.replace(A, c2_link_range_km=30.0)
    c = C2Link({"A": 25.0, "B": 15.0}, None, gs, {})
    assert c.range_km_for("A", A) == 25.0 and c.range_km_for("B", B) == 15.0
    assert c.range_km_for("A", spec_override) == 25.0                          # mission 值優先於 spec 覆寫槽
    c = C2Link(20.0, None, gs, {})
    assert c.range_km_for("A", A) == 20.0 and c.range_km_for("B", B) == 20.0   # 單值套用兩構型
    c = C2Link(None, None, gs, {})
    assert c.range_km_for("A", spec_override) == 30.0                          # → spec 覆寫槽
    assert c.range_km_for("A", A) is None                                      # → None（preflight NO_GO，D11）
    c = C2Link({"A": 25.0}, None, gs, {})
    assert c.range_km_for("B", B) is None and c.range_km_for("B", dataclasses.replace(B, c2_link_range_km=12.0)) == 12.0
    with pytest.raises(MissionInputError):
        C2Link({"C": 1.0}, None, gs, {})
    with pytest.raises(MissionInputError):
        C2Link(-1.0, None, gs, {})
    with pytest.raises(MissionInputError):
        C2Link(25.0, None, gs, {"A1": "LAND_NOW"})


def test_c2_coverage_and_policy_defaults():
    gs = Site("GS", 23.95, 120.9, kind="home")
    c = C2Link(None, {"A": SQUARE}, gs, {"B1": "NEAREST_ALT"})
    assert c.coverage_for("A") == tuple(tuple(p) for p in SQUARE) and c.coverage_for("B") is None
    assert c.policy_for("B1", "B") == "NEAREST_ALT"
    assert c.policy_for("A1", "A") == "CONTINUE_THEN_RTB" and c.policy_for("B2", "B") == "RTB"
    assert c.t_ll_s == 60.0
    c2 = C2Link(None, SQUARE, gs, {})
    assert c2.coverage_for("A") == c2.coverage_for("B")


# ---------------------------------------------------------------------------
# 4. 空域、禁區、時間窗（D22）、氣象（D7）
# ---------------------------------------------------------------------------

def test_time_windows_three_kinds():
    area = AreaSpec("Z", SQUARE, time_window_s=[0, 3600])
    air = Airspace(time_window_s=(0, 7200))
    nf = NoFlyZone("NF", SQUARE, active_window_s=(1000, 2000))
    assert area.time_window_s == (0.0, 3600.0) and air.time_window_s == (0.0, 7200.0)
    assert nf.is_active((0, 1500)) and nf.is_active((2000, 3000)) and not nf.is_active((2001, 3000))   # 重疊即有效
    assert NoFlyZone("NF2", SQUARE).is_active((0, 1)) and nf.is_active(None)
    air2 = Airspace(no_fly=(nf, NoFlyZone("NF2", SQUARE)))
    assert [z.zone_id for z in air2.active_no_fly((3000, 4000))] == ["NF2"]
    with pytest.raises(MissionInputError):
        Airspace(no_fly=(nf, nf))
    with pytest.raises(MissionInputError):
        NoFlyZone("NF", SQUARE, z_floor_msl_m=500.0, z_ceiling_msl_m=400.0)


def test_airspace_ceiling_defaulted_flag():
    assert Airspace().ceiling_defaulted and Airspace().terrain_clearance_m is None
    assert not Airspace(max_agl_m=120).ceiling_defaulted and not Airspace(max_msl_m=500).ceiling_defaulted
    with pytest.raises(MissionInputError):
        Airspace(max_agl_m=0.0)


def test_resolve_weather_go_d7():
    th = WeatherThresholds()
    assert resolve_weather_go(A, th) == pytest.approx(8.0)                # min(8, 0.45·21.5=9.675)
    assert resolve_weather_go(B, th) == pytest.approx(8.0)                # min(8, 8.55)
    assert resolve_weather_go(A, WeatherThresholds(go_mean_wind_mps=6.0)) == 6.0
    assert th.go_gust_mps == 12.0 and th.go_visibility_m == 3000.0 and th.go_precip_mmh == 0.5
    assert th.go_temp_range_c == (-10.0, 40.0) and th.abort_temp_range_c == (-15.0, 45.0)
    assert (th.abort_mean_wind_mps, th.abort_gust_mps, th.abort_visibility_m, th.abort_precip_mmh) == (10.0, 14.0, 1500.0, 2.0)


def test_weather_go_and_abort_violations():
    th = WeatherThresholds()
    ok = WeatherObs(5, 7, 270, 8000, 0, 25)
    assert th.go_violations(ok, 8.0) == [] and th.abort_violations(ok) == []
    bad = WeatherObs(9, 13, 270, 2000, 1.0, 45)
    v = th.go_violations(bad, 8.0)
    assert len(v) == 5 and any("mean_wind" in s for s in v)
    ab = th.abort_violations(WeatherObs(11, 15, 0, 1000, 3.0, 50))
    assert len(ab) == 5
    with pytest.raises(MissionInputError):
        WeatherObs(-1, 7, 270, 8000, 0, 25)
    with pytest.raises(MissionInputError):
        WeatherThresholds(go_temp_range_c=(40, -10))


# ---------------------------------------------------------------------------
# 5. PlannerConfig（規格 §16.2 預設、legacy 覆寫）
# ---------------------------------------------------------------------------

EXPECTED_DEFAULTS = dict(
    dem_step_m=10.0, dp_eps_factor=0.1, merge_tau_factor=0.5, k_sigma=2.0, sigma_xy_m=3.0, e_xt_m=5.0, delta_edge_m=15.0,
    b_trk_m=50.0, b_h_m=50.0, b_v_m=30.0, dh_band_min_m=20.0, t_settle_s=2.0, k_slope_pen=20.0, e_max_m=3.0,
    psi_max_deg=30.0, dtheta_int_deg=5.0, lambda_cut=1e-3, lambda_n=30.0, convexify_whole=False, widen_on_lift=False,
    v_g_min_mps=3.0, r_eff_wind_policy="max_layer", k_bm=1.15, k_as=1.05, temp_cap_coef_per_c=0.01, temp_cap_floor=0.8,
    site_max_slope_deg=5.0, transition_profile="rotor_assisted", nu=(0.15, 0.15, 0.20, 0.20, 0.20, 0.10),
    omega=(0.2, 0.15, 0.2, 0.15, 0.1, 0.2), kappa_load=0.85, n_local_search=200, multi_sortie=False, t_swap_s=600.0,
    dt_dep_s=60.0, t_sep_ldg_s=90.0, idp_branching="event", transfer_types=("rev", "same"), K=6, B=3, eps_sum=0.05,
    mu_paper=0.0, tau0_s=60.0, realtime_budget_s=1.0, d_d_m=25.0, n_smooth=50, c1=1.0, c2=0.5, lr=0.2, eps_col=1e-3,
    L_max_m=500.0, k_gsd=1.0, k_gsd_relax=1.2, h_transit_offset_A_m=50.0, dh_sep_m=50.0, delta_prof_m=10.0, d_sep_m=100.0,
    d_sep_lat_m=60.0, k_t=0.05, dt_4d_s=2.0, delta_1=0.05, delta_2=0.15, energy_margin_frac=0.05, r_conf_m=150.0,
    gsd_confirm_m=0.025, t_conf_max_s=900.0, tau_cue=0.10, tau_w=0.05, confirm_mode="strips", cov_raster_m=5.0,
    fov_model="sensor", metric="makespan", legacy_mode=False, seed=0,
    enforce_energy=True, enforce_link=True, enforce_time_windows=True, t_rtb_lower_bound=True,
    e_ext_override_m=None, h_ceil_override_m=None,
    # ⚠⚠⚠ vendored 修改 #2（見 docs/hv_dccpp/VENDOR.md）：這兩個**不在規格 §16.2 裡**。
    #   估計層原本用固定 skip-k 序列，而規劃層（IDP beam）是搜尋的 ⇒ T_single 系統性
    #   悲觀且偏差隨條帶數二次成長，再傳進掃描方向排序與 λ_floor。
    #   `sweep_sequence="fixed"` 可逐位元退回上游行為（legacy_mode 也會強制 fixed）。
    sweep_sequence="search", sweep_exact_max_n=9,
    # ⚠⚠⚠ vendored 修改 #3（見 docs/hv_dccpp/VENDOR.md）：這四個也**不在規格 §16.2 裡**。
    #   `repair` 是逐衝突貪婪、`_compact_makespan` 一次只平移一機 ⇒ 協同平移做不到。
    #   實測 HV4SITL9 起飛被排成 0/289.1/623.9/1265.6 s（makespan 的 70.3%），而
    #   驅動它的 6 筆衝突**全部**是同一場坪的降落走廊排隊，沒有一筆是空間衝突。
    #   `global_reschedule=False` 可逐位元退回上游行為（含那四個起飛時刻），
    #   由 tests/hv_dccpp/test_global_reschedule.py 的刪除實驗鎖住。
    global_reschedule=True, reschedule_step_s=10.0,
    reschedule_objective="landing", reschedule_max_evals=20000,
)


def test_planner_config_defaults_match_spec_16_2():
    cfg = PlannerConfig()
    names = {f.name for f in dataclasses.fields(PlannerConfig)}
    assert names == set(EXPECTED_DEFAULTS), names ^ set(EXPECTED_DEFAULTS)
    for k, v in EXPECTED_DEFAULTS.items():
        assert getattr(cfg, k) == v, k
    assert cfg.psi_max_rad == pytest.approx(math.radians(30)) and cfg.dtheta_int_rad == pytest.approx(math.radians(5))
    assert cfg.delta_edge_resolved() == 15.0
    assert PlannerConfig(delta_edge_m=None).delta_edge_resolved(8.0) == pytest.approx(2.0 * 3.0 + 5.0 + 4.0)
    assert cfg.k_nearest(4) == 4 and cfg.k_nearest(100) == 6


def test_planner_config_apply_legacy():
    cfg = PlannerConfig()
    lg = cfg.apply_legacy()
    assert cfg.legacy_mode is False and cfg.B == 3 and cfg.K == 6          # 原物件不變
    assert lg.legacy_mode is True
    assert lg.idp_branching == "lockstep" and lg.transfer_types == ("rev",) and lg.B == 1 and lg.K == K_INF
    assert lg.metric == "paper" and lg.convexify_whole is True and lg.fov_model == "trapezoid"
    assert lg.k_bm == 1.0 and lg.k_as == 1.0 and lg.e_ext_override_m == 0.0
    assert not lg.enforce_energy and not lg.enforce_link and not lg.enforce_time_windows and not lg.t_rtb_lower_bound
    assert math.isinf(lg.h_ceil_override_m)
    assert lg.k_nearest(1000) == 1000
    # 未被覆寫之欄位維持
    assert lg.dem_step_m == 10.0 and lg.seed == 0 and lg.nu == cfg.nu
    # 建構時給 legacy_mode=True 亦一鍵套用（即使同時給 B=3）
    lg2 = PlannerConfig(legacy_mode=True, B=3, K=2, seed=7)
    assert lg2.B == 1 and lg2.K == K_INF and lg2.seed == 7
    assert lg2 == dataclasses.replace(lg, seed=7)
    assert PlannerConfig(K=math.inf).K == K_INF   # type: ignore[arg-type]


def test_planner_config_validation_and_json():
    with pytest.raises(ValueError):
        PlannerConfig(nu=(0.5, 0.5))
    with pytest.raises(ValueError):
        PlannerConfig(transfer_types=("foo",))
    with pytest.raises(ValueError):
        PlannerConfig(B=0)
    with pytest.raises(ValueError):
        PlannerConfig(idp_branching="random")   # type: ignore[arg-type]
    with pytest.raises(ValueError):
        PlannerConfig.from_dict({"bogus": 1})
    cfg = PlannerConfig(K=4, transfer_types=["rev"], nu=[1, 1, 1, 1, 1, 1])   # type: ignore[arg-type]
    assert cfg.transfer_types == ("rev",) and cfg.nu == (1.0,) * 6
    d = json.loads(cfg.to_json())
    assert PlannerConfig.from_dict(d) == cfg
    lg = PlannerConfig().apply_legacy()
    assert PlannerConfig.from_dict(json.loads(lg.to_json())) == lg
    assert PlannerConfig.from_dict({"K": "inf", "h_ceil_override_m": "inf"}).K == K_INF


# ---------------------------------------------------------------------------
# 6. 列舉與模式不變量
# ---------------------------------------------------------------------------

def test_mode_phase_enums_and_fw_phases():
    assert [m.name for m in Mode] == ["HOVER", "TRANSITION", "CRUISE"]
    assert [p.name for p in Phase] == ["TAKEOFF", "TRANS_OUT", "ACCEL", "CLIMB", "ORBIT", "ENTRY", "OPERATE", "TRANSFER",
                                       "RTB", "DESCENT", "DECEL", "TRANS_IN", "LAND", "HOLD"]
    assert FW_PHASES == frozenset({Phase.ACCEL, Phase.CLIMB, Phase.ORBIT, Phase.ENTRY, Phase.OPERATE, Phase.TRANSFER,
                                   Phase.RTB, Phase.DESCENT, Phase.DECEL, Phase.HOLD})
    assert FW_PHASES | VTOL_PHASES | TRANSITION_PHASES == frozenset(Phase)
    assert FW_PHASES.isdisjoint(VTOL_PHASES) and FW_PHASES.isdisjoint(TRANSITION_PHASES)


def test_mode_phase_violation_helper():
    assert mode_phase_violation(Mode.HOVER, Phase.TAKEOFF, 0.0, 18.0) is None
    assert mode_phase_violation(Mode.HOVER, Phase.OPERATE, 0.0, 18.0)          # A7 無懸停轉向
    assert mode_phase_violation(Mode.CRUISE, Phase.OPERATE, 21.5, 18.0) is None
    assert mode_phase_violation(Mode.CRUISE, Phase.OPERATE, 15.0, 18.0)        # airspeed < v_min_mission
    assert mode_phase_violation(Mode.TRANSITION, Phase.OPERATE, 21.5, 18.0)
    assert mode_phase_violation(Mode.TRANSITION, Phase.TRANS_OUT, 16.0, 18.0) is None
    assert mode_phase_violation(Mode.CRUISE, Phase.TRANS_IN, 21.5, 18.0)


# ---------------------------------------------------------------------------
# 7. load_mission：dict、JSON 字串、檔案、GeoJSON
# ---------------------------------------------------------------------------

def test_load_mission_from_dict(mission: MissionInputs):
    m = mission
    assert m.home.site_id == "H" and m.home.kind == "home" and m.alternates[0].kind == "alternate"
    assert m.alternates[0].alt_agl_m == 0.0 and m.alternates[0].approach_heading_deg == 270.0
    assert m.areas[0].area_id == "Z1" and m.areas[0].polygon_wgs84 == tuple(tuple(p) for p in SQUARE)
    assert m.areas[0].time_window_s == (0.0, 5400.0)
    assert isinstance(m.dem, DemGrid) and m.dem.z_msl.dtype == np.float32 and m.dem.sigma_z_m == 2.0
    assert [o.geom_type for o in m.obstacles] == ["point", "line"] and m.obstacles[1].top_msl_m == 130.0
    assert len(m.wind) == 2 and m.wind[1].z_ref == "agl"
    a1, b1 = m.fleet
    assert a1.spec is PLATFORM_PIONEER and b1.spec is PLATFORM_HERO
    assert a1.available_kinds == frozenset({"lidar"}) and b1.available_kinds == frozenset({"rgb"})
    assert a1.sortie.uav_id == "A1" and a1.sortie.available_wh == 1350.0 and b1.sortie.battery_temp_c == 22.0
    assert set(m.payload_reqs) == {"default", "default_rgb"} and m.payload_reqs["default"].h_target_agl_m == 160.0
    assert m.c2.range_km == 25.0 and m.c2.ground_station is m.home and m.c2.policy_for("A1", "A") == "CONTINUE_THEN_RTB"
    assert m.airspace.max_agl_m == 120.0 and m.airspace.waiver_max_agl_m == 200.0 and m.airspace.terrain_clearance_m == 36.0
    assert m.airspace.no_fly[0].zone_id == "NF1" and m.airspace.no_fly[0].active_window_s == (0.0, 600.0)
    assert m.weather_thresholds.go_gust_mps == 12.0 and m.weather_obs.temp_c == 25.0
    assert m.platform_overrides == {"A": {"terrain_clearance_m": 36.0}}
    assert m.recovery_site.site_id == "H" and m.fleet_classes == frozenset({"A", "B"})
    assert m.site_by_id("ALT1").site_id == "ALT1" and m.uav_by_id("B1") is b1 and m.area_by_id("Z1") is m.areas[0]
    assert [u.uav_id for u in m.uavs_of_class("B")] == ["B1"]


def test_load_mission_missing_and_unknown_keys():
    d = mission_dict()
    del d["fleet"]
    with pytest.raises(MissionInputError, match="fleet"):
        load_mission(d)
    d = mission_dict()
    d["alternate"] = d.pop("alternates")
    with pytest.raises(MissionInputError, match="alternate"):
        load_mission(d)
    d = mission_dict()
    d["fleet"][0]["sortie"]["avail_wh"] = 1.0
    with pytest.raises(MissionInputError, match="avail_wh"):
        load_mission(d)
    d = mission_dict()
    d["fleet"][0]["cls"] = "C"
    with pytest.raises(MissionInputError):
        load_mission(d)
    with pytest.raises(MissionInputError):
        load_mission(42)   # type: ignore[arg-type]
    with pytest.raises(MissionInputError):
        load_mission("{not json")
    with pytest.raises(MissionInputError):
        load_mission("no/such/file.json")


def test_load_mission_defaults_when_optional_missing():
    d = mission_dict()
    for k in ("c2", "airspace", "weather_thresholds", "obstacles", "wind", "alternates", "platform_overrides", "recovery_site_id"):
        d.pop(k, None)
    m = load_mission(d)
    assert m.c2.range_km is None and m.c2.range_km_for("A", A) is None      # → preflight NO_GO（D11）
    assert m.c2.ground_station is m.home and m.airspace.ceiling_defaulted and m.airspace.terrain_clearance_m is None
    assert m.weather_thresholds == WeatherThresholds() and m.obstacles == () and m.wind == () and m.alternates == ()
    assert m.recovery_site is m.home and m.platform_overrides == {}


def test_load_mission_roundtrip_dict_and_json(mission: MissionInputs):
    d1 = mission.to_dict()
    json.dumps(d1)                                              # JSON 可序列化
    m2 = load_mission(d1)
    assert m2.to_dict() == d1
    m3 = load_mission(mission.to_json())
    assert m3.to_dict() == d1
    for m in (m2, m3):
        assert m.home == mission.home and m.alternates == mission.alternates and m.areas == mission.areas
        assert m.obstacles == mission.obstacles and m.wind == mission.wind
        assert m.c2 == mission.c2 and m.airspace == mission.airspace
        assert m.weather_thresholds == mission.weather_thresholds and m.weather_obs == mission.weather_obs
        assert m.payload_reqs == mission.payload_reqs and m.platform_overrides == mission.platform_overrides
        for u, u0 in zip(m.fleet, mission.fleet):
            assert u.uav_id == u0.uav_id and u.spec == u0.spec and u.sensors == u0.sensors and u.sortie == u0.sortie
        assert np.array_equal(m.dem.z_msl, mission.dem.z_msl) and m.dem.sigma_z_m == mission.dem.sigma_z_m
        assert (m.dem.lat0_deg, m.dem.lon0_deg, m.dem.dlat_deg, m.dem.dlon_deg) == (23.9, 120.85, 0.01, 0.01)


def test_load_mission_from_file(tmp_path: Path, mission: MissionInputs):
    p = tmp_path / "mission.json"
    p.write_text(mission.to_json(indent=1), encoding="utf-8")
    m = load_mission(str(p))
    assert m.to_dict() == mission.to_dict()
    assert load_mission(p).to_dict() == mission.to_dict()


def test_load_mission_accepts_dataclass_instances_and_full_spec_dict():
    d = mission_dict()
    spec_override = dataclasses.replace(A, c2_link_range_km=30.0, terrain_clearance_m=40.0)
    d["fleet"][0]["spec"] = spec_override
    d["fleet"][0].pop("cls")
    d["fleet"][0]["sensors"] = [LidarHardware(point_rate_hz=1.2e6, is_default_example=False)]
    d["fleet"][1]["spec"] = to_jsonable(PLATFORM_HERO)          # 完整欄位 mapping（to_dict 形式）
    d["home"] = Site("H", 23.95, 120.9, kind="home")
    d["dem"] = _dem()
    d["weather_obs"] = WeatherObs(5, 7, 270, 8000, 0, 25)
    m = load_mission(d)
    assert m.fleet[0].spec == spec_override and m.fleet[0].sensors[0].point_rate_hz == 1.2e6
    assert m.fleet[1].spec == PLATFORM_HERO and m.dem is d["dem"] and m.home is d["home"]
    assert m.c2.range_km_for("A", m.fleet[0].spec) == 25.0         # mission 值優先


def test_geojson_polygon_with_hole_swaps_lonlat():
    d = mission_dict()
    d["areas"] = {
        "type": "FeatureCollection",
        "features": [{
            "type": "Feature", "id": 7,
            "properties": {"area_id": "Z2", "sensor_req": "rgb", "priority": 2, "payload_req_id": "default_rgb",
                           "name": "ignored", "surveyor": "x"},
            "geometry": {"type": "Polygon", "coordinates": [
                [[120.94, 23.945], [120.97, 23.945], [120.97, 23.955], [120.94, 23.955], [120.94, 23.945]],
                [[120.95, 23.948], [120.955, 23.948], [120.955, 23.952], [120.95, 23.948]],
            ]},
        }],
    }
    m = load_mission(d)
    (a,) = m.areas
    assert a.area_id == "Z2" and a.sensor_req == "rgb" and a.priority == 2.0 and a.payload_req_id == "default_rgb"
    assert a.polygon_wgs84 == ((23.945, 120.94), (23.945, 120.97), (23.955, 120.97), (23.955, 120.94))
    assert a.holes_wgs84 == (((23.948, 120.95), (23.948, 120.955), (23.952, 120.955)),)
    poly = shapely.Polygon([(lon, lat) for lat, lon in a.polygon_wgs84], [[(lon, lat) for lat, lon in a.holes_wgs84[0]]])
    assert poly.is_valid and poly.area < 0.03 * 0.01


def test_geojson_multipolygon_split():
    d = mission_dict()
    mp = {"type": "MultiPolygon", "coordinates": [
        [[[120.94, 23.945], [120.95, 23.945], [120.95, 23.955], [120.94, 23.955], [120.94, 23.945]]],
        [[[120.96, 23.945], [120.97, 23.945], [120.97, 23.955], [120.96, 23.955], [120.96, 23.945]],
         [[120.962, 23.948], [120.965, 23.948], [120.965, 23.951], [120.962, 23.948]]],
    ]}
    d["areas"] = [
        {"type": "Feature", "properties": {"area_id": "Z3", "priority": 3}, "geometry": mp},
        {"area_id": "Z4", "geometry": {"type": "Polygon", "coordinates": [[[120.9, 23.9], [120.91, 23.9], [120.91, 23.91]]]},
         "sensor_req": "lidar"},
        dict(mp, area_id="Z5"),                       # 裸 GeoJSON 幾何 + area_id
        {"type": "Polygon", "coordinates": [[[120.9, 23.9], [120.91, 23.9], [120.91, 23.91]]]},   # 無 id → area_4
    ]
    m = load_mission(d)
    ids = [a.area_id for a in m.areas]
    assert ids == ["Z3_1", "Z3_2", "Z4", "Z5_1", "Z5_2", "area_4"]
    z31, z32 = m.areas[0], m.areas[1]
    assert z31.priority == 3.0 and z32.priority == 3.0 and z31.holes_wgs84 == () and len(z32.holes_wgs84) == 1
    assert z32.polygon_wgs84[0] == (23.945, 120.96)
    assert m.areas[2].sensor_req == "lidar" and m.areas[2].polygon_wgs84[0] == (23.9, 120.9)
    with pytest.raises(MissionInputError):
        d["areas"] = [{"area_id": "P", "geometry": {"type": "Point", "coordinates": [120.9, 23.9]}}]
        load_mission(d)


def test_geojson_obstacles_nofly_coverage_and_sites():
    d = mission_dict()
    d["obstacles"] = {"type": "FeatureCollection", "features": [
        {"type": "Feature", "properties": {"obs_id": "PL2", "top_msl_m": 140.0, "category": "powerline"},
         "geometry": {"type": "LineString", "coordinates": [[120.92, 23.94], [120.93, 23.94], [120.94, 23.941]]}},
        {"type": "Feature", "properties": {"height_agl_m": 30.0, "category": "building", "owner": "x"},
         "geometry": {"type": "Polygon", "coordinates": [[[120.92, 23.95], [120.921, 23.95], [120.921, 23.951], [120.92, 23.95]]]}},
        {"type": "Feature", "properties": {"obs_id": "T9", "height_agl_m": 50.0, "category": "tower"},
         "geometry": {"type": "Point", "coordinates": [120.925, 23.955]}},
    ]}
    d["airspace"]["no_fly"] = [{"type": "Feature", "properties": {"zone_id": "NFG", "active_window_s": [0, 100]},
                               "geometry": {"type": "Polygon", "coordinates": [[[120.90, 23.94], [120.91, 23.94], [120.91, 23.945]]]}}]
    d["c2"]["coverage_polygon_wgs84"] = {"A": {"type": "Polygon", "coordinates": [[[120.8, 23.9], [121.0, 23.9], [121.0, 24.0], [120.8, 24.0]]]},
                                         "B": SQUARE}
    d["c2"]["ground_station"] = {"type": "Feature", "properties": {"site_id": "GS"},
                                 "geometry": {"type": "Point", "coordinates": [120.9, 23.95, 100.0]}}
    d["alternates"].append({"type": "Feature", "properties": {"site_id": "ALT2", "clear_radius_m": 10},
                            "geometry": {"type": "Point", "coordinates": [120.93, 23.96]}})
    m = load_mission(d)
    o = m.obstacles
    assert [x.obs_id for x in o] == ["PL2", "obs_2", "T9"] and [x.geom_type for x in o] == ["line", "polygon", "point"]
    assert o[0].coords_wgs84[0] == (23.94, 120.92) and o[2].coords_wgs84 == ((23.955, 120.925),)
    assert m.airspace.no_fly[0].zone_id == "NFG" and m.airspace.no_fly[0].polygon_wgs84[0] == (23.94, 120.90)
    assert m.c2.coverage_for("A")[0] == (23.9, 120.8) and m.c2.coverage_for("B") == tuple(tuple(p) for p in SQUARE)
    assert m.c2.ground_station.site_id == "GS" and m.c2.ground_station.alt_msl_m == 100.0 and m.c2.ground_station.kind == "home"
    assert m.alternates[1].site_id == "ALT2" and m.alternates[1].clear_radius_m == 10.0 and m.alternates[1].kind == "alternate"
    d["c2"]["ground_station"] = "ALT1"
    assert load_mission(d).c2.ground_station.site_id == "ALT1"


# ---------------------------------------------------------------------------
# 8. MissionInputs 驗證
# ---------------------------------------------------------------------------

def test_mission_inputs_validation_errors():
    base = load_mission(mission_dict())
    with pytest.raises(MissionInputError, match="area_id"):
        dataclasses.replace(base, areas=base.areas + base.areas)
    with pytest.raises(MissionInputError, match="payload_req_id"):
        dataclasses.replace(base, areas=(AreaSpec("Z9", SQUARE, payload_req_id="nope"),))
    with pytest.raises(MissionInputError, match="recovery_site_id"):
        dataclasses.replace(base, recovery_site_id="XX")
    with pytest.raises(MissionInputError, match="cruise_airspeed_mps"):
        dataclasses.replace(base, platform_overrides={"A": {"cruise_airspeed_mps": 25.0}})
    with pytest.raises(MissionInputError):
        dataclasses.replace(base, platform_overrides={"C": {"terrain_clearance_m": 40.0}})
    with pytest.raises(MissionInputError, match="lost_link_policy"):
        dataclasses.replace(base, c2=dataclasses.replace(base.c2, lost_link_policy={"Z9": "RTB"}))
    with pytest.raises(MissionInputError):
        dataclasses.replace(base, fleet=())
    with pytest.raises(MissionInputError):
        dataclasses.replace(base, fleet=base.fleet + (base.fleet[0],))
    with pytest.raises(MissionInputError):
        dataclasses.replace(base, alternates=(Site("H", 23.0, 120.0),))
    with pytest.raises(MissionInputError):
        dataclasses.replace(base, payload_reqs={"other": base.payload_reqs["default"]})
    ok = dataclasses.replace(base, platform_overrides={"A": {"min_fw_operation_agl_m": 130.0, "t_hold_s": 6.0},
                                                       "B": {"payload_max_kg": 0.9, "crosswind_site_max_mps": [5.0, 6.0]}})
    assert ok.platform_overrides["B"]["crosswind_site_max_mps"] == (5.0, 6.0)
    ok2 = dataclasses.replace(base, alternates=base.alternates[0], recovery_site_id="ALT1")   # 單一 Site → tuple
    assert ok2.alternates == base.alternates and ok2.recovery_site.site_id == "ALT1"
    assert M.allowed_override_keys(A) >= {"terrain_clearance_m", "c2_link_range_km", "battery_nominal_wh", "min_fw_operation_agl_m"}
    assert "payload_max_kg" in M.allowed_override_keys(B) and "payload_max_kg" not in M.allowed_override_keys(A)


# ---------------------------------------------------------------------------
# 9. 輸出型別 JSON 往返
# ---------------------------------------------------------------------------

def _wp(**kw) -> Waypoint:
    base = dict(x_m=100.0, y_m=-50.0, z_m=260.0, lat_deg=23.95, lon_deg=120.9, alt_msl_m=360.0, agl_m=160.0, t_s=12.5,
                mode=Mode.CRUISE, phase=Phase.OPERATE, sensor_on=True, strip_id="Z1_A_1", cell_id="Z1", dir=-1,
                airspeed_mps=21.5, ground_speed_mps=26.5, track_deg=90.0, heading_deg=90.0, energy_remaining_wh=1200.0,
                nearest_alternate="ALT1", alt_reachable=True, lost_link_action="CONTINUE", arc=(0.0, 0.0, 200.0, 1))
    base.update(kw)
    return Waypoint(**base)


def test_waypoint_json_roundtrip():
    w = _wp()
    d = w.to_dict()
    assert d["mode"] == "CRUISE" and d["phase"] == "OPERATE" and d["arc"] == [0.0, 0.0, 200.0, 1]
    s = json.dumps(d)
    w2 = Waypoint.from_dict(json.loads(s))
    assert w2 == w and w2.mode is Mode.CRUISE and w2.arc == (0.0, 0.0, 200.0, 1)
    w3 = Waypoint.from_dict(json.loads(json.dumps(_wp(arc=None, strip_id=None, dir=None, mode=Mode.HOVER, phase=Phase.LAND).to_dict())))
    assert w3.arc is None and w3.dir is None and w3.mode is Mode.HOVER and w3.accept_radius_m == 30.0
    assert to_jsonable(w) == d


def test_uav_plan_and_feasibility_json_roundtrip():
    plan = UAVPlan("A1", "A", [_wp(), _wp(t_s=20.0, phase=Phase.TRANSFER, sensor_on=False)], [("Z1_A_1", -1), ("Z1_A_4", 1)],
                   0.0, 1586.7, 1696.7, "H", 389.6, 510.0, 48800.0, 48.8, 0, ["planned_landing_at_alternate"])
    p2 = UAVPlan.from_dict(json.loads(plan.to_json()))
    assert p2 == plan and p2.strips == [("Z1_A_1", -1), ("Z1_A_4", 1)]
    rep = FeasibilityReport("GO_WITH_WARNINGS", [], ["provisional_params"], {"Z1": {"A": True}},
                            {"A1": {"e_usable_wh": np.float64(779.6)}}, {"A": {"R_eff": np.float64(200.0), "h_ceil_source": "waiver"}})
    r2 = FeasibilityReport.from_dict(json.loads(rep.to_json()))
    assert r2.status == "GO_WITH_WARNINGS" and r2.ok and r2.per_uav["A1"]["e_usable_wh"] == 779.6 and r2.derived["A"]["R_eff"] == 200.0
    assert not FeasibilityReport("NO_GO", ["c2_link_range_km"], [], {}, {}, {}).ok


@dataclasses.dataclass(slots=True)
class _FakeCell:                      # geometry.Cell 之替身（shapely + numpy 欄位）
    cell_id: str
    polygon: shapely.Polygon
    sweep_origin: dict
    feasible: dict


def test_mission_plan_to_dict_json_serialisable():
    cell = _FakeCell("Z1", shapely.box(4000, -600, 7000, 600), {"A": np.array([3985.0, 615.0])}, {"A": True, "B": False})
    rep = FeasibilityReport("GO", [], [], {}, {}, {})
    plan = UAVPlan("A1", "A", [_wp()], [("s1", -1)], 0.0, 1.0, 2.0, "H", 1.0, 1.0, 1.0, 1.0, 0, [])
    mp = MissionPlan({"A1": plan}, 1696.7, [cell], {"s1": cell}, [("s9", "energy")], rep, {"total": np.float64(12.0)}, (1, 2, 3))
    d = mp.to_dict()
    s = mp.to_json()
    back = json.loads(s)
    assert back["makespan_s"] == 1696.7 and back["cache_keys"] == [1, 2, 3] and back["uncovered"] == [["s9", "energy"]]
    assert back["cells"][0]["polygon"]["type"] == "Polygon" and back["cells"][0]["sweep_origin"]["A"] == [3985.0, 615.0]
    assert back["strips"]["s1"]["cell_id"] == "Z1" and back["plans"]["A1"]["waypoints"][0]["phase"] == "OPERATE"
    assert d["timing_ms"] == {"total": 12.0} and back["diff_from"] is None


def test_to_jsonable_generic():
    out = to_jsonable({("A", "default"): frozenset({"rgb", "lidar"}), Mode.HOVER: np.arange(3), "e": Phase.RTB,
                       "n": np.float32(1.5), "p": shapely.Point(1, 2), "t": (1, 2), "path": Path("x")})
    assert out == {"A|default": ["lidar", "rgb"], "HOVER": [0, 1, 2], "e": "RTB", "n": 1.5,
                   "p": {"type": "Point", "coordinates": [1.0, 2.0]}, "t": [1, 2], "path": "x"}
    json.dumps(out)
    st = UAVState(1.0, 2.0, 0.5, 200.0, 0.0, 1300.0, 200.0, 21.5)
    assert st.copy() == st and st.copy() is not st
    np.testing.assert_array_equal(st.as_pose_array(), [1.0, 2.0, 0.5])
    assert json.loads(json.dumps(st.to_dict()))["e_rem_wh"] == 1300.0


# ---------------------------------------------------------------------------
# 10. 逐機起飛站點 UAVInstance.start_site_id（選填；None → home）
# ---------------------------------------------------------------------------

def test_start_site_id_defaults_to_home(mission: MissionInputs):
    """未設定 start_site_id → 所有機起飛站點皆為 home（既有情境行為不變）。"""
    assert all(u.start_site_id is None for u in mission.fleet)
    assert mission.start_sites == {"A1": "H", "B1": "H"}
    assert mission.start_site_id_of("A1") == "H"
    assert mission.start_site("B1") is mission.home


def test_start_site_id_resolves_alternate_site(mission: MissionInputs):
    """start_site_id 指向 alternates 之一 → start_site 回傳該站；回收點不受影響（D40）。"""
    fleet = (dataclasses.replace(mission.fleet[0], start_site_id="ALT1"), mission.fleet[1])
    m = dataclasses.replace(mission, fleet=fleet)
    assert m.start_sites == {"A1": "ALT1", "B1": "H"}
    assert m.start_site("A1").site_id == "ALT1" and m.start_site("A1") is not m.home
    assert m.start_site_id_of("B1") == "H"
    assert m.recovery_site.site_id == "H"                       # D40：起飛點改變不動回收點


def test_start_site_id_unknown_site_rejected(mission: MissionInputs):
    with pytest.raises(MissionInputError, match="start_site_id"):
        dataclasses.replace(mission, fleet=(dataclasses.replace(mission.fleet[0], start_site_id="NOPE"),
                                            mission.fleet[1]))
    with pytest.raises(MissionInputError, match="start_site_id"):
        UAVInstance("A1", A, (LidarHardware(),), _sortie(), start_site_id="")


def test_start_site_id_json_roundtrip(mission: MissionInputs):
    """to_dict / load_mission 往返保留 start_site_id（未設定時輸出 None）。"""
    m = dataclasses.replace(mission, fleet=(dataclasses.replace(mission.fleet[0], start_site_id="ALT1"),
                                            mission.fleet[1]))
    d = m.to_dict()
    assert [u["start_site_id"] for u in d["fleet"]] == ["ALT1", None]
    back = load_mission(json.loads(m.to_json()))
    assert back.start_sites == {"A1": "ALT1", "B1": "H"}
    assert back.to_dict() == d


def test_load_mission_accepts_start_site_id_key():
    """JSON 輸入之 fleet[i].start_site_id 直接解析；省略／null 皆退回 home。"""
    d = mission_dict()
    d["fleet"][0]["start_site_id"] = "ALT1"
    d["fleet"][1]["start_site_id"] = None
    m = load_mission(d)
    assert m.start_sites == {"A1": "ALT1", "B1": "H"}
    d["fleet"][0]["start_site_id"] = "ALT_MISSING"
    with pytest.raises(MissionInputError, match="start_site_id"):
        load_mission(d)
