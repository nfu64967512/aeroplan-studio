"""tests/test_preflight.py — preflight.preflight（契約 §3A；規格 §6.2 步 1–13、D7、D8、D11、D12）。

情境（inline，仿契約 §W.1 mini example）：平地 100 m MSL、home (23.95°N, 120.90°E)、備降點 ENU (2000, 1500)、
矩形區域 ENU (4000,−600)–(7000,600)、單層西風 5 m/s（陣風 7）、A1（PIONEER + LiDAR、1350/1500 Wh）＋
B1（HERO + 相機、380/420 Wh）、C2 25 km、空域 120／豁免 200／淨空 36、WeatherObs 5/7 m/s、8 km、25 °C。
`PlannerConfig(dem_step_m=50)` 使每次 build_context < 0.3 s。
"""
from __future__ import annotations

import dataclasses
import json
import math

import numpy as np
import pytest

from hv_dccpp.context import build_context
from hv_dccpp.geo import DemGrid, GeoFrame
from hv_dccpp.mission import (
    Airspace,
    AreaSpec,
    C2Link,
    FeasibilityReport,
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
from hv_dccpp.preflight import (
    ENDURANCE_RANGE_TOL,
    EXAMPLE_BATTERY_NOMINAL_WH,
    check_required_inputs,
    check_sites,
    check_weather,
    preflight,
)

A = PLATFORM_PIONEER
B = PLATFORM_HERO

LAT0, LON0, Z0 = 23.95, 120.90, 100.0
REF = GeoFrame(LAT0, LON0, Z0)
CFG = PlannerConfig(dem_step_m=50.0)


def _wgs(x: float, y: float) -> tuple[float, float]:
    lat, lon, _ = REF.to_wgs84(np.array([x, y, 0.0]))[0]
    return (float(lat), float(lon))


def _ring(*xy: tuple[float, float]) -> tuple[tuple[float, float], ...]:
    return tuple(_wgs(x, y) for x, y in xy)


def _dem(sigma: float | None = 2.0, n: int = 25) -> DemGrid:
    return DemGrid(23.85, 120.78, 0.01, 0.01, np.full((n, n), Z0, dtype=np.float32), sigma_z_m=sigma)


AREA_RING = _ring((4000, -600), (7000, -600), (7000, 600), (4000, 600))
HOME = Site("H", LAT0, LON0, kind="home")
ALT1 = Site("ALT1", *_wgs(2000, 1500))
REQS = {
    "default": PayloadRequirement("default", "lidar", density_pt_m2=20, side_overlap=0.25, h_target_agl_m=160),
    "default_rgb": PayloadRequirement("default_rgb", "rgb", gsd_required_m=0.05, side_overlap=0.30,
                                      forward_overlap=0.70, h_target_agl_m=100),
}


def _a1(**kw) -> UAVInstance:
    s = dict(payload_kg=5.0, available_wh=1350.0, pack_nominal_wh=1500.0)
    s.update(kw)
    spec = s.pop("spec", A)
    auw = s.pop("auw_kg", 24.6)
    return UAVInstance("A1", spec, (LidarHardware(),), SortieEnergy("A1", auw, **s))


def _b1(**kw) -> UAVInstance:
    s = dict(payload_kg=0.6, available_wh=380.0, pack_nominal_wh=420.0)
    s.update(kw)
    spec = s.pop("spec", B)
    auw = s.pop("auw_kg", 6.65)
    return UAVInstance("B1", spec, (CameraHardware(),), SortieEnergy("B1", auw, **s))


def make_inputs(**kw) -> MissionInputs:
    d = dict(
        home=HOME, alternates=(ALT1,),
        areas=(AreaSpec("Z1", AREA_RING, sensor_req="any", payload_req_id="default"),),
        dem=_dem(), obstacles=(),
        wind=(WindLayer(0.0, 3000.0, 5.0, 270.0, gust_mps=7.0, z_ref="agl"),),
        fleet=(_a1(), _b1()),
        payload_reqs=REQS,
        c2=C2Link(25.0, None, HOME, {}),
        airspace=Airspace(max_agl_m=120.0, waiver_max_agl_m=200.0, terrain_clearance_m=36.0),
        weather_thresholds=WeatherThresholds(),
        weather_obs=WeatherObs(5, 7, 270, 8000, 0, 25),
    )
    d.update(kw)
    return MissionInputs(**d)


def _has(items: list[str], prefix: str) -> bool:
    return any(s.startswith(prefix) for s in items)


def _find(items: list[str], prefix: str) -> list[str]:
    return [s for s in items if s.startswith(prefix)]


@pytest.fixture(scope="module")
def mini_report() -> FeasibilityReport:
    return preflight(make_inputs(), CFG)


# ---------------------------------------------------------------------------
# 1. mini example → GO_WITH_WARNINGS（契約 §W.2）
# ---------------------------------------------------------------------------

def test_mini_status_and_warnings(mini_report: FeasibilityReport):
    r = mini_report
    assert isinstance(r, FeasibilityReport)
    assert r.status == "GO_WITH_WARNINGS" and r.ok
    assert r.errors == []
    w = r.warnings
    assert "provisional_params" in w
    assert _has(w, "provisional_params(A:") and _has(w, "provisional_params(B:")
    assert "sensor_spec_default_example(A)" in w and "sensor_spec_default_example(B)" in w
    assert "battery_nominal_example(A1)" in w and "battery_nominal_example(B1)" in w
    assert "reference_battery_assumed(A1)" in w and "reference_battery_assumed(B1)" in w
    assert _has(w, "battery_temp_assumed(A1)")
    assert _has(w, "avionics_unvalidated(A:") and "k_unc=1.2075" in _find(w, "avionics_unvalidated(A:")[0]
    # 不該出現
    assert not _has(w, "payload_estimated")
    assert not _has(w, "airspace_ceiling_defaulted_120")
    assert not _has(w, "endurance_range_inconsistent")
    assert not _has(w, "overridden(")
    assert not _has(w, "A_needs_waiver")
    assert not _has(w, "uncoverable(")
    assert not _has(w, "gust_above_public_wind_rating")
    assert len(w) == len(set(w))                           # 去重


def test_mini_derived_numbers(mini_report: FeasibilityReport):
    d = mini_report.derived
    assert set(d) == {"A", "B"}
    dA, dB = d["A"], d["B"]
    # R_eff（W=5 < W*）
    assert dA["R_eff"] == pytest.approx(200.0) and dB["R_eff"] == pytest.approx(120.0)
    assert dA["W_R"] == pytest.approx(5.0) and dA["W_star"] == pytest.approx(5.17, abs=0.01)
    # 地速（契約 §W.2）
    assert dA["V_g_tailwind"] == pytest.approx(26.5) and dA["V_g_headwind"] == pytest.approx(16.5)
    assert dA["V_g_crosswind"] == pytest.approx(20.91, abs=0.01)
    assert dB["V_g_tailwind"] == pytest.approx(24.0) and dB["V_g_headwind"] == pytest.approx(14.0)
    assert dB["V_g_crosswind"] == pytest.approx(18.33, abs=0.01)
    # 感測（§W.3）
    assert dA["h_op"] == pytest.approx(160.0) and dA["w"] == pytest.approx(224.07, abs=0.01)
    assert dA["s"] == pytest.approx(168.05, abs=0.01) and dA["e_ext"] == pytest.approx(53.0)
    assert dA["d_turn"] == pytest.approx(534.7, abs=0.1)
    assert dB["h_op"] == pytest.approx(100.0) and dB["w"] == pytest.approx(146.875, abs=0.01)
    assert dB["s"] == pytest.approx(102.81, abs=0.01) and dB["e_ext"] == pytest.approx(48.0)
    assert dB["d_turn"] == pytest.approx(336.0, abs=0.1)
    assert dB["V_g_cap"] == pytest.approx(29.25, abs=0.01)
    assert dA["V_g_cap"] > 60.0                                         # LiDAR 不綁
    assert "default" in dA["by_req"] and "default_rgb" in dB["by_req"]
    # 能量（§5.1、§5.3）
    assert dA["P_c_ref"] == pytest.approx(511.4, abs=0.1) and dB["P_c_ref"] == pytest.approx(185.3, abs=0.1)
    assert dA["k_unc"] == pytest.approx(1.2075) and dA["k_bm"] == 1.15 and dA["k_as"] == 1.05
    assert dA["e_TO"] == pytest.approx(39.2, abs=0.1) and dA["e_LDG"] == pytest.approx(81.2, abs=0.1)
    assert dA["E_use_rep"] == pytest.approx(779.6, abs=0.1) and dB["E_use_rep"] == pytest.approx(214.3, abs=0.1)
    assert dA["endurance_range_dev"] == pytest.approx(0.066, abs=0.002)
    assert dB["endurance_range_dev"] == pytest.approx(-0.071, abs=0.002)
    assert dA["D_use_km"] == 133.0 and dB["D_use_km"] == 79.0           # 未驗證取下緣（D8）
    # 高度上限與豁免
    assert dA["h_ceil_source"] == "waiver_max_agl_m" and dA["h_ceil"] == 200.0
    assert dA["A_needs_waiver"] is False and dB["A_needs_waiver"] is False
    assert dA["h_clr"] == 36.0 and dA["h_minfw"] == 120.0 and dB["h_minfw"] == 100.0
    # 轉換段（D8/D29/D37）
    assert dA["v_tr_cmd"] == 18.0 and dA["h_tc_cmd"] == 100.0 and dA["h_tc_clear"] == 80.0
    assert dA["d_tr"] == pytest.approx(468.0) and dA["d_acc"] == pytest.approx(138.25) and dA["t_acc"] == 7.0
    assert dA["slope_tr_deg"] == pytest.approx(4.89, abs=0.01)
    assert dB["v_tr_cmd"] == 16.0 and dB["d_tr"] == pytest.approx(384.0) and dB["slope_tr_deg"] == pytest.approx(5.95, abs=0.01)
    assert dA["t_takeoff_s"] == pytest.approx(57.0) and dA["t_landing_s"] == pytest.approx(117.0)
    assert dB["t_takeoff_s"] == pytest.approx(51.0) and dB["t_landing_s"] == pytest.approx(106.0)
    # 氣象、備降
    assert dA["W_go"] == 8.0 and dB["W_go"] == 8.0 and dA["beaufort_upper_mps"] == 10.7
    assert dA["usable_alternates"] == ["ALT1"] and dB["usable_alternates"] == ["ALT1"]
    assert dA["makespan_lb_s"] > 0 and math.isfinite(dA["makespan_lb_s"])
    assert "c2_link_range_km" in dA["provisional_params"] and "payload_max_kg" in dB["provisional_params"]


def test_mini_per_area_and_per_uav(mini_report: FeasibilityReport):
    pa = mini_report.per_area["Z1"]
    assert pa["coverable"] and pa["feasible_classes"] == ["A", "B"] and pa["uncoverable_reason"] is None
    assert pa["area_m2"] == pytest.approx(3000.0 * 1200.0, rel=2e-3)
    assert pa["dist_home_m"] == pytest.approx(4000.0, abs=1.0)
    for c in ("A", "B"):
        cr = pa["classes"][c]
        assert cr["feasible"] and cr["reason"] == "" and cr["band"]["feasible"]
        assert cr["coverable_fraction"] == pytest.approx(1.0, abs=1e-6)
        assert cr["direction_ok"] and cr["two_way_ok"] and cr["n_directions_ok"] == 72
        assert math.isfinite(cr["T_oper_lb_s"]) and cr["T_oper_lb_s"] > 0
    assert pa["classes"]["A"]["band"]["dh_band_m"] == pytest.approx(80.0, abs=5.0)     # 平地 80 − ENU 曲率起伏
    assert pa["classes"]["B"]["band"]["dh_band_m"] == pytest.approx(100.0, abs=5.0)
    pu = mini_report.per_uav
    assert set(pu) == {"A1", "B1"}
    assert pu["A1"]["ok"] and pu["B1"]["ok"] and pu["A1"]["issues"] == []
    assert pu["A1"]["E_use_wh"] == pytest.approx(779.6, abs=0.1) and pu["B1"]["E_use_wh"] == pytest.approx(214.3, abs=0.1)
    assert pu["A1"]["cruise_minutes"] == pytest.approx(75.8, abs=0.1) and pu["B1"]["cruise_minutes"] == pytest.approx(57.5, abs=0.1)
    assert pu["A1"]["lost_link_policy"] == "CONTINUE_THEN_RTB" and pu["B1"]["lost_link_policy"] == "RTB"
    assert pu["A1"]["payload_estimated"] is False and pu["A1"]["stall_margin_ok"]


def test_report_json_serialisable(mini_report: FeasibilityReport):
    s = mini_report.to_json()
    back = FeasibilityReport.from_dict(json.loads(s))
    assert back.status == mini_report.status and back.derived["A"]["R_eff"] == 200.0


def test_ctx_passed_in_gives_same_report(mini_report: FeasibilityReport):
    inputs = make_inputs()
    ctx = build_context(inputs, CFG)
    r2 = preflight(inputs, CFG, ctx)
    assert r2.status == mini_report.status and r2.errors == mini_report.errors and r2.warnings == mini_report.warnings
    assert r2.derived == mini_report.derived
    assert r2.per_area["Z1"]["feasible_classes"] == ["A", "B"]


# ---------------------------------------------------------------------------
# 2. 步 1 必填鍵 → NO_GO 並列鍵名（D11、§5.7）
# ---------------------------------------------------------------------------

def test_missing_c2_range_is_nogo_per_class():
    r = preflight(make_inputs(c2=C2Link(None, None, HOME, {})), CFG)
    assert r.status == "NO_GO" and not r.ok
    assert any("c2.range_km[A]" in e for e in r.errors)
    assert any("c2.range_km[B]" in e for e in r.errors)
    # 只給 A → 只列 B
    r = preflight(make_inputs(c2=C2Link({"A": 25.0}, None, HOME, {})), CFG)
    assert r.status == "NO_GO"
    assert any("c2.range_km[B]" in e for e in r.errors)
    assert not any("c2.range_km[A]" in e for e in r.errors)
    # coverage polygon 亦可滿足
    cov = _ring((-9000, -9000), (9000, -9000), (9000, 9000), (-9000, 9000))
    r = preflight(make_inputs(c2=C2Link(None, cov, HOME, {})), CFG)
    assert r.status != "NO_GO"
    # spec 覆寫槽
    r = preflight(make_inputs(c2=C2Link(None, None, HOME, {}),
                              platform_overrides={"A": {"c2_link_range_km": 25.0}, "B": {"c2_link_range_km": 25.0}}), CFG)
    assert r.status != "NO_GO" and _has(r.warnings, "overridden(A.c2_link_range_km, None→25.0)")


def test_missing_terrain_clearance_is_nogo():
    r = preflight(make_inputs(airspace=Airspace(max_agl_m=120.0, waiver_max_agl_m=200.0)), CFG)
    assert r.status == "NO_GO"
    assert any("terrain_clearance_m" in e for e in r.errors)
    # spec 覆寫槽解析
    r = preflight(make_inputs(airspace=Airspace(max_agl_m=120.0, waiver_max_agl_m=200.0),
                              platform_overrides={"A": {"terrain_clearance_m": 40.0}, "B": {"terrain_clearance_m": 40.0}}), CFG)
    assert r.status != "NO_GO"
    assert not any("terrain_clearance_m" in e for e in r.errors)
    assert r.derived["A"]["h_clr"] == 40.0


def test_no_alternates_when_required_is_nogo():
    r = preflight(make_inputs(alternates=()), CFG)
    assert r.status == "NO_GO"
    assert any(e.startswith("alternates_missing(") and "alternates" in e for e in r.errors)
    assert not _has(r.errors, "no_usable_alternate")
    # 報告其餘部分仍完整（build_context 成功）
    assert "Z1" in r.per_area and set(r.derived) == {"A", "B"}
    # 不要求備降點之 spec → 通過
    a_nr = dataclasses.replace(A, alternate_landing_sites_required=False)
    b_nr = dataclasses.replace(B, alternate_landing_sites_required=False)
    r = preflight(make_inputs(alternates=(), fleet=(_a1(spec=a_nr), _b1(spec=b_nr))), CFG)
    assert r.status != "NO_GO"


def test_wh_without_nominal_is_nogo():
    r = preflight(make_inputs(fleet=(_a1(pack_nominal_wh=None), _b1())), CFG)
    assert r.status == "NO_GO"
    assert any(e.startswith("energy_unresolved(A1)") and "pack_nominal_wh/battery_nominal_wh" in e for e in r.errors)
    # mAh 缺電壓
    r = preflight(make_inputs(fleet=(_a1(available_wh=None, available_mah=30000.0), _b1())), CFG)
    assert r.status == "NO_GO" and any("pack_voltage_nom_v" in e for e in r.errors)
    # 僅 soc 且 nominal None → 基準分鐘模式，非錯誤
    r = preflight(make_inputs(fleet=(_a1(available_wh=None, pack_nominal_wh=None, soc_fraction=0.9), _b1())), CFG)
    assert r.status != "NO_GO" and _has(r.warnings, "base_minute_mode(A1)")
    assert r.per_uav["A1"]["base_minute_mode"] is True and r.derived["A"]["base_minute_mode"] is True


def test_check_required_inputs_unit():
    inputs = make_inputs(c2=C2Link(None, None, HOME, {}), alternates=(),
                         airspace=Airspace(max_agl_m=120.0, waiver_max_agl_m=200.0))
    errors, _ = check_required_inputs(inputs, CFG, {"A": A, "B": B})
    joined = "\n".join(errors)
    assert "c2.range_km[A]" in joined and "c2.range_km[B]" in joined
    assert "terrain_clearance_m" in joined and "alternates" in joined
    # legacy（enforce_link=False）略過 C2
    errors, _ = check_required_inputs(inputs, PlannerConfig(legacy_mode=True), {"A": A, "B": B})
    assert not any("c2.range_km" in e for e in errors)


# ---------------------------------------------------------------------------
# 3. 步 2 氣象（D7、D8）
# ---------------------------------------------------------------------------

def test_wind_above_threshold_is_nogo():
    r = preflight(make_inputs(weather_obs=WeatherObs(9.0, 9.5, 270, 8000, 0, 25)), CFG)
    assert r.status == "NO_GO"
    assert any(e.startswith("weather_go(") and "mean_wind" in e for e in r.errors)
    # 覆寫門檻
    r = preflight(make_inputs(weather_obs=WeatherObs(9.0, 7.0, 270, 8000, 0, 25),
                              weather_thresholds=WeatherThresholds(go_mean_wind_mps=10.0)), CFG)
    assert not any("mean_wind" in e for e in r.errors)


@pytest.mark.parametrize("obs, key", [
    (WeatherObs(5, 13.0, 270, 8000, 0, 25), "gust"),
    (WeatherObs(5, 7, 270, 2000, 0, 25), "visibility"),
    (WeatherObs(5, 7, 270, 8000, 1.0, 25), "precip"),
    (WeatherObs(5, 7, 270, 8000, 0, 45), "temp"),
])
def test_weather_go_other_violations(obs: WeatherObs, key: str):
    errors, _, w_go = check_weather(make_inputs(weather_obs=obs), CFG, {"A": A, "B": B})
    assert any(key in e for e in errors)
    assert w_go == {"A": 8.0, "B": 8.0}


def test_beaufort_rating_warning():
    # 陣風 11 > Beaufort 5 上緣 10.7（警告）但 ≤ 12（放飛）；站點陣風限 8 m/s 使站點檢查失敗
    r = preflight(make_inputs(weather_obs=WeatherObs(5, 11.0, 270, 8000, 0, 25)), CFG)
    assert _has(r.warnings, "gust_above_public_wind_rating(A:") and _has(r.warnings, "gust_above_public_wind_rating(B:")
    assert not any("weather_go(" in e for e in r.errors)
    assert r.status == "NO_GO" and any("gust_exceeds" in e for e in r.errors)
    # 陣風 7 → 無警告
    _, warnings, _ = check_weather(make_inputs(), CFG, {"A": A, "B": B})
    assert not _has(warnings, "gust_above_public_wind_rating")


# ---------------------------------------------------------------------------
# 4. D12 高度帶／豁免旗標、空域上限預設
# ---------------------------------------------------------------------------

def test_A_needs_waiver_flag():
    inputs = make_inputs(airspace=Airspace(max_agl_m=120.0, terrain_clearance_m=36.0))
    # Δh_band,min=15 使 B（h_ceil 120 − h_minfw 100 − 曲率起伏 ≈ 17 m）仍可行
    r = preflight(inputs, dataclasses.replace(CFG, dh_band_min_m=15.0))
    assert r.derived["A"]["A_needs_waiver"] is True
    assert r.derived["B"]["A_needs_waiver"] is False
    assert r.derived["A"]["h_ceil"] == 120.0 and r.derived["A"]["h_ceil_source"] == "max_agl_m"
    assert _has(r.warnings, "A_needs_waiver(Z1")
    za = r.per_area["Z1"]["classes"]["A"]
    assert za["feasible"] is False and za["reason"] == "A_needs_waiver" and za["band"]["reason"] == "A_needs_waiver"
    assert r.per_area["Z1"]["classes"]["B"]["feasible"] is True
    assert r.per_area["Z1"]["feasible_classes"] == ["B"] and r.per_area["Z1"]["coverable"]
    assert r.status == "GO_WITH_WARNINGS"
    # 預設 Δh_band,min=20：B 亦不可行 → 無可行構型 → NO_GO
    r2 = preflight(inputs, CFG)
    assert r2.status == "NO_GO" and _has(r2.errors, "no_coverable_area")
    assert r2.derived["A"]["A_needs_waiver"] is True


def test_airspace_ceiling_defaulted_120():
    r = preflight(make_inputs(airspace=Airspace(terrain_clearance_m=36.0)), dataclasses.replace(CFG, dh_band_min_m=15.0))
    assert "airspace_ceiling_defaulted_120" in r.warnings
    assert r.derived["A"]["h_ceil_source"] == "default_120" and r.derived["A"]["h_ceil"] == 120.0
    assert r.derived["A"]["A_needs_waiver"] is True


def test_gsd_unattainable_flag():
    reqs = dict(REQS)
    reqs["default_rgb"] = PayloadRequirement("default_rgb", "rgb", gsd_required_m=0.02, side_overlap=0.30)
    r = preflight(make_inputs(payload_reqs=reqs), CFG)
    zb = r.per_area["Z1"]["classes"]["B"]
    assert zb["feasible"] is False and zb["reason"] == "gsd_unattainable"
    assert _has(r.warnings, "gsd_unattainable(Z1, B")
    assert r.per_area["Z1"]["feasible_classes"] == ["A"] and r.status == "GO_WITH_WARNINGS"


# ---------------------------------------------------------------------------
# 5. 步 6 機體
# ---------------------------------------------------------------------------

def test_payload_estimated_and_limits():
    r = preflight(make_inputs(fleet=(_a1(payload_kg=None), _b1())), CFG)
    assert _has(r.warnings, "payload_estimated(A1")
    assert r.per_uav["A1"]["payload_estimated"] is True and r.per_uav["A1"]["payload_kg"] == pytest.approx(5.0)
    assert r.status == "GO_WITH_WARNINGS"
    # B 酬載 1.0 kg：「1.0 以下」為嚴格 <
    r = preflight(make_inputs(fleet=(_a1(), _b1(payload_kg=1.0))), CFG)
    assert r.status == "NO_GO" and any(e.startswith("uav(B1): payload_exceeds_max") for e in r.errors)
    # A 酬載 5.0 kg 恰等於上限 → 通過；5.1 → NO_GO
    r = preflight(make_inputs(fleet=(_a1(payload_kg=5.1), _b1())), CFG)
    assert r.status == "NO_GO" and any(e.startswith("uav(A1): payload_exceeds_max") for e in r.errors)


def test_auw_exceeds_mtow_and_above_reference():
    r = preflight(make_inputs(fleet=(_a1(auw_kg=25.5), _b1())), CFG)
    assert r.status == "NO_GO" and any(e.startswith("uav(A1): auw_exceeds_mtow") for e in r.errors)
    assert _has(r.warnings, "auw_above_reference(A1")
    r = preflight(make_inputs(fleet=(_a1(auw_kg=24.9), _b1())), CFG)   # = mtow → 通過（D21 折減）
    assert not any("auw_exceeds_mtow" in e for e in r.errors) and _has(r.warnings, "auw_above_reference(A1")


def test_energy_usable_nonpositive_is_nogo():
    r = preflight(make_inputs(fleet=(_a1(available_wh=560.0), _b1())), CFG)   # 560 − 450 − 39 − 81 < 0
    assert r.status == "NO_GO" and any(e.startswith("uav(A1): energy_usable_nonpositive") for e in r.errors)
    assert r.per_uav["A1"]["E_use_wh"] < 0


# ---------------------------------------------------------------------------
# 6. 步 12 警告：endurance_range_inconsistent、overridden、battery_nominal_example
# ---------------------------------------------------------------------------

def test_endurance_range_inconsistent_warning():
    spec = dataclasses.replace(A, reference_total_range_km=300.0, usable_distance_km=(165.0, 180.0),
                               symmetric_mission_radius_km=(82.5, 90.0))
    r = preflight(make_inputs(fleet=(_a1(spec=spec), _b1())), CFG)
    assert _has(r.warnings, "endurance_range_inconsistent(A:")
    assert not _has(r.warnings, "endurance_range_inconsistent(B:")
    assert r.derived["A"]["endurance_range_dev"] > ENDURANCE_RANGE_TOL


def test_overridden_warning_and_provisional_removed():
    r = preflight(make_inputs(platform_overrides={"A": {"transition_start_agl_m": 70.0}}), CFG)
    assert "overridden(A.transition_start_agl_m, 60.0→70.0)" in r.warnings
    assert "transition_start_agl_m" not in r.derived["A"]["provisional_params"]
    assert "transition_start_agl_m" in r.derived["B"]["provisional_params"]
    assert r.derived["A"]["t_takeoff_s"] == pytest.approx(70.0 / 2.0 + 20.0 + 7.0)
    # 不允許之鍵 → NO_GO（MissionInputs 建構即拒絕）
    with pytest.raises(Exception):
        make_inputs(platform_overrides={"A": {"cruise_airspeed_mps": 25.0}})


def test_battery_nominal_example_only_for_example_values():
    r = preflight(make_inputs(fleet=(_a1(pack_nominal_wh=1400.0), _b1(pack_nominal_wh=420.0))), CFG)
    assert not _has(r.warnings, "battery_nominal_example(A1)")
    assert _has(r.warnings, "battery_nominal_example(B1)")
    assert EXAMPLE_BATTERY_NOMINAL_WH == {"A": 1500.0, "B": 420.0}


# ---------------------------------------------------------------------------
# 7. 步 7–9 區域：鏈路裁切、感測不相容、禁區
# ---------------------------------------------------------------------------

def test_link_clipping_partial_and_full():
    r = preflight(make_inputs(c2=C2Link(5.0, None, HOME, {})), CFG)      # 5 km ⊖ d_turn 535 → 4.47 km
    za = r.per_area["Z1"]["classes"]["A"]
    assert 0.0 < za["coverable_fraction"] < 0.5 and za["clip_loss"]["link"] > 0.5
    assert za["feasible"] and _has(r.warnings, "uncoverable(Z1/A: link")
    assert r.status == "GO_WITH_WARNINGS"
    r = preflight(make_inputs(c2=C2Link(3.0, None, HOME, {})), CFG)      # 區域全在鏈路外
    assert r.per_area["Z1"]["classes"]["A"]["reason"] == "outside_link"
    assert r.status == "NO_GO" and _has(r.errors, "no_coverable_area")


def test_sensor_incompatible_area_uncoverable():
    lidar_area = AreaSpec("Z1", AREA_RING, sensor_req="lidar", payload_req_id="default")
    r = preflight(make_inputs(areas=(lidar_area,), fleet=(_b1(),)), CFG)
    assert r.per_area["Z1"]["classes"]["B"]["reason"] == "sensor_incompatible"
    assert not r.per_area["Z1"]["coverable"] and _has(r.warnings, "uncoverable(Z1: sensor")
    assert r.status == "NO_GO" and _has(r.errors, "no_coverable_area")
    # 另一區可覆蓋 → 只警告
    rgb_ring = _ring((4000, -3000), (6000, -3000), (6000, -2000), (4000, -2000))
    rgb_area = AreaSpec("Z2", rgb_ring, sensor_req="rgb", payload_req_id="default_rgb")
    r = preflight(make_inputs(areas=(lidar_area, rgb_area), fleet=(_b1(),)), CFG)
    assert r.status == "GO_WITH_WARNINGS" and _has(r.warnings, "uncoverable(Z1: sensor")
    assert r.per_area["Z2"]["coverable"] and r.per_area["Z2"]["feasible_classes"] == ["B"]
    assert set(r.derived) == {"B"}


def test_both_requires_two_halves():
    both = AreaSpec("Z1", AREA_RING, sensor_req="both", payload_req_id="default")
    r = preflight(make_inputs(areas=(both,)), CFG)
    assert r.per_area["Z1"]["coverable"] and r.per_area["Z1"]["served_kinds"] == ["lidar", "rgb"]
    r = preflight(make_inputs(areas=(both,), fleet=(_a1(),)), CFG)         # 只有 A（LiDAR）→ 缺 rgb 半套
    assert not r.per_area["Z1"]["coverable"] and "rgb" in r.per_area["Z1"]["uncoverable_reason"]
    assert r.status == "NO_GO"


def test_nofly_clip_recorded():
    nf = NoFlyZone("NF1", _ring((5000, -200), (5400, -200), (5400, 200), (5000, 200)))
    r = preflight(make_inputs(airspace=Airspace(max_agl_m=120.0, waiver_max_agl_m=200.0, terrain_clearance_m=36.0,
                                                no_fly=(nf,))), CFG)
    za = r.per_area["Z1"]["classes"]["A"]
    # 400×400 禁區 ⊕ 50 m → 500×500 / 3.6e6 ≈ 6.9%
    assert za["clip_loss"]["nofly"] == pytest.approx(0.069, abs=0.005)
    assert _has(r.warnings, "uncoverable(Z1/A: nofly") and r.status == "GO_WITH_WARNINGS"


def test_site_check_failure_is_nogo():
    tower = Obstacle("T1", "point", (_wgs(5, 0),), None, 40.0, category="tower")   # home 淨空圓內
    r = preflight(make_inputs(obstacles=(tower,)), CFG)
    assert r.status == "NO_GO" and any(e.startswith("site_check(H, A)") and "obstacle_in_clear_radius" in e for e in r.errors)
    tower_alt = Obstacle("T2", "point", (_wgs(2005, 1500),), None, 40.0, category="tower")   # 備降點淨空圓內
    r = preflight(make_inputs(obstacles=(tower_alt,)), CFG)
    assert r.status == "NO_GO" and _has(r.errors, "no_usable_alternate(A)") and _has(r.warnings, "alternate_unusable(ALT1, A)")
    assert r.derived["A"]["usable_alternates"] == []


# ---------------------------------------------------------------------------
# 8. 步 10 時間窗（D22）
# ---------------------------------------------------------------------------

def test_time_window_disjoint_and_insufficient():
    airspace = Airspace(max_agl_m=120.0, waiver_max_agl_m=200.0, terrain_clearance_m=36.0, time_window_s=(0.0, 7200.0))
    area = AreaSpec("Z1", AREA_RING, sensor_req="any", payload_req_id="default", time_window_s=(8000.0, 9000.0))
    r = preflight(make_inputs(airspace=airspace, areas=(area,)), CFG)
    assert r.status == "NO_GO" and _has(r.errors, "time_window_disjoint(Z1")
    area2 = AreaSpec("Z1", AREA_RING, sensor_req="any", payload_req_id="default", time_window_s=(0.0, 60.0))
    r = preflight(make_inputs(airspace=airspace, areas=(area2,)), CFG)
    assert r.status == "NO_GO" and _has(r.errors, "time_window_insufficient(Z1")
    short = Airspace(max_agl_m=120.0, waiver_max_agl_m=200.0, terrain_clearance_m=36.0, time_window_s=(0.0, 300.0))
    r = preflight(make_inputs(airspace=short), CFG)
    assert r.status == "NO_GO" and _has(r.errors, "time_window_insufficient(airspace")
    # 足夠之窗 → 通過
    r = preflight(make_inputs(airspace=airspace), CFG)
    assert not _has(r.errors, "time_window") and r.derived["A"]["makespan_lb_s"] < 7200.0
    # enforce_time_windows=False（legacy）→ 不檢查
    r = preflight(make_inputs(airspace=short), dataclasses.replace(CFG, enforce_time_windows=False))
    assert not _has(r.errors, "time_window")


# ---------------------------------------------------------------------------
# 9. 效能
# ---------------------------------------------------------------------------

def test_preflight_fast():
    import time
    inputs = make_inputs()
    ctx = build_context(inputs, CFG)
    t = time.perf_counter()
    preflight(inputs, CFG, ctx)
    assert time.perf_counter() - t < 1.0


# ---------------------------------------------------------------------------
# 審查後新增：逐機起飛站點納入 primary（站點檢查失敗 → 錯誤，而非 alternate_unusable 警告）
# ---------------------------------------------------------------------------
def test_start_site_is_primary_per_class():
    """`UAVInstance.start_site_id` 生效後 planner 真的從該站起飛，故該站之站點檢查失敗必須是**錯誤**；

    但只對「真的從那裡起飛的構型」是硬條件（別的構型用不了該站不得使整個任務 NO_GO），且通過檢查之備降點
    即使同時是某機之起飛站，仍要算進 usable_alternates。
    """
    import dataclasses as _dc

    from hv_dccpp.scenario import build_mini_example

    inputs, cfg = build_mini_example()
    fleet = tuple(_dc.replace(u, start_site_id=("ALT1" if u.uav_id == "B1" else None)) for u in inputs.fleet)
    inp = _dc.replace(inputs, fleet=fleet)
    ctx = build_context(inp, cfg)
    errs, warns, usable = check_sites(ctx)
    assert errs == []                                       # ALT1 對 B 通過
    assert "ALT1" in usable["B"]                            # 仍算可用備降點
    assert not any(w.startswith("alternate_unusable(ALT1, B)") for w in warns)

    # 讓 ALT1 對 B 不可用（站點淨空圓內放一個高障礙）→ 必須是錯誤而不是警告
    blocked = _dc.replace(ctx, obstacles_enu=list(ctx.obstacles_enu))
    tall = [o for o in ctx.obstacles_enu]
    xy = ctx.sites_enu["ALT1"][0]
    from shapely.geometry import Point as _Pt

    from hv_dccpp.context import ObstacleEnu as _OE
    tall.append(_OE(_Pt(float(xy[0]) + 5.0, float(xy[1])), ctx.sites_enu["ALT1"][1] + 60.0, "X1", "tower", 30.0))
    blocked.obstacles_enu = tall
    errs2, _w2, usable2 = check_sites(blocked)
    assert any(e.startswith("site_check(ALT1, B)") for e in errs2)
    assert "ALT1" not in usable2["B"]
