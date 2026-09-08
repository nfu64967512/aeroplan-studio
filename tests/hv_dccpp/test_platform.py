"""tests/test_platform.py — 模組契約 §2 不變量與規格 §17.2 test_platform / test_sensor / test_energy（平台層部分）。"""
from __future__ import annotations

import dataclasses
import math
from enum import Enum, auto

import pytest

from hv_dccpp.platform import (
    BEAUFORT_UPPER_MPS,
    PLATFORM_HERO,
    PLATFORM_PIONEER,
    PROVISIONAL_SUPPLEMENTARY_DEFAULT,
    TABLE1_KEYS,
    CameraHardware,
    CameraSensor,
    EnergyModel,
    LidarHardware,
    LidarSensor,
    MissionInputError,
    PlatformSpec,
    PlatformSpecError,
    TransitionCmd,
    TrapezoidFovSpec,
    TrapezoidSensor,
    sensor_model,
    trapezoid_footprint,
)

A = PLATFORM_PIONEER
B = PLATFORM_HERO


class Mode(Enum):        # mission.Mode 之替身（mission.py 尚未存在）
    HOVER = auto()
    TRANSITION = auto()
    CRUISE = auto()


@dataclasses.dataclass(frozen=True)
class Req:               # mission.PayloadRequirement 之鴨子型別替身
    req_id: str = "default"
    kind: str = "lidar"
    gsd_required_m: float | None = None
    density_pt_m2: float | None = None
    side_overlap: float | None = None
    forward_overlap: float = 0.70
    shutter_rate_hz: float | None = None
    h_target_agl_m: float | None = None


# ---------------------------------------------------------------------------
# 表 1
# ---------------------------------------------------------------------------

TABLE1 = {
    "platform_type": ("lift_and_cruise_vtol_4plus1", "lift_and_cruise_vtol_4plus1"),
    "wingspan_m": (3.20, 2.18),
    "fuselage_length_m": (1.68, 1.14),
    "wing_area_m2": (1.00, 0.53),
    "mtow_kg": (24.9, 7.0),
    "reference_auw_kg": (24.6, 6.65),
    "reference_payload_kg": (5.0, 0.6),
    "payload_max_kg": (5.0, 1.0),
    "cruise_airspeed_mps": (21.5, 19.0),
    "cruise_airspeed_kmh": (77.4, 68.4),
    "min_mission_airspeed_mps": (18.0, (15.0, 16.0)),
    "transition_airspeed_mps": (16.0, 14.0),
    "stall_airspeed_mps": (14.0, 12.0),
    "max_airspeed_mps": (35.0, None),
    "max_climb_angle_deg": (3.5, 3.5),
    "planning_climb_angle_deg": (3.0, 3.0),
    "max_descent_angle_deg": (5.0, 5.0),
    "planning_descent_angle_deg": (4.0, 4.0),
    "max_fw_climb_rate_mps": (1.13, 1.00),
    "max_fw_descent_rate_mps": (1.50, 1.33),
    "max_bank_angle_deg": (20, 25),
    "turn_radius_theoretical_m": (130, 75),
    "turn_radius_planning_m": (200, 120),
    "vtol_site_clear_radius_m": (8, 6),
    "obstacle_horizontal_buffer_m": (30, 25),
    "transition_start_agl_m": (60, 50),
    "transition_complete_agl_m": ((80, 100), (70, 90)),
    "min_fw_operation_agl_m": (120, 100),
    "wind_resistance_public": (5, 5),
    "reference_endurance_min": (176, 136),
    "reference_total_range_km": (242, 144),
    "reserve_fraction": (0.30, 0.30),
    "usable_range_fraction": ((0.55, 0.60), (0.55, 0.60)),
    "usable_distance_km": ((133, 145), (79, 86)),
    "symmetric_mission_radius_km": ((66, 72), (39, 43)),
    "airspeed_sensor_validated": (False, False),
    "battery_monitor_validated": (False, False),
    "c2_link_range_km": (None, None),
    "terrain_clearance_m": (None, None),
    "alternate_landing_sites_required": (True, True),
}


def test_table1_keys_exact():
    names = {f.name for f in dataclasses.fields(PlatformSpec)}
    assert set(TABLE1) == set(TABLE1_KEYS)
    assert set(TABLE1_KEYS) <= names
    assert len(TABLE1_KEYS) == 40
    assert PROVISIONAL_SUPPLEMENTARY_DEFAULT <= names
    assert len(PROVISIONAL_SUPPLEMENTARY_DEFAULT) == 20
    assert "min_fw_operation_agl_m" in names and "min_fw_operation_agl_m" not in PROVISIONAL_SUPPLEMENTARY_DEFAULT


@pytest.mark.parametrize("key", list(TABLE1))
def test_table1_values(key):
    va, vb = TABLE1[key]
    assert getattr(A, key) == va, key
    assert getattr(B, key) == vb, key


def test_table1_identity_and_provisional_sets():
    assert (A.name, A.cls) == ("Pioneer PNP", "A")
    assert (B.name, B.cls) == ("HERO PNP", "B")
    base = {"vtol_site_clear_radius_m", "obstacle_horizontal_buffer_m", "transition_start_agl_m",
            "transition_complete_agl_m", "c2_link_range_km", "terrain_clearance_m"}
    assert A.provisional == frozenset(base)
    assert B.provisional == frozenset(base | {"max_airspeed_mps", "payload_max_kg"})
    assert A.provisional_supplementary == PROVISIONAL_SUPPLEMENTARY_DEFAULT
    assert B.crosswind_site_max_mps == (5.0, 6.0) and A.crosswind_site_max_mps == (6.0, 8.0)
    assert A.gust_site_max_mps == B.gust_site_max_mps == (8.0, 10.0)
    assert B.transition_ground_run_m is None and A.transition_ground_run_m is None
    assert A.battery_nominal_wh is None and A.reference_battery_wh is None


# ---------------------------------------------------------------------------
# __post_init__ 自洽檢查
# ---------------------------------------------------------------------------

def test_climb_rate_consistency():
    for s in (A, B):
        assert abs(s.cruise_airspeed_mps * math.sin(math.radians(3)) - s.max_fw_climb_rate_mps) <= 0.05 * s.max_fw_climb_rate_mps
        assert abs(s.cruise_airspeed_mps * math.sin(math.radians(4)) - s.max_fw_descent_rate_mps) <= 0.05 * s.max_fw_descent_rate_mps
    with pytest.raises(PlatformSpecError):
        dataclasses.replace(A, max_fw_climb_rate_mps=2.0)
    with pytest.raises(PlatformSpecError):
        dataclasses.replace(A, max_fw_descent_rate_mps=3.0)


def test_bank_radius_consistency():
    for s, expect in ((A, 129.5), (B, 78.9)):
        r = s.cruise_airspeed_mps ** 2 / (9.81 * math.tan(math.radians(s.max_bank_angle_deg)))
        assert r == pytest.approx(expect, abs=0.1)
        assert abs(r - s.turn_radius_theoretical_m) <= 0.10 * s.turn_radius_theoretical_m
        assert s.turn_radius_planning_m >= s.turn_radius_theoretical_m
    with pytest.raises(PlatformSpecError):
        dataclasses.replace(A, turn_radius_theoretical_m=100.0)
    with pytest.raises(PlatformSpecError):
        dataclasses.replace(A, turn_radius_planning_m=100.0)
    with pytest.raises(PlatformSpecError):
        dataclasses.replace(A, cruise_airspeed_kmh=80.0)


def test_range_fraction_consistency():
    for s in (A, B):
        for i in range(2):
            assert abs(s.usable_range_fraction[i] * s.reference_total_range_km - s.usable_distance_km[i]) <= 1.0
            assert abs(s.usable_distance_km[i] / 2 - s.symmetric_mission_radius_km[i]) <= 1.0
    with pytest.raises(PlatformSpecError):
        dataclasses.replace(A, usable_distance_km=(120.0, 145.0))
    with pytest.raises(PlatformSpecError):
        dataclasses.replace(A, symmetric_mission_radius_km=(60.0, 72.0))
    with pytest.raises(PlatformSpecError):
        dataclasses.replace(A, transition_complete_agl_m=(100.0, 80.0))


def test_replace_reruns_checks_and_allows_provisional_overrides():
    s = dataclasses.replace(A, transition_start_agl_m=70.0, c2_link_range_km=25.0, terrain_clearance_m=36.0)
    assert s.transition_start_agl_m == 70.0 and s.c2_link_range_km == 25.0
    assert s.hover_descent_time_s() == pytest.approx(80.0)
    assert A.transition_start_agl_m == 60.0     # 常數不變


# ---------------------------------------------------------------------------
# 派生量
# ---------------------------------------------------------------------------

def test_v_min_mission_and_absolute():
    assert A.v_min_mission() == 18.0 and A.v_min_absolute() == 18.0
    assert B.v_min_mission() == 16.0 and B.v_min_absolute() == 15.0


def test_R_eff_trochoid():
    assert A.r_eff(0.0) == A.turn_radius_planning_m == 200.0
    assert B.r_eff(0.0) == B.turn_radius_planning_m == 120.0
    assert A.r_eff(8.0) == pytest.approx(245.0, abs=1.0)
    assert B.r_eff(8.0) == pytest.approx(152.0, abs=1.0)
    assert A.r_eff(5.0) == 200.0 and B.r_eff(5.0) == 120.0      # W < W* → R_plan（§W.2）
    assert A.r_eff(A.w_star()) == pytest.approx(200.0, abs=1e-9)
    assert A.r_eff(A.w_star() + 0.5) > 200.0
    assert A.r_eff(-3.0) == 200.0


def test_W_star():
    assert A.w_star() == pytest.approx(5.17, abs=0.01)
    assert B.w_star() == pytest.approx(5.03, abs=0.01)


def test_gamma_c_auw():
    assert math.degrees(A.gamma_c_auw(24.6)) == pytest.approx(3.0)
    assert math.degrees(A.gamma_c_auw(24.9)) == pytest.approx(2.93, abs=0.01)
    assert math.degrees(B.gamma_c_auw(7.0)) == pytest.approx(2.68, abs=0.01)
    # clip 下限 0.6 與上限 1
    assert math.tan(A.gamma_c_auw(40.0)) == pytest.approx(0.6 * math.tan(math.radians(3)))
    assert A.gamma_c_auw(10.0) == pytest.approx(math.radians(3))
    assert A.gamma_d() == pytest.approx(math.radians(4)) and B.gamma_d() == pytest.approx(math.radians(4))
    assert A.climb_rate_mps(24.6) == pytest.approx(1.125, abs=1e-3)
    assert B.climb_rate_mps(6.65) == pytest.approx(0.994, abs=1e-3)


def test_stall_margin():
    assert A.stall_speed(24.6) == 14.0 and B.stall_speed(6.65) == 12.0
    assert 1.2 * A.stall_speed(24.9) == pytest.approx(16.9, abs=0.05)
    assert 1.2 * B.stall_speed(7.0) == pytest.approx(14.8, abs=0.05)
    assert A.stall_margin_ok(24.9) and B.stall_margin_ok(7.0)
    assert not A.stall_margin_ok(24.6 * (18.0 / (1.2 * 14.0)) ** 2 * 1.05)   # 超過 (V_min/1.2V_s)² m_ref → 失效


def test_tan_gamma_ground_and_distances():
    # A、W=8 順風 V_g=29.5：tanγ_c^g = 0.0381（2.18°）；弧段折減 cos20°
    assert A.tan_gamma_ground("climb", 29.5, 24.6, arc=False) == pytest.approx(0.0381, abs=2e-4)
    assert A.tan_gamma_ground("climb", 29.5, 24.6, arc=True) == pytest.approx(0.0381 * math.cos(math.radians(20)), abs=2e-4)
    assert A.tan_gamma_ground("descent", 26.5, 24.6, arc=True) == A.tan_gamma_ground("descent", 26.5, 24.6, arc=False)
    assert A.tan_gamma_ground("climb", 0.0, 24.6, arc=False) == math.inf
    with pytest.raises(ValueError):
        A.tan_gamma_ground("level", 20.0, 24.6, False)  # type: ignore[arg-type]
    assert A.climb_dist_m(dh_m=60, v_g_max_mps=26.5, auw_kg=24.6) == pytest.approx(1413.0, abs=5.0)
    # 無風 V_g = Va → tanγ^g = sinγ_c（規格 §3.2 公式），60/sin3° = 1146 m（規格 §11.6 之 60/tan3° = 1145 m 為近似）
    assert A.climb_dist_m(dh_m=60, v_g_max_mps=21.5, auw_kg=24.6) == pytest.approx(60 / math.sin(math.radians(3)), rel=1e-6)
    assert A.climb_dist_m(dh_m=60, v_g_max_mps=21.5, auw_kg=24.6) == pytest.approx(1145.0, abs=2.0)
    assert A.climb_dist_m(dh_m=0, v_g_max_mps=26.5, auw_kg=24.6) == 0.0
    assert A.descent_dist_m(50.0, 26.5) == pytest.approx(883.0, abs=1.0)
    assert A.descent_dist_m(60.0, 26.5) == pytest.approx(1060.0, abs=1.0)
    assert A.descent_dist_m(50.0, 21.5) == pytest.approx(50 / math.sin(math.radians(4)), rel=1e-6)   # 無風 ≈ 規格 715 m（tan 近似）
    assert A.descent_dist_m(50.0, 21.5) == pytest.approx(715.0, abs=2.0)
    assert A.descent_dist_m(-5.0, 26.5) == 0.0


def test_transition_cmd_unvalidated():
    ta, tb = A.transition_cmd(), B.transition_cmd()
    assert isinstance(ta, TransitionCmd)
    assert (ta.v_tr_mps, tb.v_tr_mps) == (18.0, 16.0)
    assert (ta.h_tc_cmd_m, tb.h_tc_cmd_m) == (100.0, 90.0)
    assert (ta.h_tc_energy_m, ta.h_tc_clear_m) == (100.0, 80.0)
    assert (tb.h_tc_energy_m, tb.h_tc_clear_m) == (90.0, 70.0)
    assert ta.d_tr_m == pytest.approx(468.0) and tb.d_tr_m == pytest.approx(384.0)
    assert ta.d_acc_m == pytest.approx(138.0, abs=0.5) and tb.d_acc_m == pytest.approx(105.0, abs=0.5)
    assert ta.t_acc_s == pytest.approx(7.0) and tb.t_acc_s == pytest.approx(6.0)
    assert math.degrees(ta.slope_tr_rad) == pytest.approx(4.9, abs=0.05)
    assert math.degrees(tb.slope_tr_rad) == pytest.approx(5.9, abs=0.1)
    assert ta.slope_tr_rad <= math.radians(A.gamma_tr_max_deg) and tb.slope_tr_rad <= math.radians(B.gamma_tr_max_deg)


def test_transition_cmd_validated_and_profiles():
    va = dataclasses.replace(A, airspeed_sensor_validated=True).transition_cmd()
    vb = dataclasses.replace(B, airspeed_sensor_validated=True).transition_cmd()
    assert (va.v_tr_mps, vb.v_tr_mps) == (16.0, 14.0)
    assert (va.h_tc_cmd_m, vb.h_tc_cmd_m) == (80.0, 70.0)
    assert va.d_tr_m == pytest.approx(384.0) and vb.d_tr_m == pytest.approx(308.0)
    assert va.t_acc_s == pytest.approx(11.0) and va.d_acc_m == pytest.approx((21.5**2 - 16**2) / 1.0)
    fw = A.transition_cmd(transition_profile="fw_slope")
    assert fw.d_tr_m == pytest.approx(763.0, abs=1.0)
    with pytest.raises(ValueError):
        A.transition_cmd(transition_profile="hover")  # type: ignore[arg-type]


def test_hero_overrides():
    h = dataclasses.replace(B, transition_ground_run_m=300.0)
    tc = h.transition_cmd()
    assert tc.d_tr_m == 300.0
    assert math.degrees(tc.slope_tr_rad) == pytest.approx(7.6, abs=0.05)
    assert tc.d_acc_m == pytest.approx(105.0) and tc.t_acc_s == pytest.approx(6.0)
    assert B.crosswind_site_max_mps == (5.0, 6.0)
    assert B.payload_max_kg == 1.0 and "payload_max_kg" in B.provisional and "max_airspeed_mps" in B.provisional
    assert B.max_airspeed_mps is None
    h2 = dataclasses.replace(B, max_airspeed_mps=30.0, payload_max_kg=0.9)
    assert h2.max_airspeed_mps == 30.0 and h2.payload_max_kg == 0.9


def test_platform_overrides_only_provisional():
    # 覆寫允許集合 = provisional ∪ provisional_supplementary ∪ {min_fw_operation_agl_m}（build_context 執行；此處驗集合內容）
    allowed = A.provisional | A.provisional_supplementary | {"min_fw_operation_agl_m"}
    names = {f.name for f in dataclasses.fields(PlatformSpec)}
    assert allowed <= names
    for k in ("cruise_airspeed_mps", "mtow_kg", "turn_radius_planning_m", "reserve_fraction"):
        assert k not in allowed
    up = dataclasses.replace(A, min_fw_operation_agl_m=150.0)
    assert up.min_fw_operation_agl_m == 150.0


def test_hover_descent_time_s():
    assert A.hover_descent_time_s() == pytest.approx(70.0)
    assert B.hover_descent_time_s() == pytest.approx(60.0)
    assert A.hover_descent_time_s(60.0) == pytest.approx(70.0)
    assert A.hover_descent_time_s(10.0) == pytest.approx(20.0)
    assert A.hover_descent_time_s(5.0) == pytest.approx(10.0)


def test_h_ceil_beaufort_empty_mass():
    assert A.h_ceil(120.0, None) == 120.0
    assert A.h_ceil(120.0, 200.0) == 200.0
    assert A.h_ceil(None, None) is None
    assert A.h_ceil(None, 180.0) == 180.0
    assert BEAUFORT_UPPER_MPS == {3: 5.4, 4: 7.9, 5: 10.7, 6: 13.8, 7: 17.1}
    assert A.wind_limit_public_mps() == 10.7 and B.wind_limit_public_mps() == 10.7
    assert dataclasses.replace(A, wind_resistance_public=6).wind_limit_public_mps() == 13.8
    with pytest.raises(ValueError):
        dataclasses.replace(A, wind_resistance_public=9).wind_limit_public_mps()


def test_payload_check():
    assert A.empty_mass_kg() == pytest.approx(19.6)
    assert B.empty_mass_kg() == pytest.approx(6.05)
    # 酬載推估：m_payload = m_i − m_empty；A ≤ 5.0、B < 1.0 為 PRECHECK 判定（此處只驗推估值）
    assert 24.6 - A.empty_mass_kg() == pytest.approx(5.0) and (24.6 - A.empty_mass_kg()) <= A.payload_max_kg
    assert 7.0 - B.empty_mass_kg() == pytest.approx(0.95) and (7.0 - B.empty_mass_kg()) < B.payload_max_kg
    assert 7.2 - B.empty_mass_kg() >= B.payload_max_kg       # 1.15 kg → NO_GO 條件成立


# ---------------------------------------------------------------------------
# 感測器
# ---------------------------------------------------------------------------

def test_lidar_swath_speed_limit():
    s = sensor_model(LidarHardware(), Req(kind="lidar", density_pt_m2=20.0, side_overlap=0.25))
    assert isinstance(s, LidarSensor) and s.kind == "lidar" and isinstance(s.hw, LidarHardware)
    assert s.swath_m(150.0) == pytest.approx(210.06, abs=0.05)
    assert s.strip_spacing_m(150.0) == pytest.approx(157.55, abs=0.05)
    assert s.swath_m(160.0) == pytest.approx(224.07, abs=0.05)
    assert s.strip_spacing_m(160.0) == pytest.approx(168.05, abs=0.05)
    assert s.max_ground_speed_mps(150.0) == pytest.approx(71.4, abs=0.1)
    assert s.agl_window()[1] == pytest.approx(332.0, abs=0.5) and s.agl_window()[0] == 0.0
    assert s.gsd_m(150.0) is None
    assert s.h_target_m(120.0, 20.0, 1.2) == 150.0
    assert s.gsd_attainable(120.0, 20.0)
    assert s.end_extension_m(26.5, 2.0) == pytest.approx(53.0)
    assert s.footprint_len_m(150.0) == 0.0
    assert sensor_model(LidarHardware(), Req(kind="lidar", density_pt_m2=None)).max_ground_speed_mps(150.0) == pytest.approx(71.4, abs=0.1)  # 預設 20


def test_camera_gsd_alt_speed():
    s = sensor_model(CameraHardware(), Req(kind="rgb", gsd_required_m=0.05, side_overlap=0.30, forward_overlap=0.70))
    assert isinstance(s, CameraSensor) and s.kind == "rgb"
    assert s.agl_window()[1] == pytest.approx(205.1, abs=0.1)
    assert s.swath_m(100.0) == pytest.approx(146.9, abs=0.05)
    assert s.strip_spacing_m(100.0) == pytest.approx(102.8, abs=0.05)
    assert s.footprint_len_m(100.0) == pytest.approx(97.5, abs=0.01)
    assert s.gsd_m(100.0) == pytest.approx(0.0244, abs=1e-4)
    assert s.max_ground_speed_mps(100.0) == pytest.approx(29.25, abs=0.01)
    assert s.h_target_m(100.0, 20.0, 1.2) == pytest.approx(171.0, abs=0.5)
    assert s.end_extension_m(24.0, 2.0, 100.0) == pytest.approx(48.0)
    assert s.end_extension_m(10.0, 2.0, 100.0) == pytest.approx(29.25)
    assert s.gsd_attainable(100.0, 20.0)
    st = sensor_model(CameraHardware(), Req(kind="rgb", gsd_required_m=0.05, h_target_agl_m=100.0))
    assert st.h_target_m(100.0, 20.0, 1.2) == 100.0
    assert st.end_extension_m(24.0, 2.0) == pytest.approx(48.0)          # h 缺省 → req.h_target


def test_gsd_unattainable_flag():
    s = sensor_model(CameraHardware(), Req(kind="rgb", gsd_required_m=0.02))
    assert s.agl_window()[1] == pytest.approx(82.05, abs=0.05)
    assert s.gsd_attainable(100.0, 20.0) is False
    assert s.h_target_m(100.0, 20.0, 1.2) < 100.0
    # 門檻：示例硬體需 GSD_req ≥ 2.93 cm
    assert sensor_model(CameraHardware(), Req(kind="rgb", gsd_required_m=0.0293)).gsd_attainable(100.0, 20.0)
    assert not sensor_model(CameraHardware(), Req(kind="rgb", gsd_required_m=0.0290)).gsd_attainable(100.0, 20.0)


def test_trapezoid_matches_paper():
    w, l, *_ = trapezoid_footprint(1200.0, 72.7, 45.0, 76.98)
    assert w == pytest.approx(2200.1, abs=0.2)
    assert l == pytest.approx(999.8, abs=0.2)
    s = sensor_model(TrapezoidFovSpec(kind="rgb", mount_angle_deg=72.7, v_fov_deg=45.0, h_fov_deg=76.98),
                     Req(kind="rgb", h_target_agl_m=1200.0, side_overlap=0.0))
    assert isinstance(s, TrapezoidSensor)
    assert s.swath_m(1200.0) == pytest.approx(2200.1, abs=0.2)
    assert s.footprint_len_m(1200.0) == pytest.approx(999.8, abs=0.2)
    assert s.strip_spacing_m(1200.0) == pytest.approx(2200.1, abs=0.2)
    assert s.gsd_m(1200.0) is None and s.agl_window() == (0.0, math.inf) and s.gsd_attainable(100.0, 20.0)
    assert s.h_target_m(100.0, 20.0, 1.2) == 1200.0
    assert s.max_ground_speed_mps(1200.0) == math.inf
    with pytest.raises(ValueError):
        trapezoid_footprint(1200.0, 20.0, 45.0, 76.98)      # 前緣角 ≤ 0


def test_conservative_agl_width_monotone():
    lid = sensor_model(LidarHardware(), Req(kind="lidar"))
    cam = sensor_model(CameraHardware(), Req(kind="rgb"))
    trap = sensor_model(TrapezoidFovSpec(kind="lidar", mount_angle_deg=72.7, v_fov_deg=45.0, h_fov_deg=76.98), Req(kind="lidar"))
    hs = [50.0, 100.0, 150.0, 200.0, 300.0]
    for s in (lid, cam, trap):
        ws = [s.swath_m(h) for h in hs]
        assert all(ws[i] < ws[i + 1] for i in range(len(ws) - 1))
        assert all(s.strip_spacing_m(h) <= s.swath_m(h) for h in hs)
    # 條帶寬以 h_min 計為保守：任何 h ≥ h_min 之足跡 ≥ w(h_min)
    assert lid.swath_m(140.0) <= lid.swath_m(150.0)


def test_requirement_overrides_hardware():
    hw = CameraHardware(max_shutter_rate_hz=2.0)
    s = sensor_model(hw, Req(kind="rgb", gsd_required_m=0.05, side_overlap=0.5, forward_overlap=0.8, shutter_rate_hz=1.5))
    assert s.strip_spacing_m(100.0) == pytest.approx(0.5 * s.swath_m(100.0))
    assert s.max_ground_speed_mps(100.0) == pytest.approx(97.5 * 0.2 * 1.5)
    assert s.defaults_applied == frozenset()
    d = sensor_model(hw, Req(kind="rgb"))
    assert d.max_ground_speed_mps(100.0) == pytest.approx(97.5 * 0.3 * 2.0)   # shutter → 硬體上限
    assert d.agl_window()[1] == pytest.approx(205.1, abs=0.1)               # GSD 預設 0.05
    assert {"side_overlap", "shutter_rate_hz", "gsd_required_m"} <= d.defaults_applied
    with pytest.raises(MissionInputError):
        sensor_model(hw, Req(kind="rgb", shutter_rate_hz=2.5))
    with pytest.raises(MissionInputError):
        sensor_model(LidarHardware(), Req(kind="rgb"))
    with pytest.raises(MissionInputError):
        sensor_model(hw, Req(kind="rgb", side_overlap=1.0))
    lid = sensor_model(LidarHardware(), Req(kind="lidar", side_overlap=0.4, density_pt_m2=40.0))
    assert lid.strip_spacing_m(150.0) == pytest.approx(0.6 * lid.swath_m(150.0))
    assert lid.max_ground_speed_mps(150.0) == pytest.approx(71.4 / 2, abs=0.1)


def test_default_sensor_warning():
    assert LidarHardware().is_default_example and CameraHardware().is_default_example
    assert not LidarHardware(is_default_example=False).is_default_example
    assert not TrapezoidFovSpec(kind="rgb", mount_angle_deg=72.7, v_fov_deg=45.0, h_fov_deg=76.98).is_default_example
    s = sensor_model(LidarHardware(), Req(kind="lidar"))
    assert s.hw.is_default_example                # PRECHECK 據此警告 sensor_spec_default_example(cls)
    assert s.req.kind == "lidar"


# ---------------------------------------------------------------------------
# 能量模型
# ---------------------------------------------------------------------------

@pytest.fixture
def ea() -> EnergyModel:
    return EnergyModel(A, 24.6, 1350.0, 1500.0, 1500.0)


@pytest.fixture
def eb() -> EnergyModel:
    return EnergyModel(B, 6.65, 380.0, 420.0, 420.0)


def test_p_c_ref_example_batteries(ea, eb):
    assert ea.p_cruise_ref_w() == pytest.approx(511.4, abs=0.1)
    assert eb.p_cruise_ref_w() == pytest.approx(185.3, abs=0.1)
    assert ea.p_hover_w() == pytest.approx(2045.0, abs=1.0)
    assert eb.p_hover_w() == pytest.approx(741.0, abs=1.0)
    assert ea.p_transition_w() == pytest.approx(2557.0, abs=1.0)
    assert ea.p_accel_w() == pytest.approx(614.0, abs=1.0)
    assert ea.k_unc == pytest.approx(1.2075)
    assert ea.p_climb_w(math.radians(3)) - ea.p_cruise_w() == pytest.approx(453.0, abs=1.0)
    assert eb.p_climb_w(math.radians(3)) - eb.p_cruise_w() == pytest.approx(108.0, abs=1.0)
    assert ea.p_descent_w(math.radians(4)) == pytest.approx(max(0.3 * 511.4, 511.4 - 24.6 * 9.81 * 21.5 * math.sin(math.radians(4)) / 0.6), abs=1.0)
    assert ea.p_descent_w(math.radians(60)) == pytest.approx(0.3 * ea.p_cruise_w())


def test_pack_size_invariant_power():
    e1 = EnergyModel(A, 24.6, 1350.0, 1500.0, 1500.0)
    e2 = EnergyModel(A, 24.6, 1100.0, 1200.0, 1500.0)
    e3 = EnergyModel(A, 24.6, 1700.0, 1800.0, 1500.0)
    assert e1.p_cruise_ref_w() == e2.p_cruise_ref_w() == e3.p_cruise_ref_w()
    assert e2.reserve_wh() == pytest.approx(360.0) and e3.reserve_wh() == pytest.approx(540.0)
    # reference None → = nominal（D39 假設）
    e4 = EnergyModel(A, 24.6, 1100.0, 1200.0, None)
    assert e4.reference_wh == 1200.0 and e4.p_cruise_ref_w() == pytest.approx(1200 / (176 / 60))


def test_auw_scaling_1p5(ea):
    heavy = EnergyModel(A, 24.9, 1350.0, 1500.0, 1500.0)
    assert heavy.p_cruise_w() / ea.p_cruise_w() == pytest.approx((24.9 / 24.6) ** 1.5)
    assert heavy.p_hover_w() / ea.p_hover_w() == pytest.approx((24.9 / 24.6) ** 1.5)
    ea.correction = 1.1
    assert ea.p_cruise_w() == pytest.approx(1.1 * 511.36, abs=0.1)
    ea.correction = 3.0
    assert ea.p_cruise_w() == pytest.approx(1.5 * 511.36, abs=0.1)       # 限幅 1.5
    ea.correction = 0.1
    assert ea.p_cruise_w() == pytest.approx(0.8 * 511.36, abs=0.1)       # 限幅 0.8


def test_landing_10m_rule_and_fixed_costs(ea, eb):
    assert A.hover_descent_time_s() == 70.0
    assert ea.e_takeoff_wh() == pytest.approx(39.2, abs=0.1)
    assert ea.e_landing_wh() == pytest.approx(81.2, abs=0.1)
    assert eb.e_takeoff_wh() == pytest.approx(12.9, abs=0.1)
    assert eb.e_landing_wh() == pytest.approx(26.9, abs=0.1)
    # 契約 §W.8：降落總時間 A 117 s = t_acc + t_tr_in + t_dn + t_hov_res
    assert A.transition_cmd().t_acc_s + A.t_tr_in_s + A.hover_descent_time_s() + A.t_hover_reserve_s == pytest.approx(117.0)
    assert B.transition_cmd().t_acc_s + B.t_tr_in_s + B.hover_descent_time_s() + B.t_hover_reserve_s == pytest.approx(106.0)


def test_reserve_deducted_once_and_usable(ea, eb):
    assert ea.reserve_wh() == pytest.approx(450.0) and eb.reserve_wh() == pytest.approx(126.0)
    assert ea.e_usable_wh() == pytest.approx(779.6, abs=0.1)
    assert eb.e_usable_wh() == pytest.approx(214.3, abs=0.1)
    assert ea.e_usable_wh() == pytest.approx(1350.0 - 450.0 - ea.e_takeoff_wh() - ea.e_landing_wh())
    assert ea.e_usable_wh() > 0 and eb.e_usable_wh() > 0
    # 巡航等效分鐘：779.6 Wh ≈ 75.8 min、B 57.5 min
    assert ea.e_usable_wh() / (ea.p_cruise_w() * ea.k_unc) * 60 == pytest.approx(75.8, abs=0.1)
    assert eb.e_usable_wh() / (eb.p_cruise_w() * eb.k_unc) * 60 == pytest.approx(57.5, abs=0.1)
    low = EnergyModel(A, 24.6, 500.0, 1500.0, 1500.0)
    assert low.e_usable_wh() < 0


def test_segment_and_power_modes(ea):
    assert ea.power_w(Mode.HOVER) == ea.p_hover_w() == ea.power_w("HOVER")
    assert ea.power_w(Mode.TRANSITION) == ea.p_transition_w() == ea.power_w("transition")
    assert ea.power_w(Mode.CRUISE) == ea.p_cruise_w()
    assert ea.power_w(Mode.CRUISE, math.radians(3)) == ea.p_climb_w(math.radians(3))
    assert ea.power_w("CRUISE", -math.radians(4)) == ea.p_descent_w(math.radians(4))
    assert ea.power_w("ACCEL") == ea.p_accel_w()
    with pytest.raises(ValueError):
        ea.power_w("ORBIT")
    assert ea.segment_wh(Mode.CRUISE, 3600.0) == pytest.approx(511.36 * 1.2075, abs=0.1)
    assert ea.segment_wh("HOVER", 30.0) == pytest.approx(2045.45 * 30 * 1.2075 / 3600, abs=0.01)
    # 任務段 1100.4 s 平地 → 188.7 Wh（§W.8）
    assert ea.segment_wh(Mode.CRUISE, 1100.4) == pytest.approx(188.7, abs=0.1)


def test_transfer_energy_climb_power(ea):
    flat = ea.transfer_wh(100.0, 0.0)
    assert flat == pytest.approx(ea.segment_wh(Mode.CRUISE, 100.0))
    up = ea.transfer_wh(100.0, 50.0)
    assert up - flat == pytest.approx(6.7, abs=0.1)         # 規格 §8.4：Δz=50 m 少算 6.7 Wh
    down = ea.transfer_wh(100.0, -50.0)
    assert down < flat
    dn_extra = (ea.p_cruise_w() - ea.p_descent_w(A.gamma_d())) * 50 / (21.5 * math.sin(A.gamma_d())) * ea.k_unc / 3600
    assert flat - down == pytest.approx(dn_extra)
    laps = ea.transfer_wh(100.0, 0.0, n_laps=2, t_lap_s=58.4)
    assert laps - flat == pytest.approx(ea.p_climb_w(A.gamma_c_auw(24.6)) * 2 * 58.4 * ea.k_unc / 3600)


def test_cruise_equiv_and_usable_distance_cap(ea, eb):
    assert ea.cruise_equiv_km(389.6) == pytest.approx(48.8, abs=0.1)     # §W.8
    assert eb.cruise_equiv_km(eb.e_usable_wh()) == pytest.approx(57.5 / 60 * 19.0 * 3.6, abs=0.1)
    assert ea.usable_distance_cap_km() == 133.0 and eb.usable_distance_cap_km() == 79.0
    av = EnergyModel(dataclasses.replace(A, battery_monitor_validated=True), 24.6, 1350.0, 1500.0, 1500.0)
    bv = EnergyModel(dataclasses.replace(B, battery_monitor_validated=True), 6.65, 380.0, 420.0, 420.0)
    assert av.usable_distance_cap_km() == 145.0 and bv.usable_distance_cap_km() == 86.0
    assert av.k_bm == 1.0 and av.k_unc == pytest.approx(1.05)
    both = EnergyModel(dataclasses.replace(A, battery_monitor_validated=True, airspeed_sensor_validated=True), 24.6, 1350.0, 1500.0, 1500.0)
    assert both.k_unc == 1.0
    # 規格 §5.4 模型自洽：驗證後 A (1−0.30−0.061)·176 min·21.5 ≈ 145 km
    d_val = (1.0 - 0.30 - (both.e_takeoff_wh() + both.e_landing_wh()) / 1500.0) * 176 / 60 * 21.5 * 3.6
    assert d_val == pytest.approx(145.0, abs=2.0)
    d_unval = ea.cruise_equiv_km(1500.0 * (1 - 0.30) - ea.e_takeoff_wh() - ea.e_landing_wh())
    assert 115.0 <= d_unval <= 133.0                                       # ≈120 km ≤ 表下緣


def test_eta_T():
    e = EnergyModel(A, 24.6, 1350.0, 1500.0, 1500.0, battery_temp_c=25.0)
    assert e.eta_T == 1.0
    e.battery_temp_c = 0.0
    assert e.eta_T == pytest.approx(0.9)
    e.battery_temp_c = -40.0
    assert e.eta_T == pytest.approx(0.8)                    # floor
    assert e.e_usable_wh() == pytest.approx(0.8 * 1350 - 450 - e.e_takeoff_wh() - e.e_landing_wh())


def test_base_minute_mode():
    e = EnergyModel(A, 24.6, 0.9 * 176.0, None, None)      # 僅 soc=0.9 → E_avail = 0.9·T_ref 基準分鐘
    assert e.base_minute_mode is True
    assert e.p_cruise_ref_w() == pytest.approx(1.0 / 60.0)
    assert e.p_cruise_w() == pytest.approx(1.0 / 60.0)
    assert e.segment_wh("CRUISE", 60.0) == pytest.approx(1.0 * e.k_unc)           # 60 s 巡航 = 1 基準分鐘 × k_unc
    assert e.segment_wh("HOVER", 60.0) == pytest.approx(4.0 * e.k_unc)
    assert e.reserve_wh() == pytest.approx(0.3 * 176.0)
    assert e.e_takeoff_wh() == pytest.approx((4 * 30 + 5 * 20 + 1.2 * 7) / 60 * e.k_unc)
    assert e.e_usable_wh() == pytest.approx(0.9 * 176 - 0.3 * 176 - e.e_takeoff_wh() - e.e_landing_wh())
    assert e.e_usable_wh() > 0
    assert e.cruise_equiv_km(60.0) == pytest.approx(60.0 * 60.0 / e.k_unc * 21.5 / 1000.0)  # 60 基準分鐘 → 3600 s/k_unc 巡航
    assert e.p_climb_w(math.radians(3)) == e.p_cruise_w()                          # 無 W 換算 → 額外功率 0（記錄於 deviations）
    explicit = EnergyModel(A, 24.6, 1350.0, 1500.0, 1500.0)
    assert explicit.base_minute_mode is False
