"""航測建圖（HARVEST 建圖模式）：SwarmCoordinator 這一層的接線與值鎖。

被測的是 mission/swarm_coordinator.py 的 HARVEST 分支：

1. `build_mapping_setup`：面板欄位 → 真相機、前向重疊、MappingModeConfig、CameraTriggerConfig（逐值）。
2. **值鎖**：規劃器的 max_shutter_rate_hz、`.param`／讀回閘門的 CAM1_INTRVAL_MIN、引入段（規劃器外延的
   lead_in_m 與匯出器開相機的 camera_lead_in_m）是**同一個數**，端到端（plan → mission items → export_all）。
3. `_uniform_camera` 用的是範例焦距／像素／1 Hz ⇒ 必須標成 is_default_example=True（已驗證的 bug：
   舊版標 False 把 preflight 的 sensor_spec_default_example 警告壓掉）。
4. 建圖閘門：plan.uncovered 非空或有 `mapping_incomplete(` 就不可上傳（status GO_WITH_WARNINGS 不夠）。
5. 不開建圖模式時預設路徑不變（前向 0.70、沒有 camera_cfg、沒有 mapping_mode）。

真規劃情境：1.2 × 1.0 km 平地、1 Pioneer（min_fw 100）+ 2 HERO、HQ IMX477 + 6 mm、120 m、無豁免。
"""
from __future__ import annotations

import math
import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from mission.swarm_coordinator import (  # noqa: E402
    SwarmCoordinator, _uniform_camera, build_mapping_setup, trigger_interval_min_s,
)

pytest.importorskip("hv_dccpp")

from hv_dccpp.ardupilot import (  # noqa: E402
    CameraTriggerConfig, MAV_CMD_DO_SET_CAM_TRIGG_DIST, export_all, read_param_file, read_qgc_wpl,
)
from hv_dccpp.mission import MappingModeConfig  # noqa: E402
from hv_dccpp.platform import CameraHardware  # noqa: E402

HOME = (23.95, 120.90)
_M_LAT = 111132.0
_M_LON = 111412.0 * math.cos(math.radians(HOME[0]))
CLASS_OF = {1: "A", 2: "B", 3: "B"}
F_MM, PIX_UM, W_PX, H_PX = 6.0, 1.55, 4056, 3040          # RPi HQ IMX477 + 6 mm
ALT = 120.0
SIDE = 0.70
SWATH = ALT * W_PX * PIX_UM * 1e-3 / F_MM                  # 125.736 m
FOOT_L = ALT * H_PX * PIX_UM * 1e-3 / F_MM                 # 94.24 m


def ll(east_m: float, north_m: float) -> tuple[float, float]:
    return (HOME[0] + north_m / _M_LAT, HOME[1] + east_m / _M_LON)


def hq6_fov_deg() -> tuple[float, float]:
    w_mm, h_mm = W_PX * PIX_UM * 1e-3, H_PX * PIX_UM * 1e-3
    return (2.0 * math.degrees(math.atan(w_mm / (2.0 * F_MM))),
            2.0 * math.degrees(math.atan(h_mm / (2.0 * F_MM))))


def mapping_params(**over) -> dict:
    """與 VTOLReconPanel.get_params() 同鍵的 HARVEST＋建圖參數（只放 HV 分支會讀的那些）。"""
    hfov, vfov = hq6_fov_deg()
    p = {
        "hfov_deg": hfov, "vfov_deg": vfov, "hv_c2_range_km": 25.0, "hv_waiver_max_agl_m": None,
        "hv_min_fw_agl_m": {"A": 100.0, "B": 100.0}, "hv_dh_band_min_m": 15.0, "hv_dem_margin_m": 3000.0,
        "target_gsd": 0.05, "hv_wind": (0.0, 0.0), "hv_reduce_waypoints": True,
        "hv_map_enabled": True, "hv_map_camera_preset": "rpi_hq_6mm",
        "hv_map_focal_mm": F_MM, "hv_map_pixel_um": PIX_UM, "hv_map_width_px": W_PX, "hv_map_height_px": H_PX,
        "hv_map_trigger_hz": 1.7, "hv_map_trigger_verified": False, "hv_map_readout_ms": None,
        "hv_map_forward_overlap": 0.80, "hv_map_side_overlap": SIDE,
        "hv_map_backend": "servo", "hv_map_firmware": None,
        "hv_map_relay_pin": None, "hv_map_relay_aux_servo": None,
        "hv_map_feedback_pin": -1, "hv_map_feedback_servo": None,
        "hv_map_max_roll_deg": 15.0, "hv_map_lead_in_m": 50.0, "hv_map_terrain_retrigger_tol": 0.10,
        "hv_map_tlog": False,
    }
    p.update(over)
    return p


def areas(*, lidar_area: bool = False) -> list[dict]:
    a = [{"area_id": 1, "priority": 1.0,
          "polygon": [ll(-600, 0), ll(600, 0), ll(600, 1000), ll(-600, 1000)]}]
    if lidar_area:
        # 單光相機之下沒有人掛 LiDAR ⇒ 這塊整塊未覆蓋（status 仍 GO_WITH_WARNINGS）
        a.append({"area_id": 2, "priority": 1.0, "sensor_req": "lidar",
                  "polygon": [ll(900, 0), ll(1500, 0), ll(1500, 600), ll(900, 600)]})
    return a


def drones() -> list[dict]:
    return [{"drone_id": i + 1, "name": f"UAV-{i + 1}", "position": ll(-60.0 + 60.0 * i, -400.0),
             "speed": 20.0, "heading": 0.0, "turn_radius": 150.0, "vehicle_type": "vtol",
             "fov_width": SWATH * (1.0 - SIDE)}
            for i in range(3)]


def plan(params: dict, *, lidar_area: bool = False) -> SwarmCoordinator:
    coord = SwarmCoordinator()
    coord.use_harvest = True
    coord.hv_class_of = lambda d: CLASS_OF[int(d["drone_id"])]
    coord.hv_home = ll(0.0, -400.0)
    coord.hv_wind = (0.0, 0.0)
    coord.hv_fence_zones = []
    coord.harvest_params = dict(params)
    coord.plan_coverage_dccpp(areas=areas(lidar_area=lidar_area), drones=drones(),
                              coverage_width_m=SWATH * (1.0 - SIDE), overlap_rate=0.0,
                              coverage_altitude=ALT, mounting_angle_deg=0.0)
    assert coord.hv_coordinator is not None, "沒走到 HV 分支"
    return coord


@pytest.fixture(scope="module")
def mapped():
    return plan(mapping_params())


@pytest.fixture(scope="module")
def unmapped():
    p = mapping_params(hv_map_enabled=False)
    return plan(p)


def _haversine_m(a, b):
    la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
    h = math.sin((la2 - la1) / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2
    return 2.0 * 6371008.8 * math.asin(math.sqrt(h))


def _strip_on_indices(items):
    """每條帶的 206 開：p1 > 0 且前一個 206 是關（或沒有）。建圖模式 p3 = 0（第一張也走距離觸發），
    不能再用 p3 == 1 認；地形重觸發的 206 前一個 206 仍是開，所以不會被算進來。"""
    out, prev_on = [], False
    for i, it in enumerate(items):
        if it["command"] == MAV_CMD_DO_SET_CAM_TRIGG_DIST:
            on = it["param1"] > 0
            if on and not prev_on:
                out.append(i)
            prev_on = on
    return out


# ═══════════════════════════ 1. 純值：觸發間隔與設定 ═══════════════════════════
@pytest.mark.parametrize("rate, interval", [
    (2.0, 0.50), (1.0, 1.00), (2.5, 0.40), (1.7, 0.60), (1.9, 0.54), (3.0, 0.34), (0.5, 2.00),
])
def test_interval_is_ceiled_to_twenty_ms(rate, interval):
    """CAM1_INTRVAL_MIN = 1/實測率向上取整到 0.02 s；向上 ⇒ 飛控放行的觸發率 ≤ 實測值。"""
    got = trigger_interval_min_s(rate)
    assert got == pytest.approx(interval, abs=1e-12)
    assert 1.0 / got <= rate + 1e-12
    assert round(got / 0.02) * 0.02 == pytest.approx(got, abs=1e-12)


@pytest.mark.parametrize("bad", [0.0, -1.0, float("nan"), float("inf"), None, "2"])
def test_interval_rejects_nonsense(bad):
    with pytest.raises(ValueError):
        trigger_interval_min_s(bad)


def test_setup_builds_the_real_camera_and_locks_the_numbers():
    s = build_mapping_setup(mapping_params())
    cam = s["camera"]
    assert isinstance(cam, CameraHardware)
    assert cam.focal_length_mm == pytest.approx(F_MM)
    assert cam.pixel_pitch_um == pytest.approx(PIX_UM)
    assert cam.sensor_width_mm == pytest.approx(W_PX * PIX_UM * 1e-3)
    assert cam.sensor_height_mm == pytest.approx(H_PX * PIX_UM * 1e-3)
    assert cam.is_default_example is False
    cc = s["camera_cfg"]
    assert isinstance(cc, CameraTriggerConfig)
    # 值鎖：1.7 Hz → 0.60 s → 規劃器用 1/0.60
    assert cc.interval_min_s == pytest.approx(0.60, abs=1e-12)
    assert cam.max_shutter_rate_hz == pytest.approx(1.0 / cc.interval_min_s, rel=1e-12)
    assert s["trigger_rate_planned_hz"] == cam.max_shutter_rate_hz
    assert s["trigger_rate_hz"] == pytest.approx(1.7)
    # 引入段：同一個數
    mm = s["mapping_mode"]
    assert isinstance(mm, MappingModeConfig)
    assert mm.lead_in_m == cc.camera_lead_in_m == pytest.approx(50.0)
    # 其餘欄位逐值
    assert (cc.backend, cc.duration_s, cc.auto_only, cc.max_roll_deg, cc.terrain_retrigger_tol) == \
        ("servo", 0.1, True, 15.0, 0.10)
    assert s["forward_overlap"] == pytest.approx(0.80) and s["side_overlap"] == pytest.approx(SIDE)
    # 未經台架驗證 ⇒ 有警告（不擋）；1.7 → 1.667 的量化也要說出來
    assert any("未經台架驗證" in w for w in s["warnings"])
    assert any("1.667" in w for w in s["warnings"])


def test_setup_exact_rate_has_no_quantisation_warning():
    s = build_mapping_setup(mapping_params(hv_map_trigger_hz=2.0, hv_map_trigger_verified=True))
    assert s["camera_cfg"].interval_min_s == pytest.approx(0.5)
    assert s["camera"].max_shutter_rate_hz == pytest.approx(2.0)
    assert not any("未經台架驗證" in w for w in s["warnings"])
    assert not any("量化" in w for w in s["warnings"])


def test_setup_side_overlap_below_060_warns():
    s = build_mapping_setup(mapping_params(hv_map_side_overlap=0.30))
    assert any("0.30" in w and "0.60" in w for w in s["warnings"])
    s2 = build_mapping_setup(mapping_params(hv_map_side_overlap=0.60))
    assert not any("側向重疊" in w for w in s2["warnings"])


@pytest.mark.parametrize("over, needle", [
    ({"hv_map_backend": "relay", "hv_map_relay_pin": 50}, "firmware"),
    ({"hv_map_backend": "mavlink_v2"}, "firmware"),
    ({"hv_map_backend": "relay", "hv_map_firmware": "4.5.7"}, "relay_pin"),
    ({"hv_map_firmware": "abc"}, "看不懂的韌體版本"),
    ({"hv_map_focal_mm": 0.0}, "焦距"),
    ({"hv_map_trigger_hz": 0.0}, "觸發率"),
    ({"hv_map_forward_overlap": 1.0}, "前向重疊"),
    ({"hv_map_lead_in_m": -1.0}, "引入段"),
    ({"hv_map_backend": "gopro"}, "相機後端"),
])
def test_setup_refuses_with_the_camera_config_message(over, needle):
    """relay／mavlink_v2 沒給韌體版本 ⇒ CameraTriggerConfig 丟例外，訊息要原樣帶出（不猜、不吞）。"""
    with pytest.raises(ValueError) as ei:
        build_mapping_setup(mapping_params(**over))
    assert needle in str(ei.value)


def test_setup_relay_fields_reach_the_config():
    s = build_mapping_setup(mapping_params(hv_map_backend="relay", hv_map_firmware="4.5.7",
                                           hv_map_relay_pin=50, hv_map_relay_aux_servo=9,
                                           hv_map_feedback_pin=51, hv_map_feedback_servo=10))
    cc = s["camera_cfg"]
    assert (cc.backend, cc.firmware, cc.relay_pin, cc.aux_servo_channel_for_relay) == ("relay", "4.5.7", 50, 9)
    assert (cc.feedback_pin, cc.feedback_servo_channel) == (51, 10)


def test_setup_zero_retrigger_tol_means_off():
    s = build_mapping_setup(mapping_params(hv_map_terrain_retrigger_tol=None))
    assert s["camera_cfg"].terrain_retrigger_tol is None


# ═══════════════════════════ 2. _uniform_camera 的範例旗標 ═══════════════════════════
def test_uniform_camera_still_using_example_values_is_flagged():
    """焦距 16 mm／像素 3.9 µm／1 Hz 都是範例值 ⇒ is_default_example=True（preflight 才會警告）。"""
    assert _uniform_camera(hfov_deg=60.0, vfov_deg=45.0).is_default_example is True
    assert _uniform_camera(hfov_deg=0.0, vfov_deg=0.0).is_default_example is True


# ═══════════════════════════ 3. 真規劃：值鎖端到端 ═══════════════════════════
def test_mapping_plan_uses_the_panel_camera(mapped):
    hv = mapped.hv_coordinator
    sensors = [s for (_c, _r), s in hv.last_ctx.sensors.items()]
    assert sensors, "ctx 沒有感測器"
    for s in sensors:
        assert s.hw.focal_length_mm == pytest.approx(F_MM)
        assert s.hw.pixel_pitch_um == pytest.approx(PIX_UM)
        assert s.hw.is_default_example is False
        assert s.r.forward_overlap == pytest.approx(0.80)
        assert s.r.side_overlap == pytest.approx(SIDE)
        # 值鎖①：規劃器用的拍照率 = 1 / CAM1_INTRVAL_MIN
        assert s.r.shutter_rate_hz == pytest.approx(1.0 / hv.camera_cfg.interval_min_s, rel=1e-12)
    # 預設範例相機的警告不該出現（用的是真相機）
    assert not any("sensor_spec_default_example" in str(w) for w in hv.last_plan.feasibility.warnings)


def test_mapping_plan_lead_in_is_one_number_end_to_end(mapped, tmp_path):
    hv = mapped.hv_coordinator
    lead = 50.0
    assert hv.mapping_mode.lead_in_m == pytest.approx(lead)
    assert hv.camera_cfg.camera_lead_in_m == pytest.approx(lead)
    assert hv.mapping_effective["config"]["lead_in_m"] == pytest.approx(lead)
    # 任務項目：每條帶的 206 開掛在「條帶起點後 lead_in 處」那個共線 NAV 之後
    n_checked = 0
    for did in hv.id_map:
        items = hv.mission_items(did, camera_cfg=hv.camera_cfg)
        ons = set(_strip_on_indices(items))
        navs = []                           # (index, (lat, lon))
        for i, it in enumerate(items):
            if it["command"] == 16 and it["seq"] > 0:
                navs.append((i, (it["lat"], it["lon"])))
            if i in ons:
                # 建圖模式：第一張也走距離觸發（受 CAM_MAX_ROLL 擋），不是 p3=1 立即拍
                assert it["param3"] == 0.0
                prev = [p for j, p in navs if j < i]
                assert len(prev) >= 2
                assert _haversine_m(prev[-2], prev[-1]) == pytest.approx(lead, abs=0.5)
                n_checked += 1
    assert n_checked >= 3, "沒有檢查到任何條帶"


def test_mapping_plan_interval_reaches_export_and_readback_gate(mapped, tmp_path):
    """值鎖②：CAM1_INTRVAL_MIN 在 .param（匯出）、讀回閘門的預期值、規劃器三處是同一個數。"""
    hv = mapped.hv_coordinator
    cc = hv.camera_cfg
    summary = export_all(hv.last_plan, hv.last_ctx, hv.cfg, tmp_path, camera_cfg=cc)
    assert summary["camera_cfg"]["interval_min_s"] == pytest.approx(cc.interval_min_s)
    rate = {s.r.shutter_rate_hz for s in hv.last_ctx.sensors.values()}
    for did, uid in hv.id_map.items():
        if uid not in summary["uavs"]:
            continue
        par = read_param_file(summary["uavs"][uid]["files"]["param"])
        exp = hv.expected_camera_params(did, camera_cfg=cc)
        assert par["CAM1_INTRVAL_MIN"] == pytest.approx(cc.interval_min_s, abs=1e-6)
        assert exp["CAM1_INTRVAL_MIN"] == pytest.approx(cc.interval_min_s, abs=1e-12)
        for r in rate:
            assert 1.0 / par["CAM1_INTRVAL_MIN"] == pytest.approx(r, rel=1e-5)
        # 讀回閘門的每一個預期值都等於匯出的 .param
        for k, v in exp.items():
            assert par[k] == pytest.approx(v, abs=1e-6), k
        # 匯出的 .waypoints 就是上傳的項目
        items = hv.mission_items(did, camera_cfg=cc)
        wpl = read_qgc_wpl(summary["uavs"][uid]["files"]["waypoints"])
        assert [it.command for it in wpl] == [it["command"] for it in items]
        for a, b in zip(wpl, items):
            assert a.param1 == pytest.approx(b["param1"], abs=1e-4)
            assert a.param2 == pytest.approx(b["param2"], abs=1e-4)
            assert a.x == pytest.approx(b["lat"], abs=1e-7) and a.y == pytest.approx(b["lon"], abs=1e-7)


def test_mapping_trigger_distance_uses_the_forward_overlap(mapped):
    """206 開的 p1 = 足跡縱長 ×(1 − 0.80)，用面板相機獨立重算（不是 0.70、不是範例相機）。"""
    hv = mapped.hv_coordinator
    want = FOOT_L * (1.0 - 0.80)                                 # 18.848 m @ 120 m
    ds = []
    for did in hv.id_map:
        items = hv.mission_items(did, camera_cfg=hv.camera_cfg)
        ds.extend(items[i]["param1"] for i in _strip_on_indices(items))
    assert ds
    for d in ds:
        assert d == pytest.approx(want, rel=2e-3)


def test_mapping_plan_is_uploadable_when_complete(mapped):
    gate = mapped.hv_coordinator.mapping_gate()
    assert gate == {"uploadable": True, "reasons": []}
    assert mapped.hv_mapping_gate == gate
    assert mapped.hv_mapping_setup["camera_cfg"] is mapped.hv_coordinator.camera_cfg


def test_mapping_effective_values_are_reported():
    seen: list = []
    c = SwarmCoordinator()
    c.use_harvest = True
    c.hv_class_of = lambda d: CLASS_OF[int(d["drone_id"])]
    c.hv_home = ll(0.0, -400.0)
    c.hv_wind = (0.0, 0.0)
    c.hv_fence_zones = []
    c.hv_on_report = seen.append
    c.harvest_params = mapping_params()
    c.plan_coverage_dccpp(areas=areas(), drones=drones(), coverage_width_m=SWATH * (1.0 - SIDE),
                          overlap_rate=0.0, coverage_altitude=ALT)
    notes = [n for p in seen for n in (p.get("validation_notes") or [])]
    setup = [n for n in notes if n.startswith("mapping_setup(")]
    # 規劃前送一次（NO_GO 時尾端跑不到）、尾端再併進 validation_notes 一次：同一句話
    assert setup and len(set(setup)) == 1
    t = setup[0]
    for frag in ("f=6", "1.55", "4056×3040", "CAM1_INTRVAL_MIN 0.60", "前向 0.80", "側向 0.70", "引入段 50",
                 "servo", "未經台架驗證"):
        assert frag in t, (frag, t)
    # 刈幅核對不再誤報（建圖模式送的是相機刈幅，不是扣了重疊的間距）
    adapter_notes = [n for p in seen for n in (p.get("notes") or [])]
    assert not any(str(n).startswith("fov_width_mismatch") for n in adapter_notes)


# ═══════════════════════════ 4. 建圖閘門（真規劃） ═══════════════════════════
def test_gate_blocks_a_plan_with_an_uncovered_area():
    """單光相機下 LiDAR 區整塊未覆蓋：preflight 仍是 GO_WITH_WARNINGS，但建圖不可上傳。"""
    seen: list = []
    c = SwarmCoordinator()
    c.use_harvest = True
    c.hv_class_of = lambda d: CLASS_OF[int(d["drone_id"])]
    c.hv_home = ll(0.0, -400.0)
    c.hv_wind = (0.0, 0.0)
    c.hv_fence_zones = []
    c.hv_on_report = seen.append
    c.harvest_params = mapping_params()
    c.plan_coverage_dccpp(areas=areas(lidar_area=True), drones=drones(),
                          coverage_width_m=SWATH * (1.0 - SIDE), overlap_rate=0.0, coverage_altitude=ALT)
    hv = c.hv_coordinator
    assert hv.last_plan.feasibility.status != "NO_GO"
    assert hv.last_plan.uncovered, "前提：要真的有未覆蓋區"
    gate = hv.mapping_gate()
    assert gate["uploadable"] is False
    assert any("未覆蓋" in r for r in gate["reasons"])
    assert any("mapping_incomplete(" in r for r in gate["reasons"])
    notes = [n for p in seen for n in (p.get("validation_notes") or [])]
    assert any(n.startswith("mapping_upload_blocked(") for n in notes)


# ═══════════════════════════ 5. 不開建圖：預設路徑不變 ═══════════════════════════
def test_default_path_keeps_forward_070_and_no_camera_cfg(unmapped):
    hv = unmapped.hv_coordinator
    assert hv.mapping_mode is None and hv.camera_cfg is None
    assert unmapped.hv_mapping_setup is None and unmapped.hv_mapping_gate is None
    for s in hv.last_ctx.sensors.values():
        assert s.r.forward_overlap == pytest.approx(0.70)
        assert s.hw.focal_length_mm == pytest.approx(16.0)          # 範例相機（面板 HFOV 反推）
        assert s.hw.is_default_example is True
    assert any("sensor_spec_default_example" in str(w) for w in hv.last_plan.feasibility.warnings)
