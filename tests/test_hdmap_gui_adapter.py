"""航測建圖：轉接層（hv_dccpp/integration/aeroplan_adapter.py）給 GUI 上傳／匯出用的介面。

1. `HvDccppCoordinator.mission_items(drone_id, camera_cfg=...)`：SITLLink.upload_mission_items 吃的 dict 形狀
   （seq 0 = Home 槽），內容 = hv 匯出器 plan_to_mission_items(camera_trigger=True, camera_cfg)；
   camera_cfg 的引入段 ≠ 規劃時的 lead_in_m ⇒ 拒絕。
2. `to_upload_tuples` 的 NAV_WAYPOINT param2：ArduPlane 拿它當**接受半徑**。舊版填 r_eff（Pioneer 200 m、
   HERO 120 m）⇒ 條帶尾端提早最多 200 m 被判到達。修正後 = hv 匯出器同一條 min(R_eff/2, 30)。
3. `_default_payload_reqs` / `to_mission_inputs` / `HvDccppCoordinator` 的 forward_overlap（預設 0.70 不變）。
4. `mapping_gate()`、`expected_camera_params()`、`expected_photo_count()`。
"""
from __future__ import annotations

import dataclasses
import math
import os
import types

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

pytest.importorskip("hv_dccpp")

from hv_dccpp.ardupilot import (  # noqa: E402
    CameraTriggerConfig, MAV_CMD_DO_LAND_START, MAV_CMD_DO_SET_CAM_TRIGG_DIST, accept_radius_m,
)
from hv_dccpp.integration.aeroplan_adapter import (  # noqa: E402
    AeroPlanAdapterError, HvDccppCoordinator, _default_payload_reqs, expected_photo_count,
    mission_item_dicts, to_mission_inputs, to_upload_tuples,
)
from hv_dccpp.mission import MappingModeConfig, PayloadRequirement, PlannerConfig  # noqa: E402
from hv_dccpp.platform import PLATFORM_HERO, PLATFORM_PIONEER, CameraHardware  # noqa: E402

HOME = (23.95, 120.90)
_M_LAT = 111132.0
_M_LON = 111412.0 * math.cos(math.radians(HOME[0]))
HQ6 = CameraHardware(focal_length_mm=6.0, sensor_width_mm=4056 * 1.55e-3, sensor_height_mm=3040 * 1.55e-3,
                     pixel_pitch_um=1.55, max_shutter_rate_hz=2.0, is_default_example=False)
CLASSES = {1: "A", 2: "B", 3: "B"}
LEAD = 50.0
CC = CameraTriggerConfig(interval_min_s=0.5, max_roll_deg=15.0, camera_lead_in_m=LEAD)
ITEM_KEYS = {"seq", "command", "frame", "lat", "lon", "alt", "param1", "param2", "param3", "param4",
             "current", "autocontinue"}


def _ll(e, n):
    return (HOME[0] + n / _M_LAT, HOME[1] + e / _M_LON)


def _areas():
    return [{"area_id": 1, "priority": 1.0, "polygon": [_ll(-600, 0), _ll(600, 0), _ll(600, 1000), _ll(-600, 1000)]}]


def _drones():
    return [{"drone_id": i + 1, "name": f"UAV-{i + 1}", "position": _ll(-60.0 + 60.0 * i, -400.0),
             "speed": 20.0, "heading": 0.0, "turn_radius": 150.0, "vehicle_type": "vtol", "fov_width": 37.7}
            for i in range(3)]


def _coord(mapping, camera_cfg=None, **kw):
    req = PayloadRequirement("default", "rgb", gsd_required_m=0.05, side_overlap=0.70, forward_overlap=0.80,
                             h_target_agl_m=120.0)
    return HvDccppCoordinator(
        class_of=lambda d: CLASSES[int(d["drone_id"])],
        platform_a=dataclasses.replace(PLATFORM_PIONEER, min_fw_operation_agl_m=100.0), platform_b=PLATFORM_HERO,
        sensor_a=(HQ6,), sensor_b=(HQ6,), payload_reqs={"default": req},
        cfg=PlannerConfig(dem_step_m=25.0, dh_band_min_m=15.0), home=_ll(0.0, -400.0), wind=(0.0, 0.0),
        waiver_max_agl_m=None, dem_margin_m=3000.0, mapping_mode=mapping, camera_cfg=camera_cfg, **kw)


def _plan(hv):
    hv.plan_coverage_dccpp(_areas(), _drones(), coverage_width_m=125.7, overlap_rate=0.70, coverage_altitude=120.0)
    return hv


def _flying(hv):
    """有任務的機（hv 計畫只收分到工作的 UAV）。"""
    return [d for d, u in sorted(hv.id_map.items()) if u in hv.last_plan.plans]


@pytest.fixture(scope="module")
def hv_map():
    return _plan(_coord(MappingModeConfig(lead_in_m=LEAD), camera_cfg=CC))


@pytest.fixture(scope="module")
def hv_plain():
    return _plan(_coord(None))


# ═══════════════════════════ 1. mission_items ═══════════════════════════
def test_mission_items_have_the_upload_shape(hv_map):
    assert _flying(hv_map)
    for did in _flying(hv_map):
        items = hv_map.mission_items(did, camera_cfg=CC)
        assert items, did
        for it in items:
            assert set(it) == ITEM_KEYS
            assert isinstance(it["seq"], int) and isinstance(it["command"], int) and isinstance(it["frame"], int)
            assert all(isinstance(it[k], float) for k in ("lat", "lon", "alt", "param1", "param2", "param3",
                                                          "param4"))
        assert [it["seq"] for it in items] == list(range(len(items)))
        home = items[0]
        # seq 0 = Home 槽：**這架的起飛坪**的經緯與 MSL，frame 0（GLOBAL，MSL）。2026-10-14 F1 起不再是規劃 home：
        # ArduPlane 解鎖時把 home 重設到飛機自己的位置，相對高度的基準就是起飛坪（R1 #2）。
        up = hv_map.last_plan.plans[hv_map.id_map[did]]
        pad = hv_map.last_ctx.inputs.sites[up.takeoff_site]
        assert (home["command"], home["frame"]) == (16, 0)
        assert home["lat"] == pytest.approx(pad.lat_deg, abs=1e-12)
        assert home["lon"] == pytest.approx(pad.lon_deg, abs=1e-12)


def test_mission_items_carry_one_camera_on_per_strip(hv_map):
    total = 0
    for did in _flying(hv_map):
        uid = hv_map.id_map[did]
        up = hv_map.last_plan.plans.get(uid)
        items = hv_map.mission_items(did, camera_cfg=CC)
        cams = [it for it in items if it["command"] == MAV_CMD_DO_SET_CAM_TRIGG_DIST]
        on = [it for it in cams if it["param1"] > 0 and it["param3"] == 1.0]
        n_strips = len(up.strips) if up is not None else 0
        assert len(on) == n_strips, (uid, len(on), n_strips)
        total += len(on)
        if n_strips:
            # DO_LAND_START 緊接 206(0)（失效保護跳到這裡時相機一定關）
            i = next(k for k, it in enumerate(items) if it["command"] == MAV_CMD_DO_LAND_START)
            assert items[i + 1]["command"] == MAV_CMD_DO_SET_CAM_TRIGG_DIST and items[i + 1]["param1"] == 0.0
    assert total >= 3


def test_mission_items_equal_the_hv_exporter(hv_map):
    from hv_dccpp.ardupilot import plan_to_mission_items
    did, uid = next((d, u) for d, u in hv_map.id_map.items() if u in hv_map.last_plan.plans and hv_map.last_plan.plans[u].strips)
    ref = plan_to_mission_items(hv_map.last_plan.plans[uid], hv_map.last_ctx, hv_map.cfg, alt_frame="relative",
                                camera_trigger=True, camera_cfg=CC)
    assert hv_map.mission_items(did, camera_cfg=CC) == mission_item_dicts(ref)


def test_mission_items_refuse_a_different_lead_in(hv_map, hv_plain):
    with pytest.raises(AeroPlanAdapterError, match="引入段"):
        hv_map.mission_items(_flying(hv_map)[0], camera_cfg=dataclasses.replace(CC, camera_lead_in_m=0.0))
    with pytest.raises(AeroPlanAdapterError, match="引入段"):
        hv_plain.mission_items(_flying(hv_plain)[0], camera_cfg=CC)   # 沒開建圖（lead 0）卻要開相機引入段 50
    # 沒開建圖、引入段 0 ⇒ 允許（條帶沒外延也不需要）
    assert hv_plain.mission_items(_flying(hv_plain)[0], camera_cfg=dataclasses.replace(CC, camera_lead_in_m=0.0))


def test_mission_items_input_errors(hv_map):
    with pytest.raises(AeroPlanAdapterError, match="沒有任務"):
        hv_map.mission_items(99, camera_cfg=CC)
    with pytest.raises(AeroPlanAdapterError):
        hv_map.mission_items(_flying(hv_map)[0], camera_cfg={"backend": "servo"})
    with pytest.raises(AeroPlanAdapterError, match="尚未規劃"):
        _coord(None).mission_items(1, camera_cfg=CC)


def test_constructor_refuses_mismatched_lead_in():
    with pytest.raises(AeroPlanAdapterError, match="引入段"):
        _coord(MappingModeConfig(lead_in_m=LEAD), camera_cfg=dataclasses.replace(CC, camera_lead_in_m=40.0))
    with pytest.raises(AeroPlanAdapterError):
        _coord(None, camera_cfg="servo")


def test_qgc_waypoints_passes_the_camera_cfg_through(hv_map):
    lines = hv_map.qgc_waypoints(camera_cfg=CC)
    for did in _flying(hv_map):
        items = hv_map.mission_items(did, camera_cfg=CC)
        got = lines[str(did)]
        assert len(got) == len(items) + 1                              # + 檔頭
        cmds = [int(ln.split("\t")[3]) for ln in got[1:]]
        assert cmds == [it["command"] for it in items]
    # 不給 camera_cfg ⇒ 舊行為（沒有引入段 NAV）⇒ 有條帶的那架項目比較少
    plain = hv_map.qgc_waypoints()
    did = next(d for d, u in hv_map.id_map.items() if u in hv_map.last_plan.plans and hv_map.last_plan.plans[u].strips)
    assert len(plain[str(did)]) < len(lines[str(did)])


# ═══════════════════════════ 2. to_upload_tuples 的接受半徑（bug fix） ═══════════════════════════
def test_upload_tuples_nav_param2_is_the_acceptance_radius_for_both_classes(hv_plain):
    """值鎖：每個 NAV_WAYPOINT 的 param2 ≤ 30 m，且等於 hv 匯出器的 min(R_eff/2, 30)。"""
    ctx = hv_plain.last_ctx
    w_ref = ctx.wind.max_speed() if ctx.wind is not None else 0.0
    seen_cls = set()
    for did, tl in hv_plain.upload_tuples().items():
        cls = hv_plain.id_map[did][0]
        seen_cls.add(cls)
        want = accept_radius_m(ctx.specs[cls].r_eff(w_ref))
        p2 = [t[5] for t in tl if t[3] == 16 and t[5] != 0.0]
        assert p2, did
        for v in p2:
            assert v <= 30.0
            assert v == pytest.approx(want)
        # r_eff 本身沒有被改掉（幾何用）：Pioneer 200、HERO 120
        assert ctx.specs[cls].r_eff(w_ref) >= 120.0
    assert seen_cls == {"A", "B"}


def test_to_upload_tuples_derives_param2_from_the_turn_radius():
    wp = types.SimpleNamespace(lat=23.9, lon=120.9, alt=120.0, segment_type="OPERATION", heading_compass_deg=0.0)
    bp = types.SimpleNamespace(waypoints=[wp, wp])
    for r, want in ((200.0, 30.0), (120.0, 30.0), (50.0, 25.0), (20.0, 10.0)):
        tl = to_upload_tuples(bp, turn_radius_m=r)
        assert {t[5] for t in tl if t[3] == 16} == {want}, r
    # 明確給接受半徑 ⇒ 原樣使用（有效值契約：不改使用者的值）
    tl = to_upload_tuples(bp, turn_radius_m=200.0, accept_radius_m=12.0)
    assert any(t[5] == 12.0 for t in tl if t[3] == 16)
    assert not any(t[5] == 200.0 for t in tl)


# ═══════════════════════════ 3. forward_overlap 參數 ═══════════════════════════
def test_default_payload_reqs_forward_overlap_default_and_override():
    hw = {"B": (HQ6,)}
    reqs, rid, _ = _default_payload_reqs(hw, 0.3, 120.0, [])
    assert reqs[rid].forward_overlap == pytest.approx(0.70)
    reqs2, rid2, _ = _default_payload_reqs(hw, 0.3, 120.0, [], forward_overlap=0.85)
    assert reqs2[rid2].forward_overlap == pytest.approx(0.85)


def test_to_mission_inputs_threads_forward_overlap():
    kw = dict(coverage_width_m=125.7, overlap_rate=0.7, coverage_altitude=120.0, home=_ll(0, -400),
              class_of=lambda d: CLASSES[int(d["drone_id"])], sensor_a=(HQ6,), sensor_b=(HQ6,),
              waiver_max_agl_m=None, dem_step_m=200.0)
    inp, _ = to_mission_inputs(_areas(), _drones(), **kw)
    assert all(r.forward_overlap == pytest.approx(0.70) for r in inp.payload_reqs.values())
    inp2, _ = to_mission_inputs(_areas(), _drones(), forward_overlap=0.80, **kw)
    assert all(r.forward_overlap == pytest.approx(0.80) for r in inp2.payload_reqs.values())
    with pytest.raises(AeroPlanAdapterError):
        to_mission_inputs(_areas(), _drones(), forward_overlap=1.0, **kw)


# ═══════════════════════════ 4. mapping_gate ═══════════════════════════
def _fake_plan(uncovered=(), warnings=(), status="GO_WITH_WARNINGS"):
    return types.SimpleNamespace(uncovered=list(uncovered),
                                 feasibility=types.SimpleNamespace(warnings=list(warnings), status=status))


def test_gate_uncovered_blocks_even_with_go_with_warnings():
    hv = _coord(MappingModeConfig(lead_in_m=LEAD), camera_cfg=CC)
    hv.last_plan = _fake_plan(uncovered=[("Z2_1", "no_feasible_insertion")])
    g = hv.mapping_gate()
    assert g["uploadable"] is False and any("Z2_1" in r for r in g["reasons"])


def test_gate_mapping_incomplete_warning_alone_blocks():
    hv = _coord(MappingModeConfig(lead_in_m=LEAD), camera_cfg=CC)
    hv.last_plan = _fake_plan(warnings=["mapping_incomplete(未覆蓋 1 項：Z1:x；拼出來的圖會有破洞)"])
    g = hv.mapping_gate()
    assert g["uploadable"] is False and any(r.startswith("mapping_incomplete(") for r in g["reasons"])
    hv.last_plan = _fake_plan()
    hv.notes = ["preflight:mapping_incomplete(未覆蓋 2 項)"]
    assert hv.mapping_gate()["uploadable"] is False


def test_gate_clean_plan_and_no_plan():
    hv = _coord(MappingModeConfig(lead_in_m=LEAD), camera_cfg=CC)
    assert hv.mapping_gate()["uploadable"] is False                   # 還沒規劃
    hv.last_plan = _fake_plan(warnings=["mapping_small_blocks(< 3 條：c=2)"])
    assert hv.mapping_gate() == {"uploadable": True, "reasons": []}
    hv.last_plan = _fake_plan(status="NO_GO")
    assert hv.mapping_gate()["uploadable"] is False


def test_gate_on_real_complete_plan(hv_map):
    assert hv_map.mapping_gate() == {"uploadable": True, "reasons": []}


# ═══════════════════════════ 5. expected_camera_params ═══════════════════════════
def test_expected_camera_params_servo(hv_map):
    exp = hv_map.expected_camera_params(_flying(hv_map)[0], camera_cfg=CC)
    assert exp == {"CAM1_TYPE": 1.0, "CAM1_DURATION": 0.1, "CAM1_INTRVAL_MIN": 0.5, "CAM_AUTO_ONLY": 1.0,
                   "CAM_MAX_ROLL": 15.0, "CAM1_FEEDBAK_PIN": -1.0, "CAM1_FEEDBAK_POL": 1.0,
                   "CAM1_TRIGG_DIST": 0.0, "SERVO10_FUNCTION": 10.0}


def test_expected_camera_params_relay_45(hv_map):
    cc = dataclasses.replace(CC, backend="relay", firmware="4.5.7", relay_pin=50, aux_servo_channel_for_relay=9)
    exp = hv_map.expected_camera_params(_flying(hv_map)[0], camera_cfg=cc)
    assert exp["CAM1_TYPE"] == 2.0 and exp["RELAY1_FUNCTION"] == 4.0 and exp["RELAY1_PIN"] == 50.0
    assert exp["SERVO9_FUNCTION"] == -1.0 and exp["CAM1_RELAY_ON"] == 1.0
    assert not any(k.startswith(("AIRSPEED", "Q_", "FENCE", "BATT", "WP_")) for k in exp)


# ═══════════════════════════ 6. expected_photo_count ═══════════════════════════
def _m_per_deg_lat(lat_deg):
    """WGS84 子午線每度長（公尺）——合成條帶要用真的公尺，否則 floor(L/d) 會差一張。"""
    a, e2 = 6378137.0, 6.69437999014e-3
    s = math.sin(math.radians(lat_deg))
    return math.radians(1.0) * a * (1.0 - e2) / (1.0 - e2 * s * s) ** 1.5


def _nav(seq, n_m):
    lat, lon = HOME[0] + n_m / _m_per_deg_lat(HOME[0]), HOME[1]
    return {"seq": seq, "command": 16, "frame": 3, "lat": lat, "lon": lon, "alt": 120.0,
            "param1": 0.0, "param2": 30.0, "param3": 0.0, "param4": 0.0, "current": 0, "autocontinue": 1}


def _cam(seq, d, p3=1.0):
    return {"seq": seq, "command": 206, "frame": 3, "lat": 0.0, "lon": 0.0, "alt": 0.0,
            "param1": float(d), "param2": 0.0, "param3": float(p3), "param4": 0.0, "current": 0, "autocontinue": 1}


def test_expected_photo_count_is_floor_plus_one_per_strip():
    """沒有空速資訊（無 DO_CHANGE_SPEED）⇒ 連續模型 floor(L/d)+1；關 = 匯出器的 206(0, 0, 0, 0)（p3 = 0）。
    （2026-10-14 F1：p3 = 1 的關在 ArduPlane 也會拍一張，所以這裡的關明確給 p3 = 0；tick 量化與接受半徑另見
    tests/test_hdmap_fix_exporter.py。）"""
    home = dict(_nav(0, -500.0), frame=0)
    items = [home, _nav(1, 0.0), _cam(2, 20.0), _nav(3, 1010.0), _cam(4, 0.0, p3=0.0),          # 1010 / 20 ⇒ 50 + 1
             _nav(5, 1300.0), _cam(6, 30.0), _nav(7, 1395.0), _cam(8, 0.0, p3=0.0)]            # 95 / 30 ⇒ 3 + 1
    assert expected_photo_count(items) == 51 + 4
    # 首張不立即拍（p3=0）⇒ floor(L/d)
    items2 = [home, _nav(1, 0.0), _cam(2, 20.0, p3=0.0), _nav(3, 1010.0), _cam(4, 0.0, p3=0.0)]
    assert expected_photo_count(items2) == 50
    # 地形重發：中途改 d，剩餘距離以新 d 接續（110/20 ⇒ 5 張、餘 10 m；再 105 m ⇒ 115/10 ⇒ 11 張）
    items3 = [home, _nav(1, 0.0), _cam(2, 20.0), _nav(3, 110.0), _cam(4, 10.0, p3=0.0), _nav(5, 215.0),
              _cam(6, 0.0, p3=0.0)]
    assert expected_photo_count(items3) == 1 + 5 + 11
    # 多條 NAV 的折線：L = 各段總和（ON 掛在第一個 NAV 之後）
    items4 = [home, _nav(1, 0.0), _cam(2, 25.0), _nav(3, 30.0), _nav(4, 60.0), _nav(5, 90.0), _cam(6, 0.0, p3=0.0)]
    assert expected_photo_count(items4) == 1 + 3
    assert expected_photo_count([home, _nav(1, 0.0), _nav(2, 500.0)]) == 0


def test_expected_photo_count_on_a_real_mission(hv_map):
    """真任務：每條帶 floor(L/d)+1，加總 > 條帶數（只驗數量級，細節由上面的合成情境鎖）。"""
    for did in _flying(hv_map):
        up = hv_map.last_plan.plans[hv_map.id_map[did]]
        n = expected_photo_count(hv_map.mission_items(did, camera_cfg=CC))
        if up.strips:
            assert n > 10 * len(up.strips)
        else:
            assert n == 0
