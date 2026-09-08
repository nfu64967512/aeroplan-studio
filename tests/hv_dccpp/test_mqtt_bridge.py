"""tests/test_mqtt_bridge.py — hv_dccpp.integration.mqtt_bridge 純函式與假 client 測試（不需 broker）。

情境：home (23.7027, 120.4193)（使用者地圖預設中心附近）、平地合成 DEM、五角形區域（ENU 約 1.8 km × 1.3 km，
距 home 2 km）、1 Pioneer + 1 HERO。第二個情境為舊系統 4 角點 payload。
"""
from __future__ import annotations

import collections
import json
import math
import os
import tempfile

import numpy as np
import pytest

from hv_dccpp.geo import GeoFrame
from hv_dccpp.integration import mqtt_bridge as mb
from hv_dccpp.mission import MissionInputError, MissionInputs, PlannerConfig

HOME = (23.7027, 120.4193)
HOME_MSL = 100.0
PENTAGON_ENU = [(2000.0, -500.0), (3400.0, -600.0), (3800.0, 300.0), (3000.0, 700.0), (2100.0, 500.0)]


def _latlon_ring(ring_enu, home=HOME, h0=HOME_MSL):
    fr = GeoFrame(home[0], home[1], h0)
    w = fr.to_wgs84(np.array([[x, y, 0.0] for x, y in ring_enu], dtype=np.float64))
    return [{"lat": float(a), "lon": float(b)} for a, b in w[:, :2]]


def _bbox(points):
    lats = [p["lat"] for p in points]
    lons = [p["lon"] for p in points]
    return min(lats), max(lats), min(lons), max(lons)


@pytest.fixture(scope="module")
def request5() -> dict:
    alt = _latlon_ring([(500.0, 1200.0)])[0]
    return {
        "mission_id": "t-pent",
        "home": {"lat": HOME[0], "lon": HOME[1], "alt_msl": HOME_MSL, "id": "H"},
        "alternates": [dict(alt, id="ALT1")],
        "areas": [{"area_id": "Z1", "polygon": _latlon_ring(PENTAGON_ENU), "sensor_req": "any", "priority": 1.0}],
        "drones": [
            {"id": "A1", "platform": "pioneer_pnp", "auw_kg": 24.6, "payload_kg": 5.0, "available_wh": 1350.0,
             "nominal_wh": 1500.0},
            {"id": "B1", "platform": "hero_pnp", "auw_kg": 6.65, "payload_kg": 0.6, "available_wh": 380.0,
             "nominal_wh": 420.0},
        ],
        "wind": {"speed_mps": 4.0, "dir_from_deg": 270.0, "gust_mps": 6.0},
        "c2_range_km": 25.0,
        "terrain_clearance_m": 36.0,
        "airspace": {"max_agl_m": 120.0, "waiver_max_agl_m": 200.0},
        "planner": {"dem_step_m": 50.0},
    }


@pytest.fixture(scope="module")
def result5(request5) -> mb.BridgeResult:
    return mb.handle_request_full(request5, raise_errors=True)


# ---------------------------------------------------------------------------
# request_to_mission_inputs
# ---------------------------------------------------------------------------

def test_request_to_mission_inputs_fields(request5):
    notes: list[str] = []
    inputs, cfg = mb.request_to_mission_inputs(request5, notes=notes)
    assert isinstance(inputs, MissionInputs) and isinstance(cfg, PlannerConfig)
    assert inputs.home.site_id == "H" and inputs.home.kind == "home" and inputs.home.alt_msl_m == HOME_MSL
    assert [s.site_id for s in inputs.alternates] == ["ALT1"]
    (area,) = inputs.areas
    assert area.area_id == "Z1" and area.sensor_req == "any" and len(area.polygon_wgs84) == 5
    assert [u.uav_id for u in inputs.fleet] == ["A1", "B1"]
    assert [u.cls for u in inputs.fleet] == ["A", "B"]
    assert inputs.fleet[0].available_kinds == frozenset({"lidar"})
    assert inputs.fleet[1].available_kinds == frozenset({"rgb"})
    assert inputs.fleet[0].sortie.available_wh == 1350.0 and inputs.fleet[0].sortie.pack_nominal_wh == 1500.0
    (wl,) = inputs.wind
    assert wl.speed_mps == 4.0 and wl.dir_from_deg == 270.0 and wl.gust_mps == 6.0 and wl.z_ref == "agl"
    assert inputs.c2.range_km == 25.0 and inputs.airspace.terrain_clearance_m == 36.0
    assert inputs.airspace.max_agl_m == 120.0 and inputs.airspace.waiver_max_agl_m == 200.0
    assert inputs.weather_obs.mean_wind_mps == 4.0 and inputs.weather_obs.wind_dir_from_deg == 270.0
    assert set(inputs.payload_reqs) >= {"default", "default_rgb"}
    # 合成平地 DEM 涵蓋 home、備降點與區域，且高程 = home MSL
    dem = inputs.dem
    lat_min, lat_max, lon_min, lon_max = _bbox(request5["areas"][0]["polygon"] + [request5["home"]] + request5["alternates"])
    assert dem.lat0_deg < lat_min and dem.lon0_deg < lon_min
    assert dem.lat_max_deg > lat_max and dem.lon_max_deg > lon_max
    assert np.allclose(dem.z_msl, HOME_MSL) and dem.sigma_z_m == 2.0
    assert cfg.dem_step_m == 50.0
    assert any(n.startswith("dem_synthetic_flat") for n in notes)
    assert not any(n.startswith("c2_range_assumed") for n in notes)


def test_request_defaults_and_notes():
    req = {"home": {"lat": HOME[0], "lon": HOME[1]},
           "areas": [{"area_id": "Z1", "polygon": _latlon_ring(PENTAGON_ENU)}]}
    notes: list[str] = []
    inputs, cfg = mb.request_to_mission_inputs(req, notes=notes)
    tags = {n.split("(")[0] for n in notes}
    assert {"default_drones", "alternate_auto_placed", "energy_assumed", "wind_assumed", "weather_obs_assumed",
            "c2_range_assumed", "terrain_clearance_assumed", "airspace_assumed", "dem_synthetic_flat"} <= tags
    assert [u.uav_id for u in inputs.fleet] == ["A1", "B1"]
    assert inputs.fleet[0].sortie.soc_fraction == 0.9 and inputs.fleet[0].sortie.available_wh is None
    assert len(inputs.alternates) == 1 and inputs.alternates[0].site_id == "ALT1"
    m_lat, m_lon = mb.metres_per_degree(HOME[0])
    d = math.hypot((inputs.alternates[0].lat_deg - HOME[0]) * m_lat, (inputs.alternates[0].lon_deg - HOME[1]) * m_lon)
    assert d == pytest.approx(1000.0, rel=0.02)
    assert inputs.c2.range_km == mb.BridgeDefaults().c2_range_km
    assert inputs.airspace.terrain_clearance_m == mb.BridgeDefaults().terrain_clearance_m
    assert inputs.airspace.waiver_max_agl_m == 200.0
    assert inputs.wind[0].speed_mps == 0.0
    assert np.allclose(inputs.dem.z_msl, 0.0)          # 無 home.alt_msl → 平地 0 m
    assert cfg.dem_step_m == 50.0


def test_request_variants():
    """點格式 [lat, lon]、分層風、逐構型 c2、planner 覆寫、感測器清單、多區。"""
    poly = [[p["lat"], p["lon"]] for p in _latlon_ring(PENTAGON_ENU)]
    req = {
        "home": [HOME[0], HOME[1], HOME_MSL],
        "areas": {"Z1": {"polygon": poly, "sensor_req": "rgb", "priority": 2.0, "time_window_s": [0, 3600]},
                  "Z2": {"polygon": _latlon_ring([(2000, 1500), (3000, 1500), (3000, 2200), (2000, 2200)]),
                         "sensor_req": "lidar"}},
        "drones": [{"id": "P1", "platform": "Pioneer PNP", "sensors": ["lidar", "rgb"], "soc": 0.8,
                    "lost_link_policy": "RTB"},
                   {"id": "H1", "platform": "HERO", "available_wh": 300, "nominal_wh": 420}],
        "wind": [{"z_bottom_m": 0, "z_top_m": 300, "speed_mps": 3, "dir_from_deg": 200},
                 {"z_bottom_m": 300, "z_top_m": 800, "speed_mps": 6, "dir_from_deg": 220, "z_ref": "agl"}],
        "c2_range_km": {"A": 25, "B": 30},
        "obstacles": [{"obs_id": "T1", "coords": [_latlon_ring([(1000.0, 200.0)])[0]], "height_agl_m": 40, "category": "tower"}],
        "airspace": {"max_agl_m": 120, "waiver_max_agl_m": 200, "terrain_clearance_m": 40,
                     "no_fly": [{"zone_id": "NF1", "polygon": _latlon_ring([(4500, -900), (4900, -900), (4900, -700), (4500, -700)])}]},
        "planner": {"dem_step_m": 40.0, "d_sep_m": 120.0},
        "recovery_site_id": "H",
    }
    inputs, cfg = mb.request_to_mission_inputs(req)
    assert sorted(a.area_id for a in inputs.areas) == ["Z1", "Z2"]
    z1 = inputs.area_by_id("Z1")
    assert z1.sensor_req == "rgb" and z1.priority == 2.0 and z1.time_window_s == (0.0, 3600.0)
    assert inputs.fleet[0].available_kinds == frozenset({"lidar", "rgb"}) and inputs.fleet[0].sortie.soc_fraction == 0.8
    assert inputs.c2.lost_link_policy == {"P1": "RTB"}
    assert inputs.c2.range_km == {"A": 25.0, "B": 30.0}
    assert len(inputs.wind) == 2 and inputs.wind[1].z_top_m == 800.0
    assert inputs.obstacles[0].obs_id == "T1" and inputs.obstacles[0].geom_type == "point"
    assert inputs.airspace.no_fly[0].zone_id == "NF1" and inputs.airspace.terrain_clearance_m == 40.0
    assert cfg.dem_step_m == 40.0 and cfg.d_sep_m == 120.0
    assert inputs.recovery_site_id == "H"


def test_request_errors():
    base = {"home": {"lat": HOME[0], "lon": HOME[1]}, "areas": [{"area_id": "Z1", "polygon": _latlon_ring(PENTAGON_ENU)}]}
    with pytest.raises(MissionInputError):
        mb.request_to_mission_inputs({"home": base["home"]})                       # 缺 areas
    with pytest.raises(MissionInputError):
        mb.request_to_mission_inputs({"areas": base["areas"]})                     # 缺 home
    with pytest.raises(MissionInputError):
        mb.request_to_mission_inputs(dict(base, drones=[{"id": "X", "platform": "phantom"}]))
    with pytest.raises(MissionInputError):
        mb.request_to_mission_inputs(dict(base, planner={"not_a_param": 1}))
    with pytest.raises(MissionInputError):
        mb.request_to_mission_inputs(dict(base, areas=[{"area_id": "Z1", "polygon": _latlon_ring(PENTAGON_ENU[:2])}]))
    with pytest.raises(MissionInputError):
        mb.request_to_mission_inputs(dict(base, drones=[{"id": "X", "platform": "hero_pnp", "sensors": ["sonar"]}]))
    # 錯誤不拋出時 → status ERROR
    res = mb.handle_request_full({"home": base["home"]})
    assert res.status["status"] == "ERROR" and res.payloads == {} and res.status["errors"]
    res = mb.handle_request_full(b"{not json")
    assert res.status["status"] == "ERROR"


# ---------------------------------------------------------------------------
# handle_request：五角形 + 1 Pioneer + 1 HERO
# ---------------------------------------------------------------------------

def test_handle_request_5corner(request5, result5):
    res = result5
    assert res.ok, res.status
    assert res.status["status"] in ("GO", "GO_WITH_WARNINGS")
    assert res.mission_id == "t-pent" and res.plan is not None and res.ctx is not None
    assert set(res.payloads) == {"A1", "B1"}
    assert res.status["drones"] == {"A1": res.payloads["A1"]["summary"]["status"], "B1": res.payloads["B1"]["summary"]["status"]}
    assert res.status["topics"]["A1"] == "hvdccpp/mission/t-pent/A1"
    assert res.elapsed_s < 10.0
    lat_min, lat_max, lon_min, lon_max = _bbox(request5["areas"][0]["polygon"] + [request5["home"]] + request5["alternates"])
    pad = 0.02                                                    # ≈ 2 km（起飛迴圈／轉場走廊）
    for uid, pl in res.payloads.items():
        assert pl["mission_id"] == "t-pent" and pl["drone_id"] == uid
        assert pl["platform"] == {"A1": "pioneer_pnp", "B1": "hero_pnp"}[uid] and pl["cls"] == uid[0]
        s = pl["summary"]
        assert s["status"] in ("GO", "GO_WITH_WARNINGS")
        assert s["n_waypoints"] == len(pl["waypoints"]) > 10 and s["strips"]
        assert s["t_land_s"] > s["t_arr_ptc_s"] > s["t_takeoff_s"] >= 0.0
        assert s["energy_margin_wh"] > 0 and s["energy_used_wh"] > 0 and s["distance_m"] > 1000
        assert s["home"] == {"lat": HOME[0], "lon": HOME[1], "alt_msl": HOME_MSL}
        assert s["frame"] == "MAV_FRAME_GLOBAL_RELATIVE_ALT" and s["landing_site"] == "H"
        assert s["makespan_s"] == pytest.approx(res.plan.makespan_s)
        # QGC WPL 110
        txt = pl["qgc_wpl"]
        assert txt.startswith("QGC WPL 110\n")
        lines = txt.strip().splitlines()[1:]
        rows = [l.split("\t") for l in lines]
        assert all(len(r) == 12 for r in rows)
        assert [int(r[0]) for r in rows] == list(range(len(rows)))
        assert rows[0][1] == "1" and rows[0][2] == "0" and rows[0][3] == "16"          # home 列
        assert float(rows[0][8]) == pytest.approx(HOME[0]) and float(rows[0][10]) == pytest.approx(HOME_MSL)
        cmds = collections.Counter(int(r[3]) for r in rows)
        assert cmds[84] == 1 and cmds[85] == 1 and cmds[189] == 1 and cmds[178] >= 1 and cmds[16] > 10
        assert int(rows[1][3]) == 84 and rows[1][2] == "3"
        assert int(rows[-1][3]) == 85 and float(rows[-1][10]) == 0.0
        assert sum(1 for r in rows if int(r[1]) == 1) == 1
        trig = 181 if uid == "A1" else 206
        assert cmds[trig] >= 2 and cmds[trig] % 2 == 0                               # 每條帶開／關成對
        assert s["n_qgc_items"] == len(rows)
        # 航點清單
        wps = pl["waypoints"]
        ts = [w["t_s"] for w in wps]
        assert all(b >= a - 1e-9 for a, b in zip(ts, ts[1:]))
        assert wps[0]["phase"] == "TAKEOFF" and wps[-1]["phase"] == "LAND"
        assert any(w["sensor_on"] for w in wps) and any(w["phase"] == "OPERATE" for w in wps)
        for w in wps:
            assert w["alt_rel"] == pytest.approx(w["alt_msl"] - HOME_MSL, abs=1e-6)
            assert lat_min - pad <= w["lat"] <= lat_max + pad and lon_min - pad <= w["lon"] <= lon_max + pad
            assert w["mode"] in ("HOVER", "TRANSITION", "CRUISE")
            if w["sensor_on"]:
                assert w["phase"] == "OPERATE" and w["strip_id"] is not None and w["dir"] in (-1, 1)
        assert max(w["alt_agl"] for w in wps) <= 200.0 + 1e-6
        assert pl["rally"] and pl["rally"][0]["id"] == "ALT1"
        assert pl["geofence"] and len(pl["geofence"]["inclusion"]) >= 4 and pl["geofence"]["alt_max_rel_m"] == 200.0
    # A 與 B 的作業高度分層
    za = [w["alt_agl"] for w in res.payloads["A1"]["waypoints"] if w["phase"] == "OPERATE"]
    zb = [w["alt_agl"] for w in res.payloads["B1"]["waypoints"] if w["phase"] == "OPERATE"]
    assert min(za) - max(zb) >= 50.0
    # 整體 JSON 可序列化（無 NaN）
    json.dumps(res.payloads, allow_nan=False)
    json.dumps(res.status, allow_nan=False)


def test_qgc_loader_roundtrip(result5):
    pytest.importorskip("pymavlink")
    os.environ.setdefault("MAVLINK20", "1")
    from pymavlink import mavwp
    for uid, pl in result5.payloads.items():
        path = os.path.join(tempfile.mkdtemp(), f"{uid}.waypoints")
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(pl["qgc_wpl"])
        ld = mavwp.MAVWPLoader()
        n = ld.load(path)
        assert n == pl["summary"]["n_qgc_items"] == ld.count()
        assert ld.wp(0).current == 1 and ld.wp(0).command == mb._MAV.MAV_CMD_NAV_WAYPOINT
        assert ld.wp(1).command == mb._MAV.MAV_CMD_NAV_VTOL_TAKEOFF
        assert ld.wp(n - 1).command == mb._MAV.MAV_CMD_NAV_VTOL_LAND


def test_plan_to_payloads_options(result5):
    res = result5
    pay = mb.plan_to_payloads(res.plan, res.ctx, res.cfg, mission_id="x", assumptions=["a"])
    assert set(pay) == {"A1", "B1"} and pay["A1"]["mission_id"] == "x" and pay["A1"]["summary"]["assumptions"] == ["a"]
    assert len(pay["A1"]["waypoints"]) == len(res.payloads["A1"]["waypoints"])
    # MSL frame：frame 0、alt = alt_msl
    pay_msl = mb.plan_to_payloads(res.plan, res.ctx, res.cfg, mission_id="x", qgc=mb.QgcOptions(frame="msl"))
    rows = [l.split("\t") for l in pay_msl["B1"]["qgc_wpl"].strip().splitlines()[1:]]
    navs = [r for r in rows if int(r[3]) == 16][1:]
    assert all(r[2] == "0" for r in navs) and float(navs[5][10]) > HOME_MSL + 50.0
    assert pay_msl["B1"]["summary"]["frame"] == "MAV_FRAME_GLOBAL"
    # 關閉相機／速度指令
    pay_min = mb.plan_to_payloads(res.plan, res.ctx, res.cfg, qgc=mb.QgcOptions(cam_trigger=False, speed_cmd=False, land_start=False))
    cmds = collections.Counter(int(l.split("\t")[3]) for l in pay_min["A1"]["qgc_wpl"].strip().splitlines()[1:])
    assert cmds[181] == 0 and cmds[206] == 0 and cmds[178] == 0 and cmds[189] == 0 and cmds[84] == 1 and cmds[85] == 1
    # qgc_wpl_from_plan 與 payload 一致
    spec = res.ctx.specs["A"]
    txt = mb.qgc_wpl_from_plan(res.plan.plans["A1"], spec, (HOME[0], HOME[1], HOME_MSL), res.ctx.frame, plan=res.plan, ctx=res.ctx)
    assert txt == res.payloads["A1"]["qgc_wpl"]


def test_status_payload_nogo():
    """c2 範圍無效 → build_context/preflight NO_GO → 每架 payload NO_GO 且 status 含錯誤。"""
    req = {"mission_id": "nogo", "home": {"lat": HOME[0], "lon": HOME[1], "alt_msl": HOME_MSL},
           "areas": [{"area_id": "Z1", "polygon": _latlon_ring(PENTAGON_ENU)}],
           "drones": [{"id": "A1", "platform": "pioneer_pnp"}],
           "airspace": {"max_agl_m": 60.0}}                     # 上限 60 m < min_fw 120 → A 不可行
    res = mb.handle_request_full(req)
    assert res.status["status"] in ("NO_GO", "ERROR"), res.status
    assert set(res.payloads) == {"A1"} and res.payloads["A1"]["summary"]["status"] in ("NO_GO", "ERROR")
    assert res.payloads["A1"]["qgc_wpl"] == "" and res.payloads["A1"]["waypoints"] == []
    assert res.status["errors"] or res.payloads["A1"]["summary"]["errors"]
    json.dumps(res.payloads, allow_nan=False)


# ---------------------------------------------------------------------------
# legacy 4 角點 payload
# ---------------------------------------------------------------------------

LEGACY_RING = [(1500.0, -500.0), (2700.0, -500.0), (2700.0, 400.0), (1500.0, 400.0)]


def test_legacy_payload_detection_and_conversion():
    pts = _latlon_ring(LEGACY_RING)
    legacy = {"waypoints": pts, "altitude": 100, "speed": 12}
    assert mb.is_legacy_payload(legacy)
    assert not mb.is_legacy_payload({"areas": [], "waypoints": pts})
    notes: list[str] = []
    req = mb.legacy_to_request(legacy, notes=notes, mission_id="lg")
    assert req["mission_id"] == "lg" and req["areas"][0]["sensor_req"] == "any"
    assert [(p["lat"], p["lon"]) for p in req["areas"][0]["polygon"]] == [(p["lat"], p["lon"]) for p in pts]
    assert [d["id"] for d in req["drones"]] == ["A1", "B1"]
    assert "home" in req and any(n.startswith("home_auto_placed") for n in notes)
    assert any(n.startswith("legacy_flight_params_ignored") for n in notes)
    assert req["home"]["lon"] < min(p["lon"] for p in pts)
    notes2: list[str] = []
    inputs, cfg = mb.request_to_mission_inputs(legacy, notes=notes2)
    assert "legacy_payload" in notes2
    assert inputs.areas[0].sensor_req == "any" and len(inputs.areas[0].polygon_wgs84) == 4
    assert [u.uav_id for u in inputs.fleet] == ["A1", "B1"] and inputs.home.site_id == "H"
    # 自訂機隊／home
    inputs2, _ = mb.request_to_mission_inputs(legacy, default_drones=[{"id": "H7", "platform": "hero_pnp"}],
                                               default_home={"lat": HOME[0], "lon": HOME[1], "alt_msl": HOME_MSL})
    assert [u.uav_id for u in inputs2.fleet] == ["H7"] and inputs2.home.lat_deg == HOME[0]
    # 純點清單與 polygon 鍵亦可；< 3 點錯誤
    assert mb.request_to_mission_inputs([[p["lat"], p["lon"]] for p in pts])[0].areas[0].area_id == "Z1"
    assert mb.request_to_mission_inputs({"polygon": pts[:3]})[0].areas[0].area_id == "Z1"
    with pytest.raises(MissionInputError):
        mb.request_to_mission_inputs({"waypoints": pts[:2]})


def test_legacy_4point_end_to_end():
    pts = _latlon_ring(LEGACY_RING)
    payloads = mb.handle_request({"waypoints": pts, "mission_id": "lg4"},
                                 default_home={"lat": HOME[0], "lon": HOME[1], "alt_msl": HOME_MSL})
    assert set(payloads) == {"A1", "B1"}
    for uid, pl in payloads.items():
        assert pl["summary"]["status"] in ("GO", "GO_WITH_WARNINGS"), pl["summary"]["errors"]
        assert pl["qgc_wpl"].startswith("QGC WPL 110\n") and len(pl["waypoints"]) > 10
        assert "legacy_payload" in pl["summary"]["assumptions"]
        assert pl["summary"]["uncovered"] == []


# ---------------------------------------------------------------------------
# MqttBridge（假 client，不需 broker）
# ---------------------------------------------------------------------------

class _FakeClient:
    def __init__(self):
        self.published: list[tuple[str, str, int, bool]] = []
        self.subscribed: list[tuple[str, int]] = []
        self.connected = None
        self.on_connect = None
        self.on_message = None
        self.on_disconnect = None

    def connect(self, host, port, keepalive):
        self.connected = (host, port, keepalive)
        self.on_connect(self, None, {}, 0)                       # v1 簽章

    def subscribe(self, topic, qos=0):
        self.subscribed.append((topic, qos))

    def publish(self, topic, payload, qos=0, retain=False):
        self.published.append((topic, payload, qos, retain))

    def loop_forever(self):
        pass

    def disconnect(self):
        self.connected = None


class _Msg:
    def __init__(self, topic, payload):
        self.topic, self.payload = topic, payload


def _fake_result(payload) -> mb.BridgeResult:
    mid = payload["mission_id"]
    pays = {d["id"]: {"mission_id": mid, "drone_id": d["id"], "qgc_wpl": "QGC WPL 110\n", "waypoints": [],
                      "summary": {"status": "GO"}} for d in payload.get("drones", mb.DEFAULT_DRONES)}
    return mb.BridgeResult(mid, pays, {"status": "GO", "mission_id": mid, "event": "planned"})


def test_bridge_fake_client_dispatch():
    seen = []
    br = mb.MqttBridge("broker.local", 1884, client=_FakeClient(), handler=_fake_result, on_result=seen.append, qos=1)
    br.connect()
    fc = br.client
    assert fc.connected == ("broker.local", 1884, 60)
    assert set(t for t, _q in fc.subscribed) == {mb.TOPIC_REQUEST, *mb.LEGACY_TOPICS}
    assert fc.published[0][0] == mb.TOPIC_STATUS and json.loads(fc.published[0][1])["event"] == "online"
    n0 = len(fc.published)
    req = {"mission_id": "m9", "home": {"lat": HOME[0], "lon": HOME[1]}, "areas": [], "drones": [{"id": "A1"}, {"id": "B1"}]}
    br.on_message(fc, None, _Msg(mb.TOPIC_REQUEST, json.dumps(req).encode("utf-8")))
    topics = [t for t, *_ in fc.published[n0:]]
    assert topics == [mb.TOPIC_STATUS, "hvdccpp/mission/m9/A1", "hvdccpp/mission/m9/B1", mb.TOPIC_STATUS]
    assert json.loads(fc.published[n0][1])["event"] == "planning"
    assert json.loads(fc.published[n0 + 1][1])["drone_id"] == "A1"
    assert json.loads(fc.published[-1][1])["status"] == "GO"
    assert all(q == 1 for _t, _p, q, _r in fc.published)
    assert len(seen) == 1 and seen[0].mission_id == "m9" and br.results[0] is seen[0]
    # 缺 mission_id → 自動產生且回應主題一致
    br.on_message(fc, None, _Msg(mb.TOPIC_REQUEST, json.dumps({"home": {}, "areas": [], "drones": [{"id": "X"}]})))
    last = json.loads(fc.published[-2][1])
    assert fc.published[-2][0] == f"hvdccpp/mission/{last['mission_id']}/X"
    # 壞 JSON → 只發狀態 error
    out = br.handle_bytes(mb.TOPIC_REQUEST, b"{oops")
    assert out[0][0] == mb.TOPIC_STATUS and out[0][1]["event"] == "error"
    # v2 簽章 on_connect 亦可
    br.on_connect(fc, None, {}, 0, None)
    br.disconnect()
    assert fc.connected is None


def test_bridge_legacy_chunks_and_points():
    got = []

    def handler(payload):
        got.append(payload)
        return _fake_result(dict(payload, drones=[{"id": "A1"}]))

    br = mb.MqttBridge(client=_FakeClient(), handler=handler, legacy_n_points=4)
    pts = _latlon_ring(LEGACY_RING)
    # 分段 {"waypoint","index","total"}：亂序、齊全才處理
    for i in (2, 0, 3):
        assert br.handle_bytes("drone/waypoints", json.dumps({"waypoint": pts[i], "index": i, "total": 4})) == []
    out = br.handle_bytes("drone/waypoints", json.dumps({"waypoint": pts[1], "index": 1, "total": 4, "mission_id": "c1"}))
    assert got and got[-1]["mission_id"] == "c1" and got[-1]["waypoints"] == pts
    assert [t for t, _p in out] == ["hvdccpp/mission/c1/A1", mb.TOPIC_STATUS]
    # 單點累積
    for p in pts[:3]:
        assert br.handle_bytes("drone/waypoints", json.dumps(p)) == []
    out = br.handle_bytes("drone/waypoints", json.dumps(pts[3]))
    assert got[-1]["waypoints"] == pts and out
    # 整批 legacy
    out = br.handle_bytes("drone/waypoints", json.dumps({"waypoints": pts}))
    assert got[-1]["waypoints"] == pts and out[0][0].startswith("hvdccpp/mission/")


def test_bridge_real_handler_on_legacy_topic():
    """假 client + 真 handler：legacy 4 點 → 兩架回應（與 test_legacy_4point_end_to_end 同情境）。"""
    br = mb.MqttBridge(client=_FakeClient(), default_home={"lat": HOME[0], "lon": HOME[1], "alt_msl": HOME_MSL})
    out = br.handle_bytes("drone/waypoints", json.dumps({"waypoints": _latlon_ring(LEGACY_RING), "mission_id": "lgb"}))
    topics = [t for t, _p in out]
    assert topics == ["hvdccpp/mission/lgb/A1", "hvdccpp/mission/lgb/B1", mb.TOPIC_STATUS]
    assert out[-1][1]["status"] in ("GO", "GO_WITH_WARNINGS")
    assert out[0][1]["qgc_wpl"].startswith("QGC WPL 110")
    assert br.results[0].ok
