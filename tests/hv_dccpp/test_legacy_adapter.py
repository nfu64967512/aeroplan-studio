"""tests/test_legacy_adapter.py — HVDCCPPGeneratorAdapter（既有規劃器替換層）。

情境：雲林斗六附近 (23.70N, 120.42E) 之 5 角凹多邊形（≈ 1.3 km × 1.3 km）、`FlightParameters` 型之 SimpleNamespace、
HERO PNP 單機；平地合成 DEM（base 50 m MSL）。
"""
from __future__ import annotations

import math
import os
import time
from types import SimpleNamespace

import numpy as np
import pytest

os.environ.setdefault("MAVLINK20", "1")

from hv_dccpp.integration import legacy_adapter as LA
from hv_dccpp.integration.legacy_adapter import (HVDCCPPGeneratorAdapter, LegacyCollisionAvoidance, MAV, MissionItem,
                                                 mission_items_to_qgc, plan_to_mission_items, thin_waypoints)
from hv_dccpp.mission import Mode, Phase, SortieEnergy, Waypoint
from hv_dccpp.platform import PLATFORM_HERO, PLATFORM_PIONEER

CORNERS = [(23.700, 120.420), (23.710, 120.421), (23.712, 120.432), (23.704, 120.427), (23.698, 120.431)]
"""5 角凹多邊形（第 4 點為反射頂點）。"""
PARAMS = SimpleNamespace(altitude=120, angle=0, spacing=50, speed=19, yaw_speed=60, safety_distance=5)

CMD_TAKEOFF = MAV.MAV_CMD_NAV_VTOL_TAKEOFF          # 84
CMD_LAND = MAV.MAV_CMD_NAV_VTOL_LAND                # 85
CMD_WP = MAV.MAV_CMD_NAV_WAYPOINT                   # 16
CMD_YAW = MAV.MAV_CMD_CONDITION_YAW                 # 115
CMD_SPEED = MAV.MAV_CMD_DO_CHANGE_SPEED             # 178
CMD_CAM = MAV.MAV_CMD_DO_SET_CAM_TRIGG_DIST         # 206
CMD_LAND_START = MAV.MAV_CMD_DO_LAND_START          # 189
CMD_LOITER_ALT = MAV.MAV_CMD_NAV_LOITER_TO_ALT      # 31


def _cols(line: str) -> list[str]:
    return line.split("\t")


def _cmds(lines: list[str]) -> list[int]:
    return [int(_cols(l)[3]) for l in lines[1:]]


@pytest.fixture(scope="module")
def hero_mission():
    ad = HVDCCPPGeneratorAdapter("hero_pnp")
    t0 = time.perf_counter()
    lines, path = ad.generate_complete_mission(CORNERS, PARAMS, 0, 1, True, 0.0)
    return ad, lines, path, time.perf_counter() - t0


# ---------------------------------------------------------------------------
# generate_complete_mission：QGC 格式
# ---------------------------------------------------------------------------

def test_qgc_header_and_12_columns(hero_mission):
    _ad, lines, _path, dt = hero_mission
    assert lines[0] == "QGC WPL 110"
    assert len(lines) > 10
    for l in lines[1:]:
        assert len(_cols(l)) == 12, l
    # seq 連續、第 0 列 current=1 其餘 0
    seqs = [int(_cols(l)[0]) for l in lines[1:]]
    assert seqs == list(range(len(seqs)))
    assert [int(_cols(l)[1]) for l in lines[1:]] == [1] + [0] * (len(seqs) - 1)
    assert dt < 5.0


def test_home_row_is_frame0_cmd16(hero_mission):
    _ad, lines, _path, _dt = hero_mission
    c = _cols(lines[1])
    assert int(c[2]) == MAV.MAV_FRAME_GLOBAL and int(c[3]) == CMD_WP
    assert abs(float(c[8]) - CORNERS[0][0]) < 1e-6 and abs(float(c[9]) - CORNERS[0][1]) < 1e-6
    assert float(c[10]) == pytest.approx(50.0, abs=0.5)      # home MSL（平地 50 m）
    # 其餘列 frame 3（相對 home）
    assert all(int(_cols(l)[2]) == MAV.MAV_FRAME_GLOBAL_RELATIVE_ALT for l in lines[2:])


def test_vtol_commands_present_no_condition_yaw(hero_mission):
    _ad, lines, _path, _dt = hero_mission
    cmds = _cmds(lines)
    assert cmds.count(CMD_TAKEOFF) == 1 and cmds.count(CMD_LAND) == 1
    assert CMD_YAW not in cmds
    assert MAV.MAV_CMD_NAV_TAKEOFF not in cmds          # 不在任務中段插入 NAV_TAKEOFF
    assert MAV.MAV_CMD_NAV_RETURN_TO_LAUNCH not in cmds
    assert cmds[-1] == CMD_LAND
    assert cmds.index(CMD_TAKEOFF) < 4
    assert CMD_LAND_START in cmds and cmds.index(CMD_LAND_START) < len(cmds) - 1
    # 相機觸發成對（開/關）、且開在 84 之後、關在 85 之前
    cam = [float(_cols(l)[4]) for l in lines[1:] if int(_cols(l)[3]) == CMD_CAM]
    assert len(cam) >= 2 and len(cam) % 2 == 0
    assert cam[0] > 0 and cam[-1] == 0.0
    assert all((v > 0) == (i % 2 == 0) for i, v in enumerate(cam))
    # VTOL_TAKEOFF 高度 = H_ts（HERO 50 m）
    to = [l for l in lines[1:] if int(_cols(l)[3]) == CMD_TAKEOFF][0]
    assert float(_cols(to)[10]) == pytest.approx(PLATFORM_HERO.transition_start_agl_m)
    # 速度指令為 params.speed（19 = HERO 巡航）且 ≥ v_min
    sp = [float(_cols(l)[5]) for l in lines[1:] if int(_cols(l)[3]) == CMD_SPEED]
    assert sp and max(sp) <= 19.0 + 1e-9 and min(sp) >= PLATFORM_HERO.v_min_mission() - 1e-9


def test_waypoint_altitudes_and_density(hero_mission):
    ad, lines, _path, _dt = hero_mission
    wp = [l for l in lines[1:] if int(_cols(l)[3]) == CMD_WP][1:]     # 去掉 home 列
    alts = np.array([float(_cols(l)[10]) for l in wp])
    assert alts.min() >= 0.0 and alts.max() <= 200.0 + 1.0            # ≤ waiver 200 m AGL（平地 → 相對高 ≈ AGL）
    assert np.median(alts) == pytest.approx(120.0, abs=5.0)           # 作業高度 = params.altitude
    # 接受半徑 r_acc = min(R_eff/2, 30)
    assert all(0 < float(_cols(l)[5]) <= 30.0 + 1e-9 for l in wp)
    # 點數已刪減：遠少於原始航點；且 ≤ 700（ArduPilot 常見任務容量）
    n_raw = len(ad.last_plan.plans["B1"].waypoints)
    assert len(wp) < 0.7 * n_raw and len(wp) <= 700, (len(wp), n_raw)


def test_latlon_path_inside_3km_box(hero_mission):
    _ad, lines, path, _dt = hero_mission
    assert path and all(isinstance(p, tuple) and len(p) == 2 for p in path)
    lat_c = sum(p[0] for p in CORNERS) / 5
    lon_c = sum(p[1] for p in CORNERS) / 5
    for lat, lon in path:
        assert abs(lat - lat_c) * 111132.0 < 3000.0
        assert abs(lon - lon_c) * 111412.0 * math.cos(math.radians(lat_c)) < 3000.0
    # 首點 = 起飛點（home = 第一角點）、末點 = 落點（home）
    assert path[0] == pytest.approx(CORNERS[0], abs=1e-6)
    assert path[-1] == pytest.approx(CORNERS[0], abs=1e-6)
    # 與 NAV 列數一致
    n_nav = sum(1 for l in lines[2:] if int(_cols(l)[3]) in (CMD_WP, CMD_TAKEOFF, CMD_LAND, CMD_LOITER_ALT))
    assert len(path) == n_nav


def test_plan_covers_area_and_forced_angle(hero_mission):
    ad, _lines, _path, _dt = hero_mission
    mp = ad.last_plan
    assert mp is not None and mp.uncovered == [] and set(mp.plans) == {"B1"}
    up = mp.plans["B1"]
    assert up.waypoints[0].phase is Phase.TAKEOFF and up.waypoints[-1].phase is Phase.LAND
    assert up.energy_margin_wh > 0
    # angle=0（既有語意：東西向條帶）→ θ = 90°（模 180°）
    for c in mp.cells:
        if c.feasible.get("B"):
            a = math.degrees(c.alpha_rad["B"]) % 180.0
            assert min(abs(a - 90.0), abs(a - 270.0)) < 1e-6
    # 條帶間距 = spacing 提示（50 m：side_overlap = 1 − 50/176）
    sp = [s.spacing_m for s in mp.strips.values() if s.cls == "B"]
    assert sp and np.median(sp) == pytest.approx(50.0, abs=1.5)
    assert any(w.startswith("home_defaulted") for w in ad.warnings)


def test_mavwp_loader_round_trip(hero_mission, tmp_path):
    pytest.importorskip("pymavlink")
    from pymavlink import mavwp
    _ad, lines, _path, _dt = hero_mission
    p = tmp_path / "region1.waypoints"
    p.write_text("\n".join(lines) + "\n", encoding="utf-8")
    loader = mavwp.MAVWPLoader()
    n = loader.load(str(p))
    assert n == len(lines) - 1
    w = loader.wp(1)
    assert w.command in (CMD_SPEED, MAV.MAV_CMD_NAV_DELAY)
    cmds = [loader.wp(i).command for i in range(n)]
    assert CMD_TAKEOFF in cmds and cmds[-1] == CMD_LAND


# ---------------------------------------------------------------------------
# 其他既有簽章
# ---------------------------------------------------------------------------

def test_subdivide_polygon_two_parts():
    ad = HVDCCPPGeneratorAdapter("hero_pnp")
    parts = ad.subdivide_polygon(CORNERS, 2)
    assert len(parts) == 2
    from shapely.geometry import Polygon
    from hv_dccpp.geo import GeoFrame
    fr = GeoFrame(CORNERS[0][0], CORNERS[0][1], 0.0)

    def poly(ring):
        xy = fr.to_enu_xy([p[0] for p in ring], [p[1] for p in ring])
        return Polygon(xy).buffer(0)
    whole = poly(CORNERS)
    ps = [poly(r) for r in parts]
    assert all(len(r) >= 3 for r in parts)
    assert all(q.area > 0.15 * whole.area for q in ps)
    assert abs(sum(q.area for q in ps) - whole.area) < 0.02 * whole.area
    assert ps[0].intersection(ps[1]).area < 0.01 * whole.area
    assert ad.subdivide_rectangle(CORNERS, 3) and len(ad.subdivide_rectangle(CORNERS, 3)) == 3
    assert ad.subdivide_polygon(CORNERS, 1) == [tuple(c) for c in CORNERS] or ad.subdivide_polygon(CORNERS, 1) == [list(map(tuple, CORNERS))]


def test_collision_avoidance_loiter_delay():
    ca = LegacyCollisionAvoidance(safety_distance=5.0)
    assert ca.calculate_loiter_delay([], (23.7, 120.42), 19.0) == 0.0
    # 前機沿北向每 100 m 一點：離開 5 m 安全距離之第一點為第 2 點（100 m）→ 100/20 + 2 = 7 s
    prev = [(23.7 + i * 100.0 / 111132.0, 120.42) for i in range(5)]
    d = ca.calculate_loiter_delay(prev, (23.7, 120.42), 20.0)
    assert d == pytest.approx(7.0, abs=0.05)
    ad = HVDCCPPGeneratorAdapter("hero_pnp")
    assert ad.collision_avoidance.safety_distance == 5.0
    assert ad.collision_avoidance.calculate_loiter_delay(prev, (23.7, 120.42), 20.0) == pytest.approx(7.0, abs=0.05)


def test_waypoints_to_qgc_format_latlon_list():
    ad = HVDCCPPGeneratorAdapter("hero_pnp")
    pts = [(23.700, 120.420), (23.702, 120.420), (23.702, 120.422), (23.700, 120.422), (23.700, 120.420)]
    lines = ad.waypoints_to_qgc_format(pts, PARAMS, 0, 2, loiter_time=12.0)
    assert lines[0] == "QGC WPL 110" and all(len(_cols(l)) == 12 for l in lines[1:])
    cmds = _cmds(lines)
    assert cmds.count(CMD_TAKEOFF) == 1 and cmds[-1] == CMD_LAND and CMD_YAW not in cmds
    assert MAV.MAV_CMD_NAV_DELAY in cmds and float(_cols(lines[1 + cmds.index(MAV.MAV_CMD_NAV_DELAY)])[4]) == 12.0
    wps = [l for l in lines[1:] if int(_cols(l)[3]) == CMD_WP][1:]
    assert all(float(_cols(l)[10]) == pytest.approx(120.0) for l in wps)
    assert ad.waypoints_to_qgc_format([], PARAMS, 0, 1) == ["QGC WPL 110"]


def test_waypoints_to_qgc_format_waypoint_objects(hero_mission):
    ad, lines, _path, _dt = hero_mission
    wps = ad.last_plan.plans["B1"].waypoints
    lines2 = ad.waypoints_to_qgc_format(wps, PARAMS, 0, 1)
    assert lines2 == lines   # 同一航線再序列化 → 逐列相同


# ---------------------------------------------------------------------------
# plan_to_mission_items 單元：點刪減與弧段
# ---------------------------------------------------------------------------

def _wp(x, y, z, phase, sensor_on=False, strip=None, arc=None, va=19.0):
    mode = Mode.HOVER if phase in (Phase.TAKEOFF, Phase.LAND) else (
        Mode.TRANSITION if phase in (Phase.TRANS_OUT, Phase.TRANS_IN) else Mode.CRUISE)
    return Waypoint(x, y, z, 23.7 + y / 111132.0, 120.42 + x / 102000.0, 50.0 + z, z, 0.0, mode,
                    phase, sensor_on, strip, None, None, va, va, 90.0, 90.0, 300.0, "H", True, "RTB", arc, 30.0)


def test_thin_waypoints_straight_line():
    P = np.column_stack([np.linspace(0, 1000, 41), np.zeros(41), np.full(41, 120.0)])
    keep = np.zeros(41, bool)
    assert thin_waypoints(P, keep, 2.0, 3.0, 400.0) == [0, 16, 32, 40]      # L_max 400 m → 每 400 m 一點
    P[20, 1] = 5.0                                                          # 5 m 偏離 → 保留
    idx = thin_waypoints(P, keep, 2.0, 3.0, 1e9)
    assert 20 in idx or 19 in idx


def test_plan_to_mission_items_arc_subsample_and_loiter():
    from hv_dccpp.geo import GeoFrame
    fr = GeoFrame(23.7, 120.42, 50.0)
    R = 120.0
    wps = [_wp(0, 0, 0, Phase.TAKEOFF)]
    wps += [_wp(0, 100 * k, 50.0 + 5 * k, Phase.TRANS_OUT) for k in range(1, 5)]
    # 直線 ENTRY 1 km（每 25 m 一點）
    wps += [_wp(0, 400 + 25 * k, 90.0, Phase.ENTRY) for k in range(1, 41)]
    # 作業條帶 + 整圈盤旋（同一 arc 參數，掃 360°）
    wps += [_wp(25 * k, 1400, 120.0, Phase.OPERATE, True, "S1") for k in range(0, 41)]
    cx, cy = 1000.0, 1400.0 + R
    wps += [_wp(cx + R * math.sin(a), cy - R * math.cos(a), 120.0, Phase.TRANSFER, arc=(cx, cy, R, -1))
            for a in np.linspace(0, 2 * math.pi, 37)]
    wps += [_wp(1000 - 25 * k, 1400 + 2 * R, 120.0, Phase.OPERATE, True, "S2") for k in range(0, 41)]
    wps += [_wp(0, 1400 + 2 * R - 100 * k, 120.0 - 5 * k, Phase.RTB) for k in range(1, 6)]
    wps += [_wp(0, 500, 90.0, Phase.DECEL), _wp(0, 400, 90.0, Phase.DECEL)]
    wps += [_wp(0, 400 - 100 * k, 90.0 - 10 * k, Phase.TRANS_IN) for k in range(1, 5)]
    wps += [_wp(0, 0, 0.0, Phase.LAND)]
    items = plan_to_mission_items(wps, spec=PLATFORM_HERO, home_latlon_msl=(23.7, 120.42, 50.0), frame=fr,
                                  trigger_dist_m=35.0, speed_cap_mps=19.0, loiter_time_s=0.0)
    cmds = [it.command for it in items]
    assert cmds[0] == CMD_WP and items[0].frame == MAV.MAV_FRAME_GLOBAL and items[0].current == 1
    assert cmds.count(CMD_TAKEOFF) == 1 and cmds.count(CMD_LAND) == 1 and cmds[-1] == CMD_LAND
    assert cmds.count(CMD_LOITER_ALT) == 1                       # 整圈 → NAV_LOITER_TO_ALT
    lo = items[cmds.index(CMD_LOITER_ALT)]
    assert lo.param2 == pytest.approx(-R)                        # 左轉 → 負半徑（逆時針）
    c_ll = fr.to_wgs84(np.array([cx, cy, 120.0]))[0]
    assert lo.x == pytest.approx(c_ll[0], abs=1e-7) and lo.y == pytest.approx(c_ll[1], abs=1e-7)
    assert cmds.count(CMD_CAM) == 4                              # 兩條帶 → 開/關各兩次
    cam = [it.param1 for it in items if it.command == CMD_CAM]
    assert cam == [35.0, 0.0, 35.0, 0.0]
    assert cmds.count(CMD_LAND_START) == 1
    n_wp = cmds.count(CMD_WP) - 1
    assert n_wp <= 16, n_wp                                      # 1 km 直線只留端點；條帶只留兩端
    lines = mission_items_to_qgc(items)
    assert all(len(_cols(l)) == 12 for l in lines[1:]) and int(_cols(lines[1])[1]) == 1
    # 弧段未滿整圈 → 每 ≥ 30° 一點
    wps2 = [w for w in wps if not (w.phase is Phase.TRANSFER)]
    half = [_wp(cx + R * math.sin(a), cy - R * math.cos(a), 120.0, Phase.TRANSFER, arc=(cx, cy, R, -1))
            for a in np.linspace(0, math.pi, 19)]
    k0 = next(i for i, w in enumerate(wps2) if w.strip_id == "S2")
    wps2 = wps2[:k0] + half + wps2[k0:]
    items2 = plan_to_mission_items(wps2, spec=PLATFORM_HERO, home_latlon_msl=(23.7, 120.42, 50.0), frame=fr,
                                   trigger_dist_m=35.0, arc_step_deg=30.0)
    c2 = [it.command for it in items2]
    assert CMD_LOITER_ALT not in c2
    assert c2.count(CMD_WP) - 1 <= n_wp + 8                       # 180°/30° ≈ 6 個弧點


# ---------------------------------------------------------------------------
# Pioneer、能量覆寫、多機
# ---------------------------------------------------------------------------

def test_pioneer_and_custom_energy():
    ad = HVDCCPPGeneratorAdapter("pioneer_pnp", home_latlon=(23.703, 120.418), wind=(4.0, 270.0),
                                 energy=SortieEnergy("x", 24.6, payload_kg=5.0, available_wh=1200.0, pack_nominal_wh=1500.0))
    params = SimpleNamespace(altitude=150, angle=45, spacing=80, speed=21.5, yaw_speed=60, safety_distance=5)
    lines, path = ad.generate_complete_mission(CORNERS, params, 1, 2, False, 30.0)
    cmds = _cmds(lines)
    assert cmds.count(CMD_TAKEOFF) == 1 and cmds[-1] == CMD_LAND and CMD_YAW not in cmds
    assert MAV.MAV_CMD_NAV_DELAY in cmds
    assert set(ad.last_plan.plans) == {"A2"} and ad.last_plan.uncovered == []
    assert ad.last_inputs.fleet[0].sortie.available_wh == 1200.0 and ad.last_inputs.fleet[0].uav_id == "A2"
    to = [l for l in lines[1:] if int(_cols(l)[3]) == CMD_TAKEOFF][0]
    assert float(_cols(to)[10]) == pytest.approx(PLATFORM_PIONEER.transition_start_agl_m)
    assert abs(float(_cols(to)[8]) - 23.703) < 1e-6
    for c in ad.last_plan.cells:
        if c.feasible.get("A"):
            assert (math.degrees(c.alpha_rad["A"]) - 135.0) % 180.0 == pytest.approx(0.0, abs=1e-6)


def test_plan_multi_two_drones():
    ad = HVDCCPPGeneratorAdapter("hero_pnp", home_latlon=(23.700, 120.420), angle_mode="ignore")
    res = ad.plan_multi(CORNERS, [{"id": "B1", "platform": "hero_pnp"}, {"id": "B2", "platform": "hero_pnp"}], PARAMS)
    assert set(res) == {"B1", "B2"}
    flown = [u for u, (ln, _p) in res.items() if ln]
    assert flown
    for u in flown:
        lines, path = res[u]
        cmds = _cmds(lines)
        assert cmds.count(CMD_TAKEOFF) == 1 and cmds[-1] == CMD_LAND and path
    # 第二架起飛延遲 → NAV_DELAY
    delays = {u: [float(_cols(l)[4]) for l in res[u][0][1:] if int(_cols(l)[3]) == MAV.MAV_CMD_NAV_DELAY] for u in flown}
    if len(flown) == 2:
        assert any(d and d[0] > 0 for d in delays.values())
    used = [s for u in flown for s, _d in ad.last_plan.plans[u].strips]
    assert len(used) == len(set(used)) and ad.last_plan.uncovered == []


def test_errors_and_duck_typing():
    with pytest.raises(ValueError):
        HVDCCPPGeneratorAdapter("dji")
    ad = HVDCCPPGeneratorAdapter("hero_pnp")
    with pytest.raises(LA.LegacyAdapterError):
        ad.generate_complete_mission(CORNERS[:2], PARAMS, 0, 1)
    with pytest.raises(LA.LegacyAdapterError):
        HVDCCPPGeneratorAdapter("hero_pnp", sensor_kind="lidar").generate_complete_mission(CORNERS, PARAMS, 0, 1)
    # params 為 dict 亦可（duck typing）
    p = {"altitude": 120, "angle": 0, "spacing": 50, "speed": 19}
    lines = ad.waypoints_to_qgc_format([(23.7, 120.42), (23.701, 120.42), (23.7, 120.42)], p, 0, 1)
    assert _cmds(lines).count(CMD_TAKEOFF) == 1
