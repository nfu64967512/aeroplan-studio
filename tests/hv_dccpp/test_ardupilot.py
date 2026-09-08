"""tests/test_ardupilot.py — `hv_dccpp.ardupilot`（UAVPlan → ArduPilot QuadPlane 任務／rally／fence／參數／QGC .plan）。

情境：`scenario.build_mini_example()`（dem_step 50、terrain_margin 1500）經 `planner.plan_mission` 之 MissionPlan（A1 lidar、B1 rgb）。
檢查：pymavlink `mavwp.MAVWPLoader` 與 `read_qgc_wpl` 可讀回；第 0 列 home；首 NAV 為 VTOL_TAKEOFF、末為 VTOL_LAND；
相機觸發開／關成對；項目數 < 300；相對高度 ≥ 0；弧段外切離散偏差 ≤ e_max；參數檔可解析；fence／rally／.plan 結構。
"""
from __future__ import annotations

import copy
import dataclasses
import json
import math
import os
from types import SimpleNamespace

import numpy as np
import pytest
from shapely.geometry import Point, Polygon

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil, mavwp  # noqa: E402

from hv_dccpp import ardupilot as ap  # noqa: E402
from hv_dccpp.context import build_context  # noqa: E402
from hv_dccpp.mission import Mode, Phase, UAVPlan, Waypoint  # noqa: E402
from hv_dccpp.planner import plan_mission  # noqa: E402
from hv_dccpp.scenario import build_hills_scenario, build_mini_example  # noqa: E402

M = mavutil.mavlink


# ---------------------------------------------------------------------------
# fixtures
# ---------------------------------------------------------------------------
@pytest.fixture(scope="module")
def mini():
    inputs, cfg = build_mini_example()
    cfg = dataclasses.replace(cfg, dem_step_m=50.0)
    ctx = build_context(inputs, cfg, terrain_margin_m=1500.0)
    mp = plan_mission(inputs, cfg, ctx=ctx)
    assert set(mp.plans) == {"A1", "B1"}
    items = {uid: ap.plan_to_mission_items(up, ctx, cfg) for uid, up in mp.plans.items()}
    return SimpleNamespace(inputs=inputs, cfg=cfg, ctx=ctx, mp=mp, items=items)


@pytest.fixture(scope="module")
def exported(mini, tmp_path_factory):
    out = tmp_path_factory.mktemp("ap_export")
    summary = ap.export_all(mini.mp, mini.ctx, mini.cfg, out)
    return SimpleNamespace(out=out, summary=summary)


def _nav(items):
    return [it for it in items if it.is_nav]


# ---------------------------------------------------------------------------
# 常數（以 pymavlink 為準）
# ---------------------------------------------------------------------------
def test_constants_match_pymavlink():
    assert ap.MAV_CMD_NAV_VTOL_TAKEOFF == M.MAV_CMD_NAV_VTOL_TAKEOFF == 84
    assert ap.MAV_CMD_NAV_VTOL_LAND == M.MAV_CMD_NAV_VTOL_LAND == 85
    assert ap.MAV_CMD_NAV_LOITER_TO_ALT == M.MAV_CMD_NAV_LOITER_TO_ALT == 31
    assert ap.MAV_CMD_DO_SET_CAM_TRIGG_DIST == M.MAV_CMD_DO_SET_CAM_TRIGG_DIST == 206
    assert ap.MAV_CMD_DO_LAND_START == M.MAV_CMD_DO_LAND_START == 189
    assert ap.MAV_CMD_NAV_RALLY_POINT == M.MAV_CMD_NAV_RALLY_POINT == 5100
    assert ap.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION == 5001
    assert ap.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION == 5002
    assert ap.MAV_CMD_NAV_FENCE_RETURN_POINT == 5000
    assert ap.FRAME_OF == {"relative": 3, "msl": 0, "terrain": 10}
    assert ap.HAVE_PYMAVLINK


# ---------------------------------------------------------------------------
# 任務項目結構
# ---------------------------------------------------------------------------
def test_home_row_and_sequence(mini):
    for uid, items in mini.items.items():
        h = items[0]
        assert (h.seq, h.frame, h.command, h.current) == (0, M.MAV_FRAME_GLOBAL, M.MAV_CMD_NAV_WAYPOINT, 1)
        assert h.x == pytest.approx(mini.inputs.home.lat_deg) and h.y == pytest.approx(mini.inputs.home.lon_deg)
        assert h.z == pytest.approx(100.0, abs=1e-3)          # home MSL（合成 DEM 100 m）
        assert [it.seq for it in items] == list(range(len(items)))
        assert all(it.autocontinue == 1 for it in items)
        assert all(it.current == 0 for it in items[1:])
        nav = _nav(items[1:])
        assert nav[0].command == M.MAV_CMD_NAV_VTOL_TAKEOFF
        assert nav[-1].command == M.MAV_CMD_NAV_VTOL_LAND
        assert 10 < len(items) < 300, (uid, len(items))
        assert sum(1 for it in items if it.command == M.MAV_CMD_NAV_VTOL_TAKEOFF) == 1
        assert sum(1 for it in items if it.command == M.MAV_CMD_NAV_VTOL_LAND) == 1
        assert sum(1 for it in items if it.command == M.MAV_CMD_DO_LAND_START) == 1
        # 全部航點皆為相對高度框架，且 DO 指令不在 NAV 集合中
        assert all(it.frame == M.MAV_FRAME_GLOBAL_RELATIVE_ALT for it in nav)


def test_takeoff_and_land_altitudes(mini):
    for uid, items in mini.items.items():
        spec = mini.ctx.specs[mini.mp.plans[uid].cls]
        to = next(it for it in items if it.command == M.MAV_CMD_NAV_VTOL_TAKEOFF)
        ld = next(it for it in items if it.command == M.MAV_CMD_NAV_VTOL_LAND)
        assert to.z == pytest.approx(spec.transition_start_agl_m, abs=0.05)
        assert (to.x, to.y) == pytest.approx((mini.inputs.home.lat_deg, mini.inputs.home.lon_deg), abs=1e-7)
        assert ld.z == pytest.approx(0.0, abs=0.05)                  # 落點 home 地面（相對 0）
        assert (ld.x, ld.y) == pytest.approx((mini.inputs.home.lat_deg, mini.inputs.home.lon_deg), abs=1e-6)
        # 起飛後第一個 NAV_WAYPOINT 高度 ≥ transition_complete（指令值 H_tc,cmd）
        first_wp = next(it for it in items if it.command == M.MAV_CMD_NAV_WAYPOINT and it.seq > to.seq)
        assert first_wp.z >= spec.transition_cmd().h_tc_cmd_m - 0.5


def test_relative_altitudes_nonnegative_and_below_ceiling(mini):
    for uid, items in mini.items.items():
        z = [it.z for it in _nav(items[1:])]
        assert min(z) >= 0.0
        assert max(z) <= mini.ctx.h_ceil_m[mini.mp.plans[uid].cls] + 15.0     # 地形平坦：AGL ≈ 相對高度


def test_change_speed_after_its_nav(mini):
    """DO_CHANGE_SPEED 描述「出發腿」⇒ 置於該航點 NAV 之後（ArduPilot 抵達前一 NAV 才執行其後之 DO_），
    與酬載開關及 integration/mqtt_bridge 之順序一致。"""
    for uid, items in mini.items.items():
        spec = mini.ctx.specs[mini.mp.plans[uid].cls]
        cs = [it for it in items if it.command == M.MAV_CMD_DO_CHANGE_SPEED]
        assert cs, uid
        assert cs[0].param1 == 0.0 and cs[0].param2 == pytest.approx(spec.cruise_airspeed_mps)
        first_wp = next(it for it in items if it.command == M.MAV_CMD_NAV_WAYPOINT and it.seq > 0)
        assert cs[0].seq == first_wp.seq + 1                      # 緊接第一個固定翼航點之後
        for c in cs:
            assert items[c.seq - 1].is_nav, (uid, c.seq)          # 前一項必為 NAV（DO_ 掛在該 NAV 上）
        assert all(c.param2 >= spec.v_min_mission() - 1e-9 for c in cs)


def test_camera_trigger_pairs_balanced_rgb(mini):
    items = mini.items["B1"]
    cam = [it for it in items if it.command == M.MAV_CMD_DO_SET_CAM_TRIGG_DIST]
    on = [it for it in cam if it.param1 > 0]
    off = [it for it in cam if it.param1 == 0]
    n_strips = len(mini.mp.plans["B1"].strips)
    assert len(on) == len(off) == n_strips >= 1
    # 交替：on, off, on, off …；最後為 off；DO_LAND_START 在最後一個 off 之後
    state = False
    for it in cam:
        assert (it.param1 > 0) != state
        state = it.param1 > 0
    assert state is False
    # 酬載指令置於其所描述之「出發腿」起點 NAV 之後（ArduPilot 抵達該 NAV 時執行）；關閉早於落地
    land = next(it for it in items if it.command == M.MAV_CMD_NAV_VTOL_LAND)
    assert off[-1].seq < land.seq
    by_seq = {it.seq: it for it in items}
    for it in cam:
        prev = by_seq[it.seq - 1]
        assert prev.is_nav or prev.command == M.MAV_CMD_DO_SET_CAM_TRIGG_DIST
    # 觸發距離 = l(h)(1 − o_fwd)：B @100 m AGL → 97.5 × 0.3 = 29.25 m
    sensor = mini.ctx.sensors[("B", "default_rgb")]
    for it in on:
        assert it.param1 == pytest.approx(ap.trigger_distance_m(sensor, 100.0), rel=0.05)
    assert ap.trigger_distance_m(sensor, 100.0) == pytest.approx(29.25, abs=0.1)
    # A1（lidar）不使用相機觸發，改用伺服佔位
    assert not [it for it in mini.items["A1"] if it.command == M.MAV_CMD_DO_SET_CAM_TRIGG_DIST]


def test_lidar_servo_pairs_balanced(mini):
    items = mini.items["A1"]
    sv = [it for it in items if it.command == M.MAV_CMD_DO_SET_SERVO]
    on = [it for it in sv if it.param2 == ap.LIDAR_SERVO_PWM_ON]
    off = [it for it in sv if it.param2 == ap.LIDAR_SERVO_PWM_OFF]
    assert len(on) == len(off) == len(mini.mp.plans["A1"].strips)
    assert all(it.param1 == ap.LIDAR_SERVO_CHANNEL for it in sv)
    state = False
    for it in sv:
        assert (it.param2 == ap.LIDAR_SERVO_PWM_ON) != state
        state = it.param2 == ap.LIDAR_SERVO_PWM_ON
    assert state is False


def test_camera_trigger_disabled(mini):
    up = mini.mp.plans["B1"]
    items = ap.plan_to_mission_items(up, mini.ctx, mini.cfg, camera_trigger=False)
    assert not [it for it in items if it.command in (M.MAV_CMD_DO_SET_CAM_TRIGG_DIST, M.MAV_CMD_DO_SET_SERVO)]
    assert _nav(items[1:])[0].command == M.MAV_CMD_NAV_VTOL_TAKEOFF


def test_strip_endpoints_preserved(mini):
    """每條帶之起／訖航點（sensor 開／關前後之 NAV_WAYPOINT）與原 UAVPlan 條帶端點重合（< 1 m）。"""
    for uid in ("A1", "B1"):
        up = mini.mp.plans[uid]
        items = mini.items[uid]
        frame = mini.ctx.frame
        cmd_on = M.MAV_CMD_DO_SET_CAM_TRIGG_DIST if uid == "B1" else M.MAV_CMD_DO_SET_SERVO
        starts = []
        for it in items:
            if it.command == cmd_on and (it.param1 > 0 if uid == "B1" else it.param2 == ap.LIDAR_SERVO_PWM_ON):
                # 開啟指令緊接在條帶起點 NAV **之後**（抵達該點才開始掃描）
                prev = [j for j in items if j.seq < it.seq and j.is_nav][-1]
                starts.append(frame.to_enu(prev.x, prev.y)[0, :2])
        oper_starts = []
        prev = None
        for w in up.waypoints:
            if w.phase is Phase.OPERATE and w.sensor_on and w.strip_id != prev:
                oper_starts.append((w.x_m, w.y_m))
            prev = w.strip_id if w.phase is Phase.OPERATE else None
        assert len(starts) == len(oper_starts) == len(up.strips)
        for a, b in zip(starts, oper_starts):
            assert math.hypot(a[0] - b[0], a[1] - b[1]) < 1.0


def test_waypoint_spacing_and_accept_radius(mini):
    """相鄰 NAV_WAYPOINT 間距 ≥ r_acc（非必要點已依接受半徑丟棄）；param2 = r_acc = min(R_eff/2, 30)。"""
    for uid, items in mini.items.items():
        spec = mini.ctx.specs[mini.mp.plans[uid].cls]
        r_acc = ap.accept_radius_m(spec.r_eff(mini.ctx.wind.max_speed()))
        assert r_acc == 30.0
        wps = [it for it in items if it.command == M.MAV_CMD_NAV_WAYPOINT and it.seq > 0]
        assert all(it.param2 == pytest.approx(r_acc) for it in wps)
        xy = mini.ctx.frame.to_enu([it.x for it in wps], [it.y for it in wps])[:, :2]
        d = np.hypot(*np.diff(xy, axis=0).T)
        # 弧首頂點距弧起點 = R·tan(δ/2) ≥ R·tan(Δψ/4)：A（R 200）≥ 34.7 m > r_acc → 無小於 r_acc 之相鄰對；
        # B（R 120）≈ 25 m < r_acc → 只允許發生在 Ω 轉場之弧頂點／短弧起點旁，且數量 ≤ 2 × 轉場數
        r_eff = spec.r_eff(mini.ctx.wind.max_speed())
        dpsi = 2.0 * math.acos(r_eff / (r_eff + mini.cfg.e_max_m))
        assert d.min() >= 0.99 * min(r_acc, r_eff * math.tan(dpsi / 4)), (uid, d.min())
        close = np.where(d < r_acc * 0.99)[0]
        if r_eff * math.tan(dpsi / 2) >= r_acc:
            assert close.size == 0, (uid, d[close])
        else:
            assert close.size <= 2 * len(mini.mp.plans[uid].strips)
            pts = ap.reduce_waypoints(mini.mp.plans[uid].waypoints, spec, mini.cfg, r_acc)
            nav_pts = [p for p in pts if p.kind == "wp"]
            assert len(nav_pts) == len(wps)
            for k in close:
                assert nav_pts[k].vertex or nav_pts[k].protected or nav_pts[k + 1].vertex or nav_pts[k + 1].protected
    items = ap.plan_to_mission_items(mini.mp.plans["A1"], mini.ctx, mini.cfg, wp_radius_m=20.0)
    assert all(it.param2 == 20.0 for it in items if it.command == M.MAV_CMD_NAV_WAYPOINT and it.seq > 0)


def test_track_deviation_within_e_max(mini):
    """縮減後航點折線對原密集航點之側向偏差 ≤ e_max + 高度剖面容差引入之零偏差（原航點在折線 e_max 內）。"""
    e_max = mini.cfg.e_max_m
    for uid, up in mini.mp.plans.items():
        items = mini.items[uid]
        wps = [it for it in items if it.command in (M.MAV_CMD_NAV_WAYPOINT,) and it.seq > 0]
        xy = mini.ctx.frame.to_enu([it.x for it in wps], [it.y for it in wps])[:, :2]
        from shapely.geometry import LineString
        line = LineString(xy)
        fw = [w for w in up.waypoints if w.phase in (Phase.ENTRY, Phase.OPERATE, Phase.TRANSFER, Phase.RTB, Phase.CLIMB)]
        dev = np.array([line.distance(Point(w.x_m, w.y_m)) for w in fw])
        assert dev.max() <= e_max + 0.5, (uid, dev.max())


# ---------------------------------------------------------------------------
# 弧段離散（論文 Eq.28/29、D27）
# ---------------------------------------------------------------------------
def test_arc_vertices_geometry():
    R, e = 200.0, 3.0
    dpsi = 2.0 * math.acos(R / (R + e))
    c = (100.0, -50.0)
    p0 = np.array([c[0] + R, c[1]])                      # 幾何角 0
    p1 = np.array([c[0], c[1] - R])                       # 幾何角 −90°（順時針 90°）
    v = ap.arc_vertices(p0, p1, c, R, +1, dpsi)
    n = math.ceil((math.pi / 2) / dpsi)
    assert len(v) == n
    r = np.hypot(v[:, 0] - c[0], v[:, 1] - c[1])
    delta = (math.pi / 2) / n
    assert np.allclose(r, R / math.cos(delta / 2))
    assert (r - R).max() <= e + 1e-9                      # 外切偏差 ≤ e_max
    ang = np.unwrap(np.arctan2(v[:, 1] - c[1], v[:, 0] - c[0]))
    assert np.all(np.diff(ang) < 0)                       # 順時針
    # 逆時針（turn_dir −1）自 p1 回 p0
    v2 = ap.arc_vertices(p1, p0, c, R, -1, dpsi)
    assert len(v2) == n
    ang2 = np.unwrap(np.arctan2(v2[:, 1] - c[1], v2[:, 0] - c[0]))
    assert np.all(np.diff(ang2) > 0)
    # 掃掠角 0 → 無頂點
    assert ap.arc_vertices(p0, p0, c, R, +1, dpsi, sweep_rad=0.0).shape == (0, 2)


def _wp(x, y, z, t, phase, *, mode=Mode.CRUISE, arc=None, va=19.0, strip=None, on=False, trk=0.0):
    return Waypoint(x_m=x, y_m=y, z_m=z, lat_deg=0.0, lon_deg=0.0, alt_msl_m=z + 100.0, agl_m=z, t_s=t, mode=mode,
                    phase=phase, sensor_on=on, strip_id=strip, cell_id=None, dir=None, airspeed_mps=va,
                    ground_speed_mps=va, track_deg=trk, heading_deg=trk, energy_remaining_wh=300.0,
                    nearest_alternate="H", alt_reachable=True, lost_link_action="RTB", arc=arc)


def _synthetic_plan(with_orbit: str | None = None) -> UAVPlan:
    """B 機合成計畫：起飛 → P_tc → 半圓弧（R 120，右轉）→ 直線條帶 → 直線返航 → 降落；可插入 ORBIT。"""
    R = 120.0
    wps = [_wp(0.0, 0.0, 0.0, 0.0, Phase.TAKEOFF, mode=Mode.HOVER, va=0.0),
           _wp(0.0, 0.0, 50.0, 25.0, Phase.TRANS_OUT, mode=Mode.TRANSITION, va=16.0, trk=0.0),
           _wp(0.0, 384.0, 90.0, 45.0, Phase.ACCEL, va=19.0, trk=0.0)]
    t = 51.0
    p_acc = np.array([0.0, 489.0])
    wps.append(_wp(p_acc[0], p_acc[1], 90.0, t, Phase.CLIMB, arc=(R, 489.0, R, +1), trk=0.0))
    # 右轉半圓：θ 0 → π，圓心 (R, 489)；取樣 5°
    c = np.array([R, 489.0])
    for k in range(1, 36):
        th = math.radians(5.0 * k)
        p = c - R * np.array([math.cos(th), -math.sin(th)])
        t += R * math.radians(5.0) / 19.0
        arc = (R, 489.0, R, +1) if k < 36 else None
        wps.append(_wp(p[0], p[1], 90.0 + 10.0 * k / 36.0, t, Phase.ENTRY, arc=arc, trk=math.degrees(th)))
    p_end = c - R * np.array([math.cos(math.pi), 0.0])          # (2R, 489)
    wps.append(_wp(p_end[0], p_end[1], 100.0, t, Phase.ENTRY, trk=180.0))
    if with_orbit == "delay":
        wps.append(_wp(p_end[0], p_end[1], 100.0, t, Phase.ORBIT, arc=(p_end[0] + R, p_end[1], R, +1), trk=180.0))
        t += 3 * 2 * math.pi * R / 19.0
        wps.append(_wp(p_end[0], p_end[1], 100.0, t, Phase.ENTRY, trk=180.0))
    if with_orbit == "climb":
        # 兩圈螺旋爬升 100 → 140，圓心 (p_end.x + R, p_end.y)，順時針
        cc = np.array([p_end[0] + R, p_end[1]])
        for k in range(1, 73):
            th = math.pi + math.radians(5.0 * k)
            p = cc + R * np.array([math.cos(th), math.sin(th)])
            t += R * math.radians(5.0) / 19.0
            wps.append(_wp(p[0], p[1], 100.0 + 40.0 * k / 72.0, t, Phase.ORBIT, arc=(cc[0], cc[1], R, -1), trk=0.0))
        wps.append(_wp(p_end[0], p_end[1], 140.0, t, Phase.ENTRY, trk=180.0))
    z_op = wps[-1].z_m
    # 條帶：向南 1000 m，每 25 m 取樣
    for k in range(0, 41):
        y = p_end[1] - 25.0 * k
        t += 25.0 / 19.0
        wps.append(_wp(p_end[0], y, z_op, t, Phase.OPERATE, strip="S1", on=True, trk=180.0))
    y_end = p_end[1] - 1000.0
    for k in range(1, 21):
        t += 25.0 / 19.0
        wps.append(_wp(p_end[0], y_end - 25.0 * k, z_op, t, Phase.RTB, trk=180.0))
    wps.append(_wp(p_end[0], y_end - 500.0, 90.0, t + 5, Phase.DECEL, trk=180.0))
    wps.append(_wp(p_end[0], y_end - 600.0, 90.0, t + 11, Phase.TRANS_IN, mode=Mode.TRANSITION, va=16.0, trk=180.0))
    wps.append(_wp(p_end[0], y_end - 984.0, 50.0, t + 36, Phase.LAND, mode=Mode.HOVER, va=0.0))
    wps.append(_wp(p_end[0], y_end - 984.0, 0.0, t + 96, Phase.LAND, mode=Mode.HOVER, va=0.0))
    return UAVPlan("B1", "B", wps, [("S1", -1)], 0.0, t + 11, t + 111, "H", 100.0, 50.0, 3000.0, 3.0, 0, [])


def test_synthetic_arc_reduction_and_orbits(mini):
    ctx, cfg = mini.ctx, mini.cfg
    spec = ctx.specs["B"]
    dpsi = math.degrees(2.0 * math.acos(120.0 / 123.0))
    plan = _synthetic_plan()
    items = ap.plan_to_mission_items(plan, ctx, cfg, camera_trigger=True)
    nav = _nav(items[1:])
    assert nav[0].command == M.MAV_CMD_NAV_VTOL_TAKEOFF and nav[-1].command == M.MAV_CMD_NAV_VTOL_LAND
    # 半圓弧 180° → ⌈180/25.4⌉ = 8 個外切頂點（+ 弧起點、弧終點）
    n_arc = math.ceil(180.0 / dpsi)
    assert n_arc == 8
    wps = [it for it in items if it.command == M.MAV_CMD_NAV_WAYPOINT and it.seq > 0]
    xy = ctx.frame.to_enu([it.x for it in wps], [it.y for it in wps])[:, :2]
    c = np.array([120.0, 489.0])
    r = np.hypot(xy[:, 0] - c[0], xy[:, 1] - c[1])
    on_arc = np.abs(r - 120.0 / math.cos(math.radians(180.0 / n_arc) / 2)) < 0.05
    assert on_arc.sum() == n_arc
    assert (r[on_arc] - 120.0).max() <= cfg.e_max_m + 1e-6
    # 直線條帶：只留起／訖點（41 個取樣 → 2）；伺服／相機成對
    cam = [it for it in items if it.command == M.MAV_CMD_DO_SET_CAM_TRIGG_DIST]
    assert [c.param1 > 0 for c in cam] == [True, False]
    # 落點 = 最後 LAND 航點位置、相對高度 0；DO_LAND_START 出現於 RTB 首點前
    ld = nav[-1]
    assert ld.z == pytest.approx(0.0, abs=0.05)
    land_start = next(it for it in items if it.command == M.MAV_CMD_DO_LAND_START)
    assert land_start.seq > cam[0].seq and cam[-1].seq < ld.seq
    # 延遲盤旋 → NAV_LOITER_TIME（param3 = +R 順時針），時長 = 3 圈
    items_d = ap.plan_to_mission_items(_synthetic_plan("delay"), ctx, cfg)
    lt = [it for it in items_d if it.command == M.MAV_CMD_NAV_LOITER_TIME]
    assert len(lt) == 1 and lt[0].param3 == pytest.approx(120.0)
    assert lt[0].param1 == pytest.approx(3 * 2 * math.pi * 120.0 / 19.0, rel=1e-6)
    cxy = ctx.frame.to_enu(lt[0].x, lt[0].y)[0, :2]
    assert cxy == pytest.approx((360.0, 489.0), abs=0.5)
    # 螺旋爬升兩圈（逆時針）→ NAV_LOITER_TO_ALT，param2 = −R，alt = 目標高度
    items_c = ap.plan_to_mission_items(_synthetic_plan("climb"), ctx, cfg)
    la = [it for it in items_c if it.command == M.MAV_CMD_NAV_LOITER_TO_ALT]
    assert len(la) == 1 and la[0].param2 == pytest.approx(-120.0)
    assert la[0].z == pytest.approx(140.0, abs=0.6)
    assert not [it for it in items_c if it.command == M.MAV_CMD_NAV_LOITER_TURNS]
    assert spec.v_min_mission() <= 19.0


# ---------------------------------------------------------------------------
# QGC WPL 110 I/O
# ---------------------------------------------------------------------------
def test_wpl_roundtrip_and_pymavlink_loader(mini, tmp_path):
    for uid, items in mini.items.items():
        path = tmp_path / f"{uid}.waypoints"
        ap.write_qgc_wpl(items, path)
        text = path.read_text(encoding="utf-8").splitlines()
        assert text[0] == "QGC WPL 110"
        assert all(len(line.split("\t")) == 12 for line in text[1:])
        lat_str = text[1].split("\t")[8]
        assert len(lat_str.split(".")[1]) == 7
        back = ap.read_qgc_wpl(path)
        assert len(back) == len(items)
        for a, b in zip(items, back):
            assert (a.seq, a.frame, a.command, a.current, a.autocontinue) == (b.seq, b.frame, b.command, b.current,
                                                                              b.autocontinue)
            assert (a.param1, a.param2, a.param3, a.param4) == pytest.approx((b.param1, b.param2, b.param3, b.param4),
                                                                             abs=1e-7)
            assert (a.x, a.y) == pytest.approx((b.x, b.y), abs=6e-8)
            assert a.z == pytest.approx(b.z, abs=1e-5)
        loader = mavwp.MAVWPLoader()
        n = loader.load(str(path))
        assert n == len(items)
        w1 = loader.wp(1)
        assert w1.command == M.MAV_CMD_NAV_VTOL_TAKEOFF and w1.frame == M.MAV_FRAME_GLOBAL_RELATIVE_ALT
        assert loader.wp(n - 1).command == M.MAV_CMD_NAV_VTOL_LAND
        assert loader.wp(0).current == 1 and loader.wp(0).frame == 0


def test_alt_frames(mini):
    up = mini.mp.plans["A1"]
    rel = ap.plan_to_mission_items(up, mini.ctx, mini.cfg, alt_frame="relative")
    msl = ap.plan_to_mission_items(up, mini.ctx, mini.cfg, alt_frame="msl")
    ter = ap.plan_to_mission_items(up, mini.ctx, mini.cfg, alt_frame="terrain")
    assert len(rel) == len(msl) == len(ter)
    for a, b, c in zip(_nav(rel[1:]), _nav(msl[1:]), _nav(ter[1:])):
        assert b.frame == M.MAV_FRAME_GLOBAL and c.frame == M.MAV_FRAME_GLOBAL_TERRAIN_ALT
        assert b.z - a.z == pytest.approx(100.0, abs=1e-3)                  # MSL − relative = home MSL
        if a.command == M.MAV_CMD_NAV_WAYPOINT:
            assert c.z == pytest.approx(a.z, abs=1.5)                       # 平地：AGL ≈ 相對高度
    with pytest.raises(ValueError):
        ap.plan_to_mission_items(up, mini.ctx, mini.cfg, alt_frame="agl")   # type: ignore[arg-type]


# ---------------------------------------------------------------------------
# rally / fence / 參數 / .plan / export_all
# ---------------------------------------------------------------------------
def test_rally_items(mini):
    r = ap.rally_items(mini.inputs, mini.ctx)
    assert [it.command for it in r] == [M.MAV_CMD_NAV_RALLY_POINT] * 2
    assert all(it.mission_type == M.MAV_MISSION_TYPE_RALLY and it.frame == M.MAV_FRAME_GLOBAL_RELATIVE_ALT for it in r)
    assert (r[0].x, r[0].y) == (mini.inputs.home.lat_deg, mini.inputs.home.lon_deg)
    alt1 = mini.inputs.alternates[0]
    assert (r[1].x, r[1].y) == (alt1.lat_deg, alt1.lon_deg)
    h_tc = max(sp.transition_cmd().h_tc_cmd_m for sp in mini.ctx.specs.values())
    assert r[0].z == pytest.approx(h_tc, abs=0.05)
    assert r[1].z == pytest.approx(h_tc, abs=1.0)


def test_fence_items_mini(mini):
    for cls in ("A", "B"):
        f = ap.fence_items(mini.inputs, mini.ctx, cls)
        assert f[0].command == M.MAV_CMD_NAV_FENCE_RETURN_POINT
        assert (f[0].x, f[0].y) == (mini.inputs.home.lat_deg, mini.inputs.home.lon_deg)
        incl = [it for it in f if it.command == M.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION]
        assert len(incl) >= 4 and all(it.param1 == len(incl) for it in incl)
        assert all(it.mission_type == M.MAV_MISSION_TYPE_FENCE for it in f)
        assert not [it for it in f if it.command == M.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION]
        xy = mini.ctx.frame.to_enu([it.x for it in incl], [it.y for it in incl])[:, :2]
        poly = Polygon(xy)
        assert poly.is_valid
        # 含全部區域 ⊕ 2R_eff 且在鏈路圓內
        r_eff = mini.ctx.specs[cls].r_eff(mini.ctx.wind.max_speed())
        assert ap.fence_buffer_m(mini.ctx, cls) >= 2 * r_eff
        for area in mini.ctx.areas_enu.values():
            assert poly.contains(area.buffer(ap.fence_buffer_m(mini.ctx, cls) - 6.0))
        assert poly.contains(Point(0.0, 0.0))
        assert poly.within(mini.ctx.link_poly[cls].buffer(10.0))
        # 航線在 fence 內
        wps = [it for it in mini.items[f"{cls}1"] if it.is_nav and it.seq > 0]
        pts = mini.ctx.frame.to_enu([it.x for it in wps], [it.y for it in wps])[:, :2]
        assert all(poly.covers(Point(p)) for p in pts)


def test_fence_exclusion_hills():
    inputs, cfg = build_hills_scenario()
    cfg = dataclasses.replace(cfg, dem_step_m=50.0)
    ctx = build_context(inputs, cfg, terrain_margin_m=800.0)
    f = ap.fence_items(inputs, ctx, "B")
    excl = [it for it in f if it.command == M.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION]
    assert len(excl) == 4 and all(it.param1 == 4 for it in excl)
    xy = ctx.frame.to_enu([it.x for it in excl], [it.y for it in excl])[:, :2]
    poly = Polygon(xy)
    assert poly.bounds == pytest.approx((9000.0, -1400.0, 11000.0, -400.0), abs=1.0)
    incl = [it for it in f if it.command == M.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION]
    assert incl and f[0].command == M.MAV_CMD_NAV_FENCE_RETURN_POINT
    plan = ap.to_qgc_plan_json(mini_items := [ap._item(0, M.MAV_CMD_NAV_WAYPOINT, 0, x=24.1, y=120.75, z=100.0, current=1)],
                               ap.rally_items(inputs, ctx), f)
    assert len(mini_items) == 1
    polys = plan["geoFence"]["polygons"]
    assert [p["inclusion"] for p in polys] == [True, False]
    assert len(polys[1]["polygon"]) == 4


def test_recommended_params_and_file(mini, tmp_path):
    ctx, cfg = mini.ctx, mini.cfg
    spec, model = ctx.specs["A"], ctx.models["A1"]
    p = ap.recommended_params(spec, model, cfg, mini.mp.plans["A1"], pack_voltage_v=44.4, wind_speed_mps=5.0,
                              h_ceil_agl_m=200.0, lost_link_policy="CONTINUE_THEN_RTB")
    for k in ("AIRSPEED_CRUISE", "AIRSPEED_MIN", "AIRSPEED_MAX", "AIRSPEED_STALL", "TECS_CLMB_MAX", "TECS_SINK_MAX",
              "TECS_SINK_MIN", "ROLL_LIMIT_DEG", "WP_LOITER_RAD", "WP_RADIUS", "RTL_RADIUS", "Q_ENABLE",
              "Q_TRANSITION_MS", "Q_ASSIST_SPEED", "Q_RTL_ALT", "Q_LAND_FINAL_ALT", "FENCE_ENABLE", "FENCE_TYPE",
              "FENCE_ALT_MAX", "FENCE_ACTION", "BATT_CAPACITY", "BATT_LOW_MAH", "BATT_CRT_MAH", "FS_LONG_ACTN",
              "TERRAIN_ENABLE"):
        assert k in p and k in ap.PARAM_COMMENTS, k
    assert p["AIRSPEED_CRUISE"] == 21.5 and p["AIRSPEED_MIN"] == 18.0 and p["AIRSPEED_MAX"] == 35.0
    assert p["AIRSPEED_STALL"] == 14.0 and p["ROLL_LIMIT_DEG"] == 20.0
    assert p["TECS_CLMB_MAX"] == pytest.approx(1.13) and p["TECS_SINK_MAX"] == pytest.approx(1.5)
    assert p["WP_LOITER_RAD"] == pytest.approx(spec.r_eff(5.0), abs=0.1) == 200.0
    assert p["WP_RADIUS"] == 30.0 and p["Q_ENABLE"] == 1 and p["Q_TRANSITION_MS"] == 20000
    assert p["Q_ASSIST_SPEED"] > spec.stall_airspeed_mps
    assert p["FENCE_ALT_MAX"] >= 200.0 and p["FENCE_TYPE"] == 5
    assert p["FS_LONG_ACTN"] == 0 and p["TERRAIN_ENABLE"] == 0
    assert p["BATT_CAPACITY"] == pytest.approx(1500.0 / 44.4 * 1000.0, abs=1.0)
    assert p["BATT_CRT_MAH"] == pytest.approx(450.0 / 44.4 * 1000.0, abs=1.0)
    assert p["BATT_LOW_MAH"] == pytest.approx((450.0 + model.e_landing_wh()) / 44.4 * 1000.0, abs=1.0)
    assert p["BATT_LOW_MAH"] > p["BATT_CRT_MAH"]
    # 電壓未知 → 無 mAh 參數；RTB 政策 → RTL；terrain 框架 → TERRAIN_ENABLE
    q = ap.recommended_params(ctx.specs["B"], ctx.models["B1"], cfg, lost_link_policy="RTB", alt_frame="terrain")
    assert "BATT_CAPACITY" not in q and q["FS_LONG_ACTN"] == 1 and q["TERRAIN_ENABLE"] == 1
    assert q["AIRSPEED_MAX"] == pytest.approx(1.4 * 19.0, abs=0.1)     # HERO max_airspeed None → 暫定
    assert q["AIRSPEED_MIN"] == 16.0 and q["WP_LOITER_RAD"] == 120.0
    # 檔案格式：NAME,VALUE；註解以 # 開頭；可讀回
    path = tmp_path / "A1.param"
    ap.write_param_file(p, path)
    lines = path.read_text(encoding="utf-8").splitlines()
    data = [ln for ln in lines if not ln.startswith("#")]
    assert len(data) == len(p)
    for ln in data:
        name, val = ln.split(",")
        assert name in p
        float(val)
    back = ap.read_param_file(path)
    assert back == pytest.approx(p)
    assert any(ln.startswith("# AIRSPEED_CRUISE:") for ln in lines)


def test_qgc_plan_json(mini, tmp_path):
    items = mini.items["B1"]
    rally = ap.rally_items(mini.inputs, mini.ctx)
    fence = ap.fence_items(mini.inputs, mini.ctx, "B")
    path = tmp_path / "B1.plan"
    plan = ap.to_qgc_plan_json(items, rally, fence, path)
    loaded = json.loads(path.read_text(encoding="utf-8"))
    assert loaded == plan
    assert plan["fileType"] == "Plan" and plan["version"] == 1
    m = plan["mission"]
    assert m["vehicleType"] == 2 and m["firmwareType"] == 3
    assert m["plannedHomePosition"] == pytest.approx([items[0].x, items[0].y, items[0].z])
    assert len(m["items"]) == len(items) - 1
    assert m["items"][0]["command"] == M.MAV_CMD_NAV_VTOL_TAKEOFF and m["items"][-1]["command"] == M.MAV_CMD_NAV_VTOL_LAND
    assert all(len(it["params"]) == 7 and it["type"] == "SimpleItem" for it in m["items"])
    assert m["cruiseSpeed"] == pytest.approx(19.0)
    assert len(plan["rallyPoints"]["points"]) == 2
    polys = plan["geoFence"]["polygons"]
    assert len(polys) == 1 and polys[0]["inclusion"] is True
    assert len(polys[0]["polygon"]) == sum(1 for it in fence if it.command == 5001)
    with pytest.raises(ValueError):
        ap.to_qgc_plan_json([], rally, fence)


def test_export_all(exported, mini):
    out, s = exported.out, exported.summary
    names = sorted(os.listdir(out))
    for uid in ("A1", "B1"):
        for suffix in (".waypoints", "_rally.txt", "_fence.txt", ".param", ".plan"):
            assert f"{uid}{suffix}" in names
        loader = mavwp.MAVWPLoader()
        n = loader.load(str(out / f"{uid}.waypoints"))
        assert n == len(mini.items[uid]) == s["uavs"][uid]["n_items"]
        back = ap.read_qgc_wpl(out / f"{uid}_rally.txt")
        assert [it.command for it in back] == [M.MAV_CMD_NAV_RALLY_POINT] * 2
        fence = ap.read_qgc_wpl(out / f"{uid}_fence.txt")
        assert fence[0].command == M.MAV_CMD_NAV_FENCE_RETURN_POINT
        params = ap.read_param_file(out / f"{uid}.param")
        assert params["Q_ENABLE"] == 1 and "AIRSPEED_CRUISE" in params
        assert "BATT_CAPACITY" not in params            # mini SortieEnergy 無 pack_voltage_nom_v
        json.loads((out / f"{uid}.plan").read_text(encoding="utf-8"))
    summ = json.loads((out / "mission_summary.json").read_text(encoding="utf-8"))
    assert set(summ["uavs"]) == {"A1", "B1"}
    assert summ["makespan_s"] == pytest.approx(mini.mp.makespan_s)
    assert summ["home"]["alt_msl_m"] == pytest.approx(100.0, abs=1e-3)
    assert summ["uavs"]["A1"]["landing_site"] == "H" and summ["uavs"]["A1"]["n_nav"] < summ["uavs"]["A1"]["n_items"]
    assert summ["uavs"]["B1"]["params"]["AIRSPEED_CRUISE"] == 19.0


def test_mission_item_helpers(mini):
    it = mini.items["A1"][3]
    kw = it.as_mission_item_int_kwargs(1, 1)
    assert kw["x"] == int(round(it.x * 1e7)) and kw["mission_type"] == 0 and kw["seq"] == 3
    d = it.to_dict()
    assert d["command"] == it.command and set(d) >= {"seq", "frame", "command", "x", "y", "z"}
    items = copy.deepcopy(mini.items["A1"])
    items[5].seq = 99
    ap.renumber(items)
    assert [i.seq for i in items] == list(range(len(items)))
    empty = UAVPlan("A1", "A", [], [], 0.0, 0.0, 0.0, "H", 0.0, 0.0, 0.0, 0.0, 0, [])
    assert len(ap.plan_to_mission_items(empty, mini.ctx, mini.cfg)) == 1


# ---------------------------------------------------------------------------
# 審查回歸：ArduPlane 參數列舉、酬載參數、圍籬高度框架、WP_RADIUS、盤旋半徑、起飛時間原點
# ---------------------------------------------------------------------------
def test_batt_failsafe_actions_use_arduplane_enum(mini):
    """ArduPlane 之 BATT_FS_*_ACT 列舉是 0:None 1:RTL 2:Land 3:Terminate 4:QLand 6:Loiter-to-QLand
    （**不是** Copter 的 2=RTL/3=LAND）；QuadPlane 建議 RTL / QLand。3 = 飛行終止，絕不可出現。"""
    ctx, cfg = mini.ctx, mini.cfg
    for cls, uid in (("A", "A1"), ("B", "B1")):
        p = ap.recommended_params(ctx.specs[cls], ctx.models[uid], cfg, mini.mp.plans[uid])
        assert p["BATT_FS_LOW_ACT"] == 1.0                      # RTL
        assert p["BATT_FS_CRT_ACT"] == 4.0                      # QLand
        assert "Terminate" in ap.PARAM_COMMENTS["BATT_FS_CRT_ACT"]
        # 動作有輸出但沒有門檻時必須明講（mini 無 pack_voltage_nom_v）
        w = []
        ap.recommended_params(ctx.specs[cls], ctx.models[uid], cfg, mini.mp.plans[uid], warnings=w)
        assert any("BATT_LOW_MAH" in s for s in w)
    # 恢復參數（RTL / QRTL / 集結點）必須存在，否則 FENCE_ACTION=1、FS_LONG_ACTN=1 與 DO_LAND_START 都到不了
    p = ap.recommended_params(ctx.specs["A"], ctx.models["A1"], cfg, mini.mp.plans["A1"])
    assert p["Q_RTL_MODE"] == 1.0 and p["RTL_AUTOLAND"] == 2.0 and p["FENCE_RET_RALLY"] == 1.0


def test_q_assist_speed_below_airspeed_min(mini):
    """Q_ASSIST_SPEED 必須 < AIRSPEED_MIN（等於下限時 TECS 每次降到下限都會啟動升力馬達）且 > 失速。"""
    for cls, uid in (("A", "A1"), ("B", "B1")):
        p = ap.recommended_params(mini.ctx.specs[cls], mini.ctx.models[uid], mini.cfg)
        assert p["AIRSPEED_STALL"] < p["Q_ASSIST_SPEED"] < p["AIRSPEED_MIN"], (cls, p["Q_ASSIST_SPEED"])


def test_payload_params_enable_the_do_commands(exported, mini):
    """任務中每一類 DO_ 酬載指令都必須有對應的啟用參數，否則上機後是空指令。"""
    for uid in ("A1", "B1"):
        items = mini.items[uid]
        params = ap.read_param_file(exported.out / f"{uid}.param")
        if any(it.command == M.MAV_CMD_DO_SET_CAM_TRIGG_DIST for it in items):
            assert params.get("CAM1_TYPE") == 1.0                                  # servo 相機後端（4.5+）
            assert params.get(f"SERVO{ap.CAMERA_SERVO_CHANNEL}_FUNCTION") == 10.0   # CameraTrigger
        if any(it.command == M.MAV_CMD_DO_SET_SERVO for it in items):
            # 0 = Disabled ⇒ 該輸出開放給任務 DO_SET_SERVO（否則韌體回 Channel N is already in use）
            assert params.get(f"SERVO{ap.LIDAR_SERVO_CHANNEL}_FUNCTION") == 0.0


def test_fence_alt_max_is_above_home_not_agl(mini):
    """AC_Fence 比對的是「相對 home 高度」；地形高於起降場時 AGL 會低估。"""
    ctx, cfg = mini.ctx, mini.cfg
    up = mini.mp.plans["A1"]
    home_msl = 100.0
    rel_max = max(w.alt_msl_m for w in up.waypoints) - home_msl
    p = ap.recommended_params(ctx.specs["A"], None, cfg, up, home_msl_m=home_msl)
    assert p["FENCE_ALT_MAX"] >= rel_max + cfg.b_v_m - 1.0
    # 地形比起降場高 200 m：以 AGL 計會低 200 m
    hi = dataclasses.replace(up, waypoints=[dataclasses.replace(w, alt_msl_m=w.alt_msl_m + 200.0) for w in up.waypoints])
    p_hi = ap.recommended_params(ctx.specs["A"], None, cfg, hi, home_msl_m=home_msl)
    assert p_hi["FENCE_ALT_MAX"] >= p["FENCE_ALT_MAX"] + 199.0
    w = []
    ap.recommended_params(ctx.specs["A"], None, cfg, hi, warnings=w)     # 無 home_msl → 退回 AGL 並警告
    assert any("FENCE_ALT_MAX" in s for s in w)


def test_export_fence_alt_max_covers_every_item(exported, mini):
    for uid in ("A1", "B1"):
        params = ap.read_param_file(exported.out / f"{uid}.param")
        z = [it.z for it in ap.read_qgc_wpl(exported.out / f"{uid}.waypoints") if it.is_nav and it.seq > 0]
        assert params["FENCE_ALT_MAX"] >= max(z), uid


def test_wp_radius_below_min_nav_spacing(exported, mini):
    """ArduPlane 對所有航點都用 WP_RADIUS（NAV_WAYPOINT param2 被忽略）⇒ WP_RADIUS 必須小於最小相鄰間距，
    否則後點與前點同一週期被判抵達，掛其後之 DO_ 指令會提早觸發。"""
    for uid in ("A1", "B1"):
        d_min = ap.min_nav_spacing_m(mini.items[uid], mini.ctx)
        params = ap.read_param_file(exported.out / f"{uid}.param")
        assert d_min is not None and params["WP_RADIUS"] < d_min, (uid, params["WP_RADIUS"], d_min)
        assert params["WP_RADIUS"] >= 10.0


def test_loiter_radius_agrees_with_wp_loiter_rad(mini):
    """ArduPlane 對 NAV_LOITER_TIME 只讀 param3 的正負號（方向），半徑一律用 WP_LOITER_RAD。"""
    for uid in ("A1", "B1"):
        items = mini.items[uid]
        r = ap.orbit_radius_m(items)
        cls = mini.mp.plans[uid].cls
        p = ap.recommended_params(mini.ctx.specs[cls], None, mini.cfg, loiter_radius_m=r)
        if r is not None:
            assert p["WP_LOITER_RAD"] == pytest.approx(r, abs=0.05)
        else:
            assert p["WP_LOITER_RAD"] == pytest.approx(mini.ctx.specs[cls].r_eff(0.0), abs=0.1)
    # NAV_LOITER_TURNS 之 param3 進到 ArduPlane 的 p1 高位元組 → 整數且 ≤ 255 m
    assert ap._loiter_turns_radius(-228.4) == -228.0
    assert ap._loiter_turns_radius(400.0) == float(ap.LOITER_TURNS_MAX_RADIUS_M)


def test_takeoff_time_origin(mini, tmp_path):
    """匯出之任務必須帶著錯開起飛的資訊：nav_delay 內含 NAV_DELAY(93)，operator 則寫在檔頭與 summary。"""
    spread = max(u.t_takeoff_s for u in mini.mp.plans.values()) - min(u.t_takeoff_s for u in mini.mp.plans.values())
    assert spread > 0.0                                     # mini 之 A1/B1 起飛差 60 s
    out_a = tmp_path / "nav_delay"
    s_a = ap.export_all(mini.mp, mini.ctx, mini.cfg, out_a, takeoff_delay="nav_delay")
    assert s_a["takeoff"]["mode"] == "nav_delay" and s_a["takeoff"]["spread_s"] == pytest.approx(spread)
    for uid, u in s_a["uavs"].items():
        items = ap.read_qgc_wpl(out_a / f"{uid}.waypoints")
        delay = [it for it in items if it.command == M.MAV_CMD_NAV_DELAY]
        if u["takeoff_delay_s"] > 0:
            assert len(delay) == 1 and delay[0].seq == 1
            assert delay[0].param1 == pytest.approx(u["takeoff_delay_s"], abs=0.05)
            assert items[2].command == M.MAV_CMD_NAV_VTOL_TAKEOFF
        else:
            assert not delay
        mavwp.MAVWPLoader().load(str(out_a / f"{uid}.waypoints"))          # pymavlink 仍讀得動
    out_b = tmp_path / "operator"
    s_b = ap.export_all(mini.mp, mini.ctx, mini.cfg, out_b, takeoff_delay="operator")
    assert s_b["takeoff"]["mode"] == "operator"
    for uid, u in s_b["uavs"].items():
        txt = (out_b / f"{uid}.waypoints").read_text(encoding="utf-8")
        assert txt.splitlines()[0] == ap.QGC_WPL_HEADER
        assert f"START THIS MISSION AT t0 + {u['takeoff_delay_s']:.1f} s" in txt
        assert not [it for it in ap.read_qgc_wpl(out_b / f"{uid}.waypoints") if it.command == M.MAV_CMD_NAV_DELAY]
        n = mavwp.MAVWPLoader().load(str(out_b / f"{uid}.waypoints"))       # 註解須為 ASCII，否則 mavwp 讀不動
        assert n == u["n_items"]
    with pytest.raises(ValueError):
        ap.export_all(mini.mp, mini.ctx, mini.cfg, tmp_path / "bad", takeoff_delay="whenever")  # type: ignore[arg-type]


def test_rally_altitude_is_always_above_home(mini):
    """ArduPilot 的 RallyLocation 沒有 MSL 框架（高度是相對 home 的 int16）⇒ msl 亦輸出相對高度。"""
    home_msl = 100.0
    h_tc = max(sp.transition_cmd().h_tc_cmd_m for sp in mini.ctx.specs.values())
    for frame in ("relative", "msl"):
        r = ap.rally_items(mini.inputs, mini.ctx, frame)
        assert all(it.frame == M.MAV_FRAME_GLOBAL_RELATIVE_ALT for it in r), frame
        assert r[0].z == pytest.approx(h_tc, abs=0.05), frame
        assert r[0].z < home_msl + h_tc - 1.0                     # 不是 MSL
    t = ap.rally_items(mini.inputs, mini.ctx, "terrain")
    assert all(it.frame == M.MAV_FRAME_GLOBAL_TERRAIN_ALT for it in t)
    assert t[0].z == pytest.approx(h_tc, abs=0.05)


def test_qgc_altitude_mode_terrain(mini):
    """QGC(>=4.3) AltitudeMode：3 = CalcAboveTerrain（由地形算 AMSL）、4 = TerrainFrame。"""
    items = ap.plan_to_mission_items(mini.mp.plans["A1"], mini.ctx, mini.cfg, alt_frame="terrain")
    d = ap.to_qgc_plan_json(items, None, None)
    modes = {it["AltitudeMode"] for it in d["mission"]["items"] if it["frame"] == M.MAV_FRAME_GLOBAL_TERRAIN_ALT}
    assert modes == {4}
    rel = ap.to_qgc_plan_json(mini.items["A1"], None, None)
    assert {it["AltitudeMode"] for it in rel["mission"]["items"]
            if it["frame"] == M.MAV_FRAME_GLOBAL_RELATIVE_ALT} == {1}
