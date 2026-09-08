"""tests/test_mavlink_link.py — `hv_dccpp.mavlink_link`（設計文件 §15.5）。

離線：任務項目正規化／QGC WPL 讀寫／seqmap（§15.5.2）／後備轉換／遙測換算／事件規則。
線上：以 `tools/mock_autopilot.py` 子程序（空閒 UDP 埠、`--time-scale 60 --duration 40 --quiet`）跑完整協定：
心跳 → 上傳 mini A1 任務 + rally + fence（ACK=0、下載逐項相等）→ PARAM_SET → MISSION_START → 遙測直到 vtol_state=FW
→ `to_planner_telemetry` → `FleetLink.replan_and_reupload`（MISSION_SET_CURRENT）。線上測試約 8–12 s（協定往返受 mock
20 ms 迴圈限制，無法 < 5 s）。pymavlink 缺席時整檔 skip。
"""
from __future__ import annotations

import dataclasses
import math
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("pymavlink")

from pymavlink import mavutil  # noqa: E402

from hv_dccpp import dynamic as dyn  # noqa: E402
from hv_dccpp import mavlink_link as ml  # noqa: E402
from hv_dccpp.context import build_context  # noqa: E402
from hv_dccpp.mission import Phase  # noqa: E402
from hv_dccpp.planner import plan_mission  # noqa: E402
from hv_dccpp.scenario import build_mini_example  # noqa: E402

mav = mavutil.mavlink
def _repo_root(start: Path) -> Path:
    """往上找到含 ``tools/mock_autopilot.py`` 的那一層。

    ⚠ vendored 修改（見 docs/hv_dccpp/VENDOR.md）：上游這一行是
    ``parents[1]``，因為那邊測試就放在 repo 根的 tests/ 底下。搬進 AeroPlan 後
    測試在 tests/hv_dccpp/，parents[1] 會指到 tests/ ⇒ 找不到檔案就**整條 skip**，
    而 skip 在報表上跟通過長得一樣。改成往上找，兩種佈局都對。
    """
    for p in start.parents:
        if (p / "tools" / "mock_autopilot.py").exists():
            return p
    return start.parents[1]


ROOT = _repo_root(Path(__file__).resolve())
MOCK = ROOT / "tools" / "mock_autopilot.py"


# ---------------------------------------------------------------------------
# fixtures
# ---------------------------------------------------------------------------
@pytest.fixture(scope="module")
def env():
    inputs, cfg = build_mini_example()
    cfg = dataclasses.replace(cfg, dem_step_m=50.0)
    ctx = build_context(inputs, cfg, terrain_margin_m=1500.0)
    plan = plan_mission(inputs, cfg, ctx=ctx)
    assert plan.feasibility.status != "NO_GO" and "A1" in plan.plans
    a1 = plan.plans["A1"]
    items = ml.plan_to_mission_items(a1, ctx, cfg)
    return {"inputs": inputs, "cfg": cfg, "ctx": ctx, "plan": plan, "a1": a1, "items": items}


def _mid_strip_tel(env, frac: float = 0.5, **kw) -> tuple[ml.VehicleTelemetry, str, int]:
    """A1 第一條帶中點之 VehicleTelemetry（mission_seq = 該條帶終點項目）。"""
    a1, ctx, items = env["a1"], env["ctx"], env["items"]
    sid = a1.strips[0][0]
    sm = items.seqmap
    seq_end = sm.strip_end_seq(sid)
    spans = ml._oper_spans(a1)
    p0, p1, d, i0, i_end = spans[sid]
    p = p0 + frac * (p1 - p0)
    z = a1.waypoints[i0].z_m
    ll = ctx.frame.to_wgs84(np.array([[p[0], p[1], z]]))[0]
    t = a1.waypoints[i0].t_s + frac * (a1.waypoints[i_end].t_s - a1.waypoints[i0].t_s)
    base = dict(t_s=float(t), lat_deg=float(ll[0]), lon_deg=float(ll[1]), alt_msl_m=float(ll[2]), rel_alt_m=160.0,
                heading_deg=float(a1.waypoints[i0].track_deg), groundspeed_mps=22.0, airspeed_mps=21.5,
                battery_pct=None, battery_wh_remaining=None, wind_speed_mps=None, wind_dir_from_deg=None,
                vtol_state=int(mav.MAV_VTOL_STATE_FW), landed_state=int(mav.MAV_LANDED_STATE_IN_AIR),
                mission_seq=int(seq_end), mode="AUTO", armed=True, link_ok=True,
                track_deg=float(a1.waypoints[i0].track_deg))
    base.update(kw)
    return ml.VehicleTelemetry(**base), sid, seq_end


# ---------------------------------------------------------------------------
# 任務項目 / QGC WPL
# ---------------------------------------------------------------------------
def test_as_mission_item_variants():
    d = {"seq": 3, "frame": 3, "command": mav.MAV_CMD_NAV_WAYPOINT, "param2": 30.0, "lat_deg": 23.95, "lon_deg": 120.9,
         "alt_m": 150.0, "strip_id": "S1", "phase": Phase.OPERATE, "dir": -1, "wp_index": 7}
    it = ml.as_mission_item(d)
    assert (it.seq, it.frame, it.command, it.param2, it.x, it.y, it.z) == (3, 3, 16, 30.0, 23.95, 120.9, 150.0)
    assert it.strip_id == "S1" and it.phase == "OPERATE" and it.dir == -1 and it.wp_index == 7
    row = ml.as_mission_item(it.to_row().split("\t"))
    assert ml.items_equal([it], [row])
    nine = ml.as_mission_item((3, 16, 0, 30, 0, 0, 23.95, 120.9, 150.0), seq=5)
    assert nine.seq == 5 and nine.command == 16 and nine.z == 150.0
    # 1e7 整數座標 → 度
    it2 = ml.as_mission_item({"command": 16, "x": 239500000, "y": 1209000000, "z": 1.0})
    assert it2.x == pytest.approx(23.95) and it2.y == pytest.approx(120.9)
    # ardupilot.MissionItem（duck typing）
    ap = pytest.importorskip("hv_dccpp.ardupilot")
    api = ap.MissionItem(2, 3, 16, 0, 1, 0.0, 30.0, 0.0, 0.0, 23.95, 120.9, 100.0)
    it3 = ml.as_mission_item(api)
    assert it3.seq == 2 and it3.x == 23.95 and it3.param2 == 30.0 and it3.phase is None


def test_items_and_wpl_roundtrip(env, tmp_path):
    items = env["items"]
    assert isinstance(items, ml.MissionItems) and items.seqmap is not None
    # 第 0 列 home：frame GLOBAL、NAV_WAYPOINT、current 1；起飛 NAV_VTOL_TAKEOFF；末項 NAV_VTOL_LAND
    assert items[0].frame == mav.MAV_FRAME_GLOBAL and items[0].command == mav.MAV_CMD_NAV_WAYPOINT and items[0].current == 1
    cmds = [it.command for it in items]
    assert cmds[1] == mav.MAV_CMD_NAV_VTOL_TAKEOFF and cmds[-1] == mav.MAV_CMD_NAV_VTOL_LAND
    assert mav.MAV_CMD_DO_LAND_START in cmds and mav.MAV_CMD_DO_CHANGE_SPEED in cmds
    assert [it.seq for it in items] == list(range(len(items)))
    p = tmp_path / "A1.waypoints"
    ml.write_qgc_wpl(items, p)
    lines = p.read_text(encoding="utf-8").splitlines()
    assert lines[0] == "QGC WPL 110" and all(len(l.split("\t")) == 12 for l in lines[1:])
    back = ml.read_qgc_wpl(p)
    assert ml.items_equal(items, back)
    from pymavlink import mavwp
    try:
        assert mavwp.MAVWPLoader().load(str(p)) == len(items)
    except TypeError:
        # pymavlink 2.4.49 mavwp.load 在 MAVLINK20=1 下多傳一個引數（上游 bug），非檔案格式問題
        pass
    # 不等：改一個座標
    back[5].x += 1e-5
    assert not ml.items_equal(items, back)


# ---------------------------------------------------------------------------
# seqmap（§15.5.2）
# ---------------------------------------------------------------------------
def test_seqmap_semantics_and_roundtrip(env, tmp_path):
    a1, items, ctx = env["a1"], env["items"], env["ctx"]
    sm = items.seqmap
    assert sm.uav_id == "A1" and sm.n_items == len(items) and sm.frame == mav.MAV_FRAME_GLOBAL_RELATIVE_ALT
    assert sm.home_msl_m == pytest.approx(a1.waypoints[0].alt_msl_m - a1.waypoints[0].agl_m, abs=0.5)
    strips = [s for s, _d in a1.strips]
    for sid in strips:
        seqs = sm.strip_seqs(sid)
        assert seqs, sid
        s0, s1 = sm.strip_start_seq(sid), sm.strip_end_seq(sid)
        assert sm.items[s0].s_frac_end == pytest.approx(0.0, abs=1e-6)
        assert sm.items[s1].s_frac_end == pytest.approx(1.0, abs=1e-6)
        assert sm.items[s1].s_frac_start <= sm.items[s1].s_frac_end
        assert sm.items[s0].dir == dict(a1.strips)[sid]
        # 條帶內 → (sid, True)；條帶之後第一個 seq → (sid, False)
        assert sm.strip_at(s1) == (sid, True)
        nxt = sm.seq_after_strip(sid)
        assert nxt == s1 + 1 and sm.strip_at(nxt)[0] == sid and sm.strip_at(nxt)[1] is False
    # 條帶順序與 plan.strips 一致
    order = sorted(strips, key=lambda s: sm.strip_start_seq(s))
    assert order == strips
    # 起飛前 → 無條帶
    assert sm.strip_at(1) == (None, False)
    # DO_ 指令繼承下一 NAV 項目之 phase
    for it in items:
        if it.command in (mav.MAV_CMD_DO_CHANGE_SPEED, mav.MAV_CMD_DO_SET_CAM_TRIGG_DIST, mav.MAV_CMD_DO_SET_SERVO):
            assert sm.items[it.seq].phase is not None
    # 中繼資料已寫回項目
    assert items[sm.strip_start_seq(strips[0])].strip_id == strips[0]
    # JSON 往返
    path = sm.save(tmp_path / "A1.seqmap.json")
    sm2 = ml.MissionSeqMap.load(path)
    assert sm2.to_dict() == sm.to_dict()
    # 不帶中繼資料之項目（僅座標）→ 以計畫對應得到相同條帶對照
    bare = [ml.MissionItemInt(i, it.frame, it.command, it.current, it.autocontinue, it.param1, it.param2, it.param3,
                              it.param4, it.x, it.y, it.z) for i, it in enumerate(items)]
    sm3 = ml.mission_seq_map(bare, a1, ctx)
    for k, e in sm.items.items():
        assert sm3.items[k].strip_id == e.strip_id and sm3.items[k].phase == e.phase
    # seq_of_wp：首個對應航點索引 ≥ 條帶首 OPER 航點之 seq = 該條帶第一個 NAV 項目（DO_ 指令 wp_index=None 不算）
    k0 = ml._oper_spans(a1)[strips[0]][3]
    first_nav = min(q for q in sm.strip_seqs(strips[0]) if sm.items[q].wp_index is not None)
    assert sm.seq_of_wp(k0) == first_nav and sm.items[first_nav].wp_index == k0


def test_fallback_converter(env):
    a1, ctx, cfg = env["a1"], env["ctx"], env["cfg"]
    items = ml.plan_to_mission_items_fallback(a1, ctx, cfg)
    cmds = [it.command for it in items]
    assert cmds[0] == mav.MAV_CMD_NAV_WAYPOINT and items[0].current == 1 and items[0].frame == mav.MAV_FRAME_GLOBAL
    assert cmds[1] == mav.MAV_CMD_NAV_VTOL_TAKEOFF and items[1].z == ctx.specs["A"].transition_start_agl_m
    assert cmds[2] == mav.MAV_CMD_DO_CHANGE_SPEED and items[2].param2 == ctx.specs["A"].cruise_airspeed_mps
    assert cmds[-1] == mav.MAV_CMD_NAV_VTOL_LAND and items[-1].z == 0.0
    # 相機開／關成對且開在條帶起點之後
    trig = [it for it in items if it.command == mav.MAV_CMD_DO_SET_CAM_TRIGG_DIST]
    assert len(trig) == 2 * len(a1.strips)
    assert all(t.param1 > 0 for t in trig[0::2]) and all(t.param1 == 0 for t in trig[1::2])
    # DO_LAND_START 在首個 RTB 航點前
    i_ls = cmds.index(mav.MAV_CMD_DO_LAND_START)
    assert items[i_ls + 1].phase == Phase.RTB.name
    # 高度 = alt_msl − home_msl（相對 home）
    hm = a1.waypoints[0].alt_msl_m - a1.waypoints[0].agl_m
    nav = [it for it in items if it.command == mav.MAV_CMD_NAV_WAYPOINT and it.wp_index is not None]
    for it in nav[:20]:
        assert it.z == pytest.approx(a1.waypoints[it.wp_index].alt_msl_m - hm, abs=1e-6)
    sm = items.seqmap
    for sid, _d in a1.strips:
        assert sm.strip_seqs(sid) and sm.items[sm.strip_end_seq(sid)].s_frac_end == pytest.approx(1.0)
    # 用 msl 框架
    items_msl = ml.plan_to_mission_items_fallback(a1, ctx, cfg, frame=mav.MAV_FRAME_GLOBAL)
    it = next(x for x in items_msl if x.command == mav.MAV_CMD_NAV_WAYPOINT and x.wp_index is not None)
    assert it.z == pytest.approx(a1.waypoints[it.wp_index].alt_msl_m, abs=1e-6)


def test_rally_fence_params(env):
    inputs, ctx, cfg, plan = env["inputs"], env["ctx"], env["cfg"], env["plan"]
    rally = ml.rally_items(inputs, ctx)
    assert rally.mission_type == mav.MAV_MISSION_TYPE_RALLY
    assert all(it.command == mav.MAV_CMD_NAV_RALLY_POINT and it.mission_type == mav.MAV_MISSION_TYPE_RALLY for it in rally)
    assert {(round(it.x, 6), round(it.y, 6)) for it in rally} >= {(round(s.lat_deg, 6), round(s.lon_deg, 6))
                                                                  for s in inputs.alternates}
    fence = ml.fence_items(plan, ctx, cfg, cls="A")
    assert fence[0].command == mav.MAV_CMD_NAV_FENCE_RETURN_POINT
    incl = [it for it in fence if it.command == mav.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION]
    assert incl and all(it.param1 == len(incl) for it in incl)
    # 圍籬涵蓋全部航點
    from shapely.geometry import Point, Polygon
    poly = Polygon([(it.y, it.x) for it in incl])
    for w in plan.plans["A1"].waypoints[::25]:
        assert poly.contains(Point(w.lon_deg, w.lat_deg))
    params = ml.suggest_params(ctx.specs["A"], ctx, cfg, uav_plan=plan.plans["A1"], uav_id="A1")
    assert params["AIRSPEED_CRUISE"] == 21.5 and params["WP_LOITER_RAD"] >= 200.0 and "FENCE_ALT_MAX" in params


# ---------------------------------------------------------------------------
# 遙測換算 / 事件
# ---------------------------------------------------------------------------
def test_to_planner_telemetry(env):
    a1, ctx, items = env["a1"], env["ctx"], env["items"]
    tel, sid, seq_end = _mid_strip_tel(env, 0.5, battery_pct=80.0)
    pt = ml.to_planner_telemetry(tel, a1, ctx, "A1", seqmap=items.seqmap)
    assert isinstance(pt, dyn.Telemetry) and pt.uav_id == "A1" and pt.link_ok
    p0, p1, d, i0, i_end = ml._oper_spans(a1)[sid]
    mid = 0.5 * (p0 + p1)
    assert math.hypot(pt.x_m - mid[0], pt.y_m - mid[1]) < 0.5
    assert abs(pt.z_m - a1.waypoints[i0].z_m) < 0.5
    assert pt.current_strip_id == sid and pt.strip_progress == pytest.approx(0.5, abs=0.01)
    assert pt.theta_rad == pytest.approx(math.radians(a1.waypoints[i0].track_deg) % (2 * math.pi), abs=1e-9)
    assert pt.v_air_mps == 21.5
    nom = ctx.models["A1"].nominal_wh
    assert pt.e_tel_wh == pytest.approx(0.8 * nom)
    # Wh 優先於 %
    tel2 = dataclasses.replace(tel, battery_wh_remaining=700.0)
    assert ml.to_planner_telemetry(tel2, a1, ctx, "A1", seqmap=items.seqmap).e_tel_wh == 700.0
    # 條帶之後（轉移首個 seq）→ 前一條帶、進度 1
    tel3 = dataclasses.replace(tel, mission_seq=seq_end + 1)
    pt3 = ml.to_planner_telemetry(tel3, a1, ctx, "A1", seqmap=items.seqmap)
    assert pt3.current_strip_id == sid and pt3.strip_progress == 1.0
    # 地速低 → 用機首方位；無 seqmap → 依時間軸估計條帶
    tel4 = dataclasses.replace(tel, groundspeed_mps=1.0, heading_deg=45.0, track_deg=200.0)
    pt4 = ml.to_planner_telemetry(tel4, a1, ctx, "A1")
    assert pt4.theta_rad == pytest.approx(math.radians(45.0)) and pt4.current_strip_id == sid
    assert pt4.strip_progress == pytest.approx(0.5, abs=0.01)
    # MissionPlan 亦可
    assert ml.to_planner_telemetry(tel, env["plan"], ctx, "A1", seqmap=items.seqmap).current_strip_id == sid


def test_detect_events(env):
    a1, ctx, cfg, items = env["a1"], env["ctx"], env["cfg"], env["items"]
    sm = items.seqmap
    tel, sid, _ = _mid_strip_tel(env, 0.5)
    # 與計畫一致之能量、風 → 無事件
    e_plan = ml._plan_energy_at(a1, tel.t_s)
    nom = ctx.models["A1"].nominal_wh
    tel_ok = dataclasses.replace(tel, battery_wh_remaining=e_plan + 0.5 * cfg.delta_1 * nom, wind_speed_mps=5.0,
                                 wind_dir_from_deg=270.0)
    assert ml.detect_events(tel_ok, a1, ctx, cfg, tel, uav_id="A1", seqmap=sm) == []
    # 能量偏差 > δ1·E_nom → EnergyUpdate
    tel_e = dataclasses.replace(tel, battery_wh_remaining=e_plan - 2.0 * cfg.delta_1 * nom)
    ev = ml.detect_events(tel_e, a1, ctx, cfg, tel, uav_id="A1", seqmap=sm)
    assert len(ev) == 1 and isinstance(ev[0], dyn.EnergyUpdate) and ev[0].e_tel_wh == pytest.approx(tel_e.battery_wh_remaining)
    # 失鏈 / 恢復
    tel_ll = dataclasses.replace(tel, link_ok=False)
    ev = ml.detect_events(tel_ll, a1, ctx, cfg, tel, uav_id="A1", seqmap=sm)
    assert [type(e) for e in ev] == [dyn.LinkLoss] and ev[0].t_s == tel.t_s
    ev = ml.detect_events(tel, a1, ctx, cfg, tel_ll, uav_id="A1", seqmap=sm)
    assert [type(e) for e in ev] == [dyn.LinkRestored] and ev[0].telemetry.current_strip_id == sid
    assert ml.detect_events(tel_ll, a1, ctx, cfg, tel_ll, uav_id="A1", seqmap=sm) == []
    # 風：|ΔW| > 1 → WindUpdate（層數不變、該層改寫）；30 s 內不重發
    tel_w = dataclasses.replace(tel, wind_speed_mps=8.0, wind_dir_from_deg=250.0)
    ev = ml.detect_events(tel_w, a1, ctx, cfg, tel, uav_id="A1", seqmap=sm)
    assert len(ev) == 1 and isinstance(ev[0], dyn.WindUpdate)
    assert len(ev[0].layers) == len(ctx.inputs.wind) and any(l.speed_mps == 8.0 and l.dir_from_deg == 250.0 for l in ev[0].layers)
    assert ml.detect_events(tel_w, a1, ctx, cfg, tel, uav_id="A1", seqmap=sm, last_wind_update_t_s=tel.t_s - 10) == []
    # 意外落地：空中計畫段 + landed_state ON_GROUND（前一筆在空中）→ UAVExit
    tel_g = dataclasses.replace(tel, landed_state=int(mav.MAV_LANDED_STATE_ON_GROUND))
    ev = ml.detect_events(tel_g, a1, ctx, cfg, tel, uav_id="A1", seqmap=sm)
    assert [type(e) for e in ev] == [dyn.UAVExit] and ev[0].reason == "landed_unexpected"
    # 起飛前在地面 → 無事件
    tel_pre = dataclasses.replace(tel_g, t_s=0.0)
    assert ml.detect_events(tel_pre, a1, ctx, cfg, tel_g, uav_id="A1", seqmap=sm) == []
    # dynamic.Telemetry 亦可（僅鏈路／能量規則）
    pt = ml.to_planner_telemetry(tel_e, a1, ctx, "A1", seqmap=sm)
    ev = ml.detect_events(pt, a1, ctx, cfg, None, uav_id="A1")
    assert [type(e) for e in ev] == [dyn.EnergyUpdate]


def test_pymavlink_missing_message(monkeypatch):
    monkeypatch.setattr(ml, "HAVE_PYMAVLINK", False)
    with pytest.raises(ImportError, match="pymavlink"):
        ml.plane_mode_map()


# ---------------------------------------------------------------------------
# 線上：mock autopilot
# ---------------------------------------------------------------------------
def _free_udp_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture
def mock_autopilot(env):
    if not MOCK.exists():
        pytest.skip("tools/mock_autopilot.py 不存在")
    inputs, a1 = env["inputs"], env["a1"]
    home_msl = a1.waypoints[0].alt_msl_m - a1.waypoints[0].agl_m
    port = _free_udp_port()
    cmd = [sys.executable, str(MOCK), "--listen", f"udpin:127.0.0.1:{port}",
           "--home", f"{inputs.home.lat_deg},{inputs.home.lon_deg},{home_msl},0", "--speed", "21.5",
           "--time-scale", "60", "--duration", "40", "--quiet", "--wind", "5,270"]
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    time.sleep(0.4)
    try:
        yield port
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()


def test_vehicle_link_end_to_end(env, mock_autopilot, tmp_path):
    port = mock_autopilot
    inputs, ctx, cfg, plan, a1, items = (env["inputs"], env["ctx"], env["cfg"], env["plan"], env["a1"], env["items"])
    link = ml.VehicleLink(f"udpout:127.0.0.1:{port}", timeout_s=10.0, time_source="boot", uav_id="A1")
    try:
        hb = link.wait_heartbeat()
        assert hb.autopilot == mav.MAV_AUTOPILOT_ARDUPILOTMEGA and link.target_system == 1 and link.link_ok
        # 任務 / rally / fence 上傳：ACK=0
        res = link.upload_mission(items)
        assert res.ok and res.ack_type == mav.MAV_MISSION_ACCEPTED and res.n_items == len(items)
        rally = ml.rally_items(inputs, ctx)
        fence = ml.fence_items(plan, ctx, cfg, cls="A")
        assert link.upload_rally(rally).ok and link.upload_fence(fence).ok
        # 下載逐項相等（含 mission_type）
        back = link.download_mission()
        assert len(back) == len(items) and ml.items_equal(items, back)
        assert all(it.mission_type == mav.MAV_MISSION_TYPE_MISSION for it in back)
        assert ml.items_equal(rally, link.download_mission(mav.MAV_MISSION_TYPE_RALLY))
        assert ml.items_equal(fence, link.download_mission(mav.MAV_MISSION_TYPE_FENCE))
        # 下載回來之項目可重建同一 seqmap
        sm_back = ml.mission_seq_map(back, a1, ctx)
        assert {k: (e.strip_id, e.phase) for k, e in sm_back.items.items()} == \
               {k: (e.strip_id, e.phase) for k, e in items.seqmap.items.items()}
        # 參數
        assert link.set_params({"WP_RADIUS": 30.0, "AIRSPEED_CRUISE": 21.5}) == {"WP_RADIUS": True, "AIRSPEED_CRUISE": True}
        assert link.get_param("WP_RADIUS") == 30.0 and link.get_param("NO_SUCH_PARAM", timeout_s=0.3, retries=0) is None
        # 模式 / ARM / 起飛
        assert link.set_mode("AUTO") and link.arm(True) and link.start_mission(1)
        with pytest.raises(ValueError):
            link.set_mode("NOT_A_MODE")
        t1 = time.time()
        tel = None
        while time.time() - t1 < 20:
            tel = link.read_telemetry(1.0)
            if tel is not None and tel.vtol_state == mav.MAV_VTOL_STATE_FW:
                break
        assert tel is not None and tel.vtol_state == mav.MAV_VTOL_STATE_FW
        assert tel.armed and tel.mode == "AUTO" and tel.link_ok and tel.landed_state == mav.MAV_LANDED_STATE_IN_AIR
        assert tel.battery_pct is not None and 0 < tel.battery_pct <= 100
        assert tel.wind_speed_mps == pytest.approx(5.0) and tel.wind_dir_from_deg == pytest.approx(270.0)
        assert tel.mission_seq >= 2 and tel.rel_alt_m > 30.0 and tel.t_s > 0
        # → 規劃器 Telemetry：ENU 距 home 合理、link_ok、能量 = % × E_nom
        pt = ml.to_planner_telemetry(tel, a1, ctx, "A1", seqmap=items.seqmap)
        assert isinstance(pt, dyn.Telemetry) and pt.link_ok and math.hypot(pt.x_m, pt.y_m) < 3000.0
        assert pt.e_tel_wh == pytest.approx(tel.battery_pct / 100.0 * ctx.models["A1"].nominal_wh)
        assert abs(pt.z_m - (tel.alt_msl_m - ctx.frame.h0_msl_m)) < 2.0
        evs = ml.detect_events(tel, a1, ctx, cfg, None, uav_id="A1", seqmap=items.seqmap)
        assert all(not isinstance(e, (dyn.LinkLoss, dyn.UAVExit)) for e in evs)
        # FleetLink：upload_plan（輸出檔）+ replan_and_reupload（MISSION_SET_CURRENT 至條帶後）
        fleet = ml.FleetLink({"A1": link})
        reports = fleet.upload_plan(plan, ctx, cfg, out_dir=tmp_path, uav_ids=["A1"])
        rep = reports["A1"]
        assert rep.mission.ok and rep.rally is not None and rep.rally.ok and rep.fence is not None and rep.fence.ok
        assert (tmp_path / "A1.waypoints").exists() and (tmp_path / "A1.seqmap.json").exists()
        assert ml.MissionSeqMap.load(tmp_path / "A1.seqmap.json").to_dict() == rep.seqmap.to_dict()
        sid0 = a1.strips[0][0]
        rr = fleet.replan_and_reupload("A1", a1, ctx, cfg, current_strip_id=sid0, out_dir=tmp_path)
        assert rr.upload.ok and rr.switch_seq == rr.seqmap.seq_after_strip(sid0) and rr.set_current_ok
        tel2 = link.read_telemetry(1.0)
        assert tel2 is not None and tel2.mission_seq >= rr.switch_seq
        # poll：規劃器 Telemetry 與事件
        tels, _events = fleet.poll(plan, ctx, cfg, timeout_s=0.5)
        assert "A1" in tels and isinstance(tels["A1"], dyn.Telemetry)
        assert link.clear_mission(mav.MAV_MISSION_TYPE_RALLY) and link.download_mission(mav.MAV_MISSION_TYPE_RALLY) == []
    finally:
        link.close()


# ---------------------------------------------------------------------------
# 審查回歸：ENERGY_UPDATE 遲滯／防抖（EnergyEventState）
# ---------------------------------------------------------------------------
def test_energy_event_debounce(env):
    """對著 mock（電池模型與計畫不同）輪詢時，同一段能量偏差只能送一次 EnergyUpdate，不是每筆都送。"""
    a1, ctx, cfg, items = env["a1"], env["ctx"], env["cfg"], env["items"]
    sm = items.seqmap
    tel, _sid, _ = _mid_strip_tel(env, 0.5)
    nom = ctx.models["A1"].nominal_wh
    e_plan = ml._plan_energy_at(a1, tel.t_s)
    dev = 2.0 * cfg.delta_1 * nom                    # 穩定的系統性偏差（mock 電池模型 vs 計畫）
    st = ml.EnergyEventState()
    fired = []
    for k in range(60):                              # 4 Hz 輪詢 ⇒ 15 s；時間軸沿計畫時間前進
        t = tel.t_s + 0.25 * k
        tk = dataclasses.replace(tel, t_s=t, battery_wh_remaining=ml._plan_energy_at(a1, t) - dev)
        ev = ml.detect_events(tk, a1, ctx, cfg, tk, uav_id="A1", seqmap=sm, energy_state=st)
        fired.extend(e for e in ev if isinstance(e, dyn.EnergyUpdate))
    assert len(fired) == 1, [e.t_s for e in fired]   # 60 筆超標 → 1 個事件
    assert st.n_events == 1 and st.n_suppressed >= 50
    # 無狀態呼叫（單次檢查）維持原本語意：每筆都送
    t2 = tel.t_s + 1.0
    tk = dataclasses.replace(tel, t_s=t2, battery_wh_remaining=ml._plan_energy_at(a1, t2) - dev)
    assert len([e for e in ml.detect_events(tk, a1, ctx, cfg, tk, uav_id="A1", seqmap=sm)
                if isinstance(e, dyn.EnergyUpdate)]) == 1


def test_energy_event_debounce_gates(env):
    """三道閘：dwell（單筆突波不觸發）、遲滯（門檻附近不抖動）、重新觸發（偏差再變 δ1·E_nom 才重送）。"""
    a1, ctx, cfg, items = env["a1"], env["ctx"], env["cfg"], env["items"]
    sm = items.seqmap
    tel, _sid, _ = _mid_strip_tel(env, 0.5)
    nom = ctx.models["A1"].nominal_wh
    thr = cfg.delta_1 * nom

    def run(state, t, dev):
        tk = dataclasses.replace(tel, t_s=t, battery_wh_remaining=ml._plan_energy_at(a1, t) - dev)
        return [e for e in ml.detect_events(tk, a1, ctx, cfg, tk, uav_id="A1", seqmap=sm, energy_state=state)
                if isinstance(e, dyn.EnergyUpdate)]

    # (1) 單筆突波：第一筆超標不觸發（dwell 未滿）
    st = ml.EnergyEventState()
    assert run(st, tel.t_s, 3.0 * thr) == []
    assert run(st, tel.t_s + 0.5, 0.0) == []                       # 回到正常 → 重置
    assert st.t_first_exceed_s is None
    # (2) 持續超標：滿足 dwell + 樣本數後送出一次
    st = ml.EnergyEventState()
    assert run(st, 100.0, 2.0 * thr) == []
    assert run(st, 100.0 + ml.ENERGY_DWELL_S, 2.0 * thr)
    assert run(st, 100.0 + ml.ENERGY_DWELL_S + 1.0, 2.0 * thr) == []      # 已送過，偏差未變 → 不重送
    # (3) 遲滯：落到 release 帶（0.6–1.0 δ1）不算離開也不重觸發
    assert run(st, 200.0, 0.8 * thr) == []
    assert st.t_first_exceed_s is not None
    # (4) 偏差再擴大 δ1·E_nom → 重送
    ev = run(st, 210.0, 2.0 * thr) or run(st, 211.0, 3.2 * thr)
    assert ev and st.n_events == 2
    # (5) 距上次事件 ≥ min_interval 且仍超標 → 重送
    st2 = ml.EnergyEventState()
    assert run(st2, 300.0, 2.0 * thr) == [] and run(st2, 310.0, 2.0 * thr)
    assert run(st2, 310.0 + ml.ENERGY_MIN_INTERVAL_S, 2.0 * thr)
    assert st2.n_events == 2


def test_fleetlink_keeps_energy_state():
    fl = ml.FleetLink({})
    assert fl.energy_state == {}
    st = fl.energy_state.setdefault("A1", ml.EnergyEventState())
    assert isinstance(st, ml.EnergyEventState) and st.n_events == 0
