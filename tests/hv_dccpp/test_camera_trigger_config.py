"""tests/hv_dccpp/test_camera_trigger_config.py — 相機觸發設定（`ardupilot.CameraTriggerConfig`）與任務中 206 之擺放。

數值鎖（不是「沒丟例外」）：
* 每條帶恰一個 206 開，p1 與 `trigger_distance_m(感測器, 條帶起點 AGL)` 在 float32 下相等；
* DO_LAND_START 緊接 206(0)（失效保護 RTL 跳到此處會略過所有條帶的關閉）；DO_LAND_START 與 NAV_VTOL_LAND 之間沒有開；
* lead-in：共線 NAV 在條帶起點後 lead_in 處（haversine 誤差 < 0.5 m），206 開掛在它之後；
* 地形重發：條帶中途 AGL 偏離 > tol 才重發、p3 = 0、依據之 AGL 隨重發更新；
* recommended_params：servo／relay（4.4.4 與 4.5.7 參數名）／mavlink_v2／回授 之鍵與值完全相符；CAM1_DURATION = 0.1；
* export_all 之 `.param` 與 mavlink_link.suggest_params 對同一設定之 CAM*／RELAY*／SERVO* 完全一致。

全部離線：不開序列埠、不連 MAVLink（`FleetLink()` 不帶任何連線，只走匯出）。不用 pymavlink 的 mavwp（2.4.49 在
MAVLINK20=1 下有上游 TypeError），一律以本模組的 `read_qgc_wpl` 讀回。
"""
from __future__ import annotations

import dataclasses
import logging
import math
import os
from types import SimpleNamespace

import numpy as np
import pytest

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

from hv_dccpp import ardupilot as ap  # noqa: E402
from hv_dccpp import mavlink_link as ml  # noqa: E402
from hv_dccpp.context import build_context  # noqa: E402
from hv_dccpp.mission import Mode, Phase, UAVPlan, Waypoint  # noqa: E402
from hv_dccpp.planner import plan_mission  # noqa: E402
from hv_dccpp.scenario import build_mini_example  # noqa: E402

M = mavutil.mavlink
CAM = M.MAV_CMD_DO_SET_CAM_TRIGG_DIST
LAND_START = M.MAV_CMD_DO_LAND_START
VTOL_LAND = M.MAV_CMD_NAV_VTOL_LAND
NAV_WP = M.MAV_CMD_NAV_WAYPOINT
CAMKEYS = ("CAM", "RELAY", "SERVO")


# ---------------------------------------------------------------------------
# fixtures / helpers
# ---------------------------------------------------------------------------
@pytest.fixture(scope="module")
def mini():
    inputs, cfg = build_mini_example()
    cfg = dataclasses.replace(cfg, dem_step_m=50.0)
    ctx = build_context(inputs, cfg, terrain_margin_m=1500.0)
    mp = plan_mission(inputs, cfg, ctx=ctx)
    assert set(mp.plans) == {"A1", "B1"}
    return SimpleNamespace(inputs=inputs, cfg=cfg, ctx=ctx, mp=mp, sensor=ctx.sensors[("B", "default_rgb")])


def _haversine_m(lat1, lon1, lat2, lon2) -> float:
    r = 6371008.8
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def _cam(items):
    return [it for it in items if it.command == CAM]


def _on(items):
    return [it for it in items if it.command == CAM and it.param1 > 0]


def _prev_nav(items, seq):
    return [it for it in items if it.seq < seq and it.is_nav][-1]


def _strip_starts(up: UAVPlan) -> list[Waypoint]:
    """每條帶第一個 OPERATE 航點（任務順序）。"""
    out, seen = [], set()
    for w in up.waypoints:
        if w.phase is Phase.OPERATE and w.sensor_on and w.strip_id not in seen:
            seen.add(w.strip_id)
            out.append(w)
    return out


def _cam_params(p):
    return {k: v for k, v in p.items() if k.startswith(CAMKEYS)}


def _wp(x, y, z, t, phase, *, agl=None, mode=Mode.CRUISE, va=19.0, strip=None, on=False, trk=0.0):
    return Waypoint(x_m=x, y_m=y, z_m=z, lat_deg=0.0, lon_deg=0.0, alt_msl_m=z + 100.0,
                    agl_m=z if agl is None else agl, t_s=t, mode=mode, phase=phase, sensor_on=on, strip_id=strip,
                    cell_id=None, dir=None, airspeed_mps=va, ground_speed_mps=va, track_deg=trk, heading_deg=trk,
                    energy_remaining_wh=300.0, nearest_alternate="H", alt_reachable=True, lost_link_action="RTB",
                    arc=None)


def _synthetic(strip_legs, *, transfer_before_rtb=False, rtb_va=19.0) -> UAVPlan:
    """B1 合成計畫（直線，無弧）：起飛 → 北向 ENTRY → 一條帶（`strip_legs` = [(dx, dy, agl), …]，每 25 m 取樣；
    腿上的取樣用該腿之 agl，腿終點＝轉角則用**下一腿**之 agl）→（可選）TRANSFER 東移 → RTB 東移再南下 → 降落。
    z 恆為 100（只有 AGL 變：模擬地形起伏而飛機等高）；轉角（航向變 > 1°）會成為條帶中途的 NAV。"""
    wps = [_wp(0.0, 0.0, 0.0, 0.0, Phase.TAKEOFF, mode=Mode.HOVER, va=0.0),
           _wp(0.0, 0.0, 50.0, 25.0, Phase.TRANS_OUT, mode=Mode.TRANSITION, va=16.0),
           _wp(0.0, 384.0, 90.0, 45.0, Phase.ACCEL),
           _wp(0.0, 500.0, 100.0, 51.0, Phase.ENTRY)]
    t = 60.0
    x, y = 0.0, 600.0
    wps.append(_wp(x, y, 100.0, t, Phase.OPERATE, agl=strip_legs[0][2], strip="S1", on=True))
    for li, (dx, dy, agl) in enumerate(strip_legs):
        n = int(round(math.hypot(dx, dy) / 25.0))
        agl_corner = strip_legs[li + 1][2] if li + 1 < len(strip_legs) else agl
        for k in range(1, n + 1):
            t += 25.0 / 19.0
            wps.append(_wp(x + dx * k / n, y + dy * k / n, 100.0, t, Phase.OPERATE,
                           agl=agl_corner if k == n else agl, strip="S1", on=True))
        x, y = x + dx, y + dy
    if transfer_before_rtb:
        for k in range(1, 9):
            t += 25.0 / 19.0
            wps.append(_wp(x + 25.0 * k, y, 100.0, t, Phase.TRANSFER))
        x += 200.0
    for k in range(1, 13):
        t += 25.0 / rtb_va
        wps.append(_wp(x + 25.0 * k, y, 100.0, t, Phase.RTB, va=rtb_va))
    x += 300.0
    for k in range(1, 21):
        t += 25.0 / rtb_va
        wps.append(_wp(x, y - 50.0 * k, 100.0, t, Phase.RTB, va=rtb_va))
    y -= 1000.0
    wps.append(_wp(x, y - 400.0, 90.0, t + 25, Phase.DECEL, va=rtb_va))
    wps.append(_wp(x, y - 500.0, 90.0, t + 31, Phase.TRANS_IN, mode=Mode.TRANSITION, va=16.0))
    wps.append(_wp(x, y - 880.0, 50.0, t + 56, Phase.LAND, mode=Mode.HOVER, va=0.0))
    wps.append(_wp(x, y - 880.0, 0.0, t + 116, Phase.LAND, mode=Mode.HOVER, va=0.0))
    return UAVPlan("B1", "B", wps, [("S1", 1)], 0.0, t + 31, t + 131, "H", 100.0, 50.0, 3000.0, 3.0, 0, [])


# ---------------------------------------------------------------------------
# 任務項目：每帶一開、float32 距離、DO_LAND_START 之後的 206(0)
# ---------------------------------------------------------------------------
def test_one_on_per_strip_and_float32_distance(mini):
    up = mini.mp.plans["B1"]
    for cc in (None, ap.CameraTriggerConfig()):
        items = ap.plan_to_mission_items(up, mini.ctx, mini.cfg, camera_cfg=cc)
        on = _on(items)
        starts = _strip_starts(up)
        assert len(on) == len(up.strips) == len(starts) == 3
        for it, w in zip(on, starts):
            # 給了 camera_cfg ⇒ 帶起點 AGL 與規劃作業高度 100 m 差 ≤ 0.5 m 時用 100 m（F1 (e)：平地各帶 p1 逐位元相同）；
            # None ＝ 舊行為（該點 AGL）
            h = w.agl_m if cc is None else (100.0 if abs(w.agl_m - 100.0) <= 0.5 else w.agl_m)
            d = ap.trigger_distance_m(mini.sensor, h)
            assert np.float32(it.param1) == np.float32(d)
            assert it.param3 == 1.0 and it.param2 == 0.0 and it.param4 == 0.0
            # 開掛在條帶起點 NAV 之後（< 1 m）
            nav = _prev_nav(items, it.seq)
            xy = mini.ctx.frame.to_enu(nav.x, nav.y)[0, :2]
            assert math.hypot(xy[0] - w.x_m, xy[1] - w.y_m) < 1.0
        assert ap.trigger_distance_m(mini.sensor, 100.0) == pytest.approx(29.25, abs=0.01)
    # first_shot_immediate=False → p3 = 0，其餘不變
    items0 = ap.plan_to_mission_items(up, mini.ctx, mini.cfg,
                                      camera_cfg=ap.CameraTriggerConfig(first_shot_immediate=False))
    assert [it.param3 for it in _on(items0)] == [0.0, 0.0, 0.0]
    base = ap.plan_to_mission_items(up, mini.ctx, mini.cfg, camera_cfg=ap.CameraTriggerConfig())   # 同為給 camera_cfg
    assert [(it.command, it.param1, it.x, it.y, it.z) for it in items0] == \
        [(it.command, it.param1, it.x, it.y, it.z) for it in base]


def test_land_start_is_followed_by_off_and_closes_last_strip(mini):
    up = mini.mp.plans["B1"]
    items = ap.plan_to_mission_items(up, mini.ctx, mini.cfg)
    by_seq = {it.seq: it for it in items}
    ls = [it for it in items if it.command == LAND_START]
    assert len(ls) == 1
    ls = ls[0]
    land = next(it for it in items if it.command == VTOL_LAND)
    nxt = by_seq[ls.seq + 1]
    assert nxt.command == CAM and (nxt.param1, nxt.param3) == (0.0, 0.0)
    # 206(0) 在下一個 NAV 之前；DO_LAND_START 與落地之間沒有開
    next_nav = next(it for it in items if it.seq > ls.seq and it.is_nav)
    assert ls.seq < nxt.seq < next_nav.seq
    assert not [it for it in _on(items) if ls.seq < it.seq <= land.seq]
    # 最後一條帶終點＝RTB 首點：開 → NAV(條帶終點) → DO_LAND_START → 206(0)，中間沒有別的 206
    last_on = _on(items)[-1]
    end_nav = by_seq[ls.seq - 1]
    assert end_nav.command == NAV_WP
    assert [it.command for it in items if last_on.seq < it.seq < ls.seq] == [NAV_WP]
    last_sid = up.strips[-1][0]
    i_last = max(i for i, w in enumerate(up.waypoints) if w.strip_id == last_sid and w.phase is Phase.OPERATE)
    w_end = up.waypoints[i_last + 1]
    assert w_end.phase is Phase.RTB
    xy = mini.ctx.frame.to_enu(end_nav.x, end_nav.y)[0, :2]
    assert math.hypot(xy[0] - w_end.x_m, xy[1] - w_end.y_m) < 1.0
    # 每條帶恰一個關：最後一條帶的關就是 DO_LAND_START 後那一個
    assert len([it for it in _cam(items) if it.param1 == 0.0]) == len(up.strips)
    # LiDAR 機（A1）沒開過相機 ⇒ DO_LAND_START 後不補 206，且完全沒有 206
    a_items = ap.plan_to_mission_items(mini.mp.plans["A1"], mini.ctx, mini.cfg)
    a_ls = next(it for it in a_items if it.command == LAND_START)
    assert not _cam(a_items) and a_items[a_ls.seq + 1].is_nav


def test_land_start_off_with_transfer_and_speed_change(mini):
    """(1) 條帶後有 TRANSFER（相機已關）：DO_LAND_START 照舊在第一個 RTB NAV 之前，仍緊接 206(0)（跳轉保護）；
    (2) 條帶終點＝RTB 首點且 RTB 改空速：NAV(終點) → DO_CHANGE_SPEED → DO_LAND_START → 206(0)。"""
    ctx, cfg = mini.ctx, mini.cfg
    up = _synthetic([(0.0, 1000.0, 100.0)], transfer_before_rtb=True)
    items = ap.plan_to_mission_items(up, ctx, cfg)
    by_seq = {it.seq: it for it in items}
    ls = next(it for it in items if it.command == LAND_START)
    cam = _cam(items)
    assert [c.param1 > 0 for c in cam] == [True, False, False]
    assert cam[1].seq < ls.seq and by_seq[ls.seq + 1] is cam[2] and by_seq[ls.seq + 2].is_nav
    first_rtb = by_seq[ls.seq + 2]
    xy = ctx.frame.to_enu(first_rtb.x, first_rtb.y)[0, :2]
    # 第一個「降落段」NAV：TRANSFER→RTB 東移同屬一條直線 run，故是 RTB 轉南的轉角 (500, 1600)
    p_rtb = next(q for q in ap.reduce_waypoints(up.waypoints, ctx.specs["B"], cfg, 30.0) if q.phase is Phase.RTB)
    assert (p_rtb.x, p_rtb.y) == pytest.approx((500.0, 1600.0), abs=1e-6)
    assert math.hypot(xy[0] - p_rtb.x, xy[1] - p_rtb.y) < 1.0
    # (2)
    up2 = _synthetic([(0.0, 1000.0, 100.0)], rtb_va=21.0)
    items2 = ap.plan_to_mission_items(up2, ctx, cfg)
    by2 = {it.seq: it for it in items2}
    ls2 = next(it for it in items2 if it.command == LAND_START)
    seq_cmds = [by2[ls2.seq - 2].command, by2[ls2.seq - 1].command, ls2.command, by2[ls2.seq + 1].command]
    assert seq_cmds == [NAV_WP, M.MAV_CMD_DO_CHANGE_SPEED, LAND_START, CAM]
    assert by2[ls2.seq - 1].param2 == 21.0 and by2[ls2.seq + 1].param1 == 0.0
    assert [c.param1 > 0 for c in _cam(items2)] == [True, False]
    for c in (it for it in items2 if it.command == M.MAV_CMD_DO_CHANGE_SPEED):
        assert by2[c.seq - 1].is_nav                    # DO_CHANGE_SPEED 仍緊接其 NAV


# ---------------------------------------------------------------------------
# lead-in
# ---------------------------------------------------------------------------
def test_lead_in_nav_inserted_collinear_at_distance(mini):
    up = mini.mp.plans["B1"]
    lead = 40.0
    # 對照組：同樣給 camera_cfg（不插引入段）——p1 的高度規則（F1 (e)）只在給了 camera_cfg 時套用
    base = ap.plan_to_mission_items(up, mini.ctx, mini.cfg, camera_cfg=ap.CameraTriggerConfig())
    items = ap.plan_to_mission_items(up, mini.ctx, mini.cfg, camera_cfg=ap.CameraTriggerConfig(camera_lead_in_m=lead))
    on, on0 = _on(items), _on(base)
    assert len(on) == len(on0) == 3
    assert [it.param1 for it in on] == [it.param1 for it in on0]          # d 仍依條帶起點（不因引入段改變）
    by_seq = {it.seq: it for it in items}
    lead_navs = []
    for it in on:
        ln = by_seq[it.seq - 1]                       # 開緊接 lead-in NAV
        st = _prev_nav(items, ln.seq)                 # 再前一個 NAV＝條帶起點
        # 引入 NAV 接受半徑 1 m（F1 (a)：r_acc 30 m 會讓 206 開提早 30 m 執行）；條帶起點仍是 r_acc
        assert ln.command == NAV_WP and ln.frame == st.frame
        assert ln.param2 == ap.LEAD_IN_ACCEPT_RADIUS_M == 1.0 and st.param2 == 30.0
        assert ln.z == st.z
        assert _haversine_m(st.x, st.y, ln.x, ln.y) == pytest.approx(lead, abs=0.5)
        end = next(x for x in items if x.seq > ln.seq and x.is_nav)        # 條帶終點
        p = mini.ctx.frame.to_enu([st.x, ln.x, end.x], [st.y, ln.y, end.y])[:, :2]
        u = (p[2] - p[0]) / np.linalg.norm(p[2] - p[0])
        v = p[1] - p[0]
        assert abs(float(u[0] * v[1] - u[1] * v[0])) < 0.5           # 共線（橫向偏差）
        assert float(np.dot(u, v)) == pytest.approx(lead, abs=0.5)
        lead_navs.append(ln.seq)
    # 除了插入的 3 個 lead-in NAV，其餘項目與預設完全相同
    rest = [it for it in items if it.seq not in lead_navs]
    # 唯一差別：對照組（沒有引入段）的 206 開直接掛在帶起點 ⇒ 帶起點用 1 m 接受半徑；有引入段時帶起點回到 r_acc
    base_starts = {_prev_nav(base, it.seq).seq for it in _on(base)}
    assert {by_seq0.param2 for by_seq0 in base if by_seq0.seq in base_starts} == {ap.LEAD_IN_ACCEPT_RADIUS_M}
    mask = lambda it, starts: (it.command, it.param1, None if it.seq in starts else it.param2, it.param3,
                               it.x, it.y, it.z)
    starts = {_prev_nav(items, s).seq for s in lead_navs}
    assert [mask(it, starts) for it in rest] == [mask(it, base_starts) for it in base]


def test_lead_in_skipped_on_short_strip_with_warning(mini, caplog):
    up = mini.mp.plans["B1"]
    base = ap.plan_to_mission_items(up, mini.ctx, mini.cfg, camera_cfg=ap.CameraTriggerConfig())
    with caplog.at_level(logging.WARNING, logger="hv_dccpp.ardupilot"):
        items = ap.plan_to_mission_items(up, mini.ctx, mini.cfg,
                                         camera_cfg=ap.CameraTriggerConfig(camera_lead_in_m=2000.0))
    assert [(it.command, it.x, it.y, it.param1) for it in items] == [(it.command, it.x, it.y, it.param1) for it in base]
    msgs = [r.getMessage() for r in caplog.records if "lead-in" in r.getMessage()]
    assert len(msgs) == 3 and all("2000.0" in m for m in msgs)
    # 合成條帶 1000 m：lead 500 → 剛好 = L/2 可插；lead 501 → 不插
    up_s = _synthetic([(0.0, 1000.0, 100.0)])
    n500 = len(ap.plan_to_mission_items(up_s, mini.ctx, mini.cfg, camera_cfg=ap.CameraTriggerConfig(camera_lead_in_m=500.0)))
    n501 = len(ap.plan_to_mission_items(up_s, mini.ctx, mini.cfg, camera_cfg=ap.CameraTriggerConfig(camera_lead_in_m=501.0)))
    n0 = len(ap.plan_to_mission_items(up_s, mini.ctx, mini.cfg))
    assert (n500, n501) == (n0 + 1, n0)


# ---------------------------------------------------------------------------
# 地形重發 206
# ---------------------------------------------------------------------------
def test_terrain_retrigger(mini):
    ctx, cfg, sensor = mini.ctx, mini.cfg, mini.sensor
    # 三腿條帶：AGL 100 → 130（+30%）→ 135（相對 130 只 +3.8%；相對原 100 是 +35%）
    legs = [(0.0, 500.0, 100.0), (87.0, 492.0, 130.0), (-87.0, 492.0, 135.0)]
    up = _synthetic(legs)
    plain = ap.plan_to_mission_items(up, ctx, cfg)
    assert len(_on(plain)) == 1
    off0 = next(it for it in _cam(plain) if it.param1 == 0.0)
    nav_in_strip = [it for it in plain if it.is_nav and _on(plain)[0].seq < it.seq < off0.seq]
    assert len(nav_in_strip) == 3                     # 兩個轉角（中途 NAV）＋條帶終點
    items = ap.plan_to_mission_items(up, ctx, cfg, camera_cfg=ap.CameraTriggerConfig(terrain_retrigger_tol=0.10))
    on = _on(items)
    assert len(on) == 2
    assert np.float32(on[0].param1) == np.float32(ap.trigger_distance_m(sensor, 100.0)) and on[0].param3 == 1.0
    assert np.float32(on[1].param1) == np.float32(ap.trigger_distance_m(sensor, 130.0)) and on[1].param3 == 0.0
    corner = {it.seq: it for it in items}[on[1].seq - 1]
    assert corner.is_nav
    xy = ctx.frame.to_enu(corner.x, corner.y)[0, :2]
    assert math.hypot(xy[0] - 0.0, xy[1] - 1100.0) < 1.0           # 第一個轉角
    assert [it.param1 > 0 for it in _cam(items)] == [True, True, False]
    # 容差 0.5（30% < 50%）／None → 不重發
    for tol in (0.5, None):
        it2 = ap.plan_to_mission_items(up, ctx, cfg, camera_cfg=ap.CameraTriggerConfig(terrain_retrigger_tol=tol))
        assert len(_on(it2)) == 1
    # 容差 0.03：130 重發，135 相對 130 為 3.85% > 3% 也重發
    it3 = ap.plan_to_mission_items(up, ctx, cfg, camera_cfg=ap.CameraTriggerConfig(terrain_retrigger_tol=0.03))
    assert [round(it.param1, 4) for it in _on(it3)] == \
        [round(ap.trigger_distance_m(sensor, a), 4) for a in (100.0, 130.0, 135.0)]
    # 平地 mini（條帶只有起訖兩點）→ 不重發
    b1 = ap.plan_to_mission_items(mini.mp.plans["B1"], ctx, cfg,
                                  camera_cfg=ap.CameraTriggerConfig(terrain_retrigger_tol=0.10))
    assert len(_on(b1)) == 3


# ---------------------------------------------------------------------------
# recommended_params
# ---------------------------------------------------------------------------
def _camw(w):
    """只留相機相關的警告（mini 無電池電壓 ⇒ 另有一則 BATT_* 警告；HERO 的 AIRSPEED_MAX 1.4×19 = 26.6 是 AP_Int16，
    匯出 26 並記一則取整有效值警告——2026-10-14 F1 (c)，值另由 tests/test_hdmap_fix_exporter.py 鎖）。"""
    return [s for s in w if not s.startswith(("BATT_", "AIRSPEED_MAX 26.6 -> 26:"))]


def _rp(mini, cc, kinds=("rgb",), warn=None):
    return ap.recommended_params(mini.ctx.specs["B"], None, mini.cfg, sensor_kinds=list(kinds), camera_cfg=cc,
                                 warnings=warn)


COMMON = {"CAM1_DURATION": 0.1, "CAM1_INTRVAL_MIN": 0.0, "CAM_AUTO_ONLY": 1.0, "CAM_MAX_ROLL": 0.0,
          "CAM1_FEEDBAK_PIN": -1.0, "CAM1_FEEDBAK_POL": 1.0, "CAM1_TRIGG_DIST": 0.0}


def test_recommended_params_default_and_servo(mini):
    assert _cam_params(_rp(mini, None)) == {"CAM1_TYPE": 1.0, "CAM1_DURATION": 0.1, "SERVO10_FUNCTION": 10.0}
    assert _cam_params(_rp(mini, ap.CameraTriggerConfig())) == {**COMMON, "CAM1_TYPE": 1.0, "SERVO10_FUNCTION": 10.0}
    cc = ap.CameraTriggerConfig(servo_channel=12, duration_s=0.2, interval_min_s=0.56, auto_only=False,
                                max_roll_deg=15.0)
    assert _cam_params(_rp(mini, cc)) == {**COMMON, "CAM1_TYPE": 1.0, "SERVO12_FUNCTION": 10.0, "CAM1_DURATION": 0.2,
                                          "CAM1_INTRVAL_MIN": 0.56, "CAM_AUTO_ONLY": 0.0, "CAM_MAX_ROLL": 15.0}
    # 預設（None）只有 lidar → 無相機鍵
    assert _cam_params(_rp(mini, None, kinds=("lidar",))) == {"SERVO9_FUNCTION": 0.0}


def test_recommended_params_relay_by_firmware(mini):
    w45: list[str] = []
    cc45 = ap.CameraTriggerConfig(backend="relay", firmware="4.5.7", relay_pin=50, aux_servo_channel_for_relay=9,
                                  interval_min_s=0.56, max_roll_deg=10.0)
    assert _cam_params(_rp(mini, cc45, warn=w45)) == {
        **COMMON, "CAM1_TYPE": 2.0, "CAM1_RELAY_ON": 1.0, "RELAY1_FUNCTION": 4.0, "RELAY1_PIN": 50.0,
        "SERVO9_FUNCTION": -1.0, "CAM1_INTRVAL_MIN": 0.56, "CAM_MAX_ROLL": 10.0}
    assert _camw(w45) == []
    w44: list[str] = []
    cc44 = dataclasses.replace(cc45, firmware="4.4.4")
    assert _cam_params(_rp(mini, cc44, warn=w44)) == {
        **COMMON, "CAM1_TYPE": 2.0, "CAM1_RELAY_ON": 1.0, "RELAY_PIN": 50.0, "SERVO9_FUNCTION": -1.0,
        "CAM1_INTRVAL_MIN": 0.56, "CAM_MAX_ROLL": 10.0}
    assert len(_camw(w44)) == 1 and "TRIM_ARSPD_CM" in _camw(w44)[0]
    cc47 = ap.CameraTriggerConfig(backend="relay", firmware="4.7.2", relay_instance=2, relay_pin=51)
    w47: list[str] = []
    assert _cam_params(_rp(mini, cc47, warn=w47)) == {
        **COMMON, "CAM1_TYPE": 2.0, "CAM1_RELAY_ON": 1.0, "RELAY2_FUNCTION": 4.0, "RELAY2_PIN": 51.0}
    assert len(_camw(w47)) == 1 and "SERVOn_FUNCTION=-1" in _camw(w47)[0]          # 沒給 aux 通道 ⇒ 明講


def test_recommended_params_mavlink_v2_and_feedback(mini):
    assert _cam_params(_rp(mini, ap.CameraTriggerConfig(backend="mavlink_v2", firmware="4.5.7"))) == \
        {**COMMON, "CAM1_TYPE": 6.0}
    cc = ap.CameraTriggerConfig(backend="relay", firmware="4.5.7", relay_pin=50, aux_servo_channel_for_relay=9,
                                feedback_pin=51, feedback_polarity=0, feedback_servo_channel=10)
    assert _cam_params(_rp(mini, cc)) == {
        **COMMON, "CAM1_TYPE": 2.0, "CAM1_RELAY_ON": 1.0, "RELAY1_FUNCTION": 4.0, "RELAY1_PIN": 50.0,
        "SERVO9_FUNCTION": -1.0, "CAM1_FEEDBAK_PIN": 51.0, "CAM1_FEEDBAK_POL": 0.0, "SERVO10_FUNCTION": -1.0}
    w: list[str] = []
    cc2 = ap.CameraTriggerConfig(feedback_pin=52)
    assert _cam_params(_rp(mini, cc2, warn=w))["CAM1_FEEDBAK_PIN"] == 52.0
    assert len(_camw(w)) == 1 and "CAM1_FEEDBAK_PIN 52" in _camw(w)[0]
    # 每個輸出的鍵都有說明
    for c in (cc, cc2, ap.CameraTriggerConfig(servo_channel=13),
              ap.CameraTriggerConfig(backend="relay", firmware="4.4.4", relay_pin=50, aux_servo_channel_for_relay=11),
              ap.CameraTriggerConfig(backend="relay", firmware="4.7.2", relay_instance=6, relay_pin=55)):
        comments = {**ap.PARAM_COMMENTS, **ap.camera_param_comments(c)}
        assert all(k in comments for k in _cam_params(_rp(mini, c))), c


def test_recommended_params_camera_cfg_not_applicable(mini):
    cc = ap.CameraTriggerConfig(backend="relay", firmware="4.5.7", relay_pin=50, aux_servo_channel_for_relay=9)
    w: list[str] = []
    assert _cam_params(_rp(mini, cc, kinds=("lidar",), warn=w)) == {"SERVO9_FUNCTION": 0.0}
    assert any("NOT applied" in s for s in w)
    # 雙感測同機：relay 的 AUX1＝SERVO9 與 LiDAR 佔位 SERVO9 衝突 ⇒ 拒絕
    with pytest.raises(ValueError, match="SERVO9_FUNCTION"):
        _rp(mini, cc, kinds=("rgb", "lidar"))
    assert _cam_params(_rp(mini, dataclasses.replace(cc, aux_servo_channel_for_relay=11), kinds=("rgb", "lidar")))[
        "SERVO11_FUNCTION"] == -1.0


@pytest.mark.parametrize("kw, msg", [
    (dict(backend="gopro"), "backend"),
    (dict(backend="relay", relay_pin=50), "firmware"),
    (dict(backend="relay", firmware="4.5.7"), "relay_pin"),
    (dict(backend="mavlink_v2"), "4.5"),
    (dict(backend="mavlink_v2", firmware="4.4.4"), "4.5"),
    (dict(backend="relay", firmware="4.4.4", relay_pin=50, relay_instance=2), "relay instance 0"),
    (dict(backend="relay", firmware="4.5.7", relay_pin=50, relay_instance=7), "relay_instance"),
    (dict(backend="relay", firmware="4.4.4", relay_pin=200), "relay_pin"),
    (dict(firmware="4.3.7"), "4.4.0"),
    (dict(firmware="latest"), "版本"),
    (dict(duration_s=0.0), "duration_s"),
    (dict(duration_s=5.5), "duration_s"),
    (dict(duration_s=float("nan")), "duration_s"),
    (dict(interval_min_s=-0.1), "interval_min_s"),
    (dict(max_roll_deg=181.0), "max_roll_deg"),
    (dict(servo_channel=0), "servo_channel"),
    (dict(servo_channel=33), "servo_channel"),
    (dict(servo_channel=True), "servo_channel"),
    (dict(feedback_pin=-2), "feedback_pin"),
    (dict(feedback_polarity=2), "feedback_polarity"),
    (dict(feedback_servo_channel=11), "feedback_pin = -1"),
    (dict(feedback_pin=51, feedback_servo_channel=10), "SERVO10"),
    (dict(camera_lead_in_m=-1.0), "camera_lead_in_m"),
    (dict(terrain_retrigger_tol=0.0), "terrain_retrigger_tol"),
    (dict(terrain_retrigger_tol=1.0), "terrain_retrigger_tol"),
    (dict(auto_only=1), "auto_only"),
    (dict(relay_pin=50), "relay_pin"),
    (dict(backend="relay", firmware="4.5.7", relay_pin=50, servo_channel=11), "servo_channel"),
    (dict(backend="relay", firmware="4.5.7", relay_pin=50, aux_servo_channel_for_relay=9, feedback_pin=51,
          feedback_servo_channel=9), "SERVO9"),
    (dict(backend="relay", firmware="4.5.7", relay_pin=50, feedback_pin=50), "feedback_pin"),
])
def test_config_validation(kw, msg):
    with pytest.raises(ValueError, match=msg):
        ap.CameraTriggerConfig(**kw)


def test_config_valid_edges_and_properties():
    assert ap.CameraTriggerConfig(duration_s=5.0, interval_min_s=10.0, max_roll_deg=180.0).cam_type == 1
    cc = ap.CameraTriggerConfig(backend="relay", firmware="V4.8.0-dev", relay_pin=1015)
    assert cc.firmware_version == (4, 8, 0) and cc.cam_type == 2
    assert ap.CameraTriggerConfig(backend="mavlink_v2", firmware="4.5").cam_type == 6
    assert ap.CameraTriggerConfig().firmware_version is None
    with pytest.raises(dataclasses.FrozenInstanceError):
        cc.duration_s = 1.0  # type: ignore[misc]
    with pytest.raises(TypeError):
        ap.plan_to_mission_items(UAVPlan("B1", "B", [], [], 0.0, 0.0, 0.0, "H", 0.0, 0.0, 0.0, 0.0, 0, []), None,
                                 None, camera_cfg={"backend": "relay"})  # type: ignore[arg-type]


# ---------------------------------------------------------------------------
# 匯出／上傳路徑一致
# ---------------------------------------------------------------------------
def test_export_all_param_matches_suggest_params(mini, tmp_path):
    ctx, cfg = mini.ctx, mini.cfg
    cc = ap.CameraTriggerConfig(backend="relay", firmware="4.5.7", relay_pin=50, aux_servo_channel_for_relay=11,
                                interval_min_s=0.56, max_roll_deg=12.0, camera_lead_in_m=40.0)
    s = ap.export_all(mini.mp, ctx, cfg, tmp_path, camera_cfg=cc)
    assert s["camera_cfg"] == dataclasses.asdict(cc)
    for uid in ("A1", "B1"):
        up = mini.mp.plans[uid]
        file_p = ap.read_param_file(tmp_path / f"{uid}.param")
        sug = ml.suggest_params(ctx.specs[up.cls], ctx, cfg, uav_plan=up, uav_id=uid, camera_cfg=cc)
        assert _cam_params(file_p) == pytest.approx(_cam_params(sug)), uid
    b = ap.read_param_file(tmp_path / "B1.param")
    assert _cam_params(b) == {**COMMON, "CAM1_TYPE": 2.0, "CAM1_RELAY_ON": 1.0, "RELAY1_FUNCTION": 4.0,
                              "RELAY1_PIN": 50.0, "SERVO11_FUNCTION": -1.0, "CAM1_INTRVAL_MIN": 0.56,
                              "CAM_MAX_ROLL": 12.0}
    assert _cam_params(ap.read_param_file(tmp_path / "A1.param")) == {"SERVO9_FUNCTION": 0.0}
    text = (tmp_path / "B1.param").read_text(encoding="utf-8")
    assert "# RELAY1_FUNCTION: 4 = Camera" in text and "# SERVO11_FUNCTION: -1 = GPIO" in text
    assert "NOT applied" in (tmp_path / "A1.param").read_text(encoding="utf-8")
    # 匯出之 .waypoints ＝ 同設定之 plan_to_mission_items（含 lead-in）
    back = ap.read_qgc_wpl(tmp_path / "B1.waypoints")
    ref = ap.plan_to_mission_items(mini.mp.plans["B1"], ctx, cfg, camera_cfg=cc)
    assert len(back) == len(ref) == s["uavs"]["B1"]["n_items"]
    for a, r in zip(back, ref):
        assert (a.command, a.frame) == (r.command, r.frame)
        assert (a.param1, a.param2, a.param3) == pytest.approx((r.param1, r.param2, r.param3), abs=1e-6)
        assert (a.x, a.y) == pytest.approx((r.x, r.y), abs=1e-7)
    # 預設（無 camera_cfg）：summary 不多鍵，.param 之 CAM1_DURATION 為 0.1 秒
    s0 = ap.export_all(mini.mp, ctx, cfg, tmp_path / "default")
    assert "camera_cfg" not in s0
    assert _cam_params(ap.read_param_file(tmp_path / "default" / "B1.param")) == \
        {"CAM1_TYPE": 1.0, "CAM1_DURATION": 0.1, "SERVO10_FUNCTION": 10.0}


def test_upload_plan_threads_camera_cfg_to_export(mini, tmp_path, caplog):
    """FleetLink 不帶任何連線 ⇒ 不上傳，只走 export_all（kw_exp 白名單必須含 camera_cfg）。"""
    cc = ap.CameraTriggerConfig(backend="mavlink_v2", firmware="4.6.3", max_roll_deg=10.0)
    fl = ml.FleetLink()
    with caplog.at_level(logging.WARNING, logger="hv_dccpp.mavlink_link"):
        rep = fl.upload_plan(mini.mp, mini.ctx, mini.cfg, out_dir=tmp_path, camera_cfg=cc, z_tol_m=5.0)
    assert rep == {}
    assert _cam_params(ap.read_param_file(tmp_path / "B1.param")) == {**COMMON, "CAM1_TYPE": 6.0, "CAM_MAX_ROLL": 10.0}
    # 沒傳給 export_all 的鍵要明講
    assert any("z_tol_m" in r.getMessage() and "export_all" in r.getMessage() for r in caplog.records)
    assert not any("camera_cfg" in r.getMessage() for r in caplog.records)


def test_call_compat_warns_on_dropped_kwargs(caplog):
    def f(a, *, x=1):
        return a + x

    with caplog.at_level(logging.WARNING, logger="hv_dccpp.mavlink_link"):
        assert ml._call_compat(f, 1, x=2, camera_cfg="relay", bogus=3) == 3
    msgs = [r.getMessage() for r in caplog.records]
    assert len(msgs) == 1 and "bogus" in msgs[0] and "camera_cfg" in msgs[0] and "'x'" not in msgs[0]
    caplog.clear()
    with caplog.at_level(logging.WARNING, logger="hv_dccpp.mavlink_link"):
        assert ml._call_compat(f, 1, x=5) == 6
    assert not caplog.records


def test_fallback_one_metre_distance_warns(mini, caplog):
    a1 = mini.mp.plans["A1"]
    with caplog.at_level(logging.WARNING, logger="hv_dccpp.mavlink_link"):
        items = ml.plan_to_mission_items_fallback(a1, mini.ctx, mini.cfg)
    assert {it.param1 for it in items if it.command == CAM and it.param1 > 0} == {1.0}
    assert any("1.0 m" in r.getMessage() for r in caplog.records)
    caplog.clear()
    with caplog.at_level(logging.WARNING, logger="hv_dccpp.mavlink_link"):
        items = ml.plan_to_mission_items_fallback(a1, mini.ctx, mini.cfg, cam_trigger_dist_m=25.0)
    assert {it.param1 for it in items if it.command == CAM and it.param1 > 0} == {25.0}
    assert not [r for r in caplog.records if "1.0 m" in r.getMessage()]
