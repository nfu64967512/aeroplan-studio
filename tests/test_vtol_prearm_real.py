"""真實開發模擬：三個 PreArm 擋解鎖項的值層級鎖。

背景（2026-08-25 實飛量到，靠 force-arm 21196 才繞過去的三條）：

    PreArm: VTOL land too short, min 108m
    PreArm: Gyro 0 rate 400Hz < loop rate*1.8 540Hz
    PreArm: Main loop slow (267Hz < 300Hz)

真機上前兩條會直接擋住解鎖。修法原則是「讓檢查真的通過」而不是關掉檢查：

  1. 降落進場段：飛控用**它自己的** AIRSPEED_CRUISE(24) 算煞停距離
     （24²/(2×Q_TRANS_DECEL=2)=144 m，PreArm 門檻 108 = 0.75×144），跟面板
     速度無關。規劃器新增 `land_approach_m`，且匯出/上傳都不再於
     NAV_VTOL_LAND 前塞同座標航點（那會把進場段變 0 m）。
  2+3. `_SCHED_LOOP_RATE['VTOL']` 300→200：SITL 陀螺儀 400 Hz，300×1.8=540>400
     恆擋；200×1.8=360≤400，且實測本機只跑得動 ~267 Hz。
  4. 解鎖改「先走真 PreArm，被擋才 force + 大聲警告」——force 不再是預設路徑。
"""

import math

import pytest

from core.dccpp.dccpp_path_builder import (
    BuiltPath, BuiltWaypoint, DCCPPPathBuilder, SegmentLabel,
)
from core.dccpp.vtol_profile import (
    AIRSPEED_CRUISE, Q_TRANS_DECEL, VTOLProfile,
)
from mission.vtol_mission_exporter import VTOLMissionExporter

HOME_LAT, HOME_LON = 25.077923, 121.231959

#: 飛控 PreArm「VTOL land too short」實測門檻（min 108m = 0.75 × 煞停 144 m）
FC_PREARM_MIN_M = 108.0


def _haversine_m(lat1, lon1, lat2, lon2):
    R = 6378137.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def _built_path(cruise=22.0, transition=40.0, alt=120.0):
    path = BuiltPath(uav_id=1, waypoints=[
        BuiltWaypoint(lat=HOME_LAT + 0.006, lon=HOME_LON - 0.011, alt=alt,
                      heading_compass_deg=90.0, segment_type=SegmentLabel.OPERATION),
        BuiltWaypoint(lat=HOME_LAT + 0.008, lon=HOME_LON + 0.018, alt=alt,
                      heading_compass_deg=90.0, segment_type=SegmentLabel.OPERATION),
    ])
    b = DCCPPPathBuilder(min_turn_radius=100.0, default_altitude=alt)
    b.prepend_vtol_takeoff(path, HOME_LAT, HOME_LON, 0.0, transition, cruise)
    b.append_vtol_landing(path, HOME_LAT, HOME_LON, 180.0, transition, cruise)
    return path


def _export_items(path, transition=40.0, cruise=22.0):
    lines = VTOLMissionExporter(transition_alt_m=transition, cruise_speed_ms=cruise,
                                turn_radius_m=100.0).generate_waypoint_lines(
        path.waypoints, altitude=120.0, uav_id=1)
    out = []
    for ln in lines[1:]:
        f = ln.rstrip('\n').split('\t')
        out.append(dict(cmd=int(f[3]), lat=float(f[8]), lon=float(f[9]),
                        alt=float(f[10])))
    return out


# ═════════════════════════════════════════════════════════════════════
#  修正 1：降落進場段長度
# ═════════════════════════════════════════════════════════════════════
def test_land_approach_uses_fc_airspeed_not_panel_speed():
    """面板速度 18 的煞停只有 81 m —— land_approach 必須用飛控的 24 算。"""
    prof = VTOLProfile(transition_alt_m=40.0, cruise_alt_m=120.0,
                       cruise_speed_mps=18.0)
    fc_stop = AIRSPEED_CRUISE ** 2 / (2.0 * Q_TRANS_DECEL)
    assert prof.back_transition_m < FC_PREARM_MIN_M, \
        '前提：面板速度算出來的距離本來就過不了門檻，否則這條測不到東西'
    assert prof.land_approach_m > fc_stop, \
        f'進場段 {prof.land_approach_m} 沒有超過飛控煞停距離 {fc_stop}'


@pytest.mark.parametrize('panel_speed', [18.0, 20.0, 22.0, 28.0, 34.0])
def test_land_approach_clears_prearm_at_any_panel_speed(panel_speed):
    prof = VTOLProfile(transition_alt_m=40.0, cruise_alt_m=120.0,
                       cruise_speed_mps=panel_speed)
    assert prof.land_approach_m >= FC_PREARM_MIN_M * 1.2


def test_exported_final_approach_leg_clears_prearm():
    """真正被飛控檢查的量：匯出檔裡「NAV_VTOL_LAND 的前一點 → 降落點」距離。"""
    items = _export_items(_built_path())
    cmds = [it['cmd'] for it in items]
    i = cmds.index(85)
    land, prev = items[i], items[i - 1]
    assert prev['cmd'] == 16
    d = _haversine_m(prev['lat'], prev['lon'], land['lat'], land['lon'])
    assert d >= FC_PREARM_MIN_M, \
        f'進場段只有 {d:.1f} m < PreArm 門檻 {FC_PREARM_MIN_M} m ⇒ 真機拒絕解鎖'


def test_exporter_skips_the_hover_over_land_point():
    """NAV_VTOL_LAND 前不可以有「同座標 @ 轉換高度」的懸停點（進場段會變 0 m）。"""
    items = _export_items(_built_path())
    cmds = [it['cmd'] for it in items]
    i = cmds.index(85)
    land = items[i]
    for it in items[:i]:
        if it['cmd'] != 16:
            continue
        same = (abs(it['lat'] - land['lat']) < 1e-9
                and abs(it['lon'] - land['lon']) < 1e-9)
        assert not same, f'發現降落點正上方的多餘航點 alt={it["alt"]}'


def test_planner_still_keeps_hover_point_for_display():
    """對照組：規劃器的航點裡**仍保留**懸停點（3D 顯示的垂直降落段要用），
    只是匯出時跳過 —— 沒有這條，上面那條可能只是因為點根本不存在而假綠。"""
    path = _built_path()
    landing = [w for w in path.waypoints
               if getattr(w.segment_type, 'name', '') == 'LANDING']
    hover = [w for w in landing
             if abs(w.lat - HOME_LAT) < 1e-9 and abs(w.lon - HOME_LON) < 1e-9
             and w.alt > 1.0]
    assert hover, '規劃器的懸停點不見了 —— 3D 垂直降落段會消失'


def test_landing_ground_distance_uses_land_approach():
    prof = VTOLProfile(transition_alt_m=40.0, cruise_alt_m=120.0,
                       cruise_speed_mps=18.0)
    assert prof.landing_ground_distance_m == pytest.approx(
        prof.fw_descent_distance_m + prof.land_approach_m)


# ═════════════════════════════════════════════════════════════════════
#  修正 2+3：主迴圈 / 陀螺儀取樣率
# ═════════════════════════════════════════════════════════════════════
SITL_GYRO_HZ = 400.0        # SITL 模擬 IMU 的陀螺儀取樣率（PreArm 訊息實測）
GYRO_RATIO = 1.8            # PreArm 要求 gyro >= loop × 1.8
HOST_SUSTAINED_HZ = 267.0   # 本機實測撐得住的主迴圈（PreArm 訊息實測）


def test_vtol_loop_rate_satisfies_gyro_prearm():
    from mission.sitl_launcher import _SCHED_LOOP_RATE
    loop = _SCHED_LOOP_RATE['VTOL']
    assert loop * GYRO_RATIO <= SITL_GYRO_HZ, \
        f'loop {loop}×{GYRO_RATIO}={loop*GYRO_RATIO} > 陀螺儀 {SITL_GYRO_HZ} ⇒ PreArm 恆擋'


def test_vtol_loop_rate_is_sustainable_on_this_host():
    from mission.sitl_launcher import _SCHED_LOOP_RATE
    assert _SCHED_LOOP_RATE['VTOL'] <= HOST_SUSTAINED_HZ, \
        'loop rate 超過本機實測能力 ⇒ PreArm「Main loop slow」恆擋'


def test_old_rate_really_violates_both():
    """對照組：舊值 300 兩條都過不了 —— 證明上面兩條不是恆真。"""
    assert 300 * GYRO_RATIO > SITL_GYRO_HZ
    assert 300 > HOST_SUSTAINED_HZ


def test_identity_parm_carries_the_vtol_rate():
    from mission.sitl_launcher import identity_parm_text, _SCHED_LOOP_RATE
    txt = identity_parm_text('VTOL', 1)
    assert f"SCHED_LOOP_RATE {_SCHED_LOOP_RATE['VTOL']}" in txt.replace('\t', ' ') \
        or str(_SCHED_LOOP_RATE['VTOL']) in txt


# ═════════════════════════════════════════════════════════════════════
#  修正 4：先真解鎖，force 只是大聲的後備
# ═════════════════════════════════════════════════════════════════════
class _FakeMavlink:
    """只記錄 command_long_send 的 param2（0=正常解鎖、21196=force）。"""

    def __init__(self):
        self.arm_param2 = []

    def command_long_send(self, *a):
        # (sys, comp, cmd, conf, p1..p7)
        if a[2] == 400:                      # MAV_CMD_COMPONENT_ARM_DISARM
            self.arm_param2.append(a[5])     # param2


class _FakeConn:
    def __init__(self, arms_normally):
        self.mav = _FakeMavlink()
        self.target_system, self.target_component = 1, 1
        self._arms = arms_normally
        self._n = 0

    def recv_match(self, type=None, blocking=True, timeout=None):
        self._n += 1
        if self._n > 8:
            return None

        class _HB:
            def get_type(self):
                return 'HEARTBEAT'
        hb = _HB()
        hb.base_mode = 128 if self._arms else 0    # MAV_MODE_FLAG_SAFETY_ARMED
        return hb


def _link_with(conn):
    """不啟動 QThread、不連線 —— 只借 _arm_with_prearm 的函式本體。"""
    from mission.sitl_link import SITLLink
    link = SITLLink.__new__(SITLLink)
    link._mav = conn

    class _Sig:
        def emit(self, *a):
            pass
    link.status_text = _Sig()
    return link


def test_normal_arm_success_never_forces():
    conn = _FakeConn(arms_normally=True)
    link = _link_with(conn)
    assert link._arm_with_prearm(force_fallback=True, wait_s=0.5) is True
    assert conn.mav.arm_param2 == [0], \
        f'應該只發一次正常解鎖(param2=0)，實際 {conn.mav.arm_param2}'


def test_blocked_arm_falls_back_to_force_with_warning():
    conn = _FakeConn(arms_normally=False)
    link = _link_with(conn)
    assert link._arm_with_prearm(force_fallback=True, wait_s=0.5) is True
    assert conn.mav.arm_param2 == [0, 21196], \
        '被擋時應先試正常(0)再 force(21196)'


def test_blocked_arm_without_fallback_stays_blocked():
    conn = _FakeConn(arms_normally=False)
    link = _link_with(conn)
    assert link._arm_with_prearm(force_fallback=False, wait_s=0.5) is False
    assert conn.mav.arm_param2 == [0], 'force_fallback=False 不可以偷偷 force'


def test_no_arming_check_superstition_left():
    """ARMING_CHECK 在 4.6+ 已移除 —— 對它 param_set 是無作用的迷信碼，
    留著只會讓人誤以為檢查被關掉了。鎖住不許回來。"""
    from pathlib import Path
    import mission.sitl_link as sl
    src = Path(sl.__file__).read_text(encoding='utf-8')
    assert "b'ARMING_CHECK'" not in src


def test_upload_landing_branch_has_no_pre_land_waypoint():
    """⚠ 源碼層級妥協（上傳路徑是巢狀函式）：NAV_VTOL_LAND 前不可再塞
    「降落點上空 @ 轉換高度」的 NAV_WAYPOINT。"""
    from pathlib import Path
    import ui.main_window as mw
    src = Path(mw.__file__).read_text(encoding='utf-8')
    assert '階段 1: 先飛到降落點上空' not in src
