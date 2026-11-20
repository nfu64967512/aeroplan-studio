"""4+1 QuadPlane 起降剖面：幾何推導 + 實際航點形狀。

背景（2026-08-24）
------------------
VTOL 偵察頁面原本規劃出來的是**固定翼跑道起降**：從 Home 沿跑道滑行
``runway_length_m``（預設 500 m）再階梯爬升，面板上的「轉換高度」
**沒有任何地方讀它**（`plan_coverage_dccpp` 連這個參數都沒有）。
使用者在 3D 圖上看到的是一條長斜線，而不是垂直起降。

現在 `vehicle_type=='vtol'` 會走 `prepend_vtol_takeoff` / `append_vtol_landing`：

    Home(地面) ─垂直─> 轉換高度 ─前向過渡─> 固定翼爬升 ─> 巡航高度

這組測試鎖住兩件事：
  1. 幾何推導（VTOLProfile）與我們實際寫進 SITL 的參數一致
  2. 產生的**航點形狀**真的是垂直的，而且會隨面板參數改變
"""

from __future__ import annotations

import math

import pytest

from core.dccpp.dccpp_path_builder import BuiltPath, BuiltWaypoint, DCCPPPathBuilder, SegmentLabel
from core.dccpp.vtol_profile import (
    Q_PILOT_SPD_UP,
    Q_TRANS_DECEL,
    Q_TRANSITION_MS,
    TECS_CLMB_MAX,
    VTOLProfile,
    VTOLProfileError,
)

_R = 6378137.0


def _hav(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * _R * math.asin(math.sqrt(a))


# ══════════════════════════════════════════════════════════════
#  1. 幾何推導
# ══════════════════════════════════════════════════════════════

def test_constants_match_the_params_we_actually_ship():
    """剖面常數必須與 generate_vtol_params() 寫進 SITL 的值一致。

    兩邊漂掉的話，規劃器算的距離就跟飛控實際飛的行為對不上。
    """
    from mission.sitl_launcher import generate_vtol_params
    import tempfile
    from pathlib import Path
    text = generate_vtol_params(Path(tempfile.mkdtemp()) / "v.parm").read_text(
        encoding="utf-8", errors="replace")

    def _val(name):
        for line in text.splitlines():
            line = line.split('#')[0].strip()
            if line.startswith(name + " ") or line.startswith(name + "\t"):
                return float(line.split()[1])
        raise AssertionError(f"vtol_default.parm 找不到 {name}")

    assert _val("Q_TRANSITION_MS") == Q_TRANSITION_MS
    assert _val("Q_TRANS_DECEL") == Q_TRANS_DECEL
    assert _val("Q_PILOT_SPD_UP") == Q_PILOT_SPD_UP
    assert _val("TECS_CLMB_MAX") == TECS_CLMB_MAX


def test_forward_transition_scales_with_speed():
    """前向過渡距離 = ½·v·t，與巡航速度成正比。"""
    slow = VTOLProfile(60.0, 200.0, 20.0)
    fast = VTOLProfile(60.0, 200.0, 40.0)
    assert fast.forward_transition_m == pytest.approx(2 * slow.forward_transition_m)
    assert fast.forward_transition_m == pytest.approx(
        0.5 * 40.0 * (Q_TRANSITION_MS / 1000.0))


def test_back_transition_uses_the_binding_limit():
    """反向過渡取「減速度限制」與「時間限制」的較大者。"""
    p = VTOLProfile(60.0, 200.0, 40.0)
    by_decel = 40.0 ** 2 / (2 * Q_TRANS_DECEL)       # 400 m
    by_time = 0.5 * 40.0 * 3.0                       # 60 m
    assert p.back_transition_m == pytest.approx(max(by_decel, by_time))
    assert p.back_transition_m > p.forward_transition_m, (
        "減速需要的距離應該比加速長"
    )


def test_fw_climb_distance_tracks_altitude_and_speed():
    base = VTOLProfile(60.0, 200.0, 40.0)
    assert base.fw_climb_distance_m == pytest.approx((200 - 60) / TECS_CLMB_MAX * 40.0)
    higher = VTOLProfile(60.0, 300.0, 40.0)
    assert higher.fw_climb_distance_m > base.fw_climb_distance_m
    slower = VTOLProfile(60.0, 200.0, 20.0)
    assert slower.fw_climb_distance_m == pytest.approx(base.fw_climb_distance_m / 2)


def test_higher_transition_alt_shortens_the_fw_climb():
    """轉換高度拉高 ⇒ 固定翼還要再爬的高度變少 ⇒ 爬升段變短。"""
    low = VTOLProfile(60.0, 200.0, 40.0)
    high = VTOLProfile(100.0, 200.0, 40.0)
    assert high.fw_climb_distance_m < low.fw_climb_distance_m


def test_vertical_climb_time_uses_the_multirotor_rate():
    p = VTOLProfile(60.0, 200.0, 40.0)
    assert p.vertical_climb_time_s == pytest.approx(60.0 / Q_PILOT_SPD_UP)


@pytest.mark.parametrize("ta,ca,sp,msg", [
    (0.0, 200.0, 40.0, "轉換高度"),
    (-10.0, 200.0, 40.0, "轉換高度"),
    (60.0, 200.0, 0.0, "巡航速度"),
    (200.0, 100.0, 40.0, "低於轉換高度"),
])
def test_impossible_profiles_are_rejected_loudly(ta, ca, sp, msg):
    """巡航高度低於轉換高度等無解組態必須丟錯，不可默默回傳垃圾幾何。"""
    with pytest.raises(VTOLProfileError, match=msg):
        VTOLProfile(ta, ca, sp)


# ══════════════════════════════════════════════════════════════
#  2. 實際航點形狀
# ══════════════════════════════════════════════════════════════

HOME_LAT, HOME_LON = 25.08, 121.22


def _cruise_stub(alt=200.0):
    """一段假的巡航航點，讓 builder 有東西可以前後接。"""
    return BuiltPath(uav_id=1, waypoints=[
        BuiltWaypoint(lat=HOME_LAT + 0.01, lon=HOME_LON + 0.01, alt=alt,
                      heading_compass_deg=90.0, segment_type=SegmentLabel.OPERATION),
        BuiltWaypoint(lat=HOME_LAT + 0.01, lon=HOME_LON + 0.02, alt=alt,
                      heading_compass_deg=90.0, segment_type=SegmentLabel.OPERATION),
    ])


def _takeoff(trans_alt=60.0, cruise_alt=200.0, speed=40.0, bearing=90.0):
    b = DCCPPPathBuilder(min_turn_radius=100.0, default_altitude=cruise_alt)
    path = _cruise_stub(cruise_alt)
    b.prepend_vtol_takeoff(path, HOME_LAT, HOME_LON, bearing, trans_alt, speed)
    return [w for w in path.waypoints if w.segment_type == SegmentLabel.TAKEOFF]


def _landing(trans_alt=60.0, cruise_alt=200.0, speed=40.0, bearing=180.0):
    b = DCCPPPathBuilder(min_turn_radius=100.0, default_altitude=cruise_alt)
    path = _cruise_stub(cruise_alt)
    b.append_vtol_landing(path, HOME_LAT, HOME_LON, bearing, trans_alt, speed)
    return [w for w in path.waypoints if w.segment_type == SegmentLabel.LANDING]


def test_takeoff_first_leg_is_purely_vertical():
    """這是 4+1 VTOL 與固定翼跑道起飛的分野：前兩點必須同經緯度。"""
    w = _takeoff()
    assert w[0].alt == 0.0
    horiz = _hav(w[0].lat, w[0].lon, w[1].lat, w[1].lon)
    assert horiz == pytest.approx(0.0, abs=1e-6), (
        f"起飛第一段水平位移 {horiz:.3f} m —— 應為純垂直爬升"
    )
    assert w[1].alt == pytest.approx(60.0)


def test_takeoff_vertical_height_follows_the_transition_alt_setting():
    """面板改「轉換高度」，垂直段高度就要跟著改。"""
    for ta in (40.0, 60.0, 120.0):
        w = _takeoff(trans_alt=ta)
        assert w[1].alt == pytest.approx(ta)
        assert _hav(w[0].lat, w[0].lon, w[1].lat, w[1].lon) == pytest.approx(0.0, abs=1e-6)


def test_takeoff_transition_leg_is_level_and_scales_with_speed():
    """[1]→[2] 是前向過渡：高度不變、長度隨速度變。"""
    for sp in (20.0, 40.0):
        w = _takeoff(speed=sp)
        assert w[2].alt == pytest.approx(w[1].alt), "過渡段不該改變高度"
        d = _hav(w[1].lat, w[1].lon, w[2].lat, w[2].lon)
        expect = VTOLProfile(60.0, 200.0, sp).forward_transition_m
        assert d == pytest.approx(expect, rel=1e-3)


def test_takeoff_reaches_cruise_altitude():
    for ca in (150.0, 200.0, 300.0):
        w = _takeoff(cruise_alt=ca)
        assert w[-1].alt == pytest.approx(ca, rel=1e-6)
        assert all(w[i].alt <= w[i+1].alt + 1e-9 for i in range(len(w)-1)), "高度必須單調不降"


def test_takeoff_climb_distance_responds_to_altitude():
    """航高拉高 ⇒ 固定翼爬升段變長（不是寫死的 runway_length）。"""
    def climb_len(ca):
        w = _takeoff(cruise_alt=ca)
        return _hav(w[2].lat, w[2].lon, w[-1].lat, w[-1].lon)
    assert climb_len(300.0) > climb_len(200.0) > climb_len(150.0)


def test_landing_last_leg_is_purely_vertical():
    w = _landing()
    assert w[-1].alt == 0.0
    horiz = _hav(w[-2].lat, w[-2].lon, w[-1].lat, w[-1].lon)
    assert horiz == pytest.approx(0.0, abs=1e-6), (
        f"降落最後一段水平位移 {horiz:.3f} m —— 應為純垂直下降"
    )
    assert w[-2].alt == pytest.approx(60.0)


def test_landing_touches_down_at_home():
    w = _landing()
    assert w[-1].lat == pytest.approx(HOME_LAT, abs=1e-9)
    assert w[-1].lon == pytest.approx(HOME_LON, abs=1e-9)


def test_landing_altitude_is_monotonically_descending():
    w = _landing()
    assert all(w[i].alt >= w[i+1].alt - 1e-9 for i in range(len(w)-1)), "高度必須單調不升"


def test_vtol_takeoff_is_not_the_runway_takeoff():
    """刪除實驗式對照：固定翼版第一段一定有水平位移，VTOL 版一定沒有。

    若哪天 VTOL 分支被接錯回固定翼版，這條會紅。
    """
    b = DCCPPPathBuilder(min_turn_radius=100.0, default_altitude=200.0)
    fw = _cruise_stub()
    b.prepend_takeoff(fw, HOME_LAT, HOME_LON, 90.0, climb_distance_m=500.0)
    fw_wps = [w for w in fw.waypoints if w.segment_type == SegmentLabel.TAKEOFF]
    fw_first = _hav(fw_wps[0].lat, fw_wps[0].lon, fw_wps[1].lat, fw_wps[1].lon)
    assert fw_first > 50.0, "固定翼起飛第一段本來就該有水平位移"

    vt = _takeoff()
    vt_first = _hav(vt[0].lat, vt[0].lon, vt[1].lat, vt[1].lon)
    assert vt_first == pytest.approx(0.0, abs=1e-6)


# ══════════════════════════════════════════════════════════════
#  3. 垂直段不能弄壞下游（消費端回歸）
# ══════════════════════════════════════════════════════════════
# 2026-08-24 工作流指出：VTOL 起飛第一段是純垂直，wps[0] 與 wps[1] 同經緯度。
# 任何「只比第二點就算方位角」的地方都會靜靜退回 0°。

def test_spawn_heading_scan_skips_the_vertical_leg():
    """SITL 生成朝向必須往後掃到第一個有水平位移的點。

    只看 waypoints[1] 的話，VTOL 的垂直起飛會讓方位角退回 0°，
    SITL 就會把飛機生在錯的朝向上。
    """
    import math as _m
    w = _takeoff(bearing=90.0)          # 朝東起飛
    u_lat, u_lon = float(w[0].lat), float(w[0].lon)

    # 舊寫法：只看第二點
    dN = (float(w[1].lat) - u_lat) * 111320.0
    dE = (float(w[1].lon) - u_lon) * 111320.0 * _m.cos(_m.radians(u_lat))
    assert abs(dN) + abs(dE) <= 1e-6, "垂直段本來就該沒有水平位移"

    # 新寫法：往後掃
    hdg = 0.0
    for nxt in w[1:]:
        dN = (float(nxt.lat) - u_lat) * 111320.0
        dE = (float(nxt.lon) - u_lon) * 111320.0 * _m.cos(_m.radians(u_lat))
        if abs(dN) + abs(dE) > 1e-6:
            hdg = (_m.degrees(_m.atan2(dE, dN))) % 360.0
            break
    assert hdg == pytest.approx(90.0, abs=1.0), (
        f"往後掃應得到起飛方位 90°，實際 {hdg:.1f}°"
    )


def test_mainwindow_uses_a_forward_scan_for_dccpp_spawn_heading():
    """守衛實際的程式碼：DCCPP 分支不可只比 waypoints[1]。"""
    from pathlib import Path
    src = (Path(__file__).resolve().parent.parent / "ui" / "main_window.py").read_text(
        encoding="utf-8", errors="replace")
    assert "for nxt in wps_u[1:]:" in src, (
        "ui/main_window.py 的 DCCPP 起飛朝向計算沒有往後掃，"
        "VTOL 垂直起飛會讓朝向退回 0°"
    )


def test_zero_length_leg_is_safe_for_the_coverage_verifier():
    """覆蓋率驗算的點到折線距離必須容得下零長度線段。"""
    from core.recon.coverage_verifier import _min_dist_to_polyline
    import numpy as np
    pts = np.array([[0.0, 0.0], [10.0, 0.0]])
    line = np.array([[5.0, 5.0], [5.0, 5.0]])      # 退化成一個點
    d = _min_dist_to_polyline(pts, line)
    assert np.all(np.isfinite(d)), "零長度線段讓距離變成 NaN/inf"
    assert d[0] == pytest.approx(math.hypot(5.0, 5.0))


def test_takeoff_head_split_exempts_the_vertical_leg():
    """NFZ 避障必須豁免起飛段，否則垂直爬升會被 2D 弧長重採樣擠掉。

    `_resample_alts_by_arclength` 用**水平**弧長比例重採樣高度；垂直段的
    水平弧長是 0，佔比也是 0，整段爬升會消失。
    """
    from ui.main_window import MainWindow
    w = _takeoff()          # 7~9 個 TAKEOFF 點，前兩點同經緯度
    tail = [BuiltWaypoint(lat=HOME_LAT + 0.05, lon=HOME_LON + 0.05, alt=200.0,
                          heading_compass_deg=90.0, segment_type=SegmentLabel.OPERATION)]
    head, rest = MainWindow._split_takeoff_head(w + tail)
    assert head, "起飛段沒有被剝離"
    # 垂直對（地面點 + 轉換高度點）必須留在豁免段
    assert head[0].alt == 0.0
    assert any(abs(x.alt - 60.0) < 1e-6 for x in head), "轉換高度點不在豁免段"
    # 爬升完成點留在避障段，離場巡航才會繞 NFZ
    assert rest[0].segment_type == SegmentLabel.TAKEOFF
    assert rest[0].alt == pytest.approx(200.0)
    # 不可弄丟或重複航點
    assert len(head) + len(rest) == len(w) + len(tail)


def test_takeoff_head_split_is_a_noop_without_takeoff_segments():
    from ui.main_window import MainWindow
    only_op = _cruise_stub()
    head, rest = MainWindow._split_takeoff_head(only_op.waypoints)
    assert head == []
    assert len(rest) == len(only_op.waypoints)


def test_collision_avoidance_never_lifts_the_ground_point():
    """避撞的高度偏移不可動到起降段 —— 地面點被抬高等於「在半空中降落」。"""
    from core.dccpp.collision_avoidance import CollisionAvoidance
    from core.dccpp.dccpp_path_builder import BuiltPath as _BP

    def _mk(uid, dlat):
        b = DCCPPPathBuilder(min_turn_radius=100.0, default_altitude=200.0)
        p = _BP(uav_id=uid, waypoints=[
            BuiltWaypoint(lat=HOME_LAT + dlat, lon=HOME_LON + 0.001 * i, alt=200.0,
                          heading_compass_deg=90.0, segment_type=SegmentLabel.OPERATION)
            for i in range(30)])
        b.prepend_vtol_takeoff(p, HOME_LAT + dlat, HOME_LON, 90.0, 60.0, 40.0)
        b.append_vtol_landing(p, HOME_LAT + dlat, HOME_LON, 180.0, 60.0, 40.0)
        return p

    paths = {1: _mk(1, 0.0), 2: _mk(2, 1e-5)}     # 幾乎重疊 → 一定有衝突
    ca = CollisionAvoidance(min_separation_m=50.0, alt_offset_m=60.0,
                            cruise_speed_mps=40.0)
    ca.run(paths)
    for uid, bp in paths.items():
        for w in bp.waypoints:
            if w.segment_type in (SegmentLabel.TAKEOFF, SegmentLabel.LANDING):
                assert w.alt <= 200.0 + 1e-6, (
                    f"UAV{uid} 的起降段航點被抬到 {w.alt:.1f} m"
                )
        ground = [w for w in bp.waypoints
                  if w.segment_type in (SegmentLabel.TAKEOFF, SegmentLabel.LANDING)
                  and w is bp.waypoints[0] or w is bp.waypoints[-1]]
        for g in ground:
            assert g.alt == pytest.approx(0.0, abs=1e-9), (
                f"UAV{uid} 的地面點高度變成 {g.alt:.2f} m —— 會變成半空中起降"
            )


def test_split_takeoff_head_is_lossless():
    """拆開再接回必須完全還原 —— 拆分本身不可弄丟航點。"""
    from ui.main_window import MainWindow
    w = _takeoff()
    tail = [BuiltWaypoint(lat=HOME_LAT + 0.05, lon=HOME_LON + 0.05, alt=200.0,
                          heading_compass_deg=90.0, segment_type=SegmentLabel.OPERATION)]
    full = w + tail
    head, rest = MainWindow._split_takeoff_head(full)
    assert head + rest == full


def test_nfz_avoidance_reattaches_the_exempt_takeoff_head():
    """避障後必須把豁免的起飛段接回去。

    ⚠ 這是**源碼層級**檢查（比值層級弱），因為要走到那行得先組出完整的
    NFZ 避障場景。之所以還是留著：把 `takeoff_head +` 拿掉並不會讓任何
    值層級測試變紅，但會讓整段起飛航點**憑空消失** —— 沒有守衛更糟。
    若日後有辦法用值層級覆蓋這條路徑，應該換掉它。
    """
    from pathlib import Path
    src = (Path(__file__).resolve().parent.parent / "ui" / "main_window.py").read_text(
        encoding="utf-8", errors="replace")
    assert "takeoff_head, _rest_wps = self._split_takeoff_head(" in src, (
        "NFZ 避障沒有剝離起飛段，垂直爬升會被 2D 弧長重採樣擠掉"
    )
    assert "apath.waypoints = takeoff_head + new_wps + landing_tail" in src, (
        "避障後沒有把豁免的起飛段接回去 —— 起飛航點會整段消失"
    )


# ══════════════════════════════════════════════════════════════
#  4. 長度與時間帳（VTOL 專屬）
# ══════════════════════════════════════════════════════════════
# 2026-08-24 突變測試發現：管線 fixture 唯一的固定翼走的是跑道起飛，
# 所以 VTOL 路徑的長度累加與垂直段計時**完全沒有測試覆蓋** ——
# 把 takeoff_length_m 的累加拿掉、或把垂直段改用巡航速度計時，都能全綠。

def _built_with_vtol_ends(trans_alt=60.0, cruise_alt=200.0, speed=40.0):
    b = DCCPPPathBuilder(min_turn_radius=100.0, default_altitude=cruise_alt)
    path = _cruise_stub(cruise_alt)
    path.operation_length_m = 1000.0          # 假裝有作業段長度
    b.prepend_vtol_takeoff(path, HOME_LAT, HOME_LON, 90.0, trans_alt, speed)
    b.append_vtol_landing(path, HOME_LAT, HOME_LON, 180.0, trans_alt, speed)
    return path


def test_vtol_takeoff_and_landing_lengths_are_accumulated():
    """VTOL 起降段必須把注入的航跡長度記進累加欄位。

    不記的話 total_length_m 少算整段起降，makespan 就會低估
    （這正是畫面上 makespan 比最慢那架還短的成因）。
    """
    p = _built_with_vtol_ends()
    assert p.takeoff_length_m > 0.0, "VTOL 起飛長度沒有被累加"
    assert p.landing_length_m > 0.0, "VTOL 降落長度沒有被累加"
    # 起飛段至少包含垂直爬升 + 前向過渡
    prof = VTOLProfile(60.0, 200.0, 40.0)
    assert p.takeoff_length_m >= prof.transition_alt_m + prof.forward_transition_m - 1.0


def test_total_length_includes_the_vtol_ends():
    p = _built_with_vtol_ends()
    assert p.total_length_m == pytest.approx(
        p.entry_length_m + p.operation_length_m + p.transfer_length_m
        + p.takeoff_length_m + p.landing_length_m)
    assert p.total_length_m > p.operation_length_m, "總長沒有把起降算進去"


def test_vertical_leg_is_timed_at_the_climb_rate_not_cruise_speed():
    """60 m 垂直爬升在 2.5 m/s 下要 24 秒，不是 60/40 = 1.5 秒。

    用巡航速度計時會把懸停爬升低估 16 倍。
    """
    from core.dccpp.dccpp_path_builder import estimate_flight_time_s
    p = _built_with_vtol_ends(trans_alt=60.0, speed=40.0)
    t_correct = estimate_flight_time_s(p, cruise_speed_mps=40.0, vertical_speed_mps=2.5)
    t_wrong = p.total_length_m / 40.0          # 全部用巡航速度
    # 起降各 60 m 垂直 ⇒ 兩段共 120 m，正確 48 s vs 錯誤 3 s，差 45 s
    assert t_correct - t_wrong == pytest.approx(120.0 / 2.5 - 120.0 / 40.0, abs=1.0), (
        f"垂直段計時不對：correct={t_correct:.1f}s wrong={t_wrong:.1f}s"
    )


def test_slower_climb_rate_makes_the_mission_longer():
    """爬升率是實際變數 —— 調慢它，任務時間必須變長。"""
    from core.dccpp.dccpp_path_builder import estimate_flight_time_s
    p = _built_with_vtol_ends()
    fast = estimate_flight_time_s(p, 40.0, vertical_speed_mps=5.0)
    slow = estimate_flight_time_s(p, 40.0, vertical_speed_mps=1.0)
    assert slow > fast + 10.0, "爬升率沒有影響任務時間"


def test_higher_transition_alt_costs_more_hover_time():
    """轉換高度越高，垂直爬升越久。"""
    from core.dccpp.dccpp_path_builder import estimate_flight_time_s
    low = estimate_flight_time_s(_built_with_vtol_ends(trans_alt=40.0), 40.0, 2.5)
    high = estimate_flight_time_s(_built_with_vtol_ends(trans_alt=120.0), 40.0, 2.5)
    assert high > low


# ═══════════════════════════════════════════════════════════════════
#  爬升角來源:機體 vs 飛控
# ═══════════════════════════════════════════════════════════════════
# ⚠️⚠️ 飛控的 TECS_CLMB_MAX 是**能力上限**,機體航跡角才是**實際飛得出來的**。
#   拿上限當實際值會把固定翼爬升的水平距離少算約 4 倍。
#   這一節鎖住:(a) 兩個來源確實差很多,(b) from_airframe 真的帶進機體的值,
#   (c) 不給角度時行為**完全不變**(舊呼叫端不受影響)。

import math as _math

from core.base.airframe import HERO_PNP, PIONEER_PNP


def test_the_two_climb_angle_sources_really_differ():
    """可觸發性:先證明兩個來源算出來差很多,否則下面的斷言驗不到東西。"""
    fc = VTOLProfile(transition_alt_m=100.0, cruise_alt_m=200.0,
                     cruise_speed_mps=21.5)
    af = VTOLProfile.from_airframe(PIONEER_PNP, cruise_alt_m=200.0)
    assert fc.climb_angle_source == "flight-controller"
    assert af.climb_angle_source == "airframe"
    ratio = af.fw_climb_distance_m / fc.fw_climb_distance_m
    assert ratio > 3.5, (
        f"兩個來源的爬升距離只差 {ratio:.2f} 倍 —— 這一節失去意義,"
        "要嘛 TECS_CLMB_MAX 變了,要嘛機體爬升角變了"
    )


def test_from_airframe_pulls_everything_from_the_record():
    p = VTOLProfile.from_airframe(PIONEER_PNP, cruise_alt_m=200.0)
    assert p.cruise_speed_mps == PIONEER_PNP.cruise_airspeed_mps
    assert p.climb_path_angle_deg == PIONEER_PNP.max_climb_path_angle_deg
    assert p.descent_path_angle_deg == PIONEER_PNP.max_descent_path_angle_deg
    assert p.transition_alt_m == PIONEER_PNP.transition_complete_agl_m
    assert p.climb_angle_deg == pytest.approx(3.0)


def test_airframe_climb_distance_is_h_over_tan_gamma():
    """機體路徑用 h/tan γ,不是 h/爬升率×速度。"""
    p = VTOLProfile.from_airframe(PIONEER_PNP, cruise_alt_m=200.0)
    h = p.climb_height_m
    assert p.fw_climb_distance_m == pytest.approx(
        h / _math.tan(_math.radians(3.0)), rel=1e-9)
    assert p.fw_descent_distance_m == pytest.approx(
        h / _math.tan(_math.radians(4.0)), rel=1e-9)
    # 下降比爬升陡 ⇒ 下降佔地較短
    assert p.fw_descent_distance_m < p.fw_climb_distance_m


def test_overrides_can_beat_the_record():
    p = VTOLProfile.from_airframe(PIONEER_PNP, cruise_alt_m=200.0,
                                  transition_alt_m=60.0, cruise_speed_mps=25.0)
    assert p.transition_alt_m == 60.0
    assert p.cruise_speed_mps == 25.0
    # 角度仍來自機體
    assert p.climb_angle_deg == pytest.approx(3.0)


def test_hero_needs_more_takeoff_distance_than_pioneer():
    """非直覺但正確:較小的 HERO 起飛佔地**更長**。

    因為它的轉換完成高度較低(90 vs 100 m),要用同一個 3° 爬更多高度到
    同一個巡航層。這正是「逐機」才算得出來的東西 —— 用全隊共用值會看不到。
    """
    pio = VTOLProfile.from_airframe(PIONEER_PNP, cruise_alt_m=200.0)
    her = VTOLProfile.from_airframe(HERO_PNP, cruise_alt_m=200.0)
    assert her.transition_alt_m < pio.transition_alt_m
    assert her.takeoff_ground_distance_m > pio.takeoff_ground_distance_m


def test_no_angle_means_behaviour_is_exactly_unchanged():
    """不給角度時必須跟加這個功能之前**完全一樣**,舊呼叫端才不受影響。"""
    p = VTOLProfile(transition_alt_m=60.0, cruise_alt_m=200.0,
                    cruise_speed_mps=40.0)
    assert p.climb_angle_source == "flight-controller"
    assert p.fw_climb_distance_m == pytest.approx(
        (200 - 60) / TECS_CLMB_MAX * 40.0)
    assert p.climb_angle_deg == pytest.approx(
        _math.degrees(_math.atan2(TECS_CLMB_MAX, 40.0)))


@pytest.mark.parametrize("bad", [0.0, 90.0, -3.0, 120.0])
def test_invalid_path_angle_is_rejected(bad):
    with pytest.raises(VTOLProfileError, match=r"不在 \(0, 90\)"):
        VTOLProfile(transition_alt_m=60.0, cruise_alt_m=200.0,
                    cruise_speed_mps=20.0, climb_path_angle_deg=bad)


def test_from_airframe_without_any_transition_altitude_raises():
    from dataclasses import replace
    naked = replace(PIONEER_PNP, transition_start_agl_m=None,
                    transition_complete_agl_m=None)
    with pytest.raises(VTOLProfileError, match="沒有轉換高度"):
        VTOLProfile.from_airframe(naked, cruise_alt_m=200.0)
