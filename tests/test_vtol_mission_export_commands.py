"""VTOLMissionExporter 匯出指令序列的值層級鎖。

背景（2026-08-24）：匯出的任務原本會插入兩個 `MAV_CMD_DO_VTOL_TRANSITION`
(3000)。對照 ArduPilot 官方 QuadPlane AUTO Missions 文件後拿掉：

  * NAV_VTOL_TAKEOFF 到達高度後飛向下一航點時，飛控本來就會 "transitioning
    to fixed wing flight as needed" —— 明確插 param1=4 是多餘的。
  * ArduPlane 4.1+ 的 NAV_VTOL_LAND 預設保持固定翼進場、接近才 airbrake
    轉 VTOL。提早插 param1=3 會強制轉多旋翼，抵銷掉那個省電設計。

這裡鎖的是**輸出的值**（哪些 command 真的出現在檔案裡），不是原始碼長相：
上游、下游或旁路改法都躲不掉。每條斷言都附帶「這個設定真的能讓它發生」的
對照組（explicit_transitions=True），避免變成永遠不可能觸發的儀表。
"""

import pytest

from core.dccpp.dccpp_path_builder import (
    BuiltPath, BuiltWaypoint, DCCPPPathBuilder, SegmentLabel,
)
from mission.vtol_mission_exporter import VTOLMissionExporter

HOME_LAT, HOME_LON = 25.078978, 121.236349
TRANSITION_ALT = 60.0
CRUISE_SPEED = 18.0
MISSION_ALT = 120.0

CMD_NAV_WAYPOINT = 16
CMD_NAV_VTOL_TAKEOFF = 84
CMD_NAV_VTOL_LAND = 85
CMD_DO_CHANGE_SPEED = 178
CMD_DO_SET_HOME = 179
CMD_DO_VTOL_TRANSITION = 3000


# ─────────────────────────────────────────────────────────────
#  夾具：一條真的走過 prepend_vtol_takeoff / append_vtol_landing
#         的路徑（不是手刻航點），確保鎖的是產品路徑
# ─────────────────────────────────────────────────────────────
@pytest.fixture
def vtol_path():
    path = BuiltPath(uav_id=1, waypoints=[
        BuiltWaypoint(lat=HOME_LAT + 0.006, lon=HOME_LON - 0.011,
                      alt=MISSION_ALT, heading_compass_deg=90.0,
                      segment_type=SegmentLabel.OPERATION),
        BuiltWaypoint(lat=HOME_LAT + 0.008, lon=HOME_LON + 0.018,
                      alt=MISSION_ALT, heading_compass_deg=90.0,
                      segment_type=SegmentLabel.OPERATION),
    ])
    builder = DCCPPPathBuilder(min_turn_radius=100.0,
                               default_altitude=MISSION_ALT)
    builder.prepend_vtol_takeoff(path, HOME_LAT, HOME_LON, 0.0,
                                 TRANSITION_ALT, CRUISE_SPEED)
    builder.append_vtol_landing(path, HOME_LAT, HOME_LON, 180.0,
                                TRANSITION_ALT, CRUISE_SPEED)
    return path


def _export(path, explicit_transitions=False):
    exporter = VTOLMissionExporter(transition_alt_m=TRANSITION_ALT,
                                   cruise_speed_ms=CRUISE_SPEED,
                                   turn_radius_m=100.0,
                                   explicit_transitions=explicit_transitions)
    return exporter.generate_waypoint_lines(path.waypoints,
                                            altitude=MISSION_ALT, uav_id=1)


def _items(lines):
    """把 QGC WPL 110 行解析成 dict 列表（跳過表頭）。"""
    out = []
    for line in lines[1:]:
        f = line.rstrip('\n').split('\t')
        out.append(dict(
            seq=int(f[0]), current=int(f[1]), frame=int(f[2]), cmd=int(f[3]),
            p1=float(f[4]), p2=float(f[5]), p3=float(f[6]), p4=float(f[7]),
            x=float(f[8]), y=float(f[9]), z=float(f[10]),
            autocontinue=int(f[11]),
        ))
    return out


def _cmds(lines):
    return [it['cmd'] for it in _items(lines)]


# ─────────────────────────────────────────────────────────────
#  預設行為：不插 DO_VTOL_TRANSITION
# ─────────────────────────────────────────────────────────────
def test_default_export_has_no_vtol_transition(vtol_path):
    assert CMD_DO_VTOL_TRANSITION not in _cmds(_export(vtol_path))


def test_default_sequence_is_takeoff_waypoints_land(vtol_path):
    """預設序列必須是 179 → 178 → 84 → 16×N → 85，中間只有普通航點。"""
    cmds = _cmds(_export(vtol_path))
    assert cmds[0] == CMD_DO_SET_HOME
    assert cmds[1] == CMD_DO_CHANGE_SPEED
    assert cmds[2] == CMD_NAV_VTOL_TAKEOFF
    assert cmds[-1] == CMD_NAV_VTOL_LAND
    assert set(cmds[3:-1]) == {CMD_NAV_WAYPOINT}, cmds


def test_default_takeoff_is_followed_directly_by_a_waypoint(vtol_path):
    """84 之後**立刻**是巡航航點——沒有任何 DO 指令夾在中間。"""
    cmds = _cmds(_export(vtol_path))
    i = cmds.index(CMD_NAV_VTOL_TAKEOFF)
    assert cmds[i + 1] == CMD_NAV_WAYPOINT


def test_default_land_is_preceded_directly_by_a_waypoint(vtol_path):
    """85 之前**立刻**是巡航航點——沒有提早強制轉 MC。"""
    cmds = _cmds(_export(vtol_path))
    i = cmds.index(CMD_NAV_VTOL_LAND)
    assert cmds[i - 1] == CMD_NAV_WAYPOINT


# ─────────────────────────────────────────────────────────────
#  對照組：explicit_transitions=True 時「真的能讓它發生」
#  —— 沒有這組，上面每條斷言都可能是永遠不可能觸發的假綠燈
# ─────────────────────────────────────────────────────────────
def test_explicit_flag_reinstates_both_transitions(vtol_path):
    cmds = _cmds(_export(vtol_path, explicit_transitions=True))
    assert cmds.count(CMD_DO_VTOL_TRANSITION) == 2, cmds


def test_explicit_fw_transition_sits_right_after_takeoff(vtol_path):
    items = _items(_export(vtol_path, explicit_transitions=True))
    i = [it['cmd'] for it in items].index(CMD_NAV_VTOL_TAKEOFF)
    assert items[i + 1]['cmd'] == CMD_DO_VTOL_TRANSITION
    assert items[i + 1]['p1'] == pytest.approx(4.0)   # MAV_VTOL_STATE_FW


def test_explicit_mc_transition_sits_right_before_land(vtol_path):
    items = _items(_export(vtol_path, explicit_transitions=True))
    i = [it['cmd'] for it in items].index(CMD_NAV_VTOL_LAND)
    assert items[i - 1]['cmd'] == CMD_DO_VTOL_TRANSITION
    assert items[i - 1]['p1'] == pytest.approx(3.0)   # MAV_VTOL_STATE_MC


def test_flag_changes_exactly_two_items(vtol_path):
    """兩態之間**只**差那兩筆；差 0 代表開關沒接上，差更多代表順帶改了別的。"""
    off = _cmds(_export(vtol_path))
    on = _cmds(_export(vtol_path, explicit_transitions=True))
    assert len(on) - len(off) == 2
    assert [c for c in on if c != CMD_DO_VTOL_TRANSITION] == off


# ─────────────────────────────────────────────────────────────
#  拿掉指令後最容易靜默壞掉的地方：seq 編號
# ─────────────────────────────────────────────────────────────
@pytest.mark.parametrize('explicit', [False, True])
def test_seq_numbers_stay_contiguous_from_zero(vtol_path, explicit):
    """少發一筆卻沒少加 seq，會讓飛控看到編號跳號 → 任務上傳被拒。"""
    items = _items(_export(vtol_path, explicit_transitions=explicit))
    assert [it['seq'] for it in items] == list(range(len(items)))


@pytest.mark.parametrize('explicit', [False, True])
def test_seq_zero_is_home_and_only_one_current(vtol_path, explicit):
    items = _items(_export(vtol_path, explicit_transitions=explicit))
    assert items[0]['cmd'] == CMD_DO_SET_HOME
    assert sum(it['current'] for it in items) == 0


# ─────────────────────────────────────────────────────────────
#  拿掉指令不該動到剖面本身
# ─────────────────────────────────────────────────────────────
def test_takeoff_altitude_still_the_transition_altitude(vtol_path):
    items = _items(_export(vtol_path))
    to = next(it for it in items if it['cmd'] == CMD_NAV_VTOL_TAKEOFF)
    assert to['z'] == pytest.approx(TRANSITION_ALT)


def test_waypoint_count_unchanged_by_flag(vtol_path):
    off = _cmds(_export(vtol_path))
    on = _cmds(_export(vtol_path, explicit_transitions=True))
    assert (off.count(CMD_NAV_WAYPOINT)
            == on.count(CMD_NAV_WAYPOINT))


def test_default_flag_is_off():
    """預設值本身就是要鎖的東西：有人把 default 改回 True 要立刻紅。"""
    assert VTOLMissionExporter().explicit_transitions is False


def test_summary_text_matches_what_was_emitted(vtol_path, tmp_path):
    """摘要檔不能宣稱有插轉換指令而檔案裡其實沒有（反向也一樣）。"""
    from core.trajectory.dccpp_path_assembler import AssembledPath
    for explicit in (False, True):
        exporter = VTOLMissionExporter(transition_alt_m=TRANSITION_ALT,
                                       cruise_speed_ms=CRUISE_SPEED,
                                       turn_radius_m=100.0,
                                       explicit_transitions=explicit)
        out = tmp_path / f'exp_{explicit}'
        out.mkdir()
        apath = AssembledPath(uav_id=1, waypoints=vtol_path.waypoints)
        exporter.export_all({1: apath}, str(out), altitude=MISSION_ALT,
                            speed=CRUISE_SPEED)
        summaries = list(out.glob('VTOL_briefing.txt'))
        assert summaries, f'沒有產生摘要檔：{list(out.iterdir())}'
        text = summaries[0].read_text(encoding='utf-8')
        claims = 'DO_VTOL_TRANSITION(3000' in text
        assert claims is explicit, (explicit, text)


# ─────────────────────────────────────────────────────────────
#  ⚠ 妥協：上傳飛控是另一條路徑（ui/main_window 內的巢狀函式），
#     沒有可呼叫的接縫，只能鎖原始碼。拿掉它不會讓上面任何值層級
#     測試變紅，但會讓上傳的任務與匯出的檔案剖面不一致。
# ─────────────────────────────────────────────────────────────
def test_upload_path_also_emits_no_transition():
    from pathlib import Path
    import ui.main_window as mw
    src = Path(mw.__file__).read_text(encoding='utf-8')
    assert 'CMD_VTOL_TRANS' not in src, (
        '上傳飛控路徑又插回 DO_VTOL_TRANSITION，會與匯出檔不一致'
    )
