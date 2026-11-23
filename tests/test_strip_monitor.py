"""survey/strip_monitor.py：條帶段表與即時張數監測（A5）的值鎖。

段表用**真的** HARVEST 建圖任務（tests/test_hdmap_gui_adapter 的情境：1.2 × 1.0 km、1 Pioneer + 2 HERO、HQ6、
引入段 50 m）。監測器用「飛前模擬照片」重播成事件流（206 開 → 每張照片 → 206 關），再注入故障：
0 張、拍攝中離開 AUTO 又直接接續、照 SOP 倒帶、跳降落序列、STATUSTEXT 全掉只剩 MISSION_CURRENT、合併回授、
遙測中斷、帶外照片。不碰 Qt、不開任何連線。
"""
from __future__ import annotations

import collections
import math

import pytest

from survey import strip_monitor as SM
from survey.strip_monitor import CamSegment, StripPhotoMonitor, camera_segments, expected_by_segment

T = pytest.importorskip("tests.test_hdmap_gui_adapter")

from hv_dccpp.ardupilot import MAV_CMD_DO_LAND_START, MAV_CMD_DO_SET_CAM_TRIGG_DIST  # noqa: E402
from hv_dccpp.mission import MappingModeConfig  # noqa: E402


@pytest.fixture(scope="module")
def mission():
    """(uid, items, plan 的條帶序, 飛前模擬照片) —— 第一架有任務的機。"""
    hv = T._plan(T._coord(MappingModeConfig(lead_in_m=T.LEAD), camera_cfg=T.CC))
    did = T._flying(hv)[0]
    uid = hv.id_map[did]
    items = hv.mission_items(did, camera_cfg=T.CC)
    from core.recon.exposure_verifier import simulate_mission_photos
    spd = [it["param2"] for it in items if it["command"] == 178 and it["param1"] == 0.0 and it["param2"] > 0]
    sim = simulate_mission_photos(items, uav=uid, airspeed_mps=spd[0], wind=(0.0, 0.0),
                                  intrval_min_s=T.CC.interval_min_s, trigger_duration_s=T.CC.duration_s)
    return uid, items, [s for s, _d in hv.last_plan.plans[uid].strips], sim.photos


def _by_seq(items):
    return {it["seq"]: it for it in items}


# ═══════════════════════════ 段表 ═══════════════════════════
def test_segments_match_the_exported_mission(mission):
    uid, items, strips, photos = mission
    segs = camera_segments(items, strip_ids=strips)
    by = _by_seq(items)
    assert len(segs) == len(strips) == sum(1 for it in items if it["command"] == MAV_CMD_DO_SET_CAM_TRIGG_DIST
                                           and it["param1"] > 0)
    assert [s.index for s in segs] == list(range(1, len(segs) + 1))
    for s in segs:
        assert by[s.on_seq]["command"] == MAV_CMD_DO_SET_CAM_TRIGG_DIST and by[s.on_seq]["param1"] == s.trigg_dist_m > 0
        assert by[s.off_seq]["command"] == MAV_CMD_DO_SET_CAM_TRIGG_DIST and by[s.off_seq]["param1"] == 0.0
        assert by[s.arm_nav_seq]["command"] == 16 and by[s.arm_nav_seq]["param2"] == 1.0   # 引入 NAV 接受半徑 1 m
        assert s.arm_nav_seq < s.on_seq < s.end_nav_seq < s.off_seq
        assert s.rejoin_nav_seq < s.arm_nav_seq and by[s.rejoin_nav_seq]["command"] == 16
        assert s.retrigger_seqs == ()                                     # 平地：沒有地形重觸發
    assert segs[-1].closed_by_land_start and not any(s.closed_by_land_start for s in segs[:-1])
    assert by[segs[-1].off_seq - 1]["command"] == MAV_CMD_DO_LAND_START
    # 段的序號 ＝ 飛前模擬照片的 strip 鍵（同一個 206 開窗規則）
    keys = {p.strip for p in photos if p.strip is not None}
    assert keys == {str(s.index) for s in segs}
    # 段之間的正常完成：MISSION_CURRENT 會是 off 後第一個 NAV
    for a, b in zip(segs, segs[1:]):
        assert a.next_nav_seq is not None and a.next_nav_seq <= b.arm_nav_seq


def test_segment_table_rules_on_synthetic_items():
    """地形重觸發（開著時再送 206 d'）不是新段；DO_LAND_START 後多出來的 206(0) 忽略；長度不符 ⇒ 拒絕。"""
    def it(seq, cmd, p1=0.0):
        return {"seq": seq, "command": cmd, "param1": p1}
    items = [it(0, 16), it(1, 84), it(2, 16), it(3, 16), it(4, 206, 18.8), it(5, 16), it(6, 206, 17.5), it(7, 16),
             it(8, 206, 0.0), it(9, 16), it(10, 16), it(11, 206, 18.8), it(12, 16), it(13, 189), it(14, 206, 0.0),
             it(15, 206, 0.0), it(16, 85)]
    segs = camera_segments(items, expected=[30, 12])
    assert [(s.on_seq, s.off_seq, s.arm_nav_seq, s.end_nav_seq, s.next_nav_seq, s.rejoin_nav_seq) for s in segs] == \
        [(4, 8, 3, 7, 9, 2), (11, 14, 10, 12, 16, 9)]
    assert segs[0].retrigger_seqs == (6,) and segs[0].trigg_dist_m == 18.8
    assert segs[1].closed_by_land_start and not segs[0].closed_by_land_start
    assert [s.expected for s in segs] == [30, 12]
    assert segs[0].contains_current(4) and segs[0].contains_current(7) and not segs[0].contains_current(3)
    with pytest.raises(ValueError, match="不靜默錯位"):
        camera_segments(items, strip_ids=["a"])
    with pytest.raises(ValueError, match="不靜默錯位"):
        camera_segments(items, expected=[1, 2, 3])
    assert expected_by_segment(["1", "1", None, "2"], 2) == [2, 1]
    with pytest.raises(ValueError, match="對不上"):
        expected_by_segment(["3"], 2)


# ═══════════════════════════ 重播 ═══════════════════════════
def _replay(mon, segs, photos, *, skip_photos_in=(), drop_statustext=False, mc_period=0.25, t0=0.0):
    """飛前模擬照片 → 事件流。206 開在該段第一張前 0.05 s、關在最後一張後 1.0 s；MISSION_CURRENT 依段區間給。"""
    by_strip = collections.defaultdict(list)
    for p in photos:
        if p.strip is not None:
            by_strip[int(p.strip)].append(float(p.t_s))
    mon.mode("AUTO", True, t0)
    t_end = t0
    for s in segs:
        ts = by_strip[s.index]
        t_on, t_off = t0 + ts[0] - 0.05, t0 + ts[-1] + 1.0
        if drop_statustext:
            mon.mission_current(s.arm_nav_seq, t_on - 0.3)
            mon.mission_current(s.end_nav_seq, t_on)
        else:
            mon.item_started(s.on_seq, t_on)
        tt = t_on
        for t in ts:
            while drop_statustext and tt + mc_period < t0 + t:
                tt += mc_period
                mon.mission_current(s.end_nav_seq, tt)
            if s.index not in skip_photos_in:
                mon.photos(1, t0 + t)
        if drop_statustext:
            mon.mission_current(s.next_nav_seq, t_off)
        else:
            mon.item_started(s.off_seq, t_off)
        t_end = t_off
    return t_end


def test_replay_counts_equal_the_preflight_simulation_and_no_alarm(mission):
    _uid, items, strips, photos = mission
    segs0 = camera_segments(items)
    exp = expected_by_segment([p.strip for p in photos], len(segs0))
    segs = camera_segments(items, strip_ids=strips, expected=exp)
    mon = StripPhotoMonitor(segs)
    t_end = _replay(mon, segs, photos)
    assert mon.poll(t_end + 10.0) == []
    sm = mon.summary()
    assert [r["photos"] for r in sm] == exp and all(r["state"] == "done" and r["attempts"] == 1 for r in sm)
    assert mon.finish(t_end + 20.0) == [] and mon.off_strip_photos == 0
    assert mon.progress_text() == f"條帶 {len(segs)}/{len(segs)} 完成"
    assert mon.effective == {"min_ratio": 0.8, "grace_s": 2.0, "gap_s": 5.0, "n_segments": len(segs),
                             "expected_known": True}


def test_zero_photos_alarm_fires_once_after_grace(mission):
    _uid, items, strips, photos = mission
    exp = expected_by_segment([p.strip for p in photos], len(strips))
    segs = camera_segments(items, strip_ids=strips, expected=exp)
    mon = StripPhotoMonitor(segs)
    by3 = [float(p.t_s) for p in photos if p.strip == "3"]
    mon.mode("AUTO", True, 0.0)
    for s in segs[:3]:
        ts = [float(p.t_s) for p in photos if p.strip == str(s.index)]
        mon.item_started(s.on_seq, ts[0] - 0.05)
        t = ts[0]
        while t < ts[-1]:                                                  # 實際鏈路：MISSION_CURRENT 持續在送
            mon.mission_current(s.end_nav_seq, t)
            t += 1.0
        if s.index != 3:
            for t in ts:
                mon.photos(1, t)
        mon.item_started(s.off_seq, ts[-1] + 1.0)
    t_off3 = by3[-1] + 1.0
    assert [a.kind for a in mon.poll(t_off3 + 1.9)] == []                  # grace 之前不判
    al = mon.poll(t_off3 + 2.1)
    assert [(a.kind, a.segment, a.strip_id, a.severity, a.photos) for a in al] == \
        [("zero_photos", 3, strips[2], "critical", 0)]
    assert f"seq {segs[2].rejoin_nav_seq}" in al[0].message and al[0].rejoin_nav_seq == segs[2].rejoin_nav_seq
    assert mon.poll(t_off3 + 30.0) == []                                   # 只發一次


def test_interrupt_then_direct_auto_is_flagged_and_sop_rewind_is_not(mission):
    _uid, items, strips, photos = mission
    segs = camera_segments(items, strip_ids=strips)
    s = segs[1]
    # (a) 帶上切 LOITER、再直接切回 AUTO（MIS_RESTART=0 從帶終點接續）
    mon = StripPhotoMonitor(segs)
    mon.mode("AUTO", True, 0.0)
    mon.item_started(s.on_seq, 10.0)
    mon.mission_current(s.end_nav_seq, 10.1)
    for k in range(5):
        mon.photos(1, 11.0 + k)
    mon.mode("LOITER", True, 16.0)
    al = mon.poll(16.1)
    assert [a.kind for a in al] == ["interrupted"] and f"seq {s.rejoin_nav_seq}" in al[0].message
    mon.mission_current(s.end_nav_seq, 30.0)
    mon.mode("AUTO", True, 31.0)
    al = mon.poll(31.1)
    assert [(a.kind, a.severity) for a in al] == [("resumed_without_rewind", "critical")]
    for k in range(3):
        mon.photos(1, 32.0 + k)
    mon.item_started(s.off_seq, 40.0)
    assert mon.poll(43.0) == []
    row = mon.summary()[1]
    assert (row["state"], row["attempts"], row["photos"], row["resumed_without_rewind"]) == ("done", 1, 8, True)
    assert "⚠ #2" in mon.progress_text()
    # (b) 照 SOP：暫停後把目前航點設回段前（rejoin），再切 AUTO、重飛 ⇒ 第 2 次嘗試，沒有 0 張／接續警報
    mon = StripPhotoMonitor(segs)
    mon.mode("AUTO", True, 0.0)
    mon.item_started(s.on_seq, 10.0)
    mon.mode("LOITER", True, 12.0)
    assert [a.kind for a in mon.poll(12.1)] == ["interrupted"]
    mon.item_started(s.rejoin_nav_seq, 20.0)                               # Set WP → "Mission: <rejoin> WP"
    mon.mission_current(s.rejoin_nav_seq, 20.0)
    mon.mode("AUTO", True, 21.0)
    mon.mission_current(s.arm_nav_seq, 40.0)
    mon.item_started(s.on_seq, 60.0)
    for k in range(6):
        mon.photos(1, 61.0 + k)
    mon.item_started(s.off_seq, 70.0)
    assert mon.poll(80.0) == []
    row = mon.summary()[1]
    assert (row["state"], row["attempts"], row["photos_last"], row["resumed_without_rewind"]) == ("done", 2, 6, False)


def test_jump_to_land_sequence_and_unflown_strips(mission):
    _uid, items, strips, _photos = mission
    segs = camera_segments(items, strip_ids=strips)
    mon = StripPhotoMonitor(segs)
    mon.mode("AUTO", True, 0.0)
    mon.item_started(segs[0].on_seq, 1.0)
    mon.photos(4, 2.0)
    mon.item_started(segs[-1].off_seq - 1, 5.0)                           # 失效保護：跳 DO_LAND_START
    al = mon.poll(8.0)
    assert [(a.kind, a.segment, a.photos) for a in al] == [("jumped", 1, 4)]
    fin = mon.finish(100.0)
    assert sorted(a.segment for a in fin if a.kind == "not_flown") == list(range(2, len(segs) + 1))
    assert all(a.kind == "not_flown" for a in fin)
    assert mon.finish(200.0) == []


def test_mission_current_alone_reproduces_the_counts(mission):
    """STATUSTEXT 全掉（低速無線電佇列滿），只剩 4 Hz 的 MISSION_CURRENT ⇒ 逐段張數仍與模擬相同、沒有警報。"""
    _uid, items, strips, photos = mission
    exp = expected_by_segment([p.strip for p in photos], len(strips))
    segs = camera_segments(items, strip_ids=strips, expected=exp)
    mon = StripPhotoMonitor(segs)
    t_end = _replay(mon, segs, photos, drop_statustext=True)
    assert mon.poll(t_end + 10.0) == [] and [r["photos"] for r in mon.summary()] == exp


def test_merged_feedback_gap_and_off_strip_photos(mission):
    _uid, items, strips, _photos = mission
    segs = camera_segments(items, strip_ids=strips, expected=[10] * len(strips))
    mon = StripPhotoMonitor(segs)
    mon.mode("AUTO", True, 0.0)
    # 合併回授：一則 CAMERA_FEEDBACK 帶 7 張（SITLLink 已算好增量）
    mon.item_started(segs[0].on_seq, 1.0)
    mon.photos(7, 2.0)
    mon.photos(3, 2.5)
    mon.item_started(segs[0].off_seq, 3.0)
    # 第 2 段：中間遙測中斷 8 s、0 張 ⇒ 不確定（info），不是 0 張（critical）
    mon.item_started(segs[1].on_seq, 4.0)
    mon.item_started(segs[1].off_seq, 12.5)
    # 帶外：AUTO 中、沒有段開著、離上一段關閉已超過 grace（grace 內的算遲到的回授，歸上一段）
    mon.photos(2, 15.0)
    al = mon.poll(20.0)
    assert [(a.kind, a.segment, a.severity) for a in al] == [("off_strip_photos", None, "warning"),
                                                            ("unverified_count", 2, "info")]
    assert mon.summary()[0]["photos"] == 10 and mon.off_strip_photos == 2
    # 張數不足：預期 10、門檻 floor(0.8×10)=8
    mon.item_started(segs[2].on_seq, 30.0)
    mon.photos(7, 31.0)
    mon.item_started(segs[2].off_seq, 33.0)
    al = mon.poll(40.0)
    assert [(a.kind, a.photos, a.expected) for a in al] == [("under_count", 7, 10)] and "< 8" in al[0].message


def test_first_photo_before_the_on_message_is_attributed_to_the_strip(mission):
    """p3=0 時第一張在 206 開的同一個 50 Hz tick；CAMERA_FEEDBACK 可能比 STATUSTEXT 先到 ⇒ grace 內歸給該段。"""
    _uid, items, strips, _photos = mission
    segs = camera_segments(items, strip_ids=strips)
    mon = StripPhotoMonitor(segs)
    mon.mode("AUTO", True, 0.0)
    mon.photos(1, 10.0)
    mon.item_started(segs[0].on_seq, 10.4)
    mon.photos(1, 11.0)
    mon.item_started(segs[0].off_seq, 12.0)
    assert mon.poll(20.0) == [] and mon.summary()[0]["photos"] == 2 and mon.off_strip_photos == 0


@pytest.mark.parametrize("kw", [dict(min_ratio=0.0), dict(min_ratio=1.5), dict(grace_s=-1.0), dict(gap_s=float("nan"))])
def test_bad_thresholds_are_refused(kw):
    with pytest.raises(ValueError):
        StripPhotoMonitor([], **kw)


def test_alarm_kinds_are_the_documented_set():
    assert SM.ALARM_KINDS == ("zero_photos", "under_count", "unverified_count", "interrupted",
                              "resumed_without_rewind", "jumped", "not_flown", "off_strip_photos")
    assert math.isclose(SM.DEFAULT_MIN_RATIO, 0.8) and SM.MAV_CMD_NAV_LAST == 95


def test_expected_photos_by_segment_attribution():
    """逐段預估（轉接層）：p3=1 的開／關立即拍算在那一段（關 ⇒ 剛關的那段），相機關著的 p3=1 算帶外；
    合計 ＋ 帶外 ＝ expected_photo_count（GUI 顯示的「預估 M 張」）。"""
    from hv_dccpp.integration.aeroplan_adapter import expected_photo_count, expected_photos_by_segment

    def it(seq, cmd, p1=0.0, p3=0.0, n=0.0, p2=0.0):
        return {"seq": seq, "command": cmd, "param1": p1, "param2": p2, "param3": p3,
                "lat": 23.95 + n / 111132.0, "lon": 120.9}
    items = [it(0, 16), it(1, 178, 0.0, p2=20.0), it(2, 16, n=0.0), it(3, 206, 10.0, 1.0), it(4, 16, n=105.0),
             it(5, 206, 0.0, 1.0), it(6, 16, n=210.0), it(7, 206, 0.0, 1.0), it(8, 206, 10.0), it(9, 16, n=315.0),
             it(10, 206, 0.0)]
    by, out = expected_photos_by_segment(items)
    # 段 1：開立即拍 1 ＋ 約 105 m／10 m 間距（50 Hz tick：20 m/s × 0.02 s = 0.4 m，10 m 整除）10 張 ＋ 關立即拍 1
    # 帶外：相機已關時的 206(0, p3=1) 1 張；段 2：約 105 m／10 m = 10 張
    assert (by, out) == ([12, 10], 1)
    assert sum(by) + out == expected_photo_count(items) == 23


def test_next_nav_starting_before_the_off_command_is_a_normal_close():
    """實測 .BIN（S1_HDMAP_20261015_PW3）：最後一條帶 帶終點 NAV → DO_LAND_START → 206(0)，飛控先開始下一個 NAV、
    0.1 s 後才執行 206(0) ⇒ 正常關閉，不是跳號；之間的照片照常歸本段。下一個 NAV 開始後 grace 內沒收到本段的
    206(0) 才是跳號（以下一個 NAV 開始的時刻記）。"""
    def it(seq, cmd, p1=0.0):
        return {"seq": seq, "command": cmd, "param1": p1}
    items = [it(0, 16), it(1, 84), it(2, 16), it(3, 16), it(4, 206, 18.8), it(5, 16), it(6, 189), it(7, 206, 0.0),
             it(8, 16), it(9, 85)]
    segs = camera_segments(items, expected=[3])
    mon = StripPhotoMonitor(segs)
    mon.mode("AUTO", True, 0.0)
    mon.item_started(4, 1.0)
    for k in range(3):
        mon.photos(1, 2.0 + k)
    mon.item_started(6, 10.0)                                          # DO_LAND_START
    mon.item_started(8, 10.0)                                          # 下一個 NAV 先開始
    mon.photos(1, 10.05)                                               # 相機到 206(0) 前仍開著
    mon.item_started(7, 10.1)                                          # 206(0)
    assert mon.poll(20.0) == [] and mon.summary()[0]["state"] == "done" and mon.summary()[0]["photos"] == 4
    # 真的跳號：下一個 NAV 開始後一直沒有本段的 206(0)
    mon = StripPhotoMonitor(segs)
    mon.mode("AUTO", True, 0.0)
    mon.item_started(4, 1.0)
    mon.photos(3, 2.0)
    mon.item_started(8, 10.0)
    assert mon.poll(11.9) == [] and mon.summary()[0]["state"] == "active"     # grace 內還不判
    al = mon.poll(12.1)
    assert [(a.kind, a.segment, a.photos) for a in al] == [("jumped", 1, 3)]
