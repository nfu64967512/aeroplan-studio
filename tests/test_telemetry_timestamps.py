# -*- coding: utf-8 -*-
"""tests/test_telemetry_timestamps.py — 3D 平順化的 Python 端（2026-10-17）。

使用者：「畫面要流暢、轉向時機頭不能慢半拍」。審查抓到 3D 插值的時間戳用「JS 收到的時刻」：GUI 30 Hz 合幀會把同一則
20 Hz 位置重送好幾次、兩批也可能貼著到 ⇒ 樣本走走停停甚至倒退。現在：
① 鏈路在收到 GLOBAL_POSITION_INT／ATTITUDE 的當下記 `pos_t`／`att_t`（epoch 秒）。
② telemetry 信號送**快照**（跨執行緒只傳參考，GUI 晚讀時不會讀到改了一半的 lat/lon）；servo_pwm 也要獨立。
③ 主視窗的地圖批次每筆帶 `pt`／`at`（epoch ms，整數）；沒收過就不帶。
"""
from __future__ import annotations

import ast
import os
import pathlib
import types

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('AEROPLAN_NO_FILE_LOG', '1')

import ui.main_window  # noqa: E402,F401  QtWebEngine 要在 Q*Application 之前匯入
from PyQt6.QtCore import QCoreApplication  # noqa: E402

_APP = QCoreApplication.instance() or QCoreApplication([])

import mission.sitl_link as sl  # noqa: E402
from mission.sitl_link import SITLLink, TelemetryFrame  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[1]


def _msg(mtype, src=1, comp=1, **fields):
    m = types.SimpleNamespace(**fields)
    m.get_type = lambda: mtype
    m.get_srcSystem = lambda: src
    m.get_srcComponent = lambda: comp
    return m


def _link():
    link = SITLLink(conn_str='tcp:127.0.0.1:5760', sysid_label=1)
    got = []
    link.telemetry.connect(got.append)          # 直接連線（同執行緒）⇒ emit 當下同步呼叫
    link._got = got
    # 把 sysid 1 認成飛機（心跳）
    link._handle_message(_msg('HEARTBEAT', type=1, autopilot=3, base_mode=0, custom_mode=0, system_status=4))
    got.clear()
    return link


def test_position_and_attitude_messages_stamp_their_receive_time(monkeypatch):
    link = _link()
    t = {'now': 1_700_000_000.250}
    monkeypatch.setattr(sl.time, 'time', lambda: t['now'])
    link._handle_message(_msg('GLOBAL_POSITION_INT', lat=234500000, lon=1202800000, alt=108000, relative_alt=100000,
                              hdg=9000, vx=1000, vy=0, vz=0, time_boot_ms=1000))
    t['now'] += 0.02
    link._handle_message(_msg('ATTITUDE', roll=0.1, pitch=0.05, yaw=1.0, time_boot_ms=1020))
    assert len(link._got) == 2
    pos_frame, att_frame = link._got
    assert pos_frame.pos_t == 1_700_000_000.250 and pos_frame.att_t == 0.0
    assert att_frame.pos_t == 1_700_000_000.250 and abs(att_frame.att_t - 1_700_000_000.270) < 1e-9
    # VFR_HUD 不改兩個時間戳
    t['now'] += 0.02
    link._handle_message(_msg('VFR_HUD', airspeed=19.0, groundspeed=18.0, heading=91, throttle=50, alt=108.0, climb=0.0))
    assert link._got[-1].pos_t == pos_frame.pos_t and link._got[-1].att_t == att_frame.att_t


def test_emitted_frames_are_snapshots_not_the_live_object():
    link = _link()
    link._handle_message(_msg('GLOBAL_POSITION_INT', lat=234500000, lon=1202800000, alt=0, relative_alt=0,
                              hdg=0, vx=0, vy=0, vz=0, time_boot_ms=1))
    first = link._got[-1]
    live = link._frames[1]
    assert first is not live, '跨執行緒送活物件會在 GUI 晚讀時撕裂'
    link._handle_message(_msg('GLOBAL_POSITION_INT', lat=235000000, lon=1203000000, alt=0, relative_alt=0,
                              hdg=0, vx=0, vy=0, vz=0, time_boot_ms=2))
    assert round(first.lat, 5) == 23.45 and round(live.lat, 5) == 23.5, '已送出的快照不能被後來的訊息改掉'
    servo = {f'servo{i}_raw': 1000 + i for i in range(1, 17)}
    link._handle_message(_msg('SERVO_OUTPUT_RAW', **servo))
    snap = link._got[-1]
    servo2 = {f'servo{i}_raw': 2000 for i in range(1, 17)}
    link._handle_message(_msg('SERVO_OUTPUT_RAW', **servo2))
    assert snap.servo_pwm[0] == 1001 and live.servo_pwm[0] == 2000, 'servo_pwm 不能跟活物件共用 list'
    assert link.get_latest_telemetry() is live or link.get_latest_telemetry().lat == live.lat


def test_batch_item_carries_ms_timestamps_only_when_known():
    from ui.main_window import MainWindow
    f = TelemetryFrame(sysid=1, lat=23.45, lon=120.28, gps_fix=3, pos_t=1_700_000_000.1234, att_t=1_700_000_000.2)
    it = MainWindow._uav_batch_item(f)
    assert it['pt'] == 1_700_000_000_123 and it['at'] == 1_700_000_000_200
    assert isinstance(it['pt'], int) and isinstance(it['at'], int)
    bare = MainWindow._uav_batch_item(TelemetryFrame(sysid=2, lat=1.0, lon=1.0, gps_fix=3))
    assert 'pt' not in bare and 'at' not in bare, '沒收過的就不帶（地圖端退回現在時刻）'


def test_handle_message_source_sets_the_stamps_in_the_right_branches():
    """AST：pos_t 只在 GLOBAL_POSITION_INT 分支設、att_t 只在 ATTITUDE 分支設；emit 前做快照。"""
    src = (ROOT / 'mission' / 'sitl_link.py').read_text(encoding='utf-8', errors='replace')
    tree = ast.parse(src)
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == '_handle_message')
    body = ast.unparse(fn)
    assert body.count('f.pos_t = time.time()') == 1 and body.count('f.att_t = time.time()') == 1
    gpi = body.index("mtype == 'GLOBAL_POSITION_INT'")
    att = body.index("mtype == 'ATTITUDE'")
    vfr = body.index("mtype == 'VFR_HUD'")
    assert gpi < body.index('f.pos_t = time.time()') < att < body.index('f.att_t = time.time()') < vfr
    assert 'snap = copy.copy(f)' in body and 'self.telemetry.emit(snap)' in body
    assert 'self.telemetry.emit(f)' not in body


# ── ④ 姿態的取樣時刻：飛控 time_boot_ms 換到牆鐘（2026-10-22）─────────────────────────────
def test_attitude_carries_its_fc_boot_time_mapped_onto_the_wall_clock_and_att_t_stays_the_receipt(monkeypatch):
    """SITL 成批送：收到時刻比飛控取樣晚 0～32 ms 不等。att_t 仍是收到時刻；att_boot_ms = 那則的 time_boot_ms；
    att_ts = 換到牆鐘的取樣時刻（下包絡 ⇒ 等於「最快那幾則」的延遲，≤ 收到時刻）。"""
    import random
    link = _link()
    rng = random.Random(7)
    base, floor = 1_700_000_000.0, 0.004            # 牆鐘 = base + 開機秒；最小傳輸延遲 4 ms
    t = {'now': 0.0}
    monkeypatch.setattr(sl.time, 'time', lambda: t['now'])
    late = []
    for k in range(1500):                           # 30 s、50 Hz
        boot_ms = 2000 + 20 * k
        t['now'] = base + boot_ms / 1000.0 + floor + rng.uniform(0.0, 0.032)
        link._handle_message(_msg('ATTITUDE', roll=0.1, pitch=0.05, yaw=1.0, time_boot_ms=boot_ms))
        f = link._got[-1]
        assert f.att_boot_ms == boot_ms and f.att_t == t['now'] and f.att_ts <= f.att_t
        if k >= 250:
            late.append(f.att_ts - (base + boot_ms / 1000.0 + floor))
    late.sort()
    assert -0.0005 <= late[0] and late[-1] < 0.002, (late[0], late[-1])
    assert late[len(late) // 2] < 0.001, '取樣時刻落在最小延遲線上（收到時刻平均晚 16 ms）'


def test_attitude_without_a_boot_time_falls_back_to_the_receipt_and_each_source_has_its_own_clock(monkeypatch):
    link = _link()
    link._handle_message(_msg('HEARTBEAT', src=2, type=1, autopilot=3, base_mode=0, custom_mode=0, system_status=4))
    monkeypatch.setattr(sl.time, 'time', lambda: 1_700_000_100.0)
    link._handle_message(_msg('ATTITUDE', roll=0.0, pitch=0.0, yaw=0.0))                        # 沒有 time_boot_ms
    f = link._got[-1]
    assert f.att_boot_ms == -1 and f.att_ts == f.att_t == 1_700_000_100.0
    link._handle_message(_msg('ATTITUDE', roll=0.0, pitch=0.0, yaw=0.0, time_boot_ms=5000))
    link._handle_message(_msg('ATTITUDE', src=2, roll=0.0, pitch=0.0, yaw=0.0, time_boot_ms=900000))
    assert set(link._boot_clocks) == {1, 2}, '每個來源 system 一個對時（兩架的開機時間互不相干）'
    assert link._got[-1].sysid == 2 and link._got[-1].att_ts == 1_700_000_100.0


def test_a_rebooted_fc_starts_a_new_clock_segment(monkeypatch):
    link = _link()
    t = {'now': 0.0}
    monkeypatch.setattr(sl.time, 'time', lambda: t['now'])
    for boot_ms in range(600000, 610000, 20):                    # 開機 600 s 時
        t['now'] = 1_700_000_000.0 + boot_ms / 1000.0
        link._handle_message(_msg('ATTITUDE', roll=0.0, pitch=0.0, yaw=0.0, time_boot_ms=boot_ms))
    t0 = t['now'] + 30.0                                          # 30 s 後重開機：開機時間從 1 s 重來
    for boot_ms in range(1000, 3000, 20):
        t['now'] = t0 + (boot_ms - 1000) / 1000.0
        link._handle_message(_msg('ATTITUDE', roll=0.0, pitch=0.0, yaw=0.0, time_boot_ms=boot_ms))
        assert abs(link._got[-1].att_ts - t['now']) < 1e-9, '新的一段：沒有延遲 ⇒ 取樣時刻 = 收到時刻'
    clk = link._boot_clocks[1]
    assert clk.reboots == 1 and clk.segments == 2
