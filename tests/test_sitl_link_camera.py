"""mission.sitl_link —— 接相機伴隨元件之前要先修好的地面站鏈路：值層級測試。

四件事（2026-10-14 群飛航測評估 §「sitl_link 修正」）：

① **同 sysid 的相機心跳蓋掉機卡**（離線已重現）：相機 / 伴隨電腦（compid 100、
   MAV_TYPE_CAMERA 30）搭上「src == sysid_label 永遠放行」，機卡被改成
   TYPE_30 / MODE_0 / disarmed。⇒ 只有自駕儀元件能動機卡，其他元件另外記。
   順帶：MAV_TYPE 手抄表錯號（VTOL 19/20/21 被寫成 20/21/22；非載具清單從 30 起錯兩格）。
② **CAMERA_FEEDBACK 被靜默丟掉**。⇒ 新信號＋計數；張數看 img_idx 跳號，不看訊息則數
   （ArduPilot 每條鏈路只有一個待送位元，忙時會合併）。
③ **沒有任何會回傳值的參數讀回 API**。⇒ param_value 信號＋同步 read_params()
   ＋float32 感知的 values_match()。
④ **tlog**：opt-in，標準 8 位元組 big-endian µs 前綴；沒呼叫 start_tlog 就不寫檔。

全部不開真連線：訊息用 pymavlink 的 MAVLink2 編碼器打包再解碼（帶真的 sysid/compid
與原始位元組），直接餵 _handle_message / _tlog_hook；run() 用假的
mavlink_connection（monkeypatch）。
"""
from __future__ import annotations

import queue
import struct
import threading
import time
from types import SimpleNamespace

import pytest
from PyQt6.QtCore import QCoreApplication
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mav2

from mission import sitl_link as sl
from mission.sitl_link import SITLLink, values_match

_app = QCoreApplication.instance() or QCoreApplication([])

# ArduPilot 飛控與 MAVSDK 類相機伺服器的心跳欄位
FC_TYPE, FC_AP = 1, 3            # MAV_TYPE_FIXED_WING / MAV_AUTOPILOT_ARDUPILOTMEGA
CAM_TYPE, CAM_AP = 30, 8         # MAV_TYPE_CAMERA / MAV_AUTOPILOT_INVALID
ARMED = 128 | 1                  # SAFETY_ARMED | CUSTOM_MODE_ENABLED
PLANE_AUTO, PLANE_RTL, PLANE_QHOVER = 10, 11, 18


def mk(src: int, comp: int, name: str, *args):
    """用 MAVLink2 編碼器打包 → 再解碼：得到帶真實 header（sysid/compid）與原始位元組的訊息。"""
    tx = mav2.MAVLink(None, srcSystem=src, srcComponent=comp)
    buf = getattr(tx, f'{name}_encode')(*args).pack(tx)
    msg = mav2.MAVLink(None).decode(bytearray(buf))
    assert bytes(msg.get_msgbuf()) == bytes(buf)
    return msg


def hb(src, comp, mtype=FC_TYPE, autopilot=FC_AP, base_mode=ARMED,
       custom_mode=PLANE_AUTO, status=4):
    return mk(src, comp, 'heartbeat', mtype, autopilot, base_mode, custom_mode, status, 3)


def cam_hb(src=1, comp=100, mtype=CAM_TYPE, autopilot=CAM_AP):
    return hb(src, comp, mtype, autopilot, 0, 0, 4)


def gpos(src, comp, lat, lon):
    return mk(src, comp, 'global_position_int', 1000, int(lat * 1e7), int(lon * 1e7),
              100_000, 50_000, 1000, 0, 0, 9000)


def cfb(src=1, comp=1, img=1, cc=0, cam=0, lat=241234567, lng=1205678901,
        alt_msl=123.5, alt_rel=80.25, roll=1.5, pitch=-2.25, yaw=359.5,
        foc=16.0, flags=0, time_usec=1_700_000_000_123_456):
    return mk(src, comp, 'camera_feedback', time_usec, 0, cam, img, lat, lng,
              alt_msl, alt_rel, roll, pitch, yaw, foc, flags, cc)


def pval(src, comp, name, value, ptype=9, count=900, index=65535):
    return mk(src, comp, 'param_value', name.encode('ascii'), value, ptype, count, index)


def make_link(sysid_label=1, **kw):
    link = SITLLink(conn_str='tcp:127.0.0.1:5760', sysid_label=sysid_label, **kw)
    link._got_tel = []
    link._got_comp = []
    link._got_cam = []
    link._got_param = []
    link._got_status = []
    link.telemetry.connect(lambda f: link._got_tel.append(
        (f.sysid, f.vehicle_type, f.mode, f.armed, round(f.lat, 5))))
    link.component_heartbeat.connect(lambda s, c, i: link._got_comp.append((s, c, i)))
    link.camera_feedback.connect(lambda s, d: link._got_cam.append((s, d)))
    link.param_value.connect(lambda *a: link._got_param.append(a))
    link.status_text.connect(lambda sev, t: link._got_status.append((sev, t)))
    return link


def card(link, sysid=1):
    f = link._frames[sysid]
    return (f.vehicle_type, f.mode, f.armed)


# ═════════════════════════════════════════════════════════════════════
# ① 元件分類：相機心跳不可以動機卡
# ═════════════════════════════════════════════════════════════════════
class TestCompanionHeartbeat:
    def test_camera_heartbeat_does_not_touch_aircraft_card(self):
        link = make_link()
        link._handle_message(hb(1, 1))
        assert card(link) == ('PLANE', 'AUTO', True)
        n_tel = len(link._got_tel)

        link._handle_message(cam_hb())            # 修正前：TYPE_30 / MODE_0 / False
        assert card(link) == ('PLANE', 'AUTO', True)
        assert len(link._got_tel) == n_tel        # 相機心跳不發 telemetry
        assert len(link._got_comp) == 1
        s, c, info = link._got_comp[0]
        assert (s, c) == (1, 100)
        assert info['type'] == 30 and info['type_name'] == 'CAMERA'
        assert info['autopilot'] == 8 and info['autopilot_component'] is False
        assert info['count'] == 1

    def test_autopilot_heartbeat_still_updates_card(self):
        link = make_link()
        link._handle_message(hb(1, 1))
        link._handle_message(cam_hb())
        link._handle_message(hb(1, 1, base_mode=1, custom_mode=PLANE_RTL))
        assert card(link) == ('PLANE', 'RTL', False)
        assert [t[2:4] for t in link._got_tel] == [('AUTO', True), ('RTL', False)]

    def test_camera_heartbeat_first_creates_no_card(self):
        link = make_link()
        link._handle_message(cam_hb())
        assert link._frames == {}
        assert link._got_tel == []
        link._handle_message(hb(1, 1))
        assert card(link) == ('PLANE', 'AUTO', True)

    def test_camera_claiming_ardupilot_autopilot_is_still_not_the_autopilot(self):
        """有些相機把 autopilot 欄位填成 ARDUPILOTMEGA —— 不管飛控先到或後到都不能算數。"""
        link = make_link()
        link._handle_message(cam_hb(autopilot=FC_AP))       # 飛控還沒開口
        assert link._frames == {}
        link._handle_message(hb(1, 1))
        link._handle_message(cam_hb(autopilot=FC_AP))       # 飛控之後
        assert card(link) == ('PLANE', 'AUTO', True)
        assert link._autopilot_compid == {1: 1}

    def test_vehicle_looking_second_component_does_not_steal_the_card(self):
        """compid 1 之後出現另一個「看起來像飛機」的元件（type 1 / autopilot 3）也不算。"""
        link = make_link()
        link._handle_message(hb(1, 1))
        link._handle_message(hb(1, 50, base_mode=0, custom_mode=PLANE_RTL))
        link._handle_message(hb(1, 191, mtype=18, autopilot=8, base_mode=0, custom_mode=0))
        assert card(link) == ('PLANE', 'AUTO', True)
        assert sorted(c for _, c, _ in link._got_comp) == [50, 191]

    def test_autopilot_on_non_default_compid(self):
        """飛控不是 compid 1 時：第一個像飛機的元件當自駕儀，之後的相機照樣分流。"""
        link = make_link(sysid_label=7)
        link._handle_message(hb(7, 50))
        link._handle_message(cam_hb(src=7))
        assert card(link, 7) == ('PLANE', 'AUTO', True)
        link._handle_message(gpos(7, 50, 24.5, 121.5))
        link._handle_message(gpos(7, 100, 0.0, 0.0))
        assert round(link._frames[7].lat, 5) == 24.5

    def test_telemetry_from_camera_component_is_not_written_to_card(self):
        link = make_link()
        link._handle_message(hb(1, 1))
        link._handle_message(gpos(1, 1, 24.1, 121.1))
        link._handle_message(gpos(1, 100, 0.0, 0.0))
        assert round(link._frames[1].lat, 5) == 24.1
        assert round(link._frames[1].lon, 5) == 121.1

    def test_component_registry(self):
        link = make_link()
        link._handle_message(hb(1, 1))
        for _ in range(3):
            link._handle_message(cam_hb())
        comps = link.get_components(1)
        assert set(comps) == {1, 100}
        assert comps[1]['autopilot_component'] is True and comps[1]['type_name'] == 'FIXED_WING'
        assert comps[100]['autopilot_component'] is False and comps[100]['count'] == 3
        assert comps[100]['last_seen'] >= comps[100]['first_seen']
        assert set(link.get_components()) == {1}
        comps[100]['count'] = 999                      # 回的是複本
        assert link.get_components(1)[100]['count'] == 3
        assert link.get_components(42) == {}


class TestMavTypeTables:
    @pytest.mark.parametrize('mav_type,expected', [
        (19, 'VTOL_DUOROTOR'), (20, 'VTOL_QUADROTOR'), (21, 'VTOL_TILTROTOR'),
        (22, 'TYPE_22'), (1, 'PLANE'), (2, 'COPTER'), (13, 'COPTER'), (14, 'COPTER'),
        (15, 'COPTER'), (10, 'ROVER'), (11, 'BOAT'), (30, 'TYPE_30'),
    ])
    def test_decode_vehicle_type_uses_real_enum_numbers(self, mav_type, expected):
        assert sl._decode_vehicle_type(mav_type) == expected

    def test_vtol_heartbeat_decodes_plane_modes(self):
        link = make_link()
        link._handle_message(hb(1, 1, mtype=20, custom_mode=PLANE_QHOVER))
        assert card(link) == ('VTOL_QUADROTOR', 'QHOVER', True)

    def test_non_vehicle_set_is_the_enum_not_the_old_hand_numbers(self):
        assert sl._non_vehicle_types() == frozenset(
            {5, 6, 18, 26, 27, 30, 31, 32, 33, 34, 36, 37, 38, 39, 40, 41, 42})

    @pytest.mark.parametrize('mav_type', [33, 34, 41, 42])   # SERVO ODID GPS WINCH：舊集合漏掉的
    def test_previously_missed_non_vehicle_types_are_rejected(self, mav_type):
        link = make_link()
        assert link._heartbeat_is_vehicle(hb(191, 1, mtype=mav_type)) is False

    @pytest.mark.parametrize('mav_type', [29, 35])           # DODECAROTOR / DECAROTOR 是飛機
    def test_big_multirotors_are_vehicles(self, mav_type):
        link = make_link()
        assert link._heartbeat_is_vehicle(hb(1, 1, mtype=mav_type)) is True


# ═════════════════════════════════════════════════════════════════════
# run()：連線時等的是「自駕儀」的心跳
# ═════════════════════════════════════════════════════════════════════
class _FakeConn:
    def __init__(self, link, heartbeats):
        self.link = link
        self._hbs = list(heartbeats)
        self.message_hooks = []
        self.target_system = 1
        self.target_component = 0
        self.mav = SimpleNamespace(heartbeat_send=lambda *a: None,
                                   request_data_stream_send=lambda *a: None)

    def wait_heartbeat(self, timeout=None, blocking=True):
        if not self._hbs:
            return None
        m = self._hbs.pop(0)
        for h in self.message_hooks:          # 模擬 pymavlink post_message 呼叫 hooks
            h(self, m)
        return m

    def recv_match(self, *a, **k):
        self.link._stop = True
        return None

    def close(self):
        pass


def _run_with(monkeypatch, link, heartbeats):
    conn = _FakeConn(link, heartbeats)
    monkeypatch.setattr(mavutil, 'mavlink_connection', lambda *a, **k: conn)
    monkeypatch.setattr(link, '_request_servo_function_params', lambda: None)
    errors = []
    link.error.connect(errors.append)
    link.run()
    return conn, errors


class TestRunInitialHeartbeat:
    def test_camera_heartbeat_first_is_skipped(self, monkeypatch, tmp_path):
        link = make_link(vehicle_hint='PLANE')
        tl = tmp_path / 'run.tlog'
        link.start_tlog(tl)
        conn, errors = _run_with(monkeypatch, link, [cam_hb(), hb(1, 1)])
        assert errors == []
        assert link._frames[1].vehicle_type == 'PLANE'          # 修正前：TYPE_30
        comps = link.get_components(1)
        assert comps[100]['autopilot_component'] is False
        assert comps[1]['autopilot_component'] is True
        assert link._tlog_hook in conn.message_hooks
        # hook 在 wait_heartbeat 之前就裝好 ⇒ 兩包心跳都記到；run() 結束時 tlog 自動關檔
        st = link.tlog_status()
        assert st['active'] is False and st['messages'] == 2
        assert link._link_closed is True and link._recv_thread_ident is None

    def test_only_camera_heartbeats_reports_it_clearly(self, monkeypatch):
        link = make_link(vehicle_hint='PLANE')
        conn, errors = _run_with(monkeypatch, link, [cam_hb()])
        assert link._frames == {}
        assert len(errors) == 1
        assert '非自駕儀' in errors[0] and 'CAMERA' in errors[0]


# ═════════════════════════════════════════════════════════════════════
# ② CAMERA_FEEDBACK
# ═════════════════════════════════════════════════════════════════════
class TestCameraFeedback:
    def test_payload_values(self):
        link = make_link()
        link._handle_message(hb(1, 1))
        n_tel = len(link._got_tel)
        link._handle_message(cfb(img=1))
        assert len(link._got_cam) == 1
        s, d = link._got_cam[0]
        assert s == 1
        assert d['img_idx'] == 1 and d['cam_idx'] == 0 and d['compid'] == 1
        assert d['lat_e7'] == 241234567 and d['lng_e7'] == 1205678901
        assert d['lat'] == pytest.approx(24.1234567, abs=1e-9)
        assert d['lng'] == pytest.approx(120.5678901, abs=1e-9)
        assert d['alt_msl'] == 123.5 and d['alt_rel'] == 80.25
        assert (d['roll'], d['pitch'], d['yaw']) == (1.5, -2.25, 359.5)
        assert d['foc_len'] == 16.0 and d['flags'] == 0 and d['completed_captures'] == 0
        assert d['time_usec'] == 1_700_000_000_123_456           # 原樣轉交
        assert (d['new_photos'], d['gap'], d['triggered'], d['missing']) == (1, None, 1, 0)
        # 不碰機卡、不發 telemetry
        assert len(link._got_tel) == n_tel and link._frames[1].lat == 0.0

    def test_coalesced_messages_are_counted_by_img_idx(self):
        link = make_link()
        link._handle_message(hb(1, 1))
        for img in (1, 2, 5, 6, 9):           # 3,4 與 7,8 被合併掉
            link._handle_message(cfb(img=img))
        st = link.get_camera_stats(1)
        assert st['messages'] == 5
        assert st['triggered'] == 9
        assert st['received'] == 5 and st['missing'] == 4
        assert st['gaps'] == [(3, 4), (7, 8)]
        assert sum(d['new_photos'] for _, d in link._got_cam) == 9
        d5 = link._got_cam[2][1]
        assert (d5['img_idx'], d5['new_photos'], d5['gap'], d5['missing']) == (5, 3, (3, 4), 2)

    def test_first_message_mid_flight(self):
        link = make_link()
        link._handle_message(hb(1, 1))
        link._handle_message(cfb(img=57))
        st = link.get_camera_stats()
        assert (st['first_img_idx'], st['triggered'], st['missing'], st['gaps']) == (57, 57, 0, [])

    def test_feedback_pin_duplicate_img_counts_completed(self):
        link = make_link()
        link._handle_message(hb(1, 1))
        link._handle_message(cfb(img=5, cc=4))
        link._handle_message(cfb(img=5, cc=5))
        link._handle_message(cfb(img=6, cc=6))
        st = link.get_camera_stats()
        assert (st['triggered'], st['completed'], st['duplicates'], st['missing']) == (6, 6, 1, 0)
        assert [d['duplicate'] for _, d in link._got_cam] == [False, True, False]
        assert [d['new_photos'] for _, d in link._got_cam] == [5, 0, 1]

    def test_completed_captures_counted_by_jumps(self):
        link = make_link()
        link._handle_message(hb(1, 1))
        for img, cc in ((1, 1), (4, 3), (6, 6)):     # 回授確認也會被合併
            link._handle_message(cfb(img=img, cc=cc))
        st = link.get_camera_stats()
        assert (st['triggered'], st['completed'], st['completed_captures']) == (6, 6, 6)
        assert [d['completed'] for _, d in link._got_cam] == [1, 3, 6]

    def test_img_idx_going_backwards_is_a_reset(self):
        link = make_link()
        link._handle_message(hb(1, 1))
        link._handle_message(cfb(img=10, cc=10))
        link._handle_message(cfb(img=2, cc=2))
        st = link.get_camera_stats()
        assert (st['resets'], st['triggered'], st['completed']) == (1, 12, 12)
        assert st['gaps'] == [(1, 1)] and st['missing'] == 1
        assert link._got_cam[-1][1]['reset'] is True
        assert any(sev == 4 and 'img_idx' in t for sev, t in link._got_status)

    def test_uint16_wrap_is_not_a_reset(self):
        link = make_link()
        link._handle_message(hb(1, 1))
        for img in (65534, 65535, 1):
            link._handle_message(cfb(img=img))
        st = link.get_camera_stats()
        assert (st['resets'], st['triggered'], st['missing'], st['gaps']) == (0, 65537, 1, [(0, 0)])

    def test_counters_are_per_sysid_and_per_camera(self):
        link = make_link()
        link._handle_message(hb(1, 1))
        link._handle_message(hb(2, 1))
        link._handle_message(cfb(src=1, img=3))
        link._handle_message(cfb(src=2, img=8))
        link._handle_message(cfb(src=1, img=4))
        link._handle_message(cfb(src=1, img=1, cam=1))
        assert link.get_camera_stats(1)['triggered'] == 4
        assert link.get_camera_stats(2)['triggered'] == 8
        assert link.get_camera_stats(1, cam_idx=1)['triggered'] == 1
        assert link.get_camera_stats(1)['resets'] == 0
        assert link.get_camera_stats(3) is None

    def test_feedback_from_non_autopilot_or_non_vehicle_is_ignored(self):
        link = make_link()
        link._handle_message(hb(1, 1))
        link._handle_message(hb(191, 1, mtype=18, autopilot=8))   # MAVROS
        link._handle_message(cfb(src=1, comp=100, img=1))
        link._handle_message(cfb(src=191, comp=1, img=1))
        assert link._got_cam == []
        assert link.get_camera_stats(1) is None and link.get_camera_stats(191) is None

    def test_stats_are_a_copy(self):
        link = make_link()
        link._handle_message(hb(1, 1))
        link._handle_message(cfb(img=1))
        link._handle_message(cfb(img=3))
        st = link.get_camera_stats()
        st['gaps'].append((99, 99))
        st['triggered'] = 0
        again = link.get_camera_stats()
        assert again['gaps'] == [(2, 2)] and again['triggered'] == 3


# ═════════════════════════════════════════════════════════════════════
# ③ 參數讀回
# ═════════════════════════════════════════════════════════════════════
class TestParamValueSignal:
    def test_every_param_value_is_emitted_with_type(self):
        link = make_link()
        link._handle_message(pval(1, 1, 'CAM1_TYPE', 2.0, ptype=2, count=900, index=123))
        link._handle_message(pval(1, 1, 'ABCDEFGHIJKLMNOP', 1.5))     # 16 字元、無 NUL
        assert link._got_param == [(1, 'CAM1_TYPE', 2.0, 2, 123, 900),
                                   (1, 'ABCDEFGHIJKLMNOP', 1.5, 9, 65535, 900)]

    def test_four_argument_slot_gets_the_documented_signature(self):
        link = make_link()
        got = []
        link.param_value.connect(lambda sysid, name, value, ptype: got.append(
            (sysid, name, value, ptype)))
        link._handle_message(pval(3, 1, 'LOG_BITMASK', 65535.0, ptype=6))
        assert got == [(3, 'LOG_BITMASK', 65535.0, 6)]

    def test_servo_path_unchanged_and_signal_also_fires(self):
        link = make_link()
        servo = []
        link.servo_param.connect(lambda ch, key, v: servo.append((ch, key, v)))
        link._handle_message(pval(1, 1, 'SERVO5_FUNCTION', 33.0, ptype=4))
        assert servo == [(5, 'FUNCTION', 33.0)]
        assert link._got_param == [(1, 'SERVO5_FUNCTION', 33.0, 4, 65535, 900)]
        assert link._frames == {}

    def test_minimal_fake_message_without_type_fields(self):
        link = make_link(sysid_label=21)
        msg = SimpleNamespace(param_id='SERVO5_FUNCTION', param_value=33.0,
                              get_type=lambda: 'PARAM_VALUE', get_srcSystem=lambda: 21)
        link._handle_message(msg)
        assert link._got_param == [(21, 'SERVO5_FUNCTION', 33.0, 0, -1, 0)]


class _FakeAutopilot:
    """扮演「接收執行緒＋飛控」：從 link._cmd_queue 取指令經 _dispatch_command 送出，
    飛控對每個 PARAM_REQUEST_READ 回一則 PARAM_VALUE，再餵回 link._handle_message。"""

    def __init__(self, link, params, *, types=None, sysid=1, drop_first=(), noise=None):
        self.link = link
        self.params = dict(params)
        self.types = dict(types or {})
        self.sysid = sysid
        self.drop_first = set(drop_first)
        self.noise = noise or (lambda name: [])
        self.target_system = sysid
        self.target_component = 0            # pymavlink 預設：廣播
        self.requests = []
        self._out = []
        self._halt = threading.Event()
        self.mav = SimpleNamespace(param_request_read_send=self._on_read,
                                   heartbeat_send=lambda *a: None)
        link._mav = self
        self._t = threading.Thread(target=self._loop, daemon=True)

    def _on_read(self, ts, tc, pid, idx):
        name = pid.decode('ascii') if isinstance(pid, bytes) else pid
        self.requests.append((ts, tc, name, idx))
        if name in self.drop_first:
            self.drop_first.discard(name)
            return
        self._out.extend(self.noise(name))
        if name in self.params:
            self._out.append(pval(self.sysid, 1, name, self.params[name],
                                  ptype=self.types.get(name, 9), count=len(self.params)))

    def _loop(self):
        while not self._halt.is_set():
            try:
                cmd, arg = self.link._cmd_queue.get(timeout=0.01)
            except queue.Empty:
                pass
            else:
                self.link._dispatch_command(cmd, arg)
            while self._out:
                self.link._handle_message(self._out.pop(0))

    def names_requested(self, name):
        return sum(1 for r in self.requests if r[2] == name)

    def __enter__(self):
        self._t.start()
        return self

    def __exit__(self, *exc):
        self._halt.set()
        self._t.join(2)


def _f32(v):
    return struct.unpack('<f', struct.pack('<f', v))[0]


class TestReadParams:
    def test_values_types_and_target_component(self):
        link = make_link()
        params = {'CAM1_TYPE': 2.0, 'CAM1_TRIGG_DIST': _f32(18.85), 'LOG_BITMASK': 65535.0}
        types = {'CAM1_TYPE': 2, 'CAM1_TRIGG_DIST': 9, 'LOG_BITMASK': 6}
        t = {}
        with _FakeAutopilot(link, params, types=types) as fc:
            got = link.read_params(list(params), timeout_s=1.0, retries=0, types_out=t)
        assert got == params
        assert t == types
        # 送到 (飛控 sysid, 自駕儀 compid 1)，不是 pymavlink 的廣播 target_component 0
        assert {(r[0], r[1], r[3]) for r in fc.requests} == {(1, 1, -1)}
        assert sorted(r[2] for r in fc.requests) == sorted(params)
        assert values_match(18.85, got['CAM1_TRIGG_DIST'], t['CAM1_TRIGG_DIST'])

    def test_retry_resends_only_missing_names(self):
        link = make_link()
        params = {'CAM1_TYPE': 2.0, 'RELAY1_PIN': 54.0}
        with _FakeAutopilot(link, params, drop_first={'CAM1_TYPE'}) as fc:
            got = link.read_params(['CAM1_TYPE', 'RELAY1_PIN'], timeout_s=0.3, retries=2)
        assert got == params
        assert fc.names_requested('CAM1_TYPE') == 2
        assert fc.names_requested('RELAY1_PIN') == 1

    def test_unknown_param_is_none_after_all_rounds(self):
        link = make_link()
        with _FakeAutopilot(link, {'CAM1_TYPE': 2.0}) as fc:
            t0 = time.monotonic()
            got = link.read_params(['CAM1_TYPE', 'NO_SUCH_PARAM'], timeout_s=0.2, retries=2)
            dt = time.monotonic() - t0
        assert got == {'CAM1_TYPE': 2.0, 'NO_SUCH_PARAM': None}
        assert fc.names_requested('NO_SUCH_PARAM') == 3
        assert fc.names_requested('CAM1_TYPE') == 1
        assert 0.55 <= dt < 2.5

    def test_replies_from_other_sources_are_not_accepted(self):
        def noise(name):
            return [pval(2, 1, name, 99.0),          # 別架飛控
                    pval(1, 100, name, 77.0)]        # 同 sysid 的相機
        link = make_link()
        with _FakeAutopilot(link, {'CAM1_TYPE': 2.0}, noise=noise):
            got = link.read_params(['CAM1_TYPE'], timeout_s=1.0, retries=0)
        assert got == {'CAM1_TYPE': 2.0}

    def test_not_connected_returns_none_without_waiting(self):
        link = make_link()
        t0 = time.monotonic()
        got = link.read_params(['CAM1_TYPE', 'RELAY1_PIN'], timeout_s=5, retries=3)
        assert got == {'CAM1_TYPE': None, 'RELAY1_PIN': None}
        assert time.monotonic() - t0 < 0.5
        assert link._cmd_queue.empty()

    @pytest.mark.parametrize('kwargs,exc', [
        (dict(names='CAM1_TYPE'), TypeError),
        (dict(names=[]), ValueError),
        (dict(names=['A' * 17]), ValueError),
        (dict(names=['CAM\x00X']), ValueError),
        (dict(names=['參數']), ValueError),
        (dict(names=['CAM1_TYPE'], timeout_s=0), ValueError),
        (dict(names=['CAM1_TYPE'], timeout_s=float('nan')), ValueError),
        (dict(names=['CAM1_TYPE'], retries=-1), ValueError),
        (dict(names=['CAM1_TYPE'], retries=1.5), ValueError),
    ])
    def test_bad_arguments_are_rejected(self, kwargs, exc):
        link = make_link()
        with pytest.raises(exc):
            link.read_params(**kwargs)

    def test_calling_from_receive_thread_raises(self):
        link = make_link()
        link._mav = object()
        link._recv_thread_ident = threading.get_ident()
        with pytest.raises(RuntimeError):
            link.read_params(['CAM1_TYPE'])

    def test_link_closing_wakes_the_waiter(self):
        link = make_link()
        with _FakeAutopilot(link, {}):
            res = {}
            th = threading.Thread(target=lambda: res.update(
                link.read_params(['NO_SUCH_PARAM'], timeout_s=5, retries=3)))
            t0 = time.monotonic()
            th.start()
            time.sleep(0.2)
            link._link_closed = True
            th.join(3)
            dt = time.monotonic() - t0
        assert res == {'NO_SUCH_PARAM': None}
        assert dt < 1.5


class TestValuesMatch:
    @pytest.mark.parametrize('expected,actual,ptype,ok', [
        (0.1, _f32(0.1), 9, True),                 # float32 讀回
        (0.1, 0.1, 9, True),
        (0.3333333, _f32(1 / 3), 9, True),          # 只印 7 位有效數字：差 1 格
        (1.0, 1.0000001, 9, True),                  # 1 格
        (1.0, 1.0 + 2 * 2 ** -23, 9, False),        # 剛好 2 格
        (1.0, 1.0 - 2 ** -24, 9, True),             # 往下 1 格（1.0 以下格距減半）
        (1.0, 1.0000003, 9, False),                 # 3 格
        (4e-07, 0.0, 9, False),                     # %.6f 把它印成 0
        (0.0, -0.0, 9, True),
        (18.85, _f32(18.85), None, True),           # 型別未知 = REAL32 規則
        (2, 2.0, 2, True),                          # INT8
        (2.4, 2.0, 2, False),                       # 使用者給的值沒生效
        (3, 3.0000002, 6, False),                   # 整數型：完全相等
        (3, 3.0000002, 9, True),                    # 同一對在 REAL32 下差 1 格
        (16777217, 16777216.0, 6, True),            # float32 的整數極限，協定本身分不出
        (float('inf'), float('inf'), 9, True),
        (float('inf'), 3.4e38, 9, False),
        (float('nan'), float('nan'), 9, False),
        (1.0, None, 9, False),
        (None, 1.0, 9, False),
        (1, 1.0, 'INT8', True),
        (1, 1.0000001, 'INT8', False),
        (1, 1.0000001, 'REAL32', True),
        (1, 1.0000001, 'MAV_PARAM_TYPE_UINT16', False),
    ])
    def test_cases(self, expected, actual, ptype, ok):
        assert values_match(expected, actual, ptype) is ok

    def test_unknown_type_name_is_rejected(self):
        with pytest.raises(ValueError):
            values_match(1.0, 1.0, 'FLOAT')


# ═════════════════════════════════════════════════════════════════════
# ④ tlog
# ═════════════════════════════════════════════════════════════════════
def _records(data: bytes, bufs):
    """依已知的封包長度切出 (時間戳, 封包) —— 順便驗證沒有多寫任何一個位元組。"""
    out, off = [], 0
    for b in bufs:
        ts = struct.unpack('>Q', data[off:off + 8])[0]
        out.append((ts, data[off + 8:off + 8 + len(b)]))
        off += 8 + len(b)
    assert off == len(data)
    return out


class TestTlog:
    def test_nothing_is_written_without_start_tlog(self, tmp_path, monkeypatch):
        monkeypatch.chdir(tmp_path)
        link = make_link()
        link._tlog_hook(None, hb(1, 1))
        link._handle_message(hb(1, 1))
        assert list(tmp_path.iterdir()) == []
        assert link.tlog_status() == {'active': False, 'path': None, 'messages': 0,
                                      'bytes': 0, 'error': None}

    def test_framing_is_standard_tlog(self, tmp_path):
        link = make_link()
        p = tmp_path / 'sub' / 'uav1.tlog'
        assert link.start_tlog(p) == str(p)
        msgs = [cam_hb(), cfb(img=7), pval(1, 1, 'CAM1_TYPE', 2.0, ptype=2)]
        t0 = time.time()
        for m in msgs:
            link._tlog_hook(None, m)
        t1 = time.time()
        st = link.stop_tlog()
        bufs = [bytes(m.get_msgbuf()) for m in msgs]
        assert st['messages'] == 3 and st['bytes'] == sum(8 + len(b) for b in bufs)
        assert st['active'] is False and st['path'] == str(p)
        recs = _records(p.read_bytes(), bufs)
        assert [r[1] for r in recs] == bufs
        for ts, _ in recs:
            assert ts & 3 == 0
            assert int(t0 * 1e6) - 4 <= ts <= int(t1 * 1e6)
        assert recs[0][0] <= recs[1][0] <= recs[2][0]
        # pymavlink 自己的 tlog 讀取器讀得懂
        log = mavutil.mavlogfile(str(p))
        got = []
        while True:
            m = log.recv_match()
            if m is None:
                break
            got.append((m.get_type(), m.get_srcSystem(), m.get_srcComponent()))
        log.close()
        assert got == [('HEARTBEAT', 1, 100), ('CAMERA_FEEDBACK', 1, 1), ('PARAM_VALUE', 1, 1)]

    def test_bad_data_and_after_stop_are_not_written(self, tmp_path):
        link = make_link()
        p = tmp_path / 'a.tlog'
        link.start_tlog(p)
        link._tlog_hook(None, SimpleNamespace(get_type=lambda: 'BAD_DATA',
                                              get_msgbuf=lambda: b'\xfe\x00junk'))
        link._tlog_hook(None, hb(1, 1))
        link.stop_tlog()
        size = p.stat().st_size
        assert size == 8 + len(hb(1, 1).get_msgbuf())
        link._tlog_hook(None, hb(1, 1))
        assert p.stat().st_size == size
        assert link.stop_tlog()['messages'] == 1

    def test_switching_and_appending(self, tmp_path):
        link = make_link()
        a, b = tmp_path / 'a.tlog', tmp_path / 'b.tlog'
        rec = 8 + len(hb(1, 1).get_msgbuf())
        link.start_tlog(a)
        link._tlog_hook(None, hb(1, 1))
        link.start_tlog(b)                    # 換檔：a 關掉
        link._tlog_hook(None, hb(1, 1))
        link.start_tlog(a)                    # 同路徑再開：附加，不覆蓋
        link._tlog_hook(None, hb(1, 1))
        link.stop_tlog()
        assert a.stat().st_size == 2 * rec
        assert b.stat().st_size == rec

    def test_write_failure_stops_logging_loudly_without_raising(self, tmp_path):
        link = make_link()
        link.start_tlog(tmp_path / 'c.tlog')
        real = link._tlog_file

        class Broken:
            def write(self, _b):
                raise OSError('disk full')

            def close(self):
                pass

        link._tlog_file = Broken()
        try:
            link._tlog_hook(None, hb(1, 1))     # 不可以丟例外（會把連線帶掉）
        finally:
            real.close()
        st = link.tlog_status()
        assert st['active'] is False and 'disk full' in st['error']
        assert any(sev == 3 and 'tlog' in t for sev, t in link._got_status)
        link._tlog_hook(None, hb(1, 1))         # 停了就停了，不再嘗試
        assert link.tlog_status()['messages'] == 0
