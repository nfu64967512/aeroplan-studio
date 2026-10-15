# -*- coding: utf-8 -*-
"""AeroPlan「寫入任務到飛控」（SITLLink.upload_mission_items）—— 值層級鎖。

═══ 這個檔在防什麼 ═══

寫入路徑是本 repo 最容易出「靜默失敗」的形狀：飛控**明確拒收**、**只收一半**、
**收了但內容被改**，三種在舊路徑（`_send_mission`）上都和成功長得一模一樣，
而下一步「切 AUTO」會照常送出去，飛的是**別的任務**。所以這裡每一種失敗
都要先證明假飛控真的做得出來（`TestTheFakeCanActuallyMisbehave`），再證明
`upload_mission_items` 對它大聲失敗。

⚠️ 假飛控自己也是我寫的：兩個都是我寫的東西互相同意，不是驗證。所以：
   * `TestReadbackRules` 用**手寫、從 ArduPilot 原始碼抄來的數值**直接餵比對函式，
     不經過假飛控的「儲存轉換」；
   * 另有 `TestLoopbackAgainstMockAutopilot`：對 `tools/mock_autopilot.py`
     （另一份獨立實作的協定伺服端）在 127.0.0.1 UDP 上跑完整來回；
   * 變異測試（在複本上改 sitl_link.py，確認這裡的測試會變紅）另外做。

⚠️ 絕不碰真實裝置：全部用記憶體內假物件；迴圈測試只連 127.0.0.1 的
   臨時埠，且子行程內明確斷言連線字串。
"""
from __future__ import annotations

import ast
import json
import math
import os
import pathlib
import struct
import subprocess
import sys
import textwrap

import pytest
from PyQt6.QtCore import QCoreApplication

from mission import sitl_link as sl
from mission.sitl_link import SITLLink

_app = QCoreApplication.instance() or QCoreApplication([])

ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC = ROOT / 'mission' / 'sitl_link.py'

NAV_WAYPOINT = 16
NAV_LOITER_TIME = 19
DO_CHANGE_SPEED = 178
DO_MOUNT_CONTROL = 205          # 不在寬鬆表 ⇒ 參數嚴格比對
DO_JUMP = 177                   # ⚠️ 不在 mission.waypoint.MAVCommand 裡

FRAME_GLOBAL, FRAME_MISSION, FRAME_REL, FRAME_REL_INT = 0, 2, 3, 6
ACK_OK, ACK_ERROR, ACK_UNSUPPORTED_FRAME, ACK_NO_SPACE = 0, 1, 2, 4
ACK_INVALID_SEQ, ACK_DENIED, ACK_CANCELLED = 13, 14, 15


def f32(x: float) -> float:
    return struct.unpack('f', struct.pack('f', x))[0]


# ══════════════════════════════════════════════════════════════════
# 假時鐘：取代 mission.sitl_link 裡的 `time` 模組
# ══════════════════════════════════════════════════════════════════
class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def time(self):
        return self.t

    def monotonic(self):
        return self.t

    def sleep(self, s):
        self.t += s


@pytest.fixture
def clock(monkeypatch):
    c = FakeClock()
    monkeypatch.setattr(sl, 'time', c)
    return c


# ══════════════════════════════════════════════════════════════════
# 假飛控：ArduPilot mission 寫入協定的伺服端（會故障）
# ══════════════════════════════════════════════════════════════════
class _Msg:
    def __init__(self, mtype, src=1, **fields):
        self._mtype = mtype
        self._src = src
        for k, v in fields.items():
            setattr(self, k, v)

    def get_type(self):
        return self._mtype

    def get_srcSystem(self):
        return self._src


_FC_LOCATION_CMDS = {16, 17, 18, 19, 21, 22, 31, 84, 85}
_FC_FRAME_BACK = {0: 0, 2: 0, 5: 0, 3: 3, 6: 3, 10: 10, 11: 10}


def fc_store(w: dict) -> dict:
    """假飛控「存進 EEPROM 再讀回」的轉換（依 AP_Mission.cpp 的行為）。"""
    cmd = w['command']
    out = dict(seq=w['seq'], command=cmd, current=0, autocontinue=1)
    if cmd in _FC_LOCATION_CMDS:
        out['frame'] = _FC_FRAME_BACK[w['frame']]
        out['x'], out['y'] = w['x'], w['y']
        cm = int(f32(f32(w['z']) * 100.0))          # alt 以 cm 的 int32 存（截斷）
        out['z'] = f32(cm * 0.01)
    else:
        out['frame'] = 0
        out['x'] = out['y'] = 0
        out['z'] = 0.0
    p = [w['param1'], w['param2'], w['param3'], w['param4']]
    if cmd == 16:           # Plane：只存 param2/3 的 uint8
        p = [0.0, float(min(255, int(p[1]))), float(min(255, int(p[2]))), 0.0]
    elif cmd == NAV_LOITER_TIME:
        p = [float(int(p[0])), 0.0, 1.0, 0.0]
    for i in range(4):
        out[f'param{i + 1}'] = f32(p[i])
    return out


class _FakeMavApi:
    def __init__(self, fc):
        self.fc = fc

    def heartbeat_send(self, *a, **kw):
        self.fc.heartbeats.append(self.fc.clock.t)
        self.fc.sent.append(('heartbeat',))

    def mission_clear_all_send(self, *a, **kw):
        self.fc.sent.append(('clear_all',))

    def mission_count_send(self, ts, tc, count, mission_type=0):
        fc = self.fc
        fc.sent.append(('count', count, mission_type))
        # ⚠️ finding（major，test_bite）：舊版這裡完全沒記 ts/tc，所以就算
        # `_write_mission_items` 把 target_system/target_component 兩個參數
        # 位置調換，這支假 FC 也測不出來——見 TestTargetSystemComponent，
        # 那裡用兩個「不相等」的值（不像 fc.target_system/target_component
        # 預設都湊巧是 1）才踩得到這種調換。不放進上面那個 `('count', ...)`
        # tuple 本身，是因為 test_wire_conversation_shape 已經對它做過精確
        # tuple 相等比對，直接加欄位會把那個測試改壞。
        fc.count_sends.append({'target_system': ts, 'target_component': tc,
                               'count': count, 'mission_type': mission_type})
        if fc.raise_on == 'count':
            raise RuntimeError('假的 socket 錯誤')
        fc.counts_seen += 1
        if fc.counts_seen <= fc.ignore_counts:
            return
        fc.expected_n = count
        fc.received = {}
        fc.next_seq = 0
        fc.complete = False
        fc._advance()

    def mission_item_int_send(self, ts, tc, seq, frame, command, current,
                              autocontinue, p1, p2, p3, p4, x, y, z,
                              mission_type=0):
        fc = self.fc
        # target_system/target_component 放進 w 字典本身（而不是另外開一個
        # list）——items_sent() 已經回傳這個字典給呼叫端，多加欄位不會動到
        # 既有測試對個別欄位（w['frame'] 等）的比對，只有直接對整個字典做
        # exact-equality 的測試才會受影響，而這個檔案裡目前沒有那種寫法。
        w = dict(seq=seq, frame=frame, command=command, current=current,
                 autocontinue=autocontinue, param1=p1, param2=p2, param3=p3,
                 param4=p4, x=x, y=y, z=z, mission_type=mission_type,
                 target_system=ts, target_component=tc)
        fc.sent.append(('item', w))
        fc.clock.t += fc.latency
        if fc.complete or fc.expected_n is None:
            fc.inbox.append(_Msg('MISSION_ACK', type=ACK_ERROR, mission_type=0))
            return
        if seq != fc.next_seq:
            # ArduPilot：seq != request_i ⇒ ACK(INVALID_SEQUENCE)，傳輸不中止
            fc.inbox.append(_Msg('MISSION_ACK', type=ACK_INVALID_SEQ,
                                 mission_type=0))
            return
        if seq in fc.drop_items:
            fc.drop_items = fc.drop_items - {seq}    # 只掉第一次
            return                                    # 掉了：靠重問
        fc.received[seq] = w
        fc.next_seq += 1
        fc._advance()
        if seq in fc.transport_dup_items:
            # 模擬「傳輸層本身」把 client 剛剛送出的這一筆複製了一份送進飛控
            # （UDP 封包重複、半雙工數傳鏈路重傳）—— client 只呼叫了這個函式
            # 一次，但飛控收到兩次：第二次到達時 seq 已經落後於 next_seq，
            # ArduPilot 的反應是回 INVALID_SEQUENCE、傳輸不中止。這跟「client
            # 自己重送」（見上面 dup_requests／`seq != fc.next_seq` 分支）在
            # client 端完全看不出差異，但成因不同：這裡 client 一次都沒有
            # 重複作答，dup_answers 不會被計到。
            fc.transport_dup_items = fc.transport_dup_items - {seq}  # 只複製一次
            fc.inbox.append(_Msg('MISSION_ACK', type=ACK_INVALID_SEQ,
                                 mission_type=0))

    # ── 讀回
    def mission_request_list_send(self, ts, tc, mission_type=0):
        fc = self.fc
        fc.sent.append(('request_list', mission_type))
        fc.t_readback = fc.clock.t
        if fc.raise_on == 'readback':
            raise RuntimeError('讀回時斷線')
        rb = fc.readback_items()
        n = fc.readback_count if fc.readback_count is not None else len(rb)
        fc.inbox.append(_Msg('MISSION_COUNT', count=n))

    def mission_request_int_send(self, ts, tc, seq, mission_type=0):
        fc = self.fc
        fc.sent.append(('request_int', seq))
        fc.clock.t += fc.readback_latency
        if seq in fc.foreign_readback_at:
            # ⚠️ fan-in 拓樸下的真實情境：另一架飛控（src≠我們的 target_system）
            # 剛好在同一時間點也在回答「seq={seq} 的內容是什麼」——這裡讓它的
            # 回覆先進收件匣，模擬「搶先抵達」，是對這個 finding 最不利的排序
            # （比「後到」更容易被誤收）。故意跟我們自己這架的 seq 對齊，
            # 才會真的撞上 `int(msg.seq) != seq` 這道舊有防線本來就會擋掉的
            # 「seq 對不上」情況——這裡要測的是「seq 對得上、但來源是別架」。
            f = fc.foreign_readback_at[seq]
            fc.inbox.append(_Msg('MISSION_ITEM_INT', src=99, seq=seq,
                                 command=f['command'], frame=f['frame'],
                                 x=f['x'], y=f['y'], z=f['z'],
                                 param1=0.0, param2=0.0, param3=0.0, param4=0.0,
                                 current=0, autocontinue=1))
        rb = fc.readback_items()
        if seq in fc.readback_drop_seqs or seq >= len(rb):
            return
        r = rb[seq]
        fc.inbox.append(_Msg('MISSION_ITEM_INT', seq=r['seq'],
                             command=r['command'], frame=r['frame'],
                             x=r['x'], y=r['y'], z=r['z'],
                             param1=r['param1'], param2=r['param2'],
                             param3=r['param3'], param4=r['param4'],
                             current=r['current'],
                             autocontinue=r['autocontinue']))

    def mission_ack_send(self, ts, tc, type, mission_type=0):
        self.fc.sent.append(('ack', type))


class FakeFC:
    """假 pymavlink 連線 + 假 ArduPilot。旋鈕都是「飛控端的一種壞行為」。"""

    def __init__(self, clock, *, request_style='INT', latency=0.0,
                 ignore_counts=0, dup_requests=None, bogus_request=None,
                 ack_at=None, early_ack_at=None, no_final_ack=False,
                 drop_items=(), resend_on_silence=True, stale_inbox=(),
                 foreign=(), readback_mutate=None, readback_drop_seqs=(),
                 readback_count=None, raise_on=None, home=(25.0, 121.0, 10.0),
                 readback_latency=0.0, transport_dup_items=(),
                 target_system=1, target_component=1,
                 foreign_readback_at=None):
        self.clock = clock
        # ⚠️ 預設都是 1：這是舊測試檔案裡大部分案例的設定，但也正好是
        # target_system/target_component 被調換也測不出來的原因（1 跟 1
        # 調換還是 1）。TestTargetSystemComponent 會刻意傳兩個不相等的值。
        self.target_system = target_system
        self.target_component = target_component
        self.request_style = request_style
        self.latency = latency
        self.ignore_counts = ignore_counts
        self.dup_requests = dict(dup_requests or {})     # seq → 額外重複請求次數
        self.bogus_request = dict(bogus_request or {})   # 該問 seq k 時改問別的
        self.ack_at = dict(ack_at or {})                 # 該問 seq k 時改回 ACK(code)
        self.early_ack_at = early_ack_at                 # 該問 seq k 時改回 ACCEPTED
        self.no_final_ack = no_final_ack
        self.drop_items = set(drop_items)
        # seq → client 只送一次，但「傳輸層」自己又複製一份送進飛控
        # （見 _FakeMavApi.mission_item_int_send）：模擬 UDP/半雙工鏈路的
        # 封包重複投遞，跟 client 自己造成的 dup_requests 是不同成因。
        self.transport_dup_items = set(transport_dup_items)
        self.resend_on_silence = resend_on_silence
        self.readback_mutate = readback_mutate
        self.readback_drop_seqs = set(readback_drop_seqs)
        self.readback_count = readback_count
        self.readback_latency = readback_latency
        # seq → 別架飛控（src=99）對同一個 seq 的「搶答」內容；見
        # _FakeMavApi.mission_request_int_send。
        self.foreign_readback_at = dict(foreign_readback_at or {})
        self.t_readback = None
        self.raise_on = raise_on
        self.home = home
        self.inbox = list(stale_inbox) + list(foreign)
        self.sent = []
        self.count_sends = []   # 見 _FakeMavApi.mission_count_send 的註解
        self.heartbeats = []
        self.counts_seen = 0
        self.expected_n = None
        self.received = {}
        self.next_seq = 0
        self.complete = False
        self.pending = None          # (seq, 上次請求時刻)
        self.recv_calls = 0
        self.mav = _FakeMavApi(self)

    # ── FC 端狀態機
    def _request(self, seq):
        mtype = 'MISSION_REQUEST_INT' if self.request_style == 'INT' else 'MISSION_REQUEST'
        self.inbox.append(_Msg(mtype, seq=seq, mission_type=0))
        self.pending = (seq, self.clock.t)

    def _advance(self):
        k = self.next_seq
        n = self.expected_n
        if k in self.ack_at and not self.complete:
            self.inbox.append(_Msg('MISSION_ACK', type=self.ack_at[k], mission_type=0))
            self.pending = None
            return
        if self.early_ack_at is not None and k == self.early_ack_at:
            self.inbox.append(_Msg('MISSION_ACK', type=ACK_OK, mission_type=0))
            self.pending = None
            return
        if k >= n:
            self.complete = True
            self.pending = None
            if not self.no_final_ack:
                self.inbox.append(_Msg('MISSION_ACK', type=ACK_OK, mission_type=0))
            return
        seq = self.bogus_request.get(k, k)
        self._request(seq)
        for _ in range(self.dup_requests.get(k, 0)):
            self._request(seq)

    def readback_items(self):
        n = self.expected_n or 0
        items = [dict(seq=0, command=16, frame=0, x=int(self.home[0] * 1e7),
                      y=int(self.home[1] * 1e7), z=self.home[2], param1=0.0,
                      param2=0.0, param3=0.0, param4=0.0, current=1,
                      autocontinue=1)]
        for s in range(1, n):
            items.append(fc_store(self.received[s]) if s in self.received
                         else dict(seq=s, command=16, frame=3, x=0, y=0, z=0.0,
                                   param1=0.0, param2=0.0, param3=0.0,
                                   param4=0.0, current=0, autocontinue=1))
        if self.readback_mutate:
            items = self.readback_mutate(items)
        return items

    # ── pymavlink 介面
    def recv_match(self, type=None, blocking=False, timeout=None):
        self.recv_calls += 1
        assert self.recv_calls < 200000, '失控迴圈（假時鐘沒有前進？）'
        want = [type] if isinstance(type, str) else list(type or [])

        def find():
            for i, m in enumerate(self.inbox):
                if not want or m.get_type() in want:
                    return self.inbox.pop(i)
            return None

        m = find()
        if m is not None or not blocking:
            return m
        self.clock.t += float(timeout or 0.0)
        if (self.resend_on_silence and self.pending is not None
                and self.clock.t - self.pending[1] >= 1.0):
            self._request(self.pending[0])
        return find()

    # ── 統計
    def items_sent(self):
        return [w for (k, *rest) in self.sent if k == 'item' for w in rest]

    def kinds(self):
        return [s[0] for s in self.sent]


# ══════════════════════════════════════════════════════════════════
# 任務資料
# ══════════════════════════════════════════════════════════════════
def wp(seq, lat, lon, alt, *, cmd=NAV_WAYPOINT, frame=FRAME_REL, **params):
    d = dict(seq=seq, command=cmd, frame=frame, lat=lat, lon=lon, alt=alt,
             param1=0.0, param2=0.0, param3=0.0, param4=0.0,
             current=0, autocontinue=1)
    d.update(params)
    return d


def mission3():
    return [
        wp(0, 25.0777, 121.2325, 32.1, frame=FRAME_GLOBAL),
        wp(1, 25.0800, 121.2000, 100.0),
        wp(2, 25.0900, 121.1705, 120.0, param2=10.0),
    ]


def mission_n(n, latency=0.0):
    it = [wp(0, 25.0, 121.0, 10.0, frame=FRAME_GLOBAL)]
    for i in range(1, n):
        it.append(wp(i, 25.0 + i * 1e-3, 121.0 + i * 1e-3, 100.0))
    return it


class Rig:
    def __init__(self, clock, fc, link, results, status):
        self.clock, self.fc, self.link = clock, fc, link
        self.results, self.status = results, status

    def upload(self, items, verify=True):
        self.link._send_mission_items(items, verify)
        return self.results


def mk(clock, **fckw):
    fc = FakeFC(clock, **fckw)
    link = SITLLink('udpin:0.0.0.0:1', sysid_label=7)
    link._mav = fc
    results, status = [], []
    link.mission_uploaded.connect(lambda s, ok, m: results.append((s, ok, m)))
    link.status_text.connect(lambda sev, txt: status.append((sev, txt)))
    return Rig(clock, fc, link, results, status)


# ══════════════════════════════════════════════════════════════════
class TestTheFakeCanActuallyMisbehave:
    """⚠️⚠️ 先證明儀器打得出每一種故障。「守衛沒咬」和「故障根本沒發生」
    在報表上長得一模一樣，而後者會被讀成前者的證據。"""

    def test_normal_flow_requests_each_seq_then_acks(self, clock):
        fc = FakeFC(clock)
        fc.mav.mission_count_send(1, 1, 2, 0)
        assert fc.inbox[-1].get_type() == 'MISSION_REQUEST_INT'
        assert fc.inbox[-1].seq == 0
        fc.mav.mission_item_int_send(1, 1, 0, 3, 16, 0, 1, 0, 0, 0, 0, 1, 1, 1.0, 0)
        assert fc.inbox[-1].seq == 1
        fc.mav.mission_item_int_send(1, 1, 1, 3, 16, 0, 1, 0, 0, 0, 0, 1, 1, 1.0, 0)
        assert fc.inbox[-1].get_type() == 'MISSION_ACK' and fc.inbox[-1].type == 0

    def test_request_style_non_int_really_happens(self, clock):
        fc = FakeFC(clock, request_style='REQUEST')
        fc.mav.mission_count_send(1, 1, 2, 0)
        assert fc.inbox[-1].get_type() == 'MISSION_REQUEST'

    def test_silent_fc_really_stays_silent(self, clock):
        fc = FakeFC(clock, ignore_counts=1)
        fc.mav.mission_count_send(1, 1, 2, 0)
        assert fc.inbox == []
        fc.mav.mission_count_send(1, 1, 2, 0)
        assert fc.inbox, '第二次 COUNT 應該有反應（ignore_counts 只吃第一次）'

    def test_duplicate_request_really_duplicates(self, clock):
        fc = FakeFC(clock, dup_requests={0: 1})
        fc.mav.mission_count_send(1, 1, 3, 0)
        assert [m.seq for m in fc.inbox] == [0, 0]

    def test_transport_dup_items_really_produces_unexplained_invalid_seq(self, clock):
        """證明假 FC 真的能重現「client 只送一次、傳輸層複製成兩次」，
        跟 dup_requests（client 自己重送）是不同機制、不同信號。"""
        fc = FakeFC(clock, transport_dup_items={0})
        fc.mav.mission_count_send(1, 1, 2, 0)
        assert fc.inbox[-1].get_type() == 'MISSION_REQUEST_INT' and fc.inbox[-1].seq == 0
        fc.mav.mission_item_int_send(1, 1, 0, 3, 16, 0, 1, 0, 0, 0, 0, 1, 1, 1.0, 0)
        # client 只呼叫了一次 mission_item_int_send，飛控卻收到一個 INVALID_SEQUENCE
        # （第二次「複製」的效果）——不是靠 client 自己重複作答湊出來的。
        kinds = [m.get_type() for m in fc.inbox]
        assert kinds.count('MISSION_ACK') == 1
        assert fc.inbox[-1].type == ACK_INVALID_SEQ

    def test_bogus_request_really_out_of_range(self, clock):
        fc = FakeFC(clock, bogus_request={1: 99})
        fc.mav.mission_count_send(1, 1, 3, 0)
        fc.mav.mission_item_int_send(1, 1, 0, 3, 16, 0, 1, 0, 0, 0, 0, 1, 1, 1.0, 0)
        assert fc.inbox[-1].seq == 99

    @pytest.mark.parametrize('code', [ACK_ERROR, ACK_NO_SPACE, ACK_DENIED,
                                      ACK_INVALID_SEQ, ACK_UNSUPPORTED_FRAME,
                                      ACK_CANCELLED])
    def test_error_ack_really_sent(self, clock, code):
        fc = FakeFC(clock, ack_at={1: code})
        fc.mav.mission_count_send(1, 1, 3, 0)
        fc.mav.mission_item_int_send(1, 1, 0, 3, 16, 0, 1, 0, 0, 0, 0, 1, 1, 1.0, 0)
        assert fc.inbox[-1].get_type() == 'MISSION_ACK' and fc.inbox[-1].type == code

    def test_early_ack_really_early(self, clock):
        fc = FakeFC(clock, early_ack_at=1)
        fc.mav.mission_count_send(1, 1, 4, 0)
        fc.mav.mission_item_int_send(1, 1, 0, 3, 16, 0, 1, 0, 0, 0, 0, 1, 1, 1.0, 0)
        assert fc.inbox[-1].get_type() == 'MISSION_ACK' and fc.next_seq == 1

    def test_no_final_ack_really_missing(self, clock):
        fc = FakeFC(clock, no_final_ack=True)
        fc.mav.mission_count_send(1, 1, 1, 0)
        fc.inbox.clear()                                       # 丟掉對 seq 0 的請求
        fc.mav.mission_item_int_send(1, 1, 0, 3, 16, 0, 1, 0, 0, 0, 0, 1, 1, 1.0, 0)
        assert fc.inbox == [] and fc.complete

    def test_stale_second_send_gets_invalid_sequence(self, clock):
        fc = FakeFC(clock)
        fc.mav.mission_count_send(1, 1, 3, 0)
        for _ in range(2):
            fc.mav.mission_item_int_send(1, 1, 0, 3, 16, 0, 1, 0, 0, 0, 0, 1, 1, 1.0, 0)
        assert fc.inbox[-1].get_type() == 'MISSION_ACK'
        assert fc.inbox[-1].type == ACK_INVALID_SEQ

    def test_lost_item_is_rerequested_after_silence(self, clock):
        fc = FakeFC(clock, drop_items={0})
        fc.mav.mission_count_send(1, 1, 2, 0)
        fc.recv_match(type=['MISSION_REQUEST_INT'])            # 取走第一個請求
        fc.mav.mission_item_int_send(1, 1, 0, 3, 16, 0, 1, 0, 0, 0, 0, 1, 1, 1.0, 0)
        assert fc.inbox == [] and fc.next_seq == 0             # 掉了
        assert fc.recv_match(type=['MISSION_REQUEST_INT'], blocking=True,
                             timeout=1.5).seq == 0             # 靜默後重問

    def test_readback_mutations_really_apply(self, clock):
        fc = FakeFC(clock, readback_mutate=lambda its: its[:-1])
        fc.expected_n = 3
        fc.received = {s: dict(seq=s, frame=3, command=16, x=1, y=1, z=1.0,
                               param1=0.0, param2=0.0, param3=0.0, param4=0.0)
                       for s in (1, 2)}
        assert len(fc.readback_items()) == 2

    def test_fc_store_really_quantises_like_the_firmware(self):
        w = dict(seq=1, frame=FRAME_MISSION, command=16, x=1, y=2, z=32.1,
                 param1=5.0, param2=10.7, param3=3.2, param4=90.0)
        out = fc_store(w)
        assert out['frame'] == 0 and out['param2'] == 10.0 and out['param4'] == 0.0
        assert out['z'] == pytest.approx(32.09)

    def test_stale_messages_and_foreign_messages_are_in_the_inbox(self, clock):
        fc = FakeFC(clock, stale_inbox=[_Msg('MISSION_ACK', type=0)],
                    foreign=[_Msg('MISSION_ACK', src=2, type=ACK_ERROR)])
        assert len(fc.inbox) == 2


# ══════════════════════════════════════════════════════════════════
class TestHappyPath:
    def test_success_emits_once_with_readable_message(self, clock):
        r = mk(clock)
        res = r.upload(mission3())
        assert len(res) == 1
        sysid, ok, msg = res[0]
        assert sysid == 7 and ok is True
        assert '已寫入並讀回驗證 3 項相符' in msg

    def test_wire_conversation_shape(self, clock):
        r = mk(clock)
        r.upload(mission3())
        k = r.fc.kinds()
        assert k[0] == 'count'
        assert r.fc.sent[0] == ('count', 3, 0), 'count 與 MISSION 類型（0）'
        assert k.count('item') == 3
        assert 'request_list' in k, 'verify=True 應在 ACK 之後讀回'
        assert k.index('request_list') > max(i for i, x in enumerate(k) if x == 'item')

    def test_never_clears_the_mission_first(self, clock):
        """ArduPilot 的 COUNT 自己會截斷；先 clear_all 在飛行中會被拒
        （AP_Mission::clear 對「已解鎖且執行中」回 false）。"""
        r = mk(clock)
        r.upload(mission3())
        assert 'clear_all' not in r.fc.kinds()

    @pytest.mark.parametrize('style', ['INT', 'REQUEST'])
    def test_both_request_styles_are_answered(self, clock, style):
        r = mk(clock, request_style=style)
        res = r.upload(mission3())
        assert res[0][1] is True, res
        assert len(r.fc.items_sent()) == 3

    def test_unsorted_but_complete_is_sorted_not_renumbered(self, clock):
        r = mk(clock)
        items = mission3()
        items.reverse()
        res = r.upload(items)
        assert res[0][1] is True, res
        sent = r.fc.items_sent()
        assert [w['seq'] for w in sent] == [0, 1, 2]
        assert sent[1]['x'] == 250800000        # 原本 seq 1 的座標仍在 seq 1

    def test_verify_false_skips_readback_and_says_so(self, clock):
        r = mk(clock)
        res = r.upload(mission3(), verify=False)
        assert res[0][1] is True
        assert 'request_list' not in r.fc.kinds()
        assert '未讀回驗證' in res[0][2]

    def test_status_text_reports_success_at_info_level(self, clock):
        r = mk(clock)
        r.upload(mission3())
        assert r.status and r.status[-1][0] == 6 and '寫入完成' in r.status[-1][1]


# ══════════════════════════════════════════════════════════════════
class TestWhatGoesOnTheWire:
    @pytest.mark.parametrize('frame', [0, 2, 3, 5, 6, 10, 11])
    def test_frame_in_the_item_is_the_frame_on_the_wire(self, clock, frame):
        """舊路徑會把 cmd 不在白名單的一律改成 3、白名單內改成 2
        —— 靜默改寫檔案裡的 frame。這裡逐項照原樣送。"""
        items = mission3()
        items[1]['frame'] = frame
        r = mk(clock)
        r.upload(items, verify=False)
        assert r.fc.items_sent()[1]['frame'] == frame

    def test_do_command_frame_is_not_guessed(self, clock):
        items = mission3() + [wp(3, 0.0, 0.0, 0.0, cmd=DO_CHANGE_SPEED,
                                 frame=FRAME_REL, param2=95.0)]
        r = mk(clock)
        r.upload(items, verify=False)
        assert r.fc.items_sent()[3]['frame'] == FRAME_REL      # 舊路徑會改成 2
        items2 = mission3()
        items2[1]['frame'] = FRAME_GLOBAL                       # 舊路徑會改成 3
        r2 = mk(clock)
        r2.upload(items2, verify=False)
        assert r2.fc.items_sent()[1]['frame'] == FRAME_GLOBAL

    def test_mission_type_is_explicitly_mission(self, clock):
        r = mk(clock)
        r.upload(mission3(), verify=False)
        assert all(w['mission_type'] == 0 for w in r.fc.items_sent())

    def test_current_is_always_zero(self, clock):
        """current==2/3 在 ArduPilot 是 GUIDED 單點 / 只改高度：不入任務、
        立刻 ACK，會劫持整個傳輸（GCS_Common.cpp::handle_mission_item）。"""
        items = mission3()
        for it, c in zip(items, (1, 2, 3)):
            it['current'] = c
        r = mk(clock)
        r.upload(items, verify=False)
        assert [w['current'] for w in r.fc.items_sent()] == [0, 0, 0]

    def test_params_and_command_pass_through(self, clock):
        items = mission3()
        items[1].update(command=DO_JUMP, param1=3.0, param2=2.0)
        items[2].update(param1=1.5, param3=-4.0, param4=90.0)
        r = mk(clock)
        r.upload(items, verify=False)
        s = r.fc.items_sent()
        assert (s[1]['command'], s[1]['param1'], s[1]['param2']) == (177, 3.0, 2.0)
        assert (s[2]['param1'], s[2]['param3'], s[2]['param4']) == (1.5, -4.0, 90.0)

    def test_autocontinue_defaults_to_one(self, clock):
        items = mission3()
        for it in items:
            del it['autocontinue']
        r = mk(clock)
        r.upload(items, verify=False)
        assert [w['autocontinue'] for w in r.fc.items_sent()] == [1, 1, 1]

    @pytest.mark.parametrize('deg, expect', [
        (25.08, 250800000),                # int() 得 250799999（浮點 250799999.99999997）
        (0.0029, 29000),                   # int() 得 28999
        (-0.123456789, -1234568),          # 負數：int() 往 0 截斷得 -1234567
        (1.00000004999, 10000000),         # .5 邊界下方
        (1.00000005001, 10000001),         # .5 邊界上方：int() 得 10000000
        (121.2325, 1212325000),
    ])
    def test_lat_lon_are_rounded_not_truncated(self, clock, deg, expect):
        items = mission3()
        if abs(deg) <= 90:
            items[1]['lat'] = deg
        items[1]['lon'] = deg
        r = mk(clock)
        r.upload(items, verify=False)
        w = r.fc.items_sent()[1]
        assert w['y'] == expect
        if abs(deg) <= 90:
            assert w['x'] == expect

    def test_the_truncation_bug_is_real(self):
        """證明上面那組數值真的抓得到 int()：否則整條測試在空轉。"""
        assert int(25.08 * 1e7) != round(25.08 * 1e7)
        assert int(1.00000005001 * 1e7) != round(1.00000005001 * 1e7)

    def test_x_y_are_ints_and_alt_is_float(self, clock):
        r = mk(clock)
        r.upload(mission3(), verify=False)
        w = r.fc.items_sent()[1]
        assert isinstance(w['x'], int) and isinstance(w['y'], int)
        assert isinstance(w['z'], float) and w['z'] == 100.0


# ══════════════════════════════════════════════════════════════════
class TestTargetSystemComponent:
    """finding（major，test_bite）：target_system/target_component 的參數順序
    在 `_write_mission_items` 裡完全沒有測試覆蓋——舊版 `_FakeMavApi` 收下
    ts/tc 卻從不記錄、也從不比對，而且大部分測試用的 FakeFC.target_system/
    target_component 剛好都是 1，兩者調換了也看不出差異。這裡用兩個刻意
    「不相等」的值（1 / 77，跟原始 finding 的重現手法一致），確保任何一次
    調換都會讓某個欄位對不上。"""

    def test_every_count_and_item_send_uses_the_links_target_system_and_component(self, clock):
        fc = FakeFC(clock, target_system=1, target_component=77)
        link = SITLLink('udpin:0.0.0.0:1', sysid_label=7)
        link._mav = fc
        results = []
        link.mission_uploaded.connect(lambda s, ok, m: results.append((s, ok, m)))
        link._send_mission_items(mission3(), verify=True)
        assert results and results[0][1] is True, results

        assert fc.count_sends, '至少要送過一次 MISSION_COUNT'
        for c in fc.count_sends:
            assert c['target_system'] == 1, c
            assert c['target_component'] == 77, c

        items = fc.items_sent()
        assert len(items) == 3
        for w in items:
            assert w['target_system'] == 1, w
            assert w['target_component'] == 77, w

    def test_target_system_and_component_swap_is_caught(self, clock):
        """直接證明「調換了會被這裡的斷言抓到」，不是只在正常順序下打勾。"""
        fc = FakeFC(clock, target_system=1, target_component=77)
        # 模擬 _write_mission_items 把兩個參數位置調換送出去（不改動
        # sitl_link.py，只是在假 FC 這一層重現「調換後線路上長怎樣」）。
        fc.mav.mission_count_send(fc.target_component, fc.target_system, 2, 0)
        assert fc.count_sends[-1]['target_system'] == 77   # 被調換：跟真正的
        assert fc.count_sends[-1]['target_component'] == 1  # target_system/component 對不上
        with pytest.raises(AssertionError):
            assert fc.count_sends[-1]['target_system'] == 1


# ══════════════════════════════════════════════════════════════════
class TestPreflightNeverTouchesTheWire:
    def bad(self, clock, items, needle):
        r = mk(clock)
        res = r.upload(items)
        assert len(res) == 1, res
        assert res[0][1] is False
        assert needle in res[0][2], res[0][2]
        assert r.fc.sent == [], f'本機檢查失敗卻碰了線路：{r.fc.sent}'

    def test_empty(self, clock):
        self.bad(clock, [], '空')

    def test_none(self, clock):
        self.bad(clock, None, '空')

    def test_not_a_dict(self, clock):
        self.bad(clock, [mission3()[0], 'oops'], '不是 dict')

    def test_single_home_only(self, clock):
        self.bad(clock, mission3()[:1], 'Home')

    def test_missing_field(self, clock):
        items = mission3()
        del items[1]['alt']
        self.bad(clock, items, 'alt')

    @pytest.mark.parametrize('field', ['lat', 'lon', 'alt', 'param1', 'param2',
                                       'param3', 'param4'])
    @pytest.mark.parametrize('val', [float('nan'), float('inf'), -float('inf')])
    def test_non_finite_numbers(self, clock, field, val):
        items = mission3()
        items[1][field] = val
        self.bad(clock, items, field)

    def test_string_seq(self, clock):
        items = mission3()
        items[1]['seq'] = 'a'
        self.bad(clock, items, 'seq')

    def test_fractional_seq(self, clock):
        items = mission3()
        items[1]['seq'] = 1.5
        self.bad(clock, items, 'seq')

    def test_bool_is_not_an_int(self, clock):
        items = mission3()
        items[1]['command'] = True
        self.bad(clock, items, 'command')

    def test_gap_in_seq(self, clock):
        items = mission3()
        items[2]['seq'] = 3
        self.bad(clock, items, 'seq')

    def test_duplicate_seq(self, clock):
        items = mission3()
        items[2]['seq'] = 1
        self.bad(clock, items, '重複')

    def test_seq_must_start_at_zero(self, clock):
        items = mission3()
        for it in items:
            it['seq'] += 1
        self.bad(clock, items, 'seq')

    @pytest.mark.parametrize('cmd', [0, 1, -3, 65536])
    def test_command_range(self, clock, cmd):
        items = mission3()
        items[1]['command'] = cmd
        self.bad(clock, items, 'command')

    @pytest.mark.parametrize('frame', [-1, 256])
    def test_frame_range(self, clock, frame):
        items = mission3()
        items[1]['frame'] = frame
        self.bad(clock, items, 'frame')

    @pytest.mark.parametrize('field,val', [('lat', 90.5), ('lat', -91.0),
                                           ('lon', 180.5), ('lon', -200.0)])
    def test_coordinate_range(self, clock, field, val):
        items = mission3()
        items[1][field] = val
        self.bad(clock, items, field)

    def test_good_boundary_values_pass(self, clock):
        items = mission3()
        items[1].update(lat=90.0, lon=-180.0, command=65535, frame=255)
        r = mk(clock)
        r.upload(items, verify=False)
        assert r.results[0][1] is True, r.results


# ══════════════════════════════════════════════════════════════════
class TestFcMisbehaviourFailsLoudly:
    def only_failure(self, r, needle=None):
        assert len(r.results) == 1, r.results
        s, ok, msg = r.results[0]
        assert ok is False, msg
        if needle:
            assert needle in msg, msg
        assert r.status and r.status[-1][0] == 3, '失敗要走 severity 3'
        return msg

    @pytest.mark.parametrize('code,name', [
        (ACK_ERROR, 'MAV_MISSION_ERROR'),
        (ACK_UNSUPPORTED_FRAME, 'UNSUPPORTED_FRAME'),
        (ACK_NO_SPACE, 'NO_SPACE'),
        (5, 'MAV_MISSION_INVALID'),
        (6, 'INVALID_PARAM1'),
        (12, 'INVALID_PARAM7'),
        (ACK_INVALID_SEQ, 'INVALID_SEQUENCE'),
        (ACK_DENIED, 'DENIED'),
        (ACK_CANCELLED, 'OPERATION_CANCELLED'),
    ])
    @pytest.mark.parametrize('when', [0, 1, 2])
    def test_error_ack_mid_transfer(self, clock, code, name, when):
        r = mk(clock, ack_at={when: code})
        r.upload(mission3())
        self.only_failure(r, name)
        assert 'request_list' not in r.fc.kinds(), '失敗後不該還去讀回'

    def test_error_ack_right_after_count(self, clock):
        r = mk(clock, ack_at={0: ACK_NO_SPACE})
        r.upload(mission3())
        self.only_failure(r, 'NO_SPACE')
        assert r.fc.items_sent() == []

    def test_unknown_ack_code_is_still_failure(self, clock):
        r = mk(clock, ack_at={1: 99})
        r.upload(mission3())
        self.only_failure(r, 'UNKNOWN(99)')

    def test_invalid_sequence_without_our_duplicate_is_a_real_error(self, clock):
        """④＋見下方「未解釋的 INVALID_SEQUENCE」：已經送出至少一項之後收到的
        INVALID_SEQUENCE 不會再立刻判死，而是等飛控是否還會繼續問——但這裡飛控
        就此不再問任何東西（FakeFC 的 ack_at 模擬「真的拒收/當機」），所以最終
        還是要在 deadline 逾時失敗，訊息仍要帶著 INVALID_SEQUENCE 這個線索。"""
        r = mk(clock, ack_at={1: ACK_INVALID_SEQ})
        r.upload(mission3())
        msg = self.only_failure(r, 'INVALID_SEQUENCE')
        assert '1/3' in msg, msg  # 已回答到 seq0 才被打斷（已回答 1/3 項）

    def test_invalid_sequence_before_any_item_sent_is_still_an_immediate_real_error(self, clock):
        """未解釋的 INVALID_SEQUENCE 只有在「已經送出至少一項」之後才會被容忍
        （才可能是傳輸層把我方送出的東西複製了一份）；一項都還沒送出時收到，
        沒有「複製了哪一筆」可以解釋，必須維持原本立刻判定為真實錯誤，
        不能被新加的容忍邏輯波及。"""
        r = mk(clock, ack_at={0: ACK_INVALID_SEQ})
        r.upload(mission3())
        msg = self.only_failure(r, 'INVALID_SEQUENCE')
        assert '0/3' in msg, msg
        assert '逾時' not in msg, '一項都沒送出時應立即判定失敗，不該等到 deadline'

    def test_transport_layer_duplicate_delivery_without_client_resend_still_succeeds(self, clock):
        """finding（major）：傳輸層本身把 client 送出的一筆項目複製成兩份送進
        飛控（UDP／半雙工數傳鏈路常見），飛控因此回一次 INVALID_SEQUENCE——但
        client 自己只送了一次、沒有任何「我方重複回答」可歸因。修法之前這會被
        判成真實拒收；修法之後（未解釋的 INVALID_SEQUENCE 在已經送出至少一項後
        改為容忍等待，讓飛控自己用它的重試機制走完剩下的項目）應該正常成功、
        讀回也驗證相符 —— 不能讓一次良性的線路重複，把一筆完全被接受的寫入
        報成失敗。"""
        r = mk(clock, transport_dup_items={1})
        r.upload(mission3())
        assert r.results[0][1] is True, r.results
        assert '相符' in r.results[0][2], r.results[0][2]
        seqs = [w['seq'] for w in r.fc.items_sent()]
        assert seqs.count(1) == 1, seqs  # client 自己只送了一次，不是靠重送湊出來的

    def test_ack_too_early(self, clock):
        r = mk(clock, early_ack_at=2)
        r.upload(mission_n(4))
        msg = self.only_failure(r)
        assert '2/4' in msg
        assert 'request_list' not in r.fc.kinds()

    def test_out_of_range_request_fails_loudly_not_indexerror(self, clock):
        r = mk(clock, bogus_request={1: 99})
        r.upload(mission3())
        msg = self.only_failure(r, '99')
        assert 'IndexError' not in msg and '例外' not in msg

    def test_no_ack_by_deadline_is_failure(self, clock):
        r = mk(clock, no_final_ack=True)
        r.upload(mission3())
        msg = self.only_failure(r, '沒有收到 ACK')
        assert '未驗證' in msg
        assert 'request_list' not in r.fc.kinds()

    def test_silent_fc_resends_count_then_gives_up(self, clock):
        r = mk(clock, ignore_counts=99)
        r.upload(mission3())
        self.only_failure(r, '無回應')
        counts = [s for s in r.fc.sent if s[0] == 'count']
        assert len(counts) == 4, '1 次 + 3 次重送（有界）'
        assert r.fc.items_sent() == []

    def test_lost_count_is_recovered_by_resend(self, clock):
        r = mk(clock, ignore_counts=1)
        r.upload(mission3())
        assert r.results[0][1] is True, r.results
        assert len([s for s in r.fc.sent if s[0] == 'count']) == 2

    def test_lost_item_recovered_by_fc_rerequest(self, clock):
        r = mk(clock, drop_items={1})
        r.upload(mission3())
        assert r.results[0][1] is True, r.results

    def test_repeated_request_is_simply_answered_again(self, clock):
        r = mk(clock, dup_requests={1: 1})
        r.upload(mission3())
        assert r.results[0][1] is True, r.results
        seqs = [w['seq'] for w in r.fc.items_sent()]
        assert seqs.count(1) == 2, seqs
        assert set(seqs) == {0, 1, 2}

    def test_two_duplicates_are_tolerated_only_as_many_as_we_caused(self, clock):
        r = mk(clock, dup_requests={1: 2})
        r.upload(mission3())
        assert r.results[0][1] is True, r.results

    def test_stale_ack_in_buffer_is_not_taken_for_ours(self, clock):
        stale = [_Msg('MISSION_ACK', type=ACK_OK, mission_type=0)]
        r = mk(clock, stale_inbox=stale)
        r.upload(mission3())
        assert r.results[0][1] is True
        assert len(r.fc.items_sent()) == 3, '被舊 ACK 提早收尾 ⇒ 一個項目都沒送'

    def test_messages_from_other_systems_are_ignored(self, clock):
        r = mk(clock)
        # 開始前就在緩衝裡的訊息會被「清緩衝」吃掉；要測過濾，必須在交易**進行中**才到
        orig = r.fc.mav.mission_item_int_send

        def inject(*a, **kw):
            orig(*a, **kw)
            if a[2] == 0:
                r.fc.inbox.extend([
                    _Msg('MISSION_ACK', src=2, type=ACK_ERROR, mission_type=0),
                    _Msg('MISSION_REQUEST_INT', src=2, seq=99, mission_type=0)])
        r.fc.mav.mission_item_int_send = inject
        r.upload(mission3())
        assert r.results[0][1] is True, r.results

    def test_other_mission_type_acks_are_ignored(self, clock):
        r = mk(clock)
        orig = r.fc.mav.mission_item_int_send

        def inject(*a, **kw):
            orig(*a, **kw)
            if a[2] == 0:
                r.fc.inbox.append(_Msg('MISSION_ACK', type=ACK_NO_SPACE,
                                       mission_type=1))     # 圍籬的 ACK
        r.fc.mav.mission_item_int_send = inject
        r.upload(mission3())
        assert r.results[0][1] is True, r.results

    def test_deadline_scales_with_item_count(self, clock):
        """大任務不能被固定的短期限誤殺：慢速鏈路每項 0.5 s。"""
        r = mk(clock, latency=1.2)
        res = r.upload(mission_n(60))
        assert res[0][1] is True, res
        assert clock.t - 1000.0 > 60.0, '假時鐘至少要走過 60 s，否則沒測到期限縮放'


# ══════════════════════════════════════════════════════════════════
class TestVerifyReadback:
    def only_failure(self, r, needle):
        assert len(r.results) == 1
        s, ok, msg = r.results[0]
        assert ok is False, msg
        assert needle in msg, msg
        return msg

    def rb(self, idx, **chg):
        def mut(items):
            items = [dict(i) for i in items]
            items[idx].update(chg)
            return items
        return mut

    def test_command_differs(self, clock):
        r = mk(clock, readback_mutate=self.rb(1, command=22))
        r.upload(mission3())
        self.only_failure(r, 'seq 1 command')

    def test_lat_differs(self, clock):
        r = mk(clock, readback_mutate=self.rb(2, x=int(25.0901 * 1e7)))
        r.upload(mission3())
        self.only_failure(r, 'seq 2 lat')

    def test_lon_differs(self, clock):
        r = mk(clock, readback_mutate=self.rb(1, y=int(121.3 * 1e7)))
        r.upload(mission3())
        self.only_failure(r, 'seq 1 lon')

    def test_alt_differs(self, clock):
        r = mk(clock, readback_mutate=self.rb(1, z=105.0))
        r.upload(mission3())
        self.only_failure(r, 'seq 1 alt')

    @pytest.mark.parametrize('field', ['param1', 'param2', 'param3', 'param4'])
    def test_param_differs_on_strict_command(self, clock, field):
        items = mission3()
        items[1].update(command=DO_MOUNT_CONTROL, param1=1.0, param2=2.0,
                        param3=3.0, param4=4.0)
        r = mk(clock, readback_mutate=self.rb(1, **{field: 9.0}))
        r.upload(items)
        self.only_failure(r, f'seq 1 {field}')

    def test_frame_differs_for_location_command(self, clock):
        # 送 3（相對高度）讀回 0（AMSL）⇒ 高度差一個 home 高度
        r = mk(clock, readback_mutate=self.rb(1, frame=0))
        r.upload(mission3())
        self.only_failure(r, 'seq 1 frame')

    def test_count_differs_extra_item(self, clock):
        def extra(items):
            return items + [dict(items[-1], seq=len(items))]
        r = mk(clock, readback_mutate=extra)
        r.upload(mission3())
        self.only_failure(r, '項數：送出 3、讀回 4')

    def test_count_differs_fewer_items(self, clock):
        r = mk(clock, readback_mutate=lambda its: its[:2])
        r.upload(mission3())
        msg = self.only_failure(r, '項數：送出 3、讀回 2')
        assert 'seq 2 讀回缺漏' in msg

    def test_count_reply_lies(self, clock):
        r = mk(clock, readback_count=2)
        r.upload(mission3())
        self.only_failure(r, '讀回')

    def test_missing_item_at_readback(self, clock):
        r = mk(clock, readback_drop_seqs={2})
        r.upload(mission3())
        msg = self.only_failure(r, '讀回驗證失敗')
        assert 'seq 2' in msg and '已 ACK' in msg

    def test_home_slot_is_allowed_to_differ(self, clock):
        # 假飛控的 home 是 (25.0, 121.0, 10.0)、我們送的是 (25.0777, …)
        r = mk(clock)
        r.upload(mission3())
        assert r.results[0][1] is True, r.results

    def test_home_slot_frame_and_command_may_differ_too(self, clock):
        r = mk(clock, readback_mutate=self.rb(0, command=22, frame=3, x=5))
        r.upload(mission3())
        assert r.results[0][1] is True, r.results

    def test_only_first_five_diffs_are_named(self, clock):
        def wreck(items):
            return [items[0]] + [dict(i, command=22) for i in items[1:]]
        r = mk(clock, readback_mutate=wreck)
        r.upload(mission_n(9))
        msg = self.only_failure(r, '共 8 處')
        assert msg.count('command（') == 5 and 'seq 6 command' not in msg

    def test_readback_uses_the_shared_fetch_helper(self, clock):
        r = mk(clock)
        calls = []
        orig = r.link._fetch_mission_items
        r.link._fetch_mission_items = lambda: (calls.append(1), orig())[1]
        r.upload(mission3())
        assert calls == [1]

    def test_fc_lossy_storage_does_not_cause_false_alarm(self, clock):
        """真飛控的量化：param2 10.7→10、param1/4 不存、alt 截斷到 cm。"""
        items = mission3()
        items[1].update(param1=5.0, param2=10.7, param3=3.2, param4=90.0,
                        frame=FRAME_MISSION, alt=32.1)
        items.append(wp(3, 0.0, 0.0, 0.0, cmd=DO_CHANGE_SPEED, frame=FRAME_MISSION,
                        param2=25.0))
        items.append(wp(4, 25.1, 121.1, 50.0, cmd=NAV_LOITER_TIME, param1=30.0,
                        param3=-1.0))
        r = mk(clock)
        r.upload(items)
        assert r.results[0][1] is True, r.results

    def test_verify_readback_ignores_a_foreign_vehicles_mission_count(self, clock):
        """finding（blocker）：fan-in 拓樸下，同一條連線上另一架飛控（不同
        srcSystem）剛好也在傳 MISSION_COUNT，_fetch_mission_items 修法之前
        會照單全收——這裡故意在真正的讀回訊息之前，塞一筆 src=99（不是我們
        target_system=1 那架）的 MISSION_COUNT 進收件匣，證明讀回驗證認得
        出那不是我們要的那架，會繼續等到真正屬於我們的那筆，而不是把別架
        飛控的任務清單拿來比對。"""
        r = mk(clock)
        # 在 upload 觸發真正的 request_list 之前，先塞一筆別架飛控的
        # MISSION_COUNT——_fetch_mission_items 的讀回迴圈要能跳過它，
        # 繼續等 mission_request_list_send 之後才會出現的、真正屬於
        # target_system=1 的那一筆。
        r.fc.inbox.append(_Msg('MISSION_COUNT', src=99, count=5))
        r.upload(mission3())
        assert r.results[0][1] is True, r.results
        assert '相符' in r.results[0][2], r.results[0][2]

    def test_verify_readback_ignores_a_foreign_vehicles_mission_item(self, clock):
        """同一個 finding，但擋在逐項讀回那一關：別架飛控（src=99）剛好在
        我們自己也在問「seq=1 的內容是什麼」的同一時刻搶答，且它回的 seq
        跟我們問的一致（只有來源不同）——修法之前 `_fetch_mission_items`
        逐項讀回那段只檢查「訊息的 seq 是不是我現在要的那個」
        （`int(msg.seq) != seq`），完全不檢查來源，會把這筆別架的內容當成
        自己讀回的資料。

        ⚠️ 刻意讓假 FC 在請求 seq=1 的「當下」才吐出這筆搶答（`foreign_
        readback_at`），而不是在交易一開始就塞進收件匣：如果塞在最前面，
        會被既有的「seq 對不上」防線（跟這個 finding的修法完全無關、原本
        就有）先擋掉、白白耗掉一次重試，測不到真正的漏洞——這裡才是對這個
        finding 最不利、也最貼近『兩架飛控同時被讀』的真實排序。

        ⚠️ 刻意用 seq=1，不是 seq=0：`_mission_readback_diffs` 本來就跳過
        seq 0（飛控自己的 Home，內容本來就會不同，見那個函式的說明），如果
        拿 seq 0 來混，混進去的座標根本不會被拿去比對，測不出這個 finding；
        只有非 0 的 seq 才真的會被逐欄比對到，混進去的資料才有機會通過驗證、
        默默蓋掉真正要驗的內容。"""
        r = mk(clock, foreign_readback_at={
            1: dict(command=16, frame=3, x=int(9.9 * 1e7), y=int(9.9 * 1e7), z=999.0),
        })
        r.upload(mission3())
        assert r.results[0][1] is True, r.results
        assert '相符' in r.results[0][2], r.results[0][2]


# ══════════════════════════════════════════════════════════════════
class TestReadbackRules:
    """直接餵比對函式、**手寫數值**（來自 AP_Mission.cpp）—— 不經過假飛控。"""

    def d(self, sent, got):
        return SITLLink._mission_readback_diffs(sent, got)

    def base(self, **kw):
        s = dict(seq=1, command=16, frame=3, lat=25.0, lon=121.0, alt=100.0,
                 param1=0.0, param2=0.0, param3=0.0, param4=0.0)
        s.update(kw)
        return s

    HOME = dict(seq=0, command=16, frame=0, lat=1.0, lon=2.0, alt=3.0,
                param1=0.0, param2=0.0, param3=0.0, param4=0.0)

    def test_identical_is_clean(self):
        s = self.base()
        assert self.d([self.HOME, s], [self.HOME, dict(s)]) == []

    @pytest.mark.parametrize('sent_frame,back_frame,ok', [
        (2, 0, True), (0, 0, True), (5, 0, True),
        (3, 3, True), (6, 3, True), (10, 10, True), (11, 10, True),
        (3, 0, False), (2, 3, False), (10, 3, False), (0, 3, False),
    ])
    def test_location_frame_normalisation(self, sent_frame, back_frame, ok):
        s = self.base(frame=sent_frame)
        g = self.base(frame=back_frame)
        assert (self.d([self.HOME, s], [self.HOME, g]) == []) is ok

    def test_non_location_command_lat_lon_alt_frame_are_not_compared(self):
        s = self.base(command=DO_CHANGE_SPEED, frame=3, lat=25.0, lon=121.0,
                      alt=50.0, param2=25.0)
        g = self.base(command=DO_CHANGE_SPEED, frame=0, lat=0.0, lon=0.0,
                      alt=0.0, param2=25.0)
        assert self.d([self.HOME, s], [self.HOME, g]) == []

    def test_non_location_param_still_compared(self):
        s = self.base(command=DO_CHANGE_SPEED, param2=95.0)
        g = self.base(command=DO_CHANGE_SPEED, param2=30.0)
        assert 'param2' in self.d([self.HOME, s], [self.HOME, g])[0]

    def test_plane_nav_waypoint_quantisation_is_accepted(self):
        s = self.base(param1=5.0, param2=10.7, param3=3.2, param4=90.0)
        g = self.base(param1=0.0, param2=10.0, param3=3.0, param4=0.0)
        assert self.d([self.HOME, s], [self.HOME, g]) == []

    def test_nav_waypoint_real_param_mismatch_is_caught(self):
        s = self.base(param2=10.0)
        g = self.base(param2=40.0)
        assert 'param2' in self.d([self.HOME, s], [self.HOME, g])[0]

    def test_uint8_clamp_is_accepted(self):
        s = self.base(param2=300.0)
        g = self.base(param2=255.0)
        assert self.d([self.HOME, s], [self.HOME, g]) == []

    def test_loiter_time_param3_is_not_echoed(self):
        s = self.base(command=NAV_LOITER_TIME, param1=30.0, param3=5.0)
        g = self.base(command=NAV_LOITER_TIME, param1=30.0, param3=1.0)
        assert self.d([self.HOME, s], [self.HOME, g]) == []
        g2 = self.base(command=NAV_LOITER_TIME, param1=99.0, param3=1.0)
        assert 'param1' in self.d([self.HOME, s], [self.HOME, g2])[0]

    def test_strict_command_needs_float32_equality(self):
        s = self.base(command=DO_MOUNT_CONTROL, param1=0.1)
        g = self.base(command=DO_MOUNT_CONTROL, param1=f32(0.1))
        assert self.d([self.HOME, s], [self.HOME, g]) == []
        g2 = self.base(command=DO_MOUNT_CONTROL, param1=0.11)
        assert 'param1' in self.d([self.HOME, s], [self.HOME, g2])[0]
        g3 = self.base(command=DO_MOUNT_CONTROL, param1=0.0)     # 寬鬆規則不可外溢
        assert 'param1' in self.d([self.HOME, s], [self.HOME, g3])[0]

    def test_nan_readback_never_matches(self):
        s = self.base(command=DO_MOUNT_CONTROL, param1=1.0)
        g = self.base(command=DO_MOUNT_CONTROL, param1=float('nan'))
        assert self.d([self.HOME, s], [self.HOME, g])

    def test_lat_lon_one_lsb_ok_two_lsb_not(self):
        s = self.base(lat=25.0777)
        assert self.d([self.HOME, s], [self.HOME, self.base(lat=25.0777 + 1e-7)]) == []
        assert self.d([self.HOME, s], [self.HOME, self.base(lat=25.0777 - 1e-7)]) == []
        diffs = self.d([self.HOME, s], [self.HOME, self.base(lat=25.0777 + 2e-7)])
        assert diffs and 'lat' in diffs[0]
        diffs = self.d([self.HOME, s], [self.HOME, self.base(lat=25.0777, lon=121.0 + 2e-7)])
        assert diffs and 'lon' in diffs[0]

    def test_alt_tolerance_is_centimetre_truncation(self):
        s = self.base(alt=32.1)
        assert self.d([self.HOME, s], [self.HOME, self.base(alt=32.09)]) == []
        assert self.d([self.HOME, s], [self.HOME, self.base(alt=32.2)])
        big = self.base(alt=3100.0)
        assert self.d([self.HOME, big], [self.HOME, self.base(alt=3100.0)]) == []
        assert self.d([self.HOME, big], [self.HOME, self.base(alt=3100.05)])

    def test_seq_zero_is_skipped_entirely(self):
        g0 = dict(self.HOME, command=22, lat=50.0, alt=999.0, frame=3)
        s = self.base()
        assert self.d([self.HOME, s], [g0, dict(s)]) == []

    def test_count_and_missing(self):
        s1, s2 = self.base(), self.base(seq=2)
        out = self.d([self.HOME, s1, s2], [self.HOME, dict(s1)])
        assert any('項數' in x for x in out) and any('seq 2' in x for x in out)

    def test_command_mismatch_short_circuits_other_fields(self):
        s = self.base()
        g = self.base(command=22, lat=1.0, alt=1.0)
        out = self.d([self.HOME, s], [self.HOME, g])
        assert len(out) == 1 and 'command' in out[0]


class TestFactTablesMatchArduPilotNames:
    """數字表抄自 AP_Mission.cpp 的**名稱**，這裡對 pymavlink 列舉核對數字，
    防止手抄的 id 悄悄錯位。"""

    def enum_id(self, name):
        from pymavlink import mavutil
        return getattr(mavutil.mavlink, 'MAV_CMD_' + name, None)

    def test_stored_in_location_names(self):
        names = ['NAV_WAYPOINT', 'NAV_LOITER_UNLIM', 'NAV_LOITER_TURNS',
                 'NAV_LOITER_TIME', 'NAV_LAND', 'NAV_TAKEOFF',
                 'NAV_CONTINUE_AND_CHANGE_ALT', 'NAV_LOITER_TO_ALT',
                 'NAV_SPLINE_WAYPOINT', 'NAV_GUIDED_ENABLE', 'DO_SET_HOME',
                 'DO_RETURN_PATH_START', 'DO_LAND_START', 'DO_GO_AROUND',
                 'DO_SET_ROI_LOCATION', 'DO_SET_ROI', 'NAV_VTOL_TAKEOFF',
                 'NAV_VTOL_LAND', 'NAV_PAYLOAD_PLACE',
                 'NAV_FENCE_POLYGON_VERTEX_INCLUSION',
                 'NAV_FENCE_POLYGON_VERTEX_EXCLUSION',
                 'NAV_FENCE_CIRCLE_INCLUSION', 'NAV_FENCE_CIRCLE_EXCLUSION',
                 'NAV_FENCE_RETURN_POINT', 'NAV_RALLY_POINT']
        ids = {self.enum_id(n) for n in names}
        assert None not in ids
        table = set(sl._MISSION_CMDS_STORED_IN_LOCATION)
        assert ids <= table
        # 表裡多的只能是 pymavlink 這版還沒有名字的 NAV_ARC_WAYPOINT(36)
        assert table - ids <= {36}

    def test_non_location_commands_are_not_in_the_table(self):
        for n in ('NAV_DELAY', 'NAV_RETURN_TO_LAUNCH', 'DO_CHANGE_SPEED',
                  'DO_JUMP', 'DO_SET_SERVO', 'DO_VTOL_TRANSITION',
                  'CONDITION_DELAY'):
            assert self.enum_id(n) not in sl._MISSION_CMDS_STORED_IN_LOCATION

    def test_frame_table_keys_are_the_ardupilot_accepted_frames(self):
        from pymavlink import mavutil
        m = mavutil.mavlink
        assert set(sl._MISSION_LOCATION_FRAME_READBACK) == {
            m.MAV_FRAME_MISSION, m.MAV_FRAME_GLOBAL, m.MAV_FRAME_GLOBAL_INT,
            m.MAV_FRAME_GLOBAL_RELATIVE_ALT, m.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            m.MAV_FRAME_GLOBAL_TERRAIN_ALT, m.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT}
        assert sl._MISSION_LOCATION_FRAME_READBACK[m.MAV_FRAME_GLOBAL_RELATIVE_ALT] \
            == m.MAV_FRAME_GLOBAL_RELATIVE_ALT

    def test_lossy_set_contains_the_common_nav_commands(self):
        for n in ('NAV_WAYPOINT', 'NAV_LOITER_UNLIM', 'NAV_LAND', 'NAV_TAKEOFF',
                  'NAV_VTOL_TAKEOFF', 'NAV_VTOL_LAND', 'DO_CHANGE_SPEED'):
            assert self.enum_id(n) in sl._MISSION_CMDS_LOSSY_PARAMS


# ══════════════════════════════════════════════════════════════════
class TestExactlyOneEmission:
    """零次 = UI 永遠停在「寫入中…」；兩次 = 先成功又改失敗。每條路徑都要恰好一次。"""

    SCENARIOS = {
        'success': (dict(), True),
        'success_no_verify': (dict(), False),
        'preflight_fail': (dict(), None),
        'ack_error': (dict(ack_at={1: ACK_NO_SPACE}), True),
        'ack_early': (dict(early_ack_at=1), True),
        'out_of_range': (dict(bogus_request={1: 99}), True),
        'silent': (dict(ignore_counts=99), True),
        'no_ack': (dict(no_final_ack=True), True),
        'readback_diff': (dict(readback_mutate=lambda i: i[:-1]), True),
        'readback_missing': (dict(readback_drop_seqs={1}), True),
        'exception_on_count': (dict(raise_on='count'), True),
        'exception_on_readback': (dict(raise_on='readback'), True),
    }

    @pytest.mark.parametrize('name', sorted(SCENARIOS))
    def test_once(self, clock, name):
        kw, verify = self.SCENARIOS[name]
        r = mk(clock, **kw)
        items = mission3()
        if name == 'preflight_fail':
            items = items[:1]
        r.link._dispatch_command('upload_mission_items',
                                 (items, True if verify is None else verify))
        assert len(r.results) == 1, (name, r.results)

    def test_exception_becomes_a_failure_not_a_crash(self, clock):
        r = mk(clock, raise_on='count')
        r.link._dispatch_command('upload_mission_items', (mission3(), True))
        s, ok, msg = r.results[0]
        assert ok is False and 'RuntimeError' in msg and '假的 socket 錯誤' in msg

    def test_exception_in_write_helper_emits_once(self, clock):
        r = mk(clock)

        def boom(*a, **k):
            raise ValueError('內部炸了')
        r.link._write_mission_items = boom
        r.link._send_mission_items(mission3(), True)
        assert len(r.results) == 1 and r.results[0][1] is False
        assert '內部炸了' in r.results[0][2]

    def test_no_connection_emits_failure_not_silence(self, clock):
        r = mk(clock)
        r.link._mav = None
        r.link._dispatch_command('upload_mission_items', (mission3(), True))
        assert len(r.results) == 1 and r.results[0][1] is False
        assert '連線' in r.results[0][2]

    def test_unusable_items_at_the_public_method_emit_once(self, clock):
        r = mk(clock)
        r.link.upload_mission_items(None)
        assert len(r.results) == 1 and r.results[0][1] is False
        assert r.link._cmd_queue.empty()

    def test_only_one_emit_site_in_the_source(self):
        tree = ast.parse(SRC.read_text(encoding='utf-8'))
        sites = []
        for node in ast.walk(tree):
            if isinstance(node, ast.FunctionDef):
                for sub in ast.walk(node):
                    if (isinstance(sub, ast.Call)
                            and isinstance(sub.func, ast.Attribute)
                            and sub.func.attr == 'emit'
                            and isinstance(sub.func.value, ast.Attribute)
                            and sub.func.value.attr == 'mission_uploaded'):
                        sites.append(node.name)
        assert sites == ['_report_mission_upload'], sites

    def test_lock_is_never_left_held(self, clock):
        for kw in ({}, dict(raise_on='count'), dict(ack_at={1: ACK_ERROR}),
                   dict(ignore_counts=99)):
            r = mk(clock, **kw)
            r.link._dispatch_command('upload_mission_items', (mission3(), True))
            assert r.link._mav_lock.acquire(blocking=False), kw
            r.link._mav_lock.release()


# ══════════════════════════════════════════════════════════════════
class TestPublicApiOnlyEnqueues:
    def test_only_enqueues_a_deep_copy(self, clock):
        r = mk(clock)
        items = mission3()
        r.link.upload_mission_items(items)
        assert r.fc.sent == [], '公開方法直接送了 MAVLink ⇒ 跨執行緒競爭'
        cmd, (snap, verify) = r.link._cmd_queue.get_nowait()
        assert cmd == 'upload_mission_items' and verify is True
        assert snap == items and snap is not items
        assert snap[1] is not items[1]
        items[1]['lat'] = 0.0           # 事後改動不可影響已排隊的快照
        assert snap[1]['lat'] == 25.0800
        assert r.results == [], '公開方法本身不該 emit（成功結果要等接收執行緒）'

    def test_verify_flag_is_carried(self, clock):
        r = mk(clock)
        r.link.upload_mission_items(mission3(), verify=False)
        assert r.link._cmd_queue.get_nowait()[1][1] is False

    def test_accepts_any_iterable_of_dicts(self, clock):
        r = mk(clock)
        r.link.upload_mission_items(iter(mission3()))
        assert len(r.link._cmd_queue.get_nowait()[1][0]) == 3

    def test_dispatch_routes_to_the_write_protocol(self, clock):
        r = mk(clock)
        r.link._dispatch_command('upload_mission_items', (mission3(), True))
        assert r.results and r.results[0][1] is True
        assert r.fc.kinds().count('item') == 3

    def test_dispatch_branch_exists_and_has_no_bare_else(self):
        tree = ast.parse(SRC.read_text(encoding='utf-8'))
        fn = next(n for n in ast.walk(tree)
                  if isinstance(n, ast.FunctionDef) and n.name == '_dispatch_command')
        consts = {c.value for c in ast.walk(fn) if isinstance(c, ast.Constant)}
        assert 'upload_mission_items' in consts
        calls = {c.func.attr for c in ast.walk(fn)
                 if isinstance(c, ast.Call) and isinstance(c.func, ast.Attribute)}
        assert '_send_mission_items' in calls
        # 沒有「else: 吞掉未知命令」：if/elif 鏈的最後一個 orelse 必須為空
        chain = next(n for n in ast.walk(fn) if isinstance(n, ast.If)
                     and isinstance(n.test, ast.Compare)
                     and isinstance(n.test.comparators[0], ast.Constant)
                     and n.test.comparators[0].value == 'arm')
        node = chain
        while node.orelse and isinstance(node.orelse[0], ast.If):
            node = node.orelse[0]
        assert node.orelse == [], '命令分派鏈尾端有 bare else ⇒ 未知命令被吞掉'

    def test_legacy_upload_is_untouched(self, clock):
        r = mk(clock)
        r.link.upload_mission([(25.0, 121.0, 100.0)])
        assert r.link._cmd_queue.get_nowait() == ('upload_mission', [(25.0, 121.0, 100.0)])

    def test_legacy_send_mission_still_clears_first(self, clock):
        """舊路徑行為不變（含 clear_all）；新路徑刻意不同。"""
        r = mk(clock)
        r.link._send_mission([(25.0, 121.0, 100.0)])
        assert r.fc.kinds()[0] == 'clear_all'

    def test_signal_exists_with_the_contract_types(self, clock):
        assert hasattr(SITLLink, 'mission_uploaded')
        link = SITLLink('udpin:0.0.0.0:1', sysid_label=3)
        got = []
        link.mission_uploaded.connect(lambda *a: got.append(a))
        link.mission_uploaded.emit(3, True, 'x')
        assert got == [(3, True, 'x')]

    def test_recv_mission_and_fetch_helper_split(self):
        tree = ast.parse(SRC.read_text(encoding='utf-8'))
        fns = {n.name: n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)}

        def calls(fn):
            return {c.func.attr for c in ast.walk(fn)
                    if isinstance(c, ast.Call) and isinstance(c.func, ast.Attribute)}
        assert '_fetch_mission_items' in calls(fns['_recv_mission'])
        assert '_fetch_mission_items' in calls(fns['_write_mission_items'])
        assert 'emit' not in calls(fns['_fetch_mission_items']), \
            '共用讀取 helper 不可 emit（否則驗證讀回會冒出「下載完成」）'
        assert 'emit' not in calls(fns['_write_mission_items'])

    def test_verify_readback_does_not_emit_download_signals(self, clock):
        r = mk(clock)
        got, failed = [], []
        r.link.mission_downloaded.connect(lambda s, i: got.append(i))
        r.link.mission_download_failed.connect(lambda s, m: failed.append(m))
        r.upload(mission3())
        assert got == [] and failed == []


# ══════════════════════════════════════════════════════════════════
class TestHeartbeatPump:
    """接收迴圈在交易期間是停住的，沒有 GCS 心跳 ⇒ FS_GCS ⇒ 自動 RTL。"""

    def gaps(self, hb):
        return [b - a for a, b in zip(hb, hb[1:])]

    def test_long_upload_keeps_heart_beating(self, clock):
        r = mk(clock, latency=0.4)
        res = r.upload(mission_n(40))
        assert res[0][1] is True, res
        hb = r.fc.heartbeats
        span = clock.t - 1000.0
        assert span > 15.0
        assert len(hb) >= int(span) - 2, (len(hb), span)
        assert max(self.gaps(hb)) <= 1.6, self.gaps(hb)

    def test_readback_phase_keeps_heart_beating_too(self, clock):
        """讀回驗證是第二段長阻塞（每項一來一回）；它也不能讓 FS_GCS 餓死。"""
        r = mk(clock, readback_latency=0.6)
        res = r.upload(mission_n(30))
        assert res[0][1] is True, res
        during = [h for h in r.fc.heartbeats if h >= r.fc.t_readback]
        assert clock.t - r.fc.t_readback > 15.0
        assert len(during) >= 12, (len(during), clock.t - r.fc.t_readback)
        assert max(self.gaps(during)) <= 1.6

    def test_waiting_for_a_silent_fc_also_pumps(self, clock):
        r = mk(clock, ignore_counts=99)
        r.upload(mission3())
        assert len(r.fc.heartbeats) >= 5
        assert max(self.gaps(r.fc.heartbeats)) <= 1.6


# ══════════════════════════════════════════════════════════════════
class TestSourceIsInvestigatedAndDocumented:
    """ArduPilot 的事實要寫在程式碼旁邊（出處），不然下一個人會把它「簡化」掉。"""

    TEXT = SRC.read_text(encoding='utf-8')

    @pytest.mark.parametrize('needle', [
        'MissionItemProtocol', 'AP_Mission', 'truncate', 'replace_cmd',
        'current == 2', 'INVALID_SEQUENCE', 'upload_timeout_ms',
        'stored_in_location', 'MISSION_REQUEST_INT',
    ])
    def test_fact_is_cited(self, needle):
        assert needle in self.TEXT


# ══════════════════════════════════════════════════════════════════
# 端到端：對 tools/mock_autopilot.py（獨立的協定伺服端）走 127.0.0.1 UDP
# ══════════════════════════════════════════════════════════════════
_LOOPBACK_CHILD = r'''
import os, sys, json, threading, time, importlib.util
os.environ["MAVLINK20"] = "1"
ROOT = sys.argv[1]
sys.path.insert(0, ROOT)
from PyQt6.QtCore import QCoreApplication
_app = QCoreApplication([])
from pymavlink import mavutil
from mission.sitl_link import SITLLink

spec = importlib.util.spec_from_file_location("mock_autopilot", os.path.join(ROOT, "tools", "mock_autopilot.py"))
ma = importlib.util.module_from_spec(spec); spec.loader.exec_module(ma)

out = {"conn": None}
stop = threading.Event()
mock = ma.MockAutopilot("udpin:127.0.0.1:0", [23.9, 120.7, 100.0, 0.0], 20.0, 1.0, None, verbose=False)
port = mock.conn.port.getsockname()[1]
conn_str = f"udpout:127.0.0.1:{port}"
# 硬性防線：這個子行程只允許連 127.0.0.1 的臨時埠，絕不連任何裝置。
assert conn_str.startswith("udpout:127.0.0.1:") and port > 1023, conn_str
out["conn"] = conn_str

def serve():
    while not stop.is_set():
        try:
            m = mock.conn.recv_match(blocking=False)
        except ConnectionResetError:
            m = None
        if m is None:
            time.sleep(0.003)
            continue
        mock.handle(m)

th = threading.Thread(target=serve, daemon=True)
client = None
try:
    th.start()
    client = mavutil.mavlink_connection(conn_str, source_system=255, dialect="ardupilotmega")
    client.target_system = 1
    client.target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
    link = SITLLink(conn_str, sysid_label=1)
    link._mav = client
    up, down, dfail = [], [], []
    link.mission_uploaded.connect(lambda s, ok, m: up.append([s, ok, m]))
    link.mission_downloaded.connect(lambda s, it: down.append(it))
    link.mission_download_failed.connect(lambda s, m: dfail.append(m))
    link._send_gcs_heartbeat()          # 讓 udpin 端學到對端位址
    time.sleep(0.3)

    items = json.loads(sys.argv[2])
    link.upload_mission_items(items, verify=True)
    cmd, arg = link._cmd_queue.get_nowait()
    link._dispatch_command(cmd, arg)
    link.download_mission()
    cmd, arg = link._cmd_queue.get_nowait()
    link._dispatch_command(cmd, arg)
    out.update(up=up, down=down, dfail=dfail,
               mock_items=[{"seq": i["seq"], "command": i["command"], "frame": i["frame"],
                            "current": i["current"]} for i in mock.missions[0]])
finally:
    stop.set()
    th.join(5)
    for c in (client, mock.conn):
        try:
            c.close()
        except Exception:
            pass
print("RESULT:" + json.dumps(out))
'''


class TestLoopbackAgainstMockAutopilot:
    def run_child(self, tmp_path, items):
        script = tmp_path / 'child_loopback.py'
        script.write_text(_LOOPBACK_CHILD, encoding='utf-8')
        env = dict(os.environ, MAVLINK20='1', PYTHONIOENCODING='utf-8')
        p = subprocess.run([sys.executable, str(script), str(ROOT), json.dumps(items)],
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', timeout=120, env=env)
        line = [ln for ln in p.stdout.splitlines() if ln.startswith('RESULT:')]
        assert line, f'子行程沒有結果\nSTDOUT:{p.stdout}\nSTDERR:{p.stderr[-2000:]}'
        return json.loads(line[0][len('RESULT:'):])

    def test_upload_verify_then_download_returns_the_same_items(self, tmp_path):
        items = [
            wp(0, 25.5, 121.25, 10.0, frame=FRAME_GLOBAL),
            wp(1, 25.75, 121.5, 100.0, param2=10.0),
            wp(2, 25.5, 121.75, 150.0, frame=FRAME_GLOBAL),
            wp(3, 0.0, 0.0, 0.0, cmd=DO_CHANGE_SPEED, frame=FRAME_MISSION, param2=22.0),
        ]
        out = self.run_child(tmp_path, items)
        assert out['conn'].startswith('udpout:127.0.0.1:')
        assert len(out['up']) == 1, out
        sysid, ok, msg = out['up'][0]
        assert ok is True, msg
        assert '已寫入並讀回驗證 4 項相符' in msg
        assert out['dfail'] == [] and len(out['down']) == 1
        back = out['down'][0]
        assert [b['seq'] for b in back] == [0, 1, 2, 3]
        assert [b['command'] for b in back] == [16, 16, 16, 178]
        assert back[1]['lat'] == pytest.approx(25.75, abs=1e-7)
        assert back[1]['alt'] == pytest.approx(100.0)
        # frame 照原樣到達 mock（沒有被舊路徑的猜測改寫）。
        # ⚠️ mock 不是 ArduPilot：它原樣回傳 frame、不做 2→0 / 6→3 的正規化，
        #    所以這裡只用 0 與 3（真飛控與 mock 行為一致的那兩個）。
        assert [m['frame'] for m in out['mock_items']] == [0, 3, 0, 2]
        assert all(m['current'] == 0 for m in out['mock_items'])
