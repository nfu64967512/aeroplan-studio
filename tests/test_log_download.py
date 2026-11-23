# -*- coding: utf-8 -*-
"""mission.sitl_link —— 經 MAVLink 下載飛控 DataFlash 日誌（.bin）的鏈路層：值層級鎖。

不開真連線、不開 SITL／COM 埠、不開 MainWindow：
  * 訊息用 pymavlink 的 MAVLink2 編碼器打包再解碼（帶真的 sysid/compid），直接餵 _handle_message
    （同 tests/test_sitl_link_camera.py 的 mk/hb/make_link）；
  * 連線物件換成 FakeMav（同 tests/test_mission_download.py 的作法）：記下每一則
    log_request_list / log_request_data / log_request_end；recv_match 永遠 None；
  * 時鐘：monkeypatch time.monotonic（列表 8 s、閒置 1 s、無進展 60 s、進度節流 0.5 s 都靠它）；
  * run() 結束的路徑用假的 mavlink_connection（同 camera 測試的 _FakeConn）。

鎖的東西（依交接單）：
  ① 列表：送 (sysid, 自駕儀 compid, 0, 0xFFFF)；三則收齊只 emit 一次且依 id 排序；num_logs=0 ⇒ 空清單；
     8 s 沒收齊 ⇒ 先對缺的 id 區間再要一次、再 8 s ⇒ log_list_failed；別架／相機的 LOG_ENTRY 不算。
  ② 下載：armed ⇒ 立即 failed 且不送請求；正常送 (id, 0, 0xFFFFFFFF)；亂序／重複／count<90 尾塊／
     ofs≥size 多餘塊／別的 id／別架 sysid（未確認與已確認兩種）／同 sysid 相機 compid 偽造 ⇒ 只有正確的塊
     被寫；缺塊 ⇒ 閒置 1 s 後補抓 (ofs, count) 正確、一段收齊立刻要下一段、每輪最多 20 段；
     收齊 ⇒ done：path 存在、大小 == size、sha256 == 內容、.part 不在、LOG_REQUEST_END 恰好一次；
     進度節流（100 塊只 emit 2 次：開始＋最後，最後一則 received == size；跨 1 MB 多一次）。
  ③ 取消 ⇒ failed('取消…')、.part 刪、END 送出；進行中再 download_log ⇒ 立即 failed；
     目的檔存在 ⇒ 寫到 _2（原檔不動）；父目錄是檔案／open 丟 OSError／寫入中 write 丟 OSError ⇒ failed 含原因；
     60 s 無進展 ⇒ failed；400 輪補抓 ⇒ failed；stop()／run() 結束 ⇒ failed('連線結束') 恰好一次；
     飛控回 STATUSTEXT "Disarm for log download" ⇒ 當下 failed（只在還沒收到任何資料時，見 ⑤）。
  ⑤ 2026-10-17 審查四項（每一項都先紅後綠）：
     鏈路已結束（stop() 關了 _mav 但不會設成 None；run() 退出亦同）⇒ download_log / request_log_list
       **立即** failed('連線已結束')、不登記、不排佇列；排在佇列裡沒派送的 'log_list'/'log_download'
       在 stop()／run() finally 各失敗一次（'連線結束'），其他指令原樣留著。
     fan-in（target_system 鎖在別架）⇒ 日誌交易一律綁 sysid_label：請求送給它、只收它的
       LOG_ENTRY/LOG_DATA、armed 預檢看它的心跳、每個信號第一個參數都是它。
     "Log download in progress"／"Disarm for log download" 只在**什麼都還沒收到**時才算拒絕我們；資料在流就忽略；
       沒收過資料就被拒 ⇒ 不送 LOG_REQUEST_END（會砍掉別條鏈路的傳輸）；已收過資料而放棄（逾時／400 輪／例外）⇒ END。
     每塊都到（phase='finishing'）之後改名／sha256／關檔失敗 ⇒ **不刪 .part**、原因寫出路徑；
       os.rename 失敗隔 0.2 s 再試、最多 5 次（4 次等待 ≤ 1 s），暫時性失敗照樣 done。
     突變自驗（scratchpad/mutate_fixes.py；基線 68 綠，每個突變後用原始位元組還原、sha256 2f5ffd1728c5aaaa 相同、CRLF 3795 不變）：
       關閉旗標忽略＋不排空佇列  63 綠 / 5 紅  TestDeadLink 全部
       _log_sysid 改回 target_system  64 綠 / 4 紅  TestFanInBinding 全部
       拒絕句一律收掉（only_before_data=False）  66 綠 / 2 紅  …ignored_once_blocks_are_flowing、…ignored_once_entries_are_flowing
       finishing 也刪 .part  66 綠 / 2 紅  …rename_failure_keeps…、…sha256_failure_keeps…
       改名只試 1 次  66 綠 / 2 紅  …rename_failure_keeps…、…rename_transient_failure…

═══ ④ 突變自驗（2026-10-17，scratchpad/mutate_log_download.py；每個突變改完跑本檔、再用原始位元組還原、
    還原後 sha256 與改前相同）═══
  size 上限    `if ofs >= size:` → `if ofs >= size + 90:`（_on_log_data）
               ⇒ 紅：test_only_correct_blocks_are_written、test_happy_path_in_order 等（IndexError／大小不符）
  只收自駕儀   `elif not self._is_autopilot_component(src, self._src_component(msg)):` → `elif False and not …`
               ⇒ 紅：test_only_correct_blocks_are_written（相機偽造的垃圾塊被寫進去、sha256 不符）
  節流         `if now - st['last_progress_emit'] >= _LOG_PROGRESS_INTERVAL_S \\` → `if True \\`
               ⇒ 紅：test_progress_is_throttled（100 塊 emit 100 次）
  不覆蓋       `while os.path.exists(cand) or os.path.exists(cand + '.part'):` → `while False and …`
               ⇒ 紅：test_existing_destination_gets_suffix（Windows：os.rename 撞到既有檔 ⇒ failed；
               POSIX：路徑沒有 _2 且原檔被蓋）
  實跑結果（基線 51 passed / 0 failed；每個突變後還原、sha256 94369abd477aeea0 與改前相同、CRLF 3683 不變）：
    size 上限      50 綠 /  1 紅  test_only_correct_blocks_are_written
    只收自駕儀元件  49 綠 /  2 紅  test_only_correct_blocks_are_written、test_log_branches_follow_the_autopilot_guard
    節流           49 綠 /  2 紅  test_progress_is_throttled、test_progress_emits_on_time_and_on_megabyte
    不覆蓋         49 綠 /  2 紅  test_existing_destination_gets_suffix、test_destination_created_during_download_is_not_overwritten
    還原後         51 綠 /  0 紅
"""
from __future__ import annotations

import hashlib
import os
import queue
import threading
import time
from types import SimpleNamespace

import pytest
from PyQt6.QtCore import QCoreApplication
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mav2

from mission import sitl_link as sl
from mission.sitl_link import SITLLink

_app = QCoreApplication.instance() or QCoreApplication([])

CHUNK = 90
FC_TYPE, FC_AP = 1, 3            # MAV_TYPE_FIXED_WING / MAV_AUTOPILOT_ARDUPILOTMEGA
CAM_TYPE, CAM_AP = 30, 8         # MAV_TYPE_CAMERA / MAV_AUTOPILOT_INVALID
ARMED = 128 | 1
DISARMED = 1
PLANE_AUTO = 10


# ═════════════════════════════════════════════════════════════════════
# 訊息工廠：真的 MAVLink2 編碼 → 解碼（帶 sysid / compid）
# ═════════════════════════════════════════════════════════════════════
def mk(src: int, comp: int, name: str, *args):
    tx = mav2.MAVLink(None, srcSystem=src, srcComponent=comp)
    buf = getattr(tx, f'{name}_encode')(*args).pack(tx)
    msg = mav2.MAVLink(None).decode(bytearray(buf))
    assert bytes(msg.get_msgbuf()) == bytes(buf)
    return msg


def hb(src=1, comp=1, base_mode=DISARMED, custom_mode=PLANE_AUTO):
    return mk(src, comp, 'heartbeat', FC_TYPE, FC_AP, base_mode, custom_mode, 4, 3)


def cam_hb(src=1, comp=100):
    return mk(src, comp, 'heartbeat', CAM_TYPE, CAM_AP, 0, 0, 4, 3)


def log_entry(src, comp, log_id, num_logs, size, time_utc, last_log_num=None):
    return mk(src, comp, 'log_entry', log_id, num_logs,
              num_logs if last_log_num is None else last_log_num, time_utc, size)


def log_data(src, comp, log_id, ofs, data: bytes):
    """LOG_DATA：data 不足 90 B 用 0 補滿（飛控也是這樣），count = 真實長度。"""
    assert len(data) <= CHUNK
    padded = list(data) + [0] * (CHUNK - len(data))
    return mk(src, comp, 'log_data', log_id, ofs, len(data), padded)


def statustext(src, comp, text: str, severity=3):
    return mk(src, comp, 'statustext', severity, text.encode('ascii'), 0, 0)


def content(size: int, seed: int = 7) -> bytes:
    """決定性的、每個位元組都不一樣的內容（補 0 的尾塊與真資料分得出來）。"""
    out = bytearray(size)
    x = seed
    for i in range(size):
        x = (x * 1103515245 + 12345) & 0x7FFFFFFF
        out[i] = (x >> 16) & 0xFF
    return bytes(out)


def blocks_of(data: bytes):
    return [(ofs, data[ofs:ofs + CHUNK]) for ofs in range(0, len(data), CHUNK)]


# ═════════════════════════════════════════════════════════════════════
# 假連線：記下送出的每一則 LOG_REQUEST_*
# ═════════════════════════════════════════════════════════════════════
class _FakeMavApi:
    def __init__(self, owner):
        self._o = owner

    def log_request_list_send(self, ts, tc, start, end):
        self._o.sent.append(('list', ts, tc, start, end))

    def log_request_data_send(self, ts, tc, log_id, ofs, count):
        self._o.sent.append(('data', ts, tc, log_id, ofs, count))

    def log_request_end_send(self, ts, tc):
        self._o.sent.append(('end', ts, tc))

    def heartbeat_send(self, *a, **k):
        self._o.sent.append(('heartbeat',))

    def request_data_stream_send(self, *a, **k):
        pass


class FakeMav:
    def __init__(self, target_system=1):
        self.target_system = target_system
        self.target_component = 0        # pymavlink 預設廣播 —— 我們不可以用它
        self.sent = []
        self.closed = 0
        self.mav = _FakeMavApi(self)

    def recv_match(self, *a, **k):
        return None

    def close(self):
        self.closed += 1

    def requests(self, kind):
        return [s for s in self.sent if s[0] == kind]


class Capture:
    def __init__(self, link):
        self.lists, self.list_failed = [], []
        self.progress, self.done, self.failed, self.status = [], [], [], []
        link.log_list.connect(lambda s, e: self.lists.append((s, e)))
        link.log_list_failed.connect(lambda s, r: self.list_failed.append((s, r)))
        link.log_download_progress.connect(lambda s, d: self.progress.append((s, dict(d))))
        link.log_download_done.connect(lambda s, d: self.done.append((s, dict(d))))
        link.log_download_failed.connect(lambda s, r: self.failed.append((s, r)))
        link.status_text.connect(lambda sev, t: self.status.append((sev, t)))


def make_link(fake=None, sysid=1):
    link = SITLLink(conn_str='tcp:127.0.0.1:5760', sysid_label=sysid)
    link._mav = fake
    cap = Capture(link)
    return link, cap


def drain(link):
    """把 _cmd_queue 裡的指令全部交給 _dispatch_command（模擬主迴圈那一段）。"""
    n = 0
    while True:
        try:
            cmd, arg = link._cmd_queue.get_nowait()
        except queue.Empty:
            return n
        link._dispatch_command(cmd, arg)
        n += 1


class Clock:
    """可控的 time.monotonic。"""
    def __init__(self, t0=1000.0):
        self.t = t0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


@pytest.fixture
def clock(monkeypatch):
    c = Clock()
    monkeypatch.setattr(sl.time, 'monotonic', c)
    return c


def start_download(tmp_path, size, log_id=3, name='log003.bin', fake=None, sysid=1):
    fake = fake or FakeMav()
    link, cap = make_link(fake, sysid=sysid)
    link._handle_message(hb(sysid, 1))
    dest = str(tmp_path / name)
    link.download_log(log_id, dest, size)
    assert drain(link) == 1
    return link, cap, fake, dest


def feed(link, data, log_id=3, src=1, comp=1, order=None):
    bl = blocks_of(data)
    idx = list(range(len(bl))) if order is None else order
    for i in idx:
        ofs, chunk = bl[i]
        link._handle_message(log_data(src, comp, log_id, ofs, chunk))


# ═════════════════════════════════════════════════════════════════════
# ① 列表
# ═════════════════════════════════════════════════════════════════════
class TestLogList:
    def test_request_goes_to_autopilot_component_not_broadcast(self):
        fake = FakeMav()
        link, cap = make_link(fake)
        link.request_log_list()
        assert drain(link) == 1
        assert fake.sent == [('list', 1, 1, 0, 0xFFFF)]
        assert cap.lists == [] and cap.list_failed == []

    def test_three_entries_emit_once_sorted_by_id(self):
        fake = FakeMav()
        link, cap = make_link(fake)
        link.request_log_list()
        drain(link)
        link._handle_message(log_entry(1, 1, 3, 3, 300, 1_700_000_030))
        link._handle_message(log_entry(1, 1, 1, 3, 100, 1_700_000_010))
        assert cap.lists == []
        link._handle_message(log_entry(1, 1, 2, 3, 200, 1_700_000_020))
        assert len(cap.lists) == 1
        sysid, entries = cap.lists[0]
        assert sysid == 1
        assert entries == [{'id': 1, 'size': 100, 'time_utc': 1_700_000_010},
                           {'id': 2, 'size': 200, 'time_utc': 1_700_000_020},
                           {'id': 3, 'size': 300, 'time_utc': 1_700_000_030}]
        assert link._log_list_state is None
        # 收齊之後再來的 LOG_ENTRY 不會再 emit
        link._handle_message(log_entry(1, 1, 2, 3, 200, 1_700_000_020))
        assert len(cap.lists) == 1 and cap.list_failed == []

    def test_zero_logs_is_an_empty_list_not_a_failure(self):
        link, cap = make_link(FakeMav())
        link.request_log_list()
        drain(link)
        link._handle_message(log_entry(1, 1, 0, 0, 0, 0))
        assert cap.lists == [(1, [])]
        assert cap.list_failed == []

    def test_timeout_retries_missing_range_then_fails(self, clock):
        fake = FakeMav()
        link, cap = make_link(fake)
        link.request_log_list()
        drain(link)
        link._handle_message(log_entry(1, 1, 1, 3, 100, 10))
        clock.advance(7.9)
        link._log_transfer_tick()
        assert cap.list_failed == [] and len(fake.requests('list')) == 1
        clock.advance(0.2)                                  # 8.1 s 沒新 LOG_ENTRY
        link._log_transfer_tick()
        assert cap.list_failed == []
        assert fake.requests('list') == [('list', 1, 1, 0, 0xFFFF), ('list', 1, 1, 2, 3)]
        clock.advance(8.1)
        link._log_transfer_tick()
        assert len(cap.list_failed) == 1
        sysid, reason = cap.list_failed[0]
        assert sysid == 1 and '8' in reason and '1/3' in reason
        assert cap.lists == []
        assert link._log_list_state is None

    def test_timeout_with_nothing_received_retries_whole_range(self, clock):
        fake = FakeMav()
        link, cap = make_link(fake)
        link.request_log_list()
        drain(link)
        clock.advance(8.5)
        link._log_transfer_tick()
        assert fake.requests('list')[-1] == ('list', 1, 1, 0, 0xFFFF)
        clock.advance(8.5)
        link._log_transfer_tick()
        assert len(cap.list_failed) == 1 and '已收 0' in cap.list_failed[0][1]

    def test_entries_from_camera_or_other_sysid_do_not_count(self):
        link, cap = make_link(FakeMav())
        link._handle_message(hb(2, 1))                      # sysid 2 是另一架已確認的飛機
        link.request_log_list()
        drain(link)
        link._handle_message(log_entry(1, 100, 1, 2, 100, 10))   # 同 sysid 的相機
        link._handle_message(log_entry(2, 1, 1, 2, 100, 10))     # 別架
        link._handle_message(log_entry(2, 1, 2, 2, 100, 10))
        link._handle_message(log_entry(5, 1, 1, 1, 100, 10))     # 沒確認過的來源
        assert cap.lists == []
        link._handle_message(log_entry(1, 1, 1, 2, 100, 10))
        link._handle_message(log_entry(1, 1, 2, 2, 200, 20))
        assert len(cap.lists) == 1 and [e['id'] for e in cap.lists[0][1]] == [1, 2]

    def test_not_connected_fails_immediately(self):
        link, cap = make_link(None)
        link.request_log_list()
        assert cap.list_failed == [(1, '尚未連線到飛控（沒有可用的 MAVLink 連線）')]
        assert link._cmd_queue.empty()

    def test_armed_fails_without_sending(self):
        fake = FakeMav()
        link, cap = make_link(fake)
        link._handle_message(hb(1, 1, base_mode=ARMED))
        link.request_log_list()
        drain(link)
        assert fake.sent == []
        assert len(cap.list_failed) == 1 and 'armed' in cap.list_failed[0][1]

    def test_list_while_download_active_fails(self, tmp_path):
        link, cap, fake, _ = start_download(tmp_path, 1000)
        link.request_log_list()
        drain(link)
        assert fake.requests('list') == []
        assert len(cap.list_failed) == 1 and 'id 3' in cap.list_failed[0][1]
        link.stop()

    def test_disarm_statustext_fails_list_now(self):
        link, cap = make_link(FakeMav())
        link.request_log_list()
        drain(link)
        link._handle_message(statustext(1, 100, 'Disarm for log download'))   # 相機說的不算
        assert cap.list_failed == []
        link._handle_message(statustext(1, 1, 'Disarm for log download'))
        assert len(cap.list_failed) == 1 and 'Disarm for log download' in cap.list_failed[0][1]


# ═════════════════════════════════════════════════════════════════════
# ② 下載：請求
# ═════════════════════════════════════════════════════════════════════
class TestDownloadRequest:
    def test_armed_fails_immediately_without_request(self, tmp_path):
        fake = FakeMav()
        link, cap = make_link(fake)
        link._handle_message(hb(1, 1, base_mode=ARMED))
        link.download_log(3, str(tmp_path / 'a.bin'), 1000)
        assert len(cap.failed) == 1
        assert cap.failed[0][0] == 1 and 'armed' in cap.failed[0][1]
        assert link._cmd_queue.empty() and fake.sent == []
        assert link.log_download_status() is None
        assert not (tmp_path / 'a.bin.part').exists()

    def test_armed_between_queue_and_dispatch_also_fails(self, tmp_path):
        fake = FakeMav()
        link, cap = make_link(fake)
        link.download_log(3, str(tmp_path / 'a.bin'), 1000)
        link._handle_message(hb(1, 1, base_mode=ARMED))
        drain(link)
        assert len(cap.failed) == 1 and 'armed' in cap.failed[0][1]
        assert fake.sent == [] and not (tmp_path / 'a.bin.part').exists()

    def test_request_covers_whole_log_and_opens_part_file(self, tmp_path, clock):
        link, cap, fake, dest = start_download(tmp_path, 1000)
        assert fake.sent == [('data', 1, 1, 3, 0, 0xFFFFFFFF)]
        assert os.path.exists(dest + '.part') and not os.path.exists(dest)
        st = link.log_download_status()
        assert st['phase'] == 'active' and st['received'] == 0 and st['size'] == 1000
        assert st['path'] == dest and st['id'] == 3
        assert cap.progress == [(1, cap.progress[0][1])] and cap.progress[0][1]['received'] == 0
        assert cap.failed == [] and cap.done == []
        link.stop()

    def test_second_download_while_active_fails_immediately(self, tmp_path):
        link, cap, fake, dest = start_download(tmp_path, 1000)
        link.download_log(4, str(tmp_path / 'other.bin'), 500)
        assert len(cap.failed) == 1
        assert cap.failed[0][1].startswith('已有下載進行中') and 'id 3' in cap.failed[0][1]
        assert link._cmd_queue.empty()
        assert link.log_download_status()['id'] == 3
        assert not (tmp_path / 'other.bin.part').exists()
        link.stop()

    def test_not_connected_fails_immediately(self, tmp_path):
        link, cap = make_link(None)
        link.download_log(3, str(tmp_path / 'a.bin'), 1000)
        assert len(cap.failed) == 1 and '尚未連線' in cap.failed[0][1]
        assert link._cmd_queue.empty() and link.log_download_status() is None

    @pytest.mark.parametrize('log_id,size,dest', [
        (3, 0, 'a.bin'), (3, -5, 'a.bin'), (-1, 100, 'a.bin'), ('x', 100, 'a.bin'),
        (3, 'big', 'a.bin'), (3, 100, ''), (3, 100, None),
    ])
    def test_bad_arguments_fail_immediately(self, tmp_path, log_id, size, dest):
        fake = FakeMav()
        link, cap = make_link(fake)
        d = str(tmp_path / dest) if dest else dest
        link.download_log(log_id, d, size)
        assert len(cap.failed) == 1 and cap.failed[0][0] == 1
        assert fake.sent == [] and link._cmd_queue.empty()
        assert link.log_download_status() is None

    def test_status_is_a_copy(self, tmp_path):
        link, cap, fake, dest = start_download(tmp_path, 1000)
        a = link.log_download_status()
        a['received'] = 999
        assert link.log_download_status()['received'] == 0
        link.stop()


# ═════════════════════════════════════════════════════════════════════
# ② 下載：資料
# ═════════════════════════════════════════════════════════════════════
class TestDownloadData:
    def test_happy_path_in_order(self, tmp_path, clock):
        data = content(1000)                       # 11 × 90 + 10：尾塊 count=10 (<90)
        link, cap, fake, dest = start_download(tmp_path, 1000)
        clock.advance(2.0)
        feed(link, data)
        assert cap.failed == []
        assert len(cap.done) == 1
        sysid, d = cap.done[0]
        assert sysid == 1 and d['id'] == 3 and d['size'] == 1000 and d['refills'] == 0
        assert d['path'] == dest and os.path.exists(dest)
        assert os.path.getsize(dest) == 1000
        assert open(dest, 'rb').read() == data
        assert d['sha256'] == hashlib.sha256(data).hexdigest()
        assert d['seconds'] == pytest.approx(2.0)
        assert not os.path.exists(dest + '.part')
        assert fake.requests('end') == [('end', 1, 1)]
        assert link.log_download_status() is None
        # 最後一則進度 received == size，而且在 done 之前
        assert cap.progress[-1][1]['received'] == 1000
        assert cap.progress[-1][1]['bytes_per_s'] == pytest.approx(500.0)

    def test_only_correct_blocks_are_written(self, tmp_path):
        data = content(1000)
        link, cap, fake, dest = start_download(tmp_path, 1000)
        link._handle_message(hb(2, 1))             # sysid 2：已確認的另一架（fan-in）
        bl = blocks_of(data)
        garbage = bytes([0xEE] * CHUNK)

        # 別人的／錯的塊：一塊都不能算
        link._handle_message(log_data(1, 100, 3, 0, garbage))         # 同 sysid 相機偽造
        link._handle_message(log_data(2, 1, 3, 90, garbage))          # 已確認的別架
        link._handle_message(log_data(9, 1, 3, 180, garbage))         # 沒確認過的來源
        link._handle_message(log_data(1, 1, 4, 270, garbage))         # 別的 id
        link._handle_message(log_data(1, 1, 3, 1000, garbage))        # ofs == size
        link._handle_message(log_data(1, 1, 3, 1080, garbage))        # ofs > size（下載中檔案長大）
        link._handle_message(log_data(1, 1, 3, 360, b''))             # count = 0
        st = link.log_download_status()
        assert st['received'] == 0 and st['blocks'] == 0
        assert st['ignored_other_id'] == 1 and st['ignored_other_src'] == 1 and st['extra_blocks'] == 2

        # 亂序 + 重複 + 尾塊
        order = [11, 0, 5, 5, 3, 1, 9, 2, 7, 4, 8, 10, 6]
        feed(link, data, order=order)
        assert cap.failed == []
        assert len(cap.done) == 1
        assert open(dest, 'rb').read() == data
        assert cap.done[0][1]['sha256'] == hashlib.sha256(data).hexdigest()
        assert os.path.getsize(dest) == 1000
        assert fake.requests('end') == [('end', 1, 1)]

    def test_missing_blocks_are_refilled_after_idle(self, tmp_path, clock):
        data = content(1000)
        link, cap, fake, dest = start_download(tmp_path, 1000)
        missing = {3, 4, 7, 11}                    # 3-4 連續一段、7 一段、尾塊 11（10 B）一段
        feed(link, data, order=[i for i in range(12) if i not in missing])
        assert cap.done == [] and cap.failed == []
        assert len(fake.requests('data')) == 1

        clock.advance(0.9)
        link._log_transfer_tick()
        assert len(fake.requests('data')) == 1     # 還沒閒置到 1 s
        clock.advance(0.2)
        link._log_transfer_tick()
        # 一次只讓一段在飛：先要第一段 (270, 180)
        assert fake.requests('data')[1:] == [('data', 1, 1, 3, 270, 180)]
        assert link.log_download_status()['refills'] == 1
        # 同一輪內再 tick 不會重送（last_rx 已更新）
        link._log_transfer_tick()
        assert len(fake.requests('data')) == 2

        feed(link, data, order=[3])
        assert len(fake.requests('data')) == 2     # 段還沒齊
        feed(link, data, order=[4])
        assert fake.requests('data')[2] == ('data', 1, 1, 3, 630, 90)     # 段收齊 ⇒ 立刻要下一段
        feed(link, data, order=[7])
        assert fake.requests('data')[3] == ('data', 1, 1, 3, 990, 10)     # 尾段 count 只到 size
        feed(link, data, order=[11])
        assert len(cap.done) == 1 and cap.failed == []
        assert cap.done[0][1]['refills'] == 1
        assert open(dest, 'rb').read() == data
        assert len(fake.requests('data')) == 4

    def test_refill_round_sends_at_most_20_segments(self, tmp_path, clock):
        n_blocks = 60
        size = n_blocks * CHUNK
        data = content(size)
        link, cap, fake, dest = start_download(tmp_path, size)
        missing = set(range(0, 60, 2))             # 30 段孤立缺塊
        feed(link, data, order=[i for i in range(n_blocks) if i not in missing])
        clock.advance(1.0)
        link._log_transfer_tick()
        assert fake.requests('data')[1:] == [('data', 1, 1, 3, 0, 90)]
        # 一段一段補：前 20 段屬於這一輪
        for k, bi in enumerate(sorted(missing)[:20]):
            feed(link, data, order=[bi])
            if k < 19:
                nxt = sorted(missing)[k + 1] * CHUNK
                assert fake.requests('data')[-1] == ('data', 1, 1, 3, nxt, 90)
        assert len(fake.requests('data')) == 21    # 第 21 段要等下一輪
        assert link.log_download_status()['refills'] == 1
        clock.advance(1.0)
        link._log_transfer_tick()
        assert fake.requests('data')[-1] == ('data', 1, 1, 3, 40 * CHUNK, 90)
        assert link.log_download_status()['refills'] == 2
        for bi in sorted(missing)[20:]:
            feed(link, data, order=[bi])
        assert len(cap.done) == 1 and cap.done[0][1]['refills'] == 2
        assert open(dest, 'rb').read() == data

    def test_progress_is_throttled(self, tmp_path, clock):
        size = 100 * CHUNK
        data = content(size)
        link, cap, fake, dest = start_download(tmp_path, size)
        feed(link, data)                           # 時鐘不動、< 1 MB ⇒ 只有開始 + 最後
        assert len(cap.done) == 1
        assert [p[1]['received'] for p in cap.progress] == [0, size]
        assert cap.progress[-1][1]['path'] == dest

    def test_progress_emits_on_time_and_on_megabyte(self, tmp_path, clock):
        # 時間：每 0.6 s 一塊 ⇒ 每塊都過 0.5 s 門檻 ⇒ 每塊一則
        size = 10 * CHUNK
        data = content(size)
        link, cap, fake, dest = start_download(tmp_path, size)
        for i in range(9):
            clock.advance(0.6)
            feed(link, data, order=[i])
        assert [p[1]['received'] for p in cap.progress] == [0] + [CHUNK * k for k in range(1, 10)]
        feed(link, data, order=[9])
        assert cap.progress[-1][1]['received'] == size and len(cap.done) == 1

        # 位元組：跨過 1 MB 多一則（時鐘不動）
        size2 = 1_000_000 + CHUNK                  # 11,112 塊；第 11,112 塊時 received = 1,000,080 ≥ 1 MB
        data2 = content(size2, seed=3)
        link2, cap2, fake2, dest2 = start_download(tmp_path, size2, log_id=5, name='log005.bin')
        feed(link2, data2, log_id=5)
        assert len(cap2.done) == 1
        assert [p[1]['received'] for p in cap2.progress] == [0, 11_112 * CHUNK, size2]
        assert cap2.done[0][1]['sha256'] == hashlib.sha256(data2).hexdigest()

    def test_eta_and_rate_from_bytes(self, tmp_path, clock):
        data = content(1000)
        link, cap, fake, dest = start_download(tmp_path, 1000)
        clock.advance(1.0)
        feed(link, data, order=[0, 1])             # 180 B / 1 s
        st = link.log_download_status()
        assert st['bytes_per_s'] == pytest.approx(180.0)
        assert st['eta_s'] == pytest.approx((1000 - 180) / 180.0)
        link.stop()


# ═════════════════════════════════════════════════════════════════════
# ③ 失敗路徑：每一條都要大聲、恰好一次、.part 不留
# ═════════════════════════════════════════════════════════════════════
class TestFailures:
    def test_cancel(self, tmp_path):
        data = content(1000)
        link, cap, fake, dest = start_download(tmp_path, 1000)
        feed(link, data, order=[0, 1])
        link.cancel_log_download()
        assert drain(link) == 1
        assert len(cap.failed) == 1 and '取消' in cap.failed[0][1]
        assert not os.path.exists(dest + '.part') and not os.path.exists(dest)
        assert fake.requests('end') == [('end', 1, 1)]
        assert link.log_download_status() is None
        assert cap.done == []
        # 之後再來的塊、tick、stop 都不會再 emit
        feed(link, data, order=[2])
        link._log_transfer_tick()
        link.stop()
        assert len(cap.failed) == 1

    def test_cancel_flag_is_seen_by_tick_before_the_queue(self, tmp_path):
        link, cap, fake, dest = start_download(tmp_path, 1000)
        link.cancel_log_download()
        link._log_transfer_tick()                  # 佇列還沒排空，tick 先看到旗標
        assert len(cap.failed) == 1 and '取消' in cap.failed[0][1]
        drain(link)
        assert len(cap.failed) == 1 and fake.requests('end') == [('end', 1, 1)]

    def test_cancel_with_nothing_in_progress_is_silent(self):
        link, cap = make_link(FakeMav())
        link.cancel_log_download()
        drain(link)
        assert cap.failed == [] and cap.done == []

    def test_existing_destination_gets_suffix(self, tmp_path):
        data = content(1000)
        dest = tmp_path / 'log003.bin'
        dest.write_bytes(b'OLD-FLIGHT')
        link, cap, fake, _ = start_download(tmp_path, 1000)
        feed(link, data)
        assert cap.failed == [] and len(cap.done) == 1
        assert cap.done[0][1]['path'] == str(tmp_path / 'log003_2.bin')
        assert (tmp_path / 'log003_2.bin').read_bytes() == data
        assert dest.read_bytes() == b'OLD-FLIGHT'
        assert not (tmp_path / 'log003.bin.part').exists()
        assert not (tmp_path / 'log003_2.bin.part').exists()

        # _2、_3 都在 ⇒ _4；殘留的 .part 也算佔用
        (tmp_path / 'log003_3.bin').write_bytes(b'x')
        (tmp_path / 'log003_4.bin.part').write_bytes(b'x')
        link2, cap2, fake2, _ = start_download(tmp_path, 1000)
        feed(link2, data)
        assert cap2.done[0][1]['path'] == str(tmp_path / 'log003_5.bin')
        assert (tmp_path / 'log003_5.bin').read_bytes() == data
        assert (tmp_path / 'log003_3.bin').read_bytes() == b'x'

    def test_destination_created_during_download_is_not_overwritten(self, tmp_path):
        data = content(1000)
        link, cap, fake, dest = start_download(tmp_path, 1000)
        feed(link, data, order=list(range(11)))
        open(dest, 'wb').write(b'SOMEONE ELSE')   # 下載中有人建立了同名檔
        feed(link, data, order=[11])
        assert len(cap.done) == 1
        assert cap.done[0][1]['path'] == str(tmp_path / 'log003_2.bin')
        assert open(dest, 'rb').read() == b'SOMEONE ELSE'
        assert (tmp_path / 'log003_2.bin').read_bytes() == data

    def test_parent_is_a_file_fails_with_reason(self, tmp_path):
        (tmp_path / 'afile').write_bytes(b'x')
        fake = FakeMav()
        link, cap = make_link(fake)
        link.download_log(3, str(tmp_path / 'afile' / 'log.bin'), 1000)
        drain(link)
        assert len(cap.failed) == 1
        assert '無法建立暫存檔' in cap.failed[0][1] and 'Error' in cap.failed[0][1]
        assert fake.sent == []
        assert link.log_download_status() is None

    def test_open_raising_oserror_fails_with_reason(self, tmp_path, monkeypatch):
        def boom(*a, **k):
            raise OSError(28, 'No space left on device')
        monkeypatch.setattr(sl, 'open', boom, raising=False)
        fake = FakeMav()
        link, cap = make_link(fake)
        link.download_log(3, str(tmp_path / 'log.bin'), 1000)
        drain(link)
        assert len(cap.failed) == 1
        assert 'No space left on device' in cap.failed[0][1]
        assert fake.sent == [] and link.log_download_status() is None

    def test_write_failure_mid_download(self, tmp_path):
        data = content(1000)
        link, cap, fake, dest = start_download(tmp_path, 1000)
        feed(link, data, order=[0])

        class Broken:
            def __init__(self, fh):
                self._fh = fh
            def seek(self, *a):
                return self._fh.seek(*a)
            def write(self, b):
                raise OSError(5, 'Input/output error')
            def close(self):
                return self._fh.close()
        with link._log_lock:
            link._log_dl['fh'] = Broken(link._log_dl['fh'])
        feed(link, data, order=[1])
        assert len(cap.failed) == 1
        assert '寫檔失敗' in cap.failed[0][1] and 'Input/output error' in cap.failed[0][1]
        assert not os.path.exists(dest + '.part') and not os.path.exists(dest)
        assert fake.requests('end') == [('end', 1, 1)]
        assert cap.done == [] and link.log_download_status() is None

    def test_sixty_seconds_without_progress_fails(self, tmp_path, clock):
        data = content(1000)
        link, cap, fake, dest = start_download(tmp_path, 1000)
        feed(link, data, order=[0])
        clock.advance(59.0)
        link._log_transfer_tick()
        assert cap.failed == []
        clock.advance(1.0)
        link._log_transfer_tick()
        assert len(cap.failed) == 1
        assert '60' in cap.failed[0][1] and '90/1,000' in cap.failed[0][1]
        assert not os.path.exists(dest + '.part')
        assert fake.requests('end') == [('end', 1, 1)]

    def test_no_data_at_all_names_the_likely_causes(self, tmp_path, clock):
        link, cap, fake, dest = start_download(tmp_path, 1000)
        clock.advance(60.0)
        link._log_transfer_tick()
        assert len(cap.failed) == 1
        r = cap.failed[0][1]
        assert '沒收到任何 LOG_DATA' in r and 'id 3 無效' in r
        assert not os.path.exists(dest + '.part')

    def test_400_refill_rounds_fails(self, tmp_path, clock):
        data = content(1000)
        link, cap, fake, dest = start_download(tmp_path, 1000)
        feed(link, data, order=[0])
        with link._log_lock:
            link._log_dl['refills'] = 400
        clock.advance(1.0)
        link._log_transfer_tick()
        assert len(cap.failed) == 1 and '400' in cap.failed[0][1] and '仍缺 1 段' in cap.failed[0][1]
        assert not os.path.exists(dest + '.part')

    def test_stop_fails_with_link_closed_exactly_once(self, tmp_path):
        data = content(1000)
        link, cap, fake, dest = start_download(tmp_path, 1000)
        feed(link, data, order=[0, 1])
        link.stop()
        assert cap.failed == [(1, '連線結束')]
        assert not os.path.exists(dest + '.part') and not os.path.exists(dest)
        assert fake.requests('end') == []          # GUI 執行緒不碰 _mav
        assert fake.closed == 1
        link.stop()
        link._log_transfer_tick()
        feed(link, data, order=[2])
        assert len(cap.failed) == 1 and cap.done == []

    def test_stop_before_dispatch_fails_queued_download(self, tmp_path):
        fake = FakeMav()
        link, cap = make_link(fake)
        link.download_log(3, str(tmp_path / 'a.bin'), 1000)
        link.stop()
        assert cap.failed == [(1, '連線結束')]
        drain(link)                                # 之後才排空佇列：不會再 emit、不開檔、不送請求
        assert len(cap.failed) == 1 and fake.sent == []
        assert not (tmp_path / 'a.bin.part').exists()

    def test_run_ending_fails_with_link_closed(self, tmp_path, monkeypatch):
        fake = FakeMav()
        dest = str(tmp_path / 'run.bin')

        class _FakeConn(FakeMav):
            def __init__(self, link):
                super().__init__()
                self.link = link
                self.message_hooks = []
                self.calls = 0

            def wait_heartbeat(self, timeout=None, blocking=True):
                return hb(1, 1)

            def recv_match(self, *a, **k):
                self.calls += 1
                if self.calls == 1:
                    self.link.download_log(3, dest, 1000)    # 第一圈：GUI 下了下載指令
                    return None
                self.link._stop = True                       # 第二圈：指令已派送、.part 已開 ⇒ 結束
                return None

        link, cap = make_link(None)
        conn = _FakeConn(link)
        monkeypatch.setattr(mavutil, 'mavlink_connection', lambda *a, **k: conn)
        monkeypatch.setattr(link, '_request_servo_function_params', lambda: None)
        monkeypatch.setattr(link, '_verify_vtol_q_enable', lambda: None)
        link.run()
        assert conn.requests('data') == [('data', 1, 1, 3, 0, 0xFFFFFFFF)]
        assert cap.failed == [(1, '連線結束')]
        assert not os.path.exists(dest + '.part') and not os.path.exists(dest)
        assert link.log_download_status() is None

    def test_disarm_statustext_fails_download_now(self, tmp_path):
        link, cap, fake, dest = start_download(tmp_path, 1000)
        link._handle_message(statustext(2, 1, 'Disarm for log download'))      # 別架說的不算
        link._handle_message(statustext(1, 100, 'Disarm for log download'))    # 相機說的不算
        assert cap.failed == []
        link._handle_message(statustext(1, 1, 'Disarm for log download'))
        assert len(cap.failed) == 1 and 'Disarm for log download' in cap.failed[0][1]
        assert not os.path.exists(dest + '.part')

    def test_log_download_in_progress_statustext_fails_download_now(self, tmp_path):
        link, cap, fake, dest = start_download(tmp_path, 1000)
        link._handle_message(statustext(1, 1, 'Log download in progress'))
        assert len(cap.failed) == 1 and 'Log download in progress' in cap.failed[0][1]

    def test_failure_is_also_reported_on_status_text(self, tmp_path):
        link, cap, fake, dest = start_download(tmp_path, 1000)
        link.cancel_log_download()
        drain(link)
        assert any(sev == 3 and '日誌 id 3 下載失敗' in t for sev, t in cap.status)


# ═════════════════════════════════════════════════════════════════════
# ⑤-1 鏈路已結束：請求不可以靜默躺在佇列裡（stop() 關了 _mav 但不會把它設成 None）
# ═════════════════════════════════════════════════════════════════════
class _RunConn(FakeMav):
    """給 run() 用的假 mavlink_connection：wait_heartbeat 回 1 號機心跳；recv_match 由子類決定。"""
    def __init__(self, link=None):
        super().__init__()
        self.link = link
        self.message_hooks = []
        self.calls = 0

    def wait_heartbeat(self, timeout=None, blocking=True):
        return hb(1, 1)


def run_link_with(link, conn, monkeypatch):
    monkeypatch.setattr(mavutil, 'mavlink_connection', lambda *a, **k: conn)
    monkeypatch.setattr(link, '_request_servo_function_params', lambda: None)
    monkeypatch.setattr(link, '_verify_vtol_q_enable', lambda: None)
    link.run()


class TestDeadLink:
    def test_requests_after_stop_fail_immediately(self, tmp_path):
        fake = FakeMav()
        link, cap = make_link(fake)
        link._handle_message(hb(1, 1))
        link.stop()
        assert fake.closed == 1 and link._mav is not None      # 舊碼就是靠這個狀態過了 `_mav is None` 守衛
        link.download_log(3, str(tmp_path / 'a.bin'), 1000)
        assert cap.failed == [(1, '連線已結束')]
        link.request_log_list()
        assert cap.list_failed == [(1, '連線已結束')]
        assert link._cmd_queue.empty()                          # 沒登記、沒排佇列
        assert link.log_download_status() is None
        assert not (tmp_path / 'a.bin.part').exists()
        link.cancel_log_download()                              # 也不排佇列、不多 emit
        assert link._cmd_queue.empty() and len(cap.failed) == 1
        assert fake.sent == []

    def test_requests_after_run_exited_by_recv_exception_fail_immediately(self, tmp_path, monkeypatch):
        class _Conn(_RunConn):
            def recv_match(self, *a, **k):
                raise RuntimeError('socket gone')

        link, cap = make_link(None)
        conn = _Conn()
        run_link_with(link, conn, monkeypatch)
        assert link._mav is conn and conn.closed == 0           # run() 退出不關、不清 _mav
        link.download_log(3, str(tmp_path / 'a.bin'), 1000)
        link.request_log_list()
        assert cap.failed == [(1, '連線已結束')]
        assert cap.list_failed == [(1, '連線已結束')]
        assert link._cmd_queue.empty() and link.log_download_status() is None
        assert not (tmp_path / 'a.bin.part').exists()
        assert conn.requests('list') == [] and conn.requests('data') == []

    def test_list_queued_but_never_dispatched_fails_once_when_run_ends(self, monkeypatch):
        class _Conn(_RunConn):
            def recv_match(self, *a, **k):
                self.link.request_log_list()      # 這一圈的佇列已經排空過了
                self.link._stop = True            # ⇒ 這則 'log_list' 永遠不會被派送
                return None

        link, cap = make_link(None)
        conn = _Conn(link)
        run_link_with(link, conn, monkeypatch)
        assert conn.requests('list') == []
        assert cap.list_failed == [(1, '連線結束')]
        assert cap.lists == [] and link._cmd_queue.empty()

    def test_download_queued_but_never_dispatched_fails_exactly_once(self, tmp_path, monkeypatch):
        dest = str(tmp_path / 'q.bin')

        class _Conn(_RunConn):
            def recv_match(self, *a, **k):
                self.link.download_log(3, dest, 1000)
                self.link._stop = True
                return None

        link, cap = make_link(None)
        conn = _Conn(link)
        run_link_with(link, conn, monkeypatch)
        assert conn.requests('data') == []
        assert cap.failed == [(1, '連線結束')]                   # 登記的狀態與佇列裡的指令是同一份：只失敗一次
        assert not os.path.exists(dest + '.part') and link.log_download_status() is None
        assert link._cmd_queue.empty()

    def test_stop_fails_queued_list_once_and_keeps_other_commands(self):
        fake = FakeMav()
        link, cap = make_link(fake)
        link.request_log_list()
        link.arm()                                              # 別的指令要原樣留在佇列裡
        link.request_log_list()
        link.stop()
        assert cap.list_failed == [(1, '連線結束'), (1, '連線結束')]
        assert list(link._cmd_queue.queue) == [('arm', None)]
        assert fake.sent == []
        link.stop()                                             # 再 stop 不會再 emit
        assert len(cap.list_failed) == 2


# ═════════════════════════════════════════════════════════════════════
# ⑤-2 fan-in：pymavlink 把 target_system 鎖在第一個心跳的那架（這裡是 2），這條鏈路 UI 標示的是 1
# ═════════════════════════════════════════════════════════════════════
class TestFanInBinding:
    @staticmethod
    def _fanin(armed_1=False, armed_2=False):
        fake = FakeMav(target_system=2)
        link, cap = make_link(fake, sysid=1)
        link._handle_message(hb(2, 1, base_mode=ARMED if armed_2 else DISARMED))   # 2 號機先開口
        link._handle_message(hb(1, 1, base_mode=ARMED if armed_1 else DISARMED))
        return link, cap, fake

    def test_list_request_goes_to_sysid_label_and_only_its_entries_count(self):
        link, cap, fake = self._fanin()
        link.request_log_list()
        drain(link)
        assert fake.sent == [('list', 1, 1, 0, 0xFFFF)]
        link._handle_message(log_entry(2, 1, 1, 1, 100, 10))    # 2 號機（自駕儀元件）回的清單：不是我們要的
        assert cap.lists == [] and cap.list_failed == []
        link._handle_message(statustext(2, 1, 'Disarm for log download'))   # 2 號機的拒絕句也不算
        assert cap.list_failed == []
        link._handle_message(log_entry(1, 1, 1, 1, 100, 10))
        assert cap.lists == [(1, [{'id': 1, 'size': 100, 'time_utc': 10}])]

    def test_download_is_bound_to_sysid_label(self, tmp_path):
        data = content(1000)
        link, cap, fake = self._fanin(armed_2=True)              # 2 號機 armed 與我們無關
        link.download_log(3, str(tmp_path / 'a.bin'), 1000)
        assert cap.failed == []
        assert drain(link) == 1
        assert cap.failed == []
        assert fake.sent == [('data', 1, 1, 3, 0, 0xFFFFFFFF)]
        assert link.log_download_status()['sysid'] == 1
        assert cap.progress == [(1, cap.progress[0][1])]
        feed(link, data, src=2)                                   # 2 號機送來的塊：全部不算
        st = link.log_download_status()
        assert st['received'] == 0 and st['ignored_other_src'] == 12
        link._handle_message(statustext(2, 1, 'Log download in progress'))   # 2 號機說的不算
        assert cap.failed == []
        feed(link, data, src=1)
        assert cap.failed == [] and len(cap.done) == 1
        assert cap.done[0][0] == 1 and all(s == 1 for s, _ in cap.progress)
        assert fake.requests('end') == [('end', 1, 1)]
        assert (tmp_path / 'a.bin').read_bytes() == data

    def test_armed_precheck_looks_at_sysid_label_not_target_system(self, tmp_path):
        link, cap, fake = self._fanin(armed_1=True)              # 我們這架 armed、2 號機沒有
        link.download_log(3, str(tmp_path / 'a.bin'), 1000)
        assert len(cap.failed) == 1 and cap.failed[0][0] == 1 and 'armed' in cap.failed[0][1]
        assert fake.sent == [] and link._cmd_queue.empty()
        link.request_log_list()
        drain(link)
        assert len(cap.list_failed) == 1 and cap.list_failed[0][0] == 1 and 'armed' in cap.list_failed[0][1]
        assert fake.sent == []

    def test_failure_signals_carry_sysid_label(self, tmp_path, clock):
        link, cap, fake = self._fanin()
        link.download_log(3, str(tmp_path / 'a.bin'), 1000)
        drain(link)
        clock.advance(60.0)
        link._log_transfer_tick()
        assert len(cap.failed) == 1 and cap.failed[0][0] == 1
        assert fake.requests('end') == [('end', 1, 1)]
        link.request_log_list()
        drain(link)
        clock.advance(8.5)
        link._log_transfer_tick()
        clock.advance(8.5)
        link._log_transfer_tick()
        assert len(cap.list_failed) == 1 and cap.list_failed[0][0] == 1
        assert fake.requests('list') == [('list', 1, 1, 0, 0xFFFF)] * 2


# ═════════════════════════════════════════════════════════════════════
# ⑤-3 飛控的拒絕句只在什麼都還沒收到時才算；已收過資料而放棄 ⇒ LOG_REQUEST_END
# ═════════════════════════════════════════════════════════════════════
class TestRejectionTextOnlyBeforeData:
    def test_in_progress_text_is_ignored_once_blocks_are_flowing(self, tmp_path):
        data = content(1000)
        link, cap, fake, dest = start_download(tmp_path, 1000)
        feed(link, data, order=[0])
        link._handle_message(statustext(1, 1, 'Log download in progress'))   # 共用埠上 Mission Planner 要清單被拒
        link._handle_message(statustext(1, 1, 'Disarm for log download'))
        assert cap.failed == [] and link.log_download_status()['phase'] == 'active'
        assert os.path.exists(dest + '.part')
        assert fake.requests('end') == []
        feed(link, data, order=list(range(1, 12)))
        assert cap.failed == [] and len(cap.done) == 1
        assert cap.done[0][1]['sha256'] == hashlib.sha256(data).hexdigest()

    def test_in_progress_text_with_nothing_received_rejects_without_end(self, tmp_path):
        link, cap, fake, dest = start_download(tmp_path, 1000)
        link._handle_message(statustext(1, 1, 'Log download in progress'))
        assert len(cap.failed) == 1 and 'Log download in progress' in cap.failed[0][1]
        assert not os.path.exists(dest + '.part')
        assert fake.requests('end') == []        # 飛控在服務別條鏈路：END 不認發信者，送了會砍掉人家的傳輸

    def test_in_progress_text_is_ignored_once_entries_are_flowing(self):
        fake = FakeMav()
        link, cap = make_link(fake)
        link.request_log_list()
        drain(link)
        link._handle_message(log_entry(1, 1, 1, 2, 100, 10))
        link._handle_message(statustext(1, 1, 'Log download in progress'))   # 列表重送時飛控還在送清單就會這樣回
        link._handle_message(statustext(1, 1, 'Disarm for log download'))
        assert cap.list_failed == [] and link._log_list_state is not None
        link._handle_message(log_entry(1, 1, 2, 2, 200, 20))
        assert len(cap.lists) == 1 and cap.list_failed == []

    def test_abandoning_a_download_that_received_data_sends_request_end(self, tmp_path, clock, monkeypatch):
        data = content(1000)
        # 補抓輪數上限
        link, cap, fake, dest = start_download(tmp_path, 1000)
        feed(link, data, order=[0])
        with link._log_lock:
            link._log_dl['refills'] = 400
        clock.advance(1.0)
        link._log_transfer_tick()
        assert len(cap.failed) == 1 and fake.requests('end') == [('end', 1, 1)]
        # 狀態機例外
        link2, cap2, fake2, dest2 = start_download(tmp_path, 1000, name='b.bin')
        feed(link2, data, order=[0])
        monkeypatch.setattr(link2, '_log_tick_download', lambda: 1 / 0)
        link2._log_transfer_tick()
        assert len(cap2.failed) == 1 and fake2.requests('end') == [('end', 1, 1)]
        # 無進展逾時
        link3, cap3, fake3, dest3 = start_download(tmp_path, 1000, name='c.bin')
        feed(link3, data, order=[0])
        clock.advance(60.0)
        link3._log_transfer_tick()
        assert len(cap3.failed) == 1 and fake3.requests('end') == [('end', 1, 1)]
        for d in (dest, dest2, dest3):
            assert not os.path.exists(d + '.part')


# ═════════════════════════════════════════════════════════════════════
# ⑤-4 每塊都到之後收尾失敗：.part 不刪、原因寫出路徑；os.rename 重試
# ═════════════════════════════════════════════════════════════════════
class TestFinishingKeepsPart:
    def test_rename_failure_keeps_the_complete_part_and_retries(self, tmp_path, monkeypatch):
        data = content(1000)
        real_rename = os.rename
        link, cap, fake, dest = start_download(tmp_path, 1000)
        calls, sleeps = [], []

        def boom(src, dst):
            calls.append((src, dst))
            raise PermissionError(13, 'being used by another process')
        monkeypatch.setattr(sl.os, 'rename', boom)
        monkeypatch.setattr(sl.time, 'sleep', lambda s: sleeps.append(s))
        feed(link, data)
        assert cap.done == []
        assert len(cap.failed) == 1
        sysid, reason = cap.failed[0]
        assert sysid == 1
        assert '改名' in reason and 'PermissionError' in reason
        assert (dest + '.part') in reason                       # 原因裡寫了保留的 .part 路徑
        assert os.path.exists(dest + '.part')
        assert os.path.getsize(dest + '.part') == 1000
        assert (tmp_path / 'log003.bin.part').read_bytes() == data
        assert not os.path.exists(dest)
        assert len(calls) == 5 and all(c == (dest + '.part', dest) for c in calls)
        assert len(sleeps) == 4 and sum(sleeps) <= 1.0
        assert fake.requests('end') == [('end', 1, 1)]
        assert link.log_download_status() is None
        # 下一趟同名下載不會蓋到保留的 .part：改用 _2
        monkeypatch.setattr(sl.os, 'rename', real_rename)
        link2, cap2, fake2, _ = start_download(tmp_path, 1000)
        feed(link2, data)
        assert cap2.failed == [] and cap2.done[0][1]['path'] == str(tmp_path / 'log003_2.bin')
        assert os.path.getsize(dest + '.part') == 1000

    def test_rename_transient_failure_is_retried_then_done(self, tmp_path, monkeypatch):
        data = content(1000)
        real_rename = os.rename
        n = {'calls': 0}

        def flaky(src, dst):
            n['calls'] += 1
            if n['calls'] < 3:
                raise PermissionError(13, 'busy')
            return real_rename(src, dst)
        monkeypatch.setattr(sl.os, 'rename', flaky)
        monkeypatch.setattr(sl.time, 'sleep', lambda s: None)
        link, cap, fake, dest = start_download(tmp_path, 1000)
        feed(link, data)
        assert cap.failed == [] and len(cap.done) == 1 and n['calls'] == 3
        assert os.path.exists(dest) and not os.path.exists(dest + '.part')
        assert cap.done[0][1]['sha256'] == hashlib.sha256(data).hexdigest()

    def test_sha256_failure_keeps_the_part(self, tmp_path, monkeypatch):
        data = content(1000)
        link, cap, fake, dest = start_download(tmp_path, 1000)

        def boom(path):
            raise OSError(5, 'Input/output error')
        monkeypatch.setattr(SITLLink, '_sha256_file', staticmethod(boom))
        feed(link, data)
        assert cap.done == [] and len(cap.failed) == 1
        assert 'sha256' in cap.failed[0][1] and (dest + '.part') in cap.failed[0][1]
        assert os.path.getsize(dest + '.part') == 1000 and not os.path.exists(dest)
        assert (tmp_path / 'log003.bin.part').read_bytes() == data

    def test_failure_before_finishing_still_deletes_the_part(self, tmp_path, clock):
        data = content(1000)
        link, cap, fake, dest = start_download(tmp_path, 1000)
        feed(link, data, order=list(range(11)))                 # 差尾塊
        clock.advance(60.0)
        link._log_transfer_tick()
        assert len(cap.failed) == 1 and '.part' not in cap.failed[0][1]
        assert not os.path.exists(dest + '.part')


# ═════════════════════════════════════════════════════════════════════
# 結構鎖：主迴圈真的會呼叫 tick、LOG_* 分支真的在守衛之後
# ═════════════════════════════════════════════════════════════════════
class TestStructure:
    def test_main_loop_ticks_even_when_recv_match_returns_none(self, monkeypatch):
        import ast
        import pathlib
        src = pathlib.Path(sl.__file__).read_text(encoding='utf-8')
        tree = ast.parse(src)
        run_link = next(n for n in ast.walk(tree)
                        if isinstance(n, ast.FunctionDef) and n.name == '_run_link')
        # 主迴圈：recv_match 之後、`if msg is None: continue` 之前要有 _log_transfer_tick()
        lines = src.splitlines()
        body = [l for l in lines[run_link.lineno - 1:run_link.end_lineno]]
        i_recv = next(i for i, l in enumerate(body) if 'recv_match(blocking=True, timeout=0.5)' in l)
        i_tick = next(i for i, l in enumerate(body) if 'self._log_transfer_tick()' in l)
        i_none = next(i for i, l in enumerate(body) if i > i_recv and l.strip() == 'if msg is None:')
        assert i_recv < i_tick < i_none

    def test_log_branches_follow_the_autopilot_guard(self):
        import pathlib
        src = pathlib.Path(sl.__file__).read_text(encoding='utf-8')
        i_guard = src.index('elif not self._is_autopilot_component(src, self._src_component(msg)):')
        i_entry = src.index("if mtype == 'LOG_ENTRY':")
        i_data = src.index("if mtype == 'LOG_DATA':")
        assert i_guard < i_entry < i_data

    def test_tick_swallows_exceptions_and_fails_loudly(self, tmp_path, monkeypatch):
        link, cap, fake, dest = start_download(tmp_path, 1000)
        monkeypatch.setattr(link, '_log_tick_download', lambda: 1 / 0)
        link._log_transfer_tick()                  # 不能把主迴圈炸掉
        assert len(cap.failed) == 1 and 'ZeroDivisionError' in cap.failed[0][1]
        assert not os.path.exists(dest + '.part')

    def test_no_log_erase_and_no_mavftp_anywhere(self):
        """AST 看「呼叫了什麼」，不看字串（docstring 本來就寫著「絕不送 LOG_ERASE」）。"""
        import ast
        import pathlib
        tree = ast.parse(pathlib.Path(sl.__file__).read_text(encoding='utf-8'))
        called = {n.func.attr for n in ast.walk(tree)
                  if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
        assert 'log_erase_send' not in called
        assert 'file_transfer_protocol_send' not in called
        assert {'log_request_list_send', 'log_request_data_send', 'log_request_end_send'} <= called
