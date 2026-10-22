# -*- coding: utf-8 -*-
"""tests/test_sitl_link_sysid_signals.py — SITLLink 的 status_text_sysid／servo_param_sysid（2026-10-22 群控／單機）。

SITL 頁要「選哪一台就列哪一台的訊息／SERVO」⇒ 鏈路的訊息要帶來源 sysid。40 多個 status_text.emit 不逐一改，
改用 DirectConnection 轉接 slot ＋ thread-local（見 sitl_link.py __init__ 尾端的註解）——這種隱性轉接一定要有
值層級測試，包括兩條執行緒同時 emit 不會標錯（L3）與「拿掉來源標記測試就會紅」（L7）。

不啟動 QThread（run() 不跑）：直接餵 _handle_message。只用 QtCore。
"""
from __future__ import annotations

import os
import sys
import threading
from types import SimpleNamespace

import pytest

os.environ.setdefault('AEROPLAN_NO_FILE_LOG', '1')

from PyQt6.QtCore import QCoreApplication, Qt  # noqa: E402

_APP = QCoreApplication.instance() or QCoreApplication([])

from mission.sitl_link import SITLLink  # noqa: E402


class FakeMsg:
    def __init__(self, mtype, src, comp=1, **fields):
        self._mtype, self._src, self._comp = mtype, src, comp
        for k, v in fields.items():
            setattr(self, k, v)

    def get_type(self):
        return self._mtype

    def get_srcSystem(self):
        return self._src

    def get_srcComponent(self):
        return self._comp


def _st(src, text, sev=4, comp=1):
    return FakeMsg('STATUSTEXT', src, comp, severity=sev, text=text)


def _pv(src, name, value, comp=1):
    return FakeMsg('PARAM_VALUE', src, comp, param_id=name, param_value=value,
                   param_type=9, param_count=900, param_index=65535)


def _link(label=1, vehicles=(2, 3)):
    link = SITLLink(conn_str='tcp:127.0.0.1:5760', sysid_label=label)
    # fan-in 的別台已經用心跳確認是飛機（run() 的 HEARTBEAT 分支會加進 _vehicle_srcs）；
    # 沒確認的來源說的 STATUSTEXT 是 'ext'（見 L9）
    link._vehicle_srcs.update(vehicles)
    link._sid_got, link._old_got, link._servo_sid, link._servo_old = [], [], [], []
    link.status_text_sysid.connect(lambda *a: link._sid_got.append(a), Qt.ConnectionType.DirectConnection)
    link.status_text.connect(lambda *a: link._old_got.append(a), Qt.ConnectionType.DirectConnection)
    link.servo_param_sysid.connect(lambda *a: link._servo_sid.append(a), Qt.ConnectionType.DirectConnection)
    link.servo_param.connect(lambda *a: link._servo_old.append(a), Qt.ConnectionType.DirectConnection)
    return link


def test_L1_statustext_carries_true_source_and_old_signal_fires_once():
    link = _link(label=1)
    link._handle_message(_st(2, 'x', sev=4))
    assert link._sid_got == [(2, 4, 'x', 'fc')]
    assert link._old_got == [(4, 'x')], '舊的 status_text 要照發、而且恰好一次'


def test_L2_gcs_side_emits_are_tagged_with_the_link_label_and_thread_local_resets():
    link = _link(label=1)
    link.status_text.emit(3, 'z')
    link._handle_message(_st(2, 'from-2', sev=6))
    link._report_mission_upload(False, 'y')            # 鏈路自己產生的（任務寫入失敗）
    assert link._sid_got[0] == (1, 3, 'z', 'gcs')
    assert link._sid_got[1] == (2, 6, 'from-2', 'fc')
    assert link._sid_got[2][0] == 1 and link._sid_got[2][3] == 'gcs' and 'y' in link._sid_got[2][2], \
        'STATUSTEXT 之後的 GCS 訊息要回到 sysid_label（thread-local 有清掉）'
    assert link._status_origin.__dict__.get('v') is None


def test_L3_two_threads_emitting_concurrently_are_never_mistagged():
    link = SITLLink(conn_str='tcp:127.0.0.1:5760', sysid_label=1)
    link._vehicle_srcs.add(2)
    got = []
    link.status_text_sysid.connect(lambda *a: got.append(a), Qt.ConnectionType.DirectConnection)
    n = 200
    barrier = threading.Barrier(2)
    old_iv = sys.getswitchinterval()
    sys.setswitchinterval(1e-6)                         # 盡量讓兩條執行緒交錯
    try:
        def worker():
            barrier.wait()
            for i in range(n):
                link._handle_message(_st(2, f'fc-{i}', sev=6))
        t = threading.Thread(target=worker)
        t.start()
        barrier.wait()
        for i in range(n):
            link.status_text.emit(6, f'gcs-{i}')
        t.join(10)
    finally:
        sys.setswitchinterval(old_iv)
    assert len(got) == 2 * n
    bad = [g for g in got if (g[2].startswith('fc-') and (g[0], g[3]) != (2, 'fc'))
           or (g[2].startswith('gcs-') and (g[0], g[3]) != (1, 'gcs'))]
    assert bad == [], f'{len(bad)} 則標錯，例如 {bad[:3]}'


def test_L4_servo_params_carry_sysid_only_from_the_autopilot_component():
    link = _link(label=1)
    link._handle_message(_pv(3, 'SERVO5_FUNCTION', 33.0, comp=1))
    assert link._servo_sid == [(3, 5, 'FUNCTION', 33.0)]
    assert link._servo_old == [(5, 'FUNCTION', 33.0)]
    link._handle_message(_pv(3, 'SERVO6_FUNCTION', 34.0, comp=100))     # 同 sysid 的相機
    link._handle_message(_pv(255, 'SERVO7_FUNCTION', 35.0, comp=1))     # GCS 回送
    assert link._servo_sid == [(3, 5, 'FUNCTION', 33.0)], '相機／255 不能帶 sysid 進 SERVO 快取'
    assert link._servo_old[1:] == [(6, 'FUNCTION', 34.0), (7, 'FUNCTION', 35.0)], '舊信號照舊不過濾'


def test_L5_non_servo_param_echo_is_tagged_param_with_its_source():
    link = _link(label=1)
    link._handle_message(_pv(2, 'WPNAV_SPEED', 500.0))
    assert link._sid_got == [(2, 6, 'PARAM WPNAV_SPEED = 500', 'param')]
    assert link._old_got == [(6, 'PARAM WPNAV_SPEED = 500')]


def test_L6_minimal_messages_without_source_do_not_crash():
    link = _link(label=7)
    link._handle_message(SimpleNamespace(get_type=lambda: 'STATUSTEXT', severity=5, text='bare'))
    assert link._sid_got == [(0, 5, 'bare', 'ext')], '來源不明 ⇒ 0（無法歸屬）、不算飛控說的'
    link._handle_message(SimpleNamespace(get_type=lambda: 'PARAM_VALUE', param_id='SERVO5_FUNCTION',
                                         param_value=33.0))
    assert link._servo_old == [(5, 'FUNCTION', 33.0)] and link._servo_sid == []


def test_L7_can_fire_removing_the_source_marker_breaks_L1(monkeypatch):
    """自證：把 _emit_status_from 換成「直接 emit」（不放來源）⇒ L1 的標記就錯了 ⇒ L1 的斷言有效。"""
    monkeypatch.setattr(SITLLink, '_emit_status_from',
                        lambda self, src, kind, sev, text: self.status_text.emit(int(sev), text))
    link = _link(label=1)
    link._handle_message(_st(2, 'x', sev=4))
    assert link._sid_got == [(1, 4, 'x', 'gcs')]
    assert link._sid_got != [(2, 4, 'x', 'fc')]


def test_L8_new_without_init_falls_back_to_plain_emit():
    """SITLLink.__new__ 的測試替身（tests/test_vtol_recon_fixes.py）：沒有轉接也不能炸。"""
    lk = SITLLink.__new__(SITLLink)
    got = []
    lk.status_text = SimpleNamespace(emit=lambda *a: got.append(a))
    lk._emit_status_from(2, 'fc', 4, 'x')
    assert got == [(4, 'x')]


def test_L9_statustext_from_non_vehicle_or_non_autopilot_sources_is_ext_not_fc():
    """2026-10-22 審查：MAVROS／伴隨電腦（191/240）以前被標成「UAV191 [FC]」；別的地面站（255）、來源不明（0）
    被當成本機訊息、出現在每一台的單機畫面。只有「已確認飛機（或本鏈路標示的機號）的自駕儀元件」才是 'fc'。"""
    link = _link(label=1, vehicles=(2,))
    link._handle_message(_st(191, 'companion says hi', sev=2, comp=240))   # 非飛機節點
    link._handle_message(_st(255, 'other GCS', sev=4, comp=190))           # 別的地面站
    link._handle_message(_st(3, 'unconfirmed vehicle', sev=6))             # 還沒收到它的心跳
    link._handle_message(_st(2, 'camera on UAV2', sev=6, comp=100))        # 同機的相機元件
    link._handle_message(_st(2, 'fc of UAV2', sev=6, comp=1))              # 已確認的飛機的自駕儀
    link._handle_message(_st(1, 'fc of label', sev=6, comp=1))             # 本鏈路標示的機號
    assert [(g[0], g[3]) for g in link._sid_got] == [
        (191, 'ext'), (255, 'ext'), (3, 'ext'), (2, 'ext'), (2, 'fc'), (1, 'fc')], link._sid_got
    assert len(link._old_got) == 6, '舊 status_text 照舊全部發（不過濾）'
