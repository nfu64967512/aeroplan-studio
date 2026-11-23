# -*- coding: utf-8 -*-
"""tests/test_sitl_hud_mode_wiring.py — main_window 對 SITL 頁「群控／單機」的接線（2026-10-22）。

不建 MainWindow：用未綁定方法跑在假 self 上（house style）。鎖的是：
  W1 on_sitl_connect 只接帶 sysid 的兩條（status_text_sysid／servo_param_sysid）：一則訊息進 HUD 恰好一次，
     舊的 status_text／servo_param 不再進 HUD（兩條都接 ⇒ 每則重複）。另有 AST 鎖。
  W2 條帶警報帶 sysid（kind='strip'）；W3 逐機斷線訊息帶 sysid、假 HUD 仍退回 lbl_status；
  W4 單機模式未選飛機（NO_TARGET）：_confirm_single_target 不跳無意義的對話框、_sitl_broadcast 說清楚原因；
  W5 建構時有呼叫 wire_uav_selection_sync(self)（AST）。
"""
from __future__ import annotations

import ast
import os
import types
from pathlib import Path

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('AEROPLAN_NO_FILE_LOG', '1')

ROOT = Path(__file__).resolve().parents[1]

try:
    import ui.main_window as mw
    from ui.main_window import MainWindow
    from PyQt6.QtCore import QObject, pyqtSignal
    from PyQt6.QtWidgets import QApplication
    _APP = QApplication.instance() or QApplication([])
    import mission.sitl_link as sl
    from mission.fleet_registry import FleetRegistry
    from mission.sitl_link import TelemetryFrame
    from ui.widgets.sitl_hud import SITLHud
    from config import features
    _QT_OK = True
except Exception as _e:                       # pragma: no cover - 環境相依
    _QT_OK = False
    _QT_ERR = _e

qt_only = pytest.mark.skipif(not _QT_OK, reason=f'PyQt6 匯入失敗: {globals().get("_QT_ERR")}')


if _QT_OK:
    class _FakeLink(QObject):
        """on_sitl_connect 會碰到的每一條信號；start() 不做事（不連線、不開埠）。"""
        connected = pyqtSignal(str)
        disconnected = pyqtSignal(str)
        telemetry = pyqtSignal(object)
        status_text = pyqtSignal(int, str)
        status_text_sysid = pyqtSignal(int, int, str, str)
        error = pyqtSignal(str)
        servo_param = pyqtSignal(int, str, float)
        servo_param_sysid = pyqtSignal(int, int, str, float)
        mission_downloaded = pyqtSignal(int, list)
        mission_download_failed = pyqtSignal(int, str)
        mission_uploaded = pyqtSignal(int, bool, str)
        camera_feedback = pyqtSignal(int, dict)
        mission_current = pyqtSignal(int, dict)
        mission_item_started = pyqtSignal(int, int, str)
        wind = pyqtSignal(int, dict)
        log_list = pyqtSignal(int, list)
        log_list_failed = pyqtSignal(int, str)
        log_download_progress = pyqtSignal(int, dict)
        log_download_done = pyqtSignal(int, dict)
        log_download_failed = pyqtSignal(int, str)

        def __init__(self, conn_str='', sysid_label=1, vehicle_hint='', parent=None):
            super().__init__()
            self.conn_str = conn_str
            self.sysid_label = int(sysid_label)
            self.started = False

        def start(self):
            self.started = True


@pytest.fixture
def fresh(monkeypatch):
    monkeypatch.setattr(features, 'STRIKE_ENABLED', False)
    monkeypatch.setattr(FleetRegistry, '_instance', FleetRegistry())
    yield


def _noop(*a, **k):
    return None


def _connect_win(hud):
    names = ['_on_sitl_link_disconnected', '_on_sitl_telemetry', '_on_mission_downloaded',
             '_on_mission_download_failed', '_on_mission_uploaded', '_on_camera_feedback',
             '_on_mapping_mission_current', '_on_mapping_item_started', '_on_wind', '_on_log_list',
             '_on_log_list_failed', '_on_log_download_progress', '_on_log_download_done',
             '_on_log_download_failed', '_update_write_wp_enabled']
    win = types.SimpleNamespace(sitl_hud=hud, _sitl_links=[], _sitl_link=None)
    for n in names:
        setattr(win, n, _noop)
    return win


@qt_only
def test_W1_on_sitl_connect_routes_only_the_sysid_signals_into_the_hud(fresh, monkeypatch):
    monkeypatch.setattr(sl, 'SITLLink', _FakeLink)
    hud = SITLHud()
    win = _connect_win(hud)
    MainWindow.on_sitl_connect(win, 'tcp:x', 2)
    link = win._sitl_links[0]
    assert isinstance(link, _FakeLink) and link.started and link.sysid_label == 2
    before = len(hud._msgs.merged())
    link.status_text_sysid.emit(2, 4, 'm', 'fc')
    after = hud._msgs.merged()
    assert len(after) == before + 1 and (after[-1].sysid, after[-1].severity, after[-1].kind) == (2, 4, 'fc')
    link.status_text.emit(4, 'old-path')
    assert len(hud._msgs.merged()) == before + 1, '舊 status_text 也接進 HUD ⇒ 每則重複'
    link.servo_param_sysid.emit(2, 5, 'FUNCTION', 33.0)
    link.servo_param.emit(5, 'FUNCTION', 99.0)
    assert hud._servo_cache == {2: {(5, 'FUNCTION'): 33.0}}
    assert FleetRegistry.instance().get_link('UAV-2') is link


@qt_only
def test_W1b_ast_lock_on_sitl_connect_wiring():
    src = (ROOT / 'ui' / 'main_window.py').read_text(encoding='utf-8')
    fn = next(n for n in ast.walk(ast.parse(src)) if isinstance(n, ast.FunctionDef) and n.name == 'on_sitl_connect')
    calls = [ast.unparse(n) for n in ast.walk(fn) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Attribute) and n.func.attr == 'connect']
    assert calls.count('link.status_text_sysid.connect(self.sitl_hud.on_status_text_sysid)') == 1, calls
    assert calls.count('link.servo_param_sysid.connect(self.sitl_hud.on_servo_param_sysid)') == 1, calls
    body = ast.unparse(fn)
    assert 'self.sitl_hud.on_status_text)' not in body and 'self.sitl_hud.on_servo_param)' not in body


@qt_only
def test_W2_strip_alarms_reach_the_hud_tagged_with_their_uav():
    hud = SITLHud()
    msgs = []
    win = types.SimpleNamespace(sitl_hud=hud, _refresh_mapping_live_status=_noop,
                                statusBar=lambda: types.SimpleNamespace(showMessage=lambda *a: msgs.append(a)))
    MainWindow._show_strip_alarms(win, 2, [types.SimpleNamespace(severity='critical', message='條帶 3：0 張')])
    e = hud._msgs.merged()[-1]
    assert (e.sysid, e.kind, e.severity, e.text) == (2, 'strip', 2, '條帶 3：0 張')
    assert '[UAV2] [條帶] 條帶 3：0 張' in hud.log_view.toPlainText()
    assert msgs and 'UAV-2' in msgs[-1][0]
    # 舊 HUD（只有 on_status_text）照舊收到
    old = []
    win.sitl_hud = types.SimpleNamespace(on_status_text=lambda sev, t: old.append((sev, t)))
    MainWindow._show_strip_alarms(win, 4, [types.SimpleNamespace(severity='warning', message='x')])
    assert old == [(4, '[條帶] UAV-4 x')]


def _disc_win(hud, links):
    return types.SimpleNamespace(
        sitl_hud=hud, _sitl_links=list(links), _sitl_link=links[0], _write_pending_sysids=set(),
        map_widget=types.SimpleNamespace(), _update_write_wp_enabled=_noop,
        statusBar=lambda: types.SimpleNamespace(showMessage=_noop))


@qt_only
def test_W3_per_uav_disconnect_message_is_tagged_and_fake_huds_still_work(fresh):
    hud = SITLHud()
    for s in (1, 2, 3):
        hud.on_telemetry(TelemetryFrame(sysid=s))
    links = [types.SimpleNamespace(sysid_label=s) for s in (1, 2, 3)]
    win = _disc_win(hud, links)
    MainWindow._on_sitl_link_disconnected(win, links[1], 'timeout')
    e = [x for x in hud._msgs.merged() if '已斷線' in x.text][-1]
    # kind 'link'（全隊警示：單機看哪一台都列）、帶斷線原因（2026-10-22 審查）
    assert e.sysid == 2 and e.kind == 'link' and 'UAV-2 已斷線（timeout；剩餘 2 台連線）' in e.text
    assert 2 not in hud._uav_cards
    # 假 HUD（tests/test_mission_write.py 那種只有 lbl_status）退回 setText
    got = []
    fake = types.SimpleNamespace(lbl_status=types.SimpleNamespace(setText=got.append),
                                 remove_card=_noop, on_disconnected=_noop)
    win2 = _disc_win(fake, [types.SimpleNamespace(sysid_label=5), types.SimpleNamespace(sysid_label=6)])
    MainWindow._on_sitl_link_disconnected(win2, win2._sitl_links[0], 'x')
    assert got and 'UAV-5 已斷線' in got[-1]


@qt_only
def test_W4_no_target_in_single_mode_is_refused_with_a_clear_reason(monkeypatch):
    hud = SITLHud()
    hud.set_view_mode('single')
    assert hud.get_target_sysid() == SITLHud.NO_TARGET
    links = [types.SimpleNamespace(sysid_label=s) for s in (1, 2, 3)]
    win = types.SimpleNamespace(sitl_hud=hud, _sitl_links=links)
    win._target_links = types.MethodType(MainWindow._target_links, win)
    # 不可以跳「UAV-1（單獨）」那種對話框（假 self 建不出 QMessageBox ⇒ 走到就會炸）
    assert MainWindow._confirm_single_target(win, '起飛') is True
    assert win._target_links() == [], 'NO_TARGET 不可以對上任何鏈路'
    shown = []
    monkeypatch.setattr(mw.QMessageBox, 'information', staticmethod(lambda *a, **k: shown.append(a)))
    sent = []
    monkeypatch.setattr(mw, 'send_link_command', lambda link, cmd, arg=None: sent.append(cmd))
    MainWindow._sitl_broadcast(win, 'arm')
    assert sent == [] and shown and '單機模式還沒選定飛機' in shown[-1][2]
    # 選了一台 ⇒ 只送那一台
    hud.on_telemetry(TelemetryFrame(sysid=2))
    assert hud.get_target_sysid() == 2
    MainWindow._sitl_broadcast(win, 'arm')
    assert sent == ['arm']


@qt_only
def test_W5_window_wires_the_selection_sync_once():
    src = (ROOT / 'ui' / 'main_window.py').read_text(encoding='utf-8')
    tree = ast.parse(src)
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MainWindow')
    calls = [ast.unparse(n) for n in ast.walk(cls) if isinstance(n, ast.Call)
             and ast.unparse(n.func) == 'wire_uav_selection_sync']
    assert calls == ['wire_uav_selection_sync(self)'], calls
