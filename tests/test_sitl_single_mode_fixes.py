# -*- coding: utf-8 -*-
"""tests/test_sitl_single_mode_fixes.py — SITL 頁「群控／單機」審查後的修正（2026-10-22）。

每一條都是值層級、拿掉修法就會紅（不建 MainWindow：未綁定方法跑在假 self 上；真 SITLHud、離屏）：
  R1  單機模式的「整批」動作只對選定那台：上傳任務（每架各自路徑時配對照算）、下載全部 .bin、寫入每架匯入；
      依時程放行不送；建圖整批上傳不送。
  R2  單機時別台斷線／別台斷線前的嚴重訊息要看得到；看的那台斷線，它的歷史與斷線那行留在 MSG 區。
  R3  HIL「全部斷開」要清 HUD（機卡、單機停住、SERVO 快取）——同 sysid 換板重連不能顯示上一顆板的值。
  R4  單機模式點圖起飛不再跳「改成全部 UAV」（預設鈕）的對話框。
  R5  rail 點已選的那一格也能把 HUD 單機對過去；群控時 HUD 自動選的那台不帶 rail；進單機時 rail 對齊。
  R6  分頁沒顯示時記著的同步，不會被 rail 的自動改選吃掉。
  R7  單機選定的那台是最後一條連線：一樣停住（不自動接下一台）；使用者按「全部斷線」才解除。
  R8  非飛機自駕儀來源（ext）只進群控合併時間線，不出現在任何一台的單機畫面。
  R9  全部斷線後才送到的舊 SERVO 讀回不能把快取建回來。
  R10 單機版面不比群控寬：SERVO 表最少約 370 px、機號 chip 會換行。
  R11 單機模式 FLIGHT 緊接 HUD（DISARM 在畫面上方）。
  R12 單機區有這台的快捷動作（下載 .bin／斷線／取消＋進度）。
  R13 PARAM 洪水摘要存進緩衝：換台／換模式重畫後還在，別台被壓下的筆數不會被歸零。
"""
from __future__ import annotations

import os
import types

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('AEROPLAN_NO_FILE_LOG', '1')

try:
    import ui.main_window as mw
    from ui.main_window import MainWindow
    from PyQt6.QtCore import QObject, Qt, pyqtSignal
    from PyQt6.QtWidgets import QApplication, QFrame, QScrollArea, QTabWidget
    _APP = QApplication.instance() or QApplication([])
    from mission.fleet_registry import FleetRegistry
    from mission.sitl_link import TelemetryFrame, servo_function_name
    from ui.widgets.sitl_hud import SITLHud
    from ui.widgets.sitl_command_panel import SitlCommandPanel
    from config import features
    _QT_OK = True
except Exception as _e:                       # pragma: no cover - 環境相依
    _QT_OK = False
    _QT_ERR = _e

qt_only = pytest.mark.skipif(not _QT_OK, reason=f'PyQt6 匯入失敗: {globals().get("_QT_ERR")}')


@pytest.fixture(autouse=True)
def fresh(monkeypatch):
    if _QT_OK:
        monkeypatch.setattr(features, 'STRIKE_ENABLED', False)
        monkeypatch.setattr(FleetRegistry, '_instance', FleetRegistry())
    yield


def _noop(*a, **k):
    return None


def _f(sid, **kw):
    kw.setdefault('lat', 23.4 + sid * 0.001)
    kw.setdefault('lon', 120.2)
    kw.setdefault('servo_pwm', [1500] * 16)
    return TelemetryFrame(sysid=sid, mode='AUTO', vehicle_type='PLANE', system_status='ACTIVE', **kw)


def _hud(*sids, mode='swarm', view=None):
    h = SITLHud()
    for s in sids:
        h.on_telemetry(_f(s))
    h.set_view_mode(mode)
    if view is not None:
        h.set_single_sysid(view)
    return h


class _UpLink:
    def __init__(self, sid):
        self.sysid_label = sid
        self.uploaded = None

    def upload_mission(self, wps):
        self.uploaded = list(wps)


def _status_sink(store):
    return lambda: types.SimpleNamespace(showMessage=lambda *a, **k: store.append(a[0]))


def _upload_win(hud, links, *, paths=None, dccpp=None, mapping=False):
    win = types.SimpleNamespace(
        sitl_hud=hud, _sitl_links=links, _diamond_plans=None, _dccpp_result=dccpp,
        map_widget=types.SimpleNamespace(paths=paths or []),
        parameter_panel=types.SimpleNamespace(get_parameters=lambda: {'altitude': 50}),
        flight_params={}, _status=[])
    win.statusBar = _status_sink(win._status)
    win._hv_mapping_upload_applies = lambda: mapping
    return win


def _bp(lat):
    """DCCPP BuiltPath 的最小替身：兩個巡航航點；第一點緯度標出是哪一架的路徑。"""
    seg = types.SimpleNamespace(name='CRUISE')
    return types.SimpleNamespace(waypoints=[
        types.SimpleNamespace(lat=lat, lon=120.2, alt=80.0, segment_type=seg, heading_compass_deg=0.0),
        types.SimpleNamespace(lat=lat + 0.01, lon=120.21, alt=80.0, segment_type=seg, heading_compass_deg=0.0)])


# ══════════════════════════════════════════════════════════════════════
# R1 單機模式的整批動作
# ══════════════════════════════════════════════════════════════════════
@qt_only
def test_R1a_upload_in_single_mode_writes_only_the_selected_aircraft():
    hud = _hud(1, 2, 3, mode='single', view=2)
    links = [_UpLink(s) for s in (1, 2, 3)]
    win = _upload_win(hud, links, paths=[[(23.4, 120.2, 50.0), (23.41, 120.21, 50.0)]])
    MainWindow.on_sitl_upload_mission(win)
    assert [l.sysid_label for l in links if l.uploaded] == [2], '單機模式把任務寫進了別台'
    assert win._status and 'UAV2' in win._status[-1] and '單機' in win._status[-1]
    # 對照：群控照舊寫每一台
    hud.set_view_mode('swarm')
    links2 = [_UpLink(s) for s in (1, 2, 3)]
    win2 = _upload_win(hud, links2, paths=[[(23.4, 120.2, 50.0)]])
    MainWindow.on_sitl_upload_mission(win2)
    assert all(l.uploaded for l in links2)


@qt_only
def test_R1b_per_uav_paths_keep_the_fleet_pairing_in_single_mode():
    """sysid 5/6/7 沒有對應的 uav_id 1/2/3 ⇒ 依連線順序配對（5→1、6→2、7→3）。單機選 6 要拿到**機 2** 的路徑；
    若只拿「選定那條連線」重新配對，它會拿到機 1 的（錯的那份）。"""
    hud = _hud(5, 6, 7, mode='single', view=6)
    links = [_UpLink(s) for s in (5, 6, 7)]
    dccpp = {'assembled_paths': {1: _bp(21.0), 2: _bp(22.0), 3: _bp(23.0)}}
    win = _upload_win(hud, links, dccpp=dccpp)
    MainWindow.on_sitl_upload_mission(win)
    assert [l.sysid_label for l in links if l.uploaded] == [6]
    first_nav = next(it for it in links[1].uploaded if it[3] != 178)     # 跳過 DO_CHANGE_SPEED
    assert abs(first_nav[0] - 22.0) < 1e-9, f'UAV6 拿錯路徑：{first_nav}'


@qt_only
def test_R1c_no_target_or_whole_fleet_plans_are_refused_in_single_mode(monkeypatch):
    shown = []
    monkeypatch.setattr(mw.QMessageBox, 'information', staticmethod(lambda *a, **k: shown.append(a)))
    empty = SITLHud()
    empty.set_view_mode('single')                               # 還沒選（NO_TARGET）
    links = [_UpLink(s) for s in (1, 2)]
    MainWindow.on_sitl_upload_mission(_upload_win(empty, links, paths=[[(23.4, 120.2, 50.0)]]))
    assert not any(l.uploaded for l in links) and '還沒選定' in shown[-1][2]
    hud = _hud(1, 2, mode='single', view=1)
    called = []
    win = _upload_win(hud, links, paths=[[(23.4, 120.2, 50.0)]], mapping=True)
    win._on_hv_mapping_upload = lambda: called.append('mapping')
    MainWindow.on_sitl_upload_mission(win)
    assert called == [] and not any(l.uploaded for l in links), '建圖整批上傳在單機模式被送出了'
    assert '整批' in shown[-1][1] or '整批' in shown[-1][2]


@qt_only
def test_R1d_release_by_schedule_is_not_sent_in_single_mode(monkeypatch):
    shown = []
    monkeypatch.setattr(mw.QMessageBox, 'information', staticmethod(lambda *a, **k: shown.append(a)))
    monkeypatch.setattr(mw.QMessageBox, 'question', staticmethod(lambda *a, **k: pytest.fail('不該問要不要放行')))
    hud = _hud(1, 2, mode='single', view=1)
    sched = {'entries': {1: {'sysid': 1, 'uav': 1, 'offset_s': 0.0}, 2: {'sysid': 2, 'uav': 2, 'offset_s': 30.0}},
             'upload_ok': {}}
    win = types.SimpleNamespace(sitl_hud=hud, _hv_release_schedule=sched, _sitl_links=[], _release=None)
    MainWindow.on_release_by_schedule(win)
    assert shown and '單機模式不送' in shown[-1][1] and win._release is None
    # 對照：群控會往下走（這裡沒有連線 ⇒ 被「沒有可以放行的飛機」擋下，不是單機的拒絕）
    hud.set_view_mode('swarm')
    crit = []
    monkeypatch.setattr(mw.QMessageBox, 'critical', staticmethod(lambda *a, **k: crit.append(a)))
    MainWindow.on_release_by_schedule(win)
    assert crit and '單機' not in crit[-1][1]


@qt_only
def test_R1e_download_all_only_downloads_the_selected_aircraft_in_single_mode():
    hud = _hud(1, 2, 3, mode='single', view=3)
    links = [types.SimpleNamespace(sysid_label=s) for s in (1, 2, 3)]
    started, status = [], []
    win = types.SimpleNamespace(sitl_hud=hud, _sitl_links=links, statusBar=_status_sink(status))
    win._link_for_sysid = types.MethodType(MainWindow._link_for_sysid, win)
    win._start_log_download = lambda sid: started.append(sid) or True
    MainWindow._start_log_download_all(win)
    assert started == [3] and 'UAV3' in status[-1]
    hud.set_view_mode('swarm')
    started.clear()
    MainWindow._start_log_download_all(win)
    assert started == [1, 2, 3]


@qt_only
def test_R1f_write_imported_only_writes_the_selected_aircrafts_file(monkeypatch):
    shown = []
    monkeypatch.setattr(mw.QMessageBox, 'information', staticmethod(lambda *a, **k: shown.append(a)))
    hud = _hud(1, 2, 3, 4, mode='single', view=2)
    links = [types.SimpleNamespace(sysid_label=s, conn_str=f'tcp:{s}') for s in (1, 2, 3, 4)]
    imported = {s: types.SimpleNamespace(loaded=f'mission-{s}') for s in (1, 2, 3)}
    writes = []
    win = types.SimpleNamespace(sitl_hud=hud, _sitl_links=links, _imported_missions=imported)
    win.on_sitl_write_wp = lambda target_links=None, mission=None: writes.append(
        ([l.sysid_label for l in target_links], mission))
    MainWindow.on_sitl_write_imported(win)
    assert writes == [([2], 'mission-2')], writes
    hud.set_single_sysid(4)                                     # UAV4 沒有匯入檔 ⇒ 什麼都不寫、說清楚
    writes.clear()
    MainWindow.on_sitl_write_imported(win)
    assert writes == [] and 'UAV4' in shown[-1][2]
    hud.set_view_mode('swarm')
    MainWindow.on_sitl_write_imported(win)
    assert [w[0] for w in writes] == [[1], [2], [3]]


@qt_only
def test_R1g_single_mode_tooltip_names_the_fleet_level_exception():
    tip = SITLHud().btn_mode_single.toolTip()
    assert '依時程放行' in tip and '所有指令只送給這一台' not in tip


# ══════════════════════════════════════════════════════════════════════
# R2 單機的斷線可見度
# ══════════════════════════════════════════════════════════════════════
def _disc_win(hud, sids):
    links = [types.SimpleNamespace(sysid_label=s) for s in sids]
    return types.SimpleNamespace(
        sitl_hud=hud, _sitl_links=list(links), _sitl_link=links[0], _write_pending_sysids=set(),
        map_widget=types.SimpleNamespace(), _update_write_wp_enabled=_noop,
        statusBar=lambda: types.SimpleNamespace(showMessage=_noop)), links


@qt_only
def test_R2a_other_aircraft_critical_then_link_loss_is_visible_in_single_view():
    hud = _hud(1, 2, 3, mode='single', view=1)
    hud.on_status_text_sysid(2, 2, 'EKF variance CRITICAL', 'fc')
    assert hud._single_chips[2].text() == 'UAV2 ●1'
    win, links = _disc_win(hud, (1, 2, 3))
    MainWindow._on_sitl_link_disconnected(win, links[1], 'heartbeat timeout')
    t = hud.single_log_view.toPlainText()
    assert 'UAV2 斷線前有 1 則未讀的嚴重訊息' in t and 'EKF variance CRITICAL' in t, t
    assert '[UAV2] [LINK] 🔌 UAV-2 已斷線（heartbeat timeout；剩餘 2 台連線）' in t, t
    assert hud.single_sysid() == 1 and hud.get_target_sysid() == 1, '別台斷線不影響看的這台'
    # 換看 UAV3：UAV2 的斷線（全隊警示）仍列，但 UAV2 的一般訊息不列
    hud.set_single_sysid(3)
    t3 = hud.single_log_view.toPlainText()
    assert '🔌 UAV-2 已斷線' in t3 and 'EKF variance CRITICAL（' not in t3.replace('最後一則：', '')


@qt_only
def test_R2b_viewed_aircraft_keeps_its_history_and_disconnect_line_after_it_drops():
    hud = _hud(1, 2, mode='single', view=2)
    for i in range(5):
        hud.on_status_text_sysid(2, 6, f'u2-fc-{i}', 'fc')
    hud.on_status_text_sysid(2, 2, 'EKF failsafe', 'fc')
    hud.on_status_text_sysid(1, 6, 'u1-only', 'fc')
    win, links = _disc_win(hud, (1, 2))
    MainWindow._on_sitl_link_disconnected(win, links[1], '連線逾時')
    t = hud.single_log_view.toPlainText()
    assert all(f'u2-fc-{i}' in t for i in range(5)) and 'EKF failsafe' in t, t
    assert '🔌 UAV-2 已斷線（連線逾時；剩餘 1 台連線）' in t and '[UAV2]' not in t, '自己的斷線不用再標 UAVn'
    assert 'u1-only' not in t
    assert hud.single_sysid() is None and hud.get_target_sysid() == SITLHud.NO_TARGET
    assert hud.lbl_cmd_target.text() == 'UAV2 已斷線（單機模式）'


# ══════════════════════════════════════════════════════════════════════
# R3 HIL 全部斷開
# ══════════════════════════════════════════════════════════════════════
if _QT_OK:
    class _HilLink(QObject):
        disconnected = pyqtSignal(str)

        def __init__(self, sid):
            super().__init__()
            self.sysid_label = sid
            self.stopped = False

        def stop(self):
            self.stopped = True

        def wait(self, _ms):
            return True


def _fill_servo(hud, sid, func=33.0):
    for ch in range(1, 17):
        for key in ('FUNCTION', 'MIN', 'TRIM', 'MAX', 'REVERSED'):
            hud.on_servo_param_sysid(sid, ch, key, func if key == 'FUNCTION' else 0.0 if key == 'REVERSED' else 1500.0)


@qt_only
def test_R3_hil_disconnect_all_clears_the_hud_and_a_same_sysid_reconnect_shows_only_new_values():
    hud = _hud(1, 2, mode='single', view=2)
    _fill_servo(hud, 2)
    assert len(hud._servo_cache[2]) == 80
    l1, l2 = _HilLink(1), _HilLink(2)
    cleared = []
    win = types.SimpleNamespace(
        sitl_hud=hud, _sitl_links=[l1, l2], _hil_links=[l2], _sitl_link=l1, _write_pending_sysids=set(),
        hil_panel=types.SimpleNamespace(on_disconnected=_noop), _update_write_wp_enabled=_noop,
        map_widget=types.SimpleNamespace(clear_uav=cleared.append),
        statusBar=lambda: types.SimpleNamespace(showMessage=_noop))
    MainWindow._on_hil_disconnect_all(win)
    _APP.processEvents()
    assert l2.stopped and 2 not in hud._uav_cards and cleared == [2]
    assert 2 not in hud._servo_cache, 'HIL 斷開後上一顆板的 SERVO 讀回還留著'
    assert hud.single_sysid() is None and hud.get_target_sysid() == SITLHud.NO_TARGET
    assert '🔌 UAV-2 已斷線（HIL 全部斷開' in hud.single_log_view.toPlainText()
    # 同 sysid 換一顆板重連：只讀回 3 筆
    hud.on_telemetry(_f(2))
    for ch, func in ((1, 4.0), (2, 19.0), (3, 70.0)):
        hud.on_servo_param_sysid(2, ch, 'FUNCTION', func)
    hud._single_chips[2].click()
    title = hud.servo_panel.lbl_title.text()
    assert '3/80' in title, title
    assert hud.servo_panel._rows[4].lbl_func.text() == '—', '上一顆板的 Motor1 還顯示著'
    assert hud.servo_panel._rows[0].lbl_func.text() == servo_function_name(4)


@qt_only
def test_R3b_hil_disconnect_of_the_only_links_restores_the_disconnected_state():
    hud = _hud(4, mode='single', view=4)
    l4 = _HilLink(4)
    win = types.SimpleNamespace(
        sitl_hud=hud, _sitl_links=[l4], _hil_links=[l4], _sitl_link=l4, _write_pending_sysids=set(),
        hil_panel=types.SimpleNamespace(on_disconnected=_noop), _update_write_wp_enabled=_noop,
        map_widget=types.SimpleNamespace(), statusBar=lambda: types.SimpleNamespace(showMessage=_noop))
    MainWindow._on_hil_disconnect_all(win)
    assert hud._uav_cards == {} and hud.btn_connect.text() == '連線'
    assert hud._single_hold is False, '使用者自己按的全部斷開 ⇒ 之後重連是新的一輪'
    hud.on_telemetry(_f(5))
    assert hud.get_target_sysid() == 5


# ══════════════════════════════════════════════════════════════════════
# R4 單機模式點圖起飛不跳「改成全部 UAV」
# ══════════════════════════════════════════════════════════════════════
if _QT_OK:
    class _Box:
        made = []

        class Icon:
            Warning = 1

        class ButtonRole:
            AcceptRole, DestructiveRole, RejectRole = 0, 1, 2

        def __init__(self, parent=None):
            _Box.made.append(self)
            self.buttons, self.default = [], None

        def setIcon(self, *_):
            pass

        def setWindowTitle(self, *_):
            pass

        def setText(self, t):
            self.text = t

        def addButton(self, text, role):
            b = types.SimpleNamespace(text=text, role=role)
            self.buttons.append(b)
            return b

        def setDefaultButton(self, b):
            self.default = b

        def exec(self):
            return 0

        def clickedButton(self):
            return self.default                    # 使用者直接按 Enter


@qt_only
def test_R4_confirm_single_target_does_not_offer_all_uavs_in_single_mode(monkeypatch):
    monkeypatch.setattr(mw, 'QMessageBox', _Box)
    _Box.made.clear()
    hud = _hud(1, 2, 3, mode='single', view=2)
    win = types.SimpleNamespace(sitl_hud=hud, _sitl_links=[types.SimpleNamespace(sysid_label=s) for s in (1, 2, 3)])
    assert MainWindow._confirm_single_target(win, '點圖起飛') is True
    assert _Box.made == [], '單機模式還跳出「改成全部 UAV」對話框'
    assert hud.view_mode() == 'single' and hud.get_target_sysid() == 2
    # 對照：群控點了機卡（09-02 情境）照舊問，Enter 會改成全部
    hud.set_view_mode('swarm')
    hud._uav_cards[2].selected.emit(2)
    assert MainWindow._confirm_single_target(win, '點圖起飛') is True
    assert len(_Box.made) == 1 and hud.get_target_sysid() is None


# ══════════════════════════════════════════════════════════════════════
# R5／R6 Command 分頁 rail 同步
# ══════════════════════════════════════════════════════════════════════
if _QT_OK:
    class _RegLink(QObject):
        telemetry = pyqtSignal(object)
        connected = pyqtSignal(str)
        disconnected = pyqtSignal(str)
        status_text = pyqtSignal(int, str)

        def __init__(self, sid):
            super().__init__()
            self.sysid_label = sid


def _rail_env(sids, *, visible=True, telemetry=None):
    reg = FleetRegistry.instance()
    for s in sids:
        reg.register(f'UAV-{s}', _RegLink(s), s)
    panel = SitlCommandPanel()
    panel.setAttribute(Qt.WidgetAttribute.WA_DontShowOnScreen, True)
    if visible:
        panel.show()
    hud = SITLHud()
    win = types.SimpleNamespace(sitl_hud=hud, sitl_command_panel=panel)
    mw.wire_uav_selection_sync(win)
    for s in (telemetry if telemetry is not None else sids):
        hud.on_telemetry(_f(s))
    return reg, panel, hud, win


@qt_only
def test_R5a_clicking_the_already_current_rail_cell_resyncs_a_held_single_view():
    reg, panel, hud, win = _rail_env((2, 3))
    hud.set_view_mode('single')
    hud.set_single_sysid(2)
    assert panel.rail.current_callsign() == 'UAV-2'
    reg.unregister('UAV-2')                        # main_window：先反註冊（rail 自動改選 UAV-3）、再 remove_card
    hud.remove_card(2)
    assert panel.rail.current_callsign() == 'UAV-3' and hud.get_target_sysid() == SITLHud.NO_TARGET
    panel.rail._cells['UAV-3'].clicked.emit('UAV-3')     # 使用者點 rail 上已選的那一格
    assert hud.single_sysid() == 3 and hud.get_target_sysid() == 3
    panel.hide()


@qt_only
def test_R5b_swarm_auto_adopt_does_not_drive_the_rail_but_entering_single_aligns_it():
    reg, panel, hud, win = _rail_env((1, 3), telemetry=(3,))
    assert panel.rail.current_callsign() == 'UAV-1'          # rail 自己自動選第一台註冊的
    assert hud.single_sysid() == 3 and hud.single_change_source() == 'auto'
    assert panel.rail.current_callsign() == 'UAV-1', '群控時 HUD 自動選的那台不該帶著 rail 換台'
    hud.set_view_mode('single')                               # 使用者明確進單機 ⇒ 對齊
    assert panel.rail.current_callsign() == 'UAV-3'
    panel.hide()


@qt_only
def test_R6_pending_sync_survives_a_rail_auto_repick_and_applies_on_show():
    reg, panel, hud, win = _rail_env((1, 2, 3), visible=False)
    hud.set_view_mode('single')
    hud._single_chips[3].click()
    assert win.__dict__.get('_uav_sync_pending') == 3
    reg.unregister('UAV-1')                                    # rail 自動改選（UAV-1 原本是它的目前那格）
    hud.remove_card(1)
    assert win.__dict__.get('_uav_sync_pending') == 3, '自動改選把記著的同步吃掉了'
    panel.show()
    _APP.processEvents()
    assert panel.rail.current_callsign() == 'UAV-3' and hud.get_target_sysid() == 3
    panel.hide()


# ══════════════════════════════════════════════════════════════════════
# R7 最後一條連線
# ══════════════════════════════════════════════════════════════════════
@qt_only
def test_R7a_selected_aircraft_as_the_last_link_still_holds():
    hud = _hud(1, 2, mode='single', view=2)
    hud.remove_card(1)
    hud.on_disconnected('連線中斷')                           # 選定那台（UAV2）是最後一條
    assert hud._single_hold is True and hud._single_lost_sysid == 2
    hud.on_telemetry(_f(1))
    assert hud.get_target_sysid() == SITLHud.NO_TARGET, '下一台有遙測的被靜默當成單機指令對象'
    assert 'UAV2 已斷線' in hud.lbl_single_empty.text()


@qt_only
def test_R7b_user_disconnect_all_releases_the_hold():
    hud = _hud(1, 2, mode='single', view=2)
    links = []
    for s in (1, 2):
        lk = types.SimpleNamespace(sysid_label=s, stop=_noop, wait=_noop,
                                   disconnected=types.SimpleNamespace(disconnect=_noop))
        links.append(lk)
    win = types.SimpleNamespace(sitl_hud=hud, _sitl_links=links, _sitl_link=links[0], _hil_links=[],
                                hil_panel=types.SimpleNamespace(on_disconnected=_noop),
                                map_widget=types.SimpleNamespace(clear_uav=_noop),
                                _write_pending_sysids=set(), _update_write_wp_enabled=_noop)
    MainWindow.on_sitl_disconnect(win)
    assert hud._single_hold is False
    hud.on_telemetry(_f(1))
    assert hud.get_target_sysid() == 1, '使用者自己全部斷線後重連是新的一輪：第一台自動選'


# ══════════════════════════════════════════════════════════════════════
# R8 ext（非飛機自駕儀來源）
# ══════════════════════════════════════════════════════════════════════
@qt_only
def test_R8_ext_lines_only_appear_in_the_swarm_timeline():
    hud = _hud(1, 2)
    hud.on_status_text_sysid(191, 2, 'companion says hi', 'ext')
    hud.on_status_text_sysid(0, 4, 'from unknown source', 'ext')
    t = hud.log_view.toPlainText()
    assert '[EXT 191] companion says hi' in t and '[EXT] from unknown source' in t and '[UAV191]' not in t
    hud.set_view_mode('single')
    for s in (1, 2):
        hud.set_single_sysid(s)
        ts = hud.single_log_view.toPlainText()
        assert 'companion' not in ts and 'unknown source' not in ts, f'UAV{s} 的單機畫面出現了非飛機來源的訊息'
    hud.on_status_text_sysid(191, 2, 'live ext', 'ext')
    assert 'live ext' not in hud.single_log_view.toPlainText() and 191 not in hud._unread_crit
    assert all(e.kind != 'ext' for e in hud._msgs.for_uav(1)) and 191 not in hud._msgs.sysids()


# ══════════════════════════════════════════════════════════════════════
# R9 全部斷線後才送到的舊 SERVO 讀回
# ══════════════════════════════════════════════════════════════════════
if _QT_OK:
    class _ServoSrc(QObject):
        servo = pyqtSignal(int, int, str, float)


@qt_only
def test_R9_queued_servo_params_after_clear_do_not_resurrect_the_cache(monkeypatch):
    hud = _hud(2)
    src = _ServoSrc()
    src.servo.connect(hud.on_servo_param_sysid, Qt.ConnectionType.QueuedConnection)
    src.servo.emit(2, 5, 'FUNCTION', 33.0)            # 接收執行緒結束前排進佇列的那一筆
    hud.on_disconnected('使用者中斷')                  # main_window：link.stop(); wait(); 然後同步清空
    src.servo.emit(4, 1, 'FUNCTION', 4.0)             # 清空之後才連上的另一台（不能被誤清）
    _APP.processEvents()
    assert 2 not in hud._servo_cache, '舊鏈路的 SERVO 讀回在清空之後把快取建回來了'
    assert hud._servo_cache.get(4) == {(1, 'FUNCTION'): 4.0}


@qt_only
def test_R9b_can_fire_without_the_deferred_purge_the_cache_comes_back(monkeypatch):
    monkeypatch.setattr(SITLHud, '_schedule_servo_purge', lambda self, sids: None)
    hud = _hud(2)
    src = _ServoSrc()
    src.servo.connect(hud.on_servo_param_sysid, Qt.ConnectionType.QueuedConnection)
    src.servo.emit(2, 5, 'FUNCTION', 33.0)
    hud.on_disconnected('使用者中斷')
    _APP.processEvents()
    assert hud._servo_cache.get(2) == {(5, 'FUNCTION'): 33.0}


# ══════════════════════════════════════════════════════════════════════
# R10／R11 版面
# ══════════════════════════════════════════════════════════════════════
@qt_only
def test_R10_single_layout_is_not_wider_than_swarm_and_its_parts_shrink():
    hud = _hud(101, 102, 103, 104, 105, 106)
    swarm_min = hud.minimumSizeHint().width()
    hud.set_view_mode('single')
    for s in (102, 103):
        for _ in range(12):
            hud.on_status_text_sysid(s, 2, 'crit', 'fc')
    assert hud._single_chips[102].text() == 'UAV102 ●12'
    assert hud.minimumSizeHint().width() <= swarm_min, '單機模式比群控還寬 ⇒ 一切到單機就冒水平捲軸'
    # SERVO 表：彈性 PWM 欄 ⇒ 最少約 370 px（以前 424 px，422 px 面板扣掉捲軸只剩 404）
    assert hud.servo_panel.minimumSizeHint().width() <= 380, hud.servo_panel.minimumSizeHint().width()
    # 機號 chip 會換行：最小寬 ＝ 最寬那顆，不是加總
    chips = list(hud._single_chips.values())
    host = chips[0].parentWidget()
    assert host.minimumSizeHint().width() < sum(c.sizeHint().width() for c in chips) / 2
    assert host.heightForWidth(150) > chips[0].sizeHint().height(), '窄的時候要換行'


@qt_only
def test_R10b_header_and_rows_stay_aligned_when_the_bar_column_stretches():
    hud = _hud(1, mode='single', view=1)
    hud.setAttribute(Qt.WidgetAttribute.WA_DontShowOnScreen, True)
    for w in (400, 620):
        hud.resize(w, 2000)
        hud.show()
        _APP.processEvents()
        p = hud.servo_panel
        head = p.layout().itemAt(1).widget().layout()
        hx = [head.itemAt(i).widget().geometry() for i in range(7)]
        row = p._rows[0]
        rx = [row.lbl_ch.geometry(), row.bar.geometry(), row.lbl_rev.geometry(), row.lbl_func.geometry(),
              row.sb_min.geometry(), row.sb_trim.geometry(), row.sb_max.geometry()]
        assert [(g.x(), g.width()) for g in hx] == [(g.x(), g.width()) for g in rx], (w, hx, rx)
    assert p._rows[0].bar.width() > 100, '寬的面板 PWM 條要跟著變寬'
    hud.hide()


def _scroll_page(hud, w=480, h=900):
    tabs = QTabWidget()
    tabs.setAttribute(Qt.WidgetAttribute.WA_DontShowOnScreen, True)
    sc = QScrollArea()
    sc.setWidgetResizable(True)
    sc.setFrameShape(QFrame.Shape.NoFrame)
    sc.setWidget(hud)
    tabs.addTab(sc, 'HUD / Servo')
    tabs.resize(w, h)
    tabs.show()
    for _ in range(3):
        _APP.processEvents()
    return tabs


@qt_only
def test_R11_flight_controls_stay_near_the_top_in_single_mode():
    hud = _hud(1, 2, 3)
    tabs = _scroll_page(hud)
    y_swarm = hud.btn_disarm.mapTo(hud, hud.btn_disarm.rect().topLeft()).y()
    hud.set_view_mode('single')                                # 第一次進單機：SERVO 自動展開
    for _ in range(3):
        _APP.processEvents()
    assert not hud.servo_panel.isHidden()
    y_single = hud.btn_disarm.mapTo(hud, hud.btn_disarm.rect().topLeft()).y()
    assert y_swarm < 700 and y_single < 700, (y_swarm, y_single)
    root = hud.layout()
    flight = hud.btn_disarm
    while flight.parentWidget() is not hud:
        flight = flight.parentWidget()
    order = [root.indexOf(w) for w in (hud._single_frame, flight, hud._single_msg_frame, hud.servo_panel,
                                       hud._single_detail_frame)]
    assert -1 not in order and order == sorted(order), order
    tabs.hide()


# ══════════════════════════════════════════════════════════════════════
# R12 單機區的快捷動作
# ══════════════════════════════════════════════════════════════════════
@qt_only
def test_R12_single_panel_actions_target_the_selected_aircraft_and_mirror_its_log_bar():
    hud = _hud(2, 3, mode='single', view=2)
    got = {'dl': [], 'disc': [], 'cancel': []}
    hud.cmd_log_download.connect(got['dl'].append)
    hud.uav_disconnect_requested.connect(got['disc'].append)
    hud.cmd_log_cancel.connect(got['cancel'].append)
    assert not hud._single_actions.isHidden() and hud.single_log_bar.isHidden()
    hud.btn_single_logdl.click()
    hud.btn_single_disconnect.click()
    assert got['dl'] == [2] and got['disc'] == [2]
    hud.on_log_progress(3, {'id': 7, 'size': 1048576, 'received': 0, 'bytes_per_s': 0})
    assert hud.single_log_bar.isHidden(), '別台的下載進度不該出現在這台的單機區'
    hud.on_log_progress(2, {'id': 9, 'size': 1048576, 'received': 524288, 'bytes_per_s': 8192, 'eta_s': 64})
    assert not hud.single_log_bar.isHidden() and '#9' in hud.lbl_single_log.text()
    assert hud.single_log_progress.value() == 500 and not hud.btn_single_log_cancel.isHidden()
    hud.btn_single_log_cancel.click()
    assert got['cancel'] == [2]
    hud.set_single_sysid(3)                                     # 換台 ⇒ 換成那台的下載列
    assert '#7' in hud.lbl_single_log.text()
    hud.on_log_done(3, {'path': 'runs/flight_logs/x/UAV-3_log7.bin', 'size': 1048576})
    assert '✔' in hud.lbl_single_log.text() and hud.btn_single_log_cancel.isHidden()


# ══════════════════════════════════════════════════════════════════════
# R13 PARAM 洪水摘要
# ══════════════════════════════════════════════════════════════════════
@qt_only
def test_R13_param_summaries_survive_rerenders_and_other_uavs_counts_are_not_zeroed():
    hud = _hud(1, 2)
    for i in range(100):
        hud.on_status_text_sysid(1, 6, f'PARAM P{i} = 1', 'param')
    hud._flush_param_summaries()
    assert '[UAV1] （參數回報 90 筆未逐筆列出）' in hud.log_view.toPlainText()
    hud.set_view_mode('single')
    hud.set_view_mode('swarm')                                  # 重畫
    assert '（參數回報 90 筆未逐筆列出）' in hud.log_view.toPlainText(), '重畫後摘要不見了'
    # 單機看 UAV1 時 UAV2 灌了一批：換台（重畫）不能把 UAV2 被壓下的筆數歸零
    hud.set_view_mode('single')
    hud.set_single_sysid(1)
    for i in range(100):
        hud.on_status_text_sysid(2, 6, f'PARAM Q{i} = 1', 'param')
    hud.set_single_sysid(2)                                     # 重畫（計時器還沒跑）
    assert '（參數回報 90 筆未逐筆列出）' in hud.single_log_view.toPlainText()
