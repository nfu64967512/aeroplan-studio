# -*- coding: utf-8 -*-
"""tests/test_sitl_uav_selection_sync.py — HUD 單機選台 ⇄ Command 分頁機卡列（2026-10-22）。

main_window.wire_uav_selection_sync（模組層級函式）把 SITLHud.single_uav_changed 與
SitlCommandPanel.current_uav_changed 接成雙向同步。鎖的是：
  S1 HUD 換台 ⇒ rail 跟著，各發一次、不回彈；S2 rail 被點 ⇒ HUD 換看，但群控的指令對象不動；
  S3 rail 選到沒遙測（沒機卡）的 callsign ⇒ HUD 不理；S4 單機 chip ⇒ rail 跟著；S5 自訂 callsign（HAWK-01）；
  S6 rail 的「自動改選」（選中那台斷線）不能讓單機模式靜默換對象；
  S7 Command 分頁沒顯示時先記著、顯示出來才套用（rail 換選會讓詳情面板向飛控要全部參數）。

真 SITLHud、真 SitlCommandPanel；FleetRegistry 換成新的實例、鏈路用假 QObject（不連線、不開埠）。
"""
from __future__ import annotations

import os
import types

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('AEROPLAN_NO_FILE_LOG', '1')

try:
    import ui.main_window as mw
    from PyQt6.QtCore import QObject, Qt, pyqtSignal
    from PyQt6.QtWidgets import QApplication
    _APP = QApplication.instance() or QApplication([])
    from mission.fleet_registry import FleetRegistry
    from mission.sitl_link import TelemetryFrame
    from ui.widgets.sitl_hud import SITLHud
    from ui.widgets.sitl_command_panel import SitlCommandPanel
    from config import features
    _QT_OK = True
except Exception as _e:                       # pragma: no cover - 環境相依
    _QT_OK = False
    _QT_ERR = _e

qt_only = pytest.mark.skipif(not _QT_OK, reason=f'PyQt6 匯入失敗: {globals().get("_QT_ERR")}')


if _QT_OK:
    class _Link(QObject):
        telemetry = pyqtSignal(object)
        connected = pyqtSignal(str)
        disconnected = pyqtSignal(str)
        status_text = pyqtSignal(int, str)             # FlightLogsPanel（Command 分頁的 Flights 頁）會接

        def __init__(self, sysid):
            super().__init__()
            self.sysid_label = sysid


@pytest.fixture
def env(monkeypatch):
    monkeypatch.setattr(features, 'STRIKE_ENABLED', False)
    monkeypatch.setattr(FleetRegistry, '_instance', FleetRegistry())
    reg = FleetRegistry.instance()
    links = {}

    def register(cs, sid):
        links[cs] = _Link(sid)
        reg.register(cs, links[cs], sid)

    for s in (1, 2, 3):
        register(f'UAV-{s}', s)
    panel = SitlCommandPanel()
    panel.setAttribute(Qt.WidgetAttribute.WA_DontShowOnScreen, True)
    panel.show()                                   # Command 分頁「看得到」（S7 另測沒顯示的情況）
    hud = SITLHud()
    win = types.SimpleNamespace(sitl_hud=hud, sitl_command_panel=panel)
    mw.wire_uav_selection_sync(win)
    for s in (1, 2, 3):
        hud.on_telemetry(TelemetryFrame(sysid=s))
    ev = {'hud': [], 'rail': []}
    hud.single_uav_changed.connect(ev['hud'].append)
    panel.rail.current_uav_changed.connect(ev['rail'].append)
    yield types.SimpleNamespace(reg=reg, links=links, register=register, panel=panel, rail=panel.rail,
                                hud=hud, win=win, ev=ev)
    panel.hide()


@qt_only
def test_S1_card_click_moves_the_rail_once_without_echo(env):
    assert env.hud.single_sysid() == 1 and env.rail.current_callsign() == 'UAV-1'
    env.hud._uav_cards[2].selected.emit(2)
    _APP.processEvents()
    assert env.rail.current_callsign() == 'UAV-2'
    assert env.ev == {'hud': [2], 'rail': ['UAV-2']}, env.ev
    assert env.hud.get_target_sysid() == 2


@qt_only
def test_S2_rail_click_changes_the_view_but_not_the_swarm_command_target(env):
    env.hud._uav_cards[2].selected.emit(2)                    # 群控指令對象 = 2
    env.ev['hud'].clear(); env.ev['rail'].clear()
    env.rail.set_current('UAV-3')
    _APP.processEvents()
    assert env.hud.single_sysid() == 3 and env.ev == {'hud': [3], 'rail': ['UAV-3']}
    assert env.hud._selected_sysid == 2 and env.hud.get_target_sysid() == 2, '群控的指令對象不能被 rail 改掉'
    assert not any('來自 Command 分頁' in e.text for e in env.hud._msgs.merged()), '群控時不必提示'
    env.hud.set_view_mode('single')
    env.rail.set_current('UAV-1')
    assert env.hud.get_target_sysid() == 1
    assert 'UAV1（來自 Command 分頁）' in env.hud.single_log_view.toPlainText(), '單機時對象從別頁改要留一行'


@qt_only
def test_S3_rail_callsign_without_telemetry_is_ignored(env):
    env.register('UAV-9', 9)
    env.ev['hud'].clear(); env.ev['rail'].clear()
    env.rail.set_current('UAV-9')
    assert env.hud.single_sysid() == 1 and env.ev['hud'] == []


@qt_only
def test_S4_single_mode_chip_moves_the_rail(env):
    env.hud.set_view_mode('single')
    env.ev['hud'].clear(); env.ev['rail'].clear()
    env.hud._single_chips[3].click()
    _APP.processEvents()
    assert env.rail.current_callsign() == 'UAV-3' and env.ev == {'hud': [3], 'rail': ['UAV-3']}


@qt_only
def test_S5_custom_callsign_resolves_through_the_registry(env):
    env.register('HAWK-01', 7)
    env.hud.on_telemetry(TelemetryFrame(sysid=7))
    env.rail.set_current('HAWK-01')
    assert env.hud.single_sysid() == 7
    env.hud._uav_cards[2].selected.emit(2)
    env.hud._uav_cards[7].selected.emit(7)
    assert env.rail.current_callsign() == 'HAWK-01'
    assert mw.uav_callsign_for_sysid(7) == 'HAWK-01' and mw.uav_sysid_for_callsign('HAWK-01') == 7
    assert mw.uav_callsign_for_sysid(42) == 'UAV-42' and mw.uav_sysid_for_callsign('UAV-42') == 42
    assert mw.uav_sysid_for_callsign('nope') is None


@qt_only
def test_S6_rail_auto_repick_on_disconnect_does_not_retarget_single_mode(env):
    env.hud.set_view_mode('single')
    assert env.hud.get_target_sysid() == 1
    env.reg.unregister('UAV-1')                    # main_window：先反註冊、再 remove_card
    assert env.rail.current_callsign() != 'UAV-1'
    assert env.hud.single_sysid() == 1, 'rail 自動改選不能帶著 HUD 換對象'
    env.hud.remove_card(1)
    assert env.hud.get_target_sysid() == SITLHud.NO_TARGET
    env.rail.set_current('UAV-3')                  # 使用者真的點了 ⇒ 才換
    assert env.hud.get_target_sysid() == 3


@qt_only
def test_S7_hidden_command_tab_defers_the_rail_until_shown(env):
    env.panel.hide()
    env.hud._uav_cards[3].selected.emit(3)
    assert env.rail.current_callsign() == 'UAV-1', 'Command 分頁沒顯示：rail 先不動（不在背景要參數）'
    assert env.win.__dict__.get('_uav_sync_pending') == 3
    env.panel.show()
    _APP.processEvents()
    assert env.rail.current_callsign() == 'UAV-3' and '_uav_sync_pending' not in env.win.__dict__
    # 記著之後 HUD 又換掉 ⇒ 顯示時套用的是最新那台
    env.panel.hide()
    env.hud._uav_cards[2].selected.emit(2)
    env.hud._uav_cards[1].selected.emit(1)
    env.panel.show()
    _APP.processEvents()
    assert env.rail.current_callsign() == 'UAV-1'
