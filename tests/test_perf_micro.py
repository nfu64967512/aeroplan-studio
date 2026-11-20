# -*- coding: utf-8 -*-
"""tests/test_perf_micro.py — 2026-10-17 多機效能的幾個小修（字型族名快取、機卡列樣式不重套、HUD 隱藏框不更新、
一幀只排一次重繪）。"""
from __future__ import annotations

import os

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('AEROPLAN_NO_FILE_LOG', '1')

try:
    import ui.main_window  # noqa: F401
    from PyQt6.QtCore import Qt
    from PyQt6.QtWidgets import QApplication
    _APP = QApplication.instance() or QApplication([])
    from ui.resources.tactical_theme import TacticalFonts as TF
    from ui.widgets.sitl_hud import SITLHud, MissionPlannerHud
    from mission.sitl_link import TelemetryFrame
    _QT_OK = True
except Exception as _e:                       # pragma: no cover
    _QT_OK = False
    _QT_ERR = _e

qt_only = pytest.mark.skipif(not _QT_OK, reason=f'PyQt6 匯入失敗: {globals().get("_QT_ERR")}')


@qt_only
def test_font_family_is_resolved_once_per_stack():
    TF._resolve_family.cache_clear()
    f1 = TF.mono(9); f2 = TF.mono(12, bold=True); f3 = TF.condensed(10)
    info = TF._resolve_family.cache_info()
    assert info.misses == 2 and info.hits == 1, info          # mono 兩次只解析一次，condensed 一次
    assert f1.family() == f2.family() and f2.bold() and f3.capitalization().name == 'AllUppercase'
    assert isinstance(TF._resolve_family(TF.SANS_STACK), str)


@qt_only
def test_hud_update_from_frame_paints_once_and_suppresses_intermediate_updates():
    """值層級：離屏顯示的姿態儀吃一幀只 paint 一次；批次期間 update() 不排重繪（回 None）。"""
    paints = []

    class _Probe(MissionPlannerHud):
        def paintEvent(self, ev):
            paints.append(1)
            return super().paintEvent(ev)
    p = _Probe()
    p.setAttribute(Qt.WidgetAttribute.WA_DontShowOnScreen, True)
    p.resize(290, 240)
    p.show()
    for _ in range(5):
        _APP.processEvents()
    paints.clear()
    p.update_from_frame(TelemetryFrame(sysid=1, roll=1, pitch=2, heading=3))
    for _ in range(5):
        _APP.processEvents()
    assert sum(paints) == 1, f'一幀要恰好一次 paint，實際 {sum(paints)}'
    p._in_batch = True
    assert p.update() is None and sum(paints) == 1, '批次期間 update() 不可以排重繪'
    p._in_batch = False
    p.hide()


@qt_only
def test_hidden_legacy_frame_and_collapsed_servo_panel_are_not_updated():
    """用「記錄」而不是「丟例外」當探針：on_telemetry 會吞掉 Exception，丟例外的探針永遠不會紅（審查抓到）。

    2026-10-22 群控／單機：SERVO OUTPUT 與 DETAIL（舊版單機框）都在單機模式才顯示、只吃選定那台。
    群控時就算把 SERVO 展開也不更新；單機（看 UAV1）時 SERVO 只在展開時、只收 sysid 1；
    DETAIL 只在 HUD 真的顯示時、只收 sysid 1。
    """
    h = SITLHud()
    f1 = TelemetryFrame(sysid=1, lat=23.45, lon=120.28, gps_fix=3, mode='AUTO', armed=True)
    f2 = TelemetryFrame(sysid=2, lat=23.46, lon=120.28, gps_fix=3, mode='RTL', armed=True)
    seen = []
    h.servo_panel.on_telemetry = lambda fr: seen.append(fr.sysid)
    # ── 群控 ──
    h.on_telemetry(f1); h.on_telemetry(f2)
    assert h.lbl_mode.text() == '---', '群控時 DETAIL 不該被更新'
    assert h.servo_panel.isHidden() and seen == [], '群控時不該更新 servo 面板'
    h._toggle_servo_panel()                     # 群控時展開：面板仍不顯示、仍不更新
    h.on_telemetry(f1); h.on_telemetry(f2)
    assert h.servo_panel.isHidden() and seen == [] and h.lbl_mode.text() == '---'
    h._toggle_servo_panel()                     # 收回去，回到預設（摺起）
    # ── 單機（看 UAV1；第一次進單機會自動展開一次）──
    h.set_view_mode('single')
    assert h.single_sysid() == 1 and not h.servo_panel.isHidden()
    seen.clear()
    h.on_telemetry(f1); h.on_telemetry(f2)
    assert seen == [1], seen
    h._toggle_servo_panel()                     # 摺起 ⇒ 不更新
    seen.clear()
    h.on_telemetry(f1)
    assert seen == []
    # DETAIL **真的顯示**（isVisible：祖先也都顯示）時才更新：HUD 本身要先 show（離屏）
    h.lbl_mode.setText('---')
    h.on_telemetry(f1)
    assert h.lbl_mode.text() == '---', 'HUD 沒顯示時 DETAIL 不更新'
    h.setAttribute(Qt.WidgetAttribute.WA_DontShowOnScreen, True)
    h.show()
    assert h._legacy_hud_frame.isVisible()
    h.on_telemetry(f2)
    assert h.lbl_mode.text() == '---', 'DETAIL 不能吃別台的'
    h.on_telemetry(f1)
    assert h.lbl_mode.text() == 'AUTO'
    h.hide()


@qt_only
def test_servo_gate_check_can_fire():
    """自證：把守衛的狀態硬設成「單機、看 UAV1、SERVO 展開」（面板其實沒顯示）探針就會記到 ⇒ 探針有效。"""
    h = SITLHud()
    h.on_telemetry(TelemetryFrame(sysid=1))
    seen = []
    h.servo_panel.on_telemetry = lambda fr: seen.append(fr.sysid)
    h._mode = 'single'
    h._single_sysid = 1
    h._servo_visible = True
    h.on_telemetry(TelemetryFrame(sysid=1))
    h.on_telemetry(TelemetryFrame(sysid=2))
    assert seen == [1]


@qt_only
def test_fleet_rail_cell_does_not_repolish_when_severity_unchanged():
    from ui.widgets import fleet_rail
    cls = next(c for c in vars(fleet_rail).values() if isinstance(c, type) and hasattr(c, 'set_severity'))
    cell = cls.__new__(cls)
    cell._severity = 'warning'
    calls = []
    cell.setStyleSheet = lambda *a: calls.append(a)
    cls.set_severity(cell, 'warning')
    assert calls == [] and cell._severity == 'warning'
