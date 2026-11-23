# -*- coding: utf-8 -*-
"""tests/test_fleet_registry_throttle.py — FleetRegistry 的 telemetry_updated 合幀（2026-10-17 多機效能）。

每條鏈路每秒幾十幀 × 6 架 × 三個純顯示訂閱者（Command 機卡列、機隊儀表板、單機詳情）的 setText／setStyleSheet
全在主執行緒；現在廣播以 FLEET_TELEMETRY_HZ 合幀，但黑板（latest／snapshot／latest_age_sec）每幀都立刻記。
"""
from __future__ import annotations

import os

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('AEROPLAN_NO_FILE_LOG', '1')

from PyQt6.QtCore import QCoreApplication, QObject, pyqtSignal  # noqa: E402

_APP = QCoreApplication.instance() or QCoreApplication([])

import mission.fleet_registry as fr  # noqa: E402
from mission.fleet_registry import FleetRegistry  # noqa: E402
from mission.sitl_link import TelemetryFrame  # noqa: E402


class _Link(QObject):
    telemetry = pyqtSignal(object)
    connected = pyqtSignal(str)
    disconnected = pyqtSignal(str)

    def __init__(self, sysid):
        super().__init__()
        self.sysid_label = sysid


@pytest.fixture
def reg(monkeypatch):
    fired = []
    monkeypatch.setattr(fr.QTimer, 'singleShot', staticmethod(lambda ms, fn: fired.append((ms, fn))))
    monkeypatch.setattr(fr, 'FLEET_TELEMETRY_HZ', 10.0)
    r = FleetRegistry()
    r._fired = fired
    return r


def _f(sysid, alt):
    return TelemetryFrame(sysid=sysid, alt_rel=alt)


def test_board_is_immediate_but_broadcast_is_coalesced_per_callsign(reg):
    got = []
    reg.telemetry_updated.connect(lambda cs, f: got.append((cs, f.alt_rel)))
    l1, l2 = _Link(1), _Link(2)
    reg.register('UAV-1', l1, 1); reg.register('UAV-2', l2, 2)
    for i in range(10):
        l1.telemetry.emit(_f(1, float(i)))
        l2.telemetry.emit(_f(2, 100.0 + i))
    assert reg.latest('UAV-1').alt_rel == 9.0 and reg.snapshot()[2].alt_rel == 109.0, '黑板要立刻是最新'
    assert got == [], '廣播要等窗口'
    assert len(reg._fired) == 2 and all(ms == 100 for ms, _ in reg._fired), '每個 callsign 只排一次'
    for _, fn in reg._fired:
        fn()
    assert got == [('UAV-1', 9.0), ('UAV-2', 109.0)], '發的是最新那幀'
    # 下一幀再排一次
    l1.telemetry.emit(_f(1, 42.0))
    assert len(reg._fired) == 3


def test_unregistered_during_window_does_not_broadcast(reg):
    got = []
    reg.telemetry_updated.connect(lambda cs, f: got.append(cs))
    l1 = _Link(1)
    reg.register('UAV-1', l1, 1)
    l1.telemetry.emit(_f(1, 1.0))
    reg.unregister('UAV-1')
    reg._fired[0][1]()
    assert got == [] and 'UAV-1' not in reg._pending_emit


def test_hz_zero_broadcasts_every_frame(monkeypatch):
    monkeypatch.setattr(fr, 'FLEET_TELEMETRY_HZ', 0.0)
    fired = []
    monkeypatch.setattr(fr.QTimer, 'singleShot', staticmethod(lambda ms, fn: fired.append(fn)))
    r = FleetRegistry()
    got = []
    r.telemetry_updated.connect(lambda cs, f: got.append(f.alt_rel))
    l1 = _Link(1)
    r.register('UAV-1', l1, 1)
    for i in range(3):
        l1.telemetry.emit(_f(1, float(i)))
    assert got == [0.0, 1.0, 2.0] and fired == []


def test_env_knob_is_wired_to_the_constant():
    import ast, pathlib
    tree = ast.parse(pathlib.Path(fr.__file__).read_text(encoding='utf-8', errors='replace'))
    assign = next(n for n in tree.body if isinstance(n, ast.AnnAssign) and isinstance(n.target, ast.Name)
                  and n.target.id == 'FLEET_TELEMETRY_HZ')
    assert ast.unparse(assign.value).startswith("_env_float('AEROPLAN_FLEET_HZ', ")
    assert fr._env_float('AEROPLAN_FLEET_HZ_NOPE', 10.0) == 10.0
