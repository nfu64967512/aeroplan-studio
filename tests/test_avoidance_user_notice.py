# -*- coding: utf-8 -*-
"""避撞「沒做成」必須讓使用者看見(2026-10-08)。

事故:`CollisionAvoidance.__init__()` 少一個參數 ⇒ 每次規劃 TypeError ⇒
`_display_dccpp_result` 的 `except Exception` 把它吞成一行 WARNING,使用者拿到
**沒做過避撞**的路徑還以為有。另一個同形狀的洞:避撞跑完仍有衝突(碰到抬升
上限/停滯),也只留在日誌。

這裡跑**真的** `run_collision_avoidance`(模組層函式,通知與狀態列用記錄器接,不起視窗),
斷言「通知有被送出」這個行為,而不是去讀原始碼。
"""

import logging
import os
import types

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from tests.test_collision_alt_cap import _overlapping_pair   # noqa: E402

try:
    from ui import main_window as mw                          # noqa: E402
    _QT_OK = True
except Exception as _e:                                       # pragma: no cover
    _QT_OK = False
    _QT_ERR = _e

pytestmark = pytest.mark.skipif(
    not _QT_OK, reason=f"Qt 匯入失敗:{globals().get('_QT_ERR', '')}")


class _FakeWindow:
    """記錄每次通知與狀態列訊息;`_notify_avoidance_problem` 另有測試用真的方法。"""

    def __init__(self):
        self.notices = []
        self.status = []
        self.statusBar = lambda: types.SimpleNamespace(
            showMessage=lambda *a: self.status.append(a))

    def _notify_avoidance_problem(self, text):
        self.notices.append(text)


@pytest.fixture
def immediate_timer(monkeypatch):
    """`QTimer.singleShot(0, f)` 在測試裡沒有事件迴圈 ⇒ 改成立刻呼叫。"""
    monkeypatch.setattr(mw.QTimer, "singleShot", staticmethod(lambda ms, f: f()))


PARAMS = {'collision_avoidance': True, 'min_separation_m': 50.0,
          'alt_offset_m': 30.0}


def _run(win, params=None, paths=None, speed=22.0):
    return mw.run_collision_avoidance(
        PARAMS if params is None else params, speed,
        _overlapping_pair() if paths is None else paths,
        status=lambda *a: win.status.append(a),
        notify=win._notify_avoidance_problem)


# ── 例外:這次事故的形狀 ────────────────────────────────────────

def test_exception_in_avoidance_reaches_the_user(monkeypatch, immediate_timer):
    import core.dccpp.collision_avoidance as ca_mod

    class _Boom:
        def __init__(self, **kw):
            raise TypeError("got an unexpected keyword argument 'max_alt_offset_m'")

    monkeypatch.setattr(ca_mod, "CollisionAvoidance", _Boom)
    # 專案 logger 是包裝類別,底下的 .logger 才是 logging.Logger,且不往上傳遞
    # (propagate=False)⇒ caplog 收不到,直接掛 handler
    records = []
    handler = logging.Handler()
    handler.emit = records.append
    mw.logger.logger.addHandler(handler)
    try:
        win = _FakeWindow()
        conflicts, actions = _run(win)
    finally:
        mw.logger.logger.removeHandler(handler)
    assert (conflicts, actions) == ([], [])
    assert len(win.notices) == 1
    assert "TypeError" in win.notices[0] and "沒有" in win.notices[0]
    assert any(r.levelno >= logging.ERROR for r in records), \
        "例外不該只是 WARNING ⇒ 日誌也要升到 ERROR"


def test_the_real_seam_and_constructor_do_not_raise(immediate_timer):
    """對照組:真的建構子 + 真的接縫 ⇒ 不應該走到例外分支(否則上一條是假綠燈)。"""
    win = _FakeWindow()
    _run(win, params=dict(PARAMS, max_alt_offset_m=None))
    assert not any("執行失敗" in n for n in win.notices)


# ── 跑完仍有衝突 ───────────────────────────────────────────────

def test_unresolved_conflicts_reach_the_user(immediate_timer):
    """上限 == 單次抬升 ⇒ 完全重疊的航跡解不開 ⇒ 要通知,而且說得出是哪兩架。"""
    win = _FakeWindow()
    conflicts, actions = _run(win, params=dict(PARAMS, max_alt_offset_m=30.0))
    assert conflicts, "夾具沒衝突 ⇒ 測不到任何東西"
    assert len(win.notices) == 1
    note = win.notices[0]
    assert "仍有" in note and "UAV-1 / UAV-2" in note
    assert "抬升上限" in note


def test_unresolved_suppresses_the_cheerful_status_message(immediate_timer):
    """「已執行 N 個避撞動作」在沒解掉時是誤導 —— 不可以和警告同時出現。"""
    win = _FakeWindow()
    _run(win, params=dict(PARAMS, max_alt_offset_m=30.0))
    assert not any("已執行" in str(s) for s in win.status)


# ── 對照組:該安靜時要安靜 ─────────────────────────────────────

def test_resolved_conflicts_do_not_nag(immediate_timer):
    win = _FakeWindow()
    conflicts, actions = _run(win, params=dict(PARAMS, max_alt_offset_m=1e9,
                                              alt_offset_m=60.0))
    assert conflicts and actions
    assert win.notices == []
    assert any("已執行" in str(s) for s in win.status)


def test_disabled_or_single_uav_is_silent(immediate_timer):
    win = _FakeWindow()
    assert _run(win, params=dict(PARAMS, collision_avoidance=False)) == ([], [])
    one = {1: _overlapping_pair()[1]}
    assert _run(win, paths=one) == ([], [])
    assert win.notices == []


# ── 文字與彈窗本身 ─────────────────────────────────────────────

def test_describe_is_none_when_everything_is_fine():
    assert mw.describe_avoidance_problem() is None
    assert mw.describe_avoidance_problem(remaining=[]) is None


def test_notify_shows_dialog_and_status(monkeypatch):
    from PyQt6.QtWidgets import QMessageBox
    shown = []
    monkeypatch.setattr(QMessageBox, "warning",
                        staticmethod(lambda parent, title, text: shown.append((title, text))))
    win = _FakeWindow()
    mw.MainWindow._notify_avoidance_problem(win, "第一行\n第二行")
    assert shown == [("避撞未完成", "第一行\n第二行")]
    assert win.status and "第一行" in win.status[0][0] and "第二行" not in win.status[0][0]
