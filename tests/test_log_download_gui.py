"""飛完後下載每台 .bin —— GUI 這一層（HUD 卡片鈕／進度列／自動下載勾選、MainWindow 的流程），不碰鏈路。

鏈路層（SITLLink.request_log_list / download_log / 信號）在 tests/test_log_download.py；這裡用假 link 記錄呼叫、
假 HUD 記錄訊息，MainWindow 的方法以 types.MethodType 綁到 FakeWin（與 tests/test_wind_gui.py 同一招）。
真 HUD 元件（UavInfoCard、SITLHud）另外 offscreen 建起來驗信號與進度列。
"""
from __future__ import annotations

import os
import types

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

pytest.importorskip("PyQt6")
from ui.main_window import MainWindow, flight_log_path, pick_latest_log   # noqa: E402（要在 QApplication 之前匯入）
from ui.widgets.sitl_hud import SITLHud, UavInfoCard                      # noqa: E402


@pytest.fixture(scope="module")
def qt_app():
    from PyQt6.QtWidgets import QApplication
    yield QApplication.instance() or QApplication([])


# ═══════════════════════════════════════════════════════════════════════
#  純函式
# ═══════════════════════════════════════════════════════════════════════
def test_flight_log_path_uses_the_fc_start_time_and_falls_back_to_now():
    import datetime as dt
    p = flight_log_path("UAV-2", 17, 1760600000)              # 2026-10-16 ~ 本地時間
    t = dt.datetime.fromtimestamp(1760600000)
    assert p.name == f"UAV-2_log17_{t.strftime('%H%M%S')}.bin" and p.parent.name == t.strftime("%Y%m%d")
    assert p.parts[-3] == "flight_logs" and p.parts[-4] == "runs"
    now = dt.datetime(2026, 10, 17, 9, 5, 7)
    p2 = flight_log_path("UAV-1", 3, 0, when=now)
    assert p2.name == "UAV-1_log3_090507.bin" and p2.parent.name == "20261017"


def test_pick_latest_log_takes_the_highest_id():
    assert pick_latest_log([]) is None and pick_latest_log(None) is None
    es = [{"id": 3, "size": 10, "time_utc": 5}, {"id": 12, "size": 1, "time_utc": 0}, {"id": 7, "size": 99, "time_utc": 9}]
    assert pick_latest_log(es)["id"] == 12
    assert pick_latest_log([{"id": 0, "size": 0, "time_utc": 0}])["id"] == 0


# ═══════════════════════════════════════════════════════════════════════
#  真 HUD 元件
# ═══════════════════════════════════════════════════════════════════════
def test_card_button_emits_sysid_and_progress_bar_shows_values(qt_app):
    card = UavInfoCard(3)
    got, cancels = [], []
    card.log_download_requested.connect(got.append)
    card.log_cancel_requested.connect(cancels.append)
    assert card.log_bar.isHidden() and card.log_state() == "idle"
    card.btn_logdl.click()
    assert got == [3]
    card.on_log_progress({"id": 9, "size": 2_000_000, "received": 500_000, "bytes_per_s": 8192.0, "eta_s": 183.0, "path": "x"})
    assert card.log_state() == "running" and card.log_progress.value() == 250
    assert "#9" in card.lbl_log.text() and "0.5 MB/1.9 MB" in card.lbl_log.text() and "8 KiB/s" in card.lbl_log.text()
    assert "剩 3:03" in card.lbl_log.text() and not card.btn_log_cancel.isHidden()
    card.btn_log_cancel.click()
    assert cancels == [3]
    card.on_log_done({"id": 9, "size": 2_000_000, "path": "C:/x/runs/flight_logs/20261017/UAV-3_log9_120000.bin", "sha256": "ab"})
    assert card.log_state() == "done" and card.log_progress.value() == 1000
    assert card.lbl_log.text().startswith("✔ UAV-3_log9_120000.bin") and card.btn_log_cancel.isHidden()
    card.on_log_failed("連線結束")
    assert card.log_state() == "failed" and card.lbl_log.text() == "✖ 連線結束"


def test_hud_routes_card_signals_and_progress_by_sysid(qt_app):
    hud = SITLHud()
    dl, cancel, alls = [], [], []
    hud.cmd_log_download.connect(dl.append)
    hud.cmd_log_cancel.connect(cancel.append)
    hud.cmd_log_download_all.connect(lambda: alls.append(1))
    c2 = hud._get_or_create_card(2)
    c5 = hud._get_or_create_card(5)
    c5.btn_logdl.click()
    c2.btn_log_cancel.click()
    hud.btn_log_download_all.click()
    assert dl == [5] and cancel == [2] and alls == [1]
    assert hud.auto_log_download_enabled() is False
    hud.chk_auto_logdl.setChecked(True)
    assert hud.auto_log_download_enabled() is True
    hud.on_log_progress(5, {"id": 1, "size": 100, "received": 50, "bytes_per_s": 1.0, "eta_s": 50})
    assert c5.log_state() == "running" and c2.log_state() == "idle"
    hud.on_log_done(2, {"id": 1, "size": 100, "path": "a/b.bin"})
    hud.on_log_failed(5, "x")
    assert c2.log_state() == "done" and c5.log_state() == "failed"
    hud.on_log_progress(9, {"size": 1, "received": 1})              # 沒這張卡：不炸
    hud.on_log_message(2, "hello", 6)


# ═══════════════════════════════════════════════════════════════════════
#  MainWindow 流程（FakeWin）
# ═══════════════════════════════════════════════════════════════════════
_BOUND = ("_log_dl_state", "_flight_logs", "_armed_seen", "_link_for_sysid", "_hud_log_msg",
          "_start_log_download", "_start_log_download_all", "_cancel_log_download", "_on_log_list",
          "_on_log_list_failed", "_on_log_download_progress", "_on_log_download_done", "_on_log_download_failed",
          "_note_armed_transition")


class FakeLink:
    def __init__(self, sysid, *, armed=False, mode="QLOITER", frame=True):
        self.sysid_label = sysid
        self.calls = []
        self._frame = types.SimpleNamespace(armed=armed, mode=mode) if frame else None

    def get_latest_telemetry(self):
        return self._frame

    def request_log_list(self):
        self.calls.append(("list",))

    def download_log(self, log_id, path, size):
        self.calls.append(("download", log_id, path, size))

    def cancel_log_download(self):
        self.calls.append(("cancel",))


class FakeHud:
    def __init__(self, auto=False):
        self.msgs, self.progress, self.done, self.failed = [], [], [], []
        self._auto = auto

    def on_log_message(self, sysid, text, severity=6):
        self.msgs.append((sysid, text, severity))

    def on_log_progress(self, sysid, info):
        self.progress.append((sysid, info))

    def on_log_done(self, sysid, info):
        self.done.append((sysid, info))

    def on_log_failed(self, sysid, reason):
        self.failed.append((sysid, reason))

    def auto_log_download_enabled(self):
        return self._auto


class _Status:
    def __init__(self):
        self.msgs = []

    def showMessage(self, m, *_a):
        self.msgs.append(m)


def _win(links, *, auto=False):
    w = types.SimpleNamespace()
    w._sitl_links = list(links)
    w.sitl_hud = FakeHud(auto)
    w._status = _Status()
    w.statusBar = lambda: w._status
    w.LOG_DOWNLOAD_AUTO_DELAY_S = MainWindow.LOG_DOWNLOAD_AUTO_DELAY_S
    for name in _BOUND:
        setattr(w, name, types.MethodType(getattr(MainWindow, name), w))
    return w


def test_download_flow_list_then_latest_then_done():
    l1 = FakeLink(1)
    w = _win([l1])
    assert w._start_log_download(1) is True and l1.calls == [("list",)] and w._log_dl_state()[1]["stage"] == "list"
    assert w._start_log_download(1) is False, "同一台進行中不可再開"
    w._on_log_list(1, [{"id": 4, "size": 1000, "time_utc": 0}, {"id": 9, "size": 2_500_000, "time_utc": 1760600000},
                       {"id": 7, "size": 5, "time_utc": 0}])
    kind, log_id, path, size = l1.calls[-1]
    assert (kind, log_id, size) == ("download", 9, 2_500_000)
    assert path.replace("\\", "/").split("/")[-1].startswith("UAV-1_log9_") and path.endswith(".bin")
    assert "runs/flight_logs/" in path.replace("\\", "/")
    assert w._log_dl_state()[1]["stage"] == "download" and "2.4 MB" in w._status.msgs[-1] and "共 3 份" in w._status.msgs[-1]
    w._on_log_download_progress(1, {"id": 9, "size": 2_500_000, "received": 100})
    assert w.sitl_hud.progress[-1][0] == 1
    w._on_log_download_done(1, {"id": 9, "size": 2_500_000, "path": path, "sha256": "deadbeef" * 8, "seconds": 10.0})
    assert 1 not in w._log_dl_state() and w._flight_logs()[1] == path
    assert "244 KiB/s" in w._status.msgs[-1] and w.sitl_hud.done[-1][0] == 1
    assert w._start_log_download(1) is True, "完成後可以再抓"


def test_armed_or_unknown_or_missing_link_is_refused_without_touching_the_link():
    armed = FakeLink(2, armed=True, mode="AUTO")
    unknown = FakeLink(3, frame=False)
    w = _win([armed, unknown])
    assert w._start_log_download(2) is False and armed.calls == [] and "已解鎖" in w.sitl_hud.msgs[-1][1]
    assert w._start_log_download(3) is False and unknown.calls == [] and "遙測" in w.sitl_hud.msgs[-1][1]
    assert w._start_log_download(9) is False and "沒有連線" in w.sitl_hud.msgs[-1][1]
    assert w._log_dl_state() == {}


def test_empty_list_and_failures_clear_the_state():
    l1 = FakeLink(1)
    w = _win([l1])
    w._start_log_download(1)
    w._on_log_list(1, [])
    assert 1 not in w._log_dl_state() and "沒有任何日誌" in w.sitl_hud.msgs[-1][1] and len(l1.calls) == 1
    w._start_log_download(1)
    w._on_log_list_failed(1, "8 s 沒收到 LOG_ENTRY")
    assert 1 not in w._log_dl_state() and w.sitl_hud.msgs[-1][2] == 3
    w._start_log_download(1)
    w._on_log_list(1, [{"id": 1, "size": 10, "time_utc": 0}])
    w._on_log_download_failed(1, "取消")
    assert 1 not in w._log_dl_state() and w.sitl_hud.failed == [(1, "取消")] and 1 not in w._flight_logs()
    n = len(l1.calls)
    w._on_log_list(1, [{"id": 1, "size": 10, "time_utc": 0}])          # 沒在等清單 ⇒ 不開下載
    assert len(l1.calls) == n and 1 not in w._log_dl_state()


def test_immediate_link_failure_does_not_get_overwritten_by_a_progress_message():
    """SITLLink 的『立即失敗』（尚未連線、armed、size 0、已有下載）是在呼叫端執行緒、
    return 之前就 emit ⇒ 主視窗的狀態已被 failed slot 清掉；之後不可以再蓋一句
    「索取中…」「下載中…」在失敗訊息上，也不可以回報 True。"""
    class SyncFailLink(FakeLink):
        def request_log_list(self):
            super().request_log_list()
            win._on_log_list_failed(self.sysid_label, '尚未連線到飛控')

        def download_log(self, log_id, path, size):
            super().download_log(log_id, path, size)
            win._on_log_download_failed(self.sysid_label, f'日誌 id {log_id} 的大小 {size} B 不合法')

    l1 = SyncFailLink(1)
    win = _win([l1])
    assert win._start_log_download(1) is False
    assert 1 not in win._log_dl_state()
    assert win.sitl_hud.msgs[-1][1].startswith('UAV-1 日誌清單失敗'), win.sitl_hud.msgs
    assert not any('索取' in m[1] for m in win.sitl_hud.msgs)

    # 清單回來但 download_log 立即失敗（例如 size 0）
    l2 = SyncFailLink(2)
    win = _win([l2])
    win._log_dl_state()[2] = {'stage': 'list', 'started': 0.0}
    win._on_log_list(2, [{'id': 7, 'size': 0, 'time_utc': 0}])
    assert l2.calls[-1][0] == 'download'
    assert 2 not in win._log_dl_state()
    assert win.sitl_hud.failed and win.sitl_hud.failed[-1][0] == 2
    assert not any('下載日誌 #7' in m for m in win._status.msgs), win._status.msgs


def test_telemetry_handler_really_calls_the_armed_transition_hook():
    """落地提示／自動下載的入口是 `_on_sitl_telemetry` 裡那一行 `self._note_armed_transition(frame)`；
    下面的流程測試是直接呼叫 `_note_armed_transition`，刪掉那一行它們照樣全綠 ⇒ 這裡用 AST 鎖住。"""
    import ast, pathlib
    src = (pathlib.Path(__file__).resolve().parents[1] / 'ui' / 'main_window.py').read_text(
        encoding='utf-8', errors='replace')
    fn = next(n for n in ast.walk(ast.parse(src))
              if isinstance(n, ast.FunctionDef) and n.name == '_on_sitl_telemetry')
    calls = [ast.unparse(n) for n in ast.walk(fn)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
             and n.func.attr == '_note_armed_transition']
    assert calls == ['self._note_armed_transition(frame)'], calls


def test_download_all_and_cancel():
    ok, armed = FakeLink(1), FakeLink(2, armed=True)
    w = _win([ok, armed])
    w._start_log_download_all()
    assert ok.calls == [("list",)] and armed.calls == [] and "1/2" in w._status.msgs[-1]
    w._cancel_log_download(1)
    assert ok.calls[-1] == ("cancel",)
    w2 = _win([])
    w2._start_log_download_all()
    assert "沒有連線" in w2._status.msgs[-1]


def test_landing_prompts_and_auto_download_only_when_enabled(monkeypatch):
    fired = []
    import ui.main_window as mw
    monkeypatch.setattr(mw.QTimer, "singleShot", staticmethod(lambda ms, fn: fired.append((ms, fn))))
    l1 = FakeLink(1)
    w = _win([l1])
    f = lambda armed: types.SimpleNamespace(sysid=1, armed=armed)         # noqa: E731
    w._note_armed_transition(f(False))                                     # 第一幀：沒有 prev，不算落地
    w._note_armed_transition(f(True))
    w._note_armed_transition(f(True))
    assert w.sitl_hud.msgs == []
    w._note_armed_transition(f(False))                                     # 解鎖→上鎖 ＝ 落地
    assert any("已上鎖（落地）" in m[1] for m in w.sitl_hud.msgs) and fired == [], "沒勾自動 ⇒ 只提示"
    w2 = _win([l1], auto=True)
    w2._note_armed_transition(f(True))
    w2._note_armed_transition(f(False))
    assert len(fired) == 1 and fired[0][0] == int(MainWindow.LOG_DOWNLOAD_AUTO_DELAY_S * 1000)
    fired[0][1]()                                                          # 計時器到 ⇒ 真的開始要清單
    assert l1.calls == [("list",)]
    # 延遲期間取消勾選 ⇒ 到點時不可以下載（審查 2026-10-17：以前 lambda 直接呼叫、不再查勾選）
    fired.clear()
    l3 = FakeLink(3)
    w3 = _win([l3], auto=True)
    w3._note_armed_transition(types.SimpleNamespace(sysid=3, armed=True))
    w3._note_armed_transition(types.SimpleNamespace(sysid=3, armed=False))
    assert len(fired) == 1
    w3.sitl_hud._auto = False                                              # 使用者在 8 秒內把勾拿掉
    fired[0][1]()
    assert l3.calls == [], "勾選已關閉卻還是去要清單"
    assert any("自動下載已取消" in m[1] for m in w3.sitl_hud.msgs)
    assert 3 not in w3._log_dl_state()


def test_forget_log_state_is_called_on_every_disconnect_path():
    """三條斷線路徑（逐機 _on_sitl_link_disconnected、全域 on_sitl_disconnect、HIL _on_hil_disconnect_all）
    都要清 _log_dl / _armed_state；後兩條先把 disconnected 信號切斷，逐機清理不會跑（審查 2026-10-17）。"""
    import ast, pathlib
    from ui.main_window import _forget_log_state
    w = types.SimpleNamespace(_log_dl={1: {"stage": "list"}, 2: {"stage": "download"}},
                              _armed_state={1: True, 2: False})
    _forget_log_state(w, 2)
    assert w._log_dl == {1: {"stage": "list"}} and w._armed_state == {1: True}
    _forget_log_state(w, None)
    assert w._log_dl == {} and w._armed_state == {}
    _forget_log_state(types.SimpleNamespace(), 1)                          # 假 self 沒這兩欄也不炸
    src = (pathlib.Path(__file__).resolve().parents[1] / 'ui' / 'main_window.py').read_text(
        encoding='utf-8', errors='replace')
    tree = ast.parse(src)
    for name in ('_on_sitl_link_disconnected', 'on_sitl_disconnect', '_on_hil_disconnect_all'):
        fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
        calls = [n for n in ast.walk(fn) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                 and n.func.id == '_forget_log_state']
        assert len(calls) == 1, f'{name} 沒有（或多次）呼叫 _forget_log_state'
