"""main_window 的航測處理接線（2026-10-20 Step 0），不建 MainWindow（假 self／unbound method）：

* 「航測處理…」已經開著 ⇒ 叫回前面，不另建第二個；C++ 物件被刪了才重建。
* 關主視窗：航測處理／ODM 背景工作還在跑 ⇒ 問「停止並關閉／不關」；停不下來的 QThread 拆信號、parent=None、寄放。
* 飛行計畫快照：建圖任務送出寫入後先建在記憶體，第一個成功的 ACK 才寫檔（狀態列寫出檔名）；全部失敗不寫；
  寫檔失敗要大聲（警告視窗＋狀態列＋log），但不影響上傳本身。

快照的計畫用 tests/test_survey_plan_snapshot.py 的假 HARVEST 計畫（不跑規劃器）；檔案只寫在 tmp_path。
"""
from __future__ import annotations

import os
import threading
import time
import types
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    import PyQt6.QtWebEngineWidgets  # noqa: F401  先載入 QtWebEngine 再建 QApplication
    from PyQt6 import sip
    from PyQt6.QtWidgets import QApplication, QDialog
    from ui import main_window as mw
    from ui.dialogs.survey_process_dialog import SurveyProcessDialog
    _QT_OK = True
except Exception as _e:                                       # pragma: no cover - 環境相依
    _QT_OK = False
    _QT_ERR = _e

pytestmark = pytest.mark.skipif(not _QT_OK, reason=f"Qt 匯入失敗：{globals().get('_QT_ERR', '')}")

from survey.pipeline import PipelineCanceled  # noqa: E402

RING_A = [(25.1, 121.2), (25.1, 121.21), (25.11, 121.21)]
RING_B = [(25.12, 121.2), (25.12, 121.21), (25.13, 121.21), (25.13, 121.2)]


@pytest.fixture(scope="module")
def qt_app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def qmb(monkeypatch):
    QMB = mw.QMessageBox
    calls = {"information": [], "warning": [], "critical": [], "question": []}
    ret = {"question": QMB.StandardButton.No}

    def mk(name):
        def _f(*a, **k):
            calls[name].append((a, k))
            return ret.get(name, QMB.StandardButton.Ok)
        return staticmethod(_f)
    for name in calls:
        monkeypatch.setattr(QMB, name, mk(name))
    return {"calls": calls, "ret": ret, "QMB": QMB}


def wait_until(cond, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        QApplication.processEvents()
        if cond():
            return True
        time.sleep(0.01)
    QApplication.processEvents()
    return cond()


class Status:
    def __init__(self):
        self.msgs = []

    def showMessage(self, m, *a):
        self.msgs.append(m)


# ═══════════════════════════ 「航測處理…」只開一個 ═══════════════════════════

def test_on_survey_process_reuses_the_open_dialog(qt_app, monkeypatch):
    made = []

    class Dlg(QDialog):
        def __init__(self, parent=None, map_widget=None, polygons=None, gcp_blocks=None):
            super().__init__(None)                       # 假 self 不是 QWidget
            made.append(self)
            self.polygons_source, self.gcp_blocks_source = polygons, gcp_blocks
            self.raised = self.activated = 0
            self.synced = 0

        def raise_(self):
            self.raised += 1
            super().raise_()

        def activateWindow(self):
            self.activated += 1
            super().activateWindow()

        def _sync_gcp_dialog_sources(self):
            self.synced += 1

    fake = types.ModuleType("ui.dialogs.survey_process_dialog")
    fake.SurveyProcessDialog = Dlg
    monkeypatch.setitem(__import__("sys").modules, "ui.dialogs.survey_process_dialog", fake)
    win = types.SimpleNamespace(survey_areas=[list(RING_A)], corners=[], map_widget=None, _hv_coordinator=None)
    win.survey_area_polygons = types.MethodType(mw.MainWindow.survey_area_polygons, win)
    mw.MainWindow.on_survey_process(win)
    assert len(made) == 1 and win._survey_process_dialog is made[0] and made[0].isVisible()
    first_blocks = made[0].gcp_blocks_source
    made[0].hide()                                       # 使用者按「關閉」＝隱藏（處理結果、ODM 狀態都還在）
    win.survey_areas.append(list(RING_B))
    mw.MainWindow.on_survey_process(win)
    d = made[0]
    assert len(made) == 1 and d.isVisible() and d.raised == 1 and d.activated == 1
    assert d.polygons_source == [RING_A, RING_B]         # 即時來源跟著主視窗更新
    assert callable(d.gcp_blocks_source) and d.gcp_blocks_source is not first_blocks and d.synced == 1
    mw.MainWindow.on_survey_process(win)                 # 開著時再按：一樣只叫到前面
    assert len(made) == 1 and d.raised == 2
    sip.delete(d)                                        # C++ 物件被刪了 ⇒ 重建
    mw.MainWindow.on_survey_process(win)
    assert len(made) == 2 and win._survey_process_dialog is made[1] and made[1].isVisible()
    made[1].close()


# ═══════════════════════════ 關主視窗 ═══════════════════════════

def _busy_dialog(*, cooperative: bool):
    """真的 SurveyProcessDialog，ODM 槽位上跑一個真的 OdmSubmitWorker（QThread）。"""
    dlg = SurveyProcessDialog()
    gate = threading.Event()
    started = threading.Event()

    def fn(job, *, client, progress, should_cancel, poll_s):
        started.set()
        if cooperative:                                  # 像流水線：每 10 ms 看一次取消
            while not should_cancel():
                time.sleep(0.01)
            raise PipelineCanceled("已停止本機等待；遠端任務 u-1 仍在節點上，可用「接回任務」繼續")
        assert gate.wait(30)                             # 停不下來的單一步驟
        return {"uuid": "u-1", "ortho_tiles": None}
    assert dlg._start_odm_worker("submit", {"out_dir": "x"}, fn, None, dlg._on_odm_finished) is True
    assert started.wait(5)
    return dlg, gate


def test_close_decision_no_keeps_everything_running(qt_app):
    dlg, gate = _busy_dialog(cooperative=True)
    w = dlg._odm_worker
    asked = []
    win = types.SimpleNamespace(_survey_process_dialog=dlg)
    assert mw.survey_close_decision(win, ask=lambda _w, t, text: (asked.append(text), False)[1]) is False
    assert asked and "ODM 送出" in asked[0] and "接回既有任務" in asked[0] and "48 h" in asked[0]
    assert dlg._odm_worker is w and w.isRunning() and not w.is_canceled()
    assert mw.survey_close_decision(win, ask=lambda *_a: True) is True       # 合作的 worker：等得到它停
    assert dlg._odm_worker is None and not dlg.is_busy() and w.isFinished()
    assert w not in mw._HV_ORPHAN_WORKERS
    assert mw.survey_close_decision(win, ask=lambda *_a: pytest.fail("不忙就不該問")) is True
    assert mw.survey_close_decision(types.SimpleNamespace()) is True             # 沒開過對話框


def test_close_decision_parks_a_thread_that_does_not_stop(qt_app):
    dlg, gate = _busy_dialog(cooperative=False)
    w = dlg._odm_worker
    win = types.SimpleNamespace(_survey_process_dialog=dlg)
    t0 = time.monotonic()
    assert mw.survey_close_decision(win, wait_ms=200, ask=lambda *_a: True) is True
    assert time.monotonic() - t0 < 5                                            # 有上限的等待
    assert w.isRunning() and w.parent() is None and w in mw._HV_ORPHAN_WORKERS
    assert dlg._odm_worker is None and not dlg.is_busy()
    sip.delete(dlg)                                      # 對話框（與主視窗）被刪：寄放的執行緒不受影響、不 qFatal
    gate.set()
    assert w.wait(5000)
    assert wait_until(lambda: w not in mw._HV_ORPHAN_WORKERS)                    # 結束後自己離開寄放區


class _Ev:
    def __init__(self):
        self.ignored = self.accepted = False

    def ignore(self):
        self.ignored = True

    def accept(self):
        self.accepted = True


def test_close_event_asks_about_the_survey_dialog(qt_app, qmb):
    dlg, _gate = _busy_dialog(cooperative=True)
    w = dlg._odm_worker
    win = types.SimpleNamespace(_survey_process_dialog=dlg, _sitl_links=[], _sitl_launcher=None,
                                current_mission=None)
    ev = _Ev()
    mw.MainWindow.closeEvent(win, ev)                    # 「否」⇒ 不關、什麼都不拆
    assert ev.ignored and not ev.accepted and len(qmb["calls"]["question"]) == 1
    assert "航測處理仍在執行" in qmb["calls"]["question"][0][0]
    assert dlg._odm_worker is w and w.isRunning()
    qmb["ret"]["question"] = qmb["QMB"].StandardButton.Yes
    ev = _Ev()
    mw.MainWindow.closeEvent(win, ev)                    # 「是」⇒ 停止、等到、照常關
    assert ev.accepted and not ev.ignored and not dlg.is_busy() and w.isFinished()


# ═══════════════════════════ 飛行計畫快照 ═══════════════════════════

class FakeLink:
    def __init__(self, base, *, fail=False):
        self.sysid_label, self.conn_str = base.sysid_label, base.conn_str
        self.fail, self.uploads = fail, []

    def get_latest_telemetry(self):
        return types.SimpleNamespace(armed=False, mode="QLOITER")

    def upload_mission_items(self, items, verify=True):
        if self.fail:
            raise RuntimeError("連線忙")
        self.uploads.append((len(items), verify))


class Panel:
    def get_params(self):
        return {"hv_map_tlog": False}


def _upload_win(tmp_path, monkeypatch, *, fail=()):
    from survey import plan_snapshot as ps
    from tests.test_survey_plan_snapshot import make_plan
    monkeypatch.setenv(ps.ENV_DIR, str(tmp_path / "snapshots"))
    hv, swarm, jobs = make_plan()
    for j in jobs:
        j["link"] = FakeLink(j["link"], fail=j["sysid"] in fail)
    win = types.SimpleNamespace(_hv_coordinator=hv, _hv_swarm_coordinator=swarm, vtol_recon_panel=Panel(),
                                _write_pending_sysids=set(), _map_readback_info={11: ["CAM1_TYPE 相符"]},
                                _map_upload_skipped=[], _map_upload_not_targeted=[14], _hv_release_schedule=None,
                                _release=None, _status=Status())
    win.statusBar = lambda: win._status
    win._update_write_wp_enabled = lambda: None
    win._refresh_mapping_live_status = lambda: None
    for name in ("_upload_mapping_missions", "_on_mission_uploaded"):
        setattr(win, name, types.MethodType(getattr(mw.MainWindow, name), win))
    return win, hv, jobs


def _snapshot_files(tmp_path):
    return sorted((tmp_path / "snapshots").rglob("flown_plan_*.json"))


def test_snapshot_is_written_on_the_first_successful_ack_and_updated_after(qt_app, tmp_path, monkeypatch, qmb):
    from survey import plan_snapshot as ps
    win, hv, jobs = _upload_win(tmp_path, monkeypatch)
    win._upload_mapping_missions(jobs)
    pend = win._flown_plan_pending
    assert len(pend) == 1 and pend[0]["sysids"] == {11, 12} and pend[0]["path"] is None
    assert _snapshot_files(tmp_path) == []                                       # 送出≠成功：還沒寫
    report = qmb["calls"]["information"][-1][0][2]
    assert "飛行計畫快照沒建成" not in report
    win._on_mission_uploaded(99, True, "別的寫入")                               # 不是這批 ⇒ 不碰
    win._on_mission_uploaded(11, False, "逾時")                                  # 失敗 ⇒ 還是不寫
    assert _snapshot_files(tmp_path) == [] and win._flown_plan_pending[0]["reported"] == {11}
    win._on_mission_uploaded(12, True, "ACCEPTED＋讀回相符")                     # 第一個成功 ⇒ 寫檔
    files = _snapshot_files(tmp_path)
    assert len(files) == 1 and win._flown_plan_pending == []                    # 這批全部回報 ⇒ 收掉
    assert any(str(files[0]) in m and m.startswith("✅ UAV-12") for m in win._status.msgs)
    snap = ps.read_snapshot(files[0])
    up = snap["upload"]["aircraft"]
    assert up["11"]["ok"] is False and up["11"]["message"] == "逾時" and up["12"]["ok"] is True
    assert snap["upload"]["not_targeted_sysids"] == [14] and snap["upload"]["tlog"] is False
    mu = snap["plan"]["mission_uploads"]
    assert sorted(int(s) for s in mu) == [11, 12] and mu["11"]["n_items"] == len(jobs[0]["items"])
    win._on_mission_uploaded(12, True, "之後的 Write WPs")                       # 收掉之後的 ACK 不改檔
    assert ps.read_snapshot(files[0]) == snap and not qmb["calls"]["warning"]


def test_snapshot_ack_order_success_first_then_update(qt_app, tmp_path, monkeypatch, qmb):
    from survey import plan_snapshot as ps
    win, hv, jobs = _upload_win(tmp_path, monkeypatch)
    win._upload_mapping_missions(jobs)
    win._on_mission_uploaded(11, True, "ok")
    files = _snapshot_files(tmp_path)
    assert len(files) == 1 and ps.read_snapshot(files[0])["upload"]["aircraft"]["12"]["ok"] is None
    win._on_mission_uploaded(12, False, "拒絕")                                  # 已寫檔 ⇒ 就地更新同一個檔
    assert _snapshot_files(tmp_path) == files
    assert ps.read_snapshot(files[0])["upload"]["aircraft"]["12"]["ok"] is False


def test_no_snapshot_when_nothing_was_sent_or_nothing_succeeded(qt_app, tmp_path, monkeypatch, qmb):
    win, hv, jobs = _upload_win(tmp_path, monkeypatch, fail=(11, 12))
    win._upload_mapping_missions(jobs)                                           # 兩架都送不出去
    assert not getattr(win, "_flown_plan_pending", None)
    win, hv, jobs = _upload_win(tmp_path, monkeypatch, fail=(12,))
    win._upload_mapping_missions(jobs)
    assert win._flown_plan_pending[0]["sysids"] == {11}                          # 只放真的送出的那架
    win._on_mission_uploaded(11, False, "逾時")
    assert _snapshot_files(tmp_path) == [] and win._flown_plan_pending == []


def test_snapshot_write_failure_is_loud_but_does_not_break_the_upload(qt_app, tmp_path, monkeypatch, qmb):
    from survey import plan_snapshot as ps
    win, hv, jobs = _upload_win(tmp_path, monkeypatch)
    win._upload_mapping_missions(jobs)

    def boom(root, snap):
        raise OSError("磁碟滿了")
    monkeypatch.setattr(ps, "write_snapshot", boom)
    win._on_mission_uploaded(11, True, "ok")                                     # 不丟例外
    assert 11 not in win._write_pending_sysids
    warn = qmb["calls"]["warning"]
    assert len(warn) == 1 and "飛行計畫快照存檔失敗" in warn[0][0][2] and "磁碟滿了" in warn[0][0][2]
    assert win._status.msgs[-1].startswith("⚠ 飛行計畫快照存檔失敗")
    assert _snapshot_files(tmp_path) == []


def test_snapshot_build_failure_goes_into_the_upload_report(qt_app, tmp_path, monkeypatch, qmb):
    from survey import plan_snapshot as ps
    win, hv, jobs = _upload_win(tmp_path, monkeypatch)

    def boom(*a, **k):
        raise ps.PlanSnapshotError("這份計畫不是航測建圖模式")
    monkeypatch.setattr(ps, "snapshot_from_hv", boom)
    win._upload_mapping_missions(jobs)
    assert all(j["link"].uploads for j in jobs)                                  # 上傳照常
    assert not getattr(win, "_flown_plan_pending", None)
    report = qmb["calls"]["information"][-1][0][2]
    assert "飛行計畫快照沒建成" in report and "不是航測建圖模式" in report
    win._on_mission_uploaded(11, True, "ok")
    assert _snapshot_files(tmp_path) == [] and not qmb["calls"]["warning"]


def test_hook_points_are_in_the_real_methods():
    """接線在對的地方：快照建在送出迴圈之後、問題清單記 log 之前；ACK 一律交給 note_flown_plan_upload；
    closeEvent 先問航測處理（「不關」沒有副作用），才問 STOT（「中止航次」不能復原），之後才開始拆其他東西；
    航測處理的工作要等關閉定案（未儲存變更也問過）才停（審查 2026-10-20）。"""
    import inspect
    up = inspect.getsource(mw.MainWindow._upload_mapping_missions)
    assert up.index("prepare_flown_plan_snapshot(") < up.index("for p in problems:")
    assert up.index("sent.append((sysid, did))") < up.index("prepare_flown_plan_snapshot(")
    assert "note_flown_plan_upload(self, sysid, ok, message)" in inspect.getsource(mw.MainWindow._on_mission_uploaded)
    ce = inspect.getsource(mw.MainWindow.closeEvent)
    assert ce.index("survey_close_ask(self)") < ce.index("_stot_close_decision") < ce.index("_forget_tele_state")
    assert ce.index("has_unsaved_changes") < ce.index("survey_close_finish(self)") < ce.rindex("event.accept()")
    src = Path(mw.__file__).read_bytes()
    assert src.count(b"\r\n") == src.count(b"\n") and src.count(b"\r") == src.count(b"\n")     # 仍是整檔 CRLF
