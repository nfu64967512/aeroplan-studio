"""2026-10-20 Step 0 對抗式審查的 UI／主視窗修正（不建 MainWindow：真的對話框＋假 self／unbound method）。

ODM 的部分對真的本機 HTTP 假節點（tests/fake_nodeodx.py）跑真的流水線與 OdmSubmitWorker；檔案只寫在 tmp_path。
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
    from PyQt6.QtWidgets import QApplication
    from ui import main_window as mw
    from ui.dialogs import survey_process_dialog as spd
    from ui.dialogs.survey_process_dialog import SurveyProcessDialog
    _QT_OK = True
except Exception as _e:                                       # pragma: no cover - 環境相依
    _QT_OK = False
    _QT_ERR = _e

pytestmark = pytest.mark.skipif(not _QT_OK, reason=f"Qt 匯入失敗：{globals().get('_QT_ERR', '')}")

from survey import pipeline  # noqa: E402
from survey.pipeline import PipelineCanceled  # noqa: E402
from tests.fake_nodeodx import FakeNodeODX  # noqa: E402
from tests.test_survey_dialog_odm_step0 import crash_at, dialog_for, factory, wait_idle, wait_until  # noqa: E402
from tests.test_survey_odm_fake_node import TOKEN, make_result  # noqa: E402


@pytest.fixture(scope="module")
def qt_app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def node(tmp_path):
    n = FakeNodeODX(tmp_path / "node", token=TOKEN).start()
    yield n
    n.stop()


@pytest.fixture
def qmb(monkeypatch):
    """主視窗的 QMessageBox 類別方法換成記錄用；``answers`` 依序回答 question（用完就答「否」）。"""
    QMB = mw.QMessageBox
    calls = {"information": [], "warning": [], "critical": [], "question": []}
    answers = []

    def mk(name):
        def _f(*a, **k):
            calls[name].append((a, k))
            if name == "question":
                return answers.pop(0) if answers else QMB.StandardButton.No
            return QMB.StandardButton.Ok
        return staticmethod(_f)
    for name in calls:
        monkeypatch.setattr(QMB, name, mk(name))
    return {"calls": calls, "answers": answers, "QMB": QMB}


_STARTED_WORKERS = []


@pytest.fixture(autouse=True)
def _stop_leftover_workers():
    """斷言失敗時也要把測試起的 QThread 停掉（否則行程結束時卡在還在跑的執行緒上）。"""
    yield
    while _STARTED_WORKERS:
        w = _STARTED_WORKERS.pop()
        try:
            w.cancel()
            w.wait(5000)
        except RuntimeError:
            pass


def _cooperative_odm_worker(dlg):
    """ODM 槽位上跑一個真的 OdmSubmitWorker（QThread），像流水線一樣每 10 ms 看一次取消。"""
    started = threading.Event()

    def fn(job, *, client, progress, should_cancel, poll_s):
        started.set()
        while not should_cancel():
            time.sleep(0.01)
        raise PipelineCanceled("已停止本機等待；遠端任務 u-1 仍在節點上，可用「接回任務」繼續")
    assert dlg._start_odm_worker("submit", {"out_dir": "x"}, fn, None, dlg._on_odm_finished) is True
    _STARTED_WORKERS.append(dlg._odm_worker)
    assert started.wait(5)
    return dlg._odm_worker


# ═══════════════════ Esc／關閉鈕：ODM 上傳只隱藏、不取消（審查 medium） ═══════════════════

def test_esc_during_an_odm_upload_only_hides_the_dialog(qt_app, tmp_path, node):
    res = make_result(tmp_path, n=8)
    dlg = dialog_for(res, node)
    dlg.show()
    assert dlg.submit_odm() is True
    w = dlg._odm_worker
    dlg.reject()                                                                # Esc
    assert not dlg.isVisible() and dlg._odm_worker is w and not w.is_canceled()
    assert "在背景繼續" in dlg.log_view.toPlainText()
    wait_idle(dlg)
    rec = pipeline.load_submission(res.out_dir)
    assert rec["phase"] == "downloaded" and "upload_canceled" not in [h["phase"] for h in rec["phase_history"]]
    # 處理（不是 ODM）在跑時照舊：要求取消、不關
    dlg.show()
    gate = threading.Event()

    def slow(sorties, out_dir, *, progress, should_cancel, **kw):
        gate.wait(5)
        raise PipelineCanceled("使用者取消")
    from ui.workers.survey_worker import SurveyProcessWorker
    pw = SurveyProcessWorker([], str(tmp_path / "o"), {}, process_fn=slow, parent=dlg)
    pw.canceled.connect(dlg._on_canceled)
    pw.failed.connect(dlg._on_failed)
    dlg._worker = pw
    pw.start()
    dlg.reject()
    assert dlg.isVisible() and pw.is_canceled() and "處理中" in dlg.lbl_errors.text()
    gate.set()
    assert pw.wait(5000)
    QApplication.processEvents()
    dlg.close()


# ═══════════════════ 主視窗關閉：問完所有問題才停（審查 low ×2） ═══════════════════

def _close_win(dlg, **kw):
    base = dict(_survey_process_dialog=dlg, _sitl_links=[], _sitl_launcher=None, current_mission=None)
    base.update(kw)
    return types.SimpleNamespace(**base)


class _Ev:
    def __init__(self):
        self.ignored = self.accepted = False

    def ignore(self):
        self.ignored = True

    def accept(self):
        self.accepted = True


def test_a_later_veto_leaves_the_survey_work_running(qt_app, qmb):
    Yes, No = qmb["QMB"].StandardButton.Yes, qmb["QMB"].StandardButton.No
    dlg = SurveyProcessDialog()
    w = _cooperative_odm_worker(dlg)
    # 航測處理「是」、未儲存的變更「否」⇒ 視窗留著，ODM 工作不能已經被停掉
    win = _close_win(dlg, current_mission=object(), has_unsaved_changes=lambda: True)
    qmb["answers"][:] = [Yes, No]
    ev = _Ev()
    mw.MainWindow.closeEvent(win, ev)
    assert ev.ignored and not ev.accepted and len(qmb["calls"]["question"]) == 2
    assert dlg._odm_worker is w and w.isRunning() and not w.is_canceled()
    # STOT「不要關閉」⇒ 同樣不能已經停了
    win = _close_win(dlg, _stot_close_decision=lambda: False)
    qmb["answers"][:] = [Yes]
    ev = _Ev()
    mw.MainWindow.closeEvent(win, ev)
    assert ev.ignored and dlg._odm_worker is w and not w.is_canceled()
    # 都同意 ⇒ 最後才停、照常關
    win = _close_win(dlg, current_mission=object(), has_unsaved_changes=lambda: True)
    qmb["answers"][:] = [Yes, Yes]
    ev = _Ev()
    mw.MainWindow.closeEvent(win, ev)
    assert ev.accepted and not dlg.is_busy() and w.isFinished()


def test_a_confirmed_remote_cancel_that_is_still_waiting_is_spelled_out_on_close(qt_app, monkeypatch):
    dlg = SurveyProcessDialog()
    _cooperative_odm_worker(dlg)
    dlg._pending_remote_cancel = ({"record_path": "D:/x/odm/submission.json"}, None)
    assert "取消遠端任務" in dlg.busy_description()
    asked = []
    win = types.SimpleNamespace(_survey_process_dialog=dlg)
    assert mw.survey_close_ask(win, ask=lambda _w, _t, text: (asked.append(text), True)[1]) is True
    assert "不會" in asked[0] and "取消遠端任務" in asked[0] and "submission.json" in asked[0]
    warned = []
    monkeypatch.setattr(spd.logger, "warning", lambda m, *a, **k: warned.append(str(m)))   # 專案 logger 不往上傳
    mw.survey_close_finish(win)
    assert dlg._pending_remote_cancel is None and not dlg.is_busy()
    assert any("不會送出" in m for m in warned)
    assert "不會送出" in dlg.log_view.toPlainText()


# ═══════════════════ 飛行計畫快照：ACK 不能只靠 sysid 對（審查 medium ×2） ═══════════════════

def _snapshot_win(tmp_path, monkeypatch):
    from survey import plan_snapshot as ps
    from tests.test_mission_write import _FakeLink, _FakeWin
    from tests.test_survey_plan_snapshot import make_plan
    monkeypatch.setenv(ps.ENV_DIR, str(tmp_path / "snapshots"))
    hv, swarm, jobs = make_plan()
    links = {11: _FakeLink(sysid=11), 12: _FakeLink(sysid=12)}
    win = _FakeWin(target_links=[links[11]], sitl_links=list(links.values()))
    win._hv_swarm_coordinator = swarm
    offsets, src = mw.harvest_takeoff_offsets(hv)
    assert mw.prepare_flown_plan_snapshot(win, hv, jobs, takeoff_offsets=offsets, takeoff_source=src) is None
    win._write_pending_sysids = {11, 12}
    return win, links


def _snapshots(tmp_path):
    return sorted((tmp_path / "snapshots").rglob("flown_plan_*.json"))


def test_an_ack_after_a_link_drop_is_not_recorded_as_the_mapping_upload(qt_app, tmp_path, monkeypatch):
    from tests.test_mission_write import _QtW
    for name in ("critical", "warning", "information", "question"):
        monkeypatch.setattr(_QtW.QMessageBox, name, staticmethod(lambda *a, **k: _QtW.QMessageBox.StandardButton.Ok))
    win, links = _snapshot_win(tmp_path, monkeypatch)
    win._on_mission_uploaded(12, False, "拒絕")
    win._on_sitl_link_disconnected(links[11], "訊號逾時")                       # 11 的 ACK 永遠不會來了
    assert win._flown_plan_pending == []                                        # 這批收掉了（沒有任何一架成功）
    win._on_mission_uploaded(11, True, "Write WPs：手動航線 12 項")              # 之後別的寫入的 ACK
    assert _snapshots(tmp_path) == []                                           # 不能變成「建圖任務寫入成功」


def test_a_write_wps_started_later_detaches_the_pending_mapping_ack(qt_app, tmp_path, monkeypatch):
    from survey import plan_snapshot as ps
    from tests.test_mission_write import _QtW, _good_mission
    monkeypatch.setattr(_QtW.QMessageBox, "question", staticmethod(lambda *a, **k: _QtW.QMessageBox.StandardButton.Yes))
    win, links = _snapshot_win(tmp_path, monkeypatch)
    win._on_mission_uploaded(12, True, "ACCEPTED＋讀回相符")                     # 12 成功 ⇒ 快照寫檔
    files = _snapshots(tmp_path)
    assert len(files) == 1
    win._write_pending_sysids = set()                                           # 11 的回報不見了（例如鏈路重連）
    win._loaded_mission = _good_mission()
    win.on_sitl_write_wp()                                                      # 對 11 寫別的任務
    assert links[11].uploaded
    snap = ps.read_snapshot(files[0])
    assert snap["upload"]["aircraft"]["11"]["ok"] is False and "Write WPs" in snap["upload"]["aircraft"]["11"]["message"]
    win._on_mission_uploaded(11, True, "Write WPs 的 ACK")
    assert ps.read_snapshot(files[0]) == snap and win._flown_plan_pending == []


def test_hil_and_global_disconnect_also_detach_pending_acks():
    import inspect
    for name in ("_on_sitl_link_disconnected", "_on_hil_disconnect_all", "on_sitl_disconnect", "on_sitl_write_wp"):
        assert "abandon_flown_plan_sysid(" in inspect.getsource(getattr(mw.MainWindow, name)), name
    assert "abandon_flown_plan_sysid(" in inspect.getsource(mw.prepare_flown_plan_snapshot)


# ═══════════════════ GCP 規劃的 areas_source 要寫快照（審查 medium ×2） ═══════════════════

def test_gcp_plan_records_the_snapshot_as_its_areas_source(qt_app, tmp_path, monkeypatch):
    from survey import gcp_plan as gp
    from survey import plan_snapshot as ps
    from tests.test_survey_dialog import RING_A
    from tests.test_survey_plan_snapshot import acked, make_plan, snap_of
    monkeypatch.setenv(ps.ENV_DIR, str(tmp_path / "snap_root"))
    snap = acked(snap_of(*make_plan()), 11)
    path = ps.write_snapshot(tmp_path / "snaps", snap)
    dlg = SurveyProcessDialog(polygons=[RING_A])
    assert dlg.load_plan_snapshot(path) is True
    g = dlg.open_gcp_plan()
    rings, label, err = g.areas()
    assert err is None and label == ps.polygons_label(snap, path) and path.name in label
    assert not label.startswith(spd.POLY_MAIN_SRC[:6])
    assert g.generate() is True
    assert g.plan.effective["areas_source"] == ps.polygons_label(snap, path)
    eff = gp.layout_geojson(g.plan)["aeroplan"]["effective"]
    assert path.name in eff["areas_source"]
    dlg.rb_plan_live.setChecked(True)                                           # 切回即時 ⇒ 標籤也回主視窗
    assert g.polygons_label is None and g.areas()[1].startswith("主視窗的作業區")
    g.close()


# ═══════════════════ 建圖匯出：明講不產生快照（審查 medium） ═══════════════════

def test_mapping_export_says_it_writes_no_snapshot(qt_app, tmp_path, monkeypatch, qmb):
    import sys
    wp = tmp_path / "UAV1.waypoints"
    summary = {"uavs": {"UAV1": {"n_items": 3, "files": {"waypoints": str(wp), "param": str(tmp_path / "UAV1.param")}}},
               "files": {"summary": str(tmp_path / "mission_summary.json")}}
    # 假的 hv_dccpp.ardupilot（不匯入、不跑規劃器；另一個工作階段正在改 hv_dccpp/）
    monkeypatch.setitem(sys.modules, "hv_dccpp.ardupilot", types.SimpleNamespace(export_all=lambda *a, **k: summary))
    monkeypatch.setattr(mw, "hv_upload_gate", lambda hv, sw: {"uploadable": True, "reasons": []})
    cc = types.SimpleNamespace(interval_min_s=1.0, duration_s=0.1, backend="servo", firmware="4.6")
    hv = types.SimpleNamespace(camera_cfg=cc, last_plan=None, last_ctx=None, cfg=None,
                               mapping_mode=types.SimpleNamespace(lead_in_m=50.0))
    status = types.SimpleNamespace(msgs=[])
    win = types.SimpleNamespace(_hv_coordinator=hv, _hv_swarm_coordinator=None,
                                statusBar=lambda: types.SimpleNamespace(showMessage=lambda m, *a: status.msgs.append(m)))
    mw.MainWindow._export_hv_mapping(win, str(tmp_path))
    text = qmb["calls"]["information"][-1][0][2]
    assert mw.FLOWN_PLAN_EXPORT_NOTE in text and "不會產生飛行計畫快照" in text
    assert "Mission Planner" in spd.PLAN_LIVE_TEXT and "沒有快照" in spd.PLAN_LIVE_TEXT


# ═══════════════════ 接回另一個架次：先問、確定才整組換（審查 medium） ═══════════════════

def test_reattaching_another_campaign_asks_then_switches_folder_and_dsm(qt_app, tmp_path, node):
    res_b = make_result(tmp_path, name="campaign_B")
    uid = crash_at(res_b, node, "running")["uuid"]
    camp_a = tmp_path / "campaign_A"
    (camp_a / "odm").mkdir(parents=True)
    a_dsm = camp_a / "A_dsm.tif"
    a_dsm.write_bytes(b"II*\x00a")
    dlg = SurveyProcessDialog()
    dlg.odm_client_factory = factory
    dlg.odm_poll_s = 0.0
    dlg.ed_odm_token.setText(TOKEN)
    dlg.ed_out_dir.setText(str(camp_a))
    dlg.ed_dsm.setText(str(a_dsm))
    dlg.cmb_dsm_vertical.setCurrentIndex(dlg.cmb_dsm_vertical.findData("ellipsoidal"))
    dlg._dsm_vertical_source = "geo.txt"
    asked = []
    dlg._confirm = lambda title, text: (asked.append(text), False)[1]          # 「否」⇒ 什麼都不動
    assert dlg.reattach_odm(res_b.out_dir) is False
    assert asked and str(camp_a) in asked[0] and str(res_b.out_dir) in asked[0]
    assert dlg.ed_out_dir.text() == str(camp_a) and dlg.ed_dsm.text() == str(a_dsm) and dlg._odm_worker is None
    assert node.requests_to(f"/task/{uid}/download") == []
    dlg._confirm = lambda title, text: True                                     # 「是」⇒ 整組換成 B
    assert dlg.reattach_odm(res_b.out_dir) is True
    wait_idle(dlg)
    assert dlg.odm_result["phase"] == "downloaded"
    assert dlg.ed_out_dir.text() == str(res_b.out_dir)
    assert dlg.ed_dsm.text() == str(Path(dlg.odm_result["extract_dir"]) / "odm_dem" / "dsm.tif")   # B 的 DSM
    assert dlg.cmb_dsm_vertical.currentData() is None and dlg._dsm_vertical_source is None        # 基準要重選
    assert "campaign_A" in dlg.log_view.toPlainText() and "A_dsm.tif" in dlg.log_view.toPlainText()
    # 同一個資料夾再接一次：不問
    dlg._confirm = lambda title, text: pytest.fail("同一個資料夾不該問")
    assert dlg.reattach_odm(res_b.out_dir) is True
    wait_idle(dlg)


# ═══════════════════ 重用開著的對話框：測區標籤要更新（審查 low） ═══════════════════

def test_reusing_a_visible_dialog_refreshes_the_polygon_label(qt_app, monkeypatch):
    import sys
    from tests.test_survey_dialog import RING_A, RING_B

    class Dlg(SurveyProcessDialog):                                             # 真的對話框；假 self 不是 QWidget
        def __init__(self, parent=None, map_widget=None, polygons=None, gcp_blocks=None):
            super().__init__(None, map_widget, polygons, gcp_blocks)
    fake = types.ModuleType("ui.dialogs.survey_process_dialog")
    fake.SurveyProcessDialog = Dlg
    monkeypatch.setitem(sys.modules, "ui.dialogs.survey_process_dialog", fake)
    win = types.SimpleNamespace(survey_areas=[list(RING_A)], corners=[], map_widget=None, _hv_coordinator=None)
    win.survey_area_polygons = types.MethodType(mw.MainWindow.survey_area_polygons, win)
    mw.MainWindow.on_survey_process(win)
    dlg = win._survey_process_dialog
    dlg.chk_use_polygon.setChecked(True)
    assert dlg.isVisible() and "1 塊" in dlg.lbl_polygon_source.text()
    win.survey_areas.append(list(RING_B))
    mw.MainWindow.on_survey_process(win)                                        # 還開著：叫到前面
    assert win._survey_process_dialog is dlg and "2 塊" in dlg.lbl_polygon_source.text()
    dlg.close()


# ═══════════════════ 自動送 ODM 沒送 ⇒ 要說；處理鈕停用 ⇒ 要說怎麼騰出來（審查 low ×2） ═══════════════════

def test_auto_submit_that_cannot_run_says_why(qt_app, tmp_path):
    from tests.test_survey_dialog import fake_result
    res = fake_result(tmp_path)
    dlg = SurveyProcessDialog()
    dlg.spn_res.setValue(2.5)
    dlg.chk_auto_submit.setChecked(True)
    g = tmp_path / "gcp_list.txt"
    g.write_text("EPSG:4326\n", encoding="utf-8")
    dlg.ed_gcp_txt.setText(str(g))
    dlg.cmb_gcp_vertical.setCurrentIndex(dlg.cmb_gcp_vertical.findData("ellipsoidal"))
    sent = []
    dlg.odm_run_fn = lambda job, **k: (sent.append(job), {})[1]
    dlg._on_finished(res)                                                       # --force-gps 還沒選 ⇒ 不送
    assert sent == [] and dlg._odm_worker is None
    assert "自動送 ODM 沒有執行" in dlg.lbl_odm_msg.text() and "--force-gps" in dlg.lbl_odm_msg.text()
    assert "自動送 ODM 沒有執行" in dlg.log_view.toPlainText() and "自動送 ODM 沒有執行" in dlg.lbl_summary.text()


def test_run_button_explains_how_to_free_it_during_an_odm_wait(qt_app):
    dlg = SurveyProcessDialog()
    w = _cooperative_odm_worker(dlg)
    assert not dlg.btn_run.isEnabled()
    tip = dlg.btn_run.toolTip()
    assert "取消" in tip and "接回既有任務" in tip and "遠端任務在節點上繼續跑" in tip
    assert dlg.start_processing() is False and "不能開始處理" in dlg.lbl_errors.text()
    dlg.cancel()
    assert w.wait(5000)
    assert wait_until(lambda: dlg._odm_worker is None)
    assert dlg.btn_run.isEnabled() and dlg.btn_run.toolTip() == ""


def test_hook_points_are_in_the_real_close_event():
    import inspect
    ce = inspect.getsource(mw.MainWindow.closeEvent)
    assert ce.index("survey_close_ask(self)") < ce.index("_stot_close_decision") < ce.index("has_unsaved_changes") \
        < ce.index("survey_close_finish(self)") < ce.rindex("event.accept()")
    assert "survey_close_decision(" not in ce
    src = Path(mw.__file__).read_bytes()
    assert src.count(b"\r\n") == src.count(b"\n") == src.count(b"\r")
