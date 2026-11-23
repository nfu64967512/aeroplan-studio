"""航測處理對話框的 Step 0（2026-10-20）UI 接線：--force-gps 必選、--max-concurrency、節點／引擎版本、真進度、
接回既有任務、取消遠端任務、品質報告列進「結果」頁、飛行計畫快照當計畫來源。

ODM 的部分對**真的本機 HTTP 假節點**（tests/fake_nodeodx.py：NodeODX v2.4.0 ＋ ODX v3.8.4 的規格）跑真的
survey.pipeline（run／resume／cancel_odm_job），經過真的 OdmSubmitWorker（QThread）；不建 MainWindow。
所有檔案只寫在 tmp_path；權杖逐檔 grep 不可出現。
"""
from __future__ import annotations

import json
import os
import shutil
import time
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    import PyQt6.QtWebEngineWidgets  # noqa: F401  先載入 QtWebEngine 再建 QApplication
    from PyQt6.QtWidgets import QApplication
    from ui.dialogs.gcp_plan_dialog import GcpPlanDialog
    from ui.dialogs import survey_process_dialog as spd
    from ui.dialogs.survey_process_dialog import SurveyProcessDialog
    _QT_OK = True
except Exception as _e:                                       # pragma: no cover - 環境相依
    _QT_OK = False
    _QT_ERR = _e

pytestmark = pytest.mark.skipif(not _QT_OK, reason=f"Qt 匯入失敗：{globals().get('_QT_ERR', '')}")

from survey import pipeline  # noqa: E402
from survey.odm_client import NodeOdmClient  # noqa: E402
from tests.fake_nodeodx import FakeNodeODX  # noqa: E402
from tests.test_survey_odm_fake_node import (CRASHES, NOOP, TOKEN, CrashSession, _Crash, gcp_file,  # noqa: E402
                                             job_for, make_result, session)


@pytest.fixture(scope="module")
def qt_app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def node(tmp_path):
    n = FakeNodeODX(tmp_path / "node", token=TOKEN).start()
    yield n
    n.stop()


def factory(url, token):
    """對話框建客戶端的方式（測試版）：不吃系統 proxy 設定的 session（127.0.0.1 不能被代理攔走）。"""
    return NodeOdmClient(url, token=token, session=session(), timeout_s=30)


def wait_until(cond, timeout=30.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        QApplication.processEvents()
        if cond():
            return True
        time.sleep(0.01)
    QApplication.processEvents()
    return cond()


def wait_idle(dlg, timeout=60.0):
    assert wait_until(lambda: dlg._odm_worker is None and dlg._pending_remote_cancel is None, timeout), \
        f"ODM 工作沒結束：{dlg._odm_op} {dlg.lbl_progress.text()}"


def dialog_for(res, node):
    dlg = SurveyProcessDialog()
    dlg.result = res
    dlg.spn_res.setValue(2.5)
    dlg.ed_odm_url.setText(node.url)
    dlg.ed_odm_token.setText(TOKEN)
    dlg.odm_client_factory = factory
    dlg.odm_poll_s = 0.0
    dlg._update_buttons()
    return dlg


def odm_rows(dlg):
    t = dlg.qa_table
    return [(t.item(r, 1).text(), t.item(r, 2).text(), t.item(r, 3).text()) for r in range(t.rowCount())
            if t.item(r, 0) is not None and t.item(r, 0).text() == "ODM"]


def assert_no_token(dlg, *roots):
    for root in roots:
        for p in Path(root).rglob("*"):
            if p.is_file():
                assert TOKEN.encode() not in p.read_bytes(), p
    for text in (dlg.log_view.toPlainText(), dlg.lbl_odm_task.text(), dlg.lbl_odm_msg.text(), dlg.lbl_errors.text(),
                 dlg.lbl_odm_node.text(), dlg.lbl_odm_quality.text(), dlg.lbl_progress.text()):
        assert TOKEN not in text


def crash_at(res, node, phase, **job_kw):
    with pytest.raises(_Crash):
        pipeline.run_odm_job(job_for(res, node, **job_kw), poll_s=0, sleep=NOOP,
                             client=NodeOdmClient(node.url, token=TOKEN, session=CrashSession(*CRASHES[phase]),
                                                  timeout_s=30))
    return pipeline.load_submission(res.out_dir)


# ─────────────────────────────────────────────────────────────────────────────
# --force-gps：有 GCP 時必選、沒有 GCP 時不適用
# ─────────────────────────────────────────────────────────────────────────────

def test_force_gps_is_required_with_gcp_and_not_applicable_without(qt_app, tmp_path):
    from tests.test_survey_dialog import wait_worker
    res = make_result(tmp_path)
    dlg = SurveyProcessDialog()
    dlg.result = res
    dlg.spn_res.setValue(2.5)
    dlg._update_buttons()
    jobs = []
    dlg.odm_run_fn = lambda job, **k: (jobs.append(job), {"ortho_tiles": None})[1]
    # 沒有 GCP：選項停用、明說不適用；送出的任務記 not_applicable_no_gcp、不送 --force-gps
    assert not dlg.cmb_force_gps.isEnabled() and dlg.lbl_force_gps.text() == spd.FORCE_GPS_NA_TEXT
    assert dlg.force_gps_choice() is None and dlg.odm_block_reason() is None
    # 有 GCP：沒有預設（第一項＝未選），不選就不能送，原因寫在按鈕與標籤上
    dlg.ed_gcp_txt.setText(str(gcp_file(tmp_path, res)))
    dlg.cmb_gcp_vertical.setCurrentIndex(dlg.cmb_gcp_vertical.findData("ellipsoidal"))
    assert dlg.cmb_force_gps.isEnabled() and dlg.cmb_force_gps.currentData() is None
    reason = dlg.odm_block_reason()
    assert "--force-gps" in reason and "沒有預設" in reason
    assert not dlg.btn_odm.isEnabled() and dlg.btn_odm.toolTip() == reason
    assert dlg.lbl_force_gps.text().startswith("✘ 必選")
    assert dlg.submit_odm() is False and jobs == [] and "--force-gps" in dlg.lbl_errors.text()
    # 兩個選項的文字照使用者的用語
    assert [dlg.cmb_force_gps.itemText(i) for i in range(1, 3)] == [
        "照片 GPS 也參與平差（--force-gps；有 PPK 時建議）", "只用 GCP（照片 GPS 不參與平差）"]
    for choice, decision, sent in ((True, "force_gps", True), (False, "gcp_only", False)):
        dlg.cmb_force_gps.setCurrentIndex(dlg.cmb_force_gps.findData(choice))
        assert dlg.odm_block_reason() is None and dlg.btn_odm.isEnabled()
        assert dlg.submit_odm() is True
        wait_worker(dlg, "_odm_worker")
        j = jobs[-1]
        assert ({"name": "force-gps", "value": True} in j["options"]) is sent
        assert j["force_gps"]["decision"] == decision and j["force_gps"]["source"] == "operator"
        assert dlg.result.effective["odm_submission"]["force_gps"]["decision"] == decision
    # 拿掉 GCP ⇒ 回到不適用（操作員剛才的選擇不會被當成「有 GCP 時的決定」送出）
    dlg.ed_gcp_txt.setText("")
    dlg.cmb_gcp_vertical.setCurrentIndex(0)
    assert not dlg.cmb_force_gps.isEnabled() and dlg.lbl_force_gps.text() == spd.FORCE_GPS_NA_TEXT
    assert dlg.submit_odm() is True
    wait_worker(dlg, "_odm_worker")
    j = jobs[-1]
    assert j["force_gps"]["decision"] == "not_applicable_no_gcp" and j["force_gps"]["sent"] is False
    assert all(o["name"] != "force-gps" for o in j["options"])


def test_max_concurrency_is_recorded_and_sent_only_when_set(qt_app, tmp_path):
    from tests.test_survey_dialog import wait_worker
    res = make_result(tmp_path)
    dlg = SurveyProcessDialog()
    dlg.result = res
    dlg.spn_res.setValue(2.5)
    jobs = []
    dlg.odm_run_fn = lambda job, **k: (jobs.append(job), {"ortho_tiles": None})[1]
    assert dlg.spn_max_conc.value() == 0 and dlg.max_concurrency_choice() is None
    assert dlg.lbl_max_conc.text().startswith("有效值：不送（引擎預設＝節點全部核心）") and "10–12" in dlg.lbl_max_conc.text()
    assert dlg.submit_odm() is True
    wait_worker(dlg, "_odm_worker")
    assert all(o["name"] != "max-concurrency" for o in jobs[-1]["options"])          # 沒設 ⇒ 不送
    assert jobs[-1]["max_concurrency"]["sent"] is False and jobs[-1]["max_concurrency"]["source"] == "engine_default"
    assert dlg.result.effective["odm_submission"]["max_concurrency"]["source"] == "engine_default"   # 不送也記
    dlg.spn_max_conc.setValue(11)
    assert dlg.lbl_max_conc.text().startswith("有效值：送 --max-concurrency 11（操作員設定）")
    assert dlg.submit_odm() is True
    wait_worker(dlg, "_odm_worker")
    assert {"name": "max-concurrency", "value": 11} in jobs[-1]["options"]
    assert jobs[-1]["max_concurrency"] == {**jobs[-1]["max_concurrency"], "sent": True, "value": 11, "source": "operator"}


# ─────────────────────────────────────────────────────────────────────────────
# 對真的假節點：測試連線、送出（進度、引擎版本、品質報告、權杖不落地）
# ─────────────────────────────────────────────────────────────────────────────

def test_probe_and_submit_against_the_fake_node(qt_app, tmp_path, node):
    res = make_result(tmp_path)
    dlg = dialog_for(res, node)
    dlg.spn_max_conc.setValue(10)
    dlg.ed_gcp_txt.setText(str(gcp_file(tmp_path, res)))
    dlg.cmb_gcp_vertical.setCurrentIndex(dlg.cmb_gcp_vertical.findData("ellipsoidal"))
    dlg.cmb_force_gps.setCurrentIndex(dlg.cmb_force_gps.findData(True))
    # 測試連線：GET /info ＋ /options，顯示引擎版本、引擎認不認得 force-gps／max-concurrency
    assert dlg.test_odm_connection() is True
    wait_idle(dlg)
    node_txt = dlg.lbl_odm_node.text()
    assert "NodeODX 2.4.0" in node_txt and "引擎 ODX 3.8.4" in node_txt
    assert "force-gps ✔" in node_txt and "max-concurrency ✔" in node_txt
    assert node.requests_to("/task/new") == []                                   # 測試連線不建任務
    # 送出：真的 run_odm_job，分批上傳 → 提交 → 輪詢 → 下載 → 品質報告
    states = []
    orig = dlg._on_odm_state                                                     # 接線前換掉 ⇒ 一則都不漏
    dlg._on_odm_state = lambda st: (states.append(st), orig(st))
    assert dlg.submit_odm() is True
    wait_idle(dlg)
    rec = pipeline.load_submission(res.out_dir)
    assert rec["phase"] == "downloaded" and dlg.odm_result["phase"] == "downloaded"
    assert rec["token_used"] is True and rec["node"]["engineVersion"] == "3.8.4"
    assert rec["force_gps"]["decision"] == "force_gps" and rec["max_concurrency"]["value"] == 10
    assert rec["plan_source"]["kind"] == "live"                                 # 送出時的計畫來源也進紀錄
    assert dlg.odm_progress.value() == 1000 and dlg.progress_bar.value() == 1000
    assert {s.get("phase") for s in states} >= {"uploading", "running", "downloaded"}
    fr = [s["frac"] for s in states if s.get("frac") is not None]
    assert fr == sorted(fr) and fr[-1] == 1.0                                    # 真的進度、不倒退
    task = dlg.lbl_odm_task.text()
    assert rec["uuid"] in task and "已下載" in task and "引擎 ODX 3.8.4" in task
    # 品質報告列進「結果」頁（機 = ODM），報告檔路徑寫出來
    items = {r[0]: r for r in odm_rows(dlg)}
    assert items["品質報告 stats.json"][2] == "✔ 通過" and items["模型分塊數"][1] == "1"
    assert "GCP 誤差" in items and items["引擎（log.json）"][1].startswith("ODX 3.8.4")
    pdf = Path(dlg.odm_result["extract_dir"]) / "odm_report" / "report.pdf"
    assert pdf.is_file() and f"report.pdf：{pdf}" in dlg.lbl_odm_quality.text()
    assert dlg.tabs.currentIndex() == 3
    # 送完可以再送（舊紀錄已收尾 ⇒ 搬到 history/）；權杖從頭到尾不落地
    assert_no_token(dlg, res.out_dir)
    assert not dlg.btn_odm_cancel_remote.isEnabled()                             # 已下載：沒有可取消的遠端任務


def test_probe_failure_is_shown(qt_app, tmp_path, node):
    dlg = SurveyProcessDialog()
    dlg.odm_client_factory = factory
    dlg.ed_odm_url.setText(node.url)
    dlg.ed_odm_token.setText("wrong-token")
    assert dlg.test_odm_connection() is True
    wait_idle(dlg)
    assert dlg.lbl_odm_node.text().startswith("✘ 連不上節點") and "authentication" in dlg.lbl_odm_node.text()
    dlg.ed_odm_url.setText(node.url + "/?token=abc")                             # 位址帶權杖 ⇒ 不連、不回顯
    assert dlg.test_odm_connection() is False and "權杖" in dlg.lbl_odm_node.text()
    assert "abc" not in dlg.lbl_odm_node.text()


# ─────────────────────────────────────────────────────────────────────────────
# 接回既有任務（當機後重開對話框）
# ─────────────────────────────────────────────────────────────────────────────

def test_reattach_resumes_a_crashed_task_and_keeps_the_checkpoint_autofill(qt_app, tmp_path, node):
    res = make_result(tmp_path)
    rec0 = crash_at(res, node, "running", gcp_txt=gcp_file(tmp_path, res), gcp_vertical="ellipsoidal",
                    force_gps=True)
    assert rec0["phase"] == "running"
    moved = tmp_path / "moved_out"                                               # 資料夾搬過也能接
    shutil.move(str(res.out_dir), str(moved))
    dlg = SurveyProcessDialog()                                                  # 新開的對話框：沒有處理結果
    dlg.odm_client_factory = factory
    dlg.odm_poll_s = 0.0
    # 送出時用了權杖、欄位空白 ⇒ 擋下（權杖不存檔，要再填）
    assert dlg.reattach_odm(moved) is False and "權杖" in dlg.lbl_errors.text() and dlg._odm_worker is None
    assert rec0["uuid"] in dlg.lbl_odm_task.text() and "節點處理中" in dlg.lbl_odm_task.text()
    assert dlg.btn_odm_cancel_remote.isEnabled()
    dlg.ed_odm_token.setText(TOKEN)
    assert dlg.reattach_odm(moved) is True
    wait_idle(dlg)
    rec = pipeline.load_submission(moved)
    assert dlg.odm_result["phase"] == rec["phase"] == "downloaded" and dlg.odm_result["uuid"] == rec0["uuid"]
    assert any(e["event"] == "reattach" for e in rec["events"])
    assert dlg.ed_odm_url.text() == node.url and dlg.ed_out_dir.text() == str(moved)
    # 檢核點自動帶入（同一條收尾路徑）：DSM 欄位帶成果裡的 dsm.tif
    assert dlg.ed_dsm.text() == str(Path(dlg.odm_result["extract_dir"]) / "odm_dem" / "dsm.tif")
    assert {r[0] for r in odm_rows(dlg)} >= {"品質報告 stats.json", "影像（上傳／進 OpenSfM／重建）"}
    assert "已下載" in dlg.lbl_odm_task.text() and dlg.odm_progress.value() == 1000
    assert_no_token(dlg, moved)


def test_reattach_refuses_finished_and_uncommitted_records_loudly(qt_app, tmp_path, node):
    res = make_result(tmp_path)
    crash_at(res, node, "uploading")                                             # 上傳到一半當機：沒有提交
    dlg = SurveyProcessDialog()
    dlg.odm_client_factory = factory
    dlg.odm_poll_s = 0.0
    dlg.ed_odm_token.setText(TOKEN)
    assert dlg.reattach_odm(res.out_dir) is True                                 # 交給 resume 判斷（節點上找不到 ⇒ 大聲）
    wait_idle(dlg)
    err = dlg.lbl_errors.text()
    assert "沒有提交" in err and "重新送出" in err and "48 h" in err and dlg.odm_result is None
    assert dlg.reattach_odm(tmp_path / "nowhere") is False and "找不到任務紀錄" in dlg.lbl_errors.text()


# ─────────────────────────────────────────────────────────────────────────────
# 取消遠端任務
# ─────────────────────────────────────────────────────────────────────────────

def test_remote_cancel_asks_first_then_calls_the_node(qt_app, tmp_path, node):
    res = make_result(tmp_path)
    uid = crash_at(res, node, "running")["uuid"]
    dlg = SurveyProcessDialog()
    dlg.odm_client_factory = factory
    dlg.ed_odm_token.setText(TOKEN)
    asked = []
    dlg._confirm = lambda title, text: (asked.append(text), False)[1]            # 操作員按「否」
    assert dlg.cancel_remote_odm(res.out_dir) is False
    assert asked and uid in asked[0] and "不能接續" in asked[0] and node.requests_to("/task/cancel") == []
    dlg._confirm = lambda title, text: True
    assert dlg.cancel_remote_odm() is True
    wait_idle(dlg)
    assert len(node.requests_to("/task/cancel")) == 1 and node.tasks[uid]["status"]["code"] == 50
    rec = pipeline.load_submission(res.out_dir)
    assert rec["phase"] == "canceled" and dlg._odm_status["phase"] == "canceled"
    assert "已取消" in dlg.lbl_odm_task.text() and not dlg.btn_odm_cancel_remote.isEnabled()
    # 取消了就不能接回（照實說），也不能再取消
    assert dlg.reattach_odm(res.out_dir) is False and "不能接回" in dlg.lbl_errors.text()
    assert dlg.cancel_remote_odm() is False and "沒有可以取消" in dlg.lbl_errors.text()
    assert_no_token(dlg, res.out_dir)


def test_remote_cancel_stops_the_local_reattach_first(qt_app, tmp_path, node):
    res = make_result(tmp_path)
    uid = crash_at(res, node, "running")["uuid"]
    dlg = SurveyProcessDialog()
    dlg.odm_client_factory = factory
    dlg.ed_odm_token.setText(TOKEN)
    dlg.odm_poll_s = 30.0                                                        # 接回後停在輪詢的等待裡
    assert dlg.reattach_odm(res.out_dir) is True
    # 當機前節點收過 2 次 info（第 3 次在送出前當掉）；接回後第 1 次輪詢就是第 3 次，接著停在 30 s 的等待裡
    assert wait_until(lambda: len(node.requests_to(f"/task/{uid}/info")) >= 3)
    dlg._confirm = lambda title, text: "會先停止它" in text
    assert dlg.cancel_remote_odm() is True and dlg._pending_remote_cancel is not None
    assert not dlg.btn_odm_cancel_remote.isEnabled()
    wait_idle(dlg, timeout=20)
    log = dlg.log_view.toPlainText().splitlines()
    # 本機先停；停止訊息是流水線給的（說明遠端還在跑、可接回），不是寫死的「NodeODM 上的任務仍在執行」
    assert any(ln.startswith("ODM 接回任務：已停止本機") and "可用「接回任務」繼續" in ln for ln in log), log
    assert node.tasks[uid]["status"]["code"] == 50 and pipeline.load_submission(res.out_dir)["phase"] == "canceled"


# ─────────────────────────────────────────────────────────────────────────────
# 計畫來源：即時 vs 飛行計畫快照
# ─────────────────────────────────────────────────────────────────────────────

def test_plan_snapshot_feeds_polygons_and_gcp_blocks_and_is_recorded(qt_app, tmp_path, monkeypatch):
    from survey import gcp_plan as gp
    from survey import plan_snapshot as ps
    from tests.test_survey_dialog import RING_A, make_session
    from tests.test_survey_plan_snapshot import AREA_XY, acked, ll, make_plan, snap_of
    monkeypatch.setenv(ps.ENV_DIR, str(tmp_path / "snap_root"))
    hv, swarm, jobs = make_plan()
    snap = acked(snap_of(hv, swarm, jobs), 11)
    path = ps.write_snapshot(tmp_path / "snaps", snap)
    live_prov = lambda: ({"L1": [RING_A]}, {"L1": {"drone_id": 9}}, "即時 HARVEST")   # noqa: E731
    dlg = SurveyProcessDialog(polygons=[RING_A], gcp_blocks=live_prov)
    b, s, _ = make_session(tmp_path)
    dlg.add_sortie_row(bin_path=str(b), session_dir=str(s))
    dlg.ed_out_dir.setText(str(tmp_path / "out"))
    dlg.rb_exposure_start.setChecked(True)
    dlg.chk_use_polygon.setChecked(True)
    # 即時（預設、舊行為）：測區來自主視窗，計畫來源記成即時狀態
    assert dlg.plan_source_mode() == "live" and dlg.rb_plan_live.isChecked()
    inp, errors = dlg.collect_inputs()
    src = inp["kwargs"]["input_sources"]
    assert errors == [] and inp["kwargs"]["polygons"] == [RING_A]
    assert src["plan_source"] == spd.PLAN_LIVE_SRC and "plan_snapshot" not in src
    # 選了快照卻還沒載入 ⇒ 拒絕（不退回即時狀態），GCP 規劃也不開
    dlg.rb_plan_snapshot.setChecked(True)
    inp, errors = dlg.collect_inputs()
    assert inp is None and len(errors) == 1 and "還沒有載入快照" in errors[0]
    assert dlg.open_gcp_plan() is None and "還沒有載入快照" in dlg.lbl_errors.text()
    # 載入快照 ⇒ 測區、GCP 區塊都來自快照；來源（檔名＋雜湊，不放絕對路徑）寫進 input_sources
    assert dlg.load_plan_snapshot(path) is True and dlg.ed_plan_snapshot.text() == str(path)
    assert dlg.lbl_plan_snapshot.text().splitlines()[0] == ps.source_label(snap, path)
    x0, y0, x1, y1 = AREA_XY
    want = [[ll(x0, y0), ll(x1, y0), ll(x1, y1), ll(x0, y1)]]
    inp, errors = dlg.collect_inputs()
    src = inp["kwargs"]["input_sources"]
    assert errors == [] and inp["kwargs"]["polygons"] == want
    assert src["polygons"] == ps.polygons_label(snap, path) and src["plan_source"] == ps.source_label(snap, path)
    assert json.loads(src["plan_snapshot"]) == ps.source_record(snap, path)
    assert str(tmp_path) not in src["plan_snapshot"] and all(isinstance(v, str) for v in src.values())
    assert dlg.plan_source_record()["path"] == str(path)                        # ODM 任務紀錄（本機）才放完整路徑
    g = dlg.open_gcp_plan()
    assert isinstance(g, GcpPlanDialog) and g.polygons_source() == want
    live_blocks, live_info = gp.blocks_from_hv(hv)
    blocks, info, label, err = g.blocks()
    assert err is None and blocks == live_blocks and info == live_info and path.name in label
    # 切回即時 ⇒ 開著的 GCP 規劃也換回即時來源
    dlg.rb_plan_live.setChecked(True)
    assert g.polygons_source == [RING_A] and g.blocks_source is live_prov
    assert dlg.collect_inputs()[0]["kwargs"]["polygons"] == [RING_A]
    # 壞掉的快照（截斷）⇒ 大聲拒絕、維持「快照」、不退回即時；GCP 規劃拿到會報錯的來源
    bad = tmp_path / "bad.json"
    bad.write_bytes(path.read_bytes()[:-40])
    assert dlg.load_plan_snapshot(bad) is False and dlg.plan_source_mode() == "snapshot"
    assert dlg.lbl_plan_snapshot.text().startswith("✘") and "載入失敗" in dlg.lbl_plan_snapshot.text()
    inp, errors = dlg.collect_inputs()
    assert inp is None and "bad.json 載入失敗" in errors[0]
    assert "載入失敗" in (g.blocks()[3] or "") and "載入失敗" in (g.areas()[2] or "")
    g.close()


def test_reopening_the_gcp_planner_picks_up_the_current_live_sources(qt_app):
    """主視窗重用對話框時會換掉即時來源（polygons／gcp_blocks）；再按「GCP／檢核點規劃…」要用新的。"""
    from tests.test_survey_dialog import RING_A, RING_B
    p1 = lambda: ({}, {}, "第一次")                                              # noqa: E731
    p2 = lambda: ({}, {}, "第二次")                                              # noqa: E731
    dlg = SurveyProcessDialog(polygons=[RING_A], gcp_blocks=p1)
    g = dlg.open_gcp_plan()
    assert g.blocks_source is p1 and g.polygons_source == [RING_A]
    dlg.gcp_blocks_source, dlg.polygons_source = p2, [RING_B]
    assert dlg.open_gcp_plan() is g and g.blocks_source is p2 and g.polygons_source == [RING_B]
    g.close()


def test_snapshot_source_reaches_the_pipeline_effective_inputs(qt_app, tmp_path, monkeypatch):
    """處理時的有效值（qa.json effective.input_sources）真的帶著快照來源：跑真的 process_campaign。"""
    from survey import plan_snapshot as ps
    from tests.test_survey_dialog import wait_worker
    from tests.test_survey_pipeline import GROUND_ELL, build_sortie
    from tests.test_survey_plan_snapshot import acked, make_plan, snap_of
    monkeypatch.setenv(ps.ENV_DIR, str(tmp_path / "snap_root"))
    snap = acked(snap_of(*make_plan()), 11)
    path = ps.write_snapshot(tmp_path / "snaps", snap)
    s = build_sortie(tmp_path / "h1", uav="HERO1", sid="S1", seed=1)
    dlg = SurveyProcessDialog()
    dlg.add_sortie_row("", str(s["bin"]), str(s["sess"]))
    dlg.rb_exposure_start.setChecked(True)
    dlg.ed_ground_h.setText(str(GROUND_ELL))
    dlg.ed_out_dir.setText(str(tmp_path / "gui"))
    assert dlg.load_plan_snapshot(path) is True
    assert dlg.start_processing() is True
    wait_worker(dlg)
    assert dlg.result is not None, dlg.lbl_summary.text()
    qa = json.loads(next((tmp_path / "gui").rglob("qa.json")).read_text(encoding="utf-8"))
    eff_src = qa["effective"]["input_sources"]
    assert json.loads(eff_src["plan_snapshot"])["plan_sha256"] == snap["plan_sha256"]
    assert eff_src["plan_source"] == ps.source_label(snap, path)
    assert f"計畫來源：{ps.source_label(snap, path)}" in dlg.lbl_summary.text().splitlines()
