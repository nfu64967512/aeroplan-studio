"""2026-10-20 Step 0 對抗式審查的修正（survey.odm_client／survey.pipeline／survey.gcp_plan），每一條一個數值層級的測試。

全部對真的本機 HTTP 假節點（tests/fake_nodeodx.py）跑；檔案只寫在 tmp_path。沒有一條對真的 NodeODX 跑過。
"""

import json
import shutil
from pathlib import Path

import pytest

from survey import gcp_plan, odm_client, pipeline
from survey.odm_client import NodeOdmClient, OdmLocalError
from survey.pipeline import PipelineError
from tests.fake_nodeodx import FakeNodeODX, default_options
from tests.test_survey_odm_fake_node import (NOOP, TOKEN, client, gcp_file, job_for, make_node, make_result, record,
                                             session)


def files_text(root) -> str:
    return "\n".join(p.read_text(encoding="utf-8", errors="replace") for p in Path(root).rglob("*.json"))


# ─────────────────────────────────────────────────────────────────────────────
# 權杖：短權杖不能把紀錄改壞（審查 high）
# ─────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("tok", ["odm", "aeroplan"])
def test_short_tokens_do_not_corrupt_the_job_record(tmp_path, tok):
    n = FakeNodeODX(tmp_path / "node", token=tok).start()
    try:
        res = make_result(tmp_path)
        c = NodeOdmClient(n.url, token=tok, session=session(), timeout_s=30)
        out = pipeline.run_odm_job(job_for(res, n), client=c, poll_s=0, sleep=NOOP)
        rec = pipeline.load_submission(res.out_dir)                         # 讀得回來（format 沒被改成 ***）
        assert rec["format"] == pipeline.SUBMISSION_FORMAT == "aeroplan-odm-submission/2"
        assert rec["phase"] == "downloaded" and "***" not in json.dumps(rec, ensure_ascii=False)
        assert rec["zip"] == str(res.out_dir / "odm" / f"{out['uuid']}_all.zip")       # 路徑沒被改
        q = json.loads((res.out_dir / "odm" / "quality_summary.json").read_text(encoding="utf-8"))
        assert q["files"]["stats_json"] == "odm_report/stats.json"
        assert f"token={tok}" not in files_text(res.out_dir)                   # URL 形式的權杖仍然不落地
        again = pipeline.resume_odm_job(res.out_dir, client=c, poll_s=0, sleep=NOOP)   # 接回仍然可以
        assert again["phase"] == "downloaded"
    finally:
        n.stop()
    # 自由文字：短權杖不逐字換（否則路徑、字詞被改壞），token=… 照樣遮；長權杖照樣逐字換
    assert odm_client.redact("C:/x/odm/submission.json?token=odm", "odm") == "C:/x/odm/submission.json?token=***"
    assert odm_client.redact(f"x {TOKEN} y", TOKEN) == "x *** y" and len(TOKEN) >= odm_client.REDACT_RAW_MIN_LEN


# ─────────────────────────────────────────────────────────────────────────────
# commit：結果不明 ≠ 沒提交（審查 T4）；沒提交而上傳完整 ⇒ 接回補送 commit（審查 T5）
# ─────────────────────────────────────────────────────────────────────────────

def test_commit_with_unknown_outcome_stays_committing_and_reattach_finishes_it(node, tmp_path):
    res = make_result(tmp_path)
    node.faults["commit_lose_response"] = 1          # 節點收了 commit、回應遺失
    node.after_commit_drop = 2                       # 接著 info 與探測都連不上
    with pytest.raises(PipelineError, match="commit 結果不明") as ei:
        pipeline.run_odm_job(job_for(res, node), client=client(node), poll_s=0, sleep=NOOP)
    assert "接回" in str(ei.value)
    rec = record(res)
    assert rec["phase"] == "committing" and "commit_unknown" in rec and rec["uuid"] in node.tasks
    assert "remote_cleanup" not in rec                                          # 沒有去刪節點上正在跑的任務
    st = pipeline.odm_submission_status(rec)
    assert st["can_reattach"] and st["can_cancel_remote"]
    with pytest.raises(PipelineError, match="沒收尾"):                          # 不會在後面排第二個任務
        pipeline.run_odm_job(job_for(res, node), client=client(node), poll_s=0, sleep=NOOP)
    assert len(node.tasks) == 1
    out = pipeline.resume_odm_job(res.out_dir, client=client(node), poll_s=0, sleep=NOOP)
    assert out["phase"] == "downloaded" and record(res)["commit_confirmed_on_reattach"] is True


def test_commit_not_delivered_is_resent_when_the_upload_is_still_there(node, tmp_path):
    res = make_result(tmp_path)
    node.faults["commit_drop"] = 1                   # commit 沒送達：info 說不存在、探測說上傳還在 ⇒ 再送一次
    out = pipeline.run_odm_job(job_for(res, node), client=client(node), poll_s=0, sleep=NOOP)
    assert out["phase"] == "downloaded"
    assert [r.get("fault") for r in node.requests_to(f"/task/new/commit/{out['uuid']}")] == ["commit_drop", None]


def test_reattach_commits_a_fully_uploaded_task_instead_of_asking_for_a_reupload(node, tmp_path):
    res = make_result(tmp_path)
    node.faults["commit_drop"] = 1                   # commit 沒送達
    node.faults["probe_drop"] = 1                    # 事後探測也連不上 ⇒ 結果不明，留在「提交中」
    with pytest.raises(PipelineError, match="commit 結果不明"):
        pipeline.run_odm_job(job_for(res, node), client=client(node), poll_s=0, sleep=NOOP, batch_max_files=2)
    rec = record(res)
    assert rec["phase"] == "committing" and rec["uuid"] not in node.tasks
    n_up = len(node.requests_to(f"/task/new/upload/{rec['uuid']}"))
    out = pipeline.resume_odm_job(res.out_dir, client=client(node), poll_s=0, sleep=NOOP)
    rec = record(res)
    assert out["phase"] == "downloaded" and rec["committed_on_reattach"] is True
    assert any(e["event"] == "commit_on_reattach" for e in rec["events"])
    # 沒有重傳：上傳請求只多了接回時的一次探測（空主體）
    ups = node.requests_to(f"/task/new/upload/{rec['uuid']}")
    assert len(ups) == n_up + 1 and ups[-1]["files"] == []
    assert rec["images_check"]["ok"] is True
    # 純函式：上傳完整、但 commit 那一步失敗（upload_failed）⇒ 可以接回（補送 commit），原因照實說
    st = pipeline.odm_submission_status(dict(rec, phase="upload_failed"))
    assert st["can_reattach"] and "補送 commit" in st["reason"]
    partial = dict(rec, phase="upload_failed", upload=dict(rec["upload"], batches=rec["upload"]["batches"][:-1]))
    assert pipeline.odm_submission_status(partial)["can_reattach"] is False


def test_cancelling_a_task_whose_commit_never_arrived_is_not_called_lost(node, tmp_path):
    res = make_result(tmp_path)
    node.faults["commit_drop"] = 1
    node.faults["probe_drop"] = 1
    with pytest.raises(PipelineError, match="commit 結果不明"):
        pipeline.run_odm_job(job_for(res, node), client=client(node), poll_s=0, sleep=NOOP)
    assert record(res)["phase"] == "committing"
    r = pipeline.cancel_odm_job(res.out_dir, client=client(node), poll_s=0, sleep=NOOP)
    rec = record(res)
    assert r["phase"] == rec["phase"] == "upload_canceled" and "commit 沒有送達" in rec["failure"]["message"]
    assert "lost" not in [h["phase"] for h in rec["phase_history"]]


# ─────────────────────────────────────────────────────────────────────────────
# 接回：本機已有成果就不重下、也不降級成 lost（審查 T1–T3）
# ─────────────────────────────────────────────────────────────────────────────

def _downloads(node, uid):
    return len(node.requests_to(f"/task/{uid}/download"))


def test_reattach_uses_the_local_zip_after_an_extraction_failure_even_when_the_node_expired(node, tmp_path,
                                                                                            monkeypatch):
    res = make_result(tmp_path)
    real = pipeline.extract_odm_zip

    def enospc(*a, **k):
        raise OSError(28, "No space left on device")
    monkeypatch.setattr(pipeline, "extract_odm_zip", enospc)
    with pytest.raises(PipelineError, match="No space left"):
        pipeline.run_odm_job(job_for(res, node), client=client(node), poll_s=0, sleep=NOOP)
    rec = record(res)
    assert rec["phase"] == "postprocess_failed" and rec["zip_bytes"] > 0
    monkeypatch.setattr(pipeline, "extract_odm_zip", real)
    node.tasks.pop(rec["uuid"])                                                # 節點 48 h 後刪了任務
    out = pipeline.resume_odm_job(res.out_dir, client=client(node), poll_s=0, sleep=NOOP)
    assert out["phase"] == "downloaded" and out["quality"]["stats_available"]
    assert _downloads(node, rec["uuid"]) == 1                                  # 沒有重下


def test_downloaded_record_is_never_downgraded_and_a_moved_folder_is_not_redownloaded(node, tmp_path):
    res = make_result(tmp_path)
    out = pipeline.run_odm_job(job_for(res, node), client=client(node), poll_s=0, sleep=NOOP)
    uid = out["uuid"]
    moved = tmp_path / "moved"
    shutil.move(str(res.out_dir), str(moved))                                  # 專案資料夾搬了（紀錄的絕對路徑失效）
    again = pipeline.resume_odm_job(moved, client=client(node), poll_s=0, sleep=NOOP)
    assert again["phase"] == "downloaded" and _downloads(node, uid) == 1
    node.tasks.pop(uid)                                                        # 節點過期
    (moved / "odm" / f"{uid}_all.zip").unlink()                                # 操作員刪了 zip、留著解壓的成果
    again = pipeline.resume_odm_job(moved, client=client(node), poll_s=0, sleep=NOOP)
    rec = pipeline.load_submission(moved)
    assert again["phase"] == rec["phase"] == "downloaded" and rec["zip"] is None
    assert any("只剩解壓後" in w for w in again["warnings"])
    shutil.rmtree(moved / "odm" / f"result_{uid}")                             # 連解壓的也刪了：照實說，但不降級
    with pytest.raises(PipelineError, match="成果檔被搬走或刪掉"):
        pipeline.resume_odm_job(moved, client=client(node), poll_s=0, sleep=NOOP)
    assert pipeline.load_submission(moved)["phase"] == "downloaded"


def test_crash_during_extraction_reuses_the_downloaded_zip(node, tmp_path, monkeypatch):
    res = make_result(tmp_path)

    class Crash(BaseException):
        pass

    def crash(*a, **k):
        raise Crash()
    monkeypatch.setattr(pipeline, "extract_odm_zip", crash)
    with pytest.raises(Crash):
        pipeline.run_odm_job(job_for(res, node), client=client(node), poll_s=0, sleep=NOOP)
    monkeypatch.undo()
    rec = record(res)
    assert rec["phase"] == "completed" and rec["zip_bytes"] > 0
    out = pipeline.resume_odm_job(res.out_dir, client=client(node), poll_s=0, sleep=NOOP)
    assert out["phase"] == "downloaded" and _downloads(node, rec["uuid"]) == 1


# ─────────────────────────────────────────────────────────────────────────────
# 提交後核對不過、取消也失敗 ⇒ abort_pending（審查 T6）
# ─────────────────────────────────────────────────────────────────────────────

def _run_into_abort_pending(node, tmp_path, name):
    res = make_result(tmp_path, name=name)
    armed = []

    def on_state(st):
        if (st.get("upload") or {}).get("batch") == 2 and not armed:          # 第一批影像送完就丟回應 ⇒ 重複檔
            armed.append("dup")
            node.faults["upload_lose_response"] = 1
        if st.get("status_code") == 10 and armed == ["dup"]:                  # 初始化完成：接著 cancel 與 remove 都斷線
            armed.append("drop")
            node.faults["drop_any"] = 2
    with pytest.raises(PipelineError, match="都沒成功"):
        pipeline.run_odm_job(job_for(res, node), client=client(node), poll_s=0, sleep=NOOP, batch_max_files=2,
                             on_state=on_state)
    return res


def test_failed_abort_is_recorded_as_pending_and_retried(node, tmp_path):
    res = _run_into_abort_pending(node, tmp_path, "a")
    rec = record(res)
    assert rec["phase"] == "abort_pending" and rec["failure"]["kind"] == "images_count_mismatch"
    assert rec["failure"]["pending_abort"] is True and rec["uuid"] in node.tasks
    st = pipeline.odm_submission_status(rec)
    assert st["can_cancel_remote"] and st["can_reattach"] and "再試一次取消" in st["reason"]
    with pytest.raises(PipelineError, match="沒收尾"):
        pipeline.run_odm_job(job_for(res, node), client=client(node), poll_s=0, sleep=NOOP)
    # 接回 ⇒ 再試一次取消＋刪除（不下載）；成功後 failed，原本的原因留著
    with pytest.raises(PipelineError, match="已取消並刪除"):
        pipeline.resume_odm_job(res.out_dir, client=client(node), poll_s=0, sleep=NOOP)
    rec = record(res)
    assert rec["phase"] == "failed" and rec["failure"]["kind"] == "images_count_mismatch"
    assert rec["failure"]["pending_abort"] is False and rec["uuid"] not in node.tasks
    assert any(a.startswith("重試：cancel: ok") for a in rec["failure"]["remote_actions"])
    # 取消遠端任務這條路也一樣
    res2 = _run_into_abort_pending(node, tmp_path, "b")
    r = pipeline.cancel_odm_job(res2.out_dir, client=client(node), poll_s=0, sleep=NOOP)
    assert r["phase"] == "failed" and record(res2)["uuid"] not in node.tasks


# ─────────────────────────────────────────────────────────────────────────────
# 附檔批：主體送完後的 5xx 不重送（審查 T8）
# ─────────────────────────────────────────────────────────────────────────────

def test_a_5xx_after_the_aux_batch_was_stored_is_not_retried(node, tmp_path):
    res = make_result(tmp_path)
    node.faults["upload_502_after_store"] = 1
    with pytest.raises(PipelineError, match="不能重送") as ei:
        pipeline.run_odm_job(job_for(res, node, gcp_txt=gcp_file(tmp_path, res), gcp_vertical="ellipsoidal",
                                     force_gps=True), client=client(node), poll_s=0, sleep=NOOP)
    assert "HTTP 502" in str(ei.value)
    rec = record(res)
    assert rec["phase"] == "upload_failed" and rec["failure"]["ambiguous"] is True
    assert node.tmp_files(rec["uuid"]) == ["gcp_list.txt", "geo.txt"]          # 沒有 geo_.txt／gcp_list_.txt
    # 影像批照舊：5xx 可以重送（節點自己在存檔前回的 500）
    c = client(node)
    import uuid as _u
    uid = str(_u.uuid4())
    c.init_task(uid, options=[])
    node.faults["upload_500"] = 1
    reps = c.upload_files(uid, [(p, p.name) for p in res.images[:2]], sleep=NOOP)
    assert reps[0]["attempts"] == 2


# ─────────────────────────────────────────────────────────────────────────────
# maxImages 連 .txt 一起算（R1 §2.3）
# ─────────────────────────────────────────────────────────────────────────────

def test_max_images_counts_the_txt_files(tmp_path):
    n = make_node(tmp_path, "n1", max_images=6)                                 # 6 張影像剛好，加 geo.txt 就超過
    try:
        res = make_result(tmp_path)
        with pytest.raises(PipelineError, match=r"共 7 個檔.*R1 §2\.3"):
            pipeline.run_odm_job(job_for(res, n), client=client(n), poll_s=0, sleep=NOOP)
        assert record(res)["failure"]["kind"] == "max_images" and n.requests_to("/task/new") == []
        # 假節點本身照規格：commit 後初始化時連 .txt 一起數 ⇒ 30 FAILED
        c = client(n)
        uid = c.new_task(res.images, options=[], geo_txt=res.paths["geo_txt"], sleep=NOOP)
        c.task_info(uid)
        info = c.task_info(uid)
        assert info["status"]["code"] == 30 and "7 images uploaded" in info["status"]["errorMessage"]
    finally:
        n.stop()


# ─────────────────────────────────────────────────────────────────────────────
# 下載：磁碟空間不足、本機寫檔錯誤 ≠ 連線中斷（審查 T10）
# ─────────────────────────────────────────────────────────────────────────────

def test_disk_space_is_checked_before_download_and_before_extraction(node, tmp_path, monkeypatch):
    res = make_result(tmp_path)
    monkeypatch.setattr(odm_client, "free_bytes", lambda p: 10)                 # 只剩 10 B
    with pytest.raises(PipelineError, match="磁碟空間不足") as ei:
        pipeline.run_odm_job(job_for(res, node), client=client(node), poll_s=0, sleep=NOOP)
    assert "接回任務" in str(ei.value)
    rec = record(res)
    assert rec["phase"] == "completed" and rec["download_error"]["kind"] == "local"
    assert not list((res.out_dir / "odm").glob("*.part")) and not list((res.out_dir / "odm").glob("*_all.zip"))
    # 下載夠、解壓不夠：解壓前就擋，不留半套 result_<uuid>/，zip 留著
    calls = []

    def free(p):
        calls.append(p)
        return 10 ** 12 if len(calls) == 1 else 10
    monkeypatch.setattr(odm_client, "free_bytes", free)
    with pytest.raises(PipelineError, match="解壓.*只剩"):
        pipeline.resume_odm_job(res.out_dir, client=client(node), poll_s=0, sleep=NOOP)
    rec = record(res)
    assert rec["phase"] == "postprocess_failed" and (res.out_dir / "odm" / f"{rec['uuid']}_all.zip").is_file()
    assert not (res.out_dir / "odm" / f"result_{rec['uuid']}").exists()
    monkeypatch.setattr(odm_client, "free_bytes", lambda p: None)
    out = pipeline.resume_odm_job(res.out_dir, client=client(node), poll_s=0, sleep=NOOP)
    # 下載請求共 2 次：第一次看到 Content-Length 就因空間不足放棄（沒寫檔），第二次真的下載；最後這次用本機 zip
    assert out["phase"] == "downloaded" and _downloads(node, rec["uuid"]) == 2


def test_a_local_write_error_is_not_reported_as_a_broken_connection(node, tmp_path, monkeypatch):
    c = client(node)
    import uuid as _u
    uid = str(_u.uuid4())
    res = make_result(tmp_path)
    c.init_task(uid, options=[])
    c.upload_files(uid, [(p, p.name) for p in res.images[:1]], sleep=NOOP)
    c.commit_task(uid)
    for _ in range(8):
        c.task_info(uid)

    class Full:
        def write(self, data):
            raise OSError(28, "No space left on device")

        def close(self):
            pass
    monkeypatch.setattr(odm_client, "open", lambda *a, **k: Full(), raising=False)
    with pytest.raises(OdmLocalError, match="寫入本機.*No space left.*接回任務") as ei:
        c.download_all(uid, tmp_path / "dl" / "all.zip")
    assert "下載中斷" not in str(ei.value) and not isinstance(ei.value, odm_client.OdmTransportError)


# ─────────────────────────────────────────────────────────────────────────────
# 送出前：任務內容與紀錄一致（審查 contract）
# ─────────────────────────────────────────────────────────────────────────────

def test_run_odm_job_refuses_a_job_whose_gcp_or_force_gps_was_changed_after_odm_job(node, tmp_path):
    res = make_result(tmp_path)
    j = job_for(res, node)
    j["gcp_txt"] = str(gcp_file(tmp_path, res))                                 # odm_job 之後才塞 GCP
    with pytest.raises(PipelineError, match="不一致.*not_applicable_no_gcp"):
        pipeline.run_odm_job(j, client=client(node), poll_s=0, sleep=NOOP)
    j2 = job_for(res, node, gcp_txt=gcp_file(tmp_path, res), gcp_vertical="ellipsoidal", force_gps=True)
    j2["options"] = [o for o in j2["options"] if o["name"] != "force-gps"]      # 決定是 force_gps，參數卻沒送
    with pytest.raises(PipelineError, match="沒有 force-gps"):
        pipeline.run_odm_job(j2, client=client(node), poll_s=0, sleep=NOOP)
    j3 = job_for(res, node)
    del j3["force_gps"]
    with pytest.raises(PipelineError, match="沒有 force_gps 的決定"):
        pipeline.run_odm_job(j3, client=client(node), poll_s=0, sleep=NOOP)
    assert node.log == []                                                       # 一個請求都沒送
    c = client(node)
    with pytest.raises(ValueError, match="必須明確給 force_gps.*沒有預設"):
        c.new_task(res.images, options=[], geo_txt=res.paths["geo_txt"], gcp_txt=gcp_file(tmp_path, res))
    with pytest.raises(ValueError, match="一致"):
        c.new_task(res.images, options=[], geo_txt=res.paths["geo_txt"], gcp_txt=gcp_file(tmp_path, res),
                   force_gps=True)


# ─────────────────────────────────────────────────────────────────────────────
# geo.txt 精度欄（審查 contract）
# ─────────────────────────────────────────────────────────────────────────────

def test_geo_txt_accuracy_columns_must_be_finite_and_positive(tmp_path, node):
    for bad in ("nan nan", "0.000 0.000", "-1 -1", "x y"):
        res = make_result(tmp_path, name=f"r{abs(hash(bad)) % 10**6}")
        geo = res.paths["geo_txt"]
        lines = geo.read_text(encoding="utf-8").splitlines()
        parts = lines[2].split()
        lines[2] = " ".join(parts[:7] + bad.split())
        geo.write_text("\n".join(lines) + "\n", encoding="utf-8")
        with pytest.raises(PipelineError, match="精度欄不是有限的正數"):
            job_for(res, node)
    # 精度欄的意義跟著處理結果走；沒記的舊結果 ⇒ 照實記成舊慣例，送出時警告
    res = make_result(tmp_path, name="legacy")
    res.effective.pop("geo_txt_accuracy")
    j = job_for(res, node)
    assert j["geo_txt_check"]["accuracy_semantics"]["recorded"] is False
    out = pipeline.run_odm_job(j, client=client(node), poll_s=0, sleep=NOOP)
    assert any("二維 HRMS" in w for w in out["warnings"])
    res = make_result(tmp_path, name="current")
    j = job_for(res, node)
    assert j["gps_accuracy"]["semantics"]["horizontal"] == "sigma_1d_from_hrms"
    # 只有部分影像有精度欄 ⇒ 其餘用引擎預設 3 m：送出時警告（不只是記錄）
    geo = res.paths["geo_txt"]
    lines = geo.read_text(encoding="utf-8").splitlines()
    lines[1] = " ".join(lines[1].split()[:7])
    geo.write_text("\n".join(lines) + "\n", encoding="utf-8")
    j = job_for(res, node)
    assert j["geo_txt_check"]["accuracy"] == "partial" and j["gps_accuracy"]["source"] == "geo.txt+engine_default"
    out = pipeline.run_odm_job(j, client=client(node), poll_s=0, sleep=NOOP)
    assert any("只有 5/6 張帶每張的精度欄" in w for w in out["warnings"])


def test_images_json_is_checked_against_geo_txt_and_its_absence_is_reported(tmp_path):
    n = make_node(tmp_path, "n1", images_json=False)
    try:
        res = make_result(tmp_path, name="a")
        out = pipeline.run_odm_job(job_for(res, n), client=client(n), poll_s=0, sleep=NOOP)
        assert any("找不到 images.json" in w for w in out["quality"]["warnings"])
    finally:
        n.stop()
    n = make_node(tmp_path, "n2")
    try:
        res = make_result(tmp_path, name="b")
        geo = res.paths["geo_txt"]
        geo.write_text(geo.read_text(encoding="utf-8").replace(" 0.030 0.050", " 0.100 0.200"), encoding="utf-8")
        out = pipeline.run_odm_job(job_for(res, n), client=client(n), poll_s=0, sleep=NOOP)
        chk = out["quality"]["images_json"]["geo_txt_check"]
        assert chk["n_checked"] == 6 and chk["n_mismatch"] == 6
        assert any("GPS 精度與 geo.txt 不同" in w for w in out["quality"]["warnings"])
    finally:
        n.stop()
    q = pipeline.read_odm_quality(tmp_path / "nothing", gps_accuracy={"source": "geo.txt"})
    assert any("task_output.txt" in w for w in q["warnings"]) and any("images.json" in w for w in q["warnings"])


# ─────────────────────────────────────────────────────────────────────────────
# CHK- 點、引擎有沒有真的用 GCP、引擎自己開 force-gps、偏差補償沒生效（審查 contract）
# ─────────────────────────────────────────────────────────────────────────────

def _gcp_with(tmp_path, res, labels) -> Path:
    g = tmp_path / "gcp_chk.txt"
    names = [p.name for p in res.images]
    rows = []
    for k, lab in enumerate(labels):
        for im in names[:5]:
            rows.append(f"{121.2 + k * 1e-3:.6f} 25.100000 {48.5 + k:.3f} {10 + k} {20 + k} {im} {lab}")
    g.write_text("EPSG:4326\n" + "\n".join(rows) + "\n", encoding="utf-8")
    return g


def test_chk_prefixed_labels_are_engine_checkpoints_not_gcps(tmp_path, node):
    res = make_result(tmp_path)
    names = [p.name for p in res.images]
    only = gcp_plan.validate_gcp_list(_gcp_with(tmp_path, res, ["CHK-01", "CHK-02"]), image_names=names,
                                      gcp_vertical="ellipsoidal", height="ellipsoidal")
    assert only["n_gcps"] == 0 and any("全部以 CHK- 開頭" in e for e in only["errors"])
    with pytest.raises(PipelineError, match="全部以 CHK- 開頭"):
        job_for(res, node, gcp_txt=_gcp_with(tmp_path, res, ["CHK-01", "CHK-02"]), gcp_vertical="ellipsoidal",
                force_gps=False)
    low = gcp_plan.validate_gcp_list(_gcp_with(tmp_path, res, ["G1", "chk-02"]), image_names=names,
                                     gcp_vertical="ellipsoidal", height="ellipsoidal")
    assert any("小寫" in e for e in low["errors"])
    mix = gcp_plan.validate_gcp_list(_gcp_with(tmp_path, res, ["G1", "G2", "CHK-03"]), image_names=names,
                                     gcp_vertical="ellipsoidal", height="ellipsoidal")
    assert mix["errors"] == [] and mix["n_gcps"] == 2 and mix["engine_checkpoints"] == ["CHK-03"]
    assert any("CHK- 開頭" in w for w in mix["warnings"])


def test_quality_report_flags_unused_gcps_engine_force_gps_and_identity_bias(tmp_path):
    root = tmp_path / "r"
    (root / "odm_report").mkdir(parents=True)
    stats = FakeNodeODX(tmp_path / "n").stats_json({"images": [f"{k}.jpg" for k in range(6)]})   # 沒有 GCP
    (root / "odm_report" / "stats.json").write_text(json.dumps(stats), encoding="utf-8")
    (root / "log.json").write_text(json.dumps({"engine": "ODX", "version": "3.8.4", "options": {"force_gps": False},
                                               "stages": [{"name": "dataset", "messages": [
                                                   {"message": "GCP file only has checkpoints, enabling --force-gps",
                                                    "type": "warning"}]}]}), encoding="utf-8")
    q = pipeline.read_odm_quality(root, n_uploaded=6, force_gps={"decision": "gcp_only"}, gcp_uploaded=True)
    assert any("has_gcp" in w for w in q["warnings"]) and any("gcp_only" in w for w in q["warnings"])
    assert any(w.startswith("引擎警告：GCP file only has checkpoints") for w in q["warnings"])
    assert q["force_gps_overridden_by_engine"] is True
    rows = pipeline.odm_quality_rows(q)
    assert any(r[1].startswith("照片位置（geo.txt）對解算位置（⚠ 引擎可能自己開了 --force-gps") for r in rows)
    # --force-gps 但相機偏差是單位轉換 ⇒ 不能宣稱已扣偏差
    stats["camera_errors"]["v2 sony ilce-6100 6000 4000 brown 0.6889"]["bias"]["translation"] = [0.0, 0.0, 0.0]
    (root / "odm_report" / "stats.json").write_text(json.dumps(stats), encoding="utf-8")
    q = pipeline.read_odm_quality(root, n_uploaded=6, force_gps={"decision": "force_gps"})
    assert q["camera_bias_identity"] is True and any("偏差補償可能沒有生效" in w for w in q["warnings"])
    rows = pipeline.odm_quality_rows(q)
    assert any("偏差補償可能沒生效" in r[1] for r in rows) and not any("已扣掉每台相機的偏差" in r[1] for r in rows)


# ─────────────────────────────────────────────────────────────────────────────
# 突變測試抓不到的兩條守門（審查 M5、M9）
# ─────────────────────────────────────────────────────────────────────────────

def test_an_option_dropped_by_the_node_aborts_the_task(tmp_path):
    n = make_node(tmp_path, "n", options=[o for o in default_options() if o["name"] != "force-gps"])
    n.faults["options_502"] = 1                                                 # 送出前的 /options 核對取不到
    try:
        res = make_result(tmp_path)
        job = job_for(res, n, gcp_txt=gcp_file(tmp_path, res), gcp_vertical="ellipsoidal", force_gps=True)
        with pytest.raises(PipelineError, match=r"節點丟掉了送出的參數 \['force-gps'\]"):
            pipeline.run_odm_job(job, client=client(n), poll_s=0, sleep=NOOP)
        rec = record(res)
        assert rec["engine_options"]["available"] is False
        assert rec["phase"] == "failed" and rec["failure"]["kind"] == "options_dropped"
        assert rec["options_check"]["dropped"] == ["force-gps"]
        assert rec["failure"]["remote_actions"] == ["cancel: ok", "remove: ok"] and rec["uuid"] not in n.tasks
    finally:
        n.stop()


def test_unknown_engine_version_is_warned_in_the_record(tmp_path):
    n = make_node(tmp_path, "n", engine_version="?")
    try:
        res = make_result(tmp_path)
        out = pipeline.run_odm_job(job_for(res, n), client=client(n), poll_s=0, sleep=NOOP)
        assert any("引擎版本未知" in w for w in record(res)["warnings"]) and any("引擎版本未知" in w for w in out["warnings"])
    finally:
        n.stop()


@pytest.fixture
def node(tmp_path):
    n = FakeNodeODX(tmp_path / "node", token=TOKEN).start()
    yield n
    n.stop()
