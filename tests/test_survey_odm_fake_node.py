"""survey.pipeline 的 NodeODX 流程：對真的本機 HTTP 假節點（tests/fake_nodeodx.py）跑 run／resume／cancel。

涵蓋：任務紀錄每個階段、權杖從不落地、進度值、提交後核對（張數、參數被丟／被改）、遠端失敗的主控台尾巴、
force_gps 三態、max_concurrency 記錄、模稜兩可的上傳、取消（提交前／提交後）、當機後在各階段接回、
品質報告（含 stats.json 缺檔／壞檔）。
"""

import json
import shutil
from collections import Counter
from pathlib import Path
from urllib.parse import urlsplit

import pytest
import requests

from survey import geotag, pipeline
from survey.odm_client import NodeOdmClient
from survey.pipeline import CampaignResult, PipelineCanceled, PipelineError
from tests.fake_nodeodx import FakeNodeODX, default_options

TOKEN = "tok-Zq9_S3CRET-4711"


def NOOP(_s):
    return None


def session():
    s = requests.Session()
    s.trust_env = False
    return s


@pytest.fixture
def node(tmp_path):
    n = FakeNodeODX(tmp_path / "node", token=TOKEN).start()
    yield n
    n.stop()


def make_node(tmp_path, name, **kw):
    return FakeNodeODX(tmp_path / name, token=TOKEN, **kw).start()


def client(node, s=None):
    return NodeOdmClient(node.url, token=TOKEN, session=s or session(), timeout_s=30)


def make_result(tmp_path, *, n=6, size=40_000, accuracy=True, name="res") -> CampaignResult:
    """最小的處理結果：n 張加標籤影像＋每張都有一行的 geo.txt（含每張的精度欄）。"""
    out = tmp_path / name
    img_dir = out / "odm" / "images"
    img_dir.mkdir(parents=True)
    imgs, lines = [], ["EPSG:4326"]
    for k in range(n):
        nm = f"HERO1_S1_{k + 1:06d}.jpg"
        p = img_dir / nm
        p.write_bytes(b"\xff\xd8" + bytes([k % 256]) * size)
        imgs.append(p)
        lines.append(f"{nm} {121.2 + k * 1e-4:.9f} 25.100000000 150.000 95.000 0.000 0.000"
                     + (" 0.030 0.050" if accuracy else ""))
    geo = out / "odm" / "geo.txt"
    geo.write_text("\n".join(lines) + "\n", encoding="utf-8")
    # 精度欄是單軸 σ（現行 geotag 的寫法）⇒ 處理結果照實記下（odm_job 帶進任務紀錄；沒記的是舊結果、會被警告）
    return CampaignResult(out_dir=out, sorties=[], rows=[], paths={"geo_txt": geo, "images_dir": img_dir}, images=imgs,
                          effective={"height": "ellipsoidal", "geo_txt_accuracy": dict(geotag.ACCURACY_SEMANTICS)},
                          qa={"passed": True, "reasons": []},
                          odm={"rolling_shutter_readout_ms": None, "rolling_shutter_error": "沒有會期", "gsd_cm": 2.5,
                               "n_images": n})


def gcp_file(tmp_path, res) -> Path:
    g = tmp_path / "gcp_list.txt"
    names = [p.name for p in res.images]
    g.write_text("EPSG:4326\n" + "".join(f"121.2 25.1 48.5 {10 + k} {20 + k} {names[k]} GCP01\n"
                                         for k in range(min(5, len(names)))), encoding="utf-8")
    return g


def job_for(res, node, **kw):
    kw.setdefault("orthophoto_resolution_cm", 2.5)
    kw.setdefault("rolling_shutter", False)
    j = pipeline.odm_job(res, node_url=node.url, **kw)
    j["orthophoto_resolution_source"] = "user"                       # 對話框會加的欄位，一起進紀錄
    return j


def record(res) -> dict:
    return json.loads((res.out_dir / "odm" / "submission.json").read_text(encoding="utf-8"))


def assert_no_token(*roots):
    for root in roots:
        for p in Path(root).rglob("*"):
            if p.is_file():
                assert TOKEN.encode() not in p.read_bytes(), p


def phases(rec) -> list:
    return [h["phase"] for h in rec["phase_history"]]


def is_subsequence(want, have) -> bool:
    it = iter(have)
    return all(w in it for w in want)


# ─────────────────────────────────────────────────────────────────────────────
# 完整流程
# ─────────────────────────────────────────────────────────────────────────────

def test_full_run_records_every_phase_reports_progress_and_never_writes_the_token(node, tmp_path):
    res = make_result(tmp_path)
    job = job_for(res, node, gcp_txt=gcp_file(tmp_path, res), gcp_vertical="ellipsoidal", force_gps=True,
                  max_concurrency=10)
    prog, states = [], []
    out = pipeline.run_odm_job(job, client=client(node), poll_s=0, sleep=NOOP, batch_max_files=2,
                               progress=lambda m, f: prog.append((m, f)), on_state=states.append)
    rec = record(res)
    # 階段
    assert rec["format"] == pipeline.SUBMISSION_FORMAT and rec["phase"] == "downloaded"
    assert is_subsequence(["planned", "initialized", "uploading", "uploaded", "committing", "committed", "running",
                           "completed", "downloaded"], phases(rec))
    # 節點、上傳批次（附檔自己一批，影像每批 ≤ 2）
    assert rec["node"]["engineVersion"] == "3.8.4" and rec["node"]["version"] == "2.4.0"
    assert rec["node_url"] == node.url and rec["token_used"] is True
    uploads = [r["files"] for r in node.requests_to(f"/task/new/upload/{out['uuid']}")]
    assert uploads[0] == ["geo.txt", "gcp_list.txt"] and [len(u) for u in uploads[1:]] == [2, 2, 2]
    assert [b["n_files"] for b in rec["upload"]["batches"]] == [2, 2, 2, 2]
    assert rec["upload"]["n_batches_planned"] == 4 and rec["upload"]["ambiguous_retries"] == 0
    assert rec["upload"]["bytes_total"] == sum(p.stat().st_size for p in res.images) + sum(
        a["bytes"] for a in rec["upload"]["aux_files"])
    # 選項：送了什麼、沒送什麼、節點改了什麼——都有來源
    assert rec["force_gps"]["decision"] == "force_gps" and rec["force_gps"]["source"] == "operator"
    assert rec["max_concurrency"] == {**rec["max_concurrency"], "sent": True, "value": 10, "source": "operator"}
    assert rec["gps_accuracy"]["sent"] is False and rec["gps_accuracy"]["source"] == "geo.txt"
    assert rec["orthophoto_resolution_source"] == "user"
    srcs = rec["option_sources"]
    assert srcs["orthophoto-resolution"] == {"sent": True, "value": 2.5, "source": "user"}
    assert srcs["force-gps"]["source"] == "operator" and srcs["max-concurrency"]["value"] == 10
    assert srcs["gps-accuracy"]["sent"] is False and srcs["gps-accuracy"]["source"] == "geo.txt"
    assert rec["options_check"]["dropped"] == [] and rec["options_check"]["changed"] == []
    assert {a["name"] for a in rec["options_check"]["added"]} == {"pc-ept", "cog", "gltf"}
    assert rec["engine_options"]["defaults"]["force-gps"]["default"] == "false"
    assert rec["images_check"] == {**rec["images_check"], "ok": True, "expected": 6, "reported": 6}
    ea = rec["engine_args"]
    assert ea["final"] and ea["mismatches"] == [] and ea["engine_line"] == "ODX 3.8.4"
    assert ea["observed"]["force_gps"] == "True" and ea["observed"]["gcp"].endswith("/gcp_list.txt")
    assert ea["observed"]["max_concurrency"] == "10"
    # 成果與品質報告
    assert out["ortho_tiles"].is_dir() and out["dsm"].name == "dsm.tif" and out["phase"] == "downloaded"
    q = out["quality"]
    assert q["stats_available"] and q["images"]["uploaded"] == 6 and q["images"]["not_reconstructed"] == 1
    assert q["gps_errors"]["rms"] == {"x": 0.024, "y": 0.021, "z": 0.054} and q["gsd_cm"] == 2.71
    assert q["gcp"]["gcp_only"]["rms"]["z"] == 0.043 and q["engine"]["version"] == "3.8.4"
    assert any("GSD 壓低" in w for w in q["warnings"]) and any("沒有重建" in w for w in q["warnings"])
    assert json.loads((res.out_dir / "odm" / "quality_summary.json").read_text(encoding="utf-8"))["uuid"] == out["uuid"]
    rows = out["quality_rows"]
    assert ("ODM", "模型分塊數", "1", True) in rows and any(r[1].startswith("相機偏差") for r in rows)
    # 進度：0–1、遞增、最後 1.0；上傳段 ≤ 0.30、節點處理段在 0.30–0.90
    fr = [f for _, f in prog if f is not None]
    assert all(0.0 <= f <= 1.0 for f in fr) and fr == sorted(fr) and fr[-1] == 1.0
    assert any(0.0 < f < 0.30 for f in fr) and any(0.30 < f < 0.90 for f in fr)
    assert any(s.get("upload", {}).get("files_done") == 8 for s in states)
    assert any(s.get("status_code") == 20 and s.get("remote_progress") for s in states)
    # 權杖：真的送出去了、但任何檔案都沒有
    assert all(r["query"].get("token") == TOKEN for r in node.log)
    assert_no_token(res.out_dir)
    assert "token" not in rec["node_url"]


def test_wrong_or_missing_token_fails_before_anything_is_uploaded(node, tmp_path):
    res = make_result(tmp_path)
    job = job_for(res, node)
    with pytest.raises(PipelineError, match="Invalid authentication token"):
        pipeline.run_odm_job(job, client=NodeOdmClient(node.url, session=session()), poll_s=0, sleep=NOOP)
    assert record(res)["phase"] == "upload_failed" and node.requests_to("/task/new") == []
    with pytest.raises(PipelineError, match="位址"):
        pipeline.odm_job(res, node_url=f"{node.url}/?token={TOKEN}", orthophoto_resolution_cm=2.5,
                         rolling_shutter=False)


def test_max_images_and_unknown_option_names_are_refused_before_upload(tmp_path):
    n1 = make_node(tmp_path, "n1", max_images=3)
    try:
        res = make_result(tmp_path, name="r1")
        with pytest.raises(PipelineError, match="最多只處理 3 張"):
            pipeline.run_odm_job(job_for(res, n1), client=client(n1), poll_s=0, sleep=NOOP)
        assert record(res)["phase"] == "upload_failed" and n1.requests_to("/task/new") == []
    finally:
        n1.stop()
    n2 = make_node(tmp_path, "n2", options=[o for o in default_options() if o["name"] != "force-gps"])
    try:
        res = make_result(tmp_path, name="r2")
        job = job_for(res, n2, gcp_txt=gcp_file(tmp_path, res), gcp_vertical="ellipsoidal", force_gps=True)
        with pytest.raises(PipelineError, match="不認得.*force-gps"):
            pipeline.run_odm_job(job, client=client(n2), poll_s=0, sleep=NOOP)
        assert record(res)["failure"]["kind"] == "options_unknown" and n2.requests_to("/task/new") == []
    finally:
        n2.stop()


def test_force_gps_tristate_and_max_concurrency_recording(tmp_path):
    n = make_node(tmp_path, "node", node_max_concurrency=12)
    try:
        res = make_result(tmp_path)
        g = gcp_file(tmp_path, res)
        with pytest.raises(PipelineError, match="force-gps"):                        # GCP + 未決定 ⇒ 不組任務
            job_for(res, n, gcp_txt=g, gcp_vertical="ellipsoidal")
        j = job_for(res, n, gcp_txt=g, gcp_vertical="ellipsoidal", force_gps=False)
        assert j["force_gps"]["decision"] == "gcp_only" and all(o["name"] != "force-gps" for o in j["options"])
        j0 = job_for(res, n, force_gps=True)                                          # 沒 GCP：不適用、不送，但記下操作員的值
        assert j0["force_gps"] == {**j0["force_gps"], "decision": "not_applicable_no_gcp", "sent": False,
                                   "operator_choice": True}
        assert all(o["name"] != "force-gps" for o in j0["options"])
        for bad in (0, -3, True, 2.5):
            with pytest.raises(PipelineError, match="max_concurrency"):
                job_for(res, n, max_concurrency=bad)
        with pytest.raises(PipelineError, match="force_gps"):
            job_for(res, n, gcp_txt=g, gcp_vertical="ellipsoidal", force_gps="yes")
        # max_concurrency 沒設：不送、記成引擎預設；節點有上限就自己補上 ⇒ 記成 node auto-added
        out = pipeline.run_odm_job(j, client=client(n), poll_s=0, sleep=NOOP)
        rec = record(res)
        assert rec["max_concurrency"]["sent"] is False and rec["max_concurrency"]["source"] == "engine_default"
        assert {"name": "max-concurrency", "value": 12} in rec["options_check"]["added"]
        assert rec["engine_args"]["observed"]["force_gps"] == "False" and out["warnings"] == [
            w for w in out["warnings"] if w.startswith("品質報告")]
        # 設了 20、節點上限 12 ⇒ 被靜默壓低：記成 node-capped 並警告
        j20 = job_for(res, n, gcp_txt=g, gcp_vertical="ellipsoidal", force_gps=False, max_concurrency=20)
        out = pipeline.run_odm_job(j20, client=client(n), poll_s=0, sleep=NOOP)
        rec = record(res)
        assert rec["options_check"]["changed"] == [{"name": "max-concurrency", "sent": 20, "node": 12}]
        assert any("max-concurrency" in w and "12" in w for w in out["warnings"])
    finally:
        n.stop()


def test_geo_txt_must_cover_every_image(tmp_path, node):
    res = make_result(tmp_path)
    lines = res.paths["geo_txt"].read_text(encoding="utf-8").splitlines()
    res.paths["geo_txt"].write_text("\n".join(lines[:-1]) + "\n", encoding="utf-8")
    with pytest.raises(PipelineError, match="geo.txt 缺 1 張"):
        job_for(res, node)
    res2 = make_result(tmp_path, name="noacc", accuracy=False)
    j = job_for(res2, node)
    assert j["gps_accuracy"]["source"] == "engine_default" and j["geo_txt_check"]["accuracy"] == "none"


# ─────────────────────────────────────────────────────────────────────────────
# 失敗、模稜兩可、取消
# ─────────────────────────────────────────────────────────────────────────────

def test_remote_failure_keeps_the_console_tail(tmp_path):
    n = make_node(tmp_path, "node", fail_task=True)
    try:
        res = make_result(tmp_path)
        with pytest.raises(PipelineError, match="Cannot process dataset") as ei:
            pipeline.run_odm_job(job_for(res, n), client=client(n), poll_s=0, sleep=NOOP)
        assert "ValueError: not enough images" in str(ei.value) and TOKEN not in str(ei.value)
        rec = record(res)
        assert rec["phase"] == "failed" and rec["failure"]["kind"] == "remote"
        tail = rec["failure"]["console_tail"]
        assert len(tail) == pipeline.ODM_CONSOLE_TAIL_LINES and tail[-1].startswith("=====")
        st = pipeline.odm_submission_status(rec)
        assert st["can_reattach"] is False and "重新送出" in st["reason"]
        assert_no_token(res.out_dir)
    finally:
        n.stop()


def test_lost_response_on_an_image_batch_is_caught_by_the_images_count_check(node, tmp_path):
    res = make_result(tmp_path)
    armed = []

    def on_state(st):
        if (st.get("upload") or {}).get("batch") == 2 and not armed:      # 第 2 批（第一批影像）送完就丟回應
            armed.append(1)
            node.faults["upload_lose_response"] = 1
    with pytest.raises(PipelineError, match="影像數 8 不等於上傳的 6 張") as ei:
        pipeline.run_odm_job(job_for(res, node), client=client(node), poll_s=0, sleep=NOOP, batch_max_files=2,
                             on_state=on_state)
    assert "送完沒回應" in str(ei.value)
    rec = record(res)
    assert rec["phase"] == "failed" and rec["failure"]["kind"] == "images_count_mismatch"
    assert rec["upload"]["ambiguous_retries"] == 1 and rec["failure"]["remote_actions"] == ["cancel: ok", "remove: ok"]
    assert rec["uuid"] not in node.tasks                                       # 已刪除，不會用錯的輸入跑幾小時


def test_lost_response_on_the_aux_batch_fails_and_says_the_upload_cannot_be_deleted(node, tmp_path):
    res = make_result(tmp_path)
    node.faults["upload_lose_response"] = 1
    with pytest.raises(PipelineError, match="不能重送") as ei:
        pipeline.run_odm_job(job_for(res, node), client=client(node), poll_s=0, sleep=NOOP)
    assert "48 h" in str(ei.value)
    rec = record(res)
    assert rec["phase"] == "upload_failed" and rec["remote_cleanup"]["result"] == "not_found"
    assert node.tmp_files(rec["uuid"]) == ["geo.txt"]                        # 規格：未提交的上傳沒有刪除 API
    assert node.requests_to("/task/remove")


def test_cancel_before_commit_and_cancel_of_a_running_remote_task(node, tmp_path):
    res = make_result(tmp_path)
    job = job_for(res, node)
    batches = []

    def on_state(st):
        b = (st.get("upload") or {}).get("files_done")
        if b is not None:
            batches.append(b)
    with pytest.raises(PipelineCanceled, match="未提交.*48 h"):
        pipeline.run_odm_job(job, client=client(node), poll_s=0, sleep=NOOP, batch_max_files=2, on_state=on_state,
                             should_cancel=lambda: bool(batches) and max(batches) >= 3)
    rec = record(res)
    assert rec["phase"] == "upload_canceled" and rec["remote_cleanup"]["endpoint"] == "/task/remove"
    assert node.requests_to("/task/new/commit/") == []
    # 提交後取消：只停本機等待（紀錄留 running），新的送出被擋，取消遠端後才能送
    prog = []
    with pytest.raises(PipelineCanceled, match="接回任務"):
        pipeline.run_odm_job(job, client=client(node), poll_s=0, sleep=NOOP, progress=lambda m, f: prog.append(m),
                             should_cancel=lambda: sum("節點" in m for m in prog) >= 2)
    rec = record(res)
    assert rec["phase"] == "running" and pipeline.odm_submission_status(rec)["can_reattach"]
    with pytest.raises(PipelineError, match="沒收尾"):
        pipeline.run_odm_job(job, client=client(node), poll_s=0, sleep=NOOP)
    r = pipeline.cancel_odm_job({"out_dir": str(res.out_dir)}, client=client(node), poll_s=0, sleep=NOOP)
    assert r["phase"] == "canceled" and r["status_code"] == 50 and node.tasks[rec["uuid"]]["status"]["code"] == 50
    st = pipeline.odm_submission_status(pipeline.load_submission(res.out_dir))
    assert st["phase"] == "canceled" and not st["can_reattach"] and not st["can_cancel_remote"]
    out = pipeline.run_odm_job(job, client=client(node), poll_s=0, sleep=NOOP)        # 舊紀錄已收尾 ⇒ 搬到 history/
    assert out["phase"] == "downloaded"
    hist = sorted(p.name for p in (res.out_dir / "odm" / "history").glob("submission_*.json"))
    assert len(hist) == 2 and any("upload_canceled" in h for h in hist) and any("canceled" in h for h in hist)
    assert_no_token(res.out_dir)


def test_polling_survives_transient_connection_drops(node, tmp_path):
    res = make_result(tmp_path)
    node.faults["info_drop"] = 2
    out = pipeline.run_odm_job(job_for(res, node), client=client(node), poll_s=0, sleep=NOOP)
    assert out["phase"] == "downloaded" and node.faults["info_drop"] == 0


# ─────────────────────────────────────────────────────────────────────────────
# 當機後接回（同一條收尾路徑）
# ─────────────────────────────────────────────────────────────────────────────

class _Crash(BaseException):
    """模擬客戶端行程當掉（不是 Exception ⇒ 流水線的任何清理都不會跑）。"""


class CrashSession(requests.Session):
    def __init__(self, method, match, nth, timing):
        super().__init__()
        self.trust_env = False
        self.method, self.match, self.nth, self.timing = method, match, nth, timing
        self.counts = Counter()

    def request(self, method, url, *a, **kw):
        path = urlsplit(url).path
        hit = method.upper() == self.method and self.match(path)
        if hit:
            self.counts["n"] += 1
            hit = self.counts["n"] == self.nth
        if hit and self.timing == "before":
            raise _Crash(path)
        resp = super().request(method, url, *a, **kw)
        if hit:
            raise _Crash(path)
        return resp


CRASHES = {
    "uploading": ("POST", lambda p: p.startswith("/task/new/upload/"), 2, "before"),
    "committing": ("POST", lambda p: p.startswith("/task/new/commit/"), 1, "after"),     # 節點已提交、回應沒收到
    "running": ("GET", lambda p: p.startswith("/task/") and p.endswith("/info"), 3, "before"),
    "completed": ("GET", lambda p: p.endswith("/download/all.zip"), 1, "before"),
}


@pytest.mark.parametrize("phase", sorted(CRASHES))
def test_reattach_after_a_client_crash(node, tmp_path, phase):
    res = make_result(tmp_path)
    job = job_for(res, node, gcp_txt=gcp_file(tmp_path, res), gcp_vertical="ellipsoidal", force_gps=False)
    with pytest.raises(_Crash):
        pipeline.run_odm_job(job, client=client(node, CrashSession(*CRASHES[phase])), poll_s=0, sleep=NOOP,
                             batch_max_files=2)
    rec = record(res)
    assert rec["phase"] == phase
    out_dir = res.out_dir
    if phase == "running":                                                      # 資料夾搬過也能接（以紀錄檔位置為準）
        out_dir = tmp_path / "moved"
        shutil.move(str(res.out_dir), str(out_dir))
    st = pipeline.odm_submission_status(pipeline.load_submission(out_dir))
    if phase == "uploading":
        assert st["can_reattach"] is False
        with pytest.raises(PipelineError, match="沒有提交.*重新送出.*48 h"):
            pipeline.resume_odm_job({"out_dir": str(out_dir)}, client=client(node), poll_s=0, sleep=NOOP)
        assert pipeline.load_submission(out_dir)["phase"] == "uploading"
        r = pipeline.cancel_odm_job({"out_dir": out_dir}, client=client(node), poll_s=0, sleep=NOOP)
        assert r["phase"] == "upload_canceled" and "48 h" in r["message"]          # 照實說：未提交的上傳沒有刪除 API
        assert pipeline.load_submission(out_dir)["remote_cleanup"]["result"] == "not_found"
        return
    assert st["can_reattach"] is True
    out = pipeline.resume_odm_job({"out_dir": str(out_dir)}, client=client(node), poll_s=0, sleep=NOOP)
    rec = pipeline.load_submission(out_dir)
    assert out["phase"] == rec["phase"] == "downloaded" and out["uuid"] == rec["uuid"]
    assert out["ortho_tiles"] == out_dir.resolve() / "odm" / f"result_{rec['uuid']}" / "orthophoto_tiles"
    assert out["quality"]["stats_available"] and (out_dir / "odm" / "quality_summary.json").is_file()
    assert any(e["event"] == "reattach" for e in rec["events"]) and rec["images_check"]["ok"] is True
    if phase == "committing":
        assert rec["commit_confirmed_on_reattach"] is True
    # 已下載的任務再接一次：不重下，只重整本機成果與品質報告（同一條收尾路徑）
    n_dl = len(node.requests_to(f"/task/{rec['uuid']}/download"))
    again = pipeline.resume_odm_job(out_dir / "odm" / "submission.json", client=client(node), poll_s=0, sleep=NOOP)
    assert again["quality"]["stats_available"] and len(node.requests_to(f"/task/{rec['uuid']}/download")) == n_dl
    assert_no_token(out_dir)


def test_reattach_reports_a_task_the_node_no_longer_has(node, tmp_path):
    res = make_result(tmp_path)
    with pytest.raises(_Crash):
        pipeline.run_odm_job(job_for(res, node), client=client(node, CrashSession(*CRASHES["running"])), poll_s=0,
                             sleep=NOOP)
    node.tasks.pop(record(res)["uuid"])                                        # 過期／被刪／節點當機遺失
    with pytest.raises(PipelineError, match="查不到任務.*48 h"):
        pipeline.resume_odm_job({"out_dir": res.out_dir}, client=client(node), poll_s=0, sleep=NOOP)
    assert record(res)["phase"] == "lost"


def test_reattach_refuses_a_different_node_and_reads_legacy_records(node, tmp_path):
    from survey.odm_client import build_options
    res = make_result(tmp_path)
    with pytest.raises(_Crash):
        pipeline.run_odm_job(job_for(res, node), client=client(node, CrashSession(*CRASHES["running"])), poll_s=0,
                             sleep=NOOP)
    with pytest.raises(PipelineError, match="不同節點"):
        pipeline.resume_odm_job({"out_dir": res.out_dir}, client=NodeOdmClient("http://127.0.0.1:1", token=TOKEN))
    # 舊版（/1）紀錄：只有 uuid（單一 POST 已提交）⇒ 當成已提交接回
    c = client(node)
    import uuid as _u
    uid = str(_u.uuid4())
    opts = build_options(orthophoto_resolution_cm=2.5)
    c.init_task(uid, options=opts)
    c.upload_files(uid, [(p, p.name) for p in res.images], sleep=NOOP)
    c.commit_task(uid)
    legacy = tmp_path / "legacy"
    (legacy / "odm").mkdir(parents=True)
    (legacy / "odm" / "submission.json").write_text(json.dumps(
        {"node_url": node.url, "uuid": uid, "options": opts, "n_images": len(res.images),
         "image_names": [p.name for p in res.images], "out_dir": "D:/somewhere/else"}), encoding="utf-8")
    assert pipeline.load_submission(legacy)["phase"] == "committed"
    out = pipeline.resume_odm_job({"out_dir": legacy}, client=c, poll_s=0, sleep=NOOP)
    assert out["phase"] == "downloaded" and pipeline.load_submission(legacy)["images_check"]["ok"] is True


# ─────────────────────────────────────────────────────────────────────────────
# 品質報告
# ─────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("stats,needle", [("missing", "找不到 odm_report/stats.json"), ("malformed", "無法解析")])
def test_missing_or_malformed_stats_json_is_reported_not_skipped(tmp_path, stats, needle):
    n = make_node(tmp_path, "node", stats=stats)
    try:
        res = make_result(tmp_path)
        out = pipeline.run_odm_job(job_for(res, n), client=client(n), poll_s=0, sleep=NOOP)
        q = out["quality"]
        assert q["stats_available"] is False and any(needle in w for w in q["warnings"])
        assert any(needle in w for w in out["warnings"]) and any(needle in w for w in record(res)["warnings"])
        assert out["quality_rows"][0][3] is False and "⚠" in out["quality_rows"][0][2]
        assert needle in pipeline.odm_quality_text(q)
    finally:
        n.stop()


def test_read_odm_quality_is_tolerant_and_explains(tmp_path):
    root = tmp_path / "r"
    (root / "odm_report").mkdir(parents=True)
    q = pipeline.read_odm_quality(root, n_uploaded=10, sent_options=[{"name": "skip-report", "value": True}])
    assert not q["stats_available"] and any("--skip-report" in w for w in q["warnings"])
    assert any("log.json" in w for w in q["warnings"])
    stats = FakeNodeODX(tmp_path / "n").stats_json({"images": [f"{k}.jpg" for k in range(10)], "gcp": "gcp_list.txt"})
    stats["reconstruction_statistics"].update(components=2, initial_shots_count=9, reprojection_error_pixels=-1)
    stats["gcp_errors"]["cp_only"] = {"error": {"x": 0.02, "y": 0.03, "z": 0.05}, "ce90": 0.04, "le90": 0.08}
    (root / "odm_report" / "stats.json").write_text(json.dumps(stats), encoding="utf-8")
    q = pipeline.read_odm_quality(root, n_uploaded=10, force_gps={"decision": "force_gps"}, uuid="u")
    assert q["images"] == {**q["images"], "uploaded": 10, "initial_shots": 9, "missing_before_sfm": 1}
    assert q["reconstruction"]["reprojection_error_pixels"] is None                # -1 = 沒有這個值
    assert q["gcp"]["checkpoints"]["rms_horizontal"] == pytest.approx((0.02 ** 2 + 0.03 ** 2) ** 0.5)
    assert any("2 塊" in w for w in q["warnings"]) and any("只有 9 張" in w for w in q["warnings"])
    rows = pipeline.odm_quality_rows(q)
    assert ("ODM", "模型分塊數", "2", False) in rows
    assert any(r[1].startswith("檢核點誤差") for r in rows) and any("已扣掉每台相機的偏差" in r[1] for r in rows)
