"""survey.odm_client 的分段上傳：對真的本機 HTTP 假 NodeODX（tests/fake_nodeodx.py）跑。

涵蓋：每批檔數／位元組上限、從磁碟串流（量客戶端行程的記憶體峰值；伺服器放子行程）、5xx／斷線重送、
「送完沒回應」的模稜兩可、取消、權杖不出現在例外訊息、位址不帶查詢字串、檔案在上傳途中變動。
"""

import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest
import requests

from survey import odm_client
from survey.odm_client import (MultipartStream, NodeOdmClient, OdmCanceled, OdmError, OdmTransportError,
                               build_options, plan_batches, sanitize_node_url)
from tests.fake_nodeodx import FakeNodeODX

TOKEN = "tok-Zq9_S3CRET-4711"


def session():
    s = requests.Session()
    s.trust_env = False                    # Windows 系統代理可能攔 127.0.0.1
    return s


@pytest.fixture
def node(tmp_path):
    n = FakeNodeODX(tmp_path / "node", token=TOKEN).start()
    yield n
    n.stop()


def client(node, **kw):
    kw.setdefault("token", TOKEN)
    return NodeOdmClient(node.url, session=session(), timeout_s=30, **kw)


def make_files(d: Path, n: int, size: int, prefix="IMG_") -> list:
    d.mkdir(parents=True, exist_ok=True)
    block = bytes(range(256)) * 4096          # 1 MiB 的區塊重複寫（不在測試裡配置整個檔案大小的記憶體）
    out = []
    for k in range(n):
        p = d / f"{prefix}{k:04d}.jpg"
        with open(p, "wb") as fh:
            left = size
            fh.write(b"\xff\xd8")
            left -= 2
            while left > 0:
                fh.write(block[:min(left, len(block))])
                left -= min(left, len(block))
        out.append(p)
    return out


def new_uuid():
    import uuid
    return str(uuid.uuid4())


# ─────────────────────────────────────────────────────────────────────────────
# 純函式
# ─────────────────────────────────────────────────────────────────────────────

def test_plan_batches_respects_both_bounds_and_lets_a_big_file_go_alone():
    items = [(f"f{k}", f"f{k}.jpg", s) for k, s in enumerate([10, 10, 10, 50, 200, 10, 10, 10, 10, 10])]
    b = plan_batches(items, max_files=3, max_bytes=60)
    sizes = [[i[2] for i in x] for x in b]
    assert sizes == [[10, 10, 10], [50], [200], [10, 10, 10], [10, 10]]
    assert all(len(x) <= 3 for x in b) and all(sum(s) <= 60 or len(s) == 1 for s in sizes)
    assert [i for x in b for i in x] == items                                  # 順序不變、一個都沒少
    with pytest.raises(ValueError):
        plan_batches(items, max_files=0)


def test_sanitize_node_url_refuses_query_and_userinfo_without_echoing_the_token():
    assert sanitize_node_url(" HTTP://10.0.0.5:3000/ ") == "http://10.0.0.5:3000"
    assert sanitize_node_url("https://odx.lab/node/") == "https://odx.lab/node"
    for bad in (f"http://10.0.0.5:3000/?token={TOKEN}", f"http://u:{TOKEN}@10.0.0.5:3000", "ftp://x", "10.0.0.5:3000",
                f"http://10.0.0.5:3000/#token={TOKEN}"):
        with pytest.raises(ValueError) as ei:
            sanitize_node_url(bad)
        assert TOKEN not in str(ei.value)
    with pytest.raises(ValueError, match="權杖"):
        NodeOdmClient(f"http://127.0.0.1:3000/?token={TOKEN}")


def test_redact_scrubs_query_tokens_and_the_literal_value():
    msg = f"Max retries exceeded with url: /task/x/info?token={TOKEN}&line=3 (Caused by ...)"
    assert TOKEN not in odm_client.redact(msg) and "token=***&line=3" in odm_client.redact(msg)
    assert odm_client.redact(f"oops {TOKEN} oops", TOKEN) == "oops *** oops"


def test_multipart_stream_reads_lazily_and_refuses_a_file_that_changed(tmp_path):
    a, b = make_files(tmp_path, 2, 300_000)
    s = MultipartStream(fields=[("name", "x")], files=[("images", a.name, a, a.stat().st_size),
                                                      ("images", b.name, b, b.stat().st_size)])
    first = s.read(16384)
    assert len(first) == 16384 and s.bytes_read == 16384 and not s.complete
    rest = b"".join(iter(lambda: s.read(16384), b""))
    assert len(first) + len(rest) == len(s) and s.complete
    s2 = MultipartStream(files=[("images", a.name, a, a.stat().st_size)])
    s2.read(1000)
    with open(a, "ab") as fh:
        fh.write(b"more")
    with pytest.raises(OdmError, match="變大"):
        while s2.read(65536):
            pass
    s3 = MultipartStream(files=[("images", b.name, b, b.stat().st_size + 10)])
    with pytest.raises(OdmError, match="變短"):
        while s3.read(65536):
            pass


def test_build_options_force_gps_and_max_concurrency():
    base = build_options(orthophoto_resolution_cm=2.5)
    assert build_options(orthophoto_resolution_cm=2.5, force_gps=False, max_concurrency=None) == base
    assert build_options(orthophoto_resolution_cm=2.5, force_gps=True, max_concurrency=12)[-2:] == [
        {"name": "force-gps", "value": True}, {"name": "max-concurrency", "value": 12}]
    for bad in (0, -1, True, 2.5, "12"):
        with pytest.raises(ValueError, match="max_concurrency"):
            build_options(orthophoto_resolution_cm=2.5, max_concurrency=bad)
    with pytest.raises(ValueError, match="force_gps"):
        build_options(orthophoto_resolution_cm=2.5, force_gps=None)


# ─────────────────────────────────────────────────────────────────────────────
# 真 HTTP
# ─────────────────────────────────────────────────────────────────────────────

def test_init_is_multipart_with_set_uuid_and_options_reach_the_node(node):
    c = client(node)
    uid = new_uuid()
    opts = build_options(orthophoto_resolution_cm=2.5, force_gps=True)
    assert c.init_task(uid, options=opts, name="F1") == uid
    body = json.loads((node.root / "tmp" / uid / "body.json").read_text(encoding="utf-8"))
    assert body["options"] == opts and body["name"] == "F1"                  # 沒被當成非 multipart 靜默丟掉
    assert node.requests_to("/task/new/init")[0]["set_uuid"] == uid
    assert node.requests_to("/task/new/init")[0]["query"]["token"] == TOKEN   # 權杖只走 query string
    with pytest.raises(OdmError, match="out of range"):
        c.init_task(new_uuid(), options=[{"name": "orthophoto-resolution", "value": -1}])


def test_batches_are_bounded_by_count_and_bytes_and_progress_is_reported(node, tmp_path):
    c = client(node)
    uid = new_uuid()
    c.init_task(uid, options=[])
    small = make_files(tmp_path / "a", 7, 10_000)
    big = make_files(tmp_path / "b", 1, 60_000, prefix="BIG_")
    files = [(p, p.name) for p in small[:3] + big + small[3:]]
    ev, batches = [], []
    reps = c.upload_files(uid, files, max_files=3, max_bytes=50_000, progress=ev.append, on_batch=batches.append,
                          sleep=lambda s: None)
    uploads = [r["files"] for r in node.requests_to(f"/task/new/upload/{uid}")]
    assert uploads == [["IMG_0000.jpg", "IMG_0001.jpg", "IMG_0002.jpg"], ["BIG_0000.jpg"],
                       ["IMG_0003.jpg", "IMG_0004.jpg", "IMG_0005.jpg"], ["IMG_0006.jpg"]]
    assert [r["n_files"] for r in reps] == [3, 1, 3, 1] and batches == reps and all(r["attempts"] == 1 for r in reps)
    assert node.tmp_files(uid) == sorted(p.name for p, _ in files)
    for p, _ in files:
        assert (node.root / "tmp" / uid / p.name).read_bytes() == p.read_bytes()
    total = sum(p.stat().st_size for p, _ in files)
    assert ev[-1]["files_done"] == 8 and ev[-1]["bytes_done"] == total and ev[-1]["bytes_total"] == total
    done = [e["bytes_done"] for e in ev]
    assert done == sorted(done) and {e["n_batches"] for e in ev} == {4}
    assert {e["file"] for e in ev if e["file"]} == {p.name for p, _ in files}   # 每個檔案完成都有回報


def test_streaming_upload_keeps_client_memory_far_below_the_upload_size(tmp_path):
    """24 × 16 MB 從 tmp_path 上傳：客戶端行程的記憶體峰值增量要遠小於總量（伺服器在子行程，不算進來）。"""
    files = make_files(tmp_path / "imgs", 24, 16 * 1024 * 1024)
    total = sum(p.stat().st_size for p in files)
    env = dict(os.environ, AEROPLAN_NO_FILE_LOG="1", PYTHONIOENCODING="utf-8")
    proc = subprocess.Popen([sys.executable, str(Path(__file__).with_name("fake_nodeodx.py")), "--root",
                             str(tmp_path / "node")], stdin=subprocess.PIPE, stdout=subprocess.PIPE, env=env)
    try:
        line = proc.stdout.readline().decode().strip()
        assert line.startswith("PORT "), line
        c = NodeOdmClient(f"http://127.0.0.1:{line.split()[1]}", session=session(), timeout_s=120)
        uid = new_uuid()
        c.init_task(uid, options=[])
        rss = _rss_reader()
        base = rss()
        peak = [base]
        stop = threading.Event()

        def sample():
            while not stop.is_set():
                peak[0] = max(peak[0], rss())
                time.sleep(0.002)
        th = threading.Thread(target=sample, daemon=True)
        th.start()
        try:
            reps = c.upload_files(uid, [(p, p.name) for p in files], sleep=lambda s: None)
        finally:
            stop.set()
            th.join()
        growth = peak[0] - base
        assert [r["n_files"] for r in reps] == [16, 8]                         # 預設上限 25 檔／256 MiB
        assert growth < 0.25 * total and growth < 200 * 2**20, f"峰值增加 {growth / 2**20:.1f} MiB"
        c.commit_task(uid)
        assert c.task_info(uid)["imagesCount"] == 24
        assert c.task_info(uid)["imagesCount"] == 24                          # 初始化後（images/）也是 24
    finally:
        proc.stdin.close()
        proc.wait(timeout=30)
        for p in files:
            p.unlink(missing_ok=True)


def _rss_reader():
    try:
        import psutil
        proc = psutil.Process()
        return lambda: proc.memory_info().rss
    except ImportError:                                  # pragma: no cover - psutil 沒裝時用 Win32 API
        import ctypes
        from ctypes import wintypes

        class PMC(ctypes.Structure):
            _fields_ = [("cb", wintypes.DWORD), ("PageFaultCount", wintypes.DWORD),
                        ("PeakWorkingSetSize", ctypes.c_size_t), ("WorkingSetSize", ctypes.c_size_t),
                        ("QuotaPeakPagedPoolUsage", ctypes.c_size_t), ("QuotaPagedPoolUsage", ctypes.c_size_t),
                        ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t), ("QuotaNonPagedPoolUsage", ctypes.c_size_t),
                        ("PagefileUsage", ctypes.c_size_t), ("PeakPagefileUsage", ctypes.c_size_t)]
        h = ctypes.windll.kernel32.GetCurrentProcess()

        def rss():
            pmc = PMC()
            pmc.cb = ctypes.sizeof(PMC)
            ctypes.windll.psapi.GetProcessMemoryInfo(h, ctypes.byref(pmc), pmc.cb)
            return pmc.WorkingSetSize
        return rss


def test_http_500_once_is_retried_without_duplicates(node, tmp_path):
    c = client(node)
    uid = new_uuid()
    c.init_task(uid, options=[])
    files = make_files(tmp_path, 4, 5_000)
    node.faults["upload_500"] = 1
    slept = []
    reps = c.upload_files(uid, [(p, p.name) for p in files], max_files=2, sleep=slept.append)
    assert reps[0]["attempts"] == 2 and reps[0]["ambiguous_retries"] == 0 and "HTTP 500" in reps[0]["errors"][0]
    assert reps[1]["attempts"] == 1 and sum(slept) == pytest.approx(odm_client.UPLOAD_BACKOFF_S[0])
    assert node.tmp_files(uid) == sorted(p.name for p in files)               # 沒有 *_ 重複檔


def test_connection_drop_mid_batch_is_a_safe_retry(node, tmp_path):
    c = client(node)
    uid = new_uuid()
    c.init_task(uid, options=[])
    files = make_files(tmp_path, 2, 16 * 1024 * 1024)                        # 32 MB：伺服器讀到一半就切，客戶端還在送
    node.faults["upload_drop"] = 1
    reps = c.upload_files(uid, [(p, p.name) for p in files], sleep=lambda s: None)
    assert reps[0]["attempts"] == 2 and reps[0]["ambiguous_retries"] == 0
    assert [r["result"] for r in node.requests_to(f"/task/new/upload/{uid}")] == ["dropped", "ok"]
    assert node.tmp_files(uid) == sorted(p.name for p in files)


def test_lost_response_after_full_body_is_ambiguous(node, tmp_path):
    """主體送完、回應遺失：影像批次照規則重送（記成 ambiguous），節點就多出 *_ 重複檔——由提交後的張數核對抓。"""
    c = client(node)
    uid = new_uuid()
    c.init_task(uid, options=[])
    files = make_files(tmp_path, 2, 2_000)
    node.faults["upload_lose_response"] = 1
    reps = c.upload_files(uid, [(p, p.name) for p in files], sleep=lambda s: None)
    assert reps[0]["attempts"] == 2 and reps[0]["ambiguous_retries"] == 1
    assert node.tmp_files(uid) == ["IMG_0000.jpg", "IMG_0000_.jpg", "IMG_0001.jpg", "IMG_0001_.jpg"]
    # 附檔（geo.txt）不允許：重送會變成 geo_.txt（被當成 GCP）⇒ 直接失敗
    uid2 = new_uuid()
    c.init_task(uid2, options=[])
    geo = tmp_path / "geo.txt"
    geo.write_text("EPSG:4326\n", encoding="utf-8")
    node.faults["upload_lose_response"] = 1
    with pytest.raises(OdmTransportError, match="不能重送") as ei:
        c.upload_files(uid2, [(geo, "geo.txt")], allow_ambiguous_retry=False, sleep=lambda s: None)
    assert ei.value.ambiguous and node.tmp_files(uid2) == ["geo.txt"]


def test_permanent_errors_are_not_retried_and_exhausted_retries_fail(node, tmp_path):
    c = client(node)
    files = make_files(tmp_path, 1, 1_000)
    with pytest.raises(OdmError, match="Invalid uuid"):
        c.upload_files(new_uuid(), [(files[0], files[0].name)], sleep=lambda s: None)
    assert len(node.requests_to("/task/new/upload/")) == 1
    uid = new_uuid()
    c.init_task(uid, options=[])
    node.faults["upload_500"] = 10
    with pytest.raises(OdmTransportError, match="試了 3 次"):
        c.upload_files(uid, [(files[0], files[0].name)], attempts=3, sleep=lambda s: None)


def test_cancel_between_batches(node, tmp_path):
    c = client(node)
    uid = new_uuid()
    c.init_task(uid, options=[])
    files = make_files(tmp_path, 5, 1_000)
    done = []
    with pytest.raises(OdmCanceled, match="第 3/3 批之前") as ei:
        c.upload_files(uid, [(p, p.name) for p in files], max_files=2, on_batch=done.append,
                       should_cancel=lambda: len(done) >= 2, sleep=lambda s: None)
    assert len(ei.value.batches) == 2 and node.tmp_files(uid) == sorted(p.name for p in files[:4])


def test_cancel_in_the_middle_of_a_batch_aborts_that_request_and_leaves_no_partial_files(node, tmp_path):
    """關主視窗時不必等一整批傳完：中斷的請求節點整批丟掉（規格 §2.2），已完成的批次留著。"""
    c = client(node)
    uid = new_uuid()
    c.init_task(uid, options=[])
    files = make_files(tmp_path, 4, 8 * 1024 * 1024)
    seen = []
    with pytest.raises(OdmCanceled, match="第 2/2 批上傳途中取消") as ei:
        c.upload_files(uid, [(p, p.name) for p in files], max_files=2, sleep=lambda s: None,
                       progress=lambda ev: seen.append(ev["files_done"]),
                       should_cancel=lambda: bool(seen) and max(seen) >= 3)        # 第 2 批的第一個檔送完就取消
    assert len(ei.value.batches) == 1
    for _ in range(50):                                                         # 伺服器發現斷線後才清檔
        if node.tmp_files(uid) == [files[0].name, files[1].name]:
            break
        time.sleep(0.05)
    assert node.tmp_files(uid) == [files[0].name, files[1].name]
    assert node.requests_to(f"/task/new/upload/{uid}")[-1]["result"] == "client aborted"


def test_commit_probe_cancel_and_remove_follow_the_spec(node, tmp_path):
    c = client(node)
    uid = new_uuid()
    c.init_task(uid, options=[])
    assert c.probe_upload_session(uid) is True                                # "Need at least 1 file."
    assert c.probe_upload_session(new_uuid()) is False                        # "Invalid uuid (not found)"
    with pytest.raises(OdmError) as ei:                                       # 未提交：沒有刪除 API
        c.remove_task(uid)
    assert ei.value.not_found
    files = make_files(tmp_path, 2, 1_000)
    c.upload_files(uid, [(p, p.name) for p in files], sleep=lambda s: None)
    assert c.commit_task(uid) == uid
    with pytest.raises(OdmError, match="Invalid uuid"):
        c.commit_task(uid)                                                    # 第二次 commit：body.json 已刪
    info = c.task_info(uid)
    assert info["status"]["code"] == 20 and info["processingTime"] == -1 and info["imagesCount"] == 2
    c.cancel_task(uid)
    assert c.task_info(uid)["status"]["code"] == 50
    c.remove_task(uid)
    with pytest.raises(OdmError) as ei:
        c.task_info(uid)
    assert ei.value.not_found


def test_token_is_redacted_from_transport_errors_and_wrong_token_is_reported(node, tmp_path):
    import socket
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()                                                                 # 沒人在聽的埠
    c = NodeOdmClient(f"http://127.0.0.1:{port}", token=TOKEN, session=session(), connect_timeout_s=2)
    with pytest.raises(OdmTransportError) as ei:
        c.node_info()
    assert TOKEN not in str(ei.value) and "token=***" in str(ei.value)
    assert ei.value.__context__ is None and ei.value.__cause__ is None         # 不留帶權杖的原例外
    bad = client(node, token="wrong")
    with pytest.raises(OdmError, match="Invalid authentication token") as ei:
        bad.node_info()
    assert "wrong" not in str(ei.value)
    with pytest.raises(OdmError, match="Invalid authentication token"):
        NodeOdmClient(node.url, session=session()).node_info()               # 沒給權杖


def test_download_checks_content_type_and_length(node, tmp_path):
    c = client(node)
    uid = new_uuid()
    c.init_task(uid, options=[])
    f = make_files(tmp_path, 1, 1_000)[0]
    c.upload_files(uid, [(f, f.name)], sleep=lambda s: None)
    c.commit_task(uid)
    with pytest.raises(OdmError, match="Asset not ready"):
        c.download_all(uid, tmp_path / "x.zip")
    for _ in range(8):
        c.task_info(uid)
    got = []
    z = c.download_all(uid, tmp_path / "out" / "all.zip", progress=lambda n, t: got.append((n, t)))
    assert z.stat().st_size == got[-1][0] == got[-1][1] and not (tmp_path / "out" / "all.zip.part").exists()
