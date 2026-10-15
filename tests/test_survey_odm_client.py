"""survey.odm_client：參數組裝與 REST 呼叫（以假 session 取代網路）。"""

import json

import pytest

from survey import odm_client
from survey.odm_client import NodeOdmClient, OdmError, build_options


class FakeResp:
    def __init__(self, status=200, payload=None, content=b"", ctype="application/json"):
        self.status_code = status
        self._payload = payload
        self._content = content
        self.headers = {"Content-Type": ctype}

    def json(self):
        if self._payload is None:
            raise ValueError("no json")
        return self._payload

    def iter_content(self, chunk_size=1):
        for i in range(0, len(self._content), 3):
            yield self._content[i:i + 3]


class FakeSession:
    def __init__(self, responses):
        self.responses = list(responses)
        self.calls = []

    def _next(self, method, url, **kw):
        self.calls.append((method, url, kw))
        if "files" in kw:   # 記下檔案內容（ExitStack 會在回傳後關檔）
            kw["file_names"] = [(field, name) for field, (name, fh) in kw["files"]]
            kw["file_bytes"] = [fh.read() for _, (_, fh) in kw["files"]]
        return self.responses.pop(0)

    def post(self, url, **kw):
        return self._next("POST", url, **kw)

    def get(self, url, **kw):
        return self._next("GET", url, **kw)


def test_build_options_names_and_values():
    opts = build_options(orthophoto_resolution_cm=3.1, gps_accuracy_m=0.5, rolling_shutter_readout_ms=86.9,
                         split=400, split_overlap_m=150)
    assert opts == [
        {"name": "orthophoto-resolution", "value": 3.1}, {"name": "dsm", "value": True},
        {"name": "tiles", "value": True}, {"name": "gps-accuracy", "value": 0.5},
        {"name": "rolling-shutter", "value": True}, {"name": "rolling-shutter-readout", "value": 86.9},
        {"name": "split", "value": 400}, {"name": "split-overlap", "value": 150.0}]
    assert build_options(orthophoto_resolution_cm=5, dsm=False, tiles=False) == [
        {"name": "orthophoto-resolution", "value": 5.0}]
    with pytest.raises(ValueError, match="0 在 ODM"):
        build_options(orthophoto_resolution_cm=5, rolling_shutter_readout_ms=0)
    with pytest.raises(ValueError, match="split"):
        build_options(orthophoto_resolution_cm=5, split_overlap_m=150)
    with pytest.raises(ValueError, match="重複"):
        build_options(orthophoto_resolution_cm=5, extra={"dsm": True})
    assert build_options(orthophoto_resolution_cm=5, dsm=False, tiles=False, extra={"fast-orthophoto": True})[-1] == \
        {"name": "fast-orthophoto", "value": True}


def test_rolling_shutter_readout_consistency():
    assert odm_client.rolling_shutter_readout([86.9, 87.3, 86.5]) == pytest.approx(86.9)
    with pytest.raises(ValueError, match="分任務"):
        odm_client.rolling_shutter_readout([86.9, 68.1])
    with pytest.raises(ValueError, match="null"):
        odm_client.rolling_shutter_readout([86.9, None])
    with pytest.raises(ValueError):
        odm_client.rolling_shutter_readout([])


def test_new_task_uploads_images_geo_and_options(tmp_path):
    imgs = []
    for k in range(2):
        p = tmp_path / f"a_{k}.jpg"
        p.write_bytes(b"jpeg%d" % k)
        imgs.append(p)
    geo = tmp_path / "geo.txt"
    geo.write_bytes(b"EPSG:4326\n")
    s = FakeSession([FakeResp(payload={"uuid": "abc-123"})])
    c = NodeOdmClient("http://127.0.0.1:3000/", token="tok", session=s, timeout_s=5)
    opts = build_options(orthophoto_resolution_cm=3.1)
    assert c.new_task(imgs, options=opts, geo_txt=geo, name="F1") == "abc-123"
    method, url, kw = s.calls[0]
    assert (method, url) == ("POST", "http://127.0.0.1:3000/task/new")
    assert kw["params"] == {"token": "tok"} and kw["timeout"] == 5
    assert json.loads(kw["data"]["options"]) == opts and kw["data"]["name"] == "F1"
    assert kw["file_names"] == [("images", "a_0.jpg"), ("images", "a_1.jpg"), ("images", "geo.txt")]
    assert kw["file_bytes"] == [b"jpeg0", b"jpeg1", b"EPSG:4326\n"]


def test_new_task_rejects_misnamed_files(tmp_path):
    img = tmp_path / "a.jpg"
    img.write_bytes(b"x")
    notes = tmp_path / "notes.txt"
    notes.write_text("x", encoding="utf-8")
    c = NodeOdmClient(session=FakeSession([]))
    with pytest.raises(ValueError, match="GCP"):
        c.new_task([img, notes], options=[])
    with pytest.raises(ValueError, match="geo.txt"):
        c.new_task([img], options=[], geo_txt=notes)
    with pytest.raises(ValueError, match="GCP 檔"):
        c.new_task([img], options=[], gcp_txt=tmp_path / "geo.txt")
    with pytest.raises(ValueError, match="沒有影像"):
        c.new_task([], options=[])


def test_errors_reported_even_with_http_200(tmp_path):
    img = tmp_path / "a.jpg"
    img.write_bytes(b"x")
    c = NodeOdmClient(session=FakeSession([FakeResp(payload={"error": "Need at least 1 file."})]))
    with pytest.raises(OdmError, match="Need at least 1 file"):
        c.new_task([img], options=[])
    c = NodeOdmClient(session=FakeSession([FakeResp(status=500, payload=None)]))
    with pytest.raises(OdmError, match="不是 JSON"):
        c.task_info("u")


def test_wait_polls_until_completed_or_failed():
    seq = [FakeResp(payload={"status": {"code": 10}}), FakeResp(payload={"status": {"code": 20}}),
           FakeResp(payload={"status": {"code": 40}, "imagesCount": 2})]
    s = FakeSession(seq)
    slept = []
    info = NodeOdmClient(session=s).wait("u1", poll_s=7, sleep=slept.append, clock=lambda: 0.0)
    assert info["imagesCount"] == 2 and slept == [7, 7]
    assert [c[1] for c in s.calls] == ["http://127.0.0.1:3000/task/u1/info"] * 3
    with pytest.raises(OdmError, match="狀態 30"):
        NodeOdmClient(session=FakeSession([FakeResp(payload={"status": {"code": 30}})])).wait(
            "u", sleep=lambda s: None)
    t = iter([0.0, 100.0])
    with pytest.raises(OdmError, match="逾時"):
        NodeOdmClient(session=FakeSession([FakeResp(payload={"status": {"code": 20}})])).wait(
            "u", timeout_s=50, sleep=lambda s: None, clock=lambda: next(t))


def test_download_all_streams_to_file(tmp_path):
    s = FakeSession([FakeResp(content=b"PK\x03\x04zipdata", ctype="application/zip")])
    out = NodeOdmClient(session=s, token="t").download_all("u9", tmp_path / "res" / "all.zip")
    assert out.read_bytes() == b"PK\x03\x04zipdata" and not (tmp_path / "res" / "all.zip.part").exists()
    assert s.calls[0][1] == "http://127.0.0.1:3000/task/u9/download/all.zip" and s.calls[0][2]["stream"] is True
    s = FakeSession([FakeResp(payload={"error": "Invalid asset"})])
    with pytest.raises(OdmError, match="Invalid asset"):
        NodeOdmClient(session=s).download_all("u9", tmp_path / "x.zip")
