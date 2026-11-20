"""payload/pi_capture —— 樹莓派相機模組擷取服務（只用假後端；不碰 GPIO、相機、序列埠）

鎖住的值：
- 挑幀：每個上升緣取「第一張 SensorTimestamp（換到 MONOTONIC）≥ edge_ns」的影格，其他影格立刻釋放；
  沒影格 → no_frame、寫入佇列滿 → dropped、寫檔失敗 → write_error。
- frames.jsonl 的每一列恰好是 F1 的 13 個鍵與型別；manifest.sha256 的雜湊等於檔案內容。
- 斷電（沒 stop）後會期仍可讀；clocks.jsonl 至少每 clock_period_s 一筆。
- UBX-CFG-VALSET 位元組（手算校驗）與 key ID 表（對照 UBX-22008968 R01）。
"""
from __future__ import annotations

import hashlib
import json
import math
import struct
import threading

import pytest

from payload.pi_capture import capture as C
from payload.pi_capture import f9p_config as F
from payload.pi_capture import imu as I
from payload.pi_capture import session as S

SEC = 1_000_000_000
MS = 1_000_000
BOOT_OFF = 7 * SEC          # BOOTTIME − MONOTONIC（系統休眠過 7 s）：轉換漏了就會挑錯影格

# F1 frames.jsonl 的鍵與型別（獨立寫一份，不引用 session.FRAME_KEYS）
F1_FRAME_TYPES = {
    "seq": (int,), "edge_ns": (int,), "sensor_ts_ns": (int, type(None)),
    "exposure_us": (int, type(None)), "frame_duration_us": (int, type(None)),
    "analogue_gain": (float, type(None)), "file": (str, type(None)),
    "bytes": (int, type(None)), "sha256": (str, type(None)), "encode_ms": (float, type(None)),
    "temp_c": (float, type(None)), "throttled": (int, type(None)), "status": (str,),
}
F1_FRAME_ORDER = ["seq", "edge_ns", "sensor_ts_ns", "exposure_us", "frame_duration_us",
                  "analogue_gain", "file", "bytes", "sha256", "encode_ms", "temp_c",
                  "throttled", "status"]


# ── 假後端 ────────────────────────────────────────────────────────────────────

class FakeClock:
    def __init__(self, mono=100 * SEC, boot_off=BOOT_OFF, real=1_760_000_000 * SEC, auto_step=0):
        self.mono, self.boot_off, self.real0, self.mono0 = mono, boot_off, real, mono
        self.auto_step = auto_step

    def monotonic_ns(self):
        v = self.mono
        self.mono += self.auto_step
        return v

    def sample(self):
        return S.ClockSample(self.mono, self.mono + self.boot_off, self.real0 + self.mono - self.mono0)

    def set(self, mono):
        self.mono = mono


class FakeFrame:
    def __init__(self, mono_ts, payload=None, *, exposure_us=800, frame_duration_us=100_000,
                 analogue_gain=2.0, fail_encode=False, block: threading.Event | None = None):
        self.sensor_ts_ns = mono_ts + BOOT_OFF          # SensorTimestamp 是 BOOTTIME
        self.payload = payload if payload is not None else f"JPEG@{mono_ts}".encode()
        self.exposure_us = exposure_us
        self.frame_duration_us = frame_duration_us
        self.analogue_gain = analogue_gain
        self.fail_encode = fail_encode
        self.block = block
        self.released = 0
        self.encoded_quality = None

    def encode(self, quality):
        if self.block is not None:
            assert self.block.wait(5.0)
        if self.fail_encode:
            raise RuntimeError("encoder boom")
        self.encoded_quality = quality
        return self.payload

    def release(self):
        self.released += 1


class FakeCamera:
    def __init__(self, buffer_count=4, clock=None, auto=False, fail_start=False):
        self.buffer_count = buffer_count
        self.cb = None
        self.clock = clock
        self.auto = auto
        self.fail_start = fail_start
        self.frames = []
        self.stopped = False

    def start(self, cb):
        if self.fail_start:
            raise RuntimeError("no camera")
        self.cb = cb

    def stop(self):
        self.stopped = True

    def poll(self):                    # 自檢用：每 poll 一次就出一張「現在」的影格
        if self.auto and self.cb is not None:
            f = FakeFrame(self.clock.mono + 1 * MS)
            self.frames.append(f)
            self.cb(f)


class FakeEdges:
    clock = "monotonic"

    def __init__(self):
        self.cb = None
        self.stopped = False

    def start(self, cb):
        self.cb = cb

    def stop(self):
        self.stopped = True


class FakeHealth:
    def __init__(self, temp=45.0, throttled=0):
        self.temp = temp
        self.thr = throttled

    def temp_c(self):
        return self.temp

    def throttled(self):
        return self.thr


class FakeReady:
    def __init__(self):
        self.calls = []

    def set(self, v):
        self.calls.append(bool(v))


class FailingImageStorage(S.LocalStorage):
    def write_atomic(self, rel, data):
        if rel.startswith("images/"):
            raise OSError(28, "No space left on device")
        super().write_atomic(rel, data)


class FixedFreeStorage(S.LocalStorage):
    def __init__(self, root, free):
        super().__init__(root)
        self.free = free

    def free_bytes(self):
        return self.free


def make_meta(sid="HERO2_20261014T010203Z", **kw):
    cam = dict(sensor="imx477", lens_mm=6.0, pixel_um=1.55, width=4056, height=3040,
               readout_ms=None, shutter="rolling", kernel="6.6.51+rpt-rpi-2712",
               dtoverlay="imx477,link-frequency=450000000")
    cam.update(kw.pop("camera", {}))
    args = dict(session_id=sid, uav="HERO2", unit_model="RPiHQ6-HERO2",
                camera=S.CameraInfo(**cam), lever_arm_m=[0.12, -0.03, 0.08],
                gnss_lever_arm_m=None, software="aeroplan-pi-capture test",
                started_utc_approx="2026-10-14T01:02:03.000Z")
    args.update(kw)
    return S.SessionMeta(**args)


def make_svc(tmp_path, *, clock=None, camera=None, storage=None, **kw):
    clock = clock or FakeClock()
    camera = camera or FakeCamera()
    storage = storage or S.LocalStorage(tmp_path / "sess")
    kw.setdefault("sync_writes", True)
    kw.setdefault("health", FakeHealth())
    svc = C.CaptureService(storage, make_meta(), camera, FakeEdges(), clock, **kw)
    svc.start()
    return svc, clock, camera, storage


def frames_of(storage):
    recs, bad = S.read_jsonl(storage.path("frames.jsonl"))
    assert bad == []
    return recs


T0 = 100 * SEC


# ── 挑幀 ──────────────────────────────────────────────────────────────────────

def test_edge_picks_first_frame_at_or_after_edge_and_releases_others(tmp_path):
    svc, clock, cam, st = make_svc(tmp_path)
    before = FakeFrame(T0 - 50 * MS)
    after1 = FakeFrame(T0 + 20 * MS)
    after2 = FakeFrame(T0 + 120 * MS)
    svc.handle_frame(before)
    svc.handle_edge(T0)
    assert before.released == 0                       # 還是回看那一張，下一張到才放
    svc.handle_frame(after1)
    assert before.released == 1                       # 早於邊緣的影格放掉、從未編碼
    assert before.encoded_quality is None
    assert after1.encoded_quality == 95 and after1.released == 1
    svc.handle_frame(after2)
    assert after2.released == 0 and after2.encoded_quality is None
    summary = svc.stop()
    assert after2.released == 1
    recs = frames_of(st)
    assert len(recs) == 1
    r = recs[0]
    assert r["seq"] == 1 and r["edge_ns"] == T0
    assert r["sensor_ts_ns"] == T0 + 20 * MS + BOOT_OFF     # 原始 BOOTTIME 值
    assert r["status"] == "ok" and r["file"] == "images/000001.jpg"
    assert r["bytes"] == len(after1.payload)
    assert r["sha256"] == hashlib.sha256(after1.payload).hexdigest()
    assert st.path("images/000001.jpg").read_bytes() == after1.payload
    assert summary == {"edges": 1, "ok": 1, "no_frame": 0, "write_error": 0,
                       "stopped_utc_approx": summary["stopped_utc_approx"]}


def test_frame_exactly_at_edge_time_is_accepted(tmp_path):
    svc, clock, cam, st = make_svc(tmp_path)
    svc.handle_edge(T0)
    f = FakeFrame(T0)                                 # 等於邊緣時刻也算（≥）
    svc.handle_frame(f)
    svc.stop()
    assert frames_of(st)[0]["sensor_ts_ns"] == T0 + BOOT_OFF


def test_frame_one_ns_before_edge_is_not_used(tmp_path):
    svc, clock, cam, st = make_svc(tmp_path)
    svc.handle_edge(T0)
    early = FakeFrame(T0 - 1)
    svc.handle_frame(early)
    late = FakeFrame(T0 + 100 * MS)
    svc.handle_frame(late)
    svc.stop()
    r = frames_of(st)[0]
    assert r["sensor_ts_ns"] == T0 + 100 * MS + BOOT_OFF
    assert early.encoded_quality is None and early.released == 1


def test_boottime_offset_is_applied_before_comparing(tmp_path):
    # 影格 BOOTTIME 比邊緣大（因為 +7 s 偏移），但換回 MONOTONIC 後早於邊緣 ⇒ 不能被選
    svc, clock, cam, st = make_svc(tmp_path)
    svc.handle_edge(T0)
    f_before = FakeFrame(T0 - 30 * MS)
    svc.handle_frame(f_before)
    assert f_before.encoded_quality is None
    svc.stop()
    assert frames_of(st)[0]["status"] == "no_frame"


def test_edge_delivered_after_its_frame_uses_lookback(tmp_path):
    svc, clock, cam, st = make_svc(tmp_path)
    f = FakeFrame(T0 + 15 * MS)
    svc.handle_frame(f)                               # 影格先到
    svc.handle_edge(T0)                               # 邊緣事件晚送到
    assert f.encoded_quality == 95
    svc.stop()
    r = frames_of(st)[0]
    assert r["status"] == "ok" and r["sensor_ts_ns"] == T0 + 15 * MS + BOOT_OFF
    assert svc.late_edges == 0


def test_late_edge_whose_first_frame_was_already_released_is_counted(tmp_path):
    svc, clock, cam, st = make_svc(tmp_path)
    f1, f2 = FakeFrame(T0 + 10 * MS), FakeFrame(T0 + 110 * MS)
    svc.handle_frame(f1)
    svc.handle_frame(f2)                              # f1 被釋放
    svc.handle_edge(T0)
    assert svc.late_edges == 1
    assert f2.encoded_quality == 95                   # 退而求其次用 f2，時刻照實記
    svc.stop()
    assert frames_of(st)[0]["sensor_ts_ns"] == T0 + 110 * MS + BOOT_OFF


def test_two_close_edges_get_distinct_frames_in_order(tmp_path):
    svc, clock, cam, st = make_svc(tmp_path)
    svc.handle_edge(T0)
    svc.handle_edge(T0 + 10 * MS)
    fa, fb = FakeFrame(T0 + 20 * MS), FakeFrame(T0 + 120 * MS)
    svc.handle_frame(fa)
    svc.handle_frame(fb)
    svc.stop()
    recs = frames_of(st)
    assert [(r["seq"], r["sensor_ts_ns"], r["file"]) for r in recs] == [
        (1, T0 + 20 * MS + BOOT_OFF, "images/000001.jpg"),
        (2, T0 + 120 * MS + BOOT_OFF, "images/000002.jpg"),
    ]


def test_missing_frame_times_out_to_no_frame(tmp_path):
    svc, clock, cam, st = make_svc(tmp_path, frame_timeout_ms=500)
    svc.handle_edge(T0)
    clock.set(T0 + 499 * MS)
    svc.tick()
    assert frames_of(st) == []                        # 還沒逾時
    clock.set(T0 + 500 * MS)
    svc.tick()
    recs = frames_of(st)
    assert len(recs) == 1
    r = recs[0]
    assert r["status"] == "no_frame" and r["seq"] == 1 and r["edge_ns"] == T0
    assert all(r[k] is None for k in ("sensor_ts_ns", "exposure_us", "frame_duration_us",
                                       "analogue_gain", "file", "bytes", "sha256", "encode_ms"))
    late = FakeFrame(T0 + 600 * MS)                   # 逾時之後才來的影格不能再配給它
    svc.handle_frame(late)
    assert late.encoded_quality is None
    svc.handle_edge(T0 + 700 * MS)                    # 下一個邊緣照常
    svc.handle_frame(FakeFrame(T0 + 750 * MS))
    s = svc.stop()
    assert [r["status"] for r in frames_of(st)] == ["no_frame", "ok"]
    assert s["edges"] == 2 and s["ok"] == 1 and s["no_frame"] == 1


def test_pending_edges_become_no_frame_on_clean_stop(tmp_path):
    svc, clock, cam, st = make_svc(tmp_path)
    svc.handle_edge(T0)
    s = svc.stop()
    assert [r["status"] for r in frames_of(st)] == ["no_frame"]
    assert s["no_frame"] == 1 and s["edges"] == 1


def test_queued_events_are_processed_by_run_once(tmp_path):
    svc, clock, cam, st = make_svc(tmp_path)
    svc._on_edge_event(T0, 41)
    cam.cb(FakeFrame(T0 + 5 * MS))
    svc.run_once(0)
    svc.stop()
    assert [r["status"] for r in frames_of(st)] == ["ok"]


def test_kernel_line_seqno_gap_is_counted(tmp_path):
    svc, clock, cam, st = make_svc(tmp_path)
    svc.handle_edge(T0, 5)
    svc.handle_edge(T0 + SEC, 6)
    svc.handle_edge(T0 + 2 * SEC, 9)                  # 漏了 7、8
    assert svc.lost_edge_events == 2
    svc.stop()


def test_writer_full_drops_frame_and_releases_it(tmp_path):
    gate = threading.Event()
    svc, clock, cam, st = make_svc(tmp_path, sync_writes=False, max_inflight_writes=1,
                                   camera=FakeCamera(buffer_count=3))
    f1 = FakeFrame(T0 + 5 * MS, block=gate)
    svc.handle_edge(T0)
    svc.handle_frame(f1)                              # 寫入執行緒卡在編碼
    svc.handle_edge(T0 + SEC)
    f2 = FakeFrame(T0 + SEC + 5 * MS, exposure_us=900)
    svc.handle_frame(f2)
    assert f2.released == 1 and f2.encoded_quality is None
    gate.set()
    s = svc.stop()
    recs = frames_of(st)
    assert [(r["seq"], r["status"]) for r in recs] == [(1, "ok"), (2, "dropped")]
    d = recs[1]
    assert d["sensor_ts_ns"] == T0 + SEC + 5 * MS + BOOT_OFF and d["exposure_us"] == 900
    assert d["file"] is None and d["bytes"] is None and d["sha256"] is None
    assert f1.released == 1
    assert s["edges"] == 2 and s["ok"] == 1


def test_io_failure_is_write_error_and_unhealthy(tmp_path):
    st = FailingImageStorage(tmp_path / "sess")
    svc, clock, cam, _ = make_svc(tmp_path, storage=st)
    f = FakeFrame(T0 + 5 * MS)
    svc.handle_edge(T0)
    svc.handle_frame(f)
    assert f.released == 1
    ok, reasons = svc.evaluate_health(clock.mono)
    assert not ok and any("寫入錯誤" in x for x in reasons)
    s = svc.stop()
    r = frames_of(st)[0]
    assert r["status"] == "write_error"
    assert r["file"] is None and r["bytes"] is None and r["sha256"] is None
    assert r["sensor_ts_ns"] == T0 + 5 * MS + BOOT_OFF
    assert s["write_error"] == 1 and s["ok"] == 0
    assert list(st.path("images").iterdir()) == []


def test_encoder_failure_is_write_error(tmp_path):
    svc, clock, cam, st = make_svc(tmp_path)
    f = FakeFrame(T0 + 5 * MS, fail_encode=True)
    svc.handle_edge(T0)
    svc.handle_frame(f)
    svc.stop()
    r = frames_of(st)[0]
    assert r["status"] == "write_error" and r["encode_ms"] is None and f.released == 1


# ── F1 格式 ──────────────────────────────────────────────────────────────────

def _scenario_all_statuses(tmp_path):
    st = S.LocalStorage(tmp_path / "sess")
    svc, clock, cam, _ = make_svc(tmp_path, storage=st, health=FakeHealth(51.5, 0x50000))
    svc.handle_edge(T0)                                   # ok
    svc.handle_frame(FakeFrame(T0 + 5 * MS, analogue_gain=4))
    svc.handle_edge(T0 + SEC)                             # write_error
    svc.handle_frame(FakeFrame(T0 + SEC + 5 * MS, fail_encode=True))
    svc.handle_edge(T0 + 2 * SEC)                         # no_frame
    clock.set(T0 + 3 * SEC)
    svc.tick()
    svc.handle_edge(T0 + 4 * SEC)                         # ok
    svc.handle_frame(FakeFrame(T0 + 4 * SEC + 1 * MS))
    return svc, st


def test_frames_jsonl_matches_f1_schema_exactly(tmp_path):
    svc, st = _scenario_all_statuses(tmp_path)
    svc.stop()
    lines = st.path("frames.jsonl").read_text(encoding="utf-8").splitlines()
    assert len(lines) == 4
    recs = [json.loads(x) for x in lines]
    for r in recs:
        assert list(r.keys()) == F1_FRAME_ORDER
        for k, types in F1_FRAME_TYPES.items():
            assert type(r[k]) in types, (k, r[k])
        assert r["status"] in ("ok", "no_frame", "write_error", "dropped")
        assert r["temp_c"] == 51.5 and r["throttled"] == 0x50000
    assert [r["status"] for r in recs] == ["ok", "write_error", "no_frame", "ok"]
    assert [r["seq"] for r in recs] == [1, 2, 3, 4]
    ok = recs[0]
    assert ok["exposure_us"] == 800 and ok["frame_duration_us"] == 100_000
    assert ok["analogue_gain"] == 4.0 and isinstance(ok["analogue_gain"], float)
    assert isinstance(ok["encode_ms"], float) and ok["encode_ms"] >= 0
    assert recs[3]["file"] == "images/000004.jpg"


def test_session_json_matches_f1(tmp_path):
    svc, clock, cam, st = make_svc(tmp_path)
    svc.stop()
    sj = json.loads(st.path("session.json").read_text(encoding="utf-8"))
    assert list(sj.keys()) == ["format", "session_id", "uav", "unit_model", "camera",
                               "lever_arm_m", "gnss_lever_arm_m", "software",
                               "started_utc_approx"]
    assert sj["format"] == "aeroplan-pi-session/1"
    assert sj["unit_model"] == "RPiHQ6-HERO2"
    assert list(sj["camera"].keys()) == ["sensor", "lens_mm", "pixel_um", "width", "height",
                                         "readout_ms", "shutter", "kernel", "dtoverlay"]
    assert sj["camera"]["width"] == 4056 and sj["camera"]["readout_ms"] is None
    assert sj["camera"]["lens_mm"] == 6.0 and sj["camera"]["shutter"] == "rolling"
    assert sj["lever_arm_m"] == [0.12, -0.03, 0.08] and sj["gnss_lever_arm_m"] is None
    summ = json.loads(st.path("summary.json").read_text(encoding="utf-8"))
    assert list(summ.keys()) == ["edges", "ok", "no_frame", "write_error", "stopped_utc_approx"]
    assert summ["stopped_utc_approx"].endswith("Z")


def test_session_meta_rejects_missing_or_bad_values():
    with pytest.raises(ValueError, match="lever_arm_m"):
        make_meta(lever_arm_m=None)
    with pytest.raises(ValueError, match="lever_arm_m"):
        make_meta(lever_arm_m=[0.1, 0.2])
    with pytest.raises(ValueError, match="gnss_lever_arm_m"):
        make_meta(gnss_lever_arm_m=[0.1, float("nan"), 0])
    with pytest.raises(ValueError, match="shutter"):
        make_meta(camera={"shutter": "electronic"})
    with pytest.raises(ValueError, match="readout_ms"):
        make_meta(camera={"readout_ms": 0})
    with pytest.raises(ValueError, match="width"):
        make_meta(camera={"width": 4056.0})
    m = make_meta(gnss_lever_arm_m=(0, 0, -0.1), camera={"readout_ms": 84.0})
    assert m.to_json()["gnss_lever_arm_m"] == [0.0, 0.0, -0.1]
    assert m.to_json()["camera"]["readout_ms"] == 84.0


def test_free_bytes_before_session_dir_exists(tmp_path):
    # 自檢在建立 <root>/selfcheck/<id> 之前就要量空間：不存在的路徑要往上找祖先，不能丟例外
    import shutil

    st = S.LocalStorage(tmp_path / "selfcheck" / "HERO2_x")
    assert not st.root.exists()
    assert abs(st.free_bytes() - shutil.disk_usage(tmp_path).free) < 100 * 10**6


def test_session_dir_is_never_overwritten(tmp_path):
    svc, clock, cam, st = make_svc(tmp_path)
    svc.stop()
    with pytest.raises(FileExistsError):
        C.CaptureService(S.LocalStorage(tmp_path / "sess"), make_meta(), FakeCamera(),
                         FakeEdges(), FakeClock(), sync_writes=True).start()


def test_manifest_lines_match_file_contents(tmp_path):
    svc, st = _scenario_all_statuses(tmp_path)
    svc.stop()
    lines = st.path("manifest.sha256").read_text(encoding="utf-8").splitlines()
    entries = []
    for line in lines:
        h, rel = line.split("  ", 1)
        assert len(h) == 64
        assert hashlib.sha256(st.path(rel).read_bytes()).hexdigest() == h
        entries.append(rel)
    assert entries == ["session.json", "images/000001.jpg", "images/000004.jpg", "summary.json"]
    recs = frames_of(st)
    by_file = {r["file"]: r["sha256"] for r in recs if r["status"] == "ok"}
    assert {rel: h for h, rel in (l.split("  ", 1) for l in lines) if rel.startswith("images/")} == by_file


def test_manifest_is_written_incrementally_before_stop(tmp_path):
    svc, clock, cam, st = make_svc(tmp_path)
    svc.handle_edge(T0)
    svc.handle_frame(FakeFrame(T0 + 1 * MS))
    lines = st.path("manifest.sha256").read_text(encoding="utf-8").splitlines()
    assert [l.split("  ", 1)[1] for l in lines] == ["session.json", "images/000001.jpg"]
    svc.abort()


def test_power_cut_without_stop_leaves_readable_session(tmp_path):
    svc, clock, cam, st = make_svc(tmp_path)
    for k in range(3):
        svc.handle_edge(T0 + k * SEC)
        svc.handle_frame(FakeFrame(T0 + k * SEC + 3 * MS))
    rep = S.verify_session(st.root)                  # 尚未 stop：資料已經在磁碟上
    assert rep["ok"], rep["errors"]
    assert rep["has_summary"] is False and rep["edges"] == 3 and rep["counts"]["ok"] == 3
    assert rep["manifest_lines"] == 4 and rep["clock_samples"] == 1
    with open(st.path("frames.jsonl"), "ab") as f:   # 斷電時寫到一半的最後一列
        f.write(b'{"seq":4,"edge_ns":10')
    rep = S.verify_session(st.root)
    assert rep["ok"] and rep["torn_last_line"] is True and rep["edges"] == 3
    svc.abort()
    assert not st.path("summary.json").exists()


def test_verify_session_detects_tampering_and_gaps(tmp_path):
    svc, clock, cam, st = make_svc(tmp_path)
    for k in range(2):
        svc.handle_edge(T0 + k * SEC)
        svc.handle_frame(FakeFrame(T0 + k * SEC + 3 * MS))
    svc.stop()
    assert S.verify_session(st.root)["ok"]
    st.path("images/000002.jpg").write_bytes(b"tampered")
    rep = S.verify_session(st.root)
    assert not rep["ok"] and any("000002" in e for e in rep["errors"])
    lines = st.path("frames.jsonl").read_text(encoding="utf-8").splitlines()
    st.path("frames.jsonl").write_text(lines[1] + "\n", encoding="utf-8")   # 刪掉 seq 1
    rep = S.verify_session(st.root)
    assert any("seq 不是從 1 起連續" in e for e in rep["errors"])
    assert any("summary.json 與 frames.jsonl 計數不符" in e for e in rep["errors"])


def test_clocks_jsonl_cadence(tmp_path):
    clock = FakeClock()
    svc, clock, cam, st = make_svc(tmp_path, clock=clock, clock_period_s=5.0)
    step = SEC // 4
    for i in range(1, 241):                            # 60 s，每 0.25 s 一次 tick
        clock.set(T0 + i * step)
        svc.tick()
    svc.stop()
    recs, bad = S.read_jsonl(st.path("clocks.jsonl"))
    assert bad == []
    mono = [r["monotonic_ns"] for r in recs]
    assert mono[0] == T0 and mono[-1] == T0 + 60 * SEC
    gaps = [b - a for a, b in zip(mono, mono[1:])]
    assert max(gaps) <= 5 * SEC
    assert len(recs) == 1 + 12 + 1                     # 開始 + 每 5 s + 結束
    for r in recs:
        assert list(r.keys()) == ["monotonic_ns", "boottime_ns", "realtime_ns"]
        assert r["boottime_ns"] - r["monotonic_ns"] == BOOT_OFF


def test_constructor_rejects_unsafe_settings(tmp_path):
    def mk(**kw):
        edges = kw.pop("edges", FakeEdges())
        cam = kw.pop("camera", FakeCamera())
        return C.CaptureService(S.LocalStorage(tmp_path / "x"), make_meta(), cam, edges,
                                FakeClock(), **kw)

    with pytest.raises(ValueError, match="F1"):
        mk(clock_period_s=10.5)
    mk(clock_period_s=10.0)
    boot_edges = FakeEdges()
    boot_edges.clock = "boottime"
    with pytest.raises(ValueError, match="MONOTONIC"):
        mk(edges=boot_edges)
    with pytest.raises(ValueError, match="buffer_count"):
        mk(camera=FakeCamera(buffer_count=3), max_inflight_writes=2)
    mk(camera=FakeCamera(buffer_count=4), max_inflight_writes=2)
    with pytest.raises(ValueError, match="buffer_count"):
        mk(camera=FakeCamera(buffer_count=2), max_inflight_writes=1)
    with pytest.raises(ValueError, match="health"):
        mk(ready=FakeReady())
    with pytest.raises(ValueError, match="jpeg_quality"):
        mk(jpeg_quality=0)
    with pytest.raises(ValueError, match="frame_timeout_ms"):
        mk(frame_timeout_ms=0)


def test_jpeg_quality_is_passed_to_encoder(tmp_path):
    svc, clock, cam, st = make_svc(tmp_path, jpeg_quality=88)
    f = FakeFrame(T0 + 1 * MS)
    svc.handle_edge(T0)
    svc.handle_frame(f)
    svc.stop()
    assert f.encoded_quality == 88


# ── 健康與就緒腳 ──────────────────────────────────────────────────────────────

def test_ready_output_high_only_while_healthy(tmp_path):
    health = FakeHealth(temp=50.0)
    ready = FakeReady()
    st = FixedFreeStorage(tmp_path / "sess", free=10 * 10**9)
    svc, clock, cam, _ = make_svc(tmp_path, storage=st, health=health, ready=ready,
                                  min_free_bytes=5 * 10**9, max_temp_c=80.0)
    assert ready.calls[-1] is False and "尚未收到任何影格" in svc.health_reasons
    svc.handle_frame(FakeFrame(T0))
    clock.set(T0 + SEC)
    svc.tick()
    assert ready.calls[-1] is True and svc.healthy
    health.temp = 80.0
    clock.set(T0 + 1_500 * MS)
    svc.tick()
    assert ready.calls[-1] is True                    # 每秒才評估一次
    clock.set(T0 + 2 * SEC)
    svc.handle_frame(FakeFrame(T0 + 2 * SEC))
    svc.tick()
    assert ready.calls[-1] is False and any("溫度" in r for r in svc.health_reasons)
    health.temp = 50.0
    clock.set(T0 + 3 * SEC)
    svc.handle_frame(FakeFrame(T0 + 3 * SEC))
    svc.tick()
    assert ready.calls[-1] is True
    clock.set(T0 + 5_001 * MS)                        # 相機 2 s 沒出幀
    svc.tick()
    assert ready.calls[-1] is False and any("沒有出幀" in r for r in svc.health_reasons)
    clock.set(T0 + 6_500 * MS)
    svc.handle_frame(FakeFrame(T0 + 6_500 * MS))
    svc.tick()
    assert ready.calls[-1] is True                    # 相機恢復
    st.free = 4 * 10**9
    clock.set(T0 + 8 * SEC)
    svc.handle_frame(FakeFrame(T0 + 8 * SEC))
    svc.tick()
    assert ready.calls[-1] is False and svc.health_reasons == ["剩餘空間 4000000000 B < 5000000000 B"]
    svc.stop()
    assert ready.calls[-1] is False


def test_failed_startup_gate_keeps_ready_low(tmp_path):
    ready = FakeReady()
    svc, clock, cam, _ = make_svc(tmp_path, ready=ready, startup_gate_ok=False)
    svc.handle_frame(FakeFrame(T0))
    clock.set(T0 + SEC)
    svc.tick()
    assert True not in ready.calls and "開機自檢未通過" in svc.health_reasons
    svc.stop()


def test_unknown_temperature_is_unhealthy(tmp_path):
    svc, clock, cam, _ = make_svc(tmp_path, health=FakeHealth(temp=None))
    svc.handle_frame(FakeFrame(T0))
    ok, reasons = svc.evaluate_health(T0)
    assert not ok and "溫度未知" in reasons
    svc.stop()


# ── 起飛前自檢 ────────────────────────────────────────────────────────────────

def _selfcheck(tmp_path, *, free=50 * 10**9, temp=45.0, camera=None, **kw):
    clock = FakeClock(auto_step=10 * MS)
    camera = camera or FakeCamera(clock=clock, auto=True)
    st = FixedFreeStorage(tmp_path / "selfcheck", free)
    kw.setdefault("planned_photos", 2000)
    kw.setdefault("bytes_per_photo", 9e6)
    kw.setdefault("frame_wait_s", 0.5)
    res = C.run_selfcheck(storage=st, meta=make_meta(), camera=camera, edges=FakeEdges(),
                          clock=clock, health=FakeHealth(temp=temp), **kw)
    return res, {c["name"]: c for c in res["checks"]}, st


def test_selfcheck_passes_with_working_fakes(tmp_path):
    res, ch, st = _selfcheck(tmp_path)
    assert res["ok"] is True and res["edge_source"] == "synthetic"
    assert [c["name"] for c in res["checks"]] == ["free_space", "temperature", "camera_open",
                                                  "test_edge"]
    assert ch["free_space"]["limit"] == 27_000_000_000 == res["required_bytes"]
    assert ch["test_edge"]["value"] == "ok"
    assert S.verify_session(st.root)["ok"]


def test_selfcheck_free_space_threshold_is_1_5x(tmp_path):
    need = math.ceil(1.5 * 2000 * 9e6)
    res, ch, _ = _selfcheck(tmp_path / "a", free=need - 1)
    assert ch["free_space"]["ok"] is False and res["ok"] is False
    assert ch["free_space"]["value"] == need - 1 and ch["free_space"]["limit"] == need
    res, ch, _ = _selfcheck(tmp_path / "b", free=need)
    assert ch["free_space"]["ok"] is True and res["ok"] is True


def test_selfcheck_temperature_failures(tmp_path):
    res, ch, _ = _selfcheck(tmp_path / "hot", temp=80.0, max_temp_c=80.0)
    assert ch["temperature"]["ok"] is False and res["ok"] is False
    res, ch, _ = _selfcheck(tmp_path / "unknown", temp=None)
    assert ch["temperature"]["ok"] is False
    res, ch, _ = _selfcheck(tmp_path / "ok", temp=79.9, max_temp_c=80.0)
    assert ch["temperature"]["ok"] is True


def test_selfcheck_camera_failures(tmp_path):
    res, ch, _ = _selfcheck(tmp_path / "nostart", camera=FakeCamera(fail_start=True))
    assert ch["camera_open"]["ok"] is False and ch["test_edge"]["ok"] is False
    assert res["ok"] is False
    silent = FakeCamera(clock=FakeClock(), auto=False)
    res, ch, _ = _selfcheck(tmp_path / "silent", camera=silent)
    assert ch["camera_open"]["ok"] is False and ch["camera_open"]["value"] == 0
    assert ch["test_edge"]["ok"] is False


def test_selfcheck_external_edge_timeout(tmp_path):
    res, ch, _ = _selfcheck(tmp_path, external_edge_timeout_s=0.2)
    assert res["edge_source"] == "external"
    assert ch["camera_open"]["ok"] is True
    assert ch["test_edge"]["ok"] is False and "外部邊緣" in ch["test_edge"]["detail"]


def test_selfcheck_write_failure(tmp_path):
    clock = FakeClock(auto_step=10 * MS)
    st = FailingImageStorage(tmp_path / "sc")
    res = C.run_selfcheck(storage=st, meta=make_meta(), camera=FakeCamera(clock=clock, auto=True),
                          edges=FakeEdges(), clock=clock, health=FakeHealth(),
                          planned_photos=10, bytes_per_photo=1.0, frame_wait_s=0.5)
    ch = {c["name"]: c for c in res["checks"]}
    assert ch["test_edge"]["ok"] is False and ch["test_edge"]["value"] == "write_error"


def test_selfcheck_rejects_bad_inputs(tmp_path):
    for kw in ({"planned_photos": 0}, {"bytes_per_photo": 0}, {"planned_photos": 1.5},
               {"bytes_per_photo": float("nan")}, {"free_space_factor": 0.9}):
        with pytest.raises(ValueError):
            _selfcheck(tmp_path, **kw)


# ── JPEG 編碼（Pillow 真的跑）─────────────────────────────────────────────────

def test_encode_jpeg_writes_unique_model_exif():
    from PIL import Image
    import io

    img = Image.new("RGB", (64, 48), (10, 200, 30))
    data = C.encode_jpeg(img, 90, make="RaspberryPi", model="RPiHQ6-HERO2", lens_mm=6.0)
    assert data[:2] == b"\xff\xd8"
    ex = Image.open(io.BytesIO(data)).getexif()
    assert ex.get(0x010F) == "RaspberryPi" and ex.get(0x0110) == "RPiHQ6-HERO2"
    assert float(ex.get_ifd(0x8769)[0x920A]) == 6.0
    lo = C.encode_jpeg(img, 10, make="RaspberryPi", model="m", lens_mm=6.0)
    assert len(lo) < len(data)


# ── UBX / ZED-F9P ────────────────────────────────────────────────────────────

# UBX-22008968 R01（u-blox F9 HPG 1.32 Interface Description）各表抄錄
EXPECTED_KEYS = {
    "CFG-RATE-MEAS": (0x30210001, "U2"), "CFG-RATE-NAV": (0x30210002, "U2"),
    "CFG-RATE-TIMEREF": (0x20210003, "E1"), "CFG-TP-TIMEGRID_TP1": (0x2005000C, "E1"),
    "CFG-MSGOUT-UBX_RXM_RAWX_UART1": (0x209102A5, "U1"),
    "CFG-MSGOUT-UBX_RXM_RAWX_UART2": (0x209102A6, "U1"),
    "CFG-MSGOUT-UBX_RXM_RAWX_USB": (0x209102A7, "U1"),
    "CFG-MSGOUT-UBX_RXM_SFRBX_UART1": (0x20910232, "U1"),
    "CFG-MSGOUT-UBX_RXM_SFRBX_UART2": (0x20910233, "U1"),
    "CFG-MSGOUT-UBX_RXM_SFRBX_USB": (0x20910234, "U1"),
    "CFG-MSGOUT-UBX_TIM_TM2_UART1": (0x20910179, "U1"),
    "CFG-MSGOUT-UBX_TIM_TM2_UART2": (0x2091017A, "U1"),
    "CFG-MSGOUT-UBX_TIM_TM2_USB": (0x2091017B, "U1"),
    "CFG-MSGOUT-UBX_NAV_PVT_UART1": (0x20910007, "U1"),
    "CFG-MSGOUT-UBX_NAV_PVT_UART2": (0x20910008, "U1"),
    "CFG-MSGOUT-UBX_NAV_PVT_USB": (0x20910009, "U1"),
    "CFG-UART1OUTPROT-UBX": (0x10740001, "L"), "CFG-UART1OUTPROT-NMEA": (0x10740002, "L"),
    "CFG-UART2OUTPROT-UBX": (0x10760001, "L"), "CFG-UART2OUTPROT-NMEA": (0x10760002, "L"),
    "CFG-USBOUTPROT-UBX": (0x10780001, "L"), "CFG-USBOUTPROT-NMEA": (0x10780002, "L"),
    "CFG-UART1-BAUDRATE": (0x40520001, "U4"), "CFG-UART2-BAUDRATE": (0x40530001, "U4"),
}


def test_key_id_table_matches_interface_description():
    assert {k: (v.key_id, v.type) for k, v in F.KEYS.items()} == EXPECTED_KEYS
    size = {"L": 1, "U1": 1, "E1": 1, "U2": 2, "U4": 4}
    for name, (kid, typ) in EXPECTED_KEYS.items():
        assert F.key_size_bytes(kid) == size[typ], name
    assert F.key_size_bytes(0x10740001) == 1 and (0x10740001 >> 28) & 7 == 1


def test_ubx_checksum_known_vector():
    # UBX-MON-VER 輪詢（class 0x0A id 0x04、空 payload）：手算 CK_A=0x0E、CK_B=0x34
    assert F.ubx_frame(0x0A, 0x04, b"") == bytes.fromhex("b5620a0400000e34")


def test_valset_rate_meas_hand_checked_bytes():
    # payload = version 00、layers 01(RAM)、reserved 00 00、key 0x30210001 小端、值 100(U2)
    # 手算 Fletcher（class..payload）：CK_A=0x51、CK_B=0xB9
    frame = F.valset([("CFG-RATE-MEAS", 100)], layers=F.LAYER_RAM)
    assert frame == bytes.fromhex("b562068a0a00 00010000 01002130 6400 51b9")


def _decode_valset(frame):
    assert frame[:4] == b"\xb5\x62\x06\x8a"
    n = struct.unpack_from("<H", frame, 4)[0]
    payload = frame[6:6 + n]
    assert len(frame) == 8 + n
    assert F.ubx_checksum(frame[2:6 + n]) == frame[6 + n:]
    version, layers = payload[0], payload[1]
    assert payload[2:4] == b"\x00\x00"
    out, i = {}, 4
    while i < len(payload):
        kid = struct.unpack_from("<I", payload, i)[0]
        sz = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8}[(kid >> 28) & 7]
        out[kid] = int.from_bytes(payload[i + 4:i + 4 + sz], "little")
        i += 4 + sz
    return version, layers, out


def test_rover_config_usb_contents():
    msgs = F.build_rover_config("USB")
    assert len(msgs) == 1 and msgs[0].expect_ack is True
    version, layers, kv = _decode_valset(msgs[0].frame)
    assert version == 0 and layers == 0x07                 # RAM + BBR + Flash
    assert kv == {
        0x30210001: 100,      # CFG-RATE-MEAS 100 ms
        0x30210002: 1,        # CFG-RATE-NAV 每次量測都解
        0x20210003: 1,        # CFG-RATE-TIMEREF GPS
        0x2005000C: 1,        # CFG-TP-TIMEGRID_TP1 GPS（TIM-TM2 時間基準）
        0x10780001: 1,        # USB 輸出 UBX
        0x10780002: 0,        # USB 關 NMEA
        0x209102A7: 1, 0x20910234: 1, 0x2091017B: 1, 0x20910009: 1,
    }


def test_rover_config_uart_requires_enough_baud():
    with pytest.raises(ValueError, match="鮑率"):
        F.build_rover_config("UART1")
    for slow in (38400, 115200, 230400):
        with pytest.raises(ValueError, match="B/s"):
            F.build_rover_config("UART1", baudrate=slow)
    # UART 8N1 每 byte 10 bit：230400 baud × 80% ⇒ 18432 B/s；估計需要 35320 B/s
    with pytest.raises(ValueError, match="可用 18432 B/s.*估計需要 35320 B/s"):
        F.build_rover_config("UART1", baudrate=230400)
    with pytest.raises(ValueError, match="鮑率"):
        F.build_rover_config("UART1", baudrate=500000)
    msgs = F.build_rover_config("UART2", baudrate=460800, layers=F.LAYER_RAM | F.LAYER_FLASH)
    assert [m.expect_ack for m in msgs] == [True, False]
    v, layers, kv = _decode_valset(msgs[0].frame)
    assert layers == 0x05
    assert kv[0x10760001] == 1 and kv[0x10760002] == 0
    assert {0x209102A6, 0x20910233, 0x2091017A, 0x20910008} <= set(kv)
    assert 0x40530001 not in kv
    v, layers, kv = _decode_valset(msgs[1].frame)
    assert kv == {0x40530001: 460800}
    assert msgs[1].frame[14:18] == bytes.fromhex("00080700")      # 460800 小端
    with pytest.raises(ValueError, match="B/s"):                      # 15 Hz 要 921600
        F.build_rover_config("UART1", baudrate=460800, meas_rate_ms=67)
    assert len(F.build_rover_config("UART1", baudrate=921600, meas_rate_ms=67)) == 2


def test_rover_config_rejects_bad_arguments():
    with pytest.raises(ValueError, match="USB"):
        F.build_rover_config("USB", baudrate=460800)
    with pytest.raises(ValueError, match="meas_rate_ms"):
        F.build_rover_config("USB", meas_rate_ms=66)
    with pytest.raises(ValueError, match="meas_rate_ms"):
        F.build_rover_config("USB", meas_rate_ms=1001)
    with pytest.raises(ValueError, match="port"):
        F.build_rover_config("SPI")
    with pytest.raises(ValueError, match="拍照間隔"):
        F.build_rover_config("USB", meas_rate_ms=500, min_photo_interval_s=0.5)
    assert F.build_rover_config("USB", meas_rate_ms=100, min_photo_interval_s=0.5)
    _, _, kv = _decode_valset(F.build_rover_config("USB", meas_rate_ms=67)[0].frame)
    assert kv[0x30210001] == 67


def test_estimated_bandwidth_numbers():
    # RAWX 8+16+32·96=3096、PVT 100、TM2 36 ⇒ 3232 B/epoch；10 Hz ⇒ 32320 + SFRBX 3000
    assert F.estimate_rover_bytes_per_s(100) == pytest.approx(35_320.0)
    assert F.estimate_rover_bytes_per_s(1000) == pytest.approx(6_232.0)


def test_valset_value_and_layer_validation():
    with pytest.raises(ValueError):
        F.valset([("CFG-MSGOUT-UBX_RXM_RAWX_USB", 256)])
    with pytest.raises(ValueError):
        F.valset([("CFG-MSGOUT-UBX_RXM_RAWX_USB", -1)])
    with pytest.raises(ValueError):
        F.valset([("CFG-USBOUTPROT-NMEA", 2)])
    with pytest.raises(ValueError):
        F.valset([("CFG-RATE-MEAS", 100)], layers=0)
    with pytest.raises(ValueError):
        F.valset([("CFG-RATE-MEAS", 100)], layers=0x08)
    with pytest.raises(ValueError):
        F.valset([("CFG-RATE-MEAS", 100)] * 65)
    assert F.valset([("CFG-RATE-MEAS", 65535)], layers=1)[14:16] == b"\xff\xff"
    with pytest.raises(ValueError):
        F.valset([("CFG-RATE-MEAS", 65536)], layers=1)


class FakeStream:
    def __init__(self, chunks=()):
        self.chunks = list(chunks)
        self.written = b""

    def write(self, b):
        self.written += b

    def flush(self):
        pass

    def read(self, n):
        return self.chunks.pop(0) if self.chunks else b""


def _ack(ok=True):
    return F.ubx_frame(0x05, 0x01 if ok else 0x00, b"\x06\x8a")


def test_apply_config_waits_for_ack_and_raises_on_nak_or_timeout():
    msgs = F.build_rover_config("UART1", baudrate=460800)
    nmea = b"$GNGGA,000000.00,,,,,0,00,99.99,,,,,,*7A\r\n"
    full = _ack()
    st = FakeStream([nmea + full[:5], full[5:]])          # ACK 跨兩次讀取、前面混 NMEA
    rep = F.apply_config(st, msgs, ack_timeout_s=1.0)
    assert st.written == msgs[0].frame + msgs[1].frame
    assert rep[0].startswith("ACK") and "不等 ACK" in rep[1]
    other_ack = F.ubx_frame(0x05, 0x01, b"\x06\x8b")     # 別則訊息的 ACK 不算數
    with pytest.raises(RuntimeError, match="NAK"):
        F.apply_config(FakeStream([other_ack, _ack(False)]), msgs[:1], ack_timeout_s=1.0)
    t = iter(range(100))
    with pytest.raises(TimeoutError):
        F.apply_config(FakeStream([other_ack]), msgs[:1], ack_timeout_s=3,
                       monotonic=lambda: next(t))


def test_ubx_reader_skips_bad_checksum():
    good = F.ubx_frame(0x0D, 0x03, bytes(28))
    bad = bytearray(good)
    bad[-1] ^= 0xFF
    r = F.UbxReader()
    out = r.feed(bytes(bad) + b"\x00junk" + good[:3])
    assert out == [] and r.bad_checksums == 1
    out = r.feed(good[3:])
    assert out == [(0x0D, 0x03, bytes(28))]


def test_rover_logger_writes_raw_bytes_and_fsyncs_periodically(tmp_path):
    st = S.LocalStorage(tmp_path / "sess")
    st.create()
    clock = FakeClock()
    chunks = [bytes([i]) * 100 for i in range(10)]
    stream = FakeStream(chunks)
    rl = F.RoverLogger(st, stream, clock, fsync_period_s=2.0, stall_warn_s=5.0)
    rl.open()
    for i in range(10):                               # 每 0.5 s 一塊
        clock.set(T0 + (i + 1) * SEC // 2)
        assert rl.step() == 100
        if i == 3:
            assert rl.fsyncs == 1                     # 剛好滿 2.0 s 就 fsync
    assert rl.fsyncs == 2                             # t=2.0 s、4.0 s
    for i in range(12):                               # 之後 6 s 沒資料
        clock.set(T0 + 5 * SEC + (i + 1) * SEC // 2)
        rl.step()
    assert rl.stall_events == 1 and rl.stalled
    rl.close()
    assert st.path("gnss/rover.ubx").read_bytes() == b"".join(chunks)
    assert rl.bytes == 1000
    with pytest.raises(FileExistsError):
        F.RoverLogger(st, FakeStream(), clock).open()


# ── IMU ──────────────────────────────────────────────────────────────────────

class FakeImu:
    def __init__(self, n_fail_at=()):
        self.i = 0
        self.n_fail_at = set(n_fail_at)

    def read(self):
        self.i += 1
        if self.i in self.n_fail_at:
            raise OSError("spi")
        return I.ImuSample(0.1 * self.i, -9.80665, 0.0, 0.001, -0.002, 1e-7, 36.5)


def test_imu_csv_header_rows_and_fsync(tmp_path):
    st = S.LocalStorage(tmp_path / "sess")
    st.create()
    clock = FakeClock()
    lg = I.ImuLogger(st, FakeImu(n_fail_at={3}), clock, rate_hz=400, fsync_period_s=1.0)
    lg.open()
    for k in range(5):
        clock.set(T0 + k * 2_500_000)
        lg.step()
    lg.close()
    lines = st.path("imu.csv").read_text(encoding="ascii").splitlines()
    assert lines[0] == "t_ns,ax,ay,az,gx,gy,gz,temp_c"
    assert len(lines) == 1 + 4 and lg.samples == 4 and lg.read_errors == 1
    first = lines[1].split(",")
    assert int(first[0]) == T0
    assert [float(x) for x in first[1:]] == [0.1, -9.80665, 0.0, 0.001, -0.002, 1e-7, 36.5]
    assert int(lines[3].split(",")[0]) == T0 + 3 * 2_500_000      # 第 3 次讀失敗，時間戳不錯位
    with pytest.raises(FileExistsError):
        I.ImuLogger(st, FakeImu(), clock).open()
    with pytest.raises(ValueError):
        I.ImuLogger(st, FakeImu(), clock, rate_hz=0)


def test_icm42688_conversion_hand_vector():
    raw = struct.pack(">7h", 0, 2048, -2048, 0, 164, 0, -1640)
    s = I.icm42688_convert(raw)
    assert s.temp_c == 25.0
    assert s.ax == pytest.approx(9.80665) and s.ay == pytest.approx(-9.80665) and s.az == 0.0
    assert s.gx == pytest.approx(math.radians(10.0))
    assert s.gz == pytest.approx(math.radians(-100.0))
    s = I.icm42688_convert(struct.pack(">7h", 1325, 0, 0, 0, 0, 0, 0))
    assert s.temp_c == pytest.approx(25.0 + 1325 / 132.48)
    with pytest.raises(ValueError):
        I.icm42688_convert(b"\x00" * 13)


# ── 命令列 ────────────────────────────────────────────────────────────────────

def test_cli_verify_and_f9p_dry_run(tmp_path, capsys):
    from payload.pi_capture import __main__ as M

    svc, clock, cam, st = make_svc(tmp_path)
    svc.handle_edge(T0)
    svc.handle_frame(FakeFrame(T0 + 1 * MS))
    svc.stop()
    assert M.main(["verify", str(st.root)]) == 0
    st.path("images/000001.jpg").write_bytes(b"x")
    assert M.main(["verify", str(st.root)]) == 1
    out = tmp_path / "cfg.ubx"
    assert M.main(["f9p-config", "--device", "unused", "--port", "USB", "--dry-run", str(out)]) == 0
    assert out.read_bytes() == F.build_rover_config("USB")[0].frame
    assert M.main(["f9p-config", "--device", "unused", "--port", "UART1", "--baud", "38400",
                   "--dry-run", str(out)]) == 2


def test_config_loader_rejects_unknown_and_missing_keys(tmp_path):
    from payload.pi_capture import __main__ as M

    base = {
        "storage_root": "/data", "uav": "HERO2", "unit_model": "RPiHQ6-HERO2",
        "camera": {"sensor": "imx477", "lens_mm": 6.0, "pixel_um": 1.55, "width": 4056,
                   "height": 3040, "readout_ms": None, "shutter": "rolling", "dtoverlay": "imx477"},
        "lever_arm_m": [0.1, 0.0, 0.05], "gnss_lever_arm_m": None,
        "capture": {"exposure_us": 800, "analogue_gain": 2.0, "colour_gains": [2.0, 1.8]},
        "trigger": {"chip": "pinctrl-rp1", "line": 17, "bias": "pull_down"},
        "ready": None, "health": {"max_temp_c": 80, "min_free_gb": 5}, "imu": None, "gnss": None,
    }
    p = tmp_path / "c.json"
    p.write_text(json.dumps(base), encoding="utf-8")
    cfg = M.load_config(p)
    assert cfg["capture"]["buffer_count"] == 4 and cfg["capture"]["jpeg_quality"] == 95
    bad = json.loads(json.dumps(base))
    bad["capture"]["exposure"] = 800                   # 打錯字
    p.write_text(json.dumps(bad), encoding="utf-8")
    with pytest.raises(M.ConfigError, match="exposure"):
        M.load_config(p)
    bad = json.loads(json.dumps(base))
    del bad["lever_arm_m"]
    p.write_text(json.dumps(bad), encoding="utf-8")
    with pytest.raises(M.ConfigError, match="lever_arm_m"):
        M.load_config(p)
    p.write_text(json.dumps(base), encoding="utf-8")
    meta = M.make_meta(M.load_config(p), "HERO2_x", 1_760_000_000 * SEC)
    assert meta.to_json()["started_utc_approx"] == "2025-10-09T08:53:20.000Z"
