"""擷取服務：GPIO 觸發邊緣 → 挑「第一張 SensorTimestamp ≥ 邊緣時刻」的影格 → 只編碼存那一張

相機連續出幀（free-run）；飛控 Relay 相機後端（CAM1_TYPE=2）每拍一張就打一個 AUX GPIO 脈衝。
同一個上升緣同時是飛控的 CAM.TimeUS 與這裡的 edge_ns，地面端用它逐張配對兩個時鐘。

時鐘：
- edge_ns：libgpiod v2 邊緣事件的核心時間戳，LineSettings.event_clock 預設 Clock.MONOTONIC
  （libgpiod bindings/python/gpiod/line_settings.py：`event_clock: Clock = Clock.MONOTONIC`）。
  F1 規定 edge_ns 是 CLOCK_MONOTONIC，所以這裡只收 MONOTONIC 的邊緣來源，其他直接拒絕。
- SensorTimestamp：libcamera 的 CLOCK_BOOTTIME（picamera2 手冊 §6.4.1：「measured in nanoseconds
  from when the system booted」）。比較前用最近一次 clocks 取樣的 (boottime − monotonic) 換到 MONOTONIC；
  frames.jsonl 寫的 sensor_ts_ns 仍是原始 BOOTTIME 值。
- ⚠ SensorTimestamp 的語意（第一列開始曝光 vs 第一個像素讀出）libcamera 與 picamera2 文件互相矛盾，
  台架 B3 定案前 UNVERIFIED；這裡照規格只比 SensorTimestamp ≥ edge。

挑幀規則（全部在 handle_edge / handle_frame / tick 三個純狀態函式裡，測試直接呼叫）：
1. 邊緣依到達順序排隊（FIFO）；新影格給最舊、且 edge_ns ≤ 影格時刻的那個邊緣。一張影格只給一個邊緣。
2. 沒人要的影格保留「最新一張」當回看（lookback），下一張到了才釋放上一張：
   邊緣事件若比影格晚送到（執行緒排程延遲），仍能拿到時刻正確的那張。其餘影格立即釋放。
3. 邊緣等超過 frame_timeout_ms（以 edge_ns 起算）還沒有影格 → status=no_frame。
4. 寫入佇列滿（max_inflight_writes）→ 影格立即釋放、status=dropped，避免卡死相機緩衝。
5. 編碼或寫檔失敗 → status=write_error（例外寫進日誌）。

每個邊緣恰好一列 frames.jsonl（立刻 fsync）；影像先寫 .tmp 再 rename；每存一張就追加 manifest.sha256。
沒有「結束才寫」的清單：飛完通常是直接拔動力電池。summary.json 只在 stop() 時寫。

實機後端（Picamera2Camera / GpiodEdgeSource / GpiodReadyOutput / SystemHealth）只在 Pi 上用，
picamera2、gpiod 都在類別方法裡才 import；Windows 測試只用假後端。
"""
from __future__ import annotations

import json
import logging
import math
import os
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass

from . import session as S

log = logging.getLogger("pi_capture")

EDGE_CLOCK = "monotonic"            # F1：edge_ns 是 CLOCK_MONOTONIC
HEALTH_PERIOD_NS = 1_000_000_000    # 溫度／空間／就緒腳每秒評估一次


@dataclass
class _Edge:
    seq: int
    edge_ns: int
    deadline_ns: int


def _opt_int(v):
    return None if v is None else int(v)


def _opt_float(v):
    return None if v is None else float(v)


class CaptureService:
    """一個會期（一架次）的擷取狀態機。

    camera：start(callback) / stop() / buffer_count；callback(frame) 送出每一張完成的影格。
        frame 需有 sensor_ts_ns(BOOTTIME)、exposure_us、frame_duration_us、analogue_gain、
        encode(quality)->bytes、release()。
    edges：start(callback) / stop() / clock；callback(edge_ns, line_seqno) 送出每個上升緣。
    clock：monotonic_ns() / sample()->ClockSample。
    health（選配）：temp_c() / throttled()；ready（選配）：set(bool)，只在健康時拉高。
    """

    def __init__(self, storage, meta, camera, edges, clock, *, health=None, ready=None,
                 jpeg_quality: int = 95, frame_timeout_ms: float = 500.0,
                 clock_period_s: float = 5.0, max_inflight_writes: int = 2,
                 sync_writes: bool = False, max_temp_c: float = 80.0,
                 min_free_bytes: int = 0, camera_stall_s: float = 2.0,
                 startup_gate_ok: bool = True):
        if not isinstance(jpeg_quality, int) or isinstance(jpeg_quality, bool) or not 1 <= jpeg_quality <= 100:
            raise ValueError(f"jpeg_quality 必須是 1–100 的整數，收到 {jpeg_quality!r}")
        if not (isinstance(frame_timeout_ms, (int, float)) and frame_timeout_ms > 0):
            raise ValueError(f"frame_timeout_ms 必須 > 0，收到 {frame_timeout_ms!r}")
        if not (isinstance(clock_period_s, (int, float)) and 0 < clock_period_s <= S.MAX_CLOCK_PERIOD_S):
            raise ValueError(f"clock_period_s 必須在 (0, {S.MAX_CLOCK_PERIOD_S}] 秒（F1 規定 ≤10 s），"
                             f"收到 {clock_period_s!r}")
        if not isinstance(max_inflight_writes, int) or max_inflight_writes < 1:
            raise ValueError(f"max_inflight_writes 必須是 ≥1 的整數，收到 {max_inflight_writes!r}")
        edge_clock = getattr(edges, "clock", None)
        if edge_clock != EDGE_CLOCK:
            raise ValueError(f"邊緣來源的時鐘是 {edge_clock!r}；F1 的 edge_ns 規定為 CLOCK_MONOTONIC，"
                             "請用 gpiod 預設的 Clock.MONOTONIC")
        bc = getattr(camera, "buffer_count", None)
        need = max(3, max_inflight_writes + 2)    # 寫入中的 + 回看 1 張 + 相機手上至少 1 張
        if not isinstance(bc, int) or bc < need:
            raise ValueError(f"camera.buffer_count={bc!r} 不足：max_inflight_writes={max_inflight_writes} "
                             f"時至少要 {need}（否則相機會因緩衝用光而停住）")
        if ready is not None and health is None:
            raise ValueError("有就緒腳（ready）就必須給 health 後端：溫度未知時不能宣告健康")

        self.storage = storage
        self.meta = meta
        self.camera = camera
        self.edges = edges
        self.clock = clock
        self.health = health
        self.ready = ready
        self.jpeg_quality = jpeg_quality
        self.max_inflight_writes = max_inflight_writes
        self.sync_writes = sync_writes
        self.max_temp_c = float(max_temp_c)
        self.min_free_bytes = int(min_free_bytes)
        self.startup_gate_ok = bool(startup_gate_ok)
        self._timeout_ns = int(round(frame_timeout_ms * 1e6))
        self._clock_period_ns = int(round(clock_period_s * 1e9))
        self._stall_ns = int(round(camera_stall_s * 1e9))

        self._lock = threading.Lock()
        self._events: queue.Queue = queue.Queue()
        self._jobs: queue.Queue = queue.Queue()
        self._writer: threading.Thread | None = None
        self._pending: deque[_Edge] = deque()
        self._held = None                       # 回看用：最新一張沒人要的影格
        self._last_released_mono: int | None = None
        self._seq = 0
        self._last_line_seqno: int | None = None
        self._inflight = 0
        self._counts = {s: 0 for s in S.FRAME_STATUSES}
        self.frames_seen = 0
        self.late_edges = 0
        self.lost_edge_events = 0
        self._write_errors = 0
        self._record_errors = 0
        self._last_frame_rx_mono: int | None = None
        self._boot_minus_mono = 0
        self._last_clock_mono: int | None = None
        self._last_health_mono: int | None = None
        self._temp_c = None
        self._throttled = None
        self.healthy = False
        self.health_reasons: list[str] = ["尚未開始"]
        self._frames = self._clocks = self._manifest = None
        self._started = False
        self._closed = False

    # ── 生命週期 ────────────────────────────────────────────────────────────

    def start(self) -> None:
        self.storage.create()
        self.storage.makedirs(S.IMAGES_DIR)
        data = json.dumps(self.meta.to_json(), ensure_ascii=False, indent=2).encode("utf-8")
        self.storage.write_atomic(S.SESSION_JSON, data)
        self._manifest = self.storage.open_append(S.MANIFEST)
        self._manifest.append_line(S.manifest_line(S.sha256_hex(data), S.SESSION_JSON))
        self._frames = self.storage.open_append(S.FRAMES_JSONL)
        self._clocks = self.storage.open_append(S.CLOCKS_JSONL)
        self._sample_clocks()
        self._started = True
        self._refresh_health(self.clock.monotonic_ns(), force=True)
        if not self.sync_writes:
            self._writer = threading.Thread(target=self._writer_loop, name="pi_capture-writer",
                                            daemon=True)
            self._writer.start()
        self.camera.start(self._on_frame_event)
        self.edges.start(self._on_edge_event)
        log.info("會期 %s 開始：邊緣時鐘 %s、JPEG q=%d、逾時 %.0f ms",
                 self.meta.session_id, EDGE_CLOCK, self.jpeg_quality, self._timeout_ns / 1e6)

    def stop(self, extra_files=()) -> dict:
        """正常停止：處理完佇列中的事件、未解的邊緣記為 no_frame、寫最後一筆時鐘與 summary.json。

        extra_files：同會期其他紀錄器已關好的檔（例如 "imu.csv"、"gnss/rover.ubx"），在這裡登記進 manifest。
        """
        if self._closed:
            raise RuntimeError("會期已關閉")
        try:
            self.edges.stop()
        except Exception:                        # noqa: BLE001 —— 停機要盡量走完
            log.exception("停止邊緣來源失敗")
        self._drain_events()
        while self._pending:
            self._emit(self._record(self._pending.popleft(), None, "no_frame"))
        self._release_held()
        if self._writer is not None:
            self._jobs.put(None)
            self._writer.join()
            self._writer = None
        try:
            self.camera.stop()
        except Exception:                        # noqa: BLE001
            log.exception("停止相機失敗")
        self._discard_frame_events()
        last = self._sample_clocks()
        for rel in extra_files:
            p = self.storage.path(rel)
            if p.exists():
                self._manifest.append_line(S.manifest_line(S.sha256_file(p), rel))
            else:
                log.warning("extra_files 指定的 %s 不存在，未登記 manifest", rel)
        summary = {
            "edges": self._seq,
            "ok": self._counts["ok"],
            "no_frame": self._counts["no_frame"],
            "write_error": self._counts["write_error"],
            "stopped_utc_approx": S.utc_iso(last.realtime_ns),
        }
        data = json.dumps(summary, ensure_ascii=False, indent=2).encode("utf-8")
        self.storage.write_atomic(S.SUMMARY_JSON, data)
        self._manifest.append_line(S.manifest_line(S.sha256_hex(data), S.SUMMARY_JSON))
        if self.ready is not None:
            self.ready.set(False)
        self._close_files()
        log.info("會期 %s 結束：%s", self.meta.session_id, summary)
        return summary

    def abort(self) -> None:
        """模擬斷電／異常結束：不寫 summary、不補最後一筆時鐘，只放掉檔案控制代碼與影格。"""
        if self._closed:
            return
        self._release_held()
        self._close_files()

    def _discard_frame_events(self) -> None:
        """相機停掉之後還排在佇列裡的影格：只釋放，不再挑幀。"""
        while True:
            try:
                ev = self._events.get_nowait()
            except queue.Empty:
                return
            if ev[0] == "frame":
                try:
                    ev[1].release()
                except Exception:                # noqa: BLE001
                    log.debug("相機關閉後釋放影格失敗", exc_info=True)

    def _close_files(self) -> None:
        for f in (self._frames, self._clocks, self._manifest):
            if f is not None:
                try:
                    f.close()
                except OSError:
                    log.exception("關檔失敗")
        self._closed = True

    # ── 事件入口（後端執行緒呼叫，只排隊）──────────────────────────────────

    def _on_edge_event(self, edge_ns: int, line_seqno: int | None = None) -> None:
        self._events.put(("edge", edge_ns, line_seqno))

    def _on_frame_event(self, frame) -> None:
        self._events.put(("frame", frame))

    def run_once(self, timeout_s: float = 0.05) -> None:
        """主迴圈一步：讓假後端有機會送事件 → 處理事件 → tick。"""
        for b in (self.camera, self.edges):
            poll = getattr(b, "poll", None)
            if poll is not None:
                poll()
        try:
            ev = self._events.get(timeout=timeout_s) if timeout_s > 0 else self._events.get_nowait()
        except queue.Empty:
            pass
        else:
            self._dispatch(ev)
            self._drain_events()
        self.tick()

    def run(self, stop_event: threading.Event) -> None:
        while not stop_event.is_set():
            self.run_once(0.05)

    def _drain_events(self) -> None:
        while True:
            try:
                ev = self._events.get_nowait()
            except queue.Empty:
                return
            self._dispatch(ev)

    def _dispatch(self, ev) -> None:
        if ev[0] == "edge":
            self.handle_edge(ev[1], ev[2])
        else:
            self.handle_frame(ev[1])

    # ── 狀態機 ──────────────────────────────────────────────────────────────

    def _mono_of(self, frame) -> int:
        return int(frame.sensor_ts_ns) - self._boot_minus_mono

    def handle_edge(self, edge_ns: int, line_seqno: int | None = None) -> None:
        self._seq += 1
        if line_seqno is not None:
            if self._last_line_seqno is not None and line_seqno != self._last_line_seqno + 1:
                lost = line_seqno - self._last_line_seqno - 1
                self.lost_edge_events += max(lost, 0)
                log.error("核心邊緣事件序號跳號 %d → %d：漏了 %d 個邊緣（核心緩衝溢位？）"
                          "；frames.jsonl 的 seq 只計收到的邊緣，會比飛控 CAM 少",
                          self._last_line_seqno, line_seqno, lost)
            self._last_line_seqno = line_seqno
        e = _Edge(self._seq, int(edge_ns), int(edge_ns) + self._timeout_ns)
        if self._last_released_mono is not None and self._last_released_mono >= e.edge_ns:
            self.late_edges += 1
            log.warning("seq=%d 邊緣事件送達太晚：時刻在它之後的第一張影格已被釋放，改用較晚的影格",
                        e.seq)
        if not self._pending and self._held is not None and self._mono_of(self._held) >= e.edge_ns:
            frame, self._held = self._held, None
            self._resolve(e, frame)
            return
        self._pending.append(e)

    def handle_frame(self, frame) -> None:
        self.frames_seen += 1
        self._last_frame_rx_mono = self.clock.monotonic_ns()
        if self._pending and self._mono_of(frame) >= self._pending[0].edge_ns:
            e = self._pending.popleft()
            self._release_held()
            self._resolve(e, frame)
            return
        self._release_held()
        self._held = frame

    def tick(self) -> None:
        now = self.clock.monotonic_ns()
        while self._pending and now >= self._pending[0].deadline_ns:
            e = self._pending.popleft()
            log.warning("seq=%d 等不到影格（>%.0f ms）→ no_frame", e.seq, self._timeout_ns / 1e6)
            self._emit(self._record(e, None, "no_frame"))
        if self._last_clock_mono is None or now - self._last_clock_mono >= self._clock_period_ns:
            self._sample_clocks()
        self._refresh_health(now)

    def _release_held(self) -> None:
        if self._held is not None:
            f, self._held = self._held, None
            self._last_released_mono = self._mono_of(f)
            f.release()

    def _resolve(self, e: _Edge, frame) -> None:
        with self._lock:
            full = self._inflight >= self.max_inflight_writes
            if not full:
                self._inflight += 1
        if full:
            frame.release()
            log.error("seq=%d 寫入佇列已滿（%d 張在編碼/寫入）→ dropped", e.seq, self.max_inflight_writes)
            self._emit(self._record(e, frame, "dropped"))
            return
        self._submit(("write", e, frame))

    def _emit(self, rec: dict) -> None:
        self._submit(("record", rec))

    def _submit(self, job) -> None:
        if self.sync_writes:
            self._execute(job)
        else:
            self._jobs.put(job)

    def _writer_loop(self) -> None:
        while True:
            job = self._jobs.get()
            if job is None:
                return
            try:
                self._execute(job)
            except Exception:                    # noqa: BLE001 —— 寫入執行緒不能死
                log.exception("寫入工作失敗")

    def _execute(self, job) -> None:
        if job[0] == "record":
            self._append_record(job[1])
            return
        _, e, frame = job
        rel = f"{S.IMAGES_DIR}/{e.seq:06d}.jpg"
        status, enc_ms, nbytes, sha, file = "ok", None, None, None, None
        try:
            t0 = time.perf_counter()
            data = frame.encode(self.jpeg_quality)
            enc_ms = (time.perf_counter() - t0) * 1e3
            self.storage.write_atomic(rel, data)
            sha, nbytes = S.sha256_hex(data), len(data)
            self._manifest.append_line(S.manifest_line(sha, rel))
            file = rel
        except Exception:                        # noqa: BLE001 —— 任何失敗都要留一列紀錄
            log.exception("seq=%d 編碼或寫檔失敗 → write_error", e.seq)
            status, nbytes, sha, file = "write_error", None, None, None
            with self._lock:
                self._write_errors += 1
            try:
                tmp = self.storage.path(rel + ".tmp")
                if tmp.exists():
                    tmp.unlink()
            except OSError:
                pass
        finally:
            frame.release()
            with self._lock:
                self._inflight -= 1
        self._append_record(self._record(e, frame, status, file=file, nbytes=nbytes, sha=sha,
                                         enc_ms=enc_ms))

    def _record(self, e: _Edge, frame, status: str, *, file=None, nbytes=None, sha=None,
                enc_ms=None) -> dict:
        return {
            "seq": e.seq,
            "edge_ns": e.edge_ns,
            "sensor_ts_ns": None if frame is None else int(frame.sensor_ts_ns),
            "exposure_us": None if frame is None else _opt_int(frame.exposure_us),
            "frame_duration_us": None if frame is None else _opt_int(frame.frame_duration_us),
            "analogue_gain": None if frame is None else _opt_float(frame.analogue_gain),
            "file": file,
            "bytes": nbytes,
            "sha256": sha,
            "encode_ms": None if enc_ms is None else round(enc_ms, 3),
            "temp_c": _opt_float(self._temp_c),
            "throttled": _opt_int(self._throttled),
            "status": status,
        }

    def _append_record(self, rec: dict) -> None:
        try:
            self._frames.append_line(json.dumps(rec, ensure_ascii=False, separators=(",", ":")))
        except Exception:                        # noqa: BLE001
            log.critical("frames.jsonl 寫入失敗（seq=%s）", rec.get("seq"), exc_info=True)
            with self._lock:
                self._record_errors += 1
        with self._lock:
            self._counts[rec["status"]] += 1

    def _sample_clocks(self):
        s = self.clock.sample()
        self._clocks.append_line(json.dumps(s.to_json(), separators=(",", ":")))
        self._last_clock_mono = s.monotonic_ns
        self._boot_minus_mono = s.boottime_ns - s.monotonic_ns
        return s

    # ── 健康與就緒腳 ────────────────────────────────────────────────────────

    def _refresh_health(self, now: int, force: bool = False) -> None:
        if not force and self._last_health_mono is not None and now - self._last_health_mono < HEALTH_PERIOD_NS:
            return
        self._last_health_mono = now
        if self.health is not None:
            try:
                self._temp_c = self.health.temp_c()
            except Exception:                    # noqa: BLE001
                self._temp_c = None
            try:
                self._throttled = self.health.throttled()
            except Exception:                    # noqa: BLE001
                self._throttled = None
        ok, reasons = self.evaluate_health(now)
        if ok != self.healthy:
            (log.info if ok else log.warning)("健康狀態 → %s %s", "OK" if ok else "NG", reasons)
        self.healthy, self.health_reasons = ok, reasons
        if self.ready is not None:
            self.ready.set(ok)

    def evaluate_health(self, now: int) -> tuple[bool, list[str]]:
        reasons = []
        if not self.startup_gate_ok:
            reasons.append("開機自檢未通過")
        if self._last_frame_rx_mono is None:
            reasons.append("尚未收到任何影格")
        elif now - self._last_frame_rx_mono > self._stall_ns:
            reasons.append(f"相機已 {(now - self._last_frame_rx_mono) / 1e9:.1f} s 沒有出幀")
        if self._temp_c is None:
            reasons.append("溫度未知")
        elif self._temp_c >= self.max_temp_c:
            reasons.append(f"溫度 {self._temp_c:.1f} °C ≥ {self.max_temp_c:.1f} °C")
        try:
            free = self.storage.free_bytes()
        except OSError:
            free = -1
        if free < self.min_free_bytes:
            reasons.append(f"剩餘空間 {free} B < {self.min_free_bytes} B")
        if self._write_errors or self._record_errors:
            reasons.append(f"寫入錯誤 {self._write_errors + self._record_errors} 次（不會自動恢復）")
        return (not reasons), reasons

    def stats(self) -> dict:
        with self._lock:
            counts = dict(self._counts)
            inflight = self._inflight
        return {
            "edges": self._seq, "counts": counts, "frames_seen": self.frames_seen,
            "pending": len(self._pending), "inflight": inflight, "late_edges": self.late_edges,
            "lost_edge_events": self.lost_edge_events, "healthy": self.healthy,
            "health_reasons": list(self.health_reasons),
        }


# ── 起飛前自檢 ──────────────────────────────────────────────────────────────

def run_selfcheck(*, storage, meta, camera, edges, clock, health, planned_photos: int,
                  bytes_per_photo: float, free_space_factor: float = 1.5,
                  max_temp_c: float = 80.0, external_edge_timeout_s: float = 0.0,
                  frame_wait_s: float = 5.0, jpeg_quality: int = 95,
                  frame_timeout_ms: float = 500.0) -> dict:
    """相機能開、剩餘空間 ≥ factor × 計畫張數 × 實測每張大小、溫度低於門檻、一個測試邊緣能存出一張影像。

    external_edge_timeout_s > 0：等真的 GPIO 邊緣（地面站送 DO_DIGICAM_CONTROL param5=1），整條鏈都測到；
    = 0：自己注入一個邊緣（只測相機→挑幀→編碼→寫檔，不測接線），結果標 edge_source="synthetic"。
    自檢的影像寫在 storage 指定的獨立目錄，不混進飛行會期（否則張數守恆會多一張）。
    回傳 {"ok", "checks":[...], ...}；CLI 依 ok 給結束碼 0/1。
    """
    if not isinstance(planned_photos, int) or isinstance(planned_photos, bool) or planned_photos < 1:
        raise ValueError(f"planned_photos 必須是 ≥1 的整數，收到 {planned_photos!r}")
    if not (isinstance(bytes_per_photo, (int, float)) and math.isfinite(bytes_per_photo)
            and bytes_per_photo > 0):
        raise ValueError(f"bytes_per_photo 必須是台架實測的正數，收到 {bytes_per_photo!r}")
    if not (isinstance(free_space_factor, (int, float)) and free_space_factor >= 1.0):
        raise ValueError(f"free_space_factor 必須 ≥ 1.0，收到 {free_space_factor!r}")

    checks = []

    def add(name, ok, value, limit, detail=""):
        checks.append({"name": name, "ok": bool(ok), "value": value, "limit": limit,
                       "detail": detail})

    required = int(math.ceil(free_space_factor * planned_photos * bytes_per_photo))
    try:
        free = int(storage.free_bytes())
    except OSError as e:
        free = None
        add("free_space", False, None, required, f"讀不到剩餘空間：{e}")
    else:
        add("free_space", free >= required, free, required,
            f"{free_space_factor} × {planned_photos} 張 × {bytes_per_photo:.0f} B")

    try:
        temp = None if health is None else health.temp_c()
    except Exception as e:                       # noqa: BLE001
        temp = None
        log.warning("讀溫度失敗：%s", e)
    add("temperature", temp is not None and temp < max_temp_c, temp, max_temp_c,
        "溫度未知視為失敗" if temp is None else "")

    edge_source = "external" if external_edge_timeout_s > 0 else "synthetic"
    svc = CaptureService(storage, meta, camera, edges, clock, health=health,
                         jpeg_quality=jpeg_quality, frame_timeout_ms=frame_timeout_ms,
                         sync_writes=True, max_temp_c=max_temp_c)
    try:
        svc.start()
    except Exception as e:                       # noqa: BLE001
        log.exception("自檢：相機或 GPIO 無法啟動")
        svc.abort()
        add("camera_open", False, 0, 1, f"啟動失敗：{e}")
        add("test_edge", False, None, "ok", "相機未啟動")
        return _selfcheck_result(checks, edge_source, required, free)

    deadline = time.monotonic() + frame_wait_s
    while svc.frames_seen == 0 and time.monotonic() < deadline:
        svc.run_once(0.05)
    add("camera_open", svc.frames_seen >= 1, svc.frames_seen, 1,
        "" if svc.frames_seen else f"{frame_wait_s} s 內沒有任何影格")

    if svc.frames_seen:
        if external_edge_timeout_s > 0:
            deadline = time.monotonic() + external_edge_timeout_s
            while svc.stats()["edges"] == 0 and time.monotonic() < deadline:
                svc.run_once(0.05)
        else:
            svc.handle_edge(clock.monotonic_ns())
        deadline = time.monotonic() + frame_wait_s
        while (svc.stats()["edges"] and sum(svc.stats()["counts"].values()) == 0
               and time.monotonic() < deadline):
            svc.run_once(0.05)
    try:
        svc.stop()
    except Exception:                            # noqa: BLE001
        log.exception("自檢：停止會期失敗")
    recs, _ = S.read_jsonl(storage.path(S.FRAMES_JSONL))
    if not recs:
        add("test_edge", False, None, "ok",
            "等不到外部邊緣" if edge_source == "external" else "沒有任何邊緣紀錄")
    else:
        r = recs[0]
        good = r["status"] == "ok"
        if good:
            p = storage.path(r["file"])
            good = p.exists() and S.sha256_hex(p.read_bytes()) == r["sha256"]
        add("test_edge", good, r["status"], "ok",
            f"seq={r['seq']} file={r['file']} bytes={r['bytes']}")
    return _selfcheck_result(checks, edge_source, required, free)


def _selfcheck_result(checks, edge_source, required, free) -> dict:
    return {"ok": all(c["ok"] for c in checks), "checks": checks, "edge_source": edge_source,
            "required_bytes": required, "free_bytes": free}


# ── JPEG 編碼（Pillow；picamera2 手冊 §6.4：CompletedRequest.make_image 回傳 PIL 影像）──────

def encode_jpeg(image, quality: int, *, make: str, model: str, lens_mm: float) -> bytes:
    """PIL 影像 → JPEG bytes，EXIF 寫 Make、每台唯一的 Model（ODM/OpenSfM 以此分開各機內參）、FocalLength。
    拍攝時間不寫進 EXIF：權威時間由地面端擬合後寫（OpenSfM 的 SubSecTime 讀法有坑，見評估 §8）。"""
    import io
    from PIL import Image, TiffImagePlugin

    exif = Image.Exif()
    exif[0x010F] = make                      # Make
    exif[0x0110] = model                     # Model
    exif.get_ifd(0x8769)[0x920A] = TiffImagePlugin.IFDRational(float(lens_mm))  # Exif IFD FocalLength
    buf = io.BytesIO()
    image.save(buf, format="JPEG", quality=int(quality), exif=exif.tobytes())
    return buf.getvalue()


# ── 實機後端（只在 Pi 上用；Windows 測試不會碰到）─────────────────────────────

class _PiFrame:
    __slots__ = ("_req", "_cam", "sensor_ts_ns", "exposure_us", "frame_duration_us",
                 "analogue_gain", "_released", "_lock")

    def __init__(self, req, md: dict, cam: "Picamera2Camera"):
        self._req = req
        self._cam = cam
        self.sensor_ts_ns = int(md["SensorTimestamp"])
        self.exposure_us = md.get("ExposureTime")
        self.frame_duration_us = md.get("FrameDuration")
        self.analogue_gain = md.get("AnalogueGain")
        self._released = False
        self._lock = threading.Lock()

    def encode(self, quality: int) -> bytes:
        img = self._req.make_image("main")
        return encode_jpeg(img, quality, make=self._cam.make, model=self._cam.model,
                           lens_mm=self._cam.lens_mm)

    def release(self) -> None:
        with self._lock:
            if self._released:
                return
            self._released = True
        self._req.release()


class Picamera2Camera:
    """picamera2 全解析度 still 設定、自由連續出幀。⚠ 硬體未驗證（台架 B1–B3）。

    依據 picamera2 手冊（RP-008156-DS，scratch hdmap_feas/picamera2.txt）：
    - create_still_configuration(buffer_count=…, queue=False, controls=…)：still 預設 buffer_count=1、
      「no frames are ever queued」（§4.2.1「More on the buffer_count」）；queue=False 保證回傳的影格不早於呼叫
      （§4.2.1「More on the queue parameter」）。
    - capture_request() / request.get_metadata() / request.make_image("main") / request.release()
      （§6.4「Capturing requests」：不 release 相機會用光緩衝而停住）。
    - 手動曝光與鎖白平衡：AeEnable=False、ExposureTime(µs)、AnalogueGain、AwbEnable=False、ColourGains
      （§5.1.1 設定隨 configuration 一起給；附錄 C：ColourGains「Setting these numbers disables AWB」）。
    - 鎖焦（CM3）：AfMode=controls.AfModeEnum.Manual、LensPosition（屈光度，0=無限遠）（§5.3.1、§5.3.3）。HQ 是機械鎖焦。
    - stream_configuration("main")["size"] 讀實際解析度，與 session.json 的 width/height 不符就拒絕啟動。
    UNVERIFIED：capture_request 在 stop() 之後是否一定返回（讀取執行緒設成 daemon）；
    request.release 跨執行緒呼叫的安全性（手冊 §6.4.2 說可在別的執行緒處理 request）。
    """

    def __init__(self, *, exposure_us: int, analogue_gain: float, colour_gains, model: str,
                 lens_mm: float, expected_size, buffer_count: int = 4, camera_num: int = 0,
                 lens_position: float | None = None, make: str = "RaspberryPi"):
        if not isinstance(exposure_us, int) or exposure_us <= 0:
            raise ValueError(f"exposure_us 必須是正整數（µs），收到 {exposure_us!r}")
        if not (isinstance(analogue_gain, (int, float)) and analogue_gain >= 1.0):
            raise ValueError(f"analogue_gain 必須 ≥ 1.0，收到 {analogue_gain!r}")
        if (not isinstance(colour_gains, (list, tuple)) or len(colour_gains) != 2
                or not all(isinstance(g, (int, float)) and g > 0 for g in colour_gains)):
            raise ValueError(f"colour_gains 必須是 [紅, 藍] 兩個正數（AWB 鎖死時一定要給），收到 {colour_gains!r}")
        if not isinstance(buffer_count, int) or buffer_count < 3:
            raise ValueError(f"buffer_count 必須 ≥ 3（still 預設 1 不會排隊），收到 {buffer_count!r}")
        self.exposure_us = exposure_us
        self.analogue_gain = float(analogue_gain)
        self.colour_gains = (float(colour_gains[0]), float(colour_gains[1]))
        self.model = model
        self.make = make
        self.lens_mm = float(lens_mm)
        self.expected_size = tuple(expected_size)
        self.buffer_count = buffer_count
        self.camera_num = camera_num
        self.lens_position = lens_position
        self._cam = None
        self._thread = None
        self._running = False

    def start(self, callback) -> None:
        from picamera2 import Picamera2           # 只在 Pi OS 上有

        cam = Picamera2(self.camera_num)
        controls = {"AeEnable": False, "AwbEnable": False, "ExposureTime": self.exposure_us,
                    "AnalogueGain": self.analogue_gain, "ColourGains": self.colour_gains}
        if self.lens_position is not None:
            if "AfMode" not in cam.camera_controls:
                cam.close()
                raise ValueError("設定了 lens_position，但這顆相機沒有對焦控制（HQ 請用機械鎖焦）")
            from libcamera import controls as lc
            controls["AfMode"] = lc.AfModeEnum.Manual
            controls["LensPosition"] = float(self.lens_position)
        cfg = cam.create_still_configuration(buffer_count=self.buffer_count, queue=False,
                                             controls=controls)
        cam.configure(cfg)
        size = tuple(cam.stream_configuration("main")["size"])
        if size != self.expected_size:
            cam.close()
            raise RuntimeError(f"相機實際解析度 {size} ≠ session.json 的 {self.expected_size}")
        cam.start()
        self._cam = cam
        self._running = True
        self._thread = threading.Thread(target=self._loop, args=(callback,),
                                        name="pi_capture-camera", daemon=True)
        self._thread.start()

    def _loop(self, callback) -> None:
        while self._running:
            req = self._cam.capture_request()
            if not self._running:
                req.release()
                return
            md = req.get_metadata()
            if "SensorTimestamp" not in md:
                log.error("影格沒有 SensorTimestamp，丟棄")
                req.release()
                continue
            callback(_PiFrame(req, md, self))

    def stop(self) -> None:
        self._running = False
        if self._cam is not None:
            self._cam.stop()
            self._cam.close()
            self._cam = None


def find_gpiochip(spec: str) -> str:
    """spec 是 /dev 路徑就直接用；否則當成晶片標籤（Pi 5 排針在 RP1 上，標籤 "pinctrl-rp1"，UNVERIFIED）
    去找。Pi 5 的排針晶片編號隨核心版本不同（舊 gpiochip4、新 gpiochip0），所以建議用標籤。"""
    if spec.startswith("/dev/"):
        return spec
    import glob
    import gpiod

    for path in sorted(glob.glob("/dev/gpiochip*")):
        if not gpiod.is_gpiochip_device(path):
            continue
        with gpiod.Chip(path) as chip:
            if chip.get_info().label == spec:
                return path
    raise FileNotFoundError(f"找不到標籤為 {spec!r} 的 gpiochip（用 gpioinfo 查）")


class GpiodEdgeSource:
    """libgpiod v2 Python 綁定的上升緣事件（核心時間戳）。⚠ 硬體未驗證（台架 B4）。

    API 依 libgpiod bindings/python/gpiod（git 主線，2.x）：
    request_lines(path, config={offset: LineSettings(...)}, consumer=…) → LineRequest；
    LineSettings(edge_detection=Edge.RISING, bias=…, event_clock=Clock.MONOTONIC)；
    LineRequest.wait_edge_events(timeout) -> bool、read_edge_events() -> list[EdgeEvent]；
    EdgeEvent.event_type (EdgeEvent.Type.RISING_EDGE)、timestamp_ns、line_seqno。
    Pi OS Bookworm 的 apt 套件 python3-libgpiod 是 v1 API（不相容）——要用 PyPI 的 gpiod ≥2（自帶 libgpiod）。
    """
    clock = EDGE_CLOCK

    def __init__(self, chip: str, line: int, *, bias: str = "pull_down",
                 consumer: str = "pi_capture"):
        if bias not in ("pull_down", "pull_up", "disabled", "as_is"):
            raise ValueError(f"bias 不支援：{bias!r}")
        self.chip = chip
        self.line = int(line)
        self.bias = bias
        self.consumer = consumer
        self._req = None
        self._thread = None
        self._running = False

    def start(self, callback) -> None:
        import gpiod
        from gpiod.line import Bias, Clock, Edge

        bias = {"pull_down": Bias.PULL_DOWN, "pull_up": Bias.PULL_UP,
                "disabled": Bias.DISABLED, "as_is": Bias.AS_IS}[self.bias]
        settings = gpiod.LineSettings(edge_detection=Edge.RISING, bias=bias,
                                      event_clock=Clock.MONOTONIC)
        self._req = gpiod.request_lines(find_gpiochip(self.chip), consumer=self.consumer,
                                        config={self.line: settings})
        self._running = True
        self._thread = threading.Thread(target=self._loop, args=(callback,),
                                        name="pi_capture-edges", daemon=True)
        self._thread.start()

    def _loop(self, callback) -> None:
        import gpiod

        while self._running:
            if not self._req.wait_edge_events(0.2):
                continue
            for ev in self._req.read_edge_events():
                if ev.event_type == gpiod.EdgeEvent.Type.RISING_EDGE:
                    callback(ev.timestamp_ns, ev.line_seqno)

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._req is not None:
            self._req.release()
            self._req = None


class GpiodReadyOutput:
    """「就緒」輸出腳：健康時拉高，其餘時間低（給日後飛控 Lua aux-auth 解鎖檢查用）。⚠ 硬體未驗證。
    ⚠ 行程被 kill -9 時，核心釋放 line 後腳位是否回到低電位 UNVERIFIED：飛控端應另接下拉，
    並考慮改成要求心跳（翻轉）而不是準位。"""

    def __init__(self, chip: str, line: int, consumer: str = "pi_capture-ready"):
        self.chip = chip
        self.line = int(line)
        self.consumer = consumer
        self._req = None
        self._state = None

    def open(self) -> None:
        import gpiod
        from gpiod.line import Direction, Value

        settings = gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE)
        self._req = gpiod.request_lines(find_gpiochip(self.chip), consumer=self.consumer,
                                        config={self.line: settings})
        self._state = False

    def set(self, healthy: bool) -> None:
        from gpiod.line import Value

        if self._req is None or self._state == bool(healthy):
            return
        self._req.set_value(self.line, Value.ACTIVE if healthy else Value.INACTIVE)
        self._state = bool(healthy)

    def close(self) -> None:
        if self._req is not None:
            self.set(False)
            self._req.release()
            self._req = None


class SystemHealth:
    """Pi 5 溫度與降頻旗標。溫度讀 sysfs thermal_zone0（毫度）；throttled 用 `vcgencmd get_throttled`
    （外部指令，結果快取由 CaptureService 每秒呼叫一次）。⚠ Pi 5 上兩者的路徑／輸出格式 UNVERIFIED。"""

    THERMAL = "/sys/class/thermal/thermal_zone0/temp"

    def temp_c(self) -> float | None:
        try:
            with open(self.THERMAL, encoding="ascii") as f:
                return int(f.read().strip()) / 1000.0
        except (OSError, ValueError):
            return None

    def throttled(self) -> int | None:
        import subprocess

        try:
            out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True,
                                 timeout=1.0, check=True).stdout
            return int(out.strip().split("=", 1)[1], 16)      # "throttled=0x50000"
        except (OSError, subprocess.SubprocessError, ValueError, IndexError):
            return None
