"""讀樹莓派相機模組的會期目錄（格式 "aeroplan-pi-session/1"，F1）。

會期目錄內容（相機模組端寫，本模組只讀）::

    session.json     開始時寫：機號、唯一 Model、相機（讀出時間 readout_ms 可為 null）、槓桿臂…
    frames.jsonl     每個觸發邊緣一列，追加後立即 fsync
    clocks.jsonl     MONOTONIC / BOOTTIME / REALTIME 三時鐘取樣（≥每 10 s 一次，加開始與結束）
    summary.json     正常停止才寫；斷電後可能不存在 ⇒ 不可依賴
    images/          JPEG
    imu.csv          （選配）
    gnss/rover.ubx   （選配，第二階段 PPK）

時間基準：``edge_ns`` 是 gpiod 的 CLOCK_MONOTONIC，``sensor_ts_ns`` 是 libcamera 的
CLOCK_BOOTTIME。兩者只差系統休眠時間，用 clocks.jsonl 的 BOOTTIME − MONOTONIC 換算；
這個差必須在 1 ms 內保持不變，否則旗標 ``clock_offset_unstable``（表示會期中休眠過）。

容錯（落地直接拔電是常態，評估 V5 #9）：
* summary.json 缺 ⇒ 旗標 ``summary_missing``，不影響讀取。
* frames.jsonl 最後一列寫到一半 ⇒ 丟掉該列、旗標 ``frames_truncated_tail``。
  中間列壞掉不是斷電造成的 ⇒ 直接拒絕（不靜默跳過）。
"""

from __future__ import annotations

import hashlib
import json
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

_log = logging.getLogger(__name__)

SESSION_FORMAT = "aeroplan-pi-session/1"
FRAME_STATUSES = ("ok", "no_frame", "write_error", "dropped")
CLOCK_OFFSET_TOL_NS = 1_000_000          # BOOTTIME − MONOTONIC 允許的漂移：1 ms
_REQUIRED_META = ("session_id", "uav", "unit_model", "camera", "lever_arm_m")
_REQUIRED_CAMERA = ("sensor", "lens_mm", "pixel_um", "width", "height", "readout_ms", "shutter")


@dataclass
class PiFrame:
    seq: int
    edge_ns: int
    sensor_ts_ns: Optional[int]
    exposure_us: Optional[int]
    frame_duration_us: Optional[int]
    analogue_gain: Optional[float]
    file: Optional[str]
    bytes: Optional[int]
    sha256: Optional[str]
    encode_ms: Optional[float]
    temp_c: Optional[float]
    throttled: Optional[int]
    status: str


@dataclass
class PiSession:
    path: Path
    meta: dict
    frames: list                       # PiFrame，依 seq 排序
    clock_offset_ns: int               # BOOTTIME − MONOTONIC（中位數）
    clock_offset_spread_ns: int
    summary: Optional[dict]
    flags: list = field(default_factory=list)
    seq_gaps: list = field(default_factory=list)        # 缺號（邊緣計數器跳號）
    missing_files: list = field(default_factory=list)   # status ok 但檔案不在
    sha256_mismatch: list = field(default_factory=list) # 有驗才會填

    @property
    def uav(self) -> str:
        return str(self.meta["uav"])

    @property
    def session_id(self) -> str:
        return str(self.meta["session_id"])

    @property
    def unit_model(self) -> str:
        return str(self.meta["unit_model"])

    @property
    def camera(self) -> dict:
        return self.meta["camera"]

    @property
    def readout_ms(self) -> Optional[float]:
        v = self.meta["camera"].get("readout_ms")
        return None if v is None else float(v)

    @property
    def lever_arm_m(self) -> np.ndarray:
        return np.asarray(self.meta["lever_arm_m"], float)

    @property
    def gnss_ubx_path(self) -> Optional[Path]:
        p = self.path / "gnss" / "rover.ubx"
        return p if p.exists() else None

    @property
    def imu_path(self) -> Optional[Path]:
        p = self.path / "imu.csv"
        return p if p.exists() else None

    def edge_boottime_ns(self) -> np.ndarray:
        """每個邊緣換到 BOOTTIME（與 sensor_ts_ns 同一時鐘），依 frames 順序。"""
        return np.asarray([f.edge_ns for f in self.frames], np.int64) + np.int64(self.clock_offset_ns)


def _opt(d, key, typ):
    v = d.get(key)
    return None if v is None else typ(v)


def _parse_frame(d: dict, where: str) -> PiFrame:
    try:
        status = d["status"]
        if status not in FRAME_STATUSES:
            raise ValueError(f"status {status!r} 不在 {FRAME_STATUSES}")
        return PiFrame(
            seq=int(d["seq"]), edge_ns=int(d["edge_ns"]),
            sensor_ts_ns=_opt(d, "sensor_ts_ns", int), exposure_us=_opt(d, "exposure_us", int),
            frame_duration_us=_opt(d, "frame_duration_us", int), analogue_gain=_opt(d, "analogue_gain", float),
            file=_opt(d, "file", str), bytes=_opt(d, "bytes", int), sha256=_opt(d, "sha256", str),
            encode_ms=_opt(d, "encode_ms", float), temp_c=_opt(d, "temp_c", float),
            throttled=_opt(d, "throttled", int), status=status)
    except (KeyError, TypeError, ValueError) as e:
        raise ValueError(f"{where}: 欄位錯誤：{e}") from None


def _read_jsonl(path: Path, *, tolerate_tail: bool):
    """回傳 (物件列表, 尾端是否截斷)。中間列壞掉一律拒絕。"""
    # 斷電可能把尾端的多位元組字元切一半：用 replace 解碼，壞掉的那列交給下面的 JSON 判斷
    text = path.read_bytes().decode("utf-8", errors="replace")
    lines = text.split("\n")
    # 最後一個元素：檔案以換行結尾時是空字串
    out, truncated = [], False
    nonempty = [(i, ln) for i, ln in enumerate(lines) if ln.strip()]
    for k, (i, ln) in enumerate(nonempty):
        try:
            out.append(json.loads(ln))
        except json.JSONDecodeError:
            if tolerate_tail and k == len(nonempty) - 1:
                truncated = True
                continue
            raise ValueError(f"{path.name} 第 {i + 1} 列不是合法 JSON（不是尾端截斷，拒絕讀取）") from None
    return out, truncated


def read_pi_session(path, *, verify_sha256: bool = False, allow_missing_clocks: bool = False) -> PiSession:
    """讀一個會期目錄。``verify_sha256`` 為真時逐張比對影像雜湊。

    ``allow_missing_clocks``：clocks.jsonl 缺時預設拒絕；明確允許時改用偏移 0
    （BOOTTIME = MONOTONIC，只在確定沒休眠時成立）並加旗標 ``clocks_missing_assumed_zero``。
    """
    path = Path(path)
    meta_p = path / "session.json"
    if not meta_p.exists():
        raise FileNotFoundError(f"{path} 不是會期目錄：缺 session.json")
    meta = json.loads(meta_p.read_text(encoding="utf-8"))
    if meta.get("format") != SESSION_FORMAT:
        raise ValueError(f"session.json format={meta.get('format')!r}，只接受 {SESSION_FORMAT!r}")
    missing = [k for k in _REQUIRED_META if k not in meta]
    missing += [f"camera.{k}" for k in _REQUIRED_CAMERA if k not in (meta.get("camera") or {})]
    if missing:
        raise ValueError(f"session.json 缺欄位：{missing}")
    if len(meta["lever_arm_m"]) != 3:
        raise ValueError(f"lever_arm_m 必須是 3 個數，收到 {meta['lever_arm_m']!r}")

    flags = []
    frames_p = path / "frames.jsonl"
    raw, truncated = _read_jsonl(frames_p, tolerate_tail=True) if frames_p.exists() else ([], False)
    if not frames_p.exists():
        flags.append("frames_missing")
    if truncated:
        flags.append("frames_truncated_tail")
    frames = [_parse_frame(d, f"frames.jsonl 第 {k + 1} 筆") for k, d in enumerate(raw)]
    seqs = [f.seq for f in frames]
    if len(set(seqs)) != len(seqs):
        dup = sorted({s for s in seqs if seqs.count(s) > 1})
        raise ValueError(f"frames.jsonl 有重複的 seq：{dup[:10]}")
    if seqs != sorted(seqs):
        flags.append("frames_out_of_order")
        frames.sort(key=lambda f: f.seq)
        seqs = [f.seq for f in frames]
    gaps = []
    if seqs:
        present = set(seqs)
        gaps = [s for s in range(1, seqs[-1] + 1) if s not in present]
        if gaps:
            flags.append("seq_gap")

    clocks_p = path / "clocks.jsonl"
    clocks, ctrunc = _read_jsonl(clocks_p, tolerate_tail=True) if clocks_p.exists() else ([], False)
    if ctrunc:
        flags.append("clocks_truncated_tail")
    offsets = [int(c["boottime_ns"]) - int(c["monotonic_ns"]) for c in clocks]
    if offsets:
        off = int(np.median(offsets))
        spread = int(max(offsets) - min(offsets))
        if spread > CLOCK_OFFSET_TOL_NS:
            flags.append("clock_offset_unstable")
            _log.warning("會期 %s：BOOTTIME−MONOTONIC 變動 %.3f ms（>1 ms，會期中可能休眠過）",
                         meta["session_id"], spread / 1e6)
    elif allow_missing_clocks:
        off, spread = 0, 0
        flags.append("clocks_missing_assumed_zero")
        _log.warning("會期 %s：沒有 clocks.jsonl，依呼叫端要求假設 BOOTTIME = MONOTONIC", meta["session_id"])
    else:
        raise ValueError(f"{path} 缺 clocks.jsonl（或是空的）：無法把 MONOTONIC 邊緣換到 BOOTTIME")

    summary = None
    summ_p = path / "summary.json"
    if summ_p.exists():
        summary = json.loads(summ_p.read_text(encoding="utf-8"))
        n_ok = sum(1 for f in frames if f.status == "ok")
        if summary.get("edges") != len(frames) or summary.get("ok") != n_ok:
            flags.append("summary_mismatch")
    else:
        flags.append("summary_missing")

    sess = PiSession(path=path, meta=meta, frames=frames, clock_offset_ns=off,
                     clock_offset_spread_ns=spread, summary=summary, flags=flags, seq_gaps=gaps)
    for f in frames:
        if f.status != "ok":
            continue
        if not f.file or not (path / f.file).is_file():
            sess.missing_files.append(f.seq)
        elif verify_sha256:
            if not f.sha256 or hashlib.sha256((path / f.file).read_bytes()).hexdigest() != f.sha256.lower():
                sess.sha256_mismatch.append(f.seq)
    if sess.missing_files:
        flags.append("missing_files")
    if sess.sha256_mismatch:
        flags.append("sha256_mismatch")
    return sess
