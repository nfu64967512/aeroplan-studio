"""會期格式 aeroplan-pi-session/1（F1）—— 常數、session.json 欄位、fsync 寫檔、時鐘、離線驗證

會期目錄（每架次一個，絕不覆寫）：
    session.json     開始時寫（原子寫入＋fsync）
    frames.jsonl     每個觸發上升緣一列，寫完立刻 fsync
    clocks.jsonl     MONOTONIC/BOOTTIME/REALTIME 三時鐘取樣，開始、結束與每 ≤10 s 一筆
    manifest.sha256  （F1 之外的附加檔）`sha256sum -c` 格式，每存一個檔就追加一列並 fsync
    imu.csv          （選配）見 imu.py
    gnss/rover.ubx   （選配，第二階段）見 f9p_config.py
    summary.json     只在正常停止時寫；斷電（直接拔動力電池）後會缺 —— 讀取端不得依賴它
    images/          JPEG，檔名 = 邊緣序號 000001.jpg

本模組只用標準函式庫，Windows 上也能跑（測試用）。
"""
from __future__ import annotations

import datetime as _dt
import hashlib
import json
import math
import os
import shutil
import time
from dataclasses import dataclass
from pathlib import Path

FORMAT = "aeroplan-pi-session/1"

# frames.jsonl 每列的鍵（F1，順序即寫出順序；不多不少）
FRAME_KEYS = (
    "seq", "edge_ns", "sensor_ts_ns", "exposure_us", "frame_duration_us",
    "analogue_gain", "file", "bytes", "sha256", "encode_ms", "temp_c",
    "throttled", "status",
)
FRAME_STATUSES = ("ok", "no_frame", "write_error", "dropped")
SUMMARY_KEYS = ("edges", "ok", "no_frame", "write_error", "stopped_utc_approx")
CLOCK_KEYS = ("monotonic_ns", "boottime_ns", "realtime_ns")
MAX_CLOCK_PERIOD_S = 10.0          # F1：clocks.jsonl 至少每 10 s 一筆
SHUTTERS = ("rolling", "global")

SESSION_JSON = "session.json"
FRAMES_JSONL = "frames.jsonl"
CLOCKS_JSONL = "clocks.jsonl"
MANIFEST = "manifest.sha256"
SUMMARY_JSON = "summary.json"
IMAGES_DIR = "images"


def _is_int(v) -> bool:
    return isinstance(v, int) and not isinstance(v, bool)


def _is_num(v) -> bool:
    return (isinstance(v, (int, float)) and not isinstance(v, bool)
            and math.isfinite(float(v)))


def _vec3(name: str, v, *, allow_none: bool) -> list | None:
    if v is None:
        if allow_none:
            return None
        raise ValueError(f"{name} 必須實測填入 [x, y, z]（機身 FRD，公尺），不可留空")
    if not isinstance(v, (list, tuple)) or len(v) != 3 or not all(_is_num(x) for x in v):
        raise ValueError(f"{name} 必須是三個有限數字 [x, y, z]，收到 {v!r}")
    return [float(x) for x in v]


@dataclass(frozen=True)
class CameraInfo:
    """session.json 的 camera 區塊。readout_ms 為實測值；沒量過就是 None，不填推算值。"""
    sensor: str
    lens_mm: float
    pixel_um: float
    width: int
    height: int
    readout_ms: float | None
    shutter: str
    kernel: str
    dtoverlay: str

    def __post_init__(self):
        for k in ("sensor", "kernel", "dtoverlay"):
            if not isinstance(getattr(self, k), str) or not getattr(self, k).strip():
                raise ValueError(f"camera.{k} 必須是非空字串")
        for k in ("lens_mm", "pixel_um"):
            if not _is_num(getattr(self, k)) or getattr(self, k) <= 0:
                raise ValueError(f"camera.{k} 必須 > 0，收到 {getattr(self, k)!r}")
        for k in ("width", "height"):
            if not _is_int(getattr(self, k)) or getattr(self, k) <= 0:
                raise ValueError(f"camera.{k} 必須是正整數，收到 {getattr(self, k)!r}")
        if self.readout_ms is not None and (not _is_num(self.readout_ms) or self.readout_ms <= 0):
            raise ValueError(f"camera.readout_ms 必須是實測的正數或 None，收到 {self.readout_ms!r}")
        if self.shutter not in SHUTTERS:
            raise ValueError(f"camera.shutter 必須是 {SHUTTERS} 之一，收到 {self.shutter!r}")

    def to_json(self) -> dict:
        return {
            "sensor": self.sensor, "lens_mm": float(self.lens_mm),
            "pixel_um": float(self.pixel_um), "width": self.width, "height": self.height,
            "readout_ms": None if self.readout_ms is None else float(self.readout_ms),
            "shutter": self.shutter, "kernel": self.kernel, "dtoverlay": self.dtoverlay,
        }


@dataclass(frozen=True)
class SessionMeta:
    """session.json（F1）。lever_arm_m 是必填的實測值：相機中心 − 飛控 GNSS 天線，機身 FRD。"""
    session_id: str
    uav: str
    unit_model: str
    camera: CameraInfo
    lever_arm_m: tuple
    gnss_lever_arm_m: tuple | None
    software: str
    started_utc_approx: str

    def __post_init__(self):
        for k in ("session_id", "uav", "unit_model", "software", "started_utc_approx"):
            if not isinstance(getattr(self, k), str) or not getattr(self, k).strip():
                raise ValueError(f"{k} 必須是非空字串")
        if any(c in self.session_id for c in "/\\"):
            raise ValueError(f"session_id 不可含路徑分隔字元：{self.session_id!r}")
        if not isinstance(self.camera, CameraInfo):
            raise ValueError("camera 必須是 CameraInfo")
        object.__setattr__(self, "lever_arm_m",
                           tuple(_vec3("lever_arm_m", self.lever_arm_m, allow_none=False)))
        g = _vec3("gnss_lever_arm_m", self.gnss_lever_arm_m, allow_none=True)
        object.__setattr__(self, "gnss_lever_arm_m", None if g is None else tuple(g))

    def to_json(self) -> dict:
        return {
            "format": FORMAT,
            "session_id": self.session_id,
            "uav": self.uav,
            "unit_model": self.unit_model,
            "camera": self.camera.to_json(),
            "lever_arm_m": list(self.lever_arm_m),
            "gnss_lever_arm_m": None if self.gnss_lever_arm_m is None else list(self.gnss_lever_arm_m),
            "software": self.software,
            "started_utc_approx": self.started_utc_approx,
        }


def utc_iso(realtime_ns: int) -> str:
    """REALTIME ns → ISO8601 UTC（毫秒）。Pi 的系統時鐘在野外可能沒校時，所以欄位名都叫 *_approx。"""
    t = _dt.datetime.fromtimestamp(realtime_ns / 1e9, tz=_dt.timezone.utc)
    return t.isoformat(timespec="milliseconds").replace("+00:00", "Z")


def new_session_id(uav: str, realtime_ns: int) -> str:
    t = _dt.datetime.fromtimestamp(realtime_ns / 1e9, tz=_dt.timezone.utc)
    return f"{uav}_{t.strftime('%Y%m%dT%H%M%SZ')}"


# ── 時鐘 ─────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class ClockSample:
    monotonic_ns: int
    boottime_ns: int
    realtime_ns: int

    def to_json(self) -> dict:
        return {"monotonic_ns": self.monotonic_ns, "boottime_ns": self.boottime_ns,
                "realtime_ns": self.realtime_ns}


class SystemClock:
    """Linux 實機時鐘。gpiod 邊緣預設 CLOCK_MONOTONIC；libcamera SensorTimestamp 是 CLOCK_BOOTTIME。
    兩者只差系統休眠時間；clocks.jsonl 讓地面端能證明（或推翻）這個差是常數。"""

    def monotonic_ns(self) -> int:
        return time.clock_gettime_ns(time.CLOCK_MONOTONIC)

    def sample(self) -> ClockSample:
        return ClockSample(
            monotonic_ns=time.clock_gettime_ns(time.CLOCK_MONOTONIC),
            boottime_ns=time.clock_gettime_ns(time.CLOCK_BOOTTIME),
            realtime_ns=time.clock_gettime_ns(time.CLOCK_REALTIME),
        )


# ── 儲存（每次寫入都 fsync：飛行結束通常是直接拔動力電池）──────────────────

def _fsync_dir(path: Path) -> None:
    if os.name == "nt":                     # Windows 不能對目錄 fsync（只在測試時跑到）
        return
    fd = os.open(str(path), os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


class AppendFile:
    """只追加的檔案。append_line = 寫一列 + flush + fsync；write/fsync 分開給串流用（rover.ubx、imu.csv）。"""

    def __init__(self, path: Path):
        self.path = path
        self._f = open(path, "ab")

    def append_line(self, text: str) -> None:
        self._f.write(text.encode("utf-8") + b"\n")
        self.fsync()

    def write(self, data: bytes) -> None:
        self._f.write(data)

    def fsync(self) -> None:
        self._f.flush()
        os.fsync(self._f.fileno())

    def close(self) -> None:
        if not self._f.closed:
            self.fsync()
            self._f.close()


class LocalStorage:
    """會期目錄。create() 時目錄必須還不存在：絕不覆寫舊會期。"""

    def __init__(self, root):
        self.root = Path(root)

    def create(self) -> None:
        self.root.mkdir(parents=True, exist_ok=False)
        _fsync_dir(self.root.parent)

    def path(self, rel: str) -> Path:
        return self.root / rel

    def makedirs(self, rel: str) -> None:
        p = self.root / rel
        p.mkdir(parents=True, exist_ok=True)
        _fsync_dir(p.parent)

    def write_atomic(self, rel: str, data: bytes) -> None:
        """先寫 .tmp、fsync、再 rename：斷電時檔案不是完整就是不存在。"""
        final = self.root / rel
        tmp = final.with_name(final.name + ".tmp")
        with open(tmp, "wb") as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, final)
        _fsync_dir(final.parent)

    def open_append(self, rel: str) -> AppendFile:
        return AppendFile(self.root / rel)

    def free_bytes(self) -> int:
        """會期目錄所在檔案系統的剩餘空間；目錄還沒建立時往上找第一個存在的祖先。"""
        p = self.root
        while not p.exists() and p.parent != p:
            p = p.parent
        return shutil.disk_usage(p).free


def sha256_hex(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path, chunk: int = 1 << 20) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(chunk), b""):
            h.update(block)
    return h.hexdigest()


def manifest_line(sha_hex: str, rel: str) -> str:
    """GNU coreutils `sha256sum` 格式（兩個空白），卸載後可直接 `sha256sum -c manifest.sha256`。"""
    return f"{sha_hex}  {rel}"


# ── 離線讀取／驗證（Pi 上卸載前後與地面端都可用）────────────────────────────

def read_jsonl(path) -> tuple[list[dict], list[int]]:
    """容錯讀 JSONL：回傳 (可解析的列, 壞列的行號)。斷電時最後一列可能只寫了一半。"""
    records, bad = [], []
    p = Path(path)
    if not p.exists():
        return records, bad
    with open(p, "rb") as f:
        for i, raw in enumerate(f.read().split(b"\n"), start=1):
            if not raw.strip():
                continue
            try:
                obj = json.loads(raw.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                bad.append(i)
                continue
            if isinstance(obj, dict):
                records.append(obj)
            else:
                bad.append(i)
    return records, bad


def frame_schema_errors(rec: dict) -> list[str]:
    """frames.jsonl 一列對 F1 的型別檢查。"""
    errs = []
    if set(rec.keys()) != set(FRAME_KEYS):
        errs.append(f"鍵不符：{sorted(set(rec) ^ set(FRAME_KEYS))}")
        return errs
    ints = ("seq", "edge_ns")
    opt_ints = ("sensor_ts_ns", "exposure_us", "frame_duration_us", "bytes", "throttled")
    opt_nums = ("analogue_gain", "encode_ms", "temp_c")
    opt_strs = ("file", "sha256")
    for k in ints:
        if not _is_int(rec[k]):
            errs.append(f"{k} 應為 int")
    for k in opt_ints:
        if rec[k] is not None and not _is_int(rec[k]):
            errs.append(f"{k} 應為 int|null")
    for k in opt_nums:
        if rec[k] is not None and not _is_num(rec[k]):
            errs.append(f"{k} 應為 number|null")
    for k in opt_strs:
        if rec[k] is not None and not isinstance(rec[k], str):
            errs.append(f"{k} 應為 str|null")
    if rec["status"] not in FRAME_STATUSES:
        errs.append(f"status 不合法：{rec['status']!r}")
    elif rec["status"] == "ok" and any(rec[k] is None for k in ("file", "bytes", "sha256", "sensor_ts_ns")):
        errs.append("status=ok 但 file/bytes/sha256/sensor_ts_ns 有空值")
    return errs


def verify_session(session_dir) -> dict:
    """檢查一個會期目錄是否可讀、檔案是否完整。不需要 summary.json（斷電時本來就沒有）。

    回傳 dict；`ok` 為 False 代表有實質問題（缺檔、雜湊不符、序號跳號、中段壞列），
    最後一列寫一半（torn）只記錄、不算失敗。
    """
    d = Path(session_dir)
    rep: dict = {"session_dir": str(d), "errors": [], "torn_last_line": False}
    try:
        meta = json.loads((d / SESSION_JSON).read_text(encoding="utf-8"))
        if meta.get("format") != FORMAT:
            rep["errors"].append(f"session.json format 不是 {FORMAT}：{meta.get('format')!r}")
    except (OSError, json.JSONDecodeError) as e:
        rep["errors"].append(f"session.json 無法讀取：{e}")

    recs, bad = read_jsonl(d / FRAMES_JSONL)
    n_lines = 0
    if (d / FRAMES_JSONL).exists():
        n_lines = len([x for x in (d / FRAMES_JSONL).read_bytes().split(b"\n") if x.strip()])
    if bad:
        if bad == [max(bad)] and max(bad) >= n_lines:
            rep["torn_last_line"] = True
        else:
            rep["errors"].append(f"frames.jsonl 中段壞列：{bad}")
    counts = {s: 0 for s in FRAME_STATUSES}
    seqs = []
    referenced = set()
    for r in recs:
        errs = frame_schema_errors(r)
        if errs:
            rep["errors"].append(f"seq={r.get('seq')!r}: {'; '.join(errs)}")
            continue
        counts[r["status"]] += 1
        seqs.append(r["seq"])
        if r["status"] == "ok":
            referenced.add(r["file"])
            p = d / r["file"]
            if not p.exists():
                rep["errors"].append(f"seq={r['seq']}: 缺檔 {r['file']}")
                continue
            data = p.read_bytes()
            if len(data) != r["bytes"] or sha256_hex(data) != r["sha256"]:
                rep["errors"].append(f"seq={r['seq']}: {r['file']} 大小或 sha256 不符")
    rep["edges"] = len(seqs)
    rep["counts"] = counts
    if seqs != list(range(1, len(seqs) + 1)):
        rep["errors"].append("seq 不是從 1 起連續遞增")

    man_lines, man_bad = 0, 0
    if (d / MANIFEST).exists():
        for line in (d / MANIFEST).read_text(encoding="utf-8", errors="replace").splitlines():
            if not line.strip():
                continue
            man_lines += 1
            h, sep, rel = line.partition("  ")
            p = d / rel
            if not sep or not p.exists() or sha256_hex(p.read_bytes()) != h:
                man_bad += 1
                rep["errors"].append(f"manifest 不符：{line[:120]}")
    rep["manifest_lines"] = man_lines

    imgs = d / IMAGES_DIR
    on_disk = set()
    if imgs.exists():
        on_disk = {f"{IMAGES_DIR}/{p.name}" for p in imgs.iterdir() if p.suffix.lower() == ".jpg"}
        rep["tmp_leftovers"] = sorted(p.name for p in imgs.iterdir() if p.name.endswith(".tmp"))
    else:
        rep["tmp_leftovers"] = []
    rep["orphan_images"] = sorted(on_disk - referenced)

    clocks, _ = read_jsonl(d / CLOCKS_JSONL)
    mono = [c["monotonic_ns"] for c in clocks if _is_int(c.get("monotonic_ns"))]
    rep["clock_samples"] = len(mono)
    rep["max_clock_gap_s"] = (max((b - a) for a, b in zip(mono, mono[1:])) / 1e9
                              if len(mono) >= 2 else None)
    if len(mono) == 0:
        rep["errors"].append("clocks.jsonl 沒有任何取樣")

    summ = d / SUMMARY_JSON
    rep["has_summary"] = summ.exists()
    if summ.exists():
        try:
            s = json.loads(summ.read_text(encoding="utf-8"))
            exp = {"edges": len(seqs), "ok": counts["ok"], "no_frame": counts["no_frame"],
                   "write_error": counts["write_error"]}
            diff = {k: (s.get(k), v) for k, v in exp.items() if s.get(k) != v}
            if diff:
                rep["errors"].append(f"summary.json 與 frames.jsonl 計數不符：{diff}")
        except (OSError, json.JSONDecodeError) as e:
            rep["errors"].append(f"summary.json 無法讀取：{e}")
    rep["ok"] = not rep["errors"]
    return rep
