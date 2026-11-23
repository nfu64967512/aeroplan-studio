"""mission/param_backup.py — 無頭（不依賴 Qt）的飛控參數備份與還原。

為什麼需要這一支
================
規劃器一次會往飛控寫 32 個參數，其中含 ``FENCE_ACTION`` / ``FS_LONG_ACTN`` /
``BATT_FS_LOW_ACT`` 這些**失效保護行為**。SITL 每趟啟動都刪 ``eeprom.bin``，
真板子**不會** —— 寫進去就蓋掉人家調好的設定，而且回不去。

而 AeroPlan 既有的 GUI 匯出／匯入是**結構性死的**（2026-09-14 查證）::

    ParamService._on_param_value      ← 唯一寫入紀錄的地方
      ↑ 只被一個 lambda 呼叫，接在 link.param_value 訊號上
          ↑ SITLLink 沒有這個訊號（只有 servo_param）
            → param_service.py:83 的 hasattr 檢查失敗，降級成一行 log

``SITLLink._handle_message`` 收到 ``PARAM_VALUE`` 只發 ``servo_param``（僅 SERVOn_*）
或一則 severity 6 的 ``status_text``，沒有任何一條路進 ``ParamService``。
於是 ``ParametersBrowser._params`` 恆為空，``_on_export`` 第一行
``if not self._params: return`` 會**靜默返回**，``_on_import`` 的 ``diff_n`` 恆為 0。
按鈕在、程式碼在，但管線從來沒接上 —— 這比沒有功能更糟，因為它看起來有。

本模組不修那條 Qt 訊號鏈，直接對 pymavlink 連線工作，因此
``hv_dccpp.mavlink_link.MavlinkLink.conn`` 與 ``SITLLink._mav`` 都能用，
腳本與 CLI 也能用。

三個不可退讓的性質
==================
1. **完整**：丟包造成的缺漏會逐 index 補抓；補不齊就 :class:`IncompleteDump`
   拋出，**絕不寫出不完整的備份**。半份備份會在還原時把沒抓到的參數留在
   規劃器寫進去的值，而且沒有人會發現。
2. **精確**：參數在 MAVLink 上是 float32。既有的
   ``hv_dccpp.mavlink_link._fmt`` 用 ``%.6f``，實測六個測資壞四個，
   ``1.23e-07`` 與 ``4e-07`` 會變成**整數 0**（見 tests）。
   :func:`fmt_param` 找出能精確讀回 float32 的最短字串。
3. **認得出是哪一台**：檔案以 **USB 序號 + sysid** 命名，不是 COM 編號。
   六顆同型板在 Windows 全叫 ``COMxx`` 而編號會在重插後換；
   用編號認飛機，接錯沒有任何徵狀（見 memory: cube-orange-plus-ramtron）。
"""

from __future__ import annotations

import hashlib
import math
import os
import re
import struct
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping, Sequence

__all__ = [
    "BoardId",
    "Snapshot",
    "IncompleteDump",
    "RestoreReport",
    "VOLATILE_PARAMS",
    "fmt_param",
    "as_f32",
    "render_param_file",
    "parse_param_file",
    "fetch_all",
    "snapshot_dir",
    "save_snapshot",
    "load_snapshot",
    "latest_snapshot",
    "backup_before_write",
    "write_params_with_backup",
    "diff_params",
    "restore",
    "board_id_for_port",
]


# ---------------------------------------------------------------------------
# float32 精確往返
# ---------------------------------------------------------------------------
def as_f32(v: float) -> float:
    """把值壓成 float32 實際能表示的那個 double。

    MAVLink 的 ``param_value`` 是 float32；比較「還原後是否相同」必須在
    float32 的格上比，否則會拿 double 的舊值跟 float32 的讀回值比而永遠不相等。

    ⚠ 超出 float32 範圍時**飽和成 ±inf**（C 的 double→float 轉型行為），
    不是拋例外。:func:`fmt_param` 在上限值附近會試到 ``3.403e+38`` 這種
    溢位的候選字串，那應該被判成「不等於」而繼續找，不是把整支函式打掉。
    """
    d = float(v)
    try:
        return struct.unpack("<f", struct.pack("<f", d))[0]
    except OverflowError:
        return math.inf if d > 0 else -math.inf


_INT_EXACT_LIMIT = 1 << 24      # float32 能精確表示每個整數的上限


# ---------------------------------------------------------------------------
# 會自己變的參數
# ---------------------------------------------------------------------------
#: 飛控**在沒有人下 PARAM_SET 的情況下自己會改**的參數。
#:
#: ⚠⚠ 這不是理論。2026-09-14 對 ArduPlane SITL（quadplane，1620 個參數）
#: 取樣 8 次、跨約 367 秒模擬時間量到的 —— 見
#: ``scratchpad/param_volatility_probe.py``：
#:
#: ===================  =========================================================
#: 參數                 觀察到的行為
#: ===================  =========================================================
#: ``ARSPD_OFFSET``     開機空速歸零校正（0 → 2014.45），之後穩定
#: ``BARO1_GND_PRESS``  地面氣壓基準，開機後收斂
#: ``BARO2_GND_PRESS``  同上
#: ``COMPASS_DEC``      由 GPS 位置自動設定磁偏角，之後穩定
#: ``MIS_TOTAL``        **任務航點數** —— 每次上傳任務就變
#: ``STAT_RUNTIME``     **單調遞增，永遠不會穩定**
#: ===================  =========================================================
#:
#: 兩個直接後果：
#:
#: 1. ``STAT_RUNTIME`` 保證**任兩份快照都不相同** ⇒ 用全表雜湊去重在真板子上
#:    永遠不會命中，備份目錄會無限長。:meth:`Snapshot.content_hash` 因此
#:    **排除**本集合。
#: 2. ``MIS_TOTAL`` 是任務航點數。把舊值寫回去會動到飛控裡存的任務
#:    ⇒ :func:`restore` 預設**不寫**本集合，改列進 ``RestoreReport.volatile``。
#:
#: ⚠ 這是 **SITL 量到的下限，不是完整清單**。真板子上還會有 SITL 跑不出來的
#: （``STAT_BOOTCNT`` / ``STAT_FLTTIME`` 這些計數器、學習式羅盤偏置、
#: ``INS_*_CALTEMP`` 溫度校正）。下面兩個標了「同族推論」的是**沒有實測**的，
#: 納入是因為它們同屬 ``STAT_`` 統計計數器而漏寫它們沒有風險。
VOLATILE_PARAMS: frozenset[str] = frozenset({
    # —— 實測（ArduPlane SITL 4.8.0-dev，2026-09-14）——
    "ARSPD_OFFSET",
    "BARO1_GND_PRESS",
    "BARO2_GND_PRESS",
    "COMPASS_DEC",
    "MIS_TOTAL",
    "STAT_RUNTIME",
    # —— 同族推論，未實測 ——
    "STAT_BOOTCNT",
    "STAT_FLTTIME",
})


def fmt_param(v: float) -> str:
    """輸出能**精確讀回同一個 float32** 的最短十進位字串。

    ⚠ 不可改用 ``%.6f``：那會讓 ``3.0517578125e-05`` 變成 ``0.000031``
    （誤差 1.6%），讓 ``4e-07`` 變成 ``0``。``COMPASS_OFS_*`` /
    ``INS_ACCOFFS_*`` / ``AHRS_TRIM_*`` 都在這個量級 —— 還原回去等於把校正歸零。
    """
    f = as_f32(v)
    if not math.isfinite(f):
        raise ValueError(f"參數值不是有限數：{v!r}")
    if f == int(f) and abs(f) < _INT_EXACT_LIMIT:
        return str(int(f))
    for precision in range(1, 10):
        s = f"{f:.{precision}g}"
        if as_f32(float(s)) == f:
            return s
    return repr(f)          # 理論上到不了：float32 九位有效數字必然足夠


# ---------------------------------------------------------------------------
# 身分
# ---------------------------------------------------------------------------
_SAFE = re.compile(r"[^A-Za-z0-9._-]+")


@dataclass(frozen=True)
class BoardId:
    """一台飛控的穩定識別。

    ``serial`` 是 USB 裝置序號（Windows 上重插不變），不是 COM 編號。
    網路端點（``udp:`` / ``tcp:``）沒有序號，用端點字串本身。
    """

    serial: str
    sysid: int

    def key(self) -> str:
        """檔案系統安全的目錄名。"""
        s = _SAFE.sub("_", (self.serial or "").strip()) or "unknown"
        return f"{s}_sys{int(self.sysid)}"


def board_id_for_port(port: str, sysid: int) -> BoardId:
    """由連線字串推出 :class:`BoardId`。

    序列埠會去 pyserial 查 USB 序號；查不到時**明確標成 ``noserial-<埠>``**，
    不靜默退回埠名 —— 使用者必須看得出這份備份的識別是弱的。
    """
    text = (port or "").strip()
    if ":" in text and not re.match(r"^[A-Za-z]:", text):
        return BoardId(serial=text, sysid=int(sysid))
    try:
        from serial.tools import list_ports
        for p in list_ports.comports():
            if p.device == text and (p.serial_number or "").strip():
                return BoardId(serial=str(p.serial_number).strip(), sysid=int(sysid))
    except Exception:                                        # noqa: BLE001
        pass
    return BoardId(serial=f"noserial-{text}", sysid=int(sysid))


# ---------------------------------------------------------------------------
# 快照
# ---------------------------------------------------------------------------
class IncompleteDump(RuntimeError):
    """抓不齊全部參數。**不可**降級成「就存已經抓到的那些」。"""

    def __init__(self, expected: int, got: int, missing: Sequence[int]) -> None:
        self.expected, self.got, self.missing = int(expected), int(got), list(missing)
        head = ", ".join(str(i) for i in self.missing[:12])
        tail = " …" if len(self.missing) > 12 else ""
        super().__init__(
            f"參數抓取不完整：飛控宣告 {expected} 個，只收到 {got} 個，"
            f"缺 index [{head}{tail}]。不寫出半份備份。"
        )


@dataclass
class Snapshot:
    """一台飛控在某個時刻的全參數。"""

    board: BoardId
    params: dict[str, float] = field(default_factory=dict)
    types: dict[str, int] = field(default_factory=dict)
    taken_utc: str = ""
    note: str = ""

    def content_hash(self, exclude: Iterable[str] | None = None) -> str:
        """只看參數內容，**不含時間戳、不含會自己變的參數** —— 供去重用。

        ⚠ 排除 :data:`VOLATILE_PARAMS` 不是偷懶。``STAT_RUNTIME`` 單調遞增，
        任兩份快照都會不同；全表雜湊在真板子上**永遠不會命中**，去重等於沒有。
        存進檔案的內容仍然是完整的 —— 排除只發生在「這兩份算不算同一份設定」
        這個判斷上。
        """
        skip = VOLATILE_PARAMS if exclude is None else {n.upper() for n in exclude}
        body = "".join(f"{k},{fmt_param(v)}\n"
                       for k, v in sorted(self.params.items()) if k not in skip)
        return hashlib.sha256(body.encode("utf-8")).hexdigest()[:16]


def render_param_file(snap: Snapshot) -> str:
    """輸出 ArduPilot 慣用的 ``NAME,VALUE`` 文字。

    決定性：名稱排序、float32 精確格式。同樣的參數集合必然產生**位元相同**
    的內文（見 memory: export-must-be-deterministic）。時間戳在註解裡，
    而 :meth:`Snapshot.content_hash` 不看註解。
    """
    head = [
        "# AeroPlan 參數備份",
        f"# board_serial: {snap.board.serial}",
        f"# sysid: {int(snap.board.sysid)}",
        f"# taken_utc: {snap.taken_utc}",
        f"# count: {len(snap.params)}",
    ]
    if snap.note:
        head.append(f"# note: {snap.note}")
    body = [f"{name},{fmt_param(snap.params[name])}" for name in sorted(snap.params)]
    return "\n".join(head + body) + "\n"


_META = re.compile(r"^#\s*([A-Za-z_]+)\s*:\s*(.*)$")


def parse_param_file(text: str) -> tuple[dict[str, float], dict[str, str]]:
    """解析參數檔，回 ``(params, meta)``。

    容忍 Mission Planner / ArduPilot / 我們自己這幾種分隔：逗號、tab、空白、
    等號。註解列以 ``#`` 開頭；``# key: value`` 形式的註解會進 ``meta``。
    """
    params: dict[str, float] = {}
    meta: dict[str, str] = {}
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.startswith("#"):
            m = _META.match(line)
            if m:
                meta[m.group(1)] = m.group(2).strip()
            continue
        parts = [p for p in re.split(r"[,\t=\s]+", line) if p]
        if len(parts) < 2:
            continue
        try:
            params[parts[0].upper()] = float(parts[1])
        except ValueError:
            continue
    return params, meta


# ---------------------------------------------------------------------------
# 抓取
# ---------------------------------------------------------------------------
def _param_name(msg: Any) -> str:
    pid = getattr(msg, "param_id", "")
    if isinstance(pid, (bytes, bytearray)):
        pid = bytes(pid).decode("utf-8", "ignore")
    return str(pid).rstrip("\x00").strip().upper()


def fetch_all(
    conn: Any,
    *,
    target_system: int | None = None,
    target_component: int | None = None,
    idle_timeout_s: float = 3.0,
    total_timeout_s: float = 180.0,
    recover_rounds: int = 3,
    recover_timeout_s: float = 1.0,
    progress: Callable[[int, int], None] | None = None,
) -> tuple[dict[str, float], dict[str, int]]:
    """PARAM_REQUEST_LIST 抓全表，缺的逐 index 補抓。

    回 ``(params, types)``。抓不齊拋 :class:`IncompleteDump`。

    ⚠ 串流一定會掉包，這就是為什麼一定要有補抓：沒有補抓的「抓到多少算多少」
    會產生一份**看起來正常**的半份備份。
    """
    ts = conn.target_system if target_system is None else int(target_system)
    tc = conn.target_component if target_component is None else int(target_component)

    params: dict[str, float] = {}
    types: dict[str, int] = {}
    by_index: dict[int, str] = {}
    declared = 0
    deadline = time.monotonic() + float(total_timeout_s)

    def _absorb(msg: Any, assume_index: int | None = None) -> None:
        nonlocal declared
        name = _param_name(msg)
        if not name:
            return
        params[name] = as_f32(msg.param_value)
        types[name] = int(getattr(msg, "param_type", 9))
        count = int(getattr(msg, "param_count", 0) or 0)
        if count > 0:
            declared = max(declared, count)
        idx = int(getattr(msg, "param_index", -1))
        # 以名稱回應的 PARAM_REQUEST_READ，ArduPilot 會回 65535；
        # 但補抓是一次要一個 index，所以知道它該算哪一格。
        if not (0 <= idx < 65535) and assume_index is not None:
            idx = assume_index
        if 0 <= idx < 65535:
            by_index[idx] = name

    conn.mav.param_request_list_send(ts, tc)
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        msg = conn.recv_match(
            type="PARAM_VALUE", blocking=True,
            timeout=min(float(idle_timeout_s), remaining),
        )
        if msg is None:
            break                                   # 串流停了 → 進補抓
        _absorb(msg)
        if progress is not None and declared:
            progress(len(by_index), declared)
        if declared and len(by_index) >= declared:
            break

    if declared <= 0:
        raise IncompleteDump(0, len(params), [])

    for _ in range(int(recover_rounds)):
        missing = sorted(set(range(declared)) - set(by_index))
        if not missing:
            break
        for idx in missing:
            if time.monotonic() >= deadline:
                break
            conn.mav.param_request_read_send(ts, tc, b"", int(idx))
            msg = conn.recv_match(type="PARAM_VALUE", blocking=True,
                                  timeout=float(recover_timeout_s))
            if msg is not None:
                _absorb(msg, assume_index=idx)

    missing = sorted(set(range(declared)) - set(by_index))
    if missing:
        raise IncompleteDump(declared, len(by_index), missing)
    return params, types


# ---------------------------------------------------------------------------
# 存放
# ---------------------------------------------------------------------------
def snapshot_dir(root: str | os.PathLike[str], board: BoardId) -> Path:
    return Path(root) / board.key()


def save_snapshot(root: str | os.PathLike[str], snap: Snapshot,
                  *, dedupe: bool = True) -> Path:
    """寫出快照；``dedupe`` 時內容完全相同就沿用既有檔案，不重複堆檔。"""
    d = snapshot_dir(root, snap.board)
    d.mkdir(parents=True, exist_ok=True)
    if dedupe:
        digest = snap.content_hash()
        for existing in sorted(d.glob("*.parm")):
            try:
                got, _meta = parse_param_file(existing.read_text(encoding="utf-8"))
            except OSError:
                continue
            other = Snapshot(board=snap.board, params=got)
            if other.content_hash() == digest:
                return existing
    if not snap.taken_utc:
        snap.taken_utc = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    stamp = snap.taken_utc.replace(":", "").replace("-", "")
    path = d / f"{stamp}_{snap.content_hash()}.parm"
    path.write_text(render_param_file(snap), encoding="utf-8")
    return path


def load_snapshot(path: str | os.PathLike[str]) -> Snapshot:
    p = Path(path)
    params, meta = parse_param_file(p.read_text(encoding="utf-8"))
    board = BoardId(serial=meta.get("board_serial", "unknown"),
                    sysid=int(meta.get("sysid", "1") or 1))
    return Snapshot(board=board, params=params,
                    taken_utc=meta.get("taken_utc", ""), note=meta.get("note", ""))


def latest_snapshot(root: str | os.PathLike[str], board: BoardId) -> Path | None:
    d = snapshot_dir(root, board)
    files = sorted(d.glob("*.parm")) if d.is_dir() else []
    return files[-1] if files else None


# ---------------------------------------------------------------------------
# 寫入前自動備份
# ---------------------------------------------------------------------------
def backup_before_write(
    conn: Any,
    board: BoardId,
    root: str | os.PathLike[str],
    *,
    note: str = "",
    **fetch_kw: Any,
) -> Path:
    """抓全表並存檔，回傳檔案路徑。抓不齊就拋 —— 呼叫端不該繼續寫入。"""
    params, types = fetch_all(conn, **fetch_kw)
    snap = Snapshot(
        board=board, params=params, types=types,
        taken_utc=datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        note=note,
    )
    return save_snapshot(root, snap)


def write_params_with_backup(
    conn: Any,
    params: Mapping[str, float],
    *,
    board: BoardId,
    root: str | os.PathLike[str],
    setter: Callable[[Mapping[str, float]], Any],
    note: str = "",
    **fetch_kw: Any,
) -> tuple[Path, Any]:
    """**先備份、再寫入**。回 ``(備份路徑, setter 的回傳值)``。

    ⚠ 順序是這支函式存在的全部理由。備份失敗會拋，``setter``
    連呼叫都不會被呼叫 —— 寧可不寫，也不要在沒有退路的情況下蓋掉別人的設定。
    """
    path = backup_before_write(conn, board, root, note=note, **fetch_kw)
    return path, setter(params)


# ---------------------------------------------------------------------------
# 比對與還原
# ---------------------------------------------------------------------------
def diff_params(
    before: Mapping[str, float], after: Mapping[str, float],
    *, ignore: Iterable[str] | None = None,
) -> dict[str, tuple[float | None, float | None]]:
    """回 ``{name: (before, after)}``，只列不同者；缺席一側為 ``None``。

    比較在 float32 的格上做 —— 兩邊都經過 :func:`as_f32`，否則會拿 double
    的舊值跟 float32 的讀回值比而永遠不相等。

    ``ignore`` 預設是**空的**：問「有什麼不一樣」時要看見全部，包括
    :data:`VOLATILE_PARAMS`。要看「設定有沒有被改動」才傳
    ``ignore=VOLATILE_PARAMS``。
    """
    skip = {n.upper() for n in (ignore or ())}
    out: dict[str, tuple[float | None, float | None]] = {}
    for name in sorted(set(before) | set(after)):
        if name in skip:
            continue
        a = as_f32(before[name]) if name in before else None
        b = as_f32(after[name]) if name in after else None
        if a != b:
            out[name] = (a, b)
    return out


@dataclass
class RestoreReport:
    written: dict[str, tuple[float, float]] = field(default_factory=dict)   # name -> (現值, 目標值)
    unchanged: list[str] = field(default_factory=list)
    unknown: list[str] = field(default_factory=list)     # 備份裡有、板子上沒有
    volatile: list[str] = field(default_factory=list)    # 飛控自己會改，刻意不寫
    failed: dict[str, str] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return not self.failed


def restore(
    conn: Any,
    snap: Snapshot,
    *,
    setter: Callable[[Mapping[str, float]], Mapping[str, bool]],
    only: Iterable[str] | None = None,
    dry_run: bool = False,
    include_volatile: bool = False,
    **fetch_kw: Any,
) -> RestoreReport:
    """把快照寫回飛控，只寫**真的不同**的那些。

    ``setter`` 用既有的寫入實作（例如 ``MavlinkLink.set_params``），
    它必須回 ``{name: ok}``。

    三個刻意的保守選擇：

    * 備份裡有、但目前板子上**不存在**的參數列進 ``unknown`` 而**不寫**
      —— 可能是不同韌體/機型的備份，硬寫會製造垃圾參數。
    * :data:`VOLATILE_PARAMS` 列進 ``volatile`` 而**不寫**。
      ``MIS_TOTAL`` 是任務航點數 —— 把舊值寫回去會動到飛控裡存的任務；
      ``STAT_RUNTIME`` 寫回去是竄改統計。要寫得明講 ``include_volatile=True``。
    * 值相同的不寫 —— 減少對 FRAM 的寫入次數。
    """
    current, _types = fetch_all(conn, **fetch_kw)
    names = set(snap.params) if only is None else {n.upper() for n in only}
    report = RestoreReport()
    todo: dict[str, float] = {}
    for name in sorted(names):
        if name not in snap.params:
            continue
        target = as_f32(snap.params[name])
        if name not in current:
            report.unknown.append(name)
            continue
        if as_f32(current[name]) == target:
            report.unchanged.append(name)
            continue
        if name in VOLATILE_PARAMS and not include_volatile:
            report.volatile.append(name)
            continue
        todo[name] = target
        report.written[name] = (as_f32(current[name]), target)
    if todo and not dry_run:
        results = setter(todo) or {}
        for name in todo:
            if not results.get(name, False):
                report.failed[name] = "PARAM_SET 未確認"
    return report


# ---------------------------------------------------------------------------
# CLI — 無頭使用入口
# ---------------------------------------------------------------------------
def _connect(port: str, *, baud: int = 115200, timeout_s: float = 20.0) -> Any:
    from pymavlink import mavutil
    conn = mavutil.mavlink_connection(port, baud=baud)
    if not conn.wait_heartbeat(timeout=timeout_s):
        raise SystemExit(f"{port}: 等不到心跳（{timeout_s:g}s）")
    return conn


def _mav_setter(conn: Any) -> Callable[[Mapping[str, float]], dict[str, bool]]:
    """用 PARAM_SET + PARAM_VALUE 回讀確認的寫入器。"""
    def _set(params: Mapping[str, float]) -> dict[str, bool]:
        out: dict[str, bool] = {}
        for name, value in params.items():
            ok = False
            for _ in range(3):
                conn.mav.param_set_send(conn.target_system, conn.target_component,
                                        name.encode("ascii"), float(value), 9)
                msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=5)
                if msg is not None and _param_name(msg) == name.upper():
                    ok = as_f32(msg.param_value) == as_f32(value)
                    break
            out[name] = ok
        return out
    return _set


def main(argv: Sequence[str] | None = None) -> int:
    """``python -m mission.param_backup <dump|list|diff|restore> ...``

    ⚠ ``restore`` **預設是試跑**，要真的寫必須明講 ``--apply``。
    對硬體寫入的工具，安全的預設值是不寫。
    """
    import argparse

    ap = argparse.ArgumentParser(prog="param_backup", description="飛控參數備份／還原")
    ap.add_argument("action", choices=["dump", "list", "diff", "restore"])
    ap.add_argument("--port", help="COM26 / tcp:127.0.0.1:5760 / udp:0.0.0.0:14550")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--root", default="runs/param_backups")
    ap.add_argument("--sysid", type=int, default=0, help="0＝用連線回報的")
    ap.add_argument("--file", help="diff/restore 用的備份檔；省略則取該板最新一份")
    ap.add_argument("--apply", action="store_true", help="restore 真的寫入")
    ap.add_argument("--include-volatile", action="store_true",
                    help="連 VOLATILE_PARAMS 一起寫（預設不寫）")
    a = ap.parse_args(argv)
    root = Path(a.root)

    if a.action == "list":
        if not root.is_dir():
            print(f"{root} 不存在")
            return 0
        for d in sorted(p for p in root.iterdir() if p.is_dir()):
            files = sorted(d.glob("*.parm"))
            print(f"{d.name}  {len(files)} 份")
            for f in files:
                _p, meta = parse_param_file(f.read_text(encoding="utf-8"))
                print(f"    {f.name}  {meta.get('count', '?')} 個  "
                      f"{meta.get('taken_utc', '')}  {meta.get('note', '')}")
        return 0

    if not a.port:
        raise SystemExit("需要 --port")
    conn = _connect(a.port, baud=a.baud)
    sysid = a.sysid or conn.target_system
    board = board_id_for_port(a.port, sysid)
    print(f"連上 {a.port}  sysid={sysid}  識別={board.key()}")

    if a.action == "dump":
        path = backup_before_write(conn, board, root, note="手動 dump")
        n = len(parse_param_file(path.read_text(encoding="utf-8"))[0])
        print(f"已存 {path}（{n} 個參數）")
        return 0

    src = Path(a.file) if a.file else latest_snapshot(root, board)
    if src is None:
        raise SystemExit(f"{board.key()} 沒有備份，也沒給 --file")
    snap = load_snapshot(src)
    print(f"對照 {src}（{len(snap.params)} 個參數）")

    if a.action == "diff":
        current, _t = fetch_all(conn)
        d = diff_params(snap.params, current)
        cfg = {k: v for k, v in d.items() if k not in VOLATILE_PARAMS}
        print(f"設定面差異 {len(cfg)} 個（另有 {len(d) - len(cfg)} 個屬飛控自行變動）")
        for name, (before, after) in cfg.items():
            print(f"  {name:<22} 備份 {before!r:<22} 飛機 {after!r}")
        return 0

    rep = restore(conn, snap, setter=_mav_setter(conn),
                  dry_run=not a.apply, include_volatile=a.include_volatile)
    head = "已寫入" if a.apply else "試跑（未寫入，要寫加 --apply）"
    print(f"{head}：需改 {len(rep.written)}、相同 {len(rep.unchanged)}、"
          f"板上沒有 {len(rep.unknown)}、刻意不寫 {len(rep.volatile)}、"
          f"失敗 {len(rep.failed)}")
    for name, (before, after) in rep.written.items():
        print(f"  {name:<22} {before!r} -> {after!r}")
    if rep.unknown:
        print(f"  板上沒有：{rep.unknown}")
    if rep.volatile:
        print(f"  飛控自行維護、刻意不寫：{sorted(rep.volatile)}")
    if rep.failed:
        print(f"  !! 失敗：{rep.failed}")
    return 0 if rep.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
