"""hv_dccpp/mavlink_link.py — pymavlink 整合層（設計文件 §15.5；可選依賴 pymavlink，`MAVLINK20=1`）。

功能
----
* :class:`VehicleLink` — 單機連線：心跳、任務／rally／fence 上傳（MISSION_COUNT → MISSION_REQUEST_INT → MISSION_ITEM_INT →
  MISSION_ACK，含逾時重送）、下載、清除、PARAM_SET／PARAM_VALUE、模式（ArduPlane 模式表）、ARM、MISSION_START、
  MISSION_SET_CURRENT、遙測彙整（GLOBAL_POSITION_INT／VFR_HUD／SYS_STATUS／BATTERY_STATUS／WIND／EXTENDED_SYS_STATE／
  MISSION_CURRENT／HEARTBEAT）→ :class:`VehicleTelemetry`。
* :func:`to_planner_telemetry` — :class:`VehicleTelemetry` → :class:`hv_dccpp.dynamic.Telemetry`（WGS84 → ENU 經 ``ctx.frame``；
  能量 = 電量% × 標稱 或 Wh；現行條帶由 MISSION_CURRENT seq 經 :class:`MissionSeqMap` 對應；§15.5.2）。
* :func:`detect_events` — LinkLoss／LinkRestored／EnergyUpdate（偏離計畫 > δ₁·E_nom，含 :class:`EnergyEventState`
  遲滯／防抖）／WindUpdate／UAVExit（§15.5.2 事件規則）。
* :class:`FleetLink` — 多機 ``{uav_id: VehicleLink}``：:meth:`FleetLink.upload_plan`（任務 + rally + fence + 檔案輸出）與
  :meth:`FleetLink.replan_and_reupload`（重傳變更後 UAVPlan 並 MISSION_SET_CURRENT 至下一條帶起點）。
* :func:`mission_seq_map` — 任務項目 → seq→條帶對照（``<uav_id>.seqmap.json`` 格式，§15.5.2）。

`hv_dccpp.ardupilot`（Plan → QuadPlane 任務項目）存在時優先使用其 ``plan_to_mission_items``／rally／fence 函式；不存在時採本模組之
私有後備轉換（:func:`plan_to_mission_items_fallback`，記錄於 CONTRACT_DEVIATIONS.md）。

Windows UDP：對端回 ICMP port-unreachable 時 recvfrom 會丟 WinError 10054，建構時以 ``SIO_UDP_CONNRESET`` 關閉
（與 ``tools/mock_autopilot.py`` 相同）。

所有 MAV_CMD／MAV_FRAME／狀態常數一律取自 ``mavutil.mavlink``，不手寫數字。
"""
from __future__ import annotations

import inspect
import json
import math
import os
import socket
import time
from dataclasses import asdict, dataclass, field, fields, replace
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping, Sequence

import numpy as np

# ArduPilot 使用 MAVLink 2；mission_type 等為 MAVLink 2 擴充欄位，必須在 import pymavlink 前設定。
os.environ.setdefault("MAVLINK20", "1")

try:  # 可選依賴
    from pymavlink import mavutil as _mavutil
    _mav = _mavutil.mavlink
    HAVE_PYMAVLINK = True
except ImportError:  # pragma: no cover - 取決於環境
    _mavutil = None
    _mav = None
    HAVE_PYMAVLINK = False

from .mission import FW_PHASES, MissionPlan, Phase, PlannerConfig, Site, UAVPlan, Waypoint, WindLayer

_PYMAVLINK_MSG = ("hv_dccpp.mavlink_link 需要 pymavlink（pip install pymavlink）；"
                  "純規劃功能不受影響，只有 MAVLink 上傳／遙測需要此套件。")

__all__ = [
    "HAVE_PYMAVLINK", "MavlinkLinkError", "MavlinkUploadError", "MavlinkTimeoutError",
    "VehicleTelemetry", "MissionItemInt", "MissionItems", "SeqEntry", "MissionSeqMap", "UploadResult",
    "PlanUploadReport", "ReuploadResult", "VehicleLink", "FleetLink",
    "as_mission_item", "mission_seq_map", "items_equal", "write_qgc_wpl", "read_qgc_wpl",
    "plan_to_mission_items", "plan_to_mission_items_fallback", "rally_items", "fence_items", "suggest_params",
    "to_planner_telemetry", "detect_events", "EnergyEventState", "plane_mode_map", "PLANE_MODE_NAMES",
]


# ---------------------------------------------------------------------------
# 例外與常數
# ---------------------------------------------------------------------------
class MavlinkLinkError(RuntimeError):
    """MAVLink 整合層一般錯誤。"""


class MavlinkUploadError(MavlinkLinkError):
    """MISSION_ACK ≠ MAV_MISSION_ACCEPTED 或協定失敗。"""


class MavlinkTimeoutError(MavlinkLinkError, TimeoutError):
    """等待訊息逾時。"""


def _require_pymavlink() -> Any:
    if not HAVE_PYMAVLINK:
        raise ImportError(_PYMAVLINK_MSG)
    return _mav


PLANE_MODE_NAMES: tuple[str, ...] = (
    "MANUAL", "CIRCLE", "STABILIZE", "TRAINING", "ACRO", "FBWA", "FBWB", "CRUISE", "AUTOTUNE", "AUTO", "RTL", "LOITER",
    "TAKEOFF", "AVOID_ADSB", "GUIDED", "INITIALISING", "QSTABILIZE", "QHOVER", "QLOITER", "QLAND", "QRTL", "QAUTOTUNE",
    "QACRO", "THERMAL", "LOITERALTQLAND", "AUTOLAND",
)
"""ArduPlane 模式名（供文件；實際編號取 ``mavutil.mode_mapping_apm``）。"""


def plane_mode_map() -> dict[str, int]:
    """ArduPlane 模式名 → custom_mode（取自 ``mavutil.mode_mapping_apm``；QuadPlane 韌體以 MAV_TYPE_FIXED_WING 回報）。"""
    _require_pymavlink()
    return {str(v).upper(): int(k) for k, v in _mavutil.mode_mapping_apm.items()}


def _nav_cmds() -> frozenset[int]:
    m = _require_pymavlink()
    return frozenset({
        m.MAV_CMD_NAV_WAYPOINT, m.MAV_CMD_NAV_VTOL_TAKEOFF, m.MAV_CMD_NAV_VTOL_LAND, m.MAV_CMD_NAV_LOITER_TO_ALT,
        m.MAV_CMD_NAV_LOITER_TURNS, m.MAV_CMD_NAV_LOITER_TIME, m.MAV_CMD_NAV_LOITER_UNLIM,
        m.MAV_CMD_NAV_RETURN_TO_LAUNCH, m.MAV_CMD_NAV_TAKEOFF, m.MAV_CMD_NAV_LAND, m.MAV_CMD_NAV_SPLINE_WAYPOINT,
    })


def _disable_windows_udp_reset(sock: Any) -> None:
    """Windows：UDP 對端回 ICMP port-unreachable 後 recvfrom 丟 WinError 10054；關掉 SIO_UDP_CONNRESET。"""
    if sock is None or not hasattr(socket, "SIO_UDP_CONNRESET"):
        return
    try:
        sock.ioctl(socket.SIO_UDP_CONNRESET, False)
    except (OSError, AttributeError):
        pass


# ---------------------------------------------------------------------------
# 資料型別
# ---------------------------------------------------------------------------
@dataclass(slots=True)
class VehicleTelemetry:
    """自駕儀遙測彙整（§15.5.2 訊息 → 欄位）。

    ``t_s``：相對 :attr:`VehicleLink.t0_epoch_s` 之秒數（``time_source='wall'``）或 ``time_boot_ms/1000 − boot_offset``
    （``time_source='boot'``；mock 之 time_boot_ms 為模擬時間）。``alt_msl_m`` 取 GLOBAL_POSITION_INT.alt（mm MSL；D42，
    不用 relative_alt 做 ENU 換算）。``vtol_state``／``landed_state`` 為 MAV_VTOL_STATE_*／MAV_LANDED_STATE_* 整數。
    ``track_deg`` 為航跡方位（自北順時針；由 vx/vy 推得，或無 vx/vy 時由連續位置差推得；皆無 → None）；
    ``groundspeed_mps`` 依序取 VFR_HUD.groundspeed → |vx,vy| → 位置差／時間差推估。
    """
    t_s: float
    lat_deg: float
    lon_deg: float
    alt_msl_m: float
    rel_alt_m: float
    heading_deg: float
    groundspeed_mps: float
    airspeed_mps: float | None
    battery_pct: float | None
    battery_wh_remaining: float | None
    wind_speed_mps: float | None
    wind_dir_from_deg: float | None
    vtol_state: int
    landed_state: int
    mission_seq: int
    mode: str
    armed: bool
    link_ok: bool
    track_deg: float | None = None
    climb_mps: float | None = None
    heartbeat_age_s: float = math.inf
    reached_seq: int | None = None
    t_epoch_s: float = 0.0
    time_boot_ms: int = 0
    sysid: int = 0

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class MissionItemInt:
    """一筆 MISSION_ITEM_INT（x/y 以「度」儲存，送出時 ×1e7）。尾端為 seqmap 中繼資料（不上傳）。"""
    seq: int
    frame: int
    command: int
    current: int
    autocontinue: int
    param1: float
    param2: float
    param3: float
    param4: float
    x: float          # lat (deg)
    y: float          # lon (deg)
    z: float          # alt (m; 依 frame)
    mission_type: int = 0
    # --- 中繼資料（seqmap；不進 MAVLink）---
    phase: str | None = None
    strip_id: str | None = None
    dir: int | None = None
    wp_index: int | None = None
    s_frac_start: float = 0.0
    s_frac_end: float = 1.0

    @property
    def lat_deg(self) -> float:
        return self.x

    @property
    def lon_deg(self) -> float:
        return self.y

    @property
    def alt_m(self) -> float:
        return self.z

    def to_row(self) -> str:
        """QGC WPL 110 一列（恰 12 欄，tab 分隔）。"""
        return "\t".join([
            str(int(self.seq)), str(int(self.current)), str(int(self.frame)), str(int(self.command)),
            _fmt(self.param1), _fmt(self.param2), _fmt(self.param3), _fmt(self.param4),
            f"{self.x:.8f}", f"{self.y:.8f}", _fmt(self.z), str(int(self.autocontinue)),
        ])

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_msg(cls, msg: Any) -> "MissionItemInt":
        """自 MISSION_ITEM_INT／MISSION_ITEM 訊息建立。"""
        t = msg.get_type()
        if t == "MISSION_ITEM_INT":
            x, y = msg.x / 1e7, msg.y / 1e7
        else:
            x, y = float(msg.x), float(msg.y)
        return cls(int(msg.seq), int(msg.frame), int(msg.command), int(msg.current), int(msg.autocontinue),
                   float(msg.param1), float(msg.param2), float(msg.param3), float(msg.param4), float(x), float(y),
                   float(msg.z), int(getattr(msg, "mission_type", 0)))


def _fmt(v: float) -> str:
    v = float(v)
    if math.isfinite(v) and v == int(v) and abs(v) < 1e15:
        return f"{int(v)}"
    return f"{v:.6f}".rstrip("0").rstrip(".")


class MissionItems(list):
    """`list[MissionItemInt]`，附 ``seqmap``（:class:`MissionSeqMap` 或 None）與 ``mission_type`` 屬性。"""
    __slots__ = ("seqmap", "mission_type")

    def __init__(self, items: Iterable[Any] = (), *, seqmap: "MissionSeqMap | None" = None, mission_type: int = 0):
        super().__init__(items)
        self.seqmap = seqmap
        self.mission_type = int(mission_type)


_META_KEYS = ("phase", "strip_id", "dir", "wp_index", "s_frac_start", "s_frac_end")


def as_mission_item(obj: Any, seq: int | None = None, mission_type: int | None = None) -> MissionItemInt:
    """任何任務項目表示 → :class:`MissionItemInt`。

    接受：:class:`MissionItemInt`；pymavlink MISSION_ITEM(_INT) 訊息；dict（鍵 seq/frame/command/current/autocontinue/
    param1..4 與 x,y,z 或 lat(_deg)/lon(_deg)/alt(_m)）；12 欄 QGC 列（tuple/list）或 9 欄 (frame, cmd, p1..p4, lat, lon, alt)；
    任何具同名屬性之物件（例如 `hv_dccpp.ardupilot` 之項目型別）。中繼資料鍵（phase/strip_id/dir/wp_index/s_frac_*）若存在即帶入。
    """
    if isinstance(obj, MissionItemInt):
        it = replace(obj)
    elif hasattr(obj, "get_type") and callable(obj.get_type):
        it = MissionItemInt.from_msg(obj)
    elif isinstance(obj, Mapping):
        it = _item_from_mapping(obj)
    elif isinstance(obj, (tuple, list)):
        it = _item_from_row(obj)
    else:
        it = _item_from_mapping({k: getattr(obj, k) for k in dir(obj) if not k.startswith("_")
                                 and not callable(getattr(obj, k, None))})
    if seq is not None:
        it.seq = int(seq)
    if mission_type is not None:
        it.mission_type = int(mission_type)
    if isinstance(it.phase, Phase):
        it.phase = it.phase.name
    return it


def _pick(d: Mapping[str, Any], *keys: str, default: Any = None) -> Any:
    for k in keys:
        if k in d and d[k] is not None:
            return d[k]
    return default


def _item_from_mapping(d: Mapping[str, Any]) -> MissionItemInt:
    x = _pick(d, "x", "lat_deg", "lat", default=0.0)
    y = _pick(d, "y", "lon_deg", "lon", default=0.0)
    z = _pick(d, "z", "alt_m", "alt", "alt_rel_m", default=0.0)
    # x/y 若為 int（1e7 度）→ 轉度
    if isinstance(x, (int, np.integer)) and not isinstance(x, bool) and abs(int(x)) > 1000:
        x = int(x) / 1e7
    if isinstance(y, (int, np.integer)) and not isinstance(y, bool) and abs(int(y)) > 1000:
        y = int(y) / 1e7
    it = MissionItemInt(
        seq=int(_pick(d, "seq", default=0)), frame=int(_pick(d, "frame", default=3)),
        command=int(_pick(d, "command", "cmd", default=16)), current=int(_pick(d, "current", default=0)),
        autocontinue=int(_pick(d, "autocontinue", default=1)),
        param1=float(_pick(d, "param1", "p1", default=0.0)), param2=float(_pick(d, "param2", "p2", default=0.0)),
        param3=float(_pick(d, "param3", "p3", default=0.0)), param4=float(_pick(d, "param4", "p4", default=0.0)),
        x=float(x), y=float(y), z=float(z), mission_type=int(_pick(d, "mission_type", default=0)),
    )
    for k in _META_KEYS:
        if k in d and d[k] is not None:
            v = d[k]
            if k == "phase":
                v = v.name if isinstance(v, Phase) else str(v)
            elif k == "strip_id":
                v = str(v)
            elif k in ("dir", "wp_index"):
                v = int(v)
            else:
                v = float(v)
            setattr(it, k, v)
    return it


def _item_from_row(row: Sequence[Any]) -> MissionItemInt:
    r = list(row)
    if len(r) == 12:
        seq, cur, frame, cmd, p1, p2, p3, p4, lat, lon, alt, ac = r
    elif len(r) == 9:
        frame, cmd, p1, p2, p3, p4, lat, lon, alt = r
        seq, cur, ac = 0, 0, 1
    else:
        raise ValueError(f"任務列須為 12 欄（QGC）或 9 欄，得 {len(r)} 欄")
    return MissionItemInt(int(seq), int(frame), int(cmd), int(cur), int(ac), float(p1), float(p2), float(p3),
                          float(p4), float(lat), float(lon), float(alt))


def items_equal(a: Sequence[Any], b: Sequence[Any], *, tol_deg: float = 2e-7, tol_alt_m: float = 1e-3,
                tol_param: float = 1e-4) -> bool:
    """兩份任務逐項比對（seq/frame/command/current/autocontinue 相等，參數與座標在容差內；忽略中繼資料）。"""
    if len(a) != len(b):
        return False
    for u, v in zip(a, b):
        p, q = as_mission_item(u), as_mission_item(v)
        if (p.seq, p.frame, p.command, p.current, p.autocontinue) != (q.seq, q.frame, q.command, q.current, q.autocontinue):
            return False
        for k in ("param1", "param2", "param3", "param4"):
            if not _close(getattr(p, k), getattr(q, k), tol_param):
                return False
        if not (_close(p.x, q.x, tol_deg) and _close(p.y, q.y, tol_deg) and _close(p.z, q.z, tol_alt_m)):
            return False
    return True


def _close(a: float, b: float, tol: float) -> bool:
    a, b = float(a), float(b)
    if math.isnan(a) and math.isnan(b):
        return True
    return abs(a - b) <= tol


# ---------------------------------------------------------------------------
# seq → 條帶對照（§15.5.2 seqmap.json）
# ---------------------------------------------------------------------------
@dataclass(slots=True)
class SeqEntry:
    """一個 seq 之條帶進度基準：``phase`` 為 `Phase` 名稱字串；``s_frac_*`` 為該項目起／訖點在條帶上的弧長比例
    （非 OPERATE 為 0/1）；``wp_index`` = `UAVPlan.waypoints` 索引（None = 無對應航點，如 DO_ 指令／home）。"""
    phase: str | None
    strip_id: str | None
    dir: int | None
    s_frac_start: float
    s_frac_end: float
    wp_index: int | None

    @property
    def phase_enum(self) -> Phase | None:
        return Phase[self.phase] if self.phase else None


@dataclass(slots=True)
class MissionSeqMap:
    """seq → :class:`SeqEntry`（``<uav_id>.seqmap.json``）。"""
    uav_id: str
    items: dict[int, SeqEntry] = field(default_factory=dict)
    n_items: int = 0
    frame: int = 3
    home_msl_m: float = 0.0
    mission_id: str = ""

    # -- 查詢 --------------------------------------------------------------
    def entry(self, seq: int) -> SeqEntry | None:
        return self.items.get(int(seq))

    def strip_at(self, seq: int) -> tuple[str | None, bool]:
        """(strip_id, in_strip)：seq 為 OPERATE 項目 → 該條帶、True；否則 → 前一個 OPERATE 條帶（已完成）、False；皆無 → (None, False)。"""
        e = self.items.get(int(seq))
        if e is not None and e.phase == Phase.OPERATE.name and e.strip_id is not None:
            return e.strip_id, True
        for s in sorted((k for k in self.items if k < int(seq)), reverse=True):
            ee = self.items[s]
            if ee.phase == Phase.OPERATE.name and ee.strip_id is not None:
                return ee.strip_id, False
        return None, False

    def strip_seqs(self, strip_id: str) -> list[int]:
        """該條帶之全部 OPERATE seq（升冪）。"""
        return sorted(k for k, e in self.items.items() if e.strip_id == strip_id and e.phase == Phase.OPERATE.name)

    def strip_end_seq(self, strip_id: str) -> int | None:
        s = self.strip_seqs(strip_id)
        return s[-1] if s else None

    def strip_start_seq(self, strip_id: str) -> int | None:
        s = self.strip_seqs(strip_id)
        return s[0] if s else None

    def seq_after_strip(self, strip_id: str) -> int | None:
        """現行條帶末端之後的第一個 seq（= 下一條帶之轉移起點；重規劃切換點）。"""
        e = self.strip_end_seq(strip_id)
        if e is None:
            return None
        later = sorted(k for k in self.items if k > e)
        return later[0] if later else None

    def seq_of_wp(self, wp_index: int) -> int | None:
        """首個對應 `wp_index ≥ 給定值` 之 seq。"""
        cands = sorted((k for k, e in self.items.items() if e.wp_index is not None and e.wp_index >= wp_index))
        return cands[0] if cands else None

    # -- 序列化 ------------------------------------------------------------
    def to_dict(self) -> dict[str, Any]:
        return {"uav_id": self.uav_id, "mission_id": self.mission_id, "n_items": int(self.n_items),
                "frame": int(self.frame), "home_msl_m": float(self.home_msl_m),
                "items": {str(k): asdict(v) for k, v in sorted(self.items.items())}}

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "MissionSeqMap":
        items = {int(k): SeqEntry(phase=v.get("phase"), strip_id=v.get("strip_id"),
                                  dir=None if v.get("dir") is None else int(v["dir"]),
                                  s_frac_start=float(v.get("s_frac_start", 0.0)),
                                  s_frac_end=float(v.get("s_frac_end", 1.0)),
                                  wp_index=None if v.get("wp_index") is None else int(v["wp_index"]))
                 for k, v in dict(d.get("items", {})).items()}
        return cls(uav_id=str(d.get("uav_id", "")), items=items, n_items=int(d.get("n_items", len(items))),
                   frame=int(d.get("frame", 3)), home_msl_m=float(d.get("home_msl_m", 0.0)),
                   mission_id=str(d.get("mission_id", "")))

    def save(self, path: str | os.PathLike[str]) -> str:
        p = Path(path)
        p.write_text(json.dumps(self.to_dict(), indent=1, ensure_ascii=False), encoding="utf-8")
        return str(p)

    @classmethod
    def load(cls, path: str | os.PathLike[str]) -> "MissionSeqMap":
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))


def _oper_spans(plan: UAVPlan) -> dict[str, tuple[np.ndarray, np.ndarray, int, int, int]]:
    """strip_id → (p_start xy, p_end xy, dir, i0, i_end)，與 `dynamic._spans` 同語意（i_end = 條帶後第一個航點）。"""
    wps = plan.waypoints
    out: dict[str, tuple[np.ndarray, np.ndarray, int, int, int]] = {}
    n = len(wps)
    i = 0
    while i < n:
        w = wps[i]
        if w.phase is Phase.OPERATE and w.strip_id is not None:
            j = i
            while j < n and wps[j].phase is Phase.OPERATE and wps[j].strip_id == w.strip_id:
                j += 1
            i_end = min(j, n - 1)
            p0 = np.array([wps[i].x_m, wps[i].y_m], dtype=float)
            p1 = np.array([wps[i_end].x_m, wps[i_end].y_m], dtype=float)
            if w.strip_id not in out:
                out[w.strip_id] = (p0, p1, int(w.dir if w.dir is not None else -1), i, i_end)
            i = j
        else:
            i += 1
    return out


def _s_frac(p_xy: np.ndarray, span: tuple[np.ndarray, np.ndarray, int, int, int]) -> float:
    p0, p1 = span[0], span[1]
    d = p1 - p0
    L2 = float(d @ d)
    if L2 <= 1e-9:
        return 1.0
    return float(min(1.0, max(0.0, float((np.asarray(p_xy, dtype=float) - p0) @ d) / L2)))


def _nearest_wp(plan: UAVPlan, lat: float, lon: float, ctx: Any | None) -> tuple[int, float]:
    """最近航點索引與距離（m；等距投影）。"""
    wps = plan.waypoints
    if not wps:
        return -1, math.inf
    lats = np.array([w.lat_deg for w in wps])
    lons = np.array([w.lon_deg for w in wps])
    k = 111_320.0
    dy = (lats - lat) * k
    dx = (lons - lon) * k * math.cos(math.radians(lat))
    d = np.hypot(dx, dy)
    j = int(np.argmin(d))
    return j, float(d[j])


def mission_seq_map(items: Sequence[Any], plan: UAVPlan | None = None, ctx: Any | None = None, *,
                    uav_id: str | None = None, match_tol_m: float = 5.0, home_msl_m: float | None = None,
                    mission_id: str = "") -> MissionSeqMap:
    """任務項目 → :class:`MissionSeqMap`（§15.5.2）。

    規則：項目自帶中繼資料（`strip_id`/`phase`/`wp_index`）者直接採用；否則若給 `plan`，NAV 項目以 lat/lon 對應最近之
    `plan.waypoints`（≤ `match_tol_m`）取 phase／strip_id／dir／wp_index，OPERATE 之 `s_frac_end` 為該點在條帶（首 OPER 航點 →
    條帶後首航點）上的投影比例，`s_frac_start` = 同條帶前一項目之 `s_frac_end`；無座標之 DO_ 指令繼承下一個 NAV 項目。
    若 `items` 為 :class:`MissionItems`（或可設屬性之 list 子類）則同時寫入 ``items.seqmap``。
    """
    nav = _nav_cmds()
    its = [as_mission_item(x) for x in items]
    spans = _oper_spans(plan) if plan is not None else {}
    span_end = {v[4]: k for k, v in spans.items()}     # 條帶後首航點索引 → strip_id
    entries: dict[int, SeqEntry] = {}
    resolved: list[SeqEntry | None] = [None] * len(its)
    # 第一輪：自帶中繼資料或位置對應
    for i, it in enumerate(its):
        has_meta = it.strip_id is not None or it.phase is not None or it.wp_index is not None
        if has_meta:
            resolved[i] = SeqEntry(it.phase, it.strip_id, it.dir, float(it.s_frac_start), float(it.s_frac_end), it.wp_index)
            continue
        if plan is None or it.command not in nav or (it.x == 0.0 and it.y == 0.0):
            continue
        j, d = _nearest_wp(plan, it.x, it.y, ctx)
        if j < 0 or d > match_tol_m:
            continue
        w = plan.waypoints[j]
        if i == 0 and it.current == 1 and it.command == _mav.MAV_CMD_NAV_WAYPOINT and it.frame == _mav.MAV_FRAME_GLOBAL:
            continue  # home 列
        end_of = span_end.get(j)
        if end_of is not None:
            # 條帶末端 p^R/p^L（計畫上屬 TRANSFER/RTB 首點）：抵達此點之航段為作業段 → OPERATE、s_frac_end = 1
            sp_ = spans[end_of]
            resolved[i] = SeqEntry(Phase.OPERATE.name, end_of, sp_[2], 0.0, 1.0, j)
            continue
        sf = 1.0
        if w.phase is Phase.OPERATE and w.strip_id in spans:
            sf = _s_frac(np.array([w.x_m, w.y_m]), spans[w.strip_id])
        resolved[i] = SeqEntry(w.phase.name, w.strip_id, w.dir, 0.0, sf if w.phase is Phase.OPERATE else 1.0, j)
    # 第二輪 a：未對應之 NAV 點（弧段重新離散點）：phase 取下一個已對應 NAV；strip_id 只在前後皆同一 OPERATE 條帶時繼承
    prev_nav: SeqEntry | None = None
    next_nav: list[SeqEntry | None] = [None] * len(its)
    nn: SeqEntry | None = None
    for i in range(len(its) - 1, -1, -1):
        next_nav[i] = nn
        if resolved[i] is not None and its[i].command in nav:
            nn = resolved[i]
    for i, it in enumerate(its):
        if resolved[i] is None and it.command in nav and next_nav[i] is not None and not (it.x == 0.0 and it.y == 0.0):
            nx = next_nav[i]
            same_strip = (prev_nav is not None and nx.strip_id is not None and prev_nav.strip_id == nx.strip_id
                          and nx.phase == Phase.OPERATE.name and prev_nav.phase == Phase.OPERATE.name)
            if same_strip:
                sf = _s_frac(np.asarray(ctx.frame.to_enu(it.x, it.y, 0.0)[0][:2]), spans[nx.strip_id]) \
                    if (ctx is not None and nx.strip_id in spans) else nx.s_frac_start
                resolved[i] = SeqEntry(nx.phase, nx.strip_id, nx.dir, 0.0, sf, None)
            else:
                ph = nx.phase if nx.phase != Phase.OPERATE.name else (prev_nav.phase if prev_nav is not None else None)
                resolved[i] = SeqEntry(ph, None, None, 0.0, 1.0, None)
        if resolved[i] is not None and it.command in nav:
            prev_nav = resolved[i]
    # 第二輪 b：DO_ 指令繼承下一個 NAV 項目；s_frac_start 串接
    nxt: SeqEntry | None = None
    for i in range(len(its) - 1, -1, -1):
        e = resolved[i]
        if e is None and its[i].command not in nav and nxt is not None:
            e = SeqEntry(nxt.phase, nxt.strip_id, nxt.dir, nxt.s_frac_start, nxt.s_frac_start, None)
            resolved[i] = e
        if e is not None and its[i].command in nav:
            nxt = e
    last_frac: dict[str, float] = {}
    for i, it in enumerate(its):
        e = resolved[i]
        if e is None:
            continue
        if e.phase == Phase.OPERATE.name and e.strip_id is not None and it.command in nav and it.wp_index is None \
                and it.strip_id is None:
            e.s_frac_start = last_frac.get(e.strip_id, 0.0)
            last_frac[e.strip_id] = e.s_frac_end
        entries[int(it.seq)] = e
    # 寫回原始 MissionItemInt 之中繼資料（items 為 MissionItemInt 時）
    for orig, e in zip(items, resolved):
        if isinstance(orig, MissionItemInt) and e is not None:
            orig.phase, orig.strip_id, orig.dir, orig.wp_index = e.phase, e.strip_id, e.dir, e.wp_index
            orig.s_frac_start, orig.s_frac_end = float(e.s_frac_start), float(e.s_frac_end)
    uid = uav_id or (plan.uav_id if plan is not None else "")
    frame = its[1].frame if len(its) > 1 else (its[0].frame if its else 3)
    hm = home_msl_m
    if hm is None:
        hm = its[0].z if its and its[0].frame == getattr(_mav, "MAV_FRAME_GLOBAL", 0) else 0.0
    sm = MissionSeqMap(uav_id=uid, items=entries, n_items=len(its), frame=int(frame), home_msl_m=float(hm),
                       mission_id=mission_id)
    try:
        items.seqmap = sm  # type: ignore[attr-defined]
    except (AttributeError, TypeError):
        pass
    return sm


# ---------------------------------------------------------------------------
# QGC WPL 110 讀寫
# ---------------------------------------------------------------------------
def write_qgc_wpl(items: Sequence[Any], path: str | os.PathLike[str]) -> str:
    """寫 QGC WPL 110（第 0 列 home：frame 0、cmd 16、current 1；恰 12 欄）。"""
    lines = ["QGC WPL 110"]
    for i, it in enumerate(items):
        m = as_mission_item(it)
        m.seq = i
        lines.append(m.to_row())
    p = Path(path)
    p.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return str(p)


def read_qgc_wpl(path: str | os.PathLike[str], mission_type: int = 0) -> MissionItems:
    txt = Path(path).read_text(encoding="utf-8").splitlines()
    if not txt or not txt[0].startswith("QGC WPL"):
        raise ValueError(f"{path}: 非 QGC WPL 檔")
    out = MissionItems(mission_type=mission_type)
    for line in txt[1:]:
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        cols = line.split("\t")
        if len(cols) != 12:
            raise ValueError(f"{path}: 列須 12 欄，得 {len(cols)}：{line!r}")
        it = _item_from_row(cols)
        it.mission_type = mission_type
        out.append(it)
    return out


# ---------------------------------------------------------------------------
# Plan → 任務項目（優先 hv_dccpp.ardupilot；後備轉換）
# ---------------------------------------------------------------------------
def _ardupilot_module() -> Any | None:
    try:
        from . import ardupilot as _ap  # type: ignore
        return _ap
    except ImportError:
        return None


def _call_compat(fn: Callable[..., Any], *args: Any, **kw: Any) -> Any:
    """依函式簽章只傳它接受的關鍵字（容許 ardupilot 之簽章差異）。"""
    try:
        sig = inspect.signature(fn)
    except (TypeError, ValueError):
        return fn(*args)
    params = sig.parameters
    accepts_var = any(p.kind is inspect.Parameter.VAR_KEYWORD for p in params.values())
    kws = {k: v for k, v in kw.items() if accepts_var or k in params}
    n_pos = sum(1 for p in params.values() if p.kind in (inspect.Parameter.POSITIONAL_ONLY,
                                                          inspect.Parameter.POSITIONAL_OR_KEYWORD))
    if any(p.kind is inspect.Parameter.VAR_POSITIONAL for p in params.values()):
        n_pos = len(args)
    return fn(*args[:n_pos], **kws)


def _home_msl(plan: UAVPlan, ctx: Any) -> tuple[float, float, float]:
    """(lat, lon, msl) 起飛點地面：首航點 lat/lon，MSL = alt_msl − agl（首航點於地面／H_ts 皆成立）。"""
    w0 = plan.waypoints[0]
    return float(w0.lat_deg), float(w0.lon_deg), float(w0.alt_msl_m - w0.agl_m)


_ALT_FRAME_NAMES = {"relative": "MAV_FRAME_GLOBAL_RELATIVE_ALT", "msl": "MAV_FRAME_GLOBAL",
                    "terrain": "MAV_FRAME_GLOBAL_TERRAIN_ALT"}


def _alt_frame_name(frame: int | str | None) -> str:
    """MAV_FRAME 整數（或 'relative'/'msl'/'terrain'）→ `ardupilot.AltFrame` 名稱；None → 'relative'。"""
    if frame is None:
        return "relative"
    if isinstance(frame, str):
        if frame not in _ALT_FRAME_NAMES:
            raise ValueError(f"alt_frame 須為 {tuple(_ALT_FRAME_NAMES)}，得 {frame!r}")
        return frame
    m = _require_pymavlink()
    for name, const in _ALT_FRAME_NAMES.items():
        if int(frame) == int(getattr(m, const)):
            return name
    raise ValueError(f"不支援之 MAV_FRAME {frame}（只支援 GLOBAL / GLOBAL_RELATIVE_ALT / GLOBAL_TERRAIN_ALT）")


def plan_to_mission_items(plan: UAVPlan, ctx: Any, cfg: PlannerConfig | None = None, *,
                          frame: int | str | None = None, mission_id: str = "", **kw: Any) -> MissionItems:
    """UAVPlan → :class:`MissionItems`（含 ``.seqmap``）。

    `hv_dccpp.ardupilot.plan_to_mission_items(plan, ctx, cfg, alt_frame, …)` 存在 → 呼叫之（`frame` 可為 MAV_FRAME 整數或
    'relative'/'msl'/'terrain'；其餘關鍵字如 `camera_trigger`、`wp_radius_m`、`strip_sensors`、`z_tol_m` 透傳），
    結果經 :func:`as_mission_item` 正規化並以 :func:`mission_seq_map`（以 lat/lon 對應計畫航點）建 seqmap；
    否則採 :func:`plan_to_mission_items_fallback`。
    """
    cfg = cfg or getattr(ctx, "cfg", None) or PlannerConfig()
    ap = _ardupilot_module()
    if ap is not None and hasattr(ap, "plan_to_mission_items"):
        raw = _call_compat(ap.plan_to_mission_items, plan, ctx, cfg, alt_frame=_alt_frame_name(frame), **kw)
        seqmap_in = getattr(raw, "seqmap", None)
        items = MissionItems([as_mission_item(x, seq=i) for i, x in enumerate(raw)])
        if isinstance(seqmap_in, MissionSeqMap):
            items.seqmap = seqmap_in
        elif isinstance(seqmap_in, Mapping) and "items" in seqmap_in:
            items.seqmap = MissionSeqMap.from_dict(seqmap_in)
        else:
            lat, lon, hm = _home_msl(plan, ctx)
            mission_seq_map(items, plan, ctx, uav_id=plan.uav_id, home_msl_m=hm, mission_id=mission_id)
        return items
    fr = None
    if frame is not None:
        m = _require_pymavlink()
        fr = int(getattr(m, _ALT_FRAME_NAMES[frame])) if isinstance(frame, str) else int(frame)
    return plan_to_mission_items_fallback(plan, ctx, cfg, frame=fr, mission_id=mission_id, **kw)


def plan_to_mission_items_fallback(plan: UAVPlan, ctx: Any, cfg: PlannerConfig | None = None, *,
                                   frame: int | None = None, mission_id: str = "", cam_trigger_dist_m: float = 0.0,
                                   min_spacing_m: float = 0.5, orbit_as_loiter: bool = True) -> MissionItems:
    """後備 UAVPlan → QuadPlane 任務項目（設計文件 §15.1 之對照；`hv_dccpp.ardupilot` 缺席時使用）。

    * seq 0 home（frame GLOBAL、NAV_WAYPOINT、current=1、alt = 起飛點 MSL）；
    * NAV_VTOL_TAKEOFF（alt = `transition_start_agl_m`、param4 yaw = TRANS_OUT 航跡）；DO_CHANGE_SPEED 巡航空速；
    * 固定翼段航點（TRANS_OUT 末點 = P_tc 起）→ NAV_WAYPOINT（param2 = accept_radius_m、alt 相對 home = alt_msl − home_msl）；
      `sensor_on` 變化處插 DO_SET_CAM_TRIGG_DIST（開：`cam_trigger_dist_m`>0 否則 1；關：0）；`airspeed_mps` 變化處插
      DO_CHANGE_SPEED；ORBIT（有 arc）→ 切入航點 + NAV_LOITER_TO_ALT（param1=1、param2=±R、alt=出口高）；
      HOLD（有 arc）→ NAV_LOITER_TURNS；首個 RTB 航點前插 DO_LAND_START；
    * NAV_VTOL_LAND（降落點 lat/lon、alt 0、param4 yaw = 進場航跡）。
    每項目附 phase／strip_id／dir／wp_index／s_frac（seqmap）。與前一 NAV 航點距離 < `min_spacing_m` 之航點合併。
    """
    m = _require_pymavlink()
    if not plan.waypoints:
        raise ValueError("plan 無航點")
    fr = int(m.MAV_FRAME_GLOBAL_RELATIVE_ALT if frame is None else frame)
    spec = ctx.specs[plan.cls] if hasattr(ctx, "specs") else None
    wps = plan.waypoints
    lat_h, lon_h, home_msl = _home_msl(plan, ctx)
    spans = _oper_spans(plan)
    span_end = {v[4]: k for k, v in spans.items()}
    out = MissionItems(mission_type=m.MAV_MISSION_TYPE_MISSION)

    def alt_of(w: Waypoint) -> float:
        if fr == m.MAV_FRAME_GLOBAL_TERRAIN_ALT:
            return float(w.agl_m)
        if fr == m.MAV_FRAME_GLOBAL:
            return float(w.alt_msl_m)
        return float(w.alt_msl_m - home_msl)

    def add(cmd: int, p1: float = 0.0, p2: float = 0.0, p3: float = 0.0, p4: float = 0.0, lat: float = 0.0,
            lon: float = 0.0, alt: float = 0.0, *, frame_: int | None = None, current: int = 0,
            w: Waypoint | None = None, wp_index: int | None = None, phase: str | None = None) -> MissionItemInt:
        it = MissionItemInt(len(out), fr if frame_ is None else int(frame_), int(cmd), current, 1, float(p1),
                            float(p2), float(p3), float(p4), float(lat), float(lon), float(alt),
                            int(m.MAV_MISSION_TYPE_MISSION))
        if w is not None:
            it.phase = w.phase.name if phase is None else phase
            it.strip_id = w.strip_id
            it.dir = w.dir
            it.wp_index = wp_index
            if w.phase is Phase.OPERATE and w.strip_id in spans:
                it.s_frac_end = _s_frac(np.array([w.x_m, w.y_m]), spans[w.strip_id])
            elif wp_index is not None and wp_index in span_end and cmd in _nav_cmds():
                sid_ = span_end[wp_index]
                it.phase, it.strip_id, it.dir, it.s_frac_end = Phase.OPERATE.name, sid_, spans[sid_][2], 1.0
        elif phase is not None:
            it.phase = phase
            it.wp_index = wp_index
        out.append(it)
        return it

    # home
    add(m.MAV_CMD_NAV_WAYPOINT, lat=lat_h, lon=lon_h, alt=home_msl, frame_=m.MAV_FRAME_GLOBAL, current=1)
    # 起飛
    i_tro = next((i for i, w in enumerate(wps) if w.phase is Phase.TRANS_OUT), None)
    yaw_to = float(wps[i_tro].track_deg) if i_tro is not None else float(wps[0].track_deg)
    h_ts = float(spec.transition_start_agl_m) if spec is not None else 60.0
    add(m.MAV_CMD_NAV_VTOL_TAKEOFF, p4=yaw_to, lat=lat_h, lon=lon_h, alt=h_ts, w=wps[0], wp_index=0,
        phase=Phase.TAKEOFF.name)
    v_cruise = float(spec.cruise_airspeed_mps) if spec is not None else float(max(w.airspeed_mps for w in wps))
    add(m.MAV_CMD_DO_CHANGE_SPEED, 0.0, v_cruise, -1.0, phase=Phase.TRANS_OUT.name)
    cur_speed = v_cruise
    sensor = False
    land_start_done = False
    last_xy: np.ndarray | None = None
    trig = float(cam_trigger_dist_m) if cam_trigger_dist_m > 0 else 1.0
    # 固定翼段：從 TRANS_OUT 之最後一點（P_tc）起
    i_ptc = i_tro
    if i_tro is not None:
        j = i_tro
        while j + 1 < len(wps) and wps[j + 1].phase is Phase.TRANS_OUT:
            j += 1
        i_ptc = j
    i_end = next((i for i, w in enumerate(wps) if w.phase in (Phase.TRANS_IN, Phase.LAND)), len(wps))
    i = i_ptc if i_ptc is not None else 0
    while i < i_end:
        w = wps[i]
        if w.phase not in FW_PHASES and w.phase is not Phase.TRANS_OUT:
            i += 1
            continue
        if w.phase is Phase.RTB and not land_start_done and i not in span_end:
            # 條帶末端（span_end）雖屬 RTB 首點，仍視為作業段終點；DO_LAND_START 置於其後之第一個 RTB 航點前
            add(m.MAV_CMD_DO_LAND_START, w=w, wp_index=i)
            land_start_done = True
        xy = np.array([w.x_m, w.y_m])
        dup = last_xy is not None and float(np.hypot(*(xy - last_xy))) < min_spacing_m
        if w.phase is Phase.ORBIT and w.arc is not None and orbit_as_loiter:
            # 切入點 + 一個 NAV_LOITER_TO_ALT（涵蓋整段 ORBIT 航點）
            if not dup:
                add(m.MAV_CMD_NAV_WAYPOINT, 0.0, w.accept_radius_m, 0.0, 0.0, w.lat_deg, w.lon_deg, alt_of(w), w=w,
                    wp_index=i)
            j = i
            while j + 1 < i_end and wps[j + 1].phase is Phase.ORBIT and wps[j + 1].arc is not None:
                j += 1
            cx, cy, R, turn = w.arc
            ll = ctx.frame.to_wgs84(np.array([[cx, cy, wps[j].z_m]]))[0]
            add(m.MAV_CMD_NAV_LOITER_TO_ALT, 1.0, float(turn) * float(R), 0.0, 1.0, float(ll[0]), float(ll[1]),
                alt_of(wps[j]), w=wps[j], wp_index=j)
            last_xy = np.array([wps[j].x_m, wps[j].y_m])
            i = j + 1
            continue
        if w.phase is Phase.HOLD and w.arc is not None:
            cx, cy, R, turn = w.arc
            ll = ctx.frame.to_wgs84(np.array([[cx, cy, w.z_m]]))[0]
            add(m.MAV_CMD_NAV_LOITER_TURNS, 1.0, 0.0, float(turn) * float(R), 1.0, float(ll[0]), float(ll[1]),
                alt_of(w), w=w, wp_index=i)
            last_xy = xy
            i += 1
            continue
        if not dup:
            add(m.MAV_CMD_NAV_WAYPOINT, 0.0, w.accept_radius_m, 0.0, 0.0, w.lat_deg, w.lon_deg, alt_of(w), w=w,
                wp_index=i)
            last_xy = xy
        else:
            # 合併：以較新的中繼資料覆寫（條帶端點 = 轉移起點）
            prev = out[-1]
            if prev.command == m.MAV_CMD_NAV_WAYPOINT and w.phase is not Phase.OPERATE and prev.phase == Phase.OPERATE.name:
                pass  # 保留 OPERATE 端點語意
        if bool(w.sensor_on) != sensor:
            sensor = bool(w.sensor_on)
            add(m.MAV_CMD_DO_SET_CAM_TRIGG_DIST, trig if sensor else 0.0, 0.0, 1.0 if sensor else 0.0, w=w, wp_index=None)
        if w.airspeed_mps > 0 and abs(float(w.airspeed_mps) - cur_speed) > 0.05:
            cur_speed = float(w.airspeed_mps)
            add(m.MAV_CMD_DO_CHANGE_SPEED, 0.0, cur_speed, -1.0, w=w, wp_index=None)
        i += 1
    if sensor:
        add(m.MAV_CMD_DO_SET_CAM_TRIGG_DIST, 0.0, 0.0, 0.0, phase=Phase.RTB.name)
    if not land_start_done:
        add(m.MAV_CMD_DO_LAND_START, phase=Phase.RTB.name)
    # 降落
    wl = wps[-1]
    i_dec = next((i for i in range(len(wps) - 1, -1, -1) if wps[i].phase is Phase.DECEL), None)
    yaw_ldg = float(wps[i_dec].track_deg) if i_dec is not None else float(wl.track_deg)
    add(m.MAV_CMD_NAV_VTOL_LAND, 0.0, 0.0, 0.0, yaw_ldg, wl.lat_deg, wl.lon_deg, 0.0, w=wl, wp_index=len(wps) - 1,
        phase=Phase.LAND.name)
    # s_frac_start 串接
    last_frac: dict[str, float] = {}
    for it in out:
        if it.phase == Phase.OPERATE.name and it.strip_id is not None and it.command in _nav_cmds():
            it.s_frac_start = last_frac.get(it.strip_id, 0.0)
            last_frac[it.strip_id] = it.s_frac_end
        elif it.strip_id is not None and it.phase == Phase.OPERATE.name:
            it.s_frac_start = it.s_frac_end = last_frac.get(it.strip_id, 0.0)
    mission_seq_map(out, plan, ctx, uav_id=plan.uav_id, home_msl_m=home_msl, mission_id=mission_id)
    return out


def rally_items(sites: Sequence[Site] | Any, ctx: Any, *, alt_rel_m: float = 100.0,
                frame: int | str | None = None) -> MissionItems:
    """備降點 → MAV_CMD_NAV_RALLY_POINT。`hv_dccpp.ardupilot.rally_items(inputs, ctx, alt_frame)` 存在則優先
    （home + 全部備降點、alt = 站地面 + H_tc；`sites` 可為 `MissionInputs` 或站點序列——後者只用於後備）；
    後備：每站 frame GLOBAL_RELATIVE_ALT、z = `alt_rel_m`。"""
    m = _require_pymavlink()
    ap = _ardupilot_module()
    if ap is not None and hasattr(ap, "rally_items"):
        inputs = sites if hasattr(sites, "home") else getattr(ctx, "inputs")
        raw = _call_compat(ap.rally_items, inputs, ctx, alt_frame=_alt_frame_name(frame))
        return MissionItems([as_mission_item(x, seq=i, mission_type=m.MAV_MISSION_TYPE_RALLY) for i, x in enumerate(raw)],
                            mission_type=m.MAV_MISSION_TYPE_RALLY)
    if hasattr(sites, "home"):
        sites = [sites.home, *sites.alternates]
    out = MissionItems(mission_type=m.MAV_MISSION_TYPE_RALLY)
    for i, s in enumerate(sites):
        out.append(MissionItemInt(i, m.MAV_FRAME_GLOBAL_RELATIVE_ALT, m.MAV_CMD_NAV_RALLY_POINT, 0, 1, 0.0, 0.0, 0.0, 0.0,
                                  float(s.lat_deg), float(s.lon_deg), float(alt_rel_m), m.MAV_MISSION_TYPE_RALLY))
    return out


def fence_items(plan: MissionPlan | UAVPlan | Sequence[UAVPlan], ctx: Any, cfg: PlannerConfig | None = None, *,
                cls: str | None = None, buffer_m: float | None = None) -> MissionItems:
    """圍籬項目。`hv_dccpp.ardupilot.fence_items(inputs, ctx, cls)` 存在則優先（`cls` None → 取 plan 首機構型）；
    後備：FENCE_RETURN_POINT(home) + inclusion（區域 ∪ 航點 ∪ 站點 之凸包 ⊕ buffer；5001，param1 = 頂點數）
    + 每個生效禁區 exclusion（5002）。"""
    m = _require_pymavlink()
    plans: list[UAVPlan]
    if isinstance(plan, MissionPlan):
        plans = list(plan.plans.values())
    elif isinstance(plan, UAVPlan):
        plans = [plan]
    else:
        plans = list(plan)
    cfg = cfg or getattr(ctx, "cfg", None) or PlannerConfig()
    ap = _ardupilot_module()
    if ap is not None and hasattr(ap, "fence_items"):
        c = cls or (plans[0].cls if plans else next(iter(getattr(ctx, "specs", {"A": None}))))
        same = [p for p in plans if p.cls == c]
        raw = _call_compat(ap.fence_items, getattr(ctx, "inputs"), ctx, c, plans=same or None)
        return MissionItems([as_mission_item(x, seq=i, mission_type=m.MAV_MISSION_TYPE_FENCE) for i, x in enumerate(raw)],
                            mission_type=m.MAV_MISSION_TYPE_FENCE)
    import shapely
    from shapely.geometry import MultiPoint
    buf = float(buffer_m) if buffer_m is not None else float(cfg.b_trk_m) * 2.0
    geoms: list[Any] = list(getattr(ctx, "areas_enu", {}).values())
    pts = [(w.x_m, w.y_m) for p in plans for w in p.waypoints]
    pts += [tuple(map(float, xy)) for xy, _z in getattr(ctx, "sites_enu", {}).values()]
    if pts:
        geoms.append(MultiPoint(pts))
    if not geoms:
        raise ValueError("fence_items: 無區域／航點可建圍籬")
    hull = shapely.unary_union(geoms).convex_hull.buffer(buf, quad_segs=4)
    ring = list(hull.exterior.coords)[:-1]
    out = MissionItems(mission_type=m.MAV_MISSION_TYPE_FENCE)
    home = getattr(ctx, "inputs").home
    out.append(MissionItemInt(0, m.MAV_FRAME_GLOBAL, m.MAV_CMD_NAV_FENCE_RETURN_POINT, 0, 1, 0.0, 0.0, 0.0, 0.0,
                              float(home.lat_deg), float(home.lon_deg), 0.0, m.MAV_MISSION_TYPE_FENCE))
    ll = ctx.frame.to_wgs84(np.column_stack([np.array(ring), np.zeros(len(ring))]))
    for k in range(len(ring)):
        out.append(MissionItemInt(len(out), m.MAV_FRAME_GLOBAL, m.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, 0, 1,
                                  float(len(ring)), 0.0, 0.0, 0.0, float(ll[k, 0]), float(ll[k, 1]), 0.0,
                                  m.MAV_MISSION_TYPE_FENCE))
    zones = getattr(ctx, "inputs").airspace.active_no_fly(getattr(ctx, "t_window", None)) \
        if getattr(cfg, "enforce_time_windows", True) else getattr(ctx, "inputs").airspace.no_fly
    for z in zones:
        ring_z = list(z.polygon_wgs84)
        for lat, lon in ring_z:
            out.append(MissionItemInt(len(out), m.MAV_FRAME_GLOBAL, m.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, 0, 1,
                                      float(len(ring_z)), 0.0, 0.0, 0.0, float(lat), float(lon), 0.0,
                                      m.MAV_MISSION_TYPE_FENCE))
    return out


def suggest_params(spec: Any, ctx: Any | None = None, cfg: PlannerConfig | None = None, *,
                   h_ceil_m: float | None = None, uav_plan: UAVPlan | None = None,
                   uav_id: str | None = None) -> dict[str, float]:
    """表 1 → ArduPilot 參數建議值（§15.2；只是建議，不自動寫入）。`ardupilot.recommended_params` 存在則優先
    （帶入該機 EnergyModel、pack 電壓、風速、h_ceil、失鏈策略、t_ll）；否則本模組之簡表。"""
    ap = _ardupilot_module()
    cfg = cfg or getattr(ctx, "cfg", None) or PlannerConfig()
    if ap is not None and hasattr(ap, "recommended_params"):
        model = None
        w_ref = 0.0
        policy = "RTB"
        t_ll = 60.0
        pack_v = None
        if ctx is not None:
            if uav_id is not None:
                model = (getattr(ctx, "models", {}) or {}).get(uav_id)
                try:
                    uav = ctx.uav(uav_id)
                    pack_v = uav.sortie.pack_voltage_nom_v
                    policy = ctx.inputs.c2.policy_for(uav_id, spec.cls)
                except Exception:  # noqa: BLE001 - 無此機
                    pass
            if model is None:
                model = (getattr(ctx, "models_rep", {}) or {}).get(spec.cls)
            wf = getattr(ctx, "wind", None)
            w_ref = float(wf.max_speed()) if wf is not None else 0.0
            inp = getattr(ctx, "inputs", None)
            t_ll = float(inp.c2.t_ll_s) if inp is not None else 60.0
            if h_ceil_m is None:
                v = (getattr(ctx, "h_ceil_m", {}) or {}).get(spec.cls)
                h_ceil_m = float(v) if v is not None and math.isfinite(float(v)) else None
        home_msl = None
        if uav_plan is not None and ctx is not None:
            try:
                home_msl = _home_msl(uav_plan, ctx)[2]
            except Exception:  # noqa: BLE001 - 無 frame／站點
                home_msl = None
        kinds = sorted({k for k in ((getattr(s, "kind", None) for (c, _r), s in (getattr(ctx, "sensors", {}) or {}).items()
                                     if c == spec.cls)) if k}) if ctx is not None else []
        return dict(_call_compat(ap.recommended_params, spec, model, cfg, uav_plan, pack_voltage_v=pack_v,
                                 wind_speed_mps=w_ref, h_ceil_agl_m=h_ceil_m, lost_link_policy=policy, t_ll_s=t_ll,
                                 home_msl_m=home_msl, sensor_kinds=kinds))
    tc = spec.transition_cmd()
    p: dict[str, float] = {
        "Q_ENABLE": 1.0,
        "AIRSPEED_CRUISE": float(spec.cruise_airspeed_mps),
        "AIRSPEED_MIN": float(spec.v_min_mission()),
        "AIRSPEED_STALL": float(spec.stall_airspeed_mps),
        "Q_ASSIST_SPEED": round(1.2 * float(spec.stall_airspeed_mps), 2),
        "TECS_CLMB_MAX": float(spec.max_fw_climb_rate_mps),
        "TECS_SINK_MAX": float(spec.max_fw_descent_rate_mps),
        "ROLL_LIMIT_DEG": float(spec.max_bank_angle_deg),
        "WP_LOITER_RAD": float(spec.turn_radius_planning_m),
        "RTL_RADIUS": float(spec.turn_radius_planning_m),
        "WP_RADIUS": float(min(spec.turn_radius_planning_m / 2.0, 30.0)),
        "Q_TRANSITION_MS": float(int(spec.t_tr_out_s * 1000)),
    }
    if spec.max_airspeed_mps is not None:
        p["AIRSPEED_MAX"] = float(spec.max_airspeed_mps)
    hc = h_ceil_m
    if hc is None and ctx is not None:
        hcm = getattr(ctx, "h_ceil_m", {}) or {}
        v = hcm.get(spec.cls)
        hc = float(v) if v is not None and math.isfinite(float(v)) else None
    if hc is not None:
        p["FENCE_ALT_MAX"] = float(hc)
        p["FENCE_ENABLE"] = 1.0
    p["FENCE_ALT_MIN"] = float(spec.min_fw_operation_agl_m)
    return p


def write_param_file(params: Mapping[str, float], path: str | os.PathLike[str], header: Sequence[str] = ()) -> str:
    lines = [f"# {h}" for h in header] + [f"{k},{_fmt(v)}" for k, v in params.items()]
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")
    return str(path)


def read_param_file(path: str | os.PathLike[str]) -> dict[str, float]:
    out: dict[str, float] = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        k, _, v = s.replace("\t", ",").partition(",")
        out[k.strip()] = float(v.strip())
    return out


# ---------------------------------------------------------------------------
# 遙測 → 規劃器 Telemetry、事件偵測
# ---------------------------------------------------------------------------
def _plan_energy_at(plan: UAVPlan, t_s: float) -> float:
    wps = plan.waypoints
    if not wps:
        return 0.0
    ts = np.array([w.t_s for w in wps], dtype=float)
    j = int(np.clip(np.searchsorted(ts, float(t_s), side="right") - 1, 0, ts.size - 1))
    a = wps[j]
    if j >= len(wps) - 1 or wps[j + 1].t_s <= a.t_s + 1e-9:
        return float(a.energy_remaining_wh)
    b = wps[j + 1]
    f = min(1.0, max(0.0, (float(t_s) - a.t_s) / (b.t_s - a.t_s)))
    return float(a.energy_remaining_wh + f * (b.energy_remaining_wh - a.energy_remaining_wh))


def _plan_phase_at(plan: UAVPlan, t_s: float) -> Phase | None:
    wps = plan.waypoints
    if not wps:
        return None
    ts = np.array([w.t_s for w in wps], dtype=float)
    j = int(np.clip(np.searchsorted(ts, float(t_s), side="right") - 1, 0, ts.size - 1))
    return wps[j].phase


def _uav_plan(plan: UAVPlan | MissionPlan, uav_id: str | None) -> UAVPlan:
    if isinstance(plan, MissionPlan):
        if uav_id is None or uav_id not in plan.plans:
            raise KeyError(f"MissionPlan 無 {uav_id!r} 之 UAVPlan")
        return plan.plans[uav_id]
    return plan


def _nominal_wh(ctx: Any, uav_id: str, plan: UAVPlan | None) -> float | None:
    model = (getattr(ctx, "models", {}) or {}).get(uav_id)
    if model is not None:
        if getattr(model, "base_minute_mode", False):
            return None
        if model.nominal_wh is not None:
            return float(model.nominal_wh)
        return float(model.available_wh)
    if plan is not None and plan.waypoints:
        return float(plan.waypoints[0].energy_remaining_wh)
    return None


def to_planner_telemetry(tel: VehicleTelemetry, plan: UAVPlan | MissionPlan, ctx: Any, uav_id: str, *,
                         seqmap: MissionSeqMap | None = None, t_s: float | None = None) -> Any:
    """:class:`VehicleTelemetry` → :class:`hv_dccpp.dynamic.Telemetry`（§15.5.2）。

    * ENU：``ctx.frame.to_enu(lat, lon, alt_msl)``（三維，D42）；θ = 航跡（地速 ≥ 3 m/s）否則機首方位；
    * 能量：`battery_wh_remaining`（Wh）優先；否則 `battery_pct/100 × E_nom`（E_nom = 該機 EnergyModel.nominal_wh →
      available_wh）；基準分鐘模式或皆無 → None（dynamic 以計畫能量 + κ 推估）；
    * 現行條帶：`seqmap`（或 `plan` 之 ``seqmap`` 屬性）以 `mission_seq` 查 :meth:`MissionSeqMap.strip_at`；OPERATE 中 →
      `strip_progress` = 位置在條帶（首 OPER 航點 → 條帶後首航點）上的投影比例；條帶之後 → 前一條帶、進度 1；
      無 seqmap → 以計畫時間軸估計（`t_s` 落在哪條帶）。
    """
    from .dynamic import Telemetry
    up = _uav_plan(plan, uav_id)
    enu = ctx.frame.to_enu(float(tel.lat_deg), float(tel.lon_deg), float(tel.alt_msl_m))[0]
    x, y, z = float(enu[0]), float(enu[1]), float(enu[2])
    if tel.track_deg is not None and tel.groundspeed_mps >= 3.0:
        theta = math.radians(float(tel.track_deg))
    else:
        theta = math.radians(float(tel.heading_deg))
    theta %= 2.0 * math.pi
    spec = ctx.specs[up.cls] if hasattr(ctx, "specs") else None
    if tel.airspeed_mps is not None and tel.airspeed_mps > 0:
        va = float(tel.airspeed_mps)
    elif tel.groundspeed_mps > 3.0 and spec is not None:
        va = float(spec.cruise_airspeed_mps)
    else:
        va = float(spec.cruise_airspeed_mps) if spec is not None else float(tel.groundspeed_mps)
    e_tel: float | None = None
    if tel.battery_wh_remaining is not None:
        e_tel = float(tel.battery_wh_remaining)
    elif tel.battery_pct is not None:
        nom = _nominal_wh(ctx, uav_id, up)
        e_tel = None if nom is None else float(tel.battery_pct) / 100.0 * nom
    tt = float(tel.t_s if t_s is None else t_s)
    sm = seqmap if seqmap is not None else getattr(up, "seqmap", None)
    strip_id: str | None = None
    progress = 0.0
    spans = _oper_spans(up)
    if isinstance(sm, MissionSeqMap) and sm.items:
        sid, inside = sm.strip_at(int(tel.mission_seq))
        if sid is not None:
            strip_id = sid
            if inside and sid in spans:
                progress = _s_frac(np.array([x, y]), spans[sid])
            else:
                progress = 1.0 if not inside else 0.0
    else:
        for sid, (p0, p1, d, i0, i_end) in spans.items():
            t0, t1 = up.waypoints[i0].t_s, up.waypoints[i_end].t_s
            if t0 <= tt <= t1:
                strip_id = sid
                progress = _s_frac(np.array([x, y]), spans[sid])
                break
    return Telemetry(uav_id=uav_id, t_s=tt, x_m=x, y_m=y, z_m=z, theta_rad=theta, v_air_mps=va, e_tel_wh=e_tel,
                     current_strip_id=strip_id, strip_progress=float(progress), link_ok=bool(tel.link_ok))


@dataclass(slots=True)
class EnergyEventState:
    """:func:`detect_events` 之 ENERGY_UPDATE 遲滯／防抖狀態（每機一份；由 :class:`FleetLink` 保存）。

    規格 §12.4 的判準 δ = (E_pred − E_tel)/E_pred > δ₁ 是「一次事件」的觸發條件，不是逐筆遙測的布林旗標：
    載具電量模型與計畫恆有系統性偏差，直接逐筆比對會在每一次輪詢都送出一個 EnergyUpdate（實測 mock 閉迴路
    330 次／次示範）。本狀態提供三道閘（D49）：

    1. **遲滯（hysteresis）**：進入門檻 |Δ| > δ₁·E_nom；離開門檻 |Δ| < `release_frac`·δ₁·E_nom（預設 0.6），
       兩者之間維持原狀態，避免在門檻附近抖動。
    2. **持續（dwell）**：超標須持續 `dwell_s` 秒且 ≥ `min_samples` 筆才成立（單筆突波不觸發）。
    3. **重新觸發（re-arm）**：已送出後，除非偏差再變化 δ₁·E_nom（`Δ` 相對上次事件之變化量）或距上次事件
       ≥ `min_interval_s` 秒且仍超標，否則不重送。
    """
    t_first_exceed_s: float | None = None
    n_exceed: int = 0
    t_last_event_s: float | None = None
    dev_at_event_wh: float | None = None
    n_events: int = 0
    n_suppressed: int = 0

    def reset_run(self) -> None:
        self.t_first_exceed_s = None
        self.n_exceed = 0

    def update(self, t_s: float, dev_wh: float, thr_wh: float, *, dwell_s: float, min_samples: int,
               min_interval_s: float, release_frac: float) -> bool:
        """回傳「本筆是否應送出 EnergyUpdate」；`dev_wh` = E_tel − E_plan（帶號），`thr_wh` = δ₁·E_nom。"""
        a = abs(float(dev_wh))
        if a < max(0.0, float(release_frac)) * thr_wh:
            self.reset_run()
            return False
        if a <= thr_wh:                                    # 遲滯帶內：不進不出
            return False
        if self.t_first_exceed_s is None:
            self.t_first_exceed_s = float(t_s)
        self.n_exceed += 1
        if float(t_s) - self.t_first_exceed_s < float(dwell_s) or self.n_exceed < int(min_samples):
            self.n_suppressed += 1
            return False
        if self.t_last_event_s is not None:
            grew = self.dev_at_event_wh is None or abs(float(dev_wh) - float(self.dev_at_event_wh)) > thr_wh
            stale = float(t_s) - self.t_last_event_s >= float(min_interval_s)
            if not (grew or stale):
                self.n_suppressed += 1
                return False
        self.t_last_event_s = float(t_s)
        self.dev_at_event_wh = float(dev_wh)
        self.n_events += 1
        return True


ENERGY_DWELL_S: float = 5.0
ENERGY_MIN_SAMPLES: int = 2
ENERGY_MIN_INTERVAL_S: float = 300.0
ENERGY_RELEASE_FRAC: float = 0.6


def detect_events(tel: Any, plan: UAVPlan | MissionPlan, ctx: Any, cfg: PlannerConfig, last_tel: Any | None = None, *,
                  uav_id: str | None = None, seqmap: MissionSeqMap | None = None, planner_tel: Any | None = None,
                  last_wind_update_t_s: float | None = None, wind_min_interval_s: float = 30.0,
                  energy_state: EnergyEventState | None = None, energy_dwell_s: float = ENERGY_DWELL_S,
                  energy_min_samples: int = ENERGY_MIN_SAMPLES,
                  energy_min_interval_s: float = ENERGY_MIN_INTERVAL_S,
                  energy_release_frac: float = ENERGY_RELEASE_FRAC) -> list[Any]:
    """§15.5.2 事件規則 → `dynamic` 事件列表。

    `tel`／`last_tel` 可為 :class:`VehicleTelemetry`（完整規則）或 `dynamic.Telemetry`（僅鏈路與能量規則）。
    * LinkLoss：`link_ok` True→False（首筆即 False 亦算）；LinkRestored：False→True（附本筆規劃器 Telemetry）；
    * EnergyUpdate：`e_tel_wh` 與計畫能量 E_plan(t) 偏差 > `cfg.delta_1 × E_nom`（E_nom = 該機標稱，缺省首航點能量）。
      給 `energy_state`（:class:`EnergyEventState`，:class:`FleetLink` 每機保存一份）時套用遲滯／持續／重新觸發三閘，
      同一段偏差只送一次；**不給時為無狀態單筆判斷**（每筆超標都送，僅適用於一次性檢查）；
    * WindUpdate：WIND 與 `ctx.wind` 於該高度之估計 |ΔW| > 1 m/s 或 |Δβ| > 15°，且距 `last_wind_update_t_s` ≥ 30 s →
      以該層改寫後之 `WindLayer` 元組（其餘層不變；無層時新增 AGL 0–inf 單層）；
    * UAVExit(reason='landed_unexpected')：`landed_state == ON_GROUND` 而計畫 phase 在空中（非 TAKEOFF/LAND/TRANS_*），
      且前一筆為在空中。
    """
    from . import dynamic as dyn
    up = _uav_plan(plan, uav_id)
    uid = uav_id or up.uav_id
    if isinstance(tel, VehicleTelemetry):
        ptel = planner_tel if planner_tel is not None else to_planner_telemetry(tel, up, ctx, uid, seqmap=seqmap)
    else:
        ptel = tel
    events: list[Any] = []
    t = float(ptel.t_s)
    last_ok = bool(last_tel.link_ok) if last_tel is not None else True
    if last_ok and not ptel.link_ok:
        events.append(dyn.LinkLoss(uid, t))
    elif (not last_ok) and ptel.link_ok:
        events.append(dyn.LinkRestored(uid, t, ptel))
    if ptel.e_tel_wh is not None and up.waypoints:
        e_plan = _plan_energy_at(up, t)
        nom = _nominal_wh(ctx, uid, up)
        if nom is not None and nom > 0:
            dev = float(ptel.e_tel_wh) - e_plan            # < 0 → 比計畫少（虧損）
            thr = float(cfg.delta_1) * float(nom)
            if energy_state is None:
                fire = abs(dev) > thr
            else:
                fire = energy_state.update(t, dev, thr, dwell_s=energy_dwell_s, min_samples=energy_min_samples,
                                           min_interval_s=energy_min_interval_s, release_frac=energy_release_frac)
            if fire:
                events.append(dyn.EnergyUpdate(uid, t, float(ptel.e_tel_wh)))
    if isinstance(tel, VehicleTelemetry):
        m = _require_pymavlink()
        # 風
        if tel.wind_speed_mps is not None and tel.wind_dir_from_deg is not None and \
                (last_wind_update_t_s is None or t - last_wind_update_t_s >= wind_min_interval_s):
            wf = getattr(ctx, "wind", None)
            z = float(ptel.z_m)
            differs = True
            if wf is not None:
                w_old = float(wf.speed_at(z))
                vec = np.asarray(wf.wind_vec(z)).reshape(-1)[:2]
                if w_old > 1e-6:
                    beta_old = math.degrees(math.atan2(-vec[0], -vec[1])) % 360.0
                    dbeta = abs((float(tel.wind_dir_from_deg) - beta_old + 180.0) % 360.0 - 180.0)
                else:
                    dbeta = 0.0
                differs = abs(float(tel.wind_speed_mps) - w_old) > 1.0 or (w_old > 1e-6 and dbeta > 15.0) \
                    or (w_old <= 1e-6 and float(tel.wind_speed_mps) > 1.0)
            if differs:
                events.append(dyn.WindUpdate(_updated_layers(ctx, z, float(tel.wind_speed_mps),
                                                             float(tel.wind_dir_from_deg)), t, None))
        # 意外落地
        if int(tel.landed_state) == int(m.MAV_LANDED_STATE_ON_GROUND):
            ph = _plan_phase_at(up, t)
            airborne_phase = ph is not None and ph not in (Phase.TAKEOFF, Phase.LAND, Phase.TRANS_IN, Phase.TRANS_OUT)
            was_air = isinstance(last_tel, VehicleTelemetry) and int(last_tel.landed_state) in (
                int(m.MAV_LANDED_STATE_IN_AIR), int(m.MAV_LANDED_STATE_TAKEOFF))
            if airborne_phase and was_air:
                events.append(dyn.UAVExit(uid, t, "landed_unexpected"))
    return events


def _updated_layers(ctx: Any, z_m: float, speed: float, dir_from_deg: float) -> tuple[WindLayer, ...]:
    """以觀測改寫含 z 之風層（層以 ENU z 對應：`WindField.layer_index`）；無層 → 單層 AGL 0–inf。"""
    wf = getattr(ctx, "wind", None)
    layers = list(getattr(wf, "layers", []) or [])
    inputs = getattr(ctx, "inputs", None)
    raw = list(getattr(inputs, "wind", ())) if inputs is not None else []
    if not raw:
        return (WindLayer(0.0, math.inf, speed, dir_from_deg, 0.0, "agl"),)
    idx = -1
    if wf is not None and layers:
        try:
            idx = int(np.asarray(wf.layer_index(z_m)).reshape(-1)[0])
        except Exception:  # pragma: no cover - 防禦
            idx = -1
    out: list[WindLayer] = []
    for k, lay in enumerate(raw):
        # WindField 依 ENU 排序，raw 依輸入順序；以物件身分對應
        target = (idx >= 0 and idx < len(layers) and layers[idx] is lay)
        if not target and idx < 0 and k == 0:
            target = True
        out.append(replace(lay, speed_mps=speed, dir_from_deg=dir_from_deg) if target else lay)
    if idx >= 0 and not any(layers[idx] is lay for lay in raw):
        out = [replace(lay, speed_mps=speed, dir_from_deg=dir_from_deg) if k == 0 else lay for k, lay in enumerate(raw)]
    return tuple(out)


# ---------------------------------------------------------------------------
# 上傳結果
# ---------------------------------------------------------------------------
@dataclass(slots=True)
class UploadResult:
    n_items: int
    ack_type: int
    mission_type: int
    elapsed_s: float
    retries_used: int = 0

    @property
    def ok(self) -> bool:
        return HAVE_PYMAVLINK and self.ack_type == _mav.MAV_MISSION_ACCEPTED


@dataclass(slots=True)
class PlanUploadReport:
    uav_id: str
    mission: UploadResult
    rally: UploadResult | None
    fence: UploadResult | None
    params: dict[str, bool] | None
    seqmap: MissionSeqMap
    items: MissionItems
    files: list[str] = field(default_factory=list)


@dataclass(slots=True)
class ReuploadResult:
    uav_id: str
    upload: UploadResult
    seqmap: MissionSeqMap
    items: MissionItems
    switch_seq: int | None
    set_current_ok: bool
    files: list[str] = field(default_factory=list)


# ---------------------------------------------------------------------------
# VehicleLink
# ---------------------------------------------------------------------------
_TELEMETRY_TYPES = ("GLOBAL_POSITION_INT", "VFR_HUD", "SYS_STATUS", "BATTERY_STATUS", "WIND", "EXTENDED_SYS_STATE",
                    "MISSION_CURRENT", "HEARTBEAT", "MISSION_ITEM_REACHED", "STATUSTEXT", "SYSTEM_TIME")


class VehicleLink:
    """單機 MAVLink 連線（pymavlink）。

    參數
    ----
    conn_str        pymavlink 連線字串（`udpout:127.0.0.1:14550`、`udpin:0.0.0.0:14550`、`tcp:127.0.0.1:5760`、`com5`）。
    sysid           目標 sysid（None → 第一個非 GCS 心跳之來源）。
    timeout_s       預設等待逾時。
    source_system   本端 sysid（GCS 慣例 255）。
    t0_epoch_s      `t_s` 基準（`time_source='wall'`；None → 建構時刻）。
    time_source     'wall'：t_s = time.time() − t0_epoch_s；'boot'：t_s = time_boot_ms/1000 − boot_offset_s。
    hb_timeout_s    心跳年齡 > 此值 → `link_ok=False`（§15.5.2 t_hb = 5 s）。
    """

    def __init__(self, conn_str: str, sysid: int | None = None, timeout_s: float = 10.0, source_system: int = 255, *,
                 source_component: int | None = None, baud: int = 57600, t0_epoch_s: float | None = None,
                 time_source: str = "wall", boot_offset_s: float = 0.0, hb_timeout_s: float = 5.0,
                 uav_id: str | None = None, autoconnect: bool = True) -> None:
        m = _require_pymavlink()
        self.conn_str = conn_str
        self.sysid = sysid
        self.timeout_s = float(timeout_s)
        self.source_system = int(source_system)
        self.source_component = int(m.MAV_COMP_ID_MISSIONPLANNER if source_component is None else source_component)
        self.baud = int(baud)
        self.t0_epoch_s = float(time.time() if t0_epoch_s is None else t0_epoch_s)
        self.time_source = time_source
        self.boot_offset_s = float(boot_offset_s)
        self.hb_timeout_s = float(hb_timeout_s)
        self.uav_id = uav_id
        self.nominal_wh: float | None = None   # 起飛時標稱能量（energy_consumed → Wh 換算用）
        self.conn: Any = None
        self.target_system: int = 0
        self.target_component: int = 0
        self.last_heartbeat_epoch_s: float | None = None
        self.last_heartbeat_msg: Any = None
        self._latest: dict[str, Any] = {}
        self._latest_t: dict[str, float] = {}
        self._reached_seq: int | None = None
        self._last_pos: tuple[float, float, float] | None = None   # (t, lat, lon) 供航跡／地速推估
        self._track_deg: float | None = None
        self._gs_est: float | None = None
        self.statustexts: list[tuple[float, int, str]] = []
        self.seqmap: MissionSeqMap | None = None
        self.last_items: MissionItems | None = None
        if autoconnect:
            self.connect()

    # -- 連線 --------------------------------------------------------------
    def connect(self) -> None:
        self.conn = _mavutil.mavlink_connection(self.conn_str, baud=self.baud, source_system=self.source_system,
                                                source_component=self.source_component)
        _disable_windows_udp_reset(getattr(self.conn, "port", None))

    def close(self) -> None:
        if self.conn is not None:
            try:
                self.conn.close()
            except Exception:  # pragma: no cover
                pass
            self.conn = None

    def __enter__(self) -> "VehicleLink":
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()

    @property
    def mav(self) -> Any:
        return self.conn.mav

    def send_heartbeat(self) -> None:
        m = _mav
        self.conn.mav.heartbeat_send(m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def heartbeat_age_s(self, now: float | None = None) -> float:
        if self.last_heartbeat_epoch_s is None:
            return math.inf
        return (time.time() if now is None else now) - self.last_heartbeat_epoch_s

    @property
    def link_ok(self) -> bool:
        return self.heartbeat_age_s() <= self.hb_timeout_s

    def _t_s(self, msg: Any | None = None) -> float:
        if self.time_source == "boot" and msg is not None and hasattr(msg, "time_boot_ms"):
            return float(msg.time_boot_ms) / 1000.0 - self.boot_offset_s
        if self.time_source == "boot":
            gp = self._latest.get("GLOBAL_POSITION_INT")
            if gp is not None:
                return float(gp.time_boot_ms) / 1000.0 - self.boot_offset_s
        return time.time() - self.t0_epoch_s

    # -- 收訊 --------------------------------------------------------------
    def _recv(self, timeout_s: float) -> Any | None:
        """收一筆訊息（任何型別）並更新內部快取；逾時 None。"""
        try:
            msg = self.conn.recv_match(blocking=True, timeout=max(0.0, float(timeout_s)))
        except (ConnectionResetError, OSError):
            return None
        if msg is None:
            return None
        self._cache(msg)
        return msg

    def _wait_for(self, types: Sequence[str], timeout_s: float, pred: Callable[[Any], bool] | None = None) -> Any | None:
        """等待符合型別（與 pred）之訊息；其他訊息照常快取。"""
        deadline = time.time() + float(timeout_s)
        tset = set(types)
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                return None
            msg = self._recv(min(remaining, 0.5))
            if msg is None:
                continue
            t = msg.get_type()
            if t in tset and (self.target_system == 0 or msg.get_srcSystem() == self.target_system
                              or t in ("MISSION_ACK",)) and (pred is None or pred(msg)):
                return msg

    def drain(self, max_s: float = 0.0) -> int:
        """非阻塞讀完緩衝（更新快取）；回收到的訊息數。"""
        n = 0
        t_end = time.time() + max_s
        while True:
            try:
                msg = self.conn.recv_match(blocking=False)
            except (ConnectionResetError, OSError):
                msg = None
            if msg is None:
                if time.time() < t_end:
                    time.sleep(0.005)
                    continue
                break
            self._cache(msg)
            n += 1
        return n

    def _cache(self, msg: Any) -> None:
        """更新心跳／目標 sysid／最新訊息快取／航跡推估。"""
        t = msg.get_type()
        if t == "BAD_DATA":
            return
        src = msg.get_srcSystem()
        if t == "HEARTBEAT" and msg.type != _mav.MAV_TYPE_GCS and src != self.source_system \
                and (self.sysid is None or src == self.sysid) and (self.target_system == 0 or src == self.target_system):
            self.target_system = src
            self.target_component = msg.get_srcComponent()
            self.conn.target_system = src
            self.conn.target_component = self.target_component
            self.last_heartbeat_epoch_s = time.time()
            self.last_heartbeat_msg = msg
        if self.target_system and src != self.target_system:
            return
        now = time.time()
        if t in _TELEMETRY_TYPES:
            self._latest[t] = msg
            self._latest_t[t] = now
        if t == "GLOBAL_POSITION_INT":
            lat, lon = msg.lat / 1e7, msg.lon / 1e7
            tb = float(msg.time_boot_ms) / 1000.0 if self.time_source == "boot" else now
            if self._last_pos is not None:
                t_prev, la0, lo0 = self._last_pos
                dy = (lat - la0) * 111_320.0
                dx = (lon - lo0) * 111_320.0 * math.cos(math.radians(lat))
                d = math.hypot(dx, dy)
                if d > 2.0:
                    self._track_deg = math.degrees(math.atan2(dx, dy)) % 360.0
                if tb - t_prev > 1e-3:
                    self._gs_est = d / (tb - t_prev)
            self._last_pos = (tb, lat, lon)
        elif t == "MISSION_ITEM_REACHED":
            self._reached_seq = int(msg.seq)
        elif t == "STATUSTEXT":
            txt = msg.text if isinstance(msg.text, str) else bytes(msg.text).decode(errors="replace")
            self.statustexts.append((now, int(msg.severity), txt.rstrip("\x00")))

    def wait_heartbeat(self, timeout_s: float | None = None) -> Any:
        """送本端 GCS 心跳（讓 udpout 對端得知位址）並等待載具心跳；設定 target_system/component。"""
        to = self.timeout_s if timeout_s is None else float(timeout_s)
        deadline = time.time() + to
        last_send = 0.0
        while time.time() < deadline:
            if time.time() - last_send >= 1.0:
                self.send_heartbeat()
                last_send = time.time()
            msg = self._recv(min(0.5, max(0.0, deadline - time.time())))
            if msg is not None and msg.get_type() == "HEARTBEAT" and self.target_system \
                    and msg.get_srcSystem() == self.target_system:
                return msg
        raise MavlinkTimeoutError(f"{self.conn_str}: {to} s 內無載具心跳")

    # -- 任務協定 ----------------------------------------------------------
    def _send_item(self, it: MissionItemInt, mission_type: int) -> None:
        self.conn.mav.mission_item_int_send(
            self.target_system, self.target_component, int(it.seq), int(it.frame), int(it.command), int(it.current),
            int(it.autocontinue), float(it.param1), float(it.param2), float(it.param3), float(it.param4),
            int(round(it.x * 1e7)), int(round(it.y * 1e7)), float(it.z), int(mission_type))

    def upload_mission(self, items: Sequence[Any], mission_type: int = 0, *, timeout_s: float | None = None,
                       retries: int = 3, step_timeout_s: float = 2.0) -> UploadResult:
        """MISSION_COUNT → (MISSION_REQUEST_INT|MISSION_REQUEST → MISSION_ITEM_INT)× n → MISSION_ACK。

        每步逾時 `step_timeout_s` 重送（COUNT 或最後一筆項目），累計超過 `retries` 次或總逾時 → MavlinkTimeoutError；
        ACK ≠ MAV_MISSION_ACCEPTED → MavlinkUploadError。回傳 :class:`UploadResult`。
        """
        m = _mav
        mt = int(mission_type)
        its = [as_mission_item(x, seq=i, mission_type=mt) for i, x in enumerate(items)]
        if mt == m.MAV_MISSION_TYPE_MISSION and its:
            its[0].current = 1
        to = self.timeout_s * 3 if timeout_s is None else float(timeout_s)
        t0 = time.time()
        deadline = t0 + to
        n_retry = 0
        self.conn.mav.mission_count_send(self.target_system, self.target_component, len(its), mt)
        last_seq: int | None = None
        while time.time() < deadline:
            msg = self._wait_for(("MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"),
                                 min(step_timeout_s, deadline - time.time()),
                                 lambda mm: int(getattr(mm, "mission_type", mt)) == mt)
            if msg is None:
                n_retry += 1
                if n_retry > retries:
                    raise MavlinkTimeoutError(f"upload_mission(type={mt}): 對端無回應（{n_retry - 1} 次重送後）")
                if last_seq is None:
                    self.conn.mav.mission_count_send(self.target_system, self.target_component, len(its), mt)
                else:
                    self._send_item(its[last_seq], mt)
                continue
            if msg.get_type() == "MISSION_ACK":
                ack = int(msg.type)
                if ack != m.MAV_MISSION_ACCEPTED:
                    raise MavlinkUploadError(f"upload_mission(type={mt}): MISSION_ACK={ack}（{_ack_name(ack)}）")
                if last_seq is None and its:
                    # 尚未送任何項目即 ACK（例：對端拒絕 count）→ 視為錯誤
                    raise MavlinkUploadError(f"upload_mission(type={mt}): 未請求項目即 ACK")
                return UploadResult(len(its), ack, mt, time.time() - t0, n_retry)
            seq = int(msg.seq)
            if not (0 <= seq < len(its)):
                raise MavlinkUploadError(f"upload_mission(type={mt}): 對端請求 seq {seq} 超出 0..{len(its) - 1}")
            self._send_item(its[seq], mt)
            last_seq = seq
        raise MavlinkTimeoutError(f"upload_mission(type={mt}): {to} s 內未完成")

    def upload_rally(self, items: Sequence[Any], **kw: Any) -> UploadResult:
        return self.upload_mission(items, _mav.MAV_MISSION_TYPE_RALLY, **kw)

    def upload_fence(self, items: Sequence[Any], **kw: Any) -> UploadResult:
        return self.upload_mission(items, _mav.MAV_MISSION_TYPE_FENCE, **kw)

    def download_mission(self, mission_type: int = 0, *, timeout_s: float | None = None, retries: int = 3,
                         step_timeout_s: float = 2.0) -> MissionItems:
        """MISSION_REQUEST_LIST → MISSION_COUNT → 逐 seq MISSION_REQUEST_INT → MISSION_ITEM_INT；結束送 MISSION_ACK。"""
        m = _mav
        mt = int(mission_type)
        to = self.timeout_s * 3 if timeout_s is None else float(timeout_s)
        deadline = time.time() + to
        count: int | None = None
        for attempt in range(retries + 1):
            self.conn.mav.mission_request_list_send(self.target_system, self.target_component, mt)
            msg = self._wait_for(("MISSION_COUNT",), min(step_timeout_s, deadline - time.time()),
                                 lambda mm: int(getattr(mm, "mission_type", mt)) == mt)
            if msg is not None:
                count = int(msg.count)
                break
        if count is None:
            raise MavlinkTimeoutError(f"download_mission(type={mt}): 無 MISSION_COUNT")
        out = MissionItems(mission_type=mt)
        for seq in range(count):
            got = None
            for attempt in range(retries + 1):
                self.conn.mav.mission_request_int_send(self.target_system, self.target_component, seq, mt)
                got = self._wait_for(("MISSION_ITEM_INT", "MISSION_ITEM"), min(step_timeout_s, deadline - time.time()),
                                     lambda mm, s=seq: int(mm.seq) == s and int(getattr(mm, "mission_type", mt)) == mt)
                if got is not None:
                    break
            if got is None:
                raise MavlinkTimeoutError(f"download_mission(type={mt}): seq {seq} 無回應")
            it = MissionItemInt.from_msg(got)
            it.mission_type = mt
            out.append(it)
        self.conn.mav.mission_ack_send(self.target_system, self.target_component, m.MAV_MISSION_ACCEPTED, mt)
        return out

    def clear_mission(self, mission_type: int = 0, *, timeout_s: float | None = None) -> bool:
        """MISSION_CLEAR_ALL → MISSION_ACK；回 ACK==ACCEPTED。"""
        m = _mav
        mt = int(mission_type)
        self.conn.mav.mission_clear_all_send(self.target_system, self.target_component, mt)
        msg = self._wait_for(("MISSION_ACK",), self.timeout_s if timeout_s is None else float(timeout_s))
        return msg is not None and int(msg.type) == m.MAV_MISSION_ACCEPTED

    # -- 參數 --------------------------------------------------------------
    @staticmethod
    def _param_id(msg: Any) -> str:
        pid = msg.param_id
        if isinstance(pid, bytes):
            pid = pid.decode(errors="replace")
        return str(pid).rstrip("\x00")

    def set_params(self, params: Mapping[str, float], *, timeout_s: float = 5.0, retries: int = 2,
                   tol: float = 1e-4) -> dict[str, bool]:
        """逐項 PARAM_SET 並以 PARAM_VALUE 回讀確認；回 `{name: ok}`。"""
        m = _mav
        out: dict[str, bool] = {}
        for name, value in params.items():
            ok = False
            for attempt in range(retries + 1):
                self.conn.mav.param_set_send(self.target_system, self.target_component, name.encode("ascii"),
                                             float(value), m.MAV_PARAM_TYPE_REAL32)
                msg = self._wait_for(("PARAM_VALUE",), timeout_s, lambda mm, n=name: self._param_id(mm) == n)
                if msg is not None:
                    ok = abs(float(msg.param_value) - float(value)) <= tol * max(1.0, abs(float(value)))
                    break
            out[name] = ok
        return out

    def get_param(self, name: str, *, timeout_s: float = 5.0, retries: int = 2) -> float | None:
        """PARAM_REQUEST_READ → PARAM_VALUE；無回應 None。"""
        for attempt in range(retries + 1):
            self.conn.mav.param_request_read_send(self.target_system, self.target_component, name.encode("ascii"), -1)
            msg = self._wait_for(("PARAM_VALUE",), timeout_s, lambda mm, n=name: self._param_id(mm) == n)
            if msg is not None:
                return float(msg.param_value)
        return None

    # -- 指令 --------------------------------------------------------------
    def command_long(self, command: int, *params: float, timeout_s: float | None = None, retries: int = 2,
                     confirmation: int = 0) -> int | None:
        """COMMAND_LONG → COMMAND_ACK.result（None = 無 ACK）。"""
        p = list(params) + [0.0] * (7 - len(params))
        to = self.timeout_s if timeout_s is None else float(timeout_s)
        for attempt in range(retries + 1):
            self.conn.mav.command_long_send(self.target_system, self.target_component, int(command), confirmation,
                                            *[float(x) for x in p[:7]])
            msg = self._wait_for(("COMMAND_ACK",), to, lambda mm, c=int(command): int(mm.command) == c)
            if msg is not None:
                return int(msg.result)
        return None

    def set_mode(self, name: str, *, timeout_s: float | None = None) -> bool:
        """ArduPlane 模式（`plane_mode_map`）：COMMAND_LONG DO_SET_MODE(base=CUSTOM_MODE_ENABLED, custom_mode)；
        ACK 非 ACCEPTED（或無 ACK）時改送 SET_MODE 並以心跳 custom_mode 確認。"""
        m = _mav
        modes = plane_mode_map()
        key = name.upper()
        if key not in modes:
            raise ValueError(f"未知 ArduPlane 模式 {name!r}；可用：{sorted(modes)}")
        cm = modes[key]
        res = self.command_long(m.MAV_CMD_DO_SET_MODE, m.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, cm, timeout_s=timeout_s)
        if res == m.MAV_RESULT_ACCEPTED:
            return True
        self.conn.mav.set_mode_send(self.target_system, m.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, cm)
        hb = self._wait_for(("HEARTBEAT",), self.timeout_s if timeout_s is None else float(timeout_s),
                            lambda mm: int(mm.custom_mode) == cm and mm.get_srcSystem() == self.target_system)
        return hb is not None

    def arm(self, arm: bool = True, *, force: bool = False, timeout_s: float | None = None) -> bool:
        m = _mav
        res = self.command_long(m.MAV_CMD_COMPONENT_ARM_DISARM, 1.0 if arm else 0.0, 21196.0 if force else 0.0,
                                timeout_s=timeout_s)
        return res == m.MAV_RESULT_ACCEPTED

    def start_mission(self, first_seq: int = 1, last_seq: int = 0, *, timeout_s: float | None = None) -> bool:
        """MAV_CMD_MISSION_START(param1 = first_seq, param2 = last_seq)；ArduPilot 會自行切 AUTO。"""
        m = _mav
        res = self.command_long(m.MAV_CMD_MISSION_START, float(first_seq), float(last_seq), timeout_s=timeout_s)
        return res == m.MAV_RESULT_ACCEPTED

    def set_current(self, seq: int, *, timeout_s: float | None = None) -> bool:
        """MISSION_SET_CURRENT(seq) → 等 MISSION_CURRENT.seq == seq。"""
        self.conn.mav.mission_set_current_send(self.target_system, self.target_component, int(seq))
        msg = self._wait_for(("MISSION_CURRENT",), self.timeout_s if timeout_s is None else float(timeout_s),
                             lambda mm: int(mm.seq) == int(seq))
        return msg is not None

    # -- 遙測 --------------------------------------------------------------
    def read_telemetry(self, timeout_s: float = 1.0, *, require_new_position: bool = True) -> VehicleTelemetry | None:
        """讀訊息最多 `timeout_s`，收到新的 GLOBAL_POSITION_INT（且已有 EXTENDED_SYS_STATE 與電量訊息）即提前結束；
        彙整最新快取為 :class:`VehicleTelemetry`。從未收到位置 → None。"""
        deadline = time.time() + float(timeout_s)
        t_pos0 = self._latest_t.get("GLOBAL_POSITION_INT")
        got_new = not require_new_position
        while time.time() < deadline:
            msg = self._recv(min(0.2, deadline - time.time()))
            if msg is None:
                continue
            if msg.get_type() == "GLOBAL_POSITION_INT" and self._latest_t.get("GLOBAL_POSITION_INT") != t_pos0:
                got_new = True
            if got_new and "EXTENDED_SYS_STATE" in self._latest and \
                    ("BATTERY_STATUS" in self._latest or "SYS_STATUS" in self._latest):
                break
        return self.snapshot()

    def snapshot(self) -> VehicleTelemetry | None:
        """以目前快取組 :class:`VehicleTelemetry`（不收訊）。"""
        m = _mav
        gp = self._latest.get("GLOBAL_POSITION_INT")
        if gp is None:
            return None
        hud = self._latest.get("VFR_HUD")
        bat = self._latest.get("BATTERY_STATUS")
        sysst = self._latest.get("SYS_STATUS")
        wind = self._latest.get("WIND")
        ext = self._latest.get("EXTENDED_SYS_STATE")
        cur = self._latest.get("MISSION_CURRENT")
        hb = self.last_heartbeat_msg
        lat, lon = gp.lat / 1e7, gp.lon / 1e7
        vx, vy = gp.vx / 100.0, gp.vy / 100.0
        gs = math.hypot(vx, vy)
        if hud is not None and hud.groundspeed > 0:
            gs = float(hud.groundspeed)
        elif gs <= 0.0 and self._gs_est is not None:
            gs = float(self._gs_est)          # 無 vx/vy／VFR_HUD：以位置差推估
        track: float | None = None
        if math.hypot(vx, vy) >= 3.0:
            track = math.degrees(math.atan2(vy, vx)) % 360.0
        elif self._track_deg is not None:
            track = self._track_deg
        hdg = gp.hdg / 100.0 if gp.hdg != 65535 else (float(hud.heading) if hud is not None else (track or 0.0))
        pct: float | None = None
        wh: float | None = None
        nom = self.nominal_wh
        if bat is not None:
            if bat.battery_remaining is not None and bat.battery_remaining >= 0:
                pct = float(bat.battery_remaining)
            if pct is None and getattr(bat, "energy_consumed", -1) not in (-1, None) and nom is not None:
                wh = nom - float(bat.energy_consumed) / 36.0     # hJ → Wh
        if pct is None and sysst is not None and sysst.battery_remaining >= 0:
            pct = float(sysst.battery_remaining)
        modes = {v: k for k, v in plane_mode_map().items()}
        mode = "UNKNOWN"
        armed = False
        if hb is not None:
            mode = modes.get(int(hb.custom_mode), f"MODE{int(hb.custom_mode)}")
            armed = bool(hb.base_mode & m.MAV_MODE_FLAG_SAFETY_ARMED)
        age = self.heartbeat_age_s()
        return VehicleTelemetry(
            t_s=self._t_s(gp), lat_deg=float(lat), lon_deg=float(lon), alt_msl_m=gp.alt / 1000.0,
            rel_alt_m=gp.relative_alt / 1000.0, heading_deg=float(hdg), groundspeed_mps=float(gs),
            airspeed_mps=float(hud.airspeed) if hud is not None else None, battery_pct=pct, battery_wh_remaining=wh,
            wind_speed_mps=float(wind.speed) if wind is not None else None,
            wind_dir_from_deg=float(wind.direction) % 360.0 if wind is not None else None,
            vtol_state=int(ext.vtol_state) if ext is not None else int(m.MAV_VTOL_STATE_UNDEFINED),
            landed_state=int(ext.landed_state) if ext is not None else int(m.MAV_LANDED_STATE_UNDEFINED),
            mission_seq=int(cur.seq) if cur is not None else 0, mode=mode, armed=armed, link_ok=age <= self.hb_timeout_s,
            track_deg=track, climb_mps=float(hud.climb) if hud is not None else None, heartbeat_age_s=age,
            reached_seq=self._reached_seq, t_epoch_s=time.time(), time_boot_ms=int(gp.time_boot_ms),
            sysid=int(self.target_system),
        )


def _ack_name(ack: int) -> str:
    if not HAVE_PYMAVLINK:
        return str(ack)
    e = _mav.enums.get("MAV_MISSION_RESULT", {})
    v = e.get(ack)
    return v.name if v is not None else str(ack)


# ---------------------------------------------------------------------------
# FleetLink
# ---------------------------------------------------------------------------
class FleetLink:
    """多機連線集合 ``{uav_id: VehicleLink}``。"""

    def __init__(self, links: Mapping[str, VehicleLink] | None = None) -> None:
        self.links: dict[str, VehicleLink] = dict(links or {})
        for u, l in self.links.items():
            if l.uav_id is None:
                l.uav_id = u
        self.seqmaps: dict[str, MissionSeqMap] = {}
        self.items: dict[str, MissionItems] = {}
        self.last_tel: dict[str, VehicleTelemetry] = {}
        self.last_wind_update_t_s: dict[str, float] = {}
        self.energy_state: dict[str, EnergyEventState] = {}

    def add(self, uav_id: str, link: VehicleLink) -> None:
        link.uav_id = uav_id
        self.links[uav_id] = link

    def __getitem__(self, uav_id: str) -> VehicleLink:
        return self.links[uav_id]

    def __contains__(self, uav_id: str) -> bool:
        return uav_id in self.links

    def wait_heartbeats(self, timeout_s: float | None = None) -> dict[str, Any]:
        return {u: l.wait_heartbeat(timeout_s) for u, l in self.links.items()}

    def close(self) -> None:
        for l in self.links.values():
            l.close()

    # -- 上傳 --------------------------------------------------------------
    def upload_plan(self, plan: MissionPlan, ctx: Any, cfg: PlannerConfig | None = None,
                    out_dir: str | os.PathLike[str] | None = None, *, uav_ids: Sequence[str] | None = None,
                    rally: bool = True, fence: bool = True, write_params: bool = True, set_params: bool = False,
                    frame: int | str | None = None, mission_id: str = "", timeout_s: float | None = None,
                    **item_kw: Any) -> dict[str, PlanUploadReport]:
        """每架（有連線且在 plan 中者）：任務項目（:func:`plan_to_mission_items`）上傳 + rally（home + 備降點）+
        該構型 fence；`out_dir` 給定 → `hv_dccpp.ardupilot.export_all`（`<uav>.waypoints`、`<uav>_rally.txt`、
        `<uav>_fence.txt`、`<uav>.param`、`<uav>.plan`、`mission_summary.json`；不可用時本模組後備寫 `<uav>.waypoints`、
        `rally.waypoints`、`fence_<cls>.waypoints`、`<uav>.param`）並一律加寫 `<uav>.seqmap.json`。
        `set_params=True` 才以 PARAM_SET 寫入建議參數（預設不寫，§15.2）。`item_kw` 透傳給 `plan_to_mission_items`
        （如 `camera_trigger`、`wp_radius_m`、`strip_sensors`）。"""
        cfg = cfg or getattr(ctx, "cfg", None) or PlannerConfig()
        ids = [u for u in (uav_ids or plan.plans.keys()) if u in self.links and u in plan.plans]
        od = Path(out_dir) if out_dir is not None else None
        ap = _ardupilot_module()
        exported: dict[str, Any] = {}
        if od is not None:
            od.mkdir(parents=True, exist_ok=True)
            if ap is not None and hasattr(ap, "export_all"):
                kw_exp = {k: v for k, v in item_kw.items() if k in ("camera_trigger", "wp_radius_m", "strip_sensors")}
                summary = _call_compat(ap.export_all, plan, ctx, cfg, od, alt_frame=_alt_frame_name(frame), **kw_exp)
                exported = dict(summary.get("uavs", {}))
        inputs = ctx.inputs
        r_items = rally_items(inputs, ctx, frame=frame) if rally else None
        f_by_cls: dict[str, MissionItems] = {}
        shared_files: list[str] = []
        if od is not None and not exported:
            if r_items is not None:
                shared_files.append(write_qgc_wpl(r_items, od / "rally.waypoints"))
        reports: dict[str, PlanUploadReport] = {}
        for u in ids:
            link = self.links[u]
            up = plan.plans[u]
            items = plan_to_mission_items(up, ctx, cfg, frame=frame, mission_id=mission_id, **item_kw)
            f_items: MissionItems | None = None
            if fence:
                if up.cls not in f_by_cls:
                    f_by_cls[up.cls] = fence_items(up, ctx, cfg, cls=up.cls)
                    if od is not None and not exported:
                        shared_files.append(write_qgc_wpl(f_by_cls[up.cls], od / f"fence_{up.cls}.waypoints"))
                f_items = f_by_cls[up.cls]
            files = list(shared_files)
            if od is not None:
                if exported and u in exported:
                    files.extend(str(v) for v in exported[u].get("files", {}).values())
                else:
                    files.append(write_qgc_wpl(items, od / f"{u}.waypoints"))
                if items.seqmap is not None:
                    files.append(items.seqmap.save(od / f"{u}.seqmap.json"))
            params: dict[str, float] | None = None
            if write_params or set_params:
                params = suggest_params(ctx.specs[up.cls], ctx, cfg, uav_plan=up, uav_id=u)
                if od is not None and write_params and not (exported and u in exported):
                    files.append(write_param_file(params, od / f"{u}.param",
                                                  header=[f"HV-DCCPP suggested params for {u} ({up.cls}); review before use"]))
            res_m = link.upload_mission(items, _mav.MAV_MISSION_TYPE_MISSION, timeout_s=timeout_s)
            res_r = link.upload_rally(r_items, timeout_s=timeout_s) if r_items is not None else None
            res_f = link.upload_fence(f_items, timeout_s=timeout_s) if f_items is not None else None
            pres = link.set_params(params) if (set_params and params) else None
            link.seqmap = items.seqmap
            link.last_items = items
            self.seqmaps[u] = items.seqmap
            self.items[u] = items
            reports[u] = PlanUploadReport(u, res_m, res_r, res_f, pres, items.seqmap, items, files)
        return reports

    def replan_and_reupload(self, uav_id: str, new_plan: UAVPlan | MissionPlan, ctx: Any, cfg: PlannerConfig | None = None, *,
                            current_strip_id: str | None = None, tel: VehicleTelemetry | None = None,
                            diff_start_wp: int | None = None, out_dir: str | os.PathLike[str] | None = None,
                            frame: int | None = None, mission_id: str = "", timeout_s: float | None = None,
                            set_current: bool = True) -> ReuploadResult:
        """重規劃後重傳整份新任務並 MISSION_SET_CURRENT 至切換點（設計文件 §15.0 (5)、§15.5.2）。

        切換點 `switch_seq`：現行條帶（`current_strip_id`，或由 `tel.mission_seq` 經舊 seqmap 推得）末端之後在**新**任務中的第一個
        seq（= 下一條帶之轉移起點）；現行條帶不在新任務（已完成／被切除）或無現行條帶 → `diff_start_wp`（`len(old) − len(diff)`）
        對應之首個 seq；皆無 → 不切換（`switch_seq=None`）。"""
        link = self.links[uav_id]
        up = _uav_plan(new_plan, uav_id)
        cfg = cfg or getattr(ctx, "cfg", None) or PlannerConfig()
        old_sm = self.seqmaps.get(uav_id) or link.seqmap
        if current_strip_id is None and tel is not None and old_sm is not None:
            current_strip_id, _inside = old_sm.strip_at(int(tel.mission_seq))
        items = plan_to_mission_items(up, ctx, cfg, frame=frame, mission_id=mission_id)
        sm = items.seqmap
        files: list[str] = []
        if out_dir is not None:
            od = Path(out_dir)
            od.mkdir(parents=True, exist_ok=True)
            files.append(write_qgc_wpl(items, od / f"{uav_id}.waypoints"))
            if sm is not None:
                files.append(sm.save(od / f"{uav_id}.seqmap.json"))
        res = link.upload_mission(items, _mav.MAV_MISSION_TYPE_MISSION, timeout_s=timeout_s)
        switch: int | None = None
        if sm is not None:
            if current_strip_id is not None:
                switch = sm.seq_after_strip(current_strip_id)
            if switch is None and diff_start_wp is not None:
                switch = sm.seq_of_wp(int(diff_start_wp))
        ok = False
        if set_current and switch is not None:
            ok = link.set_current(switch, timeout_s=timeout_s)
        link.seqmap = sm
        link.last_items = items
        if sm is not None:
            self.seqmaps[uav_id] = sm
        self.items[uav_id] = items
        return ReuploadResult(uav_id, res, sm, items, switch, ok, files)

    # -- 遙測 / 事件 ---------------------------------------------------------
    def poll(self, plan: MissionPlan, ctx: Any, cfg: PlannerConfig, *, timeout_s: float = 0.5,
             **event_kw: Any) -> tuple[dict[str, Any], list[Any]]:
        """每機 `read_telemetry` → 規劃器 Telemetry 與事件（`detect_events`）；回 (`{uav_id: dynamic.Telemetry}`, events)。

        ENERGY_UPDATE 以每機保存之 :class:`EnergyEventState`（`self.energy_state`）做遲滯／防抖，同一段能量偏差
        只送一次；`event_kw` 透傳給 :func:`detect_events`（如 `energy_dwell_s`、`energy_min_interval_s`）。"""
        tels: dict[str, Any] = {}
        events: list[Any] = []
        for u, link in self.links.items():
            if u not in plan.plans:
                continue
            vt = link.read_telemetry(timeout_s)
            if vt is None:
                continue
            sm = self.seqmaps.get(u) or link.seqmap
            pt = to_planner_telemetry(vt, plan.plans[u], ctx, u, seqmap=sm)
            es = self.energy_state.setdefault(u, EnergyEventState())
            evs = detect_events(vt, plan.plans[u], ctx, cfg, self.last_tel.get(u), uav_id=u, seqmap=sm, planner_tel=pt,
                                last_wind_update_t_s=self.last_wind_update_t_s.get(u), energy_state=es, **event_kw)
            for e in evs:
                if type(e).__name__ == "WindUpdate":
                    self.last_wind_update_t_s[u] = float(pt.t_s)
            tels[u] = pt
            events.extend(evs)
            self.last_tel[u] = vt
        return tels, events
