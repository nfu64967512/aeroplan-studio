"""ardupilot.py — HV-DCCPP `UAVPlan` → ArduPilot QuadPlane（ArduPlane + Q_ENABLE）任務輸出（純輸出層）。

依 `scratchpad/ardupilot_notes.md` 與 `integration_notes.md`：
* 任務項目（MAV_MISSION_TYPE_MISSION）：NAV_VTOL_TAKEOFF(84) → NAV_WAYPOINT(16)／NAV_LOITER_TO_ALT(31)／
  NAV_LOITER_TURNS(18)／NAV_LOITER_TIME(19) → DO_LAND_START(189) → NAV_VTOL_LAND(85)；
  DO_CHANGE_SPEED(178) 於空速改變處；作業條帶以 DO_SET_CAM_TRIGG_DIST(206) dist／0 包住（rgb），
  LiDAR 以 DO_SET_SERVO(183) 佔位（見 :data:`LIDAR_SERVO_CHANNEL`）。
* 弧段（`Waypoint.arc`）以外切多邊形重新離散（規格 §8.5、論文 Eq.(28)(29)、D27：
  Δψ = min(2·acos(R/(R+e_max)), ψ_max)），直線段只留端點（高度剖面以 Douglas–Peucker 保留 > `delta_prof_m` 之轉折），
  並丟棄距前一航點小於接受半徑 r_acc = min(R_eff/2, 30) 之非必要點。
* 高度框架：`relative`（MAV_FRAME_GLOBAL_RELATIVE_ALT=3，alt = MSL − home MSL）、`msl`（MAV_FRAME_GLOBAL=0）、
  `terrain`（MAV_FRAME_GLOBAL_TERRAIN_ALT=10，alt = AGL；需 TERRAIN_ENABLE=1）。
* 第 0 列 home（frame 0、cmd 16、current 1）；QGC WPL 110（12 欄 tab 分隔，lat/lon 7 位小數）；
  rally（NAV_RALLY_POINT 5100）、fence（5000/5001/5002）、Mission Planner `.param`、QGC `.plan` JSON。

本模組只依賴 numpy／shapely 與 hv_dccpp 之 mission/context/platform/dubins；pymavlink 為**可選**（僅用於常數自檢，
未安裝時使用本檔內建之同值常數）。所有 MAV_CMD 常數以 `mavutil.mavlink.MAV_CMD_*` 取得（不手寫數字）。
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, asdict
from typing import Any, Iterable, Literal, Mapping, Sequence

import numpy as np
import shapely
from shapely.geometry import MultiPolygon, Polygon
from shapely.ops import unary_union

from .context import MissionContext
from .decompose import d_turn_from_r
from .dubins import discretize_step_rad
from .mission import FW_PHASES, MissionInputs, MissionPlan, Mode, Phase, PlannerConfig, UAVPlan, Waypoint
from .platform import EnergyModel, PlatformSpec, Sensor

os.environ.setdefault("MAVLINK20", "1")

try:  # pragma: no cover - 依環境
    from pymavlink import mavutil as _mavutil
    _M = _mavutil.mavlink
    HAVE_PYMAVLINK = True
except Exception:  # noqa: BLE001
    _M = None
    HAVE_PYMAVLINK = False


def _const(name: str, fallback: int) -> int:
    """`mavutil.mavlink.<name>`；pymavlink 不可用時用 fallback（值已以 pymavlink 2.4.49 驗證）。"""
    if _M is not None and hasattr(_M, name):
        return int(getattr(_M, name))          # 常數不一致時以 pymavlink 為準
    return fallback


# ---------------------------------------------------------------------------
# MAVLink 常數（值皆自 pymavlink 取得；fallback 僅供無 pymavlink 之環境）
# ---------------------------------------------------------------------------
MAV_CMD_NAV_WAYPOINT = _const("MAV_CMD_NAV_WAYPOINT", 16)
MAV_CMD_NAV_DELAY = _const("MAV_CMD_NAV_DELAY", 93)
MAV_CMD_NAV_LOITER_TURNS = _const("MAV_CMD_NAV_LOITER_TURNS", 18)
MAV_CMD_NAV_LOITER_TIME = _const("MAV_CMD_NAV_LOITER_TIME", 19)
MAV_CMD_NAV_RETURN_TO_LAUNCH = _const("MAV_CMD_NAV_RETURN_TO_LAUNCH", 20)
MAV_CMD_NAV_LOITER_TO_ALT = _const("MAV_CMD_NAV_LOITER_TO_ALT", 31)
MAV_CMD_NAV_VTOL_TAKEOFF = _const("MAV_CMD_NAV_VTOL_TAKEOFF", 84)
MAV_CMD_NAV_VTOL_LAND = _const("MAV_CMD_NAV_VTOL_LAND", 85)
MAV_CMD_DO_CHANGE_SPEED = _const("MAV_CMD_DO_CHANGE_SPEED", 178)
MAV_CMD_DO_SET_SERVO = _const("MAV_CMD_DO_SET_SERVO", 183)
MAV_CMD_DO_LAND_START = _const("MAV_CMD_DO_LAND_START", 189)
MAV_CMD_DO_SET_CAM_TRIGG_DIST = _const("MAV_CMD_DO_SET_CAM_TRIGG_DIST", 206)
MAV_CMD_DO_VTOL_TRANSITION = _const("MAV_CMD_DO_VTOL_TRANSITION", 3000)
MAV_CMD_NAV_FENCE_RETURN_POINT = _const("MAV_CMD_NAV_FENCE_RETURN_POINT", 5000)
MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = _const("MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION", 5001)
MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = _const("MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION", 5002)
MAV_CMD_NAV_RALLY_POINT = _const("MAV_CMD_NAV_RALLY_POINT", 5100)

MAV_FRAME_GLOBAL = _const("MAV_FRAME_GLOBAL", 0)
MAV_FRAME_GLOBAL_RELATIVE_ALT = _const("MAV_FRAME_GLOBAL_RELATIVE_ALT", 3)
MAV_FRAME_GLOBAL_TERRAIN_ALT = _const("MAV_FRAME_GLOBAL_TERRAIN_ALT", 10)

MAV_MISSION_TYPE_MISSION = _const("MAV_MISSION_TYPE_MISSION", 0)
MAV_MISSION_TYPE_FENCE = _const("MAV_MISSION_TYPE_FENCE", 1)
MAV_MISSION_TYPE_RALLY = _const("MAV_MISSION_TYPE_RALLY", 2)

NAV_COMMANDS: frozenset[int] = frozenset({
    MAV_CMD_NAV_WAYPOINT, MAV_CMD_NAV_LOITER_TURNS, MAV_CMD_NAV_LOITER_TIME, MAV_CMD_NAV_RETURN_TO_LAUNCH,
    MAV_CMD_NAV_LOITER_TO_ALT, MAV_CMD_NAV_VTOL_TAKEOFF, MAV_CMD_NAV_VTOL_LAND, MAV_CMD_NAV_DELAY,
})
"""會推進任務序列之 NAV 指令（DO_* 為附掛於前一 NAV 之即時指令）。"""

AltFrame = Literal["relative", "msl", "terrain"]
FRAME_OF: dict[str, int] = {"relative": MAV_FRAME_GLOBAL_RELATIVE_ALT, "msl": MAV_FRAME_GLOBAL,
                            "terrain": MAV_FRAME_GLOBAL_TERRAIN_ALT}

# LiDAR 佔位：ArduPilot 無通用「LiDAR 開／關」指令，依酬載以 AUX 伺服／繼電器觸發（待實機驗證，ardupilot_notes §1）。
LIDAR_SERVO_CHANNEL: int = 9
"""DO_SET_SERVO param1：LiDAR 錄製開關之 AUX 輸出通道（佔位；依 SERVOx_FUNCTION 配置調整）。"""
LIDAR_SERVO_PWM_ON: int = 1900
LIDAR_SERVO_PWM_OFF: int = 1100
CAMERA_SERVO_CHANNEL: int = 10
"""相機觸發輸出通道（SERVOx_FUNCTION = 10 CameraTrigger）；佔位，依實機接線調整。"""

QGC_WPL_HEADER = "QGC WPL 110"
_QGC_FIRMWARE_ARDUPILOT = 3
_QGC_VEHICLE_FIXED_WING = 2

_LANDING_PHASES = frozenset({Phase.RTB, Phase.DESCENT, Phase.DECEL, Phase.TRANS_IN, Phase.LAND})
_FULL_LAP_TOL_RAD = 0.1


# ---------------------------------------------------------------------------
# 資料型別
# ---------------------------------------------------------------------------
@dataclass(slots=True)
class MissionItem:
    """一列 MISSION_ITEM_INT 等價資料（度／公尺；`x=lat_deg`、`y=lon_deg`、`z=alt_m` 依 `frame` 解讀）。"""
    seq: int
    frame: int
    command: int
    current: int
    autocontinue: int
    param1: float
    param2: float
    param3: float
    param4: float
    x: float
    y: float
    z: float
    mission_type: int = 0

    @property
    def is_nav(self) -> bool:
        return self.command in NAV_COMMANDS

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    def as_mission_item_int_kwargs(self, target_system: int = 1, target_component: int = 1) -> dict[str, Any]:
        """`mav.mission_item_int_send(**kw)` 之關鍵字（lat/lon ×1e7 取整）。"""
        return dict(target_system=target_system, target_component=target_component, seq=self.seq, frame=self.frame,
                    command=self.command, current=self.current, autocontinue=self.autocontinue,
                    param1=float(self.param1), param2=float(self.param2), param3=float(self.param3),
                    param4=float(self.param4), x=int(round(self.x * 1e7)), y=int(round(self.y * 1e7)),
                    z=float(self.z), mission_type=self.mission_type)


def _item(seq: int, command: int, frame: int, p: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0),
          x: float = 0.0, y: float = 0.0, z: float = 0.0, current: int = 0, mission_type: int = 0) -> MissionItem:
    return MissionItem(int(seq), int(frame), int(command), int(current), 1, float(p[0]), float(p[1]), float(p[2]),
                       float(p[3]), float(x), float(y), float(z), int(mission_type))


def renumber(items: Sequence[MissionItem]) -> list[MissionItem]:
    """依序重編 `seq`（就地修改並回傳同一 list）。"""
    out = list(items)
    for i, it in enumerate(out):
        it.seq = i
    return out


# ---------------------------------------------------------------------------
# 幾何輔助
# ---------------------------------------------------------------------------
def _home_msl(ctx: MissionContext) -> float:
    xy, z = ctx.sites_enu[ctx.inputs.home.site_id]
    return float(ctx.frame.enu_z_to_msl(np.asarray(xy, dtype=float).reshape(1, 2), float(z))[0])


def _site_msl(ctx: MissionContext, site_id: str) -> float:
    xy, z = ctx.sites_enu[site_id]
    return float(ctx.frame.enu_z_to_msl(np.asarray(xy, dtype=float).reshape(1, 2), float(z))[0])


def _wgs(ctx: MissionContext, xy: np.ndarray, z: np.ndarray | float) -> np.ndarray:
    """ENU (N,2)+z → (N,3) [lat, lon, msl]。"""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    zz = np.broadcast_to(np.asarray(z, dtype=float), (xy.shape[0],))
    return ctx.frame.to_wgs84(np.column_stack([xy, zz]))


def accept_radius_m(r_eff_m: float, wp_radius_m: float | None = None) -> float:
    """r_acc = min(R_eff/2, 30) m（規格 §8.5）；`wp_radius_m` 覆寫。"""
    if wp_radius_m is not None:
        return float(wp_radius_m)
    return float(min(0.5 * float(r_eff_m), 30.0))


def _arc_sweep(pts: np.ndarray, center: tuple[float, float]) -> float:
    """依序取樣點之累積掃掠角（rad，帶號：+ 逆時針（幾何角增）、− 順時針）。"""
    ang = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
    d = np.diff(ang)
    d = (d + math.pi) % (2.0 * math.pi) - math.pi
    return float(d.sum())


def arc_vertices(p_start: np.ndarray, p_end: np.ndarray, center: tuple[float, float], R: float, turn_dir: int,
                 dpsi_rad: float, sweep_rad: float | None = None, force_single: bool = False) -> np.ndarray:
    """外切多邊形頂點（論文 Eq.(29)）：自 `p_start` 沿圓 (center, R) 轉向 `turn_dir`（+1 順時針／右轉、−1 逆時針／左轉）
    至 `p_end`，n = ⌈|Δ|/Δψ⌉ 段，頂點在每段中點方向、徑向距 R/cos(δ/2)；回傳 (n,2)，**不含** p_end。
    `sweep_rad` 可給定（帶號幾何掃掠角）以消除 2π 模糊。|Δ| ≤ Δψ 時弦之矢高已 ≤ e_max，回傳空陣列
    （`force_single=True` 仍給 1 個頂點）。"""
    cx, cy = center
    a0 = math.atan2(p_start[1] - cy, p_start[0] - cx)
    if sweep_rad is None:
        a1 = math.atan2(p_end[1] - cy, p_end[0] - cx)
        if turn_dir > 0:      # 順時針：幾何角遞減
            sweep = -((a0 - a1) % (2.0 * math.pi))
        else:
            sweep = (a1 - a0) % (2.0 * math.pi)
    else:
        sweep = float(sweep_rad)
    if abs(sweep) < 1e-9:
        return np.zeros((0, 2))
    n = max(1, int(math.ceil(abs(sweep) / dpsi_rad - 1e-9)))
    if n == 1 and not force_single:
        # |Δ| ≤ Δψ：弦之矢高 R(1 − cos(Δ/2)) ≤ R·e_max/(R + e_max) < e_max，直接以起訖點連線即可，不需頂點
        return np.zeros((0, 2))
    delta = sweep / n
    r_v = R / math.cos(0.5 * abs(delta))
    ang = a0 + (np.arange(n) + 0.5) * delta
    return np.column_stack([cx + r_v * np.cos(ang), cy + r_v * np.sin(ang)])


def _dp_keep(s: np.ndarray, z: np.ndarray, tol: float) -> np.ndarray:
    """(s, z) 折線之 Douglas–Peucker 保留遮罩（端點恆保留）。"""
    n = len(s)
    keep = np.zeros(n, dtype=bool)
    if n == 0:
        return keep
    keep[0] = keep[-1] = True
    stack = [(0, n - 1)]
    while stack:
        i, j = stack.pop()
        if j - i < 2:
            continue
        zi = z[i] + (z[j] - z[i]) * (s[i + 1:j] - s[i]) / max(s[j] - s[i], 1e-9)
        dev = np.abs(z[i + 1:j] - zi)
        k = int(np.argmax(dev))
        if dev[k] > tol:
            m = i + 1 + k
            keep[m] = True
            stack.append((i, m))
            stack.append((m, j))
    return keep


# ---------------------------------------------------------------------------
# 航點縮減（規格 §8.5；ardupilot_notes §1）
# ---------------------------------------------------------------------------
@dataclass(slots=True)
class _NavPoint:
    """縮減後之導航點（ENU）。`kind`: 'wp' | 'takeoff' | 'loiter_alt' | 'loiter_turns' | 'loiter_time' | 'land'。"""
    x: float
    y: float
    z: float
    agl: float
    t: float
    phase: Phase
    mode: Mode
    v_air: float
    sensor_on: bool
    strip_id: str | None
    essential: bool
    kind: str = "wp"
    radius_signed: float = 0.0
    laps: int = 0
    duration_s: float = 0.0
    src: int = -1                      # 來源 Waypoint 索引
    vertex: bool = False               # 外切多邊形頂點（在相鄰接點之切線上；不因接受半徑被丟棄）
    protected: bool = False            # 無頂點短弧之起點（弦矢高不可忽略；不因接受半徑被丟棄，但不在切線上）
    into_line: bool = False            # 弧起點且入航段為直線（與前一保留點、首頂點共線 → 可省略）


def _runs(wps: Sequence[Waypoint]) -> list[tuple[int, int, str]]:
    """分割為 [(i0, i1, kind)]（含 i1）：同一弧（arc tuple 相同）為 'arc'；直線為 'line'；
    分割於 arc／strip／sensor／mode 變化處及走廊／ORBIT 階段邊界（FW 階段標籤 CLIMB→ENTRY 等中途改名不切分）。"""
    runs: list[tuple[int, int, str]] = []
    n = len(wps)
    i = 0

    def key_of(w: Waypoint) -> tuple:
        ph = w.phase if (w.phase is Phase.ORBIT or w.phase not in FW_PHASES) else None
        return (w.arc, ph, w.strip_id, w.sensor_on, w.mode)

    while i < n:
        w = wps[i]
        key = key_of(w)
        j = i
        while j + 1 < n:
            if key_of(wps[j + 1]) != key:
                break
            j += 1
        runs.append((i, j, "arc" if w.arc is not None else "line"))
        i = j + 1
    return runs


def _boundary(wps: Sequence[Waypoint], i: int) -> bool:
    """第 i 點是否為必要點：strip／sensor／mode 邊界或緊接轉換段之首點（FW 階段標籤之變化本身不構成必要點）。"""
    if i == 0 or i >= len(wps):
        return True
    p, q = wps[i - 1], wps[i]
    return (p.strip_id != q.strip_id or p.sensor_on != q.sensor_on or p.mode is not q.mode
            or p.phase in (Phase.TRANS_OUT, Phase.TRANS_IN, Phase.TAKEOFF, Phase.LAND))


def reduce_waypoints(wps: Sequence[Waypoint], spec: PlatformSpec, cfg: PlannerConfig, r_acc_m: float,
                     z_tol_m: float | None = None, heading_tol_deg: float = 1.0) -> list[_NavPoint]:
    """`UAVPlan.waypoints`（build_3d 之密集取樣）→ autopilot 導航點：

    * 弧段：同一 `arc` 之連續點以外切多邊形重離散（Δψ 依 D27，e_max = cfg.e_max_m、ψ_max = cfg.psi_max_deg），
      ORBIT 階段之整圈（|Δ| ≥ 2π − 0.1）改為盤旋指令（loiter_alt／loiter_turns／loiter_time）。
    * 直線段：保留端點、航向轉折（> heading_tol_deg）、高度剖面 DP 轉折（> z_tol_m；預設 cfg.delta_prof_m）。
    * TRANS_OUT／TRANS_IN 之中間取樣點全部略去（ArduPilot 自行轉換）；TAKEOFF → 'takeoff'、LAND → 'land'。
    * 非必要點距前一保留點 < r_acc 時丟棄。
    """
    n = len(wps)
    if n == 0:
        return []
    z_tol = float(cfg.delta_prof_m if z_tol_m is None else z_tol_m)
    hd_tol = math.radians(heading_tol_deg)
    xy = np.array([[w.x_m, w.y_m] for w in wps], dtype=float)
    z = np.array([w.z_m for w in wps], dtype=float)
    agl = np.array([w.agl_m for w in wps], dtype=float)
    t = np.array([w.t_s for w in wps], dtype=float)
    out: list[_NavPoint] = []

    def mk(i: int, kind: str = "wp", essential: bool = False, **kw: Any) -> _NavPoint:
        w = wps[i]
        return _NavPoint(float(xy[i, 0]), float(xy[i, 1]), float(z[i]), float(agl[i]), float(t[i]), w.phase, w.mode,
                         float(w.airspeed_mps), bool(w.sensor_on), w.strip_id, essential, kind, src=i, **kw)

    for (i0, i1, kind) in _runs(wps):
        w0 = wps[i0]
        ph = w0.phase
        if ph is Phase.TAKEOFF:
            out.append(mk(i0, "takeoff", True))
            continue
        if ph is Phase.LAND:
            out.append(mk(i0, "land", True))
            continue
        if ph in (Phase.TRANS_OUT, Phase.TRANS_IN):
            continue                                   # ArduPilot 自行執行轉換；端點由相鄰 ACCEL／LAND 供給
        i_end = min(i1 + 1, n - 1)                     # 本 run 之出航段終點（下一 run 首點）
        if kind == "arc":
            assert w0.arc is not None
            cx, cy, R, tdir = float(w0.arc[0]), float(w0.arc[1]), float(w0.arc[2]), int(w0.arc[3])
            pts = xy[i0:i_end + 1]
            sweep = _arc_sweep(pts, (cx, cy))
            single_closed = (i_end == i0) or (np.hypot(*(pts[-1] - pts[0])) < 1.0 and abs(sweep) < 1e-6)
            dz = float(z[i_end] - z[i0])
            dt = float(t[i_end] - t[i0]) if i_end > i0 else 0.0
            if ph is Phase.ORBIT and (abs(sweep) >= 2.0 * math.pi - _FULL_LAP_TOL_RAD or single_closed):
                # 整圈盤旋（起飛／進入螺旋、去衝突延遲盤旋）
                va = max(float(w0.airspeed_mps), 1e-6)
                t_lap = 2.0 * math.pi * R / va
                laps = int(round(abs(sweep) / (2.0 * math.pi))) if abs(sweep) >= 2.0 * math.pi - _FULL_LAP_TOL_RAD \
                    else max(1, int(round(dt / t_lap))) if dt > 0 else 1
                p = mk(i0, "loiter_alt" if abs(dz) >= 1.0 else ("loiter_time" if dt > 0 else "loiter_turns"), True,
                       radius_signed=(R if tdir > 0 else -R), laps=laps, duration_s=dt)
                p.x, p.y = cx, cy
                p.z, p.agl = float(z[i_end]), float(agl[i_end])
                out.append(p)
                continue
            dpsi = discretize_step_rad(R, cfg.e_max_m, cfg.psi_max_rad)
            # |Δ| ≤ Δψ 之短弧不產生頂點（弦矢高 ≤ e_max）；矢高不可忽略（≥ 0.25 m）時其起點承載幾何 → 保護
            short_arc = 1e-9 < abs(sweep) <= dpsi + 1e-9
            sagitta = R * (1.0 - math.cos(0.5 * abs(sweep)))
            out.append(mk(i0, "wp", _boundary(wps, i0), into_line=(i0 > 0 and wps[i0 - 1].arc is None),
                          protected=(short_arc and sagitta >= 0.25)))
            if abs(sweep) < 1e-9:
                continue
            verts = arc_vertices(xy[i0], xy[i_end], (cx, cy), R, tdir, dpsi, sweep_rad=sweep)
            if len(verts):
                frac = (np.arange(len(verts)) + 0.5) / len(verts)
                zs = z[i0] + frac * dz
                ts = t[i0] + frac * dt
                agls = agl[i0] + frac * (agl[i_end] - agl[i0])
                for k, v in enumerate(verts):
                    out.append(_NavPoint(float(v[0]), float(v[1]), float(zs[k]), float(agls[k]), float(ts[k]), ph,
                                         w0.mode, float(w0.airspeed_mps), bool(w0.sensor_on), w0.strip_id, False,
                                         "wp", src=i0, vertex=True))
            continue
        # ---- 直線 run：i0..i1，出航段終點 i_end ----
        idx = np.arange(i0, i_end + 1)
        seg = xy[idx]
        d = np.diff(seg, axis=0)
        s = np.concatenate([[0.0], np.cumsum(np.hypot(d[:, 0], d[:, 1]))])
        keep = _dp_keep(s, z[idx], z_tol)
        if len(idx) > 2:
            hdg = np.arctan2(d[:, 0], d[:, 1])
            dh = np.abs((np.diff(hdg) + math.pi) % (2.0 * math.pi) - math.pi)
            keep[1:-1] |= dh > hd_tol
        keep[-1] = False                               # 終點由下一 run 首點供給
        for k, i in enumerate(idx):
            if keep[k]:
                out.append(mk(int(i), "wp", _boundary(wps, int(i)) if k == 0 else False))
    # 末點（LAND 已處理；若計畫無 LAND，補最後一點）
    if wps[-1].phase is not Phase.LAND and (not out or out[-1].src != n - 1):
        out.append(mk(n - 1, "wp", True))
    # ---- 接受半徑過濾：距前一保留點 < r_acc 時，非必要／非頂點／非保護之點丟棄；必要點取代前一可丟棄點；
    #      弧頂點取代前一可丟棄之弧起點，惟須共線（入航為直線或前一保留點為頂點——切線構造）；否則並存 ----
    filtered: list[_NavPoint] = []
    for p in out:
        if filtered and p.kind == "wp" and filtered[-1].kind in ("wp", "takeoff"):
            if math.hypot(p.x - filtered[-1].x, p.y - filtered[-1].y) < r_acc_m:
                if not (p.essential or p.vertex or p.protected):
                    continue
                last = filtered[-1]
                collinear = last.into_line or (len(filtered) >= 2 and filtered[-2].vertex)
                droppable = last.kind == "wp" and not (last.essential or last.vertex or last.protected)
                if droppable and (p.essential or collinear):
                    filtered.pop()
        filtered.append(p)
    return filtered


# ---------------------------------------------------------------------------
# 感測器觸發距離
# ---------------------------------------------------------------------------
def _sensor_for(uav_plan: UAVPlan, ctx: MissionContext, strip_id: str | None,
                strip_sensors: Mapping[str, Sensor] | None) -> Sensor | None:
    """條帶之感測器：`strip_sensors[strip_id]` → 同構型主感測種類之 ctx.sensors → 該構型任一。"""
    if strip_sensors is not None and strip_id is not None and strip_id in strip_sensors:
        return strip_sensors[strip_id]
    cls = uav_plan.cls
    cands = [(k, s) for k, s in ctx.sensors.items() if k[0] == cls]
    if not cands:
        return None
    try:
        primary_kind = ctx.uav(uav_plan.uav_id).primary_sensor.kind
    except Exception:  # noqa: BLE001
        primary_kind = None
    for _k, s in cands:
        if getattr(s, "kind", None) == primary_kind:
            return s
    return cands[0][1]


LOITER_TURNS_MAX_RADIUS_M: int = 255
"""NAV_LOITER_TURNS 之 |param3|：ArduPlane 將半徑存入 `cmd.p1` 高位元組（整數公尺，上限 255）。"""


def _loiter_turns_radius(radius_signed: float) -> float:
    """NAV_LOITER_TURNS param3：取整並夾至 ±255 m（ArduPlane `p1 = (radius_m << 8) | turns`）。"""
    r = int(round(abs(float(radius_signed))))
    r = max(0, min(LOITER_TURNS_MAX_RADIUS_M, r))
    return float(-r if radius_signed < 0 else r)


def orbit_radius_m(items: Sequence[MissionItem]) -> float | None:
    """任務中盤旋指令實際使用之半徑（m）；無盤旋 → None。

    ArduPlane 對 NAV_LOITER_TIME(19) **不讀** param3 之半徑（只讀正負號＝方向），一律以 `WP_LOITER_RAD` 飛；
    因此匯出時須把 `WP_LOITER_RAD` 設為本值，否則延遲盤旋之圈時／半徑與去衝突所保留者不符。
    """
    rs: list[float] = []
    for it in items:
        if it.command in (MAV_CMD_NAV_LOITER_TIME, MAV_CMD_NAV_LOITER_TURNS) and abs(it.param3) > 1e-6:
            rs.append(abs(float(it.param3)))
        elif it.command == MAV_CMD_NAV_LOITER_TO_ALT and abs(it.param2) > 1e-6:
            rs.append(abs(float(it.param2)))
    return max(rs) if rs else None


def min_nav_spacing_m(items: Sequence[MissionItem], ctx: MissionContext) -> float | None:
    """相鄰（有座標之）NAV 項目之最小水平間距（m）；不足兩點 → None。

    ArduPlane 以 `WP_RADIUS` 判定所有航點之抵達（NAV_WAYPOINT param2 被忽略），故 `WP_RADIUS` 必須小於本值，
    否則後一航點在前一航點被判抵達的同一瞬間即被判抵達，掛在其後之 DO_ 指令會提早觸發。
    """
    pts = [(it.x, it.y) for it in items
           if it.is_nav and it.seq > 0 and not (it.x == 0.0 and it.y == 0.0)]
    if len(pts) < 2:
        return None
    xy = ctx.frame.to_enu(np.array([p[0] for p in pts]), np.array([p[1] for p in pts]))[:, :2]
    d = np.hypot(*np.diff(xy, axis=0).T)
    return float(d.min()) if d.size else None


def trigger_distance_m(sensor: Sensor, h_agl_m: float) -> float:
    """相機等距觸發距離 = 足跡前向長 l(h)·(1 − o_fwd)（規格 §4.2）；LiDAR（l=0）→ 0。"""
    l_fp = float(sensor.footprint_len_m(float(h_agl_m)))
    fwd = float(getattr(getattr(sensor, "r", None), "forward_overlap", getattr(getattr(sensor, "req", None),
                                                                              "forward_overlap", 0.7)))
    return max(0.0, l_fp * (1.0 - fwd))


# ---------------------------------------------------------------------------
# Plan → 任務項目
# ---------------------------------------------------------------------------
def plan_to_mission_items(uav_plan: UAVPlan, ctx: MissionContext, cfg: PlannerConfig,
                          alt_frame: AltFrame = "relative", camera_trigger: bool = True,
                          wp_radius_m: float | None = None, *,
                          strip_sensors: Mapping[str, Sensor] | None = None,
                          z_tol_m: float | None = None,
                          takeoff_delay_s: float | None = None) -> list[MissionItem]:
    """`UAVPlan` → ArduPilot QuadPlane 任務項目（第 0 列 home）。

    對應：TAKEOFF → NAV_VTOL_TAKEOFF（alt = transition_start_agl）；TRANS_OUT 略；ACCEL／CLIMB／ENTRY／TRANSFER／RTB／
    DESCENT／DECEL → NAV_WAYPOINT（弧段外切離散、直線端點）；ORBIT 整圈 → NAV_LOITER_TO_ALT（param2 = ±R，順時針正）
    ／NAV_LOITER_TURNS／NAV_LOITER_TIME；空速改變 → DO_CHANGE_SPEED(airspeed)；OPERATE 條帶起／訖 →
    DO_SET_CAM_TRIGG_DIST(dist)／(0)（rgb）或 DO_SET_SERVO 開／關（lidar）。**DO_CHANGE_SPEED 與酬載開／關一律置於
    其所描述之「出發腿」起點 NAV 之後**（ArduPilot 於抵達前一個 NAV 時即執行其後之 DO_；`Waypoint.airspeed_mps`／
    `Waypoint.sensor_on` 描述該點之出發腿），與 `integration/mqtt_bridge` 之順序一致（D45）；
    第一個降落段航點前 → DO_LAND_START；
    TRANS_IN 略；LAND → NAV_VTOL_LAND（落點；alt 0 相對 home）。

    `alt_frame`：relative（alt = MSL − home MSL）、msl、terrain（alt = AGL）。`wp_radius_m` None → min(R_eff/2, 30)。
    `strip_sensors`（strip_id → Sensor，planner 之 decompose.resolve_sensor 結果）決定觸發距離；缺省依機隊主感測種類。
    `takeoff_delay_s` > 0 時於 home 列之後、NAV_VTOL_TAKEOFF 之前插入 MAV_CMD_NAV_DELAY(93)（param1 = 秒；
    param2–4 = −1 表「不使用時刻」），使多機任務保留計畫之錯開起飛時刻（D46）。
    """
    if alt_frame not in FRAME_OF:
        raise ValueError(f"alt_frame must be one of {tuple(FRAME_OF)}, got {alt_frame!r}")
    frame = FRAME_OF[alt_frame]
    spec = ctx.specs[uav_plan.cls]
    w_ref = ctx.wind.max_speed() if ctx.wind is not None else 0.0
    r_eff = spec.r_eff(w_ref)
    r_acc = accept_radius_m(r_eff, wp_radius_m)
    home_msl = _home_msl(ctx)
    home_ll = (ctx.inputs.home.lat_deg, ctx.inputs.home.lon_deg)
    items: list[MissionItem] = [_item(0, MAV_CMD_NAV_WAYPOINT, MAV_FRAME_GLOBAL, x=home_ll[0], y=home_ll[1],
                                      z=home_msl, current=1)]
    wps = uav_plan.waypoints
    if not wps:
        return items
    if takeoff_delay_s is not None and float(takeoff_delay_s) > 0.0:
        # NAV_DELAY：param1 = 延遲秒數；param2/3/4 = −1（不使用「當日時刻」形式）
        items.append(_item(0, MAV_CMD_NAV_DELAY, MAV_FRAME_GLOBAL_RELATIVE_ALT,
                           (float(takeoff_delay_s), -1.0, -1.0, -1.0)))
    pts = reduce_waypoints(wps, spec, cfg, r_acc, z_tol_m=z_tol_m)
    xy = np.array([[p.x, p.y] for p in pts], dtype=float)
    wgs = _wgs(ctx, xy, np.array([p.z for p in pts]))

    def alt_of(k: int) -> float:
        if alt_frame == "relative":
            return float(wgs[k, 2] - home_msl)
        if alt_frame == "msl":
            return float(wgs[k, 2])
        return float(pts[k].agl)

    v_cmd: float | None = None
    land_start_done = False
    cam_on = False
    lidar_on = False
    cur_strip: str | None = None
    trig_kind: str | None = None

    def open_strip(k: int) -> None:
        nonlocal cam_on, lidar_on, cur_strip, trig_kind
        p = pts[k]
        sensor = _sensor_for(uav_plan, ctx, p.strip_id, strip_sensors)
        # 規格 §6.2：同機雙感測（`platform.CombinedSensor`）一次飛行同時開兩個酬載 → 兩個 DO_ 皆送出。
        subs = tuple(getattr(sensor, "sensors", None) or ((sensor,) if sensor is not None else ()))
        cur_strip = p.strip_id
        trig_kind = getattr(sensor, "kind", None) if sensor is not None else None
        if not subs:
            items.append(_item(0, MAV_CMD_DO_SET_SERVO, MAV_FRAME_GLOBAL_RELATIVE_ALT,
                               (float(LIDAR_SERVO_CHANNEL), float(LIDAR_SERVO_PWM_ON), 0.0, 0.0)))
            lidar_on = True
            return
        for sub in subs:
            if getattr(sub, "kind", None) == "rgb":
                dist = trigger_distance_m(sub, p.agl)
                items.append(_item(0, MAV_CMD_DO_SET_CAM_TRIGG_DIST, MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                   (dist, 0.0, 1.0, 0.0)))
                cam_on = True
            else:   # lidar（或未知）→ 伺服佔位
                items.append(_item(0, MAV_CMD_DO_SET_SERVO, MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                   (float(LIDAR_SERVO_CHANNEL), float(LIDAR_SERVO_PWM_ON), 0.0, 0.0)))
                lidar_on = True

    def close_strip() -> None:
        nonlocal cam_on, lidar_on, cur_strip
        if cam_on:
            items.append(_item(0, MAV_CMD_DO_SET_CAM_TRIGG_DIST, MAV_FRAME_GLOBAL_RELATIVE_ALT, (0.0, 0.0, 0.0, 0.0)))
            cam_on = False
        if lidar_on:
            items.append(_item(0, MAV_CMD_DO_SET_SERVO, MAV_FRAME_GLOBAL_RELATIVE_ALT,
                               (float(LIDAR_SERVO_CHANNEL), float(LIDAR_SERVO_PWM_OFF), 0.0, 0.0)))
            lidar_on = False
        cur_strip = None

    for k, p in enumerate(pts):
        lat, lon = float(wgs[k, 0]), float(wgs[k, 1])
        if p.kind == "takeoff":
            h_ts = float(spec.transition_start_agl_m)
            if alt_frame == "terrain":
                z_to = h_ts
            elif alt_frame == "msl":
                z_to = float(wgs[k, 2]) + h_ts
            else:
                z_to = float(wgs[k, 2]) + h_ts - home_msl
            items.append(_item(0, MAV_CMD_NAV_VTOL_TAKEOFF, frame, (0.0, 0.0, 0.0, 0.0), lat, lon, z_to))
            continue
        if p.kind == "land":
            if camera_trigger and (cam_on or lidar_on):
                close_strip()
            if not land_start_done:
                items.append(_item(0, MAV_CMD_DO_LAND_START, MAV_FRAME_GLOBAL_RELATIVE_ALT))
                land_start_done = True
            # 落點高度 = 該站地面（LAND 航點本身位於 H_ts 之懸停下降起點，不可直接用其 z）
            site_msl = _site_msl(ctx, uav_plan.landing_site) if uav_plan.landing_site in ctx.sites_enu \
                else float(wgs[k, 2]) - float(spec.transition_start_agl_m)
            z_ld = 0.0 if alt_frame == "terrain" else (site_msl if alt_frame == "msl" else site_msl - home_msl)
            items.append(_item(0, MAV_CMD_NAV_VTOL_LAND, frame, (0.0, 0.0, 0.0, 0.0), lat, lon, z_ld))
            continue
        # ---- 條帶開／關（實際 append 在本點之 NAV 之後，見下）----
        want_open = bool(camera_trigger and p.sensor_on and p.phase is Phase.OPERATE and p.strip_id is not None
                         and cur_strip != p.strip_id)
        want_close = bool(camera_trigger and (cam_on or lidar_on) and (want_open or not (
            p.sensor_on and p.phase is Phase.OPERATE and p.strip_id == cur_strip)))
        # ---- 降落序列 ----
        if not land_start_done and p.phase in _LANDING_PHASES:
            items.append(_item(0, MAV_CMD_DO_LAND_START, MAV_FRAME_GLOBAL_RELATIVE_ALT))
            land_start_done = True
        # ---- 空速（描述本點之出發腿 ⇒ 與酬載開關同樣置於本點 NAV 之後，見下）----
        want_speed = bool(p.mode is Mode.CRUISE and p.v_air > 0 and (v_cmd is None or abs(p.v_air - v_cmd) > 0.05))
        # ---- 導航點 ----
        alt = alt_of(k)
        if p.kind == "loiter_alt":
            # NAV_LOITER_TO_ALT：ArduPlane 讀 param2 為半徑（帶號＝方向）
            items.append(_item(0, MAV_CMD_NAV_LOITER_TO_ALT, frame, (0.0, p.radius_signed, 0.0, 1.0), lat, lon, alt))
        elif p.kind == "loiter_turns":
            # NAV_LOITER_TURNS：ArduPlane 以 |param3| 取整（≤ 255 m）為半徑，param3 < 0 為逆時針
            items.append(_item(0, MAV_CMD_NAV_LOITER_TURNS, frame,
                               (float(p.laps), 0.0, _loiter_turns_radius(p.radius_signed), 1.0), lat, lon, alt))
        elif p.kind == "loiter_time":
            # NAV_LOITER_TIME：ArduPlane 只讀 param3 之「正負號」（方向），半徑一律用 WP_LOITER_RAD
            # ⇒ recommended_params 之 WP_LOITER_RAD 取本計畫實際盤旋半徑（見 orbit_radius_m）
            items.append(_item(0, MAV_CMD_NAV_LOITER_TIME, frame, (float(p.duration_s), 0.0, p.radius_signed, 1.0),
                               lat, lon, alt))
        else:
            items.append(_item(0, MAV_CMD_NAV_WAYPOINT, frame, (0.0, r_acc, 0.0, 0.0), lat, lon, alt))
        # ArduPilot 於「抵達前一個 NAV」時執行其後之 DO_ 指令 ⇒ 描述本點**出發腿**的指令必須放在本點 NAV **之後**
        if want_speed:
            items.append(_item(0, MAV_CMD_DO_CHANGE_SPEED, MAV_FRAME_GLOBAL_RELATIVE_ALT, (0.0, float(p.v_air), -1.0, 0.0)))
            v_cmd = float(p.v_air)
        if want_close:
            close_strip()
        if want_open:
            open_strip(k)
    if camera_trigger and (cam_on or lidar_on):
        close_strip()
    return renumber(items)


# ---------------------------------------------------------------------------
# QGC WPL 110
# ---------------------------------------------------------------------------
def _fmt(v: float) -> str:
    s = f"{v:.8f}".rstrip("0").rstrip(".")
    return s if s not in ("", "-0") else "0"


def write_qgc_wpl(items: Sequence[MissionItem], path: str | os.PathLike, comments: Sequence[str] = ()) -> str:
    """寫出 QGC WPL 110（12 欄 tab 分隔：seq current frame cmd p1 p2 p3 p4 lat lon alt autocontinue；lat/lon 7 位）。

    `comments` 於版本列之後以 `#` 開頭寫出（pymavlink `mavwp.MAVWPLoader` 與本模組 :func:`read_qgc_wpl` 皆會略過）。
    **一律轉為 ASCII**：`mavwp.MAVWPLoader.load` 以系統預設編碼（Windows 為 cp950）開檔，非 ASCII 註解會讓它
    丟 UnicodeDecodeError。
    """
    lines = [QGC_WPL_HEADER]
    for c in comments:
        for ln in str(c).splitlines():
            lines.append("# " + ln.encode("ascii", "replace").decode("ascii"))
    for it in items:
        lines.append("\t".join([str(int(it.seq)), str(int(it.current)), str(int(it.frame)), str(int(it.command)),
                                _fmt(it.param1), _fmt(it.param2), _fmt(it.param3), _fmt(it.param4),
                                f"{it.x:.7f}", f"{it.y:.7f}", f"{it.z:.6f}", str(int(it.autocontinue))]))
    text = "\n".join(lines) + "\n"
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    return str(path)


def read_qgc_wpl(path: str | os.PathLike, mission_type: int = 0) -> list[MissionItem]:
    """讀回 QGC WPL 110 → `list[MissionItem]`（格式錯誤丟 ValueError）。"""
    with open(path, "r", encoding="utf-8") as f:
        header = f.readline().strip()
        if header != QGC_WPL_HEADER:
            raise ValueError(f"unsupported waypoint file header {header!r}")
        out: list[MissionItem] = []
        for ln, line in enumerate(f, start=2):
            if not line.strip() or line.startswith("#"):
                continue
            a = line.split()
            if len(a) != 12:
                raise ValueError(f"line {ln}: expected 12 columns, got {len(a)}")
            out.append(MissionItem(int(a[0]), int(a[2]), int(a[3]), int(a[1]), int(a[11]), float(a[4]), float(a[5]),
                                   float(a[6]), float(a[7]), float(a[8]), float(a[9]), float(a[10]), int(mission_type)))
    return out


# ---------------------------------------------------------------------------
# Rally / Fence
# ---------------------------------------------------------------------------
def rally_items(inputs: MissionInputs, ctx: MissionContext, alt_frame: AltFrame = "relative") -> list[MissionItem]:
    """NAV_RALLY_POINT(5100)：home + 全部備降點；alt = 該站地面 + max_cls(H_tc,cmd)。

    ArduPilot 之 `RallyLocation` **沒有 MSL 框架**（高度為 int16「相對 home」，terrain 另以旗標表示），故
    `alt_frame='msl'` 亦輸出相對 home 高度並使用 MAV_FRAME_GLOBAL_RELATIVE_ALT；只有 `'terrain'` 用
    MAV_FRAME_GLOBAL_TERRAIN_ALT（AGL）。D47。
    """
    frame = FRAME_OF["terrain"] if alt_frame == "terrain" else MAV_FRAME_GLOBAL_RELATIVE_ALT
    home_msl = _home_msl(ctx)
    h_tc = max(sp.transition_cmd(transition_profile=ctx.cfg.transition_profile).h_tc_cmd_m for sp in ctx.specs.values()) \
        if ctx.specs else 100.0
    out: list[MissionItem] = []
    for site in (inputs.home, *inputs.alternates):
        if site.site_id not in ctx.sites_enu:
            continue
        msl = _site_msl(ctx, site.site_id)
        alt = h_tc if alt_frame == "terrain" else (msl + h_tc - home_msl)
        out.append(_item(len(out), MAV_CMD_NAV_RALLY_POINT, frame, x=site.lat_deg, y=site.lon_deg, z=alt,
                         mission_type=MAV_MISSION_TYPE_RALLY))
    return out


def _largest_polygon(g: shapely.Geometry) -> Polygon | None:
    if g is None or g.is_empty:
        return None
    if isinstance(g, Polygon):
        return g
    if isinstance(g, MultiPolygon):
        return max(g.geoms, key=lambda p: p.area)
    polys = [p for p in getattr(g, "geoms", []) if isinstance(p, Polygon)]
    return max(polys, key=lambda p: p.area) if polys else None


def _ring_latlon(ctx: MissionContext, poly: Polygon, simplify_m: float = 5.0) -> list[tuple[float, float]]:
    ring = shapely.Polygon(poly.exterior).simplify(simplify_m, preserve_topology=True)
    coords = np.asarray(ring.exterior.coords, dtype=float)[:-1]
    if len(coords) >= 3 and shapely.LinearRing(coords).is_ccw is False:
        coords = coords[::-1]
    ll = _wgs(ctx, coords[:, :2], 0.0)
    return [(float(a), float(b)) for a, b in ll[:, :2]]


def fence_buffer_m(ctx: MissionContext, cls: str) -> float:
    """inclusion 外擴量 = max(2·R_eff, d_turn(s→0) = R_eff(1+√3) + e_ext,max)（D38：Ω 轉場超出端線量）。"""
    spec = ctx.specs[cls]
    w_ref = ctx.wind.max_speed() if ctx.wind is not None else 0.0
    r_eff = spec.r_eff(w_ref)
    v_g_max = spec.cruise_airspeed_mps + w_ref
    e_ext = 0.0
    for (c, _rid), sensor in ctx.sensors.items():
        if c == cls:
            try:
                e_ext = max(e_ext, float(sensor.end_extension_m(v_g_max, ctx.cfg.t_settle_s)))
            except Exception:  # noqa: BLE001
                pass
    if ctx.cfg.e_ext_override_m is not None:
        e_ext = float(ctx.cfg.e_ext_override_m)
    return max(2.0 * r_eff, d_turn_from_r(r_eff, 0.0, e_ext))


def fence_polygons(inputs: MissionInputs, ctx: MissionContext, cls: str,
                   plans: Iterable[UAVPlan] | None = None) -> tuple[Polygon | None, list[Polygon]]:
    """(inclusion, exclusions)（ENU）：inclusion = ConvexHull(區域 ∪ 站點 ⊕ 起降走廊 ∪ 航線 ⊕ b_trk) ⊕ max(2·R_eff, d_turn)
    ∩ P_link,c；exclusion = 有效禁飛區（未加 b_trk；航線已由規劃器保持 ⊕ b_trk 淨空）。"""
    spec = ctx.specs[cls]
    w_ref = ctx.wind.max_speed() if ctx.wind is not None else 0.0
    r_eff = spec.r_eff(w_ref)
    tc = spec.transition_cmd(transition_profile=ctx.cfg.transition_profile)
    d_corr = tc.d_tr_m + tc.d_acc_m + 2.0 * r_eff          # 起降走廊（轉換＋加速段＋進入／下降迴圈）可及範圍
    geoms: list[shapely.Geometry] = list(ctx.areas_enu.values())
    geoms += [shapely.Point(float(xy[0]), float(xy[1])).buffer(d_corr, quad_segs=8) for xy, _z in ctx.sites_enu.values()]
    for up in (plans or []):
        if up.cls == cls and len(up.waypoints) >= 2:
            geoms.append(shapely.LineString([(w.x_m, w.y_m) for w in up.waypoints]).buffer(ctx.cfg.b_trk_m, quad_segs=4))
    hull = unary_union(geoms).convex_hull.buffer(fence_buffer_m(ctx, cls), quad_segs=16)
    link = ctx.link_poly.get(cls) if ctx.link_poly else None
    incl = hull if (link is None or link.is_empty) else hull.intersection(link)
    inclusion = _largest_polygon(incl)
    excl: list[Polygon] = []
    for nf in inputs.airspace.active_no_fly(ctx.t_window) if ctx.cfg.enforce_time_windows else inputs.airspace.no_fly:
        ring = np.asarray(nf.polygon_wgs84, dtype=float)
        xy = ctx.frame.to_enu(ring[:, 0], ring[:, 1])[:, :2]
        p = Polygon(xy)
        if p.is_valid and not p.is_empty:
            excl.append(p)
    return inclusion, excl


def fence_items(inputs: MissionInputs, ctx: MissionContext, cls: str,
                plans: Iterable[UAVPlan] | None = None) -> list[MissionItem]:
    """FENCE 項目：NAV_FENCE_RETURN_POINT(5000)=home → inclusion 頂點 5001（param1=頂點數） → exclusion 5002。
    `plans`（該構型之 UAVPlan）給定時 inclusion 亦涵蓋航線 ⊕ b_trk。"""
    inclusion, exclusions = fence_polygons(inputs, ctx, cls, plans)
    out: list[MissionItem] = [_item(0, MAV_CMD_NAV_FENCE_RETURN_POINT, MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                    x=inputs.home.lat_deg, y=inputs.home.lon_deg, z=0.0,
                                    mission_type=MAV_MISSION_TYPE_FENCE)]
    if inclusion is not None:
        ll = _ring_latlon(ctx, inclusion)
        for lat, lon in ll:
            out.append(_item(len(out), MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, MAV_FRAME_GLOBAL_RELATIVE_ALT,
                             (float(len(ll)), 0.0, 0.0, 0.0), lat, lon, 0.0, mission_type=MAV_MISSION_TYPE_FENCE))
    for poly in exclusions:
        ll = _ring_latlon(ctx, poly, simplify_m=0.5)
        for lat, lon in ll:
            out.append(_item(len(out), MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, MAV_FRAME_GLOBAL_RELATIVE_ALT,
                             (float(len(ll)), 0.0, 0.0, 0.0), lat, lon, 0.0, mission_type=MAV_MISSION_TYPE_FENCE))
    return out


# ---------------------------------------------------------------------------
# 參數建議
# ---------------------------------------------------------------------------
PARAM_COMMENTS: dict[str, str] = {
    "AIRSPEED_CRUISE": "cruise_airspeed_mps (Table 1) — TECS target airspeed in AUTO [m/s] (old TRIM_ARSPD_CM cm/s)",
    "AIRSPEED_MIN": "v_min_mission (min_mission_airspeed_mps upper bound) [m/s] (old ARSPD_FBW_MIN)",
    "AIRSPEED_MAX": "max_airspeed_mps; provisional when Table 1 value is None (1.4*cruise) [m/s] (old ARSPD_FBW_MAX)",
    "AIRSPEED_STALL": "stall_airspeed_mps [m/s] (ArduPlane 4.5+)",
    "TECS_CLMB_MAX": "max_fw_climb_rate_mps = Va*sin(planning_climb_angle) [m/s]",
    "TECS_SINK_MAX": "max_fw_descent_rate_mps = Va*sin(planning_descent_angle) [m/s]",
    "TECS_SINK_MIN": "minimum sink rate at cruise (0.9*planning descent rate) [m/s]",
    "ROLL_LIMIT_DEG": "max_bank_angle_deg [deg] (old LIM_ROLL_CD centideg)",
    "WP_LOITER_RAD": "loiter radius [m]; positive = clockwise. ArduPlane flies NAV_LOITER_TIME/TURNS at THIS radius "
                     "(NAV_LOITER_TIME param3 only carries the direction sign), so it is set to the radius the "
                     "planner used for the delay orbits when the mission has any, else R_eff (wind-inflated)",
    "WP_RADIUS": "waypoint acceptance radius [m]; ArduPlane uses this for EVERY waypoint (NAV_WAYPOINT param2 is "
                 "ignored), so it is kept below 0.8x the smallest consecutive NAV spacing of this mission "
                 "(cap r_acc = min(R_eff/2, 30), floor 10) so a waypoint is never 'reached' together with its "
                 "predecessor and its attached DO_ commands fire on position",
    "RTL_RADIUS": "RTL loiter radius = R_eff [m]",
    "Q_ENABLE": "QuadPlane enabled (lift-and-cruise 4+1)",
    "Q_TRANSITION_MS": "forward transition timeout = t_tr_out_s * 1000 [ms]",
    "Q_ASSIST_SPEED": "VTOL assist below this airspeed [m/s]; MUST stay below AIRSPEED_MIN (else the lift motors "
                      "spin up whenever TECS regulates down to its minimum) and above stall: "
                      "min(v_tr, AIRSPEED_MIN - 1) floored at 1.15*AIRSPEED_STALL",
    "Q_RTL_ALT": "QRTL approach altitude = min_fw_operation_agl_m [m AGL]",
    "Q_RTL_MODE": "1 = an RTL becomes QRTL within RTL_RADIUS of home (a QuadPlane RTL otherwise ends in a "
                  "fixed-wing loiter over home). Required for the RTL selected by FENCE_ACTION / FS_LONG_ACTN / "
                  "BATT_FS_LOW_ACT to end in a VTOL landing",
    "Q_LAND_FINAL_ALT": "final vertical descent phase altitude (10 m rule of hover_descent_time_s) [m]",
    "RTL_AUTOLAND": "2 = go directly to the landing sequence at DO_LAND_START (every exported mission contains one; "
                    "with the default 0 the sequence is never used on RTL and arming is refused)",
    "FENCE_ENABLE": "geofence enabled",
    "FENCE_TYPE": "bitmask: 1 = max alt, 4 = polygon (inclusion/exclusion)",
    "FENCE_ALT_MAX": "max altitude fence [m ABOVE HOME] (AC_Fence compares relative alt, NOT AGL): "
                     "max(plan alt_msl - home_msl) + b_v, at least the airspace ceiling h_ceil",
    "FENCE_ACTION": "1 = RTL on breach",
    "FENCE_RET_RALLY": "1 = return to the nearest rally point (this export ships a rally set) instead of the fence "
                       "return point",
    "BATT_CAPACITY": "pack nominal capacity = nominal_wh / pack_voltage * 1000 [mAh]",
    "BATT_LOW_MAH": "low battery threshold = (reserve + e_LDG) in mAh — planner reserve r*E_nom plus landing energy",
    "BATT_CRT_MAH": "critical battery threshold = reserve r*E_nom in mAh",
    "BATT_FS_LOW_ACT": "low battery action, ArduPLANE enum (0:None 1:RTL 2:Land 3:Terminate 4:QLand 6:Loiter to "
                       "QLand — NOT the Copter enum): 1 = RTL. Needs BATT_LOW_MAH (or BATT_LOW_VOLT) to trigger",
    "BATT_FS_CRT_ACT": "critical battery action, ArduPlane enum: 4 = QLand (3 is Terminate — flight termination). "
                       "Needs BATT_CRT_MAH (or BATT_CRT_VOLT) to trigger",
    "FS_LONG_ACTN": "long GCS failsafe action from lost_link_policy: 0 = continue (CONTINUE_THEN_RTB), 1 = RTL (RTB/NEAREST_ALT)",
    "FS_LONG_TIMEOUT": "long failsafe timeout = c2.t_ll_s [s]",
    "TERRAIN_ENABLE": "1 when waypoints use MAV_FRAME_GLOBAL_TERRAIN_ALT (alt_frame='terrain')",
    "TERRAIN_FOLLOW": "terrain following in auto modes (1 with terrain frame)",
    "CAM1_TYPE": "1 = servo camera backend (ArduPlane 4.5+; pre-4.5 use CAM_TRIGG_TYPE=0). Without a camera "
                 "backend the mission's DO_SET_CAM_TRIGG_DIST(206) items are a no-op",
    "CAM1_DURATION": "camera trigger pulse duration [0.1 s units]",
    f"SERVO{CAMERA_SERVO_CHANNEL}_FUNCTION": "10 = CameraTrigger — the output the camera backend pulses. "
                                             "PLACEHOLDER channel; match the payload wiring",
    f"SERVO{LIDAR_SERVO_CHANNEL}_FUNCTION": "0 = Disabled, i.e. available to mission DO_SET_SERVO(183). Without "
                                            "this ArduPilot refuses the LiDAR record on/off items with "
                                            "'Channel N is already in use'. PLACEHOLDER channel; match the wiring",
}


def recommended_params(spec: PlatformSpec, model: EnergyModel | None, cfg: PlannerConfig,
                       uav_plan: UAVPlan | None = None, *, pack_voltage_v: float | None = None,
                       wind_speed_mps: float = 0.0, h_ceil_agl_m: float | None = None,
                       lost_link_policy: str = "RTB", alt_frame: AltFrame = "relative",
                       t_ll_s: float = 60.0, home_msl_m: float | None = None,
                       min_nav_spacing_m: float | None = None, loiter_radius_m: float | None = None,
                       sensor_kinds: Iterable[str] = (), warnings: list[str] | None = None) -> dict[str, float]:
    """表 1 → ArduPilot 參數建議（ardupilot_notes §3）。回傳 {參數名: 值}；每鍵之說明見 :data:`PARAM_COMMENTS`。

    BATT_CAPACITY／BATT_LOW_MAH／BATT_CRT_MAH 只在 `pack_voltage_v` 已知（SortieEnergy.pack_voltage_nom_v）且模型非
    基準分鐘模式時輸出；無法輸出門檻時 BATT_FS_*_ACT 仍輸出（動作正確才不會誤觸發終止），但於 `warnings` 記一筆，
    由操作者於機上設定門檻。

    * `home_msl_m` + `uav_plan`：FENCE_ALT_MAX 依 **相對 home 高度**（AC_Fence 之判準）計算，而非 AGL（D48）。
    * `min_nav_spacing_m`（:func:`min_nav_spacing_m`）：WP_RADIUS 取 0.8×該值（夾在 [10, r_acc]）。
    * `loiter_radius_m`（:func:`orbit_radius_m`）：WP_LOITER_RAD 取本任務實際盤旋半徑。
    * `sensor_kinds`：本構型之感測種類（'rgb'／'lidar'）→ 輸出使 DO_SET_CAM_TRIGG_DIST／DO_SET_SERVO 生效之酬載參數。
    """
    warn = warnings if warnings is not None else []
    va = float(spec.cruise_airspeed_mps)
    r_eff = spec.r_eff(wind_speed_mps)
    v_max = float(spec.max_airspeed_mps) if spec.max_airspeed_mps is not None else round(1.4 * va, 1)
    tc = spec.transition_cmd(transition_profile=cfg.transition_profile)
    v_min = float(spec.v_min_mission())
    # Q_ASSIST_SPEED 必須 < AIRSPEED_MIN（否則巡航中 TECS 一降到下限就啟動升力馬達）且 > 失速
    q_assist = min(float(tc.v_tr_mps), v_min - 1.0)
    q_assist = max(q_assist, 1.15 * float(spec.stall_airspeed_mps))
    if q_assist >= v_min:                                   # 極窄之速度窗（理論上不會發生）
        q_assist = 0.5 * (float(spec.stall_airspeed_mps) + v_min)
        warn.append("Q_ASSIST_SPEED squeezed between AIRSPEED_STALL and AIRSPEED_MIN; verify on the vehicle")
    r_acc = accept_radius_m(r_eff)
    if min_nav_spacing_m is not None and math.isfinite(min_nav_spacing_m) and min_nav_spacing_m > 0:
        wp_rad = max(10.0, min(r_acc, math.floor(0.8 * float(min_nav_spacing_m))))
        if wp_rad >= float(min_nav_spacing_m):
            warn.append(f"WP_RADIUS {wp_rad:.0f} m >= min NAV spacing {float(min_nav_spacing_m):.1f} m; "
                        "consecutive waypoints may be consumed in one cycle")
    else:
        wp_rad = r_acc
    p: dict[str, float] = {
        "AIRSPEED_CRUISE": va,
        "AIRSPEED_MIN": v_min,
        "AIRSPEED_MAX": v_max,
        "AIRSPEED_STALL": float(spec.stall_airspeed_mps),
        "TECS_CLMB_MAX": float(spec.max_fw_climb_rate_mps),
        "TECS_SINK_MAX": float(spec.max_fw_descent_rate_mps),
        "TECS_SINK_MIN": round(0.9 * va * math.sin(math.radians(spec.planning_descent_angle_deg)), 2),
        "ROLL_LIMIT_DEG": float(spec.max_bank_angle_deg),
        "WP_LOITER_RAD": float(round(loiter_radius_m if loiter_radius_m else r_eff, 1)),
        "WP_RADIUS": float(round(wp_rad, 1)),
        "RTL_RADIUS": float(round(r_eff, 1)),
        "Q_ENABLE": 1.0,
        "Q_TRANSITION_MS": float(int(round(spec.t_tr_out_s * 1000.0))),
        "Q_ASSIST_SPEED": float(round(q_assist, 1)),
        "Q_RTL_ALT": float(spec.min_fw_operation_agl_m),
        "Q_RTL_MODE": 1.0,
        "Q_LAND_FINAL_ALT": 10.0,
        "RTL_AUTOLAND": 2.0,
        "FENCE_ENABLE": 1.0,
        "FENCE_TYPE": 5.0,
        "FENCE_ACTION": 1.0,
        "FENCE_RET_RALLY": 1.0,
        "FS_LONG_ACTN": 0.0 if lost_link_policy == "CONTINUE_THEN_RTB" else 1.0,
        "FS_LONG_TIMEOUT": float(t_ll_s),
        "TERRAIN_ENABLE": 1.0 if alt_frame == "terrain" else 0.0,
        "TERRAIN_FOLLOW": 1.0 if alt_frame == "terrain" else 0.0,
        # ArduPlane 之列舉（非 Copter）：1 = RTL、4 = QLand（3 是 Terminate）
        "BATT_FS_LOW_ACT": 1.0,
        "BATT_FS_CRT_ACT": 4.0,
    }
    # ---- 酬載參數（沒有這些，任務中的 DO_SET_CAM_TRIGG_DIST／DO_SET_SERVO 不會有任何作用）----
    kinds = {str(k) for k in sensor_kinds}
    if "rgb" in kinds:
        p["CAM1_TYPE"] = 1.0                               # servo backend（4.5+；4.4 以前為 CAM_TRIGG_TYPE=0）
        p["CAM1_DURATION"] = 1.0
        p[f"SERVO{CAMERA_SERVO_CHANNEL}_FUNCTION"] = 10.0  # CameraTrigger
    if "lidar" in kinds or not kinds:
        p[f"SERVO{LIDAR_SERVO_CHANNEL}_FUNCTION"] = 0.0    # Disabled ⇒ 開放給任務 DO_SET_SERVO
    # ---- FENCE_ALT_MAX：AC_Fence 比對「相對 home 高度」，不是 AGL ----
    h_ceil = float(h_ceil_agl_m) if h_ceil_agl_m is not None and math.isfinite(h_ceil_agl_m) else None
    rel_max: float | None = None
    if uav_plan is not None and uav_plan.waypoints:
        if home_msl_m is not None and math.isfinite(home_msl_m):
            rel_max = max(float(w.alt_msl_m) for w in uav_plan.waypoints) - float(home_msl_m) + cfg.b_v_m
        else:                                              # 無 home MSL：退回 AGL（平地才等價）並警告
            rel_max = max(float(w.agl_m) for w in uav_plan.waypoints) + cfg.b_v_m
            warn.append("FENCE_ALT_MAX derived from AGL because home_msl_m was not supplied; over terrain higher "
                        "than the launch site the fence can trip mid-mission")
    if rel_max is not None:
        h_ceil = rel_max if h_ceil is None else max(h_ceil, rel_max)
    if h_ceil is None:
        h_ceil = float(spec.min_fw_operation_agl_m) + 100.0
    p["FENCE_ALT_MAX"] = float(math.ceil(h_ceil))
    if model is not None and pack_voltage_v is not None and pack_voltage_v > 0 and not model.base_minute_mode \
            and model.nominal_wh is not None:
        wh2mah = 1000.0 / float(pack_voltage_v)
        p["BATT_CAPACITY"] = float(round(float(model.nominal_wh) * wh2mah))
        p["BATT_CRT_MAH"] = float(round(model.reserve_wh() * wh2mah))
        p["BATT_LOW_MAH"] = float(round((model.reserve_wh() + model.e_landing_wh()) * wh2mah))
    else:
        warn.append("BATT_LOW_MAH/BATT_CRT_MAH not exported (pack voltage or energy model unknown): "
                    "BATT_FS_LOW_ACT/BATT_FS_CRT_ACT have no threshold until you set BATT_LOW_MAH/BATT_CRT_MAH "
                    "(or BATT_LOW_VOLT/BATT_CRT_VOLT) on the vehicle")
    return p


def write_param_file(params: Mapping[str, float], path: str | os.PathLike, comments: Mapping[str, str] | None = None,
                     warnings: Sequence[str] = ()) -> str:
    """Mission Planner `.param` 格式：每列 `NAME,VALUE`；`#` 開頭為註解（含每鍵說明與 `warnings`）。"""
    cm = PARAM_COMMENTS if comments is None else comments
    lines = ["# HV-DCCPP recommended ArduPilot (ArduPlane/QuadPlane) parameters",
             "# Verify against the vehicle firmware version before upload (names are ArduPlane 4.5+)."]
    for w in warnings:
        lines.append(f"# WARNING: {w}")
    for k in sorted(params):
        v = params[k]
        if k in cm:
            lines.append(f"# {k}: {cm[k]}")
        vs = str(int(v)) if float(v).is_integer() else f"{float(v):.6g}"
        lines.append(f"{k},{vs}")
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    return str(path)


def read_param_file(path: str | os.PathLike) -> dict[str, float]:
    """讀回 `.param`（`NAME,VALUE` 或 `NAME VALUE`；忽略 `#` 註解）。"""
    out: dict[str, float] = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            parts = s.replace(",", " ").replace("\t", " ").split()
            if len(parts) < 2:
                raise ValueError(f"bad param line: {line!r}")
            out[parts[0]] = float(parts[1])
    return out


# ---------------------------------------------------------------------------
# QGC .plan JSON
# ---------------------------------------------------------------------------
def _fence_groups(fence: Sequence[MissionItem]) -> list[dict[str, Any]]:
    polys: list[dict[str, Any]] = []
    cur: list[list[float]] = []
    cur_cmd: int | None = None
    for it in fence:
        if it.command in (MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION):
            if cur_cmd is not None and (it.command != cur_cmd or len(cur) >= int(it.param1)):
                polys.append({"inclusion": cur_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, "polygon": cur,
                              "version": 1})
                cur = []
            cur_cmd = it.command
            cur.append([it.x, it.y])
        else:
            if cur:
                polys.append({"inclusion": cur_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, "polygon": cur,
                              "version": 1})
                cur, cur_cmd = [], None
    if cur:
        polys.append({"inclusion": cur_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, "polygon": cur, "version": 1})
    return polys


def to_qgc_plan_json(items: Sequence[MissionItem], rally: Sequence[MissionItem] | None,
                     fence: Sequence[MissionItem] | None, path: str | os.PathLike | None = None,
                     cruise_speed_mps: float | None = None) -> dict[str, Any]:
    """QGC `.plan` v1 JSON（vehicleType 2 固定翼、firmwareType 3 ArduPilot）；`path` 給定時亦寫檔。"""
    if not items:
        raise ValueError("items must contain the home row")
    home = items[0]
    mission_items: list[dict[str, Any]] = []
    for it in items[1:]:
        # QGC (>= 4.3) AltitudeMode：0 None、1 Relative、2 Absolute、3 CalcAboveTerrain（由地形資料算 AMSL）、
        # 4 TerrainFrame（= MAV_FRAME_GLOBAL_TERRAIN_ALT）。
        alt_mode = {MAV_FRAME_GLOBAL_RELATIVE_ALT: 1, MAV_FRAME_GLOBAL: 2, MAV_FRAME_GLOBAL_TERRAIN_ALT: 4}.get(it.frame, 1)
        mission_items.append({
            "type": "SimpleItem", "autoContinue": bool(it.autocontinue), "command": int(it.command),
            "doJumpId": int(it.seq), "frame": int(it.frame),
            "params": [it.param1, it.param2, it.param3, it.param4, it.x, it.y, it.z],
            "AMSLAltAboveTerrain": None, "Altitude": it.z, "AltitudeMode": alt_mode,
        })
    if cruise_speed_mps is None:
        sp = [it.param2 for it in items if it.command == MAV_CMD_DO_CHANGE_SPEED and it.param2 > 0]
        cruise_speed_mps = float(sp[0]) if sp else 20.0
    plan = {
        "fileType": "Plan", "version": 1, "groundStation": "QGroundControl",
        "mission": {"cruiseSpeed": float(cruise_speed_mps), "hoverSpeed": 5.0, "firmwareType": _QGC_FIRMWARE_ARDUPILOT,
                    "vehicleType": _QGC_VEHICLE_FIXED_WING, "version": 2,
                    "plannedHomePosition": [home.x, home.y, home.z], "items": mission_items},
        "geoFence": {"version": 2, "circles": [], "polygons": _fence_groups(fence or [])},
        "rallyPoints": {"version": 2, "points": [[r.x, r.y, r.z] for r in (rally or [])
                                                 if r.command == MAV_CMD_NAV_RALLY_POINT]},
    }
    if path is not None:
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            json.dump(plan, f, indent=2)
    return plan


# ---------------------------------------------------------------------------
# 一鍵匯出
# ---------------------------------------------------------------------------
def export_all(plan: MissionPlan, ctx: MissionContext, cfg: PlannerConfig, out_dir: str | os.PathLike, *,
               alt_frame: AltFrame = "relative", camera_trigger: bool = True, wp_radius_m: float | None = None,
               strip_sensors: Mapping[str, Sensor] | None = None, write_plan_json: bool = True,
               takeoff_delay: Literal["operator", "nav_delay"] = "operator", t0_s: float | None = None
               ) -> dict[str, Any]:
    """每機寫出 `<uav_id>.waypoints`、`<uav_id>_rally.txt`、`<uav_id>_fence.txt`、`<uav_id>.param`
    （及 `<uav_id>.plan` QGC JSON），加 `mission_summary.json`；回傳 summary dict。

    **起飛時刻**（D46）：計畫之 `t_takeoff_s` 是 4D 去衝突與走廊排程的結果，任務檔本身不帶時間原點，若同時 AUTO
    所有機，同墊 60 s 間隔與全部去衝突延遲都會失效。`takeoff_delay`：

    * ``'operator'``（預設，與 `mavlink_link.FleetLink` 之「由地面站依 t_takeoff 逐機 MISSION_START」一致）：
      不插入任何項目，但把 `t0 + Δt` 寫進 `<uav>.waypoints` 的檔頭註解與 `mission_summary.json` 之 `takeoff` 區塊；
    * ``'nav_delay'``：於 NAV_VTOL_TAKEOFF 之前插入 MAV_CMD_NAV_DELAY(93)（param1 = t_takeoff − t0 秒），
      任務自帶錯開；此時**不可**再由地面站延後 MISSION_START（會重複延遲）。

    `t0_s` 預設為全隊最小之 `t_takeoff_s`。
    """
    out_dir = os.fspath(out_dir)
    os.makedirs(out_dir, exist_ok=True)
    if takeoff_delay not in ("operator", "nav_delay"):
        raise ValueError(f"takeoff_delay must be 'operator' or 'nav_delay', got {takeoff_delay!r}")
    inputs = ctx.inputs
    home_msl = _home_msl(ctx)
    rally = rally_items(inputs, ctx, alt_frame)
    fences: dict[str, list[MissionItem]] = {}
    t0 = float(t0_s) if t0_s is not None else (min(float(u.t_takeoff_s) for u in plan.plans.values())
                                               if plan.plans else 0.0)
    delays = {uid: max(0.0, float(up.t_takeoff_s) - t0) for uid, up in plan.plans.items()}
    summary: dict[str, Any] = {
        "home": {"site_id": inputs.home.site_id, "lat_deg": inputs.home.lat_deg, "lon_deg": inputs.home.lon_deg,
                 "alt_msl_m": home_msl},
        "alt_frame": alt_frame, "makespan_s": float(plan.makespan_s), "feasibility": plan.feasibility.status,
        "uncovered": [list(u) for u in plan.uncovered],
        "takeoff": {"mode": takeoff_delay, "t0_s": t0, "spread_s": (max(delays.values()) if delays else 0.0),
                    "delay_s": {u: round(d, 1) for u, d in delays.items()},
                    "note": ("每機任務第 1 列為 NAV_DELAY(93)，Δt 已內含於任務；地面站請一次 MISSION_START，勿再延後。"
                             if takeoff_delay == "nav_delay" else
                             "任務不含時間原點：地面站須於 t0 + delay_s[uav] 才對該機 MISSION_START"
                             "（規格 13.5 同墊 60 s 間隔與 4D 去衝突延遲全靠此）。")},
        "uavs": {},
    }
    for uid, up in plan.plans.items():
        cls = up.cls
        spec = ctx.specs[cls]
        uav = ctx.uav(uid)
        model = ctx.models.get(uid) or ctx.models_rep.get(cls)
        items = plan_to_mission_items(up, ctx, cfg, alt_frame, camera_trigger, wp_radius_m, strip_sensors=strip_sensors,
                                      takeoff_delay_s=delays[uid] if takeoff_delay == "nav_delay" else None)
        if cls not in fences:
            fences[cls] = fence_items(inputs, ctx, cls, [q for q in plan.plans.values() if q.cls == cls])
        fence = fences[cls]
        w_ref = ctx.wind.max_speed() if ctx.wind is not None else 0.0
        kinds = {getattr(s, "kind", None) for (c, _r), s in ctx.sensors.items() if c == cls}
        if strip_sensors:
            kinds |= {getattr(strip_sensors[s], "kind", None) for s, _d in up.strips if s in strip_sensors}
        p_warn: list[str] = []
        params = recommended_params(spec, model, cfg, up, pack_voltage_v=uav.sortie.pack_voltage_nom_v,
                                    wind_speed_mps=w_ref, h_ceil_agl_m=ctx.h_ceil_m.get(cls),
                                    lost_link_policy=inputs.c2.policy_for(uid, cls), alt_frame=alt_frame,
                                    t_ll_s=inputs.c2.t_ll_s, home_msl_m=home_msl,
                                    min_nav_spacing_m=min_nav_spacing_m(items, ctx),
                                    loiter_radius_m=orbit_radius_m(items),
                                    sensor_kinds=[k for k in kinds if k], warnings=p_warn)
        files = {
            "waypoints": os.path.join(out_dir, f"{uid}.waypoints"),
            "rally": os.path.join(out_dir, f"{uid}_rally.txt"),
            "fence": os.path.join(out_dir, f"{uid}_fence.txt"),
            "param": os.path.join(out_dir, f"{uid}.param"),
        }
        wp_comments = [f"HV-DCCPP {uid} ({cls}); plan t_takeoff = {float(up.t_takeoff_s):.1f} s, t_land = "
                       f"{float(up.t_land_s):.1f} s (mission time origin t0 = {t0:.1f} s)"]
        if takeoff_delay == "nav_delay":
            wp_comments.append(f"seq 1 = NAV_DELAY {delays[uid]:.1f} s: start the mission once, do NOT delay "
                               f"MISSION_START as well")
        else:
            wp_comments.append(f"START THIS MISSION AT t0 + {delays[uid]:.1f} s. The mission carries no time origin; "
                               f"starting all vehicles together voids the 60 s pad spacing and every 4D "
                               f"deconfliction delay.")
        write_qgc_wpl(items, files["waypoints"], wp_comments)
        write_qgc_wpl(rally, files["rally"])
        write_qgc_wpl(fence, files["fence"])
        write_param_file(params, files["param"], warnings=p_warn)
        if write_plan_json:
            files["plan"] = os.path.join(out_dir, f"{uid}.plan")
            to_qgc_plan_json(items, rally, fence, files["plan"], cruise_speed_mps=spec.cruise_airspeed_mps)
        summary["uavs"][uid] = {
            "cls": cls, "n_items": len(items), "n_nav": sum(1 for it in items if it.is_nav),
            "n_waypoints_in": len(up.waypoints), "strips": [[s, int(d)] for s, d in up.strips],
            "t_takeoff_s": float(up.t_takeoff_s), "t_arr_ptc_s": float(up.t_arr_ptc_s), "t_land_s": float(up.t_land_s),
            "landing_site": up.landing_site, "energy_used_wh": float(up.energy_used_wh),
            "energy_margin_wh": float(up.energy_margin_wh), "distance_m": float(up.distance_m),
            "orbits_inserted": int(up.orbits_inserted), "warnings": list(up.warnings),
            "takeoff_delay_s": round(delays[uid], 1), "param_warnings": p_warn,
            "params": params, "files": files,
        }
    with open(os.path.join(out_dir, "mission_summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    summary["files"] = {"summary": os.path.join(out_dir, "mission_summary.json")}
    return summary


__all__ = [
    "MissionItem", "plan_to_mission_items", "reduce_waypoints", "arc_vertices", "accept_radius_m", "trigger_distance_m",
    "write_qgc_wpl", "read_qgc_wpl", "rally_items", "fence_items", "fence_polygons", "fence_buffer_m", "recommended_params",
    "write_param_file", "read_param_file", "to_qgc_plan_json", "export_all", "renumber", "PARAM_COMMENTS",
    "NAV_COMMANDS", "FRAME_OF", "LIDAR_SERVO_CHANNEL", "LIDAR_SERVO_PWM_ON", "LIDAR_SERVO_PWM_OFF",
    "CAMERA_SERVO_CHANNEL", "LOITER_TURNS_MAX_RADIUS_M", "orbit_radius_m", "min_nav_spacing_m", "HAVE_PYMAVLINK",
]
