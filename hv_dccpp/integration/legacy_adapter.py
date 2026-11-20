"""legacy_adapter.py — HV-DCCPP 對使用者既有 Tkinter 規劃器的替換層（integration_notes.md §注入點）。

用途
----
使用者既有系統（``drone_path_planner_A--main`` 等）在 ``main_app.py`` 以

    waypoint_lines, waypoints = self.waypoint_generator.generate_complete_mission(
        region_corners, params, idx, sub_count, start_from_left, loiter_time)

呼叫 boustrophedon 產生器，並穿透存取 ``self.waypoint_generator.collision_avoidance.calculate_loiter_delay(...)``。
本模組之 :class:`HVDCCPPGeneratorAdapter` 提供**完全相同的簽章**，內部改為：

1. 以角點 ``(lat, lon)``、`params`（duck typing：``altitude / angle / spacing / speed / safety_distance``）組出
   :class:`hv_dccpp.mission.MissionInputs`（一個區域、一架 UAV、平地或使用者提供之 DEM）；
2. 呼叫 :func:`hv_dccpp.planner.plan_mission`（完整 HV-DCCPP：高度包絡、Dubins 轉場、能量、起降走廊）；
3. 以 :func:`plan_to_mission_items` 轉為 ArduPilot QuadPlane 任務項目（``NAV_VTOL_TAKEOFF`` 84、``DO_CHANGE_SPEED`` 178、
   ``NAV_WAYPOINT`` 16、``NAV_LOITER_TO_ALT`` 31、``DO_SET_CAM_TRIGG_DIST`` 206、``DO_LAND_START`` 189、``NAV_VTOL_LAND`` 85），
   序列化為 QGC WPL 110（tab 分隔 12 欄，第 0 列 home：frame 0、cmd 16、current 1）。
   **不輸出** ``CONDITION_YAW`` 115（固定翼段忽略）、亦不在任務中段插入 ``NAV_TAKEOFF``（既有系統之已知缺陷）。

高度：任務項目一律 ``MAV_FRAME_GLOBAL_RELATIVE_ALT``（alt = MSL − home MSL；ardupilot_notes §1）。
指令常數一律取自 ``pymavlink.mavutil.mavlink.MAV_CMD_*``（pymavlink 缺席時退回同值常數，見 :data:`MAV`）。

契約備註：契約 v1.1 未定義本模組；``hv_dccpp/ardupilot.py``（Plan → 任務項目）尚不存在，故 Plan → 任務項目之轉換以
本模組之 :func:`plan_to_mission_items` 私有實作（記於 CONTRACT_DEVIATIONS.md）。
"""
from __future__ import annotations

import contextlib
import dataclasses
import math
import os
from dataclasses import dataclass, replace
from typing import Any, Callable, Iterator, Literal, Mapping, Sequence

import numpy as np
import shapely
from shapely.geometry import MultiPolygon, Polygon, box

from ..geo import DemGrid, GeoFrame
from ..geometry import convex_partition
from ..mission import (AreaSpec, Airspace, C2Link, MissionInputError, MissionInputs, MissionPlan, PayloadRequirement,
                       Phase, PlannerConfig, Site, SortieEnergy, UAVInstance, UAVPlan, Waypoint, WeatherObs,
                       WeatherThresholds, WindLayer)
from ..platform import PLATFORM_HERO, PLATFORM_PIONEER, CameraHardware, LidarHardware, PlatformSpec, SensorHardware
from ..scenario import synthetic_dem_for_latlon_bounds
from .. import decompose as _decompose
from ..planner import plan_mission

os.environ.setdefault("MAVLINK20", "1")
try:  # pragma: no cover - 依環境
    from pymavlink import mavutil as _mavutil
    MAV = _mavutil.mavlink
    """MAVLink 常數來源（pymavlink ``mavutil.mavlink``）。"""
    HAVE_PYMAVLINK = True
except ImportError:  # pragma: no cover
    class _MavConst:
        """pymavlink 缺席時之後備常數（值與 common.xml 一致；只供離線序列化）。"""
        MAV_CMD_NAV_WAYPOINT = 16
        MAV_CMD_NAV_LOITER_TURNS = 18
        MAV_CMD_NAV_LOITER_TIME = 19
        MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
        MAV_CMD_NAV_TAKEOFF = 22
        MAV_CMD_NAV_LOITER_TO_ALT = 31
        MAV_CMD_NAV_VTOL_TAKEOFF = 84
        MAV_CMD_NAV_VTOL_LAND = 85
        MAV_CMD_NAV_DELAY = 93
        MAV_CMD_CONDITION_YAW = 115
        MAV_CMD_DO_CHANGE_SPEED = 178
        MAV_CMD_DO_SET_HOME = 179
        MAV_CMD_DO_LAND_START = 189
        MAV_CMD_DO_DIGICAM_CONTROL = 203
        MAV_CMD_DO_SET_CAM_TRIGG_DIST = 206
        MAV_FRAME_GLOBAL = 0
        MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
        MAV_FRAME_GLOBAL_TERRAIN_ALT = 10

    MAV = _MavConst()
    HAVE_PYMAVLINK = False

__all__ = [
    "HVDCCPPGeneratorAdapter", "LegacyCollisionAvoidance", "LegacyAdapterError", "MissionItem",
    "plan_to_mission_items", "mission_items_to_qgc", "thin_waypoints", "simple_vtol_mission_items",
    "PLATFORM_KEYS", "DEFAULT_ENERGY", "QGC_HEADER", "MAV", "HAVE_PYMAVLINK",
]

QGC_HEADER = "QGC WPL 110"
PLATFORM_KEYS: dict[str, PlatformSpec] = {"pioneer_pnp": PLATFORM_PIONEER, "hero_pnp": PLATFORM_HERO}
"""`platform` 字串 → 表 1 PlatformSpec（A = Pioneer PNP、B = HERO PNP）。"""
DEFAULT_ENERGY: dict[str, dict[str, float]] = {
    "A": {"auw_kg": 24.6, "payload_kg": 5.0, "available_wh": 1500.0, "pack_nominal_wh": 1500.0},
    "B": {"auw_kg": 6.65, "payload_kg": 0.6, "available_wh": 420.0, "pack_nominal_wh": 420.0},
}
"""`energy=None` 時之預設：示例電池滿電（規格 §3.5 battery_nominal_wh 示例 1500/420 Wh，基準 AUW）。"""

_FULL_LOITER_DEG = 300.0
"""同一弧參數之連續航點掃過 ≥ 300° 視為整圈盤旋 → 以 NAV_LOITER_TO_ALT 取代外切多邊形頂點。"""
_SPEED_EPS_MPS = 0.5
_AUTO_ALTERNATE_OFFSET_M = 400.0


class LegacyAdapterError(RuntimeError):
    """規劃失敗（preflight NO_GO、區域不可覆蓋等）；`errors` 為 FeasibilityReport 錯誤字串。"""

    def __init__(self, msg: str, errors: Sequence[str] = ()) -> None:
        super().__init__(msg)
        self.errors = list(errors)


# ---------------------------------------------------------------------------
# 任務項目（QGC WPL 110 一列）
# ---------------------------------------------------------------------------

@dataclass(slots=True)
class MissionItem:
    """一個 MAVLink 任務項目 = QGC WPL 110 之一列（12 欄：seq current frame command p1 p2 p3 p4 x y z autocontinue）。"""
    seq: int
    current: int
    frame: int
    command: int
    param1: float = 0.0
    param2: float = 0.0
    param3: float = 0.0
    param4: float = 0.0
    x: float = 0.0          # lat（deg）
    y: float = 0.0          # lon（deg）
    z: float = 0.0          # alt（m；frame 3 → 相對 home）
    autocontinue: int = 1

    def to_qgc_line(self) -> str:
        return "\t".join([
            str(int(self.seq)), str(int(self.current)), str(int(self.frame)), str(int(self.command)),
            f"{self.param1:.8f}", f"{self.param2:.8f}", f"{self.param3:.8f}", f"{self.param4:.8f}",
            f"{self.x:.8f}", f"{self.y:.8f}", f"{self.z:.6f}", str(int(self.autocontinue)),
        ])

    @property
    def is_nav(self) -> bool:
        return int(self.command) in (MAV.MAV_CMD_NAV_WAYPOINT, MAV.MAV_CMD_NAV_VTOL_TAKEOFF, MAV.MAV_CMD_NAV_VTOL_LAND,
                                     MAV.MAV_CMD_NAV_LOITER_TO_ALT, MAV.MAV_CMD_NAV_LOITER_TURNS,
                                     MAV.MAV_CMD_NAV_LOITER_TIME, MAV.MAV_CMD_NAV_TAKEOFF)


def mission_items_to_qgc(items: Sequence[MissionItem]) -> list[str]:
    """任務項目 → QGC WPL 110 文字列（首列標頭；seq 重新編號、第 0 列 current=1）。"""
    lines = [QGC_HEADER]
    for k, it in enumerate(items):
        it.seq = k
        it.current = 1 if k == 0 else 0
        lines.append(it.to_qgc_line())
    return lines


# ---------------------------------------------------------------------------
# 幾何小工具
# ---------------------------------------------------------------------------

def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(0.5 * dphi) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(0.5 * dl) ** 2
    return 2.0 * r * math.asin(min(1.0, math.sqrt(a)))


def _latlon_list(corners: Sequence[Any]) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    for p in corners:
        if isinstance(p, Mapping):
            out.append((float(p["lat"]), float(p["lon"])))
        else:
            out.append((float(p[0]), float(p[1])))
    if len(out) >= 2 and out[0] == out[-1]:
        out.pop()
    return out


def _polys_of(geom: Any) -> list[Polygon]:
    if geom is None or geom.is_empty:
        return []
    if isinstance(geom, Polygon):
        return [geom]
    if isinstance(geom, MultiPolygon):
        return [g for g in geom.geoms if not g.is_empty]
    if hasattr(geom, "geoms"):
        out: list[Polygon] = []
        for g in geom.geoms:
            out.extend(_polys_of(g))
        return out
    return []


def _bisect_polygon(poly: Polygon) -> list[Polygon]:
    """以軸向直線（沿較長邊界方向）把多邊形切成面積相等之兩半（面積二分搜尋）。"""
    minx, miny, maxx, maxy = poly.bounds
    along_x = (maxx - minx) >= (maxy - miny)
    lo, hi = (minx, maxx) if along_x else (miny, maxy)
    target = 0.5 * poly.area
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        left = poly.intersection(box(minx, miny, mid, maxy) if along_x else box(minx, miny, maxx, mid))
        if left.area < target:
            lo = mid
        else:
            hi = mid
    cut = 0.5 * (lo + hi)
    a = poly.intersection(box(minx, miny, cut, maxy) if along_x else box(minx, miny, maxx, cut))
    b = poly.intersection(box(cut, miny, maxx, maxy) if along_x else box(minx, cut, maxx, maxy))
    parts = _polys_of(a) + _polys_of(b)
    return parts if len(parts) >= 2 else [poly]


def _group_by_area(parts: list[Polygon], n: int) -> list[list[Polygon]]:
    """沿主軸排序後貪婪分成 n 組（各組面積接近 total/n）。"""
    if n <= 1 or len(parts) <= 1:
        return [list(parts)]
    allp = shapely.union_all(parts)
    minx, miny, maxx, maxy = allp.bounds
    along_x = (maxx - minx) >= (maxy - miny)
    order = sorted(parts, key=lambda p: (p.centroid.x if along_x else p.centroid.y))
    total = sum(p.area for p in order)
    groups: list[list[Polygon]] = [[] for _ in range(n)]
    acc = 0.0
    g = 0
    for p in order:
        if g < n - 1 and groups[g] and acc + 0.5 * p.area >= total * (g + 1) / n:
            g += 1
        groups[g].append(p)
        acc += p.area
    return [grp for grp in groups if grp]


# ---------------------------------------------------------------------------
# 既有系統之 collision_avoidance 介面
# ---------------------------------------------------------------------------

class LegacyCollisionAvoidance:
    """`main_app.py:635` 穿透存取之物件：``calculate_loiter_delay(prev_waypoints, current_start, cruise_speed)``。

    語意與既有 ``collision_avoidance.py`` 一致：沿前一架航點列表累計距離，直到其航點離本機起點超過
    ``safety_distance``，所需時間 = 累計距離／巡航速度，再加 2 s 緩衝；航點少於 2 個 → 0。
    另加 ``min_delay_s``（預設 0）供整合者要求最小起飛間隔（HV-DCCPP 本身已以 ``cfg.dt_dep_s`` 排程起飛）。
    """

    def __init__(self, safety_distance: float = 5.0, min_delay_s: float = 0.0) -> None:
        self.safety_distance = float(safety_distance)
        self.min_delay_s = float(min_delay_s)

    @staticmethod
    def calculate_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        return _haversine_m(lat1, lon1, lat2, lon2)

    def calculate_loiter_delay(self, prev_waypoints: Sequence[Any], current_start: Sequence[float],
                               cruise_speed: float) -> float:
        pts = _latlon_list(prev_waypoints) if prev_waypoints else []
        if len(pts) < 2:
            return max(0.0, self.min_delay_s)
        v = max(float(cruise_speed), 0.1)
        s0 = (float(current_start[0]), float(current_start[1]))
        cum = 0.0
        clearance = 0.0
        for i in range(len(pts) - 1):
            cum += self.calculate_distance(pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1])
            if self.calculate_distance(s0[0], s0[1], pts[i + 1][0], pts[i + 1][1]) > self.safety_distance:
                clearance = cum / v
                break
        return max(clearance + 2.0, self.min_delay_s)


# ---------------------------------------------------------------------------
# Plan → 任務項目
# ---------------------------------------------------------------------------

def thin_waypoints(P: np.ndarray, keep: np.ndarray, tol_xy_m: float = 2.0, tol_z_m: float = 3.0,
                   l_max_m: float = 1000.0) -> list[int]:
    """點刪減（同 altitude.reduce_points 之弦檢查，但容差為 autopilot 用之 2 m／3 m；ardupilot_notes §1：直線段只留端點）。

    `P` (n,3) ENU；`keep` True 者一律保留；回傳保留索引（升冪，含首末）。
    """
    n = int(P.shape[0])
    if n == 0:
        return []
    keep = np.asarray(keep, dtype=bool)
    out = [0]
    last = 0
    j = 1
    while j < n:
        if keep[j] or j == n - 1:
            out.append(j)
            last = j
            j += 1
            continue
        cand = j + 1
        chord = P[cand] - P[last]
        L = float(np.linalg.norm(chord))
        ok = L <= l_max_m
        if ok and L > 1e-9:
            mid = P[last + 1:cand] - P[last]
            t = np.clip((mid[:, :2] @ chord[:2]) / max(float(chord[0] ** 2 + chord[1] ** 2), 1e-9), 0.0, 1.0)
            proj = t[:, None] * chord[None, :]
            dxy = np.hypot(mid[:, 0] - proj[:, 0], mid[:, 1] - proj[:, 1])
            dz = np.abs(mid[:, 2] - proj[:, 2])
            ok = bool(np.all(dxy <= tol_xy_m) and np.all(dz <= tol_z_m))
        elif ok:
            ok = True
        if ok:
            j += 1
        else:
            out.append(j)
            last = j
            j += 1
    if out[-1] != n - 1:
        out.append(n - 1)
    return out


@dataclass(slots=True)
class _Node:
    lat: float
    lon: float
    rel_alt: float
    xy: tuple[float, float]
    z: float
    phase: Phase
    sensor_on: bool
    airspeed: float
    accept_radius: float
    strip_id: str | None
    loiter_radius_signed: float | None = None     # NAV_LOITER_TO_ALT（None → NAV_WAYPOINT）


def _arc_key(arc: tuple[float, float, float, int] | None) -> tuple | None:
    if arc is None:
        return None
    return (round(float(arc[0]), 3), round(float(arc[1]), 3), round(float(arc[2]), 3), int(arc[3]))


def _route_waypoints(wps: Sequence[Waypoint]) -> list[Waypoint]:
    """起飛（VTOL_TAKEOFF）與降落（VTOL_LAND）由指令承擔：略去 TAKEOFF／LAND／TRANS_IN 樣本，TRANS_OUT 只留末點（P_tc）。"""
    out: list[Waypoint] = []
    n = len(wps)
    for i, w in enumerate(wps):
        if w.phase in (Phase.TAKEOFF, Phase.LAND, Phase.TRANS_IN):
            continue
        if w.phase is Phase.TRANS_OUT:
            nxt = wps[i + 1].phase if i + 1 < n else None
            if nxt is Phase.TRANS_OUT:
                continue
        out.append(w)
    return out


def _nodes_from_waypoints(route: Sequence[Waypoint], home_msl_m: float, frame: GeoFrame | None,
                          tol_xy_m: float, tol_z_m: float, l_max_m: float, arc_step_deg: float = 30.0) -> list[_Node]:
    n = len(route)
    if n == 0:
        return []
    keys = [_arc_key(w.arc) for w in route]
    # 弧段連續段（同一 arc 參數）
    runs: list[tuple[int, int]] = []
    i = 0
    while i < n:
        j = i
        while j + 1 < n and keys[j + 1] is not None and keys[j + 1] == keys[i] and keys[i] is not None:
            j += 1
        runs.append((i, j))
        i = j + 1
    loiter_runs: dict[int, tuple[int, float]] = {}   # start idx → (end idx, signed radius)
    drop = np.zeros(n, dtype=bool)                    # 弧段內角步 < arc_step 之中間點（autopilot 自行轉彎；D27 上限 30°）
    step = math.radians(max(float(arc_step_deg), 1.0))
    for a, b in runs:
        if keys[a] is None or b - a < 1:
            continue
        cx, cy, R, d = route[a].arc  # type: ignore[misc]
        ang = np.unwrap(np.array([math.atan2(w.y_m - cy, w.x_m - cx) for w in route[a:b + 1]]))
        swept = abs(float(ang[-1] - ang[0]))
        if frame is not None and b - a >= 2 and math.degrees(swept) >= _FULL_LOITER_DEG:
            loiter_runs[a] = (b, float(R) * (1.0 if int(d) > 0 else -1.0))
            continue
        last_ang = float(ang[0])
        for k in range(a + 1, b):
            if abs(float(ang[k - a]) - last_ang) >= step:
                last_ang = float(ang[k - a])
            else:
                drop[k] = True
    keep = np.zeros(n, dtype=bool)
    keep[0] = keep[-1] = True
    for k in range(1, n):
        w, p = route[k], route[k - 1]
        if (w.phase is not p.phase or w.sensor_on != p.sensor_on or w.strip_id != p.strip_id
                or abs(w.airspeed_mps - p.airspeed_mps) > _SPEED_EPS_MPS or keys[k] != keys[k - 1]):
            keep[k] = True
            keep[k - 1] = True
    for a, (b, _r) in loiter_runs.items():
        keep[a] = keep[b] = True
    drop &= ~keep
    elig = np.flatnonzero(~drop)
    P = np.array([[w.x_m, w.y_m, w.z_m] for w in route], dtype=float)
    kept_local = thin_waypoints(P[elig], keep[elig], tol_xy_m, tol_z_m, l_max_m)
    kept_set = {int(elig[i]) for i in kept_local}
    nodes: list[_Node] = []
    k = 0
    while k < n:
        if k in loiter_runs and frame is not None:
            b, r_signed = loiter_runs[k]
            cx, cy, _R, _d = route[k].arc  # type: ignore[misc]
            w_end = route[b]
            ll = frame.to_wgs84(np.array([cx, cy, w_end.z_m]))[0]
            nodes.append(_Node(float(ll[0]), float(ll[1]), float(w_end.alt_msl_m - home_msl_m), (cx, cy), w_end.z_m,
                               w_end.phase, w_end.sensor_on, w_end.airspeed_mps, w_end.accept_radius_m, w_end.strip_id,
                               loiter_radius_signed=r_signed))
            k = b + 1
            continue
        if k in kept_set:
            w = route[k]
            if nodes and nodes[-1].loiter_radius_signed is None and \
                    math.hypot(w.x_m - nodes[-1].xy[0], w.y_m - nodes[-1].xy[1]) < 1.0 and \
                    w.sensor_on == nodes[-1].sensor_on and w.phase is nodes[-1].phase:
                k += 1
                continue
            nodes.append(_Node(w.lat_deg, w.lon_deg, float(w.alt_msl_m - home_msl_m), (w.x_m, w.y_m), w.z_m, w.phase,
                               bool(w.sensor_on), float(w.airspeed_mps), float(w.accept_radius_m), w.strip_id))
        k += 1
    return nodes


def plan_to_mission_items(plan: UAVPlan | Sequence[Waypoint], *, spec: PlatformSpec, home_latlon_msl: tuple[float, float, float],
                          frame: GeoFrame | None = None, trigger_dist_m: float | None = 30.0, speed_cap_mps: float | None = None,
                          loiter_time_s: float = 0.0, landing_latlon: tuple[float, float] | None = None,
                          lidar_only: bool = False, tol_xy_m: float = 2.0, tol_z_m: float = 3.0,
                          l_max_m: float = 1000.0, arc_step_deg: float = 30.0) -> list[MissionItem]:
    """UAVPlan（或 Waypoint 列）→ ArduPilot QuadPlane 任務項目（ardupilot_notes §1）。

    序列：home 列（frame 0、NAV_WAYPOINT、current 1、alt = home MSL）→ [NAV_DELAY 93（`loiter_time_s`>0；Plane 4.1+ 待驗證）]
    → DO_CHANGE_SPEED（airspeed）→ NAV_VTOL_TAKEOFF（alt = H_ts = `spec.transition_start_agl_m`）→ 航點
    （NAV_WAYPOINT frame 3、param2 = 接受半徑 r_acc；整圈爬升／等待盤旋 → NAV_LOITER_TO_ALT，半徑正 = 順時針）
    ；作業條帶以 DO_SET_CAM_TRIGG_DIST(dist)／(0) 包住（LiDAR-only → DO_DIGICAM_CONTROL 1／0，待驗證）；
    空速變化 > 0.5 m/s → DO_CHANGE_SPEED；首個 DECEL 樣本（S^app）前插 DO_LAND_START → NAV_VTOL_LAND（alt 0）於落點。
    `speed_cap_mps`：指令空速上限（規劃仍用表 1 空速）；下限 `spec.v_min_mission()`。
    點刪減：直線段以弦容差 `tol_xy_m`／`tol_z_m` 刪點（只留端點）；Dubins 弧段每 `arc_step_deg`（≤ 30°，D27）留一點
    （ardupilot_notes §1：L1 追蹤會提前切入，弧段不宜過密）。
    """
    wps = list(plan.waypoints) if isinstance(plan, UAVPlan) else list(plan)
    items: list[MissionItem] = []
    lat_h, lon_h, msl_h = (float(v) for v in home_latlon_msl)
    items.append(MissionItem(0, 1, MAV.MAV_FRAME_GLOBAL, MAV.MAV_CMD_NAV_WAYPOINT, x=lat_h, y=lon_h, z=msl_h))
    if not wps:
        return items
    F = MAV.MAV_FRAME_GLOBAL_RELATIVE_ALT
    v_min = float(spec.v_min_mission())
    v_cap = float(spec.cruise_airspeed_mps) if speed_cap_mps is None else max(float(speed_cap_mps), v_min)

    def cmd_speed(v: float) -> float:
        return min(max(float(v), v_min), v_cap)

    if loiter_time_s and loiter_time_s > 0:
        items.append(MissionItem(0, 0, F, MAV.MAV_CMD_NAV_DELAY, param1=float(loiter_time_s)))
    cur_speed = cmd_speed(spec.cruise_airspeed_mps)
    items.append(MissionItem(0, 0, F, MAV.MAV_CMD_DO_CHANGE_SPEED, param1=0.0, param2=cur_speed, param3=-1.0))
    w0 = wps[0]
    items.append(MissionItem(0, 0, F, MAV.MAV_CMD_NAV_VTOL_TAKEOFF, param4=0.0, x=w0.lat_deg, y=w0.lon_deg,
                             z=float(spec.transition_start_agl_m)))
    route = _route_waypoints(wps)
    nodes = _nodes_from_waypoints(route, msl_h, frame, tol_xy_m, tol_z_m, l_max_m, arc_step_deg)
    cam_on = False
    land_start_done = False
    trig = float(trigger_dist_m) if trigger_dist_m is not None and trigger_dist_m > 0 else 0.0

    def cam(on: bool) -> MissionItem:
        if lidar_only:
            return MissionItem(0, 0, F, MAV.MAV_CMD_DO_DIGICAM_CONTROL, param1=1.0 if on else 0.0)
        return MissionItem(0, 0, F, MAV.MAV_CMD_DO_SET_CAM_TRIGG_DIST, param1=trig if on else 0.0)

    for nd in nodes:
        if nd.phase is Phase.DECEL and not land_start_done:
            if cam_on:
                items.append(cam(False))
                cam_on = False
            items.append(MissionItem(0, 0, F, MAV.MAV_CMD_DO_LAND_START, x=nd.lat, y=nd.lon))
            land_start_done = True
        v = cmd_speed(nd.airspeed)
        if abs(v - cur_speed) > _SPEED_EPS_MPS:
            items.append(MissionItem(0, 0, F, MAV.MAV_CMD_DO_CHANGE_SPEED, param1=0.0, param2=v, param3=-1.0))
            cur_speed = v
        if nd.loiter_radius_signed is not None:
            items.append(MissionItem(0, 0, F, MAV.MAV_CMD_NAV_LOITER_TO_ALT, param1=0.0, param2=nd.loiter_radius_signed,
                                     param3=0.0, param4=1.0, x=nd.lat, y=nd.lon, z=max(nd.rel_alt, 0.0)))
        else:
            items.append(MissionItem(0, 0, F, MAV.MAV_CMD_NAV_WAYPOINT, param1=0.0, param2=float(nd.accept_radius),
                                     param3=0.0, param4=0.0, x=nd.lat, y=nd.lon, z=max(nd.rel_alt, 0.0)))
        # 酬載開關描述「本點之出發腿」；ArduPilot 於抵達前一個 NAV 時執行其後之 DO_ ⇒ 必須放在本點 NAV 之後
        if nd.sensor_on != cam_on:
            items.append(cam(nd.sensor_on))
            cam_on = nd.sensor_on
    if cam_on:
        items.append(cam(False))
    if not land_start_done:
        last = nodes[-1] if nodes else None
        if last is not None:
            items.append(MissionItem(0, 0, F, MAV.MAV_CMD_DO_LAND_START, x=last.lat, y=last.lon))
    if landing_latlon is None:
        wl = wps[-1]
        landing_latlon = (wl.lat_deg, wl.lon_deg)
    items.append(MissionItem(0, 0, F, MAV.MAV_CMD_NAV_VTOL_LAND, x=float(landing_latlon[0]), y=float(landing_latlon[1]), z=0.0))
    return items


def simple_vtol_mission_items(latlon: Sequence[tuple[float, float]], *, spec: PlatformSpec, altitude_agl_m: float,
                              speed_mps: float, home_latlon_msl: tuple[float, float, float] | None = None,
                              trigger_dist_m: float = 30.0, loiter_time_s: float = 0.0, accept_radius_m: float = 30.0,
                              landing_latlon: tuple[float, float] | None = None) -> list[MissionItem]:
    """既有系統之 `(lat, lon)` 航點列 → VTOL 正確之任務項目（全部航點同高；首點起飛、末點降落；
    中間航點以相機觸發包住）。`home_latlon_msl` None → 首點、MSL 0（相對高度模式下 home 列 alt 僅供顯示）。"""
    pts = _latlon_list(latlon)
    if home_latlon_msl is None:
        home_latlon_msl = (pts[0][0], pts[0][1], 0.0) if pts else (0.0, 0.0, 0.0)
    lat_h, lon_h, msl_h = home_latlon_msl
    items = [MissionItem(0, 1, MAV.MAV_FRAME_GLOBAL, MAV.MAV_CMD_NAV_WAYPOINT, x=lat_h, y=lon_h, z=msl_h)]
    if not pts:
        return items
    F = MAV.MAV_FRAME_GLOBAL_RELATIVE_ALT
    v = min(max(float(speed_mps), float(spec.v_min_mission())), float(spec.cruise_airspeed_mps))
    if loiter_time_s and loiter_time_s > 0:
        items.append(MissionItem(0, 0, F, MAV.MAV_CMD_NAV_DELAY, param1=float(loiter_time_s)))
    items.append(MissionItem(0, 0, F, MAV.MAV_CMD_DO_CHANGE_SPEED, param1=0.0, param2=v, param3=-1.0))
    items.append(MissionItem(0, 0, F, MAV.MAV_CMD_NAV_VTOL_TAKEOFF, x=pts[0][0], y=pts[0][1],
                             z=float(spec.transition_start_agl_m)))
    alt = float(altitude_agl_m)
    items.append(MissionItem(0, 0, F, MAV.MAV_CMD_NAV_WAYPOINT, param2=accept_radius_m, x=pts[0][0], y=pts[0][1], z=alt))
    interior = pts[1:-1] if len(pts) >= 3 else pts[1:]
    if interior:
        items.append(MissionItem(0, 0, F, MAV.MAV_CMD_DO_SET_CAM_TRIGG_DIST, param1=float(trigger_dist_m)))
        for la, lo in interior:
            items.append(MissionItem(0, 0, F, MAV.MAV_CMD_NAV_WAYPOINT, param2=accept_radius_m, x=la, y=lo, z=alt))
        items.append(MissionItem(0, 0, F, MAV.MAV_CMD_DO_SET_CAM_TRIGG_DIST, param1=0.0))
    end = pts[-1] if len(pts) >= 2 else pts[0]
    items.append(MissionItem(0, 0, F, MAV.MAV_CMD_DO_LAND_START, x=end[0], y=end[1]))
    if len(pts) >= 2:
        items.append(MissionItem(0, 0, F, MAV.MAV_CMD_NAV_WAYPOINT, param2=accept_radius_m, x=end[0], y=end[1], z=alt))
    land = landing_latlon if landing_latlon is not None else end
    items.append(MissionItem(0, 0, F, MAV.MAV_CMD_NAV_VTOL_LAND, x=float(land[0]), y=float(land[1]), z=0.0))
    return items


# ---------------------------------------------------------------------------
# 掃描方向覆寫（與 PlannerConfig 無關；暫時替換 decompose.direction_candidates）
# ---------------------------------------------------------------------------

@contextlib.contextmanager
def _direction_override(alpha_rad: float | None, mode: str) -> Iterator[None]:
    """`mode="force"` → 候選只剩 α；`"hint"` → α 置於候選之首；`"ignore"`／α None → 不變。
    僅在 `with` 區塊內替換模組屬性 `decompose.direction_candidates`（decompose_area 以全域名稱呼叫），離開即還原。"""
    if alpha_rad is None or mode == "ignore":
        yield
        return
    orig = _decompose.direction_candidates
    a = float(alpha_rad) % (2.0 * math.pi)

    def patched(cell_poly: Polygon, ctx: Any, z_ref_m: float) -> list[float]:
        if mode == "force":
            return [a]
        return [a] + [c for c in orig(cell_poly, ctx, z_ref_m) if abs(((c - a + math.pi) % (2 * math.pi)) - math.pi) > 1e-6]

    _decompose.direction_candidates = patched  # type: ignore[assignment]
    try:
        yield
    finally:
        _decompose.direction_candidates = orig  # type: ignore[assignment]


def legacy_angle_to_theta_rad(angle_deg: float) -> float:
    """既有系統 `params.angle`（`project_and_rotate` 旋轉 θ 後沿旋轉 x 軸鋪線 ⇒ 條帶方向 = ENU (cos θ, −sin θ)）
    → HV-DCCPP 航跡方位 θ_N（自北順時針）：θ_N = 90° + angle（模 180° 等價）。"""
    return math.radians(90.0 + float(angle_deg)) % (2.0 * math.pi)


# ---------------------------------------------------------------------------
# Adapter
# ---------------------------------------------------------------------------

@dataclass(slots=True)
class _Params:
    altitude: float | None
    angle: float | None
    spacing: float | None
    speed: float | None
    safety_distance: float

    @classmethod
    def read(cls, params: Any) -> "_Params":
        def g(name: str) -> float | None:
            if params is None:
                return None
            v = getattr(params, name, None) if not isinstance(params, Mapping) else params.get(name)
            return None if v is None else float(v)
        sd = g("safety_distance")
        return cls(g("altitude"), g("angle"), g("spacing"), g("speed"), 5.0 if sd is None else sd)


class HVDCCPPGeneratorAdapter:
    """`self.waypoint_generator` 之替換物（integration_notes §注入點 1）。

    參數
    ----
    platform
        ``"pioneer_pnp"``（A）或 ``"hero_pnp"``（B）；`generate_complete_mission` 之單機平台。
    home_latlon
        起降點 `(lat, lon)`；None → 第一次呼叫之第一個角點（快取，之後各區共用）。
    dem
        :class:`geo.DemGrid`；None → 以 `base_msl_m` 之平地合成 DEM（涵蓋幾何 ⊕ `terrain_margin_m` + 1 km，σ_z 2 m）。
    alternates
        備降點 `[(lat, lon) | Site]`；None → 自動於 home 背向區域 400 m 處置一備降點
        （表 1 `alternate_landing_sites_required=true`，缺則 preflight NO_GO）。
    wind
        `(speed_mps, dir_from_deg)` 單層（AGL 0–3000 m）。
    c2_range_km / terrain_clearance_m / waiver_max_agl_m
        表 2 必填項（D11）與 D12 豁免上限（None → 120 m AGL，A 於平地不可行）。
    energy
        :class:`mission.SortieEnergy`；None → 示例電池滿電（:data:`DEFAULT_ENERGY`）。
    cfg
        :class:`mission.PlannerConfig`；None → 預設但 `dem_step_m=25`（平地不需 10 m）。
    sensor_kind
        任務感測種類（`"rgb"` 攝影測量＝既有系統用途；`"lidar"` 僅 Pioneer）。
    angle_mode
        `params.angle` 之用法：`"force"`（唯一掃描方向；不可行時自動退回 `"hint"` 並記警告）、`"hint"`、`"ignore"`。
    spacing_mode
        `params.spacing`：`"overlap_hint"` → 換算為側向重疊率（0.05–0.85 內才採用）；`"ignore"`。

    `generate_complete_mission` 的 `params` 只以 duck typing 讀 ``altitude / angle / spacing / speed / safety_distance``。
    """

    def __init__(self, platform: Literal["pioneer_pnp", "hero_pnp"] = "hero_pnp",
                 home_latlon: tuple[float, float] | None = None, dem: DemGrid | None = None,
                 alternates: Sequence[Any] | None = None, wind: tuple[float, float] = (0.0, 0.0),
                 c2_range_km: float = 25.0, terrain_clearance_m: float = 36.0, waiver_max_agl_m: float | None = 200.0,
                 energy: SortieEnergy | None = None, cfg: PlannerConfig | None = None, *,
                 base_msl_m: float = 50.0, terrain_margin_m: float = 2000.0, sensor_kind: Literal["rgb", "lidar"] = "rgb",
                 hardware: Sequence[SensorHardware] | None = None,
                 angle_mode: Literal["force", "hint", "ignore"] = "force",
                 spacing_mode: Literal["overlap_hint", "ignore"] = "overlap_hint",
                 weather_temp_c: float = 25.0, safety_distance: float = 5.0) -> None:
        if platform not in PLATFORM_KEYS:
            raise ValueError(f"platform 須為 {sorted(PLATFORM_KEYS)}，得 {platform!r}")
        self.platform = platform
        self.spec: PlatformSpec = PLATFORM_KEYS[platform]
        self.home_latlon = None if home_latlon is None else (float(home_latlon[0]), float(home_latlon[1]))
        self.dem = dem
        self.alternates = list(alternates) if alternates else None
        self.wind = (float(wind[0]), float(wind[1]))
        self.c2_range_km = float(c2_range_km)
        self.terrain_clearance_m = float(terrain_clearance_m)
        self.waiver_max_agl_m = None if waiver_max_agl_m is None else float(waiver_max_agl_m)
        self.energy = energy
        self.cfg = cfg if cfg is not None else PlannerConfig(dem_step_m=25.0)
        self.base_msl_m = float(base_msl_m)
        self.terrain_margin_m = float(terrain_margin_m)
        self.sensor_kind = sensor_kind
        self.hardware = tuple(hardware) if hardware is not None else None
        self.angle_mode = angle_mode
        self.spacing_mode = spacing_mode
        self.weather_temp_c = float(weather_temp_c)
        self.collision_avoidance = LegacyCollisionAvoidance(safety_distance)
        self.warnings: list[str] = []
        self.last_plan: MissionPlan | None = None
        self.last_inputs: MissionInputs | None = None
        self.last_items: dict[str, list[MissionItem]] = {}
        self._dem_cache: dict[tuple, DemGrid] = {}
        self._last_frame: GeoFrame | None = None
        self._last_home_msl: float | None = None
        self._last_trigger: float | None = None

    # -- 平台／機隊 ------------------------------------------------------------
    @staticmethod
    def _spec_for(platform: str) -> PlatformSpec:
        if platform not in PLATFORM_KEYS:
            raise ValueError(f"platform 須為 {sorted(PLATFORM_KEYS)}，得 {platform!r}")
        return PLATFORM_KEYS[platform]

    def _hardware_for(self, spec: PlatformSpec, hardware: Sequence[SensorHardware] | None = None) -> tuple[SensorHardware, ...]:
        if hardware is not None:
            return tuple(hardware)
        if self.hardware is not None:
            return self.hardware
        return (LidarHardware(), CameraHardware()) if spec.cls == "A" else (CameraHardware(),)

    def _energy_for(self, uav_id: str, spec: PlatformSpec, energy: SortieEnergy | Mapping[str, Any] | None) -> SortieEnergy:
        if energy is None:
            energy = self.energy
        if energy is None:
            return SortieEnergy(uav_id, **DEFAULT_ENERGY[spec.cls])
        if isinstance(energy, SortieEnergy):
            return replace(energy, uav_id=uav_id)
        kw = dict(energy)
        kw.pop("uav_id", None)
        return SortieEnergy(uav_id, **kw)

    def _uav(self, uav_id: str, platform: str, energy: Any = None, hardware: Sequence[SensorHardware] | None = None) -> UAVInstance:
        spec = self._spec_for(platform)
        return UAVInstance(uav_id, spec, self._hardware_for(spec, hardware), self._energy_for(uav_id, spec, energy))

    # -- 輸入組裝 ------------------------------------------------------------
    def _resolve_home(self, corners: Sequence[tuple[float, float]]) -> tuple[float, float]:
        if self.home_latlon is None:
            self.home_latlon = corners[0]
            self.warnings.append(f"home_defaulted_to_first_corner({corners[0][0]:.6f},{corners[0][1]:.6f})")
        return self.home_latlon

    def _resolve_alternates(self, home: tuple[float, float], corners: Sequence[tuple[float, float]]) -> tuple[Site, ...]:
        if self.alternates:
            out: list[Site] = []
            for i, a in enumerate(self.alternates):
                if isinstance(a, Site):
                    out.append(a if a.kind == "alternate" else replace(a, kind="alternate"))
                else:
                    la, lo = _latlon_list([a])[0]
                    out.append(Site(f"ALT{i + 1}", la, lo, kind="alternate"))
            return tuple(out)
        # 自動備降點：home 背向區域質心 400 m
        c_lat = sum(p[0] for p in corners) / len(corners)
        c_lon = sum(p[1] for p in corners) / len(corners)
        frame = GeoFrame(home[0], home[1], 0.0)
        c = frame.to_enu_xy(c_lat, c_lon)[0]
        d = math.hypot(c[0], c[1])
        ux, uy = (-c[0] / d, -c[1] / d) if d > 1.0 else (0.0, -1.0)
        w = frame.to_wgs84(np.array([ux * _AUTO_ALTERNATE_OFFSET_M, uy * _AUTO_ALTERNATE_OFFSET_M, 0.0]))[0]
        self.warnings.append("alternate_auto_placed(ALT1, 400 m from home)")
        return (Site("ALT1", float(w[0]), float(w[1]), kind="alternate"),)

    def _dem_for(self, points: Sequence[tuple[float, float]]) -> DemGrid:
        if self.dem is not None:
            return self.dem
        lats = [p[0] for p in points]
        lons = [p[1] for p in points]
        lat_c = 0.5 * (min(lats) + max(lats))
        margin_m = self.terrain_margin_m + 1000.0
        dlat = margin_m / 111132.0
        dlon = margin_m / (111412.0 * math.cos(math.radians(lat_c)))
        key = (round(min(lats) - dlat, 4), round(min(lons) - dlon, 4), round(max(lats) + dlat, 4), round(max(lons) + dlon, 4))
        dem = self._dem_cache.get(key)
        if dem is None:
            dem = synthetic_dem_for_latlon_bounds(key[0], key[1], key[2], key[3], 50.0, base_msl_m=self.base_msl_m, sigma_z_m=2.0)
            self._dem_cache[key] = dem
        return dem

    def _payload_req(self, p: _Params, specs: Sequence[PlatformSpec], hw_list: Sequence[SensorHardware]) -> PayloadRequirement:
        ceiling = self.waiver_max_agl_m if self.waiver_max_agl_m is not None else 120.0
        h_minfw = min(float(s.min_fw_operation_agl_m) for s in specs)
        h_target = None
        if p.altitude is not None:
            h_target = min(max(float(p.altitude), h_minfw), ceiling)
            if abs(h_target - float(p.altitude)) > 1e-9:
                self.warnings.append(f"altitude_clamped({p.altitude}->{h_target})")
        gsd = None
        side = None
        cam = next((h for h in hw_list if isinstance(h, CameraHardware)), None)
        if self.sensor_kind == "rgb":
            if cam is None:
                raise LegacyAdapterError("sensor_kind='rgb' 但機隊無 CameraHardware")
            h_ref = h_target if h_target is not None else max(h_minfw + self.cfg.dh_band_min_m, 100.0)
            # h_max = GSD·f/p ≥ 1.05·h_target（platform.CameraSensor.agl_window）
            gsd = 1.05 * h_ref * (cam.pixel_pitch_um * 1e-6) / (cam.focal_length_mm * 1e-3)
            if p.spacing is not None and p.spacing > 0 and self.spacing_mode == "overlap_hint":
                w = h_ref * cam.sensor_width_mm / cam.focal_length_mm
                o = 1.0 - float(p.spacing) / w
                if 0.05 <= o <= 0.85:
                    side = o
                else:
                    self.warnings.append(f"spacing_hint_ignored(spacing={p.spacing}, swath={w:.1f})")
        else:
            if not any(isinstance(h, LidarHardware) for h in hw_list):
                raise LegacyAdapterError("sensor_kind='lidar' 但機隊無 LidarHardware（HERO 無 LiDAR）")
        return PayloadRequirement("default", self.sensor_kind, gsd_required_m=gsd, side_overlap=side,
                                  forward_overlap=0.70, h_target_agl_m=h_target)

    def build_inputs(self, corners: Sequence[Any], params: Any, fleet: Sequence[UAVInstance]) -> MissionInputs:
        """角點 + params + 機隊 → :class:`mission.MissionInputs`（一個區域 `Z1`、sensor_req = `sensor_kind`）。"""
        pts = _latlon_list(corners)
        if len(pts) < 3:
            raise LegacyAdapterError("corners 至少 3 個角點")
        if not fleet:
            raise LegacyAdapterError("fleet 不可為空")
        p = _Params.read(params)
        home_ll = self._resolve_home(pts)
        home = Site("H", home_ll[0], home_ll[1], kind="home")
        alts = self._resolve_alternates(home_ll, pts)
        dem = self._dem_for(pts + [home_ll] + [a.latlon for a in alts])
        specs = [u.spec for u in fleet]
        hw_list = [h for u in fleet for h in u.sensors]
        req = self._payload_req(p, specs, hw_list)
        area = AreaSpec("Z1", tuple(pts), sensor_req=self.sensor_kind, priority=1.0, payload_req_id="default")
        speed, dir_from = self.wind
        wind = (WindLayer(0.0, 3000.0, speed, dir_from, gust_mps=speed * 1.3, z_ref="agl"),)
        c2 = C2Link(range_km=self.c2_range_km, coverage_polygon_wgs84=None, ground_station=home, lost_link_policy={})
        airspace = Airspace(max_agl_m=120.0, waiver_max_agl_m=self.waiver_max_agl_m, terrain_clearance_m=self.terrain_clearance_m)
        obs = WeatherObs(mean_wind_mps=speed, gust_mps=speed * 1.3, wind_dir_from_deg=dir_from, visibility_m=10000.0,
                         precip_mmh=0.0, temp_c=self.weather_temp_c)
        return MissionInputs(home=home, alternates=alts, areas=(area,), dem=dem, obstacles=(), wind=wind,
                             fleet=tuple(fleet), payload_reqs={"default": req}, c2=c2, airspace=airspace,
                             weather_thresholds=WeatherThresholds(), weather_obs=obs)

    # -- 規劃 --------------------------------------------------------------
    def plan(self, corners: Sequence[Any], params: Any, fleet: Sequence[UAVInstance]) -> MissionPlan:
        """組裝輸入並執行 :func:`planner.plan_mission`（含掃描方向覆寫）；NO_GO → :class:`LegacyAdapterError`。"""
        inputs = self.build_inputs(corners, params, fleet)
        p = _Params.read(params)
        alpha = legacy_angle_to_theta_rad(p.angle) if p.angle is not None else None
        mode = self.angle_mode
        with _direction_override(alpha, mode):
            mp = plan_mission(inputs, self.cfg, terrain_margin_m=self.terrain_margin_m)
        if mp.feasibility.status != "NO_GO" and alpha is not None and mode == "force" and (mp.uncovered or not mp.plans):
            self.warnings.append("forced_angle_infeasible_fallback_to_hint")
            with _direction_override(alpha, "hint"):
                mp = plan_mission(inputs, self.cfg, terrain_margin_m=self.terrain_margin_m)
        if mp.feasibility.status == "NO_GO":
            raise LegacyAdapterError("HV-DCCPP preflight NO_GO: " + "; ".join(mp.feasibility.errors), mp.feasibility.errors)
        self.last_inputs = inputs
        self.last_plan = mp
        self._last_frame = GeoFrame(inputs.home.lat_deg, inputs.home.lon_deg, self._home_msl(mp, inputs))
        self._last_home_msl = self._last_frame.h0_msl_m
        self.warnings.extend(f"plan:{w}" for w in mp.feasibility.warnings if w.startswith(("alternate_unusable", "airspace")))
        return mp

    @staticmethod
    def _home_msl(mp: MissionPlan, inputs: MissionInputs) -> float:
        for up in mp.plans.values():
            if up.waypoints:
                w = up.waypoints[0]
                return float(w.alt_msl_m - w.agl_m)
        z = float(inputs.dem.sample_msl(inputs.home.lat_deg, inputs.home.lon_deg)[0])
        return z if math.isfinite(z) else 0.0

    def _trigger_dist(self, uav: UAVInstance, req: PayloadRequirement, h_agl_m: float) -> float | None:
        cam = next((h for h in uav.sensors if isinstance(h, CameraHardware)), None)
        if cam is None:
            return None
        l_m = h_agl_m * cam.sensor_height_mm / cam.focal_length_mm
        return max(1.0, l_m * (1.0 - float(req.forward_overlap)))

    def _items_for(self, up: UAVPlan, uav: UAVInstance, inputs: MissionInputs, p: _Params, loiter_time: float) -> list[MissionItem]:
        frame = self._last_frame
        msl_h = float(self._last_home_msl or 0.0)
        oper = [w.agl_m for w in up.waypoints if w.phase is Phase.OPERATE]
        h_op = float(np.median(oper)) if oper else (p.altitude or 120.0)
        trig = self._trigger_dist(uav, inputs.payload_reqs["default"], h_op)
        self._last_trigger = trig
        landing = inputs.sites.get(up.landing_site) or inputs.home
        items = plan_to_mission_items(up, spec=uav.spec, home_latlon_msl=(inputs.home.lat_deg, inputs.home.lon_deg, msl_h),
                                      frame=frame, trigger_dist_m=trig, speed_cap_mps=p.speed, loiter_time_s=loiter_time,
                                      landing_latlon=(landing.lat_deg, landing.lon_deg), lidar_only=trig is None,
                                      tol_xy_m=self.cfg.e_max_m * 0.5, tol_z_m=3.0)
        self.last_items[up.uav_id] = items
        return items

    @staticmethod
    def _latlon_path(items: Sequence[MissionItem]) -> list[tuple[float, float]]:
        return [(float(it.x), float(it.y)) for it in items[1:] if it.is_nav]

    # -- 既有簽章 ------------------------------------------------------------
    def generate_complete_mission(self, corners: Sequence[Any], params: Any, region_idx: int, total_regions: int,
                                  start_from_left: bool = True, loiter_time: float = 0.0
                                  ) -> tuple[list[str], list[tuple[float, float]]]:
        """既有 `WaypointGenerator.generate_complete_mission` 之替換：單機（`self.platform`）覆蓋 `corners` 區域。

        `region_idx`/`total_regions` 只用於命名 UAV（`<cls><region_idx+1>`）；`start_from_left` 由 HV-DCCPP 自行選入口
        （忽略）；`loiter_time` > 0 → 起飛前 NAV_DELAY（既有系統之多機起飛錯開）。
        回傳 `(qgc_lines, latlon_path)`：QGC WPL 110 文字列與 NAV 項目之 `(lat, lon)` 序列（起飛點起、落點止）。
        """
        cls = self.spec.cls
        uav_id = f"{cls}{int(region_idx) + 1}"
        uav = self._uav(uav_id, self.platform)
        mp = self.plan(corners, params, [uav])
        up = mp.plans.get(uav_id)
        if up is None or not up.waypoints:
            raise LegacyAdapterError(f"UAV {uav_id} 無航線（uncovered={mp.uncovered}）", [str(u) for u in mp.uncovered])
        if mp.uncovered:
            self.warnings.append(f"uncovered({region_idx}): {mp.uncovered}")
        items = self._items_for(up, uav, self.last_inputs, _Params.read(params), float(loiter_time))  # type: ignore[arg-type]
        return mission_items_to_qgc(items), self._latlon_path(items)

    def plan_multi(self, corners: Sequence[Any], drones: Sequence[Mapping[str, Any]], params: Any = None, *,
                   loiter_time: float = 0.0) -> dict[str, tuple[list[str], list[tuple[float, float]]]]:
        """多機（異構）聯合規劃：`drones = [{"id", "platform", "energy"?, "hardware"?}, ...]` → `{id: (qgc_lines, latlon_path)}`。
        未獲派航線之機 → `([], [])`；各機起飛時刻差（`cfg.dt_dep_s` 排程）以 NAV_DELAY 表達（+ `loiter_time`）。"""
        fleet = [self._uav(str(d["id"]), str(d.get("platform", self.platform)), d.get("energy"), d.get("hardware")) for d in drones]
        if len({u.uav_id for u in fleet}) != len(fleet):
            raise LegacyAdapterError("drones id 重複")
        mp = self.plan(corners, params, fleet)
        p = _Params.read(params)
        out: dict[str, tuple[list[str], list[tuple[float, float]]]] = {}
        for uav in fleet:
            up = mp.plans.get(uav.uav_id)
            if up is None or not up.waypoints:
                out[uav.uav_id] = ([], [])
                continue
            delay = float(loiter_time) + max(0.0, float(up.t_takeoff_s))
            items = self._items_for(up, uav, self.last_inputs, p, delay)  # type: ignore[arg-type]
            out[uav.uav_id] = (mission_items_to_qgc(items), self._latlon_path(items))
        return out

    def waypoints_to_qgc_format(self, waypoints: Sequence[Any], params: Any, region_idx: int, total_regions: int,
                                loiter_time: float = 0.0) -> list[str]:
        """既有 `waypoints_to_qgc_format` 之替換（VTOL 正確）。`waypoints` 為 `(lat, lon)` 列（所有中間航點同
        `params.altitude`、相機觸發包住）或 :class:`mission.Waypoint` 列（走 :func:`plan_to_mission_items`）。"""
        p = _Params.read(params)
        if not waypoints:
            return [QGC_HEADER]
        if isinstance(waypoints[0], Waypoint):
            wps = list(waypoints)
            w0 = wps[0]
            msl_h = self._last_home_msl if self._last_home_msl is not None else float(w0.alt_msl_m - w0.agl_m)
            cam = next((h for h in self._hardware_for(self.spec) if isinstance(h, CameraHardware)), None)
            h_ref = p.altitude if p.altitude is not None else 120.0
            trig = self._last_trigger if self._last_trigger is not None else (
                None if cam is None else max(1.0, h_ref * cam.sensor_height_mm / cam.focal_length_mm * 0.3))
            items = plan_to_mission_items(wps, spec=self.spec, home_latlon_msl=(w0.lat_deg, w0.lon_deg, msl_h),
                                          frame=self._last_frame, trigger_dist_m=trig, speed_cap_mps=p.speed,
                                          loiter_time_s=float(loiter_time), lidar_only=trig is None)
        else:
            pts = _latlon_list(waypoints)
            cam = next((h for h in self._hardware_for(self.spec) if isinstance(h, CameraHardware)), None)
            alt = p.altitude if p.altitude is not None else float(self.spec.min_fw_operation_agl_m)
            trig = 30.0 if cam is None else max(1.0, alt * cam.sensor_height_mm / cam.focal_length_mm * 0.3)
            items = simple_vtol_mission_items(pts, spec=self.spec, altitude_agl_m=alt,
                                              speed_mps=p.speed if p.speed is not None else self.spec.cruise_airspeed_mps,
                                              trigger_dist_m=trig, loiter_time_s=float(loiter_time))
        return mission_items_to_qgc(items)

    def subdivide_polygon(self, corners: Sequence[Any], n: int, spacing_m: float = 0.0) -> list[list[tuple[float, float]]]:
        """既有 `RegionDivider.subdivide_polygon` 之替換：:func:`geometry.convex_partition`（規格 §7.2）後依面積分成 `n` 組。

        片數不足 `n` 時以面積二分（軸向直線）補足；各組取聯集（不連通 → 凸包）；回傳每組 `(lat, lon)` 外環。
        `spacing_m` → 凸分割之 s_ref（≤ 0 → max(50 m, √A/20)）。
        """
        pts = _latlon_list(corners)
        n = max(1, int(n))
        if len(pts) < 3:
            return [pts]
        if n == 1:
            return [pts]
        lat0 = sum(p[0] for p in pts) / len(pts)
        lon0 = sum(p[1] for p in pts) / len(pts)
        frame = GeoFrame(lat0, lon0, 0.0)
        xy = frame.to_enu_xy([p[0] for p in pts], [p[1] for p in pts])
        poly = Polygon(xy).buffer(0)
        parts = _polys_of(poly)
        if not parts:
            return [pts]
        poly = max(parts, key=lambda g: g.area)
        s_ref = float(spacing_m) if spacing_m and spacing_m > 0 else max(50.0, math.sqrt(poly.area) / 20.0)
        parts = list(convex_partition(poly, s_ref, 0.5 * s_ref))
        parts = [g for g in parts if g.area > 1.0] or [poly]
        while len(parts) < n:
            parts.sort(key=lambda g: g.area, reverse=True)
            big = parts.pop(0)
            halves = _bisect_polygon(big)
            if len(halves) < 2:
                parts.append(big)
                break
            parts.extend(halves)
        groups = _group_by_area(parts, n)
        out: list[list[tuple[float, float]]] = []
        for grp in groups:
            u = shapely.union_all(grp).buffer(0)
            g = u if isinstance(u, Polygon) else u.convex_hull
            ring = np.asarray(g.exterior.coords, dtype=float)[:-1]
            w = frame.to_wgs84(np.column_stack([ring, np.zeros(len(ring))]))
            out.append([(float(a), float(b)) for a, b in w[:, :2]])
        return out

    def subdivide_rectangle(self, corners: Sequence[Any], n: int, spacing_m: float = 0.0) -> list[list[tuple[float, float]]]:
        """既有 `RegionDivider.subdivide_rectangle` 之替換（委派 :meth:`subdivide_polygon`）。"""
        return self.subdivide_polygon(corners, n, spacing_m)
