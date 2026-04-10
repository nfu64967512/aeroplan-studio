"""
DCCPP 路徑建構器 v2
====================
取代舊 ``core.trajectory.dccpp_path_assembler`` 的核心組裝邏輯。

設計原則
--------
1. 全程使用真實 ``DubinsTrajectoryGenerator.calculate_path``。當相鄰掃描線
   間距小於 4·R_min 時自動退化為 CCC (RLR/LRL)，間距 < 2·R_min 時退化為
   loiter — 這正是論文 Fig. 5(c) 中那種「外凸 U-turn loop」的視覺特徵。
2. 嚴格遵守論文 Eq. (24)–(26) 的方向約束：transfer 的離開方向必須等於前
   一段 op 的飛行方向，進入方向必須等於下一段 op 的飛行方向。
3. 座標統一在 ENU local meters 計算，最後一次轉回 lat/lon 輸出。

呼叫範例
--------
    builder = DCCPPPathBuilder(min_turn_radius=80.0, default_altitude=120.0,
                               step_size=10.0)
    built = builder.build(
        start_lat, start_lon, start_heading_compass_deg,
        ordered_scan_lines,                # List[ScanLine]
        directions,                        # List[int] +1=L→R / -1=R→L
        transformer,                       # CoordinateTransformer (與 ScanLine 共用)
    )
"""

from __future__ import annotations

import math
import logging
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Sequence, Tuple

import numpy as np

from core.dccpp.area_processor import ScanLine

logger = logging.getLogger(__name__)


# ============================================================
# 資料結構
# ============================================================

class SegmentLabel(Enum):
    TAKEOFF = auto()
    ENTRY = auto()
    OPERATION = auto()
    TRANSFER = auto()
    LANDING = auto()


@dataclass
class BuiltWaypoint:
    lat: float
    lon: float
    alt: float
    heading_compass_deg: float
    segment_type: SegmentLabel

    def to_tuple(self) -> Tuple[float, float]:
        return (self.lat, self.lon)


@dataclass
class BuiltPath:
    uav_id: int
    waypoints: List[BuiltWaypoint] = field(default_factory=list)
    entry_length_m: float = 0.0
    operation_length_m: float = 0.0
    transfer_length_m: float = 0.0

    @property
    def total_length_m(self) -> float:
        return self.entry_length_m + self.operation_length_m + self.transfer_length_m

    def to_latlon(self) -> List[Tuple[float, float]]:
        return [w.to_tuple() for w in self.waypoints]


# ============================================================
# 核心建構器
# ============================================================

# compass (0=N, CW) ↔ math (0=E, CCW)
def _compass_to_math(c: float) -> float:
    return (90.0 - c) % 360.0


def _math_to_compass(m: float) -> float:
    return (90.0 - m) % 360.0


class DCCPPPathBuilder:
    """論文圖 5(a)(b)(c) 對應的路徑生成器。

    Args:
        min_turn_radius   : R_min [m]，必須 > 0
        default_altitude  : 預設飛行高度 [m]
        step_size         : Dubins 曲線離散步長 [m]
        max_climb_deg     : Dubins 模組爬升角上限
    """

    def __init__(
        self,
        min_turn_radius: float,
        default_altitude: float = 100.0,
        step_size: float = 10.0,
        max_climb_deg: float = 15.0,
    ) -> None:
        if min_turn_radius <= 0:
            raise ValueError(f"min_turn_radius 必須 > 0，實際 {min_turn_radius}")
        self.R = float(min_turn_radius)
        self.alt = float(default_altitude)
        self.step = float(step_size)
        self._dubins = self._make_dubins(min_turn_radius, max_climb_deg)

    @staticmethod
    def _make_dubins(R: float, max_climb_deg: float):
        from core.base.fixed_wing_constraints import FixedWingConstraints
        from core.trajectory.dubins_trajectory import DubinsTrajectoryGenerator
        # 反推空速：R = V²/(g·tan(φ))·safety
        phi_deg = 45.0
        safety = 1.2
        g = 9.81
        v = math.sqrt(R * g * math.tan(math.radians(phi_deg)) / safety)
        c = FixedWingConstraints(
            cruise_airspeed_mps=v,
            max_bank_angle_deg=phi_deg,
            safety_factor=safety,
        )
        return DubinsTrajectoryGenerator(c, max_climb_angle_deg=max_climb_deg)

    # ────────────────────────────────────────────────────────
    # 主入口：建構單架 UAV 的完整路徑
    # ────────────────────────────────────────────────────────
    def build(
        self,
        start_lat: float,
        start_lon: float,
        start_heading_compass_deg: float,
        ordered_scan_lines: Sequence[ScanLine],
        directions: Sequence[int],
        transformer,
        uav_id: int = 0,
    ) -> BuiltPath:
        """生成從起點 → 依序掃描所有 ordered_scan_lines 的完整航點序列。

        Args:
            start_lat / start_lon / start_heading_compass_deg : UAV 起點
            ordered_scan_lines : 已排好順序的 ScanLine
            directions         : 對應每條 scan line 的飛行方向 (+1 = pL→pR, -1 = pR→pL)
            transformer        : CoordinateTransformer (與 ScanLine 公尺座標一致)
            uav_id             : 寫入 BuiltPath
        """
        from core.trajectory.dubins_trajectory import Pose3D

        path = BuiltPath(uav_id=uav_id)
        if not ordered_scan_lines:
            return path

        if len(directions) != len(ordered_scan_lines):
            raise ValueError("directions 與 scan_lines 長度不一致")

        # ── 起始 pose (ENU + math heading) ──
        s_enu = transformer.geo_to_local(start_lat, start_lon)
        prev_x, prev_y = float(s_enu[0]), float(s_enu[1])
        prev_h_math = _compass_to_math(start_heading_compass_deg)
        prev_pose = Pose3D(x=prev_x, y=prev_y, z=self.alt, heading_deg=prev_h_math)

        for i, sl in enumerate(ordered_scan_lines):
            d = directions[i]
            if d not in (-1, 1):
                raise ValueError(f"direction 必須是 ±1，實際 {d}")

            # 確定本段 entry / exit (ENU)
            if d == 1:
                ex_xy, en_exit = sl.pL_xy, sl.pR_xy
                op_h_math = _compass_to_math(sl.heading_compass_deg)
            else:
                ex_xy, en_exit = sl.pR_xy, sl.pL_xy
                op_h_math = _compass_to_math((sl.heading_compass_deg + 180.0) % 360.0)

            entry_pose = Pose3D(x=ex_xy[0], y=ex_xy[1], z=self.alt, heading_deg=op_h_math)
            exit_pose = Pose3D(x=en_exit[0], y=en_exit[1], z=self.alt, heading_deg=op_h_math)

            # ── 1. Entry / Transfer Dubins ──
            label = SegmentLabel.ENTRY if i == 0 else SegmentLabel.TRANSFER
            wps_xy, dub_len = self._dubins_waypoints(prev_pose, entry_pose)
            for (wx, wy, wh_math) in wps_xy:
                lat, lon = self._xy_to_latlon(transformer, wx, wy)
                path.waypoints.append(BuiltWaypoint(
                    lat=lat, lon=lon, alt=self.alt,
                    heading_compass_deg=_math_to_compass(wh_math),
                    segment_type=label,
                ))
            if i == 0:
                path.entry_length_m += dub_len
            else:
                path.transfer_length_m += dub_len

            # ── 2. Operation 直線段 (離散) ──
            op_wps = self._straight_waypoints(ex_xy, en_exit, op_h_math)
            for (wx, wy, wh_math) in op_wps:
                lat, lon = self._xy_to_latlon(transformer, wx, wy)
                path.waypoints.append(BuiltWaypoint(
                    lat=lat, lon=lon, alt=self.alt,
                    heading_compass_deg=_math_to_compass(wh_math),
                    segment_type=SegmentLabel.OPERATION,
                ))
            path.operation_length_m += sl.length_m

            prev_pose = exit_pose

        return path

    # ────────────────────────────────────────────────────────
    # Dubins waypoints
    # ────────────────────────────────────────────────────────
    def _dubins_waypoints(
        self, start_pose, end_pose
    ) -> Tuple[List[Tuple[float, float, float]], float]:
        """呼叫真實 Dubins 並離散化。
        轉彎段 (LEFT/RIGHT) 以 step_size 密集取樣維持曲線形狀；
        直線段 (STRAIGHT) 僅保留終點，避免產生大量共線航點。
        """
        from core.trajectory.dubins_trajectory import SegmentType, rad_to_deg, normalize_angle
        try:
            dpath = self._dubins.calculate_path(start_pose, end_pose)
            if not dpath.is_feasible:
                logger.warning(f"Dubins infeasible: {dpath.warning}")
                return self._fallback_line(start_pose, end_pose)

            wps: List[Tuple[float, float, float]] = []
            for seg in dpath.segments:
                if seg.length < 1e-6:
                    continue
                if seg.seg_type == SegmentType.STRAIGHT:
                    # 直線段只保留終點
                    hdg = seg.angle_start_rad
                    x_end = seg.center_x + seg.length * math.cos(hdg)
                    y_end = seg.center_y + seg.length * math.sin(hdg)
                    wps.append((x_end, y_end, normalize_angle(rad_to_deg(hdg), 0.0)))
                else:
                    # 弧段按 step_size 取樣（不含起點，避免重複）
                    n = max(2, int(math.ceil(seg.length / self.step)))
                    for i in range(1, n + 1):
                        frac = i / n
                        angle = seg.angle_start_rad + frac * seg.angle_sweep_rad
                        x = seg.center_x + seg.radius * math.cos(angle)
                        y = seg.center_y + seg.radius * math.sin(angle)
                        if seg.seg_type == SegmentType.LEFT_TURN:
                            hdg_deg = rad_to_deg(angle + math.pi / 2)
                        else:
                            hdg_deg = rad_to_deg(angle - math.pi / 2)
                        wps.append((x, y, normalize_angle(hdg_deg, 0.0)))
            # 確保終點精確落在 end_pose
            if wps:
                wps[-1] = (end_pose.x, end_pose.y,
                           normalize_angle(end_pose.heading_deg, 0.0))
            return wps, float(dpath.total_length)
        except Exception as e:
            logger.warning(f"Dubins exception, fallback to line: {e}")
            return self._fallback_line(start_pose, end_pose)

    @staticmethod
    def _fallback_line(s, e) -> Tuple[List[Tuple[float, float, float]], float]:
        L = math.hypot(e.x - s.x, e.y - s.y)
        return [(e.x, e.y, e.heading_deg)], L

    def _straight_waypoints(
        self, p0_xy: Tuple[float, float], p1_xy: Tuple[float, float], heading_math: float,
    ) -> List[Tuple[float, float, float]]:
        # 直線段只保留起點與終點，避免飛控收到過密的中間航點
        L = math.hypot(p1_xy[0] - p0_xy[0], p1_xy[1] - p0_xy[1])
        if L < 1e-3:
            return [(p1_xy[0], p1_xy[1], heading_math)]
        return [
            (p0_xy[0], p0_xy[1], heading_math),
            (p1_xy[0], p1_xy[1], heading_math),
        ]

    # ────────────────────────────────────────────────────────
    # Takeoff / Landing 補丁（讓 ArduPlane AUTO 能順利執行）
    # ────────────────────────────────────────────────────────
    def prepend_takeoff(
        self,
        built: "BuiltPath",
        home_lat: float,
        home_lon: float,
        runway_bearing_compass_deg: float,
        climb_distance_m: Optional[float] = None,
    ) -> None:
        """在 BuiltPath 最前面插入 [HOME(alt=0), CLIMB_OUT(alt=cruise)] 兩個
        TAKEOFF 航點。匯出時 idx=0 變成 DO_SET_HOME，idx=1 變成 NAV_TAKEOFF。
        """
        if climb_distance_m is None:
            climb_distance_m = max(8.0 * self.R, 400.0)

        # 階梯爬升：HOME(0) → 若干中繼點 → 巡航高度
        # 每段爬升高度 ~ 25m，最少 3 段最多 6 段
        cruise_alt = float(self.alt)
        n_steps = max(3, min(6, int(round(cruise_alt / 25.0))))

        bearing_rad = math.radians(runway_bearing_compass_deg)
        cos_lat = max(math.cos(math.radians(home_lat)), 1e-6)

        def _offset(dist_m):
            dN = dist_m * math.cos(bearing_rad)
            dE = dist_m * math.sin(bearing_rad)
            return (home_lat + dN / 111320.0,
                    home_lon + dE / (111320.0 * cos_lat))

        takeoff_wps = [BuiltWaypoint(
            lat=home_lat, lon=home_lon, alt=0.0,
            heading_compass_deg=runway_bearing_compass_deg,
            segment_type=SegmentLabel.TAKEOFF,
        )]
        # NAV_TAKEOFF 目標：第 1 個爬升點（距離 = climb_distance/n_steps，
        # 高度 = cruise/n_steps），讓 ArduPlane 從地面平順起飛
        for i in range(1, n_steps + 1):
            frac = i / n_steps
            lat_i, lon_i = _offset(climb_distance_m * frac)
            alt_i = cruise_alt * frac
            takeoff_wps.append(BuiltWaypoint(
                lat=lat_i, lon=lon_i, alt=alt_i,
                heading_compass_deg=runway_bearing_compass_deg,
                segment_type=SegmentLabel.TAKEOFF,
            ))
        built.waypoints = takeoff_wps + built.waypoints

    def append_landing(
        self,
        built: "BuiltPath",
        home_lat: float,
        home_lon: float,
        runway_bearing_compass_deg: float,
        approach_distance_m: Optional[float] = None,
        approach_alt_m: Optional[float] = None,
    ) -> None:
        """在 BuiltPath 最後追加 [APPROACH(alt=approach), TOUCHDOWN(alt=0)]
        兩個 LANDING 航點。匯出時最後一點變成 NAV_LAND。
        """
        if approach_distance_m is None:
            approach_distance_m = max(6.0 * self.R, 300.0)
        if approach_alt_m is None:
            approach_alt_m = max(self.alt * 0.3, 30.0)

        # approach 點在 home 的「逆 runway_bearing」方向（即 final 進場起點）
        cruise_alt = float(self.alt)
        rev_rad = math.radians((runway_bearing_compass_deg + 180.0) % 360.0)
        cos_lat = max(math.cos(math.radians(home_lat)), 1e-6)

        def _rev_offset(dist_m):
            dN = dist_m * math.cos(rev_rad)
            dE = dist_m * math.sin(rev_rad)
            return (home_lat + dN / 111320.0,
                    home_lon + dE / (111320.0 * cos_lat))

        # 階梯下降：從 cruise_alt 逐步降到 approach_alt（約 30m）再 TOUCHDOWN(0)
        n_steps = max(3, min(6, int(round(cruise_alt / 25.0))))
        landing_wps = []
        # 下降點從最遠端開始（距 home 最遠 = approach_distance * n_steps/n_steps）
        # 每段距離 = approach_distance / n_steps
        # 高度從 cruise_alt 線性降到 approach_alt_m
        for i in range(n_steps):
            frac = i / n_steps  # 0 .. (n-1)/n
            # 距離從 approach_distance 線性遞減到 approach_distance/n_steps
            dist = approach_distance_m * (1.0 - frac * (1.0 - 1.0 / n_steps))
            lat_i, lon_i = _rev_offset(dist)
            alt_i = cruise_alt - (cruise_alt - approach_alt_m) * frac
            landing_wps.append(BuiltWaypoint(
                lat=lat_i, lon=lon_i, alt=alt_i,
                heading_compass_deg=runway_bearing_compass_deg,
                segment_type=SegmentLabel.LANDING,
            ))
        # 最終進場點 (approach_alt) + 觸地 (0)
        appr_lat, appr_lon = _rev_offset(approach_distance_m / n_steps * 0.5)
        landing_wps.append(BuiltWaypoint(
            lat=appr_lat, lon=appr_lon, alt=approach_alt_m,
            heading_compass_deg=runway_bearing_compass_deg,
            segment_type=SegmentLabel.LANDING,
        ))
        landing_wps.append(BuiltWaypoint(
            lat=home_lat, lon=home_lon, alt=0.0,
            heading_compass_deg=runway_bearing_compass_deg,
            segment_type=SegmentLabel.LANDING,
        ))
        built.waypoints.extend(landing_wps)

    @staticmethod
    def _xy_to_latlon(transformer, x: float, y: float) -> Tuple[float, float]:
        # CoordinateTransformer 用 ENU: x=East, y=North → local_to_geo(east, north)
        g = transformer.local_to_geo(x, y)
        return (g.latitude, g.longitude)
