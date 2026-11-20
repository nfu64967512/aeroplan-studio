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
    #: 起降段航跡長度。⚠ 跑道起降的階梯點是既有 golden 的一部分，沿用舊行為
    #: 不累加；只有 VTOL 剖面會填這兩欄。不累加的話 total_length_m 少算整段
    #: 起降，makespan 就會比最慢那一架還短。
    takeoff_length_m: float = 0.0
    landing_length_m: float = 0.0

    @property
    def total_length_m(self) -> float:
        return (self.entry_length_m + self.operation_length_m
                + self.transfer_length_m
                + self.takeoff_length_m + self.landing_length_m)

    def to_latlon(self) -> List[Tuple[float, float]]:
        return [w.to_tuple() for w in self.waypoints]


# ============================================================
# 飛行時間估算
# ============================================================

#: 水平位移小於這個值就當成「原地垂直」段（多旋翼懸停爬升／下降）。
#: 起降剖面的垂直段本來就是同一組經緯度、水平位移恰好 0；留 1 m 是給
#: 下游的重採樣與避撞偏移一點浮點餘裕。
_HOVER_LEG_EPS_M = 1.0


def estimate_flight_time_s(path, cruise_speed_mps: float,
                           vertical_speed_mps: float) -> float:
    """估算一條航線的飛行時間（秒）。

    每一段依它**實際上是哪一種飛行**計時：

      * 純垂直段（水平位移 ≈ 0）＝ 多旋翼懸停爬升／下降 ⇒ ``垂直距離 / 垂直速度``
      * 其餘 ⇒ ``三維航跡長度 / 巡航速度``

    ⚠ 為什麼不能直接用 ``total_length_m / cruise_speed``：VTOL 的起降段是**純垂直**的，
    水平長度 0、垂直 60 m。用巡航速度計時會把 2.5 m/s 的懸停爬升算成 40 m/s
    ——60 m 的爬升從 24 秒變成 1.5 秒，一趟起降就低估了 45 秒。

    ⚠⚠ 為什麼不是每段都取 ``max(水平/巡航, 垂直/垂直速度)``：那樣會把**固定翼**
    爬升段也用懸停爬升率計時。實測 40 m/s、TECS 爬升率 5 m/s 的爬升段坡度是 8:1，
    而 ``max`` 的分界在 16:1 ⇒ 整段固定翼爬升被當成懸停，一趟起降多算 56 秒，
    而且「轉換高度拉高」會變成任務時間**變短**（固定翼爬升段縮短的量比垂直段
    增加的量還大）—— 方向都反了。固定翼是沿航跡以巡航速度飛，不是垂直爬升。

    這支與 `ui/main_window.py` 偵察頁摘要表、makespan 顯示共用，兩邊才不會出現兩個數字。

    Args:
        path:               具 ``waypoints``（每點有 lat/lon/alt）的路徑物件
        cruise_speed_mps:   巡航（水平）速度，m/s
        vertical_speed_mps: 垂直速度（多旋翼段爬升／下降率），m/s

    Returns:
        秒。航點少於 2 個時回 0.0。
    """
    wps = list(getattr(path, 'waypoints', None) or [])
    if len(wps) < 2:
        return 0.0
    v_h = max(float(cruise_speed_mps), 1e-6)
    v_v = max(float(vertical_speed_mps), 1e-6)
    total = 0.0
    for a, b in zip(wps, wps[1:]):
        lat1, lon1 = float(a.lat), float(a.lon)
        lat2, lon2 = float(b.lat), float(b.lon)
        d_north = (lat2 - lat1) * 111320.0
        d_east = (lon2 - lon1) * 111320.0 * math.cos(math.radians((lat1 + lat2) / 2.0))
        horiz = math.hypot(d_north, d_east)
        vert = abs(float(getattr(b, 'alt', 0.0)) - float(getattr(a, 'alt', 0.0)))
        if horiz <= _HOVER_LEG_EPS_M:
            total += vert / v_v          # 多旋翼段：垂直爬升／下降
        else:
            total += math.hypot(horiz, vert) / v_h   # 固定翼段：沿航跡巡航
    return total


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
            # Entry / Transfer 都簡化為僅終點，飛控透過 WP 觸發半徑自行轉彎
            wps_xy, dub_len = self._dubins_waypoints(
                prev_pose, entry_pose, simplify=True
            )
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
        self, start_pose, end_pose, simplify: bool = False,
    ) -> Tuple[List[Tuple[float, float, float]], float]:
        """呼叫真實 Dubins 並離散化。

        Args:
            simplify: 若為 True，僅保留終點（不做弧段密集取樣），
                      適用於 TRANSFER 段——飛控可透過 WP 觸發半徑自行轉彎。
        """
        from core.trajectory.dubins_trajectory import SegmentType, rad_to_deg, normalize_angle
        try:
            dpath = self._dubins.calculate_path(start_pose, end_pose)
            if not dpath.is_feasible:
                logger.warning(f"Dubins infeasible: {dpath.warning}")
                return self._fallback_line(start_pose, end_pose)

            # 簡化模式：轉移段只保留終點，由飛控 WP 觸發半徑處理轉彎
            if simplify:
                end_hdg = normalize_angle(end_pose.heading_deg, 0.0)
                return [(end_pose.x, end_pose.y, end_hdg)], float(dpath.total_length)

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
        landing_rollout_m: float = 0.0,
    ) -> None:
        """在 BuiltPath 最後追加 [APPROACH(alt=approach), TOUCHDOWN(alt=0)]
        兩個 LANDING 航點。匯出時最後一點變成 NAV_LAND。

        landing_rollout_m: 觸地後滑行煞車距離（公尺）。
            若 > 0，觸地點會沿進場反方向提前此距離，
            確保飛機減速停止時不超過跑道末端（Home 點）。
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

        # 觸地點：若有滑行距離則提前，確保煞車後不超過跑道末端
        rollout = landing_rollout_m or 0.0
        if rollout > 0:
            td_lat, td_lon = _rev_offset(rollout)
        else:
            td_lat, td_lon = home_lat, home_lon

        # 整條進場路徑改以「觸地點」為錨點，
        # 確保下降階梯與最終進場點隨 rollout 一起平移。
        # 否則只有 td 被往後推、五邊進場仍貼在 home 附近。
        def _td_rev_offset(dist_m):
            dN = dist_m * math.cos(rev_rad)
            dE = dist_m * math.sin(rev_rad)
            return (td_lat + dN / 111320.0,
                    td_lon + dE / (111320.0 * cos_lat))

        # 階梯下降：從 cruise_alt 逐步降到 approach_alt（約 30m）再 TOUCHDOWN(0)
        n_steps = max(3, min(6, int(round(cruise_alt / 25.0))))
        landing_wps = []
        # 下降點從最遠端開始（距觸地點最遠 = approach_distance）
        # 每段距離 = approach_distance / n_steps
        # 高度從 cruise_alt 線性降到 approach_alt_m
        for i in range(n_steps):
            frac = i / n_steps  # 0 .. (n-1)/n
            # 距離從 approach_distance 線性遞減到 approach_distance/n_steps
            dist = approach_distance_m * (1.0 - frac * (1.0 - 1.0 / n_steps))
            lat_i, lon_i = _td_rev_offset(dist)
            alt_i = cruise_alt - (cruise_alt - approach_alt_m) * frac
            landing_wps.append(BuiltWaypoint(
                lat=lat_i, lon=lon_i, alt=alt_i,
                heading_compass_deg=runway_bearing_compass_deg,
                segment_type=SegmentLabel.LANDING,
            ))
        # 最終進場點 (approach_alt) + 觸地 (0)
        appr_lat, appr_lon = _td_rev_offset(approach_distance_m / n_steps * 0.5)
        landing_wps.append(BuiltWaypoint(
            lat=appr_lat, lon=appr_lon, alt=approach_alt_m,
            heading_compass_deg=runway_bearing_compass_deg,
            segment_type=SegmentLabel.LANDING,
        ))
        landing_wps.append(BuiltWaypoint(
            lat=td_lat, lon=td_lon, alt=0.0,
            heading_compass_deg=runway_bearing_compass_deg,
            segment_type=SegmentLabel.LANDING,
        ))
        built.waypoints.extend(landing_wps)

    # ────────────────────────────────────────────────────────
    # 4+1 QuadPlane 垂直起降剖面
    # ────────────────────────────────────────────────────────
    # ⚠️⚠️⚠️ 這兩支與上面的跑道版**不是**同一件事，不可互相退回：
    #   跑道版第一段一定有水平位移（滑跑），VTOL 版第一段一定沒有（懸停爬升）。
    #   幾何全部來自 core.dccpp.vtol_profile.VTOLProfile，那裡的常數與
    #   mission/sitl_launcher.generate_vtol_params() 寫進 SITL 的值同步。

    @staticmethod
    def _bearing_offset(lat: float, lon: float,
                        bearing_compass_deg: float, dist_m: float):
        """沿 compass 方位角平移 dist_m。

        用與 :meth:`prepend_takeoff` 相同的平面近似（每度 111320 m），
        兩條路徑的座標慣例才不會各說各話。
        """
        rad = math.radians(bearing_compass_deg)
        cos_lat = max(math.cos(math.radians(lat)), 1e-6)
        return (lat + dist_m * math.cos(rad) / 111320.0,
                lon + dist_m * math.sin(rad) / (111320.0 * cos_lat))

    @staticmethod
    def _polyline_3d_length_m(wps: List["BuiltWaypoint"]) -> float:
        """航點列的三維航跡長度。

        ⚠ 垂直段的水平長度是 0，只用水平距離累加會把整段懸停爬升算成 0 m。
        """
        total = 0.0
        for a, b in zip(wps, wps[1:]):
            cos_lat = math.cos(math.radians((a.lat + b.lat) / 2.0))
            dN = (b.lat - a.lat) * 111320.0
            dE = (b.lon - a.lon) * 111320.0 * cos_lat
            dU = b.alt - a.alt
            total += math.sqrt(dN * dN + dE * dE + dU * dU)
        return total

    def _climb_steps(self, cruise_alt: float) -> int:
        """階梯段數，與跑道版同一條規則（每段約 25 m，夾在 3–6 段）。"""
        return max(3, min(6, int(round(float(cruise_alt) / 25.0))))

    def prepend_vtol_takeoff(
        self,
        built: "BuiltPath",
        home_lat: float,
        home_lon: float,
        bearing_compass_deg: float,
        transition_alt_m: float,
        cruise_speed_mps: Optional[float] = None,
        cruise_alt_m: Optional[float] = None,
        climb_path_angle_deg: Optional[float] = None,
    ) -> None:
        """在最前面插入 4+1 QuadPlane 的垂直起飛剖面。

        產生的 TAKEOFF 航點依序是::

            [0] Home 地面 (alt=0)
            [1] Home 正上方 (alt=轉換高度)      ← 純垂直，與 [0] 同經緯度
            [2] 前向過渡終點 (alt=轉換高度)      ← 等高，長度隨巡航速度變
            [3..] 固定翼階梯爬升 → 巡航高度

        匯出時 [0] 變 DO_SET_HOME、垂直段變 NAV_VTOL_TAKEOFF。

        Args:
            bearing_compass_deg: 起飛離場方位角（0=北，順時針）
            transition_alt_m:    MC→FW 轉換高度（面板「轉換高度」）
            cruise_speed_mps:    固定翼段巡航空速；``None`` 用飛控預設
            cruise_alt_m:        巡航高度；``None`` 用建構器的 default_altitude
            climb_path_angle_deg: 機體實際爬升航跡角；``None`` 才退回飛控
                                 ``TECS_CLMB_MAX``（兩者水平距離差約 4 倍）
        """
        from core.dccpp.vtol_profile import AIRSPEED_CRUISE, VTOLProfile

        cruise_alt = float(self.alt if cruise_alt_m is None else cruise_alt_m)
        speed = float(AIRSPEED_CRUISE if cruise_speed_mps is None else cruise_speed_mps)
        prof = VTOLProfile(
            transition_alt_m=float(transition_alt_m),
            cruise_alt_m=cruise_alt,
            cruise_speed_mps=speed,
            climb_path_angle_deg=climb_path_angle_deg,
        )
        hdg = float(bearing_compass_deg) % 360.0

        def _wp(lat, lon, alt):
            return BuiltWaypoint(lat=lat, lon=lon, alt=alt,
                                 heading_compass_deg=hdg,
                                 segment_type=SegmentLabel.TAKEOFF)

        # [0][1] 純垂直：同經緯度，只有高度不同
        wps = [_wp(home_lat, home_lon, 0.0),
               _wp(home_lat, home_lon, prof.transition_alt_m)]

        # [2] 前向過渡：等高
        d_trans = prof.forward_transition_m
        t_lat, t_lon = self._bearing_offset(home_lat, home_lon, hdg, d_trans)
        wps.append(_wp(t_lat, t_lon, prof.transition_alt_m))

        # [3..] 固定翼階梯爬升
        d_climb = prof.fw_climb_distance_m
        if prof.climb_height_m > 0.0 and d_climb > 0.0:
            n = self._climb_steps(cruise_alt)
            for i in range(1, n + 1):
                frac = i / n
                lat_i, lon_i = self._bearing_offset(
                    home_lat, home_lon, hdg, d_trans + d_climb * frac)
                alt_i = prof.transition_alt_m + prof.climb_height_m * frac
                wps.append(_wp(lat_i, lon_i, alt_i))
        # 最後一點必須**恰好**是巡航高度（frac=1 的浮點誤差會讓 3D 圖差一點）
        wps[-1].alt = cruise_alt

        # ⚠ 累加的是「這次注入**讓航跡多出多少**」＝ 注入段本身 + 接回原路徑的那一腿。
        #   少算接回那一腿，total_length_m 就會比實際飛的短一大截，makespan 跟著低估。
        tail = built.waypoints
        built.takeoff_length_m += self._polyline_3d_length_m(
            wps + tail[:1] if tail else wps)
        built.waypoints = wps + tail

    def append_vtol_landing(
        self,
        built: "BuiltPath",
        home_lat: float,
        home_lon: float,
        bearing_compass_deg: float,
        transition_alt_m: float,
        cruise_speed_mps: Optional[float] = None,
        cruise_alt_m: Optional[float] = None,
        descent_path_angle_deg: Optional[float] = None,
    ) -> None:
        """在最後追加 4+1 QuadPlane 的垂直降落剖面。

        產生的 LANDING 航點依序是::

            [0..] 固定翼階梯下降 → 轉換高度
            [-3]  最終進場點 (alt=轉換高度)，距 Home = land_approach_m
            [-2]  Home 正上方 (alt=轉換高度)   ← 3D 顯示的垂直下降段起點
            [-1]  Home 地面 (alt=0)            ← 純垂直，與 [-2] 同經緯度

        ⚠ ``land_approach_m`` 用的是**飛控自己的** AIRSPEED_CRUISE 算煞停距離，
        不是面板速度 —— PreArm「VTOL land too short」檢查的就是它。
        ⚠ [-2] 那個懸停點只給畫面用；匯出時必須跳過，否則
        NAV_VTOL_LAND 的進場段會變成 0 m 而被 PreArm 擋下。

        Args:
            bearing_compass_deg: 最終進場**航向**（飛機朝這個方位飛向 Home）
        """
        from core.dccpp.vtol_profile import AIRSPEED_CRUISE, VTOLProfile

        cruise_alt = float(self.alt if cruise_alt_m is None else cruise_alt_m)
        speed = float(AIRSPEED_CRUISE if cruise_speed_mps is None else cruise_speed_mps)
        prof = VTOLProfile(
            transition_alt_m=float(transition_alt_m),
            cruise_alt_m=cruise_alt,
            cruise_speed_mps=speed,
            descent_path_angle_deg=descent_path_angle_deg,
        )
        hdg = float(bearing_compass_deg) % 360.0
        back = (hdg + 180.0) % 360.0        # 進場點在 Home 的「來向」

        def _wp(lat, lon, alt):
            return BuiltWaypoint(lat=lat, lon=lon, alt=alt,
                                 heading_compass_deg=hdg,
                                 segment_type=SegmentLabel.LANDING)

        d_appr = prof.land_approach_m
        d_desc = prof.fw_descent_distance_m
        wps: List[BuiltWaypoint] = []

        # 階梯下降：由遠而近，高度 cruise → 轉換高度
        if prof.climb_height_m > 0.0 and d_desc > 0.0:
            n = self._climb_steps(cruise_alt)
            for i in range(n):
                frac = i / n
                lat_i, lon_i = self._bearing_offset(
                    home_lat, home_lon, back, d_appr + d_desc * (1.0 - frac))
                alt_i = cruise_alt - prof.climb_height_m * frac
                wps.append(_wp(lat_i, lon_i, alt_i))

        # 最終進場點（PreArm 量的就是這一段到降落點的距離）
        a_lat, a_lon = self._bearing_offset(home_lat, home_lon, back, d_appr)
        wps.append(_wp(a_lat, a_lon, prof.transition_alt_m))
        # 降落點正上方的懸停點 + 觸地點：純垂直
        wps.append(_wp(home_lat, home_lon, prof.transition_alt_m))
        wps.append(_wp(home_lat, home_lon, 0.0))

        # 同上：接上原路徑最後一點的那一腿也是這次注入造成的
        head = built.waypoints[-1:]
        built.landing_length_m += self._polyline_3d_length_m(head + wps)
        built.waypoints.extend(wps)

    @staticmethod
    def _xy_to_latlon(transformer, x: float, y: float) -> Tuple[float, float]:
        # CoordinateTransformer 用 ENU: x=East, y=North → local_to_geo(east, north)
        g = transformer.local_to_geo(x, y)
        return (g.latitude, g.longitude)
