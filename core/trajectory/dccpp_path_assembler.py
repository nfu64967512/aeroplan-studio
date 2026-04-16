"""
DCCPP 路徑組裝器
============================
將 IDPSolver 的排序結果（MDTSPResult）組裝為含 Dubins 曲線的
完整可飛行航點序列。

核心功能:
    IDPSolver output (ordered OperationSegments per UAV)
    → Entry Dubins (UAV start → first segment)
    → Operation (scan line)
    → Transfer Dubins (end of segment N → start of segment N+1)
    → ... repeat ...
    → AssembledPath (lat/lon/alt + heading + segment label)

座標轉換:
    DubinsTrajectoryGenerator 使用 NED (x=North, y=East)
    CoordinateTransformer 使用 ENU (x=East, y=North)
    本模組負責 ENU ↔ NED 的轉換。
"""

from __future__ import annotations

import math
import logging
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
from enum import Enum, auto

logger = logging.getLogger(__name__)


# ============================================================
# 資料結構
# ============================================================

class SegmentLabel(Enum):
    """航點所屬路徑段類型"""
    TAKEOFF = auto()     # 起飛爬升路徑
    ENTRY = auto()       # 進入路徑（Dubins 曲線）
    OPERATION = auto()   # 作業路徑（直線掃描）
    TRANSFER = auto()    # 轉移路徑（Dubins 曲線）
    LANDING = auto()     # 降落進場路徑


@dataclass
class AssembledWaypoint:
    """單個組裝航點"""
    lat: float
    lon: float
    alt: float
    heading_deg: float
    segment_type: SegmentLabel

    def to_tuple(self) -> Tuple[float, float]:
        return (self.lat, self.lon)

    def to_tuple_3d(self) -> Tuple[float, float, float]:
        return (self.lat, self.lon, self.alt)


@dataclass
class AssembledPath:
    """單架 UAV 的完整組裝路徑"""
    uav_id: int
    waypoints: List[AssembledWaypoint] = field(default_factory=list)
    takeoff_length_m: float = 0.0
    entry_length_m: float = 0.0
    operation_length_m: float = 0.0
    transfer_length_m: float = 0.0
    landing_length_m: float = 0.0

    @property
    def total_length_m(self) -> float:
        return (self.takeoff_length_m + self.entry_length_m +
                self.operation_length_m + self.transfer_length_m +
                self.landing_length_m)

    def to_latlon_list(self) -> List[Tuple[float, float]]:
        return [wp.to_tuple() for wp in self.waypoints]

    def to_latlon_3d_list(self) -> List[Tuple[float, float, float]]:
        return [wp.to_tuple_3d() for wp in self.waypoints]

    def get_segment_waypoints(self, label: SegmentLabel) -> List[AssembledWaypoint]:
        return [wp for wp in self.waypoints if wp.segment_type == label]


# ============================================================
# 路徑組裝器
# ============================================================

class DCCPPPathAssembler:
    """
    DCCPP 路徑組裝器。

    將 MDTSPResult 中每架 UAV 的有序 OperationSegment 列表
    組裝為含 Dubins 曲線的完整可飛行航點序列。

    使用方式:
        from core.trajectory.dubins_trajectory import DubinsTrajectoryGenerator
        from core.base.fixed_wing_constraints import FixedWingConstraints

        constraints = FixedWingConstraints(min_turn_radius=50.0, ...)
        dubins_gen = DubinsTrajectoryGenerator(constraints)
        assembler = DCCPPPathAssembler(dubins_gen=dubins_gen)

        assembled = assembler.assemble_all(uavs, mdtsp_result)
    """

    def __init__(
        self,
        dubins_gen=None,
        step_size: float = 10.0,
        default_altitude: float = 100.0,
        turn_radius: float = 50.0,
    ):
        """
        參數:
            dubins_gen       : DubinsTrajectoryGenerator 實例（可選，固定翼用）
            step_size        : Dubins 曲線離散步長 (m)
            default_altitude : 預設飛行高度 (m)
            turn_radius      : 預設最小轉彎半徑 (m)，用於無 dubins_gen 時的備用
        """
        self._dubins_gen = dubins_gen
        self._step_size = step_size
        self._default_alt = default_altitude
        self._turn_radius = turn_radius

    def assemble_all(
        self,
        uavs,
        mdtsp_result,
    ) -> Dict[int, AssembledPath]:
        """
        為所有 UAV 組裝完整路徑。

        參數:
            uavs          : List[UAVState]
            mdtsp_result  : MDTSPResult

        返回:
            {uav_id: AssembledPath}
        """
        uav_map = {u.uav_id: u for u in uavs}
        results = {}

        for uid, ops in mdtsp_result.uav_assignments.items():
            uav = uav_map.get(uid)
            if uav is None or not ops:
                results[uid] = AssembledPath(uav_id=uid)
                continue

            # 取得決策矩陣中該 UAV 的方向資訊
            directions = self._extract_directions(uid, ops, mdtsp_result)
            results[uid] = self.assemble_uav_path(uav, ops, directions)

        return results

    def assemble_uav_path(
        self,
        uav,
        operations,
        directions: Optional[List[int]] = None,
    ) -> AssembledPath:
        """
        組裝單架 UAV 的完整路徑。

        參數:
            uav        : UAVState
            operations : List[OperationSegment]（已排序）
            directions : 各段的進入方向 [+1=L→R, -1=R→L]

        返回:
            AssembledPath
        """
        if not operations:
            return AssembledPath(uav_id=uav.uav_id)

        path = AssembledPath(uav_id=uav.uav_id)
        is_fw = hasattr(uav, 'turn_radius') and uav.turn_radius > 0

        # 建立座標轉換器（以區域質心為原點）
        all_pts = []
        for op in operations:
            all_pts.append(op.left_point)
            all_pts.append(op.right_point)
        center_lat = sum(p[0] for p in all_pts) / len(all_pts)
        center_lon = sum(p[1] for p in all_pts) / len(all_pts)

        try:
            from core.geometry.coordinate import CoordinateTransformer
            transformer = CoordinateTransformer(center_lat, center_lon)
        except ImportError:
            transformer = None

        # 設定各段方向（預設交替方向）
        if directions is None or len(directions) < len(operations):
            directions = []
            d = 1
            for _ in operations:
                directions.append(d)
                d = -d

        # ── 當前狀態 ──
        cur_lat, cur_lon = uav.position
        cur_heading = uav.heading

        for i, op in enumerate(operations):
            d = directions[i] if i < len(directions) else 1

            # 確定進入/離開端點
            if d == 1:  # L→R
                entry_pt = op.left_point
                exit_pt = op.right_point
            else:       # R→L
                entry_pt = op.right_point
                exit_pt = op.left_point

            # 計算作業路徑航向
            op_heading = self._calc_bearing(entry_pt, exit_pt)

            # ── Entry/Transfer Dubins 曲線 ──
            if is_fw and transformer is not None:
                dubins_wps = self._generate_dubins_waypoints(
                    transformer,
                    cur_lat, cur_lon, cur_heading,
                    entry_pt[0], entry_pt[1], op_heading,
                    uav.turn_radius,
                )
                label = SegmentLabel.ENTRY if i == 0 else SegmentLabel.TRANSFER
                dubins_len = 0.0
                for wp_lat, wp_lon, wp_hdg in dubins_wps:
                    path.waypoints.append(AssembledWaypoint(
                        lat=wp_lat, lon=wp_lon,
                        alt=self._default_alt,
                        heading_deg=wp_hdg,
                        segment_type=label,
                    ))
                # 計算 Dubins 長度
                dubins_len = self._path_length_latlon(
                    [(cur_lat, cur_lon)]
                    + [(w[0], w[1]) for w in dubins_wps]
                )
                if i == 0:
                    path.entry_length_m += dubins_len
                else:
                    path.transfer_length_m += dubins_len
            else:
                # 多旋翼：直線連接
                label = SegmentLabel.ENTRY if i == 0 else SegmentLabel.TRANSFER
                path.waypoints.append(AssembledWaypoint(
                    lat=entry_pt[0], lon=entry_pt[1],
                    alt=self._default_alt,
                    heading_deg=op_heading,
                    segment_type=label,
                ))
                seg_len = self._haversine(cur_lat, cur_lon, entry_pt[0], entry_pt[1])
                if i == 0:
                    path.entry_length_m += seg_len
                else:
                    path.transfer_length_m += seg_len

            # ── 作業路徑（直線）──
            # 加入進入端點（若 Dubins 已覆蓋則跳過重複點）
            path.waypoints.append(AssembledWaypoint(
                lat=entry_pt[0], lon=entry_pt[1],
                alt=self._default_alt,
                heading_deg=op_heading,
                segment_type=SegmentLabel.OPERATION,
            ))

            # 若 waypoints 存在，加入中間點
            if op.waypoints:
                for wp in op.waypoints:
                    if isinstance(wp, (tuple, list)) and len(wp) >= 2:
                        path.waypoints.append(AssembledWaypoint(
                            lat=wp[0], lon=wp[1],
                            alt=self._default_alt,
                            heading_deg=op_heading,
                            segment_type=SegmentLabel.OPERATION,
                        ))

            # 加入離開端點
            path.waypoints.append(AssembledWaypoint(
                lat=exit_pt[0], lon=exit_pt[1],
                alt=self._default_alt,
                heading_deg=op_heading,
                segment_type=SegmentLabel.OPERATION,
            ))

            path.operation_length_m += op.length_m if op.length_m > 0 else \
                self._haversine(entry_pt[0], entry_pt[1], exit_pt[0], exit_pt[1])

            # 更新當前狀態
            cur_lat, cur_lon = exit_pt
            cur_heading = op_heading

        return path

    # ── Dubins 航點生成 ──────────────────────────────────

    def _generate_dubins_waypoints(
        self,
        transformer,
        start_lat: float, start_lon: float, start_heading: float,
        end_lat: float, end_lon: float, end_heading: float,
        turn_radius: float,
    ) -> List[Tuple[float, float, float]]:
        """
        使用 DubinsTrajectoryGenerator 產生 Dubins 曲線航點。

        座標轉換: lat/lon → ENU → NED → Dubins → NED → ENU → lat/lon

        返回:
            [(lat, lon, heading_deg), ...]
        """
        # lat/lon → ENU (x=East, y=North)
        start_enu = transformer.geo_to_local(start_lat, start_lon)
        end_enu = transformer.geo_to_local(end_lat, end_lon)

        # ENU → NED (swap x,y): NED(x=North, y=East)
        start_ned_x = start_enu[1]  # North
        start_ned_y = start_enu[0]  # East
        end_ned_x = end_enu[1]
        end_ned_y = end_enu[0]

        # 嘗試使用 DubinsTrajectoryGenerator
        if self._dubins_gen is not None:
            try:
                from core.trajectory.dubins_trajectory import Pose3D
                start_pose = Pose3D(
                    x=start_ned_x, y=start_ned_y,
                    z=self._default_alt,
                    heading_deg=start_heading,
                )
                end_pose = Pose3D(
                    x=end_ned_x, y=end_ned_y,
                    z=self._default_alt,
                    heading_deg=end_heading,
                )
                dubins_path = self._dubins_gen.calculate_path(start_pose, end_pose)
                if dubins_path.is_feasible:
                    poses = self._dubins_gen.generate_waypoints(
                        dubins_path, self._step_size
                    )
                    result = []
                    for p in poses:
                        # NED → ENU: swap x,y
                        geo = transformer.local_to_geo(p.y, p.x)  # (east=p.y, north=p.x)
                        result.append((geo.latitude, geo.longitude, p.heading_deg))
                    return result
            except Exception as e:
                logger.warning(f"Dubins 計算失敗，改用弧線近似: {e}")

        # ── 備用：解析弧線近似 ──
        return self._approximate_dubins(
            transformer,
            start_ned_x, start_ned_y, start_heading,
            end_ned_x, end_ned_y, end_heading,
            turn_radius,
        )

    def _approximate_dubins(
        self,
        transformer,
        sx, sy, sh,
        ex, ey, eh,
        r,
    ) -> List[Tuple[float, float, float]]:
        """
        Dubins 解析近似（不依賴 DubinsTrajectoryGenerator）。
        生成從 (sx,sy,sh) 到 (ex,ey,eh) 的 圓弧-直線-圓弧 路徑，
        圓弧半徑 = r（轉彎半徑）。

        座標系: NED (x=North, y=East)，航向 0°=North，順時針增加。
        """
        result = []
        dist = math.hypot(ex - sx, ey - sy)
        if dist < 1.0:
            return result

        # 確保 r > 0
        if r <= 0:
            r = self._turn_radius if self._turn_radius > 0 else 50.0

        step = self._step_size

        # 航向轉弧度 (NED: 0=N, 90=E)
        sh_rad = math.radians(sh)
        eh_rad = math.radians(eh)

        # 起點/終點的方向向量 (NED: dx=cos(h), dy=sin(h))
        s_dx, s_dy = math.cos(sh_rad), math.sin(sh_rad)
        e_dx, e_dy = math.cos(eh_rad), math.sin(eh_rad)

        # ── 選擇最佳轉彎方向（LSL/RSR 簡化）──
        # 起點左/右轉圓心
        # 右轉圓心: 航向右方 90°
        s_rc = (sx + r * math.cos(sh_rad + math.pi / 2),
                sy + r * math.sin(sh_rad + math.pi / 2))
        s_lc = (sx + r * math.cos(sh_rad - math.pi / 2),
                sy + r * math.sin(sh_rad - math.pi / 2))
        # 終點左/右轉圓心
        e_rc = (ex + r * math.cos(eh_rad + math.pi / 2),
                ey + r * math.sin(eh_rad + math.pi / 2))
        e_lc = (ex + r * math.cos(eh_rad - math.pi / 2),
                ey + r * math.sin(eh_rad - math.pi / 2))

        # 選 RSR 或 LSL 中較短者
        d_rr = math.hypot(e_rc[0] - s_rc[0], e_rc[1] - s_rc[1])
        d_ll = math.hypot(e_lc[0] - s_lc[0], e_lc[1] - s_lc[1])

        if d_rr <= d_ll:
            # RSR: 右轉 → 直線 → 右轉
            c1, c2 = s_rc, e_rc
            turn_dir = 1  # 順時針
        else:
            # LSL: 左轉 → 直線 → 左轉
            c1, c2 = s_lc, e_lc
            turn_dir = -1  # 逆時針

        # ── 計算切線點 ──
        cc_dist = math.hypot(c2[0] - c1[0], c2[1] - c1[1])
        if cc_dist < 1e-6:
            # 兩圓心重合 → 單純圓弧
            cc_angle = sh_rad
        else:
            cc_angle = math.atan2(c2[1] - c1[1], c2[0] - c1[0])

        # 同向切線 (RSR/LSL): 切線方向 = 圓心連線方向
        tangent_angle = cc_angle

        # 圓弧 1 切線離開點
        t1x = c1[0] + r * math.cos(tangent_angle + turn_dir * math.pi / 2)
        t1y = c1[1] + r * math.sin(tangent_angle + turn_dir * math.pi / 2)
        # 圓弧 2 切線進入點
        t2x = c2[0] + r * math.cos(tangent_angle + turn_dir * math.pi / 2)
        t2y = c2[1] + r * math.sin(tangent_angle + turn_dir * math.pi / 2)

        # ── 生成圓弧 1: 起點 → 切線點 t1 ──
        a1_start = math.atan2(sy - c1[1], sx - c1[0])
        a1_end = math.atan2(t1y - c1[1], t1x - c1[0])
        arc1_sweep = self._arc_sweep(a1_start, a1_end, turn_dir)
        arc1_len = abs(arc1_sweep) * r
        n_arc1 = max(2, int(arc1_len / step))
        for i in range(1, n_arc1 + 1):
            t = i / n_arc1
            a = a1_start + t * arc1_sweep
            px = c1[0] + r * math.cos(a)
            py = c1[1] + r * math.sin(a)
            hdg = math.degrees(a + turn_dir * math.pi / 2) % 360
            geo = transformer.local_to_geo(py, px)
            result.append((geo.latitude, geo.longitude, hdg))

        # ── 生成直線段: t1 → t2 ──
        straight_dist = math.hypot(t2x - t1x, t2y - t1y)
        straight_hdg = math.degrees(tangent_angle) % 360
        if straight_dist > step:
            n_str = max(2, int(straight_dist / step))
            for i in range(1, n_str + 1):
                t = i / n_str
                px = t1x + t * (t2x - t1x)
                py = t1y + t * (t2y - t1y)
                geo = transformer.local_to_geo(py, px)
                result.append((geo.latitude, geo.longitude, straight_hdg))

        # ── 生成圓弧 2: t2 → 終點 ──
        a2_start = math.atan2(t2y - c2[1], t2x - c2[0])
        a2_end = math.atan2(ey - c2[1], ex - c2[0])
        arc2_sweep = self._arc_sweep(a2_start, a2_end, turn_dir)
        arc2_len = abs(arc2_sweep) * r
        n_arc2 = max(2, int(arc2_len / step))
        for i in range(1, n_arc2 + 1):
            t = i / n_arc2
            a = a2_start + t * arc2_sweep
            px = c2[0] + r * math.cos(a)
            py = c2[1] + r * math.sin(a)
            hdg = math.degrees(a + turn_dir * math.pi / 2) % 360
            geo = transformer.local_to_geo(py, px)
            result.append((geo.latitude, geo.longitude, hdg))

        return result

    @staticmethod
    def _arc_sweep(a_start: float, a_end: float, direction: int) -> float:
        """
        計算圓弧掃過角度。
        direction: +1=順時針(CW), -1=逆時針(CCW)
        返回帶符號的掃過角度。
        """
        diff = a_end - a_start
        if direction == 1:  # CW → 角度遞減
            while diff > 0:
                diff -= 2 * math.pi
            if diff == 0:
                diff = -2 * math.pi
        else:  # CCW → 角度遞增
            while diff < 0:
                diff += 2 * math.pi
            if diff == 0:
                diff = 2 * math.pi
        return diff

    # ── 方向提取 ──────────────────────────────────────────

    def _extract_directions(self, uav_id, ops, mdtsp_result) -> List[int]:
        """從 MDTSPResult 的決策矩陣中提取方向序列"""
        E = None
        if mdtsp_result.decision_matrix and uav_id in mdtsp_result.decision_matrix:
            E = mdtsp_result.decision_matrix[uav_id]

        if E is None:
            # 預設交替方向
            return [(-1) ** i for i in range(len(ops))]

        directions = []
        prev_idx = -1
        for op in ops:
            row = prev_idx + 1
            col = op.index
            if 0 <= row < len(E) and 0 <= col < len(E[0]):
                d = E[row][col]
                directions.append(d if d != 0 else 1)
            else:
                directions.append(1)
            prev_idx = op.index

        return directions

    # ── 工具方法 ──────────────────────────────────────────

    @staticmethod
    def _calc_bearing(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """計算兩點之間的航向角（度，0=北，順時針為正）"""
        dlat = p2[0] - p1[0]
        dlon = p2[1] - p1[1]
        # 考慮緯度的經度縮放
        dlon_m = dlon * math.cos(math.radians((p1[0] + p2[0]) / 2))
        bearing = math.degrees(math.atan2(dlon_m, dlat)) % 360
        return bearing

    @staticmethod
    def _haversine(lat1, lon1, lat2, lon2) -> float:
        """Haversine 距離（公尺）"""
        R = 6_371_000.0
        rlat1, rlat2 = math.radians(lat1), math.radians(lat2)
        dlat = rlat2 - rlat1
        dlon = math.radians(lon2 - lon1)
        a = (math.sin(dlat / 2) ** 2
             + math.cos(rlat1) * math.cos(rlat2) * math.sin(dlon / 2) ** 2)
        return R * 2.0 * math.asin(min(1.0, math.sqrt(a)))

    @staticmethod
    def _path_length_latlon(points: List[Tuple[float, float]]) -> float:
        """計算 lat/lon 路徑總長度"""
        total = 0.0
        for i in range(1, len(points)):
            total += DCCPPPathAssembler._haversine(
                points[i - 1][0], points[i - 1][1],
                points[i][0], points[i][1],
            )
        return total

    # ================================================================
    # 起飛 / 降落路徑生成（論文 Section 2.3.2 Entry Path 延伸）
    # ================================================================

    @staticmethod
    def _offset_latlon(lat: float, lon: float, dist_m: float, bearing_deg: float):
        """從 (lat,lon) 沿 bearing 偏移 dist_m，回傳 (lat2, lon2)"""
        R = 6371000.0
        br = math.radians(bearing_deg)
        lat_r = math.radians(lat)
        d_lat = dist_m * math.cos(br) / R
        d_lon = dist_m * math.sin(br) / (R * math.cos(lat_r))
        return lat + math.degrees(d_lat), lon + math.degrees(d_lon)

    def generate_takeoff_waypoints(
        self,
        home_lat: float, home_lon: float,
        takeoff_bearing_deg: float,
        runway_m: float = 50.0,
        climb_alt_m: float = 30.0,
        mission_alt_m: float = None,
        turn_radius: float = None,
    ) -> List[AssembledWaypoint]:
        """
        生成起飛爬升路徑航點。

        路徑結構：
          [0] Home 地面
          [1] 跑道末端（NAV_TAKEOFF 高度）
          [2..N] 余弦漸進爬升至任務高度

        返回含 SegmentLabel.TAKEOFF 的航點序列。
        """
        alt = mission_alt_m if mission_alt_m else self._default_alt
        tr = turn_radius if turn_radius and turn_radius > 0 else self._turn_radius
        wps = []

        # [0] Home (地面)
        wps.append(AssembledWaypoint(
            lat=home_lat, lon=home_lon, alt=0.0,
            heading_deg=takeoff_bearing_deg,
            segment_type=SegmentLabel.TAKEOFF,
        ))

        # [1] 跑道末端
        end_lat, end_lon = self._offset_latlon(
            home_lat, home_lon, runway_m, takeoff_bearing_deg
        )
        wps.append(AssembledWaypoint(
            lat=end_lat, lon=end_lon, alt=climb_alt_m,
            heading_deg=takeoff_bearing_deg,
            segment_type=SegmentLabel.TAKEOFF,
        ))

        # [2..N] 余弦爬升至 mission_alt
        CLIMB_STEPS = 5
        climb_span = 4.0 * tr
        for i in range(1, CLIMB_STEPS + 1):
            t = i / CLIMB_STEPS
            dist = runway_m + t * climb_span
            alt_frac = 1.0 - math.cos(math.pi * t / 2.0)
            pt_alt = climb_alt_m + (alt - climb_alt_m) * alt_frac
            pt_lat, pt_lon = self._offset_latlon(
                home_lat, home_lon, dist, takeoff_bearing_deg
            )
            wps.append(AssembledWaypoint(
                lat=pt_lat, lon=pt_lon, alt=pt_alt,
                heading_deg=takeoff_bearing_deg,
                segment_type=SegmentLabel.TAKEOFF,
            ))

        return wps

    def generate_landing_waypoints(
        self,
        landing_lat: float, landing_lon: float,
        landing_bearing_deg: float,
        pattern_alt_m: float = 80.0,
        downwind_offset_m: float = 200.0,
        pattern_leg_length_m: float = 300.0,
        final_approach_dist_m: float = 400.0,
        landing_rollout_m: float = 0.0,
    ) -> List[AssembledWaypoint]:
        """
        生成五邊進場降落路徑航點（右手飛行模式）。

        路徑結構：
          [0] 下風邊入口
          [1] 下風轉基邊
          [2..N-1] 基邊 → 最終進場下降
          [N] 觸地點（NAV_LAND）

        landing_rollout_m: 觸地後滑行煞車距離，觸地點會提前此距離。

        返回含 SegmentLabel.LANDING 的航點序列。
        """
        wps = []
        # 五邊方向計算（右手模式）
        approach_hdg = landing_bearing_deg  # 最終進場方向
        # 下風邊方向 = 進場反向
        downwind_hdg = (approach_hdg + 180.0) % 360.0
        # 右偏方向（右手模式）
        right_hdg = (approach_hdg + 90.0) % 360.0

        # 觸地點：若有滑行距離則提前
        rollout = landing_rollout_m or 0.0
        if rollout > 0:
            td_lat, td_lon = self._offset_latlon(
                landing_lat, landing_lon, rollout,
                (approach_hdg + 180.0) % 360.0  # 進場反方向 = 提前
            )
        else:
            td_lat, td_lon = landing_lat, landing_lon

        # 最終進場起點 (FAF)
        faf_lat, faf_lon = self._offset_latlon(
            td_lat, td_lon, final_approach_dist_m,
            (approach_hdg + 180.0) % 360.0  # 從觸地點往回推
        )

        # 基邊轉彎點（FAF 右偏）
        base_lat, base_lon = self._offset_latlon(
            faf_lat, faf_lon, downwind_offset_m, right_hdg
        )

        # 下風邊轉基邊點
        dw_end_lat, dw_end_lon = self._offset_latlon(
            base_lat, base_lon, pattern_leg_length_m, downwind_hdg
        )

        # 下風邊入口（再往下風方向延伸）
        dw_start_lat, dw_start_lon = self._offset_latlon(
            dw_end_lat, dw_end_lon, pattern_leg_length_m, downwind_hdg
        )

        # [0] 下風邊入口
        wps.append(AssembledWaypoint(
            lat=dw_start_lat, lon=dw_start_lon, alt=pattern_alt_m,
            heading_deg=downwind_hdg, segment_type=SegmentLabel.LANDING,
        ))

        # [1] 下風轉基邊
        wps.append(AssembledWaypoint(
            lat=dw_end_lat, lon=dw_end_lon, alt=pattern_alt_m,
            heading_deg=downwind_hdg, segment_type=SegmentLabel.LANDING,
        ))

        # [2] 基邊轉彎
        wps.append(AssembledWaypoint(
            lat=base_lat, lon=base_lon, alt=pattern_alt_m * 0.7,
            heading_deg=(right_hdg + 180.0) % 360.0,
            segment_type=SegmentLabel.LANDING,
        ))

        # [3] FAF（最終進場起點）
        wps.append(AssembledWaypoint(
            lat=faf_lat, lon=faf_lon, alt=pattern_alt_m * 0.4,
            heading_deg=approach_hdg, segment_type=SegmentLabel.LANDING,
        ))

        # [4] 觸地點 (NAV_LAND)
        wps.append(AssembledWaypoint(
            lat=td_lat, lon=td_lon, alt=0.0,
            heading_deg=approach_hdg, segment_type=SegmentLabel.LANDING,
        ))

        return wps

    def prepend_takeoff(
        self,
        path: AssembledPath,
        home_lat: float, home_lon: float,
        takeoff_bearing_deg: float,
        **kwargs,
    ):
        """在已組裝路徑前端插入起飛路徑"""
        takeoff_wps = self.generate_takeoff_waypoints(
            home_lat, home_lon, takeoff_bearing_deg,
            mission_alt_m=self._default_alt,
            turn_radius=self._turn_radius,
            **kwargs,
        )
        # 計算起飛路徑長度
        pts = [(w.lat, w.lon) for w in takeoff_wps]
        path.takeoff_length_m = self._path_length_latlon(pts)
        # 插入到航點序列前端
        path.waypoints = takeoff_wps + path.waypoints

    def append_landing(
        self,
        path: AssembledPath,
        landing_lat: float, landing_lon: float,
        landing_bearing_deg: float,
        **kwargs,
    ):
        """在已組裝路徑末端附加降落路徑"""
        landing_wps = self.generate_landing_waypoints(
            landing_lat, landing_lon, landing_bearing_deg, **kwargs,
        )
        # 計算降落路徑長度
        pts = [(w.lat, w.lon) for w in landing_wps]
        path.landing_length_m = self._path_length_latlon(pts)
        # 附加到航點序列末端
        path.waypoints = path.waypoints + landing_wps
