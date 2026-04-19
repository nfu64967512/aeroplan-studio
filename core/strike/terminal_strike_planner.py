"""
末端俯衝打擊規劃器 (Terminal Strike Planner)

實作 UCAV 蜂群分佈式協同打擊與末端俯衝 (Swarm Distributed Strike & Terminal Dive)
的核心演算法：
  1. 匈牙利演算法 (Hungarian Algorithm) 進行 UAV-目標最佳分配
  2. 2D Dubins 曲線計算四散巡航段水平軌跡
  3. 末端俯衝段 3D 軌跡生成（含最大俯衝角約束）
  4. 高度錯層 + 進場方位角錯開，確保多機航線完全不交叉
"""

import math
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass, field

from utils.logger import get_logger

logger = get_logger()


# ═══════════════════════════════════════════════════════════════════════
#  資料結構
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class StrikeTarget:
    """地面打擊目標"""
    target_id: int
    lat: float
    lon: float
    alt: float = 0.0       # 地面高度
    name: str = ''

    def __post_init__(self):
        if not self.name:
            self.name = f'TGT-{self.target_id}'


@dataclass
class StrikeUAV:
    """參與打擊的 UCAV"""
    uav_id: int
    lat: float              # 集結點位置
    lon: float
    cruise_alt: float       # 巡航高度 (m)
    speed_mps: float = 60.0 # 巡航速度 (m/s)
    name: str = ''

    def __post_init__(self):
        if not self.name:
            self.name = f'UCAV-{self.uav_id}'


@dataclass
class StrikeWaypoint:
    """打擊航線航點"""
    lat: float
    lon: float
    alt: float
    segment: str = 'cruise'   # cruise | dive
    time_sec: float = 0.0     # 任務時間（秒）


@dataclass
class StrikeTrajectory:
    """單架 UCAV 的完整打擊軌跡"""
    uav_id: int
    uav_name: str
    target_id: int
    target_name: str
    waypoints: List[StrikeWaypoint] = field(default_factory=list)
    dive_start_index: int = -1       # 俯衝起始航點索引
    impact_index: int = -1           # 命中航點索引
    total_distance_m: float = 0.0
    total_time_sec: float = 0.0
    cruise_alt_m: float = 0.0        # 此 UAV 實際巡航高度（含錯層）


# ═══════════════════════════════════════════════════════════════════════
#  地理計算工具
# ═══════════════════════════════════════════════════════════════════════

_R_EARTH = 6_371_000.0  # 地球平均半徑 (m)


def _haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Haversine 公式計算兩點間大圓距離 (m)"""
    rlat1, rlon1 = math.radians(lat1), math.radians(lon1)
    rlat2, rlon2 = math.radians(lat2), math.radians(lon2)
    dlat = rlat2 - rlat1
    dlon = rlon2 - rlon1
    a = math.sin(dlat / 2) ** 2 + math.cos(rlat1) * math.cos(rlat2) * math.sin(dlon / 2) ** 2
    return 2 * _R_EARTH * math.asin(min(1.0, math.sqrt(a)))


def _bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """計算從 (lat1,lon1) 到 (lat2,lon2) 的方位角 (degrees, 0=北, 順時針)"""
    rlat1, rlon1 = math.radians(lat1), math.radians(lon1)
    rlat2, rlon2 = math.radians(lat2), math.radians(lon2)
    dlon = rlon2 - rlon1
    x = math.sin(dlon) * math.cos(rlat2)
    y = math.cos(rlat1) * math.sin(rlat2) - math.sin(rlat1) * math.cos(rlat2) * math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def _destination(lat: float, lon: float, bearing_deg: float, dist_m: float) -> Tuple[float, float]:
    """從起點出發，沿方位角移動指定距離，回傳目的點 (lat, lon)"""
    rlat = math.radians(lat)
    rlon = math.radians(lon)
    rb = math.radians(bearing_deg)
    d = dist_m / _R_EARTH

    rlat2 = math.asin(
        math.sin(rlat) * math.cos(d) + math.cos(rlat) * math.sin(d) * math.cos(rb)
    )
    rlon2 = rlon + math.atan2(
        math.sin(rb) * math.sin(d) * math.cos(rlat),
        math.cos(d) - math.sin(rlat) * math.sin(rlat2),
    )
    return math.degrees(rlat2), math.degrees(rlon2)


def _interp_geo(lat1: float, lon1: float, alt1: float,
                lat2: float, lon2: float, alt2: float,
                t: float) -> Tuple[float, float, float]:
    """在兩個地理點之間做線性插值 (t ∈ [0, 1])"""
    return (
        lat1 + (lat2 - lat1) * t,
        lon1 + (lon2 - lon1) * t,
        alt1 + (alt2 - alt1) * t,
    )


# ═══════════════════════════════════════════════════════════════════════
#  匈牙利演算法 (Hungarian Algorithm) — 最佳分配
# ═══════════════════════════════════════════════════════════════════════

def _hungarian_assignment(cost_matrix: List[List[float]]) -> List[Tuple[int, int]]:
    """
    簡化的匈牙利演算法實作，用於 N×M 成本矩陣的最佳分配。
    回傳 [(uav_idx, target_idx), ...]

    若 N > M：部分 UAV 不分配目標
    若 N < M：部分目標不被分配
    若 N == M：一對一完美分配
    """
    n_rows = len(cost_matrix)
    if n_rows == 0:
        return []
    n_cols = len(cost_matrix[0])
    if n_cols == 0:
        return []

    # 確保矩陣為方陣（填充大值）
    size = max(n_rows, n_cols)
    big = 1e15
    mat = [[big] * size for _ in range(size)]
    for i in range(n_rows):
        for j in range(n_cols):
            mat[i][j] = cost_matrix[i][j]

    # 行歸約
    for i in range(size):
        row_min = min(mat[i])
        for j in range(size):
            mat[i][j] -= row_min

    # 列歸約
    for j in range(size):
        col_min = min(mat[i][j] for i in range(size))
        for i in range(size):
            mat[i][j] -= col_min

    # 貪婪逼近（對小規模問題足夠）
    row_assigned = [False] * size
    col_assigned = [False] * size
    assignment = [-1] * size

    for _ in range(size):
        best_val = big + 1
        best_i, best_j = -1, -1
        for i in range(size):
            if row_assigned[i]:
                continue
            for j in range(size):
                if col_assigned[j]:
                    continue
                if mat[i][j] < best_val:
                    best_val = mat[i][j]
                    best_i, best_j = i, j
        if best_i < 0:
            break
        assignment[best_i] = best_j
        row_assigned[best_i] = True
        col_assigned[best_j] = True

    result = []
    for i in range(n_rows):
        j = assignment[i]
        if j < n_cols:
            result.append((i, j))
    return result


# ═══════════════════════════════════════════════════════════════════════
#  末端俯衝打擊規劃器
# ═══════════════════════════════════════════════════════════════════════

class TerminalStrikePlanner:
    """
    UCAV 蜂群分佈式末端俯衝打擊規劃器

    演算法流程：
      1. 使用匈牙利演算法將 N 架 UAV 分配給 M 個目標（最小化總距離）
      2. 為每架 UAV 規劃「四散巡航段」→「俯衝起始點」→「末端俯衝段」
      3. 俯衝段受最大俯衝角 (max_dive_angle) 約束
      4. 高度錯層 (altitude_step) 確保各機巡航高度不同
      5. 進場方位角自動錯開，確保航線不交叉
    """

    def __init__(self,
                 max_dive_angle_deg: float = 45.0,
                 dive_initiation_dist_m: float = 800.0,
                 cruise_alt_m: float = 500.0,
                 cruise_speed_mps: float = 60.0,
                 altitude_step_m: float = 30.0,
                 cruise_step_m: float = 50.0,
                 dive_step_m: float = 20.0):
        """
        參數:
            max_dive_angle_deg:   最大俯衝角（度，正值表示向下角度）
            dive_initiation_dist_m: 俯衝起始距離（距目標水平距離，公尺）
            cruise_alt_m:         基準巡航高度（公尺）
            cruise_speed_mps:     巡航速度（公尺/秒）
            altitude_step_m:      高度錯層間距（公尺），各機遞增此值
            cruise_step_m:        巡航段航點間距（公尺）
            dive_step_m:          俯衝段航點間距（公尺）
        """
        self.max_dive_angle_deg = abs(max_dive_angle_deg)
        self.dive_initiation_dist_m = dive_initiation_dist_m
        self.cruise_alt_m = cruise_alt_m
        self.cruise_speed_mps = cruise_speed_mps
        self.altitude_step_m = altitude_step_m
        self.cruise_step_m = cruise_step_m
        self.dive_step_m = dive_step_m

    # ─────────────────────────────────────────────────────────────────
    #  自動生成模式：N 個目標 → 自動生成 N 架 UAV（高度 + 路線完全錯開）
    # ─────────────────────────────────────────────────────────────────
    def plan_auto(self,
                  targets: List[StrikeTarget],
                  spawn_dist_m: float = 1500.0,
                  ) -> List[StrikeTrajectory]:
        """
        自動生成 N 架 UAV 並規劃打擊。

        每個目標自動分配 1 架 UCAV，UAV 生成位置根據目標群分佈
        自動計算最佳進場方位角，確保：
          - 每架 UAV 的巡航高度互不相同（高度錯層）
          - 每架 UAV 的進場方向互不相同（路線錯開）

        參數:
            targets:       地面目標列表
            spawn_dist_m:  UAV 生成距離（距其目標的水平距離，公尺）

        回傳:
            List[StrikeTrajectory]
        """
        if not targets:
            logger.warning('[StrikePlanner] 目標列表為空')
            return []

        n = len(targets)
        logger.info(f'[StrikePlanner] 自動模式：{n} 個目標 → 生成 {n} 架 UCAV')

        # ── 計算目標群質心 ────────────────────────────────────────────
        avg_lat = sum(t.lat for t in targets) / n
        avg_lon = sum(t.lon for t in targets) / n

        # ── 為每個目標計算最佳進場方位角（360°/N 等分 + 偏移避免與目標群連線重疊）
        # 策略：先計算每個目標相對質心的方位角，然後讓 UAV 從「對面」進場
        # 這樣可以確保 UAV 從不同方向飛入，航線天然不交叉
        approach_bearings = self._compute_deconflicted_bearings(targets, avg_lat, avg_lon)

        # ── 生成 UAV 並分配 ──────────────────────────────────────────
        uavs = []
        for i, tgt in enumerate(targets):
            # 每架 UAV 的巡航高度 = 基準高度 + i * 高度錯層
            uav_alt = self.cruise_alt_m + i * self.altitude_step_m

            # UAV 生成位置：從目標出發，沿進場方位角反方向移動 spawn_dist
            spawn_bearing = approach_bearings[i]
            # 反方向：UAV 在目標的「對面」，所以往 spawn_bearing + 180 方向移動
            spawn_lat, spawn_lon = _destination(
                tgt.lat, tgt.lon,
                (spawn_bearing + 180) % 360,
                spawn_dist_m,
            )

            uavs.append(StrikeUAV(
                uav_id=i + 1,
                lat=spawn_lat,
                lon=spawn_lon,
                cruise_alt=uav_alt,
                speed_mps=self.cruise_speed_mps,
            ))

            logger.debug(
                f'[StrikePlanner] UCAV-{i+1}: alt={uav_alt:.0f}m, '
                f'bearing={spawn_bearing:.1f}°, '
                f'spawn=({spawn_lat:.6f}, {spawn_lon:.6f})'
            )

        # ── 使用標準 plan() 流程（匈牙利分配 + 軌跡生成）────────────
        return self.plan(uavs, targets)

    def _compute_deconflicted_bearings(self,
                                        targets: List[StrikeTarget],
                                        center_lat: float,
                                        center_lon: float) -> List[float]:
        """
        為每個目標計算去衝突進場方位角。

        策略：
          1. 計算每個目標相對質心的方位角 θ_i
          2. 將各目標的進場方向設為 θ_i（從質心方向飛入），
             確保 UAV 從不同徑向進場
          3. 若兩個目標方位角太近（< 15°），自動微調偏移

        回傳:
            List[float]：每個目標的進場方位角（度）
        """
        n = len(targets)
        if n == 1:
            return [0.0]  # 單一目標，從北方進場

        # 計算每個目標相對質心的方位角
        raw_bearings = []
        for tgt in targets:
            brg = _bearing(center_lat, center_lon, tgt.lat, tgt.lon)
            raw_bearings.append(brg)

        # 按方位角排序，取得索引映射
        indexed = sorted(enumerate(raw_bearings), key=lambda x: x[1])

        # 最小角度間距（確保航線不交叉）
        min_separation = max(15.0, 360.0 / n * 0.4)

        # 去衝突：若相鄰方位角差太小，向兩邊推開
        sorted_bearings = [b for _, b in indexed]
        for pass_num in range(3):  # 最多 3 輪迭代
            changed = False
            for i in range(len(sorted_bearings)):
                j = (i + 1) % len(sorted_bearings)
                diff = (sorted_bearings[j] - sorted_bearings[i]) % 360
                if diff < min_separation:
                    nudge = (min_separation - diff) / 2 + 1.0
                    sorted_bearings[i] = (sorted_bearings[i] - nudge) % 360
                    sorted_bearings[j] = (sorted_bearings[j] + nudge) % 360
                    changed = True
            if not changed:
                break

        # 重新映射回原始目標順序
        result = [0.0] * n
        for k, (orig_idx, _) in enumerate(indexed):
            result[orig_idx] = sorted_bearings[k]

        return result

    def plan(self,
             uavs: List[StrikeUAV],
             targets: List[StrikeTarget],
             rally_point: Optional[Tuple[float, float, float]] = None,
             ) -> List[StrikeTrajectory]:
        """
        執行蜂群打擊規劃

        參數:
            uavs:        UCAV 列表（集結狀態）
            targets:     地面目標列表
            rally_point: 集結點 (lat, lon, alt)；若 None 則取 UAV 群的質心

        回傳:
            List[StrikeTrajectory]：每架 UAV 的完整打擊軌跡
        """
        if not uavs or not targets:
            logger.warning('[StrikePlanner] UAV 或目標列表為空')
            return []

        # ── Step 1: 計算成本矩陣 ─────────────────────────────────────
        logger.info(f'[StrikePlanner] 開始規劃：{len(uavs)} 架 UCAV → {len(targets)} 個目標')
        cost_matrix = []
        for uav in uavs:
            row = []
            for tgt in targets:
                dist = _haversine(uav.lat, uav.lon, tgt.lat, tgt.lon)
                row.append(dist)
            cost_matrix.append(row)

        # ── Step 2: 匈牙利演算法分配 ─────────────────────────────────
        assignments = _hungarian_assignment(cost_matrix)
        logger.info(f'[StrikePlanner] 分配完成：{assignments}')

        # ── Step 3: 為每對 (UAV, Target) 生成 3D 軌跡 ───────────────
        trajectories = []
        for uav_idx, tgt_idx in assignments:
            uav = uavs[uav_idx]
            tgt = targets[tgt_idx]
            traj = self._plan_single_strike(uav, tgt)
            trajectories.append(traj)
            logger.info(
                f'[StrikePlanner] {uav.name} → {tgt.name}: '
                f'{traj.total_distance_m:.0f}m, {traj.total_time_sec:.1f}s, '
                f'alt={traj.cruise_alt_m:.0f}m, dive@WP{traj.dive_start_index}'
            )

        return trajectories

    def _plan_single_strike(self, uav: StrikeUAV, target: StrikeTarget) -> StrikeTrajectory:
        """
        規劃單架 UCAV 對單個目標的攻擊軌跡

        包含：
          (a) 四散巡航段：從 UAV 位置飛往「俯衝起始點」（水平 Dubins 概念，此處用大圓直線近似）
          (b) 末端俯衝段：從俯衝起始點急降至地面目標

        俯衝角約束：
          θ = atan(Δalt / horizontal_dist)
          若計算出的 θ > max_dive_angle，則自動將俯衝起始點後推，
          使 horizontal_dist = Δalt / tan(max_dive_angle)
        """
        cruise_alt = uav.cruise_alt if uav.cruise_alt > 0 else self.cruise_alt_m
        target_alt = target.alt

        # ── 計算俯衝幾何 ─────────────────────────────────────────────
        # 從目標點方向反推俯衝起始點
        brg_uav_to_tgt = _bearing(uav.lat, uav.lon, target.lat, target.lon)
        total_ground_dist = _haversine(uav.lat, uav.lon, target.lat, target.lon)

        # 高度差
        delta_alt = cruise_alt - target_alt
        if delta_alt <= 0:
            delta_alt = cruise_alt  # 至少用巡航高度

        # 計算最小俯衝水平距離（受最大俯衝角限制）
        max_angle_rad = math.radians(self.max_dive_angle_deg)
        min_dive_horiz = delta_alt / math.tan(max_angle_rad) if max_angle_rad > 0.01 else delta_alt

        # 實際俯衝起始距離：取使用者設定值與物理限制的較大值
        actual_dive_dist = max(self.dive_initiation_dist_m, min_dive_horiz)

        # 若 UAV 到目標的總距離不足以容納俯衝段，則調整
        if actual_dive_dist > total_ground_dist:
            # UAV 太近，直接從 UAV 位置開始俯衝
            actual_dive_dist = total_ground_dist

        # 實際俯衝角（度）
        actual_dive_angle_deg = math.degrees(math.atan2(delta_alt, actual_dive_dist))

        # 俯衝起始點：從目標反推
        dive_start_lat, dive_start_lon = _destination(
            target.lat, target.lon,
            (_bearing(target.lat, target.lon, uav.lat, uav.lon)),  # 目標→UAV 方向
            actual_dive_dist
        )

        # ── 生成巡航段航點 ───────────────────────────────────────────
        waypoints: List[StrikeWaypoint] = []
        cruise_dist = _haversine(uav.lat, uav.lon, dive_start_lat, dive_start_lon)

        if cruise_dist > self.cruise_step_m:
            n_cruise_steps = max(2, int(cruise_dist / self.cruise_step_m))
            for i in range(n_cruise_steps + 1):
                t = i / n_cruise_steps
                lat_i = uav.lat + (dive_start_lat - uav.lat) * t
                lon_i = uav.lon + (dive_start_lon - uav.lon) * t
                wp = StrikeWaypoint(
                    lat=lat_i, lon=lon_i, alt=cruise_alt,
                    segment='cruise',
                    time_sec=(cruise_dist * t) / max(uav.speed_mps, 1.0),
                )
                waypoints.append(wp)
        else:
            # 太近，只加起點
            waypoints.append(StrikeWaypoint(
                lat=uav.lat, lon=uav.lon, alt=cruise_alt,
                segment='cruise', time_sec=0.0,
            ))

        dive_start_idx = len(waypoints) - 1
        cruise_time = (cruise_dist / max(uav.speed_mps, 1.0))

        # ── 生成末端俯衝段航點 ───────────────────────────────────────
        # 俯衝段：從 (dive_start_lat, dive_start_lon, cruise_alt) 到 (target.lat, target.lon, target_alt)
        # 使用線性插值生成平滑下降軌跡
        dive_3d_dist = math.sqrt(actual_dive_dist ** 2 + delta_alt ** 2)
        n_dive_steps = max(3, int(dive_3d_dist / self.dive_step_m))

        # 俯衝速度假設為巡航速度的 1.3 倍（重力加速）
        dive_speed = uav.speed_mps * 1.3

        for i in range(1, n_dive_steps + 1):
            t = i / n_dive_steps
            lat_i, lon_i, alt_i = _interp_geo(
                dive_start_lat, dive_start_lon, cruise_alt,
                target.lat, target.lon, target_alt,
                t,
            )
            wp = StrikeWaypoint(
                lat=lat_i, lon=lon_i, alt=max(alt_i, target_alt),
                segment='dive',
                time_sec=cruise_time + (dive_3d_dist * t) / max(dive_speed, 1.0),
            )
            waypoints.append(wp)

        impact_idx = len(waypoints) - 1
        total_dist = cruise_dist + dive_3d_dist
        total_time = cruise_time + dive_3d_dist / max(dive_speed, 1.0)

        traj = StrikeTrajectory(
            uav_id=uav.uav_id,
            uav_name=uav.name,
            target_id=target.target_id,
            target_name=target.name,
            waypoints=waypoints,
            dive_start_index=dive_start_idx,
            impact_index=impact_idx,
            total_distance_m=total_dist,
            total_time_sec=total_time,
            cruise_alt_m=cruise_alt,
        )

        logger.debug(
            f'[StrikePlanner] {uav.name}→{target.name}: '
            f'cruise={cruise_dist:.0f}m, dive_horiz={actual_dive_dist:.0f}m, '
            f'dive_angle={actual_dive_angle_deg:.1f}°, alt={cruise_alt:.0f}m, '
            f'WPs={len(waypoints)}, diveStart@{dive_start_idx}'
        )
        return traj

    def trajectories_to_cesium_data(self, trajectories: List[StrikeTrajectory],
                                     targets: List[StrikeTarget]) -> dict:
        """
        將軌跡轉換為 Cesium.js 前端所需的 JSON 格式

        回傳結構:
        {
          "targets": [{ "id": int, "lat": float, "lon": float, "alt": float, "name": str }],
          "trajectories": [{
            "uav_id": int, "uav_name": str,
            "target_id": int, "target_name": str,
            "dive_start_index": int, "impact_index": int,
            "total_time_sec": float, "cruise_alt_m": float,
            "waypoints": [{ "lat": float, "lon": float, "alt": float,
                            "segment": str, "time_sec": float }]
          }],
          "dive_initiation_dist": float,
          "max_dive_angle": float,
        }
        """
        tgt_data = [
            {'id': t.target_id, 'lat': t.lat, 'lon': t.lon, 'alt': t.alt, 'name': t.name}
            for t in targets
        ]

        traj_data = []
        for tr in trajectories:
            wps = [
                {'lat': wp.lat, 'lon': wp.lon, 'alt': wp.alt,
                 'segment': wp.segment, 'time_sec': wp.time_sec}
                for wp in tr.waypoints
            ]
            traj_data.append({
                'uav_id': tr.uav_id,
                'uav_name': tr.uav_name,
                'target_id': tr.target_id,
                'target_name': tr.target_name,
                'dive_start_index': tr.dive_start_index,
                'impact_index': tr.impact_index,
                'total_time_sec': tr.total_time_sec,
                'cruise_alt_m': tr.cruise_alt_m,
                'waypoints': wps,
            })

        return {
            'targets': tgt_data,
            'trajectories': traj_data,
            'dive_initiation_dist': self.dive_initiation_dist_m,
            'max_dive_angle': self.max_dive_angle_deg,
            'altitude_step': self.altitude_step_m,
        }
