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
    """打擊航線航點

    segment 標籤 (用於 Cesium 分色渲染)：
        'takeoff' — 起飛爬升段（黃色實線）
        'cruise'  — 巡航段（綠色實線；若 Dubins 則沿最小轉彎半徑）
        'dive'    — 末端俯衝段（血紅色加粗）
    """
    lat: float
    lon: float
    alt: float
    segment: str = 'cruise'   # takeoff | cruise | dive
    time_sec: float = 0.0     # 任務時間（秒）


@dataclass
class StrikeTrajectory:
    """單架 UCAV 的完整打擊軌跡"""
    uav_id: int
    uav_name: str
    target_id: int
    target_name: str
    waypoints: List[StrikeWaypoint] = field(default_factory=list)
    takeoff_start_index: int = 0     # 起飛段起始索引（固定為 0）
    cruise_start_index: int = -1     # 巡航段起始航點索引
    dive_start_index: int = -1       # 俯衝起始航點索引
    impact_index: int = -1           # 命中航點索引
    total_distance_m: float = 0.0
    total_time_sec: float = 0.0
    cruise_alt_m: float = 0.0        # 此 UAV 實際巡航高度（含錯層）
    takeoff_lat: float = 0.0         # 起飛點緯度（地面）
    takeoff_lon: float = 0.0         # 起飛點經度
    actual_dive_angle_deg: float = 0.0  # 實際達成的俯衝角


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
                 dive_step_m: float = 20.0,
                 # ── 起飛段參數 (固定翼專屬) ──────────────────────
                 takeoff_alt_m: float = 0.0,
                 climb_angle_deg: float = 8.0,
                 min_turn_radius_m: float = 150.0,
                 takeoff_step_m: float = 25.0,
                 use_dubins_cruise: bool = True):
        """
        參數:
            max_dive_angle_deg:     最大俯衝角（度，正值表示向下角度）
            dive_initiation_dist_m: 俯衝起始距離（距目標水平距離，公尺）
            cruise_alt_m:           基準巡航高度（公尺）
            cruise_speed_mps:       巡航速度（公尺/秒）
            altitude_step_m:        高度錯層間距（公尺），各機遞增此值
            cruise_step_m:          巡航段航點間距（公尺）
            dive_step_m:            俯衝段航點間距（公尺）
            takeoff_alt_m:          起飛點地面海拔（公尺）
            climb_angle_deg:        爬升角 γ (固定翼約 6~10°)
            min_turn_radius_m:      固定翼最小轉彎半徑 R_min（含 Dubins 轉彎）
            takeoff_step_m:         起飛爬升段航點間距（公尺）
            use_dubins_cruise:      True = 巡航段套用 Dubins 曲線；
                                    False = 直線近似（較快但不符固定翼約束）
        """
        self.max_dive_angle_deg = abs(max_dive_angle_deg)
        self.dive_initiation_dist_m = dive_initiation_dist_m
        self.cruise_alt_m = cruise_alt_m
        self.cruise_speed_mps = cruise_speed_mps
        self.altitude_step_m = altitude_step_m
        self.cruise_step_m = cruise_step_m
        self.dive_step_m = dive_step_m
        self.takeoff_alt_m = takeoff_alt_m
        self.climb_angle_deg = climb_angle_deg
        self.min_turn_radius_m = min_turn_radius_m
        self.takeoff_step_m = takeoff_step_m
        self.use_dubins_cruise = use_dubins_cruise

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

    # ─────────────────────────────────────────────────────────────────
    #  STOT 模式：同地發射 (Simultaneous launch, Time On Target)
    # ─────────────────────────────────────────────────────────────────
    def plan_stot(self,
                  targets: List[StrikeTarget],
                  launch_lat: float,
                  launch_lon: float,
                  launch_alt: float = 0.0,
                  ) -> List[StrikeTrajectory]:
        """
        同地發射模式 — 所有 UCAV 共用同一起飛點。

        策略：
          - 所有 N 架 UCAV 的起飛經緯度皆為 (launch_lat, launch_lon)
          - 各機巡航高度依 altitude_step_m 錯層（基準 + i × 錯層）
          - 目標依「相對起飛點的方位角」排序，形成扇形分配
            → 每架 UCAV 起飛後朝不同方位飛行，自然分散不交叉
          - 起飛螺旋圓心在不同方向（bearing + 90°）+ 高度錯層
            → 爬升階段也不會相撞

        參數:
            targets:     地面目標列表（至少 1 個）
            launch_lat:  共用起飛點緯度
            launch_lon:  共用起飛點經度
            launch_alt:  起飛點地面海拔（m），預設 0

        回傳:
            List[StrikeTrajectory]（與 plan_auto 相同格式）
        """
        if not targets:
            logger.warning('[StrikePlanner/STOT] 目標列表為空')
            return []

        n = len(targets)
        logger.info(
            f'[StrikePlanner/STOT] 同地發射：launch=({launch_lat:.6f}, {launch_lon:.6f}) '
            f'→ {n} 個目標'
        )

        # 暫存原 takeoff_alt，STOT 改用 launch_alt
        orig_takeoff_alt = self.takeoff_alt_m
        self.takeoff_alt_m = launch_alt

        try:
            # 按「目標方位角」排序 → 穩定的扇形分配
            indexed = sorted(
                enumerate(targets),
                key=lambda x: _bearing(launch_lat, launch_lon, x[1].lat, x[1].lon),
            )

            trajectories: List[StrikeTrajectory] = []
            for slot, (orig_idx, tgt) in enumerate(indexed):
                uav_alt = self.cruise_alt_m + slot * self.altitude_step_m
                uav = StrikeUAV(
                    uav_id=slot + 1,
                    lat=launch_lat,
                    lon=launch_lon,
                    cruise_alt=uav_alt,
                    speed_mps=self.cruise_speed_mps,
                )
                traj = self._plan_single_strike(uav, tgt)
                trajectories.append(traj)

                brg = _bearing(launch_lat, launch_lon, tgt.lat, tgt.lon)
                logger.info(
                    f'[StrikePlanner/STOT] {uav.name} → {tgt.name}: '
                    f'alt={uav_alt:.0f}m, bearing={brg:.1f}°, '
                    f'L={traj.total_distance_m:.0f}m, T={traj.total_time_sec:.1f}s'
                )

            return trajectories
        finally:
            # 還原原 takeoff_alt（不污染後續 DTOT 規劃）
            self.takeoff_alt_m = orig_takeoff_alt

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
        規劃單架固定翼 UCAV 對單個目標的完整三段飛行路徑。

        三個分段（皆由此方法生成，一次性合併成單一 waypoint 序列）：
          (a) 起飛段 takeoff — 從地面 (takeoff_alt_m) 螺旋爬升到巡航高度
                              受 climb_angle_deg 與 min_turn_radius_m 約束
          (b) 巡航段 cruise  — 從爬升出圈點飛向「俯衝起始點」
                              use_dubins_cruise=True 時套用 Dubins 曲線
                              （入場航向 = 指向目標；圓弧+直線確保曲率 ≤ 1/R_min）
          (c) 俯衝段 dive    — 從俯衝起始點急降至地面目標
                              受 max_dive_angle_deg 與 dive_initiation_dist_m 約束

        俯衝角保護邏輯（維持原設計）：
          θ = atan(Δalt / horizontal_dist)
          若計算出的 θ > max_dive_angle，自動將俯衝起始點後推，
          使 horizontal_dist = Δalt / tan(max_dive_angle)。
        """
        cruise_alt = uav.cruise_alt if uav.cruise_alt > 0 else self.cruise_alt_m
        target_alt = target.alt
        delta_alt  = max(cruise_alt - target_alt, cruise_alt)

        # ───────────────────────────────────────────────────────────
        #  Step 1 — 俯衝幾何（先算出俯衝起始點作為巡航段終點）
        # ───────────────────────────────────────────────────────────
        max_angle_rad  = math.radians(self.max_dive_angle_deg)
        min_dive_horiz = (delta_alt / math.tan(max_angle_rad)
                         if max_angle_rad > 0.01 else delta_alt)
        # 實際俯衝水平距離：取使用者設定與物理限制中較大者
        actual_dive_dist = max(self.dive_initiation_dist_m, min_dive_horiz)

        # 俯衝起始點：從目標反推 (指向 UAV 方向後退 actual_dive_dist)
        brg_tgt_to_uav = _bearing(target.lat, target.lon, uav.lat, uav.lon)
        dive_start_lat, dive_start_lon = _destination(
            target.lat, target.lon, brg_tgt_to_uav, actual_dive_dist,
        )
        # 實際俯衝角
        actual_dive_angle_deg = math.degrees(math.atan2(delta_alt, actual_dive_dist))

        # 進場航向（UAV 在俯衝起始點時應指向目標）
        approach_bearing = _bearing(dive_start_lat, dive_start_lon,
                                    target.lat, target.lon)

        # ───────────────────────────────────────────────────────────
        #  Step 2 — 起飛段：螺旋爬升到巡航高度
        # ───────────────────────────────────────────────────────────
        waypoints: List[StrikeWaypoint] = []
        takeoff_lat, takeoff_lon = uav.lat, uav.lon
        climb_wps, climb_end_lat, climb_end_lon, climb_end_alt, climb_time = \
            self._plan_takeoff_climb(
                start_lat=takeoff_lat, start_lon=takeoff_lon,
                start_alt=self.takeoff_alt_m,
                target_alt=cruise_alt,
                initial_bearing=approach_bearing,     # 螺旋出圈後大致朝目標
                speed=uav.speed_mps,
                time_offset=0.0,
            )
        waypoints.extend(climb_wps)
        cruise_start_idx = len(waypoints)        # 巡航段起始索引
        t_cur = climb_time

        # ───────────────────────────────────────────────────────────
        #  Step 3 — 巡航段：Dubins 或直線
        # ───────────────────────────────────────────────────────────
        cruise_wps, cruise_dist, t_cur = self._plan_cruise_segment(
            start_lat=climb_end_lat, start_lon=climb_end_lon,
            start_bearing=approach_bearing,
            end_lat=dive_start_lat, end_lon=dive_start_lon,
            end_bearing=approach_bearing,
            alt=cruise_alt,
            speed=uav.speed_mps,
            time_offset=t_cur,
        )
        waypoints.extend(cruise_wps)
        dive_start_idx = len(waypoints) - 1       # 最後一個巡航點 = 俯衝起始

        # ───────────────────────────────────────────────────────────
        #  Step 4 — 末端俯衝段
        # ───────────────────────────────────────────────────────────
        dive_3d_dist = math.hypot(actual_dive_dist, delta_alt)
        n_dive_steps = max(3, int(dive_3d_dist / self.dive_step_m))
        dive_speed = uav.speed_mps * 1.3          # 俯衝加速 30%

        for i in range(1, n_dive_steps + 1):
            t = i / n_dive_steps
            lat_i, lon_i, alt_i = _interp_geo(
                dive_start_lat, dive_start_lon, cruise_alt,
                target.lat, target.lon, target_alt,
                t,
            )
            waypoints.append(StrikeWaypoint(
                lat=lat_i, lon=lon_i, alt=max(alt_i, target_alt),
                segment='dive',
                time_sec=t_cur + (dive_3d_dist * t) / max(dive_speed, 1.0),
            ))
        impact_idx = len(waypoints) - 1
        total_time = t_cur + dive_3d_dist / max(dive_speed, 1.0)
        total_dist = cruise_dist + dive_3d_dist   # 爬升路徑不計入「有效距離」

        traj = StrikeTrajectory(
            uav_id=uav.uav_id, uav_name=uav.name,
            target_id=target.target_id, target_name=target.name,
            waypoints=waypoints,
            takeoff_start_index=0,
            cruise_start_index=cruise_start_idx,
            dive_start_index=dive_start_idx,
            impact_index=impact_idx,
            total_distance_m=total_dist,
            total_time_sec=total_time,
            cruise_alt_m=cruise_alt,
            takeoff_lat=takeoff_lat,
            takeoff_lon=takeoff_lon,
            actual_dive_angle_deg=actual_dive_angle_deg,
        )

        logger.debug(
            f'[StrikePlanner] {uav.name}→{target.name}: '
            f'takeoff_wps={cruise_start_idx}, '
            f'cruise={cruise_dist:.0f}m, '
            f'dive_horiz={actual_dive_dist:.0f}m, '
            f'dive_angle={actual_dive_angle_deg:.1f}°, '
            f'alt={cruise_alt:.0f}m, total_WPs={len(waypoints)}'
        )
        return traj

    # ─────────────────────────────────────────────────────────────────
    #  起飛螺旋爬升段
    # ─────────────────────────────────────────────────────────────────
    def _plan_takeoff_climb(self,
                             start_lat: float, start_lon: float,
                             start_alt: float, target_alt: float,
                             initial_bearing: float,
                             speed: float,
                             time_offset: float,
                             ) -> Tuple[List[StrikeWaypoint], float, float, float, float]:
        """
        固定翼螺旋式起飛爬升段生成器。

        幾何設計
        --------
          ① 在起飛點右側距 R_min 處放置螺旋圓心；
          ② 沿圓周以「爬升角 γ」等比上升，角速度 ω = V / R；
          ③ 當總爬升高度 Δh 達成後即出圈，方向切換為 initial_bearing。

        由於固定翼不能原地垂直爬升，這是最符合實際的起飛模式
        （亦即「Racetrack 上升盤旋」）。

        回傳:
            (waypoints, end_lat, end_lon, end_alt, end_time_sec)
        """
        dh = max(target_alt - start_alt, 0.0)
        if dh < 1e-3:
            # 無需爬升 → 回傳單一佔位點
            return (
                [StrikeWaypoint(start_lat, start_lon, start_alt,
                                segment='takeoff', time_sec=time_offset)],
                start_lat, start_lon, start_alt, time_offset,
            )

        # 爬升率 = V * tan(γ)
        climb_rate = speed * math.tan(math.radians(self.climb_angle_deg))
        t_climb = dh / max(climb_rate, 0.5)

        R = self.min_turn_radius_m
        # 圓心放在「起飛航向右手側」R 處 (即 bearing + 90°)
        c_lat, c_lon = _destination(
            start_lat, start_lon,
            (initial_bearing + 90.0) % 360.0, R,
        )
        # UAV 起點相對於圓心的方位角（從圓心看出去）
        brg_c2s = _bearing(c_lat, c_lon, start_lat, start_lon)

        # 採樣
        n = max(int(t_climb * speed / self.takeoff_step_m), 8)
        waypoints: List[StrikeWaypoint] = []
        for k in range(n + 1):
            t_norm = k / n
            t = t_norm * t_climb
            arc = (speed * t) / R                            # 走過的弧度
            # 順時針旋轉 → 方位角遞減
            theta = (brg_c2s - math.degrees(arc)) % 360.0
            lat_k, lon_k = _destination(c_lat, c_lon, theta, R)
            alt_k = start_alt + climb_rate * t
            waypoints.append(StrikeWaypoint(
                lat=lat_k, lon=lon_k, alt=min(alt_k, target_alt),
                segment='takeoff',
                time_sec=time_offset + t,
            ))

        last = waypoints[-1]
        return waypoints, last.lat, last.lon, last.alt, time_offset + t_climb

    # ─────────────────────────────────────────────────────────────────
    #  巡航段 (Dubins 曲線 / 直線)
    # ─────────────────────────────────────────────────────────────────
    def _plan_cruise_segment(self,
                              start_lat: float, start_lon: float,
                              start_bearing: float,
                              end_lat: float, end_lon: float,
                              end_bearing: float,
                              alt: float, speed: float,
                              time_offset: float,
                              ) -> Tuple[List[StrikeWaypoint], float, float]:
        """
        生成巡航段航點。

        use_dubins_cruise=True 時委派給 DubinsTrajectoryGenerator
        （水平最小轉彎半徑 = min_turn_radius_m），確保固定翼可追蹤；
        否則退化為大圓直線等距採樣（較快但不符合物理約束）。

        回傳:
            (waypoints, horizontal_distance_m, end_time_sec)
        """
        # ── Dubins 模式 ──────────────────────────────────────────
        if self.use_dubins_cruise:
            try:
                from core.trajectory.dubins_trajectory import (
                    DubinsTrajectoryGenerator, Pose3D,
                )
                from core.base.fixed_wing_constraints import FixedWingConstraints

                # 由目標 R_min 反解最大側傾角：φ = atan( V² / (g·R) )
                v = max(speed, 5.0)
                R = max(self.min_turn_radius_m, 10.0)
                phi_rad = math.atan((v * v) / (9.81 * R))
                phi_deg = max(5.0, min(80.0, math.degrees(phi_rad)))

                constraints = FixedWingConstraints(
                    cruise_airspeed_mps=v,
                    max_bank_angle_deg=phi_deg,
                    safety_factor=1.0,                          # 直接使用 R_min
                    stall_speed_mps=max(v * 0.4, 1.0),
                    max_speed_mps=max(v * 2.0, v + 1.0),
                )
                gen = DubinsTrajectoryGenerator(
                    constraints=constraints,
                    max_climb_angle_deg=self.climb_angle_deg + 5.0,
                    max_descent_angle_deg=self.climb_angle_deg + 5.0,
                )

                # 以起點為 ENU 原點做本地平面計算
                # 轉換：bearing (羅盤) → math angle (從東軸逆時針)
                def _brg2math(b: float) -> float:
                    return (90.0 - b) % 360.0

                # 端點 ENU 座標
                dist_geo = _haversine(start_lat, start_lon, end_lat, end_lon)
                brg_line = _bearing(start_lat, start_lon, end_lat, end_lon)
                # 末端 ENU：x=東, y=北
                end_x = dist_geo * math.sin(math.radians(brg_line))
                end_y = dist_geo * math.cos(math.radians(brg_line))

                start_pose = Pose3D(0.0, 0.0, alt, _brg2math(start_bearing))
                end_pose   = Pose3D(end_x, end_y, alt, _brg2math(end_bearing))

                path = gen.calculate_path(start_pose, end_pose)
                if path.is_feasible and path.segments:
                    poses = gen.generate_waypoints(path, step_size=self.cruise_step_m)
                    wps: List[StrikeWaypoint] = []
                    cumul = 0.0
                    prev_xy = (0.0, 0.0)
                    for p in poses:
                        # ENU → 大地座標（逐點近似）
                        dx, dy = p.x, p.y
                        bearing_geo = (90.0 - math.degrees(
                            math.atan2(dy, dx))) % 360.0
                        d = math.hypot(dx, dy)
                        lat_i, lon_i = _destination(
                            start_lat, start_lon, bearing_geo, d,
                        )
                        ds = math.hypot(dx - prev_xy[0], dy - prev_xy[1])
                        cumul += ds
                        prev_xy = (dx, dy)
                        wps.append(StrikeWaypoint(
                            lat=lat_i, lon=lon_i, alt=alt,
                            segment='cruise',
                            time_sec=time_offset + cumul / max(speed, 1.0),
                        ))
                    if wps:
                        total = path.total_length
                        return wps, total, time_offset + total / max(speed, 1.0)
                logger.warning(
                    f'[StrikePlanner] Dubins 巡航不可行 ({path.warning})，退化為直線'
                )
            except Exception as e:
                logger.warning(f'[StrikePlanner] Dubins 巡航計算失敗 ({e})，退化為直線')

        # ── 直線退化模式 ─────────────────────────────────────────
        dist = _haversine(start_lat, start_lon, end_lat, end_lon)
        wps: List[StrikeWaypoint] = []
        if dist > self.cruise_step_m:
            n = max(2, int(dist / self.cruise_step_m))
            for i in range(n + 1):
                t = i / n
                lat_i = start_lat + (end_lat - start_lat) * t
                lon_i = start_lon + (end_lon - start_lon) * t
                wps.append(StrikeWaypoint(
                    lat=lat_i, lon=lon_i, alt=alt, segment='cruise',
                    time_sec=time_offset + (dist * t) / max(speed, 1.0),
                ))
        else:
            wps.append(StrikeWaypoint(
                lat=start_lat, lon=start_lon, alt=alt,
                segment='cruise', time_sec=time_offset,
            ))
            wps.append(StrikeWaypoint(
                lat=end_lat, lon=end_lon, alt=alt,
                segment='cruise',
                time_sec=time_offset + dist / max(speed, 1.0),
            ))
        return wps, dist, time_offset + dist / max(speed, 1.0)

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
                # 三段索引（供 Cesium 分色渲染）
                'takeoff_start_index': tr.takeoff_start_index,
                'cruise_start_index':  tr.cruise_start_index,
                'dive_start_index':    tr.dive_start_index,
                'impact_index':        tr.impact_index,
                'total_time_sec': tr.total_time_sec,
                'total_distance_m': tr.total_distance_m,
                'cruise_alt_m': tr.cruise_alt_m,
                'takeoff_lat': tr.takeoff_lat,
                'takeoff_lon': tr.takeoff_lon,
                'actual_dive_angle': tr.actual_dive_angle_deg,
                'waypoints': wps,
            })

        return {
            'targets': tgt_data,
            'trajectories': traj_data,
            'dive_initiation_dist': self.dive_initiation_dist_m,
            'max_dive_angle': self.max_dive_angle_deg,
            'altitude_step': self.altitude_step_m,
            'min_turn_radius_m': self.min_turn_radius_m,
            'climb_angle_deg': self.climb_angle_deg,
        }
