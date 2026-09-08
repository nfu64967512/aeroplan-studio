"""
群飛協調器模組
負責多無人機任務的協調、同步、避撞等功能
支持時間錯開、高度分層、區域分配等策略
"""

import sys
from pathlib import Path
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass
import math
import logging

from utils.math_utils import EARTH_RADIUS_M  # 0-2 標準化：WGS84 單一來源

# 添加專案根目錄到路徑
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

logger = logging.getLogger(__name__)

from mission.mission_manager import Mission
from mission.survey_mission import SurveyMission
from mission.waypoint import MAVCommand
from mission.coverage_path import CoveragePath, OperationSegment


# ==========================================
# 無人機資料類
# ==========================================
@dataclass
class DroneInfo:
    """無人機資訊"""
    drone_id: int                                    # 無人機 ID
    name: str                                        # 名稱
    position: Optional[Tuple[float, float]] = None   # 當前位置 (lat, lon)
    speed_mps: float = 10.0                          # 飛行速度（公尺/秒）
    mission: Optional[Mission] = None                # 分配的任務
    start_time: float = 0.0                          # 起飛時間（秒）
    rtl_altitude: float = 50.0                       # RTL 高度（公尺）
    status: str = "ready"                            # 狀態


# ==========================================
# 群飛任務類
# ==========================================
class SwarmMission:
    """
    群飛任務類
    
    管理多個無人機的協同任務
    """
    
    def __init__(self, name: str = "Swarm Mission"):
        """
        初始化群飛任務
        
        參數:
            name: 任務名稱
        """
        self.name = name
        self.drones: List[DroneInfo] = []
        
        # 群飛參數
        self.swarm_params = {
            'strategy': 'sequential',       # 策略：sequential, simultaneous
            'safety_distance': 5.0,        # 安全距離（公尺）
            'altitude_increment': 3.0,     # RTL 高度遞增（公尺）
            'time_buffer': 2.0,            # 時間緩衝（秒）
            'coordination_mode': 'smart',  # 協調模式：smart, simple
        }
        
        # 統計資訊
        self.stats = {
            'total_drones': 0,
            'total_waypoints': 0,
            'total_distance': 0.0,
            'estimated_time': 0.0,
        }
    
    def add_drone(self, drone_id: int, name: str) -> DroneInfo:
        """
        添加無人機
        
        參數:
            drone_id: 無人機 ID
            name: 名稱
        
        返回:
            DroneInfo 實例
        """
        drone = DroneInfo(drone_id=drone_id, name=name)
        self.drones.append(drone)
        self.stats['total_drones'] = len(self.drones)
        return drone
    
    def assign_mission(self, drone_id: int, mission: Mission):
        """
        分配任務給無人機
        
        參數:
            drone_id: 無人機 ID
            mission: 任務實例
        """
        for drone in self.drones:
            if drone.drone_id == drone_id:
                drone.mission = mission
                break
    
    def set_strategy(self, strategy: str):
        """
        設定群飛策略
        
        參數:
            strategy: 策略名稱
                - sequential: 順序執行（時間錯開）
                - simultaneous: 同時執行（空間分離）
        """
        if strategy not in ('sequential', 'simultaneous'):
            raise ValueError(f"不支援的策略: {strategy}")
        
        self.swarm_params['strategy'] = strategy
    
    def calculate_loiter_times(self) -> List[float]:
        """計算各機 LOITER 等待時間（簡單時間錯開）。

        返回:
            每台無人機的等待時間列表

        註：原本嘗試 import 不存在的 collision_avoidance.CollisionAvoidanceSystem
        做碰撞感知 loiter，該模組/類別從未存在 → ImportError → runtime 一直走簡單
        時間錯開 fallback。已移除無法解析的死 import，直接回傳該 fallback（零行為變更）。
        若日後要做碰撞感知 loiter，應改用 core.dccpp.collision_avoidance 並實作對應方法。
        """
        return [i * 5.0 for i in range(len(self.drones))]
    
    def calculate_rtl_altitudes(self, base_altitude: float = 50.0) -> List[float]:
        """
        計算 RTL 高度（高度分層）
        
        參數:
            base_altitude: 基準高度（公尺）
        
        返回:
            每台無人機的 RTL 高度列表
        """
        rtl_altitudes = []
        increment = self.swarm_params['altitude_increment']
        
        for idx in range(len(self.drones)):
            # 反向分配（最後一台最高）
            altitude = base_altitude + (len(self.drones) - idx - 1) * increment
            rtl_altitudes.append(altitude)
        
        return rtl_altitudes
    
    def apply_collision_avoidance(self):
        """
        應用避撞策略到所有無人機任務
        """
        strategy = self.swarm_params['strategy']
        
        if strategy == 'sequential':
            # 順序執行策略：計算 LOITER 時間
            loiter_times = self.calculate_loiter_times()
            
            for idx, drone in enumerate(self.drones):
                if drone.mission:
                    drone.start_time = loiter_times[idx]
                    
                    # 插入 LOITER 命令到任務中
                    if loiter_times[idx] > 0 and len(drone.mission.waypoints) > 0:
                        self._insert_loiter_to_mission(
                            drone.mission, loiter_times[idx]
                        )
        
        elif strategy == 'simultaneous':
            # 同時執行策略：確保空間分離
            # 這裡假設任務已經通過區域分割實現空間分離
            pass
        
        # 計算並應用 RTL 高度分層
        base_alt = max(
            (drone.mission.params.get('altitude', 50.0) 
             for drone in self.drones if drone.mission),
            default=50.0
        )
        
        rtl_altitudes = self.calculate_rtl_altitudes(base_alt)
        
        for idx, drone in enumerate(self.drones):
            if drone.mission:
                drone.rtl_altitude = rtl_altitudes[idx]
                self._update_rtl_altitude(drone.mission, rtl_altitudes[idx])
    
    def _insert_loiter_to_mission(self, mission: Mission, loiter_time: float):
        """
        插入 LOITER 命令到任務
        
        參數:
            mission: 任務實例
            loiter_time: 等待時間（秒）
        """
        # 註：原本嘗試 import 不存在的 collision_avoidance.CollisionAvoidanceSystem
        # 的 insert_loiter_command，把 LOITER 命令插進任務航點；但該模組/類別/方法
        # 從未存在，故此方法 runtime 一直是 no-op（每次呼叫只印一行錯誤）。loiter
        # 時序實際由 drone.start_time 表示（見 apply_collision_avoidance）。
        # 已移除無法解析的死 import，改為明確 no-op。若日後要真的把 LOITER 命令
        # 插入任務航點序列，需另行實作（mission.waypoints 的 QGC 互轉已具備）。
        return
    
    def _update_rtl_altitude(self, mission: Mission, rtl_altitude: float):
        """
        更新任務的 RTL 高度
        
        參數:
            mission: 任務實例
            rtl_altitude: RTL 高度（公尺）
        """
        for waypoint in mission.waypoints:
            # 更新 TAKEOFF 到 RTL 高度的命令
            if waypoint.command == MAVCommand.NAV_TAKEOFF:
                # 在 RTL 前插入 TAKEOFF 到新高度的命令
                pass
    
    def calculate_statistics(self):
        """計算群飛統計資訊"""
        total_waypoints = 0
        total_distance = 0.0
        max_time = 0.0
        
        for drone in self.drones:
            if drone.mission:
                total_waypoints += len(drone.mission.waypoints)
                total_distance += drone.mission.waypoints.calculate_total_distance()
                
                # 計算該無人機的完成時間
                flight_time = drone.mission.waypoints.estimate_flight_time(
                    drone.mission.params.get('speed', 10.0)
                )
                completion_time = drone.start_time + flight_time
                max_time = max(max_time, completion_time)
        
        self.stats.update({
            'total_waypoints': total_waypoints,
            'total_distance': total_distance,
            'estimated_time': max_time,
        })
    
    def validate(self) -> Tuple[bool, List[str]]:
        """
        驗證群飛任務
        
        返回:
            (是否有效, 錯誤訊息列表)
        """
        errors = []
        
        # 檢查無人機數量
        if len(self.drones) == 0:
            errors.append("沒有無人機")
        
        # 檢查每台無人機的任務
        for drone in self.drones:
            if drone.mission is None:
                errors.append(f"無人機 {drone.name} 沒有分配任務")
            else:
                # 驗證任務
                valid, mission_errors = drone.mission.validate()
                if not valid:
                    errors.extend([f"{drone.name}: {e}" for e in mission_errors])
        
        # 檢查空間分離（simultaneous 策略）
        if self.swarm_params['strategy'] == 'simultaneous':
            if not self._check_spatial_separation():
                errors.append("無人機任務區域存在重疊，無法同時執行")
        
        return len(errors) == 0, errors
    
    def _check_spatial_separation(self) -> bool:
        """
        檢查空間分離
        
        返回:
            是否有足夠的空間分離
        """
        # 簡化實現：檢查邊界框是否重疊
        bboxes = []
        
        for drone in self.drones:
            if drone.mission:
                bbox = drone.mission.waypoints.get_bounding_box()
                if bbox:
                    bboxes.append(bbox)
        
        # 檢查每對邊界框是否重疊
        for i in range(len(bboxes)):
            for j in range(i + 1, len(bboxes)):
                if self._bboxes_overlap(bboxes[i], bboxes[j]):
                    return False
        
        return True
    
    def _bboxes_overlap(self, bbox1: Tuple[float, float, float, float],
                       bbox2: Tuple[float, float, float, float]) -> bool:
        """
        檢查兩個邊界框是否重疊
        
        參數:
            bbox1, bbox2: (min_lat, min_lon, max_lat, max_lon)
        
        返回:
            是否重疊
        """
        return not (bbox1[2] < bbox2[0] or bbox1[0] > bbox2[2] or
                   bbox1[3] < bbox2[1] or bbox1[1] > bbox2[3])
    
    def get_drone_by_id(self, drone_id: int) -> Optional[DroneInfo]:
        """
        根據 ID 獲取無人機
        
        參數:
            drone_id: 無人機 ID
        
        返回:
            DroneInfo 實例，未找到返回 None
        """
        for drone in self.drones:
            if drone.drone_id == drone_id:
                return drone
        return None
    
    def generate_mission_briefing(self) -> str:
        """
        生成群飛任務簡報
        
        返回:
            簡報文字
        """
        briefing = []
        briefing.append("=" * 50)
        briefing.append(f"群飛任務簡報: {self.name}")
        briefing.append("=" * 50)
        briefing.append("")
        
        briefing.append(f"群飛策略: {self.swarm_params['strategy']}")
        briefing.append(f"無人機數量: {len(self.drones)}")
        briefing.append(f"安全距離: {self.swarm_params['safety_distance']}m")
        briefing.append(f"高度遞增: {self.swarm_params['altitude_increment']}m")
        briefing.append("")
        
        briefing.append("統計資訊:")
        briefing.append(f"  - 總航點數: {self.stats['total_waypoints']}")
        briefing.append(f"  - 總航程: {self.stats['total_distance']:.1f}m")
        briefing.append(f"  - 預估時間: {self.stats['estimated_time']/60:.1f}分鐘")
        briefing.append("")
        
        briefing.append("無人機詳情:")
        for idx, drone in enumerate(self.drones, 1):
            briefing.append(f"\n無人機 {idx}: {drone.name}")
            briefing.append(f"  - ID: {drone.drone_id}")
            briefing.append(f"  - 起飛時間: {drone.start_time:.1f}秒")
            briefing.append(f"  - RTL高度: {drone.rtl_altitude:.1f}m")
            
            if drone.mission:
                stats = drone.mission.get_statistics()
                briefing.append(f"  - 航點數: {stats['total_waypoints']}")
                briefing.append(f"  - 航程: {stats['total_distance']:.1f}m")
                briefing.append(f"  - 飛行時間: {stats['estimated_time']/60:.1f}分鐘")
        
        briefing.append("")
        briefing.append("=" * 50)
        
        return '\n'.join(briefing)
    
    def __str__(self) -> str:
        """字串表示"""
        return f"SwarmMission(name='{self.name}', drones={len(self.drones)})"
    
    def __repr__(self) -> str:
        """詳細表示"""
        return self.__str__()


def _zone_polygon_latlon(z):
    """FenceZone → [(lat, lon), ...]。圓形展開成 24 邊形。認不得就回 None。"""
    verts = getattr(z, 'vertices', None)
    if verts and len(verts) >= 3:
        return [(float(a), float(b)) for a, b in verts]
    c, r = getattr(z, 'center', None), float(getattr(z, 'radius_m', 0.0) or 0.0)
    if c and r > 0.0:
        lat0 = float(c[0])
        m_lat, m_lon = 111132.0, 111412.0 * math.cos(math.radians(lat0))
        return [(lat0 + r * math.sin(2 * math.pi * k / 24) / m_lat,
                 float(c[1]) + r * math.cos(2 * math.pi * k / 24) / m_lon)
                for k in range(24)]
    return None


def split_areas_around_nfz(areas, zones, *, inset_m: float, logger=None):
    """把與禁航區重疊的作業區切成「繞過禁航區的子區」。

    ⚠⚠⚠ 為什麼需要這一步（實測，不是推測）：
    禁航區蓋住某一塊作業區時，HV 規劃器是把**整塊區**判成 uncoverable，而且
    連帶把**其他不相干的區也一起丟掉**。實測 4 塊區、1A+3B、禁航區只壓在第 2 塊上：

        蓋住 ≥42.9% ⇒ **0 台上工、4 個區全部 no_feasible_insertion**
        蓋住 ≤28%   ⇒ 掉一台，剩 3 台

    同構型機隊也一樣，不是混編才有。所以在送進規劃器**之前**先把區切開，
    讓它看到的每一塊都與禁航區無關 —— 這就是 Choset 的 boustrophedon cellular
    decomposition，用障礙物的臨界點切胞。

    `inset_m` 是禁航區往外膨脹的量。⚠ 航跡繞開**不等於**相機沒拍到：刈幅 w 的
    相機沿邊界飛會拍進去 w/2，所以要「一格都不拍到」就得給 `inset_m = w/2`，
    代價是禁航區周圍一圈寬 w/2 掃不到。

    回傳新的 areas 清單；沒有重疊時原樣回傳（同一個物件，不複製）。
    """
    if not areas or not zones:
        return areas
    try:
        from shapely.geometry import Polygon, box
        from shapely.ops import unary_union
    except ImportError:                                    # pragma: no cover
        if logger:
            logger.warning('shapely 不在，禁航區切區跳過')
        return areas

    zps = []
    for z in zones:
        pts = _zone_polygon_latlon(z)
        if pts and len(pts) >= 3:
            p = Polygon(pts)
            if not p.is_valid:
                p = p.buffer(0)
            if not p.is_empty:
                zps.append(p)
    if not zps:
        return areas
    # 經緯度直接用度為單位做幾何：inset 換算成緯度度數（保守，經度方向更寬）
    inset_deg = float(inset_m) / 111132.0
    grow = unary_union([p.buffer(inset_deg, join_style=2) for p in zps])

    # 子區用**全新的整數 id**（理由見下方賦值處），從現有最大值往上長。
    next_id = max((int(a['area_id']) for a in areas
                   if str(a.get('area_id', '')).isdigit()), default=0) + 1
    out, n_split = [], 0
    for a in areas:
        poly = Polygon(a['polygon'])
        if not poly.is_valid:
            poly = poly.buffer(0)
        if poly.is_empty or not poly.intersects(grow):
            out.append(a)
            continue
        free = poly.difference(grow)
        if free.is_empty:
            if logger:
                logger.warning(f"nfz_covers_area(area_id={a.get('area_id')}) "
                               f"整塊被禁航區蓋掉，從任務移除")
            continue
        # BCD：在禁航區的經度極值處切一刀
        x0, y0, x1, y1 = free.bounds
        cuts = sorted({c for z in zps
                       for c in (z.buffer(inset_deg, join_style=2).bounds[0],
                                 z.buffer(inset_deg, join_style=2).bounds[2])
                       if x0 < c < x1})
        xs = [x0 - 1e-6] + cuts + [x1 + 1e-6]
        pieces = []
        for lo, hi in zip(xs, xs[1:]):
            sl = free.intersection(box(lo, y0 - 1e-6, hi, y1 + 1e-6))
            for g in getattr(sl, 'geoms', [sl]):
                if g.geom_type == 'Polygon' and not g.is_empty:
                    pieces.append(g)
        # ★ 面積守恆：切區絕不可以靜默吃掉面積
        lost = free.area - sum(g.area for g in pieces)
        if abs(lost) > max(free.area * 1e-6, 1e-12):
            if logger:
                logger.error(f"nfz_split_lost_area(area_id={a.get('area_id')}, "
                             f"{lost / free.area * 100:.2f}%) ⇒ 放棄切割，原樣送出")
            out.append(a)
            continue
        for k, g in enumerate(pieces):
            sub = dict(a)
            # ⚠ `Polygon(a['polygon'])` 是拿 (lat, lon) 當 (x, y) 建的，
            #   所以 coords 出來還是 (lat, lon)，**不可以**再交換一次。
            sub['polygon'] = [(float(x), float(y)) for x, y in g.exterior.coords[:-1]]
            # ⚠⚠⚠ 不可以用 "3_1" 這種字串當 id：轉接層會把它 int() 成 31，
            #   於是與使用者真的畫了第 31 塊區、或機隊切區產生的 31 號子區撞號，
            #   而且撞號之後完全靜默。機隊切區早就改成整數 id + `_src_area_id`
            #   了（見 `split_area_for_fleet`），這裡跟進，兩條切區路徑一致。
            sub['area_id'] = next_id
            sub['_src_area_id'] = a.get('_src_area_id', a.get('area_id'))
            next_id += 1
            out.append(sub)
        n_split += 1
        if logger:
            logger.info(f"nfz_split(area_id={a.get('area_id')}) → {len(pieces)} 塊子區"
                        f"（禁航區佔 {(1 - free.area / poly.area) * 100:.1f}%）")
    if n_split and logger:
        logger.warning(f'nfz_area_split({n_split} 塊區被禁航區切開 ⇒ '
                       f'{len(areas)} → {len(out)} 塊)')
    return out


def _min_width_dir(pts_xy):
    """最小寬度方向（rad）。條帶垂直於它 ⇒ 條帶最長、掉頭最少（教科書做法）。"""
    from shapely.geometry import Polygon
    hull = Polygon(pts_xy).convex_hull
    cs = list(hull.exterior.coords)[:-1]
    best, best_w = 0.0, float('inf')
    for i in range(len(cs)):
        ax, ay = cs[i]
        bx, by = cs[(i + 1) % len(cs)]
        th = math.atan2(by - ay, bx - ax)
        # ⚠⚠⚠ 旋轉 −th 之後的 y 座標是 **s·x + c·y**（c,s = cos(−th), sin(−th)）。
        #   舊版寫成 `-s*x + c*y`，因為 s = −sin(th)，等於量了「對 x 軸鏡射後」
        #   的寬度 ⇒ 對 1560×800 的區塊在 23°–67° 與 113°–157° 這兩段方位
        #   （180° 裡的 90°）會選到**長軸**當最小寬度方向。
        #   後果：切出來是 390×800 的短塊而不是 1560×200 的長條，條帶被剁成
        #   1/4 長、掉頭次數翻倍 —— 正是本函式 docstring 說要避免的事。
        #   sin 項在 th=0 消失、cos 項在 th=90 消失,所以**軸對齊的區塊剛好免疫**,
        #   而既有測試用的就是軸對齊矩形 ⇒ 一直沒被抓到。
        #   :573 的旋轉本來就寫對（`sn*x + c*y`），這裡跟它對齊。
        c, s = math.cos(-th), math.sin(-th)
        ys = [(s * x + c * y) for x, y in cs]
        w = max(ys) - min(ys)
        if w < best_w:
            best_w, best = w, th
    return best


def split_area_for_fleet(areas, n_uav: int, *, logger=None, max_pieces: int = 16):
    """把作業區切成夠多的子區，讓整個機隊都分得到工作。

    ⚠⚠⚠ 為什麼需要：**單一大區配混編機隊會產出空計畫**。
    殘帶對拍賣把每個母胞鎖成「構型 A 一台 + 構型 B 一台」，胞太少就餵不飽機隊，
    而且**加機完全沒用**。實測 20 km²、1 Pioneer + 3 HERO：

        單一大區 5.0×4.0 km  → 胞 1  → **0/4 台上工**
        同一塊切成 2 條      → 胞 4  → 2/4
        同一塊切成 3 條      → 胞 7  → 3/4
        同一塊切成 4 條      → 胞 8  → **4/4、未覆蓋 0**

    所以在送進規劃器之前先切。切的方向是**垂直於最小寬度方向**（也就是沿著
    掃描步進的那一軸切），這樣每一塊子區的最小寬度方向與原區相同 ⇒
    條帶方向不變、長度不變，只是被分段。沿另一軸切會把長條帶剁短，掉頭變多。

    每塊區分到的份數依面積加權；總份數 = 機隊台數（實測 k = n_uav 剛好餵飽）。
    ⚠ 不是切越多越好：實測切 6 條反而多出 1 塊未覆蓋，所以有 `max_pieces` 上限。
    """
    if not areas or n_uav <= 1:
        return areas
    try:
        from shapely.geometry import Polygon, box
    except ImportError:                                    # pragma: no cover
        if logger:
            logger.warning('shapely 不在，機隊切區跳過')
        return areas

    polys, lat0 = [], None
    for a in areas:
        pts = list(a['polygon'])
        if len(pts) < 3:
            return areas
        if lat0 is None:
            lat0 = sum(p[0] for p in pts) / len(pts)
        polys.append(pts)
    m_lat, m_lon = 111132.0, 111412.0 * math.cos(math.radians(lat0))
    to_xy = lambda p: ((p[1] - areas[0]['polygon'][0][1]) * m_lon,      # noqa: E731
                       (p[0] - areas[0]['polygon'][0][0]) * m_lat)
    to_ll = lambda q: (areas[0]['polygon'][0][0] + q[1] / m_lat,        # noqa: E731
                       areas[0]['polygon'][0][1] + q[0] / m_lon)

    shapes = [Polygon([to_xy(p) for p in pts]) for pts in polys]
    # ⚠⚠⚠ 不可以靜默用 `buffer(0)` 修好就當沒事。自交多邊形（使用者畫岔了）
    #   修完可能只剩一半：實測蝴蝶結 (0,0)-(1000,1000)-(1000,0)-(0,1000)
    #   兩瓣共 500,000 m²，`buffer(0)` 只留 250,000 m²，而下面那道「面積守恆」
    #   守衛的基準線是**修好之後**的形狀 ⇒ 結構上看不見這 50%，還照樣印成功。
    #   自交是使用者的輸入錯誤，要大聲說，不要幫他掃一半。
    for a, s in zip(areas, shapes):
        if s.is_valid:
            continue
        try:
            from shapely.validation import explain_validity
            why = explain_validity(s)
        except Exception:                                   # noqa: BLE001
            why = 'invalid polygon'
        rep = s.buffer(0)
        raise ValueError(
            f"作業區 area_id={a.get('area_id')} 的多邊形不合法（{why}）："
            f"修補後面積 {rep.area:,.0f} m²、原始 {s.area:,.0f} m²。"
            f"自己畫岔的區塊不會被默默掃一半，請把邊界改成不自交的。")
    total = sum(s.area for s in shapes)
    if total <= 0:
        return areas

    # ── 份數：先按面積四捨五入，再用「最大餘數」把總份數補到 n_uav ────────
    #   ⚠ 只做四捨五入時，3 塊等面積區配 4 台會得到 k=round(1.33)=1 ⇒ **完全不切**，
    #     3 個胞餵 4 台，第 4 台分不到工作。補足之後才是 4 塊。
    quota = [n_uav * s.area / total for s in shapes]
    ks = [max(1, min(max_pieces, int(math.floor(q)))) for q in quota]
    order = sorted(range(len(shapes)), key=lambda i: quota[i] - math.floor(quota[i]),
                   reverse=True)
    gi = 0
    while sum(ks) < min(n_uav, max_pieces * len(shapes)) and order:
        i = order[gi % len(order)]
        if ks[i] < max_pieces:
            ks[i] += 1
        elif all(k >= max_pieces for k in ks):
            break
        gi += 1

    out, n_split = [], 0
    next_id = max((int(a['area_id']) for a in areas
                   if str(a.get('area_id', '')).isdigit()), default=0) + 1
    for a, s, k in zip(areas, shapes, ks):
        if k <= 1 or s.is_empty:
            out.append(a)
            continue
        th = _min_width_dir(list(s.exterior.coords)[:-1])
        c, sn = math.cos(-th), math.sin(-th)
        rot = Polygon([(c * x - sn * y, sn * x + c * y) for x, y in s.exterior.coords[:-1]])
        x0, y0, x1, y1 = rot.bounds
        # 沿掃描步進軸（旋轉後的 y）等份切 ⇒ 條帶方向與長度不變
        edges = [y0 + (y1 - y0) * i / k for i in range(k + 1)]
        ci, si = math.cos(th), math.sin(th)
        pieces = []
        for lo, hi in zip(edges, edges[1:]):
            g = rot.intersection(box(x0 - 1.0, lo, x1 + 1.0, hi))
            for gg in getattr(g, 'geoms', [g]):
                if gg.geom_type == 'Polygon' and gg.area > 1.0:
                    pieces.append(Polygon([(ci * x - si * y, si * x + ci * y)
                                           for x, y in gg.exterior.coords[:-1]]))
        # ★ 面積守恆
        lost = s.area - sum(p.area for p in pieces)
        if not pieces or abs(lost) > max(s.area * 1e-6, 1.0):
            if logger:
                logger.error(f"fleet_split_lost_area(area_id={a.get('area_id')}, "
                             f"{lost:.1f} m²) ⇒ 放棄切割，原樣送出")
            out.append(a)
            continue
        for i, p in enumerate(pieces):
            sub = dict(a)
            sub['polygon'] = [to_ll(q) for q in p.exterior.coords[:-1]]
            # ⚠ 子區要有**自己的**整數 id（規劃器用它當鍵），但下游（GUI 的
            #   allocation、統計面板）要看到的是**使用者畫的那塊區**。所以把
            #   來源記在 `_src_area_id`，HV 分支收到結果後照它把鍵換回去。
            #   用整數而不是 "1_1" 這種字串：轉接層會把底線吃掉變成 11，
            #   之後就對不回來了。
            sub['area_id'] = next_id
            sub['_src_area_id'] = a.get('area_id')
            next_id += 1
            out.append(sub)
        n_split += 1
        if logger:
            logger.info(f"fleet_split(area_id={a.get('area_id')}) → {len(pieces)} 條子區"
                        f"（機隊 {n_uav} 台）")
    if n_split and logger:
        logger.warning(f'fleet_area_split({len(areas)} → {len(out)} 塊；'
                       f'單一大區配混編機隊會產出空計畫，所以先切）')
    return out


def _uniform_camera(*, hfov_deg: float, vfov_deg: float):
    """兩個構型共用的單光相機（正下方視角）。

    由面板的 HFOV／VFOV 反推針孔相機幾何，使刈幅與面板算出來的足跡一致：
    ``swath = 2·h·tan(HFOV/2) = h·sensor_width/focal`` ⇒ ``sensor_width = 2·f·tan(HFOV/2)``。
    焦距與像素間距沿用 :class:`hv_dccpp.platform.CameraHardware` 的預設值（GSD 由它們決定）。
    HFOV／VFOV 沒給時退回該預設相機。

    ⚠ ``is_default_example=False``：這是使用者給的規格，不該被標成「範例硬體」。
    """
    import math as _m
    from hv_dccpp.platform import CameraHardware
    _f = 16.0                       # CameraHardware.focal_length_mm 的預設值
    if hfov_deg > 0.0 and vfov_deg > 0.0:
        return CameraHardware(focal_length_mm=_f,
                              sensor_width_mm=2.0 * _f * _m.tan(_m.radians(hfov_deg) / 2.0),
                              sensor_height_mm=2.0 * _f * _m.tan(_m.radians(vfov_deg) / 2.0),
                              is_default_example=False)
    return CameraHardware(is_default_example=False)


# ==========================================
# 群飛協調器
# ==========================================
class SwarmCoordinator:
    """
    群飛協調器
    
    提供高階 API 來協調多無人機任務
    """
    
    def __init__(self):
        """初始化協調器"""
        self.current_swarm: Optional[SwarmMission] = None
        self._active_drones: List[DroneInfo] = []   # 最近一次分配使用的無人機列表

        # ── HV-DCCPP（異構雙構型 VTOL）分支旗標 ────────────────────────
        #   預設全部關閉 ⇒ plan_coverage_dccpp 的行為與既有 v2/legacy 路徑
        #   逐位元相同，既有 golden 測試不受影響。呼叫端要用時自行打開。
        #   接法見 docs/AEROPLAN_接入手冊.md §5、docs/adr/0007-hv-dccpp-planner.md。
        self.use_hv_dccpp: bool = False
        self.hv_class_of = None          # Callable[[dict], 'A'|'B']；★ 不給整隊會被當成構型 A
        self.hv_params: Dict[str, Any] = {}
        self.hv_home = None              # (lat, lon)
        self.hv_alternates = None        # [(lat, lon), ...] 備降點
        self.hv_wind = None              # (speed_mps, dir_from_deg)
        self.hv_fence_zones = None       # [FenceZone, ...]；GUI 端先取好再傳進來（見下）
        self.hv_progress = None          # Callable[[float, str], None]
        self.hv_on_report = None         # Callable[[dict], None]
        self.hv_coordinator = None       # 規劃後填入 HvDccppCoordinator，供匯出／上傳／重規劃取用
        self.hv_validation = None        # 規劃後填入 hv_dccpp.validate() 的結果（轉接層自己不驗）
        # ── 單光相機（uniform sensor）─────────────────────────────────
        #   True（預設）：兩個構型掛**同一台**單光相機 ⇒ 酬載不再是分工依據，
        #   分工完全由平台性能（續航、速度、轉彎半徑、作業高度下限）決定。
        #   False：回到轉接層預設，構型 A 被當成 LiDAR、B 被當成相機。
        #   ⚠ 關掉它之前先看 docs/AEROPLAN_接入手冊.md §10.5：某構型找不到
        #     對應 kind 的 PayloadRequirement 時**不會報錯**，只會整型分不到工作。
        self.hv_uniform_sensor: bool = True
        # 禁航區切區（BCD 前處理）。預設開；關掉會退回「整塊區判成不可覆蓋」的行為。
        self.hv_split_areas_around_nfz: bool = True
        # 機隊切區：單一大區自動切成夠多的子區。⚠ 關掉會退回「單一大區 +
        # 混編機隊 = 空計畫」的行為。見 split_area_for_fleet。
        self.hv_split_area_for_fleet: bool = True
    
    def create_swarm_from_survey(self, survey_mission: SurveyMission,
                                num_drones: int = None) -> SwarmMission:
        """
        從 Survey 任務創建群飛任務
        
        參數:
            survey_mission: Survey 任務實例
            num_drones: 無人機數量（預設使用子區域數量）
        
        返回:
            SwarmMission 實例
        """
        # 確保已生成子區域
        if not survey_mission.sub_regions:
            survey_mission.generate_sub_regions()
        
        # 確定無人機數量
        if num_drones is None:
            num_drones = len(survey_mission.sub_regions)
        
        # 創建群飛任務
        swarm = SwarmMission(f"{survey_mission.name} - Swarm")
        
        # 添加無人機
        for i in range(num_drones):
            swarm.add_drone(i + 1, f"Drone_{i + 1}")
        
        # 為每個子區域創建獨立任務
        for idx, region_corners in enumerate(survey_mission.sub_regions):
            if idx >= num_drones:
                break
            
            # 創建子任務
            sub_mission = SurveyMission(f"{survey_mission.name}_Region_{idx + 1}")
            
            # 複製參數
            sub_mission.params = survey_mission.params.copy()
            sub_mission.survey_params = survey_mission.survey_params.copy()
            
            # 設定子區域
            sub_mission.set_survey_area(region_corners)
            sub_mission.survey_params['subdivisions'] = 1  # 單一區域
            
            # 生成航點
            sub_mission.generate_survey_waypoints()
            
            # 分配給無人機
            swarm.assign_mission(idx + 1, sub_mission)
        
        # 應用避撞策略
        swarm.apply_collision_avoidance()
        
        # 計算統計
        swarm.calculate_statistics()
        
        self.current_swarm = swarm
        return swarm
    
    def create_swarm_from_missions(self, missions: List[Mission],
                                  strategy: str = 'sequential') -> SwarmMission:
        """
        從多個任務創建群飛任務
        
        參數:
            missions: 任務列表
            strategy: 群飛策略
        
        返回:
            SwarmMission 實例
        """
        swarm = SwarmMission("Custom Swarm")
        swarm.set_strategy(strategy)
        
        # 添加無人機並分配任務
        for idx, mission in enumerate(missions):
            drone = swarm.add_drone(idx + 1, f"Drone_{idx + 1}")
            swarm.assign_mission(drone.drone_id, mission)
        
        # 應用避撞策略
        swarm.apply_collision_avoidance()
        
        # 計算統計
        swarm.calculate_statistics()
        
        self.current_swarm = swarm
        return swarm
    
    def export_swarm_missions(self, swarm: SwarmMission,
                            output_dir: str) -> List[str]:
        """
        匯出群飛任務檔案
        
        參數:
            swarm: 群飛任務
            output_dir: 輸出目錄
        
        返回:
            已匯出的檔案路徑列表
        """
        import os
        
        os.makedirs(output_dir, exist_ok=True)
        exported_files = []
        
        for drone in swarm.drones:
            if drone.mission:
                # 匯出航點檔案
                filename = f"drone_{drone.drone_id}_{drone.name}.waypoints"
                filepath = os.path.join(output_dir, filename)
                
                lines = drone.mission.waypoints.to_qgc_format()
                with open(filepath, 'w', encoding='utf-8') as f:
                    f.write('\n'.join(lines))
                
                exported_files.append(filepath)
        
        # 生成任務簡報
        briefing_file = os.path.join(output_dir, "swarm_briefing.txt")
        with open(briefing_file, 'w', encoding='utf-8') as f:
            f.write(swarm.generate_mission_briefing())
        
        exported_files.append(briefing_file)
        
        print(f"已匯出 {len(exported_files)} 個檔案到: {output_dir}")
        return exported_files
    
    def get_current_swarm(self) -> Optional[SwarmMission]:
        """獲取當前群飛任務"""
        return self.current_swarm

    # ==========================================
    # Task 4：基於成本函數的智能多機分配
    # ==========================================
    @staticmethod
    def _haversine_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """
        計算兩個經緯度點間的 Haversine 距離（公尺）。

        參數:
            p1, p2: (lat, lon) 格式
        """
        R = EARTH_RADIUS_M
        lat1, lon1 = math.radians(p1[0]), math.radians(p1[1])
        lat2, lon2 = math.radians(p2[0]), math.radians(p2[1])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        return R * 2.0 * math.asin(math.sqrt(a))

    def _min_entry_distance(self, area: CoveragePath, drone: DroneInfo) -> float:
        """計算無人機到區域最近作業路徑端點的 haversine 距離（公尺）"""
        if not area.operations or drone.position is None:
            return 0.0
        best = float('inf')
        for op in area.operations:
            d_l = self._haversine_distance(drone.position, op.left_point)
            d_r = self._haversine_distance(drone.position, op.right_point)
            best = min(best, d_l, d_r)
        return best

    def allocate_by_cost(
        self,
        areas: List[CoveragePath],
        drones: List[DroneInfo],
        weights: Tuple[float, float, float, float] = (0.25, 0.25, 0.25, 0.25),
    ) -> Dict[int, List[int]]:
        """
        基於加權成本函數的智能多機區域分配（論文 Eq.30 + Algorithm 1）。

        成本函數 Jk = w1×C_dist + w2×C_count + w3×C_length + w4×C_priority
        各項成本先做全區域歸一化，再計算加權和。
        根據 Jk 比例用最大餘額法決定每個區域分配的無人機數量，
        再以貪婪策略從剩餘可用無人機中選取進入距離最短的若干架。

        參數:
            areas:   結構化覆蓋路徑列表
            drones:  無人機列表（需有 position 資訊）
            weights: (w1, w2, w3, w4)
                     w1=進入距離成本  w2=路徑數成本
                     w3=路徑長度成本  w4=優先級成本

        返回:
            Dict[area_id, List[drone_id]]
        """
        if not areas or not drones:
            return {a.area_id: [] for a in areas}

        w1, w2, w3, w4 = weights
        n_areas = len(areas)
        n_drones = len(drones)

        # ── 計算各項原始成本 ────────────────────────────────────────
        # C_dist: 所有無人機到該區域的最小距離均值（越遠越難分配）
        c_dist = []
        for area in areas:
            dists = [
                self._min_entry_distance(area, d)
                for d in drones if d.position is not None
            ]
            c_dist.append(sum(dists) / len(dists) if dists else 0.0)

        # C_count: 作業路徑數
        c_count = [float(len(a.operations)) for a in areas]

        # C_length: 作業路徑總長度
        c_length = [a.total_length() for a in areas]

        # C_priority: 區域優先級
        c_priority = [a.priority for a in areas]

        # ── 歸一化（除以總和，避免除零）────────────────────────────
        def _normalize(vals: List[float]) -> List[float]:
            s = sum(vals)
            if s < 1e-9:
                return [1.0 / n_areas] * n_areas
            return [v / s for v in vals]

        c_dist_n    = _normalize(c_dist)
        c_count_n   = _normalize(c_count)
        c_length_n  = _normalize(c_length)
        c_priority_n = _normalize(c_priority)

        # ── 加權成本 Jk ──────────────────────────────────────────────
        J = [
            w1 * c_dist_n[i] + w2 * c_count_n[i] +
            w3 * c_length_n[i] + w4 * c_priority_n[i]
            for i in range(n_areas)
        ]
        sum_J = sum(J) or 1.0

        # ── 最大餘額法分配無人機數量 ──────────────────────────────────
        exact = [n_drones * j / sum_J for j in J]
        floor_ell = [int(e) for e in exact]
        remainder = [e - f for e, f in zip(exact, floor_ell)]

        total_floor = sum(floor_ell)
        extra = n_drones - total_floor
        # 餘額最大的區域各補 1
        sorted_by_remainder = sorted(range(n_areas), key=lambda i: remainder[i], reverse=True)
        ell = list(floor_ell)
        for i in range(extra):
            ell[sorted_by_remainder[i]] += 1

        # ── 貪婪分配（按 Jk 從大到小，每次選最近的 ℓk 架無人機）────
        allocation: Dict[int, List[int]] = {a.area_id: [] for a in areas}
        available = list(drones)

        area_order = sorted(range(n_areas), key=lambda i: J[i], reverse=True)

        for area_idx in area_order:
            area = areas[area_idx]
            k = min(ell[area_idx], len(available))
            if k <= 0:
                continue

            # 蒐集端點
            endpoints = []
            for op in area.operations:
                endpoints.extend([op.left_point, op.right_point])

            # 對可用無人機按距離排序
            def _drone_min_dist(drone: DroneInfo) -> float:
                if drone.position is None or not endpoints:
                    return float('inf')
                return min(self._haversine_distance(drone.position, ep) for ep in endpoints)

            available.sort(key=_drone_min_dist)

            for i in range(k):
                drone = available[i]
                allocation[area.area_id].append(drone.drone_id)

            available = available[k:]  # 移除已分配的前 k 架

        # 若有剩餘無人機，補充給需求最大的區域
        if available:
            areas_by_need = sorted(areas, key=lambda a: len(a.operations), reverse=True)
            for i, drone in enumerate(available):
                area = areas_by_need[i % len(areas_by_need)]
                allocation[area.area_id].append(drone.drone_id)

        # 儲存本次分配使用的無人機列表
        self._active_drones = list(drones)

        return allocation

    # ==========================================
    # Task 5：動態重規劃框架
    # ==========================================
    def handle_drone_dropout(
        self,
        dropout_drone_id: int,
        current_coverage_paths: List[CoveragePath],
    ) -> Dict[int, List[int]]:
        """
        處理無人機故障退出事件。

        流程：
        1. 找出故障無人機負責但未完成的 OperationSegment
        2. 將這些未完成段標記為未分配（assigned_drone_id = None）
        3. 從剩餘在線無人機中，用 allocate_by_cost() 重新分配

        參數:
            dropout_drone_id:       故障無人機 ID
            current_coverage_paths: 當前所有區域的覆蓋路徑狀態

        返回:
            更新後的分配 Dict[area_id, List[drone_id]]
        """
        # 步驟 1：找出並解除故障無人機的未完成任務
        affected_count = 0
        for cp in current_coverage_paths:
            removed = cp.unassign_drone(dropout_drone_id)
            affected_count += removed

        # 步驟 2：取得剩餘在線無人機
        remaining_drones = [
            d for d in self._active_drones
            if d.drone_id != dropout_drone_id
        ]
        if not remaining_drones:
            return {cp.area_id: [] for cp in current_coverage_paths}

        # 步驟 3：只對仍有未完成路徑的區域重新分配
        areas_with_work = [
            cp for cp in current_coverage_paths
            if cp.get_uncovered_operations()
        ]
        if not areas_with_work:
            return {cp.area_id: [] for cp in current_coverage_paths}

        new_allocation = self.allocate_by_cost(areas_with_work, remaining_drones)

        # 補全沒有工作的區域（返回空列表）
        full_allocation: Dict[int, List[int]] = {cp.area_id: [] for cp in current_coverage_paths}
        full_allocation.update(new_allocation)

        # 將分配結果寫回 OperationSegment
        for cp in areas_with_work:
            drone_ids = new_allocation.get(cp.area_id, [])
            unassigned_ops = cp.get_unassigned_operations()
            for i, op in enumerate(unassigned_ops):
                if drone_ids:
                    op.assigned_drone_id = drone_ids[i % len(drone_ids)]

        return full_allocation

    def handle_new_area(
        self,
        new_area_polygon: List[Tuple[float, float]],
        new_area_id: int,
        scan_angle_deg: float,
        coverage_width_m: float,
        current_coverage_paths: List[CoveragePath],
        available_drones: List[DroneInfo],
    ) -> CoveragePath:
        """
        處理新區域出現事件。

        流程：
        1. 用 RegionDivider.decompose_by_fov() 分解新區域為作業路徑
        2. 建立新的 CoveragePath
        3. 用 allocate_by_cost() 從可用無人機中分配
        4. 將分配結果寫回 OperationSegment

        參數:
            new_area_polygon:       新區域邊界 [(lat,lon),...]
            new_area_id:            新區域 ID
            scan_angle_deg:         掃描方向（度）
            coverage_width_m:       FOV 覆蓋寬度（公尺）
            current_coverage_paths: 現有覆蓋路徑（供計算分配成本時參考）
            available_drones:       可用無人機列表

        返回:
            新區域的 CoveragePath（已完成無人機分配）
        """
        from core.geometry.region_divider import RegionDivider

        # 步驟 1：分解為作業路徑
        fov_paths = RegionDivider.decompose_by_fov(
            new_area_polygon, scan_angle_deg, coverage_width_m
        )

        # 步驟 2：建立 CoveragePath
        new_cp = CoveragePath(
            area_id=new_area_id,
            area_polygon=new_area_polygon,
            scan_angle_deg=scan_angle_deg,
        )
        for fp in fov_paths:
            new_cp.operations.append(OperationSegment(
                index=fp['index'],
                left_point=fp['left_point'],
                right_point=fp['right_point'],
                length_m=fp['length_m'],
            ))

        # 步驟 3：只用新區域進行分配（節省計算量）
        allocation = self.allocate_by_cost([new_cp], available_drones)
        drone_ids = allocation.get(new_area_id, [])

        # 步驟 4：寫回分配結果（輪替分配）
        if drone_ids:
            for i, op in enumerate(new_cp.operations):
                op.assigned_drone_id = drone_ids[i % len(drone_ids)]

        return new_cp

    # ==========================================
    # Task 7：整合入口 — 一鍵最佳化群飛覆蓋任務
    # ==========================================
    def create_optimized_swarm_coverage(
        self,
        areas: List[Dict[str, Any]],
        drones: List[Dict[str, Any]],
        coverage_width_m: float,
        overlap_rate: float = 0.1,
        auto_scan_angle: bool = True,
        weights: Tuple[float, float, float, float] = (0.25, 0.25, 0.25, 0.25),
    ) -> SwarmMission:
        """
        一鍵建立最佳化群飛覆蓋任務（整合任務 1–6 全部改進）。

        流程：
        1. 對每個區域：
           a. 若 auto_scan_angle=True，用 PolygonUtils.optimal_scan_angle() 計算最佳掃描方向
           b. 用 RegionDivider.decompose_by_fov() 分解為作業路徑
           c. 建立 CoveragePath
        2. 用 allocate_by_cost() 智能分配無人機到各區域
        3. 在各區域內按無人機數量輪替分配 OperationSegment
        4. 對每架無人機，用 SequenceOptimizer.optimize_sequence() 最佳化順序
        5. 生成 SwarmMission 並計算統計

        參數:
            areas:            區域定義列表，每個元素：
                              {'polygon': [(lat,lon),...],
                               'priority': float, 'area_id': int}
            drones:           無人機定義列表，每個元素：
                              {'drone_id': int, 'name': str,
                               'position': (lat,lon), 'speed': float}
            coverage_width_m: FOV 覆蓋寬度（公尺）
            overlap_rate:     重疊率（0–1）
            auto_scan_angle:  是否自動計算最佳掃描方向
            weights:          allocate_by_cost 的 (w1,w2,w3,w4) 權重

        返回:
            SwarmMission 實例（含完整路徑與統計）
        """
        from core.geometry.region_divider import RegionDivider
        from core.geometry.polygon import PolygonUtils
        from core.global_planner.sequence_optimizer import SequenceOptimizer

        # ── 步驟 1：建立 DroneInfo 物件 ──────────────────────────────
        drone_infos: List[DroneInfo] = []
        for d in drones:
            drone_infos.append(DroneInfo(
                drone_id=d['drone_id'],
                name=d.get('name', f"Drone_{d['drone_id']}"),
                position=d.get('position'),
                speed_mps=d.get('speed', 10.0),
            ))

        # ── 步驟 2：為每個區域建立 CoveragePath ──────────────────────
        coverage_paths: List[CoveragePath] = []
        for area_dict in areas:
            polygon  = area_dict['polygon']
            area_id  = area_dict.get('area_id', id(area_dict))
            priority = area_dict.get('priority', 1.0)

            if auto_scan_angle:
                scan_angle = PolygonUtils.optimal_scan_angle(polygon)
            else:
                scan_angle = area_dict.get('scan_angle_deg', 0.0)

            fov_paths = RegionDivider.decompose_by_fov(
                polygon, scan_angle, coverage_width_m, overlap_rate
            )

            cp = CoveragePath(
                area_id=area_id,
                area_polygon=polygon,
                scan_angle_deg=scan_angle,
                priority=priority,
            )
            for fp in fov_paths:
                cp.operations.append(OperationSegment(
                    index=fp['index'],
                    left_point=fp['left_point'],
                    right_point=fp['right_point'],
                    length_m=fp['length_m'],
                ))
            coverage_paths.append(cp)

        # ── 步驟 3：智能分配無人機到各區域 ──────────────────────────
        allocation = self.allocate_by_cost(coverage_paths, drone_infos, weights)

        # ── 步驟 4：區域內輪替分配 OperationSegment ──────────────────
        for cp in coverage_paths:
            area_drone_ids = allocation.get(cp.area_id, [])
            if not area_drone_ids:
                continue
            for i, op in enumerate(cp.operations):
                op.assigned_drone_id = area_drone_ids[i % len(area_drone_ids)]

        # ── 步驟 5：對每架無人機最佳化其作業路徑順序 ────────────────
        drone_optimized_ops: Dict[int, List[OperationSegment]] = {}
        for drone_info in drone_infos:
            all_ops: List[OperationSegment] = []
            for cp in coverage_paths:
                all_ops.extend(cp.get_operations_by_drone(drone_info.drone_id))

            if not all_ops:
                continue

            start_pos = drone_info.position or coverage_paths[0].area_polygon[0]
            drone_optimized_ops[drone_info.drone_id] = SequenceOptimizer.optimize_sequence(
                start_pos, all_ops
            )

        # ── 步驟 6：組裝 SwarmMission ─────────────────────────────────
        swarm = SwarmMission("最佳化群飛覆蓋任務")
        swarm.drones = list(drone_infos)
        swarm.stats['total_drones'] = len(drone_infos)
        swarm.stats['total_waypoints'] = sum(
            len(cp.operations) * 2 for cp in coverage_paths
        )

        total_op_dist = sum(cp.total_length() for cp in coverage_paths)
        total_transit_dist = 0.0
        for drone_info in drone_infos:
            ops = drone_optimized_ops.get(drone_info.drone_id, [])
            if ops:
                start = drone_info.position or coverage_paths[0].area_polygon[0]
                transit = SequenceOptimizer(
                ).calculate_total_path_length(start, ops) - sum(o.length_m for o in ops)
                total_transit_dist += max(0.0, transit)

        swarm.stats['total_distance'] = total_op_dist + total_transit_dist

        avg_speed = (sum(d.speed_mps for d in drone_infos) / len(drone_infos)
                     if drone_infos else 10.0)
        swarm.stats['estimated_time'] = (
            swarm.stats['total_distance'] / avg_speed if avg_speed > 0 else 0.0
        )

        # 附加 coverage_paths 到 swarm 方便後續使用
        swarm.coverage_paths = coverage_paths  # type: ignore[attr-defined]
        swarm.allocation = allocation           # type: ignore[attr-defined]

        # ── 步驟 7：將 CoveragePath 轉為各無人機的 WaypointSequence ──
        default_altitude = 50.0  # 預設飛行高度（公尺）
        for drone_info in drone_infos:
            ops = drone_optimized_ops.get(drone_info.drone_id, [])
            if not ops:
                continue
            if drone_info.mission is None:
                drone_info.mission = Mission(f"Drone_{drone_info.drone_id}_Coverage")
            # 建立臨時 CoveragePath 以使用 to_waypoint_sequence()
            temp_cp = CoveragePath(
                area_id=drone_info.drone_id,
                area_polygon=[],
                scan_angle_deg=0.0,
                operations=list(ops),
            )
            drone_info.mission.waypoints = temp_cp.to_waypoint_sequence(
                altitude=default_altitude,
                speed=drone_info.speed_mps,
            )

        self.current_swarm = swarm
        return swarm

    # ==========================================
    # DCCPP 論文整合：MDTSP + IDP + GDA
    # ==========================================
    def plan_coverage_dccpp(
        self,
        areas: List[Dict[str, Any]],
        drones: List[Dict[str, Any]],
        coverage_width_m: float,
        overlap_rate: float = 0.1,
        auto_scan_angle: bool = True,
        coverage_altitude: float = 100.0,
        terrain_func=None,
        enable_altitude: bool = True,
        mounting_angle_deg: float = 0.0,
        dem_path: str = "",
        coordination_mode: str = "uncoordinated",
        runway_bearing_deg: float | None = None,
        runway_length_m: float | None = None,
        landing_bearing_deg: float | None = None,
        landing_rollout_m: float = 0.0,
        vtol_transition_alt_m: float | None = None,
    ) -> Dict[str, Any]:
        """
        使用論文 DCCPP 完整演算法進行多區域多 UAV 覆蓋路徑規劃。

        整合流程：
        1. 建立 CoveragePath（FOV 分解 + 最佳掃描角）
        2. GreedyAllocator 分配 UAV 至各區域（Algorithm 1）
        3. IDPSolver 求解各區域內作業路徑序列（Algorithm 2）
        4. DCCPPPathAssembler 組裝含 Dubins 曲線的完整航點路徑
        5. AltitudePlanner 進行 2D→3D 高度規劃 + GDA 平滑（Algorithm 3）
        6. 可選：CoordinatedPlanner 同步各 UAV 進入時間

        同時支援多旋翼（turn_radius=0）與固定翼（turn_radius>0）。

        參數:
            areas:              區域定義列表
            drones:             無人機定義列表
            coverage_width_m:   FOV 覆蓋寬度（公尺）
            overlap_rate:       重疊率（0–1）
            auto_scan_angle:    是否自動計算最佳掃描方向
            coverage_altitude:  覆蓋高度（公尺）
            terrain_func:       地形查詢函數 (lat, lon) → 海拔
            enable_altitude:    是否啟用高度規劃
            mounting_angle_deg: 感測器掛載角（度），>0 時啟用梯形 FOV 模型
            dem_path:           DEM 檔案路徑，非空時載入 DEM 進行地形查詢
            coordination_mode:  "uncoordinated" 或 "coordinated"
            vtol_transition_alt_m:
                                VTOL 的 MC↔FW 轉換高度（m AGL）。
                                ⚠ 只有 HV-DCCPP 分支（self.use_hv_dccpp=True）會真的用它產生
                                垂直起降剖面；既有 v2 / legacy 路徑目前**沒有**垂直剖面能力
                                （DCCPPPathBuilder 缺 prepend_vtol_takeoff/append_vtol_landing），
                                此時只記一筆 warning 後照跑道剖面規劃，不會靜默吞掉。

        返回:
            Dict 包含：
            - 'results'         : {area_id: MDTSPResult}
            - 'allocation'      : {area_id: [uav_id, ...]}
            - 'coverage_paths'  : List[CoveragePath]
            - 'uav_states'      : List[UAVState]
            - 'total_makespan'  : float（秒）
            - 'assembled_paths' : {uav_id: AssembledPath} 或 None
        """
        from core.geometry.region_divider import RegionDivider
        from core.geometry.polygon import PolygonUtils
        from core.global_planner.mdtsp_solver import (
            UAVState, VehicleType, DCCPPSolver,
            GreedyAllocator, IDPSolver, AltitudePlanner,
            CoordinatedPlanner,
        )
        # 優先使用 core/dccpp/ 的 numpy 向量化實作
        try:
            from core.dccpp.task_allocator import UAVTaskAllocator
            from core.dccpp.idp_solver import IDP_Solver
            from core.dccpp.altitude_planner import AltitudePlanner as AltPlannerNew
            _use_new_dccpp = True
        except ImportError:
            _use_new_dccpp = False

        # ── 梯形 FOV 模型（可選）────────────────────────
        actual_width = coverage_width_m
        if mounting_angle_deg > 0:
            try:
                from sensors.fov_model import TrapezoidalFOV
                fov_model = TrapezoidalFOV(
                    mounting_angle_deg=mounting_angle_deg,
                    vertical_half_angle_deg=22.5,
                    horizontal_half_angle_deg=38.5,
                )
                actual_width = fov_model.coverage_width(coverage_altitude)
                import logging as _log
                _log.getLogger(__name__).info(
                    f"梯形 FOV: mounting={mounting_angle_deg}°, "
                    f"coverage_width={actual_width:.1f}m (原始 {coverage_width_m:.1f}m)"
                )
            except Exception as e:
                import logging as _log
                _log.getLogger(__name__).warning(f"FOV 模型載入失敗: {e}")

        # ── DEM 地形載入（可選）────────────────────────
        if dem_path and terrain_func is None:
            try:
                from sensors.dem_terrain import DEMTerrainManager
                dem_mgr = DEMTerrainManager(default_elevation=0.0)
                if dem_mgr.load_dem(dem_path):
                    terrain_func = dem_mgr.get_elevation
                    import logging as _log
                    _log.getLogger(__name__).info(f"DEM 載入成功: {dem_path}")
            except Exception as e:
                import logging as _log
                _log.getLogger(__name__).warning(f"DEM 載入失敗: {e}")

        # ══ HV-DCCPP 分支（異構雙構型 VTOL）══════════════════════════════
        #   接法與理由見 docs/AEROPLAN_接入手冊.md §5、docs/adr/0007-hv-dccpp-planner.md。
        #   插在這裡的理由：terrain_func 已由上面的 DEM 區塊解析完成，而 UAVState /
        #   CoveragePath 都還沒建立 ⇒ 提早 return 不會浪費任何工作。
        #   回傳的 6 鍵字典與下方 v2 路徑同形 ⇒ 地圖、統計、匯出、SITL 上傳全部不用改。
        if self.use_hv_dccpp:
            try:
                from hv_dccpp.integration.aeroplan_adapter import (
                    HvDccppCoordinator, to_planner_config,
                )
                # ★ AeroPlan 自己有兩個 PlannerConfig ⇒ hv 的一律用別名（手冊 §17.2）
                from hv_dccpp.mission import PlannerConfig as HvPlannerConfig
            except ImportError as _e:
                import logging as _log
                _log.getLogger(__name__).warning(
                    f"hv_dccpp 未安裝或匯入失敗，降級為既有 DCCPP：{_e}")
            else:
                import logging as _log
                _p = dict(self.hv_params or {})
                # ⚠⚠ 規劃**前**發出的 note 一律先收進這裡再一次送。
                #   `hv_on_report` 在很多消費端是 dict.update，一則一則送的話
                #   後送的會蓋掉先送的 —— 實測 dh_band_min_relaxed 把
                #   min_fw_lowered 整則吃掉。尾端的 `_vnotes` 也會併進來。
                _prenotes: list[str] = []

                def _hv_get(*names, default=None):
                    """依序取第一個「有給且非 None」的鍵；面板鍵一律帶 hv_ 前綴。"""
                    for _n in names:
                        if _p.get(_n) is not None:
                            return _p[_n]
                    return default

                # 禁飛區：NFZ + 威脅區（GEOFENCE 是 inclusion，語意相反，不能送）
                # ★ 優先用呼叫端先取好的那份：`FenceZoneRegistry` 是主執行緒的 QObject 單例，
                #   規劃跑在背景執行緒時不該由這裡去讀它（GUI 端在 _configure_hv_dccpp 已取好）。
                _zones = list(self.hv_fence_zones) if self.hv_fence_zones is not None else None
                if _zones is None:
                    try:
                        from mission.fence_zone import FenceZoneRegistry, ZoneCategory
                        _reg = FenceZoneRegistry.instance()
                        _zones = (_reg.by_category(ZoneCategory.NFZ)
                                  + _reg.by_category(ZoneCategory.THREAT))
                    except Exception as _e:
                        # 只有「PyQt6 不在（headless）／registry 還沒建立」才會走到這裡。
                        # 用 warning 而不是 debug：靜悄悄地把所有禁飛區丟掉是不能接受的。
                        _log.getLogger(__name__).warning(
                            f"HV-DCCPP: 取不到 fence zone，本次規劃**不含任何禁飛區**：{_e}")
                if _zones:
                    _log.getLogger(__name__).info(f"HV-DCCPP: 帶入 {len(_zones)} 個禁飛／威脅區")

                # 單光相機：兩個構型掛同一台，kind 一律 'rgb'。
                # 轉接層 `_hardware_for` 是**照構型字母**決定 kind（A→lidar、B→rgb），
                # 所以不明確給 sensor_a/sensor_b 就會憑空生出一個酬載差異。
                # ── 機隊切區：單一大區 → 夠多的子區 ─────────────────────
                #   ⚠⚠⚠ 單一大區配混編機隊會產出**空計畫**（殘帶對拍賣把母胞鎖成
                #   「A 一台 + B 一台」，胞太少餵不飽機隊，加機也沒用）。
                #   實測 20 km²、1 Pioneer + 3 HERO：整塊 0/4 台，切 4 條 4/4 台。
                #   詳見 split_area_for_fleet。
                _src_of = {}
                if bool(_hv_get('hv_split_area_for_fleet',
                                default=self.hv_split_area_for_fleet)):
                    areas = split_area_for_fleet(
                        areas, len(drones or []), logger=_log.getLogger(__name__))
                    _src_of = {a['area_id']: a['_src_area_id'] for a in areas
                               if a.get('_src_area_id') is not None}

                # ── 禁航區切區（BCD 前處理）───────────────────────────
                #   把與禁航區重疊的作業區切成繞過它的子區，再送進規劃器。
                #   不做的話：禁航區蓋住一塊區 ≥42.9% ⇒ **整趟任務 0 台上工**
                #   （連不相干的區也一起被丟）。詳見 split_areas_around_nfz。
                if bool(_hv_get('hv_split_areas_around_nfz',
                                default=self.hv_split_areas_around_nfz)) and _zones:
                    _sw = (2.0 * float(coverage_altitude)
                           * math.tan(math.radians(
                               float(_hv_get('hfov_deg', default=0.0) or 0.0)) / 2.0))
                    areas = split_areas_around_nfz(
                        areas, _zones, inset_m=(_sw / 2.0 if _sw > 0 else 0.0),
                        logger=_log.getLogger(__name__))

                _sensor_pair = None
                if bool(_hv_get('hv_uniform_sensor', default=self.hv_uniform_sensor)):
                    # ⚠⚠ 作業區可以自己宣告 sensor_req。單光相機之下沒有人掛 LiDAR，
                    #   那種區會變成 no_feasible_class 被**整塊丟掉**，而 status 仍是
                    #   GO_WITH_WARNINGS、errors 是空的、raise_on_no_go 也不會 raise
                    #   —— 只在 plan.uncovered 與一則 note 裡。實測：同一份幾何
                    #   異構酬載 4 架 9 條帶 0 未覆蓋，單光相機變成 1 架 4 條帶 1 未覆蓋。
                    _bad = [a.get('area_id', i) for i, a in enumerate(areas or [])
                            if str(a.get('sensor_req') or 'any').lower()
                            not in ('any', 'rgb', '')]
                    if _bad:
                        _log.getLogger(__name__).warning(
                            f"HV-DCCPP: 單光相機模式下，作業區 {_bad} 宣告了非 rgb 的 "
                            f"sensor_req ⇒ 這些區會整塊變成未覆蓋。"
                            f"把 sensor_req 拿掉（或設 'rgb'／'any'），"
                            f"或關掉 hv_uniform_sensor。")
                    _sensor_pair = (_uniform_camera(
                        hfov_deg=float(_hv_get('hfov_deg', default=0.0) or 0.0),
                        vfov_deg=float(_hv_get('vfov_deg', default=0.0) or 0.0)),)
                    _log.getLogger(__name__).info(
                        f"HV-DCCPP: 兩構型共用單光相機 {_sensor_pair[0]!r}")
                    if abs(float(mounting_angle_deg)) > 1e-6:
                        # ⚠ 兩邊的 mount 角**基準不同**：面板是「離天底的傾角」（0=正下方），
                        #   hv 的 TrapezoidFovSpec 是「離水平的俯角」（90=正下方）。
                        #   直接把面板值送過去會得到錯的足跡，所以這裡不模型化傾角，只記一筆。
                        _log.getLogger(__name__).warning(
                            f"HV-DCCPP: 單光相機以正下方（nadir）建模，"
                            f"面板的 mounting_angle_deg={mounting_angle_deg} 未計入足跡")

                # ── 面板把重疊吃進 fov_width_m ⇒ 要換算回 overlap_rate ────
                #   ⚠⚠⚠ HV 的條帶間距**完全不看** coverage_width_m：實測 width 給
                #   50 / 113.2 / 150 / 300 m，間距一律是相機推導的 161.7 m。
                #   它只吃 overlap_rate（→ PayloadRequirement.side_overlap）。
                #   而「VTOL 偵查」面板送的是 fov_width_m = 刈幅×(1−重疊)、
                #   overlap_rate = 0.0（註解寫「重疊已經吃進 fov_width_m」）
                #   ⇒ **使用者設的重疊在 HV 這條路徑上被整個丟掉，一律以 0% 重疊規劃**。
                #   實測後果（4 塊 520×600 m、1A+3B、面板慣例）：舊 DCCPP 覆蓋 100%，
                #   HV 只有 73.1 / 98.9 / 90.3 / 73.3%，而它自己回報 uncovered=0。
                _ov = float(overlap_rate or 0.0)
                if _ov <= 0.0 and float(coverage_width_m or 0.0) > 0.0:
                    _swath = (2.0 * float(coverage_altitude)
                              * math.tan(math.radians(float(_hv_get('hfov_deg', default=0.0)
                                                            or 0.0)) / 2.0))
                    if _swath > 0.0 and float(coverage_width_m) < _swath:
                        _ov = min(0.95, max(0.0, 1.0 - float(coverage_width_m) / _swath))
                        _log.getLogger(__name__).warning(
                            f"overlap_folded_into_width(面板 fov_width_m="
                            f"{float(coverage_width_m):.1f} m、刈幅 {_swath:.1f} m "
                            f"⇒ 還原 overlap_rate={_ov:.3f})；不還原的話 HV 會以 0% 重疊規劃")
                        _prenotes.append(f'overlap_folded_into_width({_ov:.3f})')

                # ── 用面板的「目標 GSD」而不是自動合成的 1.05×h ──────────
                #   ⚠⚠⚠ 這條與上面的 AGL 上限是**兩件獨立的事**，不要混在一起：
                #     h_ceil  來自空域（豁免／法規），管的是 Δh_band
                #     h_max   來自相機（GSD 反算），管的是 gsd_attainable
                #   `gsd_attainable ⇔ h_max ≥ h_minfw + dh_band_min_m`
                #   （platform.py:739），而 h_max = GSD·f/p。本機相機 f=16 mm、
                #   p=3.9 µm ⇒ h_max ≈ 4102.6 × GSD，構型 A 需 h_max ≥ 140 m
                #   ⇒ **GSD ≥ 0.0342 m/px**。
                #   轉接層不給 payload_reqs 時會自己合成 gsd = 1.05·h·p/f
                #   （aeroplan_adapter.py:1011）⇒ h_max = 1.05×作業高度，
                #   120 m 只有 126 m < 140 ⇒ 構型 A 分不到工作。
                #   面板本來就有「目標 GSD」欄位（預設 0.05 m/px），只是沒被送下來。
                #   實測（2.45 km²、1A+1B、作業高度 120 m、h_ceil 200 m）：
                #     自動合成（h_max 126 m）→ 只有 HERO，makespan 2085 s
                #     GSD 0.05（h_max 205 m）→ **兩型都有**，1284 s
                #     GSD 0.03（h_max 123 m）→ 只有 HERO（反向臂）
                _reqs = None
                _gsd = _hv_get('target_gsd', 'gsd_required_m', default=None)
                try:
                    _gsd = float(_gsd) if _gsd is not None else None
                except (TypeError, ValueError):
                    _gsd = None
                if _gsd and _gsd > 0.0:
                    from hv_dccpp.mission import PayloadRequirement as _PReq
                    _reqs = {'default': _PReq(
                        'default', 'rgb', gsd_required_m=_gsd,
                        side_overlap=(_ov if 0.0 <= _ov < 1.0 else None),
                        forward_overlap=0.70,
                        h_target_agl_m=float(coverage_altitude))}
                    _log.getLogger(__name__).info(
                        f"HV-DCCPP: 採用面板目標 GSD {_gsd:.3f} m/px"
                        f"（不用自動合成的 1.05×作業高度）")

                # ── AGL 上限（h_ceil）與可用高度帶：先算給人看 ──────────
                #   ⚠⚠⚠ 上游 `Airspace.max_agl_m` 寫死 **120**（民航法規值），
                #   `h_ceil = waiver_max_agl_m if given else max_agl_m`。
                #   所以 waiver=None 就是「沒有豁免」＝法規上限 120 m。
                #   可指派條件（decompose.altitude_band）：
                #       Δh_band = h_ceil − max(h_minfw, h_clr) − 地形起伏 ≥ dh_band_min_m
                #   實測（2.45 km²、1A+1B、作業高度 120、GSD 0.05）：
                #       h_ceil 120 → **整趟 NO_GO**（連 HERO 都不行，起伏吃掉那 20 m）
                #       h_ceil 125…140 → 只有 HERO（makespan 2085 s）
                #       h_ceil ≥ 141   → 兩型都有（1424 s；200 m 時 1284 s）
                #   NO_GO 訊息本身只寫「所有區域皆無可行構型」，看不出是高度帶，
                #   所以這裡先把每個構型的算式攤開。
                _wv = _hv_get('hv_waiver_max_agl_m', 'waiver_max_agl_m', default=None)
                _waiver = None if _wv is None else float(_wv)
                _ceil = 120.0 if _waiver is None else _waiver
                from hv_dccpp.platform import (PLATFORM_HERO as _PLAT_B,
                                               PLATFORM_PIONEER as _PLAT_A)

                # ── 最低固定翼作業高度：可以往下調，但要說出代價 ──────────
                #   ⚠⚠⚠ 表 1 的 120（Pioneer）／100（HERO）**不是機體能力極限**：
                #     * 平台參數表對這一列的使用方式欄就寫「任務前需依地形、感測器
                #       與法規調整」，而且它沒有像旁邊的轉換高度那樣標「暫定」——
                #       它是使用者給的規劃值，全 repo 找不到任何原廠或法規出處。
                #     * 規格 D35 明文 `h_minfw 只對 ENTRY/OPER/TRANS/RTB 生效`；
                #       CLIMB/DESCENT 的 z_lo 不含它，ACCEL/DECEL 更是落在
                #       H_tc（A 80–100 m）——**規劃器自己就讓飛機飛在 120 以下**。
                #     * 全 repo 沒有任何一句話說低於它會失速／失去操縱權限。
                #   ⚠ 但它有真實下游：`Q_RTL_ALT`（ardupilot.py:922）會進**匯出的
                #     .param**，每條路徑都生效（HV4SITL8 讀回 Q_RTL_ALT,100 已確認）。
                #     `FENCE_ALT_MIN`（mavlink_link.py:996）也綁著它，但那是
                #     `upload_plan(write_params/set_params)` 才走的路 ——
                #     AeroPlan 的 SITL 流程只上傳任務、參數走 --defaults ⇒ 不會送。
                #   ⚠ 上游 `platform_overrides` **只允許調高**（context.py:285），
                #     所以走轉接層自己的 `platform_a=`／`platform_b=`（:1653）。
                #     表 1 檔案一個字都不改。
                #   實測（2.45 km²、h_ceil=120、作業高度 120、GSD 0.05、風 0）：
                #       A=120 B=100 band=20 → NO_GO
                #       A=100 B=100 band=20 → NO_GO（只差不到 1 m 的地形起伏）
                #       A=99  B=99  band=20 → ['A','B'] 1424 s
                #       A=100 B=100 band=15 → ['A','B'] 1424 s
                #   ⇒ h_ceil 120 配 min_fw 100 時可用窗恰好 20 m，等於把 20 m 的
                #     坡度包絡餘裕壓到零，**任何**起伏都會讓它不可行。
                _spec_a, _spec_b = _PLAT_A, _PLAT_B
                _mf = _hv_get('hv_min_fw_agl_m', 'min_fw_agl_m', default=None)
                if isinstance(_mf, dict):
                    import dataclasses as _dc
                    _lowered = []
                    for _key, _base in (('A', _PLAT_A), ('B', _PLAT_B)):
                        _v = _mf.get(_key)
                        if _v is None:
                            continue
                        _v = float(_v)
                        if abs(_v - float(_base.min_fw_operation_agl_m)) < 1e-9:
                            continue
                        # 轉換完成高度的上緣是硬底線：低於它等於要求飛機在轉換
                        # 還沒完成前就用固定翼巡航。表 1 給 A 80–100、B 70–90。
                        _tc_hi = float(_base.transition_complete_agl_m[1])
                        if _v < _tc_hi - 1e-9:
                            raise ValueError(
                                f"min_fw_agl_m[{_key}]={_v:g} 低於轉換完成高度上緣 "
                                f"{_tc_hi:g} m ⇒ 等於要求在轉換完成前就固定翼巡航")
                        _new = _dc.replace(_base, min_fw_operation_agl_m=_v)
                        if _key == 'A':
                            _spec_a = _new
                        else:
                            _spec_b = _new
                        if _v < float(_base.min_fw_operation_agl_m):
                            _lowered.append(
                                f"{_base.name}（構型 {_key}）"
                                f"{_base.min_fw_operation_agl_m:g} → {_v:g} m")
                    if _lowered:
                        _m = 'min_fw_lowered(' + '；'.join(_lowered) + ')'
                        _log.getLogger(__name__).warning(
                            f"HV-DCCPP: {_m}。表 1 的值是規劃值不是機體極限（使用方式欄："
                            f"「任務前需依地形、感測器與法規調整」），但它會寫進飛控的 "
                            f"Q_RTL_ALT（匯出的 .param，每條路徑都生效）⇒ QRTL 進場高度"
                            f"跟著降；另外 FENCE_ALT_MIN 也綁著它，但只在走 "
                            f"mavlink_link.upload_plan 寫參數那條路時才會送出。")
                        _prenotes.append(_m)

                # ── 高度帶下限（坡度包絡在起伏地形上需要的活動空間）─────────
                #   ⚠ 規格 §4.6 給的 20 m 是「本文預設」(來源 D)，不是任務輸入，
                #     而且全部文件都找不到它為什麼是 20 不是 15 或 25 的推導。
                #     上游自己的測試在 h_ceil=120 的情境就用 15
                #     （tests/hv_dccpp/test_preflight.py:354）。
                #   ⚠⚠ 同一個常數還被另外兩處借用：相機的 GSD 可達性判準
                #     （h_max ≥ h_minfw + 它）與相機預設目標高（h_max − 它）。
                #     調小 = 三件事一起放寬，所以要記進 note。
                _pc_over = dict(_p.get('hv_cfg') or {}) if isinstance(_p, dict) else {}
                _bm = _hv_get('hv_dh_band_min_m', 'dh_band_min_m', default=None)
                if _bm is not None and 'dh_band_min_m' not in _pc_over:
                    _pc_over['dh_band_min_m'] = float(_bm)
                _pp = dict(_p) if isinstance(_p, dict) else {}
                if _pc_over:
                    _pp['hv_cfg'] = _pc_over
                _pcfg = to_planner_config(_pp, base=HvPlannerConfig(dem_step_m=25.0))
                if float(_pcfg.dh_band_min_m) < 20.0 - 1e-9:
                    _mb = f"dh_band_min_relaxed({_pcfg.dh_band_min_m:g} m，預設 20)"
                    _log.getLogger(__name__).warning(
                        f"HV-DCCPP: {_mb}。這同時放寬了 GSD 可達性判準"
                        f"（h_max ≥ 最低作業高度 + {_pcfg.dh_band_min_m:g}）與相機預設目標高。")
                    _prenotes.append(_mb)

                try:
                    _band_min = float(getattr(_pcfg, 'dh_band_min_m', 20.0))
                    _clr = float(_hv_get('hv_terrain_clearance_m',
                                         'terrain_clearance_m', default=36.0) or 36.0)
                    # ⚠ 這裡算的是**平地**的帶寬；真正的判準還要再扣地形起伏
                    #   (z_max − z_min)。所以「剛好等於下限」也是會失敗的
                    #   —— HERO 在 h_ceil=120 就是 20 − 起伏 < 20 而 NO_GO。
                    #   把「勉強夠」的也一起講出來，不然使用者會以為只有 A 有事。
                    _tight, _thin = [], []
                    for _sp in (_spec_a, _spec_b):
                        _lo = max(float(_sp.min_fw_operation_agl_m), _clr)
                        _bw = _ceil - _lo
                        _txt = (f"{_sp.name}（構型 {_sp.cls}）：{_ceil:.0f} − {_lo:.0f}"
                                f" = {_bw:.0f} m")
                        if _bw < _band_min:
                            _tight.append(f"{_txt} < {_band_min:.0f} m ⇒ 需要 h_ceil ≥ "
                                          f"{_lo + _band_min:.0f} m 再加地形起伏")
                        elif _bw < _band_min + 5.0:
                            _thin.append(f"{_txt}，只比下限多 {_bw - _band_min:.0f} m")
                    if _tight:
                        _m = ('altitude_band_too_narrow(' + '；'.join(_tight) + ')')
                        _log.getLogger(__name__).warning(
                            f"HV-DCCPP: {_m}。AGL 上限是"
                            f"{'法規值 120 m（未申請豁免）' if _waiver is None else f'豁免值 {_waiver:.0f} m'}"
                            f"；這些構型會分不到工作，全部落空時整趟是 NO_GO。"
                            + (f"另外 {'；'.join(_thin)}，地形起伏會把它吃掉。" if _thin else ""))
                        _prenotes.append(_m)
                except Exception as _e:      # noqa: BLE001
                    _log.getLogger(__name__).debug(f"高度帶預檢跳過：{_e}")

                # 規劃前先送一次：NO_GO 會直接丟例外，尾端那次根本跑不到。
                if _prenotes and self.hv_on_report:
                    self.hv_on_report({'validation_note': ' ; '.join(_prenotes),
                                       'validation_notes': list(_prenotes)})

                _hv = HvDccppCoordinator(
                    class_of=self.hv_class_of,          # ★ 不給的話整隊會被當成構型 A
                    # ★ 生效後的平台規格（可能調過最低固定翼作業高度）。不傳的話
                    #   轉接層會用它自己匯入時綁定的表 1 常數，面板的值就靜靜失效。
                    platform_a=_spec_a, platform_b=_spec_b,
                    sensor_a=_sensor_pair, sensor_b=_sensor_pair,
                    payload_reqs=_reqs,
                    cfg=_pcfg,
                    fence_zones=_zones,
                    home=self.hv_home,
                    alternates=self.hv_alternates,
                    wind=(self.hv_wind if self.hv_wind is not None else _hv_get('hv_wind')),
                    c2_range_km=float(_hv_get('hv_c2_range_km', 'c2_range_km', default=25.0)),
                    terrain_clearance_m=_hv_get('hv_terrain_clearance_m', 'terrain_clearance_m'),
                    # ⚠⚠⚠ None ＝**沒有豁免** ⇒ 上游用 max_agl_m（寫死 120，就是
                    #   民航法規值）當 h_ceil。`float(None)` 會炸，而給 200.0 當
                    #   預設等於在使用者沒說的情況下**假設他拿到了 200 m 豁免**。
                    waiver_max_agl_m=_waiver,
                    dem_step_m=float(_hv_get('hv_dem_step_m', default=50.0)),
                    # ★ 一定要 3000：轉接層預設 1500 < context.TERRAIN_MARGIN_M(3000)
                    #   ⇒ ENU 地形網格 55% 取樣為 nan（手冊 §11.1 實測）
                    dem_margin_m=float(_hv_get('hv_dem_margin_m', default=3000.0)),
                    energy=_hv_get('hv_energy'),
                    reduce_waypoints=bool(_hv_get('hv_reduce_waypoints', default=True)),
                    progress=self.hv_progress, on_report=self.hv_on_report,
                    raise_on_no_go=True)
                self.hv_coordinator = _hv      # 供匯出／上傳（手冊 §12）與重規劃（§13）取用

                result = _hv.plan_coverage_dccpp(
                    areas=areas, drones=drones, coverage_width_m=coverage_width_m,
                    overlap_rate=_ov, auto_scan_angle=auto_scan_angle,
                    coverage_altitude=coverage_altitude,
                    terrain_func=terrain_func,   # DEM 已在上面轉成回呼
                    enable_altitude=enable_altitude,
                    mounting_angle_deg=mounting_angle_deg,
                    dem_path="",                 # 已轉成 terrain_func，不重複讀 raster
                    coordination_mode=coordination_mode,
                    runway_bearing_deg=runway_bearing_deg, runway_length_m=runway_length_m,
                    landing_bearing_deg=landing_bearing_deg,
                    landing_rollout_m=landing_rollout_m)

                _log.getLogger(__name__).info(
                    f"HV-DCCPP: makespan={result['total_makespan']:.1f}s "
                    f"({len(result.get('assembled_paths') or {})} 機) "
                    f"id_map={_hv.id_map} notes={len(_hv.notes)}")

                # ── 把切出來的子區鍵換回使用者畫的那塊區 ────────────────
                #   不換的話 `allocation` 會用子區 id 當鍵，任何拿它對回輸入
                #   區域的下游都會對不上（契約測試就是這樣抓到的）。
                if _src_of and isinstance(result.get('allocation'), dict):
                    _merged: dict = {}
                    for _k, _v in result['allocation'].items():
                        _src = _src_of.get(_k, _k)
                        _merged.setdefault(_src, [])
                        for _d in (_v if isinstance(_v, (list, tuple)) else [_v]):
                            if _d not in _merged[_src]:
                                _merged[_src].append(_d)
                    result['allocation'] = {k: sorted(v) for k, v in _merged.items()}

                # ── 收尾（一）：有沒有人真的分到工作 ────────────────────
                #   ⚠⚠⚠ 混編機隊（A、B 都有）碰到**只切出一個胞**的作業區時，
                #   殘帶對拍賣把每個母胞硬鎖成「A 一台 + B 一台」，兩台吃不下就
                #   整胞放棄 ⇒ 回傳 makespan=0.0、assembled_paths=None、
                #   allocation={}，而 status 仍是 GO_WITH_WARNINGS、errors 空的。
                #   實測（5.0×4.0 km 單一區、風 5 m/s）：
                #       4A+0B → 4/4 台、8A+0B → 8/8 台、0A+4B → 4/4 台
                #       1A+1B / 4A+4B / 8A+8B → **0 台**，加機完全沒用
                #   同樣 20 km² 切成 4 塊就變成 8/8 台。決定量是**胞的數目**不是面積。
                #   ⚠ 兩道收尾都可能有話要說，所以先收集再一次送出：
                #     `hv_on_report` 在 GUI 端是 dict.update，後送的會**蓋掉**先送的。
                _vnotes: list[str] = list(_prenotes)
                _working = [u for u, p in (_hv.last_plan.plans or {}).items() if p.strips]
                # ── 部分機隊閒置：要說出**是哪個構型**沒分到 ────────────
                #   實測（2.45 km² 單區、1 Pioneer + 1 HERO、風 0）：
                #     作業高度 ≥134 m → 兩型都有工作
                #     作業高度 ≤133 m → **只有 HERO**，Pioneer 一條條帶都沒有
                #   而且與對地淨空無關（手動設 0 結果相同），兩型的 h_op 也一樣，
                #   所以不是高度被夾，是分配層算出來不派它。
                #   使用者只看到「1/2 架」，完全不知道要調哪個旋鈕，所以這裡點名。
                #   ⚠ 提示只在**該構型真的閒置**時才給。無條件附上去會在
                #     「兩型都有工作、只是台數少」的情況指向錯誤的旋鈕。
                if drones and _working and len(_working) < len(drones):
                    _idle_cls = sorted({self.hv_class_of(d) for d in drones}
                                       - {p.cls for p in _hv.last_plan.plans.values()
                                          if p.strips})
                    _msg = (f"partial_fleet_assigned({len(_working)}/{len(drones)} 台；"
                            f"沒分到工作的構型 {_idle_cls or '—'})")
                    _hint = ''
                    if 'A' in _idle_cls:
                        # 民航法規把作業高度壓在 120 m AGL，而合成 GSD 會把
                        # 高度上限訂在 1.05×h ⇒ 可用高度帶只有 0.05×h，要
                        # ≥134 m 才夠 20 m。所以在 120 m 下**不能**叫人調高，
                        # 要叫他把目標 GSD 填進去（面板預設 0.05 就夠）。
                        _hint = ('。構型 A 的最低固定翼作業高度是 120 m；沒填目標 '
                                 'GSD 時要 ~134 m 才會被派工，但那已經超過法規的 '
                                 '120 m ⇒ 請在面板填目標 GSD（≥0.035 m/px）')
                    _log.getLogger(__name__).warning(f"HV-DCCPP: {_msg}{_hint}")
                    _vnotes.append(_msg)

                if drones and not _working:
                    _msg = (f"no_aircraft_assigned(機隊 {len(drones)} 台，"
                            f"胞 {len(_hv.last_plan.cells)} 個)")
                    _log.getLogger(__name__).error(
                        f"HV-DCCPP: {_msg} ⇒ 這份計畫是空的（makespan="
                        f"{result.get('total_makespan')}）。混編機隊在只切出一個胞的"
                        f"作業區上會這樣：把作業區切成多塊，或改用單一構型。")
                    _vnotes.append(_msg)

                # ── 收尾（二）：真的跑一次 hv_dccpp.validate() ──────────
                #   ⚠⚠ 轉接層**從來不呼叫它**（全 repo grep 不到），所以在這裡補。
                #   preflight 的 feasibility 與 plan.uncovered 只涵蓋「可行性與覆蓋」，
                #   時空去衝突與能量預算是 validate 才會檢查的。實測成本 0.7 s。
                #   結果走 notes（前綴 `validate_failed`）讓 GUI 的橫幅顯示得到。
                self.hv_validation = None
                try:
                    import hv_dccpp as _hvlib
                    _cache = _hvlib.build_validation_cache(
                        _hv.last_plan, _hv.last_ctx, budget_only=False)
                    _v = _hvlib.validate(_hv.last_plan, _hv.last_ctx, _cache, _hv.cfg)
                    self.hv_validation = _v
                    if not _v.passed:
                        _bad = [k for k, ok in (('coverage', _v.coverage_ok),
                                                ('energy', _v.energy_ok)) if not ok]
                        if _v.conflicts:
                            _bad.append(f'conflicts={len(_v.conflicts)}')
                        _msg = f"validate_failed({', '.join(_bad) or 'unknown'})"
                        _log.getLogger(__name__).error(f"HV-DCCPP: {_msg}")
                        _vnotes.append(_msg)
                    else:
                        _log.getLogger(__name__).info(
                            f"HV-DCCPP: validate passed（衝突 0，"
                            f"最小水平間距 {_v.min_sep_h_m:.1f} m）")
                except Exception as _e:      # noqa: BLE001
                    # 驗證本身壞掉不該把規劃結果吃掉，但也不可以靜悄悄。
                    _log.getLogger(__name__).warning(
                        f"HV-DCCPP: validate 沒跑成功（{type(_e).__name__}: {_e}）"
                        f"⇒ 本次規劃**沒有**經過時空去衝突與能量驗證")
                    _vnotes.append('validate_failed(驗證本身沒跑成功)')
                if _vnotes and self.hv_on_report:
                    self.hv_on_report({'validation_note': ' ; '.join(_vnotes),
                                       'validation_notes': list(_vnotes)})
                return result

        # ⚠ 垂直起降剖面由 DCCPP v2 直通管線負責（見下方 prepend_vtol_takeoff）。
        #   legacy 管線沒有這個能力，走到那裡要說出來，不要靜默把參數吞掉。
        _vtol_alt = (float(vtol_transition_alt_m)
                     if vtol_transition_alt_m is not None
                     and float(vtol_transition_alt_m) > 0.0 else None)
        #: 宣告自己是 VTOL 的機號 —— 這些機體走垂直剖面而不是跑道剖面。
        #: ⚠ 判別式必須是 vehicle_type，不能是 turn_radius>0：VTOL 有轉彎半徑
        #:   但**不需要跑道**，用半徑判會規劃出 500 m 跑道起降。
        _vtol_ids = {d['drone_id'] for d in drones
                     if str(d.get('vehicle_type', '')).lower() == 'vtol'}

        # ── 建立 UAVState 物件 ────────────────────────────
        uav_states: List[UAVState] = []
        for d in drones:
            vtype = VehicleType.FIXED_WING if d.get(
                'vehicle_type', 'multirotor'
            ) == 'fixed_wing' else VehicleType.MULTIROTOR
            uav_states.append(UAVState(
                uav_id=d['drone_id'],
                position=d.get('position', (0.0, 0.0)),
                heading=d.get('heading', 0.0),
                turn_radius=d.get('turn_radius', 0.0),
                speed=d.get('speed', 15.0),
                vehicle_type=vtype,
                fov_width=d.get('fov_width', coverage_width_m),
            ))

        # ── 建立 CoveragePath 物件 ────────────────────────
        coverage_paths: List[CoveragePath] = []
        for area_dict in areas:
            polygon = area_dict['polygon']
            area_id = area_dict.get('area_id', id(area_dict))
            priority = area_dict.get('priority', 1.0)

            if auto_scan_angle:
                scan_angle = PolygonUtils.optimal_scan_angle(polygon)
            else:
                scan_angle = area_dict.get('scan_angle_deg', 0.0)

            fov_paths = RegionDivider.decompose_by_fov(
                polygon, scan_angle, actual_width, overlap_rate
            )

            cp = CoveragePath(
                area_id=area_id,
                area_polygon=polygon,
                scan_angle_deg=scan_angle,
                priority=priority,
            )
            for fp in fov_paths:
                cp.operations.append(OperationSegment(
                    index=fp['index'],
                    left_point=fp['left_point'],
                    right_point=fp['right_point'],
                    length_m=fp['length_m'],
                ))
            coverage_paths.append(cp)

        # ── 建立 Dubins 路徑組裝器（可選）──────────────────
        path_assembler = None
        try:
            from core.trajectory.dccpp_path_assembler import DCCPPPathAssembler
            # 檢測是否有固定翼 UAV
            has_fw = any(u.turn_radius > 0 for u in uav_states)
            dubins_gen = None
            default_turn_radius = 50.0
            if has_fw:
                try:
                    import math as _math
                    from core.base.fixed_wing_constraints import FixedWingConstraints
                    from core.trajectory.dubins_trajectory import DubinsTrajectoryGenerator
                    fw_uav = next(u for u in uav_states if u.turn_radius > 0)
                    default_turn_radius = fw_uav.turn_radius
                    # FixedWingConstraints 用 R = V²/(g·tan(φ)) · safety 計算半徑
                    # 反推出對應所需空速：V = sqrt(R · g · tan(φ) / safety)
                    _phi_deg = 45.0
                    _safety = 1.2
                    _g = 9.81
                    _v_needed = _math.sqrt(
                        default_turn_radius * _g * _math.tan(_math.radians(_phi_deg)) / _safety
                    )
                    constraints = FixedWingConstraints(
                        cruise_airspeed_mps=_v_needed,
                        max_bank_angle_deg=_phi_deg,
                        safety_factor=_safety,
                    )
                    dubins_gen = DubinsTrajectoryGenerator(constraints)
                    logger.info(
                        f"DCCPP Dubins 生成器建立：R={constraints.get_min_turn_radius():.1f}m, "
                        f"V={_v_needed:.1f}m/s, φ={_phi_deg}°"
                    )
                except Exception as e:
                    logger.warning(f"Dubins 生成器建立失敗，退化為弧線近似: {e}")
                    pass  # 退化為近似弧線
            path_assembler = DCCPPPathAssembler(
                dubins_gen=dubins_gen,
                default_altitude=coverage_altitude,
                turn_radius=default_turn_radius,
            )
        except ImportError:
            pass

        # ── 建立協調規劃器（可選）──────────────────────
        coord_planner = None
        if coordination_mode == "coordinated":
            coord_planner = CoordinatedPlanner(
                cruise_speed=uav_states[0].speed if uav_states else 15.0,
                turn_radius=default_turn_radius if 'default_turn_radius' in dir() else 50.0,
            )

        # ── DCCPP v2 直通管線（AreaProcessor → Allocator → IDP_Solver → PathBuilder）──
        if _use_new_dccpp:
            try:
                from core.dccpp.area_processor import AreaProcessor
                from core.dccpp.dccpp_path_builder import DCCPPPathBuilder
                from core.dccpp.idp_solver import to_legacy_mdtsp_result

                # 反推 R_min（取最大值，覆蓋所有固定翼）
                _R = max((u.turn_radius for u in uav_states if u.turn_radius > 0), default=50.0)

                # 1. 各區域產生 ScanLine + transformer
                w_ovl = actual_width * float(overlap_rate)
                w_i = actual_width
                from core.geometry.coordinate import CoordinateTransformer as _CT
                ap = AreaProcessor()
                area_results = {}      # area_id -> AreaProcessingResult
                area_transformers = {} # area_id -> CoordinateTransformer
                effective_scan_angles = {}   # area_id -> 實際生效的掃描角 (compass)
                for area_dict in areas:
                    aid = area_dict.get('area_id', id(area_dict))
                    # ⚠⚠ 手動掃描角必須送到**這一支**才算生效。只送到 legacy 的
                    #    RegionDivider 上，log 會照印角度、面板會照顯示，但真正
                    #    飛出來的航點是這條管線產的，一點都不會變。
                    _ang = (None if auto_scan_angle
                            else float(area_dict.get('scan_angle_deg', 0.0)))
                    ar = ap.process_latlon(area_dict['polygon'], w_i, w_ovl,
                                           scan_angle_deg=_ang)
                    area_results[aid] = ar
                    area_transformers[aid] = _CT(ar.centroid_latlon[0], ar.centroid_latlon[1])
                    # 顯示值 = 飛的值：自動模式回報幾何算出來的角，手動模式回報
                    # 面板送進來的角（正規化到 0..360，面板允許負角）。
                    effective_scan_angles[aid] = float(ar.min_width_result.alpha_k_deg)

                # 2. 任務分配
                allocator = UAVTaskAllocator()
                alloc_res = allocator.allocate(uav_states, coverage_paths)

                # 3. 各區域 IDP + 路徑建構
                idp = IDP_Solver(min_turn_radius=_R, altitude=coverage_altitude)
                builder = DCCPPPathBuilder(
                    min_turn_radius=_R,
                    default_altitude=coverage_altitude,
                    step_size=10.0,
                )
                results = {}
                all_assembled = {}
                uav_by_id = {u.uav_id: u for u in uav_states}
                for cov in coverage_paths:
                    aid = cov.area_id
                    assigned_ids = alloc_res.area_assignments.get(aid, [])
                    if not assigned_ids:
                        continue
                    uav_inputs = [{
                        'uav_id': uid,
                        'lat': uav_by_id[uid].position[0],
                        'lon': uav_by_id[uid].position[1],
                        'heading': uav_by_id[uid].heading,
                    } for uid in assigned_ids]

                    idp_res = idp.solve(uav_inputs, area_results[aid], area_transformers[aid])
                    idp_res.area_id = aid

                    # 為每架 UAV 組裝完整 Dubins 路徑
                    tf = area_transformers[aid]
                    for uid, plan in idp_res.per_uav.items():
                        u = uav_by_id[uid]
                        built = builder.build(
                            start_lat=u.position[0],
                            start_lon=u.position[1],
                            start_heading_compass_deg=u.heading,
                            ordered_scan_lines=plan.ordered_scan_lines,
                            directions=plan.directions,
                            transformer=tf,
                            uav_id=uid,
                        )
                        # 為固定翼自動補上 TAKEOFF / LANDING，
                        # 讓 ArduPlane AUTO 能完整執行（NAV_TAKEOFF→任務→NAV_LAND）
                        if (u.turn_radius > 0 or uid in _vtol_ids) and built.waypoints:
                            # 使用使用者設定的跑道方向；未設定時才自動計算
                            if runway_bearing_deg is not None:
                                takeoff_bearing = float(runway_bearing_deg)
                            else:
                                first_wp = built.waypoints[0]
                                takeoff_bearing = math.degrees(math.atan2(
                                    (first_wp.lon - u.position[1]) *
                                    math.cos(math.radians(u.position[0])),
                                    first_wp.lat - u.position[0],
                                )) % 360.0
                            if landing_bearing_deg is not None:
                                _landing_bearing = float(landing_bearing_deg)
                            else:
                                last_wp = built.waypoints[-1]
                                _landing_bearing = math.degrees(math.atan2(
                                    (u.position[1] - last_wp.lon) *
                                    math.cos(math.radians(last_wp.lat)),
                                    u.position[0] - last_wp.lat,
                                )) % 360.0

                        if uid in _vtol_ids and _vtol_alt is not None and built.waypoints:
                            # ── 4+1 QuadPlane：真的垂直起降 ──
                            #   ⚠ 不是「跑道剖面配 VTOL 韌體」：第一段必須是純
                            #     垂直爬升（同經緯度），否則 3D 圖上是一條 500 m
                            #     斜線，而面板的「轉換高度」根本沒人讀。
                            builder.prepend_vtol_takeoff(
                                built,
                                home_lat=u.position[0],
                                home_lon=u.position[1],
                                bearing_compass_deg=takeoff_bearing,
                                transition_alt_m=_vtol_alt,
                                cruise_speed_mps=u.speed,
                            )
                            builder.append_vtol_landing(
                                built,
                                home_lat=u.position[0],
                                home_lon=u.position[1],
                                bearing_compass_deg=_landing_bearing,
                                transition_alt_m=_vtol_alt,
                                cruise_speed_mps=u.speed,
                            )
                        elif u.turn_radius > 0 and built.waypoints:
                            builder.prepend_takeoff(
                                built,
                                home_lat=u.position[0],
                                home_lon=u.position[1],
                                runway_bearing_compass_deg=takeoff_bearing,
                                climb_distance_m=runway_length_m,
                            )
                            builder.append_landing(
                                built,
                                home_lat=u.position[0],
                                home_lon=u.position[1],
                                runway_bearing_compass_deg=_landing_bearing,
                                landing_rollout_m=float(landing_rollout_m or 0.0),
                            )
                        all_assembled[uid] = built

                    legacy = to_legacy_mdtsp_result(idp_res, aid)
                    legacy.assembled_paths = {
                        uid: all_assembled[uid] for uid in idp_res.per_uav if uid in all_assembled
                    }
                    results[aid] = legacy

                total_makespan = max(
                    (sum(p.total_length_m for p in [bp]) / max(uav_by_id[bp.uav_id].speed, 1e-3)
                     for bp in all_assembled.values()),
                    default=0.0,
                )

                return {
                    'results': results,
                    'allocation': alloc_res.area_assignments,
                    'coverage_paths': coverage_paths,
                    'uav_states': uav_states,
                    'total_makespan': total_makespan,
                    'assembled_paths': all_assembled if all_assembled else None,
                    'effective_scan_angles': effective_scan_angles,
                }
            except Exception as _e:
                logger.warning(f"DCCPP v2 直通管線失敗，回退至舊 DCCPPSolver: {_e}", exc_info=True)

        if _use_new_dccpp:
            solver = DCCPPSolver(
                allocator=UAVTaskAllocator(),
                idp_solver=IDPSolver(),
                altitude_planner=AltPlannerNew(
                    h_cov=coverage_altitude,
                ),
                path_assembler=path_assembler,
                coordinated_planner=coord_planner,
            )
        else:
            solver = DCCPPSolver(
                allocator=GreedyAllocator(),
                idp_solver=IDPSolver(),
                altitude_planner=AltitudePlanner(
                    coverage_altitude=coverage_altitude,
                ),
                path_assembler=path_assembler,
                coordinated_planner=coord_planner,
            )
        results = solver.solve(
            uav_states, coverage_paths,
            terrain_func=terrain_func,
            enable_altitude=enable_altitude,
            enable_dubins_assembly=(path_assembler is not None),
            coordination_mode=coordination_mode,
        )

        # ── 計算總 makespan ──────────────────────────────
        total_makespan = max(
            (r.makespan for r in results.values()),
            default=0.0,
        )

        # ── 提取分配結果 ─────────────────────────────────
        alloc_result = solver.allocator.allocate(uav_states, coverage_paths)
        # 相容新舊介面：新版返回 AllocationResult，舊版返回 Dict
        if hasattr(alloc_result, 'area_assignments'):
            allocation = alloc_result.area_assignments
        else:
            allocation = alloc_result

        # ── 彙整所有 assembled_paths ─────────────────────
        all_assembled = {}
        for area_id, mdtsp_result in results.items():
            if mdtsp_result.assembled_paths:
                all_assembled.update(mdtsp_result.assembled_paths)

        return {
            'results': results,
            'allocation': allocation,
            'coverage_paths': coverage_paths,
            'uav_states': uav_states,
            'total_makespan': total_makespan,
            'assembled_paths': all_assembled if all_assembled else None,
        }


# ==========================================
# 使用範例
# ==========================================
def _test_new_features():
    """測試 Tasks 4/5/7 的新功能（allocate_by_cost / handle_* / create_optimized_swarm_coverage）"""
    print("=" * 60)
    print("Tasks 4/5/7 整合測試")
    print("=" * 60)

    # ── 準備：建立兩塊區域和三架無人機 ─────────────────────────────
    area_a = {
        'area_id': 1, 'priority': 2.0,
        'polygon': [
            (23.700, 120.420), (23.705, 120.420),
            (23.705, 120.426), (23.700, 120.426),
        ],
    }
    area_b = {
        'area_id': 2, 'priority': 1.0,
        'polygon': [
            (23.706, 120.420), (23.710, 120.420),
            (23.710, 120.424), (23.706, 120.424),
        ],
    }
    drones_cfg = [
        {'drone_id': 1, 'name': 'UAV-1', 'position': (23.702, 120.419), 'speed': 12.0},
        {'drone_id': 2, 'name': 'UAV-2', 'position': (23.708, 120.418), 'speed': 10.0},
        {'drone_id': 3, 'name': 'UAV-3', 'position': (23.703, 120.427), 'speed': 10.0},
    ]

    coordinator = SwarmCoordinator()

    # ── Task 7：create_optimized_swarm_coverage ──────────────────────
    swarm = coordinator.create_optimized_swarm_coverage(
        areas=[area_a, area_b],
        drones=drones_cfg,
        coverage_width_m=100.0,
        overlap_rate=0.1,
        auto_scan_angle=True,
    )
    print(f"SwarmMission: {swarm}")
    print(f"  無人機數: {swarm.stats['total_drones']}")
    print(f"  總航點數: {swarm.stats['total_waypoints']}")
    print(f"  總距離: {swarm.stats['total_distance']:.0f}m")
    print(f"  預估時間: {swarm.stats['estimated_time']/60:.1f}min")
    assert swarm.stats['total_drones'] == 3
    assert swarm.stats['total_waypoints'] > 0

    # 列出各區域作業路徑分配
    cps = swarm.coverage_paths
    alloc = swarm.allocation
    for cp in cps:
        drone_ids = alloc.get(cp.area_id, [])
        print(f"\n  區域 {cp.area_id}: scan={cp.scan_angle_deg:.1f}°, "
              f"ops={len(cp.operations)}, drones={drone_ids}")
        for op in cp.operations[:3]:
            print(f"    {op}")

    # ── Task 4：單獨測試 allocate_by_cost ────────────────────────────
    print("\n[Task 4] allocate_by_cost 測試")
    drone_infos = [DroneInfo(**{k: v for k, v in
                   {'drone_id': d['drone_id'], 'name': d['name'],
                    'position': d['position'], 'speed_mps': d['speed']}.items()})
                   for d in drones_cfg]
    alloc2 = coordinator.allocate_by_cost(cps, drone_infos)
    total_assigned = sum(len(v) for v in alloc2.values())
    print(f"  分配結果: {alloc2}")
    print(f"  總分配無人機數: {total_assigned} (預期 3)")
    assert total_assigned == 3

    # ── Task 5：handle_drone_dropout ────────────────────────────────
    print("\n[Task 5] handle_drone_dropout 測試")
    dropout_id = alloc2[cps[0].area_id][0] if alloc2[cps[0].area_id] else 1
    result = coordinator.handle_drone_dropout(dropout_id, cps)
    total_after = sum(len(v) for v in result.values())
    print(f"  UAV-{dropout_id} 故障後重新分配: {result}")
    print(f"  剩餘分配數: {total_after} (預期 2)")
    assert total_after == 2

    # ── Task 5：handle_new_area ─────────────────────────────────────
    print("\n[Task 5] handle_new_area 測試")
    new_poly = [
        (23.711, 120.420), (23.714, 120.420),
        (23.714, 120.424), (23.711, 120.424),
    ]
    remaining_drones = [d for d in drone_infos if d.drone_id != dropout_id]
    new_cp = coordinator.handle_new_area(
        new_area_polygon=new_poly,
        new_area_id=99,
        scan_angle_deg=0.0,
        coverage_width_m=100.0,
        current_coverage_paths=cps,
        available_drones=remaining_drones,
    )
    print(f"  新區域 CoveragePath: {new_cp}")
    print(f"  作業路徑數: {len(new_cp.operations)}")
    assert len(new_cp.operations) > 0
    assert any(op.assigned_drone_id is not None for op in new_cp.operations)

    print("\n所有斷言通過")


if __name__ == '__main__':
    _test_new_features()
    print()
    from survey_mission import SurveyMissionBuilder
    
    # 建立 Survey 任務
    corners = [
        (23.702732, 120.419333),
        (23.703732, 120.419333),
        (23.703732, 120.420333),
        (23.702732, 120.420333),
    ]
    
    survey = (SurveyMissionBuilder("群飛測試")
             .set_area(corners)
             .set_altitude(50.0)
             .set_speed(10.0)
             .set_grid(angle=0.0, spacing=10.0)
             .set_subdivisions(4, spacing=3.0)
             .build())
    
    # 創建群飛任務
    coordinator = SwarmCoordinator()
    swarm = coordinator.create_swarm_from_survey(survey)
    
    # 顯示簡報
    print(swarm.generate_mission_briefing())
    
    # 驗證
    valid, errors = swarm.validate()
    if valid:
        print("\n✓ 群飛任務驗證通過")
    else:
        print("\n✗ 群飛任務驗證失敗:")
        for error in errors:
            print(f"  - {error}")
