"""
群飛協調器模組
負責多無人機任務的協調、同步、避撞等功能
支持時間錯開、高度分層、區域分配等策略
"""

import sys
from pathlib import Path
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass, field
import math
import logging

# 添加專案根目錄到路徑
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

logger = logging.getLogger(__name__)

from mission.mission_manager import Mission
from mission.survey_mission import SurveyMission
from mission.waypoint import Waypoint, WaypointSequence, MAVCommand
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
        """
        計算 LOITER 等待時間（智能避撞模式）
        
        返回:
            每台無人機的等待時間列表
        """
        try:
            from collision_avoidance import CollisionAvoidanceSystem
            
            collision_system = CollisionAvoidanceSystem(
                self.swarm_params['safety_distance']
            )
            
            loiter_times = []
            prev_waypoints = None
            
            for idx, drone in enumerate(self.drones):
                if not drone.mission or len(drone.mission.waypoints) == 0:
                    loiter_times.append(0.0)
                    continue
                
                # 獲取導航航點
                nav_waypoints = drone.mission.waypoints.get_navigation_waypoints()
                waypoints = [(wp.lat, wp.lon) for wp in nav_waypoints]
                
                if idx == 0:
                    # 第一台無人機不需要等待
                    loiter_times.append(0.0)
                    prev_waypoints = waypoints
                else:
                    # 計算需要等待的時間
                    if prev_waypoints and waypoints:
                        start_point = waypoints[0]
                        speed = drone.mission.params.get('speed', 10.0)
                        
                        loiter_time = collision_system.calculate_loiter_delay(
                            prev_waypoints, start_point, speed
                        )
                        
                        # 加上額外的時間錯開
                        loiter_time += idx * self.swarm_params['time_buffer']
                        
                        loiter_times.append(loiter_time)
                        prev_waypoints = waypoints
                    else:
                        loiter_times.append(idx * self.swarm_params['time_buffer'])
            
            return loiter_times
        except ImportError:
            # 如果無法導入 collision_avoidance，使用簡單的時間錯開
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
        try:
            from collision_avoidance import CollisionAvoidanceSystem
            
            collision_system = CollisionAvoidanceSystem()
            
            # 轉換為 QGC 格式
            lines = mission.waypoints.to_qgc_format()
            
            # 插入 LOITER 命令（在速度設定後）
            updated_lines = collision_system.insert_loiter_command(
                lines, loiter_time, insert_after_line=2
            )
            
            # 更新任務航點
            mission.waypoints = WaypointSequence.from_qgc_format(updated_lines)
        except Exception as e:
            print(f"插入 LOITER 命令失敗: {e}")
    
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
        R = 6371000.0
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
                for area_dict in areas:
                    aid = area_dict.get('area_id', id(area_dict))
                    ar = ap.process_latlon(area_dict['polygon'], w_i, w_ovl)
                    area_results[aid] = ar
                    area_transformers[aid] = _CT(ar.centroid_latlon[0], ar.centroid_latlon[1])

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
                        if u.turn_radius > 0 and built.waypoints:
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
                            builder.prepend_takeoff(
                                built,
                                home_lat=u.position[0],
                                home_lon=u.position[1],
                                runway_bearing_compass_deg=takeoff_bearing,
                                climb_distance_m=runway_length_m,
                            )
                            if landing_bearing_deg is not None:
                                _landing_bearing = float(landing_bearing_deg)
                            else:
                                last_wp = built.waypoints[-1]
                                _landing_bearing = math.degrees(math.atan2(
                                    (u.position[1] - last_wp.lon) *
                                    math.cos(math.radians(last_wp.lat)),
                                    u.position[0] - last_wp.lat,
                                )) % 360.0
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
