"""
結構化覆蓋路徑模組

將覆蓋任務的路徑結構化為 Entry / Operation / Transfer 三類，
支援後續動態重規劃（無人機故障退出、新區域出現等場景）。

對應論文《Multiple fixed-wing UAVs collaborative coverage 3D path planning
method for complex areas》Section 2.3 的 Operation Path Model。
"""

import math
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict


# ==========================================
# 單條作業路徑段
# ==========================================
@dataclass
class OperationSegment:
    """
    單條作業路徑段（Operation Path Segment）

    代表一條平行掃描線，包含左右端點、飛行方向、
    以及分配狀態（已完成 / 已分配至哪台無人機）。
    """
    index: int                                    # 作業路徑編號（在區域內唯一）
    left_point: Tuple[float, float]               # 左端點 (lat, lon)
    right_point: Tuple[float, float]              # 右端點 (lat, lon)
    waypoints: List[Tuple[float, float]] = field(default_factory=list)
    length_m: float = 0.0                         # 作業路徑長度（公尺）
    covered: bool = False                         # 是否已完成覆蓋
    assigned_drone_id: Optional[int] = None       # 分配的無人機 ID（None = 未分配）

    def entry_points(self) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """返回 (left_point, right_point)，供距離計算使用"""
        return self.left_point, self.right_point

    def __repr__(self) -> str:
        status = "done" if self.covered else (
            f"drone={self.assigned_drone_id}" if self.assigned_drone_id is not None
            else "unassigned"
        )
        return (f"OperationSegment(idx={self.index}, "
                f"len={self.length_m:.1f}m, {status})")


# ==========================================
# 結構化覆蓋路徑
# ==========================================
@dataclass
class CoveragePath:
    """
    結構化覆蓋路徑（對應論文中一個子區域 Zk 的完整覆蓋規劃）

    包含：
    - 區域邊界與掃描方向
    - 所有 OperationSegment（作業路徑列表）
    - 區域優先級（供多機分配成本函數使用）
    """
    area_id: int                                  # 區域 ID
    area_polygon: List[Tuple[float, float]]       # 區域邊界 [(lat,lon), ...]
    scan_angle_deg: float                         # 掃描方向（度，0=北，順時針為正）
    operations: List[OperationSegment] = field(default_factory=list)
    priority: float = 1.0                         # 區域優先級（越大越優先）

    # ── 查詢方法 ──────────────────────────────────────────────────────
    def get_uncovered_operations(self) -> List[OperationSegment]:
        """取得所有未完成的作業路徑"""
        return [op for op in self.operations if not op.covered]

    def get_operations_by_drone(self, drone_id: int) -> List[OperationSegment]:
        """取得分配給指定無人機的所有作業路徑"""
        return [op for op in self.operations if op.assigned_drone_id == drone_id]

    def get_unassigned_operations(self) -> List[OperationSegment]:
        """取得尚未分配無人機的作業路徑"""
        return [op for op in self.operations if op.assigned_drone_id is None]

    # ── 修改方法 ──────────────────────────────────────────────────────
    def mark_covered(self, index: int) -> bool:
        """
        標記指定編號的作業路徑為已完成。

        返回:
            True 表示成功找到並標記；False 表示編號不存在
        """
        for op in self.operations:
            if op.index == index:
                op.covered = True
                return True
        return False

    def unassign_drone(self, drone_id: int) -> int:
        """
        取消指定無人機的所有未完成作業路徑分配。

        返回:
            取消分配的路徑數量
        """
        count = 0
        for op in self.operations:
            if op.assigned_drone_id == drone_id and not op.covered:
                op.assigned_drone_id = None
                count += 1
        return count

    # ── 統計方法 ──────────────────────────────────────────────────────
    def total_length(self) -> float:
        """計算所有作業路徑總長度（公尺）"""
        return sum(op.length_m for op in self.operations)

    def total_uncovered_length(self) -> float:
        """計算未覆蓋路徑總長度（公尺）"""
        return sum(op.length_m for op in self.operations if not op.covered)

    def coverage_rate(self) -> float:
        """計算覆蓋完成率（0–1）"""
        if not self.operations:
            return 1.0
        covered = sum(1 for op in self.operations if op.covered)
        return covered / len(self.operations)

    def to_waypoint_sequence(
        self,
        altitude: float = 50.0,
        speed: float = 10.0,
        home_position: Optional[Tuple[float, float]] = None,
        include_takeoff: bool = True,
        include_rtl: bool = True,
    ) -> 'WaypointSequence':
        """
        將覆蓋路徑轉換為 MAVLink 相容的 WaypointSequence

        轉換邏輯：
        1. 若 include_takeoff=True，加入 HOME + DO_CHANGE_SPEED + NAV_TAKEOFF
        2. 按 operations 順序，以之字形飛行依序生成 NAV_WAYPOINT
        3. 若 include_rtl=True，最後加入 NAV_RETURN_TO_LAUNCH

        參數:
            altitude: 飛行高度（公尺，相對高度）
            speed: 飛行速度（m/s）
            home_position: HOME 位置 (lat, lon)，若 None 則用第一條作業路徑的起點
            include_takeoff: 是否包含起飛指令
            include_rtl: 是否包含 RTL

        返回:
            WaypointSequence 實例
        """
        # 延遲匯入，避免模組載入時的循環依賴
        import sys as _sys
        from pathlib import Path as _Path
        _root = _Path(__file__).parent.parent
        if str(_root) not in _sys.path:
            _sys.path.insert(0, str(_root))
        from mission.waypoint import (  # noqa: PLC0415
            WaypointSequence, create_home_waypoint,
            create_takeoff_waypoint, create_navigation_waypoint,
            create_rtl_waypoint, create_change_speed_command,
        )

        seq = WaypointSequence()

        # 決定 HOME 位置
        if home_position is None:
            home_position = self.operations[0].left_point if self.operations else (0.0, 0.0)
        home_lat, home_lon = home_position

        # 起飛指令序列（seq 編號由 WaypointSequence.add() 自動更新）
        if include_takeoff:
            seq.add(create_home_waypoint(home_lat, home_lon, 0.0))
            seq.add(create_change_speed_command(speed, 0))
            seq.add(create_takeoff_waypoint(home_lat, home_lon, altitude, 0))

        # 依作業路徑順序生成航點（之字形：每段選擇離上一終點最近的端點為起點）
        prev_end: Optional[Tuple[float, float]] = home_position if include_takeoff else None

        for op in self.operations:
            left = op.left_point
            right = op.right_point

            # 選擇離 prev_end 較近的端點為起點
            if prev_end is not None:
                if _haversine(prev_end, right) < _haversine(prev_end, left):
                    start, end = right, left
                else:
                    start, end = left, right
            else:
                start, end = left, right

            seq.add(create_navigation_waypoint(start[0], start[1], altitude, 0))
            seq.add(create_navigation_waypoint(end[0], end[1], altitude, 0))
            prev_end = end

        # 加入 RTL
        if include_rtl:
            seq.add(create_rtl_waypoint(0))

        return seq

    @staticmethod
    def to_swarm_waypoint_sequences(
        coverage_paths: List['CoveragePath'],
        drone_assignments: Dict[int, List[int]],
        altitude: float = 50.0,
        speed: float = 10.0,
    ) -> Dict[int, 'WaypointSequence']:
        """
        批量轉換多區域多機覆蓋路徑為各無人機的 WaypointSequence

        參數:
            coverage_paths: 所有區域的 CoveragePath
            drone_assignments: {area_id: [drone_id, ...]} 分配結果
            altitude: 飛行高度
            speed: 飛行速度

        返回:
            {drone_id: WaypointSequence}
        """
        # 延遲匯入
        import sys as _sys
        from pathlib import Path as _Path
        _root = _Path(__file__).parent.parent
        if str(_root) not in _sys.path:
            _sys.path.insert(0, str(_root))
        from mission.waypoint import WaypointSequence  # noqa: PLC0415

        # 收集各無人機分配到的所有作業路徑（跨區域）
        drone_ops: Dict[int, List['OperationSegment']] = {}
        for cp in coverage_paths:
            area_drone_ids = drone_assignments.get(cp.area_id, [])
            for op in cp.operations:
                if op.assigned_drone_id is not None and op.assigned_drone_id in area_drone_ids:
                    drone_ops.setdefault(op.assigned_drone_id, []).append(op)

        # 為每架無人機建立臨時 CoveragePath 並轉換
        result: Dict[int, 'WaypointSequence'] = {}
        for drone_id, ops in drone_ops.items():
            if not ops:
                result[drone_id] = WaypointSequence()
                continue
            temp_cp = CoveragePath(
                area_id=drone_id,
                area_polygon=[],
                scan_angle_deg=0.0,
                operations=ops,
            )
            result[drone_id] = temp_cp.to_waypoint_sequence(
                altitude=altitude,
                speed=speed,
            )

        return result

    def __repr__(self) -> str:
        return (f"CoveragePath(area_id={self.area_id}, "
                f"ops={len(self.operations)}, "
                f"priority={self.priority}, "
                f"coverage={self.coverage_rate():.0%})")


def _haversine(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """計算兩個 (lat, lon) 點之間的距離（公尺）"""
    R = 6371000.0
    lat1, lon1 = math.radians(p1[0]), math.radians(p1[1])
    lat2, lon2 = math.radians(p2[0]), math.radians(p2[1])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return R * 2 * math.asin(math.sqrt(a))


# ==========================================
# 單元測試
# ==========================================
if __name__ == '__main__':

    def test_coverage_path():
        """測試 CoveragePath 和 OperationSegment 的基本操作"""
        print("=" * 55)
        print("test_coverage_path()")
        print("=" * 55)

        # 建立測試作業路徑段
        ops = [
            OperationSegment(
                index=i,
                left_point=(23.700 + i * 0.001, 120.420),
                right_point=(23.700 + i * 0.001, 120.426),
                length_m=611.0,
            )
            for i in range(5)
        ]

        # 建立 CoveragePath
        cp = CoveragePath(
            area_id=1,
            area_polygon=[
                (23.700, 120.420), (23.705, 120.420),
                (23.705, 120.426), (23.700, 120.426),
            ],
            scan_angle_deg=0.0,
            operations=ops,
            priority=2.0,
        )

        print(f"建立 CoveragePath: {cp}")
        print(f"總長度: {cp.total_length():.0f}m")
        assert cp.total_length() == 5 * 611.0

        # 測試分配
        for i in range(3):
            cp.operations[i].assigned_drone_id = 1
        for i in range(3, 5):
            cp.operations[i].assigned_drone_id = 2

        assert len(cp.get_operations_by_drone(1)) == 3
        assert len(cp.get_operations_by_drone(2)) == 2
        print(f"無人機1 路徑數: {len(cp.get_operations_by_drone(1))} (預期 3)")
        print(f"無人機2 路徑數: {len(cp.get_operations_by_drone(2))} (預期 2)")

        # 測試標記完成
        cp.mark_covered(0)
        cp.mark_covered(1)
        assert len(cp.get_uncovered_operations()) == 3
        print(f"完成 2 條後未完成數: {len(cp.get_uncovered_operations())} (預期 3)")
        print(f"覆蓋率: {cp.coverage_rate():.0%}")

        # 測試取消分配
        removed = cp.unassign_drone(2)
        assert removed == 2
        assert len(cp.get_unassigned_operations()) == 2
        print(f"取消無人機2分配後未分配數: {len(cp.get_unassigned_operations())} (預期 2)")

        print("\n所有斷言通過")

    def test_to_waypoint_sequence():
        """測試 to_waypoint_sequence()：驗證 seq 連續性與 QGC 格式"""
        print("=" * 55)
        print("test_to_waypoint_sequence()")
        print("=" * 55)

        # 建立含 5 條作業路徑的 CoveragePath（雲林座標）
        ops = [
            OperationSegment(
                index=i,
                left_point=(23.700 + i * 0.001, 120.420),
                right_point=(23.700 + i * 0.001, 120.426),
                length_m=611.0,
            )
            for i in range(5)
        ]
        cp = CoveragePath(
            area_id=1,
            area_polygon=[
                (23.700, 120.420), (23.705, 120.420),
                (23.705, 120.426), (23.700, 120.426),
            ],
            scan_angle_deg=0.0,
            operations=ops,
        )

        # 呼叫 to_waypoint_sequence()
        wseq = cp.to_waypoint_sequence(
            altitude=50.0,
            speed=10.0,
            home_position=(23.700, 120.419),
        )

        # 驗證 seq 連續性
        seqs = [wp.seq for wp in wseq.waypoints]
        print(f"  航點數量: {len(seqs)}")
        print(f"  seq 序列: {seqs}")
        assert seqs == list(range(len(seqs))), f"seq 不連續: {seqs}"
        # 預期: HOME(0) + SPEED(1) + TAKEOFF(2) + 5*2 waypoints(3..12) + RTL(13) = 14 個
        assert len(seqs) == 14, f"預期 14 個航點，實際 {len(seqs)}"
        print(f"  [OK] seq 連續性驗證通過")

        # 用 to_qgc_format() 輸出並列印前 5 行
        lines = wseq.to_qgc_format()
        print(f"  QGC 格式前 5 行：")
        for line in lines[:5]:
            print(f"    {line.rstrip()}")

        print("\n  所有斷言通過")

    test_coverage_path()
    print()
    test_to_waypoint_sequence()
