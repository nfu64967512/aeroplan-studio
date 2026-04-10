"""
作業路徑序列最佳化器模組

為每架無人機找到最短的作業路徑執行順序，減少轉移路徑長度。
演算法：最近鄰啟發式（Nearest Neighbor）+ 2-opt 局部改進。

對多旋翼簡化：轉向自由，不需論文中的二元方向決策，
直接以端點間 haversine 距離作為轉移成本。
"""

import sys
import math
from pathlib import Path
from typing import List, Tuple, TYPE_CHECKING

# 將專案根目錄加入路徑，以便直接執行此檔案時也能匯入
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from mission.coverage_path import OperationSegment


class SequenceOptimizer:
    """
    作業路徑序列最佳化器

    基於最近鄰啟發式 + 2-opt 局部改進，為每架無人機找到
    最短的作業路徑執行順序。

    設計原則：
    - 飛行方向由執行時動態決定（從較近端點進入）
    - 最佳化目標：最小化無人機在作業路徑之間的轉移總距離
    """

    @staticmethod
    def optimize_sequence(
        drone_position: Tuple[float, float],
        operations: List[OperationSegment],
    ) -> List[OperationSegment]:
        """
        最佳化作業路徑執行順序。

        演算法分兩階段：
        1. 最近鄰啟發式：從無人機當前位置出發，每次選取距離最近的
           未訪問作業路徑（以較近端點計算進入距離）
        2. 2-opt 局部改進：嘗試反轉子序列，若能縮短總長度則接受

        參數:
            drone_position: 無人機當前位置 (lat, lon)
            operations: 分配給該無人機的作業路徑列表

        返回:
            排序後的作業路徑列表（順序即為執行順序）
        """
        n = len(operations)
        if n == 0:
            return []
        if n == 1:
            return list(operations)

        hav = SequenceOptimizer._haversine

        # ── 階段 1：最近鄰啟發式 ──────────────────────────────────
        remaining = list(range(n))
        sequence: List[int] = []
        current_pos = drone_position

        while remaining:
            best_idx_in_remaining = 0
            best_dist = float('inf')

            for ri, op_i in enumerate(remaining):
                op = operations[op_i]
                d = min(hav(current_pos, op.left_point),
                        hav(current_pos, op.right_point))
                if d < best_dist:
                    best_dist = d
                    best_idx_in_remaining = ri

            chosen = remaining.pop(best_idx_in_remaining)
            sequence.append(chosen)

            # 更新當前位置為該作業路徑的出口端點（較遠的那端）
            op = operations[chosen]
            d_l = hav(current_pos, op.left_point)
            d_r = hav(current_pos, op.right_point)
            current_pos = op.right_point if d_l <= d_r else op.left_point

        # ── 階段 2：2-opt 局部改進 ────────────────────────────────
        best_cost = SequenceOptimizer._route_transit_cost(
            drone_position, sequence, operations
        )

        improved = True
        while improved:
            improved = False
            for i in range(n - 1):
                for j in range(i + 1, n):
                    # 反轉子序列 [i..j]
                    new_seq = sequence[:i] + list(reversed(sequence[i:j + 1])) + sequence[j + 1:]
                    new_cost = SequenceOptimizer._route_transit_cost(
                        drone_position, new_seq, operations
                    )
                    if new_cost < best_cost - 1e-3:
                        sequence = new_seq
                        best_cost = new_cost
                        improved = True

        return [operations[i] for i in sequence]

    @staticmethod
    def calculate_total_path_length(
        drone_position: Tuple[float, float],
        ordered_operations: List[OperationSegment],
    ) -> float:
        """
        計算從無人機位置出發，按給定順序執行所有作業路徑的總長度。

        包含：轉移路徑（transit）+ 各作業路徑長度（op.length_m）。
        飛行方向採貪婪策略：從較近的端點進入。

        參數:
            drone_position: 無人機出發位置 (lat, lon)
            ordered_operations: 已排序的作業路徑列表

        返回:
            總飛行距離（公尺）
        """
        if not ordered_operations:
            return 0.0

        total = 0.0
        current_pos = drone_position
        hav = SequenceOptimizer._haversine

        for op in ordered_operations:
            d_l = hav(current_pos, op.left_point)
            d_r = hav(current_pos, op.right_point)
            if d_l <= d_r:
                transit = d_l
                current_pos = op.right_point
            else:
                transit = d_r
                current_pos = op.left_point
            total += transit + op.length_m

        return total

    # ── 私有輔助方法 ──────────────────────────────────────────────────
    @staticmethod
    def _route_transit_cost(
        drone_position: Tuple[float, float],
        sequence: List[int],
        operations: List[OperationSegment],
    ) -> float:
        """
        計算路線的純轉移成本（不含作業路徑長度，加快 2-opt 評估）。

        因為各 op.length_m 固定不變，最佳化 transit 即等於最佳化 total。
        """
        total = 0.0
        current_pos = drone_position
        hav = SequenceOptimizer._haversine

        for op_i in sequence:
            op = operations[op_i]
            d_l = hav(current_pos, op.left_point)
            d_r = hav(current_pos, op.right_point)
            if d_l <= d_r:
                total += d_l
                current_pos = op.right_point
            else:
                total += d_r
                current_pos = op.left_point

        return total

    @staticmethod
    def _haversine(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """
        計算兩個經緯度點間的 Haversine 距離（公尺）。

        參數:
            p1, p2: (lat, lon) 格式

        返回:
            距離（公尺）
        """
        R = 6371000.0
        lat1 = math.radians(p1[0])
        lat2 = math.radians(p2[0])
        dlat = lat2 - lat1
        dlon = math.radians(p2[1] - p1[1])
        a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        return R * 2.0 * math.asin(math.sqrt(a))


# ==========================================
# 單元測試
# ==========================================
if __name__ == '__main__':
    import random

    def test_sequence_optimizer():
        """測試 SequenceOptimizer，使用雲林地區模擬作業路徑。"""
        print("=" * 55)
        print("test_sequence_optimizer()")
        print("=" * 55)

        # 模擬 8 條 N-S 掃描線（E-W 間距 90m，各長 ~556m）
        lon_to_m = 111111.0 * math.cos(math.radians(23.700))  # ~101853
        ops = []
        for i in range(8):
            lon = 120.420 + (i * 90.0 / lon_to_m)
            ops.append(OperationSegment(
                index=i,
                left_point=(23.700, lon),
                right_point=(23.705, lon),
                length_m=555.6,
            ))

        drone_pos = (23.702, 120.419)  # 從左側中央出發

        # ── 測試最佳化
        optimized = SequenceOptimizer.optimize_sequence(drone_pos, ops)
        assert len(optimized) == len(ops), "輸出長度不符"
        assert set(o.index for o in optimized) == set(o.index for o in ops), "路徑集合不符"

        opt_len = SequenceOptimizer.calculate_total_path_length(drone_pos, optimized)
        print(f"最佳化後順序: {[o.index for o in optimized]}")
        print(f"最佳化總長度: {opt_len:.1f}m")

        # 隨機順序作為基準
        random.seed(42)
        shuffled = ops[:]
        random.shuffle(shuffled)
        rand_len = SequenceOptimizer.calculate_total_path_length(drone_pos, shuffled)
        print(f"隨機順序長度: {rand_len:.1f}m")
        print(f"改善幅度: {(rand_len - opt_len) / rand_len * 100:.1f}%")
        assert opt_len <= rand_len + 1.0, "最佳化後不應比隨機更長"

        # ── 邊界案例：空列表
        empty = SequenceOptimizer.optimize_sequence(drone_pos, [])
        assert empty == []

        # ── 邊界案例：單一路徑
        single = SequenceOptimizer.optimize_sequence(drone_pos, [ops[0]])
        assert len(single) == 1

        print("\n所有斷言通過")

    test_sequence_optimizer()
