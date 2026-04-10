"""
模組三：具備機型適應性的路徑成本計算器
=========================================
對應論文《Multiple fixed-wing UAVs collaborative coverage 3D path planning
method for complex areas》第 2.3 節。

核心設計（策略模式 Strategy Pattern）：
    PathCostCalculator     — 抽象基類，定義計算介面
    MultiRotorCostCalculator — 多旋翼策略：Euclidean 直線距離
    FixedWingCostCalculator  — 固定翼策略：Dubins 曲線最短路徑
    create_cost_calculator() — 工廠函式，依機型自動選擇策略

機型切換邏輯：
    MULTI_ROTOR → 忽略轉彎半徑，直接計算兩點 Euclidean 距離
                  原因：多旋翼可原地懸停與自由轉向，無方向約束
    FIXED_WING  → 必須計算 Dubins 曲線（考慮最小轉彎半徑 R_min 與起終點航向角）
                  原因：固定翼無法停飛，轉向需要有限的轉彎半徑

IDP 整合介面：
    上層 IDPSolver（改進動態規劃）透過 compute_transition_cost() 建立狀態轉移矩陣：
        D[i, k, k'] = UAV i 從作業段 k 轉移到作業段 k' 的最短路徑成本
        E[i, k, k'] = 對應的進入方向（+1=左到右, -1=右到左）

主要數學符號：
    R_min        : 固定翼最小轉彎半徑（由 DCCPPUAVConfig 帶入）
    χ_start/end  : 起點/終點航向角（Dubins 路徑輸入參數）
    d^{entry}    : 進入路徑長度（UAV 起始位置到第一作業段）
    d^{trans}    : 轉移路徑長度（作業段 k 到作業段 k' 的銜接段）

作者: NCIST_planner_V1
版本: 1.0.0
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict, Any

import numpy as np

from core.dccpp.uav_models import DCCPPUAVConfig, UAVType


# ============================================================
# 路徑成本結果資料結構
# ============================================================

@dataclass
class PathCostResult:
    """
    單次路徑轉移的成本計算結果。

    Attributes:
        cost          : 路徑成本（距離 [m] 或正規化後的代價值）
        path_points   : 路徑離散點列表 [(x, y), ...]（固定翼含 Dubins 中間點）
        path_type     : 路徑類型描述字串（'euclidean' / 'dubins_RSR' 等）
        is_feasible   : 是否可行（Dubins 距離過近可能不可行）
        warning       : 警告訊息（不可行或退化時填入）
        entry_direction : 進入方向（+1=左到右進入, -1=右到左進入）
                          對應論文 E_{i,k} 決策矩陣中的值
    """
    cost: float
    path_points: List[Tuple[float, float]] = field(default_factory=list)
    path_type: str = "unknown"
    is_feasible: bool = True
    warning: str = ""
    entry_direction: int = 1  # +1 = L→R, -1 = R→L


@dataclass
class TransitionMatrix:
    """
    IDP 演算法使用的狀態轉移矩陣。

    對應論文 Section 2.3.4 的路徑長度矩陣 D_{i,k} 和決策矩陣 E_{i,k}。

    Attributes:
        uav_id      : UAV ID
        n_segments  : 作業段總數
        cost_matrix : D 矩陣，shape (n_segments+1, n_segments+1, 2)
                      第三維：[0]=從左端進入的成本, [1]=從右端進入的成本
                      cost_matrix[k, k', d] = 從段 k 離開到段 k' 以方向 d 進入的成本
        entry_costs : 各作業段的「進入成本」（UAV 起始位置到該段的距離）
                      shape (n_segments, 2)，第二維同 cost_matrix
    """
    uav_id: int
    n_segments: int
    cost_matrix: np.ndarray   # shape: (n_seg, n_seg, 2)，dtype: float64
    entry_costs: np.ndarray   # shape: (n_seg, 2)，dtype: float64


# ============================================================
# 抽象基類
# ============================================================

class PathCostCalculator(ABC):
    """
    路徑成本計算器抽象基類。

    定義統一的計算介面，讓 IDPSolver 無需知道底層機型差異。
    子類依機型實作 compute_cost()。
    """

    def __init__(self, uav_config: DCCPPUAVConfig) -> None:
        """
        Args:
            uav_config: UAV 完整配置（含機型、R_min、速度等）
        """
        self._cfg = uav_config

    @property
    def uav_config(self) -> DCCPPUAVConfig:
        """返回關聯的 UAV 配置"""
        return self._cfg

    @abstractmethod
    def compute_cost(
        self,
        start_pos: Tuple[float, float],
        start_heading_deg: float,
        end_pos: Tuple[float, float],
        end_heading_deg: float,
    ) -> PathCostResult:
        """
        計算從起點（含航向）到終點（含航向）的路徑成本。

        Args:
            start_pos         : 起點座標 (x, y) [m]
            start_heading_deg : 起點航向角 χ_start [度]，0=正東(x+)，逆時針為正
            end_pos           : 終點座標 (x, y) [m]
            end_heading_deg   : 終點航向角 χ_end [度]

        Returns:
            PathCostResult 含成本值與路徑點
        """

    def compute_transition_matrix(
        self,
        operation_segments: List[Any],
        start_pos: Optional[Tuple[float, float]] = None,
        start_heading_deg: float = 0.0,
    ) -> TransitionMatrix:
        """
        建立 IDP 所需的狀態轉移矩陣。

        枚舉所有「作業段對 (k, k')」與「進入方向 d ∈ {+1, -1}」，
        呼叫 compute_cost() 填入 D 矩陣。

        注意：每個作業段有兩個端點（left_point / right_point），
              對應兩種進入方向（L→R 或 R→L）。
              離開方向決定了下一段的有效起點位置與航向。

        Args:
            operation_segments  : OperationSegment 列表（每段含 left_point、right_point、heading）
            start_pos           : UAV 起始位置（用於計算 entry_costs）
            start_heading_deg   : UAV 初始航向角 [度]

        Returns:
            TransitionMatrix（D 矩陣 + entry 成本向量）
        """
        n = len(operation_segments)
        if n == 0:
            return TransitionMatrix(
                uav_id=self._cfg.uav_id,
                n_segments=0,
                cost_matrix=np.zeros((0, 0, 2), dtype=np.float64),
                entry_costs=np.zeros((0, 2), dtype=np.float64),
            )

        # ── 預先提取各段端點與航向 ─────────────────────────────
        # 每段有兩個入射點：
        #   direction=0 (L→R): 從 left_point 進入，heading = seg_heading
        #   direction=1 (R→L): 從 right_point 進入，heading = seg_heading + 180°
        seg_entry_points: List[List[Tuple[float, float]]] = []
        seg_exit_points: List[List[Tuple[float, float]]] = []
        seg_headings: List[List[float]] = []

        for seg in operation_segments:
            lp = _extract_point(seg.left_point)
            rp = _extract_point(seg.right_point)
            h_lr = _extract_heading(seg)           # L→R 時的掃描方向角
            h_rl = (h_lr + 180.0) % 360.0          # R→L 時相反

            # direction=0: L→R，進入點=left, 離開點=right
            # direction=1: R→L，進入點=right, 離開點=left
            seg_entry_points.append([lp, rp])
            seg_exit_points.append([rp, lp])
            seg_headings.append([h_lr, h_rl])

        # ── D 矩陣：cost_matrix[k, k', d_k'] ────────────────────
        # k  = 前一作業段的索引
        # k' = 下一作業段的索引
        # d_k' = 進入 k' 的方向（0=L→R, 1=R→L）
        # 成本 = 從段 k「離開點」到段 k'「進入點」的路徑轉移成本
        #
        # 前一作業段 k 以最短成本的離開方向決定（本矩陣只存從最佳離開方向到各進入方向）
        # 完整實作應儲存 (k, d_k, k', d_k')，此處簡化為 (k, k', d_k')
        # IDPSolver 會再對 d_k 做動態規劃

        cost_matrix = np.full((n, n, 2), np.inf, dtype=np.float64)

        for k in range(n):
            for d_k in range(2):  # k 的離開方向
                exit_pos = seg_exit_points[k][d_k]
                exit_heading = seg_headings[k][d_k]

                for k_prime in range(n):
                    if k_prime == k:
                        continue  # 不計算自迴圈
                    for d_kp in range(2):  # k' 的進入方向
                        entry_pos = seg_entry_points[k_prime][d_kp]
                        entry_heading = seg_headings[k_prime][d_kp]

                        result = self.compute_cost(
                            exit_pos, exit_heading,
                            entry_pos, entry_heading,
                        )

                        if result.is_feasible:
                            # 儲存最小成本（對 d_k 取 min）
                            old = cost_matrix[k, k_prime, d_kp]
                            cost_matrix[k, k_prime, d_kp] = min(old, result.cost)

        # ── Entry 成本：UAV 起始位置到各作業段 ──────────────────
        entry_costs = np.full((n, 2), np.inf, dtype=np.float64)

        if start_pos is not None:
            for k in range(n):
                for d_k in range(2):
                    entry_pos = seg_entry_points[k][d_k]
                    entry_heading = seg_headings[k][d_k]
                    result = self.compute_cost(
                        start_pos, start_heading_deg,
                        entry_pos, entry_heading,
                    )
                    if result.is_feasible:
                        entry_costs[k, d_k] = result.cost

        return TransitionMatrix(
            uav_id=self._cfg.uav_id,
            n_segments=n,
            cost_matrix=cost_matrix,
            entry_costs=entry_costs,
        )


# ============================================================
# 多旋翼策略：Euclidean 直線距離
# ============================================================

class MultiRotorCostCalculator(PathCostCalculator):
    """
    多旋翼路徑成本計算器。

    由於多旋翼可原地懸停與自由轉向，忽略起終點的航向角約束，
    路徑成本直接等於兩點間的 Euclidean 直線距離。

    計算公式：
        cost = √((x_end - x_start)² + (y_end - y_start)²)

    注意：此計算器也適用於「直升機」或其他可懸停 UAV。
    """

    def compute_cost(
        self,
        start_pos: Tuple[float, float],
        start_heading_deg: float,
        end_pos: Tuple[float, float],
        end_heading_deg: float,
    ) -> PathCostResult:
        """
        計算多旋翼兩點間的 Euclidean 直線距離。

        航向角（start_heading_deg / end_heading_deg）對多旋翼無影響，
        但保留介面參數以維持與固定翼計算器的一致性。

        Args:
            start_pos         : 起點 (x, y) [m]
            start_heading_deg : 起點航向（多旋翼忽略此值）
            end_pos           : 終點 (x, y) [m]
            end_heading_deg   : 終點航向（多旋翼忽略此值）

        Returns:
            PathCostResult，cost = Euclidean 距離，path_points = [起點, 終點]
        """
        # ── Euclidean 距離計算（向量化）────────────────────────
        p_start = np.array(start_pos, dtype=np.float64)
        p_end = np.array(end_pos, dtype=np.float64)
        diff = p_end - p_start
        # 歐氏距離：‖p_end - p_start‖₂
        cost = float(np.linalg.norm(diff))

        return PathCostResult(
            cost=cost,
            path_points=[
                (float(p_start[0]), float(p_start[1])),
                (float(p_end[0]),   float(p_end[1])),
            ],
            path_type="euclidean",
            is_feasible=True,
            warning="",
        )


# ============================================================
# 固定翼策略：Dubins 曲線
# ============================================================

class FixedWingCostCalculator(PathCostCalculator):
    """
    固定翼路徑成本計算器。

    固定翼不能原地轉向，轉移路徑必須滿足最小轉彎半徑 R_min 約束。
    採用 Dubins 曲線計算從 (start_pos, χ_start) 到 (end_pos, χ_end) 的最短路徑。

    Dubins 路徑類型（CSC + CCC 共 6 種，取最短）：
        CSC 類型：RSR, LSL, RSL, LSR
        CCC 類型：RLR, LRL

    若 DubinsTrajectoryGenerator 可用，優先使用完整 6-type 計算；
    否則使用解析近似公式（保障模組獨立性）。

    解析近似公式（估算 Dubins 長度）：
        d_dubins ≈ d_euclidean + R_min * |Δχ|_rad  （航向差補償）
        若 d_euclidean < 2 * R_min，則加入 bulb turn 額外弧長補償

    論文中此計算器用於：
        d^{entry}_{i,k}  : UAV i 從初始位置到作業段 k 的進入路徑成本
        d^{trans}_{i,k,k'}: UAV i 從作業段 k 轉移到作業段 k' 的轉移路徑成本

    Args:
        uav_config    : UAV 配置（需 uav_type=FIXED_WING，min_turning_radius > 0）
        use_exact     : 是否嘗試使用 DubinsTrajectoryGenerator（精確解）；
                        若 False 或導入失敗則改用解析近似
        waypoint_step : Dubins 路徑離散步長 [m]（用於生成 path_points）
    """

    def __init__(
        self,
        uav_config: DCCPPUAVConfig,
        use_exact: bool = True,
        waypoint_step: float = 5.0,
    ) -> None:
        super().__init__(uav_config)

        if uav_config.uav_type != UAVType.FIXED_WING:
            raise ValueError(
                f"FixedWingCostCalculator 只適用於 FIXED_WING 機型，"
                f"實際機型：{uav_config.uav_type}"
            )
        if uav_config.min_turning_radius <= 0:
            raise ValueError(
                f"固定翼 R_min 必須 > 0，實際值：{uav_config.min_turning_radius}"
            )

        self._r_min = uav_config.min_turning_radius  # R_min [m]
        self._waypoint_step = waypoint_step

        # ── 嘗試載入精確 Dubins 計算器 ─────────────────────────
        self._dubins_gen = None
        if use_exact:
            self._dubins_gen = self._try_load_dubins_generator()

    def _try_load_dubins_generator(self):
        """
        嘗試載入 DubinsTrajectoryGenerator，失敗則返回 None（使用解析近似）。

        設計為 lazy-loading 以保持模組的獨立性：
        即使 dubins_trajectory 模組不可用，此計算器仍可提供解析近似結果。
        """
        try:
            from core.trajectory.dubins_trajectory import DubinsTrajectoryGenerator
            from core.base.fixed_wing_constraints import FixedWingConstraints

            # 用 R_min 初始化 FixedWingConstraints（供 DubinsTrajectoryGenerator 使用）
            constraints = FixedWingConstraints(
                min_turn_radius=self._r_min,
                cruise_speed=self._cfg.cruise_speed,
            )
            return DubinsTrajectoryGenerator(constraints)
        except Exception:
            return None

    def compute_cost(
        self,
        start_pos: Tuple[float, float],
        start_heading_deg: float,
        end_pos: Tuple[float, float],
        end_heading_deg: float,
    ) -> PathCostResult:
        """
        計算固定翼從起點（含航向）到終點（含航向）的 Dubins 最短路徑成本。

        優先使用精確 Dubins 計算（DubinsTrajectoryGenerator）；
        若無法使用則退回解析近似。

        Args:
            start_pos         : 起點 (x, y) [m]
            start_heading_deg : 起點航向角 χ_start [度]（固定翼方向約束輸入）
            end_pos           : 終點 (x, y) [m]
            end_heading_deg   : 終點航向角 χ_end [度]（固定翼方向約束輸入）

        Returns:
            PathCostResult：
                cost       = Dubins 曲線長度 [m]
                path_points = 曲線離散採樣點（含起終點）
                path_type  = Dubins 路徑類型字串（如 'dubins_RSR'）
        """
        if self._dubins_gen is not None:
            return self._compute_exact_dubins(
                start_pos, start_heading_deg,
                end_pos, end_heading_deg,
            )
        else:
            return self._compute_approximate_dubins(
                start_pos, start_heading_deg,
                end_pos, end_heading_deg,
            )

    def _compute_exact_dubins(
        self,
        start_pos: Tuple[float, float],
        start_heading_deg: float,
        end_pos: Tuple[float, float],
        end_heading_deg: float,
    ) -> PathCostResult:
        """
        使用 DubinsTrajectoryGenerator 計算精確 Dubins 路徑。

        座標系：Dubins 使用 NED（x=North, y=East）；
        本模組使用 ENU（x=East, y=North）。
        轉換規則：ENU(x, y) → NED(x=y_enu, y=x_enu)
        """
        try:
            from core.trajectory.dubins_trajectory import Pose3D

            # ENU (x=East, y=North) → NED (x=North, y=East)
            start_pose = Pose3D(
                x=start_pos[1],           # NED x = ENU y（North）
                y=start_pos[0],           # NED y = ENU x（East）
                z=self._cfg.altitude,
                heading_deg=start_heading_deg,
            )
            end_pose = Pose3D(
                x=end_pos[1],
                y=end_pos[0],
                z=self._cfg.altitude,
                heading_deg=end_heading_deg,
            )

            dubins_path = self._dubins_gen.calculate_path(start_pose, end_pose)

            if not dubins_path.is_feasible:
                # 精確計算不可行，退回解析近似
                return self._compute_approximate_dubins(
                    start_pos, start_heading_deg,
                    end_pos, end_heading_deg,
                )

            # 取得離散航點
            poses = self._dubins_gen.generate_waypoints(dubins_path, self._waypoint_step)

            # NED → ENU 轉換
            path_points = [
                (float(p.y), float(p.x))  # ENU(x=East=NED.y, y=North=NED.x)
                for p in poses
            ]

            path_type_str = (
                f"dubins_{dubins_path.path_type.value}"
                if dubins_path.path_type is not None
                else "dubins"
            )

            return PathCostResult(
                cost=float(dubins_path.total_length),
                path_points=path_points,
                path_type=path_type_str,
                is_feasible=True,
                warning=dubins_path.warning,
            )

        except Exception as e:
            # 精確計算發生例外，退回解析近似
            result = self._compute_approximate_dubins(
                start_pos, start_heading_deg,
                end_pos, end_heading_deg,
            )
            result.warning = f"Dubins 精確計算失敗（{e}），使用解析近似"
            return result

    def _compute_approximate_dubins(
        self,
        start_pos: Tuple[float, float],
        start_heading_deg: float,
        end_pos: Tuple[float, float],
        end_heading_deg: float,
    ) -> PathCostResult:
        """
        解析近似估算 Dubins 路徑長度。

        對應 mdtsp_solver.py 中的 _dubins_estimate() 函式邏輯，
        此處做了更精確的多路徑類型展開（RSR / LSL 近似）。

        估算公式：
            d_dubins ≈ d_euclidean + R_min * |Δχ|_rad
            若 d_euclidean < 2 * R_min：
                加入 bulb turn 補償 = π * R_min - d_euclidean
        """
        p_start = np.array(start_pos, dtype=np.float64)
        p_end = np.array(end_pos, dtype=np.float64)
        r = self._r_min

        # 直線距離
        d = float(np.linalg.norm(p_end - p_start))

        # 航向差 Δχ（取最短弧，限制在 [0, 180]）
        delta_chi = abs(end_heading_deg - start_heading_deg) % 360.0
        if delta_chi > 180.0:
            delta_chi = 360.0 - delta_chi
        delta_chi_rad = math.radians(delta_chi)

        # 基礎轉彎補償弧長 ≈ R_min * |Δχ|
        turn_compensation = r * delta_chi_rad

        # Bulb turn 補償（兩點距離 < 2R 時無法直接到達，需要額外繞圈）
        bulb_compensation = 0.0
        if d < 2.0 * r:
            # 最短 bulb turn 近似：π * R_min（最差情況為完整半圓）
            bulb_compensation = math.pi * r - d
            bulb_compensation = max(0.0, bulb_compensation)

        dubins_length = d + turn_compensation + bulb_compensation

        # 生成簡化路徑點（S 型插值，用於視覺化）
        n_pts = max(3, int(d / self._waypoint_step)) if d > 0 else 3
        path_points: List[Tuple[float, float]] = []
        for i in range(n_pts + 1):
            t = i / n_pts
            # S 型插值（Smoothstep）：3t² - 2t³
            s = 3.0 * t * t - 2.0 * t * t * t
            pt = p_start + s * (p_end - p_start)
            path_points.append((float(pt[0]), float(pt[1])))

        return PathCostResult(
            cost=float(dubins_length),
            path_points=path_points,
            path_type="dubins_approx",
            is_feasible=True,
            warning="",
        )


# ============================================================
# 工廠函式：依機型建立計算器
# ============================================================

def create_cost_calculator(
    uav_config: DCCPPUAVConfig,
    use_exact_dubins: bool = True,
    dubins_waypoint_step: float = 5.0,
) -> PathCostCalculator:
    """
    路徑成本計算器工廠函式。

    依據 uav_config.uav_type 自動選擇對應的計算器策略：
        MULTI_ROTOR → MultiRotorCostCalculator（Euclidean 直線距離）
        FIXED_WING  → FixedWingCostCalculator（Dubins 曲線）

    Args:
        uav_config          : UAV 完整配置
        use_exact_dubins    : 固定翼是否使用精確 Dubins（需要 core.trajectory 可用）
        dubins_waypoint_step: Dubins 路徑離散步長 [m]

    Returns:
        對應機型的 PathCostCalculator 子類實例

    Example:
        # 建立多旋翼計算器
        mr_uav = make_multi_rotor_uav(uav_id=1, position=(0, 0))
        mr_calc = create_cost_calculator(mr_uav)
        result = mr_calc.compute_cost((0,0), 0.0, (100,100), 90.0)

        # 建立固定翼計算器
        fw_uav = make_fixed_wing_uav(uav_id=2, position=(0,0), min_turning_radius=50.0)
        fw_calc = create_cost_calculator(fw_uav)
        result = fw_calc.compute_cost((0,0), 0.0, (200,0), 0.0)
    """
    if uav_config.uav_type == UAVType.MULTI_ROTOR:
        # ── 多旋翼：Euclidean 直線距離，忽略航向約束 ──────────
        return MultiRotorCostCalculator(uav_config)

    elif uav_config.uav_type == UAVType.FIXED_WING:
        # ── 固定翼：Dubins 曲線，考慮 R_min 與航向約束 ─────────
        return FixedWingCostCalculator(
            uav_config,
            use_exact=use_exact_dubins,
            waypoint_step=dubins_waypoint_step,
        )

    else:
        raise ValueError(
            f"未知的機型：{uav_config.uav_type}。"
            f"請使用 UAVType.MULTI_ROTOR 或 UAVType.FIXED_WING。"
        )


# ============================================================
# IDP 整合工具：建立多 UAV 的轉移矩陣字典
# ============================================================

def build_transition_matrices(
    uav_configs: List[DCCPPUAVConfig],
    operation_segments: List[Any],
    use_exact_dubins: bool = True,
) -> Dict[int, TransitionMatrix]:
    """
    為所有 UAV 批次建立狀態轉移矩陣，供 IDPSolver 使用。

    Args:
        uav_configs        : 所有 UAV 的配置列表
        operation_segments : 作業段列表（OperationSegment）
        use_exact_dubins   : 是否使用精確 Dubins 計算

    Returns:
        {uav_id: TransitionMatrix} 字典
    """
    matrices: Dict[int, TransitionMatrix] = {}

    for cfg in uav_configs:
        calculator = create_cost_calculator(cfg, use_exact_dubins)
        matrix = calculator.compute_transition_matrix(
            operation_segments,
            start_pos=cfg.position,
            start_heading_deg=cfg.initial_heading_deg,
        )
        matrices[cfg.uav_id] = matrix

    return matrices


# ============================================================
# 私有工具函式
# ============================================================

def _extract_point(point_data: Any) -> Tuple[float, float]:
    """
    從多種格式中提取 2D 座標點。

    支援格式：
        - (lat, lon) tuple/list
        - np.ndarray [x, y, ...]
        - 物件有 .x/.y 或 .lat/.lon 屬性
    """
    if isinstance(point_data, (tuple, list)) and len(point_data) >= 2:
        return (float(point_data[0]), float(point_data[1]))
    if isinstance(point_data, np.ndarray) and point_data.ndim >= 1:
        return (float(point_data[0]), float(point_data[1] if len(point_data) > 1 else 0))
    if hasattr(point_data, 'x') and hasattr(point_data, 'y'):
        return (float(point_data.x), float(point_data.y))
    if hasattr(point_data, 'lat') and hasattr(point_data, 'lon'):
        return (float(point_data.lat), float(point_data.lon))
    raise TypeError(f"無法從 {type(point_data)} 提取 2D 座標點")


def _extract_heading(segment: Any) -> float:
    """
    從作業段提取掃描方向航向角 [度]。

    支援格式：
        - segment.heading（度）
        - segment.heading_deg（度）
        - 自動依 left/right 端點計算
    """
    if hasattr(segment, 'heading_deg'):
        return float(segment.heading_deg)
    if hasattr(segment, 'heading'):
        return float(segment.heading)

    # 依兩端點計算航向角
    try:
        lp = _extract_point(segment.left_point)
        rp = _extract_point(segment.right_point)
        dx = rp[0] - lp[0]
        dy = rp[1] - lp[1]
        return math.degrees(math.atan2(dy, dx)) % 360.0
    except Exception:
        return 0.0
