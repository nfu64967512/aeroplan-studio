"""
Algorithm 1：基於貪婪算法的多區域 UAV 任務分配器
====================================================
對應論文《Multiple fixed-wing UAVs collaborative coverage 3D path planning
method for complex areas (2025)》Section 2.3.3 / Algorithm 1。

核心公式：
    Eq. 30 — 區域評估指標 J_k（正規化加權成本函數）：
        J_k = u1*(d̄_min,k / Σd̄_min,j) + u2*(n_ops,k / Σn_ops,j)
            + u3*(L_ops,k / ΣL_ops,j)  + u4*(threat_k / Σthreat_j)

    Eq. 31 — 區域應分配的 UAV 數量 l_k（依 J_k 比例分配）：
        l_k = round(N_UAV × J_k / Σ J_j)

貪婪分配邏輯：
    1. 計算所有區域的 J_k（Eq. 30），依 J_k 從大到小排序
    2. 依 Eq. 31 計算各區域應分配的 UAV 數量 l_k
    3. 對每個區域（按 J_k 降序），從可用 UAV 池中選取「進入路徑成本最短」的 l_k 架

雙機型支援：
    FIXED_WING  → 進入路徑成本以 Dubins 曲線估算
    MULTI_ROTOR → 進入路徑成本以 Haversine / Euclidean 距離計算

作者: NCIST_planner_V1
版本: 1.0.0
"""

from __future__ import annotations

import math
import logging
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Callable

import numpy as np

from mission.coverage_path import CoveragePath, OperationSegment
from core.global_planner.mdtsp_solver import UAVState, VehicleType

logger = logging.getLogger(__name__)


# ============================================================
# 結果資料結構
# ============================================================

@dataclass
class AllocationResult:
    """
    多區域 UAV 分配結果。

    Attributes:
        area_assignments : {area_id: [uav_id, ...]} 各區域分配的 UAV ID 列表
        area_jk_values   : {area_id: J_k} 各區域的評估指標
        area_lk_values   : {area_id: l_k} 各區域應分配的 UAV 數
        area_order       : 依 J_k 降序排列的 area_id 列表
        unassigned_uav_ids: 未被分配的 UAV ID（因四捨五入導致）
    """
    area_assignments: Dict[int, List[int]] = field(default_factory=dict)
    area_jk_values: Dict[int, float] = field(default_factory=dict)
    area_lk_values: Dict[int, int] = field(default_factory=dict)
    area_order: List[int] = field(default_factory=list)
    unassigned_uav_ids: List[int] = field(default_factory=list)


# ============================================================
# 距離計算工具（模組內部私有）
# ============================================================

def _haversine_m(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """
    Haversine 公式計算兩個 (lat, lon) 點間的地面距離 [m]。
    適用於多旋翼（直線飛行，無方向約束）。
    """
    R = 6_371_000.0
    lat1, lat2 = math.radians(p1[0]), math.radians(p2[0])
    dlat = lat2 - lat1
    dlon = math.radians(p2[1] - p1[1])
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return R * 2.0 * math.asin(min(1.0, math.sqrt(a)))


def _dubins_entry_estimate(
    from_pos: Tuple[float, float],
    from_heading_deg: float,
    to_pos: Tuple[float, float],
    to_heading_deg: float,
    r_min: float,
) -> float:
    """
    估算固定翼 Dubins 進入路徑長度（解析近似）。

    公式：d_dubins ≈ d_straight + R_min * |Δχ|_rad
          若 d_straight < 2R_min，加入 bulb turn 補償

    Args:
        from_pos / to_pos     : (lat, lon) 位置座標
        from_heading_deg / to_heading_deg: 起終點航向角 [度]
        r_min                 : 最小轉彎半徑 R_min [m]

    Returns:
        估算 Dubins 路徑長度 [m]
    """
    d = _haversine_m(from_pos, to_pos)
    if r_min <= 0 or d <= 0:
        return d

    # 航向差 Δχ（取最短弧）
    dchi = abs(to_heading_deg - from_heading_deg) % 360.0
    if dchi > 180.0:
        dchi = 360.0 - dchi
    dchi_rad = math.radians(dchi)

    extra = r_min * dchi_rad  # 轉彎補償弧長
    # Bulb turn：兩點過近時，需要額外繞圈
    if d < 2.0 * r_min:
        extra += math.pi * r_min - d
        extra = max(0.0, extra)

    return d + extra


def _entry_cost(
    uav: UAVState,
    target_pos: Tuple[float, float],
    target_heading_deg: float,
) -> float:
    """
    計算 UAV 從當前位置到目標進入點的路徑成本。

    機型切換邏輯：
        FIXED_WING  → Dubins 估算（考慮 R_min 與航向差）
        MULTI_ROTOR → Haversine 直線距離（可原地轉向，無需 Dubins）
    """
    if uav.vehicle_type == VehicleType.FIXED_WING and uav.turn_radius > 0:
        return _dubins_entry_estimate(
            uav.position, uav.heading,
            target_pos, target_heading_deg,
            uav.turn_radius,
        )
    return _haversine_m(uav.position, target_pos)


def _scan_heading_deg(op: OperationSegment) -> float:
    """計算作業段的掃描航向角（度，0=北，順時針為正）"""
    dlat = op.right_point[0] - op.left_point[0]
    dlon = op.right_point[1] - op.left_point[1]
    # 考慮緯度對經度距離的縮放（近似）
    lat_mid = (op.left_point[0] + op.right_point[0]) / 2.0
    dlon_m = dlon * math.cos(math.radians(lat_mid))
    return math.degrees(math.atan2(dlon_m, dlat)) % 360.0


def _min_entry_distance_to_area(
    uav: UAVState,
    coverage: CoveragePath,
) -> float:
    """
    計算 UAV 到達某區域任意作業段的最短進入距離（考慮兩端點進入方向）。
    對應論文 Eq. 30 中的 d̄_min,k 分量（UAV i 對區域 k 的最短進入距離）。
    """
    ops = coverage.operations
    if not ops:
        return 0.0

    min_d = math.inf
    for op in ops:
        h = _scan_heading_deg(op)
        # 試左端進入（L→R）與右端進入（R→L）
        d_left = _entry_cost(uav, op.left_point, h)
        d_right = _entry_cost(uav, op.right_point, (h + 180.0) % 360.0)
        min_d = min(min_d, d_left, d_right)
    return min_d


# ============================================================
# UAVTaskAllocator — Algorithm 1 主類別
# ============================================================

class UAVTaskAllocator:
    """
    基於貪婪算法的多區域 UAV 任務分配器（論文 Algorithm 1）。

    設計要點：
    1. 所有 J_k 的四個分量皆經過正規化（除以全域總和），確保量綱一致
    2. 進入路徑成本 d̄_min,k = 所有 UAV 到該區域最短進入距離的均值
    3. 分配時依 J_k 降序（高需求區域優先）
    4. UAV 選取策略：選入路徑成本最低的 l_k 架 UAV

    Args:
        u1 : Eq. 30 中最短進入路徑距離成本的權重（預設 0.25）
        u2 : Eq. 30 中作業路徑數量成本的權重（預設 0.25）
        u3 : Eq. 30 中作業路徑總長度成本的權重（預設 0.25）
        u4 : Eq. 30 中區域威脅等級成本的權重（預設 0.25）

    Example::
        allocator = UAVTaskAllocator(u1=0.25, u2=0.25, u3=0.25, u4=0.25)
        result = allocator.allocate(uav_list, coverage_list)
        print(result.area_assignments)  # {area_id: [uav_ids]}
    """

    def __init__(
        self,
        u1: float = 0.25,
        u2: float = 0.25,
        u3: float = 0.25,
        u4: float = 0.25,
    ) -> None:
        # 驗證權重加總為 1（允許小誤差）
        total = u1 + u2 + u3 + u4
        if not math.isclose(total, 1.0, rel_tol=1e-3):
            logger.warning(
                f"權重加總為 {total:.4f}，論文建議為 1.0。將自動正規化。"
            )
            u1, u2, u3, u4 = u1 / total, u2 / total, u3 / total, u4 / total

        self.u1 = u1  # ω₁：進入路徑距離成本權重
        self.u2 = u2  # ω₂：作業路徑數量成本權重
        self.u3 = u3  # ω₃：作業路徑總長度成本權重
        self.u4 = u4  # ω₄：區域威脅等級成本權重

    # ──────────────────────────────────────────────────────────
    # 公開介面
    # ──────────────────────────────────────────────────────────

    def compute_jk_values(
        self,
        uavs: List[UAVState],
        coverages: List[CoveragePath],
    ) -> np.ndarray:
        """
        計算所有區域的評估指標向量 J = [J_1, J_2, ..., J_K]（論文 Eq. 30）。

        Eq. 30（正規化版本）：
            J_k = u1 * (d̄_min,k / Σ_j d̄_min,j)
                + u2 * (n_ops,k  / Σ_j n_ops,j)
                + u3 * (L_ops,k  / Σ_j L_ops,j)
                + u4 * (threat_k / Σ_j threat_j)

        其中：
            d̄_min,k  = 所有 UAV 到達區域 k 最短進入距離的均值
            n_ops,k  = 區域 k 的作業路徑數量
            L_ops,k  = 區域 k 所有作業路徑的總長度 [m]
            threat_k = 區域 k 的威脅等級（即 coverage.priority）

        Args:
            uavs      : 所有可用 UAV 的狀態列表
            coverages : 所有覆蓋區域列表

        Returns:
            shape (K,) 的 numpy 陣列，代表各區域的 J_k 值
        """
        K = len(coverages)
        n_U = len(uavs)
        if K == 0 or n_U == 0:
            return np.zeros(K, dtype=np.float64)

        # ── 用 numpy 矩陣一次性計算各原始分量 ───────────────
        # 各分量向量 shape: (K,)

        # 分量 1：d̄_min,k — 各 UAV 到區域 k 最短進入距離的均值
        # d_entry_matrix shape: (n_U, K)，d_entry_matrix[i, k] = UAV i 到區域 k 的最短進入距離
        d_entry_matrix = np.zeros((n_U, K), dtype=np.float64)
        for i, uav in enumerate(uavs):
            for k, cov in enumerate(coverages):
                d_entry_matrix[i, k] = _min_entry_distance_to_area(uav, cov)
        # 對所有 UAV 取均值，得到各區域的平均最短進入距離 d̄_min,k
        d_min_vec: np.ndarray = d_entry_matrix.mean(axis=0)  # shape: (K,)

        # 分量 2：n_ops,k — 各區域作業路徑數量
        n_ops_vec: np.ndarray = np.array(
            [len(cov.operations) for cov in coverages], dtype=np.float64
        )

        # 分量 3：L_ops,k — 各區域作業路徑總長度 [m]
        L_ops_vec: np.ndarray = np.array(
            [sum(op.length_m for op in cov.operations) for cov in coverages],
            dtype=np.float64,
        )

        # 分量 4：threat_k — 各區域威脅等級（由 coverage.priority 代理）
        threat_vec: np.ndarray = np.array(
            [cov.priority for cov in coverages], dtype=np.float64
        )

        # ── 正規化各分量（除以全區域總和，避免除零）────────
        def _normalize(v: np.ndarray) -> np.ndarray:
            """將向量正規化為比例（總和為 1），若總和為 0 則返回均等分配"""
            s = v.sum()
            if s < 1e-12:
                return np.ones_like(v) / max(len(v), 1)
            return v / s

        d_min_norm = _normalize(d_min_vec)    # 各分量正規化後的比例
        n_ops_norm = _normalize(n_ops_vec)
        L_ops_norm = _normalize(L_ops_vec)
        threat_norm = _normalize(threat_vec)

        # ── Eq. 30：加權求和 ────────────────────────────────
        # J_k = u1*d̄_norm + u2*n_ops_norm + u3*L_ops_norm + u4*threat_norm
        J: np.ndarray = (
            self.u1 * d_min_norm
            + self.u2 * n_ops_norm
            + self.u3 * L_ops_norm
            + self.u4 * threat_norm
        )

        return J  # shape: (K,)

    def compute_lk_values(
        self,
        jk_values: np.ndarray,
        n_uavs: int,
    ) -> np.ndarray:
        """
        依 Eq. 31 計算各區域應分配的 UAV 數量向量 l = [l_1, ..., l_K]。

        Eq. 31：
            l_k = round(N_UAV × J_k / Σ_j J_j)

        保證：Σ l_k == N_UAV（透過修正循環）
        保證：l_k >= 1（每個區域至少一架 UAV）

        Args:
            jk_values : shape (K,) 的 J_k 向量
            n_uavs    : 可用 UAV 總數 N_UAV

        Returns:
            shape (K,) 的整數陣列，代表各區域的 l_k
        """
        K = len(jk_values)
        if K == 0 or n_uavs == 0:
            return np.zeros(K, dtype=np.int64)

        # Eq. 31：l_k = round(N_UAV × J_k / Σ J_j)
        J_sum = jk_values.sum()
        if J_sum < 1e-12:
            # J_k 全為 0 時均等分配
            raw = np.full(K, n_uavs / K)
        else:
            raw = n_uavs * jk_values / J_sum  # shape: (K,)

        lk = np.maximum(1, np.round(raw)).astype(np.int64)  # 至少分配 1 架

        # ── 修正：確保 Σ l_k == N_UAV ──────────────────────
        # 依 J_k 降序優先調整（高需求區域優先加減）
        order = np.argsort(-jk_values)  # J_k 降序索引
        diff = int(lk.sum()) - n_uavs   # 超出（正）或不足（負）的數量

        if diff > 0:
            # 超出：從低 J_k 區域（反序）逐一減少
            for idx in reversed(order):
                if diff <= 0:
                    break
                if lk[idx] > 1:  # 保持 ≥ 1
                    lk[idx] -= 1
                    diff -= 1
        elif diff < 0:
            # 不足：向高 J_k 區域逐一增加
            for idx in order:
                if diff >= 0:
                    break
                lk[idx] += 1
                diff += 1

        return lk  # shape: (K,), 整數, Σ = N_UAV

    def allocate(
        self,
        uavs: List[UAVState],
        coverages: List[CoveragePath],
    ) -> AllocationResult:
        """
        執行完整的貪婪任務分配（Algorithm 1 主流程）。

        流程：
            Step 1: 計算各區域 J_k（Eq. 30）
            Step 2: 依 J_k 降序排序所有區域
            Step 3: 計算各區域 l_k（Eq. 31）
            Step 4: 對每個區域（J_k 降序），選取進入成本最低的 l_k 架 UAV

        Args:
            uavs      : 所有可用 UAV 列表（含位置、航向、機型）
            coverages : 所有覆蓋區域列表（含作業段、優先級）

        Returns:
            AllocationResult 含分配結果、J_k 值、l_k 值等
        """
        n_U = len(uavs)
        K = len(coverages)

        if n_U == 0 or K == 0:
            logger.warning("UAV 數量或區域數量為 0，無法分配。")
            return AllocationResult()

        # ── Step 1：計算 J_k 向量（Eq. 30）─────────────────
        jk_arr: np.ndarray = self.compute_jk_values(uavs, coverages)

        # ── Step 2：依 J_k 降序排序區域（高需求優先）────────
        area_order_idx: np.ndarray = np.argsort(-jk_arr)  # 降序索引

        # ── Step 3：計算 l_k（Eq. 31）────────────────────────
        lk_arr: np.ndarray = self.compute_lk_values(jk_arr, n_U)

        # ── Step 4：貪婪選取最適合的 l_k 架 UAV ─────────────
        # available_idx：可用 UAV 在 uavs 列表中的索引池
        available_idx: List[int] = list(range(n_U))

        area_assignments: Dict[int, List[int]] = {}
        area_jk_values: Dict[int, float] = {}
        area_lk_values: Dict[int, int] = {}

        for k_idx in area_order_idx:
            cov = coverages[int(k_idx)]
            lk = int(lk_arr[k_idx])
            jk = float(jk_arr[k_idx])

            area_jk_values[cov.area_id] = jk
            area_lk_values[cov.area_id] = lk

            if not available_idx:
                area_assignments[cov.area_id] = []
                logger.warning(f"區域 {cov.area_id}：可用 UAV 已耗盡（J_k={jk:.4f}）")
                continue

            # 對每個可用 UAV 計算其到此區域的「最短進入成本」
            # 取 top-lk 架成本最低的 UAV
            entry_costs: np.ndarray = np.array(
                [_min_entry_distance_to_area(uavs[i], cov) for i in available_idx],
                dtype=np.float64,
            )

            # 按成本升序選取 lk 架（成本最低的優先）
            n_select = min(lk, len(available_idx))
            selected_local_idx: np.ndarray = np.argsort(entry_costs)[:n_select]
            selected_global_idx: List[int] = [available_idx[int(i)] for i in selected_local_idx]

            area_assignments[cov.area_id] = [uavs[i].uav_id for i in selected_global_idx]

            # 從可用池中移除已分配的 UAV
            selected_set = set(selected_global_idx)
            available_idx = [i for i in available_idx if i not in selected_set]

            logger.debug(
                f"區域 {cov.area_id}: J_k={jk:.4f}, l_k={lk}, "
                f"分配 UAV: {area_assignments[cov.area_id]}"
            )

        return AllocationResult(
            area_assignments=area_assignments,
            area_jk_values=area_jk_values,
            area_lk_values=area_lk_values,
            area_order=[coverages[int(i)].area_id for i in area_order_idx],
            unassigned_uav_ids=[uavs[i].uav_id for i in available_idx],
        )

    # ──────────────────────────────────────────────────────────
    # 診斷 / 偵錯工具
    # ──────────────────────────────────────────────────────────

    def explain_allocation(
        self,
        uavs: List[UAVState],
        coverages: List[CoveragePath],
    ) -> str:
        """
        返回詳細的分配診斷報告（供偵錯使用）。

        Args:
            uavs      : UAV 列表
            coverages : 覆蓋區域列表

        Returns:
            多行診斷字串
        """
        jk_arr = self.compute_jk_values(uavs, coverages)
        lk_arr = self.compute_lk_values(jk_arr, len(uavs))
        order = np.argsort(-jk_arr)

        lines = [
            "=" * 60,
            "UAV 任務分配診斷報告（Algorithm 1）",
            f"總 UAV 數：{len(uavs)}，總區域數：{len(coverages)}",
            f"權重 u1={self.u1}, u2={self.u2}, u3={self.u3}, u4={self.u4}",
            "-" * 60,
        ]
        for rank, k_idx in enumerate(order):
            cov = coverages[int(k_idx)]
            lines.append(
                f"排序 #{rank+1} | 區域 {cov.area_id} | "
                f"J_k={jk_arr[k_idx]:.4f} | l_k={lk_arr[k_idx]} | "
                f"作業段={len(cov.operations)} | 威脅={cov.priority:.2f}"
            )
        lines.append("=" * 60)
        return "\n".join(lines)
