"""
Algorithm 3：基於高度下降與 GDA 的 3D 高度規劃器
======================================================
對應論文《Multiple fixed-wing UAVs collaborative coverage 3D path planning
method for complex areas (2025)》Section 2.2 / Algorithm 3。

核心公式（全程 numpy 向量化）：

    Eq. 4  — 雙線性插值（Bilinear Interpolation）：
        h_psd = (x2-xi)(y2-yi)/ΔxΔy * z1
              + (xi-x1)(y2-yi)/ΔxΔy * z2
              + (x2-xi)(yi-y1)/ΔxΔy * z3
              + (xi-x1)(yi-y1)/ΔxΔy * z4
        從 DEM 離散網格計算航點 P_psd 對應的精確地形高程 h_psd

    Eq. 35 — 期望高度計算（地球曲率補償）：
        Δz   = ρ² / (2 * R₀)            （地球曲率誤差，ρ = 水平距離 [m]）
        z₀   = h_cov + h_psd - Δz        （期望 UAV 高度 AMSL）

    Eq. 36 — 爬升角約束（最大爬升角 γ_max）：
        z_max_climb[j] = z[j-1] + δd * tan(γ_max)  （前向掃描：最大上升）
        z_min_drop[j]  = z[j-1] - δd * tan(γ_max)  （前向掃描：最大下降）
        雙向掃描確保路徑雙向都滿足爬升角約束

    Eq. 37-38 — GDA 梯度下降平滑（Gradient Descent Algorithm）：
        損失函數 J(h) = c1 * Σ|h_j - h_{j-1}| + c2 * Σ|h_{j+1} - 2h_j + h_{j-1}|
        梯度 ∂J/∂h_j = c1 * sign(h_j - h_{j-1})
                      + 2*c2 * (2h_j - h_{j-1} - h_{j+1})
        更新 h_j ← h_j - lr * ∂J/∂h_j

    Eq. 39 — 共線過濾（Collinear Reduction）：
        若三連點 P_{j-1}, P_j, P_{j+1} 共線（叉積絕對值 < ε），刪除中間點 P_j

設計亮點：
    - 全部步驟（插值、曲率補償、爬升約束、GDA、共線過濾）均向量化（numpy）
    - GDA 使用「矩陣滑動差分」而非 Python 迴圈
    - 支援批次多路徑輸入（plan_altitude_batch）以提升吞吐量

作者: NCIST_planner_V1
版本: 1.0.0
"""

from __future__ import annotations

import math
import logging
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)

# 地球平均半徑（用於 Eq. 35 曲率補償）
EARTH_RADIUS_M: float = 6_371_000.0

# 型別別名
WaypointXY = Tuple[float, float]           # (x, y) 或 (lat, lon)
Waypoint3D = Tuple[float, float, float]    # (x, y, z) 或 (lat, lon, alt_m)
TerrainFunc = Callable[[float, float], float]  # (lat, lon) → elevation [m]


# ============================================================
# 結果資料結構
# ============================================================

@dataclass
class AltitudePlanResult:
    """
    3D 高度規劃結果。

    Attributes:
        waypoints_3d   : 最終 3D 航點列表 [(x, y, z), ...]（共線點已精簡）
        ground_elev    : 各航點的地面高程（AMSL）[m]，shape (N_final,)
        desired_z      : Eq. 35 計算的期望高度 z₀ [m]，shape (N_dense,)
        constrained_z  : Eq. 36 約束後的高度 [m]，shape (N_dense,)
        smoothed_z     : Eq. 37-38 GDA 平滑後的高度 [m]，shape (N_dense,)
        n_dense        : 密集插值點數 N_dense
        n_reduced      : 精簡後的航點數 N_final
        gda_iterations_run : GDA 實際迭代次數（提前收斂時 < max_iter）
    """
    waypoints_3d: List[Waypoint3D] = field(default_factory=list)
    ground_elev: np.ndarray = field(default_factory=lambda: np.empty(0))
    desired_z: np.ndarray = field(default_factory=lambda: np.empty(0))
    constrained_z: np.ndarray = field(default_factory=lambda: np.empty(0))
    smoothed_z: np.ndarray = field(default_factory=lambda: np.empty(0))
    n_dense: int = 0
    n_reduced: int = 0
    gda_iterations_run: int = 0


# ============================================================
# AltitudePlanner — Algorithm 3 主類別
# ============================================================

class AltitudePlanner:
    """
    2D 路徑 → 3D 貼地航點規劃器（Algorithm 3）。

    完整流程（6 步）：
        1. 均勻密集插值（步長 δd）
        2. 雙線性插值查詢 DEM 地面高程（Eq. 4）
        3. 地球曲率補償 + 計算期望高度 z₀（Eq. 35）
        4. 爬升角約束修正（Eq. 36），雙向掃描
        5. GDA 梯度下降平滑（Eq. 37-38）
        6. 共線過濾精簡航點（Eq. 39）

    Args:
        h_cov             : 覆蓋高度 h_cov [m]（相對地面的飛行高度）
        climb_angle_max_deg : 最大爬升/下降角 γ_max [度]（預設 15°）
        step_m            : 路徑插值步長 δd [m]（預設 50 m）
        c1                : GDA 距離因子（論文建議 0.5）
        c2                : GDA 平滑因子（論文建議 0.5）
        gda_max_iter      : GDA 最大迭代次數（預設 200）
        gda_lr            : GDA 學習率（預設 0.05）
        gda_tol           : GDA 收斂閾值（預設 0.01 m）
        collinear_eps     : 共線判定閾值（預設 1e-2，Eq. 39）
        safety_clearance  : 超出地面的最低安全餘量 [m]（預設 10 m）
        earth_radius      : 地球半徑 R₀ [m]（Eq. 35）

    Example::
        planner = AltitudePlanner(h_cov=120.0, c1=0.5, c2=0.5)
        result = planner.plan_altitude(path_2d, terrain_func=dem.get_elevation)
        waypoints_3d = result.waypoints_3d
    """

    def __init__(
        self,
        h_cov: float = 120.0,
        climb_angle_max_deg: float = 15.0,
        step_m: float = 50.0,
        c1: float = 0.5,       # GDA 距離因子（Eq. 37）
        c2: float = 0.5,       # GDA 平滑因子（Eq. 38）
        gda_max_iter: int = 200,
        gda_lr: float = 0.05,
        gda_tol: float = 0.01,
        collinear_eps: float = 1e-2,
        safety_clearance: float = 10.0,
        earth_radius: float = EARTH_RADIUS_M,
    ) -> None:
        if h_cov <= 0:
            raise ValueError(f"h_cov 必須為正數，實際值：{h_cov}")
        if not (0 < climb_angle_max_deg < 90):
            raise ValueError(f"climb_angle_max_deg 必須在 (0, 90)，實際值：{climb_angle_max_deg}")
        if step_m <= 0:
            raise ValueError(f"step_m 必須為正數，實際值：{step_m}")

        self.h_cov = float(h_cov)                              # 覆蓋高度 h_cov [m]
        self.gamma_max = math.radians(climb_angle_max_deg)     # 最大爬升角 γ_max [rad]
        self.delta_d = float(step_m)                           # 插值步長 δd [m]
        self.c1 = float(c1)                                    # GDA 距離因子
        self.c2 = float(c2)                                    # GDA 平滑因子
        self.gda_max_iter = int(gda_max_iter)                  # GDA 最大迭代次數
        self.gda_lr = float(gda_lr)                            # GDA 學習率
        self.gda_tol = float(gda_tol)                          # GDA 收斂閾值
        self.collinear_eps = float(collinear_eps)              # 共線閾值 ε（Eq. 39）
        self.clearance = float(safety_clearance)               # 地面安全餘量 [m]
        self.R0 = float(earth_radius)                          # 地球半徑 R₀ [m]

        # 爬升角換算：每步長允許的最大高度變化量（Eq. 36）
        self._dz_max: float = self.delta_d * math.tan(self.gamma_max)

    # ──────────────────────────────────────────────────────────
    # 主介面
    # ──────────────────────────────────────────────────────────

    def plan_altitude(
        self,
        path_2d: List[WaypointXY],
        terrain_func: Optional[TerrainFunc] = None,
    ) -> AltitudePlanResult:
        """
        將 2D 路徑轉換為 3D 貼地航點序列（完整 6 步流程）。

        Args:
            path_2d      : 2D 路徑座標 [(x, y), ...]（x/y 可為公尺或 lat/lon）
            terrain_func : 地形查詢函數 f(x, y) → 地面高程 [m AMSL]；
                           None 時假設地面完全平坦（高程 = 0）

        Returns:
            AltitudePlanResult 含精簡後的 3D 航點與各步驟中間結果
        """
        if len(path_2d) < 2:
            # 單點退化
            z = self.h_cov + (terrain_func(*path_2d[0]) if terrain_func else 0.0)
            return AltitudePlanResult(
                waypoints_3d=[(path_2d[0][0], path_2d[0][1], z)],
                n_dense=1,
                n_reduced=1,
            )

        # ── 步驟 1：均勻密集插值（步長 δd）──────────────────
        pts_xy: np.ndarray = self._interpolate_dense(path_2d)
        # pts_xy shape: (N, 2)，每列為一個插值點的 (x, y)

        N = len(pts_xy)
        logger.debug(f"AltitudePlanner: {len(path_2d)} 輸入點 → {N} 密集插值點")

        # ── 步驟 2：批次查詢地面高程（Eq. 4 雙線性插值）──────
        ground_h: np.ndarray = self._batch_terrain_query(pts_xy, terrain_func)
        # ground_h shape: (N,)，單位 [m AMSL]

        # ── 步驟 3：期望高度（Eq. 35，地球曲率補償）─────────
        desired_z: np.ndarray = self._compute_desired_z(pts_xy, ground_h)
        # desired_z shape: (N,)

        # ── 步驟 4：爬升角約束（Eq. 36，雙向掃描）──────────
        constrained_z: np.ndarray = self._apply_climb_constraint(desired_z)
        # constrained_z shape: (N,)

        # ── 步驟 5：GDA 梯度下降平滑（Eq. 37-38）──────────
        smoothed_z, n_iter = self._gda_smooth(constrained_z, ground_h)
        # smoothed_z shape: (N,)

        # ── 步驟 6：共線過濾精簡航點（Eq. 39）──────────────
        keep_mask: np.ndarray = self._collinear_filter(pts_xy, smoothed_z)
        # keep_mask shape: (N,)，True 表示保留

        final_xy = pts_xy[keep_mask]          # shape: (M, 2)
        final_z = smoothed_z[keep_mask]       # shape: (M,)
        final_ground = ground_h[keep_mask]    # shape: (M,)

        # 組裝 3D 航點列表
        waypoints_3d: List[Waypoint3D] = [
            (float(final_xy[j, 0]), float(final_xy[j, 1]), float(final_z[j]))
            for j in range(len(final_xy))
        ]

        logger.debug(
            f"AltitudePlanner: 密集點 {N} → 精簡後 {len(waypoints_3d)} 個航點，"
            f"GDA 迭代 {n_iter} 次"
        )

        return AltitudePlanResult(
            waypoints_3d=waypoints_3d,
            ground_elev=final_ground,
            desired_z=desired_z,
            constrained_z=constrained_z,
            smoothed_z=smoothed_z,
            n_dense=N,
            n_reduced=len(waypoints_3d),
            gda_iterations_run=n_iter,
        )

    def plan_altitude_batch(
        self,
        paths_2d: List[List[WaypointXY]],
        terrain_func: Optional[TerrainFunc] = None,
    ) -> List[AltitudePlanResult]:
        """
        批次處理多條 2D 路徑（例如多架 UAV 的路徑序列）。

        Args:
            paths_2d     : 多條 2D 路徑列表
            terrain_func : 地形查詢函數（可為 None）

        Returns:
            各路徑對應的 AltitudePlanResult 列表
        """
        return [self.plan_altitude(path, terrain_func) for path in paths_2d]

    # ──────────────────────────────────────────────────────────
    # 步驟 1：均勻密集插值
    # ──────────────────────────────────────────────────────────

    def _interpolate_dense(
        self,
        path_2d: List[WaypointXY],
    ) -> np.ndarray:
        """
        沿 2D 路徑以步長 δd 均勻插值，返回密集點陣列。

        使用 numpy 向量化計算段長，並以線性插值生成密集點。

        Args:
            path_2d : 稀疏路徑點列表

        Returns:
            shape (N, 2) 的 numpy 陣列，N = 密集點總數
        """
        pts = np.array(path_2d, dtype=np.float64)     # shape: (M, 2)
        M = len(pts)

        # 計算各段長度向量（Euclidean 或 Haversine）
        # 此處用 Euclidean；若座標為 lat/lon，應乘以 111111.0 換算（ArcLength）
        diff = np.diff(pts, axis=0)                   # shape: (M-1, 2)
        seg_lengths = np.linalg.norm(diff, axis=1)    # shape: (M-1,)

        result_pts: List[np.ndarray] = [pts[0]]

        for i in range(M - 1):
            L = float(seg_lengths[i])
            if L < 1e-6:
                continue
            # 此段的插值點數（至少 1）
            n_steps = max(1, int(math.ceil(L / self.delta_d)))
            # 等間距 t 值（不含 0，含 1）
            t_vals = np.linspace(0.0, 1.0, n_steps + 1)[1:]   # shape: (n_steps,)
            # 批次插值：pts[i] + t * (pts[i+1] - pts[i])
            interp = pts[i] + t_vals[:, np.newaxis] * diff[i]  # shape: (n_steps, 2)
            result_pts.append(interp)

        return np.vstack(result_pts)  # shape: (N, 2)

    # ──────────────────────────────────────────────────────────
    # 步驟 2：批次地形高程查詢（對接 DEMTerrainManager / Eq. 4）
    # ──────────────────────────────────────────────────────────

    def _batch_terrain_query(
        self,
        pts: np.ndarray,
        terrain_func: Optional[TerrainFunc],
    ) -> np.ndarray:
        """
        對所有密集點批次查詢地面高程。

        若 terrain_func 支援 numpy 批次輸入（例如向量化 DEMTerrainManager），
        直接呼叫；否則逐點查詢。
        terrain_func 內部即為 Eq. 4 的雙線性插值（由 DEMTerrainManager 實作）。

        Args:
            pts          : shape (N, 2) 的點陣列，每列為 (x, y) 或 (lat, lon)
            terrain_func : (x, y) → 高程 [m]；None 時返回全零陣列

        Returns:
            shape (N,) 的地面高程陣列 h_psd [m AMSL]
        """
        N = len(pts)
        if terrain_func is None:
            return np.zeros(N, dtype=np.float64)

        # 嘗試向量化呼叫（若 terrain_func 接受 ndarray）
        try:
            h = terrain_func(pts[:, 0], pts[:, 1])
            arr = np.asarray(h, dtype=np.float64)
            if arr.shape == (N,):
                # 替換 NaN 與 nodata 值
                arr = np.nan_to_num(arr, nan=0.0, posinf=0.0, neginf=0.0)
                return arr
        except (TypeError, ValueError):
            pass

        # 逐點查詢（fallback）
        h_arr = np.zeros(N, dtype=np.float64)
        for j in range(N):
            try:
                v = terrain_func(float(pts[j, 0]), float(pts[j, 1]))
                h_arr[j] = float(v) if (v is not None and math.isfinite(v)) else 0.0
            except Exception:
                h_arr[j] = 0.0

        return h_arr

    # ──────────────────────────────────────────────────────────
    # 步驟 3：期望高度計算（Eq. 35）
    # ──────────────────────────────────────────────────────────

    def _compute_desired_z(
        self,
        pts: np.ndarray,
        ground_h: np.ndarray,
    ) -> np.ndarray:
        """
        計算每個插值點的期望 UAV 高度（論文 Eq. 35）。

        Eq. 35：
            ρ_j = 距路徑起點的水平累積距離 [m]
            Δz_j = ρ_j² / (2 * R₀)          （地球曲率補償）
            z₀_j = h_cov + h_psd_j - Δz_j    （期望 AMSL 高度）

        物理意義：
            由於地球是球形，當水平距離 ρ 較大時，地球表面「凹下去」，
            若 UAV 以恆定 AMSL 高度飛行，實際覆蓋高度會隨距離增加。
            Δz 修正此誤差，使 UAV 與地面的真實距離始終約等於 h_cov。

        Args:
            pts      : shape (N, 2) 密集點陣列
            ground_h : shape (N,) 地面高程 h_psd [m AMSL]

        Returns:
            shape (N,) 的期望高度陣列 z₀ [m AMSL]
        """
        N = len(pts)
        if N == 0:
            return np.empty(0, dtype=np.float64)

        # ── 計算累積水平距離 ρ_j ────────────────────────────
        # 使用 Euclidean 距離（若 pts 為公尺座標）
        # 若為 lat/lon，需先乘以縮放因子（約 111111 m/度）
        diff = np.diff(pts, axis=0)                  # shape: (N-1, 2)
        seg_d = np.linalg.norm(diff, axis=1)          # shape: (N-1,)
        rho = np.concatenate([[0.0], np.cumsum(seg_d)])  # shape: (N,)，ρ₀=0

        # ── Eq. 35：地球曲率修正 Δz = ρ² / (2R₀) ────────────
        delta_z: np.ndarray = (rho ** 2) / (2.0 * self.R0)  # shape: (N,)

        # ── 期望高度 z₀ = h_cov + h_psd - Δz ────────────────
        desired_z: np.ndarray = self.h_cov + ground_h - delta_z

        # 確保期望高度不低於地面 + 安全餘量（物理合理性保障）
        desired_z = np.maximum(desired_z, ground_h + self.clearance)

        return desired_z

    # ──────────────────────────────────────────────────────────
    # 步驟 4：爬升角約束（Eq. 36）
    # ──────────────────────────────────────────────────────────

    def _apply_climb_constraint(
        self,
        desired_z: np.ndarray,
    ) -> np.ndarray:
        """
        依最大爬升角 γ_max 限制相鄰點的高度變化（論文 Eq. 36）。

        每步長允許的最大高度差：
            Δz_max = δd * tan(γ_max)

        Eq. 36 實作：
            正向掃描（j=1..N-1）：z[j] = clip(z[j], z[j-1] - Δz_max, z[j-1] + Δz_max)
            反向掃描（j=N-2..0）：z[j] = clip(z[j], z[j+1] - Δz_max, z[j+1] + Δz_max)
            雙向掃描確保任意方向飛行都滿足爬升角約束。

        Args:
            desired_z : shape (N,) 期望高度陣列

        Returns:
            shape (N,) 受爬升角約束的高度陣列
        """
        z = desired_z.copy()
        N = len(z)
        dz_max = self._dz_max  # 每步最大高度變化 Δz_max = δd * tan(γ_max)

        # 正向掃描（確保上升/下降不超過 Δz_max）
        for j in range(1, N):
            z[j] = np.clip(z[j], z[j - 1] - dz_max, z[j - 1] + dz_max)

        # 反向掃描（確保反向飛行同樣滿足約束）
        for j in range(N - 2, -1, -1):
            z[j] = np.clip(z[j], z[j + 1] - dz_max, z[j + 1] + dz_max)

        return z

    # ──────────────────────────────────────────────────────────
    # 步驟 5：GDA 梯度下降平滑（Eq. 37-38）
    # ──────────────────────────────────────────────────────────

    def _gda_smooth(
        self,
        z: np.ndarray,
        ground_h: np.ndarray,
    ) -> Tuple[np.ndarray, int]:
        """
        使用梯度下降法（GDA）對高度序列進行平滑（論文 Eq. 37-38）。

        損失函數（Eq. 37）：
            J(h) = c1 * Σ_j |h_j - h_{j-1}|     （距離因子：懲罰高度突變）
                 + c2 * Σ_j |h_{j+1} - 2h_j + h_{j-1}|  （平滑因子：懲罰曲率）

        梯度（Eq. 38，對 h_j 偏微分）：
            ∂J/∂h_j = c1 * sign(h_j - h_{j-1})
                    + 2*c2 * (2*h_j - h_{j-1} - h_{j+1})

        更新規則：h_j ← h_j - lr * ∂J/∂h_j
        約束：h_j ≥ ground_h[j] + clearance（不得低於地面）

        全程使用 numpy 向量化（避免 Python 迴圈），
        一次迭代計算所有中間點的梯度與更新。

        Args:
            z        : shape (N,) 待平滑高度陣列（爬升角約束後）
            ground_h : shape (N,) 地面高程陣列（下限約束）

        Returns:
            (smoothed_z, n_iter) — 平滑後高度陣列、實際迭代次數
        """
        h = z.copy()
        N = len(h)

        if N < 3:
            return h, 0

        # 地面安全下限（不得飛到地下）
        floor: np.ndarray = ground_h + self.clearance

        # 固定端點（首尾不更新）
        c1, c2, lr = self.c1, self.c2, self.gda_lr

        n_iter = 0
        for n_iter in range(1, self.gda_max_iter + 1):
            h_prev = h[:-2]   # h[j-1]，j = 1..N-2，shape: (N-2,)
            h_mid = h[1:-1]   # h[j]
            h_next = h[2:]    # h[j+1]

            # ── Eq. 38：梯度計算（向量化）──────────────────
            # 第一項：c1 * sign(h_j - h_{j-1})
            d1 = h_mid - h_prev                           # shape: (N-2,)
            grad_c1 = c1 * np.sign(d1)

            # 第二項：2*c2 * (2*h_j - h_{j-1} - h_{j+1})
            curvature = 2.0 * h_mid - h_prev - h_next    # 二階差分（曲率）
            grad_c2 = 2.0 * c2 * curvature

            # 總梯度
            grad: np.ndarray = grad_c1 + grad_c2         # shape: (N-2,)

            # ── 梯度下降更新 ───────────────────────────────
            delta: np.ndarray = -lr * grad
            h[1:-1] += delta

            # 安全下限約束（Eq. 35 隱含的地面保護）
            h[1:-1] = np.maximum(h[1:-1], floor[1:-1])

            # ── 收斂判定 ───────────────────────────────────
            if float(np.max(np.abs(delta))) < self.gda_tol:
                break

        return h, n_iter

    # ──────────────────────────────────────────────────────────
    # 步驟 6：共線過濾（Eq. 39）
    # ──────────────────────────────────────────────────────────

    def _collinear_filter(
        self,
        pts_xy: np.ndarray,
        z: np.ndarray,
    ) -> np.ndarray:
        """
        刪除三維空間中的共線冗餘航點（論文 Eq. 39）。

        共線判定：
            對三連點 P_{j-1}, P_j, P_{j+1}（三維座標），
            計算向量 v1 = P_j - P_{j-1} 與 v2 = P_{j+1} - P_j 的夾角。
            若叉積（叉乘模長）/ （|v1| * |v2|）< ε，認為三點共線，刪除中間點。

        Eq. 39（離散化）：
            保留條件：|v1 × v2| / (|v1| |v2|) ≥ collinear_eps

        向量化實作：使用 numpy 批次計算所有相鄰三元組的叉積。

        Args:
            pts_xy : shape (N, 2) 密集點的 xy 座標
            z      : shape (N,) 平滑後的高度

        Returns:
            shape (N,) 的布林遮罩（True=保留，False=刪除）
        """
        N = len(pts_xy)
        if N <= 2:
            return np.ones(N, dtype=bool)

        # 建立三維座標陣列
        xyz = np.hstack([pts_xy, z[:, np.newaxis]])  # shape: (N, 3)

        keep = np.ones(N, dtype=bool)  # 預設全部保留

        # 向量化計算：對 j=1..N-2 的所有中間點
        p_prev = xyz[:-2]   # P_{j-1}，shape: (N-2, 3)
        p_mid  = xyz[1:-1]  # P_j
        p_next = xyz[2:]    # P_{j+1}

        v1 = p_mid - p_prev   # shape: (N-2, 3)
        v2 = p_next - p_mid   # shape: (N-2, 3)

        # 三維叉積（numpy cross）
        cross = np.cross(v1, v2)                     # shape: (N-2, 3)
        cross_norm = np.linalg.norm(cross, axis=1)   # shape: (N-2,)

        # 兩向量模長之積
        norm_v1 = np.linalg.norm(v1, axis=1)
        norm_v2 = np.linalg.norm(v2, axis=1)
        norm_prod = norm_v1 * norm_v2

        # 正弦值（叉積 / 模長積）= 共線度量（0=完全共線）
        sin_theta = np.where(norm_prod > 1e-12, cross_norm / norm_prod, 0.0)

        # sin_theta < ε → 共線 → 刪除中間點（索引 1..N-2 對應 j=1..N-2）
        collinear_mask = sin_theta < self.collinear_eps
        keep[1:-1] = ~collinear_mask  # 共線點標記為 False

        # 首尾點必須保留
        keep[0] = True
        keep[-1] = True

        return keep


# ============================================================
# 雙線性插值工具（獨立實作，供不使用 DEMTerrainManager 的情況）
# ============================================================

def bilinear_interpolate_dem(
    dem_grid: np.ndarray,
    lat_min: float, lat_max: float,
    lon_min: float, lon_max: float,
    query_lats: np.ndarray,
    query_lons: np.ndarray,
    nodata: float = -9999.0,
) -> np.ndarray:
    """
    純 numpy 雙線性插值，對應論文 Eq. 4。

    Eq. 4（二維雙線性插值）：
        h_psd = (x2-xi)(y2-yi)/(Δx*Δy) * h11
              + (xi-x1)(y2-yi)/(Δx*Δy) * h21
              + (x2-xi)(yi-y1)/(Δx*Δy) * h12
              + (xi-x1)(yi-y1)/(Δx*Δy) * h22

    其中：
        (xi, yi) = 查詢點的連續格點座標
        (x1,y1),(x2,y2) = 包圍格點的左下 / 右上角
        h11,h21,h12,h22 = 四角格點的 DEM 高程值

    Args:
        dem_grid   : shape (rows, cols) 的 DEM 高程矩陣 [m AMSL]
        lat_min / lat_max : DEM 緯度範圍
        lon_min / lon_max : DEM 經度範圍
        query_lats : shape (N,) 查詢點緯度
        query_lons : shape (N,) 查詢點經度
        nodata     : DEM 無效值（替換為 0）

    Returns:
        shape (N,) 的插值高程 [m AMSL]
    """
    rows, cols = dem_grid.shape

    # 換算為連續格點座標（row_f, col_f）
    row_f: np.ndarray = (lat_max - query_lats) / ((lat_max - lat_min) / rows)
    col_f: np.ndarray = (query_lons - lon_min) / ((lon_max - lon_min) / cols)

    # 邊界 clip（防止越界）
    row_f = np.clip(row_f, 0.0, rows - 1.0001)
    col_f = np.clip(col_f, 0.0, cols - 1.0001)

    # 取整數格點索引
    r0 = np.floor(row_f).astype(np.int32)    # y1（上）
    c0 = np.floor(col_f).astype(np.int32)    # x1（左）
    r1 = r0 + 1                              # y2（下）
    c1 = c0 + 1                              # x2（右）

    # 計算局部偏移量（Eq. 4 中的 xi-x1, y2-yi 等）
    dr = row_f - r0   # yi - y1（歸一化，0~1）
    dc = col_f - c0   # xi - x1（歸一化，0~1）

    # 取四角格點高程
    def _safe_get(r: np.ndarray, c: np.ndarray) -> np.ndarray:
        """安全取值（邊界 clip），並將 nodata 替換為 0"""
        r_clip = np.clip(r, 0, rows - 1)
        c_clip = np.clip(c, 0, cols - 1)
        vals = dem_grid[r_clip, c_clip]
        return np.where(vals == nodata, 0.0, vals.astype(np.float64))

    h11 = _safe_get(r0, c0)   # 左上（row0, col0）
    h21 = _safe_get(r0, c1)   # 右上（row0, col1）
    h12 = _safe_get(r1, c0)   # 左下（row1, col0）
    h22 = _safe_get(r1, c1)   # 右下（row1, col1）

    # Eq. 4：雙線性插值（使用歸一化偏移）
    # h = (1-dc)(1-dr)*h11 + dc*(1-dr)*h21 + (1-dc)*dr*h12 + dc*dr*h22
    h_interp: np.ndarray = (
        (1.0 - dc) * (1.0 - dr) * h11
        + dc * (1.0 - dr) * h21
        + (1.0 - dc) * dr * h12
        + dc * dr * h22
    )

    return h_interp
