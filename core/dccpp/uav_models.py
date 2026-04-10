"""
模組一：雙機型 UAV 數據結構與通用介面
========================================
對應論文《Multiple fixed-wing UAVs collaborative coverage 3D path planning
method for complex areas》第 2.1 節。

核心設計：
    - UAVType      : 機型枚舉（固定翼 / 多旋翼）
    - FOVConfig    : 感測器視野配置（論文中感測器覆蓋寬度 w_s）
    - DCCPPUAVConfig : 完整 UAV 配置，包含位置、FOV、高度與機型差異化參數

機型關鍵差異：
    FIXED_WING  → min_turning_radius (R_min > 0)，路徑規劃需滿足 Dubins 曲線約束
    MULTI_ROTOR → min_turning_radius = 0.0，可原地懸停與自由轉向

作者: NCIST_planner_V1
版本: 1.0.0
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Tuple, Optional

import numpy as np


# ============================================================
# 機型枚舉
# ============================================================

class UAVType(Enum):
    """
    無人機機型枚舉。

    論文中以此區分路徑轉移的計算方式：
        FIXED_WING  → 必須用 Dubins 曲線（受最小轉彎半徑 R_min 約束）
        MULTI_ROTOR → 直接計算 Euclidean 距離（可原地懸停與轉向）
    """
    FIXED_WING = auto()    # 固定翼：有最小轉彎半徑限制
    MULTI_ROTOR = auto()   # 多旋翼：可懸停，無轉彎半徑限制


# ============================================================
# 感測器視野配置
# ============================================================

@dataclass
class FOVConfig:
    """
    感測器視野（Field of View）配置。

    論文符號對照：
        fov_angle_deg  : 感測器開角（半角），對應相機/雷達的感測角度 θ_fov
        coverage_width : 地面覆蓋寬度 w_s [m]，掃描條帶（swath）的實際寬度
                         w_s = 2 * altitude * tan(θ_fov / 2)

    若 coverage_width 未手動指定（為 None），則在 __post_init__ 中
    依據飛行高度自動計算；上層呼叫 compute_coverage_width() 時傳入高度即可。

    Attributes:
        fov_angle_deg  : 感測器半開角 [度]，例如相機水平 FOV
        coverage_width : 預設地面覆蓋寬度 [m]；None 代表依高度動態計算
        overlap_ratio  : 相鄰掃描條帶重疊率（0~1），影響有效條帶間距 d_s
                         論文中用於計算 d_s = w_s * (1 - overlap_ratio)
    """
    fov_angle_deg: float = 60.0          # 感測器開角（半角），單位：度
    coverage_width: Optional[float] = None  # 覆蓋寬度 w_s [m]，None 為自動計算
    overlap_ratio: float = 0.2           # 掃描條帶重疊率，用於計算有效間距

    def __post_init__(self) -> None:
        """驗證 FOV 參數合法性"""
        if not (0.0 < self.fov_angle_deg < 180.0):
            raise ValueError(
                f"fov_angle_deg 必須在 (0, 180) 範圍內，實際值：{self.fov_angle_deg}"
            )
        if not (0.0 <= self.overlap_ratio < 1.0):
            raise ValueError(
                f"overlap_ratio 必須在 [0, 1) 範圍內，實際值：{self.overlap_ratio}"
            )
        if self.coverage_width is not None and self.coverage_width <= 0:
            raise ValueError(
                f"coverage_width 必須為正數，實際值：{self.coverage_width}"
            )

    def compute_coverage_width(self, altitude: float) -> float:
        """
        依飛行高度計算地面覆蓋寬度 w_s。

        公式（平地近似）：
            w_s = 2 * altitude * tan(θ_fov / 2)

        其中 θ_fov = fov_angle_deg（整個開角，非半角）

        Args:
            altitude: 飛行高度 [m]（相對地面高度 AGL）

        Returns:
            地面覆蓋寬度 w_s [m]
        """
        if self.coverage_width is not None:
            # 若已手動指定，直接返回
            return self.coverage_width

        if altitude <= 0:
            raise ValueError(f"飛行高度必須為正數，實際值：{altitude}")

        # w_s = 2 * h * tan(θ_fov / 2)，θ_fov 為完整開角
        half_fov_rad = math.radians(self.fov_angle_deg / 2.0)
        return 2.0 * altitude * math.tan(half_fov_rad)

    def compute_strip_spacing(self, altitude: float) -> float:
        """
        計算有效掃描條帶間距 d_s（考慮重疊率後的實際前進距離）。

        公式：
            d_s = w_s * (1 - overlap_ratio)

        Args:
            altitude: 飛行高度 [m]

        Returns:
            有效條帶間距 d_s [m]
        """
        w_s = self.compute_coverage_width(altitude)
        return w_s * (1.0 - self.overlap_ratio)


# ============================================================
# 完整 UAV 配置（DCCPP 主數據結構）
# ============================================================

@dataclass
class DCCPPUAVConfig:
    """
    DCCPP 無人機完整配置資料結構。

    對應論文 Section 2.1 的 UAV 狀態向量：
        S_i = (x_i, y_i, χ_i, r_i)

    其中：
        (x_i, y_i) → position（起始位置）
        χ_i        → initial_heading_deg（初始航向角）
        r_i        → min_turning_radius（最小轉彎半徑 R_min）

    額外屬性（本模組擴充）：
        uav_id           : 無人機唯一識別碼
        uav_type         : 機型（FIXED_WING / MULTI_ROTOR）
        fov              : 感測器視野配置
        altitude         : 飛行高度 [m]
        cruise_speed     : 巡航速度 [m/s]
        min_turning_radius : R_min [m]
                            固定翼：由速度與最大滾轉角決定，必須 > 0
                            多旋翼：預設為 0（無轉彎半徑限制）

    Attributes:
        uav_id             : 無人機 ID（整數）
        position           : 起始位置 (x, y) [m] 或 (lat, lon) [度]
        fov                : FOVConfig 感測器視野配置
        altitude           : 飛行高度 AGL [m]
        uav_type           : 機型枚舉（FIXED_WING / MULTI_ROTOR）
        min_turning_radius : R_min [m]；多旋翼預設 0.0
        initial_heading_deg: 初始航向角 χ_i [度]，0=北，順時針為正
        cruise_speed       : 巡航速度 [m/s]
    """

    uav_id: int
    position: Tuple[float, float]              # 起始位置 (x, y) 或 (lat, lon)
    fov: FOVConfig                             # 感測器視野配置
    altitude: float                            # 飛行高度 AGL [m]
    uav_type: UAVType = UAVType.MULTI_ROTOR    # 機型，預設多旋翼

    # R_min：最小轉彎半徑
    #   固定翼 → 由 FixedWingConstraints 計算後帶入（必須 > 0）
    #   多旋翼 → 預設為 0.0（無轉彎半徑限制，可原地旋轉）
    min_turning_radius: float = 0.0

    initial_heading_deg: float = 0.0           # 初始航向角 χ_i [度]
    cruise_speed: float = 15.0                 # 巡航速度 [m/s]

    def __post_init__(self) -> None:
        """驗證配置參數合法性，並強制固定翼 R_min > 0"""
        if self.altitude <= 0:
            raise ValueError(f"飛行高度必須為正數，實際值：{self.altitude}")

        if self.cruise_speed <= 0:
            raise ValueError(f"巡航速度必須為正數，實際值：{self.cruise_speed}")

        if self.min_turning_radius < 0:
            raise ValueError(
                f"min_turning_radius 不得為負數，實際值：{self.min_turning_radius}"
            )

        # 機型一致性檢查：固定翼必須有有限轉彎半徑
        if self.uav_type == UAVType.FIXED_WING and self.min_turning_radius == 0.0:
            raise ValueError(
                "固定翼（FIXED_WING）的 min_turning_radius 必須 > 0。"
                "多旋翼若設定了 FIXED_WING 機型屬於配置錯誤，請確認。"
            )

        # 機型一致性檢查：多旋翼若設定非零轉彎半徑僅警告（允許，但通常不需要）
        if self.uav_type == UAVType.MULTI_ROTOR and self.min_turning_radius > 0.0:
            import warnings
            warnings.warn(
                f"MULTI_ROTOR 機型通常 min_turning_radius=0，"
                f"但設定了 {self.min_turning_radius:.1f} m。"
                f"路徑成本計算器仍將使用 Euclidean 距離。",
                UserWarning,
                stacklevel=2,
            )

    # ── 唯讀屬性 ────────────────────────────────────────────

    @property
    def is_fixed_wing(self) -> bool:
        """是否為固定翼機型"""
        return self.uav_type == UAVType.FIXED_WING

    @property
    def is_multi_rotor(self) -> bool:
        """是否為多旋翼機型"""
        return self.uav_type == UAVType.MULTI_ROTOR

    @property
    def position_array(self) -> np.ndarray:
        """以 numpy 陣列返回起始位置 [x, y]"""
        return np.array(self.position, dtype=np.float64)

    @property
    def coverage_width(self) -> float:
        """
        依當前飛行高度計算地面覆蓋寬度 w_s [m]。
        這是掃描條帶的完整寬度，論文中用於決定相鄰掃描線間距。
        """
        return self.fov.compute_coverage_width(self.altitude)

    @property
    def strip_spacing(self) -> float:
        """
        有效掃描條帶間距 d_s [m]（考慮重疊率）。
        d_s = w_s * (1 - overlap_ratio)
        """
        return self.fov.compute_strip_spacing(self.altitude)

    # ── 工具方法 ────────────────────────────────────────────

    def initial_heading_rad(self) -> float:
        """返回初始航向角（弧度）"""
        return math.radians(self.initial_heading_deg)

    def summary(self) -> str:
        """返回人類可讀的配置摘要（用於 debug / logging）"""
        type_str = "固定翼 (Fixed-Wing)" if self.is_fixed_wing else "多旋翼 (Multi-Rotor)"
        rmin_str = f"{self.min_turning_radius:.1f} m" if self.is_fixed_wing else "N/A（多旋翼無限制）"
        return (
            f"[UAV #{self.uav_id}] 機型：{type_str}\n"
            f"  起始位置  : {self.position}\n"
            f"  飛行高度  : {self.altitude:.1f} m\n"
            f"  巡航速度  : {self.cruise_speed:.1f} m/s\n"
            f"  初始航向  : {self.initial_heading_deg:.1f}°\n"
            f"  R_min     : {rmin_str}\n"
            f"  FOV 開角  : {self.fov.fov_angle_deg:.1f}°\n"
            f"  覆蓋寬度  : {self.coverage_width:.2f} m\n"
            f"  條帶間距  : {self.strip_spacing:.2f} m"
        )


# ============================================================
# 工廠函式：快速建立常見 UAV 配置
# ============================================================

def make_fixed_wing_uav(
    uav_id: int,
    position: Tuple[float, float],
    min_turning_radius: float,
    altitude: float = 100.0,
    cruise_speed: float = 20.0,
    fov_angle_deg: float = 60.0,
    coverage_width: Optional[float] = None,
    initial_heading_deg: float = 0.0,
) -> DCCPPUAVConfig:
    """
    快速建立固定翼 UAV 配置。

    Args:
        uav_id             : UAV 識別碼
        position           : 起始位置 (x, y)
        min_turning_radius : R_min [m]（必填，固定翼不得為 0）
        altitude           : 飛行高度 [m]
        cruise_speed       : 巡航速度 [m/s]
        fov_angle_deg      : 感測器開角 [度]
        coverage_width     : 固定覆蓋寬度 [m]，None 為自動計算
        initial_heading_deg: 初始航向角 [度]

    Returns:
        DCCPPUAVConfig 固定翼實例
    """
    return DCCPPUAVConfig(
        uav_id=uav_id,
        position=position,
        fov=FOVConfig(
            fov_angle_deg=fov_angle_deg,
            coverage_width=coverage_width,
        ),
        altitude=altitude,
        uav_type=UAVType.FIXED_WING,
        min_turning_radius=min_turning_radius,
        initial_heading_deg=initial_heading_deg,
        cruise_speed=cruise_speed,
    )


def make_multi_rotor_uav(
    uav_id: int,
    position: Tuple[float, float],
    altitude: float = 50.0,
    cruise_speed: float = 10.0,
    fov_angle_deg: float = 60.0,
    coverage_width: Optional[float] = None,
    initial_heading_deg: float = 0.0,
) -> DCCPPUAVConfig:
    """
    快速建立多旋翼 UAV 配置。

    多旋翼的 min_turning_radius 自動設為 0（可原地轉向）。

    Args:
        uav_id             : UAV 識別碼
        position           : 起始位置 (x, y)
        altitude           : 飛行高度 [m]
        cruise_speed       : 巡航速度 [m/s]
        fov_angle_deg      : 感測器開角 [度]
        coverage_width     : 固定覆蓋寬度 [m]，None 為自動計算
        initial_heading_deg: 初始航向角 [度]

    Returns:
        DCCPPUAVConfig 多旋翼實例
    """
    return DCCPPUAVConfig(
        uav_id=uav_id,
        position=position,
        fov=FOVConfig(
            fov_angle_deg=fov_angle_deg,
            coverage_width=coverage_width,
        ),
        altitude=altitude,
        uav_type=UAVType.MULTI_ROTOR,
        min_turning_radius=0.0,    # 多旋翼預設 0：無轉彎半徑限制
        initial_heading_deg=initial_heading_deg,
        cruise_speed=cruise_speed,
    )
