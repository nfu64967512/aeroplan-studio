"""
梯形 FOV 地面投影模型
============================
對應論文《Multiple fixed-wing UAVs collaborative coverage 3D path planning
method for complex areas》Section 2.2.1 (Eq. 2-3)。

固定翼 UAV 的感測器具有掛載角 (α_m)、垂直張角 (α_v) 和水平張角 (α_h)，
其地面投影為梯形 ABCD，而非單純的矩形。

當掛載角 α_m = 0（垂直朝下）時退化為矩形，與現有 CameraModel 行為一致。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from sensors.camera_model import CameraSpecs


# ============================================================
# 資料結構
# ============================================================

@dataclass
class TrapezoidFootprint:
    """
    梯形 FOV 地面投影幾何結果

    Attributes:
        near_width_m   : 近端（前方）掃描寬度 w_f (m)
        far_width_m    : 遠端（後方）掃描寬度 w_b (m)
        near_dist_m    : 近端距 UAV 正下方投影的前方距離 d_f (m)
        far_dist_m     : 遠端距 UAV 正下方投影的後方距離（負值表在後方）
        center_width_m : 中心掃描寬度 w_i (m)
        center_dist_m  : FOV 中心距 UAV 正下方投影的前方距離 l_i (m)
        effective_width_m : 等效覆蓋寬度（用於計算掃描間距）
        lf_m           : FOV 前方邊界到投影中心的距離 l_f (m)
    """
    near_width_m: float
    far_width_m: float
    near_dist_m: float
    far_dist_m: float
    center_width_m: float
    center_dist_m: float
    effective_width_m: float
    lf_m: float


# ============================================================
# 梯形 FOV 計算器
# ============================================================

class TrapezoidalFOV:
    """
    基於論文 Eq. 2-3 的梯形 FOV 地面投影模型。

    可由 CameraSpecs 自動計算 α_v / α_h，
    或直接指定垂直/水平半張角。

    使用方式:
        fov = TrapezoidalFOV(camera_specs=my_camera, mounting_angle_deg=72.7)
        fp = fov.compute_ground_footprint(altitude_m=1200.0)
        strip_width = fov.effective_strip_width(altitude_m=1200.0, overlap_rate=0.1)
    """

    def __init__(
        self,
        camera_specs: Optional[CameraSpecs] = None,
        mounting_angle_deg: float = 0.0,
        vertical_half_angle_deg: Optional[float] = None,
        horizontal_half_angle_deg: Optional[float] = None,
    ):
        """
        參數:
            camera_specs             : 相機硬體規格（自動計算 α_v / α_h）
            mounting_angle_deg       : 感測器掛載角 α_m（度），0=垂直朝下，90=水平前視
            vertical_half_angle_deg  : 垂直半張角 α_v/2（度），若 None 則從 camera_specs 計算
            horizontal_half_angle_deg: 水平半張角 α_h/2（度），若 None 則從 camera_specs 計算
        """
        self.alpha_m = math.radians(mounting_angle_deg)

        if vertical_half_angle_deg is not None:
            self.alpha_v_half = math.radians(vertical_half_angle_deg)
        elif camera_specs is not None:
            vfov = 2.0 * math.atan(
                camera_specs.sensor_height_mm / (2.0 * camera_specs.focal_length_mm)
            )
            self.alpha_v_half = vfov / 2.0
        else:
            self.alpha_v_half = math.radians(22.5)  # 預設 45° 全角

        if horizontal_half_angle_deg is not None:
            self.alpha_h_half = math.radians(horizontal_half_angle_deg)
        elif camera_specs is not None:
            hfov = 2.0 * math.atan(
                camera_specs.sensor_width_mm / (2.0 * camera_specs.focal_length_mm)
            )
            self.alpha_h_half = hfov / 2.0
        else:
            self.alpha_h_half = math.radians(38.5)  # 預設 77° 全角

        self.camera_specs = camera_specs

    def compute_ground_footprint(self, altitude_m: float) -> TrapezoidFootprint:
        """
        計算給定高度下 FOV 的地面投影幾何。

        論文 Eq. 2:
            w_f = 2 * h * tan(α_h/2) / sin(α_m + α_v/2)
            w_b = 2 * h * tan(α_h/2) / sin(α_m - α_v/2)
            d_f = h * tan(α_m + α_v/2)  (前方距離)
            ...

        論文 Eq. 3:
            l_i = (d_f + l_f) / 2
            w_i = (w_f + w_b) / 2

        參數:
            altitude_m: 相對地面高度 (m)

        返回:
            TrapezoidFootprint 包含完整幾何資訊
        """
        h = max(altitude_m, 1.0)
        am = self.alpha_m
        av = self.alpha_v_half
        ah = self.alpha_h_half

        # ── 特例：掛載角 = 0（垂直朝下）→ 矩形 FOV ──
        if abs(am) < 1e-6:
            w = 2.0 * h * math.tan(ah)
            d = h * math.tan(av)
            return TrapezoidFootprint(
                near_width_m=w,
                far_width_m=w,
                near_dist_m=d,
                far_dist_m=-d,
                center_width_m=w,
                center_dist_m=0.0,
                effective_width_m=w,
                lf_m=d,
            )

        # ── 論文 Eq. 2：前方/後方掃描寬度 ──
        # 前方邊界（α_m + α_v/2 方向）
        angle_near = am + av
        angle_far = am - av

        # 安全限制：角度不能 >= 90°
        angle_near = min(angle_near, math.radians(89.0))
        if angle_far < math.radians(1.0):
            angle_far = math.radians(1.0)

        # 前方距離（感測器前方邊界到 UAV 正下方投影的水平距離）
        d_f = h * math.tan(angle_near)
        # 後方距離
        d_b = h * math.tan(angle_far)

        # 前方寬度 w_f（近端，較窄）
        sin_near = math.sin(angle_near)
        w_f = 2.0 * h * math.tan(ah) / sin_near if sin_near > 0.01 else 2.0 * h * math.tan(ah)

        # 後方寬度 w_b（遠端，較寬 — 因為角度較小、距離較近）
        sin_far = math.sin(angle_far)
        w_b = 2.0 * h * math.tan(ah) / sin_far if sin_far > 0.01 else 2.0 * h * math.tan(ah)

        # ── 論文 Eq. 3：等效參數 ──
        lf_m = (d_f + d_b) / 2.0   # FOV 高度（沿飛行方向）
        center_dist = (d_f + d_b) / 2.0   # FOV 中心到正下方的距離
        center_width = (w_f + w_b) / 2.0   # 中心寬度

        # 等效覆蓋寬度取較窄的寬度（保守估計，確保完整覆蓋）
        effective_width = min(w_f, w_b)

        return TrapezoidFootprint(
            near_width_m=w_f,
            far_width_m=w_b,
            near_dist_m=d_f,
            far_dist_m=d_b,
            center_width_m=center_width,
            center_dist_m=center_dist,
            effective_width_m=effective_width,
            lf_m=lf_m,
        )

    def effective_strip_width(
        self,
        altitude_m: float,
        overlap_rate: float = 0.1,
    ) -> float:
        """
        計算有效掃描間距（用於 RegionDivider.decompose_by_fov）。

        strip_spacing = effective_width × (1 - overlap_rate)

        參數:
            altitude_m   : 飛行高度 (m)
            overlap_rate : 重疊率 (0–1)

        返回:
            掃描線間距 (m)
        """
        fp = self.compute_ground_footprint(altitude_m)
        return fp.effective_width_m * (1.0 - overlap_rate)

    def coverage_width(self, altitude_m: float) -> float:
        """
        取得 FOV 的等效覆蓋寬度 w_i（用於覆蓋規劃中的 coverage_width_m）。

        參數:
            altitude_m: 飛行高度 (m)

        返回:
            等效覆蓋寬度 (m)
        """
        fp = self.compute_ground_footprint(altitude_m)
        return fp.effective_width_m

    def forward_distance(self, altitude_m: float) -> float:
        """
        取得 FOV 中心前方距離 l_i（用於觸發間距計算）。

        參數:
            altitude_m: 飛行高度 (m)

        返回:
            前方覆蓋距離 (m)
        """
        fp = self.compute_ground_footprint(altitude_m)
        return fp.center_dist_m
