"""感測器足跡建模 —— 由相機規格與航高反算地面掃描寬度 / GSD，或由目標 GSD 反算航高。

用途
----
VTOL 群飛偵察頁面需要「先決定要看多細，再決定飛多高、掃多寬」，而 DCCPP 規劃器
吃的是 ``fov_width_m``（地面掃描寬度）。這個模組就是兩者之間的換算層。

幾何模型
--------
機體水平、相機固定下視並可向前/側傾斜 ``mount_angle_deg``（0 = 正下方 nadir）。

  * 斜距（footprint 中心）      R      = h / cos(θ)
  * 橫向掃描寬度（cross-track） W      = 2·R·tan(HFOV/2)
  * 縱向足跡長度（along-track） L      = h·(tan(θ+β) − tan(θ−β))，β = VFOV/2
  * 地面取樣距離               GSD    = W / image_width_px

θ=0 時 W 退化為 2·h·tan(HFOV/2)，與正下視公式一致。

⚠ 傾斜角 + 半個 VFOV 逼近 90° 時相機會看到地平線，足跡長度發散。本模組在
``θ + VFOV/2 >= _MAX_LOOK_ANGLE_DEG`` 時直接丟 :class:`FootprintError`，
不回傳一個看起來很大但沒有意義的數字 —— 這類「靜默給出垃圾值」正是本專案
反覆踩過的坑。

單位一律 SI：公尺、公尺/秒、度。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

__all__ = [
    "CameraSpec",
    "FootprintError",
    "Footprint",
    "swath_width_m",
    "gsd_m_per_px",
    "altitude_for_gsd_m",
    "effective_sweep_width_m",
    "compute_footprint",
]

# 超過這個視線角就視為看到地平線（足跡發散），拒絕計算
_MAX_LOOK_ANGLE_DEG = 85.0


class FootprintError(ValueError):
    """相機幾何無解（視線角過大、參數非法）時丟出。"""


@dataclass(frozen=True)
class CameraSpec:
    """相機規格。

    參數
    ----
    hfov_deg : 橫向（cross-track）視場角，決定掃描寬度
    vfov_deg : 縱向（along-track）視場角，決定足跡長度
    image_width_px : 影像橫向像素數，決定 GSD
    mount_angle_deg : 掛載傾斜角，0 = 正下視；正值 = 向前/側傾
    """

    hfov_deg: float = 60.0
    vfov_deg: float = 45.0
    image_width_px: int = 4000
    mount_angle_deg: float = 0.0

    def __post_init__(self) -> None:
        if not (0.0 < self.hfov_deg < 180.0):
            raise FootprintError(f"hfov_deg 必須在 (0, 180) 之間，收到 {self.hfov_deg}")
        if not (0.0 < self.vfov_deg < 180.0):
            raise FootprintError(f"vfov_deg 必須在 (0, 180) 之間，收到 {self.vfov_deg}")
        if self.image_width_px <= 0:
            raise FootprintError(f"image_width_px 必須為正，收到 {self.image_width_px}")
        if not (-_MAX_LOOK_ANGLE_DEG < self.mount_angle_deg < _MAX_LOOK_ANGLE_DEG):
            raise FootprintError(
                f"mount_angle_deg 必須在 ±{_MAX_LOOK_ANGLE_DEG}° 內，"
                f"收到 {self.mount_angle_deg}"
            )

    @property
    def max_look_angle_deg(self) -> float:
        """視線最遠邊緣的角度（掛載角 + 半個 VFOV）。"""
        return abs(self.mount_angle_deg) + self.vfov_deg / 2.0

    def validate_geometry(self) -> None:
        """檢查足跡是否有限；發散時丟 FootprintError。"""
        if self.max_look_angle_deg >= _MAX_LOOK_ANGLE_DEG:
            raise FootprintError(
                f"掛載角 {self.mount_angle_deg}° + 半個 VFOV {self.vfov_deg / 2:.1f}° "
                f"= {self.max_look_angle_deg:.1f}°，已達 {_MAX_LOOK_ANGLE_DEG}° 視線上限："
                f"相機會看到地平線、足跡長度發散。請調小掛載角或 VFOV。"
            )


@dataclass(frozen=True)
class Footprint:
    """單張影像在地面的足跡。"""

    swath_width_m: float        # 橫向掃描寬度
    along_track_m: float        # 縱向足跡長度
    gsd_m_per_px: float         # 中心 GSD
    worst_gsd_m_per_px: float   # 最遠邊緣 GSD（傾斜時會比中心差）
    slant_range_m: float        # 到足跡中心的斜距
    near_edge_m: float          # 足跡近邊到星下點的水平距離
    far_edge_m: float           # 足跡遠邊到星下點的水平距離


def _check_altitude(altitude_agl_m: float) -> None:
    if altitude_agl_m <= 0.0:
        raise FootprintError(f"航高必須為正，收到 {altitude_agl_m}")


def swath_width_m(cam: CameraSpec, altitude_agl_m: float) -> float:
    """地面橫向掃描寬度（公尺）。"""
    _check_altitude(altitude_agl_m)
    cam.validate_geometry()
    slant = altitude_agl_m / math.cos(math.radians(cam.mount_angle_deg))
    return 2.0 * slant * math.tan(math.radians(cam.hfov_deg) / 2.0)


def gsd_m_per_px(cam: CameraSpec, altitude_agl_m: float) -> float:
    """足跡中心的地面取樣距離（公尺/像素）。"""
    return swath_width_m(cam, altitude_agl_m) / float(cam.image_width_px)


def altitude_for_gsd_m(cam: CameraSpec, target_gsd_m_per_px: float) -> float:
    """由目標 GSD 反算所需航高（公尺 AGL）。

    GSD 與航高成正比，所以直接反解即可。
    """
    if target_gsd_m_per_px <= 0.0:
        raise FootprintError(f"目標 GSD 必須為正，收到 {target_gsd_m_per_px}")
    cam.validate_geometry()
    # gsd(h) = h · k  ⇒  h = gsd / k，k 取 h=1 時的值
    k = gsd_m_per_px(cam, 1.0)
    return target_gsd_m_per_px / k


def effective_sweep_width_m(swath_m: float, overlap_rate: float) -> float:
    """扣掉相鄰條帶重疊後的有效掃描寬度 —— 這才是 DCCPP 該吃的條帶間距。

    overlap_rate 為相鄰條帶重疊比例（0.0–0.95）。
    """
    if swath_m <= 0.0:
        raise FootprintError(f"掃描寬度必須為正，收到 {swath_m}")
    if not (0.0 <= overlap_rate < 1.0):
        raise FootprintError(f"重疊率必須在 [0, 1) 之間，收到 {overlap_rate}")
    return swath_m * (1.0 - overlap_rate)


def compute_footprint(cam: CameraSpec, altitude_agl_m: float) -> Footprint:
    """一次算出完整足跡幾何。"""
    _check_altitude(altitude_agl_m)
    cam.validate_geometry()

    theta = math.radians(cam.mount_angle_deg)
    beta = math.radians(cam.vfov_deg) / 2.0

    slant = altitude_agl_m / math.cos(theta)
    swath = 2.0 * slant * math.tan(math.radians(cam.hfov_deg) / 2.0)

    near = altitude_agl_m * math.tan(theta - beta)
    far = altitude_agl_m * math.tan(theta + beta)
    along = far - near

    # 最遠邊緣的斜距決定最差 GSD
    slant_far = altitude_agl_m / math.cos(theta + beta)
    swath_far = 2.0 * slant_far * math.tan(math.radians(cam.hfov_deg) / 2.0)

    return Footprint(
        swath_width_m=swath,
        along_track_m=along,
        gsd_m_per_px=swath / float(cam.image_width_px),
        worst_gsd_m_per_px=swath_far / float(cam.image_width_px),
        slant_range_m=slant,
        near_edge_m=near,
        far_edge_m=far,
    )
