"""4+1 QuadPlane 起降剖面幾何。

為什麼要有這個模組
------------------
DCCPP 規劃器原本只會產生**固定翼跑道起降**：從 Home 沿跑道方向滑行
``runway_length_m``（預設 500 m），再以階梯爬升爬到巡航高度。VTOL 偵察頁面
選的是 4+1 QuadPlane，卻沿用了同一條路徑 —— 於是規劃出來的軌跡是一條長長的
斜線，而面板上的「轉換高度」根本沒有任何地方讀它。

這裡把 4+1 QuadPlane 的起降剖面寫成可計算的幾何：

    起飛：Home(地面) ──垂直爬升──> 轉換高度 ──前向過渡──> 固定翼爬升 ──> 巡航高度
    降落：巡航高度 ──固定翼下降──> 轉換高度 ──反向過渡──> 垂直下降 ──> Home(地面)

常數來源
--------
下列常數**對應 mission/sitl_launcher.generate_vtol_params() 實際寫進 SITL 的值**，
讓規劃器與飛控參數不會各說各話。改任何一邊都要同時改另一邊
（tests/test_vtol_profile.py 有值層級守衛擋住漂移）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:                       # 只為型別註解，避免執行期循環匯入
    from core.base.airframe import Airframe

__all__ = [
    "VTOLProfileError",
    "VTOLProfile",
    "Q_TRANSITION_MS",
    "Q_BACKTRANS_MS",
    "Q_TRANS_DECEL",
    "Q_PILOT_SPD_UP",
    "TECS_CLMB_MAX",
    "TECS_SINK_MAX",
    "AIRSPEED_CRUISE",
]

# ── 與 vtol_default.parm 同步的常數 ──────────────────────────
Q_TRANSITION_MS = 3000.0   # 前向過渡（MC→FW）時間 [ms]
Q_BACKTRANS_MS = 3000.0    # 反向過渡（FW→MC）時間 [ms]
Q_TRANS_DECEL = 2.0        # 反向過渡減速度 [m/s²]
Q_PILOT_SPD_UP = 2.5       # 多旋翼段垂直爬升率 [m/s]
TECS_CLMB_MAX = 5.0        # 固定翼段最大爬升率 [m/s]
TECS_SINK_MAX = 5.0        # 固定翼段最大下沉率 [m/s]
AIRSPEED_CRUISE = 24.0     # 飛控自己的巡航空速 [m/s]（vtol_default.parm）

# 前向過渡至少要走這麼遠，避免速度很低時算出 0 長度的過渡段
_MIN_TRANSITION_M = 20.0

# NAV_VTOL_LAND 進場段的安全餘裕。
# ⚠ 飛控的 PreArm 檢查「VTOL land too short」用的是**它自己的** AIRSPEED_CRUISE
#   算煞停距離（實測 24 m/s、Q_TRANS_DECEL=2 ⇒ 煞停 144 m、PreArm 門檻 108 m
#   = 0.75×煞停），跟面板設的巡航速度**無關**。規劃器若只用面板速度（例如 18）
#   算反向過渡距離會得到 81 m —— 低於門檻，真機直接拒絕解鎖。
_LAND_APPROACH_MARGIN = 1.25


class VTOLProfileError(ValueError):
    """起降剖面參數無解時丟出。"""


@dataclass(frozen=True)
class VTOLProfile:
    """一組 VTOL 起降剖面參數，以及由它推出的各段幾何。

    參數
    ----
    transition_alt_m : 垂直爬升到多高才轉固定翼（面板「轉換高度」）
    cruise_alt_m     : 任務巡航高度（面板「偵察航高」）
    cruise_speed_mps : 固定翼段巡航空速（面板「巡航速度」）
    """

    transition_alt_m: float
    cruise_alt_m: float
    cruise_speed_mps: float

    #: 固定翼段的**機體**爬升/下降航跡角（度）。
    #:
    #: ⚠️⚠️ 給了就用它；``None`` 才退回用飛控的 ``TECS_CLMB_MAX`` /
    #: ``TECS_SINK_MAX`` 推。兩者差很多，而且方向固定：
    #:
    #:   飛控常數 5.0 m/s @24 m/s ⇒ 11.77°  ⇒ 爬 100 m 只要 480 m
    #:   Pioneer / HERO 實機 3.0°           ⇒ 爬 100 m 需要 **1908 m**
    #:
    #: 飛控參數是**能力上限**，機體航跡角才是**實際飛得出來的**。拿上限當
    #: 實際值會把起降佔用的水平距離少算約 4 倍。用 :meth:`from_airframe`
    #: 就會自動帶進來；:attr:`climb_angle_source` 隨時可查用的是哪一個。
    climb_path_angle_deg: Optional[float] = None
    descent_path_angle_deg: Optional[float] = None

    def __post_init__(self) -> None:
        if self.transition_alt_m <= 0:
            raise VTOLProfileError(
                f"轉換高度必須為正，收到 {self.transition_alt_m}"
            )
        if self.cruise_speed_mps <= 0:
            raise VTOLProfileError(
                f"巡航速度必須為正，收到 {self.cruise_speed_mps}"
            )
        if self.cruise_alt_m < self.transition_alt_m:
            raise VTOLProfileError(
                f"巡航高度 {self.cruise_alt_m} m 低於轉換高度 "
                f"{self.transition_alt_m} m —— 這樣沒有固定翼爬升段可言，"
                f"請把轉換高度調低或把偵察航高調高。"
            )
        for fld in ("climb_path_angle_deg", "descent_path_angle_deg"):
            v = getattr(self, fld)
            if v is not None and not (0.0 < v < 90.0):
                raise VTOLProfileError(
                    f"{fld} = {v}° 不在 (0, 90)。0° 永遠爬不上去，"
                    f"90° 的水平距離是 0 —— 兩者都不是航跡角。"
                )

    # ── 從機體紀錄建構 ───────────────────────────────────────
    @classmethod
    def from_airframe(
        cls,
        airframe: "Airframe",
        cruise_alt_m: float,
        transition_alt_m: Optional[float] = None,
        cruise_speed_mps: Optional[float] = None,
    ) -> "VTOLProfile":
        """用 :class:`core.base.airframe.Airframe` 建剖面。

        速度、爬升角、下降角、轉換高度全部來自機體，不再從飛控常數推。
        ``transition_alt_m`` / ``cruise_speed_mps`` 可覆寫（任務需要時）。

        ⚠️ 機體的 ``transition_complete_agl_m`` 目前多半是暫定值
        （``transition_agl_is_placeholder``）。這裡照用，但呼叫端若要出報表
        必須自己檢查那個旗標並標註。
        """
        alt = transition_alt_m
        if alt is None:
            alt = airframe.transition_complete_agl_m or airframe.transition_start_agl_m
        if alt is None:
            raise VTOLProfileError(
                f"{airframe.name} 沒有轉換高度，也沒有從呼叫端傳入 "
                f"transition_alt_m —— 無法決定垂直段要爬多高。"
            )
        return cls(
            transition_alt_m=float(alt),
            cruise_alt_m=float(cruise_alt_m),
            cruise_speed_mps=float(cruise_speed_mps
                                   if cruise_speed_mps is not None
                                   else airframe.cruise_airspeed_mps),
            climb_path_angle_deg=airframe.max_climb_path_angle_deg,
            descent_path_angle_deg=airframe.max_descent_path_angle_deg,
        )

    @property
    def climb_angle_source(self) -> str:
        """``'airframe'`` 或 ``'flight-controller'`` —— 用的是哪一個爬升角。

        ⚠️ 兩者差約 4 倍水平距離，所以任何報表都該把這個字串印出來，
        不要讓讀者去猜。
        """
        return ("airframe" if self.climb_path_angle_deg is not None
                else "flight-controller")

    # ── 垂直段 ───────────────────────────────────────────────
    @property
    def vertical_climb_time_s(self) -> float:
        """以 Q_PILOT_SPD_UP 垂直爬到轉換高度所需時間。"""
        return self.transition_alt_m / Q_PILOT_SPD_UP

    @property
    def vertical_descent_time_s(self) -> float:
        return self.transition_alt_m / Q_PILOT_SPD_UP

    # ── 過渡段 ───────────────────────────────────────────────
    @property
    def forward_transition_m(self) -> float:
        """MC→FW：由靜止加速到巡航速度所需的水平距離。

        等加速 d = ½·v·t，t = Q_TRANSITION_MS。
        """
        d = 0.5 * self.cruise_speed_mps * (Q_TRANSITION_MS / 1000.0)
        return max(d, _MIN_TRANSITION_M)

    @property
    def back_transition_m(self) -> float:
        """FW→MC：由巡航速度減速到懸停所需的水平距離。

        取兩個限制的較大者：
          * 減速度限制 d = v² / (2·Q_TRANS_DECEL)
          * 過渡時間限制 d = ½·v·Q_BACKTRANS_MS
        """
        by_decel = self.cruise_speed_mps ** 2 / (2.0 * Q_TRANS_DECEL)
        by_time = 0.5 * self.cruise_speed_mps * (Q_BACKTRANS_MS / 1000.0)
        return max(by_decel, by_time, _MIN_TRANSITION_M)

    @property
    def land_approach_m(self) -> float:
        """NAV_VTOL_LAND 前的最後進場段至少要多長。

        取兩者較大：
          * 依**面板巡航速度**算的反向過渡距離（back_transition_m）
          * 依**飛控 AIRSPEED_CRUISE** 算的煞停距離 × 安全餘裕
            —— PreArm「VTOL land too short」用的是後者，跟面板速度無關。
        """
        fc_stop = AIRSPEED_CRUISE ** 2 / (2.0 * Q_TRANS_DECEL)
        return max(self.back_transition_m, fc_stop * _LAND_APPROACH_MARGIN)

    # ── 固定翼爬升 / 下降段 ──────────────────────────────────
    @property
    def climb_height_m(self) -> float:
        """轉換高度 → 巡航高度 的高度差。"""
        return self.cruise_alt_m - self.transition_alt_m

    @property
    def fw_climb_distance_m(self) -> float:
        """爬完 climb_height 所需的水平距離。

        有機體爬升角就用 ``h / tan γ``；沒有才退回飛控的 ``TECS_CLMB_MAX``。
        ⚠️ 兩者差約 4 倍，見 :attr:`climb_path_angle_deg` 的說明。
        """
        if self.climb_height_m <= 0:
            return 0.0
        if self.climb_path_angle_deg is not None:
            return self.climb_height_m / math.tan(
                math.radians(self.climb_path_angle_deg))
        t = self.climb_height_m / TECS_CLMB_MAX
        return self.cruise_speed_mps * t

    @property
    def fw_descent_distance_m(self) -> float:
        if self.climb_height_m <= 0:
            return 0.0
        if self.descent_path_angle_deg is not None:
            return self.climb_height_m / math.tan(
                math.radians(self.descent_path_angle_deg))
        t = self.climb_height_m / TECS_SINK_MAX
        return self.cruise_speed_mps * t

    @property
    def climb_angle_deg(self) -> float:
        """固定翼爬升段的爬升角（度）。

        機體給了就是機體的值；沒給才由飛控爬升率與巡航速度反推。
        用 :attr:`climb_angle_source` 查是哪一個。
        """
        if self.climb_path_angle_deg is not None:
            return float(self.climb_path_angle_deg)
        return math.degrees(math.atan2(TECS_CLMB_MAX, self.cruise_speed_mps))

    @property
    def descent_angle_deg(self) -> float:
        if self.descent_path_angle_deg is not None:
            return float(self.descent_path_angle_deg)
        return math.degrees(math.atan2(TECS_SINK_MAX, self.cruise_speed_mps))

    # ── 整體 ─────────────────────────────────────────────────
    @property
    def takeoff_ground_distance_m(self) -> float:
        """起飛整段佔用的水平距離（垂直段為 0）。"""
        return self.forward_transition_m + self.fw_climb_distance_m

    @property
    def landing_ground_distance_m(self) -> float:
        return self.fw_descent_distance_m + self.land_approach_m

    def summary(self) -> str:
        return (
            f"VTOL 剖面：垂直爬升 {self.transition_alt_m:.0f} m "
            f"({self.vertical_climb_time_s:.0f} s) → 前向過渡 "
            f"{self.forward_transition_m:.0f} m → 固定翼爬升 "
            f"{self.fw_climb_distance_m:.0f} m 至 {self.cruise_alt_m:.0f} m "
            f"(爬升角 {self.climb_angle_deg:.1f}°，來源 {self.climb_angle_source})；"
            f"降落反向，反向過渡 {self.back_transition_m:.0f} m"
        )
