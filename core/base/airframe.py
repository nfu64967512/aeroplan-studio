"""機體能力紀錄 —— 規劃層唯一的「這台飛機能做什麼」事實來源。

為什麼要有這個模組
------------------
規劃層原本用 ``turn_radius > 0`` 當固定翼的判別式，而那一個述詞同時承擔了
**三件可以分開的事**：

    1. 巡航段需不需要 Dubins 曲線（有沒有最小轉彎半徑）
    2. 是不是固定翼
    3. 需不需要跑道起降

VTOL 把這三件事拆開了：它有最小轉彎半徑、是固定翼、但**不需要跑道**。
於是 ``turn_radius=0`` 的 VTOL 會靜靜地拿不到起降段，而
``turn_radius>0`` 的 VTOL 會被當成跑道機。本模組把三件事拆成三個欄位。

⚠️⚠️ 另一個實測到的後果：任務分配器用
``vehicle_type == FIXED_WING and turn_radius > 0`` 決定走直線成本還是 Dubins
成本，而 ``'vtol'`` 字串在 ``swarm_coordinator`` 被轉成 MULTIROTOR
⇒ 分配器算 556.6 m 直線、路徑建構器飛 870.8 m Dubins，**低估 56%**。
成本模型與幾何產生器必須讀**同一份**紀錄，這個分歧才會結構上消失。

為什麼是純 dataclass，不是抽象基底
----------------------------------
``core/base/`` 裡四個模組的採用率實測：``fixed_wing_constraints.py``
（純 dataclass、無繼承）擴散到 **18** 個檔；``planner_base.py`` /
``vehicle_base.py`` / ``constraint_base.py``（抽象基底 + 工廠）分別只有
**2 / 1 / 0** 個採用者，而且 ``PlannerFactory.create()`` 全 repo 被呼叫 0 次。
⇒ 這個 repo 裡**資料契約會擴散，呼叫形狀契約不會**。所以這裡只有資料。

慣例
----
* 速度一律 m/s（空速，不是地速）
* 角度一律度
* 高度一律 AGL（相對起飛點），與 ``vtol_profile`` 一致
* 轉彎半徑公式 ``R = V² / (g · tan φ)``，穩態協調轉彎，
  與 ``core/base/fixed_wing_constraints.py`` 同一條
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

__all__ = [
    "AirframeError",
    "Airframe",
    "PIONEER_PNP",
    "HERO_PNP",
    "AIRFRAMES",
]

G = 9.80665


class AirframeError(ValueError):
    """機體參數自相矛盾時丟出。訊息必須指出改哪一個欄位。"""


@dataclass(frozen=True)
class Airframe:
    """一台飛機的規劃相關能力。

    ⚠️ 這裡放的是**規劃用**的值，不是型錄極限。型錄極限要留裕度之後才寫進來，
    每個欄位的裕度記在該機體實例的註解裡。
    """

    name: str

    # ── 三個被 turn_radius>0 混在一起的性質，拆開 ──────────────
    can_hover: bool               # 能不能原地停（多旋翼段）
    needs_runway: bool            # 起降需不需要跑道
    has_turn_radius: bool         # 巡航段受不受最小轉彎半徑限制

    # ── 速度（空速 m/s）────────────────────────────────────────
    cruise_airspeed_mps: float
    min_mission_airspeed_mps: float      # 規劃下限，要高於轉換與失速
    stall_airspeed_mps: float            # 保守失速值
    transition_airspeed_mps: Optional[float] = None   # VTOL 才有

    # ── 姿態與航跡角上限（規劃值，已留裕度）───────────────────
    max_bank_angle_deg: float = 25.0
    max_climb_path_angle_deg: float = 3.0
    max_descent_path_angle_deg: float = 4.0

    # ── 航線設計半徑 ──────────────────────────────────────────
    # ⚠ 理論最小半徑是「物理上轉得過來」，不是「航線該用的」。
    #   實際航線要留裕度（風、姿態建立、L1 追蹤誤差），用這個值。
    design_turn_radius_m: Optional[float] = None

    # ── 質量 ──────────────────────────────────────────────────
    mass_kg: float = 0.0
    payload_kg: float = 0.0

    # ── 續航與航程（型錄基準 + 規劃折扣）──────────────────────
    catalogue_endurance_min: float = 0.0
    catalogue_range_km: float = 0.0
    reserve_fraction: float = 0.30        # 保留電量
    usable_range_fraction: float = 0.55   # 規劃可用比率（對型錄總航程）

    # ── VTOL 起降剖面高度（AGL）───────────────────────────────
    transition_start_agl_m: Optional[float] = None
    transition_complete_agl_m: Optional[float] = None
    min_fw_segment_agl_m: Optional[float] = None

    #: ⚠️ 上面三個高度是否仍為**暫定值**。原廠 .param 讀進來之前恆為 True。
    #: 規劃器可以用，但任何把它當成已驗證數據的報表都要標註。
    transition_agl_is_placeholder: bool = True

    # ──────────────────────────────────────────────────────────
    def __post_init__(self) -> None:
        if self.cruise_airspeed_mps <= 0:
            raise AirframeError(f"{self.name}: 巡航空速必須為正")
        if not (0.0 < self.max_bank_angle_deg < 90.0):
            raise AirframeError(
                f"{self.name}: 坡度上限 {self.max_bank_angle_deg}° 不在 (0, 90) —— "
                "90° 的轉彎半徑是 0，物理上不成立。"
            )
        for f in ("max_climb_path_angle_deg", "max_descent_path_angle_deg"):
            v = getattr(self, f)
            if not (0.0 < v < 90.0):
                raise AirframeError(f"{self.name}: {f} = {v}° 不在 (0, 90)")

        # 速度階梯：巡航 ≥ 規劃下限 > 轉換 ≥ 失速
        if self.min_mission_airspeed_mps > self.cruise_airspeed_mps:
            raise AirframeError(
                f"{self.name}: 規劃下限 {self.min_mission_airspeed_mps} "
                f"高於巡航 {self.cruise_airspeed_mps}，順序反了。"
            )
        if self.min_mission_airspeed_mps <= self.stall_airspeed_mps:
            raise AirframeError(
                f"{self.name}: 規劃下限 {self.min_mission_airspeed_mps} 沒有高於"
                f"失速 {self.stall_airspeed_mps} —— 不可以貼著失速規劃。"
            )
        if (self.transition_airspeed_mps is not None
                and self.transition_airspeed_mps < self.stall_airspeed_mps):
            raise AirframeError(
                f"{self.name}: 轉換空速 {self.transition_airspeed_mps} 低於失速 "
                f"{self.stall_airspeed_mps}"
            )
        if not (0.0 <= self.reserve_fraction < 1.0):
            raise AirframeError(f"{self.name}: 保留電量比率要在 [0, 1)")
        if not (0.0 < self.usable_range_fraction <= 1.0):
            raise AirframeError(f"{self.name}: 可用比率要在 (0, 1]")

        # VTOL 一致性:能懸停就不該需要跑道
        if self.can_hover and self.needs_runway:
            raise AirframeError(
                f"{self.name}: 能懸停卻標成需要跑道 —— 這兩個欄位就是為了"
                "把「能不能停」與「要不要跑道」分開才存在的。"
            )
        if self.transition_airspeed_mps is not None and not self.can_hover:
            raise AirframeError(
                f"{self.name}: 有轉換空速卻不能懸停 —— 轉換是 VTOL 專有的。"
            )

    # ── 轉彎 ──────────────────────────────────────────────────
    @property
    def theoretical_turn_radius_m(self) -> float:
        """穩態協調轉彎的最小半徑 ``R = V² / (g·tan φ)``。

        這是**物理下限**，不是航線該用的半徑。用 :attr:`turn_radius_m`。
        """
        if not self.has_turn_radius:
            return 0.0
        return (self.cruise_airspeed_mps ** 2
                / (G * math.tan(math.radians(self.max_bank_angle_deg))))

    @property
    def turn_radius_m(self) -> float:
        """規劃該用的轉彎半徑:有設計值就用設計值，否則退回理論值。"""
        if not self.has_turn_radius:
            return 0.0
        return (self.design_turn_radius_m
                if self.design_turn_radius_m is not None
                else self.theoretical_turn_radius_m)

    # ── 爬升 / 下降 ───────────────────────────────────────────
    @property
    def climb_rate_mps(self) -> float:
        """由航跡角推出的爬升率 ``V·sin γ``。

        ⚠️ 這是**幾何**推出來的，不是 TECS_CLMB_MAX。飛控參數是能力上限，
        機體航跡角才是實際飛得出來的。
        """
        return self.cruise_airspeed_mps * math.sin(
            math.radians(self.max_climb_path_angle_deg))

    @property
    def descent_rate_mps(self) -> float:
        return self.cruise_airspeed_mps * math.sin(
            math.radians(self.max_descent_path_angle_deg))

    def climb_distance_m(self, height_m: float) -> float:
        """爬升 ``height_m`` 需要的水平距離。"""
        if height_m <= 0:
            return 0.0
        return height_m / math.tan(math.radians(self.max_climb_path_angle_deg))

    def descent_distance_m(self, height_m: float) -> float:
        if height_m <= 0:
            return 0.0
        return height_m / math.tan(math.radians(self.max_descent_path_angle_deg))

    # ── 航程預算 ──────────────────────────────────────────────
    @property
    def usable_range_km(self) -> float:
        """規劃可用航程 = 型錄總航程 × 可用比率。"""
        return self.catalogue_range_km * self.usable_range_fraction

    @property
    def max_symmetric_radius_km(self) -> float:
        """對稱往返任務的最大理論半徑（去程=回程，不含作業航程）。"""
        return self.usable_range_km / 2.0

    @property
    def catalogue_implied_airspeed_mps(self) -> float:
        """型錄隱含的平均速度 = 總航程 / 續航時間。"""
        if self.catalogue_endurance_min <= 0:
            return 0.0
        return (self.catalogue_range_km * 1000.0
                / (self.catalogue_endurance_min * 60.0))

    @property
    def cruises_faster_than_catalogue(self) -> bool:
        """規劃巡航是否**快過**型錄隱含速度。

        ⚠️ True 代表型錄航程是在比規劃巡航更省的速度上量的 ⇒ 照規劃巡航飛
        會拿不到型錄航程，可用比率必須同時吸收保留電量**和**這個速度懲罰。
        兩台機體的保守程度因此可能不一樣，不能套同一個比率就當作等價。
        """
        implied = self.catalogue_implied_airspeed_mps
        return implied > 0.0 and self.cruise_airspeed_mps > implied

    def summary(self) -> str:
        return (
            f"[{self.name}] 巡航 {self.cruise_airspeed_mps:.1f} m/s  "
            f"R_設計 {self.turn_radius_m:.0f} m (理論 {self.theoretical_turn_radius_m:.0f})  "
            f"爬升 {self.climb_rate_mps:.2f} m/s @{self.max_climb_path_angle_deg:.1f}°  "
            f"可用 {self.usable_range_km:.0f} km  半徑 {self.max_symmetric_radius_km:.0f} km"
        )


# ═════════════════════════════════════════════════════════════════════
#  實機
#
#  ⚠️ 下列數值是**規劃值**，已對型錄極限留裕度：
#     爬升 3.5° → 3.0（留 0.5°）；下降 5° → 4.0（留 1°）；
#     坡度 Pioneer 上 22/下 28 → 20；HERO 30 → 25。
#  ⚠️ 轉換/完成高度為暫定值，`transition_agl_is_placeholder=True`，
#     讀到原廠 .param 之後要覆寫並把旗標關掉。
# ═════════════════════════════════════════════════════════════════════

PIONEER_PNP = Airframe(
    name="Pioneer PNP",
    can_hover=True,
    needs_runway=False,
    has_turn_radius=True,
    cruise_airspeed_mps=21.5,
    min_mission_airspeed_mps=18.0,
    stall_airspeed_mps=14.0,
    transition_airspeed_mps=16.0,
    max_bank_angle_deg=20.0,
    max_climb_path_angle_deg=3.0,
    max_descent_path_angle_deg=4.0,
    design_turn_radius_m=160.0,      # 建議 160–200，取下限
    mass_kg=24.6,
    payload_kg=5.0,                  # 5 kg LiDAR 工況
    catalogue_endurance_min=176.0,
    catalogue_range_km=242.0,
    reserve_fraction=0.30,
    usable_range_fraction=0.55,
    transition_start_agl_m=60.0,
    transition_complete_agl_m=100.0,  # 暫定 80–100，取上限（保守）
    min_fw_segment_agl_m=120.0,
)

HERO_PNP = Airframe(
    name="HERO PNP",
    can_hover=True,
    needs_runway=False,
    has_turn_radius=True,
    cruise_airspeed_mps=19.0,
    # ⚠ 原表寫 15–16。取 16 —— 對「最低速度」而言，**大的那個才是保守**
    #   （離失速與轉換空速更遠）。
    min_mission_airspeed_mps=16.0,
    # ⚠ 型錄失速 10 m/s 是特定測試條件，規劃一律用 12 當保守值。
    stall_airspeed_mps=12.0,
    transition_airspeed_mps=14.0,
    max_bank_angle_deg=25.0,
    max_climb_path_angle_deg=3.0,
    max_descent_path_angle_deg=4.0,
    design_turn_radius_m=100.0,      # 建議 100–130，取下限
    mass_kg=6.65,
    payload_kg=0.6,                  # 600 g 測繪工況
    catalogue_endurance_min=136.0,
    catalogue_range_km=144.0,
    reserve_fraction=0.30,
    usable_range_fraction=0.55,
    transition_start_agl_m=50.0,
    transition_complete_agl_m=90.0,   # 暫定 70–90，取上限（保守）
    min_fw_segment_agl_m=100.0,
)

AIRFRAMES = {a.name: a for a in (PIONEER_PNP, HERO_PNP)}
