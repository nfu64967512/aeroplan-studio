"""platform.py — 表 1 PlatformSpec、感測器（硬體／需求分離）、能量模型（模組契約 §2）。

第 0 層模組：只依賴標準函式庫與 numpy；不得 import 任何更高層模組（mission、wind、…）。

內容
----
* :data:`BEAUFORT_UPPER_MPS`、:data:`PROVISIONAL_SUPPLEMENTARY_DEFAULT`（規格 §3.5、§6.2 步 2）。
* :class:`TransitionCmd`、:class:`PlatformSpec`（規格 §3.1–§3.4、D1、D8、D9、D10、D21、D29、D37）。
* :data:`PLATFORM_PIONEER`（構型 A）與 :data:`PLATFORM_HERO`（構型 B）：`platform_params.md` 表 1 逐列常數，
  數值不得更改。
* 感測器（規格 §4、D43）：:class:`LidarHardware`、:class:`CameraHardware`、:class:`TrapezoidFovSpec`
  與 :func:`sensor_model`（回傳實作 :class:`Sensor` 協定之物件）。
* :class:`EnergyModel`（規格 §5.1–§5.7、D6、D20、D39）。

全域約定（契約 §0）：長度 m、時間 s、速度 m/s、能量 Wh、功率 W、質量 kg；介面欄位以 ``_deg`` 結尾者為度，
其餘角度一律 rad。所有高度為 AGL 或 ENU z（本模組不接觸 MSL）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, fields
from typing import Any, Literal, Protocol, runtime_checkable

__all__ = [
    "Range",
    "G_MPS2",
    "BEAUFORT_UPPER_MPS",
    "PROVISIONAL_SUPPLEMENTARY_DEFAULT",
    "TABLE1_KEYS",
    "MissionInputError",
    "PlatformSpecError",
    "TransitionCmd",
    "PlatformSpec",
    "PLATFORM_PIONEER",
    "PLATFORM_HERO",
    "LidarHardware",
    "CameraHardware",
    "TrapezoidFovSpec",
    "SensorHardware",
    "PayloadRequirementLike",
    "Sensor",
    "LidarSensor",
    "CameraSensor",
    "TrapezoidSensor",
    "CombinedSensor",
    "combine_sensors",
    "sensor_model",
    "trapezoid_footprint",
    "crabbed_width_m",
    "EnergyModel",
]

Range = tuple[float, float]
"""(lo, hi) 區間；表 1 以「a–b」表示之列一律以此型別承載。"""

G_MPS2: float = 9.81
"""重力加速度（規格 §5.1 例：24.6·9.81·1.125/0.6 = 453 W）。"""

BEAUFORT_UPPER_MPS: dict[int, float] = {3: 5.4, 4: 7.9, 5: 10.7, 6: 13.8, 7: 17.1}
"""蒲福風級各級上緣風速（規格 §6.2 步 2）；`wind_resistance_public` 只作警告門檻，不作放行門檻（D7）。"""

PROVISIONAL_SUPPLEMENTARY_DEFAULT: frozenset[str] = frozenset({
    "battery_nominal_wh", "reference_battery_wh", "reference_test_depth_of_discharge", "hover_power_ratio",
    "t_tr_out_s", "t_tr_in_s", "t_hover_reserve_s", "a_tr_mps2", "t_hold_s", "transition_ground_run_m",
    "gamma_tr_max_deg", "p_accel_ratio", "vtol_climb_rate_mps", "vtol_descent_rate_mps", "prop_efficiency",
    "idle_descent_ratio", "dv_tr_margin_mps", "k_gamma_auw", "crosswind_site_max_mps", "gust_site_max_mps",
})
"""規格 §3.5 全部補充參數鍵（皆 provisional，可由 `MissionInputs.platform_overrides` 覆寫）。"""

TABLE1_KEYS: tuple[str, ...] = (
    "platform_type", "wingspan_m", "fuselage_length_m", "wing_area_m2",
    "mtow_kg", "reference_auw_kg", "reference_payload_kg", "payload_max_kg",
    "cruise_airspeed_mps", "cruise_airspeed_kmh", "min_mission_airspeed_mps",
    "transition_airspeed_mps", "stall_airspeed_mps", "max_airspeed_mps",
    "max_climb_angle_deg", "planning_climb_angle_deg", "max_descent_angle_deg", "planning_descent_angle_deg",
    "max_fw_climb_rate_mps", "max_fw_descent_rate_mps", "max_bank_angle_deg",
    "turn_radius_theoretical_m", "turn_radius_planning_m",
    "vtol_site_clear_radius_m", "obstacle_horizontal_buffer_m",
    "transition_start_agl_m", "transition_complete_agl_m", "min_fw_operation_agl_m",
    "wind_resistance_public", "reference_endurance_min", "reference_total_range_km",
    "reserve_fraction", "usable_range_fraction", "usable_distance_km", "symmetric_mission_radius_km",
    "airspeed_sensor_validated", "battery_monitor_validated",
    "c2_link_range_km", "terrain_clearance_m", "alternate_landing_sites_required",
)
"""`platform_params.md` 表 1 的 40 個參數鍵（順序同表）。"""


class MissionInputError(ValueError):
    """任務輸入錯誤（契約 §3 定義於 mission.py；本模組因依賴方向無法 import，故在此提供同名基礎類別。

    mission.py 應 ``from .platform import MissionInputError`` 重新匯出，使全套件為同一類別；
    :func:`sensor_model` 在丟出時會優先嘗試取用 mission.py 的定義（延遲 import），以保證 ``except`` 一致。
    """


class PlatformSpecError(ValueError):
    """表 1 常數自洽檢查失敗（`PlatformSpec.__post_init__`）。"""


def _mission_input_error() -> type[ValueError]:
    """回傳全套件應使用的 MissionInputError 類別（優先 mission.py，延遲 import 避免循環）。"""
    try:  # pragma: no cover - 依 mission.py 是否存在而異
        from .mission import MissionInputError as _E  # type: ignore[import-not-found]
        return _E
    except Exception:  # noqa: BLE001
        return MissionInputError


def _is_range(v: Any) -> bool:
    return isinstance(v, tuple) and len(v) == 2 and all(isinstance(x, (int, float)) for x in v)


def _lo(v: float | Range) -> float:
    return float(v[0]) if _is_range(v) else float(v)


def _hi(v: float | Range) -> float:
    return float(v[1]) if _is_range(v) else float(v)


# ---------------------------------------------------------------------------
# TransitionCmd / PlatformSpec
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class TransitionCmd:
    """轉換段指令（規格 §3.4、D8、D29、D37）。

    v_tr_mps      轉換空速指令：`transition_airspeed_mps`（+ `dv_tr_margin_mps` 若空速計未驗證，D8）。
    h_tc_cmd_m    指令完成高度：驗證→區間下緣、未驗證→上緣（D29）。
    h_tc_energy_m 能量計算一律用上緣（D29）。
    h_tc_clear_m  走廊淨空檢查一律用下緣（D29）。
    d_tr_m        轉換水平距離 = v_tr²/(2 a_tr) + v_tr·t_hold（未驗證 A 468、B 384；驗證 384/308），
                  或 `transition_ground_run_m` 覆寫。
    d_acc_m       FW_ACCEL/DECEL 直線長 (Va² − v_tr²)/(2 a_tr)（A 138、B 105）。
    t_acc_s       (Va − v_tr)/a_tr（A 7、B 6）。
    slope_tr_rad  atan((h_tc_hi − H_ts)/d_tr)：旋翼輔助轉換段坡度（A 4.9°、B 5.9°），須 ≤ gamma_tr_max（D37）。
    """
    v_tr_mps: float
    h_tc_cmd_m: float
    h_tc_energy_m: float
    h_tc_clear_m: float
    d_tr_m: float
    d_acc_m: float
    t_acc_s: float
    slope_tr_rad: float


@dataclass(frozen=True, slots=True)
class PlatformSpec:
    """表 1 平台參數（鍵名一字不改）＋ 規格 §3.5 補充參數＋派生量方法（規格 §3.3）。

    方法皆為純函式，只以地速 `v_g` 為輸入（風三角在 wind.py；wind.py 之同名函式為薄包裝）。
    建構（含 `dataclasses.replace` 覆寫）時 `__post_init__` 執行自洽檢查，失敗丟 :class:`PlatformSpecError`。
    """
    name: str
    cls: Literal["A", "B"]
    platform_type: str
    wingspan_m: float
    fuselage_length_m: float
    wing_area_m2: float
    mtow_kg: float
    reference_auw_kg: float
    reference_payload_kg: float
    payload_max_kg: float
    cruise_airspeed_mps: float
    cruise_airspeed_kmh: float
    min_mission_airspeed_mps: float | Range
    transition_airspeed_mps: float
    stall_airspeed_mps: float
    max_airspeed_mps: float | None
    max_climb_angle_deg: float
    planning_climb_angle_deg: float
    max_descent_angle_deg: float
    planning_descent_angle_deg: float
    max_fw_climb_rate_mps: float
    max_fw_descent_rate_mps: float
    max_bank_angle_deg: float
    turn_radius_theoretical_m: float
    turn_radius_planning_m: float
    vtol_site_clear_radius_m: float
    obstacle_horizontal_buffer_m: float
    transition_start_agl_m: float
    transition_complete_agl_m: Range
    min_fw_operation_agl_m: float
    wind_resistance_public: int
    reference_endurance_min: float
    reference_total_range_km: float
    reserve_fraction: float
    usable_range_fraction: Range
    usable_distance_km: Range
    symmetric_mission_radius_km: Range
    airspeed_sensor_validated: bool
    battery_monitor_validated: bool
    c2_link_range_km: float | None
    terrain_clearance_m: float | None
    alternate_landing_sites_required: bool
    # --- 補充（provisional，規格 §3.5）---
    battery_nominal_wh: float | None = None          # 示例 1500/420，使用時警告 battery_nominal_example
    reference_battery_wh: float | None = None        # 基準測試電池（D39）；None → = nominal 並警告 reference_battery_assumed
    reference_test_depth_of_discharge: float = 1.0   # f_dep（provisional）
    hover_power_ratio: float = 4.0
    t_tr_out_s: float = 20.0
    t_tr_in_s: float = 25.0
    t_hover_reserve_s: float = 15.0
    a_tr_mps2: float = 0.5
    t_hold_s: float = 8.0
    transition_ground_run_m: float | None = None     # 可選覆寫 d_tr；None → 公式（規格 §3.4）
    gamma_tr_max_deg: float = 8.0                    # 旋翼輔助轉換段坡度上限（D37）
    p_accel_ratio: float = 1.2                       # FW_ACCEL/DECEL 功率 = p·P_c
    vtol_climb_rate_mps: float = 2.0
    vtol_descent_rate_mps: float = 1.0               # 最後 10 m 0.5 m/s
    prop_efficiency: float = 0.6
    idle_descent_ratio: float = 0.3
    dv_tr_margin_mps: float = 2.0
    k_gamma_auw: float = 2.0
    crosswind_site_max_mps: Range = (6.0, 8.0)       # (unvalidated, validated)；HERO (5.0, 6.0)
    gust_site_max_mps: Range = (8.0, 10.0)
    provisional: frozenset[str] = frozenset()        # 表 1 之暫定鍵
    provisional_supplementary: frozenset[str] = PROVISIONAL_SUPPLEMENTARY_DEFAULT   # §3.5 全部鍵

    # ------------------------------------------------------------------ 檢查
    def __post_init__(self) -> None:
        """表 1 自洽檢查（規格 §3.3）。容差：kmh ±0.1；爬升／下降率 ±5%；R_theo ±10%；距離 ±1 km。"""
        err: list[str] = []
        va = float(self.cruise_airspeed_mps)
        if self.cls not in ("A", "B"):
            err.append(f"cls 必須為 'A' 或 'B'，得 {self.cls!r}")
        for key in ("transition_complete_agl_m", "usable_range_fraction", "usable_distance_km",
                    "symmetric_mission_radius_km", "crosswind_site_max_mps", "gust_site_max_mps"):
            v = getattr(self, key)
            if not _is_range(v) or v[0] > v[1]:
                err.append(f"{key} 須為 (lo, hi) 且 lo ≤ hi，得 {v!r}")
        vm = self.min_mission_airspeed_mps
        if _is_range(vm) and vm[0] > vm[1]:
            err.append(f"min_mission_airspeed_mps 區間 lo ≤ hi 不成立：{vm!r}")
        # 1) kmh == 3.6·mps（±0.1）
        if abs(self.cruise_airspeed_kmh - 3.6 * va) > 0.1:
            err.append(f"cruise_airspeed_kmh {self.cruise_airspeed_kmh} ≠ 3.6×{va} = {3.6 * va:.2f}（±0.1）")
        # 2) Va·sin(γ_c) ≈ max_fw_climb_rate（±5%）
        cr = va * math.sin(math.radians(self.planning_climb_angle_deg))
        if abs(cr - self.max_fw_climb_rate_mps) > 0.05 * self.max_fw_climb_rate_mps:
            err.append(f"Va·sin({self.planning_climb_angle_deg}°) = {cr:.3f} vs max_fw_climb_rate_mps "
                       f"{self.max_fw_climb_rate_mps}（±5%）")
        # 3) Va·sin(γ_d) ≈ max_fw_descent_rate（±5%）
        dr = va * math.sin(math.radians(self.planning_descent_angle_deg))
        if abs(dr - self.max_fw_descent_rate_mps) > 0.05 * self.max_fw_descent_rate_mps:
            err.append(f"Va·sin({self.planning_descent_angle_deg}°) = {dr:.3f} vs max_fw_descent_rate_mps "
                       f"{self.max_fw_descent_rate_mps}（±5%）")
        # 4) Va²/(g tanφ) ≈ R_theo（±10%）
        r_theo = va * va / (G_MPS2 * math.tan(math.radians(self.max_bank_angle_deg)))
        if abs(r_theo - self.turn_radius_theoretical_m) > 0.10 * self.turn_radius_theoretical_m:
            err.append(f"Va²/(g·tanφ) = {r_theo:.1f} vs turn_radius_theoretical_m "
                       f"{self.turn_radius_theoretical_m}（±10%）")
        # 5) R_plan ≥ R_theo
        if self.turn_radius_planning_m < self.turn_radius_theoretical_m:
            err.append("turn_radius_planning_m < turn_radius_theoretical_m")
        # 6) usable_distance ≈ fraction × total_range（±1 km，逐上下緣）
        if _is_range(self.usable_range_fraction) and _is_range(self.usable_distance_km):
            for i, tag in ((0, "lo"), (1, "hi")):
                expect = self.usable_range_fraction[i] * self.reference_total_range_km
                if abs(expect - self.usable_distance_km[i]) > 1.0:
                    err.append(f"usable_distance_km[{tag}] {self.usable_distance_km[i]} vs "
                               f"fraction×range = {expect:.1f}（±1 km）")
        # 7) symmetric_radius ≈ usable_distance/2（±1 km）
        if _is_range(self.symmetric_mission_radius_km) and _is_range(self.usable_distance_km):
            for i, tag in ((0, "lo"), (1, "hi")):
                expect = self.usable_distance_km[i] / 2.0
                if abs(expect - self.symmetric_mission_radius_km[i]) > 1.0:
                    err.append(f"symmetric_mission_radius_km[{tag}] {self.symmetric_mission_radius_km[i]} vs "
                               f"usable/2 = {expect:.1f}（±1 km）")
        # 8) 速度序：V_s ≤ V_tr ≤ V_a,min(lo) 且 V_a,min(hi) ≤ V_a ≤ V_max
        if not (self.stall_airspeed_mps <= self.transition_airspeed_mps <= _lo(vm) and _hi(vm) <= va):
            err.append("速度序 stall ≤ transition ≤ min_mission ≤ cruise 不成立")
        if self.max_airspeed_mps is not None and va > self.max_airspeed_mps:
            err.append("cruise_airspeed_mps > max_airspeed_mps")
        # 9) 質量序
        if self.reference_auw_kg > self.mtow_kg:
            err.append("reference_auw_kg > mtow_kg")
        if self.reference_payload_kg > self.reference_auw_kg:
            err.append("reference_payload_kg > reference_auw_kg")
        if not (0.0 <= self.reserve_fraction < 1.0):
            err.append("reserve_fraction 須在 [0,1)")
        if err:
            raise PlatformSpecError(f"PlatformSpec {self.name!r} 自洽檢查失敗：" + "；".join(err))

    # ------------------------------------------------------------------ 空速
    def v_min_mission(self) -> float:
        """規劃用任務空速下限 V_a,min：Range → 上緣（HERO 16.0；D9）。"""
        return _hi(self.min_mission_airspeed_mps)

    def v_min_absolute(self) -> float:
        """遙測絕對下限：Range → 下緣（HERO 15.0；僅遙測，不用於規劃；D9）。"""
        return _lo(self.min_mission_airspeed_mps)

    # ------------------------------------------------------------------ 轉彎
    def r_eff(self, wind_speed_mps: float) -> float:
        """風修正有效轉彎半徑 R_eff(W) = max{R_plan, R_theo (1+W/Va)²}（規格 §3.2、D1；擺線最大曲率半徑）。"""
        w = max(0.0, float(wind_speed_mps))
        return max(self.turn_radius_planning_m,
                   self.turn_radius_theoretical_m * (1.0 + w / self.cruise_airspeed_mps) ** 2)

    def w_star(self) -> float:
        """R_plan 內建裕度可吸收之風速 W* = Va(√(R_plan/R_theo) − 1)（A 5.17、B 5.03 m/s；規格 §3.2）。"""
        return self.cruise_airspeed_mps * (math.sqrt(self.turn_radius_planning_m / self.turn_radius_theoretical_m) - 1.0)

    # ------------------------------------------------------------------ 坡度／失速（AUW 修正，D21）
    def _auw_factor(self, auw_kg: float) -> float:
        """clip(1 − k_γ(m/m_ref − 1), 0.6, 1)。"""
        f = 1.0 - self.k_gamma_auw * (float(auw_kg) / self.reference_auw_kg - 1.0)
        return min(1.0, max(0.6, f))

    def gamma_c_auw(self, auw_kg: float) -> float:
        """規劃爬升空氣坡度 γ_c(m)（rad）：tanγ_c(m) = tanγ_c^plan · clip(1 − k_γ(m/m_ref − 1), 0.6, 1)（D21）。"""
        return math.atan(math.tan(math.radians(self.planning_climb_angle_deg)) * self._auw_factor(auw_kg))

    def gamma_d(self) -> float:
        """規劃下降空氣坡度 γ_d（rad；不隨 AUW，D21）。"""
        return math.radians(self.planning_descent_angle_deg)

    def stall_speed(self, auw_kg: float) -> float:
        """失速空速 V_s(m) = V_s √(m/m_ref)（D21）。"""
        return self.stall_airspeed_mps * math.sqrt(float(auw_kg) / self.reference_auw_kg)

    def stall_margin_ok(self, auw_kg: float) -> bool:
        """失速裕度 V_a,min ≥ 1.2·V_s(m)（D9、D21；A@24.9：16.9 ≤ 18 ✓、B@7.0：14.8 ≤ 16 ✓）。"""
        return self.v_min_mission() >= 1.2 * self.stall_speed(auw_kg)

    def climb_rate_mps(self, auw_kg: float) -> float:
        """爬升率 Va·sinγ_c(m)（與風無關；A@m_ref 1.125 m/s）。"""
        return self.cruise_airspeed_mps * math.sin(self.gamma_c_auw(auw_kg))

    def tan_gamma_ground(self, which: Literal["climb", "descent"], v_g_mps: float, auw_kg: float, arc: bool) -> float:
        """地面坡度正切（規格 §3.2）：

        climb   : tanγ^g_c = Va·sinγ_c(m)·κ / V_g，弧段 κ = cos φ_max（傾角下升力需求），直線 κ = 1。
        descent : tanγ^g_d = Va·sinγ_d / V_g（不隨 AUW、不折減）。
        V_g ≤ 0（該方向不可行）→ 回傳 inf。
        """
        vg = float(v_g_mps)
        if vg <= 0.0:
            return math.inf
        if which == "climb":
            kappa = math.cos(math.radians(self.max_bank_angle_deg)) if arc else 1.0
            return self.climb_rate_mps(auw_kg) * kappa / vg
        if which == "descent":
            return self.cruise_airspeed_mps * math.sin(self.gamma_d()) / vg
        raise ValueError(f"which 須為 'climb' 或 'descent'，得 {which!r}")

    def climb_dist_m(self, dh_m: float, v_g_max_mps: float, auw_kg: float) -> float:
        """爬升 dh 所需水平距離 dh/tanγ^g_c（保守：以順風最大 V_g；直線 κ=1）。

        呼叫一律用關鍵字：``climb_dist_m(dh_m=60, v_g_max_mps=26.5, auw_kg=24.6)`` → 1413 m（規格 §11.6）。
        dh ≤ 0 → 0。
        """
        if dh_m <= 0.0:
            return 0.0
        return float(dh_m) / self.tan_gamma_ground("climb", v_g_max_mps, auw_kg, arc=False)

    def descent_dist_m(self, dh_m: float, v_g_max_mps: float) -> float:
        """下降 dh 所需水平距離 dh/tanγ^g_d（保守：順風最大 V_g；A dh=50 @26.5 → 883 m，dh=60 → 1060 m；規格 §11.7）。"""
        if dh_m <= 0.0:
            return 0.0
        return float(dh_m) / self.tan_gamma_ground("descent", v_g_max_mps, self.reference_auw_kg, arc=False)

    # ------------------------------------------------------------------ 轉換段
    def transition_cmd(self, *, transition_profile: Literal["rotor_assisted", "fw_slope"] = "rotor_assisted") -> TransitionCmd:
        """轉換段指令（規格 §3.4、D8、D29、D37）。

        * 未驗證空速計：V_tr,cmd = V_tr + dV_tr_margin、H_tc,cmd = 區間上緣；驗證後 V_tr、下緣。
        * d_tr = `transition_ground_run_m`（覆寫）或 V_tr,cmd²/(2a_tr) + V_tr,cmd·t_hold（A 468、B 384 m）。
        * `transition_profile="fw_slope"`（旋翼不輔助）：d_tr ← max(d_tr, (H_tc,hi − H_ts)/tanγ_c^plan)（A 763 m）。
        * d_acc = (Va² − V_tr,cmd²)/(2a_tr)、t_acc = (Va − V_tr,cmd)/a_tr（A 138 m／7 s、B 105 m／6 s）。
        * slope_tr = atan((H_tc,hi − H_ts)/d_tr)（A 4.9°、B 5.9°；須 ≤ gamma_tr_max_deg，由呼叫端檢查）。
        """
        validated = bool(self.airspeed_sensor_validated)
        v_tr = self.transition_airspeed_mps + (0.0 if validated else self.dv_tr_margin_mps)
        lo, hi = float(self.transition_complete_agl_m[0]), float(self.transition_complete_agl_m[1])
        h_cmd = lo if validated else hi
        if self.transition_ground_run_m is not None:
            d_tr = float(self.transition_ground_run_m)
        else:
            d_tr = v_tr * v_tr / (2.0 * self.a_tr_mps2) + v_tr * self.t_hold_s
        if transition_profile == "fw_slope":
            d_tr = max(d_tr, (hi - self.transition_start_agl_m) / math.tan(math.radians(self.planning_climb_angle_deg)))
        elif transition_profile != "rotor_assisted":
            raise ValueError(f"transition_profile 須為 'rotor_assisted' 或 'fw_slope'，得 {transition_profile!r}")
        va = self.cruise_airspeed_mps
        d_acc = max(0.0, (va * va - v_tr * v_tr) / (2.0 * self.a_tr_mps2))
        t_acc = max(0.0, (va - v_tr) / self.a_tr_mps2)
        slope = math.atan2(hi - self.transition_start_agl_m, d_tr) if d_tr > 0 else math.pi / 2
        return TransitionCmd(v_tr_mps=v_tr, h_tc_cmd_m=h_cmd, h_tc_energy_m=hi, h_tc_clear_m=lo,
                             d_tr_m=d_tr, d_acc_m=d_acc, t_acc_s=t_acc, slope_tr_rad=slope)

    # ------------------------------------------------------------------ 其他
    def h_ceil(self, max_agl_m: float | None, waiver_max_agl_m: float | None) -> float | None:
        """空域 AGL 上限 h_ceil = waiver_max_agl_m if given else max_agl_m（規格 §4.6、D12）；皆 None → None。"""
        return float(waiver_max_agl_m) if waiver_max_agl_m is not None else (None if max_agl_m is None else float(max_agl_m))

    def wind_limit_public_mps(self) -> float:
        """能力宣稱之風速上緣 BEAUFORT_UPPER_MPS[wind_resistance_public]（Beaufort 5 → 10.7；僅警告門檻，D7）。"""
        try:
            return BEAUFORT_UPPER_MPS[int(self.wind_resistance_public)]
        except KeyError as e:
            raise ValueError(f"wind_resistance_public={self.wind_resistance_public} 不在 BEAUFORT_UPPER_MPS 表內") from e

    def empty_mass_kg(self) -> float:
        """空重推估 m_empty = reference_auw_kg − reference_payload_kg（酬載推估用；規格 §3.1）。"""
        return self.reference_auw_kg - self.reference_payload_kg

    def hover_descent_time_s(self, h_ts_m: float | None = None) -> float:
        """懸停下降時間 t_dn = (H_ts − 10)/v_z,dn + 10/0.5（最後 10 m 以 0.5 m/s；A 70、B 60 s）。

        與 altitude.landing_corridor 之 seg_v 共用（規格 §5.3、§11.7）。h_ts ≤ 10 → h_ts/0.5。
        """
        h = self.transition_start_agl_m if h_ts_m is None else float(h_ts_m)
        if h <= 10.0:
            return max(0.0, h) / 0.5
        return (h - 10.0) / self.vtol_descent_rate_mps + 10.0 / 0.5


# ---------------------------------------------------------------------------
# 表 1 常數（platform_params.md；數值不得更改）
# ---------------------------------------------------------------------------

_PROVISIONAL_A: frozenset[str] = frozenset({
    "vtol_site_clear_radius_m", "obstacle_horizontal_buffer_m", "transition_start_agl_m",
    "transition_complete_agl_m", "c2_link_range_km", "terrain_clearance_m",
})

PLATFORM_PIONEER: PlatformSpec = PlatformSpec(
    name="Pioneer PNP", cls="A",
    platform_type="lift_and_cruise_vtol_4plus1",
    wingspan_m=3.20, fuselage_length_m=1.68, wing_area_m2=1.00,
    mtow_kg=24.9, reference_auw_kg=24.6, reference_payload_kg=5.0, payload_max_kg=5.0,
    cruise_airspeed_mps=21.5, cruise_airspeed_kmh=77.4,
    min_mission_airspeed_mps=18.0,
    transition_airspeed_mps=16.0, stall_airspeed_mps=14.0,
    max_airspeed_mps=35.0,
    max_climb_angle_deg=3.5, planning_climb_angle_deg=3.0,
    max_descent_angle_deg=5.0, planning_descent_angle_deg=4.0,
    max_fw_climb_rate_mps=1.13, max_fw_descent_rate_mps=1.50,
    max_bank_angle_deg=20.0,
    turn_radius_theoretical_m=130.0, turn_radius_planning_m=200.0,
    vtol_site_clear_radius_m=8.0, obstacle_horizontal_buffer_m=30.0,
    transition_start_agl_m=60.0, transition_complete_agl_m=(80.0, 100.0),
    min_fw_operation_agl_m=120.0,
    wind_resistance_public=5,
    reference_endurance_min=176.0, reference_total_range_km=242.0,
    reserve_fraction=0.30, usable_range_fraction=(0.55, 0.60),
    usable_distance_km=(133.0, 145.0), symmetric_mission_radius_km=(66.0, 72.0),
    airspeed_sensor_validated=False, battery_monitor_validated=False,
    c2_link_range_km=None, terrain_clearance_m=None,
    alternate_landing_sites_required=True,
    crosswind_site_max_mps=(6.0, 8.0), gust_site_max_mps=(8.0, 10.0),
    provisional=_PROVISIONAL_A,
)
"""構型 A（表 1 Pioneer PNP 欄）。"""

PLATFORM_HERO: PlatformSpec = PlatformSpec(
    name="HERO PNP", cls="B",
    platform_type="lift_and_cruise_vtol_4plus1",
    wingspan_m=2.18, fuselage_length_m=1.14, wing_area_m2=0.53,
    mtow_kg=7.0, reference_auw_kg=6.65, reference_payload_kg=0.6, payload_max_kg=1.0,   # 「1.0 以下」→ 1.0（provisional）
    cruise_airspeed_mps=19.0, cruise_airspeed_kmh=68.4,
    min_mission_airspeed_mps=(15.0, 16.0),          # 規劃 16.0；15.0 僅遙測（D9）
    transition_airspeed_mps=14.0, stall_airspeed_mps=12.0,
    max_airspeed_mps=None,                          # 待實機確認（D10）
    max_climb_angle_deg=3.5, planning_climb_angle_deg=3.0,
    max_descent_angle_deg=5.0, planning_descent_angle_deg=4.0,
    max_fw_climb_rate_mps=1.00, max_fw_descent_rate_mps=1.33,
    max_bank_angle_deg=25.0,
    turn_radius_theoretical_m=75.0, turn_radius_planning_m=120.0,
    vtol_site_clear_radius_m=6.0, obstacle_horizontal_buffer_m=25.0,
    transition_start_agl_m=50.0, transition_complete_agl_m=(70.0, 90.0),
    min_fw_operation_agl_m=100.0,
    wind_resistance_public=5,
    reference_endurance_min=136.0, reference_total_range_km=144.0,
    reserve_fraction=0.30, usable_range_fraction=(0.55, 0.60),
    usable_distance_km=(79.0, 86.0), symmetric_mission_radius_km=(39.0, 43.0),
    airspeed_sensor_validated=False, battery_monitor_validated=False,
    c2_link_range_km=None, terrain_clearance_m=None,
    alternate_landing_sites_required=True,
    crosswind_site_max_mps=(5.0, 6.0), gust_site_max_mps=(8.0, 10.0),
    transition_ground_run_m=None,                   # 公式 384；規格 §3.5 舊值 300 僅作覆寫示例
    provisional=_PROVISIONAL_A | frozenset({"max_airspeed_mps", "payload_max_kg"}),
)
"""構型 B（表 1 HERO PNP 欄）。"""


# ---------------------------------------------------------------------------
# 感測器：硬體與需求分離（D43）
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class LidarHardware:
    """LiDAR 硬體（規格 §4.1）。預設值為示例（表 1／表 2 未提供），使用時 PRECHECK 警告 sensor_spec_default_example。"""
    kind: Literal["lidar"] = "lidar"
    fov_full_deg: float = 120.0
    fov_usable_deg: float = 70.0
    point_rate_hz: float = 600_000.0
    return_efficiency: float = 0.5
    max_range_m: float = 450.0
    range_margin: float = 0.9
    is_default_example: bool = True


@dataclass(frozen=True, slots=True)
class CameraHardware:
    """RGB 相機硬體（規格 §4.2）。預設值為示例。"""
    kind: Literal["rgb"] = "rgb"
    focal_length_mm: float = 16.0
    sensor_width_mm: float = 23.5
    sensor_height_mm: float = 15.6
    pixel_pitch_um: float = 3.9
    max_shutter_rate_hz: float = 1.0
    is_default_example: bool = True


@dataclass(frozen=True, slots=True)
class TrapezoidFovSpec:
    """論文梯形 FOV（Eq.(2)(3)；規格 §4.3；`fov_model="trapezoid"`）。角度皆為度、須在 (0°,180°)。"""
    kind: Literal["lidar", "rgb"]
    mount_angle_deg: float
    v_fov_deg: float
    h_fov_deg: float
    is_default_example: bool = False


SensorHardware = LidarHardware | CameraHardware | TrapezoidFovSpec


@runtime_checkable
class PayloadRequirementLike(Protocol):
    """`mission.PayloadRequirement` 之鴨子型別視圖（本模組不 import mission）。

    只讀取以下屬性；缺省（None）時依規格 §16.2 預設：side_overlap 0.25(lidar)/0.30(rgb)、forward_overlap 0.70、
    shutter_rate_hz → 硬體上限、density_pt_m2 20、gsd_required_m 0.05、h_target_agl_m → Sensor.h_target_m 預設。
    """
    kind: str
    gsd_required_m: float | None
    density_pt_m2: float | None
    side_overlap: float | None
    forward_overlap: float
    shutter_rate_hz: float | None
    h_target_agl_m: float | None


class Sensor(Protocol):
    """由 :func:`sensor_model` 產生之感測器模型（區域需求覆寫硬體預設；契約 §2）。"""
    kind: str
    hw: SensorHardware
    req: Any

    def swath_m(self, h_agl_m: float) -> float: ...
    def footprint_len_m(self, h_agl_m: float) -> float: ...
    def strip_spacing_m(self, h_agl_m: float) -> float: ...
    def max_ground_speed_mps(self, h_agl_m: float) -> float: ...
    def gsd_m(self, h_agl_m: float) -> float | None: ...
    def agl_window(self) -> tuple[float, float]: ...
    def h_target_m(self, h_min_fw_m: float, dh_band_min_m: float, k_gsd_relax: float) -> float: ...
    def gsd_attainable(self, h_min_fw_m: float, dh_band_min_m: float) -> bool: ...
    def end_extension_m(self, v_g_max_mps: float, t_settle_s: float) -> float: ...


_DEFAULT_SIDE_OVERLAP: dict[str, float] = {"lidar": 0.25, "rgb": 0.30}   # D25
_DEFAULT_FORWARD_OVERLAP: float = 0.70                                      # D25
_DEFAULT_DENSITY_PT_M2: float = 20.0                                        # 規格 §16.2
_DEFAULT_GSD_REQUIRED_M: float = 0.05                                       # 規格 §16.2
_DEFAULT_H_TARGET_LIDAR_M: float = 150.0                                    # 規格 §4.6


@dataclass(frozen=True, slots=True)
class _ResolvedReq:
    """需求解析結果（硬體預設 ← 區域需求覆寫）。`defaults_applied` 列出採用預設之鍵（供警告）。"""
    side_overlap: float
    forward_overlap: float
    shutter_rate_hz: float | None
    density_pt_m2: float | None
    gsd_required_m: float | None
    h_target_agl_m: float | None
    defaults_applied: frozenset[str]


def _resolve_req(hw: SensorHardware, req: Any) -> _ResolvedReq:
    E = _mission_input_error()
    kind = getattr(req, "kind", None) or hw.kind
    if kind != hw.kind:
        raise E(f"PayloadRequirement.kind={kind!r} 與感測器硬體 kind={hw.kind!r} 不符")
    applied: set[str] = set()
    side = getattr(req, "side_overlap", None)
    if side is None:
        side = _DEFAULT_SIDE_OVERLAP[kind]
        applied.add("side_overlap")
    fwd = getattr(req, "forward_overlap", None)
    if fwd is None:
        fwd = _DEFAULT_FORWARD_OVERLAP
        applied.add("forward_overlap")
    if not (0.0 <= side < 1.0) or not (0.0 <= fwd < 1.0):
        raise E(f"重疊率須在 [0,1)：side_overlap={side}, forward_overlap={fwd}")
    hw_max_shutter = getattr(hw, "max_shutter_rate_hz", None)
    shutter = getattr(req, "shutter_rate_hz", None)
    if shutter is None:
        shutter = hw_max_shutter
        applied.add("shutter_rate_hz")
    elif hw_max_shutter is not None and shutter > hw_max_shutter + 1e-12:
        raise E(f"shutter_rate_hz={shutter} 超過硬體上限 max_shutter_rate_hz={hw_max_shutter}")
    elif shutter <= 0.0:
        raise E(f"shutter_rate_hz 須 > 0，得 {shutter}")
    density = getattr(req, "density_pt_m2", None)
    gsd = getattr(req, "gsd_required_m", None)
    if kind == "lidar" and density is None and not isinstance(hw, TrapezoidFovSpec):
        density = _DEFAULT_DENSITY_PT_M2
        applied.add("density_pt_m2")
    if kind == "rgb" and gsd is None and isinstance(hw, CameraHardware):
        gsd = _DEFAULT_GSD_REQUIRED_M
        applied.add("gsd_required_m")
    if density is not None and density <= 0.0:
        raise E(f"density_pt_m2 須 > 0，得 {density}")
    if gsd is not None and gsd <= 0.0:
        raise E(f"gsd_required_m 須 > 0，得 {gsd}")
    h_t = getattr(req, "h_target_agl_m", None)
    if h_t is not None and h_t <= 0.0:
        raise E(f"h_target_agl_m 須 > 0，得 {h_t}")
    return _ResolvedReq(side_overlap=float(side), forward_overlap=float(fwd),
                        shutter_rate_hz=None if shutter is None else float(shutter),
                        density_pt_m2=None if density is None else float(density),
                        gsd_required_m=None if gsd is None else float(gsd),
                        h_target_agl_m=None if h_t is None else float(h_t),
                        defaults_applied=frozenset(applied))


def _end_extension(v_g_max_mps: float, t_settle_s: float, footprint_len_m: float, forward_overlap: float) -> float:
    """e_ext = max(V_g,max·t_settle, l(1 − o_fwd))（規格 §4.5）。"""
    return max(float(v_g_max_mps) * float(t_settle_s), footprint_len_m * (1.0 - forward_overlap))


def crabbed_width_m(swath_m: float, footprint_len_m: float, crab_rad: float) -> float:
    """偏流 ψ 後機身固定足跡（橫 w、縱 l）的保守橫向可用寬 w_eff = w cosψ − l sinψ（建圖模式 crab_margin）。

    足跡隨機頭轉 ψ，航跡法向上「整段航線都看得到」的帶寬只剩 w cosψ − l sinψ。HQ6@120 m（w 125.7、l 94.2）、
    V_a 19、W 5 → ψ 15.26°、w_eff ≈ 96.5 m。ψ = 0 → 原樣回傳 w（舊行為逐位元不變）。"""
    if not crab_rad:
        return float(swath_m)
    return float(swath_m) * math.cos(crab_rad) - float(footprint_len_m) * math.sin(crab_rad)


@dataclass(frozen=True, slots=True)
class LidarSensor:
    """LiDAR 感測器模型（規格 §4.1）：w = 2h tan(θ_use/2)、s = w(1−o_side)、V_cap = f_p η_ret/(w ρ_req)、
    h_max = k_rng r_max cos(θ_use/2)。示例硬體 h=150：w 210.1、s 157.5 m；h_max 332 m。"""
    kind: str
    hw: LidarHardware
    req: Any
    r: _ResolvedReq
    crab_rad: float = 0.0          # 建圖模式偏流餘裕 ψ（0 ＝ 舊行為）；見 crabbed_width_m

    @property
    def defaults_applied(self) -> frozenset[str]:
        return self.r.defaults_applied

    def swath_m(self, h_agl_m: float) -> float:
        return 2.0 * float(h_agl_m) * math.tan(math.radians(self.hw.fov_usable_deg) / 2.0)

    def footprint_len_m(self, h_agl_m: float) -> float:
        """線掃描：沿航跡足跡連續，前向長度取 0（e_ext 只由 V_g·t_settle 決定；規格 §W.3 e_ext,A = 53 m）。"""
        return 0.0

    def strip_spacing_m(self, h_agl_m: float) -> float:
        if self.crab_rad:
            return crabbed_width_m(self.swath_m(h_agl_m), 0.0, self.crab_rad) * (1.0 - self.r.side_overlap)
        return self.swath_m(h_agl_m) * (1.0 - self.r.side_overlap)

    def max_ground_speed_mps(self, h_agl_m: float) -> float:
        """V_g^cap = f_p η_ret/(w(h) ρ_req)；無密度需求 → inf。"""
        if self.r.density_pt_m2 is None:
            return math.inf
        w = self.swath_m(h_agl_m)
        if w <= 0.0:
            return math.inf
        return self.hw.point_rate_hz * self.hw.return_efficiency / (w * self.r.density_pt_m2)

    def gsd_m(self, h_agl_m: float) -> float | None:
        return None

    def agl_window(self) -> tuple[float, float]:
        """(0, k_rng·r_max·cos(θ_use/2))。"""
        return (0.0, self.hw.range_margin * self.hw.max_range_m * math.cos(math.radians(self.hw.fov_usable_deg) / 2.0))

    def h_target_m(self, h_min_fw_m: float, dh_band_min_m: float, k_gsd_relax: float) -> float:
        """req.h_target_agl_m 或預設 150 m（規格 §4.6；clip 至高度帶由 decompose 負責）。"""
        return self.r.h_target_agl_m if self.r.h_target_agl_m is not None else _DEFAULT_H_TARGET_LIDAR_M

    def gsd_attainable(self, h_min_fw_m: float, dh_band_min_m: float) -> bool:
        """h_max ≥ h_minfw + Δh_band,min（規格 §4.2 之通式；LiDAR 以測距上限為 h_max）。"""
        return self.agl_window()[1] >= float(h_min_fw_m) + float(dh_band_min_m)

    def end_extension_m(self, v_g_max_mps: float, t_settle_s: float, h_agl_m: float | None = None) -> float:
        return _end_extension(v_g_max_mps, t_settle_s, 0.0, self.r.forward_overlap)


@dataclass(frozen=True, slots=True)
class CameraSensor:
    """相機感測器模型（規格 §4.2）：w = h S_w/f、l = h S_h/f、GSD = h p/f、h_max = GSD_req f/p、
    s = w(1−o_side)、V_cap = l(1−o_fwd) f_s。示例硬體 GSD_req 5 cm：h_max 205 m；h=100：w 146.9、s 102.8、
    l 97.5 m、GSD 2.44 cm、V_cap 29.3 m/s。"""
    kind: str
    hw: CameraHardware
    req: Any
    r: _ResolvedReq
    crab_rad: float = 0.0          # 建圖模式偏流餘裕 ψ（0 ＝ 舊行為）：s = (w cosψ − l sinψ)(1 − o_side)

    @property
    def defaults_applied(self) -> frozenset[str]:
        return self.r.defaults_applied

    def swath_m(self, h_agl_m: float) -> float:
        return float(h_agl_m) * self.hw.sensor_width_mm / self.hw.focal_length_mm

    def footprint_len_m(self, h_agl_m: float) -> float:
        return float(h_agl_m) * self.hw.sensor_height_mm / self.hw.focal_length_mm

    def strip_spacing_m(self, h_agl_m: float) -> float:
        if self.crab_rad:
            return crabbed_width_m(self.swath_m(h_agl_m), self.footprint_len_m(h_agl_m),
                                   self.crab_rad) * (1.0 - self.r.side_overlap)
        return self.swath_m(h_agl_m) * (1.0 - self.r.side_overlap)

    def max_ground_speed_mps(self, h_agl_m: float) -> float:
        """V_g^cap = l(h)(1 − o_fwd) f_s；無快門率 → inf。"""
        if self.r.shutter_rate_hz is None:
            return math.inf
        return self.footprint_len_m(h_agl_m) * (1.0 - self.r.forward_overlap) * self.r.shutter_rate_hz

    def gsd_m(self, h_agl_m: float) -> float | None:
        """GSD(h) = h·p/f（m）。"""
        return float(h_agl_m) * (self.hw.pixel_pitch_um * 1e-6) / (self.hw.focal_length_mm * 1e-3)

    def agl_window(self) -> tuple[float, float]:
        """(0, h_max)，h_max = GSD_req·f/p；無 GSD 需求 → inf。"""
        if self.r.gsd_required_m is None:
            return (0.0, math.inf)
        return (0.0, self.r.gsd_required_m * (self.hw.focal_length_mm * 1e-3) / (self.hw.pixel_pitch_um * 1e-6))

    def h_target_m(self, h_min_fw_m: float, dh_band_min_m: float, k_gsd_relax: float) -> float:
        """req.h_target_agl_m 或預設 min(h_max/k_gsd_relax, h_max − Δh_band,min)（示例硬體 ≈171 m；規格 §4.6）。

        不取 h_max 本身（否則貼著上界飛、微小地形起伏即 INFEASIBLE(OPER)）。h_max = inf → 回傳 inf（由 decompose clip）。
        """
        if self.r.h_target_agl_m is not None:
            return self.r.h_target_agl_m
        h_max = self.agl_window()[1]
        if not math.isfinite(h_max):
            return math.inf
        return min(h_max / float(k_gsd_relax), h_max - float(dh_band_min_m))

    def gsd_attainable(self, h_min_fw_m: float, dh_band_min_m: float) -> bool:
        """GSD 可達性：h_max(GSD_req) ≥ h_minfw + Δh_band,min（示例硬體需 GSD_req ≥ 2.93 cm；規格 §4.2）。"""
        return self.agl_window()[1] >= float(h_min_fw_m) + float(dh_band_min_m)

    def end_extension_m(self, v_g_max_mps: float, t_settle_s: float, h_agl_m: float | None = None) -> float:
        """e_ext = max(V_g,max t_settle, l_B(h)(1−o_fwd))；h 缺省 → req.h_target_agl_m，再缺省 → h_max（保守取大）。"""
        h = h_agl_m if h_agl_m is not None else (self.r.h_target_agl_m if self.r.h_target_agl_m is not None else self.agl_window()[1])
        if not math.isfinite(h):
            return float(v_g_max_mps) * float(t_settle_s)
        return _end_extension(v_g_max_mps, t_settle_s, self.footprint_len_m(h), self.r.forward_overlap)


def trapezoid_footprint(h_agl_m: float, mount_angle_deg: float, v_fov_deg: float, h_fov_deg: float,
                        gamma_rad: float = 0.0) -> tuple[float, float, float, float, float, float]:
    """論文 Eq.(2)(3) 梯形足跡（規格 §4.3；**除以** sin 的斜距投影形式）。

    回傳 (w, l, w_b, w_f, l_f, d_f)：
        w_b = 2h tan(α_h/2)/sin(α_m − γ + α_v/2)、w_f = 2h tan(α_h/2)/sin(α_m − γ − α_v/2)、
        l_f = h/tan(α_m − γ − α_v/2) − h/tan(α_m − γ + α_v/2)、d_f = 0.5h/tan(α_m − γ − α_v/2) + 0.5h/tan(α_m − γ + α_v/2)、
        w = (w_b + w_f)/2、l = d_f + l_f/2。
    測試值 h=1200、α_m=72.7°、α_v=45°、α_h=76.98° → w ≈ 2200.1、l ≈ 999.8 m。
    α_m + α_v/2 > 90° 時後緣越過天底、tan 為負（允許）；前緣角 α_m − γ − α_v/2 須在 (0°, 180°)。
    """
    am = math.radians(mount_angle_deg) - float(gamma_rad)
    av = math.radians(v_fov_deg)
    ah = math.radians(h_fov_deg)
    a_near = am + av / 2.0    # 後緣（近端，靠天底）
    a_far = am - av / 2.0     # 前緣（遠端）
    if not (0.0 < a_far < math.pi) or not (0.0 < a_near < math.pi) or not (0.0 < ah < math.pi):
        raise ValueError("梯形 FOV 角度須使 α_m−γ±α_v/2 與 α_h 皆在 (0°,180°)")
    h = float(h_agl_m)
    half_w = 2.0 * h * math.tan(ah / 2.0)
    w_b = half_w / math.sin(a_near)
    w_f = half_w / math.sin(a_far)
    l_f = h / math.tan(a_far) - h / math.tan(a_near)
    d_f = 0.5 * h / math.tan(a_far) + 0.5 * h / math.tan(a_near)
    w = 0.5 * (w_b + w_f)
    l = d_f + 0.5 * l_f
    return (w, l, w_b, w_f, l_f, d_f)


@dataclass(frozen=True, slots=True)
class TrapezoidSensor:
    """論文梯形 FOV 感測器模型（規格 §4.3；`fov_model="trapezoid"`／legacy）。

    無像元／測距資訊：gsd_m → None、agl_window → (0, inf)、gsd_attainable → True；
    h_target 預設：req.h_target_agl_m 或 150 m（論文情境由任務輸入指定 1200 m）。
    """
    kind: str
    hw: TrapezoidFovSpec
    req: Any
    r: _ResolvedReq
    crab_rad: float = 0.0          # 建圖模式偏流餘裕 ψ（0 ＝ 舊行為）；l 取 footprint_len_m（保守近似）

    @property
    def defaults_applied(self) -> frozenset[str]:
        return self.r.defaults_applied

    def _dims(self, h_agl_m: float) -> tuple[float, float]:
        w, l, *_ = trapezoid_footprint(h_agl_m, self.hw.mount_angle_deg, self.hw.v_fov_deg, self.hw.h_fov_deg)
        return w, l

    def swath_m(self, h_agl_m: float) -> float:
        return self._dims(h_agl_m)[0]

    def footprint_len_m(self, h_agl_m: float) -> float:
        return self._dims(h_agl_m)[1]

    def strip_spacing_m(self, h_agl_m: float) -> float:
        if self.crab_rad:
            w, l = self._dims(h_agl_m)
            return crabbed_width_m(w, l, self.crab_rad) * (1.0 - self.r.side_overlap)
        return self.swath_m(h_agl_m) * (1.0 - self.r.side_overlap)

    def max_ground_speed_mps(self, h_agl_m: float) -> float:
        """rgb 且有快門率 → l(1−o_fwd) f_s；否則 inf。"""
        if self.kind == "rgb" and self.r.shutter_rate_hz is not None:
            return self.footprint_len_m(h_agl_m) * (1.0 - self.r.forward_overlap) * self.r.shutter_rate_hz
        return math.inf

    def gsd_m(self, h_agl_m: float) -> float | None:
        return None

    def agl_window(self) -> tuple[float, float]:
        return (0.0, math.inf)

    def h_target_m(self, h_min_fw_m: float, dh_band_min_m: float, k_gsd_relax: float) -> float:
        return self.r.h_target_agl_m if self.r.h_target_agl_m is not None else _DEFAULT_H_TARGET_LIDAR_M

    def gsd_attainable(self, h_min_fw_m: float, dh_band_min_m: float) -> bool:
        return True

    def end_extension_m(self, v_g_max_mps: float, t_settle_s: float, h_agl_m: float | None = None) -> float:
        """legacy 情境 e_ext = 0 由 cfg 控制（呼叫端傳 t_settle=0 且不計前向重疊）；此處依 §4.5 通式。"""
        h = h_agl_m if h_agl_m is not None else self.r.h_target_agl_m
        if h is None:
            return float(v_g_max_mps) * float(t_settle_s)
        return _end_extension(v_g_max_mps, t_settle_s, self.footprint_len_m(h), self.r.forward_overlap)


@dataclass(frozen=True, slots=True)
class CombinedSensor:
    """同機同時掛載兩種感測器時之「合併感測器」（規格 §6.2 之 ``both`` 區「取較窄者鋪一套」選項）。

    規格 §6.2 允許 ``both`` 區由「A（LiDAR）+ B（RGB）各鋪一套條帶」或「同一架同時掛載雙感測時，兩套條帶各用
    對應 Sensor 之寬度：分兩套鋪設，**或取較窄者鋪一套**」。本類別實作後者：條帶間距／航速上限取兩者之嚴格值
    （min），高度窗取交集，端延伸取兩者最大，因此一次飛行即同時滿足兩種感測需求（覆蓋率驗證於
    :mod:`hv_dccpp.sim` 依 ``sensors`` 逐感測器各算一次足跡）。

    `kind`／`hw`／`req`／`r` 沿用主感測器（清單第一個），故所有既有「以 kind 判種類」之程式碼行為不變；需要兩種
    種類者請讀 :attr:`kinds` 或 :attr:`sensors`。
    """
    kind: str
    hw: SensorHardware
    req: Any
    r: _ResolvedReq
    sensors: tuple[Any, ...]

    @property
    def kinds(self) -> tuple[str, ...]:
        return tuple(str(getattr(s, "kind", "")) for s in self.sensors)

    @property
    def defaults_applied(self) -> frozenset[str]:
        out: set[str] = set()
        for s in self.sensors:
            out |= set(getattr(s, "defaults_applied", frozenset()))
        return frozenset(out)

    def swath_m(self, h_agl_m: float) -> float:
        return min(float(s.swath_m(h_agl_m)) for s in self.sensors)

    def footprint_len_m(self, h_agl_m: float) -> float:
        return max(float(s.footprint_len_m(h_agl_m)) for s in self.sensors)

    def strip_spacing_m(self, h_agl_m: float) -> float:
        return min(float(s.strip_spacing_m(h_agl_m)) for s in self.sensors)

    def max_ground_speed_mps(self, h_agl_m: float) -> float:
        return min(float(s.max_ground_speed_mps(h_agl_m)) for s in self.sensors)

    def gsd_m(self, h_agl_m: float) -> float | None:
        vals = [s.gsd_m(h_agl_m) for s in self.sensors]
        vals = [float(v) for v in vals if v is not None]
        return max(vals) if vals else None

    def agl_window(self) -> tuple[float, float]:
        wins = [s.agl_window() for s in self.sensors]
        return (max(float(w[0]) for w in wins), min(float(w[1]) for w in wins))

    def h_target_m(self, h_min_fw_m: float, dh_band_min_m: float, k_gsd_relax: float) -> float:
        return min(float(s.h_target_m(h_min_fw_m, dh_band_min_m, k_gsd_relax)) for s in self.sensors)

    def gsd_attainable(self, h_min_fw_m: float, dh_band_min_m: float) -> bool:
        return all(bool(s.gsd_attainable(h_min_fw_m, dh_band_min_m)) for s in self.sensors)

    def end_extension_m(self, v_g_max_mps: float, t_settle_s: float, h_agl_m: float | None = None) -> float:
        return max(float(s.end_extension_m(v_g_max_mps, t_settle_s, h_agl_m)) for s in self.sensors)


def combine_sensors(primary: Any, *others: Any) -> Any:
    """`primary` 與 `others` 之合併感測器（規格 §6.2）。只有一個 → 原樣回傳；種類重複者忽略。"""
    subs: list[Any] = [primary]
    seen = {str(getattr(primary, "kind", ""))}
    for s in others:
        k = str(getattr(s, "kind", ""))
        if s is None or k in seen:
            continue
        seen.add(k)
        subs.append(s)
    if len(subs) == 1:
        return primary
    return CombinedSensor(kind=str(getattr(primary, "kind", "")), hw=primary.hw, req=primary.req,
                          r=primary.r, sensors=tuple(subs))


def sensor_model(hw: SensorHardware, req: Any) -> Sensor:
    """由硬體規格與區域需求建立感測器模型（D43）。

    * `req.kind` 須與 `hw.kind` 一致；`req.shutter_rate_hz` 須 ≤ `hw.max_shutter_rate_hz`，否則 MissionInputError。
    * 需求缺省值（None）依規格 §16.2 補：side_overlap 0.25/0.30、forward_overlap 0.70、shutter → 硬體上限、
      density 20 pt/m²、GSD 0.05 m；採用之預設鍵列於 `.defaults_applied`。
    """
    r = _resolve_req(hw, req)
    if isinstance(hw, LidarHardware):
        return LidarSensor(kind="lidar", hw=hw, req=req, r=r)
    if isinstance(hw, CameraHardware):
        return CameraSensor(kind="rgb", hw=hw, req=req, r=r)
    if isinstance(hw, TrapezoidFovSpec):
        return TrapezoidSensor(kind=hw.kind, hw=hw, req=req, r=r)
    raise _mission_input_error()(f"未知感測器硬體型別 {type(hw).__name__}")


# ---------------------------------------------------------------------------
# 能量模型（規格 §5）
# ---------------------------------------------------------------------------

def _mode_name(mode: Any) -> str:
    name = getattr(mode, "name", mode)
    if not isinstance(name, str):
        raise ValueError(f"mode 須為 Mode 列舉或其名稱字串，得 {mode!r}")
    return name.upper()


@dataclass(slots=True)
class EnergyModel:
    """單一 Wh 能量模型（規格 §5.1–§5.7、D6、D20、D39）；一律以 uav_id 為鍵建立（每機 AUW、E_avail 各異）。

    欄位
    ----
    spec, auw_kg            平台與當架次 AUW m。
    available_wh            E_avail（由 SortieEnergy 解析：Wh → mAh×V/1000 → SoC×nominal）。
    nominal_wh              當架次標稱 E_nom（保留 r·E_nom 以此計）；None 且 reference None → 基準分鐘模式。
    reference_wh            基準測試電池 E_ref（D39）；None → = nominal_wh（呼叫端警告 reference_battery_assumed）。
    battery_temp_c          電池溫度（η_T）。
    correction              κ 線上修正（使用時限幅 0.8–1.5）。
    k_bm, k_as              未驗證保守係數（cfg）；spec 驗證旗標為 True 時強制 1.0。
    base_minute_mode        兩者皆 None → True：所有能量欄位改為「基準巡航分鐘」（1 unit = P_c^ref·60 s，
                            P_c^ref ≡ 1/60 unit/s），E_nom = T_ref。
    temp_cap_coef_per_c, temp_cap_floor   η_T = max(floor, 1 − c_T·max(0, 10 − T_bat))（cfg 值；契約未列，額外欄位）。

    單位：Wh 模式下段能量 = P·t·k_unc/3600；基準分鐘模式下 = P·t·k_unc（P 為 unit/s）。
    基準分鐘模式下無法將 m g Va sinγ/η_p（W）換算為 unit/s，爬升／下降額外功率取 0（已記錄於 CONTRACT_DEVIATIONS）。
    """
    spec: PlatformSpec
    auw_kg: float
    available_wh: float
    nominal_wh: float | None
    reference_wh: float | None
    battery_temp_c: float = 20.0
    correction: float = 1.0
    k_bm: float = 1.15
    k_as: float = 1.05
    base_minute_mode: bool = False
    temp_cap_coef_per_c: float = 0.01
    temp_cap_floor: float = 0.8

    def __post_init__(self) -> None:
        if self.reference_wh is None and self.nominal_wh is not None:
            self.reference_wh = float(self.nominal_wh)          # D39：缺省 = 當架次標稱（呼叫端警告）
        if self.reference_wh is None and self.nominal_wh is None:
            self.base_minute_mode = True                        # 規格 §5.7
        if self.spec.battery_monitor_validated:
            self.k_bm = 1.0                                     # 規格 §5.2
        if self.spec.airspeed_sensor_validated:
            self.k_as = 1.0
        if self.auw_kg <= 0.0:
            raise ValueError("auw_kg 須 > 0")

    # ------------------------------------------------------------------ 係數
    @property
    def k_unc(self) -> float:
        """k_unc = k_bm·k_as（未驗證 1.15×1.05 = 1.2075）。"""
        return self.k_bm * self.k_as

    @property
    def eta_T(self) -> float:
        """溫度容量係數 η_T = max(floor, 1 − c_T·max(0, 10 − T_bat))（規格 §5.2）。"""
        return max(self.temp_cap_floor, 1.0 - self.temp_cap_coef_per_c * max(0.0, 10.0 - self.battery_temp_c))

    @property
    def kappa(self) -> float:
        """限幅後之線上修正 κ ∈ [0.8, 1.5]。"""
        return min(1.5, max(0.8, self.correction))

    def _e_nom(self) -> float:
        """當架次標稱能量（Wh；基準分鐘模式為 T_ref 分鐘）。"""
        if self.base_minute_mode:
            return float(self.spec.reference_endurance_min)
        assert self.nominal_wh is not None
        return float(self.nominal_wh)

    def _wh(self, power: float, duration_s: float) -> float:
        """功率×時間×k_unc → 能量（Wh，或基準分鐘模式之 unit）。"""
        e = power * float(duration_s) * self.k_unc
        return e if self.base_minute_mode else e / 3600.0

    def _time_s(self, energy: float, power: float) -> float:
        """能量／功率 → 秒。"""
        t = energy / power
        return t if self.base_minute_mode else t * 3600.0

    # ------------------------------------------------------------------ 功率（規格 §5.1）
    def p_cruise_ref_w(self) -> float:
        """P_c^ref = f_dep·E_ref/T_ref（與電池組容量無關，D39；A 1500 Wh → 511.4 W、B 420 Wh → 185.3 W）。

        基準分鐘模式 → 1/60 unit/s。
        """
        if self.base_minute_mode:
            return 1.0 / 60.0
        assert self.reference_wh is not None
        return self.spec.reference_test_depth_of_discharge * self.reference_wh / (self.spec.reference_endurance_min / 60.0)

    def p_cruise_w(self) -> float:
        """P_c(m) = κ·P_c^ref·(m/m_ref)^1.5。"""
        return self.kappa * self.p_cruise_ref_w() * (self.auw_kg / self.spec.reference_auw_kg) ** 1.5

    def p_hover_w(self) -> float:
        """P_h = k_h·P_c（k_h = hover_power_ratio = 4.0，D20）。"""
        return self.spec.hover_power_ratio * self.p_cruise_w()

    def p_transition_w(self) -> float:
        """P_tr = P_h + P_c（旋翼輔助，D20/D37）。"""
        return self.p_hover_w() + self.p_cruise_w()

    def p_accel_w(self) -> float:
        """P_acc = p_accel_ratio·P_c（FW_ACCEL/DECEL）。"""
        return self.spec.p_accel_ratio * self.p_cruise_w()

    def _climb_extra_w(self, gamma_rad: float) -> float:
        """m g Va sin|γ|/η_p（W）；基準分鐘模式無法換算 → 0。"""
        if self.base_minute_mode:
            return 0.0
        return self.auw_kg * G_MPS2 * self.spec.cruise_airspeed_mps * math.sin(abs(float(gamma_rad))) / self.spec.prop_efficiency

    def p_climb_w(self, gamma_rad: float) -> float:
        """P↑ = P_c + m g Va sinγ/η_p（A@3°：+453 W）。"""
        return self.p_cruise_w() + self._climb_extra_w(gamma_rad)

    def p_descent_w(self, gamma_rad: float) -> float:
        """P↓ = max(η_idle·P_c, P_c − m g Va sin|γ|/η_p)。"""
        pc = self.p_cruise_w()
        return max(self.spec.idle_descent_ratio * pc, pc - self._climb_extra_w(gamma_rad))

    def power_w(self, mode: Any, gamma_rad: float = 0.0) -> float:
        """依飛行模式取功率：HOVER → P_h；TRANSITION → P_tr；CRUISE → γ>0 P↑、γ<0 P↓、γ=0 P_c。

        `mode` 接受 `mission.Mode` 列舉成員或名稱字串（"HOVER"/"TRANSITION"/"CRUISE"；另容許 "ACCEL" → P_acc）。
        """
        name = _mode_name(mode)
        if name == "HOVER":
            return self.p_hover_w()
        if name == "TRANSITION":
            return self.p_transition_w()
        if name == "CRUISE":
            g = float(gamma_rad)
            if g > 0.0:
                return self.p_climb_w(g)
            if g < 0.0:
                return self.p_descent_w(g)
            return self.p_cruise_w()
        if name == "ACCEL":
            return self.p_accel_w()
        raise ValueError(f"未知 mode {mode!r}")

    # ------------------------------------------------------------------ 段能量（規格 §5.2–§5.3）
    def segment_wh(self, mode: Any, duration_s: float, gamma_rad: float = 0.0) -> float:
        """e_seg = P_mode(m)·t·k_unc（/3600 → Wh）。"""
        return self._wh(self.power_w(mode, gamma_rad), duration_s)

    def transfer_wh(self, t_s: float, dz_m: float, n_laps: int = 0, t_lap_s: float = 0.0) -> float:
        """轉移能量（規格 §8.4）：
        k_unc[P_c t + (P↑−P_c)·max(0,Δz)/(Va sinγ_c) − (P_c−P↓)·max(0,−Δz)/(Va sinγ_d) + P↑ n_lap T_lap]。
        entry_e／rtb_e 同式。γ_c 取 gamma_c_auw(m)；螺旋圈以 P↑ 計。
        """
        pc = self.p_cruise_w()
        gc = self.spec.gamma_c_auw(self.auw_kg)
        gd = self.spec.gamma_d()
        p_up = self.p_climb_w(gc)
        p_dn = self.p_descent_w(gd)
        va = self.spec.cruise_airspeed_mps
        dz = float(dz_m)
        e = pc * float(t_s)
        e += (p_up - pc) * max(0.0, dz) / (va * math.sin(gc))
        e -= (pc - p_dn) * max(0.0, -dz) / (va * math.sin(gd))
        e += p_up * max(0, int(n_laps)) * float(t_lap_s)
        return self._wh(e, 1.0)

    def reserve_wh(self) -> float:
        """保留能量 r·E_nom（當架次標稱；只扣一次，D6）。"""
        return self.spec.reserve_fraction * self._e_nom()

    def e_takeoff_wh(self) -> float:
        """e^TO = k_unc[P_h·H_ts/v_z,up + P_tr·t_tr^out + P_acc·t_acc]（A 39.2、B 12.9 Wh；規格 §5.3）。"""
        s = self.spec
        tc = s.transition_cmd()
        e = self.p_hover_w() * s.transition_start_agl_m / s.vtol_climb_rate_mps
        e += self.p_transition_w() * s.t_tr_out_s
        e += self.p_accel_w() * tc.t_acc_s
        return self._wh(e, 1.0)

    def e_landing_wh(self) -> float:
        """e^LDG = k_unc[P_acc·t_acc + P_tr·t_tr^in + P_h·t_dn + P_h·t_hov,res]，t_dn = hover_descent_time_s
        （10 m 規則；A 81.2、B 26.9 Wh；規格 §5.3）。"""
        s = self.spec
        tc = s.transition_cmd()
        e = self.p_accel_w() * tc.t_acc_s
        e += self.p_transition_w() * s.t_tr_in_s
        e += self.p_hover_w() * s.hover_descent_time_s()
        e += self.p_hover_w() * s.t_hover_reserve_s
        return self._wh(e, 1.0)

    def e_usable_wh(self) -> float:
        """可規劃能量 E^use = η_T·E_avail − r·E_nom − e^TO − e^LDG（A 1350 Wh → 779.6；> 0 才可規劃）。"""
        return self.eta_T * float(self.available_wh) - self.reserve_wh() - self.e_takeoff_wh() - self.e_landing_wh()

    def cruise_equiv_km(self, wh: float) -> float:
        """巡航等效距離 = wh/(P_c·k_unc) [h]·Va·3.6（km；規格 §5.4；389.6 Wh → 48.8 km）。"""
        return self._time_s(float(wh), self.p_cruise_w() * self.k_unc) * self.spec.cruise_airspeed_mps / 1000.0

    def usable_distance_cap_km(self) -> float:
        """表 1 usable_distance_km 獨立上限 D^use：battery_monitor 未驗證取下緣（133/79）、驗證取上緣（145/86）。"""
        lo, hi = self.spec.usable_distance_km
        return float(hi) if self.spec.battery_monitor_validated else float(lo)


# 模組載入時之自檢：表 1 鍵全部存在於 PlatformSpec、provisional 鍵皆為欄位名
_FIELD_NAMES = {f.name for f in fields(PlatformSpec)}
assert set(TABLE1_KEYS) <= _FIELD_NAMES, "TABLE1_KEYS 與 PlatformSpec 欄位不一致"
assert PROVISIONAL_SUPPLEMENTARY_DEFAULT <= _FIELD_NAMES, "PROVISIONAL_SUPPLEMENTARY_DEFAULT 含非欄位鍵"
assert PLATFORM_PIONEER.provisional <= _FIELD_NAMES and PLATFORM_HERO.provisional <= _FIELD_NAMES
