"""mission.py — 表 2 MissionInputs、規劃輸出型別、PlannerConfig、`load_mission`（模組契約 §3）。

第 1 層模組：只依賴 `platform`、`geo`（第 0 層）；**不含** preflight（位於 `preflight.py`）。
對應規格：§6.1（表 2 → 內部表示）、§16.2（PlannerConfig 全部參數與預設）、§5.7（可用電量解析順序）、
§15（`legacy_mode` 退化條件）、D7（氣象門檻）、D11（c2 / terrain_clearance 必填，缺則 NO_GO — 由 preflight 判定）、
D22（三種時間窗）、D25（重疊率預設）、D26（失鏈策略）、D40（落點政策：recovery_site_id None → home）。

全域約定（契約 §0）
------------------
* 輸入型別 `frozen=True, slots=True`；輸出／狀態型別 `slots=True`。
* 輸入座標一律 WGS84（deg）`(lat, lon)`；高度以 MSL 或 AGL（m）給定，ENU 轉換在 `context.build_context`（D42）。
  只有 GeoJSON 幾何物件使用 `(lon, lat)`，`load_mission` 會自動交換。
* 單位：長度 m、時間 s、速度 m/s、能量 Wh；欄位以 `_deg` 結尾者為度。
* 輸入錯誤一律丟 :class:`MissionInputError`（與 `platform.MissionInputError` 為同一類別）。

JSON 介面
---------
`load_mission(dict | JSON 字串 | 檔案路徑)` → :class:`MissionInputs`；`MissionInputs.to_dict()/to_json()` 之輸出可原樣
回饋 `load_mission`（往返一致）。`Waypoint`／`UAVPlan`／`FeasibilityReport` 提供 `to_dict`/`from_dict`；
`MissionPlan.to_dict()` 為單向輸出（cells/strips 以通用序列化輸出，不支援反解）。
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field, fields, is_dataclass, replace
from enum import Enum, auto
from pathlib import Path
from typing import Any, Literal, Mapping, Sequence

import numpy as np

from .geo import DemGrid
from .platform import (
    PLATFORM_HERO,
    PLATFORM_PIONEER,
    CameraHardware,
    LidarHardware,
    MissionInputError,
    PlatformSpec,
    SensorHardware,
    TrapezoidFovSpec,
)

__all__ = [
    "MissionInputError",
    "CLASSES",
    "DEFAULT_LOST_LINK_POLICY",
    "K_INF",
    "Site",
    "AreaSpec",
    "Obstacle",
    "WindLayer",
    "SortieEnergy",
    "PayloadRequirement",
    "C2Link",
    "NoFlyZone",
    "Airspace",
    "WeatherThresholds",
    "WeatherObs",
    "UAVInstance",
    "MissionInputs",
    "PlannerConfig",
    "Mode",
    "Phase",
    "FW_PHASES",
    "VTOL_PHASES",
    "TRANSITION_PHASES",
    "mode_phase_violation",
    "UAVState",
    "Waypoint",
    "UAVPlan",
    "FeasibilityReport",
    "MissionPlan",
    "load_mission",
    "resolve_weather_go",
    "allowed_override_keys",
    "to_jsonable",
]

CLASSES: tuple[str, ...] = ("A", "B")
"""構型代號：A = Pioneer PNP、B = HERO PNP。"""

DEFAULT_LOST_LINK_POLICY: dict[str, str] = {"A": "CONTINUE_THEN_RTB", "B": "RTB"}
"""失鏈策略缺省（規格 §12.5、D26）：A 續飛至 t_ll 後 RTB；B 立即 RTB。"""

K_INF: int = 2**31 - 1
"""`legacy_mode` 之 `K = ∞`（k-nearest 候選數不設限）的整數表示（`PlannerConfig.K` 型別為 int）。"""

_SENSOR_REQS = ("lidar", "rgb", "both", "any")
_OBSTACLE_GEOMS = ("point", "line", "polygon")
_OBSTACLE_CATEGORIES = ("tower", "building", "tree", "powerline", "other")
_LOST_LINK_POLICIES = ("RTB", "CONTINUE_THEN_RTB", "NEAREST_ALT")
_SITE_KINDS = ("home", "alternate")
_WIND_REFS = ("msl", "agl")
_PAYLOAD_KINDS = ("lidar", "rgb")
_SPEC_BY_NAME: dict[str, PlatformSpec] = {
    "A": PLATFORM_PIONEER, "PIONEER": PLATFORM_PIONEER, "PIONEERPNP": PLATFORM_PIONEER,
    "B": PLATFORM_HERO, "HERO": PLATFORM_HERO, "HEROPNP": PLATFORM_HERO,
}
_SPEC_BY_CLS: dict[str, PlatformSpec] = {"A": PLATFORM_PIONEER, "B": PLATFORM_HERO}
_HW_TYPES: dict[str, type] = {
    "LidarHardware": LidarHardware, "CameraHardware": CameraHardware, "TrapezoidFovSpec": TrapezoidFovSpec,
}


# ---------------------------------------------------------------------------
# 小工具：數值／座標驗證與正規化
# ---------------------------------------------------------------------------

def _is_num(v: Any) -> bool:
    return isinstance(v, (int, float, np.integer, np.floating)) and not isinstance(v, (bool, np.bool_))


def _num(v: Any, name: str, *, allow_inf: bool = False) -> float:
    """要求有限實數（或 allow_inf 時允許 ±inf），回傳 float。"""
    if not _is_num(v):
        raise MissionInputError(f"{name} 須為實數，得 {v!r}")
    x = float(v)
    if math.isnan(x) or (not allow_inf and math.isinf(x)):
        raise MissionInputError(f"{name} 須為有限實數，得 {v!r}")
    return x


def _opt_num(v: Any, name: str, *, allow_inf: bool = False) -> float | None:
    return None if v is None else _num(v, name, allow_inf=allow_inf)


def _nonneg(v: float, name: str) -> float:
    if v < 0.0:
        raise MissionInputError(f"{name} 須 ≥ 0，得 {v}")
    return v


def _positive(v: float, name: str) -> float:
    if v <= 0.0:
        raise MissionInputError(f"{name} 須 > 0，得 {v}")
    return v


def _str_id(v: Any, name: str) -> str:
    if not isinstance(v, str) or not v.strip():
        raise MissionInputError(f"{name} 須為非空字串，得 {v!r}")
    return v


def _latlon_pair(p: Any, name: str) -> tuple[float, float]:
    """`(lat_deg, lon_deg)`（允許第三個元素被忽略）→ tuple[float, float]，並檢查範圍。"""
    if isinstance(p, Mapping):
        if "lat_deg" in p and "lon_deg" in p:
            p = (p["lat_deg"], p["lon_deg"])
        elif "lat" in p and "lon" in p:
            p = (p["lat"], p["lon"])
        else:
            raise MissionInputError(f"{name}: 座標 mapping 須含 lat_deg/lon_deg，得 {p!r}")
    if isinstance(p, (str, bytes)) or not isinstance(p, Sequence) and not isinstance(p, np.ndarray):
        raise MissionInputError(f"{name}: 座標須為 (lat_deg, lon_deg) 序列，得 {p!r}")
    seq = list(p)
    if len(seq) < 2:
        raise MissionInputError(f"{name}: 座標須至少含 (lat_deg, lon_deg)，得 {p!r}")
    lat = _num(seq[0], f"{name}.lat_deg")
    lon = _num(seq[1], f"{name}.lon_deg")
    if not (-90.0 <= lat <= 90.0):
        raise MissionInputError(f"{name}: lat_deg={lat} 超出 [-90, 90]")
    if not (-180.0 <= lon <= 180.0):
        raise MissionInputError(f"{name}: lon_deg={lon} 超出 [-180, 180]")
    return (lat, lon)


def _ring(pts: Any, name: str, min_n: int = 3) -> tuple[tuple[float, float], ...]:
    """頂點序列 → tuple[(lat, lon), …]；去除重複的閉合頂點（GeoJSON 慣例）；檢查頂點數 ≥ min_n。"""
    if pts is None or isinstance(pts, (str, bytes, Mapping)):
        raise MissionInputError(f"{name} 須為 (lat, lon) 頂點序列，得 {pts!r}")
    out = [_latlon_pair(p, f"{name}[{i}]") for i, p in enumerate(list(pts))]
    if len(out) >= 2 and out[0] == out[-1]:
        out = out[:-1]
    if len(out) < min_n:
        raise MissionInputError(f"{name} 須至少 {min_n} 個相異頂點，得 {len(out)}")
    return tuple(out)


def _window(w: Any, name: str) -> tuple[float, float] | None:
    if w is None:
        return None
    if isinstance(w, (str, bytes)) or not isinstance(w, Sequence) or len(w) != 2:
        raise MissionInputError(f"{name} 須為 (lo, hi) 二元組，得 {w!r}")
    lo = _num(w[0], f"{name}[0]", allow_inf=True)
    hi = _num(w[1], f"{name}[1]", allow_inf=True)
    if lo > hi:
        raise MissionInputError(f"{name}: lo={lo} > hi={hi}")
    return (lo, hi)


def _pair(w: Any, name: str) -> tuple[float, float]:
    r = _window(w, name)
    assert r is not None
    return r


def _set(v: Any, allowed: Sequence[str], name: str) -> str:
    if v not in allowed:
        raise MissionInputError(f"{name} 須為 {list(allowed)} 之一，得 {v!r}")
    return v


# ---------------------------------------------------------------------------
# 表 2 輸入型別（frozen + slots）
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class Site:
    """起降點／備降點（表 2「起飛點與備降點」；規格 §6.1）。

    `alt_msl_m` 與 `alt_agl_m` 二擇一（AGL 以 DEM 換算；皆 None → 由 DEM 取地面高）。
    `clear_radius_m` None → `spec.vtol_site_clear_radius_m`；`approach_heading_deg` None → 逆風進場（D19）。
    """
    site_id: str
    lat_deg: float
    lon_deg: float
    alt_msl_m: float | None = None
    alt_agl_m: float | None = None
    kind: Literal["home", "alternate"] = "alternate"
    clear_radius_m: float | None = None
    approach_heading_deg: float | None = None

    def __post_init__(self) -> None:
        _str_id(self.site_id, "Site.site_id")
        lat, lon = _latlon_pair((self.lat_deg, self.lon_deg), f"Site[{self.site_id}]")
        object.__setattr__(self, "lat_deg", lat)
        object.__setattr__(self, "lon_deg", lon)
        msl = _opt_num(self.alt_msl_m, f"Site[{self.site_id}].alt_msl_m")
        agl = _opt_num(self.alt_agl_m, f"Site[{self.site_id}].alt_agl_m")
        if msl is not None and agl is not None:
            raise MissionInputError(f"Site[{self.site_id}]: alt_msl_m 與 alt_agl_m 二擇一，不可同時給定")
        object.__setattr__(self, "alt_msl_m", msl)
        object.__setattr__(self, "alt_agl_m", agl)
        _set(self.kind, _SITE_KINDS, f"Site[{self.site_id}].kind")
        cr = _opt_num(self.clear_radius_m, f"Site[{self.site_id}].clear_radius_m")
        if cr is not None:
            _positive(cr, f"Site[{self.site_id}].clear_radius_m")
        object.__setattr__(self, "clear_radius_m", cr)
        object.__setattr__(self, "approach_heading_deg",
                           _opt_num(self.approach_heading_deg, f"Site[{self.site_id}].approach_heading_deg"))

    @property
    def latlon(self) -> tuple[float, float]:
        return (self.lat_deg, self.lon_deg)


@dataclass(frozen=True, slots=True)
class AreaSpec:
    """任務區域（表 2「任務區域」；規格 §1.1、§6.1）。`polygon_wgs84` 為 `(lat, lon)` 外環、`holes_wgs84` 內環。"""
    area_id: str
    polygon_wgs84: tuple[tuple[float, float], ...]
    holes_wgs84: tuple[tuple[tuple[float, float], ...], ...] = ()
    sensor_req: Literal["lidar", "rgb", "both", "any"] = "any"
    priority: float = 1.0                          # 論文 val_k
    time_window_s: tuple[float, float] | None = None
    payload_req_id: str = "default"

    def __post_init__(self) -> None:
        _str_id(self.area_id, "AreaSpec.area_id")
        n = f"AreaSpec[{self.area_id}]"
        object.__setattr__(self, "polygon_wgs84", _ring(self.polygon_wgs84, f"{n}.polygon_wgs84"))
        holes = self.holes_wgs84 or ()
        if isinstance(holes, (str, bytes, Mapping)):
            raise MissionInputError(f"{n}.holes_wgs84 須為內環序列")
        object.__setattr__(self, "holes_wgs84",
                           tuple(_ring(h, f"{n}.holes_wgs84[{i}]") for i, h in enumerate(list(holes))))
        _set(self.sensor_req, _SENSOR_REQS, f"{n}.sensor_req")
        object.__setattr__(self, "priority", _nonneg(_num(self.priority, f"{n}.priority"), f"{n}.priority"))
        object.__setattr__(self, "time_window_s", _window(self.time_window_s, f"{n}.time_window_s"))
        _str_id(self.payload_req_id, f"{n}.payload_req_id")


@dataclass(frozen=True, slots=True)
class Obstacle:
    """障礙物（表 2「障礙物資料」）。`top_msl_m` 與 `height_agl_m` 二擇一；頂高於 `KeepoutSet.build` 轉 ENU z（D42）。

    `keepout_buffer_m` None → `cfg.b_h_m`（航路 keep-out 緩衝，D13；非表 1 之 `obstacle_horizontal_buffer_m`）。
    """
    obs_id: str
    geom_type: Literal["point", "line", "polygon"]
    coords_wgs84: tuple[tuple[float, float], ...]
    top_msl_m: float | None
    height_agl_m: float | None
    category: Literal["tower", "building", "tree", "powerline", "other"] = "other"
    keepout_buffer_m: float | None = None

    def __post_init__(self) -> None:
        _str_id(self.obs_id, "Obstacle.obs_id")
        n = f"Obstacle[{self.obs_id}]"
        _set(self.geom_type, _OBSTACLE_GEOMS, f"{n}.geom_type")
        min_n = {"point": 1, "line": 2, "polygon": 3}[self.geom_type]
        if self.geom_type == "polygon":
            coords = _ring(self.coords_wgs84, f"{n}.coords_wgs84", 3)
        else:
            if self.coords_wgs84 is None or isinstance(self.coords_wgs84, (str, bytes, Mapping)):
                raise MissionInputError(f"{n}.coords_wgs84 須為 (lat, lon) 序列")
            coords = tuple(_latlon_pair(p, f"{n}.coords_wgs84[{i}]") for i, p in enumerate(list(self.coords_wgs84)))
            if len(coords) < min_n:
                raise MissionInputError(f"{n}: geom_type={self.geom_type} 須至少 {min_n} 個頂點，得 {len(coords)}")
        object.__setattr__(self, "coords_wgs84", coords)
        top = _opt_num(self.top_msl_m, f"{n}.top_msl_m")
        hgt = _opt_num(self.height_agl_m, f"{n}.height_agl_m")
        if (top is None) == (hgt is None):
            raise MissionInputError(f"{n}: top_msl_m 與 height_agl_m 須恰給定其一")
        if hgt is not None:
            _nonneg(hgt, f"{n}.height_agl_m")
        object.__setattr__(self, "top_msl_m", top)
        object.__setattr__(self, "height_agl_m", hgt)
        _set(self.category, _OBSTACLE_CATEGORIES, f"{n}.category")
        kb = _opt_num(self.keepout_buffer_m, f"{n}.keepout_buffer_m")
        if kb is not None:
            _nonneg(kb, f"{n}.keepout_buffer_m")
        object.__setattr__(self, "keepout_buffer_m", kb)


@dataclass(frozen=True, slots=True)
class WindLayer:
    """分層風（表 2「風場」）。`dir_from_deg` 為氣象「來向」；吹向向量 w = −W(sin β, cos β)（契約 §0）。

    `z_ref="agl"` 時層界以 home 地面高換算為 ENU z（`WindField` 建構時；記錄假設）。
    """
    z_bottom_m: float
    z_top_m: float
    speed_mps: float
    dir_from_deg: float
    gust_mps: float = 0.0
    z_ref: Literal["msl", "agl"] = "msl"

    def __post_init__(self) -> None:
        zb = _num(self.z_bottom_m, "WindLayer.z_bottom_m", allow_inf=True)
        zt = _num(self.z_top_m, "WindLayer.z_top_m", allow_inf=True)
        if zt <= zb:
            raise MissionInputError(f"WindLayer: z_top_m={zt} 須 > z_bottom_m={zb}")
        object.__setattr__(self, "z_bottom_m", zb)
        object.__setattr__(self, "z_top_m", zt)
        object.__setattr__(self, "speed_mps", _nonneg(_num(self.speed_mps, "WindLayer.speed_mps"), "WindLayer.speed_mps"))
        object.__setattr__(self, "dir_from_deg", _num(self.dir_from_deg, "WindLayer.dir_from_deg"))
        object.__setattr__(self, "gust_mps", _nonneg(_num(self.gust_mps, "WindLayer.gust_mps"), "WindLayer.gust_mps"))
        _set(self.z_ref, _WIND_REFS, "WindLayer.z_ref")

    @property
    def dir_from_rad(self) -> float:
        return math.radians(self.dir_from_deg)

    def wind_vec(self) -> np.ndarray:
        """吹向向量 (wx, wy) = −W(sin β, cos β)（契約 §0）。"""
        b = self.dir_from_rad
        return np.array([-self.speed_mps * math.sin(b), -self.speed_mps * math.cos(b)], dtype=np.float64)


@dataclass(frozen=True, slots=True)
class SortieEnergy:
    """當架次 AUW 與可用電量（表 2「當架次 AUW」「可用電量」；規格 §5.7）。

    解析順序（`available_wh_resolved`）：`available_wh` → `available_mah × pack_voltage_nom_v / 1000` →
    `soc_fraction × nominal`。給了 Wh／mAh 但 nominal（`pack_nominal_wh` → `spec.battery_nominal_wh`）皆 None →
    `MissionInputError`（訊息列鍵名；preflight 轉 NO_GO）；僅 `soc_fraction` 且 nominal None → 回 None（基準分鐘模式）。
    `payload_kg` None → `auw − spec.empty_mass_kg()` 推估並警告 `payload_estimated`（preflight）。
    `battery_temp_c` None → `WeatherObs.temp_c` 並警告 `battery_temp_assumed`（見 :meth:`battery_temp_resolved`）。
    """
    uav_id: str
    auw_kg: float
    payload_kg: float | None = None
    available_wh: float | None = None
    available_mah: float | None = None
    pack_voltage_nom_v: float | None = None
    soc_fraction: float | None = None
    pack_nominal_wh: float | None = None
    battery_temp_c: float | None = None

    def __post_init__(self) -> None:
        _str_id(self.uav_id, "SortieEnergy.uav_id")
        n = f"SortieEnergy[{self.uav_id}]"
        object.__setattr__(self, "auw_kg", _positive(_num(self.auw_kg, f"{n}.auw_kg"), f"{n}.auw_kg"))
        for key in ("payload_kg", "available_wh", "available_mah", "pack_voltage_nom_v", "pack_nominal_wh"):
            v = _opt_num(getattr(self, key), f"{n}.{key}")
            if v is not None:
                if key == "payload_kg":
                    _nonneg(v, f"{n}.{key}")
                else:
                    _positive(v, f"{n}.{key}")
            object.__setattr__(self, key, v)
        soc = _opt_num(self.soc_fraction, f"{n}.soc_fraction")
        if soc is not None and not (0.0 < soc <= 1.0):
            raise MissionInputError(f"{n}.soc_fraction 須在 (0, 1]，得 {soc}")
        object.__setattr__(self, "soc_fraction", soc)
        object.__setattr__(self, "battery_temp_c", _opt_num(self.battery_temp_c, f"{n}.battery_temp_c"))

    # -- 解析 -----------------------------------------------------------------
    def nominal_wh_resolved(self, spec: PlatformSpec) -> float | None:
        """E_nom：`pack_nominal_wh`（當架次實際電池組）→ `spec.battery_nominal_wh`（示例值，preflight 警告）→ None。"""
        if self.pack_nominal_wh is not None:
            return float(self.pack_nominal_wh)
        if spec.battery_nominal_wh is not None:
            return float(spec.battery_nominal_wh)
        return None

    def available_wh_resolved(self, nominal_wh: float | None) -> float | None:
        """E_avail（Wh）；規格 §5.7 解析順序。

        * `available_wh` 給定 → 原值（nominal None → MissionInputError，鍵名 pack_nominal_wh/battery_nominal_wh）。
        * `available_mah` 給定 → mAh × V / 1000（缺 `pack_voltage_nom_v` 或 nominal None → MissionInputError）。
        * 僅 `soc_fraction` → soc × nominal；nominal None → None（基準分鐘模式：E_avail = soc × T_ref 由 EnergyModel 建構者處理）。
        * 三者皆缺 → MissionInputError。
        """
        n = f"SortieEnergy[{self.uav_id}]"
        if self.available_wh is not None:
            if nominal_wh is None:
                raise MissionInputError(
                    f"{n}: 給定 available_wh 但標稱容量未知（pack_nominal_wh 與 spec.battery_nominal_wh 皆 None）；"
                    f"缺鍵：pack_nominal_wh/battery_nominal_wh")
            return float(self.available_wh)
        if self.available_mah is not None:
            if self.pack_voltage_nom_v is None:
                raise MissionInputError(f"{n}: 給定 available_mah 但缺 pack_voltage_nom_v；缺鍵：pack_voltage_nom_v")
            if nominal_wh is None:
                raise MissionInputError(
                    f"{n}: 給定 available_mah 但標稱容量未知（pack_nominal_wh 與 spec.battery_nominal_wh 皆 None）；"
                    f"缺鍵：pack_nominal_wh/battery_nominal_wh")
            return float(self.available_mah) * float(self.pack_voltage_nom_v) / 1000.0
        if self.soc_fraction is not None:
            if nominal_wh is None:
                return None
            return float(self.soc_fraction) * float(nominal_wh)
        raise MissionInputError(f"{n}: 未給可用電量；缺鍵：available_wh/available_mah/soc_fraction")

    def is_base_minute_candidate(self, spec: PlatformSpec) -> bool:
        """僅給 soc_fraction 且 nominal 兩層皆 None → 基準分鐘模式（規格 §5.7）。"""
        return (self.available_wh is None and self.available_mah is None and self.soc_fraction is not None
                and self.nominal_wh_resolved(spec) is None)

    def payload_kg_resolved(self, spec: PlatformSpec) -> tuple[float, bool]:
        """`(payload_kg, estimated)`；None → auw − spec.empty_mass_kg()（下限 0），estimated=True → 警告 payload_estimated。"""
        if self.payload_kg is not None:
            return float(self.payload_kg), False
        return max(0.0, float(self.auw_kg) - spec.empty_mass_kg()), True

    def battery_temp_resolved(self, obs: "WeatherObs | None") -> tuple[float, bool]:
        """`(battery_temp_c, assumed)`；None → `obs.temp_c`（assumed=True → 警告 battery_temp_assumed）；obs 亦 None → 20 °C。"""
        if self.battery_temp_c is not None:
            return float(self.battery_temp_c), False
        if obs is not None:
            return float(obs.temp_c), True
        return 20.0, True


@dataclass(frozen=True, slots=True)
class PayloadRequirement:
    """區域級酬載任務需求（D43）；`AreaSpec.payload_req_id` 指向。缺省鍵由 `platform.sensor_model` 依 §16.2 補：
    side_overlap 0.25（lidar）/0.30（rgb）、forward_overlap 0.70（D25）、shutter → 硬體上限、
    density 20 pt/m²、GSD 0.05 m、h_target → `Sensor.h_target_m`（A 150；B ≈171）。
    """
    req_id: str
    kind: Literal["lidar", "rgb"]
    gsd_required_m: float | None = None
    density_pt_m2: float | None = None
    side_overlap: float | None = None
    forward_overlap: float = 0.70
    shutter_rate_hz: float | None = None
    h_target_agl_m: float | None = None

    def __post_init__(self) -> None:
        _str_id(self.req_id, "PayloadRequirement.req_id")
        n = f"PayloadRequirement[{self.req_id}]"
        _set(self.kind, _PAYLOAD_KINDS, f"{n}.kind")
        for key in ("gsd_required_m", "density_pt_m2", "shutter_rate_hz", "h_target_agl_m"):
            v = _opt_num(getattr(self, key), f"{n}.{key}")
            if v is not None:
                _positive(v, f"{n}.{key}")
            object.__setattr__(self, key, v)
        so = _opt_num(self.side_overlap, f"{n}.side_overlap")
        if so is not None and not (0.0 <= so < 1.0):
            raise MissionInputError(f"{n}.side_overlap 須在 [0, 1)，得 {so}")
        object.__setattr__(self, "side_overlap", so)
        fo = _num(self.forward_overlap, f"{n}.forward_overlap")
        if not (0.0 <= fo < 1.0):
            raise MissionInputError(f"{n}.forward_overlap 須在 [0, 1)，得 {fo}")
        object.__setattr__(self, "forward_overlap", fo)


@dataclass(frozen=True, slots=True)
class C2Link:
    """C2 資料鏈路覆蓋（表 2；規格 §6.1、D11、D26）。

    `range_km`：單值套用兩構型，或 `{"A": …, "B": …}` 逐構型；None → `spec.c2_link_range_km` 覆寫槽 → None（preflight NO_GO）。
    `coverage_polygon_wgs84`：`(lat, lon)` 多邊形或 `{cls: polygon}`。
    `lost_link_policy`：per `uav_id`；缺省 A: CONTINUE_THEN_RTB、B: RTB（:meth:`policy_for`）。
    """
    range_km: float | dict[str, float] | None
    coverage_polygon_wgs84: tuple[tuple[float, float], ...] | dict[str, tuple[tuple[float, float], ...]] | None
    ground_station: Site
    lost_link_policy: dict[str, Literal["RTB", "CONTINUE_THEN_RTB", "NEAREST_ALT"]]
    t_ll_s: float = 60.0

    def __post_init__(self) -> None:
        r = self.range_km
        if r is None:
            pass
        elif isinstance(r, Mapping):
            rr: dict[str, float] = {}
            for k, v in r.items():
                _set(k, CLASSES, "C2Link.range_km key")
                if v is not None:
                    rr[str(k)] = _positive(_num(v, f"C2Link.range_km[{k}]"), f"C2Link.range_km[{k}]")
            r = rr
        else:
            r = _positive(_num(r, "C2Link.range_km"), "C2Link.range_km")
        object.__setattr__(self, "range_km", r)
        cov = self.coverage_polygon_wgs84
        if cov is None:
            pass
        elif isinstance(cov, Mapping):
            cc: dict[str, tuple[tuple[float, float], ...]] = {}
            for k, v in cov.items():
                _set(k, CLASSES, "C2Link.coverage_polygon_wgs84 key")
                if v is not None:
                    cc[str(k)] = _ring(v, f"C2Link.coverage_polygon_wgs84[{k}]")
            cov = cc
        else:
            cov = _ring(cov, "C2Link.coverage_polygon_wgs84")
        object.__setattr__(self, "coverage_polygon_wgs84", cov)
        if not isinstance(self.ground_station, Site):
            raise MissionInputError("C2Link.ground_station 須為 Site")
        pol = self.lost_link_policy
        if pol is None:
            pol = {}
        if not isinstance(pol, Mapping):
            raise MissionInputError("C2Link.lost_link_policy 須為 {uav_id: policy} 字典")
        for k, v in pol.items():
            _set(v, _LOST_LINK_POLICIES, f"C2Link.lost_link_policy[{k}]")
        object.__setattr__(self, "lost_link_policy", dict(pol))
        object.__setattr__(self, "t_ll_s", _nonneg(_num(self.t_ll_s, "C2Link.t_ll_s"), "C2Link.t_ll_s"))

    def range_km_for(self, cls: str, spec: PlatformSpec) -> float | None:
        """P_link,c 半徑：mission 值（逐構型或單值）→ `spec.c2_link_range_km` 覆寫槽 → None。"""
        r = self.range_km
        if isinstance(r, Mapping):
            v = r.get(cls)
            if v is not None:
                return float(v)
        elif r is not None:
            return float(r)
        if spec.c2_link_range_km is not None:
            return float(spec.c2_link_range_km)
        return None

    def coverage_for(self, cls: str) -> tuple[tuple[float, float], ...] | None:
        """該構型之 coverage polygon（單值套用兩構型）；None = 未給。"""
        cov = self.coverage_polygon_wgs84
        if isinstance(cov, Mapping):
            return cov.get(cls)
        return cov

    def policy_for(self, uav_id: str, cls: str) -> str:
        """失鏈策略：mission 值 → 構型缺省（A CONTINUE_THEN_RTB、B RTB；D26）。"""
        p = self.lost_link_policy.get(uav_id)
        if p is not None:
            return p
        return DEFAULT_LOST_LINK_POLICY.get(cls, "RTB")


@dataclass(frozen=True, slots=True)
class NoFlyZone:
    """禁飛區（表 2「空域與法規」）。樓層／天花板為 MSL（解析時轉 ENU z，D42）；`active_window_s` None → 永遠有效；
    與任務窗重疊即視為有效（D22）。"""
    zone_id: str
    polygon_wgs84: tuple[tuple[float, float], ...]
    z_floor_msl_m: float | None = None
    z_ceiling_msl_m: float | None = None
    active_window_s: tuple[float, float] | None = None

    def __post_init__(self) -> None:
        _str_id(self.zone_id, "NoFlyZone.zone_id")
        n = f"NoFlyZone[{self.zone_id}]"
        object.__setattr__(self, "polygon_wgs84", _ring(self.polygon_wgs84, f"{n}.polygon_wgs84"))
        fl = _opt_num(self.z_floor_msl_m, f"{n}.z_floor_msl_m")
        ce = _opt_num(self.z_ceiling_msl_m, f"{n}.z_ceiling_msl_m")
        if fl is not None and ce is not None and ce <= fl:
            raise MissionInputError(f"{n}: z_ceiling_msl_m={ce} 須 > z_floor_msl_m={fl}")
        object.__setattr__(self, "z_floor_msl_m", fl)
        object.__setattr__(self, "z_ceiling_msl_m", ce)
        object.__setattr__(self, "active_window_s", _window(self.active_window_s, f"{n}.active_window_s"))

    def is_active(self, t_window: tuple[float, float] | None) -> bool:
        """D22：`active_window_s` None → 永遠有效；否則與任務窗 `t_window` 重疊即有效（t_window None → 視為有效）。"""
        if self.active_window_s is None or t_window is None:
            return True
        a0, a1 = self.active_window_s
        b0, b1 = t_window
        return a0 <= b1 and b0 <= a1


@dataclass(frozen=True, slots=True)
class Airspace:
    """空域與法規（表 2）。`max_agl_m` 與 `max_msl_m` 皆 None → preflight 採 120 m AGL 並警告 `airspace_ceiling_defaulted_120`；
    `waiver_max_agl_m` 為豁免後上限（D12）；`terrain_clearance_m` 必填（Airspace → spec 覆寫槽 → NO_GO，D11）。"""
    max_agl_m: float | None = None
    waiver_max_agl_m: float | None = None
    max_msl_m: float | None = None
    no_fly: tuple[NoFlyZone, ...] = ()
    time_window_s: tuple[float, float] | None = None
    terrain_clearance_m: float | None = None

    def __post_init__(self) -> None:
        for key in ("max_agl_m", "waiver_max_agl_m", "max_msl_m", "terrain_clearance_m"):
            v = _opt_num(getattr(self, key), f"Airspace.{key}")
            if v is not None:
                if key in ("max_agl_m", "waiver_max_agl_m"):
                    _positive(v, f"Airspace.{key}")
                elif key == "terrain_clearance_m":
                    _nonneg(v, f"Airspace.{key}")
            object.__setattr__(self, key, v)
        nf = self.no_fly or ()
        if isinstance(nf, NoFlyZone):
            nf = (nf,)
        nf = tuple(nf)
        for z in nf:
            if not isinstance(z, NoFlyZone):
                raise MissionInputError("Airspace.no_fly 元素須為 NoFlyZone")
        ids = [z.zone_id for z in nf]
        if len(set(ids)) != len(ids):
            raise MissionInputError(f"Airspace.no_fly zone_id 重複：{ids}")
        object.__setattr__(self, "no_fly", nf)
        object.__setattr__(self, "time_window_s", _window(self.time_window_s, "Airspace.time_window_s"))

    @property
    def ceiling_defaulted(self) -> bool:
        """`max_agl_m` 與 `max_msl_m` 皆 None（preflight → 120 m AGL 並警告）。"""
        return self.max_agl_m is None and self.max_msl_m is None

    def active_no_fly(self, t_window: tuple[float, float] | None) -> tuple[NoFlyZone, ...]:
        """在任務窗內有效之禁區（D22）。"""
        return tuple(z for z in self.no_fly if z.is_active(t_window))


@dataclass(frozen=True, slots=True)
class WeatherThresholds:
    """作業氣象門檻（表 2；規格 §6.2 步 2、§12.9、D7）。`go_mean_wind_mps` None → min(8.0, 0.45·Va) 逐構型（`resolve_weather_go`）。"""
    go_mean_wind_mps: float | None = None
    go_gust_mps: float = 12.0
    go_visibility_m: float = 3000.0
    go_precip_mmh: float = 0.5
    go_temp_range_c: tuple[float, float] = (-10.0, 40.0)
    abort_mean_wind_mps: float = 10.0
    abort_gust_mps: float = 14.0
    abort_visibility_m: float = 1500.0
    abort_precip_mmh: float = 2.0
    abort_temp_range_c: tuple[float, float] = (-15.0, 45.0)

    def __post_init__(self) -> None:
        object.__setattr__(self, "go_mean_wind_mps", _opt_num(self.go_mean_wind_mps, "WeatherThresholds.go_mean_wind_mps"))
        for key in ("go_gust_mps", "go_visibility_m", "go_precip_mmh", "abort_mean_wind_mps", "abort_gust_mps",
                    "abort_visibility_m", "abort_precip_mmh"):
            object.__setattr__(self, key, _nonneg(_num(getattr(self, key), f"WeatherThresholds.{key}"), f"WeatherThresholds.{key}"))
        object.__setattr__(self, "go_temp_range_c", _pair(self.go_temp_range_c, "WeatherThresholds.go_temp_range_c"))
        object.__setattr__(self, "abort_temp_range_c", _pair(self.abort_temp_range_c, "WeatherThresholds.abort_temp_range_c"))

    def go_violations(self, obs: "WeatherObs", w_go_mps: float) -> list[str]:
        """放飛門檻違反清單（空 = 可放飛；規格 §6.2 步 2）。`w_go_mps` = min over fleet of `resolve_weather_go`。"""
        out: list[str] = []
        if obs.mean_wind_mps > w_go_mps:
            out.append(f"mean_wind {obs.mean_wind_mps:.1f} > go {w_go_mps:.1f} m/s")
        if obs.gust_mps > self.go_gust_mps:
            out.append(f"gust {obs.gust_mps:.1f} > go {self.go_gust_mps:.1f} m/s")
        if obs.visibility_m < self.go_visibility_m:
            out.append(f"visibility {obs.visibility_m:.0f} < go {self.go_visibility_m:.0f} m")
        if obs.precip_mmh > self.go_precip_mmh:
            out.append(f"precip {obs.precip_mmh:.2f} > go {self.go_precip_mmh:.2f} mm/h")
        lo, hi = self.go_temp_range_c
        if not (lo <= obs.temp_c <= hi):
            out.append(f"temp {obs.temp_c:.1f} outside go range [{lo:.1f}, {hi:.1f}] C")
        return out

    def abort_violations(self, obs: "WeatherObs") -> list[str]:
        """中止門檻違反清單（空 = 不中止；規格 §12.9、D7）。"""
        out: list[str] = []
        if obs.mean_wind_mps > self.abort_mean_wind_mps:
            out.append(f"mean_wind {obs.mean_wind_mps:.1f} > abort {self.abort_mean_wind_mps:.1f} m/s")
        if obs.gust_mps > self.abort_gust_mps:
            out.append(f"gust {obs.gust_mps:.1f} > abort {self.abort_gust_mps:.1f} m/s")
        if obs.visibility_m < self.abort_visibility_m:
            out.append(f"visibility {obs.visibility_m:.0f} < abort {self.abort_visibility_m:.0f} m")
        if obs.precip_mmh > self.abort_precip_mmh:
            out.append(f"precip {obs.precip_mmh:.2f} > abort {self.abort_precip_mmh:.2f} mm/h")
        lo, hi = self.abort_temp_range_c
        if not (lo <= obs.temp_c <= hi):
            out.append(f"temp {obs.temp_c:.1f} outside abort range [{lo:.1f}, {hi:.1f}] C")
        return out


@dataclass(frozen=True, slots=True)
class WeatherObs:
    """氣象觀測（`temp_c` 僅門檻用；電池溫度缺省時亦取此值並警告）。"""
    mean_wind_mps: float
    gust_mps: float
    wind_dir_from_deg: float
    visibility_m: float
    precip_mmh: float
    temp_c: float
    t_s: float = 0.0

    def __post_init__(self) -> None:
        for key in ("mean_wind_mps", "gust_mps", "visibility_m", "precip_mmh"):
            object.__setattr__(self, key, _nonneg(_num(getattr(self, key), f"WeatherObs.{key}"), f"WeatherObs.{key}"))
        object.__setattr__(self, "wind_dir_from_deg", _num(self.wind_dir_from_deg, "WeatherObs.wind_dir_from_deg"))
        object.__setattr__(self, "temp_c", _num(self.temp_c, "WeatherObs.temp_c"))
        object.__setattr__(self, "t_s", _num(self.t_s, "WeatherObs.t_s"))


@dataclass(frozen=True, slots=True)
class UAVInstance:
    """機隊成員：平台規格（已套用覆寫前之基礎 spec）、機上感測器硬體（主感測器在前）、當架次能量。
    A 掛 RGB 時 `sensors=(LidarHardware(), CameraHardware())`。

    `start_site_id`（選填）：本機**起飛**站點之 `Site.site_id`（各機自有地面站；論文 §4.1 情境）。
    None → `MissionInputs.home`（既有情境與測試行為不變）。回收／降落仍依 D40（`recovery_site_id` → home），
    起飛點與回收點可不同。站點存在性由 `MissionInputs.__post_init__` 檢查。
    """
    uav_id: str
    spec: PlatformSpec
    sensors: tuple[SensorHardware, ...]
    sortie: SortieEnergy
    start_site_id: str | None = None

    def __post_init__(self) -> None:
        _str_id(self.uav_id, "UAVInstance.uav_id")
        n = f"UAVInstance[{self.uav_id}]"
        if not isinstance(self.spec, PlatformSpec):
            raise MissionInputError(f"{n}.spec 須為 PlatformSpec")
        if self.spec.cls not in CLASSES:
            raise MissionInputError(f"{n}.spec.cls 須為 A/B，得 {self.spec.cls!r}")
        s = self.sensors
        if isinstance(s, (LidarHardware, CameraHardware, TrapezoidFovSpec)):
            s = (s,)
        s = tuple(s or ())
        if not s:
            raise MissionInputError(f"{n}.sensors 不可為空（主感測器在前）")
        for hw in s:
            if not isinstance(hw, (LidarHardware, CameraHardware, TrapezoidFovSpec)):
                raise MissionInputError(f"{n}.sensors 元素須為 LidarHardware/CameraHardware/TrapezoidFovSpec，得 {type(hw).__name__}")
        object.__setattr__(self, "sensors", s)
        if not isinstance(self.sortie, SortieEnergy):
            raise MissionInputError(f"{n}.sortie 須為 SortieEnergy")
        if self.sortie.uav_id != self.uav_id:
            raise MissionInputError(f"{n}: sortie.uav_id={self.sortie.uav_id!r} 與 uav_id 不符")
        if self.start_site_id is not None:
            _str_id(self.start_site_id, f"{n}.start_site_id")
            object.__setattr__(self, "start_site_id", str(self.start_site_id))

    @property
    def available_kinds(self) -> frozenset[str]:
        """機上可用感測種類 {hw.kind}（取代舊 a_has_rgb 布林欄位；κ(σ,c) 判定用）。"""
        return frozenset(hw.kind for hw in self.sensors)

    @property
    def cls(self) -> str:
        return self.spec.cls

    @property
    def primary_sensor(self) -> SensorHardware:
        return self.sensors[0]


def allowed_override_keys(spec: PlatformSpec) -> frozenset[str]:
    """`platform_overrides` 允許鍵：provisional ∪ provisional_supplementary ∪ {min_fw_operation_agl_m}（規格 §3.5）。"""
    return frozenset(spec.provisional) | frozenset(spec.provisional_supplementary) | frozenset({"min_fw_operation_agl_m"})


@dataclass(frozen=True, slots=True)
class MissionInputs:
    """表 2 全部任務輸入（規格 §6.1）。`platform_overrides[cls]` 只允許 provisional ∪ provisional_supplementary
    與 `min_fw_operation_agl_m`（只可調高，由 `build_context` 檢查）；`recovery_site_id` None → home（D40）。"""
    home: Site
    alternates: tuple[Site, ...]
    areas: tuple[AreaSpec, ...]
    dem: DemGrid
    obstacles: tuple[Obstacle, ...]
    wind: tuple[WindLayer, ...]
    fleet: tuple[UAVInstance, ...]
    payload_reqs: dict[str, PayloadRequirement]
    c2: C2Link
    airspace: Airspace
    weather_thresholds: WeatherThresholds
    weather_obs: WeatherObs
    t0_epoch_s: float = 0.0
    platform_overrides: dict[str, dict[str, float]] = field(default_factory=dict)
    recovery_site_id: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.home, Site):
            raise MissionInputError("MissionInputs.home 須為 Site")
        for key, typ in (("alternates", Site), ("areas", AreaSpec), ("obstacles", Obstacle),
                         ("wind", WindLayer), ("fleet", UAVInstance)):
            v = getattr(self, key)
            if isinstance(v, typ):
                v = (v,)
            v = tuple(v or ())
            for x in v:
                if not isinstance(x, typ):
                    raise MissionInputError(f"MissionInputs.{key} 元素須為 {typ.__name__}，得 {type(x).__name__}")
            object.__setattr__(self, key, v)
        if not isinstance(self.dem, DemGrid):
            raise MissionInputError("MissionInputs.dem 須為 DemGrid")
        if not isinstance(self.c2, C2Link):
            raise MissionInputError("MissionInputs.c2 須為 C2Link")
        if not isinstance(self.airspace, Airspace):
            raise MissionInputError("MissionInputs.airspace 須為 Airspace")
        if not isinstance(self.weather_thresholds, WeatherThresholds):
            raise MissionInputError("MissionInputs.weather_thresholds 須為 WeatherThresholds")
        if not isinstance(self.weather_obs, WeatherObs):
            raise MissionInputError("MissionInputs.weather_obs 須為 WeatherObs")
        # 站點 id 唯一
        site_ids = [self.home.site_id] + [s.site_id for s in self.alternates]
        if len(set(site_ids)) != len(site_ids):
            raise MissionInputError(f"站點 site_id 重複：{site_ids}")
        # 區域 id 唯一、非空
        if not self.areas:
            raise MissionInputError("MissionInputs.areas 不可為空")
        area_ids = [a.area_id for a in self.areas]
        if len(set(area_ids)) != len(area_ids):
            raise MissionInputError(f"AreaSpec.area_id 重複：{area_ids}")
        obs_ids = [o.obs_id for o in self.obstacles]
        if len(set(obs_ids)) != len(obs_ids):
            raise MissionInputError(f"Obstacle.obs_id 重複：{obs_ids}")
        # 機隊
        if not self.fleet:
            raise MissionInputError("MissionInputs.fleet 不可為空")
        uav_ids = [u.uav_id for u in self.fleet]
        if len(set(uav_ids)) != len(uav_ids):
            raise MissionInputError(f"UAVInstance.uav_id 重複：{uav_ids}")
        # 逐機起飛站點（選填；None → home）
        for u in self.fleet:
            if u.start_site_id is not None and u.start_site_id not in site_ids:
                raise MissionInputError(
                    f"UAVInstance[{u.uav_id}].start_site_id={u.start_site_id!r} 不是已知站點 {site_ids}")
        # 酬載需求
        pr = self.payload_reqs
        if pr is None:
            pr = {}
        if isinstance(pr, (list, tuple)):
            pr = {r.req_id: r for r in pr}
        if not isinstance(pr, Mapping):
            raise MissionInputError("MissionInputs.payload_reqs 須為 {req_id: PayloadRequirement}")
        for k, r in pr.items():
            if not isinstance(r, PayloadRequirement):
                raise MissionInputError(f"payload_reqs[{k!r}] 須為 PayloadRequirement")
            if r.req_id != k:
                raise MissionInputError(f"payload_reqs 鍵 {k!r} 與 req_id {r.req_id!r} 不符")
        object.__setattr__(self, "payload_reqs", dict(pr))
        missing = sorted({a.payload_req_id for a in self.areas} - set(pr))
        if missing:
            raise MissionInputError(f"AreaSpec.payload_req_id 未定義於 payload_reqs：{missing}")
        # 失鏈策略之 uav_id 須存在
        unknown = sorted(set(self.c2.lost_link_policy) - set(uav_ids))
        if unknown:
            raise MissionInputError(f"C2Link.lost_link_policy 含未知 uav_id：{unknown}")
        object.__setattr__(self, "t0_epoch_s", _num(self.t0_epoch_s, "MissionInputs.t0_epoch_s"))
        # 覆寫
        ov = self.platform_overrides
        if ov is None:
            ov = {}
        if not isinstance(ov, Mapping):
            raise MissionInputError("MissionInputs.platform_overrides 須為 {cls: {key: value}}")
        clean: dict[str, dict[str, float]] = {}
        specs_by_cls = {u.spec.cls: u.spec for u in self.fleet}
        for cls, kv in ov.items():
            _set(cls, CLASSES, "platform_overrides key")
            if kv is None:
                continue
            if not isinstance(kv, Mapping):
                raise MissionInputError(f"platform_overrides[{cls!r}] 須為 {{key: value}}")
            spec = specs_by_cls.get(cls, _SPEC_BY_CLS[cls])
            allowed = allowed_override_keys(spec)
            bad = sorted(set(kv) - allowed)
            if bad:
                raise MissionInputError(f"platform_overrides[{cls!r}] 含不允許覆寫之鍵：{bad}（只允許 provisional ∪ "
                                        f"provisional_supplementary ∪ {{min_fw_operation_agl_m}}）")
            clean[cls] = {str(k): (tuple(v) if isinstance(v, (list, tuple)) else v) for k, v in kv.items()}
        object.__setattr__(self, "platform_overrides", clean)
        # 回收點（D40）
        if self.recovery_site_id is not None:
            _str_id(self.recovery_site_id, "MissionInputs.recovery_site_id")
            if self.recovery_site_id not in site_ids:
                raise MissionInputError(f"recovery_site_id={self.recovery_site_id!r} 不是已知站點 {site_ids}")

    # -- 查詢 -----------------------------------------------------------------
    @property
    def sites(self) -> dict[str, Site]:
        """site_id → Site（home 在前）。"""
        d = {self.home.site_id: self.home}
        for s in self.alternates:
            d[s.site_id] = s
        return d

    def site_by_id(self, site_id: str) -> Site:
        try:
            return self.sites[site_id]
        except KeyError as e:
            raise MissionInputError(f"未知站點 {site_id!r}") from e

    @property
    def recovery_site(self) -> Site:
        """回收點：`recovery_site_id` → home（D40）。"""
        return self.home if self.recovery_site_id is None else self.site_by_id(self.recovery_site_id)

    def start_site_id_of(self, uav_id: str) -> str:
        """該機起飛站點 id：`UAVInstance.start_site_id` → home（未設定時；既有情境行為不變）。"""
        return self.uav_by_id(uav_id).start_site_id or self.home.site_id

    def start_site(self, uav_id: str) -> Site:
        """該機起飛站點 :class:`Site`（未設定 `start_site_id` → home）。"""
        return self.site_by_id(self.start_site_id_of(uav_id))

    @property
    def start_sites(self) -> dict[str, str]:
        """uav_id → 起飛站點 site_id（未設定者填 home）。"""
        return {u.uav_id: (u.start_site_id or self.home.site_id) for u in self.fleet}

    def area_by_id(self, area_id: str) -> AreaSpec:
        for a in self.areas:
            if a.area_id == area_id:
                return a
        raise MissionInputError(f"未知區域 {area_id!r}")

    def uav_by_id(self, uav_id: str) -> UAVInstance:
        for u in self.fleet:
            if u.uav_id == uav_id:
                return u
        raise MissionInputError(f"未知 UAV {uav_id!r}")

    @property
    def fleet_classes(self) -> frozenset[str]:
        return frozenset(u.spec.cls for u in self.fleet)

    def uavs_of_class(self, cls: str) -> tuple[UAVInstance, ...]:
        return tuple(u for u in self.fleet if u.spec.cls == cls)

    # -- 序列化 -----------------------------------------------------------------
    def to_dict(self) -> dict[str, Any]:
        """JSON 可序列化字典；`load_mission(inputs.to_dict())` 往返一致（numpy → list、frozenset → sorted list）。"""
        return {
            "home": to_jsonable(self.home),
            "alternates": [to_jsonable(s) for s in self.alternates],
            "areas": [to_jsonable(a) for a in self.areas],
            "dem": _dem_to_dict(self.dem),
            "obstacles": [to_jsonable(o) for o in self.obstacles],
            "wind": [to_jsonable(w) for w in self.wind],
            "fleet": [_uav_to_dict(u) for u in self.fleet],
            "payload_reqs": {k: to_jsonable(v) for k, v in self.payload_reqs.items()},
            "c2": to_jsonable(self.c2),
            "airspace": to_jsonable(self.airspace),
            "weather_thresholds": to_jsonable(self.weather_thresholds),
            "weather_obs": to_jsonable(self.weather_obs),
            "t0_epoch_s": self.t0_epoch_s,
            "platform_overrides": to_jsonable(self.platform_overrides),
            "recovery_site_id": self.recovery_site_id,
        }

    def to_json(self, indent: int | None = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, ensure_ascii=False)


# ---------------------------------------------------------------------------
# PlannerConfig（規格 §16.2 全部參數）
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class PlannerConfig:
    """演算法／模型參數（規格 §16.2；欄位名 = 表中參數名）。

    `legacy_mode=True`（規格 §15、D32）→ 建構時一鍵改寫：`idp_branching="lockstep"`、`transfer_types=("rev",)`、`B=1`、
    `K=K_INF`、`metric="paper"`、`convexify_whole=True`、`fov_model="trapezoid"`、`e_ext_override_m=0`、`k_bm=k_as=1.0`、
    能量／鏈路／時間窗限制關閉（`enforce_energy/enforce_link/enforce_time_windows=False`）、`t_rtb_lower_bound=False`
    （t̂_RTB=0）、`h_ceil_override_m=inf`。:meth:`apply_legacy` 回傳套用後之複本。
    契約未列之尾端欄位（`enforce_*`、`t_rtb_lower_bound`、`e_ext_override_m`、`h_ceil_override_m`）皆有預設值，
    供上層模組讀取退化旗標（記錄於 CONTRACT_DEVIATIONS）。
    """
    dem_step_m: float = 10.0
    dp_eps_factor: float = 0.1
    merge_tau_factor: float = 0.5
    k_sigma: float = 2.0
    sigma_xy_m: float = 3.0
    e_xt_m: float = 5.0
    delta_edge_m: float | None = 15.0          # None → k_σ·σ_xy + e_xt + δ_w/2（delta_edge_resolved）
    b_trk_m: float = 50.0
    b_h_m: float = 50.0
    b_v_m: float = 30.0
    dh_band_min_m: float = 20.0
    t_settle_s: float = 2.0
    k_slope_pen: float = 20.0
    e_max_m: float = 3.0
    psi_max_deg: float = 30.0
    dtheta_int_deg: float = 5.0
    lambda_cut: float = 1e-3
    lambda_n: float = 30.0
    convexify_whole: bool = False
    widen_on_lift: bool = False
    v_g_min_mps: float = 3.0
    r_eff_wind_policy: Literal["max_layer", "op_layer"] = "max_layer"
    k_bm: float = 1.15
    k_as: float = 1.05
    temp_cap_coef_per_c: float = 0.01
    temp_cap_floor: float = 0.8
    site_max_slope_deg: float = 5.0
    transition_profile: Literal["rotor_assisted", "fw_slope"] = "rotor_assisted"
    nu: tuple[float, ...] = (0.15, 0.15, 0.20, 0.20, 0.20, 0.10)
    omega: tuple[float, ...] = (0.2, 0.15, 0.2, 0.15, 0.1, 0.2)
    kappa_load: float = 0.85
    n_local_search: int = 200
    multi_sortie: bool = False
    t_swap_s: float = 600.0
    dt_dep_s: float = 60.0
    t_sep_ldg_s: float = 90.0
    idp_branching: Literal["event", "lockstep"] = "event"
    transfer_types: tuple[str, ...] = ("rev", "same")
    # ⚠⚠⚠ vendored 修改 #2（見 docs/hv_dccpp/VENDOR.md）。
    #   估計層（decompose.exact_time / allocate.sweep_order）原本用**固定**的
    #   skip-k 序列，而規劃層（planner IDP beam）是**搜尋**的 ⇒ 估計器系統性悲觀，
    #   而且偏差隨條帶數二次成長（實測 n=14 多 22%、n=16 對 Held–Karp 是 1.24×），
    #   再經由 cell.T_single_s 傳進掃描方向排序、λ_floor 與配置 makespan。
    #   "fixed"  = 上游原行為（固定 _skip_sequence，可完整重現舊結果）
    #   "search" = 對候選集取 argmin（候選集**含**固定序列 ⇒ 不可能比它差）
    sweep_sequence: Literal["fixed", "search"] = "search"
    #   n ≤ 此值時額外加入 Held–Karp 精確開放路徑（2^n·n² ⇒ 9 約 41k 次運算）
    sweep_exact_max_n: int = 9
    K: int = 6
    B: int = 3
    eps_sum: float = 0.05
    mu_paper: float = 0.0
    tau0_s: float = 60.0
    realtime_budget_s: float = 1.0
    d_d_m: float = 25.0
    n_smooth: int = 50
    c1: float = 1.0
    c2: float = 0.5
    lr: float = 0.2
    eps_col: float = 1e-3
    L_max_m: float = 500.0
    k_gsd: float = 1.0
    k_gsd_relax: float = 1.2
    h_transit_offset_A_m: float = 50.0
    dh_sep_m: float = 50.0
    delta_prof_m: float = 10.0
    d_sep_m: float = 100.0
    d_sep_lat_m: float = 60.0
    k_t: float = 0.05
    dt_4d_s: float = 2.0
    delta_1: float = 0.05
    delta_2: float = 0.15
    energy_margin_frac: float = 0.05
    r_conf_m: float = 150.0
    gsd_confirm_m: float = 0.025
    t_conf_max_s: float = 900.0
    tau_cue: float = 0.10
    tau_w: float = 0.05
    confirm_mode: Literal["strips", "orbit"] = "strips"
    cov_raster_m: float = 5.0
    fov_model: Literal["sensor", "trapezoid"] = "sensor"
    metric: Literal["makespan", "paper"] = "makespan"
    legacy_mode: bool = False
    seed: int = 0
    # --- 退化旗標（契約未列；預設 = 完整演算法；legacy_mode 改寫）---
    enforce_energy: bool = True                 # False → 能量限制關閉（E 視為無限）
    enforce_link: bool = True                   # False → 鏈路／半徑限制關閉
    enforce_time_windows: bool = True           # False → 三種時間窗關閉（D22）
    t_rtb_lower_bound: bool = True              # False → 階段成本之 t̂_RTB = 0
    e_ext_override_m: float | None = None       # None → 感測器公式；legacy 0.0
    h_ceil_override_m: float | None = None      # None → 空域；legacy inf

    LEGACY_OVERRIDES = {
        # ⚠ legacy_mode 要退回論文的行為 ⇒ 序列搜尋也必須關掉，否則 D32 的
        #   「一鍵退化回基準線演算法」就不成立了。
        "sweep_sequence": "fixed",
        "idp_branching": "lockstep", "transfer_types": ("rev",), "B": 1, "K": K_INF, "metric": "paper",
        "convexify_whole": True, "fov_model": "trapezoid", "e_ext_override_m": 0.0, "k_bm": 1.0, "k_as": 1.0,
        "enforce_energy": False, "enforce_link": False, "enforce_time_windows": False, "t_rtb_lower_bound": False,
        "h_ceil_override_m": math.inf,
    }

    def __post_init__(self) -> None:
        # 序列正規化（JSON list → tuple）
        object.__setattr__(self, "nu", tuple(float(x) for x in self.nu))
        object.__setattr__(self, "omega", tuple(float(x) for x in self.omega))
        object.__setattr__(self, "transfer_types", tuple(str(x) for x in self.transfer_types))
        if isinstance(self.K, float) and math.isinf(self.K):
            object.__setattr__(self, "K", K_INF)
        if self.legacy_mode:
            for k, v in self.LEGACY_OVERRIDES.items():
                object.__setattr__(self, k, v)
        # 驗證
        if len(self.nu) != 6 or len(self.omega) != 6:
            raise ValueError("PlannerConfig.nu / omega 須各為 6 個權重")
        if any(x < 0 for x in self.nu) or any(x < 0 for x in self.omega):
            raise ValueError("PlannerConfig.nu / omega 權重須 ≥ 0")
        if not self.transfer_types or not set(self.transfer_types) <= {"rev", "same"}:
            raise ValueError(f"PlannerConfig.transfer_types 須為 ('rev',) 或 ('rev','same')，得 {self.transfer_types}")
        if int(self.K) < 1 or int(self.B) < 1:
            raise ValueError("PlannerConfig.K、B 須 ≥ 1")
        object.__setattr__(self, "K", int(self.K))
        object.__setattr__(self, "B", int(self.B))
        for key in ("dem_step_m", "b_trk_m", "b_h_m", "b_v_m", "dh_band_min_m", "e_max_m", "psi_max_deg", "dtheta_int_deg",
                    "v_g_min_mps", "d_d_m", "cov_raster_m", "dt_4d_s", "L_max_m", "dt_dep_s", "t_sep_ldg_s", "tau0_s"):
            if float(getattr(self, key)) <= 0.0:
                raise ValueError(f"PlannerConfig.{key} 須 > 0")
        for key in ("k_bm", "k_as", "temp_cap_floor", "kappa_load", "k_gsd", "k_gsd_relax"):
            if float(getattr(self, key)) <= 0.0:
                raise ValueError(f"PlannerConfig.{key} 須 > 0")
        if self.delta_edge_m is not None and float(self.delta_edge_m) < 0.0:
            raise ValueError("PlannerConfig.delta_edge_m 須 ≥ 0 或 None")
        if self.r_eff_wind_policy not in ("max_layer", "op_layer"):
            raise ValueError(f"PlannerConfig.r_eff_wind_policy 非法：{self.r_eff_wind_policy!r}")
        if self.transition_profile not in ("rotor_assisted", "fw_slope"):
            raise ValueError(f"PlannerConfig.transition_profile 非法：{self.transition_profile!r}")
        if self.idp_branching not in ("event", "lockstep"):
            raise ValueError(f"PlannerConfig.idp_branching 非法：{self.idp_branching!r}")
        if self.confirm_mode not in ("strips", "orbit"):
            raise ValueError(f"PlannerConfig.confirm_mode 非法：{self.confirm_mode!r}")
        if self.fov_model not in ("sensor", "trapezoid"):
            raise ValueError(f"PlannerConfig.fov_model 非法：{self.fov_model!r}")
        if self.metric not in ("makespan", "paper"):
            raise ValueError(f"PlannerConfig.metric 非法：{self.metric!r}")

    # -- 派生 -----------------------------------------------------------------
    def apply_legacy(self) -> "PlannerConfig":
        """回傳套用 `legacy_mode` 全部退化條件之複本（規格 §15；原物件不變）。"""
        return replace(self, legacy_mode=True)

    def effective(self) -> "PlannerConfig":
        """`legacy_mode` 已於建構時套用；此方法為明確化之同義呼叫（回傳 self）。"""
        return self

    @property
    def psi_max_rad(self) -> float:
        return math.radians(self.psi_max_deg)

    @property
    def dtheta_int_rad(self) -> float:
        return math.radians(self.dtheta_int_deg)

    @property
    def site_max_slope_rad(self) -> float:
        return math.radians(self.site_max_slope_deg)

    def delta_edge_resolved(self, delta_w_m: float = 0.0) -> float:
        """δ_edge：給定值，或 None → k_σ·σ_xy + e_xt + δ_w/2（規格 §7.2、§16.2）。"""
        if self.delta_edge_m is not None:
            return float(self.delta_edge_m)
        return self.k_sigma * self.sigma_xy_m + self.e_xt_m + 0.5 * float(delta_w_m)

    def k_nearest(self, n_candidates: int) -> int:
        """k-nearest 候選數 min(K, n)（K_INF → 全部）。"""
        return int(min(self.K, max(0, int(n_candidates))))

    def to_dict(self) -> dict[str, Any]:
        return {f.name: to_jsonable(getattr(self, f.name)) for f in fields(self)}

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "PlannerConfig":
        names = {f.name for f in fields(cls)}
        bad = sorted(set(d) - names)
        if bad:
            raise ValueError(f"PlannerConfig 未知參數：{bad}")
        kw = dict(d)
        if "K" in kw and isinstance(kw["K"], str):
            kw["K"] = K_INF if kw["K"].lower() in ("inf", "infinity") else int(kw["K"])
        for key in ("h_ceil_override_m",):
            if isinstance(kw.get(key), str) and kw[key].lower() in ("inf", "infinity"):
                kw[key] = math.inf
        return cls(**kw)

    def to_json(self, indent: int | None = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, ensure_ascii=False)


# ---------------------------------------------------------------------------
# 模式／階段列舉、規劃狀態與輸出型別
# ---------------------------------------------------------------------------

class Mode(Enum):
    """飛行模式（規格 §3.4）。"""
    HOVER = auto()
    TRANSITION = auto()
    CRUISE = auto()


class Phase(Enum):
    """航段階段（規格 §3.4、§11.1）。"""
    TAKEOFF = auto()
    TRANS_OUT = auto()
    ACCEL = auto()
    CLIMB = auto()
    ORBIT = auto()
    ENTRY = auto()
    OPERATE = auto()
    TRANSFER = auto()
    RTB = auto()
    DESCENT = auto()
    DECEL = auto()
    TRANS_IN = auto()
    LAND = auto()
    HOLD = auto()


FW_PHASES: frozenset[Phase] = frozenset({
    Phase.ACCEL, Phase.CLIMB, Phase.ORBIT, Phase.ENTRY, Phase.OPERATE, Phase.TRANSFER,
    Phase.RTB, Phase.DESCENT, Phase.DECEL, Phase.HOLD,
})
"""固定翼階段：`mode == CRUISE` 且 `airspeed ≥ spec.v_min_mission()`。"""

VTOL_PHASES: frozenset[Phase] = frozenset({Phase.TAKEOFF, Phase.LAND})
"""`mode == HOVER` 只允許出現於此集合（A7 無懸停轉向）。"""

TRANSITION_PHASES: frozenset[Phase] = frozenset({Phase.TRANS_OUT, Phase.TRANS_IN})
"""`mode == TRANSITION` 之階段。"""


def mode_phase_violation(mode: Mode, phase: Phase, airspeed_mps: float, v_min_mission_mps: float) -> str | None:
    """Waypoint 模式不變量（契約 §3；由 `sim.check_kinematics` 呼叫）：違反 → 說明字串；否則 None。

    `mode==HOVER ⇒ phase ∈ {TAKEOFF, LAND}`；`phase ∈ FW_PHASES ⇒ mode==CRUISE ∧ airspeed ≥ v_min_mission`；
    `phase ∈ {TRANS_OUT, TRANS_IN} ⇒ mode==TRANSITION`。
    """
    if mode is Mode.HOVER and phase not in VTOL_PHASES:
        return f"HOVER outside VTOL phases ({phase.name})"
    if phase in FW_PHASES:
        if mode is not Mode.CRUISE:
            return f"FW phase {phase.name} with mode {mode.name}"
        if airspeed_mps < v_min_mission_mps - 1e-9:
            return f"FW phase {phase.name} airspeed {airspeed_mps:.2f} < v_min_mission {v_min_mission_mps:.2f}"
    if phase in TRANSITION_PHASES and mode is not Mode.TRANSITION:
        return f"phase {phase.name} with mode {mode.name}"
    return None


@dataclass(slots=True)
class UAVState:
    """規劃用狀態（ENU；θ 自北順時針；z 為 ENU z）。"""
    x_m: float
    y_m: float
    theta_rad: float
    z_m: float
    t_s: float
    e_rem_wh: float
    r_eff_m: float
    v_air_mps: float

    @property
    def xy(self) -> np.ndarray:
        return np.array([self.x_m, self.y_m], dtype=np.float64)

    def as_pose_array(self) -> np.ndarray:
        """(3,) [x, y, theta]（供 dubins 向量化介面）。"""
        return np.array([self.x_m, self.y_m, self.theta_rad], dtype=np.float64)

    def copy(self) -> "UAVState":
        return replace(self)

    def to_dict(self) -> dict[str, Any]:
        return {f.name: to_jsonable(getattr(self, f.name)) for f in fields(self)}


@dataclass(slots=True)
class Waypoint:
    """輸出航點：同時給 ENU `z_m` 與 `alt_msl_m`（D42）；`arc` 弧命令 (cx, cy, R, turn_dir ±1)，None = 直線。
    `lost_link_action ∈ {"RTB", "CONTINUE", "ALT:<site_id>"}`。"""
    x_m: float
    y_m: float
    z_m: float
    lat_deg: float
    lon_deg: float
    alt_msl_m: float
    agl_m: float
    t_s: float
    mode: Mode
    phase: Phase
    sensor_on: bool
    strip_id: str | None
    cell_id: str | None
    dir: int | None
    airspeed_mps: float
    ground_speed_mps: float
    track_deg: float
    heading_deg: float
    energy_remaining_wh: float
    nearest_alternate: str
    alt_reachable: bool
    lost_link_action: str
    arc: tuple[float, float, float, int] | None = None
    accept_radius_m: float = 30.0

    def to_dict(self) -> dict[str, Any]:
        d = {f.name: to_jsonable(getattr(self, f.name)) for f in fields(self)}
        return d

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "Waypoint":
        kw = dict(d)
        kw["mode"] = _enum_from(Mode, kw["mode"])
        kw["phase"] = _enum_from(Phase, kw["phase"])
        arc = kw.get("arc")
        kw["arc"] = None if arc is None else (float(arc[0]), float(arc[1]), float(arc[2]), int(arc[3]))
        if kw.get("dir") is not None:
            kw["dir"] = int(kw["dir"])
        return cls(**kw)


@dataclass(slots=True)
class UAVPlan:
    """單機航線輸出。`t_arr_ptc_s` = 抵達降落走廊 P_tc 時刻（CorridorSchedule 用）；
    `warnings` 含 planned_landing_at_alternate、landing_over_limit。"""
    uav_id: str
    cls: str
    waypoints: list[Waypoint]
    strips: list[tuple[str, int]]
    t_takeoff_s: float
    t_arr_ptc_s: float
    t_land_s: float
    landing_site: str
    energy_used_wh: float
    energy_margin_wh: float
    distance_m: float
    cruise_equiv_km: float
    orbits_inserted: int
    warnings: list[str]

    def to_dict(self) -> dict[str, Any]:
        return {
            "uav_id": self.uav_id, "cls": self.cls,
            "waypoints": [w.to_dict() for w in self.waypoints],
            "strips": [[s, int(d)] for s, d in self.strips],
            "t_takeoff_s": self.t_takeoff_s, "t_arr_ptc_s": self.t_arr_ptc_s, "t_land_s": self.t_land_s,
            "landing_site": self.landing_site,
            "energy_used_wh": self.energy_used_wh, "energy_margin_wh": self.energy_margin_wh,
            "distance_m": self.distance_m, "cruise_equiv_km": self.cruise_equiv_km,
            "orbits_inserted": int(self.orbits_inserted), "warnings": list(self.warnings),
        }

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "UAVPlan":
        kw = dict(d)
        kw["waypoints"] = [Waypoint.from_dict(w) for w in kw.get("waypoints", [])]
        kw["strips"] = [(str(s), int(dd)) for s, dd in kw.get("strips", [])]
        kw["warnings"] = list(kw.get("warnings", []))
        return cls(**kw)

    def to_json(self, indent: int | None = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, ensure_ascii=False)


@dataclass(slots=True)
class FeasibilityReport:
    """前置檢查報告（規格 §6.2 步 13）。`derived[cls] = {R_eff, V_g_cap, h_op, w, s, P_c_ref, h_ceil_source, …}`。"""
    status: Literal["GO", "GO_WITH_WARNINGS", "NO_GO"]
    errors: list[str]
    warnings: list[str]
    per_area: dict[str, dict]
    per_uav: dict[str, dict]
    derived: dict[str, dict]

    @property
    def ok(self) -> bool:
        return self.status != "NO_GO"

    def to_dict(self) -> dict[str, Any]:
        return {f.name: to_jsonable(getattr(self, f.name)) for f in fields(self)}

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "FeasibilityReport":
        return cls(status=d["status"], errors=list(d.get("errors", [])), warnings=list(d.get("warnings", [])),
                   per_area=dict(d.get("per_area", {})), per_uav=dict(d.get("per_uav", {})),
                   derived=dict(d.get("derived", {})))

    def to_json(self, indent: int | None = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, ensure_ascii=False)


@dataclass(slots=True)
class MissionPlan:
    """整體規劃輸出。`uncovered` = (strip_id | cell_id, reason)；`cache_keys` = (geometry_key, wind_key, state_key)。"""
    plans: dict[str, UAVPlan]
    makespan_s: float
    cells: list["Cell"]
    strips: dict[str, "Strip"]
    uncovered: list[tuple[str, str]]
    feasibility: FeasibilityReport
    timing_ms: dict[str, float]
    cache_keys: tuple[int, int, int]
    diff_from: str | None = None

    def to_dict(self) -> dict[str, Any]:
        """JSON 可序列化（單向）：cells／strips 以通用序列化輸出（shapely → GeoJSON mapping、numpy → list）。"""
        return {
            "plans": {k: v.to_dict() for k, v in self.plans.items()},
            "makespan_s": self.makespan_s,
            "cells": [to_jsonable(c) for c in self.cells],
            "strips": {k: to_jsonable(s) for k, s in self.strips.items()},
            "uncovered": [[a, b] for a, b in self.uncovered],
            "feasibility": self.feasibility.to_dict(),
            "timing_ms": {k: float(v) for k, v in self.timing_ms.items()},
            "cache_keys": [int(x) for x in self.cache_keys],
            "diff_from": self.diff_from,
        }

    def to_json(self, indent: int | None = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, ensure_ascii=False)


# ---------------------------------------------------------------------------
# 氣象
# ---------------------------------------------------------------------------

def resolve_weather_go(spec: PlatformSpec, th: WeatherThresholds) -> float:
    """放飛平均風門檻 W_go = min(8.0, 0.45·Va)（A 8.0、B 8.0；D7）或 `th.go_mean_wind_mps` 覆寫值。"""
    if th.go_mean_wind_mps is not None:
        return float(th.go_mean_wind_mps)
    return min(8.0, 0.45 * float(spec.cruise_airspeed_mps))


# ---------------------------------------------------------------------------
# 通用序列化
# ---------------------------------------------------------------------------

def _enum_from(enum_cls: type[Enum], v: Any) -> Enum:
    if isinstance(v, enum_cls):
        return v
    if isinstance(v, str):
        try:
            return enum_cls[v]
        except KeyError as e:
            raise ValueError(f"{enum_cls.__name__} 無成員 {v!r}") from e
    return enum_cls(v)


def to_jsonable(obj: Any) -> Any:
    """遞迴轉為 JSON 可序列化物件：dataclass → dict、numpy → list/scalar、Enum → name、set/frozenset → sorted list、
    tuple → list、shapely（`__geo_interface__`）→ GeoJSON mapping、有 `to_dict()` 者優先呼叫之。"""
    if obj is None or isinstance(obj, (bool, str)):
        return obj
    if isinstance(obj, (int, float)):
        return obj
    if isinstance(obj, np.generic):
        return obj.item()
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, Enum):
        return obj.name
    if isinstance(obj, Path):
        return str(obj)
    if hasattr(obj, "to_dict") and callable(getattr(obj, "to_dict")):
        return obj.to_dict()
    if is_dataclass(obj) and not isinstance(obj, type):
        return {f.name: to_jsonable(getattr(obj, f.name)) for f in fields(obj)}
    if isinstance(obj, Mapping):
        out: dict[str, Any] = {}
        for k, v in obj.items():
            if isinstance(k, tuple):
                key = "|".join(str(x) for x in k)
            elif isinstance(k, Enum):
                key = k.name
            else:
                key = str(k)
            out[key] = to_jsonable(v)
        return out
    if isinstance(obj, (set, frozenset)):
        try:
            return sorted(to_jsonable(x) for x in obj)
        except TypeError:
            return [to_jsonable(x) for x in obj]
    if isinstance(obj, (list, tuple)):
        return [to_jsonable(x) for x in obj]
    if hasattr(obj, "__geo_interface__"):
        return to_jsonable(obj.__geo_interface__)
    return str(obj)


def _dem_to_dict(dem: DemGrid) -> dict[str, Any]:
    return {
        "lat0_deg": dem.lat0_deg, "lon0_deg": dem.lon0_deg, "dlat_deg": dem.dlat_deg, "dlon_deg": dem.dlon_deg,
        "z_msl": np.asarray(dem.z_msl, dtype=np.float32).tolist(),
        "is_dsm": bool(dem.is_dsm), "vertical_datum": dem.vertical_datum,
        "geoid_undulation_m": float(dem.geoid_undulation_m), "sigma_z_m": dem.sigma_z_m,
    }


def _hw_to_dict(hw: SensorHardware) -> dict[str, Any]:
    d = {"hw_type": type(hw).__name__}
    d.update({f.name: to_jsonable(getattr(hw, f.name)) for f in fields(hw)})
    return d


def _spec_to_dict(spec: PlatformSpec) -> dict[str, Any]:
    return {f.name: to_jsonable(getattr(spec, f.name)) for f in fields(spec)}


def _uav_to_dict(u: UAVInstance) -> dict[str, Any]:
    return {
        "uav_id": u.uav_id,
        "spec": _spec_to_dict(u.spec),
        "sensors": [_hw_to_dict(h) for h in u.sensors],
        "sortie": to_jsonable(u.sortie),
        "start_site_id": u.start_site_id,
    }


# ---------------------------------------------------------------------------
# load_mission：dict / JSON 字串 / 檔案 → MissionInputs（含 GeoJSON）
# ---------------------------------------------------------------------------

_TOP_KEYS = {"home", "alternates", "areas", "dem", "obstacles", "wind", "fleet", "payload_reqs", "c2", "airspace",
             "weather_thresholds", "weather_obs", "t0_epoch_s", "platform_overrides", "recovery_site_id"}
_META_KEYS = {"name", "description", "version", "meta", "metadata", "$schema", "notes", "comment", "type", "title"}
_GEOJSON_TYPES = {"Feature", "FeatureCollection", "Point", "LineString", "Polygon", "MultiPolygon", "MultiPoint",
                  "MultiLineString"}


def _kwargs(cls: type, obj: Mapping[str, Any], name: str, *, ignore_unknown: bool = False,
            drop: Sequence[str] = ()) -> dict[str, Any]:
    """依 dataclass 欄位名篩選 mapping；未知鍵 → MissionInputError（或忽略）。"""
    names = {f.name for f in fields(cls)}
    kw: dict[str, Any] = {}
    unknown: list[str] = []
    for k, v in obj.items():
        if k in drop:
            continue
        if k in names:
            kw[k] = v
        elif not ignore_unknown and not str(k).startswith("_"):
            unknown.append(str(k))
    if unknown:
        raise MissionInputError(f"{name}: 未知鍵 {sorted(unknown)}（允許：{sorted(names)}）")
    return kw


def _build(cls: type, obj: Mapping[str, Any], name: str, **extra: Any) -> Any:
    kw = _kwargs(cls, obj, name)
    kw.update(extra)
    try:
        return cls(**kw)
    except TypeError as e:  # 缺必填欄位
        raise MissionInputError(f"{name}: {e}") from e


# -- GeoJSON ------------------------------------------------------------------

def _is_geojson(obj: Any) -> bool:
    return isinstance(obj, Mapping) and obj.get("type") in _GEOJSON_TYPES and (
        "coordinates" in obj or "geometry" in obj or "features" in obj or "geometries" in obj)


def _lonlat_ring(ring: Sequence[Any], name: str) -> tuple[tuple[float, float], ...]:
    """GeoJSON 環 (lon, lat[, alt]) → (lat, lon)。"""
    pts = []
    for i, p in enumerate(ring):
        if not isinstance(p, Sequence) or isinstance(p, (str, bytes)) or len(p) < 2:
            raise MissionInputError(f"{name}[{i}]: GeoJSON 座標須為 [lon, lat]，得 {p!r}")
        pts.append((p[1], p[0]))
    return _ring(pts, name)


def _geojson_polygons(geom: Mapping[str, Any], name: str) -> list[tuple[tuple[tuple[float, float], ...],
                                                                       tuple[tuple[tuple[float, float], ...], ...]]]:
    """Polygon/MultiPolygon → [(外環, (內環, …)), …]（皆 (lat, lon)）。"""
    if geom.get("type") == "Feature":
        geom = geom.get("geometry") or {}
    t = geom.get("type")
    coords = geom.get("coordinates")
    if t == "Polygon":
        polys = [coords]
    elif t == "MultiPolygon":
        polys = list(coords or [])
    else:
        raise MissionInputError(f"{name}: GeoJSON 幾何須為 Polygon/MultiPolygon，得 {t!r}")
    out = []
    for i, rings in enumerate(polys):
        if not rings:
            raise MissionInputError(f"{name}: Polygon[{i}] 無座標環")
        ext = _lonlat_ring(rings[0], f"{name}.exterior[{i}]")
        holes = tuple(_lonlat_ring(r, f"{name}.hole[{i}][{j}]") for j, r in enumerate(rings[1:]))
        out.append((ext, holes))
    if not out:
        raise MissionInputError(f"{name}: MultiPolygon 為空")
    return out


def _geojson_obstacle_coords(geom: Mapping[str, Any], name: str) -> tuple[str, tuple[tuple[float, float], ...]]:
    if geom.get("type") == "Feature":
        geom = geom.get("geometry") or {}
    t = geom.get("type")
    c = geom.get("coordinates")
    if t == "Point":
        return "point", (_latlon_pair((c[1], c[0]), name),)
    if t == "LineString":
        pts = tuple(_latlon_pair((p[1], p[0]), f"{name}[{i}]") for i, p in enumerate(c))
        return "line", pts
    if t == "Polygon":
        return "polygon", _lonlat_ring(c[0], name)
    raise MissionInputError(f"{name}: 障礙 GeoJSON 幾何須為 Point/LineString/Polygon，得 {t!r}")


def _split_feature(it: Mapping[str, Any]) -> tuple[Mapping[str, Any] | None, dict[str, Any], bool]:
    """回傳 (geometry | None, attrs, from_geojson)。"""
    t = it.get("type")
    if t == "Feature":
        attrs = dict(it.get("properties") or {})
        if "id" in it and "area_id" not in attrs and "id" not in attrs:
            attrs["id"] = it["id"]
        return it.get("geometry"), attrs, True
    if t in ("Polygon", "MultiPolygon", "Point", "LineString") and "coordinates" in it:
        attrs = {k: v for k, v in it.items() if k not in ("type", "coordinates")}
        return it, attrs, True
    if "geometry" in it:
        attrs = {k: v for k, v in it.items() if k != "geometry"}
        g = it["geometry"]
        if isinstance(g, Mapping) and g.get("type") == "Feature":
            attrs.update(dict(g.get("properties") or {}))
            g = g.get("geometry")
        return g, attrs, True
    return None, dict(it), False


def _pop_id(attrs: dict[str, Any], key: str, fallback: str) -> str:
    for k in (key, "id", "name"):
        if k in attrs:
            v = attrs.pop(k)
            for kk in (key, "id", "name"):
                attrs.pop(kk, None)
            return str(v)
    return fallback


def _parse_areas(obj: Any) -> list[AreaSpec]:
    if isinstance(obj, Mapping):
        if obj.get("type") == "FeatureCollection":
            items = list(obj.get("features") or [])
        elif _is_geojson(obj) or "polygon_wgs84" in obj or "geometry" in obj:
            items = [obj]
        else:
            items = [dict(v, area_id=v.get("area_id", k)) if isinstance(v, Mapping) else v for k, v in obj.items()]
    elif isinstance(obj, (list, tuple)):
        items = list(obj)
    else:
        raise MissionInputError("areas 須為區域清單、GeoJSON FeatureCollection 或 {area_id: spec}")
    out: list[AreaSpec] = []
    for i, it in enumerate(items):
        out.extend(_parse_area(it, i))
    return out


def _parse_area(it: Any, i: int) -> list[AreaSpec]:
    if isinstance(it, AreaSpec):
        return [it]
    if not isinstance(it, Mapping):
        raise MissionInputError(f"areas[{i}] 須為 mapping 或 AreaSpec")
    geom, attrs, from_geojson = _split_feature(it)
    if geom is None:
        return [_build(AreaSpec, attrs, f"areas[{i}]")]
    area_id = _pop_id(attrs, "area_id", f"area_{i + 1}")
    attrs.pop("polygon_wgs84", None)
    attrs.pop("holes_wgs84", None)
    kw = _kwargs(AreaSpec, attrs, f"areas[{i}]", ignore_unknown=from_geojson, drop=("area_id",))
    parts = _geojson_polygons(geom, f"areas[{i}]({area_id})")
    if len(parts) == 1:
        ext, holes = parts[0]
        return [AreaSpec(area_id=area_id, polygon_wgs84=ext, holes_wgs84=holes, **kw)]
    return [AreaSpec(area_id=f"{area_id}_{j + 1}", polygon_wgs84=ext, holes_wgs84=holes, **kw)
            for j, (ext, holes) in enumerate(parts)]


def _parse_site(obj: Any, name: str, default_kind: str = "alternate", sites: Mapping[str, Site] | None = None) -> Site:
    if isinstance(obj, Site):
        return obj
    if isinstance(obj, str) and sites is not None:
        if obj not in sites:
            raise MissionInputError(f"{name}: 未知站點 {obj!r}")
        return sites[obj]
    if not isinstance(obj, Mapping):
        raise MissionInputError(f"{name} 須為 Site mapping")
    d = dict(obj)
    geom, attrs, from_geojson = _split_feature(d)
    if geom is not None:
        if geom.get("type") != "Point":
            raise MissionInputError(f"{name}: 站點 GeoJSON 幾何須為 Point")
        c = geom["coordinates"]
        attrs["lat_deg"], attrs["lon_deg"] = c[1], c[0]
        if len(c) >= 3 and "alt_msl_m" not in attrs and "alt_agl_m" not in attrs:
            attrs["alt_msl_m"] = c[2]
        attrs["site_id"] = _pop_id(attrs, "site_id", f"{name}")
        d = {k: v for k, v in attrs.items() if k in {f.name for f in fields(Site)}} if from_geojson else attrs
    d.setdefault("kind", default_kind)
    return _build(Site, d, name)


def _parse_dem(obj: Any) -> DemGrid:
    if isinstance(obj, DemGrid):
        return obj
    if not isinstance(obj, Mapping):
        raise MissionInputError("dem 須為 DemGrid 或 mapping")
    d = dict(obj)
    z = d.get("z_msl")
    if isinstance(z, str):
        p = Path(z)
        if not p.exists():
            raise MissionInputError(f"dem.z_msl 檔案不存在：{z}")
        z = np.load(p)
        if hasattr(z, "files"):                      # .npz → 取第一個陣列
            z = z[z.files[0]]
    elif isinstance(z, Mapping):
        shape = tuple(int(x) for x in z["shape"])
        z = np.asarray(z["data"], dtype=np.float32).reshape(shape)
    elif z is None:
        raise MissionInputError("dem.z_msl 缺失")
    d["z_msl"] = np.asarray(z, dtype=np.float32)
    kw = _kwargs(DemGrid, d, "dem")
    try:
        return DemGrid(**kw)
    except (TypeError, ValueError) as e:
        raise MissionInputError(f"dem: {e}") from e


def _parse_obstacle(it: Any, i: int) -> Obstacle:
    if isinstance(it, Obstacle):
        return it
    if not isinstance(it, Mapping):
        raise MissionInputError(f"obstacles[{i}] 須為 mapping 或 Obstacle")
    geom, attrs, from_geojson = _split_feature(it)
    if geom is None:
        d = dict(attrs)
        d.setdefault("top_msl_m", None)
        d.setdefault("height_agl_m", None)
        return _build(Obstacle, d, f"obstacles[{i}]")
    obs_id = _pop_id(attrs, "obs_id", f"obs_{i + 1}")
    gt, coords = _geojson_obstacle_coords(geom, f"obstacles[{i}]({obs_id})")
    attrs.pop("geom_type", None)
    attrs.pop("coords_wgs84", None)
    kw = _kwargs(Obstacle, attrs, f"obstacles[{i}]", ignore_unknown=from_geojson, drop=("obs_id",))
    kw.setdefault("top_msl_m", None)
    kw.setdefault("height_agl_m", None)
    return Obstacle(obs_id=obs_id, geom_type=gt, coords_wgs84=coords, **kw)


def _parse_wind(obj: Any) -> tuple[WindLayer, ...]:
    if obj is None:
        return ()
    if isinstance(obj, (WindLayer, Mapping)):
        obj = [obj]
    out = []
    for i, it in enumerate(obj):
        if isinstance(it, WindLayer):
            out.append(it)
        elif isinstance(it, Mapping):
            out.append(_build(WindLayer, it, f"wind[{i}]"))
        else:
            raise MissionInputError(f"wind[{i}] 須為 mapping 或 WindLayer")
    return tuple(out)


def _spec_lookup(key: str) -> PlatformSpec:
    k = "".join(ch for ch in str(key).upper() if ch.isalnum())
    if k in _SPEC_BY_NAME:
        return _SPEC_BY_NAME[k]
    raise MissionInputError(f"未知平台 {key!r}（可用：A/B/PIONEER/HERO/'Pioneer PNP'/'HERO PNP'）")


def _parse_spec(obj: Any, name: str) -> PlatformSpec:
    if isinstance(obj, PlatformSpec):
        return obj
    if isinstance(obj, str):
        return _spec_lookup(obj)
    if isinstance(obj, Mapping):
        d = dict(obj)
        if set(d) <= {"name", "cls"}:
            return _spec_lookup(d.get("cls") or d.get("name"))
        names = {f.name for f in fields(PlatformSpec)}
        bad = sorted(set(d) - names)
        if bad:
            raise MissionInputError(f"{name}.spec 未知鍵：{bad}")
        for k, v in list(d.items()):
            if k in ("provisional", "provisional_supplementary"):
                d[k] = frozenset(v or ())
            elif isinstance(v, list):
                d[k] = tuple(v)
        try:
            return PlatformSpec(**d)
        except (TypeError, ValueError) as e:
            raise MissionInputError(f"{name}.spec: {e}") from e
    raise MissionInputError(f"{name}.spec 須為 PlatformSpec、名稱字串或欄位 mapping")


def _parse_hw(obj: Any, name: str) -> SensorHardware:
    if isinstance(obj, (LidarHardware, CameraHardware, TrapezoidFovSpec)):
        return obj
    if isinstance(obj, str):
        k = obj.lower()
        if k == "lidar":
            return LidarHardware()
        if k in ("rgb", "camera"):
            return CameraHardware()
        raise MissionInputError(f"{name}: 未知感測器種類 {obj!r}")
    if not isinstance(obj, Mapping):
        raise MissionInputError(f"{name} 須為感測器 mapping／名稱")
    d = dict(obj)
    ht = d.pop("hw_type", None)
    if ht is not None:
        if ht not in _HW_TYPES:
            raise MissionInputError(f"{name}: 未知 hw_type {ht!r}")
        cls = _HW_TYPES[ht]
    elif any(k in d for k in ("mount_angle_deg", "v_fov_deg", "h_fov_deg")):
        cls = TrapezoidFovSpec
    else:
        kind = str(d.get("kind", "")).lower()
        if kind == "lidar":
            cls = LidarHardware
        elif kind in ("rgb", "camera"):
            cls = CameraHardware
            d["kind"] = "rgb"
        else:
            raise MissionInputError(f"{name}: 無法判定感測器硬體種類（給 kind='lidar'/'rgb' 或 hw_type）")
    kw = _kwargs(cls, d, name)
    try:
        return cls(**kw)
    except (TypeError, ValueError) as e:
        raise MissionInputError(f"{name}: {e}") from e


def _parse_uav(it: Any, i: int) -> UAVInstance:
    if isinstance(it, UAVInstance):
        return it
    if not isinstance(it, Mapping):
        raise MissionInputError(f"fleet[{i}] 須為 mapping 或 UAVInstance")
    d = dict(it)
    uav_id = d.pop("uav_id", None)
    if uav_id is None:
        raise MissionInputError(f"fleet[{i}]: 缺 uav_id")
    uav_id = str(uav_id)
    name = f"fleet[{uav_id}]"
    spec_obj = d.pop("spec", None)
    if spec_obj is None:
        spec_obj = d.pop("platform", None)
    cls_hint = d.pop("cls", None)
    if spec_obj is None:
        if cls_hint is None:
            raise MissionInputError(f"{name}: 須給 spec（PlatformSpec/名稱/欄位）或 cls（A/B）")
        spec = _spec_lookup(str(cls_hint))
    else:
        spec = _parse_spec(spec_obj, name)
        if cls_hint is not None and str(cls_hint) != spec.cls:
            raise MissionInputError(f"{name}: cls={cls_hint!r} 與 spec.cls={spec.cls!r} 不符")
    sensors_obj = d.pop("sensors", None)
    if sensors_obj is None:
        raise MissionInputError(f"{name}: 缺 sensors（主感測器在前）")
    if isinstance(sensors_obj, (str, Mapping, LidarHardware, CameraHardware, TrapezoidFovSpec)):
        sensors_obj = [sensors_obj]
    sensors = tuple(_parse_hw(h, f"{name}.sensors[{j}]") for j, h in enumerate(sensors_obj))
    sortie_obj = d.pop("sortie", None)
    if sortie_obj is None:
        raise MissionInputError(f"{name}: 缺 sortie（當架次 AUW／可用電量）")
    if isinstance(sortie_obj, SortieEnergy):
        sortie = sortie_obj
    elif isinstance(sortie_obj, Mapping):
        sd = dict(sortie_obj)
        sd.setdefault("uav_id", uav_id)
        sortie = _build(SortieEnergy, sd, f"{name}.sortie")
    else:
        raise MissionInputError(f"{name}.sortie 須為 mapping 或 SortieEnergy")
    start_obj = d.pop("start_site_id", None)
    if start_obj is None:
        start_obj = d.pop("start_site", None)
    if isinstance(start_obj, Site):
        start_obj = start_obj.site_id
    start_site_id = None if start_obj is None else _str_id(str(start_obj), f"{name}.start_site_id")
    unknown = [k for k in d if not str(k).startswith("_")]
    if unknown:
        raise MissionInputError(f"{name}: 未知鍵 {sorted(unknown)}")
    return UAVInstance(uav_id=uav_id, spec=spec, sensors=sensors, sortie=sortie, start_site_id=start_site_id)


def _parse_payload_reqs(obj: Any) -> dict[str, PayloadRequirement]:
    if obj is None:
        return {}
    out: dict[str, PayloadRequirement] = {}
    if isinstance(obj, Mapping):
        items = []
        for k, v in obj.items():
            if isinstance(v, PayloadRequirement):
                items.append(v)
            elif isinstance(v, Mapping):
                dd = dict(v)
                dd.setdefault("req_id", k)
                items.append(_build(PayloadRequirement, dd, f"payload_reqs[{k}]"))
            else:
                raise MissionInputError(f"payload_reqs[{k}] 須為 mapping 或 PayloadRequirement")
    elif isinstance(obj, (list, tuple)):
        items = [v if isinstance(v, PayloadRequirement) else _build(PayloadRequirement, v, f"payload_reqs[{i}]")
                 for i, v in enumerate(obj)]
    else:
        raise MissionInputError("payload_reqs 須為 {req_id: spec} 或清單")
    for r in items:
        if r.req_id in out:
            raise MissionInputError(f"payload_reqs req_id 重複：{r.req_id!r}")
        out[r.req_id] = r
    return out


def _parse_coverage(obj: Any, name: str) -> Any:
    if obj is None:
        return None
    if isinstance(obj, Mapping):
        if _is_geojson(obj):
            parts = _geojson_polygons(obj, name)
            if len(parts) != 1:
                raise MissionInputError(f"{name}: coverage 須為單一 Polygon")
            return parts[0][0]
        return {k: _parse_coverage(v, f"{name}[{k}]") for k, v in obj.items()}
    return _ring(obj, name)


def _parse_c2(obj: Any, home: Site, sites: Mapping[str, Site]) -> C2Link:
    if isinstance(obj, C2Link):
        return obj
    if obj is None:
        return C2Link(range_km=None, coverage_polygon_wgs84=None, ground_station=home, lost_link_policy={})
    if not isinstance(obj, Mapping):
        raise MissionInputError("c2 須為 mapping 或 C2Link")
    d = dict(obj)
    gs = d.pop("ground_station", None)
    ground = home if gs is None else _parse_site(gs, "c2.ground_station", default_kind="home", sites=sites)
    cov = _parse_coverage(d.pop("coverage_polygon_wgs84", None), "c2.coverage_polygon_wgs84")
    d.setdefault("range_km", None)
    d.setdefault("lost_link_policy", {})
    return _build(C2Link, d, "c2", ground_station=ground, coverage_polygon_wgs84=cov)


def _parse_nofly(obj: Any) -> tuple[NoFlyZone, ...]:
    if obj is None:
        return ()
    if isinstance(obj, Mapping):
        if obj.get("type") == "FeatureCollection":
            items = list(obj.get("features") or [])
        elif _is_geojson(obj) or "polygon_wgs84" in obj or "geometry" in obj:
            items = [obj]
        else:
            items = [dict(v, zone_id=v.get("zone_id", k)) if isinstance(v, Mapping) else v for k, v in obj.items()]
    else:
        items = list(obj)
    out: list[NoFlyZone] = []
    for i, it in enumerate(items):
        if isinstance(it, NoFlyZone):
            out.append(it)
            continue
        if not isinstance(it, Mapping):
            raise MissionInputError(f"airspace.no_fly[{i}] 須為 mapping 或 NoFlyZone")
        geom, attrs, from_geojson = _split_feature(it)
        if geom is None:
            out.append(_build(NoFlyZone, attrs, f"airspace.no_fly[{i}]"))
            continue
        zone_id = _pop_id(attrs, "zone_id", f"nofly_{i + 1}")
        attrs.pop("polygon_wgs84", None)
        kw = _kwargs(NoFlyZone, attrs, f"airspace.no_fly[{i}]", ignore_unknown=from_geojson, drop=("zone_id",))
        parts = _geojson_polygons(geom, f"airspace.no_fly[{i}]({zone_id})")
        if len(parts) == 1:
            out.append(NoFlyZone(zone_id=zone_id, polygon_wgs84=parts[0][0], **kw))
        else:
            out.extend(NoFlyZone(zone_id=f"{zone_id}_{j + 1}", polygon_wgs84=ext, **kw) for j, (ext, _h) in enumerate(parts))
    return tuple(out)


def _parse_airspace(obj: Any) -> Airspace:
    if isinstance(obj, Airspace):
        return obj
    if obj is None:
        return Airspace()
    if not isinstance(obj, Mapping):
        raise MissionInputError("airspace 須為 mapping 或 Airspace")
    d = dict(obj)
    nf = _parse_nofly(d.pop("no_fly", None))
    return _build(Airspace, d, "airspace", no_fly=nf)


def _load_obj(obj: Any) -> Mapping[str, Any]:
    if isinstance(obj, Mapping):
        return obj
    if isinstance(obj, (bytes, bytearray)):
        obj = obj.decode("utf-8")
    if isinstance(obj, os.PathLike):
        obj = os.fspath(obj)
    if isinstance(obj, str):
        s = obj.strip()
        if s.startswith("{"):
            try:
                data = json.loads(s)
            except json.JSONDecodeError as e:
                raise MissionInputError(f"JSON 解析失敗：{e}") from e
        else:
            p = Path(obj)
            if not p.exists():
                raise MissionInputError(f"任務檔案不存在（亦非 JSON 物件字串）：{obj!r}")
            try:
                with open(p, "r", encoding="utf-8") as fh:
                    data = json.load(fh)
            except json.JSONDecodeError as e:
                raise MissionInputError(f"任務檔案 {p} JSON 解析失敗：{e}") from e
        if not isinstance(data, Mapping):
            raise MissionInputError("任務 JSON 頂層須為物件")
        return data
    raise MissionInputError(f"load_mission 接受 dict、JSON 字串或檔案路徑，得 {type(obj).__name__}")


def load_mission(obj: dict | str | os.PathLike) -> MissionInputs:
    """JSON／dict／檔案路徑 → :class:`MissionInputs`（度數、Wh 皆原值；規格 §6.1）。

    * `areas`：區域清單（`polygon_wgs84` 為 (lat, lon)）、或 GeoJSON FeatureCollection／Feature／Polygon／MultiPolygon：
      自動交換 (lon, lat) → (lat, lon)、外環 → `polygon_wgs84`、內環 → `holes_wgs84`、MultiPolygon 拆成多個 AreaSpec
      （`area_id` 加尾碼 `_1, _2, …`）；Feature 之 `properties` 供 area_id/sensor_req/priority/time_window_s/payload_req_id。
    * `obstacles`／`airspace.no_fly`／`c2.coverage_polygon_wgs84`／站點亦接受 GeoJSON 幾何。
    * `fleet[i]`：`{uav_id, spec | cls, sensors, sortie, start_site_id?}`（`start_site_id` 選填：該機起飛站點，
      須為 home／alternates 之一；省略或 null → home）；`spec` 可為 "A"/"B"/"PIONEER"/"HERO"、`PlatformSpec`，或
      `MissionInputs.to_dict()` 輸出之完整欄位 mapping；`sensors` 元素為 "lidar"/"rgb" 或硬體欄位 mapping（`hw_type` 可選）。
    * 缺 `c2` → 無鏈路範圍之 C2Link（preflight NO_GO，D11）；缺 `airspace` → `Airspace()`；缺 `weather_thresholds` → 預設。
    * 已建構之 dataclass 實例可直接放在 dict 中（原樣採用）。
    """
    data = _load_obj(obj)
    unknown = sorted(k for k in data if k not in _TOP_KEYS and k not in _META_KEYS and not str(k).startswith("_"))
    if unknown:
        raise MissionInputError(f"任務輸入含未知頂層鍵：{unknown}（允許：{sorted(_TOP_KEYS)}）")
    for req in ("home", "areas", "dem", "fleet", "weather_obs"):
        if req not in data or data[req] is None:
            raise MissionInputError(f"任務輸入缺必填鍵：{req!r}")

    home = _parse_site(data["home"], "home", default_kind="home")
    alt_obj = data.get("alternates") or ()
    if isinstance(alt_obj, (Site, Mapping)):
        alt_obj = [alt_obj]
    alternates = tuple(_parse_site(s, f"alternates[{i}]", default_kind="alternate") for i, s in enumerate(alt_obj))
    sites = {home.site_id: home, **{s.site_id: s for s in alternates}}
    areas = tuple(_parse_areas(data["areas"]))
    dem = _parse_dem(data["dem"])
    obs_obj = data.get("obstacles") or ()
    if isinstance(obs_obj, Mapping) and obs_obj.get("type") == "FeatureCollection":
        obs_obj = list(obs_obj.get("features") or [])
    elif isinstance(obs_obj, (Obstacle, Mapping)):
        obs_obj = [obs_obj]
    obstacles = tuple(_parse_obstacle(o, i) for i, o in enumerate(obs_obj))
    wind = _parse_wind(data.get("wind"))
    fleet_obj = data.get("fleet")
    if isinstance(fleet_obj, Mapping):
        fleet_obj = [dict(v, uav_id=v.get("uav_id", k)) if isinstance(v, Mapping) else v for k, v in fleet_obj.items()]
    fleet = tuple(_parse_uav(u, i) for i, u in enumerate(list(fleet_obj)))
    payload_reqs = _parse_payload_reqs(data.get("payload_reqs"))
    c2 = _parse_c2(data.get("c2"), home, sites)
    airspace = _parse_airspace(data.get("airspace"))
    th_obj = data.get("weather_thresholds")
    thresholds = th_obj if isinstance(th_obj, WeatherThresholds) else (
        WeatherThresholds() if th_obj is None else _build(WeatherThresholds, th_obj, "weather_thresholds"))
    obs_w = data["weather_obs"]
    weather_obs = obs_w if isinstance(obs_w, WeatherObs) else _build(WeatherObs, obs_w, "weather_obs")
    ov = data.get("platform_overrides") or {}
    return MissionInputs(
        home=home, alternates=alternates, areas=areas, dem=dem, obstacles=obstacles, wind=wind, fleet=fleet,
        payload_reqs=payload_reqs, c2=c2, airspace=airspace, weather_thresholds=thresholds, weather_obs=weather_obs,
        t0_epoch_s=float(data.get("t0_epoch_s", 0.0) or 0.0),
        platform_overrides={str(k): dict(v) for k, v in ov.items()} if isinstance(ov, Mapping) else ov,
        recovery_site_id=data.get("recovery_site_id"),
    )
