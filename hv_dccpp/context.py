"""context.py — 共用情境物件（契約 §6A；第 2 層：只依賴 geo / platform / mission / wind）。

規格對應：§6.1（表 2 → 內部表示：MSL/AGL → ENU z，D42）、§3.5（`platform_overrides` 套用規則：只允許
provisional ∪ provisional_supplementary ∪ {min_fw_operation_agl_m}，後者只可調高；覆寫後自 provisional 集合移除
並記錄 `overridden(key, old→new)`）、§4.6／D12（h_ceil = waiver_max_agl_m if given else max_agl_m；皆 None →
120 m AGL 並警告 `airspace_ceiling_defaulted_120`）、§5.1–§5.7／D39（EnergyModel 逐機建立：E_nom / E_avail /
E_ref 解析、`reference_battery_assumed`、`battery_nominal_example`、`battery_temp_assumed`、基準分鐘模式）、
§6.2 步 1/3/4（h_clr 解析 Airspace → spec 覆寫槽 → MissionInputError；h_clr ≥ 3σ_z + 30 強制，D11；障礙燒入 D24）、
D41（NoFly ⊕ b_trk）、D22（禁區時間窗：與任務窗重疊即有效）、D43（`sensor_model(hw, req)` 逐 (cls, req_id)）、
§3.1（P_link,c 逐構型：`C2Link.range_km` / coverage polygon → spec 覆寫槽；ρ^sym 取下緣）。

全域約定（契約 §0）
------------------
* 本地 ENU（m），原點 = 主起飛點 ``home``；所有高度欄位一律 ENU z（D42），MSL 只出現於輸入型別。
* 站點 ENU 座標以 ``GeoFrame.to_enu(lat, lon, msl)`` 之完整三維轉換取得（含曲率）；多邊形頂點以 DEM 地面 MSL
  投影（比契約註「高程取 0」更準；DEM 外頂點退回 home 之 MSL）。
* 所有輸入錯誤丟 :class:`MissionInputError`（與 platform/mission 為同一類別）。

依賴 `routing.KeepoutSet`（同層、平行開發）採**延遲 import**：不存在時 ``keepouts=None`` 並記錄警告
``routing_unavailable_keepouts_none``；障礙燒入（D24）於 `KeepoutSet.build` 未燒入時由本模組補做一次。
"""
from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass, field
from typing import Any, NamedTuple, Sequence

import numpy as np
import shapely
from shapely.geometry import LineString, MultiPoint, MultiPolygon, Point, Polygon

from .geo import DemGrid, GeoFrame, TerrainGrid
from .mission import (
    Airspace,
    AreaSpec,
    C2Link,
    MappingModeConfig,
    MissionInputError,
    MissionInputs,
    NoFlyZone,
    Obstacle,
    PlannerConfig,
    Site,
    UAVInstance,
    allowed_override_keys,
)
from .platform import (
    EnergyModel,
    PlatformSpec,
    PlatformSpecError,
    Sensor,
    SensorHardware,
    combine_sensors,
    crabbed_width_m,
    sensor_model,
)
from .wind import WindField

__all__ = [
    "TERRAIN_MARGIN_M",
    "DEFAULT_CEILING_AGL_M",
    "UNBOUNDED_LINK_RADIUS_M",
    "MIN_POLYGON_AREA_M2",
    "ObstacleEnu",
    "DecomposeContext",
    "MissionContext",
    "build_context",
    "apply_platform_overrides",
    "base_specs_from_fleet",
    "resolve_terrain_clearance",
    "resolve_ceiling",
    "site_ground_msl",
    "polygon_enu",
    "obstacles_to_enu",
    "burn_obstacles",
    "build_energy_model",
    "build_sensors",
    "apply_crab_margin",
    "link_polygon",
]

TERRAIN_MARGIN_M: float = 3000.0
"""TerrainGrid 邊界 = 所有輸入幾何之外接矩形 ⊕ 此邊距（須 ≥ 走廊半寬 + d_turn，避免 corridor_* 於邊緣回 nan）。"""

DEFAULT_CEILING_AGL_M: float = 120.0
"""`Airspace.max_agl_m` 與 `max_msl_m` 皆 None 時之預設高度上限（規格 §6.1；警告 airspace_ceiling_defaulted_120）。"""

UNBOUNDED_LINK_RADIUS_M: float = 1.0e7
"""`cfg.enforce_link=False`（legacy）時之「無限」鏈路圓半徑。"""

MIN_POLYGON_AREA_M2: float = 1.0
"""輸入多邊形（區域、禁區、coverage、面障礙）之最小有效面積；小於此值視為退化（MissionInputError）。"""

_RGB_REQ_SUFFIX: str = "_rgb"
"""``both``／``any`` 區之 rgb 需求鍵約定（= req_id + 此尾碼；與 `decompose.RGB_REQ_SUFFIX` 同值）。"""

_LINK_DISC_QUAD_SEGS: int = 64
_BURN_SKIP_ON_DSM: frozenset[str] = frozenset({"tree", "building"})     # D24


# ---------------------------------------------------------------------------
# 資料型別
# ---------------------------------------------------------------------------

class ObstacleEnu(NamedTuple):
    """障礙物之 ENU 表示（未加水平緩衝；`top_z_m` 為 ENU z 頂高，**不含** b_v）。

    `altitude.site_check(..., obstacles_enu: list[tuple[Geometry, float]])` 可用
    :meth:`MissionContext.obstacle_pairs` 取得 ``(geom, top_z_m)`` 對。
    """
    geom: shapely.Geometry
    top_z_m: float
    obs_id: str
    category: str
    buffer_m: float                      # 航路 keep-out 水平緩衝（Obstacle.keepout_buffer_m → cfg.b_h_m）


@dataclass(slots=True)
class DecomposeContext:
    """分解／方向評分／條帶所需之最小情境（契約 §6A；避免 god-object 跨模組）。"""
    frame: GeoFrame
    terrain: TerrainGrid
    wind: WindField
    airspace: Airspace
    specs: dict[str, PlatformSpec]                     # cls → spec（已套用 platform_overrides）
    sensors: dict[tuple[str, str], Sensor]             # (cls, req_id) → Sensor
    available_kinds: dict[str, frozenset[str]]         # cls → 機隊該構型可用感測種類聯集
    auw_kg: dict[str, float]                           # cls → 代表 AUW（機隊最大）
    models_rep: dict[str, EnergyModel]                 # cls → 代表能量模型（最大 AUW、最小 E_avail）
    nofly_buffered: list[shapely.Polygon]              # NoFly（有效窗）⊕ b_trk（D41）
    home_xy: np.ndarray                                # (2,)
    h_clr_m: float
    h_ceil_m: dict[str, float | None]
    cfg: PlannerConfig
    mapping: MappingModeConfig | None = None           # 建圖模式（= inputs.mapping_mode；None ＝ 舊行為）


@dataclass(slots=True)
class MissionContext:
    """planner / altitude / costs / deconflict / sim 共用情境；由 :func:`build_context` 組裝（契約 §6A）。

    契約欄位之後的尾端欄位（皆有預設值）為本模組額外提供（記錄於 CONTRACT_DEVIATIONS）：
    ``nofly_buffered``、``available_kinds``、``auw_kg``（供 :meth:`decompose_ctx` 免重算）、
    ``obstacles_enu``（site_check 用）、``ground_station_xy``、``t_window``（D22 任務窗）、
    ``start_sites``（uav_id → 起飛站點 site_id；`UAVInstance.start_site_id` 未設定者為 home）。
    """
    inputs: MissionInputs
    cfg: PlannerConfig
    frame: GeoFrame
    terrain: TerrainGrid
    wind: WindField
    keepouts: Any                                      # routing.KeepoutSet；routing 不可用時 None
    specs: dict[str, PlatformSpec]
    sensors: dict[tuple[str, str], Sensor]
    models: dict[str, EnergyModel]                     # uav_id → 每機模型
    models_rep: dict[str, EnergyModel]                 # cls → 代表模型
    sites_enu: dict[str, tuple[np.ndarray, float]]     # site_id → (xy (2,), z_site ENU)
    link_poly: dict[str, shapely.Polygon]              # cls → P_link,c（ENU）
    rho_sym_m: dict[str, float]                        # cls → ρ^sym 下緣（m）；enforce_link=False → inf
    h_clr_m: float
    h_ceil_m: dict[str, float | None]                  # cls → h_ceil（AGL）
    h_ceil_source: str
    areas_enu: dict[str, shapely.Polygon]              # area_id → 有效多邊形（含洞；ENU）
    recovery_site_id: str
    warnings: list[str]
    # --- 額外（契約未列；預設值）---
    nofly_buffered: list[shapely.Polygon] = field(default_factory=list)
    available_kinds: dict[str, frozenset[str]] = field(default_factory=dict)
    auw_kg: dict[str, float] = field(default_factory=dict)
    obstacles_enu: list[ObstacleEnu] = field(default_factory=list)
    ground_station_xy: np.ndarray = field(default_factory=lambda: np.zeros(2))
    t_window: tuple[float, float] | None = None
    start_sites: dict[str, str] = field(default_factory=dict)   # uav_id → 起飛站點 site_id（預設 home）
    recovery_sites: dict[str, str] = field(default_factory=dict)  # uav_id → 回收站 site_id（預設全域 recovery；vendored #4）

    # -- 便利查詢 -------------------------------------------------------------
    @property
    def home_xy(self) -> np.ndarray:
        return self.sites_enu[self.inputs.home.site_id][0]

    @property
    def z_ground_home_m(self) -> float:
        return self.sites_enu[self.inputs.home.site_id][1]

    def start_site_id(self, uav_id: str) -> str:
        """該機**起飛**站點 id（`UAVInstance.start_site_id` → home）。回收／降落仍走 `recovery_site_id`（D40）。"""
        sid = self.start_sites.get(uav_id)
        if sid is None:
            sid = self.inputs.start_site_id_of(uav_id)
        return sid

    def recovery_site_id_of(self, uav_id: str) -> str:
        """該機**回收／降落**站點 id（缺鍵 → 全域 `recovery_site_id`，即 D40 既有行為）。"""
        return self.recovery_sites.get(uav_id) or self.recovery_site_id

    def start_xy(self, uav_id: str) -> np.ndarray:
        """該機起飛站點之 ENU xy ``(2,)``。"""
        return self.sites_enu[self.start_site_id(uav_id)][0]

    def z_ground_start_m(self, uav_id: str) -> float:
        """該機起飛站點之 ENU 地面 z。"""
        return self.sites_enu[self.start_site_id(uav_id)][1]

    def uav(self, uav_id: str) -> UAVInstance:
        return self.inputs.uav_by_id(uav_id)

    def spec_of(self, uav_id: str) -> PlatformSpec:
        """該機構型之（已覆寫）spec。"""
        return self.specs[self.uav(uav_id).cls]

    def obstacle_pairs(self) -> list[tuple[shapely.Geometry, float]]:
        """``[(geom_enu, top_z_m)]``（`altitude.site_check` 之 `obstacles_enu` 參數形式）。"""
        return [(o.geom, o.top_z_m) for o in self.obstacles_enu]

    def decompose_ctx(self) -> DecomposeContext:
        """抽出分解階段所需子集（契約 §6A）。"""
        return DecomposeContext(
            frame=self.frame, terrain=self.terrain, wind=self.wind, airspace=self.inputs.airspace,
            specs=self.specs, sensors=self.sensors, available_kinds=self.available_kinds,
            auw_kg=self.auw_kg, models_rep=self.models_rep, nofly_buffered=self.nofly_buffered,
            home_xy=self.home_xy, h_clr_m=self.h_clr_m, h_ceil_m=self.h_ceil_m, cfg=self.cfg,
            mapping=self.inputs.mapping_mode,
        )


# ---------------------------------------------------------------------------
# 覆寫（規格 §3.5）
# ---------------------------------------------------------------------------

def base_specs_from_fleet(fleet: Sequence[UAVInstance]) -> tuple[dict[str, PlatformSpec], list[str]]:
    """cls → 機隊該構型之基礎 spec（同構型取第一架；其餘不同者警告 ``spec_heterogeneous(cls)``）。"""
    specs: dict[str, PlatformSpec] = {}
    warnings: list[str] = []
    seen_differ: set[str] = set()
    for u in fleet:
        c = u.cls
        if c not in specs:
            specs[c] = u.spec
        elif u.spec != specs[c] and c not in seen_differ:
            seen_differ.add(c)
            warnings.append(f"spec_heterogeneous({c}): 機隊同構型 spec 不一致，採第一架（{[x.uav_id for x in fleet if x.cls == c][0]}）")
    return specs, warnings


def _normalise_override_value(key: str, old: Any, value: Any) -> Any:
    if value is None:
        return None
    if isinstance(value, (list, tuple)):
        return tuple(float(x) for x in value)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value
    v = float(value)
    if isinstance(old, tuple):
        raise MissionInputError(f"platform_overrides[{key}] 須為 (lo, hi) 區間，得 {value!r}")
    return v


def apply_platform_overrides(base_specs: dict[str, PlatformSpec],
                             overrides: dict[str, dict[str, Any]] | None) -> tuple[dict[str, PlatformSpec], list[str]]:
    """套用 `MissionInputs.platform_overrides`（規格 §3.5）。

    * 只允許 ``provisional ∪ provisional_supplementary ∪ {min_fw_operation_agl_m}``，其餘鍵丟 MissionInputError。
    * ``min_fw_operation_agl_m`` 只可調高（小於現值丟 MissionInputError）。
    * 以 :func:`dataclasses.replace` 套用；`PlatformSpec.__post_init__` 自洽檢查失敗（PlatformSpecError）改丟 MissionInputError。
    * 覆寫後自 ``provisional`` / ``provisional_supplementary`` 移除該鍵，並記錄 ``overridden(cls.key, old→new)``。
    * 機隊中不存在之構型之覆寫忽略並警告 ``override_ignored_no_fleet(cls)``。
    """
    out: dict[str, PlatformSpec] = {}
    warnings: list[str] = []
    ov = overrides or {}
    for cls in ov:
        if cls not in base_specs:
            warnings.append(f"override_ignored_no_fleet({cls})")
    for cls, spec in base_specs.items():
        kv = ov.get(cls) or {}
        if not kv:
            out[cls] = spec
            continue
        allowed = allowed_override_keys(spec)
        bad = sorted(set(kv) - allowed)
        if bad:
            raise MissionInputError(
                f"platform_overrides[{cls!r}] 含不允許覆寫之鍵：{bad}（只允許 provisional ∪ provisional_supplementary ∪ "
                f"{{min_fw_operation_agl_m}}）")
        changes: dict[str, Any] = {}
        for k, raw in kv.items():
            old = getattr(spec, k)
            v = _normalise_override_value(f"{cls}.{k}", old, raw)
            if k == "min_fw_operation_agl_m":
                if v is None or float(v) < float(old) - 1e-9:
                    raise MissionInputError(
                        f"platform_overrides[{cls!r}].min_fw_operation_agl_m={raw!r} 只允許調高（現值 {old}）")
            changes[k] = v
            warnings.append(f"overridden({cls}.{k}, {old!r}→{v!r})")
        new_prov = frozenset(spec.provisional) - set(changes)
        new_supp = frozenset(spec.provisional_supplementary) - set(changes)
        try:
            out[cls] = dataclasses.replace(spec, provisional=new_prov, provisional_supplementary=new_supp, **changes)
        except PlatformSpecError as e:
            raise MissionInputError(f"platform_overrides[{cls!r}] 使 PlatformSpec 自洽檢查失敗：{e}") from e
    return out, warnings


# ---------------------------------------------------------------------------
# 座標轉換小工具（D42）
# ---------------------------------------------------------------------------

def site_ground_msl(site: Site, dem: DemGrid) -> float:
    """站點高程 MSL：`alt_msl_m` → 原值；`alt_agl_m` → DEM MSL + AGL；皆 None → DEM MSL（規格 §6.1）。"""
    if site.alt_msl_m is not None:
        return float(site.alt_msl_m)
    msl = float(dem.sample_msl(site.lat_deg, site.lon_deg)[0])
    if not math.isfinite(msl):
        raise MissionInputError(f"Site[{site.site_id}] 位於 DEM 範圍外（無法由 DEM 取地面高）")
    return msl + (float(site.alt_agl_m) if site.alt_agl_m is not None else 0.0)


def _latlon_to_xy(frame: GeoFrame, dem: DemGrid, latlon: Sequence[tuple[float, float]]) -> np.ndarray:
    """(lat, lon) 序列 → ENU xy ``(N, 2)``；投影高度取 DEM 地面 MSL（DEM 外 → home 之 MSL）。"""
    arr = np.asarray(latlon, dtype=np.float64).reshape(-1, 2)
    msl = dem.sample_msl(arr[:, 0], arr[:, 1])
    h = np.where(np.isfinite(msl), msl, frame.h0_msl_m)
    return frame.to_enu(arr[:, 0], arr[:, 1], h)[:, :2]


def polygon_enu(frame: GeoFrame, dem: DemGrid, ring: Sequence[tuple[float, float]],
                holes: Sequence[Sequence[tuple[float, float]]] = (), name: str = "polygon") -> shapely.Geometry:
    """WGS84 環（含洞）→ ENU 多邊形（`make_valid`；面積須 > 0）。結果可能為 MultiPolygon（自交輸入）。"""
    shell = _latlon_to_xy(frame, dem, ring)
    hole_xy = [_latlon_to_xy(frame, dem, h) for h in holes]
    poly = shapely.make_valid(Polygon(shell, hole_xy))
    parts = [g for g in shapely.get_parts(poly) if isinstance(g, Polygon) and g.area >= MIN_POLYGON_AREA_M2]
    if not parts:
        raise MissionInputError(f"{name}: 多邊形無效或面積 < {MIN_POLYGON_AREA_M2} m²")
    geom = parts[0] if len(parts) == 1 else shapely.unary_union(parts)
    if geom.is_empty or geom.area < MIN_POLYGON_AREA_M2:
        raise MissionInputError(f"{name}: 多邊形無效或面積 < {MIN_POLYGON_AREA_M2} m²")
    return geom


def _bounds_with_margin(points: list[np.ndarray], margin_m: float) -> tuple[float, float, float, float]:
    pts = np.vstack([np.asarray(p, dtype=np.float64).reshape(-1, 2) for p in points if np.size(p)])
    return (float(pts[:, 0].min() - margin_m), float(pts[:, 1].min() - margin_m),
            float(pts[:, 0].max() + margin_m), float(pts[:, 1].max() + margin_m))


# ---------------------------------------------------------------------------
# 淨空與高度上限
# ---------------------------------------------------------------------------

def resolve_terrain_clearance(airspace: Airspace, specs: dict[str, PlatformSpec],
                              sigma_z_m: float | None) -> tuple[float, list[str]]:
    """h_clr 解析（D11；規格 §6.2 步 1/4）：`Airspace.terrain_clearance_m` → spec 覆寫槽（多構型取最大）→ MissionInputError。

    `sigma_z_m` 已知時強制 ``h_clr ≥ 3σ + 30``（不足則抬高並警告 ``terrain_clearance_raised_to_3sigma_plus_30``）；
    未知時以輸入值並警告 ``terrain_clearance_sigma_unknown``。
    """
    warnings: list[str] = []
    if airspace.terrain_clearance_m is not None:
        h = float(airspace.terrain_clearance_m)
    else:
        vals = {c: float(s.terrain_clearance_m) for c, s in specs.items() if s.terrain_clearance_m is not None}
        if not vals:
            raise MissionInputError(
                "terrain_clearance_m 未定義：Airspace.terrain_clearance_m 與 spec.terrain_clearance_m（platform_overrides）皆 None；"
                "缺鍵：terrain_clearance_m")
        h = max(vals.values())
        if len(set(vals.values())) > 1:
            warnings.append(f"terrain_clearance_per_class_differs({vals}) → 採最大 {h}")
    if sigma_z_m is not None and math.isfinite(sigma_z_m):
        floor = 3.0 * float(sigma_z_m) + 30.0
        if h < floor - 1e-9:
            warnings.append(f"terrain_clearance_raised_to_3sigma_plus_30({h:g}→{floor:g})")
            h = floor
    else:
        warnings.append("terrain_clearance_sigma_unknown")
    return h, warnings


def _max_terrain_msl(frame: GeoFrame, terrain: TerrainGrid, polys: Sequence[shapely.Geometry]) -> float:
    """多邊形集合內之最高地形 **MSL**（節點 ∪ 邊界取樣）；無有效取樣 → nan。以燒入前之 z 呼叫。"""
    best = -math.inf
    found = False
    xs, ys = terrain.xs, terrain.ys
    for poly in polys:
        minx, miny, maxx, maxy = poly.bounds
        ix = (xs >= minx) & (xs <= maxx)
        iy = (ys >= miny) & (ys <= maxy)
        pts_list: list[np.ndarray] = []
        if ix.any() and iy.any():
            gx, gy = np.meshgrid(xs[ix], ys[iy])
            pts = np.column_stack([gx.ravel(), gy.ravel()])
            inside = shapely.contains_xy(poly, pts[:, 0], pts[:, 1])
            if inside.any():
                pts_list.append(pts[inside])
        boundary = shapely.segmentize(poly.boundary, terrain.step_m / 2.0)
        pts_list.append(shapely.get_coordinates(boundary))
        pts = np.vstack(pts_list)
        z = terrain.z_at(pts)
        ok = np.isfinite(z)
        if ok.any():
            msl = frame.enu_z_to_msl(pts[ok], z[ok])
            best = max(best, float(msl.max()))
            found = True
    return best if found else math.nan


def resolve_ceiling(airspace: Airspace, cfg: PlannerConfig, specs: dict[str, PlatformSpec],
                    max_terrain_msl_m: float | None) -> tuple[dict[str, float | None], str, list[str]]:
    """h_ceil,c（AGL；規格 §4.6、D12）與來源字串。

    優先序：``cfg.h_ceil_override_m``（legacy → inf）→ ``waiver_max_agl_m`` → ``max_agl_m``（若同時給 ``max_msl_m``
    則取 ``min(max_agl, max_msl − max 地形 MSL)``）→ ``max_msl_m − max 地形 MSL``（區域內最高地形之上，保守）→
    120 m AGL 預設並警告 ``airspace_ceiling_defaulted_120``。兩構型受同一空域限制，值相同。
    """
    warnings: list[str] = []
    if cfg.h_ceil_override_m is not None:
        val: float | None = float(cfg.h_ceil_override_m)
        source = "cfg.h_ceil_override_m"
    elif airspace.waiver_max_agl_m is not None:
        val = float(airspace.waiver_max_agl_m)
        source = "waiver_max_agl_m"
    elif airspace.max_agl_m is not None:
        val = float(airspace.max_agl_m)
        source = "max_agl_m"
        if airspace.max_msl_m is not None:
            if max_terrain_msl_m is None or not math.isfinite(max_terrain_msl_m):
                raise MissionInputError("Airspace.max_msl_m 給定但區域內無有效 DEM 地形，無法換算 AGL 上限")
            msl_agl = float(airspace.max_msl_m) - float(max_terrain_msl_m)
            if msl_agl < val:
                val = msl_agl
                source = "max_msl_m(min with max_agl_m)"
    elif airspace.max_msl_m is not None:
        if max_terrain_msl_m is None or not math.isfinite(max_terrain_msl_m):
            raise MissionInputError("Airspace.max_msl_m 給定但區域內無有效 DEM 地形，無法換算 AGL 上限")
        val = float(airspace.max_msl_m) - float(max_terrain_msl_m)
        source = "max_msl_m"
    else:
        val = DEFAULT_CEILING_AGL_M
        source = "default_120"
        warnings.append("airspace_ceiling_defaulted_120")
    # spec.h_ceil 只是 waiver/max_agl 的選擇規則；此處以同一規則對每構型給值
    h_ceil = {c: (spec.h_ceil(val, None) if val is not None else None) for c, spec in specs.items()}
    return h_ceil, source, warnings


# ---------------------------------------------------------------------------
# 障礙物（D24；keep-out 幾何交由 routing）
# ---------------------------------------------------------------------------

def _ground_z_max(terrain: TerrainGrid, geom: shapely.Geometry) -> float:
    """幾何覆蓋範圍內之最高地形 ENU z（線段以 step/2 取樣；面以 max_in_polygon）；範圍外 → nan。"""
    if isinstance(geom, (Polygon, MultiPolygon)):
        return float(terrain.max_in_polygon(geom))
    if isinstance(geom, LineString):
        coords = shapely.get_coordinates(shapely.segmentize(geom, terrain.step_m / 2.0))
    else:
        coords = shapely.get_coordinates(geom)
    z = terrain.z_at(coords)
    return float(np.max(z)) if z.size else math.nan


def obstacles_to_enu(obstacles: Sequence[Obstacle], frame: GeoFrame, dem: DemGrid, terrain: TerrainGrid,
                     cfg: PlannerConfig) -> tuple[list[ObstacleEnu], list[str]]:
    """障礙物 → ENU 幾何（未緩衝）與 ENU 頂高（規格 §6.1、§8.5；D42）。

    * 點 → Point / MultiPoint、線 → LineString、面 → Polygon（make_valid）。
    * `top_msl_m` → 各頂點 ``msl_to_enu_z`` 取最大；`height_agl_m` → 幾何範圍內最高地形 ENU z + h
      （地形以**燒入前**之 TerrainGrid 取；範圍外退回頂點 DEM 值，仍無 → 略過並警告 ``obstacle_outside_dem``）。
    * `buffer_m` = `Obstacle.keepout_buffer_m` → `cfg.b_h_m`。
    """
    out: list[ObstacleEnu] = []
    warnings: list[str] = []
    for o in obstacles:
        xy = _latlon_to_xy(frame, dem, o.coords_wgs84)
        if o.geom_type == "point":
            geom: shapely.Geometry = Point(xy[0]) if xy.shape[0] == 1 else MultiPoint(xy)
        elif o.geom_type == "line":
            geom = LineString(xy)
        else:
            geom = shapely.make_valid(Polygon(xy))
            parts = [g for g in shapely.get_parts(geom) if isinstance(g, Polygon) and g.area > 0.0]
            if not parts:
                raise MissionInputError(f"Obstacle[{o.obs_id}]: 多邊形無效或面積為 0")
            geom = parts[0] if len(parts) == 1 else shapely.unary_union(parts)
        if o.top_msl_m is not None:
            top = float(np.max(frame.msl_to_enu_z(xy, float(o.top_msl_m))))
        else:
            zg = _ground_z_max(terrain, geom)
            if not math.isfinite(zg):
                msl = dem.sample_msl(np.asarray(o.coords_wgs84)[:, 0], np.asarray(o.coords_wgs84)[:, 1])
                ok = np.isfinite(msl)
                if not ok.any():
                    warnings.append(f"obstacle_outside_dem({o.obs_id}): 略過")
                    continue
                zg = float(np.max(frame.msl_to_enu_z(xy[ok], msl[ok])))
                warnings.append(f"obstacle_outside_dem({o.obs_id}): 地面高以頂點 DEM 值近似")
            top = zg + float(o.height_agl_m)
        buf = float(o.keepout_buffer_m) if o.keepout_buffer_m is not None else float(cfg.b_h_m)
        out.append(ObstacleEnu(geom=geom, top_z_m=top, obs_id=o.obs_id, category=o.category, buffer_m=buf))
    return out, warnings


def burn_obstacles(terrain: TerrainGrid, obstacles_enu: Sequence[ObstacleEnu], cfg: PlannerConfig) -> list[str]:
    """障礙燒入（D24）：``z ← max(z, top + b_v)`` 於 ``geom ⊕ disc(buffer)`` 內；`terrain.is_dsm` 時略過 tree/building。

    回傳已燒入之 obs_id。單調且冪等（重複呼叫不改變結果）。
    """
    polys: list[shapely.Polygon] = []
    tops: list[float] = []
    ids: list[str] = []
    for o in obstacles_enu:
        if terrain.is_dsm and o.category in _BURN_SKIP_ON_DSM:
            continue
        polys.append(shapely.buffer(o.geom, o.buffer_m))
        tops.append(o.top_z_m + float(cfg.b_v_m))
        ids.append(o.obs_id)
    if polys:
        terrain.burn(polys, tops)
    return ids


# ---------------------------------------------------------------------------
# 感測器、能量、鏈路
# ---------------------------------------------------------------------------

def build_sensors(fleet: Sequence[UAVInstance], payload_reqs: dict[str, Any]) -> tuple[
        dict[tuple[str, str], Sensor], dict[str, frozenset[str]], list[str]]:
    """(cls, req_id) → `sensor_model(hw, req)`（D43）；硬體取該構型機隊中第一個具該 kind 之硬體。

    回傳 ``(sensors, available_kinds, warnings)``；警告：``sensor_spec_default_example(cls)``（使用示例硬體）、
    ``sensor_hardware_heterogeneous(cls, kind)``（同構型多種同類硬體，採第一個）。
    """
    hw_by_cls_kind: dict[str, dict[str, SensorHardware]] = {}
    kinds: dict[str, set[str]] = {}
    warnings: list[str] = []
    hetero: set[tuple[str, str]] = set()
    for u in fleet:
        d = hw_by_cls_kind.setdefault(u.cls, {})
        kinds.setdefault(u.cls, set()).update(u.available_kinds)
        for hw in u.sensors:
            if hw.kind not in d:
                d[hw.kind] = hw
            elif d[hw.kind] != hw and (u.cls, hw.kind) not in hetero:
                hetero.add((u.cls, hw.kind))
                warnings.append(f"sensor_hardware_heterogeneous({u.cls}, {hw.kind}): 採第一架之硬體")
    sensors: dict[tuple[str, str], Sensor] = {}
    default_warned: set[str] = set()
    for cls, d in hw_by_cls_kind.items():
        for req_id, req in payload_reqs.items():
            hw = d.get(req.kind)
            if hw is None:
                continue
            sensors[(cls, req_id)] = sensor_model(hw, req)
            if getattr(hw, "is_default_example", False) and cls not in default_warned:
                default_warned.add(cls)
                warnings.append(f"sensor_spec_default_example({cls})")
    # 規格 §6.2：``both`` 區之 (req_id, req_id + "_rgb") 兩需求若由「同一構型」同時滿足（該構型同時掛載
    # LiDAR 與 RGB），則該構型只鋪一套條帶、取兩者較窄之間距（`platform.CombinedSensor`），兩種感測需求
    # 同時達成。此前只鋪第一個相符 Sensor 的那一套，第二個感測需求會無聲消失（審查 R-04 blocker）。
    for (cls, req_id) in list(sensors):
        if req_id.endswith(_RGB_REQ_SUFFIX):
            continue
        other = sensors.get((cls, req_id + _RGB_REQ_SUFFIX))
        base = sensors[(cls, req_id)]
        if other is None or getattr(other, "kind", None) == getattr(base, "kind", None):
            continue
        merged = combine_sensors(base, other)
        if merged is not base:
            sensors[(cls, req_id)] = merged
            warnings.append(f"dual_sensor_single_pass({cls}, {req_id}): 同機雙感測 → 取較窄間距鋪一套條帶")
    available = {c: frozenset(k) for c, k in kinds.items()}
    return sensors, available, warnings


_CRAB_REF_AGL_M: float = 100.0
"""`apply_crab_margin` 回報 w_eff/w 與檢查 w_eff > 0 的參考高度（針孔／線掃描／梯形足跡 w、l 皆 ∝ h，比值與 h 無關）。"""


def apply_crab_margin(sensors: dict[tuple[str, str], Sensor], specs: dict[str, PlatformSpec],
                      wind_speed_mps: float) -> tuple[dict[tuple[str, str], Sensor], list[str]]:
    """建圖模式偏流餘裕（`MappingModeConfig.crab_margin`）：逐構型 ψ_c = asin(W / V_a,c)（V_a,c = 巡航空速、
    W = 規劃風速），寫進該構型每個感測器的 ``crab_rad``（CombinedSensor 寫進各子感測器），使條帶間距改用
    w_eff = w cosψ − l sinψ。W ≥ V_a,c 或 w_eff ≤ 0 → MissionInputError（不默默照原間距飛）。

    回傳 ``(sensors, warnings)``；每個 (cls, req_id) 一筆 ``mapping_crab_margin(...)`` 有效值紀錄（ψ = 0 也記）。
    """
    out = dict(sensors)
    warnings: list[str] = []
    W = float(wind_speed_mps)
    for (cls, rid), s in sensors.items():
        va = float(specs[cls].cruise_airspeed_mps)
        if W >= va:
            raise MissionInputError(
                f"建圖模式偏流餘裕：規劃風速 {W:g} m/s ≥ 構型 {cls} 巡航空速 {va:g} m/s，無法偏流維持航跡")
        psi = math.asin(W / va) if W > 0.0 else 0.0
        subs = getattr(s, "sensors", None)
        if psi == 0.0:
            new, check = s, ()
        elif subs is not None:                                  # CombinedSensor：間距取子感測器最小值
            check = tuple(dataclasses.replace(x, crab_rad=psi) for x in subs)
            new = dataclasses.replace(s, sensors=check)
        else:
            new = dataclasses.replace(s, crab_rad=psi)
            check = (new,)
        ratio = 1.0
        for x in check:
            w = float(x.swath_m(_CRAB_REF_AGL_M))
            l = float(x.footprint_len_m(_CRAB_REF_AGL_M))
            w_eff = crabbed_width_m(w, l, psi)
            if not w_eff > 0.0:
                raise MissionInputError(
                    f"建圖模式偏流餘裕：構型 {cls}（{rid}）ψ = asin({W:g}/{va:g}) = {math.degrees(psi):.2f}° 時 "
                    f"w_eff = w cosψ − l sinψ = {w_eff:.1f} m ≤ 0（@{_CRAB_REF_AGL_M:g} m：w {w:.1f}、l {l:.1f} m）"
                    f"⇒ 足跡轉了之後沒有保守可用寬；降低規劃風速或把相機長邊轉成垂直航跡")
            ratio = min(ratio, w_eff / w)
        out[(cls, rid)] = new
        warnings.append(f"mapping_crab_margin({cls}/{rid}: ψ={math.degrees(psi):.2f}°（W {W:.2f} / V_a {va:.2f} m/s）"
                        f" → w_eff/w={ratio:.4f}；間距 = w_eff·(1 − side_overlap))")
    return out, warnings


def build_energy_model(uav: UAVInstance, spec: PlatformSpec, cfg: PlannerConfig,
                       weather_obs: Any | None) -> tuple[EnergyModel, list[str]]:
    """單機 EnergyModel（規格 §5.1–§5.7、D39）。

    E_nom = `sortie.nominal_wh_resolved(spec)`；E_avail = `sortie.available_wh_resolved(E_nom)`（Wh/mAh 但 nominal None →
    MissionInputError 由此傳出）；僅 soc 且 nominal None → 基準分鐘模式（E_avail = soc·T_ref；`reference_battery_wh`
    不得單獨存在）。E_ref = `spec.reference_battery_wh`（None → = E_nom，警告 ``reference_battery_assumed``）。
    k_bm / k_as 取 cfg 值（spec 驗證旗標為 True 時 EnergyModel 自行改 1.0）。
    """
    warnings: list[str] = []
    s = uav.sortie
    nominal = s.nominal_wh_resolved(spec)
    available = s.available_wh_resolved(nominal)
    reference = spec.reference_battery_wh
    if available is None:                                   # 基準分鐘模式（規格 §5.7）
        if reference is not None:
            raise MissionInputError(
                f"SortieEnergy[{uav.uav_id}]: 僅給 soc_fraction 而 reference_battery_wh={reference} 已知但標稱容量未知；"
                f"缺鍵：pack_nominal_wh/battery_nominal_wh")
        available = float(s.soc_fraction) * float(spec.reference_endurance_min)
        nominal = None
        warnings.append(f"base_minute_mode({uav.uav_id})")
    else:
        if s.pack_nominal_wh is None:
            warnings.append(f"battery_nominal_example({uav.uav_id})")
        if reference is None:
            warnings.append(f"reference_battery_assumed({uav.uav_id})")
    temp, assumed = s.battery_temp_resolved(weather_obs)
    if assumed:
        warnings.append(f"battery_temp_assumed({uav.uav_id})")
    model = EnergyModel(spec=spec, auw_kg=float(s.auw_kg), available_wh=float(available), nominal_wh=nominal,
                        reference_wh=reference, battery_temp_c=float(temp), correction=1.0,
                        k_bm=float(cfg.k_bm), k_as=float(cfg.k_as),
                        temp_cap_coef_per_c=float(cfg.temp_cap_coef_per_c), temp_cap_floor=float(cfg.temp_cap_floor))
    return model, warnings


def _representative_model(cls: str, models: list[EnergyModel], spec: PlatformSpec, cfg: PlannerConfig) -> EnergyModel:
    """構型代表模型：最大 AUW、最小 E_avail、最大 E_nom（保留量偏大 = 保守）、最低電池溫度。"""
    modes = {m.base_minute_mode for m in models}
    if len(modes) > 1:
        raise MissionInputError(f"構型 {cls} 之機隊能量單位不一致（基準分鐘模式與 Wh 混用）")
    base_minute = models[0].base_minute_mode
    auw = max(m.auw_kg for m in models)
    avail = min(m.available_wh for m in models)
    temp = min(m.battery_temp_c for m in models)
    nominal = None if base_minute else max(float(m.nominal_wh) for m in models if m.nominal_wh is not None)
    reference = None if base_minute else spec.reference_battery_wh
    return EnergyModel(spec=spec, auw_kg=auw, available_wh=avail, nominal_wh=nominal, reference_wh=reference,
                       battery_temp_c=temp, correction=1.0, k_bm=float(cfg.k_bm), k_as=float(cfg.k_as),
                       temp_cap_coef_per_c=float(cfg.temp_cap_coef_per_c), temp_cap_floor=float(cfg.temp_cap_floor))


def link_polygon(cls: str, spec: PlatformSpec, c2: C2Link, frame: GeoFrame, dem: DemGrid,
                 ground_station_xy: np.ndarray, cfg: PlannerConfig) -> shapely.Geometry:
    """P_link,c（ENU；規格 §3.1、D11）：`range_km_for(cls, spec)` 之圓（圓心 = 地面站）與／或 coverage polygon（兩者皆給取交集）。

    皆缺 → MissionInputError（列鍵名）；``cfg.enforce_link=False`` → 半徑 :data:`UNBOUNDED_LINK_RADIUS_M` 之圓。
    """
    gs = np.asarray(ground_station_xy, dtype=np.float64).reshape(2)
    if not cfg.enforce_link:
        return Point(gs).buffer(UNBOUNDED_LINK_RADIUS_M, quad_segs=8)
    parts: list[shapely.Geometry] = []
    r_km = c2.range_km_for(cls, spec)
    if r_km is not None:
        parts.append(Point(gs).buffer(float(r_km) * 1000.0, quad_segs=_LINK_DISC_QUAD_SEGS))
    cov = c2.coverage_for(cls)
    if cov is not None:
        parts.append(polygon_enu(frame, dem, cov, name=f"C2Link.coverage_polygon_wgs84[{cls}]"))
    if not parts:
        raise MissionInputError(
            f"構型 {cls} 之 C2 鏈路覆蓋未定義；缺鍵：c2.range_km[{cls}]/c2.coverage_polygon_wgs84[{cls}]/spec.c2_link_range_km")
    poly = parts[0]
    for p in parts[1:]:
        poly = poly.intersection(p)
    if poly.is_empty or poly.area <= 0.0:
        raise MissionInputError(f"構型 {cls} 之 C2 鏈路圓與 coverage polygon 交集為空")
    return poly


# ---------------------------------------------------------------------------
# build_context
# ---------------------------------------------------------------------------

def build_context(inputs: MissionInputs, cfg: PlannerConfig, *,
                  terrain_margin_m: float = TERRAIN_MARGIN_M) -> MissionContext:
    """組裝 :class:`MissionContext`（契約 §6A）。

    步驟：platform_overrides（規格 §3.5）→ `GeoFrame`（原點 = home 地面）→ 站點 ENU → `TerrainGrid.from_dem`
    （所有輸入幾何之外接矩形 ⊕ ``terrain_margin_m``）→ `WindField` → h_clr（D11）→ 區域 ENU → h_ceil（§4.6，
    以燒入前地形）→ NoFly（D22 有效窗）⊕ b_trk（D41）→ 障礙 ENU → `routing.KeepoutSet.build`（延遲 import；
    不可用時 None）＋ 障礙燒入（D24，只做一次）→ 感測器（D43）→ EnergyModel 逐機與逐構型代表 → P_link,c、ρ^sym。

    輸入不完整（terrain_clearance / C2 / E_nom）一律丟 :class:`MissionInputError`（preflight 轉 NO_GO 並列鍵名）。
    """
    warnings: list[str] = []
    dem = inputs.dem

    # 1) spec（覆寫）
    base, w = base_specs_from_fleet(inputs.fleet)
    warnings += w
    specs, w = apply_platform_overrides(base, inputs.platform_overrides)
    warnings += w

    # 2) 座標框架與站點
    home = inputs.home
    h0 = site_ground_msl(home, dem)
    frame = GeoFrame(home.lat_deg, home.lon_deg, h0, geoid_undulation_m=float(dem.geoid_undulation_m))
    sites_enu: dict[str, tuple[np.ndarray, float]] = {}
    for sid, site in inputs.sites.items():
        msl = site_ground_msl(site, dem)
        enu = frame.to_enu(site.lat_deg, site.lon_deg, msl)[0]
        sites_enu[sid] = (enu[:2].copy(), float(enu[2]))
    gs_site = inputs.c2.ground_station
    if gs_site.site_id in sites_enu and (gs_site.lat_deg, gs_site.lon_deg) == inputs.sites[gs_site.site_id].latlon:
        gs_xy = sites_enu[gs_site.site_id][0].copy()
    else:
        gs_xy = frame.to_enu(gs_site.lat_deg, gs_site.lon_deg, site_ground_msl(gs_site, dem))[0][:2].copy()

    # 3) 地形（邊界 = 所有幾何 ⊕ margin）
    pts: list[np.ndarray] = [xy for xy, _ in sites_enu.values()] + [gs_xy]
    area_rings: dict[str, tuple[np.ndarray, list[np.ndarray]]] = {}
    for a in inputs.areas:
        shell = _latlon_to_xy(frame, dem, a.polygon_wgs84)
        holes = [_latlon_to_xy(frame, dem, h) for h in a.holes_wgs84]
        area_rings[a.area_id] = (shell, holes)
        pts.append(shell)
    for z in inputs.airspace.no_fly:
        pts.append(_latlon_to_xy(frame, dem, z.polygon_wgs84))
    for o in inputs.obstacles:
        pts.append(_latlon_to_xy(frame, dem, o.coords_wgs84))
    bounds = _bounds_with_margin(pts, float(terrain_margin_m))
    terrain = TerrainGrid.from_dem(dem, frame, float(cfg.dem_step_m), bounds)
    for sid, (xy, _) in sites_enu.items():
        if not np.isfinite(terrain.z_at(xy.reshape(1, 2))[0]):
            raise MissionInputError(f"Site[{sid}] 位於 DEM 範圍外")

    # 4) 風場
    z_ground_home = sites_enu[home.site_id][1]
    wind = WindField(list(inputs.wind), frame, z_ground_home)
    warnings += [f"wind: {n}" for n in wind.notes]

    # 5) 淨空
    h_clr, w = resolve_terrain_clearance(inputs.airspace, specs, dem.sigma_z_m)
    warnings += w

    # 6) 區域（ENU；有效多邊形）
    areas_enu: dict[str, shapely.Polygon] = {}
    for a in inputs.areas:
        areas_enu[a.area_id] = polygon_enu(frame, dem, a.polygon_wgs84, a.holes_wgs84, name=f"AreaSpec[{a.area_id}]")
        if not math.isfinite(terrain.max_in_polygon(areas_enu[a.area_id])):
            warnings.append(f"area_outside_dem({a.area_id})")

    # 7) 高度上限（燒入前之地形）
    need_msl = inputs.airspace.max_msl_m is not None and inputs.airspace.waiver_max_agl_m is None \
        and cfg.h_ceil_override_m is None
    max_msl = _max_terrain_msl(frame, terrain, list(areas_enu.values())) if need_msl else None
    h_ceil, h_ceil_source, w = resolve_ceiling(inputs.airspace, cfg, specs, max_msl)
    warnings += w

    # 8) 禁區（D22 有效窗；D41 ⊕ b_trk）
    t_window = inputs.airspace.time_window_s if cfg.enforce_time_windows else None
    active_nofly: list[NoFlyZone] = list(inputs.airspace.active_no_fly(t_window))
    nofly_buffered: list[shapely.Polygon] = []
    for z in active_nofly:
        poly = polygon_enu(frame, dem, z.polygon_wgs84, name=f"NoFlyZone[{z.zone_id}]")
        nofly_buffered.append(shapely.buffer(poly, float(cfg.b_trk_m)))

    # 9) 障礙物與 keep-out
    obstacles_enu, w = obstacles_to_enu(inputs.obstacles, frame, dem, terrain, cfg)
    warnings += w
    keepouts: Any = None
    try:
        from .routing import KeepoutSet  # 延遲 import：routing 為同層平行模組
    except ImportError:
        warnings.append("routing_unavailable_keepouts_none")
    else:
        keepouts = KeepoutSet.build(list(inputs.obstacles), active_nofly, frame, terrain, cfg, t_window)
    if obstacles_enu and np.array_equal(terrain.z, terrain.z_raw):
        burned = burn_obstacles(terrain, obstacles_enu, cfg)
        if burned:
            warnings.append(f"obstacles_burned_by_context({','.join(burned)})")

    # 10) 感測器
    sensors, available_kinds, w = build_sensors(inputs.fleet, inputs.payload_reqs)
    warnings += w
    # 10b) 建圖模式（opt-in）：有效值寫進警告；偏流餘裕改感測器間距（W = 各風層最大平均風速）
    mapping = inputs.mapping_mode
    if mapping is not None:
        warnings.append("mapping_mode(" + ", ".join(f"{k}={v}" for k, v in mapping.to_dict().items()) + ")")
        if mapping.crab_margin:
            sensors, w = apply_crab_margin(sensors, specs, wind.max_speed())
            warnings += w

    # 11) 能量模型
    models: dict[str, EnergyModel] = {}
    by_cls: dict[str, list[EnergyModel]] = {}
    auw_kg: dict[str, float] = {}
    for u in inputs.fleet:
        spec = specs[u.cls]
        m, w = build_energy_model(u, spec, cfg, inputs.weather_obs)
        warnings += w
        models[u.uav_id] = m
        by_cls.setdefault(u.cls, []).append(m)
        auw_kg[u.cls] = max(auw_kg.get(u.cls, 0.0), m.auw_kg)
    models_rep = {c: _representative_model(c, ms, specs[c], cfg) for c, ms in by_cls.items()}

    # 12) 鏈路與半徑
    link_poly: dict[str, shapely.Polygon] = {}
    rho_sym: dict[str, float] = {}
    for c, spec in specs.items():
        link_poly[c] = link_polygon(c, spec, inputs.c2, frame, dem, gs_xy, cfg)
        rho_sym[c] = math.inf if not cfg.enforce_link else float(spec.symmetric_mission_radius_km[0]) * 1000.0

    # 13) 逐機起飛站點（選填 UAVInstance.start_site_id；未設定 → home）
    start_sites: dict[str, str] = {}
    for u in inputs.fleet:
        sid = u.start_site_id or home.site_id
        if sid not in sites_enu:                       # MissionInputs 已檢查，此處為防禦性
            raise MissionInputError(f"UAVInstance[{u.uav_id}].start_site_id={sid!r} 不在站點清單 {sorted(sites_enu)}")
        start_sites[u.uav_id] = sid
    if any(s != home.site_id for s in start_sites.values()):
        warnings.append("per_uav_start_sites(" + ",".join(
            f"{u}@{s}" for u, s in start_sites.items() if s != home.site_id) + ")")

    # 13b) 逐機回收站（選填 `UAVInstance.recovery_site_id`；未設定 → 全域 recovery）
    #   ⚠ vendored #4。全員未設定時 recovery_sites 每一項都等於全域值 ⇒ 下游逐位元不變。
    _rec_global = inputs.recovery_site.site_id
    recovery_sites: dict[str, str] = {}
    for u in inputs.fleet:
        _sid = u.recovery_site_id or _rec_global
        if _sid not in sites_enu:
            raise MissionInputError(
                f"UAVInstance[{u.uav_id}].recovery_site_id={_sid!r} 不在站點清單 {sorted(sites_enu)}")
        recovery_sites[u.uav_id] = _sid
    if any(x != _rec_global for x in recovery_sites.values()):
        warnings.append("per_uav_recovery_sites(" + ",".join(
            f"{u}@{x}" for u, x in recovery_sites.items() if x != _rec_global) + ")")

    return MissionContext(
        inputs=inputs, cfg=cfg, frame=frame, terrain=terrain, wind=wind, keepouts=keepouts,
        specs=specs, sensors=sensors, models=models, models_rep=models_rep, sites_enu=sites_enu,
        link_poly=link_poly, rho_sym_m=rho_sym, h_clr_m=h_clr, h_ceil_m=h_ceil, h_ceil_source=h_ceil_source,
        areas_enu=areas_enu, recovery_site_id=inputs.recovery_site.site_id, warnings=warnings,
        nofly_buffered=nofly_buffered, available_kinds=available_kinds, auw_kg=auw_kg,
        obstacles_enu=obstacles_enu, ground_station_xy=gs_xy, t_window=t_window, start_sites=start_sites,
        recovery_sites=recovery_sites,
    )
