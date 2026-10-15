"""decompose.py — 分解流程、方向評分、保守 AGL 回饋、適配度、殘帶（模組契約 §7）。

對應規格：§4.6（作業高度與高度帶政策，D12）、§7.1（總流程）、§7.2（簡化與凸分割，經 geometry）、
§7.3（候選掃描方向與風／地形感知評分，D15）、§7.4（條帶產生，經 geometry.lay_strips + altitude.local_envelope，D3）、
§7.5（殘帶指派，D17）、§7.6（構型適配度）、§9.2（T_single = exact_time）；裁決 D16、D38（迴轉帶）、D41（禁區緩衝）。

全域約定（契約 §0）：ENU 公尺；航跡方位 theta 自北順時針，t̂ = (sin θ, cos θ)、右法向 n̂ = (cos θ, −sin θ)；
掃描座標 u 沿 t̂、v 沿 n̂；條帶方向 dir ∈ {−1, +1}（−1 = LR、+1 = RL）；所有高度為 ENU z、AGL 以 `_m` 標示。

第 3 層模組：只依賴 geometry / altitude / context / wind / platform / mission / dubins / geo。
"""
from __future__ import annotations

import itertools
import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np
import shapely
from shapely.geometry import MultiPolygon, Point, Polygon

from .altitude import make_local_envelope_fn
from .context import DecomposeContext
from .dubins import dubins_all_times, omega_length, skip_k, uturn_length
from .geometry import (
    Cell,
    Strip,
    clip_band,
    convex_partition,
    lay_strips,
    min_width_direction,
    projection_width,
    residual_subcell,
    simplify_outward,
    strip_count,
    sweep_frame,
    sweep_origin,
    to_sweep_coords,
    turn_band,
)
from .mission import AreaSpec, PlannerConfig
from .platform import PLATFORM_HERO, EnergyModel, PlatformSpec, Sensor
from .wind import circle_time_factor, ground_speed, proxy_turn_time

__all__ = [
    "AltitudeBand",
    "ProxyTime",
    "altitude_band",
    "d_turn_m",
    "direction_candidates",
    "proxy_time",
    "build_strips",
    "exact_time",
    "suitability",
    "decompose_area",
    "residual_candidates",
    "sensor_compatible",
    "class_can_serve",
    "resolve_sensor",
    "r_eff_for_class",
    "end_extension_for",
    "seam_tie_overlap_m",
    "area_edge_deltas",
    "t_use_s_for",
    "RGB_REQ_SUFFIX",
    "CrossSet",
    "CrossStripRefused",
    "cross_altitude_check",
    "cross_central_band",
    "build_cross_set",
]

TWO_PI: float = 2.0 * math.pi
RGB_REQ_SUFFIX: str = "_rgb"            # context.build_sensors：both/any 區之 rgb 需求鍵 = req_id + "_rgb"
_ANGLE_TOL_RAD: float = math.radians(1.0)   # 方向候選去重容差
_PROXY_DS_M: float = 25.0               # §7.3 代理之沿線取樣間距
_SLOPE_PROBE_M: float = 10.0            # 橫向坡度探針半距（z(p ± δ n̂)）
_GRAD_EPS: float = 1e-3                 # 等高線候選：平均梯度模低於此值（rad）視為平地、不加入候選
_TINY_AREA_M2: float = 1e-6
_MIN_VETO_AREA_M2: float = 1.0          # D38 修正 R-01：小於此面積之迴轉帶／禁區相交視為數值退化，不構成硬性否決

_N_TOP_DIRS: int = 3                    # 規格 §7.1：精算前三名


# ---------------------------------------------------------------------------
# 資料型別（契約 §7）
# ---------------------------------------------------------------------------
@dataclass(slots=True)
class AltitudeBand:
    """胞層級高度帶（規格 §4.6；D12）。

    `h_op_m`：clip(h_target, h_minfw, min(h_max, h_ceil))（AGL）。
    `h_lo_m` / `h_hi_m`：可用作業 AGL 之下／上緣（`altitude.layering` 用）：h_lo = max(h_minfw, h_clr)、
    h_hi = min(h_ceil − 起伏, h_max)，起伏 = max_Ω z_g − min_Ω z_g（Ω = 胞 ⊕ disc(d_turn)）。
    `dh_band_m`：規格式 (min_Ω z_g + h_ceil) − (max_Ω z_g + max(h_minfw, h_clr))（只含 h_ceil）。
    `reason` ∈ {"", "band_lt_min", "A_needs_waiver", "gsd_unattainable"}，另有 "no_sensor"、"terrain_unknown"
    （契約外，見 CONTRACT_DEVIATIONS）。
    """
    cls: str
    h_op_m: float
    h_lo_m: float
    h_hi_m: float
    dh_band_m: float
    feasible: bool
    reason: str


@dataclass(slots=True)
class ProxyTime:
    """§7.3 方向評分結果。`one_way` ∈ {0, −1, +1}：0 = 兩向皆可行（或皆不可行，此時 T_s = inf）、
    −1 = 只有 LR（θ = alpha）可行、+1 = 只有 RL（θ + π）可行。"""
    alpha_rad: float
    T_s: float
    n_strips: int
    t_turn_s: float
    pen_slope_s: float
    pen_dz_s: float
    pen_keepout_s: float
    slope_exceeds: bool
    one_way: int


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------
def _mod2pi(x: float) -> float:
    return float(x % TWO_PI)


def _polys_of(geom: shapely.Geometry | None, min_area: float = _TINY_AREA_M2) -> list[Polygon]:
    if geom is None or geom.is_empty:
        return []
    if isinstance(geom, Polygon):
        return [geom] if geom.area > min_area else []
    out: list[Polygon] = []
    for g in getattr(geom, "geoms", []):
        out.extend(_polys_of(g, min_area))
    return out


def _h_ceil_of(ctx: DecomposeContext, cls: str) -> float:
    """h_ceil,c（AGL）：cfg.h_ceil_override_m（legacy inf）→ ctx.h_ceil_m[cls]（None → inf）。"""
    ov = getattr(ctx.cfg, "h_ceil_override_m", None)
    if ov is not None:
        return float(ov)
    v = ctx.h_ceil_m.get(cls) if ctx.h_ceil_m else None
    return math.inf if v is None else float(v)


def _z_ground_home(ctx: DecomposeContext) -> float:
    z = ctx.terrain.z_at(np.asarray(ctx.home_xy, dtype=float).reshape(1, 2))[0]
    return float(z) if np.isfinite(z) else 0.0


def _terrain_minmax(ctx: DecomposeContext, poly: shapely.Geometry) -> tuple[float, float]:
    """(min, max) ENU 地形（含燒入障礙）於多邊形內；範圍外 → (nan, nan)。"""
    z_min = ctx.terrain.min_in_polygon(poly)
    z_max = ctx.terrain.max_in_polygon(poly)
    return float(z_min), float(z_max)


def _raw_terrain_minmax(ctx: DecomposeContext, poly: shapely.Geometry) -> tuple[float, float]:
    """(min, max) **自然**地形（`terrain.z_raw`，不含燒入障礙）於多邊形內：多邊形內之格點 ∪ 各頂點所在格之四角節點。
    高度帶（規格 §4.6）之 z_g 指 DEM 地形；孤立電塔燒入後會使 Ω 內「起伏」虛增 b_v + 塔高（山丘情境 Z1 55 → 101 m），
    故此處用 z_raw（障礙淨空由局部包絡與 keep-out 另行保證）。多邊形內無節點 → 退回 :func:`_terrain_minmax`。"""
    T = ctx.terrain
    if poly is None or poly.is_empty:
        return math.nan, math.nan
    minx, miny, maxx, maxy = poly.bounds
    i0 = max(0, int(math.floor((minx - T.x0_m) / T.step_m)))
    i1 = min(T.nx - 1, int(math.ceil((maxx - T.x0_m) / T.step_m)))
    j0 = max(0, int(math.floor((miny - T.y0_m) / T.step_m)))
    j1 = min(T.ny - 1, int(math.ceil((maxy - T.y0_m) / T.step_m)))
    if i1 < i0 or j1 < j0:
        return _terrain_minmax(ctx, poly)
    xs = T.x0_m + np.arange(i0, i1 + 1) * T.step_m
    ys = T.y0_m + np.arange(j0, j1 + 1) * T.step_m
    gx, gy = np.meshgrid(xs, ys)
    inside = shapely.contains_xy(poly, gx.ravel(), gy.ravel()).reshape(gx.shape)
    vals = T.z_raw[j0:j1 + 1, i0:i1 + 1][inside].astype(float)
    # 頂點所在格之四角節點（小多邊形／邊界保守）
    vx = np.asarray(shapely.get_coordinates(poly), dtype=float)
    ci = np.clip(np.floor((vx[:, 0] - T.x0_m) / T.step_m).astype(int), 0, T.nx - 2)
    cj = np.clip(np.floor((vx[:, 1] - T.y0_m) / T.step_m).astype(int), 0, T.ny - 2)
    corner = np.concatenate([T.z_raw[cj, ci], T.z_raw[cj + 1, ci], T.z_raw[cj, ci + 1], T.z_raw[cj + 1, ci + 1]]).astype(float)
    allv = np.concatenate([vals, corner])
    allv = allv[np.isfinite(allv)]
    if allv.size == 0:
        return _terrain_minmax(ctx, poly)
    return float(allv.min()), float(allv.max())


def resolve_sensor(ctx: DecomposeContext, cls: str, req_id: str) -> tuple[Sensor, str] | None:
    """(cls, req_id) 之感測器：先找 `ctx.sensors[(cls, req_id)]`，再找 `(cls, req_id + "_rgb")`
    （context.build_sensors 對 both/any 區之 rgb 需求鍵約定）。回傳 (sensor, 實際 req_id) 或 None。"""
    key = (cls, req_id)
    if key in ctx.sensors:
        return ctx.sensors[key], req_id
    key2 = (cls, req_id + RGB_REQ_SUFFIX)
    if key2 in ctx.sensors:
        return ctx.sensors[key2], req_id + RGB_REQ_SUFFIX
    return None


def sensor_compatible(sensor_req: str, available_kinds: frozenset[str] | set[str]) -> bool:
    """κ(σ, c)（規格 §9.1 H1）：lidar → "lidar" ∈；rgb → "rgb" ∈；both → 兩者皆 ∈；any → True。"""
    kinds = frozenset(available_kinds)
    if sensor_req == "any":
        return True
    if sensor_req == "lidar":
        return "lidar" in kinds
    if sensor_req == "rgb":
        return "rgb" in kinds
    if sensor_req == "both":
        return "lidar" in kinds and "rgb" in kinds
    raise ValueError(f"unknown sensor_req {sensor_req!r}")


def class_can_serve(sensor_req: str, cls: str, available_kinds: frozenset[str] | set[str]) -> bool:
    """構型 c 能否承接需求 σ（D17）：both 區由 A（lidar）與 B（rgb）各出一套獨立條帶，故
    A 只需 "lidar"、B 只需 "rgb"；其餘同 :func:`sensor_compatible`。"""
    kinds = frozenset(available_kinds)
    if sensor_req == "both":
        if sensor_compatible("both", kinds):
            return True
        return ("lidar" in kinds) if cls == "A" else ("rgb" in kinds)
    return sensor_compatible(sensor_req, kinds)


def _wind_speed_for_r_eff(ctx: DecomposeContext, cls: str, z_top_m: float | None = None) -> float:
    """W_R,c（規格 §3.2 政策）：max_layer → [z_site + H_tc,lo, z_top] 之最大風速；op_layer → W(z_top)。
    z_top 缺省 = 地形全域最高 + h_ceil（有限）或 + 300 m。"""
    spec = ctx.specs[cls]
    z_site = _z_ground_home(ctx)
    if z_top_m is None:
        zmax = float(np.nanmax(ctx.terrain.z)) if np.isfinite(ctx.terrain.z).any() else z_site
        h_ceil = _h_ceil_of(ctx, cls)
        z_top_m = zmax + (h_ceil if math.isfinite(h_ceil) else 300.0)
    tc = spec.transition_cmd(transition_profile=ctx.cfg.transition_profile)
    z_lo = z_site + tc.h_tc_clear_m
    z_hi = max(float(z_top_m), z_lo)
    if ctx.cfg.r_eff_wind_policy == "op_layer":
        return float(ctx.wind.speed_at(z_hi))
    return float(ctx.wind.max_speed_between(z_lo, z_hi))


def r_eff_for_class(ctx: DecomposeContext, cls: str, z_top_m: float | None = None) -> float:
    """R_eff,c = spec.r_eff(W_R,c)（D1；風速政策見 :func:`_wind_speed_for_r_eff`）。"""
    return float(ctx.specs[cls].r_eff(_wind_speed_for_r_eff(ctx, cls, z_top_m)))


def end_extension_for(ctx: DecomposeContext, cls: str, sensor: Sensor, h_op_m: float,
                      z_op_m: float | None = None) -> float:
    """e_ext,c = max(V_g,max t_settle, l_B(1−o_fwd))（規格 §4.5）；cfg.e_ext_override_m（legacy 0）優先。
    V_g,max = V_a + W（W = 作業層風速；z_op 缺省 → W_R 政策）。
    建圖模式另加 `MappingModeConfig.lead_in_m`（條帶兩端各再延長；迴轉帶、代理時間、條帶端點一致吃到）。"""
    ov = getattr(ctx.cfg, "e_ext_override_m", None)
    if ov is not None:
        return float(ov) + _lead_in_m(ctx)
    spec = ctx.specs[cls]
    w = float(ctx.wind.speed_at(float(z_op_m))) if z_op_m is not None else _wind_speed_for_r_eff(ctx, cls)
    v_g_max = spec.cruise_airspeed_mps + w
    try:
        e = float(sensor.end_extension_m(v_g_max, ctx.cfg.t_settle_s, h_agl_m=float(h_op_m)))  # type: ignore[call-arg]
    except TypeError:
        e = float(sensor.end_extension_m(v_g_max, ctx.cfg.t_settle_s))
    return e + _lead_in_m(ctx)


def _lead_in_m(ctx: DecomposeContext) -> float:
    """建圖模式的條帶引入段 lead_in_m；未開建圖模式 → 0.0（x + 0.0 == x，舊行為逐位元不變）。"""
    mp = getattr(ctx, "mapping", None)
    return float(mp.lead_in_m) if mp is not None else 0.0


def seam_tie_overlap_m(ctx: DecomposeContext, widths_spacings: Sequence[tuple[float, float]]) -> float | None:
    """建圖模式接縫重疊目標 = max(w − s)（同區塊相鄰條帶的重疊；≥ side_overlap·w，有偏流餘裕時更大）。
    未開建圖模式或 seam_tie=False → None（呼叫端照用 δ_edge）。"""
    mp = getattr(ctx, "mapping", None)
    if mp is None or not mp.seam_tie:
        return None
    return max((float(w) - float(s) for w, s in widths_spacings), default=0.0)


def t_use_s_for(ctx: DecomposeContext, cls: str) -> float:
    """T^use_c（s）= e_usable / (P_c k_unc)·3600（規格 §5.3；能量關閉 → inf）。"""
    if not getattr(ctx.cfg, "enforce_energy", True):
        return math.inf
    m = ctx.models_rep.get(cls)
    if m is None:
        return math.inf
    p = m.p_cruise_w() * m.k_unc
    if not p > 0:
        return math.inf
    return max(0.0, float(m.e_usable_wh()) / p * 3600.0)


def _v_g_star(theta: np.ndarray, spec: PlatformSpec, wind_xy: np.ndarray, v_cap: float | np.ndarray,
              v_g_min: float) -> np.ndarray:
    """V_g^*(θ)（規格 §4.4）：V_a^* = max{V_a ∈ [V_a,min, V_a,c] : V_g ≤ V_cap}；V_g 對 V_a 單調 ⇒
    V_g^* = min(V_g(θ, V_a,c), V_cap)（若 V_g(V_a,c) 為 nan 則 V_g(V_a,min)）；
    V_g(θ, V_a,min) > V_cap 或 < v_g_min 或 nan → nan（該向不可行）。向量化 (N,)。"""
    th = np.atleast_1d(np.asarray(theta, dtype=float))
    vg_min = ground_speed(th, spec.v_min_mission(), wind_xy)
    vg_c = ground_speed(th, spec.cruise_airspeed_mps, wind_xy)
    with np.errstate(invalid="ignore"):
        ok = np.isfinite(vg_min) & (vg_min >= v_g_min) & (vg_min <= v_cap + 1e-9)
        vg = np.where(np.isfinite(vg_c), np.minimum(vg_c, v_cap), vg_min)
        vg = np.where(ok & (vg > 0), vg, np.nan)
    return vg


def _poly_to_uv(poly: shapely.Geometry, origin: np.ndarray, alpha: float) -> shapely.Geometry:
    """多邊形整體轉到掃描座標 (u, v)（仿射；u 沿 t̂、v 沿 n̂）。"""
    t_hat, n_hat = sweep_frame(alpha)
    o = np.asarray(origin, dtype=float).reshape(2)
    return shapely.transform(poly, lambda c: (c - o) @ np.stack([t_hat, n_hat], axis=0).T)


def _band_u_extents(poly_uv: shapely.Geometry, v_lo: np.ndarray, v_hi: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """各帶 [v_lo, v_hi] 與胞交集之 u 極值（向量化；空帶 → nan）。"""
    minx, _miny, maxx, _maxy = poly_uv.bounds
    n = len(v_lo)
    boxes = shapely.box(np.full(n, minx - 1.0), v_lo, np.full(n, maxx + 1.0), v_hi)
    inter = shapely.intersection(boxes, poly_uv)
    b = shapely.bounds(inter)
    u_lo = b[:, 0]
    u_hi = b[:, 2]
    empty = shapely.is_empty(inter) | ~np.isfinite(u_lo)
    u_lo = np.where(empty, np.nan, u_lo)
    u_hi = np.where(empty, np.nan, u_hi)
    return u_lo, u_hi


# ---------------------------------------------------------------------------
# §4.6 高度帶、D38 迴轉帶
# ---------------------------------------------------------------------------
def d_turn_m(cls: str, s_m: float, e_ext_m: float, ctx: DecomposeContext) -> float:
    """迴轉帶端線外延 d_turn,c = max{R_eff, √(4R² − (R + s/2)²) + R} + e_ext（D38；Ω 超出端線量）。
    A@160 m（s 168.05、e_ext 53）→ 534.7 m；B@100 m（s 102.81、e_ext 48）→ 336.0 m（契約 §W.3）。"""
    r = r_eff_for_class(ctx, cls)
    return d_turn_from_r(r, s_m, e_ext_m)


def d_turn_from_r(r_eff_m: float, s_m: float, e_ext_m: float) -> float:
    """同 :func:`d_turn_m`，但直接給 R_eff。"""
    r = float(r_eff_m)
    inner = 4.0 * r * r - (r + 0.5 * float(s_m)) ** 2
    omega = math.sqrt(inner) + r if inner > 0 else r
    return max(r, omega) + float(e_ext_m)


def _lateral_m(r_eff_m: float, s_m: float) -> float:
    """迴轉帶側向外延 R_eff − s/2（僅 s < 2R；D38）。"""
    return max(0.0, float(r_eff_m) - 0.5 * float(s_m))


def _end_turn_blocked(nf_uv: shapely.Geometry, u_lo: np.ndarray, u_hi: np.ndarray, v_lo: np.ndarray,
                      v_hi: np.ndarray, lateral_m: float, need_m: float) -> tuple[bool, bool]:
    """兩端「迴轉室」淨空檢查（規格 §7.3「禁區距端線 < R_eff + e_ext ⇒ 連 U 型也放不下」；D38 修正 R-01）。

    逐條帶（uv 座標，u_lo/u_hi 已含 ±e_ext）在端點外量測 need_m 深度、寬 [v_lo−lat, v_hi+lat] 之矩形，
    取其未被禁區佔用之 u 深度；某一端「所有條帶」深度都不足 ⇒ 該方向硬性不可行。
    舊碼改以 (cell ⊕ 全帶) ∩ NoFly > 1e-6 m² 否決，對緊鄰禁區之胞（decompose_area 以 F = P \\ (NoFly⊕b_trk)
    造胞，距離恆為 0）任何方向都成立，導致整片區域被丟棄。
    """
    need = max(float(need_m), 0.0)
    if need <= 0.0 or nf_uv.is_empty or len(u_lo) == 0:
        return False, False
    lat = max(float(lateral_m), 0.0)
    lo_box = shapely.box(u_lo - need, v_lo - lat, u_lo, v_hi + lat)
    hi_box = shapely.box(u_hi, v_lo - lat, u_hi + need, v_hi + lat)
    out: list[bool] = []
    for boxes, edge, sign in ((lo_box, u_lo, -1.0), (hi_box, u_hi, 1.0)):
        inter = shapely.intersection(boxes, nf_uv)
        b = shapely.bounds(inter)
        free = shapely.is_empty(inter) | (shapely.area(inter) <= _MIN_VETO_AREA_M2)
        near = (b[:, 0] - edge) if sign > 0 else (edge - b[:, 2])
        depth = np.where(free, need, np.maximum(near, 0.0))
        out.append(bool(np.all(depth < need - 1e-6)))
    return out[0], out[1]


def altitude_band(cell_poly: Polygon, cls: str, req_id: str, ctx: DecomposeContext) -> AltitudeBand:
    """胞層級高度帶（規格 §4.6，D12；無循環）。

    h_op = clip(h_target, h_minfw, min(h_max, h_ceil))；Ω = 胞 ⊕ disc(d_turn)；
    Δh_band = (min_Ω z_g + h_ceil) − (max_Ω z_g + max(h_minfw, h_clr))；可指派條件 Δh_band ≥ cfg.dh_band_min_m
    且 sensor.gsd_attainable(h_minfw, Δh_band,min)。Pioneer 於 h_ceil ≤ h_minfw 且無豁免 → reason "A_needs_waiver"。
    """
    spec = ctx.specs[cls]
    cfg = ctx.cfg
    h_minfw = float(spec.min_fw_operation_agl_m)
    h_clr = float(ctx.h_clr_m)
    h_ceil = _h_ceil_of(ctx, cls)
    found = resolve_sensor(ctx, cls, req_id)
    if found is None:
        h_op = max(h_minfw, min(h_minfw, h_ceil))
        return AltitudeBand(cls, h_op, max(h_minfw, h_clr), min(h_ceil, math.inf), -math.inf, False, "no_sensor")
    sensor, _rid = found
    h_max = float(sensor.agl_window()[1])
    h_target = float(sensor.h_target_m(h_minfw, cfg.dh_band_min_m, cfg.k_gsd_relax))
    hi_cap = min(h_max, h_ceil)
    h_op = max(h_minfw, min(h_target, hi_cap))
    # Ω 範圍：以 h_op 之間距與 e_ext 算 d_turn
    s = float(sensor.strip_spacing_m(h_op))
    e_ext = end_extension_for(ctx, cls, sensor, h_op)
    d_turn = d_turn_m(cls, s, e_ext, ctx)
    omega = cell_poly.buffer(d_turn)
    z_min, z_max = _raw_terrain_minmax(ctx, omega)
    if not (math.isfinite(z_min) and math.isfinite(z_max)):
        z_min, z_max = _raw_terrain_minmax(ctx, cell_poly)
    if not (math.isfinite(z_min) and math.isfinite(z_max)):
        return AltitudeBand(cls, h_op, max(h_minfw, h_clr), hi_cap, -math.inf, False, "terrain_unknown")
    relief = z_max - z_min
    h_lo = max(h_minfw, h_clr)
    dh_band = (z_min + h_ceil) - (z_max + h_lo)
    h_hi = min(h_ceil - relief, h_max)
    gsd_ok = bool(sensor.gsd_attainable(h_minfw, cfg.dh_band_min_m))
    reason = ""
    feasible = True
    if dh_band < float(cfg.dh_band_min_m) - 1e-9:
        feasible = False
        waiver = getattr(ctx.airspace, "waiver_max_agl_m", None) if ctx.airspace is not None else None
        if cls == "A" and waiver is None and math.isfinite(h_ceil) and h_ceil <= h_minfw + float(cfg.dh_band_min_m):
            reason = "A_needs_waiver"
        else:
            reason = "band_lt_min"
    elif not gsd_ok:
        feasible = False
        reason = "gsd_unattainable"
    return AltitudeBand(cls, float(h_op), float(h_lo), float(h_hi), float(dh_band), feasible, reason)


# ---------------------------------------------------------------------------
# §7.3 候選方向與代理時間
# ---------------------------------------------------------------------------
def _dedupe_angles(angles: Sequence[float], modulo: float = TWO_PI, tol: float = _ANGLE_TOL_RAD) -> list[float]:
    out: list[float] = []
    for a in angles:
        a = float(a) % modulo
        if not any(min(abs(a - b), modulo - abs(a - b)) <= tol for b in out):
            out.append(a)
    return out


def _contour_direction(cell_poly: Polygon, ctx: DecomposeContext) -> float | None:
    """等高線主方向 = DEM 梯度主軸 ⊥（結構張量最大特徵向量之垂直方向）；平地（|∇z| 平均 < _GRAD_EPS）→ None。
    梯度以 z_raw 之中央差分（±δ 探針）向量化取樣於胞內格點。"""
    minx, miny, maxx, maxy = cell_poly.bounds
    step = max(float(ctx.terrain.step_m), 25.0)
    nx = max(2, int((maxx - minx) / step) + 1)
    ny = max(2, int((maxy - miny) / step) + 1)
    gx, gy = np.meshgrid(np.linspace(minx, maxx, nx), np.linspace(miny, maxy, ny))
    pts = np.column_stack([gx.ravel(), gy.ravel()])
    inside = shapely.contains_xy(cell_poly, pts[:, 0], pts[:, 1])
    pts = pts[inside]
    if pts.shape[0] < 3:
        return None
    d = _SLOPE_PROBE_M
    nat = ctx.terrain.raw_view()                 # 地形坡度用自然地形（燒入障礙之人工陡壁非地形坡度）
    zx1 = nat.z_at(pts + np.array([d, 0.0]))
    zx0 = nat.z_at(pts - np.array([d, 0.0]))
    zy1 = nat.z_at(pts + np.array([0.0, d]))
    zy0 = nat.z_at(pts - np.array([0.0, d]))
    dzx = (zx1 - zx0) / (2 * d)
    dzy = (zy1 - zy0) / (2 * d)
    ok = np.isfinite(dzx) & np.isfinite(dzy)
    if ok.sum() < 3:
        return None
    dzx, dzy = dzx[ok], dzy[ok]
    if float(np.mean(np.hypot(dzx, dzy))) < _GRAD_EPS:
        return None
    J = np.array([[np.sum(dzx * dzx), np.sum(dzx * dzy)], [np.sum(dzx * dzy), np.sum(dzy * dzy)]])
    w, v = np.linalg.eigh(J)
    g = v[:, int(np.argmax(w))]            # 梯度主軸 (gx, gy)
    # 等高線方向 = 梯度 ⊥ = (−gy, gx)；航跡方位 θ = atan2(x, y)
    tx, ty = -g[1], g[0]
    return _mod2pi(math.atan2(tx, ty))


def direction_candidates(cell_poly: Polygon, ctx: DecomposeContext, z_ref_m: float) -> list[float]:
    """候選掃描方向 C_q（規格 §7.3）：各邊方位 ∪ {θ_minw} ∪ {β, β+π} ∪ {β±90°} ∪ {等高線主方向}。
    β = 風來向（`ctx.wind.dir_from_at(z_ref_m)`，ENU z）；無風層時 β = 0。回傳 [0, 2π) 之 rad 清單，去重 1°
    （順序 β、β+π、β±90°、θ_minw、各邊（長者優先；短於周長 2% 之邊略去——DP 簡化之角落碎邊）、等高線；
    去重保留先出現者，故與風軸相差 < 1° 之邊以風軸代表）；β 與 β+π 一律在內（即使風速 0）。"""
    beta = float(ctx.wind.dir_from_at(float(z_ref_m))) if ctx.wind is not None and ctx.wind.n_layers else 0.0
    cands: list[float] = [_mod2pi(beta), _mod2pi(beta + math.pi), _mod2pi(beta + 0.5 * math.pi), _mod2pi(beta - 0.5 * math.pi)]
    a_minw, _w = min_width_direction(cell_poly)
    cands.append(_mod2pi(a_minw))
    ring = np.asarray(cell_poly.exterior.coords, dtype=float)
    if len(ring) > 1:
        e = np.diff(ring, axis=0)
        L = np.hypot(e[:, 0], e[:, 1])
        thr = max(1e-6, 0.02 * float(L.sum()))
        for k in np.argsort(-L):
            if L[k] < thr:
                break
            cands.append(_mod2pi(math.atan2(float(e[k, 0]), float(e[k, 1]))))
    c = _contour_direction(cell_poly, ctx)
    if c is not None:
        cands.append(c)
    return _dedupe_angles(cands)


def _t_lap_and_dh(spec: PlatformSpec, auw_kg: float, r_eff: float, wind_speed: float) -> tuple[float, float, float]:
    """(T_lap, Δh_lap,climb, Δh_lap,descent)（規格 §11.6）：T = 2πR/V_a·factor(W)；Δh = V_a sinγ cosφ_max·T。"""
    va = spec.cruise_airspeed_mps
    factor = circle_time_factor(va, wind_speed)
    t_lap = TWO_PI * r_eff / va * factor
    kappa = math.cos(math.radians(spec.max_bank_angle_deg))
    dh_c = spec.climb_rate_mps(auw_kg) * kappa * t_lap
    dh_d = va * math.sin(spec.gamma_d()) * kappa * t_lap
    return t_lap, dh_c, dh_d


def proxy_time(cell_poly: Polygon, alpha_rad: float, cls: str, h_op_m: float, req_id: str,
               ctx: DecomposeContext) -> ProxyTime:
    """T_proxy(θ, c)（規格 §7.3，D15）：

    Σ_l |L_l|/2 (1/V_g^*(θ) + 1/V_g^*(θ+π)) + (n−1) t_turn + pen_slope + pen_Δz + pen_keepout；
    n = ⌈(W(θ) − (w − s))/s⌉；t_turn = min{L_Ω(s), L_U(ks)}·(1+(W/V_a)²)/V_a；單向可行胞（D34）改為
    Σ|L_l|/V_g^*(θ_ok) + 同向 racetrack 代理 Σ(πR + |L_l|)(1+(W/V_a)²)/V_a；兩向皆不可行 → inf。
    pen_slope = k_s Σ max(0, mean|∂_s z_g| − tanγ^g_c)|L_l|/V_g（有限；max 坡度 > tanγ_c → slope_exceeds）；
    pen_Δz = 上坡轉場 ⌈(k s tan(坡度⊥θ) − L_turn tanγ^g_c κ)⁺/Δh_lap⌉ T_lap（下坡以 γ_d 同式）；
    pen_keepout：迴轉帶（D38）∩ NoFly⊕b_trk 之面積比例 × 轉場時間；最小 U 型帶（R+e_ext）亦相交 → inf。
    條帶長以帶內裁剪多邊形之 u 極值 + 2 e_ext（論文 Eq.(8)(9)）向量化估計。
    """
    spec = ctx.specs[cls]
    cfg = ctx.cfg
    found = resolve_sensor(ctx, cls, req_id)
    if found is None:
        return ProxyTime(float(alpha_rad), math.inf, 0, 0.0, 0.0, 0.0, 0.0, False, 0)
    sensor, _ = found
    alpha = float(alpha_rad)
    h_op = float(h_op_m)
    w = float(sensor.swath_m(h_op))
    s = float(sensor.strip_spacing_m(h_op))
    if not (w > 0 and s > 0):
        return ProxyTime(alpha, math.inf, 0, 0.0, 0.0, 0.0, 0.0, False, 0)
    W_proj = projection_width(cell_poly, alpha)
    n = strip_count(W_proj, w, s)
    # 作業層風
    z_min_c, z_max_c = _terrain_minmax(ctx, cell_poly)
    if not math.isfinite(z_max_c):
        return ProxyTime(alpha, math.inf, n, 0.0, 0.0, 0.0, 0.0, False, 0)
    z_op = z_max_c + h_op
    wind_xy = np.asarray(ctx.wind.wind_vec(z_op), dtype=float).reshape(-1, 2)[0]
    W = float(np.hypot(wind_xy[0], wind_xy[1]))
    va = spec.cruise_airspeed_mps
    r_eff = r_eff_for_class(ctx, cls)
    e_ext = end_extension_for(ctx, cls, sensor, h_op, z_op_m=z_op)
    # 條帶幾何（帶內裁剪之 u 極值）
    origin = sweep_origin(cell_poly, alpha)
    poly_uv = _poly_to_uv(cell_poly, origin, alpha)
    v_lo = np.arange(n) * s
    v_hi = v_lo + w
    u_lo, u_hi = _band_u_extents(poly_uv, v_lo, v_hi)
    valid = np.isfinite(u_lo)
    if not valid.any():
        return ProxyTime(alpha, math.inf, n, 0.0, 0.0, 0.0, 0.0, False, 0)
    u_lo = u_lo[valid] - e_ext
    u_hi = u_hi[valid] + e_ext
    v_c = (v_lo[valid] + v_hi[valid]) * 0.5
    L = u_hi - u_lo
    n_eff = int(valid.sum())
    # 速度（規格 §4.4）
    v_cap = float(sensor.max_ground_speed_mps(h_op))
    vg = _v_g_star(np.array([alpha, alpha + math.pi]), spec, wind_xy, v_cap, cfg.v_g_min_mps)
    ok_lr, ok_rl = bool(np.isfinite(vg[0])), bool(np.isfinite(vg[1]))
    k = skip_k(s, r_eff)
    L_turn = min(omega_length(s, r_eff), uturn_length(k * s, r_eff))
    t_turn = proxy_turn_time(L_turn, va, W)
    if ok_lr and ok_rl:
        one_way = 0
        t_oper = float(np.sum(L) * 0.5 * (1.0 / vg[0] + 1.0 / vg[1]))
        t_turns = (n_eff - 1) * t_turn
        vg_ref = 2.0 / (1.0 / vg[0] + 1.0 / vg[1])
        vg_tail = float(max(vg[0], vg[1]))
    elif ok_lr or ok_rl:
        one_way = -1 if ok_lr else 1
        vg_ok = float(vg[0] if ok_lr else vg[1])
        t_oper = float(np.sum(L) / vg_ok)
        # 同向 racetrack 代理：每次轉場 (πR + |L_l|)(1+(W/V_a)²)/V_a
        t_turns = float(np.sum(math.pi * r_eff + L[:-1]) / va * (1.0 + (W / va) ** 2)) if n_eff > 1 else 0.0
        t_turn = t_turns / (n_eff - 1) if n_eff > 1 else t_turn
        vg_ref = vg_ok
        vg_tail = vg_ok
    else:
        return ProxyTime(alpha, math.inf, n, t_turn, 0.0, 0.0, 0.0, False, 0)

    # ---- 地形：沿線取樣（向量化）→ 沿線坡度、橫向坡度 ----
    t_hat, n_hat = sweep_frame(alpha)
    n_pts = np.maximum(2, np.ceil(L / _PROXY_DS_M).astype(int) + 1)
    total = int(n_pts.sum())
    strip_idx = np.repeat(np.arange(n_eff), n_pts)
    start = np.cumsum(n_pts) - n_pts
    j = np.arange(total) - np.repeat(start, n_pts)
    frac = j / np.repeat(n_pts - 1, n_pts)
    u = np.repeat(u_lo, n_pts) + frac * np.repeat(L, n_pts)
    v = np.repeat(v_c, n_pts)
    xy = origin[None, :] + u[:, None] * t_hat[None, :] + v[:, None] * n_hat[None, :]
    nat = ctx.terrain.raw_view()                 # 同上：§7.3 坡度懲罰以自然地形
    z = nat.z_at(xy)
    zp = nat.z_at(xy + _SLOPE_PROBE_M * n_hat[None, :])
    zm = nat.z_at(xy - _SLOPE_PROBE_M * n_hat[None, :])
    # 沿線坡度 |Δz/Δs|（同一條帶內相鄰樣本）
    dz = np.diff(z)
    du = np.diff(u)
    same = strip_idx[1:] == strip_idx[:-1]
    with np.errstate(invalid="ignore", divide="ignore"):
        slope_s = np.where(same & (du > 0), np.abs(dz / np.where(du > 0, du, 1.0)), np.nan)
    tan_c_lvl = spec.tan_gamma_ground("climb", vg_tail, ctx.auw_kg.get(cls, spec.reference_auw_kg), False)
    tan_c_arc = spec.tan_gamma_ground("climb", vg_tail, ctx.auw_kg.get(cls, spec.reference_auw_kg), True)
    tan_d_arc = spec.tan_gamma_ground("descent", vg_tail, ctx.auw_kg.get(cls, spec.reference_auw_kg), True)
    pen_slope = 0.0
    slope_exceeds = False
    fin = np.isfinite(slope_s)
    if fin.any():
        seg_idx = strip_idx[1:][fin]
        seg_slope = slope_s[fin]
        sums = np.bincount(seg_idx, weights=seg_slope, minlength=n_eff)
        cnts = np.bincount(seg_idx, minlength=n_eff)
        mean_slope = np.where(cnts > 0, sums / np.maximum(cnts, 1), 0.0)
        excess = np.maximum(0.0, mean_slope - (tan_c_lvl if math.isfinite(tan_c_lvl) else 0.0))
        pen_slope = float(cfg.k_slope_pen * np.sum(excess * L / vg_ref))
        slope_exceeds = bool(np.nanmax(seg_slope) > math.tan(spec.gamma_c_auw(ctx.auw_kg.get(cls, spec.reference_auw_kg))))
    # 橫向坡度（⊥ θ）平均
    with np.errstate(invalid="ignore"):
        cross = np.abs(zp - zm) / (2.0 * _SLOPE_PROBE_M)
    cross_mean = float(np.nanmean(cross)) if np.isfinite(cross).any() else 0.0
    pen_dz = 0.0
    if n_eff > 1 and cross_mean > 0:
        wsp = _wind_speed_for_r_eff(ctx, cls, z_op)
        t_lap, dh_lap_c, dh_lap_d = _t_lap_and_dh(spec, ctx.auw_kg.get(cls, spec.reference_auw_kg), r_eff, wsp)
        kappa = math.cos(math.radians(spec.max_bank_angle_deg))
        n_tr = n_eff - 1
        n_up = int(math.ceil(n_tr / 2.0))
        n_dn = n_tr - n_up
        tc = tan_c_arc if math.isfinite(tan_c_arc) else 0.0
        td = tan_d_arc if math.isfinite(tan_d_arc) else 0.0
        need_up = max(0.0, k * s * cross_mean - L_turn * tc * kappa)
        need_dn = max(0.0, max(k - 1, 1) * s * cross_mean - L_turn * td * kappa)
        laps_up = math.ceil(need_up / dh_lap_c - 1e-9) if dh_lap_c > 0 and need_up > 0 else 0
        laps_dn = math.ceil(need_dn / dh_lap_d - 1e-9) if dh_lap_d > 0 and need_dn > 0 else 0
        pen_dz = float((n_up * laps_up + n_dn * laps_dn) * t_lap)
    # ---- 迴轉帶禁區檢查（D38、D41）----
    pen_keepout = 0.0
    if ctx.nofly_buffered:
        lateral = _lateral_m(r_eff, s) if n_eff > 1 else 0.0
        tb = turn_band(cell_poly, alpha, d_turn_from_r(r_eff, s, e_ext), lateral)
        nf = shapely.union_all(list(ctx.nofly_buffered))
        # 硬性不可行只在「連最小 U 型都放不下」時成立（規格 §7.3）：某一端帽幾乎整片被禁區佔用。
        # 舊碼以 (cell ⊕ 帶) ∩ NoFly > 1e-6 m² 否決，對緊鄰禁區之胞恆成立（D38 修正 R-01）。
        if not nf.is_empty:
            nf_uv = _poly_to_uv(nf, origin, alpha)
            blk_lo, blk_hi = _end_turn_blocked(nf_uv, u_lo, u_hi, v_lo[valid], v_hi[valid], lateral, r_eff)
            if blk_lo or blk_hi:
                return ProxyTime(alpha, math.inf, n, t_turn, pen_slope, pen_dz, math.inf, slope_exceeds, one_way)
        if not nf.is_empty and tb.intersects(nf):
            ring_area = max(tb.area - cell_poly.area, _TINY_AREA_M2)
            blocked = tb.intersection(nf).area / ring_area
            pen_keepout = float(min(1.0, blocked) * max(n_eff - 1, 1) * t_turn)
    T = t_oper + t_turns + pen_slope + pen_dz + pen_keepout
    return ProxyTime(alpha, float(T), int(n), float(t_turn), float(pen_slope), float(pen_dz), float(pen_keepout),
                     bool(slope_exceeds), int(one_way))


# ---------------------------------------------------------------------------
# §7.4 條帶產生（含局部包絡回饋、禁區檢查）
# ---------------------------------------------------------------------------
def build_strips(cell: Cell, cls: str, alpha_rad: float, ctx: DecomposeContext) -> list[Strip]:
    """geometry.lay_strips + altitude.local_envelope 閉包（規格 §7.4，D3）；作業路徑 (p^L↔p^R) ⊕ b_trk 與
    NoFly⊕b_trk 相交之條帶標記 `feasible=False, infeasible_reason="nofly_intersect"`（D41，含 e_ext）。
    局部包絡不可行之條帶由 lay_strips 標記 `local_envelope_infeasible`。呼叫端以 `all(s.feasible …)` 判定方向可行。
    需要 `cell.h_op_m[cls]`（由 altitude_band 得）。"""
    spec = ctx.specs[cls]
    cfg = ctx.cfg
    found = resolve_sensor(ctx, cls, cell.payload_req_id)
    if found is None:
        return []
    sensor, _ = found
    h_op = float(cell.h_op_m[cls])
    alpha = float(alpha_rad)
    origin = sweep_origin(cell.polygon, alpha)
    z_min_c, z_max_c = _terrain_minmax(ctx, cell.polygon)
    z_op = (z_max_c if math.isfinite(z_max_c) else _z_ground_home(ctx)) + h_op
    r_eff = r_eff_for_class(ctx, cls)
    e_ext = end_extension_for(ctx, cls, sensor, h_op, z_op_m=z_op)
    h_ceil = _h_ceil_of(ctx, cls)
    local_env = make_local_envelope_fn(cls, spec, sensor, ctx.terrain, float(ctx.h_clr_m),
                                       None if not math.isfinite(h_ceil) else h_ceil, h_op, ctx.wind,
                                       float(ctx.auw_kg.get(cls, spec.reference_auw_kg)), cfg, origin, alpha, r_eff)
    strips = lay_strips(cell.polygon, cell.cell_id, cls, alpha, sensor, h_op, e_ext, local_env,
                        widen_on_lift=bool(cfg.widen_on_lift))
    if strips and ctx.nofly_buffered:
        # (path ⊕ b_trk) ∩ NoFly ≠ ∅ ⟺ path ∩ (NoFly ⊕ b_trk) ≠ ∅；ctx.nofly_buffered 已 ⊕ b_trk（與 routing 之 buffers_m=0 一致）
        nf = shapely.union_all(list(ctx.nofly_buffered))
        if not nf.is_empty:
            lines = shapely.linestrings(np.stack([np.array([s.pL for s in strips]), np.array([s.pR for s in strips])], axis=1))
            hit = shapely.intersects(lines, nf)
            for st, h in zip(strips, hit):
                if h:
                    st.feasible = False
                    st.infeasible_reason = "nofly_intersect"
    return strips


# ---------------------------------------------------------------------------
# §7.1 / §9.2 精算時間 T_single
# ---------------------------------------------------------------------------
def _skip_sequence(n: int, k: int) -> list[int]:
    """固定 skip-k 序列 (0, k, 2k, …, 1, 1+k, …)。"""
    k = max(1, int(k))
    return [i for start in range(k) for i in range(start, n, k)]


def _seq_candidates(n: int, k: int) -> list[list[int]]:
    """掃描序候選集。**第一個一定是固定 skip-k 序列** ⇒ argmin 不可能比它差。

    * 固定序列 `_skip_sequence(n, k)`：上游原行為。它把 n 條分成 k 個「類別」
      ``C_r = [r, r+k, r+2k, …]``，每類內部由小到大走完再跳下一類 ⇒ 每換一類就要
      「繞回去」一次，那 k−1 次長轉場的超出量是 ``(k−1)·max(0, n−2k)·s``，**對 n 二次**。
    * 折疊蛇形：同樣的 k 個類別，但列舉 k! 種類別順序 × 2 個起始方向，且相鄰類別
      以升／降**交替**走 ⇒ 換類時是就近接續而不是繞回原點。
    """
    k = max(1, min(int(k), max(1, n)))
    out: list[list[int]] = [_skip_sequence(n, k)]
    if n < 3 or k < 2:
        return out
    classes = [list(range(r, n, k)) for r in range(k)]
    classes = [c for c in classes if c]
    if len(classes) < 2 or len(classes) > 6:      # k! 只在小 k 列舉得起
        return out
    for order in itertools.permutations(range(len(classes))):
        for flip0 in (False, True):
            seq: list[int] = []
            asc = not flip0
            for ci in order:
                c = classes[ci]
                seq.extend(c if asc else list(reversed(c)))
                asc = not asc                      # 交替 ⇒ 換類時就近接續
            out.append(seq)
    return out


def _held_karp_path(C: np.ndarray) -> list[int] | None:
    """最小成本開放 Hamiltonian path（起訖皆自由）。n 太大或不可行 → None。"""
    n = int(C.shape[0])
    if n < 2 or n > 20:
        return None
    full = 1 << n
    dp = np.full((full, n), math.inf, dtype=float)
    par = np.full((full, n), -1, dtype=np.int16)
    for i in range(n):
        dp[1 << i, i] = 0.0
    for mask in range(full):
        row = dp[mask]
        for last in range(n):
            cur = row[last]
            if not math.isfinite(cur):
                continue
            for nxt in range(n):
                if mask >> nxt & 1:
                    continue
                nm = mask | (1 << nxt)
                v = cur + C[last, nxt]
                if v < dp[nm, nxt]:
                    dp[nm, nxt] = v
                    par[nm, nxt] = last
    end_mask = full - 1
    last = int(np.argmin(dp[end_mask]))
    if not math.isfinite(dp[end_mask, last]):
        return None
    seq = [last]
    mask = end_mask
    while par[mask, last] >= 0:
        prev = int(par[mask, last])
        mask ^= 1 << last
        last = prev
        seq.append(last)
    seq.reverse()
    return seq


def _best_sequence(n: int, k: int, C: np.ndarray, cfg: PlannerConfig) -> list[int]:
    """在候選集上取 argmin。``cfg.sweep_sequence == "fixed"`` → 直接回固定序列。

    ⚠ 候選集第一個就是固定序列，且比較用同一個成本矩陣 ⇒ **結果不可能比上游差**。
    """
    fixed = _skip_sequence(n, k)
    if getattr(cfg, "sweep_sequence", "fixed") != "search" or n < 3:
        return fixed
    cands = _seq_candidates(n, k)
    if n <= int(getattr(cfg, "sweep_exact_max_n", 9)):
        hk = _held_karp_path(C)
        if hk is not None:
            cands.append(hk)
    best = fixed
    best_c = float(C[fixed[:-1], fixed[1:]].sum())
    for seq in cands[1:]:
        c = float(C[seq[:-1], seq[1:]].sum())
        if c < best_c - 1e-9:
            best, best_c = seq, c
    return best


def _op_times(strips: Sequence[Strip], spec: PlatformSpec, sensor: Sensor | None, wind_xy: np.ndarray,
              cfg: PlannerConfig) -> np.ndarray:
    """(n,2) 作業時間 [l, idx]（idx 0 = LR、1 = RL）；不可行 → inf。"""
    n = len(strips)
    th = np.array([s.theta_LR_rad for s in strips], dtype=float)
    L = np.array([s.length_m for s in strips], dtype=float)
    caps = np.array([float(sensor.max_ground_speed_mps(s.h_min_m)) if sensor is not None else math.inf for s in strips])
    out = np.full((n, 2), np.inf)
    for idx, off in ((0, 0.0), (1, math.pi)):
        vg = _v_g_star(th + off, spec, wind_xy, caps, cfg.v_g_min_mps)
        with np.errstate(invalid="ignore", divide="ignore"):
            out[:, idx] = np.where(np.isfinite(vg), L / np.where(np.isfinite(vg), vg, 1.0), np.inf)
    return out


def exact_time(strips: list[Strip], cls: str, ctx: DecomposeContext) -> float:
    """T_single（契約 §7 明定；規格 §9.2 λ 下限與 T̃ 只用此值）：

    Σ_l (t_LR + t_RL)/2 + Σ 沿固定 skip-k 序列 (1, 1+k, …, 2, 2+k, …) 相鄰對之反向轉場 t(D)，t(D) 以 dubins
    最短 word 於作業層均勻風之時間（= dubins.path_time）在左、右兩端各算一次取平均。
    單向可行條帶（另一向 V_g^* 不存在，D34）：作業取可行向、轉場改同向 racetrack（end_state(dir) → 下一條 start_state(dir)）。
    任一條帶兩向皆不可行 → inf；空清單 → 0。"""
    if not strips:
        return 0.0
    spec = ctx.specs[cls]
    cfg = ctx.cfg
    found = resolve_sensor(ctx, cls, _req_of_strips(ctx, cls, strips))
    sensor = found[0] if found is not None else None
    z_op = float(np.mean([s.z_op_m for s in strips]))
    wind_xy = np.asarray(ctx.wind.wind_vec(z_op), dtype=float).reshape(-1, 2)[0]
    op = _op_times(strips, spec, sensor, wind_xy, cfg)
    both = np.isfinite(op).all(axis=1)
    if not np.isfinite(op).any(axis=1).all():
        return math.inf
    t_oper = float(np.where(both, op.mean(axis=1), np.nanmin(np.where(np.isfinite(op), op, np.nan), axis=1)).sum())
    n = len(strips)
    if n == 1:
        return t_oper
    r_eff = r_eff_for_class(ctx, cls, z_top_m=z_op)
    s_mean = float(np.mean([s.spacing_m for s in strips]))
    k = skip_k(s_mean, r_eff)

    def _pair_poses(a: int, b: int) -> tuple[list[np.ndarray], list[np.ndarray], list[float]]:
        """a → b 的轉場姿態對與權重（與上游逐字相同的取法，只是抽成函式）。"""
        sa, sb = strips[a], strips[b]
        if both[a] and both[b]:
            # 右端：a 以 LR 掃完 (p^R, θ) → b 自右端 RL (p^R_b, θ+π)；左端：a RL 掃完 (p^L, θ+π) → b 自左端 LR
            return ([sa.end_state(-1).as_array(), sa.end_state(+1).as_array()],
                    [sb.start_state(+1).as_array(), sb.start_state(-1).as_array()],
                    [0.5, 0.5])
        d = -1 if (np.isfinite(op[a, 0]) and np.isfinite(op[b, 0])) else 1
        return ([sa.end_state(d).as_array()], [sb.start_state(d).as_array()], [1.0])

    def _cost_of(seq: Sequence[int]) -> float:
        starts: list[np.ndarray] = []
        ends: list[np.ndarray] = []
        weights: list[float] = []
        for a, b in zip(seq[:-1], seq[1:]):
            st, en, w = _pair_poses(a, b)
            starts.extend(st); ends.extend(en); weights.extend(w)
        times, _lengths, _idx = dubins_all_times(np.array(starts), np.array(ends), r_eff,
                                                 spec.cruise_airspeed_mps, wind_xy, cfg.dtheta_int_rad)
        return float(np.dot(np.asarray(weights), times))

    if getattr(cfg, "sweep_sequence", "fixed") != "search" or n < 3:
        return t_oper + _cost_of(_skip_sequence(n, k))

    # ⚠⚠ vendored 修改 #2：估計層原本只算固定序列，而規劃層（IDP beam）是搜尋的
    #   ⇒ T_single 系統性悲觀且偏差隨 n 二次成長，再傳進掃描方向排序與 λ_floor。
    #   這裡建整張 n×n 轉場成本矩陣（一次 dubins_all_times），再對候選集取 argmin。
    #   候選集第一個就是固定序列 ⇒ **結果不可能比上游差**。
    starts: list[np.ndarray] = []
    ends: list[np.ndarray] = []
    weights: list[float] = []
    slots: list[tuple[int, int, int]] = []          # (a, b, 起始索引)
    for a in range(n):
        for b in range(n):
            if a == b:
                continue
            st, en, w = _pair_poses(a, b)
            slots.append((a, b, len(starts)))
            starts.extend(st); ends.extend(en); weights.extend(w)
    times, _lengths, _idx = dubins_all_times(np.array(starts), np.array(ends), r_eff,
                                             spec.cruise_airspeed_mps, wind_xy, cfg.dtheta_int_rad)
    tw = np.asarray(times, dtype=float) * np.asarray(weights, dtype=float)
    C = np.full((n, n), math.inf, dtype=float)
    for a, b, i0 in slots:
        nw = 2 if (both[a] and both[b]) else 1
        C[a, b] = float(tw[i0:i0 + nw].sum())
    seq = _best_sequence(n, k, C, cfg)
    return t_oper + float(C[seq[:-1], seq[1:]].sum())


def _req_of_strips(ctx: DecomposeContext, cls: str, strips: Sequence[Strip]) -> str:
    """條帶所屬感測器需求鍵：以 h_op 與 swath 反推（Strip 無 req_id）；找不到 → 該構型第一個鍵。"""
    cands = [(c, r) for (c, r) in ctx.sensors if c == cls]
    if not cands:
        return ""
    s0 = strips[0]
    for c, r in cands:
        sen = ctx.sensors[(c, r)]
        try:
            if abs(float(sen.swath_m(min(s0.h_min_m, s0.h_op_m))) - s0.width_m) < 1e-3:
                return r
        except Exception:  # pragma: no cover
            continue
    return cands[0][1]


# ---------------------------------------------------------------------------
# §7.6 適配度
# ---------------------------------------------------------------------------
def _sat(x: float) -> float:
    return float(min(1.0, max(0.0, x)))


def _rho_sym_b(ctx: DecomposeContext, rho_sym_m: dict[str, float] | None) -> float:
    if rho_sym_m and "B" in rho_sym_m and math.isfinite(float(rho_sym_m["B"])):
        return float(rho_sym_m["B"])
    spec_b = ctx.specs.get("B", PLATFORM_HERO)
    r = spec_b.symmetric_mission_radius_km
    lo = r[0] if isinstance(r, (tuple, list)) else r
    return float(lo) * 1000.0


def suitability(cell: Cell, cls: str, ctx: DecomposeContext, T_use_s: float, rho_sym_m: dict[str, float],
                *, n_uav: int | None = None) -> float:
    """S^c_q（規格 §7.6）= κ(σ,c)·Π_H 𝟙_H·[ν·s]·(0.5 + 0.5·𝟙[w_min ≥ 2R_eff]) ∈ [0,1]。

    s_width = 1 − frac(W_q/s_c)s_c/W_q；s_area_A = sat(A_q/(5 km·s_A))、s_area_B = 1 − s_area_A；
    s_dist_A = sat(d_q/ρ^sym_B)、s_dist_B = 1 − s_dist_A；s_turn = (1 − [(n−1)t_turn + pen_Δz]/T_q)(1 − ½ sat(d_turn/L_q))；
    s_endur = 1 − T_q/T_use；s_gsd：rgb/both 需求時 B 為 1、A 依機上 RGB 之 GSD 達成率（無 RGB → κ=0）。
    硬限制：H2（胞 ⊕ d_turn ⊂ Disc(home, ρ^sym_c)）、H4（能量，λ = min(n_uav, n_q) 濾網；`n_uav` None → n_q）、
    H5（T 有限）、H6（高度帶 = cell.feasible[cls]）；H3 需時序、H7 只影響條帶數，於此皆視為通過。
    T_q = cell.T_single_s[cls]（未鋪條帶 → 0 分）。"""
    if cls not in ctx.specs:
        return 0.0
    spec = ctx.specs[cls]
    cfg = ctx.cfg
    kinds = ctx.available_kinds.get(cls, frozenset())
    if not class_can_serve(cell.sensor_req, cls, kinds):
        return 0.0
    found = resolve_sensor(ctx, cls, cell.payload_req_id)
    if found is None:
        return 0.0
    sensor, _ = found
    if not cell.feasible.get(cls, False):
        return 0.0
    T_q = float(cell.T_single_s.get(cls, math.inf))
    n_q = int(cell.n_strips.get(cls, 0))
    if not math.isfinite(T_q) or T_q <= 0 or n_q <= 0:
        return 0.0
    h_op = float(cell.h_op_m.get(cls, spec.min_fw_operation_agl_m))
    alpha = float(cell.alpha_rad.get(cls, cell.min_width_dir_rad))
    s_c = float(sensor.strip_spacing_m(h_op))
    r_eff = r_eff_for_class(ctx, cls)
    d_turn = float(cell.d_turn_m.get(cls, d_turn_from_r(r_eff, s_c, 0.0)))
    # ---- 硬限制 ----
    if rho_sym_m and cls in rho_sym_m and math.isfinite(float(rho_sym_m[cls])):
        far = float(Point(ctx.home_xy).hausdorff_distance(cell.polygon)) + d_turn
        if far > float(rho_sym_m[cls]) + 1e-6:
            return 0.0
    lam = max(1, min(n_q, int(n_uav))) if n_uav is not None else max(1, n_q)
    if getattr(cfg, "enforce_energy", True):
        m: EnergyModel | None = ctx.models_rep.get(cls)
        if m is not None:
            p = m.p_cruise_w() * m.k_unc / 3600.0
            d_home = float(cell.dist_home_m)
            w_max = ctx.wind.max_speed() if ctx.wind is not None else 0.0
            vg_worst = max(cfg.v_g_min_mps, spec.cruise_airspeed_mps - w_max)
            e_io = 2.0 * p * d_home / vg_worst
            e_mis = p * T_q / lam
            if e_io + e_mis > float(m.e_usable_wh()) + 1e-9:
                return 0.0
    # ---- 分數 ----
    W_q = projection_width(cell.polygon, alpha)
    if W_q <= 0:
        return 0.0
    ratio = W_q / s_c
    s_width = 1.0 - (ratio - math.floor(ratio)) * s_c / W_q
    # 面積：A* = 5 km × s_A
    s_A = s_c
    if "A" in ctx.specs:
        fa = resolve_sensor(ctx, "A", cell.payload_req_id)
        if fa is not None:
            h_a = float(cell.h_op_m.get("A", ctx.specs["A"].min_fw_operation_agl_m))
            s_A = float(fa[0].strip_spacing_m(h_a))
    s_area_a = _sat(cell.area_m2 / (5000.0 * s_A))
    s_area = s_area_a if cls == "A" else 1.0 - s_area_a
    s_dist_a = _sat(cell.dist_home_m / _rho_sym_b(ctx, rho_sym_m))
    s_dist = s_dist_a if cls == "A" else 1.0 - s_dist_a
    px = proxy_time(cell.polygon, alpha, cls, h_op, cell.payload_req_id, ctx)
    L_q = float(np.mean([s.length_m for s in cell.strips.get(cls, [])])) if cell.strips.get(cls) else max(W_q, 1.0)
    turn_part = ((n_q - 1) * px.t_turn_s + px.pen_dz_s) / T_q if math.isfinite(px.t_turn_s) else 1.0
    s_turn = _sat(1.0 - turn_part) * (1.0 - 0.5 * _sat(d_turn / max(L_q, 1e-6)))
    s_endur = _sat(1.0 - T_q / T_use_s) if math.isfinite(T_use_s) and T_use_s > 0 else 1.0
    if cell.sensor_req == "rgb" or (cell.sensor_req == "both" and sensor.kind == "rgb"):
        if cls == "B":
            s_gsd = 1.0
        else:
            fr = ctx.sensors.get(("A", cell.payload_req_id + RGB_REQ_SUFFIX)) or (
                ctx.sensors.get(("A", cell.payload_req_id)) if sensor.kind == "rgb" else None)
            if fr is None:
                return 0.0
            g = fr.gsd_m(h_op)
            req = getattr(getattr(fr, "r", None), "gsd_required_m", None)
            s_gsd = 1.0 if (g is None or req is None or g <= 0) else _sat(float(req) / float(g))
    else:
        s_gsd = 1.0
    nu = cfg.nu
    score = (nu[0] * s_width + nu[1] * s_area + nu[2] * s_dist + nu[3] * s_turn + nu[4] * s_endur + nu[5] * s_gsd)
    score *= 0.5 + 0.5 * (1.0 if cell.min_width_m >= 2.0 * r_eff else 0.0)
    return _sat(score)


# ---------------------------------------------------------------------------
# §7.1 分解總流程
# ---------------------------------------------------------------------------
def _area_polygon_enu(area: AreaSpec, ctx: DecomposeContext) -> Polygon | MultiPolygon:
    """區域 WGS84 (lat, lon) → ENU（以 frame.h0 投影；與 context.areas_enu 差 < 1 m）；make_valid。"""
    frame = ctx.frame
    h0 = float(getattr(frame, "h0_msl_m", 0.0))

    def ring(pts: Sequence[tuple[float, float]]) -> np.ndarray:
        arr = np.asarray(pts, dtype=float).reshape(-1, 2)
        return frame.to_enu(arr[:, 0], arr[:, 1], h0)[:, :2]

    shell = ring(area.polygon_wgs84)
    holes = [ring(h) for h in area.holes_wgs84]
    poly = Polygon(shell, holes)
    if not poly.is_valid:
        poly = shapely.make_valid(poly)
    parts = _polys_of(poly, 1.0)
    if not parts:
        raise ValueError(f"area {area.area_id!r}: degenerate polygon")
    return parts[0] if len(parts) == 1 else MultiPolygon(parts)


def _class_sensor_params(ctx: DecomposeContext, area: AreaSpec, classes: Sequence[str]) -> dict[str, tuple[Sensor, float, float, float]]:
    """cls → (sensor, h_op, w, s)（以區域層級之高度帶初估，供 ε_dp / s_ref / τ_merge）。"""
    out: dict[str, tuple[Sensor, float, float, float]] = {}
    for cls in classes:
        if cls not in ctx.specs:
            continue
        if not class_can_serve(area.sensor_req, cls, ctx.available_kinds.get(cls, frozenset())):
            continue
        found = resolve_sensor(ctx, cls, area.payload_req_id)
        if found is None:
            continue
        sensor, _ = found
        spec = ctx.specs[cls]
        h_minfw = float(spec.min_fw_operation_agl_m)
        h_ceil = _h_ceil_of(ctx, cls)
        h_max = float(sensor.agl_window()[1])
        h_t = float(sensor.h_target_m(h_minfw, ctx.cfg.dh_band_min_m, ctx.cfg.k_gsd_relax))
        h_op = max(h_minfw, min(h_t, min(h_max, h_ceil)))
        out[cls] = (sensor, h_op, float(sensor.swath_m(h_op)), float(sensor.strip_spacing_m(h_op)))
    return out


def area_edge_deltas(ctx: DecomposeContext, params: Sequence[tuple[Sensor, float, float, float]]
                     ) -> tuple[float, float, float | None]:
    """`decompose_area` 的外擴量 ``(δ_edge, δ_cell, tie)``；`params` 為 `_class_sensor_params` 的值 (sensor, h_op, w, s)。

    * δ_edge = cfg.delta_edge_resolved(δ_w)，δ_w = max_c w/h·σ_z：區域外緣（simplify_outward）用，**建圖模式也不變**。
    * δ_cell：凸分割胞外擴；未開建圖模式 = δ_edge（舊行為）；seam_tie → max(δ_edge, tie/2)——兩胞以共用邊為界各外擴
      一半，而足跡 ⊇ 外擴後的胞（lay_strips 後置斷言）⇒ 相鄰胞足跡重疊 ≥ tie。胞外擴被 F 截住，不會長出區域外緣。
    * tie = 建圖接縫重疊目標 max_c(w − s)（:func:`seam_tie_overlap_m`）；未開 → None。相鄰子區（別的 AreaSpec）的
      接縫由 `decompose_area(seam_neighbors=…)` 往鄰區各長 tie/2 處理。"""
    sigma_z = getattr(ctx.terrain, "sigma_z_m", None) or 0.0
    delta_w = max([w / h * float(sigma_z) for (_s, h, w, _sp) in params], default=0.0)
    delta_edge = float(ctx.cfg.delta_edge_resolved(delta_w))
    tie = seam_tie_overlap_m(ctx, [(w, sp) for (_s, _h, w, sp) in params])
    delta_cell = delta_edge if tie is None else max(delta_edge, 0.5 * tie)
    return delta_edge, delta_cell, tie


def _new_cell(cell_id: str, area: AreaSpec | Cell, poly: Polygon, ctx: DecomposeContext) -> Cell:
    a_minw, w_min = min_width_direction(poly)
    d_home = float(np.hypot(*(np.asarray(poly.centroid.coords[0]) - np.asarray(ctx.home_xy, dtype=float))))
    return Cell(
        cell_id=cell_id, area_id=area.area_id, polygon=poly, sensor_req=area.sensor_req,
        payload_req_id=area.payload_req_id, priority=float(area.priority), time_window_s=area.time_window_s,
        area_m2=float(poly.area), min_width_m=float(w_min), min_width_dir_rad=float(a_minw), dist_home_m=d_home,
        sweep_origin={}, h_op_m={}, feasible={}, infeasible_reason={}, alpha_rad={}, strips={}, alt_dirs={},
        fit={}, n_strips={}, T_single_s={}, d_turn_m={}, parent_cell_id=None, residual_of=None, k_A=None,
        slope_exceeds={},
    )


def _solve_class(cell: Cell, cls: str, ctx: DecomposeContext, *, rho_sym_m: dict[str, float] | None,
                 t_use_s: float | None, n_uav: int | None, alpha_fixed: float | None = None) -> None:
    """每胞每構型：高度帶 → 方向候選 → 代理前三名 → 精鋪 + 禁區檢查 → exact_time → 最佳方向、fit。就地寫入 cell。"""
    cfg = ctx.cfg
    kinds = ctx.available_kinds.get(cls, frozenset())
    cell.feasible[cls] = False
    cell.infeasible_reason[cls] = ""
    cell.fit[cls] = 0.0
    cell.n_strips[cls] = 0
    cell.T_single_s[cls] = math.inf
    cell.strips[cls] = []
    cell.alt_dirs[cls] = []
    cell.slope_exceeds[cls] = False
    if cls not in ctx.specs:
        cell.infeasible_reason[cls] = "no_fleet"
        return
    if not class_can_serve(cell.sensor_req, cls, kinds):
        cell.infeasible_reason[cls] = "sensor_incompatible"
        return
    found = resolve_sensor(ctx, cls, cell.payload_req_id)
    if found is None:
        cell.infeasible_reason[cls] = "no_sensor"
        return
    sensor, _ = found
    band = altitude_band(cell.polygon, cls, cell.payload_req_id, ctx)
    cell.h_op_m[cls] = band.h_op_m
    s_c = float(sensor.strip_spacing_m(band.h_op_m))
    z_min_c, z_max_c = _terrain_minmax(ctx, cell.polygon)
    z_op = (z_max_c if math.isfinite(z_max_c) else _z_ground_home(ctx)) + band.h_op_m
    e_ext = end_extension_for(ctx, cls, sensor, band.h_op_m, z_op_m=z_op)
    cell.d_turn_m[cls] = d_turn_m(cls, s_c, e_ext, ctx)
    cell.sweep_origin[cls] = sweep_origin(cell.polygon, float(cell.min_width_dir_rad))
    if not band.feasible:
        cell.infeasible_reason[cls] = band.reason
        return
    # 方向候選與代理
    if alpha_fixed is not None:
        cands = [float(alpha_fixed)]
    else:
        cands = _dedupe_angles(direction_candidates(cell.polygon, ctx, z_op), modulo=math.pi)
    proxies = [proxy_time(cell.polygon, a, cls, band.h_op_m, cell.payload_req_id, ctx) for a in cands]
    proxies = [p for p in proxies if math.isfinite(p.T_s)]
    if not proxies:
        cell.infeasible_reason[cls] = "no_feasible_direction"
        return
    proxies.sort(key=lambda p: p.T_s)
    results: list[tuple[int, float, float, list[Strip], ProxyTime]] = []
    for p in proxies[:_N_TOP_DIRS]:
        try:
            strips = build_strips(cell, cls, p.alpha_rad, ctx)
        except AssertionError:
            continue
        if not strips:
            continue
        # D41 修正 R-01：僅「與禁區相交」之個別條帶剔除（其餘條帶仍可飛），不再整個方向作廢；
        # 其他不可行原因（局部包絡）仍使該方向作廢。
        keep = [s for s in strips if s.feasible]
        n_drop = sum(1 for s in strips if (not s.feasible) and s.infeasible_reason == "nofly_intersect")
        if not keep or (len(strips) - len(keep)) != n_drop:
            continue
        T = exact_time(keep, cls, ctx)
        if not math.isfinite(T):
            continue
        results.append((n_drop, p.alpha_rad, T, keep, p))
    if not results:
        cell.infeasible_reason[cls] = "no_feasible_direction"
        return
    results.sort(key=lambda r: (r[0], r[2]))
    _n_drop, alpha_best, T_best, strips_best, p_best = results[0]
    cell.alpha_rad[cls] = alpha_best
    cell.strips[cls] = strips_best
    cell.sweep_origin[cls] = np.array(strips_best[0].sweep_origin, dtype=float, copy=True)
    cell.alt_dirs[cls] = [(a, T) for _d, a, T, _s, _p in results]
    cell.n_strips[cls] = len(strips_best)
    cell.T_single_s[cls] = T_best
    cell.slope_exceeds[cls] = bool(p_best.slope_exceeds)
    cell.feasible[cls] = True
    T_use = t_use_s_for(ctx, cls) if t_use_s is None else float(t_use_s)
    cell.fit[cls] = suitability(cell, cls, ctx, T_use, rho_sym_m or {}, n_uav=n_uav)


def decompose_area(area: AreaSpec, ctx: DecomposeContext, fleet_classes: set[str], *,
                   polygon_enu: Polygon | MultiPolygon | None = None, rho_sym_m: dict[str, float] | None = None,
                   t_use_s: dict[str, float] | None = None, n_uav: dict[str, int] | None = None,
                   seam_neighbors: shapely.Geometry | None = None) -> list[Cell]:
    """DECOMPOSE（規格 §7.1）：

    P_area = simplify_outward(Z, δ_edge, ε_dp)（D31）→ F = P_area \\ ∪(NoFly⊕b_trk)（D41）→ 逐一分量
    convex_partition（s_ref = min_c s_c、τ_merge = merge_tau_factor·min_c w_c；`convexify_whole` → 凸包）→
    Q_q ← (Q_q ⊕ disc(δ_cell)) ∩ F → 每胞每構型（fleet_classes ∩ 感測相容）：altitude_band、候選方向、
    proxy_time 前三名精鋪（build_strips + 禁區檢查）、exact_time、θ* = argmin、fit = suitability。
    關鍵字：`polygon_enu`（直接給 ctx.areas_enu 之多邊形）、`rho_sym_m`／`t_use_s`／`n_uav`（cls →；供 suitability）。
    建圖模式 seam_tie（:func:`area_edge_deltas`）：δ_cell = max(δ_edge, tie/2)；`seam_neighbors`（其他區域之聯集，
    plan_mission 給）時 Z ← Z ∪ ((Z ⊕ tie/2) ∩ 鄰區) ⇒ 相鄰子區各往對方長 tie/2，接縫重疊 ≥ tie；外緣仍只外擴 δ_edge。
    未開建圖模式時 δ_cell = δ_edge、`seam_neighbors` 不使用（舊行為逐位元不變）。
    後置條件：∪ cell.polygon ⊇ Z \\ ∪(NoFly⊕b_trk)。"""
    cfg = ctx.cfg
    classes = sorted(c for c in fleet_classes if c in ctx.specs)
    params = _class_sensor_params(ctx, area, classes)
    poly_in = polygon_enu if polygon_enu is not None else _area_polygon_enu(area, ctx)
    # δ_edge（含 DEM 寬度誤差 δ_w = w σ_z/h）、ε_dp、s_ref、τ_merge；δ_cell／tie 見 area_edge_deltas
    delta_edge, delta_cell, tie = area_edge_deltas(ctx, list(params.values()))
    if tie is not None and tie > 0.0 and seam_neighbors is not None and not seam_neighbors.is_empty:
        grow = shapely.intersection(shapely.buffer(poly_in, 0.5 * tie), seam_neighbors)
        poly_in = shapely.union_all([poly_in, *_polys_of(grow, 1.0)])
    s_list = [sp for (_s, _h, _w, sp) in params.values() if sp > 0]
    w_list = [w for (_s, _h, w, _sp) in params.values() if w > 0]
    s_ref = min(s_list) if s_list else max(delta_edge, 1.0)
    w_ref = min(w_list) if w_list else s_ref
    eps_dp = float(cfg.dp_eps_factor) * s_ref
    eps_dp = min(eps_dp, 0.99 * delta_edge) if delta_edge > 0 else eps_dp
    tau_merge = float(cfg.merge_tau_factor) * w_ref

    areas = _polys_of(poly_in, 1.0)
    p_parts: list[Polygon] = []
    for z in areas:
        p_parts.extend(_polys_of(simplify_outward(z, delta_edge, eps_dp), 1.0))
    p_area = shapely.union_all(p_parts) if p_parts else Polygon()
    nofly = shapely.union_all(list(ctx.nofly_buffered)) if ctx.nofly_buffered else None
    F = p_area.difference(nofly) if nofly is not None and not nofly.is_empty else p_area
    if bool(getattr(cfg, "convexify_whole", False)):
        hull = shapely.union_all([p.convex_hull for p in _polys_of(F, 1.0)])
        F = hull.difference(nofly) if nofly is not None and not nofly.is_empty else hull
    shapely.prepare(F)

    cells: list[Cell] = []
    q = 0
    for P in _polys_of(F, 1.0):
        parts = convex_partition(P, s_ref, tau_merge, cfg.lambda_cut, cfg.lambda_n)
        for Q in parts:
            grown = Q.buffer(delta_cell).intersection(F)
            pieces = _polys_of(grown, 1.0)
            if not pieces:
                continue
            if len(pieces) == 1:
                Qq = pieces[0]
            else:
                rp = Q.representative_point()
                inside = [p for p in pieces if p.contains(rp)]
                Qq = inside[0] if inside else max(pieces, key=lambda p: p.area)
            q += 1
            cell = _new_cell(f"{area.area_id}_{q}", area, Qq, ctx)
            for cls in classes:
                _solve_class(cell, cls, ctx, rho_sym_m=rho_sym_m,
                             t_use_s=(t_use_s or {}).get(cls), n_uav=(n_uav or {}).get(cls))
            cells.append(cell)
    return cells


# ---------------------------------------------------------------------------
# §7.5 殘帶候選（D17）
# ---------------------------------------------------------------------------
def _subcell(parent: Cell, cell_id: str, poly: Polygon, ctx: DecomposeContext, *, residual_of: str | None, k_A: int) -> Cell:
    c = _new_cell(cell_id, parent, poly, ctx)
    c.parent_cell_id = parent.cell_id
    c.residual_of = residual_of
    c.k_A = k_A
    return c


def _estimate_b_time(cell: Cell, ctx: DecomposeContext, width_m: float) -> tuple[int, float]:
    """殘帶寬 width 之 B 條帶數與代理時間（以 B 現有條帶之平均作業時間與 t_turn 估）。"""
    strips_b = cell.strips.get("B", [])
    if not strips_b or width_m <= 0:
        return 0, 0.0
    w_b = float(np.mean([s.width_m for s in strips_b]))
    s_b = float(np.mean([s.spacing_m for s in strips_b]))
    n_b = strip_count(width_m, w_b, s_b)
    T_b_full = float(cell.T_single_s.get("B", math.inf))
    n_full = len(strips_b)
    per = T_b_full / n_full if math.isfinite(T_b_full) and n_full > 0 else math.inf
    return n_b, n_b * per


def residual_candidates(cell: Cell, ctx: DecomposeContext, k_list: list[int] | None = None,
                        *, rho_sym_m: dict[str, float] | None = None, t_use_s: dict[str, float] | None = None,
                        n_uav: dict[str, int] | None = None) -> list[tuple[int, Cell, Cell | None]]:
    """殘帶候選對（規格 §7.5／§9.5，D17；僅 σ = any 且 A、B 皆可行之胞）：[(k_A, A 子胞含前 k 條, B 殘帶子胞)]。

    A 鋪 k 條後頂緣 v_top = strips_A[k−1].v_hi；A 子胞 = clip(Q, v ≤ v_top)（保留原 A 條帶 1..k）；
    B 殘帶子胞 = residual_subcell(Q, θ_A, origin_A, v_top, δ_edge)（只在 v 方向外擴 δ_edge），由 B 以自身條帶重鋪
    （`_solve_class`，方向候選重新評分）。k_list None → {0, ⌈n_A/2⌉, n_A} ∪ 估計 makespan max(Ĉ_A(k), Ĉ_B(k)) 前 3 名
    （Ĉ_A = exact_time(前 k 條)、Ĉ_B = 殘帶條帶數 × B 平均條帶時間）。
    k_A == n_A → (n_A, cell, None)；k_A == 0 → (0, cell, cell)（A 子胞欄位以原胞代之，契約外，見 CONTRACT_DEVIATIONS）。
    子胞 id 為 ``{cell_id}_A_k{k}`` / ``{cell_id}_B_k{k}``（B 殘帶條帶 id 因而為 ``{cell_id}_B_k{k}_B_{l}``），
    使不同候選 k 的條帶在 CostCache 中不會互相覆蓋。結果依 k_A 遞減排序。"""
    if cell.sensor_req != "any":
        return []
    strips_a = cell.strips.get("A", [])
    if not cell.feasible.get("A", False) or not cell.feasible.get("B", False) or not strips_a or not cell.strips.get("B"):
        return []
    n_a = len(strips_a)
    alpha_a = float(cell.alpha_rad["A"])
    origin_a = np.asarray(strips_a[0].sweep_origin, dtype=float)
    delta_edge = float(ctx.cfg.delta_edge_resolved())
    # 建圖模式 seam_tie：A 子胞的足跡剛好止於 v_top，B 殘帶只往 A 側單邊外擴 ⇒ 要外擴整個 w − s，
    #   B 第一條帶才會落在「A 的第 k+1 條」的位置（接縫重疊 = 帶內相鄰重疊）。
    tie = seam_tie_overlap_m(ctx, [(s.width_m, s.spacing_m) for s in list(strips_a) + list(cell.strips.get("B", []))])
    if tie is not None:
        delta_edge = max(delta_edge, tie)
    uv = to_sweep_coords(np.asarray(cell.polygon.exterior.coords, dtype=float), origin_a, alpha_a)
    v_min_q, v_max_q = float(uv[:, 1].min()), float(uv[:, 1].max())

    def v_top_of(k: int) -> float:
        return float(strips_a[k - 1].v_hi_m)

    if k_list is None:
        ks = {0, int(math.ceil(n_a / 2.0)), n_a}
        est: list[tuple[float, int]] = []
        for k in range(0, n_a + 1):
            c_a = exact_time(list(strips_a[:k]), "A", ctx) if k > 0 else 0.0
            width = (v_max_q - (v_top_of(k) - delta_edge)) if 0 < k < n_a else (v_max_q - v_min_q if k == 0 else 0.0)
            _nb, c_b = _estimate_b_time(cell, ctx, width)
            est.append((max(c_a, c_b), k))
        est.sort()
        ks |= {k for _c, k in est[:3]}
        k_list = sorted(ks, reverse=True)
    out: list[tuple[int, Cell, Cell | None]] = []
    for k in sorted({int(k) for k in k_list}, reverse=True):
        if k < 0 or k > n_a:
            continue
        if k == n_a:
            out.append((n_a, cell, None))
            continue
        if k == 0:
            out.append((0, cell, cell))
            continue
        v_top = v_top_of(k)
        poly_a = clip_band(cell.polygon, origin_a, alpha_a, v_min_q - 1.0, v_top)
        poly_b = residual_subcell(cell.polygon, alpha_a, origin_a, v_top, delta_edge)
        parts_a = _polys_of(poly_a, 1.0)
        parts_b = _polys_of(poly_b, 1.0)
        if not parts_a or not parts_b:
            continue
        pa = parts_a[0] if len(parts_a) == 1 else max(parts_a, key=lambda p: p.area)
        pb = parts_b[0] if len(parts_b) == 1 else max(parts_b, key=lambda p: p.area)
        # 子胞 id 必須含 k：同一母胞的各候選 k 會各自鋪 B 條帶，若 id 相同，allocate 以 strip_id 加入
        # CostCache 時會互相覆蓋（不同 k 的殘帶條帶 y 位置不同），B 機就會飛到錯的條帶而留下未掃的帶。
        cell_a = _subcell(cell, f"{cell.cell_id}_A_k{k}", pa, ctx, residual_of=None, k_A=k)
        sub = [s for s in strips_a[:k]]
        cell_a.h_op_m["A"] = cell.h_op_m["A"]
        cell_a.d_turn_m["A"] = cell.d_turn_m["A"]
        cell_a.alpha_rad["A"] = alpha_a
        cell_a.sweep_origin["A"] = origin_a.copy()
        cell_a.strips["A"] = sub
        cell_a.n_strips["A"] = k
        cell_a.T_single_s["A"] = exact_time(sub, "A", ctx)
        cell_a.feasible["A"] = math.isfinite(cell_a.T_single_s["A"])
        cell_a.infeasible_reason["A"] = ""
        cell_a.alt_dirs["A"] = [(alpha_a, cell_a.T_single_s["A"])]
        cell_a.slope_exceeds["A"] = bool(cell.slope_exceeds.get("A", False))
        cell_a.feasible["B"] = False
        cell_a.infeasible_reason["B"] = "residual_split"
        cell_a.fit["B"] = 0.0
        cell_a.fit["A"] = suitability(cell_a, "A", ctx, t_use_s_for(ctx, "A") if not t_use_s else t_use_s.get("A", math.inf),
                                      rho_sym_m or {}, n_uav=(n_uav or {}).get("A"))
        cell_b = _subcell(cell, f"{cell.cell_id}_B_k{k}", pb, ctx, residual_of=cell.cell_id, k_A=k)
        _solve_class(cell_b, "B", ctx, rho_sym_m=rho_sym_m, t_use_s=(t_use_s or {}).get("B"), n_uav=(n_uav or {}).get("B"))
        cell_b.feasible["A"] = False
        cell_b.infeasible_reason["A"] = "residual_split"
        cell_b.fit["A"] = 0.0
        if not cell_b.feasible.get("B", False):
            continue
        out.append((k, cell_a, cell_b))
    return out


# ---------------------------------------------------------------------------
# 建圖模式：交叉航帶（`MappingModeConfig.cross_strips`；設計見 mission.MappingModeConfig docstring）
# ---------------------------------------------------------------------------
class CrossStripRefused(ValueError):
    """交叉航帶的使用者設定在某區塊不可行（高度超出高度帶／低於最低固定翼高度）⇒ 整份計畫 NO_GO，不夾限。"""


@dataclass(slots=True)
class CrossSet:
    """一架在一個區塊（該機在母胞實際飛的主條帶）的一組交叉航帶與其有效值。

    ``block``：該機在母胞負責的區域（主條帶足跡 ∩ 胞多邊形；整胞自己飛時 = 胞）；``region``：中央橫帶
    （面積 = ``fraction_effective`` × 區塊面積）；``imaged_area_m2``：交叉帶足跡 ∩ 區塊。
    """
    uav_id: str
    cell_id: str
    cls: str
    strips: list[Strip]
    alpha_main_rad: float
    beta_rad: float
    h_main_m: float
    h_cross_m: float
    block: shapely.Geometry
    region: shapely.Geometry
    block_area_m2: float
    region_area_m2: float
    imaged_area_m2: float
    fraction_target: float
    widened_to_two: bool
    width_m: float
    spacing_m: float
    e_ext_m: float
    d_turn_m: float
    lateral_m: float
    notes: list[str]

    @property
    def fraction_effective(self) -> float:
        """中央橫帶面積 / 區塊面積（放寬到 2 條時 > fraction_target）。"""
        return self.region_area_m2 / self.block_area_m2 if self.block_area_m2 > 0 else 0.0

    @property
    def strip_ids(self) -> list[str]:
        return [s.strip_id for s in self.strips]


def cross_altitude_check(cell: Cell, cls: str, h_main_m: float, alt_offset_m: float,
                         ctx: DecomposeContext) -> tuple[float, tuple[float, float]]:
    """交叉帶作業高度 h_x = h_main + offset，必須在該區塊的作業高度帶 [h_lo, h_hi] 內且 ≥ 最低固定翼高度，否則
    :class:`CrossStripRefused`（訊息寫出 h_x 與帶界；**不夾限**）。回傳 (h_x, (h_lo, h_hi))。

    h_lo = `altitude_band(...).h_lo_m` = max(最低固定翼, 地形淨空)；h_hi = min(h_ceil, 感測器 AGL 上限)
    ＝主條帶 h_op 被夾的同一個上限（`altitude_band` 的 hi_cap）。⚠ 不用 `AltitudeBand.h_hi_m`（= min(h_ceil − 起伏,
    感測器上限)，給 `altitude.layering` 用）：主條帶本身就飛在 h_op 而不扣起伏 —— 2×2 km 平地情境 ENU 地球曲率
    使 Ω 內「起伏」0.75 m ⇒ h_hi_m 119.25 < 主條帶 120，用它會連「同高度交叉帶」都拒絕。地形使某條交叉帶的
    局部包絡不可行時，由 `lay_strips` 標記、`build_cross_set` 略過該區塊並列警告。"""
    band = altitude_band(cell.polygon, cls, cell.payload_req_id, ctx)
    h_x = float(h_main_m) + float(alt_offset_m)
    h_minfw = float(ctx.specs[cls].min_fw_operation_agl_m)
    found = resolve_sensor(ctx, cls, cell.payload_req_id)
    h_max = float(found[0].agl_window()[1]) if found is not None else math.inf
    h_ceil = _h_ceil_of(ctx, cls)
    h_lo = max(float(band.h_lo_m), h_minfw)
    h_hi = min(h_ceil, h_max)
    tol = 1e-6
    problems: list[str] = []
    if h_x < h_minfw - tol:
        problems.append(f"低於構型 {cls} 最低固定翼作業高度 {h_minfw:g} m")
    if h_x < float(band.h_lo_m) - tol and float(band.h_lo_m) > h_minfw + tol:
        problems.append(f"低於高度帶下緣 {band.h_lo_m:g} m（地形淨空 {float(ctx.h_clr_m):g} m）")
    if h_x > h_hi + tol:
        problems.append(f"高於高度帶上緣 {h_hi:g} m（min(h_ceil {h_ceil:g}, 感測器上限 {h_max:g})）")
    if problems:
        raise CrossStripRefused(
            f"cross_alt_out_of_band({cell.cell_id}.{cls}: 交叉航帶高度 = 主條帶 {float(h_main_m):g} m "
            f"{float(alt_offset_m):+g} m = {h_x:g} m，" + "；".join(problems)
            + f"；可用範圍 [{h_lo:g}, {h_hi:g}] m ⇒ 不夾限、整份計畫不規劃)")
    return h_x, (h_lo, h_hi)


def cross_central_band(block: shapely.Geometry, beta_rad: float, fraction: float,
                       min_height_m: float = 0.0) -> tuple[shapely.Geometry | None, float]:
    """區塊的中央橫帶（沿 β 貫穿區塊、在 n̂(β) 上置中於形心；碰到區塊邊界就整段平移不截短）：
    面積 = fraction × 區塊面積（二分法）；``min_height_m`` > 0 時橫帶高度至少此值（不超過區塊寬）。
    回傳 (橫帶幾何或 None, 橫帶高度 H)。"""
    origin = sweep_origin(block, beta_rad)
    uv = to_sweep_coords(np.asarray(shapely.get_coordinates(block), dtype=float), origin, beta_rad)
    V = float(uv[:, 1].max())
    if not V > 0:
        return None, 0.0
    v_c = float(to_sweep_coords(np.asarray(block.centroid.coords[0], dtype=float), origin, beta_rad)[1])
    A = float(block.area)

    def window(H: float) -> tuple[float, float]:
        H = min(max(H, 0.0), V)
        lo = min(max(v_c - H / 2.0, 0.0), V - H)
        return lo, lo + H

    def area(H: float) -> float:
        lo, hi = window(H)
        g = clip_band(block, origin, beta_rad, lo, hi)
        return 0.0 if g is None else float(g.area)

    if fraction >= 1.0 - 1e-12:
        H = V
    else:
        target = float(fraction) * A
        lo_h, hi_h = 0.0, V
        for _ in range(200):
            mid = 0.5 * (lo_h + hi_h)
            if area(mid) < target:
                lo_h = mid
            else:
                hi_h = mid
            if hi_h - lo_h <= 1e-9 * max(1.0, V):
                break
        H = hi_h
    H = min(max(H, float(min_height_m)), V)
    if H >= V - 1e-9:
        return block, V
    lo, hi = window(H)
    return clip_band(block, origin, beta_rad, lo, hi), H


def build_cross_set(parent: Cell, cls: str, main_strips: Sequence[Strip], ctx: DecomposeContext, *, uav_id: str,
                    fraction: float, alt_offset_m: float, angle_offset_deg: float,
                    index_base: int = 0) -> CrossSet | str:
    """一架在一個區塊的交叉航帶（建圖模式）。成功 → :class:`CrossSet`；區塊放不下 2 條／碰禁區／局部包絡不可行
    → 略過原因字串（呼叫端列警告）；高度不合法 → :class:`CrossStripRefused`。

    交叉帶以 `build_strips`（= `geometry.lay_strips` + 局部包絡 + 禁區檢查，e_ext 含引入段）鋪在中央橫帶上，
    方向 β = 主條帶方向 + angle_offset、高度 h_x = 主條帶 h_op + alt_offset；條帶 id 改為
    `geometry.cross_strip_id(...)`、`index` = index_base + k、`cell_id` 仍是母胞。"""
    from dataclasses import replace as _replace
    from .geometry import cross_strip_id, strips_union_footprint

    tag = f"{parent.cell_id}@{uav_id}"
    if not main_strips:
        return f"{tag}: 沒有主條帶"
    alpha = float(main_strips[0].theta_LR_rad) % math.pi
    beta = (alpha + math.radians(float(angle_offset_deg))) % math.pi
    h_main = float(main_strips[0].h_op_m)
    h_x, _band = cross_altitude_check(parent, cls, h_main, alt_offset_m, ctx)
    found = resolve_sensor(ctx, cls, parent.payload_req_id)
    if found is None:
        return f"{tag}: 找不到感測器"
    sensor, _rid = found
    notes: list[str] = []
    fp = strips_union_footprint(list(main_strips))
    parts = _polys_of(parent.polygon.intersection(fp), 1.0)
    if not parts:
        return f"{tag}: 主條帶足跡與胞不相交"
    block = parts[0] if len(parts) == 1 else max(parts, key=lambda p: p.area)
    if len(parts) > 1:
        notes.append(f"區塊不連續（{len(parts)} 片），交叉帶只鋪最大一片")
    A = float(block.area)
    w_x = float(sensor.swath_m(h_x))
    s_x = float(sensor.strip_spacing_m(h_x))

    def lay(region: shapely.Geometry) -> list[Strip]:
        tmp = _replace(parent, polygon=region, h_op_m={**parent.h_op_m, cls: h_x})
        return build_strips(tmp, cls, beta, ctx)

    region, _H = cross_central_band(block, beta, float(fraction))
    if region is None or region.is_empty:
        return f"{tag}: 中央橫帶為空"
    try:
        strips = lay(region)
    except AssertionError as exc:
        return f"{tag}: 交叉帶鋪設失敗（{exc}）"
    widened = False
    if len(strips) < 2:
        # 至少 2 條相鄰（互相連結）：橫帶放寬到剛好 2 條（w + s）
        # 2 條覆蓋 w + s；給 w + 0.98·s（略小）⇒ lay_strips 的 n = ⌈(H − w + s)/s⌉ 剛好 2（給滿 w + s 時浮點誤差會鋪出第 3 條）
        region2, _H2 = cross_central_band(block, beta, float(fraction), min_height_m=w_x + 0.98 * s_x)
        try:
            strips2 = lay(region2) if region2 is not None and not region2.is_empty else []
        except AssertionError:
            strips2 = []
        if len(strips2) < 2:
            ext = projection_width(block, beta)
            return f"{tag}: 區塊在主條帶方向只有 {ext:.1f} m，放不下 2 條交叉帶（刈幅 {w_x:.1f} m、間距 {s_x:.1f} m）"
        region, strips, widened = region2, strips2, True
    bad = [s for s in strips if not s.feasible]
    if bad:
        why = sorted({s.infeasible_reason or "infeasible" for s in bad})
        return f"{tag}: {len(bad)}/{len(strips)} 條交叉帶不可行（{', '.join(why)}）"
    for k, s in enumerate(strips, 1):
        s.strip_id = cross_strip_id(parent.cell_id, cls, uav_id, k)
        s.index = int(index_base) + k
        s.cell_id = parent.cell_id
    _z_min_c, z_max_c = _terrain_minmax(ctx, region)
    z_op = (z_max_c if math.isfinite(z_max_c) else _z_ground_home(ctx)) + h_x
    e_ext = end_extension_for(ctx, cls, sensor, h_x, z_op_m=z_op)
    r_eff = r_eff_for_class(ctx, cls)
    imaged = float(strips_union_footprint(strips).intersection(block).area)
    return CrossSet(uav_id=uav_id, cell_id=parent.cell_id, cls=cls, strips=list(strips), alpha_main_rad=alpha,
                    beta_rad=beta, h_main_m=h_main, h_cross_m=h_x, block=block, region=region, block_area_m2=A,
                    region_area_m2=float(region.area), imaged_area_m2=imaged, fraction_target=float(fraction),
                    widened_to_two=widened, width_m=w_x, spacing_m=s_x, e_ext_m=float(e_ext),
                    d_turn_m=float(d_turn_from_r(r_eff, s_x, e_ext)), lateral_m=float(_lateral_m(r_eff, s_x)),
                    notes=notes)
