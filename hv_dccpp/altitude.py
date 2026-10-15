"""altitude.py — 坡度受限包絡、局部包絡、3D 組裝、起降走廊、盤旋（契約 §11）。

規格依據：§11.1–§11.9（取樣、ENVELOPE、局部包絡、BUILD_3D、不可行處置、起飛／降落走廊、站點檢查、分層）、
§4.5（保守 AGL）、§3.4（模式狀態機、轉換段幾何）、D2（前向下降角／後向爬升角）、D19（降落轉換起點在下風側）、
D29（H_tc 區間取用）、D35（CLIMB／DESCENT 段不含 h_minfw）、D36（OPER 下界含 h_min,l）、D37（轉換段旋翼輔助）。

* 所有高度皆為 ENU z（D42）；`Waypoint` 另給 `alt_msl_m`（`GeoFrame.enu_z_to_msl`）。
* `PathSegment2D` 與 `Sample2D` 於本模組定義（契約將 `PathSegment2D` 列於 planner.py；planner 應
  `from .altitude import PathSegment2D`，見 CONTRACT_DEVIATIONS）。
* 依賴：geo / wind / platform / dubins / context / routing / mission / geometry（第 3 層，不得 import 更高層）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Any, Callable, Literal, Sequence

import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon

from .context import MissionContext
from .dubins import (
    DubinsPath,
    DubinsSegment,
    Pose,
    discretize_step_rad,
    dubins_all_times,
    dubins_path,
    path_time,
)
from .geo import TerrainGrid
from .geometry import is_cross_strip
from .geometry import Cell, Strip, to_sweep_coords
from .mission import FW_PHASES, Mode, Phase, PlannerConfig, UAVInstance, UAVState, Waypoint, mode_phase_violation
from .platform import EnergyModel, PlatformSpec, Sensor
from .routing import KeepoutSet, RoutingError, avoid
from .wind import WindField, circle_time_factor, ground_speed, heading_from_track, time_arc

__all__ = [
    "EnvelopeResult", "envelope",
    "LocalProfile", "local_envelope", "make_local_envelope_fn",
    "PathSegment2D", "Sample2D", "resample_segments", "phase_boundaries", "sample_bounds",
    "smooth_projected", "reduce_points", "ProfileStats", "build_3d",
    "CorridorResult", "orbit_gain", "choose_orbit_center", "takeoff_corridor", "landing_corridor",
    "site_check", "layering", "climb_corridor_length_m",
    "ORBIT_WORD",
]

TWO_PI: float = 2.0 * math.pi
ORBIT_WORD: str = "ORBIT"
"""盤旋段之 `DubinsPath.word`（單一弧段、|Δθ| 可 ≥ 2π；`dubins.sample` 可直接取樣）。"""
_BIG_TAN: float = 1e6           # 無限坡度（旋翼／不受限）之數值替身
_TOL_M: float = 1e-6
_CALM_WIND_MPS: float = 2.0     # 規格 §3.4：W < 2 m/s 時轉換方位取障礙最少方位
_N_EXIT_CANDIDATES: int = 24    # 盤旋出口角 ϑ 之候選數（15° 間隔）
_MAX_EXTRA_LAPS: int = 3


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------
def _t_hat(theta: float) -> np.ndarray:
    return np.array([math.sin(theta), math.cos(theta)], dtype=float)


def _n_hat(theta: float) -> np.ndarray:
    return np.array([math.cos(theta), -math.sin(theta)], dtype=float)


def _mod2pi(x: float) -> float:
    return float(np.mod(x, TWO_PI))


def _clip_tan(t: np.ndarray) -> np.ndarray:
    """坡度陣列：nan → 0（不可爬降，保守）、inf → _BIG_TAN、負值 → 0。"""
    t = np.asarray(t, dtype=float)
    return np.clip(np.nan_to_num(t, nan=0.0, posinf=_BIG_TAN, neginf=0.0), 0.0, _BIG_TAN)


def _h_ceil_of(ctx: MissionContext, cls: str) -> float:
    if ctx.cfg.h_ceil_override_m is not None:
        return float(ctx.cfg.h_ceil_override_m)
    v = ctx.h_ceil_m.get(cls) if ctx.h_ceil_m else None
    return math.inf if v is None else float(v)


#: 交叉航帶作業段高度上界 = z_des + 此值（m；見 sample_bounds）
_CROSS_Z_TOL_M = 0.25


def _h_transit_of(ctx: MissionContext, cls: str, spec: PlatformSpec) -> float:
    """過境層 h_transit,c = h_minfw,c + h^extra·𝟙[c=A]（受 h_ceil 限制；規格 §11.9）。"""
    h = spec.min_fw_operation_agl_m + (ctx.cfg.h_transit_offset_A_m if cls == "A" else 0.0)
    return min(h, _h_ceil_of(ctx, cls))


def _dv_of(terrain: TerrainGrid) -> float:
    """走廊橫向取樣間距：半個網格步長（雙線性極值只出現在節點／端點）。"""
    return max(0.5 * float(terrain.step_m), 0.5)


def _sensor_h_max(ctx: MissionContext, cls: str, strip: Strip | None) -> float:
    """OPER 段 h_max,c：該構型感測器之 agl_window()[1]（多套時取容許 h_op 者之最小值，保守）。"""
    cands = [s for (c, _r), s in ctx.sensors.items() if c == cls]
    if not cands:
        return math.inf
    hs = [float(s.agl_window()[1]) for s in cands]
    if strip is not None:
        ok = [h for h in hs if h >= strip.h_op_m - _TOL_M]
        if ok:
            return min(ok)
    return min(hs)


def _wind_speed_at(wind: WindField | None, z_m: float) -> float:
    if wind is None:
        return 0.0
    return float(wind.speed_at(float(z_m)))


def _wind_vec_at(wind: WindField | None, z_m: np.ndarray | float) -> np.ndarray:
    z = np.atleast_1d(np.asarray(z_m, dtype=float))
    if wind is None:
        return np.zeros((z.size, 2))
    return np.asarray(wind.wind_vec(z), dtype=float).reshape(-1, 2)


def _r_eff_for(ctx: MissionContext, spec: PlatformSpec, z_site_m: float, z_top_m: float) -> float:
    """R_eff 以規格 §3.2 之 W_R 政策（max_layer：[z_site + H_tc,lo, z_top] 之最大風速）。"""
    tc = spec.transition_cmd(transition_profile=ctx.cfg.transition_profile)
    z_lo = float(z_site_m) + tc.h_tc_clear_m
    z_hi = max(float(z_top_m), z_lo)
    if ctx.cfg.r_eff_wind_policy == "op_layer":
        w = ctx.wind.speed_at(z_hi)
    else:
        w = ctx.wind.max_speed_between(z_lo, z_hi)
    return spec.r_eff(w)


# ---------------------------------------------------------------------------
# 11.2 坡度受限包絡（ENVELOPE；D2）
# ---------------------------------------------------------------------------
@dataclass(slots=True)
class EnvelopeResult:
    """`z_star` 逐點下確界、`u` 雙向上界、`feasible`（z* ≤ u 逐點）、`first_violation`（首個 z*>u 之索引）、
    `deficit_m`（max_j (z*−u)，不可行時 > 0）。"""
    z_star: np.ndarray
    u: np.ndarray
    feasible: bool
    first_violation: int | None
    deficit_m: float


def envelope(z_lo: np.ndarray, z_hi: np.ndarray, tan_c: np.ndarray, tan_d: np.ndarray,
             ds_m: np.ndarray | float, pin_start: float | None = None, pin_end: float | None = None) -> EnvelopeResult:
    """坡度受限包絡（規格 §11.2；D2；論文 Eq.36 之雙向精確版）。

    前向遞迴用**下降角**（a：波峰後只能以 γ_d 下降）、後向遞迴用**爬升角**（b：波峰前須以 γ_c 提前爬升）、
    取 max 得逐點下確界 z*；上界 u 自釘住起點以 tan_c 前向、自釘住終點以 tan_d 後向夾持。
    步 m（樣本 m → m+1，弧長 ds[m]）使用 tan_c[m]、tan_d[m]；ds 純量 = 全等特例。以 cumsum + maximum.accumulate
    向量化（與逐點遞迴等價）。nan 之 z_lo/z_hi（DEM 外）視為不可行。
    """
    lo = np.asarray(z_lo, dtype=float).reshape(-1)
    n = lo.size
    if n == 0:
        raise ValueError("envelope: z_lo must be non-empty")
    hi = np.broadcast_to(np.asarray(z_hi, dtype=float), (n,)).astype(float)
    tc = np.broadcast_to(np.asarray(tan_c, dtype=float), (n,)).astype(float)
    td = np.broadcast_to(np.asarray(tan_d, dtype=float), (n,)).astype(float)
    ds = np.asarray(ds_m, dtype=float)
    ds = np.full(max(n - 1, 0), float(ds)) if ds.ndim == 0 else ds.reshape(-1)
    if ds.size != n - 1:
        raise ValueError(f"envelope: ds_m must have N−1 = {n - 1} entries, got {ds.size}")
    if np.any(ds < 0):
        raise ValueError("envelope: ds_m must be ≥ 0")
    nan_mask = np.isnan(lo) | np.isnan(hi)
    lo = np.where(np.isnan(lo), -np.inf, lo)
    hi = np.where(np.isnan(hi), np.inf, hi)
    dc = ds * _clip_tan(tc[: n - 1])
    dd = ds * _clip_tan(td[: n - 1])
    C = np.concatenate([[0.0], np.cumsum(dc)])       # Σ_{m<j} ds tan_c
    D = np.concatenate([[0.0], np.cumsum(dd)])       # Σ_{m<j} ds tan_d
    lo_a = lo.copy()
    if pin_start is not None:
        lo_a[0] = max(lo_a[0], float(pin_start))
    lo_b = lo.copy()
    if pin_end is not None:
        lo_b[-1] = max(lo_b[-1], float(pin_end))
    a = np.maximum.accumulate(lo_a + D) - D
    b = np.maximum.accumulate((lo_b - C)[::-1])[::-1] + C
    z_star = np.maximum(a, b)
    u = hi.copy()
    if pin_start is not None:
        u[0] = min(u[0], float(pin_start))
        u = np.minimum.accumulate(u - C) + C
    if pin_end is not None:
        u[-1] = min(u[-1], float(pin_end))
        u = np.minimum.accumulate((u + D)[::-1])[::-1] - D
    with np.errstate(invalid="ignore"):
        gap = z_star - u
    viol = (gap > _TOL_M) | nan_mask | ~np.isfinite(z_star)
    if viol.any():
        j = int(np.argmax(viol))
        fin = gap[np.isfinite(gap)]
        deficit = float(fin.max()) if fin.size and fin.max() > 0 else math.inf
        return EnvelopeResult(z_star=z_star, u=u, feasible=False, first_violation=j, deficit_m=deficit)
    return EnvelopeResult(z_star=z_star, u=u, feasible=True, first_violation=None, deficit_m=0.0)


# ---------------------------------------------------------------------------
# 11.3 局部包絡（供 §7.4 條帶鋪設）
# ---------------------------------------------------------------------------
@dataclass(slots=True)
class LocalProfile:
    """`z` (N,) 局部剖面 z_loc = clip(z_des, z*, u)；`h_min_m` 只在條帶本身（strip_mask）上取 min（不含 ±R_eff 延伸段）；
    `agl_max_m` = max(z_loc − z_g^min)；`feasible` = z* ≤ u（不可行時該條帶／方向回報 §11.5）。"""
    z: np.ndarray
    h_min_m: float
    agl_max_m: float
    feasible: bool


def _slope_arrays(spec: PlatformSpec, theta: np.ndarray, arc: np.ndarray, wind_xy: np.ndarray, v_air: np.ndarray | float,
                  auw_kg: float, v_g_min: float, both_directions: bool = False) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """每樣本地面坡度 (tan_c, tan_d, V_g)（規格 §3.2）：tan_c = Va sinγ_c(m) κ / V_g（弧 κ = cos φ_max）、
    tan_d = Va sinγ_d / V_g。V_g 不可行（nan 或 < v_g_min）→ tan = 0（保守：不可爬降）。
    both_directions=True（條帶可雙向掃）→ 兩向取小。"""
    theta = np.asarray(theta, dtype=float).reshape(-1)
    n = theta.size
    va = np.broadcast_to(np.asarray(v_air, dtype=float), (n,)).astype(float)
    va = np.where(va > 0, va, spec.cruise_airspeed_mps)
    w = np.asarray(wind_xy, dtype=float)
    if w.ndim == 1:
        w = np.broadcast_to(w.reshape(1, 2), (n, 2))
    w_par = w[:, 0] * np.sin(theta) + w[:, 1] * np.cos(theta)
    w_perp = w[:, 0] * np.cos(theta) - w[:, 1] * np.sin(theta)
    with np.errstate(invalid="ignore"):
        rad = va * va - w_perp * w_perp
        vg = np.where(rad > 0, np.sqrt(np.where(rad > 0, rad, 0.0)) + w_par, np.nan)
    kappa = np.where(np.asarray(arc, dtype=bool), math.cos(math.radians(spec.max_bank_angle_deg)), 1.0)
    cr = spec.climb_rate_mps(auw_kg)
    dr = spec.cruise_airspeed_mps * math.sin(spec.gamma_d())
    ok = np.isfinite(vg) & (vg >= v_g_min)
    with np.errstate(divide="ignore", invalid="ignore"):
        tan_c = np.where(ok, cr * kappa / vg, 0.0)
        tan_d = np.where(ok, dr / vg, 0.0)
    if both_directions:
        tc2, td2, vg2 = _slope_arrays(spec, theta + math.pi, arc, w, va, auw_kg, v_g_min, False)
        tan_c = np.minimum(tan_c, tc2)
        tan_d = np.minimum(tan_d, td2)
        vg = np.fmin(vg, vg2)
    return tan_c, tan_d, vg


def local_envelope(centerline_xy: np.ndarray, theta_rad: np.ndarray, strip_mask: np.ndarray, cls: str, spec: PlatformSpec,
                   sensor: Sensor, terrain: TerrainGrid, corridor_half_width_m: float, corridor_offset_m: float,
                   h_clr_m: float, h_ceil_m: float | None, h_op_m: float, wind: WindField, auw_kg: float,
                   cfg: PlannerConfig) -> LocalProfile:
    """LOCAL_ENVELOPE（規格 §11.3）：同 ENVELOPE、無釘住；z_lo = 超集帶（中心相對飛行線沿 n̂ 偏移
    `corridor_offset_m`、半寬 `corridor_half_width_m`）之 z_g^max + max(h_clr, h_minfw)；z_hi 只含 h_ceil 與 h_max；
    z_loc = clip(z_des = z_g^max + h_op, z*, u)；h_min,l = min_{strip_mask}(z_loc − z_g^max)。
    `strip_mask` (N,) = 條帶本身樣本（±R_eff 延伸段只提供爬降空間，胞外山丘不縮窄條帶）。
    坡度限以兩掃向取小（條帶可雙向掃）。"""
    xy = np.asarray(centerline_xy, dtype=float).reshape(-1, 2)
    n = xy.shape[0]
    theta = np.broadcast_to(np.asarray(theta_rad, dtype=float), (n,)).astype(float)
    mask = np.broadcast_to(np.asarray(strip_mask, dtype=bool), (n,))
    if n == 0:
        raise ValueError("local_envelope: empty centerline")
    dv = _dv_of(terrain)
    nat = terrain.raw_view()                      # AGL 以自然地形量（同 `sample_bounds`）
    zg_max = nat.corridor_max(xy, theta, float(corridor_half_width_m), dv, offset_m=float(corridor_offset_m))
    zg_min = nat.corridor_min(xy, theta, float(corridor_half_width_m), dv, offset_m=float(corridor_offset_m))
    zg0 = nat.z_at(xy)
    zb_max = (terrain.corridor_max(xy, theta, float(corridor_half_width_m), dv, offset_m=float(corridor_offset_m))
              if nat is not terrain else zg_max)
    if not np.isfinite(zg_max).all():
        return LocalProfile(z=zg_max + h_op_m, h_min_m=float(h_op_m), agl_max_m=float(h_op_m), feasible=False)
    h_min_fw = spec.min_fw_operation_agl_m
    h_max = float(sensor.agl_window()[1]) if sensor is not None else math.inf
    h_ceil = math.inf if h_ceil_m is None else float(h_ceil_m)
    z_lo = np.maximum(zg_max + max(float(h_clr_m), h_min_fw), zb_max + float(h_clr_m))
    z_hi = np.minimum(zg_min + h_max * cfg.k_gsd, zg0 + h_ceil)
    z_des = zg_max + float(h_op_m)
    ds = np.hypot(*(np.diff(xy, axis=0).T)) if n > 1 else np.zeros(0)
    wxy = _wind_vec_at(wind, z_des)
    tan_c, tan_d, _ = _slope_arrays(spec, theta, np.zeros(n, dtype=bool), wxy, spec.cruise_airspeed_mps, auw_kg,
                                    cfg.v_g_min_mps, both_directions=True)
    env = envelope(z_lo, z_hi, tan_c, tan_d, ds)
    z_loc = np.clip(z_des, env.z_star, np.maximum(env.u, env.z_star))
    sel = mask if mask.any() else np.ones(n, dtype=bool)
    h_min = float(np.min(z_loc[sel] - zg_max[sel]))
    agl_max = float(np.max(z_loc[sel] - zg_min[sel]))
    return LocalProfile(z=z_loc, h_min_m=h_min, agl_max_m=agl_max, feasible=bool(env.feasible))


def make_local_envelope_fn(cls: str, spec: PlatformSpec, sensor: Sensor, terrain: TerrainGrid, h_clr_m: float,
                           h_ceil_m: float | None, h_op_m: float, wind: WindField, auw_kg: float, cfg: PlannerConfig,
                           sweep_origin: np.ndarray, alpha_rad: float, r_eff_m: float,
                           ) -> Callable[[np.ndarray, np.ndarray, float, float], tuple[float, float, bool, float]]:
    """把 :func:`local_envelope` 包成 `geometry.LocalEnvelopeFn` 閉包（供 `decompose.build_strips` / `geometry.lay_strips`）。

    閉包收到 (centerline_xy (N,2)、theta (N,)、band_v_lo、band_v_hi)：band 為超集帶在掃描座標（相對 `sweep_origin`、
    v 沿 n̂(alpha)）之下／上緣。閉包自行：算飛行線之 v_c → 走廊偏移 = 帶中心 − v_c、半寬 = 帶寬/2；沿 t̂ 兩端各延伸
    R_eff（同取樣間距）並以 strip_mask 標記原樣本；回傳 4-tuple (h_min, agl_max, feasible, z_op)，
    z_op = max_{strip}(z_g^max) + h_op（`Strip.z_op_m` 代表值）。"""
    origin = np.asarray(sweep_origin, dtype=float).reshape(2)

    def fn(centerline_xy: np.ndarray, theta: np.ndarray, band_v_lo: float, band_v_hi: float) -> tuple[float, float, bool, float]:
        xy = np.asarray(centerline_xy, dtype=float).reshape(-1, 2)
        n = xy.shape[0]
        th = np.broadcast_to(np.asarray(theta, dtype=float), (n,)).astype(float)
        v_c = float(to_sweep_coords(xy[:1], origin, float(alpha_rad))[0, 1])
        offset = 0.5 * (float(band_v_lo) + float(band_v_hi)) - v_c
        hw = 0.5 * (float(band_v_hi) - float(band_v_lo))
        if n >= 2:
            ds = float(np.hypot(*(xy[1] - xy[0])))
        else:
            ds = cfg.d_d_m
        ds = max(ds, 1.0)
        k = int(math.ceil(float(r_eff_m) / ds)) if r_eff_m > 0 else 0
        t_hat = _t_hat(float(th[0]))
        pre = xy[0] - t_hat[None, :] * (ds * np.arange(k, 0, -1))[:, None] if k else np.zeros((0, 2))
        post = xy[-1] + t_hat[None, :] * (ds * np.arange(1, k + 1))[:, None] if k else np.zeros((0, 2))
        xy_ext = np.vstack([pre, xy, post])
        th_ext = np.concatenate([np.full(k, th[0]), th, np.full(k, th[-1])])
        mask = np.zeros(xy_ext.shape[0], dtype=bool)
        mask[k:k + n] = True
        prof = local_envelope(xy_ext, th_ext, mask, cls, spec, sensor, terrain, hw, offset, h_clr_m, h_ceil_m,
                              h_op_m, wind, auw_kg, cfg)
        zg = terrain.raw_view().corridor_max(xy, th, hw, _dv_of(terrain), offset_m=offset)
        z_op = float(np.nanmax(zg)) + float(h_op_m) if np.isfinite(zg).any() else float(h_op_m)
        return prof.h_min_m, prof.agl_max_m, prof.feasible, z_op

    return fn


# ---------------------------------------------------------------------------
# 11.1 2D 航段與取樣
# ---------------------------------------------------------------------------
@dataclass(slots=True)
class PathSegment2D:
    """2D 航段（契約 §10；本模組定義，planner 匯入）。`path`：DubinsPath（含 AVOID／ORBIT 詞）或折線 (M,2)
    （單點 = 垂直段）。`z_hint_start_m/z_hint_end_m`：ENU z 提示（走廊段為高度線、其餘為目標）。
    `orbit` = (cx, cy, R, turn_dir ±1)；`ds_m` 取樣間距覆寫。"""
    phase: Phase
    mode: Mode
    path: DubinsPath | np.ndarray
    strip_id: str | None
    dir: int | None
    v_air_mps: float
    sensor_on: bool
    z_hint_start_m: float
    z_hint_end_m: float
    n_laps: int = 0
    orbit: tuple[float, float, float, int] | None = None
    ds_m: float | None = None


@dataclass(slots=True)
class Sample2D:
    """沿 2D 航跡之取樣（規格 §11.1）。`s_m` 累積弧長、`ds_m` 至下一樣本之弧長（末樣本 0）、`arc` 該樣本出發之
    航段是否為弧段。契約欄位之後的尾端欄位（皆有預設）為本模組附加：`z_hint_m`（自航段 z 提示線性內插）、
    `dir`、`arc_param`（(cx, cy, R, turn_dir)，供 Waypoint.arc）。"""
    x_m: float
    y_m: float
    theta_rad: float
    s_m: float
    ds_m: float
    phase: Phase
    mode: Mode
    arc: bool
    strip_id: str | None
    sensor_on: bool
    v_air_mps: float
    seg_index: int
    z_hint_m: float = math.nan
    dir: int | None = None
    arc_param: tuple[float, float, float, int] | None = None


def _sample_dubins(path: DubinsPath, d_line: float, e_max_m: float, psi_max_rad: float
                   ) -> tuple[np.ndarray, list[tuple[float, float, float, int] | None], np.ndarray]:
    """回傳 (pts (K,3) [x,y,theta], leg_arc[K], leg_len (K−1,)) —— leg_arc[k] 為自點 k 出發之航段弧參數（直線 None）、
    leg_len[k] 為點 k→k+1 之**真實弧長**（弧段取弧長而非弦長）。直線以 d_line 等分、弧段以 min(d_line, RΔψ) 等分
    （規格 §11.1；Δψ 依 D27）。"""
    R = float(path.R_m)
    dpsi = discretize_step_rad(R, e_max_m, psi_max_rad) if R > 0 else psi_max_rad
    d_arc = min(d_line, R * dpsi) if R > 0 else d_line
    st = path.start
    rows: list[np.ndarray] = [np.array([[st.x_m, st.y_m, st.theta_rad]])]
    legs: list[tuple[float, float, float, int] | None] = []
    lens: list[float] = []
    for seg in path.segments:
        if seg.length_m <= 1e-9:
            continue
        x0, y0, th0 = seg.start.x_m, seg.start.y_m, seg.start.theta_rad
        if seg.kind == "S":
            n = max(1, int(math.ceil(seg.length_m / d_line - 1e-12)))
            tau = np.linspace(0.0, 1.0, n + 1)[1:]
            xs = x0 + tau * seg.length_m * math.sin(th0)
            ys = y0 + tau * seg.length_m * math.cos(th0)
            ths = np.full(n, th0)
            legs.extend([None] * n)
        else:
            assert seg.center is not None
            sign = 1.0 if seg.kind == "R" else -1.0
            n = max(1, int(math.ceil(seg.length_m / d_arc - 1e-12)))
            tau = np.linspace(0.0, 1.0, n + 1)[1:]
            ths = th0 + seg.dtheta_rad * tau
            cx, cy = seg.center
            xs = cx - sign * R * np.cos(ths)
            ys = cy + sign * R * np.sin(ths)
            legs.extend([(float(cx), float(cy), R, int(sign))] * n)
        lens.extend([seg.length_m / n] * n)
        rows.append(np.column_stack([xs, ys, ths]))
    pts = np.vstack(rows)
    en = path.end
    if pts.shape[0] == 1:
        pts = np.vstack([pts, [[en.x_m, en.y_m, en.theta_rad]]])
        legs.append(None)
        lens.append(0.0)
    else:
        pts[-1, :2] = (en.x_m, en.y_m)
    legs.append(None)               # 末點無出發航段
    return pts, legs, np.asarray(lens, dtype=float)


def _sample_polyline(poly: np.ndarray, d_line: float) -> np.ndarray:
    """折線 (M,2) 逐腿以 d_line 等分 → (K,3) [x, y, theta(出發腿方位)]。"""
    p = np.asarray(poly, dtype=float).reshape(-1, poly.shape[-1])[:, :2]
    if p.shape[0] == 1:
        return np.array([[p[0, 0], p[0, 1], 0.0]])
    rows: list[np.ndarray] = []
    th_last = 0.0
    for i in range(p.shape[0] - 1):
        d = p[i + 1] - p[i]
        L = float(np.hypot(d[0], d[1]))
        if L <= 1e-9:
            continue
        th = math.atan2(d[0], d[1])
        th_last = th
        n = max(1, int(math.ceil(L / d_line - 1e-12)))
        tau = np.linspace(0.0, 1.0, n + 1)[:-1]
        xs = p[i, 0] + tau * d[0]
        ys = p[i, 1] + tau * d[1]
        rows.append(np.column_stack([xs, ys, np.full(n, th)]))
    if not rows:
        return np.array([[p[0, 0], p[0, 1], 0.0], [p[-1, 0], p[-1, 1], 0.0]])
    pts = np.vstack(rows + [np.array([[p[-1, 0], p[-1, 1], th_last]])])
    return pts


def resample_segments(segments: list[PathSegment2D], d_d_m: float, e_max_m: float, psi_max_rad: float) -> list[Sample2D]:
    """沿全架次 2D 航跡取樣（規格 §11.1）：直線 d_d、弧段 min(d_d, RΔψ)；每段 ds_j 逐段。
    相鄰航段之接點只保留一個樣本並採**後一航段**之屬性（phase/mode/strip/sensor_on/v_air；出發航段語意），
    末段終點保留。單點折線（垂直段）給兩個同位樣本（ds=0）。`z_hint_m` 依段內弧長比例自 z_hint_start→end 內插。"""
    out: list[Sample2D] = []
    s_acc = 0.0
    for k, seg in enumerate(segments):
        d_line = float(seg.ds_m) if seg.ds_m is not None else float(d_d_m)
        if isinstance(seg.path, DubinsPath):
            pts, legs, d = _sample_dubins(seg.path, d_line, e_max_m, psi_max_rad)
        else:
            arr = np.asarray(seg.path, dtype=float)
            if arr.ndim == 1:
                arr = arr.reshape(1, -1)
            if arr.shape[0] == 1:
                pts = np.array([[arr[0, 0], arr[0, 1], 0.0], [arr[0, 0], arr[0, 1], 0.0]])
            else:
                pts = _sample_polyline(arr, d_line)
            legs = [None] * pts.shape[0]
            d = np.hypot(*(np.diff(pts[:, :2], axis=0).T)) if pts.shape[0] > 1 else np.zeros(0)
        n = pts.shape[0]
        s_loc = np.concatenate([[0.0], np.cumsum(d)])
        L = float(s_loc[-1])
        frac = s_loc / L if L > 0 else np.linspace(0.0, 1.0, n)
        z_hint = float(seg.z_hint_start_m) + frac * (float(seg.z_hint_end_m) - float(seg.z_hint_start_m))
        start = 0
        if out and n > 0 and math.hypot(pts[0, 0] - out[-1].x_m, pts[0, 1] - out[-1].y_m) < 1e-6:
            # 接點：以後一航段屬性取代前一航段之終點樣本
            prev = out.pop()             # 前段終點與新段起點同位：ds 不變
            s_acc = prev.s_m
        for i in range(start, n):
            ds_next = float(d[i]) if i < n - 1 else 0.0
            arc_p = legs[i] if i < len(legs) else None
            out.append(Sample2D(
                x_m=float(pts[i, 0]), y_m=float(pts[i, 1]), theta_rad=_mod2pi(float(pts[i, 2])),
                s_m=s_acc + float(s_loc[i]), ds_m=ds_next, phase=seg.phase, mode=seg.mode,
                arc=arc_p is not None, strip_id=seg.strip_id, sensor_on=bool(seg.sensor_on),
                v_air_mps=float(seg.v_air_mps), seg_index=k, z_hint_m=float(z_hint[i]), dir=seg.dir, arc_param=arc_p,
            ))
        s_acc += L
    if out:
        out[-1].ds_m = 0.0
    return out


# ---------------------------------------------------------------------------
# 樣本陣列與每樣本量
# ---------------------------------------------------------------------------
class _Arrays:
    """樣本欄位之 numpy 檢視。"""
    __slots__ = ("xy", "theta", "s", "ds", "arc", "v_air", "z_hint", "phase", "n")

    def __init__(self, samples: Sequence[Sample2D]) -> None:
        self.n = len(samples)
        self.xy = np.array([[p.x_m, p.y_m] for p in samples], dtype=float).reshape(-1, 2)
        self.theta = np.array([p.theta_rad for p in samples], dtype=float)
        self.s = np.array([p.s_m for p in samples], dtype=float)
        self.ds = np.array([p.ds_m for p in samples], dtype=float)
        self.arc = np.array([p.arc for p in samples], dtype=bool)
        self.v_air = np.array([p.v_air_mps for p in samples], dtype=float)
        self.z_hint = np.array([p.z_hint_m for p in samples], dtype=float)
        self.phase = [p.phase for p in samples]

    def is_phase(self, *phases: Phase) -> np.ndarray:
        ps = set(phases)
        return np.array([p in ps for p in self.phase], dtype=bool)


_CORRIDOR_PHASES = {Phase.TAKEOFF, Phase.TRANS_OUT, Phase.TRANS_IN, Phase.LAND}
_LEVEL_PHASES = {Phase.ACCEL, Phase.DECEL}
_RELABEL_CLIMB = {Phase.ENTRY, Phase.ORBIT, Phase.HOLD, Phase.TRANSFER, Phase.RTB, Phase.CLIMB}
_RELABEL_DESC = {Phase.RTB, Phase.ORBIT, Phase.HOLD, Phase.TRANSFER, Phase.ENTRY, Phase.DESCENT}


def _obstacle_tops(ctx: MissionContext, xy: np.ndarray) -> np.ndarray:
    """z_obs(j) = max{tops_z(含 b_v)：(x_j,y_j) ∈ K_o}（規格 §11.1）；無 → −inf。禁飛區頂 = +inf。"""
    n = xy.shape[0]
    out = np.full(n, -np.inf)
    ko: KeepoutSet | None = ctx.keepouts
    if ko is None or len(ko) == 0 or n == 0:
        return out
    pts = shapely.points(xy)
    hit = ko.tree.query(pts, predicate="within")
    if hit.size == 0:
        return out
    tops = np.asarray(ko.tops_z, dtype=float)
    np.maximum.at(out, hit[0], tops[hit[1]])
    return out


def phase_boundaries(samples: list[Sample2D], cls: str, ctx: MissionContext, pin_start_z: float, pin_end_z: float,
                     auw_kg: float) -> tuple[int, int]:
    """CLIMB／DESCENT 段界（規格 §11.1、D35）。回傳 (j_c, j_d)：j_c = CLIMB 段止（首個樣本使「自起飛側 P_tc 以 tan_c
    全速爬升之上界線」≥ z_g^max + h_minfw）、j_d = DESCENT 段起（最後一個樣本使「自降落側 P_tc 以 tan_d 反推之上界線」
    ≥ z_g^max + h_minfw）。上界線同時取保守值（順風最大 V_g 之 tan，與 `spec.climb_dist_m` 一致；平地 A 471 m）與
    逐樣本實際值（含弧段 κ）兩者皆滿足，確保 CLIMB 段後之剖面必達 h_minfw。
    **就地**把 [c0, j_c) 之非 OPER 固定翼樣本改標 CLIMB、(j_d, d1] 改標 DESCENT（只在存在 ACCEL／DECEL 樣本時）。"""
    n = len(samples)
    if n == 0:
        return (0, 0)
    A = _Arrays(samples)
    spec = ctx.specs[cls]
    cfg = ctx.cfg
    fw = A.is_phase(*FW_PHASES)
    idx = np.flatnonzero(fw)
    if idx.size == 0:
        return (0, n - 1)
    i0, i1 = int(idx[0]), int(idx[-1])
    has_accel = any(p is Phase.ACCEL for p in A.phase)
    has_decel = any(p is Phase.DECEL for p in A.phase)
    c0 = i0
    while c0 <= i1 and A.phase[c0] is Phase.ACCEL:
        c0 += 1
    c0 = min(c0, i1)
    d1 = i1
    while d1 >= i0 and A.phase[d1] is Phase.DECEL:
        d1 -= 1
    d1 = max(d1, i0)
    j_c, j_d = c0, d1
    h_minfw = spec.min_fw_operation_agl_m
    zg = ctx.terrain.raw_view().corridor_max(A.xy, A.theta, cfg.b_trk_m, _dv_of(ctx.terrain))
    need = np.where(np.isfinite(zg), zg, -np.inf) + h_minfw
    z_est = np.where(np.isfinite(zg), zg, 0.0) + h_minfw
    wxy = _wind_vec_at(ctx.wind, z_est)
    tan_c, tan_d, _ = _slope_arrays(spec, A.theta, A.arc, wxy, A.v_air, auw_kg, cfg.v_g_min_mps)
    va = spec.cruise_airspeed_mps
    if has_accel and c0 <= i1:
        w = _wind_speed_at(ctx.wind, float(pin_start_z))
        tan_cons = spec.tan_gamma_ground("climb", va + w, auw_kg, arc=False)
        rel = A.s[c0:i1 + 1] - A.s[c0]
        u_cons = float(pin_start_z) + rel * tan_cons
        u_act = float(pin_start_z) + np.concatenate([[0.0], np.cumsum((A.ds * tan_c)[c0:i1])])
        ok = (u_cons >= need[c0:i1 + 1] - _TOL_M) & (u_act >= need[c0:i1 + 1] - _TOL_M)
        j_c = c0
        for j in range(c0, i1 + 1):
            if ok[j - c0] or A.phase[j] not in _RELABEL_CLIMB:
                j_c = j
                break
        else:
            j_c = i1
        for j in range(c0, j_c):
            if A.phase[j] in _RELABEL_CLIMB:
                samples[j].phase = Phase.CLIMB
    if has_decel and d1 >= i0:
        w = _wind_speed_at(ctx.wind, float(pin_end_z))
        tan_cons = spec.tan_gamma_ground("descent", va + w, auw_kg, arc=False)
        rel = A.s[d1] - A.s[i0:d1 + 1]
        u_cons = float(pin_end_z) + rel * tan_cons
        back = np.cumsum((A.ds * tan_d)[i0:d1][::-1])[::-1] if d1 > i0 else np.zeros(0)
        u_act = float(pin_end_z) + np.concatenate([back, [0.0]])
        ok = (u_cons >= need[i0:d1 + 1] - _TOL_M) & (u_act >= need[i0:d1 + 1] - _TOL_M)
        j_d = d1
        for j in range(d1, i0 - 1, -1):
            if ok[j - i0] or A.phase[j] not in _RELABEL_DESC:
                j_d = j
                break
        else:
            j_d = i0
        for j in range(j_d + 1, d1 + 1):
            if A.phase[j] in _RELABEL_DESC:
                samples[j].phase = Phase.DESCENT
    return (int(j_c), int(j_d))


def sample_bounds(samples: list[Sample2D], cls: str, ctx: MissionContext, strips: dict[str, Strip], auw_kg: float,
                  j_c: int, j_d: int) -> tuple[np.ndarray, ...]:
    """每樣本量（規格 §11.1）→ (z_lo, z_hi, z_des, tan_c, tan_d, ds)。

    * 走廊 b_j = w_l/2（OPER）或 b_trk；z_obs 取 keep-out 頂（含 b_v）。
    * ``z_g``（z_g^max/z_g^min/z_g^0）一律取 **自然地形** ``terrain.raw_view()``：AGL（h_clr/h_minfw/h_ceil/h_min,l）
      皆為「地面之上」；障礙以 ``z_obs = z_top + b_v`` 與「燒入後地形 + h_clr」兩項下界處理（否則等同要求
      障礙頂之上再加 h_minfw，且與 z_g^0 + h_ceil 之上界互相矛盾）。
    * z_lo：CLIMB/DESCENT = max(z_g^max + h_clr, z_obs)（**不含 h_minfw**，D35）；ENTRY/TRANS/RTB/ORBIT/HOLD 加 h_minfw；
      OPER 另加 z_g^max_{w/2} + strips[id].h_min_m（D36）；ACCEL/DECEL 與走廊段（TO/TRANS_OUT/TRANS_IN/LAND）
      z_lo = z_hi = z_hint（高度線）。
    * z_hi = min(z_g^min + h_max k_GSD [OPER], z_g^0 + h_ceil, z_msl,max)。
    * z_des：OPER z_g^max + h_op；ENTRY/RTB/ORBIT/HOLD/DESCENT z_g^max + h_transit；TRANS 前後 OPER 目標線性內插；CLIMB +inf。
    * tan_c/tan_d 逐樣本（θ_j、風於 z_des、弧段 κ）；ACCEL/DECEL 與走廊段 tan = 0（等高／旋翼段不參與固定翼坡度）。"""
    n = len(samples)
    spec = ctx.specs[cls]
    cfg = ctx.cfg
    if n == 0:
        z = np.zeros(0)
        return (z, z, z, z, z, z)
    A = _Arrays(samples)
    hw = np.full(n, float(cfg.b_trk_m))
    h_min_l = np.zeros(n)
    h_op_l = np.full(n, np.nan)
    h_max_l = np.full(n, np.inf)
    is_oper = np.zeros(n, dtype=bool)
    is_x = np.zeros(n, dtype=bool)                  # 建圖模式交叉航帶的作業樣本
    for j, p in enumerate(samples):
        if p.phase is Phase.OPERATE and p.strip_id is not None and p.strip_id in strips:
            st = strips[p.strip_id]
            hw[j] = 0.5 * st.width_m
            h_min_l[j] = st.h_min_m
            h_op_l[j] = st.h_op_m
            h_max_l[j] = _sensor_h_max(ctx, cls, st)
            is_oper[j] = True
            is_x[j] = is_cross_strip(p.strip_id)
    dv = _dv_of(ctx.terrain)
    nat = ctx.terrain.raw_view()                  # AGL 一律以自然地形量（障礙淨空走 z_obs，見下）
    zg_max = nat.corridor_max(A.xy, A.theta, hw, dv)
    zg_min = nat.corridor_min(A.xy, A.theta, hw, dv)
    zg0 = nat.z_at(A.xy)
    zb_max = ctx.terrain.corridor_max(A.xy, A.theta, hw, dv) if nat is not ctx.terrain else zg_max
    z_obs = _obstacle_tops(ctx, A.xy)
    h_clr = float(ctx.h_clr_m)
    h_minfw = spec.min_fw_operation_agl_m
    h_ceil = _h_ceil_of(ctx, cls)
    h_tr = _h_transit_of(ctx, cls, spec)
    ph_climb = A.is_phase(Phase.CLIMB, Phase.DESCENT)
    ph_level = A.is_phase(*_LEVEL_PHASES)
    ph_corr = A.is_phase(*_CORRIDOR_PHASES)
    ph_fixed = ph_level | ph_corr
    ph_trans = A.is_phase(Phase.TRANSFER)
    ph_desc = A.is_phase(Phase.DESCENT)
    z_lo = np.maximum(zg_max + h_clr, z_obs)
    z_lo = np.maximum(z_lo, zb_max + h_clr)       # 燒入後地形（含障礙頂 + b_v）仍保有 h_clr 淨空
    z_lo = np.where(ph_climb, z_lo, np.maximum(z_lo, zg_max + h_minfw))
    z_lo = np.where(is_oper, np.maximum(z_lo, zg_max + h_min_l), z_lo)
    z_hi = zg0 + h_ceil
    z_hi = np.where(is_oper, np.minimum(z_hi, zg_min + h_max_l * cfg.k_gsd), z_hi)
    if is_x.any():
        # 交叉航帶（MappingModeConfig.cross_strips，可與主條帶不同高）：作業段上界收緊到 z_des + _CROSS_Z_TOL_M。
        #   否則前後的進入／返航段 z_des = 過境高度 h_transit，包絡平滑會把升降延伸進交叉帶（1 km 實測：交叉 100 m、
        #   A 的 h_transit 120 m ⇒ 帶上 100 → 116.6 m）。主條帶不走這裡（舊行為逐位元不變）。
        z_hi = np.where(is_x, np.maximum(np.minimum(z_hi, zg_max + h_op_l + _CROSS_Z_TOL_M), z_lo), z_hi)
    msl_max = ctx.inputs.airspace.max_msl_m
    if msl_max is not None and cfg.h_ceil_override_m is None:
        z_hi = np.minimum(z_hi, ctx.frame.msl_to_enu_z(A.xy, float(msl_max)))
    z_lo = np.where(ph_fixed, A.z_hint, z_lo)
    z_hi = np.where(ph_fixed, A.z_hint, z_hi)
    # z_des
    z_des = zg_max + h_tr
    z_des = np.where(is_oper, zg_max + np.where(np.isnan(h_op_l), h_tr, h_op_l), z_des)
    z_des = np.where(A.is_phase(Phase.CLIMB), np.inf, z_des)
    z_des = np.where(ph_desc, zg_max + h_tr, z_des)
    z_des = np.where(ph_fixed, A.z_hint, z_des)
    if ph_trans.any() and is_oper.any():
        oper_idx = np.flatnonzero(is_oper)
        tr_idx = np.flatnonzero(ph_trans)
        s_op, z_op = A.s[oper_idx], z_des[oper_idx]
        z_des[tr_idx] = np.interp(A.s[tr_idx], s_op, z_op)
    # 坡度限
    z_wind = np.where(np.isfinite(z_des), z_des, zg_max + h_minfw)
    z_wind = np.where(np.isfinite(z_wind), z_wind, A.z_hint)
    z_wind = np.where(np.isfinite(z_wind), z_wind, 0.0)
    wxy = _wind_vec_at(ctx.wind, z_wind)
    tan_c, tan_d, _ = _slope_arrays(spec, A.theta, A.arc, wxy, A.v_air, auw_kg, cfg.v_g_min_mps)
    tan_c = np.where(ph_fixed, 0.0, tan_c)
    tan_d = np.where(ph_fixed, 0.0, tan_d)
    return (z_lo, z_hi, z_des, tan_c, tan_d, A.ds.copy())


# ---------------------------------------------------------------------------
# 11.4 平滑、點刪減、BUILD_3D
# ---------------------------------------------------------------------------
def smooth_projected(z: np.ndarray, z_des: np.ndarray, z_star: np.ndarray, u: np.ndarray, cfg: PlannerConfig) -> np.ndarray:
    """論文 Eq.37/38 投影梯度下降：z_j −= lr·(c1·sign(z_j − z_des,j) + 2c2(2z_j − z_{j−1} − z_{j+1}))，
    每步投影至 [z*, u]（規格 §11.4）；端點固定。z_des = ±inf 時 sign 項推向該側（由投影夾持）。"""
    z = np.array(z, dtype=float)
    n = z.size
    if n < 3 or cfg.n_smooth <= 0:
        return np.clip(z, z_star, np.maximum(u, z_star))
    lo = np.asarray(z_star, dtype=float)
    hi = np.maximum(np.asarray(u, dtype=float), lo)
    zd = np.asarray(z_des, dtype=float)
    for _ in range(int(cfg.n_smooth)):
        with np.errstate(invalid="ignore"):
            g = cfg.c1 * np.sign(z[1:-1] - zd[1:-1]) + 2.0 * cfg.c2 * (2.0 * z[1:-1] - z[:-2] - z[2:])
        g = np.nan_to_num(g, nan=0.0)
        z[1:-1] -= cfg.lr * g
        z = np.clip(z, lo, hi)
    return z


def reduce_points(samples: list[Sample2D], z: np.ndarray, keep: np.ndarray, cfg: PlannerConfig) -> np.ndarray:
    """點刪減（論文 Eq.39 3D 共線 + 最大間距 L_max；規格 §11.4）：自上一保留點起，若中間各點至弦
    （上一保留點 → 候選點）之 3D 距離皆 ≤ eps_col 且弦長 ≤ L_max 則可略過；否則保留前一點。`keep` 為 True 者一律保留。
    回傳保留索引（升冪，含首末）。"""
    n = len(samples)
    if n == 0:
        return np.zeros(0, dtype=int)
    P = np.column_stack([[p.x_m for p in samples], [p.y_m for p in samples], np.asarray(z, dtype=float)])
    keep = np.asarray(keep, dtype=bool)
    out = [0]
    last = 0
    j = 1
    while j < n:
        if keep[j] or j == n - 1:
            out.append(j)
            last = j
            j += 1
            continue
        # 試探是否可跳過 j：檢查 (last, j+1] 之弦
        cand = j + 1
        chord = P[cand] - P[last]
        L = float(np.linalg.norm(chord))
        ok = L <= cfg.L_max_m
        if ok and L > 0:
            mid = P[last + 1:cand] - P[last]
            t = np.clip(mid @ chord / (L * L), 0.0, 1.0)
            dev = np.linalg.norm(mid - t[:, None] * chord[None, :], axis=1)
            ok = bool(np.all(dev <= cfg.eps_col))
        elif L == 0:
            ok = True
        if ok:
            j += 1
        else:
            out.append(j)
            last = j
            j += 1
    if out[-1] != n - 1:
        out.append(n - 1)
    return np.array(sorted(set(out)), dtype=int)


@dataclass(slots=True)
class ProfileStats:
    """BUILD_3D 統計與驗證（規格 §11.4 verify）。`agl_min/max_by_strip` 以走廊 z_g^max／z_g^min 計（D36 不變量）。"""
    agl_min_by_strip: dict[str, float]
    agl_max_by_strip: dict[str, float]
    slope_max_rad: float
    slope_min_rad: float
    n_points: int
    violations: list[str]
    infeasible_at: int | None
    infeasible_phase: Phase | None


def _seg_lengths(A: _Arrays, samples: list[Sample2D]) -> np.ndarray:
    """每樣本所屬 2D 航段之總弧長（同 seg_index 之 ds 和）。"""
    idx = np.array([p.seg_index for p in samples], dtype=int)
    tot = np.zeros(idx.max() + 1 if idx.size else 0)
    np.add.at(tot, idx, A.ds)
    return tot[idx]


def _transfer_wh_vec(model: EnergyModel, t_s: np.ndarray, dz_m: np.ndarray) -> np.ndarray:
    """`EnergyModel.transfer_wh` 之向量版（規格 §8.4；備降可達性用）。"""
    pc = model.p_cruise_w()
    gc = model.spec.gamma_c_auw(model.auw_kg)
    gd = model.spec.gamma_d()
    p_up, p_dn = model.p_climb_w(gc), model.p_descent_w(gd)
    va = model.spec.cruise_airspeed_mps
    e = pc * t_s + (p_up - pc) * np.maximum(0.0, dz_m) / (va * math.sin(gc)) \
        - (pc - p_dn) * np.maximum(0.0, -dz_m) / (va * math.sin(gd))
    e = e * model.k_unc
    return e if model.base_minute_mode else e / 3600.0


def build_3d(segments: list[PathSegment2D], uav: UAVInstance, seq: Any, ctx: MissionContext, strips: dict[str, Strip],
             t_start_s: float, e_start_wh: float, pin_start_z: float, pin_end_z: float) -> tuple[list[Waypoint], ProfileStats]:
    """BUILD_3D（規格 §11.4）：取樣 → 段界（§11.1）→ 每樣本量 → ENVELOPE（釘住 P_tc 兩端）→ clip(z_des) → 二次 ENVELOPE →
    投影平滑 → 三次 ENVELOPE 夾持 → 時間戳／能量積分／備降可達／失鏈動作 → 點刪減 → 驗證。

    固定翼區塊 = 首個 FW 樣本（ACCEL 起）至末個 FW 樣本（DECEL 止）；走廊樣本（TO/TRANS_OUT/TRANS_IN/LAND）z 直接取高度線。
    ENVELOPE 不可行時仍輸出航點（z = max(z*, …)）並於 `ProfileStats.infeasible_at/violations` 回報（§11.5 由上層處置）。
    `seq`（UAVSequence，鴨子型別）只讀 `landing_site`。"""
    cfg = ctx.cfg
    cls = uav.cls
    spec = ctx.specs[cls]
    model = ctx.models.get(uav.uav_id) or ctx.models_rep[cls]
    auw = float(uav.sortie.auw_kg)
    samples = resample_segments(segments, cfg.d_d_m, cfg.e_max_m, cfg.psi_max_rad)
    n = len(samples)
    violations: list[str] = []
    empty_stats = ProfileStats({}, {}, 0.0, 0.0, 0, violations, None, None)
    if n == 0:
        return [], empty_stats
    j_c, j_d = phase_boundaries(samples, cls, ctx, pin_start_z, pin_end_z, auw)
    z_lo, z_hi, z_des, tan_c, tan_d, ds = sample_bounds(samples, cls, ctx, strips, auw, j_c, j_d)
    A = _Arrays(samples)
    fw = A.is_phase(*FW_PHASES)
    z = np.where(np.isfinite(A.z_hint), A.z_hint, np.where(np.isfinite(z_lo), z_lo, 0.0)).astype(float)
    infeasible_at: int | None = None
    infeasible_phase: Phase | None = None
    z_star_full = np.full(n, -np.inf)
    u_full = np.full(n, np.inf)
    idx = np.flatnonzero(fw)
    if idx.size:
        i0, i1 = int(idx[0]), int(idx[-1])
        sl = slice(i0, i1 + 1)
        lo, hi = z_lo[sl].copy(), z_hi[sl].copy()
        acc = A.is_phase(Phase.ACCEL)[sl]
        dec = A.is_phase(Phase.DECEL)[sl]
        lo[acc] = hi[acc] = float(pin_start_z)
        lo[dec] = hi[dec] = float(pin_end_z)
        env = envelope(lo, hi, tan_c[sl], tan_d[sl], ds[sl][:-1], pin_start_z, pin_end_z)
        z_star, u = env.z_star, np.maximum(env.u, env.z_star)
        if not env.feasible:
            infeasible_at = i0 + int(env.first_violation or 0)
            infeasible_phase = samples[infeasible_at].phase
            violations.append(f"envelope_infeasible(sample={infeasible_at}, phase={infeasible_phase.name}, "
                              f"deficit={env.deficit_m:.2f} m)")
        zz = np.clip(z_des[sl], z_star, u)
        zz = envelope(zz, u, tan_c[sl], tan_d[sl], ds[sl][:-1], pin_start_z, pin_end_z).z_star
        zz = smooth_projected(zz, z_des[sl], z_star, u, cfg)
        zz = envelope(zz, u, tan_c[sl], tan_d[sl], ds[sl][:-1], pin_start_z, pin_end_z).z_star
        zz = np.clip(zz, z_star, np.maximum(u, z_star))
        z[sl] = zz
        z_star_full[sl] = z_star
        u_full[sl] = u
    # ---- 時間與能量 ---------------------------------------------------------
    wxy = _wind_vec_at(ctx.wind, z)
    vg = np.zeros(n)
    hdg = A.theta.copy()
    pos_v = A.v_air > 0
    if pos_v.any():
        vg_all = np.full(n, np.nan)
        for va_val in np.unique(A.v_air[pos_v]):
            m = pos_v & (A.v_air == va_val)
            vg_all[m] = ground_speed(A.theta[m], float(va_val), wxy[m])
            hdg[m] = heading_from_track(A.theta[m], float(va_val), wxy[m])
        vg = np.where(np.isfinite(vg_all), vg_all, 0.0)
    hdg = np.where(np.isfinite(hdg), hdg, A.theta)
    seg_len = _seg_lengths(A, samples)
    tc = spec.transition_cmd(transition_profile=cfg.transition_profile)
    dt = np.zeros(n)
    dz = np.concatenate([np.diff(z), [0.0]])
    for j, p in enumerate(samples):
        ph = p.phase
        if ph is Phase.TAKEOFF:
            dt[j] = abs(dz[j]) / spec.vtol_climb_rate_mps if p.ds_m <= 1e-9 else 0.0
        elif ph is Phase.LAND:
            dt[j] = spec.hover_descent_time_s(abs(dz[j])) if p.ds_m <= 1e-9 else 0.0
        elif ph is Phase.TRANS_OUT:
            dt[j] = spec.t_tr_out_s * (p.ds_m / seg_len[j] if seg_len[j] > 0 else 0.0)
        elif ph is Phase.TRANS_IN:
            dt[j] = spec.t_tr_in_s * (p.ds_m / seg_len[j] if seg_len[j] > 0 else 0.0)
        elif ph in _LEVEL_PHASES:
            dt[j] = tc.t_acc_s * (p.ds_m / seg_len[j] if seg_len[j] > 0 else 0.0)
        else:
            if p.ds_m <= 0.0:
                dt[j] = 0.0
            elif vg[j] >= cfg.v_g_min_mps:
                dt[j] = p.ds_m / vg[j]
            else:
                dt[j] = p.ds_m / cfg.v_g_min_mps
                violations.append(f"v_g_infeasible(sample={j}, theta={math.degrees(p.theta_rad):.1f}deg)")
    t = float(t_start_s) + np.concatenate([[0.0], np.cumsum(dt[:-1])])
    va_c = spec.cruise_airspeed_mps
    e_seg = np.zeros(n)
    for j, p in enumerate(samples):
        if dt[j] <= 0.0:
            continue
        ph = p.phase
        if ph is Phase.TAKEOFF or ph is Phase.LAND:
            e_seg[j] = model.segment_wh(Mode.HOVER, dt[j])
        elif ph is Phase.TRANS_OUT or ph is Phase.TRANS_IN:
            e_seg[j] = model.segment_wh(Mode.TRANSITION, dt[j])
        elif ph in _LEVEL_PHASES:
            e_seg[j] = model.segment_wh("ACCEL", dt[j])
        else:
            ratio = dz[j] / (va_c * dt[j])
            gamma = math.asin(max(-1.0, min(1.0, ratio)))
            e_seg[j] = model.segment_wh(Mode.CRUISE, dt[j], gamma)
    e_rem = float(e_start_wh) - np.concatenate([[0.0], np.cumsum(e_seg[:-1])])
    # ---- 備降可達與失鏈 -------------------------------------------------------
    site_ids = list(ctx.sites_enu.keys())
    site_xy = np.array([ctx.sites_enu[s][0] for s in site_ids], dtype=float).reshape(-1, 2)
    site_z = np.array([ctx.sites_enu[s][1] for s in site_ids], dtype=float)
    dists = np.linalg.norm(A.xy[:, None, :] - site_xy[None, :, :], axis=2)         # (n, S)
    nearest = np.argmin(dists, axis=1)
    d_near = dists[np.arange(n), nearest]
    w_max = ctx.wind.max_speed() if ctx.wind is not None else 0.0
    vg_worst = max(cfg.v_g_min_mps, va_c - w_max)
    dz_alt = (site_z[nearest] + tc.h_tc_energy_m) - z
    e_alt = _transfer_wh_vec(model, d_near / vg_worst, dz_alt) + model.e_landing_wh()
    reserve = model.reserve_wh()
    reachable = e_rem >= reserve + e_alt
    policy = ctx.inputs.c2.policy_for(uav.uav_id, cls)
    committed = {Phase.RTB, Phase.DESCENT, Phase.DECEL, Phase.TRANS_IN, Phase.LAND}
    # ---- 航點 -----------------------------------------------------------------
    keep = np.zeros(n, dtype=bool)
    keep[0] = keep[-1] = True
    for j in range(1, n):
        p, q = samples[j - 1], samples[j]
        if p.phase is not q.phase or p.sensor_on != q.sensor_on or p.arc != q.arc or p.strip_id != q.strip_id \
                or p.phase in _CORRIDOR_PHASES or q.phase in _CORRIDOR_PHASES or p.seg_index != q.seg_index:
            keep[j] = True
            keep[j - 1] = True
    kept = reduce_points(samples, z, keep, cfg)
    wgs = ctx.frame.to_wgs84(np.column_stack([A.xy, z]))
    zg0 = ctx.terrain.raw_view().z_at(A.xy)          # Waypoint.agl_m = 自然地形之上（障礙淨空另由 z_obs/keep-out 保證）
    agl = z - np.where(np.isfinite(zg0), zg0, 0.0)
    v_min = spec.v_min_mission()
    waypoints: list[Waypoint] = []
    for j in kept:
        p = samples[int(j)]
        if p.phase in committed or policy == "RTB":
            action = "RTB"
        elif policy == "NEAREST_ALT":
            action = f"ALT:{site_ids[nearest[j]]}"
        else:
            action = "CONTINUE"
        cell_id = strips[p.strip_id].cell_id if (p.strip_id is not None and p.strip_id in strips) else None
        wp = Waypoint(
            x_m=p.x_m, y_m=p.y_m, z_m=float(z[j]), lat_deg=float(wgs[j, 0]), lon_deg=float(wgs[j, 1]),
            alt_msl_m=float(wgs[j, 2]), agl_m=float(agl[j]), t_s=float(t[j]), mode=p.mode, phase=p.phase,
            sensor_on=p.sensor_on, strip_id=p.strip_id, cell_id=cell_id, dir=p.dir,
            airspeed_mps=float(p.v_air_mps), ground_speed_mps=float(vg[j]),
            track_deg=math.degrees(p.theta_rad), heading_deg=math.degrees(_mod2pi(float(hdg[j]))),
            energy_remaining_wh=float(e_rem[j]), nearest_alternate=site_ids[nearest[j]],
            alt_reachable=bool(reachable[j]), lost_link_action=action, arc=p.arc_param,
        )
        waypoints.append(wp)
        v = mode_phase_violation(p.mode, p.phase, p.v_air_mps, v_min)
        if v is not None:
            violations.append(f"mode_phase(sample={int(j)}): {v}")
    # ---- 驗證 -----------------------------------------------------------------
    agl_min_by: dict[str, float] = {}
    agl_max_by: dict[str, float] = {}
    if idx.size:
        hw_op = np.array([0.5 * strips[p.strip_id].width_m if (p.phase is Phase.OPERATE and p.strip_id in strips)
                          else cfg.b_trk_m for p in samples])
        nat_v = ctx.terrain.raw_view()             # 條帶 AGL 驗證與 h_min,l（local_envelope）同基準
        zg_max = nat_v.corridor_max(A.xy, A.theta, hw_op, _dv_of(ctx.terrain))
        zg_min = nat_v.corridor_min(A.xy, A.theta, hw_op, _dv_of(ctx.terrain))
        for j, p in enumerate(samples):
            if p.phase is Phase.OPERATE and p.strip_id is not None:
                a_min, a_max = float(z[j] - zg_max[j]), float(z[j] - zg_min[j])
                agl_min_by[p.strip_id] = min(agl_min_by.get(p.strip_id, np.inf), a_min)
                agl_max_by[p.strip_id] = max(agl_max_by.get(p.strip_id, -np.inf), a_max)
        for sid, a_min in agl_min_by.items():
            if sid in strips and a_min < strips[sid].h_min_m - 1e-3:
                violations.append(f"oper_agl_below_h_min({sid}: {a_min:.2f} < {strips[sid].h_min_m:.2f})")
            h_max = _sensor_h_max(ctx, cls, strips.get(sid)) * cfg.k_gsd
            if math.isfinite(h_max) and agl_max_by[sid] > h_max + 1e-3:
                violations.append(f"oper_agl_above_h_max({sid}: {agl_max_by[sid]:.2f} > {h_max:.2f})")
        i0, i1 = int(idx[0]), int(idx[-1])
        d_z = np.diff(z[i0:i1 + 1])
        d_s = ds[i0:i1]
        ok = d_s > 1e-9
        slopes = np.where(ok, d_z / np.where(ok, d_s, 1.0), 0.0)
        tcl, tdl = _clip_tan(tan_c[i0:i1]), _clip_tan(tan_d[i0:i1])
        lvl = A.is_phase(*_LEVEL_PHASES)[i0:i1]
        bad = ok & ~lvl & ((slopes > tcl + 1e-6) | (slopes < -tdl - 1e-6))
        if bad.any():
            violations.append(f"slope_exceeded(n={int(bad.sum())}, first_sample={i0 + int(np.argmax(bad))})")
        over = z[i0:i1 + 1] > z_hi[i0:i1 + 1] + 1e-3
        if over.any():
            violations.append(f"above_z_hi(n={int(over.sum())}, first_sample={i0 + int(np.argmax(over))})")
        slope_max = float(math.atan(slopes.max())) if slopes.size else 0.0
        slope_min = float(math.atan(slopes.min())) if slopes.size else 0.0
    else:
        slope_max = slope_min = 0.0
    stats = ProfileStats(agl_min_by_strip=agl_min_by, agl_max_by_strip=agl_max_by, slope_max_rad=slope_max,
                         slope_min_rad=slope_min, n_points=len(waypoints), violations=violations,
                         infeasible_at=infeasible_at, infeasible_phase=infeasible_phase)
    return waypoints, stats


# ---------------------------------------------------------------------------
# 11.6 / 11.7 盤旋、起飛與降落走廊
# ---------------------------------------------------------------------------
@dataclass(slots=True)
class CorridorResult:
    """起飛：`segments` = seg1、seg2、seg2b(ACCEL)、(join+orbit)，**不含**最後 AVOID 進入段；`exit_state` = S_acc 或
    S_exit(ϑ*)。降落：`segments` = (orbit)、DECEL、TRANS_IN、LAND；`app_state` = S^app（FW_DECEL 起點）。"""
    segments: list[PathSegment2D]
    exit_state: UAVState
    app_state: Pose | None
    t_s: float
    e_wh: float
    n_laps: int
    orbit_center: np.ndarray | None
    issues: list[str]


def orbit_gain(spec: PlatformSpec, wind: WindField, z_m: float, auw_kg: float, R_orb_m: float,
               which: Literal["climb", "descent"]) -> tuple[float, float]:
    """盤旋每圈高度增益與圈時（規格 §11.6）：T_lap = 2πR/Va · ∮dθ/V_g（精確橢圓積分，`wind.circle_time_factor`）、
    Δh = Va sinγ cosφ_max · T_lap（弧段爬升折減）。A 無風 R=200：(61.8 m, 58.4 s)；B R=120：(35.8, 39.7)；B W=5：(37.8, 41.9)。"""
    if not R_orb_m > 0:
        raise ValueError("R_orb_m must be > 0")
    va = spec.cruise_airspeed_mps
    w = _wind_speed_at(wind, z_m)
    t_lap = TWO_PI * float(R_orb_m) / va * circle_time_factor(va, w)
    gamma = spec.gamma_c_auw(auw_kg) if which == "climb" else spec.gamma_d()
    if which not in ("climb", "descent"):
        raise ValueError("which must be 'climb' or 'descent'")
    dh = va * math.sin(gamma) * math.cos(math.radians(spec.max_bank_angle_deg)) * t_lap
    return float(dh), float(t_lap)


def _circle_line(center: np.ndarray, R: float, n: int = 48) -> LineString:
    a = np.linspace(0.0, TWO_PI, n + 1)
    return LineString(np.column_stack([center[0] + R * np.sin(a), center[1] + R * np.cos(a)]))


def _orbit_ok(center: np.ndarray, R: float, keepouts: KeepoutSet | None, link_poly: Polygon | None,
              z_hint_m: float, b_trk: float) -> bool:
    ring = _circle_line(center, R)
    if keepouts is not None and keepouts.intersects(ring, b_trk, z_hint_m):
        return False
    if link_poly is not None and not link_poly.is_empty and not link_poly.contains(ring.buffer(b_trk)):
        return False
    return True


def choose_orbit_center(near_xy: np.ndarray, prefer_dir_rad: float, R_orb_m: float, keepouts: KeepoutSet | None,
                        link_poly: Polygon | None, z_hint_m: float, cfg: PlannerConfig) -> np.ndarray | None:
    """盤旋圓心（規格 §11.6）：候選 = near_xy + d·(sinφ, cosφ)，d ∈ {R, 2R, 3R, 4R}、φ 自 prefer_dir 起每 30°
    （依與 prefer_dir 之角差排序）；圓（⊕ b_trk）不碰有效 keep-out 且在 C2 覆蓋內者優先；皆不合 → None。"""
    near = np.asarray(near_xy, dtype=float).reshape(2)
    R = float(R_orb_m)
    b = float(cfg.b_trk_m)
    offs = sorted(range(12), key=lambda k: min(k, 12 - k))
    for mult in (1.0, 2.0, 3.0, 4.0):
        for k in offs:
            phi = float(prefer_dir_rad) + k * math.pi / 6.0
            c = near + mult * R * _t_hat(phi)
            if _orbit_ok(c, R, keepouts, link_poly, z_hint_m, b):
                return c
    return None


def _orbit_path(entry: Pose, center: np.ndarray, R: float, turn: int, dtheta_abs: float) -> DubinsPath:
    """自 `entry`（在圓上、方位切線）沿 turn（+1 右／−1 左）轉 dtheta_abs 之單弧 DubinsPath（word ORBIT）。"""
    kind = "R" if turn > 0 else "L"
    dth = turn * dtheta_abs
    th_end = entry.theta_rad + dth
    end_xy = center - turn * R * _n_hat(th_end)
    end = Pose(float(end_xy[0]), float(end_xy[1]), float(th_end))
    seg = DubinsSegment(kind=kind, length_m=R * dtheta_abs, start=entry, end=end,
                        center=(float(center[0]), float(center[1])), dtheta_rad=dth)
    return DubinsPath(word=ORBIT_WORD, R_m=R, segments=(seg,), length_m=R * dtheta_abs)


def _corridor_heading(ctx: MissionContext, spec: PlatformSpec, site_id: str, site_xy: np.ndarray, z_ref_m: float,
                      d_tr: float, landing: bool) -> tuple[float, list[str]]:
    """轉換段方位（規格 §3.4、D19）：站點 `approach_heading_deg` → 逆風 β（W ≥ 2）→ 障礙最少方位（W < 2；
    以走廊線 site ↔ P_tc 至 keep-out 淨空最大者）。另回報側風／陣風限制違反（D8）。"""
    issues: list[str] = []
    site = ctx.inputs.sites.get(site_id)
    w = _wind_speed_at(ctx.wind, z_ref_m)
    beta = float(ctx.wind.dir_from_at(z_ref_m)) if ctx.wind is not None and ctx.wind.n_layers else 0.0
    if site is not None and site.approach_heading_deg is not None:
        theta = math.radians(float(site.approach_heading_deg))   # 進場與起飛皆逆風 → 同一方位
    elif w >= _CALM_WIND_MPS or ctx.keepouts is None or len(ctx.keepouts) == 0:
        theta = beta
    else:
        best, best_clr = beta, -1.0
        for k in range(12):
            th = k * math.pi / 6.0
            end = site_xy + (-1.0 if landing else 1.0) * d_tr * _t_hat(th)
            clr = ctx.keepouts.clearance_m(LineString([site_xy, end]), z_ref_m)
            if clr > best_clr + 1e-9:
                best, best_clr = th, clr
        theta = best
    theta = _mod2pi(theta)
    if w > 0:
        validated = spec.airspeed_sensor_validated
        x_max = spec.crosswind_site_max_mps[1 if validated else 0]
        cross = abs(w * math.sin(beta - theta))
        if cross > x_max + 1e-9:
            issues.append(f"crosswind_exceeds({site_id}: {cross:.1f} > {x_max:.1f} m/s)")
        g_max = spec.gust_site_max_mps[1 if validated else 0]
        gust = max(float(ctx.wind.gust_at(z_ref_m)) if ctx.wind is not None and ctx.wind.n_layers else 0.0,
                   float(ctx.inputs.weather_obs.gust_mps))
        if gust > g_max + 1e-9:
            issues.append(f"gust_exceeds({site_id}: {gust:.1f} > {g_max:.1f} m/s)")
    return theta, issues


def _transition_clearance_issues(ctx: MissionContext, spec: PlatformSpec, site_id: str, site_xy: np.ndarray,
                                 z_site: float, p_tc: np.ndarray, tc: Any, obstacles: Sequence[tuple[Any, float]] | None = None
                                 ) -> list[str]:
    """轉換走廊淨空（規格 §11.6/§11.8）：走廊 site ↔ P_tc、半寬 r_site + b_obs；障礙頂 + b_v 低於高度線
    H_ts + (H_tc,lo − H_ts)·x/d_tr（淨空用下緣，D29）。"""
    issues: list[str] = []
    site = ctx.inputs.sites.get(site_id)
    r_site = float(site.clear_radius_m) if (site is not None and site.clear_radius_m is not None) else spec.vtol_site_clear_radius_m
    hw = r_site + spec.obstacle_horizontal_buffer_m
    line = LineString([site_xy, p_tc])
    d_tr = float(np.linalg.norm(p_tc - site_xy))
    h_ts = spec.transition_start_agl_m
    obs = obstacles if obstacles is not None else ctx.obstacle_pairs()
    for k, (geom, top_z) in enumerate(obs):
        if geom is None or geom.is_empty or line.distance(geom) > hw:
            continue
        x = float(line.project(geom.centroid)) if d_tr > 0 else 0.0
        # 保守：取障礙在走廊內投影之最小 x（最靠近站點處高度線最低）
        try:
            nearest = shapely.ops.nearest_points(line, geom)[0]
            x = min(x, float(line.project(nearest)))
        except Exception:  # pragma: no cover
            pass
        line_z = z_site + h_ts + (tc.h_tc_clear_m - h_ts) * (x / d_tr if d_tr > 0 else 0.0)
        if float(top_z) + ctx.cfg.b_v_m > line_z:
            issues.append(f"transition_corridor_obstacle({site_id}: obstacle#{k} top {float(top_z):.1f}+b_v > line {line_z:.1f})")
    return issues


def climb_corridor_length_m(spec: PlatformSpec, wind_speed_mps: float, auw_kg: float | None = None, *,
                            side: Literal["climb", "descent"] = "climb") -> float:
    """CLIMB／DESCENT 走廊淨空長度 L = (h_minfw − H_tc,lo)/tanγ^g（規格 §11.1、§11.8；D29 淨空一律用下緣）。

    tanγ^g 以順風地速 V_a + W 取保守值。以平台參數表計算之結果（順風 5 m/s、參考 AUW）：

    ====== ============ ============ ============ =========== ===========
    構型   H_tc,lo (m)  h_minfw (m)  Δh (m)       L_climb (m)  L_descent (m)
    ====== ============ ============ ============ =========== ===========
    A      80           120          40           **942**      707
    B      70           100          30           **724**      543
    ====== ============ ============ ============ =========== ===========

    無風時 A 764 m／B 573 m。規格書 v1.1 §11.1、§11.8 的「A 940 m、B 480 m」中 **B 的 480 m 是錯的**：
    480 m 對應 Δh = 20 m，等於誤用了 Pioneer 的下緣 80 m 而非 HERO 的 70 m（20/0.041433 = 482.7）。
    正確值為 **724 m**（設計文件 docs/HV-DCCPP_設計文件.md:1233/:1383 已載明），本函式與
    :func:`_climb_corridor_issues` 皆以此為準。
    """
    tc = spec.transition_cmd()
    auw = float(spec.reference_auw_kg if auw_kg is None else auw_kg)
    tan_g = spec.tan_gamma_ground(side, spec.cruise_airspeed_mps + float(wind_speed_mps), auw, arc=False)
    dh = float(spec.min_fw_operation_agl_m) - float(tc.h_tc_clear_m)
    return float(dh / tan_g) if (tan_g > 0 and dh > 0) else 0.0


def _climb_corridor_issues(ctx: MissionContext, spec: PlatformSpec, site_id: str, p_tc: np.ndarray, theta: float,
                           z_site: float, tc: Any, auw_kg: float, side: Literal["climb", "descent"]) -> list[str]:
    """CLIMB／DESCENT 走廊淨空（規格 §11.8）：P_tc 後（起飛側）／前（降落側）(h_minfw − H_tc,lo)/tanγ^g 內、半寬 b_trk；
    地形 + h_clr 與障礙頂 + b_v 低於自 H_tc,lo 起以 γ^g 爬升之高度線（tan 取順風保守值）。"""
    issues: list[str] = []
    cfg = ctx.cfg
    z_tc = z_site + tc.h_tc_clear_m
    w = _wind_speed_at(ctx.wind, z_tc)
    tan_g = spec.tan_gamma_ground(side, spec.cruise_airspeed_mps + w, auw_kg, arc=False)
    L = climb_corridor_length_m(spec, w, auw_kg, side=side)          # A 942 m／B 724 m（順風 5 m/s；D29 用下緣）
    if L <= 0:
        return issues
    sgn = 1.0 if side == "climb" else -1.0
    n = max(2, int(math.ceil(L / cfg.d_d_m)) + 1)
    s = np.linspace(0.0, L, n)
    xy = p_tc[None, :] + sgn * s[:, None] * _t_hat(theta)[None, :]
    zg = ctx.terrain.raw_view().corridor_max(xy, np.full(n, theta), cfg.b_trk_m, _dv_of(ctx.terrain))
    line_z = z_tc + s * tan_g
    if np.isfinite(zg).any():
        bad = np.where(np.isfinite(zg), zg + ctx.h_clr_m > line_z + 1e-6, False)
        if bad.any():
            k = int(np.argmax(bad))
            issues.append(f"{side}_corridor_terrain({site_id}: terrain {zg[k]:.1f}+h_clr > line {line_z[k]:.1f} at {s[k]:.0f} m)")
    corridor = LineString(xy).buffer(cfg.b_trk_m)
    for k, (geom, top_z) in enumerate(ctx.obstacle_pairs()):
        if geom is None or geom.is_empty or not corridor.intersects(geom):
            continue
        d = float(LineString(xy).project(geom.centroid))
        try:
            d = min(d, float(LineString(xy).project(shapely.ops.nearest_points(LineString(xy), geom)[0])))
        except Exception:  # pragma: no cover
            pass
        if float(top_z) + cfg.b_v_m > z_tc + d * tan_g:
            issues.append(f"{side}_corridor_obstacle({site_id}: obstacle#{k})")
    return issues


def _climb_requirement(ctx: MissionContext, cls: str, spec: PlatformSpec, auw_kg: float, accel_seg: PathSegment2D | None,
                       path: DubinsPath, z_pin: float, z_end: float, v_air: float) -> float:
    """無釘住 ENVELOPE 後向遞迴之 b[0]（規格 §11.6）：對 [ACCEL, 進入路徑] 取樣、CLIMB 段界、z_lo 末端含 z_end；
    回傳所需起點高度 z*[0]（Δh_req = 該值 − z_pin）。"""
    cfg = ctx.cfg
    segs: list[PathSegment2D] = []
    if accel_seg is not None:
        segs.append(accel_seg)
    segs.append(PathSegment2D(Phase.ENTRY, Mode.CRUISE, path, None, None, v_air, False, z_pin, z_end))
    samples = resample_segments(segs, cfg.d_d_m, cfg.e_max_m, cfg.psi_max_rad)
    if not samples:
        return z_pin
    j_c, j_d = phase_boundaries(samples, cls, ctx, z_pin, z_end, auw_kg)
    z_lo, z_hi, _, tan_c, tan_d, ds = sample_bounds(samples, cls, ctx, {}, auw_kg, j_c, j_d)
    z_lo = np.where(np.isfinite(z_lo), z_lo, -np.inf)
    z_lo[-1] = max(z_lo[-1], float(z_end))
    lvl = np.array([p.phase in _LEVEL_PHASES for p in samples])
    z_lo[lvl] = min(z_pin, np.inf)
    env = envelope(z_lo, np.full(z_lo.size, np.inf), tan_c, tan_d, ds[:-1])
    return float(env.z_star[0])


def takeoff_corridor(site_id: str, uav: UAVInstance, ctx: MissionContext, S_entry: Pose, z_entry_m: float, t0_s: float,
                     entry_path: DubinsPath | None = None) -> CorridorResult:
    """TAKEOFF_CORRIDOR（規格 §11.6；D8/D29/D37）：seg1 垂直（H_ts, P_h）→ seg2 轉換（逆風 θ_tr、H_ts→H_tc,cmd 旋翼輔助）
    → seg2b FW_ACCEL（等高 d_acc）→ S_acc；爬升需求 Δh_req 以無釘住 ENVELOPE 後向遞迴 b[0] 對 `entry_path`
    （None → AVOID(S_acc→S_entry)，只用於判斷）；Δh_req > 0 → 爬升盤旋（圓心 choose_orbit_center、n_lap = ⌈Δh_req/Δh_lap⌉、
    出口 ϑ* = argmin[t(orbit→ϑ) + t(Dubins(S_exit(ϑ)→S_entry))] s.t. 自 S_exit 之包絡可行）。
    回傳 segments（不含最後進入段）、`exit_state`（S_acc 或 S_exit(ϑ*)，IDP 初始狀態）。"""
    cfg = ctx.cfg
    cls = uav.cls
    spec = ctx.specs[cls]
    model = ctx.models.get(uav.uav_id) or ctx.models_rep[cls]
    auw = float(uav.sortie.auw_kg)
    site_xy, z_site = ctx.sites_enu[site_id]
    site_xy = np.asarray(site_xy, dtype=float).reshape(2)
    tc = spec.transition_cmd(transition_profile=cfg.transition_profile)
    issues: list[str] = []
    if tc.slope_tr_rad > math.radians(spec.gamma_tr_max_deg) + 1e-9:
        issues.append(f"transition_slope_exceeds({math.degrees(tc.slope_tr_rad):.1f} > {spec.gamma_tr_max_deg} deg)")
    h_ts = spec.transition_start_agl_m
    z_ts = float(z_site) + h_ts
    z_tc = float(z_site) + tc.h_tc_cmd_m
    theta_tr, w_issues = _corridor_heading(ctx, spec, site_id, site_xy, z_tc, tc.d_tr_m, landing=False)
    issues += w_issues
    t_hat = _t_hat(theta_tr)
    p_tc = site_xy + tc.d_tr_m * t_hat
    p_acc = p_tc + tc.d_acc_m * t_hat
    va = spec.cruise_airspeed_mps
    seg1 = PathSegment2D(Phase.TAKEOFF, Mode.HOVER, site_xy.reshape(1, 2).copy(), None, None, 0.0, False, float(z_site), z_ts)
    seg2 = PathSegment2D(Phase.TRANS_OUT, Mode.TRANSITION, np.vstack([site_xy, p_tc]), None, None, tc.v_tr_mps, False, z_ts, z_tc)
    seg2b = PathSegment2D(Phase.ACCEL, Mode.CRUISE, np.vstack([p_tc, p_acc]), None, None, va, False, z_tc, z_tc)
    issues += _transition_clearance_issues(ctx, spec, site_id, site_xy, float(z_site), p_tc, tc)
    r_eff = _r_eff_for(ctx, spec, float(z_site), max(float(z_entry_m), z_tc))
    t_to = h_ts / spec.vtol_climb_rate_mps + spec.t_tr_out_s + tc.t_acc_s
    e_to = model.e_takeoff_wh()
    e_avail = model.eta_T * float(model.available_wh)
    s_acc = UAVState(float(p_acc[0]), float(p_acc[1]), theta_tr, z_tc, float(t0_s) + t_to, e_avail - e_to, r_eff, va)
    segments = [seg1, seg2, seg2b]
    # ---- 爬升需求 -------------------------------------------------------------
    s_acc_pose = Pose(float(p_acc[0]), float(p_acc[1]), theta_tr)
    path = entry_path
    tg_cache: dict = {}
    if path is None:
        try:
            path = avoid(s_acc_pose, S_entry, r_eff, ctx.keepouts, float(z_entry_m), ctx.wind, va, cfg.b_trk_m, tg_cache)
        except RoutingError as e:
            issues.append(f"entry_routing_failed({e})")
            path = None
    if path is None:
        return CorridorResult(segments, s_acc, None, t_to, e_to, 0, None, issues)
    z_req = _climb_requirement(ctx, cls, spec, auw, seg2b, path, z_tc, float(z_entry_m), va)
    dh_req = z_req - z_tc
    if dh_req <= _TOL_M:
        return CorridorResult(segments, s_acc, None, t_to, e_to, 0, None, issues)
    # ---- 爬升盤旋 -------------------------------------------------------------
    R_orb = r_eff
    z_mid = z_tc + 0.5 * dh_req
    dh_lap, t_lap = orbit_gain(spec, ctx.wind, z_mid, auw, R_orb, "climb")
    link_poly = ctx.link_poly.get(cls) if ctx.cfg.enforce_link else None
    center: np.ndarray | None = None
    turn = 1
    for sgn in (1, -1):
        c = p_acc + sgn * R_orb * _n_hat(theta_tr)
        if _orbit_ok(c, R_orb, ctx.keepouts, link_poly, z_mid, cfg.b_trk_m):
            center, turn = c, sgn
            break
    entry_pose = s_acc_pose
    join: DubinsPath | None = None
    if center is None:
        c = choose_orbit_center(p_acc, theta_tr + math.pi / 2.0, R_orb, ctx.keepouts, link_poly, z_mid, cfg)
        if c is None:
            issues.append("orbit_center_not_found")
            return CorridorResult(segments, s_acc, None, t_to, e_to, 0, None, issues)
        center = c
        # 切入點：圓上與 S_acc 最近之切線方向（右轉）
        rel = p_acc - center
        ang = math.atan2(rel[0], rel[1])
        turn = 1
        th_in = _mod2pi(ang + math.pi / 2.0)
        p_in = center - turn * R_orb * _n_hat(th_in)
        entry_pose = Pose(float(p_in[0]), float(p_in[1]), th_in)
        join = dubins_path(s_acc_pose, entry_pose, r_eff)
    n_lap = max(1, int(math.ceil(dh_req / dh_lap - 1e-9)))
    wxy_entry = _wind_vec_at(ctx.wind, float(z_entry_m))[0]
    best: tuple[float, float, float, Pose] | None = None      # (total_t, delta, z_exit, pose)
    for extra in range(_MAX_EXTRA_LAPS + 1):
        n_try = n_lap + extra
        deltas = np.linspace(0.0, TWO_PI, _N_EXIT_CANDIDATES, endpoint=False)
        th_ex = entry_pose.theta_rad + turn * deltas
        pts = center[None, :] - turn * R_orb * np.column_stack([np.cos(th_ex), -np.sin(th_ex)])
        exits = np.column_stack([pts, th_ex])
        ends = np.tile(S_entry.as_array(), (deltas.size, 1))
        t_path, _, _ = dubins_all_times(exits, ends, r_eff, va, wxy_entry)
        t_arc = np.array([time_arc(center, R_orb, entry_pose.theta_rad, turn * d, va, ctx.wind, z_mid) if d > 0 else 0.0
                          for d in deltas])
        total = t_path + t_arc
        order = np.argsort(total)
        for k in order:
            if not np.isfinite(total[k]):
                continue
            pose_k = Pose(float(exits[k, 0]), float(exits[k, 1]), float(exits[k, 2]))
            gained = dh_lap * (n_try + deltas[k] / TWO_PI)
            z_exit = z_tc + gained
            try:
                p_exit = avoid(pose_k, S_entry, r_eff, ctx.keepouts, float(z_entry_m), ctx.wind, va, cfg.b_trk_m, tg_cache)
            except RoutingError:
                continue
            z_need = _climb_requirement(ctx, cls, spec, auw, None, p_exit, z_exit, float(z_entry_m), va)
            if z_need <= z_exit + _TOL_M:
                best = (float(total[k]), float(deltas[k]), z_exit, pose_k)
                break
        if best is not None:
            n_lap = n_try
            break
    if best is None:
        issues.append("orbit_exit_infeasible")
        best = (0.0, 0.0, z_tc + dh_lap * n_lap, entry_pose)
    _, delta, z_exit, exit_pose = best
    dtheta_abs = TWO_PI * n_lap + delta
    orbit = _orbit_path(entry_pose, center, R_orb, turn, dtheta_abs)
    t_join = 0.0
    e_join = 0.0
    if join is not None and join.length_m > 1e-6:
        segments.append(PathSegment2D(Phase.ORBIT, Mode.CRUISE, join, None, None, va, False, z_tc, z_tc))
        t_join = path_time(join, va, ctx.wind, z_tc)
        if not math.isfinite(t_join):
            t_join = join.length_m / max(cfg.v_g_min_mps, 1.0)
        e_join = model.segment_wh(Mode.CRUISE, t_join)
    t_orb = path_time(orbit, va, ctx.wind, z_mid)
    if not math.isfinite(t_orb):
        t_orb = t_lap * (n_lap + delta / TWO_PI)
    gamma_c = spec.gamma_c_auw(auw)
    e_orb = model.segment_wh(Mode.CRUISE, t_orb, gamma_c)
    segments.append(PathSegment2D(Phase.ORBIT, Mode.CRUISE, orbit, None, None, va, False, z_tc, z_exit,
                                  n_laps=n_lap, orbit=(float(center[0]), float(center[1]), R_orb, int(turn))))
    exit_state = UAVState(exit_pose.x_m, exit_pose.y_m, _mod2pi(exit_pose.theta_rad), float(z_exit),
                          float(t0_s) + t_to + t_join + t_orb, e_avail - e_to - e_join - e_orb, r_eff, va)
    return CorridorResult(segments, exit_state, None, t_to + t_join + t_orb, e_to + e_join + e_orb, n_lap,
                          np.asarray(center, dtype=float), issues)


def landing_corridor(site_id: str, uav: UAVInstance, ctx: MissionContext, S_last: Pose, z_last_m: float) -> CorridorResult:
    """LANDING_CORRIDOR（規格 §11.7；D19）：θ_app = β（逆風進場）、P_tc = T − d_tr·t̂(θ_app)（下風側）、
    S^app = (P_tc − d_acc·t̂, θ_app)（FW_DECEL 起點 = 返航路徑目標）。下降距離：D_path·tan_d（順風保守）與
    前向遞迴 a[N] ≤ z_tc 精算；不足 → 下降盤旋（切於 S^app、Δh_lap,d = Va sinγ_d cosφ_max T_lap）。
    segments = (orbit)、DECEL（等高 H_tc）、TRANS_IN（H_tc→H_ts 旋翼輔助）、LAND（hover_descent_time）。"""
    cfg = ctx.cfg
    cls = uav.cls
    spec = ctx.specs[cls]
    model = ctx.models.get(uav.uav_id) or ctx.models_rep[cls]
    auw = float(uav.sortie.auw_kg)
    site_xy, z_site = ctx.sites_enu[site_id]
    site_xy = np.asarray(site_xy, dtype=float).reshape(2)
    tc = spec.transition_cmd(transition_profile=cfg.transition_profile)
    issues: list[str] = []
    h_ts = spec.transition_start_agl_m
    z_ts = float(z_site) + h_ts
    z_tc = float(z_site) + tc.h_tc_cmd_m
    theta_app, w_issues = _corridor_heading(ctx, spec, site_id, site_xy, z_tc, tc.d_tr_m, landing=True)
    issues += w_issues
    t_hat = _t_hat(theta_app)
    p_tc = site_xy - tc.d_tr_m * t_hat
    p_dec = p_tc - tc.d_acc_m * t_hat
    s_app = Pose(float(p_dec[0]), float(p_dec[1]), theta_app)
    va = spec.cruise_airspeed_mps
    issues += _transition_clearance_issues(ctx, spec, site_id, site_xy, float(z_site), p_tc, tc)
    r_eff = _r_eff_for(ctx, spec, float(z_site), max(float(z_last_m), z_tc))
    # ---- 下降距離 -------------------------------------------------------------
    n_lap = 0
    center: np.ndarray | None = None
    excess = 0.0
    dh_lap, t_lap = orbit_gain(spec, ctx.wind, z_tc, auw, r_eff, "descent")
    try:
        path = avoid(S_last, s_app, r_eff, ctx.keepouts, float(z_last_m), ctx.wind, va, cfg.b_trk_m, {})
    except RoutingError as e:
        issues.append(f"rtb_routing_failed({e})")
        path = None
    if path is not None:
        w = _wind_speed_at(ctx.wind, z_tc)
        tan_d_cons = spec.tan_gamma_ground("descent", va + w, auw, arc=False)
        need = float(z_last_m) - z_tc
        seg_rtb = PathSegment2D(Phase.RTB, Mode.CRUISE, path, None, None, va, False, float(z_last_m), z_tc)
        seg_dec0 = PathSegment2D(Phase.DECEL, Mode.CRUISE, np.vstack([p_dec, p_tc]), None, None, va, False, z_tc, z_tc)
        samples = resample_segments([seg_rtb, seg_dec0], cfg.d_d_m, cfg.e_max_m, cfg.psi_max_rad)
        j_c, j_d = phase_boundaries(samples, cls, ctx, float(z_last_m), z_tc, auw)
        z_lo, z_hi, _, tan_c, tan_d, ds = sample_bounds(samples, cls, ctx, {}, auw, j_c, j_d)
        z_lo = np.where(np.isfinite(z_lo), z_lo, -np.inf)
        lvl = np.array([p.phase in _LEVEL_PHASES for p in samples])
        z_lo[lvl] = z_tc
        env = envelope(z_lo, np.full(z_lo.size, np.inf), tan_c, tan_d, ds[:-1], pin_start=float(z_last_m))
        a_end = float(env.z_star[-1])
        excess_env = max(a_end - z_tc, 0.0)                              # 前向遞迴精算（含途中地形）
        excess_cons = max(need - path.length_m * tan_d_cons, 0.0)        # 順風保守距離（逆風之利不利用）
        excess = max(excess_env, excess_cons)
        if excess > _TOL_M:
            n_lap = int(math.ceil(excess / dh_lap - 1e-9))
    if n_lap > 0:
        link_poly = ctx.link_poly.get(cls) if cfg.enforce_link else None
        for sgn in (1, -1):
            c = p_dec + sgn * r_eff * _n_hat(theta_app)
            if _orbit_ok(c, r_eff, ctx.keepouts, link_poly, z_tc, cfg.b_trk_m):
                center = c
                turn = sgn
                break
        if center is None:
            issues.append("descent_orbit_center_not_found")
            n_lap = 0
    segments: list[PathSegment2D] = []
    t_orb = 0.0
    e_orb = 0.0
    if n_lap > 0 and center is not None:
        orbit = _orbit_path(s_app, center, r_eff, turn, TWO_PI * n_lap)
        t_orb = path_time(orbit, va, ctx.wind, z_tc)
        if not math.isfinite(t_orb):
            t_orb = t_lap * n_lap
        e_orb = model.segment_wh(Mode.CRUISE, t_orb, -spec.gamma_d())
        segments.append(PathSegment2D(Phase.ORBIT, Mode.CRUISE, orbit, None, None, va, False, z_tc + excess, z_tc,
                                      n_laps=n_lap, orbit=(float(center[0]), float(center[1]), r_eff, int(turn))))
    segments.append(PathSegment2D(Phase.DECEL, Mode.CRUISE, np.vstack([p_dec, p_tc]), None, None, va, False, z_tc, z_tc))
    segments.append(PathSegment2D(Phase.TRANS_IN, Mode.TRANSITION, np.vstack([p_tc, site_xy]), None, None, tc.v_tr_mps,
                                  False, z_tc, z_ts))
    segments.append(PathSegment2D(Phase.LAND, Mode.HOVER, site_xy.reshape(1, 2).copy(), None, None, 0.0, False, z_ts,
                                  float(z_site)))
    t_ldg = tc.t_acc_s + spec.t_tr_in_s + spec.hover_descent_time_s() + spec.t_hover_reserve_s
    e_ldg = model.e_landing_wh()
    exit_state = UAVState(float(site_xy[0]), float(site_xy[1]), theta_app, float(z_site), t_orb + t_ldg, 0.0, r_eff, 0.0)
    return CorridorResult(segments, exit_state, s_app, t_orb + t_ldg, e_ldg + e_orb, n_lap, center, issues)


# ---------------------------------------------------------------------------
# 11.8 站點檢查
# ---------------------------------------------------------------------------
def site_check(site_id: str, spec: PlatformSpec, ctx: MissionContext,
               obstacles_enu: list[tuple[shapely.Geometry, float]]) -> list[str]:
    """站點檢查（規格 §11.8、§6.2 步 5）；空 list = 通過。項目：r_site ≥ 翼展/2 + 2；DEM 坡度 ≤ site_max_slope；
    r_site 內無障礙；b_obs 內無高於 z_site + H_ts 之障礙；轉換走廊（起飛側與降落側）淨空（下緣）；CLIMB／DESCENT 走廊
    （半寬 b_trk，地形 + h_clr、障礙 + b_v 低於自 H_tc,lo 起之 γ^g 線）；下降盤旋碟形區（R_orb + b_trk）無 keep-out、在 C2 內；
    站點在 C2 內；站點風（側風、陣風）。"""
    issues: list[str] = []
    cfg = ctx.cfg
    cls = spec.cls
    if site_id not in ctx.sites_enu:
        return [f"site_unknown({site_id})"]
    site_xy, z_site = ctx.sites_enu[site_id]
    site_xy = np.asarray(site_xy, dtype=float).reshape(2)
    site = ctx.inputs.sites.get(site_id)
    r_site = float(site.clear_radius_m) if (site is not None and site.clear_radius_m is not None) else spec.vtol_site_clear_radius_m
    if r_site < 0.5 * spec.wingspan_m + 2.0 - 1e-9:
        issues.append(f"clear_radius_too_small({site_id}: {r_site:.1f} < {0.5 * spec.wingspan_m + 2.0:.1f})")
    slope = float(ctx.terrain.slope_at(site_xy.reshape(1, 2))[0])
    if not math.isfinite(slope):
        issues.append(f"site_outside_dem({site_id})")
    elif slope > cfg.site_max_slope_rad + 1e-9:
        issues.append(f"site_slope_exceeds({site_id}: {math.degrees(slope):.1f} > {cfg.site_max_slope_deg} deg)")
    pt = Point(site_xy)
    h_ts = spec.transition_start_agl_m
    for k, (geom, top_z) in enumerate(obstacles_enu):
        if geom is None or geom.is_empty:
            continue
        d = float(pt.distance(geom))
        if d < r_site:
            issues.append(f"obstacle_in_clear_radius({site_id}: obstacle#{k} at {d:.1f} m)")
        elif d < spec.obstacle_horizontal_buffer_m and float(top_z) > float(z_site) + h_ts:
            issues.append(f"obstacle_above_hts_in_buffer({site_id}: obstacle#{k} top {float(top_z):.1f})")
    tc = spec.transition_cmd(transition_profile=cfg.transition_profile)
    z_tc = float(z_site) + tc.h_tc_cmd_m
    auw = ctx.auw_kg.get(cls, spec.reference_auw_kg)
    for landing in (False, True):
        theta, w_issues = _corridor_heading(ctx, spec, site_id, site_xy, z_tc, tc.d_tr_m, landing=landing)
        if not landing:
            issues += w_issues
        sgn = -1.0 if landing else 1.0
        p_tc = site_xy + sgn * tc.d_tr_m * _t_hat(theta)
        issues += _transition_clearance_issues(ctx, spec, site_id, site_xy, float(z_site), p_tc, tc, obstacles_enu)
        issues += _climb_corridor_issues(ctx, spec, site_id, p_tc, theta, float(z_site), tc, auw,
                                         "descent" if landing else "climb")
        if landing:
            r_orb = _r_eff_for(ctx, spec, float(z_site), z_tc + spec.min_fw_operation_agl_m)
            p_dec = p_tc - tc.d_acc_m * _t_hat(theta)
            link_poly = ctx.link_poly.get(cls) if cfg.enforce_link else None
            ok = any(_orbit_ok(p_dec + s * r_orb * _n_hat(theta), r_orb, ctx.keepouts, link_poly, z_tc, cfg.b_trk_m)
                     for s in (1, -1))
            if not ok:
                issues.append(f"descent_orbit_blocked({site_id})")
    if cfg.enforce_link:
        lp = ctx.link_poly.get(cls)
        if lp is not None and not lp.is_empty and not lp.covers(pt):
            issues.append(f"site_outside_link({site_id}, {cls})")
    return issues


# ---------------------------------------------------------------------------
# 11.9 高度分層
# ---------------------------------------------------------------------------
def layering(cells: list[Cell], specs: dict[str, PlatformSpec], bands: dict[str, dict[str, Any]],
             cfg: PlannerConfig) -> dict[str, dict[str, float | str]]:
    """高度分層（規格 §11.9）：cell_id → {cls: h_op'}。判定 |h_A − h_B| ≥ Δh_sep + δ_prof；不足時依序
    提高 A（≤ h_hi,A）→ 降低 B（≥ h_lo,B）→ 反向（A 降至 h_B − need ≥ h_lo,A、或 B 升至 h_A + need ≤ h_hi,B）；
    仍不可行 → {"mode": "time_separation"}。`bands[cell][cls]` 鴨子型別（h_lo_m, h_hi_m, feasible）。
    只有單一構型可行之胞原樣回傳。示例 (150, 100) → A 160。"""
    need = float(cfg.dh_sep_m) + float(cfg.delta_prof_m)
    out: dict[str, dict[str, float | str]] = {}
    for cell in cells:
        cid = cell.cell_id
        bc = bands.get(cid, {})
        hs: dict[str, float] = {}
        for c in ("A", "B"):
            if c not in specs:
                continue
            feas = cell.feasible.get(c, False) if cell.feasible else False
            b = bc.get(c)
            if b is not None and not bool(getattr(b, "feasible", True)):
                feas = False
            if not feas:
                continue
            h = cell.h_op_m.get(c)
            if h is None:
                continue
            hs[c] = float(h)
        if len(hs) < 2:
            out[cid] = {c: h for c, h in hs.items()}
            continue
        h_a, h_b = hs["A"], hs["B"]
        if abs(h_a - h_b) >= need - 1e-9:
            out[cid] = {"A": h_a, "B": h_b}
            continue
        ba, bb = bc.get("A"), bc.get("B")
        lo_a = float(getattr(ba, "h_lo_m", h_a)) if ba is not None else h_a
        hi_a = float(getattr(ba, "h_hi_m", h_a)) if ba is not None else h_a
        lo_b = float(getattr(bb, "h_lo_m", h_b)) if bb is not None else h_b
        hi_b = float(getattr(bb, "h_hi_m", h_b)) if bb is not None else h_b
        res: dict[str, float | str] | None = None
        if h_b + need <= hi_a + 1e-9:
            res = {"A": h_b + need, "B": h_b}
        elif h_a - need >= lo_b - 1e-9:
            res = {"A": h_a, "B": h_a - need}
        elif hi_a >= lo_b + need - 1e-9:
            res = {"A": hi_a, "B": max(lo_b, hi_a - need)}
            if res["A"] - res["B"] < need - 1e-9:
                res = None
        if res is None:
            if h_b - need >= lo_a - 1e-9:
                res = {"A": h_b - need, "B": h_b}
            elif h_a + need <= hi_b + 1e-9:
                res = {"A": h_a, "B": h_a + need}
            elif hi_b >= lo_a + need - 1e-9:
                res = {"A": lo_a, "B": min(hi_b, lo_a + need)}
                if res["B"] - res["A"] < need - 1e-9:
                    res = None
        out[cid] = res if res is not None else {"mode": "time_separation"}
    return out
