"""hv_dccpp/costs.py — 三類路徑成本、能量預算表、成本快取（契約 §8；規格 §4.4、§5.5、§8.1–8.9、§10.3；D4、D14、D34）。

* 依賴（第 3 層，不得 import 更高層）：dubins / wind / geometry / geo / platform / context / routing / altitude / mission。
* 所有高度一律 ENU z（D42）；方位自北順時針；``dir −1`` = LR、``+1`` = RL、``idx = (dir+1)//2``；
  轉場型 ``k = 2·idx_from + idx_to``（0/3 同向 racetrack、1/2 反向；規格 §8.2）。
* 時間規則（規格 §8.1、D14）：直線 :math:`L/V_g(\\theta)`、弧段沿 :math:`V_g(\\theta)` 每 5° 積分
  （``dubins.dubins_all_times`` 向量化 / ``dubins.path_time``）；風於路徑起訖 ENU z 之中點取樣（路徑視為等高）。
* 高度差延長（規格 §8.4）：:math:`t\\leftarrow\\max(t(D),|\\Delta z|/(V_a\\sin\\gamma))`、螺旋圈數
  :math:`n_{lap}=\\lceil(|\\Delta z|-|D|\\tan\\gamma^g)/\\Delta h_{lap}\\rceil^+`（:math:`\\tan\\gamma^g` 取順風保守值）、
  能量含爬升／下降功率與 :math:`P_\\uparrow n_{lap}T_{lap}`（``EnergyModel.transfer_wh`` 同式）。
* 掃掠足跡（路徑 ⊕ b_trk）與 keep-out 衝突 → ``routing.avoid`` 繞行（AVOID；無解 → inf）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Sequence

import numpy as np
import shapely

from .altitude import envelope
from .context import MissionContext
from .dubins import WORDS, DubinsPath, Pose, dubins_all_times, dubins_path, path_time
from .geo import TerrainGrid
from .geometry import Strip
from .mission import PlannerConfig, UAVInstance, UAVState
from .platform import EnergyModel, PlatformSpec, Sensor
from .routing import AVOID_WORD, KeepoutSet, RoutingError, avoid, path_is_clear
from .wind import WindField, circle_time_factor, r_eff_wind_speed, worst_case_v_g

__all__ = [
    "PathCost", "op_cost", "entry_cost", "transfer_cost", "rtb_cost", "EnergyBudgetTable", "CostCache",
    "WORD_AVOID_IDX", "WORD_NONE_IDX", "R_EFF_REFRESH_TOL", "v_air_star", "resolve_site_entry",
]

TWO_PI: float = 2.0 * math.pi
WORD_AVOID_IDX: int = len(WORDS)          # 6：tr_word / entry_word 之 AVOID 繞行路徑標記
WORD_NONE_IDX: int = -1                   # 不可行（inf）
R_EFF_REFRESH_TOL: float = 0.05           # refresh_wind：R_eff(W_R) 變化 > 5% 才重算幾何層（規格 §8.9、§12.6）
_PATH_PAD_R_MULT: float = 4.0             # keep-out 候選篩選：Dubins 路徑必在起訖連線 ⊕ (4R + b_trk) 內（CCC 中弧 ≤ 4R）
_ZERO: float = 1e-9
_W_ROUND: int = 6                          # 風向量分組之四捨五入位數


# ---------------------------------------------------------------------------
# 基本工具
# ---------------------------------------------------------------------------
def _t_hat(theta: float) -> np.ndarray:
    return np.array([math.sin(theta), math.cos(theta)])


def _wind_vec_arr(wind: Any, z_m: np.ndarray | float) -> np.ndarray:
    """``WindField``（``wind_vec``）／(2,) 向量／None → 逐 z 之吹向向量 (N,2)。"""
    z = np.atleast_1d(np.asarray(z_m, dtype=float)).reshape(-1)
    if wind is None:
        return np.zeros((z.size, 2))
    if hasattr(wind, "wind_vec"):
        return np.asarray(wind.wind_vec(z), dtype=float).reshape(-1, 2)
    w = np.asarray(wind, dtype=float).reshape(2)
    return np.broadcast_to(w, (z.size, 2)).copy()


def _wind_max_speed(wind: Any) -> float:
    if wind is None:
        return 0.0
    if hasattr(wind, "max_speed"):
        return float(wind.max_speed())
    return float(np.hypot(*np.asarray(wind, dtype=float).reshape(2)))


def _pose_of(p: Any) -> Pose:
    if isinstance(p, Pose):
        return p
    if isinstance(p, UAVState):
        return Pose(float(p.x_m), float(p.y_m), float(p.theta_rad))
    a = np.asarray(p, dtype=float).reshape(3)
    return Pose(float(a[0]), float(a[1]), float(a[2]))


def _vg_array(theta: np.ndarray, v_air: np.ndarray | float, wind_xy: np.ndarray) -> np.ndarray:
    """風三角地速（規格 §3.2）向量版；``v_air`` 可逐點；|W⊥| ≥ V_a → nan。"""
    th = np.asarray(theta, dtype=float).reshape(-1)
    va = np.broadcast_to(np.asarray(v_air, dtype=float), th.shape)
    w = np.asarray(wind_xy, dtype=float)
    w = np.broadcast_to(w.reshape(-1, 2), (th.size, 2))
    st, ct = np.sin(th), np.cos(th)
    w_par = w[:, 0] * st + w[:, 1] * ct
    w_perp = w[:, 0] * ct - w[:, 1] * st
    rad = va * va - w_perp * w_perp
    with np.errstate(invalid="ignore"):
        return np.where(rad > 0.0, np.sqrt(np.where(rad > 0.0, rad, 0.0)) + w_par, np.nan)


def v_air_star(theta: np.ndarray | float, v_min: float, v_max: float, v_cap: np.ndarray | float,
               wind_xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """作業段空速選擇（規格 §4.4）：:math:`V_a^*=\\max\\{V_a\\in[V_{a,\\min},V_{a,c}]:V_g(\\theta,V_a)\\le V_g^{cap}\\}`。

    :math:`V_g` 對 :math:`V_a` 單調遞增 ⇒ :math:`V_g(V_{a,c})\\le cap` 時 :math:`V_a^*=V_{a,c}`；否則解
    :math:`\\sqrt{V_a^2-W_\\perp^2}+W_\\parallel=cap` ⇒ :math:`V_a=\\sqrt{(cap-W_\\parallel)^2+W_\\perp^2}`
    （:math:`cap<W_\\parallel` 或 :math:`V_a<V_{a,\\min}` → 不可行）。回傳 ``(V_a* (N,), V_g (N,))``，不可行為 nan。
    """
    th = np.asarray(theta, dtype=float).reshape(-1)
    cap = np.broadcast_to(np.asarray(v_cap, dtype=float), th.shape).astype(float)
    w = np.broadcast_to(np.asarray(wind_xy, dtype=float).reshape(-1, 2), (th.size, 2))
    st, ct = np.sin(th), np.cos(th)
    w_par = w[:, 0] * st + w[:, 1] * ct
    w_perp = w[:, 0] * ct - w[:, 1] * st
    vg_max = _vg_array(th, v_max, w)
    va = np.full(th.size, float(v_max))
    need = np.isfinite(vg_max) & (vg_max > cap)
    with np.errstate(invalid="ignore"):
        va_sol = np.sqrt(np.maximum(cap - w_par, 0.0) ** 2 + w_perp ** 2)
    va = np.where(need, va_sol, va)
    bad = ~np.isfinite(vg_max) | (need & ((cap < w_par) | (va_sol < v_min - _ZERO)))
    va = np.where(bad, np.nan, np.minimum(np.maximum(va, v_min), v_max))
    vg = _vg_array(th, np.where(bad, v_max, va), w)
    vg = np.where(bad, np.nan, vg)
    return va, vg


def _lap_gain(spec: PlatformSpec, auw_kg: float, R_m: float, W: np.ndarray | float,
              which: str) -> tuple[np.ndarray, np.ndarray]:
    """盤旋每圈高度增益與圈時（同 ``altitude.orbit_gain``：T = 2πR/Va·factor(Va, W)、Δh = Va sinγ cosφ_max T），
    向量化於風速 ``W``；接受任何風型別（只需風速）。"""
    Wv = np.atleast_1d(np.asarray(W, dtype=float)).reshape(-1)
    va = spec.cruise_airspeed_mps
    gamma = spec.gamma_c_auw(auw_kg) if which == "climb" else spec.gamma_d()
    T = np.empty(Wv.size)
    for w in np.unique(Wv):
        T[Wv == w] = TWO_PI * float(R_m) / va * circle_time_factor(va, float(w))
    dh = va * math.sin(gamma) * math.cos(math.radians(spec.max_bank_angle_deg)) * T
    return dh, T


def _dz_extension(spec: PlatformSpec, auw_kg: float, R_m: float, W: np.ndarray, t_path: np.ndarray, length: np.ndarray,
                  dz: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """規格 §8.4 高度差延長（向量化）。回傳 ``(t_ext, n_lap, T_lap)``：

    :math:`t_{ext}=\\max(t(D),|\\Delta z|/(V_a\\sin\\gamma))`、
    :math:`n_{lap}=\\lceil(|\\Delta z|-|D|\\tan\\gamma^g)/\\Delta h_{lap}\\rceil^+`，:math:`\\tan\\gamma^g` 以順風最大
    :math:`V_g=V_a+W`（保守，同 ``climb_dist_m``）；爬升用 γ_c(m)、下降用 γ_d；``t_ext`` **不含** 圈時（呼叫端加 n·T）。
    """
    va = spec.cruise_airspeed_mps
    gc = spec.gamma_c_auw(auw_kg)
    gd = spec.gamma_d()
    W = np.broadcast_to(np.asarray(W, dtype=float), np.shape(dz)).astype(float)
    dz = np.asarray(dz, dtype=float)
    up = dz > 0.0
    adz = np.abs(dz)
    t_min = np.where(up, adz / (va * math.sin(gc)), adz / (va * math.sin(gd)))
    vg_tail = va + np.maximum(W, 0.0)
    tan_c_g = spec.climb_rate_mps(auw_kg) / vg_tail
    tan_d_g = va * math.sin(gd) / vg_tail
    need = np.where(up, adz - length * tan_c_g, adz - length * tan_d_g)
    dh_c, T_c = _lap_gain(spec, auw_kg, R_m, W.ravel(), "climb")
    dh_d, T_d = _lap_gain(spec, auw_kg, R_m, W.ravel(), "descent")
    dh = np.where(up.ravel(), dh_c, dh_d).reshape(dz.shape)
    T = np.where(up.ravel(), T_c, T_d).reshape(dz.shape)
    with np.errstate(invalid="ignore"):
        n = np.where(need > _ZERO, np.ceil(np.maximum(need, 0.0) / dh - 1e-9), 0.0)
    n = np.where(np.isfinite(t_path), n, 0.0).astype(int)
    t_ext = np.maximum(t_path, t_min)
    return t_ext, n, T


def _transfer_wh_vec(model: EnergyModel, t_s: np.ndarray, dz_m: np.ndarray, n_laps: np.ndarray | int = 0,
                     t_lap_s: np.ndarray | float = 0.0) -> np.ndarray:
    """``EnergyModel.transfer_wh`` 之向量版（規格 §8.4）：
    :math:`k_{unc}[P_c t+(P_\\uparrow-P_c)\\max(0,\\Delta z)/(V_a\\sin\\gamma_c)-(P_c-P_\\downarrow)\\max(0,-\\Delta z)/(V_a\\sin\\gamma_d)+P_\\uparrow n_{lap}T_{lap}]`。"""
    pc = model.p_cruise_w()
    gc = model.spec.gamma_c_auw(model.auw_kg)
    gd = model.spec.gamma_d()
    p_up, p_dn = model.p_climb_w(gc), model.p_descent_w(gd)
    va = model.spec.cruise_airspeed_mps
    t = np.asarray(t_s, dtype=float)
    dz = np.asarray(dz_m, dtype=float)
    e = pc * t + (p_up - pc) * np.maximum(0.0, dz) / (va * math.sin(gc)) \
        - (pc - p_dn) * np.maximum(0.0, -dz) / (va * math.sin(gd)) \
        + p_up * np.asarray(n_laps, dtype=float) * np.asarray(t_lap_s, dtype=float)
    e = e * model.k_unc
    return e if model.base_minute_mode else e / 3600.0


def _path_vg_ok(path: DubinsPath, v_air: float, wind_xy: np.ndarray, v_g_min: float, step_rad: float = 0.0873) -> bool:
    """路徑沿線所有航向之 V_g ≥ cfg.v_g_min（規格 §3.2 可行條件；直線取其方位、弧段每 5° 取樣）。"""
    ths: list[np.ndarray] = []
    for seg in path.segments:
        if seg.length_m <= _ZERO:
            continue
        if seg.kind == "S":
            ths.append(np.array([seg.start.theta_rad]))
        else:
            n = max(1, int(math.ceil(abs(seg.dtheta_rad) / step_rad)))
            ths.append(seg.start.theta_rad + seg.dtheta_rad * np.linspace(0.0, 1.0, n + 1))
    if not ths:
        return True
    vg = _vg_array(np.concatenate(ths), v_air, wind_xy)
    return bool(np.all(np.isfinite(vg)) and np.all(vg >= v_g_min - _ZERO))


def _keepout_candidates(keepouts: KeepoutSet | None, starts: np.ndarray, ends: np.ndarray, R_m: float,
                        half_width_m: float, z_hint: np.ndarray) -> np.ndarray:
    """可能與有效 keep-out 衝突之 pair 索引（起訖連線 ⊕ (4R + half_width + 最大緩衝) 之 STRtree 批次查詢）。"""
    if keepouts is None or len(keepouts) == 0 or starts.shape[0] == 0:
        return np.zeros(0, dtype=int)
    tops = np.asarray(keepouts.tops_z, dtype=float)
    floors = np.asarray(keepouts.floors_z, dtype=float)
    coords = np.stack([starts[:, :2], ends[:, :2]], axis=1)
    lines = shapely.linestrings(coords)
    dist = _PATH_PAD_R_MULT * float(R_m) + float(half_width_m) + max(keepouts.buffers_m, default=0.0)
    hits = keepouts.tree.query(lines, predicate="dwithin", distance=dist)
    if hits.size == 0:
        return np.zeros(0, dtype=int)
    li, ki = hits[0], hits[1]
    act = (floors[ki] <= z_hint[li]) & (z_hint[li] < tops[ki])
    return np.unique(li[act])


def _pair_costs(starts: np.ndarray, ends: np.ndarray, R_m: float, z_start: np.ndarray, z_end: np.ndarray, v_air: float,
                wind: Any, keepouts: KeepoutSet | None, cfg: PlannerConfig, paths: dict[Any, DubinsPath],
                keys: Sequence[Any], geometry: bool, tg_cache: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """M 對狀態之（路徑時間、長度、word 索引、風速 W）；規格 §8.1 時間規則，向量化。

    * 幾何層：``dubins_all_times``（依起訖 z 中點之風向量分組）；keep-out 候選對以 6-word 最短路徑掃掠足跡
      （⊕ b_trk）檢查，衝突者以 ``routing.avoid`` 繞行並存入 ``paths[keys[i]]``（word = AVOID），無解 → inf。
    * ``geometry=False``：不重算繞行路徑，只對既有 ``paths`` 重算時間（風更新之時間層）。
    * V_g 下限（cfg.v_g_min）：同組風速 :math:`V_a-W<V_{g,\\min}` 時該組全部 inf（轉場必掃過逆風航向，保守）；
      繞行路徑逐段檢查。
    """
    M = starts.shape[0]
    t = np.full(M, np.inf)
    L = np.full(M, np.inf)
    word = np.full(M, WORD_NONE_IDX, dtype=np.int8)
    Wsp = np.zeros(M)
    if M == 0:
        return t, L, word, Wsp
    z_mid = 0.5 * (np.asarray(z_start, dtype=float) + np.asarray(z_end, dtype=float))
    wxy = _wind_vec_arr(wind, z_mid)
    Wsp = np.hypot(wxy[:, 0], wxy[:, 1])
    uniq, inv = np.unique(np.round(wxy, _W_ROUND), axis=0, return_inverse=True)
    inv = np.asarray(inv).reshape(-1)
    for g in range(uniq.shape[0]):
        sel = np.flatnonzero(inv == g)
        w = uniq[g]
        tt, LL, ww = dubins_all_times(starts[sel], ends[sel], R_m, v_air, w, cfg.dtheta_int_rad)
        if v_air - float(np.hypot(w[0], w[1])) < cfg.v_g_min_mps:
            tt = np.full_like(tt, np.inf)
        t[sel], L[sel], word[sel] = tt, LL, ww
    if geometry:
        for key in keys:
            paths.pop(key, None)
        z_hint = np.minimum(np.asarray(z_start, dtype=float), np.asarray(z_end, dtype=float))
        cand = _keepout_candidates(keepouts, starts, ends, R_m, cfg.b_trk_m, z_hint)
        for i in cand:
            i = int(i)
            if not np.isfinite(L[i]):
                continue
            a = Pose(float(starts[i, 0]), float(starts[i, 1]), float(starts[i, 2]))
            b = Pose(float(ends[i, 0]), float(ends[i, 1]), float(ends[i, 2]))
            direct = dubins_path(a, b, R_m)
            if path_is_clear(direct, keepouts, cfg.b_trk_m, float(z_hint[i])):
                continue
            try:
                paths[keys[i]] = avoid(a, b, R_m, keepouts, float(z_hint[i]), wxy[i], v_air, cfg.b_trk_m, tg_cache)
            except RoutingError:
                t[i], L[i], word[i] = np.inf, np.inf, WORD_NONE_IDX
    if paths:
        key_index = {k: i for i, k in enumerate(keys)}
        for key, p in paths.items():
            i = key_index.get(key)
            if i is None:
                continue
            L[i] = p.length_m
            # avoid 可能回傳「非最短但淨空」之 6-word 路徑（word ∈ WORDS）或切線圖繞行（AVOID）
            word[i] = WORDS.index(p.word) if p.word in WORDS else WORD_AVOID_IDX
            if _path_vg_ok(p, v_air, wxy[i], cfg.v_g_min_mps, cfg.dtheta_int_rad):
                t[i] = path_time(p, v_air, wxy[i], float(z_mid[i]), cfg.dtheta_int_rad)
            else:
                t[i] = np.inf
    return t, L, word, Wsp


def _sample_path(path: DubinsPath, ds_m: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """沿路徑每 ≤ ds 取樣：``(xy (N,2), theta (N,), arc (N,) bool, ds (N−1,))``（含起訖點；弧段 ds 為真實弧長）。"""
    st = path.start
    xs = [np.array([st.x_m])]
    ys = [np.array([st.y_m])]
    ths = [np.array([st.theta_rad])]
    arcs = [np.array([False])]
    dss: list[np.ndarray] = []
    for seg in path.segments:
        if seg.length_m <= _ZERO:
            continue
        n = max(1, int(math.ceil(seg.length_m / ds_m - 1e-12)))
        tau = np.linspace(0.0, 1.0, n + 1)[1:]
        x0, y0, th0 = seg.start.x_m, seg.start.y_m, seg.start.theta_rad
        if seg.kind == "S":
            x = x0 + tau * seg.length_m * math.sin(th0)
            y = y0 + tau * seg.length_m * math.cos(th0)
            th = np.full(n, th0)
        else:
            sign = 1.0 if seg.kind == "R" else -1.0
            R = seg.length_m / abs(seg.dtheta_rad)
            cx, cy = seg.center  # type: ignore[misc]
            th = th0 + seg.dtheta_rad * tau
            x = cx - sign * R * np.cos(th)
            y = cy + sign * R * np.sin(th)
        xs.append(x)
        ys.append(y)
        ths.append(th)
        arcs.append(np.full(n, seg.kind != "S"))
        dss.append(np.full(n, seg.length_m / n))
    xy = np.column_stack([np.concatenate(xs), np.concatenate(ys)])
    ds = np.concatenate(dss) if dss else np.zeros(0)
    return xy, np.concatenate(ths), np.concatenate(arcs), ds


def resolve_site_entry(sites: dict[str, Any], site_id: str, cls: str) -> tuple[Pose, float]:
    """``sites[site_id]`` 可為 ``(Pose S^app, z_tc)`` 或 ``{cls: (Pose, z_tc)}``（S^app 之 d_tr/d_acc 隨構型不同）。"""
    v = sites[site_id]
    if isinstance(v, dict):
        if cls not in v:
            raise KeyError(f"sites[{site_id!r}] has no entry for class {cls!r}")
        v = v[cls]
    pose, z = v
    return _pose_of(pose), float(z)


def _end_start_poses(strips: Sequence[Strip]) -> tuple[np.ndarray, np.ndarray]:
    """``(P_end (n,2,3), P_start (n,2,3))``：``[l, idx]``，idx 0 ↔ dir −1（LR）、1 ↔ dir +1（RL）（規格 §8.2）。"""
    n = len(strips)
    P_end = np.zeros((n, 2, 3))
    P_start = np.zeros((n, 2, 3))
    for i, s in enumerate(strips):
        for idx, d in enumerate((-1, 1)):
            P_end[i, idx] = s.end_state(d).as_array()
            P_start[i, idx] = s.start_state(d).as_array()
    return P_end, P_start


# ---------------------------------------------------------------------------
# PathCost 與三類單筆成本
# ---------------------------------------------------------------------------
@dataclass(slots=True)
class PathCost:
    """單筆路徑成本：時間 s、能量 Wh、長度 m、路徑（作業段為 None）、螺旋／盤旋圈數、高度差（ENU）。"""
    t_s: float
    e_wh: float
    length_m: float
    path: DubinsPath | None
    n_laps: int = 0
    dz_m: float = 0.0


_INF_COST = PathCost(math.inf, math.inf, math.inf, None, 0, 0.0)


def _v_min_of(spec: PlatformSpec, auw_kg: float) -> float:
    """作業段空速下限 V_a,min(m) = max(v_min_mission, 1.2·V_s(m))（規格 §3.2、§4.4）。"""
    return max(spec.v_min_mission(), 1.2 * spec.stall_speed(auw_kg))


def _v_max_of(spec: PlatformSpec) -> float:
    """作業段空速上限 = V_a,c（``max_airspeed_mps`` 只作上限檢查，D10）。"""
    va = spec.cruise_airspeed_mps
    return min(va, spec.max_airspeed_mps) if spec.max_airspeed_mps is not None else va


def op_cost(strip: Strip, dir: int, spec: PlatformSpec, sensor: Sensor, wind: WindField, energy: EnergyModel,
            cfg: PlannerConfig) -> PathCost:
    """作業段成本（規格 §4.4、§8.2 :math:`\\vec t^{oper}`）：V_a* 選擇後 :math:`t=|L_l|/V_g(\\theta_{dir})`；
    :math:`V_g^{cap}` = ``sensor.max_ground_speed_mps(h_min)``（保守 AGL）；V_g < cfg.v_g_min 或該向不可行 → inf。
    能量 = P_c·t·k_unc（巡航功率，等高）。``path=None``（直線）。"""
    if dir not in (-1, 1):
        raise ValueError("dir must be -1 or +1")
    theta = strip.theta_LR_rad if dir == -1 else strip.theta_RL_rad
    wxy = _wind_vec_arr(wind, strip.z_op_m)[0]
    cap = float(sensor.max_ground_speed_mps(strip.h_min_m)) if sensor is not None else math.inf
    va, vg = v_air_star(theta, _v_min_of(spec, energy.auw_kg), _v_max_of(spec), cap, wxy)
    if not np.isfinite(vg[0]) or vg[0] < cfg.v_g_min_mps:
        return PathCost(math.inf, math.inf, float(strip.length_m), None, 0, 0.0)
    t = float(strip.length_m) / float(vg[0])
    e = energy.segment_wh("CRUISE", t)
    return PathCost(t, float(e), float(strip.length_m), None, 0, 0.0)


def _route_cost(start: Pose, end: Pose, R_m: float, z_start: float, z_end: float, spec: PlatformSpec, wind: Any,
                energy: EnergyModel, keepouts: KeepoutSet | None, cfg: PlannerConfig,
                laps_given: tuple[int, float] | None = None, tg_cache: dict | None = None) -> PathCost:
    """單對狀態之轉移／進入／返航成本（規格 §8.1、§8.4、§8.6）。``laps_given=(n, T)``（暫定走廊之盤旋）給定且 n>0
    時取代 §8.4 之螺旋估計。"""
    if tg_cache is None:
        tg_cache = {}
    z_start, z_end = float(z_start), float(z_end)
    z_mid = 0.5 * (z_start + z_end)
    z_hint = min(z_start, z_end)
    va = spec.cruise_airspeed_mps
    wxy = _wind_vec_arr(wind, z_mid)[0]
    W = float(np.hypot(wxy[0], wxy[1]))
    try:
        path = avoid(start, end, R_m, keepouts, z_hint, wxy, va, cfg.b_trk_m, tg_cache)
    except RoutingError:
        return PathCost(math.inf, math.inf, math.inf, None, 0, z_end - z_start)
    if not _path_vg_ok(path, va, wxy, cfg.v_g_min_mps, cfg.dtheta_int_rad):
        return PathCost(math.inf, math.inf, float(path.length_m), path, 0, z_end - z_start)
    t_path = path_time(path, va, wxy, z_mid, cfg.dtheta_int_rad)
    if not math.isfinite(t_path):
        return PathCost(math.inf, math.inf, float(path.length_m), path, 0, z_end - z_start)
    dz = np.array([z_end - z_start])
    t_ext, n, T = _dz_extension(spec, energy.auw_kg, R_m, np.array([W]), np.array([t_path]), np.array([path.length_m]), dz)
    n_lap, t_lap = int(n[0]), float(T[0])
    if laps_given is not None and int(laps_given[0]) > 0:
        n_lap, t_lap = int(laps_given[0]), float(laps_given[1])
    t_total = float(t_ext[0]) + n_lap * t_lap
    e = float(_transfer_wh_vec(energy, t_ext, dz, n_lap, t_lap)[0])
    return PathCost(t_total, e, float(path.length_m), path, n_lap, float(dz[0]))


def entry_cost(state: UAVState, strip: Strip, dir: int, spec: PlatformSpec, wind: WindField, energy: EnergyModel,
               keepouts: KeepoutSet | None, cfg: PlannerConfig, n_laps: int = 0, t_lap_s: float = 0.0) -> PathCost:
    """進入成本（規格 §8.2 :math:`\\vec t^{in}`、§8.4、§11.6）：``state``（S_acc 或 S_exit，R = state.r_eff_m）→
    ``strip.start_state(dir)``（dir −1 → S^L 後 LR 掃；+1 → S^R 後 RL 掃）。``n_laps·t_lap_s``（暫定走廊之爬升盤旋）
    計入時間與能量（P↑）；n_laps = 0 時改用 §8.4 之螺旋估計。"""
    if dir not in (-1, 1):
        raise ValueError("dir must be -1 or +1")
    return _route_cost(_pose_of(state), strip.start_state(dir), float(state.r_eff_m), float(state.z_m), float(strip.z_op_m),
                       spec, wind, energy, keepouts, cfg, laps_given=(int(n_laps), float(t_lap_s)))


def transfer_cost(strip_from: Strip, dir_from: int, strip_to: Strip, dir_to: int, spec: PlatformSpec, wind: WindField,
                  energy: EnergyModel, keepouts: KeepoutSet | None, cfg: PlannerConfig,
                  r_eff_m: float | None = None) -> PathCost:
    """四型轉場（規格 §8.2；D4、D34）：``strip_from.end_state(dir_from)`` → ``strip_to.start_state(dir_to)``，
    R = ``r_eff_m``（None → spec.r_eff(W at z_mid)）；含 §8.4 高度差延長（螺旋）與爬升／下降功率；掃掠足跡 ⊕ b_trk
    衝突 → AVOID。``cfg.transfer_types`` 不含 "same" 時同向型（dir_from == dir_to）→ inf。"""
    for d in (dir_from, dir_to):
        if d not in (-1, 1):
            raise ValueError("dir must be -1 or +1")
    if strip_from.strip_id == strip_to.strip_id:
        return PathCost(math.inf, math.inf, math.inf, None, 0, 0.0)
    if dir_from == dir_to and "same" not in cfg.transfer_types:
        return PathCost(math.inf, math.inf, math.inf, None, 0, float(strip_to.z_op_m - strip_from.z_op_m))
    if r_eff_m is None:
        wxy = _wind_vec_arr(wind, 0.5 * (strip_from.z_op_m + strip_to.z_op_m))[0]
        r_eff_m = spec.r_eff(float(np.hypot(wxy[0], wxy[1])))
    return _route_cost(strip_from.end_state(dir_from), strip_to.start_state(dir_to), float(r_eff_m),
                       float(strip_from.z_op_m), float(strip_to.z_op_m), spec, wind, energy, keepouts, cfg)


def rtb_cost(strip: Strip, dir_last: int, site_app: Pose, z_site_tc: float, spec: PlatformSpec, wind: WindField,
             energy: EnergyModel, keepouts: KeepoutSet | None, cfg: PlannerConfig,
             r_eff_m: float | None = None) -> PathCost:
    """返航路徑（規格 §8.7 第四類）：``strip.end_state(dir_last)`` → S^app（FW_DECEL 起點，方位 β、高度 z_tc）。
    ``t_s`` = t(AVOID) + 下降螺旋 :math:`t_\\downarrow^{extra}`（**不含** t_acc + t_tr_in + t_dn + t_hov,res，由降落走廊
    ／allocate 另計）；``e_wh`` 依 §8.4（下降側；不含 e^LDG）。"""
    if dir_last not in (-1, 1):
        raise ValueError("dir_last must be -1 or +1")
    if r_eff_m is None:
        wxy = _wind_vec_arr(wind, 0.5 * (strip.z_op_m + z_site_tc))[0]
        r_eff_m = spec.r_eff(float(np.hypot(wxy[0], wxy[1])))
    return _route_cost(strip.end_state(dir_last), _pose_of(site_app), float(r_eff_m), float(strip.z_op_m),
                       float(z_site_tc), spec, wind, energy, keepouts, cfg)


# ---------------------------------------------------------------------------
# EnergyBudgetTable（規格 §5.5）
# ---------------------------------------------------------------------------
class EnergyBudgetTable:
    """備降能量表 (cls, strip, idx, site) → Wh：**真實值**（AVOID + 無釘住 ENVELOPE，含地形／稜線爬升、下降螺旋、
    P↑／P↓、k_unc、逆風）+ e^LDG；以 ``models_rep`` 建表（保守）。規格 §5.5。

    ``sites[site_id]`` = ``(Pose S^app, z_tc)`` 或 ``{cls: (Pose, z_tc)}``（見 :func:`resolve_site_entry`）。
    額外關鍵字：``h_clr_m``（地形淨空；契約簽章未含，預設 0）、``tg_cache``（與 CostCache 共用）、``d_sample_m``
    （剖面取樣間距，預設 cfg.d_d_m）。

    剖面規則：走廊 ⊕ b_trk 內最高地形 + max(h_minfw, h_clr)（D35：末端下降段 — 4° 下降線已低於 z_g + h_minfw 處 —
    只含 h_clr）；``envelope(pin_start = z_op)``；起點需求 z*[0] > z_op → 爬升螺旋 n↑ = ⌈Δ/Δh_lap,c⌉；末端 z*[N] > z_tc
    → 下降螺旋 n↓ = ⌈Δ/Δh_lap,d⌉（規格 §8.7 :math:`t_\\downarrow^{extra}`）。
    能量 = k_unc[P_c t + (P↑−P_c)Δz⁺/(Va sinγ_c) − (P_c−P↓)Δz⁻/(Va sinγ_d) + P↑ n↑ T + P↓ n↓ T] + e^LDG。
    """

    def __init__(self, strips: dict[str, dict[str, Strip]], sites: dict[str, Any], models_rep: dict[str, EnergyModel],
                 specs: dict[str, PlatformSpec], wind: WindField, keepouts: KeepoutSet | None, terrain: TerrainGrid,
                 cfg: PlannerConfig, *, h_clr_m: float = 0.0, tg_cache: dict | None = None,
                 d_sample_m: float | None = None, r_eff_m: dict[str, float] | None = None) -> None:
        self.sites: dict[str, Any] = dict(sites)
        self.site_ids: list[str] = list(sites.keys())
        self.models_rep = models_rep
        self.specs = specs
        self.wind = wind
        self.keepouts = keepouts
        self.terrain = terrain
        self.cfg = cfg
        self.h_clr_m = float(h_clr_m)
        self.tg_cache: dict = {} if tg_cache is None else tg_cache
        self.d_sample_m = float(cfg.d_d_m if d_sample_m is None else d_sample_m)
        self.r_eff_m: dict[str, float] = dict(r_eff_m or {})
        self.strips: dict[str, dict[str, Strip]] = {c: dict(d) for c, d in strips.items()}
        self.strip_ids: dict[str, list[str]] = {c: list(d.keys()) for c, d in strips.items()}
        self.idx: dict[str, dict[str, int]] = {c: {s: i for i, s in enumerate(ids)} for c, ids in self.strip_ids.items()}
        self.wh: dict[str, np.ndarray] = {}          # cls → (n, 2, n_sites)
        self.t: dict[str, np.ndarray] = {}           # cls → (n, 2, n_sites)
        self.laps: dict[str, np.ndarray] = {}        # cls → (n, 2, n_sites, 2) [n_up, n_down]
        self.paths: dict[tuple[str, str, str, int], DubinsPath] = {}   # (cls, site, strip_id, idx)
        for c in self.strip_ids:
            self._compute(c, self.strip_ids[c], geometry=True)

    # -- 建表 -------------------------------------------------------------------
    def _r_eff(self, cls: str) -> float:
        if cls in self.r_eff_m:
            return self.r_eff_m[cls]
        return self.specs[cls].r_eff(_wind_max_speed(self.wind))

    def _route(self, cls: str, key: tuple[str, str, str, int], start: Pose, z_start: float, s_app: Pose, z_tc: float,
               geometry: bool) -> tuple[float, float, int, int]:
        """單筆 (t_s, wh, n_up, n_down)；無路徑 → inf。"""
        spec = self.specs[cls]
        model = self.models_rep[cls]
        cfg = self.cfg
        va = spec.cruise_airspeed_mps
        R = self._r_eff(cls)
        z_mid = 0.5 * (z_start + z_tc)
        wxy = _wind_vec_arr(self.wind, z_mid)[0]
        W = float(np.hypot(wxy[0], wxy[1]))
        path = self.paths.get(key)
        if geometry or path is None:
            try:
                path = avoid(start, s_app, R, self.keepouts, min(z_start, z_tc), wxy, va, cfg.b_trk_m, self.tg_cache)
            except RoutingError:
                self.paths.pop(key, None)
                return math.inf, math.inf, 0, 0
            self.paths[key] = path
        if not _path_vg_ok(path, va, wxy, cfg.v_g_min_mps, cfg.dtheta_int_rad):
            return math.inf, math.inf, 0, 0
        t_path = path_time(path, va, wxy, z_mid, cfg.dtheta_int_rad)
        if not math.isfinite(t_path):
            return math.inf, math.inf, 0, 0
        # ---- 剖面（無釘住 ENVELOPE）--------------------------------------------
        xy, th, arc, ds = _sample_path(path, self.d_sample_m)
        N = xy.shape[0]
        auw = model.auw_kg
        vg = _vg_array(th, va, wxy)
        ok = np.isfinite(vg) & (vg >= cfg.v_g_min_mps)
        kappa = np.where(arc, math.cos(math.radians(spec.max_bank_angle_deg)), 1.0)
        with np.errstate(divide="ignore", invalid="ignore"):
            tan_c = np.where(ok, spec.climb_rate_mps(auw) * kappa / vg, 0.0)
            tan_d = np.where(ok, va * math.sin(spec.gamma_d()) / vg, 0.0)
        dv = max(0.5 * float(self.terrain.step_m), 0.5)
        z_g = self.terrain.corridor_max(xy, th, cfg.b_trk_m, dv)
        if np.isnan(z_g).any():
            fin = z_g[np.isfinite(z_g)]
            z_g = np.where(np.isnan(z_g), fin.max() if fin.size else -np.inf, z_g)
        h_fw = max(float(spec.min_fw_operation_agl_m), self.h_clr_m)
        # D35：末端下降段（自終點以 tan_d 回推之 4° 下降線低於 z_g + h_minfw 處）只含 h_clr
        D_rem = np.concatenate([np.cumsum((ds * tan_d[1:])[::-1])[::-1], [0.0]]) if N > 1 else np.zeros(1)
        z_desc = z_tc + D_rem
        in_desc = z_desc < z_g + h_fw
        z_lo = z_g + np.where(in_desc, self.h_clr_m, h_fw)
        # 起點：作業剖面已保證 z ≥ 局部地形 + h_min（D36），故起點高度取 max(z_op 代表值, z_lo[0])；
        # 爬升螺旋只計後向遞迴（前方稜線）超出此值之需求
        z0 = max(z_start, float(z_lo[0])) if np.isfinite(z_lo[0]) else z_start
        env = envelope(z_lo, np.full(N, np.inf), tan_c, tan_d, ds, pin_start=z0)
        z_star = env.z_star
        if not np.all(np.isfinite(z_star)):
            return math.inf, math.inf, 0, 0
        dh_c, T_c = _lap_gain(spec, auw, R, W, "climb")
        dh_d, T_d = _lap_gain(spec, auw, R, W, "descent")
        n_up = int(math.ceil(max(0.0, z_star[0] - z0) / float(dh_c[0]) - 1e-9))
        n_dn = int(math.ceil(max(0.0, z_star[-1] - z_tc) / float(dh_d[0]) - 1e-9))
        dzs = np.diff(z_star)
        dz_up = float(np.maximum(dzs, 0.0).sum())
        dz_dn = float(np.maximum(-dzs, 0.0).sum())
        pc = model.p_cruise_w()
        gc = spec.gamma_c_auw(auw)
        gd = spec.gamma_d()
        p_up, p_dn = model.p_climb_w(gc), model.p_descent_w(gd)
        e = pc * t_path + (p_up - pc) * dz_up / (va * math.sin(gc)) - (pc - p_dn) * dz_dn / (va * math.sin(gd))
        e += p_up * n_up * float(T_c[0]) + p_dn * n_dn * float(T_d[0])
        e = e * model.k_unc
        e = e if model.base_minute_mode else e / 3600.0
        e += model.e_landing_wh()
        t_tot = t_path + n_up * float(T_c[0]) + n_dn * float(T_d[0])
        return float(t_tot), float(e), n_up, n_dn

    def _compute(self, cls: str, strip_ids: Sequence[str], geometry: bool) -> None:
        """計算（或重算時間層）``cls`` 之指定條帶列；新條帶列以 append 方式擴表。"""
        n_sites = len(self.site_ids)
        n_old = self.wh[cls].shape[0] if cls in self.wh else 0
        n_tot = len(self.strip_ids[cls])
        if cls not in self.wh or self.wh[cls].shape[0] != n_tot:
            wh = np.full((n_tot, 2, n_sites), np.inf)
            t = np.full((n_tot, 2, n_sites), np.inf)
            laps = np.zeros((n_tot, 2, n_sites, 2), dtype=int)
            if cls in self.wh:
                wh[:n_old] = self.wh[cls]
                t[:n_old] = self.t[cls]
                laps[:n_old] = self.laps[cls]
            self.wh[cls], self.t[cls], self.laps[cls] = wh, t, laps
        for sid in strip_ids:
            i = self.idx[cls][sid]
            s = self.strips[cls][sid]
            for idx, d in enumerate((-1, 1)):
                start = s.end_state(d)
                for j, site in enumerate(self.site_ids):
                    s_app, z_tc = resolve_site_entry(self.sites, site, cls)
                    key = (cls, site, sid, idx)
                    tt, ee, nu, nd = self._route(cls, key, start, float(s.z_op_m), s_app, z_tc, geometry)
                    self.t[cls][i, idx, j] = tt
                    self.wh[cls][i, idx, j] = ee
                    self.laps[cls][i, idx, j] = (nu, nd)

    # -- 增量／更新 -------------------------------------------------------------
    def add_strips(self, cls: str, new_strips: Sequence[Strip]) -> None:
        if cls not in self.strip_ids:
            self.strip_ids[cls] = []
            self.idx[cls] = {}
            self.strips[cls] = {}
        new_ids: list[str] = []
        for s in new_strips:
            if s.strip_id in self.idx[cls]:
                raise ValueError(f"strip {s.strip_id!r} already in budget table for class {cls!r}")
            self.idx[cls][s.strip_id] = len(self.strip_ids[cls])
            self.strip_ids[cls].append(s.strip_id)
            self.strips[cls][s.strip_id] = s
            new_ids.append(s.strip_id)
        self._compute(cls, new_ids, geometry=True)

    def remove_strips(self, cls: str, strip_ids: Sequence[str]) -> None:
        if cls not in self.strip_ids:
            return
        rm = [self.idx[cls][s] for s in strip_ids if s in self.idx[cls]]
        if not rm:
            return
        keep = np.array([i for i in range(len(self.strip_ids[cls])) if i not in set(rm)], dtype=int)
        for s in strip_ids:
            self.strips[cls].pop(s, None)
            for idx in (0, 1):
                for site in self.site_ids:
                    self.paths.pop((cls, site, s, idx), None)
        self.strip_ids[cls] = [self.strip_ids[cls][i] for i in keep]
        self.idx[cls] = {s: i for i, s in enumerate(self.strip_ids[cls])}
        self.wh[cls] = self.wh[cls][keep]
        self.t[cls] = self.t[cls][keep]
        self.laps[cls] = self.laps[cls][keep]

    def refresh(self, wind: WindField, r_eff_m: dict[str, float] | None = None, geometry: bool = False) -> None:
        """風更新：只重算時間／能量層（幾何＝既有 AVOID 路徑）；``geometry=True`` 或 ``r_eff`` 改變之構型重算路徑。"""
        self.wind = wind
        old = dict(self.r_eff_m)
        if r_eff_m:
            self.r_eff_m.update(r_eff_m)
        for c in self.strip_ids:
            geo = geometry or (c in old and c in self.r_eff_m and abs(self.r_eff_m[c] - old[c]) > _ZERO)
            self._compute(c, self.strip_ids[c], geometry=geo)

    # -- 查詢 -------------------------------------------------------------------
    def _site_index(self, site_id: str) -> int:
        try:
            return self.site_ids.index(site_id)
        except ValueError as e:
            raise KeyError(f"unknown site {site_id!r}") from e

    @staticmethod
    def _idx(dir: int) -> int:
        if dir not in (-1, 1):
            raise ValueError("dir must be -1 or +1")
        return (dir + 1) // 2

    def rtb_wh(self, cls: str, strip_id: str, dir: int, site_id: str) -> float:
        return float(self.wh[cls][self.idx[cls][strip_id], self._idx(dir), self._site_index(site_id)])

    def rtb_t(self, cls: str, strip_id: str, dir: int, site_id: str) -> float:
        return float(self.t[cls][self.idx[cls][strip_id], self._idx(dir), self._site_index(site_id)])

    def min_rtb_wh(self, cls: str, strip_id: str, dir: int) -> float:
        """最小備降能量（全部站點取 min）；``dir`` = 該條掃向，端點 = ``strip.end_state(dir)``。"""
        row = self.wh[cls][self.idx[cls][strip_id], self._idx(dir)]
        return float(row.min()) if row.size else math.inf

    def best_site(self, cls: str, strip_id: str, dir: int) -> str:
        row = self.wh[cls][self.idx[cls][strip_id], self._idx(dir)]
        if row.size == 0:
            raise KeyError("budget table has no sites")
        return self.site_ids[int(np.argmin(row))]

    def endpoints(self, cls: str) -> np.ndarray:
        """``(n, 2, 2)``：``[l, idx]`` 之端點 xy（idx 0 → p^R、1 → p^L）。"""
        ids = self.strip_ids[cls]
        out = np.zeros((len(ids), 2, 2))
        for i, sid in enumerate(ids):
            s = self.strips[cls][sid]
            out[i, 0] = s.pR
            out[i, 1] = s.pL
        return out

    def _max_detour_slack_m(self) -> float:
        """全場 keep-out 之最大半周長（未給 xy 時之保守繞行寬放）。"""
        ko = self.keepouts
        if ko is None or len(ko) == 0:
            return 0.0
        try:
            return 0.5 * max((float(g.length) for g in ko.geoms), default=0.0)
        except Exception:                                   # pragma: no cover
            return 0.0

    def _detour_slack_m(self, a: np.ndarray, b: np.ndarray, z_m: float) -> float:
        """直線 a→b（⊕ b_trk）穿越有效 keep-out 時之繞行長度寬放：各阻擋多邊形之半周長（凸形繞行上界）。"""
        ko = self.keepouts
        if ko is None or len(ko) == 0:
            return 0.0
        try:
            seg = shapely.LineString([(float(a[0]), float(a[1])), (float(b[0]), float(b[1]))])
            idx = ko.tree.query(seg, predicate="dwithin", distance=float(self.cfg.b_trk_m))
            tops = np.asarray(ko.tops_z, dtype=float)
            floors = np.asarray(ko.floors_z, dtype=float)
            slack = 0.0
            for i in np.atleast_1d(idx):
                i = int(i)
                if not (floors[i] <= z_m <= tops[i]):
                    continue
                slack += 0.5 * float(ko.geoms[i].length)
            return float(slack)
        except Exception:                                   # pragma: no cover - keep-out 結構不同 → 不寬放
            return 0.0

    def point_wh(self, cls: str, xy: np.ndarray, z_m: float, site_id: str) -> float:
        """任意點之備降能量 **O(1) 粗篩**（規格 §5.5）：最近條帶端點 p* 之表值 + k_unc·P_c·||x-p*||/V_g^worst +
        爬升項 k_unc·(P_up - P_c)·max(0, max(z_ridge(x→p*)+h_clr, z_p*) - z_x)/(V_a sin γ_c κ)。

        **R-06：這不是上界。** 轉場只收直線距離，不含未知航向下的 Dubins 繞行，也不含 keep-out 繞行；實測 hills
        的 981 個航點樣本中最壞低估 17.7 Wh（−13.2%，A1 wp404 → 站點 H），而綁定航點的餘裕只有 13.1 Wh，屬同一
        數量級。低估量本身是**可計算且有界**的（見 :meth:`point_wh_slack_wh`），因此安全檢查必須採兩段式：

        1. 以 ``point_wh`` 粗篩；
        2. 餘裕 ``E_j − r·E_nom − point_wh`` 小於 :meth:`point_wh_slack_wh` 時，改以 :meth:`point_wh_exact`
           （AVOID + ENVELOPE 精算）定案。

        需要 O(1) 而不能精算的呼叫端，請改用 :meth:`point_wh_ub`（= 本值 + slack，才是真正的上界）。
        """
        if cls not in self.strip_ids or not self.strip_ids[cls]:
            return math.inf
        j = self._site_index(site_id)
        x = np.asarray(xy, dtype=float).reshape(2)
        E = self.endpoints(cls)
        d = np.hypot(E[:, :, 0] - x[0], E[:, :, 1] - x[1])
        table = self.wh[cls][:, :, j]
        score = np.where(np.isfinite(table), d, np.inf)
        if not np.isfinite(score).any():
            return math.inf
        l, idx = np.unravel_index(int(np.argmin(score)), score.shape)
        p = E[l, idx]
        base = float(table[l, idx])
        dist = float(d[l, idx])
        model = self.models_rep[cls]
        spec = self.specs[cls]
        va = spec.cruise_airspeed_mps
        z_p = float(self.strips[cls][self.strip_ids[cls][l]].z_op_m)
        vg_w = worst_case_v_g(spec, _wind_max_speed(self.wind), self.cfg.v_g_min_mps)
        pc = model.p_cruise_w()
        gc = spec.gamma_c_auw(model.auw_kg)
        p_up = model.p_climb_w(gc)
        z_ridge = -math.inf
        if dist > _ZERO:
            n = max(2, int(math.ceil(dist / self.d_sample_m)) + 1)
            tau = np.linspace(0.0, 1.0, n)
            line = x[None, :] + tau[:, None] * (p - x)[None, :]
            th = math.atan2(p[0] - x[0], p[1] - x[1])
            zg = self.terrain.corridor_max(line, np.full(n, th), self.cfg.b_trk_m, max(0.5 * self.terrain.step_m, 0.5))
            fin = zg[np.isfinite(zg)]
            z_ridge = float(fin.max()) if fin.size else -math.inf
        z_target = max(z_ridge + self.h_clr_m, z_p)
        # 爬升以「弧段爬升率」κ = cos φ_max 折減（AVOID 路徑之爬升多發生在轉彎弧上，實際爬得比水平慢）
        kappa = math.cos(math.radians(spec.max_bank_angle_deg))
        dz_up = max(0.0, z_target - float(z_m))
        e = pc * dist / vg_w + (p_up - pc) * dz_up / (va * math.sin(gc) * max(kappa, 1e-3))
        e = e * model.k_unc
        e = e if model.base_minute_mode else e / 3600.0
        return base + float(e)

    def point_wh_slack_wh(self, cls: str, xy: np.ndarray | None = None, z_m: float | None = None) -> float:
        """:meth:`point_wh` 之**低估上界**（R-06）：轉場長度誤差 ≤ 2πR_eff（未知航向下的 Dubins 繞行）
        + L_det（直線穿越 keep-out 之繞行寬放 = 各阻擋多邊形半周長；未給 ``xy`` 時取全場最大者）。

        兩段式安全檢查用它決定何時必須改呼叫 :meth:`point_wh_exact`。"""
        if cls not in self.strip_ids or not self.strip_ids[cls]:
            return math.inf
        model = self.models_rep[cls]
        spec = self.specs[cls]
        vg_w = worst_case_v_g(spec, _wind_max_speed(self.wind), self.cfg.v_g_min_mps)
        extra = 2.0 * math.pi * self._r_eff(cls)
        if xy is not None:
            x = np.asarray(xy, dtype=float).reshape(2)
            E = self.endpoints(cls)
            d = np.hypot(E[:, :, 0] - x[0], E[:, :, 1] - x[1])
            l, idx = np.unravel_index(int(np.argmin(d)), d.shape)
            z_ref = float(z_m) if z_m is not None else float(self.strips[cls][self.strip_ids[cls][l]].z_op_m)
            extra += self._detour_slack_m(x, E[l, idx], z_ref)
        else:
            extra += self._max_detour_slack_m()
        e = model.p_cruise_w() * extra / vg_w * model.k_unc
        return float(e if model.base_minute_mode else e / 3600.0)

    def point_wh_ub(self, cls: str, xy: np.ndarray, z_m: float, site_id: str) -> float:
        """真正的 O(1) **上界**（R-06）= :meth:`point_wh` + :meth:`point_wh_slack_wh`。
        不做兩段式精算的呼叫端應改用本式。"""
        v = self.point_wh(cls, xy, z_m, site_id)
        return v if not math.isfinite(v) else v + self.point_wh_slack_wh(cls, xy, z_m)

    def point_wh_exact(self, cls: str, xy: np.ndarray, z_m: float, theta_rad: float, site_id: str) -> float:
        """自 (x, θ) 以 AVOID + ENVELOPE 精算之備降能量（R-06）。

        :meth:`point_wh` 是 O(1) 粗篩且仍非嚴格上界（實測殘差 ≤ 3.4 Wh／5.2%）；餘裕接近門檻時應改用本式。
        """
        try:
            s_app, z_tc = resolve_site_entry(self.sites, site_id, cls)
        except (KeyError, ValueError, TypeError):
            return math.inf
        key = ("__point__", str(site_id), str(cls), 0)
        start = Pose(float(np.asarray(xy).reshape(2)[0]), float(np.asarray(xy).reshape(2)[1]), float(theta_rad))
        try:
            _t, wh, _nu, _nd = self._route(cls, key, start, float(z_m), s_app, float(z_tc), True)
        finally:
            self.paths.pop(key, None)
        return float(wh)

    def extreme_point_wh(self, cls: str, strip_id: str, dir: int, site_id: str) -> float:
        """Ω／U 轉場最外點（規格 §8.3、D38）：端點沿飛行方向外延 :math:`\\sqrt{4R^2-(R+s/2)^2}+R`（s<2R）或 R。"""
        s = self.strips[cls][strip_id]
        R = self._r_eff(cls)
        sp = float(s.spacing_m)
        d_ext = math.sqrt(max(4.0 * R * R - (R + 0.5 * sp) ** 2, 0.0)) + R if sp < 2.0 * R else R
        end = s.end_state(dir)
        pt = np.array([end.x_m, end.y_m]) + d_ext * _t_hat(end.theta_rad)
        return self.point_wh(cls, pt, float(s.z_op_m), site_id)


# ---------------------------------------------------------------------------
# CostCache（規格 §8.9）
# ---------------------------------------------------------------------------
@dataclass(slots=True)
class CostCache:
    """成本快取（契約 §8、規格 §8.9）；每構型獨立索引。契約欄位在前；尾端為額外欄位（幾何層快取、狀態、繞行路徑）。

    * ``op[c]`` (n,2) s、``op_e`` Wh：``[l, idx]`` l 以 idx 方向掃（V_a*、風）。
    * ``tr[c]`` (n,n,4) s：``[l, m, k]``，k = 2·idx_l + idx_m；``tr_e`` Wh、``tr_len`` m、``tr_word`` int8
      （WORDS 索引；6 = AVOID；−1 = 不可行）、``tr_dz`` m（z_op[m] − z_op[l]）、``tr_laps`` 螺旋圈數。
    * ``entry[uav]`` (n,2)：自 S_i 進入後以 idx 掃 l（含暫定走廊 n_lap·T_lap）；``rtb[(c, site)]`` (n,2)：l 以 idx 掃完之
      端點（idx 0 → p^R 朝 θ_LR；1 → p^L 朝 θ_RL）→ S^app（路徑 + 下降螺旋；不含降落固定時間）。
    * ``static_ok[c]`` (n,2)：H4 靜態濾網（規格 §10.3）。``W_R[c]``、``R_eff[c]``：§3.2 政策風速與轉彎半徑。
    """
    strip_ids: dict[str, list[str]]
    idx: dict[str, dict[str, int]]
    op: dict[str, np.ndarray]
    op_e: dict[str, np.ndarray]
    tr: dict[str, np.ndarray]
    tr_e: dict[str, np.ndarray]
    tr_len: dict[str, np.ndarray]
    tr_word: dict[str, np.ndarray]
    tr_dz: dict[str, np.ndarray]
    entry: dict[str, np.ndarray]
    entry_e: dict[str, np.ndarray]
    rtb: dict[tuple[str, str], np.ndarray]
    rtb_e: dict[tuple[str, str], np.ndarray]
    static_ok: dict[str, np.ndarray]
    budget: EnergyBudgetTable
    W_R: dict[str, float]
    geometry_key: int
    wind_key: int
    state_key: int
    # --- 額外（契約未列）---
    R_eff: dict[str, float] = field(default_factory=dict)
    strips: dict[str, dict[str, Strip]] = field(default_factory=dict)
    sites: dict[str, Any] = field(default_factory=dict)
    init_states: dict[str, UAVState] = field(default_factory=dict)
    init_laps: dict[str, tuple[int, float]] = field(default_factory=dict)
    uav_cls: dict[str, str] = field(default_factory=dict)
    wind: Any = None
    tg_cache: dict = field(default_factory=dict)
    strip_sensors: dict[str, Sensor] = field(default_factory=dict)
    tr_laps: dict[str, np.ndarray] = field(default_factory=dict)
    op_va: dict[str, np.ndarray] = field(default_factory=dict)
    op_vg: dict[str, np.ndarray] = field(default_factory=dict)
    entry_len: dict[str, np.ndarray] = field(default_factory=dict)
    entry_word: dict[str, np.ndarray] = field(default_factory=dict)
    entry_laps: dict[str, np.ndarray] = field(default_factory=dict)
    rtb_len: dict[tuple[str, str], np.ndarray] = field(default_factory=dict)
    rtb_word: dict[tuple[str, str], np.ndarray] = field(default_factory=dict)
    rtb_laps: dict[tuple[str, str], np.ndarray] = field(default_factory=dict)
    tr_paths: dict[str, dict[tuple[str, str, int], DubinsPath]] = field(default_factory=dict)
    entry_paths: dict[str, dict[tuple[str, int], DubinsPath]] = field(default_factory=dict)
    rtb_paths: dict[tuple[str, str], dict[tuple[str, int], DubinsPath]] = field(default_factory=dict)

    # ------------------------------------------------------------------ 建構
    @classmethod
    def build(cls, strips: dict[str, dict[str, Strip]], fleet: list[UAVInstance], init_states: dict[str, UAVState],
              init_laps: dict[str, tuple[int, float]], sites: dict[str, Any], ctx: MissionContext, *,
              strip_sensors: dict[str, Sensor] | None = None) -> "CostCache":
        """建表（規格 §8.9）：``strips[cls][strip_id]``；``init_states[uav]`` = 暫定起飛走廊出口（S_acc／S_exit，含 r_eff）；
        ``init_laps[uav] = (n_lap, T_lap)``；``sites[site_id] = (Pose S^app, z_tc)`` 或 ``{cls: (...)}``。
        ``strip_sensors``（strip_id → Sensor，V_g^cap 用）缺省時取該構型所有容許 h_op 之感測器中最小 V_g^cap（保守）。"""
        c = cls.__new__(cls)
        c.strip_ids = {}
        c.idx = {}
        c.op, c.op_e, c.op_va, c.op_vg = {}, {}, {}, {}
        c.tr, c.tr_e, c.tr_len, c.tr_word, c.tr_dz, c.tr_laps = {}, {}, {}, {}, {}, {}
        c.entry, c.entry_e, c.entry_len, c.entry_word, c.entry_laps = {}, {}, {}, {}, {}
        c.rtb, c.rtb_e, c.rtb_len, c.rtb_word, c.rtb_laps = {}, {}, {}, {}, {}
        c.static_ok = {}
        c.W_R, c.R_eff = {}, {}
        c.strips = {k: dict(v) for k, v in strips.items()}
        c.sites = dict(sites)
        c.init_states = dict(init_states)
        c.init_laps = {u: (int(n), float(t)) for u, (n, t) in init_laps.items()}
        c.uav_cls = {u.uav_id: u.cls for u in fleet}
        c.wind = ctx.wind
        c.tg_cache = {}
        c.strip_sensors = dict(strip_sensors or {})
        c.tr_paths, c.entry_paths, c.rtb_paths = {}, {}, {}
        for k, d in strips.items():
            c.strip_ids[k] = list(d.keys())
            c.idx[k] = {s: i for i, s in enumerate(c.strip_ids[k])}
        for k in c.strip_ids:
            c._set_wind_params(k, ctx)
        c.budget = EnergyBudgetTable(c.strips, c.sites, ctx.models_rep, ctx.specs, ctx.wind, ctx.keepouts, ctx.terrain,
                                     ctx.cfg, h_clr_m=ctx.h_clr_m, tg_cache=c.tg_cache, r_eff_m=dict(c.R_eff))
        for k in c.strip_ids:
            c._build_class(k, ctx)
        c.geometry_key = c._geometry_key()
        c.wind_key = int(getattr(ctx.wind, "version", 0))
        c.state_key = c._state_key()
        return c

    # ------------------------------------------------------------------ 內部：參數
    def _class_strips(self, cls: str) -> list[Strip]:
        return [self.strips[cls][s] for s in self.strip_ids[cls]]

    def _set_wind_params(self, cls: str, ctx: MissionContext) -> None:
        """W_R,c（規格 §3.2 政策：[z_site + H_tc,lo, max z_op] 之最大風速）與 R_eff,c。"""
        spec = ctx.specs[cls]
        z_site = float(ctx.z_ground_home_m)
        zs = [float(s.z_op_m) for s in self._class_strips(cls)]
        z_top = max(zs) if zs else z_site + spec.transition_cmd().h_tc_cmd_m
        z_top = max(z_top, z_site + spec.min_fw_operation_agl_m)
        self.W_R[cls] = float(r_eff_wind_speed(self.wind, spec, z_site, z_top, ctx.cfg.r_eff_wind_policy))
        self.R_eff[cls] = float(spec.r_eff(self.W_R[cls]))

    def _v_cap(self, cls: str, strip: Strip, ctx: MissionContext) -> float:
        sen = self.strip_sensors.get(strip.strip_id)
        if sen is not None:
            return float(sen.max_ground_speed_mps(strip.h_min_m))
        cands = [s for (c, _r), s in ctx.sensors.items() if c == cls]
        if not cands:
            return math.inf
        ok = [s for s in cands if float(s.agl_window()[1]) >= strip.h_op_m - 1e-6]
        use = ok if ok else cands
        return float(min(s.max_ground_speed_mps(strip.h_min_m) for s in use))

    # ------------------------------------------------------------------ 內部：各層
    def _compute_op(self, cls: str, ctx: MissionContext) -> None:
        """op[c] (n,2)：V_a* 選擇（規格 §4.4）、t = L/V_g；向量化。"""
        S = self._class_strips(cls)
        n = len(S)
        spec = ctx.specs[cls]
        model = ctx.models_rep[cls]
        t = np.full((n, 2), np.inf)
        e = np.full((n, 2), np.inf)
        va_s = np.full((n, 2), np.nan)
        vg_s = np.full((n, 2), np.nan)
        if n:
            z = np.array([s.z_op_m for s in S])
            wxy = _wind_vec_arr(self.wind, z)
            L = np.array([s.length_m for s in S])
            cap = np.array([self._v_cap(cls, s, ctx) for s in S])
            v_min = _v_min_of(spec, model.auw_kg)
            v_max = _v_max_of(spec)
            for idx, attr in enumerate(("theta_LR_rad", "theta_RL_rad")):
                th = np.array([getattr(s, attr) for s in S])
                va, vg = v_air_star(th, v_min, v_max, cap, wxy)
                ok = np.isfinite(vg) & (vg >= ctx.cfg.v_g_min_mps)
                t[:, idx] = np.where(ok, L / np.where(ok, vg, 1.0), np.inf)
                va_s[:, idx], vg_s[:, idx] = va, vg
            pc = model.p_cruise_w() * model.k_unc
            e = pc * t if model.base_minute_mode else pc * t / 3600.0
        self.op[cls], self.op_e[cls], self.op_va[cls], self.op_vg[cls] = t, e, va_s, vg_s

    def _tr_block(self, cls: str, rows: np.ndarray, cols: np.ndarray, ctx: MissionContext, geometry: bool
                  ) -> tuple[np.ndarray, ...]:
        """轉移區塊 (r, c, 4)：(t, e, len, word, dz, laps)；規格 §8.2 四型 + §8.4 延長。"""
        S = self._class_strips(cls)
        ids = self.strip_ids[cls]
        spec = ctx.specs[cls]
        model = ctx.models_rep[cls]
        R = self.R_eff[cls]
        P_end, P_start = _end_start_poses(S)
        z = np.array([s.z_op_m for s in S])
        r, c = rows.size, cols.size
        starts = np.empty((4, r, c, 3))
        ends = np.empty((4, r, c, 3))
        for k in range(4):
            starts[k] = P_end[rows, k // 2][:, None, :]
            ends[k] = P_start[cols, k % 2][None, :, :]
        z_s = np.broadcast_to(z[rows][None, :, None], (4, r, c)).ravel()
        z_e = np.broadcast_to(z[cols][None, None, :], (4, r, c)).ravel()
        keys = [(ids[l], ids[m], k) for k in range(4) for l in rows for m in cols]
        paths = self.tr_paths.setdefault(cls, {})
        t, L, word, W = _pair_costs(starts.reshape(-1, 3), ends.reshape(-1, 3), R, z_s, z_e, spec.cruise_airspeed_mps,
                                    self.wind, ctx.keepouts, ctx.cfg, paths, keys, geometry, self.tg_cache)
        t = t.reshape(4, r, c)
        L = L.reshape(4, r, c)
        word = word.reshape(4, r, c)
        W = W.reshape(4, r, c)
        same = rows[:, None] == cols[None, :]
        t[:, same] = np.inf
        L[:, same] = np.inf
        word[:, same] = WORD_NONE_IDX
        if "same" not in ctx.cfg.transfer_types:
            t[[0, 3]] = np.inf
            word[[0, 3]] = WORD_NONE_IDX
        dz = np.broadcast_to((z[cols][None, :] - z[rows][:, None])[None, :, :], (4, r, c))
        t_ext, n_lap, T_lap = _dz_extension(spec, model.auw_kg, R, W, t, L, dz)
        t_tot = np.where(np.isfinite(t_ext), t_ext + n_lap * T_lap, np.inf)
        e = np.where(np.isfinite(t_ext), _transfer_wh_vec(model, np.where(np.isfinite(t_ext), t_ext, 0.0), dz, n_lap, T_lap), np.inf)
        mv = lambda a: np.moveaxis(a, 0, -1)   # (4,r,c) → (r,c,4)
        return mv(t_tot), mv(e), mv(L), mv(word).astype(np.int8), mv(np.array(dz)), mv(n_lap)

    def _compute_tr(self, cls: str, ctx: MissionContext, geometry: bool) -> None:
        n = len(self.strip_ids[cls])
        ar = np.arange(n)
        if n == 0:
            for d in (self.tr, self.tr_e, self.tr_len, self.tr_dz):
                d[cls] = np.zeros((0, 0, 4))
            self.tr_word[cls] = np.zeros((0, 0, 4), dtype=np.int8)
            self.tr_laps[cls] = np.zeros((0, 0, 4), dtype=int)
            return
        t, e, L, w, dz, laps = self._tr_block(cls, ar, ar, ctx, geometry)
        self.tr[cls], self.tr_e[cls], self.tr_len[cls], self.tr_word[cls], self.tr_dz[cls], self.tr_laps[cls] = t, e, L, w, dz, laps

    def _entry_rows(self, cls: str, uav_id: str, rows: np.ndarray, ctx: MissionContext, geometry: bool
                    ) -> tuple[np.ndarray, ...]:
        """entry[uav] 之列 (r,2)：(t, e, len, word, laps)。"""
        S = self._class_strips(cls)
        ids = self.strip_ids[cls]
        spec = ctx.specs[cls]
        model = ctx.models.get(uav_id) or ctx.models_rep[cls]
        st = self.init_states[uav_id]
        R = float(st.r_eff_m) if st.r_eff_m > 0 else self.R_eff[cls]
        _, P_start = _end_start_poses(S)
        z = np.array([s.z_op_m for s in S])
        r = rows.size
        ends = np.transpose(P_start[rows], (1, 0, 2)).reshape(-1, 3)          # (2·r, 3)：idx 主序
        starts = np.tile(st.as_pose_array(), (2 * r, 1))
        z_e = np.tile(z[rows], 2)
        z_s = np.full(2 * r, float(st.z_m))
        keys = [(ids[l], idx) for idx in (0, 1) for l in rows]
        paths = self.entry_paths.setdefault(uav_id, {})
        t, L, word, W = _pair_costs(starts, ends, R, z_s, z_e, spec.cruise_airspeed_mps, self.wind, ctx.keepouts,
                                    ctx.cfg, paths, keys, geometry, self.tg_cache)
        dz = z_e - z_s
        t_ext, n_lap, T_lap = _dz_extension(spec, model.auw_kg, R, W, t, L, dz)
        n_given, t_given = self.init_laps.get(uav_id, (0, 0.0))
        if n_given > 0:
            n_lap = np.full_like(n_lap, int(n_given))
            T_lap = np.full_like(T_lap, float(t_given))
        fin = np.isfinite(t_ext)
        t_tot = np.where(fin, t_ext + n_lap * T_lap, np.inf)
        e = np.where(fin, _transfer_wh_vec(model, np.where(fin, t_ext, 0.0), dz, n_lap, T_lap), np.inf)
        shp = lambda a: a.reshape(2, r).T
        return shp(t_tot), shp(e), shp(L), shp(word).astype(np.int8), shp(n_lap)

    def _compute_entry(self, cls: str, uav_id: str, ctx: MissionContext, geometry: bool) -> None:
        n = len(self.strip_ids[cls])
        if n == 0 or uav_id not in self.init_states:
            for d in (self.entry, self.entry_e, self.entry_len):
                d[uav_id] = np.zeros((n, 2)) + (np.inf if uav_id not in self.init_states else 0.0)
            self.entry_word[uav_id] = np.full((n, 2), WORD_NONE_IDX, dtype=np.int8)
            self.entry_laps[uav_id] = np.zeros((n, 2), dtype=int)
            return
        t, e, L, w, laps = self._entry_rows(cls, uav_id, np.arange(n), ctx, geometry)
        self.entry[uav_id], self.entry_e[uav_id], self.entry_len[uav_id], self.entry_word[uav_id], self.entry_laps[uav_id] = t, e, L, w, laps

    def _rtb_rows(self, cls: str, site_id: str, rows: np.ndarray, ctx: MissionContext, geometry: bool
                  ) -> tuple[np.ndarray, ...]:
        """rtb[(c, site)] 之列 (r,2)：(t, e, len, word, laps)；起點 = end_state(idx)，終點 = S^app。"""
        S = self._class_strips(cls)
        ids = self.strip_ids[cls]
        spec = ctx.specs[cls]
        model = ctx.models_rep[cls]
        R = self.R_eff[cls]
        s_app, z_tc = resolve_site_entry(self.sites, site_id, cls)
        P_end, _ = _end_start_poses(S)
        z = np.array([s.z_op_m for s in S])
        r = rows.size
        starts = np.transpose(P_end[rows], (1, 0, 2)).reshape(-1, 3)
        ends = np.tile(s_app.as_array(), (2 * r, 1))
        z_s = np.tile(z[rows], 2)
        z_e = np.full(2 * r, z_tc)
        keys = [(ids[l], idx) for idx in (0, 1) for l in rows]
        paths = self.rtb_paths.setdefault((cls, site_id), {})
        t, L, word, W = _pair_costs(starts, ends, R, z_s, z_e, spec.cruise_airspeed_mps, self.wind, ctx.keepouts,
                                    ctx.cfg, paths, keys, geometry, self.tg_cache)
        dz = z_e - z_s
        t_ext, n_lap, T_lap = _dz_extension(spec, model.auw_kg, R, W, t, L, dz)
        fin = np.isfinite(t_ext)
        t_tot = np.where(fin, t_ext + n_lap * T_lap, np.inf)
        e = np.where(fin, _transfer_wh_vec(model, np.where(fin, t_ext, 0.0), dz, n_lap, T_lap), np.inf)
        shp = lambda a: a.reshape(2, r).T
        return shp(t_tot), shp(e), shp(L), shp(word).astype(np.int8), shp(n_lap)

    def _compute_rtb(self, cls: str, site_id: str, ctx: MissionContext, geometry: bool) -> None:
        n = len(self.strip_ids[cls])
        key = (cls, site_id)
        if n == 0:
            self.rtb[key] = np.zeros((0, 2))
            self.rtb_e[key] = np.zeros((0, 2))
            self.rtb_len[key] = np.zeros((0, 2))
            self.rtb_word[key] = np.zeros((0, 2), dtype=np.int8)
            self.rtb_laps[key] = np.zeros((0, 2), dtype=int)
            return
        t, e, L, w, laps = self._rtb_rows(cls, site_id, np.arange(n), ctx, geometry)
        self.rtb[key], self.rtb_e[key], self.rtb_len[key], self.rtb_word[key], self.rtb_laps[key] = t, e, L, w, laps

    def _compute_static_ok(self, cls: str, ctx: MissionContext) -> None:
        """H4 靜態濾網（規格 §8.9／§10.3）：r·E_nom + e_alt(端點) + e(H→端點 直線最壞) ≤ η_T·E_avail,min − e_TO。"""
        n = len(self.strip_ids[cls])
        if not ctx.cfg.enforce_energy or n == 0:
            self.static_ok[cls] = np.ones((n, 2), dtype=bool)
            return
        spec = ctx.specs[cls]
        model = ctx.models_rep[cls]
        E = self.budget.endpoints(cls) if self.budget.strip_ids.get(cls) == self.strip_ids[cls] else None
        if E is None:
            S = self._class_strips(cls)
            E = np.zeros((n, 2, 2))
            for i, s in enumerate(S):
                E[i, 0], E[i, 1] = s.pR, s.pL
        z = np.array([s.z_op_m for s in self._class_strips(cls)])
        e_alt = self.budget.wh[cls].min(axis=2) if cls in self.budget.wh and self.budget.wh[cls].shape[2] else np.zeros((n, 2))
        home = np.asarray(ctx.home_xy, dtype=float).reshape(2)
        z_home_tc = float(ctx.z_ground_home_m) + spec.transition_cmd().h_tc_cmd_m
        d = np.hypot(E[:, :, 0] - home[0], E[:, :, 1] - home[1])
        vg_w = worst_case_v_g(spec, _wind_max_speed(self.wind), ctx.cfg.v_g_min_mps)
        va = spec.cruise_airspeed_mps
        pc = model.p_cruise_w()
        gc = spec.gamma_c_auw(model.auw_kg)
        p_up = model.p_climb_w(gc)
        e_go = pc * d / vg_w + (p_up - pc) * np.maximum(0.0, z[:, None] - z_home_tc) / (va * math.sin(gc))
        e_go = e_go * model.k_unc
        e_go = e_go if model.base_minute_mode else e_go / 3600.0
        budget_avail = model.eta_T * float(model.available_wh) - model.e_takeoff_wh()
        self.static_ok[cls] = (model.reserve_wh() + e_alt + e_go) <= budget_avail + _ZERO

    def _build_class(self, cls: str, ctx: MissionContext, geometry: bool = True) -> None:
        self._compute_op(cls, ctx)
        self._compute_tr(cls, ctx, geometry)
        for uav_id, c in self.uav_cls.items():
            if c == cls and uav_id in self.init_states:
                self._compute_entry(cls, uav_id, ctx, geometry)
        for site_id in self.sites:
            self._compute_rtb(cls, site_id, ctx, geometry)
        self._compute_static_ok(cls, ctx)

    def _geometry_key(self) -> int:
        items: list[Any] = []
        for c in sorted(self.strip_ids):
            for sid in self.strip_ids[c]:
                s = self.strips[c][sid]
                items.append((c, sid, round(float(s.pL[0]), 3), round(float(s.pL[1]), 3), round(float(s.pR[0]), 3),
                              round(float(s.pR[1]), 3), round(float(s.z_op_m), 3), round(self.R_eff.get(c, 0.0), 3)))
        return hash(tuple(items)) & 0x7FFFFFFF

    def _state_key(self) -> int:
        items = tuple((u, round(s.x_m, 3), round(s.y_m, 3), round(s.theta_rad, 6), round(s.z_m, 3),
                       self.init_laps.get(u, (0, 0.0))) for u, s in sorted(self.init_states.items()))
        return hash(items) & 0x7FFFFFFF

    # ------------------------------------------------------------------ 更新
    def refresh_wind(self, wind: WindField, ctx: MissionContext) -> None:
        """風更新（規格 §8.9、§12.6）：只重算時間／能量層（幾何：Dubins 以同 R 重解 ⇒ 長度／word 逐位元相同；AVOID 路徑
        沿用）；W_R 使 R_eff 變化 > 5% 之構型才重算幾何層（含 AVOID 與預算表路徑）。"""
        self.wind = wind
        changed: dict[str, float] = {}
        for c in self.strip_ids:
            old = self.R_eff.get(c, 0.0)
            self._set_wind_params(c, ctx)
            if old <= 0 or abs(self.R_eff[c] - old) > R_EFF_REFRESH_TOL * old:
                changed[c] = self.R_eff[c]
            else:
                self.R_eff[c] = old
        if changed:
            self.tg_cache.clear()
        self.budget.refresh(wind, r_eff_m=changed if changed else None)
        for c in self.strip_ids:
            self._build_class(c, ctx, geometry=(c in changed))
        self.wind_key = int(getattr(wind, "version", self.wind_key + 1))
        if changed:
            self.geometry_key = self._geometry_key()

    def refresh_entry(self, uav_id: str, state: UAVState, n_laps: int, t_lap_s: float, ctx: MissionContext) -> None:
        """重算單機之 entry 列（IDP 後首條帶改變、盤旋圈數精算；規格 §11.6）。"""
        cls = self.uav_cls.get(uav_id) or ctx.uav(uav_id).cls
        self.uav_cls[uav_id] = cls
        self.init_states[uav_id] = state
        self.init_laps[uav_id] = (int(n_laps), float(t_lap_s))
        self.entry_paths.pop(uav_id, None)
        if cls in self.strip_ids:
            self._compute_entry(cls, uav_id, ctx, geometry=True)
        self.state_key = self._state_key()

    def add_strips(self, cls: str, new_strips: list[Strip], fleet: list[UAVInstance], init_states: dict[str, UAVState],
                   sites: dict[str, Any], ctx: MissionContext) -> None:
        """增量：只補算受影響列／行（新條帶 × 全部、全部 × 新條帶）；既有條帶對之值不變。"""
        if not new_strips:
            return
        for u in fleet:
            self.uav_cls[u.uav_id] = u.cls
        for u, s in init_states.items():
            self.init_states.setdefault(u, s)
        for k, v in sites.items():
            self.sites.setdefault(k, v)
        new_cls = cls not in self.strip_ids
        if new_cls:
            self.strip_ids[cls], self.idx[cls], self.strips[cls] = [], {}, {}
            self._set_wind_params(cls, ctx)
        n_old = len(self.strip_ids[cls])
        for s in new_strips:
            if s.strip_id in self.idx[cls]:
                raise ValueError(f"strip {s.strip_id!r} already in cache for class {cls!r}")
            self.idx[cls][s.strip_id] = len(self.strip_ids[cls])
            self.strip_ids[cls].append(s.strip_id)
            self.strips[cls][s.strip_id] = s
        n = len(self.strip_ids[cls])
        if new_cls:
            self._set_wind_params(cls, ctx)
        self.budget.r_eff_m[cls] = self.R_eff[cls]
        self.budget.add_strips(cls, new_strips)
        if new_cls or n_old == 0:
            self._build_class(cls, ctx)
            self.geometry_key = self._geometry_key()
            return
        old_i = np.arange(n_old)
        new_i = np.arange(n_old, n)
        # op（純向量化，重算整列）
        self._compute_op(cls, ctx)
        # tr：擴表並補算新列／新行
        def grow3(a: np.ndarray, fill: Any, dtype: Any = float) -> np.ndarray:
            out = np.full((n, n, 4), fill, dtype=dtype)
            out[:n_old, :n_old] = a
            return out
        tr = grow3(self.tr[cls], np.inf)
        tr_e = grow3(self.tr_e[cls], np.inf)
        tr_len = grow3(self.tr_len[cls], np.inf)
        tr_word = grow3(self.tr_word[cls], WORD_NONE_IDX, np.int8)
        tr_dz = grow3(self.tr_dz[cls], 0.0)
        tr_laps = grow3(self.tr_laps[cls], 0, int)
        for rows, cols in ((new_i, np.arange(n)), (old_i, new_i)):
            t, e, L, w, dz, laps = self._tr_block(cls, rows, cols, ctx, geometry=True)
            tr[np.ix_(rows, cols)] = t
            tr_e[np.ix_(rows, cols)] = e
            tr_len[np.ix_(rows, cols)] = L
            tr_word[np.ix_(rows, cols)] = w
            tr_dz[np.ix_(rows, cols)] = dz
            tr_laps[np.ix_(rows, cols)] = laps
        self.tr[cls], self.tr_e[cls], self.tr_len[cls], self.tr_word[cls], self.tr_dz[cls], self.tr_laps[cls] = \
            tr, tr_e, tr_len, tr_word, tr_dz, tr_laps
        # entry／rtb：補算新列
        for uav_id, c in self.uav_cls.items():
            if c != cls or uav_id not in self.init_states:
                continue
            if uav_id not in self.entry or self.entry[uav_id].shape[0] != n_old:
                self._compute_entry(cls, uav_id, ctx, geometry=True)
                continue
            t, e, L, w, laps = self._entry_rows(cls, uav_id, new_i, ctx, geometry=True)
            self.entry[uav_id] = np.vstack([self.entry[uav_id], t])
            self.entry_e[uav_id] = np.vstack([self.entry_e[uav_id], e])
            self.entry_len[uav_id] = np.vstack([self.entry_len[uav_id], L])
            self.entry_word[uav_id] = np.vstack([self.entry_word[uav_id], w])
            self.entry_laps[uav_id] = np.vstack([self.entry_laps[uav_id], laps])
        for site_id in self.sites:
            key = (cls, site_id)
            if key not in self.rtb or self.rtb[key].shape[0] != n_old:
                self._compute_rtb(cls, site_id, ctx, geometry=True)
                continue
            t, e, L, w, laps = self._rtb_rows(cls, site_id, new_i, ctx, geometry=True)
            self.rtb[key] = np.vstack([self.rtb[key], t])
            self.rtb_e[key] = np.vstack([self.rtb_e[key], e])
            self.rtb_len[key] = np.vstack([self.rtb_len[key], L])
            self.rtb_word[key] = np.vstack([self.rtb_word[key], w])
            self.rtb_laps[key] = np.vstack([self.rtb_laps[key], laps])
        self._compute_static_ok(cls, ctx)
        self.geometry_key = self._geometry_key()

    def remove_strips(self, cls: str, strip_ids: list[str]) -> None:
        """移除條帶：刪除對應列／行、更新索引；其餘值不變。"""
        if cls not in self.strip_ids:
            return
        rm = {self.idx[cls][s] for s in strip_ids if s in self.idx[cls]}
        if not rm:
            return
        keep = np.array([i for i in range(len(self.strip_ids[cls])) if i not in rm], dtype=int)
        removed = set(s for s in strip_ids if s in self.idx[cls])
        for s in removed:
            self.strips[cls].pop(s, None)
        self.strip_ids[cls] = [self.strip_ids[cls][i] for i in keep]
        self.idx[cls] = {s: i for i, s in enumerate(self.strip_ids[cls])}
        for d in (self.op, self.op_e, self.op_va, self.op_vg):
            d[cls] = d[cls][keep]
        for d in (self.tr, self.tr_e, self.tr_len, self.tr_word, self.tr_dz, self.tr_laps):
            d[cls] = d[cls][np.ix_(keep, keep)]
        if cls in self.tr_paths:
            self.tr_paths[cls] = {k: p for k, p in self.tr_paths[cls].items() if k[0] not in removed and k[1] not in removed}
        for uav_id, c in self.uav_cls.items():
            if c != cls or uav_id not in self.entry:
                continue
            for d in (self.entry, self.entry_e, self.entry_len, self.entry_word, self.entry_laps):
                d[uav_id] = d[uav_id][keep]
            if uav_id in self.entry_paths:
                self.entry_paths[uav_id] = {k: p for k, p in self.entry_paths[uav_id].items() if k[0] not in removed}
        for key in list(self.rtb):
            if key[0] != cls:
                continue
            for d in (self.rtb, self.rtb_e, self.rtb_len, self.rtb_word, self.rtb_laps):
                d[key] = d[key][keep]
            if key in self.rtb_paths:
                self.rtb_paths[key] = {k: p for k, p in self.rtb_paths[key].items() if k[0] not in removed}
        self.static_ok[cls] = self.static_ok[cls][keep]
        self.budget.remove_strips(cls, list(removed))
        self.geometry_key = self._geometry_key()

    # ------------------------------------------------------------------ 組合
    def T_row0(self, uav_id: str) -> np.ndarray:
        """論文 Eq.13 第 0 列 (n,2)：entry[uav][l, idx] + op[cls][l, idx]。"""
        cls = self.uav_cls[uav_id]
        return self.entry[uav_id] + self.op[cls]

    def T_transfer(self, cls: str) -> np.ndarray:
        """(n,n,4)：tr[l, m, k] + op[m, k % 2]（idx_m = k mod 2）。"""
        op = self.op[cls]
        return self.tr[cls] + op[:, [0, 1, 0, 1]][None, :, :]

    def strip(self, cls: str, l: int) -> Strip:
        return self.strips[cls][self.strip_ids[cls][l]]

    def n_strips(self, cls: str) -> int:
        return len(self.strip_ids.get(cls, []))
