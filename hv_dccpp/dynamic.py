"""hv_dccpp/dynamic.py — 事件與增量重規劃（契約 §12；規格 §12.1–12.9、D23、D26）。

* 最上層（第 4 層之上）：依賴 planner 及其下所有模組；不得被任何模組 import。
* 通用框架（規格 §12.1）：快照（遙測 / 計畫內插）→ L_uncov（未開始條帶 ∪ 中斷條帶裁短殘段，保留 e_ext）→
  受影響集合 A（感測相容 ∧ 能量餘裕 ≥ margin；不限退出機團隊）→ 快取增量（`cache.refresh_entry` / `add_strips` /
  `refresh_wind`）→ 受影響機序列尾端追加（`planner.beam_idp` 以錨點狀態為初始、`append_rtb`）→ 尾段 `altitude.build_3d`
  → 只對變更機去衝突（延遲盤旋）→ diff。
* 不變量：未受影響機之 `UAVPlan` 為**同一物件**（位元相同）；受影響機之既有航點（至現行／末條帶端點）為同一 Waypoint
  物件，之後只追加新尾段；`diff[u]` = 該機自錨點之後之航點。
* 錨點（anchor）：接手機 = 其序列末條帶端點（`end_state`，時間／能量取自該航點）；正在返航者 = 現時內插狀態；
  切除（cut）= 被保留之最後條帶端點；立即中止／DIVERT = 現時狀態。
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field, replace
from typing import Any, Iterable, Literal, Mapping, Sequence

import numpy as np
import shapely
from shapely.geometry import Point, Polygon

from . import decompose as _dec
from .allocate import Team
from .altitude import ORBIT_WORD, CorridorResult, PathSegment2D, build_3d, landing_corridor
from .context import MissionContext
from .costs import CostCache, PathCost, resolve_site_entry
from .deconflict import (Conflict, CorridorSchedule, check_separation, insert_delay_orbit, orbit_insert_index,
                         orbit_lap_time_s)
from .dubins import DubinsPath, DubinsSegment, Pose, dubins_path, path_time
from .geometry import Cell, Strip, strips_union_footprint
from .mission import (AreaSpec, MissionPlan, Mode, PayloadRequirement, Phase, PlannerConfig, UAVInstance, UAVPlan,
                      UAVState, Waypoint, WeatherObs, WindLayer)
from .planner import EnergyInfeasible, SeqItem, UAVSequence, append_rtb, beam_idp
from .platform import EnergyModel, PlatformSpec, sensor_model
from .routing import RoutingError, avoid
from .wind import WindField

__all__ = [
    "Telemetry", "UAVExit", "NewArea", "EnergyUpdate", "LinkLoss", "LinkRestored", "WindUpdate", "TargetCue",
    "PriorityChange", "WeatherAbort", "Event", "GhostTrack", "ReplanResult",
    "snapshot", "placeholder_plan", "ghost_track", "affected_set", "replan",
    "handle_uav_exit", "handle_new_area", "handle_energy_update", "handle_link_loss", "handle_link_restored",
    "handle_wind_update", "handle_cue", "handle_priority", "handle_weather_abort",
    "cache_from_plan", "REPLAN_BUDGET_MS",
]

_INF = math.inf
_EPS = 1e-9
_TWO_PI = 2.0 * math.pi
REPLAN_BUDGET_MS = 2000.0          # 規格 §12.1「時限 ≤ 2 s」：超時 → placeholder_used
_FINISH_STRIP_S = 60.0             # 規格 §12.9：剩餘現行條帶 < 60 s 且能量允許則完成
_CUE_N_LAPS = 2                    # 規格 §12.7 orbit 確認圈數
_GHOST_DT_S = 2.0
_CORRIDOR_TURN_DEG = 45.0          # 規格 §12.6：起降風向變化 > 45° → 重算走廊
_ORBIT_ROUNDS = 3


# ---------------------------------------------------------------------------
# 契約資料型別
# ---------------------------------------------------------------------------
@dataclass(frozen=True, slots=True)
class Telemetry:
    """遙測（ENU；`z_m` 為 ENU z；`strip_progress` ∈ [0,1] 為現行條帶已掃比例；`e_tel_wh` None → 以計畫能量推估）。"""
    uav_id: str
    t_s: float
    x_m: float
    y_m: float
    z_m: float
    theta_rad: float
    v_air_mps: float
    e_tel_wh: float | None
    current_strip_id: str | None
    strip_progress: float
    link_ok: bool


@dataclass(frozen=True, slots=True)
class UAVExit:
    uav_id: str
    t_s: float
    reason: str


@dataclass(frozen=True, slots=True)
class NewArea:
    area: AreaSpec
    t_s: float


@dataclass(frozen=True, slots=True)
class EnergyUpdate:
    uav_id: str
    t_s: float
    e_tel_wh: float


@dataclass(frozen=True, slots=True)
class LinkLoss:
    uav_id: str
    t_s: float


@dataclass(frozen=True, slots=True)
class LinkRestored:
    uav_id: str
    t_s: float
    telemetry: Telemetry


@dataclass(frozen=True, slots=True)
class WindUpdate:
    layers: tuple[WindLayer, ...]
    t_s: float
    obs: WeatherObs | None


@dataclass(frozen=True, slots=True)
class TargetCue:
    latlon_wgs84: tuple[float, float]
    src_uav_id: str
    confidence: float
    t_s: float


@dataclass(frozen=True, slots=True)
class PriorityChange:
    area_id: str
    priority: float
    t_s: float


@dataclass(frozen=True, slots=True)
class WeatherAbort:
    scope: Literal["ALL", "A", "B", "region"]
    region_wgs84: tuple[tuple[float, float], ...] | None
    reason: str
    t_s: float


Event = UAVExit | NewArea | EnergyUpdate | LinkLoss | LinkRestored | WindUpdate | TargetCue | PriorityChange | WeatherAbort


@dataclass(slots=True)
class GhostTrack:
    """失鏈機推算 4D 航跡（規格 §12.5、D26）：`samples` (M,4) [x,y,z,t]、`sigma_t` = k_t·(ETA − t_loss)（常數上界）、
    `landing_site`、`eta_s` = 抵達 P_tc 時刻（走廊預約 [ETA−60, ETA+180]）。尾端額外欄位：`policy`、`t_loss_s`、`t_rtb_s`。"""
    uav_id: str
    samples: np.ndarray
    sigma_t: float
    landing_site: str
    eta_s: float
    policy: str = "RTB"
    t_loss_s: float = 0.0
    t_rtb_s: float = 0.0


@dataclass(slots=True)
class ReplanResult:
    """重規劃結果：`plan` 新 MissionPlan（未變更機為同一 UAVPlan 物件）、`changed_uavs`、`diff[u]` = 自錨點之後之航點、
    `placeholder_used`（總耗時 > 2 s）、`timing_ms`、`notes`。尾端額外欄位：`ghosts`（LINK_LOSS 登記之幽靈航跡）、
    `placeholders`（超時時之佔位航點）。"""
    plan: MissionPlan
    changed_uavs: list[str]
    diff: dict[str, list[Waypoint]]
    placeholder_used: bool
    timing_ms: dict[str, float]
    notes: list[str]
    ghosts: list[GhostTrack] = field(default_factory=list)
    placeholders: dict[str, list[Waypoint]] = field(default_factory=dict)
    t_sync_s: float = 0.0
    """R-12：本次重規劃所用之遙測同步時刻（= 事件 t_now）。驗證此計畫時必須把它傳給
    ``sim.validate(..., t_sync_s=res.t_sync_s)``，否則 §13.3 的 σ_t = k_t (t − t_sync) 會被灌上 k_t·t_now，
    驗證器會報出重規劃器判定不存在的衝突。"""


# ---------------------------------------------------------------------------
# 計畫解析（航點 → 條帶區段、內插狀態）
# ---------------------------------------------------------------------------
@dataclass(slots=True)
class _Span:
    """一條帶之作業航點區段：`i0` = 首個 OPER 航點、`i_end` = 區段後第一個航點（= 條帶端點 p^R/p^L，錨點）。"""
    strip_id: str
    dir: int
    i0: int
    i_end: int
    t0: float
    t_end: float


def _spans(plan: UAVPlan) -> list[_Span]:
    wps = plan.waypoints
    out: list[_Span] = []
    n = len(wps)
    i = 0
    while i < n:
        w = wps[i]
        if w.phase is Phase.OPERATE and w.strip_id is not None:
            j = i
            while j < n and wps[j].phase is Phase.OPERATE and wps[j].strip_id == w.strip_id:
                j += 1
            i_end = min(j, n - 1)
            out.append(_Span(w.strip_id, int(w.dir if w.dir is not None else -1), i, i_end, float(w.t_s),
                             float(wps[i_end].t_s)))
            i = j
        else:
            i += 1
    return out


def _index_at(plan: UAVPlan, t_s: float) -> int:
    ts = np.array([w.t_s for w in plan.waypoints], dtype=float)
    if ts.size == 0:
        return -1
    return int(np.clip(np.searchsorted(ts, float(t_s), side="right") - 1, 0, ts.size - 1))


_TAKEOFF_PREFIX_PHASES = frozenset({Phase.TAKEOFF, Phase.TRANS_OUT, Phase.ACCEL, Phase.CLIMB})


def _takeoff_prefix_end(plan: UAVPlan) -> int:
    """起飛走廊（TAKEOFF/TRANS_OUT/ACCEL/CLIMB 之開頭連續段）最後一個航點索引；不存在 → −1。"""
    wps = plan.waypoints
    if not wps or wps[0].phase not in _TAKEOFF_PREFIX_PHASES:
        return -1
    k = 0
    while k + 1 < len(wps) and wps[k + 1].phase in _TAKEOFF_PREFIX_PHASES:
        k += 1
    return k


def _state_at_wp(plan: UAVPlan, j: int, spec: PlatformSpec, wind: WindField | None) -> UAVState:
    """第 j 個航點之 :class:`UAVState`（沿用該航點自己的時刻與能量）。"""
    w = plan.waypoints[j]
    ws = wind.speed_at(float(w.z_m)) if wind is not None else 0.0
    va = float(w.airspeed_mps) if w.airspeed_mps > 0 else float(spec.cruise_airspeed_mps)
    return UAVState(float(w.x_m), float(w.y_m), float(math.radians(w.track_deg)) % _TWO_PI, float(w.z_m),
                    float(w.t_s), float(w.energy_remaining_wh), float(spec.r_eff(ws)), va)


def _interp_state(plan: UAVPlan, t_s: float, spec: PlatformSpec, wind: WindField | None) -> UAVState:
    """計畫航點於 t 之線性內插狀態（θ = 該航點外出航段之 track；能量線性內插）。"""
    wps = plan.waypoints
    j = _index_at(plan, t_s)
    a = wps[j]
    if j >= len(wps) - 1 or wps[j + 1].t_s <= a.t_s + _EPS:
        x, y, z, e, t = a.x_m, a.y_m, a.z_m, a.energy_remaining_wh, max(float(t_s), a.t_s) if j < len(wps) - 1 else a.t_s
    else:
        b = wps[j + 1]
        f = (float(t_s) - a.t_s) / (b.t_s - a.t_s)
        f = min(1.0, max(0.0, f))
        x = a.x_m + f * (b.x_m - a.x_m)
        y = a.y_m + f * (b.y_m - a.y_m)
        z = a.z_m + f * (b.z_m - a.z_m)
        e = a.energy_remaining_wh + f * (b.energy_remaining_wh - a.energy_remaining_wh)
        t = float(t_s)
    w = wind.speed_at(float(z)) if wind is not None else 0.0
    va = float(a.airspeed_mps) if a.airspeed_mps > 0 else float(spec.cruise_airspeed_mps)
    return UAVState(float(x), float(y), float(math.radians(a.track_deg)) % _TWO_PI, float(z), float(t), float(e),
                    float(spec.r_eff(w)), va)


def _plan_energy_at(plan: UAVPlan, t_s: float) -> float:
    wps = plan.waypoints
    j = _index_at(plan, t_s)
    if j < 0:
        return 0.0
    a = wps[j]
    if j >= len(wps) - 1 or wps[j + 1].t_s <= a.t_s + _EPS:
        return float(a.energy_remaining_wh)
    b = wps[j + 1]
    f = min(1.0, max(0.0, (float(t_s) - a.t_s) / (b.t_s - a.t_s)))
    return float(a.energy_remaining_wh + f * (b.energy_remaining_wh - a.energy_remaining_wh))


def _plan_end_energy(plan: UAVPlan, model: EnergyModel, spec: PlatformSpec) -> float:
    """落地後剩餘能量（末航點 − 懸停保留）。"""
    if not plan.waypoints:
        return 0.0
    return float(plan.waypoints[-1].energy_remaining_wh) - float(model.segment_wh(Mode.HOVER, spec.t_hover_reserve_s))


@dataclass(slots=True)
class _Status:
    done: list[_Span]
    current: _Span | None
    progress: float
    pending: list[_Span]
    landed: bool


def _status(plan: UAVPlan, t_s: float) -> _Status:
    spans = _spans(plan)
    done: list[_Span] = []
    pending: list[_Span] = []
    cur: _Span | None = None
    prog = 0.0
    for s in spans:
        if s.t_end <= t_s + _EPS:
            done.append(s)
        elif s.t0 <= t_s:
            cur = s
            prog = (t_s - s.t0) / max(s.t_end - s.t0, _EPS)
        else:
            pending.append(s)
    landed = bool(plan.waypoints) and plan.waypoints[-1].t_s <= t_s
    return _Status(done, cur, float(min(1.0, max(0.0, prog))), pending, landed)


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------
def _t_hat(theta: float) -> np.ndarray:
    return np.array([math.sin(theta), math.cos(theta)])


def _n_hat(theta: float) -> np.ndarray:
    return np.array([math.cos(theta), -math.sin(theta)])


def _pose(state: UAVState) -> Pose:
    return Pose(float(state.x_m), float(state.y_m), float(state.theta_rad))


def _landing_fixed_time_s(spec: PlatformSpec) -> float:
    tc = spec.transition_cmd()
    return float(tc.t_acc_s + spec.t_tr_in_s + spec.hover_descent_time_s() + spec.t_hover_reserve_s)


def _cell_maps(plan: MissionPlan) -> tuple[dict[str, Cell], dict[str, Cell]]:
    """(cell_id → Cell, strip_id → Cell)。殘段條帶（`<sid>#r`）以 cell_id 對應。"""
    by_id = {c.cell_id: c for c in plan.cells}
    by_strip: dict[str, Cell] = {}
    for c in plan.cells:
        for lst in c.strips.values():
            for s in lst:
                by_strip.setdefault(s.strip_id, c)
    for sid, s in plan.strips.items():
        if sid not in by_strip and s.cell_id in by_id:
            by_strip[sid] = by_id[s.cell_id]
    return by_id, by_strip


def _cell_of(plan: MissionPlan, strip: Strip, maps: tuple[dict[str, Cell], dict[str, Cell]] | None = None) -> Cell | None:
    by_id, by_strip = maps if maps is not None else _cell_maps(plan)
    c = by_strip.get(strip.strip_id)
    if c is None:
        c = by_id.get(str(strip.cell_id))
    return c


def _z_tc_of(ctx: MissionContext, cls: str, site_id: str) -> float:
    spec = ctx.specs[cls]
    return float(ctx.sites_enu[site_id][1]) + spec.transition_cmd(transition_profile=ctx.cfg.transition_profile).h_tc_cmd_m


def _site_app(ctx: MissionContext, cache: CostCache, uav: UAVInstance, site_id: str, s_last: Pose, z_last: float
              ) -> tuple[Pose, CorridorResult]:
    ld = landing_corridor(site_id, uav, ctx, s_last, float(z_last))
    app = ld.app_state
    if app is None:
        try:
            app, _z = resolve_site_entry(cache.sites, site_id, uav.cls)
        except Exception:
            app = Pose(float(ctx.sites_enu[site_id][0][0]), float(ctx.sites_enu[site_id][0][1]), 0.0)
    return app, ld


def _route(a: Pose, b: Pose, R: float, ctx: MissionContext, cache: CostCache, z_hint: float, va: float) -> DubinsPath:
    try:
        return avoid(a, b, float(R), ctx.keepouts, float(z_hint), ctx.wind, float(va), float(ctx.cfg.b_trk_m),
                     cache.tg_cache)
    except RoutingError:
        return dubins_path(a, b, float(R))


def _orbit_path(entry: Pose, center: np.ndarray, R: float, turn: int, dtheta_abs: float) -> DubinsPath:
    """單弧 ORBIT DubinsPath（與 altitude._orbit_path 同構；於 `entry`（圓上、切線方位）沿 turn 轉 dtheta_abs）。"""
    kind = "R" if turn > 0 else "L"
    dth = turn * dtheta_abs
    th_end = entry.theta_rad + dth
    end_xy = center - turn * R * _n_hat(th_end)
    end = Pose(float(end_xy[0]), float(end_xy[1]), float(th_end))
    seg = DubinsSegment(kind=kind, length_m=R * dtheta_abs, start=entry, end=end,
                        center=(float(center[0]), float(center[1])), dtheta_rad=dth)
    return DubinsPath(word=ORBIT_WORD, R_m=R, segments=(seg,), length_m=R * dtheta_abs)


# ---------------------------------------------------------------------------
# 快照（契約 snapshot）
# ---------------------------------------------------------------------------
def _e_ext_of(plan: MissionPlan, strip: Strip, ctx: MissionContext) -> float:
    """條帶端點外延 e_ext（Strip 未存；以感測器公式重算，§4.5）。"""
    cfg = ctx.cfg
    if cfg.e_ext_override_m is not None:
        return float(cfg.e_ext_override_m)
    cell = _cell_of(plan, strip)
    req = cell.payload_req_id if cell is not None else "default"
    dc = ctx.decompose_ctx()
    found = _dec.resolve_sensor(dc, strip.cls, req)
    if found is None:
        return 0.0
    try:
        return float(_dec.end_extension_for(dc, strip.cls, found[0], float(strip.h_op_m), z_op_m=float(strip.z_op_m)))
    except Exception:
        return 0.0


def _truncate_strip(strip: Strip, dir: int, progress: float, e_ext_m: float) -> Strip | None:
    """自最後有效覆蓋點裁短之殘段（保留 e_ext 引入段）；progress ≥ 1 → None（視為完成）。"""
    p = float(min(1.0, max(0.0, progress)))
    if p >= 1.0 - 1e-6:
        return None
    start = np.asarray(strip.pL if dir == -1 else strip.pR, dtype=float)
    end = np.asarray(strip.pR if dir == -1 else strip.pL, dtype=float)
    L = float(np.hypot(*(end - start)))
    if L <= _EPS:
        return None
    u_cov = p * L
    u_new = max(0.0, u_cov - float(e_ext_m))
    q = start + (end - start) * (u_new / L)
    if dir == -1:
        pL, pR = q, end
    else:
        pL, pR = end, q
    return Strip(strip_id=f"{strip.strip_id}#r", cell_id=strip.cell_id, index=strip.index, cls=strip.cls,
                 pL=np.asarray(pL, dtype=float), pR=np.asarray(pR, dtype=float), theta_LR_rad=strip.theta_LR_rad,
                 sweep_origin=np.asarray(strip.sweep_origin, dtype=float), width_m=strip.width_m,
                 spacing_m=strip.spacing_m, h_min_m=strip.h_min_m, h_op_m=strip.h_op_m, agl_max_m=strip.agl_max_m,
                 v_lo_m=strip.v_lo_m, v_hi_m=strip.v_hi_m, length_m=float(np.hypot(*(pR - pL))), z_op_m=strip.z_op_m,
                 feasible=True, infeasible_reason="")


def _e_ref(plan: UAVPlan | None, t_s: float, e_tel: float | None, model: EnergyModel | None, spec: PlatformSpec) -> float:
    """規格 §5.6／§12.4 之 E_ref：`battery_monitor_validated` → clip(E_tel, 0.9·E_pred, 1.1·E_pred)；未驗證 → E_pred（κ 修正：
    E_start − κ·E_used,plan；遙測只作趨勢）；無遙測 → E_pred(κ)。"""
    if plan is None or not plan.waypoints:
        return float(e_tel) if e_tel is not None else 0.0
    e_start = float(plan.waypoints[0].energy_remaining_wh)
    e_pred = _plan_energy_at(plan, t_s)
    kappa = float(model.kappa) if model is not None else 1.0
    e_pred_k = e_start - kappa * (e_start - e_pred)
    if e_tel is None or not bool(spec.battery_monitor_validated):
        return float(e_pred_k)
    return float(min(1.1 * e_pred, max(0.9 * e_pred, float(e_tel))))


def _state_from_telemetry(tel: Telemetry, plan: UAVPlan | None, spec: PlatformSpec, wind: WindField | None,
                          model: EnergyModel | None = None) -> UAVState:
    e = _e_ref(plan, tel.t_s, tel.e_tel_wh, model, spec)
    w = wind.speed_at(float(tel.z_m)) if wind is not None else 0.0
    va = float(tel.v_air_mps) if tel.v_air_mps > 0 else float(spec.cruise_airspeed_mps)
    return UAVState(float(tel.x_m), float(tel.y_m), float(tel.theta_rad) % _TWO_PI, float(tel.z_m), float(tel.t_s),
                    float(e), float(spec.r_eff(w)), va)


def snapshot(plan: MissionPlan, telemetry: dict[str, Telemetry], t_s: float, ctx: MissionContext
             ) -> tuple[dict[str, UAVState], list[str], dict[str, Strip]]:
    """規格 §12.1 步 1–2：每機 S_i(t)（遙測優先，否則計畫內插）、未開始之條帶 id（全機隊）、
    中斷條帶自最後有效覆蓋點裁短之殘段（`{strip_id: Strip("<sid>#r")}`，保留 e_ext）。

    殘段只對「現行條帶」產生（每機至多一條）；接手／切除時由 handler 決定是否採用（FOV 有效者先完成該條）。"""
    states: dict[str, UAVState] = {}
    uncovered: list[str] = []
    residual: dict[str, Strip] = {}
    for u, p in plan.plans.items():
        spec = ctx.specs[p.cls]
        tel = telemetry.get(u)
        st = _status(p, t_s)
        if tel is not None:
            states[u] = _state_from_telemetry(tel, p, spec, ctx.wind, ctx.models.get(u) or ctx.models_rep.get(p.cls))
        elif p.waypoints:
            states[u] = _interp_state(p, t_s, spec, ctx.wind)
        else:
            continue
        for s in st.pending:
            if s.strip_id not in uncovered:
                uncovered.append(s.strip_id)
        cur = st.current
        prog = st.progress
        if tel is not None and tel.current_strip_id is not None:
            if tel.current_strip_id in plan.strips:
                sp = next((s for s in _spans(p) if s.strip_id == tel.current_strip_id), None)
                if sp is not None:
                    cur = sp
                    prog = float(tel.strip_progress)
        if cur is not None and cur.strip_id in plan.strips:
            strip = plan.strips[cur.strip_id]
            r = _truncate_strip(strip, cur.dir, prog, _e_ext_of(plan, strip, ctx))
            if r is not None:
                residual[cur.strip_id] = r
    return states, uncovered, residual


# ---------------------------------------------------------------------------
# 受影響集合（契約 affected_set）
# ---------------------------------------------------------------------------
def _uav_inst(ctx: MissionContext, uav_id: str) -> UAVInstance | None:
    try:
        return ctx.uav(uav_id)
    except Exception:
        return None


def _predicted_landing_e(plan: UAVPlan, state: UAVState, model: EnergyModel, spec: PlatformSpec) -> float:
    """以快照能量修正之預期落地能量：E_state − (E_plan(t) − E_plan_end)。"""
    e_plan_t = _plan_energy_at(plan, state.t_s)
    return float(state.e_rem_wh) - (e_plan_t - _plan_end_energy(plan, model, spec))


def affected_set(plan: MissionPlan, uncovered: list[str], states: dict[str, UAVState], ctx: MissionContext,
                 cache: CostCache, cfg: PlannerConfig) -> list[str]:
    """規格 §12.1 步 3：A = {i : κ(σ, c(i)) = 1 for some σ ∈ L_uncov 且 E^rem_i − e(剩餘含 RTB) ≥ r·E_nom + margin}
    （margin = cfg.energy_margin_frac·E_nom；不限退出機所在團隊；已落地者除外）。σ 由條帶所屬胞之 sensor_req 決定
    （`<sid>#r` 殘段沿用母條帶之胞）。"""
    by_id, by_strip = _cell_maps(plan)
    reqs: set[tuple[str, str]] = set()          # (sensor_req, cls_of_strip)
    for sid in uncovered:
        base = sid.split("#", 1)[0]
        s = plan.strips.get(sid) or plan.strips.get(base)
        c = by_strip.get(base) if s is None else _cell_of(plan, s, (by_id, by_strip))
        req = c.sensor_req if c is not None else "any"
        cls = s.cls if s is not None else ""
        reqs.add((req, cls))
    out: list[str] = []
    for u, p in plan.plans.items():
        st = states.get(u)
        if st is None or not p.waypoints:
            continue
        if p.waypoints[-1].t_s <= st.t_s:
            continue                                     # 已落地
        inst = _uav_inst(ctx, u)
        if inst is None:
            continue
        kinds = inst.available_kinds
        compat = False
        for req, cls in reqs:
            if req == "both":
                # both 區：A 之 lidar 條帶只能由 A 接、B 之 rgb 條帶只能由 B 接
                need = "lidar" if cls == "A" else "rgb"
                compat |= need in kinds
            else:
                compat |= _dec.class_can_serve(req, inst.cls, kinds)
        if not compat:
            continue
        model = ctx.models.get(u) or ctx.models_rep[p.cls]
        e_land = _predicted_landing_e(p, st, model, ctx.specs[p.cls])
        margin = float(cfg.energy_margin_frac) * float(model.nominal_wh or model.available_wh)
        if not cfg.enforce_energy or e_land >= model.reserve_wh() + margin - _EPS:
            out.append(u)
    return out


# ---------------------------------------------------------------------------
# 尾段組裝
# ---------------------------------------------------------------------------
def _tail_segments(cls: str, anchor: UAVState, items: Sequence[SeqItem], entry_path: DubinsPath | None,
                   rtb_path: DubinsPath | None, landing: CorridorResult, cache: CostCache, ctx: MissionContext,
                   pre: Sequence[PathSegment2D] = (), first_phase: Phase = Phase.TRANSFER) -> list[PathSegment2D]:
    spec = ctx.specs[cls]
    va = float(spec.cruise_airspeed_mps)
    R = float(cache.R_eff.get(cls, anchor.r_eff_m))
    segs: list[PathSegment2D] = list(pre)
    pose = _pose(anchor)
    z_prev = float(anchor.z_m)
    if pre:
        last = pre[-1]
        if isinstance(last.path, DubinsPath):
            pose = last.path.end
        z_prev = float(last.z_hint_end_m)
    prev: Strip | None = None
    prev_dir = 0
    for j, it in enumerate(items):
        st = cache.strips[cls][it.strip_id]
        if j == 0:
            p = entry_path
            if p is None or math.hypot(p.start.x_m - pose.x_m, p.start.y_m - pose.y_m) > 1e-3:
                p = _route(pose, st.start_state(it.dir), R, ctx, cache, min(z_prev, float(st.z_op_m)), va)
            segs.append(PathSegment2D(first_phase, Mode.CRUISE, p, None, None, va, False, z_prev, float(st.z_op_m)))
        else:
            assert prev is not None
            p = it.transfer.path if it.transfer is not None else None
            if p is None:
                p = _route(prev.end_state(prev_dir), st.start_state(it.dir), R, ctx, cache,
                           min(float(prev.z_op_m), float(st.z_op_m)), va)
            segs.append(PathSegment2D(Phase.TRANSFER, Mode.CRUISE, p, None, None, va, False, float(prev.z_op_m),
                                      float(st.z_op_m)))
        l = cache.idx[cls][it.strip_id]
        idx = (it.dir + 1) // 2
        v_op = float(cache.op_va[cls][l, idx]) if cls in cache.op_va and np.isfinite(cache.op_va[cls][l, idx]) else va
        line = np.vstack([st.pL, st.pR]) if it.dir == -1 else np.vstack([st.pR, st.pL])
        segs.append(PathSegment2D(Phase.OPERATE, Mode.CRUISE, line, it.strip_id, int(it.dir), v_op, True,
                                  float(st.z_op_m), float(st.z_op_m)))
        prev, prev_dir = st, it.dir
        pose = st.end_state(it.dir)
        z_prev = float(st.z_op_m)
    site = landing_site = None
    app = landing.app_state
    z_tc = float(landing.segments[0].z_hint_start_m) if landing.segments else z_prev
    p_rtb = rtb_path
    if app is not None and (p_rtb is None or math.hypot(p_rtb.end.x_m - app.x_m, p_rtb.end.y_m - app.y_m) > 1e-3
                            or math.hypot(p_rtb.start.x_m - pose.x_m, p_rtb.start.y_m - pose.y_m) > 1e-3):
        p_rtb = _route(pose, app, R, ctx, cache, min(z_prev, z_tc), va)
    if p_rtb is not None:
        segs.append(PathSegment2D(Phase.RTB, Mode.CRUISE, p_rtb, None, None, va, False, z_prev, z_tc))
    segs += list(landing.segments)
    del site, landing_site
    return segs


def _finish_plan(old: UAVPlan, uav: UAVInstance, keep_n: int, anchor: UAVState, segs: list[PathSegment2D],
                 landing_site: str, ctx: MissionContext, strips: Mapping[str, Strip], new_items: Sequence[SeqItem],
                 tag: str, keep_strips: Sequence[tuple[str, int]] | None = None) -> tuple[UAVPlan, list[Waypoint]]:
    """build_3d 尾段 → 既有航點[:keep_n] + 尾段 → 新 UAVPlan（原物件不變）。回傳 (plan, 尾段航點)。"""
    cls = uav.cls
    spec = ctx.specs[cls]
    model = ctx.models.get(uav.uav_id) or ctx.models_rep[cls]
    seq_like = UAVSequence(uav.uav_id, [], PathCost(0.0, 0.0, 0.0, None), None, landing_site,
                           replace(anchor), anchor.t_s, anchor.e_rem_wh, [], [], cls)
    pin_end = _z_tc_of(ctx, cls, landing_site)
    tail, stats = build_3d(segs, uav, seq_like, ctx, dict(strips), float(anchor.t_s), float(anchor.e_rem_wh),
                           float(anchor.z_m), pin_end)
    wps = list(old.waypoints[:keep_n]) + tail
    t_arr = next((w.t_s for w in tail if w.phase is Phase.TRANS_IN), None)
    if t_arr is None:
        t_arr = (wps[-1].t_s - spec.t_tr_in_s - spec.hover_descent_time_s()) if wps else anchor.t_s
    t_land = (wps[-1].t_s + spec.t_hover_reserve_s) if wps else anchor.t_s
    e_start = float(old.waypoints[0].energy_remaining_wh) if old.waypoints else anchor.e_rem_wh
    e_end = wps[-1].energy_remaining_wh - model.segment_wh(Mode.HOVER, spec.t_hover_reserve_s) if wps else anchor.e_rem_wh
    e_used = e_start - e_end
    dist = 0.0
    if len(wps) > 1:
        xy = np.array([[w.x_m, w.y_m] for w in wps])
        dist = float(np.hypot(*np.diff(xy, axis=0).T).sum())
    base_strips = list(keep_strips) if keep_strips is not None else list(old.strips)
    strips_out = base_strips + [(it.strip_id, int(it.dir)) for it in new_items]
    warnings = [w for w in old.warnings if not w.startswith("profile:")] + [f"profile:{v}" for v in stats.violations] + [tag]
    plan = UAVPlan(old.uav_id, cls, wps, strips_out, float(old.t_takeoff_s), float(t_arr), float(t_land), landing_site,
                   float(e_used), float(e_end - model.reserve_wh()), dist, float(model.cruise_equiv_km(e_used)),
                   int(old.orbits_inserted), warnings, takeoff_site=old.takeoff_site)
    return plan, tail


def _entry_path_for(cache: CostCache, cls: str, uav_id: str, anchor: UAVState, strip: Strip, dir: int) -> DubinsPath | None:
    idx = (dir + 1) // 2
    p = cache.entry_paths.get(uav_id, {}).get((strip.strip_id, idx))
    if p is not None:
        return p
    R = float(cache.R_eff.get(cls, anchor.r_eff_m))
    try:
        return dubins_path(_pose(anchor), strip.start_state(dir), R)
    except Exception:
        return None


# ---------------------------------------------------------------------------
# 重規劃工作階段
# ---------------------------------------------------------------------------
@dataclass(slots=True)
class _Anchor:
    """尾段錨點：保留既有航點 [:keep_n]，自 `state` 起追加。`strips_kept` = 保留之 (strip_id, dir)。"""
    keep_n: int
    state: UAVState
    strips_kept: list[tuple[str, int]]


class _Session:
    """單一事件之重規劃工作階段（規格 §12.1 通用框架）。"""

    def __init__(self, plan: MissionPlan, telemetry: Mapping[str, Telemetry], ctx: MissionContext, cache: CostCache,
                 cfg: PlannerConfig, t_now: float, ghosts: Sequence[GhostTrack] | None) -> None:
        self.plan = plan
        self.ctx = ctx
        self.cache = cache
        self.cfg = cfg
        self.t_now = float(t_now)
        self.telemetry = dict(telemetry)
        self.plans: dict[str, UAVPlan] = dict(plan.plans)
        self.changed: list[str] = []
        self.anchors: dict[str, int] = {}                 # uav → keep_n（diff 起點）
        self.notes: list[str] = []
        self.new_strips: dict[str, Strip] = {}
        self.new_cells: list[Cell] = []
        self.uncovered: list[tuple[str, str]] = list(plan.uncovered)
        self.ghosts: list[GhostTrack] = list(ghosts or [])
        self.timing: dict[str, float] = {}
        self._tick = time.perf_counter()
        self.states, self.pending_ids, self.residual = snapshot(plan, self.telemetry, self.t_now, ctx)
        self.lap("snapshot")
        self.strips_all: dict[str, Strip] = dict(plan.strips)
        self.cell_by_id, self.strip_cell = _cell_maps(plan)
        self.exclude: set[str] = set()

    # ---- 計時 --------------------------------------------------------------
    def lap(self, name: str) -> None:
        now = time.perf_counter()
        self.timing[name] = self.timing.get(name, 0.0) + (now - self._tick) * 1e3
        self._tick = now

    # ---- 便利 --------------------------------------------------------------
    def inst(self, u: str) -> UAVInstance:
        return self.ctx.uav(u)

    def model(self, u: str) -> EnergyModel:
        return self.ctx.models.get(u) or self.ctx.models_rep[self.plans[u].cls]

    def spec(self, u: str) -> PlatformSpec:
        return self.ctx.specs[self.plans[u].cls]

    def status(self, u: str) -> _Status:
        return _status(self.plans[u], self.t_now)

    def makespan(self) -> float:
        return max((p.t_land_s for p in self.plans.values()), default=0.0)

    def strip(self, sid: str) -> Strip:
        s = self.strips_all.get(sid)
        if s is None:
            s = self.new_strips[sid]
        return s

    def cell_of_strip(self, sid: str) -> Cell | None:
        base = sid.split("#", 1)[0]
        c = self.strip_cell.get(base) or self.strip_cell.get(sid)
        if c is None:
            s = self.strips_all.get(sid) or self.strips_all.get(base) or self.new_strips.get(sid)
            if s is not None:
                c = self.cell_by_id.get(str(s.cell_id))
        return c

    def register_strips(self, strips: Iterable[Strip], cell: Cell | None = None) -> None:
        for s in strips:
            self.strips_all[s.strip_id] = s
            self.new_strips[s.strip_id] = s
            if cell is not None:
                self.strip_cell[s.strip_id] = cell

    def mark_changed(self, u: str, keep_n: int) -> None:
        if u not in self.changed:
            self.changed.append(u)
        self.anchors[u] = min(keep_n, self.anchors.get(u, keep_n))

    # ---- 錨點 --------------------------------------------------------------
    def anchor_end(self, u: str) -> _Anchor:
        """序列末端錨點（末條帶端點）；若該時刻已過（正在返航）→ 現時錨點。"""
        p = self.plans[u]
        spans = _spans(p)
        if spans and spans[-1].t_end > self.t_now:
            return self.anchor_after_span(u, spans[-1])
        return self.anchor_now(u)

    def anchor_after_span(self, u: str, span: _Span) -> _Anchor:
        p = self.plans[u]
        cls = p.cls
        spec = self.spec(u)
        w = p.waypoints[span.i_end]
        st = self.strip(span.strip_id)
        pose = st.end_state(span.dir)
        wind_w = self.ctx.wind.speed_at(float(w.z_m)) if self.ctx.wind is not None else 0.0
        R = float(self.cache.R_eff.get(cls, spec.r_eff(wind_w)))
        state = UAVState(pose.x_m, pose.y_m, pose.theta_rad, float(w.z_m), float(w.t_s), float(w.energy_remaining_wh),
                         R, float(spec.cruise_airspeed_mps))
        kept = [(s.strip_id, s.dir) for s in spans_upto(_spans(p), span)]
        return _Anchor(span.i_end, state, kept)

    def anchor_now(self, u: str) -> _Anchor:
        p = self.plans[u]
        stt = self.status(u)
        kept = [(s.strip_id, s.dir) for s in stt.done]
        if stt.current is not None:
            kept.append((stt.current.strip_id, stt.current.dir))
        j_pre = _takeoff_prefix_end(p)
        if j_pre >= 0 and self.t_now < float(p.waypoints[j_pre].t_s) - _EPS:
            # R-09：該機尚未離場（起飛走廊還沒飛完；規劃器刻意錯開起飛，hills 最後一架 B2 在 2914 s 才起飛）。
            # 舊碼以 t_now 建錨：_index_at 夾到 0 → 保留 TAKEOFF 航點的**未來**時戳，尾段卻自 t_now 起算，
            # 產生時間倒退的 UAVPlan；deconflict._waypoint_arrays 直接丟 ValueError，replan 整個崩潰。
            # 正確作法：保留整段起飛走廊，錨在其出口並沿用該航點自己的時刻。
            return _Anchor(j_pre + 1, _state_at_wp(p, j_pre, self.spec(u), self.ctx.wind), kept)
        st = self.states.get(u) or _interp_state(p, self.t_now, self.spec(u), self.ctx.wind)
        j = _index_at(p, self.t_now)
        return _Anchor(j + 1, replace(st, t_s=max(st.t_s, self.t_now)), kept)

    # ---- IDP 尾段 -------------------------------------------------------------
    def idp_tail(self, u: str, anchor: _Anchor, strip_ids: Sequence[str], cell_order: Sequence[str] | None = None
                 ) -> tuple[UAVSequence | None, list[str]]:
        """自錨點對 strip_ids 跑 beam_idp（暖啟動 = 無；錨點為初始狀態）+ append_rtb；能量不足時自尾端逐條丟棄。
        回傳 (含 RTB 之序列 | None, leftover)。"""
        cache, ctx, cfg = self.cache, self.ctx, self.cfg
        p = self.plans[u]
        cls = p.cls
        ids = [s for s in strip_ids if s in cache.idx.get(cls, {})]
        missing = [s for s in strip_ids if s not in cache.idx.get(cls, {})]
        if not ids:
            return None, list(strip_ids)
        cache.refresh_entry(u, replace(anchor.state), 0, 0.0, ctx)
        cells = list(dict.fromkeys(str(cache.strips[cls][s].cell_id) for s in ids))
        team = Team(f"replan_{u}", cls, [u], cells, list(ids), {u: 0.0}, {u: 0})
        order = {u: list(cell_order)} if cell_order else {u: cells}
        res = beam_idp(team, cache.strips[cls], cache, ctx.models, {u: replace(anchor.state)}, cfg,
                       cell_order=order, site_id=ctx.recovery_site_id)
        seq = res.sequences.get(u)
        leftover = list(res.leftover) + missing
        if seq is None or not seq.items:
            return None, list(dict.fromkeys(list(ids) + leftover))
        model = self.model(u)
        while seq.items:
            out = append_rtb(seq, ctx, cache, model, cfg)
            if not isinstance(out, EnergyInfeasible):
                return out, list(dict.fromkeys(leftover))
            dropped = seq.items[-1]
            leftover.append(dropped.strip_id)
            seq = _drop_last(seq, cache, cls)
        return None, list(dict.fromkeys(list(ids) + leftover))

    def idp_chain(self, u: str, anchor: _Anchor, groups: Sequence[Sequence[str]]) -> tuple[UAVSequence | None, list[str]]:
        """依序對每組條帶跑 `idp_tail`（前組全部完成後才進入次組：搶占／確認優先，規格 §12.7、§12.8）並串接；
        回傳 (含 RTB 之序列 | None, leftover)。"""
        cache = self.cache
        cls = self.plans[u].cls
        cur = anchor
        merged: UAVSequence | None = None
        leftover: list[str] = []
        for grp in groups:
            ids = [s for s in grp if s not in leftover]
            if not ids:
                continue
            seq, left = self.idp_tail(u, cur, ids)
            leftover += left
            if seq is None or not seq.items:
                continue
            if merged is None:
                merged = seq
            else:
                first = replace(seq.items[0], transfer=seq.entry)
                merged = replace(merged, items=list(merged.items) + [first] + list(seq.items[1:]),
                                 decisions=list(merged.decisions) + list(seq.decisions), rtb=seq.rtb,
                                 landing_site=seq.landing_site, state_end=seq.state_end, t_end_s=seq.t_end_s,
                                 e_end_wh=seq.e_end_wh, warnings=list(merged.warnings) + list(seq.warnings))
            se = merged.state_end
            R = float(cache.R_eff.get(cls, cur.state.r_eff_m))
            cur = _Anchor(cur.keep_n, UAVState(se.x_m, se.y_m, se.theta_rad, float(se.z_m), float(se.t_s),
                                                float(se.e_rem_wh), R, cur.state.v_air_mps), cur.strips_kept)
        return merged, list(dict.fromkeys(leftover))

    def apply_seq(self, u: str, anchor: _Anchor, seq: UAVSequence, tag: str,
                  pre: Sequence[PathSegment2D] = ()) -> UAVPlan:
        """以序列（含 rtb）自錨點組尾段 → 新 UAVPlan 並登記變更。"""
        cache, ctx = self.cache, self.ctx
        p = self.plans[u]
        cls = p.cls
        inst = self.inst(u)
        site = seq.landing_site or ctx.recovery_site_id
        first = seq.items[0]
        s0 = cache.strips[cls][first.strip_id]
        entry_path = seq.entry.path if seq.entry is not None else None
        if entry_path is None:
            entry_path = _entry_path_for(cache, cls, u, anchor.state, s0, first.dir)
        se = seq.state_end
        app, ld = _site_app(ctx, cache, inst, site, Pose(se.x_m, se.y_m, se.theta_rad), float(se.z_m))
        rtb_path = seq.rtb.path if seq.rtb is not None else None
        segs = _tail_segments(cls, anchor.state, seq.items, entry_path, rtb_path, ld, cache, ctx, pre=pre)
        new, tail = _finish_plan(p, inst, anchor.keep_n, anchor.state, segs, site, ctx, self.strips_all, seq.items, tag,
                                 keep_strips=anchor.strips_kept)
        new.warnings += [f"corridor:{i}" for i in ld.issues]
        if site != ctx.recovery_site_id and not any(w.startswith("planned_landing_at_alternate") for w in new.warnings):
            new.warnings.append(f"planned_landing_at_alternate({site})")
        self.plans[u] = new
        self.mark_changed(u, anchor.keep_n)
        return new

    def rtb_now(self, u: str, anchor: _Anchor, tag: str, site_id: str | None = None,
                ok_sites: Sequence[str] | None = None) -> UAVPlan:
        """自錨點直接返航（cut／DIVERT／中止）：站點 = site_id 或 能量可達且 point_wh 最小者（回收點優先）。"""
        cache, ctx = self.cache, self.ctx
        p = self.plans[u]
        cls = p.cls
        inst = self.inst(u)
        model = self.model(u)
        st = anchor.state
        site = site_id or self.best_site(u, st, ok_sites)
        app, ld = _site_app(ctx, cache, inst, site, _pose(st), float(st.z_m))
        segs = _tail_segments(cls, st, [], None, None, ld, cache, ctx)
        new, tail = _finish_plan(p, inst, anchor.keep_n, st, segs, site, ctx, self.strips_all, [], tag,
                                 keep_strips=anchor.strips_kept)
        new.warnings += [f"corridor:{i}" for i in ld.issues]
        if site != ctx.recovery_site_id:
            new.warnings.append(f"planned_landing_at_alternate({site})")
        if self.cfg.enforce_energy and new.energy_margin_wh < -_EPS:
            new.warnings.append(f"energy_margin_negative({new.energy_margin_wh:.1f} Wh)")
        del model
        self.plans[u] = new
        self.mark_changed(u, anchor.keep_n)
        return new

    def best_site(self, u: str, st: UAVState, ok_sites: Sequence[str] | None = None) -> str:
        cache, ctx = self.cache, self.ctx
        cls = self.plans[u].cls
        model = self.model(u)
        sites = [s for s in cache.budget.site_ids] if cache.budget.site_ids else list(ctx.sites_enu)
        if ok_sites is not None:
            sites = [s for s in sites if s in set(ok_sites)] or sites
        recovery = ctx.recovery_site_id
        costs: dict[str, float] = {}
        for s in sites:
            try:
                costs[s] = float(cache.budget.point_wh(cls, st.xy, float(st.z_m), s))
            except Exception:
                costs[s] = float(np.hypot(*(st.xy - ctx.sites_enu[s][0]))) / 10.0
        if recovery in costs and st.e_rem_wh - costs[recovery] >= model.reserve_wh() - _EPS:
            return recovery
        return min(costs, key=costs.get) if costs else recovery

    # ---- 能量估計（快取層，含 κ）-------------------------------------------
    def tail_energy(self, u: str, stt: _Status, n_keep: int, site: str | None = None) -> float:
        """自現時起：完成現行條帶（剩餘比例）+ 前 n_keep 條待掃（含轉移）+ RTB（自最後條帶）+ e_LDG（κ 修正）。"""
        cache, ctx = self.cache, self.ctx
        p = self.plans[u]
        cls = p.cls
        model = self.model(u)
        site = site or p.landing_site or ctx.recovery_site_id
        idx_map = cache.idx.get(cls, {})
        e = 0.0
        last: tuple[int, int] | None = None
        if stt.current is not None and stt.current.strip_id in idx_map:
            l = idx_map[stt.current.strip_id]
            i = (stt.current.dir + 1) // 2
            e += (1.0 - stt.progress) * float(cache.op_e[cls][l, i])
            last = (l, i)
        for sp in stt.pending[:n_keep]:
            if sp.strip_id not in idx_map:
                continue
            m = idx_map[sp.strip_id]
            im = (sp.dir + 1) // 2
            if last is not None:
                e += float(cache.tr_e[cls][last[0], m, 2 * last[1] + im])
            e += float(cache.op_e[cls][m, im])
            last = (m, im)
        if last is not None and (cls, site) in cache.rtb_e:
            e += float(cache.rtb_e[(cls, site)][last[0], last[1]])
        elif last is None:
            st = self.states.get(u)
            if st is not None:
                try:
                    e += float(cache.budget.point_wh(cls, st.xy, float(st.z_m), site)) - model.e_landing_wh()
                except Exception:
                    pass
        return float(e) * float(model.kappa) + float(model.e_landing_wh())

    def tail_time(self, u: str, stt: _Status, site: str | None = None) -> float:
        cache, ctx = self.cache, self.ctx
        p = self.plans[u]
        cls = p.cls
        site = site or p.landing_site or ctx.recovery_site_id
        idx_map = cache.idx.get(cls, {})
        t = 0.0
        last: tuple[int, int] | None = None
        if stt.current is not None and stt.current.strip_id in idx_map:
            l = idx_map[stt.current.strip_id]
            i = (stt.current.dir + 1) // 2
            t += (1.0 - stt.progress) * float(cache.op[cls][l, i])
            last = (l, i)
        for sp in stt.pending:
            if sp.strip_id not in idx_map:
                continue
            m = idx_map[sp.strip_id]
            im = (sp.dir + 1) // 2
            if last is not None:
                t += float(cache.tr[cls][last[0], m, 2 * last[1] + im])
            t += float(cache.op[cls][m, im])
            last = (m, im)
        if last is not None and (cls, site) in cache.rtb:
            t += float(cache.rtb[(cls, site)][last[0], last[1]])
        return float(t) + _landing_fixed_time_s(self.spec(u))

    # ---- 切除（規格 §12.4）-----------------------------------------------------
    def cut_to_fit(self, u: str, e_ref: float, tag: str, divert: bool = False) -> list[str]:
        """自尾端逐條移除作業路徑至 E_ref − e(剩餘含 RTB) ≥ r·E_nom；移除者回傳（→ L_uncov）。
        `divert=True` → 立即 DIVERT 至最近備降點，全部剩餘 → L_uncov。"""
        p = self.plans[u]
        stt = self.status(u)
        model = self.model(u)
        reserve = float(model.reserve_wh())
        removed: list[str] = []
        if divert:
            anchor = self.anchor_now(u)
            for sp in stt.pending:
                removed.append(sp.strip_id)
            if stt.current is not None:
                r = self.residual.get(stt.current.strip_id)
                if r is not None:
                    self.register_strips([r], self.cell_of_strip(stt.current.strip_id))
                    removed.append(r.strip_id)
                anchor.strips_kept = [k for k in anchor.strips_kept if k[0] != stt.current.strip_id]
            self.rtb_now(u, anchor, tag)
            return removed
        n_pend = len(stt.pending)
        for n_keep in range(n_pend, -1, -1):
            need = self.tail_energy(u, stt, n_keep)
            if e_ref - need >= reserve - _EPS:
                if n_keep == n_pend:
                    return []                                    # 無須切除
                removed = [sp.strip_id for sp in stt.pending[n_keep:]]
                if n_keep > 0:
                    anchor = self.anchor_after_span(u, stt.pending[n_keep - 1])
                elif stt.current is not None:
                    anchor = self.anchor_after_span(u, stt.current)
                else:
                    anchor = self.anchor_now(u)
                self.rtb_now(u, anchor, tag)
                return removed
        # 連現行條帶都無法完成 → 立即返航（殘段 → L_uncov）
        removed = [sp.strip_id for sp in stt.pending]
        anchor = self.anchor_now(u)
        if stt.current is not None:
            r = self.residual.get(stt.current.strip_id)
            if r is not None:
                self.register_strips([r], self.cell_of_strip(stt.current.strip_id))
                removed.append(r.strip_id)
            anchor.strips_kept = [k for k in anchor.strips_kept if k[0] != stt.current.strip_id]
        self.rtb_now(u, anchor, tag)
        return removed

    # ---- 接手（規格 §12.1 步 3–5、§12.2）-----------------------------------------
    def reassign(self, strip_ids: Sequence[str], exclude: Iterable[str] = (), max_changes: int = 2,
                 reason: str = "no_candidate", allow_cross_class: bool = True,
                 candidates: Sequence[str] | None = None) -> list[str]:
        """L_uncov → 受影響集合 A（`affected_set`）→ 依 Δmakespan 最小者尾端追加（至多 max_changes 機）；
        同構型保留原條帶、跨構型（rgb/any）殘區以接手構型重鋪。回傳仍未覆蓋之條帶 id。"""
        cache, ctx, cfg = self.cache, self.ctx, self.cfg
        ids = [s for s in dict.fromkeys(strip_ids) if s in self.strips_all]
        if not ids:
            return []
        excl = set(exclude) | self.exclude
        # 更新快照（變更後之計畫）
        states = {u: (self.states.get(u) or _interp_state(p, self.t_now, self.spec(u), self.ctx.wind))
                  for u, p in self.plans.items() if p.waypoints}
        plan_view = replace(self.plan, plans=dict(self.plans), strips=dict(self.strips_all),
                            cells=list(self.plan.cells) + self.new_cells)
        cands = candidates if candidates is not None else affected_set(plan_view, ids, states, ctx, cache, cfg)
        cands = [u for u in cands if u not in excl and not self.status(u).landed]
        remaining = list(ids)
        changes = 0
        relay_n = 0
        while remaining and changes < max_changes and cands:
            best: tuple[float, str, UAVSequence, _Anchor, list[str], list[Strip], Cell | None] | None = None
            for u in cands:
                cls = self.plans[u].cls
                inst = self.inst(u)
                # 構型分組：同構型 → 原條帶；跨構型 → 重鋪
                same = [s for s in remaining if self.strip(s).cls == cls]
                cross = [s for s in remaining if self.strip(s).cls != cls]
                use: list[str] = []
                relaid: list[Strip] = []
                relay_cell: Cell | None = None
                if same:
                    for s in same:
                        st = self.strip(s)
                        c = self.cell_of_strip(s)
                        req = c.sensor_req if c is not None else "any"
                        if not _dec.class_can_serve(req, cls, inst.available_kinds):
                            continue
                        if s not in cache.idx.get(cls, {}):
                            self._add_to_cache(cls, [st], c)
                        use.append(s)
                if not use and cross and allow_cross_class:
                    relay_cell, relaid = self.relay_cross(cross, cls, inst, relay_n)
                    if relaid:
                        use = [s.strip_id for s in relaid]
                if not use:
                    continue
                anchor = self.anchor_end(u)
                seq, leftover = self.idp_tail(u, anchor, use)
                if seq is None:
                    continue
                t_land_new = seq.t_end_s + (seq.rtb.t_s if seq.rtb is not None else 0.0) + _landing_fixed_time_s(self.spec(u))
                others = max((p.t_land_s for v, p in self.plans.items() if v != u), default=0.0)
                dms = max(others, t_land_new) - self.makespan()
                score = (len(leftover), dms, t_land_new)
                if best is None or score < best[0]:
                    best = (score, dms, u, seq, anchor, leftover, relaid, relay_cell)
            if best is None:
                break
            _score, dms, u, seq, anchor, leftover, relaid, relay_cell = best
            if relaid:
                self.register_strips(relaid, relay_cell)
                if relay_cell is not None:
                    self.new_cells.append(relay_cell)
                    self.cell_by_id[relay_cell.cell_id] = relay_cell
                relay_n += 1
                covered_src = [s for s in remaining if self.strip(s).cls != self.plans[u].cls]
                self.notes.append(f"relaid({','.join(covered_src)} -> {self.plans[u].cls}:{len(relaid)} strips)")
                taken_src = covered_src
            else:
                taken_src = [it.strip_id for it in seq.items]
            self.apply_seq(u, anchor, seq, f"replan_takeover({','.join(it.strip_id for it in seq.items)})")
            self.notes.append(f"takeover({u}: {[it.strip_id for it in seq.items]}, dmakespan={dms:.1f} s)")
            for s in taken_src:
                if s in remaining:
                    remaining.remove(s)
            if relaid and leftover:
                # 重鋪後仍有未鋪滿之新條帶 → 以其 id 記錄
                for s in leftover:
                    if s not in remaining:
                        remaining.append(s)
            changes += 1
            cands = [c for c in cands if c != u]
        for s in remaining:
            self.uncovered.append((s, reason))
        return remaining

    def _add_to_cache(self, cls: str, strips: Sequence[Strip], cell: Cell | None) -> None:
        cache, ctx = self.cache, self.ctx
        dc = ctx.decompose_ctx()
        req = cell.payload_req_id if cell is not None else "default"
        found = _dec.resolve_sensor(dc, cls, req)
        for s in strips:
            if found is not None:
                cache.strip_sensors[s.strip_id] = found[0]
        new = [s for s in strips if s.strip_id not in cache.idx.get(cls, {})]
        if new:
            cache.add_strips(cls, new, list(ctx.inputs.fleet), {}, cache.sites, ctx)
        self.register_strips(strips, cell)

    def relay_cross(self, strip_ids: Sequence[str], cls_new: str, inst: UAVInstance, n: int) -> tuple[Cell | None, list[Strip]]:
        """跨構型接手（規格 §12.2：rgb/any 殘區以接手構型條帶重鋪）：殘區 = 未覆蓋條帶足跡 ∩ 母胞 → 新子胞 → `_solve_class`。"""
        ctx = self.ctx
        groups: dict[str, list[Strip]] = {}
        for s in strip_ids:
            st = self.strip(s)
            c = self.cell_of_strip(s)
            if c is None:
                continue
            if not _dec.class_can_serve(c.sensor_req, cls_new, inst.available_kinds) or c.sensor_req == "both":
                continue
            groups.setdefault(c.cell_id, []).append(st)
        if not groups:
            return None, []
        cid, strips = max(groups.items(), key=lambda kv: len(kv[1]))
        cell = self.cell_by_id[cid]
        region = strips_union_footprint(strips).intersection(cell.polygon.buffer(0))
        parts = [g for g in getattr(region, "geoms", [region]) if isinstance(g, Polygon) and g.area > 100.0]
        if not parts:
            return None, []
        poly = max(parts, key=lambda g: g.area)
        dc = ctx.decompose_ctx()
        new_cell = _dec._new_cell(f"{cid}_R{n + 1}", cell, poly, dc)
        new_cell.parent_cell_id = cell.cell_id
        new_cell.residual_of = cell.cell_id
        try:
            _dec._solve_class(new_cell, cls_new, dc, rho_sym_m=ctx.rho_sym_m, t_use_s=None, n_uav=None)
        except Exception as exc:  # pragma: no cover - 防禦
            self.notes.append(f"relay_failed({cid}: {exc})")
            return None, []
        if not new_cell.feasible.get(cls_new, False) or not new_cell.strips.get(cls_new):
            self.notes.append(f"relay_infeasible({cid}: {new_cell.infeasible_reason.get(cls_new, '')})")
            return None, []
        out = list(new_cell.strips[cls_new])
        self._add_to_cache(cls_new, out, new_cell)
        return new_cell, out

    # ---- 去衝突（僅變更機；延遲盤旋）------------------------------------------
    def deconflict(self) -> None:
        cfg, ctx = self.cfg, self.ctx
        changed = [u for u in self.changed if u in self.plans]
        if not changed:
            return
        plans = {u: p for u, p in self.plans.items() if p.waypoints}
        all_ids = sorted(plans)
        ghost_ids = {g.uav_id for g in self.ghosts}
        inserted: set[tuple[str, int]] = set()
        for _ in range(_ORBIT_ROUNDS):
            # skip 每輪重算：為避讓幽靈而被動到的實機這一輪起也算 changed（R-11）
            skip = {(a, b) for i, a in enumerate(all_ids) for b in all_ids[i + 1:]
                    if a not in changed and b not in changed}
            conflicts = check_separation(plans, cfg, self.ghosts or None, skip_pairs=skip, t_sync_s=self.t_now)
            sched = CorridorSchedule(cfg)
            for u, p in plans.items():
                sched.reserve_plan(p, ctx.inputs.home.site_id)
            for g in self.ghosts:
                sched.reserve_ghost(g)
            conflicts += [c for c in sched.conflicts() if c.t_s >= self.t_now - 1e-6]
            # 幽靈與其本機之預載計畫重合，非衝突；失鏈機本身不可指揮（不插入盤旋）。
            # R-11：**幽靈對手不受 `changed` 過濾**（規格 §12.5「其他機在 §13 中對其避讓」）。舊碼要求衝突雙方
            # 至少一方在 changed 裡，而 GhostTrack 的 id 永遠不在 changed，於是所有「未被重規劃之實機 vs 幽靈」
            # 的衝突在任何修復之前就被丟掉，也不會列入 residual_conflicts。
            def _keep(c: Conflict) -> bool:
                if c.uav_a == c.uav_b:
                    return False
                if c.uav_a in changed or c.uav_b in changed:
                    return True
                return (c.uav_a in ghost_ids) != (c.uav_b in ghost_ids)      # 幽靈 vs 實機 → 一律處理

            conflicts = [c for c in conflicts if _keep(c)]
            if not conflicts:
                break
            touched: set[str] = set()
            for c in sorted(conflicts, key=lambda c: c.t_s):
                cand = [x for x in (c.uav_a, c.uav_b) if x not in ghost_ids and x in plans]
                u = next((x for x in cand if x in changed), None)
                if u is None:
                    # 幽靈對手：允許動尚未被重規劃的實機（它才是可指揮的一方）
                    u = cand[0] if (cand and any(x in ghost_ids for x in (c.uav_a, c.uav_b))) else None
                if u is None or u in touched:
                    continue
                p = plans[u]
                k0 = self.anchors.get(u, 0)
                site_xy = ctx.sites_enu.get(p.landing_site, (None,))[0] if c.kind == "corridor" else None
                idx = orbit_insert_index(p, c.t_s, corridor=(c.kind == "corridor"), site_xy=site_xy)
                if idx is None or idx < k0:
                    # 錨點前不可改：取錨點後、衝突前之最後可插入階段
                    idx = next((i for i in range(len(p.waypoints) - 1, k0 - 1, -1)
                                if p.waypoints[i].phase in (Phase.TRANSFER, Phase.RTB, Phase.ENTRY, Phase.ORBIT)
                                and p.waypoints[i].t_s <= c.t_s), None)
                if idx is None or (u, idx) in inserted:
                    continue                      # 同一點已插入延遲盤旋仍衝突 → 空間重疊，非時間可解；留作殘餘
                inserted.add((u, idx))
                spec = ctx.specs[p.cls]
                z = float(p.waypoints[idx].z_m)
                t_lap, r_orb = orbit_lap_time_s(spec, ctx.wind, z, self.cache.R_eff.get(p.cls))
                dt_req = float(getattr(c, "dt_req_s", 0.0) or 0.0)
                n = max(1, int(math.ceil(max(dt_req, cfg.dt_4d_s) / max(t_lap, 1.0))))
                model = self.model(u)
                new = insert_delay_orbit(p, idx, n, t_lap, model, r_orb)
                if cfg.enforce_energy and new.energy_margin_wh < -_EPS:
                    self.notes.append(f"deconflict_energy_blocked({u})")
                    continue
                plans[u] = new
                self.plans[u] = new
                self.mark_changed(u, idx)                 # R-11：為避讓幽靈而動到的實機也算 changed
                if u not in changed:
                    changed.append(u)
                touched.add(u)
            if not touched:
                self.notes.append(f"residual_conflicts({len(conflicts)})")
                break

    # ---- 收尾 -------------------------------------------------------------------
    def finish(self, event: Event, t_begin: float, extra_uncovered_reason: str | None = None) -> ReplanResult:
        self.lap("replan")
        self.deconflict()
        self.lap("deconflict")
        diff: dict[str, list[Waypoint]] = {}
        for u in self.changed:
            if u in self.plans:
                diff[u] = list(self.plans[u].waypoints[self.anchors.get(u, 0):])
            else:
                diff[u] = []
        makespan = max((p.t_land_s for p in self.plans.values()), default=0.0)
        cells = list(self.plan.cells) + self.new_cells
        strips = dict(self.strips_all)
        uncovered = list(dict.fromkeys(self.uncovered))
        cache = self.cache
        plan = MissionPlan(dict(self.plans), float(makespan), cells, strips, uncovered, self.plan.feasibility,
                           dict(self.plan.timing_ms), (int(cache.geometry_key), int(cache.wind_key), int(cache.state_key)),
                           diff_from=type(event).__name__)
        total = (time.perf_counter() - t_begin) * 1e3
        self.timing["total"] = total
        placeholder_used = total > REPLAN_BUDGET_MS
        placeholders: dict[str, list[Waypoint]] = {}
        if placeholder_used:
            self.notes.append(f"placeholder_dispatched(total={total:.0f} ms > {REPLAN_BUDGET_MS:.0f} ms)")
            try:
                placeholders = placeholder_plan(self.plan, [u for u in self.changed if u in self.plan.plans],
                                                self.ctx, cache, t_now_s=self.t_now)
            except Exception as exc:  # pragma: no cover
                self.notes.append(f"placeholder_failed({exc})")
        return ReplanResult(plan, list(self.changed), diff, placeholder_used, dict(self.timing), list(self.notes),
                            list(self.ghosts), placeholders, float(self.t_now))


def spans_upto(spans: Sequence[_Span], span: _Span) -> list[_Span]:
    out: list[_Span] = []
    for s in spans:
        out.append(s)
        if s is span or (s.strip_id == span.strip_id and s.i0 == span.i0):
            break
    return out


def _drop_last(seq: UAVSequence, cache: CostCache, cls: str) -> UAVSequence:
    items = list(seq.items[:-1])
    decisions = list(seq.decisions[:-1])
    if not items:
        return replace(seq, items=[], decisions=[], rtb=None, landing_site=None)
    last = items[-1]
    st = cache.strips[cls][last.strip_id]
    pe = st.end_state(last.dir)
    se = UAVState(pe.x_m, pe.y_m, pe.theta_rad, float(st.z_op_m), last.t_end_s, last.e_end_wh, seq.state_end.r_eff_m,
                  seq.state_end.v_air_mps)
    return replace(seq, items=items, decisions=decisions, rtb=None, landing_site=None, state_end=se, t_end_s=last.t_end_s,
                   e_end_wh=last.e_end_wh)


# ---------------------------------------------------------------------------
# placeholder / ghost（契約）
# ---------------------------------------------------------------------------
def placeholder_plan(plan: MissionPlan, uav_ids: list[str], ctx: MissionContext, cache: CostCache, *,
                     t_now_s: float | None = None) -> dict[str, list[Waypoint]]:
    """佔位計畫（規格 §12.1 時限）：續飛現行條帶（至其端點）+ 保守 RTB（AVOID 至能量最省之降落點）。
    `t_now_s` None → 各機首條帶起始時刻（契約外關鍵字，見 CONTRACT_DEVIATIONS）。"""
    out: dict[str, list[Waypoint]] = {}
    for u in uav_ids:
        p = plan.plans.get(u)
        if p is None or not p.waypoints:
            continue
        spans = _spans(p)
        t = float(t_now_s) if t_now_s is not None else (spans[0].t0 if spans else p.waypoints[0].t_s)
        sess = _Session(plan, {}, ctx, cache, ctx.cfg, t, None)
        stt = sess.status(u)
        if stt.current is not None:
            anchor = sess.anchor_after_span(u, stt.current)
        elif stt.pending:
            anchor = sess.anchor_now(u)
        else:
            anchor = sess.anchor_now(u)
        new = sess.rtb_now(u, anchor, "placeholder")
        out[u] = list(new.waypoints[anchor.keep_n:])
    return out


def ghost_track(uav_id: str, plan: MissionPlan, policy: str, t_loss_s: float, ctx: MissionContext, cfg: PlannerConfig,
                *, cache: CostCache | None = None) -> GhostTrack:
    """失鏈機 4D 幽靈航跡（規格 §12.5、D26）：RTB → 自 t_loss 之狀態 AVOID 至計畫降落點；CONTINUE_THEN_RTB → 沿計畫
    續飛 t_ll 後返航；NEAREST_ALT → 至該航點之 nearest_alternate。樣本 Δt = cfg.dt_4d_s、σ_t = k_t·(ETA − t_loss)。"""
    p = plan.plans[uav_id]
    spec = ctx.specs[p.cls]
    va = float(spec.cruise_airspeed_mps)
    t_ll = float(ctx.inputs.c2.t_ll_s)
    t_dec = float(t_loss_s) + (t_ll if policy == "CONTINUE_THEN_RTB" else 0.0)
    t_dec = min(t_dec, float(p.waypoints[-1].t_s))
    # 決策點前沿計畫
    dt = float(cfg.dt_4d_s) if cfg.dt_4d_s > 0 else _GHOST_DT_S
    grid = np.arange(float(t_loss_s), t_dec, dt)
    samples: list[np.ndarray] = []
    for t in grid:
        s = _interp_state(p, float(t), spec, ctx.wind)
        samples.append(np.array([s.x_m, s.y_m, s.z_m, float(t)]))
    st = _interp_state(p, t_dec, spec, ctx.wind)
    j = _index_at(p, t_dec)
    wp = p.waypoints[j]
    if policy == "NEAREST_ALT" or wp.lost_link_action.startswith("ALT:"):
        site = wp.lost_link_action.split(":", 1)[1] if wp.lost_link_action.startswith("ALT:") else wp.nearest_alternate
    else:
        site = p.landing_site
    if site not in ctx.sites_enu:
        site = ctx.recovery_site_id
    inst = ctx.uav(uav_id)
    ld = landing_corridor(site, inst, ctx, _pose(st), float(st.z_m))
    app = ld.app_state
    if app is None:
        sx, sy = ctx.sites_enu[site][0]
        app = Pose(float(sx), float(sy), st.theta_rad)
    R = float(cache.R_eff.get(p.cls, st.r_eff_m)) if cache is not None else float(st.r_eff_m)
    try:
        path = avoid(_pose(st), app, R, ctx.keepouts, float(st.z_m), ctx.wind, va, float(cfg.b_trk_m),
                     cache.tg_cache if cache is not None else {})
    except RoutingError:
        path = dubins_path(_pose(st), app, R)
    t_path = path_time(path, va, ctx.wind, float(st.z_m))
    if not math.isfinite(t_path):
        t_path = path.length_m / max(cfg.v_g_min_mps, 1.0)
    z_tc = _z_tc_of(ctx, p.cls, site)
    L = float(path.length_m)
    n_s = max(2, int(math.ceil(L / max(va * dt, 1.0))) + 1)
    pts = path.sample(max(L / (n_s - 1), 1.0)) if L > 1e-6 else np.array([[st.x_m, st.y_m, st.theta_rad]])
    frac = np.linspace(0.0, 1.0, pts.shape[0])
    for f, q in zip(frac, pts):
        samples.append(np.array([q[0], q[1], st.z_m + f * (z_tc - st.z_m), t_dec + f * t_path]))
    eta = t_dec + t_path + spec.transition_cmd().t_acc_s
    arr = np.array(samples, dtype=float).reshape(-1, 4) if samples else np.zeros((0, 4))
    sigma = float(cfg.k_t) * max(0.0, eta - float(t_loss_s))
    return GhostTrack(uav_id, arr, sigma, site, float(eta), policy, float(t_loss_s), float(t_dec))


# ---------------------------------------------------------------------------
# 事件 handler
# ---------------------------------------------------------------------------
def _begin(plan: MissionPlan, event: Event, telemetry: Mapping[str, Telemetry] | None, ctx: MissionContext,
           cache: CostCache, cfg: PlannerConfig, ghosts: Sequence[GhostTrack] | None) -> tuple[_Session, float]:
    t0 = time.perf_counter()
    sess = _Session(plan, telemetry or {}, ctx, cache, cfg, float(event.t_s), ghosts)
    return sess, t0


def _exit_uav(sess: _Session, u: str, t_exit: float, keep_plan: bool, tag: str) -> list[str]:
    """退出機之未完成條帶（含殘段）→ L_uncov；`keep_plan=False` → 自計畫移除。回傳 L_uncov。"""
    p = sess.plans.get(u)
    if p is None:
        return []
    stt = _status(p, t_exit)
    lost = [sp.strip_id for sp in stt.pending]
    if stt.current is not None:
        r = sess.residual.get(stt.current.strip_id)
        if r is None:
            strip = sess.strip(stt.current.strip_id)
            r = _truncate_strip(strip, stt.current.dir, (t_exit - stt.current.t0) / max(stt.current.t_end - stt.current.t0, _EPS),
                                _e_ext_of(sess.plan, strip, sess.ctx))
        if r is not None:
            sess.register_strips([r], sess.cell_of_strip(stt.current.strip_id))
            lost.append(r.strip_id)
    if not keep_plan:
        # R-10：不要整份刪掉。已飛完的條帶若隨計畫消失，plan.uncovered 不會列出它們，
        # 但 sim.coverage_raster 以剩下的計畫重算足跡，會把那些區域算成未覆蓋 —— 同一次重規劃的兩個輸出互相矛盾
        # （hills UAV_EXIT("A1") @3000 s：13 條已飛條帶憑空消失，Z1 lidar 0.728 / Z2 0.766 FAIL，uncovered 只有 1 筆）。
        # 改為把計畫截到 t_exit：已飛航段（＝已採集的資料）留在紀錄裡，之後才是真的沒飛。
        k = max(0, _index_at(p, t_exit))
        head = list(p.waypoints[:k + 1])
        flown = {sp.strip_id for sp in stt.done} | ({stt.current.strip_id} if stt.current is not None else set())
        sess.plans[u] = replace(
            p, waypoints=head, strips=[(sid, d) for sid, d in p.strips if sid in flown],
            t_land_s=float(head[-1].t_s) if head else float(p.t_takeoff_s),
            t_arr_ptc_s=float(head[-1].t_s) if head else float(p.t_arr_ptc_s),
            energy_used_wh=float(max(0.0, p.energy_used_wh - (head[-1].energy_remaining_wh
                                                             - p.waypoints[-1].energy_remaining_wh))) if head else 0.0,
            distance_m=float(sum(math.dist((a.x_m, a.y_m, a.z_m), (b.x_m, b.y_m, b.z_m))
                                 for a, b in zip(head, head[1:]))),
            warnings=list(p.warnings) + [f"uav_exited_at_t={t_exit:.1f}s"])
        sess.mark_changed(u, k + 1)
        sess.notes.append(f"{tag}({u}): removed at t={t_exit:.1f}s (flown prefix kept), lost={lost}")
    sess.exclude.add(u)
    return lost


def handle_uav_exit(plan: MissionPlan, event: UAVExit, telemetry: Mapping[str, Telemetry] | None, ctx: MissionContext,
                    cache: CostCache, cfg: PlannerConfig, ghosts: Sequence[GhostTrack] | None = None) -> ReplanResult:
    """規格 §12.2：fleet.remove(i)；未完成條帶（含殘段）→ L_uncov；lidar/both 條帶無其他 A → UNCOVERABLE；rgb/any → B 接手
    （殘區重鋪）；同構型接手保留原條帶（受影響集合尾端追加）。"""
    sess, t0 = _begin(plan, event, telemetry, ctx, cache, cfg, ghosts)
    u = event.uav_id
    if u not in sess.plans:
        sess.notes.append(f"uav_exit({u}): not in plan")
        return sess.finish(event, t0)
    lost = _exit_uav(sess, u, sess.t_now, keep_plan=False, tag=f"uav_exit[{event.reason}]")
    remaining = sess.reassign(lost, exclude={u}, reason="uncoverable_after_exit")
    if remaining:
        sess.notes.append(f"uncovered_after_exit({remaining})")
    return sess.finish(event, t0)


def handle_new_area(plan: MissionPlan, event: NewArea, telemetry: Mapping[str, Telemetry] | None, ctx: MissionContext,
                    cache: CostCache, cfg: PlannerConfig, ghosts: Sequence[GhostTrack] | None = None) -> ReplanResult:
    """規格 §12.3：DECOMPOSE 新區 → 新條帶 add_strips → 候選機（可行構型、能量）→ 拍賣插入（尾端追加，≤2 團隊變更）；
    否則 UNCOVERABLE(energy) 待辦。"""
    sess, t0 = _begin(plan, event, telemetry, ctx, cache, cfg, ghosts)
    area = event.area
    dc = ctx.decompose_ctx()
    classes = set(ctx.specs) & set(ctx.inputs.fleet_classes)
    n_uav = {c: len(ctx.inputs.uavs_of_class(c)) for c in classes}
    # PRECHECK (7)(8)(9) 對新區（規格 §12.3）：鏈路／半徑／禁區裁切、高度帶、感測可行 → 可行構型
    poly_enu = None
    try:
        from .context import polygon_enu as _poly_enu
        from .preflight import analyse_areas as _analyse
        poly_enu = _poly_enu(ctx.frame, ctx.inputs.dem, area.polygon_wgs84, area.holes_wgs84, name=area.area_id)
        view = replace(ctx, inputs=replace(ctx.inputs, areas=(area,)), areas_enu={area.area_id: poly_enu})
        per_area, errs, _w = _analyse(view, view.decompose_ctx())
        rec = per_area.get(area.area_id, {})
        feas = set(rec.get("feasible_classes") or [])
        if errs or not rec.get("coverable", True) or not feas:
            reason = ";".join(errs) or str(rec.get("uncoverable_reason") or "no_feasible_class")
            sess.uncovered.append((area.area_id, f"precheck({reason})"))
            sess.notes.append(f"new_area({area.area_id}): precheck failed ({reason})")
            return sess.finish(event, t0)
        classes &= feas
        sess.notes.append(f"new_area({area.area_id}): precheck ok, classes={sorted(classes)}")
    except Exception as exc:  # 前置檢查失敗不阻斷：交由 decompose 判定
        sess.notes.append(f"new_area({area.area_id}): precheck skipped ({exc})")
    sess.lap("precheck")
    try:
        cells = _dec.decompose_area(area, dc, classes, polygon_enu=poly_enu, rho_sym_m=ctx.rho_sym_m, n_uav=n_uav)
    except Exception as exc:
        sess.uncovered.append((area.area_id, f"decompose_failed({exc})"))
        sess.notes.append(f"new_area({area.area_id}): decompose_failed({exc})")
        return sess.finish(event, t0)
    sess.lap("decompose")
    if not cells:
        sess.uncovered.append((area.area_id, "no_cells"))
        return sess.finish(event, t0)
    for cell in cells:
        sess.new_cells.append(cell)
        sess.cell_by_id[cell.cell_id] = cell
        per_cls: dict[str, list[Strip]] = {}
        for c in classes:
            if cell.feasible.get(c) and cell.strips.get(c):
                per_cls[c] = list(cell.strips[c])
        if not per_cls:
            sess.uncovered.append((cell.cell_id, "no_feasible_class"))
            continue
        for c, strips in per_cls.items():
            sess._add_to_cache(c, strips, cell)
        sess.lap("cache")
        # 每構型候選：擇 Δmakespan 最小者（any 胞 → A、B 條帶集擇一）
        ids_all = [s.strip_id for lst in per_cls.values() for s in lst]
        states = {v: (sess.states.get(v) or _interp_state(p, sess.t_now, sess.spec(v), ctx.wind))
                  for v, p in sess.plans.items() if p.waypoints}
        view = replace(sess.plan, plans=dict(sess.plans), strips=dict(sess.strips_all), cells=list(sess.plan.cells) + sess.new_cells)
        cands = affected_set(view, ids_all, states, ctx, cache, cfg)
        if cfg.enforce_time_windows and cell.time_window_s is not None:
            lo, hi = cell.time_window_s
            cands = [v for v in cands if sess.anchor_end(v).state.t_s <= hi]
        done = False
        best_cls: str | None = None
        best_dm = _INF
        for c, strips in per_cls.items():
            cc = [v for v in cands if sess.plans[v].cls == c]
            if not cc:
                continue
            for v in cc:
                anchor = sess.anchor_end(v)
                seq, left = sess.idp_tail(v, anchor, [s.strip_id for s in strips])
                if seq is None or left:
                    continue
                t_land = seq.t_end_s + (seq.rtb.t_s if seq.rtb else 0.0) + _landing_fixed_time_s(sess.spec(v))
                dm = max(t_land, max((p.t_land_s for w, p in sess.plans.items() if w != v), default=0.0)) - sess.makespan()
                if dm < best_dm:
                    best_dm, best_cls = dm, c
        if best_cls is not None:
            rem = sess.reassign([s.strip_id for s in per_cls[best_cls]], candidates=[v for v in cands if sess.plans[v].cls == best_cls],
                                allow_cross_class=False, reason="energy")
            done = not rem
        if not done:
            # 退而求其次：任一構型部分覆蓋
            rem = sess.reassign(ids_all, candidates=cands, allow_cross_class=False, reason="energy")
            if rem:
                sess.notes.append(f"new_area({cell.cell_id}): uncovered {rem}")
    return sess.finish(event, t0)


def _energy_update_core(sess: _Session, u: str, e_tel: float, t_s: float) -> list[str]:
    """規格 §12.4；回傳被切除之條帶（→ L_uncov）。"""
    ctx, cfg = sess.ctx, sess.cfg
    p = sess.plans[u]
    spec = sess.spec(u)
    model = sess.model(u)
    e_start = float(p.waypoints[0].energy_remaining_wh)
    e_pred = _plan_energy_at(p, t_s)
    used_plan = max(e_start - e_pred, _EPS)
    used_act = e_start - float(e_tel)
    delta = (e_pred - float(e_tel)) / max(e_pred, _EPS)
    kappa_old = float(model.kappa)
    if delta > float(cfg.delta_1):
        model.correction = min(1.5, max(0.8, kappa_old * used_act / used_plan))
        sess.notes.append(f"kappa({u}: {kappa_old:.3f} -> {model.kappa:.3f}, delta={delta:.3f})")
    validated = bool(spec.battery_monitor_validated)
    if validated:
        e_ref = min(1.1 * e_pred, max(0.9 * e_pred, float(e_tel)))
    else:
        e_ref = e_start - float(model.kappa) * used_plan
    # 快照能量改用 E_ref
    st = sess.states.get(u)
    if st is not None:
        sess.states[u] = replace(st, e_rem_wh=float(e_ref))
    stt = sess.status(u)
    reserve = float(model.reserve_wh())
    e_nom = float(model.nominal_wh or model.available_wh)
    margin = float(cfg.energy_margin_frac) * e_nom
    # DIVERT 判定
    st_now = sess.states.get(u) or _interp_state(p, t_s, spec, ctx.wind)
    site_near = sess.best_site(u, st_now)
    try:
        e_alt = float(sess.cache.budget.point_wh(p.cls, st_now.xy, float(st_now.z_m), site_near))
    except Exception:
        e_alt = 0.0
    divert = e_ref < reserve + e_alt + margin
    if not validated and delta > float(cfg.delta_2):
        divert = True
        sess.notes.append(f"divert_conservative({u}: delta={delta:.3f} > delta_2)")
    if divert:
        sess.notes.append(f"divert({u}: E_ref={e_ref:.1f} < reserve {reserve:.1f} + e_alt {e_alt:.1f} + margin {margin:.1f})")
        return sess.cut_to_fit(u, e_ref, f"divert(E_ref={e_ref:.1f})", divert=True)
    if stt.landed:
        return []
    need = sess.tail_energy(u, stt, len(stt.pending))
    if e_ref - need < reserve - _EPS:
        cut = sess.cut_to_fit(u, e_ref, f"energy_cut(E_ref={e_ref:.1f})")
        sess.notes.append(f"energy_cut({u}: removed {cut})")
        return cut
    sess.notes.append(f"energy_ok({u}: E_ref={e_ref:.1f}, need={need:.1f}, reserve={reserve:.1f})")
    return []


def handle_energy_update(plan: MissionPlan, event: EnergyUpdate, telemetry: Mapping[str, Telemetry] | None,
                         ctx: MissionContext, cache: CostCache, cfg: PlannerConfig,
                         ghosts: Sequence[GhostTrack] | None = None) -> ReplanResult:
    """規格 §12.4：δ → κ 修正（EnergyModel.correction）；E_ref（未驗證用 E_pred(κ)）；cut 自尾端逐條移除至可行；
    DIVERT（E_ref < r·E_nom + e^alt + margin，或未驗證且 δ > δ_2）；移除者 → L_uncov → 接手。"""
    sess, t0 = _begin(plan, event, telemetry, ctx, cache, cfg, ghosts)
    u = event.uav_id
    if u not in sess.plans or not sess.plans[u].waypoints:
        sess.notes.append(f"energy_update({u}): not in plan")
        return sess.finish(event, t0)
    cut = _energy_update_core(sess, u, float(event.e_tel_wh), sess.t_now)
    if cut:
        rem = sess.reassign(cut, exclude={u}, reason="energy")
        if rem:
            sess.notes.append(f"uncovered_after_cut({rem})")
    return sess.finish(event, t0)


def handle_link_loss(plan: MissionPlan, event: LinkLoss, telemetry: Mapping[str, Telemetry] | None, ctx: MissionContext,
                     cache: CostCache, cfg: PlannerConfig, ghosts: Sequence[GhostTrack] | None = None) -> ReplanResult:
    """規格 §12.5、D26：以預載策略推算幽靈航跡（其他機避讓、降落走廊預約 [ETA−60, ETA+180]）；該機自決策點
    （RTB：t_loss；CONTINUE_THEN_RTB：t_loss + t_ll）之後之條帶 → L_uncov → 接手（幽靈保留走廊獨占）。"""
    sess, t0 = _begin(plan, event, telemetry, ctx, cache, cfg, ghosts)
    u = event.uav_id
    if u not in sess.plans or not sess.plans[u].waypoints:
        sess.notes.append(f"link_loss({u}): not in plan")
        return sess.finish(event, t0)
    p = sess.plans[u]
    policy = ctx.inputs.c2.policy_for(u, p.cls)
    ghost = ghost_track(u, replace(plan, plans=dict(sess.plans)), policy, sess.t_now, ctx, cfg, cache=cache)
    sess.ghosts = [g for g in sess.ghosts if g.uav_id != u] + [ghost]
    sess.notes.append(f"ghost({u}: policy={policy}, site={ghost.landing_site}, eta={ghost.eta_s:.0f} s, sigma_t={ghost.sigma_t:.1f})")
    t_dec = float(ghost.t_rtb_s)
    stt_dec = _status(p, t_dec)
    # 該機計畫：決策點後直接返航（預載動作）
    if stt_dec.landed:
        lost: list[str] = []
    else:
        if stt_dec.current is not None and policy == "CONTINUE_THEN_RTB" and stt_dec.pending:
            anchor = sess.anchor_after_span(u, stt_dec.current)
            lost = [sp.strip_id for sp in stt_dec.pending]
        else:
            sess_state = _interp_state(p, t_dec, sess.spec(u), ctx.wind)
            j = _index_at(p, t_dec)
            stt_now = stt_dec
            kept = [(s.strip_id, s.dir) for s in stt_now.done]
            anchor = _Anchor(j + 1, sess_state, kept)
            lost = [sp.strip_id for sp in stt_now.pending]
            if stt_now.current is not None:
                strip = sess.strip(stt_now.current.strip_id)
                prog = (t_dec - stt_now.current.t0) / max(stt_now.current.t_end - stt_now.current.t0, _EPS)
                r = _truncate_strip(strip, stt_now.current.dir, prog, _e_ext_of(sess.plan, strip, ctx))
                if r is not None:
                    sess.register_strips([r], sess.cell_of_strip(strip.strip_id))
                    lost.append(r.strip_id)
        sess.rtb_now(u, anchor, f"link_loss(policy={policy})", site_id=ghost.landing_site)
    sess.exclude.add(u)
    if lost:
        rem = sess.reassign(lost, exclude={u}, reason="uncoverable_after_link_loss")
        if rem:
            sess.notes.append(f"uncovered_after_link_loss({rem})")
    return sess.finish(event, t0)


def handle_link_restored(plan: MissionPlan, event: LinkRestored, telemetry: Mapping[str, Telemetry] | None,
                         ctx: MissionContext, cache: CostCache, cfg: PlannerConfig,
                         ghosts: Sequence[GhostTrack] | None = None) -> ReplanResult:
    """規格 §12.5 末：恢復 → 幽靈移除、ENERGY_UPDATE（遙測）後重新加入拍賣（承接待辦之未覆蓋條帶）。"""
    tel = dict(telemetry or {})
    tel[event.uav_id] = event.telemetry
    sess, t0 = _begin(plan, event, tel, ctx, cache, cfg, ghosts)
    u = event.uav_id
    sess.ghosts = [g for g in sess.ghosts if g.uav_id != u]
    if u not in sess.plans or not sess.plans[u].waypoints:
        sess.notes.append(f"link_restored({u}): not in plan")
        return sess.finish(event, t0)
    if event.telemetry.e_tel_wh is not None:
        cut = _energy_update_core(sess, u, float(event.telemetry.e_tel_wh), sess.t_now)
        if cut:
            rem = sess.reassign(cut, exclude={u}, reason="energy")
            if rem:
                sess.notes.append(f"uncovered_after_cut({rem})")
    # 重新加入拍賣：待辦條帶（plan.uncovered 中仍存在者）
    todo = [sid for sid, _r in sess.uncovered if sid in sess.strips_all]
    if todo and u in sess.plans and not sess.status(u).landed:
        sess.exclude.discard(u)
        before = set(todo)
        rem = sess.reassign(todo, candidates=[u], reason="energy")
        taken = before - set(rem)
        if taken:
            sess.uncovered = [(s, r) for s, r in sess.uncovered if s not in taken]
            sess.notes.append(f"rejoin({u}: {sorted(taken)})")
    return sess.finish(event, t0)


def handle_wind_update(plan: MissionPlan, event: WindUpdate, telemetry: Mapping[str, Telemetry] | None,
                       ctx: MissionContext, cache: CostCache, cfg: PlannerConfig,
                       ghosts: Sequence[GhostTrack] | None = None) -> ReplanResult:
    """規格 §12.6：新 WindField（version+1）→ `cache.refresh_wind`（時間層；R_eff > 5% 才重建幾何層）→ 逐機
    ENERGY_SAFE_CHECK（失敗 → §12.4 cut）→ max ΔC_i > τ_w → 未完成部分重跑 IDP（保留條帶、現行條帶先完成）；
    任一層 W > W_abort → WEATHER_ABORT；起降風向變化 > 45° → 走廊重算（尾段重建時自動）。"""
    sess, t0 = _begin(plan, event, telemetry, ctx, cache, cfg, ghosts)
    old_wind = ctx.wind
    beta_old = float(old_wind.dir_from_at(float(ctx.z_ground_home_m) + 100.0)) if old_wind is not None and old_wind.n_layers else None
    new_wind = WindField(list(event.layers), ctx.frame, float(ctx.z_ground_home_m),
                         version=int(getattr(old_wind, "version", 0)) + 1)
    # 舊風之尾段時間估計
    est_old: dict[str, float] = {}
    for u, p in sess.plans.items():
        stt = sess.status(u)
        if p.waypoints and not stt.landed:
            est_old[u] = sess.tail_time(u, stt)
    r_old = dict(cache.R_eff)
    ctx.wind = new_wind
    cache.refresh_wind(new_wind, ctx)
    sess.lap("refresh_wind")
    geom = [c for c in cache.R_eff if abs(cache.R_eff[c] - r_old.get(c, cache.R_eff[c])) > 1e-9]
    sess.notes.append(f"wind_refreshed(version={new_wind.version}, geometry_rebuilt={geom})")
    # 氣象中止
    th = ctx.inputs.weather_thresholds
    w_max = max((float(l.speed_mps) for l in event.layers), default=0.0)
    obs_abort = event.obs is not None and bool(th.abort_violations(event.obs))
    if w_max > float(th.abort_mean_wind_mps) or obs_abort:
        sess.notes.append(f"weather_abort_triggered(W_max={w_max:.1f} > {th.abort_mean_wind_mps})")
        _weather_abort_core(sess, "ALL", None, f"wind_update W={w_max:.1f}")
        return sess.finish(event, t0)
    # 走廊風向變化 > 45° → 重算起降走廊（所有在空機之尾段重建：RTB／降落走廊依新風向 β 之 P_tc）
    corridor_changed = False
    if beta_old is not None and new_wind.n_layers:
        beta_new = float(new_wind.dir_from_at(float(ctx.z_ground_home_m) + 100.0))
        d = abs((beta_new - beta_old + math.pi) % _TWO_PI - math.pi)
        if math.degrees(d) > _CORRIDOR_TURN_DEG:
            corridor_changed = True
            sess.notes.append(f"corridor_heading_changed({math.degrees(d):.0f} deg)")
    # 逐機：能量檢查 → cut；ΔC → 重跑
    lost_all: list[str] = []
    tau = float(cfg.tau_w)
    for u in list(sess.plans):
        p = sess.plans[u]
        stt = sess.status(u)
        if not p.waypoints or stt.landed:
            continue
        model = sess.model(u)
        st = sess.states.get(u) or _interp_state(p, sess.t_now, sess.spec(u), ctx.wind)
        e_now = float(st.e_rem_wh)
        need = sess.tail_energy(u, stt, len(stt.pending))
        if cfg.enforce_energy and e_now - need < model.reserve_wh() - _EPS:
            cut = sess.cut_to_fit(u, e_now, "wind_energy_cut")
            sess.notes.append(f"wind_energy_cut({u}: {cut})")
            lost_all += cut
            continue
        est_new = sess.tail_time(u, stt)
        c_old = max(p.t_land_s - sess.t_now, 1.0)
        dC = est_new - est_old.get(u, est_new)
        if (abs(dC) > tau * c_old or corridor_changed) and stt.pending:
            anchor = sess.anchor_after_span(u, stt.current) if stt.current is not None else sess.anchor_now(u)
            seq, left = sess.idp_tail(u, anchor, [sp.strip_id for sp in stt.pending])
            if seq is not None:
                sess.apply_seq(u, anchor, seq, f"wind_replan(dC={dC:.0f} s)")
                sess.notes.append(f"wind_replan({u}: dC={dC:.0f} s > {tau:.0%}·{c_old:.0f}, corridor={corridor_changed})")
                lost_all += left
            else:
                sess.notes.append(f"wind_replan_failed({u})")
        elif corridor_changed:
            anchor = sess.anchor_end(u)
            sess.rtb_now(u, anchor, "wind_corridor_recomputed")
            sess.notes.append(f"wind_corridor_recomputed({u})")
        else:
            sess.notes.append(f"wind_unchanged({u}: dC={dC:.0f} s)")
    if lost_all:
        rem = sess.reassign(lost_all, reason="energy")
        if rem:
            sess.notes.append(f"uncovered_after_wind({rem})")
    return sess.finish(event, t0)


def _octagon(center: np.ndarray, r: float) -> Polygon:
    ang = np.arange(8) * (math.pi / 4.0)
    pts = np.column_stack([center[0] + r * np.cos(ang), center[1] + r * np.sin(ang)])
    return Polygon(pts)


def handle_cue(plan: MissionPlan, event: TargetCue, telemetry: Mapping[str, Telemetry] | None, ctx: MissionContext,
               cache: CostCache, cfg: PlannerConfig, ghosts: Sequence[GhostTrack] | None = None) -> ReplanResult:
    """規格 §12.7 cue-and-confirm：Q_conf = 8 邊形（r_conf）、σ = rgb、時間窗 [t, t + t_conf_max]；
    GSD_req = gsd_confirm；h_max,B(GSD_req) < h_minfw,B → `gsd_unattainable` 且 confirm_mode="orbit"；
    候選 B（Q ⊕ d_turn ⊂ P_link,B、能量可行、Δmakespan ≤ τ_cue）→ 現行條帶後插入確認條帶（2–3 條）或 2 圈盤旋；
    無 B 候選且 A 具 rgb → A 確認；否則待辦。"""
    sess, t0 = _begin(plan, event, telemetry, ctx, cache, cfg, ghosts)
    lat, lon = event.latlon_wgs84
    p_xy = np.asarray(ctx.frame.to_enu_xy(lat, lon), dtype=float).reshape(-1, 2)[0]
    r_conf = float(cfg.r_conf_m)
    poly = _octagon(p_xy, r_conf)
    spec_b = ctx.specs.get("B")
    b_uavs = [u for u, p in sess.plans.items() if p.cls == "B" and p.waypoints and not sess.status(u).landed]
    hw = None
    for u in b_uavs or [x.uav_id for x in ctx.inputs.uavs_of_class("B")]:
        inst = _uav_inst(ctx, u)
        if inst is not None:
            hw = next((h for h in inst.sensors if h.kind == "rgb"), None)
            if hw is not None:
                break
    mode = cfg.confirm_mode
    h_op: float | None = None
    sensor = None
    if spec_b is None or hw is None:
        sess.notes.append("cue: no B rgb hardware")
    else:
        req = PayloadRequirement("cue_confirm", "rgb", gsd_required_m=float(cfg.gsd_confirm_m), side_overlap=0.30,
                                 forward_overlap=0.70, h_target_agl_m=None)
        sensor = sensor_model(hw, req)
        h_max = float(sensor.agl_window()[1])
        h_minfw = float(spec_b.min_fw_operation_agl_m)
        h_ceil = ctx.h_ceil_m.get("B")
        if h_max < h_minfw - 1e-9:
            gsd_att = h_minfw * (hw.pixel_pitch_um * 1e-6) / (hw.focal_length_mm * 1e-3)
            sess.notes.append(f"gsd_unattainable(GSD_req={cfg.gsd_confirm_m:.4f} m, attainable={gsd_att:.4f} m at h_minfw)")
            mode = "orbit"
            h_op = h_minfw
        else:
            h_op = min(h_max, h_ceil) if h_ceil is not None and math.isfinite(h_ceil) else h_max
            h_op = max(h_op, h_minfw)
    area_ring = tuple((float(a), float(b)) for a, b, _h in ctx.frame.to_wgs84(np.column_stack([np.asarray(poly.exterior.coords)[:-1],
                                                                                              np.zeros(8)])))
    prio = max([c.priority for c in plan.cells] + [1.0]) * 2.0
    area = AreaSpec(f"CUE_{event.src_uav_id}_{int(event.t_s)}", area_ring, sensor_req="rgb", priority=prio,
                    time_window_s=(float(event.t_s), float(event.t_s) + float(cfg.t_conf_max_s)), payload_req_id="cue_confirm")
    dc = ctx.decompose_ctx()
    # 候選 B：鏈路 ∧ 能量餘裕
    cands: list[str] = []
    for u in b_uavs:
        d_turn = float(cache.R_eff.get("B", 0.0)) * 2.0
        if cfg.enforce_link:
            lp = ctx.link_poly.get("B")
            if lp is not None and not lp.buffer(1e-6).contains(poly.buffer(d_turn)):
                sess.notes.append(f"cue_candidate_rejected({u}: outside link)")
                continue
        st = sess.states.get(u)
        model = sess.model(u)
        if st is not None:
            e_land = _predicted_landing_e(sess.plans[u], st, model, ctx.specs["B"])
            if cfg.enforce_energy and e_land < model.reserve_wh() + cfg.energy_margin_frac * float(model.nominal_wh or model.available_wh):
                sess.notes.append(f"cue_candidate_rejected({u}: energy)")
                continue
        cands.append(u)
    done = False
    if cands and spec_b is not None and sensor is not None and h_op is not None:
        n_prev = sum(1 for c in list(plan.cells) if str(c.cell_id).startswith(area.area_id + "_"))
        cell = _dec._new_cell(f"{area.area_id}_{n_prev + 1}", area, poly, dc)
        while any(s.startswith(cell.cell_id + "_") for s in cache.idx.get("B", {})):
            n_prev += 1
            cell = _dec._new_cell(f"{area.area_id}_{n_prev + 1}", area, poly, dc)
        cell.h_op_m["B"] = float(h_op)
        cell.feasible["B"] = True
        ctx.sensors[("B", "cue_confirm")] = sensor
        strips: list[Strip] = []
        if mode == "strips":
            w_dir = float(ctx.wind.dir_from_at(float(ctx.z_ground_home_m) + h_op)) if ctx.wind is not None and ctx.wind.n_layers else 0.0
            for alpha in (w_dir, w_dir + math.pi / 2.0):
                try:
                    strips = _dec.build_strips(cell, "B", alpha, dc)
                except Exception as exc:
                    sess.notes.append(f"cue_strips_failed({exc})")
                    strips = []
                if strips and all(s.feasible for s in strips):
                    break
                strips = []
            if strips:
                cell.strips["B"] = strips
                cell.n_strips["B"] = len(strips)
                cell.alpha_rad["B"] = float(strips[0].theta_LR_rad)
                cell.sweep_origin["B"] = np.asarray(strips[0].sweep_origin, dtype=float)
                cell.T_single_s["B"] = _dec.exact_time(strips, "B", dc)
                cell.d_turn_m["B"] = float(cache.R_eff.get("B", 0.0)) * 2.0
                for s in strips:
                    cache.strip_sensors[s.strip_id] = sensor
                sess._add_to_cache("B", strips, cell)
                sess.new_cells.append(cell)
                sess.cell_by_id[cell.cell_id] = cell
                sess.lap("cue_strips")
            else:
                mode = "orbit"
                sess.notes.append("cue: strips infeasible -> orbit")
        best: tuple[float, str, UAVSequence | None, _Anchor, list[PathSegment2D]] | None = None
        ms0 = sess.makespan()
        for u in cands:
            stt = sess.status(u)
            anchor = sess.anchor_after_span(u, stt.current) if stt.current is not None else sess.anchor_now(u)
            pending = [sp.strip_id for sp in stt.pending]
            pre: list[PathSegment2D] = []
            if mode == "strips":
                seq, left = sess.idp_chain(u, anchor, [[s.strip_id for s in strips], pending])
                if seq is None or any(s.strip_id in left for s in strips):
                    continue
                if left:
                    sess.notes.append(f"cue_candidate({u}): pending leftover {left}")
                t_land = seq.t_end_s + (seq.rtb.t_s if seq.rtb else 0.0) + _landing_fixed_time_s(spec_b)
            else:
                pre, exit_state = _orbit_segments(sess, u, anchor, p_xy, float(h_op), _CUE_N_LAPS)
                anchor2 = _Anchor(anchor.keep_n, exit_state, anchor.strips_kept)
                if pending:
                    seq, left = sess.idp_tail(u, anchor2, pending)
                    if seq is None:
                        continue
                    t_land = seq.t_end_s + (seq.rtb.t_s if seq.rtb else 0.0) + _landing_fixed_time_s(spec_b)
                else:
                    seq = None
                    t_land = exit_state.t_s + 600.0
            dm = max(t_land, max((p.t_land_s for w, p in sess.plans.items() if w != u), default=0.0)) - ms0
            if dm > float(cfg.tau_cue) * max(ms0, 1.0) + _EPS:
                sess.notes.append(f"cue_candidate_rejected({u}: dmakespan {dm:.0f} s > tau_cue)")
                continue
            if best is None or dm < best[0]:
                best = (dm, u, seq, anchor, pre)
        if best is not None:
            dm, u, seq, anchor, pre = best
            if mode == "strips" and seq is not None:
                sess.apply_seq(u, anchor, seq, f"cue_confirm(strips={[s.strip_id for s in strips]})")
            elif seq is not None:
                sess.apply_seq(u, anchor, seq, "cue_confirm(orbit)", pre=pre)
            else:
                exit_state = pre[-1].path.end if pre else None
                anchor2 = _Anchor(anchor.keep_n, _orbit_exit_state(sess, u, anchor, pre), anchor.strips_kept)
                sess.rtb_now_with_pre(u, anchor, anchor2, pre, "cue_confirm(orbit)")
            sess.notes.append(f"cue_dispatched({u}: mode={mode}, dmakespan={dm:.0f} s)")
            done = True
    if not done:
        # A 具 rgb 且能量允許 → A 自行降至 max(h_min^A, h_GSD) 確認（記錄）；否則待辦
        a_rgb = [u for u, p in sess.plans.items() if p.cls == "A" and "rgb" in (_uav_inst(ctx, u).available_kinds if _uav_inst(ctx, u) else set())]
        if a_rgb:
            sess.notes.append(f"cue_fallback_A({a_rgb}): GSD limited by 120 m AGL (queued)")
        sess.uncovered.append((area.area_id, "cue_pending"))
        sess.notes.append(f"cue_queued({area.area_id}, priority={prio})")
    return sess.finish(event, t0)


def _orbit_segments(sess: _Session, u: str, anchor: _Anchor, center: np.ndarray, h_op: float, n_laps: int
                    ) -> tuple[list[PathSegment2D], UAVState]:
    """自錨點至以 center 為圓心、R_eff 之盤旋（n 圈、右轉）之 [join, ORBIT] 段與出口狀態。"""
    ctx, cache = sess.ctx, sess.cache
    p = sess.plans[u]
    cls = p.cls
    spec = sess.spec(u)
    va = float(spec.cruise_airspeed_mps)
    R = float(cache.R_eff.get(cls, anchor.state.r_eff_m))
    zg = float(ctx.terrain.z_at(np.asarray(center, dtype=float).reshape(1, 2))[0])
    if not math.isfinite(zg):
        zg = float(ctx.z_ground_home_m)
    z_orb = zg + float(h_op)
    st = anchor.state
    rel = st.xy - center
    ang = math.atan2(rel[0], rel[1])
    th_in = (ang + math.pi / 2.0) % _TWO_PI
    p_in = center - R * _n_hat(th_in)
    entry = Pose(float(p_in[0]), float(p_in[1]), th_in)
    join = _route(_pose(st), entry, R, ctx, cache, min(float(st.z_m), z_orb), va)
    orbit = _orbit_path(entry, np.asarray(center, dtype=float), R, 1, _TWO_PI * n_laps)
    segs = [PathSegment2D(Phase.TRANSFER, Mode.CRUISE, join, None, None, va, False, float(st.z_m), z_orb),
            PathSegment2D(Phase.ORBIT, Mode.CRUISE, orbit, None, None, va, False, z_orb, z_orb, n_laps=n_laps,
                          orbit=(float(center[0]), float(center[1]), R, 1))]
    t_join = path_time(join, va, ctx.wind, float(st.z_m))
    if not math.isfinite(t_join):
        t_join = join.length_m / max(ctx.cfg.v_g_min_mps, 1.0)
    t_lap, _r = orbit_lap_time_s(spec, ctx.wind, z_orb, R)
    model = sess.model(u)
    e = model.segment_wh(Mode.CRUISE, t_join + n_laps * t_lap)
    end = orbit.end
    exit_state = UAVState(end.x_m, end.y_m, end.theta_rad % _TWO_PI, z_orb, st.t_s + t_join + n_laps * t_lap,
                          st.e_rem_wh - e, R, va)
    return segs, exit_state


def _orbit_exit_state(sess: _Session, u: str, anchor: _Anchor, pre: Sequence[PathSegment2D]) -> UAVState:
    if not pre:
        return anchor.state
    last = pre[-1]
    end = last.path.end
    return replace(anchor.state, x_m=end.x_m, y_m=end.y_m, theta_rad=end.theta_rad % _TWO_PI, z_m=float(last.z_hint_end_m))


def _rtb_now_with_pre(self: _Session, u: str, anchor: _Anchor, anchor2: _Anchor, pre: Sequence[PathSegment2D], tag: str) -> UAVPlan:
    cache, ctx = self.cache, self.ctx
    p = self.plans[u]
    cls = p.cls
    inst = self.inst(u)
    st = anchor2.state
    site = self.best_site(u, st)
    app, ld = _site_app(ctx, cache, inst, site, _pose(st), float(st.z_m))
    segs = _tail_segments(cls, anchor.state, [], None, None, ld, cache, ctx, pre=pre)
    new, _tail = _finish_plan(p, inst, anchor.keep_n, anchor.state, segs, site, ctx, self.strips_all, [], tag,
                              keep_strips=anchor.strips_kept)
    self.plans[u] = new
    self.mark_changed(u, anchor.keep_n)
    return new


_Session.rtb_now_with_pre = _rtb_now_with_pre  # type: ignore[attr-defined]


def handle_priority(plan: MissionPlan, event: PriorityChange, telemetry: Mapping[str, Telemetry] | None,
                    ctx: MissionContext, cache: CostCache, cfg: PlannerConfig,
                    ghosts: Sequence[GhostTrack] | None = None) -> ReplanResult:
    """規格 §12.8：val_k ← val'（胞就地更新）；未開始之胞允許搶占（提高 → 該區胞優先，重跑該機未完成部分，能量可行）；
    進行中且降低 → 不中斷；待辦（uncovered）中該區條帶 → 重新嘗試接手。"""
    sess, t0 = _begin(plan, event, telemetry, ctx, cache, cfg, ghosts)
    aid = event.area_id
    cells = [c for c in sess.plan.cells if c.area_id == aid]
    if not cells:
        sess.notes.append(f"priority({aid}): area not in plan")
        return sess.finish(event, t0)
    old = float(cells[0].priority)
    for c in cells:
        c.priority = float(event.priority)
    sess.notes.append(f"priority({aid}: {old} -> {event.priority})")
    cids = {c.cell_id for c in cells}
    if float(event.priority) > old:
        for u in list(sess.plans):
            stt = sess.status(u)
            if stt.landed or not stt.pending:
                continue
            pend_cells = [str(sess.strip(sp.strip_id).cell_id) for sp in stt.pending]
            if not any(cc in cids for cc in pend_cells) or pend_cells[0] in cids:
                continue                                       # 無該區待掃、或已排在最前
            anchor = sess.anchor_after_span(u, stt.current) if stt.current is not None else sess.anchor_now(u)
            first = [sp.strip_id for sp in stt.pending if str(sess.strip(sp.strip_id).cell_id) in cids]
            rest = [sp.strip_id for sp in stt.pending if str(sess.strip(sp.strip_id).cell_id) not in cids]
            seq, left = sess.idp_chain(u, anchor, [first, rest])
            if seq is None or left:
                sess.notes.append(f"priority_preempt_rejected({u}: energy/leftover)")
                continue
            sess.apply_seq(u, anchor, seq, f"priority_preempt({aid})")
            sess.notes.append(f"priority_preempt({u}: {[it.strip_id for it in seq.items]})")
        todo = [sid for sid, _r in sess.uncovered if sid in sess.strips_all and str(sess.strip(sid).cell_id) in cids]
        if todo:
            rem = sess.reassign(todo, reason="energy")
            taken = set(todo) - set(rem)
            if taken:
                sess.uncovered = [(s, r) for s, r in sess.uncovered if s not in taken]
    else:
        sess.notes.append("priority_lowered: no interruption")
    return sess.finish(event, t0)


def _site_limits(ctx: MissionContext, spec: PlatformSpec, site_id: str, obs: WeatherObs | None
                 ) -> tuple[bool, float]:
    """(OK, 超限量)：側風／陣風 ≤ 轉換限（規格 §12.9）；逆風進場 → 側風 = W·|sin(β − θ_app)|。"""
    xy, z_site = ctx.sites_enu[site_id]
    z_ref = float(z_site) + spec.transition_cmd().h_tc_clear_m
    W = float(ctx.wind.speed_at(z_ref)) if ctx.wind is not None else 0.0
    gust = float(ctx.wind.gust_at(z_ref)) if ctx.wind is not None else 0.0
    if obs is not None:
        W = max(W, float(obs.mean_wind_mps))
        gust = max(gust, float(obs.gust_mps))
    site = ctx.inputs.sites.get(site_id)
    beta = float(ctx.wind.dir_from_at(z_ref)) if ctx.wind is not None and ctx.wind.n_layers else 0.0
    theta_app = math.radians(float(site.approach_heading_deg)) if site is not None and site.approach_heading_deg is not None else beta
    cross = W * abs(math.sin(beta - theta_app))
    k = 1 if spec.airspeed_sensor_validated else 0           # D8：(unvalidated, validated)
    x_max = float(np.atleast_1d(spec.crosswind_site_max_mps)[min(k, np.size(spec.crosswind_site_max_mps) - 1)])
    g_max = float(np.atleast_1d(spec.gust_site_max_mps)[min(k, np.size(spec.gust_site_max_mps) - 1)])
    over = max(0.0, cross - x_max) + max(0.0, gust - g_max)
    return over <= 1e-9, over


def _weather_abort_core(sess: _Session, scope: str, region_wgs84: Sequence[tuple[float, float]] | None, reason: str,
                        obs: WeatherObs | None = None) -> None:
    ctx, cfg, cache = sess.ctx, sess.cfg, sess.cache
    region: Polygon | None = None
    if scope == "region" and region_wgs84:
        pts = np.asarray([[a, b] for a, b in region_wgs84], dtype=float)
        region = Polygon(np.asarray(ctx.frame.to_enu_xy(pts[:, 0], pts[:, 1]), dtype=float).reshape(-1, 2))
    affected: list[str] = []
    for u, p in sess.plans.items():
        if not p.waypoints:
            continue
        stt = sess.status(u)
        if stt.landed:
            continue
        if scope in ("A", "B") and p.cls != scope:
            continue
        if region is not None:
            st = sess.states.get(u) or _interp_state(p, sess.t_now, sess.spec(u), ctx.wind)
            inside = region.contains(Point(st.x_m, st.y_m))
            for sp in stt.pending + ([stt.current] if stt.current else []):
                s = sess.strip(sp.strip_id)
                inside |= region.intersects(shapely.LineString([s.pL, s.pR]))
            if not inside:
                continue
        affected.append(u)
    if not affected:
        sess.notes.append(f"weather_abort({reason}): no airborne UAV affected")
        return
    # 每機：完成現行條帶（< 60 s 且能量允許）或立即中止；站點 OK 集合
    entries: list[tuple[float, str, _Anchor, str, float]] = []      # (margin_s, uav, anchor, site, over)
    for u in affected:
        p = sess.plans[u]
        spec = sess.spec(u)
        model = sess.model(u)
        stt = sess.status(u)
        lost = [sp.strip_id for sp in stt.pending]
        finish_cur = False
        if stt.current is not None:
            t_rem = stt.current.t_end - sess.t_now
            if t_rem < _FINISH_STRIP_S:
                need = sess.tail_energy(u, stt, 0)
                st_e = sess.states.get(u)
                e_now = float(st_e.e_rem_wh) if st_e is not None else _plan_energy_at(p, sess.t_now)
                finish_cur = e_now - need >= model.reserve_wh() - _EPS
        if finish_cur and stt.current is not None:
            anchor = sess.anchor_after_span(u, stt.current)
        else:
            anchor = sess.anchor_now(u)
            if stt.current is not None:
                r = sess.residual.get(stt.current.strip_id)
                if r is not None:
                    sess.register_strips([r], sess.cell_of_strip(stt.current.strip_id))
                    lost.append(r.strip_id)
                anchor.strips_kept = [k for k in anchor.strips_kept if k[0] != stt.current.strip_id]
        for s in lost:
            sess.uncovered.append((s, f"weather_abort({reason})"))
        sites = list(cache.budget.site_ids) if cache.budget.site_ids else list(ctx.sites_enu)
        ok = [s for s in sites if _site_limits(ctx, spec, s, obs)[0]]
        st = anchor.state
        if ok:
            # argmin t(S_i → a)
            best_s, best_t = None, _INF
            for s in ok:
                d = float(np.hypot(*(st.xy - ctx.sites_enu[s][0])))
                t_est = d / max(spec.cruise_airspeed_mps - (ctx.wind.max_speed() if ctx.wind else 0.0), cfg.v_g_min_mps)
                if t_est < best_t:
                    best_s, best_t = s, t_est
            site, over = best_s, 0.0
        else:
            site, over = min(((s, _site_limits(ctx, spec, s, obs)[1]) for s in sites), key=lambda kv: kv[1])
        try:
            e_alt = float(cache.budget.point_wh(p.cls, st.xy, float(st.z_m), site))
        except Exception:
            e_alt = 0.0
        margin_s = (float(st.e_rem_wh) - model.reserve_wh() - e_alt) / max(model.p_cruise_w(), 1.0) * 3600.0
        entries.append((margin_s, u, anchor, site or ctx.recovery_site_id, over))
    entries.sort(key=lambda e: (e[0], e[1]))
    # 降落排序：同站依能量時間餘裕升序；間隔 t_sep,LDG；後續機上風側 orbit（能量檢查）
    sched = CorridorSchedule(cfg)
    for u, p in sess.plans.items():
        if u not in affected and p.waypoints:
            sched.reserve_plan(p, ctx.inputs.home.site_id)
    for g in sess.ghosts:
        sched.reserve_ghost(g)
    for margin_s, u, anchor, site, over in entries:
        new = sess.rtb_now(u, anchor, f"weather_abort({reason})", site_id=site)
        if over > 0:
            new.warnings.append(f"landing_over_limit({site}: +{over:.1f} m/s)")
            sess.notes.append(f"landing_over_limit({u} -> {site}, +{over:.1f} m/s)")
        t_arr = float(new.t_arr_ptc_s)
        dur = (new.t_land_s + cfg.t_sep_ldg_s) - (t_arr - 30.0)
        t_free = sched.next_free(site, "landing", t_arr - 30.0, dur, exclude_uav=u)
        delay = t_free - (t_arr - 30.0)
        if delay > 1e-6:
            spec = sess.spec(u)
            model = sess.model(u)
            idx = anchor.keep_n
            z = float(new.waypoints[idx].z_m) if idx < len(new.waypoints) else float(anchor.state.z_m)
            t_lap, r_orb = orbit_lap_time_s(spec, ctx.wind, z, cache.R_eff.get(new.cls))
            n = int(math.ceil(delay / max(t_lap, 1.0)))
            e_orb = model.segment_wh(Mode.CRUISE, n * t_lap)
            if not cfg.enforce_energy or new.energy_margin_wh - e_orb >= -_EPS:
                new = insert_delay_orbit(new, idx, n, t_lap, model, r_orb)
                sess.plans[u] = new
                sess.notes.append(f"landing_wait({u}: {n} laps at {site})")
            else:
                new.warnings.append(f"landing_sequence_energy_override({site})")
                sess.notes.append(f"landing_queue_jump({u}: energy for {n} laps insufficient)")
        sched.reserve_plan(sess.plans[u], ctx.inputs.home.site_id)
    sess.notes.append(f"weather_abort({reason}): landing order {[e[1] for e in entries]}")


def handle_weather_abort(plan: MissionPlan, event: WeatherAbort, telemetry: Mapping[str, Telemetry] | None,
                         ctx: MissionContext, cache: CostCache, cfg: PlannerConfig,
                         ghosts: Sequence[GhostTrack] | None = None) -> ReplanResult:
    """規格 §12.9、D23：受影響機（scope）完成現行條帶（< 60 s 且能量允許）或立即中止；降落點 = OK 集合中最近者，
    OK = ∅ → 超限量最小者 + `landing_over_limit` 警告；同站依能量時間餘裕升序排程（t_sep,LDG），後續機 orbit 等待
    （能量不足 → 插隊／記錄）；未覆蓋 → 待辦。"""
    sess, t0 = _begin(plan, event, telemetry, ctx, cache, cfg, ghosts)
    _weather_abort_core(sess, event.scope, event.region_wgs84, event.reason)
    return sess.finish(event, t0)


# ---------------------------------------------------------------------------
# 分派
# ---------------------------------------------------------------------------
_HANDLERS = {
    UAVExit: handle_uav_exit, NewArea: handle_new_area, EnergyUpdate: handle_energy_update, LinkLoss: handle_link_loss,
    LinkRestored: handle_link_restored, WindUpdate: handle_wind_update, TargetCue: handle_cue,
    PriorityChange: handle_priority, WeatherAbort: handle_weather_abort,
}


def replan(plan: MissionPlan, event: Event, telemetry: dict[str, Telemetry], ctx: MissionContext, cache: CostCache,
           cfg: PlannerConfig, ghosts: list[GhostTrack] | None = None) -> ReplanResult:
    """規格 §12.1 REPLAN：依事件型別分派至 handle_*。未受影響機之 UAVPlan 為同一物件；受影響機既有航點同一物件、
    只追加尾段；`diff` 只含變更機自錨點後之航點；總耗時 > 2 s → `placeholder_used`（附佔位航點）。"""
    h = _HANDLERS.get(type(event))
    if h is None:
        raise TypeError(f"unknown event type {type(event).__name__}")
    return h(plan, event, telemetry, ctx, cache, cfg, ghosts)


# ---------------------------------------------------------------------------
# 快取重建（plan_mission 不回傳 CostCache；供整合／測試）
# ---------------------------------------------------------------------------
def cache_from_plan(plan: MissionPlan, ctx: MissionContext, *, site_ids: Sequence[str] | None = None) -> CostCache:
    """自 MissionPlan（strips、plans）重建與 plan_mission 同構之 CostCache：init_states = 各機首航點外之起飛出口
    （首 ENTRY/CLIMB 航點）、sites = 回收點（+ 指定備降點）之 (S^app, z_tc)。"""
    from .decompose import resolve_sensor as _rs
    strips: dict[str, dict[str, Strip]] = {c: {} for c in ctx.specs}
    for sid, s in plan.strips.items():
        strips.setdefault(s.cls, {})[sid] = s
    strips = {c: d for c, d in strips.items() if d}
    dc = ctx.decompose_ctx()
    by_id, by_strip = _cell_maps(plan)
    strip_sensors: dict[str, Any] = {}
    for sid, s in plan.strips.items():
        c = by_strip.get(sid)
        found = _rs(dc, s.cls, c.payload_req_id if c is not None else "default")
        if found is not None:
            strip_sensors[sid] = found[0]
    fleet = list(ctx.inputs.fleet)
    init_states: dict[str, UAVState] = {}
    init_laps: dict[str, tuple[int, float]] = {}
    for u, p in plan.plans.items():
        if not p.waypoints:
            continue
        spec = ctx.specs[p.cls]
        w = next((w for w in p.waypoints if w.phase in (Phase.ENTRY, Phase.CLIMB)), p.waypoints[0])
        wind_w = ctx.wind.speed_at(float(w.z_m)) if ctx.wind is not None else 0.0
        init_states[u] = UAVState(w.x_m, w.y_m, math.radians(w.track_deg) % _TWO_PI, float(w.z_m), float(w.t_s),
                                  float(w.energy_remaining_wh), float(spec.r_eff(wind_w)), float(spec.cruise_airspeed_mps))
        init_laps[u] = (0, 0.0)
    ids = list(site_ids) if site_ids is not None else [ctx.recovery_site_id]
    sites: dict[str, dict[str, tuple[Pose, float]]] = {}
    for sid in ids:
        sites[sid] = {}
        for c in ctx.specs:
            cl = list(strips.get(c, {}).values())
            us = [u for u in fleet if u.cls == c]
            if not cl or not us:
                continue
            ld = landing_corridor(sid, us[0], ctx, cl[0].end_state(-1), float(cl[0].z_op_m))
            if ld.app_state is None:
                continue
            sites[sid][c] = (ld.app_state, _z_tc_of(ctx, c, sid))
    sites = {k: v for k, v in sites.items() if v}
    return CostCache.build(strips, fleet, init_states, init_laps, sites, ctx, strip_sensors=strip_sensors)
