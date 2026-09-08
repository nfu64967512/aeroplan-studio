"""hv_dccpp/planner.py — 異構 Beam-IDP、返航附加、能量修復、2D 組裝、總流程（契約 §10；規格 §10、§9.4–9.7、§11.4–11.7、
§12.1、§15；D28、D32、D33、D34、D40）。

* 第 4 層最上端：依賴 preflight / context / decompose / costs / allocate / altitude / deconflict（不得被其下層 import）。
* 成本一律為時間（s）；階段成本 W = makespan 下界（含 t̂_RTB）+ ε 總和 + μ 論文相對項（規格 §10.2；`metric="paper"` 時只保留
  論文相對項）。
* `IDPNode` 以 (λ,) numpy 陣列存狀態；`stage_cost` / `energy_prune` / `turn_band_conflict` 對候選矩陣 (M,) 一次向量化
  （numpy fancy-index 讀 `cache.tr/op/entry/budget`），熱路徑無逐候選 Python 迴圈。
* 事件驅動分支（D33，預設）：每階段只擴展 T_i 最小之機（tie → uav_id），候選 = NAD 條帶 × 相容 2 型轉場，K-nearest（含胞完成偏好）、
  beam B、鍵去重、逐機 RTB（規格 §10.3）。鎖步分支（`idp_branching="lockstep"`，論文 Algorithm 2 結構）：每階段每機各取一條、
  Π = ∏ near_i（條帶互異）。
* 進入段由 `cache.entry`（第 0 列）計入，IDP 初始狀態 S_i = 起飛走廊出口（`takeoff_corridor(...).exit_state`）。
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field, replace
from typing import Any, Hashable, Iterable, Mapping, Sequence

import numpy as np

from .allocate import AllocationResult, Team, allocate
from .altitude import CorridorResult, PathSegment2D, build_3d, landing_corridor, layering, takeoff_corridor
from .context import MissionContext, build_context
from .costs import CostCache, PathCost
from .deconflict import CorridorSchedule, check_separation, repair, spatially_exclusive
from .decompose import altitude_band, build_strips, decompose_area, exact_time, residual_candidates, resolve_sensor
from .dubins import DubinsPath, Pose, dubins_path
from .geometry import Cell, Strip
from .mission import (FeasibilityReport, MissionInputs, MissionPlan, Mode, Phase, PlannerConfig, UAVInstance, UAVPlan,
                      UAVState, Waypoint)
from .platform import EnergyModel, PlatformSpec
from .preflight import preflight

__all__ = [
    "SeqItem", "UAVSequence", "IDPResult", "IDPNode", "EnergyInfeasible", "PathSegment2D",
    "stage_cost", "energy_prune", "turn_band_conflict", "beam_idp", "append_rtb", "repair_energy",
    "energy_safe_check", "assemble_2d", "build_uav_plan", "plan_mission",
]

_INF = math.inf
_EPS = 1e-9
_RTB_CHECK_DS_M = 200.0            # ENERGY_SAFE_CHECK 沿返航路徑之取樣間距（上界化 point_wh 為 O(1)，粗取樣即足）
_RESEQ_MAX_CALLS = 12              # §13.6 (iii) IDP 重排回呼：單次 plan_mission 之最大呼叫次數
_RESEQ_MAX_CANDIDATES = 4          # 每次呼叫之「強制首選」候選數（每個候選一次 beam_idp）
_RESEQ_TIME_BUDGET_S = 8.0         # 重排回呼之總時間預算（秒）


# ---------------------------------------------------------------------------
# 契約資料型別
# ---------------------------------------------------------------------------
@dataclass(slots=True)
class SeqItem:
    """序列一項：條帶、掃向、作業起訖時刻、作業末剩餘能量、前一轉移成本（首項 None；進入見 `UAVSequence.entry`）。"""
    strip_id: str
    dir: int
    t_start_s: float
    t_end_s: float
    e_end_wh: float
    transfer: PathCost | None


@dataclass(slots=True)
class UAVSequence:
    """單機序列（論文 Eq.22 R_i 結構）。`decisions` = 快取條帶索引序列（cache.idx[cls]）；`t_end_s/e_end_wh` = 末條帶作業末
    （不含 RTB）；`state_end` = 末條帶 `end_state(dir)`（RTB 起點）。尾端額外欄位：`warnings`（planned_landing_at_alternate 等）、
    `cls`。"""
    uav_id: str
    items: list[SeqItem]
    entry: PathCost
    rtb: PathCost | None
    landing_site: str | None
    state_end: UAVState
    t_end_s: float
    e_end_wh: float
    decisions: list[int]
    warnings: list[str] = field(default_factory=list)
    cls: str = ""


@dataclass(slots=True)
class IDPResult:
    """Beam-IDP 結果。`makespan_s` = max_i [t_end + t_RTB(真實 Dubins，回收點) + 降落固定時間]；尾端額外欄位供
    `repair_energy` 重跑：`init_states`、`cell_order`、`site_id`、`team`。"""
    sequences: dict[str, UAVSequence]
    must_rtb: list[str]
    leftover: list[str]
    makespan_s: float
    nodes_expanded: int
    degraded_B: bool
    init_states: dict[str, UAVState] = field(default_factory=dict)
    cell_order: dict[str, list[str]] = field(default_factory=dict)
    site_id: str | None = None
    team: Team | None = None


@dataclass(slots=True)
class EnergyInfeasible:
    """ENERGY_SAFE_CHECK 失敗點：`seg_index` = 含 x_j 之 SeqItem 索引（= len(items) → 返航段、len(items)+1 → 落地餘量）。"""
    uav_id: str
    seg_index: int
    sample_index: int
    strip_id: str | None
    deficit_wh: float


@dataclass(slots=True)
class IDPNode:
    """事件驅動 Beam-IDP 節點（規格 §10.4）；契約欄位皆為 (λ,) 陣列。尾端額外欄位（皆有預設）：`prev_idx`（前一條帶）、
    `t_busy_from`（最後一次轉移起始時刻）、`t_op_from` / `t_op_end`（最後一條帶作業起訖）、`dec_*`（該決策之轉移／作業時間與
    能量，供序列重建）、`depth`。"""
    uav_ids: tuple[str, ...]
    last_idx: np.ndarray
    last_dir: np.ndarray
    T: np.ndarray
    E: np.ndarray
    active: np.ndarray
    nad_mask: np.ndarray
    W: float
    parent: "IDPNode | None"
    decision: tuple[int, int, int] | None
    prev_idx: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=int))
    prev_dir: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=int))
    t_busy_from: np.ndarray = field(default_factory=lambda: np.zeros(0))
    t_op_from: np.ndarray = field(default_factory=lambda: np.zeros(0))
    t_op_end: np.ndarray = field(default_factory=lambda: np.zeros(0))
    dec_t_tr: float = 0.0
    dec_t_op: float = 0.0
    dec_e_tr: float = 0.0
    dec_e_op: float = 0.0
    depth: int = 0

    def key(self) -> Hashable:
        """去重鍵（規格 §10.4）：(NAD, ((strip_i, dir_i) for i))。"""
        return (self.nad_mask.tobytes(), tuple(zip(self.last_idx.tolist(), self.last_dir.tolist())))

    @property
    def n_uav(self) -> int:
        return len(self.uav_ids)

    def finished(self) -> bool:
        return (not bool(self.nad_mask.any())) or (not bool(self.active.any()))


# ---------------------------------------------------------------------------
# 快取層小工具（記憶化以 cache 鍵）
# ---------------------------------------------------------------------------
_MEMO: dict[tuple, Any] = {}


def _memo_key(cache: CostCache, *extra: Any) -> tuple:
    return (id(cache), int(cache.geometry_key), int(cache.wind_key), int(cache.state_key)) + tuple(extra)


def _recovery_site(cache: CostCache, cls: str, site_id: str | None = None) -> str | None:
    """RTB 站點：指定者優先（D40 回收點），否則 cache.sites 之第一個有該構型列者。"""
    if site_id is not None and (cls, site_id) in cache.rtb:
        return site_id
    for s in cache.sites:
        if (cls, s) in cache.rtb:
            return s
    for (c, s) in cache.rtb:
        if c == cls:
            return s
    return None


def _landing_fixed_time_s(spec: PlatformSpec) -> float:
    """降落固定時間 t_acc + t_tr^in + t_dn + t_hov,res（A 117 s、B 106 s；規格 §5.3）。"""
    tc = spec.transition_cmd()
    return float(tc.t_acc_s + spec.t_tr_in_s + spec.hover_descent_time_s() + spec.t_hover_reserve_s)


def _spec_of(cache: CostCache, cls: str) -> PlatformSpec:
    specs = getattr(cache.budget, "specs", None) or {}
    return specs[cls]


def _rtb_hat(cache: CostCache, cls: str, cfg: PlannerConfig, site_id: str | None) -> np.ndarray:
    """t̂_RTB (n,2)（規格 §10.2）：`cfg.t_rtb_lower_bound` → 回收點之真實快取 RTB 時間（AVOID + 螺旋；有限者），無限者以直線
    順風下界 |端點 − S^app|/(V_a + W_R) 取代（可採納）；旗標關閉（legacy）→ 0。"""
    site = _recovery_site(cache, cls, site_id)
    key = _memo_key(cache, "rtb_hat", cls, site, bool(cfg.t_rtb_lower_bound))
    out = _MEMO.get(key)
    if out is not None:
        return out
    n = cache.n_strips(cls)
    if not cfg.t_rtb_lower_bound or site is None or n == 0:
        out = np.zeros((n, 2))
    else:
        out = np.array(cache.rtb[(cls, site)], dtype=float)
        bad = ~np.isfinite(out)
        if bad.any():
            spec = _spec_of(cache, cls)
            from .costs import resolve_site_entry
            s_app, _z = resolve_site_entry(cache.sites, site, cls)
            E = cache.budget.endpoints(cls) if cache.budget.strip_ids.get(cls) == cache.strip_ids[cls] else None
            if E is None:
                E = np.zeros((n, 2, 2))
                for i, s in enumerate(cache._class_strips(cls)):
                    E[i, 0], E[i, 1] = s.pR, s.pL
            d = np.hypot(E[:, :, 0] - s_app.x_m, E[:, :, 1] - s_app.y_m)
            v = spec.cruise_airspeed_mps + float(cache.W_R.get(cls, 0.0))
            out = np.where(bad, d / max(v, 1.0), out)
    _MEMO[key] = out
    return out


def _budget_min(cache: CostCache, cls: str) -> np.ndarray:
    """budget.min_rtb_wh 之 (n,2) 向量版（快取索引序）。"""
    key = _memo_key(cache, "budget_min", cls)
    out = _MEMO.get(key)
    if out is not None:
        return out
    n = cache.n_strips(cls)
    out = np.zeros((n, 2))
    bw = cache.budget.wh.get(cls)
    if bw is not None and bw.shape[0] and bw.shape[2]:
        bidx = np.array([cache.budget.idx[cls].get(s, -1) for s in cache.strip_ids[cls]], dtype=int)
        ok = bidx >= 0
        vals = np.full((n, 2), np.inf)
        vals[ok] = bw[bidx[ok]].min(axis=2)
        out = vals
    _MEMO[key] = out
    return out


class _StripGeom:
    """迴轉帶剪枝用之條帶幾何表（快取索引序）：胞代碼、側向座標 v（沿 n̂(θ_LR)）、寬度、間距。"""
    __slots__ = ("cell", "v", "w", "s", "theta")

    def __init__(self, cache: CostCache, cls: str, strips: Mapping[str, Strip] | None) -> None:
        S = cache._class_strips(cls)
        codes: dict[str, int] = {}
        self.cell = np.zeros(len(S), dtype=int)
        self.v = np.zeros(len(S))
        self.w = np.zeros(len(S))
        self.s = np.zeros(len(S))
        self.theta = np.zeros(len(S))
        for i, st in enumerate(S):
            st = strips.get(st.strip_id, st) if strips else st
            self.cell[i] = codes.setdefault(str(st.cell_id), len(codes))
            th = float(st.theta_LR_rad)
            mid = 0.5 * (np.asarray(st.pL, dtype=float) + np.asarray(st.pR, dtype=float))
            self.v[i] = mid[0] * math.cos(th) - mid[1] * math.sin(th)
            self.w[i] = float(st.width_m)
            self.s[i] = float(st.spacing_m)
            self.theta[i] = th


def _strip_geom(cache: CostCache, cls: str, strips: Mapping[str, Strip] | None) -> _StripGeom:
    key = _memo_key(cache, "geom", cls)
    out = _MEMO.get(key)
    if out is None:
        out = _StripGeom(cache, cls, strips)
        _MEMO[key] = out
    return out


def _cand_times(node: IDPNode, i_star: int, m: np.ndarray, k: np.ndarray, cache: CostCache
                ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """候選之 (t_trans, t_oper, e_trans, e_oper)（向量化）。首階段用 entry 列。"""
    uav = node.uav_ids[i_star]
    cls = cache.uav_cls[uav]
    idx_m = k % 2
    l = int(node.last_idx[i_star])
    if l < 0:
        ent = cache.entry.get(uav)
        if ent is None or ent.shape[0] == 0:
            t_tr = np.full(m.size, np.inf)
            e_tr = np.full(m.size, np.inf)
        else:
            t_tr = ent[m, idx_m]
            e_tr = cache.entry_e[uav][m, idx_m]
    else:
        t_tr = cache.tr[cls][l, m, k]
        e_tr = cache.tr_e[cls][l, m, k]
    t_op = cache.op[cls][m, idx_m]
    e_op = cache.op_e[cls][m, idx_m]
    return np.asarray(t_tr, float), np.asarray(t_op, float), np.asarray(e_tr, float), np.asarray(e_op, float)


# ---------------------------------------------------------------------------
# 階段成本、能量剪枝、迴轉帶剪枝（契約 §10；規格 §10.2–10.3）
# ---------------------------------------------------------------------------
def stage_cost(node: IDPNode, i_star: int, cand_strip: np.ndarray, cand_k: np.ndarray, cache: CostCache,
               cfg: PlannerConfig, *, site_id: str | None = None) -> np.ndarray:
    """規格 §10.2 階段成本（向量化，M 候選一次）：

    W = max_j [T_j + 1[j=i*](t^trans + t^oper) + t̂^RTB_j] + ε Σ_j [⋯] + μ (t^trans + t^oper)/(T_i* + τ0)；
    `metric="paper"` → μ = 1、前兩項 0（論文 Eq.33 相對增量）。首階段（last_idx = −1）用 entry[i*][m, idx]；
    t̂^RTB 見 :func:`_rtb_hat`（回收點真實 RTB 或直線下界；legacy 0）。不可行候選 → inf。"""
    m = np.asarray(cand_strip, dtype=int).reshape(-1)
    k = np.asarray(cand_k, dtype=int).reshape(-1)
    uav = node.uav_ids[i_star]
    cls = cache.uav_cls[uav]
    idx_m = k % 2
    t_tr, t_op, _e1, _e2 = _cand_times(node, i_star, m, k, cache)
    t_add = t_tr + t_op
    T_i = float(node.T[i_star])
    tau0 = float(cfg.tau0_s)
    rel = t_add / (T_i + tau0)
    if cfg.metric == "paper":
        W = rel
    else:
        rh = _rtb_hat(cache, cls, cfg, site_id)
        # 未完成前 t̂_RTB 取「現端點與剩餘 NAD 任一端點」之最小值（側向中性之下界；否則每階段都偏好近端而選出 racetrack），
        # 最後一條（取 m 後 NAD 空）才用該端點之真值
        floor1, floor2, arg1 = _nad_rtb_floor(node, rh)
        entered = node.last_idx >= 0
        li = np.where(entered, node.last_idx, 0)
        di = (node.last_dir + 1) // 2
        rh_j = np.where(entered & node.active, np.minimum(rh[li, di], floor1), 0.0)
        base = node.T + rh_j
        mask = np.ones(node.n_uav, dtype=bool)
        mask[i_star] = False
        others = base[mask]
        max_other = float(others.max()) if others.size else -np.inf
        sum_other = float(others.sum()) if others.size else 0.0
        rest_floor = np.where(m == arg1, floor2, floor1)          # NAD \ {m} 之最小 RTB 下界（空 → inf）
        rh_i = np.minimum(rh[m, idx_m], rest_floor)
        fin_i = T_i + t_add + rh_i
        W = np.maximum(max_other, fin_i) + float(cfg.eps_sum) * (sum_other + fin_i) + float(cfg.mu_paper) * rel
    return np.where(np.isfinite(t_add), W, np.inf)


def _nad_rtb_floor(node: IDPNode, rh: np.ndarray) -> tuple[float, float, int]:
    """(NAD 端點 RTB 下界最小值, 次小值（不同條帶）, 最小值所在條帶)；NAD 空 → (inf, inf, −1)。"""
    nad = np.flatnonzero(node.nad_mask)
    if nad.size == 0:
        return _INF, _INF, -1
    rmin = rh[nad].min(axis=1)
    o = np.argsort(rmin, kind="stable")
    f1 = float(rmin[o[0]])
    f2 = float(rmin[o[1]]) if nad.size > 1 else _INF
    return f1, f2, int(nad[o[0]])


def energy_prune(node: IDPNode, i_star: int, cand_strip: np.ndarray, cand_k: np.ndarray, cache: CostCache,
                 model: EnergyModel | None, *, enforce: bool = True) -> np.ndarray:
    """規格 §10.3 能量剪枝（(M,) bool，True = 剪掉）：E^rem − e^trans − e^oper < r·E_nom + budget.min_rtb(c, m, idx_m)
    或 `static_ok[m, idx_m]` False；`enforce=False`（cfg.enforce_energy 關閉）只保留 static_ok（legacy 全 True）。"""
    m = np.asarray(cand_strip, dtype=int).reshape(-1)
    k = np.asarray(cand_k, dtype=int).reshape(-1)
    uav = node.uav_ids[i_star]
    cls = cache.uav_cls[uav]
    idx_m = k % 2
    prune = ~cache.static_ok[cls][m, idx_m]
    if not enforce or model is None:
        return prune
    _t1, _t2, e_tr, e_op = _cand_times(node, i_star, m, k, cache)
    E_new = float(node.E[i_star]) - e_tr - e_op
    need = float(model.reserve_wh()) + _budget_min(cache, cls)[m, idx_m]
    with np.errstate(invalid="ignore"):
        ok = np.isfinite(E_new) & (E_new >= need - _EPS)
    return prune | ~ok


def turn_band_conflict(node: IDPNode, i_star: int, cand_strip: np.ndarray, cand_k: np.ndarray,
                       strips: Mapping[str, Strip] | None, cache: CostCache, cfg: PlannerConfig) -> np.ndarray:
    """同胞多機迴轉帶時間重疊剪枝（規格 §10.2、§13.4；D18/D38）：(M,) bool，True = 剪掉。

    迴轉帶 = 胞端線外之迴轉區（左端 0／右端 1）。i* 之候選轉移佔用「起點端（現條帶 l 之終端）與終點端（m 之起始端）」
    之側向帶 [min(v_l, v_m) − ov, max(v_l, v_m) + ov]（ov = max(0, R_eff − Δv/2)：Ω 側向溢出、U 型 0；進入段只佔 m 之起始端）
    於 [T_i, T_i + t^trans]；其作業以 v_m ± d_sep,lat/2 於起始端 [t1, t1 + τ]、終端 [t2 − τ, t2] 通過迴轉區
    （τ = 2.7 R_eff / V_g），整條帶 [t1, t2] 對他機作業帶用側向間距規則 |Δv| < d_sep,lat。其他機 j（同胞、已進入）之最後
    一次轉移（[t_busy_from, t_op_from]）與作業（[t_op_from, t_op_end]）同法。同端 ∧ 側向重疊 ∧ 時間重疊 → 剪。"""
    m = np.asarray(cand_strip, dtype=int).reshape(-1)
    k = np.asarray(cand_k, dtype=int).reshape(-1)
    M = m.size
    out = np.zeros(M, dtype=bool)
    if M == 0 or node.n_uav < 2:
        return out
    uav = node.uav_ids[i_star]
    cls = cache.uav_cls[uav]
    g = _strip_geom(cache, cls, strips)
    R = float(cache.R_eff.get(cls, 0.0))
    vg_tab = cache.op_vg.get(cls)
    t_tr, t_op, _a, _b = _cand_times(node, i_star, m, k, cache)
    T0 = float(node.T[i_star])
    t1 = T0 + t_tr
    t2 = t1 + t_op
    half = 0.5 * float(cfg.d_sep_lat_m)
    idx_m = k % 2
    v_m = g.v[m]
    st_end_m = idx_m.copy()                      # LR（idx 0）自左端 0 起掃；RL 自右端 1
    en_end_m = 1 - st_end_m

    def tau_of(li: np.ndarray, idx: np.ndarray) -> np.ndarray:
        if vg_tab is None:
            return np.full(np.asarray(li).shape, 2.7 * R / 20.0)
        vg = vg_tab[li, idx]
        vg = np.where(np.isfinite(vg) & (vg > 1.0), vg, 20.0)
        return 2.7 * R / vg

    tau_m = tau_of(m, idx_m)
    l = int(node.last_idx[i_star])
    if l >= 0:
        en_end_l = 0 if int(node.last_dir[i_star]) == 1 else 1
        ov = np.maximum(0.0, R - 0.5 * np.abs(g.v[l] - v_m))      # D38：Ω（Δv < 2R）側向溢出 R − Δv/2；U 型 0
        tr_lo = np.minimum(g.v[l], v_m) - ov
        tr_hi = np.maximum(g.v[l], v_m) + ov
        tr_ends = (np.full(M, en_end_l), st_end_m)
    else:
        tr_lo, tr_hi = v_m - 1.0, v_m + 1.0
        tr_ends = (st_end_m, st_end_m)
    tr_t0, tr_t1 = np.full(M, T0), t1
    op_lo, op_hi = v_m - half, v_m + half

    def ovl_t(a0, a1, b0, b1):
        return (a0 < b1) & (b0 < a1)

    def ovl_v(lo_a, hi_a, lo_b, hi_b):
        return (lo_a < hi_b) & (lo_b < hi_a)

    for j in range(node.n_uav):
        if j == i_star:
            continue
        lj = int(node.last_idx[j])
        if lj < 0:
            continue
        same = g.cell[m] == g.cell[lj]
        if not same.any():
            continue
        dj = int(node.last_dir[j])
        idx_j = (dj + 1) // 2
        vj = float(g.v[lj])
        st_end_j = idx_j
        en_end_j = 1 - idx_j
        tau_j = float(tau_of(np.array([lj]), np.array([idx_j]))[0])
        j_op0, j_op1 = float(node.t_op_from[j]), float(node.t_op_end[j])
        jo_lo, jo_hi = vj - half, vj + half
        c = np.zeros(M, dtype=bool)
        # --- j 之作業通過迴轉區（起始端／終端） vs i* 轉移帶 --------------------------
        for e_j, (ta, tb) in ((st_end_j, (j_op0, j_op0 + tau_j)), (en_end_j, (j_op1 - tau_j, j_op1))):
            same_end = (tr_ends[0] == e_j) | (tr_ends[1] == e_j)
            c |= same_end & ovl_v(tr_lo, tr_hi, jo_lo, jo_hi) & ovl_t(tr_t0, tr_t1, ta, tb)
        # --- i* 作業通過迴轉區 vs j 之最後轉移帶 --------------------------------------
        pj = int(node.prev_idx[j]) if node.prev_idx.size else -1
        if pj >= 0:
            pdj = int(node.prev_dir[j]) if node.prev_dir.size else -dj
            en_end_pj = 0 if pdj == 1 else 1
            ovj = max(0.0, R - 0.5 * abs(float(g.v[pj]) - vj))
            jt_lo, jt_hi = min(g.v[pj], vj) - ovj, max(g.v[pj], vj) + ovj
            j_ends = (en_end_pj, st_end_j)
        else:
            jt_lo, jt_hi = vj - 1.0, vj + 1.0
            j_ends = (st_end_j, st_end_j)
        jt0, jt1 = float(node.t_busy_from[j]), j_op0
        for e_i, (ta, tb) in ((st_end_m, (t1, t1 + tau_m)), (en_end_m, (t2 - tau_m, t2))):
            same_end = (e_i == j_ends[0]) | (e_i == j_ends[1])
            c |= same_end & ovl_v(op_lo, op_hi, jt_lo, jt_hi) & ovl_t(ta, tb, jt0, jt1)
        # --- 轉移 vs 轉移（同端） ----------------------------------------------------
        same_end = np.zeros(M, dtype=bool)
        for e in (0, 1):
            same_end |= ((tr_ends[0] == e) | (tr_ends[1] == e)) & (e in j_ends)
        c |= same_end & ovl_v(tr_lo, tr_hi, jt_lo, jt_hi) & ovl_t(tr_t0, tr_t1, jt0, jt1)
        # --- 作業 vs 作業：側向間距 d_sep,lat --------------------------------------------
        c |= ovl_v(op_lo, op_hi, jo_lo, jo_hi) & ovl_t(t1, t2, j_op0, j_op1)
        out |= same & c
    return out


# ---------------------------------------------------------------------------
# 節點操作
# ---------------------------------------------------------------------------
def _root_node(uav_ids: Sequence[str], init_states: Mapping[str, UAVState], nad: np.ndarray) -> IDPNode:
    lam = len(uav_ids)
    T = np.array([float(init_states[u].t_s) for u in uav_ids])
    E = np.array([float(init_states[u].e_rem_wh) for u in uav_ids])
    return IDPNode(tuple(uav_ids), np.full(lam, -1, dtype=int), np.full(lam, -1, dtype=int), T, E,
                   np.ones(lam, dtype=bool), nad.copy(), 0.0, None, None,
                   prev_idx=np.full(lam, -1, dtype=int), prev_dir=np.full(lam, -1, dtype=int), t_busy_from=T.copy(),
                   t_op_from=T.copy(), t_op_end=T.copy())


def _child(node: IDPNode, i: int, m: int, k: int, t_tr: float, t_op: float, e_tr: float, e_op: float, W: float) -> IDPNode:
    """APPLY（規格 §10.4）：更新 S_i（新端點、依 k 之方位）、T_i、E_i、NAD、決策。"""
    T = node.T.copy()
    E = node.E.copy()
    li = node.last_idx.copy()
    ld = node.last_dir.copy()
    nad = node.nad_mask.copy()
    pv = node.prev_idx.copy()
    pvd = node.prev_dir.copy()
    tb, to, te = node.t_busy_from.copy(), node.t_op_from.copy(), node.t_op_end.copy()
    T0 = float(T[i])
    T[i] = T0 + t_tr + t_op
    E[i] = E[i] - e_tr - e_op
    pv[i] = li[i]
    pvd[i] = ld[i]
    li[i] = m
    ld[i] = -1 if (k % 2) == 0 else 1
    nad[m] = False
    tb[i], to[i], te[i] = T0, T0 + t_tr, T[i]
    return IDPNode(node.uav_ids, li, ld, T, E, node.active.copy(), nad, float(W), node, (i, int(m), int(k)),
                   prev_idx=pv, prev_dir=pvd, t_busy_from=tb, t_op_from=to, t_op_end=te, dec_t_tr=float(t_tr), dec_t_op=float(t_op),
                   dec_e_tr=float(e_tr), dec_e_op=float(e_op), depth=node.depth + 1)


def _rtb_time(node: IDPNode, i: int, cache: CostCache, site: str | None) -> float:
    uav = node.uav_ids[i]
    cls = cache.uav_cls[uav]
    l = int(node.last_idx[i])
    if l < 0 or site is None or (cls, site) not in cache.rtb:
        return 0.0 if l < 0 else _INF
    return float(cache.rtb[(cls, site)][l, (int(node.last_dir[i]) + 1) // 2])


def _node_W(node: IDPNode, cache: CostCache, cfg: PlannerConfig, site: str | None) -> float:
    """節點自身之 makespan 型成本（無新決策）：max_j (T_j + t̂_RTB,j) + ε Σ。"""
    if cfg.metric == "paper":
        return float(node.W)
    base = node.T.copy()
    for j in range(node.n_uav):
        l = int(node.last_idx[j])
        if l >= 0 and node.active[j]:
            cls = cache.uav_cls[node.uav_ids[j]]
            rh = _rtb_hat(cache, cls, cfg, site)
            f1, _f2, _a = _nad_rtb_floor(node, rh)
            base[j] += min(rh[l, (int(node.last_dir[j]) + 1) // 2], f1)
    return float(base.max() + cfg.eps_sum * base.sum())


def _rtb_child(node: IDPNode, i: int, cache: CostCache, cfg: PlannerConfig, site: str | None) -> IDPNode:
    """逐機 RTB（規格 §10.3）：T_i += t^RTB(S_i)、E 固定、active=False；序列保留、其餘機繼續。"""
    T = node.T.copy()
    t_rtb = _rtb_time(node, i, cache, site)
    if math.isfinite(t_rtb):
        T[i] += t_rtb
    act = node.active.copy()
    act[i] = False
    child = IDPNode(node.uav_ids, node.last_idx.copy(), node.last_dir.copy(), T, node.E.copy(), act, node.nad_mask.copy(),
                    node.W, node, (i, -1, -1), prev_idx=node.prev_idx.copy(), prev_dir=node.prev_dir.copy(),
                    t_busy_from=node.t_busy_from.copy(),
                    t_op_from=node.t_op_from.copy(), t_op_end=node.t_op_end.copy(), depth=node.depth + 1)
    child.W = _node_W(child, cache, cfg, site)
    return child


def _select_i_star(node: IDPNode) -> int:
    act = np.flatnonzero(node.active)
    order = sorted(act.tolist(), key=lambda j: (float(node.T[j]), node.uav_ids[j]))
    return int(order[0])


def _candidates(node: IDPNode, i: int, cache: CostCache, cfg: PlannerConfig, models: Mapping[str, EnergyModel],
                strips: Mapping[str, Strip], cell_order: Mapping[str, Sequence[str]] | None, K: int,
                use_turn_band: bool) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """機 i 之 K-nearest 候選 (m, k, t_tr, t_op, e_tr, e_op)（規格 §10.4：可行 → 剪枝 → 胞完成偏好 → K 最近）。"""
    uav = node.uav_ids[i]
    cls = cache.uav_cls[uav]
    nad = np.flatnonzero(node.nad_mask)
    empty = (np.zeros(0, dtype=int),) * 2 + (np.zeros(0),) * 4
    if nad.size == 0:
        return empty
    l = int(node.last_idx[i])
    if l < 0:
        ks = np.array([0, 1], dtype=int)
    else:
        idx_l = (int(node.last_dir[i]) + 1) // 2
        ks = np.array([2 * idx_l, 2 * idx_l + 1], dtype=int)
    m = np.repeat(nad, 2)
    k = np.tile(ks, nad.size)
    if l >= 0:
        same = (k // 2) == (k % 2)
        keep = np.ones(m.size, dtype=bool)
        if "same" not in cfg.transfer_types:
            keep &= ~same
        if "rev" not in cfg.transfer_types:
            keep &= same
        m, k = m[keep], k[keep]
    t_tr, t_op, e_tr, e_op = _cand_times(node, i, m, k, cache)
    ok = np.isfinite(t_tr) & np.isfinite(t_op)
    if ok.any():
        pr = energy_prune(node, i, m, k, cache, models.get(uav), enforce=bool(cfg.enforce_energy))
        ok &= ~pr
    if ok.any() and use_turn_band:
        # 迴轉帶剪枝為「軟」剪枝：全部候選皆衝突時保留（交由 deconflict.repair 以延遲盤旋處置），避免整機提前 RTB
        ok2 = ok & ~turn_band_conflict(node, i, m, k, strips, cache, cfg)
        if ok2.any():
            ok = ok2
    if not ok.any():
        return empty
    m, k, t_tr, t_op, e_tr, e_op = m[ok], k[ok], t_tr[ok], t_op[ok], e_tr[ok], e_op[ok]
    t_add = t_tr + t_op
    # 胞完成偏好（規格 §10.4／§10.6）：現胞尚有可行未覆蓋條帶 → near 只取現胞（K 最近）；現胞已盡 → 配置順序之下一胞；
    # 皆無 → 全部候選。（嚴格版：鄰胞條帶「更近」亦不提前離胞；見 CONTRACT_DEVIATIONS）
    g = _strip_geom(cache, cls, strips)
    pref = np.zeros(m.size, dtype=bool)
    if l >= 0:
        pref = g.cell[m] == g.cell[l]
    if not pref.any() and cell_order:
        order = cell_order.get(uav) or []
        cell_of = {sid: strips[sid].cell_id for sid in cache.strip_ids[cls] if sid in strips}
        for cid in order:
            pref = np.array([cell_of.get(cache.strip_ids[cls][int(mm)]) == cid for mm in m])
            if pref.any():
                break
    if pref.any():
        keep = np.flatnonzero(pref)
        m, k, t_tr, t_op, e_tr, e_op, t_add = m[keep], k[keep], t_tr[keep], t_op[keep], e_tr[keep], e_op[keep], t_add[keep]
    Kn = int(min(K, m.size))
    sel = np.argsort(t_add, kind="stable")[:Kn]
    return m[sel], k[sel], t_tr[sel], t_op[sel], e_tr[sel], e_op[sel]


def _expand_event(node: IDPNode, cache: CostCache, cfg: PlannerConfig, models: Mapping[str, EnergyModel],
                  strips: Mapping[str, Strip], cell_order: Mapping[str, Sequence[str]] | None, K: int, site: str | None
                  ) -> list[IDPNode]:
    i = _select_i_star(node)
    m, k, t_tr, t_op, e_tr, e_op = _candidates(node, i, cache, cfg, models, strips, cell_order, K, True)
    if m.size == 0:
        return [_rtb_child(node, i, cache, cfg, site)]
    W = stage_cost(node, i, m, k, cache, cfg, site_id=site)
    return [_child(node, i, int(m[c]), int(k[c]), float(t_tr[c]), float(t_op[c]), float(e_tr[c]), float(e_op[c]), float(W[c]))
            for c in range(m.size) if math.isfinite(W[c])]


def _expand_lockstep(node: IDPNode, cache: CostCache, cfg: PlannerConfig, models: Mapping[str, EnergyModel],
                     strips: Mapping[str, Strip], cell_order: Mapping[str, Sequence[str]] | None, K: int,
                     site: str | None, n_keep: int) -> list[IDPNode]:
    """論文 Algorithm 2 鎖步：每階段每機各取一條，Π = ∏ near_i（條帶互異；候選數不足時允許 RTB 選項），成本以向量化
    計算後只實體化最佳 `n_keep` 個聯合決策。"""
    act = [int(j) for j in np.flatnonzero(node.active)]
    n_nad = int(node.nad_mask.sum())
    opts: list[tuple[np.ndarray, ...]] = []
    for i in act:
        cand = _candidates(node, i, cache, cfg, models, strips, cell_order, K, False)
        m = cand[0]
        allow_rtb = m.size == 0 or n_nad < len(act)
        if allow_rtb:
            m = np.concatenate([m, [-1]])
            cand = (m, np.concatenate([cand[1], [-1]])) + tuple(np.concatenate([c, [0.0]]) for c in cand[2:])
        opts.append(cand)
    sizes = [o[0].size for o in opts]
    if not sizes or any(s == 0 for s in sizes):
        return [node]
    grids = np.meshgrid(*[np.arange(s) for s in sizes], indexing="ij")
    J = np.stack([g.ravel() for g in grids], axis=1)                     # (P, n_act) 選項索引
    P = J.shape[0]
    M_sel = np.stack([opts[a][0][J[:, a]] for a in range(len(act))], axis=1)
    # 條帶互異（Eq.23）且至少一機有條帶
    valid = np.ones(P, dtype=bool)
    for a in range(len(act)):
        for b in range(a + 1, len(act)):
            valid &= ~((M_sel[:, a] == M_sel[:, b]) & (M_sel[:, a] >= 0))
    valid &= (M_sel >= 0).any(axis=1)
    if not valid.any():
        return [_rtb_child(node, act[0], cache, cfg, site)]
    tau0 = float(cfg.tau0_s)
    Tn = np.repeat(node.T[None, :], P, axis=0)
    rel = np.zeros(P)
    rh_fin = np.zeros((P, node.n_uav))
    for a, i in enumerate(act):
        m_a, k_a, t_tr_a, t_op_a = opts[a][0][J[:, a]], opts[a][1][J[:, a]], opts[a][2][J[:, a]], opts[a][3][J[:, a]]
        t_add = np.where(m_a >= 0, t_tr_a + t_op_a, 0.0)
        rel += t_add / (float(node.T[i]) + tau0)
        Tn[:, i] += t_add
        if cfg.metric != "paper":
            cls = cache.uav_cls[node.uav_ids[i]]
            rh = _rtb_hat(cache, cls, cfg, site)
            mm = np.where(m_a >= 0, m_a, 0)
            f1, f2, a1 = _nad_rtb_floor(node, rh)
            # 鎖步：本階段取走 n_act 條後 NAD 是否為空 → 端點真值；否則側向中性下界
            rest = f2 if (n_nad - len(act)) <= 1 else f1
            rest = rest if (n_nad - len(act)) > 0 else _INF
            rh_fin[:, i] = np.where(m_a >= 0, np.minimum(rh[mm, np.where(k_a >= 0, k_a % 2, 0)], rest), 0.0)
            # RTB 選項：真實 RTB 時間加入 T
            l = int(node.last_idx[i])
            if l >= 0 and (m_a < 0).any():
                t_rtb = _rtb_time(node, i, cache, site)
                Tn[:, i] += np.where(m_a < 0, t_rtb if math.isfinite(t_rtb) else 0.0, 0.0)
    if cfg.metric == "paper":
        W = rel
    else:
        base = Tn + rh_fin
        for j in range(node.n_uav):
            if j not in act and node.active[j] and int(node.last_idx[j]) >= 0:
                cls = cache.uav_cls[node.uav_ids[j]]
                base[:, j] += _rtb_hat(cache, cls, cfg, site)[int(node.last_idx[j]), (int(node.last_dir[j]) + 1) // 2]
        W = base.max(axis=1) + float(cfg.eps_sum) * base.sum(axis=1) + float(cfg.mu_paper) * rel
    W = np.where(valid, W, np.inf)
    order = np.argsort(W, kind="stable")[:max(1, n_keep)]
    out: list[IDPNode] = []
    for p in order:
        if not np.isfinite(W[p]):
            continue
        cur = node
        for a, i in enumerate(act):
            jj = int(J[p, a])
            m_a = int(opts[a][0][jj])
            if m_a < 0:
                cur = _rtb_child(cur, i, cache, cfg, site)
            else:
                cur = _child(cur, i, m_a, int(opts[a][1][jj]), float(opts[a][2][jj]), float(opts[a][3][jj]),
                             float(opts[a][4][jj]), float(opts[a][5][jj]), float(W[p]))
        cur.W = float(W[p])
        out.append(cur)
    return out


# ---------------------------------------------------------------------------
# Beam-IDP（契約 §10；規格 §10.4）
# ---------------------------------------------------------------------------
def _apply_warm(node: IDPNode, uav_id: str, seq: Sequence[tuple[str, int]], cache: CostCache) -> IDPNode:
    """暖啟動：依序套用 (strip_id, dir)（不在 NAD 者仍可套用；轉移不可行者以 0 成本套用）。"""
    i = node.uav_ids.index(uav_id)
    cls = cache.uav_cls[uav_id]
    cur = node
    for sid, d in seq:
        m = cache.idx[cls].get(sid)
        if m is None:
            continue
        idx_m = (int(d) + 1) // 2
        l = int(cur.last_idx[i])
        k = idx_m if l < 0 else 2 * ((int(cur.last_dir[i]) + 1) // 2) + idx_m
        t_tr, t_op, e_tr, e_op = _cand_times(cur, i, np.array([m]), np.array([k]), cache)
        vals = [float(v[0]) if math.isfinite(v[0]) else 0.0 for v in (t_tr, t_op, e_tr, e_op)]
        cur = _child(cur, i, int(m), int(k), *vals, cur.W)
    return cur


def _walk(node: IDPNode) -> list[IDPNode]:
    chain: list[IDPNode] = []
    cur: IDPNode | None = node
    while cur is not None and cur.decision is not None:
        chain.append(cur)
        cur = cur.parent
    chain.reverse()
    return chain


def _path_of(cache: CostCache, cls: str, uav_id: str, prev: tuple[int, int] | None, m: int, idx_m: int
             ) -> DubinsPath | None:
    """自快取取回繞行路徑；非繞行對以同 R 之 Dubins 重建。"""
    ids = cache.strip_ids[cls]
    sid = ids[m]
    st = cache.strips[cls][sid]
    R = float(cache.R_eff.get(cls, 0.0))
    try:
        if prev is None:
            p = cache.entry_paths.get(uav_id, {}).get((sid, idx_m))
            if p is not None:
                return p
            s0 = cache.init_states.get(uav_id)
            if s0 is None:
                return None
            R = float(s0.r_eff_m) if s0.r_eff_m > 0 else R
            return dubins_path(Pose(s0.x_m, s0.y_m, s0.theta_rad), st.start_state(-1 if idx_m == 0 else 1), R)
        l, idx_l = prev
        k = 2 * idx_l + idx_m
        p = cache.tr_paths.get(cls, {}).get((ids[l], sid, k))
        if p is not None:
            return p
        sl = cache.strips[cls][ids[l]]
        return dubins_path(sl.end_state(-1 if idx_l == 0 else 1), st.start_state(-1 if idx_m == 0 else 1), R)
    except Exception:
        return None


def _sequences_from(node: IDPNode, cache: CostCache, init_states: Mapping[str, UAVState], site: str | None,
                    strips: Mapping[str, Strip]) -> dict[str, UAVSequence]:
    chain = _walk(node)
    per: dict[str, list[tuple[IDPNode, int, int]]] = {u: [] for u in node.uav_ids}
    for nd in chain:
        i, m, k = nd.decision  # type: ignore[misc]
        if m >= 0:
            per[node.uav_ids[i]].append((nd, m, k))
    out: dict[str, UAVSequence] = {}
    for i, uav in enumerate(node.uav_ids):
        cls = cache.uav_cls[uav]
        st0 = init_states[uav]
        va = _spec_of(cache, cls).cruise_airspeed_mps
        R = float(cache.R_eff.get(cls, st0.r_eff_m))
        items: list[SeqItem] = []
        decisions: list[int] = []
        entry = PathCost(0.0, 0.0, 0.0, None)
        t = float(st0.t_s)
        E = float(st0.e_rem_wh)
        prev: tuple[int, int] | None = None
        for j, (nd, m, k) in enumerate(per[uav]):
            idx_m = k % 2
            d = -1 if idx_m == 0 else 1
            pc = PathCost(nd.dec_t_tr, nd.dec_e_tr, 0.0, _path_of(cache, cls, uav, prev, m, idx_m))
            if prev is None:
                ent_len = cache.entry_len.get(uav)
                pc.length_m = float(ent_len[m, idx_m]) if ent_len is not None and ent_len.shape[0] > m else 0.0
                pc.n_laps = int(cache.entry_laps[uav][m, idx_m]) if uav in cache.entry_laps and cache.entry_laps[uav].shape[0] > m else 0
                pc.dz_m = float(cache.strips[cls][cache.strip_ids[cls][m]].z_op_m - st0.z_m)
                entry = pc
                transfer = None
            else:
                l, idx_l = prev
                kk = 2 * idx_l + idx_m
                pc.length_m = float(cache.tr_len[cls][l, m, kk])
                pc.n_laps = int(cache.tr_laps[cls][l, m, kk]) if cls in cache.tr_laps else 0
                pc.dz_m = float(cache.tr_dz[cls][l, m, kk])
                transfer = pc
            t_start = t + nd.dec_t_tr
            t_end = t_start + nd.dec_t_op
            E = E - nd.dec_e_tr - nd.dec_e_op
            items.append(SeqItem(cache.strip_ids[cls][m], d, t_start, t_end, E, transfer))
            decisions.append(int(m))
            t = t_end
            prev = (m, idx_m)
        if items:
            st = cache.strips[cls][items[-1].strip_id]
            pe = st.end_state(items[-1].dir)
            state_end = UAVState(pe.x_m, pe.y_m, pe.theta_rad, float(st.z_op_m), t, E, R, va)
        else:
            state_end = replace(st0)
        out[uav] = UAVSequence(uav, items, entry, None, None, state_end, t, E, decisions, [], cls)
    return out


def _final_makespan(node: IDPNode, cache: CostCache, site: str | None) -> float:
    ms = 0.0
    for i, uav in enumerate(node.uav_ids):
        l = int(node.last_idx[i])
        if l < 0:
            continue
        cls = cache.uav_cls[uav]
        t_end = float(node.t_op_end[i])
        t_rtb = _rtb_time(node, i, cache, site)
        ms = max(ms, t_end + t_rtb + _landing_fixed_time_s(_spec_of(cache, cls)))
    return ms


def beam_idp(team: Team, strips: dict[str, Strip], cache: CostCache, models: dict[str, EnergyModel],
             init_states: dict[str, UAVState], cfg: PlannerConfig,
             warm_start: dict[str, list[tuple[str, int]]] | None = None, time_budget_s: float | None = None, *,
             cell_order: Mapping[str, Sequence[str]] | None = None, inactive: Iterable[str] | None = None,
             site_id: str | None = None) -> IDPResult:
    """規格 §10.4 BEAM_IDP：`cfg.idp_branching="event"`（事件驅動，每階段一機一條、O(nBK)）或 `"lockstep"`（論文結構）。

    * beam 依 W 排序（含 max 項、t̂_RTB 與 ε 總和）、去重鍵 :meth:`IDPNode.key`；`elapsed > time_budget_s`（None →
      cfg.realtime_budget_s）→ B = 1（`degraded_B`）。
    * `warm_start`（uav → [(strip_id, dir)]）為**固定前綴**（規格 §10.5 修復用）：根節點 = 套用後之狀態，其條帶自 NAD 移除；
      `inactive` 之機於根節點即 RTB（截斷機「自截斷點直接 RTB」）。`cell_order`（uav → cell_id 序）供胞完成偏好；
      `site_id` = 回收點（D40；None → cache.sites 首鍵）。
    * 回傳每機 `UAVSequence`（不含 RTB，`append_rtb` 另加）、`must_rtb`（因 near = ∅ 提前 RTB 且當時 NAD ≠ ∅ 之機）、
      `leftover`、`makespan_s`（max_i t_end + 真實 RTB + 降落固定時間）。"""
    cls = team.cls
    uav_ids = tuple(u for u in team.uav_ids if u in init_states and u in cache.entry)
    site = _recovery_site(cache, cls, site_id)
    n = cache.n_strips(cls)
    idx_map = cache.idx.get(cls, {})
    nad = np.zeros(n, dtype=bool)
    missing: list[str] = []
    for sid in team.strip_ids:
        j = idx_map.get(sid)
        if j is None:
            missing.append(sid)
        else:
            nad[j] = True
    if not uav_ids:
        return IDPResult({}, [], list(dict.fromkeys(list(team.strip_ids))), 0.0, 0, False, dict(init_states),
                         dict(cell_order or {}), site, team)
    root = _root_node(uav_ids, init_states, nad)
    if warm_start:
        for u, seq in warm_start.items():
            if u in uav_ids and seq:
                for sid, _d in seq:
                    j = idx_map.get(sid)
                    if j is not None:
                        root.nad_mask[j] = False
                root = _apply_warm(root, u, seq, cache)
    for u in (inactive or ()):
        if u in uav_ids:
            root = _rtb_child(root, uav_ids.index(u), cache, cfg, site)
    root.W = _node_W(root, cache, cfg, site)
    branching = cfg.idp_branching
    B = max(1, int(cfg.B))
    K_all = int(cfg.k_nearest(2 * max(1, n)))
    budget = float(cfg.realtime_budget_s if time_budget_s is None else time_budget_s)
    t_begin = time.perf_counter()
    beam: list[IDPNode] = [root]
    nodes_expanded = 0
    degraded = False
    for _stage in range(2 * n + 4 * len(uav_ids) + 2):
        if not any(not nd.finished() for nd in beam):
            break
        cand: list[IDPNode] = []
        for nd in beam:
            if nd.finished():
                cand.append(nd)
                continue
            nodes_expanded += 1
            if branching == "lockstep":
                cand.extend(_expand_lockstep(nd, cache, cfg, models, strips, cell_order, K_all, site, 3 * B))
            else:
                cand.extend(_expand_event(nd, cache, cfg, models, strips, cell_order, K_all, site))
        if not cand:
            break
        best: dict[Hashable, IDPNode] = {}
        for c in cand:
            key = c.key()
            cur = best.get(key)
            if cur is None or c.W < cur.W:
                best[key] = c
        beam = sorted(best.values(), key=lambda c: (c.W, c.depth))[:B]
        if time.perf_counter() - t_begin > budget:
            B = 1
            degraded = True
    # 最佳節點：leftover 少者優先，再 max_i [T_i + t^RTB_i(S_i)]（真實 Dubins）
    scored = sorted(beam, key=lambda nd: (int(nd.nad_mask.sum()), _final_makespan(nd, cache, site), nd.W))
    top = scored[0]
    seqs = _sequences_from(top, cache, init_states, site, strips)
    must_rtb: list[str] = []
    for nd in _walk(top):
        i, m, _k = nd.decision  # type: ignore[misc]
        if m < 0 and nd.parent is not None and nd.parent.nad_mask.any():
            u = uav_ids[i]
            if u not in must_rtb:
                must_rtb.append(u)
    leftover = [cache.strip_ids[cls][j] for j in np.flatnonzero(top.nad_mask)] + missing
    return IDPResult(seqs, must_rtb, leftover, _final_makespan(top, cache, site), nodes_expanded, degraded,
                     dict(init_states), dict(cell_order or {}), site, team)


# ---------------------------------------------------------------------------
# ENERGY_SAFE_CHECK、返航附加、能量修復（規格 §5.5、§10.5、D40）
# ---------------------------------------------------------------------------
def _site_order(cache: CostCache, cls: str, l: int, idx: int, recovery: str | None) -> list[str]:
    others = [s for (c, s) in cache.rtb if c == cls and s != recovery]
    others.sort(key=lambda s: float(cache.rtb[(cls, s)][l, idx]))
    return ([recovery] if recovery is not None and (cls, recovery) in cache.rtb else []) + others


def _rtb_pathcost(cache: CostCache, cls: str, seq: UAVSequence, site: str) -> PathCost | None:
    if not seq.items:
        return None
    last = seq.items[-1]
    l = cache.idx[cls][last.strip_id]
    idx = (last.dir + 1) // 2
    t = float(cache.rtb[(cls, site)][l, idx])
    if not math.isfinite(t):
        return None
    e = float(cache.rtb_e[(cls, site)][l, idx])
    p = cache.rtb_paths.get((cls, site), {}).get((last.strip_id, idx))
    if p is None:
        from .costs import resolve_site_entry
        s_app, _z = resolve_site_entry(cache.sites, site, cls)
        st = cache.strips[cls][last.strip_id]
        try:
            p = dubins_path(st.end_state(last.dir), s_app, float(cache.R_eff.get(cls, seq.state_end.r_eff_m)))
        except Exception:
            p = None
    laps = int(cache.rtb_laps[(cls, site)][l, idx]) if (cls, site) in cache.rtb_laps else 0
    from .costs import resolve_site_entry
    _s_app, z_tc = resolve_site_entry(cache.sites, site, cls)
    dz = float(z_tc - cache.strips[cls][last.strip_id].z_op_m)
    return PathCost(t, e, float(cache.rtb_len[(cls, site)][l, idx]), p, laps, dz)


def _rtb_profile(seq: UAVSequence, rtb: PathCost, site: str, cache: CostCache, cls: str, model: EnergyModel
                 ) -> tuple[float, float]:
    """返航段之 (起飛前即已耗用之爬升螺旋能量 e_up, 沿路徑均攤之能量 e_path)（R-07）。

    規格 §5.5 之返航消耗側原本以「與弧長線性」內插 `cache.rtb_e`；但 `EnergyBudgetTable` 的同一條返航可能
    以 n_up 圈爬升螺旋開頭、n_down 圈下降螺旋結尾（山丘實測 A 最多 1／1 圈、B 最多 2／1 圈），螺旋能量並不
    沿弧長分布。本函式把 **爬升螺旋前置、下降螺旋後置**，其餘沿弧長均攤，並以 `budget.rtb_wh − e^LDG`
    （AVOID+ENVELOPE，與需求側同源）與 `cache.rtb_e` 取大者為總量，使消耗側不再樂觀。"""
    budget = cache.budget
    last = seq.items[-1]
    e_tot = float(rtb.e_wh)
    n_up = n_dn = 0
    try:
        l = cache.idx[cls][last.strip_id]
        idx = (last.dir + 1) // 2
        j = budget._site_index(site)
        e_tot = max(e_tot, float(budget.rtb_wh(cls, last.strip_id, last.dir, site)) - float(model.e_landing_wh()))
        laps = budget.laps.get(cls)
        if laps is not None:
            n_up, n_dn = int(laps[l, idx, j, 0]), int(laps[l, idx, j, 1])
    except Exception:                                       # pragma: no cover - 表格缺列 → 退回 rtb.e_wh
        return 0.0, float(rtb.e_wh)
    if n_up <= 0 and n_dn <= 0:
        return 0.0, e_tot
    spec = _spec_of(cache, cls)
    r_eff = float(cache.R_eff.get(cls, 0.0)) or 1.0
    va = float(spec.cruise_airspeed_mps)
    t_lap = 2.0 * math.pi * r_eff / max(va, 1e-3)
    gc = spec.gamma_c_auw(model.auw_kg)
    p_up, p_dn, pc = model.p_climb_w(gc), model.p_descent_w(spec.gamma_d()), model.p_cruise_w()
    conv = 1.0 if model.base_minute_mode else 1.0 / 3600.0
    e_up = max(0.0, p_up * n_up * t_lap * model.k_unc * conv)
    e_dn = max(0.0, p_dn * n_dn * t_lap * model.k_unc * conv)
    if e_up + e_dn > e_tot:                                 # 保守夾限：螺旋能量不超過總量
        s = e_tot / max(e_up + e_dn, 1e-9)
        e_up, e_dn = e_up * s, e_dn * s
    _ = pc
    e_path = max(0.0, e_tot - e_up - e_dn)
    if e_up + e_path < float(rtb.e_wh):                     # 末樣本不得比原本（線性內插 rtb.e_wh）樂觀
        e_path = max(0.0, float(rtb.e_wh) - e_up)
    return float(e_up), float(e_path)


def energy_safe_check(seq: UAVSequence, rtb: PathCost | None, site: str, cache: CostCache, ctx: MissionContext,
                      model: EnergyModel | None, cfg: PlannerConfig) -> EnergyInfeasible | None:
    """規格 §5.5 ENERGY_SAFE_CHECK（序列層級）：每條帶末 E ≥ r·E_nom + budget.min_rtb；返航路徑每 ~200 m 樣本
    E_j ≥ r·E_nom + min_a point_wh(x_j, a)（止於 P_tc）；落地 E − e^LDG ≥ r·E_nom。通過 → None。

    R-06（兩段式）：``point_wh`` 只是 O(1) **粗篩**、並非上界（實測最壞低估 17.7 Wh）。餘裕小於
    :meth:`~hv_dccpp.costs.EnergyBudgetTable.point_wh_slack_wh` 時，改以
    :meth:`~hv_dccpp.costs.EnergyBudgetTable.point_wh_exact`（AVOID+ENVELOPE 精算）定案，與
    `sim.check_energy` 同一準則。
    R-07（返航消耗側）：能量以 :func:`_rtb_profile` 之「爬升螺旋前置 + 沿弧長均攤」取代線性內插，總量取自
    ``budget.rtb_wh − e^LDG``；高度取兩端較低者（線性上爬會低估備降爬升需求）。"""
    if not cfg.enforce_energy or model is None:
        return None
    cls = seq.cls or cache.uav_cls[seq.uav_id]
    reserve = float(model.reserve_wh())
    for j, it in enumerate(seq.items):
        need = reserve + float(cache.budget.min_rtb_wh(cls, it.strip_id, it.dir))
        if it.e_end_wh < need - _EPS:
            return EnergyInfeasible(seq.uav_id, j, 0, it.strip_id, need - it.e_end_wh)
    if rtb is None:
        return EnergyInfeasible(seq.uav_id, len(seq.items), 0, None, _INF)
    E0 = float(seq.e_end_wh)
    E1 = E0 - float(rtb.e_wh)
    sites = [s for s in cache.budget.site_ids]
    if rtb.path is not None and sites:
        from .costs import resolve_site_entry
        _s_app, z_tc = resolve_site_entry(cache.sites, site, cls)
        L = float(rtb.path.length_m)
        n_s = max(2, int(math.ceil(L / _RTB_CHECK_DS_M)) + 1)
        pts = rtb.path.sample(max(L / (n_s - 1), 1.0))
        frac = np.linspace(0.0, 1.0, pts.shape[0])
        z0 = float(seq.state_end.z_m)
        e_up, e_path = _rtb_profile(seq, rtb, site, cache, cls, model)
        z_lo = min(z0, float(z_tc))
        try:
            slack = float(cache.budget.point_wh_slack_wh(cls))
        except Exception:                                   # pragma: no cover - 舊版 costs
            slack = 0.0
        for j in range(pts.shape[0]):
            E_j = E0 - e_up - frac[j] * e_path
            th = float(math.atan2(pts[min(j + 1, pts.shape[0] - 1), 0] - pts[j, 0],
                                  pts[min(j + 1, pts.shape[0] - 1), 1] - pts[j, 1])) if pts.shape[0] > 1 else 0.0
            e_alt = min(cache.budget.point_wh(cls, pts[j, :2], z_lo, s) for s in sites)
            if E_j < reserve + e_alt + slack - _EPS and math.isfinite(e_alt) and slack > 0.0:
                e_exact = _INF                              # 兩段式：餘裕接近門檻 → AVOID+ENVELOPE 精算
                for s in sites:
                    try:
                        v = float(cache.budget.point_wh_exact(cls, pts[j, :2], z_lo, th, s))
                    except Exception:                       # pragma: no cover
                        continue
                    if math.isfinite(v):
                        e_exact = min(e_exact, v)
                if math.isfinite(e_exact):
                    e_alt = e_exact
            if E_j < reserve + e_alt - _EPS:
                return EnergyInfeasible(seq.uav_id, len(seq.items), j, None, reserve + e_alt - E_j)
    if E1 - float(model.e_landing_wh()) < reserve - _EPS:
        return EnergyInfeasible(seq.uav_id, len(seq.items) + 1, 0, None, reserve + model.e_landing_wh() - E1)
    return None


def append_rtb(seq: UAVSequence, ctx: MissionContext, cache: CostCache, model: EnergyModel, cfg: PlannerConfig
               ) -> UAVSequence | EnergyInfeasible:
    """APPEND_RTB（規格 §10.5、D40）：先回收點 H（`ctx.recovery_site_id`），再依 rtb 時間遞增試備降點；每站以
    :func:`energy_safe_check` 驗證（route + path）；備降 → `warnings += planned_landing_at_alternate`。
    無條帶之序列 → 原樣回傳（不起飛）。全部不可達 → :class:`EnergyInfeasible`。"""
    cls = seq.cls or cache.uav_cls[seq.uav_id]
    if not seq.items:
        return replace(seq, rtb=None, landing_site=ctx.recovery_site_id, cls=cls)
    last = seq.items[-1]
    l = cache.idx[cls][last.strip_id]
    idx = (last.dir + 1) // 2
    recovery = _recovery_site(cache, cls, ctx.recovery_site_id)
    first_fail: EnergyInfeasible | None = None
    for site in _site_order(cache, cls, l, idx, recovery):
        pc = _rtb_pathcost(cache, cls, seq, site)
        if pc is None:
            continue
        fail = energy_safe_check(seq, pc, site, cache, ctx, model, cfg)
        if fail is None:
            warn = list(seq.warnings)
            if site != recovery:
                warn.append(f"planned_landing_at_alternate({site})")
            return replace(seq, rtb=pc, landing_site=site, warnings=warn, cls=cls)
        if first_fail is None:
            first_fail = fail
    if first_fail is None:
        first_fail = EnergyInfeasible(seq.uav_id, len(seq.items), 0, None, _INF)
    return first_fail


def repair_energy(team: Team, result: IDPResult, infeasible: dict[str, EnergyInfeasible], cache: CostCache,
                  models: dict[str, EnergyModel], cfg: PlannerConfig, *, strips: Mapping[str, Strip] | None = None
                  ) -> IDPResult:
    """REPAIR_ENERGY（規格 §10.5）：含 x_j 之 SeqItem 起截斷該機序列（x_j 在返航段 → 截去末條帶），tail（含既有
    leftover）交暖啟動 `beam_idp`（其餘機序列為固定前綴，截斷機自截斷點直接 RTB）。回傳新 IDPResult（尚未 append_rtb）。"""
    cls = team.cls
    strips = strips if strips is not None else cache.strips.get(cls, {})
    warm: dict[str, list[tuple[str, int]]] = {}
    tail: list[str] = list(result.leftover)
    forced: list[str] = []
    for uav, seq in result.sequences.items():
        items = [(it.strip_id, it.dir) for it in seq.items]
        inf = infeasible.get(uav)
        if inf is None:
            warm[uav] = items
            continue
        j = int(inf.seg_index)
        if j >= len(items):
            j = max(0, len(items) - 1)
        warm[uav] = items[:j]
        tail += [sid for sid, _d in items[j:]]
        forced.append(uav)
    all_ids = list(dict.fromkeys([s for lst in warm.values() for s, _d in lst] + tail))
    team2 = Team(team.team_id, cls, list(team.uav_ids), list(team.cell_ids), all_ids, dict(team.t0_s),
                 dict(team.sortie_index))
    init = result.init_states or {}
    res = beam_idp(team2, dict(strips), cache, models, dict(init), cfg, warm_start=warm, cell_order=result.cell_order,
                   inactive=forced, site_id=result.site_id)
    res.team = team
    return res


# ---------------------------------------------------------------------------
# 2D 組裝（規格 §11.4 前置）
# ---------------------------------------------------------------------------
def assemble_2d(uav: UAVInstance, seq: UAVSequence, takeoff: CorridorResult, landing: CorridorResult,
                cache: CostCache, ctx: MissionContext) -> list[PathSegment2D]:
    """takeoff.segments（不含進入段）+ ENTRY（`seq.entry.path`，自 takeoff.exit_state）+ 作業／轉移 + RTB（`seq.rtb.path`，
    至 S^app）+ landing.segments；AVOID 段不重複。z 提示皆為 ENU z（進入：exit_state.z → z_op；OPER：z_op；RTB：z_op →
    z_tc）；OPER 段為折線 [pL, pR]（dir −1）／[pR, pL]（+1）、`sensor_on=True`、v_air = cache.op_va。"""
    cls = uav.cls
    spec = ctx.specs[cls]
    va = float(spec.cruise_airspeed_mps)
    R = float(cache.R_eff.get(cls, takeoff.exit_state.r_eff_m))
    segs: list[PathSegment2D] = list(takeoff.segments)
    ex = takeoff.exit_state
    if not seq.items:
        return segs + list(landing.segments)
    first = seq.items[0]
    s0 = cache.strips[cls][first.strip_id]
    p_entry = seq.entry.path
    ex_pose = Pose(float(ex.x_m), float(ex.y_m), float(ex.theta_rad))
    if p_entry is None or _pose_dist(p_entry.start, ex_pose) > 1e-3:
        p_entry = dubins_path(ex_pose, s0.start_state(first.dir), float(ex.r_eff_m) if ex.r_eff_m > 0 else R)
    segs.append(PathSegment2D(Phase.ENTRY, Mode.CRUISE, p_entry, None, None, va, False, float(ex.z_m), float(s0.z_op_m)))
    prev: Strip | None = None
    prev_dir = 0
    for it in seq.items:
        st = cache.strips[cls][it.strip_id]
        if prev is not None:
            p_tr = it.transfer.path if it.transfer is not None else None
            if p_tr is None:
                p_tr = dubins_path(prev.end_state(prev_dir), st.start_state(it.dir), R)
            segs.append(PathSegment2D(Phase.TRANSFER, Mode.CRUISE, p_tr, None, None, va, False, float(prev.z_op_m),
                                      float(st.z_op_m)))
        l = cache.idx[cls][it.strip_id]
        idx = (it.dir + 1) // 2
        v_op = float(cache.op_va[cls][l, idx]) if cls in cache.op_va and np.isfinite(cache.op_va[cls][l, idx]) else va
        line = np.vstack([st.pL, st.pR]) if it.dir == -1 else np.vstack([st.pR, st.pL])
        segs.append(PathSegment2D(Phase.OPERATE, Mode.CRUISE, line, it.strip_id, int(it.dir), v_op, True,
                                  float(st.z_op_m), float(st.z_op_m)))
        prev, prev_dir = st, it.dir
    assert prev is not None
    site = seq.landing_site or ctx.recovery_site_id
    z_tc = float(ctx.sites_enu[site][1]) + spec.transition_cmd(transition_profile=ctx.cfg.transition_profile).h_tc_cmd_m
    p_rtb = seq.rtb.path if seq.rtb is not None else None
    if landing.app_state is not None and (p_rtb is None or _pose_dist(p_rtb.end, landing.app_state) > 1e-3):
        p_rtb = dubins_path(prev.end_state(prev_dir), landing.app_state, R)
    if p_rtb is not None:
        segs.append(PathSegment2D(Phase.RTB, Mode.CRUISE, p_rtb, None, None, va, False, float(prev.z_op_m), z_tc))
    segs += list(landing.segments)
    return segs


def _pose_dist(a: Pose, b: Pose) -> float:
    return math.hypot(a.x_m - b.x_m, a.y_m - b.y_m) + abs(_angdiff(a.theta_rad, b.theta_rad)) * 10.0


def _angdiff(a: float, b: float) -> float:
    d = (a - b + math.pi) % (2 * math.pi) - math.pi
    return d


# ---------------------------------------------------------------------------
# UAVPlan 組裝
# ---------------------------------------------------------------------------
def build_uav_plan(uav: UAVInstance, seq: UAVSequence, takeoff: CorridorResult, landing: CorridorResult,
                   cache: CostCache, ctx: MissionContext, t0_s: float, strips: Mapping[str, Strip]) -> tuple[UAVPlan, Any]:
    """assemble_2d → altitude.build_3d → UAVPlan（時間戳、能量、t_arr_ptc、落地時刻、警告）。"""
    cfg = ctx.cfg
    cls = uav.cls
    spec = ctx.specs[cls]
    model = ctx.models.get(uav.uav_id) or ctx.models_rep[cls]
    site = seq.landing_site or ctx.recovery_site_id
    start = _start_site(ctx, uav.uav_id)
    tc = spec.transition_cmd(transition_profile=cfg.transition_profile)
    pin_start = float(ctx.sites_enu[start][1]) + tc.h_tc_cmd_m
    pin_end = float(ctx.sites_enu[site][1]) + tc.h_tc_cmd_m
    segs = assemble_2d(uav, seq, takeoff, landing, cache, ctx)
    e_start = model.eta_T * float(model.available_wh)
    wps, stats = build_3d(segs, uav, seq, ctx, dict(strips), float(t0_s), e_start, pin_start, pin_end)
    t_arr = next((w.t_s for w in wps if w.phase is Phase.TRANS_IN), None)
    if t_arr is None:
        t_arr = (wps[-1].t_s - spec.t_tr_in_s - spec.hover_descent_time_s()) if wps else float(t0_s)
    t_land = (wps[-1].t_s + spec.t_hover_reserve_s) if wps else float(t0_s)
    e_end = wps[-1].energy_remaining_wh - model.segment_wh(Mode.HOVER, spec.t_hover_reserve_s) if wps else e_start
    e_used = e_start - e_end
    dist = 0.0
    if len(wps) > 1:
        xy = np.array([[w.x_m, w.y_m] for w in wps])
        dist = float(np.hypot(*np.diff(xy, axis=0).T).sum())
    warnings = list(seq.warnings) + [f"profile:{v}" for v in stats.violations]
    plan = UAVPlan(uav.uav_id, cls, wps, [(it.strip_id, int(it.dir)) for it in seq.items], float(t0_s), float(t_arr),
                   float(t_land), site, float(e_used), float(e_end - model.reserve_wh()), dist,
                   float(model.cruise_equiv_km(e_used)), 0, warnings)
    return plan, stats


# ---------------------------------------------------------------------------
# 總流程（契約 §10 plan_mission；規格附錄流程）
# ---------------------------------------------------------------------------
def _start_site(ctx: MissionContext, uav_id: str) -> str:
    """該機**起飛**站點（`MissionContext.start_site_id`；舊版／手搭 context → home）。回收仍走 recovery（D40）。"""
    fn = getattr(ctx, "start_site_id", None)
    if fn is None:
        return ctx.inputs.home.site_id
    try:
        sid = str(fn(uav_id))
    except Exception:                                       # pragma: no cover - 手搭 context
        return ctx.inputs.home.site_id
    return sid if sid in ctx.sites_enu else ctx.inputs.home.site_id


def _start_xy(ctx: MissionContext, uav_id: str) -> np.ndarray:
    """該機起飛站點之 ENU xy（缺省 → `ctx.home_xy`）。"""
    sid = _start_site(ctx, uav_id)
    ent = ctx.sites_enu.get(sid)
    return ctx.home_xy if ent is None else np.asarray(ent[0], dtype=float)


def _nearest_entry(strips: Sequence[Strip], xy: np.ndarray) -> tuple[Pose, float, Strip, int]:
    best: tuple[float, Strip, int] | None = None
    for s in strips:
        for side, d in ((s.pL, -1), (s.pR, 1)):
            dist = float(math.hypot(side[0] - xy[0], side[1] - xy[1]))
            if best is None or dist < best[0]:
                best = (dist, s, d)
    assert best is not None
    _d, s, d = best
    return s.start_state(d), float(s.z_op_m), s, d


def _empty_plan(report: FeasibilityReport, timing: dict[str, float], cells: list[Cell] | None = None,
                uncovered: list[tuple[str, str]] | None = None) -> MissionPlan:
    return MissionPlan({}, 0.0, list(cells or []), {}, list(uncovered or []), report, timing, (0, 0, 0))


def plan_mission(inputs: MissionInputs, cfg: PlannerConfig = PlannerConfig(), *, terrain_margin_m: float | None = None,
                 ctx: MissionContext | None = None) -> MissionPlan:
    """完整流程（契約 §10 步驟）：build_context → preflight → decompose_area×區（含 residual_candidates）→ layering
    （不足時調整 h_op 並重鋪）→ 暫定起飛走廊（入口 = 各條帶端點中最近者）→ CostCache.build → allocate（含 assign_config_and_count、
    殘帶候選對、takeoff_schedule）→ beam_idp×團隊 → append_rtb／repair_energy → 起飛走廊精算（首條帶入口；改變則
    refresh_entry）→ assemble_2d → build_3d → deconflict（CorridorSchedule、spatially_exclusive、check_separation、repair）
    → MissionPlan（timing_ms 逐階段）。NO_GO → 空計畫（plans={}）。"""
    timing: dict[str, float] = {}
    t_all = time.perf_counter()
    tick = t_all

    def lap(name: str) -> None:
        nonlocal tick
        now = time.perf_counter()
        timing[name] = (now - tick) * 1e3
        tick = now

    # ---- 1. context + preflight ---------------------------------------------
    if ctx is None:
        kw = {} if terrain_margin_m is None else {"terrain_margin_m": float(terrain_margin_m)}
        try:
            ctx = build_context(inputs, cfg, **kw)
        except Exception as exc:  # MissionInputError
            lap("context")
            rep = FeasibilityReport("NO_GO", [f"build_context: {exc}"], [], {}, {}, {})
            timing["total"] = (time.perf_counter() - t_all) * 1e3
            return _empty_plan(rep, timing)
    cfg = ctx.cfg
    lap("context")
    report = preflight(inputs, cfg, ctx)
    lap("preflight")
    if report.status == "NO_GO":
        timing["total"] = (time.perf_counter() - t_all) * 1e3
        return _empty_plan(report, timing)
    dc = ctx.decompose_ctx()
    fleet = list(inputs.fleet)
    n_uav = {c: len(inputs.uavs_of_class(c)) for c in inputs.fleet_classes}
    uncovered: list[tuple[str, str]] = []
    warnings: list[str] = []

    # ---- 2. decompose + residual candidates -----------------------------------
    cells: list[Cell] = []
    residual_pairs: dict[str, list[tuple[int, Cell, Cell | None]]] = {}
    for area in inputs.areas:
        pa = report.per_area.get(area.area_id, {})
        classes = set(pa.get("feasible_classes") or inputs.fleet_classes) & set(inputs.fleet_classes)
        if not classes:
            uncovered.append((area.area_id, pa.get("uncoverable_reason") or "no_feasible_class"))
            continue
        poly = ctx.areas_enu.get(area.area_id)
        try:
            new = decompose_area(area, dc, classes, polygon_enu=poly, rho_sym_m=ctx.rho_sym_m, n_uav=n_uav)
        except Exception as exc:
            uncovered.append((area.area_id, f"decompose_failed({exc})"))
            continue
        cells += new
    lap("decompose")

    # ---- 3. layering（不足時調整 h_op 並重鋪）-----------------------------------
    bands: dict[str, dict[str, Any]] = {}
    for cell in cells:
        bands[cell.cell_id] = {}
        for c in ctx.specs:
            if cell.h_op_m.get(c) is not None:
                try:
                    bands[cell.cell_id][c] = altitude_band(cell.polygon, c, cell.payload_req_id, dc)
                except Exception:
                    pass
    lay = layering(cells, ctx.specs, bands, cfg)
    for cell in cells:
        res = lay.get(cell.cell_id, {})
        if res.get("mode") == "time_separation":
            warnings.append(f"time_separation({cell.cell_id})")
            continue
        for c, h in res.items():
            if not isinstance(h, (int, float)) or not cell.feasible.get(c, False):
                continue
            if abs(float(h) - float(cell.h_op_m.get(c, h))) > 1e-6:
                cell.h_op_m[c] = float(h)
                try:
                    new_strips = build_strips(cell, c, float(cell.alpha_rad[c]), dc)
                    if new_strips and all(s.feasible for s in new_strips):
                        cell.strips[c] = new_strips
                        cell.n_strips[c] = len(new_strips)
                        cell.T_single_s[c] = exact_time(new_strips, c, dc)
                        warnings.append(f"layering_relaid({cell.cell_id}.{c}, h_op={h:.1f})")
                except Exception as exc:
                    warnings.append(f"layering_relay_failed({cell.cell_id}.{c}: {exc})")
    for cell in cells:
        if cell.sensor_req == "any" and cell.feasible.get("A") and cell.feasible.get("B"):
            try:
                pairs = residual_candidates(cell, dc, rho_sym_m=ctx.rho_sym_m, n_uav=n_uav)
            except Exception:
                pairs = []
            if pairs:
                residual_pairs[cell.cell_id] = pairs
    lap("layering")

    strips: dict[str, dict[str, Strip]] = {c: {} for c in inputs.fleet_classes}
    strip_sensors: dict[str, Any] = {}
    for cell in cells:
        for c in strips:
            found = resolve_sensor(dc, c, cell.payload_req_id)
            for s in cell.strips.get(c, []):
                strips[c][s.strip_id] = s
                if found is not None:
                    strip_sensors[s.strip_id] = found[0]
    if not any(strips.values()):
        timing["total"] = (time.perf_counter() - t_all) * 1e3
        return _empty_plan(report, timing, cells, uncovered + [(c.cell_id, "no_strips") for c in cells])

    # ---- 4. 暫定起降走廊 --------------------------------------------------------
    home = inputs.home.site_id
    recovery = ctx.recovery_site_id
    start_of: dict[str, str] = {u.uav_id: _start_site(ctx, u.uav_id) for u in fleet}
    init_states: dict[str, UAVState] = {}
    init_laps: dict[str, tuple[int, float]] = {}
    takeoffs: dict[str, CorridorResult] = {}
    for u in fleet:
        cl = list(strips.get(u.cls, {}).values())
        if not cl:
            continue
        s_entry, z_entry, _s, _d = _nearest_entry(cl, _start_xy(ctx, u.uav_id))
        to = takeoff_corridor(start_of[u.uav_id], u, ctx, s_entry, z_entry, 0.0)
        takeoffs[u.uav_id] = to
        init_states[u.uav_id] = to.exit_state
        init_laps[u.uav_id] = (int(to.n_laps), 0.0)
    site_ids = [recovery]
    for c in ctx.specs:
        for s in report.derived.get(c, {}).get("usable_alternates", []) or []:
            if s not in site_ids and s in ctx.sites_enu:
                site_ids.append(s)
    sites: dict[str, dict[str, tuple[Pose, float]]] = {}
    landing_rep: dict[tuple[str, str], CorridorResult] = {}
    for sid in site_ids:
        sites[sid] = {}
        for c in ctx.specs:
            cl = list(strips.get(c, {}).values())
            us = [u for u in fleet if u.cls == c and u.uav_id in init_states]
            if not cl or not us:
                continue
            ld = landing_corridor(sid, us[0], ctx, cl[0].end_state(-1), float(cl[0].z_op_m))
            if ld.app_state is None:
                continue
            landing_rep[(c, sid)] = ld
            z_tc = float(ctx.sites_enu[sid][1]) + ctx.specs[c].transition_cmd(transition_profile=cfg.transition_profile).h_tc_cmd_m
            sites[sid][c] = (ld.app_state, z_tc)
    sites = {k: v for k, v in sites.items() if v}
    lap("corridors")

    # ---- 5. CostCache + allocate ---------------------------------------------
    cache = CostCache.build(strips, fleet, init_states, init_laps, sites, ctx, strip_sensors=strip_sensors)
    lap("cache")
    alloc = allocate(cells, strips, fleet, init_states, ctx.models, cache, ctx, cfg, residual_pairs=residual_pairs)
    uncovered += list(alloc.uncoverable)
    lap("allocate")

    # 使用之胞（含殘帶子胞）
    cells_by_id: dict[str, Cell] = {c.cell_id: c for c in cells}
    for pid, k in alloc.residual_k.items():
        for kk, ca, cb in residual_pairs.get(pid, []):
            if kk == k:
                for sub in (ca, cb):
                    if sub is not None:
                        cells_by_id.setdefault(sub.cell_id, sub)
    for tm in alloc.teams:
        for cid in tm.cell_ids:
            if cid not in cells_by_id:
                for lst in residual_pairs.values():
                    for _k, ca, cb in lst:
                        for sub in (ca, cb):
                            if sub is not None and sub.cell_id == cid:
                                cells_by_id[cid] = sub
    # 採用 k_A 之後，把未採用的候選條帶自快取移除（各候選之子胞／條帶 id 含 k，彼此不同）：
    # 其他 k 的 B 殘帶條帶、母胞未用之 B 條帶（k>0）與 A 條帶 k+1..n（k=0 → 全部）。
    # 否則 plan.strips 會含大量未飛的條帶，且（修正前）同名條帶互相覆蓋造成 B 機飛錯條帶、留下未掃的帶。
    for pid, k in alloc.residual_k.items():
        drop_b: list[str] = []
        drop_a: list[str] = []
        for kk, _ca, cb in residual_pairs.get(pid, []):
            if kk == k or cb is None or cb.cell_id == pid:
                continue
            drop_b += [s.strip_id for s in cb.strips.get("B", [])]
        parent = cells_by_id.get(pid)
        if parent is not None:
            if k > 0:
                drop_b += [s.strip_id for s in parent.strips.get("B", [])]
            drop_a += [s.strip_id for s in parent.strips.get("A", [])[k:]]
        used = {sid for tm in alloc.teams for sid in tm.strip_ids}
        drop_b = [s for s in dict.fromkeys(drop_b) if s not in used]
        drop_a = [s for s in dict.fromkeys(drop_a) if s not in used]
        if drop_b:
            cache.remove_strips("B", drop_b)
        if drop_a:
            cache.remove_strips("A", drop_a)
    cell_order: dict[str, list[str]] = {u: [k.split("#", 1)[0] for k in p] for u, p in alloc.plans.items()}

    # ---- 6. Beam-IDP ×團隊 + RTB + 能量修復 ------------------------------------
    results: dict[str, IDPResult] = {}
    t0_of: dict[str, float] = {}
    seqs: dict[str, UAVSequence] = {}
    for tm in alloc.teams:
        team_init: dict[str, UAVState] = {}
        for u in tm.uav_ids:
            if u not in init_states:
                continue
            t0 = float(tm.t0_s.get(u, 0.0))
            t0_of[u] = t0
            team_init[u] = replace(init_states[u], t_s=float(init_states[u].t_s) + t0)
        if not team_init:
            continue
        res = beam_idp(tm, cache.strips.get(tm.cls, {}), cache, ctx.models, team_init, cfg, cell_order=cell_order,
                       site_id=recovery)
        for _round in range(3):
            infeasible: dict[str, EnergyInfeasible] = {}
            for u, sq in res.sequences.items():
                out = append_rtb(sq, ctx, cache, ctx.models.get(u) or ctx.models_rep[tm.cls], cfg)
                if isinstance(out, EnergyInfeasible):
                    infeasible[u] = out
                else:
                    res.sequences[u] = out
            if not infeasible:
                break
            warnings.append("energy_repair(" + ",".join(f"{u}@{f.seg_index}" for u, f in infeasible.items()) + ")")
            res = repair_energy(tm, res, infeasible, cache, ctx.models, cfg, strips=cache.strips.get(tm.cls, {}))
        else:
            for u, sq in res.sequences.items():
                if sq.rtb is None and sq.items:
                    uncovered += [(it.strip_id, "energy_rtb_infeasible") for it in sq.items]
                    res.sequences[u] = replace(sq, items=[], decisions=[])
        results[tm.team_id] = res
        seqs.update(res.sequences)
    # 規格 §9.4 末「leftover → allocate(frozen)」再拍賣：剩餘條帶不直接判 uncovered，先以凍結既有團隊之
    # 二次配置找承接者（同構型、只允許尾端追加），成功者再以 warm-start 的 beam_idp 併回序列。
    leftover_now = {tid: list(dict.fromkeys(r.leftover)) for tid, r in results.items() if r.leftover}
    if leftover_now:
        try:
            placed = _reauction_leftover(leftover_now, results, alloc, cells_by_id, strips, cache, ctx, cfg,
                                         fleet, init_states, t0_of, seqs, recovery, cell_order, warnings)
        except Exception as exc:                            # pragma: no cover - 再拍賣失敗即維持原判定
            warnings.append(f"leftover_reauction_failed({exc.__class__.__name__}: {exc})")
            placed = set()
        if placed:
            warnings.append(f"leftover_reauctioned({len(placed)})")
    else:
        placed = set()
    for tid, res in results.items():
        for sid in leftover_now.get(tid, []):
            if sid in placed:
                continue
            uncovered.append((sid, "energy_leftover" if res.must_rtb else "idp_leftover"))
    lap("idp")

    # ---- 7. 起飛走廊精算 + 2D/3D 組裝 -------------------------------------------
    plans: dict[str, UAVPlan] = {}
    uav_by_id = {u.uav_id: u for u in fleet}
    strips_all: dict[str, Strip] = {sid: s for c in cache.strips for sid, s in cache.strips[c].items()}
    for u, sq in seqs.items():
        if not sq.items:
            continue
        uav = uav_by_id[u]
        cls = uav.cls
        t0 = t0_of.get(u, 0.0)
        first = sq.items[0]
        s0 = cache.strips[cls][first.strip_id]
        idx0 = (first.dir + 1) // 2
        entry_path = cache.entry_paths.get(u, {}).get((first.strip_id, idx0))
        to = takeoff_corridor(start_of.get(u, home), uav, ctx, s0.start_state(first.dir), float(s0.z_op_m), t0,
                              entry_path=entry_path)
        old = cache.init_states.get(u)
        ex = to.exit_state
        if old is None or math.hypot(ex.x_m - old.x_m, ex.y_m - old.y_m) > 1.0 or abs(_angdiff(ex.theta_rad, old.theta_rad)) > 1e-3 \
                or abs(ex.z_m - old.z_m) > 0.5:
            cache.refresh_entry(u, replace(ex, t_s=ex.t_s - t0), int(to.n_laps), 0.0, ctx)
            sq = _rebase_entry(sq, cache, cls, ex)
        else:
            sq = _rebase_entry(sq, cache, cls, ex, keep_entry=True)
        site = sq.landing_site or recovery
        ld = landing_corridor(site, uav, ctx, Pose(sq.state_end.x_m, sq.state_end.y_m, sq.state_end.theta_rad),
                              float(sq.state_end.z_m))
        if ld.app_state is None:
            ld = landing_rep.get((cls, site)) or ld
        plan, _stats = build_uav_plan(uav, sq, to, ld, cache, ctx, t0, strips_all)
        plan.warnings += [f"corridor:{i}" for i in (to.issues + ld.issues)]
        plans[u] = plan
        seqs[u] = sq
    lap("assemble_3d")

    # ---- 8. deconflict --------------------------------------------------------
    residual_conf: list = []
    if plans:
        schedule = CorridorSchedule(cfg)
        for p in plans.values():
            schedule.reserve_plan(p, start_of.get(p.uav_id, home))
        try:
            skip = spatially_exclusive(alloc.teams, cells_by_id, cfg, r_eff_m=dict(cache.R_eff))
        except Exception:
            skip = set()
        conflicts = check_separation(plans, cfg, skip_pairs=skip)
        if conflicts or schedule.conflicts():
            hook = _make_resequence_hook(ctx, cfg, cache, uav_by_id, alloc.teams, strips_all, t0_of, start_of,
                                         recovery, landing_rep, skip, cell_order, warnings)
            plans, residual_conf = repair(plans, conflicts, ctx, ctx.models, schedule, cfg, skip_pairs=skip,
                                          takeoff_site=start_of, resequence_hook=hook)
            if residual_conf:
                warnings.append(f"residual_conflicts({len(residual_conf)})")
    lap("deconflict")

    makespan = max((p.t_land_s for p in plans.values()), default=0.0)
    report.warnings = list(dict.fromkeys(list(report.warnings) + warnings))
    timing["total"] = (time.perf_counter() - t_all) * 1e3
    return MissionPlan(plans, float(makespan), list(cells_by_id.values()), strips_all,
                       list(dict.fromkeys(uncovered)), report, timing,
                       (int(cache.geometry_key), int(cache.wind_key), int(cache.state_key)))


def _reauction_leftover(leftover_now: Mapping[str, list[str]], results: dict[str, IDPResult],
                        alloc: AllocationResult, cells_by_id: Mapping[str, Cell],
                        strips: Mapping[str, dict[str, Strip]], cache: CostCache, ctx: MissionContext,
                        cfg: PlannerConfig, fleet: Sequence[UAVInstance], init_states: Mapping[str, UAVState],
                        t0_of: dict[str, float], seqs: dict[str, UAVSequence], recovery: str | None,
                        cell_order: dict[str, list[str]], warnings: list[str]) -> set[str]:
    """規格 §9.4 末：``leftover → allocate(frozen)`` 再拍賣（原本直接判 uncovered，D-PL-4）。

    1. 既有團隊以「**實際飛到的**條帶」凍結（`frozen_teams`），故 leftover 條帶不再被原任務佔住；
    2. 每個仍有 leftover 的胞複製出一個只含 leftover 條帶的合成胞（``<cell_id>~lo``、``sensor_req="any"``——
       只有原構型有該胞條帶，其餘構型 `score` 為 0，等同固定構型），整胞未飛者則直接把原胞放回池中；
    3. :func:`~hv_dccpp.allocate.allocate` 以凍結團隊二次拍賣（凍結機只允許尾端追加）；
    4. 對接到新條帶的團隊，以既有序列為 warm-start 重跑 :func:`beam_idp` 併回，`append_rtb` 不過即整團退回。

    回傳**確實排進序列**的條帶 id 集合；其餘仍由呼叫端記為 uncovered。起飛時刻沿用第一次配置之 t0
    （二次拍賣的 `takeoff_schedule` 結果不採用，以免動到已排定之走廊時槽）。
    """
    all_left: set[str] = {s for lst in leftover_now.values() for s in lst}
    if not all_left:
        return set()
    flown: dict[str, list[tuple[str, int]]] = {u: [(it.strip_id, int(it.dir)) for it in sq.items]
                                               for u, sq in seqs.items()}
    flown_ids: set[str] = {s for lst in flown.values() for s, _d in lst}
    team_by_id = {tm.team_id: tm for tm in alloc.teams}

    def _cell_of(sid: str, cls: str) -> Cell | None:
        st = cache.strips.get(cls, {}).get(sid)
        return None if st is None else cells_by_id.get(st.cell_id)

    frozen: list[Team] = []
    cells2: dict[str, Cell] = {}
    synth: list[Cell] = []
    for tid, lst in leftover_now.items():
        tm = team_by_id.get(tid)
        if tm is None:
            continue
        cls = tm.cls
        by_cell: dict[str, list[Strip]] = {}
        for sid in lst:
            st = cache.strips.get(cls, {}).get(sid)
            cell = _cell_of(sid, cls)
            if st is None or cell is None:
                continue
            by_cell.setdefault(cell.cell_id, []).append(st)
        keep_cells: list[str] = []
        for cid in tm.cell_ids:
            cell = cells_by_id.get(cid)
            if cell is None:
                continue
            own = [s.strip_id for s in cell.strips.get(cls, [])]
            if not any(s in flown_ids for s in own):
                cells2[cid] = cell                          # 整胞未飛 → 原胞放回池中重新拍賣
                continue
            keep_cells.append(cid)
            cells2[cid] = cell
            left = by_cell.get(cid) or []
            if left:
                sub = replace(cell, cell_id=f"{cid}~lo", strips={cls: list(left)}, sensor_req="any",
                              n_strips={cls: len(left)}, feasible={cls: True}, infeasible_reason={},
                              h_op_m=dict(cell.h_op_m), alpha_rad=dict(cell.alpha_rad), fit=dict(cell.fit),
                              d_turn_m=dict(cell.d_turn_m), sweep_origin=dict(cell.sweep_origin),
                              alt_dirs=dict(cell.alt_dirs), slope_exceeds=dict(cell.slope_exceeds),
                              T_single_s={cls: float(cell.T_single_s.get(cls, 0.0)) * len(left)
                                          / max(1, len(cell.strips.get(cls, [])))},
                              parent_cell_id=cid)
                synth.append(sub)
        flown_of_team = [s for u in tm.uav_ids for s, _d in flown.get(u, [])]
        frozen.append(Team(tm.team_id, cls, list(tm.uav_ids), keep_cells, flown_of_team, dict(tm.t0_s),
                           dict(tm.sortie_index)))
    for tm in alloc.teams:                                  # 未產生 leftover 之團隊也要凍結
        if tm.team_id in leftover_now:
            continue
        keep = [cid for cid in tm.cell_ids if cid in cells_by_id]
        for cid in keep:
            cells2[cid] = cells_by_id[cid]
        frozen.append(Team(tm.team_id, tm.cls, list(tm.uav_ids), keep,
                           [s for u in tm.uav_ids for s, _d in flown.get(u, [])], dict(tm.t0_s),
                           dict(tm.sortie_index)))
    if not synth and not any(cid for cid in cells2 if cid not in {c for tm in frozen for c in tm.cell_ids}):
        return set()
    cells_list = list(cells2.values()) + synth
    alloc2 = allocate(cells_list, {c: dict(v) for c, v in strips.items()}, list(fleet), dict(init_states),
                      ctx.models, cache, ctx, cfg, frozen_teams=frozen)
    order2 = {u: [k.split("#", 1)[0] for k in p] for u, p in alloc2.plans.items()}
    placed: set[str] = set()
    for tm2 in alloc2.teams:
        cls = tm2.cls
        gained = [s for s in tm2.strip_ids if s in all_left and s not in flown_ids]
        if not gained:
            continue
        base_ids = [s for u in tm2.uav_ids for s, _d in flown.get(u, [])]
        team3 = Team(tm2.team_id, cls, list(tm2.uav_ids), list(tm2.cell_ids), base_ids + gained,
                     dict(tm2.t0_s), dict(tm2.sortie_index))
        team_init: dict[str, UAVState] = {}
        warm: dict[str, list[tuple[str, int]]] = {}
        for u in tm2.uav_ids:
            st = init_states.get(u)
            if st is None:
                continue
            t0 = float(t0_of.get(u, float(tm2.t0_s.get(u, 0.0))))
            team_init[u] = replace(st, t_s=float(st.t_s) + t0)
            if flown.get(u):
                warm[u] = list(flown[u])
        if not team_init:
            continue
        res3 = beam_idp(team3, cache.strips.get(cls, {}), cache, ctx.models, team_init, cfg, warm_start=warm,
                        cell_order={**cell_order, **order2}, site_id=recovery)
        ok = True
        for u, sq in list(res3.sequences.items()):
            out = append_rtb(sq, ctx, cache, ctx.models.get(u) or ctx.models_rep[cls], cfg)
            if isinstance(out, EnergyInfeasible):
                ok = False
                break
            res3.sequences[u] = out
        got = [s for s in gained if s not in set(res3.leftover)]
        if not ok or not got:
            continue
        for u, sq in res3.sequences.items():
            if not {s for s, _d in flown.get(u, [])} <= {it.strip_id for it in sq.items}:
                ok = False                                  # 既有覆蓋不得減少
                break
        if not ok:
            continue
        placed.update(got)
        results[tm2.team_id] = res3
        seqs.update(res3.sequences)
        cell_order.update(order2)
        for u in tm2.uav_ids:
            t0_of.setdefault(u, float(tm2.t0_s.get(u, 0.0)))
    if placed:
        warnings.append("leftover_placed(" + ",".join(sorted(placed)[:8]) + ")")
    return placed


def _make_resequence_hook(ctx: MissionContext, cfg: PlannerConfig, cache: CostCache,
                          uav_by_id: Mapping[str, UAVInstance], teams: Sequence[Team],
                          strips_all: Mapping[str, Strip], t0_of: Mapping[str, float],
                          start_of: Mapping[str, str], recovery: str | None,
                          landing_rep: Mapping[tuple[str, str], CorridorResult],
                          skip: set[tuple[str, str]], cell_order: Mapping[str, Sequence[str]],
                          warnings: list[str]):
    """建立規格 §13.6 步 (iii)「IDP 重排」之 ``resequence_hook``（`deconflict.ResequenceRequest` → 新 UAVPlan）。

    作法：對衝突涉及之機 ``u``（`req.uav_ids[0]`），取其**當前計畫**之條帶序（`req.plans[u].strips`，
    修復已改過者亦以此為準），在衝突時刻所在的條帶處切開；前綴固定，後綴放回 NAD 讓 :func:`beam_idp`
    重排。為了得到與現況不同的解，逐一以「翻轉該條帶掃向」或「先飛後綴中的另一條」作為切點後的**強制
    首選**（規格 §13.6 (iii)：交換條帶方向／改同向轉場為零成本選項），同隊其他機以其現有序列 warm-start
    後即 RTB（`inactive`），故只有 ``u`` 的計畫改變。

    每個候選都重跑 `append_rtb` → 起降走廊 → :func:`build_uav_plan`，再以 ``req.energy_check`` 與
    `deconflict.check_separation(only_uavs={u})` 評分，取「涉及 u 之衝突數 → ΣΔ_req → 降落走廊是否淨空 →
    落地時刻」最小者；沒有嚴格優於現況者回傳 ``None``。切點一律 ≥ 1，故首條帶（決定起飛走廊出口與進入段）
    不變，不需 `cache.refresh_entry`（避免污染共享快取）。航點 z 由 `build_3d` 重算，未被本回呼直接改動（D18）。

    預算：整個 `plan_mission` 至多 :data:`_RESEQ_MAX_CALLS` 次、:data:`_RESEQ_TIME_BUDGET_S` 秒。
    """
    from .deconflict import check_separation as _chk, plan_windows as _win
    team_of: dict[str, Team] = {}
    for tm in teams:
        for u in tm.uav_ids:
            team_of.setdefault(u, tm)
    budget = {"calls": 0, "t": 0.0, "hits": 0}

    def _score(plans: Mapping[str, UAVPlan], u: str, sched, t_sync: float) -> tuple[int, float, int, float]:
        conf = _chk(dict(plans), cfg, skip_pairs=skip, t_sync_s=t_sync, only_uavs={u})
        p = plans[u]
        _to, ld = _win(p, cfg)
        busy = 0 if sched is None or sched.is_free(p.landing_site, ld[0], ld[1], exclude_uav=u) else 1
        return (len(conf), float(sum(c.dt_req_s for c in conf)), busy, float(p.t_land_s))

    def _rebuild(u: str, seq: UAVSequence, t0: float) -> UAVPlan | None:
        uav = uav_by_id[u]
        cls = uav.cls
        first = seq.items[0]
        s0 = cache.strips[cls][first.strip_id]
        idx0 = (first.dir + 1) // 2
        entry_path = cache.entry_paths.get(u, {}).get((first.strip_id, idx0))
        to = takeoff_corridor(start_of.get(u, ctx.inputs.home.site_id), uav, ctx, s0.start_state(first.dir),
                              float(s0.z_op_m), t0, entry_path=entry_path)
        sq = _rebase_entry(seq, cache, cls, to.exit_state, keep_entry=True)
        site = sq.landing_site or recovery
        ld = landing_corridor(site, uav, ctx, Pose(sq.state_end.x_m, sq.state_end.y_m, sq.state_end.theta_rad),
                              float(sq.state_end.z_m))
        if ld.app_state is None:
            ld = landing_rep.get((cls, site)) or ld
        plan, _stats = build_uav_plan(uav, sq, to, ld, cache, ctx, t0, strips_all)
        plan.warnings = list(plan.warnings) + [f"corridor:{i}" for i in (to.issues + ld.issues)] + ["resequenced"]
        return plan

    def hook(req) -> dict[str, UAVPlan] | None:
        if budget["calls"] >= _RESEQ_MAX_CALLS or budget["t"] >= _RESEQ_TIME_BUDGET_S:
            return None
        t_begin = time.perf_counter()
        budget["deadline"] = t_begin + max(0.5, _RESEQ_TIME_BUDGET_S - budget["t"])
        try:
            out = _resequence_one(req)
        except Exception as exc:                            # pragma: no cover - 盡力而為，失敗即留殘餘
            warnings.append(f"resequence_failed({exc.__class__.__name__})")
            out = None
        budget["calls"] += 1
        budget["t"] += time.perf_counter() - t_begin
        if out is not None:
            budget["hits"] += 1
            warnings.append(f"resequenced({','.join(sorted(out))})")
        return out

    def _resequence_one(req) -> dict[str, UAVPlan] | None:
        for u in req.uav_ids:
            tm = team_of.get(u)
            if tm is None or u not in req.plans or u not in cache.entry:
                continue
            cur = list(req.plans[u].strips)
            if len(cur) < 3:
                continue
            t_ref = float(req.conflict.t_s) + (0.0 if u == req.conflict.uav_a else float(req.conflict.dt_ab_s))
            cut = _cut_index(req.plans[u], t_ref)
            if cut is None or cut >= len(cur):
                continue
            cls = tm.cls
            t0 = float(req.plans[u].t_takeoff_s)
            base_init = cache.init_states
            team_init: dict[str, UAVState] = {}
            for v in tm.uav_ids:
                st = base_init.get(v)
                if st is None:
                    continue
                team_init[v] = replace(st, t_s=float(st.t_s) + (t0 if v == u else float(t0_of.get(v, 0.0))))
            if u not in team_init:
                continue
            warm_others = {v: [(s, int(d)) for s, d in req.plans[v].strips] for v in tm.uav_ids
                           if v != u and v in req.plans}
            inactive = [v for v in warm_others if v in team_init]
            want = {s for s, _d in cur}
            model = req.model_of(u) or ctx.models_rep[cls]
            base = _score(req.plans, u, req.schedule, float(req.t_sync_s))
            if not _reseq_worth_trying(base):
                continue        # 該衝突在本輪稍早之修復後已消失（repair 逐筆處理的是本輪起始清單）→ 無事可修
            best: tuple[tuple[int, float, int, float], UAVPlan] | None = None
            for sid, d in _reseq_candidates(cur, cut):
                warm = dict(warm_others)
                warm[u] = [(s, int(dd)) for s, dd in cur[:cut]] + [(sid, int(d))]
                res = beam_idp(tm, cache.strips.get(cls, {}), cache, ctx.models, team_init, cfg,
                               warm_start=warm, cell_order=dict(cell_order), inactive=inactive, site_id=recovery)
                sq = res.sequences.get(u)
                if sq is None or not sq.items or {it.strip_id for it in sq.items} != want:
                    continue                                # 覆蓋不得減少
                out = append_rtb(sq, ctx, cache, model, cfg)
                if isinstance(out, EnergyInfeasible):
                    continue
                plan = _rebuild(u, out, t0)
                if plan is None or not req.energy_check(plan, model):
                    continue
                sc = _score({**req.plans, u: plan}, u, req.schedule, float(req.t_sync_s))
                if best is None or sc < best[0]:
                    best = (sc, plan)
                if time.perf_counter() > float(budget.get("deadline", math.inf)):
                    break
            if best is not None and _reseq_better(best[0], base):
                return {u: best[1]}
        return None

    return hook


def _reseq_worth_trying(base: tuple[int, float, int, float]) -> bool:
    """現況是否還有東西可修：`deconflict.repair` 逐筆處理的是**本輪起始**的衝突清單，該筆衝突常在本輪稍早
    的修復後就已消失；此時涉及該機的衝突數為 0 且降落時槽淨空，跑 IDP 重排純屬浪費（山丘實測 19 次呼叫
    全屬此類，省下約 3.5 s）。"""
    return base[0] > 0 or base[2] > 0


def _reseq_better(new: tuple[int, float, int, float], base: tuple[int, float, int, float]) -> bool:
    """重排候選之採用條件（保守）：**真的少掉一筆衝突**（或同數但降落時槽由佔用變淨空），且**落地不變晚**。

    只要求「字典序較小」是不夠的：`deconflict.repair` 是貪婪搜尋，把某機的落地往後挪換掉一筆衝突，
    往往在後續輪次引發更多起飛延遲（山丘實測 makespan 5087 s → 5657 s）。故此處額外要求 t_land 不增加。
    """
    if float(new[3]) > float(base[3]) + 1e-6:
        return False
    if new[0] < base[0]:
        return True
    return new[0] == base[0] and new[2] < base[2] and new[1] <= base[1] + 1e-6


def _cut_index(plan: UAVPlan, t_ref_s: float) -> int | None:
    """衝突時刻所在（或其後）之條帶在 ``plan.strips`` 中的索引；一律 ≥ 1（首條帶不動）。"""
    order = [s for s, _d in plan.strips]
    pos = {s: i for i, s in enumerate(order)}
    best: int | None = None
    for w in plan.waypoints:
        if w.strip_id is None or w.strip_id not in pos:
            continue
        if float(w.t_s) >= t_ref_s:
            best = pos[w.strip_id]
            break
        best = pos[w.strip_id]
    if best is None:
        return None
    return max(1, int(best))


def _reseq_candidates(cur: Sequence[tuple[str, int]], cut: int) -> list[tuple[str, int]]:
    """切點之後的「強制首選」候選（規格 §13.6 (iii)）：先翻轉原本要飛的那條之掃向，再試後綴中其他條帶之兩向。"""
    tail = list(cur[cut:])
    if not tail:
        return []
    out: list[tuple[str, int]] = [(tail[0][0], -int(tail[0][1]))]
    for sid, d in tail[1:]:
        out.append((sid, int(d)))
        out.append((sid, -int(d)))
        if len(out) >= _RESEQ_MAX_CANDIDATES:
            break
    return out[:_RESEQ_MAX_CANDIDATES]


def _rebase_entry(seq: UAVSequence, cache: CostCache, cls: str, ex: UAVState, keep_entry: bool = False) -> UAVSequence:
    """以精算後之起飛走廊出口重定序列時間：entry 取自（已 refresh 之）快取列，後續項平移 Δt、能量平移 Δe。"""
    if not seq.items:
        return seq
    first = seq.items[0]
    l = cache.idx[cls][first.strip_id]
    idx0 = (first.dir + 1) // 2
    u = seq.uav_id
    if keep_entry:
        t_ent, e_ent = float(seq.entry.t_s), float(seq.entry.e_wh)
    else:
        t_ent = float(cache.entry[u][l, idx0])
        e_ent = float(cache.entry_e[u][l, idx0])
        if not math.isfinite(t_ent):
            t_ent, e_ent = float(seq.entry.t_s), float(seq.entry.e_wh)
    new_start = float(ex.t_s) + t_ent
    dt = new_start - float(first.t_start_s)
    # e_end 之首項 = E_exit − e_entry − e_oper：以差值平移所有項
    e_op0 = float(cache.op_e[cls][l, idx0]) if cls in cache.op_e else 0.0
    de = (float(ex.e_rem_wh) - e_ent - e_op0) - float(first.e_end_wh)
    items = [SeqItem(it.strip_id, it.dir, it.t_start_s + dt, it.t_end_s + dt, it.e_end_wh + de, it.transfer) for it in seq.items]
    path = cache.entry_paths.get(u, {}).get((first.strip_id, idx0)) if not keep_entry else seq.entry.path
    if path is None:
        path = seq.entry.path
    entry = PathCost(t_ent, e_ent, float(cache.entry_len[u][l, idx0]) if u in cache.entry_len else seq.entry.length_m,
                     path, int(cache.entry_laps[u][l, idx0]) if u in cache.entry_laps else seq.entry.n_laps, seq.entry.dz_m)
    st_end = replace(seq.state_end, t_s=seq.state_end.t_s + dt, e_rem_wh=seq.state_end.e_rem_wh + de)
    return replace(seq, items=items, entry=entry, state_end=st_end, t_end_s=seq.t_end_s + dt, e_end_wh=seq.e_end_wh + de)
