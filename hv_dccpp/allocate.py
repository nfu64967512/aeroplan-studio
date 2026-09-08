"""hv_dccpp/allocate.py — 任務配置（契約 §9；規格 §9.1–§9.7、§7.6、D16、D17、D40）。

* 第 4 層：只依賴 costs / decompose / context / mission / geometry / dubins / platform（不得 import planner 以上）。
* 成本一律為時間（s）；目標 = min-max 落地時刻（含返航與降落固定時間 117/106 s）。
* 配置單位 = 「任務」(cell, cls)：一般胞一個任務；``sensor_req == "both"`` 之胞為 A、B 兩套獨立條帶 → 兩個任務
  （鍵 ``f"{cell_id}#{cls}"``）；``any`` 胞之殘帶候選對 (k_A, A 子胞, B 殘帶) 於拍賣階段同時評估（規格 §9.5）。
* 每機計畫 ``plan[i]`` = 任務鍵序列；λ_q = 含任務 q 之機數；估計 Ĉ_i 以 :func:`makespan_after_insert`
  （進入兩側擇優 + ``T_single/λ`` + 胞間 Dubins + 依終端側之 RTB + 降落固定時間）；終端側由條帶數奇偶與進入側決定。
* 團隊 = 同構型 (機, 任務) 二部圖之連通分量（規格 §9.4 Phase 5 合併）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Literal, Mapping, Sequence

import numpy as np
import shapely
from shapely.geometry import Point

from .context import MissionContext
from .costs import CostCache
from .decompose import _skip_sequence, class_can_serve, exact_time, sensor_compatible
from .dubins import skip_k
from .geometry import Cell, Strip
from .mission import PlannerConfig, UAVInstance, UAVState
from .platform import EnergyModel, PlatformSpec

__all__ = [
    "Team", "AllocationResult", "CountResult", "hard_feasible", "config_index_J", "assign_config_and_count",
    "makespan_after_insert", "makespan_after_insert_pair", "allocate", "split_for_sorties", "takeoff_schedule",
    "task_key", "plan_estimate", "PlanEstimate", "sweep_order", "BOTH_SEP", "SORTIE_SEP", "SECOND_SENSOR_TAG",
]

BOTH_SEP: str = "#"            # both 胞任務鍵：f"{cell_id}#{cls}"
SORTIE_SEP: str = "#s"         # 多架次虛擬機 id：f"{uav_id}#s{n}"（n ≥ 1）
SECOND_SENSOR_TAG: str = "sensor2"   # R-04：both 胞只有單一構型可服務時之「第二套條帶未鋪」標記鍵尾
_BAND_REASONS = frozenset({"band_lt_min", "A_needs_waiver", "gsd_unattainable", "terrain_unknown"})
_INF = math.inf
_EPS = 1e-9


# ---------------------------------------------------------------------------
# 契約資料型別
# ---------------------------------------------------------------------------
@dataclass(slots=True)
class Team:
    """團隊（契約 §9）：同構型機群、其胞序列與條帶集合、每機起飛時刻與架次序號。"""
    team_id: str
    cls: str
    uav_ids: list[str]
    cell_ids: list[str]
    strip_ids: list[str]
    t0_s: dict[str, float]
    sortie_index: dict[str, int]


@dataclass(slots=True)
class AllocationResult:
    """配置結果（契約 §9）。尾端額外欄位（契約未列）：``residual_k``（any 胞 → 採用之 k_A）、``plans``
    （機 id → 任務鍵序列；多架次虛擬機以 ``f"{uav}#s{n}"`` 為鍵）、``delays_s``（H3 延後起飛 Δ_i）。"""
    teams: list[Team]
    uncoverable: list[tuple[str, str]]
    leftover_strips: list[str]
    lambda_by_cell: dict[str, tuple[str, int]]
    estimates: dict[str, float]
    residual_k: dict[str, int] = field(default_factory=dict)
    plans: dict[str, list[str]] = field(default_factory=dict)
    delays_s: dict[str, float] = field(default_factory=dict)


@dataclass(slots=True)
class CountResult:
    """規格 §9.3 結果：``by_cell`` 任務鍵 → (cls, λ)；``partial`` 任務鍵 → 因能量只取前 k 條後之 UNCOVERABLE 條帶；
    ``infeasible`` 任務鍵 → 原因。額外：``J``（任務鍵 → J^{c*}_q）、``second_choice``（任務鍵 → c2）。"""
    by_cell: dict[str, tuple[str, int]]
    partial: dict[str, list[str]]
    infeasible: dict[str, str]
    J: dict[str, float] = field(default_factory=dict)
    second_choice: dict[str, str | None] = field(default_factory=dict)


@dataclass(slots=True)
class PlanEstimate:
    """單機計畫估計（:func:`plan_estimate`）：落地時刻 C_s、能量 e_wh（含 e^TO 與 RTB+e^LDG）、H3 延後起飛 Δ、
    每任務 (first_l, idx_in, last_l, idx_last, t_arr, t_done)、可行性與原因。"""
    C_s: float
    e_wh: float
    delay_s: float
    ok: bool
    reason: str
    per_task: list[tuple[int, int, int, int, float, float]]


# ---------------------------------------------------------------------------
# 內部：任務
# ---------------------------------------------------------------------------
def task_key(cell: Cell | str, cls: str, sensor_req: str | None = None) -> str:
    """任務鍵：一般胞 = cell_id；``both`` 胞 = ``f"{cell_id}#{cls}"``。"""
    if isinstance(cell, Cell):
        cid, req = cell.cell_id, cell.sensor_req
    else:
        cid, req = str(cell), (sensor_req or "")
    return f"{cid}{BOTH_SEP}{cls}" if req == "both" else cid


@dataclass(slots=True)
class _Task:
    key: str
    cell: Cell
    cls: str
    strip_ids: list[str]
    T_single_s: float
    fit: float
    priority: float
    time_window_s: tuple[float, float] | None
    lam_floor: int = 0
    J: float = 0.0
    parent: str | None = None          # any 胞候選對之母胞 id
    k_A: int | None = None


def _task_of(cell: Cell, cls: str, strip_ids: Sequence[str] | None = None, T_single_s: float | None = None,
             *, parent: str | None = None, k_A: int | None = None, key: str | None = None) -> _Task:
    sids = list(strip_ids) if strip_ids is not None else [s.strip_id for s in cell.strips.get(cls, [])]
    T = float(cell.T_single_s.get(cls, _INF)) if T_single_s is None else float(T_single_s)
    return _Task(key=key or task_key(cell, cls), cell=cell, cls=cls, strip_ids=sids, T_single_s=T,
                 fit=float(cell.fit.get(cls, 0.0)), priority=float(cell.priority), time_window_s=cell.time_window_s,
                 parent=parent, k_A=k_A)


def _pair_key(pid: str, parent: Cell, cell: Cell, cls: str, k: int) -> str:
    """殘帶候選對之任務鍵：母胞本身（k = n_A 之 A 獨佔／k = 0 之 B 獨佔）→ ``pid#cls``；子胞 → ``cell_id#k{k}``
    （decompose 之子胞 id 已含 k：``{pid}_A_k{k}``／``{pid}_B_k{k}``；鍵再附 k 以保險）。"""
    if cell is parent:
        return f"{pid}{BOTH_SEP}{cls}"
    return f"{cell.cell_id}{BOTH_SEP}k{int(k)}"


def _base_uav(uav_id: str) -> str:
    return uav_id.split(SORTIE_SEP, 1)[0]


def _cls_of(cache: CostCache, uav: UAVInstance | str) -> str:
    if isinstance(uav, UAVInstance):
        return uav.cls
    return cache.uav_cls[_base_uav(uav)]


def _spec_of(cache: CostCache, cls: str, uav: UAVInstance | None = None) -> PlatformSpec:
    specs = getattr(cache.budget, "specs", None) or {}
    if cls in specs:
        return specs[cls]
    if uav is not None:
        return uav.spec
    raise KeyError(f"no spec for class {cls!r}")


def _landing_fixed_time_s(spec: PlatformSpec) -> float:
    """降落固定時間 t_acc + t_tr^in + t_dn + t_hov,res（A 117 s、B 106 s；規格 §5.3／§W.4）。"""
    tc = spec.transition_cmd()
    return float(tc.t_acc_s + spec.t_tr_in_s + spec.hover_descent_time_s() + spec.t_hover_reserve_s)


def _resolve_site(cache: CostCache, cls: str, recovery_site_id: str | None) -> str | None:
    """RTB 站點（D40：一律先以回收點 H 計）；缺省 = cache.sites 第一鍵；該構型無此站之列 → 任一有列之站。"""
    cands = [s for (c, s) in cache.rtb if c == cls]
    if not cands:
        return None
    if recovery_site_id is not None and (cls, recovery_site_id) in cache.rtb:
        return recovery_site_id
    for s in cache.sites:
        if (cls, s) in cache.rtb:
            return s
    return cands[0]


def sweep_order(strip_ids: Sequence[str], cls: str, cache: CostCache) -> list[int]:
    """估計用固定 skip-k 掃描序（規格 §7.1／§9.4）：``_skip_sequence(n, skip_k(s̄, R_eff))`` 映射為 cache 索引；
    首條 = 進入條帶、末條 = RTB 條帶。條帶不在 cache → KeyError。"""
    try:
        idxs = [cache.idx[cls][sid] for sid in strip_ids]
    except KeyError as e:
        raise KeyError(f"strip {e} is not in the CostCache (call cache.add_strips first)") from e
    n = len(idxs)
    if n == 0:
        return []
    strips = [cache.strips[cls][sid] for sid in strip_ids]
    s_mean = float(np.mean([s.spacing_m for s in strips]))
    k = skip_k(s_mean, cache.R_eff.get(cls, s_mean)) if s_mean > 0 else 1
    return [idxs[p] for p in _skip_sequence(n, k)]


def _task_options(task: _Task, cache: CostCache) -> list[tuple[int, int, int, int]]:
    """任務之 (first_l, idx_in, last_l, idx_last) 選項：固定 skip-k 序列之正／反向 × 進入側；終端側由奇偶決定
    （n 奇 → idx_last = idx_in；n 偶 → 1 − idx_in）；終端向不可行（單向胞，D34 racetrack）→ 改可行向。"""
    cls = task.cls
    order = sweep_order(task.strip_ids, cls, cache)
    n = len(order)
    if n == 0:
        return []
    op = cache.op[cls]
    opts: list[tuple[int, int, int, int]] = []
    for first, last in ((order[0], order[-1]), (order[-1], order[0])):
        for idx_in in (0, 1):
            if not math.isfinite(op[first, idx_in]):
                continue
            idx_last = idx_in if n % 2 == 1 else 1 - idx_in
            if not math.isfinite(op[last, idx_last]):
                if math.isfinite(op[last, 1 - idx_last]):
                    idx_last = 1 - idx_last
                else:
                    continue
            o = (int(first), int(idx_in), int(last), int(idx_last))
            if o not in opts:
                opts.append(o)
    return opts


def _chain(tasks: Sequence[_Task], uav_id: str, cls: str, cache: CostCache, cfg: PlannerConfig, *,
           t0_s: float | None, recovery_site_id: str | None, t_window: tuple[float, float] | None,
           lam_of: Mapping[str, int] | None, model: EnergyModel | None, spec: PlatformSpec | None = None) -> PlanEstimate:
    """規格 §9.4 MAKESPAN_AFTER_INSERT 之鏈式估計（各胞 4 選項之動態規劃；時間最小、tie 能量最小）。"""
    base = _base_uav(uav_id)
    if not tasks:
        return PlanEstimate(0.0, 0.0, 0.0, True, "", [])
    if base not in cache.entry:
        return PlanEstimate(_INF, _INF, 0.0, False, "no_entry_rows", [])
    entry, entry_e = cache.entry[base], cache.entry_e[base]
    tr, tr_e = cache.tr[cls], cache.tr_e[cls]
    site = _resolve_site(cache, cls, recovery_site_id)
    rtb = cache.rtb.get((cls, site)) if site is not None else None
    if t0_s is None:
        st = cache.init_states.get(base)
        t0_s = float(st.t_s) if st is not None else 0.0
    if spec is None:
        spec = _spec_of(cache, cls)
    t_land = _landing_fixed_time_s(spec)
    enforce_tw = bool(getattr(cfg, "enforce_time_windows", True))
    p_wait = None
    if model is not None:
        p_wait = model
    # DP 狀態：(last_l, idx_last) → (t, e, delay, trace)
    states: dict[tuple[int, int] | None, tuple[float, float, float, list]] = {None: (float(t0_s), 0.0, 0.0, [])}
    for j, task in enumerate(tasks):
        opts = _task_options(task, cache)
        if not opts:
            return PlanEstimate(_INF, _INF, 0.0, False, f"no_option({task.key})", [])
        lam = max(1, int((lam_of or {}).get(task.key, 1)))
        T_task = task.T_single_s / lam
        if not math.isfinite(T_task):
            return PlanEstimate(_INF, _INF, 0.0, False, f"T_single_inf({task.key})", [])
        e_oper = model.transfer_wh(T_task, 0.0) if model is not None else 0.0
        new_states: dict[tuple[int, int] | None, tuple[float, float, float, list]] = {}
        for prev, (t, e, delay, trace) in states.items():
            for (first, idx_in, last, idx_last) in opts:
                if prev is None:
                    dt, de = float(entry[first, idx_in]), float(entry_e[first, idx_in])
                else:
                    kk = 2 * prev[1] + idx_in
                    dt, de = float(tr[prev[0], first, kk]), float(tr_e[prev[0], first, kk])
                if not math.isfinite(dt):
                    continue
                t_arr = t + dt
                e_new = e + de
                d_new = delay
                if enforce_tw and task.time_window_s is not None:
                    t_open, t_close = task.time_window_s
                    if t_arr < t_open:
                        wait = t_open - t_arr
                        if j == 0:
                            d_new += wait                        # 延後起飛（零能量）
                        elif p_wait is not None:
                            e_new += p_wait.transfer_wh(wait, 0.0)   # 等待盤旋計能量
                        t_arr = t_open
                    if t_arr + T_task > t_close + _EPS:
                        continue
                t_done = t_arr + T_task
                e_new += e_oper
                key = (last, idx_last)
                cand = (t_done, e_new, d_new, trace + [(first, idx_in, last, idx_last, t_arr, t_done)])
                cur = new_states.get(key)
                if cur is None or (cand[0], cand[1]) < (cur[0], cur[1]):
                    new_states[key] = cand
        if not new_states:
            return PlanEstimate(_INF, _INF, 0.0, False, f"infeasible_transfer_or_window({task.key})", [])
        states = new_states
    best: tuple[float, float, float, list] | None = None
    for (last, idx_last), (t, e, delay, trace) in states.items():
        if rtb is None:
            t_rtb, e_rtb = _INF, _INF
        else:
            t_rtb = float(rtb[last, idx_last])
            sid = cache.strip_ids[cls][last]
            try:
                e_rtb = cache.budget.rtb_wh(cls, sid, -1 if idx_last == 0 else 1, site)
            except KeyError:
                e_rtb = _INF
        if not math.isfinite(t_rtb):
            continue
        C = t + t_rtb + t_land
        cand = (C, e + e_rtb, delay, trace)
        if best is None or (cand[0], cand[1]) < (best[0], best[1]):
            best = cand
    if best is None:
        return PlanEstimate(_INF, _INF, 0.0, False, "rtb_infeasible", [])
    C, e_tot, delay, trace = best
    if enforce_tw and t_window is not None and C > t_window[1] + _EPS:
        return PlanEstimate(C, e_tot, delay, False, "airspace_window", trace)
    if model is not None:
        e_tot += model.e_takeoff_wh()
    return PlanEstimate(C, e_tot, delay, True, "", trace)


def _energy_ok(est: PlanEstimate, model: EnergyModel | None, cfg: PlannerConfig) -> bool:
    """H4（規格 §9.1）：e^TO + e^in + Σ e^oper/λ + e^trans + e^RTB(→H, 含 e^LDG) ≤ η_T E^avail − r E_nom。"""
    if model is None or not getattr(cfg, "enforce_energy", True):
        return True
    if not math.isfinite(est.e_wh):
        return False
    budget = model.eta_T * float(model.available_wh) - model.reserve_wh()
    return est.e_wh <= budget + _EPS


# ---------------------------------------------------------------------------
# 公開：計畫估計與 makespan
# ---------------------------------------------------------------------------
def _resolve_plan(plan: Sequence[Any], cls: str, cells: Mapping[str, Cell] | None,
                  tasks: Mapping[str, _Task] | None = None) -> list[_Task]:
    out: list[_Task] = []
    for item in plan:
        if isinstance(item, _Task):
            out.append(item)
        elif isinstance(item, Cell):
            out.append(_task_of(item, cls))
        else:
            k = str(item)
            if tasks is not None and k in tasks:
                out.append(tasks[k])
            elif cells is not None and k in cells:
                out.append(_task_of(cells[k], cls))
            elif cells is not None and k.split(BOTH_SEP, 1)[0] in cells:
                out.append(_task_of(cells[k.split(BOTH_SEP, 1)[0]], cls))
            else:
                raise KeyError(f"plan item {k!r} unknown: pass cells={{cell_id: Cell}}")
    return out


def plan_estimate(plan: Sequence[Any], uav: UAVInstance | str, cache: CostCache, cfg: PlannerConfig, *,
                  cells: Mapping[str, Cell] | None = None, t0_s: float | None = None, recovery_site_id: str | None = None,
                  t_window: tuple[float, float] | None = None, lam_by_cell: Mapping[str, int] | None = None,
                  model: EnergyModel | None = None) -> PlanEstimate:
    """單機計畫（胞序列）之落地時刻／能量估計（規格 §9.4 MAKESPAN_AFTER_INSERT；契約 §W.4）。

    ``plan`` 元素可為 Cell 或 cell_id（後者需 ``cells``）。``t0_s`` None → ``cache.init_states[uav].t_s``
    （已含起飛 57/51 s）。``lam_by_cell``：任務鍵 → λ（T_single/λ）。``model`` 給定時含能量估計。"""
    uav_id = uav.uav_id if isinstance(uav, UAVInstance) else str(uav)
    cls = _cls_of(cache, uav)
    tasks = _resolve_plan(plan, cls, cells)
    spec = uav.spec if isinstance(uav, UAVInstance) and cls not in (getattr(cache.budget, "specs", None) or {}) else None
    return _chain(tasks, uav_id, cls, cache, cfg, t0_s=t0_s, recovery_site_id=recovery_site_id, t_window=t_window,
                  lam_of=lam_by_cell, model=model, spec=spec)


def makespan_after_insert(plan_i: list[str], pos: int, cell: Cell, uav: UAVInstance, cache: CostCache, cfg: PlannerConfig,
                          *, cells: Mapping[str, Cell] | None = None, t0_s: float | None = None,
                          recovery_site_id: str | None = None, t_window: tuple[float, float] | None = None,
                          lam_by_cell: Mapping[str, int] | None = None, model: EnergyModel | None = None) -> float:
    """規格 §9.4：胞 ``cell`` 插入 ``plan_i[pos]`` 後該機之落地時刻 Ĉ_i（s；不可行 → inf）。

    = t0（起飛走廊出口時刻）+ 進入 ``entry[l, idx]``（兩側擇優、序列正反向擇優）+ Σ T_single/λ + 胞間 Dubins
    （``tr[cls][last, first, k]``）+ ``rtb[cls, H][last, idx_last]``（**終端側由條帶數奇偶與進入側決定**：n 奇 →
    idx_last = idx_in、n 偶 → 1 − idx_in）+ 降落固定時間（A 117 s／B 106 s）。``plan_i`` 為 cell_id 序列
    （需 ``cells``）或 Cell 序列。"""
    cls = uav.cls
    plan = list(plan_i)
    pos = max(0, min(int(pos), len(plan)))
    plan.insert(pos, cell)
    tasks = _resolve_plan(plan, cls, cells)
    est = _chain(tasks, uav.uav_id, cls, cache, cfg, t0_s=t0_s, recovery_site_id=recovery_site_id, t_window=t_window,
                 lam_of=lam_by_cell, model=model)
    return est.C_s if est.ok else _INF


def makespan_after_insert_pair(plans: Mapping[str, list[str]], uav_A: UAVInstance, pos_A: int, cell_A: Cell,
                               uav_B: UAVInstance, pos_B: int, cell_B: Cell | None, cache: CostCache, cfg: PlannerConfig,
                               *, cells: Mapping[str, Cell] | None = None, recovery_site_id: str | None = None,
                               t_window: tuple[float, float] | None = None, lam_by_cell: Mapping[str, int] | None = None,
                               models: Mapping[str, EnergyModel] | None = None) -> float:
    """殘帶候選對（規格 §9.5）：A 子胞插入 uav_A、B 殘帶插入 uav_B 後兩機完工之 max；``cell_B`` None → 只算 A。"""
    mA = (models or {}).get(uav_A.uav_id)
    cA = makespan_after_insert(list(plans.get(uav_A.uav_id, [])), pos_A, cell_A, uav_A, cache, cfg, cells=cells,
                               recovery_site_id=recovery_site_id, t_window=t_window, lam_by_cell=lam_by_cell, model=mA)
    if cell_B is None:
        return cA
    mB = (models or {}).get(uav_B.uav_id)
    cB = makespan_after_insert(list(plans.get(uav_B.uav_id, [])), pos_B, cell_B, uav_B, cache, cfg, cells=cells,
                               recovery_site_id=recovery_site_id, t_window=t_window, lam_by_cell=lam_by_cell, model=mB)
    return max(cA, cB)


# ---------------------------------------------------------------------------
# §9.1 硬限制
# ---------------------------------------------------------------------------
def _cell_strip_idx(cache: CostCache, cls: str, strip_ids: Sequence[str]) -> np.ndarray:
    idx = cache.idx.get(cls, {})
    return np.array([idx[s] for s in strip_ids if s in idx], dtype=int)


def _min_entry(cache: CostCache, uav_id: str, cls: str, strip_ids: Sequence[str]) -> tuple[float, float]:
    """(min entry time, min entry energy) over the cell's strips and both sides."""
    base = _base_uav(uav_id)
    rows = _cell_strip_idx(cache, cls, strip_ids)
    if base not in cache.entry or rows.size == 0:
        return _INF, _INF
    e_t = cache.entry[base][rows]
    e_e = cache.entry_e[base][rows]
    fin = np.isfinite(e_t)
    if not fin.any():
        return _INF, _INF
    return float(e_t[fin].min()), float(e_e[np.isfinite(e_e)].min()) if np.isfinite(e_e).any() else _INF


def _min_rtb(cache: CostCache, cls: str, strip_ids: Sequence[str], site: str | None) -> tuple[float, float]:
    """(min rtb time, min rtb energy incl. e^LDG) over the cell's strips and both terminal sides."""
    rows = _cell_strip_idx(cache, cls, strip_ids)
    if rows.size == 0 or site is None or (cls, site) not in cache.rtb:
        return _INF, _INF
    t = cache.rtb[(cls, site)][rows]
    fin = np.isfinite(t)
    if not fin.any():
        return _INF, _INF
    e_min = _INF
    bw = cache.budget.wh.get(cls)
    if bw is not None and cache.budget.strip_ids.get(cls) is not None:
        try:
            si = cache.budget._site_index(site)
            brows = np.array([cache.budget.idx[cls][s] for s in strip_ids if s in cache.budget.idx[cls]], dtype=int)
            if brows.size:
                v = bw[brows, :, si]
                if np.isfinite(v).any():
                    e_min = float(v[np.isfinite(v)].min())
        except KeyError:
            pass
    return float(t[fin].min()), e_min


def hard_feasible(cell: Cell, cls: str, uav: UAVInstance, model: EnergyModel, cache: CostCache, ctx: MissionContext,
                  lam: int | None = None, *, strip_ids: Sequence[str] | None = None,
                  T_single_s: float | None = None) -> tuple[bool, str]:
    """H1–H7（規格 §9.1）；回傳 (可行, 原因)。原因字串以 ``H<k>_`` 起首。

    H1 ``class_can_serve(sensor_req, cls, uav.available_kinds)`` 且 uav.cls == cls；H2 ``cell.polygon ⊕ disc(d_turn)``
    ⊂ ``ctx.link_poly[cls] ∩ Disc(H, ρ^sym_c)``（enforce_link）；H3 胞時間窗與空域窗重疊且 T_single/λ 可放入；
    H4 e^TO + e^in + e^oper/λ + e^RTB(→recovery, 含 e^LDG) ≤ η_T E^avail − r E_nom（λ None → min(n_U^c, n_q)）；
    H5 T_single 有限（V_a^* 存在，單向亦可）；H6 高度帶（``cell.feasible[cls]``）；H7 條帶存在且迴轉帶不落禁區
    （條帶 ``feasible``）。``strip_ids``／``T_single_s`` 可限定子任務（多架次分塊）。"""
    cfg = ctx.cfg
    if uav.cls != cls:
        return False, "H1_class_mismatch"
    if not class_can_serve(cell.sensor_req, cls, uav.available_kinds):
        return False, "H1_sensor"
    if not cell.feasible.get(cls, False):
        r = cell.infeasible_reason.get(cls, "")
        if r in _BAND_REASONS or r == "":
            return False, "H6_" + (r or "band")
        if r in ("no_fleet", "sensor_incompatible", "no_sensor"):
            return False, "H1_" + r
        return False, "H5_" + r
    sids = list(strip_ids) if strip_ids is not None else [s.strip_id for s in cell.strips.get(cls, [])]
    strips = [s for s in cell.strips.get(cls, []) if s.strip_id in set(sids)]
    if not sids:
        return False, "H7_no_strips"
    if any(not s.feasible for s in strips):
        return False, "H7_turn_band_keepout"
    T = float(cell.T_single_s.get(cls, _INF)) if T_single_s is None else float(T_single_s)
    if not math.isfinite(T):
        return False, "H5_no_airspeed"
    n_q = len(sids)
    if lam is None:
        n_u = sum(1 for u in ctx.inputs.fleet if u.cls == cls)
        lam = max(1, min(n_u, n_q))
    lam = max(1, int(lam))
    # H2
    if getattr(cfg, "enforce_link", True):
        d_turn = float(cell.d_turn_m.get(cls, 0.0))
        omega = cell.polygon.buffer(d_turn) if d_turn > 0 else cell.polygon
        lp = ctx.link_poly.get(cls)
        if lp is not None and not lp.buffer(1e-6).contains(omega):
            return False, "H2_link"
        rho = float(ctx.rho_sym_m.get(cls, _INF))
        if math.isfinite(rho):
            far = float(Point(ctx.home_xy).hausdorff_distance(omega))
            if far > rho + 1e-6:
                return False, "H2_radius"
    # H3
    if getattr(cfg, "enforce_time_windows", True):
        tw = cell.time_window_s
        gw = ctx.t_window
        if tw is not None:
            lo, hi = float(tw[0]), float(tw[1])
            if gw is not None:
                lo, hi = max(lo, float(gw[0])), min(hi, float(gw[1]))
            if hi - lo < T / lam - _EPS:
                return False, "H3_window"
    # H4
    if getattr(cfg, "enforce_energy", True) and model is not None:
        so = cache.static_ok.get(cls)
        rows = _cell_strip_idx(cache, cls, sids)
        if so is not None and rows.size and not so[rows].any():
            return False, "H4_static"
        site = _resolve_site(cache, cls, ctx.recovery_site_id)
        _t_in, e_in = _min_entry(cache, uav.uav_id, cls, sids)
        _t_rtb, e_rtb = _min_rtb(cache, cls, sids, site)
        if not math.isfinite(e_in):
            e_in = 0.0 if uav.uav_id not in cache.entry else _INF
        if not math.isfinite(e_rtb):
            e_rtb = model.e_landing_wh() if rows.size == 0 else _INF
        e_oper = model.transfer_wh(T / lam, 0.0)
        need = model.e_takeoff_wh() + e_in + e_oper + e_rtb
        budget = model.eta_T * float(model.available_wh) - model.reserve_wh()
        if not need <= budget + _EPS:
            return False, "H4_energy"
    return True, ""


# ---------------------------------------------------------------------------
# §9.2 配置指標 J
# ---------------------------------------------------------------------------
def _t_oper_pairs(cache: CostCache, cls: str, strip_ids: Sequence[str]) -> float:
    """T^oper = Σ_l max(t_LR,l + t_RL,l+1, t_RL,l + t_LR,l+1)（規格 §9.2；只作 J 比值用）。"""
    rows = _cell_strip_idx(cache, cls, strip_ids)
    if rows.size == 0:
        return 0.0
    op = cache.op[cls][rows]
    fin = np.where(np.isfinite(op), op, np.nan)
    if rows.size == 1:
        v = np.nanmin(fin)
        return float(v) if np.isfinite(v) else 0.0
    a = fin[:-1, 0] + fin[1:, 1]
    b = fin[:-1, 1] + fin[1:, 0]
    with np.errstate(invalid="ignore"):
        m = np.fmax(a, b)
    m = np.where(np.isnan(m), 0.0, m)
    return float(m.sum())


def _J_terms(strip_ids: Sequence[str], cls: str, priority: float, fit: float, fleet_cls: Sequence[UAVInstance],
             cache: CostCache, site: str | None) -> np.ndarray:
    t_in = 0.0
    any_fin = False
    for u in fleet_cls:
        t, _e = _min_entry(cache, u.uav_id, cls, strip_ids)
        if math.isfinite(t):
            t_in += t
            any_fin = True
    if not any_fin and fleet_cls:
        t_in = _INF
    t_rtb, _ = _min_rtb(cache, cls, strip_ids, site)
    return np.array([t_in, float(len(strip_ids)), _t_oper_pairs(cache, cls, strip_ids), float(priority),
                     t_rtb if math.isfinite(t_rtb) else _INF, float(fit)], dtype=float)


def _config_index_tasks(tasks: Sequence[_Task], cls: str, fleet_cls: Sequence[UAVInstance], cache: CostCache,
                        cfg: PlannerConfig, site: str | None) -> dict[str, float]:
    if not tasks:
        return {}
    M = np.array([_J_terms(t.strip_ids, cls, t.priority, t.fit, fleet_cls, cache, site) for t in tasks])
    bad = ~np.isfinite(M).all(axis=1)
    Mf = np.where(np.isfinite(M), M, 0.0)
    sums = Mf.sum(axis=0)
    with np.errstate(divide="ignore", invalid="ignore"):
        frac = np.where(sums > 0, Mf / np.where(sums > 0, sums, 1.0), 0.0)
    w = np.asarray(cfg.omega, dtype=float)
    J = frac @ w
    J[bad] = 0.0
    return {t.key: float(j) for t, j in zip(tasks, J)}


def config_index_J(cells: list[Cell], cls: str, fleet_cls: list[UAVInstance], cache: CostCache, cfg: PlannerConfig,
                   *, recovery_site_id: str | None = None) -> dict[str, float]:
    """規格 §9.2（擴充論文 Eq.30，時間版）：J^c_q = ω·[Σ_i min_l t^in / Σ, n_q/Σ, T^oper/Σ, val/Σ, T^RTB/Σ, S/Σ]。
    T^oper 只用於此；鍵 = 任務鍵（both 胞 ``cell_id#cls``）；無條帶或進入／RTB 皆 inf 之胞 → 0。"""
    site = _resolve_site(cache, cls, recovery_site_id)
    tasks = [_task_of(c, cls) for c in cells if c.strips.get(cls)]
    return _config_index_tasks(tasks, cls, fleet_cls, cache, cfg, site)


# ---------------------------------------------------------------------------
# §9.3 構型指派與架數
# ---------------------------------------------------------------------------
def _fleet_by_cls(fleet: Sequence[UAVInstance]) -> dict[str, list[UAVInstance]]:
    out: dict[str, list[UAVInstance]] = {}
    for u in fleet:
        out.setdefault(u.cls, []).append(u)
    return out


def _class_t_in(cache: CostCache, cls: str, fleet_cls: Sequence[UAVInstance], strip_ids: Sequence[str]) -> float:
    ts = [_min_entry(cache, u.uav_id, cls, strip_ids)[0] for u in fleet_cls]
    ts = [t for t in ts if math.isfinite(t)]
    return min(ts) if ts else _INF


def _strip_costs_in_order(cache: CostCache, cls: str, strip_ids: Sequence[str]) -> np.ndarray:
    """每條帶 (t_oper + t_trans自前一條)（依胞內順序；反向轉場兩型取平均）。"""
    rows = _cell_strip_idx(cache, cls, strip_ids)
    n = rows.size
    out = np.zeros(n)
    if n == 0:
        return out
    op = cache.op[cls][rows]
    for i in range(n):
        v = op[i][np.isfinite(op[i])]
        out[i] = float(v.mean()) if v.size else _INF
        if i > 0:
            tt = cache.tr[cls][rows[i - 1], rows[i], [1, 2]]
            tt = tt[np.isfinite(tt)]
            out[i] += float(tt.mean()) if tt.size else _INF
    return out


def _count_for_class(tasks: list[_Task], cls: str, fleet_cls: Sequence[UAVInstance], cache: CostCache,
                     cfg: PlannerConfig, T_use: float, site: str | None, res: CountResult) -> list[_Task]:
    """規格 §9.3 每構型獨立 Eq.31 + 能量下限 + Σλ ≤ n_U；回傳因 denom ≤ 0／Σλ 超出而須改派之任務。"""
    n_u = len(fleet_cls)
    reassign: list[_Task] = []
    if not tasks:
        return reassign
    J = _config_index_tasks(tasks, cls, fleet_cls, cache, cfg, site)
    for t in tasks:
        t.J = J.get(t.key, 0.0)
        res.J[t.key] = t.J
    sumJ = sum(t.J for t in tasks)
    kappa = float(cfg.kappa_load)
    keep: list[_Task] = []
    for t in tasks:
        n_q = len(t.strip_ids)
        lam_max = min(n_u, n_q)
        lam = int(round(n_u * t.J / sumJ)) if sumJ > 0 else 0
        lam = max(0, min(lam_max, lam))
        t_in = _class_t_in(cache, cls, fleet_cls, t.strip_ids)
        t_rtb, _ = _min_rtb(cache, cls, t.strip_ids, site)
        if math.isfinite(T_use):
            denom = kappa * T_use - (t_in if math.isfinite(t_in) else 0.0) - (t_rtb if math.isfinite(t_rtb) else 0.0)
        else:
            denom = _INF
        if denom <= 0:
            res.infeasible[t.key] = "energy_overhead"
            reassign.append(t)
            continue
        lam_floor = int(math.ceil(t.T_single_s / denom - _EPS)) if math.isfinite(denom) else 0
        lam_floor = max(0, lam_floor)
        if lam_floor > lam_max:
            if not cfg.multi_sortie:
                costs = _strip_costs_in_order(cache, cls, t.strip_ids)
                budget = lam_max * denom
                cum = np.cumsum(costs)
                k = int(np.searchsorted(cum, budget + _EPS, side="right"))
                res.partial[t.key] = list(t.strip_ids[k:])
                t.strip_ids = list(t.strip_ids[:k])
                t.lam_floor = lam_max
            else:
                t.lam_floor = lam_max
            lam = lam_max
        else:
            t.lam_floor = lam_floor
            lam = max(lam, lam_floor)
        res.by_cell[t.key] = (cls, lam)
        keep.append(t)
    # Σλ ≤ n_U
    def total() -> int:
        return sum(res.by_cell[t.key][1] for t in keep)
    order = sorted(keep, key=lambda t: t.J)
    changed = True
    while total() > n_u and changed:
        changed = False
        for t in order:
            c, lam = res.by_cell[t.key]
            if lam > t.lam_floor and lam > 0:
                res.by_cell[t.key] = (c, lam - 1)
                changed = True
                if total() <= n_u:
                    break
    if total() > n_u and not cfg.multi_sortie:
        for t in order:
            if total() <= n_u:
                break
            c, lam = res.by_cell[t.key]
            if lam > 0:
                res.by_cell[t.key] = (c, 0)
                res.infeasible.setdefault(t.key, "lambda_over_fleet")
                reassign.append(t)
    return reassign


def assign_config_and_count(cells: list[Cell], fleet: list[UAVInstance], models: dict[str, EnergyModel], cache: CostCache,
                            cfg: PlannerConfig, T_use_s: dict[str, float], *, recovery_site_id: str | None = None,
                            fixed_cls: Mapping[str, str] | None = None) -> CountResult:
    """規格 §9.3 ASSIGN_CONFIG_AND_COUNT（D16）：feasible = {c: S^c_q > 0}；score = S/T̃、T̃ = (t_in + T_single + t_RTB)/T_use；
    c* = argmax、c2 = 次選；每構型獨立 λ = clip(round(n_U J/ΣJ), 0, min(n_U, n_q))、denom = κ_load T_use − t_in − t_RTB
    （≤ 0 → energy_overhead → 改派 c2 或 UNCOVERABLE）、λ_floor = ⌈T_single/denom⌉；架數不足 → 部分覆蓋（``partial``）；
    Σλ > n_U → 依 J 遞減。``both`` 胞 → 兩任務（``cell_id#A``、``cell_id#B``）類別固定；``fixed_cls`` 亦可指定。"""
    res = CountResult(by_cell={}, partial={}, infeasible={})
    fb = _fleet_by_cls(fleet)
    classes = [c for c in ("A", "B") if c in fb] + [c for c in fb if c not in ("A", "B")]
    sites = {c: _resolve_site(cache, c, recovery_site_id) for c in classes}
    T_use = {c: float(T_use_s.get(c, _INF)) for c in classes}

    kinds = {c: frozenset().union(*[u.available_kinds for u in fb[c]]) for c in classes}

    def score(cell: Cell, c: str) -> float:
        sids = [s.strip_id for s in cell.strips.get(c, [])]
        T = float(cell.T_single_s.get(c, _INF))
        S = float(cell.fit.get(c, 0.0))
        if not sids or not math.isfinite(T) or S <= 0 or not cell.feasible.get(c, False):
            return 0.0
        if not class_can_serve(cell.sensor_req, c, kinds[c]):       # H1（胞 fit 可能來自 any 分解前之值）
            return 0.0
        t_in = _class_t_in(cache, c, fb[c], sids)
        t_rtb, _ = _min_rtb(cache, c, sids, sites[c])
        if not (math.isfinite(t_in) and math.isfinite(t_rtb)):
            return 0.0
        tt = t_in + T + t_rtb
        T_tilde = tt / T_use[c] if math.isfinite(T_use[c]) and T_use[c] > 0 else tt
        return S / T_tilde if T_tilde > 0 else 0.0

    pending: dict[str, list[_Task]] = {c: [] for c in classes}
    for cell in cells:
        if cell.sensor_req == "both":
            served: list[str] = []
            for c in classes:
                if cell.feasible.get(c, False) and cell.strips.get(c):
                    t = _task_of(cell, c)
                    pending[c].append(t)
                    res.second_choice[t.key] = None
                    served.append(c)
                elif c in cell.feasible:
                    res.infeasible[task_key(cell, c)] = cell.infeasible_reason.get(c, "") or "infeasible"
            # R-04：規格 §6.2 之 both 胞需要「兩種感測需求都被服務」。兩種達成方式：(a) 兩構型各鋪一套；
            # (b) 單一構型同時掛載兩種感測器 → `context.build_sensors` 產生 `CombinedSensor`，以較窄間距鋪一套
            # 條帶即同時滿足（此時 κ(both, c) 為真）。兩者皆不成立（唯一服務構型只有單一感測種類）時，第二個
            # 感測需求會無聲消失，故明確記為不可覆蓋，讓它進入 MissionPlan.uncovered。
            if len(served) == 1 and not sensor_compatible("both", kinds[served[0]]):
                res.infeasible[cell.cell_id + BOTH_SEP + SECOND_SENSOR_TAG] = "both_second_sensor_unserved"
            continue
        if fixed_cls and cell.cell_id in fixed_cls:
            c = fixed_cls[cell.cell_id]
            if c in pending:
                t = _task_of(cell, c)
                pending[c].append(t)
                res.second_choice[t.key] = None
            else:
                res.infeasible[cell.cell_id] = "no_fleet"
            continue
        sc = {c: score(cell, c) for c in classes}
        ranked = sorted((c for c in classes if sc[c] > 0), key=lambda c: -sc[c])
        if not ranked:
            res.infeasible[cell.cell_id] = "no_feasible_class"
            continue
        t = _task_of(cell, ranked[0])
        pending[ranked[0]].append(t)
        res.second_choice[t.key] = ranked[1] if len(ranked) > 1 else None
    tried: dict[str, set[str]] = {}
    for _round in range(4):
        moved = False
        for c in classes:
            tasks = pending[c]
            pending[c] = []
            if not tasks:
                continue
            re = _count_for_class(tasks, c, fb[c], cache, cfg, T_use[c], sites[c], res)
            for t in re:
                tried.setdefault(t.key, set()).add(c)
                c2 = res.second_choice.get(t.key)
                if c2 is not None and c2 not in tried[t.key] and c2 in pending and t.cell.sensor_req != "both":
                    reason = res.infeasible.pop(t.key, "")
                    res.by_cell.pop(t.key, None)
                    t2 = _task_of(t.cell, c2)
                    res.second_choice[t2.key] = None
                    pending[c2].append(t2)
                    moved = True
                    res.infeasible.setdefault(t.key + "@" + c, reason)
                elif res.infeasible.get(t.key) == "lambda_over_fleet":
                    # 無次選構型：保留 λ=0，交拍賣階段以「一機多胞」承接（規格 §9.3 末）
                    res.infeasible.pop(t.key, None)
                    res.by_cell[t.key] = (c, 0)
                else:
                    res.by_cell.pop(t.key, None)
        if not moved:
            break
    for k in list(res.infeasible):
        if "@" in k and k.split("@", 1)[0] in res.by_cell:
            res.infeasible.pop(k)
    return res


# ---------------------------------------------------------------------------
# §9.6 多架次分塊
# ---------------------------------------------------------------------------
def split_for_sorties(cell: Cell, cls: str, uav: UAVInstance, cache: CostCache, model: EnergyModel,
                      cfg: PlannerConfig) -> list[list[str]]:
    """規格 §9.6 SPLIT_FOR_SORTIES：依胞內條帶順序切成能量可行塊；budget = κ_load T_use − t_in − t_RTB
    （T_use = e_usable/(P_c k_unc)·3600；t_in 該機最短進入、t_RTB 最短 RTB）；單條帶超額者自成一塊。"""
    sids = [s.strip_id for s in cell.strips.get(cls, [])]
    if not sids:
        return []
    if not getattr(cfg, "enforce_energy", True):
        return [sids]
    p = model.p_cruise_w() * model.k_unc
    T_use = float(model.e_usable_wh()) / p * 3600.0 if p > 0 else _INF
    if not math.isfinite(T_use):
        return [sids]
    t_in, _ = _min_entry(cache, uav.uav_id, cls, sids)
    site = _resolve_site(cache, cls, None)
    t_rtb, _ = _min_rtb(cache, cls, sids, site)
    budget = cfg.kappa_load * T_use - (t_in if math.isfinite(t_in) else 0.0) - (t_rtb if math.isfinite(t_rtb) else 0.0)
    if budget <= 0:
        return [[s] for s in sids]
    costs = _strip_costs_in_order(cache, cls, sids)
    rows = _cell_strip_idx(cache, cls, sids)
    op = cache.op[cls][rows]
    chunks: list[list[str]] = []
    cur: list[str] = []
    acc = 0.0
    for i, sid in enumerate(sids):
        v = op[i][np.isfinite(op[i])]
        t_op = float(v.mean()) if v.size else _INF
        c = t_op if not cur else costs[i]
        if cur and acc + c > budget + _EPS:
            chunks.append(cur)
            cur, acc = [sid], t_op
        else:
            cur.append(sid)
            acc += c
    if cur:
        chunks.append(cur)
    return chunks


# ---------------------------------------------------------------------------
# §9.7 起飛排程
# ---------------------------------------------------------------------------
def _sched_key(uav_id: str, sortie: int) -> str:
    return uav_id if sortie <= 0 else f"{uav_id}{SORTIE_SEP}{sortie}"


def _nearest_site(cache: CostCache, state: UAVState | None) -> str:
    """起飛站分組：以初始狀態最近之 ``cache.sites`` 站（S^app 位置）為代表；無站點 → 單一佇列。"""
    if state is None or not cache.sites:
        return ""
    best, best_d = "", _INF
    for sid, v in cache.sites.items():
        vals = list(v.values()) if isinstance(v, dict) else [v]
        for pose, _z in vals:
            p = pose.as_array() if hasattr(pose, "as_array") else np.asarray(pose, dtype=float)
            d = math.hypot(float(p[0]) - state.x_m, float(p[1]) - state.y_m)
            if d < best_d:
                best, best_d = sid, d
    return best


def takeoff_schedule(teams: list[Team], cache: CostCache, estimates: dict[str, float], delays: dict[str, float],
                     cfg: PlannerConfig) -> dict[str, float]:
    """規格 §9.7：同站起飛間隔 Δt_dep；順序 = Ĉ_i 降序（tie → 進入時間長者先）；t0_i = k_i Δt_dep + Δ_i。
    站點以初始狀態最近之 ``cache.sites`` 站分組（A/B 之 S_acc 位置不同但同站）；第二架次（sortie_index ≥ 1，鍵 ``uav#s{n}``）不排隊，
    t0 = delays[鍵]（可用時刻）。"""
    out: dict[str, float] = {}
    groups: dict[tuple[float, float], list[tuple[float, float, str]]] = {}
    for team in teams:
        for u in team.uav_ids:
            s = int(team.sortie_index.get(u, 0))
            key = _sched_key(u, s)
            if s >= 1:
                out[key] = float(delays.get(key, 0.0))
                continue
            st = cache.init_states.get(u)
            g = _nearest_site(cache, st)
            ent = cache.entry.get(u)
            t_in = float(np.nanmin(np.where(np.isfinite(ent), ent, np.nan))) if ent is not None and np.isfinite(ent).any() else 0.0
            groups.setdefault(g, []).append((float(estimates.get(u, 0.0)), t_in, u))
    for g, lst in groups.items():
        lst.sort(key=lambda x: (-x[0], -x[1], x[2]))
        for k, (_c, _t, u) in enumerate(lst):
            out[u] = k * float(cfg.dt_dep_s) + float(delays.get(u, 0.0))
    return out


# ---------------------------------------------------------------------------
# §9.4 ALLOCATE
# ---------------------------------------------------------------------------
class _Alloc:
    """配置狀態機（規格 §9.4 Phase 1–5）。"""

    def __init__(self, cells: list[Cell], strips: dict[str, dict[str, Strip]], fleet: list[UAVInstance],
                 init_states: dict[str, UAVState], models: dict[str, EnergyModel], cache: CostCache, ctx: MissionContext,
                 cfg: PlannerConfig, frozen_teams: list[Team] | None,
                 residual_pairs: dict[str, list[tuple[int, Cell, Cell | None]]] | None) -> None:
        self.cells = {c.cell_id: c for c in cells}
        self.strips = strips
        self.fleet = list(fleet)
        self.fleet_by_id = {u.uav_id: u for u in fleet}
        self.fleet_by_cls = _fleet_by_cls(fleet)
        self.init_states = init_states
        self.models = dict(models)
        self.cache = cache
        self.ctx = ctx
        self.cfg = cfg
        self.site = ctx.recovery_site_id
        self.t_window = ctx.t_window if getattr(cfg, "enforce_time_windows", True) else None
        self.tasks: dict[str, _Task] = {}
        self.plans: dict[str, list[str]] = {u.uav_id: [] for u in fleet}
        self.frozen_uavs: set[str] = set()
        self.frozen_teams = list(frozen_teams or [])
        self.residual_pairs = residual_pairs or {}
        self.uncoverable: list[tuple[str, str]] = []
        self.leftover: list[str] = []
        self.residual_k: dict[str, int] = {}
        self.virtual_t0: dict[str, float] = {}
        self.est_cache: dict[tuple, PlanEstimate] = {}
        self.dc = None

    # -- 基本 -----------------------------------------------------------------
    def model_of(self, uav_id: str) -> EnergyModel | None:
        return self.models.get(_base_uav(uav_id)) or self.ctx.models.get(_base_uav(uav_id))

    def uav_of(self, uav_id: str) -> UAVInstance:
        return self.fleet_by_id[_base_uav(uav_id)]

    def cls_of(self, uav_id: str) -> str:
        return self.uav_of(uav_id).cls

    def lam_counts(self) -> dict[str, int]:
        cnt: dict[str, int] = {}
        for plan in self.plans.values():
            for k in plan:
                cnt[k] = cnt.get(k, 0) + 1
        return cnt

    def estimate(self, uav_id: str, plan: Sequence[str], lam: Mapping[str, int] | None = None) -> PlanEstimate:
        lam = lam if lam is not None else self.lam_counts()
        key = (uav_id, tuple(plan), tuple(sorted((k, lam.get(k, 1)) for k in plan)))
        est = self.est_cache.get(key)
        if est is None:
            tasks = [self.tasks[k] for k in plan]
            cls = self.cls_of(uav_id)
            est = _chain(tasks, uav_id, cls, self.cache, self.cfg, t0_s=self.virtual_t0.get(uav_id),
                         recovery_site_id=self.site, t_window=self.t_window, lam_of=lam, model=self.model_of(uav_id))
            if est.ok and not _energy_ok(est, self.model_of(uav_id), self.cfg):
                est = PlanEstimate(est.C_s, est.e_wh, est.delay_s, False, "H4_energy_plan", est.per_task)
            self.est_cache[key] = est
        return est

    def C_of(self, uav_id: str, lam: Mapping[str, int] | None = None) -> float:
        est = self.estimate(uav_id, self.plans[uav_id], lam)
        return est.C_s if est.ok else _INF

    def all_C(self) -> dict[str, float]:
        lam = self.lam_counts()
        return {u: self.C_of(u, lam) for u in self.plans}

    def C_max(self, C: Mapping[str, float] | None = None) -> float:
        C = C if C is not None else self.all_C()
        return max(C.values()) if C else 0.0

    def feasible_for(self, task: _Task, uav_id: str, lam: int | None = None) -> bool:
        u = self.uav_of(uav_id)
        ok, _ = hard_feasible(task.cell, task.cls, u, self.model_of(uav_id), self.cache, self.ctx, lam,
                              strip_ids=task.strip_ids, T_single_s=task.T_single_s)
        return ok

    def positions(self, uav_id: str) -> list[int]:
        n = len(self.plans[uav_id])
        return [n] if uav_id in self.frozen_uavs else list(range(n + 1))

    def add_task(self, task: _Task) -> None:
        self.tasks[task.key] = task

    def ensure_strips_cached(self, cell: Cell, cls: str) -> None:
        missing = [s for s in cell.strips.get(cls, []) if s.strip_id not in self.cache.idx.get(cls, {})]
        if missing:
            self.cache.add_strips(cls, missing, self.fleet, self.init_states, self.cache.sites, self.ctx)
            self.est_cache.clear()

    # -- Phase 0：凍結團隊、任務集 ------------------------------------------------
    def setup(self) -> tuple[list[_Task], list[_Task]]:
        """回傳 (計數任務, both 任務)（皆非凍結、非殘帶候選母胞）。"""
        frozen_cells: set[str] = set()
        for team in self.frozen_teams:
            for u in team.uav_ids:
                if u not in self.plans:
                    continue
                self.frozen_uavs.add(u)
                plan: list[str] = []
                for cid in team.cell_ids:
                    cell = self.cells.get(cid)
                    if cell is None:
                        raise ValueError(f"frozen team {team.team_id}: cell {cid!r} not in cells")
                    k = task_key(cell, team.cls)
                    if k not in self.tasks:
                        sids = [s for s in team.strip_ids if any(s == st.strip_id for st in cell.strips.get(team.cls, []))]
                        t = _task_of(cell, team.cls, sids or None)
                        if sids and len(sids) != len(cell.strips.get(team.cls, [])):
                            t.T_single_s = self._exact_time(cell, team.cls, sids)
                        self.add_task(t)
                    plan.append(k)
                    frozen_cells.add(cid)
                self.plans[u] = plan
        count_tasks: list[Cell] = []
        both_tasks: list[_Task] = []
        for cid, cell in self.cells.items():
            if cid in frozen_cells or cid in self.residual_pairs:
                continue
            if cell.parent_cell_id is not None and cell.parent_cell_id in self.residual_pairs:
                continue
            count_tasks.append(cell)
        return count_tasks, both_tasks

    def _exact_time(self, cell: Cell, cls: str, sids: Sequence[str]) -> float:
        if self.dc is None:
            self.dc = self.ctx.decompose_ctx()
        ss = [s for s in cell.strips.get(cls, []) if s.strip_id in set(sids)]
        try:
            return float(exact_time(ss, cls, self.dc))
        except Exception:
            T = float(cell.T_single_s.get(cls, _INF))
            n = max(1, len(cell.strips.get(cls, [])))
            return T * len(ss) / n

    # -- Phase 1：貪婪 ----------------------------------------------------------
    def phase1(self, count: CountResult) -> list[_Task]:
        """依 J 降序，每任務選 λ 架「進入時間最短且 H1–H7 可行」之未指派同構型機；回傳未指派（λ=0 或無候選）任務。"""
        open_tasks: list[_Task] = []
        free = {u.uav_id for u in self.fleet if u.uav_id not in self.frozen_uavs}
        ordered = sorted(self.tasks_from_count(count), key=lambda t: -t.J)
        for t in ordered:
            cls, lam = count.by_cell[t.key]
            if lam <= 0:
                open_tasks.append(t)
                continue
            cands = []
            for u in self.fleet_by_cls.get(cls, []):
                if u.uav_id not in free or self.plans[u.uav_id]:
                    continue
                if not self.feasible_for(t, u.uav_id, lam):
                    continue
                t_in, _ = _min_entry(self.cache, u.uav_id, cls, t.strip_ids)
                if math.isfinite(t_in):
                    cands.append((t_in, u.uav_id))
            cands.sort()
            chosen = [u for _t, u in cands[:lam]]
            if not chosen:
                open_tasks.append(t)
                continue
            for u in chosen:
                self.plans[u] = [t.key]
                free.discard(u)
        return open_tasks

    def tasks_from_count(self, count: CountResult) -> list[_Task]:
        out = []
        for key, (cls, _lam) in count.by_cell.items():
            if key in self.tasks:
                out.append(self.tasks[key])
        return out

    # -- Phase 2：拍賣 ----------------------------------------------------------
    def best_insert(self, task: _Task, C: Mapping[str, float], lam: Mapping[str, int], *,
                    uav_filter: Sequence[str] | None = None) -> tuple[tuple, str, int, float] | None:
        """對任務 task 於所有相容機／位置之最佳插入：回傳 (bid, uav_id, pos, C_i_after)。"""
        best = None
        C_max = max(C.values()) if C else 0.0
        uavs = list(uav_filter) if uav_filter is not None else [u for u in self.plans if self.cls_of(u) == task.cls]
        for u in uavs:
            if self.cls_of(u) != task.cls:
                continue
            plan = self.plans[u]
            if task.key in plan:
                continue
            lam_u = dict(lam)
            lam_u[task.key] = lam_u.get(task.key, 0) + 1
            if not self.feasible_for(task, u, lam_u[task.key]):
                continue
            others = max((c for v, c in C.items() if v != u), default=0.0)
            for pos in self.positions(u):
                new_plan = plan[:pos] + [task.key] + plan[pos:]
                est = self.estimate(u, new_plan, lam_u)
                if not est.ok:
                    continue
                delta = max(est.C_s, others) - C_max
                bid = (delta, est.C_s, -task.fit)
                if best is None or bid < best[0]:
                    best = (bid, u, pos, est.C_s)
        return best

    def phase2(self, open_tasks: list[_Task]) -> None:
        open_single: dict[str, _Task] = {t.key: t for t in open_tasks}
        open_pairs: dict[str, list[tuple[int, _Task, _Task | None]]] = {}
        for pid, cands in self.residual_pairs.items():
            parent = self.cells.get(pid)
            if parent is None:
                continue
            lst: list[tuple[int, _Task, _Task | None]] = []
            for (k, ca, cb) in cands:
                tA: _Task | None = None
                tB: _Task | None = None
                if k > 0 and ca is not None and ca.strips.get("A"):
                    tA = _task_of(ca, "A", parent=pid, k_A=k, key=_pair_key(pid, parent, ca, "A", k))
                if cb is not None and cb.strips.get("B"):
                    self.ensure_strips_cached(cb, "B")
                    tB = _task_of(cb, "B", parent=pid, k_A=k, key=_pair_key(pid, parent, cb, "B", k))
                if tA is None and tB is None:
                    continue
                for t_ in (tA, tB):
                    if t_ is not None:
                        self.add_task(t_)              # 登錄（未指派；λ 由 plans 計）
                if tA is None:
                    lst.append((k, tB, None))          # k = 0：B 獨佔
                else:
                    lst.append((k, tA, tB))
            if lst:
                open_pairs[pid] = lst
            else:
                self.uncoverable.append((pid, "no_feasible_insertion"))
        while open_single or open_pairs:
            C = self.all_C()
            lam = self.lam_counts()
            best_bid = None
            best_apply = None
            for key, t in open_single.items():
                r = self.best_insert(t, C, lam)
                if r is not None and (best_bid is None or r[0] < best_bid):
                    best_bid = r[0]
                    best_apply = ("single", key, [(t, r[1], r[2])])
            C_max = max(C.values()) if C else 0.0
            for pid, lst in open_pairs.items():
                for (k, t1, t2) in lst:
                    r1 = self.best_insert(t1, C, lam)
                    if r1 is None:
                        continue
                    if t2 is None:
                        delta = r1[0][0]
                        bid = (delta, r1[3], -max(t1.fit, 0.0))
                        apply = [(t1, r1[1], r1[2])]
                    else:
                        r2 = self.best_insert(t2, C, lam)
                        if r2 is None:
                            continue
                        others = max((c for v, c in C.items() if v not in (r1[1], r2[1])), default=0.0)
                        c_after = max(r1[3], r2[3], others)
                        bid = (c_after - C_max, max(r1[3], r2[3]), -max(t1.fit, t2.fit))
                        apply = [(t1, r1[1], r1[2]), (t2, r2[1], r2[2])]
                    if best_bid is None or bid < best_bid:
                        best_bid = bid
                        best_apply = ("pair", pid, apply, k)
            if best_apply is None:
                for key in open_single:
                    self.uncoverable.append((key, "no_feasible_insertion"))
                for pid in open_pairs:
                    self.uncoverable.append((pid, "no_feasible_insertion"))
                break
            kind, key = best_apply[0], best_apply[1]
            for (t, u, pos) in best_apply[2]:
                self.add_task(t)
                self.plans[u].insert(pos, t.key)
            if kind == "single":
                open_single.pop(key)
            else:
                open_pairs.pop(key)
                self.residual_k[key] = int(best_apply[3])

    # -- Phase 2b：多架次（規格 §9.6）--------------------------------------------
    def phase2_sorties(self) -> None:
        """所有機皆滿載仍有未覆蓋 → 加入虛擬第二架次機（同機、可用時刻 C_i + t_swap、能量同當架次），再拍賣。"""
        if not self.cfg.multi_sortie:
            return
        pending = [(k, r) for (k, r) in self.uncoverable if r == "no_feasible_insertion" and k in self.tasks]
        if not pending:
            return
        C = self.all_C()
        n_new = 0
        for u in list(self.plans):
            if SORTIE_SEP in u or u in self.frozen_uavs:
                continue
            if not self.plans[u]:
                continue
            vid = f"{u}{SORTIE_SEP}1"
            if vid in self.plans:
                continue
            st = self.cache.init_states.get(u)
            t_state = float(st.t_s) if st is not None else 0.0
            self.plans[vid] = []
            self.virtual_t0[vid] = C[u] + float(self.cfg.t_swap_s) + t_state
            n_new += 1
        if n_new == 0:
            return
        self.uncoverable = [(k, r) for (k, r) in self.uncoverable if not (r == "no_feasible_insertion" and k in self.tasks)]
        tasks = [self.tasks[k] for k, _r in pending]
        self.phase2(tasks)

    # -- Phase 3：局部改善 -------------------------------------------------------
    def _try_apply(self, new_plans: dict[str, list[str]], C_before: Mapping[str, float]) -> bool:
        """接受條件：C_max 降（tie：ΣC_i 降）且所有變動機可行。"""
        old = {u: self.plans[u] for u in new_plans}
        for u, p in new_plans.items():
            self.plans[u] = p
        lam = self.lam_counts()
        for u in new_plans:
            for k in self.plans[u]:
                if not self.feasible_for(self.tasks[k], u, lam.get(k, 1)):
                    self.plans.update(old)
                    return False
        for k, t in self.tasks.items():
            if lam.get(k, 0) > 0 and lam[k] < t.lam_floor and any(k in p for p in new_plans.values()):
                self.plans.update(old)
                return False
        C_after = self.all_C()
        if not all(math.isfinite(v) for v in C_after.values()):
            self.plans.update(old)
            return False
        cm_b, cm_a = max(C_before.values()), max(C_after.values())
        if cm_a < cm_b - _EPS or (abs(cm_a - cm_b) <= _EPS and sum(C_after.values()) < sum(C_before.values()) - _EPS):
            return True
        self.plans.update(old)
        return False

    def phase3(self) -> None:
        movable = [u for u in self.plans if u not in self.frozen_uavs]
        for _ in range(int(self.cfg.n_local_search)):
            C = self.all_C()
            if not C or not any(self.plans[u] for u in movable):
                break
            i_max = max(movable, key=lambda u: C[u]) if movable else None
            if i_max is None or not self.plans[i_max] or not math.isfinite(C[i_max]):
                break
            improved = False
            for q in list(self.plans[i_max]):
                for j in movable:
                    if j == i_max or self.cls_of(j) != self.cls_of(i_max):
                        continue
                    base_i = [k for k in self.plans[i_max] if k != q]
                    if q not in self.plans[j]:
                        for pos in range(len(self.plans[j]) + 1):
                            pj = self.plans[j][:pos] + [q] + self.plans[j][pos:]
                            if self._try_apply({i_max: base_i, j: pj}, C):
                                improved = True
                                break
                    if improved:
                        break
                    for q2 in list(self.plans[j]):
                        if q2 == q or q2 in self.plans[i_max]:
                            continue
                        pi = [q2 if k == q else k for k in self.plans[i_max]]
                        pj = [q if k == q2 else k for k in self.plans[j]]
                        if self._try_apply({i_max: pi, j: pj}, C):
                            improved = True
                            break
                    if improved:
                        break
                if improved:
                    break
            if not improved:
                break
        self.phase3_kA()

    def phase3_kA(self) -> None:
        """any 胞 k_A 局部微調：以候選清單中相鄰之 k 取代（一次 pass）。"""
        for pid, k_now in list(self.residual_k.items()):
            cands = self.residual_pairs.get(pid, [])
            ks = sorted({int(k) for k, _a, _b in cands})
            if k_now not in ks:
                continue
            i = ks.index(k_now)
            neighbours = [ks[j] for j in (i - 1, i + 1) if 0 <= j < len(ks)]
            for k_new in neighbours:
                if self._swap_residual(pid, k_now, k_new):
                    k_now = k_new
        return

    def _pair_tasks(self, pid: str, k: int) -> tuple[_Task | None, _Task | None] | None:
        parent = self.cells[pid]
        for (kk, ca, cb) in self.residual_pairs.get(pid, []):
            if int(kk) != int(k):
                continue
            tA = tB = None
            if k > 0 and ca is not None and ca.strips.get("A"):
                tA = _task_of(ca, "A", parent=pid, k_A=k, key=_pair_key(pid, parent, ca, "A", k))
            if cb is not None and cb.strips.get("B"):
                self.ensure_strips_cached(cb, "B")
                tB = _task_of(cb, "B", parent=pid, k_A=k, key=_pair_key(pid, parent, cb, "B", k))
            for t_ in (tA, tB):
                if t_ is not None:
                    self.add_task(t_)
            return tA, tB
        return None

    def _swap_residual(self, pid: str, k_old: int, k_new: int) -> bool:
        pt = self._pair_tasks(pid, k_new)
        if pt is None:
            return False
        tA, tB = pt
        old_keys = {k for k, t in self.tasks.items() if t.parent == pid and t.k_A == k_old}
        C = self.all_C()
        old_plans = {u: list(p) for u, p in self.plans.items()}
        for u in self.plans:
            self.plans[u] = [k for k in self.plans[u] if k not in old_keys]
        for t in (tA, tB):
            if t is not None:
                self.add_task(t)
        ok = True
        for t in (tA, tB):
            if t is None:
                continue
            lam = self.lam_counts()
            r = self.best_insert(t, self.all_C(), lam)
            if r is None:
                ok = False
                break
            self.plans[r[1]].insert(r[2], t.key)
        if ok:
            C_new = self.all_C()
            if max(C_new.values()) < max(C.values()) - _EPS:
                self.residual_k[pid] = k_new
                return True
        self.plans = old_plans
        return False

    # -- Phase 4：BALANCE --------------------------------------------------------
    def phase4(self) -> None:
        movable = [u for u in self.plans if u not in self.frozen_uavs]
        for _ in range(len(self.fleet)):
            C = self.all_C()
            if not movable:
                break
            i_max = max(movable, key=lambda u: C[u])
            if not self.plans[i_max] or not math.isfinite(C[i_max]):
                break
            lam = self.lam_counts()
            # 移出後原任務 λ ≥ 能量下限（且 ≥ 1）
            if any(lam[k] - 1 < max(1, self.tasks[k].lam_floor) for k in self.plans[i_max]):
                break
            best = None
            for key, t in self.tasks.items():
                if t.cls != self.cls_of(i_max) or key in self.plans[i_max] or lam.get(key, 0) == 0:
                    continue
                trial = {i_max: [key]}
                old = self.plans[i_max]
                self.plans[i_max] = [key]
                lam2 = self.lam_counts()
                ok = self.feasible_for(t, i_max, lam2[key])
                C2 = self.all_C() if ok else None
                self.plans[i_max] = old
                if not ok or C2 is None or not all(math.isfinite(v) for v in C2.values()):
                    continue
                drop = max(C.values()) - max(C2.values())
                if drop > _EPS and (best is None or drop > best[0]):
                    best = (drop, key)
            if best is None:
                break
            self.plans[i_max] = [best[1]]

    # -- Phase 5：團隊 ------------------------------------------------------------
    def build_teams(self) -> list[Team]:
        teams: list[Team] = []
        frozen_map = {u: tm for tm in self.frozen_teams for u in tm.uav_ids}
        seen_frozen: set[str] = set()
        for tm in self.frozen_teams:
            if tm.team_id in seen_frozen:
                continue
            seen_frozen.add(tm.team_id)
            cell_ids: list[str] = list(tm.cell_ids)
            strip_ids: list[str] = list(tm.strip_ids)
            for u in tm.uav_ids:
                for k in self.plans.get(u, []):
                    t = self.tasks[k]
                    if t.cell.cell_id not in cell_ids:
                        cell_ids.append(t.cell.cell_id)
                    for s in t.strip_ids:
                        if s not in strip_ids:
                            strip_ids.append(s)
            teams.append(Team(tm.team_id, tm.cls, list(tm.uav_ids), cell_ids, strip_ids, dict(tm.t0_s), dict(tm.sortie_index)))
        # 連通分量（非凍結機）
        uavs = [u for u in self.plans if u not in frozen_map and self.plans[u]]
        parent: dict[str, str] = {}

        def find(x: str) -> str:
            while parent.setdefault(x, x) != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        def union(a: str, b: str) -> None:
            ra, rb = find(a), find(b)
            if ra != rb:
                parent[ra] = rb

        for u in uavs:
            for k in self.plans[u]:
                union("u:" + u, "t:" + k)
        comps: dict[str, list[str]] = {}
        for u in uavs:
            comps.setdefault(find("u:" + u), []).append(u)
        counter: dict[str, int] = {}
        for root, members in comps.items():
            cls = self.cls_of(members[0])
            counter[cls] = counter.get(cls, 0) + 1
            cell_ids = []
            strip_ids = []
            for u in members:
                for k in self.plans[u]:
                    t = self.tasks[k]
                    if t.cell.cell_id not in cell_ids:
                        cell_ids.append(t.cell.cell_id)
                    for s in t.strip_ids:
                        if s not in strip_ids:
                            strip_ids.append(s)
            uav_ids = [_base_uav(u) for u in members]
            sortie = {_base_uav(u): (int(u.split(SORTIE_SEP, 1)[1]) if SORTIE_SEP in u else 0) for u in members}
            teams.append(Team(f"T{cls}{counter[cls]}", cls, uav_ids, cell_ids, strip_ids, {u: 0.0 for u in uav_ids}, sortie))
        return teams


def allocate(cells: list[Cell], strips: dict[str, dict[str, Strip]], fleet: list[UAVInstance],
             init_states: dict[str, UAVState], models: dict[str, EnergyModel], cache: CostCache, ctx: MissionContext,
             cfg: PlannerConfig, method: Literal["greedy", "auction"] = "auction", frozen_teams: list[Team] | None = None,
             residual_pairs: dict[str, list[tuple[int, Cell, Cell | None]]] | None = None, *,
             T_use_s: dict[str, float] | None = None) -> AllocationResult:
    """規格 §9.4 ALLOCATE（Phase 1 貪婪 → Phase 2 拍賣（一機多胞、殘帶候選對）→ Phase 3 局部改善 MOVE/SWAP（+ k_A 微調）
    → Phase 4 BALANCE → Phase 5 團隊合併）+ §9.7 起飛排程。

    * ``method="greedy"`` 只跑 Phase 1（其餘任務記 uncoverable(no_feasible_insertion)）+ Phase 5。
    * ``frozen_teams``：其機之既有胞序列不重排、只允許尾端追加；其條帶不再指派。
    * ``residual_pairs[parent_cell_id]``：``decompose.residual_candidates`` 之 [(k_A, A 子胞, B 殘帶)]；母胞不進入計數，
      候選對於 Phase 2 同時評估（B 殘帶條帶若不在 cache 中由本函式 ``cache.add_strips``）。
    * ``T_use_s``（cls → s）缺省 = 各機模型 e_usable/(P_c k_unc)·3600 之最小值（``enforce_energy=False`` → inf）。
    * 多架次（``cfg.multi_sortie``）：Phase 2 無可插入者 → 加入虛擬第二架次機（``uav#s1``，可用時刻 C_i + t_swap）。
    """
    A = _Alloc(cells, strips, fleet, init_states, models, cache, ctx, cfg, frozen_teams, residual_pairs)
    count_cells, _ = A.setup()
    if T_use_s is None:
        T_use_s = {}
        for c, us in A.fleet_by_cls.items():
            vals = []
            for u in us:
                m = A.model_of(u.uav_id)
                if m is None or not getattr(cfg, "enforce_energy", True):
                    continue
                p = m.p_cruise_w() * m.k_unc
                vals.append(float(m.e_usable_wh()) / p * 3600.0 if p > 0 else _INF)
            T_use_s[c] = max(0.0, min(vals)) if vals else _INF
    count = assign_config_and_count(count_cells, fleet, A.models, cache, cfg, T_use_s, recovery_site_id=ctx.recovery_site_id)
    for key, (cls, _lam) in count.by_cell.items():
        cid = key.split(BOTH_SEP, 1)[0]
        cell = A.cells[cid]
        t = _task_of(cell, cls)
        if key in count.partial:
            kept = [s for s in t.strip_ids if s not in set(count.partial[key])]
            t.strip_ids = kept
            t.T_single_s = A._exact_time(cell, cls, kept) if kept else 0.0
            A.leftover.extend(count.partial[key])
            for s in count.partial[key]:
                A.uncoverable.append((s, "energy"))
        t.lam_floor = 0
        t.J = count.J.get(key, 0.0)
        if not t.strip_ids:
            A.uncoverable.append((key, "energy"))
            continue
        A.add_task(t)
    for key, reason in count.infeasible.items():
        if "@" in key:
            continue
        if key not in count.by_cell:
            A.uncoverable.append((key, reason))
    # 能量下限（λ_floor）供 Phase 3/4：以 count 之 by_cell λ 與 T_single 推回
    open_tasks = A.phase1(count)
    if method == "auction":
        A.phase2(open_tasks)
        A.phase2_sorties()
        A.phase3()
        A.phase4()
    else:
        for t in open_tasks:
            A.uncoverable.append((t.key, "no_feasible_insertion"))
        for pid in A.residual_pairs:
            A.uncoverable.append((pid, "no_feasible_insertion"))
    teams = A.build_teams()
    # 估計、H3 延後、起飛排程
    lam = A.lam_counts()
    estimates: dict[str, float] = {}
    delays: dict[str, float] = {}
    for u in A.plans:
        est = A.estimate(u, A.plans[u], lam)
        estimates[u] = est.C_s if est.ok else (est.C_s if math.isfinite(est.C_s) else _INF)
        if est.delay_s > 0:
            delays[u] = est.delay_s
    for vid, t0 in A.virtual_t0.items():
        delays[vid] = t0
    t0 = takeoff_schedule(teams, cache, estimates, delays, cfg)
    for tm in teams:
        for u in tm.uav_ids:
            key = _sched_key(u, int(tm.sortie_index.get(u, 0)))
            if key in t0:
                tm.t0_s[u] = t0[key]
                if key in estimates and math.isfinite(estimates[key]):
                    estimates[key] += t0[key] - delays.get(key, 0.0) if int(tm.sortie_index.get(u, 0)) == 0 else 0.0
    lambda_by_cell: dict[str, tuple[str, int]] = {}
    for key, t in A.tasks.items():
        n = lam.get(key, 0)
        if n > 0:
            lambda_by_cell[t.key if t.cell.sensor_req == "both" else t.cell.cell_id] = (t.cls, n)
    seen = set()
    unc: list[tuple[str, str]] = []
    for (k, r) in A.uncoverable:
        if (k, r) in seen:
            continue
        seen.add((k, r))
        if r == "no_feasible_insertion" and (k in lambda_by_cell or k in A.residual_k):
            continue                                        # 多架次補救後已指派
        unc.append((k, r))
    return AllocationResult(teams=teams, uncoverable=unc, leftover_strips=list(dict.fromkeys(A.leftover)),
                            lambda_by_cell=lambda_by_cell, estimates=estimates, residual_k=dict(A.residual_k),
                            plans={u: list(p) for u, p in A.plans.items()}, delays_s=delays)
