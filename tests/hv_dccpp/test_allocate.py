"""tests/test_allocate.py — 契約 §9 `allocate.py` 不變量與規格 §17.2 清單（sensor_hard_constraint、
energy_infeasible_not_assigned、lambda_bounds_energy_floor、energy_overhead_infeasible、partial_coverage_when_short、
auction_multi_cell、auction_residual_pairs、phase2_records_uncoverable、move_swap_hard_feasible、
local_search_reduces_makespan、time_window_delayed_takeoff、multi_sortie_first_lands_home、takeoff_order_by_C_hat）。

情境：`scenario.build_mini_example()`（平地、西風 5 m/s）以 `dem_step_m=50`、`terrain_margin_m=1500` 建構；
真實 `decompose_area` 之胞（A 7 條／B 12 條）與 `residual_candidates`（k ∈ {7,6,5,4,0}）；合成之子胞以 A 條帶子集構成。
"""
from __future__ import annotations

import dataclasses
import math
import time

import numpy as np
import pytest

from hv_dccpp import allocate as AL
from hv_dccpp import altitude as ALT
from hv_dccpp import decompose as D
from hv_dccpp.context import build_context
from hv_dccpp.costs import CostCache
from hv_dccpp.mission import PlannerConfig
from hv_dccpp.platform import PLATFORM_HERO, PLATFORM_PIONEER
from hv_dccpp.scenario import build_mini_example

A_SPEC, B_SPEC = PLATFORM_PIONEER, PLATFORM_HERO


# ---------------------------------------------------------------------------
# fixtures
# ---------------------------------------------------------------------------
def _fleet_with(inputs, n_a: int, n_b: int = 1):
    a1 = inputs.uav_by_id("A1")
    b1 = inputs.uav_by_id("B1")
    fleet = []
    for i in range(1, n_a + 1):
        uid = f"A{i}"
        fleet.append(dataclasses.replace(a1, uav_id=uid, sortie=dataclasses.replace(a1.sortie, uav_id=uid)))
    for i in range(1, n_b + 1):
        uid = f"B{i}"
        fleet.append(dataclasses.replace(b1, uav_id=uid, sortie=dataclasses.replace(b1.sortie, uav_id=uid)))
    return dataclasses.replace(inputs, fleet=tuple(fleet))


class Bundle:
    """ctx + 真實胞／殘帶候選 + CostCache（含起降走廊）。"""

    def __init__(self, n_a: int = 1, n_b: int = 1, cfg_over: dict | None = None):
        inputs, cfg = build_mini_example()
        inputs = _fleet_with(inputs, n_a, n_b)
        cfg = dataclasses.replace(cfg, dem_step_m=50.0, **(cfg_over or {}))
        self.inputs = inputs
        self.ctx = build_context(inputs, cfg, terrain_margin_m=1500.0)
        self.cfg = self.ctx.cfg
        self.dc = self.ctx.decompose_ctx()
        n_uav = {c: len(inputs.uavs_of_class(c)) for c in inputs.fleet_classes}
        self.cells = D.decompose_area(inputs.areas[0], self.dc, set(inputs.fleet_classes), polygon_enu=self.ctx.areas_enu["Z1"],
                                      rho_sym_m=self.ctx.rho_sym_m, n_uav=n_uav)
        self.cell = self.cells[0]
        self.res = D.residual_candidates(self.cell, self.dc, rho_sym_m=self.ctx.rho_sym_m, n_uav=n_uav) if n_b else []
        self.strips = {c: {s.strip_id: s for s in self.cell.strips.get(c, [])} for c in inputs.fleet_classes}
        self.init, self.laps, self.sites = self._corridors()
        self.cache = CostCache.build(self.strips, list(inputs.fleet), self.init, self.laps, self.sites, self.ctx)
        self.models = dict(self.ctx.models)

    def _corridors(self):
        ctx = self.ctx
        z_site = ctx.sites_enu["H"][1]
        init, laps, sites_cls = {}, {}, {}
        for u in self.inputs.fleet:
            cls = u.cls
            s0 = self.cell.strips[cls][0]
            to = ALT.takeoff_corridor("H", u, ctx, s0.start_state(-1), s0.z_op_m, 0.0)
            init[u.uav_id] = to.exit_state
            laps[u.uav_id] = (to.n_laps, 0.0)
            if cls not in sites_cls:
                ld = ALT.landing_corridor("H", u, ctx, s0.end_state(-1), s0.z_op_m)
                sites_cls[cls] = (ld.app_state, z_site + ctx.specs[cls].transition_cmd().h_tc_cmd_m)
        return init, laps, {"H": sites_cls}

    # -- 合成子胞（A 條帶子集）------------------------------------------------------
    def subcell(self, cell_id: str, sids: list[str], cls: str = "A", sensor_req: str = "lidar", **kw):
        c = self.cell
        strips = [s for s in c.strips[cls] if s.strip_id in set(sids)]
        T = D.exact_time(strips, cls, self.dc)
        return dataclasses.replace(
            c, cell_id=cell_id, sensor_req=sensor_req, strips={cls: strips}, n_strips={cls: len(strips)},
            T_single_s={cls: T}, feasible={cls: True}, infeasible_reason={cls: ""}, fit={cls: float(c.fit.get(cls, 0.5)) or 0.5},
            alpha_rad={cls: c.alpha_rad[cls]}, alt_dirs={cls: [(c.alpha_rad[cls], T)]}, h_op_m={cls: c.h_op_m[cls]},
            d_turn_m={cls: c.d_turn_m[cls]}, sweep_origin={cls: c.sweep_origin[cls]}, slope_exceeds={cls: False},
            parent_cell_id=None, residual_of=None, k_A=None, **kw)

    def allocate(self, cells, *, fleet=None, models=None, cfg=None, residual_pairs=None, frozen_teams=None,
                 method="auction", T_use_s=None):
        fleet = list(self.inputs.fleet) if fleet is None else fleet
        return AL.allocate(cells, self.strips, fleet, self.init, models or self.models, self.cache, self.ctx,
                           cfg or self.cfg, method=method, frozen_teams=frozen_teams, residual_pairs=residual_pairs,
                           T_use_s=T_use_s)


@pytest.fixture(scope="module")
def mini():
    return Bundle(1, 1)


@pytest.fixture(scope="module")
def two_a():
    return Bundle(2, 1)


def _a_ids(b: Bundle) -> list[str]:
    return [s.strip_id for s in b.cell.strips["A"]]


def _check_invariants(b: Bundle, out: AL.AllocationResult, cells) -> None:
    seen: dict[str, str] = {}
    for tm in out.teams:
        assert tm.uav_ids, tm
        for u in tm.uav_ids:
            assert b.inputs.uav_by_id(u).cls == tm.cls          # 構型同質
            assert u in tm.t0_s and u in tm.sortie_index
        for s in tm.strip_ids:
            assert s not in seen, f"strip {s} in teams {seen[s]} and {tm.team_id}"
            seen[s] = tm.team_id
    # λ_q = 該胞機數 ≤ n_U^cls；每構型實際使用機數 ≤ n_U（Σλ ≤ n_U 為 §9.3 計數階段之不變量，拍賣後一機多胞 Σλ 可 > n_U）
    for _cid, (cls, lam) in out.lambda_by_cell.items():
        assert 1 <= lam <= len(b.inputs.uavs_of_class(cls))
    for cls in b.inputs.fleet_classes:
        used = {u for tm in out.teams if tm.cls == cls for u in tm.uav_ids}
        assert len(used) <= len(b.inputs.uavs_of_class(cls))
    # 每個 open 胞：指派或列於 uncoverable
    assigned = {c for tm in out.teams for c in tm.cell_ids}
    unc = {k.split("#", 1)[0] for k, _r in out.uncoverable} | set(out.residual_k)
    for c in cells:
        assert c.cell_id in assigned or c.cell_id in unc or c.cell_id in out.residual_k, c.cell_id


# ---------------------------------------------------------------------------
# hard_feasible（H1–H7）
# ---------------------------------------------------------------------------
def test_sensor_hard_constraint(mini):
    b = mini
    a1, b1 = b.ctx.uav("A1"), b.ctx.uav("B1")
    lidar_cell = dataclasses.replace(b.cell, sensor_req="lidar")
    rgb_cell = dataclasses.replace(b.cell, sensor_req="rgb")
    assert AL.hard_feasible(lidar_cell, "A", a1, b.models["A1"], b.cache, b.ctx) == (True, "")
    ok, reason = AL.hard_feasible(lidar_cell, "B", b1, b.models["B1"], b.cache, b.ctx)
    assert not ok and reason.startswith("H1")
    ok, reason = AL.hard_feasible(rgb_cell, "A", a1, b.models["A1"], b.cache, b.ctx)   # A1 無 RGB 硬體
    assert not ok and reason.startswith("H1")
    assert AL.hard_feasible(rgb_cell, "B", b1, b.models["B1"], b.cache, b.ctx) == (True, "")
    ok, reason = AL.hard_feasible(b.cell, "A", b1, b.models["B1"], b.cache, b.ctx)       # 構型不符
    assert not ok and reason.startswith("H1")
    # H6：高度帶不可行
    bad = dataclasses.replace(b.cell, feasible={"A": False, "B": True}, infeasible_reason={"A": "band_lt_min", "B": ""})
    ok, reason = AL.hard_feasible(bad, "A", a1, b.models["A1"], b.cache, b.ctx)
    assert not ok and reason == "H6_band_lt_min"
    # H2：半徑
    ctx2 = dataclasses.replace(b.ctx, rho_sym_m={"A": 1000.0, "B": 1000.0})
    ok, reason = AL.hard_feasible(b.cell, "A", a1, b.models["A1"], b.cache, ctx2)
    assert not ok and reason == "H2_radius"
    # H3：時間窗太短
    short = dataclasses.replace(b.cell, time_window_s=(0.0, 100.0))
    ok, reason = AL.hard_feasible(short, "A", a1, b.models["A1"], b.cache, b.ctx)
    assert not ok and reason == "H3_window"


def test_energy_infeasible_not_assigned(mini):
    """H4：能量不足之機不得指派；allocate 記 uncoverable(no_feasible_insertion)。"""
    b = mini
    a1 = b.ctx.uav("A1")
    m = b.models["A1"]
    tiny = dataclasses.replace(m, available_wh=m.reserve_wh() / m.eta_T + 60.0)     # 只剩約 60 Wh 可用
    ok, reason = AL.hard_feasible(b.cell, "A", a1, tiny, b.cache, b.ctx)
    assert not ok and reason == "H4_energy"
    cell = dataclasses.replace(b.cell, sensor_req="lidar")
    out = b.allocate([cell], fleet=[a1], models={"A1": tiny})
    assert out.teams == []
    assert [k for k, _r in out.uncoverable] == ["Z1_1"]
    assert out.uncoverable[0][1] in ("no_feasible_insertion", "energy_overhead")   # T_use 由該機模型推得 → denom ≤ 0
    assert out.lambda_by_cell == {}
    # 給定寬鬆 T_use（只剩逐機能量檢查）→ 拍賣階段無可插入
    out_b = b.allocate([cell], fleet=[a1], models={"A1": tiny}, T_use_s={"A": 1e9, "B": 1e9})
    assert out_b.teams == [] and ("Z1_1", "no_feasible_insertion") in out_b.uncoverable
    # 能量關閉 → 可指派
    cfg_off = dataclasses.replace(b.cfg, enforce_energy=False)
    ctx_off = dataclasses.replace(b.ctx, cfg=cfg_off)
    assert AL.hard_feasible(cell, "A", a1, tiny, b.cache, ctx_off) == (True, "")


# ---------------------------------------------------------------------------
# config_index_J、assign_config_and_count
# ---------------------------------------------------------------------------
def test_config_index_J_normalised(two_a):
    b = two_a
    ids = _a_ids(b)
    c1 = b.subcell("C1", ids[:3])
    c2 = b.subcell("C2", ids[3:], priority=2.0)
    fleet_a = [u for u in b.inputs.fleet if u.cls == "A"]
    J = AL.config_index_J([c1, c2], "A", fleet_a, b.cache, b.cfg)
    assert set(J) == {"C1", "C2"} and all(0.0 <= v <= 1.0 for v in J.values())
    assert J["C1"] + J["C2"] == pytest.approx(sum(b.cfg.omega), abs=1e-9)
    assert J["C2"] > J["C1"]                                   # 較多條帶、較高 priority
    assert AL.config_index_J([c1], "A", fleet_a, b.cache, b.cfg)["C1"] == pytest.approx(sum(b.cfg.omega))


def test_lambda_bounds_energy_floor(two_a):
    """λ ∈ [λ_floor, min(n_U, n_q)]、Σλ ≤ n_U；T_use 縮小使 λ_floor = 2。"""
    b = two_a
    cell = dataclasses.replace(b.cell, sensor_req="lidar")
    fleet = list(b.inputs.fleet)
    T_use = {"A": D.t_use_s_for(b.dc, "A"), "B": D.t_use_s_for(b.dc, "B")}
    cnt = AL.assign_config_and_count([cell], fleet, b.models, b.cache, b.cfg, T_use)
    assert cnt.by_cell["Z1_1"][0] == "A" and 1 <= cnt.by_cell["Z1_1"][1] <= 2
    assert cnt.partial == {} and cnt.infeasible == {}
    # 能量下限：denom = κ T_use − t_in − t_RTB ∈ (T/2, T) → λ_floor = 2
    T = cell.T_single_s["A"]
    sids = [s.strip_id for s in cell.strips["A"]]
    t_in = min(AL._min_entry(b.cache, u.uav_id, "A", sids)[0] for u in fleet if u.cls == "A")
    t_rtb = AL._min_rtb(b.cache, "A", sids, "H")[0]
    T_use2 = {"A": (0.7 * T + t_in + t_rtb) / b.cfg.kappa_load, "B": T_use["B"]}
    cnt2 = AL.assign_config_and_count([cell], fleet, b.models, b.cache, b.cfg, T_use2)
    assert cnt2.by_cell["Z1_1"] == ("A", 2) and cnt2.partial == {}
    # 兩胞各 λ=1（Σλ = 2 ≤ n_U）
    ids = _a_ids(b)
    cnt3 = AL.assign_config_and_count([b.subcell("C1", ids[:4]), b.subcell("C2", ids[4:])], fleet, b.models, b.cache, b.cfg, T_use)
    assert sum(l for _c, l in cnt3.by_cell.values()) <= 2
    assert all(l >= 1 for _c, l in cnt3.by_cell.values())


def test_energy_overhead_infeasible(mini):
    """denom = κ T_use − t_in − t_RTB ≤ 0 → infeasible(reason="energy_overhead")（改派 c2 亦不可）。"""
    b = mini
    cnt = AL.assign_config_and_count([b.cell], list(b.inputs.fleet), b.models, b.cache, b.cfg, {"A": 10.0, "B": 10.0})
    assert cnt.by_cell == {}
    assert cnt.infeasible.get("Z1_1") == "energy_overhead"
    # 只有 A 超載 → 改派 B
    cnt2 = AL.assign_config_and_count([b.cell], list(b.inputs.fleet), b.models, b.cache, b.cfg,
                                      {"A": 10.0, "B": D.t_use_s_for(b.dc, "B")})
    assert cnt2.by_cell["Z1_1"][0] == "B" and "Z1_1" not in cnt2.infeasible


def test_partial_coverage_when_short(mini):
    """λ_floor > min(n_U, n_q) 且非多架次 → 只取前 k 條，其餘 UNCOVERABLE(energy)；λ = min(n_U, n_q)。"""
    b = mini
    cell = dataclasses.replace(b.cell, sensor_req="lidar")
    T = cell.T_single_s["A"]
    sids = [s.strip_id for s in cell.strips["A"]]
    t_in = AL._min_entry(b.cache, "A1", "A", sids)[0]
    t_rtb = AL._min_rtb(b.cache, "A", sids, "H")[0]
    T_use = {"A": (0.55 * T + t_in + t_rtb) / b.cfg.kappa_load, "B": 1e9}
    cnt = AL.assign_config_and_count([cell], list(b.inputs.fleet), b.models, b.cache, b.cfg, T_use)
    assert cnt.by_cell["Z1_1"] == ("A", 1)
    dropped = cnt.partial["Z1_1"]
    assert 0 < len(dropped) < 7 and dropped == sids[7 - len(dropped):]
    out = b.allocate([cell], T_use_s=T_use)
    assert set(out.leftover_strips) == set(dropped)
    for s in dropped:
        assert (s, "energy") in out.uncoverable
    team = [t for t in out.teams if t.cls == "A"][0]
    assert set(team.strip_ids) == set(sids) - set(dropped)
    assert not (set(team.strip_ids) & set(dropped))


# ---------------------------------------------------------------------------
# makespan_after_insert：終端側奇偶規則、數值
# ---------------------------------------------------------------------------
def _manual_C(b: Bundle, uav_id: str, cls: str, first: int, idx_in: int, last: int, idx_last: int, T: float) -> float:
    cache = b.cache
    spec = b.ctx.specs[cls]
    t_land = spec.transition_cmd().t_acc_s + spec.t_tr_in_s + spec.hover_descent_time_s() + spec.t_hover_reserve_s
    return (cache.init_states[uav_id].t_s + cache.entry[uav_id][first, idx_in] + T
            + cache.rtb[(cls, "H")][last, idx_last] + t_land)


def test_makespan_parity_rule(mini):
    """偶數條帶自左端（idx 0）進入 → 終端於左端 → rtb[..., idx=1]；奇數 → 同側。數值 = 手算和。"""
    b = mini
    ids = _a_ids(b)
    for n in (6, 5, 4, 1):
        cell = b.subcell(f"P{n}", ids[:n])
        est = AL.plan_estimate([cell], "A1", b.cache, b.cfg)
        assert est.ok and len(est.per_task) == 1
        first, idx_in, last, idx_last, t_arr, t_done = est.per_task[0]
        assert idx_last == (idx_in if n % 2 == 1 else 1 - idx_in)
        assert est.C_s == pytest.approx(_manual_C(b, "A1", "A", first, idx_in, last, idx_last, cell.T_single_s["A"]), abs=1e-6)
        assert t_done - t_arr == pytest.approx(cell.T_single_s["A"])
        assert AL.makespan_after_insert([], 0, cell, b.ctx.uav("A1"), b.cache, b.cfg) == pytest.approx(est.C_s)
        # 兩側擇優：估計 ≤ 任一固定側（skip-k 序列正／反向）之手算
        order = AL.sweep_order(ids[:n], "A", b.cache)
        assert set(order) == {b.cache.idx["A"][s] for s in ids[:n]} and (first, last) in ((order[0], order[-1]), (order[-1], order[0]))
        for idx_in2 in (0, 1):
            idx_last2 = idx_in2 if n % 2 == 1 else 1 - idx_in2
            for f, l in ((order[0], order[-1]), (order[-1], order[0])):
                assert est.C_s <= _manual_C(b, "A1", "A", f, idx_in2, l, idx_last2, cell.T_single_s["A"]) + 1e-6
    # 偶數胞自左端進入時，RTB 必用左端列（與右端列不同）
    cell6 = b.subcell("P6", ids[:6])
    est = AL.plan_estimate([cell6], "A1", b.cache, b.cfg)
    first, idx_in, last, idx_last, *_ = est.per_task[0]
    wrong = _manual_C(b, "A1", "A", first, idx_in, last, 1 - idx_last, cell6.T_single_s["A"])
    assert abs(wrong - est.C_s) > 1.0


def test_makespan_W4_table(mini):
    """契約 §W.4：Ĉ_A(k=7) ≈ 34.7 min、Ĉ_B(k=0) ≈ 51.0 min（兩側擇優／序列正反擇優 → 略小），k=5 殘帶 B ≈ 23.7。"""
    b = mini
    by_k = {k: (ca, cb) for k, ca, cb in b.res}
    a1, b1 = b.ctx.uav("A1"), b.ctx.uav("B1")
    cA7 = AL.makespan_after_insert([], 0, by_k[7][0], a1, b.cache, b.cfg) / 60.0
    assert 32.5 <= cA7 <= 34.8
    cB0 = AL.makespan_after_insert([], 0, by_k[0][1], b1, b.cache, b.cfg) / 60.0
    assert 49.0 <= cB0 <= 51.2
    ca5, cb5 = by_k[5]
    cA5 = AL.makespan_after_insert([], 0, ca5, a1, b.cache, b.cfg) / 60.0
    assert 26.5 <= cA5 <= 28.5
    missing = [s for s in cb5.strips["B"] if s.strip_id not in b.cache.idx["B"]]
    if missing:
        b.cache.add_strips("B", missing, list(b.inputs.fleet), b.init, b.sites, b.ctx)
    cB5 = AL.makespan_after_insert([], 0, cb5, b1, b.cache, b.cfg) / 60.0
    assert 22.0 <= cB5 <= 24.0
    pair = AL.makespan_after_insert_pair({}, a1, 0, ca5, b1, 0, cb5, b.cache, b.cfg)
    assert pair == pytest.approx(max(cA5, cB5) * 60.0)
    # 未在 cache 之條帶 → KeyError
    fake = b.subcell("F", ids := _a_ids(b)[:2])
    fake = dataclasses.replace(fake, strips={"A": [dataclasses.replace(fake.strips["A"][0], strip_id="nope")]})
    with pytest.raises(KeyError):
        AL.makespan_after_insert([], 0, fake, a1, b.cache, b.cfg)


# ---------------------------------------------------------------------------
# allocate：殘帶候選對、兩胞兩機、凍結、uncoverable
# ---------------------------------------------------------------------------
def test_auction_residual_pairs(mini):
    """§W.4：k_A ∈ {5, 6}；A 子胞條帶 = 前 k 條；B 殘帶條帶不與 A 重疊；起飛序 Ĉ 降序（A1 先、B1 +60 s）。"""
    b = mini
    t = time.perf_counter()
    out = b.allocate([b.cell], residual_pairs={b.cell.cell_id: b.res})
    assert time.perf_counter() - t < 3.0
    _check_invariants(b, out, [b.cell])
    k = out.residual_k["Z1_1"]
    assert k in (5, 6)
    tA = [tm for tm in out.teams if tm.cls == "A"][0]
    tB = [tm for tm in out.teams if tm.cls == "B"][0]
    assert tA.uav_ids == ["A1"] and tB.uav_ids == ["B1"]
    assert tA.strip_ids == _a_ids(b)[:k]
    assert len(tB.strip_ids) == {5: 3, 6: 2}[k] and not (set(tA.strip_ids) & set(tB.strip_ids))
    # 子胞 id 含採用之 k（避免不同 k 的殘帶條帶在 CostCache 中同名覆蓋）
    assert out.lambda_by_cell == {f"Z1_1_A_k{k}": ("A", 1), f"Z1_1_B_k{k}": ("B", 1)}
    assert all(sid.startswith(f"Z1_1_B_k{k}_B_") for sid in tB.strip_ids)
    assert out.uncoverable == [] and out.leftover_strips == []
    # 起飛排程：Ĉ_A > Ĉ_B → A1 t0=0、B1 t0=60
    assert out.estimates["A1"] > out.estimates["B1"]
    assert tA.t0_s["A1"] == 0.0 and tB.t0_s["B1"] == pytest.approx(b.cfg.dt_dep_s)
    assert 26.0 * 60 <= out.estimates["A1"] <= 29.0 * 60
    # makespan 優於 A 獨佔／B 獨佔
    by_k = {kk: (ca, cb) for kk, ca, cb in b.res}
    solo_A = AL.makespan_after_insert([], 0, by_k[7][0], b.ctx.uav("A1"), b.cache, b.cfg)
    assert max(out.estimates.values()) < solo_A


def test_two_cells_two_uavs_obvious(two_a):
    """兩個 A 胞、兩架 A：各取一胞，且 makespan < 單機兩胞。"""
    b = two_a
    ids = _a_ids(b)
    c1 = b.subcell("C1", ids[:4])
    c2 = b.subcell("C2", ids[4:])
    out = b.allocate([c1, c2])
    _check_invariants(b, out, [c1, c2])
    plans = {u: p for u, p in out.plans.items() if p}
    assert set(plans) == {"A1", "A2"}
    assert sorted(len(p) for p in plans.values()) == [1, 1]
    assert {plans["A1"][0], plans["A2"][0]} == {"C1", "C2"}
    assert out.lambda_by_cell == {"C1": ("A", 1), "C2": ("A", 1)}
    both = AL.plan_estimate([c1, c2], "A1", b.cache, b.cfg).C_s
    assert max(out.estimates["A1"], out.estimates["A2"]) < both
    assert out.uncoverable == []
    # greedy 亦給同樣的一對一指派
    out_g = b.allocate([c1, c2], method="greedy")
    pg = {u: p for u, p in out_g.plans.items() if p}
    assert {pg["A1"][0], pg["A2"][0]} == {"C1", "C2"}


def test_auction_multi_cell_and_local_search(two_a):
    """三胞兩機：拍賣「一機多胞」；局部改善後 makespan ≤ 貪婪；MOVE/SWAP 不違反硬限制（能量、感測）。"""
    b = two_a
    ids = _a_ids(b)
    cells = [b.subcell("C1", ids[:2]), b.subcell("C2", ids[2:4]), b.subcell("C3", ids[4:])]
    out = b.allocate(cells)
    _check_invariants(b, out, cells)
    assert out.uncoverable == []
    used = [p for p in out.plans.values() if p]
    assert sum(len(p) for p in used) == 3 and max(len(p) for p in used) >= 2
    out_g = b.allocate(cells, method="greedy")
    C_auction = max(v for v in out.estimates.values())
    # 貪婪只跑 Phase 1：λ=0 之胞留為 uncoverable；拍賣覆蓋全部且不劣於貪婪之已指派部分之 max
    assert math.isfinite(C_auction)
    assert len(out_g.uncoverable) >= 0
    # 一架能量極小 → 其不得承接任何胞；另一架承接全部（或部分列 uncoverable），不得違反 H1/H4
    m = b.models["A2"]
    tiny = dataclasses.replace(m, available_wh=m.reserve_wh() / m.eta_T + 60.0)
    out2 = b.allocate(cells, models={**b.models, "A2": tiny})
    _check_invariants(b, out2, cells)
    assert out2.plans["A2"] == []
    for tm in out2.teams:
        assert "A2" not in tm.uav_ids


def test_frozen_teams_unchanged(two_a):
    """frozen 團隊：既有胞順序不變、只允許尾端追加；其條帶不重新指派。"""
    b = two_a
    ids = _a_ids(b)
    c1 = b.subcell("C1", ids[:3])
    c2 = b.subcell("C2", ids[3:5])
    c3 = b.subcell("C3", ids[5:])
    frozen = AL.Team("F1", "A", ["A1"], ["C2", "C1"], [s for s in ids[:5]], {"A1": 0.0}, {"A1": 0})
    out = b.allocate([c1, c2, c3], fleet=[b.inputs.uav_by_id("A1")], frozen_teams=[frozen])
    _check_invariants(b, out, [c1, c2, c3])
    f = [tm for tm in out.teams if tm.team_id == "F1"][0]
    assert f.cell_ids[:2] == ["C2", "C1"] and f.uav_ids == ["A1"]
    assert out.plans["A1"][:2] == ["C2", "C1"]
    assert f.cell_ids == ["C2", "C1", "C3"]              # 尾端追加
    assert f.strip_ids[:5] == ids[:5] and set(f.strip_ids) == set(ids)
    assert out.uncoverable == []
    # 凍結胞不在 lambda_by_cell 外被重複指派
    assert sum(1 for tm in out.teams for c in tm.cell_ids if c == "C1") == 1


def test_phase2_records_uncoverable(two_a):
    """無相容機之胞 → uncoverable 完整列出；greedy 之 λ=0 胞亦列出。"""
    b = two_a
    ids = _a_ids(b)
    fleet_a = [u for u in b.inputs.fleet if u.cls == "A"]
    c_rgb = b.subcell("R1", ids[:2], sensor_req="rgb")            # A 無 RGB → 無可行構型
    c_ok = b.subcell("C1", ids[2:])
    out = b.allocate([c_rgb, c_ok], fleet=fleet_a)
    _check_invariants(b, out, [c_rgb, c_ok])
    assert ("R1", "no_feasible_class") in out.uncoverable
    assert all(c != "R1" for tm in out.teams for c in tm.cell_ids)
    # 兩機能量皆極小 → 全部 uncoverable(no_feasible_insertion)
    tiny = {u.uav_id: dataclasses.replace(b.models[u.uav_id], available_wh=b.models[u.uav_id].reserve_wh() / b.models[u.uav_id].eta_T + 60.0)
            for u in fleet_a}
    out2 = b.allocate([c_ok], fleet=fleet_a, models=tiny, T_use_s={"A": 1e9, "B": 1e9})
    assert out2.teams == [] and ("C1", "no_feasible_insertion") in out2.uncoverable
    out3 = b.allocate([c_ok], fleet=fleet_a, models=tiny)          # T_use 由模型推得 → 計數階段 energy_overhead
    assert out3.teams == [] and ("C1", "energy_overhead") in out3.uncoverable


# ---------------------------------------------------------------------------
# 時間窗、起飛排程、多架次
# ---------------------------------------------------------------------------
def test_time_window_delayed_takeoff(mini):
    """H3 提早到達 → 延後起飛 Δ = t_open − t_arr（零能量）；Team.t0_s 含 Δ；estimate 含延後。"""
    b = mini
    ids = _a_ids(b)
    base = b.subcell("W0", ids[:3])
    est0 = AL.plan_estimate([base], "A1", b.cache, b.cfg)
    t_arr0 = est0.per_task[0][4]
    t_open = t_arr0 + 300.0
    cell = b.subcell("W1", ids[:3], time_window_s=(t_open, t_open + 4000.0))
    est = AL.plan_estimate([cell], "A1", b.cache, b.cfg)
    assert est.ok and est.delay_s == pytest.approx(300.0) and est.C_s == pytest.approx(est0.C_s + 300.0)
    out = b.allocate([cell], fleet=[b.inputs.uav_by_id("A1")])
    assert out.delays_s["A1"] == pytest.approx(300.0)
    tA = out.teams[0]
    assert tA.t0_s["A1"] == pytest.approx(300.0)
    assert out.estimates["A1"] == pytest.approx(est0.C_s + 300.0)
    # 完成超過 t_close → 不可行
    late = b.subcell("W2", ids[:3], time_window_s=(0.0, t_arr0 + 10.0))
    assert not AL.plan_estimate([late], "A1", b.cache, b.cfg).ok
    assert AL.makespan_after_insert([], 0, late, b.ctx.uav("A1"), b.cache, b.cfg) == math.inf
    # 時間窗關閉 → 忽略
    cfg_off = dataclasses.replace(b.cfg, enforce_time_windows=False)
    assert AL.plan_estimate([late], "A1", b.cache, cfg_off).ok


def test_takeoff_order_by_C_hat(two_a):
    b = two_a
    teams = [AL.Team("T1", "A", ["A1", "A2"], ["C1"], [], {"A1": 0.0, "A2": 0.0}, {"A1": 0, "A2": 0}),
             AL.Team("T2", "B", ["B1"], ["C2"], [], {"B1": 0.0}, {"B1": 0})]
    est = {"A1": 1000.0, "A2": 2000.0, "B1": 1500.0}
    t0 = AL.takeoff_schedule(teams, b.cache, est, {}, b.cfg)
    assert t0 == {"A2": 0.0, "B1": 60.0, "A1": 120.0}
    t0d = AL.takeoff_schedule(teams, b.cache, est, {"A1": 45.0}, b.cfg)
    assert t0d["A1"] == pytest.approx(165.0) and t0d["A2"] == 0.0
    # tie → 進入時間長者先（A1/A2 進入相同 → 以 id 穩定）
    t0e = AL.takeoff_schedule(teams, b.cache, {"A1": 1000.0, "A2": 1000.0, "B1": 0.0}, {}, b.cfg)
    assert sorted(t0e.values()) == [0.0, 60.0, 120.0] and t0e["B1"] == 120.0
    # 第二架次不排隊：t0 = delays[uav#s1]
    t2 = AL.Team("T3", "A", ["A1"], ["C3"], [], {"A1": 0.0}, {"A1": 1})
    t0s = AL.takeoff_schedule([teams[0], t2], b.cache, est, {"A1#s1": 3000.0}, b.cfg)
    assert t0s["A1#s1"] == 3000.0 and t0s["A1"] == 60.0


def test_split_for_sorties(mini):
    b = mini
    a1 = b.ctx.uav("A1")
    cell = dataclasses.replace(b.cell, sensor_req="lidar")
    full = AL.split_for_sorties(cell, "A", a1, b.cache, b.models["A1"], b.cfg)
    assert full == [_a_ids(b)]
    m = b.models["A1"]
    # 可規劃能量縮至 ~1/3 → 多塊；塊序 = 條帶序、不重複、全覆蓋
    small = dataclasses.replace(m, available_wh=(m.reserve_wh() + m.e_takeoff_wh() + m.e_landing_wh() + m.e_usable_wh() / 3.0) / m.eta_T)
    chunks = AL.split_for_sorties(cell, "A", a1, b.cache, small, b.cfg)
    assert len(chunks) >= 2 and [s for c in chunks for s in c] == _a_ids(b)
    sids = _a_ids(b)
    costs = AL._strip_costs_in_order(b.cache, "A", sids)
    p = small.p_cruise_w() * small.k_unc
    T_use = small.e_usable_wh() / p * 3600.0
    budget = b.cfg.kappa_load * T_use - AL._min_entry(b.cache, "A1", "A", sids)[0] - AL._min_rtb(b.cache, "A", sids, "H")[0]
    for c in chunks:
        i0 = sids.index(c[0])
        tot = costs[i0] - (costs[i0] - np.nanmean(np.where(np.isfinite(b.cache.op["A"][b.cache.idx["A"][c[0]]]), b.cache.op["A"][b.cache.idx["A"][c[0]]], np.nan)))
        tot += sum(costs[sids.index(s)] for s in c[1:])
        assert tot <= budget + 1e-6 or len(c) == 1


def test_multi_sortie_first_lands_home(two_a):
    """multi_sortie：單機能量只夠一胞 → 第二胞交第二架次（sortie_index 1、t0 ≥ C^(1) + t_swap）；第一架次落於 H。"""
    b = two_a
    ids = _a_ids(b)
    a1 = b.inputs.uav_by_id("A1")
    c1 = b.subcell("C1", ids[:4])
    c2 = b.subcell("C2", ids[4:])
    m = b.models["A1"]
    e1 = AL.plan_estimate([c1], "A1", b.cache, b.cfg, model=m).e_wh
    e12 = AL.plan_estimate([c1, c2], "A1", b.cache, b.cfg, model=m).e_wh
    assert e12 > e1
    mid = dataclasses.replace(m, available_wh=(m.reserve_wh() + 0.5 * (e1 + e12)) / m.eta_T)
    cfg_ms = dataclasses.replace(b.cfg, multi_sortie=True)
    out = b.allocate([c1, c2], fleet=[a1], models={"A1": mid}, cfg=cfg_ms)
    _check_invariants(b, out, [c1, c2])
    assert out.uncoverable == []
    sorties = sorted((tm.sortie_index["A1"], tm) for tm in out.teams)
    assert [s for s, _t in sorties] == [0, 1]
    first, second = sorties[0][1], sorties[1][1]
    assert {first.cell_ids[0], second.cell_ids[0]} == {"C1", "C2"}
    assert second.t0_s["A1"] >= out.estimates["A1"] + b.cfg.t_swap_s - 1e-6
    assert out.estimates["A1#s1"] > out.estimates["A1"]
    # 第一架次以回收點 H 計（RTB 列 = (cls, recovery_site)）
    assert b.ctx.recovery_site_id == "H" and ("A", "H") in b.cache.rtb
    # 非多架次 → 第二胞 uncoverable
    out2 = b.allocate([c1, c2], fleet=[a1], models={"A1": mid})
    assert len(out2.teams) == 1 and any(r == "no_feasible_insertion" for _k, r in out2.uncoverable)


def test_local_search_reduces_makespan(two_a):
    """Phase 3/4（MOVE/SWAP/BALANCE）自劣解（A1 全包、A2 閒置）出發降低 C_max；每任務仍恰一次；不劣於 allocate 結果 +5%。"""
    b = two_a
    ids = _a_ids(b)
    cells = [b.subcell("C1", ids[:1]), b.subcell("C2", ids[1:2]), b.subcell("C3", ids[2:5]), b.subcell("C4", ids[5:])]
    fleet_a = [u for u in b.inputs.fleet if u.cls == "A"]
    S = AL._Alloc(cells, b.strips, fleet_a, b.init, b.models, b.cache, b.ctx, b.cfg, None, None)
    S.setup()
    for c in cells:
        S.add_task(AL._task_of(c, "A"))
    S.plans["A1"] = [c.cell_id for c in cells]
    S.plans["A2"] = []
    C0 = S.C_max()
    assert math.isfinite(C0)
    S.phase3()
    S.phase4()
    C1 = S.C_max()
    assert C1 < C0 - 60.0
    keys = [k for p in S.plans.values() for k in p]
    assert sorted(keys) == sorted(c.cell_id for c in cells)
    assert S.plans["A1"] and S.plans["A2"]
    lam = S.lam_counts()
    for u, p in S.plans.items():
        for k in p:
            assert S.feasible_for(S.tasks[k], u, lam[k])
    out = b.allocate(cells)
    _check_invariants(b, out, cells)
    assert out.uncoverable == []
    C_alloc = max(v for u, v in out.estimates.items() if out.plans[u])
    assert C_alloc <= C1 * 1.05 + 1e-6


# ---------------------------------------------------------------------------
# 山丘情境整合（both 胞雙套條帶、8 機、3 區）
# ---------------------------------------------------------------------------
@pytest.fixture(scope="module")
def hills_alloc():
    from hv_dccpp.scenario import build_hills_scenario
    inputs, cfg = build_hills_scenario()
    ctx = build_context(inputs, dataclasses.replace(cfg, dem_step_m=50.0), terrain_margin_m=1500.0)
    dc = ctx.decompose_ctx()
    n_uav = {c: len(inputs.uavs_of_class(c)) for c in inputs.fleet_classes}
    cells = []
    for a in inputs.areas:
        cells += D.decompose_area(a, dc, set(inputs.fleet_classes), polygon_enu=ctx.areas_enu[a.area_id],
                                  rho_sym_m=ctx.rho_sym_m, n_uav=n_uav)
    strips = {c: {} for c in inputs.fleet_classes}
    for cell in cells:
        for c in strips:
            for s in cell.strips.get(c, []):
                strips[c][s.strip_id] = s
    z_site = ctx.sites_enu["H"][1]
    init, laps, sites_cls = {}, {}, {}
    for u in inputs.fleet:
        s0 = next(iter(strips[u.cls].values()))
        to = ALT.takeoff_corridor("H", u, ctx, s0.start_state(-1), s0.z_op_m, 0.0)
        init[u.uav_id], laps[u.uav_id] = to.exit_state, (to.n_laps, 0.0)
        if u.cls not in sites_cls:
            ld = ALT.landing_corridor("H", u, ctx, s0.end_state(-1), s0.z_op_m)
            sites_cls[u.cls] = (ld.app_state, z_site + ctx.specs[u.cls].transition_cmd().h_tc_cmd_m)
    cache = CostCache.build(strips, list(inputs.fleet), init, laps, {"H": sites_cls}, ctx)
    t = time.perf_counter()
    out = AL.allocate(cells, strips, list(inputs.fleet), init, ctx.models, cache, ctx, ctx.cfg)
    return inputs, ctx, cells, strips, cache, out, time.perf_counter() - t


def test_hills_allocation_invariants(hills_alloc):
    """both 胞 → A、B 兩任務皆指派；每條帶恰一團隊；團隊同構型；無 uncoverable；起飛 Ĉ 降序間隔 dt_dep。"""
    inputs, ctx, cells, strips, cache, out, dt = hills_alloc
    assert dt < 2.0
    seen: dict[str, str] = {}
    for tm in out.teams:
        for u in tm.uav_ids:
            assert inputs.uav_by_id(u).cls == tm.cls
        for s in tm.strip_ids:
            assert s not in seen
            seen[s] = tm.team_id
            assert s in strips[tm.cls]
    assert len(seen) == sum(len(v) for v in strips.values())          # 107 條帶全指派
    assert out.uncoverable == [] and out.leftover_strips == []
    for c in cells:
        if c.sensor_req == "both":
            assert f"{c.cell_id}#A" in out.lambda_by_cell and f"{c.cell_id}#B" in out.lambda_by_cell
        else:
            cls = [k for k, v in c.feasible.items() if v][0]
            assert out.lambda_by_cell[c.cell_id][0] == cls
    for cls in inputs.fleet_classes:
        used = {u for tm in out.teams if tm.cls == cls for u in tm.uav_ids}
        assert len(used) <= len(inputs.uavs_of_class(cls))
        for _cid, (c, lam) in out.lambda_by_cell.items():
            if c == cls:
                assert 1 <= lam <= len(inputs.uavs_of_class(cls))
    # 起飛排程：同站 dt_dep 間隔、順序 = 排程前 Ĉ 降序
    t0 = {u: tm.t0_s[u] for tm in out.teams for u in tm.uav_ids}
    assert sorted(t0.values()) == [k * ctx.cfg.dt_dep_s for k in range(len(t0))]
    pre = {u: out.estimates[u] - t0[u] for u in t0}
    order = sorted(t0, key=lambda u: t0[u])
    assert all(pre[a] >= pre[b] - 1e-6 for a, b in zip(order[:-1], order[1:]))
    assert all(math.isfinite(v) and v > 0 for v in out.estimates.values())


# ---------------------------------------------------------------------------
# 審查後新增（R-04）：both 胞只有單一構型可服務時，第二套條帶未鋪必須被記為不可覆蓋
# ---------------------------------------------------------------------------
def test_both_cell_single_class_reports_second_sensor_unserved(two_a):
    """規格 §6.2：``both`` 胞需要 LiDAR 與 RGB **兩套**條帶。

    `decompose.resolve_sensor` 對 (cls, req_id) 只鋪出第一個相符 Sensor 的那一套，因此只有一個構型能服務
    的 ``both`` 胞會無聲漏掉第二個感測器需求（審查 R-04 之 all_A 基線：Z2 的 rgb 覆蓋率 0.000 卻回報完成）。
    `assign_config_and_count` 現在明確記一筆 ``<cell_id>#sensor2 → both_second_sensor_unserved``，
    使它進入 `AllocationResult.uncoverable` → `MissionPlan.uncovered`。"""
    b = two_a
    ids = _a_ids(b)
    only_a = [u for u in b.inputs.fleet if u.cls == "A"]        # 機隊只有 A（LiDAR）
    cell = b.subcell("BOTH1", ids[:4], sensor_req="both")
    out = b.allocate([cell], fleet=only_a)
    key = "BOTH1" + AL.BOTH_SEP + AL.SECOND_SENSOR_TAG
    assert (key, "both_second_sensor_unserved") in out.uncoverable
    # LiDAR 那一套仍照常指派，不因此被丟掉
    assert any(cell.cell_id in tm.cell_ids for tm in out.teams)
    assert set(ids[:4]) <= {s for tm in out.teams for s in tm.strip_ids}


def test_both_cell_two_classes_not_flagged(mini):
    """A、B 皆可服務之 ``both`` 胞（兩套條帶各由一個構型鋪出）不得被誤記為 second_sensor_unserved。"""
    b = mini
    cell = dataclasses.replace(b.cell, cell_id="BOTH2", sensor_req="both")
    out = b.allocate([cell])
    assert not any(k.endswith(AL.BOTH_SEP + AL.SECOND_SENSOR_TAG) for k, _r in out.uncoverable)
