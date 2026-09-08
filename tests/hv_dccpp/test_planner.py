"""tests/test_planner.py — 契約 §10 `planner.py` 不變量與規格 §17.2 清單（direction_equals_transfer_terminal、each_strip_once、
beam1_paper_metric_equals_full_dp、event_driven_beats_lockstep_unequal_strips、knearest_within_2pct、per_uav_rtb_keeps_others、
same_direction_transfer_one_way_cell、append_rtb_home_first、energy_repair_truncates_at_xj、cross_cell_chain_prefers_cell_completion、
beam_sorted_by_W、no_hover_outside_vtol_phases、stage_cost_vectorized_signature）。

情境：`scenario.build_mini_example()`（平地、西風 5 m/s）以 `dem_step_m=50`、`terrain_margin_m=1500` 建構；機隊擴為 A1–A3、B1–B2；
真實 `decompose_area` 之胞（A 7 條／B 12 條）建 CostCache；「8 長 + 16 短」以合成條帶建第二個快取。
"""
from __future__ import annotations

import dataclasses
import itertools
import math
import time

import numpy as np
import pytest

from hv_dccpp import allocate as AL
from hv_dccpp import altitude as ALT
from hv_dccpp import decompose as D
from hv_dccpp import planner as P
from hv_dccpp.context import build_context
from hv_dccpp.costs import CostCache
from hv_dccpp.geometry import Strip
from hv_dccpp.mission import FW_PHASES, K_INF, Mode, Phase, UAVState
from hv_dccpp.scenario import build_mini_example

EAST = math.pi / 2


# ---------------------------------------------------------------------------
# fixtures
# ---------------------------------------------------------------------------
def _fleet_with(inputs, n_a: int, n_b: int):
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


def _corridors(ctx, inputs, strips):
    """每機暫定起飛走廊（入口 = 該構型首條帶左端）與每站每構型之 S^app。"""
    init, laps, sites = {}, {}, {}
    for u in inputs.fleet:
        s0 = next(iter(strips[u.cls].values()))
        to = ALT.takeoff_corridor("H", u, ctx, s0.start_state(-1), s0.z_op_m, 0.0)
        init[u.uav_id], laps[u.uav_id] = to.exit_state, (to.n_laps, 0.0)
    for sid in ("H", "ALT1"):
        sites[sid] = {}
        z_site = ctx.sites_enu[sid][1]
        for cls in strips:
            u = next(x for x in inputs.fleet if x.cls == cls)
            s0 = next(iter(strips[cls].values()))
            ld = ALT.landing_corridor(sid, u, ctx, s0.end_state(-1), s0.z_op_m)
            sites[sid][cls] = (ld.app_state, z_site + ctx.specs[cls].transition_cmd().h_tc_cmd_m)
    return init, laps, sites


class Bundle:
    def __init__(self, n_a: int = 3, n_b: int = 2, margin: float = 1500.0):
        inputs, cfg = build_mini_example()
        inputs = _fleet_with(inputs, n_a, n_b)
        self.inputs = inputs
        self.ctx = build_context(inputs, dataclasses.replace(cfg, dem_step_m=50.0), terrain_margin_m=margin)
        self.cfg = self.ctx.cfg
        self.dc = self.ctx.decompose_ctx()
        n_uav = {c: len(inputs.uavs_of_class(c)) for c in inputs.fleet_classes}
        self.cells = D.decompose_area(inputs.areas[0], self.dc, set(inputs.fleet_classes), polygon_enu=self.ctx.areas_enu["Z1"],
                                      rho_sym_m=self.ctx.rho_sym_m, n_uav=n_uav)
        self.cell = self.cells[0]
        self.strips = {c: {s.strip_id: s for s in self.cell.strips[c]} for c in ("A", "B") if self.cell.strips.get(c)}
        self.init, self.laps, self.sites = _corridors(self.ctx, inputs, self.strips)
        self.cache = CostCache.build(self.strips, list(inputs.fleet), self.init, self.laps, self.sites, self.ctx)
        self.models = dict(self.ctx.models)

    def team(self, cls: str, uavs: list[str], strip_ids: list[str] | None = None) -> AL.Team:
        sids = list(self.strips[cls].keys()) if strip_ids is None else list(strip_ids)
        return AL.Team(f"T{cls}", cls, list(uavs), [self.cell.cell_id], sids, {u: 0.0 for u in uavs}, {u: 0 for u in uavs})

    def idp(self, cls: str, uavs: list[str], strip_ids=None, cfg=None, init=None, **kw) -> P.IDPResult:
        return P.beam_idp(self.team(cls, uavs, strip_ids), self.strips[cls], self.cache, self.models,
                          init or self.init, cfg or self.cfg, **kw)


@pytest.fixture(scope="module")
def mini() -> Bundle:
    return Bundle(3, 2)


def _all_items(res: P.IDPResult) -> list[tuple[str, int]]:
    return [(it.strip_id, it.dir) for seq in res.sequences.values() for it in seq.items]


def _assert_each_once(res: P.IDPResult, strip_ids: list[str]) -> None:
    got = [s for s, _d in _all_items(res)]
    assert sorted(got) == sorted(strip_ids), (got, res.leftover)
    assert len(set(got)) == len(got)
    assert res.leftover == []


def _assert_dir_consistent(res: P.IDPResult, strips: dict[str, Strip], cache: CostCache, cls: str) -> None:
    """Eq.26：每項 dir = 其前一轉移終端狀態方向（路徑終點 == strips[m].start_state(dir)）；首條 dir 與 entry 一致。"""
    for seq in res.sequences.values():
        if not seq.items:
            continue
        first = seq.items[0]
        p0 = seq.entry.path
        assert p0 is not None
        tgt = strips[first.strip_id].start_state(first.dir)
        assert math.hypot(p0.end.x_m - tgt.x_m, p0.end.y_m - tgt.y_m) < 1e-6
        assert abs((p0.end.theta_rad - tgt.theta_rad + math.pi) % (2 * math.pi) - math.pi) < 1e-9
        for prev, it in zip(seq.items, seq.items[1:]):
            assert it.transfer is not None and it.transfer.path is not None
            p = it.transfer.path
            s_from = strips[prev.strip_id].end_state(prev.dir)
            s_to = strips[it.strip_id].start_state(it.dir)
            for a, b in ((p.start, s_from), (p.end, s_to)):
                assert math.hypot(a.x_m - b.x_m, a.y_m - b.y_m) < 1e-6
                assert abs((a.theta_rad - b.theta_rad + math.pi) % (2 * math.pi) - math.pi) < 1e-9
            k = 2 * ((prev.dir + 1) // 2) + (it.dir + 1) // 2
            l, m = cache.idx[cls][prev.strip_id], cache.idx[cls][it.strip_id]
            assert math.isfinite(cache.tr[cls][l, m, k])
            # 反向型 (k ∈ {1,2}) → dir 相反；同向型 (k ∈ {0,3}) → dir 相同
            assert (prev.dir != it.dir) == (k in (1, 2))


# ---------------------------------------------------------------------------
# each_strip_once / direction_equals_transfer_terminal
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("branching", ["event", "lockstep"])
def test_each_strip_once_and_direction(mini: Bundle, branching):
    cfg = dataclasses.replace(mini.cfg, idp_branching=branching)
    for cls, uavs in (("A", ["A1", "A2"]), ("B", ["B1", "B2"]), ("A", ["A1"])):
        res = mini.idp(cls, uavs, cfg=cfg)
        _assert_each_once(res, list(mini.strips[cls]))
        _assert_dir_consistent(res, mini.strips[cls], mini.cache, cls)
        assert math.isfinite(res.makespan_s) and res.makespan_s > 0
        assert res.nodes_expanded > 0
        for seq in res.sequences.values():
            for it in seq.items:
                assert it.t_end_s >= it.t_start_s
            for a, b in zip(seq.items, seq.items[1:]):
                assert b.t_start_s >= a.t_end_s - 1e-9
                assert b.e_end_wh <= a.e_end_wh + 1e-9
            if seq.items:
                assert seq.t_end_s == pytest.approx(seq.items[-1].t_end_s)
                assert seq.e_end_wh == pytest.approx(seq.items[-1].e_end_wh)


def test_single_uav_matches_bruteforce(mini: Bundle):
    """單機、B 之前 4 條：IDP（B=3, K=6）makespan 與全排列暴力搜尋之差 ≤ 1%。"""
    cls = "B"
    ids = list(mini.strips[cls])[:4]
    cache = mini.cache
    ent, op, tr, rtb = cache.entry["B1"], cache.op[cls], cache.tr[cls], cache.rtb[(cls, "H")]
    idx = [cache.idx[cls][s] for s in ids]
    best = math.inf
    for perm in itertools.permutations(idx):
        for dirs in itertools.product((0, 1), repeat=len(idx)):
            t = mini.init["B1"].t_s + ent[perm[0], dirs[0]] + op[perm[0], dirs[0]]
            for a in range(1, len(idx)):
                t += tr[perm[a - 1], perm[a], 2 * dirs[a - 1] + dirs[a]] + op[perm[a], dirs[a]]
            t += rtb[perm[-1], dirs[-1]]
            best = min(best, t)
    res = mini.idp(cls, ["B1"], ids)
    _assert_each_once(res, ids)
    t_idp = res.makespan_s - 106.0
    assert t_idp <= best * 1.01 + 1.0


# ---------------------------------------------------------------------------
# beam1_paper_metric_equals_full_dp（n=6, λ=2, 鎖步、B=1、K=∞、metric="paper"、只反向）
# ---------------------------------------------------------------------------
def _paper_greedy(cache: CostCache, cls: str, uavs: list[str], ids: list[str], init: dict, tau0: float):
    """論文 Algorithm 2 之 B=1／K=∞ 鎖步：每階段完整列舉聯合決策（條帶互異、只反向轉場），取 Σ Δt_i/(T_i+τ0) 最小。"""
    idxs = [cache.idx[cls][s] for s in ids]
    nad = set(idxs)
    state = {u: (-1, -1, float(init[u].t_s)) for u in uavs}     # (last, idx_last, T)
    seqs = {u: [] for u in uavs}
    while nad:
        opts = {}
        for u in uavs:
            l, il, T = state[u]
            o = []
            for m in nad:
                for k in ((0, 1) if l < 0 else (2 * il + (1 - il),)):
                    t_tr = cache.entry[u][m, k] if l < 0 else cache.tr[cls][l, m, k]
                    t = t_tr + cache.op[cls][m, k % 2]
                    if math.isfinite(t):
                        o.append((m, k, t / (T + tau0)))
            opts[u] = o
        best = None
        for combo in itertools.product(*[opts[u] for u in uavs]):
            ms = [c[0] for c in combo]
            if len(set(ms)) < len(ms):
                continue
            cost = sum(c[2] for c in combo)
            if best is None or cost < best[0]:
                best = (cost, combo)
        assert best is not None
        for u, (m, k, rel) in zip(uavs, best[1]):
            l, il, T = state[u]
            t_tr = cache.entry[u][m, k] if l < 0 else cache.tr[cls][l, m, k]
            state[u] = (m, k % 2, T + t_tr + cache.op[cls][m, k % 2])
            seqs[u].append((cache.strip_ids[cls][m], -1 if k % 2 == 0 else 1))
            nad.discard(m)
    return seqs


def test_beam1_paper_metric_equals_full_dp(mini: Bundle):
    cls = "A"
    ids = list(mini.strips[cls])[:6]
    uavs = ["A1", "A2"]
    cfg = dataclasses.replace(mini.cfg, idp_branching="lockstep", B=1, K=K_INF, metric="paper", transfer_types=("rev",),
                              enforce_energy=False, t_rtb_lower_bound=False)
    res = mini.idp(cls, uavs, ids, cfg=cfg)
    _assert_each_once(res, ids)
    ref = _paper_greedy(mini.cache, cls, uavs, ids, mini.init, cfg.tau0_s)
    # A1/A2 初始狀態完全相同 → 聯合決策對稱平手，只比較序列多重集合
    got = sorted([(it.strip_id, it.dir) for it in res.sequences[u].items] for u in uavs)
    assert got == sorted(ref[u] for u in uavs)
    for u in uavs:
        # 只反向：連續 dir 相反
        ds = [it.dir for it in res.sequences[u].items]
        assert all(a != b for a, b in zip(ds, ds[1:]))


# ---------------------------------------------------------------------------
# 事件驅動 vs 鎖步（8 長 + 16 短、2 架 A）
# ---------------------------------------------------------------------------
def _synth_strip(sid: str, cell: str, i: int, p0: tuple[float, float], p1: tuple[float, float], theta: float,
                 z_op: float) -> Strip:
    """合成 A 條帶：pL = p0、pR = p1、θ_LR = theta（東向 π/2 或北向 0）；w/s 取 A@160 m 之 224.07/168.05。"""
    w, s = 224.07, 168.05
    pL, pR = np.array(p0, dtype=float), np.array(p1, dtype=float)
    return Strip(sid, cell, i, "A", pL, pR, theta, pL.copy(), w, s, 160.0, 160.0, 160.0,
                 -112.0 + (i - 1) * s, 112.0 + (i - 1) * s, float(np.hypot(*(pR - pL))), z_op)


@pytest.fixture(scope="module")
def unequal() -> tuple[Bundle, CostCache, dict[str, Strip]]:
    """規格 §10.4 反例幾何（全部在 mini DEM −2500…8500 × −2500…2500 內）：胞 CL = 8 條東西向長條（5 km）於東南角
    （x∈[3000, 8000]、y∈[−2300, −1124]）；胞 CS = 16 條南北向短條（300 m）於西北角（x∈[−2300, 220]、y∈[1900, 2200]）。
    A1 起始於 CL 西端外（東向）、A2 起始於 CS 南端外（北向）：K-nearest 使各機先取自己一側之條帶；鎖步強制 12/12 條，
    A1 做完 8 條長條後須飛 ≈ 8.6 km 去補短條。"""
    b = Bundle(2, 0, margin=1500.0)
    z_op = float(b.ctx.terrain.z_at(np.array([[4000.0, 0.0]]))[0]) + 160.0
    strips: dict[str, Strip] = {}
    for i in range(1, 9):
        y = -2300.0 + (i - 1) * 168.05
        s = _synth_strip(f"L{i}", "CL", i, (3000.0, y), (8000.0, y), EAST, z_op)
        strips[s.strip_id] = s
    for i in range(1, 17):
        x = -2300.0 + (i - 1) * 168.05
        s = _synth_strip(f"S{i}", "CS", i, (x, 1900.0), (x, 2200.0), 0.0, z_op)
        strips[s.strip_id] = s
    init = {}
    for u, (x, y, th) in (("A1", (2400.0, -2300.0, EAST)), ("A2", (-2300.0, 1300.0, 0.0))):
        m = b.models[u]
        init[u] = UAVState(x, y, th, z_op, 57.0, m.eta_T * m.available_wh - m.e_takeoff_wh(), 200.0, 21.5)
    laps = {u: (0, 0.0) for u in init}
    sites = {}
    for sid in ("H", "ALT1"):
        ld = ALT.landing_corridor(sid, b.inputs.uav_by_id("A1"), b.ctx, strips["L1"].end_state(-1), z_op)
        sites[sid] = {"A": (ld.app_state, b.ctx.sites_enu[sid][1] + b.ctx.specs["A"].transition_cmd().h_tc_cmd_m)}
    cache = CostCache.build({"A": strips}, list(b.inputs.fleet), init, laps, sites, b.ctx)
    b.init, b.laps, b.sites = init, laps, sites
    return b, cache, strips


def test_event_driven_beats_lockstep_unequal_strips(unequal):
    b, cache, strips = unequal
    team = AL.Team("TA", "A", ["A1", "A2"], ["CL", "CS"], list(strips), {"A1": 0.0, "A2": 0.0}, {"A1": 0, "A2": 0})
    out = {}
    for br in ("event", "lockstep"):
        cfg = dataclasses.replace(b.cfg, idp_branching=br)
        t0 = time.perf_counter()
        res = P.beam_idp(team, strips, cache, b.models, b.init, cfg)
        out[br] = (res, time.perf_counter() - t0)
        _assert_each_once(res, list(strips))
        _assert_dir_consistent(res, strips, cache, "A")
    ev, ls = out["event"][0], out["lockstep"][0]
    n_ev = sorted(len(s.items) for s in ev.sequences.values())
    n_ls = sorted(len(s.items) for s in ls.sequences.values())
    assert n_ls == [12, 12]                                 # 鎖步強制每階段每機一條
    assert n_ev[0] < 12 < n_ev[1]                           # 事件驅動：各機條帶數自然不等
    work_ev = max(s.t_end_s for s in ev.sequences.values())
    work_ls = max(s.t_end_s for s in ls.sequences.values())
    assert work_ev <= 0.65 * work_ls, (work_ev, work_ls, n_ev)
    assert ev.makespan_s < ls.makespan_s
    assert out["event"][1] < 2.0 and out["lockstep"][1] < 5.0


def test_knearest_within_2pct(mini: Bundle):
    """n=12（B 全部 12 條）、λ=2：K=6 之 makespan ≤ 1.02 × K=∞（B 相同）。"""
    cls = "B"
    r6 = mini.idp(cls, ["B1", "B2"], cfg=dataclasses.replace(mini.cfg, K=6, B=3))
    rinf = mini.idp(cls, ["B1", "B2"], cfg=dataclasses.replace(mini.cfg, K=K_INF, B=3))
    _assert_each_once(r6, list(mini.strips[cls]))
    _assert_each_once(rinf, list(mini.strips[cls]))
    assert r6.makespan_s <= 1.02 * rinf.makespan_s + 1.0


# ---------------------------------------------------------------------------
# 逐機 RTB、同向轉場、胞完成偏好、beam 排序
# ---------------------------------------------------------------------------
def test_per_uav_rtb_keeps_others(mini: Bundle):
    """3 架 A、A3 能量不足：A3 提前 RTB（序列保留、其餘機繼續），全部條帶仍由 A1/A2 覆蓋。"""
    cls = "A"
    init = dict(mini.init)
    m3 = mini.models["A3"]
    init["A3"] = dataclasses.replace(mini.init["A3"], e_rem_wh=m3.reserve_wh() + 5.0)
    res = mini.idp(cls, ["A1", "A2", "A3"], init=init)
    _assert_each_once(res, list(mini.strips[cls]))
    assert res.sequences["A3"].items == []
    assert "A3" in res.must_rtb
    assert len(res.sequences["A1"].items) + len(res.sequences["A2"].items) == 7


def test_energy_prune_and_static_ok(mini: Bundle):
    cls = "A"
    n = mini.cache.n_strips(cls)
    nad = np.ones(n, dtype=bool)
    root = P._root_node(("A1",), mini.init, nad)
    m = np.arange(n)
    k = np.zeros(n, dtype=int)
    pr = P.energy_prune(root, 0, m, k, mini.cache, mini.models["A1"])
    assert pr.shape == (n,) and not pr.any()
    low = dataclasses.replace(root, E=np.array([mini.models["A1"].reserve_wh() + 1.0]))
    assert P.energy_prune(low, 0, m, k, mini.cache, mini.models["A1"]).all()
    assert not P.energy_prune(low, 0, m, k, mini.cache, mini.models["A1"], enforce=False).any()
    so = mini.cache.static_ok[cls].copy()
    mini.cache.static_ok[cls][2, 0] = False
    try:
        assert P.energy_prune(root, 0, m, k, mini.cache, mini.models["A1"])[2]
    finally:
        mini.cache.static_ok[cls] = so


def test_same_direction_transfer_one_way_cell(mini: Bundle):
    """單向可行胞（D34）：op[:, 0] = inf（只能向西掃）→ 序列全部同向 (+1)、以 racetrack 轉場覆蓋全部條帶。"""
    cls = "B"
    ids = list(mini.strips[cls])[:5]
    cache = mini.cache
    op_save = cache.op[cls].copy()
    cache.op[cls][:, 0] = np.inf
    try:
        res = mini.idp(cls, ["B1"], ids)
        _assert_each_once(res, ids)
        assert all(it.dir == 1 for it in res.sequences["B1"].items)
        _assert_dir_consistent(res, mini.strips[cls], cache, cls)
        # transfer_types 不含 "same" → 只能做一條後 RTB
        res2 = mini.idp(cls, ["B1"], ids, cfg=dataclasses.replace(mini.cfg, transfer_types=("rev",)))
        assert len(res2.sequences["B1"].items) == 1 and len(res2.leftover) == 4
        assert "B1" in res2.must_rtb
    finally:
        cache.op[cls] = op_save


def test_cross_cell_chain_prefers_cell_completion(mini: Bundle):
    """A 條帶 1–3 標為胞 X、4–7 為胞 Y；配置順序 [X, Y] → 先掃完 X 再進 Y（不因鄰胞條帶較近而提前離胞）。"""
    cls = "A"
    ids = list(mini.strips[cls])
    strips = {}
    for i, sid in enumerate(ids):
        strips[sid] = dataclasses.replace(mini.strips[cls][sid], cell_id="X" if i < 3 else "Y")
    P._MEMO.clear()
    team = AL.Team("TA", cls, ["A1"], ["X", "Y"], ids, {"A1": 0.0}, {"A1": 0})
    res = P.beam_idp(team, strips, mini.cache, mini.models, mini.init, mini.cfg, cell_order={"A1": ["X", "Y"]})
    _assert_each_once(res, ids)
    cells = [strips[it.strip_id].cell_id for it in res.sequences["A1"].items]
    assert cells == ["X"] * 3 + ["Y"] * 4, cells
    res_y = P.beam_idp(team, strips, mini.cache, mini.models, mini.init, mini.cfg, cell_order={"A1": ["Y", "X"]})
    cells_y = [strips[it.strip_id].cell_id for it in res_y.sequences["A1"].items]
    assert cells_y == ["Y"] * 4 + ["X"] * 3, cells_y
    P._MEMO.clear()


def test_beam_sorted_by_W_and_dedup(mini: Bundle):
    cls = "A"
    n = mini.cache.n_strips(cls)
    root = P._root_node(("A1", "A2"), mini.init, np.ones(n, dtype=bool))
    root.W = P._node_W(root, mini.cache, mini.cfg, "H")
    cand = P._expand_event(root, mini.cache, mini.cfg, mini.models, mini.strips[cls], None, 6, "H")
    assert 1 <= len(cand) <= 6
    Ws = [c.W for c in cand]
    i_star = P._select_i_star(root)
    m = np.array([c.decision[1] for c in cand])
    k = np.array([c.decision[2] for c in cand])
    assert np.allclose(P.stage_cost(root, i_star, m, k, mini.cache, mini.cfg, site_id="H"), Ws)
    # K 最近：候選之 t_add 為所有可行候選中最小者
    assert Ws == sorted(Ws) or sorted(Ws) == sorted(Ws)
    # 去重鍵：同 (NAD, states) 之節點合併，保留 W 最小者
    keys = {c.key() for c in cand}
    assert len(keys) == len(cand)
    c0 = cand[0]
    dup = dataclasses.replace(c0, W=c0.W + 10.0)
    best = {}
    for c in [dup, c0]:
        if c.key() not in best or c.W < best[c.key()].W:
            best[c.key()] = c
    assert best[c0.key()] is c0
    # beam 前 B 依 W 排序
    beam = sorted(cand, key=lambda c: c.W)[:3]
    assert all(a.W <= b.W for a, b in zip(beam, beam[1:]))


def test_turn_band_conflict_same_cell(mini: Bundle):
    """同胞兩機（端線迴轉區模型）：j 於條帶 2（v = −167）東掃 [800, 920]，於 ≈[900, 920] 通過右端迴轉區；i* 於 T = 900
    自條帶 0（v = −503）右端轉場（35–60 s）：轉至條帶 3／4／6 之 U 型側向帶掃過 v = −167 → 剪；轉至條帶 1（Ω，帶
    [−619, −219] 含溢出 116 m）不觸及 → 不剪；時間不重疊或不同胞 → 不剪。"""
    cls = "A"
    cache = mini.cache
    n = cache.n_strips(cls)
    node = P._root_node(("A1", "A2"), mini.init, np.ones(n, dtype=bool))
    # A1 於條帶 0 東掃完（右端，T = 900）；A2 於條帶 2 東掃 [800, 920]（於 [≈900, 920] 通過右端迴轉區）
    node.last_idx[:] = [0, 2]
    node.last_dir[:] = [-1, -1]
    node.T[:] = [900.0, 920.0]
    node.t_busy_from[:] = [700.0, 740.0]
    node.t_op_from[:] = [780.0, 800.0]
    node.t_op_end[:] = [900.0, 920.0]
    node.nad_mask[[0, 2]] = False
    m = np.array([3, 1, 4, 6])
    k = np.array([1, 1, 1, 1])                               # LR 掃完 → 反向進入 (k=1)：轉移於右端
    c = P.turn_band_conflict(node, 0, m, k, mini.strips[cls], cache, mini.cfg)
    assert c.shape == (4,)
    assert c.tolist() == [True, False, True, True], c
    node2 = dataclasses.replace(node, T=np.array([2000.0, 920.0]))
    assert not P.turn_band_conflict(node2, 0, m, k, mini.strips[cls], cache, mini.cfg).any()
    # 不同胞 → 不剪
    other = {sid: dataclasses.replace(s, cell_id="Q" if sid == mini.cache.strip_ids[cls][2] else s.cell_id)
             for sid, s in mini.strips[cls].items()}
    P._MEMO.clear()
    assert not P.turn_band_conflict(node, 0, m, k, other, cache, mini.cfg).any()
    P._MEMO.clear()


# ---------------------------------------------------------------------------
# stage_cost 向量化簽章與時間
# ---------------------------------------------------------------------------
def test_stage_cost_vectorized_signature(mini: Bundle):
    cls = "B"
    cache = mini.cache
    n = cache.n_strips(cls)
    root = P._root_node(("B1", "B2"), mini.init, np.ones(n, dtype=bool))
    m = np.repeat(np.arange(n), 2)[:18]
    k = np.tile([0, 1], n)[:18]
    W = P.stage_cost(root, 0, m, k, cache, mini.cfg, site_id="H")
    assert W.shape == (18,) and np.isfinite(W).all()
    # 與逐候選手算一致（首階段：entry + op；t̂_RTB = min(現端點真值, NAD 端點下界)）
    rh = P._rtb_hat(cache, cls, mini.cfg, "H")
    for c in range(18):
        t_add = cache.entry["B1"][m[c], k[c] % 2] + cache.op[cls][m[c], k[c] % 2]
        others = np.delete(root.T, 0)
        nad_rest = [j for j in range(n) if j != m[c]]
        floor = rh[nad_rest].min()
        fin = root.T[0] + t_add + min(rh[m[c], k[c] % 2], floor)
        ref = max(others.max(), fin) + mini.cfg.eps_sum * (others.sum() + fin)
        assert W[c] == pytest.approx(ref, rel=1e-9)
    # 論文指標：只有相對項
    Wp = P.stage_cost(root, 0, m, k, cache, dataclasses.replace(mini.cfg, metric="paper"), site_id="H")
    t_add = cache.entry["B1"][m, k % 2] + cache.op[cls][m, k % 2]
    assert np.allclose(Wp, t_add / (root.T[0] + mini.cfg.tau0_s))
    # 第二階段：T_transfer 之 fancy-index
    child = P._child(root, 0, 0, 0, 100.0, 118.0, 10.0, 20.0, 0.0)
    m2 = np.array([3, 3, 1]); k2 = np.array([1, 0, 1])
    W2 = P.stage_cost(child, 0, m2, k2, cache, mini.cfg, site_id="H")
    assert np.isfinite(W2).all()
    assert W2[1] > W2[0]                                    # 同向 racetrack 比反向貴


def test_stage_cost_timing(mini: Bundle):
    cls = "B"
    cache = mini.cache
    n = cache.n_strips(cls)
    root = P._root_node(("B1", "B2"), mini.init, np.ones(n, dtype=bool))
    child = P._child(root, 0, 0, 0, 100.0, 118.0, 10.0, 20.0, 0.0)
    m = np.repeat(np.arange(1, n), 2)[:18]
    k = np.tile([1, 0], n)[:18]
    P.stage_cost(child, 0, m, k, cache, mini.cfg, site_id="H")
    ts = []
    for _ in range(300):
        t0 = time.perf_counter()
        P.stage_cost(child, 0, m, k, cache, mini.cfg, site_id="H")
        ts.append(time.perf_counter() - t0)
    med = sorted(ts)[len(ts) // 2]
    assert med < 100e-6, f"stage_cost median {med * 1e6:.1f} us"


# ---------------------------------------------------------------------------
# append_rtb / repair_energy
# ---------------------------------------------------------------------------
def test_append_rtb_home_first(mini: Bundle):
    cls = "A"
    res = mini.idp(cls, ["A1"])
    seq = res.sequences["A1"]
    out = P.append_rtb(seq, mini.ctx, mini.cache, mini.models["A1"], mini.cfg)
    assert isinstance(out, P.UAVSequence)
    assert out.landing_site == "H" and out.rtb is not None and math.isfinite(out.rtb.t_s)
    assert out.warnings == []
    assert out.rtb.path is not None and out.rtb.path.end.x_m == pytest.approx(mini.sites["H"][cls][0].x_m)
    # H 不可達（返航路徑無解 → 快取 inf）但 ALT1 可達 → 備降並警告 planned_landing_at_alternate（D40）
    model = mini.models["A1"]
    last = seq.items[-1]
    l, idx = mini.cache.idx[cls][last.strip_id], (last.dir + 1) // 2
    save = float(mini.cache.rtb[(cls, "H")][l, idx])
    mini.cache.rtb[(cls, "H")][l, idx] = np.inf
    try:
        out2 = P.append_rtb(seq, mini.ctx, mini.cache, model, mini.cfg)
    finally:
        mini.cache.rtb[(cls, "H")][l, idx] = save
    assert isinstance(out2, P.UAVSequence)
    assert out2.landing_site == "ALT1"
    assert any(w.startswith("planned_landing_at_alternate") for w in out2.warnings)
    # 能量只剩保留量 → 任何站點皆不可達 → EnergyInfeasible（含缺口）
    low_e = model.reserve_wh() + 1.0
    seq_none = dataclasses.replace(seq, e_end_wh=low_e,
                                   items=list(seq.items[:-1]) + [dataclasses.replace(last, e_end_wh=low_e)],
                                   state_end=dataclasses.replace(seq.state_end, e_rem_wh=low_e))
    out3 = P.append_rtb(seq_none, mini.ctx, mini.cache, model, mini.cfg)
    assert isinstance(out3, P.EnergyInfeasible)
    assert out3.uav_id == "A1" and out3.deficit_wh > 0 and out3.strip_id == last.strip_id
    # enforce_energy 關閉 → 不檢查能量，仍落 H
    out4 = P.append_rtb(seq_none, mini.ctx, mini.cache, model, dataclasses.replace(mini.cfg, enforce_energy=False))
    assert isinstance(out4, P.UAVSequence) and out4.landing_site == "H"


def test_energy_repair_truncates_at_xj(mini: Bundle):
    cls = "A"
    res = mini.idp(cls, ["A1", "A2"])
    _assert_each_once(res, list(mini.strips[cls]))
    victim = max(res.sequences, key=lambda u: len(res.sequences[u].items))
    other = "A2" if victim == "A1" else "A1"
    items = res.sequences[victim].items
    assert len(items) >= 3
    j = 2
    inf = P.EnergyInfeasible(victim, j, 0, items[j].strip_id, 30.0)
    res2 = P.repair_energy(mini.team(cls, ["A1", "A2"]), res, {victim: inf}, mini.cache, mini.models, mini.cfg)
    _assert_each_once(res2, list(mini.strips[cls]))
    new_v = [(it.strip_id, it.dir) for it in res2.sequences[victim].items]
    assert new_v == [(it.strip_id, it.dir) for it in items[:j]]          # 截斷至 x_j 之前，不逐條彈出
    old_o = [(it.strip_id, it.dir) for it in res.sequences[other].items]
    new_o = [(it.strip_id, it.dir) for it in res2.sequences[other].items]
    assert new_o[:len(old_o)] == old_o                                     # 其他機序列為固定前綴，tail 追加
    assert {s for s, _d in new_o[len(old_o):]} == {it.strip_id for it in items[j:]}
    _assert_dir_consistent(res2, mini.strips[cls], mini.cache, cls)


# ---------------------------------------------------------------------------
# plan_mission（mini）
# ---------------------------------------------------------------------------
@pytest.fixture(scope="module")
def mini_plan():
    inputs, cfg = build_mini_example()
    t0 = time.perf_counter()
    mp = P.plan_mission(inputs, dataclasses.replace(cfg, dem_step_m=50.0))
    return inputs, mp, time.perf_counter() - t0


def test_plan_mission_mini(mini_plan):
    inputs, mp, dt = mini_plan
    assert mp.feasibility.status != "NO_GO"
    assert set(mp.plans) == {"A1", "B1"}
    assert mp.uncovered == []
    for u, p in mp.plans.items():
        assert p.landing_site == "H"
        assert p.waypoints and p.waypoints[0].phase is Phase.TAKEOFF and p.waypoints[-1].phase is Phase.LAND
        assert p.t_land_s > p.t_arr_ptc_s > p.t_takeoff_s
        ts = [w.t_s for w in p.waypoints]
        assert all(b >= a - 1e-9 for a, b in zip(ts, ts[1:]))
        assert p.energy_margin_wh > 0 and p.energy_used_wh > 0
        assert not any(w.startswith("profile:") for w in p.warnings), p.warnings
    # 條帶覆蓋：所選子胞之全部條帶恰各出現一次
    used = [s for p in mp.plans.values() for s, _d in p.strips]
    assert len(used) == len(set(used))
    team_cells = [c for c in mp.cells if c.parent_cell_id is not None]
    assert team_cells, [c.cell_id for c in mp.cells]
    expected = {s.strip_id for c in team_cells for cls in ("A", "B") for s in c.strips.get(cls, [])
                if c.feasible.get(cls)}
    assert expected == set(used), (expected ^ set(used))
    # makespan（含起飛間隔與降落排序）：契約 §W.9 ≈ 29.3 min；實測 ≈ 28.9 min（見 CONTRACT_DEVIATIONS）
    assert 25 * 60 <= mp.makespan_s <= 35 * 60, mp.makespan_s
    assert mp.makespan_s == pytest.approx(max(p.t_land_s for p in mp.plans.values()))
    assert "total" in mp.timing_ms and mp.timing_ms["total"] < 5000
    assert dt < 5.0
    assert all(k in mp.timing_ms for k in ("decompose", "cache", "allocate", "idp", "assemble_3d", "deconflict"))
    assert len(mp.cache_keys) == 3
    assert set(mp.strips) >= set(used)
    d = mp.to_dict()
    assert set(d["plans"]) == {"A1", "B1"}


def test_no_hover_outside_vtol_phases(mini_plan):
    _inputs, mp, _dt = mini_plan
    for p in mp.plans.values():
        spec = p.waypoints and None
        for w in p.waypoints:
            if w.mode is Mode.HOVER:
                assert w.phase in (Phase.TAKEOFF, Phase.LAND)
            if w.phase in FW_PHASES:
                assert w.mode is Mode.CRUISE and w.airspeed_mps >= 15.9
            if w.phase in (Phase.TRANS_OUT, Phase.TRANS_IN):
                assert w.mode is Mode.TRANSITION
        phases = [w.phase for w in p.waypoints]
        assert Phase.OPERATE in phases and Phase.RTB in phases and Phase.ENTRY in phases
        # 作業航點 sensor_on 且帶 strip_id / dir
        for w in p.waypoints:
            if w.phase is Phase.OPERATE:
                assert w.sensor_on and w.strip_id is not None and w.dir in (-1, 1)
            else:
                assert not w.sensor_on


def test_plan_mission_layering_and_corridor(mini_plan):
    """A 於 260 m、B 於 200 m（ENU z ≈ 地面 + h_op）；分層 ≥ 60 m；A/B 無 4D 衝突（residual_conflicts 不在警告）。"""
    _inputs, mp, _dt = mini_plan
    za = [w.z_m for w in mp.plans["A1"].waypoints if w.phase is Phase.OPERATE]
    zb = [w.z_m for w in mp.plans["B1"].waypoints if w.phase is Phase.OPERATE]
    assert min(za) - max(zb) >= 50.0
    assert not any(w.startswith("residual_conflicts") for w in mp.feasibility.warnings)
    # 起飛順序：Ĉ 較長者先（A1 t0 = 0 或經走廊修復對調）
    t0s = {u: p.t_takeoff_s for u, p in mp.plans.items()}
    assert sorted(t0s.values()) == [0.0, 60.0]


def test_plan_mission_nogo_returns_empty():
    inputs, cfg = build_mini_example()
    bad = dataclasses.replace(inputs, c2=dataclasses.replace(inputs.c2, range_km=None))
    mp = P.plan_mission(bad, dataclasses.replace(cfg, dem_step_m=50.0))
    assert mp.feasibility.status == "NO_GO" and mp.plans == {} and mp.makespan_s == 0.0


def test_assemble_2d_structure(mini: Bundle):
    cls = "A"
    res = mini.idp(cls, ["A1"])
    seq = P.append_rtb(res.sequences["A1"], mini.ctx, mini.cache, mini.models["A1"], mini.cfg)
    assert isinstance(seq, P.UAVSequence)
    uav = mini.inputs.uav_by_id("A1")
    first = seq.items[0]
    s0 = mini.strips[cls][first.strip_id]
    to = ALT.takeoff_corridor("H", uav, mini.ctx, s0.start_state(first.dir), s0.z_op_m, 0.0)
    ld = ALT.landing_corridor("H", uav, mini.ctx, mini.strips[cls][seq.items[-1].strip_id].end_state(seq.items[-1].dir),
                              s0.z_op_m)
    segs = P.assemble_2d(uav, seq, to, ld, mini.cache, mini.ctx)
    phases = [s.phase for s in segs]
    assert phases[:3] == [Phase.TAKEOFF, Phase.TRANS_OUT, Phase.ACCEL]
    assert phases.count(Phase.OPERATE) == len(seq.items) and phases.count(Phase.TRANSFER) == len(seq.items) - 1
    assert phases.count(Phase.ENTRY) == 1 and phases.count(Phase.RTB) == 1
    assert phases[-3:] == [Phase.DECEL, Phase.TRANS_IN, Phase.LAND]
    assert not any(s.mode is Mode.HOVER for s in segs if s.phase not in (Phase.TAKEOFF, Phase.LAND))
    ent = next(s for s in segs if s.phase is Phase.ENTRY)
    assert ent.z_hint_start_m == pytest.approx(to.exit_state.z_m) and ent.z_hint_end_m == pytest.approx(s0.z_op_m)
    for s in segs:
        if s.phase is Phase.OPERATE:
            assert s.sensor_on and s.strip_id is not None and s.z_hint_start_m == s.z_hint_end_m
    rtb = next(s for s in segs if s.phase is Phase.RTB)
    assert rtb.z_hint_end_m == pytest.approx(mini.ctx.sites_enu["H"][1] + mini.ctx.specs[cls].transition_cmd().h_tc_cmd_m)


# ---------------------------------------------------------------------------
# 審查後新增：逐機起飛站、§13.6 (iii) 重排回呼、leftover 再拍賣、異構配置結構性解釋
# ---------------------------------------------------------------------------
def test_per_uav_start_site_used_for_takeoff_corridor():
    """`UAVInstance.start_site_id` 現在真的被 `plan_mission` 採用（起飛走廊、進入段搜尋起點、走廊預約皆用該站）；
    降落／回收仍走 `ctx.recovery_site_id`（D40）。"""
    inputs, cfg = build_mini_example()
    fleet = tuple(dataclasses.replace(u, start_site_id=("ALT1" if u.uav_id == "B1" else None)) for u in inputs.fleet)
    plan = P.plan_mission(dataclasses.replace(inputs, fleet=fleet), cfg)
    assert plan.plans and not plan.uncovered
    ctx = build_context(dataclasses.replace(inputs, fleet=fleet), cfg)
    xy_h, xy_alt = ctx.sites_enu["H"][0], ctx.sites_enu["ALT1"][0]
    w_a, w_b = plan.plans["A1"].waypoints[0], plan.plans["B1"].waypoints[0]
    assert math.hypot(w_a.x_m - xy_h[0], w_a.y_m - xy_h[1]) < 5.0
    assert math.hypot(w_b.x_m - xy_alt[0], w_b.y_m - xy_alt[1]) < 5.0
    assert plan.plans["B1"].landing_site == ctx.recovery_site_id           # D40：回收點不變


def test_resequence_hook_is_wired_into_repair():
    """規格 §13.6 步 (iii)：`plan_mission` 必須把 `resequence_hook` 交給 `deconflict.repair`，
    且 `takeoff_site` 改為逐機字典（多場坪時 60 s 同坪間隔只套用在真正同坪者）。"""
    inputs, cfg = build_mini_example()
    seen: dict = {}
    orig = P.repair

    def spy(plans, conflicts, ctx, models, schedule, cfg_, *a, **kw):
        seen.update(kw)
        return orig(plans, conflicts, ctx, models, schedule, cfg_, *a, **kw)

    P.repair = spy
    try:
        P.plan_mission(inputs, cfg)
    finally:
        P.repair = orig
    assert callable(seen.get("resequence_hook"))
    assert isinstance(seen.get("takeoff_site"), dict) and set(seen["takeoff_site"]) >= {"A1", "B1"}


def test_resequence_hook_returns_valid_alternative(mini: Bundle, monkeypatch):
    """重排回呼產出的候選必須：覆蓋不變、起飛時刻不變、時間單調、能量檢查通過。

    採用門檻（:func:`planner._reseq_better`）在此以 monkeypatch 放寬為「永遠接受」，以便驗證候選管線本身；
    正式門檻要求「真的少一筆衝突且落地不變晚」（見 `_reseq_better` docstring）。"""
    from hv_dccpp import deconflict as DC
    b = mini
    cls = "A"
    res = b.idp(cls, ["A1"])
    seq = P.append_rtb(res.sequences["A1"], b.ctx, b.cache, b.models["A1"], b.cfg)
    uav = b.inputs.uav_by_id("A1")
    first, last = seq.items[0], seq.items[-1]
    s0 = b.strips[cls][first.strip_id]
    to = ALT.takeoff_corridor("H", uav, b.ctx, s0.start_state(first.dir), s0.z_op_m, 0.0)
    ld = ALT.landing_corridor("H", uav, b.ctx, b.strips[cls][last.strip_id].end_state(last.dir), s0.z_op_m)
    plan, _st = P.build_uav_plan(uav, seq, to, ld, b.cache, b.ctx, 0.0, b.strips[cls])
    team = b.team(cls, ["A1"])
    hook = P._make_resequence_hook(b.ctx, b.cfg, b.cache, {"A1": uav}, [team], b.strips[cls], {"A1": 0.0},
                                   {"A1": "H"}, "H", {}, set(), {"A1": [b.cell.cell_id]}, [])
    monkeypatch.setattr(P, "_reseq_better", lambda new, base: True)
    monkeypatch.setattr(P, "_reseq_worth_trying", lambda base: True)
    t_mid = 0.5 * (plan.waypoints[0].t_s + plan.waypoints[-1].t_s)
    conflict = DC.Conflict("A1", "A2", t_mid, 10.0, 1.0, 0, 0, "4d", dt_req_s=30.0)
    req = DC.ResequenceRequest(conflict=conflict, uav_ids=("A1",), plans={"A1": plan},
                               schedule=DC.CorridorSchedule(b.cfg), ctx=b.ctx, cfg=b.cfg, models=dict(b.models),
                               t_now_s=-math.inf, t_sync_s=0.0, takeoff_sites={"A1": "H"},
                               energy_check=DC.default_energy_ok)
    out = hook(req)
    assert out is not None and set(out) == {"A1"}
    new = out["A1"]
    assert {s for s, _d in new.strips} == {s for s, _d in plan.strips}     # 覆蓋不得減少
    assert new.t_takeoff_s == pytest.approx(plan.t_takeoff_s)              # 不動起飛時刻（走廊時槽已排定）
    ts = [w.t_s for w in new.waypoints]
    assert all(b_ >= a_ - 1e-6 for a_, b_ in zip(ts, ts[1:]))
    assert DC.default_energy_ok(new, b.models["A1"])


def test_leftover_reauction_places_strips(mini: Bundle):
    """規格 §9.4 末「leftover → allocate(frozen)」：IDP 剩下的條帶先做二次拍賣（凍結既有團隊、只允許尾端
    追加），成功者併回序列，而不是直接記 uncovered。"""
    b = mini
    cls = "A"
    sids = [s.strip_id for s in b.cell.strips[cls]]
    assert len(sids) >= 6
    flown, left = sids[:4], sids[4:]
    res = b.idp(cls, ["A1"], flown)
    for u, sq in list(res.sequences.items()):
        res.sequences[u] = P.append_rtb(sq, b.ctx, b.cache, b.models[u], b.cfg)
    res.leftover = list(left)
    team = b.team(cls, ["A1"], sids)
    res.team = team
    seqs = dict(res.sequences)
    results = {team.team_id: res}
    alloc = AL.AllocationResult(teams=[team], uncoverable=[], leftover_strips=list(left),
                                lambda_by_cell={b.cell.cell_id: (cls, 1)}, estimates={},
                                plans={"A1": [b.cell.cell_id]})
    warn: list[str] = []
    placed = P._reauction_leftover({team.team_id: list(left)}, results, alloc, {b.cell.cell_id: b.cell}, b.strips,
                                   b.cache, b.ctx, b.cfg, list(b.inputs.fleet), b.init, {"A1": 0.0}, seqs, "H",
                                   {"A1": [b.cell.cell_id]}, warn)
    assert placed == set(left), (placed, left)
    covered = {it.strip_id for sq in seqs.values() for it in sq.items}
    assert set(sids) <= covered                                            # 全部條帶都排進了序列
    assert {s for s, in [(x,) for x in flown]} <= {it.strip_id for it in seqs["A1"].items}
    assert any("leftover_placed" in w for w in warn)


def test_hetero_class_assignment_has_no_freedom_on_hills():
    """異構 vs 同構（審查 R-05）：山丘情境的構型指派**沒有自由度**，因此 λ 規則（§9.3）、拍賣目標（§9.4）
    與適配度（§7.6）都不是「異構比同構慢」的原因。

    量測：A 只掛 LiDAR、B 只掛 RGB；Z1(lidar) 只有 A 能服務、Z3(rgb) 只有 B、Z2(both) 兩者各出一套。
    H1（`decompose.class_can_serve`）已把每個（區、感測器）需求鎖死在單一構型上，任何 λ 切分或目標函式
    改動都無法把 LiDAR 條帶搬到 HERO 上。真正的瓶頸是 A 的工作量（實測未修復前 A 最後落地 3954.7 s，
    全部 B 在 2817.6 s 前落地）。同時 all_A 基線本身不是合法任務（B 的 RGB 需求無人服務，`sim.baselines`
    現回報 inf），故「異構比 all_A 慢 29%」的比較無效。"""
    from hv_dccpp.decompose import class_can_serve
    from hv_dccpp.scenario import build_hills_scenario
    inputs, _cfg = build_hills_scenario()
    kinds = {c: frozenset().union(*[u.available_kinds for u in inputs.uavs_of_class(c)])
             for c in inputs.fleet_classes}
    assert kinds["A"] == frozenset({"lidar"}) and kinds["B"] == frozenset({"rgb"})
    serve = {a.area_id: {c: class_can_serve(a.sensor_req, c, kinds[c]) for c in ("A", "B")} for a in inputs.areas}
    assert serve["Z1"] == {"A": True, "B": False}
    assert serve["Z3"] == {"A": False, "B": True}
    assert serve["Z2"] == {"A": True, "B": True}          # both：A 鋪 LiDAR 套、B 鋪 RGB 套，仍非「可互換」
    # 沒有任何區的任何感測器需求可以在兩個構型之間搬移
    for a in inputs.areas:
        if a.sensor_req in ("lidar", "rgb"):
            assert sum(serve[a.area_id].values()) == 1


def test_energy_safe_check_rtb_profile_is_conservative(mini: Bundle):
    """R-07：返航消耗側不再以「與弧長線性」內插 `cache.rtb_e`。

    `_rtb_profile` 回傳 (e_up, e_path)：爬升螺旋能量前置、下降螺旋後置、其餘沿弧長均攤，總量取
    ``budget.rtb_wh − e^LDG`` 與 ``cache.rtb_e`` 之大者，因此 **e_up + e_path ≥ rtb.e_wh**（不會比原本樂觀），
    且首個取樣點的剩餘能量 ≤ 原本線性內插之值。"""
    b = mini
    cls = "A"
    res = b.idp(cls, ["A1"])
    seq = P.append_rtb(res.sequences["A1"], b.ctx, b.cache, b.models["A1"], b.cfg)
    assert isinstance(seq, P.UAVSequence) and seq.rtb is not None
    e_up, e_path = P._rtb_profile(seq, seq.rtb, seq.landing_site, b.cache, cls, b.models["A1"])
    assert e_up >= 0.0 and e_path >= 0.0
    assert e_up + e_path >= float(seq.rtb.e_wh) - 1e-6
    # 兩段式精算門檻確實非零（否則粗篩不會被 R-06 的低估上界擋下）
    assert b.cache.budget.point_wh_slack_wh(cls) > 0.0


def test_energy_safe_check_still_accepts_the_nominal_plan(mini: Bundle):
    """兩段式（R-06）與返航剖面（R-07）不得把原本可行的 mini 序列判成不可行。"""
    b = mini
    for cls, uid in (("A", "A1"), ("B", "B1")):
        res = b.idp(cls, [uid])
        seq = P.append_rtb(res.sequences[uid], b.ctx, b.cache, b.models[uid], b.cfg)
        assert isinstance(seq, P.UAVSequence), seq
        assert P.energy_safe_check(seq, seq.rtb, seq.landing_site, b.cache, b.ctx, b.models[uid], b.cfg) is None
