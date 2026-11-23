"""tests/test_costs.py — 契約 §8 `costs.py` 不變量與規格 §17.2 清單（cost_matrix_shape_four、index_convention_idx、
T_transfer_uses_idx_m、wind_refresh_keeps_geometry、budget_table_o1、static_filter_h4、entry_includes_orbit_laps、
rtb_index_end_side、per_class_indices_add_remove、budget_table_terrain_detour、transfer keep-out AVOID、§8.4 螺旋）。

情境：`scenario.build_mini_example()`（平地、西風 5 m/s、A1/B1）以 `dem_step_m=50`、`terrain_margin_m=1500` 建構；
條帶依契約 §W.3（A 7 條 y = 503 … −505.3、B 殘帶 3 條 y = −337.8 … −543.4）。
"""
from __future__ import annotations

import dataclasses
import math
import time

import numpy as np
import pytest
import shapely
from shapely.geometry import Point

from hv_dccpp import altitude as ALT
from hv_dccpp import costs as C
from hv_dccpp.context import build_context
from hv_dccpp.dubins import WORDS, Pose, dubins_path, omega_length, uturn_length
from hv_dccpp.geometry import Strip
from hv_dccpp.mission import PlannerConfig, UAVState, WindLayer
from hv_dccpp.platform import PLATFORM_HERO, PLATFORM_PIONEER, EnergyModel
from hv_dccpp.routing import AVOID_WORD, KeepoutSet
from hv_dccpp.scenario import build_mini_example
from hv_dccpp.wind import WindField

A = PLATFORM_PIONEER
B = PLATFORM_HERO
EAST = math.pi / 2
YS_A = [503.0, 334.9, 166.9, -1.2, -169.2, -337.3, -505.3]
YS_B = [-337.8, -440.6, -543.4]


# ---------------------------------------------------------------------------
# fixtures
# ---------------------------------------------------------------------------
@pytest.fixture(scope="module")
def mini():
    inputs, cfg = build_mini_example()
    ctx = build_context(inputs, dataclasses.replace(cfg, dem_step_m=50.0), terrain_margin_m=1500.0)
    return inputs, ctx.cfg, ctx


def _z_op(ctx, h: float) -> float:
    return float(ctx.terrain.z_at(np.array([[5500.0, 0.0]]))[0]) + h


def _strip_a(i: int, y: float, z_op: float, sid: str | None = None) -> Strip:
    return Strip(sid or f"A{i}", "Z1", i, "A", np.array([3932.0, y]), np.array([7068.0, y]), EAST,
                 np.array([3985.0, 615.0]), 224.07, 168.05, 160.0, 160.0, 160.0, -112.0 + (i - 1) * 168.05,
                 112.0 + (i - 1) * 168.05, 3136.0, z_op)


def _strip_b(i: int, y: float, z_op: float) -> Strip:
    return Strip(f"B{i}", "Z1", i, "B", np.array([3937.0, y]), np.array([7063.0, y]), EAST,
                 np.array([3985.0, 615.0]), 146.88, 102.81, 100.0, 100.0, 100.0, 0.0 + (i - 1) * 102.81,
                 146.88 + (i - 1) * 102.81, 3126.0, z_op)


def _strips(ctx):
    zA, zB = _z_op(ctx, 160.0), _z_op(ctx, 100.0)
    return {"A": {f"A{i+1}": _strip_a(i + 1, y, zA) for i, y in enumerate(YS_A)},
            "B": {f"B{i+1}": _strip_b(i + 1, y, zB) for i, y in enumerate(YS_B)}}


def _corridors(ctx, strips):
    a1, b1 = ctx.uav("A1"), ctx.uav("B1")
    zA, zB = strips["A"]["A1"].z_op_m, strips["B"]["B1"].z_op_m
    to_A = ALT.takeoff_corridor("H", a1, ctx, strips["A"]["A1"].start_state(-1), zA, 0.0)
    to_B = ALT.takeoff_corridor("H", b1, ctx, strips["B"]["B1"].start_state(-1), zB, 0.0)
    ld_A = ALT.landing_corridor("H", a1, ctx, strips["A"]["A3"].end_state(-1), zA)
    ld_B = ALT.landing_corridor("H", b1, ctx, strips["B"]["B2"].end_state(-1), zB)
    z_site = ctx.sites_enu["H"][1]
    sites = {"H": {"A": (ld_A.app_state, z_site + A.transition_cmd().h_tc_cmd_m),
                   "B": (ld_B.app_state, z_site + B.transition_cmd().h_tc_cmd_m)}}
    init = {"A1": to_A.exit_state, "B1": to_B.exit_state}
    laps = {"A1": (to_A.n_laps, 0.0), "B1": (to_B.n_laps, 0.0)}
    return init, laps, sites


@pytest.fixture(scope="module")
def built(mini):
    inputs, cfg, ctx = mini
    strips = _strips(ctx)
    init, laps, sites = _corridors(ctx, strips)
    cache = C.CostCache.build(strips, list(inputs.fleet), init, laps, sites, ctx)
    return ctx, strips, init, laps, sites, cache


def _wf(ctx, speed: float, dir_from: float, version: int = 1) -> WindField:
    return WindField([WindLayer(0.0, 3000.0, speed, dir_from, speed + 2.0, "agl")], ctx.frame, ctx.z_ground_home_m,
                     version=version)


# ---------------------------------------------------------------------------
# 形狀、索引約定（cost_matrix_shape_four、index_convention_idx、T_transfer_uses_idx_m）
# ---------------------------------------------------------------------------
def test_cost_matrix_shape_four_and_diag_inf(built):
    ctx, strips, init, laps, sites, cache = built
    for cls, n in (("A", 7), ("B", 3)):
        assert cache.tr[cls].shape == (n, n, 4) and cache.tr_e[cls].shape == (n, n, 4)
        assert cache.tr_len[cls].shape == (n, n, 4) and cache.tr_word[cls].shape == (n, n, 4)
        assert cache.op[cls].shape == (n, 2) and cache.op_e[cls].shape == (n, 2)
        assert cache.static_ok[cls].shape == (n, 2) and cache.static_ok[cls].dtype == bool
        d = np.arange(n)
        assert np.isinf(cache.tr[cls][d, d, :]).all()
        assert np.isinf(cache.tr_e[cls][d, d, :]).all()
        off = ~np.eye(n, dtype=bool)
        assert np.isfinite(cache.tr[cls][off]).all()
        assert cache.strip_ids[cls] == list(strips[cls]) and all(cache.idx[cls][s] == i for i, s in enumerate(strips[cls]))
    assert cache.tr_word["A"].dtype == np.int8
    assert cache.entry["A1"].shape == (7, 2) and cache.entry["B1"].shape == (3, 2)
    assert cache.rtb[("A", "H")].shape == (7, 2) and cache.rtb[("B", "H")].shape == (3, 2)


def test_index_convention_idx(built):
    """tr[l,m,k]：起始 = strips[l].end_state(dir_l)、終端 = strips[m].start_state(dir_m)；k=0 終端方位 θ_LR、k=1 θ_LR+π。"""
    ctx, strips, init, laps, sites, cache = built
    S = list(strips["A"].values())
    l, m = 0, 3
    model = ctx.models_rep["A"]
    for k in range(4):
        d_l, d_m = (-1, 1)[k // 2], (-1, 1)[k % 2]
        pc = C.transfer_cost(S[l], d_l, S[m], d_m, A, ctx.wind, model, ctx.keepouts, ctx.cfg, r_eff_m=cache.R_eff["A"])
        assert pc.path is not None
        st, en = pc.path.start, pc.path.end
        exp_s, exp_e = S[l].end_state(d_l), S[m].start_state(d_m)
        assert (st.x_m, st.y_m, st.theta_rad) == (exp_s.x_m, exp_s.y_m, exp_s.theta_rad)
        assert (en.x_m, en.y_m, en.theta_rad) == (exp_e.x_m, exp_e.y_m, exp_e.theta_rad)
        if k % 2 == 0:
            assert en.theta_rad == pytest.approx(S[m].theta_LR_rad)
            assert np.allclose([en.x_m, en.y_m], S[m].pL)
        else:
            assert en.theta_rad == pytest.approx((S[m].theta_LR_rad + math.pi) % (2 * math.pi))
            assert np.allclose([en.x_m, en.y_m], S[m].pR)
        assert cache.tr["A"][l, m, k] == pytest.approx(pc.t_s, abs=1e-6)
        assert cache.tr_len["A"][l, m, k] == pytest.approx(pc.length_m, abs=1e-6)
        assert cache.tr_e["A"][l, m, k] == pytest.approx(pc.e_wh, abs=1e-9)
    # 同向 racetrack（k=0/3）比反向長（≈ 2πR + L）
    assert cache.tr_len["A"][0, 1, 0] > cache.tr_len["A"][0, 1, 1]
    assert cache.tr_len["A"][0, 1, 0] > 2 * math.pi * 200.0 + 3000.0


def test_T_transfer_uses_idx_m(built):
    ctx, strips, init, laps, sites, cache = built
    for cls in ("A", "B"):
        T = cache.T_transfer(cls)
        n = len(strips[cls])
        for k in range(4):
            fin = np.isfinite(cache.tr[cls][:, :, k])
            diff = T[:, :, k][fin] - cache.tr[cls][:, :, k][fin]
            assert np.allclose(diff, np.broadcast_to(cache.op[cls][None, :, k % 2], (n, n))[fin])
            assert np.isinf(T[:, :, k][~fin]).all()
    r0 = cache.T_row0("A1")
    assert np.allclose(r0, cache.entry["A1"] + cache.op["A"])


# ---------------------------------------------------------------------------
# §W.3 / §W.5 / §W.6 / §W.7 手算數值
# ---------------------------------------------------------------------------
def test_w5_transfer_numbers(built):
    ctx, strips, init, laps, sites, cache = built
    trA, lenA = cache.tr["A"], cache.tr_len["A"]
    # skip-3 U 型：1E → 4W，k = 2·0 + 1 = 1：732.5 m、35.5 s
    assert lenA[0, 3, 1] == pytest.approx(uturn_length(3 * 168.05, 200.0), abs=0.5)
    assert lenA[0, 3, 1] == pytest.approx(732.5, abs=1.0)
    assert trA[0, 3, 1] == pytest.approx(35.5, abs=0.5)
    assert trA[0, 3, 1] > lenA[0, 3, 1] / 21.5                       # D14：L/Va 非保守
    # 相鄰 Ω（1 → 2）：omega_length(168.05, 200) = 1253.3 m、61.1 s；Ω(2s)（4 → 2）950.2 m／46.3 s
    assert lenA[0, 1, 1] == pytest.approx(omega_length(168.05, 200.0), abs=0.5)
    assert lenA[0, 1, 1] == pytest.approx(1253.3, abs=1.0) and trA[0, 1, 1] == pytest.approx(61.1, abs=0.5)
    assert lenA[3, 1, 1] == pytest.approx(950.2, abs=1.0) and trA[3, 1, 1] == pytest.approx(46.3, abs=0.5)
    assert cache.T_transfer("A")[0, 3, 1] == pytest.approx(225.6, abs=0.5)
    # B 殘帶：1→3 Ω(2s) 559.8 m／31.3 s、3→2 Ω 749.1 m／41.9 s
    assert cache.tr_len["B"][0, 2, 1] == pytest.approx(559.8, abs=1.0) and cache.tr["B"][0, 2, 1] == pytest.approx(31.3, abs=0.5)
    assert cache.tr_len["B"][2, 1, 1] == pytest.approx(749.1, abs=1.0) and cache.tr["B"][2, 1, 1] == pytest.approx(41.9, abs=0.5)


def test_w3_op_numbers_and_energy(built):
    ctx, strips, init, laps, sites, cache = built
    assert cache.op["A"][:, 0] == pytest.approx(118.3, abs=0.1)     # 東（順風 26.5）
    assert cache.op["A"][:, 1] == pytest.approx(190.1, abs=0.1)     # 西（逆風 16.5）
    assert cache.op["B"][:, 0] == pytest.approx(130.3, abs=0.1)
    assert cache.op["B"][:, 1] == pytest.approx(223.3, abs=0.1)
    m = ctx.models_rep["A"]
    assert cache.op_e["A"][0, 0] == pytest.approx(m.p_cruise_w() * 118.34 * m.k_unc / 3600.0, rel=1e-3)
    pc = C.op_cost(strips["A"]["A1"], -1, A, ctx.sensors[("A", "default")], ctx.wind, m, ctx.cfg)
    assert pc.t_s == pytest.approx(cache.op["A"][0, 0]) and pc.path is None and pc.length_m == 3136.0
    assert np.all(cache.op_va["A"] == 21.5)                            # V_cap 66.9 > V_g → V_a* = V_a,c


def test_w6_w7_entry_and_rtb_numbers(built):
    ctx, strips, init, laps, sites, cache = built
    assert cache.entry["A1"][0, 0] == pytest.approx(201.8, abs=0.5) and cache.entry_len["A1"][0, 0] == pytest.approx(5168, abs=3)
    assert cache.entry["A1"][0, 1] == pytest.approx(350.8, abs=0.5) and cache.entry_len["A1"][0, 1] == pytest.approx(8937, abs=3)
    assert cache.entry["B1"][0, 0] == pytest.approx(205.4, abs=0.5) and cache.entry_len["B1"][0, 0] == pytest.approx(4804, abs=3)
    assert cache.rtb[("A", "H")][2, 0] == pytest.approx(422.3, abs=0.5) and cache.rtb_len[("A", "H")][2, 0] == pytest.approx(7094, abs=3)
    assert cache.rtb[("B", "H")][1, 0] == pytest.approx(490.7, abs=0.5) and cache.rtb_len[("B", "H")][1, 0] == pytest.approx(6954, abs=3)
    # 進入路徑之能量含 Δz 爬升功率（S_acc z=100 → z_op ≈ 157.6）
    m = ctx.models["A1"]
    t = cache.entry["A1"][0, 0]
    assert cache.entry_e["A1"][0, 0] > m.segment_wh("CRUISE", t)
    assert cache.entry_e["A1"][0, 0] == pytest.approx(m.transfer_wh(t, strips["A"]["A1"].z_op_m - init["A1"].z_m), rel=1e-6)


# ---------------------------------------------------------------------------
# rtb 端點側（rtb_index_end_side）
# ---------------------------------------------------------------------------
def test_rtb_index_end_side(built):
    ctx, strips, init, laps, sites, cache = built
    s = strips["A"]["A3"]
    s_app, z_tc = C.resolve_site_entry(sites, "H", "A")
    m = ctx.models_rep["A"]
    for idx, d in enumerate((-1, 1)):
        pc = C.rtb_cost(s, d, s_app, z_tc, A, ctx.wind, m, ctx.keepouts, ctx.cfg, r_eff_m=200.0)
        st = pc.path.start
        p_exp = s.pR if idx == 0 else s.pL
        th_exp = s.theta_LR_rad if idx == 0 else s.theta_RL_rad
        assert np.allclose([st.x_m, st.y_m], p_exp) and st.theta_rad == pytest.approx(th_exp)
        en = pc.path.end
        assert (en.x_m, en.y_m, en.theta_rad) == (s_app.x_m, s_app.y_m, s_app.theta_rad)
        assert cache.rtb[("A", "H")][2, idx] == pytest.approx(pc.t_s, abs=1e-6)
        assert cache.rtb_e[("A", "H")][2, idx] == pytest.approx(pc.e_wh, abs=1e-9)
        assert pc.dz_m == pytest.approx(z_tc - s.z_op_m)
    # 右端（朝東掃完）返航較長：W.4「偶數 k 自左端進入以 W 結束於左端，RTB 只有 201–207 s」
    assert cache.rtb[("A", "H")][2, 0] > cache.rtb[("A", "H")][2, 1]
    assert cache.rtb[("A", "H")][2, 1] == pytest.approx(201.8, abs=6.0)
    # 起點在 p^R：rtb_cost 之 dir_last 非法值
    with pytest.raises(ValueError):
        C.rtb_cost(s, 0, s_app, z_tc, A, ctx.wind, m, ctx.keepouts, ctx.cfg)


# ---------------------------------------------------------------------------
# entry：盤旋圈數（entry_includes_orbit_laps）、refresh_entry
# ---------------------------------------------------------------------------
def test_entry_includes_orbit_laps(built):
    ctx, strips, init, laps, sites, cache = built
    m = ctx.models["A1"]
    s = strips["A"]["A1"]
    base = C.entry_cost(init["A1"], s, -1, A, ctx.wind, m, ctx.keepouts, ctx.cfg)
    with_laps = C.entry_cost(init["A1"], s, -1, A, ctx.wind, m, ctx.keepouts, ctx.cfg, n_laps=2, t_lap_s=60.9)
    assert base.n_laps == 0
    assert with_laps.n_laps == 2 and with_laps.t_s == pytest.approx(base.t_s + 2 * 60.9)
    gc = A.gamma_c_auw(m.auw_kg)
    assert with_laps.e_wh - base.e_wh == pytest.approx(m.p_climb_w(gc) * 2 * 60.9 * m.k_unc / 3600.0, rel=1e-9)
    # refresh_entry：同狀態 + 2 圈 → entry 列全部 +2·T_lap；幾何（長度）不變；state_key 改變
    e0 = cache.entry["A1"].copy()
    l0 = cache.entry_len["A1"].copy()
    k0 = cache.state_key
    cache.refresh_entry("A1", init["A1"], 2, 60.9, ctx)
    assert np.allclose(cache.entry["A1"], e0 + 2 * 60.9)
    assert np.array_equal(cache.entry_len["A1"], l0) and np.all(cache.entry_laps["A1"] == 2)
    assert cache.state_key != k0
    cache.refresh_entry("A1", init["A1"], 0, 0.0, ctx)
    assert np.allclose(cache.entry["A1"], e0)
    assert cache.state_key == k0
    # 新狀態（更近條帶）→ 進入時間變短
    near = UAVState(3000.0, 503.0, EAST, init["A1"].z_m, 0.0, init["A1"].e_rem_wh, 200.0, 21.5)
    cache.refresh_entry("A1", near, 0, 0.0, ctx)
    assert cache.entry["A1"][0, 0] < e0[0, 0]
    cache.refresh_entry("A1", init["A1"], 0, 0.0, ctx)
    assert np.allclose(cache.entry["A1"], e0)


# ---------------------------------------------------------------------------
# 風更新（wind_refresh_keeps_geometry）
# ---------------------------------------------------------------------------
def test_wind_refresh_keeps_geometry(mini):
    inputs, cfg, ctx = mini
    strips = _strips(ctx)
    init, laps, sites = _corridors(ctx, strips)
    cache = C.CostCache.build(strips, list(inputs.fleet), init, laps, sites, ctx)
    len0 = {c: cache.tr_len[c].copy() for c in ("A", "B")}
    word0 = {c: cache.tr_word[c].copy() for c in ("A", "B")}
    t0 = {c: cache.tr[c].copy() for c in ("A", "B")}
    op0 = cache.op["A"].copy()
    rtb0 = cache.rtb[("A", "H")].copy()
    gk0, wk0 = cache.geometry_key, cache.wind_key
    # 同風速、來向改為北風 → R_eff 不變 → 幾何逐位元相同、時間層改變
    cache.refresh_wind(_wf(ctx, 5.0, 0.0, version=1), ctx)
    for c in ("A", "B"):
        assert np.array_equal(cache.tr_len[c], len0[c]) and np.array_equal(cache.tr_word[c], word0[c])
        fin = np.isfinite(t0[c])
        assert not np.allclose(cache.tr[c][fin], t0[c][fin])
    assert cache.R_eff == {"A": 200.0, "B": 120.0} and cache.W_R == {"A": 5.0, "B": 5.0}
    assert cache.wind_key == 1 and wk0 == 0 and cache.geometry_key == gk0
    # 北風：東西向作業兩向皆側風 → op 兩向相等
    assert cache.op["A"][0, 0] == pytest.approx(cache.op["A"][0, 1]) and cache.op["A"][0, 0] != pytest.approx(op0[0, 0])
    assert not np.allclose(cache.rtb[("A", "H")], rtb0)
    # 回到原風場 → 與初建完全一致
    cache.refresh_wind(ctx.wind, ctx)
    for c in ("A", "B"):
        assert np.allclose(cache.tr[c], t0[c]) and np.array_equal(cache.tr_len[c], len0[c])
    assert np.allclose(cache.op["A"], op0) and np.allclose(cache.rtb[("A", "H")], rtb0)
    # W = 9 m/s：R_eff,A = 130·(1+9/21.5)² = 260 > 200·1.05 → 幾何層重算（長度改變）
    cache.refresh_wind(_wf(ctx, 9.0, 270.0, version=2), ctx)
    assert cache.R_eff["A"] == pytest.approx(A.r_eff(9.0)) and cache.R_eff["A"] > 200.0 * 1.05
    assert not np.allclose(cache.tr_len["A"][np.isfinite(len0["A"])], len0["A"][np.isfinite(len0["A"])])
    R9 = A.r_eff(9.0)                                 # 3s = 504 < 2R = 520 → Ω 型成為最短（只斷言長度不斷言 word）
    assert cache.tr_len["A"][0, 3, 1] == pytest.approx(min(omega_length(3 * 168.05, R9), uturn_length(3 * 168.05, R9)), abs=0.5)
    assert cache.geometry_key != gk0 and cache.wind_key == 2


# ---------------------------------------------------------------------------
# 增量（per_class_indices_add_remove）
# ---------------------------------------------------------------------------
def test_add_strips_keeps_existing_and_remove(mini):
    inputs, cfg, ctx = mini
    strips_all = _strips(ctx)
    init, laps, sites = _corridors(ctx, strips_all)
    full = C.CostCache.build(strips_all, list(inputs.fleet), init, laps, sites, ctx)
    partial = {"A": {k: v for k, v in list(strips_all["A"].items())[:5]}, "B": strips_all["B"]}
    cache = C.CostCache.build(partial, list(inputs.fleet), init, laps, sites, ctx)
    T5 = cache.T_transfer("A").copy()
    e5 = cache.tr_e["A"].copy()
    ent5 = cache.entry["A1"].copy()
    rtb5 = cache.rtb[("A", "H")].copy()
    bud5 = cache.budget.wh["A"].copy()
    cache.add_strips("A", list(strips_all["A"].values())[5:], list(inputs.fleet), init, sites, ctx)
    assert cache.strip_ids["A"] == full.strip_ids["A"] and cache.idx["A"] == full.idx["A"]
    T7 = cache.T_transfer("A")
    assert np.array_equal(T7[:5, :5], T5) and np.array_equal(cache.tr_e["A"][:5, :5], e5)
    assert np.array_equal(cache.entry["A1"][:5], ent5) and np.array_equal(cache.rtb[("A", "H")][:5], rtb5)
    assert np.array_equal(cache.budget.wh["A"][:5], bud5)
    # 與一次建滿 7 條相同
    for name in ("tr", "tr_e", "tr_len", "tr_dz", "op", "op_e", "static_ok"):
        assert np.allclose(getattr(cache, name)["A"], getattr(full, name)["A"]), name
    assert np.array_equal(cache.tr_word["A"], full.tr_word["A"])
    assert np.allclose(cache.entry["A1"], full.entry["A1"]) and np.allclose(cache.rtb[("A", "H")], full.rtb[("A", "H")])
    assert np.allclose(cache.budget.wh["A"], full.budget.wh["A"])
    assert cache.geometry_key == full.geometry_key
    # 移除中間兩條 → 索引壓縮、其餘值不變
    cache.remove_strips("A", ["A2", "A5"])
    keep = [0, 2, 3, 5, 6]
    assert cache.strip_ids["A"] == ["A1", "A3", "A4", "A6", "A7"] and cache.idx["A"]["A7"] == 4
    assert np.array_equal(cache.tr["A"], full.tr["A"][np.ix_(keep, keep)])
    assert np.array_equal(cache.T_transfer("A"), full.T_transfer("A")[np.ix_(keep, keep)])
    assert np.array_equal(cache.entry["A1"], full.entry["A1"][keep]) and np.array_equal(cache.rtb[("A", "H")], full.rtb[("A", "H")][keep])
    assert np.array_equal(cache.budget.wh["A"], full.budget.wh["A"][keep]) and cache.budget.strip_ids["A"] == cache.strip_ids["A"]
    assert cache.budget.min_rtb_wh("A", "A7", -1) == full.budget.min_rtb_wh("A", "A7", -1)
    assert "B" in cache.tr and cache.tr["B"].shape == (3, 3, 4)          # 另一構型不受影響
    # 新構型增量（無既有列）與重複 id
    with pytest.raises(ValueError):
        cache.add_strips("A", [strips_all["A"]["A1"]], list(inputs.fleet), init, sites, ctx)


# ---------------------------------------------------------------------------
# 能量預算表（budget_table_o1、budget_table_terrain_detour）與靜態濾網（static_filter_h4）
# ---------------------------------------------------------------------------
def test_budget_table_o1_and_point_upper_bound(built):
    ctx, strips, init, laps, sites, cache = built
    bud = cache.budget
    m = ctx.models_rep["A"]
    s3 = strips["A"]["A3"]
    # §W.8：min_rtb_wh("A", strip3, −1) = P_c·422.3·k_unc/3600 + e_LDG（=153.6，平地）；本表另含 §8.4 下降功率折抵（≈ −4.8 Wh）
    e_ref = m.p_cruise_w() * 422.3 * m.k_unc / 3600.0 + m.e_landing_wh()
    v = bud.min_rtb_wh("A", "A3", -1)
    assert e_ref - 6.0 <= v <= e_ref + 0.5
    assert bud.best_site("A", "A3", -1) == "H" and bud.rtb_wh("A", "A3", -1, "H") == v
    assert bud.rtb_t("A", "A3", -1, "H") == pytest.approx(422.3, abs=0.5)
    assert np.all(bud.laps["A"] == 0)                                     # 平地：無爬升／下降螺旋
    # O(1) 查詢：端點處 point_wh == 表值（粗篩）；離端點越遠越大；Ω/U 最外點 ≥ 表值
    vg_w = max(ctx.cfg.v_g_min_mps, 21.5 - 5.0)
    at_end = bud.point_wh("A", s3.pR, s3.z_op_m, "H")
    assert at_end == pytest.approx(v)
    far = bud.point_wh("A", s3.pR + np.array([0.0, 60.0]), s3.z_op_m, "H")
    assert far > at_end
    assert far - at_end == pytest.approx(m.p_cruise_w() * 60.0 / vg_w * m.k_unc / 3600.0, rel=1e-6)
    assert bud.extreme_point_wh("A", "A3", -1, "H") >= v
    # R-06：point_wh **不是**上界（只收直線轉場）；低估量的上界 = point_wh_slack_wh，
    # point_wh_ub 才是 O(1) 上界。舊文件字面說「任意點上界化」，兩個安全檢查也照上界用，故補齊這兩個方法。
    slack = m.p_cruise_w() * 2.0 * math.pi * bud._r_eff("A") / vg_w * m.k_unc / 3600.0
    assert bud.point_wh_slack_wh("A") == pytest.approx(slack, rel=1e-6)   # 平地無 keep-out → 只有 2πR_eff
    assert bud.point_wh_ub("A", s3.pR, s3.z_op_m, "H") == pytest.approx(at_end + slack, rel=1e-6)
    # 精算值（自端點、沿條帶航向）不應超過 O(1) 上界
    th = float(s3.end_state(-1).theta_rad)
    ex = bud.point_wh_exact("A", s3.pR, s3.z_op_m, th, "H")
    assert math.isfinite(ex) and ex <= bud.point_wh_ub("A", s3.pR, s3.z_op_m, "H") + 1e-6
    # 低於端點高度之點加爬升項
    low = bud.point_wh("A", s3.pR, s3.z_op_m - 50.0, "H")
    gc = A.gamma_c_auw(m.auw_kg)
    kappa = math.cos(math.radians(A.max_bank_angle_deg))                   # R-06：弧段爬升率折減
    assert low - at_end == pytest.approx(
        (m.p_climb_w(gc) - m.p_cruise_w()) * 50.0 / (21.5 * math.sin(gc) * kappa) * m.k_unc / 3600.0, rel=1e-6)
    # B（§W.8：e^alt ≈ 57.4 Wh）
    mb = ctx.models_rep["B"]
    e_ref_b = mb.p_cruise_w() * 490.7 * mb.k_unc / 3600.0 + mb.e_landing_wh()
    assert e_ref_b - 3.0 <= bud.min_rtb_wh("B", "B2", -1) <= e_ref_b + 0.5


def test_budget_table_terrain_detour(mini):
    """谷地 → 稜線（規格 §5.5 反例）：B 條帶與 S^app 之間加 +350 m 山脊 → 備降能量 ≥ 平地值 + 爬升能量 + 螺旋。"""
    inputs, cfg, ctx = mini
    zB = _z_op(ctx, 100.0)
    strips = {"B": {"B1": _strip_b(1, -337.8, zB)}}
    init, laps, sites = _corridors(ctx, _strips(ctx))
    sites_b = {"H": sites["H"]}
    flat = C.EnergyBudgetTable(strips, sites_b, ctx.models_rep, ctx.specs, ctx.wind, ctx.keepouts, ctx.terrain, ctx.cfg,
                               h_clr_m=ctx.h_clr_m)
    terr = ctx.terrain.copy()
    xs = terr.x0_m + terr.step_m * np.arange(terr.nx)
    band = (xs >= 1200.0) & (xs <= 2200.0)
    terr.z[:, band] += 350.0
    ridge = C.EnergyBudgetTable(strips, sites_b, ctx.models_rep, ctx.specs, ctx.wind, ctx.keepouts, terr, ctx.cfg,
                                h_clr_m=ctx.h_clr_m)
    e_flat = flat.min_rtb_wh("B", "B1", -1)
    e_ridge = ridge.min_rtb_wh("B", "B1", -1)
    m = ctx.models_rep["B"]
    gc = B.gamma_c_auw(m.auw_kg)
    dz_need = 350.0 + ctx.h_clr_m - 100.0            # 稜線上方 h_clr 相對 z_op（AGL 100）
    e_climb = (m.p_climb_w(gc) - m.p_cruise_w()) * dz_need / (19.0 * math.sin(gc)) * m.k_unc / 3600.0
    assert e_climb > 8.0                              # 規格反例量級（350 m ≈ 12.8 Wh）
    assert e_ridge >= e_flat + 0.8 * e_climb
    # 直線估計（P_c·L/V_g^worst + e_LDG）完全遺漏爬升項
    L = ridge.paths[("B", "H", "B1", 0)].length_m
    straight = m.p_cruise_w() * L / max(ctx.cfg.v_g_min_mps, 19.0 - 5.0) * m.k_unc / 3600.0 + m.e_landing_wh()
    assert e_ridge >= straight + 0.8 * e_climb
    # 稜線（x ≤ 2200）之後至 S^app（x = 489）不足以 4° 下降 300 m → 下降螺旋；時間亦增加
    assert ridge.laps["B"][0, 0, 0, 1] >= 1
    assert ridge.rtb_t("B", "B1", -1, "H") > flat.rtb_t("B", "B1", -1, "H")
    assert np.all(flat.laps["B"] == 0)


def test_static_filter_h4(mini):
    inputs, cfg, ctx = mini
    strips = _strips(ctx)
    init, laps, sites = _corridors(ctx, strips)
    cache = C.CostCache.build(strips, list(inputs.fleet), init, laps, sites, ctx)
    assert cache.static_ok["A"].all() and cache.static_ok["B"].all()
    # 代表模型可用能量極低 → 遠端條帶不可安全抵達（r·E_nom + e_alt + e(H→端點) > η_T E_avail − e_TO）
    mA = ctx.models_rep["A"]
    poor = dataclasses.replace(mA, available_wh=mA.reserve_wh() + mA.e_takeoff_wh() + 60.0)
    ctx2 = dataclasses.replace(ctx, models_rep={"A": poor, "B": ctx.models_rep["B"]})
    cache2 = C.CostCache.build(strips, list(inputs.fleet), init, laps, sites, ctx2)
    assert not cache2.static_ok["A"].any() and cache2.static_ok["B"].all()
    # enforce_energy=False → 全部 True
    ctx3 = dataclasses.replace(ctx2, cfg=dataclasses.replace(ctx.cfg, enforce_energy=False))
    cache3 = C.CostCache.build(strips, list(inputs.fleet), init, laps, sites, ctx3)
    assert cache3.static_ok["A"].all()


# ---------------------------------------------------------------------------
# V_a* 選擇（規格 §4.4）
# ---------------------------------------------------------------------------
class _CapSensor:
    def __init__(self, cap: float) -> None:
        self.cap = cap

    def max_ground_speed_mps(self, h: float) -> float:
        return self.cap

    def agl_window(self):
        return (0.0, 1e3)


def test_op_cost_v_air_star_cap_and_infeasible(built):
    ctx, strips, init, laps, sites, cache = built
    s = strips["A"]["A1"]
    m = ctx.models_rep["A"]
    w = ctx.wind.wind_vec([s.z_op_m])[0]
    # 東向順風 V_g = 26.5 > cap 22 → V_a* 降至使 V_g = 22 = 17 m/s（≥ v_min 18? 否 → 不可行）；cap 24 → V_a* = 19
    pc = C.op_cost(s, -1, A, _CapSensor(24.0), ctx.wind, m, ctx.cfg)
    assert pc.t_s == pytest.approx(3136.0 / 24.0)
    va, vg = C.v_air_star(EAST, 18.0, 21.5, 24.0, w)
    assert va[0] == pytest.approx(19.0) and vg[0] == pytest.approx(24.0)
    assert C.op_cost(s, -1, A, _CapSensor(22.0), ctx.wind, m, ctx.cfg).t_s == math.inf     # V_g(V_a,min)=23 > 22
    assert C.op_cost(s, 1, A, _CapSensor(22.0), ctx.wind, m, ctx.cfg).t_s == pytest.approx(190.1, abs=0.1)   # 逆風不受限
    assert C.op_cost(s, 1, A, _CapSensor(math.inf), ctx.wind, m, ctx.cfg).t_s == pytest.approx(190.1, abs=0.1)
    # 側風 ≥ V_a → nan → inf
    va, vg = C.v_air_star(0.0, 18.0, 21.5, math.inf, np.array([25.0, 0.0]))
    assert np.isnan(va[0]) and np.isnan(vg[0])
    # 單向可行胞：順風超 cap 而逆風可行 → op[l,0] = inf、op[l,1] 有限（D34 同向轉場仍可覆蓋）
    cache_cap = C.CostCache.build({"A": strips["A"]}, [ctx.uav("A1")], init, laps, sites, ctx,
                                  strip_sensors={sid: _CapSensor(22.0) for sid in strips["A"]})
    assert np.isinf(cache_cap.op["A"][:, 0]).all() and np.isfinite(cache_cap.op["A"][:, 1]).all()
    T = cache_cap.T_transfer("A")
    assert np.isinf(T[:, :, 0]).all() and np.isinf(T[:, :, 2]).all()      # 終端 LR 者不可行
    assert np.isfinite(T[0, 1, 3]) and np.isfinite(T[0, 1, 1])            # 同向 racetrack（k=3）與反向（k=1）可行


# ---------------------------------------------------------------------------
# §8.4 高度差延長、keep-out 繞行、同向型關閉
# ---------------------------------------------------------------------------
def test_dz_extension_spiral_and_climb_power(built):
    ctx, strips, init, laps, sites, cache = built
    m = ctx.models_rep["A"]
    s1 = strips["A"]["A1"]
    s4_flat = strips["A"]["A4"]
    s4_up = dataclasses.replace(s4_flat, z_op_m=s4_flat.z_op_m + 300.0)
    flat = C.transfer_cost(s1, -1, s4_flat, 1, A, ctx.wind, m, ctx.keepouts, ctx.cfg, r_eff_m=200.0)
    up = C.transfer_cost(s1, -1, s4_up, 1, A, ctx.wind, m, ctx.keepouts, ctx.cfg, r_eff_m=200.0)
    gc = A.gamma_c_auw(m.auw_kg)
    assert up.length_m == pytest.approx(flat.length_m)                     # 幾何相同
    assert up.dz_m == pytest.approx(300.0) and up.n_laps >= 1
    assert up.t_s >= 300.0 / (21.5 * math.sin(gc))
    # 圈數 = ⌈(Δz − L·tanγ^g)/Δh_lap⌉（順風保守 tanγ^g、Δh_lap 同 altitude.orbit_gain）
    dh_lap, t_lap = ALT.orbit_gain(A, ctx.wind, 0.5 * (s1.z_op_m + s4_up.z_op_m), m.auw_kg, 200.0, "climb")
    tan_g = A.tan_gamma_ground("climb", 21.5 + 5.0, m.auw_kg, arc=False)
    n_exp = math.ceil((300.0 - flat.length_m * tan_g) / dh_lap - 1e-9)
    assert up.n_laps == n_exp
    assert up.t_s == pytest.approx(max(flat.t_s, 300.0 / (21.5 * math.sin(gc))) + n_exp * t_lap, rel=1e-6)
    assert up.e_wh == pytest.approx(m.transfer_wh(up.t_s - n_exp * t_lap, 300.0, n_exp, t_lap), rel=1e-9)
    assert up.e_wh > flat.e_wh + (m.p_climb_w(gc) - m.p_cruise_w()) * 300.0 / (21.5 * math.sin(gc)) * m.k_unc / 3600.0 * 0.99
    # 下降：t 至少 |Δz|/(Va sinγ_d)，能量低於等高
    s4_dn = dataclasses.replace(s4_flat, z_op_m=s4_flat.z_op_m - 40.0)
    dn = C.transfer_cost(s1, -1, s4_dn, 1, A, ctx.wind, m, ctx.keepouts, ctx.cfg, r_eff_m=200.0)
    assert dn.t_s >= 40.0 / (21.5 * math.sin(A.gamma_d())) and dn.e_wh < flat.e_wh and dn.n_laps == 0
    # 快取版一致（tr_dz、tr_laps）
    strips2 = {"A": {"A1": s1, "A4": s4_up}}
    c2 = C.CostCache.build(strips2, [ctx.uav("A1")], init, laps, sites, ctx)
    assert c2.tr_dz["A"][0, 1, 1] == pytest.approx(300.0) and c2.tr_laps["A"][0, 1, 1] == n_exp
    assert c2.tr["A"][0, 1, 1] == pytest.approx(up.t_s, rel=1e-9) and c2.tr_e["A"][0, 1, 1] == pytest.approx(up.e_wh, rel=1e-9)


def test_transfer_keepout_avoid(mini):
    """迴轉區（條帶東端外）之電塔 → 該轉場繞行（word AVOID、長度增加）；其他轉場不受影響。"""
    inputs, cfg, ctx = mini
    strips = _strips(ctx)
    init, laps, sites = _corridors(ctx, strips)
    zA = strips["A"]["A1"].z_op_m
    tower = Point(7250.0, 250.0)
    ks = KeepoutSet(polys=[tower.buffer(50.0)], tops_z=[zA + 500.0], ids=["T"], tree=shapely.STRtree([tower.buffer(50.0)]),
                    geoms=[tower], buffers_m=[50.0])
    ctx2 = dataclasses.replace(ctx, keepouts=ks)
    base = C.CostCache.build(strips, list(inputs.fleet), init, laps, sites, ctx)
    cache = C.CostCache.build(strips, list(inputs.fleet), init, laps, sites, ctx2)
    # 1E → 4W（U 型於 x ∈ [7068, 7268]、y ∈ [−1, 503]）被電塔擋住 → 改走淨空之路徑（非最短 word 或切線圖 AVOID）
    p = cache.tr_paths["A"][("A1", "A4", 1)]
    assert p.word == AVOID_WORD or p.word in WORDS
    assert cache.tr_word["A"][0, 3, 1] == (C.WORD_AVOID_IDX if p.word == AVOID_WORD else WORDS.index(p.word))
    assert cache.tr_word["A"][0, 3, 1] != base.tr_word["A"][0, 3, 1]
    assert cache.tr_len["A"][0, 3, 1] > base.tr_len["A"][0, 3, 1] and cache.tr["A"][0, 3, 1] > base.tr["A"][0, 3, 1]
    assert cache.tr_len["A"][0, 3, 1] == pytest.approx(p.length_m)
    assert not ks.intersects(p.polyline(5.0), cfg.b_trk_m, zA)
    assert ks.intersects(dubins_path(strips["A"]["A1"].end_state(-1), strips["A"]["A4"].start_state(1), 200.0).polyline(5.0),
                         cfg.b_trk_m, zA)
    # 單筆函式一致
    m = ctx.models_rep["A"]
    pc = C.transfer_cost(strips["A"]["A1"], -1, strips["A"]["A4"], 1, A, ctx.wind, m, ks, ctx.cfg, r_eff_m=200.0)
    assert pc.path.word == p.word and pc.t_s == pytest.approx(cache.tr["A"][0, 3, 1], rel=1e-9)
    # 切線圖繞行（AVOID word）：以電塔正好落在 U 型直線段上、且環型亦被擋之組態驗證 word 標記
    pts = [Point(7268.0, 250.0), Point(6868.0, 250.0)]          # U 型直線段與環型皆被擋 → 切線圖繞行
    ks_wall = KeepoutSet(polys=[p_.buffer(50.0) for p_ in pts], tops_z=[zA + 500.0] * 2, ids=["T1", "T2"],
                         tree=shapely.STRtree([p_.buffer(50.0) for p_ in pts]), geoms=pts, buffers_m=[50.0, 50.0])
    pc_w = C.transfer_cost(strips["A"]["A1"], -1, strips["A"]["A4"], 1, A, ctx.wind, m, ks_wall, ctx.cfg, r_eff_m=200.0)
    assert pc_w.path is not None and pc_w.path.word == AVOID_WORD and pc_w.length_m > base.tr_len["A"][0, 3, 1]
    assert not ks_wall.intersects(pc_w.path.polyline(5.0), cfg.b_trk_m, zA)
    cache_w = C.CostCache.build({"A": strips["A"]}, [ctx.uav("A1")], init, laps, sites, dataclasses.replace(ctx, keepouts=ks_wall))
    assert cache_w.tr_word["A"][0, 3, 1] == C.WORD_AVOID_IDX and cache_w.tr["A"][0, 3, 1] == pytest.approx(pc_w.t_s, rel=1e-9)
    # 西端轉場（2 → 1 反向自左端）不受影響
    assert cache.tr_word["A"][1, 0, 2] == base.tr_word["A"][1, 0, 2] and cache.tr["A"][1, 0, 2] == pytest.approx(base.tr["A"][1, 0, 2])
    # 電塔頂低於作業高度 → 不繞行（2.5D）
    ks_low = KeepoutSet(polys=[tower.buffer(50.0)], tops_z=[zA - 10.0], ids=["T"], tree=shapely.STRtree([tower.buffer(50.0)]),
                        geoms=[tower], buffers_m=[50.0])
    cache_low = C.CostCache.build(strips, list(inputs.fleet), init, laps, sites, dataclasses.replace(ctx, keepouts=ks_low))
    assert np.array_equal(cache_low.tr_len["A"], base.tr_len["A"])
    # 起點被包圍 → inf
    ks_enc = KeepoutSet(polys=[Point(7068.0, 503.0).buffer(120.0)], tops_z=[zA + 500.0], ids=["E"],
                        tree=shapely.STRtree([Point(7068.0, 503.0).buffer(120.0)]))
    pc_enc = C.transfer_cost(strips["A"]["A1"], -1, strips["A"]["A4"], 1, A, ctx.wind, m, ks_enc, ctx.cfg, r_eff_m=200.0)
    assert pc_enc.t_s == math.inf and pc_enc.e_wh == math.inf


def test_transfer_types_rev_only(mini):
    inputs, cfg, ctx = mini
    strips = _strips(ctx)
    init, laps, sites = _corridors(ctx, strips)
    ctx2 = dataclasses.replace(ctx, cfg=dataclasses.replace(ctx.cfg, transfer_types=("rev",)))
    cache = C.CostCache.build(strips, list(inputs.fleet), init, laps, sites, ctx2)
    assert np.isinf(cache.tr["A"][:, :, [0, 3]]).all()
    off = ~np.eye(7, dtype=bool)
    assert np.isfinite(cache.tr["A"][:, :, 1][off]).all() and np.isfinite(cache.tr["A"][:, :, 2][off]).all()
    m = ctx.models_rep["A"]
    pc = C.transfer_cost(strips["A"]["A1"], -1, strips["A"]["A2"], -1, A, ctx.wind, m, ctx.keepouts, ctx2.cfg, r_eff_m=200.0)
    assert pc.t_s == math.inf
    assert C.transfer_cost(strips["A"]["A1"], -1, strips["A"]["A1"], 1, A, ctx.wind, m, ctx.keepouts, ctx.cfg).t_s == math.inf


# ---------------------------------------------------------------------------
# 效能：30 條 × 4 型 < 1 s
# ---------------------------------------------------------------------------
def test_build_timing_30_strips(mini):
    inputs, cfg, ctx = mini
    zA = _z_op(ctx, 160.0)
    strips = {"A": {f"S{i}": _strip_a(i + 1, 503.0 - 85.0 * i, zA, sid=f"S{i}") for i in range(30)}}
    init, laps, sites = _corridors(ctx, _strips(ctx))
    C.CostCache.build({"A": {"S0": strips["A"]["S0"]}}, [ctx.uav("A1")], init, laps, sites, ctx)   # 暖機（import／JIT 無關）
    t0 = time.perf_counter()
    cache = C.CostCache.build(strips, [ctx.uav("A1")], init, laps, sites, ctx)
    dt = time.perf_counter() - t0
    assert cache.tr["A"].shape == (30, 30, 4) and cache.budget.wh["A"].shape == (30, 2, 1)
    off = ~np.eye(30, dtype=bool)
    assert np.isfinite(cache.tr["A"][off]).all()
    assert dt < 1.0, f"CostCache.build for 30 strips took {dt:.2f} s"
    t0 = time.perf_counter()
    cache.refresh_wind(_wf(ctx, 5.0, 180.0, version=3), ctx)
    assert time.perf_counter() - t0 < 1.0
