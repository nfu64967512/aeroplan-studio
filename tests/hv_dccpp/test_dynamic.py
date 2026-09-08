"""tests/test_dynamic.py — 契約 §12 `dynamic.py` 不變量 + 規格 §17.2 清單。

情境：`scenario.build_mini_example()`（平地、西風 5 m/s；A1 + B1、Z1 = any）以 dem_step 50 / terrain_margin 1500 建構，
`planner.plan_mission` 一次（module fixture），每個測試以新 ctx + `dynamic.cache_from_plan` 重建快取（handler 會就地
更新 cache／ctx.models.κ／ctx.wind）。事件時刻 t_mid = A1 第三條帶中點（A1、B1 皆在空）。

規格 §17.2：uav_exit_partial_strip；uav_exit_single_uav_team_affected_set；new_area_incremental；energy_deviation_kappa
（未驗證用 E_pred）；link_loss_ghost_track；wind_update_time_layer_only；cue_dispatches_B；cue_gsd_unattainable_orbit；
priority_preempt；weather_abort_landing_order（能量餘裕）；weather_abort_all_over_limit。
"""
from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

from hv_dccpp import dynamic as dyn
from hv_dccpp.context import build_context
from hv_dccpp.costs import CostCache
from hv_dccpp.mission import AreaSpec, Phase, WindLayer
from hv_dccpp.planner import plan_mission
from hv_dccpp.scenario import build_mini_example
from hv_dccpp.wind import WindField

# ---------------------------------------------------------------------------
# fixtures
# ---------------------------------------------------------------------------
STEP = 50.0
MARGIN = 1500.0


def _build(inputs, cfg):
    return build_context(inputs, cfg, terrain_margin_m=MARGIN)


@pytest.fixture(scope="module")
def base():
    inputs, cfg = build_mini_example()
    cfg = dataclasses.replace(cfg, dem_step_m=STEP)
    ctx = _build(inputs, cfg)
    plan = plan_mission(inputs, cfg, ctx=ctx)
    assert plan.feasibility.status != "NO_GO" and set(plan.plans) == {"A1", "B1"}
    a1 = plan.plans["A1"]
    spans = dyn._spans(a1)
    assert len(spans) >= 3
    t_mid = 0.5 * (spans[2].t0 + spans[2].t_end)
    assert dyn._status(plan.plans["B1"], t_mid).landed is False
    return {"inputs": inputs, "cfg": cfg, "plan": plan, "t_mid": t_mid}


@pytest.fixture
def env(base):
    """每測試新 ctx／cache（handler 會就地更新 cache 與 ctx.models 之 κ）。"""
    ctx = _build(base["inputs"], base["cfg"])
    cache = dyn.cache_from_plan(base["plan"], ctx)
    return {**base, "ctx": ctx, "cache": cache}


def _latlon(ctx, x, y):
    ll = ctx.frame.to_wgs84(np.array([[float(x), float(y), 0.0]]))[0]
    return float(ll[0]), float(ll[1])


def _rect_area(ctx, aid, x0, y0, x1, y1, **kw):
    ring = [_latlon(ctx, x0, y0), _latlon(ctx, x1, y0), _latlon(ctx, x1, y1), _latlon(ctx, x0, y1)]
    return AreaSpec(aid, tuple(ring), **kw)


def _check_incremental(old_plan, res):
    """契約不變量：未變更機 UAVPlan 同一物件；變更機既有航點（至錨點）同一 Waypoint 物件、diff = 錨點後航點。"""
    for u, p in res.plan.plans.items():
        old = old_plan.plans.get(u)
        if u not in res.changed_uavs:
            assert p is old, f"{u} unchanged but new object"
            assert u not in res.diff
            continue
        assert u in res.diff
        tail = res.diff[u]
        k = len(p.waypoints) - len(tail)
        assert k >= 0
        assert all(a is b for a, b in zip(p.waypoints[:k], (old.waypoints if old else [])[:k]))
        assert all(a is b for a, b in zip(p.waypoints[k:], tail))
        ts = [w.t_s for w in p.waypoints]
        assert all(t1 >= t0 - 1e-9 for t0, t1 in zip(ts, ts[1:]))
    for u in res.changed_uavs:
        if u not in res.plan.plans:
            assert res.diff.get(u) == []


# ---------------------------------------------------------------------------
# snapshot / affected_set
# ---------------------------------------------------------------------------
def test_snapshot_states_and_residual(env):
    plan, ctx, t = env["plan"], env["ctx"], env["t_mid"]
    states, uncovered, residual = dyn.snapshot(plan, {}, t, ctx)
    assert set(states) == {"A1", "B1"}
    a1 = plan.plans["A1"]
    st = dyn._status(a1, t)
    assert st.current is not None and st.current.strip_id in residual
    for sp in st.pending:
        assert sp.strip_id in uncovered
    for sp in st.done:
        assert sp.strip_id not in uncovered
    r = residual[st.current.strip_id]
    assert r.strip_id.endswith("#r") and r.cls == "A"
    s = plan.strips[st.current.strip_id]
    # 殘段在原線上、長度 = (1−progress)·L + e_ext
    e_ext = dyn._e_ext_of(plan, s, ctx)
    assert e_ext > 0
    L = float(np.hypot(*(s.pR - s.pL)))
    assert r.length_m == pytest.approx((1.0 - st.progress) * L + e_ext, abs=1.0)
    end = s.pR if st.current.dir == -1 else s.pL
    r_end = r.pR if st.current.dir == -1 else r.pL
    assert np.allclose(end, r_end)
    st_a = states["A1"]
    assert st_a.t_s == pytest.approx(t) and st_a.e_rem_wh == pytest.approx(dyn._plan_energy_at(a1, t))


def test_snapshot_uses_telemetry(env):
    plan, ctx, t = env["plan"], env["ctx"], env["t_mid"]
    a1 = plan.plans["A1"]
    cur = dyn._status(a1, t).current
    tel = dyn.Telemetry("A1", t, 5500.0, 300.0, 160.0, 1.5, 22.0, None, cur.strip_id, 0.3, True)
    states, _u, residual = dyn.snapshot(plan, {"A1": tel}, t, ctx)
    assert states["A1"].x_m == 5500.0 and states["A1"].y_m == 300.0 and states["A1"].z_m == 160.0
    s = plan.strips[cur.strip_id]
    L = float(np.hypot(*(s.pR - s.pL)))
    assert residual[cur.strip_id].length_m == pytest.approx(0.7 * L + dyn._e_ext_of(plan, s, ctx), abs=1.0)
    # 未驗證電量監測：遙測 e_tel 不直接作 E_rem（§5.6），採 E_pred(κ)
    tel2 = dataclasses.replace(tel, e_tel_wh=400.0)
    states2, _u, _r = dyn.snapshot(plan, {"A1": tel2}, t, ctx)
    assert states2["A1"].e_rem_wh == pytest.approx(dyn._plan_energy_at(a1, t))


def test_uav_exit_single_uav_team_affected_set(env):
    plan, ctx, cache, cfg, t = env["plan"], env["ctx"], env["cache"], env["cfg"], env["t_mid"]
    states, _u, _r = dyn.snapshot(plan, {}, t, ctx)
    lost = [sp.strip_id for sp in dyn._status(plan.plans["A1"], t).pending]
    aff = dyn.affected_set(plan, lost, states, ctx, cache, cfg)
    assert "B1" in aff                                   # any 區：B（不同團隊）可接手
    # lidar 區 → B 不相容
    cells = [dataclasses.replace(c, sensor_req="lidar") for c in plan.cells]
    view = dataclasses.replace(plan, cells=cells)
    assert "B1" not in dyn.affected_set(view, lost, states, ctx, cache, cfg)
    # 已落地者除外
    states_late = {u: dataclasses.replace(s, t_s=1e6) for u, s in states.items()}
    assert dyn.affected_set(plan, lost, states_late, ctx, cache, cfg) == []


# ---------------------------------------------------------------------------
# UAV_EXIT
# ---------------------------------------------------------------------------
def test_uav_exit_partial_strip(env):
    plan, ctx, cache, cfg, t = env["plan"], env["ctx"], env["cache"], env["cfg"], env["t_mid"]
    st = dyn._status(plan.plans["A1"], t)
    lost = [sp.strip_id for sp in st.pending] + [st.current.strip_id + "#r"]
    res = dyn.replan(plan, dyn.UAVExit("A1", t, "motor_failure"), {}, ctx, cache, cfg)
    # R-10：退出機的計畫不是整份刪掉，而是截到 t_exit —— 已飛航段（＝已採集的資料）必須留在紀錄裡，
    # 否則 sim.coverage_raster 會把那些區域算成未覆蓋，和 plan.uncovered 互相矛盾。
    assert "A1" in res.changed_uavs
    a1 = res.plan.plans["A1"]
    assert a1.waypoints and a1.waypoints[-1].t_s <= t + 1e-6          # 沒有 t_exit 之後的航點
    assert len(a1.waypoints) < len(plan.plans["A1"].waypoints)
    assert a1.t_land_s == pytest.approx(a1.waypoints[-1].t_s)
    assert any(w.startswith("uav_exited_at_t=") for w in a1.warnings)
    assert all(sid not in {sp.strip_id for sp in st.pending} for sid, _d in a1.strips)   # 未飛的不留
    _check_incremental(plan, res)
    b1 = res.plan.plans["B1"]
    covered = "B1" in res.changed_uavs
    unc = {s for s, _ in res.plan.uncovered}
    if covered:
        # 跨構型接手（any → B 重鋪殘區）：新條帶尾端追加、在快取中、殘區胞登記
        assert any(n.startswith("relaid(") for n in res.notes)
        new_ids = [s for s, _d in b1.strips if s not in {x for x, _ in plan.plans["B1"].strips}]
        assert new_ids and all(s in cache.idx["B"] for s in new_ids)
        assert all(s in res.plan.strips for s in new_ids)
        assert any(c.residual_of is not None for c in res.plan.cells)
        assert b1.t_land_s > plan.plans["B1"].t_land_s
        assert b1.waypoints[-1].phase is Phase.LAND
    else:
        assert all(s in unc for s in lost)
    assert res.plan.diff_from == "UAVExit"
    assert res.timing_ms["total"] > 0 and res.placeholder_used is False


# ---------------------------------------------------------------------------
# NEW_AREA
# ---------------------------------------------------------------------------
def test_new_area_incremental(env):
    plan, ctx, cache, cfg, t = env["plan"], env["ctx"], env["cache"], env["cfg"], env["t_mid"]
    area = _rect_area(ctx, "NEW1", 4500, 1200, 5300, 1700, sensor_req="rgb", priority=1.0, payload_req_id="default_rgb")
    n_b_before = len(cache.idx["B"])
    res = dyn.replan(plan, dyn.NewArea(area, t), {}, ctx, cache, cfg)
    _check_incremental(plan, res)
    assert res.plan.plans["A1"] is plan.plans["A1"]          # A 不變（rgb 區）
    assert res.changed_uavs == ["B1"]
    b1 = res.plan.plans["B1"]
    old = plan.plans["B1"]
    new_ids = [s for s, _d in b1.strips if s.startswith("NEW1")]
    assert new_ids
    assert b1.strips[: len(old.strips)] == old.strips           # 既有序列不重排，尾端追加
    assert len(cache.idx["B"]) == n_b_before + len(new_ids)
    assert any(c.area_id == "NEW1" for c in res.plan.cells)
    assert all(s in res.plan.strips for s in new_ids)
    assert not [u for u, _ in res.plan.uncovered if u.startswith("NEW1")]
    assert b1.waypoints[-1].phase is Phase.LAND and b1.energy_margin_wh >= 0


def test_new_area_precheck_infeasible(env):
    """鏈路外之新區 → precheck 失敗 → uncovered 記錄，無機變更。"""
    plan, ctx, cache, cfg, t = env["plan"], env["ctx"], env["cache"], env["cfg"], env["t_mid"]
    area = _rect_area(ctx, "FAR", 40000, 0, 41000, 800, sensor_req="rgb", payload_req_id="default_rgb")
    res = dyn.replan(plan, dyn.NewArea(area, t), {}, ctx, cache, cfg)
    assert res.changed_uavs == []
    assert any(u == "FAR" for u, _ in res.plan.uncovered)
    _check_incremental(plan, res)


# ---------------------------------------------------------------------------
# ENERGY_UPDATE
# ---------------------------------------------------------------------------
def test_energy_deviation_kappa(env):
    plan, ctx, cache, cfg, t = env["plan"], env["ctx"], env["cache"], env["cfg"], env["t_mid"]
    a1 = plan.plans["A1"]
    model = ctx.models["A1"]
    spec = ctx.specs["A"]
    assert spec.battery_monitor_validated is False
    e_start = a1.waypoints[0].energy_remaining_wh
    e_pred = dyn._plan_energy_at(a1, t)
    e_tel = 0.90 * e_pred                                  # δ = 0.10 ∈ (δ1, δ2)
    res = dyn.replan(plan, dyn.EnergyUpdate("A1", t, e_tel), {}, ctx, cache, cfg)
    kappa = min(1.5, max(0.8, (e_start - e_tel) / (e_start - e_pred)))
    assert model.kappa == pytest.approx(kappa)
    assert any(n.startswith("kappa(A1") for n in res.notes)
    # 未驗證 → E_ref = E_pred(κ)，非遙測值
    e_ref = dyn._e_ref(a1, t, e_tel, model, spec)
    assert e_ref == pytest.approx(e_start - kappa * (e_start - e_pred))
    assert not any(n.startswith("divert") for n in res.notes)
    assert res.changed_uavs == []                          # 餘裕充足：不切除
    _check_incremental(plan, res)
    # 驗證後 → clip(E_tel, 0.9 E_pred, 1.1 E_pred)
    spec_v = dataclasses.replace(spec, battery_monitor_validated=True)
    assert dyn._e_ref(a1, t, 0.5 * e_pred, model, spec_v) == pytest.approx(0.9 * e_pred)
    assert dyn._e_ref(a1, t, 2.0 * e_pred, model, spec_v) == pytest.approx(1.1 * e_pred)


def test_energy_update_divert(env):
    plan, ctx, cache, cfg, t = env["plan"], env["ctx"], env["cache"], env["cfg"], env["t_mid"]
    res = dyn.replan(plan, dyn.EnergyUpdate("A1", t, 400.0), {}, ctx, cache, cfg)   # δ ≫ δ2 → 保守 DIVERT
    assert any(n.startswith("divert") for n in res.notes)
    _check_incremental(plan, res)
    a1 = res.plan.plans["A1"]
    old = plan.plans["A1"]
    assert len(a1.strips) < len(old.strips)
    assert a1.waypoints[-1].phase is Phase.LAND
    assert a1.t_land_s < old.t_land_s
    tail = res.diff["A1"]
    assert tail and tail[0].t_s == pytest.approx(t, abs=1e-6)       # 立即中止：自現時狀態返航
    assert not any(w.phase is Phase.OPERATE for w in tail)
    lost = {s for s, _ in old.strips} - {s for s, _ in a1.strips}
    unc = {s for s, _ in res.plan.uncovered}
    assert "B1" in res.changed_uavs or lost <= unc


def test_energy_update_cut_and_rtb(env):
    """E_ref − e(剩餘含 RTB) < r·E_nom 但不需 DIVERT → 自尾端逐條移除（保留現行條帶端點為錨點）。"""
    plan, ctx, cache, cfg = env["plan"], env["ctx"], env["cache"], env["cfg"]
    a1 = plan.plans["A1"]
    t = dyn._spans(a1)[0].t0 + 5.0                                       # 首條帶剛開始：待掃最多
    model = ctx.models["A1"]
    ctx.models["A1"] = dataclasses.replace(model, nominal_wh=3400.0)     # 保留 1020 Wh → 尾端不夠
    cfg = dataclasses.replace(cfg, energy_margin_frac=0.0)                # DIVERT 門檻 = 保留 + e_alt（無額外 margin）
    e_tel = dyn._plan_energy_at(a1, t)
    res = dyn.replan(plan, dyn.EnergyUpdate("A1", t, e_tel), {}, ctx, cache, cfg)
    assert any(n.startswith("energy_cut(A1") for n in res.notes), res.notes
    assert not any(n.startswith("divert") for n in res.notes)
    _check_incremental(plan, res)
    new = res.plan.plans["A1"]
    assert 0 < len(new.strips) < len(a1.strips)
    assert new.strips == a1.strips[: len(new.strips)]
    st = dyn._status(a1, t)
    kept = [s for s, _ in new.strips]
    assert st.current.strip_id in kept                     # 現行條帶完成後才返航
    tail = res.diff["A1"]
    assert tail[0].t_s >= t and tail[-1].phase is Phase.LAND
    assert not any(w.phase is Phase.OPERATE for w in tail)
    removed = [s for s, _ in a1.strips[len(new.strips):]]
    unc = {s for s, _ in res.plan.uncovered}
    assert "B1" in res.changed_uavs or set(removed) <= unc


# ---------------------------------------------------------------------------
# LINK_LOSS / LINK_RESTORED
# ---------------------------------------------------------------------------
def test_link_loss_ghost_track(env):
    plan, ctx, cache, cfg, t = env["plan"], env["ctx"], env["cache"], env["cfg"], env["t_mid"]
    a1 = plan.plans["A1"]
    t_ll = ctx.inputs.c2.t_ll_s
    g = dyn.ghost_track("A1", plan, "CONTINUE_THEN_RTB", t, ctx, cfg, cache=cache)
    assert g.uav_id == "A1" and g.samples.shape[1] == 4
    ts = g.samples[:, 3]
    assert ts[0] == pytest.approx(t) and np.all(np.diff(ts) >= -1e-9)
    assert g.t_rtb_s == pytest.approx(t + t_ll)
    # 決策點前沿計畫
    for row in g.samples[: max(1, int(t_ll / cfg.dt_4d_s) - 1)]:
        s = dyn._interp_state(a1, float(row[3]), ctx.specs["A"], ctx.wind)
        assert math.hypot(s.x_m - row[0], s.y_m - row[1]) < 1e-6
    assert g.landing_site == ctx.recovery_site_id
    assert g.eta_s > t + t_ll and g.sigma_t == pytest.approx(cfg.k_t * (g.eta_s - t))
    # 末樣本在降落走廊 P_tc 附近（RTB 終點 = S^app，距 home 不遠）
    home = ctx.sites_enu[g.landing_site][0]
    assert math.hypot(g.samples[-1, 0] - home[0], g.samples[-1, 1] - home[1]) < 1500.0
    g2 = dyn.ghost_track("A1", plan, "RTB", t, ctx, cfg, cache=cache)
    assert g2.t_rtb_s == pytest.approx(t) and g2.eta_s < g.eta_s
    # replan：幽靈登記、該機決策點後條帶 → 接手／待辦
    res = dyn.replan(plan, dyn.LinkLoss("A1", t), {}, ctx, cache, cfg)
    assert len(res.ghosts) == 1 and res.ghosts[0].uav_id == "A1"
    assert "A1" in res.changed_uavs
    _check_incremental(plan, res)
    new = res.plan.plans["A1"]
    assert new.waypoints[-1].phase is Phase.LAND and len(new.strips) < len(a1.strips)
    st_dec = dyn._status(a1, t + t_ll)
    kept = {s for s, _ in new.strips}
    if st_dec.current is not None:
        assert st_dec.current.strip_id in kept                        # CONTINUE：完成 t_ll 時之現行條帶
    assert all(sp.strip_id in kept for sp in st_dec.done)
    lost = {sp.strip_id for sp in st_dec.pending}
    assert not (lost & kept)
    unc = {s for s, _ in res.plan.uncovered}
    assert "B1" in res.changed_uavs or lost <= unc
    # 幽靈可餵入 check_separation（其他機避讓）
    from hv_dccpp.deconflict import check_separation
    check_separation({u: p for u, p in res.plan.plans.items() if u != "A1"}, cfg, res.ghosts)


def test_link_restored_rejoins(env):
    plan, ctx, cache, cfg, t = env["plan"], env["ctx"], env["cache"], env["cfg"], env["t_mid"]
    res1 = dyn.replan(plan, dyn.LinkLoss("A1", t), {}, ctx, cache, cfg)
    t2 = t + 20.0
    a1 = res1.plan.plans["A1"]
    s = dyn._interp_state(a1, t2, ctx.specs["A"], ctx.wind)
    tel = dyn.Telemetry("A1", t2, s.x_m, s.y_m, s.z_m, s.theta_rad, 22.0, None, None, 0.0, True)
    res2 = dyn.replan(res1.plan, dyn.LinkRestored("A1", t2, tel), {}, ctx, cache, cfg, res1.ghosts)
    assert res2.ghosts == []
    _check_incremental(res1.plan, res2)


# ---------------------------------------------------------------------------
# WIND_UPDATE
# ---------------------------------------------------------------------------
def test_wind_update_time_layer_only(env, monkeypatch):
    plan, ctx, cache, cfg, t = env["plan"], env["ctx"], env["cache"], env["cfg"], env["t_mid"]
    calls = {"refresh": 0, "build": 0}
    orig = CostCache.refresh_wind

    def spy(self, wind, ctx_):
        calls["refresh"] += 1
        return orig(self, wind, ctx_)

    def no_build(*a, **k):
        calls["build"] += 1
        raise AssertionError("CostCache.build must not be called by handle_wind_update")

    monkeypatch.setattr(CostCache, "refresh_wind", spy)
    monkeypatch.setattr(CostCache, "build", classmethod(no_build))
    tr_len_before = {c: cache.tr_len[c].copy() for c in cache.tr_len}
    tr_before = {c: cache.tr[c].copy() for c in cache.tr}
    r_eff_before = dict(cache.R_eff)
    v0 = ctx.wind.version
    layers = (WindLayer(0.0, 3000.0, 5.2, 270.0, 7.0),)           # +4 %：R_eff 變化 < 5 % → 幾何層不變
    res = dyn.replan(plan, dyn.WindUpdate(layers, t, None), {}, ctx, cache, cfg)
    assert calls["refresh"] == 1 and calls["build"] == 0
    assert cache.wind_key == v0 + 1 and ctx.wind.version == v0 + 1
    assert res.plan.cache_keys[1] == v0 + 1
    for c in tr_len_before:
        assert np.array_equal(tr_len_before[c], cache.tr_len[c])       # 幾何層位元相同
        assert cache.R_eff[c] == pytest.approx(r_eff_before[c])
        fin = np.isfinite(tr_before[c]) & np.isfinite(cache.tr[c])
        assert not np.allclose(tr_before[c][fin], cache.tr[c][fin])   # 時間層已重算
    assert not any(n.startswith("weather_abort") for n in res.notes)
    _check_incremental(plan, res)


def test_wind_update_abort_on_strong_wind(env):
    plan, ctx, cache, cfg, t = env["plan"], env["ctx"], env["cache"], env["cfg"], env["t_mid"]
    layers = (WindLayer(0.0, 3000.0, 12.0, 270.0, 15.0),)         # > W_abort 10 m/s
    res = dyn.replan(plan, dyn.WindUpdate(layers, t, None), {}, ctx, cache, cfg)
    assert any(n.startswith("weather_abort_triggered") for n in res.notes)
    assert set(res.changed_uavs) == {"A1", "B1"}
    for u in ("A1", "B1"):
        p = res.plan.plans[u]
        assert p.waypoints[-1].phase is Phase.LAND and len(p.strips) <= len(plan.plans[u].strips)
    _check_incremental(plan, res)


# ---------------------------------------------------------------------------
# TARGET_CUE
# ---------------------------------------------------------------------------
def test_cue_dispatches_B(env):
    plan, ctx, cache, cfg, t = env["plan"], env["ctx"], env["cache"], env["cfg"], env["t_mid"]
    cue = dyn.TargetCue(_latlon(ctx, 5500.0, -200.0), "A1", 0.9, t)
    res = dyn.replan(plan, cue, {}, ctx, cache, cfg)
    assert res.changed_uavs == ["B1"] and res.plan.plans["A1"] is plan.plans["A1"]
    assert any(n.startswith("cue_dispatched(B1: mode=strips") for n in res.notes), res.notes
    _check_incremental(plan, res)
    b1 = res.plan.plans["B1"]
    cue_ids = [s for s, _ in b1.strips if s.startswith("CUE_")]
    assert 1 <= len(cue_ids) <= 3
    # 插在現行條帶之後、優先於其餘待掃條帶
    st = dyn._status(plan.plans["B1"], t)
    order = [s for s, _ in b1.strips]
    i_cur = order.index(st.current.strip_id) if st.current is not None else len(dyn._status(plan.plans["B1"], t).done) - 1
    assert order[i_cur + 1] in cue_ids
    assert all(s in cache.idx["B"] and s in res.plan.strips for s in cue_ids)
    cell = next(c for c in res.plan.cells if c.area_id.startswith("CUE_"))
    assert cell.sensor_req == "rgb" and cell.time_window_s[1] - cell.time_window_s[0] == pytest.approx(cfg.t_conf_max_s)
    # 確認高度 = clip(h_max,B(GSD_req), h_minfw, h_ceil)
    h_op = res.plan.strips[cue_ids[0]].h_op_m
    assert ctx.specs["B"].min_fw_operation_agl_m <= h_op <= ctx.h_ceil_m["B"] + 1e-6
    assert b1.waypoints[-1].phase is Phase.LAND
    wps_cue = [w for w in b1.waypoints if w.strip_id in cue_ids and w.phase is Phase.OPERATE]
    assert wps_cue and all(w.sensor_on for w in wps_cue)
    assert min(w.t_s for w in wps_cue) <= t + cfg.t_conf_max_s


def test_cue_gsd_unattainable_orbit(env):
    plan, ctx, cache, cfg, t = env["plan"], env["ctx"], env["cache"], env["cfg"], env["t_mid"]
    cfg2 = dataclasses.replace(cfg, gsd_confirm_m=0.01)               # h_max = 41 m < h_minfw 100 m
    cue = dyn.TargetCue(_latlon(ctx, 5500.0, -200.0), "A1", 0.9, t)
    res = dyn.replan(plan, cue, {}, ctx, cache, cfg2)
    assert any(n.startswith("gsd_unattainable(") for n in res.notes), res.notes
    assert any("mode=orbit" in n for n in res.notes)
    assert res.changed_uavs == ["B1"]
    _check_incremental(plan, res)
    b1 = res.plan.plans["B1"]
    assert not any(s.startswith("CUE_") for s, _ in b1.strips)
    orb = [w for w in b1.waypoints if w.phase is Phase.ORBIT and w.arc is not None
           and math.hypot(w.arc[0] - 5500.0, w.arc[1] + 200.0) < 1.0]            # 確認盤旋（非去衝突延遲盤旋）
    assert orb
    cx, cy, R, _turn = orb[0].arc
    assert R == pytest.approx(cache.R_eff["B"])
    assert orb[-1].t_s - orb[0].t_s >= 1.5 * 2 * math.pi * R / ctx.specs["B"].cruise_airspeed_mps   # ≈ 2 圈
    z_ground = float(ctx.terrain.z_at(np.array([[5500.0, -200.0]]))[0])
    assert orb[0].z_m - z_ground == pytest.approx(ctx.specs["B"].min_fw_operation_agl_m, abs=5.0)   # 最低高度盤旋
    assert b1.waypoints[-1].phase is Phase.LAND


# ---------------------------------------------------------------------------
# PRIORITY_CHANGE
# ---------------------------------------------------------------------------
def test_priority_preempt(env):
    plan, ctx, cache, cfg, t = env["plan"], env["ctx"], env["cache"], env["cfg"], env["t_mid"]
    area = _rect_area(ctx, "NEW1", 4500, 1200, 5300, 1700, sensor_req="rgb", priority=1.0, payload_req_id="default_rgb")
    r1 = dyn.replan(plan, dyn.NewArea(area, t), {}, ctx, cache, cfg)
    assert r1.changed_uavs == ["B1"]
    b1 = r1.plan.plans["B1"]
    st = dyn._status(b1, t)
    pend = [str(r1.plan.strips[sp.strip_id].cell_id) for sp in st.pending]
    assert pend and not pend[0].startswith("NEW1") and any(c.startswith("NEW1") for c in pend)   # 新區排在尾端
    # 降低優先度 → 不中斷
    r_low = dyn.replan(r1.plan, dyn.PriorityChange("NEW1", 0.5, t), {}, ctx, cache, cfg)
    assert r_low.changed_uavs == [] and any("no interruption" in n for n in r_low.notes)
    # 提高 → 未開始之胞搶占：新區條帶排到其餘待掃之前
    r2 = dyn.replan(r1.plan, dyn.PriorityChange("NEW1", 5.0, t), {}, ctx, cache, cfg)
    assert r2.changed_uavs == ["B1"], r2.notes
    _check_incremental(r1.plan, r2)
    b1b = r2.plan.plans["B1"]
    st2 = dyn._status(b1b, t)
    pend2 = [str(r2.plan.strips[sp.strip_id].cell_id) for sp in st2.pending]
    assert pend2[0].startswith("NEW1")
    assert sorted(s for s, _ in b1b.strips) == sorted(s for s, _ in b1.strips)       # 條帶集合不變（只重排）
    assert all(c.priority == 5.0 for c in r2.plan.cells if c.area_id == "NEW1")


# ---------------------------------------------------------------------------
# WEATHER_ABORT
# ---------------------------------------------------------------------------
def _margin_s(res, u, ctx, cache):
    p = res.plan.plans[u]
    w0 = res.diff[u][0]
    model = ctx.models[u]
    e_alt = cache.budget.point_wh(p.cls, np.array([w0.x_m, w0.y_m]), w0.z_m, p.landing_site)
    return (w0.energy_remaining_wh - model.reserve_wh() - e_alt) / model.p_cruise_w() * 3600.0


def test_weather_abort_landing_order(env):
    plan, ctx, cache, cfg, t = env["plan"], env["ctx"], env["cache"], env["cfg"], env["t_mid"]
    res = dyn.replan(plan, dyn.WeatherAbort("ALL", None, "storm", t), {}, ctx, cache, cfg)
    assert set(res.changed_uavs) == {"A1", "B1"}
    _check_incremental(plan, res)
    note = next(n for n in res.notes if "landing order" in n)
    order = [u for u in ("A1", "B1") if u in note]
    order = sorted(order, key=lambda u: note.index(u))
    m = {u: _margin_s(res, u, ctx, cache) for u in ("A1", "B1")}
    assert order == sorted(m, key=m.get)                              # 能量時間餘裕升序（最緊者先）
    for u in ("A1", "B1"):
        p = res.plan.plans[u]
        assert p.waypoints[-1].phase is Phase.LAND
        assert not any(w.phase is Phase.OPERATE and w.t_s > t + dyn._FINISH_STRIP_S + 200.0 for w in res.diff[u])
        assert p.landing_site in ctx.sites_enu and not any(w.startswith("landing_over_limit") for w in p.warnings)
    # 同站降落間隔 ≥ t_sep,LDG（後續機以盤旋等待）
    first, second = order
    pa, pb = res.plan.plans[first], res.plan.plans[second]
    if pa.landing_site == pb.landing_site:
        assert pb.t_arr_ptc_s - 30.0 >= pa.t_land_s + cfg.t_sep_ldg_s - 1e-6
        assert pb.orbits_inserted > 0 or pb.t_arr_ptc_s - 30.0 >= pa.t_land_s + cfg.t_sep_ldg_s
    unc = {s for s, _ in res.plan.uncovered}
    assert unc                                                        # 未覆蓋 → 待辦


def test_weather_abort_scope_and_region(env):
    plan, ctx, cache, cfg, t = env["plan"], env["ctx"], env["cache"], env["cfg"], env["t_mid"]
    res = dyn.replan(plan, dyn.WeatherAbort("B", None, "gust", t), {}, ctx, cache, cfg)
    assert res.changed_uavs == ["B1"] and res.plan.plans["A1"] is plan.plans["A1"]
    _check_incremental(plan, res)
    far = tuple(_latlon(ctx, x, y) for x, y in ((30000, 30000), (31000, 30000), (31000, 31000), (30000, 31000)))
    res2 = dyn.replan(plan, dyn.WeatherAbort("region", far, "cell", t), {}, ctx, cache, cfg)
    assert res2.changed_uavs == []


def test_weather_abort_all_over_limit(env):
    plan, ctx, cache, cfg, t = env["plan"], env["ctx"], env["cache"], env["cfg"], env["t_mid"]
    gusty = WindField([WindLayer(0.0, 3000.0, 5.0, 270.0, 12.0)], ctx.frame, ctx.z_ground_home_m,
                      version=ctx.wind.version + 1)                   # 陣風 12 > 站點限 8（未驗證）
    ctx.wind = gusty
    res = dyn.replan(plan, dyn.WeatherAbort("ALL", None, "gust", t), {}, ctx, cache, cfg)
    assert set(res.changed_uavs) == {"A1", "B1"}
    for u in ("A1", "B1"):
        p = res.plan.plans[u]
        assert any(w.startswith("landing_over_limit(") for w in p.warnings), p.warnings
        assert p.waypoints[-1].phase is Phase.LAND                    # 仍降落（優於不落）
    assert any(n.startswith("landing_over_limit(") for n in res.notes)
    _check_incremental(plan, res)


# ---------------------------------------------------------------------------
# placeholder / dispatcher
# ---------------------------------------------------------------------------
def test_placeholder_when_over_budget(env, monkeypatch):
    plan, ctx, cache, cfg, t = env["plan"], env["ctx"], env["cache"], env["cfg"], env["t_mid"]
    monkeypatch.setattr(dyn, "REPLAN_BUDGET_MS", 0.0)
    res = dyn.replan(plan, dyn.UAVExit("A1", t, "x"), {}, ctx, cache, cfg)
    assert res.placeholder_used is True and res.timing_ms["total"] > dyn.REPLAN_BUDGET_MS
    assert any(n.startswith("placeholder_dispatched") for n in res.notes)
    for u, wps in res.placeholders.items():
        assert u in res.changed_uavs and wps and wps[-1].phase is Phase.LAND
    ph = dyn.placeholder_plan(plan, ["A1", "B1"], ctx, cache, t_now_s=t)
    for u in ("A1", "B1"):
        wps = ph[u]
        assert wps[-1].phase is Phase.LAND and not any(w.phase is Phase.OPERATE for w in wps)
        st = dyn._status(plan.plans[u], t)
        if st.current is not None:
            assert wps[0].t_s >= st.current.t_end - 1e-6             # 續飛現行條帶至端點後返航


def test_replan_dispatch_and_unknown_event(env):
    plan, ctx, cache, cfg, t = env["plan"], env["ctx"], env["cache"], env["cfg"], env["t_mid"]
    res = dyn.replan(plan, dyn.PriorityChange("Z1", 2.0, t), {}, ctx, cache, cfg)
    assert isinstance(res, dyn.ReplanResult) and res.plan.diff_from == "PriorityChange"
    assert res.changed_uavs == [] and res.diff == {}
    with pytest.raises(TypeError):
        dyn.replan(plan, object(), {}, ctx, cache, cfg)  # type: ignore[arg-type]
    res2 = dyn.replan(plan, dyn.UAVExit("ZZ", t, "x"), {}, ctx, cache, cfg)
    assert res2.changed_uavs == [] and any("not in plan" in n for n in res2.notes)


# ---------------------------------------------------------------------------
# R-09（blocker）：尚未離場之機（t_now < 起飛走廊出口時刻）
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("t_now", [10.0, 30.0, 59.0])
def test_replan_before_takeoff_keeps_corridor_and_time_order(base, t_now):
    """規劃器刻意錯開起飛（mini：B1 t=0、A1 t=60；hills 最後一架 2914 s），所以重規劃視窗有很大一段時間裡
    仍有飛機在地面。舊碼的 anchor_now 以 t_now 建錨：_index_at 夾到 0 → 保留 TAKEOFF 航點的**未來**時戳，
    尾段卻自 t_now 起算，UAVPlan 的 t_s 因此倒退，deconflict._waypoint_arrays 直接丟 ValueError，
    replan() 以例外收場而不是回傳 ReplanResult。修正後錨在起飛走廊出口、沿用其原時刻。"""
    inputs, cfg, plan = base["inputs"], base["cfg"], base["plan"]
    assert plan.plans["A1"].t_takeoff_s > t_now                     # A1 尚未起飛
    for event in (dyn.WeatherAbort("ALL", None, "storm", t_now), dyn.EnergyUpdate("A1", t_now, 300.0)):
        ctx = _build(inputs, cfg)
        cache = dyn.cache_from_plan(plan, ctx)
        res = dyn.replan(plan, event, {}, ctx, cache, cfg)           # 不得丟例外
        for u, p in res.plan.plans.items():
            ts = [w.t_s for w in p.waypoints]
            assert all(b >= a - 1e-9 for a, b in zip(ts, ts[1:])), (u, type(event).__name__)
            assert p.t_land_s >= p.t_takeoff_s
            if u == "A1":
                assert ts[0] == pytest.approx(plan.plans["A1"].waypoints[0].t_s)   # 起飛走廊原時刻保留
                j_pre = dyn._takeoff_prefix_end(plan.plans["A1"])
                assert len(p.waypoints) > j_pre                                     # 整段走廊都留著
                assert all(a is b for a, b in zip(p.waypoints[:j_pre + 1],
                                                  plan.plans["A1"].waypoints[:j_pre + 1]))
        assert res.t_sync_s == pytest.approx(t_now)
