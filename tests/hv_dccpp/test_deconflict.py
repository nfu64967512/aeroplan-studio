"""tests/test_deconflict.py — deconflict（契約 §13；規格 §13、D18、§17.2 deconflict 清單）。

合成計畫（直接建 Waypoint）＋ mini example 之 MissionContext（dem_step 50 m）。
"""
from __future__ import annotations

import math
from dataclasses import replace

import numpy as np
import pytest
from shapely.geometry import box

from hv_dccpp import deconflict as D
from hv_dccpp.context import build_context
from hv_dccpp.deconflict import (
    Conflict,
    CorridorSchedule,
    check_separation,
    repair,
    spatially_exclusive,
    trajectory_samples,
)
from hv_dccpp.geometry import Cell
from hv_dccpp.mission import Mode, Phase, PlannerConfig, UAVPlan, Waypoint
from hv_dccpp.platform import PLATFORM_HERO, PLATFORM_PIONEER, EnergyModel
from hv_dccpp.scenario import build_mini_example

CFG = PlannerConfig()


# ---------------------------------------------------------------------------
# 合成計畫工具
# ---------------------------------------------------------------------------

def wp(x: float, y: float, z: float, t: float, phase: Phase = Phase.OPERATE, *, track_deg: float = 90.0,
       energy: float = 1000.0, mode: Mode | None = None, airspeed: float = 21.5, cell_id: str | None = None) -> Waypoint:
    if mode is None:
        mode = Mode.HOVER if phase in (Phase.TAKEOFF, Phase.LAND) else (
            Mode.TRANSITION if phase in (Phase.TRANS_OUT, Phase.TRANS_IN) else Mode.CRUISE)
    return Waypoint(x, y, z, 0.0, 0.0, z + 100.0, z, t, mode, phase, phase is Phase.OPERATE,
                    None, cell_id, None, airspeed, 20.0, track_deg, track_deg, energy, "ALT1", True, "RTB")


def leg(p0: tuple[float, float], p1: tuple[float, float], z: float, t0: float, t1: float, n: int = 21,
        phase: Phase = Phase.OPERATE, energy0: float = 1000.0, energy1: float = 900.0, cell_id: str | None = None,
        include_start: bool = True) -> list[Waypoint]:
    """直線航段等時取樣（含 track_deg）。"""
    x0, y0 = p0
    x1, y1 = p1
    trk = math.degrees(math.atan2(x1 - x0, y1 - y0)) % 360.0
    out = []
    for i in range(0 if include_start else 1, n):
        f = i / (n - 1)
        out.append(wp(x0 + f * (x1 - x0), y0 + f * (y1 - y0), z, t0 + f * (t1 - t0), phase, track_deg=trk,
                      energy=energy0 + f * (energy1 - energy0), cell_id=cell_id))
    return out


def make_plan(uav_id: str, cls: str, wps: list[Waypoint], *, t_takeoff: float | None = None,
              t_arr_ptc: float | None = None, t_land: float | None = None, landing_site: str = "H",
              energy_margin: float = 200.0) -> UAVPlan:
    t0 = wps[0].t_s if t_takeoff is None else t_takeoff
    tl = wps[-1].t_s if t_land is None else t_land
    ta = tl - 110.0 if t_arr_ptc is None else t_arr_ptc
    return UAVPlan(uav_id, cls, wps, [], t0, ta, tl, landing_site, 300.0, energy_margin, 10000.0, 20.0, 0, [])


HOME2 = (2000.0, 1500.0)                                                        # mini 之 ALT1（ENU）


def strip_plan(uav_id: str, cls: str, y: float, z: float, t0: float, t1: float, east: bool = True,
               x0: float = 4000.0, x1: float = 7000.0, energy_margin: float = 200.0,
               home: tuple[float, float] = (0.0, 0.0), site: str = "H") -> UAVPlan:
    """起飛 → ENTRY → 單條帶（東西向）→ RTB → 落地 之簡化計畫（起降於 ``home``／``site``）。"""
    a, b = ((x0, y), (x1, y)) if east else ((x1, y), (x0, y))
    hx, hy = home
    wps = [wp(hx, hy, 0.0, t0 - 250.0, Phase.TAKEOFF), wp(hx, hy, 30.0, t0 - 220.0, Phase.TAKEOFF)]
    wps += leg(home, a, z, t0 - 220.0, t0, 11, Phase.ENTRY, 1000.0, 950.0, include_start=False)
    wps += leg(a, b, z, t0, t1, 31, Phase.OPERATE, 950.0, 850.0, cell_id="Z1", include_start=False)
    wps += leg(b, home, z, t1, t1 + 300.0, 11, Phase.RTB, 850.0, 750.0, include_start=False)
    wps += [wp(hx, hy, 0.0, t1 + 400.0, Phase.LAND, energy=700.0)]
    return make_plan(uav_id, cls, wps, t_takeoff=t0 - 250.0, t_arr_ptc=t1 + 300.0, t_land=t1 + 400.0,
                     landing_site=site, energy_margin=energy_margin)


def oper_only(uav_id: str, y: float, z: float, t0: float, t1: float, east: bool = True,
              phase: Phase = Phase.OPERATE) -> UAVPlan:
    a, b = ((4000.0, y), (7000.0, y)) if east else ((7000.0, y), (4000.0, y))
    return make_plan(uav_id, "A", leg(a, b, z, t0, t1, 31, phase, cell_id="Z1"))


@pytest.fixture(scope="module")
def mini_ctx():
    inputs, cfg = build_mini_example()
    return build_context(inputs, replace(cfg, dem_step_m=50.0), terrain_margin_m=800.0)


# ---------------------------------------------------------------------------
# trajectory_samples / sigma_t_since_sync
# ---------------------------------------------------------------------------

def test_trajectory_samples_grid_and_sigma_since_sync():
    wps = leg((0.0, 0.0), (2000.0, 0.0), 260.0, 100.0, 200.0, 3)
    plan = make_plan("A1", "A", wps)
    s = trajectory_samples(plan, 2.0, t_sync_s=0.0, k_t=CFG.k_t)
    assert s.shape == (51, 5)
    assert s[0, 3] == 100.0 and s[-1, 3] == 200.0
    np.testing.assert_allclose(np.diff(s[:, 3]), 2.0)
    np.testing.assert_allclose(s[:, 0], 20.0 * (s[:, 3] - 100.0))          # 線性內插精確
    np.testing.assert_allclose(s[:, 2], 260.0)
    np.testing.assert_allclose(s[:, 4], CFG.k_t * s[:, 3])                 # σ_t = k_t (t − t_sync)
    s2 = trajectory_samples(plan, 2.0, t_sync_s=150.0, k_t=CFG.k_t)
    np.testing.assert_allclose(s2[:, 4], CFG.k_t * np.maximum(0.0, s2[:, 3] - 150.0))
    assert s2[0, 4] == 0.0 and s2[-1, 4] == pytest.approx(2.5)
    # 非整數倍：末樣本 = 末航點
    s3 = trajectory_samples(make_plan("A1", "A", leg((0, 0), (100, 0), 1.0, 0.0, 7.0, 2)), 2.0)
    assert s3[-1, 3] == 7.0 and s3.shape[0] == 5
    # 單一航點 / 空計畫
    assert trajectory_samples(make_plan("A1", "A", [wp(1, 2, 3, 4)]), 2.0).shape == (1, 5)
    assert trajectory_samples(replace(make_plan("A1", "A", [wp(1, 2, 3, 4)]), waypoints=[]), 2.0).shape == (0, 5)


def test_trajectory_samples_duplicate_time_jump():
    wps = [wp(0, 0, 0, 0.0), wp(0, 0, 50, 10.0), wp(100, 0, 50, 10.0), wp(200, 0, 50, 20.0)]
    s = trajectory_samples(make_plan("A1", "A", wps), 5.0)
    assert s[2, 3] == 10.0 and s[2, 0] == pytest.approx(100.0)              # 時間重複 → 跳躍完成
    assert s[3, 0] == pytest.approx(150.0)


# ---------------------------------------------------------------------------
# CorridorSchedule
# ---------------------------------------------------------------------------

def test_corridor_reservation_semantics_and_next_free():
    sch = CorridorSchedule(CFG)
    a1 = strip_plan("A1", "A", 0.0, 260.0, 300.0, 1200.0)
    a1 = replace(a1, t_takeoff_s=0.0, t_arr_ptc_s=1586.7, t_land_s=1696.7)
    b1 = strip_plan("B1", "B", -400.0, 200.0, 300.0, 1000.0)
    b1 = replace(b1, t_takeoff_s=60.0, t_arr_ptc_s=1370.0, t_land_s=1470.0)
    assert sch.reserve_plan(a1, "H") == (True, True)
    (t0, t1), (l0, l1) = D.plan_windows(a1, CFG)
    assert (t0, t1) == (0.0, 60.0) and (l0, l1) == (pytest.approx(1556.7), pytest.approx(1786.7))
    to_ok, ld_ok = sch.reserve_plan(b1, "H")
    assert to_ok is True and ld_ok is False                                   # [1340,1560] 與 [1556.7,1786.7] 重疊 3.3 s
    cs = sch.conflicts()
    assert len(cs) == 1
    c = cs[0]
    assert c.kind == "corridor" and (c.uav_a, c.uav_b) == ("A1", "B1") and c.site_id == "H"
    assert c.res_kinds == ("landing", "landing")
    assert c.dt_req_s == pytest.approx(3.3, abs=1e-6) and c.t_s == pytest.approx(1556.7)
    # next_free：跨越既有預約
    assert sch.next_free("H", "takeoff", 30.0, 60.0) == pytest.approx(120.0)  # [30,90] 撞 [0,60]→60，[60,120] 撞 B1 [60,120]→120
    assert sch.next_free("H", "landing", 1500.0, 100.0) == pytest.approx(1786.7)
    assert sch.next_free("H", "takeoff", 60.0, 0.0) == pytest.approx(60.0)   # 相接不算重疊
    assert sch.next_free("ALT1", "landing", 10.0, 50.0) == 10.0
    # release / is_free
    assert sch.release("B1") == 2 and sch.conflicts() == []
    assert sch.is_free("H", 1340.0, 1560.0) is False and sch.is_free("H", 1340.0, 1556.7) is True
    with pytest.raises(ValueError):
        sch.reserve("H", "hover", 0.0, 1.0, "X")


def test_corridor_takeoff_landing_mutual_exclusion_and_ghost():
    sch = CorridorSchedule(CFG)
    assert sch.reserve("H", "landing", 100.0, 200.0, "A1")
    assert not sch.reserve("H", "takeoff", 150.0, 210.0, "B1")              # 起降共用走廊
    c = sch.conflicts()[0]
    assert c.res_kinds == ("landing", "takeoff") and c.dt_req_s == pytest.approx(50.0)

    class Ghost:
        uav_id, landing_site, eta_s, sigma_t = "G1", "H", 1000.0, 5.0
        samples = np.zeros((0, 4))
    assert sch.reserve_ghost(Ghost())
    r = sch.reservations_of("G1")[0]
    assert (r.t_start_s, r.t_end_s) == (940.0, 1180.0)


# ---------------------------------------------------------------------------
# check_separation
# ---------------------------------------------------------------------------

def test_parallel_strips_60m_apart_same_time_conflict_and_symmetric():
    a = strip_plan("A1", "A", 0.0, 260.0, 300.0, 450.0, east=True)
    b = strip_plan("A2", "A", 60.0, 260.0, 300.0, 450.0, east=False, home=HOME2, site="ALT1")   # 對飛，側距 60 m
    cs = check_separation({"A1": a, "A2": b}, CFG)
    assert len(cs) == 1
    c = cs[0]
    assert c.kind == "4d" and (c.uav_a, c.uav_b) == ("A1", "A2")
    assert 300.0 <= c.t_s <= 450.0
    assert c.d_h_m == pytest.approx(60.0, abs=1.0) and c.d_v_m == pytest.approx(0.0)
    assert c.dt_req_s > 0 and c.delay_a_s > 0 and c.delay_b_s > 0
    # 對稱：字典順序對調 → 同一筆
    cs2 = check_separation({"A2": b, "A1": a}, CFG)
    assert [(x.uav_a, x.uav_b, x.idx_a, x.idx_b, round(x.d_h_m, 6), round(x.t_s, 6)) for x in cs] == \
           [(x.uav_a, x.uav_b, x.idx_a, x.idx_b, round(x.d_h_m, 6), round(x.t_s, 6)) for x in cs2]
    # idx 對應 trajectory_samples
    sa = trajectory_samples(a, CFG.dt_4d_s, k_t=CFG.k_t)
    sb = trajectory_samples(b, CFG.dt_4d_s, k_t=CFG.k_t)
    assert sa[c.idx_a, 3] == pytest.approx(c.t_s)
    assert math.hypot(*(sa[c.idx_a, :2] - sb[c.idx_b, :2])) == pytest.approx(c.d_h_m)
    assert sb[c.idx_b, 3] - sa[c.idx_a, 3] == pytest.approx(c.dt_ab_s)


def test_altitude_separated_no_conflict_layering_A_above_B():
    a = strip_plan("A1", "A", 0.0, 260.0, 300.0, 450.0, east=True)          # 160 m AGL
    b = strip_plan("B1", "B", 60.0, 200.0, 300.0, 450.0, east=False, home=HOME2, site="ALT1")   # 100 m AGL → Δz 60
    assert check_separation({"A1": a, "B1": b}, CFG) == []
    b2 = strip_plan("B1", "B", 60.0, 215.0, 300.0, 450.0, east=False, home=HOME2, site="ALT1")  # Δz = 45 → 衝突
    cs = check_separation({"A1": a, "B1": b2}, CFG)
    assert len(cs) == 1 and cs[0].d_v_m == pytest.approx(45.0)


def test_codirectional_lateral_threshold_d_sep_lat():
    a = oper_only("A1", 0.0, 260.0, 300.0, 450.0)
    b = oper_only("A2", 80.0, 260.0, 300.0, 450.0)                             # 同向平行 80 m ≥ d_sep_lat 60
    assert check_separation({"A1": a, "A2": b}, CFG) == []
    b3 = oper_only("A2", 50.0, 260.0, 300.0, 450.0)                            # 50 m < 60 → 衝突
    cs = check_separation({"A1": a, "A2": b3}, CFG)
    assert len(cs) == 1 and cs[0].d_h_m == pytest.approx(50.0)
    # 非作業段（ENTRY）同向 80 m → 一般門檻 d_sep=100 → 衝突
    assert check_separation({"A1": a, "A2": oper_only("A2", 80.0, 260.0, 300.0, 450.0, phase=Phase.ENTRY)}, CFG)
    # 對飛 80 m → 衝突
    assert check_separation({"A1": a, "A2": oper_only("A2", 80.0, 260.0, 300.0, 450.0, east=False)}, CFG)


def test_time_separated_and_sigma_growth():
    # 同一條帶、相隔 300 s；起飛早（t 小 → σ 小）→ 無衝突
    a = oper_only("A1", 0.0, 260.0, 300.0, 450.0)
    b = oper_only("A2", 0.0, 260.0, 600.0, 750.0)
    assert check_separation({"A1": a, "A2": b}, CFG) == []
    # t ≈ 4000 s：σ 已飽和於 SIGMA_T_MAX_S → 視窗 = Δt_4d + 2σ_max
    win = CFG.dt_4d_s + 2.0 * D.SIGMA_T_MAX_S
    a2 = oper_only("A1", 0.0, 260.0, 4000.0, 4150.0)
    near = oper_only("A2", 0.0, 260.0, 4000.0 + 0.8 * win, 4150.0 + 0.8 * win)
    far = oper_only("A2", 0.0, 260.0, 4000.0 + 1.6 * win, 4150.0 + 1.6 * win)
    cs2 = check_separation({"A1": a2, "A2": near}, CFG)
    assert cs2 and all(c.dt_req_s > 0 for c in cs2) and max(c.t_s for c in cs2) >= 4000.0
    assert check_separation({"A1": a2, "A2": far}, CFG) == []
    # 以 t_sync = 4000 重算 σ → σ ≤ 22 s → 無衝突
    assert check_separation({"A1": a2, "A2": near}, CFG, t_sync_s=4000.0) == []


def test_sigma_t_cap_and_override(monkeypatch):
    """σ_t = min(k_t·(t − t_sync), SIGMA_T_MAX_S)（契約偏離；規格 §13.3 原式無上限）。"""
    plan = oper_only("A1", 0.0, 260.0, 0.0, 4000.0)
    s = trajectory_samples(plan, 2.0, k_t=CFG.k_t)                              # 預設上限
    t = s[:, 3]
    np.testing.assert_allclose(s[:, 4], np.minimum(CFG.k_t * t, D.SIGMA_T_MAX_S))
    assert s[-1, 4] == pytest.approx(D.SIGMA_T_MAX_S)                           # 已飽和
    assert D.SIGMA_T_MAX_S <= 60.0                                              # 審查建議 30–60 s
    # sigma_max_s=inf → 回到規格原式
    s_inf = trajectory_samples(plan, 2.0, k_t=CFG.k_t, sigma_max_s=math.inf)
    np.testing.assert_allclose(s_inf[:, 4], CFG.k_t * t)
    # check_separation 走 cfg（若 PlannerConfig 提供 sigma_t_max_s 則優先），否則模組常數
    assert D.sigma_t_max_of(CFG) == D.SIGMA_T_MAX_S
    a2 = oper_only("A1", 0.0, 260.0, 4000.0, 4150.0)
    b2 = oper_only("A2", 0.0, 260.0, 4300.0, 4450.0)                            # 相隔 300 s
    assert check_separation({"A1": a2, "A2": b2}, CFG) == []                    # 上限 45 → 視窗 92 s
    monkeypatch.setattr(D, "SIGMA_T_MAX_S", math.inf)
    assert check_separation({"A1": a2, "A2": b2}, CFG)                          # 無上限 → 視窗 402 s → 衝突


def test_skip_pairs_only_exempts_operate_samples():
    """``skip_pairs``（:func:`spatially_exclusive`）只涵蓋作業段；轉場段仍須檢查（否則 RTB 對撞會被漏掉）。"""
    a = oper_only("A1", 0.0, 260.0, 300.0, 450.0)                               # 純 OPERATE
    b = oper_only("A2", 50.0, 260.0, 300.0, 450.0)
    assert check_separation({"A1": a, "A2": b}, CFG)
    assert check_separation({"A1": a, "A2": b}, CFG, skip_pairs={("A1", "A2")}) == []
    # 同樣幾何但相位為 RTB（轉場）→ skip_pairs 不再豁免
    a_r = oper_only("A1", 0.0, 260.0, 300.0, 450.0, phase=Phase.RTB)
    b_r = oper_only("A2", 50.0, 260.0, 300.0, 450.0, phase=Phase.RTB)
    assert check_separation({"A1": a_r, "A2": b_r}, CFG, skip_pairs={("A1", "A2")})
    # 一方作業、一方轉場也不豁免
    assert check_separation({"A1": a, "A2": b_r}, CFG, skip_pairs={("A1", "A2")})


def test_check_separation_only_uavs_filter():
    a = oper_only("A1", 0.0, 260.0, 300.0, 450.0)
    b = oper_only("A2", 50.0, 260.0, 300.0, 450.0)
    c = oper_only("A3", 100.0, 260.0, 300.0, 450.0)
    plans = {"A1": a, "A2": b, "A3": c}
    full = check_separation(plans, CFG)
    assert len(full) >= 2
    sub = check_separation(plans, CFG, only_uavs={"A1"})
    assert sub and all(x.involves("A1") for x in sub)
    assert len(sub) < len(full)


def test_ghost_track_as_extra_track():
    a = strip_plan("A1", "A", 0.0, 260.0, 300.0, 450.0)

    class Ghost:
        uav_id, landing_site, eta_s, sigma_t = "G1", "H", 900.0, 5.0
        t = np.arange(300.0, 451.0, 2.0)
        samples = np.column_stack([np.full(t.size, 5500.0), 1500.0 - 20.0 * (t - 300.0), np.full(t.size, 260.0), t])
    cs = check_separation({"A1": a}, CFG, ghosts=[Ghost()])                  # 幽靈於 t=375 穿越 (5500, 0)
    assert len(cs) == 1 and cs[0].kind == "ghost" and (cs[0].uav_a, cs[0].uav_b) == ("A1", "G1")
    assert cs[0].d_h_m < 30.0 and abs(cs[0].t_s - 375.0) <= 2.0 and cs[0].delay_a_s > 0   # 2 s 取樣格
    assert check_separation({"A1": a}, CFG, ghosts=[Ghost()], skip_pairs={("G1", "A1")}) == []
    assert check_separation({}, CFG, ghosts=[Ghost(), Ghost()]) == []        # 幽靈之間不檢查


# ---------------------------------------------------------------------------
# spatially_exclusive
# ---------------------------------------------------------------------------

def _cell(cid: str, poly, cls: str, alpha: float, d_turn: float) -> Cell:
    return Cell(cid, "Z", poly, "any", "default", 1.0, None, poly.area, 0.0, alpha, 0.0,
                {cls: np.array(poly.bounds[:2])}, {cls: 160.0}, {cls: True}, {cls: ""}, {cls: alpha}, {cls: []},
                {cls: []}, {cls: 1.0}, {cls: 0}, {cls: 100.0}, {cls: d_turn})


class Team:
    def __init__(self, team_id, cls, uav_ids, cell_ids):
        self.team_id, self.cls, self.uav_ids, self.cell_ids = team_id, cls, list(uav_ids), list(cell_ids)


def test_spatial_exclusion_skips_4d():
    alpha = math.pi / 2
    cells = {
        "C1": _cell("C1", box(4000, -600, 7000, 600), "A", alpha, 535.0),
        "C2": _cell("C2", box(4000, 800, 7000, 2000), "A", alpha, 535.0),        # 側向間隔 200 m ≥ d_sep
        "C3": _cell("C3", box(7300, -600, 9000, 600), "A", alpha, 535.0),        # 沿 t̂ 相距 300 < 2·535 → 迴轉帶相交
        "C4": _cell("C4", box(4000, 800, 7000, 2000), "B", alpha, 336.0),
    }
    teams = [Team("T1", "A", ["A1"], ["C1"]), Team("T2", "A", ["A2", "A3"], ["C2"]),
             Team("T3", "A", ["A4"], ["C3"]), Team("T4", "B", ["B1"], ["C4"]), Team("T5", "A", ["A5"], [])]
    ex = spatially_exclusive(teams, cells, CFG)
    assert ("A1", "A2") in ex and ("A1", "A3") in ex
    assert ("A1", "A4") not in ex                                               # 迴轉帶相交
    assert ("A2", "A4") in ex and ("A3", "A4") in ex
    assert not any("B1" in p for p in ex) and not any("A5" in p for p in ex)   # 異構型／無胞團隊不判定
    # 側向外延 R_eff − s/2 使 C1/C2（間隔 200 m、d_sep 100）不再互斥：lateral 200 → 距離 −200
    ex2 = spatially_exclusive(teams, cells, CFG, r_eff_m={"A": 200.0})
    assert ("A1", "A2") not in ex2
    # 互斥對之衝突不出現於 conflicts
    a = strip_plan("A1", "A", 0.0, 260.0, 300.0, 450.0, east=True)
    b = strip_plan("A2", "A", 60.0, 260.0, 300.0, 450.0, east=False, home=HOME2, site="ALT1")
    assert check_separation({"A1": a, "A2": b}, CFG)
    assert check_separation({"A1": a, "A2": b}, CFG, skip_pairs=ex) == []


# ---------------------------------------------------------------------------
# repair
# ---------------------------------------------------------------------------

def test_repair_prefers_takeoff_delay_corridor_W9(mini_ctx):
    """契約 §W.9：B1 [1340,1560] 與 A1 [1556.7,1786.7] 重疊 3.3 s → 換序 B1 t0=0、A1 t0=60 → A1 落地 1756.7。"""
    ctx = mini_ctx
    a1 = strip_plan("A1", "A", 0.0, 260.0, 300.0, 1296.7)                   # 觸地 1696.7
    a1 = replace(a1, t_takeoff_s=0.0, t_arr_ptc_s=1586.7)
    b1 = strip_plan("B1", "B", -400.0, 200.0, 360.0, 1070.0)                # 航點依 t0 = 60 s；觸地 1470
    b1 = replace(b1, t_takeoff_s=60.0, t_arr_ptc_s=1370.0)
    plans = {"A1": a1, "B1": b1}
    sch = CorridorSchedule(CFG)
    sch.reserve_plan(a1, "H")
    sch.reserve_plan(b1, "H")
    assert len(sch.conflicts()) == 1
    cs = check_separation(plans, CFG)
    assert cs == []
    new, residual = repair(plans, cs, ctx, ctx.models, sch, CFG)
    assert residual == []
    assert new["B1"].t_takeoff_s == pytest.approx(0.0) and new["A1"].t_takeoff_s == pytest.approx(60.0)
    assert new["A1"].t_land_s == pytest.approx(1756.7) and new["B1"].t_land_s == pytest.approx(1410.0)
    assert new["A1"].orbits_inserted == 0 and new["B1"].orbits_inserted == 0     # 零能量方案
    assert new["A1"].energy_used_wh == a1.energy_used_wh and new["B1"].energy_margin_wh == b1.energy_margin_wh
    assert sch.conflicts() == []
    r = {x.kind: (x.t_start_s, x.t_end_s) for x in sch.reservations_of("A1")}
    assert r["takeoff"] == (60.0, 120.0) and r["landing"] == (pytest.approx(1616.7), pytest.approx(1846.7))
    # 原計畫不變、航點全部平移 60 s、z 不變
    assert a1.t_takeoff_s == 0.0 and plans["A1"] is a1
    np.testing.assert_allclose([w.t_s for w in new["A1"].waypoints], [w.t_s + 60.0 for w in a1.waypoints])
    np.testing.assert_array_equal([w.z_m for w in new["A1"].waypoints], [w.z_m for w in a1.waypoints])


def _crossing_plans(t_cross: float, margin: float = 200.0) -> dict[str, UAVPlan]:
    """A1 東飛、A2 北飛，於 (0,0)、t_cross 交會（同層）。"""
    def build(uid, p0, p1, t_land):
        wps = [wp(*p0, 0.0, t_cross - 500.0, Phase.TAKEOFF), wp(*p0, 30.0, t_cross - 470.0, Phase.TAKEOFF)]
        wps += leg(p0, p0, 300.0, t_cross - 470.0, t_cross - 300.0, 2, Phase.ENTRY, include_start=False)
        wps += leg(p0, p1, 300.0, t_cross - 300.0, t_cross + 300.0, 61, Phase.OPERATE, include_start=False)
        wps += leg(p1, p1, 300.0, t_cross + 300.0, t_land - 100.0, 2, Phase.RTB, include_start=False)
        wps += [wp(*p1, 0.0, t_land, Phase.LAND)]
        return make_plan(uid, "A", wps, t_takeoff=t_cross - 500.0, t_arr_ptc=t_land - 100.0, t_land=t_land,
                         energy_margin=margin)
    a = build("A1", (-6000.0, 0.0), (6000.0, 0.0), t_cross + 1000.0)
    b = build("A2", (0.0, -6000.0), (0.0, 6000.0), t_cross + 2000.0)
    b = replace(b, t_takeoff_s=b.t_takeoff_s + 60.0)
    return {"A1": a, "A2": b}


def test_detect_repair_one_shot_n_laps(mini_ctx):
    ctx = mini_ctx
    t_cross = 2000.0
    plans = _crossing_plans(t_cross)
    models = {"A1": ctx.models["A1"], "A2": EnergyModel(PLATFORM_PIONEER, 24.6, 1350.0, 1500.0, 1500.0)}
    cs = check_separation(plans, CFG)
    assert len(cs) == 1
    c = cs[0]
    assert c.d_h_m < 5.0 and abs(c.dt_ab_s) < 3.0
    sig = min(CFG.k_t * t_cross, D.SIGMA_T_MAX_S)
    assert c.dt_req_s == pytest.approx(CFG.dt_4d_s + 2 * sig - abs(c.dt_ab_s), abs=2.5)
    sch = CorridorSchedule(CFG)
    t_now = t_cross - 100.0                                                     # 兩機皆已起飛 → 只能盤旋
    new, residual = repair(plans, cs, ctx, models, sch, CFG, t_now_s=t_now)
    assert residual == []
    delayed = [u for u in new if new[u].orbits_inserted > 0]
    assert len(delayed) == 1
    u = delayed[0]
    p_new, p_old = new[u], plans[u]
    t_lap, r_orb = D.orbit_lap_time_s(ctx.specs["A"], ctx.wind, 300.0)
    assert t_lap == pytest.approx(60.9, abs=0.1) and r_orb == 200.0
    need = c.delay_a_s if u == "A1" else c.delay_b_s
    n_exp = math.ceil(need / t_lap)
    assert n_exp >= 2 and p_new.orbits_inserted == n_exp                        # 一次算足 n 圈
    # 插入位置：最近前一轉移（ENTRY 段起點）；其後航點時間 +n·T_lap、能量 −e_orbit、z 不變
    k = next(i for i, w in enumerate(p_new.waypoints) if w.phase is Phase.ORBIT)
    assert p_old.waypoints[k].phase is Phase.ENTRY and p_new.waypoints[k].arc is not None
    assert p_new.waypoints[k].arc[2] == 200.0 and p_new.waypoints[k].t_s == p_old.waypoints[k].t_s
    e_orbit = models[u].segment_wh("CRUISE", n_exp * t_lap)
    assert e_orbit == pytest.approx(n_exp * 511.36 * t_lap * 1.2075 / 3600.0, rel=1e-3)
    for w_old, w_new in zip(p_old.waypoints[k:], p_new.waypoints[k + 1:]):
        assert w_new.t_s == pytest.approx(w_old.t_s + n_exp * t_lap)
        assert w_new.energy_remaining_wh == pytest.approx(w_old.energy_remaining_wh - e_orbit)
        assert w_new.z_m == w_old.z_m and w_new.x_m == w_old.x_m
    assert p_new.energy_margin_wh == pytest.approx(p_old.energy_margin_wh - e_orbit)
    assert p_new.t_land_s == pytest.approx(p_old.t_land_s + n_exp * t_lap)
    assert p_new.t_takeoff_s == p_old.t_takeoff_s                               # 已起飛：不動起飛
    assert D.default_energy_ok(p_new, models[u])
    # 未被修改之機位元相同
    other = "A2" if u == "A1" else "A1"
    assert new[other] is plans[other]
    # 修復後無 4D 衝突
    assert check_separation(new, CFG) == []


def test_repair_on_ground_uses_takeoff_delay_not_orbit(mini_ctx):
    ctx = mini_ctx
    plans = _crossing_plans(2000.0)
    models = {"A1": ctx.models["A1"], "A2": EnergyModel(PLATFORM_PIONEER, 24.6, 1350.0, 1500.0, 1500.0)}
    cs = check_separation(plans, CFG)
    sch = CorridorSchedule(CFG)
    new, residual = repair(plans, cs, ctx, models, sch, CFG)                    # t_now = −inf：尚未起飛
    assert residual == []
    assert all(p.orbits_inserted == 0 for p in new.values())
    changed = [u for u in new if new[u] is not plans[u]]
    assert changed
    for u in changed:
        assert new[u].energy_margin_wh == plans[u].energy_margin_wh             # 零能量
        assert new[u].t_takeoff_s != plans[u].t_takeoff_s
    assert check_separation(new, CFG) == [] and sch.conflicts() == []


def test_repair_energy_fail_leaves_residual_and_hook(mini_ctx):
    ctx = mini_ctx
    plans = _crossing_plans(2000.0, margin=5.0)                                 # 5 Wh 不足一圈
    models = {"A1": ctx.models["A1"], "A2": EnergyModel(PLATFORM_PIONEER, 24.6, 1350.0, 1500.0, 1500.0)}
    cs = check_separation(plans, CFG)
    sch = CorridorSchedule(CFG)
    new, residual = repair(plans, cs, ctx, models, sch, CFG, t_now_s=1950.0)
    assert residual and residual[0].kind == "4d"
    assert new["A1"] is plans["A1"] and new["A2"] is plans["A2"]
    # (iii) hook：可回傳新計畫（此處以「全段延後 400 s」模擬 IDP 重排）
    calls = []

    def hook(uid, plan, conflict):
        calls.append((uid, conflict.kind))
        return D.shift_plan(plan, 400.0, from_index=2)
    new2, residual2 = repair(plans, cs, ctx, models, sch.copy(), CFG, t_now_s=1950.0, resequence_hook=hook)
    assert calls and residual2 == []
    assert new2["A1"].orbits_inserted == 0 and new2["A2"].orbits_inserted == 0
    # 能量重驗也可注入
    seen = []

    def echeck(plan, model):
        seen.append(plan.uav_id)
        return True
    new3, residual3 = repair(plans, cs, ctx, models, CorridorSchedule(CFG), CFG, t_now_s=1950.0, energy_check=echeck)
    assert seen and residual3 == [] and any(p.orbits_inserted > 0 for p in new3.values())


def test_repair_no_vertical_offset_D18(mini_ctx):
    ctx = mini_ctx
    a = strip_plan("A1", "A", 0.0, 260.0, 300.0, 450.0, east=True)
    b = strip_plan("A2", "A", 60.0, 260.0, 300.0, 450.0, east=False, home=HOME2, site="ALT1")
    plans = {"A1": a, "A2": b}
    models = {"A1": ctx.models["A1"], "A2": EnergyModel(PLATFORM_PIONEER, 24.6, 1350.0, 1500.0, 1500.0)}
    cs = check_separation(plans, CFG)
    assert cs
    for t_now in (-math.inf, 200.0):
        sch = CorridorSchedule(CFG)
        new, residual = repair(plans, cs, ctx, models, sch, CFG, t_now_s=t_now, takeoff_site={"A2": "ALT1"})
        assert residual == []
        if t_now > 0:
            assert any(p.orbits_inserted >= 1 for p in new.values())          # 已起飛 → 盤旋
        else:
            assert all(p.orbits_inserted == 0 for p in new.values())          # 地面 → 起飛延遲（零能量）
        for u in plans:
            z_old = [w.z_m for w in plans[u].waypoints]
            z_new = [w.z_m for w in new[u].waypoints if w.phase is not Phase.ORBIT]
            assert z_new == z_old
            assert D.default_energy_ok(new[u], models[u])
        assert check_separation(new, CFG) == [] and sch.conflicts() == []


def test_landing_priority_over_takeoff_and_hold_point(mini_ctx):
    """降落 vs 起飛走廊衝突：延遲起飛機；已起飛則降落機於保持點盤旋（距站 ≥ 2 km）。"""
    ctx = mini_ctx
    lander = strip_plan("A1", "A", 0.0, 260.0, 300.0, 400.0)                   # 落地 800、抵 P_tc 700 → [670, 890]
    taker = strip_plan("A2", "A", 0.0, 260.0, 1500.0, 1600.0)
    taker = replace(taker, t_takeoff_s=700.0)                                   # 起飛 [700, 760] 撞降落
    plans = {"A1": lander, "A2": taker}
    models = {"A1": ctx.models["A1"], "A2": EnergyModel(PLATFORM_PIONEER, 24.6, 1350.0, 1500.0, 1500.0)}
    sch = CorridorSchedule(CFG)
    new, residual = repair(plans, [], ctx, models, sch, CFG)
    assert residual == [] and sch.conflicts() == []
    # 降落優先：降落機不被延遲、不被迫盤旋（工期壓縮可能把它「提前」，但絕不延後）
    assert new["A1"].t_land_s <= lander.t_land_s + 1e-6 and new["A1"].orbits_inserted == 0
    assert new["A1"].energy_margin_wh == lander.energy_margin_wh
    # 讓路的是起飛機：其起飛窗不再與降落機之降落窗重疊
    (t0, t1), _ = D.plan_windows(new["A2"], CFG)
    _, (l0, l1) = D.plan_windows(new["A1"], CFG)
    assert not (t0 < l1 - 1e-9 and l0 < t1 - 1e-9)
    assert new["A2"].orbits_inserted == 0
    # 兩機皆已起飛（t_now=750）：起飛機不可延 → 降落機於保持點盤旋
    sch2 = CorridorSchedule(CFG)
    new2, residual2 = repair(plans, [], ctx, models, sch2, CFG, t_now_s=750.0)
    assert residual2 == [] and new2["A2"] is taker
    p = new2["A1"]
    assert p.orbits_inserted >= 1
    k = next(i for i, w in enumerate(p.waypoints) if w.phase is Phase.ORBIT)
    assert lander.waypoints[k].phase is Phase.RTB
    assert math.hypot(p.waypoints[k].x_m, p.waypoints[k].y_m) >= D.HOLD_DISTANCE_M
    assert sch2.conflicts() == []


def test_orbit_lap_time_numbers():
    t_a, r_a = D.orbit_lap_time_s(PLATFORM_PIONEER, np.array([5.0, 0.0]), 0.0)
    t_b, r_b = D.orbit_lap_time_s(PLATFORM_HERO, np.array([5.0, 0.0]), 0.0)
    assert (t_a, r_a) == (pytest.approx(60.9, abs=0.1), 200.0)                 # 規格 §11.6
    assert (t_b, r_b) == (pytest.approx(41.9, abs=0.1), 120.0)
    assert D.orbit_lap_time_s(PLATFORM_PIONEER, None, 0.0)[0] == pytest.approx(58.4, abs=0.1)


def test_shift_plan_and_insert_orbit_helpers():
    a = strip_plan("A1", "A", 0.0, 260.0, 300.0, 450.0)
    s = D.shift_plan(a, 10.0, from_index=3, energy_wh=2.0)
    assert s.t_takeoff_s == a.t_takeoff_s and s.t_land_s == a.t_land_s + 10.0
    assert s.waypoints[2] is a.waypoints[2] and s.waypoints[3].t_s == a.waypoints[3].t_s + 10.0
    assert s.waypoints[3].energy_remaining_wh == a.waypoints[3].energy_remaining_wh - 2.0
    model = EnergyModel(PLATFORM_PIONEER, 24.6, 1350.0, 1500.0, 1500.0)
    k = D.orbit_insert_index(a, 400.0)
    assert a.waypoints[k].phase is Phase.ENTRY and a.waypoints[k - 1].phase is Phase.TAKEOFF
    o = D.insert_delay_orbit(a, k, 2, 60.0, model, 200.0)
    assert len(o.waypoints) == len(a.waypoints) + 1 and o.orbits_inserted == 2
    h = o.waypoints[k]
    assert h.phase is Phase.ORBIT and h.mode is Mode.CRUISE and not h.sensor_on
    cx, cy, R, d = h.arc
    th = math.radians(h.track_deg)
    assert (cx, cy, R, d) == (pytest.approx(h.x_m + 200 * math.cos(th)), pytest.approx(h.y_m - 200 * math.sin(th)), 200.0, 1)
    assert o.waypoints[k + 1].t_s == a.waypoints[k].t_s + 120.0 and o.t_land_s == a.t_land_s + 120.0
    assert o.distance_m == pytest.approx(a.distance_m + 2 * 2 * math.pi * 200.0)
    assert D.insert_delay_orbit(a, k, 0, 60.0, model, 200.0) is a
    # 走廊保持點：RTB 段距站 ≥ 2 km 之最後航點
    kk = D.orbit_insert_index(a, a.t_arr_ptc_s, corridor=True, site_xy=np.zeros(2))
    assert a.waypoints[kk].phase is Phase.RTB and math.hypot(a.waypoints[kk].x_m, a.waypoints[kk].y_m) >= 2000.0


def test_check_separation_scale_8_uavs_same_layer():
    """8 機同層同區（最壞情形）、每機 ~1800 s、Δt=2 s → ~7000 樣本；含 σ 寬視窗仍 < 0.5 s（規格 §14）。"""
    import time
    plans = {}
    for i in range(8):
        y = -600 + i * 170.0
        t0 = 300 + 60 * i
        wps = [wp(0, 0, 0, t0 - 250, Phase.TAKEOFF), wp(0, 0, 30, t0 - 220, Phase.TAKEOFF)]
        wps += leg((0, 0), (4000, y), 260.0, t0 - 220, t0, 11, Phase.ENTRY, include_start=False)
        t = t0
        for k in range(6):
            east = (k % 2 == 0)
            a, b = ((4000, y + 20 * k), (7000, y + 20 * k)) if east else ((7000, y + 20 * k), (4000, y + 20 * k))
            wps += leg(a, b, 260.0, t, t + 150, 121, Phase.OPERATE, include_start=False)
            wps += leg(b, b, 260.0, t + 150, t + 190, 3, Phase.TRANSFER, include_start=False)
            t += 190
        wps += leg((wps[-1].x_m, wps[-1].y_m), (0, 0), 260.0, t, t + 300, 11, Phase.RTB, include_start=False)
        wps += [wp(0, 0, 0, t + 400, Phase.LAND)]
        plans[f"A{i + 1}"] = make_plan(f"A{i + 1}", "A", wps, t_takeoff=t0 - 250, t_arr_ptc=t + 300, t_land=t + 400)
    n = sum(trajectory_samples(p, CFG.dt_4d_s).shape[0] for p in plans.values())
    assert n > 6000
    for t_sync in (0.0, 1500.0):
        t1 = time.perf_counter()
        cs = check_separation(plans, CFG, t_sync_s=t_sync)
        assert time.perf_counter() - t1 < 0.5
        assert cs and all(c.uav_a < c.uav_b for c in cs)
        assert len({c.key() for c in cs}) == len(cs)


# ---------------------------------------------------------------------------
# 走廊時槽擬合（_fit_takeoff_shift / _fit_takeoff_advance / prev_free）
# ---------------------------------------------------------------------------

def _brute_min_shift(sch: CorridorSchedule, plan: UAVPlan, s_req: float, to_site: str,
                     forbidden=(), hi: float = 4000.0, step: float = 0.1) -> float | None:
    """暴力解：最小 s ≥ s_req 使起降預約皆無重疊且 s 不在禁止區間內。"""
    (ts0, ts1), (ls0, ls1) = D.plan_windows(plan, CFG)
    s = float(s_req)
    while s <= hi:
        if (sch.is_free(to_site, ts0 + s, ts1 + s) and sch.is_free(plan.landing_site, ls0 + s, ls1 + s)
                and not any(lo < s < h for lo, h in forbidden)):
            return s
        s = round(s + step, 6)
    return None


def _busy_site_schedule() -> tuple[CorridorSchedule, UAVPlan]:
    """單一站點被 5 架佔滿的走廊表 + 一架待安排之計畫（模擬山丘 8 機共用 H）。"""
    sch = CorridorSchedule(CFG)
    for k in range(5):
        sch.reserve("H", "takeoff", 120.0 + 60.0 * k, 180.0 + 60.0 * k, f"X{k}")
        sch.reserve("H", "landing", 2300.0 + 250.0 * k, 2520.0 + 250.0 * k, f"X{k}")
    plan = strip_plan("B4", "B", 0.0, 200.0, 900.0, 2500.0)
    plan = replace(plan, t_takeoff_s=700.0, t_arr_ptc_s=2800.0, t_land_s=2900.0)
    return sch, plan


def test_fit_takeoff_shift_is_minimal_and_avoids_forbidden_intervals():
    """(任務 2) 走廊 ∩ 禁止延遲區間之最小元素；舊版只做走廊，26.7 s 需求被推成 2083 s 後又落回禁止區。"""
    sch, plan = _busy_site_schedule()
    s_req = 26.7
    got = D._fit_takeoff_shift(sch, plan, s_req, "H", CFG, -math.inf)
    exp = _brute_min_shift(sch, plan, s_req, "H")
    assert got is not None and exp is not None and got == pytest.approx(exp, abs=0.2)
    forbid = ((0.0, 20.0), (got - 50.0, got + 120.0))
    got2 = D._fit_takeoff_shift(sch, plan, s_req, "H", CFG, -math.inf, forbid)
    exp2 = _brute_min_shift(sch, plan, s_req, "H", forbid)
    assert got2 is not None and exp2 is not None and got2 == pytest.approx(exp2, abs=0.2)
    assert got2 > got and not any(lo < got2 < hi for lo, hi in forbid)
    assert D._fit_takeoff_shift(sch, plan, 0.0, "H", CFG, 1500.0) >= 1500.0 - plan.t_takeoff_s


def test_prev_free_and_takeoff_advance():
    """prev_free = next_free 之反向；_fit_takeoff_advance 給出「提前起飛」（規格 §13.6 (i) 換序）。"""
    sch = CorridorSchedule(CFG)
    sch.reserve("H", "takeoff", 100.0, 160.0, "A1")
    sch.reserve("H", "landing", 900.0, 1120.0, "A1")
    assert sch.prev_free("H", "takeoff", 150.0, 60.0) == pytest.approx(40.0)
    assert sch.prev_free("H", "takeoff", 300.0, 60.0) == pytest.approx(300.0)
    assert sch.prev_free("ALT1", "landing", 50.0, 20.0) == pytest.approx(50.0)
    plan = strip_plan("B1", "B", 0.0, 200.0, 900.0, 1400.0)
    plan = replace(plan, t_takeoff_s=600.0, t_arr_ptc_s=1700.0, t_land_s=1800.0)
    # 不需提前（s = 0 已可行）-> None
    assert D._fit_takeoff_advance(sch, plan, 0.0, "H", CFG, -math.inf) is None
    # s = 0 落在禁止區間 (-200, 50) 內 -> 必須往負方向跳出，且結果仍要走廊乾淨
    forbid = ((-200.0, 50.0),)
    s = D._fit_takeoff_advance(sch, plan, 0.0, "H", CFG, -math.inf, forbid)
    assert s is not None and s <= -200.0 + 1e-9
    (t0, t1), (l0, l1) = D.plan_windows(plan, CFG)
    assert sch.is_free("H", t0 + s, t1 + s) and sch.is_free(plan.landing_site, l0 + s, l1 + s)
    assert t0 + s >= -1e-9 and not any(lo < s < hi for lo, hi in forbid)
    assert D._fit_takeoff_advance(sch, plan, 0.0, "H", CFG, 600.0, forbid) is None   # t_now 卡住


# ---------------------------------------------------------------------------
# 延遲盤旋 vs 降落時槽（任務 3）
# ---------------------------------------------------------------------------

def _repair_state(plans, ctx, models, sched, **kw):
    return D._Repair(plans=dict(plans), ctx=ctx, models=dict(models), sched=sched, cfg=CFG,
                     to_site={u: "H" for u in plans}, t_now_s=kw.get("t_now_s", -math.inf), t_sync_s=0.0,
                     v_ref_mps=20.0, ghosts={}, energy_check=D.default_energy_ok, hook=None)


def test_orbit_n_for_sizes_by_delta_req_and_escapes_forbidden():
    """(任務 3) 圈數只由 Δ_req（與禁止延遲區間）決定，與降落時槽無關。"""
    assert D._orbit_n_for(0.0, 60.9, ()) == 1
    assert D._orbit_n_for(100.0, 60.9, ()) == 2
    assert D._orbit_n_for(122.0, 60.9, ()) == 3
    # 2 圈（121.8 s）落在禁止區間 -> 往上跳到 3 圈
    assert D._orbit_n_for(100.0, 60.9, ((100.0, 130.0),)) == 3


def test_orbit_delay_is_sized_by_conflict_not_by_landing_slot(mini_ctx):
    """(任務 3) 降落時槽被佔時，仍要產生「圈數 = ⌈Δ_req/T_lap⌉」的候選（由走廊連鎖吸收時槽），
    舊版只會把圈數一路加到有空時槽（山丘：1 -> 39/58 圈）而後能量重驗必敗、完全沒有候選。"""
    ctx = mini_ctx
    t_cross = 1500.0
    plans = _crossing_plans(t_cross)
    models = {u: EnergyModel(PLATFORM_PIONEER, 24.6, 1350.0, 1500.0, 1500.0) for u in plans}
    t_lap, _r = D.orbit_lap_time_s(ctx.specs["A"], ctx.wind, 300.0)
    cs = check_separation(plans, CFG)
    assert len(cs) == 1
    need = max(cs[0].delay_a_s, cs[0].delay_b_s)
    n_free_exp = math.ceil(need / t_lap)

    # (a) 降落走廊乾淨 -> 圈數就是 Δ_req 決定的值
    sch = CorridorSchedule(CFG)
    for u, p in plans.items():
        sch.reserve_plan(p, "H")
    R = _repair_state(plans, ctx, models, sch, t_now_s=t_cross - 100.0)
    acts = D._orbit_actions(cs[0], R)
    assert acts and min(int(a.cost) for a in acts) <= n_free_exp

    # (b) 降落窗之後塞滿 6 個 250 s 的「不可動」預約（他人／幽靈）-> 仍有候選（後備：加圈數）
    sch2 = CorridorSchedule(CFG)
    for u, p in plans.items():
        sch2.reserve_plan(p, "H")
    (_, _), (l0, _l1) = D.plan_windows(plans["A1"], CFG)
    for k in range(6):
        sch2.reserve("H", "landing", l0 + 30.0 + 250.0 * k, l0 + 280.0 + 250.0 * k, f"Z{k}")
    R2 = _repair_state(plans, ctx, models, sch2, t_now_s=t_cross - 100.0)
    acts2 = D._orbit_actions(cs[0], R2)
    assert acts2
    best = min(acts2, key=lambda a: a.cost)
    u = next(u for u, p in best.plans.items() if p.orbits_inserted > 0)
    assert D.default_energy_ok(best.plans[u], models[u])


def test_cascade_orbit_moves_a_ground_uav_out_of_the_landing_slot(mini_ctx):
    """(任務 3) 盤旋後撞到的降落時槽，若佔用者尚未起飛，改由**它**讓開（連鎖），而不是自己多繞圈。"""
    ctx = mini_ctx
    flying = strip_plan("A1", "A", 0.0, 260.0, 300.0, 1200.0)              # 觸地 1600、P_tc 1500
    ground = strip_plan("A2", "A", 900.0, 260.0, 2000.0, 2300.0)           # 起飛 1750（> t_now、在 A1 降落窗之後）
    plans = {"A1": flying, "A2": ground}
    models = {u: EnergyModel(PLATFORM_PIONEER, 24.6, 1350.0, 1500.0, 1500.0) for u in plans}
    sch = CorridorSchedule(CFG)
    for u, p in plans.items():
        sch.reserve_plan(p, "H")
    assert sch.conflicts() == []                                           # 原本不衝突
    R = _repair_state(plans, ctx, models, sch, t_now_s=400.0)              # A1 已起飛、A2 尚未
    delayed = D.insert_delay_orbit(flying, D.orbit_insert_index(flying, 400.0), 3, 60.9, models["A1"], 200.0)
    s2 = CorridorSchedule(CFG)
    s2.reserve_plan(delayed, "H")
    s2.reserve_plan(ground, "H")
    assert s2.conflicts()                                                  # 盤旋後撞上 A2 的降落窗
    out = D._cascade_orbit(R, "A1", delayed)
    assert out is not None and "A2" in out and "A1" not in out
    assert out["A2"].t_takeoff_s > ground.t_takeoff_s                       # 讓開的是尚未起飛的 A2
    s3 = CorridorSchedule(CFG)
    s3.reserve_plan(delayed, "H")
    s3.reserve_plan(out["A2"], "H")
    assert s3.conflicts() == []


def test_cascade_shift_moves_blockers_instead_of_self(mini_ctx):
    """(任務 2/3) 走廊被佔 → 連鎖讓開（被擠者尚未起飛時），而不是把自己推到下一個空時槽。"""
    ctx = mini_ctx
    a = strip_plan("A1", "A", 0.0, 260.0, 300.0, 1200.0)
    b = strip_plan("A2", "A", 900.0, 260.0, 300.0, 1200.0)
    b = replace(b, t_takeoff_s=a.t_takeoff_s + 60.0, t_arr_ptc_s=a.t_arr_ptc_s + 60.0,
                t_land_s=a.t_land_s + 60.0)
    plans = {"A1": a, "A2": b}
    models = {u: EnergyModel(PLATFORM_PIONEER, 24.6, 1350.0, 1500.0, 1500.0) for u in plans}
    sch = CorridorSchedule(CFG)
    for u, p in plans.items():
        sch.reserve_plan(p, "H")
    R = _repair_state(plans, ctx, models, sch)
    casc = D._cascade_shift(R, "A1", 40.0)
    assert casc is not None and casc["A1"] == pytest.approx(40.0)
    s2 = CorridorSchedule(CFG)
    for u, p in plans.items():
        s2.reserve_plan(D.shift_plan(p, casc.get(u, 0.0)), "H")
    assert s2.conflicts() == []
    R2 = _repair_state(plans, ctx, models, sch.copy(), t_now_s=1e9)
    assert D._cascade_shift(R2, "A1", 40.0) is None


# ---------------------------------------------------------------------------
# 起降走廊豁免之正當性（任務 4）
# ---------------------------------------------------------------------------

def _exempt_sample_pairs(pa: UAVPlan, pb: UAVPlan):
    """被 SITE_CORRIDOR_PHASES 規則豁免之樣本對 [(t_a, t_b, pad_xy)]。"""
    ta = D._track_from_plan(pa, CFG.dt_4d_s, 0.0, CFG)
    tb = D._track_from_plan(pb, CFG.dt_4d_s, 0.0, CFG)
    r = max(CFG.d_sep_m, CFG.d_sep_lat_m)
    ii, jj, _ = D._cross_pairs(ta.samples[:, :2], tb.samples[:, :2], r)
    pads_a, pads_b = ta.pad(), tb.pad()
    out = []
    for i, j in zip(ii, jj):
        pa_xy, pb_xy = pads_a[i], pads_b[j]
        if np.isnan(pa_xy[0]) or np.isnan(pb_xy[0]):
            continue
        if math.hypot(pa_xy[0] - pb_xy[0], pa_xy[1] - pb_xy[1]) > D.SAME_PAD_TOL_M:
            continue
        out.append((float(ta.samples[i, 3]), float(tb.samples[j, 3]), tuple(pa_xy)))
    return out


def test_site_corridor_exemption_is_covered_by_schedule():
    """(任務 4) 每組被豁免之樣本對，兩機在**同一站**都有涵蓋該時刻之走廊預約 → 必落在 CorridorSchedule 管轄內。"""
    a = strip_plan("A1", "A", 0.0, 260.0, 300.0, 1200.0)
    b = strip_plan("B1", "B", -400.0, 200.0, 340.0, 1000.0)
    sch = CorridorSchedule(CFG)
    sch.reserve_plan(a, "H")
    sch.reserve_plan(b, "H")
    pairs = _exempt_sample_pairs(a, b)
    assert pairs

    def covering(uav_id, t):
        return [r for r in sch.reservations if r.uav_id == uav_id and r.t_start_s - 1e-6 <= t <= r.t_end_s + 1e-6]

    for t_a, t_b, _pad in pairs:
        ra, rb = covering("A1", t_a), covering("B1", t_b)
        assert ra and rb
        assert {r.site_id for r in ra} == {r.site_id for r in rb} == {"H"}
    assert isinstance(sch.conflicts(), list)


def test_site_corridor_exemption_requires_same_pad_and_own_window():
    """(任務 4) 三個必要條件缺一即不豁免：不同場坪、時刻在自己預約窗外、非走廊階段。"""
    a = strip_plan("A1", "A", 0.0, 260.0, 300.0, 1200.0)
    far = strip_plan("A2", "A", 0.0, 260.0, 300.0, 1200.0, home=(3000.0, 0.0), site="ALT1")
    assert not _exempt_sample_pairs(a, far)
    same = strip_plan("A2", "A", 500.0, 260.0, 300.0, 1200.0)
    assert _exempt_sample_pairs(a, same)
    shifted = replace(same, t_takeoff_s=same.t_takeoff_s + 5000.0)
    ta = D._track_from_plan(shifted, CFG.dt_4d_s, 0.0, CFG)
    assert np.isnan(ta.pad()[0, 0])
    trk = D._track_from_plan(a, CFG.dt_4d_s, 0.0, CFG)
    (_, _), (l0, _l1) = D.plan_windows(a, CFG)
    mid = int(np.argmin(np.abs(trk.samples[:, 3] - 0.5 * (a.waypoints[0].t_s + l0))))
    assert np.isnan(trk.pad()[mid, 0])


def test_same_pad_corridor_pair_is_left_to_corridor_schedule():
    """同站同時起飛之兩機：4D 不報（豁免），CorridorSchedule 報。"""
    a = strip_plan("A1", "A", 0.0, 260.0, 300.0, 1200.0)
    b = strip_plan("A2", "A", 400.0, 260.0, 300.0, 1200.0)
    dep = [w.t_s for w in a.waypoints if w.phase is Phase.TAKEOFF]
    arr = [w.t_s for w in a.waypoints if w.phase is Phase.LAND]
    cs = check_separation({"A1": a, "A2": b}, CFG)
    assert not [c for c in cs if c.t_s <= max(dep) + 1e-6]               # TAKEOFF 段（同場坪）無 4D 衝突
    assert not [c for c in cs if c.t_s >= min(arr) - 1e-6]               # LAND 段亦然
    assert cs and all(max(dep) < c.t_s < min(arr) for c in cs)           # 只剩轉場段（ENTRY/RTB）之衝突
    sch = CorridorSchedule(CFG)
    sch.reserve_plan(a, "H")
    sch.reserve_plan(b, "H")
    cc = sch.conflicts()
    assert cc and any(x.res_kinds == ("takeoff", "takeoff") for x in cc)


# ---------------------------------------------------------------------------
# (iii) IDP 重排 hook（任務 1）
# ---------------------------------------------------------------------------

def test_resequence_hook_receives_request_and_may_change_several_uavs(mini_ctx):
    """(任務 1) 新式回呼 hook(ResequenceRequest) -> dict|UAVPlan|None；輸入含衝突、機序、計畫、走廊表、ctx/cfg。"""
    ctx = mini_ctx
    plans = _crossing_plans(2000.0, margin=5.0)
    models = {"A1": ctx.models["A1"], "A2": EnergyModel(PLATFORM_PIONEER, 24.6, 1350.0, 1500.0, 1500.0)}
    cs = check_separation(plans, CFG)
    assert cs
    seen = []

    def hook(req):
        seen.append(req)
        u = req.uav_ids[0]
        return {u: D.shift_plan(req.plans[u], 400.0, from_index=2)}

    new, residual = repair(plans, cs, ctx, models, CorridorSchedule(CFG), CFG, t_now_s=1950.0,
                           resequence_hook=hook)
    assert seen and residual == []
    req = seen[0]
    assert isinstance(req, D.ResequenceRequest)
    assert req.conflict.kind in ("4d", "corridor", "ghost")
    assert req.uav_ids and set(req.uav_ids) <= {"A1", "A2"}
    assert req.plan_of(req.uav_ids[0]) is req.plans[req.uav_ids[0]]
    assert req.cfg is CFG and req.ctx is ctx
    assert isinstance(req.schedule, CorridorSchedule)
    assert req.takeoff_sites and req.t_now_s == 1950.0 and req.t_sync_s == 0.0
    assert req.energy_check(req.plans[req.uav_ids[0]], req.model_of(req.uav_ids[0])) in (True, False)
    assert req.uav_ids[0] == min(req.uav_ids, key=lambda u: req.plans[u].t_land_s)
    assert any(new[u] is not plans[u] for u in plans)
    assert all(p.orbits_inserted == 0 for p in new.values())


def test_resequence_hook_none_and_rejected_result_leave_residual(mini_ctx):
    ctx = mini_ctx
    plans = _crossing_plans(2000.0, margin=5.0)
    models = {"A1": ctx.models["A1"], "A2": EnergyModel(PLATFORM_PIONEER, 24.6, 1350.0, 1500.0, 1500.0)}
    cs = check_separation(plans, CFG)
    new, residual = repair(plans, cs, ctx, models, CorridorSchedule(CFG), CFG, t_now_s=1950.0,
                           resequence_hook=lambda req: None)
    assert residual and all(new[u] is plans[u] for u in plans)
    new2, residual2 = repair(plans, cs, ctx, models, CorridorSchedule(CFG), CFG, t_now_s=1950.0,
                             resequence_hook=lambda req: {req.uav_ids[0]: req.plans[req.uav_ids[0]]})
    assert residual2


def test_resequence_hook_legacy_three_arg_signature(mini_ctx):
    """相容舊式 hook(uav_id, plan, conflict) -> UAVPlan | None。"""
    ctx = mini_ctx
    plans = _crossing_plans(2000.0, margin=5.0)
    models = {"A1": ctx.models["A1"], "A2": EnergyModel(PLATFORM_PIONEER, 24.6, 1350.0, 1500.0, 1500.0)}
    cs = check_separation(plans, CFG)
    calls = []

    def hook(uid, plan, conflict):
        calls.append((uid, conflict.kind))
        return D.shift_plan(plan, 400.0, from_index=2)

    new, residual = repair(plans, cs, ctx, models, CorridorSchedule(CFG), CFG, t_now_s=1950.0,
                           resequence_hook=hook)
    assert calls and residual == []
    assert D._hook_arity(hook) == 3 and D._hook_arity(lambda req: None) == 1


# ---------------------------------------------------------------------------
# 禁止延遲區間 / 工期壓縮
# ---------------------------------------------------------------------------

def test_forbidden_intervals_and_escape():
    # 延遲 b 之禁止區間 (L, U) = ((-dt - win)/(1+k), (win - dt)/(1-k))；k=0 -> (-350,-250) 與 (-50, 50)
    dt = np.array([0.0, 300.0])
    win = np.array([50.0, 50.0])
    ivs = D._forbidden_intervals(dt, win, 0.0)
    assert ivs == ((-350.0, -250.0), (-50.0, 50.0))
    assert D._escape(0.0, ivs) == pytest.approx(50.0)
    assert D._escape(-300.0, ivs) == pytest.approx(-250.0)
    assert D._escape(100.0, ivs) == pytest.approx(100.0)
    assert D._escape_down(0.0, ivs) == pytest.approx(-50.0)
    assert D._escape_down(-260.0, ivs) == pytest.approx(-350.0)
    iv_k = D._forbidden_intervals(np.array([0.0]), np.array([50.0]), 0.05)
    assert iv_k[0][1] == pytest.approx(50.0 / 0.95) and iv_k[0][0] == pytest.approx(-50.0 / 1.05)
    assert D._merge_intervals([(0.0, 10.0), (5.0, 20.0), (30.0, 40.0)]) == ((0.0, 20.0), (30.0, 40.0))
    a = oper_only("A1", 0.0, 260.0, 300.0, 450.0)
    b = oper_only("A2", 50.0, 260.0, 300.0, 450.0)
    c = check_separation({"A1": a, "A2": b}, CFG)[0]
    assert c.forbid_a and c.forbid_b and c.forbid_now_a and c.forbid_now_b
    assert c.delay_a_s == pytest.approx(D._escape(0.0, c.forbid_a))
    assert c.delay_b_s == pytest.approx(D._escape(0.0, c.forbid_b))
    assert D._escape(0.0, c.forbid_now_a) <= c.delay_a_s + 1e-9


def test_compact_makespan_recovers_excess_delay(mini_ctx):
    """修復只會加延遲；壓縮步在不新增衝突下把多餘延遲吐回來（零能量、不改 z）。"""
    ctx = mini_ctx
    base = strip_plan("A1", "A", 0.0, 260.0, 300.0, 1200.0)
    a = D.shift_plan(base, 3000.0)
    plans = {"A1": a}
    models = {"A1": ctx.models["A1"]}
    sch = CorridorSchedule(CFG)
    sch.reserve_plan(a, "H")
    R = _repair_state(plans, ctx, models, sch)
    assert D._compact_makespan(R)
    assert R.plans["A1"].t_takeoff_s < a.t_takeoff_s
    assert R.plans["A1"].t_land_s < a.t_land_s
    assert R.plans["A1"].energy_margin_wh == a.energy_margin_wh
    assert [w.z_m for w in R.plans["A1"].waypoints] == [w.z_m for w in a.waypoints]
    assert R.sched.conflicts() == []
