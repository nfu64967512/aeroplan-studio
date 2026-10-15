"""起降走廊豁免必須比站點**身分**，不是場坪**座標**（vendored 修改 #8）。

⚠⚠⚠ 2026-09-09 實際缺陷。兩層寫法不一致：

* `_pair_metrics` 的豁免：兩機場坪座標相距 ≤ `SAME_PAD_TOL_M`（50 m）就當「同一站」，
  不計 4D 衝突，理由是「該處互斥由 `CorridorSchedule` 保證」。
* `CorridorSchedule` 的互斥：以 ``site_id`` **字串**為鍵。

⇒ 兩個**不同**的站相距 < 50 m 時，豁免成立但排程**不會**把它們排進同一條隊列，
真違規就被靜靜丟掉。四台、每機一個起飛站（`per_uav_takeoff_sites`）的實測：

===========  ================  =========  ==============  =================
起飛站間距   被豁免的真違規    真同站     **不同站**      最近的一筆
===========  ================  =========  ==============  =================
200 m        0                 0          0               —
100 m        0                 0          0               —
60 m         496               **0**      **496**         30.0 m / 6.0 m
30 m         1037              **0**      **1037**        15.0 m / 3.6 m
15 m         1062              **0**      **1062**        7.5 m / 22.3 m
===========  ================  =========  ==============  =================

門檻是水平 100 m／垂直 50 m，而 ``validate`` **每一組都回報通過**。100% 的受害樣本
都是不同站，真同站一筆都沒有。最嚴重一筆是一機**起飛**、一機**降落**（B2 於
t=756 s 爬過 20 m，B3 於 t=766 s 降到 14 m，水平 30 m）。

⚠⚠ 為什麼潛伏到現在：舊 docstring 明寫「不同站（相距數 km）」——在「每機一個起飛站」
之前，不同站真的都在數公里外，0–50 m 這一帶從來不存在。

⚠⚠⚠ 而且既有那條「守著豁免」的測試咬不到：
``test_site_corridor_exemption_is_covered_by_schedule`` **自己**把兩機都
``reserve_plan(p, "H")``，再斷言兩者站點相同 —— 循環論證，結構上不可能紅。
另一條 ``..._requires_same_pad_and_own_window`` 測「不同場坪」用的是 3 km 外。
"""

from __future__ import annotations

import math

import numpy as np
import pytest

pytest.importorskip("shapely")
pytest.importorskip("scipy")

from hv_dccpp import deconflict as D                            # noqa: E402
from hv_dccpp.deconflict import check_separation                # noqa: E402
from hv_dccpp.mission import Mode, Phase, PlannerConfig, UAVPlan, Waypoint   # noqa: E402

CFG = PlannerConfig()


def _wp(x, y, z, t, phase, *, track_deg=90.0, energy=1000.0):
    mode = Mode.HOVER if phase in (Phase.TAKEOFF, Phase.LAND) else (
        Mode.TRANSITION if phase in (Phase.TRANS_OUT, Phase.TRANS_IN) else Mode.CRUISE)
    return Waypoint(x, y, z, 0.0, 0.0, z + 100.0, z, t, mode, phase, False,
                    None, None, None, 21.5, 20.0, track_deg, track_deg, energy, "ALT1", True, "RTB")


def _leg(p0, p1, z0, z1, t0, t1, phase, n=11):
    (x0, y0), (x1, y1) = p0, p1
    trk = math.degrees(math.atan2(x1 - x0, y1 - y0)) % 360.0
    return [_wp(x0 + f * (x1 - x0), y0 + f * (y1 - y0), z0 + f * (z1 - z0), t0 + f * (t1 - t0), phase,
                track_deg=trk)
            for f in (i / (n - 1) for i in range(n))]


def _departing(uav_id, pad, t0, *, to_site, ld_site, ld_pad):
    """``t0`` 起自 ``pad`` 垂直起飛（0→60 m，30 s），再以 TRANS_OUT 在 30 s 內離場到 3 km 外。

    ⚠⚠ 離場段必須**快**、仍屬走廊相位、而且方位要跟進場段**相反**：
    (a) 起飛窗只有 ``dt_dep_s``=60 s，超出窗的樣本站點身分為 ""（不豁免）；
    (b) 4D 視窗含 σ_t（上限 45 s×2）⇒ 視窗寬達 92 s，**七十秒外**的樣本照樣配對成功。
    第一版把 ENTRY 留在坪上方，第二版讓離場與返場同向（兩條線在坪外 100 m 內交會）——
    兩次撞出來的都是**非走廊**樣本（RTB／ENTRY），測到的根本不是豁免。
    """
    px, py = pad
    wps = _leg(pad, pad, 0.0, 60.0, t0, t0 + 30.0, Phase.TAKEOFF, 16)
    wps += _leg(pad, (3000.0, -2000.0), 60.0, 260.0, t0 + 30.0, t0 + 60.0, Phase.TRANS_OUT, 16)[1:]
    wps += _leg((3000.0, -2000.0), (9000.0, -9000.0), 260.0, 260.0, t0 + 60.0, t0 + 1140.0, Phase.ENTRY)[1:]
    wps += _leg((9000.0, -9000.0), ld_pad, 260.0, 60.0, t0 + 1140.0, t0 + 1170.0, Phase.RTB, 6)[1:]
    wps += _leg(ld_pad, ld_pad, 60.0, 0.0, t0 + 1170.0, t0 + 1200.0, Phase.LAND, 6)[1:]
    return UAVPlan(uav_id, "A", wps, [], t0, t0 + 1170.0, t0 + 1200.0, ld_site,
                   300.0, 200.0, 10000.0, 20.0, 0, [], takeoff_site=to_site)


def _arriving(uav_id, pad, t_land, *, to_site, ld_site, to_pad):
    """自遠處回場，最後 30 s 以 TRANS_IN 由 (200 m, 60 m) 進場、於 ``t_land`` 觸地於 ``pad``。"""
    t0 = t_land - 1200.0
    px, py = pad
    wps = _leg(to_pad, to_pad, 0.0, 60.0, t0, t0 + 30.0, Phase.TAKEOFF, 6)
    wps += _leg(to_pad, (-3000.0, 2000.0), 60.0, 260.0, t0 + 30.0, t0 + 60.0, Phase.TRANS_OUT, 6)[1:]
    wps += _leg((-3000.0, 2000.0), (px + 200.0, py), 260.0, 60.0, t0 + 60.0, t_land - 30.0, Phase.RTB)[1:]
    wps += _leg((px + 200.0, py), pad, 60.0, 0.0, t_land - 30.0, t_land, Phase.TRANS_IN, 16)[1:]
    return UAVPlan(uav_id, "A", wps, [], t0, t_land - 30.0, t_land, ld_site,
                   300.0, 200.0, 10000.0, 20.0, 0, [], takeoff_site=to_site)


# 一台於 (0,0) 垂直起飛、一台於 (30,0) 垂直降落，兩段在時間上重疊 ⇒ 只有走廊相位會靠近。
PAD_A = (0.0, 0.0)
PAD_B = (30.0, 0.0)
T_MEET = 3000.0


def _scenario(*, to_site_a, ld_site_b):
    a = _departing("A1", PAD_A, T_MEET, to_site=to_site_a, ld_site="FAR", ld_pad=(20000.0, -20000.0))
    b = _arriving("A2", PAD_B, T_MEET + 20.0, to_site="FAR2", ld_site=ld_site_b,
                  to_pad=(-20000.0, 20000.0))
    return {"A1": a, "A2": b}


def _corridor_conflicts(plans):
    return [c for c in check_separation(plans, CFG)
            if abs(c.t_s - T_MEET) < 60.0]


# ---------------------------------------------------------------------------


def test_the_two_aircraft_really_do_come_within_the_threshold():
    """⭐ 前置斷言：情境**真的**造出一筆該報的違規，否則下面幾條等於什麼都沒驗。

    ⚠ 這正是既有守衛缺的東西 —— 沒有它，「沒報衝突」讀起來像「安全」，
    其實可能只是兩機根本沒靠近。
    """
    plans = _scenario(to_site_a="TO_A", ld_site_b="H")
    ta = D._track_from_plan(plans["A1"], CFG.dt_4d_s, 0.0, CFG)
    tb = D._track_from_plan(plans["A2"], CFG.dt_4d_s, 0.0, CFG)
    ii, jj, _ = D._cross_pairs(ta.samples[:, :3], tb.samples[:, :3], 1e9)
    d_h = np.hypot(ta.samples[ii, 0] - tb.samples[jj, 0], ta.samples[ii, 1] - tb.samples[jj, 1])
    d_v = np.abs(ta.samples[ii, 2] - tb.samples[jj, 2])
    dt = np.abs(tb.samples[jj, 3] - ta.samples[ii, 3])
    close = (d_h < CFG.d_sep_m) & (d_v < CFG.dh_sep_m) & (dt < CFG.dt_4d_s)
    assert close.any(), "情境沒有任何時空重疊 ⇒ 這個檔量不到東西，要換情境"
    # 兩機都在自己的走廊窗內（豁免條件 1、2 成立），差別只在條件 3
    assert (ta.site()[ii][close] == "TO_A").any()
    assert (tb.site()[jj][close] == "H").any()


def test_distinct_sites_30m_apart_are_not_exempt():
    """⭐⭐⭐ 值層級：兩個**不同**的站相距 30 m ⇒ 必須照報 4D 衝突。

    修好之前這條是紅的：座標 30 m ≤ SAME_PAD_TOL_M(50) ⇒ 被當成同站豁免掉，
    而 `CorridorSchedule` 以 site_id 為鍵，根本不會把 TO_A 和 H 排進同一條隊列。
    """
    plans = _scenario(to_site_a="TO_A", ld_site_b="H")
    assert _corridor_conflicts(plans), (
        "起飛站 TO_A 與降落站 H 相距 30 m 但是**不同的站**，走廊排程不會互斥它們 ⇒ "
        "4D 必須自己報，不可以豁免")


def test_same_site_is_still_exempt():
    """⭐⭐ 反向臂：同一個站就要豁免（否則同站起降在 σ_t 視窗下恆為衝突，修復發散）。

    ⚠ 幾何與上一條**完全相同**，只有 site_id 不同 ⇒ 差異只可能來自身分判定。
    """
    plans = _scenario(to_site_a="H", ld_site_b="H")
    assert not _corridor_conflicts(plans), "同站起降應由 CorridorSchedule 管，4D 不該報"


def test_missing_ids_fall_back_to_pad_distance():
    """計畫沒帶 `takeoff_site`（手搭 UAVPlan）時退回比座標 —— 缺陷在這條路徑上仍在。

    ⚠ 記錄而非核可：正式規劃路徑一律帶身分（見
    `test_planner_always_stamps_the_takeoff_site`），所以退回路徑只影響手搭計畫。
    """
    plans = _scenario(to_site_a="", ld_site_b="H")
    assert not _corridor_conflicts(plans), "身分未知時應退回舊行為（座標 ≤ 50 m 視為同站）"


def test_exemption_predicate_reads_identity_not_distance():
    """⭐⭐ 直接鎖判準：站點相同 ⇒ 豁免；站點不同 ⇒ 不豁免，**與場坪距離無關**。

    ⚠ 用值層級鎖（餵資料看輸出），不是鎖原始碼那一行長什麼樣。
    """
    plans = _scenario(to_site_a="TO_A", ld_site_b="H")
    ta = D._track_from_plan(plans["A1"], CFG.dt_4d_s, 0.0, CFG)
    tb = D._track_from_plan(plans["A2"], CFG.dt_4d_s, 0.0, CFG)
    ii, jj, _ = D._cross_pairs(ta.samples[:, :3], tb.samples[:, :3], 1e9)
    _dh, _dv, _dt, _win, spatial = D._pair_metrics(ta, tb, ii, jj, CFG)

    sa, sb = ta.site()[ii], tb.site()[jj]
    pads = np.hypot(ta.pad()[ii][:, 0] - tb.pad()[jj][:, 0],
                    ta.pad()[ii][:, 1] - tb.pad()[jj][:, 1])
    near_but_distinct = (sa != "") & (sb != "") & (sa != sb) & (pads <= D.SAME_PAD_TOL_M)
    assert near_but_distinct.any(), "情境沒有『場坪很近但站點不同』的樣本 ⇒ 這條測不到"

    d_h = np.hypot(ta.samples[ii, 0] - tb.samples[jj, 0], ta.samples[ii, 1] - tb.samples[jj, 1])
    d_v = np.abs(ta.samples[ii, 2] - tb.samples[jj, 2])
    should = near_but_distinct & (d_h < CFG.d_sep_m) & (d_v < CFG.dh_sep_m)
    assert should.any() and spatial[should].all(), (
        "場坪 30 m 但站點不同的樣本被判成『空間分離』⇒ 豁免還在比座標")


def test_planner_always_stamps_the_takeoff_site():
    """⭐⭐⭐ 值層級：正式規劃出來的每一架都必須帶起飛站身分，且等於 planner 用的**起飛**站。

    ⚠ 少了這條，只要有人漏填 `takeoff_site`，豁免就整批**靜靜**退回比座標
    —— 而退回路徑的表徵是「一切正常」。

    ⚠⚠ 必須有一架的起飛站 ≠ 降落站，否則這條分辨不出「蓋成 start」和「蓋成 landing_site」：
    mini example 全隊都是 H 起、H 降，突變成 ``takeoff_site=site`` 時 **8 個突變臂裡唯一
    沒咬到的那一個**就是這樣溜過去的。
    """
    import dataclasses

    import hv_dccpp.planner as P
    from hv_dccpp.context import build_context
    from hv_dccpp.scenario import build_mini_example

    inputs, cfg = build_mini_example()
    victim = inputs.fleet[0].uav_id
    alt = next((k for k in inputs.sites if k != inputs.home.site_id), None)
    assert alt, "mini example 沒有第二個站 ⇒ 這條分辨不出起飛站與降落站"
    fleet = tuple(dataclasses.replace(u, start_site_id=(alt if u.uav_id == victim else None))
                  for u in inputs.fleet)
    ins = dataclasses.replace(inputs, fleet=fleet)
    ctx = build_context(ins, cfg)

    # 前置斷言：這個設定**真的**讓某一架的起飛站不同於降落站
    assert P._start_site(ctx, victim) == alt != ins.home.site_id

    plan = P.plan_mission(ins, cfg)
    assert plan.plans, "mini example 沒規劃出任何計畫 ⇒ 這條測不到"
    assert victim in plan.plans, f"{victim} 沒被規劃 ⇒ 前置斷言失效"
    assert plan.plans[victim].landing_site != alt, (
        f"{victim} 的降落站也變成 {alt} ⇒ 這條又分辨不出兩者，要換受測機")
    for uid, p in plan.plans.items():
        assert p.takeoff_site, f"{uid} 沒帶起飛站 ⇒ 豁免會靜靜退回比座標"
        assert p.takeoff_site == P._start_site(ctx, uid), (
            f"{uid} 的 takeoff_site={p.takeoff_site!r} 與 planner 實際用的 "
            f"{P._start_site(ctx, uid)!r} 不一致")
