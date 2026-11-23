"""vendored 修改 #11：貪婪修復留下殘留衝突時，收尾的全域重排照樣出手、改求全綠（`deconflict.repair`）。

⚠⚠⚠ 根因（2026-10-21，首飛情境 ``gui_legal_120_1a2b_pw3``、crosswind 基準、規劃風 3 m/s 來自 270°）：
validate 回報 1 筆 4D 殘留 B2/B3（各降在自己的坪 TO_B2／TO_B3，兩坪相距 10 m；水平 10 m、垂直 13 m、
dt_ab = 90 s，門檻 Δt + σ_a + σ_b = 2 + 45 + 45 = 92 s）。repair 逐筆記錄到的是：

1. 第 1 輪貪婪先為 A1/B3 把 **A1 延後 1391 s**，A1 的起飛（TO_A1，離 TO_B3 20 m）落在 t = 1451 s，
   緊貼在 B3 降落（1345 s）之後；
2. B2/B3 的每個候選都**只在這一對內**成立：B3 +178.5 s（Conflict.delay_b_s）讓 B3 的降落壓到 A1 的起飛爬升
   （水平 20 m、垂直 48.7 m、dt −0.5 s ⇒ Δ_req 91.5），B2 +271 s 一樣撞 A1，B3 繞 2／5 圈、B2 繞 7 圈也都留
   一筆新衝突 ⇒ (衝突數, ΣΔ_req) 沒有嚴格下降，全被 `_repair_one` 的單調性門檻擋掉（這道門檻是對的：
   放行會把衝突推給別的機對）；
3. 真正解得掉它的是協同平移（B2、B3、A1 一起重排），而唯一做得到的 `_global_reschedule`（修改 #3）
   被 ``and not current`` 擋著——**只有在已經沒有衝突時才跑**。

修改：有殘留時照樣呼叫，``resolve=True`` ⇒ 現況不合格，任何全綠擺放都比它好（``incumbent`` 從 ∞ 起算）；
落地上界與「套用前全機對完整驗證」不變 ⇒ 找不到全綠解就原樣退回、殘留照舊回報。沒有殘留時走原路徑
（``resolve=False``）⇒ 原本就全綠的計畫逐位元不變。

本檔的鎖：

* 合成情境（快）：相鄰兩坪、間隔 90 s。``max_rounds=0`` 把貪婪整個拿掉，只剩收尾這一步——
  每一條「改後全綠」都配一條**前置斷言**「舊閘門（``_old_gate``）下仍有殘留」，否則鎖在任何實作下都會綠。
* 掃描完成目標（``reschedule_objective="scan"``）：所有全綠解的掃描完成時刻都比紅的現況晚 ⇒
  沒有 ``incumbent = ∞`` 就選不到（以落地為目標時這條在數學上咬不到，理由見 `_global_reschedule`）。
* 現況上界內無解：第二段把上界放寬一趟最長架次（工期變長、留註記）；真的無解（兩架都已起飛互撞）
  就原樣退回，殘留照舊回報，計畫物件不變。
* 真實首飛情境（慢，`gui_legal_120_1a2b_pw3`＋crosswind）：validate 通過；舊閘門下仍是那一筆 B2/B3。
* 報告：收尾重排有出手 ⇒ 規劃器警告 ``reschedule_resolved_residual(...)``（makespan 前後、cap、參與的機），
  轉接層 report 的 warnings／notes 原樣帶著、GUI 列為 loud note；沒出手（沒有殘留、或舊閘門）就一個字都不寫。
"""
from __future__ import annotations

import dataclasses
import math

import pytest

pytest.importorskip("shapely")
pytest.importorskip("scipy")

from hv_dccpp import deconflict as D                            # noqa: E402
from hv_dccpp.context import build_context                      # noqa: E402
from hv_dccpp.deconflict import CorridorSchedule, check_separation, repair   # noqa: E402
from hv_dccpp.mission import Mode, Phase, PlannerConfig, UAVPlan, Waypoint   # noqa: E402
from hv_dccpp.platform import PLATFORM_HERO, PLATFORM_PIONEER, EnergyModel  # noqa: E402
from hv_dccpp.scenario import build_mini_example                # noqa: E402

CFG = PlannerConfig()


def _wp(x, y, z, t, phase, *, track_deg=90.0, energy=1000.0):
    mode = Mode.HOVER if phase in (Phase.TAKEOFF, Phase.LAND) else (
        Mode.TRANSITION if phase in (Phase.TRANS_OUT, Phase.TRANS_IN) else Mode.CRUISE)
    return Waypoint(x, y, z, 0.0, 0.0, z + 100.0, z, t, mode, phase, phase is Phase.OPERATE,
                    None, None, None, 21.5, 20.0, track_deg, track_deg, energy, "ALT1", True, "RTB")


def _leg(p0, p1, z0, z1, t0, t1, phase, n=11, e0=1000.0, e1=1000.0):
    (x0, y0), (x1, y1) = p0, p1
    trk = math.degrees(math.atan2(x1 - x0, y1 - y0)) % 360.0
    return [_wp(x0 + f * (x1 - x0), y0 + f * (y1 - y0), z0 + f * (z1 - z0), t0 + f * (t1 - t0), phase,
                track_deg=trk, energy=e0 + f * (e1 - e0))
            for f in (i / (n - 1) for i in range(n))]


def _sortie(uav_id, cls, *, to_pad, to_site, pad, site, t_land, out, t_scan_end, t0=0.0):
    """t0 自 ``to_pad`` 垂直起飛 → 快速離場 3 km（方向 ``out``，各機不同 ⇒ 遠處航段互不相交）→ 遠處作業
    （OPERATE，相機開）到 ``t_scan_end`` → 返場 → 最後 30 s 由 200 m 外、60 m 高進場觸地於 ``pad``（站 ``site``）。"""
    ox, oy = out
    far1 = (to_pad[0] + 3000.0 * ox, to_pad[1] + 3000.0 * oy)
    far2 = (to_pad[0] + 9000.0 * ox, to_pad[1] + 9000.0 * oy)
    app = (pad[0] + 200.0 * ox, pad[1] + 200.0 * oy)
    wps = _leg(to_pad, to_pad, 0.0, 60.0, t0, t0 + 30.0, Phase.TAKEOFF, 7)
    wps += _leg(to_pad, far1, 60.0, 260.0, t0 + 30.0, t0 + 60.0, Phase.TRANS_OUT, 7)[1:]
    wps += _leg(far1, far2, 260.0, 260.0, t0 + 60.0, t_scan_end, Phase.OPERATE, 21, 1000.0, 400.0)[1:]
    wps += _leg(far2, app, 260.0, 60.0, t_scan_end, t_land - 30.0, Phase.RTB, 21, 400.0, 300.0)[1:]
    wps += _leg(app, pad, 60.0, 0.0, t_land - 30.0, t_land, Phase.TRANS_IN, 7, 300.0, 290.0)[1:]
    return UAVPlan(uav_id, cls, wps, [], t0, t_land - 30.0, t_land, site,
                   710.0, 5.0, 30000.0, 20.0, 0, [], takeoff_site=to_site)


# 兩架 HERO 的降落坪相距 10 m（與首飛情境 TO_B2／TO_B3 相同）、降落相隔 90 s（< Δt + σ_a + σ_b = 92 s）。
# 兩架都在 t=0 起飛 ⇒ 誰都不能提前；起飛站在 6 km 外兩端（同時起飛不互擾）；B2 在北、B3 在南，只在坪邊相遇。
PAD_B2, PAD_B3 = (0.0, 0.0), (10.0, 0.0)


def _pads(*, with_a1=True, b2_scan=1000.0, b3_scan=1100.0, a1_scan=500.0):
    plans = {
        "B2": _sortie("B2", "B", to_pad=(0.0, 3000.0), to_site="N_B2", pad=PAD_B2, site="TO_B2", t_land=1410.0,
                      out=(0.0, 1.0), t_scan_end=b2_scan),
        "B3": _sortie("B3", "B", to_pad=(10.0, -3000.0), to_site="S_B3", pad=PAD_B3, site="TO_B3", t_land=1500.0,
                      out=(0.0, -1.0), t_scan_end=b3_scan),
    }
    if with_a1:            # 遠處的長任務：決定 makespan（＝重排的落地上界），與兩架 HERO 沒有交集
        plans["A1"] = _sortie("A1", "A", to_pad=(6000.0, 6000.0), to_site="FAR_A1", pad=(6000.0, 6000.0),
                              site="FAR_A1", t_land=3000.0, out=(1.0, 0.0), t_scan_end=a1_scan)
    return plans


def _models():
    return {"B2": EnergyModel(PLATFORM_HERO, 24.6, 1350.0, 1500.0, 1500.0),
            "B3": EnergyModel(PLATFORM_HERO, 24.6, 1350.0, 1500.0, 1500.0),
            "A1": EnergyModel(PLATFORM_PIONEER, 24.6, 1350.0, 1500.0, 1500.0)}


@pytest.fixture(scope="module")
def mini_ctx():
    inputs, cfg = build_mini_example()
    return build_context(inputs, dataclasses.replace(cfg, dem_step_m=50.0), terrain_margin_m=800.0)


@pytest.fixture
def old_gate(monkeypatch):
    """修改前的閘門（``and not current``）：有殘留時收尾的全域重排根本不跑。

    以「resolve=True 的呼叫直接回空集合」重現——與舊碼不呼叫完全等價（舊碼沒有這個參數）。"""
    real = D._global_reschedule

    def gated(R, **kw):
        if kw.get("resolve"):
            return set()
        return real(R, **kw)
    monkeypatch.setattr(D, "_global_reschedule", gated)


def _repair(ctx, plans, cfg=CFG, **kw):
    sch = CorridorSchedule(cfg)
    cs = check_separation(plans, cfg)
    new, residual = repair(plans, cs, ctx, _models(), sch, cfg,
                           takeoff_site={u: p.takeoff_site for u, p in plans.items()}, **kw)
    return cs, new, residual, sch


def _only_time_shifted(new, plans):
    """全域重排只動時間戳：航點 x/y/z、能量、盤旋圈數逐點相同，每機整段平移同一個量。"""
    for u, p in plans.items():
        q = new[u]
        assert q.orbits_inserted == p.orbits_inserted == 0
        assert len(q.waypoints) == len(p.waypoints)
        d = q.t_takeoff_s - p.t_takeoff_s
        for a, b in zip(p.waypoints, q.waypoints):
            assert (b.x_m, b.y_m, b.z_m, b.energy_remaining_wh) == (a.x_m, a.y_m, a.z_m, a.energy_remaining_wh)
            assert b.t_s == pytest.approx(a.t_s + d)
        assert q.energy_margin_wh == p.energy_margin_wh


# ---------------------------------------------------------------------------


def test_scenario_really_has_the_adjacent_pad_conflict():
    """⭐ 前置斷言：情境真的造出「相鄰兩坪、間隔 90 s」那一筆，而且是不同站（不被走廊豁免、走廊表也不管）。"""
    plans = _pads()
    cs = check_separation(plans, CFG)
    assert [(c.kind, c.uav_a, c.uav_b) for c in cs] == [("4d", "B2", "B3")]
    c = cs[0]
    assert c.d_h_m < 100.0 and c.d_v_m < 50.0
    assert 80.0 <= c.dt_ab_s < CFG.dt_4d_s + 2 * D.SIGMA_T_MAX_S
    sch = CorridorSchedule(CFG)
    for p in plans.values():
        sch.reserve_plan(p, p.takeoff_site)
    assert sch.conflicts() == []


def test_old_gate_leaves_the_residual(mini_ctx, old_gate):
    """⭐ 前置斷言：舊閘門下（貪婪拿掉）殘留原封不動 —— 下一條的「全綠」才有意義。"""
    plans = _pads()
    _cs, new, residual, _sch = _repair(mini_ctx, plans, max_rounds=0)
    assert [(c.kind, c.uav_a, c.uav_b) for c in residual] == [("4d", "B2", "B3")]
    assert all(new[u] is plans[u] for u in plans)


def test_residual_is_resolved_by_coordinated_reschedule(mini_ctx):
    plans = _pads()
    _cs, new, residual, sch = _repair(mini_ctx, plans, max_rounds=0)
    assert residual == []
    assert check_separation(new, CFG) == [] and sch.conflicts() == []
    _only_time_shifted(new, plans)
    # makespan 由遠處的 A1（t=0 起飛、不能再提前）釘死在 3000 s ⇒ 全綠解的 makespan **等於**現況。
    #   舊的「目標要嚴格改善」在這裡會把它退掉 ⇒ 這一條也咬 `incumbent = ∞`，不只咬閘門。
    assert max(p.t_land_s for p in new.values()) == max(p.t_land_s for p in plans.values()) == 3000.0
    assert all(p.t_takeoff_s >= 0.0 for p in new.values())
    # 兩架 HERO 的降落真的拉開到門檻以外（不是靠別的機對抵銷）
    assert abs(new["B3"].t_land_s - new["B2"].t_land_s) >= CFG.dt_4d_s + 2 * D.SIGMA_T_MAX_S
    note = [w for w in new["B2"].warnings if w.startswith("residual_resolved_by_reschedule(")]
    assert len(note) == 1 and "cap=kept" in note[0] and "makespan 3000.0->3000.0s" in note[0]


def test_global_reschedule_off_still_means_off(mini_ctx):
    """``global_reschedule=False`` 仍是總開關：殘留留著（逐位元退回上游行為的那條路不能被這次修改打開）。"""
    plans = _pads()
    cfg = dataclasses.replace(CFG, global_reschedule=False)
    _cs, new, residual, _sch = _repair(mini_ctx, plans, cfg, max_rounds=0)
    assert [(c.uav_a, c.uav_b) for c in residual] == [("B2", "B3")]
    assert all(new[u] is plans[u] for u in plans)


def test_scan_objective_accepts_a_green_plan_that_scans_later(mini_ctx):
    """以掃描完成為目標：B3 掃得最晚（1100 s），任何全綠解都得把 B2 或 B3 往後推 ⇒ 掃描完成只會更晚。
    現況是紅的 ⇒ 仍要選全綠解（``incumbent`` 從 ∞ 起算），不能因為「目標沒有變好」就退回紅的。"""
    plans = _pads()
    cfg = dataclasses.replace(CFG, reschedule_objective="scan")
    scan_now = max(D.scan_done_s(p) for p in plans.values())
    assert scan_now == pytest.approx(1100.0)
    _cs, new, residual, sch = _repair(mini_ctx, plans, cfg, max_rounds=0)
    assert residual == [] and check_separation(new, cfg) == [] and sch.conflicts() == []
    assert max(D.scan_done_s(p) for p in new.values()) > scan_now        # 前提成立：全綠解確實掃得比較晚
    _only_time_shifted(new, plans)


def test_no_green_within_current_makespan_relaxes_the_cap_once(mini_ctx):
    """沒有 A1 ⇒ 落地上界就是 B3 的 1500 s，而兩機都在 t=0 起飛、誰都不能提前 ⇒ 現況上界內沒有全綠解。
    第二段把上界放寬一趟最長架次：全綠、只動時間、makespan 變長，並在被動到的計畫留下註記。"""
    plans = _pads(with_a1=False)
    _cs, new, residual, sch = _repair(mini_ctx, plans, max_rounds=0)
    assert residual == [] and check_separation(new, CFG) == [] and sch.conflicts() == []
    _only_time_shifted(new, plans)
    ms_old = max(p.t_land_s for p in plans.values())
    ms_new = max(p.t_land_s for p in new.values())
    assert ms_old == 1500.0 < ms_new <= ms_old + max(p.t_land_s - p.t_takeoff_s for p in plans.values())
    notes = {u: [w for w in p.warnings if w.startswith("residual_resolved_by_reschedule(")] for u, p in new.items()}
    assert all(len(n) == 1 and "cap=relaxed" in n[0] for n in notes.values()), notes
    assert f"makespan {ms_old:.1f}->{ms_new:.1f}s" in notes["B2"][0]


def test_relaxed_cap_old_gate_leaves_the_residual(mini_ctx, old_gate):
    """⭐ 前置斷言（上一條）：舊閘門下同一個情境殘留原封不動。"""
    plans = _pads(with_a1=False)
    _cs, new, residual, _sch = _repair(mini_ctx, plans, max_rounds=0)
    assert [(c.kind, c.uav_a, c.uav_b) for c in residual] == [("4d", "B2", "B3")]
    assert all(new[u] is plans[u] for u in plans)


def test_unsolvable_residual_keeps_every_plan_and_reports(mini_ctx):
    """兩架 HERO **都已起飛**（t_now = 100 s）⇒ 只能盤旋，而 5 Wh 餘裕連一圈都不夠。兩架遠處的地面機
    正好在 t_now 起飛（仍可動、但不能再提前）、與衝突無關。重排擺得出這兩架，可是套用前的全機對驗證
    仍看到 B2/B3（兩架已起飛者之間的衝突，擺放時從不檢查）⇒ 整個退回：每一份計畫都是原物件
    （沒有白改、沒有註記），殘留照舊回報、不被吞掉。"""
    t_now = 100.0
    plans = _pads(with_a1=False)
    for uid, xy, out in (("A1", (6000.0, 6000.0), (1.0, 0.0)), ("A2", (-6000.0, -6000.0), (-1.0, 0.0))):
        plans[uid] = _sortie(uid, "A", to_pad=xy, to_site=f"FAR_{uid}", pad=xy, site=f"FAR_{uid}", t0=t_now,
                             t_land=3000.0, out=out, t_scan_end=600.0)
    models = dict(_models(), A2=EnergyModel(PLATFORM_PIONEER, 24.6, 1350.0, 1500.0, 1500.0))
    sch = CorridorSchedule(CFG)
    cs = check_separation(plans, CFG)
    assert [(c.uav_a, c.uav_b) for c in cs] == [("B2", "B3")]
    new, residual = repair(plans, cs, mini_ctx, models, sch, CFG, t_now_s=t_now,
                           takeoff_site={u: p.takeoff_site for u, p in plans.items()})
    assert [(c.kind, c.uav_a, c.uav_b) for c in residual] == [("4d", "B2", "B3")]
    assert all(new[u] is plans[u] for u in plans)


def test_default_rounds_still_green_and_time_only(mini_ctx):
    """預設輪數：貪婪先跑（這個情境貪婪自己就解得掉）。結果全綠、只動時間。
    ⚠ 這條**不分辨**修改前後，只防這次修改把貪婪那條路弄壞。"""
    plans = _pads()
    _cs, new, residual, sch = _repair(mini_ctx, plans)
    assert residual == [] and check_separation(new, CFG) == [] and sch.conflicts() == []
    _only_time_shifted(new, plans)


# ---------------------------------------------------------------------------
# 報告：收尾重排有出手 ⇒ 規劃器警告 reschedule_resolved_residual(...)（UAVPlan.warnings 不會進 feasibility）
# ---------------------------------------------------------------------------

_RESCHED = "reschedule_resolved_residual("
_PER_PLAN = "residual_resolved_by_reschedule("


def test_planner_note_kept_cap_value_lock(mini_ctx):
    """上界沒放寬：三架都可動 ⇒ 三架都參與；makespan 被 A1 釘在 3000 s ⇒ ±0。"""
    from hv_dccpp.planner import _reschedule_resolved_note
    _cs, new, residual, _sch = _repair(mini_ctx, _pads(), max_rounds=0)
    assert residual == []
    assert _reschedule_resolved_note(new) == (
        "reschedule_resolved_residual(4D 殘留衝突由收尾全域重排解掉（vendored #11）："
        "makespan 3000.0->3000.0 s（+0.0 s，+0.0%；前者是貪婪修復後仍有殘留、不能上傳的那份），"
        "cap=kept（落地上界維持原 makespan）；參與重排：A1、B2、B3)")


def test_planner_note_relaxed_cap_says_how_much_longer(mini_ctx):
    """上界放寬：工期比紅的那份長 ⇒ 前後值、差值、百分比與「工期拉長」都要寫出來。"""
    from hv_dccpp.planner import _reschedule_resolved_note
    plans = _pads(with_a1=False)
    _cs, new, residual, _sch = _repair(mini_ctx, plans, max_rounds=0)
    assert residual == []
    a, b = max(p.t_land_s for p in plans.values()), max(p.t_land_s for p in new.values())
    assert a == 1500.0 < b
    note = _reschedule_resolved_note(new)
    assert note == ("reschedule_resolved_residual(4D 殘留衝突由收尾全域重排解掉（vendored #11）："
                    f"makespan {a:.1f}->{b:.1f} s（{b - a:+.1f} s，{100.0 * (b - a) / a:+.1f}%；"
                    "前者是貪婪修復後仍有殘留、不能上傳的那份），"
                    "cap=relaxed（原工期內排不出全綠 ⇒ 落地上界放寬一趟最長架次、工期拉長）；參與重排：B2、B3)")


def test_planner_note_absent_when_reschedule_did_not_act(mini_ctx, old_gate):
    """有殘留但收尾重排沒出手（舊閘門）⇒ 沒有註記、沒有警告。"""
    from hv_dccpp.planner import _reschedule_resolved_note
    _cs, new, residual, _sch = _repair(mini_ctx, _pads(), max_rounds=0)
    assert residual != [] and _reschedule_resolved_note(new) is None


def test_planner_note_absent_when_greedy_solved_it(mini_ctx):
    """貪婪自己解掉（沒有殘留）⇒ 收尾走 resolve=False ⇒ 沒有註記、沒有警告。"""
    from hv_dccpp.planner import _reschedule_resolved_note
    _cs, new, residual, _sch = _repair(mini_ctx, _pads())
    assert residual == []
    assert not any(str(w).startswith(_PER_PLAN) for p in new.values() for w in p.warnings)
    assert _reschedule_resolved_note(new) is None


def test_planner_note_unknown_format_is_passed_through_not_dropped():
    """逐機註記的格式日後被改掉時，彙整不能靜默吞掉：原樣帶出。"""
    from hv_dccpp.planner import _reschedule_resolved_note
    p = _pads()["B2"]
    odd = dataclasses.replace(p, warnings=["residual_resolved_by_reschedule(格式改了)"])
    assert _reschedule_resolved_note({"B2": odd, "B3": _pads()["B3"]}) == (
        "reschedule_resolved_residual(4D 殘留衝突由收尾全域重排解掉（vendored #11）："
        "residual_resolved_by_reschedule(格式改了)；參與重排：B2)")


def test_plan_without_residual_has_no_reschedule_warning():
    """整條規劃器路徑、沒有殘留的情境（mini 範例）：報告裡沒有這條警告、計畫上也沒有逐機註記。"""
    from hv_dccpp.planner import plan_mission
    mp = plan_mission(*build_mini_example())
    assert not any(str(w).startswith("residual_conflicts(") for w in mp.feasibility.warnings)
    assert not any(str(w).startswith(_RESCHED) for w in mp.feasibility.warnings)
    assert not any(str(w).startswith(_PER_PLAN) for p in mp.plans.values() for w in p.warnings)


# ---------------------------------------------------------------------------
# 真實首飛情境（慢：生產 GUI 路徑，含回收站自動指派的三次規劃）
# ---------------------------------------------------------------------------

_SURVEY_ENV = {"HV_MAPPING": "1", "HV_CAMERA": "hq6", "HV_SHUTTER_HZ": "2", "HV_SIDE_OVERLAP": "0.7",
               "HV_FWD_OVERLAP": "0.8"}


def _first_flight(monkeypatch, basis, payloads: list | None = None):
    """``gui_legal_120_1a2b_pw3``（規劃風 3 m/s 來自 270°）＋建圖旋鈕，偏流基準 ``basis``；回傳 (plan, validate)。

    ``payloads``：給一個 list 就依**到達順序**記下 SwarmCoordinator 送出的每一則 report（GUI 端是逐則累加
    loud note，不是 ``_plan_gui`` 那種 ``dict.update``）。"""
    import hv_dccpp as hvlib
    runner = pytest.importorskip("scripts.hv_sitl_run")
    scenarios = pytest.importorskip("scripts.hv_sitl_scenarios")
    monkeypatch.delenv("HV_RECOVERY_AUTO", raising=False)
    knobs = runner.survey_knobs(_SURVEY_ENV)
    orig = runner.SurveyKnobs.mapping_config
    monkeypatch.setattr(runner.SurveyKnobs, "mapping_config",
                        lambda self: dataclasses.replace(orig(self), crab_margin_basis=basis))
    if payloads is not None:
        from mission.swarm_coordinator import SwarmCoordinator
        real = SwarmCoordinator.plan_coverage_dccpp

        def tee(self, *a, **kw):
            sink = self.hv_on_report

            def rec(p):
                payloads.append(dict(p))
                if sink is not None:
                    sink(p)
            self.hv_on_report = rec
            return real(self, *a, **kw)
        monkeypatch.setattr(SwarmCoordinator, "plan_coverage_dccpp", tee)
    sc = scenarios.SCENARIOS["gui_legal_120_1a2b_pw3"]
    assert tuple(sc.wind) == (3.0, 270.0)
    hv, _report = runner._plan_gui(runner.apply_survey_knobs(sc, knobs), knobs)
    plan, ctx, cfg = hv.last_plan, hv.last_ctx, hv.cfg
    return plan, hvlib.validate(plan, ctx, hvlib.build_validation_cache(plan, ctx, budget_only=False), cfg)


@pytest.fixture(scope="module")
def first_flight_xw():
    """首飛情境、crosswind 基準（改後）：只規劃一次（約 1 分鐘），回傳 (plan, validate, 依序的 report)。"""
    payloads: list = []
    with pytest.MonkeyPatch.context() as mp:
        plan, v = _first_flight(mp, "crosswind", payloads)
    return plan, v, payloads


def test_first_flight_crosswind_old_gate_has_the_b2_b3_residual(monkeypatch, old_gate):
    """⭐ 前置斷言（修改前的行為）：B2/B3 降在相距 10 m 的自己的坪、只隔 90 s，validate 擋下。"""
    plan, v = _first_flight(monkeypatch, "crosswind")
    assert not v.passed
    assert [(c.kind, c.uav_a, c.uav_b) for c in v.conflicts] == [("4d", "B2", "B3")]
    c = v.conflicts[0]
    assert c.d_h_m == pytest.approx(10.0, abs=0.5) and c.dt_ab_s == pytest.approx(90.0, abs=1.0)
    assert {plan.plans[u].landing_site for u in ("B2", "B3")} == {"TO_B2", "TO_B3"}
    # 收尾重排沒出手（舊閘門）⇒ 報告不能說它出手過
    assert not any(str(w).startswith(_RESCHED) for w in plan.feasibility.warnings)
    assert not any(str(w).startswith(_PER_PLAN) for p in plan.plans.values() for w in p.warnings)


def test_first_flight_crosswind_plan_validates(first_flight_xw):
    """首飛情境、crosswind 基準：validate 通過（評估 2026-10-20 第 5.4 節切換預設的第 1 條）。"""
    plan, v, _payloads = first_flight_xw
    assert v.passed and v.conflicts == []
    assert plan.uncovered == []
    assert {plan.plans[u].landing_site for u in ("B2", "B3")} == {"TO_B2", "TO_B3"}     # 仍降回自己的坪


# 首飛情境（crosswind）收尾重排的值鎖：紅的那份 2697.9 s（= 修改前的 makespan，見
# runs/artifacts/RESIDUAL_4D_2026-10-21/matrix_compare.txt 的 mx_new_s3_d270）→ 全綠 2671.0 s，上界沒放寬
_FIRST_FLIGHT_NOTE = ("reschedule_resolved_residual(4D 殘留衝突由收尾全域重排解掉（vendored #11）："
                      "makespan 2697.9->2671.0 s（-26.9 s，-1.0%；前者是貪婪修復後仍有殘留、不能上傳的那份），"
                      "cap=kept（落地上界維持原 makespan）；參與重排：A1、B2、B3)")


def test_first_flight_report_says_the_reschedule_acted(first_flight_xw):
    """⭐ GUI 看得到：規劃器警告 → 轉接層 report（warnings 與 notes 原樣）→ MainWindow 的 loud note 與偵察頁狀態字。"""
    plan, _v, payloads = first_flight_xw
    assert [w for w in plan.feasibility.warnings if str(w).startswith(_RESCHED)] == [_FIRST_FLIGHT_NOTE]
    assert plan.makespan_s == pytest.approx(2671.0, abs=0.05)                  # 「後」＝最終計畫的 makespan
    adapter = [p for p in payloads if "status" in p]                            # 轉接層那份（勝出那次規劃）
    assert len(adapter) == 1, [sorted(p) for p in payloads]
    assert _FIRST_FLIGHT_NOTE in adapter[0]["warnings"]
    assert _FIRST_FLIGHT_NOTE in adapter[0]["notes"]                            # 原樣（不是 preflight: 前綴）
    try:
        import types

        from ui.main_window import MainWindow, harvest_report_lines
    except Exception as e:                                                      # pragma: no cover
        pytest.skip(f"Qt 匯入失敗：{e}")
    w = types.SimpleNamespace(_HV_LOUD_NOTES=MainWindow._HV_LOUD_NOTES, _hv_last_report={})
    for p in payloads:                                                          # 前置 → 轉接層 → 尾端，依到達順序
        MainWindow._on_hv_report(w, p)
    assert _FIRST_FLIGHT_NOTE in w._hv_last_report["loud_notes"]               # 尾端那則沒有把它蓋掉
    assert f"HARVEST：{_FIRST_FLIGHT_NOTE}" in harvest_report_lines(w._hv_last_report)
