"""動態重規劃 × 建圖模式交叉航帶（dynamic._Session.reassign / relay_cross）— 數值鎖。

規則（2026-10-15 定）：交叉航帶（``geometry.is_cross_strip``，id 內嵌原機 ``geometry.cross_strip_owner``）設計上只由
**原機**緊接自己的主條帶飛。重規劃時

* 只准原機拿回，且原機不得是幽靈（失鏈中）；其他機一律不接，同構型也不行；
* 永不跨構型重鋪；重鋪也不得把它算成「已覆蓋」（舊碼 covered_src 會靜默吞掉）；
* 沒人接 → ``(sid, "cross_not_reassigned")`` 進 uncovered ＋ 每原機一筆 ``cross_not_reassigned(...)`` note；
* 沒有交叉帶時閘門完全不作用（tests/hv_dccpp/test_dynamic.py 不改仍全過；本檔另以「交叉帶拿掉」的對照組鎖主條帶逐值相同）。

審查修正（2026-10-15 晚）：

* M1：閘門只在 ``mapping_mode.cross_strips=True`` 時開（`dynamic._xstrip_gate`）；建圖模式關閉時含 ``_cross_`` 的 id
  （繞過 AreaSpec 驗證的舊 area_id，如 ``"river_cross_1"``）照主條帶處理 —— 與「閘門拿掉」逐值相同。AreaSpec／NewArea
  拒收含保留字串的 area_id（MissionInputError；AreaSpec 本身的鎖在 test_mission.py）。
* M2：`reassign` 分兩段（主條帶段與「交叉帶不存在」逐值相同 → 交叉帶段只還原機）；原機拿回時 cross_first 只有在
  不多少飛主條帶時才選 ⇒ 能量吃緊時退的是交叉帶。
* L1：``cross_allow_neighbor_airspace=False`` 時，母區塊主條帶已在他機手上（同構型接手／跨構型重鋪）⇒ 原機不拿回，
  cause ＝ ``neighbor_airspace(<他機>)``；放行時照舊拿回。
* L2：note 的 ``owner=<狀態>`` 照留（格式寫在 `dynamic.XSTRIP_NOT_REASSIGNED`）；同一 session 內 (原機, ids, cause) 相同只記一筆。

情境（與 test_cross_strips.py 同一套輸入、同一個明確放行設定 ON）：

* ``x1`` = ``_plan(ON)``（1×1 km 單區）：實際只有 **A1、B3** 起飛（B2 在機隊裡但沒有計畫）；A1 的交叉帶排在主條帶
  **之前**、B3 的在**之後**。A1 失鏈時 B3 以跨構型重鋪接 A1 的主條帶。
* ``x2`` = ``_inputs(ON, km=1.0, split=2)``（1×1 km 切兩區）：**A1、B2、B3 三架都飛、三架都有交叉帶**（都排在主條帶
  之前）。B2 失鏈時同構型的 B3 會接 —— 舊碼連 B2 的交叉帶（含殘段 ``#r``）一起原樣交給 B3。

每個測試都用新的 ctx／cache／計畫深拷貝（handler 會就地改 cache、ctx.models.κ、PriorityChange 會改 cell.priority）。
"""
from __future__ import annotations

import copy
import dataclasses
import time
from types import SimpleNamespace

import pytest

from hv_dccpp import dynamic as dyn
from hv_dccpp import geometry as G
from hv_dccpp.context import build_context
from hv_dccpp.geometry import cross_strip_id, cross_strip_owner, is_cross_strip
from hv_dccpp.mission import AreaSpec, MappingModeConfig, MissionInputError
from hv_dccpp.planner import plan_mission
from hv_dccpp.scenario import build_mini_example
from hv_dccpp.sim import check_mission_integrity

from .test_cross_strips import FORBID, ON, _plan
from .test_dynamic import MARGIN, STEP, _check_incremental
from .test_mapping_mode import CFG, _inputs

XNR = "cross_not_reassigned"
#: 與 ON 只差「鄰機空域＝禁止（預設）」：同一份計畫（ON 規劃出來、三架都有交叉帶）在禁止模式下動態重規劃
ON_FORBID = dataclasses.replace(ON, cross_allow_neighbor_airspace=False)


# ---------------------------------------------------------------------------
# fixtures / helpers
# ---------------------------------------------------------------------------
@pytest.fixture(scope="module")
def x1():
    plan, inp = _plan(ON)
    assert plan.feasibility.status != "NO_GO" and plan.uncovered == []
    assert {"A1", "B3"} <= set(plan.plans), sorted(plan.plans)
    return plan, inp


@pytest.fixture(scope="module")
def x2():
    inp = _inputs(ON, km=1.0, split=2)
    plan = plan_mission(inp, CFG)
    assert plan.feasibility.status != "NO_GO" and plan.uncovered == []
    assert set(plan.plans) == {"A1", "B2", "B3"}, sorted(plan.plans)
    for u, up in plan.plans.items():
        assert any(is_cross_strip(s) for s, _d in up.strips), u                 # 三架都有交叉帶
    return plan, inp


def _fresh(fx, mm=None):
    """計畫深拷貝 + 新 ctx + 新 cache（handler 會就地改這三樣）。``mm`` ≠ None ⇒ ctx 換成這個建圖模式設定（計畫不變）。"""
    plan0, inp = fx
    plan = copy.deepcopy(plan0)
    if mm is not None:
        inp = dataclasses.replace(inp, mapping_mode=mm)
    ctx = build_context(inp, CFG)
    return plan, ctx, dyn.cache_from_plan(plan, ctx)


def _xspans(plan, u):
    return [s for s in dyn._spans(plan.plans[u]) if is_cross_strip(s.strip_id)]


def _mspans(plan, u):
    return [s for s in dyn._spans(plan.plans[u]) if not is_cross_strip(s.strip_id)]


def _mid(span):
    return 0.5 * (span.t0 + span.t_end)


def _ids(p):
    """該機計畫裡所有條帶 id（strips 清單 ∪ 航點上的 strip_id）。"""
    return [s for s, _d in p.strips] + [w.strip_id for w in p.waypoints if w.strip_id is not None]


def _assert_owner_rule(plan):
    """全域不變量：任何一架的計畫裡只有它自己的交叉帶（含殘段 #r）。"""
    for v, p in plan.plans.items():
        bad = sorted({s for s in _ids(p) if is_cross_strip(s) and cross_strip_owner(s) != v})
        assert not bad, (v, bad)


def _runs(flags):
    return sum(1 for i, f in enumerate(flags) if f and (i == 0 or not flags[i - 1]))


def _not_flown(plan, ids):
    bad = {f"strip_not_flown({s})" for s in ids}
    return [x for x in check_mission_integrity(plan) if x in bad]


def _wp_sig(p):
    return [(w.phase, w.strip_id, w.x_m, w.y_m, w.z_m, w.t_s, w.energy_remaining_wh) for w in p.waypoints]


# ---------------------------------------------------------------------------
# (d) geometry.cross_strip_owner
# ---------------------------------------------------------------------------
def test_cross_strip_owner_parses_ids():
    assert "cross_strip_owner" in G.__all__
    sid = cross_strip_id("Z1_1_B_k10", "B", "B3", 4)
    assert sid == "Z1_1_B_k10_B_cross_B3_4" and cross_strip_owner(sid) == "B3"
    assert cross_strip_owner(sid + "#r") == "B3"                                 # 殘段：去掉 #r 後同一原機
    assert cross_strip_owner(cross_strip_id("Z1_1", "A", "A1", 12)) == "A1"
    assert cross_strip_owner(cross_strip_id("Z1_1", "B", "UAV_7", 2)) == "UAV_7"   # 機號含底線
    assert cross_strip_owner(SimpleNamespace(strip_id=sid)) == "B3"
    for non in ("Z1_1_A_4", "Z1_1_A_4#r", "Z1_1_R1_B_3", "", None):
        assert cross_strip_owner(non) is None, non
    # 含標記但格式不合：仍是交叉帶、原機未知 ⇒ None（呼叫端當「沒人能接」，不是當主條帶放行）
    for bad in ("Z1_1_A_cross_A1", "Z1_1_A_cross_A1_x", "Z1_1_A_cross__3", "Z1_1_A_cross_A1_²"):
        assert is_cross_strip(bad) and cross_strip_owner(bad) is None, bad
    # 第一個標記才是 cross_strip_id 加的（母胞 id 不可能含標記）⇒ 機號本身含標記也解得對（舊：取最後一個 → "2"）
    odd = cross_strip_id("Z1_1", "B", "U_cross_2", 3)
    assert odd == "Z1_1_B_cross_U_cross_2_3" and cross_strip_owner(odd) == "U_cross_2"
    assert cross_strip_owner(odd + "#r") == "U_cross_2"
    # 母胞 id 含保留字串（繞過 AreaSpec 驗證）⇒ 不產生無法辨識原機的 id
    for cell in ("river_cross_1_1", "river_cross"):                        # 後者 + "_A_cross_…" 會先組出一個標記
        with pytest.raises(ValueError, match="_cross_"):
            cross_strip_id(cell, "A", "A1", 1)


def test_cross_strip_owner_matches_the_flying_uav_in_real_plans(x1, x2):
    for plan, _inp in (x1, x2):
        n = 0
        for u, up in plan.plans.items():
            for s, _d in up.strips:
                if is_cross_strip(s):
                    assert cross_strip_owner(s) == u, (u, s)
                    n += 1
                else:
                    assert cross_strip_owner(s) is None, s
        assert n >= 8


# ---------------------------------------------------------------------------
# (a) 失鏈：同構型的 B3 不得接 B2 的交叉帶（舊碼：全部含 #r 原樣交給 B3）
# ---------------------------------------------------------------------------
def test_link_loss_never_hands_cross_strips_to_same_class_uav(x2):
    assert dyn.XSTRIP_NOT_REASSIGNED == XNR == "cross_not_reassigned"
    plan, ctx, cache = _fresh(x2)
    xs = _xspans(plan, "B2")
    t = _mid(xs[1])                                          # B2 第 2 條交叉帶中點
    assert ctx.inputs.c2.policy_for("B2", "B") == "RTB"      # ⇒ 決策點 = t_loss
    st = dyn._status(plan.plans["B2"], t)
    assert st.current is not None and is_cross_strip(st.current.strip_id)
    exp = [sp.strip_id for sp in st.pending if is_cross_strip(sp.strip_id)] + [st.current.strip_id + "#r"]
    assert len(exp) == 3 and all(cross_strip_owner(s) == "B2" for s in exp)
    mains_pending = [sp.strip_id for sp in st.pending if not is_cross_strip(sp.strip_id)]
    assert mains_pending

    res = dyn.replan(plan, dyn.LinkLoss("B2", t), {}, ctx, cache, CFG)
    _check_incremental(plan, res)
    _assert_owner_rule(res.plan)
    for v in ("A1", "B3"):
        assert not [s for s in _ids(res.plan.plans[v]) if cross_strip_owner(s) == "B2"], v
    # 非空轉：B3 確實接了 B2 的主條帶（同構型接手路徑真的跑到），而且只接主條帶
    new_b3 = [s for s, _d in res.plan.plans["B3"].strips if s not in {x for x, _ in plan.plans["B3"].strips}]
    assert new_b3 and set(new_b3) <= set(mains_pending)
    # A1（跨構型候選）不得把 B2 的交叉帶拿去重鋪：只有失鏈機本身與接主條帶的 B3 被改
    assert res.changed_uavs == ["B2", "B3"] and res.plan.plans["A1"] is plan.plans["A1"]
    # 每條被擋的交叉帶都在 uncovered，理由固定、只有這一個
    unc = set(res.plan.uncovered)
    assert all((s, XNR) in unc for s in exp)
    assert sorted(r for s, r in res.plan.uncovered if s in exp) == [XNR] * 3
    # 每原機一筆 note，逐字
    notes = [n for n in res.notes if n.startswith(XNR + "(")]
    assert notes == [f"{XNR}(B2: {exp}; cause=uncoverable_after_link_loss; owner=ghost)"]
    assert _not_flown(res.plan, exp) == []


# ---------------------------------------------------------------------------
# (a') 跨構型重鋪不得吞掉交叉帶（舊碼：covered_src 標成已覆蓋 ⇒ 不在 uncovered、strip_not_flown）
# ---------------------------------------------------------------------------
def test_relay_never_swallows_or_relays_cross_strips(x1):
    plan, ctx, cache = _fresh(x1)
    xs = _xspans(plan, "A1")
    assert _mspans(plan, "A1")[0].t0 > xs[-1].t_end          # x1 的 A1：交叉帶整段排在主條帶之前
    t = _mid(xs[1])
    res = dyn.replan(plan, dyn.LinkLoss("A1", t), {}, ctx, cache, CFG)
    _check_incremental(plan, res)
    g = res.ghosts[0]
    assert g.policy == "CONTINUE_THEN_RTB" and g.t_rtb_s > t
    st = dyn._status(plan.plans["A1"], g.t_rtb_s)
    assert st.current is None                                 # 決策點落在兩條交叉帶之間 ⇒ 無殘段
    exp = [sp.strip_id for sp in st.pending if is_cross_strip(sp.strip_id)]
    mains = [sp.strip_id for sp in st.pending if not is_cross_strip(sp.strip_id)]
    assert len(exp) == 2 and mains
    # 非空轉：B3 以跨構型重鋪接了 A1 的主條帶；重鋪來源（note）只有主條帶
    relaid = [n for n in res.notes if n.startswith("relaid(")]
    assert len(relaid) == 1 and relaid[0].startswith(f"relaid({','.join(mains)} -> B:") and relaid[0].endswith(" strips)")
    assert not any(s in relaid[0] for s in exp)
    new_b3 = [s for s, _d in res.plan.plans["B3"].strips if s not in {x for x, _ in plan.plans["B3"].strips}]
    assert new_b3 and all(res.plan.strips[s].cls == "B" and not is_cross_strip(s) for s in new_b3)
    _assert_owner_rule(res.plan)
    assert all((s, XNR) in set(res.plan.uncovered) for s in exp)
    assert [n for n in res.notes if n.startswith(XNR + "(")] == \
        [f"{XNR}(A1: {exp}; cause=uncoverable_after_link_loss; owner=ghost)"]
    assert _not_flown(res.plan, exp) == []                    # 舊碼：strip_not_flown(Z1_1_A_cross_A1_4/_2)


def test_relay_cross_skips_cross_strips_without_side_effects(x1):
    plan, ctx, cache = _fresh(x1)
    t = _mid(_xspans(plan, "A1")[1])
    sess = dyn._Session(plan, {}, ctx, cache, CFG, t, None)
    xids = [s.strip_id for s in _xspans(plan, "A1")]
    n_b = len(cache.idx["B"])
    cell, out = sess.relay_cross(xids, "B", ctx.uav("B3"), 0)
    assert (cell, out) == (None, []) and sess.new_strips == {} and sess.notes == []
    assert len(cache.idx["B"]) == n_b                          # 沒碰快取
    # 對照：同一顆 B3、同一個胞的主條帶照樣能重鋪（不是因為別的原因回空）
    cell2, out2 = sess.relay_cross([s.strip_id for s in _mspans(plan, "A1")], "B", ctx.uav("B3"), 0)
    assert cell2 is not None and out2 and not any(is_cross_strip(s.strip_id) for s in out2)


# ---------------------------------------------------------------------------
# (b) 主條帶逐值不變：與「交叉帶事先拿掉」的對照組比
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("key,lost_uav", [("x2", "B2"), ("x1", "A1")])
def test_main_strips_reassigned_exactly_as_without_cross_strips(request, monkeypatch, key, lost_uav):
    fx = request.getfixturevalue(key)
    t = _mid(_xspans(fx[0], lost_uav)[1])

    plan, ctx, cache = _fresh(fx)
    res = dyn.replan(plan, dyn.LinkLoss(lost_uav, t), {}, ctx, cache, CFG)

    orig = dyn._Session.reassign

    def _without_cross(self, strip_ids, *a, **k):          # 對照組：交叉帶根本不交給 reassign
        return orig(self, [s for s in strip_ids if not is_cross_strip(s)], *a, **k)

    monkeypatch.setattr(dyn._Session, "reassign", _without_cross)
    plan_r, ctx_r, cache_r = _fresh(fx)
    ref = dyn.replan(plan_r, dyn.LinkLoss(lost_uav, t), {}, ctx_r, cache_r, CFG)
    monkeypatch.undo()

    others = [v for v in ref.changed_uavs if v != lost_uav]
    assert others                                            # 非空轉：對照組裡真的有別架接了主條帶
    assert res.changed_uavs == ref.changed_uavs
    assert set(res.plan.plans) == set(ref.plan.plans)
    for v in res.plan.plans:
        assert res.plan.plans[v].strips == ref.plan.plans[v].strips, v
        assert _wp_sig(res.plan.plans[v]) == _wp_sig(ref.plan.plans[v]), v
    assert set(res.plan.strips) == set(ref.plan.strips)
    assert [x for x in res.plan.uncovered if not is_cross_strip(x[0])] == list(ref.plan.uncovered)
    skip = (XNR + "(", "uncovered_after_link_loss(")
    assert [n for n in res.notes if not n.startswith(skip)] == [n for n in ref.notes if not n.startswith(skip)]
    # 差別只有：交叉帶進了 uncovered（對照組裡它們被丟掉、根本沒人管）
    assert {s for s, r in res.plan.uncovered if is_cross_strip(s)} == \
        {s for s, r in res.plan.uncovered if r == XNR} != set()


# ---------------------------------------------------------------------------
# (c) 原機：失鏈中（幽靈）永遠拿不回；連得上就拿得回
# ---------------------------------------------------------------------------
def _loss_then_priority(fx, ghosts_on: bool):
    plan, ctx, cache = _fresh(fx)
    t = _mid(_xspans(plan, "B2")[1])
    res1 = dyn.replan(plan, dyn.LinkLoss("B2", t), {}, ctx, cache, CFG)
    blocked = [s for s, r in res1.plan.uncovered if r == XNR]
    area = next(c.area_id for c in res1.plan.cells if c.cell_id == res1.plan.strips[blocked[0]].cell_id)
    t2 = t + 20.0
    assert not dyn._status(res1.plan.plans["B2"], t2).landed                # B2 還在空中（返航中）⇒ 是候選
    res2 = dyn.replan(res1.plan, dyn.PriorityChange(area, 5.0, t2), {}, ctx, cache, CFG,
                      res1.ghosts if ghosts_on else None)
    return res1, res2, blocked


def test_ghost_owner_never_retakes_its_cross_strips(x2):
    res1, res2, blocked = _loss_then_priority(x2, ghosts_on=True)
    assert len(blocked) == 3 and [g.uav_id for g in res2.ghosts] == ["B2"]
    assert res2.plan.plans["B2"] is res1.plan.plans["B2"]                   # 幽靈：計畫一個字都沒動
    assert not set(blocked) & set(_ids(res2.plan.plans["B2"]))
    _assert_owner_rule(res2.plan)
    assert all((s, XNR) in set(res2.plan.uncovered) for s in blocked)
    assert [n for n in res2.notes if n.startswith(XNR + "(")] == [f"{XNR}(B2: {blocked}; cause=energy; owner=ghost)"]


def test_reachable_owner_retakes_its_cross_strips(x2):
    """同一個 PriorityChange，只差 B2 不在幽靈清單 ⇒ B2 拿回自己的交叉帶（含殘段），其他機仍一條都沒有。"""
    res1, res2, blocked = _loss_then_priority(x2, ghosts_on=False)
    _check_incremental(res1.plan, res2)
    assert "B2" in res2.changed_uavs
    b2 = res2.plan.plans["B2"]
    old = [s for s, _d in res1.plan.plans["B2"].strips]
    flown = set(_ids(res1.plan.plans["B2"]))                  # 含失鏈時飛到一半的那條（航點上有、strips 清單沒有）
    tail = [s for s, _d in b2.strips if s not in flown]
    assert [s for s, _d in b2.strips][: len(old)] == old and tail == [s for s, _d in b2.strips][-len(tail):]
    assert sorted(tail) == sorted(blocked)
    assert not [s for s, _r in res2.plan.uncovered if s in blocked]
    assert not [n for n in res2.notes if n.startswith(XNR + "(")]
    _assert_owner_rule(res2.plan)
    assert _not_flown(res2.plan, blocked) == []
    assert _runs([is_cross_strip(s) for s, _d in b2.strips]) == 1          # 接續原本那段交叉帶


def test_link_restored_owner_retakes_its_cross_strips(x2):
    plan, ctx, cache = _fresh(x2)
    t = _mid(_xspans(plan, "B2")[1])
    res1 = dyn.replan(plan, dyn.LinkLoss("B2", t), {}, ctx, cache, CFG)
    blocked = [s for s, r in res1.plan.uncovered if r == XNR]
    assert len(blocked) == 3
    t2 = t + 20.0
    s = dyn._interp_state(res1.plan.plans["B2"], t2, ctx.specs["B"], ctx.wind)
    tel = dyn.Telemetry("B2", t2, s.x_m, s.y_m, s.z_m, s.theta_rad, 22.0, None, None, 0.0, True)
    res2 = dyn.replan(res1.plan, dyn.LinkRestored("B2", t2, tel), {}, ctx, cache, CFG, res1.ghosts)
    _check_incremental(res1.plan, res2)
    assert res2.ghosts == [] and res2.changed_uavs == ["B2"]
    assert f"rejoin(B2: {sorted(blocked)})" in res2.notes
    b2 = [s for s, _d in res2.plan.plans["B2"].strips]
    assert set(blocked) <= set(b2) and _runs([is_cross_strip(x) for x in b2]) == 1
    assert not [x for x, _r in res2.plan.uncovered if x in blocked]
    _assert_owner_rule(res2.plan)
    assert _not_flown(res2.plan, blocked) == []


# ---------------------------------------------------------------------------
# 原機拿回時的排序：交叉帶在新尾段裡連成一段（session 層，原機是唯一候選）
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("u,where,x_first", [
    ("A1", "cross", True),       # x1 A1：交叉帶在前、打斷在交叉帶中 ⇒ 先接續交叉帶、再主條帶
    ("B3", "main", False),       # x1 B3：交叉帶在後、打斷在主條帶中 ⇒ 主條帶先、交叉帶接在後面
])
def test_owner_retake_keeps_cross_strips_one_contiguous_run(x1, u, where, x_first):
    plan, ctx, cache = _fresh(x1)
    span = (_xspans(plan, u)[1] if where == "cross" else _mspans(plan, u)[5])
    t = _mid(span)
    sess = dyn._Session(plan, {}, ctx, cache, CFG, t, None)
    stt = sess.status(u)
    pend = [sp.strip_id for sp in stt.pending]
    n_x = sum(map(is_cross_strip, pend))
    assert n_x >= 2 and len(pend) - n_x >= 2
    sess.rtb_now(u, sess.anchor_after_span(u, stt.current), "test_cut")    # 先把待掃全部切掉
    n_keep = len(sess.plans[u].strips)
    left = sess.reassign(pend, candidates=[u], reason="energy")
    assert left == [] and not [n for n in sess.notes if n.startswith(XNR + "(")]
    ids = [s for s, _d in sess.plans[u].strips]
    tail = ids[n_keep:]
    assert sorted(tail) == sorted(pend)
    flags = [is_cross_strip(s) for s in tail]
    n_m = len(tail) - n_x
    assert flags == ([True] * n_x + [False] * n_m if x_first else [False] * n_m + [True] * n_x), flags
    assert _runs([is_cross_strip(s) for s in ids]) == 1                    # 整架只有一段交叉帶
    # 主條帶段先接、交叉帶段把自己的交叉帶併進同一段尾段（第 1 段的 takeover 只列主條帶；排序寫在 cross_retaken）
    order = "cross_first" if x_first else "main_first"
    assert [n for n in sess.notes if n.startswith("takeover(")] and \
        all(not any(is_cross_strip(x) for x in n.split("[", 1)[1].split("]", 1)[0].split(", "))
            for n in sess.notes if n.startswith("takeover("))
    assert [n for n in sess.notes if n.startswith("cross_retaken(")] and \
        [n for n in sess.notes if n.startswith("cross_retaken(")][0].startswith(
            f"cross_retaken({u}: {[s for s in tail if is_cross_strip(s)]}; order={order}; ")
    _assert_owner_rule(SimpleNamespace(plans=dict(sess.plans)))


# ---------------------------------------------------------------------------
# note 的原機狀態：逐一鎖值（session 層直接呼叫 reassign）
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("case", ["not_candidate", "not_taken", "excluded", "ghost", "landed"])
def test_blocked_note_owner_state(x2, case):
    plan, ctx, cache = _fresh(x2)
    xs = _xspans(plan, "B2")
    t = _mid(xs[1])
    ghosts = None
    if case == "ghost":
        ghosts = [dyn.ghost_track("B2", plan, "RTB", t, ctx, CFG, cache=cache)]
    if case == "landed":
        t = plan.plans["B2"].t_land_s + 1.0                                   # B2 已落地（B3 更早落地）
    sess = dyn._Session(plan, {}, ctx, cache, CFG, t, ghosts)
    ids = [s.strip_id for s in xs]                                           # B2 的整組交叉帶
    kw = {"not_candidate": dict(candidates=["B3"]),                          # 同構型、能量夠，但不是原機
          "not_taken": dict(candidates=["B2"], max_changes=0),
          "excluded": dict(exclude={"B2"}, candidates=["B2", "B3"]),
          "ghost": dict(candidates=["B2", "B3"]),
          "landed": dict(candidates=["B2", "B3"])}[case]
    before = {u: p for u, p in sess.plans.items()}
    rem = sess.reassign(ids, reason="test_reason", **kw)
    assert rem == ids
    assert all(sess.plans[u] is before[u] for u in before)                   # 沒有任何一架被改
    assert sess.uncovered[-len(ids):] == [(s, XNR) for s in ids]
    assert [n for n in sess.notes if n.startswith(XNR + "(")] == [f"{XNR}(B2: {ids}; cause=test_reason; owner={case})"]


# ---------------------------------------------------------------------------
# 其他會呼叫 reassign 的事件也守同一條規則
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("kind", ["exit", "energy_divert"])
def test_other_events_respect_the_owner_rule(x2, kind):
    plan, ctx, cache = _fresh(x2)
    t = _mid(_xspans(plan, "B2")[1])
    st = dyn._status(plan.plans["B2"], t)
    exp = [sp.strip_id for sp in st.pending if is_cross_strip(sp.strip_id)] + [st.current.strip_id + "#r"]
    if kind == "exit":
        ev, cause = dyn.UAVExit("B2", t, "motor_failure"), "uncoverable_after_exit"
    else:
        ev, cause = dyn.EnergyUpdate("B2", t, 0.3 * dyn._plan_energy_at(plan.plans["B2"], t)), "energy"
    res = dyn.replan(plan, ev, {}, ctx, cache, CFG)
    _check_incremental(plan, res)
    if kind == "energy_divert":
        assert any(n.startswith("divert") for n in res.notes), res.notes
    assert "B3" in res.changed_uavs                                          # 非空轉：主條帶有人接
    _assert_owner_rule(res.plan)
    assert all((s, XNR) in set(res.plan.uncovered) for s in exp)
    assert [n for n in res.notes if n.startswith(XNR + "(")] == [f"{XNR}(B2: {exp}; cause={cause}; owner=excluded)"]
    assert _not_flown(res.plan, exp) == []


# ---------------------------------------------------------------------------
# M1：閘門只在建圖模式開交叉帶時作用；保留字串的 area_id 一律拒收
# ---------------------------------------------------------------------------
def test_xstrip_gate_reads_the_mapping_mode():
    gate = dyn._xstrip_gate
    assert gate(SimpleNamespace(mapping_mode=None)) == (False, False)
    assert gate(SimpleNamespace()) == (False, False)                          # 沒有這個屬性的 inputs ＝ 沒開建圖模式
    assert gate(SimpleNamespace(mapping_mode=MappingModeConfig())) == (False, False)   # 建圖模式開、交叉帶關
    assert gate(SimpleNamespace(mapping_mode=FORBID)) == (True, False)
    assert gate(SimpleNamespace(mapping_mode=ON)) == (True, True)


def test_session_computes_the_gate_once_from_ctx(x1):
    sid = next(s for s in x1[0].strips if is_cross_strip(s))
    for mm, exp in ((None, (True, True)), (ON_FORBID, (True, False)), (MappingModeConfig(), (False, False))):
        plan, ctx, cache = _fresh(x1, mm)
        sess = dyn._Session(plan, {}, ctx, cache, CFG, 0.0, None)
        assert (sess.xstrips_on, sess.xstrips_allow_neighbor) == exp, mm
        assert sess.is_x(sid) is exp[0] and sess.is_x(sid + "#r") is exp[0]
        assert sess.is_x("Z1_1_A_3") is False


@pytest.fixture(scope="module")
def mini_bypass():
    """建圖模式**關閉**的 mini 情境（test_dynamic.py 同一套），但 area_id 繞過 AreaSpec 驗證改成 ``"river_cross_1"``
    （模擬驗證加上之前的舊輸入）⇒ 每一條主條帶 id 都含 ``"_cross_"``（審查 M1 的重現情境）。"""
    inputs, cfg = build_mini_example()
    cfg = dataclasses.replace(cfg, dem_step_m=STEP)
    a0 = inputs.areas[0]
    with pytest.raises(MissionInputError, match="_cross_"):
        dataclasses.replace(a0, area_id="river_cross_1")                       # 正常路徑已經建不出來
    bad = copy.copy(a0)
    object.__setattr__(bad, "area_id", "river_cross_1")
    inputs = dataclasses.replace(inputs, areas=(bad,) + tuple(inputs.areas[1:]))
    assert inputs.mapping_mode is None
    ctx = build_context(inputs, cfg, terrain_margin_m=MARGIN)
    plan = plan_mission(inputs, cfg, ctx=ctx)
    assert plan.feasibility.status != "NO_GO" and set(plan.plans) == {"A1", "B1"}
    assert plan.strips and all(is_cross_strip(s) for s in plan.strips)      # 每一條主條帶都「長得像」交叉帶
    spans = dyn._spans(plan.plans["A1"])
    return inputs, cfg, plan, 0.5 * (spans[2].t0 + spans[2].t_end)


def _mini_replan(fx, event):
    inputs, cfg, plan0, _t = fx
    ctx = build_context(inputs, cfg, terrain_margin_m=MARGIN)
    plan = copy.deepcopy(plan0)
    return plan, dyn.replan(plan, event, {}, ctx, dyn.cache_from_plan(plan, ctx), cfg)


def test_gate_is_a_no_op_when_mapping_is_off(mini_bypass, monkeypatch):
    """舊閘門只看 id ⇒ A1 退出後 B1 一條都不准接（重鋪也被擋），全部記 cross_not_reassigned。建圖模式關閉時現在與
    「閘門整個拿掉」（dynamic 內的 is_cross_strip 恆 False）逐值相同。"""
    ev = dyn.UAVExit("A1", mini_bypass[3], "motor_failure")
    plan, res = _mini_replan(mini_bypass, ev)
    monkeypatch.setattr(dyn, "is_cross_strip", lambda _s: False)
    monkeypatch.setattr(dyn, "cross_strip_owner", lambda _s: None)
    _plan_r, ref = _mini_replan(mini_bypass, ev)
    monkeypatch.undo()
    _check_incremental(plan, res)
    assert res.changed_uavs == ref.changed_uavs and "B1" in res.changed_uavs    # 非空轉：B1 真的接了 A1 的條帶
    assert any(n.startswith(("relaid(", "takeover(B1")) for n in res.notes), res.notes
    assert set(res.plan.plans) == set(ref.plan.plans)
    for v in res.plan.plans:
        assert res.plan.plans[v].strips == ref.plan.plans[v].strips, v
        assert _wp_sig(res.plan.plans[v]) == _wp_sig(ref.plan.plans[v]), v
    assert set(res.plan.strips) == set(ref.plan.strips)
    assert res.plan.uncovered == ref.plan.uncovered
    skip = ("placeholder_",)
    assert [n for n in res.notes if not n.startswith(skip)] == [n for n in ref.notes if not n.startswith(skip)]
    assert not any(r == XNR for _s, r in res.plan.uncovered)
    assert not any(n.startswith((XNR, "cross_retaken")) for n in res.notes)


def test_new_area_and_cue_ids_with_the_reserved_tag_are_refused(mini_bypass):
    inputs, cfg, plan0, t = mini_bypass
    ctx = build_context(inputs, cfg, terrain_margin_m=MARGIN)
    cache = dyn.cache_from_plan(plan0, ctx)
    ring = inputs.areas[0].polygon_wgs84
    with pytest.raises(MissionInputError, match="_cross_"):
        AreaSpec("bridge_cross_1", ring)                                       # NewArea 的區本身就建不出來
    for aid in ("bridge_cross_1", "bridge_cross"):
        bad = copy.copy(AreaSpec("bridge-cross-1", ring))
        object.__setattr__(bad, "area_id", aid)                                # 繞過驗證的物件：handler 再擋一次
        with pytest.raises(MissionInputError, match=f"NewArea: area_id='{aid}'"):
            dyn.replan(plan0, dyn.NewArea(bad, t), {}, ctx, cache, cfg)
    # TargetCue 的確認區 id 是 f"CUE_{src_uav_id}_{t}"：機號含保留字串 ⇒ AreaSpec 大聲拒收（不靜默改名）
    with pytest.raises(MissionInputError, match="CUE_A1_cross_9_"):
        dyn.replan(plan0, dyn.TargetCue(ring[0], "A1_cross_9", 0.9, t), {}, ctx, cache, cfg)


# ---------------------------------------------------------------------------
# M2：原機拿回時，交叉帶永遠排擠不到主條帶（能量吃緊 ⇒ 退的是交叉帶）
# ---------------------------------------------------------------------------
def _tight_session(fx, u, kappa):
    """x1 的 ``u`` 在第 2 條交叉帶中點被切掉全部待掃（rtb_now），之後每次排程的錨點能量少 ``kappa`` Wh。"""
    plan, ctx, cache = _fresh(fx)
    t = _mid(_xspans(plan, u)[1])
    sess = dyn._Session(plan, {}, ctx, cache, CFG, t, None)
    stt = sess.status(u)
    pend = [sp.strip_id for sp in stt.pending]
    sess.rtb_now(u, sess.anchor_after_span(u, stt.current), "test_cut")
    real = sess.anchor_end

    def _tight(v):
        a = real(v)
        a.state = dataclasses.replace(a.state, e_rem_wh=float(a.state.e_rem_wh) - kappa)
        return a

    sess.anchor_end = _tight
    return sess, pend, len(sess.plans[u].strips)


@pytest.mark.parametrize("kappa", [500.0, 600.0])
def test_owner_retake_never_trades_main_strips_for_cross_strips(x1, kappa):
    u = "A1"
    sess, pend, n_keep = _tight_session(x1, u, kappa)
    xs = [s for s in pend if is_cross_strip(s)]
    ms = [s for s in pend if not is_cross_strip(s)]
    assert len(xs) == 2 and len(ms) == 10
    # 對照：只交主條帶（＝第 1 段單獨跑）；兩種順序直接排（舊碼 x_first 用的就是 [交叉帶, 主條帶]）
    ref, _p, _n = _tight_session(x1, u, kappa)
    ref_left = ref.reassign(ms, candidates=[u], reason="energy")
    ref_tail = [s for s, _d in ref.plans[u].strips][n_keep:]
    pm, _p, _n = _tight_session(x1, u, kappa)
    _seq, mf_left = pm.idp_chain(u, pm.anchor_end(u), [ms, xs])
    pc, _p, _n = _tight_session(x1, u, kappa)
    _seq, cf_left = pc.idp_chain(u, pc.anchor_end(u), [xs, ms])

    left = sess.reassign(pend, candidates=[u], reason="energy")
    tail = [s for s, _d in sess.plans[u].strips][n_keep:]
    main_left = [s for s in left if not is_cross_strip(s)]
    assert main_left == ref_left                                                  # 主條帶：與「沒有交叉帶」相同
    assert sorted(main_left) == sorted(s for s in mf_left if not is_cross_strip(s))   # 也就是 main-first 的結果
    assert sorted(s for s in tail if not is_cross_strip(s)) == sorted(ref_tail)
    retaken = [n for n in sess.notes if n.startswith("cross_retaken(")]
    blocked = [n for n in sess.notes if n.startswith(XNR + "(")]
    if kappa == 600.0:
        # 能量只夠 6 條主條帶：舊的 [交叉帶, 主條帶] 會為了 2 條交叉帶多丟 1 條主條帶（5 條）；現在丟的是交叉帶
        assert len(main_left) == 4 and sum(not is_cross_strip(s) for s in cf_left) == 5
        assert tail == ref_tail and not any(is_cross_strip(s) for s in tail)
        assert [s for s in left if is_cross_strip(s)] == xs
        assert sess.uncovered[-2:] == [(s, XNR) for s in xs] and retaken == []
        assert blocked == [f"{XNR}(A1: {xs}; cause=energy; owner=not_taken)"]
    else:
        # 主條帶兩種順序都放得下：[交叉帶, 主條帶] 多保住 1 條交叉帶（main-first 會退 1 條）⇒ 選 cross_first
        assert main_left == [] and [s for s in mf_left if is_cross_strip(s)] == [xs[1]] and cf_left == []
        assert left == [] and blocked == []
        assert [is_cross_strip(s) for s in tail] == [True, True] + [False] * 10        # 接續被打斷的那段交叉帶
        assert len(retaken) == 1 and retaken[0].startswith(f"cross_retaken(A1: {tail[:2]}; order=cross_first; ")


# ---------------------------------------------------------------------------
# L1：鄰機空域規則（cross_allow_neighbor_airspace）× 原機拿回
# ---------------------------------------------------------------------------
def test_relay_parent_only_matches_relay_cells():
    def c(cid, parent, res):
        return SimpleNamespace(cell_id=cid, parent_cell_id=parent, residual_of=res)

    assert dyn._relay_parent(c("Z2_1_R1", "Z2_1", "Z2_1")) == "Z2_1"
    assert dyn._relay_parent(c("Z2_1_R1_R2", "Z2_1_R1", "Z2_1_R1")) == "Z2_1_R1"
    for cell in (c("Z2_1_B_k6", "Z2_1", "Z2_1"),      # decompose 的 B 子胞：residual_of 也是母胞，但不是重鋪
                 c("Z2_1_A_k6", "Z2_1", None), c("Z2_1_R1", None, None), c("Z2_1_R1x", "Z2_1", "Z2_1"),
                 c("Z2_1_R", "Z2_1", "Z2_1"), c("Z9_R1", "Z2_1", "Z2_1")):
        assert dyn._relay_parent(cell) is None, cell


def test_block_holders_in_the_planned_allocation_are_only_the_owner(x2):
    """沒有任何重新分派時，每條交叉帶的母區塊只有原機自己持有主條帶 ⇒ 規則不會誤擋（B3 的 Z2_1_B_k6 雖然
    parent／residual_of 都是 A1 的 Z2_1，但它是 B3 自己的區塊，不是重鋪）。"""
    plan, ctx, cache = _fresh(x2, ON_FORBID)
    sess = dyn._Session(plan, {}, ctx, cache, CFG, 0.0, None)
    held = sess.main_block_holders()
    xs = [s for s in plan.strips if is_cross_strip(s)]
    assert len(xs) == 12
    assert {sess.block_of(s) for s in xs} == {"Z1_1", "Z2_1", "Z2_1_B_k6"}
    for s in xs:
        assert held[sess.block_of(s)] == {cross_strip_owner(s)}, s


def _loss_then_restore(fx, lost, mm):
    plan, ctx, cache = _fresh(fx, mm)
    t = _mid(_xspans(plan, lost)[1])
    res1 = dyn.replan(plan, dyn.LinkLoss(lost, t), {}, ctx, cache, CFG)
    blocked = [s for s, r in res1.plan.uncovered if r == XNR]
    t2 = t + 20.0
    s = dyn._interp_state(res1.plan.plans[lost], t2, ctx.specs[plan.plans[lost].cls], ctx.wind)
    tel = dyn.Telemetry(lost, t2, s.x_m, s.y_m, s.z_m, s.theta_rad, 22.0, None, None, 0.0, True)
    res2 = dyn.replan(res1.plan, dyn.LinkRestored(lost, t2, tel), {}, ctx, cache, CFG, res1.ghosts)
    return res1, res2, blocked


@pytest.mark.parametrize("allow", [False, True])
@pytest.mark.parametrize("lost,holder,how", [("B2", "B3", "same_class"), ("A1", "B2", "relay")])
def test_neighbor_airspace_rule_on_owner_retake(x2, lost, holder, how, allow):
    """失鏈 → 母區塊主條帶交給 ``holder``（同構型原樣接手／跨構型重鋪）→ 恢復連線後原機想拿回交叉帶：
    禁止（預設）⇒ 不還，cause ＝ neighbor_airspace(holder)；明確放行 ⇒ 照舊拿回。"""
    res1, res2, blocked = _loss_then_restore(x2, lost, ON if allow else ON_FORBID)
    p0 = x2[0]
    blk = p0.strips[blocked[0].split("#", 1)[0]].cell_id
    before = {x for x, _d in p0.plans[holder].strips}
    got = [s for s, _d in res1.plan.plans[holder].strips if s not in before]
    assert got and len(blocked) == {"B2": 3, "A1": 2}[lost]
    if how == "same_class":
        assert {res1.plan.strips[s].cell_id for s in got} == {blk}
    else:
        cells = {c.cell_id: c for c in res1.plan.cells}
        assert {res1.plan.strips[s].cell_id for s in got} == {f"{blk}_R1"}
        assert dyn._relay_parent(cells[f"{blk}_R1"]) == blk
    assert [n for n in res1.notes if n.startswith(XNR + "(")] == \
        [f"{XNR}({lost}: {blocked}; cause=uncoverable_after_link_loss; owner=ghost)"]
    if allow:
        assert res2.changed_uavs == [lost] and f"rejoin({lost}: {sorted(blocked)})" in res2.notes
        assert not [x for x, _r in res2.plan.uncovered if x in blocked]
        assert not [n for n in res2.notes if n.startswith(XNR + "(")]
        assert _not_flown(res2.plan, blocked) == []
    else:
        assert res2.changed_uavs == [] and res2.plan.plans[lost] is res1.plan.plans[lost]
        assert [x for x in res2.plan.uncovered if x[0] in blocked] == [(s, XNR) for s in blocked]
        assert [n for n in res2.notes if n.startswith(XNR + "(")] == \
            [f"{XNR}({lost}: {blocked}; cause=neighbor_airspace({holder}); owner=not_taken)"]
        assert not any(n.startswith(("rejoin(", "cross_retaken(")) for n in res2.notes)
    _assert_owner_rule(res2.plan)


# ---------------------------------------------------------------------------
# L2：note 去重
# ---------------------------------------------------------------------------
def test_identical_blocked_notes_are_recorded_once_per_session(x2):
    plan, ctx, cache = _fresh(x2)
    t = _mid(_xspans(plan, "B2")[1])
    sess = dyn._Session(plan, {}, ctx, cache, CFG, t, None)
    ids = [s.strip_id for s in _xspans(plan, "B2")]
    for _ in range(2):
        assert sess.reassign(ids, reason="energy", candidates=["B3"]) == ids
    # (原機, ids, cause) 相同、只有 owner 狀態不同 ⇒ 仍算同一筆（保留第一筆）
    assert sess.reassign(ids, reason="energy", candidates=["B2"], max_changes=0) == ids
    assert sess.reassign(ids, reason="other", candidates=["B3"]) == ids          # cause 不同 ⇒ 另記
    assert sess.reassign(ids[:2], reason="energy", candidates=["B3"]) == ids[:2]  # ids 不同 ⇒ 另記
    exp = [f"{XNR}(B2: {ids}; cause=energy; owner=not_candidate)",
           f"{XNR}(B2: {ids}; cause=other; owner=not_candidate)",
           f"{XNR}(B2: {ids[:2]}; cause=energy; owner=not_candidate)"]
    assert [n for n in sess.notes if n.startswith(XNR + "(")] == exp
    res = sess.finish(dyn.PriorityChange("Z1", 1.0, t), time.perf_counter())
    assert [n for n in res.notes if n.startswith(XNR + "(")] == exp
    assert [x for x in res.plan.uncovered if x[0] in ids] == [(s, XNR) for s in ids]


# ---------------------------------------------------------------------------
# 審查補洞（主控 2026-10-15）：鄰機空域「禁止（預設）」下的正向路徑，與同一次 reassign 內第 1 段之後的持有者
# ---------------------------------------------------------------------------
def _cut_b3(fx, mm):
    plan, ctx, cache = _fresh(fx, mm)
    t = _mid(_mspans(plan, "B3")[5])
    sess = dyn._Session(plan, {}, ctx, cache, CFG, t, None)
    stt = sess.status("B3")
    pend = [sp.strip_id for sp in stt.pending]
    sess.rtb_now("B3", sess.anchor_after_span("B3", stt.current), "test_cut")
    return sess, pend


def test_forbid_mode_owner_alone_retakes_its_own_cross_strips(x1):
    """預設（禁止）最常走的路徑：母區塊的主條帶只有原機自己持有 ⇒ 原機拿回自己的交叉帶（不是一律不還）。
    審查突變 V5（把原機自己也算成鄰機）在這裡被抓到。"""
    sess, pend = _cut_b3(x1, ON_FORBID)
    assert (sess.xstrips_on, sess.xstrips_allow_neighbor) == (True, False)
    xs = [s for s in pend if is_cross_strip(s)]
    assert xs and all(cross_strip_owner(s) == "B3" for s in xs)
    held = sess.main_block_holders()
    assert all(held.get(sess.block_of(s), set()) <= {"B3"} for s in xs)
    left = sess.reassign(pend, candidates=["B3"], reason="energy")
    assert left == []
    assert any(n.startswith("cross_retaken(B3:") for n in sess.notes)
    assert not any(n.startswith(XNR) for n in sess.notes)


def _cut_b2_in_cross(fx, mm):
    plan, ctx, cache = _fresh(fx, mm)
    sess = dyn._Session(plan, {}, ctx, cache, CFG, _mid(_xspans(plan, "B2")[1]), None)
    stt = sess.status("B2")
    pend = [sp.strip_id for sp in stt.pending]
    sess.rtb_now("B2", sess.anchor_after_span("B2", stt.current), "test_cut")
    return sess, pend


@pytest.mark.parametrize("mm", ["forbid", "allow"])
def test_retake_rule_uses_holders_after_phase_one_of_the_same_reassign(x2, mm):
    """同一次 reassign：第 1 段把 B2 母區塊剩下的主條帶交給同構型的 B3 ⇒ 第 2 段判斷要用**第 1 段之後**的持有者：
    禁止（預設）⇒ B2 的交叉帶不還（neighbor_airspace(B3)）；放行 ⇒ 照樣還給 B2（舊行為）。
    審查突變 V3（改讀 session 開始時的計畫）在禁止模式下會放行 ⇒ 被抓到。"""
    sess, pend = _cut_b2_in_cross(x2, ON_FORBID if mm == "forbid" else ON)
    xs = [s for s in pend if is_cross_strip(s)]
    assert xs and all(cross_strip_owner(s) == "B2" for s in xs)
    left = sess.reassign(pend, candidates=["B2", "B3"], reason="energy")
    take = [n for n in sess.notes if n.startswith("takeover(B3:")]
    assert take, sess.notes                                           # 非空轉：B3 真的在第 1 段接了主條帶
    blocks = {sess.block_of(s) for s in xs}
    assert all("B3" in sess.main_block_holders().get(b, set()) for b in blocks)
    notes = [n for n in sess.notes if n.startswith((XNR, "cross_retaken"))]
    if mm == "forbid":
        assert set(xs) <= set(left)
        assert notes == [f"{XNR}(B2: {xs}; cause=neighbor_airspace(B3); owner=not_taken)"]
    else:
        assert not set(xs) & set(left)
        assert len(notes) == 1 and notes[0].startswith("cross_retaken(B2:")
