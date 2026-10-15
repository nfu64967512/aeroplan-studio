"""建圖模式交叉航帶（MappingModeConfig.cross_strips）— 規劃器核心層的數值鎖。

情境（與 test_mapping_mode.py 同一套輸入）：1×1 km 平地、HQ IMX477 + 6 mm（120 m：w 125.736 m、2 Hz）、
側向 0.70／前向 0.80、1 Pioneer（min_fw 100）+ 2 HERO、無豁免（h_ceil 120）。單一區 ⇒ A 飛 A 殘帶子區、
B 飛 B 殘帶子區（兩機各有一個「實際飛的區塊」，而不是整個胞）。

每條斷言**數值**；會變的地方先證明關閉時（MappingModeConfig() 預設）沒有該行為。
"""
from __future__ import annotations

import dataclasses
import math
from types import SimpleNamespace

import numpy as np
import pytest
import shapely
from shapely.geometry import Polygon

import hv_dccpp.planner as P
from hv_dccpp import decompose as D
from hv_dccpp.allocate import Team
from hv_dccpp.ardupilot import MAV_CMD_DO_SET_CAM_TRIGG_DIST, CameraTriggerConfig, plan_to_mission_items, \
    trigger_distance_m
from hv_dccpp.context import build_context
from hv_dccpp.geometry import (CROSS_STRIP_TAG, cross_strip_id, is_cross_strip, strips_union_footprint,
                               sweep_origin, to_sweep_coords)
from hv_dccpp.mission import MappingMissionInputs, MappingModeConfig, MissionInputError, load_mission
from hv_dccpp.planner import plan_mission
from hv_dccpp.sim import build_validation_cache, validate

from .test_mapping_mode import CFG, _inputs

ON = MappingModeConfig(cross_strips=True, cross_fraction=0.2)


def _plan(mm):
    inp = _inputs(mm, km=1.0, split=1)
    return plan_mission(inp, CFG), inp


@pytest.fixture(scope="module")
def off():
    return _plan(MappingModeConfig())


@pytest.fixture(scope="module")
def on():
    return _plan(ON)


@pytest.fixture(scope="module")
def low():
    """交叉帶低 20 m（120 → 100 m ＝ 最低固定翼高度 100，帶下緣剛好）。"""
    return _plan(dataclasses.replace(ON, cross_alt_offset_m=-20.0))


def _cross_ids(plan, uav=None):
    return [s for u, up in plan.plans.items() if uav in (None, u) for s, _d in up.strips if is_cross_strip(s)]


# ---------------------------------------------------------------------------
# 1. 設定物件
# ---------------------------------------------------------------------------
def test_config_off_by_default_and_to_dict_unchanged():
    m = MappingModeConfig()
    assert (m.cross_strips, m.cross_fraction, m.cross_alt_offset_m, m.cross_angle_offset_deg) == (False, 0.2, 0.0, 90.0)
    # 沒開交叉航帶 ⇒ to_dict 與加入交叉航帶之前逐鍵相同（build_context 的 mapping_mode(...) 警告也因此不變）
    assert m.to_dict() == {"seam_tie": True, "no_altitude_layering": True, "crab_margin": True, "lead_in_m": 0.0,
                           "min_block_strips": 3}
    d = MappingModeConfig(cross_strips=True, cross_fraction=0.3, cross_alt_offset_m=-20).to_dict()
    assert {k: d[k] for k in ("cross_strips", "cross_fraction", "cross_alt_offset_m", "cross_angle_offset_deg")} == \
        {"cross_strips": True, "cross_fraction": 0.3, "cross_alt_offset_m": -20.0, "cross_angle_offset_deg": 90.0}


@pytest.mark.parametrize("bad", [
    dict(cross_strips=False, cross_fraction=0.3),          # 沒開卻給值 ⇒ 會被靜默忽略 ⇒ 拒絕
    dict(cross_strips=False, cross_alt_offset_m=-20.0),
    dict(cross_strips=False, cross_angle_offset_deg=80.0),
    dict(cross_strips=True, cross_fraction=0.0), dict(cross_strips=True, cross_fraction=-0.1),
    dict(cross_strips=True, cross_fraction=1.01), dict(cross_strips=True, cross_fraction=float("nan")),
    dict(cross_strips=True, cross_alt_offset_m=float("inf")), dict(cross_strips=True, cross_alt_offset_m="5"),
    dict(cross_strips=True, cross_angle_offset_deg=0.0), dict(cross_strips=True, cross_angle_offset_deg=180.0),
    dict(cross_strips=True, cross_angle_offset_deg=9.0), dict(cross_strips=True, cross_angle_offset_deg=171.0),
    dict(cross_strips=1),
])
def test_config_rejects(bad):
    with pytest.raises(MissionInputError):
        MappingModeConfig(**bad)


@pytest.mark.parametrize("ok", [dict(cross_fraction=1.0), dict(cross_angle_offset_deg=10.0),
                                dict(cross_angle_offset_deg=170.0), dict(cross_angle_offset_deg=-90.0),
                                dict(cross_alt_offset_m=25)])
def test_config_accepts_edges(ok):
    m = MappingModeConfig(cross_strips=True, **ok)
    assert m.cross_strips is True and isinstance(m.cross_alt_offset_m, float)


def test_mapping_inputs_roundtrip_with_cross():
    mm = MappingModeConfig(cross_strips=True, cross_fraction=0.25, cross_alt_offset_m=-10.0)
    inp = _inputs(mm)
    d = inp.to_dict()
    assert d["mapping_mode"]["cross_fraction"] == 0.25 and d["mapping_mode"]["cross_alt_offset_m"] == -10.0
    back = load_mission(d)
    assert type(back) is MappingMissionInputs and back.mapping_mode == mm


def test_cross_strip_id_and_tag():
    sid = cross_strip_id("Z1_1_B_k10", "B", "B3", 4)
    assert sid == "Z1_1_B_k10_B_cross_B3_4" and CROSS_STRIP_TAG in sid
    assert is_cross_strip(sid) and is_cross_strip(SimpleNamespace(strip_id=sid))
    assert not is_cross_strip("Z1_1_A_4") and not is_cross_strip(None)


# ---------------------------------------------------------------------------
# 2. 中央橫帶（純幾何）
# ---------------------------------------------------------------------------
def _rect(w, h, x0=0.0, y0=0.0):
    return Polygon([(x0, y0), (x0 + w, y0), (x0 + w, y0 + h), (x0, y0 + h)])


@pytest.mark.parametrize("frac", [0.05, 0.2, 0.3, 0.5, 0.9])
def test_central_band_area_and_centring_on_rectangle(frac):
    blk = _rect(600.0, 2000.0)                                  # 主條帶沿北（α = 0），交叉帶沿東（β = 90°）
    reg, H = D.cross_central_band(blk, math.pi / 2.0, frac)
    assert reg.area == pytest.approx(frac * blk.area, rel=1e-6)
    assert H == pytest.approx(frac * 2000.0, rel=1e-6)
    minx, miny, maxx, maxy = reg.bounds
    assert (minx, maxx) == (pytest.approx(0.0), pytest.approx(600.0))            # 東西向貫穿整個區塊
    assert 0.5 * (miny + maxy) == pytest.approx(1000.0, abs=1e-6)                # 置中於形心


def test_central_band_on_irregular_block_and_edges():
    blk = Polygon([(0, 0), (800, 0), (800, 300), (300, 1500), (0, 1500)])      # 梯形＋缺角
    for frac in (0.1, 0.2, 0.37):
        reg, _H = D.cross_central_band(blk, math.pi / 2.0, frac)
        assert reg.area == pytest.approx(frac * blk.area, rel=1e-6)
        assert reg.within(blk.buffer(1e-6))
    full, H = D.cross_central_band(blk, math.pi / 2.0, 1.0)
    assert full.equals(blk) and H == pytest.approx(1500.0)
    reg, H = D.cross_central_band(_rect(600, 2000), math.pi / 2.0, 0.01, min_height_m=163.4)
    assert H == pytest.approx(163.4) and reg.area == pytest.approx(600 * 163.4, rel=1e-9)
    # 形心靠一端（三角形）：橫帶整段平移不截短，面積照樣 = frac·A
    tri = Polygon([(0, 0), (1000, 0), (0, 1000)])
    reg, _H = D.cross_central_band(tri, math.pi / 2.0, 0.6)
    assert reg.area == pytest.approx(0.6 * tri.area, rel=1e-6)


# ---------------------------------------------------------------------------
# 3. 規劃：只在開啟時有、同一架、緊鄰母區塊、轉 90°、面積比例、≥ 2 條
# ---------------------------------------------------------------------------
def test_no_cross_strips_when_off(off):
    plan, _inp = off
    assert plan.plans and not _cross_ids(plan) and not any(is_cross_strip(s) for s in plan.strips)
    assert "cross_strips" not in plan.timing_ms
    assert not any(w.startswith(("mapping_cross", "cross_")) for w in plan.feasibility.warnings)


def test_cross_strips_exist_for_every_flown_block(on, off):
    plan, _inp = on
    assert plan.feasibility.status != "NO_GO" and "cross_strips" in plan.timing_ms
    for u, up in plan.plans.items():
        main_cells = {plan.strips[s].cell_id for s, _d in up.strips if not is_cross_strip(s)}
        cross = [s for s, _d in up.strips if is_cross_strip(s)]
        assert main_cells and cross, u
        for s in cross:
            assert f"{CROSS_STRIP_TAG}{u}_" in s                           # 同一架（id 帶機號）
            assert plan.strips[s].cell_id in main_cells                    # 母區塊是這架自己飛的
            assert plan.strips[s].cls == up.cls
    # 主條帶完全不變（交叉帶是加在後面的，不重鋪主條帶）
    plan0, _ = off
    main_on = {s for s in plan.strips if not is_cross_strip(s)}
    assert main_on == set(plan0.strips)
    for s in main_on:
        assert np.allclose(plan.strips[s].pL, plan0.strips[s].pL) and np.allclose(plan.strips[s].pR, plan0.strips[s].pR)
    s_txt = next(w for w in plan.feasibility.warnings if w.startswith("mapping_cross("))
    assert "面積比例 0.2" in s_txt and all(u in s_txt for u in plan.plans)


def test_cross_set_is_contiguous_and_adjacent_to_its_block(on):
    plan, _inp = on
    for u, up in plan.plans.items():
        ids = [s for s, _d in up.strips]
        flags = [is_cross_strip(s) for s in ids]
        runs = [i for i in range(1, len(flags)) if flags[i] != flags[i - 1]]
        assert len(runs) == 1, (u, flags)                                 # 一個區塊 ⇒ 一段連續交叉帶
        i = runs[0]
        a, b = ids[i - 1], ids[i]                                          # 交界兩側一主一交叉、同一母胞
        assert plan.strips[a].cell_id == plan.strips[b].cell_id


def test_cross_direction_is_rotated_90deg(on):
    plan, _inp = on
    for u, up in plan.plans.items():
        mains = [plan.strips[s] for s, _d in up.strips if not is_cross_strip(s)]
        alpha = mains[0].theta_LR_rad % math.pi
        for s in _cross_ids(plan, u):
            d = (plan.strips[s].theta_LR_rad - alpha) % math.pi
            assert abs(math.degrees(d) - 90.0) < 0.5, (u, s, math.degrees(d))


def _rebuild_set(plan, inp, u, **kw):
    ctx = build_context(inp, CFG)
    dc = ctx.decompose_ctx()
    up = plan.plans[u]
    mains = [plan.strips[s] for s, _d in up.strips if not is_cross_strip(s)]
    cell = next(c for c in plan.cells if c.cell_id == mains[0].cell_id)
    args = dict(fraction=0.2, alt_offset_m=0.0, angle_offset_deg=90.0)
    args.update(kw)
    return D.build_cross_set(cell, up.cls, mains, dc, uav_id=u, **args), cell, mains


def test_cross_region_area_fraction_and_block_is_the_uavs_own_part(on):
    plan, inp = on
    smaller = []
    for u, up in plan.plans.items():
        cs, cell, mains = _rebuild_set(plan, inp, u)
        assert isinstance(cs, D.CrossSet)
        # 「該機實際飛的區塊」＝ 主條帶足跡 ∩ 胞：A 的殘帶條帶掛在母胞上、只飛前 k 條 ⇒ 區塊比母胞小；
        # B 飛的是自己的殘帶子胞（整胞）⇒ 區塊 = 子胞
        blk = cell.polygon.intersection(strips_union_footprint(mains))
        assert cs.block_area_m2 == pytest.approx(blk.area, rel=1e-9)
        assert cs.block_area_m2 <= cell.polygon.area + 1e-6
        if cs.block_area_m2 < cell.polygon.area - 1.0:
            smaller.append(up.cls)
        assert cs.fraction_effective == pytest.approx(0.2, abs=1e-6) and not cs.widened_to_two
        # 鋪滿中央橫帶 ⇒ 成像面積 ≥ 橫帶，最多多出一條間距＋刈幅的邊
        ext = float(np.ptp(to_sweep_coords(np.asarray(shapely.get_coordinates(cs.block)), sweep_origin(cs.block, cs.beta_rad),
                                           cs.beta_rad)[:, 0]))
        assert cs.region_area_m2 - 1e-6 <= cs.imaged_area_m2 <= cs.region_area_m2 + (cs.width_m + cs.spacing_m) * ext
        assert len(cs.strips) >= 2
        # 規劃裡的交叉帶就是這一組（同 id、同端點）
        got = {s: plan.strips[s] for s in _cross_ids(plan, u)}
        assert set(got) == set(cs.strip_ids)
        for st in cs.strips:
            assert np.allclose(got[st.strip_id].pL, st.pL) and np.allclose(got[st.strip_id].pR, st.pR)
    assert smaller == ["A"]


def test_tiny_fraction_is_widened_to_two_strips_and_reported():
    plan, inp = _plan(dataclasses.replace(ON, cross_fraction=0.01))
    for u in plan.plans:
        cs, _cell, _m = _rebuild_set(plan, inp, u, fraction=0.01)
        assert cs.widened_to_two and len(cs.strips) == 2
        assert cs.fraction_effective > 0.05                                # 有效比例照實寫出來
        assert len(_cross_ids(plan, u)) == 2
    s_txt = next(w for w in plan.feasibility.warnings if w.startswith("mapping_cross("))
    assert s_txt.count("已放寬到 2 條") == len(plan.plans) and "目標 0.01" in s_txt


def test_block_that_cannot_host_two_strips_is_skipped(on):
    plan, inp = on
    u = next(iter(plan.plans))
    ctx = build_context(inp, CFG)
    dc = ctx.decompose_ctx()
    up = plan.plans[u]
    mains = [plan.strips[s] for s, _d in up.strips if not is_cross_strip(s)]
    cell = next(c for c in plan.cells if c.cell_id == mains[0].cell_id)
    # 把母胞壓成沿主條帶方向只有 100 m（< 刈幅 125.7 m）⇒ 交叉帶最多 1 條 ⇒ 略過並說出原因
    c = cell.polygon.centroid
    thin = shapely.box(c.x - 2000, c.y - 50, c.x + 2000, c.y + 50)
    narrow = dataclasses.replace(cell, polygon=thin)
    res = D.build_cross_set(narrow, up.cls, mains, dc, uav_id=u, fraction=0.2, alt_offset_m=0.0, angle_offset_deg=90.0)
    assert isinstance(res, str) and "放不下 2 條交叉帶" in res and "100.0 m" in res


def test_skipped_block_is_listed_in_plan_warnings(monkeypatch):
    """區塊放不下 2 條（或碰禁區）時 build_cross_set 回原因字串 ⇒ 該區塊沒有交叉帶、`mapping_cross_skipped(...)` 列出原因，
    其他區塊照排；不是錯誤、不擋上傳（使用者看得到哪幾塊沒有）。"""
    real = D.build_cross_set

    def fake(parent, cls, main_strips, ctx, *, uav_id, **kw):
        if cls == "B":
            return f"{parent.cell_id}@{uav_id}: 區塊在主條帶方向只有 100.0 m，放不下 2 條交叉帶（測試）"
        return real(parent, cls, main_strips, ctx, uav_id=uav_id, **kw)

    monkeypatch.setattr(D, "build_cross_set", fake)
    plan, _inp = _plan(ON)
    b = [u for u in plan.plans if u.startswith("B")]
    assert b and all(not _cross_ids(plan, u) for u in b)
    assert all(_cross_ids(plan, u) for u in plan.plans if u.startswith("A"))
    w = next(x for x in plan.feasibility.warnings if x.startswith("mapping_cross_skipped("))
    assert all(f"@{u}: 區塊在主條帶方向只有 100.0 m" in w for u in b)
    assert not any(r.startswith("cross_") for _s, r in plan.uncovered)        # 略過不是「沒飛到」


# ---------------------------------------------------------------------------
# 4. 高度：帶邊界照收，超出一律拒絕（不夾限）
# ---------------------------------------------------------------------------
def test_altitude_check_band_edges(on):
    plan, inp = on
    dc = build_context(inp, CFG).decompose_ctx()
    for c in plan.cells:
        for cls, h_op in c.h_op_m.items():
            if not c.feasible.get(cls):
                continue
            h_x, (lo, hi) = D.cross_altitude_check(c, cls, h_op, 0.0, dc)
            assert (h_x, lo, hi) == (h_op, 100.0, 120.0)                   # min_fw 100、h_ceil 120
            assert D.cross_altitude_check(c, cls, h_op, lo - h_op, dc)[0] == 100.0
            for off in (lo - h_op - 1.0, hi - h_op + 0.5):
                with pytest.raises(D.CrossStripRefused, match="cross_alt_out_of_band"):
                    D.cross_altitude_check(c, cls, h_op, off, dc)


@pytest.mark.parametrize("off", [-21.0, 0.5])
def test_plan_refuses_out_of_band_offset(off):
    plan, _inp = _plan(dataclasses.replace(ON, cross_alt_offset_m=off))
    assert plan.feasibility.status == "NO_GO" and not plan.plans
    msg = " ".join(plan.feasibility.errors)
    assert "cross_alt_out_of_band" in msg and f"{120.0 + off:g} m" in msg
    assert "[100, 120]" in msg


def test_low_cross_strips_fly_at_100m(low):
    plan, _inp = low
    assert plan.feasibility.status != "NO_GO"
    for u, up in plan.plans.items():
        xs = _cross_ids(plan, u)
        assert xs and all(plan.strips[s].h_op_m == pytest.approx(100.0) for s in xs)
        assert all(plan.strips[s].h_op_m == pytest.approx(120.0) for s, _d in up.strips if not is_cross_strip(s))
        # 間距以交叉高度算（100 m 足跡較窄 ⇒ 間距 0.3·w(100)）
        w100 = 100.0 * 4056 * 1.55e-3 / 6.0
        assert plan.strips[xs[0]].spacing_m == pytest.approx(0.3 * w100, rel=1e-9)
        # 交叉帶全程真的在 100 m：進入／返航段的目標高度是過境高度 h_transit（A 120 m），若不收緊交叉帶作業段的
        #   高度上界，包絡平滑會把升降延伸進交叉帶（修前實測帶上 100 → 116.6 m）
        agl = [w.agl_m for w in up.waypoints if w.strip_id in set(xs)]
        assert agl and max(agl) < 101.0 and min(agl) > 99.0, (u, max(agl), min(agl))
        assert not any(v.startswith("profile:envelope_infeasible") for v in up.warnings), up.warnings
    # 交叉帶不同高是刻意的 ⇒ 不報「主作業高度不一致」；摘要寫出交叉高度與 GSD 比
    w = plan.feasibility.warnings
    assert not any(x.startswith("mapping_altitude_nonuniform") for x in w)
    s_txt = next(x for x in w if x.startswith("mapping_cross("))
    assert "高度 100 m（主 120 m，GSD 比 0.833）" in s_txt and "高度差 -20 m" in s_txt


# ---------------------------------------------------------------------------
# 5. 匯出（206 開關、觸發距離）與驗證
# ---------------------------------------------------------------------------
def _on_windows(items):
    out, prev = [], False
    for it in items:
        if it.command == MAV_CMD_DO_SET_CAM_TRIGG_DIST:
            on_ = it.param1 > 0
            if on_ and not prev:
                out.append(float(it.param1))
            prev = on_
    return out


@pytest.mark.parametrize("which", ["on", "low"])
def test_206_on_off_per_cross_strip_and_trigger_distance(which, on, low):
    plan, inp = {"on": on, "low": low}[which]
    ctx = build_context(inp, CFG)
    cc = CameraTriggerConfig(interval_min_s=0.5, max_roll_deg=15.0, camera_lead_in_m=0.0)
    for u, up in plan.plans.items():
        items = plan_to_mission_items(up, ctx, CFG, camera_trigger=True, camera_cfg=cc)
        wins = _on_windows(items)
        n_off = sum(1 for it in items if it.command == MAV_CMD_DO_SET_CAM_TRIGG_DIST and it.param1 == 0)
        assert len(wins) == len(up.strips) and n_off >= len(up.strips)      # 每條帶（含交叉）一開一關
        sen = next(s for (c, _r), s in ctx.sensors.items() if c == up.cls)
        for (sid, _d), d in zip(up.strips, wins):
            h = plan.strips[sid].h_op_m
            other = 120.0 if abs(h - 100.0) < 1e-6 else 100.0
            assert abs(d - trigger_distance_m(sen, h)) < abs(d - trigger_distance_m(sen, other)), (u, sid, h, d)
            if not is_cross_strip(sid):
                assert d == pytest.approx(trigger_distance_m(sen, h), rel=1e-9)     # 主條帶：吸附到作業高度，逐值
            else:
                # 交叉帶：以相機開著時飛到的最低 AGL 算 ⇒ 不比作業高度的 d 大（前向重疊不低於規劃）；
                #   帶終點可能已在下一段的升降上（縮點後線性），最多小幾 %
                assert trigger_distance_m(sen, h) * 0.97 <= d <= trigger_distance_m(sen, h) * (1 + 5e-3), (u, sid, h, d)
    if which == "low":
        assert trigger_distance_m(sen, 100.0) == pytest.approx(100.0 * 3040 * 1.55e-3 / 6.0 * 0.2)   # 15.707 m


def test_exporter_uses_lowest_strip_agl_for_cross_trigger_distance(low):
    """交叉帶起點若還在升降途中（例如 build_3d 的包絡讓高度延伸進作業段），206 的 d 以本帶 OPERATE 航點中
    最低的 AGL 算（保守 ⇒ 前向重疊不低於規劃）；主條帶照舊用帶起點 AGL（0.5 m 內吸附到作業高度）。"""
    plan, inp = low
    ctx = build_context(inp, CFG)
    cc = CameraTriggerConfig(interval_min_s=0.5, max_roll_deg=15.0, camera_lead_in_m=0.0)
    u, up = next((u, up) for u, up in plan.plans.items() if _cross_ids(plan, u))
    sid = _cross_ids(plan, u)[0]
    i0 = next(i for i, w in enumerate(up.waypoints) if w.strip_id == sid)
    wps = list(up.waypoints)
    wps[i0] = dataclasses.replace(wps[i0], z_m=wps[i0].z_m + 15.0, agl_m=wps[i0].agl_m + 15.0)   # 帶起點高 15 m
    up2 = dataclasses.replace(up, waypoints=wps)
    sen = next(s for (c, _r), s in ctx.sensors.items() if c == up.cls)
    items = plan_to_mission_items(up2, ctx, CFG, camera_trigger=True, camera_cfg=cc)
    wins = _on_windows(items)
    k = [s for s, _d in up.strips].index(sid)
    assert wins[k] == pytest.approx(trigger_distance_m(sen, 100.0), rel=5e-3)        # 不是 d(115) = 18.06 m
    assert trigger_distance_m(sen, 115.0) > wins[k] * 1.1


def test_validate_passes_with_cross_strips(on):
    plan, inp = on
    ctx = build_context(inp, CFG)
    v = validate(plan, ctx, build_validation_cache(plan, ctx, budget_only=True), CFG)
    assert v.integrity_ok and v.energy_ok and not v.conflicts and v.passed, v.details


# ---------------------------------------------------------------------------
# 6. 能量不夠 ⇒ 逐組退回、列入未覆蓋（不靜默少飛）；關閉時整段不執行
# ---------------------------------------------------------------------------
def test_energy_infeasible_cross_set_goes_to_uncovered(monkeypatch):
    real = P.append_rtb

    def fake(seq, ctx, cache, model, cfg):
        if any(is_cross_strip(it.strip_id) for it in seq.items) and seq.uav_id.startswith("B"):
            return P.EnergyInfeasible(seq.uav_id, len(seq.items), 0, None, 12.5)
        return real(seq, ctx, cache, model, cfg)

    monkeypatch.setattr(P, "append_rtb", fake)
    plan, inp = _plan(ON)
    b = [u for u in plan.plans if u.startswith("B")]
    a = [u for u in plan.plans if u.startswith("A")]
    assert b and a
    for u in b:
        assert not _cross_ids(plan, u)                                      # B 的交叉帶退回
    assert all(_cross_ids(plan, u) for u in a)                              # A 不受影響
    dropped = [s for s, r in plan.uncovered if r == "cross_energy_infeasible"]
    assert dropped and all(is_cross_strip(s) and any(f"_cross_{u}_" in s for u in b) for s in dropped)
    assert all(s in plan.strips for s in dropped)                           # 留在 plan.strips（可見）＋列未覆蓋
    w = " ".join(plan.feasibility.warnings)
    assert "mapping_cross_dropped(" in w and "缺 12.5 Wh" in w and "mapping_incomplete(" in w
    ctx = build_context(inp, CFG)
    v = validate(plan, ctx, build_validation_cache(plan, ctx, budget_only=True), CFG)
    assert v.integrity_ok                                                    # 未飛的交叉帶已列 uncovered


def test_post_assembly_energy_check_drops_cross_sets(monkeypatch):
    """組裝後以 sim.check_energy（validate 同一準則，含重飛能量 e_MA）複驗；有交叉帶的機不過 ⇒ 逐組退回、列入未覆蓋。"""
    import hv_dccpp.sim as S
    real = S.check_energy

    def fake(plan_u, model, ctx, cache):
        out = list(real(plan_u, model, ctx, cache))
        if plan_u.uav_id.startswith("A") and any(is_cross_strip(s) for s, _d in plan_u.strips):
            out.append("missed_approach_energy: E(P_tc)=1.0 < r*E_nom+e_MA=2.0")
        return out

    monkeypatch.setattr(S, "check_energy", fake)
    plan, inp = _plan(ON)
    a = [u for u in plan.plans if u.startswith("A")]
    b = [u for u in plan.plans if u.startswith("B")]
    assert a and all(not _cross_ids(plan, u) for u in a)                      # A 的交叉帶退回
    assert all(_cross_ids(plan, u) for u in b)                                # B 不受影響
    dropped = [s for s, r in plan.uncovered if r == "cross_energy_infeasible"]
    assert dropped and all(f"_cross_{a[0]}_" in s for s in dropped)
    w = " ".join(plan.feasibility.warnings)
    assert "組裝後能量複驗不過（missed_approach_energy" in w
    ctx = build_context(inp, CFG)
    v = validate(plan, ctx, build_validation_cache(plan, ctx, budget_only=True), CFG)
    assert v.integrity_ok


def test_cross_code_path_not_entered_when_off(monkeypatch, off):
    def boom(*a, **k):
        raise AssertionError("交叉航帶沒開卻進了 _insert_cross_strips")

    monkeypatch.setattr(P, "_insert_cross_strips", boom)
    plan, _inp = _plan(MappingModeConfig())
    ref, _ = off
    assert plan.makespan_s == ref.makespan_s and set(plan.strips) == set(ref.strips)
    assert {u: p.strips for u, p in plan.plans.items()} == {u: p.strips for u, p in ref.plans.items()}


def test_rebuild_sequence_reproduces_idp_arithmetic(monkeypatch):
    """`_rebuild_sequence` 用與 Beam-IDP 同一套 _apply_warm／_sequences_from 算術：插入前（IDP + append_rtb 的輸出）
    原樣重算，每項的作業起訖時刻與剩餘能量逐值相同 ⇒ 插入後的時間／能量差全來自交叉帶本身。"""
    real = P._insert_cross_strips
    checked: list[str] = []

    def spy(seqs, ctx, dc, cfg, cache, cells_by_id, fleet, init_states, t0_of, recovery, *rest):
        for u, sq in seqs.items():
            if not sq.items:
                continue
            init = dataclasses.replace(init_states[u], t_s=float(init_states[u].t_s) + float(t0_of.get(u, 0.0)))
            rb = P._rebuild_sequence(u, [(it.strip_id, it.dir) for it in sq.items], cache, init, sq.landing_site)
            assert [(a.strip_id, a.dir) for a in rb.items] == [(b.strip_id, b.dir) for b in sq.items]
            for a, b in zip(rb.items, sq.items):
                assert a.t_start_s == pytest.approx(b.t_start_s, abs=1e-9)
                assert a.t_end_s == pytest.approx(b.t_end_s, abs=1e-9)
                assert a.e_end_wh == pytest.approx(b.e_end_wh, abs=1e-9)
            checked.append(u)
        return real(seqs, ctx, dc, cfg, cache, cells_by_id, fleet, init_states, t0_of, recovery, *rest)

    monkeypatch.setattr(P, "_insert_cross_strips", spy)
    plan, _inp = _plan(ON)
    assert len(checked) == len(plan.plans) >= 2


# ---------------------------------------------------------------------------
# 7. 空間互斥豁免：交叉帶伸出胞邊時要取消豁免
# ---------------------------------------------------------------------------
def test_cross_skip_filter_drops_exemption_when_cross_band_reaches_neighbour(on):
    plan, inp = on
    u = next(u for u in plan.plans if u.startswith("A"))
    cs, cell, _m = _rebuild_set(plan, inp, u)
    pl = P.CrossPlacement(cs, "before", "x", 0.0, [])
    cache = SimpleNamespace(R_eff={"A": 150.0})
    cfg = SimpleNamespace(d_sep_m=50.0)
    team_u = Team("T1", "A", [u], [cell.cell_id], [], {}, {})
    # 鄰隊的胞：放在交叉帶伸出端外側 50–200 m（主條帶的迴轉帶碰不到，交叉帶的會）
    xmin, ymin, xmax, ymax = cs.block.bounds
    far = dataclasses.replace(cell, cell_id="FAR", polygon=shapely.box(xmax + 20000, ymin, xmax + 20600, ymax))
    near_poly = shapely.box(xmax + 300, ymin + 0.4 * (ymax - ymin), xmax + 900, ymin + 0.6 * (ymax - ymin))
    near = dataclasses.replace(cell, cell_id="NEAR", polygon=near_poly)
    cells = {cell.cell_id: cell, "FAR": far, "NEAR": near}
    t_far = Team("T2", "A", ["A9"], ["FAR"], [], {}, {})
    t_near = Team("T3", "A", ["A8"], ["NEAR"], [], {}, {})
    skip = {(u, "A9"), (u, "A8")}
    out = P._cross_skip_filter(skip, [team_u, t_far, t_near], cells, {u: [pl]}, cache, cfg)
    assert (u, "A9") in out and (u, "A8") not in out
    assert P._cross_skip_filter(skip, [team_u, t_far, t_near], cells, {}, cache, cfg) == skip


def test_cross_airspace_intrusion_into_neighbour_blocks_is_reported(on, off):
    """審查探針 2026-10-15：交叉帶（含 e_ext＋引入段與迴轉）沿交叉方向伸進他機獨占主條帶區塊、同高度、只靠 4D 時間
    錯開 ⇒ 必須明講（`mapping_cross_airspace(...)`），不能宣稱「交叉帶不與他機共用空域」。"""
    plan, _inp = on
    w = next(x for x in plan.feasibility.warnings if x.startswith("mapping_cross_airspace("))
    flying = [u for u in plan.plans if _cross_ids(plan, u)]
    assert flying and all(f"{u} → " in w for u in flying), w
    areas = [float(t.split(" ha")[0].split()[-1]) for t in w.split("、") if " ha" in t]
    assert areas and all(a > 0.0 for a in areas), w
    plan0, _ = off
    assert not any(x.startswith("mapping_cross_airspace(") for x in plan0.feasibility.warnings)
    # 只算實際飛的交叉帶：計畫裡沒有任何交叉帶 ⇒ 沒有重疊 ⇒ None
    assert P._cross_airspace_note(plan0.plans, plan0.strips, {}) is None
