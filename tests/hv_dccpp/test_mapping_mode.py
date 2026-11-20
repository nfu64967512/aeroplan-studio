"""建圖模式（MappingModeConfig / MappingMissionInputs）— 規劃器核心層的數值鎖。

情境：平地、HQ IMX477 + 6 mm（120 m：w 125.736、l 94.24 m、2 Hz）、側向 0.70／前向 0.80、
1 Pioneer（min_fw 調為 100）+ 2 HERO、無豁免（h_ceil 120）。每條都斷言**數值**，並先證明舊行為
（未開建圖模式）會給出不同的值，免得測試「永遠會過」。
"""
from __future__ import annotations

import dataclasses
import math
from types import SimpleNamespace

import numpy as np
import pytest
import shapely

from hv_dccpp import decompose as D
from hv_dccpp.context import apply_crab_margin, build_context, build_sensors
from hv_dccpp.geometry import strips_union_footprint, to_sweep_coords
from hv_dccpp.integration.aeroplan_adapter import to_mission_inputs
from hv_dccpp.mission import (
    MappingMissionInputs, MappingModeConfig, MissionInputError, MissionInputs, PayloadRequirement, PlannerConfig,
    load_mission,
)
from hv_dccpp.planner import _mapping_plan_notes, plan_mission
from hv_dccpp.platform import (
    PLATFORM_HERO, PLATFORM_PIONEER, CameraHardware, LidarHardware, combine_sensors, crabbed_width_m, sensor_model,
)
from hv_dccpp.preflight import preflight

HOME = (23.95, 120.90)
_M_LAT = 111132.0
_M_LON = 111412.0 * math.cos(math.radians(HOME[0]))
HQ6 = CameraHardware(focal_length_mm=6.0, sensor_width_mm=4056 * 1.55e-3, sensor_height_mm=3040 * 1.55e-3,
                     pixel_pitch_um=1.55, max_shutter_rate_hz=2.0, is_default_example=False)
W120 = 120.0 * 4056 * 1.55e-3 / 6.0          # 125.736 m
L120 = 120.0 * 3040 * 1.55e-3 / 6.0          # 94.24 m
SIDE = 0.70
TIE = W120 * SIDE                            # 無偏流時 w − s = side·w = 88.0152 m
CFG = PlannerConfig(dem_step_m=25.0, dh_band_min_m=15.0, d_sep_m=50.0, dh_sep_m=50.0)
PLAT_A = dataclasses.replace(PLATFORM_PIONEER, min_fw_operation_agl_m=100.0)
CLASSES = {1: "A", 2: "B", 3: "B"}


def _ll(e, n):
    return (HOME[0] + n / _M_LAT, HOME[1] + e / _M_LON)


def _areas(km=1.0, split=1):
    h = km * 500.0
    xs = [-h + 2 * h * i / split for i in range(split + 1)]
    return [{"area_id": i + 1, "priority": 1.0, "polygon": [_ll(x0, -h), _ll(x1, -h), _ll(x1, h), _ll(x0, h)]}
            for i, (x0, x1) in enumerate(zip(xs, xs[1:]))]


def _drones(km=1.0):
    return [{"drone_id": i + 1, "name": f"UAV-{i + 1}", "position": _ll(-60.0 + 60.0 * i, -km * 500.0 - 400.0),
             "speed": 20.0, "heading": 0.0, "turn_radius": 150.0, "vehicle_type": "vtol", "fov_width": 125.7}
            for i in range(3)]


def _inputs(mapping=None, *, km=1.0, split=1, fwd=0.80, cam=HQ6, wind=(0.0, 0.0), waiver=None, h=120.0):
    req = PayloadRequirement("default", "rgb", gsd_required_m=0.05, side_overlap=SIDE, forward_overlap=fwd,
                             h_target_agl_m=h)
    inputs, _ = to_mission_inputs(_areas(km, split), _drones(km), coverage_width_m=125.7, overlap_rate=SIDE,
                                  coverage_altitude=h, home=_ll(0.0, -km * 500.0 - 400.0),
                                  class_of=lambda d: CLASSES[int(d["drone_id"])], platform_a=PLAT_A,
                                  platform_b=PLATFORM_HERO, sensor_a=(cam,), sensor_b=(cam,),
                                  payload_reqs={"default": req}, wind=wind, waiver_max_agl_m=waiver,
                                  dem_margin_m=3000.0, mapping_mode=mapping)
    return inputs


@pytest.fixture(scope="module")
def ctx_off():
    return build_context(_inputs(None), CFG)


@pytest.fixture(scope="module")
def ctx_on():
    return build_context(_inputs(MappingModeConfig()), CFG)


# ---------------------------------------------------------------------------
# 1. 設定物件與輸入型別
# ---------------------------------------------------------------------------
def test_config_defaults_and_rejects_bad_values():
    m = MappingModeConfig()
    assert (m.seam_tie, m.no_altitude_layering, m.crab_margin, m.lead_in_m, m.min_block_strips) == (True, True, True, 0.0, 3)
    assert m.to_dict() == {"seam_tie": True, "no_altitude_layering": True, "crab_margin": True, "lead_in_m": 0.0,
                           "min_block_strips": 3}
    assert MappingModeConfig(lead_in_m=25).lead_in_m == 25.0
    for bad in (dict(seam_tie="yes"), dict(no_altitude_layering=1), dict(crab_margin=None), dict(lead_in_m=-1.0),
                dict(lead_in_m=float("nan")), dict(lead_in_m="10"), dict(min_block_strips=-1),
                dict(min_block_strips=2.5), dict(min_block_strips=True)):
        with pytest.raises(MissionInputError):
            MappingModeConfig(**bad)


def test_mapping_inputs_type_and_roundtrip():
    plain = _inputs(None)
    assert type(plain) is MissionInputs and plain.mapping_mode is None
    assert "mapping_mode" not in plain.to_dict()
    assert "mapping_mode" not in {f.name for f in dataclasses.fields(MissionInputs)}      # 表 2 欄位不動
    mm = MappingModeConfig(lead_in_m=30.0, min_block_strips=4)
    on = _inputs(mm)
    assert type(on) is MappingMissionInputs and on.mapping_mode is mm
    d = on.to_dict()
    assert d["mapping_mode"] == {"seam_tie": True, "no_altitude_layering": True, "crab_margin": True, "lead_in_m": 30.0,
                                 "min_block_strips": 4}
    back = load_mission(d)
    assert type(back) is MappingMissionInputs and back.mapping_mode == mm and back.to_dict() == d
    assert type(load_mission(plain.to_dict())) is MissionInputs
    kw = {f.name: getattr(plain, f.name) for f in dataclasses.fields(MissionInputs)}
    with pytest.raises(MissionInputError, match="mapping_mode"):
        MappingMissionInputs(**kw, mapping_mode="on")
    assert MappingMissionInputs(**kw, mapping_mode={"lead_in_m": 5}).mapping_mode == MappingModeConfig(lead_in_m=5)


# ---------------------------------------------------------------------------
# 2. 偏流餘裕 crab_margin
# ---------------------------------------------------------------------------
def test_crabbed_width_numbers():
    psi_b = math.asin(5.0 / 19.0)
    assert math.degrees(psi_b) == pytest.approx(15.2575, abs=1e-4)
    assert crabbed_width_m(W120, L120, psi_b) == pytest.approx(96.504, abs=1e-3)       # 規格例 ≈ 96.5
    psi_a = math.asin(5.0 / 21.5)
    assert math.degrees(psi_a) == pytest.approx(13.4477, abs=1e-4)
    assert crabbed_width_m(W120, L120, psi_a) == pytest.approx(100.372, abs=1e-3)
    assert crabbed_width_m(W120, L120, 0.0) == W120                                    # ψ = 0 原樣
    assert crabbed_width_m(100.0, 0.0, math.radians(60.0)) == pytest.approx(50.0)       # 線掃描 l = 0 → w cosψ


def test_sensor_spacing_uses_w_eff():
    req = PayloadRequirement("r", "rgb", gsd_required_m=0.05, side_overlap=SIDE, forward_overlap=0.8)
    cam = sensor_model(HQ6, req)
    assert cam.strip_spacing_m(120.0) == pytest.approx(W120 * 0.3)
    crab = dataclasses.replace(cam, crab_rad=math.asin(5.0 / 19.0))
    assert crab.strip_spacing_m(120.0) == pytest.approx(96.504 * 0.3, abs=1e-3)
    assert crab.swath_m(120.0) == cam.swath_m(120.0) and crab.footprint_len_m(120.0) == cam.footprint_len_m(120.0)
    lid = sensor_model(LidarHardware(), PayloadRequirement("l", "lidar", density_pt_m2=20.0, side_overlap=0.25))
    lid_c = dataclasses.replace(lid, crab_rad=math.radians(30.0))
    assert lid_c.strip_spacing_m(150.0) == pytest.approx(lid.swath_m(150.0) * math.cos(math.radians(30.0)) * 0.75)


def test_apply_crab_margin_values_and_refusals(ctx_off):
    sensors, _k, _w = build_sensors(ctx_off.inputs.fleet, ctx_off.inputs.payload_reqs)
    out, warns = apply_crab_margin(sensors, ctx_off.specs, 5.0)
    assert out[("B", "default")].crab_rad == pytest.approx(math.asin(5.0 / 19.0))
    assert out[("A", "default")].crab_rad == pytest.approx(math.asin(5.0 / 21.5))
    assert out[("B", "default")].strip_spacing_m(120.0) == pytest.approx(28.951, abs=1e-3)
    assert out[("A", "default")].strip_spacing_m(120.0) == pytest.approx(30.112, abs=1e-3)
    assert len(warns) == 2 and any("B/default: ψ=15.26°" in w and "w_eff/w=0.7675" in w for w in warns)
    calm, warns0 = apply_crab_margin(sensors, ctx_off.specs, 0.0)
    assert calm[("B", "default")] is sensors[("B", "default")] and len(warns0) == 2     # ψ=0 也記有效值
    with pytest.raises(MissionInputError, match="巡航空速"):
        apply_crab_margin(sensors, ctx_off.specs, 21.5)                                # W ≥ V_a(Pioneer)
    with pytest.raises(MissionInputError, match="w_eff"):
        apply_crab_margin(sensors, ctx_off.specs, 19.0)                                # Pioneer ψ 62°：w_eff < 0
    portrait = CameraHardware(focal_length_mm=6.0, sensor_width_mm=3.0, sensor_height_mm=6.3, pixel_pitch_um=1.55,
                              max_shutter_rate_hz=2.0, is_default_example=False)
    s2 = {("B", "p"): sensor_model(portrait, PayloadRequirement("p", "rgb", side_overlap=0.7))}
    with pytest.raises(MissionInputError, match="w_eff"):
        apply_crab_margin(s2, ctx_off.specs, 12.0)                                     # ψ 39°：3cos − 6.3sin < 0
    # 同機雙感測：偏流寫進每個子感測器，間距取最小
    lid = sensor_model(LidarHardware(), PayloadRequirement("l", "lidar", density_pt_m2=20.0, side_overlap=0.25))
    comb = combine_sensors(lid, sensors[("B", "default")])
    out3, _ = apply_crab_margin({("B", "c"): comb}, ctx_off.specs, 5.0)
    psi = math.asin(5.0 / 19.0)
    want = min(lid.swath_m(120.0) * math.cos(psi) * 0.75, 96.504 * 0.3)
    assert out3[("B", "c")].strip_spacing_m(120.0) == pytest.approx(want, abs=1e-3)
    assert all(x.crab_rad == pytest.approx(psi) for x in out3[("B", "c")].sensors)


def test_build_context_applies_crab_only_in_mapping_mode():
    off = build_context(_inputs(None, wind=(5.0, 270.0)), CFG)
    on = build_context(_inputs(MappingModeConfig(), wind=(5.0, 270.0)), CFG)
    nocrab = build_context(_inputs(MappingModeConfig(crab_margin=False), wind=(5.0, 270.0)), CFG)
    assert off.sensors[("B", "default")].crab_rad == 0.0
    assert not any(w.startswith("mapping_") for w in off.warnings)
    assert on.sensors[("B", "default")].crab_rad == pytest.approx(math.asin(5.0 / 19.0))
    assert on.sensors[("B", "default")].strip_spacing_m(120.0) == pytest.approx(28.951, abs=1e-3)
    assert "mapping_mode(seam_tie=True, no_altitude_layering=True, crab_margin=True, lead_in_m=0.0, min_block_strips=3)" \
        in on.warnings
    assert any(w.startswith("mapping_crab_margin(B/default: ψ=15.26°") for w in on.warnings)
    assert nocrab.sensors[("B", "default")].crab_rad == 0.0
    assert not any(w.startswith("mapping_crab_margin") for w in nocrab.warnings)
    assert on.decompose_ctx().mapping == MappingModeConfig() and off.decompose_ctx().mapping is None


# ---------------------------------------------------------------------------
# 3. 接縫共用重疊 seam_tie
# ---------------------------------------------------------------------------
def _params(ctx):
    dc = ctx.decompose_ctx()
    return dc, list(D._class_sensor_params(dc, ctx.inputs.areas[0], ["A", "B"]).values())


def test_area_edge_deltas(ctx_off, ctx_on):
    dc, ps = _params(ctx_off)
    assert D.area_edge_deltas(dc, ps) == (15.0, 15.0, None)
    dc, ps = _params(ctx_on)
    d_edge, d_cell, tie = D.area_edge_deltas(dc, ps)
    assert d_edge == 15.0 and tie == pytest.approx(TIE) and d_cell == pytest.approx(TIE / 2.0)
    wind_ctx = build_context(_inputs(MappingModeConfig(), wind=(5.0, 270.0)), CFG)
    dc, ps = _params(wind_ctx)
    assert D.area_edge_deltas(dc, ps)[2] == pytest.approx(W120 - 28.951, abs=1e-3)       # 偏流時取 w − s，更大
    dc, ps = _params(build_context(_inputs(MappingModeConfig(seam_tie=False)), CFG))
    assert D.area_edge_deltas(dc, ps) == (15.0, 15.0, None)


def _cells(ctx, classes, neighbors=None):
    dc = ctx.decompose_ctx()
    kw = {} if neighbors is None else {"seam_neighbors": neighbors}
    return D.decompose_area(ctx.inputs.areas[0], dc, set(classes), polygon_enu=ctx.areas_enu["Z1"],
                            rho_sym_m=ctx.rho_sym_m, **kw)


def test_decompose_grows_into_neighbors_only_in_mapping_mode(ctx_off, ctx_on):
    z = ctx_on.areas_enu["Z1"]
    x0, y0, x1, y1 = z.bounds
    neighbor = shapely.box(x1, y0, x1 + 500.0, y1)                    # 東側相鄰子區
    on = _cells(ctx_on, ["B"], neighbor)
    assert len(on) == 1
    bx0, by0, bx1, by1 = on[0].polygon.bounds
    assert bx1 - x1 == pytest.approx(TIE / 2.0 + 15.0, abs=0.5)        # 往鄰區長 tie/2，再外擴 δ_edge
    assert x0 - bx0 == pytest.approx(15.0, abs=0.5) and y0 - by0 == pytest.approx(15.0, abs=0.5)   # 外緣只外擴 δ_edge
    assert by1 - y1 == pytest.approx(15.0, abs=0.5)
    fp = strips_union_footprint(on[0].strips["B"])
    assert fp.buffer(1e-3).contains(shapely.box(x1, y0, x1 + TIE / 2.0, y1))   # 足跡確實長進鄰區
    off = _cells(ctx_off, ["B"], neighbor)
    plain = _cells(ctx_off, ["B"])
    assert off[0].polygon.equals_exact(plain[0].polygon, 0.0)          # 未開建圖模式：seam_neighbors 不使用
    assert off[0].polygon.bounds[2] - x1 == pytest.approx(15.0, abs=0.5)


def test_residual_split_overlaps_full_strip_overlap(ctx_off, ctx_on):
    got = {}
    for tag, ctx in (("off", ctx_off), ("on", ctx_on)):
        cells = _cells(ctx, ["A", "B"])
        cell = cells[0]
        assert cell.feasible["A"] and cell.feasible["B"]
        pairs = D.residual_candidates(cell, ctx.decompose_ctx(), rho_sym_m=ctx.rho_sym_m)
        inner = [(k, ca, cb) for k, ca, cb in pairs if cb is not None and 0 < k < cell.n_strips["A"]]
        assert inner
        sa = cell.strips["A"]
        for k, _ca, cb in inner:
            v = to_sweep_coords(np.asarray(shapely.get_coordinates(cb.polygon)), sa[0].sweep_origin, cell.alpha_rad["A"])
            got.setdefault(tag, []).append(float(sa[k - 1].v_hi_m - v[:, 1].min()))
    assert got["off"] and all(x == pytest.approx(15.0, abs=1e-6) for x in got["off"])
    assert got["on"] and all(x == pytest.approx(TIE, abs=1e-6) for x in got["on"])


# ---------------------------------------------------------------------------
# 4. 引入段 lead_in_m
# ---------------------------------------------------------------------------
def test_lead_in_extends_each_strip_end(ctx_on):
    dc0 = ctx_on.decompose_ctx()
    dc50 = dataclasses.replace(dc0, mapping=MappingModeConfig(lead_in_m=50.0))
    sensor = dc0.sensors[("B", "default")]
    e0 = D.end_extension_for(dc0, "B", sensor, 120.0)
    assert D.end_extension_for(dc50, "B", sensor, 120.0) - e0 == pytest.approx(50.0, abs=1e-9)
    assert D.end_extension_for(ctx_on.decompose_ctx(), "B", sensor, 120.0) == e0
    cell = _cells(ctx_on, ["B"])[0]
    s0 = D.build_strips(cell, "B", cell.alpha_rad["B"], dc0)
    s50 = D.build_strips(cell, "B", cell.alpha_rad["B"], dc50)
    assert len(s0) == len(s50) > 3
    for a, b in zip(s0, s50):
        assert b.length_m - a.length_m == pytest.approx(100.0, abs=1e-6)
        t = np.array([math.sin(a.theta_LR_rad), math.cos(a.theta_LR_rad)])
        assert float((a.pL - b.pL) @ t) == pytest.approx(50.0, abs=1e-6)
        assert float((b.pR - a.pR) @ t) == pytest.approx(50.0, abs=1e-6)
    # 有 e_ext 覆寫（legacy）時也加
    dc_ov = dataclasses.replace(dc50, cfg=dataclasses.replace(CFG, e_ext_override_m=0.0))
    assert D.end_extension_for(dc_ov, "B", sensor, 120.0) == pytest.approx(50.0)


# ---------------------------------------------------------------------------
# 5. 事後檢查（碎區塊、高度不一、未覆蓋）
# ---------------------------------------------------------------------------
def test_mapping_plan_notes_unit():
    strips = {f"s{i}": SimpleNamespace(cell_id=c, h_op_m=h) for i, (c, h) in
              enumerate([("Z1_1", 120.0)] * 5 + [("Z1_1_B_k5", 120.0)] * 2 + [("Z2_1", 120.0)])}
    plans = {"A1": SimpleNamespace(strips=[(f"s{i}", -1) for i in range(5)]),
             "B2": SimpleNamespace(strips=[("s5", 1), ("s6", -1), ("s7", 1), ("ghost", 1)])}
    out = _mapping_plan_notes(plans, strips, 3)
    assert out == ["mapping_small_blocks(< 3 條：Z1_1_B_k5=2, Z2_1=1；碎區塊削弱整體平差，分解未改)"]
    assert _mapping_plan_notes(plans, strips, 0) == []
    assert _mapping_plan_notes(plans, strips, 2) == ["mapping_small_blocks(< 2 條：Z2_1=1；碎區塊削弱整體平差，分解未改)"]
    strips["s7"] = SimpleNamespace(cell_id="Z2_1", h_op_m=180.0)
    out = _mapping_plan_notes(plans, strips, 0, [("Z3_1", "no_feasible_insertion")])
    assert out == ["mapping_incomplete(未覆蓋 1 項：Z3_1:no_feasible_insertion；拼出來的圖會有破洞)",
                   "mapping_altitude_nonuniform(作業 AGL 120.0–180.0 m ⇒ GSD 相差 1.50 倍)"]
    strips["s7"] = SimpleNamespace(cell_id="Z2_1", h_op_m=120.9)
    assert _mapping_plan_notes(plans, strips, 0) == []                 # 1 m 以內視為同高


# ---------------------------------------------------------------------------
# 6. 不分層 no_altitude_layering（真規劃；200 m 豁免）
# ---------------------------------------------------------------------------
@pytest.fixture(scope="module")
def waiver_plans():
    out = {}
    # "on" 的 min_block_strips=100 只影響事後警告（每個飛的區塊都 < 100 條 ⇒ 全部列出），不影響規劃
    for tag, mm in (("off", None), ("on", MappingModeConfig(min_block_strips=100)),
                    ("layer", MappingModeConfig(no_altitude_layering=False))):
        out[tag] = plan_mission(_inputs(mm, waiver=200.0), CFG)
    return out


def _flown_h(plan):
    return {u: sorted({round(float(plan.strips[sid].h_op_m), 3) for sid, _d in up.strips}) for u, up in plan.plans.items()}


def test_waiver_layering_lifts_pioneer_when_mapping_off(waiver_plans):
    off = waiver_plans["off"]
    h = _flown_h(off)
    assert h["A1"] == [180.0] and min(v for u, hs in h.items() if u != "A1" for v in hs) == 120.0
    assert any(w.startswith("layering_relaid(") for w in off.feasibility.warnings)
    assert waiver_plans["layer"].plans and _flown_h(waiver_plans["layer"])["A1"] == [180.0]   # 旗標關掉就照舊分層


def test_waiver_mapping_keeps_one_altitude(waiver_plans):
    on = waiver_plans["on"]
    assert on.plans and not on.uncovered
    hs = {v for vals in _flown_h(on).values() for v in vals}
    assert hs == {120.0}
    assert all(float(on.strips[sid].width_m) == pytest.approx(W120) for up in on.plans.values() for sid, _d in up.strips)
    w = on.feasibility.warnings
    assert any(x.startswith("mapping_no_altitude_layering(") and "A 120.0/B 120.0" in x for x in w)
    assert not any(x.startswith("layering_relaid(") for x in w)
    assert not any(x.startswith("mapping_altitude_nonuniform") for x in w)


def test_small_block_warning_lists_flown_blocks_of_the_real_plan(waiver_plans):
    for tag, k in (("on", 100), ("layer", 3)):
        plan = waiver_plans[tag]
        cnt: dict[str, int] = {}
        for up in plan.plans.values():
            for sid, _d in up.strips:
                cnt[plan.strips[sid].cell_id] = cnt.get(plan.strips[sid].cell_id, 0) + 1
        small = sorted((c, n) for c, n in cnt.items() if n < k)
        got = [w for w in plan.feasibility.warnings if w.startswith("mapping_small_blocks(")]
        want = ([f"mapping_small_blocks(< {k} 條：" + ", ".join(f"{c}={n}" for c, n in small) + "；碎區塊削弱整體平差，分解未改)"]
                if small else [])
        assert got == want, (tag, got, cnt)
    assert len(cnt) >= 1 and all(n >= 3 for n in cnt.values())          # "layer" 情境沒有碎區塊
    on = waiver_plans["on"].feasibility.warnings
    assert sum(w.startswith("mapping_small_blocks(< 100 條：") for w in on) == 1
    assert not any(w.startswith("mapping_") for w in waiver_plans["off"].feasibility.warnings)


# ---------------------------------------------------------------------------
# 7. 快門限速的 NO_GO 訊息（兩種模式都有；不改判定）
# ---------------------------------------------------------------------------
_CAM_1HZ = dataclasses.replace(HQ6, max_shutter_rate_hz=1.0)


def test_shutter_cap_no_go_names_rate_altitude_and_overlap():
    rep = preflight(_inputs(None, fwd=0.85, cam=_CAM_1HZ), CFG)
    assert rep.status == "NO_GO"
    assert rep.errors[0] == "no_coverable_area: 所有區域皆無可行構型（感測相容／高度帶／鏈路／方向）"
    v_cap = L120 * 0.15 * 1.0
    for c, v_min in (("A", 18.0), ("B", 16.0)):
        crec = rep.per_area["Z1"]["classes"][c]
        assert crec["reason"] == "no_feasible_direction"                   # 判定原樣
        lim = crec["shutter_limit"]
        assert lim["v_cap_mps"] == pytest.approx(v_cap)
        assert lim["v_g_needed_mps"] == pytest.approx(v_min, abs=1e-6)
        assert lim["f_req_hz"] == pytest.approx(v_min / (L120 * 0.15), rel=1e-6)
        assert lim["h_req_m"] == pytest.approx(120.0 * v_min / v_cap, rel=1e-6)
        assert lim["forward_overlap_max"] == pytest.approx(1.0 - v_min / L120, rel=1e-6)
    msgs = [e for e in rep.errors if e.startswith("shutter_rate_caps_speed(")]
    assert len(msgs) == 2
    assert "需觸發率 ≥ 1.27 Hz" in msgs[0] and "此路不通" in msgs[0] and "前向重疊 ≤ 0.81" in msgs[0]
    assert "需觸發率 ≥ 1.13 Hz" in msgs[1] and "136 m" in msgs[1]
    rep_w = preflight(_inputs(None, fwd=0.85, cam=_CAM_1HZ, waiver=200.0), CFG)
    assert rep_w.status == "NO_GO" and any("或作業高度 ≥ 153 m（可用上限 194 m）" in e for e in rep_w.errors)


def test_shutter_cap_single_class_is_warning_only():
    rep = preflight(_inputs(None, fwd=0.82, cam=_CAM_1HZ), CFG)
    assert rep.status != "NO_GO"
    a = rep.per_area["Z1"]["classes"]["A"]
    b = rep.per_area["Z1"]["classes"]["B"]
    assert a["reason"] == "no_feasible_direction" and b["feasible"] and "shutter_limit" not in b
    assert a["shutter_limit"]["f_req_hz"] == pytest.approx(18.0 / (L120 * 0.18), rel=1e-6)
    assert sum(w.startswith("shutter_rate_caps_speed(Z1, A:") for w in rep.warnings) == 1
    assert not any(e.startswith("shutter_rate_caps_speed") for e in rep.errors)
    ok = preflight(_inputs(None, fwd=0.80), CFG)
    assert not any("shutter_rate_caps_speed" in x for x in ok.warnings + ok.errors)
    assert all("shutter_limit" not in c for c in ok.per_area["Z1"]["classes"].values())


def test_preflight_records_mapping_effective_values():
    rep = preflight(_inputs(MappingModeConfig(lead_in_m=20.0), split=2, wind=(5.0, 270.0)), CFG)
    b = rep.derived["B"]["mapping"]
    assert b["crab_deg"] == pytest.approx(15.2575, abs=1e-3) and b["w_eff_m"] == pytest.approx(96.504, abs=1e-3)
    assert b["s_m"] == pytest.approx(28.951, abs=1e-3) and b["seam_overlap_target_m"] == pytest.approx(W120 - 28.951, abs=1e-3)
    assert b["lead_in_m"] == 20.0 and b["l_m"] == pytest.approx(L120)
    off = preflight(_inputs(None, split=2, wind=(5.0, 270.0)), CFG)
    assert b["e_ext_m"] - off.derived["B"]["e_ext"] == pytest.approx(20.0)
    assert "mapping" not in off.derived["B"] and "mapping_seam" not in off.per_area["Z1"]
    seam = rep.per_area["Z1"]["mapping_seam"]
    tie = W120 - 28.951
    assert seam["overlap_target_m"] == pytest.approx(tie, abs=1e-3)
    assert seam["delta_cell_m"] == pytest.approx(tie / 2.0, abs=1e-3) and seam["neighbor_growth_m"] == pytest.approx(tie / 2.0, abs=1e-3)
    assert seam["delta_residual_m"] == pytest.approx(tie, abs=1e-3) and seam["delta_area_outer_m"] == 15.0
    assert set(rep.derived) == {"A", "B"}                                    # derived 的鍵仍只有構型
