"""建圖模式經轉接層（HvDccppCoordinator / to_mission_inputs）的真規劃驗收。

情境：2×2 km 平地、1 Pioneer（min_fw 100）+ 2 HERO、HQ IMX477 + 6 mm（120 m：w 125.736、l 94.24 m、2 Hz）、
側向 0.70／前向 0.80、無豁免、坪相距 60 m。``split3`` ＝ 像 GUI 的 split_area_for_fleet 一樣切成三塊相鄰子區。

接縫量法：兩架的足跡聯集 FP_a、FP_b（只看任務區內），「只有 a 看到」與「只有 b 看到」兩塊之間的最短距離
＝ 接縫上**最窄處**兩架共同看到的帶寬。建圖模式要求它 ≥ side_overlap·w（= 88.0 m）。
"""
from __future__ import annotations

import dataclasses
import itertools
import math
import os

import numpy as np
import pytest
import shapely
from shapely.ops import unary_union

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from hv_dccpp.geometry import strips_union_footprint  # noqa: E402
from hv_dccpp.integration.aeroplan_adapter import (  # noqa: E402
    AeroPlanAdapterError, HvDccppCoordinator, to_mission_inputs,
)
from hv_dccpp.mission import (  # noqa: E402
    MappingMissionInputs, MappingModeConfig, MissionInputs, PayloadRequirement, PlannerConfig,
)
from hv_dccpp.platform import PLATFORM_HERO, PLATFORM_PIONEER, CameraHardware  # noqa: E402

HOME = (23.95, 120.90)
_M_LAT = 111132.0
_M_LON = 111412.0 * math.cos(math.radians(HOME[0]))
HQ6 = CameraHardware(focal_length_mm=6.0, sensor_width_mm=4056 * 1.55e-3, sensor_height_mm=3040 * 1.55e-3,
                     pixel_pitch_um=1.55, max_shutter_rate_hz=2.0, is_default_example=False)
W120 = 120.0 * 4056 * 1.55e-3 / 6.0
SIDE = 0.70
NEED = SIDE * W120                       # 88.0152 m
CLASSES = {1: "A", 2: "B", 3: "B"}


def _ll(e, n):
    return (HOME[0] + n / _M_LAT, HOME[1] + e / _M_LON)


def _areas(km=2.0, split=1):
    h = km * 500.0
    xs = [-h + 2 * h * i / split for i in range(split + 1)]
    return [{"area_id": i + 1, "priority": 1.0, "polygon": [_ll(x0, -h), _ll(x1, -h), _ll(x1, h), _ll(x0, h)]}
            for i, (x0, x1) in enumerate(zip(xs, xs[1:]))]


def _drones(km=2.0):
    return [{"drone_id": i + 1, "name": f"UAV-{i + 1}", "position": _ll(-60.0 + 60.0 * i, -km * 500.0 - 400.0),
             "speed": 20.0, "heading": 0.0, "turn_radius": 150.0, "vehicle_type": "vtol", "fov_width": 125.7}
            for i in range(3)]


def _coord(mapping=None, *, wind=(0.0, 0.0), reports=None, km=2.0):
    req = PayloadRequirement("default", "rgb", gsd_required_m=0.05, side_overlap=SIDE, forward_overlap=0.80,
                             h_target_agl_m=120.0)
    kw = dict(class_of=lambda d: CLASSES[int(d["drone_id"])],
              platform_a=dataclasses.replace(PLATFORM_PIONEER, min_fw_operation_agl_m=100.0), platform_b=PLATFORM_HERO,
              sensor_a=(HQ6,), sensor_b=(HQ6,), payload_reqs={"default": req},
              cfg=PlannerConfig(dem_step_m=25.0, dh_band_min_m=15.0, d_sep_m=50.0, dh_sep_m=50.0),
              home=_ll(0.0, -km * 500.0 - 400.0), wind=wind, waiver_max_agl_m=None, dem_margin_m=3000.0,
              on_report=(reports.append if reports is not None else None))
    if mapping is not None:
        kw["mapping_mode"] = mapping
    return HvDccppCoordinator(**kw)


def _plan(mapping=None, *, split=1, km=2.0, wind=(0.0, 0.0)):
    reports: list = []
    hv = _coord(mapping, wind=wind, reports=reports, km=km)
    hv.plan_coverage_dccpp(_areas(km, split), _drones(km), coverage_width_m=125.7, overlap_rate=SIDE,
                           coverage_altitude=120.0)
    return hv, reports


@pytest.fixture(scope="module")
def plans():
    return {
        "split3_off": _plan(None, split=3),
        "split3_on": _plan(MappingModeConfig(), split=3),
        "single_off": _plan(None, split=1),
        "single_on": _plan(MappingModeConfig(), split=1),
    }


def _flown(hv):
    p = hv.last_plan
    return {u: [p.strips[sid] for sid, _d in up.strips] for u, up in p.plans.items() if up.strips}


def _seams(hv):
    """{(a, b): 最窄共同帶寬}，只列任務區內足跡有重疊的機對。"""
    aoi = unary_union(list(hv.last_ctx.areas_enu.values()))
    fps = {u: strips_union_footprint(st) for u, st in _flown(hv).items()}
    out = {}
    for a, b in itertools.combinations(sorted(fps), 2):
        inter = fps[a].intersection(fps[b]).intersection(aoi)
        if inter.area < 1.0:
            continue
        ra = fps[a].difference(fps[b]).intersection(aoi)
        rb = fps[b].difference(fps[a]).intersection(aoi)
        out[(a, b)] = float(ra.distance(rb)) if (ra.area >= 1.0 and rb.area >= 1.0) else math.inf
    return out


def _frac_ge3(hv, grid=10.0):
    aoi = unary_union(list(hv.last_ctx.areas_enu.values()))
    x0, y0, x1, y1 = aoi.bounds
    gx, gy = np.meshgrid(np.arange(x0 + grid / 2, x1, grid), np.arange(y0 + grid / 2, y1, grid))
    P = np.column_stack([gx.ravel(), gy.ravel()])
    P = P[shapely.contains_xy(aoi, P[:, 0], P[:, 1])]
    cnt = np.zeros(len(P), dtype=int)
    for st in _flown(hv).values():
        for s in st:
            t = np.array([math.sin(s.theta_LR_rad), math.cos(s.theta_LR_rad)])
            n = np.array([t[1], -t[0]])
            d = P - s.pL
            L = float((s.pR - s.pL) @ t)
            cnt += ((d @ t >= 0) & (d @ t <= L) & (np.abs(d @ n) <= s.width_m / 2)).astype(int)
    return float((cnt >= 3).mean())


# ---------------------------------------------------------------------------
# 接縫重疊（規格項 1）
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("case", ["split3", "single"])
def test_seam_overlap_reaches_side_overlap_times_swath(plans, case):
    off, _ = plans[f"{case}_off"]
    on, _ = plans[f"{case}_on"]
    s_off = _seams(off)
    assert s_off and min(s_off.values()) < 0.5 * NEED, s_off            # 舊行為：跨機接縫只有十幾～二十幾 m
    s_on = _seams(on)
    assert s_on, "建圖模式下至少要有一條跨機接縫"
    assert min(s_on.values()) >= NEED - 0.01, s_on
    assert on.last_plan.uncovered == []
    # 同區塊內相鄰條帶的側向重疊不變（w − s = 0.70 w），接縫只是追上它
    sp = [s.spacing_m for st in _flown(on).values() for s in st]
    assert max(abs(x - W120 * 0.3) for x in sp) < 0.02, sp             # 局部包絡讓 h 差到 ~0.01 m


def test_more_ground_seen_by_three_strips(plans):
    off = _frac_ge3(plans["split3_off"][0])
    on = _frac_ge3(plans["split3_on"][0])
    assert on > off + 0.1 and on > 0.9, (off, on)


# ---------------------------------------------------------------------------
# 轉接層管線與有效值（規格項 7）
# ---------------------------------------------------------------------------
def test_default_path_has_no_mapping(plans):
    hv, reports = plans["split3_off"]
    assert type(hv.last_inputs) is MissionInputs and hv.mapping_mode is None and hv.mapping_effective is None
    assert not any("mapping" in n for n in hv.notes)
    assert "mapping_mode" not in reports[-1]
    inputs, _ = to_mission_inputs(_areas(), _drones(), coverage_width_m=125.7, overlap_rate=SIDE,
                                  coverage_altitude=120.0, class_of=lambda d: CLASSES[int(d["drone_id"])])
    assert type(inputs) is MissionInputs


def test_mapping_reaches_planner_and_is_recorded(plans):
    hv, reports = plans["split3_on"]
    assert type(hv.last_inputs) is MappingMissionInputs and hv.last_inputs.mapping_mode is hv.mapping_mode
    eff = hv.mapping_effective
    assert eff["config"] == MappingModeConfig().to_dict()
    for c in ("A", "B"):
        m = eff["per_class"][c]
        assert m["crab_deg"] == 0.0 and m["w_m"] == pytest.approx(W120) and m["s_m"] == pytest.approx(W120 * 0.3)
        assert m["seam_overlap_target_m"] == pytest.approx(NEED)
    for aid in ("Z1", "Z2", "Z3"):
        s = eff["seam_by_area"][aid]
        assert s["overlap_target_m"] == pytest.approx(NEED) and s["delta_cell_m"] == pytest.approx(NEED / 2)
        assert s["neighbor_growth_m"] == pytest.approx(NEED / 2) and s["delta_residual_m"] == pytest.approx(NEED)
        assert s["delta_area_outer_m"] == 15.0
    assert reports[-1]["mapping_mode"] == eff
    note = next(n for n in hv.notes if n.startswith("mapping_mode_effective("))
    assert "接縫重疊目標 88.0 m" in note and "s=37.7 m" in note
    assert "preflight:mapping_mode(seam_tie=True, no_altitude_layering=True, crab_margin=True, lead_in_m=0.0, " \
           "min_block_strips=3)" in hv.notes
    inputs, _ = to_mission_inputs(_areas(), _drones(), coverage_width_m=125.7, overlap_rate=SIDE,
                                  coverage_altitude=120.0, class_of=lambda d: CLASSES[int(d["drone_id"])],
                                  mapping_mode=MappingModeConfig(lead_in_m=12.0))
    assert type(inputs) is MappingMissionInputs and inputs.mapping_mode.lead_in_m == 12.0


def test_rejects_non_config_mapping_mode():
    with pytest.raises(AeroPlanAdapterError, match="mapping_mode"):
        _coord({"seam_tie": True})
    with pytest.raises(AeroPlanAdapterError, match="mapping_mode"):
        to_mission_inputs(_areas(), _drones(), coverage_width_m=125.7, mapping_mode="on",
                          class_of=lambda d: CLASSES[int(d["drone_id"])])


def test_lead_in_reaches_strip_ends():
    hv, _ = _plan(MappingModeConfig(lead_in_m=40.0), km=1.0)
    eff = hv.mapping_effective["per_class"]
    assert eff["B"]["lead_in_m"] == 40.0 and eff["B"]["e_ext_m"] == pytest.approx(38.0 + 40.0)
    assert eff["A"]["e_ext_m"] == pytest.approx(43.0 + 40.0)
    cells = {c.cell_id: c for c in hv.last_plan.cells}
    over = []
    for st in _flown(hv).values():
        for s in st:
            line = shapely.LineString([s.pL, s.pR])
            inside = line.intersection(cells[s.cell_id].polygon)
            t = (s.pR - s.pL) / s.length_m
            u = np.asarray(shapely.get_coordinates(inside)) @ t - float(s.pL @ t)
            e = eff[s.cls]["e_ext_m"]
            assert u.min() >= e - 1e-6 and s.length_m - u.max() >= e - 1e-6
            over.append(u.min() - e)
    assert min(over) == pytest.approx(0.0, abs=1e-6)            # 有條帶正好只外伸 e_ext + lead_in


# ---------------------------------------------------------------------------
# 偏流餘裕（規格項 3）與未覆蓋警告
# ---------------------------------------------------------------------------
def test_crab_margin_spacing_in_flown_strips():
    on, _ = _plan(MappingModeConfig(), km=1.0, wind=(5.0, 270.0))
    off, _ = _plan(None, km=1.0, wind=(5.0, 270.0))
    def spacings(hv, cls):
        return [s.spacing_m for u, st in _flown(hv).items() if u[0] == cls for s in st]

    def close(xs, want):                                              # 局部包絡讓 h 差到 ~0.01 m
        return bool(xs) and max(abs(x - want) for x in xs) < 0.02

    assert close(spacings(on, "A"), 30.112) and close(spacings(on, "B"), 28.951)   # w_eff(1 − 0.7)，ψ 13.45°／15.26°
    assert close(spacings(off, "A"), 37.721) and close(spacings(off, "B"), 37.721)
    assert on.mapping_effective["per_class"]["B"]["crab_deg"] == pytest.approx(15.2575, abs=1e-3)
    if on.last_plan.uncovered:
        assert any(w.startswith("mapping_incomplete(") for w in on.last_plan.feasibility.warnings)
    else:
        assert not any(w.startswith("mapping_incomplete(") for w in on.last_plan.feasibility.warnings)
