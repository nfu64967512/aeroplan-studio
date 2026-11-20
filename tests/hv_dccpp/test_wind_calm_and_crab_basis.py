"""2026-10-20 vendored 修改的數值鎖：(1) 無風基準 —— W < 2 m/s 時起降轉換方位與掃描軸候選不跟輸入風向；
(2) 建圖偏流餘裕依側風分量（``MappingModeConfig.crab_margin_basis="crosswind"``；**不是預設**——預設仍是舊的
``"max_speed"``，評估第 5.2 節說明為什麼）。

評估與實飛依據：docs/評估_2026-10-20_HARVEST偏流餘裕與無風基準.md。

「有風的規劃逐位元不變」用**值鎖**：``_GOLDEN`` 是用修改前的程式（HEAD b15070b 的 hv_dccpp）對同一組輸入算出的
整份計畫摘要（plans 的 to_dict JSON 之 sha256 前 20 碼）。日後刻意改了有風的規劃時才更新它，並在 VENDOR.md 記一筆。
"""
from __future__ import annotations

import dataclasses
import hashlib
import json
import math

import pytest

from hv_dccpp import altitude as ALT
from hv_dccpp import decompose as D
from hv_dccpp.context import build_context
from hv_dccpp.integration.aeroplan_adapter import to_mission_inputs
from hv_dccpp.mission import MappingModeConfig, MissionInputError, PayloadRequirement, PlannerConfig, WindLayer
from hv_dccpp.planner import plan_mission
from hv_dccpp.platform import PLATFORM_HERO, PLATFORM_PIONEER, CameraHardware, crabbed_width_m
from hv_dccpp.scenario import build_mini_example
from hv_dccpp.wind import CALM_WIND_MPS, WindField, crosswind_max_mps, reference_dir_from_rad

# 建圖情境：與 tests/hv_dccpp/test_mapping_mode.py 相同（HQ6@120 m：w 125.736、l 94.24；側向 0.70／前向 0.80；
# 1 km × 1 km、條帶東西向；1 Pioneer（min_fw 100）+ 2 HERO）
HOME = (23.95, 120.90)
_M_LAT = 111132.0
_M_LON = 111412.0 * math.cos(math.radians(HOME[0]))
HQ6 = CameraHardware(focal_length_mm=6.0, sensor_width_mm=4056 * 1.55e-3, sensor_height_mm=3040 * 1.55e-3,
                     pixel_pitch_um=1.55, max_shutter_rate_hz=2.0, is_default_example=False)
W120 = 120.0 * 4056 * 1.55e-3 / 6.0          # 125.736 m
L120 = 120.0 * 3040 * 1.55e-3 / 6.0          # 94.24 m
SIDE = 0.70
CFG = PlannerConfig(dem_step_m=25.0, dh_band_min_m=15.0, d_sep_m=50.0, dh_sep_m=50.0)
CLASSES = {1: "A", 2: "B", 3: "B"}
VA = {"A": PLATFORM_PIONEER.cruise_airspeed_mps, "B": PLATFORM_HERO.cruise_airspeed_mps}   # 21.5 / 19.0
XW = MappingModeConfig(crab_margin_basis="crosswind")

# 修改前的程式算出的計畫摘要（見模組說明）
_GOLDEN = {
    "mini_5_270": "0431f787c4f3ac0dae3d", "mini_5_0": "297c298f2e536638126f", "mini_2_90": "1e73cb9b4aa2748d5bd5",
    "mini_0_0": "c86d47f54fb43bd54c2e",
    "map_maxspeed_5_270": "3c5860cf288bc4eee749", "map_maxspeed_5_0": "d50ea599562b101a41c6",
    "map_maxspeed_3_45": "78a14ff9a1be1ace22b1",
}


def _ll(e, n):
    return (HOME[0] + n / _M_LAT, HOME[1] + e / _M_LON)


def _map_inputs(mapping, wind, km=1.0):
    h = km * 500.0
    areas = [{"area_id": 1, "priority": 1.0, "polygon": [_ll(-h, -h), _ll(h, -h), _ll(h, h), _ll(-h, h)]}]
    drones = [{"drone_id": i + 1, "name": f"UAV-{i + 1}", "position": _ll(-60.0 + 60.0 * i, -h - 400.0),
               "speed": 20.0, "heading": 0.0, "turn_radius": 150.0, "vehicle_type": "vtol", "fov_width": 125.7}
              for i in range(3)]
    req = PayloadRequirement("default", "rgb", gsd_required_m=0.05, side_overlap=SIDE, forward_overlap=0.80,
                             h_target_agl_m=120.0)
    inputs, _ = to_mission_inputs(areas, drones, coverage_width_m=125.7, overlap_rate=SIDE, coverage_altitude=120.0,
                                  home=_ll(0.0, -h - 400.0), class_of=lambda d: CLASSES[int(d["drone_id"])],
                                  platform_a=dataclasses.replace(PLATFORM_PIONEER, min_fw_operation_agl_m=100.0),
                                  platform_b=PLATFORM_HERO, sensor_a=(HQ6,), sensor_b=(HQ6,),
                                  payload_reqs={"default": req}, wind=wind, waiver_max_agl_m=None,
                                  dem_margin_m=3000.0, mapping_mode=mapping)
    return inputs


def _mini(spd, dir_from):
    inputs, cfg = build_mini_example()
    return dataclasses.replace(inputs, wind=(WindLayer(0.0, 3000.0, spd, dir_from, 1.3 * spd, "agl"),)), cfg


def _digest(plan) -> str:
    d = {"makespan": plan.makespan_s, "plans": plan.to_dict()["plans"]}
    return hashlib.sha256(json.dumps(d, sort_keys=True, default=str).encode()).hexdigest()[:20]


def _layer(spd, dir_from):
    return WindLayer(0.0, 3000.0, spd, dir_from, 1.3 * spd, "agl")


@pytest.fixture(scope="module")
def mini_ctx():
    inputs, cfg = build_mini_example()
    return build_context(inputs, dataclasses.replace(cfg, dem_step_m=50.0), terrain_margin_m=1500.0)


def _with_wind(ctx, *layers):
    return dataclasses.replace(ctx, wind=WindField(list(layers), ctx.frame, ctx.wind.z_ground_home_m))


# ---------------------------------------------------------------------------
# 1. 無風基準
# ---------------------------------------------------------------------------
def test_reference_dir_calm_is_north_and_windy_is_unchanged(mini_ctx):
    fr = mini_ctx.frame
    assert CALM_WIND_MPS == 2.0 and ALT._CALM_WIND_MPS == CALM_WIND_MPS
    assert reference_dir_from_rad(WindField([], fr, 0.0), 100.0) == 0.0
    for d in (0.0, 90.0, 270.0, 123.4):
        assert reference_dir_from_rad(WindField([_layer(0.0, d)], fr, 0.0), 100.0) == 0.0
        assert reference_dir_from_rad(WindField([_layer(1.99, d)], fr, 0.0), 100.0) == 0.0
        w2 = WindField([_layer(2.0, d)], fr, 0.0)                     # 門檻本身不算無風（舊碼 w >= 2 → θ = β）
        assert reference_dir_from_rad(w2, 100.0) == w2.dir_from_at(100.0) == math.radians(d)


def test_direction_candidates_calm_ignore_wind_dir(mini_ctx):
    poly = mini_ctx.areas_enu["Z1"]
    base = D.direction_candidates(poly, _with_wind(mini_ctx).decompose_ctx(), 260.0)        # 沒有風層
    for spd, d in ((0.0, 270.0), (0.0, 37.0), (1.5, 90.0)):
        got = D.direction_candidates(poly, _with_wind(mini_ctx, _layer(spd, d)).decompose_ctx(), 260.0)
        assert got == base, (spd, d)
    # 有風：前四個仍是 β、β+π、β+90°、β−90°（舊行為）
    windy = D.direction_candidates(poly, _with_wind(mini_ctx, _layer(5.0, 300.0)).decompose_ctx(), 260.0)
    b = math.radians(300.0)
    assert windy[:4] == [D._mod2pi(b), D._mod2pi(b + math.pi), D._mod2pi(b + 0.5 * math.pi), D._mod2pi(b - 0.5 * math.pi)]
    assert windy != base


def test_corridor_heading_calm_without_keepouts_is_north(mini_ctx):
    spec = mini_ctx.specs["A"]
    xy, z = mini_ctx.sites_enu["H"]
    ctx0 = dataclasses.replace(mini_ctx, keepouts=None)
    for d in (0.0, 90.0, 270.0):
        for spd in (0.0, 1.9):
            th, issues = ALT._corridor_heading(_with_wind(ctx0, _layer(spd, d)), spec, "H", xy, z + 100.0, 300.0, False)
            assert th == 0.0 and issues == [], (spd, d)
        th5, _ = ALT._corridor_heading(_with_wind(ctx0, _layer(5.0, d)), spec, "H", xy, z + 100.0, 300.0, True)
        assert th5 == math.radians(d)                                             # 有風：仍逆風（舊行為）


def test_calm_plan_is_identical_for_any_wind_direction():
    """0 m/s 來自 270° 與來自 0° 的計畫逐位元相同（修改前：規劃矩陣 945 s vs 1209 s）。"""
    p0 = plan_mission(*_mini(0.0, 0.0))
    p270 = plan_mission(*_mini(0.0, 270.0))
    assert _digest(p270) == _digest(p0)
    ctx = build_context(*_mini(0.0, 270.0))
    assert any(w.startswith("calm_wind_direction_ignored(規劃風 0 m/s < 2 m/s") and "270°" in w for w in ctx.warnings)
    assert not any(w.startswith("calm_wind_direction_ignored") for w in build_context(*_mini(0.0, 0.0)).warnings)
    assert not any(w.startswith("calm_wind_direction_ignored") for w in build_context(*_mini(5.0, 270.0)).warnings)


@pytest.mark.parametrize("key,spd,d", [("mini_5_270", 5.0, 270.0), ("mini_5_0", 5.0, 0.0), ("mini_2_90", 2.0, 90.0),
                                       ("mini_0_0", 0.0, 0.0)])
def test_windy_and_north_calm_plans_unchanged(key, spd, d):
    """有風（含門檻 2.0 m/s）與 0 m/s 來自 0° 的計畫 = 修改前（值鎖）。"""
    assert _digest(plan_mission(*_mini(spd, d))) == _GOLDEN[key]


# ---------------------------------------------------------------------------
# 2. 建圖偏流餘裕：側風分量
# ---------------------------------------------------------------------------
def test_crosswind_component(mini_ctx):
    fr = mini_ctx.frame
    w = WindField([_layer(5.0, 270.0)], fr, 0.0)
    assert crosswind_max_mps(w, math.pi / 2) == 0.0 and crosswind_max_mps(w, 3 * math.pi / 2) == 0.0   # 沿風
    assert crosswind_max_mps(w, 0.0) == pytest.approx(5.0, abs=1e-12)                                   # 純側風
    assert crosswind_max_mps(w, math.pi / 4) == pytest.approx(5.0 / math.sqrt(2.0), abs=1e-12)
    two = WindField([WindLayer(0.0, 100.0, 3.0, 0.0, 3.9, "agl"), WindLayer(100.0, 3000.0, 4.0, 270.0, 5.2, "agl")], fr, 0.0)
    assert crosswind_max_mps(two, math.pi / 2) == pytest.approx(3.0)      # 取所有層的最大側風（同 max_speed 不分高度）
    assert crosswind_max_mps(WindField([], fr, 0.0), 0.3) == 0.0


def test_basis_config_values_and_refusals():
    m = MappingModeConfig()
    assert m.crab_margin_basis == "max_speed" and m == MappingModeConfig(crab_margin_basis="max_speed")
    assert "crab_margin_basis" not in m.to_dict()                # 預設＝舊行為 ⇒ to_dict 與加入前逐位元相同
    assert XW.to_dict()["crab_margin_basis"] == "crosswind"
    assert list(XW.to_dict()).index("crab_margin_basis") == list(XW.to_dict()).index("crab_margin") + 1
    with pytest.raises(MissionInputError):
        MappingModeConfig(crab_margin_basis="direction")
    with pytest.raises(MissionInputError):                    # 沒開偏流餘裕卻設基準 ⇒ 不會生效的值，拒絕
        MappingModeConfig(crab_margin=False, crab_margin_basis="crosswind")
    assert "crab_margin_basis" not in MappingModeConfig(crab_margin=False).to_dict()
    ctx = build_context(_map_inputs(XW, (3.0, 270.0)), CFG)
    assert any(w == "mapping_mode(seam_tie=True, no_altitude_layering=True, crab_margin=True, crab_margin_basis=crosswind, "
               "lead_in_m=0.0, min_block_strips=3)" for w in ctx.warnings)
    assert any(w.startswith("mapping_crab_margin(B/default: ψ=9.08°") and w.endswith("見 mapping_crab_effective)")
               for w in ctx.warnings)


@pytest.mark.parametrize("dir_from", [270.0, 0.0, 45.0])
def test_track_sensor_uses_crosswind_component(dir_from):
    ctx = build_context(_map_inputs(XW, (5.0, dir_from)), CFG)
    dc = ctx.decompose_ctx()
    old = build_context(_map_inputs(MappingModeConfig(), (5.0, dir_from)), CFG).decompose_ctx()
    for cls in ("A", "B"):
        env = ctx.sensors[(cls, "default")]
        assert env.crab_rad == pytest.approx(math.asin(5.0 / VA[cls]))              # 全方向最壞值不變（給不知方向的檢查）
        assert old.sensors[(cls, "default")] == env
        for alpha_deg in (90.0, 0.0, 45.0, 135.0):
            a = math.radians(alpha_deg)
            w_perp = abs(5.0 * math.sin(a - math.radians(dir_from)))
            w_perp = 0.0 if w_perp < 1e-9 else w_perp
            psi = math.asin(w_perp / VA[cls])
            ts = D.track_sensor(dc, cls, env, a)
            assert ts.crab_rad == pytest.approx(psi, abs=1e-12)
            assert ts.strip_spacing_m(120.0) == pytest.approx(crabbed_width_m(W120, L120, psi) * (1 - SIDE), abs=1e-9)
            assert ts.strip_spacing_m(120.0) >= env.strip_spacing_m(120.0) - 1e-12     # 只會變寬或不變
            assert D.track_sensor(old, cls, env, a) is env                              # 舊基準：原樣
    off = build_context(_map_inputs(None, (5.0, dir_from)), CFG).decompose_ctx()
    s_off = off.sensors[("B", "default")]
    assert D.track_sensor(off, "B", s_off, 0.3) is s_off                               # 非建圖：原樣


def test_proxy_time_ranks_directions_with_their_own_spacing():
    """方向評分（proxy_time）也要用該方向的側風分量：沿風方向的條帶數 = 無偏流餘裕，側風方向 = 舊基準。"""
    ctx = build_context(_map_inputs(XW, (5.0, 270.0)), CFG)
    dc = ctx.decompose_ctx()
    old = build_context(_map_inputs(MappingModeConfig(), (5.0, 270.0)), CFG).decompose_ctx()
    nocrab = build_context(_map_inputs(MappingModeConfig(crab_margin=False), (5.0, 270.0)), CFG).decompose_ctx()
    poly = next(iter(ctx.areas_enu.values()))
    for cls in ("A", "B"):
        along = [D.proxy_time(poly, math.pi / 2, cls, 120.0, "default", c).n_strips for c in (dc, nocrab, old)]
        assert along[0] == along[1] and along[0] < along[2], (cls, along)        # 新 = 無餘裕 < 舊
        cross = [D.proxy_time(poly, 0.0, cls, 120.0, "default", c).n_strips for c in (dc, old)]
        assert cross[0] == cross[1], (cls, cross)                                 # 純側風：與舊基準相同


def test_along_wind_exactly_matches_no_margin_spacing():
    """沿風的條帶間距與「沒有偏流餘裕」逐位元相同（ψ = 0.0 ⇒ crabbed_width_m 原樣回傳 w）。"""
    dc = build_context(_map_inputs(XW, (5.0, 270.0)), CFG).decompose_ctx()
    nocrab = build_context(_map_inputs(MappingModeConfig(crab_margin=False), (5.0, 270.0)), CFG).decompose_ctx()
    for cls in ("A", "B"):
        ts = D.track_sensor(dc, cls, dc.sensors[(cls, "default")], math.pi / 2)
        assert ts.crab_rad == 0.0
        assert ts.strip_spacing_m(120.0) == nocrab.sensors[(cls, "default")].strip_spacing_m(120.0)


@pytest.mark.parametrize("wind", [(5.0, 270.0), (5.0, 0.0), (3.0, 45.0)])
def test_flown_spacing_follows_strip_crosswind_and_is_reported(wind):
    """端到端：每條實際飛的條帶間距 = w_eff(ψ(該條航跡)) (1 − side)；有效值警告逐軸列出同樣的數。"""
    plan = plan_mission(_map_inputs(XW, wind), CFG)
    flown = [plan.strips[s] for p in plan.plans.values() for s, _d in p.strips]
    assert flown
    for st in flown:
        w_perp = abs(wind[0] * math.sin(float(st.theta_LR_rad) - math.radians(wind[1])))
        psi = math.asin(w_perp / VA[st.cls]) if w_perp >= 1e-9 else 0.0
        want = crabbed_width_m(float(st.width_m), float(st.width_m) * L120 / W120, psi) * (1 - SIDE)
        assert float(st.spacing_m) == pytest.approx(want, abs=0.02), (st.strip_id, math.degrees(st.theta_LR_rad))
    eff = [w for w in plan.feasibility.warnings if str(w).startswith("mapping_crab_effective(")]
    assert len(eff) == 1 and "crosswind 基準" in eff[0]
    old = plan_mission(_map_inputs(MappingModeConfig(), wind), CFG)
    assert not any(str(w).startswith("mapping_crab_effective(") for w in old.feasibility.warnings)


@pytest.mark.parametrize("key,wind", [("map_maxspeed_5_270", (5.0, 270.0)), ("map_maxspeed_5_0", (5.0, 0.0)),
                                      ("map_maxspeed_3_45", (3.0, 45.0))])
def test_default_basis_is_the_old_planner(key, wind):
    """預設（= ``crab_margin_basis="max_speed"``）的建圖計畫 = 修改前的建圖計畫（值鎖）。"""
    assert _digest(plan_mission(_map_inputs(MappingModeConfig(), wind), CFG)) == _GOLDEN[key]


def test_along_wind_mapping_plan_needs_fewer_strips():
    """沿風 5 m/s：舊基準條帶間距縮 20%／23%；新基準 = 無偏流餘裕的間距，條帶比舊基準少。"""
    new = plan_mission(_map_inputs(XW, (5.0, 270.0)), CFG)
    old = plan_mission(_map_inputs(MappingModeConfig(), (5.0, 270.0)), CFG)
    n_new = sum(len(p.strips) for p in new.plans.values())
    n_old = sum(len(p.strips) for p in old.plans.values())
    assert n_new < n_old
    axes_new = {round(math.degrees(new.strips[s].theta_LR_rad) % 180.0) for p in new.plans.values() for s, _d in p.strips}
    assert axes_new == {90}                                    # 東西向（沿風），偏流 0 ⇒ 間距 = w(1 − side)
    for p in new.plans.values():
        for s, _d in p.strips:
            assert new.strips[s].spacing_m == pytest.approx(new.strips[s].width_m * (1 - SIDE), abs=1e-9)
