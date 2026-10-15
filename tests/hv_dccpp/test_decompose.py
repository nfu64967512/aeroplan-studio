"""tests/test_decompose.py — decompose.py（契約 §7、規格 §4.6／§7／§9.2、D3/D12/D15/D16/D17/D38/D41）。

規格 §17.2 清單：direction_min_width_no_wind、direction_changes_with_wind、wind_axis_candidates_present、
slope_penalty_finite、delta_z_penalty_orbits、turn_band_omega_overshoot、one_way_feasible_cell_proxy_finite、
altitude_band_polygon_level、residual_candidate_pairs；另加契約 §W.3 數值、L 形分割、禁區、exact_time 定義、
suitability、sensor_compatible 真值表、效能。
"""
from __future__ import annotations

import dataclasses
import math
import time

import numpy as np
import pytest
import shapely
from shapely.geometry import Polygon

from hv_dccpp import decompose as D
from hv_dccpp.context import build_context
from hv_dccpp.dubins import dubins_path, path_time
from hv_dccpp.geometry import min_width_direction, strips_union_footprint
from hv_dccpp.mission import Airspace, AreaSpec, NoFlyZone, WindLayer
from hv_dccpp.scenario import build_hills_scenario, build_mini_example, enu_ring_to_wgs84

# ---------------------------------------------------------------------------
# fixtures
# ---------------------------------------------------------------------------
_STEP = 50.0
_MARGIN = 1500.0


def _ctx(inputs, cfg):
    return build_context(inputs, dataclasses.replace(cfg, dem_step_m=_STEP), terrain_margin_m=_MARGIN)


@pytest.fixture(scope="module")
def mini():
    inputs, cfg = build_mini_example()
    ctx = _ctx(inputs, cfg)
    return inputs, cfg, ctx, ctx.decompose_ctx()


@pytest.fixture(scope="module")
def mini_calm():
    inputs, cfg = build_mini_example()
    inputs = dataclasses.replace(inputs, wind=())
    ctx = _ctx(inputs, cfg)
    return inputs, cfg, ctx, ctx.decompose_ctx()


@pytest.fixture(scope="module")
def mini_cells(mini):
    inputs, cfg, ctx, dc = mini
    cells = D.decompose_area(inputs.areas[0], dc, {"A", "B"}, polygon_enu=ctx.areas_enu["Z1"], rho_sym_m=ctx.rho_sym_m)
    return cells


@pytest.fixture(scope="module")
def hills():
    inputs, cfg = build_hills_scenario()
    ctx = _ctx(inputs, cfg)
    return inputs, cfg, ctx, ctx.decompose_ctx()


def _frame_of(ctx):
    return ctx.frame


def _mini_with_area(ring_enu, *, sensor_req="any", wind=None, holes=(), nofly=()):
    """以 mini 輸入為底、換區域多邊形（ENU 環）、風與禁區。"""
    inputs, cfg = build_mini_example()
    ctx0 = _ctx(inputs, cfg)
    frame = ctx0.frame
    area = AreaSpec("Z1", enu_ring_to_wgs84(frame, tuple(ring_enu)),
                    holes_wgs84=tuple(enu_ring_to_wgs84(frame, tuple(h)) for h in holes),
                    sensor_req=sensor_req, priority=1.0, payload_req_id="default")
    kw = {"areas": (area,)}
    if wind is not None:
        kw["wind"] = wind
    if nofly:
        zones = tuple(NoFlyZone(f"NF{i}", enu_ring_to_wgs84(frame, tuple(r))) for i, r in enumerate(nofly))
        kw["airspace"] = dataclasses.replace(inputs.airspace, no_fly=zones)
    inputs = dataclasses.replace(inputs, **kw)
    ctx = _ctx(inputs, cfg)
    return inputs, cfg, ctx, ctx.decompose_ctx()


# ---------------------------------------------------------------------------
# 高度帶（規格 §4.6、D12）、迴轉帶（D38）
# ---------------------------------------------------------------------------
def test_altitude_band_polygon_level(mini):
    """§W.3：A Δh_band = (100+200)−(100+120) = 80（ENU 曲率下垂 ≤ 4 m 使 Ω 內起伏 > 0）、h_op 160；B Δh 100、h_op 100。"""
    inputs, cfg, ctx, dc = mini
    poly = ctx.areas_enu["Z1"]
    a = D.altitude_band(poly, "A", "default", dc)
    b = D.altitude_band(poly, "B", "default", dc)
    assert a.feasible and a.reason == "" and a.h_op_m == pytest.approx(160.0)
    assert 74.0 <= a.dh_band_m <= 80.0 + 1e-6
    assert a.h_lo_m == pytest.approx(120.0) and 194.0 <= a.h_hi_m <= 200.0 + 1e-6
    assert b.feasible and b.h_op_m == pytest.approx(100.0) and 94.0 <= b.dh_band_m <= 100.0 + 1e-6
    assert b.h_lo_m == pytest.approx(100.0)
    # band 是胞多邊形層級：更大的 Ω（更寬胞）不改變平地結果
    big = poly.buffer(500.0)
    a2 = D.altitude_band(big, "A", "default", dc)
    assert a2.feasible and abs(a2.dh_band_m - a.dh_band_m) < 3.0


def test_A_needs_waiver_without_waiver():
    """D12：h_ceil = 120 且無豁免 → Δh_band ≤ 0 → A 不可行、reason A_needs_waiver；B 仍可行。"""
    inputs, cfg = build_mini_example()
    inputs = dataclasses.replace(inputs, airspace=Airspace(max_agl_m=120.0, waiver_max_agl_m=None, terrain_clearance_m=36.0))
    ctx = _ctx(inputs, cfg)
    dc = ctx.decompose_ctx()
    poly = ctx.areas_enu["Z1"]
    a = D.altitude_band(poly, "A", "default", dc)
    assert not a.feasible and a.reason == "A_needs_waiver" and a.dh_band_m <= 0.0
    b = D.altitude_band(poly, "B", "default", dc)
    assert not b.feasible or b.dh_band_m >= 20.0


def test_gsd_unattainable_band():
    """規格 §4.2：GSD 2 cm → h_max 82 m < h_minfw 100 + 20 → B reason gsd_unattainable。"""
    inputs, cfg = build_mini_example()
    reqs = dict(inputs.payload_reqs)
    reqs["default_rgb"] = dataclasses.replace(reqs["default_rgb"], gsd_required_m=0.02, h_target_agl_m=None)
    inputs = dataclasses.replace(inputs, payload_reqs=reqs)
    ctx = _ctx(inputs, cfg)
    dc = ctx.decompose_ctx()
    b = D.altitude_band(ctx.areas_enu["Z1"], "B", "default", dc)
    assert not b.feasible and b.reason == "gsd_unattainable"


def test_turn_band_omega_overshoot(mini):
    """D38：Ω 超出端線量 A@150（s 157.55）486.9 m、A@120（s 126）501 m、B（s 102.81）288 m（不含 e_ext）；
    §W.3 含 e_ext：A@160 534.7、B@100 336.0。"""
    inputs, cfg, ctx, dc = mini
    assert D.d_turn_from_r(200.0, 157.55, 0.0) == pytest.approx(486.9, abs=0.5)
    assert D.d_turn_from_r(200.0, 126.0, 0.0) == pytest.approx(501.0, abs=1.0)
    assert D.d_turn_from_r(120.0, 102.81, 0.0) == pytest.approx(288.0, abs=0.5)
    sa = dc.sensors[("A", "default")]
    sb = dc.sensors[("B", "default_rgb")]
    e_a = D.end_extension_for(dc, "A", sa, 160.0)
    e_b = D.end_extension_for(dc, "B", sb, 100.0)
    assert e_a == pytest.approx(53.0, abs=0.01) and e_b == pytest.approx(48.0, abs=0.01)
    assert D.d_turn_m("A", sa.strip_spacing_m(160.0), e_a, dc) == pytest.approx(534.7, abs=0.2)
    assert D.d_turn_m("B", sb.strip_spacing_m(100.0), e_b, dc) == pytest.approx(336.0, abs=0.2)
    # s ≥ 2R → 只剩 R + e_ext
    assert D.d_turn_from_r(200.0, 500.0, 10.0) == pytest.approx(210.0)


# ---------------------------------------------------------------------------
# 方向候選與代理（規格 §7.3、D15）
# ---------------------------------------------------------------------------
def test_wind_axis_candidates_present(mini):
    inputs, cfg, ctx, dc = mini
    poly = ctx.areas_enu["Z1"]
    cands = D.direction_candidates(poly, dc, 260.0)
    deg = sorted(round(math.degrees(a)) % 360 for a in cands)
    beta = 270
    for want in (beta, (beta + 180) % 360, (beta + 90) % 360, (beta - 90) % 360):
        assert any(abs(d - want) <= 1 for d in deg), (want, deg)
    a_minw, _ = min_width_direction(poly)
    assert any(min(abs(a - a_minw), abs(a - a_minw - math.pi), abs(a - a_minw + math.pi)) < math.radians(1.0) for a in cands)
    assert all(0.0 <= a < 2 * math.pi for a in cands)


def test_direction_min_width_no_wind(mini_calm):
    """W=0、平地：proxy_time 之 argmin = 最小寬度方向（mod π）。"""
    inputs, cfg, ctx, dc = mini_calm
    poly = ctx.areas_enu["Z1"]
    cands = D.direction_candidates(poly, dc, 260.0)
    px = [D.proxy_time(poly, a, "A", 160.0, "default", dc) for a in cands]
    best = min(px, key=lambda p: p.T_s)
    a_minw, _ = min_width_direction(poly)
    d = abs((best.alpha_rad - a_minw + math.pi / 2) % math.pi - math.pi / 2)
    assert d < math.radians(1.0)
    assert best.n_strips == 7 and best.one_way == 0 and best.pen_slope_s == 0.0 and best.pen_dz_s == 0.0
    # 無風 t_turn = min(Ω, U)/V_a = 732.5/21.5
    assert best.t_turn_s == pytest.approx(732.5 / 21.5, abs=0.1)


def test_proxy_numbers_with_wind(mini):
    """§W.3：東西向 n_A = 7、n_B = 12；南北向 A 18、B 29；t_turn = 732.5/21.5·(1+(5/21.5)²)。"""
    inputs, cfg, ctx, dc = mini
    poly = ctx.areas_enu["Z1"]
    pa_ew = D.proxy_time(poly, math.pi / 2, "A", 160.0, "default", dc)
    pa_ns = D.proxy_time(poly, 0.0, "A", 160.0, "default", dc)
    pb_ew = D.proxy_time(poly, math.pi / 2, "B", 100.0, "default", dc)
    pb_ns = D.proxy_time(poly, 0.0, "B", 100.0, "default", dc)
    assert (pa_ew.n_strips, pa_ns.n_strips, pb_ew.n_strips, pb_ns.n_strips) == (7, 18, 12, 29)
    assert pa_ew.T_s < pa_ns.T_s and pb_ew.T_s < pb_ns.T_s
    assert pa_ew.t_turn_s == pytest.approx(732.5 / 21.5 * (1 + (5 / 21.5) ** 2), abs=0.1)
    # 兩向作業時間：7 條 × 3136/2·(1/26.5 + 1/16.5) ≈ 1079 s（條帶長因 DP 角落 ≈ 3127–3136）
    t_oper = pa_ew.T_s - 6 * pa_ew.t_turn_s
    assert t_oper == pytest.approx(7 * 3131 * 0.5 * (1 / 26.5 + 1 / 16.5), rel=0.01)
    # 對稱：θ 與 θ+π 相同
    assert D.proxy_time(poly, 3 * math.pi / 2, "A", 160.0, "default", dc).T_s == pytest.approx(pa_ew.T_s, rel=1e-9)


def test_direction_changes_with_wind():
    """正方形胞：W=0 時南北／東西打平；W=8 側風（β±90°）往返因子 2L/√(V²−W²) < 2LV/(V²−W²) → 南北向勝出。"""
    square = ((4000.0, -1000.0), (6000.0, -1000.0), (6000.0, 1000.0), (4000.0, 1000.0))
    _i, _c, ctx0, dc0 = _mini_with_area(square, wind=())
    poly = ctx0.areas_enu["Z1"]
    p_ns0 = D.proxy_time(poly, 0.0, "A", 160.0, "default", dc0)
    p_ew0 = D.proxy_time(poly, math.pi / 2, "A", 160.0, "default", dc0)
    assert p_ns0.T_s == pytest.approx(p_ew0.T_s, rel=0.02)
    _i, _c, ctx8, dc8 = _mini_with_area(square, wind=(WindLayer(0.0, 3000.0, 8.0, 270.0, 9.0, "agl"),))
    poly8 = ctx8.areas_enu["Z1"]
    p_ns = D.proxy_time(poly8, 0.0, "A", 160.0, "default", dc8)
    p_ew = D.proxy_time(poly8, math.pi / 2, "A", 160.0, "default", dc8)
    assert p_ns.T_s < p_ew.T_s
    cands = D.direction_candidates(poly8, dc8, 260.0)
    best = min((D.proxy_time(poly8, a, "A", 160.0, "default", dc8) for a in cands), key=lambda p: p.T_s)
    assert abs(math.sin(best.alpha_rad)) < 1e-6      # 南北向（0 或 180°）


def _sloped_inputs(slope_deg: float, ring_enu):
    """mini 輸入 + 沿 +y 之等坡平面 DEM（z = 100 + tan(slope)·y）。"""
    inputs, cfg = build_mini_example()
    ctx0 = _ctx(inputs, cfg)
    frame = ctx0.frame
    dem = inputs.dem
    rows, cols = dem.z_msl.shape
    lat = dem.lat0_deg + dem.dlat_deg * np.arange(rows)
    lon = dem.lon0_deg + dem.dlon_deg * np.arange(cols)
    LAT, LON = np.meshgrid(lat, lon, indexing="ij")
    enu = frame.to_enu(LAT.ravel(), LON.ravel(), 100.0)
    y = enu[:, 1].reshape(rows, cols)
    z = (100.0 + math.tan(math.radians(slope_deg)) * y).astype(np.float32)
    dem2 = dataclasses.replace(dem, z_msl=z)
    area = AreaSpec("Z1", enu_ring_to_wgs84(frame, tuple(ring_enu)), sensor_req="any", priority=1.0, payload_req_id="default")
    inputs = dataclasses.replace(inputs, dem=dem2, areas=(area,))
    ctx = _ctx(inputs, cfg)
    return inputs, cfg, ctx, ctx.decompose_ctx()


def test_slope_penalty_finite_and_delta_z_penalty_orbits():
    """5° 等坡（沿 y）、3 km × 0.6 km 胞：
    - 南北向（順坡）：沿線坡度 0.0875 > tanγ^g_c → pen_slope > 0、slope_exceeds、T 有限（D15）；
    - 東西向（沿等高線）：pen_slope = 0，橫向坡度 × k s = 44 m > L_turn tanγ κ ≈ 36 → 每個上坡轉場 1 圈 → pen_Δz > 0；
    - 候選含等高線方向（90°）；argmin 為東西向。"""
    ring = ((4000.0, -300.0), (7000.0, -300.0), (7000.0, 300.0), (4000.0, 300.0))
    inputs, cfg, ctx, dc = _sloped_inputs(5.0, ring)
    poly = ctx.areas_enu["Z1"]
    cands = D.direction_candidates(poly, dc, 300.0)
    assert any(abs((math.degrees(a) % 180) - 90) < 1.0 for a in cands)
    p_ns = D.proxy_time(poly, 0.0, "A", 160.0, "default", dc)
    p_ew = D.proxy_time(poly, math.pi / 2, "A", 160.0, "default", dc)
    assert math.isfinite(p_ns.T_s) and p_ns.pen_slope_s > 0 and p_ns.slope_exceeds
    assert p_ew.pen_slope_s == pytest.approx(0.0, abs=1e-6) and not p_ew.slope_exceeds
    assert p_ew.pen_dz_s > 0 and p_ew.n_strips == 4
    spec = dc.specs["A"]
    t_lap = 2 * math.pi * 200 / spec.cruise_airspeed_mps
    assert p_ew.pen_dz_s >= 0.9 * 2 * t_lap                   # 2 個上坡轉場各 ≥ 1 圈
    assert p_ns.pen_dz_s == pytest.approx(0.0)
    best = min((D.proxy_time(poly, a, "A", 160.0, "default", dc) for a in cands), key=lambda p: p.T_s)
    assert abs(math.cos(best.alpha_rad)) < 1e-6


def test_one_way_feasible_cell_proxy_finite():
    """D34：B 快門 0.5 Hz → V_cap = 14.6 m/s；W=5 順風 24 > cap（不可行）、逆風 14 ≤ cap → 單向可行，代理有限、one_way ≠ 0；
    decompose_area 亦得有限 T_single（同向 racetrack 轉場）。"""
    inputs, cfg = build_mini_example()
    reqs = dict(inputs.payload_reqs)
    reqs["default_rgb"] = dataclasses.replace(reqs["default_rgb"], shutter_rate_hz=0.5)
    inputs = dataclasses.replace(inputs, payload_reqs=reqs)
    ctx = _ctx(inputs, cfg)
    dc = ctx.decompose_ctx()
    poly = ctx.areas_enu["Z1"]
    p = D.proxy_time(poly, math.pi / 2, "B", 100.0, "default", dc)
    assert math.isfinite(p.T_s) and p.one_way != 0
    # 逆風向（西，θ=270° = LR 方向）可行 → alpha=270 之 one_way = −1；alpha=90 之 one_way = +1
    p2 = D.proxy_time(poly, 3 * math.pi / 2, "B", 100.0, "default", dc)
    assert {p.one_way, p2.one_way} == {-1, 1}
    # 南北向：V_g = 18.33 > 14.6 兩向皆不可行 → inf
    assert not math.isfinite(D.proxy_time(poly, 0.0, "B", 100.0, "default", dc).T_s)
    cells = D.decompose_area(inputs.areas[0], dc, {"B"}, polygon_enu=poly, rho_sym_m=ctx.rho_sym_m)
    assert len(cells) == 1 and cells[0].feasible["B"] and math.isfinite(cells[0].T_single_s["B"])
    strips = cells[0].strips["B"]
    assert len(strips) == 12
    # 作業只用可行向：exact_time ≥ Σ L/14 + 同向轉場
    assert cells[0].T_single_s["B"] > sum(s.length_m for s in strips) / 14.0


def test_proxy_infeasible_turn_band_in_nofly():
    """D38/D41：禁區緊貼胞端線（距離 < R_eff + e_ext）→ 該方向不可行（inf）；垂直方向仍有限。"""
    ring = ((4000.0, -600.0), (7000.0, -600.0), (7000.0, 600.0), (4000.0, 600.0))
    nf = ((7100.0, -700.0), (7300.0, -700.0), (7300.0, 700.0), (7100.0, 700.0))       # ⊕ b_trk 50 → x ≥ 7050 > 7000
    inputs, cfg, ctx, dc = _mini_with_area(ring, nofly=(nf,))
    poly = ctx.areas_enu["Z1"]
    p_ew = D.proxy_time(poly, math.pi / 2, "A", 160.0, "default", dc)
    p_ns = D.proxy_time(poly, 0.0, "A", 160.0, "default", dc)
    assert not math.isfinite(p_ew.T_s) and p_ew.pen_keepout_s == math.inf
    assert math.isfinite(p_ns.T_s)


def test_proxy_nofly_island_does_not_veto_every_direction():
    """R-01（blocker）：胞內／胞旁的小塊禁區不得讓**所有**方向都變成 inf。

    decompose_area 以 F = P_area 減去 (NoFly⊕b_trk) 造胞，緊鄰禁區之胞與禁區緩衝區距離恆為 0；舊碼的硬性否決條件是
    `turn_band(cell, α, R+e, 0) ∩ NoFly > 1e-6 m²`，而 turn_band 是把整個胞沿 ±t̂ 平移的 Minkowski 和，
    任何 α 都會讓貼齊的邊界掃進緩衝區 → 每個方向都 inf → 整個區域被丟掉、覆蓋率 0，
    但 feasibility.status 仍停在 GO_WITH_WARNINGS。修正後只有「端線外連 U 型都放不下」才 inf。"""
    ring = ((4000.0, -600.0), (7000.0, -600.0), (7000.0, 600.0), (4000.0, 600.0))
    island = ((5450.0, -50.0), (5550.0, -50.0), (5550.0, 50.0), (5450.0, 50.0))       # 胞中央的小島
    inputs, cfg, ctx, dc = _mini_with_area(ring, nofly=(island,))
    poly = ctx.areas_enu["Z1"]
    ok = [a for a in np.arange(0.0, math.pi, math.radians(5.0))
          if math.isfinite(D.proxy_time(poly, float(a), "A", 160.0, "default", dc).T_s)]
    assert len(ok) >= 30, len(ok)                       # 36 個候選方向幾乎全部應該可行
    # 貼齊禁區之胞（F 之一塊）同樣不可被全數否決
    cells = D.decompose_area(inputs.areas[0], dc, {"A"})
    assert cells and any(c.feasible.get("A") for c in cells)
    assert all(c.infeasible_reason.get("A", "") != "no_feasible_direction"
               for c in cells if not c.feasible.get("A")), [(c.cell_id, dict(c.infeasible_reason)) for c in cells]


# ---------------------------------------------------------------------------
# decompose_area（規格 §7.1）與 §W.3 數值
# ---------------------------------------------------------------------------
def test_mini_decompose_w3_numbers(mini, mini_cells):
    """§W.3：1 個凸胞；A 7 條／B 12 條；θ* = 90°（東西向）；作業時間 A 東 118.3／西 190.1、B 130.3／223.3。"""
    inputs, cfg, ctx, dc = mini
    cells = mini_cells
    assert len(cells) == 1
    c = cells[0]
    assert c.area_id == "Z1" and c.sensor_req == "any"
    assert c.feasible == {"A": True, "B": True}
    assert c.n_strips == {"A": 7, "B": 12}
    assert len(c.strips["A"]) == 7 and len(c.strips["B"]) == 12
    for cls in ("A", "B"):
        assert abs(math.sin(c.alpha_rad[cls])) == pytest.approx(1.0, abs=1e-6)          # 東西向
        # ⚠⚠ 前兩條原本是 `alpha_rad == alt_dirs[0][0]` 與 `T_single == alt_dirs[0][1]`，
        #   那是**建構上必然為真**的（`decompose.py:1129-1130` 同一次排序寫出兩者）
        #   ⇒ 恆綠、鎖不到東西。改成鎖真的可能壞掉的三件事。
        alts = c.alt_dirs[cls]
        assert (c.alpha_rad[cls], c.T_single_s[cls]) == alts[0]                 # 首名即選中者
        assert [t for _a, t in alts] == sorted(t for _a, t in alts)             # 依 T 遞增
        assert len({round(a % math.pi, 9) for a, _t in alts}) == len(alts)      # 方向互異（mod π）
        assert all(s.feasible for s in c.strips[cls])
        assert all(s.cell_id == c.cell_id and s.cls == cls for s in c.strips[cls])
        assert 0.0 <= c.fit[cls] <= 1.0
        assert strips_union_footprint(c.strips[cls]).buffer(1e-3).contains(c.polygon)
        assert np.allclose(c.sweep_origin[cls], c.strips[cls][0].sweep_origin)
    assert c.h_op_m == {"A": 160.0, "B": 100.0}
    assert c.d_turn_m["A"] == pytest.approx(534.7, abs=0.3) and c.d_turn_m["B"] == pytest.approx(336.0, abs=0.3)
    # 條帶長 ≈ 3030 + 2·53（DP 角落略短）；y 中心（§W.3）：503.0, 334.9, …, −505.3
    sa = c.strips["A"]
    assert all(3120.0 <= s.length_m <= 3137.0 for s in sa)
    ys = [s.pL[1] for s in sa]
    want = [503.0, 334.9, 166.9, -1.2, -169.2, -337.3, -505.3]
    assert np.allclose(ys, want, atol=2.0)
    assert all(abs(s.h_min_m - 160.0) < 1e-6 and abs(s.width_m - 224.07) < 0.05 and abs(s.spacing_m - 168.05) < 0.05 for s in sa)
    sb = c.strips["B"]
    assert all(3110.0 <= s.length_m <= 3127.0 for s in sb)
    assert all(abs(s.width_m - 146.875) < 0.05 for s in sb)
    # 作業時間（V_a*：cap 不綁）
    wind_xy = np.array([5.0, 0.0])
    op_a = D._op_times(sa, dc.specs["A"], dc.sensors[("A", "default")], wind_xy, cfg)
    op_b = D._op_times(sb, dc.specs["B"], dc.sensors[("B", "default_rgb")], wind_xy, cfg)
    assert op_a[0, 0] == pytest.approx(sa[0].length_m / 26.5, rel=1e-9) and op_a[0, 1] == pytest.approx(sa[0].length_m / 16.5, rel=1e-9)
    assert op_b[0, 0] == pytest.approx(sb[0].length_m / 24.0, rel=1e-9) and op_b[0, 1] == pytest.approx(sb[0].length_m / 14.0, rel=1e-9)
    # T_single = 兩向平均 + skip-3 序列轉場（4 × U(3s)=35.5 + 2 × 其它）≈ 1079 + 240
    t_oper_a = float(op_a.mean(axis=1).sum())
    assert 1070.0 <= t_oper_a <= 1085.0
    assert 200.0 <= c.T_single_s["A"] - t_oper_a <= 300.0
    # union ⊇ 區域
    assert c.polygon.buffer(1e-3).contains(ctx.areas_enu["Z1"])
    assert all(abs(s.z_op_m - (160.0 - 2.0)) < 3.0 for s in sa)      # ENU z（曲率下垂 ≈ −1 … −4 m）


def test_exact_time_definition(mini, mini_cells):
    """契約明定：Σ(t_LR+t_RL)/2 + Σ skip-k 相鄰對反向轉場（左右端各算一次取平均，dubins.path_time）。"""
    inputs, cfg, ctx, dc = mini
    c = mini_cells[0]
    sa = c.strips["A"]
    spec = dc.specs["A"]
    wind_xy = np.array([5.0, 0.0])
    # 單條
    t1 = D.exact_time(sa[:1], "A", dc)
    assert t1 == pytest.approx(0.5 * (sa[0].length_m / 26.5 + sa[0].length_m / 16.5), rel=1e-9)
    # 兩條（k=3 → 序列 0,1）：右端 end_state(−1)→start_state(+1)、左端 end_state(+1)→start_state(−1)
    z = float(np.mean([s.z_op_m for s in sa[:2]]))
    w = dc.wind.wind_vec(z)[0]
    r = spec.r_eff(5.0)
    tr = 0.0
    for d_end, d_start in ((-1, 1), (1, -1)):
        p = dubins_path(sa[0].end_state(d_end), sa[1].start_state(d_start), r)
        tr += 0.5 * path_time(p, spec.cruise_airspeed_mps, w, z, cfg.dtheta_int_rad)
    t2 = D.exact_time(sa[:2], "A", dc)
    op = 0.5 * sum(s.length_m / 26.5 + s.length_m / 16.5 for s in sa[:2])
    assert t2 == pytest.approx(op + tr, rel=1e-6)
    # §W.5：skip-3 轉場 1→4（U 型 732.5 m）右端 35.5 s；Ω 1→2 1253 m
    p14 = dubins_path(sa[0].end_state(-1), sa[3].start_state(1), r)
    assert p14.length_m == pytest.approx(732.5, abs=1.0)
    assert path_time(p14, 21.5, w, z, cfg.dtheta_int_rad) == pytest.approx(35.5, abs=0.5)
    assert dubins_path(sa[0].end_state(-1), sa[1].start_state(1), r).length_m == pytest.approx(1253.3, abs=1.5)
    assert D.exact_time([], "A", dc) == 0.0


def test_L_shaped_area_multiple_cells():
    """L 形區域 → ≥ 2 個胞、union ⊇ 區域、各胞 A 可行且條帶覆蓋各胞。"""
    L = ((4000.0, -800.0), (7000.0, -800.0), (7000.0, 0.0), (5200.0, 0.0), (5200.0, 1200.0), (4000.0, 1200.0))
    inputs, cfg, ctx, dc = _mini_with_area(L)
    poly = ctx.areas_enu["Z1"]
    cells = D.decompose_area(inputs.areas[0], dc, {"A", "B"}, polygon_enu=poly, rho_sym_m=ctx.rho_sym_m)
    assert len(cells) >= 2
    u = shapely.union_all([c.polygon for c in cells])
    assert u.buffer(1e-3).contains(poly)
    ids = [c.cell_id for c in cells]
    assert len(set(ids)) == len(ids)
    for c in cells:
        assert c.parent_cell_id is None and c.k_A is None
        for cls in ("A", "B"):
            assert c.feasible[cls], (c.cell_id, cls, c.infeasible_reason)
            assert c.strips[cls] and c.n_strips[cls] == len(c.strips[cls])
            assert c.alpha_rad[cls] == c.alt_dirs[cls][0][0]
            assert strips_union_footprint(c.strips[cls]).buffer(1e-3).contains(c.polygon)
            assert math.isfinite(c.T_single_s[cls]) and 0.0 <= c.fit[cls] <= 1.0
    # 胞只在區域內部邊界互相重疊（外擴 δ_edge）；無 NoFly → union ≈ P_area
    assert u.area <= poly.buffer(cfg.delta_edge_resolved() * 1.01 + 15.0).area


def test_nofly_removed_and_strips_clear():
    """D41：F = P \\ (NoFly ⊕ b_trk)；union ⊇ Z \\ NoFly⊕b_trk；胞不含禁區內部；條帶 ⊕ b_trk 不與禁區相交。"""
    ring = ((4000.0, -600.0), (7000.0, -600.0), (7000.0, 600.0), (4000.0, 600.0))
    nf = ((5300.0, -900.0), (5700.0, -900.0), (5700.0, 900.0), (5300.0, 900.0))      # 南北向切斷區域
    inputs, cfg, ctx, dc = _mini_with_area(ring, nofly=(nf,))
    poly = ctx.areas_enu["Z1"]
    assert dc.nofly_buffered
    nfb = shapely.union_all(dc.nofly_buffered)
    cells = D.decompose_area(inputs.areas[0], dc, {"A", "B"}, polygon_enu=poly, rho_sym_m=ctx.rho_sym_m)
    assert len(cells) >= 2
    u = shapely.union_all([c.polygon for c in cells])
    assert u.buffer(1e-3).contains(poly.difference(nfb))
    assert u.intersection(nfb).area < 1.0
    for c in cells:
        for cls in ("A", "B"):
            assert c.feasible[cls], (c.cell_id, cls, c.infeasible_reason)
            for s in c.strips[cls]:
                # path ⊕ b_trk ∩ NoFly = ∅ ⟺ path ∩ (NoFly ⊕ b_trk) = ∅
                assert not shapely.LineString([s.pL, s.pR]).intersects(nfb)


def test_hills_all_areas(hills):
    """山丘情境：Z1（lidar）A 可行、B 不相容；Z2（both，含洞 C 形）多胞、A 與 B 各自條帶；Z3（rgb）B 可行；
    union ⊇ 區域 \\ NoFly⊕b_trk；山脊 → Z1 slope_exceeds 旗標。"""
    inputs, cfg, ctx, dc = hills
    nfb = shapely.union_all(dc.nofly_buffered) if dc.nofly_buffered else None
    t0 = time.time()
    all_cells = {a.area_id: D.decompose_area(a, dc, {"A", "B"}, polygon_enu=ctx.areas_enu[a.area_id], rho_sym_m=ctx.rho_sym_m)
                 for a in inputs.areas}
    assert time.time() - t0 < 6.0
    for aid, cells in all_cells.items():
        poly = ctx.areas_enu[aid]
        target = poly.difference(nfb) if nfb is not None else poly
        assert shapely.union_all([c.polygon for c in cells]).buffer(1e-3).contains(target)
    z1 = all_cells["Z1"]
    assert len(z1) == 1 and z1[0].feasible["A"] and not z1[0].feasible["B"]
    assert z1[0].infeasible_reason["B"] == "sensor_incompatible" and z1[0].fit["B"] == 0.0
    assert z1[0].slope_exceeds["A"] and z1[0].n_strips["A"] >= 15
    z2 = all_cells["Z2"]
    assert len(z2) >= 2
    assert any(c.feasible["A"] for c in z2) and any(c.feasible["B"] for c in z2)
    for c in z2:
        for cls in ("A", "B"):
            if c.feasible[cls]:
                assert c.fit[cls] > 0.0 and c.strips[cls]
    z3 = all_cells["Z3"]
    assert len(z3) == 1 and z3[0].feasible["B"] and not z3[0].feasible["A"]


# ---------------------------------------------------------------------------
# 殘帶（規格 §7.5、§9.5、D17）
# ---------------------------------------------------------------------------
def test_residual_candidate_pairs(mini, mini_cells):
    """§W.4：k ∈ {0, ⌈7/2⌉=4, 7} ∪ 前 3 名 {5, 6, 4}；k=5 → B 殘帶 3 條（y ≤ −266.3、中心 −337.8/−440.6/−543.4）；k=6 → 2 條。"""
    inputs, cfg, ctx, dc = mini
    c = mini_cells[0]
    res = D.residual_candidates(c, dc, rho_sym_m=ctx.rho_sym_m)
    ks = [k for k, _a, _b in res]
    assert ks == sorted(ks, reverse=True)
    assert set(ks) >= {0, 4, 7}
    assert {5, 6} <= set(ks)
    by_k = {k: (a, b) for k, a, b in res}
    assert by_k[7][0] is c and by_k[7][1] is None
    assert by_k[0][1].n_strips["B"] == 12
    n_res = {5: 3, 6: 2, 4: 5}
    for k in (4, 5, 6):
        ca, cb = by_k[k]
        assert ca.k_A == k and cb.k_A == k and ca.parent_cell_id == c.cell_id and cb.residual_of == c.cell_id
        assert ca.n_strips["A"] == k and len(ca.strips["A"]) == k
        assert [s.strip_id for s in ca.strips["A"]] == [s.strip_id for s in c.strips["A"][:k]]
        assert ca.feasible["A"] and not ca.feasible["B"] and math.isfinite(ca.T_single_s["A"])
        assert cb.feasible["B"] and not cb.feasible["A"]
        assert cb.n_strips["B"] == n_res[k]
        assert strips_union_footprint(cb.strips["B"]).buffer(1e-3).contains(cb.polygon)
        assert strips_union_footprint(ca.strips["A"]).buffer(1e-3).contains(ca.polygon)
        # A 子胞 ∪ B 殘帶 ⊇ 原胞
        assert shapely.union_all([ca.polygon, cb.polygon]).buffer(1e-3).contains(c.polygon)
        assert 0.0 <= ca.fit["A"] <= 1.0 and 0.0 <= cb.fit["B"] <= 1.0
    ca5, cb5 = by_k[5]
    assert cb5.polygon.bounds[3] == pytest.approx(-266.3, abs=2.0)
    ys = sorted((s.pL[1] for s in cb5.strips["B"]), reverse=True)
    assert np.allclose(ys, [-337.8, -440.6, -543.4], atol=2.0)
    assert ca5.polygon.bounds[1] == pytest.approx(-281.3, abs=2.0)
    # 指定 k_list
    res2 = D.residual_candidates(c, dc, k_list=[3, 7], rho_sym_m=ctx.rho_sym_m)
    assert [k for k, _a, _b in res2] == [7, 3] and res2[1][2].n_strips["B"] == 7
    # 非 any 胞 → []
    c2 = dataclasses.replace(c, sensor_req="lidar")
    assert D.residual_candidates(c2, dc) == []


# ---------------------------------------------------------------------------
# suitability（規格 §7.6）與 sensor_compatible
# ---------------------------------------------------------------------------
def test_sensor_compatible_truth_table():
    L, R, LR, N = frozenset({"lidar"}), frozenset({"rgb"}), frozenset({"lidar", "rgb"}), frozenset()
    assert D.sensor_compatible("any", N) and D.sensor_compatible("any", L)
    assert D.sensor_compatible("lidar", L) and D.sensor_compatible("lidar", LR) and not D.sensor_compatible("lidar", R)
    assert D.sensor_compatible("rgb", R) and D.sensor_compatible("rgb", LR) and not D.sensor_compatible("rgb", L)
    assert D.sensor_compatible("both", LR) and not D.sensor_compatible("both", L) and not D.sensor_compatible("both", R)
    assert not D.sensor_compatible("both", N)
    # both：A 出 lidar、B 出 rgb（D17）
    assert D.class_can_serve("both", "A", L) and not D.class_can_serve("both", "A", R)
    assert D.class_can_serve("both", "B", R) and not D.class_can_serve("both", "B", L)
    with pytest.raises(ValueError):
        D.sensor_compatible("thermal", L)


def test_suitability_properties(mini, mini_cells):
    inputs, cfg, ctx, dc = mini
    c = mini_cells[0]
    T_use = {cls: D.t_use_s_for(dc, cls) for cls in ("A", "B")}
    assert T_use["A"] > 3600 and T_use["B"] > 1800
    fa = D.suitability(c, "A", dc, T_use["A"], ctx.rho_sym_m)
    fb = D.suitability(c, "B", dc, T_use["B"], ctx.rho_sym_m)
    assert 0.0 < fa <= 1.0 and 0.0 < fb <= 1.0
    assert fa == pytest.approx(c.fit["A"]) and fb == pytest.approx(c.fit["B"])
    # 感測不相容 → 0
    c_lidar = dataclasses.replace(c, sensor_req="lidar")
    assert D.suitability(c_lidar, "B", dc, T_use["B"], ctx.rho_sym_m) == 0.0
    assert D.suitability(c_lidar, "A", dc, T_use["A"], ctx.rho_sym_m) > 0.0
    # H2：ρ^sym 小於胞距離 → 0
    assert D.suitability(c, "A", dc, T_use["A"], {"A": 1000.0, "B": 39000.0}) == 0.0
    # 高度帶不可行 → 0
    c_bad = dataclasses.replace(c, feasible={"A": False, "B": True})
    assert D.suitability(c_bad, "A", dc, T_use["A"], ctx.rho_sym_m) == 0.0
    # s_endur：T_use 越小分數越低
    assert D.suitability(c, "A", dc, c.T_single_s["A"] * 1.05, ctx.rho_sym_m) < fa
    # H4：λ=1 且能量緊 → 0（以極小 T_use 不影響 H4；改用 n_uav=1 與極大 T_single）
    c_long = dataclasses.replace(c, T_single_s={"A": 1e6, "B": 1e6})
    assert D.suitability(c_long, "A", dc, T_use["A"], ctx.rho_sym_m, n_uav=1) == 0.0


def test_decompose_skips_classes_not_in_fleet(mini):
    inputs, cfg, ctx, dc = mini
    cells = D.decompose_area(inputs.areas[0], dc, {"A"}, polygon_enu=ctx.areas_enu["Z1"])
    assert len(cells) == 1 and set(cells[0].feasible) == {"A"} and cells[0].n_strips == {"A": 7}
    cells2 = D.decompose_area(inputs.areas[0], dc, {"A", "B", "C"}, polygon_enu=ctx.areas_enu["Z1"])
    assert set(cells2[0].feasible) == {"A", "B"}


def test_build_strips_marks_nofly(mini_cells, mini):
    """D41：條帶 ⊕ b_trk 與禁區相交 → Strip.feasible False（nofly_intersect）。"""
    inputs, cfg, ctx, dc = mini
    c = mini_cells[0]
    dc2 = dataclasses.replace(dc, nofly_buffered=[Polygon([(5400, 400), (5600, 400), (5600, 700), (5400, 700)])])
    strips = D.build_strips(c, "A", math.pi / 2, dc2)
    assert len(strips) == 7
    bad = [s for s in strips if not s.feasible]
    assert bad and all(s.infeasible_reason == "nofly_intersect" for s in bad)
    assert all(s.pL[1] > 300.0 for s in bad)


def test_performance_mini(mini):
    inputs, cfg, ctx, dc = mini
    t0 = time.time()
    cells = D.decompose_area(inputs.areas[0], dc, {"A", "B"}, polygon_enu=ctx.areas_enu["Z1"], rho_sym_m=ctx.rho_sym_m)
    D.residual_candidates(cells[0], dc, rho_sym_m=ctx.rho_sym_m)
    assert time.time() - t0 < 3.0


def test_area_polygon_from_wgs84(mini):
    """不給 polygon_enu 時自 WGS84 轉 ENU，與 ctx.areas_enu 差 < 1 m。"""
    inputs, cfg, ctx, dc = mini
    p = D._area_polygon_enu(inputs.areas[0], dc)
    assert p.symmetric_difference(ctx.areas_enu["Z1"]).area < 1200.0 * 4 * 1.0
    assert abs(p.area - ctx.areas_enu["Z1"].area) / p.area < 1e-4
