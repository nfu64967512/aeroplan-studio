"""tests/test_geometry.py — 模組契約 §6 不變量與規格 §17.2 之 geometry 測試。"""
from __future__ import annotations

import math

import numpy as np
import pytest
import shapely
from shapely.geometry import LineString, MultiPolygon, Polygon

from hv_dccpp import geometry as G

# ---------------------------------------------------------------------------
# 測試用感測器（規格 §4.1 示例 LiDAR 70°、o_side 0.25；§4.2 示例相機 16 mm / 23.5 mm、o_side 0.30）
# ---------------------------------------------------------------------------
class FakeLidar:
    def __init__(self, fov_deg: float = 70.0, o_side: float = 0.25) -> None:
        self.k = 2.0 * math.tan(math.radians(fov_deg / 2.0))
        self.o_side = o_side

    def swath_m(self, h_agl_m: float) -> float:
        return self.k * h_agl_m

    def strip_spacing_m(self, h_agl_m: float) -> float:
        return self.swath_m(h_agl_m) * (1.0 - self.o_side)


class FakeCamera:
    def swath_m(self, h_agl_m: float) -> float:
        return h_agl_m * 23.5 / 16.0

    def strip_spacing_m(self, h_agl_m: float) -> float:
        return self.swath_m(h_agl_m) * 0.7


def flat_env(h: float):
    """平地局部包絡：h_min = agl_max = h_op、可行。"""
    return lambda xy, th, lo, hi: (h, h, True)


RECT_1270 = Polygon([(0, 0), (3000, 0), (3000, 1270), (0, 1270)])
CELL_W = Polygon([(3985, -615), (7015, -615), (7015, 615), (3985, 615)])   # 契約 §W：3 km × 1.2 km ⊕ 15 m
L_SHAPE = Polygon([(0, 0), (500, 0), (500, 250), (250, 250), (250, 500), (0, 500)])
NOTCH = Polygon([(20, 0), (500, 0), (500, 500), (0, 500), (0, 20), (20, 20)])  # 500 m 方形、角落 20 m 凹口
EAST = math.pi / 2


# ---------------------------------------------------------------------------
# Strip / Pose 約定（D4、§8.2）
# ---------------------------------------------------------------------------
def _strip(theta: float = EAST) -> G.Strip:
    return G.Strip(strip_id="s", cell_id="c", index=1, cls="A", pL=np.array([0.0, 0.0]), pR=np.array([100.0, 0.0]),
                   theta_LR_rad=theta, sweep_origin=np.zeros(2), width_m=210.0, spacing_m=157.5, h_min_m=150.0,
                   h_op_m=150.0, agl_max_m=150.0, v_lo_m=0.0, v_hi_m=210.0, length_m=100.0, z_op_m=250.0)


def test_strip_end_start_state_conventions():
    s = _strip()
    th_rl = (EAST + math.pi) % (2 * math.pi)
    e = s.end_state(-1)      # LR 掃完 → (pR, θ_LR)
    assert (e.x_m, e.y_m) == (100.0, 0.0) and e.theta_rad == pytest.approx(EAST)
    e = s.end_state(+1)      # RL 掃完 → (pL, θ_RL)
    assert (e.x_m, e.y_m) == (0.0, 0.0) and e.theta_rad == pytest.approx(th_rl)
    st = s.start_state(-1)   # LR 開掃 → (pL, θ_LR)
    assert (st.x_m, st.y_m) == (0.0, 0.0) and st.theta_rad == pytest.approx(EAST)
    st = s.start_state(+1)   # RL 開掃 → (pR, θ_RL)
    assert (st.x_m, st.y_m) == (100.0, 0.0) and st.theta_rad == pytest.approx(th_rl)
    assert s.pose("L") == s.start_state(-1) and s.pose("LR") == s.end_state(-1)
    assert s.pose("R") == s.start_state(+1) and s.pose("RL") == s.end_state(+1)
    assert np.array_equal(s.endpoint(-1), s.pL) and np.array_equal(s.endpoint(+1), s.pR)
    with pytest.raises(ValueError):
        s.end_state(0)
    assert s.pose("L").as_array().shape == (3,)


# ---------------------------------------------------------------------------
# 掃描座標（契約 §0）
# ---------------------------------------------------------------------------
def test_sweep_frame_right_normal():
    t, n = G.sweep_frame(EAST)
    assert np.allclose(t, [1, 0]) and np.allclose(n, [0, -1])     # 向東時右法向指南
    t, n = G.sweep_frame(0.0)
    assert np.allclose(t, [0, 1]) and np.allclose(n, [1, 0])


def test_sweep_origin_consistent():
    o = G.sweep_origin(CELL_W, EAST)
    assert np.allclose(o, [3985.0, 615.0])                             # 契約 §W：胞西北角
    uv = G.to_sweep_coords(shapely.get_coordinates(CELL_W), o, EAST)
    assert uv[:, 0].min() == pytest.approx(0.0) and uv[:, 1].min() == pytest.approx(0.0)
    assert uv[:, 1].max() == pytest.approx(1230.0) and uv[:, 0].max() == pytest.approx(3030.0)
    # v 增加 = 向南：y = 615 − v
    xy = G.from_sweep_coords(np.array([[0.0, 100.0]]), o, EAST)
    assert np.allclose(xy, [[3985.0, 515.0]])
    for alpha in (0.3, 1.0, 2.5, 4.0):
        o2 = G.sweep_origin(L_SHAPE, alpha)
        uv2 = G.to_sweep_coords(shapely.get_coordinates(L_SHAPE), o2, alpha)
        assert uv2.min(axis=0) == pytest.approx([0.0, 0.0], abs=1e-9)


def test_sweep_coords_roundtrip():
    rng = np.random.default_rng(1)
    uv = rng.uniform(-5000, 5000, size=(200, 2))
    origin = np.array([1234.5, -678.9])
    for alpha in (0.0, 0.7, EAST, 3.9):
        xy = G.from_sweep_coords(uv, origin, alpha)
        assert np.allclose(G.to_sweep_coords(xy, origin, alpha), uv, atol=1e-9)
    single = G.to_sweep_coords(np.array([10.0, 20.0]), origin, 0.7)
    assert single.shape == (2,)


def test_projection_width_and_min_width_direction():
    assert G.projection_width(RECT_1270, EAST) == pytest.approx(1270.0)
    assert G.projection_width(RECT_1270, 0.0) == pytest.approx(3000.0)
    alpha, w = G.min_width_direction(RECT_1270)
    assert w == pytest.approx(1270.0) and alpha == pytest.approx(EAST)
    assert G.projection_width(RECT_1270, alpha) == pytest.approx(w)
    # 旋轉後仍找到最小寬度方向
    rot = shapely.affinity.rotate(RECT_1270, 30, origin=(0, 0))
    alpha_r, w_r = G.min_width_direction(rot)
    assert w_r == pytest.approx(1270.0, abs=1e-6)
    assert G.projection_width(rot, alpha_r) == pytest.approx(1270.0, abs=1e-6)
    # 非凸：以凸包計（L 形最小寬 500）
    _, w_l = G.min_width_direction(L_SHAPE)
    assert w_l == pytest.approx(500.0)
    assert G.min_width_direction(Polygon()) == (0.0, 0.0)


# ---------------------------------------------------------------------------
# 簡化、凹度、凸分割（§7.2、D31）
# ---------------------------------------------------------------------------
def _jagged(seed: int = 0, n: int = 200, r: float = 1000.0, noise: float = 3.0) -> Polygon:
    rng = np.random.default_rng(seed)
    ang = np.linspace(0, 2 * math.pi, n, endpoint=False)
    rr = r + rng.normal(0, noise, n)
    return Polygon(np.stack([rr * np.cos(ang), rr * np.sin(ang)], axis=1))


def test_simplify_outward_contains_original():
    jag = _jagged()
    out = G.simplify_outward(jag, 15.0, 10.0)
    assert out.buffer(1e-6).contains(jag)
    assert len(out.exterior.coords) < len(jag.exterior.coords) / 3
    # ε ≥ δ 仍保證外擴（simplified ∪ original）
    out2 = G.simplify_outward(jag, 5.0, 40.0)
    assert out2.buffer(1e-6).contains(jag)
    # 含洞：洞內縮並簡化，原多邊形仍被包含
    holed = Polygon(jag.exterior.coords, holes=[_jagged(3, 60, 200, 2.0).exterior.coords])
    out3 = G.simplify_outward(holed, 15.0, 10.0)
    assert out3.buffer(1e-6).contains(holed)
    assert out3.area <= holed.buffer(15.0).area + 1e-6


def test_concavity_values():
    assert G.concavity(RECT_1270) == pytest.approx(0.0, abs=1e-9)
    assert G.concavity(L_SHAPE) == pytest.approx(250 / math.sqrt(2), rel=1e-3)
    c = G.concavity(NOTCH)
    assert 20 / math.sqrt(2) - 0.5 <= c <= 20.0 + 0.5      # 角落 20 m 凹口：Hausdorff 深度 20/√2（凸包橋為對角線）
    tri = Polygon([(0, 0), (100, 0), (50, 80)])
    assert G.concavity(tri) == pytest.approx(0.0, abs=1e-9)


def test_is_convex_tolerance():
    assert G.is_convex(RECT_1270)
    assert not G.is_convex(L_SHAPE)
    near = Polygon([(0, 0), (100, 0), (200, -1.0), (300, 0), (300, 100), (0, 100)])   # 反射 ~1.1° < 2°
    assert G.is_convex(near, tol_deg=2.0) and not G.is_convex(near, tol_deg=0.5)


def test_small_notch_accepted_near_convex():
    parts = G.convex_partition(NOTCH, s_ref_m=102.8, tau_merge_m=73.0)
    assert parts == [NOTCH]


def test_concave_L_two_convex():
    parts = G.convex_partition(L_SHAPE, s_ref_m=102.8, tau_merge_m=73.0)
    assert len(parts) == 2
    assert all(G.is_convex(p) for p in parts)
    assert sum(p.area for p in parts) == pytest.approx(L_SHAPE.area)
    assert shapely.union_all(parts).buffer(1e-6).contains(L_SHAPE)
    areas = sorted(p.area for p in parts)
    assert areas == pytest.approx([62500.0, 125000.0])


def test_polygon_with_hole():
    holed = Polygon([(0, 0), (600, 0), (600, 600), (0, 600)], holes=[[(250, 250), (250, 350), (350, 350), (350, 250)]])
    parts = G.convex_partition(holed, s_ref_m=102.8, tau_merge_m=73.0)
    assert len(parts) >= 2
    assert all(len(p.interiors) == 0 for p in parts)
    assert sum(p.area for p in parts) == pytest.approx(holed.area, rel=1e-9)
    assert shapely.union_all(parts).symmetric_difference(holed).area < 1e-6
    for p in parts:
        assert G.is_convex(p) or G.concavity(p) <= 73.0 + 1e-6


def test_merge_shallow_notch():
    n_fn = lambda p: max(1, math.ceil(G.min_width_direction(p)[1] / 102.8 - 1e-9))
    a = Polygon([(0, 0), (300, 0), (300, 500), (0, 500)])
    b = Polygon([(300, 0), (500, 0), (500, 500), (300, 500)])
    merged = G.merge_shallow([a, b], 73.0, n_fn)
    assert len(merged) == 1 and merged[0].area == pytest.approx(250000.0)
    assert G.is_convex(merged[0])
    # 合併後凹度 > τ → 不合併
    top = Polygon([(0, 250), (250, 250), (250, 500), (0, 500)])
    bottom = Polygon([(0, 0), (500, 0), (500, 250), (0, 250)])
    assert len(G.merge_shallow([top, bottom], 73.0, n_fn)) == 2
    # 淺凹口（≤ τ）且不增加條帶數 → 合併
    sliver = Polygon([(0, 500), (480, 500), (480, 520), (0, 520)])          # 深 20 m 之淺凹口
    big = Polygon([(0, 0), (500, 0), (500, 500), (0, 500)])
    m2 = G.merge_shallow([big, sliver], 73.0, n_fn)
    assert len(m2) == 1 and G.concavity(m2[0]) <= 73.0


def test_partition_always_returns():
    ang = np.linspace(0, 2 * math.pi, 17)[:-1]
    star = Polygon([(math.cos(a) * (300 if i % 2 == 0 else 80), math.sin(a) * (300 if i % 2 == 0 else 80)) for i, a in enumerate(ang)])
    parts = G.convex_partition(star, s_ref_m=1000.0, tau_merge_m=5.0)      # A_min = 1e6 > 面積 → A_min=0 重試
    assert len(parts) >= 1
    assert sum(p.area for p in parts) == pytest.approx(star.area, rel=1e-9)
    assert all(G.is_convex(p, 2.0) or G.concavity(p) <= 5.0 for p in parts)
    assert G.convex_partition(Polygon(), 100.0, 50.0) == []
    # 極小反射頂點面積片段：仍回傳
    thin = Polygon([(0, 0), (1000, 0), (1000, 10), (500, 9), (0, 10)])
    assert len(G.convex_partition(thin, 500.0, 0.1)) >= 1


def test_multipolygon_after_nofly():
    """nofly_buffer_btrk：P \\ (NoFly ⊕ b_trk) 為 MultiPolygon → 逐一分割，union ⊇ 差集。"""
    area = Polygon([(0, 0), (3000, 0), (3000, 1200), (0, 1200)])
    nofly_line = LineString([(1500, -100), (1500, 1300)])
    F = area.difference(nofly_line.buffer(50.0))
    assert isinstance(F, MultiPolygon) and len(F.geoms) == 2
    parts = G.convex_partition(F, s_ref_m=157.5, tau_merge_m=105.0)
    assert len(parts) == 2
    assert shapely.union_all(parts).buffer(1e-6).contains(F)
    assert all(p.distance(nofly_line) >= 50.0 - 1e-6 for p in parts)


# ---------------------------------------------------------------------------
# 條帶（§7.4、D3）
# ---------------------------------------------------------------------------
def _check_relay(strips: list[G.Strip]) -> None:
    for a, b in zip(strips, strips[1:]):
        o_side = 1.0 - a.spacing_m / a.width_m
        assert b.v_lo_m == pytest.approx(a.v_hi_m - o_side * a.width_m, abs=1e-6)
        assert b.v_lo_m == pytest.approx(a.v_lo_m + a.spacing_m, abs=1e-6)


def test_strip_count_formula():
    assert G.strip_count(1230, 210.06, 157.55) == 8
    assert G.strip_count(1270, 210.06, 157.55) == 8
    assert G.strip_count(1230, 224.07, 168.05) == 7          # 契約 §W.3（A @160 m）
    assert G.strip_count(1230, 146.88, 102.81) == 12         # 契約 §W.3（B）
    assert G.strip_count(10, 210.06, 157.55) == 1
    with pytest.raises(ValueError):
        G.strip_count(100, 10, 0)


def test_lay_strips_matches_count():
    sen = FakeLidar()
    strips = G.lay_strips(RECT_1270, "c1", "A", EAST, sen, 150.0, 53.0, flat_env(150.0))
    assert len(strips) == 8 == G.strip_count(1270, sen.swath_m(150), sen.strip_spacing_m(150))
    w, s = sen.swath_m(150.0), sen.strip_spacing_m(150.0)
    for W in (1155.0, 1156.0, 1200.0, 1260.0, 1265.0, 1312.0, 1313.5, 1400.0, 2000.0):
        poly = Polygon([(0, 0), (3000, 0), (3000, W), (0, W)])
        assert len(G.lay_strips(poly, "c", "A", EAST, sen, 150.0, 53.0, flat_env(150.0))) == G.strip_count(W, w, s), W


def test_lay_strips_fields_and_mini_example():
    """契約 §W.3：A @160 m 7 條、u ∈ [3932, 7068]、L=3136；B @100 m 12 條、Δ=47.8 → 平移 23.9 m。"""
    strips = G.lay_strips(CELL_W, "Z1", "A", EAST, FakeLidar(), 160.0, 53.0, flat_env(160.0))
    assert len(strips) == 7
    w, s = 2 * 160 * math.tan(math.radians(35)), 2 * 160 * math.tan(math.radians(35)) * 0.75
    assert strips[0].width_m == pytest.approx(w) and strips[0].spacing_m == pytest.approx(s)
    delta = 6 * s + w - 1230.0
    assert delta == pytest.approx(2.37, abs=0.02)
    for l, st in enumerate(strips):
        assert st.index == l + 1 and st.cell_id == "Z1" and st.cls == "A"
        assert st.theta_LR_rad == pytest.approx(EAST)
        assert np.allclose(st.sweep_origin, [3985.0, 615.0])
        assert st.pL[0] == pytest.approx(3932.0) and st.pR[0] == pytest.approx(7068.0)
        assert st.length_m == pytest.approx(3136.0)
        v_c = l * s + w / 2 - delta / 2
        assert st.pL[1] == pytest.approx(615.0 - v_c) and st.pR[1] == pytest.approx(615.0 - v_c)   # y = 615 − v
        assert st.v_lo_m == pytest.approx(l * s - delta / 2) and st.v_hi_m == pytest.approx(l * s - delta / 2 + w)
        assert st.h_min_m == 160.0 and st.h_op_m == 160.0 and st.agl_max_m == 160.0 and st.feasible
        assert st.z_op_m == 160.0                                          # 閉包未提供 z_op → h_op（契約偏差紀錄）
    assert strips[0].v_lo_m == pytest.approx(-delta / 2) and strips[-1].v_hi_m == pytest.approx(1230.0 + delta / 2)
    _check_relay(strips)
    assert len({st.strip_id for st in strips}) == 7
    sb = G.lay_strips(CELL_W, "Z1", "B", EAST, FakeCamera(), 100.0, 48.0, flat_env(100.0))
    assert len(sb) == 12
    assert sb[-1].v_hi_m - 1230.0 == pytest.approx(47.8 / 2, abs=0.1)
    assert sb[0].v_lo_m == pytest.approx(-47.8 / 2, abs=0.1)
    assert sb[0].pL[0] == pytest.approx(3937.0) and sb[0].length_m == pytest.approx(3126.0)


def test_strips_union_covers_cell():
    sen = FakeLidar()
    hexa = Polygon([(0, 0), (1500, -200), (3200, 300), (2900, 1400), (1200, 1700), (-300, 900)])
    for alpha in (0.0, 0.4, EAST, 2.2, 3.5, 5.0):
        strips = G.lay_strips(hexa, "h", "A", alpha, sen, 150.0, 53.0, flat_env(150.0))
        fp = G.strips_union_footprint(strips)
        assert fp.buffer(1e-3).contains(hexa)
        _check_relay(strips)
        # 每條帶之 u 範圍 ⊇ 帶內子多邊形極值（論文 Eq.(8)(9)）
        o = G.sweep_origin(hexa, alpha)
        for st in strips:
            sub = G.clip_band(hexa, o, alpha, st.v_lo_m, st.v_hi_m)
            uv = G.to_sweep_coords(shapely.get_coordinates(sub), o, alpha)
            uL = G.to_sweep_coords(st.pL, o, alpha)[0]
            uR = G.to_sweep_coords(st.pR, o, alpha)[0]
            assert uL <= uv[:, 0].min() - 53.0 + 1e-6 and uR >= uv[:, 0].max() + 53.0 - 1e-6
    # 含洞胞亦覆蓋（帶飛越洞）
    holed = Polygon([(0, 0), (2000, 0), (2000, 1000), (0, 1000)], holes=[[(800, 300), (800, 700), (1200, 700), (1200, 300)]])
    strips = G.lay_strips(holed, "hh", "A", EAST, sen, 150.0, 53.0, flat_env(150.0))
    assert G.strips_union_footprint(strips).buffer(1e-3).contains(holed)


def test_superset_band_conservative():
    """h_min 收縮：寬度 = swath(h_min)，帶 ⊂ 超集帶，走廊皆以超集帶回報。"""
    sen = FakeLidar()
    calls: list[tuple[float, float]] = []

    def env(xy, th, lo, hi):
        calls.append((lo, hi))
        return (120.0, 120.0, True)

    strips = G.lay_strips(RECT_1270, "c", "A", EAST, sen, 150.0, 53.0, env)
    for st in strips:
        assert st.width_m == pytest.approx(sen.swath_m(min(st.h_min_m, 150.0)))
        assert st.width_m == pytest.approx(sen.swath_m(120.0))
        assert st.h_min_m == 120.0 and st.spacing_m == pytest.approx(sen.strip_spacing_m(120.0))
    # 每條帶：第一次呼叫的超集帶寬 = swath(150)，後續帶 [v_lo, v_lo + swath(120)] ⊂ 超集帶
    assert all(hi - lo == pytest.approx(sen.swath_m(150.0)) for lo, hi in calls)
    _check_relay(strips)
    assert G.strips_union_footprint(strips).buffer(1e-3).contains(RECT_1270)
    assert len(strips) == G.strip_count(1270, sen.swath_m(120), sen.strip_spacing_m(120))


def test_width_never_exceeds_superset():
    """h_min > h_B（包絡整段抬高）：寬度仍取超集帶 swath(h_B)；widen_on_lift=True 才以新帶確認後加寬。"""
    sen = FakeLidar()
    lifted = lambda xy, th, lo, hi: (190.0, 190.0, True)
    strips = G.lay_strips(RECT_1270, "c", "A", EAST, sen, 150.0, 53.0, lifted)
    for st in strips:
        assert st.h_min_m == 190.0
        assert st.width_m == pytest.approx(sen.swath_m(150.0))
        assert st.width_m == pytest.approx(sen.swath_m(min(st.h_min_m, 150.0)))
    wide = G.lay_strips(RECT_1270, "c", "A", EAST, sen, 150.0, 53.0, lifted, widen_on_lift=True)
    assert all(st.width_m == pytest.approx(sen.swath_m(190.0)) for st in wide)
    assert len(wide) < len(strips)
    assert G.strips_union_footprint(wide).buffer(1e-3).contains(RECT_1270)


def test_centered_shift_relay_no_gap():
    """置中平移後第 1 條走廊移入 v<0 區使 h_min 下降 → 自該條重鋪、不再平移、無縫且覆蓋。"""
    sen = FakeLidar()

    def env(xy, th, lo, hi):
        return (120.0, 120.0, True) if lo < -1.0 else (150.0, 150.0, True)

    strips = G.lay_strips(RECT_1270, "c", "A", EAST, sen, 150.0, 53.0, env)
    delta = 7 * sen.strip_spacing_m(150) + sen.swath_m(150) - 1270.0
    assert strips[0].v_lo_m == pytest.approx(-delta / 2)
    assert strips[0].width_m == pytest.approx(sen.swath_m(120.0)) and strips[0].h_min_m == 120.0
    assert all(st.width_m == pytest.approx(sen.swath_m(150.0)) for st in strips[1:])
    assert strips[-1].v_hi_m >= 1270.0 - 1e-9
    _check_relay(strips)
    assert G.strips_union_footprint(strips).buffer(1e-3).contains(RECT_1270)
    assert [st.index for st in strips] == list(range(1, len(strips) + 1))


def test_lay_strips_infeasible_flag_and_z_op():
    sen = FakeLidar()
    bad = lambda xy, th, lo, hi: (150.0, 150.0, False)
    strips = G.lay_strips(RECT_1270, "c", "A", EAST, sen, 150.0, 53.0, bad)
    assert strips and all(not st.feasible and st.infeasible_reason for st in strips)
    with pytest.raises(G.StripInfeasibleError):
        G.lay_strips(RECT_1270, "c", "A", EAST, sen, 150.0, 53.0, bad, raise_on_infeasible=True)
    nan_env = lambda xy, th, lo, hi: (float("nan"), float("nan"), False)
    strips = G.lay_strips(RECT_1270, "c", "A", EAST, sen, 150.0, 53.0, nan_env)
    assert all(math.isfinite(st.h_min_m) and st.width_m > 0 for st in strips)
    with_z = lambda xy, th, lo, hi: (150.0, 150.0, True, 350.0)
    strips = G.lay_strips(RECT_1270, "c", "A", EAST, sen, 150.0, 53.0, with_z)
    assert all(st.z_op_m == 350.0 for st in strips)
    # 閉包收到之中心線：沿 t̂、theta = alpha、v 於帶中心
    seen = {}

    def spy(xy, th, lo, hi):
        seen["xy"], seen["th"], seen["band"] = xy, th, (lo, hi)
        return (150.0, 150.0, True)

    G.lay_strips(RECT_1270, "c", "A", EAST, sen, 150.0, 53.0, spy)
    assert np.all(seen["th"] == EAST) and seen["xy"].shape[1] == 2 and seen["xy"].shape[0] >= 2
    assert np.allclose(np.diff(seen["xy"][:, 1]), 0.0)


def test_clip_band_and_residual_subcell():
    o = G.sweep_origin(CELL_W, EAST)
    sub = G.clip_band(CELL_W, o, EAST, 100.0, 300.0)
    assert isinstance(sub, Polygon) and sub.area == pytest.approx(3030.0 * 200.0)
    assert G.clip_band(CELL_W, o, EAST, 2000.0, 2100.0) is None
    assert G.clip_band(CELL_W, o, EAST, 300.0, 100.0) is None
    # U 形胞：帶穿過缺口 → MultiPolygon
    U = Polygon([(0, 0), (900, 0), (900, 600), (600, 600), (600, 200), (300, 200), (300, 600), (0, 600)])
    oU = G.sweep_origin(U, EAST)                                      # α=90°：v = 600 − y，帶橫切兩臂
    sub = G.clip_band(U, oU, EAST, 50.0, 150.0)
    assert isinstance(sub, MultiPolygon) and len(sub.geoms) == 2
    assert sub.area == pytest.approx(2 * 300.0 * 100.0)
    one = G.clip_band(U, G.sweep_origin(U, 0.0), 0.0, 250.0, 650.0)   # α=0：帶含底部橫桿 → 單一 Polygon
    assert isinstance(one, Polygon)
    # 殘帶（契約 §W.4：k_A=5、v_top=896.3、δ=15 → v ≥ 881.3 ↔ y ≤ −266.3、寬 348.7）
    res = G.residual_subcell(CELL_W, EAST, o, 896.3, 15.0)
    assert res is not None and res.bounds[3] == pytest.approx(-266.3) and res.bounds[1] == pytest.approx(-615.0)
    assert G.projection_width(res, EAST) == pytest.approx(348.7)
    assert res.bounds[0] == pytest.approx(3985.0) and res.bounds[2] == pytest.approx(7015.0)   # u 方向沿用胞邊界
    assert G.residual_subcell(CELL_W, EAST, o, 1230.0, 0.0) is None
    # B 以自身條帶重鋪殘帶：3 條、跨度 352.5、平移 1.9 m（契約 §W.4）
    sb = G.lay_strips(res, "Z1_B", "B", EAST, FakeCamera(), 100.0, 48.0, flat_env(100.0))
    assert len(sb) == 3
    assert sb[-1].v_hi_m - sb[0].v_lo_m == pytest.approx(352.5, abs=0.1)
    assert sb[0].v_lo_m == pytest.approx(-1.9, abs=0.1)


def test_turn_band_extents():
    tb = G.turn_band(CELL_W, EAST, 487.0, 121.0)
    assert tb.bounds == pytest.approx((3985 - 487, -615 - 121, 7015 + 487, 615 + 121))
    assert tb.area == pytest.approx((3030 + 2 * 487) * (1230 + 2 * 121))
    assert tb.contains(CELL_W)
    # 端線方向沿 t̂：α=0 時 d_turn 加在 y、lateral 加在 x
    tb0 = G.turn_band(CELL_W, 0.0, 487.0, 0.0)
    assert tb0.bounds == pytest.approx((3985, -615 - 487, 7015, 615 + 487))
    # 非凸：Minkowski 和精確（⊇ 各方向平移）
    t, n = G.sweep_frame(0.7)
    tbL = G.turn_band(L_SHAPE, 0.7, 100.0, 30.0)
    for su in (-1, 1):
        for sv in (-1, 1):
            shifted = shapely.affinity.translate(L_SHAPE, *(su * 100.0 * t + sv * 30.0 * n))
            assert tbL.buffer(1e-6).contains(shifted)
    assert G.turn_band(L_SHAPE, 0.7, 0.0, 0.0) is L_SHAPE


def test_lay_strips_multipolygon_gap_and_cell_dataclass():
    """MultiPolygon 胞（殘帶／禁區差集）：投影間隙處跳到下一個有頂點的 v，仍覆蓋且無限迴圈。"""
    sen = FakeLidar()
    mp = MultiPolygon([Polygon([(0, 0), (2000, 0), (2000, 400), (0, 400)]),
                       Polygon([(0, 1500), (2000, 1500), (2000, 1900), (0, 1900)])])
    strips = G.lay_strips(mp, "mp", "A", EAST, sen, 150.0, 53.0, flat_env(150.0))
    assert len(strips) == 6
    assert strips[2].v_hi_m < 1000.0 < strips[3].v_lo_m                 # 間隙未鋪帶
    assert G.strips_union_footprint(strips).buffer(1e-3).contains(mp)
    assert [st.index for st in strips] == list(range(1, 7))
    cell = G.Cell(cell_id="c", area_id="a", polygon=mp.geoms[0], sensor_req="any", payload_req_id="d", priority=1.0,
                  time_window_s=None, area_m2=mp.geoms[0].area, min_width_m=400.0, min_width_dir_rad=EAST, dist_home_m=0.0,
                  sweep_origin={"A": G.sweep_origin(mp.geoms[0], EAST)}, h_op_m={"A": 150.0}, feasible={"A": True},
                  infeasible_reason={}, alpha_rad={"A": EAST}, strips={"A": strips}, alt_dirs={}, fit={}, n_strips={"A": 6},
                  T_single_s={}, d_turn_m={})
    assert cell.slope_exceeds == {} and cell.k_A is None and cell.parent_cell_id is None and cell.residual_of is None


def test_strips_union_footprint_geometry():
    strips = G.lay_strips(RECT_1270, "c", "A", EAST, FakeLidar(), 150.0, 53.0, flat_env(150.0))
    fp = G.strips_union_footprint(strips)
    assert isinstance(fp, Polygon)
    assert fp.bounds == pytest.approx((-53.0, 1270.0 - strips[-1].v_hi_m, 3053.0, -strips[0].v_lo_m + 1270.0))
    assert G.strips_union_footprint([]).is_empty
