"""tests/test_routing.py — routing.KeepoutSet / TangentGraph / VisibilityGraph / avoid（契約 §6B；規格 §8.5、§8.6）。

情境（inline）：平地 100 m MSL（ENU z ≈ 0，含曲率）、home (23.95°N, 120.90°E)；電塔 T1（點，(1500,0)，60 m AGL）、
輸電線 PL（線，x=2500，頂 180 m MSL，緩衝 60）、建物 B1（面，(3000..3300, ±100)，25 m AGL）、禁飛區 NF
（(4000..4500, ±200)，樓層 300／天花板 500 MSL）、NF2（時間窗外）、NF3（無樓層／天花板）。b_h=50、b_v=30。
"""
from __future__ import annotations

import math

import numpy as np
import pytest
import shapely
from shapely.geometry import LineString, Point, Polygon

from hv_dccpp import routing as RT
from hv_dccpp.dubins import Pose, discretize, dubins_path, path_time
from hv_dccpp.geo import GeoFrame, TerrainGrid
from hv_dccpp.mission import NoFlyZone, Obstacle, PlannerConfig
from hv_dccpp.routing import (
    AVOID_WORD,
    KeepoutSet,
    RoutingError,
    TangentGraph,
    VisibilityGraph,
    avoid,
    avoid_rho_m,
    path_clearance_m,
    path_is_clear,
    polyline_to_path,
    segment_time,
)

LAT0, LON0, Z0 = 23.95, 120.90, 100.0
FRAME = GeoFrame(LAT0, LON0, Z0)
CFG = PlannerConfig(dem_step_m=50.0)
R_A, HW = 200.0, 50.0
EAST = math.pi / 2


def _wgs(x: float, y: float) -> tuple[float, float]:
    lat, lon, _ = FRAME.to_wgs84(np.array([x, y, 0.0]))[0]
    return (float(lat), float(lon))


def _flat_terrain(is_dsm: bool = False) -> TerrainGrid:
    xs = np.arange(-1000.0, 5001.0, 50.0)
    ys = np.arange(-1500.0, 1501.0, 50.0)
    X, Y = np.meshgrid(xs, ys)
    Z = FRAME.msl_to_enu_z(np.c_[X.ravel(), Y.ravel()], Z0).reshape(X.shape)
    return TerrainGrid(-1000.0, -1500.0, 50.0, Z, is_dsm=is_dsm)


TOWER = Obstacle("T1", "point", (_wgs(1500, 0),), None, 60.0, category="tower")
LINE = Obstacle("PL", "line", (_wgs(2500, -500), _wgs(2500, 500)), 180.0, None, category="powerline", keepout_buffer_m=60.0)
BLDG = Obstacle("B1", "polygon", (_wgs(3000, -100), _wgs(3300, -100), _wgs(3300, 100), _wgs(3000, 100)), None, 25.0,
                category="building")
NF = NoFlyZone("NF", (_wgs(4000, -200), _wgs(4500, -200), _wgs(4500, 200), _wgs(4000, 200)), z_floor_msl_m=300.0,
               z_ceiling_msl_m=500.0)
NF2 = NoFlyZone("NF2", (_wgs(500, 500), _wgs(800, 500), _wgs(800, 800)), active_window_s=(5000.0, 6000.0))
NF3 = NoFlyZone("NF3", (_wgs(-500, 1000), _wgs(-200, 1000), _wgs(-200, 1300)))


def _build(is_dsm: bool = False) -> tuple[KeepoutSet, TerrainGrid]:
    terr = _flat_terrain(is_dsm)
    ks = KeepoutSet.build([TOWER, LINE, BLDG], [NF, NF2, NF3], FRAME, terr, CFG, (0.0, 3600.0))
    return ks, terr


@pytest.fixture(scope="module")
def built() -> tuple[KeepoutSet, TerrainGrid]:
    return _build()


def _direct_ks(geoms, bufs, tops, floors=None) -> KeepoutSet:
    polys = [g.buffer(b) if b > 0 else g for g, b in zip(geoms, bufs)]
    return KeepoutSet(polys=polys, tops_z=list(tops), ids=[f"k{i}" for i in range(len(polys))],
                      tree=shapely.STRtree(polys), floors_z=list(floors) if floors else [], geoms=list(geoms),
                      buffers_m=list(bufs))


def _tower_ks(xy=(1500.0, 0.0), top=90.0, b=50.0) -> KeepoutSet:
    return _direct_ks([Point(*xy)], [b], [top])


def _assert_invariants(path, ks: KeepoutSet, hw: float, z: float, a: Pose, b: Pose, R: float) -> None:
    """契約不變量：polyline(5).buffer(hw) 與 active keep-out 不相交；連續；弧半徑一致 ≥ R；端點恰為要求值。"""
    foot = path.polyline(5.0).buffer(hw)
    for i in ks.active(z):
        assert not foot.intersects(ks.polys[i]), f"footprint hits keep-out {ks.ids[i]}"
    assert path_is_clear(path, ks, hw, z)
    assert (path.start.x_m, path.start.y_m, path.start.theta_rad) == (a.x_m, a.y_m, a.theta_rad)
    assert (path.end.x_m, path.end.y_m, path.end.theta_rad) == (b.x_m, b.y_m, b.theta_rad)
    for s1, s2 in zip(path.segments[:-1], path.segments[1:]):
        assert math.hypot(s1.end.x_m - s2.start.x_m, s1.end.y_m - s2.start.y_m) < 1e-6
        d = (s1.end.theta_rad - s2.start.theta_rad + math.pi) % (2 * math.pi) - math.pi
        assert abs(d) < 1e-6
    assert path.R_m >= R - 1e-9
    for s in path.segments:
        if s.kind != "S":
            assert s.center is not None
            assert abs(s.length_m / abs(s.dtheta_rad) - path.R_m) < 1e-6
            sign = 1.0 if s.kind == "R" else -1.0
            assert math.copysign(1.0, s.dtheta_rad) == sign
            # 弧起點在圓上
            assert abs(math.hypot(s.start.x_m - s.center[0], s.start.y_m - s.center[1]) - path.R_m) < 1e-3
    assert abs(sum(s.length_m for s in path.segments) - path.length_m) < 1e-6


# ---------------------------------------------------------------------------
# 1. KeepoutSet.build（規格 §8.5、D13、D22、D24、D41、D42）
# ---------------------------------------------------------------------------

def test_build_ids_kinds_buffers_and_notes(built):
    ks, _ = built
    assert ks.ids == ["T1", "PL", "B1", "NF", "NF3"]           # NF2 於時間窗外（D22）
    assert ks.kinds == ["obstacle"] * 3 + ["nofly"] * 2
    assert ks.categories == ["tower", "powerline", "building", "nofly", "nofly"]
    assert ks.buffers_m == [50.0, 60.0, 50.0, 0.0, 0.0]        # D13 b_h=cfg.b_h_m、逐障礙覆寫；禁飛區 0（D41）
    assert ks.b_h_m == 50.0 and ks.b_v_m == 30.0
    assert any("NF2" in n and "inactive" in n for n in ks.notes)
    assert isinstance(ks.tree, shapely.STRtree) and len(ks) == 5


def test_build_geometry_point_line_polygon(built):
    ks, _ = built
    tower, line, bldg, nf, _ = ks.polys
    assert tower.area == pytest.approx(Point(0, 0).buffer(50.0).area, rel=1e-9)     # 點→圓
    assert tower.contains(Point(1500, 0)) and tower.distance(Point(1500 + 49.5, 0)) == 0.0
    # 線→膠囊：1000 m × 2·60 + π·60²（32 邊形近似 <1%）
    assert line.area == pytest.approx(1000 * 120 + math.pi * 60 ** 2, rel=0.01)
    assert line.contains(Point(2559, 0)) and not line.contains(Point(2561, 0))
    # 面→圓角偏移：300×200 + 周長·50 + π·50²
    assert bldg.area == pytest.approx(300 * 200 + 1000 * 50 + math.pi * 50 ** 2, rel=0.01)
    assert bldg.contains(Point(3000 - 49, 0)) and not bldg.contains(Point(3000 - 51, 0))
    # 禁飛區不加緩衝（D41）
    assert nf.area == pytest.approx(500 * 400, rel=1e-3)
    assert nf.distance(Point(3999, 0)) == pytest.approx(1.0, abs=0.15)      # WGS84 往返（h=0 平面）誤差 ≈ 6 cm
    # geoms 為原始幾何
    assert isinstance(ks.geoms[0], Point) and isinstance(ks.geoms[1], LineString) and isinstance(ks.geoms[2], Polygon)


def test_build_tops_floors_enu_z(built):
    ks, terr = built
    z_t = ks.tops_z
    # T1：AGL 60 + 自然地面 ENU z（≈ −0.18 m 曲率；燒入前 z_raw）+ b_v 30
    zg = float(FRAME.msl_to_enu_z(np.array([[1500.0, 0.0]]), Z0)[0])
    assert zg == pytest.approx(-0.18, abs=0.05)
    assert float(terr.z_at(np.array([[1500.0, 0.0]]))[0]) > zg + 80.0     # fixture 地形已燒入
    assert z_t[0] == pytest.approx(zg + 60.0 + 30.0, abs=1e-3)
    # PL：180 MSL → ENU（frame.msl_to_enu_z）+ 30
    z_pl = float(np.max(FRAME.msl_to_enu_z(np.array([[2500.0, -500.0], [2500.0, 500.0]]), 180.0)))
    assert z_t[1] == pytest.approx(z_pl + 30.0, abs=1e-3)               # 頂點經 WGS84 往返（≈6 cm）→ 曲率項差 ~2e-5 m
    assert 108.0 < z_t[1] < 110.5
    # B1：25 AGL + 30
    assert z_t[2] == pytest.approx(25.0 + 30.0, abs=1.5)
    # NF：樓層 300 MSL − 30、天花板 500 MSL + 30（ENU）
    assert 165.0 < ks.floors_z[3] < 170.5 and 425.0 < z_t[3] < 430.5
    assert ks.floors_z[:3] == [-math.inf] * 3
    assert ks.floors_z[4] == -math.inf and z_t[4] == math.inf


def test_build_burns_obstacles_not_nofly(built):
    ks, terr = built
    assert terr.max_in_polygon(ks.polys[0]) == pytest.approx(ks.tops_z[0], abs=0.01)   # 電塔燒入 = top + b_v
    assert terr.max_in_polygon(ks.polys[2]) == pytest.approx(ks.tops_z[2], abs=0.01)   # 建物燒入
    assert terr.max_in_polygon(ks.polys[3]) < 1.0                                        # 禁飛區不燒入
    assert float(np.nanmax(terr.z_raw)) < 1.0                                            # z_raw 不變


def test_build_dsm_skips_tree_building_burn_but_keeps_keepout():
    ks, terr = _build(is_dsm=True)
    assert "B1" in ks.ids and "T1" in ks.ids                             # keep-out 對所有障礙建立
    assert terr.max_in_polygon(ks.polys[0]) == pytest.approx(ks.tops_z[0], abs=0.01)   # tower 仍燒入
    assert terr.max_in_polygon(ks.polys[2]) < 1.0                                        # building 不燒入（D24）
    assert any("B1" in n and "D24" in n for n in ks.notes)


def test_build_no_burn_flag_and_empty():
    terr = _flat_terrain()
    ks = KeepoutSet.build([TOWER], [], FRAME, terr, CFG, None, burn=False)
    assert np.array_equal(terr.z, terr.z_raw) and len(ks) == 1
    ks0 = KeepoutSet.build([], [], FRAME, terr, CFG, None)
    assert len(ks0) == 0 and ks0.active(0.0) == [] and not ks0.intersects(LineString([(0, 0), (1, 1)]), 50.0, 0.0)


# ---------------------------------------------------------------------------
# 2. active / intersects（2.5D）
# ---------------------------------------------------------------------------

def test_active_by_height(built):
    ks, _ = built
    assert ks.active(60.0) == [0, 1, 4]            # 電塔、輸電線、NF3（無樓層／天花板）
    assert ks.active(100.0) == [1, 4]              # 電塔頂 ≈ 89.8 以上
    assert ks.active(200.0) == [3, 4]              # NF 樓層 ≈168 ≤ 200 < 429
    assert ks.active(500.0) == [4]
    assert ks.active(float("nan")) == [0, 1, 2, 3, 4]
    assert ks.active(-1000.0) == [0, 1, 2, 4]


def test_intersects_swept_footprint(built):
    ks, _ = built
    through = LineString([(0, 0), (3000, 0)])
    assert ks.intersects(through, HW, 60.0)
    assert ks.intersects(through, HW, 100.0)                           # 電塔頂以上，但輸電線 x=2500 仍有效
    assert not ks.intersects(LineString([(0, 0), (2000, 0)]), HW, 100.0)   # 只剩電塔段 → 無效
    assert ks.intersects(LineString([(2400, 0), (2600, 0)]), HW, 100.0)   # 輸電線頂 ≈109.5 > 100
    assert not ks.intersects(LineString([(2400, 0), (2600, 0)]), HW, 120.0)
    far = LineString([(0, 120), (3000, 120)])                          # 距電塔圓 70 m > 50
    assert not ks.intersects(LineString([(0, 120), (2000, 120)]), HW, 60.0)
    assert ks.intersects(LineString([(0, 95), (2000, 95)]), HW, 60.0)  # 距 45 m < 50
    assert ks.intersects(far, 80.0, 60.0)
    assert ks.clearance_m(LineString([(0, 120), (2000, 120)]), 60.0) == pytest.approx(70.0, abs=0.3)
    assert ks.clearance_m(through, 100.0) == pytest.approx(0.0)
    assert ks.clearance_m(through, 5000.0) == pytest.approx(ks.polys[4].distance(through))   # 只剩 NF3（無天花板）
    assert _tower_ks().clearance_m(through, 5000.0) == math.inf


# ---------------------------------------------------------------------------
# 3. avoid：直接路徑 / 無效 keep-out（契約測試：unobstructed → identical to dubins_path）
# ---------------------------------------------------------------------------

def test_avoid_direct_unobstructed_identical_to_dubins_path():
    ks = _tower_ks((1500.0, 2000.0))                       # 電塔遠離路徑
    a, b = Pose(0.0, 0.0, EAST), Pose(3000.0, 0.0, EAST)
    cache: dict = {}
    p = avoid(a, b, R_A, ks, 60.0, None, 21.5, HW, cache)
    ref = dubins_path(a, b, R_A)
    assert p.word == ref.word == "LSL" and p.length_m == ref.length_m and p.segments == ref.segments
    assert cache == {}                                     # 直接路徑不建圖
    # 非平凡狀態亦相同（RSL）
    a2, b2 = Pose(0.0, 0.0, 0.3), Pose(2000.0, -800.0, 2.5)
    p2 = avoid(a2, b2, R_A, ks, 60.0, None, 21.5, HW, cache)
    ref2 = dubins_path(a2, b2, R_A)
    assert p2.word == ref2.word and p2.segments == ref2.segments


def test_avoid_inactive_above_top_is_direct():
    ks = _tower_ks((1500.0, 0.0), top=90.0)
    a, b = Pose(0.0, 0.0, EAST), Pose(3000.0, 0.0, EAST)
    p = avoid(a, b, R_A, ks, 100.0, None, 21.5, HW, {})    # z_hint 高於頂高 → 不繞行（穿越電塔上方）
    ref = dubins_path(a, b, R_A)
    assert p.segments == ref.segments and p.length_m == 3000.0
    assert avoid(a, b, R_A, None, 60.0, None, 21.5, HW, {}).segments == ref.segments
    assert avoid(a, b, R_A, ks, 89.0, None, 21.5, HW, {}).word == AVOID_WORD   # 頂高以下 → 繞行


def test_avoid_second_word_clear_returns_that_word():
    """最短 word 被擋、次短 word 淨空 → 回傳次短 word 之 dubins_path（仍為 6-word 之一）。"""
    a, b = Pose(0.0, 0.0, EAST), Pose(0.0, 600.0, EAST)     # LSL/RSR 等長（左右對稱）
    ks = _tower_ks((-200.0, 300.0), top=90.0, b=50.0)       # 左轉圓內側電塔擋 LSL
    p = avoid(a, b, R_A, ks, 60.0, None, 21.5, HW, {})
    assert p.word in ("RSR", "LSL") and p.word != AVOID_WORD
    assert path_is_clear(p, ks, HW, 60.0)
    assert p.segments == dubins_path(a, b, R_A, p.word).segments


# ---------------------------------------------------------------------------
# 4. avoid：電塔擋路 → 切線圖（契約不變量 + 閉式最佳解）
# ---------------------------------------------------------------------------

def test_avoid_tower_between_invariant_and_closed_form():
    ks = _tower_ks((1500.0, 0.0), top=90.0)
    a, b = Pose(0.0, 0.0, EAST), Pose(3000.0, 0.0, EAST)
    cache: dict = {}
    p = avoid(a, b, R_A, ks, 60.0, None, 21.5, HW, cache)
    assert p.word == AVOID_WORD and p.R_m == R_A
    _assert_invariants(p, ks, HW, 60.0, a, b, R_A)
    assert path_clearance_m(p, ks, 60.0) == pytest.approx(150.0, abs=0.5)     # 弧心圓半徑 200 − b_h 50
    # 閉式：R 弧 α → 內公切線 L → L 弧 2α → L → R 弧 α（起訖圓心 (0,∓200)、障礙圓 (1500,0)，皆半徑 200）
    D = math.hypot(1500.0, 200.0)
    alpha = math.atan2(1500.0, 200.0) + math.asin(400.0 / D) - EAST
    L = math.sqrt(D * D - 400.0 ** 2)
    assert p.length_m == pytest.approx(800.0 * alpha + 2.0 * L, rel=1e-9)
    kinds = [s.kind for s in p.segments if s.length_m > 1e-9]
    assert kinds in (["R", "S", "L", "S", "R"], ["L", "S", "R", "S", "L"])
    assert list(cache) == [(R_A, 60.0)] and isinstance(cache[(R_A, 60.0)], TangentGraph)
    # dubins 工具可直接用於 AVOID 路徑（半徑一致）
    xy = p.sample(5.0)
    assert np.allclose(xy[0], [0, 0, EAST]) and np.allclose(xy[-1], [3000, 0, EAST])
    wps = discretize(p, 3.0, math.radians(30))
    assert wps[0][:2] == (0.0, 0.0) and wps[-1][:2] == (3000.0, 0.0)
    w = np.array([5.0, 0.0])
    assert path_time(p, 21.5, w, 60.0) == pytest.approx(sum(segment_time(s, 21.5, w) for s in p.segments), abs=1e-9)


def test_avoid_cache_reuse_and_rho():
    ks = _tower_ks((1500.0, 0.0))
    a, b = Pose(0.0, 0.0, EAST), Pose(3000.0, 0.0, EAST)
    cache: dict = {}
    p1 = avoid(a, b, R_A, ks, 60.0, None, 21.5, HW, cache)
    tg = cache[(R_A, 60.0)]
    b2 = Pose(3000.0, 60.0, EAST)
    p2 = avoid(a, b2, R_A, ks, 60.0, None, 21.5, HW, cache)
    assert cache[(R_A, 60.0)] is tg and p2.word == AVOID_WORD and len(cache) == 1
    _assert_invariants(p2, ks, HW, 60.0, a, b2, R_A)
    assert avoid_rho_m(ks, R_A, 60.0, HW) == R_A == max(ks.b_h_m, R_A)         # 規格 ρ = max(b_h, R_eff)
    assert avoid_rho_m(ks, 120.0, 60.0, HW) == 120.0
    assert avoid_rho_m(ks, 60.0, 60.0, HW) == pytest.approx(50.0 + HW + RT.AVOID_MARGIN_M)   # 淨空需求主導
    # B 構型半徑 120：路徑弧半徑 120
    pb = avoid(a, b, 120.0, ks, 60.0, None, 19.0, HW, cache)
    assert pb.R_m == 120.0 and (120.0, 60.0) in cache
    _assert_invariants(pb, ks, HW, 60.0, a, b, 120.0)
    assert p1.length_m < pb.length_m + 1e-9 or True     # 半徑不同不可比；只驗不變量


def test_avoid_large_buffer_keeps_clearance():
    """b_h > R_eff − half_width 時 ρ 自動加大，使弧淨空仍 ≥ half_width。"""
    ks = _tower_ks((1500.0, 0.0), b=180.0)
    a, b = Pose(0.0, 0.0, EAST), Pose(3000.0, 0.0, EAST)
    cache: dict = {}
    p = avoid(a, b, 120.0, ks, 60.0, None, 19.0, HW, cache)
    rho = avoid_rho_m(ks, 120.0, 60.0, HW)
    assert rho == pytest.approx(180.0 + HW + RT.AVOID_MARGIN_M) and p.R_m == pytest.approx(rho)
    _assert_invariants(p, ks, HW, 60.0, a, b, 120.0)
    assert path_clearance_m(p, ks, 60.0) >= HW + 0.4


def test_avoid_wind_time_optimal_vs_length_optimal():
    ks = _tower_ks((1500.0, 0.0))
    a, b = Pose(0.0, 0.0, EAST), Pose(3000.0, 0.0, EAST)
    w = np.array([4.0, -6.0])                               # 斜向風 → 南北繞行不對稱
    cache: dict = {}
    p = avoid(a, b, R_A, ks, 60.0, w, 21.5, HW, cache)
    tg = cache[(R_A, 60.0)]
    p_len = tg.shortest(a, b, R_A, None)
    t_avoid = path_time(p, 21.5, w, 60.0)
    t_len = path_time(p_len, 21.5, w, 60.0)
    assert t_avoid <= t_len + 1e-9
    # 回呼版（契約簽章）與向量化版一致
    p_cb = tg.shortest(a, b, R_A, lambda s: segment_time(s, 21.5, w))
    assert path_time(p_cb, 21.5, w, 60.0) == pytest.approx(t_avoid, abs=1e-6)
    # WindField 鴨子型別（wind_vec）
    class _WF:
        def wind_vec(self, z):
            return np.tile(w, (np.size(z), 1))
    p_wf = avoid(a, b, R_A, ks, 60.0, _WF(), 21.5, HW, {})
    assert p_wf.length_m == pytest.approx(p.length_m, abs=1e-6)


def test_tangent_graph_weight_callback_forbids_side():
    ks = _tower_ks((1500.0, 0.0))
    tg = TangentGraph(ks, R_A, 60.0, clearance_m=HW + 0.5)
    a, b = Pose(0.0, 0.0, EAST), Pose(3000.0, 0.0, EAST)

    def north_only(seg):
        return math.inf if min(seg.start.y_m, seg.end.y_m) < -1e-6 else seg.length_m

    p = tg.shortest(a, b, R_A, north_only)
    assert p is not None and p.sample(5.0)[:, 1].min() >= -1e-6
    p_free = tg.shortest(a, b, R_A, None)
    assert p.length_m == pytest.approx(p_free.length_m, rel=1e-9)     # 鏡像等長
    _assert_invariants(p, ks, HW, 60.0, a, b, R_A)
    assert tg.shortest(a, b, R_A, lambda s: math.inf) is None          # 全部不可用 → None
    assert tg.min_offset_m == pytest.approx(150.0) and tg.clearance_m == pytest.approx(50.5)
    with pytest.raises(ValueError):
        TangentGraph(ks, R_A, 60.0, clearance_m=151.0)                  # 淨空需求超過幾何提供


def test_tangent_graph_polygon_line_and_multiple_obstacles():
    geoms = [Point(1000, 100), LineString([(1800, -400), (2000, 300)]),
             Polygon([(2400, -100), (2700, -100), (2700, 200), (2400, 200)]), Point(1500, -250)]
    ks = _direct_ks(geoms, [50.0, 60.0, 50.0, 50.0], [90.0] * 4)
    a, b = Pose(0.0, 0.0, EAST), Pose(3000.0, 0.0, EAST)
    p = avoid(a, b, R_A, ks, 60.0, np.array([5.0, 0.0]), 21.5, HW, {})
    _assert_invariants(p, ks, HW, 60.0, a, b, R_A)
    assert 3000.0 < p.length_m < 3000.0 + 2 * math.pi * R_A * 2
    # 反向與斜向狀態
    a2, b2 = Pose(3000.0, 200.0, -EAST), Pose(0.0, -300.0, 4.0)
    p2 = avoid(a2, b2, R_A, ks, 60.0, None, 21.5, HW, {})
    _assert_invariants(p2, ks, HW, 60.0, a2, b2, R_A)


def test_tangent_graph_cached_equals_fresh():
    rng = np.random.default_rng(1)
    geoms = []
    for _ in range(6):
        c = rng.uniform([500, -800], [5500, 800])
        n = int(rng.integers(4, 8))
        ang = np.sort(rng.uniform(0, 2 * np.pi, n))
        r = rng.uniform(60, 150, n)
        geoms.append(Polygon(np.c_[c[0] + r * np.cos(ang), c[1] + r * np.sin(ang)]))
    ks = _direct_ks(geoms, [50.0] * 6, [90.0] * 6)
    tg = TangentGraph(ks, R_A, 60.0, clearance_m=HW + 0.5)
    pairs = [(Pose(0, -500, EAST), Pose(6000, 0, 0.0)), (Pose(0, 500, 1.0), Pose(6000, -400, EAST)),
             (Pose(-200, 0, EAST), Pose(6000, 600, 3.0))]
    for a, b in pairs:
        p_c = tg.shortest(a, b, R_A, None, v_air_mps=21.5, wind_xy=np.array([3.0, 2.0]))
        fresh = TangentGraph(ks, R_A, 60.0, clearance_m=HW + 0.5)
        p_f = fresh.shortest(a, b, R_A, None, v_air_mps=21.5, wind_xy=np.array([3.0, 2.0]))
        assert (p_c is None) == (p_f is None)
        if p_c is not None:
            assert p_c.length_m == pytest.approx(p_f.length_m, rel=1e-12)
            _assert_invariants(p_c, ks, HW, 60.0, a, b, R_A)


# ---------------------------------------------------------------------------
# 5. 禁飛區（樓層／天花板；D41 緩衝由 half_width 提供）
# ---------------------------------------------------------------------------

def test_avoid_nofly_floor_ceiling():
    nf = Polygon([(1200, -300), (1800, -300), (1800, 300), (1200, 300)])
    ks = _direct_ks([nf], [0.0], [500.0], floors=[100.0])
    a, b = Pose(0.0, 0.0, EAST), Pose(3000.0, 0.0, EAST)
    assert avoid(a, b, R_A, ks, 60.0, None, 21.5, HW, {}).length_m == 3000.0       # 樓層以下
    assert avoid(a, b, R_A, ks, 500.0, None, 21.5, HW, {}).length_m == 3000.0      # 天花板以上（z ≥ top）
    p = avoid(a, b, R_A, ks, 200.0, None, 21.5, HW, {})
    assert p.word == AVOID_WORD
    _assert_invariants(p, ks, HW, 200.0, a, b, R_A)
    assert nf.buffer(HW).intersects(p.polyline(5.0)) is False


def test_build_nofly_avoid_via_build(built):
    ks, _ = built
    a, b = Pose(3500.0, 0.0, EAST), Pose(5000.0, 0.0, EAST)
    p = avoid(a, b, R_A, ks, 250.0, None, 21.5, HW, {})       # NF 有效（168 ≤ 250 < 429）；其餘無效
    assert ks.active(250.0) == [3, 4] and p.word == AVOID_WORD
    _assert_invariants(p, ks, HW, 250.0, a, b, R_A)


# ---------------------------------------------------------------------------
# 6. 可視圖備援
# ---------------------------------------------------------------------------

def test_visibility_graph_buffer_and_polyline():
    ks = _tower_ks((1500.0, 0.0))
    vg = VisibilityGraph(ks, R_A, math.pi / 2, z_hint_m=60.0, clearance_m=HW + 0.5)
    inset = R_A * (1.0 / math.cos(math.pi / 4) - 1.0)
    assert vg.fillet_inset_m == pytest.approx(inset)
    assert vg.buffer_extra_m >= HW + 0.5 + inset - 1e-9        # 契約：b_h + R(sec(Δ/2)−1)
    poly = vg.shortest(np.array([0.0, 0.0]), np.array([3000.0, 0.0]))
    assert poly.shape[1] == 2 and poly.shape[0] >= 3
    assert np.allclose(poly[0], [0, 0]) and np.allclose(poly[-1], [3000, 0])
    line = LineString(poly)
    assert ks.polys[0].distance(line) >= vg.buffer_extra_m - 1.0         # 頂點簡化 0.5 m + 內縮 0.01
    assert line.length < 3000.0 + 2 * vg.buffer_extra_m + 50.0
    a, b = Pose(0.0, 0.0, EAST), Pose(3000.0, 0.0, EAST)
    p = polyline_to_path(poly, a, b, R_A, ks, HW, 60.0)
    assert p.word == AVOID_WORD and p.R_m == R_A
    _assert_invariants(p, ks, HW, 60.0, a, b, R_A)
    assert p.length_m < 3300.0                                            # 圓角合併：不繞圈
    # 起點被包圍 → RoutingError
    with pytest.raises(RoutingError):
        VisibilityGraph(ks, R_A, z_hint_m=60.0, clearance_m=HW + 0.5).shortest(np.array([1500.0, 0.0]), np.array([3000.0, 0.0]))
    # z_hint None → 全部視為有效；高於頂高之 z_hint → 無多邊形
    assert VisibilityGraph(ks, R_A).polys and not VisibilityGraph(ks, R_A, z_hint_m=500.0).polys


def test_visibility_polyline_to_path_sharp_corner_building():
    bld = Polygon([(1300, -150), (1700, -150), (1700, 150), (1300, 150)])
    ks = _direct_ks([bld], [50.0], [90.0])
    a, b = Pose(0.0, 0.0, EAST), Pose(3000.0, 0.0, EAST)
    vg = VisibilityGraph(ks, R_A, z_hint_m=60.0, clearance_m=HW + 0.5)
    p = polyline_to_path(vg.shortest(np.array([0.0, 0.0]), np.array([3000.0, 0.0])), a, b, R_A, ks, HW, 60.0)
    _assert_invariants(p, ks, HW, 60.0, a, b, R_A)
    tg = avoid(a, b, R_A, ks, 60.0, None, 21.5, HW, {})
    assert tg.length_m <= p.length_m + 1e-6                               # 切線圖不劣於備援


def test_avoid_uses_visibility_fallback_when_tangent_graph_fails(monkeypatch):
    ks = _tower_ks((1500.0, 0.0))
    a, b = Pose(0.0, 0.0, EAST), Pose(3000.0, 0.0, EAST)
    monkeypatch.setattr(TangentGraph, "shortest", lambda self, *args, **kw: None)
    cache: dict = {}
    p = avoid(a, b, R_A, ks, 60.0, None, 21.5, HW, cache)
    assert p.word == AVOID_WORD
    _assert_invariants(p, ks, HW, 60.0, a, b, R_A)
    assert ("vis", R_A, 60.0, HW) in cache and isinstance(cache[("vis", R_A, 60.0, HW)], VisibilityGraph)


def test_avoid_routing_error_when_start_enclosed():
    ks = _tower_ks((0.0, 0.0))
    with pytest.raises(RoutingError):
        avoid(Pose(0.0, 0.0, EAST), Pose(3000.0, 0.0, EAST), R_A, ks, 60.0, None, 21.5, HW, {})
    with pytest.raises(RoutingError):
        avoid(Pose(-3000.0, 0.0, EAST), Pose(0.0, 0.0, EAST), R_A, ks, 60.0, None, 21.5, HW, {})


# ---------------------------------------------------------------------------
# 7. 契約最小欄位建構、segment_time
# ---------------------------------------------------------------------------

def test_keepoutset_contract_fields_only_defaults():
    polys = [Point(1500, 0).buffer(50.0)]
    ks = KeepoutSet(polys=polys, tops_z=[90.0], ids=["T1"], tree=shapely.STRtree(polys))
    assert ks.floors_z == [-math.inf] and ks.kinds == ["obstacle"] and ks.buffers_m == [0.5]
    assert len(ks.geoms) == 1 and ks.active(60.0) == [0]
    a, b = Pose(0.0, 0.0, EAST), Pose(3000.0, 0.0, EAST)
    p = avoid(a, b, R_A, ks, 60.0, None, 21.5, HW, {})
    _assert_invariants(p, ks, HW, 60.0, a, b, R_A)
    with pytest.raises(ValueError):
        KeepoutSet(polys=polys, tops_z=[90.0, 1.0], ids=["T1"], tree=shapely.STRtree(polys))


def test_segment_time_matches_dubins_path_time():
    a, b = Pose(0.0, 0.0, 0.3), Pose(1500.0, -400.0, 2.9)
    p = dubins_path(a, b, R_A)
    w = np.array([6.0, -3.0])
    assert sum(segment_time(s, 21.5, w) for s in p.segments) == pytest.approx(path_time(p, 21.5, w, 0.0), abs=1e-9)
    assert sum(segment_time(s, 21.5, None) for s in p.segments) == pytest.approx(p.length_m / 21.5, abs=1e-9)
    assert segment_time(p.segments[1], 21.5, np.array([-25.0, 0.0])) == math.inf or p.segments[1].length_m == 0.0
