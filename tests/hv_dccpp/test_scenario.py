"""tests/test_scenario.py — 模組契約 §16（scenario.py）：合成 DEM 形狀／山丘／山脊、§W.1 mini example 逐項、
山丘情境建構與 load_mission JSON 往返、論文 Table 2 情境（規格 §15、§17.1 (7) 之輸入）。
"""
from __future__ import annotations

import json
import math

import numpy as np
import pytest
import shapely

from hv_dccpp import scenario as S
from hv_dccpp.geo import DemGrid, GeoFrame
from hv_dccpp.mission import (
    AreaSpec,
    MissionInputError,
    MissionInputs,
    NoFlyZone,
    Obstacle,
    PlannerConfig,
    Site,
    load_mission,
)
from hv_dccpp.platform import (
    PLATFORM_HERO,
    PLATFORM_PIONEER,
    CameraHardware,
    LidarHardware,
    PlatformSpec,
    TrapezoidFovSpec,
    sensor_model,
)


# ---------------------------------------------------------------------------
# 工具
# ---------------------------------------------------------------------------

def _enu_poly(frame: GeoFrame, area: AreaSpec, h_msl: float = 0.0) -> shapely.Polygon:
    ring = frame.to_enu_xy([p[0] for p in area.polygon_wgs84], [p[1] for p in area.polygon_wgs84], h_msl)
    holes = [frame.to_enu_xy([p[0] for p in h], [p[1] for p in h], h_msl) for h in area.holes_wgs84]
    return shapely.Polygon(ring, holes)


def _enu_xy(frame: GeoFrame, site: Site, h_msl: float = 0.0) -> np.ndarray:
    return frame.to_enu_xy(site.lat_deg, site.lon_deg, h_msl)[0]


def _is_convex(poly: shapely.Polygon, tol_m2: float = 1.0) -> bool:
    return abs(poly.convex_hull.area - poly.area) < tol_m2


def _dem_slope_deg(dem: DemGrid, step_m: float) -> np.ndarray:
    gy, gx = np.gradient(dem.z_msl.astype(np.float64), step_m)
    return np.degrees(np.arctan(np.hypot(gx, gy)))


def _sample_slope_deg(dem: DemGrid, lat: float, lon: float, d_m: float = 30.0) -> float:
    m_lat, m_lon = S.metres_per_degree(lat)
    z0 = float(dem.sample_msl(lat, lon)[0])
    zn = float(dem.sample_msl(lat + d_m / m_lat, lon)[0])
    ze = float(dem.sample_msl(lat, lon + d_m / m_lon)[0])
    return math.degrees(math.atan(math.hypot((zn - z0) / d_m, (ze - z0) / d_m)))


@pytest.fixture(scope="module")
def mini() -> tuple[MissionInputs, PlannerConfig]:
    return S.build_mini_example()


@pytest.fixture(scope="module")
def hills() -> tuple[MissionInputs, PlannerConfig]:
    return S.build_hills_scenario()


@pytest.fixture(scope="module")
def paper() -> tuple[MissionInputs, PlannerConfig]:
    return S.build_paper_table2_scenario()


# ---------------------------------------------------------------------------
# synthetic_dem：形狀、平地、山丘、山脊、輸入驗證
# ---------------------------------------------------------------------------

def test_synthetic_dem_shape_and_spacing():
    dem = S.synthetic_dem(24.0, 121.0, (3.0, 1.5), 50.0, base_msl_m=250.0)
    assert isinstance(dem, DemGrid)
    assert dem.z_msl.shape == (31, 61)                  # rows = ⌈1500/50⌉+1、cols = ⌈3000/50⌉+1
    assert dem.z_msl.dtype == np.float32
    assert dem.lat0_deg == 24.0 and dem.lon0_deg == 121.0
    m_lat, m_lon = S.metres_per_degree(24.0 + 750.0 / S.metres_per_degree(24.0)[0])   # 模組以中心緯度換算
    assert dem.dlat_deg * m_lat == pytest.approx(50.0, abs=1e-6)
    assert dem.dlon_deg * m_lon == pytest.approx(50.0, abs=1e-6)
    assert dem.dlon_deg * S.metres_per_degree(24.0)[1] == pytest.approx(50.0, abs=1e-2)   # 南緣（cos φ 差 0.75 km）亦 ≈ 50 m
    assert np.all(dem.z_msl == np.float32(250.0))
    assert dem.sigma_z_m is None and dem.is_dsm is False
    # 純量 size_km → 正方形；sigma／is_dsm 轉交
    d2 = S.synthetic_dem(24.0, 121.0, 1.0, 100.0, sigma_z_m=2.0, is_dsm=True)
    assert d2.z_msl.shape == (11, 11) and d2.sigma_z_m == 2.0 and d2.is_dsm is True
    # 非整數倍：⌈⌉+1
    d3 = S.synthetic_dem(24.0, 121.0, (0.95, 0.42), 100.0)
    assert d3.z_msl.shape == (6, 11)


def test_synthetic_dem_hills_amplitude_and_lattice_flatness():
    A, lam, step = 12.0, 3000.0, 30.0
    dem = S.synthetic_dem(24.0, 121.0, (9.0, 6.0), step, base_msl_m=100.0, amplitude_m=A, wavelength_m=lam)
    z = dem.z_msl.astype(np.float64)
    assert z.max() <= 100.0 + A + 1e-3 and z.min() >= 100.0 - A - 1e-3
    assert z.max() > 100.0 + 0.95 * A and z.min() < 100.0 - 0.95 * A   # 山丘確實存在
    # 最大坡度 atan(2πA/λ)
    sl = _dem_slope_deg(dem, step)
    assert sl.max() <= math.degrees(math.atan(2 * math.pi * A / lam)) + 0.05
    # 半波長格點上梯度 = 0（節點 (u,v) = (k λ/2, m λ/2) → 索引 50·k, 50·m）
    gy, gx = np.gradient(z, step)
    for i, j in ((50, 50), (100, 100), (0, 150), (100, 250)):
        assert abs(gx[i, j]) < 1e-9 and abs(gy[i, j]) < 1e-9
    # 無山丘 → 平地；noise → 由 seed 決定且有界
    flat = S.synthetic_dem(24.0, 121.0, (1.0, 1.0), 50.0)
    assert np.ptp(flat.z_msl) == 0.0
    n1 = S.synthetic_dem(24.0, 121.0, (2.0, 2.0), 50.0, noise_m=3.0, seed=1)
    n1b = S.synthetic_dem(24.0, 121.0, (2.0, 2.0), 50.0, noise_m=3.0, seed=1)
    n2 = S.synthetic_dem(24.0, 121.0, (2.0, 2.0), 50.0, noise_m=3.0, seed=2)
    assert np.array_equal(n1.z_msl, n1b.z_msl) and not np.array_equal(n1.z_msl, n2.z_msl)
    assert np.abs(n1.z_msl.astype(float) - 100.0).max() <= 3.0 + 1e-3


def test_synthetic_dem_ridge_steeper_than_3deg():
    lat0, lon0, step = 24.0, 121.0, 20.0
    m_lat, m_lon = S.metres_per_degree(lat0 + 3.0 / 111.0)
    # 山脊中心於 DEM 內 (u, v) = (4000, 3000) m，方位 30°，高 30、寬 400、長 4500
    center = (lat0 + 3000.0 / m_lat, lon0 + 4000.0 / m_lon)
    ridge = {"center_latlon": center, "azimuth_deg": 30.0, "height_m": 30.0, "width_m": 400.0, "length_m": 4500.0}
    dem = S.synthetic_dem(lat0, lon0, (8.0, 6.0), step, base_msl_m=100.0, ridge=ridge)
    z = dem.z_msl.astype(np.float64)
    assert z.max() == pytest.approx(130.0, abs=0.05)             # 脊頂 = base + height
    assert z.min() == pytest.approx(100.0, abs=1e-6)
    sl = _dem_slope_deg(dem, step)
    assert sl.max() > 3.0
    assert sl.max() == pytest.approx(math.degrees(math.atan(30.0 * math.pi / 400.0)), abs=0.5)   # 13.3°
    # 沿脊線（方位 30°）之中段為全高；橫向 ±200 m 外為平地；長度外為平地
    t = np.array([math.sin(math.radians(30.0)), math.cos(math.radians(30.0))])
    n = np.array([math.cos(math.radians(30.0)), -math.sin(math.radians(30.0))])
    c = np.array([4000.0, 3000.0])

    def z_at(uv: np.ndarray) -> float:
        return float(dem.sample_msl(lat0 + uv[1] / m_lat, lon0 + uv[0] / m_lon)[0])

    # 脊頂為曲率最大處，20 m 節點之雙線性取樣誤差 ≤ h(π/(w/2))²·step²/8 ≈ 0.37 m
    assert z_at(c) == pytest.approx(130.0, abs=0.5)
    assert z_at(c + 1500.0 * t) == pytest.approx(130.0, abs=0.5)
    assert z_at(c + 100.0 * n) == pytest.approx(100.0 + 15.0, abs=0.3)   # 升餘弦於 d⊥ = w/4 → h/2（反曲點，內插精確）
    assert z_at(c + 250.0 * n) == pytest.approx(100.0, abs=0.1)
    assert z_at(c + 2400.0 * t) == pytest.approx(100.0, abs=0.1)


def test_synthetic_dem_input_validation():
    with pytest.raises(MissionInputError):
        S.synthetic_dem(24.0, 121.0, (0.0, 1.0), 10.0)
    with pytest.raises(MissionInputError):
        S.synthetic_dem(24.0, 121.0, (1.0, 1.0), 0.0)
    with pytest.raises(MissionInputError):
        S.synthetic_dem(24.0, 121.0, (1.0, 1.0), 10.0, amplitude_m=-1.0)
    with pytest.raises(MissionInputError):
        S.synthetic_dem(24.0, 121.0, (1.0, 1.0), 10.0, ridge={"center_latlon": (24.0, 121.0), "height_m": 10.0})
    with pytest.raises(MissionInputError):
        S.synthetic_dem(24.0, 121.0, (1.0, 1.0), 10.0,
                        ridge={"center_latlon": (24.0, 121.0), "azimuth_deg": 0.0, "height_m": 10.0, "width_m": 0.0,
                               "length_m": 100.0})
    with pytest.raises(MissionInputError):
        S.synthetic_dem_for_latlon_bounds(24.0, 121.0, 23.0, 122.0, 100.0)


def test_synthetic_dem_for_frame_covers_enu_bounds():
    frame = GeoFrame(24.0, 121.0, 100.0)
    dem = S.synthetic_dem_for_frame(frame, (-2000.0, -1000.0, 6000.0, 3000.0), 50.0)
    sw = frame.to_enu_xy(dem.lat0_deg, dem.lon0_deg, 100.0)[0]
    ne = frame.to_enu_xy(dem.lat_max_deg, dem.lon_max_deg, 100.0)[0]
    assert sw == pytest.approx((-2000.0, -1000.0), abs=0.5)
    assert ne[0] >= 6000.0 - 1.0 and ne[1] >= 3000.0 - 1.0 and ne[0] < 6100.0 and ne[1] < 3100.0
    assert dem.z_msl.shape == (81, 161)


# ---------------------------------------------------------------------------
# build_mini_example：契約 §W.1 逐項
# ---------------------------------------------------------------------------

def test_mini_example_values(mini):
    inputs, cfg = mini
    assert isinstance(inputs, MissionInputs) and cfg == PlannerConfig()
    # home / DEM
    assert (inputs.home.lat_deg, inputs.home.lon_deg) == (23.95, 120.90)
    assert inputs.home.kind == "home" and inputs.home.alt_msl_m is None and inputs.home.alt_agl_m is None
    dem = inputs.dem
    assert np.all(dem.z_msl == np.float32(100.0)) and dem.sigma_z_m == 2.0 and dem.is_dsm is False
    m_lat, m_lon = S.metres_per_degree(23.95)
    assert dem.dlat_deg * m_lat == pytest.approx(10.0, abs=1e-3) and dem.dlon_deg * m_lon == pytest.approx(10.0, abs=1e-3)
    frame = S.scenario_frame(inputs)
    assert frame.h0_msl_m == pytest.approx(100.0, abs=1e-6)
    # 區域 Z1：ENU (4000,−600)→(7000,600)
    z1 = inputs.areas[0]
    assert len(inputs.areas) == 1 and z1.area_id == "Z1" and z1.sensor_req == "any" and z1.priority == 1.0
    assert z1.payload_req_id == "default" and z1.holes_wgs84 == () and z1.time_window_s is None
    poly = _enu_poly(frame, z1, 100.0)
    assert poly.bounds == pytest.approx((4000.0, -600.0, 7000.0, 600.0), abs=0.05)
    assert poly.area == pytest.approx(3000.0 * 1200.0, rel=1e-4)
    # DEM 涵蓋區域、站點與迴轉帶裕度
    assert dem.contains([p[0] for p in z1.polygon_wgs84], [p[1] for p in z1.polygon_wgs84]).all()
    sw = frame.to_enu_xy(dem.lat0_deg, dem.lon0_deg, 100.0)[0]
    ne = frame.to_enu_xy(dem.lat_max_deg, dem.lon_max_deg, 100.0)[0]
    assert sw[0] <= -2000.0 and sw[1] <= -2000.0 and ne[0] >= 8000.0 and ne[1] >= 2000.0
    # 備降點 ALT1 ENU (2000, 1500)
    assert len(inputs.alternates) == 1 and inputs.alternates[0].site_id == "ALT1"
    assert _enu_xy(frame, inputs.alternates[0], 100.0) == pytest.approx((2000.0, 1500.0), abs=0.05)
    assert inputs.recovery_site.site_id == "H"
    # 風／氣象
    assert len(inputs.wind) == 1
    w = inputs.wind[0]
    assert (w.speed_mps, w.dir_from_deg, w.gust_mps) == (5.0, 270.0, 7.0)
    assert w.wind_vec() == pytest.approx((5.0, 0.0), abs=1e-12)          # 西風吹向東
    o = inputs.weather_obs
    assert (o.mean_wind_mps, o.gust_mps, o.wind_dir_from_deg, o.visibility_m, o.precip_mmh, o.temp_c) == (5.0, 7.0, 270.0, 8000.0, 0.0, 25.0)
    assert inputs.obstacles == () and inputs.airspace.no_fly == ()
    # 機隊
    assert [u.uav_id for u in inputs.fleet] == ["A1", "B1"]
    a1, b1 = inputs.fleet
    assert a1.spec is PLATFORM_PIONEER and a1.sensors == (LidarHardware(),) and a1.available_kinds == {"lidar"}
    assert (a1.sortie.auw_kg, a1.sortie.payload_kg, a1.sortie.available_wh, a1.sortie.pack_nominal_wh) == (24.6, 5.0, 1350.0, 1500.0)
    assert b1.spec is PLATFORM_HERO and b1.sensors == (CameraHardware(),) and b1.available_kinds == {"rgb"}
    assert (b1.sortie.auw_kg, b1.sortie.payload_kg, b1.sortie.available_wh, b1.sortie.pack_nominal_wh) == (6.65, 0.6, 380.0, 420.0)
    # 酬載需求 → §W.3 足跡數字
    rq = inputs.payload_reqs
    assert set(rq) == {"default", "default_rgb"}
    assert (rq["default"].kind, rq["default"].density_pt_m2, rq["default"].side_overlap, rq["default"].h_target_agl_m) == ("lidar", 20.0, 0.25, 160.0)
    r = rq["default_rgb"]
    assert (r.kind, r.gsd_required_m, r.side_overlap, r.forward_overlap, r.h_target_agl_m) == ("rgb", 0.05, 0.30, 0.70, 100.0)
    sa = sensor_model(LidarHardware(), rq["default"])
    assert sa.swath_m(160.0) == pytest.approx(224.07, abs=0.01) and sa.strip_spacing_m(160.0) == pytest.approx(168.05, abs=0.01)
    sb = sensor_model(CameraHardware(), r)
    assert sb.swath_m(100.0) == pytest.approx(146.875, abs=1e-6) and sb.strip_spacing_m(100.0) == pytest.approx(102.81, abs=0.01)
    assert sb.agl_window()[1] == pytest.approx(205.1, abs=0.1)
    # C2 / 空域
    assert inputs.c2.range_km_for("A", PLATFORM_PIONEER) == 25.0 and inputs.c2.range_km_for("B", PLATFORM_HERO) == 25.0
    assert inputs.c2.ground_station is inputs.home and inputs.c2.coverage_polygon_wgs84 is None
    asp = inputs.airspace
    assert (asp.max_agl_m, asp.waiver_max_agl_m, asp.terrain_clearance_m) == (120.0, 200.0, 36.0)
    assert PLATFORM_PIONEER.h_ceil(asp.max_agl_m, asp.waiver_max_agl_m) == 200.0
    assert asp.terrain_clearance_m >= 3.0 * dem.sigma_z_m + 30.0
    assert inputs.platform_overrides == {} and inputs.recovery_site_id is None


def test_mini_example_dict_roundtrip(mini):
    inputs, _ = mini
    d = inputs.to_dict()
    m2 = load_mission(d)
    assert m2.to_dict() == d
    assert m2.fleet[0].spec == PLATFORM_PIONEER and m2.dem.z_msl.shape == inputs.dem.z_msl.shape


# ---------------------------------------------------------------------------
# build_hills_scenario
# ---------------------------------------------------------------------------

def test_hills_scenario_structure(hills):
    inputs, cfg = hills
    assert cfg == PlannerConfig()
    frame = S.scenario_frame(inputs)
    assert frame.h0_msl_m == pytest.approx(100.0, abs=0.01)
    # 機隊 3×A + 5×B
    assert len(inputs.fleet) == 8
    assert [u.uav_id for u in inputs.uavs_of_class("A")] == ["A1", "A2", "A3"]
    assert [u.uav_id for u in inputs.uavs_of_class("B")] == ["B1", "B2", "B3", "B4", "B5"]
    for u in inputs.fleet:
        spec = u.spec
        assert spec in (PLATFORM_PIONEER, PLATFORM_HERO)
        assert u.sortie.auw_kg <= spec.mtow_kg
        assert u.sortie.payload_kg <= spec.payload_max_kg
        assert 0 < u.sortie.available_wh <= u.sortie.pack_nominal_wh
        assert u.sortie.nominal_wh_resolved(spec) == u.sortie.pack_nominal_wh
    # 3 區：lidar 凸／both 凹含洞／rgb 小遠
    assert [a.area_id for a in inputs.areas] == ["Z1", "Z2", "Z3"]
    assert [a.sensor_req for a in inputs.areas] == ["lidar", "both", "rgb"]
    p1, p2, p3 = (_enu_poly(frame, a, 100.0) for a in inputs.areas)
    assert p1.is_valid and p2.is_valid and p3.is_valid
    assert _is_convex(p1) and p1.area > 8e6
    assert not _is_convex(p2) and len(inputs.areas[1].holes_wgs84) == 1 and len(p2.interiors) == 1
    assert p3.area == pytest.approx(800.0 * 600.0, rel=1e-3)
    assert p3.centroid.distance(shapely.Point(0.0, 0.0)) > 12_000.0
    assert p1.area > p2.area > p3.area
    assert inputs.areas[1].time_window_s == (0.0, 7200.0)
    for a in inputs.areas:
        assert a.payload_req_id in inputs.payload_reqs
    assert inputs.payload_reqs["both_survey"].kind == "lidar" and inputs.payload_reqs["both_survey_rgb"].kind == "rgb"
    assert inputs.payload_reqs["rgb_far"].kind == "rgb" and inputs.payload_reqs["lidar_survey"].kind == "lidar"
    # 障礙：2 塔、1 輸電線、1 建物；1 禁飛
    cats = sorted((o.geom_type, o.category) for o in inputs.obstacles)
    assert cats == [("line", "powerline"), ("point", "tower"), ("point", "tower"), ("polygon", "building")]
    for o in inputs.obstacles:
        assert isinstance(o, Obstacle) and o.height_agl_m is not None and o.top_msl_m is None
    assert len(inputs.airspace.no_fly) == 1 and isinstance(inputs.airspace.no_fly[0], NoFlyZone)
    nf = inputs.airspace.no_fly[0]
    nf_poly = shapely.Polygon(frame.to_enu_xy([p[0] for p in nf.polygon_wgs84], [p[1] for p in nf.polygon_wgs84], 100.0))
    line_h_z3 = shapely.LineString([(0.0, 0.0), tuple(p3.centroid.coords[0])])
    assert nf_poly.intersects(line_h_z3)                     # 禁飛區在 home→Z3 直線上（迫使繞行）
    assert not any(nf_poly.intersects(p) for p in (p1, p2, p3))
    # 風：兩層 4@100–300、7@300–600，皆 from 240°
    assert [(w.z_bottom_m, w.z_top_m, w.speed_mps, w.dir_from_deg, w.z_ref) for w in inputs.wind] == [
        (100.0, 300.0, 4.0, 240.0, "agl"), (300.0, 600.0, 7.0, 240.0, "agl")]
    assert inputs.weather_obs.mean_wind_mps == 4.0 and inputs.weather_obs.wind_dir_from_deg == 240.0
    # 站點：home + 2 備降
    assert [s.site_id for s in inputs.alternates] == ["ALT1", "ALT2"]
    assert _enu_xy(frame, inputs.alternates[0], 100.0) == pytest.approx((-1500.0, 3000.0), abs=0.05)
    assert _enu_xy(frame, inputs.alternates[1], 100.0) == pytest.approx((9000.0, -3000.0), abs=0.05)
    # C2 30 km、空域 120／200、淨空 36
    assert inputs.c2.range_km_for("A", PLATFORM_PIONEER) == 30.0 and inputs.c2.range_km_for("B", PLATFORM_HERO) == 30.0
    assert (inputs.airspace.max_agl_m, inputs.airspace.waiver_max_agl_m, inputs.airspace.terrain_clearance_m) == (120.0, 200.0, 36.0)
    assert inputs.airspace.time_window_s == (0.0, 10800.0)
    assert inputs.airspace.terrain_clearance_m >= 3.0 * inputs.dem.sigma_z_m + 30.0
    # DEM 涵蓋所有幾何
    dem = inputs.dem
    for a in inputs.areas:
        assert dem.contains([p[0] for p in a.polygon_wgs84], [p[1] for p in a.polygon_wgs84]).all()
    for s in inputs.sites.values():
        assert dem.contains(s.lat_deg, s.lon_deg).all()
    for o in inputs.obstacles:
        assert dem.contains([p[0] for p in o.coords_wgs84], [p[1] for p in o.coords_wgs84]).all()


def test_hills_scenario_terrain(hills):
    inputs, _ = hills
    dem = inputs.dem
    frame = S.scenario_frame(inputs)
    # 山丘存在（起伏 ≥ 2A 之九成）且山脊比 3° 陡
    z = dem.z_msl.astype(np.float64)
    assert np.ptp(z) >= 0.9 * 2.0 * S.HILLS_AMPLITUDE_M
    sl = _dem_slope_deg(dem, S.HILLS_DEM_STEP_M)
    assert sl.max() > 3.0
    # 山脊橫越區域 1：沿區域 1 內橫切線取樣之坡度 > 3°、且脊頂高於周圍山丘
    p1 = _enu_poly(frame, inputs.areas[0], 100.0)
    c = np.array(S.HILLS_RIDGE_ENU["center_xy"])
    az = math.radians(S.HILLS_RIDGE_ENU["azimuth_deg"])
    n = np.array([math.cos(az), -math.sin(az)])
    d = np.linspace(-600.0, 600.0, 241)
    pts = c[None, :] + d[:, None] * n[None, :]
    assert shapely.contains_xy(p1, pts[:, 0], pts[:, 1]).all()
    w = frame.to_wgs84(np.column_stack([pts, np.zeros(len(d))]))
    zz = dem.sample_msl(w[:, 0], w[:, 1])
    slope_deg = np.degrees(np.arctan(np.abs(np.gradient(zz, d[1] - d[0]))))
    assert slope_deg.max() > 3.0
    assert zz.max() - min(zz[0], zz[-1]) > 0.8 * S.HILLS_RIDGE_ENU["height_m"]
    # D12 高度帶預算：區域 1 ⊕ 600 m（≥ d_turn,A 535 m）內起伏 ≤ 60 m → A 於 waiver 200 仍可指派
    p1b = p1.buffer(600.0)
    xs = np.arange(p1b.bounds[0], p1b.bounds[2], 25.0)
    ys = np.arange(p1b.bounds[1], p1b.bounds[3], 25.0)
    X, Y = np.meshgrid(xs, ys)
    grid = np.column_stack([X.ravel(), Y.ravel()])
    inside = shapely.contains_xy(p1b, grid[:, 0], grid[:, 1])
    wg = frame.to_wgs84(np.column_stack([grid[inside], np.zeros(int(inside.sum()))]))
    zg = dem.sample_msl(wg[:, 0], wg[:, 1])
    assert np.isfinite(zg).all()
    assert zg.max() - zg.min() <= 60.0
    # 站點坡度 < 5°（規格 §6.2 步 5）
    for s in inputs.sites.values():
        assert _sample_slope_deg(dem, s.lat_deg, s.lon_deg) < 5.0


def test_hills_scenario_json_roundtrip(hills):
    inputs, _ = hills
    js = inputs.to_json()
    m2 = load_mission(js)
    assert m2.to_dict() == inputs.to_dict()
    assert json.loads(js)["fleet"][0]["uav_id"] == "A1"
    assert m2.fleet[3].spec == PLATFORM_HERO and m2.fleet[0].spec == PLATFORM_PIONEER
    assert np.array_equal(m2.dem.z_msl, inputs.dem.z_msl)
    assert [a.sensor_req for a in m2.areas] == ["lidar", "both", "rgb"]
    assert len(m2.areas[1].holes_wgs84) == 1 and len(m2.airspace.no_fly) == 1 and len(m2.obstacles) == 4


# ---------------------------------------------------------------------------
# build_paper_table2_scenario（規格 §15、§17.1 (7)）
# ---------------------------------------------------------------------------

def test_legacy_spec_consistency():
    spec = S.PLATFORM_LEGACY_FIXED_WING
    assert isinstance(spec, PlatformSpec) and spec.cls == "A"
    assert spec.cruise_airspeed_mps == 230.0 and spec.v_min_mission() == 230.0
    assert spec.turn_radius_theoretical_m == 4000.0 and spec.turn_radius_planning_m == 4000.0
    assert spec.r_eff(0.0) == 4000.0 and spec.w_star() == pytest.approx(0.0, abs=1e-9)
    assert spec.max_bank_angle_deg == pytest.approx(53.43, abs=0.01)
    assert spec.airspeed_sensor_validated and spec.battery_monitor_validated and spec.provisional == frozenset()
    assert spec.transition_cmd().slope_tr_rad <= math.radians(spec.gamma_tr_max_deg)
    assert spec.stall_margin_ok(spec.reference_auw_kg)
    # 自訂 V/R 亦自洽
    s2 = S.legacy_fixed_wing_spec(v_mps=100.0, r_min_m=1000.0)
    assert s2.r_eff(0.0) == 1000.0 and s2.cruise_airspeed_kmh == 360.0


def test_paper_table2_scenario(paper):
    inputs, cfg = paper
    # legacy 一鍵退化（規格 §15）
    assert cfg.legacy_mode and cfg.idp_branching == "lockstep" and cfg.transfer_types == ("rev",)
    assert cfg.B == 1 and cfg.K >= 2**31 - 1 and cfg.metric == "paper" and cfg.convexify_whole and cfg.fov_model == "trapezoid"
    assert cfg.e_ext_override_m == 0.0 and cfg.k_bm == 1.0 and cfg.k_as == 1.0
    assert not cfg.enforce_energy and not cfg.enforce_link and not cfg.enforce_time_windows and not cfg.t_rtb_lower_bound
    assert math.isinf(cfg.h_ceil_override_m)
    # 3 機、同一 legacy 規格、梯形 FOV
    assert [u.uav_id for u in inputs.fleet] == ["U1", "U2", "U3"]
    for u in inputs.fleet:
        assert u.spec == S.PLATFORM_LEGACY_FIXED_WING
        assert u.sensors == (TrapezoidFovSpec("lidar", 72.7, 45.0, 76.98),)
    sm = sensor_model(inputs.fleet[0].sensors[0], inputs.payload_reqs["paper"])
    assert sm.swath_m(1200.0) == pytest.approx(2200.1, abs=0.1)         # 規格 §4.3 測試值
    assert sm.footprint_len_m(1200.0) == pytest.approx(999.8, abs=0.1)
    assert sm.strip_spacing_m(1200.0) == pytest.approx(2200.1, abs=0.1)   # 論文條帶間距 = w（side_overlap 0）
    assert inputs.payload_reqs["paper"].h_target_agl_m == 1200.0
    # 站點 = Table 2 三機位置（home = U1）
    assert (inputs.home.lat_deg, inputs.home.lon_deg) == (36.895344, 93.728342)
    assert [s.site_id for s in inputs.alternates] == ["U2", "U3"]
    assert (inputs.alternates[1].lat_deg, inputs.alternates[1].lon_deg) == (36.411013, 93.604920)
    assert set(S.PAPER_TABLE2_UAV_START) == {"U1", "U2", "U3"}
    assert all(v[3] == 30.0 for v in S.PAPER_TABLE2_UAV_START.values())
    # 三機各自其地面站起飛（論文 §4.1）；回收點仍為 home = U1（D40）
    assert inputs.start_sites == {"U1": "U1", "U2": "U2", "U3": "U3"}
    for u in inputs.fleet:
        assert u.start_site_id == u.uav_id
        st = inputs.start_site(u.uav_id)
        assert (st.lat_deg, st.lon_deg) == S.PAPER_TABLE2_UAV_START[u.uav_id][:2]
    assert inputs.recovery_site.site_id == "U1"
    # Z1 六頂點、面積 ≈ 2604.34 km²（ENU 切平面投影，1% 內）
    z1 = inputs.areas[0]
    assert len(z1.polygon_wgs84) == 6 and z1.sensor_req == "any" and z1.payload_req_id == "paper"
    assert z1.polygon_wgs84[0] == (37.071295, 94.035474)
    frame = S.scenario_frame(inputs)
    poly = _enu_poly(frame, z1, S._PAPER_BASE_MSL_M)
    assert poly.is_valid and poly.area / 1e6 == pytest.approx(2604.34, rel=0.01)
    # 平地 DEM 2700 m、涵蓋任務圖 [93.5°,36.5°]–[94.9°,37.5°] 與所有頂點／站點
    dem = inputs.dem
    assert np.all(dem.z_msl == np.float32(2700.0))
    assert dem.contains([36.5, 37.5], [93.5, 94.9]).all()
    assert dem.contains([p[0] for p in z1.polygon_wgs84], [p[1] for p in z1.polygon_wgs84]).all()
    for s in inputs.sites.values():
        assert dem.contains(s.lat_deg, s.lon_deg).all()
    assert frame.h0_msl_m == pytest.approx(2700.0, abs=1e-6)
    # 無風、鏈路／空域不設限
    assert all(w.speed_mps == 0.0 for w in inputs.wind) and inputs.weather_obs.mean_wind_mps == 0.0
    assert inputs.c2.range_km_for("A", inputs.fleet[0].spec) == 1000.0
    assert inputs.airspace.terrain_clearance_m == 30.0
    # JSON 往返（含 legacy PlatformSpec 與 TrapezoidFovSpec）
    m2 = load_mission(inputs.to_json())
    assert m2.to_dict() == inputs.to_dict()
    assert m2.fleet[0].spec == S.PLATFORM_LEGACY_FIXED_WING and isinstance(m2.fleet[0].sensors[0], TrapezoidFovSpec)


# ---------------------------------------------------------------------------
# 輔助函式
# ---------------------------------------------------------------------------

def test_helpers_enu_wgs84():
    frame = GeoFrame(23.95, 120.90, 100.0)
    ring = S.enu_ring_to_wgs84(frame, [(0.0, 0.0), (1000.0, 0.0), (1000.0, 500.0)])
    assert len(ring) == 3 and ring[0] == pytest.approx((23.95, 120.90), abs=1e-9)
    back = frame.to_enu_xy([p[0] for p in ring], [p[1] for p in ring], 100.0)
    assert back == pytest.approx(np.array([[0.0, 0.0], [1000.0, 0.0], [1000.0, 500.0]]), abs=1e-3)
    site = S.site_from_enu(frame, "X", 300.0, -400.0, approach_heading_deg=90.0)
    assert site.kind == "alternate" and site.approach_heading_deg == 90.0
    assert frame.to_enu_xy(site.lat_deg, site.lon_deg, 100.0)[0] == pytest.approx((300.0, -400.0), abs=1e-3)
    m_lat, m_lon = S.metres_per_degree(0.0)
    assert m_lat == pytest.approx(110574.0, abs=5.0) and m_lon == pytest.approx(111320.0, abs=5.0)
    assert S.metres_per_degree(60.0)[1] == pytest.approx(55800.0, abs=50.0)


def test_scenario_frame_uses_site_altitude(mini):
    inputs, _ = mini
    home_msl = Site("H", 23.95, 120.90, alt_msl_m=123.0, kind="home")
    m = MissionInputs(home=home_msl, alternates=inputs.alternates, areas=inputs.areas, dem=inputs.dem,
                      obstacles=(), wind=inputs.wind, fleet=inputs.fleet, payload_reqs=inputs.payload_reqs,
                      c2=inputs.c2, airspace=inputs.airspace, weather_thresholds=inputs.weather_thresholds,
                      weather_obs=inputs.weather_obs)
    assert S.scenario_frame(m).h0_msl_m == 123.0
    home_agl = Site("H", 23.95, 120.90, alt_agl_m=5.0, kind="home")
    m2 = MissionInputs(home=home_agl, alternates=inputs.alternates, areas=inputs.areas, dem=inputs.dem,
                       obstacles=(), wind=inputs.wind, fleet=inputs.fleet, payload_reqs=inputs.payload_reqs,
                       c2=inputs.c2, airspace=inputs.airspace, weather_thresholds=inputs.weather_thresholds,
                       weather_obs=inputs.weather_obs)
    assert S.scenario_frame(m2).h0_msl_m == pytest.approx(105.0, abs=1e-6)
    far = Site("H", 30.0, 130.0, kind="home")
    m3 = MissionInputs(home=far, alternates=inputs.alternates, areas=inputs.areas, dem=inputs.dem,
                       obstacles=(), wind=inputs.wind, fleet=inputs.fleet, payload_reqs=inputs.payload_reqs,
                       c2=inputs.c2, airspace=inputs.airspace, weather_thresholds=inputs.weather_thresholds,
                       weather_obs=inputs.weather_obs)
    with pytest.raises(MissionInputError):
        S.scenario_frame(m3)


# ---------------------------------------------------------------------------
# 下游有效性：三個情境皆能經 context.build_context 組裝（keep-out／感測器／能量模型／站點）
# ---------------------------------------------------------------------------

def test_scenarios_build_context(mini, hills, paper):
    context = pytest.importorskip("hv_dccpp.context")
    from dataclasses import replace

    # mini（契約 §W.1/§W.8）：無障礙、h_clr 36、h_ceil 200（waiver）、E_use A 779.6／B 214.3 Wh
    inputs, cfg = mini
    ctx = context.build_context(inputs, replace(cfg, dem_step_m=50.0), terrain_margin_m=800.0)
    assert ctx.h_clr_m == 36.0 and ctx.h_ceil_m == {"A": 200.0, "B": 200.0}
    assert set(ctx.sensors) == {("A", "default"), ("B", "default_rgb")}
    assert ctx.sites_enu["ALT1"][0] == pytest.approx((2000.0, 1500.0), abs=0.1)
    assert ctx.models["A1"].e_usable_wh() == pytest.approx(779.6, abs=0.5)
    assert ctx.models["B1"].e_usable_wh() == pytest.approx(214.3, abs=0.5)
    assert ctx.keepouts is None or ctx.keepouts.ids == []
    assert ctx.rho_sym_m == {"A": 66000.0, "B": 39000.0}

    # hills：5 個 keep-out（2 塔、輸電線、建物、禁飛）、both 區之 lidar／rgb 需求各自建立感測器、8 機皆有可規劃能量
    inputs, cfg = hills
    ctx = context.build_context(inputs, replace(cfg, dem_step_m=50.0), terrain_margin_m=800.0)
    if ctx.keepouts is not None:
        assert set(ctx.keepouts.ids) == {"T1", "T2", "PL1", "BLD1", "NF1"}
    assert {("A", "lidar_survey"), ("A", "both_survey"), ("B", "both_survey_rgb"), ("B", "rgb_far")} <= set(ctx.sensors)
    assert set(ctx.models) == {"A1", "A2", "A3", "B1", "B2", "B3", "B4", "B5"}
    assert all(m.e_usable_wh() > 0.0 for m in ctx.models.values())
    assert set(ctx.sites_enu) == {"H", "ALT1", "ALT2"}
    z1 = ctx.areas_enu["Z1"]
    assert ctx.terrain.max_in_polygon(z1) - ctx.terrain.min_in_polygon(z1) <= 60.0      # D12 高度帶預算
    assert ctx.terrain.max_in_polygon(z1) - ctx.terrain.min_in_polygon(z1) >= 40.0      # 山脊確實在區域 1 內
    assert ctx.h_ceil_m == {"A": 200.0, "B": 200.0}

    # paper（legacy）：h_ceil = ∞、鏈路不設限、梯形 FOV w ≈ 2200.1
    inputs, cfg = paper
    ctx = context.build_context(inputs, cfg, terrain_margin_m=5000.0)
    assert math.isinf(ctx.h_ceil_m["A"]) and math.isinf(ctx.rho_sym_m["A"])
    assert set(ctx.sensors) == {("A", "paper")}
    assert ctx.sensors[("A", "paper")].swath_m(1200.0) == pytest.approx(2200.1, abs=0.1)
    assert set(ctx.sites_enu) == {"U1", "U2", "U3"}
    assert ctx.areas_enu["Z1"].area / 1e6 == pytest.approx(2604.34, rel=0.01)


def test_paper_table2_start_sites_resolved_in_context(paper):
    """build_context 將三機起飛站點解析為各自地面站之 ENU（U1 為原點）。"""
    from hv_dccpp.context import build_context
    inputs, cfg = paper
    ctx = build_context(inputs, cfg, terrain_margin_m=5000.0)
    assert ctx.start_sites == {"U1": "U1", "U2": "U2", "U3": "U3"}
    assert np.allclose(ctx.start_xy("U1"), 0.0, atol=1e-6)            # home = U1
    for uid in ("U2", "U3"):
        assert np.allclose(ctx.start_xy(uid), ctx.sites_enu[uid][0])
        assert np.hypot(*(ctx.start_xy(uid) - ctx.home_xy)) > 20_000.0   # 各站相距 > 20 km
    assert any(w.startswith("per_uav_start_sites") for w in ctx.warnings)
    assert ctx.recovery_site_id == "U1"


def test_mini_and_hills_scenarios_keep_home_start():
    """既有情境未設 start_site_id → 起飛站點仍為 home（回歸保護）。"""
    for build in (S.build_mini_example, S.build_hills_scenario):
        inputs, _cfg = build()
        assert all(u.start_site_id is None for u in inputs.fleet)
        assert set(inputs.start_sites.values()) == {inputs.home.site_id}
