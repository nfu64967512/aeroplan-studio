"""tests/test_geo.py — 契約 §1 不變量與規格 §17.2 之 test_geo 項目。

enu_roundtrip_mm；curvature_drop_50km(≈196 m)；bilinear_matches_eq4；corridor_max_terrain；enu_z_msl_roundtrip
＋ burn 單調、多邊形 max/min、坡度、DEM 高程基準。
"""
from __future__ import annotations

import math

import numpy as np
import pytest
import shapely

from hv_dccpp.geo import (
    WGS84_A,
    WGS84_E2,
    DemGrid,
    GeoFrame,
    TerrainGrid,
    ecef_to_geodetic,
    geodetic_to_ecef,
)

LAT0, LON0, H0 = 23.95, 120.90, 100.0
PHI0 = math.radians(LAT0)
N0 = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(PHI0) ** 2)                 # 卯酉圈曲率半徑
M0 = WGS84_A * (1.0 - WGS84_E2) / (1.0 - WGS84_E2 * math.sin(PHI0) ** 2) ** 1.5  # 子午圈曲率半徑
DEG_PER_M_LAT = math.degrees(1.0 / M0)
DEG_PER_M_LON = math.degrees(1.0 / (N0 * math.cos(PHI0)))


@pytest.fixture
def frame() -> GeoFrame:
    return GeoFrame(LAT0, LON0, H0)


def make_dem(step_m: float = 10.0, half_size_m: float = 1500.0, fn=None, **kw) -> DemGrid:
    """以 home 為中心之經緯規則 DEM；fn(lat, lon) → MSL（預設平地 100 m）。"""
    n_lat = int(2 * half_size_m / step_m) + 1
    n_lon = n_lat
    dlat = step_m * DEG_PER_M_LAT
    dlon = step_m * DEG_PER_M_LON
    lat0 = LAT0 - (n_lat // 2) * dlat
    lon0 = LON0 - (n_lon // 2) * dlon
    lat = lat0 + np.arange(n_lat) * dlat
    lon = lon0 + np.arange(n_lon) * dlon
    glon, glat = np.meshgrid(lon, lat)
    z = np.full(glat.shape, 100.0) if fn is None else fn(glat, glon)
    return DemGrid(lat0, lon0, dlat, dlon, z.astype(np.float32), **kw)


# ---------------------------------------------------------------------------
# ECEF / 大地座標
# ---------------------------------------------------------------------------
def test_ecef_geodetic_roundtrip_mm():
    rng = np.random.default_rng(1)
    lat = rng.uniform(-89.9, 89.9, 2000)
    lon = rng.uniform(-180.0, 180.0, 2000)
    h = rng.uniform(-500.0, 9000.0, 2000)
    # 加入極點與赤道特例
    lat = np.concatenate([lat, [90.0, -90.0, 0.0, 0.0]])
    lon = np.concatenate([lon, [0.0, 45.0, 0.0, 180.0]])
    h = np.concatenate([h, [0.0, 100.0, 0.0, 5.0]])
    g = ecef_to_geodetic(geodetic_to_ecef(lat, lon, h))
    d_lat_m = np.abs(g[:, 0] - lat) * math.pi / 180 * WGS84_A
    # 經度差在極點無意義（cos φ = 0）
    d_lon_m = np.abs((g[:, 1] - lon + 180.0) % 360.0 - 180.0) * math.pi / 180 * WGS84_A * np.cos(np.radians(lat))
    assert d_lat_m.max() < 1e-3
    assert d_lon_m.max() < 1e-3
    assert np.abs(g[:, 2] - h).max() < 1e-3


def test_geodetic_to_ecef_reference_point():
    # 赤道、本初子午線、h=0 → (a, 0, 0)；北極 → (0, 0, b)
    p = geodetic_to_ecef([0.0, 90.0], [0.0, 0.0], [0.0, 0.0])
    assert np.allclose(p[0], [WGS84_A, 0.0, 0.0], atol=1e-6)
    assert np.allclose(p[1], [0.0, 0.0, WGS84_A * math.sqrt(1.0 - WGS84_E2)], atol=1e-6)


# ---------------------------------------------------------------------------
# GeoFrame
# ---------------------------------------------------------------------------
def test_rot_orthonormal_and_up_is_normal(frame: GeoFrame):
    assert np.allclose(frame.rot @ frame.rot.T, np.eye(3), atol=1e-12)
    lam = math.radians(LON0)
    up = np.array([math.cos(PHI0) * math.cos(lam), math.cos(PHI0) * math.sin(lam), math.sin(PHI0)])
    assert np.allclose(frame.rot[2], up)
    assert frame.origin_ecef.shape == (3,)
    assert np.allclose(frame.to_enu(LAT0, LON0, H0), np.zeros((1, 3)), atol=1e-9)


def test_enu_roundtrip_mm(frame: GeoFrame):
    rng = np.random.default_rng(2)
    lat = LAT0 + rng.uniform(-0.6, 0.6, 3000)
    lon = LON0 + rng.uniform(-0.6, 0.6, 3000)
    h = rng.uniform(-100.0, 5000.0, 3000)
    enu = frame.to_enu(lat, lon, h)
    assert enu.shape == (3000, 3)
    back = frame.to_wgs84(enu)
    assert np.abs(back[:, 0] - lat).max() * math.pi / 180 * WGS84_A < 1e-3
    assert np.abs(back[:, 1] - lon).max() * math.pi / 180 * WGS84_A * math.cos(PHI0) < 1e-3
    assert np.abs(back[:, 2] - h).max() < 1e-3
    # (3,) 輸入亦接受，回傳 (1,3)
    assert frame.to_wgs84(enu[0]).shape == (1, 3)


def test_curvature_drop_50km(frame: GeoFrame):
    """50 km 處平地（MSL = h0）之 ENU z ≈ −196 m（曲率下垂；A9、契約 §1）。"""
    p_east = frame.to_enu(LAT0, LON0 + 50e3 * DEG_PER_M_LON, H0)[0]
    p_north = frame.to_enu(LAT0 + 50e3 * DEG_PER_M_LAT, LON0, H0)[0]
    for p, radius in ((p_east, N0), (p_north, M0)):
        d = math.hypot(p[0], p[1])
        assert abs(d - 50e3) < 100.0
        assert abs(p[2] + 196.0) < 1.5
        assert abs(p[2] + d * d / (2.0 * radius)) < 0.3      # 一階下垂 d²/(2R)
    assert abs(p_east[0] - 50e3) < 5.0 and abs(p_east[1]) < 100.0
    assert abs(p_north[1] - 50e3) < 5.0 and abs(p_north[0]) < 1e-6
    # msl_to_enu_z 給出同一數值
    z = frame.msl_to_enu_z(np.array([[50e3, 0.0]]), H0)
    assert abs(z[0] + 196.0) < 1.5


def test_enu_z_msl_roundtrip(frame: GeoFrame):
    rng = np.random.default_rng(3)
    xy = rng.uniform(-50e3, 50e3, (2000, 2))
    z = rng.uniform(-300.0, 3000.0, 2000)
    msl = frame.enu_z_to_msl(xy, z)
    assert msl.shape == (2000,)
    z2 = frame.msl_to_enu_z(xy, msl)
    assert np.abs(z2 - z).max() < 1e-3
    # 反向：MSL → ENU → MSL
    h = rng.uniform(0.0, 3000.0, 2000)
    assert np.abs(frame.enu_z_to_msl(xy, frame.msl_to_enu_z(xy, h)) - h).max() < 1e-3
    # 原點：MSL = h0 ↔ z = 0；純量廣播
    assert abs(frame.msl_to_enu_z(np.array([[0.0, 0.0]]), H0)[0]) < 1e-9
    assert abs(frame.enu_z_to_msl(np.array([0.0, 0.0]), 0.0)[0] - H0) < 1e-9
    # 差值 = h0 + 曲率項（非常數）
    diff = msl - z
    assert abs(diff.min() - H0) < 1.0 and diff.max() > H0 + 100.0
    # nan 原樣傳回
    assert np.isnan(frame.msl_to_enu_z(np.array([[1.0, 2.0]]), np.nan)[0])


def test_to_enu_xy_matches_to_enu(frame: GeoFrame):
    rng = np.random.default_rng(4)
    lat = LAT0 + rng.uniform(-0.2, 0.2, 100)
    lon = LON0 + rng.uniform(-0.2, 0.2, 100)
    xy = frame.to_enu_xy(lat, lon)
    assert xy.shape == (100, 2)
    assert np.allclose(xy, frame.to_enu(lat, lon, 0.0)[:, :2])


def test_geoid_undulation_shifts_ellipsoid_height_only():
    f0 = GeoFrame(LAT0, LON0, H0, geoid_undulation_m=0.0)
    f1 = GeoFrame(LAT0, LON0, H0, geoid_undulation_m=25.0)
    assert abs(f1.h0_ell_m - (H0 + 25.0)) < 1e-12
    lat, lon = LAT0 + 0.1, LON0 + 0.1
    # 同一 MSL 點在兩框架之 ENU 幾乎相同（僅 25 m 橢球高造成之 ~1e-5 比例差）
    e0 = f0.to_enu(lat, lon, 300.0)[0]
    e1 = f1.to_enu(lat, lon, 300.0)[0]
    assert np.abs(e0 - e1).max() < 0.1
    assert np.abs(f1.to_wgs84(e1)[0] - [lat, lon, 300.0]).max() < 1e-6


# ---------------------------------------------------------------------------
# DemGrid
# ---------------------------------------------------------------------------
def test_dem_grid_validation():
    with pytest.raises(ValueError):
        DemGrid(0.0, 0.0, 1e-4, 1e-4, np.zeros((1, 5), np.float32))
    with pytest.raises(ValueError):
        DemGrid(0.0, 0.0, -1e-4, 1e-4, np.zeros((5, 5), np.float32))
    with pytest.raises(ValueError):
        DemGrid(0.0, 0.0, 1e-4, 1e-4, np.zeros((5, 5)), vertical_datum="agl")  # type: ignore[arg-type]
    dem = DemGrid(0.0, 0.0, 1e-4, 1e-4, np.zeros((5, 7)))
    assert dem.z_msl.dtype == np.float32 and dem.rows == 5 and dem.cols == 7
    assert dem.bounds_wgs84() == pytest.approx((0.0, 0.0, 4e-4, 6e-4), abs=1e-12)


def test_dem_sample_bilinear_plane_and_nan():
    dem = make_dem(fn=lambda lat, lon: 100.0 + 3000.0 * (lat - LAT0) + 2000.0 * (lon - LON0))
    rng = np.random.default_rng(5)
    lat = rng.uniform(dem.lat0_deg, dem.lat_max_deg, 500)
    lon = rng.uniform(dem.lon0_deg, dem.lon_max_deg, 500)
    expect = 100.0 + 3000.0 * (lat - LAT0) + 2000.0 * (lon - LON0)
    assert np.abs(dem.sample_msl(lat, lon) - expect).max() < 1e-3
    assert np.isnan(dem.sample_msl(dem.lat0_deg - 1e-3, LON0)[0])
    assert np.isnan(dem.sample_msl(LAT0, dem.lon_max_deg + 1e-3)[0])
    # 節點值精確
    assert dem.sample_msl(dem.lat0_deg, dem.lon0_deg)[0] == pytest.approx(float(dem.z_msl[0, 0]), abs=1e-6)


def test_dem_ellipsoid_datum(frame: GeoFrame):
    undulation = 20.0
    f_geoid = GeoFrame(LAT0, LON0, H0, geoid_undulation_m=undulation)
    dem_msl = make_dem(fn=lambda lat, lon: 100.0 + 50.0 * np.sin((lon - LON0) * 300.0))
    dem_ell = DemGrid(dem_msl.lat0_deg, dem_msl.lon0_deg, dem_msl.dlat_deg, dem_msl.dlon_deg,
                      dem_msl.z_msl + undulation, vertical_datum="ellipsoid", geoid_undulation_m=undulation)
    assert np.allclose(dem_ell.sample_msl(LAT0, LON0), dem_msl.sample_msl(LAT0, LON0), atol=1e-4)
    b = (-500.0, -500.0, 500.0, 500.0)
    t_msl = TerrainGrid.from_dem(dem_msl, f_geoid, 10.0, b)
    t_ell = TerrainGrid.from_dem(dem_ell, f_geoid, 10.0, b)
    assert np.abs(t_msl.z - t_ell.z).max() < 1e-3


# ---------------------------------------------------------------------------
# TerrainGrid：雙線性 / from_dem
# ---------------------------------------------------------------------------
def test_bilinear_matches_eq4_plane_exact():
    """論文 Eq.(4)：對平面 z = a + b·x + c·y 雙線性內插精確；節點值精確；胞心 = 四角平均。"""
    step = 10.0
    xs = -200.0 + np.arange(41) * step
    ys = -100.0 + np.arange(21) * step
    gx, gy = np.meshgrid(xs, ys)
    plane = lambda x, y: 5.0 + 0.01 * x - 0.02 * y  # noqa: E731
    tg = TerrainGrid(-200.0, -100.0, step, plane(gx, gy))
    assert tg.nx == 41 and tg.ny == 21 and tg.bounds() == (-200.0, -100.0, 200.0, 100.0)
    rng = np.random.default_rng(6)
    xy = np.column_stack([rng.uniform(-200, 200, 1000), rng.uniform(-100, 100, 1000)])
    assert np.abs(tg.z_at(xy) - plane(xy[:, 0], xy[:, 1])).max() < 1e-4
    # 節點與邊界
    nodes = np.column_stack([gx.ravel(), gy.ravel()])
    assert np.abs(tg.z_at(nodes) - tg.z.ravel()).max() < 1e-6
    assert np.abs(tg.z_at(np.array([[200.0, 100.0], [-200.0, -100.0]])) - plane(np.array([200.0, -200.0]), np.array([100.0, -100.0]))).max() < 1e-5
    # 非線性網格：胞心 = 四角平均（Eq.4 之權重）
    z_rand = rng.uniform(0, 100, (21, 41))
    tg2 = TerrainGrid(-200.0, -100.0, step, z_rand)
    v = tg2.z_at(np.array([[-195.0, -95.0]]))[0]
    assert v == pytest.approx(float(z_rand[0:2, 0:2].astype(np.float32).mean()), abs=1e-4)
    # 範圍外 nan
    out = tg.z_at(np.array([[-200.1, 0.0], [0.0, 100.1], [np.nan, 0.0]]))
    assert np.isnan(out).all()


def test_from_dem_flat_includes_curvature(frame: GeoFrame):
    dem = make_dem(sigma_z_m=2.0)
    tg = TerrainGrid.from_dem(dem, frame, 10.0, (-1000.0, -1000.0, 1000.0, 1000.0))
    assert (tg.nx, tg.ny) == (201, 201)
    assert tg.bounds() == (-1000.0, -1000.0, 1000.0, 1000.0)
    assert tg.z.dtype == np.float32 and tg.z_raw.dtype == np.float32
    assert tg.sigma_z_m == 2.0 and tg.is_dsm is False
    assert not np.isnan(tg.z).any()
    xy = np.array([[0.0, 0.0], [1000.0, 0.0], [-1000.0, -1000.0], [333.3, -777.7]])
    z = tg.z_at(xy)
    assert abs(z[0]) < 1e-4                                            # home 地表 = z 0
    assert z[1] < -0.07 and z[1] > -0.09                               # 1 km：d²/(2R) ≈ 0.078 m
    assert np.abs(z - frame.msl_to_enu_z(xy, 100.0)).max() < 1e-3      # 節點以 MSL→ENU 精確轉換
    assert np.abs(frame.enu_z_to_msl(xy, z) - 100.0).max() < 1e-3     # 反向得回 DEM 之 MSL


def test_from_dem_plane_matches_eq4_end_to_end(frame: GeoFrame):
    """DEM 為 lat/lon 平面 → 任意 ENU 點：enu_z_to_msl(xy, z_at(xy)) 等於該點 lat/lon 之平面值。"""
    a, b = 4000.0, -2500.0
    plane = lambda lat, lon: 150.0 + a * (lat - LAT0) + b * (lon - LON0)  # noqa: E731
    dem = make_dem(step_m=10.0, half_size_m=1500.0, fn=plane)
    tg = TerrainGrid.from_dem(dem, frame, 10.0, (-1200.0, -1200.0, 1200.0, 1200.0))
    assert not np.isnan(tg.z).any()
    rng = np.random.default_rng(7)
    xy = rng.uniform(-1200.0, 1200.0, (2000, 2))
    z = tg.z_at(xy)
    wgs = frame.to_wgs84(np.column_stack([xy, z]))
    assert np.abs(wgs[:, 2] - plane(wgs[:, 0], wgs[:, 1])).max() < 1e-3


def test_from_dem_outside_dem_is_nan(frame: GeoFrame):
    dem = make_dem(step_m=50.0, half_size_m=500.0)
    tg = TerrainGrid.from_dem(dem, frame, 50.0, (-800.0, -800.0, 800.0, 800.0))
    assert np.isnan(tg.z_at(np.array([[700.0, 700.0]]))[0])
    assert not np.isnan(tg.z_at(np.array([[0.0, 0.0]]))[0])
    with pytest.raises(ValueError):
        TerrainGrid.from_dem(dem, frame, 50.0, (0.0, 0.0, -1.0, 1.0))


# ---------------------------------------------------------------------------
# 走廊
# ---------------------------------------------------------------------------
def ridge_grid(y_ridge: float, height: float = 80.0, sigma: float = 8.0) -> TerrainGrid:
    step = 5.0
    xs = -600.0 + np.arange(241) * step
    ys = -300.0 + np.arange(121) * step
    gx, gy = np.meshgrid(xs, ys)
    z = 10.0 + height * np.exp(-0.5 * ((gy - y_ridge) / sigma) ** 2)
    return TerrainGrid(-600.0, -300.0, step, z)


def test_corridor_max_zero_hw_equals_z_at():
    tg = ridge_grid(-30.0)
    rng = np.random.default_rng(8)
    path = np.column_stack([rng.uniform(-500, 500, 300), rng.uniform(-200, 200, 300)])
    theta = rng.uniform(-math.pi, math.pi, 300)
    np.testing.assert_array_equal(tg.corridor_max(path, theta, 0.0, 5.0), tg.z_at(path))
    np.testing.assert_array_equal(tg.corridor_min(path, theta, 0.0, 5.0), tg.z_at(path))
    np.testing.assert_array_equal(tg.corridor_max(path, theta, np.zeros(300), 5.0), tg.z_at(path))
    # 純量 theta 廣播
    assert tg.corridor_max(path, 0.3, 0.0, 5.0).shape == (300,)


def test_corridor_max_terrain():
    """右法向 n̂(θ) = (cos θ, −sin θ)：θ = 90°（向東）時 v 增加 = 向南（契約 §0）。"""
    tg = ridge_grid(-30.0)            # 山脊在 y = −30（航跡南側）
    path = np.column_stack([np.linspace(-500.0, 500.0, 41), np.zeros(41)])
    theta = np.full(41, math.pi / 2)
    base = tg.z_at(path)
    wide = tg.corridor_max(path, theta, 50.0, 5.0)         # v ∈ [−50, 50] step 5 含 v=30
    narrow = tg.corridor_max(path, theta, 10.0, 5.0)
    assert np.allclose(wide, 90.0, atol=0.05)              # 山脊頂 10 + 80
    assert np.all(narrow < 30.0) and np.all(narrow >= base)
    assert np.all(wide >= narrow)
    # 走廊最低點 = 遠離山脊側之平地
    assert np.all(tg.corridor_min(path, theta, 50.0, 5.0) <= base + 1e-6)
    assert np.allclose(tg.corridor_min(path, theta, 50.0, 5.0), 10.0, atol=0.05)
    # offset：hw=0、offset=+30 → 正是 path + 30·n̂ = (x, −30) 山脊頂；offset=−30 → (x, +30) 平地
    n_hat = np.array([math.cos(math.pi / 2), -math.sin(math.pi / 2)])
    np.testing.assert_array_equal(tg.corridor_max(path, theta, 0.0, 5.0, offset_m=30.0), tg.z_at(path + 30.0 * n_hat))
    assert np.allclose(tg.corridor_max(path, theta, 0.0, 5.0, offset_m=30.0), 90.0, atol=1e-4)
    assert np.allclose(tg.corridor_max(path, theta, 0.0, 5.0, offset_m=-30.0), 10.0, atol=0.05)
    # 反向航跡（θ = 270°）：n̂ 指北，山脊在 v = −30，仍在對稱走廊內
    assert np.allclose(tg.corridor_max(path, np.full(41, 1.5 * math.pi), 50.0, 5.0), 90.0, atol=0.05)
    # 走廊偏一側：offset=−30, hw=20 → v ∈ [−50, −10]（北側）不含山脊
    assert np.all(tg.corridor_max(path, theta, 20.0, 5.0, offset_m=-30.0) < 15.0)


def test_corridor_half_width_array_and_spacing():
    tg = ridge_grid(-30.0, sigma=3.0)
    path = np.column_stack([np.linspace(-400.0, 400.0, 17), np.zeros(17)])
    theta = np.full(17, math.pi / 2)
    hw = np.where(np.arange(17) % 2 == 0, 50.0, 0.0)
    got = tg.corridor_max(path, theta, hw, 5.0)
    wide = tg.corridor_max(path, theta, 50.0, 5.0)
    zero = tg.z_at(path)
    np.testing.assert_allclose(got[::2], wide[::2])
    np.testing.assert_array_equal(got[1::2], zero[1::2])
    # 取樣間距 ≤ dv：dv 越小越不會漏掉窄山脊（sigma 3 m）
    coarse = tg.corridor_max(path, theta, 50.0, 40.0)
    fine = tg.corridor_max(path, theta, 50.0, 1.0)
    assert np.all(fine >= coarse - 1e-6) and np.allclose(fine, 90.0, atol=0.2)
    with pytest.raises(ValueError):
        tg.corridor_max(path, theta, 50.0, 0.0)
    with pytest.raises(ValueError):
        tg.corridor_max(path, theta, -1.0, 5.0)


def test_corridor_nan_when_outside_grid():
    tg = ridge_grid(-30.0)
    path = np.array([[0.0, -280.0], [0.0, 0.0]])
    theta = np.full(2, math.pi / 2)          # n̂ 指南 → 第一點走廊 v=+50 → y=−330 出界
    out = tg.corridor_max(path, theta, 50.0, 5.0)
    assert np.isnan(out[0]) and np.isfinite(out[1])


# ---------------------------------------------------------------------------
# 多邊形、燒入、坡度
# ---------------------------------------------------------------------------
def test_max_min_in_polygon_plane():
    step = 10.0
    xs = -300.0 + np.arange(61) * step
    ys = -300.0 + np.arange(61) * step
    gx, gy = np.meshgrid(xs, ys)
    plane = lambda x, y: 20.0 + 0.05 * x + 0.02 * y  # noqa: E731
    tg = TerrainGrid(-300.0, -300.0, step, plane(gx, gy))
    verts = [(-123.4, -50.0), (200.0, -10.0), (77.0, 150.0), (-40.0, 90.0)]
    poly = shapely.Polygon(verts)
    vals = [plane(x, y) for x, y in verts]
    assert tg.max_in_polygon(poly) == pytest.approx(max(vals), abs=1e-3)
    assert tg.min_in_polygon(poly) == pytest.approx(min(vals), abs=1e-3)
    # 小於一格的多邊形仍有值（邊界取樣）
    tiny = shapely.Polygon([(1.0, 1.0), (3.0, 1.0), (3.0, 3.0)])
    assert tg.max_in_polygon(tiny) == pytest.approx(plane(3.0, 3.0), abs=1e-3)
    assert tg.min_in_polygon(tiny) == pytest.approx(plane(1.0, 1.0), abs=1e-3)
    # MultiPolygon 接受；空 → nan；範圍外 → nan
    multi = shapely.MultiPolygon([poly, tiny])
    assert tg.max_in_polygon(multi) == pytest.approx(max(vals), abs=1e-3)
    assert math.isnan(tg.max_in_polygon(shapely.Polygon()))
    assert math.isnan(tg.min_in_polygon(shapely.Polygon([(290, 290), (400, 290), (400, 400)])))


def test_burn_monotone():
    rng = np.random.default_rng(9)
    step = 10.0
    tg = TerrainGrid(-300.0, -300.0, step, rng.uniform(0.0, 30.0, (61, 61)))
    raw_before = tg.z_raw.copy()
    z_before = tg.z.copy()
    poly1 = shapely.Polygon([(-100.0, -100.0), (50.0, -120.0), (60.0, 40.0), (-80.0, 30.0)])
    poly2 = shapely.Point(150.0, 150.0).buffer(40.0)
    tg.burn([poly1, poly2], [80.0, 120.0])
    assert np.all(tg.z >= z_before)
    np.testing.assert_array_equal(tg.z_raw, raw_before)                # z_raw 不變
    # 多邊形內任意點（含邊界附近）之雙線性值 ≥ top
    pts = np.column_stack([rng.uniform(-100, 60, 3000), rng.uniform(-120, 40, 3000)])
    inside = shapely.contains_xy(poly1, pts[:, 0], pts[:, 1])
    assert np.all(tg.z_at(pts[inside]) >= 80.0 - 1e-4)
    ring = shapely.get_coordinates(shapely.segmentize(poly1.exterior, 1.0))
    assert np.all(tg.z_at(ring) >= 80.0 - 1e-4)
    assert tg.max_in_polygon(poly1) >= 80.0 and tg.min_in_polygon(poly1) >= 80.0 - 1e-4
    assert tg.z_at(np.array([[150.0, 150.0]]))[0] >= 120.0 - 1e-4
    # 遠處不受影響
    far = np.array([[-250.0, 250.0], [250.0, -250.0]])
    np.testing.assert_array_equal(tg.z_at(far), _bilinear_ref(z_before, tg, far))
    # 再燒較低之 top → 不變；重複燒入 → 不變（單調不減、冪等）
    z_mid = tg.z.copy()
    tg.burn([poly1], [10.0])
    np.testing.assert_array_equal(tg.z, z_mid)
    tg.burn([poly1, poly2], [80.0, 120.0])
    np.testing.assert_array_equal(tg.z, z_mid)
    # nan / None top 跳過；長度不符拒絕
    tg.burn([poly1], [float("nan")])
    np.testing.assert_array_equal(tg.z, z_mid)
    with pytest.raises(ValueError):
        tg.burn([poly1], [1.0, 2.0])


def _bilinear_ref(grid: np.ndarray, tg: TerrainGrid, xy: np.ndarray) -> np.ndarray:
    return TerrainGrid(tg.x0_m, tg.y0_m, tg.step_m, grid).z_at(xy)


def test_slope_at_plane_and_raw():
    step = 10.0
    xs = -300.0 + np.arange(61) * step
    ys = -300.0 + np.arange(61) * step
    gx, gy = np.meshgrid(xs, ys)
    b, c = 0.03, -0.04
    tg = TerrainGrid(-300.0, -300.0, step, 50.0 + b * gx + c * gy)
    rng = np.random.default_rng(10)
    xy = np.column_stack([rng.uniform(-300, 300, 500), rng.uniform(-300, 300, 500)])
    assert np.allclose(tg.slope_at(xy), math.atan(math.hypot(b, c)), atol=1e-5)
    flat = TerrainGrid(0.0, 0.0, step, np.full((5, 5), 7.0))
    assert np.allclose(flat.slope_at(np.array([[10.0, 10.0], [33.0, 5.0]])), 0.0)
    assert np.isnan(tg.slope_at(np.array([[1000.0, 0.0]]))[0])
    # 燒入不改變預設（z_raw）坡度；raw=False 才看見人工陡壁
    tg.burn([shapely.Point(0.0, 0.0).buffer(30.0)], [500.0])
    assert np.allclose(tg.slope_at(np.array([[0.0, 0.0], [35.0, 0.0]])), math.atan(math.hypot(b, c)), atol=1e-5)
    assert tg.slope_at(np.array([[45.0, 0.0]]), raw=False)[0] > math.radians(60.0)


def test_copy_is_independent():
    tg = TerrainGrid(0.0, 0.0, 10.0, np.zeros((5, 5)))
    cp = tg.copy()
    cp.burn([shapely.Point(20.0, 20.0).buffer(5.0)], [9.0])
    assert tg.z.max() == 0.0 and cp.z.max() == 9.0
