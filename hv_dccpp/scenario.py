"""scenario.py — 情境建構（模組契約 §16）：合成 DEM、§W mini example、附錄「山丘」情境、論文 Table 2 情境。

只依賴第 0／1 層的 `geo`、`platform`、`mission`（契約 §0 依賴表允許 scenario 依賴
context/mission/geo/geometry/platform/planner；本模組實際只需前三者，可在任何時點單獨匯入）。

設計原則
--------
* 所有情境先以 **ENU（原點 = home）** 設計（矩形、頂點、站點皆為公尺），再經 `GeoFrame.to_wgs84`
  轉成 WGS84 `(lat, lon)` 任務輸入（契約 §0：輸入／輸出 WGS84 + MSL；內部 ENU）。
* 合成 DEM（:func:`synthetic_dem`）為 WGS84 經緯規則網格（`DemGrid` 語意：`lat0/lon0` = 左下角節點），
  節點間距由 `step_m` 換算成 `dlat/dlon`；地形 = 平地 + 可選正弦山丘 + 可選山脊（皆以公尺定義）。
* 三個情境：
  - :func:`build_mini_example`：契約 §W.1 逐項（3 km × 1.2 km、A1 h_target 160／B1 h_target 100、西風 5 m/s、示例硬體）。
  - :func:`build_hills_scenario`：規格 §17.2 `runtime_under_1s`／`example_coverage_ge_99p5` 之「8 機／3 區／山脊／
    電塔／輸電線／禁飛／分層風」情境。
  - :func:`build_paper_table2_scenario`：規格 §15／§17.1 (7)：論文 Table 2（3 機、V=230、R=4000、h=1200、
    梯形 FOV 72.7°/45°/76.98°、Z1 六頂點）＋ `PlannerConfig(legacy_mode=True)`。
"""
from __future__ import annotations

import math
from typing import Any, Mapping, Sequence

import numpy as np

from .geo import DemGrid, GeoFrame
from .mission import (
    Airspace,
    AreaSpec,
    C2Link,
    MissionInputError,
    MissionInputs,
    NoFlyZone,
    Obstacle,
    PayloadRequirement,
    PlannerConfig,
    Site,
    SortieEnergy,
    UAVInstance,
    WeatherObs,
    WeatherThresholds,
    WindLayer,
)
from .platform import (
    G_MPS2,
    PLATFORM_HERO,
    PLATFORM_PIONEER,
    CameraHardware,
    LidarHardware,
    PlatformSpec,
    TrapezoidFovSpec,
)

__all__ = [
    # 契約 §16
    "synthetic_dem",
    "build_mini_example",
    "build_hills_scenario",
    "build_paper_table2_scenario",
    # 輔助（非契約，供測試／上層使用）
    "metres_per_degree",
    "synthetic_dem_for_frame",
    "synthetic_dem_for_latlon_bounds",
    "enu_to_latlon",
    "enu_ring_to_wgs84",
    "site_from_enu",
    "scenario_frame",
    "legacy_fixed_wing_spec",
    "PLATFORM_LEGACY_FIXED_WING",
    # 常數
    "MINI_HOME_LATLON", "MINI_BASE_MSL_M", "MINI_DEM_STEP_M", "MINI_DEM_SIGMA_M", "MINI_DEM_BOUNDS_ENU",
    "MINI_AREA_BOUNDS_ENU", "MINI_ALT1_ENU",
    "HILLS_HOME_LATLON", "HILLS_BASE_MSL_M", "HILLS_DEM_STEP_M", "HILLS_DEM_BOUNDS_ENU", "HILLS_AMPLITUDE_M",
    "HILLS_WAVELENGTH_M", "HILLS_RIDGE_ENU", "HILLS_AREA1_ENU", "HILLS_AREA2_ENU", "HILLS_AREA2_HOLE_ENU",
    "HILLS_AREA3_ENU", "HILLS_SITES_ENU", "HILLS_NOFLY_ENU",
    "PAPER_TABLE2_UAV_START", "PAPER_TABLE2_Z1_LONLAT_ALT", "PAPER_TABLE2_V_MPS", "PAPER_TABLE2_R_MIN_M",
    "PAPER_TABLE2_H_COV_M", "PAPER_TABLE2_FOV_DEG", "PAPER_TABLE2_HEADING_DEG", "PAPER_TABLE2_DEM_BOUNDS_LATLON",
]


# ---------------------------------------------------------------------------
# 小工具：經緯度 ↔ 公尺、ENU → WGS84
# ---------------------------------------------------------------------------

def metres_per_degree(lat_deg: float) -> tuple[float, float]:
    """WGS84 於緯度 `lat_deg` 之 (每度緯度公尺數, 每度經度公尺數)（標準級數展開；誤差 < 1 m/deg）。

    只用於合成 DEM 的節點間距（`step_m → dlat/dlon`）與山丘／山脊在 DEM 內的公尺座標；
    任務幾何的 WGS84 ↔ ENU 一律走 :class:`geo.GeoFrame`（精確 ECEF）。
    """
    phi = math.radians(float(lat_deg))
    m_lat = 111132.954 - 559.822 * math.cos(2.0 * phi) + 1.175 * math.cos(4.0 * phi)
    m_lon = 111412.84 * math.cos(phi) - 93.5 * math.cos(3.0 * phi) + 0.118 * math.cos(5.0 * phi)
    return m_lat, m_lon


def enu_to_latlon(frame: GeoFrame, x_m: float, y_m: float, z_m: float = 0.0) -> tuple[float, float]:
    """ENU 點 → `(lat_deg, lon_deg)`（經 `GeoFrame.to_wgs84`，z 預設 0 = home 高度面）。"""
    w = frame.to_wgs84(np.array([[float(x_m), float(y_m), float(z_m)]], dtype=np.float64))
    return float(w[0, 0]), float(w[0, 1])


def enu_ring_to_wgs84(frame: GeoFrame, xy: Sequence[Sequence[float]] | np.ndarray) -> tuple[tuple[float, float], ...]:
    """ENU 頂點序列 `(N,2)` → WGS84 `((lat, lon), …)`（`AreaSpec.polygon_wgs84` / `NoFlyZone` / `Obstacle` 用）。"""
    arr = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    w = frame.to_wgs84(np.column_stack([arr, np.zeros(arr.shape[0])]))
    return tuple((float(a), float(b)) for a, b in w[:, :2])


def site_from_enu(frame: GeoFrame, site_id: str, x_m: float, y_m: float, kind: str = "alternate", **kw: Any) -> Site:
    """以 ENU 位置建立 :class:`mission.Site`（高度缺省由 DEM 取；其餘關鍵字轉交 `Site`）。"""
    lat, lon = enu_to_latlon(frame, x_m, y_m)
    return Site(site_id, lat, lon, kind=kind, **kw)


def scenario_frame(inputs: MissionInputs) -> GeoFrame:
    """由任務輸入建立 home 為原點之 :class:`geo.GeoFrame`（h0 = `home.alt_msl_m`，缺省由 DEM 取 (+alt_agl_m)）。

    與 `context.build_context` 的做法相同，供測試／viz 在不建 context 時取得座標框架。
    """
    home = inputs.home
    if home.alt_msl_m is not None:
        h0 = float(home.alt_msl_m)
    else:
        z = float(inputs.dem.sample_msl(home.lat_deg, home.lon_deg)[0])
        if not math.isfinite(z):
            raise MissionInputError(f"home {home.site_id!r} 不在 DEM 範圍內，無法取地面高")
        h0 = z + (float(home.alt_agl_m) if home.alt_agl_m is not None else 0.0)
    return GeoFrame(home.lat_deg, home.lon_deg, h0)


# ---------------------------------------------------------------------------
# 合成 DEM（契約 §16 synthetic_dem）
# ---------------------------------------------------------------------------

_RIDGE_KEYS = ("center_latlon", "azimuth_deg", "height_m", "width_m", "length_m")


def _ridge_field(U: np.ndarray, V: np.ndarray, ridge: Mapping[str, Any],
                 lat0_deg: float, lon0_deg: float, m_lat: float, m_lon: float) -> np.ndarray:
    """山脊高程場（公尺；與 `U, V` 同形）。

    山脊為沿方位 `azimuth_deg`（自北順時針）之直線脊，橫剖面為升餘弦
    ``h·½(1+cos(π d⊥/(w/2)))``（|d⊥| ≤ w/2），兩端在 `length_m/2` 內以同樣的升餘弦收尾。
    橫向最大坡度 = ``atan(h·π/w)``（h=30、w=400 → 13.3°），故「比 3° 陡」等價於 ``h·π/w > tan 3° = 0.0524``。
    """
    missing = [k for k in _RIDGE_KEYS if k not in ridge]
    if missing:
        raise MissionInputError(f"synthetic_dem: ridge 缺鍵 {missing}（需 {list(_RIDGE_KEYS)}）")
    unknown = sorted(set(ridge) - set(_RIDGE_KEYS))
    if unknown:
        raise MissionInputError(f"synthetic_dem: ridge 含未知鍵 {unknown}")
    lat_c, lon_c = (float(v) for v in ridge["center_latlon"])
    az = math.radians(float(ridge["azimuth_deg"]))
    height = float(ridge["height_m"])
    width = float(ridge["width_m"])
    length = float(ridge["length_m"])
    if not (width > 0.0 and length > 0.0):
        raise MissionInputError("synthetic_dem: ridge width_m / length_m 須 > 0")
    uc = (lon_c - lon0_deg) * m_lon
    vc = (lat_c - lat0_deg) * m_lat
    t_u, t_v = math.sin(az), math.cos(az)          # 沿脊單位向量 t̂ = (sin θ, cos θ)（契約 §0）
    n_u, n_v = math.cos(az), -math.sin(az)         # 右法向 n̂ = (cos θ, −sin θ)
    du = U - uc
    dv = V - vc
    d_par = du * t_u + dv * t_v
    d_perp = du * n_u + dv * n_v
    hw = 0.5 * width
    taper = min(hw, 0.5 * length)
    cross = np.where(np.abs(d_perp) <= hw, 0.5 * (1.0 + np.cos(np.pi * d_perp / hw)), 0.0)
    a = np.abs(d_par)
    flat_len = 0.5 * length - taper
    with np.errstate(invalid="ignore", divide="ignore"):
        tail = 0.5 * (1.0 + np.cos(np.pi * np.clip(a - flat_len, 0.0, taper) / taper))
    along = np.where(a <= flat_len, 1.0, np.where(a <= 0.5 * length, tail, 0.0))
    return height * cross * along


def _rough_field(U: np.ndarray, V: np.ndarray, noise_m: float, wavelength_m: float, seed: int) -> np.ndarray:
    """低通隨機起伏（6 個隨機方向／波長／相位之正弦和，最大振幅 ≤ `noise_m`）；`seed` 決定樣式。

    不用白噪聲：10 m 節點上 1 m 之白噪聲即產生 5.7° 的假坡度。
    """
    rng = np.random.default_rng(int(seed))
    n = 6
    amp = rng.uniform(0.5, 1.0, n)
    amp /= amp.sum()
    ang = rng.uniform(0.0, 2.0 * np.pi, n)
    lam = rng.uniform(0.25, 1.0, n) * float(wavelength_m)
    phase = rng.uniform(0.0, 2.0 * np.pi, n)
    z = np.zeros_like(U)
    for k in range(n):
        z += amp[k] * np.sin(2.0 * np.pi * (U * np.cos(ang[k]) + V * np.sin(ang[k])) / lam[k] + phase[k])
    return float(noise_m) * z


def synthetic_dem(lat0_deg: float, lon0_deg: float, size_km: tuple[float, float] | float, step_m: float,
                  base_msl_m: float = 100.0, amplitude_m: float = 0.0, ridge: dict | None = None, seed: int = 0,
                  *, wavelength_m: float = 3000.0, noise_m: float = 0.0, sigma_z_m: float | None = None,
                  is_dsm: bool = False) -> DemGrid:
    """合成 DEM（契約 §16）：平地 + 可選正弦山丘 + 可選山脊。

    參數
    ----
    lat0_deg, lon0_deg
        **左下角節點**（與 :class:`geo.DemGrid` 的 `lat0/lon0` 同義）。
    size_km
        `(east_km, north_km)` 東西／南北範圍（純量 → 正方形）。節點數 `cols = ⌈east/step⌉+1`、`rows = ⌈north/step⌉+1`；
        `dlat = step/m_lat`、`dlon = step/m_lon`（於 DEM 中心緯度換算）。
    step_m
        節點間距（公尺）。
    base_msl_m
        平地高程（m MSL）。
    amplitude_m
        山丘振幅 A：`z += A·sin(2πu/λ)·sin(2πv/λ)`，`u, v` = 自左下角起算的東／北公尺；`λ = wavelength_m`。
        此積形式在半波長格點 `(k·λ/2, m·λ/2)` 上梯度恰為 0（可把站點放在格點上以通過 5° 站點坡度檢查），
        最大坡度 `atan(2πA/λ)`（A=12、λ=3000 → 1.4°）。
    ridge
        `{"center_latlon": (lat, lon), "azimuth_deg": θ, "height_m": h, "width_m": w, "length_m": L}`：
        沿方位 θ（自北順時針）之升餘弦剖面山脊；橫向最大坡度 `atan(hπ/w)`，`hπ/w > tan 3°` 即「比 3° 陡」。
    seed
        只影響 `noise_m > 0` 時的隨機低通起伏（`noise_m` 為其最大振幅）。
    sigma_z_m, is_dsm
        轉交 :class:`geo.DemGrid`（`sigma_z_m` 進入 h_clr ≥ 3σ+30 下限，D11）。

    回傳 `DemGrid`（float32 m MSL、`vertical_datum="msl"`）。輸入錯誤丟 `MissionInputError`。
    """
    if np.isscalar(size_km):
        size = (float(size_km), float(size_km))  # type: ignore[arg-type]
    else:
        try:
            size = (float(size_km[0]), float(size_km[1]))  # type: ignore[index]
        except (TypeError, IndexError) as e:
            raise MissionInputError("synthetic_dem: size_km 須為 (east_km, north_km) 或純量") from e
    if not (size[0] > 0.0 and size[1] > 0.0):
        raise MissionInputError(f"synthetic_dem: size_km 須 > 0，得 {size}")
    step = float(step_m)
    if not step > 0.0:
        raise MissionInputError(f"synthetic_dem: step_m 須 > 0，得 {step_m}")
    if float(amplitude_m) < 0.0 or float(noise_m) < 0.0:
        raise MissionInputError("synthetic_dem: amplitude_m / noise_m 須 ≥ 0")
    if not float(wavelength_m) > 0.0:
        raise MissionInputError("synthetic_dem: wavelength_m 須 > 0")
    if not (-90.0 <= float(lat0_deg) <= 90.0 and -180.0 <= float(lon0_deg) <= 360.0):
        raise MissionInputError(f"synthetic_dem: lat0/lon0 超出範圍 ({lat0_deg}, {lon0_deg})")

    east_m = size[0] * 1000.0
    north_m = size[1] * 1000.0
    cols = max(2, int(math.ceil(east_m / step - 1e-9)) + 1)
    rows = max(2, int(math.ceil(north_m / step - 1e-9)) + 1)
    m_lat0, _ = metres_per_degree(lat0_deg)
    lat_mid = float(lat0_deg) + 0.5 * north_m / m_lat0
    m_lat, m_lon = metres_per_degree(lat_mid)
    dlat = step / m_lat
    dlon = step / m_lon

    u = np.arange(cols, dtype=np.float64) * step
    v = np.arange(rows, dtype=np.float64) * step
    U, V = np.meshgrid(u, v)                               # (rows, cols)：rows 沿 lat（v）、cols 沿 lon（u）
    z = np.full((rows, cols), float(base_msl_m), dtype=np.float64)
    if float(amplitude_m) > 0.0:
        k = 2.0 * np.pi / float(wavelength_m)
        z += float(amplitude_m) * np.sin(k * U) * np.sin(k * V)
    if float(noise_m) > 0.0:
        z += _rough_field(U, V, float(noise_m), float(wavelength_m), seed)
    if ridge is not None:
        if not isinstance(ridge, Mapping):
            raise MissionInputError("synthetic_dem: ridge 須為 dict")
        z += _ridge_field(U, V, ridge, float(lat0_deg), float(lon0_deg), m_lat, m_lon)
    return DemGrid(float(lat0_deg), float(lon0_deg), dlat, dlon, z.astype(np.float32),
                   is_dsm=bool(is_dsm), sigma_z_m=sigma_z_m)


def synthetic_dem_for_frame(frame: GeoFrame, bounds_enu: tuple[float, float, float, float], step_m: float,
                            **kw: Any) -> DemGrid:
    """以 ENU 邊界 `(xmin, ymin, xmax, ymax)`（m，原點 = frame）建立合成 DEM：左下角 = `to_wgs84(xmin, ymin)`，
    範圍 `((xmax−xmin)/1000, (ymax−ymin)/1000)` km；其餘關鍵字轉交 :func:`synthetic_dem`。

    DEM 為經緯對齊矩形而 ENU 邊界為切平面矩形，兩者在 20 km 內相差 < 10 m；呼叫端請留邊界裕度。
    """
    xmin, ymin, xmax, ymax = (float(v) for v in bounds_enu)
    if not (xmax > xmin and ymax > ymin):
        raise MissionInputError("synthetic_dem_for_frame: bounds_enu 須 xmax > xmin 且 ymax > ymin")
    lat_c, lon_c = enu_to_latlon(frame, xmin, ymin)
    return synthetic_dem(lat_c, lon_c, ((xmax - xmin) / 1000.0, (ymax - ymin) / 1000.0), step_m, **kw)


def synthetic_dem_for_latlon_bounds(lat_min: float, lon_min: float, lat_max: float, lon_max: float, step_m: float,
                                    **kw: Any) -> DemGrid:
    """以 WGS84 邊界建立合成 DEM（左下角 = (lat_min, lon_min)；範圍以中心緯度換算成 km）。"""
    if not (lat_max > lat_min and lon_max > lon_min):
        raise MissionInputError("synthetic_dem_for_latlon_bounds: 邊界須 lat_max > lat_min 且 lon_max > lon_min")
    m_lat, m_lon = metres_per_degree(0.5 * (lat_min + lat_max))
    size = ((lon_max - lon_min) * m_lon / 1000.0, (lat_max - lat_min) * m_lat / 1000.0)
    return synthetic_dem(lat_min, lon_min, size, step_m, **kw)


# ---------------------------------------------------------------------------
# §W mini example（契約 §W.1 逐項）
# ---------------------------------------------------------------------------

MINI_HOME_LATLON: tuple[float, float] = (23.95, 120.90)
"""home (23.9500°N, 120.9000°E)；MSL 由 DEM 取（平地 100 m）。"""
MINI_BASE_MSL_M: float = 100.0
MINI_DEM_STEP_M: float = 10.0
MINI_DEM_SIGMA_M: float = 2.0
MINI_DEM_BOUNDS_ENU: tuple[float, float, float, float] = (-2500.0, -2500.0, 8500.0, 2500.0)
"""DEM 涵蓋範圍（ENU m）：起飛走廊西側、區域東側 1.5 km、迴轉帶／走廊南北各 ≥ 1.2 km 裕度。"""
MINI_AREA_BOUNDS_ENU: tuple[float, float, float, float] = (4000.0, -600.0, 7000.0, 600.0)
"""任務區域 Z1：矩形 ENU (4000,−600)→(7000,600)，3 km × 1.2 km，近邊距起飛點 4 km。"""
MINI_ALT1_ENU: tuple[float, float] = (2000.0, 1500.0)


def _rect_ring(bounds: tuple[float, float, float, float]) -> list[tuple[float, float]]:
    x0, y0, x1, y1 = bounds
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def build_mini_example() -> tuple[MissionInputs, PlannerConfig]:
    """契約 §W.1 mini example（所有 §W 手算數值之輸入）。

    * home (23.95°N, 120.90°E)，MSL 由 DEM 取；合成平地 DEM z = 100 m MSL、10 m 步長、`sigma_z_m = 2.0`；
      `Airspace(max_agl_m=120, waiver_max_agl_m=200, terrain_clearance_m=36)`（36 ≥ 3σ+30；示例假設已取得 200 m 豁免，D12）。
    * 區域 `Z1`：ENU 矩形 (4000,−600)→(7000,600) 經 `GeoFrame` 轉 WGS84；`sensor_req="any"`、`priority=1`、`payload_req_id="default"`。
    * 備降點 `ALT1`：ENU (2000, 1500)。風：單層 5 m/s from 270°（西風）、陣風 7；`WeatherObs` 5/7 m/s、能見度 8 km、無雨、25 °C。
    * 機隊：`A1`（PLATFORM_PIONEER、`LidarHardware()`、`SortieEnergy(24.6, 5.0, 1350 Wh, pack 1500)`）；
      `B1`（PLATFORM_HERO、`CameraHardware()`、`SortieEnergy(6.65, 0.6, 380 Wh, pack 420)`）。
    * 酬載需求：`default`（lidar、density 20、side 0.25、h_target 160——使 |h_A−h_B| ≥ Δh_sep+δ_prof=60）；
      `default_rgb`（rgb、GSD 0.05、side 0.30、forward 0.70、h_target 100）。
    * `C2Link(range_km=25)`（單值 → A、B 皆 25 km）；`PlannerConfig()` 全預設。
    """
    lat_h, lon_h = MINI_HOME_LATLON
    frame = GeoFrame(lat_h, lon_h, MINI_BASE_MSL_M)
    dem = synthetic_dem_for_frame(frame, MINI_DEM_BOUNDS_ENU, MINI_DEM_STEP_M,
                                  base_msl_m=MINI_BASE_MSL_M, sigma_z_m=MINI_DEM_SIGMA_M)
    home = Site("H", lat_h, lon_h, kind="home")
    alt1 = site_from_enu(frame, "ALT1", *MINI_ALT1_ENU)
    area = AreaSpec("Z1", enu_ring_to_wgs84(frame, _rect_ring(MINI_AREA_BOUNDS_ENU)),
                    sensor_req="any", priority=1.0, payload_req_id="default")
    wind = (WindLayer(0.0, 3000.0, 5.0, 270.0, 7.0, "agl"),)
    a1 = UAVInstance("A1", PLATFORM_PIONEER, (LidarHardware(),),
                     SortieEnergy("A1", 24.6, payload_kg=5.0, available_wh=1350.0, pack_nominal_wh=1500.0))
    b1 = UAVInstance("B1", PLATFORM_HERO, (CameraHardware(),),
                     SortieEnergy("B1", 6.65, payload_kg=0.6, available_wh=380.0, pack_nominal_wh=420.0))
    reqs = {
        "default": PayloadRequirement("default", "lidar", density_pt_m2=20.0, side_overlap=0.25, h_target_agl_m=160.0),
        "default_rgb": PayloadRequirement("default_rgb", "rgb", gsd_required_m=0.05, side_overlap=0.30,
                                          forward_overlap=0.70, h_target_agl_m=100.0),
    }
    c2 = C2Link(range_km=25.0, coverage_polygon_wgs84=None, ground_station=home, lost_link_policy={})
    airspace = Airspace(max_agl_m=120.0, waiver_max_agl_m=200.0, terrain_clearance_m=36.0)
    obs = WeatherObs(mean_wind_mps=5.0, gust_mps=7.0, wind_dir_from_deg=270.0, visibility_m=8000.0,
                     precip_mmh=0.0, temp_c=25.0)
    inputs = MissionInputs(home=home, alternates=(alt1,), areas=(area,), dem=dem, obstacles=(), wind=wind,
                           fleet=(a1, b1), payload_reqs=reqs, c2=c2, airspace=airspace,
                           weather_thresholds=WeatherThresholds(), weather_obs=obs)
    return inputs, PlannerConfig()


# ---------------------------------------------------------------------------
# 山丘情境（規格附錄／§17.2：8 機／3 區／山脊／電塔／輸電線／建物／禁飛／分層風／2 備降點）
# ---------------------------------------------------------------------------

HILLS_HOME_LATLON: tuple[float, float] = (24.10, 120.75)
HILLS_BASE_MSL_M: float = 100.0
HILLS_DEM_STEP_M: float = 30.0
"""DEM 原生節點距（TerrainGrid 仍以 cfg.dem_step_m=10 m 重取樣）；30 m 對 3 km 波長山丘與 400 m 寬山脊已足夠。"""
HILLS_DEM_BOUNDS_ENU: tuple[float, float, float, float] = (-3000.0, -4500.0, 15500.0, 7500.0)
HILLS_AMPLITUDE_M: float = 12.0
HILLS_WAVELENGTH_M: float = 3000.0
"""山丘：A=12 m、λ=3 km（最大坡度 1.4°）。home 與兩備降點皆置於半波長格點（相對 DEM 左下角 (3000,4500)、(1500,7500)、
(12000,1500)）→ 站點坡度 0。D12 高度帶預算：A 胞（⊕ 迴轉帶）內地形起伏須 ≤ waiver 200 − h_minfw 120 − Δh_band,min 20 = 60 m，
區域 1 內起伏 ≤ 2A + 山脊高 = 54 m。"""
HILLS_RIDGE_ENU: dict[str, Any] = {"center_xy": (6300.0, 1300.0), "azimuth_deg": 30.0, "height_m": 30.0,
                                   "width_m": 400.0, "length_m": 4500.0}
"""橫越區域 1 的山脊（ENU 中心；橫向最大坡度 atan(30π/400) = 13.3° > 3°）。"""
HILLS_AREA1_ENU: tuple[tuple[float, float], ...] = (
    (4000.0, 300.0), (5500.0, -200.0), (7800.0, 0.0), (8600.0, 1000.0), (8000.0, 2500.0), (5800.0, 2900.0), (4300.0, 2000.0))
"""區域 1（`lidar`）：大型凸七邊形（≈ 4.6 km × 3.1 km），山丘＋山脊之上。"""
HILLS_AREA2_ENU: tuple[tuple[float, float], ...] = (
    (-600.0, 3900.0), (3200.0, 3900.0), (3200.0, 5000.0), (1900.0, 5000.0), (1900.0, 5500.0), (3200.0, 5500.0),
    (3200.0, 6600.0), (-600.0, 6600.0))
"""區域 2（`both`）：東側開口之 C 形凹多邊形（3.8 km × 2.7 km）。"""
HILLS_AREA2_HOLE_ENU: tuple[tuple[float, float], ...] = ((200.0, 4500.0), (900.0, 4500.0), (900.0, 5100.0), (200.0, 5100.0))
HILLS_AREA3_ENU: tuple[tuple[float, float], ...] = ((13000.0, -1200.0), (13800.0, -1200.0), (13800.0, -600.0), (13000.0, -600.0))
"""區域 3（`rgb`）：遠處小矩形 800 m × 600 m（距 home ≈ 13.4 km；B 之 C2 30 km／ρ_sym 39 km 內）。"""
HILLS_SITES_ENU: dict[str, tuple[float, float]] = {"H": (0.0, 0.0), "ALT1": (-1500.0, 3000.0), "ALT2": (9000.0, -3000.0)}
HILLS_NOFLY_ENU: tuple[tuple[float, float], ...] = ((9000.0, -1400.0), (11000.0, -1400.0), (11000.0, -400.0), (9000.0, -400.0))
"""禁飛區 NF1：位於 home → 區域 3 直線上（迫使繞行）。"""
_HILLS_TOWERS_ENU: tuple[tuple[str, float, float, float], ...] = (("T1", 2000.0, 300.0, 80.0), ("T2", 9000.0, 1500.0, 60.0))
_HILLS_POWERLINE_ENU: tuple[tuple[float, float], ...] = ((-1000.0, 3300.0), (1200.0, 3450.0), (3500.0, 3500.0))
_HILLS_BUILDING_ENU: tuple[tuple[float, float], ...] = ((11800.0, -1000.0), (12100.0, -1000.0), (12100.0, -700.0), (11800.0, -700.0))


def build_hills_scenario() -> tuple[MissionInputs, PlannerConfig]:
    """「山丘」情境（規格 §17.2 `runtime_under_1s`／`example_coverage_ge_99p5` 之 8 機／3 區情境）。

    * 機隊 8 架：`A1–A3`（Pioneer、LiDAR、1350/1400/1300 of 1500 Wh）、`B1–B5`（HERO、相機、370–400 of 420 Wh）。
    * 區域 3 個：`Z1` 大型凸 `lidar` 區（山丘＋橫越山脊 13.3° > 3°）、`Z2` 含洞之凹 `both` 區（時間窗 0–7200 s）、
      `Z3` 遠處小 `rgb` 區（13.4 km）。酬載需求：`lidar_survey`（h_target 160）、`both_survey`／`both_survey_rgb`、`rgb_far`。
    * 障礙：電塔 `T1`(80 m)、`T2`(60 m)、輸電線 `PL1`(45 m)、建物 `BLD1`(25 m)；禁飛 `NF1`（home→Z3 直線上）。
    * 風：4 m/s @ 100–300 m AGL、7 m/s @ 300–600 m AGL，皆 from 240°（陣風 6／9）；`WeatherObs` 4/6 m/s、22 °C。
    * 站點：`H` + 備降 `ALT1`(−1.5, 3.0 km)、`ALT2`(9.0, −3.0 km)（皆在山丘格點上，坡度 0）；C2 30 km；
      `Airspace(120, waiver 200, terrain_clearance 36, time_window 0–10800 s)`；`PlannerConfig()` 全預設。
    """
    lat_h, lon_h = HILLS_HOME_LATLON
    frame = GeoFrame(lat_h, lon_h, HILLS_BASE_MSL_M)
    rc = HILLS_RIDGE_ENU
    ridge = {"center_latlon": enu_to_latlon(frame, *rc["center_xy"]), "azimuth_deg": rc["azimuth_deg"],
             "height_m": rc["height_m"], "width_m": rc["width_m"], "length_m": rc["length_m"]}
    dem = synthetic_dem_for_frame(frame, HILLS_DEM_BOUNDS_ENU, HILLS_DEM_STEP_M, base_msl_m=HILLS_BASE_MSL_M,
                                  amplitude_m=HILLS_AMPLITUDE_M, ridge=ridge, wavelength_m=HILLS_WAVELENGTH_M,
                                  sigma_z_m=2.0)
    home = Site("H", lat_h, lon_h, kind="home")
    alt1 = site_from_enu(frame, "ALT1", *HILLS_SITES_ENU["ALT1"])
    alt2 = site_from_enu(frame, "ALT2", *HILLS_SITES_ENU["ALT2"])
    areas = (
        AreaSpec("Z1", enu_ring_to_wgs84(frame, HILLS_AREA1_ENU), sensor_req="lidar", priority=2.0,
                 payload_req_id="lidar_survey"),
        AreaSpec("Z2", enu_ring_to_wgs84(frame, HILLS_AREA2_ENU), holes_wgs84=(enu_ring_to_wgs84(frame, HILLS_AREA2_HOLE_ENU),),
                 sensor_req="both", priority=1.5, time_window_s=(0.0, 7200.0), payload_req_id="both_survey"),
        AreaSpec("Z3", enu_ring_to_wgs84(frame, HILLS_AREA3_ENU), sensor_req="rgb", priority=1.0,
                 payload_req_id="rgb_far"),
    )
    obstacles: list[Obstacle] = [
        Obstacle(oid, "point", (enu_to_latlon(frame, x, y),), top_msl_m=None, height_agl_m=h, category="tower")
        for oid, x, y, h in _HILLS_TOWERS_ENU
    ]
    obstacles.append(Obstacle("PL1", "line", enu_ring_to_wgs84(frame, _HILLS_POWERLINE_ENU), top_msl_m=None,
                              height_agl_m=45.0, category="powerline"))
    obstacles.append(Obstacle("BLD1", "polygon", enu_ring_to_wgs84(frame, _HILLS_BUILDING_ENU), top_msl_m=None,
                              height_agl_m=25.0, category="building"))
    wind = (WindLayer(100.0, 300.0, 4.0, 240.0, 6.0, "agl"), WindLayer(300.0, 600.0, 7.0, 240.0, 9.0, "agl"))
    fleet: list[UAVInstance] = []
    for uid, e in (("A1", 1350.0), ("A2", 1400.0), ("A3", 1300.0)):
        fleet.append(UAVInstance(uid, PLATFORM_PIONEER, (LidarHardware(),),
                                 SortieEnergy(uid, 24.6, payload_kg=5.0, available_wh=e, pack_nominal_wh=1500.0)))
    for uid, auw, pl, e in (("B1", 6.65, 0.6, 380.0), ("B2", 6.65, 0.6, 400.0), ("B3", 6.70, 0.65, 390.0),
                            ("B4", 6.65, 0.6, 370.0), ("B5", 6.65, 0.6, 385.0)):
        fleet.append(UAVInstance(uid, PLATFORM_HERO, (CameraHardware(),),
                                 SortieEnergy(uid, auw, payload_kg=pl, available_wh=e, pack_nominal_wh=420.0)))
    reqs = {
        "lidar_survey": PayloadRequirement("lidar_survey", "lidar", density_pt_m2=20.0, side_overlap=0.25,
                                           h_target_agl_m=160.0),
        "both_survey": PayloadRequirement("both_survey", "lidar", density_pt_m2=20.0, side_overlap=0.25,
                                          h_target_agl_m=160.0),
        "both_survey_rgb": PayloadRequirement("both_survey_rgb", "rgb", gsd_required_m=0.05, side_overlap=0.30,
                                              forward_overlap=0.70, h_target_agl_m=100.0),
        "rgb_far": PayloadRequirement("rgb_far", "rgb", gsd_required_m=0.05, side_overlap=0.30, forward_overlap=0.70,
                                      h_target_agl_m=100.0),
    }
    c2 = C2Link(range_km=30.0, coverage_polygon_wgs84=None, ground_station=home, lost_link_policy={})
    nofly = (NoFlyZone("NF1", enu_ring_to_wgs84(frame, HILLS_NOFLY_ENU)),)
    airspace = Airspace(max_agl_m=120.0, waiver_max_agl_m=200.0, no_fly=nofly, time_window_s=(0.0, 10800.0),
                        terrain_clearance_m=36.0)
    obs = WeatherObs(mean_wind_mps=4.0, gust_mps=6.0, wind_dir_from_deg=240.0, visibility_m=9000.0,
                     precip_mmh=0.0, temp_c=22.0)
    inputs = MissionInputs(home=home, alternates=(alt1, alt2), areas=areas, dem=dem, obstacles=tuple(obstacles),
                           wind=wind, fleet=tuple(fleet), payload_reqs=reqs, c2=c2, airspace=airspace,
                           weather_thresholds=WeatherThresholds(), weather_obs=obs)
    return inputs, PlannerConfig()


# ---------------------------------------------------------------------------
# 論文 Table 2 情境（規格 §15、§17.1 (7)；legacy_mode）
# ---------------------------------------------------------------------------

PAPER_TABLE2_V_MPS: float = 230.0
PAPER_TABLE2_R_MIN_M: float = 4000.0
PAPER_TABLE2_H_COV_M: float = 1200.0
PAPER_TABLE2_HEADING_DEG: float = 30.0
PAPER_TABLE2_FOV_DEG: tuple[float, float, float] = (72.7, 45.0, 76.98)
"""(α_m 安裝角, α_v 垂直張角, α_h 水平張角)；h=1200 → w ≈ 2200.1、l ≈ 999.8 m（規格 §4.3）。"""
PAPER_TABLE2_UAV_START: dict[str, tuple[float, float, float, float]] = {
    "U1": (36.895344, 93.728342, 2730.8, PAPER_TABLE2_HEADING_DEG),
    "U2": (36.607982, 93.697064, 2684.2, PAPER_TABLE2_HEADING_DEG),
    "U3": (36.411013, 93.604920, 2687.8, PAPER_TABLE2_HEADING_DEG),
}
"""論文 Table 2 初始狀態 uav_id → (lat_deg, lon_deg, alt_msl_m, heading_deg)。論文原文以 (lon, lat, alt) 列出。
U1 位置為 home、U2/U3 位置為站點 `U2`/`U3`；`build_paper_table2_scenario` 以 `UAVInstance.start_site_id = uav_id`
讓三機各自其地面站起飛（論文 §4.1）；legacy 執行器（sim.paper_table2_check）亦可以此字典建立各機起始 `UAVState`。"""
PAPER_TABLE2_Z1_LONLAT_ALT: tuple[tuple[float, float, float], ...] = (
    (94.035474, 37.071295, 2698.7), (94.191312, 37.379347, 2704.4), (94.635268, 37.560118, 3083.0),
    (94.642987, 37.309750, 2691.4), (94.811039, 37.127469, 2684.5), (94.387015, 36.949887, 2680.8))
"""論文 Table 2 區域 Z1 六頂點 (lon, lat, alt)；面積 2604.34 km²、最大頂點高差 402.2 m、val=1。"""
PAPER_TABLE2_DEM_BOUNDS_LATLON: tuple[float, float, float, float] = (36.30, 93.40, 37.75, 95.00)
"""(lat_min, lon_min, lat_max, lon_max)：涵蓋論文任務圖 [93.5°,36.5°]–[94.9°,37.5°] 與 Z1 北頂點 37.56° 及 4 km 迴轉裕度。"""
_PAPER_DEM_STEP_M: float = 500.0
_PAPER_BASE_MSL_M: float = 2700.0
_PAPER_BATTERY_WH: float = 50_000.0


def legacy_fixed_wing_spec(v_mps: float = PAPER_TABLE2_V_MPS, r_min_m: float = PAPER_TABLE2_R_MIN_M,
                           name: str = "Legacy fixed-wing (paper Table 2)") -> PlatformSpec:
    """論文同構固定翼平台之 :class:`platform.PlatformSpec`（規格 §15 退化條件用）。

    * `cruise = min_mission = V`（論文 A2 定速）、`R_theo = R_plan = R_min`、`max_bank_angle_deg = atan(V²/(g R))`
      使 `__post_init__` 之 `Va²/(g tanφ) ≈ R_theo` 恰好成立（V=230、R=4000 → 53.4°）。
    * 坡度沿用 3°/4°（規劃）與 3.5°/5°（上限）——論文 Table 2 之 γ_i = 0 為作業段水平飛行，Eq.36 之俯仰限制值論文未列。
    * 能量／鏈路在 `legacy_mode` 下關閉，但仍給自洽值：T_ref 180 min → 總航程 2484 km、`reserve_fraction = 0`、
      `usable_range_fraction = (1, 1)`、示例電池 50 kWh；航電旗標皆 True（k_unc = 1）；`provisional = ∅`（無 provisional 警告）。
    * 轉換段：`transition_ground_run_m = 800`、`a_tr = 5 m/s²`（避免 V=230 之公式給出數十 km 的加減速段）；
      cls 取 "A"（同構機隊）。
    """
    v = float(v_mps)
    r = float(r_min_m)
    if not (v > 0.0 and r > 0.0):
        raise MissionInputError("legacy_fixed_wing_spec: v_mps / r_min_m 須 > 0")
    bank_deg = math.degrees(math.atan(v * v / (G_MPS2 * r)))
    t_ref_min = 180.0
    total_range_km = v * t_ref_min * 60.0 / 1000.0
    return PlatformSpec(
        name=name, cls="A", platform_type="fixed_wing_legacy",
        wingspan_m=10.0, fuselage_length_m=8.0, wing_area_m2=20.0,
        mtow_kg=1000.0, reference_auw_kg=900.0, reference_payload_kg=100.0, payload_max_kg=100.0,
        cruise_airspeed_mps=v, cruise_airspeed_kmh=round(3.6 * v, 1),
        min_mission_airspeed_mps=v,
        transition_airspeed_mps=min(60.0, v), stall_airspeed_mps=min(50.0, v),
        max_airspeed_mps=None,
        max_climb_angle_deg=3.5, planning_climb_angle_deg=3.0,
        max_descent_angle_deg=5.0, planning_descent_angle_deg=4.0,
        max_fw_climb_rate_mps=v * math.sin(math.radians(3.0)),
        max_fw_descent_rate_mps=v * math.sin(math.radians(4.0)),
        max_bank_angle_deg=bank_deg,
        turn_radius_theoretical_m=r, turn_radius_planning_m=r,
        vtol_site_clear_radius_m=10.0, obstacle_horizontal_buffer_m=30.0,
        transition_start_agl_m=60.0, transition_complete_agl_m=(80.0, 100.0),
        min_fw_operation_agl_m=100.0,
        wind_resistance_public=5,
        reference_endurance_min=t_ref_min, reference_total_range_km=total_range_km,
        reserve_fraction=0.0, usable_range_fraction=(1.0, 1.0),
        usable_distance_km=(total_range_km, total_range_km),
        symmetric_mission_radius_km=(0.5 * total_range_km, 0.5 * total_range_km),
        airspeed_sensor_validated=True, battery_monitor_validated=True,
        c2_link_range_km=None, terrain_clearance_m=None,
        alternate_landing_sites_required=False,
        battery_nominal_wh=_PAPER_BATTERY_WH, reference_battery_wh=_PAPER_BATTERY_WH,
        a_tr_mps2=5.0, transition_ground_run_m=800.0,
        provisional=frozenset(),
    )


PLATFORM_LEGACY_FIXED_WING: PlatformSpec = legacy_fixed_wing_spec()
"""論文 Table 2 之同構固定翼平台（V=230 m/s、R=4000 m）。"""


def build_paper_table2_scenario() -> tuple[MissionInputs, PlannerConfig]:
    """論文 Table 2 情境（規格 §15、§17.1 (7)：3 機、V=230、R=4000、h=1200、w≈2200、平地 → 路徑長 590–650 km）。

    * home = U1 地面站 (36.895344°N, 93.728342°E)、站點 `U2`/`U3` = 其餘兩機之地面站（起始狀態見
      :data:`PAPER_TABLE2_UAV_START`）；高度皆由 DEM 取。三機各以 `UAVInstance.start_site_id = uav_id`
      自**各自**地面站起飛（論文 §4.1）；回收點仍為 home（D40）。
    * DEM：平地 2700 m MSL、500 m 節點、涵蓋 :data:`PAPER_TABLE2_DEM_BOUNDS_LATLON`（規格 §17.1 (7) 指定平地）。
    * 區域 `Z1`：論文六頂點（(lon, lat) → (lat, lon)）、`sensor_req="any"`、`payload_req_id="paper"`：
      `PayloadRequirement("paper", "lidar", side_overlap=0, forward_overlap=0, h_target_agl_m=1200)`（論文條帶間距 = w）。
    * 機隊 `U1–U3`：:data:`PLATFORM_LEGACY_FIXED_WING`、`TrapezoidFovSpec("lidar", 72.7, 45, 76.98)`、50 kWh 示例電池。
    * 無風（單一 0 m/s 層）、C2 1000 km、`Airspace(max_agl_m=3000, terrain_clearance_m=30)`。
    * `PlannerConfig(legacy_mode=True, dem_step_m=250)`：legacy 一鍵退化（鎖步 IDP、`("rev",)`、B=1、K=∞、`metric="paper"`、
      整體凸化、梯形 FOV、e_ext=0、k_unc=1、能量／鏈路／時間窗關閉、t_RTB=0、h_ceil=∞）；`dem_step_m=250` 因區域跨 70 km。
    """
    lat_h, lon_h, _alt_h, _hdg = PAPER_TABLE2_UAV_START["U1"]
    lat_min, lon_min, lat_max, lon_max = PAPER_TABLE2_DEM_BOUNDS_LATLON
    dem = synthetic_dem_for_latlon_bounds(lat_min, lon_min, lat_max, lon_max, _PAPER_DEM_STEP_M,
                                          base_msl_m=_PAPER_BASE_MSL_M)
    home = Site("U1", lat_h, lon_h, kind="home")
    alternates = tuple(Site(uid, PAPER_TABLE2_UAV_START[uid][0], PAPER_TABLE2_UAV_START[uid][1], kind="alternate")
                       for uid in ("U2", "U3"))
    ring = tuple((lat, lon) for lon, lat, _alt in PAPER_TABLE2_Z1_LONLAT_ALT)
    area = AreaSpec("Z1", ring, sensor_req="any", priority=1.0, payload_req_id="paper")
    am, av, ah = PAPER_TABLE2_FOV_DEG
    hw = TrapezoidFovSpec("lidar", am, av, ah)
    fleet = tuple(
        UAVInstance(uid, PLATFORM_LEGACY_FIXED_WING, (hw,),
                    SortieEnergy(uid, PLATFORM_LEGACY_FIXED_WING.reference_auw_kg,
                                 payload_kg=PLATFORM_LEGACY_FIXED_WING.reference_payload_kg,
                                 available_wh=_PAPER_BATTERY_WH, pack_nominal_wh=_PAPER_BATTERY_WH),
                    start_site_id=uid)
        for uid in ("U1", "U2", "U3"))
    reqs = {"paper": PayloadRequirement("paper", "lidar", side_overlap=0.0, forward_overlap=0.0,
                                        h_target_agl_m=PAPER_TABLE2_H_COV_M)}
    c2 = C2Link(range_km=1000.0, coverage_polygon_wgs84=None, ground_station=home, lost_link_policy={})
    airspace = Airspace(max_agl_m=3000.0, terrain_clearance_m=30.0)
    wind = (WindLayer(0.0, 5000.0, 0.0, 0.0, 0.0, "agl"),)
    obs = WeatherObs(mean_wind_mps=0.0, gust_mps=0.0, wind_dir_from_deg=0.0, visibility_m=10000.0,
                     precip_mmh=0.0, temp_c=15.0)
    inputs = MissionInputs(home=home, alternates=alternates, areas=(area,), dem=dem, obstacles=(), wind=wind,
                           fleet=fleet, payload_reqs=reqs, c2=c2, airspace=airspace,
                           weather_thresholds=WeatherThresholds(), weather_obs=obs)
    return inputs, PlannerConfig(legacy_mode=True, dem_step_m=250.0)
