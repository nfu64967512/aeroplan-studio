"""geo.py — WGS84 ↔ ECEF ↔ ENU 座標框架、DEM 網格與 ENU 地形網格。

契約 §1；規格 §2（符號）、§3（座標框架）、§11.1（走廊地形取樣）、D42（高度基準一律 ENU z）、
A9（ENU z 已含地球曲率，論文 Eq.(35) 不需另作補償）。

全域約定（契約 §0）
------------------
* 本地 ENU（m），原點 = 主起飛點 ``home``；``x`` = East、``y`` = North、``z`` = Up。
* 航跡方位 ``theta``（rad，自北順時針）：``t̂ = (sin θ, cos θ)``，右法向 ``n̂ = (cos θ, −sin θ)``。
* 內部一律 ENU ``z``；MSL 只出現在輸入型別，於解析時以 :meth:`GeoFrame.msl_to_enu_z` 轉換。
  ENU z 與 MSL 之差 = h0 + 曲率項（位置相依；50 km 處 ≈ 196 m），不得手動相減。
* 地形高度以雙線性內插（論文 Eq.(4)）取值；範圍外回 ``nan``。

第 0 層模組：只依賴 numpy / shapely。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

import numpy as np
import shapely
from numpy.typing import ArrayLike

__all__ = [
    "WGS84_A",
    "WGS84_F",
    "WGS84_B",
    "WGS84_E2",
    "geodetic_to_ecef",
    "ecef_to_geodetic",
    "GeoFrame",
    "DemGrid",
    "TerrainGrid",
]

# ---------------------------------------------------------------------------
# WGS84 橢球常數
# ---------------------------------------------------------------------------
WGS84_A: float = 6378137.0
WGS84_F: float = 1 / 298.257223563
WGS84_B: float = WGS84_A * (1.0 - WGS84_F)              # 短半軸
WGS84_E2: float = WGS84_F * (2.0 - WGS84_F)             # 第一離心率平方 e²
_WGS84_EP2: float = (WGS84_A**2 - WGS84_B**2) / WGS84_B**2   # 第二離心率平方 e'²

_CHUNK: int = 1 << 20   # from_dem 逐塊處理之節點數（限制記憶體峰值）


# ---------------------------------------------------------------------------
# 大地座標 ↔ ECEF
# ---------------------------------------------------------------------------
def geodetic_to_ecef(lat_deg: ArrayLike, lon_deg: ArrayLike, h_ell_m: ArrayLike) -> np.ndarray:
    """WGS84 大地座標（度、**橢球高**）→ ECEF（m）。

    參數可為純量或任意可廣播之陣列；回傳 ``(N, 3)`` float64。
    """
    lat = np.radians(np.atleast_1d(np.asarray(lat_deg, dtype=np.float64)))
    lon = np.radians(np.atleast_1d(np.asarray(lon_deg, dtype=np.float64)))
    h = np.atleast_1d(np.asarray(h_ell_m, dtype=np.float64))
    lat, lon, h = np.broadcast_arrays(lat, lon, h)
    sl, cl = np.sin(lat), np.cos(lat)
    n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sl * sl)     # 卯酉圈曲率半徑 N(φ)
    x = (n + h) * cl * np.cos(lon)
    y = (n + h) * cl * np.sin(lon)
    z = (n * (1.0 - WGS84_E2) + h) * sl
    return np.stack([x, y, z], axis=-1).reshape(-1, 3)


def ecef_to_geodetic(xyz: np.ndarray) -> np.ndarray:
    """ECEF（m）→ WGS84 ``[lat_deg, lon_deg, h_ell_m]``（``(N, 3)`` float64）。

    Bowring (1976) 閉式初值 + 兩次定點修正 ``φ ← atan2(z + e²N(φ)sinφ, p)``
    （契約：Bowring 閉式 + 牛頓修正；精度 < 1 mm，實際 ~1e-9 m）。
    橢球高以 ``h = p·cosφ + z·sinφ − a·sqrt(1 − e²sin²φ)`` 計算，兩極無奇異點。
    """
    arr = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    x, y, z = arr[:, 0], arr[:, 1], arr[:, 2]
    p = np.hypot(x, y)
    lon = np.arctan2(y, x)
    theta = np.arctan2(z * WGS84_A, p * WGS84_B)
    st, ct = np.sin(theta), np.cos(theta)
    lat = np.arctan2(z + _WGS84_EP2 * WGS84_B * st**3, p - WGS84_E2 * WGS84_A * ct**3)
    for _ in range(2):
        sl = np.sin(lat)
        n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sl * sl)
        lat = np.arctan2(z + WGS84_E2 * n * sl, p)
    sl, cl = np.sin(lat), np.cos(lat)
    h = p * cl + z * sl - WGS84_A * np.sqrt(1.0 - WGS84_E2 * sl * sl)
    return np.stack([np.degrees(lat), np.degrees(lon), h], axis=-1)


# ---------------------------------------------------------------------------
# GeoFrame
# ---------------------------------------------------------------------------
class GeoFrame:
    """本地 ENU 框架。原點 ``(lat0, lon0, h0_msl)``（規格 §3、A9、D42）。

    * ``geoid_undulation_m`` = 大地水準面起伏 N：橢球高 = MSL + N。本框架為 MSL↔橢球 轉換的
      唯一依據（DemGrid 之 ``vertical_datum="ellipsoid"`` 亦先換成 MSL 再經本框架）。
    * ``rot``：ECEF→ENU 旋轉矩陣，列序 (E, N, U)；``enu = rot @ (ecef − origin_ecef)``。
    * ENU z 為「沿原點法向之高度」，遠離原點時因地球曲率而低於 MSL 差值：
      50 km 處平地 z ≈ −196 m（契約不變量）。
    """

    __slots__ = ("lat0_deg", "lon0_deg", "h0_msl_m", "geoid_undulation_m", "origin_ecef", "rot")

    def __init__(self, lat0_deg: float, lon0_deg: float, h0_msl_m: float, geoid_undulation_m: float = 0.0) -> None:
        self.lat0_deg: float = float(lat0_deg)
        self.lon0_deg: float = float(lon0_deg)
        self.h0_msl_m: float = float(h0_msl_m)
        self.geoid_undulation_m: float = float(geoid_undulation_m)
        self.origin_ecef: np.ndarray = geodetic_to_ecef(self.lat0_deg, self.lon0_deg, self.h0_ell_m)[0]
        phi = math.radians(self.lat0_deg)
        lam = math.radians(self.lon0_deg)
        sp, cp, sl, cl = math.sin(phi), math.cos(phi), math.sin(lam), math.cos(lam)
        self.rot: np.ndarray = np.array(
            [
                [-sl, cl, 0.0],                 # East
                [-sp * cl, -sp * sl, cp],       # North
                [cp * cl, cp * sl, sp],         # Up（原點橢球法向）
            ],
            dtype=np.float64,
        )

    # -- 基本屬性 -----------------------------------------------------------
    @property
    def h0_ell_m(self) -> float:
        """原點橢球高 = h0_msl + N。"""
        return self.h0_msl_m + self.geoid_undulation_m

    def __repr__(self) -> str:  # pragma: no cover - 顯示用
        return (f"GeoFrame(lat0_deg={self.lat0_deg}, lon0_deg={self.lon0_deg}, "
                f"h0_msl_m={self.h0_msl_m}, geoid_undulation_m={self.geoid_undulation_m})")

    # -- 轉換 -----------------------------------------------------------------
    def to_enu(self, lat_deg: ArrayLike, lon_deg: ArrayLike, h_msl_m: ArrayLike = 0.0) -> np.ndarray:
        """WGS84（deg）+ MSL（m）→ ENU ``(N, 3)`` ``[x, y, z]``（m）。精確（經 ECEF），含曲率。"""
        h_ell = np.asarray(h_msl_m, dtype=np.float64) + self.geoid_undulation_m
        ecef = geodetic_to_ecef(lat_deg, lon_deg, h_ell)
        return (ecef - self.origin_ecef) @ self.rot.T

    def to_wgs84(self, enu: np.ndarray) -> np.ndarray:
        """ENU ``(N, 3)``（或 ``(3,)``）→ ``(N, 3)`` ``[lat_deg, lon_deg, h_msl_m]``。"""
        arr = np.asarray(enu, dtype=np.float64).reshape(-1, 3)
        ecef = arr @ self.rot + self.origin_ecef          # rot 正交 ⇒ rot⁻¹ = rotᵀ
        out = ecef_to_geodetic(ecef)
        out[:, 2] -= self.geoid_undulation_m
        return out

    def to_enu_xy(self, lat_deg: ArrayLike, lon_deg: ArrayLike, h_msl_m: ArrayLike = 0.0) -> np.ndarray:
        """WGS84 → ENU 水平座標 ``(N, 2)``；高程預設取 0（多邊形用；契約 §1）。"""
        return self.to_enu(lat_deg, lon_deg, h_msl_m)[:, :2]

    def enu_z_to_msl(self, xy: np.ndarray, z_enu_m: ArrayLike) -> np.ndarray:
        """ENU ``(xy, z)`` → MSL 高 ``(N,)``；以 :meth:`to_wgs84` 精確轉換（D42）。"""
        xy_arr = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
        z = np.broadcast_to(np.asarray(z_enu_m, dtype=np.float64), (xy_arr.shape[0],))
        return self.to_wgs84(np.column_stack([xy_arr, z]))[:, 2]

    def msl_to_enu_z(self, xy: np.ndarray, h_msl_m: ArrayLike, tol_m: float = 1e-7, max_iter: int = 8) -> np.ndarray:
        """MSL 高 → ENU z ``(N,)``（:meth:`enu_z_to_msl` 之反函數；互逆 < 1e-3 m，實際 ~1e-9）。

        沿 ENU 垂直線求 ``msl(z) = h`` 之根：定點迭代 ``z ← z + (h − msl(z))``。
        因 d(msl)/dz = cos(原點法向與當地法向夾角) ≈ 1（50 km 處 1 − 3e-5），
        每次迭代誤差縮小約 3e-5 倍，3–4 次即達機器精度。``nan`` 輸入原樣傳回。
        """
        xy_arr = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
        h = np.broadcast_to(np.asarray(h_msl_m, dtype=np.float64), (xy_arr.shape[0],)).astype(np.float64)
        z = h - self.h0_msl_m                                    # 初值：忽略曲率
        with np.errstate(invalid="ignore"):
            for _ in range(max_iter):
                dz = h - self.enu_z_to_msl(xy_arr, z)
                finite = np.isfinite(dz)
                if not finite.any():
                    break
                z = z + np.where(finite, dz, 0.0)
                if np.max(np.abs(dz[finite])) < tol_m:
                    break
        return z


# ---------------------------------------------------------------------------
# 雙線性內插（論文 Eq.(4)）
# ---------------------------------------------------------------------------
def _bilinear(grid: np.ndarray, r: np.ndarray, c: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    """在規則網格 ``grid[(rows, cols)]`` 於分數索引 ``(r, c)`` 做雙線性內插（論文 Eq.(4)）。

    範圍外（超出 ``[0, rows−1] × [0, cols−1]`` 逾 ``eps``）回 ``nan``；``nan`` 索引亦回 ``nan``。
    節點處精確等於節點值；對平面精確。
    """
    rows, cols = grid.shape
    r = np.asarray(r, dtype=np.float64)
    c = np.asarray(c, dtype=np.float64)
    with np.errstate(invalid="ignore"):
        valid = (r >= -eps) & (r <= rows - 1 + eps) & (c >= -eps) & (c <= cols - 1 + eps)
    r = np.where(valid, r, 0.0)
    c = np.where(valid, c, 0.0)
    r = np.clip(r, 0.0, rows - 1.0)
    c = np.clip(c, 0.0, cols - 1.0)
    r0 = np.minimum(np.floor(r).astype(np.intp), rows - 2)
    c0 = np.minimum(np.floor(c).astype(np.intp), cols - 2)
    fr = r - r0
    fc = c - c0
    g00 = grid[r0, c0].astype(np.float64)
    g01 = grid[r0, c0 + 1].astype(np.float64)
    g10 = grid[r0 + 1, c0].astype(np.float64)
    g11 = grid[r0 + 1, c0 + 1].astype(np.float64)
    out = ((1.0 - fr) * (1.0 - fc) * g00 + (1.0 - fr) * fc * g01
           + fr * (1.0 - fc) * g10 + fr * fc * g11)
    out = np.where(valid, out, np.nan)
    return out


# ---------------------------------------------------------------------------
# DemGrid（表 2「地形模型」）
# ---------------------------------------------------------------------------
@dataclass(frozen=True, slots=True)
class DemGrid:
    """WGS84 經緯規則網格 DEM/DSM（表 2「地形模型」；規格 §6.1）。

    * ``(lat0_deg, lon0_deg)`` = 左下角**節點**；``z_msl[i, j]`` 位於 ``(lat0 + i·dlat, lon0 + j·dlon)``
      （rows 沿 lat、cols 沿 lon）；float32、單位 m。
    * ``vertical_datum``：``"msl"``（預設）時 ``z_msl`` 即正高；``"ellipsoid"`` 時 ``z_msl`` 為橢球高，
      取用時以 ``z_msl − geoid_undulation_m`` 換成 MSL（本 DEM 自帶之起伏 N），再經 GeoFrame 進 ENU。
    * ``is_dsm=True``：D24——tree/building 類障礙不再燒入（由上層 KeepoutSet.build 判斷）。
    * ``sigma_z_m``：DEM 高程 1σ 誤差，進入 ``h_clr ≥ 3σ + 30`` 下限（D11；由 preflight 使用）。
    """

    lat0_deg: float
    lon0_deg: float
    dlat_deg: float
    dlon_deg: float
    z_msl: np.ndarray
    is_dsm: bool = False
    vertical_datum: Literal["msl", "ellipsoid"] = "msl"
    geoid_undulation_m: float = 0.0
    sigma_z_m: float | None = None

    def __post_init__(self) -> None:
        z = np.asarray(self.z_msl, dtype=np.float32)
        if z.ndim != 2 or z.shape[0] < 2 or z.shape[1] < 2:
            raise ValueError("DemGrid.z_msl must be a 2-D array with at least 2 rows and 2 cols")
        if not (self.dlat_deg > 0.0 and self.dlon_deg > 0.0):
            raise ValueError("DemGrid.dlat_deg / dlon_deg must be positive")
        if self.vertical_datum not in ("msl", "ellipsoid"):
            raise ValueError(f"DemGrid.vertical_datum must be 'msl' or 'ellipsoid', got {self.vertical_datum!r}")
        object.__setattr__(self, "z_msl", z)

    # -- 幾何 -----------------------------------------------------------------
    @property
    def rows(self) -> int:
        return int(self.z_msl.shape[0])

    @property
    def cols(self) -> int:
        return int(self.z_msl.shape[1])

    @property
    def lat_max_deg(self) -> float:
        return self.lat0_deg + (self.rows - 1) * self.dlat_deg

    @property
    def lon_max_deg(self) -> float:
        return self.lon0_deg + (self.cols - 1) * self.dlon_deg

    def bounds_wgs84(self) -> tuple[float, float, float, float]:
        """``(lat_min, lon_min, lat_max, lon_max)``（節點範圍）。"""
        return (self.lat0_deg, self.lon0_deg, self.lat_max_deg, self.lon_max_deg)

    def contains(self, lat_deg: ArrayLike, lon_deg: ArrayLike, eps_deg: float = 1e-12) -> np.ndarray:
        """點是否落在 DEM 節點範圍內（bool 陣列）。"""
        lat = np.asarray(lat_deg, dtype=np.float64)
        lon = np.asarray(lon_deg, dtype=np.float64)
        return ((lat >= self.lat0_deg - eps_deg) & (lat <= self.lat_max_deg + eps_deg)
                & (lon >= self.lon0_deg - eps_deg) & (lon <= self.lon_max_deg + eps_deg))

    # -- 取樣 -----------------------------------------------------------------
    def sample_msl(self, lat_deg: ArrayLike, lon_deg: ArrayLike) -> np.ndarray:
        """於 ``(lat, lon)`` 雙線性內插（論文 Eq.(4)）→ **MSL** 高 ``(N,)``；範圍外 ``nan``。

        ``vertical_datum="ellipsoid"`` 時已減去 ``geoid_undulation_m``。
        """
        lat = np.atleast_1d(np.asarray(lat_deg, dtype=np.float64))
        lon = np.atleast_1d(np.asarray(lon_deg, dtype=np.float64))
        lat, lon = np.broadcast_arrays(lat, lon)
        r = (lat - self.lat0_deg) / self.dlat_deg
        c = (lon - self.lon0_deg) / self.dlon_deg
        z = _bilinear(self.z_msl, r.ravel(), c.ravel()).reshape(lat.shape)
        if self.vertical_datum == "ellipsoid":
            z = z - self.geoid_undulation_m
        return z


# ---------------------------------------------------------------------------
# TerrainGrid（ENU 規則網格）
# ---------------------------------------------------------------------------
class TerrainGrid:
    """ENU 規則網格（步長 ``step_m``），雙線性內插（論文 Eq.(4)）。含障礙燒入（D24）。

    * 節點 ``z[j, i]`` 位於 ``(x0 + i·step, y0 + j·step)``（``(ny, nx)`` float32）；``z`` 為 **ENU z**
      （已含地球曲率下垂，見 :meth:`from_dem`）。``z_raw`` 為燒入前副本。
    * 所有取樣函式範圍外回 ``nan``；``corridor_*`` / ``*_in_polygon`` 若任一取樣點在範圍外亦回 ``nan``
      （地形未知即視為未知，不以部分資料估計）。
    * 額外屬性 ``sigma_z_m``、``is_dsm`` 自 DemGrid 轉載（供 preflight / KeepoutSet 使用）。
    """

    __slots__ = ("x0_m", "y0_m", "step_m", "nx", "ny", "z", "z_raw", "sigma_z_m", "is_dsm", "_grad", "_raw_view")

    def __init__(self, x0_m: float, y0_m: float, step_m: float, z: np.ndarray,
                 z_raw: np.ndarray | None = None, sigma_z_m: float | None = None, is_dsm: bool = False) -> None:
        arr = np.array(z, dtype=np.float32, copy=True)
        if arr.ndim != 2 or arr.shape[0] < 2 or arr.shape[1] < 2:
            raise ValueError("TerrainGrid.z must be a 2-D array with at least 2 rows and 2 cols")
        if not step_m > 0.0:
            raise ValueError("TerrainGrid.step_m must be positive")
        self.x0_m: float = float(x0_m)
        self.y0_m: float = float(y0_m)
        self.step_m: float = float(step_m)
        self.ny: int = int(arr.shape[0])
        self.nx: int = int(arr.shape[1])
        self.z: np.ndarray = arr
        self.z_raw: np.ndarray = arr.copy() if z_raw is None else np.array(z_raw, dtype=np.float32, copy=True)
        if self.z_raw.shape != arr.shape:
            raise ValueError("TerrainGrid.z_raw shape must match z")
        self.sigma_z_m: float | None = None if sigma_z_m is None else float(sigma_z_m)
        self.is_dsm: bool = bool(is_dsm)
        self._grad: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self._raw_view: "TerrainGrid | None" = None

    def __repr__(self) -> str:  # pragma: no cover - 顯示用
        return (f"TerrainGrid(x0_m={self.x0_m}, y0_m={self.y0_m}, step_m={self.step_m}, "
                f"nx={self.nx}, ny={self.ny})")

    # -- 建構 -----------------------------------------------------------------
    @classmethod
    def from_dem(cls, dem: DemGrid, frame: GeoFrame, step_m: float,
                 bounds_enu: tuple[float, float, float, float]) -> "TerrainGrid":
        """自 DEM 重取樣為 ENU 規則網格（規格 §6.2 步 4；``cfg.dem_step_m`` 預設 10 m）。

        ``bounds_enu = (xmin, ymin, xmax, ymax)``；網格自 ``(xmin, ymin)`` 起以 ``step_m`` 鋪至
        涵蓋 ``(xmax, ymax)``。每個節點 ``(x, y)``：

        1. 沿 ENU 垂直線找地表點：``(lat, lon) = to_wgs84([x, y, z_g])`` → DEM 雙線性取 MSL →
           ``z_g = msl_to_enu_z((x, y), msl)``，定點迭代至收斂（垂直線非當地法向，lat/lon 隨 z_g 微變）。
        2. 因此節點 ``z`` 為精確 ENU z（含曲率下垂；A9），且
           ``frame.enu_z_to_msl(xy, terrain.z_at(xy)) ≈ DEM MSL``。

        DEM 範圍外之節點為 ``nan``。
        """
        xmin, ymin, xmax, ymax = (float(v) for v in bounds_enu)
        if not (xmax > xmin and ymax > ymin):
            raise ValueError("bounds_enu must satisfy xmax > xmin and ymax > ymin")
        if not step_m > 0.0:
            raise ValueError("step_m must be positive")
        nx = int(math.ceil((xmax - xmin) / step_m - 1e-9)) + 1
        ny = int(math.ceil((ymax - ymin) / step_m - 1e-9)) + 1
        nx = max(nx, 2)
        ny = max(ny, 2)
        xs = xmin + np.arange(nx, dtype=np.float64) * step_m
        ys = ymin + np.arange(ny, dtype=np.float64) * step_m
        gx, gy = np.meshgrid(xs, ys)                      # (ny, nx)
        xy_all = np.column_stack([gx.ravel(), gy.ravel()])
        z_all = np.empty(xy_all.shape[0], dtype=np.float64)
        for s in range(0, xy_all.shape[0], _CHUNK):
            xy = xy_all[s:s + _CHUNK]
            z = np.zeros(xy.shape[0], dtype=np.float64)
            with np.errstate(invalid="ignore"):
                for _ in range(4):
                    wgs = frame.to_wgs84(np.column_stack([xy, z]))
                    msl = dem.sample_msl(wgs[:, 0], wgs[:, 1])
                    z_new = frame.msl_to_enu_z(xy, msl)
                    finite = np.isfinite(z_new) & np.isfinite(z)
                    delta = np.abs(z_new[finite] - z[finite]).max() if finite.any() else 0.0
                    z = z_new
                    if delta < 1e-4:
                        break
            z_all[s:s + _CHUNK] = z
        grid = z_all.reshape(ny, nx).astype(np.float32)
        return cls(xmin, ymin, step_m, grid, sigma_z_m=dem.sigma_z_m, is_dsm=dem.is_dsm)

    def copy(self) -> "TerrainGrid":
        """深複製（燒入前後對照、情境分支用）。"""
        return TerrainGrid(self.x0_m, self.y0_m, self.step_m, self.z, z_raw=self.z_raw,
                           sigma_z_m=self.sigma_z_m, is_dsm=self.is_dsm)

    def raw_view(self) -> "TerrainGrid":
        """**自然地形**（燒入前 ``z_raw``）之取樣視圖：``z = z_raw`` 的 :class:`TerrainGrid`（快取）。

        AGL 語意（``h_clr`` / ``h_minfw`` / ``h_ceil`` / 條帶 ``h_min,l``）一律指「地面之上」，
        障礙淨空另由 ``z_obs = z_top + b_v``（規格 §11.1）與 keep-out 2.5D 測試負責；若直接用燒入後
        ``z`` 量 AGL，等同要求在障礙頂之上再加 ``h_minfw``（規格未要求），並使空域上限 ``z_g^0 + h_ceil``
        在障礙旁相互矛盾（此為 hills 情境 ``above_ceiling`` 之根因）。
        `z_raw` 未被燒入修改，故視圖可安全快取；`burn()` 不影響其內容。
        """
        if self._raw_view is None:
            if self.z is self.z_raw or np.array_equal(self.z, self.z_raw):
                self._raw_view = self
            else:
                self._raw_view = TerrainGrid(self.x0_m, self.y0_m, self.step_m, self.z_raw, z_raw=self.z_raw,
                                             sigma_z_m=self.sigma_z_m, is_dsm=self.is_dsm)
        return self._raw_view

    # -- 幾何 -----------------------------------------------------------------
    def bounds(self) -> tuple[float, float, float, float]:
        """``(xmin, ymin, xmax, ymax)``（節點範圍）。"""
        return (self.x0_m, self.y0_m,
                self.x0_m + (self.nx - 1) * self.step_m,
                self.y0_m + (self.ny - 1) * self.step_m)

    @property
    def xs(self) -> np.ndarray:
        """節點 x 座標 ``(nx,)``。"""
        return self.x0_m + np.arange(self.nx, dtype=np.float64) * self.step_m

    @property
    def ys(self) -> np.ndarray:
        """節點 y 座標 ``(ny,)``。"""
        return self.y0_m + np.arange(self.ny, dtype=np.float64) * self.step_m

    def _frac_index(self, xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        arr = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
        r = (arr[:, 1] - self.y0_m) / self.step_m
        c = (arr[:, 0] - self.x0_m) / self.step_m
        return r, c

    # -- 取樣 -----------------------------------------------------------------
    def z_at(self, xy: np.ndarray) -> np.ndarray:
        """``(N, 2)`` → ENU 地表高 ``(N,)`` m（雙線性，論文 Eq.(4)）；範圍外 ``nan``。"""
        r, c = self._frac_index(xy)
        return _bilinear(self.z, r, c)

    def _gradient(self, key: str) -> tuple[np.ndarray, np.ndarray]:
        cached = self._grad.get(key)
        if cached is None:
            grid = self.z_raw if key == "raw" else self.z
            gy, gx = np.gradient(grid.astype(np.float64), self.step_m)    # axis0 = y, axis1 = x
            cached = (gx.astype(np.float32), gy.astype(np.float32))
            self._grad[key] = cached
        return cached

    def slope_at(self, xy: np.ndarray, raw: bool = True) -> np.ndarray:
        """地面坡度 ``(N,)`` rad（梯度模 ``atan|∇z|``；中央差分梯度場之雙線性取樣）。

        預設以 ``z_raw``（燒入前之自然地形）計算：障礙燒入產生的人工陡壁不應被視為地形坡度
        （站點檢查 §6.2 步 5、方向評分 §7.3 皆指 DEM 坡度）。``raw=False`` 則用燒入後之 ``z``。
        範圍外 ``nan``。
        """
        gx, gy = self._gradient("raw" if raw else "z")
        r, c = self._frac_index(xy)
        sx = _bilinear(gx, r, c)
        sy = _bilinear(gy, r, c)
        return np.arctan(np.hypot(sx, sy))

    # -- 走廊 -----------------------------------------------------------------
    def _corridor_z(self, path_xy: np.ndarray, theta_rad: ArrayLike, half_width_m: ArrayLike,
                    dv_m: float, offset_m: float) -> np.ndarray:
        """回傳 ``(N, M)``：每個航跡樣本沿右法向 ``n̂(θ) = (cos θ, −sin θ)`` 於
        ``v ∈ [offset − hw, offset + hw]`` 之地形 z（間距 ≤ dv_m；含兩端點）。"""
        path = np.asarray(path_xy, dtype=np.float64).reshape(-1, 2)
        n_pts = path.shape[0]
        theta = np.broadcast_to(np.asarray(theta_rad, dtype=np.float64), (n_pts,))
        hw_arr = np.asarray(half_width_m, dtype=np.float64)
        if hw_arr.ndim > 1:
            hw_arr = hw_arr.reshape(-1)
        hw = np.broadcast_to(hw_arr, (n_pts,))
        if not dv_m > 0.0:
            raise ValueError("dv_m must be positive")
        if np.any(hw < 0.0):
            raise ValueError("half_width_m must be non-negative")
        hw_max = float(hw.max()) if n_pts else 0.0
        m = int(math.ceil(2.0 * hw_max / dv_m - 1e-9)) + 1 if hw_max > 0.0 else 1
        t = np.linspace(-1.0, 1.0, m) if m > 1 else np.zeros(1)
        v = float(offset_m) + hw[:, None] * t[None, :]                       # (N, M)
        n_hat = np.stack([np.cos(theta), -np.sin(theta)], axis=-1)           # (N, 2) 右法向
        pts = path[:, None, :] + v[:, :, None] * n_hat[:, None, :]           # (N, M, 2)
        return self.z_at(pts.reshape(-1, 2)).reshape(n_pts, m)

    def corridor_max(self, path_xy: np.ndarray, theta_rad: np.ndarray, half_width_m: np.ndarray | float,
                     dv_m: float, offset_m: float = 0.0) -> np.ndarray:
        """走廊內最高地形 ``z_g^max`` ``(N,)``（規格 §11.1）。

        對每個樣本 ``i``（方位 ``θ_i``），沿右法向 ``n̂(θ_i) = (cos θ_i, −sin θ_i)`` 於
        ``v ∈ [offset − hw_i, offset + hw_i]`` 以間距 ≤ ``dv_m`` 取樣（含端點）取最大 z。
        ``half_width_m`` 可為純量或 ``(N,)``；``hw = 0`` 時恆等於 ``z_at(path + offset·n̂)``。
        任一取樣點在範圍外 → 該樣本 ``nan``。
        """
        zc = self._corridor_z(path_xy, theta_rad, half_width_m, dv_m, offset_m)
        return zc.max(axis=1)

    def corridor_min(self, path_xy: np.ndarray, theta_rad: np.ndarray, half_width_m: np.ndarray | float,
                     dv_m: float, offset_m: float = 0.0) -> np.ndarray:
        """走廊內最低地形 ``z_g^min`` ``(N,)``；取樣規則同 :meth:`corridor_max`。"""
        zc = self._corridor_z(path_xy, theta_rad, half_width_m, dv_m, offset_m)
        return zc.min(axis=1)

    # -- 多邊形 ---------------------------------------------------------------
    def _bbox_indices(self, bounds: tuple[float, float, float, float]) -> tuple[int, int, int, int] | None:
        minx, miny, maxx, maxy = bounds
        i0 = max(0, int(math.floor((minx - self.x0_m) / self.step_m)))
        i1 = min(self.nx - 1, int(math.ceil((maxx - self.x0_m) / self.step_m)))
        j0 = max(0, int(math.floor((miny - self.y0_m) / self.step_m)))
        j1 = min(self.ny - 1, int(math.ceil((maxy - self.y0_m) / self.step_m)))
        if i1 < i0 or j1 < j0:
            return None
        return i0, i1, j0, j1

    @staticmethod
    def _parts(geom: shapely.Geometry) -> list[shapely.Polygon]:
        if geom is None or geom.is_empty:
            return []
        parts = [g for g in shapely.get_parts(geom) if isinstance(g, shapely.Polygon) and not g.is_empty]
        return parts

    def _poly_values(self, poly: shapely.Polygon) -> np.ndarray:
        """多邊形內節點值 ∪ 邊界（含環頂點，段長 ≤ step/2）取樣值。

        雙線性曲面在每個胞格內之極值出現於胞格角點；多邊形內部區域之極值因此落在
        內部節點或多邊形邊界上，故此取樣集之 max/min 即多邊形內連續曲面之 max/min（至 step/2 取樣）。
        """
        vals: list[np.ndarray] = []
        for part in self._parts(poly):
            idx = self._bbox_indices(part.bounds)
            if idx is not None:
                i0, i1, j0, j1 = idx
                gx, gy = np.meshgrid(self.x0_m + np.arange(i0, i1 + 1) * self.step_m,
                                     self.y0_m + np.arange(j0, j1 + 1) * self.step_m)
                inside = shapely.contains_xy(part, gx.ravel(), gy.ravel())
                if inside.any():
                    vals.append(self.z[j0:j1 + 1, i0:i1 + 1].ravel()[inside].astype(np.float64))
            boundary = shapely.segmentize(part.boundary, self.step_m / 2.0)
            coords = shapely.get_coordinates(boundary)
            if coords.shape[0]:
                vals.append(self.z_at(coords))
        if not vals:
            return np.empty(0, dtype=np.float64)
        return np.concatenate(vals)

    def max_in_polygon(self, poly: shapely.Polygon) -> float:
        """多邊形內最高 ENU 地形（節點 ∪ 邊界取樣）；空／範圍外 → ``nan``。"""
        vals = self._poly_values(poly)
        return float(np.max(vals)) if vals.size else float("nan")

    def min_in_polygon(self, poly: shapely.Polygon) -> float:
        """多邊形內最低 ENU 地形（節點 ∪ 邊界取樣）；空／範圍外 → ``nan``。"""
        vals = self._poly_values(poly)
        return float(np.min(vals)) if vals.size else float("nan")

    # -- 障礙燒入 -------------------------------------------------------------
    def burn(self, polys: list[shapely.Polygon], tops_z: list[float]) -> None:
        """障礙燒入：多邊形內 ``z ← max(z, top)``（D24；``polys`` 已含水平緩衝、``tops_z`` 為 ENU z）。

        為使雙線性曲面在多邊形**內部每一點**皆 ≥ top（而非僅節點），燒入範圍取
        ``poly ⊕ disc(step·√2)``——多邊形內任一點之四個相鄰節點皆在此範圍內。
        單調不減：``z_raw`` 不變；重複燒入或較低之 top 不會降低 ``z``。
        """
        if len(polys) != len(tops_z):
            raise ValueError("polys and tops_z must have the same length")
        dil = self.step_m * math.sqrt(2.0) + 1e-6
        changed = False
        for poly, top in zip(polys, tops_z):
            if poly is None or top is None or not np.isfinite(top):
                continue
            for part in self._parts(shapely.buffer(poly, dil)):
                idx = self._bbox_indices(part.bounds)
                if idx is None:
                    continue
                i0, i1, j0, j1 = idx
                gx, gy = np.meshgrid(self.x0_m + np.arange(i0, i1 + 1) * self.step_m,
                                     self.y0_m + np.arange(j0, j1 + 1) * self.step_m)
                inside = shapely.contains_xy(part, gx.ravel(), gy.ravel()).reshape(gx.shape)
                if not inside.any():
                    continue
                sub = self.z[j0:j1 + 1, i0:i1 + 1]          # view → 就地修改
                sub[inside] = np.maximum(sub[inside], np.float32(top))
                changed = True
        if changed:
            self._grad.pop("z", None)
            self._raw_view = None                 # 燒入後 z ≠ z_raw：自然地形視圖需重建
