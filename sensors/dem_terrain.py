"""
DEM 地形雙線性插值管理器
============================
對應論文《Multiple fixed-wing UAVs collaborative coverage 3D path planning
method for complex areas》Section 2.2.2 (Eq. 4)。

支援 GeoTIFF (.tif) 格式的 DEM 數據，提供雙線性插值查詢。
當無 DEM 數據時可退化為常數海拔或梯度模擬。
"""

from __future__ import annotations

import math
import logging
from typing import Optional, Tuple, List

import numpy as np

from sensors.terrain_manager import TerrainManagerBase

logger = logging.getLogger(__name__)


class DEMTerrainManager(TerrainManagerBase):
    """
    基於 DEM 資料的地形管理器，支援雙線性插值。

    使用方式:
        dem = DEMTerrainManager()
        if dem.load_dem("path/to/dem.tif"):
            elev = dem.get_elevation(36.5, 93.5)  # 查詢海拔
        # 也可作為 terrain_func 傳入 AltitudePlanner
        alt_planner.plan_altitude(path_2d, terrain_func=dem.get_elevation)
    """

    def __init__(self, default_elevation: float = 0.0):
        self._default_elev = default_elevation
        self._data: Optional[np.ndarray] = None
        self._lat_min = 0.0
        self._lat_max = 0.0
        self._lon_min = 0.0
        self._lon_max = 0.0
        self._rows = 0
        self._cols = 0
        self._dlat = 0.0
        self._dlon = 0.0
        self._loaded = False
        self._nodata = -9999.0
        logger.info(f"DEMTerrainManager initialized (default_elev={default_elevation})")

    # ── DEM 載入 ──────────────────────────────────────────

    def load_dem(self, file_path: str) -> bool:
        """
        載入 GeoTIFF DEM 檔案。

        嘗試順序:
        1. rasterio (推薦)
        2. GDAL
        3. 簡易 numpy 格式 (.npy) 搭配 metadata

        參數:
            file_path: DEM 檔案路徑

        返回:
            True 表示載入成功
        """
        # ── 嘗試 rasterio ──
        try:
            return self._load_rasterio(file_path)
        except ImportError:
            pass
        except Exception as e:
            logger.warning(f"rasterio 載入失敗: {e}")

        # ── 嘗試 GDAL ──
        try:
            return self._load_gdal(file_path)
        except ImportError:
            pass
        except Exception as e:
            logger.warning(f"GDAL 載入失敗: {e}")

        # ── 嘗試 numpy (.npy + .json metadata) ──
        try:
            return self._load_numpy(file_path)
        except Exception as e:
            logger.warning(f"numpy 載入失敗: {e}")

        logger.error(f"無法載入 DEM: {file_path} (需安裝 rasterio 或 GDAL)")
        return False

    def _load_rasterio(self, file_path: str) -> bool:
        """使用 rasterio 載入 GeoTIFF"""
        import rasterio
        with rasterio.open(file_path) as src:
            self._data = src.read(1).astype(np.float64)
            transform = src.transform
            self._rows, self._cols = self._data.shape
            self._nodata = src.nodata if src.nodata is not None else -9999.0
            # 從 affine transform 取得邊界
            self._lon_min = transform.c
            self._lat_max = transform.f
            self._dlon = transform.a
            self._dlat = transform.e  # 負值（從上到下）
            self._lon_max = self._lon_min + self._cols * self._dlon
            self._lat_min = self._lat_max + self._rows * self._dlat
        self._loaded = True
        logger.info(
            f"DEM 載入成功 (rasterio): {self._rows}×{self._cols}, "
            f"lat=[{self._lat_min:.4f}, {self._lat_max:.4f}], "
            f"lon=[{self._lon_min:.4f}, {self._lon_max:.4f}]"
        )
        return True

    def _load_gdal(self, file_path: str) -> bool:
        """使用 GDAL 載入 GeoTIFF"""
        from osgeo import gdal
        ds = gdal.Open(file_path)
        if ds is None:
            raise RuntimeError(f"GDAL 無法開啟 {file_path}")
        band = ds.GetRasterBand(1)
        self._data = band.ReadAsArray().astype(np.float64)
        self._rows, self._cols = self._data.shape
        self._nodata = band.GetNoDataValue() or -9999.0
        gt = ds.GetGeoTransform()
        self._lon_min = gt[0]
        self._dlon = gt[1]
        self._lat_max = gt[3]
        self._dlat = gt[5]
        self._lon_max = self._lon_min + self._cols * self._dlon
        self._lat_min = self._lat_max + self._rows * self._dlat
        ds = None
        self._loaded = True
        logger.info(
            f"DEM 載入成功 (GDAL): {self._rows}×{self._cols}, "
            f"lat=[{self._lat_min:.4f}, {self._lat_max:.4f}], "
            f"lon=[{self._lon_min:.4f}, {self._lon_max:.4f}]"
        )
        return True

    def _load_numpy(self, file_path: str) -> bool:
        """
        載入 numpy 格式 DEM：
        - .npy 檔案 + 同名 .json 檔案包含 metadata
          {"lat_min": ..., "lat_max": ..., "lon_min": ..., "lon_max": ...}
        """
        import json
        from pathlib import Path
        npy_path = Path(file_path)
        meta_path = npy_path.with_suffix('.json')
        if not npy_path.exists():
            raise FileNotFoundError(f"找不到 {npy_path}")
        self._data = np.load(str(npy_path)).astype(np.float64)
        self._rows, self._cols = self._data.shape
        if meta_path.exists():
            with open(meta_path, 'r') as f:
                meta = json.load(f)
            self._lat_min = meta['lat_min']
            self._lat_max = meta['lat_max']
            self._lon_min = meta['lon_min']
            self._lon_max = meta['lon_max']
        else:
            raise FileNotFoundError(f"找不到 metadata: {meta_path}")
        self._dlat = -(self._lat_max - self._lat_min) / self._rows
        self._dlon = (self._lon_max - self._lon_min) / self._cols
        self._loaded = True
        logger.info(
            f"DEM 載入成功 (numpy): {self._rows}×{self._cols}"
        )
        return True

    # ── 海拔查詢（雙線性插值，論文 Eq. 4）──────────────

    def get_elevation(self, lat: float, lon: float) -> float:
        """
        取得指定座標的地面海拔（雙線性插值）。

        論文 Eq. 4:
            z_g = (x2-xi)(y2-yi)/(dx*dy) * z1
                + (xi-x1)(y2-yi)/(dx*dy) * z2
                + (x2-xi)(yi-y1)/(dx*dy) * z3
                + (xi-x1)(yi-y1)/(dx*dy) * z4

        參數:
            lat: 緯度（度）
            lon: 經度（度）

        返回:
            海拔高度 (m, AMSL)
        """
        if not self._loaded or self._data is None:
            return self._default_elev

        # 轉為行列座標（連續值）
        row_f = (self._lat_max - lat) / abs(self._dlat)
        col_f = (lon - self._lon_min) / abs(self._dlon)

        # 邊界檢查
        if row_f < 0 or row_f >= self._rows - 1 or col_f < 0 or col_f >= self._cols - 1:
            return self._default_elev

        r0 = int(row_f)
        c0 = int(col_f)
        r1 = min(r0 + 1, self._rows - 1)
        c1 = min(c0 + 1, self._cols - 1)

        # 子像素偏移
        dr = row_f - r0
        dc = col_f - c0

        # 四角值
        z00 = self._data[r0, c0]
        z01 = self._data[r0, c1]
        z10 = self._data[r1, c0]
        z11 = self._data[r1, c1]

        # 處理 nodata
        for z in (z00, z01, z10, z11):
            if z == self._nodata or np.isnan(z):
                return self._default_elev

        # 雙線性插值
        z = (z00 * (1 - dc) * (1 - dr)
             + z01 * dc * (1 - dr)
             + z10 * (1 - dc) * dr
             + z11 * dc * dr)

        return float(z)

    def check_collision(
        self, lat: float, lon: float, alt_amsl: float,
        buffer_m: float = 10.0,
    ) -> bool:
        """
        檢查是否會撞地。

        參數:
            lat, lon: 座標
            alt_amsl: 飛行高度 (AMSL, m)
            buffer_m: 安全餘量 (m)

        返回:
            True 表示會碰撞
        """
        ground = self.get_elevation(lat, lon)
        return alt_amsl < (ground + buffer_m)

    def get_elevation_grid(
        self,
        lat_range: Tuple[float, float],
        lon_range: Tuple[float, float],
        resolution: float = 100.0,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        取得指定區域的海拔網格（供視覺化使用）。

        參數:
            lat_range: (lat_min, lat_max)
            lon_range: (lon_min, lon_max)
            resolution: 網格間距 (m)

        返回:
            (lats, lons, elevations) 三個 2D numpy 陣列
        """
        lat_step = resolution / 111111.0
        lon_step = resolution / (111111.0 * math.cos(math.radians(
            (lat_range[0] + lat_range[1]) / 2
        )))

        lats = np.arange(lat_range[0], lat_range[1], lat_step)
        lons = np.arange(lon_range[0], lon_range[1], lon_step)
        lat_grid, lon_grid = np.meshgrid(lats, lons, indexing='ij')
        elev_grid = np.zeros_like(lat_grid)

        for i in range(lat_grid.shape[0]):
            for j in range(lat_grid.shape[1]):
                elev_grid[i, j] = self.get_elevation(lat_grid[i, j], lon_grid[i, j])

        return lat_grid, lon_grid, elev_grid

    @property
    def is_loaded(self) -> bool:
        return self._loaded

    @property
    def bounds(self) -> Optional[Tuple[float, float, float, float]]:
        """返回 (lat_min, lat_max, lon_min, lon_max) 或 None"""
        if not self._loaded:
            return None
        return (self._lat_min, self._lat_max, self._lon_min, self._lon_max)
