"""
下載虎尾鎮 SRTM DEM 並存為 GeoTIFF
覆蓋範圍: 120.39°E – 120.49°E, 23.67°N – 23.75°N
解析度: SRTM3 (~90m)
"""

import numpy as np
import json
from pathlib import Path

# 虎尾鎮範圍
BOUNDS = {
    'lon_min': 120.39,
    'lon_max': 120.49,
    'lat_min': 23.67,
    'lat_max': 23.75,
}
RESOLUTION = 0.001  # 約 100m 格點

OUTPUT_TIF = "huwei_dem.tif"
OUTPUT_NPY = "huwei_dem.npy"
OUTPUT_JSON = "huwei_dem.json"


def download_via_srtm():
    """使用 srtm.py 套件下載 SRTM3 資料"""
    import srtm
    print("正在從 NASA SRTM 下載虎尾 DEM（首次需要時間）...")
    data = srtm.get_data()

    lats = np.arange(BOUNDS['lat_max'], BOUNDS['lat_min'], -RESOLUTION)
    lons = np.arange(BOUNDS['lon_min'], BOUNDS['lon_max'],  RESOLUTION)

    rows = len(lats)
    cols = len(lons)
    grid = np.zeros((rows, cols), dtype=np.float32)

    for i, lat in enumerate(lats):
        for j, lon in enumerate(lons):
            elev = data.get_elevation(lat, lon)
            grid[i, j] = elev if elev is not None else 0.0
        if i % 10 == 0:
            print(f"  進度: {i}/{rows} 列", end='\r')

    print(f"\n下載完成: {rows}×{cols} 格點")
    return grid, lats, lons


def save_geotiff(grid, lats, lons):
    """存為 GeoTIFF（需要 rasterio）"""
    try:
        import rasterio
        from rasterio.transform import from_bounds
        from rasterio.crs import CRS

        transform = from_bounds(
            BOUNDS['lon_min'], BOUNDS['lat_min'],
            BOUNDS['lon_max'], BOUNDS['lat_max'],
            grid.shape[1], grid.shape[0]
        )
        with rasterio.open(
            OUTPUT_TIF, 'w',
            driver='GTiff',
            height=grid.shape[0],
            width=grid.shape[1],
            count=1,
            dtype=grid.dtype,
            crs=CRS.from_epsg(4326),
            transform=transform,
            nodata=-9999,
        ) as dst:
            dst.write(grid, 1)
        print(f"✅ GeoTIFF 已儲存: {OUTPUT_TIF}")
        return True
    except Exception as e:
        print(f"⚠️  GeoTIFF 儲存失敗 ({e})，改用 numpy 格式")
        return False


def save_numpy(grid):
    """備用：存為 numpy + JSON（不需 rasterio）"""
    np.save(OUTPUT_NPY, grid)
    meta = {
        'lat_min': BOUNDS['lat_min'],
        'lat_max': BOUNDS['lat_max'],
        'lon_min': BOUNDS['lon_min'],
        'lon_max': BOUNDS['lon_max'],
    }
    with open(OUTPUT_JSON, 'w') as f:
        json.dump(meta, f, indent=2)
    print(f"✅ Numpy DEM 已儲存: {OUTPUT_NPY} + {OUTPUT_JSON}")


def verify(output_file):
    """驗證 DEMTerrainManager 能正確讀取"""
    import sys
    sys.path.insert(0, str(Path(__file__).parent))
    from sensors.dem_terrain import DEMTerrainManager

    dem = DEMTerrainManager()
    ok = dem.load_dem(output_file)
    if not ok:
        print("❌ DEMTerrainManager 讀取失敗")
        return

    test_points = [
        (23.7074, 120.4387, "虎尾鎮中心"),
        (23.6950, 120.4200, "虎尾西側"),
        (23.7200, 120.4600, "虎尾東側"),
    ]
    print("\n📍 高程驗證:")
    for lat, lon, name in test_points:
        elev = dem.get_elevation(lat, lon)
        print(f"  {name} ({lat:.4f}, {lon:.4f}): {elev:.1f} m")


if __name__ == '__main__':
    grid, lats, lons = download_via_srtm()

    # 優先存 GeoTIFF，失敗則存 numpy
    if save_geotiff(grid, lats, lons):
        verify(OUTPUT_TIF)
    else:
        save_numpy(grid)
        verify(OUTPUT_NPY)
