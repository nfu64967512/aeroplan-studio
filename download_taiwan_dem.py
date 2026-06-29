"""
下載全台 DEM 資料 → 合成單一 GeoTIFF

資料來源：NASA SRTM3（3 arc-sec ≈ 90 m），公開無需登入。
從多個鏡像下載 1°×1° 的 .hgt.zip tiles，本機合成（mosaic）後裁切到 BBox 範圍，
輸出 GeoTIFF 供 sensors/dem_terrain.py 與 3D Cesium 地圖直接讀取。

使用方式
--------
    python download_taiwan_dem.py                  # 全台主島（預設）
    python download_taiwan_dem.py --full           # 含澎湖、蘭嶼、綠島、釣魚台
    python download_taiwan_dem.py --bbox 119.3 21.8 122.1 25.5  # 自訂
    python download_taiwan_dem.py --output my.tif  # 自訂輸出檔名

進階
----
    --product SRTM1   ：30 m 高解析度（檔案較大；需可用鏡像）
    --no-cache        ：不快取 tile（預設快取於 data/cache/srtm3/）
    --dry-run         ：只列出將下載的 tile，不真正下載

執行需要套件： numpy, rasterio, requests, tqdm（缺 tqdm 也能跑，無進度條）
"""

from __future__ import annotations

import argparse
import io
import sys
import time
import zipfile
from pathlib import Path
from typing import Optional

import numpy as np

try:
    from tqdm import tqdm
except ImportError:
    def tqdm(it=None, **kw):
        return it if it is not None else None

import requests


# ══════════════════════════════════════════════════════════════════════
#  常數與預設範圍
# ══════════════════════════════════════════════════════════════════════

# 全台主島（不含外島），覆蓋本島 + 部分海岸線
BBOX_MAIN = dict(lon_min=119.9, lon_max=122.05, lat_min=21.8, lat_max=25.35)

# 含澎湖、金門外側、馬祖、蘭嶼、綠島、釣魚台等
BBOX_FULL = dict(lon_min=118.1, lon_max=124.6, lat_min=21.6, lat_max=26.4)

# 1°×1° SRTM3 tile：1201×1201 int16 big-endian (3 arc-sec = ~90 m)
# 1°×1° SRTM1 tile：3601×3601 int16 big-endian (1 arc-sec = ~30 m)
SRTM3_PX = 1201
SRTM1_PX = 3601
SRTM_NODATA = -32768

# 公開鏡像（無需登入）— 失敗自動 fallback 到下一個
# 注意：AWS Skadi (/skadi/) 一律服務 SRTM1（30m），不在 SRTM3 清單。
SRTM3_MIRRORS = [
    'https://srtm.kurviger.de/SRTM3/Eurasia/{name}.hgt.zip',
    'https://step.esa.int/auxdata/dem/SRTMGL3/{name}.SRTMGL3.hgt.zip',
]
SRTM1_MIRRORS = [
    'https://step.esa.int/auxdata/dem/SRTMGL1/{name}.SRTMGL1.hgt.zip',
    # Mapzen / AWS Skadi（無需登入，gz 而非 zip，3601×3601）
    'https://elevation-tiles-prod.s3.amazonaws.com/skadi/{prefix}/{name}.hgt.gz',
]

CACHE_DIR_DEFAULT = Path(__file__).parent / 'data' / 'cache' / 'srtm'


# ══════════════════════════════════════════════════════════════════════
#  Tile 工具
# ══════════════════════════════════════════════════════════════════════

def tile_name(lat: int, lon: int) -> str:
    """SRTM tile 命名規則：N23E120 表 lat 23~24, lon 120~121。"""
    NS = 'N' if lat >= 0 else 'S'
    EW = 'E' if lon >= 0 else 'W'
    return f'{NS}{abs(lat):02d}{EW}{abs(lon):03d}'


def tiles_for_bbox(bbox: dict) -> list[tuple[int, int]]:
    """回傳覆蓋 bbox 的所有 1°×1° tile (lat, lon) 列表。"""
    lat0 = int(np.floor(bbox['lat_min']))
    lat1 = int(np.ceil(bbox['lat_max']))
    lon0 = int(np.floor(bbox['lon_min']))
    lon1 = int(np.ceil(bbox['lon_max']))
    tiles = []
    for lat in range(lat0, lat1):
        for lon in range(lon0, lon1):
            tiles.append((lat, lon))
    return tiles


# ══════════════════════════════════════════════════════════════════════
#  HTTP 下載 + 解壓
# ══════════════════════════════════════════════════════════════════════

def _read_hgt_buffer(buf: bytes, expected_px: int) -> np.ndarray:
    """從原始 hgt 二進位資料解析為 (px, px) int16 陣列。"""
    expected_bytes = expected_px * expected_px * 2
    if len(buf) != expected_bytes:
        raise ValueError(
            f'HGT 大小不符：期望 {expected_bytes} bytes ({expected_px}×{expected_px}), '
            f'實得 {len(buf)} bytes'
        )
    return np.frombuffer(buf, dtype='>i2').reshape(expected_px, expected_px)


def _decompress_response(content: bytes, name: str) -> bytes:
    """支援 .zip 與 .gz 壓縮格式，回傳原始 .hgt bytes。"""
    if content[:2] == b'PK':   # zip
        with zipfile.ZipFile(io.BytesIO(content)) as zf:
            inner = [n for n in zf.namelist() if n.lower().endswith('.hgt')]
            if not inner:
                raise ValueError(f'{name}.zip 內無 .hgt 檔案')
            with zf.open(inner[0]) as f:
                return f.read()
    if content[:2] == b'\x1f\x8b':  # gzip
        import gzip
        return gzip.decompress(content)
    raise ValueError(f'{name}: 未知壓縮格式 (magic={content[:4]!r})')


def download_tile(lat: int, lon: int, mirrors: list[str],
                   expected_px: int,
                   cache_dir: Optional[Path] = None,
                   timeout: float = 30.0) -> Optional[np.ndarray]:
    """
    下載單一 SRTM tile 並回傳 (px, px) int16 陣列。
    失敗（含海洋區無此 tile）回傳 None。
    """
    name = tile_name(lat, lon)
    prefix = f'{"N" if lat >= 0 else "S"}{abs(lat):02d}'

    # 1) 檢查正快取（已下載成功）與負快取（已知海洋/缺資料）
    if cache_dir is not None:
        cache_file = cache_dir / f'{name}.hgt.npy'
        miss_file  = cache_dir / f'{name}.miss'
        if cache_file.exists():
            return np.load(cache_file)
        if miss_file.exists():
            return None

    # 2) 依序嘗試各鏡像
    for url_tpl in mirrors:
        url = url_tpl.format(name=name, prefix=prefix)
        try:
            r = requests.get(url, timeout=timeout)
            if r.status_code != 200:
                continue
            raw = _decompress_response(r.content, name)
            arr = _read_hgt_buffer(raw, expected_px)
            if cache_dir is not None:
                cache_dir.mkdir(parents=True, exist_ok=True)
                np.save(cache_file, arr)
            return arr
        except Exception:
            continue

    # 所有鏡像皆失敗 → 寫入負快取避免下次重複嘗試。
    # 若是暫時的網路問題，刪除 .miss 檔即可重試（或用 --no-cache）。
    if cache_dir is not None:
        cache_dir.mkdir(parents=True, exist_ok=True)
        miss_file.touch()
    return None


# ══════════════════════════════════════════════════════════════════════
#  Mosaic + 裁切
# ══════════════════════════════════════════════════════════════════════

def build_mosaic(bbox: dict, tiles: list[tuple[int, int]],
                  product: str = 'SRTM3',
                  cache_dir: Optional[Path] = None) -> tuple[np.ndarray, dict]:
    """
    下載所有 tile 並合成單一 raster。
    回傳 (mosaic_array, meta)，其中 meta 包含實際輸出 bbox 與 transform 用。
    """
    if product == 'SRTM3':
        px = SRTM3_PX
        mirrors = SRTM3_MIRRORS
    elif product == 'SRTM1':
        px = SRTM1_PX
        mirrors = SRTM1_MIRRORS
    else:
        raise ValueError(f'未知 product: {product}')

    lat_min = min(t[0] for t in tiles)
    lat_max = max(t[0] for t in tiles) + 1
    lon_min = min(t[1] for t in tiles)
    lon_max = max(t[1] for t in tiles) + 1
    n_lat = lat_max - lat_min
    n_lon = lon_max - lon_min

    # SRTM tile 邊界重疊 1 px → mosaic 寬高 = n × (px-1) + 1
    H = n_lat * (px - 1) + 1
    W = n_lon * (px - 1) + 1
    mosaic = np.full((H, W), SRTM_NODATA, dtype=np.int16)
    # 追蹤哪些區塊真的有下載到 tile（用於區分「山頂 void」與「海洋」）
    downloaded = np.zeros((H, W), dtype=bool)

    n_ok = 0
    n_skip = 0
    iterable = tqdm(tiles, desc=f'下載 {product} tiles', unit='tile') or tiles
    for (tlat, tlon) in iterable:
        arr = download_tile(tlat, tlon, mirrors, px, cache_dir=cache_dir)
        if arr is None:
            n_skip += 1
            continue
        # SRTM hgt: 第 0 列在最北（lat_max-1 那格的頂端），列號往南遞增
        row_off = (lat_max - 1 - tlat) * (px - 1)
        col_off = (tlon - lon_min) * (px - 1)
        mosaic[row_off:row_off + px, col_off:col_off + px] = arr
        downloaded[row_off:row_off + px, col_off:col_off + px] = True
        n_ok += 1

    print(f'\n  [OK] 已下載 {n_ok} tiles，{n_skip} 個跳過（海洋/缺資料）')

    # ── 處理 nodata：分兩種狀況 ──────────────────────────────────────
    # 1) 內陸 void（SRTM3 山頂、湖面等）：用最近鄰補，避免高山讀到 0
    # 2) 海洋（沒下載到 tile 的區塊）：填 0（海平面）
    void_mask = (mosaic == SRTM_NODATA) & downloaded
    n_void = int(void_mask.sum())
    if n_void > 0:
        from scipy.ndimage import distance_transform_edt
        valid_mask = (mosaic != SRTM_NODATA) & downloaded
        # 最近鄰索引：對每個 invalid 像素找最近的 valid 像素
        _, (ri, ci) = distance_transform_edt(~valid_mask, return_indices=True)
        mosaic[void_mask] = mosaic[ri[void_mask], ci[void_mask]]
        print(f'  [OK] 內陸 void 填補 {n_void:,} 像素（最近鄰）')

    # 海洋區塊 → 海平面 0
    mosaic[mosaic == SRTM_NODATA] = 0
    mosaic = mosaic.astype(np.int16)

    meta = dict(
        lat_min=float(lat_min), lat_max=float(lat_max),
        lon_min=float(lon_min), lon_max=float(lon_max),
        height=H, width=W, product=product,
    )
    return mosaic, meta


def crop_to_bbox(mosaic: np.ndarray, mosaic_meta: dict, bbox: dict) -> tuple[np.ndarray, dict]:
    """把 mosaic 裁到使用者要求的 bbox。"""
    H, W = mosaic.shape
    # mosaic[0] 對應 lat_max（北邊）
    lat_per_row = (mosaic_meta['lat_max'] - mosaic_meta['lat_min']) / (H - 1)
    lon_per_col = (mosaic_meta['lon_max'] - mosaic_meta['lon_min']) / (W - 1)

    row0 = max(0, int(np.floor((mosaic_meta['lat_max'] - bbox['lat_max']) / lat_per_row)))
    row1 = min(H, int(np.ceil((mosaic_meta['lat_max'] - bbox['lat_min']) / lat_per_row)) + 1)
    col0 = max(0, int(np.floor((bbox['lon_min'] - mosaic_meta['lon_min']) / lon_per_col)))
    col1 = min(W, int(np.ceil((bbox['lon_max'] - mosaic_meta['lon_min']) / lon_per_col)) + 1)

    cropped = mosaic[row0:row1, col0:col1]
    new_meta = dict(
        lat_min=mosaic_meta['lat_max'] - row1 * lat_per_row,
        lat_max=mosaic_meta['lat_max'] - row0 * lat_per_row,
        lon_min=mosaic_meta['lon_min'] + col0 * lon_per_col,
        lon_max=mosaic_meta['lon_min'] + col1 * lon_per_col,
        height=cropped.shape[0],
        width=cropped.shape[1],
        product=mosaic_meta['product'],
    )
    return cropped, new_meta


# ══════════════════════════════════════════════════════════════════════
#  輸出 GeoTIFF
# ══════════════════════════════════════════════════════════════════════

def save_geotiff(arr: np.ndarray, meta: dict, output: Path) -> None:
    import rasterio
    from rasterio.transform import from_bounds
    from rasterio.crs import CRS

    output.parent.mkdir(parents=True, exist_ok=True)
    transform = from_bounds(
        meta['lon_min'], meta['lat_min'],
        meta['lon_max'], meta['lat_max'],
        arr.shape[1], arr.shape[0],
    )
    with rasterio.open(
        str(output), 'w',
        driver='GTiff',
        height=arr.shape[0], width=arr.shape[1],
        count=1, dtype=arr.dtype,
        crs=CRS.from_epsg(4326),
        transform=transform,
        nodata=SRTM_NODATA,
        compress='deflate',
        predictor=2,
        tiled=True,
        blockxsize=512, blockysize=512,
    ) as dst:
        dst.write(arr, 1)
    print(f'  [OK] GeoTIFF 已寫入: {output}')


# ══════════════════════════════════════════════════════════════════════
#  CLI
# ══════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(
        description='下載全台 DEM → 合成 GeoTIFF',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument('--bbox', nargs=4, type=float,
                    metavar=('LON_MIN', 'LAT_MIN', 'LON_MAX', 'LAT_MAX'),
                    help='自訂 bbox（覆寫 --full）')
    ap.add_argument('--full', action='store_true',
                    help='含澎湖/蘭嶼/綠島/馬祖等外島（預設只主島）')
    ap.add_argument('--product', choices=['SRTM3', 'SRTM1'], default='SRTM3',
                    help='SRTM3 = 90m，SRTM1 = 30m（資料量約 9 倍）')
    ap.add_argument('--output', '-o', type=Path, default=Path('taiwan_dem.tif'),
                    help='輸出 GeoTIFF 檔名')
    ap.add_argument('--cache-dir', type=Path, default=CACHE_DIR_DEFAULT,
                    help='Tile 快取目錄')
    ap.add_argument('--no-cache', action='store_true',
                    help='不快取 tile（不建議；會重覆下載）')
    ap.add_argument('--dry-run', action='store_true',
                    help='只列出將下載的 tile，不真正下載')
    args = ap.parse_args()

    # 決定 bbox
    if args.bbox:
        bbox = dict(lon_min=args.bbox[0], lat_min=args.bbox[1],
                     lon_max=args.bbox[2], lat_max=args.bbox[3])
        scope = '自訂'
    elif args.full:
        bbox = dict(BBOX_FULL)
        scope = '全台 + 外島'
    else:
        bbox = dict(BBOX_MAIN)
        scope = '主島'

    cache_dir = None if args.no_cache else args.cache_dir
    if cache_dir is not None:
        cache_dir = cache_dir / args.product

    tiles = tiles_for_bbox(bbox)
    print('=' * 64)
    print('  AeroPlan Studio — Taiwan DEM Downloader')
    print('=' * 64)
    print(f'  範圍 ({scope}): '
          f'[{bbox["lon_min"]:.2f}, {bbox["lat_min"]:.2f}] – '
          f'[{bbox["lon_max"]:.2f}, {bbox["lat_max"]:.2f}]')
    print(f'  Product: {args.product}'
          f' ({"~90m" if args.product == "SRTM3" else "~30m"})')
    print(f'  Tiles  : {len(tiles)} 個 1°×1°')
    print(f'  快取   : {cache_dir if cache_dir else "停用"}')
    print(f'  輸出   : {args.output}')
    print()

    if args.dry_run:
        print('  [DRY RUN] 將下載的 tiles：')
        for lat, lon in tiles:
            print(f'    {tile_name(lat, lon)}')
        return 0

    t0 = time.time()
    mosaic, mosaic_meta = build_mosaic(bbox, tiles, args.product, cache_dir)

    cropped, meta = crop_to_bbox(mosaic, mosaic_meta, bbox)
    print(f'\n  Mosaic: {mosaic.shape} → 裁切後: {cropped.shape}')
    print(f'  輸出範圍: '
          f'[{meta["lon_min"]:.4f}, {meta["lat_min"]:.4f}] – '
          f'[{meta["lon_max"]:.4f}, {meta["lat_max"]:.4f}]')
    print(f'  高程範圍: {int(cropped.min())} ~ {int(cropped.max())} m')

    save_geotiff(cropped, meta, args.output)
    print(f'\n  [TIME] 總耗時 {time.time() - t0:.1f}s')
    print(f'  [HINT] 主程式載入：把 {args.output} 路徑傳入 DEMTerrainManager.load_dem()')
    return 0


if __name__ == '__main__':
    sys.exit(main())
