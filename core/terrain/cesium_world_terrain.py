"""Cesium World Terrain（Cesium ion asset 1）高程取樣 —— 給規劃器當 ``terrain_func(lat, lon)``。

為什麼自己解圖磚：規劃器（HARVEST）吃的是 ``f(lat, lon) -> 高程`` 回呼，跑在背景執行緒、不能依賴 GUI 的 3D 頁；
CesiumJS 的 ``sampleTerrainMostDetailed`` 取的是同一份 quantized-mesh 圖磚，這裡用同一組公開端點讀回來（Python 端）。

⚠⚠⚠ **授權（2026-10-09 查證，非法律意見）**
* 免費的 Community 方案 = 個人/非商用（含探索性商用、政府開發、**未獲經費**的教育活動），串流 15 GB/月。
  圖磚的 attribution 也寫著「Upgrade for commercial use」。有研究經費或商用要付費。
* 條款 §2.2.2：不得複製／儲存／散布 Cesium Data Output 供**離線或本機環境**使用（例外：Clips、
  瀏覽器的一般性快取）。⇒ **本模組刻意只放記憶體**：不寫圖磚、不寫高程檔、不存 DEM。
  行程結束就消失；要固定資料請用別的來源（專案的 ``taiwan_dem.tif``）。
* 必須顯示 attribution：``attributions`` 屬性是端點回傳的來源（USGS／CGIAR-CSI／Copernicus 等），
  規劃的 notes 會帶出。

⚠ 高程是**橢球高**（WGS84），不是海拔：台灣比正高（EGM96）高約 15–22 m。規劃只用相對高度，
但不可把它當 AMSL 去對 MSL 的東西。

⚠ 解析度：圖磚取樣間距約 35–40 m（台灣最深可用 level 13），山區細節比 SRTM3(90 m) 好、
比專用 LiDAR DEM 差。
"""
from __future__ import annotations

import logging
import math
import struct
import threading
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

logger = logging.getLogger(__name__)

__all__ = ["CesiumWorldTerrain", "WorldTerrainError", "decode_quantized_mesh", "rasterize_mesh",
           "rasterize_mesh_reference",
           "tile_xy", "tile_bounds"]

ION_API = "https://api.cesium.com"
WORLD_TERRAIN_ASSET_ID = 1
RASTER_N = 129                      # 每張圖磚點陣化成 129×129（level 13 約 19 m 間距）
_HEADER = struct.Struct("<3d2f4d3d")   # 88 bytes
_ACCEPT = "application/vnd.quantized-mesh,application/octet-stream;q=0.9"


class WorldTerrainError(RuntimeError):
    """取樣失敗。訊息只含狀態碼與原因，**不含 token、不含帶 token 的網址**。"""


# ─────────────────────────────────────────────────────────────────────
#  圖磚幾何（EPSG:4326 / TMS：level 0 有 2×1 片、每片 180°，y 由南往北）
# ─────────────────────────────────────────────────────────────────────
def tile_xy(lat: float, lon: float, level: int) -> Tuple[int, int]:
    n = 1 << level
    w = 180.0 / n
    x = int(math.floor((lon + 180.0) / w))
    y = int(math.floor((lat + 90.0) / w))
    return min(max(x, 0), 2 * n - 1), min(max(y, 0), n - 1)


def tile_bounds(level: int, x: int, y: int) -> Tuple[float, float, float, float]:
    """``(west, south, east, north)``（度）。"""
    w = 180.0 / (1 << level)
    return -180.0 + x * w, -90.0 + y * w, -180.0 + (x + 1) * w, -90.0 + (y + 1) * w


# ─────────────────────────────────────────────────────────────────────
#  quantized-mesh 解碼
# ─────────────────────────────────────────────────────────────────────
@dataclass
class QuantizedMesh:
    u: np.ndarray          # uint16，0..32767，西 → 東
    v: np.ndarray          # uint16，0..32767，南 → 北
    h: np.ndarray          # float64，公尺（橢球高）
    tri: np.ndarray        # (T, 3) 頂點索引
    min_h: float
    max_h: float


def _zigzag_delta(a: np.ndarray) -> np.ndarray:
    a = a.astype(np.int32)
    return np.cumsum((a >> 1) ^ -(a & 1)).astype(np.int32)


def decode_quantized_mesh(data: bytes) -> QuantizedMesh:
    """解 Cesium quantized-mesh 1.0 圖磚（只取頂點與三角形；延伸資料忽略）。"""
    if len(data) < _HEADER.size + 4:
        raise WorldTerrainError(f"圖磚太短（{len(data)} bytes）")
    hdr = _HEADER.unpack_from(data, 0)
    min_h, max_h = float(hdr[3]), float(hdr[4])
    off = _HEADER.size
    (vc,) = struct.unpack_from("<I", data, off)
    off += 4
    if vc <= 0 or off + 6 * vc > len(data):
        raise WorldTerrainError(f"圖磚頂點數不合理（{vc}）")
    u = _zigzag_delta(np.frombuffer(data, "<u2", vc, off)); off += 2 * vc
    v = _zigzag_delta(np.frombuffer(data, "<u2", vc, off)); off += 2 * vc
    hq = _zigzag_delta(np.frombuffer(data, "<u2", vc, off)); off += 2 * vc
    wide = vc > 65536
    align = 4 if wide else 2
    if off % align:
        off += align - off % align
    if off + 4 > len(data):
        raise WorldTerrainError("圖磚缺索引資料")
    (nt,) = struct.unpack_from("<I", data, off)
    off += 4
    dt = "<u4" if wide else "<u2"
    isz = 4 if wide else 2
    if nt <= 0 or off + nt * 3 * isz > len(data):
        raise WorldTerrainError(f"圖磚三角形數不合理（{nt}）")
    code = np.frombuffer(data, dt, nt * 3, off).astype(np.int64)
    # high-water-mark 解碼：code 0 = 新頂點（最高水位 +1），其餘 = 回頭參照
    idx = np.empty_like(code)
    hw = 0
    for i, c in enumerate(code.tolist()):
        idx[i] = hw - c
        if c == 0:
            hw += 1
    if idx.min() < 0 or idx.max() >= vc:
        raise WorldTerrainError("圖磚索引超出頂點範圍")
    h = min_h + (hq.astype(np.float64) / 32767.0) * (max_h - min_h)
    return QuantizedMesh(u.astype(np.uint16), v.astype(np.uint16), h, idx.reshape(-1, 3), min_h, max_h)


def rasterize_mesh(mesh: QuantizedMesh, n: int = RASTER_N) -> np.ndarray:
    """三角網 → n×n 規則點陣（``out[j, i]``：i = 西→東、j = 南→北）。

    用 matplotlib 的 ``LinearTriInterpolator``（C 實作的 TriFinder，**同一個三角網、精確的重心內插**）；
    逐三角形的純 numpy 版（:func:`rasterize_mesh_reference`）慢 ~20 倍，只留作測試對照。
    """
    from matplotlib.tri import LinearTriInterpolator, Triangulation
    gx = mesh.u.astype(np.float64) / 32767.0 * (n - 1)
    gy = mesh.v.astype(np.float64) / 32767.0 * (n - 1)
    tri = Triangulation(gx, gy, mesh.tri)
    xi, yi = np.meshgrid(np.arange(n, dtype=np.float64), np.arange(n, dtype=np.float64))
    z = LinearTriInterpolator(tri, mesh.h)(xi, yi)
    return np.ma.filled(z.astype(np.float32), np.nan)


def rasterize_mesh_reference(mesh: QuantizedMesh, n: int = RASTER_N) -> np.ndarray:
    """（參考實作，慢）逐三角形重心座標填值；測試用來對照 :func:`rasterize_mesh`。"""
    gx = mesh.u.astype(np.float64) / 32767.0 * (n - 1)
    gy = mesh.v.astype(np.float64) / 32767.0 * (n - 1)
    z = mesh.h
    out = np.full((n, n), np.nan, dtype=np.float32)
    eps = 1e-9
    for a, b, c in mesh.tri.tolist():
        x0, y0, x1, y1, x2, y2 = gx[a], gy[a], gx[b], gy[b], gx[c], gy[c]
        den = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
        if abs(den) < 1e-12:
            continue
        xa = max(int(math.floor(min(x0, x1, x2) - eps)), 0)
        xb = min(int(math.ceil(max(x0, x1, x2) + eps)), n - 1)
        ya = max(int(math.floor(min(y0, y1, y2) - eps)), 0)
        yb = min(int(math.ceil(max(y0, y1, y2) + eps)), n - 1)
        if xb < xa or yb < ya:
            continue
        px = np.arange(xa, xb + 1, dtype=np.float64)[None, :]
        py = np.arange(ya, yb + 1, dtype=np.float64)[:, None]
        l0 = ((y1 - y2) * (px - x2) + (x2 - x1) * (py - y2)) / den
        l1 = ((y2 - y0) * (px - x2) + (x0 - x2) * (py - y2)) / den
        l2 = 1.0 - l0 - l1
        m = (l0 >= -1e-6) & (l1 >= -1e-6) & (l2 >= -1e-6)
        if not m.any():
            continue
        val = l0 * z[a] + l1 * z[b] + l2 * z[c]
        sub = out[ya:yb + 1, xa:xb + 1]
        sub[m] = val[m]
    return out


def _fill_holes(r: np.ndarray) -> np.ndarray:
    """極少數沒被任何三角形蓋到的格點（浮點邊界）以最近的有值格補上，不丟資料也不留 nan。"""
    bad = ~np.isfinite(r)
    if not bad.any():
        return r
    if bad.all():
        raise WorldTerrainError("圖磚點陣化後沒有任何有效高程")
    from scipy.ndimage import distance_transform_edt
    idx = distance_transform_edt(bad, return_distances=False, return_indices=True)
    return r[tuple(idx)]


# ─────────────────────────────────────────────────────────────────────
#  取樣器
# ─────────────────────────────────────────────────────────────────────
class CesiumWorldTerrain:
    """``__call__(lat, lon)`` / :meth:`get_elevation` → 橢球高（m）。

    * 只放**記憶體**（LRU，預設 256 片 ≈ 17 MB）；行程結束就沒了。
    * ``workers`` 預設 **2**：實測 24 片圖磚 1 條 3.8 s、2 條 1.8 s、4 條 3.3 s、6 條 15.7 s、24 條 40 s ——
      並行越高反而越慢（服務端對同一 token 的並行連線會節流）。別調大。
    * 不吞錯：:meth:`prefetch` 失敗會丟 :class:`WorldTerrainError`；``get_elevation`` 在沒預抓到的地方
      才會臨時抓，失敗時記進 :attr:`failures` 並回 ``None``（轉接層會以平地值填補 ⇒ 呼叫端必須檢查 ``failures``）。
    """

    def __init__(self, token: str, *, asset_id: int = WORLD_TERRAIN_ASSET_ID, session: Any = None,
                 timeout: float = 20.0, max_tiles: int = 256, workers: int = 2):
        if not token or not str(token).strip():
            raise WorldTerrainError("沒有 Cesium ion token（config/settings.json 的 map.cesium_ion_token 或環境變數 CESIUM_ION_TOKEN）")
        self._token = str(token).strip()
        self._asset = int(asset_id)
        if session is None:
            import requests
            session = requests.Session()
        self._s = session
        self._timeout = float(timeout)
        self._max_tiles = int(max_tiles)
        self._workers = int(workers)
        self._lock = threading.RLock()
        self._ready = False
        self._base = ""
        self._auth_hdr: Dict[str, str] = {}
        self._tpl = "{z}/{x}/{y}.terrain?v={version}"
        self._version = "1.2.0"
        self._available: List[List[Dict[str, int]]] = []
        self._level_memo: Dict[Tuple[int, int], int] = {}
        self.attributions: List[str] = []
        self._tiles: "OrderedDict[Tuple[int, int, int], Optional[np.ndarray]]" = OrderedDict()
        self.failures: List[str] = []
        self.stats = {"tiles_fetched": 0, "bytes": 0, "cache_hits": 0, "not_found": 0}

    # ── 連線 ───────────────────────────────────────────────────────
    def _http(self, url: str, headers: Dict[str, str]):
        try:
            return self._s.get(url, headers=headers, timeout=self._timeout)
        except Exception as e:                                    # noqa: BLE001
            raise WorldTerrainError(f"連線失敗（{type(e).__name__}）") from None

    def _ensure(self) -> None:
        with self._lock:
            if self._ready:
                return
            r = self._http(f"{ION_API}/v1/assets/{self._asset}/endpoint",
                           {"Authorization": f"Bearer {self._token}"})
            if r.status_code in (401, 403):
                raise WorldTerrainError(f"Cesium ion 拒絕這個 token（HTTP {r.status_code}）：請確認 token 有效且能存取 asset {self._asset}")
            if r.status_code != 200:
                raise WorldTerrainError(f"取得地形端點失敗（HTTP {r.status_code}）")
            ep = r.json()
            self._base = str(ep["url"]).rstrip("/")
            self._auth_hdr = {"Authorization": f"Bearer {ep['accessToken']}", "Accept": _ACCEPT}
            self.attributions = [str(a.get("html", "")) for a in (ep.get("attributions") or [])]
            lj = self._http(self._base + "/layer.json", self._auth_hdr)
            if lj.status_code != 200:
                raise WorldTerrainError(f"取得 layer.json 失敗（HTTP {lj.status_code}）")
            layer = lj.json()
            if str(layer.get("scheme", "tms")).lower() != "tms" or "4326" not in str(layer.get("projection", "4326")):
                raise WorldTerrainError(f"不支援的圖磚方案：{layer.get('scheme')}/{layer.get('projection')}")
            self._tpl = (layer.get("tiles") or [self._tpl])[0]
            self._version = str(layer.get("version", self._version))
            self._available = layer.get("available") or []
            self._ready = True

    @property
    def attribution_text(self) -> str:
        """端點回傳的資料來源（去掉 HTML），給 notes／log 用。"""
        import re
        txt = " ".join(re.sub(r"<[^>]+>", " ", a) for a in self.attributions)
        return re.sub(r"\s+", " ", txt).strip()

    # ── 層級 / 圖磚 ─────────────────────────────────────────────────
    def max_level_at(self, lat: float, lon: float) -> int:
        """layer.json 的 ``available`` 標示的最深有資料層級（0..len-1）。

        以「最深層級的圖磚索引」為鍵記憶：低層級的 (x, y) 就是它右移，所以同一鍵的結果完全相同。
        （不記憶的話每個取樣點要掃上萬個範圍 ⇒ 832 µs/點，28 600 點 24 s。）
        """
        self._ensure()
        deepest = max(len(self._available) - 1, 0)
        x15, y15 = tile_xy(lat, lon, deepest)
        key = (x15, y15)
        hit = self._level_memo.get(key)
        if hit is not None:
            return hit
        best = 0
        for lv, ranges in enumerate(self._available):
            sh = deepest - lv
            x, y = x15 >> sh, y15 >> sh
            if any(r["startX"] <= x <= r["endX"] and r["startY"] <= y <= r["endY"] for r in ranges):
                best = lv
        self._level_memo[key] = best
        return best

    def _fetch_raster(self, z: int, x: int, y: int) -> Optional[np.ndarray]:
        url = f"{self._base}/" + self._tpl.format(z=z, x=x, y=y, version=self._version)
        r = self._http(url, self._auth_hdr)
        if r.status_code == 404:
            self.stats["not_found"] += 1
            return None
        if r.status_code != 200:
            raise WorldTerrainError(f"取得圖磚 {z}/{x}/{y} 失敗（HTTP {r.status_code}）")
        self.stats["tiles_fetched"] += 1
        self.stats["bytes"] += len(r.content)
        return _fill_holes(rasterize_mesh(decode_quantized_mesh(r.content)))

    def _tile(self, z: int, x: int, y: int) -> Optional[np.ndarray]:
        key = (z, x, y)
        with self._lock:
            if key in self._tiles:
                self._tiles.move_to_end(key)
                self.stats["cache_hits"] += 1
                return self._tiles[key]
        r = self._fetch_raster(z, x, y)                # 網路與解碼不持鎖
        with self._lock:
            self._tiles[key] = r
            self._tiles.move_to_end(key)
            while len(self._tiles) > self._max_tiles:
                self._tiles.popitem(last=False)
        return r

    def _raster_for(self, lat: float, lon: float) -> Tuple[np.ndarray, int, int, int]:
        self._ensure()
        for z in range(self.max_level_at(lat, lon), -1, -1):
            x, y = tile_xy(lat, lon, z)
            r = self._tile(z, x, y)
            if r is not None:
                return r, z, x, y
        raise WorldTerrainError(f"({lat:.5f},{lon:.5f}) 沒有任何層級的圖磚")

    # ── 取樣 ───────────────────────────────────────────────────────
    def get_elevation(self, lat: float, lon: float) -> Optional[float]:
        try:
            r, z, x, y = self._raster_for(float(lat), float(lon))
        except WorldTerrainError as e:
            with self._lock:
                if len(self.failures) < 50:
                    self.failures.append(str(e))
            return None
        w, s, e_, n_ = tile_bounds(z, x, y)
        fx = min(max((lon - w) / (e_ - w), 0.0), 1.0) * (RASTER_N - 1)
        fy = min(max((lat - s) / (n_ - s), 0.0), 1.0) * (RASTER_N - 1)
        i0, j0 = min(int(fx), RASTER_N - 2), min(int(fy), RASTER_N - 2)
        dx, dy = fx - i0, fy - j0
        a, b = r[j0, i0], r[j0, i0 + 1]
        c, d = r[j0 + 1, i0], r[j0 + 1, i0 + 1]
        return float(a * (1 - dx) * (1 - dy) + b * dx * (1 - dy) + c * (1 - dx) * dy + d * dx * dy)

    __call__ = get_elevation

    def sample_grid(self, lats: Sequence[float], lons: Sequence[float]) -> np.ndarray:
        """規則經緯網格的高程（向量化）：回傳 ``(len(lats), len(lons))``，取不到的格是 ``nan``。

        做法：以「最深層級圖磚」為單位把網格切成區塊（區塊內每個點落在同一張圖磚），每個區塊整塊做雙線性
        取樣 ⇒ 100 萬格 < 1 s（逐點 ``get_elevation`` 要 ~11 s）。圖磚沒抓到的區塊記進 :attr:`failures`、填 ``nan``。
        """
        lats = np.asarray(lats, dtype=float)
        lons = np.asarray(lons, dtype=float)
        self._ensure()
        deepest = max(len(self._available) - 1, 0)
        n15 = 1 << deepest
        w15 = 180.0 / n15
        ry = np.clip(np.floor((lats + 90.0) / w15).astype(np.int64), 0, n15 - 1)
        cx = np.clip(np.floor((lons + 180.0) / w15).astype(np.int64), 0, 2 * n15 - 1)
        out = np.full((len(lats), len(lons)), np.nan, dtype=np.float64)

        def _runs(idx: np.ndarray):
            cuts = np.flatnonzero(np.diff(idx) != 0) + 1
            starts = np.concatenate(([0], cuts))
            ends = np.concatenate((cuts, [len(idx)]))
            return list(zip(starts.tolist(), ends.tolist()))

        for r0, r1 in _runs(ry):
            for c0, c1 in _runs(cx):
                la = lats[r0:r1]
                lo = lons[c0:c1]
                try:
                    r, z, x, y = self._raster_for(float(la[len(la) // 2]), float(lo[len(lo) // 2]))
                except WorldTerrainError as e:
                    with self._lock:
                        if len(self.failures) < 50:
                            self.failures.append(str(e))
                    continue
                w_, s_, e_, n_ = tile_bounds(z, x, y)
                fx = np.clip((lo - w_) / (e_ - w_), 0.0, 1.0) * (RASTER_N - 1)
                fy = np.clip((la - s_) / (n_ - s_), 0.0, 1.0) * (RASTER_N - 1)
                i0 = np.minimum(fx.astype(np.int64), RASTER_N - 2)
                j0 = np.minimum(fy.astype(np.int64), RASTER_N - 2)
                dx = (fx - i0)[None, :]
                dy = (fy - j0)[:, None]
                J, I = j0[:, None], i0[None, :]
                out[r0:r1, c0:c1] = (r[J, I] * (1 - dx) * (1 - dy) + r[J, I + 1] * dx * (1 - dy)
                                     + r[J + 1, I] * (1 - dx) * dy + r[J + 1, I + 1] * dx * dy)
        return out

    def prefetch(self, bounds_latlon: Tuple[float, float, float, float], *, max_tiles: int = 400) -> Dict[str, Any]:
        """把 ``(lat_min, lon_min, lat_max, lon_max)`` 涵蓋的圖磚抓進記憶體（以區域中心的最深層級）。

        失敗直接丟 :class:`WorldTerrainError`（**不靜默**）。回傳統計。
        """
        lat_min, lon_min, lat_max, lon_max = (float(v) for v in bounds_latlon)
        if not (lat_max > lat_min and lon_max > lon_min):
            raise WorldTerrainError(f"邊界不合理：{bounds_latlon!r}")
        self._ensure()
        lat_c, lon_c = 0.5 * (lat_min + lat_max), 0.5 * (lon_min + lon_max)
        z = self.max_level_at(lat_c, lon_c)
        x0, y0 = tile_xy(lat_min, lon_min, z)
        x1, y1 = tile_xy(lat_max, lon_max, z)
        want = [(z, x, y) for x in range(x0, x1 + 1) for y in range(y0, y1 + 1)]
        if len(want) > max_tiles:
            raise WorldTerrainError(f"區域太大：level {z} 需要 {len(want)} 片圖磚（上限 {max_tiles}）")
        errs: List[str] = []

        def _one(k):
            try:
                self._tile(*k)
            except WorldTerrainError as e:
                errs.append(str(e))

        before = dict(self.stats)
        with ThreadPoolExecutor(max_workers=max(1, self._workers)) as ex:
            list(ex.map(_one, want))
        if errs:
            raise WorldTerrainError(f"預抓圖磚失敗 {len(errs)}/{len(want)} 片：{errs[0]}")
        return {"level": z, "tiles": len(want), "fetched": self.stats["tiles_fetched"] - before["tiles_fetched"],
                "bytes": self.stats["bytes"] - before["bytes"]}

    def relief_m(self, bounds_latlon: Tuple[float, float, float, float], n: Optional[int] = None, *,
                 step_m: float = 30.0, max_n: int = 300) -> Tuple[float, float]:
        """區域內地形的 ``(最低, 最高)``。要先 :meth:`prefetch`。

        取樣間距預設 ``step_m``=30 m（約等於圖磚解析度），每軸最多 ``max_n`` 點；給 ``n`` 則固定 n×n。
        ⚠ 不能用固定的 40×40：區域一大格距就拉到幾百公尺，會漏掉最高/最低點 ⇒ 起伏被低估
        （2026-10-10 實例：66 m 的起伏被量成 62 m，提示說 177 m 夠、實測要 181 m）。
        """
        lat_min, lon_min, lat_max, lon_max = bounds_latlon
        if n is not None:
            nla = nlo = max(int(n), 2)
        else:
            m_lat = 111320.0
            m_lon = 111320.0 * max(0.2, math.cos(math.radians(0.5 * (lat_min + lat_max))))
            nla = min(max(int(math.ceil((lat_max - lat_min) * m_lat / step_m)) + 1, 2), int(max_n))
            nlo = min(max(int(math.ceil((lon_max - lon_min) * m_lon / step_m)) + 1, 2), int(max_n))
        vals = [self.get_elevation(la, lo)
                for la in np.linspace(lat_min, lat_max, nla) for lo in np.linspace(lon_min, lon_max, nlo)]
        vals = [v for v in vals if v is not None]
        if not vals:
            raise WorldTerrainError("區域內取不到任何高程")
        return float(min(vals)), float(max(vals))
