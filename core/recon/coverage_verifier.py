"""覆蓋率驗算 —— 規劃完之後，實際檢查偵察區有沒有被掃完。

為什麼要有這個
--------------
DCCPP 產生的是「理論上掃得完」的路徑，但實際覆蓋率會被這些因素咬掉：
迴轉半徑讓 VTOL 在邊角切出去、避撞繞道、條帶間距用了錯的掃描寬度、
區域是凹多邊形。規劃器**不會回報**這些，所以要獨立驗算一次。

作法
----
在多邊形外接矩形上鋪等距格網（本地 ENU 公尺座標），保留落在多邊形內的格心，
再把每架 UAV 的航跡當成折線，標記距折線 ≤ sweep_width/2 的格心。
輸出覆蓋率、漏掃格心座標、以及重掃次數分佈。

⚠ 這裡量的是**幾何覆蓋**：假設感測器沿航跡連續成像、且不看地形遮蔽與姿態擾動。
它能抓到「規劃就沒掃到」，抓不到「飛的時候沒拍到」。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

__all__ = ["CoverageReport", "verify_coverage"]

_EARTH_R = 6378137.0
# 格心數量上限，避免使用者把格距設成 0.1 m 就把 UI 卡死
_MAX_CELLS = 2_000_000


@dataclass
class _Grid:
    """驗算用的規則格網（本地 ENU），保留下來供熱力圖呈現，不必重算。"""

    nx: int
    ny: int
    lat0: float
    lon0: float
    xmin: float
    ymin: float
    cell_size_m: float
    inside: "np.ndarray"      # bool，長度 nx*ny，索引 = ix*ny + iy
    hits: "np.ndarray"        # int32，同長度；多邊形外一律 0


@dataclass
class CoverageReport:
    """覆蓋率驗算結果。"""

    total_cells: int                       # 落在多邊形內的格心數
    covered_cells: int                     # 至少被掃過一次的格心數
    coverage_ratio: float                  # covered / total（0–1）
    cell_size_m: float
    sweep_width_m: float
    missed_points: List[Tuple[float, float]] = field(default_factory=list)
    """漏掃格心的 (lat, lon)。已依上限抽樣，見 missed_truncated。"""
    missed_truncated: bool = False
    overlap_histogram: Dict[int, int] = field(default_factory=dict)
    """{被**幾架**掃到: 格心數}。⚠ 單機重飛同一塊地十次，這裡仍然只算 1 ——
    要看「跑了多少重複的路徑」請改用 :attr:`pass_histogram`。"""
    pass_histogram: Dict[int, int] = field(default_factory=dict)
    """{被掃過**幾趟**: 格心數}，含同一架自己重掃。這才是「重複路徑」的量。"""
    per_uav_cells: Dict[int, int] = field(default_factory=dict)
    """{uav_id: 該機掃到的格心數}"""
    grid: Optional["_Grid"] = field(default=None, repr=False)
    """驗算格網本身，供熱力圖呈現用（見 :meth:`to_display_grid`）。"""

    @property
    def coverage_pct(self) -> float:
        return self.coverage_ratio * 100.0

    def to_display_grid(self, max_side: int = 60) -> Optional[dict]:
        """把驗算格網聚合成適合畫熱力圖的粗格。

        驗算格距通常是 10–20 m，一個幾平方公里的偵察區就是數萬到數百萬格；
        直接一格一個 Cesium entity 會把瀏覽器打死。這裡把細格聚合成最多
        ``max_side × max_side`` 的顯示格，每個顯示格的值 = 其涵蓋的細格中
        「被掃到」的比例（0–1），沒有任何細格落在多邊形內的顯示格回傳 ``None``。

        回傳
        ----
        dict 或 None（沒有格網時）::

            {
              "lat_min":…, "lat_max":…, "lon_min":…, "lon_max":…,
              "rows": ny_disp, "cols": nx_disp,
              "values": [ …, ]   # 長度 rows*cols，列優先(row-major)；
                                 # None = 該顯示格完全在偵察區外
            }

        ``values`` 的 0 = 全漏掃（熱力圖畫紅），1 = 全覆蓋（畫藍）。
        """
        g = self.grid
        if g is None or g.nx <= 0 or g.ny <= 0:
            return None

        inside = g.inside.reshape(g.nx, g.ny)
        hit = (g.hits.reshape(g.nx, g.ny) > 0) & inside

        # 每個顯示格聚合多少細格
        fx = max(1, int(math.ceil(g.nx / max_side)))
        fy = max(1, int(math.ceil(g.ny / max_side)))
        nx_d = int(math.ceil(g.nx / fx))
        ny_d = int(math.ceil(g.ny / fy))

        # 補齊到可整除再 reshape 聚合
        pad_x = nx_d * fx - g.nx
        pad_y = ny_d * fy - g.ny
        ins = np.pad(inside, ((0, pad_x), (0, pad_y)), constant_values=False)
        hts = np.pad(hit, ((0, pad_x), (0, pad_y)), constant_values=False)
        ins_b = ins.reshape(nx_d, fx, ny_d, fy).sum(axis=(1, 3))
        hts_b = hts.reshape(nx_d, fx, ny_d, fy).sum(axis=(1, 3))

        # 顯示格的地理範圍（用細格網的實際外框，非多邊形外框）
        x0, y0 = g.xmin, g.ymin
        x1 = x0 + g.nx * g.cell_size_m
        y1 = y0 + g.ny * g.cell_size_m
        (lat_min, lon_min), (lat_max, lon_max) = _to_latlon(
            np.array([[x0, y0], [x1, y1]], dtype=float), g.lat0, g.lon0
        )

        # values 以 row-major 輸出：row = 緯度方向(y/ny_d)，col = 經度方向(x/nx_d)
        values: List[Optional[float]] = []
        for r in range(ny_d):
            for c in range(nx_d):
                n_in = int(ins_b[c, r])
                values.append(None if n_in == 0 else float(hts_b[c, r]) / n_in)

        return {
            "lat_min": lat_min, "lat_max": lat_max,
            "lon_min": lon_min, "lon_max": lon_max,
            "rows": ny_d, "cols": nx_d,
            "values": values,
        }

    @property
    def redundant_ratio(self) -> float:
        """被 **2 架以上**掃到的格心占「已覆蓋」的比例 —— 多機重疊偵察的程度。

        ⚠⚠⚠ 這**不是**「跑了多少重複路徑」。它以「不同 UAV」計數，所以單機任務
        下它在數學上恆為 0，不論那台飛機把同一塊地掃了幾遍。要量重複路徑請用
        :attr:`repeat_ratio` 或 :attr:`overcoverage_x`。
        """
        if self.covered_cells <= 0:
            return 0.0
        redundant = sum(n for k, n in self.overlap_histogram.items() if k >= 2)
        return redundant / self.covered_cells

    @property
    def repeat_ratio(self) -> float:
        """被掃 **2 趟以上**的格心占「已覆蓋」的比例 —— 真正的「重複路徑」程度。

        與 :attr:`redundant_ratio` 的差別：這裡算的是**趟數**，同一架自己來回
        掃兩次會被算到；單機任務下它**可以**是非零值。
        """
        if self.covered_cells <= 0 or not self.pass_histogram:
            return 0.0
        return sum(n for k, n in self.pass_histogram.items() if k >= 2) / self.covered_cells

    @property
    def overcoverage_x(self) -> float:
        """過度覆蓋**倍率** = 總掃描趟次 ÷ 已覆蓋格心數。1.0 = 每塊地剛好掃一次。

        ⚠ 覆蓋率的上限是 100%，結構上看不見「掃太多次」；這個量沒有上限，
        所以它是唯一能分辨「剛好掃完」與「掃了三遍」的指標。
        """
        if self.covered_cells <= 0 or not self.pass_histogram:
            return 0.0
        return sum(k * n for k, n in self.pass_histogram.items()) / self.covered_cells

    def summary(self) -> str:
        return (
            f"覆蓋率 {self.coverage_pct:.2f}%"
            f"（{self.covered_cells}/{self.total_cells} 格，格距 {self.cell_size_m:g} m，"
            f"掃描寬度 {self.sweep_width_m:g} m），"
            f"過度覆蓋 {self.overcoverage_x:.3f}×"
            f"（重掃 {self.repeat_ratio * 100:.1f}%、多機重疊 {self.redundant_ratio * 100:.1f}%）"
        )


def _to_local_xy(points: Sequence[Tuple[float, float]],
                 lat0: float, lon0: float) -> np.ndarray:
    """(lat, lon) → 以 (lat0, lon0) 為原點的本地 ENU 公尺座標。"""
    if not len(points):
        return np.zeros((0, 2), dtype=float)
    arr = np.asarray(points, dtype=float)
    lat = arr[:, 0]
    lon = arr[:, 1]
    x = np.radians(lon - lon0) * _EARTH_R * math.cos(math.radians(lat0))
    y = np.radians(lat - lat0) * _EARTH_R
    return np.column_stack([x, y])


def _to_latlon(xy: np.ndarray, lat0: float, lon0: float) -> List[Tuple[float, float]]:
    if xy.size == 0:
        return []
    lat = lat0 + np.degrees(xy[:, 1] / _EARTH_R)
    lon = lon0 + np.degrees(xy[:, 0] / (_EARTH_R * math.cos(math.radians(lat0))))
    return [(float(a), float(b)) for a, b in zip(lat, lon)]


def _points_in_polygon(pts: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """向量化 ray-casting；回傳 bool 陣列。凹多邊形亦適用。"""
    if pts.size == 0:
        return np.zeros(0, dtype=bool)
    x, y = pts[:, 0], pts[:, 1]
    inside = np.zeros(len(pts), dtype=bool)
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        # 射線與邊相交測試
        cond = ((yi > y) != (yj > y))
        with np.errstate(divide="ignore", invalid="ignore"):
            xint = (xj - xi) * (y - yi) / (yj - yi + 1e-300) + xi
        inside ^= cond & (x < xint)
        j = i
    return inside


def _pass_counts(pts: np.ndarray, line: np.ndarray, half: float) -> np.ndarray:
    """每個格心被**這一條航跡**掃過幾**趟**（不是幾架，也不是被幾個航段碰到）。

    ⚠⚠⚠ 為什麼需要這個：舊的算法是 ``hit_count += (dist <= half)``，其中 ``hit``
    是布林 ⇒ **一架飛機不論在同一塊地上來回飛幾次都只加 1**。實測：同一台沿同一條
    12 線 lawnmower 飛**三遍**，`redundant_ratio` 回 **0.0000**、直方圖 `{1: 360}`。
    也就是說「有沒有跑重複的路徑」這件事，在單機情形下**數學上不可能**被舊指標量到，
    而它恰好被讀成「沒有重複」。

    判準不用距離門檻也不用魔數：航跡是有序的，**連續的航段算同一趟**，
    要「離開之後再回來」（中間至少斷一段）才算新的一趟。掉頭會跨過許多航段，
    所以來回掃同一塊地會正確地記成 2 趟。
    """
    n = len(pts)
    passes = np.zeros(n, dtype=np.int32)
    if n == 0 or len(line) == 0:
        return passes
    if len(line) == 1:
        return (np.hypot(pts[:, 0] - line[0, 0],
                         pts[:, 1] - line[0, 1]) <= half).astype(np.int32)

    last_seg = np.full(n, -2, dtype=np.int64)      # −2 ⇒ 從未被掃到
    a, b = line[:-1], line[1:]
    seg = b - a
    seg_len2 = np.einsum("ij,ij->i", seg, seg)
    for k in range(len(a)):
        d, L2 = seg[k], seg_len2[k]
        if L2 <= 0.0:                              # 退化航段（原地重複點）
            dist = np.hypot(pts[:, 0] - a[k, 0], pts[:, 1] - a[k, 1])
        else:
            t = np.clip(((pts - a[k]) @ d) / L2, 0.0, 1.0)
            px = a[k, 0] + t * d[0]
            py = a[k, 1] + t * d[1]
            dist = np.hypot(pts[:, 0] - px, pts[:, 1] - py)
        hit = dist <= half
        passes[hit & (last_seg < k - 1)] += 1      # 上一段沒掃到 ⇒ 新的一趟
        last_seg[hit] = k
    return passes


def _split_runs(path) -> List[List[Tuple[float, float]]]:
    """路徑 → 連續段的清單。``None`` = 斷點（相機關閉、或兩段成像不相連），兩側**不連線**。

    ⚠⚠ 為什麼要有斷點：驗算把一架的點當成**一條折線**。HARVEST 縮點後每條條帶只剩起點帶
    ``sensor_on``（終點是別的標籤），只取「相機開著的航點」就變成 N 個孤立的點，而驗算把這些點
    用斜線連起來 ⇒ 量的是一條跨過整個區域的鋸齒線，不是飛出去的條帶（2026-10-09 使用者問
    「為何 HARVEST 覆蓋率驗算都那麼差」：同一份計畫 GUI 算 75.06%，實際路徑是 100.00%）。
    舊碼其實已經寫了 ``if p is not None`` 但只是把 None **濾掉**，等於把兩側接起來。
    沒有 None 時只有一段，行為與以前完全相同。
    """
    runs: List[List[Tuple[float, float]]] = []
    cur: List[Tuple[float, float]] = []
    for p in path:
        if p is None:
            if cur:
                runs.append(cur)
                cur = []
        else:
            cur.append(p)
    if cur:
        runs.append(cur)
    return runs


def _min_dist_to_polyline(pts: np.ndarray, line: np.ndarray) -> np.ndarray:
    """每個點到折線的最短距離（向量化，逐段掃過）。"""
    if pts.size == 0:
        return np.zeros(0, dtype=float)
    if len(line) == 1:
        return np.hypot(pts[:, 0] - line[0, 0], pts[:, 1] - line[0, 1])
    best = np.full(len(pts), np.inf, dtype=float)
    a = line[:-1]
    b = line[1:]
    seg = b - a
    seg_len2 = np.einsum("ij,ij->i", seg, seg)
    for k in range(len(a)):
        d = seg[k]
        L2 = seg_len2[k]
        if L2 <= 1e-12:                      # 退化成點
            dist = np.hypot(pts[:, 0] - a[k, 0], pts[:, 1] - a[k, 1])
        else:
            w = pts - a[k]
            t = np.clip((w @ d) / L2, 0.0, 1.0)
            proj = a[k] + np.outer(t, d)
            dist = np.hypot(pts[:, 0] - proj[:, 0], pts[:, 1] - proj[:, 1])
        np.minimum(best, dist, out=best)
    return best


def verify_coverage(
    uav_paths: Dict[int, Sequence[Tuple[float, float]]],
    polygon: Sequence[Tuple[float, float]],
    sweep_width_m: float,
    cell_size_m: float = 20.0,
    max_missed_points: int = 3000,
) -> CoverageReport:
    """驗算 UAV 航跡對偵察區的幾何覆蓋率。

    參數
    ----
    uav_paths : {uav_id: [(lat, lon), ...]}，每架的航跡折線；路徑中的 ``None`` 是**斷點**
                （相機關閉／不相連的成像段），兩側不連線、各算一趟（見 ``_split_runs``）
    polygon   : [(lat, lon), ...] 偵察區多邊形（不需重複首點）
    sweep_width_m : 感測器有效掃描寬度（見 sensor_footprint.effective_sweep_width_m）
    cell_size_m   : 驗算格網解析度；越小越準也越慢
    max_missed_points : 回傳的漏掃座標上限（超過就抽樣，並設 missed_truncated）

    回傳
    ----
    CoverageReport
    """
    if len(polygon) < 3:
        raise ValueError(f"多邊形至少需要 3 個點，收到 {len(polygon)}")
    if sweep_width_m <= 0.0:
        raise ValueError(f"掃描寬度必須為正，收到 {sweep_width_m}")
    if cell_size_m <= 0.0:
        raise ValueError(f"格距必須為正，收到 {cell_size_m}")

    lat0 = float(np.mean([p[0] for p in polygon]))
    lon0 = float(np.mean([p[1] for p in polygon]))
    poly_xy = _to_local_xy(polygon, lat0, lon0)

    xmin, ymin = poly_xy.min(axis=0)
    xmax, ymax = poly_xy.max(axis=0)
    nx = max(1, int(math.ceil((xmax - xmin) / cell_size_m)))
    ny = max(1, int(math.ceil((ymax - ymin) / cell_size_m)))
    if nx * ny > _MAX_CELLS:
        raise ValueError(
            f"格網會產生 {nx * ny:,} 個格心，超過上限 {_MAX_CELLS:,}。"
            f"請把格距從 {cell_size_m:g} m 調大（目前區域約 "
            f"{(xmax - xmin):.0f}×{(ymax - ymin):.0f} m）。"
        )

    # 格心（cell centre）而非格點，避免邊界重複計數
    gx = xmin + (np.arange(nx) + 0.5) * cell_size_m
    gy = ymin + (np.arange(ny) + 0.5) * cell_size_m
    mesh = np.stack(np.meshgrid(gx, gy, indexing="ij"), axis=-1).reshape(-1, 2)

    inside = _points_in_polygon(mesh, poly_xy)
    cells = mesh[inside]
    total = int(len(cells))
    if total == 0:
        return CoverageReport(
            total_cells=0, covered_cells=0, coverage_ratio=0.0,
            cell_size_m=cell_size_m, sweep_width_m=sweep_width_m,
            grid=_Grid(nx=nx, ny=ny, lat0=lat0, lon0=lon0, xmin=float(xmin),
                       ymin=float(ymin), cell_size_m=cell_size_m,
                       inside=inside, hits=np.zeros(nx * ny, dtype=np.int32)),
        )

    half = sweep_width_m / 2.0
    hit_count = np.zeros(total, dtype=np.int32)    # 「被幾**架**掃到」
    pass_count = np.zeros(total, dtype=np.int32)   # 「被掃過幾**趟**」（含同機重掃）
    per_uav: Dict[int, int] = {}
    for uav_id, path in uav_paths.items():
        runs = _split_runs(path)
        if not runs:
            per_uav[int(uav_id)] = 0
            continue
        # ⚠ 兩個量是不同的東西，不可互相代替（見 `_pass_counts` 的說明）：
        #   hit_count 回答「幾架飛機掃到這裡」——單機重飛十次仍然是 1；
        #   pass_count 回答「這塊地被掃了幾次」——那才是「有沒有跑重複的路徑」。
        #   斷開的每一段（條帶）各算一趟：離開之後再回來才算新的一趟。
        p = np.zeros(total, dtype=np.int32)
        for run in runs:
            p += _pass_counts(cells, _to_local_xy(run, lat0, lon0), half)
        hit = p > 0
        per_uav[int(uav_id)] = int(hit.sum())
        hit_count += hit.astype(np.int32)
        pass_count += p

    covered_mask = hit_count > 0
    covered = int(covered_mask.sum())

    missed_xy = cells[~covered_mask]
    truncated = False
    if len(missed_xy) > max_missed_points:
        truncated = True
        step = int(math.ceil(len(missed_xy) / max_missed_points))
        missed_xy = missed_xy[::step]

    hist: Dict[int, int] = {}
    for k, c in zip(*np.unique(hit_count[covered_mask], return_counts=True)):
        hist[int(k)] = int(c)
    phist: Dict[int, int] = {}
    for k, c in zip(*np.unique(pass_count[covered_mask], return_counts=True)):
        phist[int(k)] = int(c)

    # 把「只算內部格」的結果散回完整格網，供熱力圖呈現（不必重算一次）
    full_hits = np.zeros(nx * ny, dtype=np.int32)
    full_hits[inside] = hit_count

    return CoverageReport(
        total_cells=total,
        covered_cells=covered,
        coverage_ratio=covered / total,
        cell_size_m=cell_size_m,
        sweep_width_m=sweep_width_m,
        missed_points=_to_latlon(missed_xy, lat0, lon0),
        missed_truncated=truncated,
        overlap_histogram=hist,
        pass_histogram=phist,
        per_uav_cells=per_uav,
        grid=_Grid(nx=nx, ny=ny, lat0=lat0, lon0=lon0, xmin=float(xmin),
                   ymin=float(ymin), cell_size_m=cell_size_m,
                   inside=inside, hits=full_hits),
    )
