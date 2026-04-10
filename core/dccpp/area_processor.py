"""
DCCPP 區域前處理 v2
====================
忠實實作論文 (Wang et al., Defence Technology 2025) Section 2.2 / 2.3.1。

核心數學物件
------------
給定子區域 Z_k （凸多邊形，逆時針），對每條邊 E_{k,m(m+1)} 計算「該邊到所
有其他頂點的最大垂直距離」即為「該邊方向上的多邊形寬度」。所有邊取最小者
即為最小寬度 w_{k,min}：

    w_{k,min} = min_m  max_u  perp_dist( v_{k,u},  E_{k,m(m+1)} )

對應的邊  E_{k,min}、最遠頂點  v_{k,min}、單位切向 ub_{k,min}、單位法向
nb_{k,min} (指向 v_{k,min}) 即決定最佳掃描方向 α_k。

掃描線層 L_{k,l} (Eq. 6/7)
--------------------------
    ΔwT_l = (l-1)·(w_i - w_ovl)
    ΔwB_l = l·w_i + (l-1)·w_ovl
    L^T_{k,l}(q) = v_{k,2} + q·ub_min + ΔwT_l·nb_min   (條帶上邊界)
    L^B_{k,l}(q) = v_{k,2} + q·ub_min + ΔwB_l·nb_min   (條帶下邊界)
    L_{k,l} (中線) = v_{k,2} + q·ub_min + (ΔwT_l+ΔwB_l)/2·nb_min

每條中線與凸多邊形的邊做線段求交，得到 p^L_{k,l}, p^R_{k,l}。

座標
----
輸入 polygon 可以是 lat/lon 或 local ENU meters；若給 transformer 則內部
全部以 ENU 公尺座標計算，輸出 lat/lon。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

PolygonPoints = List[Tuple[float, float]]


# ============================================================
# 結果結構
# ============================================================

@dataclass
class MinWidthResult:
    """論文 w_{k,min} / E_{k,min} / v_{k,min} / nb_{k,min} 等資訊。

    所有座標皆為 local ENU meters (x=East, y=North)。
    """
    min_width: float                          # w_{k,min} [m]
    edge_index: int                           # E_{k,min} 在凸包中的邊索引 (m)
    v2: Tuple[float, float]                   # v_{k,2}：E_{k,min} 的「右」端點
    v1: Tuple[float, float]                   # v_{k,1}：E_{k,min} 的「左」端點
    v_furthest: Tuple[float, float]           # v_{k,min}
    ub_min: Tuple[float, float]               # 單位切向 (從 v1 指向 v2)
    nb_min: Tuple[float, float]               # 單位法向 (指向 v_furthest)
    alpha_k_deg: float                        # 掃描線方位角 (compass, 0=N, CW)
    edge_angle_math_deg: float                # E_{k,min} 的數學角 (0=East, CCW)


@dataclass
class ScanLine:
    """單條作業掃描線 L_{k,l} (對應論文 Section 2.3.1)。"""
    index: int                              # l (從 0 開始)
    # 端點：local ENU meters
    pL_xy: Tuple[float, float]              # p^L_{k,l}
    pR_xy: Tuple[float, float]              # p^R_{k,l}
    # 端點：lat/lon （若有 transformer）
    pL_latlon: Optional[Tuple[float, float]] = None
    pR_latlon: Optional[Tuple[float, float]] = None
    length_m: float = 0.0                   # 作業段長度 |pR - pL|
    delta_w_top: float = 0.0                # ΔwT_l
    delta_w_bot: float = 0.0                # ΔwB_l
    heading_compass_deg: float = 0.0        # L→R 飛行方位角 (compass)
    heading_math_deg: float = 0.0           # 同上的數學角


@dataclass
class AreaProcessingResult:
    """完整前處理輸出。"""
    original_polygon_latlon: PolygonPoints
    convex_hull_latlon: PolygonPoints
    convex_hull_xy: PolygonPoints
    centroid_latlon: Tuple[float, float]
    min_width_result: MinWidthResult
    scan_lines: List[ScanLine] = field(default_factory=list)
    w_i: float = 0.0                        # 條帶寬 (FOV 覆蓋寬)
    w_ovl: float = 0.0                      # 條帶重疊寬度


# ============================================================
# 基礎演算法：DP / Convex Hull
# ============================================================

def douglas_peucker(points: Sequence[Tuple[float, float]], epsilon: float) -> PolygonPoints:
    """Ramer–Douglas–Peucker 折線簡化。"""
    if len(points) < 3:
        return list(points)
    pts = np.asarray(points, dtype=np.float64)
    n = len(pts)
    keep = np.zeros(n, dtype=bool)
    keep[0] = keep[-1] = True

    def _rdp(s: int, e: int) -> None:
        if e - s < 2:
            return
        seg = pts[e] - pts[s]
        L = float(np.hypot(seg[0], seg[1]))
        if L < 1e-12:
            return
        u = seg / L
        diff = pts[s + 1:e] - pts[s]
        d = np.abs(diff[:, 0] * u[1] - diff[:, 1] * u[0])
        k = int(np.argmax(d))
        if d[k] > epsilon:
            mid = s + 1 + k
            keep[mid] = True
            _rdp(s, mid)
            _rdp(mid, e)

    _rdp(0, n - 1)
    return [tuple(p) for p in pts[keep]]


def convex_hull(points: Sequence[Tuple[float, float]]) -> PolygonPoints:
    """Andrew monotone chain，回傳逆時針凸包 (不含重複首尾)。"""
    pts = sorted(set((float(p[0]), float(p[1])) for p in points))
    if len(pts) < 3:
        return [tuple(p) for p in pts]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


# ============================================================
# 最小寬度（論文 Sec 2.2.3 嚴格實作）
# ============================================================

def find_min_width(hull_xy: Sequence[Tuple[float, float]]) -> MinWidthResult:
    """嚴格依論文枚舉每條邊與其餘頂點的最大垂直距離。"""
    pts = np.asarray(hull_xy, dtype=np.float64)
    n = len(pts)
    if n < 3:
        raise ValueError(f"凸包至少需 3 頂點，實際 {n}")

    # 確保逆時針
    area2 = 0.0
    for i in range(n):
        j = (i + 1) % n
        area2 += pts[i, 0] * pts[j, 1] - pts[j, 0] * pts[i, 1]
    if area2 < 0:
        pts = pts[::-1]

    best_width = math.inf
    best_m = 0
    best_furthest_idx = 0
    best_normal = np.array([0.0, 1.0])
    best_tangent = np.array([1.0, 0.0])

    for m in range(n):
        a = pts[m]
        b = pts[(m + 1) % n]
        edge = b - a
        L = float(np.hypot(edge[0], edge[1]))
        if L < 1e-9:
            continue
        u = edge / L                                   # 單位切向 (a→b)
        nrm = np.array([-u[1], u[0]])                  # 左手法向 (對逆時針凸包，指向多邊形內側)

        # 對其餘頂點計算 |signed perp dist|，取最大者作為「該邊方向上的多邊形寬度」
        max_d = 0.0
        far_idx = m
        for k in range(n):
            if k == m or k == (m + 1) % n:
                continue
            d = abs(float(np.dot(pts[k] - a, nrm)))
            if d > max_d:
                max_d = d
                far_idx = k

        if max_d < best_width:
            best_width = max_d
            best_m = m
            best_furthest_idx = far_idx
            best_tangent = u
            # 法向必須指向 v_{k,min}（與該頂點同側）
            sign = float(np.dot(pts[far_idx] - a, nrm))
            best_normal = nrm if sign >= 0 else -nrm

    v1 = pts[best_m]                       # E_{k,min} 起點 (對應論文 v_{k,1})
    v2 = pts[(best_m + 1) % n]             # E_{k,min} 終點 (對應論文 v_{k,2})
    v_far = pts[best_furthest_idx]

    # 數學角 (atan2 在 (East,North) 空間 = 0 指向 East, CCW)
    edge_math_deg = math.degrees(math.atan2(best_tangent[1], best_tangent[0]))
    # 轉為羅盤方位 (0=North, CW)：compass = (90 - math) mod 360
    compass = (90.0 - edge_math_deg) % 360.0

    return MinWidthResult(
        min_width=float(best_width),
        edge_index=best_m,
        v2=(float(v2[0]), float(v2[1])),
        v1=(float(v1[0]), float(v1[1])),
        v_furthest=(float(v_far[0]), float(v_far[1])),
        ub_min=(float(best_tangent[0]), float(best_tangent[1])),
        nb_min=(float(best_normal[0]), float(best_normal[1])),
        alpha_k_deg=compass,
        edge_angle_math_deg=edge_math_deg,
    )


# ============================================================
# 線段求交：水平掃描線 ↔ 凸多邊形邊
# ============================================================

def _seg_polygon_intersections(
    p0: np.ndarray, dir_vec: np.ndarray, hull_pts: np.ndarray
) -> List[np.ndarray]:
    """無限長直線 (p0 + t·dir_vec) 與凸多邊形所有邊的交點。

    對凸多邊形而言至多 2 個交點；若擦邊則只回傳 1 個或 0 個。
    """
    out: List[np.ndarray] = []
    n = len(hull_pts)
    for i in range(n):
        a = hull_pts[i]
        b = hull_pts[(i + 1) % n]
        edge = b - a
        # 解 [dir_vec, -edge] · [t, s] = a - p0
        denom = dir_vec[0] * (-edge[1]) - dir_vec[1] * (-edge[0])
        if abs(denom) < 1e-12:
            continue
        rhs = a - p0
        t = (rhs[0] * (-edge[1]) - rhs[1] * (-edge[0])) / denom
        s = (dir_vec[0] * rhs[1] - dir_vec[1] * rhs[0]) / denom
        if -1e-9 <= s <= 1.0 + 1e-9:
            out.append(p0 + t * dir_vec)
    # 去重
    uniq: List[np.ndarray] = []
    for p in out:
        if not any(np.hypot(*(p - q)) < 1e-6 for q in uniq):
            uniq.append(p)
    return uniq


# ============================================================
# 掃描線生成 (論文 Eq. 6/7 嚴格)
# ============================================================

def generate_paper_scan_lines(
    hull_xy: Sequence[Tuple[float, float]],
    mwr: MinWidthResult,
    w_i: float,
    w_ovl: float,
) -> List[ScanLine]:
    """依論文 Eq. 6/7 生成所有作業掃描線。

    Args:
        hull_xy : 凸包頂點 (local meters，逆時針)
        mwr     : find_min_width 結果
        w_i     : 單條條帶寬度 (FOV ground projection 的有效寬)
        w_ovl   : 相鄰條帶的重疊寬度 (m)，0 = 無重疊
    """
    if w_i <= 0:
        raise ValueError(f"w_i 必須 > 0，實際 {w_i}")
    if w_ovl < 0 or w_ovl >= w_i:
        raise ValueError(f"w_ovl 必須在 [0, w_i)，實際 {w_ovl}")

    hull = np.asarray(hull_xy, dtype=np.float64)
    v2 = np.asarray(mwr.v2, dtype=np.float64)
    ub = np.asarray(mwr.ub_min, dtype=np.float64)
    nb = np.asarray(mwr.nb_min, dtype=np.float64)
    W = mwr.min_width

    if W <= 0:
        return []

    # 按論文 ΔwB_l = l·w_i + (l-1)·w_ovl，l 從 1 起
    # 條帶總數：找到第一個使 ΔwB_l ≥ W 的 l
    step = w_i + w_ovl
    n_layers = max(1, int(math.ceil((W - w_i) / step)) + 1)
    # 安全上限
    n_layers = min(n_layers, 4096)

    scan_lines: List[ScanLine] = []
    for li in range(1, n_layers + 1):
        dw_top = (li - 1) * (w_i - w_ovl)
        dw_bot = li * w_i + (li - 1) * w_ovl
        if dw_top > W and li > 1:
            break
        # 中線 = 條帶幾何中心
        dw_mid = 0.5 * (dw_top + min(dw_bot, W + w_i))  # 讓最後一條條帶仍位於區域內
        # 中線一點 + 切向，做無限長直線
        p0 = v2 + dw_mid * nb
        ints = _seg_polygon_intersections(p0, ub, hull)
        if len(ints) < 2:
            continue
        # 沿 ub 排序：左 = 小，右 = 大
        proj = [float(np.dot(p - p0, ub)) for p in ints]
        order = np.argsort(proj)
        pL = ints[order[0]]
        pR = ints[order[-1]]
        length = float(np.hypot(*(pR - pL)))
        if length < 1e-3:
            continue

        # 飛行航向 (L→R)
        math_deg = math.degrees(math.atan2(ub[1], ub[0]))
        compass = (90.0 - math_deg) % 360.0

        scan_lines.append(ScanLine(
            index=len(scan_lines),
            pL_xy=(float(pL[0]), float(pL[1])),
            pR_xy=(float(pR[0]), float(pR[1])),
            length_m=length,
            delta_w_top=float(dw_top),
            delta_w_bot=float(dw_bot),
            heading_compass_deg=compass,
            heading_math_deg=math_deg,
        ))

    return scan_lines


# ============================================================
# 高階主類別：AreaProcessor
# ============================================================

class AreaProcessor:
    """多邊形 → 凸包 → 最小寬度 → 論文掃描線。

    使用方式 (lat/lon 輸入)::

        ap = AreaProcessor(dp_epsilon_m=2.0)
        result = ap.process_latlon(polygon_latlon, w_i=120.0, w_ovl=12.0)
        for sl in result.scan_lines:
            print(sl.pL_latlon, sl.pR_latlon)
    """

    def __init__(self, dp_epsilon_m: float = 1.0, min_vertices: int = 4) -> None:
        self.dp_epsilon_m = dp_epsilon_m
        self.min_vertices = min_vertices

    # ─────────── lat/lon 接口 ───────────
    def process_latlon(
        self,
        polygon_latlon: Sequence[Tuple[float, float]],
        w_i: float,
        w_ovl: float = 0.0,
    ) -> AreaProcessingResult:
        if len(polygon_latlon) < 3:
            raise ValueError(f"多邊形至少 3 點，實際 {len(polygon_latlon)}")

        from core.geometry.coordinate import CoordinateTransformer
        # 區域質心作為 ENU 原點
        c_lat = sum(p[0] for p in polygon_latlon) / len(polygon_latlon)
        c_lon = sum(p[1] for p in polygon_latlon) / len(polygon_latlon)
        tf = CoordinateTransformer(c_lat, c_lon)

        # lat/lon → ENU
        xy_pts: List[Tuple[float, float]] = []
        for lat, lon in polygon_latlon:
            enu = tf.geo_to_local(lat, lon)
            xy_pts.append((float(enu[0]), float(enu[1])))

        # DP 簡化 (在公尺空間做)
        simplified = douglas_peucker(xy_pts, self.dp_epsilon_m)
        if len(simplified) < self.min_vertices:
            simplified = xy_pts

        # 凸包
        hull = convex_hull(simplified)
        if len(hull) < 3:
            hull = convex_hull(xy_pts)

        # 最小寬度
        mwr = find_min_width(hull)

        # 掃描線
        scan_lines = generate_paper_scan_lines(hull, mwr, w_i, w_ovl)

        # 每條掃描線端點轉回 lat/lon
        for sl in scan_lines:
            gL = tf.local_to_geo(sl.pL_xy[0], sl.pL_xy[1])
            gR = tf.local_to_geo(sl.pR_xy[0], sl.pR_xy[1])
            sl.pL_latlon = (gL.latitude, gL.longitude)
            sl.pR_latlon = (gR.latitude, gR.longitude)

        # 凸包 lat/lon
        hull_latlon: PolygonPoints = []
        for x, y in hull:
            g = tf.local_to_geo(x, y)
            hull_latlon.append((g.latitude, g.longitude))

        return AreaProcessingResult(
            original_polygon_latlon=list(polygon_latlon),
            convex_hull_latlon=hull_latlon,
            convex_hull_xy=list(hull),
            centroid_latlon=(c_lat, c_lon),
            min_width_result=mwr,
            scan_lines=scan_lines,
            w_i=w_i,
            w_ovl=w_ovl,
        )


# ============================================================
# 向後相容 alias
# ============================================================

def compute_min_width_direction(hull, strip_spacing=None):
    """舊 API 包裝；回傳 MinWidthResult (新版欄位)。"""
    return find_min_width(hull)
