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
    ΔwB_l = ΔwT_l + w_i

⚠ 論文 Eq.(7) 印的是 ``ΔwB_l = l·w_i + (l-1)·w_ovl``，**不可照抄**：那個 '+'
  與「條帶寬度恆為 w_i」自相矛盾，且會讓 w_ovl 在中線 (ΔwT+ΔwB)/2 上剛好對消，
  使重疊率完全失效（UI 調了等於沒調）。這裡採用與 ΔwT_l 一致的 pitch = w_i - w_ovl。
  值層級鎖見 tests/test_dccpp_strip_pitch.py。

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


def width_along_direction(
    hull_xy: Sequence[Tuple[float, float]],
    angle_deg: float,
) -> MinWidthResult:
    """指定掃描方向時的「寬度」—— 手動掃描角走的那條路。

    ⚠ 這裡的寬度是多邊形在**該方向法向上的投影跨距**，不是
    :func:`find_min_width` 的最小值。用最小值會讓條帶數恆等於自動模式，
    也就是角度靜默失效（見 tests/test_dccpp_scan_angle.py 的說明）。

    Args:
        hull_xy   : 凸包頂點（local ENU meters，x=East, y=North）
        angle_deg : 掃描線飛行方位角（compass，0=北，順時針為正）。
                    可為任意實數，內部取 mod 360。

    Returns:
        與 :func:`find_min_width` 同型的 :class:`MinWidthResult`，
        可直接餵給 :func:`generate_paper_scan_lines`。

    設計筆記
    --------
    ``nb_min`` 取 ``ub_min`` 的**左法向**，與 :func:`find_min_width` 在逆時針
    凸包上得到的法向同一個慣例；``v2`` 取投影最小的頂點。兩者合起來保證
    「把自動算出來的 ``alpha_k_deg`` 當手動角送回來」會**逐點**重現自動的
    掃描線（否則條帶順序會整組反過來）。
    """
    pts = np.asarray(hull_xy, dtype=np.float64)
    if len(pts) < 3:
        raise ValueError(f"凸包至少需 3 頂點，實際 {len(pts)}")

    compass = float(angle_deg) % 360.0
    math_deg = (90.0 - compass) % 360.0
    rad = math.radians(math_deg)
    ub = np.array([math.cos(rad), math.sin(rad)])       # 單位切向（飛行方向）
    nb = np.array([-ub[1], ub[0]])                      # 左法向（條帶推進方向）

    proj = pts @ nb
    i_min = int(np.argmin(proj))
    i_max = int(np.argmax(proj))
    width = float(proj[i_max] - proj[i_min])

    anchor = pts[i_min]
    return MinWidthResult(
        min_width=width,
        edge_index=-1,                       # 手動角不對應任何一條凸包邊
        v2=(float(anchor[0]), float(anchor[1])),
        v1=(float(anchor[0]), float(anchor[1])),
        v_furthest=(float(pts[i_max][0]), float(pts[i_max][1])),
        ub_min=(float(ub[0]), float(ub[1])),
        nb_min=(float(nb[0]), float(nb[1])),
        alpha_k_deg=compass,
        edge_angle_math_deg=math_deg,
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

#: 單一區域的掃描線數硬上限（防止病態參數炸掉規劃）。超過即拒收，不截斷。
_MAX_SCAN_LINES = 4096


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

    # 條帶排布：寬度恆為 w_i，相鄰條帶中線間距（pitch）= w_i - w_ovl。
    #
    # ⚠ 不可照抄論文 Eq.(7) 印出的 ΔwB_l = l·w_i + (l-1)·w_ovl：那個 '+' 與
    #   「條帶寬度恆為 w_i」自相矛盾，會讓 w_ovl 在中線 (ΔwT+ΔwB)/2 上剛好
    #   對消 ⇒ 使用者在 UI 調重疊率，規劃結果一個字都不會變。同理條帶數的
    #   級距必須是 w_i - w_ovl，用 w_i + w_ovl 會讓重疊率**反向**（越重疊
    #   越稀疏）。值層級鎖見 tests/test_dccpp_strip_pitch.py。
    pitch = w_i - w_ovl                       # 上面已保證 0 < pitch <= w_i
    # 條帶總數：第一條蓋住 [0, w_i]，其後每條推進一個 pitch，直到蓋滿 W。
    # 減 1e-9 吸收浮點誤差，避免 (W - w_i) 剛好是 pitch 整數倍時多出一條重複線。
    n_layers = max(1, int(math.ceil((W - w_i) / pitch - 1e-9)) + 1)
    # 安全上限。⚠ 不可靜默截斷：pitch 修正後（見上）高重疊率真的會撞到這個上限，
    #   而截斷等於交出一份「蓋不滿區域」的規劃且完全沒有告警 —— 正是本專案要
    #   避免的那類失效。撞到就大聲拒收。
    if n_layers > _MAX_SCAN_LINES:
        raise ValueError(
            f"需要 {n_layers} 條掃描線（上限 {_MAX_SCAN_LINES}）："
            f"W={W:.1f} m, w_i={w_i} m, w_ovl={w_ovl} m -> pitch={pitch:.3f} m。"
            "重疊率過高，請調低 w_ovl 或增大 w_i。"
        )

    scan_lines: List[ScanLine] = []
    for li in range(1, n_layers + 1):
        dw_top = (li - 1) * pitch
        dw_bot = dw_top + w_i                 # 寬度恆為 w_i，不隨 l 變寬
        if dw_top > W and li > 1:
            break
        # 中線 = 條帶幾何中心
        dw_mid = 0.5 * (dw_top + dw_bot)
        # 最後一條通常會超出遠緣；若中線落到區域外，_seg_polygon_intersections
        # 會找不到交點而把整條**靜默丟掉**，遠緣留下最寬達 w_i/2 的漏掃帶。
        # 此時把整條條帶往內拉到貼齊遠緣（中線 W - w_i/2），仍然蓋滿邊界。
        if dw_mid > W:
            if W >= w_i:
                dw_top = W - w_i              # 貼齊遠緣
            else:
                dw_top = 0.5 * (W - w_i)      # 區域比一條條帶還窄 ⇒ 置中（dw_top < 0）
            dw_bot = dw_top + w_i
            dw_mid = 0.5 * (dw_top + dw_bot)  # 回報值與實際中線永遠一致
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
        scan_angle_deg: Optional[float] = None,
    ) -> AreaProcessingResult:
        """多邊形 → 凸包 → 掃描線。

        Args:
            scan_angle_deg: 手動掃描方位角（compass，0=北）。``None``（預設）
                代表自動模式，方向由 :func:`find_min_width` 的幾何最小寬度決定。
                ⚠ 這個參數必須一路送到這裡才算生效 —— 只送到上游而不改
                方向，log 會照印角度但飛出來的航點完全不變。
        """
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

        # 掃描方向：手動角優先，否則用幾何最小寬度
        if scan_angle_deg is None:
            mwr = find_min_width(hull)
        else:
            mwr = width_along_direction(hull, float(scan_angle_deg))

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
