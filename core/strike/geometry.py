"""
core.strike.geometry — 蜂群打擊共用幾何工具
=============================================

此模組集中所有 strike planner 共用的：
    * Haversine / bearing / destination (WGS-84 近似)
    * Local ENU 互轉
    * 3D 向量運算
    * Dubins 路徑長度（LRU cache 版）
    * 角度差 (angular diff)

先前各 strike planner 都 re-implement 這些函式（約 2500 行重複），
統一後：各 planner 只匯入本模組，不再重造輪子。

相容性
------
為避免 break 現有 `from core.strike.swarm_strike_planner import ...` 的程式碼，
swarm_strike_planner 仍 re-export 這些符號。新程式碼建議改用本模組。
"""
from __future__ import annotations

import math
from functools import lru_cache
from typing import Any, Iterable, List, Optional, Tuple

from utils.math_utils import EARTH_RADIUS_M

# (0-2 標準化) 地球半徑統一為 WGS-84 赤道半徑 6_378_137（單一來源：utils.math_utils），
# 與 GPS / MAVLink / ArduPilot 一致。原為球面平均 6_371_000；strike 全幾何
# （haversine / destination / ENU / Dubins）隨之等比 +0.112%。
R_EARTH: float = EARTH_RADIUS_M

# MAVLink frame: 3 = MAV_FRAME_GLOBAL_RELATIVE_ALT
MAV_FRAME_REL: int = 3


# ═══════════════════════════════════════════════════════════════════════
#  大地座標工具
# ═══════════════════════════════════════════════════════════════════════

def haversine(lat1: float, lon1: float,
              lat2: float, lon2: float) -> float:
    """Haversine 公式計算大圓距離 (m)"""
    rlat1, rlon1 = math.radians(lat1), math.radians(lon1)
    rlat2, rlon2 = math.radians(lat2), math.radians(lon2)
    dlat, dlon = rlat2 - rlat1, rlon2 - rlon1
    a = (math.sin(dlat / 2) ** 2 +
         math.cos(rlat1) * math.cos(rlat2) * math.sin(dlon / 2) ** 2)
    return 2 * R_EARTH * math.asin(min(1.0, math.sqrt(a)))


def bearing_deg(lat1: float, lon1: float,
                lat2: float, lon2: float) -> float:
    """從 (lat1, lon1) 到 (lat2, lon2) 的方位角 (度，0=北，順時針)"""
    rlat1, rlon1 = math.radians(lat1), math.radians(lon1)
    rlat2, rlon2 = math.radians(lat2), math.radians(lon2)
    dlon = rlon2 - rlon1
    x = math.sin(dlon) * math.cos(rlat2)
    y = (math.cos(rlat1) * math.sin(rlat2) -
         math.sin(rlat1) * math.cos(rlat2) * math.cos(dlon))
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def destination(lat: float, lon: float,
                bearing_deg_: float, dist_m: float) -> Tuple[float, float]:
    """從起點沿指定方位角移動 dist_m 後的目的地 (lat, lon)"""
    rlat = math.radians(lat)
    rlon = math.radians(lon)
    rbrg = math.radians(bearing_deg_)
    d = dist_m / R_EARTH
    rlat2 = math.asin(
        math.sin(rlat) * math.cos(d) +
        math.cos(rlat) * math.sin(d) * math.cos(rbrg)
    )
    rlon2 = rlon + math.atan2(
        math.sin(rbrg) * math.sin(d) * math.cos(rlat),
        math.cos(d) - math.sin(rlat) * math.sin(rlat2),
    )
    return math.degrees(rlat2), math.degrees(rlon2)


def angular_diff(a_deg: float, b_deg: float) -> float:
    """兩角度的最短差 (0 ≤ 回傳值 ≤ 180°)"""
    d = abs((a_deg - b_deg) % 360.0)
    return min(d, 360.0 - d)


def assign_omnidirectional_slots(
    positions: List[Tuple[Any, float, float]],
    target_lat: float,
    target_lon: float,
    offset_deg: float = 0.0,
) -> List[Tuple[Any, float]]:
    """將 N 個來源點分配到以目標為中心、360° 均勻分佈的攻擊方位角 slot。

    這是 swarm / advanced_swarm / vtol_swarm / recon_to_strike 四個 planner
    原本各自重複實作的「方位排序 + 最小角差旋轉對齊」演算法，收斂為單一權威來源。
    與原四份逐位元等價（同 bearing_deg / angular_diff、同嚴格 `<` tie-break）。

    參數
    ----
    positions : List[(key, lat, lon)]
        key 為穩定識別子（可為 UAV 物件、uid 等），呼叫端用以映回來源。
    target_lat, target_lon : float
        目標座標。
    offset_deg : float
        攻擊向量起始相位（預設 0.0；recon 路徑固定用 0.0，切勿傳非 0 值）。

    回傳
    ----
    List[(key, attack_heading_deg)]，順序為依方位升冪排序後與攻擊向量的對應。
    """
    n = len(positions)
    if n == 0:
        return []
    if n == 1:
        key, lat, lon = positions[0]
        return [(key, bearing_deg(lat, lon, target_lat, target_lon))]

    # 依各點相對目標的方位角升冪排序（Timsort 穩定，方位相同保留原序）
    by = sorted(
        positions,
        key=lambda p: bearing_deg(p[1], p[2], target_lat, target_lon),
    )
    # 360° 均勻攻擊向量
    vectors = sorted((offset_deg + 360.0 / n * k) % 360.0 for k in range(n))
    # 圓形旋轉對齊：使總角差最小（嚴格 < 保留最小 shift 的 tie-break）
    best_shift, best_cost = 0, float('inf')
    for shift in range(n):
        cost = sum(
            angular_diff(
                bearing_deg(by[i][1], by[i][2], target_lat, target_lon),
                vectors[(i + shift) % n],
            )
            for i in range(n)
        )
        if cost < best_cost:
            best_cost, best_shift = cost, shift
    return [(by[i][0], vectors[(i + best_shift) % n]) for i in range(n)]


# ═══════════════════════════════════════════════════════════════════════
#  Local ENU 互轉 (小範圍線性近似，誤差 < 20 km 可忽略)
# ═══════════════════════════════════════════════════════════════════════

def latlon_to_enu(lat: float, lon: float, alt: float,
                   ref_lat: float, ref_lon: float,
                   ref_alt: float = 0.0) -> Tuple[float, float, float]:
    """經緯度 → 以 ref 為原點的 ENU 公尺 (east, north, up)"""
    coslat = math.cos(math.radians(ref_lat))
    dx = math.radians(lon - ref_lon) * R_EARTH * coslat
    dy = math.radians(lat - ref_lat) * R_EARTH
    return dx, dy, alt - ref_alt


def enu_to_latlon(dx: float, dy: float, dz: float,
                   ref_lat: float, ref_lon: float,
                   ref_alt: float = 0.0) -> Tuple[float, float, float]:
    """ENU → 經緯度 (lat, lon, alt)"""
    coslat = math.cos(math.radians(ref_lat))
    return (
        ref_lat + math.degrees(dy / R_EARTH),
        ref_lon + math.degrees(dx / (R_EARTH * max(coslat, 1e-9))),
        ref_alt + dz,
    )


# ═══════════════════════════════════════════════════════════════════════
#  3D 向量運算
# ═══════════════════════════════════════════════════════════════════════

def v3_norm(v: Tuple[float, float, float]) -> float:
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


def v3_normalize(v: Tuple[float, float, float],
                  eps: float = 1e-9) -> Tuple[float, float, float]:
    n = v3_norm(v)
    if n < eps:
        return (0.0, 0.0, 0.0)
    return (v[0] / n, v[1] / n, v[2] / n)


def v3_add(*vs: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (sum(v[0] for v in vs),
            sum(v[1] for v in vs),
            sum(v[2] for v in vs))


def v3_scale(v: Tuple[float, float, float], s: float
              ) -> Tuple[float, float, float]:
    return (v[0] * s, v[1] * s, v[2] * s)


# ═══════════════════════════════════════════════════════════════════════
#  Dubins 路徑長度 — LRU cache 版
# ═══════════════════════════════════════════════════════════════════════

def _mod2pi(x: float) -> float:
    return x - 2.0 * math.pi * math.floor(x / (2.0 * math.pi))


def _dubins_LSL(alpha: float, beta: float, d: float) -> Optional[float]:
    tmp0 = d + math.sin(alpha) - math.sin(beta)
    p_sqr = (2 + d * d - 2 * math.cos(alpha - beta)
             + 2 * d * (math.sin(alpha) - math.sin(beta)))
    if p_sqr < 0:
        return None
    tmp1 = math.atan2(math.cos(beta) - math.cos(alpha), tmp0)
    t = _mod2pi(-alpha + tmp1)
    p = math.sqrt(p_sqr)
    q = _mod2pi(beta - tmp1)
    return t + p + q


def _dubins_RSR(alpha: float, beta: float, d: float) -> Optional[float]:
    tmp0 = d - math.sin(alpha) + math.sin(beta)
    p_sqr = (2 + d * d - 2 * math.cos(alpha - beta)
             + 2 * d * (math.sin(beta) - math.sin(alpha)))
    if p_sqr < 0:
        return None
    tmp1 = math.atan2(math.cos(alpha) - math.cos(beta), tmp0)
    t = _mod2pi(alpha - tmp1)
    p = math.sqrt(p_sqr)
    q = _mod2pi(-beta + tmp1)
    return t + p + q


def _dubins_LSR(alpha: float, beta: float, d: float) -> Optional[float]:
    p_sqr = (-2 + d * d + 2 * math.cos(alpha - beta)
             + 2 * d * (math.sin(alpha) + math.sin(beta)))
    if p_sqr < 0:
        return None
    p = math.sqrt(p_sqr)
    tmp1 = math.atan2(-math.cos(alpha) - math.cos(beta),
                      d + math.sin(alpha) + math.sin(beta)) - math.atan2(-2.0, p)
    t = _mod2pi(-alpha + tmp1)
    q = _mod2pi(-_mod2pi(beta) + tmp1)
    return t + p + q


def _dubins_RSL(alpha: float, beta: float, d: float) -> Optional[float]:
    p_sqr = (d * d - 2 + 2 * math.cos(alpha - beta)
             - 2 * d * (math.sin(alpha) + math.sin(beta)))
    if p_sqr < 0:
        return None
    p = math.sqrt(p_sqr)
    tmp1 = math.atan2(math.cos(alpha) + math.cos(beta),
                      d - math.sin(alpha) - math.sin(beta)) - math.atan2(2.0, p)
    t = _mod2pi(alpha - tmp1)
    q = _mod2pi(beta - tmp1)
    return t + p + q


def _dubins_impl(start_lat: float, start_lon: float, start_hdg: float,
                  end_lat: float, end_lon: float, end_hdg: float,
                  R: float) -> float:
    """Dubins 最短路徑長度實作 (純算法)"""
    dist = haversine(start_lat, start_lon, end_lat, end_lon)
    brg_compass = bearing_deg(start_lat, start_lon, end_lat, end_lon)
    dx = dist * math.sin(math.radians(brg_compass))
    dy = dist * math.cos(math.radians(brg_compass))

    def _to_math_rad(c: float) -> float:
        return math.radians((90.0 - c) % 360.0)

    h_s = _to_math_rad(start_hdg)
    h_e = _to_math_rad(end_hdg)

    R_eff = max(R, 0.01)
    theta = math.atan2(dy, dx)
    d = dist / R_eff
    alpha = _mod2pi(h_s - theta)
    beta  = _mod2pi(h_e - theta)

    lengths = []
    for fn in (_dubins_LSL, _dubins_RSR, _dubins_LSR, _dubins_RSL):
        Ln = fn(alpha, beta, d)
        if Ln is not None and math.isfinite(Ln):
            lengths.append(Ln)
    if lengths:
        return min(lengths) * R_eff
    return dist + 2.0 * R_eff


@lru_cache(maxsize=2048)
def _dubins_cached(s_lat_q: int, s_lon_q: int, s_hdg_q: int,
                    e_lat_q: int, e_lon_q: int, e_hdg_q: int,
                    r_q: int) -> float:
    """LRU cache 層：將浮點參數量化為整數 key，讓「幾乎相同」的查詢能命中"""
    return _dubins_impl(
        s_lat_q / 1e6, s_lon_q / 1e6, s_hdg_q / 10.0,
        e_lat_q / 1e6, e_lon_q / 1e6, e_hdg_q / 10.0,
        r_q / 100.0,
    )


def dubins_shortest_length(start_lat: float, start_lon: float, start_heading_deg: float,
                            end_lat: float, end_lon: float, end_heading_deg: float,
                            turn_radius_m: float) -> float:
    """兩個 2D pose 之間的最短 Dubins 路徑長度 (m)，帶 LRU cache。

    量化：座標 1e-6° (≈0.1m)、航向 0.1°、半徑 0.01m。
    典型 10 機場景可達 5–10× 加速。
    """
    return _dubins_cached(
        int(round(start_lat * 1e6)),
        int(round(start_lon * 1e6)),
        int(round(start_heading_deg * 10.0)),
        int(round(end_lat * 1e6)),
        int(round(end_lon * 1e6)),
        int(round(end_heading_deg * 10.0)),
        int(round(turn_radius_m * 100.0)),
    )


def clear_dubins_cache() -> None:
    """清空 Dubins LRU cache (單元測試用)"""
    _dubins_cached.cache_clear()


def dubins_cache_info():
    """回傳 Dubins cache 統計 (hits/misses/currsize/maxsize)"""
    return _dubins_cached.cache_info()


# ═══════════════════════════════════════════════════════════════════════
#  批次並行 Dubins — 大規模群飛規劃加速
# ═══════════════════════════════════════════════════════════════════════
# 使用場景：N×M 配對矩陣 (N 架 UCAV × M 個目標候選 IP)。
# 單核 Python 純計算，當 N·M > 200 時值得用 ProcessPool 平行化。
# ═══════════════════════════════════════════════════════════════════════

_PairPose = Tuple[
    float, float, float,   # start lat/lon/heading
    float, float, float,   # end lat/lon/heading
    float,                  # radius
]


def dubins_batch(pairs: Iterable[_PairPose]) -> List[float]:
    """序列版本批次計算 Dubins 路徑長度。

    先嘗試 cache 命中，未命中才計算。N 不大時比 ProcessPool 快。
    """
    return [
        dubins_shortest_length(
            p[0], p[1], p[2], p[3], p[4], p[5], p[6]
        )
        for p in pairs
    ]


def dubins_batch_parallel(pairs: Iterable[_PairPose],
                           workers: Optional[int] = None,
                           min_threshold: int = 5000) -> List[float]:
    """平行版本批次計算 — 當 pair 數 > min_threshold 時使用 ProcessPool。

    ⚠️ 重要：Windows 上 ProcessPool spawn cost 很高 (~600ms)，且 LRU cache
    已讓序列版本極快。**實務上大部分場景序列版反而更快**，本 API 留給：
      - 未來引入「非 cache 友善」工作負載 (如每次目標都不同)
      - 超大規模群飛 (N·M > 10000)

    實測 (Python 3.14 on Windows):
        N = 500   → 序列 3.4ms / 平行 685ms  (序列完勝)
        N = 5000  → 序列 ~35ms / 平行 ~1.1s
        N = 50000 → 序列 ~350ms / 平行 ~1.5s  (此時平行才勝出)

    Parameters
    ----------
    pairs :
        迭代器，每個元素為 7-tuple (s_lat, s_lon, s_hdg, e_lat, e_lon, e_hdg, R)
    workers :
        進程數；None = os.cpu_count()
    min_threshold :
        低於此數量直接走序列版本

    Returns
    -------
    List[float] : 與 pairs 順序一致的路徑長度列表
    """
    pair_list = list(pairs)
    if len(pair_list) < min_threshold:
        return dubins_batch(pair_list)

    # ProcessPool：注意每個子進程有獨立的 LRU cache
    try:
        from concurrent.futures import ProcessPoolExecutor
    except ImportError:
        return dubins_batch(pair_list)

    try:
        with ProcessPoolExecutor(max_workers=workers) as ex:
            # 用 starmap 避免 partial 的 pickle 複雜度
            results = list(ex.map(_dubins_tuple_wrapper, pair_list,
                                    chunksize=max(1, len(pair_list) // 16)))
        return results
    except Exception:
        # 回退到序列版本（某些環境如 frozen exe 無法 spawn）
        return dubins_batch(pair_list)


def _dubins_tuple_wrapper(pair: _PairPose) -> float:
    """ProcessPool worker：7-tuple 解包後呼叫 dubins"""
    return dubins_shortest_length(
        pair[0], pair[1], pair[2], pair[3], pair[4], pair[5], pair[6]
    )


# ═══════════════════════════════════════════════════════════════════════
#  Dubins 路徑採樣 — 視覺化用
# ═══════════════════════════════════════════════════════════════════════
# 既有 dubins_shortest_length 只回傳長度供時間/距離計算；polyline 視覺化
# 則需要實際走過的 (lat, lon) 採樣點。下面的函式回傳完整 Dubins 軌跡。
#
# 6 種 Dubins path：CSC = LSL/RSR/LSR/RSL (Curve-Straight-Curve)
#                  CCC = RLR/LRL（兩端距離 < 4R 時才可能最短，本實作省略）
# 對固定翼蜂群場景距離通常 ≫ 4R，CSC 已涵蓋實用情況。
# ═══════════════════════════════════════════════════════════════════════

def _dubins_seg_LSL(alpha: float, beta: float, d: float):
    tmp0 = d + math.sin(alpha) - math.sin(beta)
    p_sqr = (2 + d * d - 2 * math.cos(alpha - beta)
             + 2 * d * (math.sin(alpha) - math.sin(beta)))
    if p_sqr < 0:
        return None
    tmp1 = math.atan2(math.cos(beta) - math.cos(alpha), tmp0)
    t = _mod2pi(-alpha + tmp1)
    p = math.sqrt(p_sqr)
    q = _mod2pi(beta - tmp1)
    return ('LSL', t, p, q)


def _dubins_seg_RSR(alpha: float, beta: float, d: float):
    tmp0 = d - math.sin(alpha) + math.sin(beta)
    p_sqr = (2 + d * d - 2 * math.cos(alpha - beta)
             + 2 * d * (math.sin(beta) - math.sin(alpha)))
    if p_sqr < 0:
        return None
    tmp1 = math.atan2(math.cos(alpha) - math.cos(beta), tmp0)
    t = _mod2pi(alpha - tmp1)
    p = math.sqrt(p_sqr)
    q = _mod2pi(-beta + tmp1)
    return ('RSR', t, p, q)


def _dubins_seg_LSR(alpha: float, beta: float, d: float):
    p_sqr = (-2 + d * d + 2 * math.cos(alpha - beta)
             + 2 * d * (math.sin(alpha) + math.sin(beta)))
    if p_sqr < 0:
        return None
    p = math.sqrt(p_sqr)
    tmp1 = math.atan2(-math.cos(alpha) - math.cos(beta),
                      d + math.sin(alpha) + math.sin(beta)) - math.atan2(-2.0, p)
    t = _mod2pi(-alpha + tmp1)
    q = _mod2pi(-_mod2pi(beta) + tmp1)
    return ('LSR', t, p, q)


def _dubins_seg_RSL(alpha: float, beta: float, d: float):
    p_sqr = (d * d - 2 + 2 * math.cos(alpha - beta)
             - 2 * d * (math.sin(alpha) + math.sin(beta)))
    if p_sqr < 0:
        return None
    p = math.sqrt(p_sqr)
    tmp1 = math.atan2(math.cos(alpha) + math.cos(beta),
                      d - math.sin(alpha) - math.sin(beta)) - math.atan2(2.0, p)
    t = _mod2pi(alpha - tmp1)
    q = _mod2pi(beta - tmp1)
    return ('RSL', t, p, q)


def _dubins_step(x: float, y: float, theta: float,
                  ch: str, length: float, R: float):
    """從 (x, y, theta) 在 math 座標系下走 ch 段 length×R 公尺，回傳新 pose。
    ch ∈ {'L', 'S', 'R'}；length 為「弧度/R」無單位，乘 R 即實際距離。
    """
    if ch == 'L':
        new_theta = theta + length
        nx = x + R * (math.sin(new_theta) - math.sin(theta))
        ny = y - R * (math.cos(new_theta) - math.cos(theta))
    elif ch == 'R':
        new_theta = theta - length
        nx = x - R * (math.sin(new_theta) - math.sin(theta))
        ny = y + R * (math.cos(new_theta) - math.cos(theta))
    else:  # 'S'
        nx = x + R * length * math.cos(theta)
        ny = y + R * length * math.sin(theta)
        new_theta = theta
    return nx, ny, new_theta


def dubins_path_samples(start_lat: float, start_lon: float, start_heading_deg: float,
                         end_lat: float, end_lon: float, end_heading_deg: float,
                         turn_radius_m: float, n_samples: int = 30
                         ) -> List[Tuple[float, float]]:
    """
    生成 Dubins 最短路徑的 (lat, lon) 採樣點列表，供 polyline 視覺化。

    Parameters
    ----------
    start_*, end_* : 起終點位姿（compass 航向：0°=N, 順時針）
    turn_radius_m  : 最小轉彎半徑 (m)
    n_samples      : 採樣段數（總點數 = n_samples + 1，含起終點）

    Returns
    -------
    List[(lat, lon)] — 從起點到終點的平滑 Dubins 軌跡；
    若距離過短或 dubins 無解，退化為直線兩點。
    """
    R = max(turn_radius_m, 0.01)
    dist = haversine(start_lat, start_lon, end_lat, end_lon)
    if dist < 0.1:
        return [(start_lat, start_lon), (end_lat, end_lon)]

    # 把座標轉為以起點為原點的本地 ENU；math 系統 x=East, y=North
    brg_compass = bearing_deg(start_lat, start_lon, end_lat, end_lon)
    dx = dist * math.sin(math.radians(brg_compass))
    dy = dist * math.cos(math.radians(brg_compass))

    def _to_math_rad(c: float) -> float:
        return math.radians((90.0 - c) % 360.0)

    h_s = _to_math_rad(start_heading_deg)
    h_e = _to_math_rad(end_heading_deg)
    theta = math.atan2(dy, dx)
    d = dist / R
    alpha = _mod2pi(h_s - theta)
    beta = _mod2pi(h_e - theta)

    # 列舉 4 種 CSC 候選，取最短
    candidates = []
    for fn in (_dubins_seg_LSL, _dubins_seg_RSR,
               _dubins_seg_LSR, _dubins_seg_RSL):
        seg = fn(alpha, beta, d)
        if seg is None:
            continue
        kind, t, p, q = seg
        if not (math.isfinite(t) and math.isfinite(p) and math.isfinite(q)):
            continue
        candidates.append((t + p + q, kind, t, p, q))
    if not candidates:
        # 無解 → 退化直線
        return [(start_lat, start_lon), (end_lat, end_lon)]

    _, kind, t, p, q = min(candidates, key=lambda c: c[0])
    L_total = t + p + q

    # 沿 path 等弧長採樣 (math 座標系下)
    pts_xy: List[Tuple[float, float]] = []
    for i in range(n_samples + 1):
        s = (i / float(n_samples)) * L_total
        # 確定 s 落在哪一段、走多少
        if s <= t:
            x, y, _ = _dubins_step(0.0, 0.0, h_s, kind[0], s, R)
        elif s <= t + p:
            x_t, y_t, theta_t = _dubins_step(0.0, 0.0, h_s, kind[0], t, R)
            x, y, _ = _dubins_step(x_t, y_t, theta_t, kind[1], s - t, R)
        else:
            x_t, y_t, theta_t = _dubins_step(0.0, 0.0, h_s, kind[0], t, R)
            x_p, y_p, theta_p = _dubins_step(x_t, y_t, theta_t, kind[1], p, R)
            x, y, _ = _dubins_step(x_p, y_p, theta_p, kind[2], s - t - p, R)
        pts_xy.append((x, y))

    # 把 ENU (xi=East, yi=North) 轉回 (lat, lon)
    coslat = math.cos(math.radians(start_lat))
    pts: List[Tuple[float, float]] = []
    for (xi, yi) in pts_xy:
        new_lat = start_lat + math.degrees(yi / R_EARTH)
        new_lon = start_lon + math.degrees(xi / (R_EARTH * max(coslat, 1e-9)))
        pts.append((new_lat, new_lon))
    return pts
