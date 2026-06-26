"""S 形機動航點產生器 — STOT 補時用。

STOT(同時命中)中，路徑較短的 UAV 會比別人早到，需要「消耗時間」對齊命中時刻。
原本用 NAV_LOITER_TIME 在預備點盤旋；本模組改用「S 形蛇行(weave)」：在 預備點→俯衝起點
之間左右擺動，把路徑拉長到「直線距離 + 速度×待消耗秒數」，讓飛機以巡航速度飛完恰好
多花掉 time_to_burn 秒。相較盤旋圈，S 機動有前進、戰術上更貼近實際突防。
"""
from __future__ import annotations

import math
from typing import List, Tuple

_R = 6371000.0


def _hav(la1: float, lo1: float, la2: float, lo2: float) -> float:
    dla = math.radians(la2 - la1)
    dlo = math.radians(lo2 - lo1)
    h = (math.sin(dla / 2) ** 2
         + math.cos(math.radians(la1)) * math.cos(math.radians(la2)) * math.sin(dlo / 2) ** 2)
    return 2 * _R * math.asin(min(1.0, math.sqrt(h)))


def _bearing(la1: float, lo1: float, la2: float, lo2: float) -> float:
    y = math.sin(math.radians(lo2 - lo1)) * math.cos(math.radians(la2))
    x = (math.cos(math.radians(la1)) * math.sin(math.radians(la2))
         - math.sin(math.radians(la1)) * math.cos(math.radians(la2)) * math.cos(math.radians(lo2 - lo1)))
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def _dest(lat: float, lon: float, brg_deg: float, dist_m: float) -> Tuple[float, float]:
    br = math.radians(brg_deg)
    d_n = dist_m * math.cos(br)
    d_e = dist_m * math.sin(br)
    return (lat + d_n / 111320.0,
            lon + d_e / (111320.0 * math.cos(math.radians(lat))))


def generate_s_maneuver_path(start_lat: float, start_lon: float,
                             end_lat: float, end_lon: float,
                             cruise_speed_mps: float, time_to_burn_s: float,
                             turn_radius_m: float, alt: float,
                             ) -> List[Tuple[float, float, float]]:
    """從 start → end 產生 S 形 weave 航點。

    回傳航點序列總飛行長度 ≈ 直線距離 + cruise_speed × time_to_burn
    （以巡航速度飛完會比直飛多花 time_to_burn 秒）。
    若 time_to_burn 太小(<5s)或增益不足，退回單一 end 直線點。
    """
    direct = _hav(start_lat, start_lon, end_lat, end_lon)
    extra = cruise_speed_mps * max(time_to_burn_s, 0.0)
    if time_to_burn_s < 5.0 or extra < max(direct * 0.10, 60.0) or direct < 1.0:
        return [(end_lat, end_lon, alt)]
    target_len = direct + extra
    brg = _bearing(start_lat, start_lon, end_lat, end_lon)
    perp = brg + 90.0
    # weave 段數：約每 350m 前進一段，至少 4 段（段越多 = 蛇行越細密）
    n = max(4, int(round(direct / 350.0)))

    def _build(amp: float) -> List[Tuple[float, float, float]]:
        wp: List[Tuple[float, float, float]] = []
        for k in range(1, n):
            f = direct * k / n
            b_lat, b_lon = _dest(start_lat, start_lon, brg, f)
            side = amp if (k % 2 == 1) else -amp     # 左右交替 → S 形
            w_lat, w_lon = _dest(b_lat, b_lon, perp, side)
            wp.append((w_lat, w_lon, alt))
        wp.append((end_lat, end_lon, alt))
        return wp

    # 對擺幅 amp 做二分搜尋，使實際飛行長度 = target_len（路徑長對 amp 單調遞增）
    lo, hi = 0.0, max(direct, extra)
    for _ in range(32):
        mid = (lo + hi) / 2.0
        if path_length_m(start_lat, start_lon, _build(mid)) < target_len:
            lo = mid
        else:
            hi = mid
    return _build((lo + hi) / 2.0)


def path_length_m(start_lat: float, start_lon: float,
                  wpts: List[Tuple[float, float, float]]) -> float:
    """從 start 起算，依序經過 wpts 的總飛行長度（m）。"""
    pts = [(start_lat, start_lon)] + [(w[0], w[1]) for w in wpts]
    return sum(_hav(pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1])
               for i in range(len(pts) - 1))
