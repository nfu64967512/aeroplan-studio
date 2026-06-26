"""S 形機動航點產生器 — STOT 補時用。

STOT(同時命中)中，路徑較短的 UAV 會比別人早到，需要「消耗時間」對齊命中時刻。
原本用 NAV_LOITER_TIME 在預備點盤旋；本模組改用「S 形蛇行(weave)」：在 預備點→俯衝起點
之間左右擺動，把路徑拉長到「直線距離 + 速度×待消耗秒數」，讓飛機以巡航速度飛完恰好
多花掉 time_to_burn 秒。相較盤旋圈，S 機動有前進、戰術上更貼近實際突防。

幾何工具(haversine/bearing_deg/destination)一律重用 core.strike.geometry，不再自帶副本(DRY)。
"""
from __future__ import annotations

import math
from typing import List, Tuple

from core.strike.geometry import bearing_deg, destination, haversine


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
    direct = haversine(start_lat, start_lon, end_lat, end_lon)
    extra = cruise_speed_mps * max(time_to_burn_s, 0.0)
    if time_to_burn_s < 5.0 or extra < max(direct * 0.10, 60.0) or direct < 1.0:
        return [(end_lat, end_lon, alt)]
    target_len = direct + extra
    brg = bearing_deg(start_lat, start_lon, end_lat, end_lon)
    perp = brg + 90.0
    # weave 段數：約每 350m 前進一段，至少 4 段（段越多 = 蛇行越細密）
    n = max(4, int(round(direct / 350.0)))

    def _build(amp: float) -> List[Tuple[float, float, float]]:
        wp: List[Tuple[float, float, float]] = []
        for k in range(1, n):
            f = direct * k / n
            b_lat, b_lon = destination(start_lat, start_lon, brg, f)
            side = amp if (k % 2 == 1) else -amp     # 左右交替 → S 形
            w_lat, w_lon = destination(b_lat, b_lon, perp, side)
            wp.append((w_lat, w_lon, alt))
        wp.append((end_lat, end_lon, alt))
        return wp

    # 對擺幅 amp 做二分搜尋，使實際飛行長度 = target_len（路徑長對 amp 單調遞增）
    lo, hi = 0.0, max(direct, extra)
    for _ in range(40):
        mid = (lo + hi) / 2.0
        if path_length_m(start_lat, start_lon, _build(mid)) < target_len:
            lo = mid
        else:
            hi = mid
        if (hi - lo) < 0.5:                          # 收斂到 <0.5m 即停（不必跑滿）
            break
    return _build((lo + hi) / 2.0)


def path_length_m(start_lat: float, start_lon: float,
                  wpts: List[Tuple[float, float, float]]) -> float:
    """從 start 起算，依序經過 wpts 的總飛行長度（m）。"""
    pts = [(start_lat, start_lon)] + [(w[0], w[1]) for w in wpts]
    return sum(haversine(pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1])
               for i in range(len(pts) - 1))
