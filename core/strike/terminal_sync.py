"""終端同步打擊幾何（Terminal-Sync STOT）：把多機「同時命中」做成可重用、可測試的能力。

實測背景（6 機 SITL，桃機/SITL 預設場）：純開環 S 機動補時的命中散度高達 ~38.7s，
對實飛空速保真度過於敏感。改用下列三段式「終端同步」機制後，散度穩定收斂到 ~4s：

  Phase 1  各機 GUIDED 飛到「距目標等距(stage_m)、不同方位」的 push point（繞開），先到者盤旋等待
  Phase 2  待全機都到 push point 後「同一刻」全部釋放，平飛(分層高度)直撲目標
  Phase 3  飛行中以 TimeOnTargetController 持續修正各機空速，收斂到同一命中時刻

關鍵經驗（已實證，務必遵守）：
  * 釋放段必須「平飛」收斂——俯衝(下降)會被重力加速到 v_max，DO_CHANGE_SPEED 失效、修正無力
    （實測：俯衝版 16s vs 平飛版 4s）。分層高度本身即提供垂直間隔避撞。
  * 等距 push point 讓「同步釋放」等價於「同時命中」（俯衝/平飛時間相同），衝刺越短累積變異越小。
  * 不同方位 push point 給出 6 個不對穿的進場向量 → 繞開、且終端最小間隔可維持 ~100m。

本模組只負責「純幾何 / 就緒判斷」(無 MAVLink、無狀態)，與單一職責的
[[tot_controller]] (速度修正) 組合即為完整 GCS 端執行器；方便單元測試與重用。
"""
from __future__ import annotations

from typing import Dict, List, Tuple

from core.strike.geometry import haversine, destination


def fan_bearings(center_deg: float, arc_deg: float, n: int) -> List[float]:
    """在 center_deg 兩側、總張角 arc_deg 的扇形內，均勻取 n 個方位（度）。

    用於產生「不對穿」的多機進場方位：arc_deg<=180 時各機從同一側扇形進場，
    不會有兩機對頭飛向目標（避免終端對穿碰撞）。n==1 時回傳 [center_deg]。
    """
    if n <= 0:
        raise ValueError(f"n 需 >=1，收到 {n}")
    if not (0.0 < arc_deg <= 360.0):
        raise ValueError(f"arc_deg 需在 (0,360]，收到 {arc_deg}")
    if n == 1:
        return [center_deg % 360.0]
    step = arc_deg / (n - 1)
    start = center_deg - arc_deg / 2.0
    return [(start + i * step) % 360.0 for i in range(n)]


def equidistant_push_points(
    target_lat: float, target_lon: float,
    bearings_deg: List[float], stage_m: float,
) -> List[Tuple[float, float]]:
    """回傳一組「距目標等距 stage_m、各自落在 bearings_deg 方位」的 push point (lat,lon)。

    等距是「同步釋放 → 同時命中」的前提；不同方位提供繞開的進場向量。
    """
    if stage_m <= 0.0:
        raise ValueError(f"stage_m 需 >0，收到 {stage_m}")
    return [destination(target_lat, target_lon, b, stage_m) for b in bearings_deg]


def layered_altitudes(n: int, base_m: float, step_m: float) -> List[float]:
    """回傳 n 個分層高度 [base, base+step, ...]，提供平飛收斂/命中時的垂直避撞間隔。"""
    if n <= 0:
        raise ValueError(f"n 需 >=1，收到 {n}")
    if step_m <= 0.0:
        raise ValueError(f"step_m 需 >0，收到 {step_m}")
    return [base_m + i * step_m for i in range(n)]


def all_staged(
    positions: Dict[int, Tuple[float, float]],
    push_points: Dict[int, Tuple[float, float]],
    arrive_m: float,
) -> bool:
    """是否「全機」都已抵達各自 push point（<= arrive_m）→ 可進入 Phase 2 同步釋放。

    positions / push_points 皆以 sysid 為鍵；缺任一機的位置即視為尚未就緒。
    arrive_m 越小釋放時越等距（命中越齊），但需 > GUIDED 盤旋半徑才到得了。
    """
    if not push_points:
        return False
    for sid, pp in push_points.items():
        p = positions.get(sid)
        if p is None or haversine(p[0], p[1], pp[0], pp[1]) > arrive_m:
            return False
    return True
