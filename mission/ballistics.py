"""投放彈體的自由飛行彈道（純邏輯，不依賴 PyQt / MAVLink，可離線單元測試）。

用途：SITL 面板按下投彈後，在 3D 地圖上即時畫出彈體從投放點落到地面的軌跡，
並在落點畫一個紅色圓形標示影響範圍。

**這是視覺化用的簡化模型，不是投放精度模型。**
明確不做的事（與 swarm_strike 的立場一致 —— 把限制講清楚比把成果講大聲重要）：

  * 沒有風場。實際落點會被風偏帶走，這裡完全不模擬。
  * 沒有升力、沒有攻角、沒有姿態動力學。彈體被當成一個只受重力與阻力的質點。
  * 阻力用單一係數 k（= ½ρ·Cd·A / m，單位 1/m）代表，且**不隨高度變化密度**。
  * 地面高度由呼叫端給定（預設用起飛場高），沒有地形查詢。

也就是說：畫出來的落點是「無風、給定阻力係數下的理想彈道」。
拿它當展示與時序參考可以，拿它當投放精度依據不行。

模型
----
    加速度 a = g·ẑ_down − k·|v|·v

    k 越大掉得越慢、越接近垂直落下（終端速度 = sqrt(g/k)）：
      dumb  k=0.0008 → 終端速度 ≈ 111 m/s（低阻力傻瓜彈，前拋距離明顯）
      smoke k=0.0200 → 終端速度 ≈  22 m/s（高阻力煙霧彈/減速彈，幾乎垂直落）

座標
----
以投放點為原點的局部 ENU（等距長方投影），數百公尺~數公里尺度誤差可忽略。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Tuple

G = 9.80665                  # m/s^2
_M_PER_DEG_LAT = 111320.0

# 彈種 → 阻力係數 k (1/m)。名字與 UI 上的選項對應。
# k = 阻力係數 (1/m)；blast_radius_m = 地圖上標示的影響範圍半徑 (m)。
# ⚠️ blast_radius_m 是**顯示用的示意值**，不是毀傷半徑 —— 本系統沒有毀傷模型
#    （見模組開頭「明確不做的事」）。
STORE_KINDS = {
    'dumb':  dict(k=0.0008, label='傻瓜彈',  color='#ff6b1a', blast_radius_m=8.0),
    'smoke': dict(k=0.0200, label='煙霧彈',  color='#c8c8c8', blast_radius_m=10.0),
}
DEFAULT_KIND = 'dumb'

_MAX_TOF_S = 180.0           # 模擬上限，避免異常輸入跑不完
_DT = 0.01                   # 積分步長 (s)
_SAMPLE_DT = 0.08            # 輸出取樣間隔 (s) — 給 Cesium 內插用，不必太密


@dataclass
class Trajectory:
    """一次投放的結果。"""
    samples: List[Tuple[float, float, float, float]] = field(default_factory=list)
    # 每筆 = (t 秒, lat, lon, alt_msl)
    impact_lat: float = 0.0
    impact_lon: float = 0.0
    impact_alt: float = 0.0      # = ground_msl
    tof: float = 0.0             # 飛行時間 (s)
    range_m: float = 0.0         # 投放點到落點的水平距離（＝前拋距離）
    impact_speed: float = 0.0    # 落地總速 (m/s)
    kind: str = DEFAULT_KIND

    def as_dict(self) -> dict:
        return dict(
            samples=[[round(t, 3), round(la, 7), round(lo, 7), round(al, 2)]
                     for (t, la, lo, al) in self.samples],
            impact=[round(self.impact_lat, 7), round(self.impact_lon, 7),
                    round(self.impact_alt, 2)],
            tof=round(self.tof, 3),
            range_m=round(self.range_m, 1),
            impact_speed=round(self.impact_speed, 1),
            kind=self.kind,
            blast_radius_m=STORE_KINDS.get(
                self.kind, STORE_KINDS[DEFAULT_KIND])['blast_radius_m'],
        )


def meters_per_deg(lat_deg: float) -> Tuple[float, float]:
    """該緯度下 (每經度公尺, 每緯度公尺)。"""
    return _M_PER_DEG_LAT * math.cos(math.radians(lat_deg)), _M_PER_DEG_LAT


def simulate(lat: float, lon: float, alt_msl: float,
             vn: float = 0.0, ve: float = 0.0, vd: float = 0.0,
             ground_msl: float = 0.0,
             kind: str = DEFAULT_KIND) -> Trajectory:
    """從 (lat, lon, alt_msl) 以速度 (vn, ve, vd) 投放，積分到落地。

    vn/ve = 北/東向速度 (m/s)，vd = **向下**為正 (m/s，與 MAVLink 的 vz 同號)。
    ground_msl = 地面海拔 (m)。alt_msl <= ground_msl 時直接回傳零飛行時間。
    """
    k = STORE_KINDS.get(kind, STORE_KINDS[DEFAULT_KIND])['k']
    m_lon, m_lat = meters_per_deg(lat)

    # 局部 ENU：n 北、e 東、d 下（相對投放點）
    n = e = 0.0
    h = float(alt_msl)                      # 目前海拔
    t = 0.0
    samples = [(0.0, lat, lon, h)]
    next_sample = _SAMPLE_DT

    if h <= ground_msl:
        # 已在地面（或參數不合理）：不模擬，直接落點 = 投放點
        tr = Trajectory(samples=samples, impact_lat=lat, impact_lon=lon,
                        impact_alt=float(ground_msl), tof=0.0, kind=kind)
        return tr

    def accel(_vn, _ve, _vd):
        sp = math.sqrt(_vn * _vn + _ve * _ve + _vd * _vd)
        # a = g·down − k·|v|·v
        return (-k * sp * _vn,
                -k * sp * _ve,
                G - k * sp * _vd)

    prev = (t, n, e, h, vn, ve, vd)
    while t < _MAX_TOF_S and h > ground_msl:
        # RK2（中點法）：對這個尺度足夠，且比 Euler 穩定得多
        an, ae, ad = accel(vn, ve, vd)
        mn, me, md = vn + an * _DT / 2, ve + ae * _DT / 2, vd + ad * _DT / 2
        an2, ae2, ad2 = accel(mn, me, md)

        prev = (t, n, e, h, vn, ve, vd)
        n += mn * _DT
        e += me * _DT
        h -= md * _DT                       # d 向下為正 ⇒ 高度減少
        vn += an2 * _DT
        ve += ae2 * _DT
        vd += ad2 * _DT
        t += _DT

        if t >= next_sample and h > ground_msl:
            samples.append((t, lat + n / m_lat, lon + e / m_lon, h))
            next_sample += _SAMPLE_DT

    # 在最後一步內線性內插出精確的觸地點（否則落點會落在步長誤差內）
    t0, n0, e0, h0, vn0, ve0, vd0 = prev
    if h0 > ground_msl and h0 != h:
        f = (h0 - ground_msl) / (h0 - h)
        f = min(max(f, 0.0), 1.0)
    else:
        f = 1.0
    ti = t0 + (t - t0) * f
    ni = n0 + (n - n0) * f
    ei = e0 + (e - e0) * f
    vni = vn0 + (vn - vn0) * f
    vei = ve0 + (ve - ve0) * f
    vdi = vd0 + (vd - vd0) * f

    ilat = lat + ni / m_lat
    ilon = lon + ei / m_lon
    samples.append((ti, ilat, ilon, float(ground_msl)))

    return Trajectory(
        samples=samples,
        impact_lat=ilat, impact_lon=ilon, impact_alt=float(ground_msl),
        tof=ti,
        range_m=math.hypot(ni, ei),
        impact_speed=math.sqrt(vni * vni + vei * vei + vdi * vdi),
        kind=kind,
    )


def simulate_from_frame(frame, ground_msl: float | None = None,
                        kind: str = DEFAULT_KIND) -> Trajectory:
    """由 SITLLink 的 TelemetryFrame 直接算一發。

    ground_msl 預設取「起飛場高」= alt_msl − alt_rel。這是最不需要額外資料、
    又對得上 SITL/實機的做法：投放高度是相對起飛點量的，落地就是回到起飛點高度。
    """
    if ground_msl is None:
        ground_msl = float(frame.alt_msl) - float(frame.alt_rel)
    vn = getattr(frame, 'vel_n', None)
    ve = getattr(frame, 'vel_e', None)
    vd = getattr(frame, 'vel_d', None)
    if vn is None or ve is None or vd is None:
        # 沒有速度分量時退回「地速沿航向 + 爬升率」。⚠️ 有風時航向 != 航跡，
        # 這個退路會有偏差；正常情況 GLOBAL_POSITION_INT 都有 vx/vy/vz。
        brg = math.radians(float(frame.heading))
        gs = float(frame.ground_speed)
        vn, ve, vd = gs * math.cos(brg), gs * math.sin(brg), -float(frame.climb)
    return simulate(float(frame.lat), float(frame.lon), float(frame.alt_msl),
                    float(vn), float(ve), float(vd),
                    ground_msl=float(ground_msl), kind=kind)
