"""風場資料（地圖顯示用）：規劃風、外部氣象網格（Open-Meteo）、飛機即時風的共同格式。純 Python（不用 Qt）。

風向一律用**氣象慣例**：``dir_from_deg`` ＝ 風「吹來」的方向，真北起順時針；u ＝ 東向分量、v ＝ 北向分量（空氣的移動方向）：
``u = -s·sin(dir)``、``v = -s·cos(dir)``（西風 270° ⇒ 往東吹 ⇒ u = +s）。HARVEST 規劃器的 ``hv_wind = (風速, 來向°)``
與 ArduPilot ``WIND`` 訊息的 direction 都是同一個慣例。

⚠ 三種風的意義不同，標籤（legend）一律寫明來源與限制（有效值契約）：
* **規劃風**：面板的平均風速／風向，規劃器假設全區均勻的一層風；不是實測。
* **外部氣象網格**：Open-Meteo 模式預報（不是實測），座標會被吸附到模式格點（解析度約 1–11 km，依模式），
  地形與局部陣風不會反映；要求的點數與實際對應到的模式格點數都寫進 notes。
* **飛機即時風**：飛控 EKF 的估計（MAVLink ``WIND``），要定翼飛行中才準；起飛前仍須現場實測。
"""
from __future__ import annotations

import json
import math
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable, Optional, Sequence

OPEN_METEO_URL = "https://api.open-meteo.com/v1/forecast"
#: 支援的高度：10 m 用「目前」值（current，15 分鐘級）；120 m（＝作業高度上限）用「這個小時」的逐時值
OPEN_METEO_HEIGHTS = {"10m": ("current", "wind_speed_10m", "wind_direction_10m"),
                      "120m": ("hourly", "wind_speed_120m", "wind_direction_120m")}
#: 一次請求的點數上限（URL 長度與 API 負擔；6×6 = 36 點就足以顯示 1–2 km 作業區一帶）
OPEN_METEO_MAX_POINTS = 100


class WindFetchError(RuntimeError):
    """抓外部風場失敗（訊息是給操作員看的繁中說明）。"""


def dir_to_uv(dir_from_deg: float, speed_mps: float) -> tuple[float, float]:
    """氣象風向（來向）＋風速 → (u 東向, v 北向)。"""
    d, s = math.radians(float(dir_from_deg)), float(speed_mps)
    return -s * math.sin(d), -s * math.cos(d)


def uv_to_dir(u: float, v: float) -> tuple[float, float]:
    """(u, v) → (來向°, 風速)；靜風回 (0, 0)。"""
    s = math.hypot(u, v)
    if s < 1e-9:
        return 0.0, 0.0
    return (math.degrees(math.atan2(-u, -v)) % 360.0), s


def _finite(x: Any) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(float(x))


@dataclass(frozen=True)
class WindField:
    """規則網格上的風（u／v 以 [i 緯度][j 經度] 排列，緯度、經度遞增）。單點 ＋ bbox ＝ 均勻場。
    缺值（模式沒給）用 NaN，顯示端略過那一格。"""
    lats: tuple
    lons: tuple
    u: tuple
    v: tuple
    height_m: float
    source: str                       # 'planning' | 'open-meteo'
    label: str                        # 圖例文字（來源、時間、限制）
    time_utc: Optional[str] = None
    bbox: Optional[tuple] = None      # (lat_min, lon_min, lat_max, lon_max)；均勻場必填
    notes: tuple = field(default_factory=tuple)

    def __post_init__(self) -> None:
        ny, nx = len(self.lats), len(self.lons)
        if ny < 1 or nx < 1:
            raise ValueError("風場至少要一個點")
        if list(self.lats) != sorted(self.lats) or list(self.lons) != sorted(self.lons):
            raise ValueError("風場的緯度、經度要遞增")
        for name, g in (("u", self.u), ("v", self.v)):
            if len(g) != ny or any(len(r) != nx for r in g):
                raise ValueError(f"風場 {name} 的形狀要是 {ny}×{nx}")
        if (ny == 1 or nx == 1) and self.bbox is None:
            raise ValueError("單列／單行的風場要給 bbox（覆蓋範圍），否則顯示端不知道畫多大")

    def speeds(self) -> list:
        return [math.hypot(a, b) for ru, rv in zip(self.u, self.v) for a, b in zip(ru, rv)
                if math.isfinite(a) and math.isfinite(b)]

    def max_speed(self) -> float:
        s = self.speeds()
        return max(s) if s else float("nan")

    def to_js(self, *, mode: str = "flow") -> dict:
        """顯示引擎（ui/widgets/web_shared/wind_flow.js）的 field 物件；NaN 轉 null（JSON 不能有 NaN）。"""
        def row(r):
            return [float(x) if math.isfinite(x) else None for x in r]
        d = {"lats": [float(x) for x in self.lats], "lons": [float(x) for x in self.lons],
             "u": [row(r) for r in self.u], "v": [row(r) for r in self.v],
             "height_m": float(self.height_m), "label": self.label, "mode": mode}
        if self.bbox is not None:
            d["bbox"] = [float(x) for x in self.bbox]
        return d


def area_bbox(polygons: Iterable[Sequence[Sequence[float]]], *, margin_m: float = 300.0) -> tuple:
    """作業區（[(lat, lon), …] 的串列）→ 外擴 margin_m 的 bbox (lat_min, lon_min, lat_max, lon_max)。"""
    pts = [(float(p[0]), float(p[1])) for ring in polygons for p in ring]
    pts = [p for p in pts if math.isfinite(p[0]) and math.isfinite(p[1])]
    if not pts:
        raise ValueError("沒有作業區可以決定風場範圍")
    la0, la1 = min(p[0] for p in pts), max(p[0] for p in pts)
    lo0, lo1 = min(p[1] for p in pts), max(p[1] for p in pts)
    dlat = float(margin_m) / 111_320.0
    dlon = float(margin_m) / (111_320.0 * max(0.01, math.cos(math.radians(0.5 * (la0 + la1)))))
    return (la0 - dlat, lo0 - dlon, la1 + dlat, lo1 + dlon)


def uniform_field(bbox: Sequence[float], speed_mps: float, dir_from_deg: float, *, height_m: float = 120.0,
                  label: Optional[str] = None, source: str = "planning") -> WindField:
    """均勻場（規劃風）：一個點 ＋ 覆蓋範圍。"""
    if not (_finite(speed_mps) and float(speed_mps) >= 0.0 and _finite(dir_from_deg)):
        raise ValueError(f"風速／風向不合法：{speed_mps!r}, {dir_from_deg!r}")
    u, v = dir_to_uv(dir_from_deg, speed_mps)
    la0, lo0, la1, lo1 = (float(x) for x in bbox)
    lab = label or (f"規劃風（面板假設、全區均勻，非實測）：{float(speed_mps):.1f} m/s 來自 "
                    f"{float(dir_from_deg) % 360:.0f}°")
    return WindField(lats=(0.5 * (la0 + la1),), lons=(0.5 * (lo0 + lo1),), u=((u,),), v=((v,),),
                     height_m=float(height_m), source=source, label=lab, bbox=(la0, lo0, la1, lo1))


def grid_points(bbox: Sequence[float], nx: int, ny: int) -> tuple[list, list]:
    """bbox 內 ny×nx 個均勻格點（含邊界）：(lats, lons)，都遞增。"""
    nx, ny = int(nx), int(ny)
    if nx < 2 or ny < 2:
        raise ValueError("外部風場網格至少 2×2")
    if nx * ny > OPEN_METEO_MAX_POINTS:
        raise ValueError(f"外部風場網格 {nx}×{ny} = {nx * ny} 點，超過上限 {OPEN_METEO_MAX_POINTS}")
    la0, lo0, la1, lo1 = (float(x) for x in bbox)
    if not (la1 > la0 and lo1 > lo0):
        raise ValueError(f"bbox 不合法：{bbox!r}")
    lats = [la0 + (la1 - la0) * i / (ny - 1) for i in range(ny)]
    lons = [lo0 + (lo1 - lo0) * j / (nx - 1) for j in range(nx)]
    return lats, lons


def build_open_meteo_url(lats: Sequence[float], lons: Sequence[float], height: str = "10m") -> str:
    """把 ny×nx 格點（逐列：緯度外層、經度內層）組成一次 Open-Meteo 多點請求。"""
    if height not in OPEN_METEO_HEIGHTS:
        raise ValueError(f"高度只能是 {sorted(OPEN_METEO_HEIGHTS)}，收到 {height!r}")
    kind, spd, dirn = OPEN_METEO_HEIGHTS[height]
    pts = [(la, lo) for la in lats for lo in lons]
    q = {"latitude": ",".join(f"{la:.5f}" for la, _lo in pts),
         "longitude": ",".join(f"{lo:.5f}" for _la, lo in pts),
         "wind_speed_unit": "ms", "timezone": "UTC"}
    if kind == "current":
        q["current"] = f"{spd},{dirn},wind_gusts_10m"
    else:
        q["hourly"] = f"{spd},{dirn}"
        q["forecast_hours"] = "1"
    return OPEN_METEO_URL + "?" + urllib.parse.urlencode(q)


def parse_open_meteo(payload: Any, lats: Sequence[float], lons: Sequence[float], height: str = "10m") -> WindField:
    """Open-Meteo 多點回應（list；單點時是 dict）→ WindField（格點位置用**要求的**座標，模式實際吸附到的格點寫進 notes）。"""
    kind, spd_k, dir_k = OPEN_METEO_HEIGHTS[height]
    items = payload if isinstance(payload, list) else [payload]
    n = len(lats) * len(lons)
    if len(items) != n:
        raise WindFetchError(f"Open-Meteo 回了 {len(items)} 筆，要求的是 {n} 點")
    u = [[math.nan] * len(lons) for _ in lats]
    v = [[math.nan] * len(lons) for _ in lats]
    times, snapped, gusts, missing = set(), set(), [], 0
    for k, it in enumerate(items):
        i, j = divmod(k, len(lons))
        if not isinstance(it, dict) or it.get("error"):
            raise WindFetchError(f"Open-Meteo 第 {k + 1} 點回應錯誤：{(it or {}).get('reason') if isinstance(it, dict) else it!r}")
        block = it.get(kind) or {}
        s, d = block.get(spd_k), block.get(dir_k)
        t = block.get("time")
        if kind == "hourly":
            s = s[0] if isinstance(s, list) and s else None
            d = d[0] if isinstance(d, list) and d else None
            t = t[0] if isinstance(t, list) and t else None
        else:
            g = block.get("wind_gusts_10m")
            if _finite(g):
                gusts.append(float(g))
        if t:
            times.add(str(t))
        snapped.add((round(float(it.get("latitude", math.nan)), 4), round(float(it.get("longitude", math.nan)), 4)))
        if _finite(s) and _finite(d):
            u[i][j], v[i][j] = dir_to_uv(float(d), float(s))
        else:
            missing += 1
    t_txt = ",".join(sorted(times)) or "時間未知"
    h_m = 10.0 if height == "10m" else 120.0
    notes = [f"要求 {n} 點，模式實際對應 {len(snapped)} 個格點（座標會吸附到模式格點；格點少代表解析度比網格粗）"]
    if missing:
        notes.append(f"{missing} 點沒有風值（顯示略過）")
    if gusts:
        notes.append(f"10 m 陣風最大 {max(gusts):.1f} m/s")
    label = (f"Open-Meteo 預報（模式資料，非實測）{height} 風，{t_txt} UTC；{len(snapped)} 個模式格點"
             + (f"；陣風最大 {max(gusts):.1f} m/s" if gusts else ""))
    return WindField(lats=tuple(lats), lons=tuple(lons), u=tuple(tuple(r) for r in u), v=tuple(tuple(r) for r in v),
                     height_m=h_m, source="open-meteo", label=label, time_utc=t_txt, notes=tuple(notes))


def fetch_open_meteo(bbox: Sequence[float], *, nx: int = 6, ny: int = 6, height: str = "10m", timeout_s: float = 20.0,
                     opener: Optional[Callable] = None) -> WindField:
    """抓 bbox 一帶的風場（會送出格點經緯度給 Open-Meteo；呼叫端要讓操作員明確按下才呼叫）。"""
    lats, lons = grid_points(bbox, nx, ny)
    url = build_open_meteo_url(lats, lons, height)
    op = opener or urllib.request.urlopen
    try:
        with op(url, timeout=timeout_s) as r:
            payload = json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        raise WindFetchError(f"Open-Meteo 回應 HTTP {e.code}：{e.reason}") from None
    except (urllib.error.URLError, TimeoutError, OSError) as e:
        raise WindFetchError(f"連不到 Open-Meteo（{e}）：要網路連線；離線時只能顯示規劃風與飛機即時風") from None
    except ValueError as e:
        raise WindFetchError(f"Open-Meteo 回應不是 JSON：{e}") from None
    if isinstance(payload, dict) and payload.get("error"):
        raise WindFetchError(f"Open-Meteo 拒絕請求：{payload.get('reason')}")
    return parse_open_meteo(payload, lats, lons, height)
