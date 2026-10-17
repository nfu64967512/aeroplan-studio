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

import dataclasses
import json
import math
import re
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


def field_mean_wind(f: WindField) -> Optional[tuple]:
    """網格的**向量平均**（寫入規劃風用）：(風速, 來向°, 有效格數)；沒有有效格回 None。
    向量平均會比各格風速的算術平均小（方向不一時互相抵銷），這是刻意的：規劃器要的是一層均勻風。"""
    su = sv = 0.0
    n = 0
    for ru, rv in zip(f.u, f.v):
        for a, b in zip(ru, rv):
            if math.isfinite(a) and math.isfinite(b):
                su += a
                sv += b
                n += 1
    if n == 0:
        return None
    d, s = uv_to_dir(su / n, sv / n)
    return s, d, n


# ─────────────────────────────────────────────────────────────────────
#  中央氣象署自動氣象站觀測（O-A0001-001）：真的量測，不是模式
# ─────────────────────────────────────────────────────────────────────
CWA_STATION_URL = "https://opendata.cwa.gov.tw/api/v1/rest/datastore/O-A0001-001"
#: CWA 特殊值（開放資料平臺資料標準說明文件 V1.05）：-99 ＝ 缺值或資料異常；X ＝ 儀器故障（字串，轉不成數字 ⇒ None）
CWA_MISSING = -99.0
#: 風向 990 ＝ 風向不定（不是 990°！取模會變成 270° 被當成西風）
CWA_DIR_VARIABLE = 990.0
#: WMO 地面風的**標準**觀測高度；⚠ CWA 官方文件沒有公布各站風速計的實際離地高度（局屬站可能在屋頂），
#:   所以只能說「標準 10 m」，不能說「該站離地 10 m」。不管哪一個都**不是作業高度**
CWA_ANEMOMETER_HEIGHT_M = 10.0
_CWA_ID_RE = re.compile(r"[A-Za-z][A-Za-z0-9]{4,6}|[0-9]{5,6}")


def looks_like_cwa_station_id(s: str) -> bool:
    """測站代碼（466920、C0A520、C1F9M1…）vs 測站名稱（中文）。"""
    return bool(_CWA_ID_RE.fullmatch((s or "").strip()))


def _num_cwa(x: Any) -> Optional[float]:
    """CWA 的數值欄位：-99 ＝ 缺值 → None；非數字 → None。"""
    try:
        if isinstance(x, bool) or x is None:
            return None
        v = float(x)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(v) or abs(v - CWA_MISSING) < 1e-9:
        return None
    return v


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6_371_000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(min(1.0, math.sqrt(a)))


@dataclass(frozen=True)
class StationWind:
    """一個自動氣象站的一筆觀測（風是 10 分鐘平均；陣風是該小時內最大）。缺值 None。"""
    station_id: str
    name: str
    lat: float
    lon: float
    alt_m: Optional[float]
    obs_time: str
    speed_mps: Optional[float]
    dir_from_deg: Optional[float]
    gust_mps: Optional[float] = None
    gust_dir_deg: Optional[float] = None
    gust_time: Optional[str] = None
    pressure_hpa: Optional[float] = None
    county: str = ""
    town: str = ""
    distance_m: Optional[float] = None      # 距作業區中心（找最近測站時才有）
    matched_by: str = ""                    # 'id' | 'exact' | 'substring' | 'nearest'（＋'+nearest'）：怎麼挑到的，退回要看得見
    dir_variable: bool = False              # 風向回報 990（風向不定）：dir_from_deg 為 None，但風速可能有值

    def has_wind(self) -> bool:
        """有風速**且**有確定的風向才算（風向不定的不能畫箭頭、也不能寫進規劃器）。"""
        return self.speed_mps is not None and self.dir_from_deg is not None

    def title(self) -> str:
        return f"{self.name}（{self.station_id}）"

    def how_matched(self) -> str:
        m = self.matched_by or ""
        if m.startswith("substring"):
            return "名稱以包含比對命中" + ("、多站取最近" if m.endswith("+nearest") else "")
        if m == "nearest":
            return "離作業區最近且有風值"
        if m.endswith("+nearest"):
            return "多站取最近"
        return ""

    def summary(self) -> str:
        """狀態列／圖例用的一行：來源、時間、風、陣風、距離、高度提醒。"""
        how = self.how_matched()
        parts = [f"CWA 測站 {self.title()}{'（' + how + '）' if how else ''} 觀測 {self.obs_time or '時間未知'}："]
        if self.has_wind():
            parts.append(f"風 {self.speed_mps:.1f} m/s 來自 {self.dir_from_deg:.0f}°")
        elif self.dir_variable and self.speed_mps is not None:
            parts.append(f"風 {self.speed_mps:.1f} m/s、風向不定（990）")
        elif self.dir_variable:
            parts.append("風速缺值、風向不定（990）")
        else:
            parts.append("這筆沒有風值（-99／X）")
        if self.gust_mps is not None:
            parts.append(f"；陣風 {self.gust_mps:.1f} m/s" + (f" 來自 {self.gust_dir_deg:.0f}°" if self.gust_dir_deg is not None else ""))
        if self.pressure_hpa is not None:
            parts.append(f"；氣壓 {self.pressure_hpa:.1f} hPa")
        if self.distance_m is not None:
            parts.append(f"；距作業區中心 {self.distance_m / 1000.0:.1f} km")
        parts.append(f"；地面測站風（標準觀測高度 {CWA_ANEMOMETER_HEIGHT_M:.0f} m，CWA 未公布該站實際離地高度；"
                     "非作業高度，通常比 100–120 m 的風小）")
        return "".join(parts)

    def to_arrow(self, sysid: int) -> dict:
        """地圖箭頭列（ui.widgets.wind_layer.wind_arrows_to_compact 的輸入）。"""
        return {"sysid": int(sysid), "lat": self.lat, "lon": self.lon, "alt_rel": 0.0,
                "dir_from_deg": self.dir_from_deg, "speed_mps": self.speed_mps,
                "label": f"CWA {self.name} 觀測 {(self.obs_time or '')[11:16]}"}


def short_text(s: str, n: int = 12) -> str:
    """回顯使用者輸入時截斷：太長的字串（例如貼錯欄位的授權碼）不能整串進 log／狀態列。"""
    s = str(s or "")
    return s if len(s) <= n else f"{s[:n]}…（共 {len(s)} 字）"


def build_cwa_url(key: str, *, station_id: Optional[str] = None) -> str:
    """O-A0001-001 的請求 URL（授權碼在查詢字串裡 ⇒ 這個字串**不能**進 log，用 redact_cwa_url）。"""
    if not key:
        raise ValueError("沒有 CWA API 授權碼")
    q = {"Authorization": key, "format": "JSON"}
    if station_id:
        q["StationId"] = station_id.strip().upper()
    return CWA_STATION_URL + "?" + urllib.parse.urlencode(q)


def redact_cwa_url(url: str) -> str:
    return re.sub(r"(Authorization=)[^&]*", r"\1***", url)


def parse_cwa_stations(payload: Any) -> list:
    """O-A0001-001 的 JSON → [StationWind]（含沒有風值的站；WGS84 座標；缺經緯度的站略過）。
    結構不對就丟 WindFetchError（附上拿到的鍵，不猜）。"""
    if not isinstance(payload, dict):
        raise WindFetchError(f"CWA 回應不是 JSON 物件：{type(payload).__name__}")
    if str(payload.get("success", "")).lower() != "true":
        raise WindFetchError(f"CWA 回應 success={payload.get('success')!r}"
                             + (f"：{payload.get('message')}" if payload.get("message") else ""))
    recs = payload.get("records")
    stations = recs.get("Station") if isinstance(recs, dict) else None
    if not isinstance(stations, list):
        raise WindFetchError("CWA 回應沒有 records.Station（鍵："
                             f"{sorted(payload)} / {sorted(recs) if isinstance(recs, dict) else type(recs).__name__}）")
    out = []
    for s in stations:
        if not isinstance(s, dict):
            continue
        geo = s.get("GeoInfo") or {}
        coords = geo.get("Coordinates") or []
        wgs = next((c for c in coords if isinstance(c, dict) and "WGS84" in str(c.get("CoordinateName", "")).upper()),
                   coords[0] if coords and isinstance(coords[0], dict) else {})
        lat, lon = _num_cwa(wgs.get("StationLatitude")), _num_cwa(wgs.get("StationLongitude"))
        if lat is None or lon is None or abs(lat) > 90 or abs(lon) > 180:
            continue
        we = s.get("WeatherElement") or {}
        gust = we.get("GustInfo") or {}
        occ = gust.get("Occurred_at") or {}
        gust_t = occ.get("DateTime")
        gust_t = str(gust_t) if gust_t and str(gust_t).strip() not in ("-99", "-99.0") else None
        wd, gd = _num_cwa(we.get("WindDirection")), _num_cwa(occ.get("WindDirection"))
        wd_var = wd is not None and abs(wd - CWA_DIR_VARIABLE) < 1e-9
        gd_var = gd is not None and abs(gd - CWA_DIR_VARIABLE) < 1e-9
        # 990 以外超出 0–360 的風向不是合法值：當成缺值（不取模，取模會靜默變成別的方向）
        if wd is not None and not wd_var and not (0.0 <= wd <= 360.0):
            wd = None
        if gd is not None and not gd_var and not (0.0 <= gd <= 360.0):
            gd = None
        out.append(StationWind(
            station_id=str(s.get("StationId") or ""), name=str(s.get("StationName") or ""),
            lat=lat, lon=lon, alt_m=_num_cwa(geo.get("StationAltitude")),
            obs_time=str((s.get("ObsTime") or {}).get("DateTime") or ""),
            speed_mps=_num_cwa(we.get("WindSpeed")), dir_from_deg=wd if not wd_var else None,
            dir_variable=wd_var,
            gust_mps=_num_cwa(gust.get("PeakGustSpeed")), gust_dir_deg=gd if not gd_var else None,
            gust_time=gust_t,
            pressure_hpa=_num_cwa(we.get("AirPressure")),
            county=str(geo.get("CountyName") or ""), town=str(geo.get("TownName") or "")))
    return out


def pick_cwa_station(stations: Sequence[StationWind], *, station: str = "",
                     near: Optional[Sequence[float]] = None) -> StationWind:
    """挑一站：給代碼 ⇒ 代碼相等；給名稱 ⇒ 先全等、再包含；多站命中時有 near 取最近、沒有 near 就丟錯列出候選
    （不靜默取第一個）；都沒給 ⇒ 離 near 最近且有風值。選到的站沒有風值 ⇒ 丟錯（不拿別站代替）。
    回傳的 matched_by 記錄怎麼挑到的（退回到包含比對要看得見）。"""
    want = (station or "").strip()
    with_d = list(stations)
    if near is not None:
        la, lo = float(near[0]), float(near[1])
        with_d = [dataclasses.replace(s, distance_m=haversine_m(la, lo, s.lat, s.lon)) for s in stations]

    def nearest(cands):
        return min(cands, key=lambda s: (s.distance_m if s.distance_m is not None else float("inf"), s.station_id))

    if want:
        if looks_like_cwa_station_id(want):
            hit = [s for s in with_d if s.station_id.upper() == want.upper()]
            what, how = f"代碼 {want.upper()}", "id"
        else:
            exact = [s for s in with_d if s.name == want]
            hit = exact or [s for s in with_d if want in s.name]
            what, how = f"名稱「{short_text(want)}」", ("exact" if exact else "substring")
        if not hit:
            hint = ""
            if near is not None and with_d:
                top = sorted((s for s in with_d if s.has_wind()), key=lambda s: s.distance_m)[:3]
                hint = "；離作業區最近有風值的：" + "、".join(f"{s.title()} {s.distance_m / 1000:.1f} km" for s in top)
            raise WindFetchError(f"CWA 找不到測站{what}（共 {len(with_d)} 站）{hint}")
        if len(hit) > 1 and near is None:
            raise WindFetchError(f"CWA {what}有 {len(hit)} 站符合：" + "、".join(
                f"{s.title()} {s.county}{s.town}" for s in hit[:8]) + ("…" if len(hit) > 8 else "")
                + "；請改填測站代碼或全名，或先圈作業區／選起降場讓它取最近的")
        chosen = nearest(hit) if len(hit) > 1 else hit[0]
        if len(hit) > 1:
            how += "+nearest"
    else:
        if near is None:
            raise WindFetchError("沒給測站也沒有作業區／起降場可以找最近測站")
        cands = [s for s in with_d if s.has_wind()]
        if not cands:
            raise WindFetchError(f"CWA {len(with_d)} 站這一筆都沒有風值")
        chosen, how = nearest(cands), "nearest"
    if not chosen.has_wind():
        why = ("風向不定（990）" + (f"、風速 {chosen.speed_mps:.1f} m/s" if chosen.speed_mps is not None else "")
               if chosen.dir_variable else "沒有風值（-99／X）")
        raise WindFetchError(f"CWA 測站 {chosen.title()} 這筆觀測（{chosen.obs_time or '時間未知'}）{why}，"
                             "不能畫箭頭也不能寫進規劃器；沒有拿別站代替，換一站或等下一筆")
    return dataclasses.replace(chosen, matched_by=how)


def fetch_cwa_station(key: str, *, station: str = "", near: Optional[Sequence[float]] = None,
                      timeout_s: float = 20.0, opener: Optional[Callable] = None) -> StationWind:
    """抓 O-A0001-001：給代碼就只要那一站，否則抓全部在本機挑（名稱包含／最近）。錯誤訊息不含授權碼。"""
    if not key:
        raise WindFetchError("沒有 CWA API 授權碼（config/settings.json 的 weather.cwa_api_key 或環境變數 CWA_API_KEY）")
    want = (station or "").strip()
    url = build_cwa_url(key, station_id=want if looks_like_cwa_station_id(want) else None)
    op = opener or urllib.request.urlopen
    try:
        with op(url, timeout=timeout_s) as r:
            payload = json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        extra = "（授權碼不對或沒開通這個資料集）" if e.code in (401, 403) else ""
        raise WindFetchError(f"CWA 回應 HTTP {e.code}：{e.reason}{extra}") from None
    except (urllib.error.URLError, TimeoutError, OSError) as e:
        raise WindFetchError(f"連不到 CWA（{redact_cwa_url(str(e))}）：要網路連線") from None
    except ValueError as e:
        raise WindFetchError(f"CWA 回應不是 JSON：{e}") from None
    return pick_cwa_station(parse_cwa_stations(payload), station=want, near=near)
