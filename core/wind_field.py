"""風場資料（地圖顯示用）：規劃風、外部氣象網格（Open-Meteo）、飛機即時風的共同格式。純 Python（不用 Qt）。

風向一律用**氣象慣例**：``dir_from_deg`` ＝ 風「吹來」的方向，真北起順時針；u ＝ 東向分量、v ＝ 北向分量（空氣的移動方向）：
``u = -s·sin(dir)``、``v = -s·cos(dir)``（西風 270° ⇒ 往東吹 ⇒ u = +s）。HARVEST 規劃器的 ``hv_wind = (風速, 來向°)``
與 ArduPilot ``WIND`` 訊息的 direction 都是同一個慣例。

⚠ 三種風的意義不同，標籤（legend）一律寫明來源與限制（有效值契約）：
* **規劃風**：面板的平均風速／風向，規劃器假設全區均勻的一層風；不是實測。
* **外部氣象網格**：Open-Meteo 模式預報（不是實測），座標會被吸附到模式格點（解析度依模式：分層風固定用的
  icon_global 實際供應的是 0.125° 格點，約 13–14 km；Open-Meteo 文件標示 ~11 km），地形與局部陣風不會反映；
  要求的點數與實際對應到的模式格點數都寫進 notes（作業區一帶 6×6 點常常只落在 1–4 個模式格點）。
  **分層風**（WindProfile）：同一個模式、同一個逐時時間步的 10／80／120 m（離地）三層；層間不內插、不外推。
* **飛機即時風**：飛控 EKF 的估計（MAVLink ``WIND``），要定翼飛行中才準；起飛前仍須現場實測。
"""
from __future__ import annotations

import bisect
import dataclasses
import datetime
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

    def to_page(self, *, mode: str = "flow") -> dict:
        """to_js() 的別名（分層風的頁面 payload 用；兩個名字給的是同一個 dict）。"""
        return self.to_js(mode=mode)


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
#  分層風（10／80／120 m）：Open-Meteo 同一個模式、同一個逐時時間步
# ─────────────────────────────────────────────────────────────────────
#: 使用者定調（2026-10-19）：先只做這三層；層與層之間**不內插**、範圍外**不外推**
PROFILE_HEIGHTS_M = (10.0, 80.0, 120.0)
#: 三層固定用同一個模式。⚠ 不用 best_match：它可能在不同高度混用不同模式，而 GFS 的 120 m 是從 100 m 以指數律推出來的。
#:   2026-10-19 實測（tests/fixtures/open_meteo_profile_*）：icon_global 逐時 10／80／120／180 m 都有值、
#:   單一模式時回應鍵不帶後綴（wind_speed_80m）、多模式時帶 ``_<model>`` 後綴
PROFILE_MODEL_DEFAULT = "icon_global"
#: Open-Meteo 逐時風提供的離地高度（m）
PROFILE_HEIGHTS_SUPPORTED = (10, 80, 120, 180)
_MODEL_RE = re.compile(r"[a-z0-9_]{1,64}")
#: 已實測的模式格點間距（度）：2026-10-19 icon_global 回應的座標全落在 0.125° 格點上
#:   （23.4／23.6 → 23.375／23.625；120.3／120.45／120.6 → 120.25／120.5／120.625）。Open-Meteo 文件寫 ICON Global
#:   0.1°（~11 km），實際供應的是 0.125°。沒實測過的模式不寫數字（notes 改寫回應裡看得到的最小間距）
PROFILE_MODEL_GRID_DEG = {"icon_global": 0.125}
#: 請求固定 wind_speed_unit=ms、timezone=UTC；回應的單位／時差不是這個就丟錯（不換算、不猜）
PROFILE_SPEED_UNIT = "m/s"
PROFILE_DIR_UNIT = "°"


def _fmt_h(h: float) -> str:
    return f"{float(h):g}"


def _check_profile_heights(heights: Iterable[Any]) -> tuple:
    """高度清單 → 遞增的 float tuple；只收 Open-Meteo 有的 10／80／120／180（不收其他高度、不收重複）。"""
    try:
        hs = list(heights)
    except TypeError:
        raise ValueError(f"高度要是清單：{heights!r}") from None
    out = []
    for h in hs:
        if not _finite(h) or float(h) != int(float(h)) or int(float(h)) not in PROFILE_HEIGHTS_SUPPORTED:
            raise ValueError(f"分層風的高度只能是 {'／'.join(str(x) for x in PROFILE_HEIGHTS_SUPPORTED)} m，收到 {h!r}")
        out.append(float(int(float(h))))
    if not out:
        raise ValueError("分層風至少要一個高度")
    if len(set(out)) != len(out):
        raise ValueError(f"分層風的高度重複：{hs!r}")
    return tuple(sorted(out))


def _check_model(model: Any) -> str:
    m = str(model or "")
    if not _MODEL_RE.fullmatch(m):
        raise ValueError(f"模式名稱不合法：{model!r}（Open-Meteo 的 models 參數，例如 icon_global）")
    return m


@dataclass(frozen=True)
class WindProfile:
    """同一個模式、同一個逐時時間步的幾層風（每層一個 WindField，高度遞增；只放**有資料**的層）。
    ⚠ 只有這幾層：層與層之間不內插、最低層以下與最高層以上不外推（使用者定調）。高度是模式地表以上（離地）。"""
    layers: tuple
    model: str
    time_utc: str
    label: str
    notes: tuple = field(default_factory=tuple)
    bbox: Optional[tuple] = None      # (lat_min, lon_min, lat_max, lon_max)

    def __post_init__(self) -> None:
        layers = tuple(self.layers or ())
        if not layers:
            raise ValueError("分層風至少要有一層")
        if not all(isinstance(f, WindField) for f in layers):
            raise ValueError("分層風的每一層都要是 WindField")
        hs = [float(f.height_m) for f in layers]
        if any(b <= a for a, b in zip(hs, hs[1:])):
            raise ValueError(f"分層風的高度要嚴格遞增：{hs}")
        object.__setattr__(self, "layers", layers)
        object.__setattr__(self, "notes", tuple(str(n) for n in (self.notes or ())))
        if self.bbox is not None:
            bb = tuple(float(x) for x in self.bbox)
            if len(bb) != 4 or not all(math.isfinite(x) for x in bb) or bb[2] < bb[0] or bb[3] < bb[1]:
                raise ValueError(f"bbox 不合法：{self.bbox!r}")
            object.__setattr__(self, "bbox", bb)

    def heights(self) -> list:
        return [float(f.height_m) for f in self.layers]

    def layer(self, h_m: Any) -> Optional[WindField]:
        """剛好那個高度的層；沒有就 None（不找最接近的，要最接近的用 profile_wind_at）。"""
        if not _finite(h_m):
            return None
        return next((f for f in self.layers if abs(float(f.height_m) - float(h_m)) < 1e-6), None)

    def to_json(self) -> dict:
        """可 JSON 化的摘要（沒有 NaN；不含網格本身，網格用各層的 to_js()）。"""
        def num(x):
            return float(x) if math.isfinite(x) else None
        return {"source": "open-meteo", "model": self.model, "time_utc": self.time_utc, "label": self.label,
                "notes": list(self.notes), "bbox": list(self.bbox) if self.bbox is not None else None,
                "heights_m": self.heights(), "height_reference": "離地（模式地表以上）",
                "interpolation": "none",
                "layers": [{"height_m": float(f.height_m), "label": f.label, "time_utc": f.time_utc,
                            "n_valid": len(f.speeds()), "max_speed_mps": num(f.max_speed())} for f in self.layers]}


def build_open_meteo_profile_url(lats: Sequence[float], lons: Sequence[float], *,
                                 heights: Iterable[float] = PROFILE_HEIGHTS_M,
                                 model: str = PROFILE_MODEL_DEFAULT) -> str:
    """ny×nx 格點（逐列：緯度外層、經度內層，同 build_open_meteo_url）的分層逐時風請求：
    每個高度 ``wind_speed_<h>m``＋``wind_direction_<h>m``，全部 hourly（同一個時間步）、指定單一模式、這個小時一筆。"""
    hs = _check_profile_heights(heights)
    m = _check_model(model)
    pts = [(float(la), float(lo)) for la in lats for lo in lons]
    if not pts:
        raise ValueError("沒有格點")
    if len(pts) > OPEN_METEO_MAX_POINTS:
        raise ValueError(f"外部風場 {len(pts)} 點，超過上限 {OPEN_METEO_MAX_POINTS}")
    q = {"latitude": ",".join(f"{la:.5f}" for la, _lo in pts),
         "longitude": ",".join(f"{lo:.5f}" for _la, lo in pts),
         "hourly": ",".join(f"wind_speed_{_fmt_h(h)}m,wind_direction_{_fmt_h(h)}m" for h in hs),
         "models": m, "wind_speed_unit": "ms", "timezone": "UTC", "forecast_hours": "1"}
    return OPEN_METEO_URL + "?" + urllib.parse.urlencode(q)


def _utc_hour(x: Any) -> Optional[datetime.datetime]:
    """datetime／ISO 字串 → 取整到小時的 naive UTC datetime；看不懂回 None。naive 輸入當成 UTC。"""
    if isinstance(x, datetime.datetime):
        d = x
    elif isinstance(x, str) and x.strip():
        s = x.strip()
        if s.endswith(("Z", "z")):
            s = s[:-1] + "+00:00"
        try:
            d = datetime.datetime.fromisoformat(s)
        except ValueError:
            return None
    else:
        return None
    if d.tzinfo is not None:
        d = d.astimezone(datetime.timezone.utc).replace(tzinfo=None)
    return d.replace(minute=0, second=0, microsecond=0)


def _series(hourly: dict, key: str, model: str) -> Any:
    """多模式回應的鍵帶 ``_<model>`` 後綴；單一模式不帶。只認**這個**模式，不拿別的模式頂替。"""
    if f"{key}_{model}" in hourly:
        return hourly[f"{key}_{model}"]
    return hourly.get(key)


def _unit_of(units: dict, key: str, model: str) -> Any:
    """hourly_units 裡這個變數的單位（後綴規則同 _series）；沒有回 None。"""
    if f"{key}_{model}" in units:
        return units[f"{key}_{model}"]
    return units.get(key)


def _grid_spacing_note(model: str, cells: set, lat_mid: float) -> str:
    """模式格點間距：實測過的模式寫度數與公里；沒實測過的寫回應裡吸附到的格點最小間距（量得到才寫）。"""
    deg = PROFILE_MODEL_GRID_DEG.get(model)
    if deg is not None:
        km_ns = deg * 111.32
        km_ew = deg * 111.32 * max(0.01, math.cos(math.radians(lat_mid)))
        return (f"模式 {model} 的格點間距 {deg:g}°（這裡約 {km_ns:.1f} km × {km_ew:.1f} km；"
                "2026-10-19 實測回應座標，Open-Meteo 文件標示 ~11 km）")
    diffs = []
    for axis in (0, 1):
        vals = sorted({c[axis] for c in cells})
        diffs += [b - a for a, b in zip(vals, vals[1:]) if b - a > 1e-9]
    if diffs:
        return f"模式 {model} 的格點間距沒有實測過；這次回應吸附到的格點最小間距 {min(diffs):g}°"
    return f"模式 {model} 的格點間距沒有實測過（這次只對應到 1 個格點，量不出間距）"


def parse_open_meteo_profile(payload: Any, lats: Sequence[float], lons: Sequence[float], *,
                             heights: Iterable[float] = PROFILE_HEIGHTS_M, model: str = PROFILE_MODEL_DEFAULT,
                             when_utc: Any = None) -> WindProfile:
    """Open-Meteo 分層逐時回應（多點是 list；單點是 dict）→ WindProfile。

    * 時間步：取時間等於 when_utc（預設現在）所在 UTC 小時的那一筆；回應裡沒有那個小時（時鐘差、跨整點）就跟
      既有 120 m 路徑一樣取伺服器給的第一筆（forecast_hours=1 ⇒ 伺服器的「這個小時」），並寫進 notes。
      **所有高度、所有點都用同一個時間步**；各點時間對不起來就丟錯，不混用。
    * 某一層在所有點都沒有值（模式不提供或全 null）⇒ 整層略過並寫 note；不從其他層內插或外推。
    * 一層都沒有 ⇒ WindFetchError；結構不對 ⇒ WindFetchError（繁中、附實際的鍵）。
    * 不只相信 URL：回應的 ``hourly_units`` 風速不是 m/s、風向不是 °，或 ``utc_offset_seconds`` 不是 0
      ⇒ WindFetchError（不換算、不把當地時間標成 UTC）；回應沒附這兩項 ⇒ 照請求解讀並寫 note「未能核對」。"""
    hs = _check_profile_heights(heights)
    m = _check_model(model)
    lats = [float(x) for x in lats]
    lons = [float(x) for x in lons]
    if len(lats) < 2 or len(lons) < 2:
        raise ValueError("分層風場網格至少 2×2")
    if lats != sorted(lats) or lons != sorted(lons):
        raise ValueError("風場的緯度、經度要遞增")
    if isinstance(payload, dict) and payload.get("error"):
        raise WindFetchError(f"Open-Meteo 拒絕請求：{payload.get('reason')}")
    if isinstance(payload, dict):
        items = [payload]
    elif isinstance(payload, list):
        items = payload
    else:
        raise WindFetchError(f"Open-Meteo 回應格式不對：要 list 或 dict，收到 {type(payload).__name__}")
    n = len(lats) * len(lons)
    if len(items) != n:
        raise WindFetchError(f"Open-Meteo 回了 {len(items)} 筆，要求的是 {n} 點")
    target = _utc_hour(when_utc if when_utc is not None else datetime.datetime.now(datetime.timezone.utc))
    if target is None:
        raise ValueError(f"when_utc 看不懂：{when_utc!r}")
    target_txt = target.strftime("%Y-%m-%dT%H:%M")

    grids = {h: ([[math.nan] * len(lons) for _ in lats], [[math.nan] * len(lons) for _ in lats]) for h in hs}
    valid = {h: 0 for h in hs}
    present = {h: 0 for h in hs}                 # 回應裡有這一層欄位的點數（區分「模式不提供」與「全 null」）
    cells_ok = {h: set() for h in hs}
    times, snapped, fallback = set(), set(), False
    no_units = no_unit_keys = no_offset = 0
    elevs = []
    for k, it in enumerate(items):
        i, j = divmod(k, len(lons))
        if not isinstance(it, dict) or it.get("error"):
            why = it.get("reason") if isinstance(it, dict) else it
            raise WindFetchError(f"Open-Meteo 第 {k + 1} 點回應錯誤：{why!r}")
        hourly = it.get("hourly")
        if not isinstance(hourly, dict):
            raise WindFetchError(f"Open-Meteo 第 {k + 1} 點沒有 hourly 區塊（鍵：{sorted(it)}）")
        off = it.get("utc_offset_seconds")
        if off is None:
            no_offset += 1
        elif not _finite(off) or float(off) != 0.0:
            raise WindFetchError(f"Open-Meteo 第 {k + 1} 點的回應不是 UTC 時間（utc_offset_seconds={off!r}、"
                                 f"timezone={it.get('timezone')!r}）：請求固定 timezone=UTC；不換算、不把當地時間標成 UTC")
        units = it.get("hourly_units")
        if units is None:
            no_units += 1
        elif not isinstance(units, dict):
            raise WindFetchError(f"Open-Meteo 第 {k + 1} 點的 hourly_units 不是物件：{units!r}")
        if _finite(it.get("elevation")):
            elevs.append(float(it.get("elevation")))
        tl = hourly.get("time")
        if not isinstance(tl, list) or not tl or not all(isinstance(t, str) for t in tl):
            raise WindFetchError(f"Open-Meteo 第 {k + 1} 點的 hourly.time 不是時間字串清單：{tl!r}")
        idx = next((x for x, t in enumerate(tl) if _utc_hour(t) == target), None)
        if idx is None:
            idx, fallback = 0, True
        times.add(tl[idx])
        la_s, lo_s = it.get("latitude"), it.get("longitude")
        cell = (round(float(la_s), 4), round(float(lo_s), 4)) if _finite(la_s) and _finite(lo_s) else None
        if cell is not None:
            snapped.add(cell)
        for h in hs:
            vals = []
            for key in (f"wind_speed_{_fmt_h(h)}m", f"wind_direction_{_fmt_h(h)}m"):
                ser = _series(hourly, key, m)
                if ser is None:
                    vals.append(None)
                    continue
                if not isinstance(ser, list) or len(ser) != len(tl):
                    got = len(ser) if isinstance(ser, list) else type(ser).__name__
                    raise WindFetchError(f"Open-Meteo 第 {k + 1} 點的 {key} 長度 {got} 與時間 {len(tl)} 筆不符")
                if isinstance(units, dict):
                    want = PROFILE_SPEED_UNIT if key.startswith("wind_speed_") else PROFILE_DIR_UNIT
                    unit = _unit_of(units, key, m)
                    if unit is None:
                        no_unit_keys += 1
                    elif unit != want:
                        what = "風速" if key.startswith("wind_speed_") else "風向"
                        raise WindFetchError(f"Open-Meteo 第 {k + 1} 點的{what} {key} 單位是 {unit!r}，不是 {want!r}"
                                             "（請求固定 wind_speed_unit=ms；不換算、不猜）")
                vals.append(ser[idx])
            spd, d = vals
            if _series(hourly, f"wind_speed_{_fmt_h(h)}m", m) is not None:
                present[h] += 1
            # 負風速、0–360 以外的風向不是合法值：當缺值（不取模，取模會靜默變成別的方向）
            if _finite(spd) and _finite(d) and float(spd) >= 0.0 and 0.0 <= float(d) <= 360.0:
                u, v = dir_to_uv(float(d), float(spd))
                grids[h][0][i][j], grids[h][1][i][j] = u, v
                valid[h] += 1
                if cell is not None:
                    cells_ok[h].add(cell)
    if len(times) != 1:
        raise WindFetchError(f"Open-Meteo 各點的預報時間不一致：{sorted(times)}（不混用不同時間步）")
    t_txt = next(iter(times))

    layers, dropped, notes = [], [], []
    for h in hs:
        hm = _fmt_h(h)
        if valid[h] == 0:
            dropped.append(hm)
            if present[h] == 0:
                notes.append(f"{hm} m：回應沒有這一層（模式 {m} 不提供 wind_speed_{hm}m），已略過；不從其他高度內插或外推")
            else:
                notes.append(f"{hm} m：模式 {m} 這個時間步在 {n} 個點都沒有風值，已略過；不從其他高度內插或外推")
            continue
        u, v = grids[h]
        lab = f"Open-Meteo 預報（{m}，非實測）{hm} m 風，{t_txt} UTC；{len(cells_ok[h])} 個模式格點"
        lnotes = [f"要求 {n} 點，有值的對應 {len(cells_ok[h])} 個模式格點（座標會吸附到模式格點）"]
        if valid[h] < n:
            lnotes.append(f"{n - valid[h]} 點沒有風值（顯示略過）")
            notes.append(f"{hm} m：{n - valid[h]} 點沒有風值（顯示略過）")
        layers.append(WindField(lats=tuple(lats), lons=tuple(lons), u=tuple(tuple(r) for r in u),
                                v=tuple(tuple(r) for r in v), height_m=float(h), source="open-meteo", label=lab,
                                time_utc=t_txt, notes=tuple(lnotes)))
    if not layers:
        raise WindFetchError(f"Open-Meteo（模式 {m}）在 {'／'.join(_fmt_h(h) for h in hs)} m 都沒有風值："
                             + "；".join(notes))
    have = "／".join(_fmt_h(f.height_m) for f in layers)
    head = [f"{have} m 都取同一個逐時時間步 {t_txt} UTC（Open-Meteo「這個小時」的預報值，不是現在的觀測）",
            f"所有高度固定用模式 {m}（不用 best_match，避免不同高度混用不同模式）",
            f"只有 {have} m 這幾層：高度是模式地表以上（離地），層與層之間不內插、範圍外不外推",
            f"要求 {n} 點，模式實際對應 {len(snapped)} 個格點（座標會吸附到模式格點；格點少代表解析度比網格粗）",
            _grid_spacing_note(m, snapped, 0.5 * (lats[0] + lats[-1]))]
    if len(snapped) == 1:
        head.append(f"要求的 {n} 點全部對應同一個模式格點 ⇒ 範圍內各點的風完全相同；圖上看到的差異不是模式解析出來的風場梯度")
    elif 1 < len(snapped) < n:
        head.append(f"要求的 {n} 點只對應 {len(snapped)} 個模式格點 ⇒ 範圍內的變化只來自這幾個格點的分界，"
                    "不是解析出來的風場梯度")
    if elevs:
        head.append(f"回應的 elevation 是要求點的 90 m DEM 高程 {min(elevs):.0f}–{max(elevs):.0f} m（不是模式地表）；"
                    "各層的「離地」是相對模式格點地表，與實際地形可能差很多")
    if no_units or no_unit_keys:
        head.append(f"{no_units + no_unit_keys} 處回應沒有附單位（hourly_units）：依請求的 wind_speed_unit=ms 解讀，未能核對")
    if no_offset:
        head.append(f"{no_offset} 點的回應沒有 utc_offset_seconds：時間依請求的 timezone=UTC 解讀，未能核對")
    if fallback:
        head.append(f"回應的時間 {t_txt} UTC 不是本機時鐘的 {target_txt} UTC（時鐘差或剛跨整點），"
                    "用伺服器給的第一筆（伺服器的「這個小時」）")
    label = (f"Open-Meteo 預報（{m}，非實測）分層 {have} m 風，{t_txt} UTC；{len(snapped)} 個模式格點"
             + (f"；{'／'.join(dropped)} m 無資料已略過" if dropped else ""))
    return WindProfile(layers=tuple(layers), model=m, time_utc=t_txt, label=label, notes=tuple(head + notes),
                       bbox=(lats[0], lons[0], lats[-1], lons[-1]))


def _open_meteo_get_json(url: str, timeout_s: float, opener: Optional[Callable]) -> Any:
    op = opener or urllib.request.urlopen
    try:
        with op(url, timeout=timeout_s) as r:
            payload = json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        why = ""
        try:                                    # 400 時 Open-Meteo 會在 body 說哪個參數不對（例如模式不支援某高度）
            body = json.loads(e.read().decode("utf-8"))
            if isinstance(body, dict) and body.get("reason"):
                why = f"（{body.get('reason')}）"
        except Exception:
            why = ""
        raise WindFetchError(f"Open-Meteo 回應 HTTP {e.code}：{e.reason}{why}") from None
    except (urllib.error.URLError, TimeoutError, OSError) as e:
        raise WindFetchError(f"連不到 Open-Meteo（{e}）：要網路連線；離線時只能顯示規劃風與飛機即時風") from None
    except ValueError as e:
        raise WindFetchError(f"Open-Meteo 回應不是 JSON：{e}") from None
    if isinstance(payload, dict) and payload.get("error"):
        raise WindFetchError(f"Open-Meteo 拒絕請求：{payload.get('reason')}")
    return payload


def fetch_open_meteo_profile(bbox: Sequence[float], *, nx: int = 6, ny: int = 6,
                             heights: Iterable[float] = PROFILE_HEIGHTS_M, model: str = PROFILE_MODEL_DEFAULT,
                             timeout_s: float = 20.0, opener: Optional[Callable] = None) -> WindProfile:
    """抓 bbox 一帶 ny×nx 格點的分層風（會送出格點經緯度給 Open-Meteo；呼叫端要讓操作員明確按下才呼叫）。"""
    hs = _check_profile_heights(heights)
    lats, lons = grid_points(bbox, nx, ny)
    url = build_open_meteo_profile_url(lats, lons, heights=hs, model=model)
    prof = parse_open_meteo_profile(_open_meteo_get_json(url, timeout_s, opener), lats, lons, heights=hs, model=model)
    return dataclasses.replace(prof, bbox=tuple(float(x) for x in bbox))


def _field_uv_at(f: WindField, lat: float, lon: float) -> Optional[tuple]:
    """水平雙線性內插 (u, v)，跟顯示引擎 wind_flow.js 的 interpInto 同一套：範圍外 ⇒ None、四角任一缺值 ⇒ None；
    單列／單行（均勻場）用 bbox 當範圍。"""
    lats, lons = f.lats, f.lons
    ni, nj = len(lats), len(lons)
    half = 0.01                                 # 引擎的 UNIFORM_HALF_DEG（WindField 單列／單行一定有 bbox，備而不用）
    if ni > 1:
        la0, la1 = lats[0], lats[-1]
    elif f.bbox is not None:
        la0, la1 = f.bbox[0], f.bbox[2]
    else:
        la0, la1 = lats[0] - half, lats[0] + half
    if nj > 1:
        lo0, lo1 = lons[0], lons[-1]
    elif f.bbox is not None:
        lo0, lo1 = f.bbox[1], f.bbox[3]
    else:
        lo0, lo1 = lons[0] - half, lons[0] + half
    if not (la0 <= lat <= la1 and lo0 <= lon <= lo1):
        return None

    def lo_index(a, x):                         # 最大的 i ∈ [0, n-2] 使 a[i] <= x（同引擎 bsearch）
        return min(max(bisect.bisect_right(a, x) - 1, 0), len(a) - 2)

    i0 = i1 = j0 = j1 = 0
    fi = fj = 0.0
    if ni > 1:
        i0 = lo_index(lats, lat)
        i1 = i0 + 1
        d = lats[i1] - lats[i0]
        fi = min(max((lat - lats[i0]) / d, 0.0), 1.0) if d > 0 else 0.0
    if nj > 1:
        j0 = lo_index(lons, lon)
        j1 = j0 + 1
        d = lons[j1] - lons[j0]
        fj = min(max((lon - lons[j0]) / d, 0.0), 1.0) if d > 0 else 0.0
    corners = [(i0, j0, (1 - fi) * (1 - fj)), (i0, j1, (1 - fi) * fj), (i1, j0, fi * (1 - fj)), (i1, j1, fi * fj)]
    su = sv = 0.0
    for i, j, w in corners:
        a, b = f.u[i][j], f.v[i][j]
        if not (math.isfinite(a) and math.isfinite(b)):
            return None
        su += a * w
        sv += b * w
    return su, sv


def profile_column(profile: WindProfile, lat: float, lon: float) -> list:
    """某一點每一層的風（水平雙線性，同顯示引擎；u／v 向量內插，方向不一時風速會比各角小）：
    [{'h_m', 'speed_mps', 'dir_from_deg'}]，高度遞增；那一層在這點沒有值（範圍外／四角缺值）就不列。可 JSON 化。"""
    if not (_finite(lat) and _finite(lon)):
        return []
    out = []
    for f in profile.layers:
        uv = _field_uv_at(f, float(lat), float(lon))
        if uv is None:
            continue
        d, s = uv_to_dir(*uv)
        if math.isfinite(s) and math.isfinite(d):
            out.append({"h_m": float(f.height_m), "speed_mps": float(s), "dir_from_deg": float(d)})
    return out


def profile_wind_at(profile: WindProfile, lat: float, lon: float, h_m: float) -> Optional[tuple]:
    """寫入規劃器用（規劃器只吃一層風）：這一點上**有值**的層裡最接近 h_m 的那一層 ⇒ (風速, 來向°, 用到的高度)。
    不內插、不外推；一樣近取較低的那層；那一點沒有任何一層有值（或輸入不是有限數）⇒ None。"""
    if not _finite(h_m):
        return None
    col = profile_column(profile, lat, lon)
    if not col:
        return None
    h = float(h_m)
    best = min(col, key=lambda c: (abs(c["h_m"] - h), c["h_m"]))
    return best["speed_mps"], best["dir_from_deg"], best["h_m"]


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
