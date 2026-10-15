"""GCP（地面控制點）與檢核點的佈設規劃 —— 純函式：不碰 GUI、不連線、不讀設定檔。

為什麼要有這支（docs/討論_2026-10-15_HARVEST航拍建圖_NotebookLM.md A2、B1；docs/評估_2026-10-14 §7、§8）
=================================================================================================
只用飛控 GNSS（公尺級）做不到車道級；沒有獨立的檢核點，就無法宣告任何精度。使用者決定（2026-10-15）
**交叉航帶與加密 GCP 兩條路線都做，再比較** ⇒ 這裡提供兩個具名方案：

* ``cross_strips``：交叉航帶＋最少 GCP —— 4 角＋中央（ODM：至少 5 個、角落加中央）＋每個 UAV 區塊 1 個。
* ``dense_gcp``：加密 GCP（不飛交叉航帶）—— 4 角＋4 邊中＋中央 = 9（ODM：大案 8–10）＋每個 UAV 區塊 1 個
  ＋每條跨機接縫 1 個。

兩個方案的檢核點規則相同（獨立於 GCP、分層均勻、每條跨機接縫一對），比較時才公平。

文獻事實（不誇大；出處見上面兩份文件）
-------------------------------------
* Gerke & Przybilla 2016（定翼、105 m、GSD 2.7 cm）：交叉塊同時換了方向**和高度**（75 對 105 m）、只蓋約 20%
  面積；**平地**、4 個 GCP、無 RTK 時 Z RMSE 16.5 → 2.4 cm；起伏大的料堆區沒有顯著差異；GCP 到 17–18 個後差異消失。
* Štroner 2021：同高度的垂直雙網格只有小幅改善；同高度正射＋斜拍消除了 RTK-only 的高程偏差（<0.03 m）；
  研究中**沒有**觀察到穹頂效應。
* ODM GCP 文件（docs.opendronemap.org/gcp，2026-10-15 核對）：至少 5 個、大案 8–10 個；角落附近加中央；
  間距大於單張足跡；每個 GCP 至少出現在 5 張影像（同頁另一處寫 3 張 —— 我們取 5）。
* ASPRS 第 2 版：≥30 個檢核點（否則附小專案聲明）；檢核點精度至少是產品的 2 倍；RMSE 要與檢核點測量誤差合成（RSS）
  —— 這部分在 survey/checkpoints.py。

每條規則都是 :class:`LayoutParams` 的參數（預設值與來源寫在欄位旁），實際用的值一律回報在 ``LayoutPlan.effective``；
做不到的規則**大聲警告並回報有效值**，不靜默放寬。

座標
----
輸入的作業區／區塊是 WGS84 (緯度, 經度) ring；幾何運算在以作業區中心為原點的橫麥卡托（pyproj，WGS84 橢球）
平面上做，輸出再換回經緯度。規劃位置只是「去哪裡放標靶」；實測值的框架與高程基準以外業（e-GNSS[2025] 橢球高）為準，
gcp_list 樣板與外業表都把框架寫明，**不靜默混用基準**（評估 §8：本場址大地起伏約 +18.5 m）。
"""
from __future__ import annotations

import csv
import io
import json
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence
from xml.sax.saxutils import escape as _xml_escape

import numpy as np
import shapely
from pyproj import CRS, Transformer
from pyproj.exceptions import CRSError
from shapely.geometry import LineString, MultiPolygon, Point, Polygon
from shapely.ops import nearest_points, polylabel, unary_union
from shapely.validation import explain_validity

FORMAT = "aeroplan-gcp-plan/1"
CSV_COLUMNS = ("id", "type", "lat", "lon", "block", "purpose", "note")
SURVEY_COLUMNS = ("id", "type", "planned_lat", "planned_lon", "lat", "lon", "h", "frame", "vertical_datum",
                  "sigma_h_m", "sigma_v_m", "antenna_height_m", "block", "purpose")
TYPES = ("GCP", "CHK")

# HQ6 相機、120 m AGL 的單張足跡（實作文件 2026-10-14：125.7 × 94.2 m；w ⟂ 航向、l ∥ 航向）
DEFAULT_FOOTPRINT_W_M = 125.7
DEFAULT_FOOTPRINT_L_M = 94.2
DEFAULT_SURVEY_FRAME = "e-GNSS[2025]"
VERTICALS = ("ellipsoidal", "orthometric")

PURPOSE_TEXT = {
    "corner": "角落（ODM：角落附近＋中央）",
    "centre": "中央",
    "edge_mid": "邊中點（加密方案，ODM 大案 8–10）",
    "block": "UAV 區塊控制（三台相機各自率定 ⇒ 每個區塊至少 1 個）",
    "seam": "跨機接縫（兩區塊的影像都看得到，綁住兩塊）",
    "split": "ODM split 子模型（每個子模型 ≥3 個）",
    "min_total": "補足 GCP 總數",
    "seam_pair": "接縫檢核點對（量跨機高程階差）",
    "stratified": "分層均勻檢核點（等面積 Voronoi 分層）",
}


class GcpPlanError(ValueError):
    """輸入不合理（訊息是給操作員看的繁中說明）。"""


#: 交叉航帶方案的 GCP 超過這個數就大聲警告（它的意義是「少 GCP」；Gerke 2016 在 17–18 個時差異消失）
CROSS_PRESET_GCP_WARN = 10


@dataclass(frozen=True)
class Preset:
    name: str
    label: str
    base: tuple                  # 每塊作業區的基本佈設：'corner' / 'edge_mid' / 'centre'
    per_block: bool              # 每個 UAV 區塊加 1 個
    seam_gcp: bool               # 每條跨機接縫加 1 個
    min_total: int               # GCP 總數下限
    description: str


PRESETS = {
    "cross_strips": Preset(
        "cross_strips", "交叉航帶＋最少 GCP", ("corner", "centre"), True, False, 5,
        "4 角＋中央（ODM 至少 5 個）＋每個 UAV 區塊 1 個；精度靠交叉航帶（Gerke 2016：平地、少 GCP 時有效）"),
    "dense_gcp": Preset(
        "dense_gcp", "加密 GCP（不飛交叉航帶）", ("corner", "edge_mid", "centre"), True, True, 8,
        "4 角＋4 邊中＋中央 = 9（ODM 大案 8–10）＋每個 UAV 區塊 1 個＋每條跨機接縫 1 個"),
}


@dataclass(frozen=True)
class LayoutParams:
    """佈設參數。``None`` = 用右邊註明的預設（由足跡推得），實際值寫在 ``LayoutPlan.effective``。"""
    preset: str = "cross_strips"
    footprint_w_m: float = DEFAULT_FOOTPRINT_W_M      # 單張足跡寬（⟂ 航向）
    footprint_l_m: float = DEFAULT_FOOTPRINT_L_M      # 單張足跡長（∥ 航向）
    edge_band_m: Optional[float] = None               # None → w/2（驗證器：外緣 13–18 m 只有 3–4 視）
    min_inset_m: float = 30.0                         # 距作業區邊界至少（ODM：避開邊緣；我們 SITL：≥20–30 m）
    gcp_min_spacing_m: Optional[float] = None         # None → max(w, l)（ODM：間距大於單張足跡）
    chk_gcp_min_m: Optional[float] = None             # None → l（檢核點與 GCP 至少相隔一個足跡長）
    chk_inset_m: Optional[float] = None               # None → 與 GCP 相同的內縮
    n_checkpoints: int = 20                           # 20；ASPRS 第 2 版要 30（否則附小專案聲明）
    seam_pair_offset_m: Optional[float] = None        # None → w/2：接縫檢核點離「對方區塊」至少這麼遠
    min_seam_length_m: Optional[float] = None         # None → l：比這短的接觸不算接縫
    per_block: Optional[bool] = None                  # None → 方案預設
    seam_gcp: Optional[bool] = None                   # None → 方案預設
    min_total: Optional[int] = None                   # None → 方案預設（5／8）
    split_submodels: int = 0                          # ODM split 子模型數（0 = 不分割）
    min_gcp_per_split: int = 3                        # ODM：每個子模型 ≥3 個 GCP
    large_area_km2: float = 1.0                       # 「大案」門檻 —— 我們的假設（ODM 沒給面積數字）
    large_min_gcp: int = 8                            # 大案時 ODM 建議 8–10
    seed: int = 0                                     # 候選點抖動與分層初始化的種子（同輸入同種子 ⇒ 同結果）
    max_candidates: int = 20000                       # 候選點上限（決定放點解析度）

    def check(self) -> None:
        if self.preset not in PRESETS:
            raise GcpPlanError(f"方案只能是 {sorted(PRESETS)} 之一，收到 {self.preset!r}")
        for name in ("footprint_w_m", "footprint_l_m", "min_inset_m", "large_area_km2"):
            _positive(getattr(self, name), name, allow_zero=(name == "min_inset_m"))
        for name in ("edge_band_m", "gcp_min_spacing_m", "chk_gcp_min_m", "chk_inset_m", "seam_pair_offset_m",
                     "min_seam_length_m"):
            v = getattr(self, name)
            if v is not None:
                _positive(v, name, allow_zero=True)
        for name, lo in (("n_checkpoints", 0), ("split_submodels", 0), ("min_gcp_per_split", 1), ("seed", 0),
                         ("large_min_gcp", 1), ("max_candidates", 500)):
            v = getattr(self, name)
            if isinstance(v, bool) or not isinstance(v, (int, np.integer)) or v < lo:
                raise GcpPlanError(f"{name} 要是 ≥{lo} 的整數，收到 {v!r}")
        if self.split_submodels == 1:
            raise GcpPlanError("split_submodels = 1 沒有意義（0 = 不分割，≥2 = 子模型數）")
        if self.min_total is not None and (isinstance(self.min_total, bool) or not isinstance(self.min_total, int)
                                           or self.min_total < 1):
            raise GcpPlanError(f"min_total 要是 ≥1 的整數，收到 {self.min_total!r}")


def _positive(v, name, *, allow_zero=False):
    if isinstance(v, bool) or not isinstance(v, (int, float, np.integer, np.floating)) or not math.isfinite(v):
        raise GcpPlanError(f"{name} 要是有限實數，收到 {v!r}")
    if v < 0 or (v == 0 and not allow_zero):
        raise GcpPlanError(f"{name} 要 {'≥' if allow_zero else '>'} 0，收到 {v!r}")


# ─────────────────────────────────────────────────────────────────────────────
# 結果
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class PlannedPoint:
    id: str
    type: str                  # 'GCP' / 'CHK'
    lat: float
    lon: float
    block: str                 # 所在 UAV 區塊（重疊處 'A1|B2'；沒有區塊資訊為 ''）
    purpose: str               # PURPOSE_TEXT 的鍵
    note: str
    e_m: float                 # 局部平面座標（測試與距離計算用）
    n_m: float
    seam: str = ""             # 接縫檢核點對：'A1|B2'
    pair: int = 0              # 接縫檢核點對的序號（1 起算）

    def row(self) -> dict:
        return {"id": self.id, "type": self.type, "lat": f"{self.lat:.8f}", "lon": f"{self.lon:.8f}",
                "block": self.block, "purpose": self.purpose, "note": self.note}


@dataclass
class LayoutPlan:
    points: list
    effective: dict
    warnings: list
    notes: list
    counts: dict
    seams: list
    blocks: dict
    checks: list               # [(規則, 通過?, 說明)]
    areas: list                # 作業區 ring（緯度, 經度）
    block_rings: dict          # 區塊 → ring 的串列（裁到作業區內）

    def gcps(self) -> list:
        return [p for p in self.points if p.type == "GCP"]

    def checkpoints(self) -> list:
        return [p for p in self.points if p.type == "CHK"]

    def by_id(self, pid: str) -> PlannedPoint:
        for p in self.points:
            if p.id == pid:
                return p
        raise KeyError(pid)


# ─────────────────────────────────────────────────────────────────────────────
# 輸入正規化與局部座標
# ─────────────────────────────────────────────────────────────────────────────

def _is_ring_list(obj) -> bool:
    try:
        inner = obj[0][0]
    except (TypeError, IndexError, KeyError):
        return False
    if isinstance(inner, (str, bytes)):
        return False
    try:
        return len(inner) >= 2
    except TypeError:
        return False


def _ring(ring, where: str) -> list:
    pts = []
    try:
        for p in ring:
            lat, lon = p[0], p[1]
            if isinstance(lat, bool) or isinstance(lon, bool):
                raise TypeError
            pts.append((float(lat), float(lon)))
    except (TypeError, ValueError, IndexError):
        raise GcpPlanError(f"{where}的角點要是（緯度, 經度）數字，收到 {str(ring)[:200]}") from None
    if len(pts) >= 2 and pts[0] == pts[-1]:
        pts = pts[:-1]
    if len(pts) < 3:
        raise GcpPlanError(f"{where}至少要 3 個角點，收到 {len(pts)}")
    for lat, lon in pts:
        if not (math.isfinite(lat) and math.isfinite(lon)) or not (-90 <= lat <= 90 and -180 <= lon <= 180):
            raise GcpPlanError(f"{where}的角點不是合理的經緯度：{(lat, lon)}")
    return pts


def normalize_areas(polygons) -> list:
    """作業區：ring 的串列 [[(lat, lon), …], …]（MainWindow.survey_area_polygons() 的形狀）；單一 ring 也收。"""
    if polygons is None:
        raise GcpPlanError("沒有作業區：請先在地圖上圈好並完成作業區")
    raw = list(polygons) if _is_ring_list(polygons) else [polygons]
    if not raw:
        raise GcpPlanError("沒有作業區：請先在地圖上圈好並完成作業區")
    return [_ring(r, f"第 {k} 塊作業區") for k, r in enumerate(raw, start=1)]


def normalize_blocks(blocks) -> dict:
    """UAV 區塊：{標籤: ring 或 ring 的串列}。None / 空 = 沒有區塊資訊。"""
    if not blocks:
        return {}
    if not isinstance(blocks, Mapping):
        raise GcpPlanError(f"區塊要是 {{標籤: ring 的串列}}，收到 {type(blocks).__name__}")
    out = {}
    for label in sorted(blocks, key=str):
        val = blocks[label]
        name = str(label).strip()
        if not name or "|" in name or "," in name:
            raise GcpPlanError(f"區塊標籤不能空白、也不能含 '|' 或 ','：{label!r}")
        rings = list(val) if _is_ring_list(val) else [val]
        out[name] = [_ring(r, f"區塊 {name} 第 {k} 塊") for k, r in enumerate(rings, start=1)]
    return out


class LocalFrame:
    """以 (lat0, lon0) 為中心的橫麥卡托（WGS84 橢球，k=1）。2 km 範圍內距離誤差 < 0.1 mm。"""

    def __init__(self, lat0: float, lon0: float):
        self.lat0, self.lon0 = float(lat0), float(lon0)
        proj = (f"+proj=tmerc +lat_0={self.lat0:.10f} +lon_0={self.lon0:.10f} +k=1 +x_0=0 +y_0=0 "
                "+ellps=WGS84 +units=m +no_defs")
        self.proj = proj
        self._fwd = Transformer.from_crs("EPSG:4326", proj, always_xy=True)
        self._inv = Transformer.from_crs(proj, "EPSG:4326", always_xy=True)

    def en(self, lat, lon):
        e, n = self._fwd.transform(np.asarray(lon, float), np.asarray(lat, float))
        return e, n

    def ll(self, e, n):
        lon, lat = self._inv.transform(np.asarray(e, float), np.asarray(n, float))
        return lat, lon

    def polygon(self, ring) -> Polygon:
        lat = np.asarray([p[0] for p in ring])
        lon = np.asarray([p[1] for p in ring])
        e, n = self.en(lat, lon)
        return Polygon(np.column_stack([e, n]))

    def rings_of(self, geom) -> list:
        out = []
        for poly in _parts(geom):
            xy = np.asarray(poly.exterior.coords)[:-1]
            lat, lon = self.ll(xy[:, 0], xy[:, 1])
            out.append([(float(a), float(b)) for a, b in zip(lat, lon)])
        return out


def _parts(geom) -> list:
    if geom is None or geom.is_empty:
        return []
    if isinstance(geom, Polygon):
        return [geom]
    if isinstance(geom, MultiPolygon):
        return list(geom.geoms)
    return [g for g in getattr(geom, "geoms", []) if isinstance(g, Polygon)]


def _largest(geom) -> Optional[Polygon]:
    parts = _parts(geom)
    return max(parts, key=lambda p: p.area) if parts else None


def _close(geom, d: float = 0.05):
    """形態學閉運算（先外擴再內縮 d m，直角保持）：合併只差浮點誤差的相接多邊形。"""
    return geom.buffer(d, join_style="mitre", mitre_limit=10.0).buffer(-d, join_style="mitre", mitre_limit=10.0)


def _inset(geom, d: float):
    """往內縮 d m（尖角保持直角：mitre）。"""
    if d <= 0:
        return geom
    return geom.buffer(-d, join_style="mitre", mitre_limit=10.0)


# ─────────────────────────────────────────────────────────────────────────────
# 由 HARVEST 規劃取每架的區塊（唯讀）
# ─────────────────────────────────────────────────────────────────────────────

def blocks_from_hv(hv) -> tuple:
    """``HvDccppCoordinator`` 最近一次的規劃 → ({UAV 標籤: ring 的串列}, {標籤: 說明})。**唯讀**，不改任何狀態。

    每架的區塊 = 它飛的每條**主**條帶「中心線 ± 條帶間距/2」的聯集（條帶含引入段外伸，之後會被裁到作業區內）。
    建圖模式的接縫外擴讓相鄰兩架的區塊重疊，重疊處就是接縫。區塊的內洞不保留（只取外框）。

    交叉航帶不算：它的兩端（e_ext＋引入段）會伸進鄰機區塊，算進去會把區塊撐大 10–50%、接縫寬度變成
    2–13 倍（審查探針 2026-10-15）。排除的條數記在 info[標籤]['n_cross_excluded']。
    """
    from hv_dccpp.geometry import is_cross_strip
    plan = getattr(hv, "last_plan", None)
    ctx = getattr(hv, "last_ctx", None)
    if plan is None or ctx is None:
        raise GcpPlanError("沒有 HARVEST 規劃結果（請先用 HARVEST 規劃一次）")
    id_map = dict(getattr(hv, "id_map", None) or {})
    rev = {u: d for d, u in id_map.items()}
    out, info = {}, {}
    for uid in sorted(plan.plans):
        up = plan.plans[uid]
        bands = []
        n_cross = 0
        for sid, _dirn in up.strips:
            if is_cross_strip(sid):
                n_cross += 1
                continue
            st = plan.strips.get(sid)
            if st is None:
                continue
            p0 = np.asarray(st.pL, float).reshape(-1)[:2]
            p1 = np.asarray(st.pR, float).reshape(-1)[:2]
            half = float(st.spacing_m) / 2.0
            if not (half > 0 and np.all(np.isfinite(p0)) and np.all(np.isfinite(p1))):
                continue
            bands.append(LineString([tuple(p0), tuple(p1)]).buffer(half, cap_style="flat"))
        if not bands:
            continue
        # 相鄰條帶的帶剛好相接，浮點誤差會留下零寬的縫 ⇒ 先閉運算（±5 cm）合成一塊
        u = _close(unary_union(bands))
        rings = []
        for poly in _parts(u):
            xy = np.asarray(poly.exterior.coords)[:-1]
            w = np.asarray(ctx.frame.to_wgs84(np.column_stack([xy, np.zeros(len(xy))])), float)
            rings.append([(float(a), float(b)) for a, b in w[:, :2]])
        label = str(uid)
        out[label] = rings
        info[label] = {"uav_id": str(uid), "drone_id": rev.get(uid), "cls": str(getattr(up, "cls", "")),
                       "n_strips": len(up.strips) - n_cross, "n_cross_excluded": n_cross}
    if not out:
        raise GcpPlanError("HARVEST 規劃裡沒有任何一架有條帶")
    return out, info


# ─────────────────────────────────────────────────────────────────────────────
# 候選點與放點
# ─────────────────────────────────────────────────────────────────────────────

def _pool(region, max_n: int, rng) -> tuple:
    """區域內的抖動格點（種子決定）。回傳 (N×2 陣列, 格距)。"""
    area = region.area
    step = max(1.0, math.sqrt(area / float(max_n)))
    minx, miny, maxx, maxy = region.bounds
    xs = np.arange(minx + step / 2.0, maxx, step)
    ys = np.arange(miny + step / 2.0, maxy, step)
    gx, gy = np.meshgrid(xs, ys)
    jx = rng.uniform(-0.3, 0.3, gx.shape) * step
    jy = rng.uniform(-0.3, 0.3, gy.shape) * step
    gx, gy = gx + jx, gy + jy
    inside = shapely.contains_xy(region, gx, gy)
    return np.column_stack([gx[inside], gy[inside]]), step


def _in(region, pts: np.ndarray) -> np.ndarray:
    if region is None or region.is_empty or len(pts) == 0:
        return np.zeros(len(pts), bool)
    return shapely.contains_xy(region, pts[:, 0], pts[:, 1])


def _min_dist(pts: np.ndarray, others: Sequence) -> np.ndarray:
    if not len(others):
        return np.full(len(pts), np.inf)
    o = np.asarray(others, float).reshape(-1, 2)
    d = np.hypot(pts[:, None, 0] - o[None, :, 0], pts[:, None, 1] - o[None, :, 1])
    return d.min(axis=1)


def _centre_of(geom) -> tuple:
    """幾何的「中央」：質心在裡面就用質心，否則用最大一塊的不可達極點（polylabel）。"""
    c = geom.centroid
    if geom.contains(c):
        return float(c.x), float(c.y)
    big = _largest(geom)
    if big is None:
        p = geom.representative_point()
        return float(p.x), float(p.y)
    p = polylabel(big, tolerance=0.5)
    return float(p.x), float(p.y)


def _farthest(pool: np.ndarray, region, existing: Sequence) -> Optional[tuple]:
    """區域內離既有點最遠的候選點；沒有既有點時取區域中央。回傳 ((e, n), 最近距離) 或 None。"""
    if region is None or region.is_empty:
        return None
    cand = pool[_in(region, pool)]
    if not len(cand):
        p = region.representative_point()
        cand = np.asarray([[p.x, p.y]])
    if not len(existing):
        ce, cn = _centre_of(region)
        k = int(np.argmin(np.hypot(cand[:, 0] - ce, cand[:, 1] - cn)))
        return (float(cand[k, 0]), float(cand[k, 1])), math.inf
    d = _min_dist(cand, existing)
    k = int(np.argmax(d))
    return (float(cand[k, 0]), float(cand[k, 1])), float(d[k])


def _nearest_in(pool: np.ndarray, region, target: tuple) -> Optional[tuple]:
    if region is None or region.is_empty:
        return None
    cand = pool[_in(region, pool)]
    if not len(cand):
        return None
    k = int(np.argmin(np.hypot(cand[:, 0] - target[0], cand[:, 1] - target[1])))
    return float(cand[k, 0]), float(cand[k, 1])


def _clockwise_from_north(pts, centre) -> list:
    return sorted(pts, key=lambda p: (math.degrees(math.atan2(p[0] - centre[0], p[1] - centre[1])) % 360.0))


def _layout_points(pin, kinds) -> list:
    """一塊（已內縮的）作業區的基本佈設：[(purpose, (e, n), 說明)]。"""
    out = []
    rect = shapely.minimum_rotated_rectangle(pin)
    if not isinstance(rect, Polygon):          # 退化（線或點）
        ce, cn = _centre_of(pin)
        return [("centre", (ce, cn), "內縮後退化成線，只放中央")] if "centre" in kinds else []
    corners = [tuple(c) for c in np.asarray(rect.exterior.coords)[:4]]
    centre = (rect.centroid.x, rect.centroid.y)
    corners = _clockwise_from_north(corners, centre)

    def snap(p):
        q = nearest_points(pin, Point(p))[0]
        return float(q.x), float(q.y)

    if "corner" in kinds:
        for c in corners:
            out.append(("corner", snap(c), ""))
    if "edge_mid" in kinds:
        mids = [((corners[i][0] + corners[(i + 1) % 4][0]) / 2.0, (corners[i][1] + corners[(i + 1) % 4][1]) / 2.0)
                for i in range(4)]
        for m in _clockwise_from_north(mids, centre):
            out.append(("edge_mid", snap(m), ""))
    if "centre" in kinds:
        out.append(("centre", _centre_of(pin), ""))
    return out


# ─────────────────────────────────────────────────────────────────────────────
# 接縫
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class _Seam:
    a: str
    b: str
    region: Any                # 兩區塊的重疊（或相接處的細帶）
    kind: str                  # 'overlap' / 'abut'
    length_m: float
    width_m: float
    mid: tuple


def _find_seams(blocks: dict, *, tol: float, min_len: float, notes: list) -> list:
    labels = sorted(blocks)
    out = []
    for i, a in enumerate(labels):
        for b in labels[i + 1:]:
            ga, gb = blocks[a], blocks[b]
            reg, kind = ga.intersection(gb), "overlap"
            if reg.is_empty or reg.area < tol * tol:
                reg, kind = ga.buffer(tol).intersection(gb.buffer(tol)), "abut"
                if reg.is_empty or reg.area <= 0:
                    continue
            rect = shapely.minimum_rotated_rectangle(reg)
            if isinstance(rect, Polygon):
                c = np.asarray(rect.exterior.coords)[:4]
                sides = [float(np.hypot(*(c[(k + 1) % 4] - c[k]))) for k in range(4)]
                length = max(sides)
            else:
                length = float(rect.length)
            if length < min_len:
                notes.append(f"區塊 {a}／{b} 只接觸 {length:.0f} m（< {min_len:.1f} m），不算接縫")
                continue
            c = reg.centroid
            mid = nearest_points(reg, c)[0] if not reg.contains(c) else c
            out.append(_Seam(a, b, reg, kind, length, float(reg.area / length) if length > 0 else 0.0,
                             (float(mid.x), float(mid.y))))
    return out


# ─────────────────────────────────────────────────────────────────────────────
# 分層檢核點（等面積 Voronoi：Lloyd）
# ─────────────────────────────────────────────────────────────────────────────

def _cvt(cand: np.ndarray, fixed: Sequence, n_free: int, rng, iters: int = 100) -> np.ndarray:
    """候選點上的 k-means（固定點不動）→ n_free 個分層中心（貼回最近的候選點，保證落在允許區內）。"""
    if n_free <= 0 or not len(cand):
        return np.zeros((0, 2))
    fixed = np.asarray(fixed, float).reshape(-1, 2)
    gens = []
    # k-means++ 初始化（考慮固定點）
    d2 = (_min_dist(cand, fixed) ** 2) if len(fixed) else None
    for _ in range(n_free):
        if d2 is None or not np.isfinite(d2).any() or d2.sum() <= 0:
            k = int(rng.integers(len(cand)))
        else:
            p = d2 / d2.sum()
            k = int(rng.choice(len(cand), p=p))
        gens.append(cand[k].copy())
        dk = np.hypot(cand[:, 0] - cand[k, 0], cand[:, 1] - cand[k, 1]) ** 2
        d2 = dk if d2 is None else np.minimum(d2, dk)
    gens = np.asarray(gens, float)
    nf = len(fixed)
    for _ in range(iters):
        allg = np.vstack([fixed, gens]) if nf else gens
        d = (cand[:, None, 0] - allg[None, :, 0]) ** 2 + (cand[:, None, 1] - allg[None, :, 1]) ** 2
        lab = np.argmin(d, axis=1)
        new = gens.copy()
        for j in range(n_free):
            m = lab == nf + j
            if m.any():
                new[j] = cand[m].mean(axis=0)
        moved = float(np.max(np.hypot(*(new - gens).T))) if n_free else 0.0
        gens = new
        if moved < 0.01:
            break
    # 貼回候選點（非凸區域的質心可能落在外面）；不重複
    allg = np.vstack([fixed, gens]) if nf else gens
    d = (cand[:, None, 0] - allg[None, :, 0]) ** 2 + (cand[:, None, 1] - allg[None, :, 1]) ** 2
    lab = np.argmin(d, axis=1)
    used, out = set(), []
    for j in range(n_free):
        idx = np.nonzero(lab == nf + j)[0]
        if not len(idx):
            idx = np.arange(len(cand))
        order = idx[np.argsort(np.hypot(cand[idx, 0] - gens[j, 0], cand[idx, 1] - gens[j, 1]), kind="stable")]
        k = next((int(i) for i in order if int(i) not in used), None)
        if k is None:
            continue
        used.add(k)
        out.append(cand[k])
    return np.asarray(out, float).reshape(-1, 2)


def _kmeans_labels(cand: np.ndarray, n: int, rng, iters: int = 100) -> tuple:
    cent = _cvt(cand, [], n, rng, iters)
    d = (cand[:, None, 0] - cent[None, :, 0]) ** 2 + (cand[:, None, 1] - cent[None, :, 1]) ** 2
    return np.argmin(d, axis=1), cent


# ─────────────────────────────────────────────────────────────────────────────
# 主函式
# ─────────────────────────────────────────────────────────────────────────────

def plan_layout(areas, blocks=None, params: Optional[LayoutParams] = None, *, blocks_source: str = "") -> LayoutPlan:
    """作業區（＋每架的 UAV 區塊）→ GCP 與檢核點的建議位置。

    ``areas``：ring 的串列 [[(lat, lon), …], …]；``blocks``：{UAV 標籤: ring 或 ring 的串列}（可省略）。
    """
    params = params or LayoutParams()
    params.check()
    preset = PRESETS[params.preset]
    rings = normalize_areas(areas)
    braw = normalize_blocks(blocks)
    warnings, notes = [], []
    w, l = float(params.footprint_w_m), float(params.footprint_l_m)

    allpts = np.asarray([p for r in rings for p in r], float)
    frame = LocalFrame(float(allpts[:, 0].mean()), float(allpts[:, 1].mean()))
    polys = []
    for k, r in enumerate(rings, start=1):
        p = frame.polygon(r)
        if not p.is_valid:
            raise GcpPlanError(f"第 {k} 塊作業區自相交或不是簡單多邊形（{explain_validity(p)}）")
        polys.append(p)
    union = unary_union(polys)
    area_m2 = float(union.area)

    eff = {"format": FORMAT, "preset": preset.name, "preset_label": preset.label,
           "preset_description": preset.description,
           "footprint_w_m": w, "footprint_l_m": l, "seed": int(params.seed),
           "local_frame": frame.proj, "area_m2": area_m2, "n_areas": len(rings)}
    src = {}

    def pick(name, value, default, why):
        if value is None:
            src[name] = f"預設：{why}"
            return float(default)
        src[name] = "使用者"
        return float(value)

    edge_band = pick("edge_band_m", params.edge_band_m, w / 2.0, "w/2（外緣低視數帶）")
    inset = max(float(params.min_inset_m), edge_band)
    src["inset_m"] = f"max(最小內縮 {params.min_inset_m:g} m, 邊緣帶 {edge_band:g} m)"
    spacing = pick("gcp_min_spacing_m", params.gcp_min_spacing_m, max(w, l), "max(w, l)（ODM：大於單張足跡）")
    chk_gcp = pick("chk_gcp_min_m", params.chk_gcp_min_m, l, "l（一個足跡長）")
    chk_inset = pick("chk_inset_m", params.chk_inset_m, inset, "與 GCP 相同的內縮")
    pair_off = pick("seam_pair_offset_m", params.seam_pair_offset_m, w / 2.0, "w/2")
    min_seam = pick("min_seam_length_m", params.min_seam_length_m, l, "l")
    per_block = preset.per_block if params.per_block is None else bool(params.per_block)
    seam_gcp = preset.seam_gcp if params.seam_gcp is None else bool(params.seam_gcp)
    min_total = preset.min_total if params.min_total is None else int(params.min_total)
    src["per_block"] = "方案預設" if params.per_block is None else "使用者"
    src["seam_gcp"] = "方案預設" if params.seam_gcp is None else "使用者"
    src["min_total"] = "方案預設" if params.min_total is None else "使用者"
    large = area_m2 / 1e6 >= float(params.large_area_km2)
    eff.update(edge_band_m=edge_band, inset_m=inset, gcp_min_spacing_m=spacing, chk_gcp_min_m=chk_gcp,
               chk_inset_m=chk_inset, seam_pair_offset_m=pair_off, min_seam_length_m=min_seam,
               per_block=per_block, seam_gcp=seam_gcp, min_total=min_total,
               n_checkpoints_requested=int(params.n_checkpoints), split_submodels=int(params.split_submodels),
               min_gcp_per_split=int(params.min_gcp_per_split), large_area_km2=float(params.large_area_km2),
               large_min_gcp=int(params.large_min_gcp), is_large_area=bool(large), base_layout=list(preset.base),
               sources=src)

    inner = _inset(union, inset)
    if inner.is_empty:
        raise GcpPlanError(f"作業區太小：內縮 {inset:.1f} m（邊緣低視數帶）之後沒有可以放點的範圍")
    rng = np.random.default_rng(int(params.seed))
    pool, step = _pool(union, int(params.max_candidates), rng)
    eff["candidate_step_m"] = step

    # ── 區塊 ──
    bgeo, block_rings = {}, {}
    for label, brs in braw.items():
        g = unary_union([frame.polygon(r).buffer(0) for r in brs]).intersection(union)
        if g.is_empty or g.area < 1.0:
            warnings.append(f"區塊 {label} 與作業區沒有交集（HARVEST 規劃與目前的作業區不一致？），不使用")
            continue
        bgeo[label] = g
        block_rings[label] = frame.rings_of(g)
    if bgeo:
        bands = unary_union(list(bgeo.values()))
        eff["blocks_band_coverage"] = float(bands.area / area_m2)
        # 一致性：條帶帶寬只到「最外條帶 ± s/2」，但影像蓋到 ± w/2 ⇒ 以「帶 ＋ w/2」量作業區有多少沒有任何一架拍到
        imaged = bands.buffer(w / 2.0).intersection(union)
        cov = float(imaged.area / area_m2)
        eff["blocks_coverage"] = cov
        if cov < 0.995:
            warnings.append(f"HARVEST 區塊（含半個足跡 w/2 的影像範圍）只蓋到作業區的 {cov:.1%}"
                            "（規劃與目前的作業區不一致？請重新規劃再產生）")
        # 邊緣補齊：作業區裡沒落在任何條帶帶內的部分（通常是外緣 < w/2 的那一圈）依「最近的區塊」分給各區塊
        # （候選格點上的最近鄰，格距解析度；相鄰格子稍微放大以免留縫，分界處會有一格寬的重疊）
        gaps = union.difference(bands)
        filled = float(gaps.area) if not gaps.is_empty else 0.0
        gx = pool[_in(gaps, pool)] if filled >= 1.0 else np.zeros((0, 2))
        if len(gx):
            labels = sorted(bgeo)
            pts = shapely.points(gx)
            near = np.argmin(np.vstack([shapely.distance(bgeo[lb], pts) for lb in labels]), axis=0)
            half = step                                       # 格子邊長 2·step ≥ 抖動後的最大格距 1.6·step
            for j, lb in enumerate(labels):
                sel = gx[near == j]
                if len(sel):
                    cells = shapely.box(sel[:, 0] - half, sel[:, 1] - half, sel[:, 0] + half, sel[:, 1] + half)
                    bgeo[lb] = bgeo[lb].union(unary_union(cells).intersection(gaps))
        eff["blocks_gap_filled_m2"] = filled
        if filled:
            notes.append(f"條帶帶寬（± s/2）沒蓋到的作業區 {filled:.0f} m²（多半是外緣一圈）依最近的區塊分給各區塊")
        bgeo = {lb: _close(g).intersection(union) for lb, g in bgeo.items()}
        block_rings = {lb: frame.rings_of(g) for lb, g in bgeo.items()}
    else:
        notes.append("沒有 UAV 區塊資訊 ⇒ 不加「每區塊 GCP」、不找跨機接縫（沒有接縫檢核點對）")
    eff["blocks"] = sorted(bgeo)
    eff["blocks_source"] = blocks_source or ("呼叫端提供" if bgeo else "無")
    excl = {}
    for label, g in bgeo.items():
        others = [bgeo[o] for o in bgeo if o != label]
        e_ = g.difference(unary_union(others)) if others else g
        excl[label] = e_ if not e_.is_empty and e_.area > 1.0 else g
    seams = _find_seams(bgeo, tol=max(1.0, step), min_len=min_seam, notes=notes) if len(bgeo) >= 2 else []

    # ── GCP ──
    gcps = []          # dict(e, n, purpose, note)

    def gxy():
        return [(g["e"], g["n"]) for g in gcps]

    def add(purpose, xy, note):
        gcps.append({"e": float(xy[0]), "n": float(xy[1]), "purpose": purpose, "note": note})

    # 基本佈設（角落／邊中／中央）以「相接作業區的連通塊」為單位：相鄰作業區的共用邊不是測區的角，逐塊套用會在
    # 內部放一堆「角點」（3 塊相鄰作業區 ⇒ 最少 GCP 方案膨脹到 18 個，落進 Gerke 說差異消失的 17–18 個區間）
    comps = _parts(_close(union))
    base_units = polys if len(comps) >= len(polys) else comps
    if base_units is comps:
        notes.append(f"{len(polys)} 塊作業區相接成 {len(comps)} 塊連通區：角落／中央 GCP 以連通區為單位放"
                     "（相鄰作業區的共用邊不是測區的角）")
    for k, poly in enumerate(base_units, start=1):
        pin = _inset(poly, inset).intersection(inner)
        tag = (f"第 {k} 塊作業區" if base_units is polys else f"第 {k} 塊連通作業區") if len(base_units) > 1 else "作業區"
        if pin.is_empty:
            warnings.append(f"{tag}內縮 {inset:.1f} m 後是空的（太窄）：這塊不放角落／中央 GCP")
            continue
        skipped = []
        for purpose, xy, why in _layout_points(pin, preset.base):
            d = float(_min_dist(np.asarray([xy]), gxy())[0])
            if d < spacing:
                skipped.append((purpose, d))
                continue
            add(purpose, xy, f"{tag}{PURPOSE_TEXT[purpose].split('（')[0]}" + (f"；{why}" if why else ""))
        if skipped:
            what = "、".join(f"{PURPOSE_TEXT[p].split('（')[0]}（距既有 GCP {d:.0f} m）" for p, d in skipped)
            warnings.append(f"{tag}：{len(skipped)} 個基本佈設點與既有 GCP 距離 < 間距 {spacing:.1f} m，未放：{what}")

    if per_block:
        for label in sorted(bgeo):
            region = excl[label].intersection(inner)
            if region.is_empty:
                region = bgeo[label].intersection(inner)
            got = _farthest(pool, region, gxy())
            if got is None:
                warnings.append(f"區塊 {label} 內縮 {inset:.1f} m 後沒有空間：這個區塊沒有自己的 GCP")
                continue
            xy, d = got
            add("block", xy, f"UAV 區塊 {label} 的控制點")
            if d < spacing:
                warnings.append(f"區塊 {label} 的 GCP 離最近的 GCP 只有 {d:.1f} m（< 間距 {spacing:.1f} m）：區塊太小")

    if seam_gcp:
        for s in seams:
            region = s.region.intersection(inner)
            got = _farthest(pool, region, gxy()) if not region.is_empty else None
            if got is None:
                warnings.append(f"接縫 {s.a}｜{s.b} 在內縮範圍內沒有空間：沒有放接縫 GCP")
                continue
            xy, d = got
            add("seam", xy, f"跨機接縫 {s.a}｜{s.b}")
            if d < spacing:
                warnings.append(f"接縫 {s.a}｜{s.b} 的 GCP 離最近的 GCP 只有 {d:.1f} m（< 間距 {spacing:.1f} m）")

    split_info = []
    if params.split_submodels >= 2:
        n_split = int(params.split_submodels)
        if len(pool) < n_split:
            raise GcpPlanError(f"作業區太小，分不成 {n_split} 個子模型")
        lab, cent = _kmeans_labels(pool, n_split, np.random.default_rng(int(params.seed) + 7))
        cand_inner = _in(inner, pool)
        for j in range(n_split):
            mine = (lab == j)

            def count():
                if not gcps:
                    return 0
                g = np.asarray(gxy())
                d = (g[:, None, 0] - cent[None, :, 0]) ** 2 + (g[:, None, 1] - cent[None, :, 1]) ** 2
                return int((np.argmin(d, axis=1) == j).sum())

            before = count()
            while count() < params.min_gcp_per_split:
                sub = pool[mine & cand_inner]
                if not len(sub):
                    warnings.append(f"ODM 子模型 {j + 1} 在內縮範圍內沒有空間，GCP 只有 {count()} 個")
                    break
                d = _min_dist(sub, gxy())
                kk = int(np.argmax(d))
                add("split", sub[kk], f"ODM split 子模型 {j + 1}（每子模型 ≥{params.min_gcp_per_split}）")
                if d[kk] < spacing:
                    warnings.append(f"ODM 子模型 {j + 1} 補的 GCP 離最近的 GCP 只有 {d[kk]:.1f} m（< 間距 {spacing:.1f} m）")
            clat, clon = frame.ll(cent[j, 0], cent[j, 1])
            split_info.append({"submodel": j + 1, "centre_lat": float(clat), "centre_lon": float(clon),
                               "n_gcp_before": before, "n_gcp": count()})
        notes.append("ODM 的 split 是依影像位置分群；這裡以作業區面積的 k-means 近似，實際子模型以 ODM 為準")
    eff["split"] = split_info

    need = min_total
    while len(gcps) < need:
        got = _farthest(pool, inner, gxy())
        if got is None:
            warnings.append(f"GCP 只放得下 {len(gcps)} 個（< 下限 {need}）")
            break
        xy, d = got
        add("min_total", xy, f"補足總數（下限 {need}）")
        if d < spacing:
            warnings.append(f"補足總數的 GCP 離最近的 GCP 只有 {d:.1f} m（< 間距 {spacing:.1f} m）：作業區太小")
    if preset.name == "cross_strips" and len(gcps) > CROSS_PRESET_GCP_WARN:
        warnings.append(f"⚠ 交叉航帶方案放了 {len(gcps)} 個 GCP（> {CROSS_PRESET_GCP_WARN}）：這已不是「少 GCP」。"
                        "Gerke & Przybilla 2016：GCP 到 17–18 個時交叉航帶有無的差異消失 ⇒ 拿這組比較交叉航帶的效益會失效")
    if large and len(gcps) < params.large_min_gcp:
        warnings.append(f"作業區 {area_m2 / 1e6:.2f} km² ≥ {params.large_area_km2:g} km²（大案，我們的門檻），"
                        f"ODM 建議 8–10 個 GCP，這個方案只有 {len(gcps)} 個"
                        + ("（交叉航帶方案刻意少 GCP；比較時要記得這一點）" if preset.name == "cross_strips" else ""))

    # ── 檢核點 ──
    chk_area = _inset(union, chk_inset)
    if gcps:
        allowed = chk_area.difference(unary_union([Point(x, y).buffer(chk_gcp, quad_segs=32) for x, y in gxy()]))
    else:
        allowed = chk_area
    chks = []           # dict(e, n, purpose, note, seam, pair)
    seam_out = []
    for s in seams:
        ra = excl[s.a].intersection(allowed).difference(bgeo[s.b].buffer(pair_off))
        rb = excl[s.b].intersection(allowed).difference(bgeo[s.a].buffer(pair_off))
        pa, pb = _nearest_in(pool, ra, s.mid), _nearest_in(pool, rb, s.mid)
        mlat, mlon = frame.ll(s.mid[0], s.mid[1])
        rec = {"blocks": [s.a, s.b], "kind": s.kind, "length_m": s.length_m, "width_m": s.width_m,
               "mid_lat": float(mlat), "mid_lon": float(mlon), "gcp": None, "chk_pair": None,
               "pair_distance_m": None}
        if pa is None or pb is None:
            miss = "、".join(x for x, p in ((s.a, pa), (s.b, pb)) if p is None)
            warnings.append(f"接縫 {s.a}｜{s.b}：區塊 {miss} 那一側放不下檢核點（離對方區塊 ≥{pair_off:.1f} m、"
                            f"離 GCP ≥{chk_gcp:.1f} m、內縮 {chk_inset:.1f} m）⇒ 這條接縫量不到高程階差")
        else:
            seam_label = f"{s.a}|{s.b}"
            k = len([c for c in chks if c["seam"] == seam_label]) + 1
            for side, p, other in ((s.a, pa, s.b), (s.b, pb, s.a)):
                chks.append({"e": p[0], "n": p[1], "purpose": "seam_pair", "seam": seam_label, "pair": k,
                             "note": f"seam={seam_label} pair={k} side={side}"})
            rec["pair_distance_m"] = float(math.hypot(pa[0] - pb[0], pa[1] - pb[1]))
        seam_out.append(rec)

    n_req = int(params.n_checkpoints)
    n_fixed = len(chks)
    if n_fixed > n_req:
        warnings.append(f"每條接縫一對檢核點需要 {n_fixed} 個，多於要求的 {n_req} 個 ⇒ 檢核點數改為 {n_fixed}"
                        "（接縫規則優先）")
    n_free = max(0, n_req - n_fixed)
    cand = pool[_in(allowed, pool)]
    if n_free > len(cand):
        warnings.append(f"允許放檢核點的範圍只有 {len(cand)} 個候選點，放不下 {n_free} 個分層檢核點")
        n_free = len(cand)
    if n_free and not len(cand):
        warnings.append("扣掉 GCP 周圍與邊緣帶之後，沒有可以放分層檢核點的地方")
    free = _cvt(cand, [(c["e"], c["n"]) for c in chks], n_free, np.random.default_rng(int(params.seed) + 1))
    if len(free) < n_free:
        warnings.append(f"分層檢核點只放得下 {len(free)} 個（要求 {n_free}）")
    order = sorted(range(len(free)), key=lambda i: (-round(free[i, 1], 3), round(free[i, 0], 3)))
    for i in order:
        chks.append({"e": float(free[i, 0]), "n": float(free[i, 1]), "purpose": "stratified", "seam": "",
                     "pair": 0, "note": "分層均勻"})

    # ── 組結果 ──
    def label_of(x, y):
        pt = Point(x, y)
        return "|".join(lb for lb in sorted(bgeo) if bgeo[lb].buffer(0.5).covers(pt))

    points = []
    for i, g in enumerate(gcps, start=1):
        lat, lon = frame.ll(g["e"], g["n"])
        points.append(PlannedPoint(f"GCP{i:02d}", "GCP", float(lat), float(lon), label_of(g["e"], g["n"]),
                                   g["purpose"], g["note"], g["e"], g["n"]))
    for i, c in enumerate(chks, start=1):
        lat, lon = frame.ll(c["e"], c["n"])
        points.append(PlannedPoint(f"CHK{i:02d}", "CHK", float(lat), float(lon), label_of(c["e"], c["n"]),
                                   c["purpose"], c["note"], c["e"], c["n"], seam=c["seam"], pair=c["pair"]))
    # 接縫紀錄補上點代號
    for rec in seam_out:
        lab = "|".join(rec["blocks"])
        ids = [p.id for p in points if p.type == "CHK" and p.seam == lab]
        rec["chk_pair"] = ids[:2] if ids else None
    for p in points:
        if p.type == "GCP" and p.purpose == "seam":
            for rec in seam_out:
                if p.note.endswith(f"{rec['blocks'][0]}｜{rec['blocks'][1]}"):
                    rec["gcp"] = p.id

    blocks_info = {}
    for label in sorted(bgeo):
        inside = [p for p in points if label in p.block.split("|")]
        blat, blon = frame.ll(*_centre_of(bgeo[label]))
        blocks_info[label] = {"area_m2": float(bgeo[label].area), "n_gcp": sum(p.type == "GCP" for p in inside),
                              "n_chk": sum(p.type == "CHK" for p in inside), "centre_lat": float(blat),
                              "centre_lon": float(blon)}
    n_g = sum(p.type == "GCP" for p in points)
    n_c = sum(p.type == "CHK" for p in points)
    counts = {"GCP": n_g, "CHK": n_c, "total": n_g + n_c,
              "GCP_by_purpose": _count_by(points, "GCP"), "CHK_by_purpose": _count_by(points, "CHK"),
              "seams": len(seams), "seam_pairs": sum(1 for r in seam_out if r["chk_pair"]),
              "n_checkpoints_effective": n_c}
    eff["n_checkpoints_effective"] = n_c
    plan = LayoutPlan(points=points, effective=eff, warnings=warnings, notes=notes, counts=counts, seams=seam_out,
                      blocks=blocks_info, checks=[], areas=rings, block_rings=block_rings)
    plan.checks = check_layout(plan, _frame=frame, _union=union, _bgeo=bgeo)
    return plan


def _count_by(points, typ) -> dict:
    out = {}
    for p in points:
        if p.type == typ:
            out[p.purpose] = out.get(p.purpose, 0) + 1
    return dict(sorted(out.items()))


def check_layout(plan: LayoutPlan, *, _frame=None, _union=None, _bgeo=None) -> list:
    """逐條規則核對（獨立於放點的程式路徑重新量距離）：[(規則, 通過?, 說明)]。"""
    eff = plan.effective
    frame = _frame or LocalFrame(*np.asarray([p for r in plan.areas for p in r], float).mean(axis=0))
    union = _union if _union is not None else unary_union([frame.polygon(r) for r in plan.areas])
    if _bgeo is not None:
        bgeo = _bgeo
    else:
        bgeo = {lb: unary_union([frame.polygon(r) for r in rs]) for lb, rs in plan.block_rings.items()}
    xy = {p.id: np.asarray(frame.en(p.lat, p.lon), float) for p in plan.points}
    g = [p for p in plan.points if p.type == "GCP"]
    c = [p for p in plan.points if p.type == "CHK"]
    out = []
    out.append((f"GCP 總數 ≥ {eff['min_total']}", len(g) >= eff["min_total"], f"{len(g)} 個"))
    bnd = union.boundary
    if g:
        dmin = min(float(bnd.distance(Point(*xy[p.id]))) for p in g)
        out.append((f"GCP 距作業區邊界 ≥ {eff['inset_m']:.1f} m（避開邊緣低視數帶）",
                    dmin >= eff["inset_m"] - 0.05, f"最近 {dmin:.1f} m"))
        if len(g) > 1:
            gg = np.asarray([xy[p.id] for p in g])
            d = np.hypot(gg[:, None, 0] - gg[None, :, 0], gg[:, None, 1] - gg[None, :, 1])
            d[np.diag_indices(len(g))] = np.inf
            out.append((f"GCP 間距 ≥ {eff['gcp_min_spacing_m']:.1f} m（ODM：大於單張足跡）",
                        float(d.min()) >= eff["gcp_min_spacing_m"] - 0.05, f"最近 {float(d.min()):.1f} m"))
    for lb in sorted(bgeo):
        n = sum(1 for p in g if bgeo[lb].buffer(0.5).covers(Point(*xy[p.id])))
        out.append((f"UAV 區塊 {lb} 內 ≥1 個 GCP", n >= 1, f"{n} 個"))
    for s in eff.get("split") or []:
        out.append((f"ODM 子模型 {s['submodel']} ≥ {eff['min_gcp_per_split']} 個 GCP",
                    s["n_gcp"] >= eff["min_gcp_per_split"], f"{s['n_gcp']} 個"))
    if c and g:
        cc = np.asarray([xy[p.id] for p in c])
        gg = np.asarray([xy[p.id] for p in g])
        d = float(np.hypot(cc[:, None, 0] - gg[None, :, 0], cc[:, None, 1] - gg[None, :, 1]).min())
        out.append((f"檢核點與 GCP 相距 ≥ {eff['chk_gcp_min_m']:.1f} m", d >= eff["chk_gcp_min_m"] - 0.05,
                    f"最近 {d:.1f} m"))
    if c:
        dmin = min(float(bnd.distance(Point(*xy[p.id]))) for p in c)
        out.append((f"檢核點距作業區邊界 ≥ {eff['chk_inset_m']:.1f} m", dmin >= eff["chk_inset_m"] - 0.05,
                    f"最近 {dmin:.1f} m"))
    for rec in plan.seams:
        out.append((f"接縫 {rec['blocks'][0]}｜{rec['blocks'][1]} 有一對檢核點", bool(rec["chk_pair"]),
                    "、".join(rec["chk_pair"]) if rec["chk_pair"] else "沒有"))
    out.append((f"檢核點數 = 要求的 {eff['n_checkpoints_requested']}",
                len(c) == eff["n_checkpoints_requested"], f"{len(c)} 個"))
    out.append(("ASPRS 第 2 版：檢核點 ≥ 30（否則附小專案聲明）", len(c) >= 30, f"{len(c)} 個"))
    return out


def compare_presets(areas, blocks=None, params: Optional[LayoutParams] = None, *, blocks_source: str = "") -> dict:
    """兩個方案各產生一次（其他參數相同）→ {方案: 摘要}，給「比較」表用。"""
    params = params or LayoutParams()
    out = {}
    for name in PRESETS:
        p = plan_layout(areas, blocks, _replace(params, preset=name), blocks_source=blocks_source)
        out[name] = {"label": PRESETS[name].label, "GCP": p.counts["GCP"], "CHK": p.counts["CHK"],
                     "total": p.counts["total"], "seams": p.counts["seams"], "seam_pairs": p.counts["seam_pairs"],
                     "n_warnings": len(p.warnings), "flight": "加飛交叉航帶（面積與高度依交叉航帶設定）" if name == "cross_strips"
                     else "不加飛", "description": PRESETS[name].description}
    return out


def _replace(params: LayoutParams, **kw) -> LayoutParams:
    from dataclasses import replace
    return replace(params, **kw)


# ─────────────────────────────────────────────────────────────────────────────
# ODM gcp_list.txt：標頭（與 ODM opendm/location.py parse_srs_header 同一套規則）、讀檔、驗證
# ─────────────────────────────────────────────────────────────────────────────

def parse_odm_srs_header(header: str) -> CRS:
    """ODM 接受的三種標頭：``WGS84 UTM <zone><N|S>``、含 ``+proj`` 的 PROJ 字串、``EPSG:<code>``；其他 ODM 會直接失敗。"""
    h = str(header or "").strip().lstrip("﻿")
    ref = h.split(" ")
    try:
        if len(ref) >= 3 and ref[0] == "WGS84" and ref[1] == "UTM":
            pole = ref[2][-1].upper()
            zone = int(ref[2][:-1])
            if pole not in ("N", "S") or not (1 <= zone <= 60):
                raise ValueError
            return CRS.from_proj4(f"+proj=utm +zone={zone} +datum=WGS84 +units=m +no_defs"
                                  + (" +south" if pole == "S" else ""))
        if "+proj" in h:
            return CRS.from_proj4(h.strip("'"))
        if h.lower().startswith("epsg:"):
            return CRS.from_epsg(int(h[5:]))
    except (ValueError, CRSError) as e:
        raise GcpPlanError(f"gcp_list.txt 標頭 {h!r} 解析失敗：{e}") from None
    raise GcpPlanError(f"gcp_list.txt 標頭 {h!r} 不是 ODM 認得的格式（WGS84 UTM 51N／+proj=…／EPSG:xxxx）")


def utm_header_for(lat: float, lon: float) -> str:
    zone = int((lon + 180.0) // 6.0) + 1
    return f"WGS84 UTM {zone}{'N' if lat >= 0 else 'S'}"


_PLACEHOLDER_RE = re.compile(r"[<>]")


@dataclass
class GcpLine:
    lineno: int
    x: float
    y: float
    z: float
    im_x: float
    im_y: float
    image: str
    name: str                  # 第 7 欄（GCP 名稱）；沒有時用座標當鍵（與 ODM check_entries 相同）


def read_gcp_list(path) -> tuple:
    """讀 gcp_list.txt → (標頭, CRS, [GcpLine], 問題清單)。不合格的行列在問題清單（不靜默跳過）。"""
    p = Path(path)
    try:
        text = p.read_text(encoding="utf-8-sig")
    except (OSError, UnicodeDecodeError) as e:
        raise GcpPlanError(f"讀不到 {p}：{e}") from None
    lines = [ln.strip() for ln in text.replace("﻿", "").strip().split("\n")]
    if not lines or not lines[0]:
        raise GcpPlanError(f"{p.name} 是空的（第一行要是座標系標頭）")
    header = lines[0]
    crs = parse_odm_srs_header(header)
    out, problems = [], []
    for i, ln in enumerate(lines[1:], start=2):
        if not ln or ln.startswith("#"):
            continue
        parts = ln.split()
        if _PLACEHOLDER_RE.search(ln):
            problems.append(f"第 {i} 行還有樣板佔位（<…>）：{ln[:80]}")
            continue
        if len(parts) < 6:
            problems.append(f"第 {i} 行只有 {len(parts)} 欄（ODM 至少要 geo_x geo_y geo_z im_x im_y image_name）")
            continue
        try:
            x, y, z, ix, iy = (float(v) for v in parts[:5])
        except ValueError:
            problems.append(f"第 {i} 行前 5 欄不是數字：{ln[:80]}")
            continue
        if not all(math.isfinite(v) for v in (x, y, z, ix, iy)):
            problems.append(f"第 {i} 行有非有限值（NaN／inf；ODM 文件：不要用 NaN）")
            continue
        name = parts[6] if len(parts) >= 7 else f"{parts[0]} {parts[1]} {parts[2]}"
        out.append(GcpLine(i, x, y, z, ix, iy, parts[5], name))
    return header, crs, out, problems


def validate_gcp_list(path, *, image_names: Optional[Sequence[str]] = None, image_sizes: Optional[Mapping] = None,
                      gcp_vertical: Optional[str] = None, height: Optional[str] = None,
                      forbidden_names: Sequence[str] = (), min_images: int = 5, min_gcps: int = 5) -> dict:
    """送 ODM 前核對 gcp_list.txt。回傳摘要（errors 非空 ⇒ 不能送）。

    * 高程基準：``gcp_vertical`` 必須明確給，而且要等於 geo.txt 用的 ``height``（不混用，評估 §8）。
    * 影像檔名：不在本次上傳影像裡的行，ODM 會**靜默丟掉**（opendm/gcp.py make_filtered_copy）⇒ 這裡擋下。
    * 像素座標超出影像 ⇒ 擋（多半是單位或寬高顛倒）。
    * 檢核點的名稱出現在 GCP 檔 ⇒ 擋（檢核點一旦當控制點，精度報告就沒有意義）。
    * 每個 GCP 少於 ``min_images`` 張 ⇒ 警告（ODM 文件建議 ≥5；ODM 程式少於 3 張才警告）；GCP 少於 ``min_gcps`` ⇒ 警告。
    * geo_z = 0.0 ⇒ 警告（ODM 文件把 0.0 當「沒有高程」）。
    """
    p = Path(path)
    errors, warns = [], []
    if not p.is_file():
        raise GcpPlanError(f"GCP 檔不存在：{p}")
    if p.suffix.lower() != ".txt" or p.name.lower() in ("geo.txt", "image_groups.txt"):
        errors.append(f"GCP 檔必須是 .txt、而且不能叫 geo.txt／image_groups.txt（NodeODM 依檔名分類）：{p.name}")
    if gcp_vertical not in VERTICALS:
        errors.append("請明確指定 gcp_list.txt 的 geo_z 是橢球高還是正高（不猜）")
    elif height is not None and gcp_vertical != height:
        errors.append(f"gcp_list.txt 的高程是{_vtext(gcp_vertical)}，geo.txt／EXIF 用的是{_vtext(height)}："
                      "兩者混用會差一個大地起伏（本場址約 18.5 m）⇒ 不送")
    header, crs, lines, problems = read_gcp_list(p)
    errors += problems
    names = set(image_names) if image_names is not None else None
    per_gcp = {}
    seen = set()
    for g in lines:
        if names is not None and g.image not in names:
            errors.append(f"第 {g.lineno} 行的影像 {g.image} 不在本次要上傳的影像裡（ODM 會靜默丟掉這一行）")
        if image_sizes and g.image in image_sizes:
            wpx, hpx = image_sizes[g.image]
            if not (0 <= g.im_x <= wpx and 0 <= g.im_y <= hpx):
                errors.append(f"第 {g.lineno} 行像素 ({g.im_x:g}, {g.im_y:g}) 超出影像 {wpx}×{hpx}")
        if (g.name, g.image) in seen:
            warns.append(f"第 {g.lineno} 行：{g.name} 在 {g.image} 重複標記")
        seen.add((g.name, g.image))
        per_gcp.setdefault(g.name, {"images": set(), "xyz": (g.x, g.y, g.z)})["images"].add(g.image)
        if g.z == 0.0:
            warns.append(f"第 {g.lineno} 行 geo_z = 0.0（ODM 文件把 0.0 當作「沒有高程」）")
    bad = sorted(set(per_gcp) & set(forbidden_names))
    if bad:
        errors.append(f"檢核點 {bad} 出現在 GCP 檔：檢核點不能拿來當控制點，否則精度報告沒有意義")
    for name, rec in sorted(per_gcp.items()):
        n = len(rec["images"])
        if n < min_images:
            warns.append(f"{name} 只標在 {n} 張影像（ODM 建議 ≥{min_images}）")
    if len(per_gcp) < min_gcps:
        warns.append(f"GCP 只有 {len(per_gcp)} 個（ODM 建議至少 {min_gcps} 個）")
    if not lines:
        errors.append("GCP 檔沒有任何有效的資料行")
    return {"path": str(p), "header": header, "crs": crs.to_string(), "crs_is_geographic": bool(crs.is_geographic),
            "n_lines": len(lines), "n_gcps": len(per_gcp), "gcp_vertical": gcp_vertical,
            "images_per_gcp": {k: len(v["images"]) for k, v in sorted(per_gcp.items())},
            "errors": errors, "warnings": warns}


def _vtext(v) -> str:
    return {"ellipsoidal": "橢球高", "orthometric": "正高"}.get(v, str(v))


# ─────────────────────────────────────────────────────────────────────────────
# 輸出
# ─────────────────────────────────────────────────────────────────────────────

def layout_csv_text(plan: LayoutPlan) -> str:
    buf = io.StringIO()
    w = csv.DictWriter(buf, fieldnames=list(CSV_COLUMNS), lineterminator="\n")
    w.writeheader()
    for p in plan.points:
        w.writerow(p.row())
    return buf.getvalue()


def read_layout_csv(path) -> list:
    """讀回 layout CSV → [dict]（id, type, lat, lon, block, purpose, note, seam, pair）。接縫對從 note 的
    ``seam=A|B pair=k`` 取。"""
    out = []
    with Path(path).open("r", encoding="utf-8-sig", newline="") as fh:
        rd = csv.DictReader(fh)
        missing = [c for c in CSV_COLUMNS if c not in (rd.fieldnames or [])]
        if missing:
            raise GcpPlanError(f"{Path(path).name} 不是佈設 CSV：缺欄位 {missing}")
        for r in rd:
            m = re.search(r"seam=(\S+) pair=(\d+)", r.get("note") or "")
            out.append({**{c: r[c] for c in CSV_COLUMNS}, "lat": float(r["lat"]), "lon": float(r["lon"]),
                        "seam": m.group(1) if m else "", "pair": int(m.group(2)) if m else 0})
    return out


def layout_geojson(plan: LayoutPlan) -> dict:
    feats = []
    for p in plan.points:
        feats.append({"type": "Feature", "geometry": {"type": "Point", "coordinates": [round(p.lon, 8), round(p.lat, 8)]},
                      "properties": {"id": p.id, "type": p.type, "block": p.block, "purpose": p.purpose,
                                     "purpose_text": PURPOSE_TEXT.get(p.purpose, p.purpose), "note": p.note,
                                     "seam": p.seam, "pair": p.pair}})
    for k, r in enumerate(plan.areas, start=1):
        ring = [[round(b, 8), round(a, 8)] for a, b in r] + [[round(r[0][1], 8), round(r[0][0], 8)]]
        feats.append({"type": "Feature", "geometry": {"type": "Polygon", "coordinates": [ring]},
                      "properties": {"id": f"AREA{k}", "type": "AREA"}})
    return {"type": "FeatureCollection", "features": feats,
            "aeroplan": {"format": FORMAT, "effective": _jsonable(plan.effective), "counts": plan.counts,
                         "warnings": list(plan.warnings)}}


def layout_kml(plan: LayoutPlan, *, name: str = "GCP／檢核點佈設") -> str:
    style = {"GCP": "ff0000ff", "CHK": "ffffb000"}      # KML 顏色是 aabbggrr：GCP 紅、CHK 藍
    out = ['<?xml version="1.0" encoding="UTF-8"?>', '<kml xmlns="http://www.opengis.net/kml/2.2">', "<Document>",
           f"<name>{_xml_escape(name)}</name>"]
    for t, col in style.items():
        out.append(f'<Style id="{t}"><IconStyle><color>{col}</color><scale>1.1</scale></IconStyle>'
                   f"<LabelStyle><scale>0.9</scale></LabelStyle></Style>")
    out.append('<Style id="AREA"><LineStyle><color>ff00c1ff</color><width>2</width></LineStyle>'
               "<PolyStyle><fill>0</fill></PolyStyle></Style>")
    for t, title in (("GCP", "GCP（控制點）"), ("CHK", "檢核點")):
        out.append(f"<Folder><name>{title}</name>")
        for p in plan.points:
            if p.type != t:
                continue
            desc = f"{PURPOSE_TEXT.get(p.purpose, p.purpose)}；區塊 {p.block or '—'}；{p.note}"
            out.append(f"<Placemark><name>{_xml_escape(p.id)}</name><description>{_xml_escape(desc)}</description>"
                       f"<styleUrl>#{t}</styleUrl><Point><coordinates>{p.lon:.8f},{p.lat:.8f},0</coordinates>"
                       "</Point></Placemark>")
        out.append("</Folder>")
    out.append("<Folder><name>作業區</name>")
    for k, r in enumerate(plan.areas, start=1):
        coords = " ".join(f"{b:.8f},{a:.8f},0" for a, b in list(r) + [r[0]])
        out.append(f"<Placemark><name>作業區 {k}</name><styleUrl>#AREA</styleUrl><Polygon><outerBoundaryIs>"
                   f"<LinearRing><coordinates>{coords}</coordinates></LinearRing></outerBoundaryIs></Polygon>"
                   "</Placemark>")
    out.append("</Folder></Document></kml>")
    return "\n".join(out) + "\n"


def _to_header_xy(header: str, lat: float, lon: float) -> tuple:
    crs = parse_odm_srs_header(header)
    t = Transformer.from_crs("EPSG:4326", crs, always_xy=True)
    x, y = t.transform(lon, lat)
    if crs.is_geographic:
        return f"{x:.8f}", f"{y:.8f}"
    return f"{x:.3f}", f"{y:.3f}"


def gcp_template_text(plan: LayoutPlan, *, projection: str, vertical: str = "ellipsoidal",
                      survey_frame: str = DEFAULT_SURVEY_FRAME) -> str:
    """ODM gcp_list.txt **樣板**（只放 GCP；檢核點絕不能進這個檔）。

    第一行是 ODM 標頭；``#`` 開頭的說明行 ODM 會略過（opendm/gcp.py：``line[0] != "#"``）。每個 GCP 一行
    ``geo_x geo_y geo_z im_x im_y image_name gcp_name``：geo_x/geo_y 先放**規劃位置**換到標頭座標系的值，geo_z 與
    影像座標是 ``<…>`` 佔位 ⇒ 沒填完的樣板送不進流水線（validate_gcp_list 擋）。
    """
    if vertical not in VERTICALS:
        raise GcpPlanError(f"高程基準要是 {VERTICALS} 之一，收到 {vertical!r}")
    parse_odm_srs_header(projection)
    lines = [projection.strip(),
             f"# AeroPlan GCP 樣板（{FORMAT}；方案 {plan.effective['preset']}）",
             f"# geo_x geo_y 的座標系 = 第一行標頭；geo_z = {_vtext(vertical)}；外業量測框架 = {survey_frame}。",
             "# 下面的 geo_x geo_y 是「規劃位置」（WGS84 經緯度換算）：外業量完後要以**實測值**取代（同一座標系），",
             "# 或用 AeroPlan「由實測 CSV 產生 gcp_list」重新產生；geo_z 一定要填實測高（記得扣天線高）。",
             "# 每個 GCP 至少在 5 張影像上標記（ODM 文件）⇒ 每個 GCP 複製成 ≥5 行，各填一張影像的 im_x im_y 檔名。",
             "# 影像檔名要用 odm/images/ 裡的檔名（加標籤影像）；不在上傳清單裡的行 ODM 會靜默丟掉。",
             "# 檢核點（CHK）不可放進這個檔。"]
    for p in plan.gcps():
        x, y = _to_header_xy(projection, p.lat, p.lon)
        lines.append(f"{x} {y} <geo_z> <im_x> <im_y> <image_name> {p.id}")
    return "\n".join(lines) + "\n"


def read_field_survey_csv(path) -> list:
    """外業實測 CSV（SURVEY_COLUMNS 或 checkpoints.py 的欄位）→ [dict]。lat/lon/h 空白的列回報為缺漏。"""
    rows = []
    with Path(path).open("r", encoding="utf-8-sig", newline="") as fh:
        rd = csv.DictReader(fh)
        need = ("id", "lat", "lon", "h", "frame", "vertical_datum")
        miss = [c for c in need if c not in (rd.fieldnames or [])]
        if miss:
            raise GcpPlanError(f"{Path(path).name} 缺欄位 {miss}")
        for i, r in enumerate(rd, start=2):
            rows.append({**r, "_line": i})
    return rows


def gcp_list_from_survey(survey_csv, *, projection: str, vertical: str, frame_crs: Optional[str] = None) -> str:
    """實測 GCP（外業 CSV 的 type=GCP 列）→ 只差影像座標的 gcp_list.txt 文字。

    實測框架名（例如 e-GNSS[2025]）不是 PROJ 可解析的座標系時，必須用 ``frame_crs`` 明確說它對應哪個經緯度座標系
    （不猜）；高程基準必須等於 ``vertical``（不換算）。
    """
    if vertical not in VERTICALS:
        raise GcpPlanError(f"高程基準要是 {VERTICALS} 之一，收到 {vertical!r}")
    dst = parse_odm_srs_header(projection)
    rows = [r for r in read_field_survey_csv(survey_csv) if (r.get("type") or "GCP").strip().upper() == "GCP"]
    if not rows:
        raise GcpPlanError("實測 CSV 裡沒有 type=GCP 的列")
    ortho = None
    if vertical == "orthometric":
        # 「正高」不只一種：EGM96 與 TWVD2001 差的不是常數，同一份 gcp_list 混用就錯（與 checkpoints.py 一致）
        systems = {}
        for r in rows:
            vd = (r.get("vertical_datum") or "").strip()
            if _vkind(vd) == "orthometric":
                systems.setdefault(_ortho_system(vd), []).append(r["id"].strip())
        if None in systems:
            raise GcpPlanError(f"{systems[None]} 的高程基準只寫「正高」：要寫明是 EGM96 還是 TWVD2001（不猜）")
        if len(systems) > 1:
            raise GcpPlanError("同一份實測 CSV 混用了不同正高系統："
                               + "；".join(f"{k} {v}" for k, v in sorted(systems.items()))
                               + "（不換算、不混用）")
        ortho = next(iter(systems), None)
    vtxt = _vtext(vertical) + (f"（{ortho}）" if ortho else "")
    lines = [projection.strip(), f"# 由實測 CSV 產生；geo_z = {vtxt}；每個 GCP 複製成 ≥5 行填影像座標"]
    for r in rows:
        try:
            lat, lon, h = float(r["lat"]), float(r["lon"]), float(r["h"])
        except (TypeError, ValueError):
            raise GcpPlanError(f"第 {r['_line']} 行（{r.get('id')}）的 lat/lon/h 沒填或不是數字") from None
        vd = (r.get("vertical_datum") or "").strip()
        if _vkind(vd) != vertical:
            raise GcpPlanError(f"第 {r['_line']} 行（{r['id']}）的高程基準是 {vd!r}，不是{_vtext(vertical)}：不換算、不混用")
        src = _frame_crs((r.get("frame") or "").strip(), frame_crs)
        t = Transformer.from_crs(src, dst, always_xy=True)
        x, y = t.transform(lon, lat)
        fx = (f"{x:.8f}", f"{y:.8f}") if dst.is_geographic else (f"{x:.3f}", f"{y:.3f}")
        lines.append(f"{fx[0]} {fx[1]} {h:.3f} <im_x> <im_y> <image_name> {r['id'].strip()}")
    return "\n".join(lines) + "\n"


def _vkind(vd: str) -> Optional[str]:
    v = vd.strip().lower()
    if v in ("ellipsoidal", "ellipsoid", "h_ell", "橢球高"):
        return "ellipsoidal"
    if v in ("orthometric", "egm96", "twvd2001", "正高"):
        return "orthometric"
    return None


_ORTHO_SYSTEMS = ("EGM96", "TWVD2001")


def _ortho_system(vd: str) -> Optional[str]:
    """正高的具體系統（'EGM96'／'TWVD2001'）；只寫「正高」／orthometric ⇒ None（不夠明確）。"""
    v = vd.strip().lower()
    for s in _ORTHO_SYSTEMS:
        if v == s.lower():
            return s
    return None


def declared_ortho_system(path) -> Optional[str]:
    """gcp_list.txt 的 ``#`` 說明行裡宣告的正高系統（gcp_list_from_survey 寫的「geo_z = 正高（EGM96）」）；沒宣告 ⇒ None。"""
    try:
        text = Path(path).read_text(encoding="utf-8-sig")
    except (OSError, UnicodeDecodeError):
        return None
    for ln in text.splitlines():
        if ln.startswith("#"):
            m = re.search(r"geo_z = 正高（(\w+)）", ln)
            if m and m.group(1) in _ORTHO_SYSTEMS:
                return m.group(1)
    return None


def _frame_crs(frame: str, frame_crs: Optional[str]) -> CRS:
    if frame_crs:
        try:
            crs = CRS.from_user_input(frame_crs)
        except CRSError as e:
            raise GcpPlanError(f"框架對應的座標系 {frame_crs!r} 無法解析：{e}") from None
    else:
        try:
            crs = CRS.from_user_input(frame)
        except CRSError:
            raise GcpPlanError(f"實測框架 {frame!r} 不是 PROJ 可解析的座標系：請明確指定它對應的經緯度座標系"
                               "（例如 +proj=longlat +ellps=GRS80 +no_defs）——不會把它當 WGS84") from None
    if not crs.is_geographic:
        raise GcpPlanError(f"實測框架對應的座標系要是經緯度座標系，收到 {crs.to_string()}")
    return crs


def field_sheet_text(plan: LayoutPlan, *, survey_frame: str = DEFAULT_SURVEY_FRAME, vertical: str = "ellipsoidal",
                     projection: Optional[str] = None) -> str:
    eff = plan.effective
    L = ["AeroPlan 外業表：GCP／檢核點",
         f"方案：{eff['preset_label']}（{eff['preset']}）— {eff['preset_description']}",
         f"GCP {plan.counts['GCP']} 個、檢核點 {plan.counts['CHK']} 個；作業區 {eff['area_m2'] / 1e6:.3f} km²",
         "",
         "【量測規範（不混用基準）】",
         f"* 平面框架：{survey_frame}（e-GNSS【2025】不等於 TWD97，評估 §8）。",
         f"* 高程：{_vtext(vertical)}（e-GNSS 給的是橢球高）；每點記錄天線高，填實測值前先扣掉。",
         "* 本場址大地起伏約 +18.5 m：橢球高與正高混用會差這麼多。",
         "* 每點填 σ（水平、垂直，公尺）：σ水平＝平面二維 RMS（GNSS 的 HRMS），不是單軸 σ；σ垂直＝VRMS。"
         "ASPRS 第 2 版要把檢核點量測誤差與產品 RMSE 合成（RSS），"
         "而且檢核點精度要至少是產品的 2 倍。",
         "* 檢核點（CHK）不能拿來當控制點，也不要在 WebODM 裡標記它。",
         "* 每個 GCP 要在至少 5 張影像上看得到（ODM 文件）；標靶放在開闊平坦處、避開陰影與移動物。",
         ]
    if projection:
        L.append(f"* gcp_list.txt 標頭（geo_x geo_y 的座標系）：{projection}")
    L += ["", f"內縮 {eff['inset_m']:.1f} m（邊緣低視數帶）；GCP 間距 ≥ {eff['gcp_min_spacing_m']:.1f} m；"
          f"檢核點離 GCP ≥ {eff['chk_gcp_min_m']:.1f} m", ""]
    L.append("編號    類型  規劃緯度       規劃經度        區塊      用途")
    for p in plan.points:
        L.append(f"{p.id:<7} {p.type:<4} {p.lat:.8f}  {p.lon:.8f}  {p.block or '—':<8}  "
                 f"{PURPOSE_TEXT.get(p.purpose, p.purpose)}｜{p.note}")
        L.append("        實測：緯度 ____________ 經度 ____________ 高 ________ 天線高 ______ "
                 "σ水平 ____ σ垂直 ____ 時間 ______")
    if plan.warnings:
        L += ["", "【規劃警告】"] + [f"* {w}" for w in plan.warnings]
    return "\n".join(L) + "\n"


def survey_template_csv_text(plan: LayoutPlan, *, survey_frame: str = DEFAULT_SURVEY_FRAME,
                             vertical: str = "ellipsoidal") -> str:
    """外業填寫用 CSV：規劃位置在 planned_*；lat/lon/h/σ 空白給外業填（checkpoints.py 與 gcp_list_from_survey 讀）。"""
    buf = io.StringIO()
    w = csv.DictWriter(buf, fieldnames=list(SURVEY_COLUMNS), lineterminator="\n")
    w.writeheader()
    for p in plan.points:
        w.writerow({"id": p.id, "type": p.type, "planned_lat": f"{p.lat:.8f}", "planned_lon": f"{p.lon:.8f}",
                    "lat": "", "lon": "", "h": "", "frame": survey_frame, "vertical_datum": vertical,
                    "sigma_h_m": "", "sigma_v_m": "", "antenna_height_m": "", "block": p.block, "purpose": p.purpose})
    return buf.getvalue()


def export_layout(plan: LayoutPlan, out_dir, *, projection: str, vertical: str = "ellipsoidal",
                  survey_frame: str = DEFAULT_SURVEY_FRAME, overwrite: bool = False, stem: str = "gcp_plan") -> dict:
    """寫出 CSV、KML、GeoJSON、gcp_list 樣板、外業表、外業 CSV、effective JSON。同輸入 ⇒ 位元組相同。"""
    if vertical not in VERTICALS:
        raise GcpPlanError(f"高程基準要是 {VERTICALS} 之一，收到 {vertical!r}")
    parse_odm_srs_header(projection)
    out = Path(out_dir)
    files = {"csv": out / f"{stem}.csv", "kml": out / f"{stem}.kml", "geojson": out / f"{stem}.geojson",
             "gcp_template": out / "gcp_list_template.txt", "field_sheet": out / "field_sheet.txt",
             "survey_csv": out / "field_survey_template.csv", "effective": out / f"{stem}.effective.json"}
    exist = [p.name for p in files.values() if p.exists()]
    if exist and not overwrite:
        raise GcpPlanError(f"{out} 已有 {exist}：請換資料夾或明確選擇覆寫")
    out.mkdir(parents=True, exist_ok=True)
    texts = {
        "csv": layout_csv_text(plan),
        "kml": layout_kml(plan),
        "geojson": json.dumps(layout_geojson(plan), ensure_ascii=False, indent=1, sort_keys=True) + "\n",
        "gcp_template": gcp_template_text(plan, projection=projection, vertical=vertical, survey_frame=survey_frame),
        "field_sheet": field_sheet_text(plan, survey_frame=survey_frame, vertical=vertical, projection=projection),
        "survey_csv": survey_template_csv_text(plan, survey_frame=survey_frame, vertical=vertical),
        "effective": json.dumps({"format": FORMAT, "effective": _jsonable(plan.effective), "counts": plan.counts,
                                 "seams": plan.seams, "blocks": plan.blocks, "warnings": plan.warnings,
                                 "notes": plan.notes, "checks": [list(c) for c in plan.checks],
                                 "export": {"projection": projection, "vertical": vertical,
                                            "survey_frame": survey_frame}},
                                ensure_ascii=False, indent=1, sort_keys=True, default=_jsonable) + "\n",
    }
    for k, p in files.items():
        # CSV 給 Excel 開：UTF-8 BOM（中文備註才不會亂碼）；其他 UTF-8 無 BOM；一律 LF
        enc = "utf-8-sig" if k in ("csv", "survey_csv") else "utf-8"
        p.write_text(texts[k], encoding=enc, newline="\n")
    return {k: p for k, p in files.items()}


def _jsonable(o):
    if isinstance(o, dict):
        return {str(k): _jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_jsonable(v) for v in o]
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, float) and not math.isfinite(o):
        return None
    if isinstance(o, (str, int, float, bool)) or o is None:
        return o
    return str(o)


def map_points(plan: LayoutPlan) -> list:
    """地圖圖層用：[{id, type, lat, lon, label}]。"""
    return [{"id": p.id, "type": p.type, "lat": p.lat, "lon": p.lon,
             "label": p.id + (f"（{p.block}）" if p.block else "")} for p in plan.points]
