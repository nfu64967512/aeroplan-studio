"""找平坦位置 —— 在作業區附近找「起伏夠小、不用豁免也能規劃」的同尺寸窗口。

為什麼需要（2026-10-10 使用者：「如果能不用豁免是最好的」）：HARVEST 的可用高度帶是
``AGL 上限 − 最低固定翼作業高度 − 高度帶下限 ≥ 地形起伏``。法規上限 120 m、Pioneer 最低 100 m ⇒ 只容得下
≤15 m 的起伏（HERO 最低 90 m、下限 5 ⇒ 25 m）。實測桃園 20×30 km 內，與使用者作業區同大小的陸地窗口
起伏 ≤15 m 的**一個都沒有**、≤25 m 的只占約 0.3% ⇒ 要靠搜尋，不能靠運氣。

演算法：以 ``step_m``（預設 60 m）取樣地形成網格 → 對「作業區外框大小」的窗口算 max−min（scipy 的
maximum/minimum filter）→ 排除含水面/海面的窗口 → 依「能不能不用豁免、起伏、距離」排序 → 非極大抑制。

⚠ 起伏是用**外框**（軸對齊的 bbox）算的，比實際多邊形保守；旋轉過的多邊形可能略平。
⚠ 地形資料是地表高度（含樹與建物）、約 35 m 解析度：結果是「值得去看的候選」，不是保證。
⚠ 不檢查起降場坡度（≤5°）與禁航區；平移後請重新確認起降場。
⚠ 水面判定是啟發式：高度眾數占 ≥8%、且該格 5×5 鄰域起伏 <0.5 m（海面/水面/資料空洞的高度「完全固定」）
  才當水面，含 >1% 這種格的窗口排除。真平地的高度總有細微變化，不會被誤殺。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

__all__ = ["FlatCandidate", "FlatSearch", "FlatSearchError", "find_flat_windows", "required_ceiling_m",
           "areas_window", "translate_areas", "translate_point", "transform_point", "transform_areas"]

LatLon = Tuple[float, float]
_M_LAT = 111320.0


class FlatSearchError(RuntimeError):
    """搜尋失敗（取樣失敗太多、參數不合理）。"""


@dataclass(frozen=True)
class FlatCandidate:
    rank: int
    lat: float                      # 窗口中心
    lon: float
    relief_m: float                 # 窗口內 max − min
    z_min: float
    z_max: float
    distance_km: float              # 與原作業區中心的距離
    need_m: Dict[str, float]        # 各構型需要的 AGL 上限 = max(最低作業高度, 淨空) + 下限 + 起伏
    fits: Dict[str, bool]           # 各構型在法規上限下是否容得下（不用豁免）
    ceil_m: float = 120.0           # 上面「容得下」所對的 AGL 上限

    @property
    def slack_m(self) -> Dict[str, float]:
        """各構型的餘裕 = 上限 − 需要（<0 ⇒ 不用豁免容不下）。"""
        return {c: self.ceil_m - v for c, v in self.need_m.items()}

    def to_dict(self) -> Dict[str, Any]:
        return {"rank": self.rank, "lat": self.lat, "lon": self.lon, "relief_m": self.relief_m,
                "z_min": self.z_min, "z_max": self.z_max, "distance_km": self.distance_km,
                "need_m": dict(self.need_m), "fits": dict(self.fits), "ceil_m": self.ceil_m}


@dataclass
class FlatSearch:
    candidates: List[FlatCandidate]
    window_h_m: float
    window_w_m: float
    step_m: float
    cells: int
    ceil_m: float
    here_relief_m: Optional[float] = None       # 原位置的起伏（同尺寸窗口）
    water_fraction: float = 0.0
    notes: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {"candidates": [c.to_dict() for c in self.candidates], "window_h_m": self.window_h_m,
                "window_w_m": self.window_w_m, "step_m": self.step_m, "cells": self.cells, "ceil_m": self.ceil_m,
                "here_relief_m": self.here_relief_m, "water_fraction": self.water_fraction,
                "notes": list(self.notes), "any_fits": self.any_fits}

    @property
    def any_fits(self) -> Dict[str, bool]:
        keys = sorted({k for c in self.candidates for k in c.fits})
        return {k: any(c.fits.get(k, False) for c in self.candidates) for k in keys}


def required_ceiling_m(relief_m: float, min_fw_agl_m: float, band_min_m: float, clearance_m: float = 36.0) -> float:
    """需要的 AGL 上限（與 ``SwarmCoordinator`` 的高度帶預檢同一條式子）。"""
    return max(float(min_fw_agl_m), float(clearance_m)) + float(band_min_m) + float(relief_m)


# ─────────────────────────────────────────────────────────────────────
#  幾何
# ─────────────────────────────────────────────────────────────────────
def areas_window(areas: Sequence[Sequence[LatLon]]) -> Tuple[float, float, LatLon]:
    """所有作業區的聯集外框 → ``(南北長 m, 東西寬 m, 外框中心)``。"""
    pts = [(float(la), float(lo)) for a in areas for (la, lo) in a]
    if len(pts) < 3:
        raise FlatSearchError("需要至少一塊有 3 個角點的作業區")
    la0, la1 = min(p[0] for p in pts), max(p[0] for p in pts)
    lo0, lo1 = min(p[1] for p in pts), max(p[1] for p in pts)
    lat_c, lon_c = 0.5 * (la0 + la1), 0.5 * (lo0 + lo1)
    m_lon = _M_LAT * max(0.2, math.cos(math.radians(lat_c)))
    return (la1 - la0) * _M_LAT, (lo1 - lo0) * m_lon, (lat_c, lon_c)


def translate_point(p: LatLon, src: LatLon, dst: LatLon) -> LatLon:
    """把 ``p`` 隨「src → dst」平移（以度為單位，形狀不變）。"""
    return (float(p[0]) + (dst[0] - src[0]), float(p[1]) + (dst[1] - src[1]))


def transform_point(p: LatLon, src: LatLon, dst: LatLon, scale: float = 1.0) -> LatLon:
    """``p`` 以 ``src`` 為中心縮放 ``scale`` 倍後，再把 ``src`` 搬到 ``dst``（以度為單位，形狀不變）。"""
    return (dst[0] + (float(p[0]) - src[0]) * scale, dst[1] + (float(p[1]) - src[1]) * scale)


def transform_areas(areas: Sequence[Sequence[LatLon]], src: LatLon, dst: LatLon,
                    scale: float = 1.0) -> List[List[LatLon]]:
    if scale <= 0:
        raise FlatSearchError("縮放倍率必須為正")
    return [[transform_point(p, src, dst, scale) for p in a] for a in areas]


def translate_areas(areas: Sequence[Sequence[LatLon]], src: LatLon, dst: LatLon) -> List[List[LatLon]]:
    return transform_areas(areas, src, dst, 1.0)


def _dist_km(a: LatLon, b: LatLon) -> float:
    dn = (a[0] - b[0]) * _M_LAT
    de = (a[1] - b[1]) * _M_LAT * math.cos(math.radians(0.5 * (a[0] + b[0])))
    return math.hypot(dn, de) / 1000.0


# ─────────────────────────────────────────────────────────────────────
#  搜尋
# ─────────────────────────────────────────────────────────────────────
def find_flat_windows(terrain: Any, center: LatLon, radius_km: float, height_m: float, width_m: float, *,
                      min_fw_agl_m: Optional[Dict[str, float]] = None, band_min_m: float = 15.0,
                      clearance_m: float = 36.0, ceil_m: float = 120.0, step_m: float = 30.0,
                      margin_m: float = 200.0, top_n: int = 8, exclude_water: bool = True, max_nodes: int = 2_500_000) -> FlatSearch:
    """在 ``center`` 周圍 ``radius_km`` 內找同尺寸（``height_m`` × ``width_m``）的低起伏窗口。

    ``terrain``：``prefetch((lat_min, lon_min, lat_max, lon_max))`` 與 ``get_elevation(lat, lon)``
    （例：:class:`core.terrain.CesiumWorldTerrain`；回 ``None`` = 取樣失敗）。
    ``min_fw_agl_m``：``{'A': 100, 'B': 100}`` 各構型最低固定翼作業高度（決定「不用豁免」的起伏門檻）。
    ``margin_m``：窗口四周外擴多少再量起伏。規劃器的高度帶是逐胞、**胞外擴一個轉彎距離 d_turn** 後量地形
    （``decompose.altitude_band``：Ω = 胞 ⊕ disc(d_turn)），HERO 約 120 m 轉彎半徑加端點延伸 ⇒ 預設 200 m。
    不外擴時預測偏樂觀（實測 0.7 倍邊長的窗口預測可行、規劃器卻只覆蓋一部分）。
    """
    from scipy.ndimage import maximum_filter, minimum_filter, uniform_filter

    if radius_km <= 0 or height_m <= 0 or width_m <= 0:
        raise FlatSearchError("搜尋半徑與窗口尺寸必須為正")
    mf = dict(min_fw_agl_m or {"A": 100.0, "B": 100.0})
    lat_c, lon_c = float(center[0]), float(center[1])
    m_lon = _M_LAT * max(0.2, math.cos(math.radians(lat_c)))
    half_la = (radius_km * 1000.0 + 0.5 * height_m + margin_m) / _M_LAT
    half_lo = (radius_km * 1000.0 + 0.5 * width_m + margin_m) / m_lon
    bounds = (lat_c - half_la, lon_c - half_lo, lat_c + half_la, lon_c + half_lo)
    terrain.prefetch(bounds)

    step = max(float(step_m), 10.0)
    for _ in range(10):
        nla = int((bounds[2] - bounds[0]) * _M_LAT / step) + 1
        nlo = int((bounds[3] - bounds[1]) * m_lon / step) + 1
        if nla * nlo <= max_nodes:
            break
        step *= 1.5
    lats = np.linspace(bounds[0], bounds[2], nla)
    lons = np.linspace(bounds[1], bounds[3], nlo)
    grid_fn = getattr(terrain, "sample_grid", None)
    if grid_fn is not None:
        Z = np.asarray(grid_fn(lats, lons), dtype=float)
        bad = int((~np.isfinite(Z)).sum())
    else:
        Z = np.full((nla, nlo), np.nan)
        bad = 0
        for i, la in enumerate(lats):
            for j, lo in enumerate(lons):
                v = terrain.get_elevation(float(la), float(lo))
                if v is None or not np.isfinite(v):
                    bad += 1
                else:
                    Z[i, j] = v
    if bad > 0.05 * Z.size:
        raise FlatSearchError(f"地形取樣失敗 {bad}/{Z.size} 格（>5%）：{(getattr(terrain, 'failures', None) or ['?'])[0]}")

    notes: List[str] = []
    valid = np.isfinite(Z)
    water = np.zeros_like(valid)
    water_fraction = 0.0
    if exclude_water and valid.any():
        vals, cnt = np.unique(np.round(Z[valid]), return_counts=True)
        mode = float(vals[cnt.argmax()])
        share = float(cnt.max()) / float(valid.sum())
        if share >= 0.08:
            # 水面/海面/資料空洞 = 高度「完全固定」：眾數附近、且 5×5 鄰域幾乎沒有起伏。
            # 只看眾數占比會把大片真平地（高度略有變化、取整後落在同一格）誤殺——那正是要找的東西。
            Zv = np.where(valid, Z, mode)
            local = maximum_filter(Zv, size=5, mode='nearest') - minimum_filter(Zv, size=5, mode='nearest')
            water = valid & (np.abs(Z - mode) < 1.5) & (local < 0.5)
            water_fraction = float(water.mean())
            notes.append(f"水面/海面判定：高度眾數 {mode:.0f} m（占 {100 * share:.0f}%），其中完全平坦的格 "
                         f"{100 * water_fraction:.0f}%")

    h = max(2, int(round((height_m + 2 * margin_m) / step)))
    w = max(2, int(round((width_m + 2 * margin_m) / step)))
    Zf = np.where(valid, Z, np.nanmedian(Z))
    mx = maximum_filter(Zf, size=(h, w), mode='nearest')
    mn = minimum_filter(Zf, size=(h, w), mode='nearest')
    relief = mx - mn
    bad_frac = uniform_filter((~valid).astype(float), size=(h, w), mode='nearest')
    water_frac = uniform_filter(water.astype(float), size=(h, w), mode='nearest')
    ok = (bad_frac < 1e-9) & (water_frac < 0.01)
    # 中心要在半徑內、且窗口完整落在網格內（不吃邊界外推）
    ii, jj = np.meshgrid(np.arange(nla), np.arange(nlo), indexing='ij')
    ok &= (ii >= h // 2) & (ii < nla - (h - h // 2)) & (jj >= w // 2) & (jj < nlo - (w - w // 2))
    dn = (lats[:, None] - lat_c) * _M_LAT
    de = (lons[None, :] - lon_c) * m_lon
    ok &= np.hypot(dn, de) <= radius_km * 1000.0
    if not ok.any():
        raise FlatSearchError("搜尋範圍內沒有完整的陸地窗口（範圍太小或全是水面）")

    # 原位置的起伏（同尺寸窗口、取最近的格）
    i0 = int(np.clip(round((lat_c - lats[0]) * _M_LAT / step), 0, nla - 1))
    j0 = int(np.clip(round((lon_c - lons[0]) * m_lon / step), 0, nlo - 1))
    here = float(relief[i0, j0]) if valid[i0, j0] else None

    # 排序鍵：不用豁免能飛（任一構型）優先 → 起伏（2 m 一級，同級取近的）→ 距離
    fit_any = np.zeros_like(ok)
    for cls, mfa in mf.items():
        fit_any |= relief <= (ceil_m - max(float(mfa), clearance_m) - band_min_m) + 1e-9
    dist = np.hypot(dn, de)
    key = np.where(ok, (~fit_any).astype(float) * 1e6 + np.floor(relief / 2.0) * 1e3 + dist / 1000.0, np.inf)
    order = np.argsort(key, axis=None)
    min_sep = 0.5 * max(height_m, width_m) + margin_m
    picked: List[Tuple[int, int]] = []
    for k in order:
        if not np.isfinite(key.flat[k]):
            break
        i, j = np.unravel_index(k, key.shape)
        if all(math.hypot((lats[i] - lats[a]) * _M_LAT, (lons[j] - lons[b]) * m_lon) >= min_sep for a, b in picked):
            picked.append((int(i), int(j)))
        if len(picked) >= max(top_n * 4, 24):   # 候選池放大：粗網格會低估且同級很多，細取樣複核後名次會換
            break

    def _refine(lat: float, lon: float, rl: float, z0: float, z1: float) -> Tuple[float, float, float]:
        """細取樣（~30 m）複核起伏：粗網格的 max−min 會低估（60 m 網格把 66 m 量成 58 m）。"""
        fn = getattr(terrain, "relief_m", None)
        if fn is None:
            return rl, z0, z1
        try:
            la_h, lo_h = 0.5 * (height_m + 2 * margin_m) / _M_LAT, 0.5 * (width_m + 2 * margin_m) / m_lon
            lo_, hi_ = fn((lat - la_h, lon - lo_h, lat + la_h, lon + lo_h))
            return max(float(hi_ - lo_), rl), min(float(lo_), z0), max(float(hi_), z1)
        except Exception:                                   # noqa: BLE001 - 複核失敗就用粗值（寧可保守）
            return rl, z0, z1

    raw: List[Tuple[float, float, float, float, float, float]] = []
    for (i, j) in picked:
        zi0, zi1 = max(i - h // 2, 0), min(i + (h - h // 2), nla)
        zj0, zj1 = max(j - w // 2, 0), min(j + (w - w // 2), nlo)
        sub = Z[zi0:zi1, zj0:zj1]
        rl, z0, z1 = _refine(float(lats[i]), float(lons[j]), float(relief[i, j]),
                             float(np.nanmin(sub)), float(np.nanmax(sub)))
        raw.append((float(lats[i]), float(lons[j]), rl, z0, z1, _dist_km((float(lats[i]), float(lons[j])), (lat_c, lon_c))))

    def _fits_any(rl: float) -> bool:
        return any(required_ceiling_m(rl, mfa, band_min_m, clearance_m) <= ceil_m + 1e-9 for mfa in mf.values())

    raw.sort(key=lambda t: ((0 if _fits_any(t[2]) else 1), math.floor(t[2] / 2.0), t[5]))
    cands: List[FlatCandidate] = []
    for r, (la, lo, rl, z0, z1, dk) in enumerate(raw[:top_n], 1):
        need = {c: required_ceiling_m(rl, mfa, band_min_m, clearance_m) for c, mfa in mf.items()}
        cands.append(FlatCandidate(rank=r, lat=la, lon=lo, relief_m=rl, z_min=z0, z_max=z1, distance_km=dk,
                                   need_m=need, fits={c: need[c] <= ceil_m + 1e-9 for c in mf},
                                   ceil_m=float(ceil_m)))
    if here is not None and getattr(terrain, "relief_m", None) is not None:
        here = _refine(lat_c, lon_c, here, 0.0, 0.0)[0]          # 原位置也用細取樣（與規劃器看到的一致）
    return FlatSearch(candidates=cands, window_h_m=float(height_m), window_w_m=float(width_m), step_m=step,
                      cells=int(Z.size), ceil_m=float(ceil_m), here_relief_m=here, water_fraction=water_fraction,
                      notes=notes)
