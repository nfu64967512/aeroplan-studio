"""檢核點精度報告（ASPRS 第 2 版的報告方式）：實測檢核點 vs 成果（正射上量的平面座標、DSM 取的高程）。

輸入
====
* 實測檢核點 CSV（外業；``survey.gcp_plan`` 的 field_survey_template.csv 填完的樣子）：
  ``id, lat, lon, h, frame, vertical_datum, sigma_h_m, sigma_v_m``（可有 ``type`` 欄：只取 CHK）。
  σ 必填（ASPRS 第 2 版要把檢核點量測誤差合成進去 —— 不當 0）。
  ``sigma_h_m`` 是**平面二維 RMS**（GNSS 的 HRMS，= √(σE² + σN²)），不是單軸 σ：填單軸 σ 會讓量測誤差
  低估 √2 倍。``sigma_v_m`` 是垂直單軸 σ（VRMS）。
* 成果的平面座標（選填）：操作員在正射上量的 CSV ``id, lat, lon, frame``（可加 ``h, vertical_datum``）。
  框架名必須與實測檢核點**完全相同**（本模組不做框架轉換）。
* 成果的高程（選填）：DSM GeoTIFF，在實測 XY 上以 rasterio 雙線性取值。實測點先依 DSM 的 CRS 轉換
  （pyproj，always_xy）；DSM 的高程基準必須明確宣告，與檢核點不同時**拒絕**，除非明確給大地起伏 N。

輸出（ASPRS 第 2 版的報告方式；2026-10-15 依任務說明核對的要點）
=================================================================
* RMSE_H（= √(RMSE_E² + RMSE_N²)）、RMSE_V、n、平均誤差（偏差）、最大值。
* 與檢核點量測誤差合成（RSS）：RMSE_H,合成 = √(RMSE_H² + RMSE_H,量測²)，垂直同理；RMSE_H,量測 = √(mean σ_h²)。
* 檢核點精度至少是產品的 2 倍：給了目標精度就對目標比，否則對實測 RMSE 比（並寫明依據）。
* n ≥ 30 才符合 ASPRS 第 2 版的最少檢核點數；不足時附「小專案聲明」。
* 逐 UAV 區塊、逐跨機接縫（每對檢核點的 Z 誤差差 = 兩區塊的高程階差）的分項。

誤差的方向一律是「成果 − 實測」。
"""
from __future__ import annotations

import csv
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Optional, Sequence, Union

import numpy as np
from pyproj import CRS, Transformer
from pyproj.exceptions import CRSError

from survey import datum

FORMAT = "aeroplan-checkpoint-report/1"
VDATUMS = ("ellipsoidal", "EGM96", "TWVD2001")
ORTHO = ("EGM96", "TWVD2001")
ASPRS_MIN_CHECKPOINTS = 30
SURVEY_ACCURACY_FACTOR = 2.0          # ASPRS 第 2 版：檢核點精度至少是產品的 2 倍
PLANNED_MISMATCH_WARN_M = 10.0        # 實測離規劃位置超過這麼多 ⇒ 編號可能對錯


class CheckpointError(ValueError):
    """輸入不合理（訊息是給操作員看的繁中說明）。"""


@dataclass(frozen=True)
class SurveyedPoint:
    id: str
    lat: float
    lon: float
    h: float
    frame: str
    vdatum: str
    sigma_h_m: float
    sigma_v_m: float
    planned_lat: Optional[float] = None
    planned_lon: Optional[float] = None


@dataclass(frozen=True)
class MeasuredPoint:
    id: str
    lat: float
    lon: float
    frame: str
    h: Optional[float] = None
    vdatum: Optional[str] = None


def normalize_vdatum(v: str) -> str:
    """'ellipsoidal'（含「橢球高」）／'EGM96'／'TWVD2001'。只寫「正高」不夠 —— 要寫明是哪一個正高系統。"""
    s = str(v or "").strip()
    low = s.lower()
    if low in ("ellipsoidal", "ellipsoid", "h_ell", "橢球高"):
        return "ellipsoidal"
    for d in ORTHO:
        if low == d.lower():
            return d
    if low in ("orthometric", "正高", "amsl", "msl"):
        raise CheckpointError(f"高程基準 {s!r} 不夠明確：正高要寫明是 EGM96 還是 TWVD2001")
    raise CheckpointError(f"高程基準要是 {VDATUMS} 之一，收到 {s!r}")


def _num(row, key, line, what, errors, *, required=True) -> Optional[float]:
    s = (row.get(key) or "").strip()
    if not s:
        if required:
            errors.append(f"第 {line} 行（{row.get('id', '').strip() or '無編號'}）{what}沒填")
        return None
    try:
        v = float(s)
    except ValueError:
        errors.append(f"第 {line} 行（{row.get('id', '').strip()}）{what}不是數字：{s!r}")
        return None
    if not math.isfinite(v):
        errors.append(f"第 {line} 行（{row.get('id', '').strip()}）{what}不是有限數值：{s!r}")
        return None
    return v


def read_surveyed_csv(path, *, types: Sequence[str] = ("CHK",)) -> list:
    """實測檢核點。空白欄位**不跳過**：任何一列缺 lat/lon/h/σ 就整份拒絕並列出（避免靜默少算）。"""
    p = Path(path)
    with p.open("r", encoding="utf-8-sig", newline="") as fh:
        rd = csv.DictReader(fh)
        cols = rd.fieldnames or []
        need = ("id", "lat", "lon", "h", "frame", "vertical_datum", "sigma_h_m", "sigma_v_m")
        miss = [c for c in need if c not in cols]
        if miss:
            raise CheckpointError(f"{p.name} 缺欄位 {miss}（需要 {list(need)}）")
        rows = list(enumerate(rd, start=2))
    errors, out, seen = [], [], set()
    for line, r in rows:
        if "type" in cols and (r.get("type") or "").strip().upper() not in types:
            continue
        pid = (r.get("id") or "").strip()
        if not pid:
            errors.append(f"第 {line} 行沒有編號")
            continue
        if pid in seen:
            errors.append(f"第 {line} 行：編號 {pid} 重複")
            continue
        seen.add(pid)
        lat = _num(r, "lat", line, "緯度", errors)
        lon = _num(r, "lon", line, "經度", errors)
        h = _num(r, "h", line, "高", errors)
        sh = _num(r, "sigma_h_m", line, "σ 水平（ASPRS 第 2 版要合成量測誤差，不當 0）", errors)
        sv = _num(r, "sigma_v_m", line, "σ 垂直（ASPRS 第 2 版要合成量測誤差，不當 0）", errors)
        frame = (r.get("frame") or "").strip()
        if not frame:
            errors.append(f"第 {line} 行（{pid}）框架沒填")
        try:
            vd = normalize_vdatum(r.get("vertical_datum"))
        except CheckpointError as e:
            errors.append(f"第 {line} 行（{pid}）：{e}")
            vd = None
        for v, what in ((sh, "σ 水平"), (sv, "σ 垂直")):
            if v is not None and v < 0:
                errors.append(f"第 {line} 行（{pid}）{what}不能是負的")
        if lat is not None and not -90 <= lat <= 90 or lon is not None and not -180 <= lon <= 180:
            errors.append(f"第 {line} 行（{pid}）經緯度超出範圍")
        pl = _num(r, "planned_lat", line, "", [], required=False) if "planned_lat" in cols else None
        po = _num(r, "planned_lon", line, "", [], required=False) if "planned_lon" in cols else None
        if None in (lat, lon, h, sh, sv, vd) or not frame:
            continue
        out.append(SurveyedPoint(pid, lat, lon, h, frame, vd, sh, sv, pl, po))
    if errors:
        raise CheckpointError(f"{p.name}：" + "；".join(errors[:20]) + ("…" if len(errors) > 20 else ""))
    if not out:
        raise CheckpointError(f"{p.name} 沒有任何檢核點（type 欄只取 {list(types)}）")
    frames = sorted({s.frame for s in out})
    vds = sorted({s.vdatum for s in out})
    if len(frames) > 1 or len(vds) > 1:
        raise CheckpointError(f"{p.name} 混用框架 {frames}／高程基準 {vds}：一份檢核點只能有一種（不混用）")
    return out


def read_measured_csv(path) -> dict:
    """操作員在正射上量的成果座標 → {id: MeasuredPoint}。"""
    p = Path(path)
    with p.open("r", encoding="utf-8-sig", newline="") as fh:
        rd = csv.DictReader(fh)
        cols = rd.fieldnames or []
        miss = [c for c in ("id", "lat", "lon", "frame") if c not in cols]
        if miss:
            raise CheckpointError(f"{p.name} 缺欄位 {miss}（需要 id, lat, lon, frame；可加 h, vertical_datum）")
        rows = list(enumerate(rd, start=2))
    out, errors = {}, []
    for line, r in rows:
        pid = (r.get("id") or "").strip()
        if not pid:
            continue
        if pid in out:
            errors.append(f"第 {line} 行：編號 {pid} 重複")
            continue
        lat = _num(r, "lat", line, "緯度", errors)
        lon = _num(r, "lon", line, "經度", errors)
        h = _num(r, "h", line, "高", errors, required=False) if "h" in cols else None
        vd = None
        if h is not None:
            try:
                vd = normalize_vdatum(r.get("vertical_datum"))
            except CheckpointError as e:
                errors.append(f"第 {line} 行（{pid}）：{e}")
        frame = (r.get("frame") or "").strip()
        if not frame:
            errors.append(f"第 {line} 行（{pid}）框架沒填")
        if lat is None or lon is None or not frame:
            continue
        out[pid] = MeasuredPoint(pid, lat, lon, frame, h, vd)
    if errors:
        raise CheckpointError(f"{p.name}：" + "；".join(errors[:20]))
    return out


# ─────────────────────────────────────────────────────────────────────────────
# DSM
# ─────────────────────────────────────────────────────────────────────────────

def frame_to_crs(frame: str, frame_crs: Optional[str] = None) -> CRS:
    """檢核點框架名 → pyproj CRS。框架名本身不是 PROJ 可解析的（例如 e-GNSS[2025]）就必須明確給 ``frame_crs``。"""
    src = frame_crs or frame
    try:
        crs = CRS.from_user_input(src)
    except CRSError:
        raise CheckpointError(f"檢核點框架 {frame!r} 不是 PROJ 可解析的座標系：請明確指定它對應的經緯度座標系"
                              "（例如 +proj=longlat +ellps=GRS80 +no_defs）—— 不會把它當 WGS84") from None
    if not crs.is_geographic:
        raise CheckpointError(f"檢核點的經緯度要對應經緯度座標系，收到 {crs.to_string()}")
    return crs


def raster_vertical_datum(crs_wkt: str) -> Optional[str]:
    """DSM 的 CRS 若是含垂直分量的複合 CRS，回傳我們的高程基準名；沒有垂直分量 ⇒ None（只能靠宣告）。"""
    try:
        crs = CRS.from_user_input(crs_wkt)
    except CRSError:
        return None
    subs = crs.sub_crs_list or []
    vert = [c for c in subs if c.is_vertical]
    if not vert:
        return None
    name = (vert[0].name or "").upper()
    if "EGM96" in name or "EGM 96" in name:
        return "EGM96"
    if "TWVD" in name or "TAIWAN VERTICAL" in name:
        return "TWVD2001"
    if "ELLIPSOID" in name:
        return "ellipsoidal"
    return f"other:{vert[0].name}"


def sample_dsm(dsm_path, lonlat: Sequence, *, src_crs: CRS) -> dict:
    """在 (經度, 緯度) 上雙線性取 DSM 第 1 波段。回傳 {'z': [值或 None], 'reason': [...], 'crs', 'pixel_m', ...}。

    任一需要的像素是 nodata／遮罩 ⇒ 該點 None（理由寫明），不拿鄰近值湊。
    """
    import rasterio
    from rasterio.windows import Window
    p = Path(dsm_path)
    if not p.is_file():
        raise CheckpointError(f"DSM 不存在：{p}")
    with rasterio.open(p) as ds:
        if ds.crs is None:
            raise CheckpointError(f"{p.name} 沒有座標系（CRS）：不能在實測 XY 上取值")
        dst = CRS.from_user_input(ds.crs.to_wkt())
        t = Transformer.from_crs(src_crs, dst, always_xy=True)
        inv = ~ds.transform
        zs, why = [], []
        for lon, lat in lonlat:
            x, y = t.transform(float(lon), float(lat))
            col, row = inv * (x, y)
            c, r = col - 0.5, row - 0.5                   # 以像素中心內插
            c0, r0 = int(math.floor(c)), int(math.floor(r))
            if c0 < 0 or r0 < 0 or c0 + 1 >= ds.width or r0 + 1 >= ds.height:
                zs.append(None)
                why.append("在 DSM 範圍外（或貼邊、湊不到 2×2 像素）")
                continue
            blk = ds.read(1, window=Window(c0, r0, 2, 2), masked=True)
            if np.ma.is_masked(blk) and np.ma.getmaskarray(blk).any():
                zs.append(None)
                why.append("DSM 在這裡沒有資料（nodata）")
                continue
            v = np.asarray(blk, float)
            if not np.all(np.isfinite(v)):
                zs.append(None)
                why.append("DSM 在這裡是 NaN")
                continue
            fc, fr = c - c0, r - r0
            z = (v[0, 0] * (1 - fc) * (1 - fr) + v[0, 1] * fc * (1 - fr) + v[1, 0] * (1 - fc) * fr
                 + v[1, 1] * fc * fr)
            zs.append(float(z))
            why.append("")
        res = abs(ds.transform.a), abs(ds.transform.e)
        return {"z": zs, "reason": why, "crs": dst.to_string(), "crs_wkt": ds.crs.to_wkt(),
                "pixel": [float(res[0]), float(res[1])], "pixel_units": (dst.axis_info[0].unit_name
                                                                          if dst.axis_info else ""),
                "vertical_in_crs": raster_vertical_datum(ds.crs.to_wkt()), "path": str(p)}


# ─────────────────────────────────────────────────────────────────────────────
# 版面（區塊、接縫對）
# ─────────────────────────────────────────────────────────────────────────────

def layout_index(layout) -> dict:
    """佈設（gcp_plan.LayoutPlan、read_layout_csv 的列，或 None）→ {id: {type, block, purpose, seam, pair, side, lat, lon}}。"""
    if layout is None:
        return {}
    pts = getattr(layout, "points", None)
    out = {}
    if pts is not None:
        for p in pts:
            m = re.search(r"side=(\S+)", p.note or "")
            out[p.id] = {"type": p.type, "block": p.block, "purpose": p.purpose, "seam": p.seam, "pair": p.pair,
                         "side": m.group(1) if m else "", "lat": p.lat, "lon": p.lon}
        return out
    for r in layout:
        m = re.search(r"side=(\S+)", r.get("note") or "")
        out[str(r["id"])] = {"type": r.get("type", ""), "block": r.get("block", ""), "purpose": r.get("purpose", ""),
                             "seam": r.get("seam", ""), "pair": int(r.get("pair") or 0),
                             "side": m.group(1) if m else "", "lat": float(r["lat"]), "lon": float(r["lon"])}
    return out


# ─────────────────────────────────────────────────────────────────────────────
# 統計
# ─────────────────────────────────────────────────────────────────────────────

def _stats(v: Sequence[float]) -> dict:
    a = np.asarray([x for x in v if x is not None], float)
    n = int(a.size)
    if n == 0:
        return {"n": 0, "mean": None, "rmse": None, "std": None, "max_abs": None}
    return {"n": n, "mean": float(a.mean()), "rmse": float(math.sqrt(float(np.mean(a * a)))),
            "std": float(a.std(ddof=1)) if n > 1 else None, "max_abs": float(np.max(np.abs(a)))}


def _rss(a: Optional[float], b: Optional[float]) -> Optional[float]:
    if a is None or b is None:
        return None
    return float(math.hypot(a, b))


def _undulation_at(undulation, lat, lon, ortho: str) -> tuple:
    if isinstance(undulation, datum.GeoidModel):
        if undulation.datum != ortho:
            raise CheckpointError(f"大地起伏模型是 {undulation.datum}，但要換算的正高是 {ortho}")
        return undulation.n_at(lat, lon), f"GeoidModel({undulation.datum}; {undulation.source})"
    n = float(undulation)
    if not math.isfinite(n):
        raise CheckpointError(f"大地起伏 N 不是有限數值：{undulation!r}")
    return n, f"使用者給的常數 N = {n:g} m（{ortho}）"


def accuracy_report(surveyed: Sequence[SurveyedPoint], *, measured: Optional[Mapping] = None, dsm_path=None,
                    dsm_vertical: Optional[str] = None, undulation: Union[None, float, datum.GeoidModel] = None,
                    frame_crs: Optional[str] = None, layout=None, gcp_names: Sequence[str] = (),
                    target_rmse_h_m: Optional[float] = None, target_rmse_v_m: Optional[float] = None,
                    min_checkpoints: int = ASPRS_MIN_CHECKPOINTS) -> dict:
    """實測檢核點 ＋ 成果 → 精度報告（dict，可直接 json.dumps）。"""
    pts = list(surveyed)
    if not pts:
        raise CheckpointError("沒有實測檢核點")
    frames = {p.frame for p in pts}
    vds = {p.vdatum for p in pts}
    if len(frames) > 1 or len(vds) > 1:
        raise CheckpointError(f"檢核點混用框架 {sorted(frames)}／高程基準 {sorted(vds)}")
    frame, vd = pts[0].frame, pts[0].vdatum
    if measured is None and dsm_path is None:
        raise CheckpointError("沒有成果可比：至少要給正射量測 CSV 或 DSM")
    idx = layout_index(layout)
    warnings, notes = [], []
    clash = sorted({p.id for p in pts} & set(gcp_names))
    if clash:
        raise CheckpointError(f"檢核點 {clash} 也出現在 GCP 檔：檢核點不能拿來當控制點，精度報告沒有意義")
    as_gcp = sorted(p.id for p in pts if idx.get(p.id, {}).get("type") == "GCP")
    if as_gcp:
        raise CheckpointError(f"{as_gcp} 在佈設裡是 GCP，不能當檢核點")
    for p in pts:
        ref = idx.get(p.id)
        plat = p.planned_lat if p.planned_lat is not None else (ref or {}).get("lat")
        plon = p.planned_lon if p.planned_lon is not None else (ref or {}).get("lon")
        if plat is not None and plon is not None:
            e, n = datum.local_en_m(plat, plon, p.lat, p.lon)
            d = float(math.hypot(e, n))
            if d > PLANNED_MISMATCH_WARN_M:
                warnings.append(f"{p.id} 的實測位置離規劃位置 {d:.1f} m（編號對錯？）")

    rows = {p.id: {"id": p.id, "block": idx.get(p.id, {}).get("block", ""), "dE": None, "dN": None, "dH": None,
                   "dZ": None, "sigma_h_m": p.sigma_h_m, "sigma_v_m": p.sigma_v_m, "h_source": None,
                   "v_source": None, "reason": ""} for p in pts}
    eff = {"format": FORMAT, "frame": frame, "vertical_datum": vd, "frame_crs": frame_crs, "n_surveyed": len(pts),
           "target_rmse_h_m": target_rmse_h_m, "target_rmse_v_m": target_rmse_v_m,
           "min_checkpoints": int(min_checkpoints), "gcp_names_checked": sorted(gcp_names),
           "layout": bool(idx), "measured": measured is not None, "dsm": None}

    # 平面（與高程，若量測 CSV 有高）
    if measured is not None:
        mframes = {m.frame for m in measured.values()}
        if mframes and mframes != {frame}:
            raise CheckpointError(f"正射量測的框架 {sorted(mframes)} 與檢核點的 {frame!r} 不同：本工具不做框架轉換")
        unknown = sorted(set(measured) - {p.id for p in pts})
        if unknown:
            warnings.append(f"正射量測 CSV 有不在實測檢核點裡的編號 {unknown}（不採用）")
        for p in pts:
            m = measured.get(p.id)
            if m is None:
                rows[p.id]["reason"] += "正射上沒有量；"
                continue
            e, n = datum.local_en_m(p.lat, p.lon, m.lat, m.lon)
            rows[p.id].update(dE=float(e), dN=float(n), dH=float(math.hypot(e, n)), h_source="正射量測")
            if m.h is not None and dsm_path is None:
                if m.vdatum != vd:
                    raise CheckpointError(f"{p.id}：正射量測的高程基準 {m.vdatum} 與檢核點的 {vd} 不同（不混用）")
                rows[p.id].update(dZ=float(m.h - p.h), v_source="正射量測 CSV 的 h")
    # 高程（DSM）
    if dsm_path is not None:
        if dsm_vertical is None:
            raise CheckpointError("請明確宣告 DSM 的高程基準（橢球高／EGM96／TWVD2001）—— 不猜")
        dv = normalize_vdatum(dsm_vertical)
        src = frame_to_crs(frame, frame_crs)
        smp = sample_dsm(dsm_path, [(p.lon, p.lat) for p in pts], src_crs=src)
        vin = smp["vertical_in_crs"]
        if vin is not None and vin != dv:
            raise CheckpointError(f"DSM 的 CRS 寫的垂直基準是 {vin}，與宣告的 {dv} 不同")
        conv = None
        if dv != vd:
            if undulation is None:
                raise CheckpointError(f"DSM 是 {dv}、檢核點是 {vd}：高程基準不同，沒有明確給大地起伏 N ⇒ 拒絕（不混用、不當 0）")
            if dv in ORTHO and vd in ORTHO:
                raise CheckpointError(f"DSM 是 {dv}、檢核點是 {vd}：兩種正高之間的換算不支援")
            conv = "dsm_ell_to_ortho" if dv == "ellipsoidal" else "dsm_ortho_to_ell"
        n_used = []
        for p, z, why in zip(pts, smp["z"], smp["reason"]):
            if z is None:
                rows[p.id]["reason"] += f"DSM：{why}；"
                continue
            if conv is None:
                zc = z
            else:
                ortho = vd if conv == "dsm_ell_to_ortho" else dv
                n, nsrc = _undulation_at(undulation, p.lat, p.lon, ortho)
                n_used.append(nsrc)
                zc = z - n if conv == "dsm_ell_to_ortho" else z + n
            rows[p.id].update(dZ=float(zc - p.h), v_source="DSM 雙線性")
        eff["dsm"] = {"path": smp["path"], "crs": smp["crs"], "pixel": smp["pixel"], "pixel_units": smp["pixel_units"],
                      "declared_vertical": dv, "vertical_in_crs": vin, "conversion": conv,
                      "undulation": sorted(set(n_used))[:3] if n_used else None, "points_crs": src.to_string()}
        if smp["pixel_units"] and smp["pixel_units"].startswith("metre") and max(smp["pixel"]) > 0.5:
            notes.append(f"DSM 像素 {max(smp['pixel']):.2f} m 偏粗：檢核點高程誤差會含內插誤差")

    table = [rows[p.id] for p in pts]
    hor = [r for r in table if r["dE"] is not None]
    ver = [r for r in table if r["dZ"] is not None]
    se, sn = _stats([r["dE"] for r in hor]), _stats([r["dN"] for r in hor])
    sv = _stats([r["dZ"] for r in ver])
    rmse_h = _rss(se["rmse"], sn["rmse"])
    surv_h = float(math.sqrt(np.mean([r["sigma_h_m"] ** 2 for r in hor]))) if hor else None
    surv_v = float(math.sqrt(np.mean([r["sigma_v_m"] ** 2 for r in ver]))) if ver else None
    horizontal = {"n": len(hor), "rmse_e_m": se["rmse"], "rmse_n_m": sn["rmse"], "rmse_h_m": rmse_h,
                  "mean_e_m": se["mean"], "mean_n_m": sn["mean"],
                  "max_h_m": max((r["dH"] for r in hor), default=None),
                  "survey_rmse_h_m": surv_h, "combined_rmse_h_m": _rss(rmse_h, surv_h)}
    vertical = {"n": len(ver), "rmse_v_m": sv["rmse"], "mean_v_m": sv["mean"], "std_v_m": sv["std"],
                "max_abs_v_m": sv["max_abs"], "survey_rmse_v_m": surv_v,
                "combined_rmse_v_m": _rss(sv["rmse"], surv_v)}

    def ratio_check(survey, product_meas, target):
        if survey is None or (product_meas is None and target is None):
            return None
        basis = "目標精度" if target is not None else "實測 RMSE（沒有給目標精度）"
        prod = float(target) if target is not None else float(product_meas)
        ok = survey <= prod / SURVEY_ACCURACY_FACTOR if survey > 0 else True
        return {"survey_rmse_m": survey, "product_rmse_m": prod, "basis": basis,
                "ratio": (prod / survey) if survey > 0 else None, "ok": bool(ok)}

    n_min = min(len(hor), len(ver)) if hor and ver else (len(hor) or len(ver))
    meets = n_min >= int(min_checkpoints)
    statement = (f"檢核點 n = {n_min} ≥ {min_checkpoints}，符合 ASPRS 第 2 版的最少檢核點數。" if meets else
                 f"小專案聲明：本報告只有 n = {n_min} 個檢核點，少於 ASPRS 第 2 版要求的 {min_checkpoints} 個；"
                 "精度數字只代表本次作業範圍，引用時必須連同本聲明。")
    asprs = {"n_h": len(hor), "n_v": len(ver), "min_checkpoints": int(min_checkpoints), "meets_min": bool(meets),
             "statement": statement,
             "survey_vs_product_h": ratio_check(surv_h, rmse_h, target_rmse_h_m),
             "survey_vs_product_v": ratio_check(surv_v, sv["rmse"], target_rmse_v_m),
             "meets_target_h": (None if target_rmse_h_m is None or horizontal["combined_rmse_h_m"] is None
                                else horizontal["combined_rmse_h_m"] <= target_rmse_h_m),
             "meets_target_v": (None if target_rmse_v_m is None or vertical["combined_rmse_v_m"] is None
                                else vertical["combined_rmse_v_m"] <= target_rmse_v_m)}
    for k in ("survey_vs_product_h", "survey_vs_product_v"):
        c = asprs[k]
        if c is not None and not c["ok"]:
            warnings.append(f"{'水平' if k.endswith('h') else '垂直'}：檢核點量測 RMSE {c['survey_rmse_m'] * 100:.1f} cm "
                            f"沒有比{c['basis']} {c['product_rmse_m'] * 100:.1f} cm 好 2 倍以上（ASPRS 第 2 版）")

    # 逐區塊
    per_block = {}
    for r in table:
        b = r["block"] or "（無區塊資訊）"
        per_block.setdefault(b, []).append(r)
    blocks = {}
    for b, rs in sorted(per_block.items()):
        e_, n_ = _stats([x["dE"] for x in rs if x["dE"] is not None]), _stats([x["dN"] for x in rs if x["dN"] is not None])
        z_ = _stats([x["dZ"] for x in rs if x["dZ"] is not None])
        blocks[b] = {"n": len(rs), "n_v": z_["n"], "rmse_v_m": z_["rmse"], "mean_v_m": z_["mean"],
                     "std_v_m": z_["std"], "n_h": e_["n"], "rmse_h_m": _rss(e_["rmse"], n_["rmse"])}
    # 逐接縫
    seams = {}
    for pid, info in idx.items():
        if info.get("purpose") == "seam_pair" and info.get("seam") and pid in rows:
            seams.setdefault(info["seam"], {}).setdefault(info["pair"], {})[info.get("side") or info["block"]] = pid
    seam_out = []
    for lab in sorted(seams):
        a, b = lab.split("|", 1)
        pairs = []
        for k in sorted(seams[lab]):
            d = seams[lab][k]
            ia, ib = d.get(a), d.get(b)
            za = rows[ia]["dZ"] if ia else None
            zb = rows[ib]["dZ"] if ib else None
            pairs.append({"pair": k, "id_a": ia, "id_b": ib, "dz_a_m": za, "dz_b_m": zb,
                          "step_m": None if za is None or zb is None else float(za - zb)})
        steps = [p["step_m"] for p in pairs if p["step_m"] is not None]
        ba, bb = blocks.get(a), blocks.get(b)
        blk_step = blk_se = None
        if ba and bb and ba["mean_v_m"] is not None and bb["mean_v_m"] is not None:
            blk_step = float(ba["mean_v_m"] - bb["mean_v_m"])
            if ba["std_v_m"] is not None and bb["std_v_m"] is not None:
                blk_se = float(math.sqrt(ba["std_v_m"] ** 2 / ba["n_v"] + bb["std_v_m"] ** 2 / bb["n_v"]))
        seam_out.append({"seam": lab, "block_a": a, "block_b": b, "pairs": pairs,
                         "mean_pair_step_m": float(np.mean(steps)) if steps else None,
                         "block_mean_step_m": blk_step, "block_mean_step_se_m": blk_se})
    if idx and not seam_out:
        notes.append("佈設裡沒有接縫檢核點對（沒有 UAV 區塊資訊？）⇒ 沒有跨機高程階差")
    missing = [r["id"] for r in table if r["dE"] is None and r["dZ"] is None]
    if missing:
        warnings.append(f"{len(missing)} 個檢核點沒有成果值（{', '.join(missing[:10])}）：不計入 RMSE")
    return {"format": FORMAT, "effective": eff, "points": table, "horizontal": horizontal, "vertical": vertical,
            "asprs": asprs, "per_block": blocks, "per_seam": seam_out, "warnings": warnings, "notes": notes}


def _cm(v) -> str:
    return "—" if v is None else f"{v * 100:.1f} cm"


def report_lines(rep: dict) -> list:
    """給對話框與 Markdown 用的文字。"""
    h, v, a = rep["horizontal"], rep["vertical"], rep["asprs"]
    L = ["檢核點精度報告（ASPRS 第 2 版的報告方式；誤差 = 成果 − 實測）",
         f"框架 {rep['effective']['frame']}，高程基準 {rep['effective']['vertical_datum']}",
         f"水平 n = {h['n']}：RMSE_E {_cm(h['rmse_e_m'])}、RMSE_N {_cm(h['rmse_n_m'])}、RMSE_H {_cm(h['rmse_h_m'])}；"
         f"平均 E {_cm(h['mean_e_m'])}、N {_cm(h['mean_n_m'])}；最大 {_cm(h['max_h_m'])}；"
         f"量測 RMSE {_cm(h['survey_rmse_h_m'])} ⇒ 合成 RMSE_H {_cm(h['combined_rmse_h_m'])}",
         f"垂直 n = {v['n']}：RMSE_V {_cm(v['rmse_v_m'])}、平均（偏差）{_cm(v['mean_v_m'])}、"
         f"標準差 {_cm(v['std_v_m'])}、最大 {_cm(v['max_abs_v_m'])}；量測 RMSE {_cm(v['survey_rmse_v_m'])} "
         f"⇒ 合成 RMSE_V {_cm(v['combined_rmse_v_m'])}",
         a["statement"]]
    for k, name in (("survey_vs_product_h", "水平"), ("survey_vs_product_v", "垂直")):
        c = a[k]
        if c is not None:
            L.append(f"{name}檢核點精度 vs {c['basis']}：{_cm(c['survey_rmse_m'])} vs {_cm(c['product_rmse_m'])}"
                     f"（{'✔ 至少好 2 倍' if c['ok'] else '✘ 不到 2 倍'}）")
    for b, s in rep["per_block"].items():
        L.append(f"區塊 {b}：n {s['n']}；RMSE_V {_cm(s['rmse_v_m'])}、平均 {_cm(s['mean_v_m'])}；RMSE_H {_cm(s['rmse_h_m'])}")
    for s in rep["per_seam"]:
        L.append(f"接縫 {s['seam']}：每對階差平均 {_cm(s['mean_pair_step_m'])}；區塊平均差 "
                 f"{_cm(s['block_mean_step_m'])}" + (f"（±{_cm(s['block_mean_step_se_m'])}）"
                                                    if s["block_mean_step_se_m"] is not None else ""))
    L += [f"⚠ {w}" for w in rep["warnings"]] + [f"・{n}" for n in rep["notes"]]
    return L


def write_report(rep: dict, out_dir, *, stem: str = "checkpoint_report") -> dict:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    jp, mp = out / f"{stem}.json", out / f"{stem}.md"
    jp.write_text(json.dumps(rep, ensure_ascii=False, indent=1, sort_keys=True) + "\n", encoding="utf-8",
                  newline="\n")
    lines = ["# " + report_lines(rep)[0], ""] + [f"- {x}" for x in report_lines(rep)[1:]] + [
        "", "| 編號 | 區塊 | dE (cm) | dN (cm) | dH (cm) | dZ (cm) | σh (cm) | σv (cm) | 備註 |",
        "|---|---|---|---|---|---|---|---|---|"]
    for r in rep["points"]:
        f = (lambda x: "—" if x is None else f"{x * 100:.1f}")
        lines.append(f"| {r['id']} | {r['block'] or '—'} | {f(r['dE'])} | {f(r['dN'])} | {f(r['dH'])} | {f(r['dZ'])} | "
                     f"{f(r['sigma_h_m'])} | {f(r['sigma_v_m'])} | {r['reason'] or ''} |")
    mp.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")
    return {"json": jp, "md": mp}


def checkpoint_ids(path) -> list:
    """實測 CSV 的檢核點編號（不驗值；給 gcp_list 檢查「檢核點不能當控制點」用）。"""
    with Path(path).open("r", encoding="utf-8-sig", newline="") as fh:
        rd = csv.DictReader(fh)
        cols = rd.fieldnames or []
        return [r["id"].strip() for r in rd if r.get("id") and ("type" not in cols
                                                               or (r.get("type") or "").strip().upper() == "CHK")]


__all__ = ["CheckpointError", "SurveyedPoint", "MeasuredPoint", "read_surveyed_csv", "read_measured_csv",
           "sample_dsm", "accuracy_report", "report_lines", "write_report", "layout_index", "frame_to_crs",
           "raster_vertical_datum", "normalize_vdatum", "checkpoint_ids"]
