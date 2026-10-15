"""I1 交付物：photos.csv（F2）＋ GeoJSON 點（含航向）＋ 說明座標／高程／時間系統的 JSON。

F2 欄位定義（其他套件依此讀寫，順序固定）：

* ``yaw_deg``：相機／機身航向，自**真北**順時針（不是航跡）；``cog_deg``：對地航跡。
* ``h_ell_m``：橢球高；``h_orth_m``：``crs_v`` 所指正高系統的高度（沒有大地起伏模型時留空，
  **不拿 N=0 算**）。``crs_v`` ∈ {ellipsoidal, EGM96, TWVD2001, unknown}：正高欄有值時是它的
  基準，只剩橢球高時為 ellipsoidal，兩者皆無為 unknown。
* ``source`` ∈ {fc_interp, ppk, sfm}；``flags`` 以 ';' 串接。
* 時間：``t_gpst_s`` = GPS 週×604800 + 週內秒（無閏秒）；UTC = GPST − 18 s；
  ``t_fc_us`` = 飛控自開機的 µs（一個架次只開機一次）。

GeoJSON（RFC 7946 §4）座標第三維**必須**是 WGS84 橢球高 ⇒ 只在有 h_ell_m 時寫三維，否則二維；
正高與其基準放在 properties。輸出可重現：列依 (uav, t_gpst_s/t_fc_us, seq) 排序，不寫入當下時間。
"""

from __future__ import annotations

import csv
import json
from datetime import timezone
from pathlib import Path
from typing import Optional

from survey import datum

F2_COLUMNS = ("image_id", "file", "uav", "strip", "seq", "t_fc_us", "t_gpst_s", "lat_deg", "lon_deg",
              "h_ell_m", "h_orth_m", "yaw_deg", "pitch_deg", "roll_deg", "cog_deg", "speed_n_mps",
              "speed_e_mps", "speed_d_mps", "sigma_h_m", "sigma_v_m", "source", "crs_h", "crs_v", "flags")
SOURCES = ("fc_interp", "ppk", "sfm")
_INT_COLS = ("strip", "seq", "t_fc_us")
_STR_COLS = ("image_id", "file", "uav", "source", "crs_h", "crs_v", "flags")
_DECIMALS = {"t_gpst_s": 6, "lat_deg": 9, "lon_deg": 9, "h_ell_m": 3, "h_orth_m": 3, "yaw_deg": 3,
             "pitch_deg": 3, "roll_deg": 3, "cog_deg": 3, "speed_n_mps": 3, "speed_e_mps": 3,
             "speed_d_mps": 3, "sigma_h_m": 3, "sigma_v_m": 3}
I1_FORMAT = "aeroplan-i1/1"


def _validate(rows):
    for r in rows:
        extra = set(r) - set(F2_COLUMNS)
        if extra:
            raise ValueError(f"{r.get('image_id')}: 不在 F2 的欄位 {sorted(extra)}")
        if r.get("source") not in SOURCES:
            raise ValueError(f"{r.get('image_id')}: source={r.get('source')!r} 不在 {SOURCES}")
        if r.get("crs_v") not in datum.VERTICAL_DATUMS:
            raise ValueError(f"{r.get('image_id')}: crs_v={r.get('crs_v')!r} 不在 {datum.VERTICAL_DATUMS}")
        if r.get("h_orth_m") is not None and r["crs_v"] not in datum.ORTHOMETRIC_DATUMS:
            raise ValueError(f"{r.get('image_id')}: 有 h_orth_m 但 crs_v={r['crs_v']!r} 不是正高基準")
    ids = [r["image_id"] for r in rows]
    if len(set(ids)) != len(ids):
        raise ValueError("image_id 重複")


def _sort_key(r):
    t = r.get("t_gpst_s")
    return (str(r.get("uav")), t if t is not None else float("inf"), r.get("t_fc_us") or 0, r.get("seq") or 0)


def _fmt(col, v):
    if v is None or (isinstance(v, str) and v == ""):     # survey.ppk.outputs 以 "" 表示空欄（strip 等）
        return ""
    if col in _INT_COLS:
        return str(int(v))
    if col in _DECIMALS:
        return f"{float(v):.{_DECIMALS[col]}f}"
    return str(v)


def write_photos_csv(path, rows) -> Path:
    rows = sorted(rows, key=_sort_key)
    _validate(rows)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as fh:
        w = csv.writer(fh, lineterminator="\n")
        w.writerow(F2_COLUMNS)
        for r in rows:
            w.writerow([_fmt(c, r.get(c)) for c in F2_COLUMNS])
    return path


def read_photos_csv(path) -> list:
    """讀回 F2；空欄為 None，數值欄轉型。表頭不是 F2 就拒絕。"""
    with Path(path).open("r", encoding="utf-8", newline="") as fh:
        rd = csv.reader(fh)
        header = next(rd)
        if tuple(header) != F2_COLUMNS:
            raise ValueError(f"{path}: 表頭不是 F2：{header}")
        out = []
        for line in rd:
            r = {}
            for c, v in zip(F2_COLUMNS, line):
                if v == "":
                    r[c] = None if c != "flags" else ""
                elif c in _INT_COLS:
                    r[c] = int(v)
                elif c in _STR_COLS:
                    r[c] = v
                else:
                    r[c] = float(v)
            out.append(r)
    return out


def to_geojson(rows) -> dict:
    feats = []
    for r in sorted(rows, key=_sort_key):
        if r.get("lat_deg") is None or r.get("lon_deg") is None:
            continue
        coords = [round(r["lon_deg"], 9), round(r["lat_deg"], 9)]
        if r.get("h_ell_m") is not None:
            coords.append(round(r["h_ell_m"], 3))
        props = {c: r.get(c) for c in F2_COLUMNS if c not in ("lat_deg", "lon_deg")}
        props["heading_deg"] = r.get("yaw_deg")
        props["t_utc"] = (datum.gpst_to_utc(r["t_gpst_s"]).astimezone(timezone.utc).isoformat(timespec="milliseconds")
                          if r.get("t_gpst_s") is not None else None)
        feats.append({"type": "Feature", "id": r["image_id"],
                      "geometry": {"type": "Point", "coordinates": coords}, "properties": props})
    return {"type": "FeatureCollection", "features": feats}


def sidecar(rows, meta: Optional[dict] = None) -> dict:
    """描述交付物的座標、高程、時間系統與來源。``meta`` 由呼叫端提供（位姿的 effective、擬合摘要…）。"""
    meta = dict(meta or {})
    times = sorted(r["t_gpst_s"] for r in rows if r.get("t_gpst_s") is not None)
    epoch = datum.gpst_to_utc(times[len(times) // 2]).isoformat(timespec="seconds") if times else None
    return {
        "format": I1_FORMAT,
        "n_photos": len(rows),
        "columns": list(F2_COLUMNS),
        "crs_h": sorted({r.get("crs_h") for r in rows if r.get("crs_h")}),
        "crs_v": sorted({r.get("crs_v") for r in rows if r.get("crs_v")}),
        "frame_epoch": meta.pop("frame_epoch", None),
        "observation_epoch_utc": epoch,
        "time_system": {
            "t_gpst_s": "GPS week*604800 + time-of-week (s), no leap seconds",
            "gpst_minus_utc_s": datum.GPST_MINUS_UTC_S,
            "t_fc_us": "microseconds since flight-controller boot (one boot per sortie)",
        },
        "position": "camera centre (exposure station), lever arm applied",
        "heading": "yaw_deg = body/camera heading clockwise from true north; cog_deg = course over ground",
        "heights": "h_ell_m = ellipsoidal; h_orth_m = orthometric in crs_v (empty when no geoid model was given)",
        "geojson_z": "WGS84 ellipsoidal height when h_ell_m is known (RFC 7946 §4), otherwise 2D",
        "sources": {s: sum(1 for r in rows if r.get("source") == s) for s in SOURCES},
        "uavs": sorted({str(r.get("uav")) for r in rows}),
        "meta": meta,
    }


def write_i1_deliverable(out_dir, rows, meta: Optional[dict] = None) -> dict:
    """寫出 photos.csv、photos.geojson、photos.meta.json，回傳三個路徑。"""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    csv_p = write_photos_csv(out / "photos.csv", rows)
    gj_p = out / "photos.geojson"
    gj_p.write_text(json.dumps(to_geojson(rows), ensure_ascii=False, indent=1, sort_keys=True), encoding="utf-8",
                    newline="\n")
    meta_p = out / "photos.meta.json"
    meta_p.write_text(json.dumps(sidecar(rows, meta), ensure_ascii=False, indent=2, sort_keys=True, default=str),
                      encoding="utf-8", newline="\n")
    return {"csv": csv_p, "geojson": gj_p, "meta": meta_p}
