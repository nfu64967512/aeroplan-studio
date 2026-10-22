"""ODM／ODX 的影像地理定位檔 geo.txt。

格式（docs.opendronemap.org/geo/，ODM 3.5.4 文件）::

    <projection>                      # 第一行：PROJ 字串、EPSG:xxxx 或 "WGS84 UTM 51N"
    image_name geo_x geo_y [geo_z] [yaw] [pitch] [roll] [horz_accuracy] [vert_accuracy] [extras...]

對照 ODM 原始碼（OpenDroneMap/ODM master，2026-10-14 取得）：

* ``opendm/geo.py`` GeoFile：``x, y = parts[1:3]`` 再 ``location.transform3(srs, EPSG:4326, x, y, z)``
  ⇒ EPSG:4326 時 geo_x = **經度**、geo_y = 緯度；第 5–7 欄 yaw/pitch/roll（任一為 NaN 就三個都當
  未提供），第 8–9 欄水平／垂直精度（m）。
* ``opendm/photo.py`` ``update_with_geo_entry`` 用精度覆寫 GPS 先驗權重（gps_xy_stddev／
  gps_z_stddev → OpenSfM 的 latitude_std/altitude_std），沒給時退回 --gps-accuracy（預設 3 m）。
* 同檔 ``compute_opk``（:916-967）把 yaw/pitch/roll 當 Bäumker 的 NED 機體角（航向自北順時針、
  抬頭為正、右翼下為正），並假設「相機正下視、影像上緣朝飛行方向」。我們寫的是飛控 ATT 的
  機體姿態（同一慣例）；相機安裝方向不同時要另外處理（本模組不猜，``include_angles=False``
  會寫 ``nan nan nan``，ODM 視為未提供但仍讀精度欄）。
* 影像以檔名比對（``GeoFile.entries[filename]``）⇒ 名稱必須等於上傳／匯出的檔名，
  見 :func:`export_filename`（EXIF 匯出也用同一個名字）。

精度欄的單位與意義（2026-10-20 審查，依 R1 §6.2／§8.2：ODX v3.8.4 modules/photo.py、OpenSfM stats）：
ODX 把第 8 欄存成 ``gps_xy_stddev``，OpenSfM 拿它**同時**當 x 與 y 各自的 σ（單軸）。我們列裡的 ``sigma_h_m``
是**二維** HRMS（PPK：hypot(sdn, sde)；飛控：GPA.HAcc；見 survey/checkpoints.py 開頭）⇒ 直接寫進去會讓每一軸的
σ 大 √2、先驗權重低估 2 倍。所以第 8 欄寫 ``sigma_h_m / √2``（假設東、北等向），第 9 欄照寫 ``sigma_v_m``（本來就是
單軸）。這個換算記在 :data:`ACCURACY_SEMANTICS`，處理時寫進 qa.json 的 effective.geo_txt_accuracy、送 ODM 時進任務紀錄。
"""

from __future__ import annotations

import logging
import math
from pathlib import Path, PurePosixPath
from typing import Optional

_log = logging.getLogger(__name__)

_HEIGHT_KEY = {"ellipsoidal": "h_ell_m", "orthometric": "h_orth_m"}

#: geo.txt 精度欄寫的是什麼（有效值契約：換算要記錄）
ACCURACY_SEMANTICS = {
    "horizontal": "sigma_1d_from_hrms", "vertical": "sigma_v_1d", "recorded": True,
    "horizontal_factor": 1.0 / math.sqrt(2.0),
    "note": "第 8 欄＝單軸 σ＝sigma_h_m（二維 HRMS）/ √2（假設東北等向；ODX 當成 x、y 各自的 σ，R1 §6.2）；"
            "第 9 欄＝sigma_v_m（垂直單軸 σ）",
}


def _sigma_text(v, what: str, name: str) -> str:
    """精度欄的文字：必須是有限的正數（0／負數／NaN 會讓 ODX 拿到零或 NaN 的 σ）；太小就多印幾位，不印成 0.000。"""
    x = float(v)
    if not math.isfinite(x) or x <= 0.0:
        raise ValueError(f"{name} 的{what}精度 {v!r} 不是有限的正數：拒絕寫 geo.txt（ODX 會拿到零或 NaN 的 σ）")
    return f"{x:.3f}" if x >= 0.0005 else f"{x:.3e}"


def export_filename(row) -> str:
    """匯出影像與 geo.txt 共用的檔名：``<image_id><原副檔名>``（三機同名的 000001.jpg 不會撞）。"""
    suffix = PurePosixPath(row.get("file") or "").suffix or ".jpg"
    return f"{row['image_id']}{suffix}"


def write_geo_txt(path, rows, *, height: str, projection: Optional[str] = None,
                  include_angles: bool = True, include_accuracy: bool = True) -> int:
    """寫 geo.txt，回傳寫入的影像數。

    ``height``：'ellipsoidal'（h_ell_m）或 'orthometric'（h_orth_m，全部列的 crs_v 必須相同）。
    ``projection``：預設依列的 crs_h 推（只認 EPSG:4326）；其他框架（例如 e-GNSS[2025]）
    ODM 不認得名稱，必須由呼叫端明確給 PROJ 可解析的字串，並會記錄在 log。
    任何一列缺位置、所選高度或精度都拒絕（不靜默略過，也不補預設值）。
    """
    if height not in _HEIGHT_KEY:
        raise ValueError(f"height 必須是 'ellipsoidal' 或 'orthometric'，收到 {height!r}")
    key = _HEIGHT_KEY[height]
    rows = list(rows)
    if not rows:
        raise ValueError("沒有任何照片可寫")
    crs_h = {r.get("crs_h") for r in rows}
    if len(crs_h) != 1:
        raise ValueError(f"列的水平框架不一致：{sorted(map(str, crs_h))}（不可混在同一個 geo.txt）")
    crs_h = crs_h.pop()
    if projection is None:
        if crs_h != "EPSG:4326":
            raise ValueError(f"crs_h={crs_h!r} 不是 EPSG:4326，請明確給 projection（PROJ 可解析）")
        projection = "EPSG:4326"
    elif crs_h != projection:
        _log.warning("geo.txt 標頭寫 %r，但照片的 crs_h 是 %r（呼叫端明確指定）", projection, crs_h)
    if height == "orthometric":
        vs = {r.get("crs_v") for r in rows}
        if len(vs) != 1:
            raise ValueError(f"正高的基準不一致：{sorted(map(str, vs))}")
    bad = []
    lines = [projection]
    names = set()
    for r in rows:
        need = ["lat_deg", "lon_deg", key] + (["yaw_deg", "pitch_deg", "roll_deg"] if include_angles else []) \
            + (["sigma_h_m", "sigma_v_m"] if include_accuracy else [])
        missing = [k for k in need if r.get(k) is None]
        if missing:
            bad.append((r.get("image_id"), missing))
            continue
        name = export_filename(r)
        if name in names:
            raise ValueError(f"geo.txt 檔名重複：{name}")
        names.add(name)
        parts = [name, f"{r['lon_deg']:.9f}", f"{r['lat_deg']:.9f}", f"{r[key]:.3f}"]
        if include_angles:
            parts += [f"{r['yaw_deg']:.3f}", f"{r['pitch_deg']:.3f}", f"{r['roll_deg']:.3f}"]
        elif include_accuracy:
            parts += ["nan", "nan", "nan"]
        if include_accuracy:
            parts += [_sigma_text(float(r["sigma_h_m"]) * ACCURACY_SEMANTICS["horizontal_factor"], "水平", name),
                      _sigma_text(r["sigma_v_m"], "垂直", name)]
        lines.append(" ".join(parts))
    if bad:
        raise ValueError(f"{len(bad)} 張照片缺欄位，拒絕寫 geo.txt：{bad[:5]}")
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")   # 各平台位元組相同
    return len(lines) - 1
