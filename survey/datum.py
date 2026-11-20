"""時間系統與座標／高程基準 —— 航測 ingest 唯一的換算處。

為什麼獨立成一支
================
整條鏈混著三種高度、兩種時間，任何一處「順手」換算都會靜默錯 18 m 或 18 s：

* 飛控 ``GPS.Alt`` / ``POS.Alt`` / ``CAM.Alt`` 是 **AMSL**，用的是接收機內建的大地起伏
  （u-blox 為低解析 EGM96）；EKF 的高度還融合了氣壓計。
* e-GNSS／PPK 給的是 **橢球高**；Cesium 也用橢球高。
* 本場址（25.10N 121.20E）EGM96 大地起伏 N ≈ +18.54 m（評估 §8，pyproj 計算）——
  **只用在測試與文件，絕不當預設值**。

規則（有效值契約）：每個高度都帶著它的 ``crs_v``；換算一定要有明確的大地起伏 N
（來自日誌 GPA，或使用者明確給的值／格網）；N 未知時正高欄留空，**不拿 N=0 硬算**。

ArduPilot 的「undulation」正負號與大地測量慣例相反
--------------------------------------------------
``AP_GPS::get_undulation`` 的註解：“A positive value means the AMSL height is higher than
WGS84 ellipsoid height”，``Write_GPS`` 以 ``h_ell = alt − undulation`` 算橢球高
（Plane-4.5.7 libraries/AP_GPS/AP_GPS.cpp:2333-2342、1470-1473）。u-blox 驅動填的是
``hMSL − height``。所以日誌的 ``GPA.Und`` = H − h = **−N**。
另外 ``have_undulation`` 為假時 ``Und`` 照樣寫 0，與真正的 0 分不出來 ⇒ ``Und == 0`` 視為未知。
較新的韌體改寫 ``GPA.AEl``（橢球高，cm；未知時寫 INT32_MIN，master AP_GPS.cpp:1940-1946），
此時 N = AEl − GPS.Alt。兩種欄位都依該檔 FMT 判斷，不靠版本號猜。

時間（F3）：GPST 秒 = GPS 週 × 604800 + 週內秒（不含閏秒）；UTC = GPST − 18 s。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from typing import Callable, Optional, Union

import numpy as np

# ── 時間 ────────────────────────────────────────────────────────────────
GPS_EPOCH = datetime(1980, 1, 6, tzinfo=timezone.utc)
SECONDS_PER_WEEK = 604800
# GPST − UTC。2017-01-01 起為 18 s；之前的日期本模組不處理（沒有閏秒表 ⇒ 直接拒絕）
GPST_MINUS_UTC_S = 18
_LEAP_VALID_FROM = datetime(2017, 1, 1, tzinfo=timezone.utc)


def gpst_seconds(week: int, tow_s: float) -> float:
    """GPS 週與週內秒 → GPST 秒（自 1980-01-06 起算，不含閏秒）。"""
    return float(week) * SECONDS_PER_WEEK + float(tow_s)


def gpst_to_utc(gpst_s: float) -> datetime:
    """GPST 秒 → UTC（含時區的 datetime）。2017 以前的時刻直接拒絕。"""
    utc = GPS_EPOCH + timedelta(seconds=float(gpst_s) - GPST_MINUS_UTC_S)
    if utc < _LEAP_VALID_FROM:
        raise ValueError(f"UTC {utc.isoformat()} 早於 2017-01-01：GPST−UTC 不是 {GPST_MINUS_UTC_S} s，本模組不處理")
    return utc


def utc_to_gpst(utc: datetime) -> float:
    """UTC（含時區）→ GPST 秒。"""
    if utc.tzinfo is None:
        raise ValueError("utc 必須含時區（naive datetime 會被誤當本地時間）")
    if utc < _LEAP_VALID_FROM:
        raise ValueError(f"UTC {utc.isoformat()} 早於 2017-01-01，本模組不處理")
    return (utc - GPS_EPOCH).total_seconds() + GPST_MINUS_UTC_S


# ── 垂直基準 ────────────────────────────────────────────────────────────
VERTICAL_DATUMS = ("ellipsoidal", "EGM96", "TWVD2001", "unknown")
ORTHOMETRIC_DATUMS = ("EGM96", "TWVD2001")


@dataclass(frozen=True)
class GeoidModel:
    """使用者明確給的大地起伏模型（N = h_ell − H，大地測量慣例，台灣約 +18.5 m）。

    ``undulation_m`` 可以是常數（小區域）或 ``f(lat_deg, lon_deg) -> N`` 的格網查詢。
    ``source`` 寫進交付物 metadata，說明這個 N 從哪來。
    """
    datum: str
    undulation_m: Union[float, Callable[[float, float], float]]
    source: str = ""

    def __post_init__(self):
        if self.datum not in ORTHOMETRIC_DATUMS:
            raise ValueError(f"GeoidModel.datum 必須是 {ORTHOMETRIC_DATUMS} 之一，收到 {self.datum!r}")

    def n_at(self, lat_deg: float, lon_deg: float) -> float:
        n = self.undulation_m(lat_deg, lon_deg) if callable(self.undulation_m) else self.undulation_m
        n = float(n)
        if not math.isfinite(n):
            raise ValueError(f"大地起伏模型在 ({lat_deg}, {lon_deg}) 回傳非有限值 {n}")
        return n


@dataclass(frozen=True)
class ReceiverUndulation:
    """飛控 GNSS 接收機自己用的大地起伏（把它的 AMSL 還原成橢球高用）。

    ``n_m`` 採大地測量慣例 N = h_ell − H_amsl；``None`` 表示日誌裡沒有可信的值。
    """
    n_m: Optional[float]
    source: str               # 'GPA.Und' / 'GPA.AEl' / 'user' / 'none'
    n_samples: int = 0
    spread_m: Optional[float] = None


# DFReader 把 'e'（int32 cm）乘 0.01；INT32_MIN 就是 −21474836.48 m
_AEL_UNKNOWN_BELOW_M = -1.0e6


def receiver_undulation_from_log(log, gps_instance: int = 0) -> ReceiverUndulation:
    """從 DataFlash 的 GPA（配 GPS）取接收機大地起伏。欄位依 FMT 判斷（Und 或 AEl）。"""
    gpa = log.tables.get("GPA") or {}
    gps = log.tables.get("GPS") or {}
    if not gpa or len(gpa.get("TimeUS", ())) == 0:
        return ReceiverUndulation(None, "none")
    inst = gpa.get("I")
    sel = np.ones(len(gpa["TimeUS"]), bool) if inst is None else (np.asarray(inst) == gps_instance)
    if "AEl" in gpa:
        if not gps or len(gps.get("TimeUS", ())) == 0:
            return ReceiverUndulation(None, "none")
        # GPA 與 GPS 在 Write_GPS 裡以同一個 time_us 先後寫入 ⇒ 用 (TimeUS, I) 精確配對
        g_inst = gps.get("I")
        g_sel = np.ones(len(gps["TimeUS"]), bool) if g_inst is None else (np.asarray(g_inst) == gps_instance)
        alt_by_t = dict(zip(np.asarray(gps["TimeUS"])[g_sel].tolist(), np.asarray(gps["Alt"])[g_sel].tolist()))
        ns = []
        for t, ael in zip(np.asarray(gpa["TimeUS"])[sel].tolist(), np.asarray(gpa["AEl"])[sel].tolist()):
            if ael <= _AEL_UNKNOWN_BELOW_M or t not in alt_by_t:
                continue
            ns.append(ael - alt_by_t[t])
        source = "GPA.AEl"
    elif "Und" in gpa:
        und = np.asarray(gpa["Und"], float)[sel]
        # ArduPilot：Und = H − h（AP_GPS.cpp get_undulation 註解）⇒ N = −Und；0 代表「接收機沒給」
        ns = [-u for u in und.tolist() if u != 0.0 and math.isfinite(u)]
        source = "GPA.Und"
    else:
        return ReceiverUndulation(None, "none")
    if not ns:
        return ReceiverUndulation(None, source + "(none valid)")
    arr = np.asarray(ns, float)
    return ReceiverUndulation(float(np.median(arr)), source, int(arr.size), float(arr.max() - arr.min()))


def fc_heights(h_amsl: float, lat_deg: float, lon_deg: float, *,
               receiver: ReceiverUndulation,
               geoid: Optional[GeoidModel] = None,
               fc_amsl_datum: Optional[str] = None):
    """飛控 AMSL 高 → (h_ell, h_orth, crs_v, flags)。

    * h_ell = H_amsl + N_接收機；接收機 N 未知 ⇒ None（旗標 ``h_ell_unknown``）。
    * 給了 ``geoid`` ⇒ h_orth = h_ell − N_geoid，crs_v = geoid.datum（h_ell 未知時無法換算 ⇒ 留空）。
    * 沒給 ``geoid`` 但使用者明確宣告 ``fc_amsl_datum``（接收機 AMSL 屬哪個正高系統）⇒ h_orth = H_amsl。
    * 兩者都沒有 ⇒ h_orth 留空；crs_v = 'ellipsoidal'（只剩橢球高）或 'unknown'（什麼都沒有）。
    """
    if geoid is not None and fc_amsl_datum is not None:
        raise ValueError("geoid 與 fc_amsl_datum 不能同時給：兩者都在定義 h_orth_m，必須只有一個來源")
    if fc_amsl_datum is not None and fc_amsl_datum not in ORTHOMETRIC_DATUMS:
        raise ValueError(f"fc_amsl_datum 必須是 {ORTHOMETRIC_DATUMS} 之一，收到 {fc_amsl_datum!r}")
    flags = []
    h_amsl = float(h_amsl)
    h_ell = None
    if receiver.n_m is not None:
        h_ell = h_amsl + receiver.n_m
    else:
        flags.append("h_ell_unknown")
    h_orth = None
    if geoid is not None:
        if h_ell is None:
            flags.append("h_orth_needs_h_ell")
        else:
            h_orth = h_ell - geoid.n_at(lat_deg, lon_deg)
        crs_v = geoid.datum if h_orth is not None else ("ellipsoidal" if h_ell is not None else "unknown")
    elif fc_amsl_datum is not None:
        h_orth = h_amsl
        crs_v = fc_amsl_datum
        flags.append("h_orth_receiver_geoid")
    else:
        crs_v = "ellipsoidal" if h_ell is not None else "unknown"
    if h_ell is None and h_orth is None:
        flags.append("height_unknown")
    return h_ell, h_orth, crs_v, flags


def ellipsoidal_heights(h_ell: float, lat_deg: float, lon_deg: float, *,
                        geoid: Optional[GeoidModel] = None):
    """橢球高（PPK、e-GNSS）→ (h_ell, h_orth, crs_v, flags)；沒給 geoid 時正高留空。"""
    h_ell = float(h_ell)
    if geoid is None:
        return h_ell, None, "ellipsoidal", []
    return h_ell, h_ell - geoid.n_at(lat_deg, lon_deg), geoid.datum, []


# ── 水平：WGS84 小位移換算 ─────────────────────────────────────────────
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def wgs84_radii(lat_deg):
    """子午圈曲率半徑 M 與卯酉圈曲率半徑 N（m）。"""
    s = np.sin(np.radians(lat_deg))
    w = 1.0 - WGS84_E2 * s * s
    return WGS84_A * (1.0 - WGS84_E2) / w ** 1.5, WGS84_A / np.sqrt(w)


def ned_offset_to_llh(lat_deg, lon_deg, h_m, dn_m, de_m, dd_m):
    """在 (lat, lon, h) 加上局部 NED 位移（≲ 數十 m，誤差 < 1 mm）。高度沿用輸入的基準。"""
    lat_deg = np.asarray(lat_deg, float)
    m, n = wgs84_radii(lat_deg)
    lat2 = lat_deg + np.degrees(np.asarray(dn_m, float) / (m + h_m))
    lon2 = np.asarray(lon_deg, float) + np.degrees(np.asarray(de_m, float) / ((n + h_m) * np.cos(np.radians(lat_deg))))
    return lat2, lon2, np.asarray(h_m, float) - np.asarray(dd_m, float)


def local_en_m(lat0_deg, lon0_deg, lat_deg, lon_deg):
    """以 (lat0, lon0) 為原點的局部東、北距離（m；小區域等距近似，2 km 內誤差 ~mm）。"""
    m, n = wgs84_radii(lat0_deg)
    e = np.radians(np.asarray(lon_deg, float) - lon0_deg) * n * math.cos(math.radians(lat0_deg))
    no = np.radians(np.asarray(lat_deg, float) - lat0_deg) * m
    return e, no


# ArduPilot Location 的平面近似（libraries/AP_Common/Location.cpp：LOCATION_SCALING_FACTOR，
# 經度用中點緯度縮放）。XKF1 的 PN/PE 是 EKF 以同一近似從原點算出來的 ⇒ 反算要用同一套才一致。
_AP_LOCATION_SCALING_FACTOR = 0.011131884502145034   # m / (1e-7 deg)


def ap_ned_to_llh(origin_lat_deg, origin_lon_deg, origin_alt_m, pn_m, pe_m, pd_m):
    """EKF 原點 + NED（XKF1 PN/PE/PD）→ (lat, lon, alt)，與 ArduPilot ``Location::offset`` 相同近似。"""
    pn = np.asarray(pn_m, float)
    pe = np.asarray(pe_m, float)
    dlat_deg = pn / (_AP_LOCATION_SCALING_FACTOR * 1e7)
    lat = origin_lat_deg + dlat_deg
    lon_scale = np.maximum(np.cos(np.radians(origin_lat_deg + dlat_deg / 2.0)), 0.01)
    lon = origin_lon_deg + pe / (_AP_LOCATION_SCALING_FACTOR * 1e7) / lon_scale
    return lat, lon, origin_alt_m - np.asarray(pd_m, float)
