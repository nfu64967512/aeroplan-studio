"""survey/ppk/pos.py — 讀 RTKLIB（demo5）rnx2rtkp 輸出的 .pos 軌跡。

格式依據：rtklibexplorer/RTKLIB demo5 分支 src/solution.c（outsolheads／outpos／outecef／outsols）。
- 表頭行以 ``%`` 開頭；欄名行是 ``%  GPST  latitude(deg) longitude(deg) height(m) Q ns sdn(m) ...``，
  時間標籤取自 ``{"GPST","UTC ","JST "}``（依 out-timesys）。
- 時間欄位：out-timeform=tow ⇒ ``%4d%s%*.*f``（週、分隔、週內秒）；=hms ⇒ time2str 的
  ``yyyy/mm/dd hh:mm:ss.sss``。兩種都佔資料行的**兩個欄位**，欄名行只寫一個標籤。
- 經緯高（out-solformat=llh、out-degform=deg）：latitude(deg) longitude(deg) height(m) Q ns
  sdn(m) sde(m) sdu(m) sdne(m) sdeu(m) sdun(m) age(s) ratio；out-outvel=on 時再接
  vn(m/s) ve(m/s) vu(m/s) sdvn sdve sdvu sdvne sdveu sdvun。
- ECEF（out-solformat=xyz）：x-ecef(m) y-ecef(m) z-ecef(m) Q ns sdx(m) sdy(m) sdz(m) sdxy(m)
  sdyz(m) sdzx(m) age(s) ratio；速度 vx(m/s) vy(m/s) vz(m/s) sdvx … sdvzx。
- 對角項是標準差（SQRT），交叉項是 ``sqvar(cov) = sign(cov)·sqrt(|cov|)``，**可為負**。
- 高度型別在 ``% (lat/lon/height=WGS84/ellipsoidal,Q=1:fix,2:float,...)`` 這一行
  （out-height=geodetic 時寫 geodetic，是減過大地起伏的正高）。
- Q：1 fix、2 float、3 sbas、4 dgps、5 single、6 ppp。

刻意拒絕（直接丟錯，不猜）：時間不是 GPST（UTC/JST）、高度是 geodetic、度分秒格式、
ENU 基線格式、沒有表頭（out-outhead=off ⇒ 無從確認欄位與高度型別）。
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import numpy as np

from survey.ppk.events import GPS_WEEK_S
from survey.ppk.geodesy import ecef_to_llh, ecef_to_ned_delta, ned_basis

__all__ = [
    "Q_FIX",
    "Q_FLOAT",
    "Q_SBAS",
    "Q_DGPS",
    "Q_SINGLE",
    "Q_PPP",
    "GPS_WEEK_S",
    "PosFormatError",
    "PosTrajectory",
    "gpst_calendar_to_seconds",
    "parse_pos",
    "read_pos",
]

Q_FIX, Q_FLOAT, Q_SBAS, Q_DGPS, Q_SINGLE, Q_PPP = 1, 2, 3, 4, 5, 6
_GPS_EPOCH = datetime(1980, 1, 6)

_LLH_REQ = ("latitude(deg)", "longitude(deg)", "height(m)", "Q", "ns",
            "sdn(m)", "sde(m)", "sdu(m)", "sdne(m)", "sdeu(m)", "sdun(m)")
_XYZ_REQ = ("x-ecef(m)", "y-ecef(m)", "z-ecef(m)", "Q", "ns",
            "sdx(m)", "sdy(m)", "sdz(m)", "sdxy(m)", "sdyz(m)", "sdzx(m)")
_LLH_VEL = ("vn(m/s)", "ve(m/s)", "vu(m/s)")
_XYZ_VEL = ("vx(m/s)", "vy(m/s)", "vz(m/s)")
_HEIGHT_RE = re.compile(r"lat/lon/height=([^/,\s]+)/([^,\s)]+)")


class PosFormatError(ValueError):
    """.pos 檔的格式或設定不是本模組能正確解讀的。"""


@dataclass
class PosTrajectory:
    """一條 PPK 軌跡；所有陣列長度相同，時間嚴格遞增。高度一律是**橢球高**。"""
    t_gpst_s: np.ndarray          # GPST 秒 = week·604800 + TOW（無閏秒）
    lat_deg: np.ndarray
    lon_deg: np.ndarray
    h_ell_m: np.ndarray
    q: np.ndarray                 # int
    ns: np.ndarray                # int
    sdn_m: np.ndarray
    sde_m: np.ndarray
    sdu_m: np.ndarray
    sdne_m: np.ndarray            # sign(cov)·sqrt(|cov|)
    sdeu_m: np.ndarray
    sdun_m: np.ndarray
    age_s: np.ndarray | None
    ratio: np.ndarray | None
    vel_ned_mps: np.ndarray | None  # (N,3)；檔案沒有速度欄位時為 None
    solution_format: str          # "llh" | "xyz"
    time_format: str              # "tow" | "hms"
    datum_label: str              # 表頭寫的 "WGS84"；實際框架＝基站座標的框架（見 outputs.py）
    header: list[str] = field(default_factory=list)
    skipped_lines: list[int] = field(default_factory=list)  # 被略過的行號（只允許最後一行截斷）
    source: str | None = None

    def __len__(self) -> int:
        return int(self.t_gpst_s.size)

    @property
    def sigma_h_m(self) -> np.ndarray:
        """水平 σ = sqrt(sdn² + sde²)（2D RMS，與 u-blox hAcc 同一類量）。"""
        return np.hypot(self.sdn_m, self.sde_m)

    @property
    def sigma_v_m(self) -> np.ndarray:
        return np.abs(self.sdu_m)


def gpst_calendar_to_seconds(date: str, time: str) -> float:
    """``yyyy/mm/dd`` ``hh:mm:ss.sss``（GPST 曆法時間，**沒有閏秒**）→ GPST 秒。

    GPST 不插閏秒，所以直接用日曆差即可；秒的小數部分分開加，避免 1.4e9 量級下失去精度。
    """
    hh, mm, ss = time.split(":")
    whole, _, frac = ss.partition(".")
    base = datetime.strptime(f"{date} {hh}:{mm}:00", "%Y/%m/%d %H:%M:%S")
    days = (base - _GPS_EPOCH).days
    secs = (base - _GPS_EPOCH).seconds
    return float(days * 86400 + secs + int(whole)) + (float("0." + frac) if frac else 0.0)


def _sq_signed(v: np.ndarray) -> np.ndarray:
    """sqvar 的反運算：sign·sd² = 共變異數。"""
    return np.sign(v) * v * v


def parse_pos(text: str, source: str | None = None) -> PosTrajectory:
    header: list[str] = []
    col_line: list[str] | None = None
    time_label = None
    height_kind = None
    datum = None
    rows: list[tuple[int, list[str]]] = []
    for ln, raw in enumerate(text.splitlines(), start=1):
        line = raw.strip()
        if not line:
            continue
        if line.startswith("%"):
            header.append(raw)
            toks = line[1:].split()
            if toks and toks[0] in ("GPST", "UTC", "JST"):
                col_line = toks
                time_label = toks[0]
            m = _HEIGHT_RE.search(line)
            if m:
                datum, height_kind = m.group(1), m.group(2)
            elif "x/y/z-ecef=" in line:
                datum = line.split("x/y/z-ecef=", 1)[1].split(",", 1)[0]
            continue
        rows.append((ln, line.replace(",", " ").split()))

    if col_line is None:
        raise PosFormatError(
            ".pos 沒有欄名表頭（% GPST ...）—— 請以 out-outhead=on 重新解算；"
            "沒有表頭就無法確認欄位順序、時間系統與高度型別")
    if time_label != "GPST":
        raise PosFormatError(
            f".pos 的時間系統是 {time_label}，不是 GPST —— 請設 out-timesys=gpst 重新解算"
            "（不在這裡自行加減閏秒）")
    names = col_line[1:]
    if any("(d'" in n for n in names):
        raise PosFormatError(".pos 是度分秒格式 —— 請設 out-degform=deg")
    if "latitude(deg)" in names:
        fmt, req, vel_names = "llh", _LLH_REQ, _LLH_VEL
        if height_kind is None:
            raise PosFormatError(
                ".pos 表頭缺 (lat/lon/height=...) 說明行，無法確認高度是橢球高還是正高")
        if height_kind != "ellipsoidal":
            raise PosFormatError(
                f".pos 的高度是 {height_kind}，不是 ellipsoidal —— 請設 out-height=ellipsoidal；"
                "正高要在下游以明確的大地起伏模型換算，不能混進 PPK 軌跡")
    elif "x-ecef(m)" in names:
        fmt, req, vel_names = "xyz", _XYZ_REQ, _XYZ_VEL
    else:
        raise PosFormatError(f".pos 欄位無法辨識（只支援 llh 與 xyz）：{' '.join(names)}")
    if datum not in (None, "WGS84"):
        raise PosFormatError(f".pos 的 datum 是 {datum}，只接受 RTKLIB 不做轉換的 WGS84 標籤")
    missing = [n for n in req if n not in names]
    if missing:
        raise PosFormatError(f".pos 缺必要欄位：{missing}")
    idx = {n: k for k, n in enumerate(names)}
    has_vel = all(n in idx for n in vel_names)
    ntok = 2 + len(names)   # 時間在資料行佔兩個 token，欄名行只寫一個標籤（已去掉）

    tf = None
    good: list[list[str]] = []
    skipped: list[int] = []
    # RTKLIB 每行都以換行結束；檔尾沒有換行 ⇒ 最後一行可能截在數字中間而欄位數剛好對，直接略過並記下
    cut = bool(rows) and not text.endswith(("\n", "\r"))
    if cut:
        skipped.append(rows[-1][0])
        rows = rows[:-1]
    for k, (ln, toks) in enumerate(rows):
        if len(toks) != ntok:
            if k == len(rows) - 1 and not cut:
                skipped.append(ln)   # 最後一行截斷（解算中斷、檔案還在寫）—— 記下來，不靜默
                continue
            raise PosFormatError(f".pos 第 {ln} 行有 {len(toks)} 個欄位，應為 {ntok}")
        this_tf = "hms" if "/" in toks[0] else "tow"
        if tf is None:
            tf = this_tf
        elif this_tf != tf:
            raise PosFormatError(f".pos 第 {ln} 行時間格式與前面不同")
        good.append(toks)
    if not good:
        raise PosFormatError(".pos 沒有任何解")

    try:
        if tf == "hms":
            t = np.array([gpst_calendar_to_seconds(r[0], r[1]) for r in good])
        else:
            t = np.array([int(r[0]) * GPS_WEEK_S + float(r[1]) for r in good])
        val = np.array([[float(x) for x in r[2:]] for r in good], dtype=float)
    except ValueError as e:
        raise PosFormatError(f".pos 數值無法解析：{e}") from None
    if np.any(np.diff(t) <= 0):
        k = int(np.argmax(np.diff(t) <= 0))
        raise PosFormatError(f".pos 時間不是嚴格遞增（第 {k + 2} 筆解）")

    def col(n: str) -> np.ndarray:
        return val[:, idx[n]]

    vel_ned = None
    if fmt == "llh":
        lat, lon, h = col("latitude(deg)"), col("longitude(deg)"), col("height(m)")
        sdn, sde, sdu = col("sdn(m)"), col("sde(m)"), col("sdu(m)")
        sdne, sdeu, sdun = col("sdne(m)"), col("sdeu(m)"), col("sdun(m)")
        if has_vel:
            vel_ned = np.stack([col("vn(m/s)"), col("ve(m/s)"), -col("vu(m/s)")], axis=-1)
    else:
        xyz = np.stack([col("x-ecef(m)"), col("y-ecef(m)"), col("z-ecef(m)")], axis=-1)
        lat, lon, h = ecef_to_llh(xyz)
        # ECEF 共變異數轉到 NED，再取 RTKLIB 的 sd 表示法（對角開根號、交叉項 sqvar）
        sx, sy, sz = col("sdx(m)"), col("sdy(m)"), col("sdz(m)")
        cxy, cyz, czx = _sq_signed(col("sdxy(m)")), _sq_signed(col("sdyz(m)")), _sq_signed(col("sdzx(m)"))
        cov = np.empty((len(t), 3, 3))
        cov[:, 0, 0], cov[:, 1, 1], cov[:, 2, 2] = sx * sx, sy * sy, sz * sz
        cov[:, 0, 1] = cov[:, 1, 0] = cxy
        cov[:, 1, 2] = cov[:, 2, 1] = cyz
        cov[:, 0, 2] = cov[:, 2, 0] = czx
        b = ned_basis(lat, lon)
        cn = np.einsum("nij,njk,nlk->nil", b, cov, b)

        def sq(v):
            return np.sign(v) * np.sqrt(np.abs(v))
        sdn, sde, sdu = np.sqrt(cn[:, 0, 0]), np.sqrt(cn[:, 1, 1]), np.sqrt(cn[:, 2, 2])
        # RTKLIB 的 sdu 是「上」軸，與 NED 的 D 只差正負號，交叉項 ne、eu、un 依 ENU 定義
        sdne, sdeu, sdun = sq(cn[:, 0, 1]), sq(-cn[:, 1, 2]), sq(-cn[:, 2, 0])
        if has_vel:
            v = np.stack([col("vx(m/s)"), col("vy(m/s)"), col("vz(m/s)")], axis=-1)
            vel_ned = ecef_to_ned_delta(lat, lon, v)

    return PosTrajectory(
        t_gpst_s=t, lat_deg=np.asarray(lat, float), lon_deg=np.asarray(lon, float),
        h_ell_m=np.asarray(h, float),
        q=col("Q").astype(int), ns=col("ns").astype(int),
        sdn_m=sdn, sde_m=sde, sdu_m=sdu, sdne_m=sdne, sdeu_m=sdeu, sdun_m=sdun,
        age_s=col("age(s)") if "age(s)" in idx else None,
        ratio=col("ratio") if "ratio" in idx else None,
        vel_ned_mps=vel_ned, solution_format=fmt, time_format=tf or "tow",
        datum_label=datum or "WGS84", header=header, skipped_lines=skipped, source=source,
    )


def read_pos(path: str | Path) -> PosTrajectory:
    p = Path(path)
    return parse_pos(p.read_text(encoding="utf-8", errors="strict"), source=str(p))
