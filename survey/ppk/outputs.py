"""survey/ppk/outputs.py — PPK 結果 → photos.csv（F2）相容列，以及逐條帶／時間窗的 fix 比例報告。

座標框架：PPK 解出的位置落在**基站座標所屬的框架**（ZED-F9P Integration Manual 附錄 B：rover
"will calculate its position in the reference frame used by the service provider"）。e-GNSS【2025】
不等於 TWD97，也不是 EPSG:4326 ⇒ ``crs_h`` 一律由呼叫端明確傳入（通常就是 BaseStation.frame），
這裡沒有預設值，也不會改寫成別的標籤。高度是橢球高（crs_v="ellipsoidal"），h_orth_m 留空：
正高要用明確的大地起伏模型在下游換算，不能用 N=0 靜默算出來。

fix 比例門檻預設 0.95（設計 G5：fix 比例 <95% 的條帶要標出）。
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np

from survey.ppk.events import PairedEvent, PairingResult
from survey.ppk.interp import InterpResult
from survey.ppk.pos import Q_FIX, Q_FLOAT, PosTrajectory

__all__ = [
    "PHOTOS_CSV_FIELDS",
    "FIX_RATIO_THRESHOLD",
    "NOT_FIXED_FLAGS",
    "photo_events",
    "ppk_photo_rows",
    "write_photos_csv",
    "FixWindow",
    "strip_windows",
    "fixed_windows",
    "fix_ratio_report",
]

# F2：photos.csv 欄位（順序即表頭順序）
PHOTOS_CSV_FIELDS = (
    "image_id", "file", "uav", "strip", "seq", "t_fc_us", "t_gpst_s", "lat_deg", "lon_deg",
    "h_ell_m", "h_orth_m", "yaw_deg", "pitch_deg", "roll_deg", "cog_deg", "speed_n_mps",
    "speed_e_mps", "speed_d_mps", "sigma_h_m", "sigma_v_m", "source", "crs_h", "crs_v", "flags",
)
FIX_RATIO_THRESHOLD = 0.95
NOT_FIXED_FLAGS = frozenset({"ppk_float", "ppk_gap", "ppk_out_of_range"})

_DECIMALS = {
    "t_gpst_s": 6, "lat_deg": 9, "lon_deg": 9, "h_ell_m": 4, "h_orth_m": 4,
    "yaw_deg": 3, "pitch_deg": 3, "roll_deg": 3, "cog_deg": 3,
    "speed_n_mps": 3, "speed_e_mps": 3, "speed_d_mps": 3, "sigma_h_m": 4, "sigma_v_m": 4,
}


def photo_events(pairing: PairingResult) -> tuple[list[PairedEvent], list[tuple[int, str]]]:
    """挑出真的有照片、且有曝光時刻的事件；其餘逐一列出原因（不靜默丟掉）。"""
    keep: list[PairedEvent] = []
    skipped: list[tuple[int, str]] = []
    for ev in pairing.events:
        if ev.status not in (None, "ok"):
            skipped.append((ev.seq, f"status_{ev.status}"))
        elif ev.t_exposure_gpst_s is None:
            skipped.append((ev.seq, "no_exposure_time"))
        else:
            keep.append(ev)
    return keep, skipped


def _num(v) -> float | None:
    if v is None:
        return None
    v = float(v)
    return None if not math.isfinite(v) else v


def ppk_photo_rows(
    events: Sequence[PairedEvent],
    interp: InterpResult,
    *,
    crs_h: str,
    uav: str = "",
    strip_of_seq: Mapping[int, object] | None = None,
    image_id_of_seq: Mapping[int, str] | None = None,
    attitude_deg=None,
) -> list[dict]:
    """``events[i]`` 對應 ``interp`` 的第 i 列（呼叫端用 events 的曝光時刻做內插）。

    attitude_deg：與 events 對齊的 (M,3) [yaw, pitch, roll]（就是做槓桿臂用的那一組）；不給則留空。
    """
    if not isinstance(crs_h, str) or not crs_h.strip():
        raise ValueError("crs_h 必須明確給出基站座標的框架名稱（例如 'e-GNSS[2025]'）")
    if len(events) != len(interp):
        raise ValueError(f"events {len(events)} 筆與內插結果 {len(interp)} 筆不符")
    att = None
    if attitude_deg is not None:
        att = np.asarray(attitude_deg, dtype=float).reshape(-1, 3)
        if att.shape[0] != len(events):
            raise ValueError(f"姿態 {att.shape[0]} 筆與 events {len(events)} 筆不符")
    rows: list[dict] = []
    cog = interp.cog_deg
    for i, ev in enumerate(events):
        if ev.t_exposure_gpst_s is None or abs(ev.t_exposure_gpst_s - interp.t_gpst_s[i]) > 1e-6:
            raise ValueError(f"第 {i} 筆事件（seq {ev.seq}）的曝光時刻與內插時刻不一致")
        flags: list[str] = []
        for f in (*ev.flags, *interp.flags[i]):
            if f not in flags:
                flags.append(f)
        v = interp.vel_ned_mps[i]
        rows.append({
            "image_id": (image_id_of_seq or {}).get(ev.seq, ""),
            "file": ev.file or "",
            "uav": uav,
            "strip": (strip_of_seq or {}).get(ev.seq, ""),
            "seq": ev.seq,
            "t_fc_us": None,
            "t_gpst_s": _num(interp.t_gpst_s[i]),
            "lat_deg": _num(interp.lat_deg[i]),
            "lon_deg": _num(interp.lon_deg[i]),
            "h_ell_m": _num(interp.h_ell_m[i]),
            "h_orth_m": None,
            "yaw_deg": None if att is None else _num(att[i, 0]),
            "pitch_deg": None if att is None else _num(att[i, 1]),
            "roll_deg": None if att is None else _num(att[i, 2]),
            "cog_deg": _num(cog[i]),
            "speed_n_mps": _num(v[0]),
            "speed_e_mps": _num(v[1]),
            "speed_d_mps": _num(v[2]),
            "sigma_h_m": _num(interp.sigma_h_m[i]),
            "sigma_v_m": _num(interp.sigma_v_m[i]),
            "source": "ppk",
            "crs_h": crs_h,
            "crs_v": "ellipsoidal",
            "flags": ";".join(flags),
        })
    return rows


def _fmt(key: str, v) -> str:
    if v is None:
        return ""
    if isinstance(v, float):
        if not math.isfinite(v):
            return ""
        d = _DECIMALS.get(key)
        return f"{v:.{d}f}" if d is not None else repr(v)
    return str(v)


def write_photos_csv(rows: Sequence[Mapping], path: str | Path) -> Path:
    p = Path(path)
    with open(p, "w", encoding="utf-8", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(PHOTOS_CSV_FIELDS)
        for r in rows:
            w.writerow([_fmt(k, r.get(k)) for k in PHOTOS_CSV_FIELDS])
    return p


# ---------------------------------------------------------------------------
# fix 比例報告
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class FixWindow:
    label: str
    t_start_gpst_s: float
    t_end_gpst_s: float
    n_epochs: int
    n_fix: int
    n_float: int
    n_other: int               # Q 3–6（sbas/dgps/single/ppp）
    fix_ratio: float | None    # 沒有任何 epoch 時為 None（視同不合格）
    n_photos: int
    n_photos_not_fixed: int    # 帶 ppk_float／ppk_gap／ppk_out_of_range 的照片數
    below_threshold: bool


def strip_windows(t_gpst_s, strips) -> list[tuple[str, float, float]]:
    """依照片的條帶標籤，取每條帶第一張到最後一張的時間範圍。"""
    t = np.asarray(t_gpst_s, dtype=float)
    out: dict[str, list[float]] = {}
    for ti, s in zip(t, strips):
        if s is None or s == "" or not math.isfinite(ti):
            continue
        lo_hi = out.setdefault(str(s), [ti, ti])
        lo_hi[0], lo_hi[1] = min(lo_hi[0], ti), max(lo_hi[1], ti)
    return [(k, v[0], v[1]) for k, v in sorted(out.items(), key=lambda kv: kv[1][0])]


def fixed_windows(t_start: float, t_end: float, width_s: float) -> list[tuple[str, float, float]]:
    if not (width_s > 0):
        raise ValueError(f"width_s 必須 > 0，收到 {width_s!r}")
    out = []
    t = float(t_start)
    while t < t_end:
        out.append((f"{t:.1f}", t, min(t + width_s, float(t_end))))
        t += width_s
    return out


def fix_ratio_report(
    traj: PosTrajectory,
    windows: Sequence[tuple[str, float, float]],
    *,
    threshold: float = FIX_RATIO_THRESHOLD,
    photo_t=None,
    photo_flags: Sequence[Sequence[str]] | None = None,
) -> list[FixWindow]:
    """每個 (label, t0, t1) 視窗（含兩端）的 epoch fix 比例；可附上照片時間與旗標一起統計。"""
    if not (0.0 < threshold <= 1.0):
        raise ValueError(f"threshold 必須在 (0, 1]，收到 {threshold!r}")
    pt = None if photo_t is None else np.asarray(photo_t, dtype=float)
    if pt is not None and photo_flags is not None and len(photo_flags) != pt.size:
        raise ValueError("photo_flags 與 photo_t 長度不符")
    out: list[FixWindow] = []
    for label, t0, t1 in windows:
        m = (traj.t_gpst_s >= t0) & (traj.t_gpst_s <= t1)
        q = traj.q[m]
        n = int(q.size)
        n_fix = int(np.count_nonzero(q == Q_FIX))
        n_float = int(np.count_nonzero(q == Q_FLOAT))
        ratio = n_fix / n if n else None
        n_ph = n_bad = 0
        if pt is not None:
            pm = (pt >= t0) & (pt <= t1)
            n_ph = int(np.count_nonzero(pm))
            if photo_flags is not None:
                n_bad = sum(1 for k in np.nonzero(pm)[0] if NOT_FIXED_FLAGS & set(photo_flags[k]))
        out.append(FixWindow(
            label=str(label), t_start_gpst_s=float(t0), t_end_gpst_s=float(t1), n_epochs=n,
            n_fix=n_fix, n_float=n_float, n_other=n - n_fix - n_float, fix_ratio=ratio,
            n_photos=n_ph, n_photos_not_fixed=n_bad,
            below_threshold=(ratio is None or ratio < threshold),
        ))
    return out
