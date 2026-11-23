# -*- coding: utf-8 -*-
"""Shared primitives for the AASRC2026 result tables.  No repo imports.

Conventions fixed here (and quoted in every table footnote):

GEODESY  two local-tangent scalings are carried side by side because the repo
  disagrees with itself:
    "repo"    m_lat = 111132.0, m_lon = 111412.0*cos(lat)      <- hv_sitl_report.py:129-133
              (and binsep.py), i.e. what every published flown separation used
    "planner" m_lat = 111132.954 - 559.822cos2f + 1.175cos4f   <- hv_dccpp/scenario.py:92-93
              m_lon = 111412.84cos f - 93.5cos3f + 0.118cos5f
              i.e. the frame in which the 100 m rule is actually enforced.
  At the site (23.95 N) the two latitude scales differ by 0.337 %, and the
  B2-B3 encounter is almost purely north-south, so this is NOT negligible at a
  0.7 m margin.  Both are reported; the planner frame is authoritative.

CLOCK  every instance counts TimeUS from its own boot and the three SITL
  processes start ~1 s apart.  All cross-aircraft work is done on GPS
  time-of-week (GPS.GWk*604800 + GPS.GMS/1000), a genuinely common clock.
  Fleet t=0 is the AUTO-mode entry of the aircraft whose planned take-off
  delay is 0, so flown times are directly comparable with plan t_land_s.

TOUCHDOWN  primary definition = last downward crossing of RelHomeAlt = 2.0 m
  (interpolated, dataflash ~6 Hz).  The disarm-based definition used by
  hv_sitl_report.py is also computed, but for the LAST aircraft to land it is
  contaminated: the monitor loop stops at that moment, so its "disarmed"
  sample is the final CSV row rather than a real transition.
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
HERE = Path(__file__).resolve().parent
CACHE = HERE.parent / "cache"
OUT = HERE.parent / "out"
UAVS = ("A1", "B2", "B3")
AIRBORNE_M = 2.0
D_SEP_H, D_SEP_V = 100.0, 50.0
CEIL_LEGAL_M = 121.92          # 400 ft
CEIL_TARGET_M = 120.000


def m_per_deg(lat_deg: float, frame: str) -> tuple[float, float]:
    f = math.radians(lat_deg)
    if frame == "repo":
        return 111132.0, 111412.0 * math.cos(f)
    if frame == "planner":
        return (111132.954 - 559.822 * math.cos(2 * f) + 1.175 * math.cos(4 * f),
                111412.84 * math.cos(f) - 93.5 * math.cos(3 * f) + 0.118 * math.cos(5 * f))
    raise ValueError(frame)


def load(trial: str, uav: str) -> dict:
    return dict(np.load(CACHE / f"{trial}_{uav}.npz"))


def gps_clock(d: dict) -> tuple[float, float, int]:
    g = d["gps"]
    tow = g[:, 1] * 604800.0 + g[:, 2] / 1000.0
    off = tow - g[:, 0]
    return float(np.median(off)), float(off.max() - off.min()), int(len(off))


def first_auto_t(d: dict) -> float | None:
    md = d.get("mode")
    if md is None or not len(md):
        return None
    hit = md[md[:, 1] == 10.0]
    return float(hit[0, 0]) if len(hit) else None


def cross_time(t: np.ndarray, y: np.ndarray, level: float, i: int) -> float:
    """Interpolated time at which y crosses `level` between samples i and i+1."""
    y0, y1 = y[i], y[i + 1]
    if y1 == y0:
        return float(t[i])
    return float(t[i] + (level - y0) / (y1 - y0) * (t[i + 1] - t[i]))


def airborne_window(t: np.ndarray, alt: np.ndarray, level: float = AIRBORNE_M
                    ) -> tuple[float | None, float | None]:
    """(first upward crossing, last downward crossing) of `level`, interpolated."""
    above = alt > level
    if not above.any():
        return None, None
    up = np.flatnonzero((~above[:-1]) & above[1:])
    dn = np.flatnonzero(above[:-1] & (~above[1:]))
    t0 = cross_time(t, alt, level, int(up[0])) if len(up) else float(t[int(np.argmax(above))])
    t1 = cross_time(t, alt, level, int(dn[-1])) if len(dn) else float(t[-1])
    return t0, t1


def time_above(t: np.ndarray, y: np.ndarray, level: float) -> float:
    """Total time y > level, by linear interpolation of the crossings."""
    tot = 0.0
    for i in range(len(t) - 1):
        a, b, dt = y[i], y[i + 1], t[i + 1] - t[i]
        if dt <= 0:
            continue
        if a > level and b > level:
            tot += dt
        elif a > level and b <= level:
            tot += dt * (a - level) / (a - b)
        elif a <= level and b > level:
            tot += dt * (b - level) / (b - a)
    return tot


def disarm_touchdown_from_csv(trial: str, uav: str) -> tuple[float | None, bool]:
    """hv_sitl_report.py:68-80 rule, plus a flag for the 'last row' artifact."""
    p = ROOT / "runs" / "artifacts" / trial / uav / "flight.csv"
    rows = [r for r in csv.DictReader(p.open(encoding="utf-8")) if r["t_s"] and r["lat"]]
    peak, td = 0.0, None
    for r in rows:
        a = float(r["rel_alt_m"])
        peak = max(peak, a)
        if peak > 20.0 and a < 2.0 and r["armed"].strip().lower() in ("false", "0"):
            td = float(r["t_s"])
            break
    return td, (td is not None and abs(td - float(rows[-1]["t_s"])) < 1e-6)


def csv_series(trial: str, uav: str) -> dict:
    p = ROOT / "runs" / "artifacts" / trial / uav / "flight.csv"
    rows = [r for r in csv.DictReader(p.open(encoding="utf-8")) if r["t_s"] and r["lat"]]
    def col(k, f=float):
        return np.array([f(r[k]) for r in rows])
    out = {"t": col("t_s"), "lat": col("lat"), "lon": col("lon"),
           "alt": col("rel_alt_m"), "as": col("airspeed"), "gs": col("groundspeed"),
           "mode": [r["mode"] for r in rows],
           "armed": [r["armed"].strip().lower() in ("true", "1") for r in rows]}
    if "wind_spd_est" in rows[0]:
        out["wspd"] = col("wind_spd_est")
        out["wdir"] = col("wind_dir_from_est")
    return out


def path_len(lat: np.ndarray, lon: np.ndarray, lat0: float, frame: str) -> float:
    ml, mo = m_per_deg(lat0, frame)
    dx = np.diff(lon) * mo
    dy = np.diff(lat) * ml
    return float(np.hypot(dx, dy).sum())


def interp(tq: np.ndarray, t: np.ndarray, y: np.ndarray) -> np.ndarray:
    return np.interp(tq, t, y)


def circ_median_deg(a: np.ndarray) -> float:
    s, c = np.median(np.sin(np.radians(a))), np.median(np.cos(np.radians(a)))
    return float(math.degrees(math.atan2(s, c)) % 360.0)


def vec_to_from_dir(vn: float, ve: float) -> tuple[float, float]:
    """EKF wind state (vector the air is moving TOWARD, N/E) -> (speed, FROM deg)."""
    spd = math.hypot(vn, ve)
    frm = math.degrees(math.atan2(-ve, -vn)) % 360.0
    return spd, frm


def jload(p: Path):
    return json.loads(Path(p).read_text(encoding="utf-8"))


def jdump(p: Path, obj) -> None:
    Path(p).write_text(json.dumps(obj, indent=1, ensure_ascii=False,
                                  sort_keys=True, default=str), encoding="utf-8")
