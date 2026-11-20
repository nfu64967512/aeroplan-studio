# -*- coding: utf-8 -*-
"""label update_paper -- primitives, written from scratch for this label.

No import of ana_lib.py, binalt.py, binsep.py, accept.py or hv_sitl_report.py.
Conventions are restated here so every table footnote can quote them:

GEODESY   'ecef'   exact WGS84 geodetic -> ECEF -> local ENU at the home datum.
                   This is the frame mission geometry actually lives in
                   (hv_dccpp/geo.py GeoFrame, referenced by scenario.py:87-88),
                   so it is the frame the 100 m rule is enforced in.  AUTHORITATIVE.
          'planner' ellipsoidal m/deg series (hv_dccpp/scenario.py:92-93).
          'repo'    flat 111132 m/deg lat (hv_sitl_report.py) -- carried only to
                    show what the previously published numbers used.

CLOCK     each SITL instance counts TimeUS from its own boot; the three
          processes start ~1 s apart.  Everything cross-aircraft is done on GPS
          time-of-week (GPS.GWk*604800 + GPS.GMS/1000).

AIRBORNE  rel alt (POS.RelHomeAlt) > 2.0 m.  Makespan t0 = the AUTO-mode entry
          of the aircraft whose planned take-off delay is 0, minus that delay;
          touchdown = last downward crossing of 2.0 m, interpolated.
"""
from __future__ import annotations

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
CEIL_OPS_M = 120.000
CEIL_LEGAL_M = 121.92

WGS84_A, WGS84_F = 6378137.0, 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def jload(p):
    return json.loads(Path(p).read_text(encoding="utf-8"))


def jdump(p, obj):
    Path(p).write_text(json.dumps(obj, indent=1, ensure_ascii=False, sort_keys=True,
                                  default=str), encoding="utf-8")


def load(trial: str, uav: str) -> dict:
    return dict(np.load(CACHE / f"{trial}_{uav}.npz"))


def m_per_deg(lat_deg: float, frame: str):
    f = math.radians(lat_deg)
    if frame == "repo":
        return 111132.0, 111412.0 * math.cos(f)
    if frame == "planner":
        return (111132.954 - 559.822 * math.cos(2 * f) + 1.175 * math.cos(4 * f),
                111412.84 * math.cos(f) - 93.5 * math.cos(3 * f) + 0.118 * math.cos(5 * f))
    raise ValueError(frame)


def _ecef(lat, lon, h):
    p, l = np.radians(lat), np.radians(lon)
    n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * np.sin(p) ** 2)
    return np.stack([(n + h) * np.cos(p) * np.cos(l),
                     (n + h) * np.cos(p) * np.sin(l),
                     (n * (1.0 - WGS84_E2) + h) * np.sin(p)], axis=-1)


def enu(lat, lon, lat0, lon0):
    lat = np.asarray(lat, dtype=float)
    lon = np.asarray(lon, dtype=float)
    d = _ecef(lat, lon, np.zeros_like(lat)) - _ecef(np.array([lat0]), np.array([lon0]),
                                                    np.array([0.0]))
    p0, l0 = math.radians(lat0), math.radians(lon0)
    e = -math.sin(l0) * d[:, 0] + math.cos(l0) * d[:, 1]
    n = (-math.sin(p0) * math.cos(l0) * d[:, 0] - math.sin(p0) * math.sin(l0) * d[:, 1]
         + math.cos(p0) * d[:, 2])
    return e, n


def gps_offset(d: dict) -> float:
    """boot-clock -> GPS time-of-week offset (median over the whole log)."""
    g = d["gps"]
    tow = g[:, 1] * 604800.0 + g[:, 2] / 1000.0
    return float(np.median(tow - g[:, 0]))


def first_auto_t(d: dict):
    md = d.get("mode")
    if md is None or not len(md):
        return None
    hit = md[md[:, 1] == 10.0]
    return float(hit[0, 0]) if len(hit) else None


def cross_time(t, y, level, i):
    y0, y1 = y[i], y[i + 1]
    if y1 == y0:
        return float(t[i])
    return float(t[i] + (level - y0) / (y1 - y0) * (t[i + 1] - t[i]))


def airborne_window(t, alt, level=AIRBORNE_M):
    above = alt > level
    if not above.any():
        return None, None
    up = np.flatnonzero((~above[:-1]) & above[1:])
    dn = np.flatnonzero(above[:-1] & (~above[1:]))
    t0 = cross_time(t, alt, level, int(up[0])) if len(up) else float(t[int(np.argmax(above))])
    t1 = cross_time(t, alt, level, int(dn[-1])) if len(dn) else float(t[-1])
    return t0, t1


def time_above(t, y, level):
    """Total time y > level, by linear interpolation of the level crossings."""
    a, b = y[:-1], y[1:]
    dt = np.diff(t)
    ok = dt > 0
    tot = 0.0
    both = ok & (a > level) & (b > level)
    tot += float(dt[both].sum())
    down = ok & (a > level) & (b <= level)
    tot += float((dt[down] * (a[down] - level) / (a[down] - b[down])).sum())
    up = ok & (a <= level) & (b > level)
    tot += float((dt[up] * (b[up] - level) / (b[up] - a[up])).sum())
    return tot


def path_len_enu(lat, lon, lat0, lon0) -> float:
    e, n = enu(lat, lon, lat0, lon0)
    return float(np.hypot(np.diff(e), np.diff(n)).sum())


def read_waypoints(p: Path):
    """QGC WPL 110 -> list of dicts. Columns: seq cur frame cmd p1..p4 x y z auto."""
    rows = []
    for ln in Path(p).read_text(encoding="utf-8").splitlines():
        if not ln.strip() or ln.startswith("#") or ln.startswith("QGC"):
            continue
        f = ln.split("\t")
        if len(f) < 12:
            f = ln.split()
        rows.append({"seq": int(f[0]), "frame": int(f[2]), "cmd": int(f[3]),
                     "lat": float(f[8]), "lon": float(f[9]), "alt": float(f[10])})
    return rows


def vec_to_from(vn: float, ve: float):
    """EKF wind state (vector the air moves TOWARD) -> (speed, FROM bearing deg)."""
    return math.hypot(vn, ve), math.degrees(math.atan2(-ve, -vn)) % 360.0
