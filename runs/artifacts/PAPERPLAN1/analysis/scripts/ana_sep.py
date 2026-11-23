# -*- coding: utf-8 -*-
"""Stage 2b: inter-aircraft separation, exact under a piecewise-linear track.

Improvement over both separation_from_tracks.json (2 s grid over ~1 Hz
telemetry; its own docstring calls it an upper bound) and binsep.py (native
6 Hz samples, but still evaluated only AT samples):

  * the two tracks are put on the UNION of their native POS times, so each is
    piecewise linear with breakpoints at every sample of either aircraft;
  * on each interval the relative position is linear in t, so the minimum of
    |dp| is solved in closed form.  Nothing is left between samples.

Three geodesy frames are carried because the repo disagrees with itself:
  repo     flat 111132 m/deg lat  -- hv_sitl_report.py / binsep.py, i.e. what
           every previously published flown separation used;
  planner  ellipsoidal series     -- hv_dccpp/scenario.py:92-93;
  ecef     exact WGS84 ECEF->ENU  -- hv_dccpp/geo.py GeoFrame, which
           scenario.py:87-88 states is what mission geometry actually uses,
           and therefore the frame in which the 100 m rule is enforced.

Symmetry note: hv_dccpp/sim.py:1007-1034 min_separation() evaluates the PLANNED
minimum on a grid of cfg.dt_4d_s = 2.0 s (hv_dccpp/mission.py:1092), so
validation.json's min_sep_h_m is itself a 2 s upper bound.  grid_cost_m below
measures what a 2 s grid costs on these very tracks, so the two sides can be
compared honestly.
"""
from __future__ import annotations

import itertools
import math

import numpy as np

from ana_lib import (AIRBORNE_M, D_SEP_H, D_SEP_V, OUT, ROOT, UAVS, gps_clock,
                     jdump, jload, load, m_per_deg)

TRIALS = ("HV4W55", "HV4W05", "HV4W00", "BEAMIDP1", "HV4SITL15", "HV4SITL16")
WGS84_A, WGS84_F = 6378137.0, 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def _ecef(lat, lon, h):
    p, l = np.radians(lat), np.radians(lon)
    n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * np.sin(p) ** 2)
    return np.stack([(n + h) * np.cos(p) * np.cos(l),
                     (n + h) * np.cos(p) * np.sin(l),
                     (n * (1.0 - WGS84_E2) + h) * np.sin(p)], axis=-1)


def ecef_enu(lat, lon, lat0, lon0):
    """Exact WGS84 ECEF -> local ENU (east, north) at the h = 0 datum."""
    d = _ecef(lat, lon, np.zeros_like(lat)) - _ecef(
        np.array([lat0]), np.array([lon0]), np.array([0.0]))
    p0, l0 = math.radians(lat0), math.radians(lon0)
    e = -math.sin(l0) * d[:, 0] + math.cos(l0) * d[:, 1]
    n = (-math.sin(p0) * math.cos(l0) * d[:, 0] - math.sin(p0) * math.sin(l0) * d[:, 1]
         + math.cos(p0) * d[:, 2])
    return e, n


def track(trial: str, uav: str, lat0: float, lon0: float, frame: str):
    d = load(trial, uav)
    p = d["pos"]
    off, _, _ = gps_clock(d)
    if frame == "ecef":
        x, y = ecef_enu(p[:, 1], p[:, 2], lat0, lon0)
    else:
        ml, mo = m_per_deg(lat0, frame)
        x, y = (p[:, 2] - lon0) * mo, (p[:, 1] - lat0) * ml
    return p[:, 0] + off, x, y, p[:, 4]


def _prep(t1, x1, y1, z1, t2, x2, y2, z2, grid_s: float | None = None):
    lo, hi = max(t1[0], t2[0]), min(t1[-1], t2[-1])
    if hi <= lo:
        return None
    if grid_s:
        tu = np.arange(lo, hi + 1e-9, grid_s)
    else:
        tu = np.union1d(t1[(t1 >= lo) & (t1 <= hi)], t2[(t2 >= lo) & (t2 <= hi)])
        tu = np.unique(np.concatenate([[lo], tu, [hi]]))
    a = [np.interp(tu, t1, v) for v in (x1, y1, z1)]
    b = [np.interp(tu, t2, v) for v in (x2, y2, z2)]
    return tu, a, b


def pair_min(t1, x1, y1, z1, t2, x2, y2, z2, grid_s=None, exact=True):
    pr = _prep(t1, x1, y1, z1, t2, x2, y2, z2, grid_s)
    if pr is None:
        return {"n_airborne_samples": 0, "min_h_m": None,
                "note": "no overlapping log span"}
    tu, a, b = pr
    air = (a[2] > AIRBORNE_M) & (b[2] > AIRBORNE_M)
    if air.sum() < 2:
        return {"n_airborne_samples": int(air.sum()), "min_h_m": None,
                "note": "never airborne simultaneously"}
    dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    best = {"h": math.inf}
    best_rule = {"h": math.inf}
    minv = {"v": math.inf}
    n_viol = 0
    idx = np.flatnonzero(air[:-1] & air[1:])
    for i in idx:
        p0 = np.array([dx[i], dy[i]])
        dd = np.array([dx[i + 1] - dx[i], dy[i + 1] - dy[i]])
        den = float(dd @ dd)
        cand = {0.0, 1.0}
        if exact and den > 0:
            cand.add(float(np.clip(-(p0 @ dd) / den, 0.0, 1.0)))
        for ss in cand:
            h = float(np.hypot(*(p0 + ss * dd)))
            v = float(abs(dz[i] + ss * (dz[i + 1] - dz[i])))
            tt = float(tu[i] + ss * (tu[i + 1] - tu[i]))
            if h < best["h"]:
                best = {"h": h, "v": v, "t": tt}
            if v < D_SEP_V and h < best_rule["h"]:
                best_rule = {"h": h, "v": v, "t": tt}
            if v < minv["v"]:
                minv = {"v": v, "h": h, "t": tt}
            if h < D_SEP_H and v < D_SEP_V:
                n_viol += 1
    return {
        "n_union_samples": int(len(tu)), "n_airborne_samples": int(air.sum()),
        "n_airborne_intervals": int(len(idx)),
        "min_h_m": round(best["h"], 4), "d_v_at_min_h_m": round(best["v"], 4),
        "t_gps_at_min_h_s": round(best["t"], 3),
        "min_h_while_v_under_50_m": (round(best_rule["h"], 4)
                                     if math.isfinite(best_rule["h"]) else None),
        "min_v_m": round(minv["v"], 4) if math.isfinite(minv["v"]) else None,
        "d_h_at_min_v_m": round(minv["h"], 4) if math.isfinite(minv["v"]) else None,
        "n_violations": int(n_viol),
        "margin_to_100m_m": round(best["h"] - D_SEP_H, 4),
    }


def main() -> None:
    res = {}
    for trial in TRIALS:
        ms = jload(ROOT / "runs" / "artifacts" / trial / "plan" / "mission_summary.json")
        lat0, lon0 = float(ms["home"]["lat_deg"]), float(ms["home"]["lon_deg"])
        val = jload(ROOT / "runs" / "artifacts" / trial / "validation.json")
        rep_f = ROOT / "runs" / "artifacts" / trial / "separation_from_tracks.json"
        rep = jload(rep_f) if rep_f.exists() else {"min_h_m": None}
        entry = {
            "planned_min_sep_h_m": val["min_sep_h_m"],
            "planned_source": f"runs/artifacts/{trial}/validation.json (2 s grid, "
                              f"hv_dccpp/sim.py:1007-1034 + mission.py:1092)",
            "reporter_min_h_m": rep["min_h_m"],
            "reporter_source": f"runs/artifacts/{trial}/separation_from_tracks.json "
                               f"(2 s grid over ~1 Hz telemetry; upper bound)",
            "frames": {}}
        for frame in ("repo", "planner", "ecef"):
            tr = {u: track(trial, u, lat0, lon0, frame) for u in UAVS}
            pairs = {f"{a}-{b}": pair_min(*tr[a], *tr[b])
                     for a, b in itertools.combinations(UAVS, 2)}
            cand = [(v["min_h_m"], k) for k, v in pairs.items()
                    if v.get("min_h_m") is not None]
            mn = min(cand) if cand else (None, None)
            entry["frames"][frame] = {
                "pairs": pairs, "fleet_min_h_m": mn[0], "binding_pair": mn[1],
                "n_violations": sum(v.get("n_violations", 0) for v in pairs.values())}
        bp = entry["frames"]["ecef"]["binding_pair"]
        if bp:
            a, b = bp.split("-")
            tr = {u: track(trial, u, lat0, lon0, "ecef") for u in UAVS}
            exact = entry["frames"]["ecef"]["pairs"][bp]["min_h_m"]
            gc = {}
            for g in (0.2, 0.5, 1.0, 2.0, 5.0):
                r = pair_min(*tr[a], *tr[b], grid_s=g, exact=False)
                gc[f"{g:g}s"] = {"sample_wise_min_h_m": r["min_h_m"],
                                 "overstates_by_m": round(r["min_h_m"] - exact, 4)}
            entry["grid_cost_ecef_frame"] = {
                "exact_min_h_m": exact, "binding_pair": bp, "by_grid": gc,
                "note": ("what a fixed time grid costs on these very tracks; the "
                         "planner's own min_sep_h_m uses a 2 s grid")}
            entry["native_sample_wise_ecef_m"] = pair_min(
                *tr[a], *tr[b], exact=False)["min_h_m"]
        res[trial] = entry
        rp = entry['reporter_min_h_m']
        print(f"{trial:9s} planned {entry['planned_min_sep_h_m']:8.3f} | reporter "
              f"{(f'{rp:7.2f}' if rp is not None else '     --')} | repo {entry['frames']['repo']['fleet_min_h_m']:8.3f} "
              f"| planner {entry['frames']['planner']['fleet_min_h_m']:8.3f} "
              f"| ecef {entry['frames']['ecef']['fleet_min_h_m']:8.3f} "
              f"[{entry['frames']['ecef']['binding_pair']}]")
    jdump(OUT / "separation.json", res)


if __name__ == "__main__":
    main()
