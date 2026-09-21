"""C2 evidence: is there any wind in the existing SITL sorties?

Read-only. For every runs/artifacts/<TRIAL>/<UAV>/flight.csv it computes, over
samples that are airborne and in fixed-wing cruise:

    delta = groundspeed - airspeed          [m/s]

and bins them by track heading (from consecutive lat/lon).  With a steady wind
W from direction beta, delta ~= W*cos(theta - (beta+180)) ; so an EAST leg and a
WEST leg must differ by ~2W.  Zero wind => delta ~ 0 on every heading.

This is the *same* metric to run after the wind re-run, so the two are directly
comparable.  Usage:

    python runs/artifacts/PAPERPLAN1/c2_wind/wind_probe_from_flightcsv.py TRIAL [TRIAL...]
"""
from __future__ import annotations

import csv
import json
import math
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
ART = ROOT / "runs" / "artifacts"
M_LAT = 111132.0


def rows_of(p: Path):
    out = []
    for r in csv.DictReader(p.open(encoding="utf-8")):
        if not r["lat"] or not r["t_s"]:
            continue
        row = {"t": float(r["t_s"]), "lat": float(r["lat"]), "lon": float(r["lon"]),
               "alt": float(r["rel_alt_m"]), "as": float(r["airspeed"]),
               "gs": float(r["groundspeed"]), "mode": r["mode"]}
        # columns added by the C2 patch; absent in every pre-patch sortie
        for k, col in (("w_est", "wind_spd_est"), ("w_dir", "wind_dir_from_est")):
            v = (r.get(col) or "").strip()
            row[k] = float(v) if v else None
        out.append(row)
    return out


def analyse(trial: str) -> dict:
    d = ART / trial
    per = {}
    for sub in sorted(x for x in d.iterdir() if (x / "flight.csv").exists()):
        rs = rows_of(sub / "flight.csv")
        samples = []
        for a, b in zip(rs, rs[1:]):
            if a["alt"] < 30.0 or b["alt"] < 30.0:      # airborne + past VTOL transition
                continue
            if a["gs"] < 10.0 or a["as"] < 10.0:        # fixed-wing cruise only
                continue
            dt = b["t"] - a["t"]
            if not (0.1 < dt < 30.0):
                continue
            dy = (b["lat"] - a["lat"]) * M_LAT
            dx = (b["lon"] - a["lon"]) * 111412.0 * math.cos(math.radians(a["lat"]))
            if math.hypot(dx, dy) < 5.0:
                continue
            theta = math.degrees(math.atan2(dx, dy)) % 360.0   # track, deg from N cw
            samples.append((theta, a["gs"] - a["as"], a["gs"], a["as"],
                            a["w_est"], a["w_dir"]))
        if not samples:
            continue
        w_est = [s[4] for s in samples if s[4] is not None]
        w_dir = [s[5] for s in samples if s[5] is not None]
        def _bin(lo, hi):
            v = [s[1] for s in samples if lo <= s[0] < hi]
            return {"n": len(v), "median_gs_minus_as": round(statistics.median(v), 3)} if v else {"n": 0}
        east = _bin(45.0, 135.0)
        west = _bin(225.0, 315.0)
        ew = (None if not (east["n"] and west["n"])
              else round(east["median_gs_minus_as"] - west["median_gs_minus_as"], 3))
        per[sub.name] = {
            "n_cruise_samples": len(samples),
            "median_gs_minus_as_all": round(statistics.median([s[1] for s in samples]), 3),
            "max_abs_gs_minus_as": round(max(abs(s[1]) for s in samples), 3),
            "median_airspeed": round(statistics.median([s[3] for s in samples]), 2),
            "median_groundspeed": round(statistics.median([s[2] for s in samples]), 2),
            "eastbound_45_135": east, "westbound_225_315": west,
            "east_minus_west": ew,
            "implied_wind_along_EW_mps": None if ew is None else round(ew / 2.0, 3),
            # the flight controller's own EKF wind estimate (MAVLink WIND);
            # only present once the C2 patch adds the two columns.
            "fc_wind_est_median_mps": round(statistics.median(w_est), 3) if w_est else None,
            "fc_wind_dir_from_median_deg": round(statistics.median(w_dir), 1) if w_dir else None,
        }
    return {"trial": trial, "per_uav": per}


if __name__ == "__main__":
    trials = sys.argv[1:] or ["HV4SITL16"]
    res = [analyse(t) for t in trials]
    print(json.dumps(res, ensure_ascii=False, indent=1))
