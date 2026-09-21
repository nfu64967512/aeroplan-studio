# -*- coding: utf-8 -*-
"""SIGNED wind acceptance on BOTH axes (north-south as well as east-west).

Why this exists: the C2 probe (runs/artifacts/PAPERPLAN1/c2_wind/
wind_probe_from_flightcsv.py) only bins EAST vs WEST, because every sortie so
far blew from 270 deg and the survey strips run east-west, so the along-strip
legs carried the whole signal.  A 5 m/s wind FROM 180 deg is a pure CROSSWIND on
those strips: its east-minus-west asymmetry is ~0 BY CONSTRUCTION, and "~0"
is exactly what "the wind never reached the simulator" also looks like.  So for
the 180 deg cell the acceptance rule has to be read on the north-south legs.

Metric, filters and binning are copied from the C2 probe so the numbers are
directly comparable:  delta = groundspeed - airspeed, airborne (rel_alt >= 30 m),
fixed-wing cruise (gs and as >= 10 m/s), consecutive samples >= 5 m apart,
track from consecutive lat/lon.  Bins are 90 deg wide and centred on the axis.

With a steady wind of speed W FROM direction beta, delta = W*cos(theta - (beta+180)).
  beta = 270 (from west, air moves EAST):  east - west = +2W,  north - south ~ 0
  beta = 180 (from south, air moves NORTH): north - south = +2W, east - west ~ 0
The constant airspeed/density bias rides on BOTH bins and cancels in the
difference, which is why the acceptance quantity is the difference and not the
level.  A sign flip (-2W) means the direction convention is wrong: do NOT
"fix" it by flipping SIM_WIND_DIR (RUNBOOK section 8).

    python windaxis.py TRIAL [TRIAL ...]

Writes <TRIAL>_windaxis.json into this label's evidence folder. Read-only on
everything else.
"""
from __future__ import annotations

import csv
import json
import math
import statistics
import sys
from pathlib import Path

ROOT = Path(r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
ART = ROOT / "runs" / "artifacts"
OUT = ART / "PAPERPLAN1" / "sorties2"
M_LAT = 111132.0


def samples_of(p: Path):
    rs = []
    for r in csv.DictReader(p.open(encoding="utf-8")):
        if not r["lat"] or not r["t_s"]:
            continue
        rs.append({"t": float(r["t_s"]), "lat": float(r["lat"]), "lon": float(r["lon"]),
                   "alt": float(r["rel_alt_m"]), "as": float(r["airspeed"]),
                   "gs": float(r["groundspeed"])})
    out = []
    for a, b in zip(rs, rs[1:]):
        if a["alt"] < 30.0 or b["alt"] < 30.0:
            continue
        if a["gs"] < 10.0 or a["as"] < 10.0:
            continue
        dt = b["t"] - a["t"]
        if not (0.1 < dt < 30.0):
            continue
        dy = (b["lat"] - a["lat"]) * M_LAT
        dx = (b["lon"] - a["lon"]) * 111412.0 * math.cos(math.radians(a["lat"]))
        if math.hypot(dx, dy) < 5.0:
            continue
        out.append((math.degrees(math.atan2(dx, dy)) % 360.0, a["gs"] - a["as"]))
    return out


def _bin(sm, lo, hi):
    if lo < hi:
        v = [d for th, d in sm if lo <= th < hi]
    else:                                   # wraps through 0
        v = [d for th, d in sm if th >= lo or th < hi]
    return {"n": len(v),
            "median_gs_minus_as": round(statistics.median(v), 3) if v else None}


def main():
    res = {}
    for trial in sys.argv[1:]:
        d = ART / trial
        per = {}
        for sub in sorted(x for x in d.iterdir() if (x / "flight.csv").exists()):
            sm = samples_of(sub / "flight.csv")
            if not sm:
                continue
            e, w = _bin(sm, 45.0, 135.0), _bin(sm, 225.0, 315.0)
            n, s = _bin(sm, 315.0, 45.0), _bin(sm, 135.0, 225.0)
            def diff(a, b):
                if a["n"] and b["n"]:
                    return round(a["median_gs_minus_as"] - b["median_gs_minus_as"], 3)
                return None
            per[sub.name] = {
                "n_cruise_samples": len(sm),
                "eastbound_45_135": e, "westbound_225_315": w,
                "northbound_315_45": n, "southbound_135_225": s,
                "east_minus_west": diff(e, w),
                "north_minus_south": diff(n, s),
            }
        res[trial] = per
        OUT.mkdir(parents=True, exist_ok=True)
        (OUT / f"{trial}_windaxis.json").write_text(
            json.dumps({"trial": trial, "per_uav": per}, ensure_ascii=False, indent=1),
            encoding="utf-8")
    print(json.dumps(res, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
