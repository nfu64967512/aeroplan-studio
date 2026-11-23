"""C2 evidence: planned vs flown for an EXISTING sortie, without touching its artifacts.

`scripts/hv_sitl_report.py` computes exactly these numbers but writes them back
into `runs/artifacts/<TRIAL>/`.  This is a read-only replica of its arithmetic
(`_clock_offset`, `_touchdown_s`, `_flown_km`, `_min_separation`) so the C2
baseline row can be quoted without modifying the committed evidence.

    python runs/artifacts/PAPERPLAN1/c2_wind/replay_planned_vs_flown.py HV4SITL16
"""
from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
ART = ROOT / "runs" / "artifacts"
M_LAT = 111132.0


def _rows(p: Path):
    return [r for r in csv.DictReader(p.open(encoding="utf-8")) if r["t_s"]]


def _clock_offset(out: Path, uavs) -> float:
    f = out / "release_schedule.json"
    if not f.exists():
        return 0.0
    rel = json.loads(f.read_text(encoding="utf-8"))
    sp = float(rel.get("speedup") or 1.0)
    act = rel.get("actual_release_wall_s") or {}
    offs = []
    for uav in uavs:
        if uav not in act:
            continue
        auto = [r for r in _rows(out / uav / "flight.csv") if r["mode"] == "AUTO"]
        if auto:
            offs.append(float(auto[0]["t_s"]) - float(act[uav]) * sp)
    if not offs:
        return 0.0
    offs.sort()
    return offs[len(offs) // 2]


def _touchdown(t, alt, armed) -> float:
    peak = 0.0
    for ts, a, ar in zip(t, alt, armed):
        peak = max(peak, a)
        if peak > 20.0 and a < 2.0 and ar is False:
            return ts
    return float("nan")


def _km(lon, lat) -> float:
    d = 0.0
    for i in range(1, len(lat)):
        dy = (lat[i] - lat[i - 1]) * M_LAT
        dx = (lon[i] - lon[i - 1]) * 111412.0 * math.cos(math.radians(lat[i]))
        d += math.hypot(dx, dy)
    return d / 1000.0


def main(trial: str) -> dict:
    out = ART / trial
    mission = json.loads((out / "plan" / "mission_summary.json").read_text(encoding="utf-8"))
    uavs = sorted(mission["uavs"])
    off = _clock_offset(out, uavs)
    rows, series = [], {}
    for uav in uavs:
        rs = [r for r in _rows(out / uav / "flight.csv") if r["lat"]]
        t = [float(r["t_s"]) - off for r in rs]
        lat = [float(r["lat"]) for r in rs]
        lon = [float(r["lon"]) for r in rs]
        alt = [float(r["rel_alt_m"]) for r in rs]
        armed = [r["armed"].strip().lower() in ("true", "1") for r in rs]
        series[uav] = (t, lat, lon, alt)
        td = _touchdown(t, alt, armed)
        air = [x for x, a in zip(t, alt) if a > 2.0]
        u = mission["uavs"][uav]
        rows.append({
            "uav": uav,
            "plan_t_takeoff_s": round(u["t_takeoff_s"], 1),
            "flown_t_takeoff_s": round(air[0], 1) if air else None,
            "plan_t_land_s": round(u["t_land_s"], 1),
            "flown_t_land_s": None if td != td else round(td, 1),
            "land_err_s": None if td != td else round(td - u["t_land_s"], 1),
            "plan_km": round(u["distance_m"] / 1000.0, 3),
            "flown_km": round(_km(lon, lat), 3),
            "km_err_pct": round(100.0 * (_km(lon, lat) - u["distance_m"] / 1000.0)
                                / max(u["distance_m"] / 1000.0, 1e-9), 2),
            "peak_alt_m": round(max(alt), 2),
        })
    plan_ms = max(r["plan_t_land_s"] for r in rows)
    flown_ms = max((r["flown_t_land_s"] or float("nan")) for r in rows)
    return {
        "trial": trial, "clock_offset_s": round(off, 1),
        "planned_makespan_s": plan_ms,
        "flown_makespan_s": None if flown_ms != flown_ms else round(flown_ms, 1),
        "makespan_err_s": None if flown_ms != flown_ms else round(flown_ms - plan_ms, 1),
        "makespan_err_pct": None if flown_ms != flown_ms
        else round(100.0 * (flown_ms - plan_ms) / plan_ms, 2),
        "planned_total_km": round(sum(r["plan_km"] for r in rows), 3),
        "flown_total_km": round(sum(r["flown_km"] for r in rows), 3),
        "total_km_err_pct": round(100.0 * (sum(r["flown_km"] for r in rows)
                                           - sum(r["plan_km"] for r in rows))
                                  / sum(r["plan_km"] for r in rows), 2),
        "peak_alt_max_m": max(r["peak_alt_m"] for r in rows),
        "per_uav": rows,
    }


if __name__ == "__main__":
    print(json.dumps([main(t) for t in (sys.argv[1:] or ["HV4SITL16"])],
                     ensure_ascii=False, indent=1))
