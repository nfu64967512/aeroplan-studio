# -*- coding: utf-8 -*-
"""Build this label's two summary records, every number recomputed from artifacts.

  spawn_record.json    per-aircraft commanded home / actual spawn / displacement
                       vector and bearing / pairwise spacing before and after /
                       settle behaviour, for every sortie that has a readback.
  ceiling_control.json HV4W55 (ceiling 119.5) vs HV4W55R (ceiling 120.0) under
                       identical wind: exported items above 120, flown time
                       above 120, maxima, makespan and distance.

    python make_records.py
"""
from __future__ import annotations

import json
import math
from pathlib import Path

ROOT = Path(r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
ART = ROOT / "runs" / "artifacts"
HERE = ART / "PAPERPLAN1" / "sorties2"
SORTIES = ART / "PAPERPLAN1" / "sorties"
M_LAT = 111132.0


def jl(p):
    return json.loads(Path(p).read_text(encoding="utf-8")) if Path(p).exists() else None


def pair_m(alat, alon, blat, blon):
    return math.hypot((alat - blat) * M_LAT,
                      (alon - blon) * 111412.0 * math.cos(math.radians(alat)))


def spawn_record():
    trials = ["HV4W00", "HV4W05", "HV4W55", "HV4W55R", "HV4W55S", "BEAMIDP1"]
    out = {"note": "displacement = actual spawn minus the --home handed to SITL; "
                   "bearing is degrees clockwise from north; pairwise spacing is "
                   "computed from the same coordinates, planned vs actual",
           "per_trial": {}}
    for t in trials:
        sr = jl(ART / t / "spawn_readback.json")
        if sr is None:
            continue
        sc = jl(ART / t / "scenario.json") or {}
        rows = []
        for r in sr["rows"]:
            dn = (r["got_lat"] - r["want_lat"]) * M_LAT
            de = ((r["got_lon"] - r["want_lon"]) * 111412.0
                  * math.cos(math.radians(r["want_lat"])))
            tr = [s for s in r.get("settle_trace", [])
                  if abs(s.get("d_north_m", 0.0)) < 1e4]
            rows.append({
                "uav": r["uav"], "takeoff_site": r["takeoff_site"],
                "commanded_home": [r["want_lat"], r["want_lon"]],
                "actual_spawn": [r["got_lat"], r["got_lon"]],
                "d_north_m": round(dn, 3), "d_east_m": round(de, 3),
                "displacement_m": round(math.hypot(dn, de), 3),
                "bearing_deg": round(math.degrees(math.atan2(de, dn)) % 360.0, 2),
                "settle_s": r["settle_s"],
                "settle_trace_samples": len(tr),
                "final_groundspeed_mps": (tr[-1].get("groundspeed_mps") if tr else None),
                "heading_span_deg": ([tr[0].get("heading_deg"), tr[-1].get("heading_deg")]
                                     if tr else None),
            })
        pairs = []
        for i, a in enumerate(sr["rows"]):
            for b in sr["rows"][i + 1:]:
                pw = pair_m(a["want_lat"], a["want_lon"], b["want_lat"], b["want_lon"])
                pg = pair_m(a["got_lat"], a["got_lon"], b["got_lat"], b["got_lon"])
                pairs.append({"pair": a["uav"] + "-" + b["uav"],
                              "planned_m": round(pw, 4), "actual_m": round(pg, 4),
                              "delta_m": round(pg - pw, 4)})
        out["per_trial"][t] = {
            "sim_wind": sc.get("sim_wind"), "plan_wind": sc.get("wind"),
            "spawn_spread_m": sr.get("spawn_spread_m"),
            "tol_m": sr.get("tol_m", 2.0),
            "tol_default_m": sr.get("tol_default_m", 2.0),
            "tol_overridden": sr.get("tol_overridden", False),
            "tol_source": sr.get("tol_source", "default (runner predates the override)"),
            "max_displacement_m": round(max(r["displacement_m"] for r in rows), 3),
            "rows": rows,
            "pairwise_spacing": pairs,
            "rigid_translation_max_abs_delta_m": round(
                max(abs(p["delta_m"]) for p in pairs), 4),
        }
    return out


def ceiling_control():
    ex = jl(HERE / "exported_alt.json")
    res = {"design": "single variable: HV4W55 and HV4W55R differ only in the planning "
                     "ceiling (119.5 vs 120.0 m). Plan wind 5 m/s @ 270 and SITL wind "
                     "5 m/s @ 270 in both; same scenario, same fleet, same spawn "
                     "stations, same exported item counts (82/53/84); the three "
                     "per-aircraft .param files are byte-identical between the two.",
           "per_trial": {}}
    for t, binlog in (("HV4W55", "HV4W55_recheck"), ("HV4W55R", "HV4W55R")):
        ba = jl(HERE / (binlog + "_binalt.json"))
        bs = jl(HERE / (t + "_binsep.json")) or jl(SORTIES / (t + "_binsep.json"))
        ms = jl(ART / t / "plan" / "mission_summary.json")
        fs = jl(ART / t / "flight_summary.json")
        sc = jl(ART / t / "scenario.json")
        val = jl(ART / t / "validation.json")
        per_alt = {u: {"flown_max_rel_alt_m": r["max_rel_home_alt_m"],
                       "flown_time_above_120_s": r["time_above_120_s"],
                       "pos_samples": r["n_pos"], "pos_rate_hz": r["pos_rate_hz"]}
                   for u, r in ba["per_uav"].items()}
        res["per_trial"][t] = {
            "ceiling_m": sc["h_ceil_m"], "coverage_altitude_m": sc["altitude_m"],
            "sim_wind": sc["sim_wind"], "plan_wind": sc["wind"],
            "dataflash_log": {u: r["log"] for u, r in ba["per_uav"].items()},
            "exported": ex[t],
            "flown_altitude": per_alt,
            "flown_max_rel_alt_m": ba["fleet_max_rel_alt_m"],
            "flown_time_above_120_s_fleet": round(
                sum(r["time_above_120_s"] for r in ba["per_uav"].values()), 2),
            "planned_makespan_s": ms["makespan_s"],
            "flown_makespan_s": max(r["flown_t_land_s"] for r in fs),
            "planned_total_km": round(sum(r["plan_km"] for r in fs), 3),
            "flown_total_km": round(sum(r["flown_km"] for r in fs), 3),
            "planned_min_sep_h_m": val["min_sep_h_m"],
            "flown_min_sep_h_m_dataflash": bs["min_h_m"] if bs else None,
        }
    a, b = res["per_trial"]["HV4W55"], res["per_trial"]["HV4W55R"]
    res["delta_120_minus_1195"] = {
        "exported_items_above_120": (b["exported"]["fleet_items_above_120"]
                                     - a["exported"]["fleet_items_above_120"]),
        "flown_time_above_120_s": round(b["flown_time_above_120_s_fleet"]
                                        - a["flown_time_above_120_s_fleet"], 2),
        "flown_max_rel_alt_m": round(b["flown_max_rel_alt_m"] - a["flown_max_rel_alt_m"], 4),
        "planned_makespan_s": round(b["planned_makespan_s"] - a["planned_makespan_s"], 4),
        "flown_makespan_s": round(b["flown_makespan_s"] - a["flown_makespan_s"], 2),
        "planned_total_km": round(b["planned_total_km"] - a["planned_total_km"], 3),
        "flown_total_km": round(b["flown_total_km"] - a["flown_total_km"], 3),
    }
    return res


def main():
    (HERE / "spawn_record.json").write_text(
        json.dumps(spawn_record(), ensure_ascii=False, indent=1), encoding="utf-8")
    (HERE / "ceiling_control.json").write_text(
        json.dumps(ceiling_control(), ensure_ascii=False, indent=1), encoding="utf-8")
    print("ok")


if __name__ == "__main__":
    main()
