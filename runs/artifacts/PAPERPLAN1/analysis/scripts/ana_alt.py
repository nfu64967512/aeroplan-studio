# -*- coding: utf-8 -*-
"""Stage 2d: altitude compliance under decision D2 (plan at a 119.5 m ceiling).

Exported maximum is re-parsed from plan/<uav>.waypoints column 11 (0-based 10),
skipping the lat=0 rows (HOME and the non-navigation items), i.e. the same rule
hv_sitl_report.py uses, but recomputed here.

Flown maximum comes from dataflash POS.RelHomeAlt (~6 Hz).  That includes the
two 120.0 m sorties HV4SITL15/16: their .BIN logs 00000013/00000014 are still on
disk and their mtimes match runs/artifacts/HV4SITL1{5,6}/A1/flight.csv to the
100 ns tick, which is what ties log to trial.  The 1 Hz flight.csv peak is kept
alongside so the cost of the slower stream is visible rather than assumed.
"""
from __future__ import annotations

import csv

import numpy as np

from ana_lib import (CEIL_LEGAL_M, CEIL_TARGET_M, OUT, ROOT, UAVS, jdump, jload,
                     load, time_above)

DATAFLASH = ("HV4W55", "HV4W05", "HV4W00", "BEAMIDP1", "HV4SITL15", "HV4SITL16")
CSV_ONLY = ()


def exported_max(trial: str, uav: str) -> dict:
    p = ROOT / "runs" / "artifacts" / trial / "plan" / f"{uav}.waypoints"
    alts, n = [], 0
    for ln in p.read_text(encoding="utf-8").splitlines()[1:]:
        if ln.startswith("#"):
            continue
        f = ln.split("\t")
        if len(f) >= 11 and float(f[8]) != 0.0:
            alts.append(float(f[10]))
            n += 1
    return {"max_exported_rel_alt_m": round(max(alts), 4),
            "n_nav_items": n,
            "n_items_above_120": int(sum(1 for a in alts if a > CEIL_TARGET_M)),
            "source": f"runs/artifacts/{trial}/plan/{uav}.waypoints col 11"}


def csv_alt(trial: str, uav: str) -> dict:
    p = ROOT / "runs" / "artifacts" / trial / uav / "flight.csv"
    rows = [r for r in csv.DictReader(p.open(encoding="utf-8")) if r["t_s"] and r["lat"]]
    t = np.array([float(r["t_s"]) for r in rows])
    a = np.array([float(r["rel_alt_m"]) for r in rows])
    return {"max_rel_alt_m_csv_1hz": round(float(a.max()), 4),
            "time_above_120_s_csv_1hz": round(time_above(t, a, CEIL_TARGET_M), 3),
            "time_above_121_92_s_csv_1hz": round(time_above(t, a, CEIL_LEGAL_M), 3),
            "median_dt_s": round(float(np.median(np.diff(t))), 3),
            "source": f"runs/artifacts/{trial}/{uav}/flight.csv"}


def main() -> None:
    res, miss = {}, []
    for trial in DATAFLASH + CSV_ONLY:
        run = ROOT / "runs" / "artifacts" / trial
        sc = jload(run / "scenario.json")
        entry = {"scenario_altitude_m": sc["altitude_m"],
                 "scenario_h_ceil_m": sc.get("h_ceil_m"),
                 "scenario_waiver_max_agl_m": sc.get("waiver_max_agl_m"),
                 "has_dataflash": trial in DATAFLASH, "per_uav": {}}
        for u in UAVS:
            e = exported_max(trial, u)
            c = csv_alt(trial, u)
            row = {**e, **c}
            row["exported_overshoot_vs_ceiling_m"] = round(
                e["max_exported_rel_alt_m"] - float(sc["altitude_m"]), 4)
            if trial in DATAFLASH:
                d = load(trial, u)["pos"]
                t, agl = d[:, 0], d[:, 4]
                mx = float(agl.max())
                row.update({
                    "max_rel_alt_m_flown": round(mx, 4),
                    "tracking_excess_vs_exported_m": round(mx - e["max_exported_rel_alt_m"], 4),
                    "margin_to_120_000_m": round(CEIL_TARGET_M - mx, 4),
                    "margin_to_121_92_m": round(CEIL_LEGAL_M - mx, 4),
                    "time_above_120_000_s": round(time_above(t, agl, CEIL_TARGET_M), 3),
                    "time_above_121_92_s": round(time_above(t, agl, CEIL_LEGAL_M), 3),
                    "peak_missed_by_1hz_m": round(mx - row["max_rel_alt_m_csv_1hz"], 4),
                    "flown_source": f"dataflash POS.RelHomeAlt, ~6 Hz"})
                miss.append(round(mx - row["max_rel_alt_m_csv_1hz"], 4))
            entry["per_uav"][u] = row
        entry["fleet_max_exported_m"] = max(
            r["max_exported_rel_alt_m"] for r in entry["per_uav"].values())
        if trial in DATAFLASH:
            entry["fleet_max_flown_m"] = max(
                r["max_rel_alt_m_flown"] for r in entry["per_uav"].values())
            entry["fleet_margin_to_120_m"] = round(
                CEIL_TARGET_M - entry["fleet_max_flown_m"], 4)
            entry["fleet_margin_to_121_92_m"] = round(
                CEIL_LEGAL_M - entry["fleet_max_flown_m"], 4)
        else:
            entry["fleet_max_flown_m_csv_1hz"] = max(
                r["max_rel_alt_m_csv_1hz"] for r in entry["per_uav"].values())
        res[trial] = entry
    band = {"n": len(miss), "min_m": min(miss), "max_m": max(miss),
            "median_m": round(float(np.median(miss)), 4),
            "note": ("peak that a 1 Hz flight.csv misses relative to the ~6 Hz "
                     "dataflash, measured on the 12 aircraft-sorties that have both; "
                     "negative means the two estimators differ, not that 1 Hz saw more")}
    for t in CSV_ONLY:
        res[t]["fleet_max_flown_corrected_upper_m"] = round(
            res[t]["fleet_max_flown_m_csv_1hz"] + band["max_m"], 4)
        res[t]["correction_band_applied"] = band
    res["_1hz_peak_miss_band"] = band
    jdump(OUT / "altitude.json", res)
    for t in DATAFLASH + CSV_ONLY:
        e = res[t]
        fl = e.get("fleet_max_flown_m") or e.get("fleet_max_flown_m_csv_1hz")
        print(f"{t:10s} ceil {e['scenario_altitude_m']:6.1f} exported {e['fleet_max_exported_m']:9.4f} "
              f"flown {fl:9.4f} margin120 {120.0 - fl:+.4f} "
              f"{'(1 Hz only)' if t in CSV_ONLY else ''}")
    print("1 Hz peak-miss band:", band)


if __name__ == "__main__":
    main()
