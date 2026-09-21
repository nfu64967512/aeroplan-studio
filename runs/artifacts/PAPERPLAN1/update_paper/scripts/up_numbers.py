# -*- coding: utf-8 -*-
"""label update_paper / stage 6 -- the single machine-readable number set.

Everything that appears in TABLES_ZH.md / PAPER_TEXT_ZH.md of THIS label is
emitted here with its provenance, so a reviewer can diff the prose against a
file rather than against a transcript.
"""
from __future__ import annotations

import json
import math

from up_lib import OUT, ROOT, UAVS, jdump, jload

ART = ROOT / "runs" / "artifacts"
M = jload(OUT / "metrics.json")
W = jload(OUT / "wind.json")
C = jload(OUT / "corrections.json")
S = jload(OUT / "spawn_verification.json")

ORDER = ["HV4SITL15", "HV4SITL16", "HV4W00", "HV4W05", "HV4W55", "HV4W55R",
         "HV4W55S", "BEAMIDP1"]


def wind_row(t: str) -> dict:
    """The signed kinematic wind test, on the axis the wind should show on."""
    sc = jload(ART / t / "scenario.json")
    sim = sc.get("sim_wind") or {}
    spd = float(sim.get("spd_mps", 0.0))
    dr = float(sim.get("dir_from_deg", 0.0))
    if spd <= 0.0:
        signal, null = "east_minus_west", "north_minus_south"
        expect = 0.0
    elif abs(((dr - 270.0) + 180) % 360 - 180) < 1.0:
        signal, null, expect = "east_minus_west", "north_minus_south", 2 * spd
    elif abs(((dr - 180.0) + 180) % 360 - 180) < 1.0:
        signal, null, expect = "north_minus_south", "east_minus_west", 2 * spd
    else:
        signal, null, expect = "east_minus_west", "north_minus_south", None
    out = {"sim_wind_spd_mps": spd, "sim_wind_dir_from_deg": dr,
           "plan_wind": sc.get("wind"),
           "signal_axis": signal, "null_axis": null, "expected_signal_mps": expect}
    for floor in ("100m", "30m"):
        r = W["floors"][floor][t]
        out[f"signal_{floor}"] = {u: r[u][signal] for u in UAVS}
        out[f"null_{floor}"] = {u: r[u][null] for u in UAVS}
        out[f"n_{floor}"] = {u: {k: r[u]["bins"][k]["n"] for k in ("E", "W", "N", "S")}
                             for u in UAVS}
    out["ekf_wind"] = W["ekf"][t]
    return out


def main() -> None:
    rows = {}
    for t in ORDER:
        m = M[t]
        up = jload(ART / t / "mission_upload.json")
        sc = jload(ART / t / "scenario.json")
        rows[t] = {
            "ceiling_m": sc.get("h_ceil_m"),
            "scenario_key": sc.get("key"),
            "plan_wind": sc.get("wind"),
            "sim_wind": sc.get("sim_wind"),
            "makespan": {"planned_s": round(m["timing"]["planned_makespan_s"], 4),
                         "flown_s": m["timing"]["flown_makespan_2m_s"],
                         "delta_s": m["timing"]["makespan_delta_s"],
                         "err_pct": m["timing"]["makespan_err_pct"]},
            "distance": {"planned_km": m["timing"]["planned_total_km"],
                         "flown_km": m["timing"]["flown_total_km"],
                         "delta_km": m["timing"]["total_dist_delta_km"],
                         "err_pct": m["timing"]["total_dist_err_pct"]},
            "gate_to_gate_err_pct": {u: C["gate_to_gate_prediction_error"][t]["per_uav"][u]["err_pct"]
                                     for u in UAVS},
            "gate_to_gate_range_pct": C["gate_to_gate_prediction_error"][t]["gate_err_range_pct"],
            "binding_uav": C["gate_to_gate_prediction_error"][t]["binding_uav"],
            "separation": {
                "planned_m": round(m["separation"]["planned_min_sep_h_m"], 4),
                "flown_exact_m": m["separation"]["frames"]["ecef"]["min_h_m"],
                "binding_pair": m["separation"]["frames"]["ecef"]["binding_pair"],
                "vert_at_min_m": m["separation"]["frames"]["ecef"]["pairs"][
                    m["separation"]["frames"]["ecef"]["binding_pair"]]["vert_at_min_m"],
                "margin_to_100_m": round(m["separation"]["frames"]["ecef"]["min_h_m"] - 100.0, 4),
                "plan_minus_flown_m": round(m["separation"]["planned_min_sep_h_m"]
                                            - m["separation"]["frames"]["ecef"]["min_h_m"], 4),
                "n_violations": m["separation"]["frames"]["ecef"]["n_violations"],
                "n_paired_samples": m["separation"]["frames"]["ecef"]["n_paired_samples"],
                "reporter_2s_grid_m": C["K1_separation_note_e"]["per_trial"][t]["reporter_2s_grid_m"],
                "binsep_native_flat_m": C["K1_separation_note_e"]["per_trial"][t]["binsep_native_flat_m"],
            },
            "altitude": {
                "exported_max_rel_m": m["altitude"]["fleet_exported_max_rel_m"],
                "exported_n_above_120": m["altitude"]["fleet_exported_n_above_120"],
                "exported_n_rel_items": m["altitude"]["fleet_exported_n_rel_items"],
                "flown_max_agl_m": m["altitude"]["fleet_flown_max_rel_m"],
                "flown_max_agl_per_uav": {u: m["altitude"]["per_uav"][u]["flown_max_rel_m"]
                                          for u in UAVS},
                "margin_to_120_m": m["altitude"]["fleet_margin_to_120_m"],
                "t_above_120_s": m["altitude"]["fleet_t_above_120_s"],
                "t_above_120_per_uav": {u: m["altitude"]["per_uav"][u]["t_above_120_s"]
                                        for u in UAVS},
                "tracking_excess_m": {u: m["altitude"]["per_uav"][u]["tracking_excess_m"]
                                      for u in UAVS},
            },
            "coverage": {"swath_m": m["coverage"]["swath_m"],
                         "areas": m["coverage"]["areas"]},
            "params": {"n_ok": m["params"]["n_ok"], "n_rows": m["params"]["n_rows"],
                       "strict_exact": m["params"]["strict_exact"],
                       "strict_float32": m["params"]["strict_float32"]},
            "items": {"per_uav": {u: m["timing"]["per_uav"][u]["plan_n_items"] for u in UAVS},
                      "upload_readback_ok": (up.get("ok") if isinstance(up, dict) else None)},
            "wind_test": wind_row(t),
        }
    res = {
        "generated_by": "runs/artifacts/PAPERPLAN1/update_paper/scripts/up_numbers.py",
        "inputs": ["out/metrics.json", "out/wind.json", "out/corrections.json",
                   "out/spawn_verification.json"],
        "conventions": {
            "separation_frame": "exact WGS84 geodetic -> ECEF -> local ENU; continuous-time "
                                "minimum on the union of both aircraft's native POS times",
            "makespan": "t0 = AUTO entry of the aircraft whose planned take-off delay is 0, "
                        "minus that delay; touchdown = last downward crossing of 2.0 m AGL",
            "gate_to_gate": "(flown land - flown release) / (plan land - plan take-off); "
                            "immune to the t0 convention AND to the ground wait",
            "coverage": "track buffered by swath/2, intersected with the survey polygon in "
                        "ENU -- an UPPER bound",
            "altitude": "POS.RelHomeAlt at native ~5.1-6.4 Hz; time above a level by linear "
                        "interpolation of the crossings, not by sample counting",
        },
        "trials": rows,
        "ceiling_control": C["ceiling_control_HV4W55_vs_HV4W55R"],
        "corrections": {k: C[k] for k in C if k.startswith("K")},
        "spawn_verification": S,
        "survey_geometry": C["survey_geometry"],
        "crosstrack_wind_triangle": W["crosstrack_180"],
    }
    jdump(OUT / "NUMBERS.json", res)
    print("wrote", OUT / "NUMBERS.json")
    for t in ORDER:
        r = rows[t]
        print(f"{t:10s} ceil {str(r['ceiling_m']):>5s} plan{str(r['plan_wind']):>14s} "
              f"sim {r['wind_test']['sim_wind_spd_mps']:.0f}@{r['wind_test']['sim_wind_dir_from_deg']:.0f} "
              f"| mk {r['makespan']['planned_s']:9.3f}->{r['makespan']['flown_s']:9.3f} "
              f"| km {r['distance']['planned_km']:7.3f}->{r['distance']['flown_km']:7.3f} "
              f"| sep {r['separation']['planned_m']:8.3f}->{r['separation']['flown_exact_m']:8.3f} "
              f"| cov {r['coverage']['areas'][0]['ratio']:.6f}")


if __name__ == "__main__":
    main()
