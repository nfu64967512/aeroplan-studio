# -*- coding: utf-8 -*-
"""Merge every per-sortie measurement into one machine-readable results file.

Sources, all produced by the runs themselves in this session:
  runs/artifacts/<TRIAL>/...                      the runner's own artifacts
  runs/artifacts/PAPERPLAN1/sorties/<T>_accept.json   recomputed gates (accept.py)
  runs/artifacts/PAPERPLAN1/sorties/<T>_binalt.json   full-rate altitude (binalt.py)
  runs/artifacts/PAPERPLAN1/sorties/<T>_binsep.json   full-rate separation (binsep.py)
  runs/artifacts/PAPERPLAN1/sorties/logs/<T>.t0/.t1   wall clock

Nothing here is copied from another agent's prose.  Planned values quoted for
comparison come from prep_runner/plan_matrix.json and are labelled as such.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

ROOT = Path(r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
S = ROOT / "runs/artifacts/PAPERPLAN1/sorties"
LOGS = S / "logs"

# Wall clock is taken from the .t0/.t1 epoch files each run wrote, never from
# prose.  BEAMIDP1 was flown in the interrupted session (its .t1 was never
# written); its fly duration is therefore measured to the mtime of
# release_schedule.json, the last file the fly step writes, and its report step
# was run separately in this session.
WALL_NOTE = {
    "BEAMIDP1": ("fly timed t0 -> mtime(release_schedule.json); report run "
                 "separately in a later session"),
}

# Why each sortie is or is not in the flown set.
STATUS = {
    "HV4W55":  ("flown_accepted", "all gates pass"),
    "HV4W05":  ("flown_accepted", "all gates pass"),
    "HV4W00":  ("flown_accepted", "all gates pass"),
    "HV4W55S": ("blocked_preflight",
                "G3 spawn gate: all 3 aircraft 5.01-5.12 m from their planned "
                "takeoff station (tol 2.0 m); runner SystemExit before arming. "
                "Reproduced bit-identically on a second independent run. Cause: "
                "wind from 180 deg is a pure TAILWIND on a north-facing parked "
                "aircraft, which rolls it ~5.05 m toward bearing ~44 deg before "
                "arming. Fleet translates rigidly: pairwise spacing preserved to "
                "0.02 m (10.01 -> 10.02 m), so the plan/SITL inconsistency G3 "
                "exists to catch is NOT present. Not relaxed on my own initiative."),
    "HV4W55R": ("not_flown",
                "optional, and its stated precondition (sorties 1-4 all pass) "
                "was not met because HV4W55S is blocked. Independently "
                "unblocked: its 270 deg wind spawns cleanly (0.13-0.21 m)."),
    "BEAMIDP1": ("flown_accepted",
                 "flown in the interrupted session; its report step was missing "
                 "and was completed here. All gates pass."),
}

MATRIX = {
    "HV4W55":  dict(order=1, cell="W55_c1195",  plan_wind=[5.0, 270.0], sim_wind=[5.0, 270.0], ceiling_m=119.5),
    "HV4W05":  dict(order=2, cell="W05_c1195",  plan_wind=[0.0, 270.0], sim_wind=[5.0, 270.0], ceiling_m=119.5),
    "HV4W00":  dict(order=3, cell="W00_c1195",  plan_wind=[0.0, 270.0], sim_wind=[0.0, 270.0], ceiling_m=119.5),
    "HV4W55S": dict(order=4, cell="W55S_c1195", plan_wind=[5.0, 180.0], sim_wind=[5.0, 180.0], ceiling_m=119.5),
    "HV4W55R": dict(order=5, cell="W55_c120",   plan_wind=[5.0, 270.0], sim_wind=[5.0, 270.0], ceiling_m=120.0),
    "BEAMIDP1": dict(order=6, cell="BEAMIDP1",  plan_wind=[5.0, 270.0], sim_wind=[0.0, 270.0], ceiling_m=119.5),
}


def load(p):
    return json.loads(p.read_text(encoding="utf-8")) if p.exists() else None


def main():
    pm = load(ROOT / "runs/artifacts/PAPERPLAN1/prep_runner/plan_matrix.json") or {}
    out = {
        "label": "sorties",
        "generated_by": "runs/artifacts/PAPERPLAN1/sorties/scripts/make_results.py",
        "note": ("Every number under `measured` was recomputed in this session from files the "
                 "runs produced.  `planned_reference` is prep_runner/plan_matrix.json, quoted "
                 "for comparison only.  Values marked ESTIMATE are estimates."),
        "sorties": {},
    }
    for trial, meta in sorted(MATRIX.items(), key=lambda kv: kv[1]["order"]):
        acc = load(S / f"{trial}_accept.json")
        binl = load(S / f"{trial}_binalt.json")
        rec = {"order": meta["order"], "trial": trial,
               "plan_wind_mps_dirfrom": meta["plan_wind"],
               "sim_wind_mps_dirfrom": meta["sim_wind"],
               "ceiling_m": meta["ceiling_m"],
               "flown": acc is not None and acc.get("gates", {}).get("G5_landed", {}).get("pass", False)}
        if meta["cell"] in pm:
            c = pm[meta["cell"]]
            rec["planned_reference"] = {"source": "prep_runner/plan_matrix.json",
                                        "makespan_s": c.get("makespan_s"),
                                        "min_sep_h_m": c.get("min_sep_h_m")}
        rec["status"], rec["status_reason"] = STATUS.get(trial, ("unknown", ""))
        wc = {}
        if (LOGS / f"{trial}.t0").exists():
            t0 = int((LOGS / f"{trial}.t0").read_text().strip())
            if (LOGS / f"{trial}.t1").exists():
                wc["first_attempt_total_s"] = int(
                    (LOGS / f"{trial}.t1").read_text().strip()) - t0
            elif trial == "BEAMIDP1":
                rs = ROOT / "runs/artifacts/BEAMIDP1/release_schedule.json"
                if rs.exists():
                    wc["fly_s"] = int(rs.stat().st_mtime) - t0
        if (LOGS / f"{trial}_rerun.t0").exists() and (LOGS / f"{trial}_rerun.t1").exists():
            wc["rerun_total_s"] = (int((LOGS / f"{trial}_rerun.t1").read_text().strip())
                                   - int((LOGS / f"{trial}_rerun.t0").read_text().strip()))
        if trial in WALL_NOTE:
            wc["note"] = WALL_NOTE[trial]
        if wc:
            rec["wall_clock_s"] = wc
        if acc:
            g = acc["gates"]
            m = acc["measured"]
            rec["gates"] = {k: v.get("pass") for k, v in g.items()}
            rec["all_gates_pass"] = acc.get("all_gates_pass")
            rec["measured"] = {
                "sim_wind_readback": g.get("G2_wind_readback", {}).get("commanded"),
                "wind_readback_rows_ok": (g.get("G2_wind_readback", {}).get("n_ok"),
                                          g.get("G2_wind_readback", {}).get("n_rows")),
                "min_groundspeed_actuals": g.get("G2_wind_readback", {}).get("min_groundspeed_actuals"),
                "params_readback": (g.get("G1_params", {}).get("n_ok"),
                                    g.get("G1_params", {}).get("n_rows")),
                "fence_alt_max": m.get("fence_alt_max"),
                "spawn_max_err_m": g.get("G3_spawn", {}).get("max_err_m"),
                "mission_items_read_back": g.get("G4_mission_items", {}).get("read_back"),
                "rtl_radius_m": g.get("H3_rtl_radius", {}).get("values"),
                "exported_max_rel_alt_m": {k: v["max_rel_alt_m"]
                                           for k, v in m.get("H1_exported_max_rel_alt_m", {}).items()},
                "exported_items_above_120": {k: v["n_items_above_120"]
                                             for k, v in m.get("H1_exported_max_rel_alt_m", {}).items()},
                "flown_max_rel_alt_m_flightcsv_1hz": g.get("H2_below_120", {}).get("per_uav"),
                "flight_csv_median_dt_s": {k: v["median_dt_s"] for k, v in
                                           g.get("G6_sampling", {}).get("per_uav", {}).items()},
                "planned": m.get("planned"),
                "flown_times_and_distance": m.get("flown"),
                "separation": g.get("S_separation"),
                "wind_probe_from_flightcsv": m.get("wind_probe"),
                "scenario_as_recorded": m.get("scenario"),
            }
        bins = load(S / f"{trial}_binsep.json")
        if bins:
            rec["measured"] = rec.get("measured", {})
            # The authoritative separation: every native POS sample of both
            # aircraft, aligned on GPS time-of-week.  The reporter's
            # separation_from_tracks.json is a 2.0 s interpolation and therefore
            # an UPPER bound -- both are carried so the gap is visible.
            rec["measured"]["separation_dataflash_fullrate"] = {
                "min_h_m": bins.get("min_h_m"), "pair": bins.get("pair"),
                "d_v_at_min_m": bins.get("d_v_at_min_m"),
                "n_violations": bins.get("n_violations"),
                "margin_above_100m_m": (None if bins.get("min_h_m") is None
                                        else round(bins["min_h_m"] - 100.0, 3)),
                "pairs": bins.get("pairs"),
                "pairs_never_concurrent": bins.get("pairs_never_concurrent"),
                "pos_median_dt_s": {k: v["median_dt_s"]
                                    for k, v in bins["per_uav"].items()},
                "rule": bins.get("rule"),
            }
        if binl:
            rec["measured"] = rec.get("measured", {})
            rec["measured"]["flown_max_rel_alt_m_dataflash"] = {
                k: v.get("max_rel_home_alt_m") for k, v in binl["per_uav"].items()}
            rec["measured"]["dataflash_pos_rate_hz"] = {
                k: v.get("pos_rate_hz") for k, v in binl["per_uav"].items()}
            rec["measured"]["time_above_120_s"] = {
                k: v.get("time_above_120_s") for k, v in binl["per_uav"].items()}
            rec["measured"]["ekf_wind_from_dataflash"] = {
                k: v.get("ekf_wind") for k, v in binl["per_uav"].items()}
            # ⚠ binalt.py's `mean_from_dir_deg` is an ARITHMETIC mean of a
            #   wrapping angle, and it includes the pre-takeoff samples where the
            #   estimate is ~0 m/s and its direction is random.  On HV4W55/HV4W05
            #   that drags A1 to 238.7/240.7 deg, which would fail the 270+/-20
            #   cross-check for no physical reason.  Derive the direction from the
            #   MEDIAN north/east components instead: no wrap, no zero-wind
            #   pollution.  Quote this field, not mean_from_dir_deg.
            der = {}
            for k, v in binl["per_uav"].items():
                w = (v or {}).get("ekf_wind") or {}
                vwn, vwe = w.get("median_VWN_mps"), w.get("median_VWE_mps")
                if vwn is None or vwe is None:
                    continue
                der[k] = {
                    "speed_mps": round(math.hypot(vwn, vwe), 3),
                    "from_dir_deg": round(math.degrees(math.atan2(-vwe, -vwn)) % 360.0, 1),
                    "median_VWN_mps": vwn, "median_VWE_mps": vwe,
                }
            rec["measured"]["ekf_wind_from_median_components"] = {
                "per_uav": der,
                "note": ("derived here from median VWN/VWE; supersedes "
                         "ekf_wind.mean_from_dir_deg, which is a circular-mean "
                         "artifact and must not be quoted"),
            }
            rec["measured"]["fleet_max_rel_alt_m_dataflash"] = binl.get("fleet_max_rel_alt_m")
            rec["measured"]["H2_below_120_dataflash_pass"] = binl.get("H2_below_120_pass")
        # If the spawn gate failed, compute WHAT moved, from the file itself:
        # a rigid translation of the whole fleet is a different failure from the
        # plan/SITL station mismatch G3 was written to catch.
        sp = load(ROOT / "runs/artifacts" / trial / "spawn_readback.json")
        if sp and not (acc or {}).get("gates", {}).get("G3_spawn", {}).get("pass", True):
            rows = sp["rows"]

            def _m(a_lat, a_lon, b_lat, b_lon):
                dy = (a_lat - b_lat) * 111132.0
                dx = (a_lon - b_lon) * 111412.0 * math.cos(math.radians(b_lat))
                return dx, dy

            disp = {}
            for r in rows:
                dx, dy = _m(r["got_lat"], r["got_lon"], r["want_lat"], r["want_lon"])
                disp[r["uav"]] = {"north_m": round(dy, 3), "east_m": round(dx, 3),
                                  "dist_m": round(math.hypot(dx, dy), 3),
                                  "bearing_deg": round(math.degrees(math.atan2(dx, dy)) % 360, 1),
                                  "settle_s": r["settle_s"]}
            gaps = {}
            for i, a in enumerate(rows):
                for b in rows[i + 1:]:
                    dxw, dyw = _m(a["want_lat"], a["want_lon"], b["want_lat"], b["want_lon"])
                    dxg, dyg = _m(a["got_lat"], a["got_lon"], b["got_lat"], b["got_lon"])
                    gaps[f'{a["uav"]}-{b["uav"]}'] = {
                        "planned_m": round(math.hypot(dxw, dyw), 3),
                        "actual_m": round(math.hypot(dxg, dyg), 3),
                        "delta_m": round(math.hypot(dxg, dyg) - math.hypot(dxw, dyw), 3)}
            rec["spawn_diagnosis"] = {
                "tol_m": sp.get("tol_m"), "displacement_from_commanded_home": disp,
                "pairwise_spacing": gaps,
                "reading": ("whole fleet translated rigidly; pairwise spacing "
                            "preserved, so the plan-vs-SITL station mismatch that "
                            "G3 exists to catch is NOT what failed here"),
                "reproduced": True,
            }
        out["sorties"][trial] = rec

    (S / "sortie_results.json").write_text(
        json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    print("wrote", S / "sortie_results.json")


if __name__ == "__main__":
    main()
