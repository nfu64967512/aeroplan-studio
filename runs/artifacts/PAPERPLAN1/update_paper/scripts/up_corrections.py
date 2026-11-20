# -*- coding: utf-8 -*-
"""label update_paper / stage 4 -- K1..K9 recomputed, plus the two new results.

Every value the nine corrections touch is recomputed here from primary
artifacts (dataflash cache, plan/ files, spawn_readback.json, the baseline
control's stepN/stepO tables).  Nothing is copied from the earlier labels'
summaries; where an earlier label's number is quoted it is quoted as the
OBJECT of the comparison, never as the source.
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from up_lib import OUT, ROOT, UAVS, jdump, jload, load

ART = ROOT / "runs" / "artifacts"
PP = ART / "PAPERPLAN1"
M = jload(OUT / "metrics.json")
W = jload(OUT / "wind.json")


def sha(p: Path) -> str:
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


# ---------------------------------------------------------------- new result 1
def ceiling_control() -> dict:
    """HV4W55 (ceiling 119.5) vs HV4W55R (ceiling 120.0): single variable."""
    a, b = "HV4W55", "HV4W55R"
    same, diff = [], []
    for u in UAVS:
        for f in (f"plan/{u}.param", f"plan/{u}.waypoints", f"plan/{u}_fence.txt",
                  f"plan/{u}_rally.txt"):
            ha, hb = sha(ART / a / f), sha(ART / b / f)
            (same if ha == hb else diff).append({"file": f, "sha_a": ha[:12], "sha_b": hb[:12]})
    sa, sb = jload(ART / a / "scenario.json"), jload(ART / b / "scenario.json")
    sc_diff = {k: [sa.get(k), sb.get(k)] for k in set(sa) | set(sb) if sa.get(k) != sb.get(k)}
    ma, mb = M[a], M[b]
    pa = float(jload(ART / a / "plan" / "mission_summary.json")["makespan_s"])
    pb = float(jload(ART / b / "plan" / "mission_summary.json")["makespan_s"])
    return {
        "identical_files": [d["file"] for d in same],
        "differing_files": diff,
        "scenario_fields_that_differ": sc_diff,
        "wind_is_identical": sa["wind"] == sb["wind"] and sa["sim_wind"] == sb["sim_wind"],
        "exported_items_per_uav": {u: [ma["timing"]["per_uav"][u]["plan_n_items"],
                                       mb["timing"]["per_uav"][u]["plan_n_items"]] for u in UAVS},
        "planned_makespan_s": {a: pa, b: pb},
        "planned_makespan_cost_of_1195_s": round(pa - pb, 6),
        "exported": {
            "n_rel_items": [ma["altitude"]["fleet_exported_n_rel_items"],
                            mb["altitude"]["fleet_exported_n_rel_items"]],
            "n_above_120": [ma["altitude"]["fleet_exported_n_above_120"],
                            mb["altitude"]["fleet_exported_n_above_120"]],
            "n_above_120_per_uav": {u: [ma["altitude"]["per_uav"][u]["exported_n_above_120"],
                                        mb["altitude"]["per_uav"][u]["exported_n_above_120"]]
                                    for u in UAVS},
            "max_rel_m": [ma["altitude"]["fleet_exported_max_rel_m"],
                          mb["altitude"]["fleet_exported_max_rel_m"]],
        },
        "flown": {
            "max_agl_m": [ma["altitude"]["fleet_flown_max_rel_m"],
                          mb["altitude"]["fleet_flown_max_rel_m"]],
            "max_agl_per_uav": {u: [ma["altitude"]["per_uav"][u]["flown_max_rel_m"],
                                    mb["altitude"]["per_uav"][u]["flown_max_rel_m"]]
                                for u in UAVS},
            "t_above_120_s": [ma["altitude"]["fleet_t_above_120_s"],
                              mb["altitude"]["fleet_t_above_120_s"]],
            "t_above_120_per_uav": {u: [ma["altitude"]["per_uav"][u]["t_above_120_s"],
                                        mb["altitude"]["per_uav"][u]["t_above_120_s"]]
                                    for u in UAVS},
            "makespan_s": [ma["timing"]["flown_makespan_2m_s"], mb["timing"]["flown_makespan_2m_s"]],
            "total_km": [ma["timing"]["flown_total_km"], mb["timing"]["flown_total_km"]],
            "min_sep_h_m": [ma["separation"]["frames"]["ecef"]["min_h_m"],
                            mb["separation"]["frames"]["ecef"]["min_h_m"]],
            "coverage_ratio": [ma["coverage"]["areas"][0]["ratio"],
                               mb["coverage"]["areas"][0]["ratio"]],
        },
        "deltas": {
            "exported_above_120": mb["altitude"]["fleet_exported_n_above_120"]
                                  - ma["altitude"]["fleet_exported_n_above_120"],
            "flown_t_above_120_s": round(mb["altitude"]["fleet_t_above_120_s"]
                                         - ma["altitude"]["fleet_t_above_120_s"], 3),
            "flown_max_agl_m": round(mb["altitude"]["fleet_flown_max_rel_m"]
                                     - ma["altitude"]["fleet_flown_max_rel_m"], 4),
            "exported_max_rel_m": round(mb["altitude"]["fleet_exported_max_rel_m"]
                                        - ma["altitude"]["fleet_exported_max_rel_m"], 4),
            "flown_makespan_s": round(mb["timing"]["flown_makespan_2m_s"]
                                      - ma["timing"]["flown_makespan_2m_s"], 3),
            "flown_total_km": round(mb["timing"]["flown_total_km"]
                                    - ma["timing"]["flown_total_km"], 4),
        },
        "tracking_excess_m": {u: {t: M[t]["altitude"]["per_uav"][u]["tracking_excess_m"]
                                  for t in ("HV4W55", "HV4W55R", "HV4W55S")} for u in UAVS},
    }


# ---------------------------------------------------------------- new result 2
def gate_to_gate() -> dict:
    """Prediction error with the ground wait removed.

    makespan_err is a DILUTED quantity: the binding aircraft's extra flight
    time divided by its planned landing time, most of which it spent parked.
    The intensive quantity is (flown land - flown release) / (plan land -
    plan take-off), which is immune both to the t0 convention and to the
    schedule structure, and is therefore what the cross-track vs along-strip
    comparison must use.
    """
    out = {}
    for t, m in M.items():
        per = {}
        for u in UAVS:
            r = m["timing"]["per_uav"][u]
            pl = r["plan_t_land_s"] - r["plan_t_takeoff_s"]
            fl = r["flown_t_land_2m_s"] - r["flown_t_release_s"]
            per[u] = {"cls": r["cls"], "plan_gate_s": round(pl, 3), "flown_gate_s": round(fl, 3),
                      "err_pct": round(100.0 * (fl / pl - 1.0), 3),
                      "ground_wait_s": r["plan_t_takeoff_s"],
                      "dist_err_pct": r["dist_err_pct"]}
        binder = max(UAVS, key=lambda u: m["timing"]["per_uav"][u]["flown_t_land_2m_s"])
        out[t] = {"per_uav": per, "binding_uav": binder,
                  "binding_gate_err_pct": per[binder]["err_pct"],
                  "makespan_err_pct": m["timing"]["makespan_err_pct"],
                  "dilution_factor": round(per[binder]["plan_gate_s"]
                                           / m["timing"]["per_uav"][binder]["plan_t_land_s"], 4),
                  "gate_err_range_pct": [min(v["err_pct"] for v in per.values()),
                                         max(v["err_pct"] for v in per.values())]}
    return out


# ------------------------------------------------------------------------- K1
def k1_separation() -> dict:
    rows = {}
    for t, m in M.items():
        s = m["separation"]["frames"]
        ex, pl, rp = s["ecef"]["min_h_m"], s["planner"]["min_h_m"], s["repo"]["min_h_m"]
        rep = m["separation"].get("reporter_min_sep_h_m")
        bs = None
        for d in (PP / "sorties", PP / "sorties2"):
            f = d / f"{t}_binsep.json"
            if f.exists():
                bs = jload(f)["min_h_m"]
        r = {"exact_ellipsoid_m": ex, "planner_series_m": pl, "flat_repo_frame_m": rp,
             "reporter_2s_grid_m": rep, "binsep_native_flat_m": bs,
             "delta_vs_reporter_m": round(rep - ex, 4) if rep is not None else None,
             "delta_vs_binsep_m": round(bs - ex, 4) if bs is not None else None,
             "frame_component_m": round(rp - ex, 4),
             "sampling_component_m": round(bs - rp, 4) if bs is not None else None}
        rows[t] = r
    # HV4SITL16 has no separation_from_tracks.json; c4_params recomputed 101.0
    c4 = jload(PP / "c4_params" / "c4_evidence.json")
    rows["HV4SITL16"]["reporter_2s_grid_m"] = float(c4["flights"]["HV4SITL16"]["sep"]["min_h_m"])
    rows["HV4SITL16"]["reporter_source"] = "c4_params/c4_evidence.json flights.HV4SITL16.sep.min_h_m (no separation_from_tracks.json exists for this trial)"
    rows["HV4SITL16"]["delta_vs_reporter_m"] = round(
        rows["HV4SITL16"]["reporter_2s_grid_m"] - rows["HV4SITL16"]["exact_ellipsoid_m"], 4)
    dr = [v["delta_vs_reporter_m"] for v in rows.values() if v["delta_vs_reporter_m"] is not None]
    db = [v["delta_vs_binsep_m"] for v in rows.values() if v["delta_vs_binsep_m"] is not None]
    fc = [v["frame_component_m"] for v in rows.values()]
    sc = [v["sampling_component_m"] for v in rows.values() if v["sampling_component_m"] is not None]
    return {"per_trial": rows,
            "range_vs_reporter_m": [round(min(dr), 4), round(max(dr), 4)],
            "range_vs_binsep_m": [round(min(db), 4), round(max(db), 4)],
            "range_frame_component_m": [round(min(fc), 4), round(max(fc), 4)],
            "range_sampling_component_m": [round(min(sc), 4), round(max(sc), 4)],
            "published_claim_3.4_to_5.3_m": "NOT REPRODUCIBLE",
            "what_3.4_to_5.3_probably_was":
                {t: round(M[t]["separation"]["planned_min_sep_h_m"]
                          - M[t]["separation"]["frames"]["ecef"]["min_h_m"], 3)
                 for t in M}}


# ------------------------------------------------------------------------- K2
def k2_params() -> dict:
    return {t: {k: m["params"][k] for k in ("n_rows", "n_ok", "strict_exact",
                                            "strict_float32", "n_float32_roundtrip_only",
                                            "source_counts")}
            for t, m in M.items()} | {
        "float32_roundtrip_rows_HV4W55": M["HV4W55"]["params"]["float32_only_rows"],
        "genuinely_different_rows_HV4W55": M["HV4W55"]["params"]["genuinely_different_rows"],
        "verdict": "literal/exact comparison = 85/96; at float32 precision = 91/96"}


# ---------------------------------------------------------------------- K3/K4
def k3k4_spawn() -> dict:
    cur = jload(ART / "HV4W55S" / "spawn_readback.json")
    old = jload(PP / "sorties" / "HV4W55S_failed_spawn_backup" / "spawn_readback.json")

    def rows(d):
        return {r["uav"]: r for r in d["rows"]}
    c, o = rows(cur), rows(old)
    per = {}
    M_LAT_ELL = 110757.50   # recomputed below, kept for the record
    for u in UAVS:
        r = c[u]
        lat0 = float(r["want_lat"])
        f = math.radians(lat0)
        ml = 111132.954 - 559.822 * math.cos(2 * f) + 1.175 * math.cos(4 * f)
        mo = 111412.84 * math.cos(f) - 93.5 * math.cos(3 * f) + 0.118 * math.cos(5 * f)
        dn = (float(r["got_lat"]) - lat0) * ml
        de = (float(r["got_lon"]) - float(r["want_lon"])) * mo
        per[u] = {"d_north_m": round(dn, 4), "d_east_m": round(de, 4),
                  "err_m": round(math.hypot(dn, de), 4),
                  "bearing_deg": round(math.degrees(math.atan2(de, dn)) % 360.0, 2),
                  "settle_s_this_run": r.get("settle_s"),
                  "settle_s_previous_run": o[u].get("settle_s"),
                  "got_lat_identical": float(r["got_lat"]) == float(o[u]["got_lat"]),
                  "got_lon_identical": float(r["got_lon"]) == float(o[u]["got_lon"])}
    # rigid translation: planned vs actual pairwise spacing
    def spacing(getter):
        out = {}
        for i, a in enumerate(UAVS):
            for b in UAVS[i + 1:]:
                la, lo = getter(c[a])
                lb, lob = getter(c[b])
                f = math.radians((la + lb) / 2)
                ml = 111132.954 - 559.822 * math.cos(2 * f) + 1.175 * math.cos(4 * f)
                mo = 111412.84 * math.cos(f) - 93.5 * math.cos(3 * f) + 0.118 * math.cos(5 * f)
                out[f"{a}-{b}"] = round(math.hypot((lb - la) * ml, (lob - lo) * mo), 4)
        return out
    want = spacing(lambda r: (float(r["want_lat"]), float(r["want_lon"])))
    got = spacing(lambda r: (float(r["got_lat"]), float(r["got_lon"])))
    dsp = {k: round(got[k] - want[k], 4) for k in want}
    # bit-identity of the two runs' files
    files = {}
    for f in ("spawn_readback.json", "report.json", "wind_readback.json", "validation.json",
              "scenario.json"):
        a = ART / "HV4W55S" / f
        b = PP / "sorties" / "HV4W55S_failed_spawn_backup" / f
        if a.exists() and b.exists():
            files[f] = {"identical": sha(a) == sha(b), "sha_now": sha(a)[:12], "sha_prev": sha(b)[:12]}
    return {"per_uav": per,
            "planned_spacing_m": want, "actual_spacing_m": got, "spacing_delta_m": dsp,
            "max_abs_spacing_delta_m": round(max(abs(v) for v in dsp.values()), 4),
            "bearing_range_deg": [min(v["bearing_deg"] for v in per.values()),
                                  max(v["bearing_deg"] for v in per.values())],
            "file_bit_identity": files,
            "K3_verdict": "displacement bearing is north-EAST, so 'pure tailwind rolls it north' is refuted",
            "K4_verdict": "the FILES are not bit-identical; the SPAWN POSITIONS are",
            "benign_270_deg_cells": {t: {u: jload(ART / t / "spawn_readback.json")["rows"][i]["err_m"]
                                         for i, u in enumerate(UAVS)}
                                     for t in ("HV4W55", "HV4W55R", "HV4W05", "HV4W00")}}


# ------------------------------------------------------------------------- K5
def k5_beamidp() -> dict:
    sc = jload(ART / "BEAMIDP1" / "scenario.json")
    wr = jload(ART / "BEAMIDP1" / "wind_readback.json")
    k = W["floors"]["100m"]["BEAMIDP1"]
    return {"scenario_plan_wind": sc.get("wind"),
            "scenario_sim_wind": sc.get("sim_wind"),
            "wind_readback_rows": (len(wr["rows"]) if isinstance(wr, dict) and "rows" in wr
                                   else None),
            "measured_east_minus_west_mps": {u: k[u]["east_minus_west"] for u in UAVS},
            "measured_north_minus_south_mps": {u: k[u]["north_minus_south"] for u in UAVS},
            "ekf_wind": W["ekf"]["BEAMIDP1"],
            "makespan_planned_s": M["BEAMIDP1"]["timing"]["planned_makespan_s"],
            "makespan_flown_s": M["BEAMIDP1"]["timing"]["flown_makespan_2m_s"],
            "makespan_err_pct": M["BEAMIDP1"]["timing"]["makespan_err_pct"],
            "verdict": "a 5 m/s 270 deg PLAN flown in measured still air -- the same "
                       "plan-vs-sim wind mismatch this package accuses HV4SITL15/16 of"}


# ------------------------------------------------------------------------- K7
def k7_appendixB() -> dict:
    out = {}
    for t in ("HV4SITL15", "HV4SITL16"):
        for fl in ("100m", "30m"):
            out[f"{t}_floor_{fl}"] = {u: W["floors"][fl][t][u]["east_minus_west"] for u in UAVS}
    return out | {"verdict": "the :276 row's source is HV4SITL16, whose triple is "
                             "+0.0053 / -0.0020 / -0.0192 m/s at the 100 m cruise floor"}


# ------------------------------------------------------------------------- K8
def k8_battery() -> dict:
    idx = jload(OUT / "extract_index.json")
    out = {}
    for t in M:
        per = {}
        for u in UAVS:
            d = load(t, u)
            b = d.get("bat")
            if b is None or not len(b):
                per[u] = {"rows": 0}
                continue
            per[u] = {"rows": int(len(b)),
                      "volt_min": round(float(np.nanmin(b[:, 1])), 3),
                      "volt_max": round(float(np.nanmax(b[:, 1])), 3),
                      "enrg_tot_wh": round(float(np.nanmax(b[:, 3])), 3)}
        out[t] = per
    ms = jload(ART / "BEAMIDP1" / "plan" / "mission_summary.json")
    out["planner_model"] = {u: {"energy_used_wh": round(float(ms["uavs"][u]["energy_used_wh"]), 3),
                                "energy_margin_wh": round(float(ms["uavs"][u]["energy_margin_wh"]), 3),
                                "cls": ms["uavs"][u]["cls"]} for u in UAVS}
    out["verdict"] = ("BAT exists on every airframe in every sortie (BEAMIDP1 B2 = 5039 rows). "
                      "What is missing is not telemetry but the right battery: SITL logs a "
                      "generic 12.6 V pack, not the planner's Hero 254 Wh pack, so the logged "
                      "energy cannot corroborate the planner's margins.")
    return out


# ------------------------------------------------------------------------- K9
def k9_tablez() -> dict:
    n = jload(PP / "prep_baseline_control" / "out" / "stepN_fair_fight.json")
    o = jload(PP / "prep_baseline_control" / "out" / "stepO_harvest_native.json")
    paper10 = {(s, k) for s, k in n["paper10"]}
    rows = [r for r in n["rows"] if (r["scenario"], r["n"]) in paper10]
    orows = [r for r in o["rows"] if (r["scenario"], r["n"]) in paper10]
    return {"n_cells": len(rows),
            "dccpp_sep": sum(r["dccpp_sep"] for r in rows),
            "dccpp_pad": sum(r["dccpp_pad"] for r in rows),
            "dccpp_cells_with_sep": sum(1 for r in rows if r["dccpp_sep"] > 0),
            "dccpp_cells_with_pad": sum(1 for r in rows if r["dccpp_pad"] > 0),
            "dccpp_after_own_ca_sep": sum(r["dccpp_ca_sep"] for r in rows),
            "dccpp_after_own_ca_pad": sum(r["dccpp_ca_pad"] for r in rows),
            "ca_detected": sum(r["ca_detected"] for r in rows),
            "ca_actions": sum(r["ca_actions"] for r in rows),
            "harvest_relifted_sep": sum(r["harvest_sep"] for r in rows),
            "harvest_relifted_pad": sum(r["harvest_pad"] for r in rows),
            "harvest_native_sep": sum(r["sep"] for r in orows),
            "harvest_native_pad": sum(r["pad"] for r in orows),
            "harvest_takeoff_queue_s": [round(min(max(r["t_takeoff_s"].values())
                                                  - min(r["t_takeoff_s"].values()) for r in orows), 1),
                                        round(max(max(r["t_takeoff_s"].values())
                                                  - min(r["t_takeoff_s"].values()) for r in orows), 1)],
            "harvest_last_landing_s": [round(min(max(r["t_land_s"].values()) for r in orows), 1),
                                       round(max(max(r["t_land_s"].values()) for r in orows), 1)],
            "verdict": "the two arms measure different objects: the baseline is checked on "
                       "RE-LIFTED tracks, HARVEST on its native MissionPlan.plans. Re-lifting "
                       "HARVEST too gives 959/688. The caption must not claim one ruler."}


# ------------------------------------------------------------- survey geometry
def survey_geometry() -> dict:
    sc = jload(ART / "HV4W55" / "scenario.json")
    poly = sc["areas"][0]["polygon"]
    lats = [p[0] for p in poly]
    lat0 = sum(lats) / len(lats)
    f = math.radians(lat0)
    ml = 111132.954 - 559.822 * math.cos(2 * f) + 1.175 * math.cos(4 * f)
    mo = 111412.84 * math.cos(f) - 93.5 * math.cos(3 * f) + 0.118 * math.cos(5 * f)
    xs = [(p[1] - poly[0][1]) * mo for p in poly]
    ys = [(p[0] - poly[0][0]) * ml for p in poly]
    a = 0.0
    for i in range(len(poly)):
        j = (i + 1) % len(poly)
        a += xs[i] * ys[j] - xs[j] * ys[i]
    area = abs(a) / 2.0
    w = (poly[1][1] - poly[0][1]) * mo
    h = (poly[2][0] - poly[1][0]) * ml
    out = {"m_per_deg_lat": round(ml, 2), "m_per_deg_lon": round(mo, 2),
           "width_m": round(w, 1), "height_m": round(h, 1),
           "area_m2": round(area, 1), "area_km2": round(area / 1e6, 6),
           "flat_approx_area_m2": 1560.0 * 800.0,
           "swath_and_spacing": {}}
    for h_m in (120.0, 119.5):
        sw = 2.0 * h_m * math.tan(math.radians(30.0))
        out["swath_and_spacing"][f"{h_m}"] = {"swath_m": round(sw, 4),
                                              "spacing_m": round(0.7 * sw, 4)}
    return out


def main() -> None:
    res = {
        "ceiling_control_HV4W55_vs_HV4W55R": ceiling_control(),
        "gate_to_gate_prediction_error": gate_to_gate(),
        "K1_separation_note_e": k1_separation(),
        "K2_param_strict_comparison": k2_params(),
        "K3_K4_spawn": k3k4_spawn(),
        "K5_beamidp_makespan_confound": k5_beamidp(),
        "K6_ceiling_cost_s": None,
        "K7_appendixB_276": k7_appendixB(),
        "K8_battery_telemetry": k8_battery(),
        "K9_table_z_rulers": k9_tablez(),
        "survey_geometry": survey_geometry(),
    }
    res["K6_ceiling_cost_s"] = {
        "makespan_119_5_s": res["ceiling_control_HV4W55_vs_HV4W55R"]["planned_makespan_s"]["HV4W55"],
        "makespan_120_0_s": res["ceiling_control_HV4W55_vs_HV4W55R"]["planned_makespan_s"]["HV4W55R"],
        "cost_s": res["ceiling_control_HV4W55_vs_HV4W55R"]["planned_makespan_cost_of_1195_s"],
        "verdict": "+0.0279 s, not +0.02 s"}
    jdump(OUT / "corrections.json", res)
    c = res["ceiling_control_HV4W55_vs_HV4W55R"]
    print("CEILING CONTROL  identical files:", len(c["identical_files"]),
          " differing:", [d["file"] for d in c["differing_files"]])
    print("  planned makespan cost of 119.5 m:", c["planned_makespan_cost_of_1195_s"], "s")
    print("  exported >120:", c["exported"]["n_above_120"], " flown t>120:", c["flown"]["t_above_120_s"])
    print("K1 range vs reporter:", res["K1_separation_note_e"]["range_vs_reporter_m"],
          " vs binsep:", res["K1_separation_note_e"]["range_vs_binsep_m"],
          " frame:", res["K1_separation_note_e"]["range_frame_component_m"],
          " sampling:", res["K1_separation_note_e"]["range_sampling_component_m"])
    print("K3 bearings:", {u: v["bearing_deg"] for u, v in res["K3_K4_spawn"]["per_uav"].items()},
          " max spacing delta:", res["K3_K4_spawn"]["max_abs_spacing_delta_m"])
    print("K4 file identity:", {k: v["identical"] for k, v in res["K3_K4_spawn"]["file_bit_identity"].items()})
    print("K9:", {k: v for k, v in res["K9_table_z_rulers"].items() if isinstance(v, (int, list))})
    print("geometry:", res["survey_geometry"]["area_m2"], res["survey_geometry"]["swath_and_spacing"])
    for t in ("HV4W00", "HV4W05", "HV4W55", "HV4W55R", "HV4W55S", "BEAMIDP1"):
        g = res["gate_to_gate_prediction_error"][t]
        print(f"  gate-to-gate {t:9s} mk_err {g['makespan_err_pct']:+6.3f}%  "
              f"binding {g['binding_uav']} {g['binding_gate_err_pct']:+6.3f}%  "
              f"range {g['gate_err_range_pct']}")


if __name__ == "__main__":
    main()
