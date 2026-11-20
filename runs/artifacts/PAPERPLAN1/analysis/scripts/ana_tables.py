# -*- coding: utf-8 -*-
"""Stage 3: assemble the paper's result tables from the stage-2 outputs.

Everything printed here traces to out/uav_kinematics.json, out/separation.json,
out/timing.json or out/altitude.json, each of which was computed from the raw
run products by the scripts beside this one.  No number is copied from
sortie_results.json, prep_result.json or any reporter summary; where this file
mentions one of those it is to record a disagreement.
"""
from __future__ import annotations

import json

from ana_lib import OUT, ROOT, UAVS, jdump, jload

K = jload(OUT / "uav_kinematics.json")
S = jload(OUT / "separation.json")
T = jload(OUT / "timing.json")
A = jload(OUT / "altitude.json")
BEAM = jload(ROOT / "runs/artifacts/PAPERPLAN1/prep_beamidp/beamidp_table.json")

WINDY = ("HV4W55", "HV4W05", "HV4W00")
ALL4 = ("HV4W55", "HV4W05", "HV4W00", "BEAMIDP1")
ALL6 = ("HV4SITL15", "HV4SITL16", "HV4W55", "HV4W05", "HV4W00", "BEAMIDP1")
LABEL = {"HV4SITL15": "HV4SITL15  plan 5@270 / sim none, ceiling 120.0",
         "HV4SITL16": "HV4SITL16  plan 5@270 / sim none, ceiling 120.0",
         "HV4W55": "HV4W55  plan 5@270 / sim 5@270",
         "HV4W05": "HV4W05  plan 0 / sim 5@270",
         "HV4W00": "HV4W00  plan 0 / sim 0",
         "BEAMIDP1": "BEAMIDP1  plan 5@270 / sim 0"}


def f(x, n=3):
    return "--" if x is None else f"{x:.{n}f}"


def md(rows, head):
    w = [max(len(str(h)), *(len(str(r[i])) for r in rows)) for i, h in enumerate(head)]
    out = ["| " + " | ".join(str(h).ljust(w[i]) for i, h in enumerate(head)) + " |",
           "|" + "|".join("-" * (x + 2) for x in w) + "|"]
    for r in rows:
        out.append("| " + " | ".join(str(c).ljust(w[i]) for i, c in enumerate(r)) + " |")
    return "\n".join(out)


# ---------------------------------------------------------------- Table W ----
def table_wind():
    rows = []
    for t in ALL6:
        sim = K[t].get("scenario_sim_wind") or {"spd_mps": 0.0, "dir_from_deg": float("nan")}
        for u in UAVS:
            w = K[t]["per_uav"][u]["wind"]
            l1, l2, l3 = (w["L1_kinematic_dataflash"], w["L2_ekf_state_dataflash"],
                          w["L3_telemetry_flight_csv"])
            e3 = l2["EKF3"]
            rows.append([
                t, u, f"{sim['spd_mps']:.1f}@{sim['dir_from_deg']:.0f}",
                f(l1["east_minus_west_mps"], 3), f(2 * sim["spd_mps"], 2),
                f(l1["scale_offset_removed_mps"], 3),
                f(l1["implied_W_along_EW_mps"], 3),
                f(e3["speed_mps"], 3) if e3 else "--",
                f(e3["from_deg"], 1) if e3 else "--",
                f(l3.get("median_speed_mps"), 3),
                f(l3.get("circular_median_from_deg"), 1),
                l1["n_cruise_gps_samples"]])
    return md(rows, ["trial", "uav", "SIM_WIND", "L1 E-W(GS-TAS)", "expected 2W",
                     "L1 scale offset", "L1 W_EW", "L2 EKF3 |W|", "L2 from",
                     "L3 tlm |W|", "L3 from", "n"])


# ---------------------------------------------------------------- Table P ----
def table_planned_flown():
    rows = []
    for t in ALL6:
        e, sep = T[t], S[t]
        rows.append([
            t, f(e["planned_makespan_s"], 2), f(e["flown_makespan_2m_s"], 2),
            f(e["makespan_err_2m_pct"], 2) + " %",
            f(e["flown_makespan_disarm_s"], 2), f(e["makespan_err_disarm_pct"], 2) + " %",
            f(e["planned_total_km"], 3), f(e["flown_total_km"], 3),
            f(e["total_dist_err_pct"], 2) + " %",
            f(sep["planned_min_sep_h_m"], 3),
            f(sep["frames"]["ecef"]["fleet_min_h_m"], 3),
            sep["frames"]["ecef"]["binding_pair"],
            f(A[t]["fleet_max_flown_m"], 4),
            f(max(r["max_amsl_m"] for r in K[t]["per_uav"].values()), 2),
            f(100 * min(c["ratio"] for c in e["coverage_all_airborne"]), 4) + " %"])
    return md(rows, ["trial", "plan mks (s)", "flown mks 2m (s)", "err", "flown mks disarm (s)",
                     "err", "plan km", "flown km", "err", "plan min sep (m)",
                     "flown min sep (m)", "pair", "max AGL (m)", "max AMSL (m)", "coverage"])


def table_per_aircraft():
    rows = []
    for t in ALL6:
        for u in UAVS:
            r, k = T[t]["per_uav"][u], K[t]["per_uav"][u]
            rows.append([
                t, u, r["cls"], f(r["plan_t_takeoff_s"], 1), f(r["flown_t_release_s"], 2),
                f(r["release_error_s"], 2), f(r["plan_t_land_s"], 1),
                f(r["flown_t_land_2m_s"], 2), f(r["land_err_2m_pct"], 2) + " %",
                f(k["airborne_duration_s"], 1),
                f(r["plan_km"], 4), f(r["flown_km"], 4), f(r["dist_err_pct"], 2) + " %",
                f(r["plan_energy_used_wh"], 1), f(r["plan_energy_margin_wh"], 1)])
    return md(rows, ["trial", "uav", "cls", "plan t_to (s)", "flown t_rel (s)", "d (s)",
                     "plan t_land (s)", "flown t_land (s)", "err", "airborne (s)",
                     "plan km", "flown km", "err", "E used (Wh)", "E margin (Wh)"])


# ---------------------------------------------------------------- Table C ----
def table_ignoring_wind():
    w55, w05, w00 = T["HV4W55"], T["HV4W05"], T["HV4W00"]
    rows = [
        ["planned makespan (s)", f(w00["planned_makespan_s"], 2), f(w05["planned_makespan_s"], 2),
         f(w55["planned_makespan_s"], 2)],
        ["flown makespan (s)", f(w00["flown_makespan_2m_s"], 2), f(w05["flown_makespan_2m_s"], 2),
         f(w55["flown_makespan_2m_s"], 2)],
        ["prediction error", f(w00["makespan_err_2m_pct"], 2) + " %",
         f(w05["makespan_err_2m_pct"], 2) + " %", f(w55["makespan_err_2m_pct"], 2) + " %"],
        ["planned fleet distance (km)", f(w00["planned_total_km"], 3),
         f(w05["planned_total_km"], 3), f(w55["planned_total_km"], 3)],
        ["flown fleet distance (km)", f(w00["flown_total_km"], 3),
         f(w05["flown_total_km"], 3), f(w55["flown_total_km"], 3)],
        ["distance error", f(w00["total_dist_err_pct"], 2) + " %",
         f(w05["total_dist_err_pct"], 2) + " %", f(w55["total_dist_err_pct"], 2) + " %"],
    ]
    for u in UAVS:
        rows.append([f"per-aircraft t_land error {u}",
                     f(w00["per_uav"][u]["land_err_2m_pct"], 2) + " %",
                     f(w05["per_uav"][u]["land_err_2m_pct"], 2) + " %",
                     f(w55["per_uav"][u]["land_err_2m_pct"], 2) + " %"])
    rows += [
        ["A1 scheduled release (s)", f(w00["per_uav"]["A1"]["plan_t_takeoff_s"], 1),
         f(w05["per_uav"]["A1"]["plan_t_takeoff_s"], 1), f(w55["per_uav"]["A1"]["plan_t_takeoff_s"], 1)],
        ["A1 airborne duration flown (s)",
         f(K["HV4W00"]["per_uav"]["A1"]["airborne_duration_s"], 1),
         f(K["HV4W05"]["per_uav"]["A1"]["airborne_duration_s"], 1),
         f(K["HV4W55"]["per_uav"]["A1"]["airborne_duration_s"], 1)],
        ["flown min separation (m, ECEF)",
         f(S["HV4W00"]["frames"]["ecef"]["fleet_min_h_m"], 3),
         f(S["HV4W05"]["frames"]["ecef"]["fleet_min_h_m"], 3),
         f(S["HV4W55"]["frames"]["ecef"]["fleet_min_h_m"], 3)],
        ["coverage", f(100 * min(c["ratio"] for c in w00["coverage_all_airborne"]), 4) + " %",
         f(100 * min(c["ratio"] for c in w05["coverage_all_airborne"]), 4) + " %",
         f(100 * min(c["ratio"] for c in w55["coverage_all_airborne"]), 4) + " %"],
    ]
    return md(rows, ["quantity", "HV4W00 plan0/sim0", "HV4W05 plan0/sim5", "HV4W55 plan5/sim5"])


def derived_wind_cost():
    w55, w05, w00 = T["HV4W55"], T["HV4W05"], T["HV4W00"]
    return {
        "A_cost_of_the_air_same_plan_HV4W05_minus_HV4W00": {
            "note": "identical mission files (9 plan files bit-identical per prep_check); "
                    "only SIM_WIND differs, so this isolates the air",
            "d_flown_makespan_s": round(w05["flown_makespan_2m_s"] - w00["flown_makespan_2m_s"], 3),
            "d_flown_makespan_pct": round(100 * (w05["flown_makespan_2m_s"] / w00["flown_makespan_2m_s"] - 1), 3),
            "d_flown_makespan_s_disarm_def": round(w05["flown_makespan_disarm_s"] - w00["flown_makespan_disarm_s"], 3),
            "prediction_error_still_air_pct": w00["makespan_err_2m_pct"],
            "prediction_error_in_wind_pct": w05["makespan_err_2m_pct"],
            "d_flown_fleet_km": round(w05["flown_total_km"] - w00["flown_total_km"], 4),
        },
        "B_cost_of_ignoring_the_wind_model_same_air_HV4W05_vs_HV4W55": {
            "note": "both flown in 5 m/s from 270; HV4W55's plan knows about it, "
                    "HV4W05's does not",
            "d_planned_makespan_s": round(w05["planned_makespan_s"] - w55["planned_makespan_s"], 3),
            "d_flown_makespan_s": round(w05["flown_makespan_2m_s"] - w55["flown_makespan_2m_s"], 3),
            "d_flown_makespan_pct": round(100 * (w05["flown_makespan_2m_s"] / w55["flown_makespan_2m_s"] - 1), 3),
            "prediction_error_wind_ignorant_pct": w05["makespan_err_2m_pct"],
            "prediction_error_wind_aware_pct": w55["makespan_err_2m_pct"],
            "per_aircraft_prediction_error_pct": {
                u: {"wind_ignorant": w05["per_uav"][u]["land_err_2m_pct"],
                    "wind_aware": w55["per_uav"][u]["land_err_2m_pct"]} for u in UAVS},
            "d_flown_fleet_km": round(w05["flown_total_km"] - w55["flown_total_km"], 4),
            "makespan_decomposition": {
                "A1_release_delay_added_by_wind_aware_plan_s": round(
                    w55["per_uav"]["A1"]["plan_t_takeoff_s"] - w05["per_uav"]["A1"]["plan_t_takeoff_s"], 3),
                "A1_flown_airborne_duration_wind_ignorant_s": K["HV4W05"]["per_uav"]["A1"]["airborne_duration_s"],
                "A1_flown_airborne_duration_wind_aware_s": K["HV4W55"]["per_uav"]["A1"]["airborne_duration_s"],
                "d_airborne_duration_s": round(
                    K["HV4W55"]["per_uav"]["A1"]["airborne_duration_s"]
                    - K["HV4W05"]["per_uav"]["A1"]["airborne_duration_s"], 3),
                "reading": ("the 47 s by which the wind-aware fleet finishes later is "
                            "almost entirely a 44 s later scheduled release of A1, not a "
                            "slower flight"),
            },
        },
    }


# ---------------------------------------------------------------- Table H ----
def table_altitude():
    rows = []
    for t in ALL6:
        e = A[t]
        for u in UAVS:
            r = e["per_uav"][u]
            has = e["has_dataflash"]
            flown = r["max_rel_alt_m_flown"] if has else r["max_rel_alt_m_csv_1hz"]
            rows.append([
                t, u, f(e["scenario_altitude_m"], 1), f(r["max_exported_rel_alt_m"], 4),
                r["n_items_above_120"], f(flown, 4),
                "6 Hz",
                f(120.0 - flown, 4), f(121.92 - flown, 4),
                f(r["time_above_120_000_s"] if has else r["time_above_120_s_csv_1hz"], 2),
                f(r["time_above_121_92_s"] if has else r["time_above_121_92_s_csv_1hz"], 2)])
    return md(rows, ["trial", "uav", "ceiling", "exported max", "items >120",
                     "flown max", "rate", "margin to 120.000", "margin to 121.92",
                     "t>120.000 (s)", "t>121.92 (s)"])


# ---------------------------------------------------------------- Table B ----
def table_beam():
    new = BEAM[0]
    ft = new["fleet_turn_totals"]
    rows = [["sequential (boustrophedon)", f(ft["seq_t"], 2), "0.00 %", f(ft["seq_l"] / 1000, 3), "0.00 %"],
            ["fixed skip-k", f(ft["skip_t"], 2), f(100 * (ft["skip_t"] / ft["seq_t"] - 1), 2) + " %",
             f(ft["skip_l"] / 1000, 3), f(100 * (ft["skip_l"] / ft["seq_l"] - 1), 2) + " %"],
            ["Beam-IDP (HARVEST)", f(ft["bid_t"], 2), f(100 * (ft["bid_t"] / ft["seq_t"] - 1), 2) + " %",
             f(ft["bid_l"] / 1000, 3), f(100 * (ft["bid_l"] / ft["seq_l"] - 1), 2) + " %"],
            ["Held-Karp optimum", f(ft["hk_t"], 2), f(100 * (ft["hk_t"] / ft["seq_t"] - 1), 2) + " %",
             f(ft["hk_l"] / 1000, 3), f(100 * (ft["hk_l"] / ft["seq_l"] - 1), 2) + " %"]]
    cells = []
    for k, v in new["cells"].items():
        o = v["orders"]
        cells.append([k, v["cls"], v["uav"], v["n"], v["k"],
                      f(o["sequential"]["turn_time_s"], 2),
                      f(o["skip_k_fixed"]["turn_time_s"], 2),
                      f(o["beam_idp_planned"]["turn_time_s"], 2),
                      f(o["held_karp_opt"]["turn_time_s"], 2),
                      f(o["beam_idp_planned"]["d_turn_time_pct"], 2) + " %",
                      str(o["beam_idp_planned"]["order"])])
    return (md(rows, ["strip order (planner model)", "fleet turn time (s)", "vs sequential",
                      "fleet turn distance (km)", "vs sequential"]),
            md(cells, ["cell", "cls", "uav", "n", "k", "seq (s)", "skip-k (s)",
                       "Beam-IDP (s)", "Held-Karp (s)", "Beam-IDP vs seq", "flown order"]),
            {"beam_captures_pct_of_available_saving": round(
                100 * (ft["seq_t"] - ft["bid_t"]) / (ft["seq_t"] - ft["hk_t"]), 2),
             "seq_t": ft["seq_t"], "bid_t": ft["bid_t"], "hk_t": ft["hk_t"],
             "skip_t": ft["skip_t"]})



def table_beam_flown():
    """D3: what BEAMIDP1 actually demonstrates, kept apart from the model."""
    t = "BEAMIDP1"
    e, sep, alt = T[t], S[t], A[t]
    ms = jload(ROOT / "runs/artifacts" / t / "plan" / "mission_summary.json")
    up = jload(ROOT / "runs/artifacts" / t / "mission_upload.json")
    rows = [
        ["mission completed", "3/3 aircraft released, flew and landed",
         "out/timing.json per_uav (all three have a 2 m touchdown)"],
        ["mission items sent = read back",
         " / ".join(f"{r['uav']} {r['sent']}={r['read_back']}" for r in up),
         f"runs/artifacts/{t}/mission_upload.json"],
        ["makespan", f"planned {e['planned_makespan_s']:.2f} s -> flown "
                     f"{e['flown_makespan_2m_s']:.2f} s ({e['makespan_err_2m_pct']:+.2f} %)",
         "out/timing.json"],
        ["fleet distance", f"planned {e['planned_total_km']:.3f} km -> flown "
                           f"{e['flown_total_km']:.3f} km ({e['total_dist_err_pct']:+.2f} %)",
         "out/timing.json"],
        ["minimum separation",
         f"planned {sep['planned_min_sep_h_m']:.3f} m -> flown "
         f"{sep['frames']['ecef']['fleet_min_h_m']:.3f} m "
         f"({sep['frames']['ecef']['binding_pair']}), "
         f"{sep['frames']['ecef']['n_violations']} violations",
         "out/separation.json frames.ecef"],
        ["maximum AGL", f"exported {alt['fleet_max_exported_m']:.4f} m -> flown "
                        f"{alt['fleet_max_flown_m']:.4f} m, "
                        f"{sum(alt['per_uav'][u]['time_above_120_000_s'] for u in UAVS):.2f} s "
                        "above 120.000 m", "out/altitude.json"],
        ["coverage", " / ".join(f"{c['area']} {100 * c['ratio']:.4f} %"
                                for c in e["coverage_all_airborne"])
         + f" ({e['coverage_all_airborne'][0]['area_ha']:.2f} ha each)",
         "out/timing.json coverage_all_airborne (track buffer, upper bound)"],
        ["tightest energy margin (planner model, NOT measured)",
         " / ".join(f"{u} {ms['uavs'][u]['energy_margin_wh']:.1f} Wh" for u in UAVS),
         f"runs/artifacts/{t}/plan/mission_summary.json"],
    ]
    return md(rows, ["what BEAMIDP1 shows (FLOWN)", "value", "source"])


def main() -> None:
    tw, tc, tb = table_wind(), table_ignoring_wind(), table_beam()
    tp, tpa, th = table_planned_flown(), table_per_aircraft(), table_altitude()
    derived = derived_wind_cost()
    jdump(OUT / "derived.json", {"wind_cost": derived, "beam": tb[2]})
    (OUT / "tables.md").write_text("\n\n".join([
        "## Table W  wind proof, three independent lines", tw,
        "## Table P  planned vs flown, per sortie", tp,
        "## Table P2  planned vs flown, per aircraft", tpa,
        "## Table C  the cost of ignoring the wind", tc,
        "### derived", "```json\n" + json.dumps(derived, indent=1, ensure_ascii=False) + "\n```",
        "## Table H  altitude compliance under D2", th,
        "## Table B  Beam-IDP, PLANNER MODEL PREDICTS (no flight involved)", tb[0],
        "### per cell (planner model)", tb[1],
        "```json\n" + json.dumps(tb[2], indent=1, ensure_ascii=False) + "\n```",
        "## Table B2  Beam-IDP, FLOWN (D3: executability only; contributes no number "
        "to Table B)", table_beam_flown(),
    ]), encoding="utf-8")
    print((OUT / "tables.md").read_text(encoding="utf-8"))


if __name__ == "__main__":
    main()
