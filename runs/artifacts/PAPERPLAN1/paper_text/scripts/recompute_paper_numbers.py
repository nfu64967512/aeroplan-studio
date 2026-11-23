# -*- coding: utf-8 -*-
"""Recompute EVERY number that the paper_text package puts into the paper.

Rule of this label: a number may enter the Word file only if this script prints it.
Nothing here is copied from prep_result.json, sortie_results.json, hv_sitl_report.py,
binalt.py or binsep.py.  Sources are, in order of preference:

  1. the per-sortie products written by the runner        runs/artifacts/<trial>/...
  2. the dataflash-derived analysis products              PAPERPLAN1/analysis/out/*.json
  3. the planner library itself, called here              hv_dccpp.*
  4. the benchmark-control products                       PAPERPLAN1/prep_baseline_control/out/*
  5. the Beam-IDP planner-model table                     PAPERPLAN1/prep_beamidp/beamidp_table.json

Run:  python runs/artifacts/PAPERPLAN1/paper_text/scripts/recompute_paper_numbers.py
Out:  runs/artifacts/PAPERPLAN1/paper_text/out/NUMBERS.json  (+ stdout log)

No SITL, no MAVProxy, no git, no network, no COM port.  Read-only on the repo.
"""
from __future__ import annotations

import csv
import json
import math
import os
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", ".."))
sys.path.insert(0, REPO)

PP = os.path.join(REPO, "runs", "artifacts", "PAPERPLAN1")
ANA = os.path.join(PP, "analysis", "out")
OUT = os.path.join(PP, "paper_text", "out")

GUI_TRIALS = ["HV4SITL15", "HV4SITL16", "HV4W00", "HV4W05", "HV4W55"]
ALL_TRIALS = GUI_TRIALS + ["BEAMIDP1"]


def j(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


N: dict = {"_note": "every value here was recomputed by this script; see 'src' fields"}


# ---------------------------------------------------------------- 1. timing
timing = j(os.path.join(ANA, "timing.json"))
sep = j(os.path.join(ANA, "separation.json"))
alt = j(os.path.join(ANA, "altitude.json"))
uav = j(os.path.join(ANA, "uav_kinematics.json"))

N["sorties"] = {}
for tr in ALL_TRIALS:
    t = timing[tr]
    s = sep[tr]["frames"]["ecef"]
    a = alt[tr]
    row = {
        "planned_makespan_s": round(t["planned_makespan_s"], 2),
        "flown_makespan_2m_s": t["flown_makespan_2m_s"],
        "makespan_err_2m_pct": t["makespan_err_2m_pct"],
        "flown_makespan_disarm_s": t["flown_makespan_disarm_s"],
        "makespan_err_disarm_pct": t["makespan_err_disarm_pct"],
        "planned_total_km": t["planned_total_km"],
        "flown_total_km": t["flown_total_km"],
        "total_dist_err_pct": t["total_dist_err_pct"],
        "planned_min_sep_h_m": round(sep[tr]["planned_min_sep_h_m"], 3),
        "flown_min_sep_h_exact_m": s["fleet_min_h_m"],
        "flown_min_sep_h_repo_flat_m": sep[tr]["frames"]["repo"]["fleet_min_h_m"],
        "flown_min_sep_binding_pair": s["binding_pair"],
        "d_v_at_min_h_m": s["pairs"][s["binding_pair"]]["d_v_at_min_h_m"],
        "n_airborne_samples_binding_pair": s["pairs"][s["binding_pair"]]["n_airborne_samples"],
        "n_violations": s["n_violations"],
        "ceiling_m": a["scenario_h_ceil_m"],
        "max_exported_rel_alt_m": a["fleet_max_exported_m"],
        "max_flown_rel_alt_m": a["fleet_max_flown_m"],
        "margin_to_120_000_m": a["fleet_margin_to_120_m"],
        "margin_to_121_92_m": a["fleet_margin_to_121_92_m"],
        "items_above_120_total": sum(v["n_items_above_120"] for v in a["per_uav"].values()),
        "time_above_120_000_fleet_s": round(sum(v["time_above_120_000_s"] for v in a["per_uav"].values()), 2),
        "time_above_121_92_fleet_s": round(sum(v["time_above_121_92_s"] for v in a["per_uav"].values()), 2),
        "coverage": [(c["area"], c["area_m2"], round(c["ratio"], 6)) for c in (t["coverage_all_airborne"] or [])],
        "swath_m": t["swath_m_recomputed"],
        "per_uav": {},
    }
    for u, p in t["per_uav"].items():
        row["per_uav"][u] = {
            "cls": p["cls"],
            "plan_t_takeoff_s": p["plan_t_takeoff_s"],
            "flown_t_release_s": p["flown_t_release_s"],
            "plan_t_land_s": round(p["plan_t_land_s"], 1),
            "flown_t_land_2m_s": p["flown_t_land_2m_s"],
            "land_err_2m_pct": p["land_err_2m_pct"],
            "plan_km": p["plan_km"],
            "flown_km": p["flown_km"],
            "dist_err_pct": p["dist_err_pct"],
            "plan_n_items": p["plan_n_items"],
            "plan_energy_used_wh": p["plan_energy_used_wh"],
            "plan_energy_margin_wh": p["plan_energy_margin_wh"],
            "max_exported_rel_alt_m": alt[tr]["per_uav"][u]["max_exported_rel_alt_m"],
            "max_flown_rel_alt_m": alt[tr]["per_uav"][u]["max_rel_alt_m_flown"],
            "n_items_above_120": alt[tr]["per_uav"][u]["n_items_above_120"],
            "time_above_120_000_s": alt[tr]["per_uav"][u]["time_above_120_000_s"],
        }
    N["sorties"][tr] = row

# ------------------------------------------------- 2. mission upload + params
N["upload_and_params"] = {}
for tr in ["HV4W55", "HV4W05", "HV4W00", "BEAMIDP1", "HV4SITL16", "HV4SITL15"]:
    d = os.path.join(REPO, "runs", "artifacts", tr)
    rec: dict = {}
    mu = os.path.join(d, "mission_upload.json")
    if os.path.exists(mu):
        rec["mission_items"] = {r["uav"]: (r["sent"], r["read_back"], r["ok"]) for r in j(mu)}
    pr = os.path.join(d, "params_readback.csv")
    if os.path.exists(pr):
        rows = list(csv.DictReader(open(pr, encoding="utf-8")))
        exported = {}
        for ac in ("A1", "B2", "B3"):
            pf = os.path.join(d, "plan", ac + ".param")
            if not os.path.exists(pf):
                continue
            for line in open(pf, encoding="utf-8"):
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                k, v = line.split(",", 1)
                exported[(ac, k)] = float(v)
        tol = lambda e: max(0.01, 0.001 * abs(e))
        st_ok = st_n = 0
        st_diffs = []
        for r in rows:
            key = (r["uav"], r["param"])
            if key not in exported:
                continue
            st_n += 1
            e, g = exported[key], float(r["actual"])
            if abs(g - e) <= tol(e):
                st_ok += 1
            else:
                st_diffs.append([r["uav"], r["param"], e, g])
        d3_ok = d3_n = 0
        d3_diffs = []
        for r in rows:
            d3_n += 1
            e, g = float(r["expected"]), float(r["actual"])
            if abs(g - e) <= tol(e):
                d3_ok += 1
            else:
                d3_diffs.append([r["uav"], r["param"], e, g])
        rec["readback_rows"] = len(rows)
        rec["readback_ok"] = sum(1 for r in rows if r["ok"] == "True")
        rec["per_uav_rows"] = {u: sum(1 for r in rows if r["uav"] == u) for u in ("A1", "B2", "B3")}
        rec["exported_params_per_uav"] = {ac: sum(1 for k in exported if k[0] == ac) for ac in ("A1", "B2", "B3")}
        rec["strict_vs_exported_file"] = f"{st_ok}/{st_n}"
        rec["strict_diffs"] = st_diffs
        rec["no_integer_rescue"] = f"{d3_ok}/{d3_n}"
        rec["no_integer_rescue_diffs"] = d3_diffs
        rec["RTL_RADIUS"] = {r["uav"]: float(r["actual"]) for r in rows if r["param"] == "RTL_RADIUS"}
        rec["WP_LOITER_RAD"] = {r["uav"]: float(r["actual"]) for r in rows if r["param"] == "WP_LOITER_RAD"}
        rec["FENCE_ALT_MAX"] = {r["uav"]: float(r["actual"]) for r in rows if r["param"] == "FENCE_ALT_MAX"}
    wr = os.path.join(d, "wind_readback.json")
    if os.path.exists(wr):
        w = j(wr)
        rec["wind_readback"] = {
            "sim_wind": w["sim_wind"],
            "plan_wind": w.get("plan_wind"),
            "rows_ok": f"{sum(1 for r in w['rows'] if r['ok'])}/{len(w['rows'])}",
        }
    N["upload_and_params"][tr] = rec

# ------------------------------------------------------------- 3. wind proof
N["wind_proof"] = {}
for tr in ALL_TRIALS:
    per = {}
    for ac, d in uav[tr]["per_uav"].items():
        w = d["wind"]
        l1, l2, l3 = w["L1_kinematic_dataflash"], w["L2_ekf_state_dataflash"]["EKF3"], w["L3_telemetry_flight_csv"]
        per[ac] = {
            "L1_east_minus_west_mps": l1["east_minus_west_mps"],
            "L1_scale_offset_mps": l1["scale_offset_removed_mps"],
            "L1_n_east": l1["n_by_sector"]["east"],
            "L1_n_west": l1["n_by_sector"]["west"],
            "L1_north_minus_south_mps": l1["north_minus_south_mps"],
            "L1_n_north": l1["n_by_sector"]["north"],
            "L1_n_south": l1["n_by_sector"]["south"],
            "L2_ekf3_speed_mps": l2["speed_mps"],
            "L2_ekf3_from_deg": l2["from_deg"],
            "L3_tlm_speed_mps": (l3["median_speed_mps"] if l3.get("available") else None),
            "L3_tlm_from_deg": (l3["circular_median_from_deg"] if l3.get("available") else None),
        }
    N["wind_proof"][tr] = per

# expected east-minus-west, from the planner's own ground-speed law
from hv_dccpp.platform import PLATFORM_PIONEER as SPEC_A, PLATFORM_HERO as SPEC_B, G_MPS2  # noqa: E402
from hv_dccpp.dubins import skip_k  # noqa: E402

N["wind_triangle_expectation"] = {}
for nm, spec in (("A_Pioneer", SPEC_A), ("B_HERO", SPEC_B)):
    Va, W = spec.cruise_airspeed_mps, 5.0
    east, west, cross = Va + W, Va - W, math.sqrt(Va * Va - W * W)
    N["wind_triangle_expectation"][nm] = {
        "Va_mps": Va,
        "Vg_east_mps": round(east, 3),
        "Vg_west_mps": round(west, 3),
        "Vg_crosswind_mps": round(cross, 3),
        "drift_deg": round(math.degrees(math.asin(W / Va)), 3),
        "east_minus_west_of_Vg_minus_Va_mps": round((east - Va) - (west - Va), 3),
    }

# -------------------------------------------------------------- 4. R_eff / k
N["r_eff"] = {}
for nm, spec in (("A_Pioneer", SPEC_A), ("B_HERO", SPEC_B)):
    Va, phi = spec.cruise_airspeed_mps, spec.max_bank_angle_deg
    Rgeo = Va * Va / (G_MPS2 * math.tan(math.radians(phi)))
    rec = {
        "Va_mps": Va,
        "max_bank_angle_deg": phi,
        "R_theo_table_m": spec.turn_radius_theoretical_m,
        "R_geometric_m": round(Rgeo, 2),
        "geometric_minus_table_pct": round(100 * (Rgeo - spec.turn_radius_theoretical_m) / spec.turn_radius_theoretical_m, 2),
        "R_plan_m": spec.turn_radius_planning_m,
        "w_star_mps": round(spec.w_star(), 4),
        "bank_needed_downwind_phase_on_R_plan_deg": {},
        "R_eff_by_wind_m": {},
        "k_by_wind_s96591": {},
    }
    for W in (0.0, 5.0, 6.0, 8.0):
        r = spec.r_eff(W)
        rec["R_eff_by_wind_m"][str(W)] = round(r, 2)
        rec["k_by_wind_s96591"][str(W)] = skip_k(96.591, r)
        Vg = Va + W
        rec["bank_needed_downwind_phase_on_R_plan_deg"][str(W)] = round(
            math.degrees(math.atan(Vg * Vg / (G_MPS2 * spec.turn_radius_planning_m))), 2
        )
    N["r_eff"][nm] = rec
N["r_eff"]["_g_mps2"] = G_MPS2
N["r_eff"]["_k_at_s96995"] = {
    "A_W0": skip_k(96.995, SPEC_A.r_eff(0.0)),
    "B_W0": skip_k(96.995, SPEC_B.r_eff(0.0)),
}

# ------------------------------------------------- 5. swath / spacing / GSD
N["strip_geometry"] = {}
for h in (120.0, 119.5):
    w = 2 * h * math.tan(math.radians(60.0) / 2)
    N["strip_geometry"][str(h)] = {"swath_m": round(w, 3), "spacing_m": round(w * 0.70, 3)}
N["strip_geometry"]["_gsd_m_per_px_at_119_5"] = round(119.5 * 3.9e-6 / 16e-3, 4)

# ------------------------------------------------------------ 6. survey area
scn = j(os.path.join(REPO, "runs", "artifacts", "HV4W55", "scenario.json"))
poly = scn["areas"][0]["polygon"]
lats = [p[0] for p in poly]
lons = [p[1] for p in poly]
lat0 = sum(lats) / len(lats)
a_e, f_e = 6378137.0, 1 / 298.257223563
e2 = f_e * (2 - f_e)
s0 = math.sin(math.radians(lat0))
m_lat = math.pi / 180 * a_e * (1 - e2) / (1 - e2 * s0 * s0) ** 1.5
m_lon = math.pi / 180 * a_e / math.sqrt(1 - e2 * s0 * s0) * math.cos(math.radians(lat0))
dlat, dlon = max(lats) - min(lats), max(lons) - min(lons)
N["survey_area"] = {
    "polygon": poly,
    "m_per_deg_lat_wgs84": round(m_lat, 2),
    "m_per_deg_lon_wgs84": round(m_lon, 2),
    "extent_NS_m_ellipsoidal": round(dlat * m_lat, 2),
    "extent_EW_m_ellipsoidal": round(dlon * m_lon, 2),
    "area_m2_ellipsoidal": round(dlat * m_lat * dlon * m_lon, 1),
    "extent_NS_m_flat_111132": round(dlat * 111132.0, 2),
    "extent_EW_m_flat": round(dlon * 101819.43, 2),
    "area_m2_flat": round(dlat * 111132.0 * dlon * 101819.43, 1),
    "note": "the scenario polygon was authored with the flat 111132 m/deg; the planner reads it through the exact WGS84 frame",
}

# --------------------------------------------------------------- 7. Beam-IDP
bt = j(os.path.join(PP, "prep_beamidp", "beamidp_table.json"))
new = bt[0]
orders = ["sequential", "skip_k_fixed", "beam_idp_planned", "held_karp_opt"]
tt = {o: 0.0 for o in orders}
td = {o: 0.0 for o in orders}
te = {o: 0.0 for o in orders}
per_uav_delta: dict = {}
cells = []
for name, c in new["cells"].items():
    for o in orders:
        tt[o] += c["orders"][o]["turn_time_s"]
        td[o] += c["orders"][o]["turn_dist_m"]
        te[o] += c["orders"][o]["turn_energy_wh"]
    per_uav_delta[c["uav"]] = per_uav_delta.get(c["uav"], 0.0) + (
        c["orders"]["sequential"]["turn_time_s"] - c["orders"]["beam_idp_planned"]["turn_time_s"]
    )
    cells.append(
        {
            "cell": name,
            "uav": c["uav"],
            "cls": c["cls"],
            "n": c["n"],
            "k": c["k"],
            "R_eff_m": c["r_eff_m"],
            "seq_turn_time_s": c["orders"]["sequential"]["turn_time_s"],
            "bid_turn_time_s": c["orders"]["beam_idp_planned"]["turn_time_s"],
            "bid_vs_seq_pct": round(
                100
                * (c["orders"]["beam_idp_planned"]["turn_time_s"] - c["orders"]["sequential"]["turn_time_s"])
                / c["orders"]["sequential"]["turn_time_s"],
                2,
            ),
            "hk_turn_time_s": c["orders"]["held_karp_opt"]["turn_time_s"],
        }
    )
fr = j(os.path.join(PP, "prep_beamidp", "fly_ready.json"))
per_ac_seq = {u: round(m["t_land_s"] + per_uav_delta[u], 2) for u, m in fr["mission_summary_uavs"].items()}
N["beam_idp"] = {
    "scenario": new["scenario"],
    "trial": new["trial"],
    "h_ceil_m": new["h_ceil_m"],
    "wind": new["wind"],
    "planned_makespan_s": new["makespan_s"],
    "fleet_turn_time_s": {o: round(tt[o], 2) for o in orders},
    "fleet_turn_dist_m": {o: round(td[o], 1) for o in orders},
    "fleet_turn_energy_wh": {o: round(te[o], 2) for o in orders},
    "bid_vs_seq_time_pct": round(100 * (tt["beam_idp_planned"] - tt["sequential"]) / tt["sequential"], 2),
    "bid_vs_seq_dist_pct": round(100 * (td["beam_idp_planned"] - td["sequential"]) / td["sequential"], 2),
    "bid_vs_seq_energy_pct": round(100 * (te["beam_idp_planned"] - te["sequential"]) / te["sequential"], 2),
    "skip_vs_seq_time_pct": round(100 * (tt["skip_k_fixed"] - tt["sequential"]) / tt["sequential"], 2),
    "hk_vs_seq_time_pct": round(100 * (tt["held_karp_opt"] - tt["sequential"]) / tt["sequential"], 2),
    "bid_above_hk_pct": round(100 * (tt["beam_idp_planned"] - tt["held_karp_opt"]) / tt["held_karp_opt"], 2),
    "bid_captures_pct_of_available": round(
        100 * (tt["sequential"] - tt["beam_idp_planned"]) / (tt["sequential"] - tt["held_karp_opt"]), 2
    ),
    "cells": cells,
    "per_uav_seq_minus_bid_turn_time_s": {k: round(v, 2) for k, v in per_uav_delta.items()},
    "makespan_sequential_estimate_PER_AIRCRAFT_s": per_ac_seq,
    "makespan_sequential_estimate_CHOSEN_s": round(max(per_ac_seq.values()), 2),
    "makespan_sequential_estimate_CHOSEN_pct": round(
        100 * (max(per_ac_seq.values()) - new["makespan_s"]) / new["makespan_s"], 2
    ),
    "makespan_sequential_estimate_REJECTED_s": 2541.8,
    "makespan_sequential_estimate_REJECTED_recipe": "planned makespan + fleet turn-time delta / n_uav = 2348.04 + 581.15/3",
    "makespan_sequential_estimate_REJECTED_reproduced_s": round(new["makespan_s"] + (tt["sequential"] - tt["beam_idp_planned"]) / 3, 2),
}

# paper scenario (Table Y): are the four orders identical?
N["paper_scenario_orders"] = {}
for e in bt[1:]:
    rows = []
    fl = {}
    for name, c in e["cells"].items():
        r = {"cell": name, "uav": c["uav"], "cls": c["cls"], "n": c["n"], "k": c["k"],
             "triggers": c["triggers"], "spacing_m": c["spacing_m"], "R_eff_m": c["r_eff_m"], "orders": {}}
        for o, oo in c["orders"].items():
            r["orders"][o] = {"order": oo["order"], "turn_time_s": oo["turn_time_s"], "turn_dist_m": oo["turn_dist_m"]}
            fl[o] = fl.get(o, [0.0, 0.0])
            fl[o][0] += oo["turn_time_s"]
            fl[o][1] += oo["turn_dist_m"]
        rows.append(r)
    orders_present = sorted({o for c in e["cells"].values() for o in c["orders"]})
    all_orders_equal = all(
        len({tuple(c["orders"][o]["order"]) for o in c["orders"]}) == 1 for c in e["cells"].values()
    )
    all_costs_equal = all(
        len({round(c["orders"][o]["turn_time_s"], 6) for o in c["orders"]}) == 1 for c in e["cells"].values()
    )
    N["paper_scenario_orders"][e["label"]] = {
        "scenario": e["scenario"],
        "h_ceil_m": e["h_ceil_m"],
        "makespan_s": e["makespan_s"],
        "cells": rows,
        "fleet_turn": {o: [round(v[0], 2), round(v[1], 1)] for o, v in fl.items()},
        "ALL_FOUR_ORDERS_IDENTICAL": all_orders_equal,
        "ALL_FOUR_COSTS_IDENTICAL": all_costs_equal,
        "orders_present": orders_present,
    }

# ------------------------------------------------------ 8. KPI / baseline row
l2 = j(os.path.join(PP, "prep_baseline_control", "out", "stepL2_dmodel_vs_baseline.json"))
Dm = l2["D_model_km"]
fr_rows = sorted(((r["cell"], r["d_ceq"] / Dm, r["r1_over"], r["n"], r["d_plan"]) for r in l2["rows"]), key=lambda x: x[1])
over = [r for r in fr_rows if r[2] > 0]
under = [r for r in fr_rows if r[2] == 0]
N["kpi_energy"] = {
    "D_model_km": Dm,
    "D_use_km": l2["D_use_km"],
    "ratio_D_model_over_D_use": round(Dm / l2["D_use_km"], 4),
    "cells_over_D_model": {r[0]: {"frac_pct": round(100 * r[1], 1), "over": f"{r[2]}/{r[3]}"} for r in over},
    "other_cells_frac_pct_range": [round(100 * under[0][1], 1), round(100 * under[-1][1], 1)],
    "max_frac_under_D_use_pct": round(100 * max(r[4] for r in fr_rows) / l2["D_use_km"], 1),
    "n_cells": len(fr_rows),
}
# estimator calibration band, parsed from the log the number must come from
cal = []
with open(os.path.join(PP, "prep_baseline_control", "out", "stepI_dplan_rule.log"), encoding="utf-8") as f:
    block = None
    for line in f:
        if line.startswith("  HARVEST S"):
            block = line.split()[1]
        elif block and line.strip().startswith("A "):
            parts = line.split()
            cal.append((block, int(parts[1]), float(parts[-1])))
        elif line.strip().startswith("estimator error"):
            block = None
errs = [c[2] for c in cal]
N["kpi_calibration"] = {
    "n_aircraft": len(errs),
    "cells": sorted({c[0] for c in cal}),
    "err_pct_min": min(errs),
    "err_pct_max": max(errs),
    "err_pct_mean": round(sum(errs) / len(errs), 2),
    "S1_n4_only_err_pct": [c[2] for c in cal if c[0] == "S1/n=4"],
    "per_aircraft": cal,
    "note": "positive = the estimator OVERstates the cost (the strict direction)",
}

# ------------------------------------------------------- 9. wind cost A vs B
w05, w00, w55 = N["sorties"]["HV4W05"], N["sorties"]["HV4W00"], N["sorties"]["HV4W55"]
N["wind_cost"] = {
    "A_cost_of_the_air": {
        "pair": "HV4W05 minus HV4W00 (identical mission files, only SIM_WIND differs)",
        "d_makespan_2m_s": round(w05["flown_makespan_2m_s"] - w00["flown_makespan_2m_s"], 3),
        "d_makespan_2m_pct": round(
            100 * (w05["flown_makespan_2m_s"] - w00["flown_makespan_2m_s"]) / w00["flown_makespan_2m_s"], 3
        ),
        "d_makespan_disarm_s": round(w05["flown_makespan_disarm_s"] - w00["flown_makespan_disarm_s"], 3),
        "d_fleet_km": round(w05["flown_total_km"] - w00["flown_total_km"], 4),
        "prediction_err_still_air_pct": w00["makespan_err_2m_pct"],
        "prediction_err_in_wind_pct": w05["makespan_err_2m_pct"],
        "per_uav_err_still_air_pct": {u: v["land_err_2m_pct"] for u, v in w00["per_uav"].items()},
        "per_uav_err_in_wind_pct": {u: v["land_err_2m_pct"] for u, v in w05["per_uav"].items()},
    },
    "B_cost_of_ignoring_the_wind_model": {
        "pair": "HV4W05 vs HV4W55 (same 5 m/s air; one plan knows, one does not)",
        "d_flown_makespan_s": round(w05["flown_makespan_2m_s"] - w55["flown_makespan_2m_s"], 3),
        "d_flown_makespan_pct": round(
            100 * (w05["flown_makespan_2m_s"] - w55["flown_makespan_2m_s"]) / w55["flown_makespan_2m_s"], 3
        ),
        "d_planned_makespan_s": round(w05["planned_makespan_s"] - w55["planned_makespan_s"], 3),
        "prediction_err_wind_aware_pct": w55["makespan_err_2m_pct"],
        "prediction_err_wind_ignorant_pct": w05["makespan_err_2m_pct"],
        "per_uav_err_wind_aware_pct": {u: v["land_err_2m_pct"] for u, v in w55["per_uav"].items()},
        "per_uav_err_wind_ignorant_pct": {u: v["land_err_2m_pct"] for u, v in w05["per_uav"].items()},
        "A1_release_delay_s": round(w55["per_uav"]["A1"]["plan_t_takeoff_s"] - w05["per_uav"]["A1"]["plan_t_takeoff_s"], 2),
        "A1_airborne_wind_aware_s": round(w55["per_uav"]["A1"]["flown_t_land_2m_s"] - w55["per_uav"]["A1"]["flown_t_release_s"], 3),
        "A1_airborne_wind_ignorant_s": round(w05["per_uav"]["A1"]["flown_t_land_2m_s"] - w05["per_uav"]["A1"]["flown_t_release_s"], 3),
        "d_fleet_km": round(w05["flown_total_km"] - w55["flown_total_km"], 4),
    },
}
d = N["wind_cost"]["B_cost_of_ignoring_the_wind_model"]
d["d_A1_airborne_s"] = round(d["A1_airborne_wind_aware_s"] - d["A1_airborne_wind_ignorant_s"], 3)

# --------------------------------------------- 10. gust NO_GO (planner-level)
gl = j(os.path.join(PP, "c2_wind", "evidence", "r_eff_and_gust_limits.json"))
N["gust_no_go"] = {
    "rule": gl["A_Pioneer"]["adapter_gust_rule"],
    "site_gust_max_mps": gl["A_Pioneer"]["gust_site_max_mps"][0],
    "max_wind_before_NOGO_mps": gl["A_Pioneer"]["max_wind_before_gust_NOGO_mps"],
    "measured_refusal_at_W7": "AeroPlanAdapterError NO_GO: gust_exceeds(H: 9.1 > 8.0 m/s) "
    "(PAPERPLAN1/c2_wind/evidence/planner_wind_sweep_2_4_6_7.json, row wind_mps=7.0)",
    "arithmetic_at_W8_mps": round(1.3 * 8.0, 2),
    "hero_crosswind_site_max_mps": gl["B_HERO"]["crosswind_site_max_mps"][0],
}

# -------------------------------------------- 11. planned-side grid-phase risk
N["planned_sep_is_also_a_bound"] = {
    tr: sep[tr]["grid_cost_ecef_frame"]["by_grid"] for tr in GUI_TRIALS if "grid_cost_ecef_frame" in sep[tr]
}

os.makedirs(OUT, exist_ok=True)
with open(os.path.join(OUT, "NUMBERS.json"), "w", encoding="utf-8") as f:
    json.dump(N, f, ensure_ascii=False, indent=1, sort_keys=True)

# ------------------------------------------------------------------- stdout
print("NUMBERS.json written to", os.path.join(OUT, "NUMBERS.json"))
print()
print("TABLE 2 candidate (main result = HV4W55, 119.5 m ceiling, plan 5@270 / sim 5@270)")
m = N["sorties"]["HV4W55"]
print("  makespan planned %.2f -> flown %.2f s (%.2f %%)" % (m["planned_makespan_s"], m["flown_makespan_2m_s"], m["makespan_err_2m_pct"]))
print("  fleet km planned %.3f -> flown %.3f (%.2f %%)" % (m["planned_total_km"], m["flown_total_km"], m["total_dist_err_pct"]))
print("  min sep planned %.3f -> flown %.3f m  pair %s  d_v %.3f  viol %d" % (
    m["planned_min_sep_h_m"], m["flown_min_sep_h_exact_m"], m["flown_min_sep_binding_pair"], m["d_v_at_min_h_m"], m["n_violations"]))
print("  max alt exported %.4f -> flown %.4f  margin to 120 %.4f  t>120 %.2f s  items>120 %d" % (
    m["max_exported_rel_alt_m"], m["max_flown_rel_alt_m"], m["margin_to_120_000_m"], m["time_above_120_000_fleet_s"], m["items_above_120_total"]))
print("  coverage", m["coverage"])
print()
print("BEAM-IDP fleet turn time", N["beam_idp"]["fleet_turn_time_s"], "->", N["beam_idp"]["bid_vs_seq_time_pct"], "%")
print("  sequential makespan estimate CHOSEN %.2f s (+%.2f %%); rejected recipe reproduces %.2f s" % (
    N["beam_idp"]["makespan_sequential_estimate_CHOSEN_s"],
    N["beam_idp"]["makespan_sequential_estimate_CHOSEN_pct"],
    N["beam_idp"]["makespan_sequential_estimate_REJECTED_reproduced_s"]))
for lab, v in N["paper_scenario_orders"].items():
    print("  TableY", lab, "orders identical:", v["ALL_FOUR_ORDERS_IDENTICAL"], "costs identical:", v["ALL_FOUR_COSTS_IDENTICAL"])
print()
print("KPI energy: other cells", N["kpi_energy"]["other_cells_frac_pct_range"], "%; over:", N["kpi_energy"]["cells_over_D_model"])
print("KPI calibration band: %.2f %% .. %+.2f %% (mean %+.2f %%) over %d aircraft" % (
    N["kpi_calibration"]["err_pct_min"], N["kpi_calibration"]["err_pct_max"],
    N["kpi_calibration"]["err_pct_mean"], N["kpi_calibration"]["n_aircraft"]))
print()
print("R_eff:", {k: (v["w_star_mps"], v["R_eff_by_wind_m"], v["k_by_wind_s96591"]) for k, v in N["r_eff"].items() if k.startswith(("A_", "B_"))})
print("survey area ellipsoidal %.1f m2 vs flat %.1f m2" % (N["survey_area"]["area_m2_ellipsoidal"], N["survey_area"]["area_m2_flat"]))
print("strip geometry", N["strip_geometry"])
