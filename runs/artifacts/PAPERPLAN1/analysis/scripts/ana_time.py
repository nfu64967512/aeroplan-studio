# -*- coding: utf-8 -*-
"""Stage 2c: planned vs flown timing, distance and coverage.

Fleet clock.  Each SITL instance counts TimeUS from its own boot and the three
processes start ~1 s apart, so hv_sitl_report.py's single median clock_offset
(runs/artifacts/<T>/hv_sitl_report.py:83-113) mixes three different zeros.
Here every aircraft is put on GPS time-of-week and fleet t = 0 is the AUTO-mode
entry of the aircraft whose PLANNED take-off delay is 0, which is exactly the
plan's own time origin.

Touchdown, two definitions:
  td_2m     last downward crossing of RelHomeAlt = 2.0 m, dataflash ~6 Hz.
            Primary: clean, uniform, and independent of the monitor loop.
  td_disarm hv_sitl_report.py's rule (below 2 m AND disarmed, ~1 Hz CSV).
            For the LAST aircraft to land this is contaminated - the monitor
            loop stops at that instant, so the "disarmed" sample is the final
            CSV row.  flagged per aircraft.
"""
from __future__ import annotations

import numpy as np
import shapely
from shapely.geometry import Polygon
from shapely.ops import unary_union

from ana_lib import (OUT, ROOT, UAVS, airborne_window, disarm_touchdown_from_csv,
                     first_auto_t, gps_clock, jdump, jload, load, m_per_deg,
                     path_len)

TRIALS = ("HV4W55", "HV4W05", "HV4W00", "BEAMIDP1", "HV4SITL15", "HV4SITL16")
SURVEY_AGL_M = 100.0


def coverage(trial: str, lat0: float, lon0: float, swath_m: float,
             survey_only: bool) -> list[dict]:
    """Track-buffer coverage recomputed from dataflash POS (~6 Hz)."""
    sc = jload(ROOT / "runs" / "artifacts" / trial / "scenario.json")
    ml, mo = m_per_deg(lat0, "ecef_proxy" if False else "planner")
    lines = []
    for u in UAVS:
        p = load(trial, u)["pos"]
        t0, t1 = airborne_window(p[:, 0], p[:, 4])
        m = (p[:, 0] >= t0) & (p[:, 0] <= t1)
        if survey_only:
            m &= p[:, 4] > SURVEY_AGL_M
        xy = np.column_stack([(p[m, 2] - lon0) * mo, (p[m, 1] - lat0) * ml])
        # split at gaps so a pause is not bridged by a straight line
        br = np.flatnonzero(np.diff(p[m, 0]) > 5.0)
        for seg in np.split(xy, br + 1):
            if len(seg) >= 2:
                lines.append(shapely.LineString(seg))
    if not lines:
        return []
    band = unary_union([ln.buffer(swath_m / 2.0, cap_style=2) for ln in lines])
    out = []
    for a in sc["areas"]:
        ring = [((lo - lon0) * mo, (la - lat0) * ml) for la, lo in a["polygon"]]
        poly = Polygon(ring)
        out.append({"area": f"Z{a['area_id']}",
                    "area_m2": round(poly.area, 1),
                    "area_ha": round(poly.area / 1e4, 2),
                    "covered_m2": round(poly.intersection(band).area, 1),
                    "ratio": round(poly.intersection(band).area / poly.area, 6)})
    return out


def main() -> None:
    matrix = jload(ROOT / "runs" / "artifacts" / "PAPERPLAN1" / "prep_runner" / "plan_matrix.json")
    cell_of = {"HV4W55": "W55_c1195", "HV4W05": "W05_c1195", "HV4W00": "W00_c1195"}
    res = {}
    for trial in TRIALS:
        run = ROOT / "runs" / "artifacts" / trial
        ms = jload(run / "plan" / "mission_summary.json")
        sc = jload(run / "scenario.json")
        rel = jload(run / "release_schedule.json")
        lat0, lon0 = float(ms["home"]["lat_deg"]), float(ms["home"]["lon_deg"])
        delays = {u: float(ms["uavs"][u]["t_takeoff_s"]) for u in UAVS}
        zero_uav = min(delays, key=lambda u: delays[u])
        clocks, auto_c, td2_c, tdd_c, per = {}, {}, {}, {}, {}
        for u in UAVS:
            d = load(trial, u)
            off, spread, _ = gps_clock(d)
            p = d["pos"]
            ta, tb = airborne_window(p[:, 0], p[:, 4])
            clocks[u] = off
            auto_c[u] = first_auto_t(d) + off
            td2_c[u] = tb + off
            tdd, art = disarm_touchdown_from_csv(trial, u)
            tdd_c[u] = (tdd + off) if tdd is not None else None
            per[u] = {"gps_offset_s": round(off, 4), "gps_offset_spread_s": round(spread, 4),
                      "disarm_is_last_csv_row": art,
                      "t_first_airborne_boot_s": round(ta, 3)}
        t0 = auto_c[zero_uav] - delays[zero_uav]
        rows = {}
        for u in UAVS:
            pl = ms["uavs"][u]
            df = load(trial, u)["pos"]
            ta, tb = airborne_window(df[:, 0], df[:, 4])
            air = (df[:, 0] >= ta) & (df[:, 0] <= tb)
            km = path_len(df[air, 1], df[air, 2], lat0, "planner") / 1000.0
            rows[u] = {
                "cls": pl["cls"],
                "plan_t_takeoff_s": pl["t_takeoff_s"],
                "flown_t_release_s": round(auto_c[u] - t0, 3),
                "release_error_s": round(auto_c[u] - t0 - pl["t_takeoff_s"], 3),
                "plan_t_land_s": round(float(pl["t_land_s"]), 3),
                "flown_t_land_2m_s": round(td2_c[u] - t0, 3),
                "flown_t_land_disarm_s": (round(tdd_c[u] - t0, 3)
                                          if tdd_c[u] is not None else None),
                "land_err_2m_pct": round(100.0 * ((td2_c[u] - t0) / float(pl["t_land_s"]) - 1.0), 3),
                "plan_km": round(float(pl["distance_m"]) / 1000.0, 4),
                "flown_km": round(km, 4),
                "dist_err_pct": round(100.0 * (km * 1000.0 / float(pl["distance_m"]) - 1.0), 3),
                "plan_energy_used_wh": round(float(pl["energy_used_wh"]), 2),
                "plan_energy_margin_wh": round(float(pl["energy_margin_wh"]), 2),
                "plan_n_items": pl["n_items"],
                **per[u]}
        plan_mk = float(ms["makespan_s"])
        mk2 = max(r["flown_t_land_2m_s"] for r in rows.values())
        mkd = max(r["flown_t_land_disarm_s"] for r in rows.values()
                  if r["flown_t_land_disarm_s"] is not None)
        swath = 2.0 * float(sc["altitude_m"]) * np.tan(np.radians(float(sc["hfov_deg"]) / 2.0))
        cov_f = run / "coverage_from_tracks.json"
        cov_rep = jload(cov_f) if cov_f.exists() else {}
        entry = {
            "fleet_zero_uav": zero_uav,
            "t0_gps_tow_s": round(t0, 3),
            "planned_makespan_s": plan_mk,
            "planned_makespan_matrix_s": matrix[cell_of[trial]]["makespan_s"] if trial in cell_of else None,
            "flown_makespan_2m_s": round(mk2, 3),
            "flown_makespan_disarm_s": round(mkd, 3),
            "makespan_err_2m_pct": round(100.0 * (mk2 / plan_mk - 1.0), 3),
            "makespan_err_disarm_pct": round(100.0 * (mkd / plan_mk - 1.0), 3),
            "planned_total_km": round(sum(r["plan_km"] for r in rows.values()), 4),
            "flown_total_km": round(sum(r["flown_km"] for r in rows.values()), 4),
            "total_dist_err_pct": round(100.0 * (sum(r["flown_km"] for r in rows.values())
                                                 / sum(r["plan_km"] for r in rows.values()) - 1.0), 3),
            "swath_m_recomputed": round(float(swath), 3),
            "swath_m_reporter": cov_rep.get("swath_m"),
            "coverage_all_airborne": coverage(trial, lat0, lon0, float(swath), False),
            "coverage_survey_band_only": coverage(trial, lat0, lon0, float(swath), True),
            "coverage_reporter": cov_rep.get("areas"),
            "per_uav": rows,
            "release_schedule": rel,
        }
        res[trial] = entry
        print(f"{trial:9s} plan {plan_mk:8.2f} -> 2m {mk2:8.2f} ({entry['makespan_err_2m_pct']:+.2f}%) "
              f"| disarm {mkd:8.2f} ({entry['makespan_err_disarm_pct']:+.2f}%) "
              f"| km {entry['planned_total_km']:.3f} -> {entry['flown_total_km']:.3f} "
              f"({entry['total_dist_err_pct']:+.2f}%)")
    jdump(OUT / "timing.json", res)


if __name__ == "__main__":
    main()
