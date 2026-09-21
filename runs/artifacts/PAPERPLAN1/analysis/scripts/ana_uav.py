# -*- coding: utf-8 -*-
"""Stage 2a: per-aircraft kinematics, altitude compliance and wind proof."""
from __future__ import annotations

import numpy as np

from ana_lib import (AIRBORNE_M, CEIL_LEGAL_M, CEIL_TARGET_M, OUT, ROOT, UAVS,
                     airborne_window, circ_median_deg, csv_series,
                     disarm_touchdown_from_csv, first_auto_t, gps_clock, jdump,
                     jload, load, path_len, time_above, vec_to_from_dir)

TRIALS = {"HV4W55": 15, "HV4W05": 16, "HV4W00": 17, "BEAMIDP1": 18,
          "HV4SITL15": 13, "HV4SITL16": 14}
CRUISE_AGL_M = 100.0          # fixed-wing survey band; below this it is VTOL/transition


def wind_lines(d: dict, t_air: tuple[float, float]) -> dict:
    """Two dataflash-native readings of the air mass, per aircraft."""
    g, a, p = d["gps"], d["arsp"], d["pos"]
    t0, t1 = t_air
    agl = np.interp(g[:, 0], p[:, 0], p[:, 4])
    aspd = np.interp(g[:, 0], a[:, 0], a[:, 1])
    ok = (g[:, 0] >= t0) & (g[:, 0] <= t1) & (agl > CRUISE_AGL_M) & (aspd > 10.0)
    gs, crs, va = g[ok, 3], g[ok, 4] % 360.0, aspd[ok]
    diff = gs - va
    sect = {"east": (45.0, 135.0), "south": (135.0, 225.0), "west": (225.0, 315.0)}
    med, cnt = {}, {}
    for k, (lo, hi) in sect.items():
        m = (crs >= lo) & (crs < hi)
        cnt[k] = int(m.sum())
        med[k] = float(np.median(diff[m])) if m.any() else None
    mn = (crs >= 315.0) | (crs < 45.0)
    med["north"] = float(np.median(diff[mn])) if mn.any() else None
    cnt["north"] = int(mn.sum())

    def sub(a1, a2):
        return None if (a1 is None or a2 is None) else a1 - a2

    ew = sub(med["east"], med["west"])
    ns = sub(med["north"], med["south"])
    half = (None if (med["east"] is None or med["west"] is None)
            else 0.5 * (med["east"] + med["west"]))
    l1 = {
        "n_cruise_gps_samples": int(ok.sum()),
        "median_airspeed_mps": round(float(np.median(va)), 3) if len(va) else None,
        "median_groundspeed_mps": round(float(np.median(gs)), 3) if len(gs) else None,
        "median_gs_minus_as_all_mps": round(float(np.median(diff)), 3) if len(diff) else None,
        "by_sector_median_gs_minus_as_mps": {k: (round(v, 3) if v is not None else None)
                                             for k, v in med.items()},
        "n_by_sector": cnt,
        "east_minus_west_mps": round(ew, 3) if ew is not None else None,
        "north_minus_south_mps": round(ns, 3) if ns is not None else None,
        "implied_W_along_EW_mps": round(ew / 2.0, 3) if ew is not None else None,
        "implied_W_along_NS_mps": round(ns / 2.0, 3) if ns is not None else None,
        "scale_offset_removed_mps": round(half, 3) if half is not None else None,
        "note": ("east-minus-west cancels any heading-independent groundspeed/airspeed "
                 "scale offset exactly; the half-sum is that offset"),
    }
    l2 = {}
    for key, name in (("xkf2", "EKF3"), ("nkf2", "EKF2")):
        w = d.get(key)
        if w is None or not len(w):
            l2[name] = None
            continue
        agl_w = np.interp(w[:, 0], p[:, 0], p[:, 4])
        m = (w[:, 0] >= t0) & (w[:, 0] <= t1) & (agl_w > CRUISE_AGL_M)
        if not m.any():
            l2[name] = None
            continue
        vn, ve = float(np.median(w[m, 1])), float(np.median(w[m, 2]))
        spd, frm = vec_to_from_dir(vn, ve)
        l2[name] = {"n": int(m.sum()), "median_VWN_mps": round(vn, 3),
                    "median_VWE_mps": round(ve, 3), "speed_mps": round(spd, 3),
                    "from_deg": round(frm, 1),
                    "source": f"dataflash {key.upper()} core 0, cruise band only"}
    return {"L1_kinematic_dataflash": l1, "L2_ekf_state_dataflash": l2}


def wind_l3_telemetry(trial: str, uav: str) -> dict:
    s = csv_series(trial, uav)
    if "wspd" not in s:
        return {"available": False,
                "note": "flight.csv has no wind_spd_est column (pre-patch run)"}
    m = s["alt"] > CRUISE_AGL_M
    if not m.any():
        return {"available": True, "n": 0}
    spd, dirs = s["wspd"][m], s["wdir"][m]
    return {"available": True, "n": int(m.sum()),
            "median_speed_mps": round(float(np.median(spd)), 3),
            "circular_median_from_deg": round(circ_median_deg(dirs), 1),
            "arithmetic_mean_from_deg": round(float(np.mean(dirs)), 1),
            "source": f"runs/artifacts/{trial}/{uav}/flight.csv MAVLink WIND, ~1 Hz",
            "note": ("independent transport (telemetry), same estimator as L2; the "
                     "arithmetic mean is shown only to expose the angle-wrap artefact")}


def per_uav(trial: str, uav: str, home_lat: float) -> dict:
    d = load(trial, uav)
    p = d["pos"]
    t, lat, lon, amsl, agl = p[:, 0], p[:, 1], p[:, 2], p[:, 3], p[:, 4]
    off, spread, ngps = gps_clock(d)
    t0a, t1a = airborne_window(t, agl)
    air = (t >= t0a) & (t <= t1a)
    imax = int(np.argmax(agl))
    r = {
        "log": f"sitl/vtol/i{100 + UAVS.index(uav)}/logs/{TRIALS[trial]:08d}.BIN",
        "n_pos": int(len(t)),
        "pos_rate_hz": round(len(t) / (t[-1] - t[0]), 2),
        "pos_median_dt_s": round(float(np.median(np.diff(t))), 4),
        "pos_max_dt_s": round(float(np.diff(t).max()), 3),
        "gps_offset_s": round(off, 4), "gps_offset_spread_s": round(spread, 4),
        "n_gps": ngps,
        "t_auto_boot_s": first_auto_t(d),
        "t_first_airborne_boot_s": round(t0a, 3),
        "t_touchdown_2m_boot_s": round(t1a, 3),
        "airborne_duration_s": round(t1a - t0a, 3),
        "max_rel_alt_m": round(float(agl.max()), 4),
        "t_at_max_rel_alt_boot_s": round(float(t[imax]), 2),
        "max_amsl_m": round(float(amsl.max()), 4),
        "amsl_at_max_rel_alt_m": round(float(amsl[imax]), 4),
        "home_amsl_implied_m": round(float(amsl[imax] - agl[imax]), 4),
        "margin_to_120_000_m": round(CEIL_TARGET_M - float(agl.max()), 4),
        "margin_to_121_92_m": round(CEIL_LEGAL_M - float(agl.max()), 4),
        "time_above_120_000_s": round(time_above(t, agl, CEIL_TARGET_M), 3),
        "time_above_121_92_s": round(time_above(t, agl, CEIL_LEGAL_M), 3),
        "p99_rel_alt_m": round(float(np.percentile(agl[air], 99)), 4),
        "p999_rel_alt_m": round(float(np.percentile(agl[air], 99.9)), 4),
        "flown_km_dataflash_repo_frame": round(
            path_len(lat[air], lon[air], home_lat, "repo") / 1000.0, 4),
        "flown_km_dataflash_planner_frame": round(
            path_len(lat[air], lon[air], home_lat, "planner") / 1000.0, 4),
    }
    s = csv_series(trial, uav)
    sa = s["alt"] > AIRBORNE_M
    r["max_rel_alt_m_csv_1hz"] = round(float(s["alt"].max()), 4)
    r["alt_peak_missed_by_1hz_m"] = round(float(agl.max()) - float(s["alt"].max()), 4)
    r["flown_km_csv_1hz_repo_frame"] = round(
        path_len(s["lat"][sa], s["lon"][sa], home_lat, "repo") / 1000.0, 4)
    r["csv_median_dt_s"] = round(float(np.median(np.diff(s["t"]))), 3)
    r["csv_max_dt_s"] = round(float(np.diff(s["t"]).max()), 3)
    td, artefact = disarm_touchdown_from_csv(trial, uav)
    r["t_touchdown_disarm_csv_boot_s"] = td
    r["disarm_touchdown_is_last_csv_row"] = artefact
    r["wind"] = wind_lines(d, (t0a, t1a))
    r["wind"]["L3_telemetry_flight_csv"] = wind_l3_telemetry(trial, uav)
    return r


def main() -> None:
    res = {}
    for trial in TRIALS:
        sc = jload(ROOT / "runs" / "artifacts" / trial / "scenario.json")
        ms = jload(ROOT / "runs" / "artifacts" / trial / "plan" / "mission_summary.json")
        home_lat = float(ms["home"]["lat_deg"])
        res[trial] = {"home_lat_deg": home_lat,
                      "scenario_key": sc["key"],
                      "scenario_altitude_m": sc["altitude_m"],
                      "scenario_plan_wind": sc["wind"],
                      "scenario_sim_wind": sc.get("sim_wind"),
                      "per_uav": {u: per_uav(trial, u, home_lat) for u in UAVS}}
        print(trial, "ok")
    jdump(OUT / "uav_kinematics.json", res)


if __name__ == "__main__":
    main()
