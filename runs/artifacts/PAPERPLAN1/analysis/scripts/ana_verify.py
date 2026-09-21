# -*- coding: utf-8 -*-
"""Stage 4: independent cross-checks of the two numbers this analysis changed.

V1  the closest-approach distance, recomputed with a geodesic (Vincenty
    inverse on WGS84) between the two interpolated positions at the instant of
    closest approach, instead of any local-tangent projection.
V2  the reporter's own makespan recipe (single median clock offset over the
    1 Hz CSV), reproduced here so the difference from the per-aircraft GPS
    clock used in ana_time.py is attributable rather than asserted.
V3  the survey-area scale: which metres-per-degree the scenario polygon was
    actually written with.
"""
from __future__ import annotations

import csv
import math

import numpy as np

from ana_lib import OUT, ROOT, UAVS, gps_clock, jdump, jload, load

TRIALS = ("HV4W55", "HV4W05", "HV4W00", "BEAMIDP1")


def vincenty(lat1, lon1, lat2, lon2):
    a, f = 6378137.0, 1 / 298.257223563
    b = (1 - f) * a
    L = math.radians(lon2 - lon1)
    U1, U2 = math.atan((1 - f) * math.tan(math.radians(lat1))), math.atan((1 - f) * math.tan(math.radians(lat2)))
    sU1, cU1, sU2, cU2 = math.sin(U1), math.cos(U1), math.sin(U2), math.cos(U2)
    lam = L
    for _ in range(200):
        sl, cl = math.sin(lam), math.cos(lam)
        ss = math.hypot(cU2 * sl, cU1 * sU2 - sU1 * cU2 * cl)
        if ss == 0:
            return 0.0
        cs = sU1 * sU2 + cU1 * cU2 * cl
        sig = math.atan2(ss, cs)
        sa = cU1 * cU2 * sl / ss
        c2a = 1 - sa * sa
        c2sm = cs - 2 * sU1 * sU2 / c2a if c2a != 0 else 0.0
        C = f / 16 * c2a * (4 + f * (4 - 3 * c2a))
        lam_p = lam
        lam = L + (1 - C) * f * sa * (sig + C * ss * (c2sm + C * cs * (-1 + 2 * c2sm ** 2)))
        if abs(lam - lam_p) < 1e-14:
            break
    u2 = c2a * (a * a - b * b) / (b * b)
    A = 1 + u2 / 16384 * (4096 + u2 * (-768 + u2 * (320 - 175 * u2)))
    B = u2 / 1024 * (256 + u2 * (-128 + u2 * (74 - 47 * u2)))
    dsig = B * ss * (c2sm + B / 4 * (cs * (-1 + 2 * c2sm ** 2)
                                     - B / 6 * c2sm * (-3 + 4 * ss ** 2) * (-3 + 4 * c2sm ** 2)))
    return b * A * (sig - dsig)


def v1(trial: str, sep: dict) -> dict:
    fr = sep["frames"]["ecef"]
    pair = fr["binding_pair"]
    a, b = pair.split("-")
    t = fr["pairs"][pair]["t_gps_at_min_h_s"]
    pos = {}
    for u in (a, b):
        d = load(trial, u)
        off, _, _ = gps_clock(d)
        p = d["pos"]
        tc = p[:, 0] + off
        pos[u] = (float(np.interp(t, tc, p[:, 1])), float(np.interp(t, tc, p[:, 2])),
                  float(np.interp(t, tc, p[:, 4])))
    g = vincenty(pos[a][0], pos[a][1], pos[b][0], pos[b][1])
    return {"pair": pair, "t_gps_s": t,
            "lat_lon": {a: pos[a][:2], b: pos[b][:2]},
            "geodesic_m": round(g, 4),
            "ecef_enu_m": fr["pairs"][pair]["min_h_m"],
            "repo_frame_m": sep["frames"]["repo"]["pairs"][pair]["min_h_m"],
            "geodesic_minus_ecef_m": round(g - fr["pairs"][pair]["min_h_m"], 4),
            "repo_frame_optimism_m": round(sep["frames"]["repo"]["pairs"][pair]["min_h_m"] - g, 4),
            "d_v_m": fr["pairs"][pair]["d_v_at_min_h_m"]}


def v2(trial: str) -> dict:
    """hv_sitl_report.py:83-113 + :68-80, reproduced verbatim in method."""
    run = ROOT / "runs" / "artifacts" / trial
    rel = jload(run / "release_schedule.json")
    sp = float(rel.get("speedup") or 1.0)
    act = rel.get("actual_release_wall_s") or {}
    offs, td = [], {}
    for u in UAVS:
        rows = [r for r in csv.DictReader((run / u / "flight.csv").open(encoding="utf-8")) if r["t_s"]]
        auto = [r for r in rows if r["mode"] == "AUTO"]
        if u in act and auto:
            offs.append(float(auto[0]["t_s"]) - float(act[u]) * sp)
        peak = 0.0
        for r in rows:
            if not r["lat"]:
                continue
            a = float(r["rel_alt_m"])
            peak = max(peak, a)
            if peak > 20.0 and a < 2.0 and r["armed"].strip().lower() in ("false", "0"):
                td[u] = float(r["t_s"])
                break
    offs.sort()
    off = offs[len(offs) // 2]
    return {"median_offset_s": round(off, 3),
            "per_uav_offsets_s": [round(o, 3) for o in offs],
            "offset_spread_s": round(offs[-1] - offs[0], 3),
            "reporter_makespan_s": round(max(td.values()) - off, 3),
            "reporter_t_land_s": {u: round(v - off, 3) for u, v in td.items()}}


def v3() -> dict:
    sc = jload(ROOT / "runs/artifacts/HV4W55/scenario.json")
    poly = sc["areas"][0]["polygon"]
    lats = sorted({p[0] for p in poly})
    lons = sorted({p[1] for p in poly})
    dlat, dlon = lats[-1] - lats[0], lons[-1] - lons[0]
    phi = math.radians(sum(lats) / 2)
    return {
        "polygon_dlat_deg": dlat, "polygon_dlon_deg": dlon,
        "implied_m_per_deg_lat_if_800m": round(800.0 / dlat, 2),
        "implied_m_per_deg_lon_if_1560m": round(1560.0 / dlon, 2),
        "repo_m_per_deg_lat": 111132.0,
        "repo_m_per_deg_lon": round(111412.0 * math.cos(phi), 2),
        "ellipsoidal_m_per_deg_lat": round(111132.954 - 559.822 * math.cos(2 * phi)
                                           + 1.175 * math.cos(4 * phi), 2),
        "true_north_south_extent_m": round(dlat * (111132.954 - 559.822 * math.cos(2 * phi)
                                                   + 1.175 * math.cos(4 * phi)), 2),
        "nominal_north_south_extent_m": 800.0,
        "note": ("the scenario polygon was written with the flat 111132 m/deg, so the "
                 "nominal 1560 x 800 m area is 1560 x 797.3 m on the ellipsoid; the "
                 "planner then reads it through the exact GeoFrame, so the planner's own "
                 "frame is the ellipsoidal one and the 100 m rule lives there"),
    }


def main() -> None:
    S = jload(OUT / "separation.json")
    res = {"V1_geodesic_closest_approach": {t: v1(t, S[t]) for t in TRIALS},
           "V2_reporter_makespan_recipe": {t: v2(t) for t in TRIALS},
           "V3_survey_area_scale": v3()}
    jdump(OUT / "verify.json", res)
    for t in TRIALS:
        a, b = res["V1_geodesic_closest_approach"][t], res["V2_reporter_makespan_recipe"][t]
        print(f"{t:9s} V1 geodesic {a['geodesic_m']:9.4f} vs ecef {a['ecef_enu_m']:9.4f} "
              f"(d={a['geodesic_minus_ecef_m']:+.4f}) repo optimism {a['repo_frame_optimism_m']:+.4f} | "
              f"V2 reporter makespan {b['reporter_makespan_s']:.2f} offsets {b['per_uav_offsets_s']}")
    print(json.dumps(res["V3_survey_area_scale"], indent=1))


if __name__ == "__main__":
    import json
    main()
