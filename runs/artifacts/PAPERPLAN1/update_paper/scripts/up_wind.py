# -*- coding: utf-8 -*-
"""label update_paper / stage 3 -- the wind evidence, on BOTH axes.

Two things the earlier labels did separately, done here in one place and from
the native-rate dataflash rather than the ~1 Hz flight.csv:

 1. the signed axis test.  delta = groundspeed - true airspeed, median per
    90 deg heading bin.  For a steady wind of speed W FROM bearing b,
    delta(theta) = W cos(theta - (b+180)), so
        b = 270 -> east-minus-west = +2W, north-minus-south ~ 0
        b = 180 -> north-minus-south = +2W, east-minus-west ~ 0
    The heading-independent groundspeed/airspeed scale bias cancels in the
    difference and shows up as the HALF-SUM of the two bins.
    The altitude floor is swept (100 m = the cruise band used by the C2/analysis
    labels, 30 m = the floor the c2_wind probe used) because on the 180 deg
    sortie the north-south legs are transit legs, and a 100 m floor starves
    them: B2's southbound bin keeps exactly ONE sample.  Both are reported with
    their n, and no median over fewer than 10 samples is quoted anywhere.

 2. the CROSS-TRACK half of the wind triangle, which only the 180 deg cell can
    show.  On an east-west strip a wind from 180 is pure crosswind, so the
    aircraft crabs and its groundspeed drops to sqrt(Va^2 - W^2).  Predicted
    half-sum = (no-wind half-sum of the same aircraft) + sqrt(Va^2 - W^2) - Va,
    with Va MEASURED (median true airspeed in the same band), not nominal.
"""
from __future__ import annotations

import json
import math

import numpy as np

from up_lib import OUT, UAVS, jdump, jload, load, vec_to_from

TRIALS = ("HV4W55", "HV4W55R", "HV4W55S", "HV4W05", "HV4W00", "BEAMIDP1",
          "HV4SITL15", "HV4SITL16")
FLOORS = (100.0, 30.0)
AS_MIN = 10.0
MIN_N = 10          # never quote a median from fewer samples than this


def axis(trial: str, floor: float) -> dict:
    res = {}
    for u in UAVS:
        d = load(trial, u)
        g, a, p = d["gps"], d["arsp"], d["pos"]
        rel = np.interp(g[:, 0], p[:, 0], p[:, 4])
        asp = np.interp(g[:, 0], a[:, 0], a[:, 1])
        ok = (rel > floor) & (asp > AS_MIN) & (g[:, 3] > AS_MIN)
        crs, dv, av = g[ok, 4] % 360.0, g[ok, 3] - asp[ok], asp[ok]
        b = {}
        for name, c in (("E", 90.0), ("W", 270.0), ("N", 0.0), ("S", 180.0)):
            sel = np.abs((crs - c + 180.0) % 360.0 - 180.0) <= 45.0
            b[name] = {"n": int(sel.sum()),
                       "median": round(float(np.median(dv[sel])), 4) if sel.sum() else None,
                       "usable": bool(sel.sum() >= MIN_N)}

        def pair(x, y):
            if b[x]["usable"] and b[y]["usable"]:
                return (round(b[x]["median"] - b[y]["median"], 4),
                        round((b[x]["median"] + b[y]["median"]) / 2.0, 4))
            return None, None
        ew, ew_half = pair("E", "W")
        ns, ns_half = pair("N", "S")
        res[u] = {"bins": b, "east_minus_west": ew, "ew_half_sum": ew_half,
                  "north_minus_south": ns, "ns_half_sum": ns_half,
                  "median_airspeed_mps": round(float(np.median(av)), 4) if len(av) else None,
                  "n_total": int(ok.sum())}
    return res


def ekf(trial: str) -> dict:
    res = {}
    for u in UAVS:
        d = load(trial, u)
        p = d["pos"]
        w = d["xkf2"] if ("xkf2" in d and len(d["xkf2"])) else d.get("nkf2")
        relw = np.interp(w[:, 0], p[:, 0], p[:, 4])
        sel = relw > 100.0
        spd, frm = vec_to_from(float(np.median(w[sel, 1])), float(np.median(w[sel, 2])))
        res[u] = {"spd_mps": round(spd, 4), "from_deg": round(frm, 2), "n": int(sel.sum()),
                  "note": "median of the N/E components, NOT a mean of the wrapping bearing"}
    return res


def crosstrack() -> dict:
    """What a pure crosswind does to along-strip groundspeed (180 deg cell)."""
    base = axis("HV4W00", 100.0)      # same fleet, same strips, no wind
    test = axis("HV4W55S", 100.0)
    W = 5.0
    out = {}
    for u in UAVS:
        va = test[u]["median_airspeed_mps"]
        crab = math.sqrt(va * va - W * W) - va
        pred = base[u]["ew_half_sum"] + crab
        meas = test[u]["ew_half_sum"]
        out[u] = {"no_wind_ew_half_sum": base[u]["ew_half_sum"],
                  "measured_airspeed_mps": va,
                  "pure_crab_deficit_mps": round(crab, 4),
                  "predicted_ew_half_sum": round(pred, 4),
                  "measured_ew_half_sum": meas,
                  "residual_mps": round(meas - pred, 4),
                  "n_east": test[u]["bins"]["E"]["n"], "n_west": test[u]["bins"]["W"]["n"]}
    return out


def main() -> None:
    res = {"floors": {}, "ekf": {}, "crosstrack_180": crosstrack(),
           "method": {"metric": "GPS.Spd - interpolated ARSP.Airspeed (I=0)",
                      "bins": "90 deg wide, centred on E/W/N/S, GPS.GCrs",
                      "filters": "rel alt > floor, airspeed > 10, groundspeed > 10",
                      "min_n_for_a_quoted_median": MIN_N}}
    for f in FLOORS:
        res["floors"][f"{f:.0f}m"] = {t: axis(t, f) for t in TRIALS}
    for t in TRIALS:
        res["ekf"][t] = ekf(t)
    jdump(OUT / "wind.json", res)
    for t in TRIALS:
        for f in FLOORS:
            r = res["floors"][f"{f:.0f}m"][t]
            print(f"{t:10s} floor{f:5.0f}  " + "  ".join(
                f"{u} EW {str(r[u]['east_minus_west']):>8s}(n{r[u]['bins']['E']['n']}/{r[u]['bins']['W']['n']}) "
                f"NS {str(r[u]['north_minus_south']):>8s}(n{r[u]['bins']['N']['n']}/{r[u]['bins']['S']['n']})"
                for u in UAVS))
    print("\nCROSS-TRACK (180 deg):")
    for u, v in res["crosstrack_180"].items():
        print(f"  {u} nowind {v['no_wind_ew_half_sum']:+.4f}  Va {v['measured_airspeed_mps']:.3f} "
              f"crab {v['pure_crab_deficit_mps']:+.4f} -> pred {v['predicted_ew_half_sum']:+.4f} "
              f"meas {v['measured_ew_half_sum']:+.4f}  resid {v['residual_mps']:+.4f}")


if __name__ == "__main__":
    main()
