# -*- coding: utf-8 -*-
"""label update_paper / stage 5 -- independent check of the HV4W55S spawn creep.

The sorties2 label diagnosed it; this label re-derives the load-bearing facts
from primary data so the paper's annotation does not rest on a summary:

 A. is the displacement REAL motion or an estimator artefact?
    -> compare the first logged SIM (simulator truth) and POS (EKF) samples.
 B. is it locked to the wind, or to the body axis?
    -> regress the displacement-increment bearing of the pre-arm settle trace
       against the aircraft's own contemporaneous heading.  Steps shorter than
       2 cm are DISCARDED: the trace stores lat/lon to 1e-7 deg ~ 1.1 cm, so
       their bearings are quantisation noise, not data.
 C. does the airframe weathervane on the ground?
    -> SIM.Yaw over the parked window (arming -> first lift-off).
"""
from __future__ import annotations

import math

import numpy as np

from up_lib import OUT, ROOT, UAVS, enu, jdump, jload, load

ART = ROOT / "runs" / "artifacts"
MIN_STEP_M = 0.10          # the trace stores lat/lon to 1e-7 deg ~ 1.1 cm, so a 3 cm
                           # step is 3 quanta and its bearing is meaningless; 0.10 m is
                           # well below the genuine 0.41-0.50 m steps and well above noise


def main() -> None:
    sp = jload(ART / "HV4W55S" / "spawn_readback.json")
    want = {r["uav"]: (float(r["want_lat"]), float(r["want_lon"])) for r in sp["rows"]}
    out = {"A_truth_vs_estimate": {}, "B_body_axis_arc": {}, "C_ground_weathervane": {},
           "method": {"min_step_m": MIN_STEP_M,
                      "note": "the dataflash only starts at arming, so the arc itself can "
                              "only come from the runner's pre-arm settle_trace; the "
                              "dataflash independently fixes the END STATE."}}
    for u in UAVS:
        d = load("HV4W55S", u)
        s, p = d["sim"], d["pos"]
        la, lo = want[u]
        se, sn = enu(s[:1, 1], s[:1, 2], la, lo)
        pe, pn = enu(p[:1, 1], p[:1, 2], la, lo)
        out["A_truth_vs_estimate"][u] = {
            "sim_truth_first_sample": {"d_north_m": round(float(sn[0]), 4),
                                       "d_east_m": round(float(se[0]), 4)},
            "ekf_pos_first_sample": {"d_north_m": round(float(pn[0]), 4),
                                     "d_east_m": round(float(pe[0]), 4)},
            "truth_minus_ekf_m": round(float(math.hypot(sn[0] - pn[0], se[0] - pe[0])), 4)}
        # parked window: from the first log sample to the first time rel alt > 2 m
        rel = np.interp(s[:, 0], p[:, 0], p[:, 4])
        lift = np.flatnonzero(rel > 2.0)
        end = int(lift[0]) if len(lift) else len(s)
        yaw, alt = s[:end, 4], s[:end, 3]
        out["C_ground_weathervane"][u] = {
            "window_s": round(float(s[end - 1, 0] - s[0, 0]), 2),
            "n": int(end),
            "yaw_first_deg": round(float(yaw[0]), 2),
            "yaw_last_deg": round(float(yaw[-1]), 2),
            "yaw_monotonic_increasing": bool(np.all(np.diff(yaw) >= -0.05)),
            "sim_wind_dir_from_deg": jload(ART / "HV4W55S" / "scenario.json")["sim_wind"]["dir_from_deg"],
            "sim_alt_min_m": round(float(alt.min()), 3),
            "sim_alt_max_m": round(float(alt.max()), 3)}
    for r in sp["rows"]:
        tr = [x for x in r["settle_trace"]
              if abs(x["d_north_m"]) < 1e4 and abs(x["d_east_m"]) < 1e4]
        kept, resid, path = [], [], 0.0
        for a, b in zip(tr, tr[1:]):
            dn, de = b["d_north_m"] - a["d_north_m"], b["d_east_m"] - a["d_east_m"]
            step = math.hypot(dn, de)
            path += step
            if step < MIN_STEP_M:
                continue
            brg = math.degrees(math.atan2(de, dn)) % 360.0
            hdg = (a["heading_deg"] + b["heading_deg"]) / 2.0
            resid.append((brg - hdg + 180.0) % 360.0 - 180.0)
            kept.append({"dt_from_s": a["dt_s"], "dt_to_s": b["dt_s"], "step_m": round(step, 4),
                         "bearing_deg": round(brg, 2), "heading_deg": round(hdg, 2),
                         "residual_deg": round(resid[-1], 2),
                         "groundspeed_mps": a["groundspeed_mps"]})
        net = (math.hypot(tr[-1]["d_north_m"] - tr[0]["d_north_m"],
                          tr[-1]["d_east_m"] - tr[0]["d_east_m"]) if len(tr) > 1 else 0.0)
        out["B_body_axis_arc"][r["uav"]] = {
            "settle_s": r.get("settle_s"),
            "n_trace_rows": len(r["settle_trace"]),
            "n_steps_above_threshold": len(kept),
            "residual_median_deg": round(float(np.median(resid)), 2) if resid else None,
            "residual_max_abs_deg": round(float(np.max(np.abs(resid))), 2) if resid else None,
            "heading_swept_deg": ([round(kept[0]["heading_deg"], 2),
                                   round(kept[-1]["heading_deg"], 2)] if kept else None),
            "step_m_range": ([round(min(k["step_m"] for k in kept), 3),
                              round(max(k["step_m"] for k in kept), 3)] if kept else None),
            "path_length_m": round(path, 3), "net_chord_m": round(net, 3),
            "steps": kept,
            "usable": len(kept) >= 5,
            "why_not_usable": (None if len(kept) >= 5 else
                               "this aircraft settled before the trace began; its trace only "
                               "records sub-centimetre residual motion, so it fixes the END "
                               "STATE but says nothing about the path")}
    jdump(OUT / "spawn_verification.json", out)
    for u in UAVS:
        a = out["A_truth_vs_estimate"][u]
        b = out["B_body_axis_arc"][u]
        c = out["C_ground_weathervane"][u]
        print(f"{u}: truth dN {a['sim_truth_first_sample']['d_north_m']:.3f} "
              f"dE {a['sim_truth_first_sample']['d_east_m']:.3f}  "
              f"truth-EKF {a['truth_minus_ekf_m']:.4f} m | arc steps {b['n_steps_above_threshold']} "
              f"resid med {b['residual_median_deg']} max {b['residual_max_abs_deg']} "
              f"usable {b['usable']} | yaw {c['yaw_first_deg']} -> {c['yaw_last_deg']} "
              f"mono {c['yaw_monotonic_increasing']} over {c['window_s']} s, "
              f"alt {c['sim_alt_min_m']}-{c['sim_alt_max_m']}")


if __name__ == "__main__":
    main()
