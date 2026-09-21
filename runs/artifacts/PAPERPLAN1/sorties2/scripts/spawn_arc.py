# -*- coding: utf-8 -*-
"""Is the pre-arm creep a straight roll, or an arc traced while weathervaning?

Test: for every consecutive pair of samples in the pre-arm settle trace
(spawn_readback.json, written by the patched runner), compare the bearing of the
POSITION INCREMENT against the vehicle's own contemporaneous HEADING.  A body-
forward creep gives increment bearing == heading at every step; a wind-driven
slide gives increment bearing == a fixed wind bearing regardless of heading.

    python spawn_arc.py <TRIAL> [TRIAL ...]
"""
from __future__ import annotations

import json
import math
import statistics
import sys
from pathlib import Path

ROOT = Path(r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
OUT = ROOT / "runs" / "artifacts" / "PAPERPLAN1" / "sorties2"


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def main():
    res = {}
    for trial in sys.argv[1:]:
        sr = json.loads((ROOT / "runs" / "artifacts" / trial /
                         "spawn_readback.json").read_text(encoding="utf-8"))
        per = {}
        for r in sr["rows"]:
            tr = [s for s in r.get("settle_trace", [])
                  if abs(s["d_north_m"]) < 1e4 and abs(s["d_east_m"]) < 1e4]
            steps = []
            for a, b in zip(tr, tr[1:]):
                dn, de = b["d_north_m"] - a["d_north_m"], b["d_east_m"] - a["d_east_m"]
                step = math.hypot(dn, de)
                if step < 0.10:                       # below the 1e-7 deg position quantum + noise
                    continue
                if a["heading_deg"] is None or b["heading_deg"] is None:
                    continue
                inc = math.degrees(math.atan2(de, dn)) % 360.0
                hdg = (a["heading_deg"] + wrap(b["heading_deg"] - a["heading_deg"]) / 2.0) % 360.0
                steps.append({"t_s": b["dt_s"], "step_m": round(step, 3),
                              "increment_bearing_deg": round(inc, 2),
                              "mean_heading_deg": round(hdg, 2),
                              "residual_deg": round(wrap(inc - hdg), 2)})
            if not steps:
                per[r["uav"]] = {"n_steps": 0}
                continue
            resid = [s["residual_deg"] for s in steps]
            # the alternative hypothesis: every increment points the same way
            incs = [s["increment_bearing_deg"] for s in steps]
            per[r["uav"]] = {
                "n_steps": len(steps),
                "heading_span_deg": [steps[0]["mean_heading_deg"], steps[-1]["mean_heading_deg"]],
                "increment_bearing_span_deg": [min(incs), max(incs)],
                "residual_vs_own_heading_deg": {
                    "median": round(statistics.median(resid), 2),
                    "max_abs": round(max(abs(x) for x in resid), 2)},
                "net_displacement_m": round(math.hypot(
                    tr[-1]["d_north_m"] - tr[0]["d_north_m"],
                    tr[-1]["d_east_m"] - tr[0]["d_east_m"]), 3),
                "path_length_m": round(sum(s["step_m"] for s in steps), 3),
                "creep_speed_mps_wallclock": round(
                    sum(s["step_m"] for s in steps) / (steps[-1]["t_s"] - tr[0]["dt_s"]), 4),
                "steps": steps}
        res[trial] = per
        (OUT / f"{trial}_spawn_arc.json").write_text(
            json.dumps({"trial": trial, "per_uav": per}, ensure_ascii=False, indent=1),
            encoding="utf-8")
    print(json.dumps(res, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
