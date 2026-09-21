# -*- coding: utf-8 -*-
"""Exported-waypoint altitude census: how many items sit above the 120 m limit.

Reads runs/artifacts/<TRIAL>/plan/<UAV>.waypoints (QGC WPL 110). Only
MAV_FRAME_GLOBAL_RELATIVE_ALT (frame 3) rows carry an AGL altitude; the z of
other frames is not comparable, so they are counted separately, never mixed in.

    python exported_alt.py TRIAL [TRIAL ...]
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
OUT = ROOT / "runs" / "artifacts" / "PAPERPLAN1" / "sorties2"
LIMIT = 120.0


def main():
    res = {}
    for trial in sys.argv[1:]:
        per, fleet_max, fleet_over = {}, None, 0
        for wp in sorted((ROOT / "runs" / "artifacts" / trial / "plan").glob("*.waypoints")):
            uav = wp.stem
            rel, other = [], 0
            for ln in wp.read_text(encoding="utf-8").splitlines():
                f = ln.split("\t")
                if len(f) < 12 or not f[0].isdigit():
                    continue
                frame, z = int(f[2]), float(f[10])
                if frame == 3:
                    rel.append(z)
                else:
                    other += 1
            if not rel:
                continue
            over = [z for z in rel if z > LIMIT]
            per[uav] = {"n_items_total": len(rel) + other,
                        "n_rel_alt_items": len(rel), "n_other_frame_items": other,
                        "max_rel_alt_m": round(max(rel), 4),
                        "n_above_120": len(over),
                        "max_excess_m": round(max(over) - LIMIT, 4) if over else 0.0}
            fleet_max = max(rel) if fleet_max is None else max(fleet_max, max(rel))
            fleet_over += len(over)
        res[trial] = {"per_uav": per,
                      "fleet_max_exported_rel_alt_m": round(fleet_max, 4) if fleet_max else None,
                      "fleet_items_above_120": fleet_over}
        OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "exported_alt.json").write_text(json.dumps(res, ensure_ascii=False, indent=1),
                                           encoding="utf-8")
    print(json.dumps(res, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
