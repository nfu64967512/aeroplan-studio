# -*- coding: utf-8 -*-
"""C5 step 6 -- the "ArduPilot mission export / SITL-executable" KPI axis.

Runs the project's real exporter (mission/vtol_mission_exporter.py) on the
DCCPP baseline's own AssembledPath for one benchmark cell and measures the
thing ArduPlane actually rejects missions for: the horizontal distance
between the last NAV_WAYPOINT and the NAV_VTOL_LAND point. ArduPlane's
PreArm refuses arming with "VTOL land too short, min <N>m".

Also records which segment labels the DCCPP path carries, because the
exporter's vertical-point guard is keyed on those labels.

usage: python step6_export_axis.py <SCEN> <N> [mix]
"""
import json
import math
import sys
import tempfile
from pathlib import Path

REPO = Path(r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
BENCH = REPO / "research/cpp_benchmark"
for p in (str(REPO), str(BENCH)):
    if p not in sys.path:
        sys.path.insert(0, p)

import logging                                               # noqa: E402
logging.disable(logging.CRITICAL)

import matrix as M                                           # noqa: E402
from bench import _drones, polys_to_areas                    # noqa: E402
from mission.swarm_coordinator import SwarmCoordinator       # noqa: E402
from mission.vtol_mission_exporter import VTOLMissionExporter  # noqa: E402

SCEN, N = sys.argv[1], int(sys.argv[2])
MIX = len(sys.argv) > 3 and sys.argv[3] == "mix"
OUT = Path(__file__).resolve().parents[1] / "out"
POLYS, _ = M.SCEN[SCEN]
N_A, N_B = (N - N // 2, N // 2) if MIX else (N, 0)

CMD_NAV_WAYPOINT, CMD_VTOL_TAKEOFF, CMD_VTOL_LAND = 16, 84, 85
CMD_DO_CHANGE_SPEED, CMD_DO_SET_HOME, CMD_DO_VTOL_TRANSITION = 178, 179, 3000


def hdist(a, b):
    return math.hypot((a[0] - b[0]) * 111320.0,
                      (a[1] - b[1]) * 111320.0 * math.cos(math.radians(a[0])))


drones, cls = _drones(N_A, N_B, M.PITCH)
c = SwarmCoordinator()
res = c.plan_coverage_dccpp(areas=polys_to_areas(POLYS), drones=drones,
                            coverage_width_m=M.PITCH, overlap_rate=0.0,
                            coverage_altitude=M.ALT, mounting_angle_deg=0.0)
ap = res.get("assembled_paths") or {}

exp = VTOLMissionExporter(transition_alt_m=60.0, cruise_speed_ms=21.5,
                          turn_radius_m=200.0, explicit_transitions=True)
rep = {"scenario": SCEN, "n": N, "mix": MIX, "n_paths": len(ap),
       "vtol_transition_alt_m_passed_to_planner": None,
       "note": ("plan_coverage_dccpp was called WITHOUT vtol_transition_alt_m "
                "(exactly as research/cpp_benchmark/bench.run_planner does), so the "
                "DCCPP path carries a RUNWAY takeoff/landing profile, not a vertical one."),
       "per_uav": []}

for uid, built in list(ap.items()):
    wps = built.waypoints
    segs = {}
    for w in wps:
        k = getattr(w.segment_type, "name", "?")
        segs[k] = segs.get(k, 0) + 1
    try:
        lines = exp.generate_waypoint_lines(wps, altitude=M.ALT, uav_id=int(uid))
        err = None
    except TypeError:
        # fall back: inspect the real signature and retry positionally
        import inspect
        sig = str(inspect.signature(exp.generate_waypoint_lines))
        lines, err = None, f"signature mismatch: generate_waypoint_lines{sig}"
    except Exception as e:                                   # noqa: BLE001
        lines, err = None, f"{type(e).__name__}: {e}"

    row = {"uav": uid, "cls": cls[int(uid)], "n_waypoints_in": len(wps),
           "segment_types": segs, "export_error": err}
    if lines:
        parsed = []
        for ln in lines:
            f = str(ln).strip().split("\t")
            if len(f) >= 12:
                parsed.append(dict(seq=int(f[0]), cmd=int(f[3]),
                                   lat=float(f[8]), lon=float(f[9]), alt=float(f[10])))
        row["n_mission_items"] = len(parsed)
        hist = {}
        for q in parsed:
            hist[q["cmd"]] = hist.get(q["cmd"], 0) + 1
        row["cmd_histogram"] = hist
        row["has_vtol_takeoff"] = CMD_VTOL_TAKEOFF in hist
        row["has_vtol_land"] = CMD_VTOL_LAND in hist
        land = next((q for q in reversed(parsed) if q["cmd"] == CMD_VTOL_LAND), None)
        prev_nav = None
        if land is not None:
            for q in parsed:
                if q["seq"] < land["seq"] and q["cmd"] == CMD_NAV_WAYPOINT:
                    prev_nav = q
        if land and prev_nav:
            d = hdist((prev_nav["lat"], prev_nav["lon"]), (land["lat"], land["lon"]))
            row["land_approach_m"] = round(d, 1)
            row["prearm_min_m"] = 108.0     # observed ArduPlane PreArm threshold
            row["prearm_would_block"] = bool(d < 108.0)
        else:
            row["land_approach_m"] = None
    rep["per_uav"].append(row)

n = [r for r in rep["per_uav"] if r.get("land_approach_m") is not None]
rep["summary"] = dict(
    exported=sum(1 for r in rep["per_uav"] if not r.get("export_error")),
    with_vtol_land=sum(1 for r in rep["per_uav"] if r.get("has_vtol_land")),
    prearm_blocked=sum(1 for r in n if r.get("prearm_would_block")),
    approach_min_m=(min(r["land_approach_m"] for r in n) if n else None),
    approach_max_m=(max(r["land_approach_m"] for r in n) if n else None))

tag = f"{SCEN}_n{N}" + ("_mix" if MIX else "")
f = OUT / f"step6_export_{tag}.json"
f.write_text(json.dumps(rep, indent=2, ensure_ascii=False, default=str), encoding="utf-8")
print(json.dumps(rep, indent=2, ensure_ascii=False, default=str)[:6000])
print("\nwrote", f)
