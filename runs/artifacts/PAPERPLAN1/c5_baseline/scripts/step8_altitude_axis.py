# -*- coding: utf-8 -*-
"""C5 step 8 -- the "regulatory altitude" KPI axis.

The closed benchmark hands HARVEST hv_waiver_max_agl_m = 200.0 (see
research/cpp_benchmark/bench.run_planner), so the 120 m civil limit is NOT
binding in any of the 24 configurations step5 scanned: both planners fly at
the benchmark's 140 m and neither is flagged. That makes 'regulatory
altitude' a capability claim with no measurement behind it.

This script measures it properly: re-plan the same cell with the waiver at
120 m instead of 200 m and record what each planner does.

usage: python step8_altitude_axis.py <SCEN> <N> [waiver_m]
"""
import json
import sys
from pathlib import Path

REPO = Path(r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
BENCH = REPO / "research/cpp_benchmark"
for p in (str(REPO), str(BENCH)):
    if p not in sys.path:
        sys.path.insert(0, p)

import logging                                               # noqa: E402
logging.disable(logging.CRITICAL)

import matrix as M                                           # noqa: E402
from bench import _drones, polys_to_areas, xy, HOME_LL, metrics   # noqa: E402
from mission.swarm_coordinator import SwarmCoordinator       # noqa: E402

SCEN, N = sys.argv[1], int(sys.argv[2])
WAIVERS = [float(x) for x in (sys.argv[3:] or ["200.0", "120.0"])]
OUT = Path(__file__).resolve().parents[1] / "out"
POLYS, _ = M.SCEN[SCEN]


def harvest(waiver):
    drones, cls = _drones(N, 0, M.PITCH)
    c = SwarmCoordinator()
    c.use_harvest = True
    c.hv_class_of = lambda d: cls[int(d["drone_id"])]
    c.hv_home = HOME_LL
    c.hv_wind = (5.0, 270.0)
    c.hv_fence_zones = []
    c.harvest_params = {"hfov_deg": 60.0, "vfov_deg": 45.0, "hv_c2_range_km": 25.0,
                        "hv_waiver_max_agl_m": waiver, "hv_dem_margin_m": 3000.0}
    res = c.plan_coverage_dccpp(areas=polys_to_areas(POLYS), drones=drones,
                                coverage_width_m=M.PITCH, overlap_rate=0.0,
                                coverage_altitude=M.ALT, mounting_angle_deg=0.0)
    return c, res


rep = {"scenario": SCEN, "n": N, "bench_coverage_altitude_m": M.ALT, "runs": []}
for w in WAIVERS:
    try:
        c, res = harvest(w)
        hvc = getattr(c, "hv_coordinator", None)
        v = c.hv_validation
        plan = hvc.last_plan if hvc else None
        tracks = [t for t in ([xy(q.lat, q.lon) for q in p.waypoints]
                              for p in (res.get("assembled_paths") or {}).values()) if len(t) >= 2]
        m = metrics(POLYS, tracks, M.SWATH) if tracks else {}
        agl = []
        if plan:
            for p in plan.plans.values():
                agl += [q.agl_m for q in p.waypoints if q.phase.name == "OPERATE"]
        row = dict(waiver_max_agl_m=w, makespan_s=res.get("total_makespan"),
                   max_km=m.get("max_km"), coverage=m.get("coverage"),
                   n_aircraft_with_route=len(tracks),
                   uncovered=(len(plan.uncovered or ()) if plan else None),
                   operate_agl_min_m=(round(min(agl), 2) if agl else None),
                   operate_agl_max_m=(round(max(agl), 2) if agl else None),
                   validate_passed=(bool(v.passed) if v else None),
                   agl_ok=(bool(v.agl_ok) if v else None),
                   airspace_ok=(bool(v.airspace_ok) if v else None),
                   coverage_ok=(bool(v.coverage_ok) if v else None),
                   feasibility_status=(plan.feasibility.status if plan else None),
                   feasibility_errors=(plan.feasibility.errors[:4] if plan else None),
                   feasibility_warnings=(plan.feasibility.warnings[:4] if plan else None))
    except Exception as e:                                   # noqa: BLE001
        row = dict(waiver_max_agl_m=w, error=f"{type(e).__name__}: {e}")
    rep["runs"].append(row)
    print(json.dumps(row, ensure_ascii=False, default=str))

f = OUT / f"step8_altitude_{SCEN}_n{N}.json"
f.write_text(json.dumps(rep, indent=2, ensure_ascii=False, default=str), encoding="utf-8")
print("\nwrote", f)
