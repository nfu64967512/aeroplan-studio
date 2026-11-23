# -*- coding: utf-8 -*-
"""prep_baseline_control step F -- why does HARVEST decline S1/n=2 and S5/n=8?

Those are the two benchmark cells excluded from the paper's 0.9%-22.9% claim
(HARVEST coverage 0.5277 / 0.617, self_uncovered 1 / 4, self_validate FAIL).
A reviewer will ask why HARVEST "fails" there, so we pull the planner's OWN
reason codes instead of guessing:

  * MissionPlan.uncovered   -> [(strip_id|cell_id, reason)]
  * FeasibilityReport       -> the planner's own go/no-go
  * validate()              -> per-check pass/fail
  * per-cell arithmetic     -> required cruise-equivalent km vs usable range,
                               so the reason code can be checked against physics.

usage: python stepF_excluded_cells.py
"""
import json
import math
import sys
import time
from pathlib import Path

REPO = Path(r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
BENCH = REPO / "research/cpp_benchmark"
for p in (str(REPO), str(BENCH)):
    if p not in sys.path:
        sys.path.insert(0, p)

import logging                                                   # noqa: E402
logging.disable(logging.CRITICAL)

import matrix as M                                               # noqa: E402
from bench import _drones, polys_to_areas, xy, HOME_LL, metrics  # noqa: E402
from mission.swarm_coordinator import SwarmCoordinator           # noqa: E402
from hv_dccpp.platform import PLATFORM_PIONEER, PLATFORM_HERO    # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "out"
OUT.mkdir(parents=True, exist_ok=True)
SPEC = {"A": PLATFORM_PIONEER, "B": PLATFORM_HERO}


def jsonable(x, depth=0):
    if depth > 6:
        return str(x)
    if x is None or isinstance(x, (bool, int, float, str)):
        return x
    if isinstance(x, (list, tuple, set)):
        return [jsonable(v, depth + 1) for v in x]
    if isinstance(x, dict):
        return {str(k): jsonable(v, depth + 1) for k, v in x.items()}
    for attr in ("to_dict",):
        if hasattr(x, attr):
            try:
                return jsonable(getattr(x, attr)(), depth + 1)
            except Exception:                                    # noqa: BLE001
                pass
    if hasattr(x, "__dict__") or hasattr(x, "__slots__"):
        d = {}
        names = list(getattr(x, "__slots__", ())) or list(vars(x).keys())
        for k in names:
            if k.startswith("_"):
                continue
            try:
                d[k] = jsonable(getattr(x, k), depth + 1)
            except Exception:                                    # noqa: BLE001
                d[k] = "<unreadable>"
        return d or str(x)
    return str(x)


def run(scen, n):
    polys, _ = M.SCEN[scen]
    drones, cls = _drones(n, 0, M.PITCH)
    c = SwarmCoordinator()
    c.use_harvest = True
    c.hv_class_of = lambda d: cls[int(d["drone_id"])]
    c.hv_home = HOME_LL
    c.hv_wind = (5.0, 270.0)
    c.hv_fence_zones = []
    c.harvest_params = {"hfov_deg": 60.0, "vfov_deg": 45.0, "hv_c2_range_km": 25.0,
                        "hv_waiver_max_agl_m": 200.0, "hv_dem_margin_m": 3000.0}
    t0 = time.perf_counter()
    res = c.plan_coverage_dccpp(areas=polys_to_areas(polys), drones=drones,
                                coverage_width_m=M.PITCH, overlap_rate=0.0,
                                coverage_altitude=M.ALT, mounting_angle_deg=0.0)
    sec = time.perf_counter() - t0
    tracks = [t for t in ([xy(w.lat, w.lon) for w in p.waypoints]
                          for p in (res.get("assembled_paths") or {}).values())
              if len(t) >= 2]
    m = metrics(polys, tracks, M.SWATH)

    hv = getattr(c, "hv_coordinator", None)
    plan = getattr(hv, "last_plan", None) if hv else None
    val = getattr(c, "hv_validation", None)

    rec = dict(scenario=scen, n=n, sec=round(sec, 1), ruler_coverage=m["coverage"],
               ruler_max_km=m["max_km"], ruler_total_km=m["total_km"],
               makespan_s=res.get("total_makespan"))

    if plan is not None:
        rec["uncovered"] = [[a, b] for a, b in (plan.uncovered or [])]
        rec["n_cells"] = len(plan.cells or [])
        rec["n_strips"] = len(plan.strips or {})
        rec["makespan_planner_s"] = plan.makespan_s
        rec["feasibility"] = jsonable(plan.feasibility)
        per = {}
        for uid, p in (plan.plans or {}).items():
            per[uid] = dict(cls=p.cls, n_strips=len(p.strips or []),
                            cruise_equiv_km=round(float(p.cruise_equiv_km), 3),
                            distance_m=round(float(p.distance_m), 1),
                            energy_used_wh=round(float(p.energy_used_wh), 1),
                            energy_margin_wh=round(float(p.energy_margin_wh), 1),
                            t_land_s=round(float(p.t_land_s), 1),
                            orbits=int(p.orbits_inserted),
                            warnings=list(p.warnings or []),
                            usable_km_lo=SPEC[p.cls].usable_distance_km[0],
                            usable_km_hi=SPEC[p.cls].usable_distance_km[1])
        rec["per_uav"] = per
        # -- the arithmetic behind the reason code -------------------------
        cell_info = []
        for cell in (plan.cells or []):
            strips_here = [s for s in (plan.strips or {}).values()
                           if str(getattr(s, "cell_id", "")) == str(getattr(cell, "cell_id", ""))]
            L = sum(float(getattr(s, "length_m", 0.0)) for s in strips_here)
            cell_info.append(dict(cell_id=str(getattr(cell, "cell_id", "?")),
                                  n_strips=len(strips_here),
                                  strip_length_total_km=round(L / 1000.0, 2),
                                  area_km2=round(float(getattr(cell, "area_m2", 0.0)) / 1e6, 3)
                                  if getattr(cell, "area_m2", None) else None))
        rec["cells"] = cell_info
    else:
        rec["uncovered"] = None
        rec["note"] = "hv_coordinator/last_plan unavailable"

    if val is not None:
        rec["validate_passed"] = bool(getattr(val, "passed", False))
        rec["validate"] = jsonable(val)
    return rec


ALL = {}
for scen, n in (("S1", 2), ("S5", 8), ("S1", 4)):     # S1/4 = a PASSING control
    print(f"--- HARVEST {scen} n={n} ---", flush=True)
    try:
        r = run(scen, n)
    except Exception as e:                                       # noqa: BLE001
        import traceback
        r = dict(scenario=scen, n=n, error=f"{type(e).__name__}: {e}",
                 tb=traceback.format_exc()[-2000:])
        print("   ERROR", r["error"])
    ALL[f"{scen}_n{n}"] = r
    if "error" not in r:
        print(f"   coverage={r['ruler_coverage']} max_km={r['ruler_max_km']} "
              f"uncovered={r.get('uncovered')} validate_passed={r.get('validate_passed')}")
        for uid, p in (r.get("per_uav") or {}).items():
            print(f"   {uid} cls={p['cls']} strips={p['n_strips']} "
                  f"cruise_equiv={p['cruise_equiv_km']} km / usable {p['usable_km_lo']}-{p['usable_km_hi']} km "
                  f"margin={p['energy_margin_wh']} Wh warn={p['warnings']}")

f = OUT / "stepF_excluded_cells.json"
f.write_text(json.dumps(ALL, indent=2, ensure_ascii=False, default=str), encoding="utf-8")
print("")
print("wrote", f)
