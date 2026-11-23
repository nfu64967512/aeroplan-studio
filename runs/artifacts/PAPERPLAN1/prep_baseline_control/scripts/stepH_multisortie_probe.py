# -*- coding: utf-8 -*-
"""prep_baseline_control step H -- is the HARVEST "failure" on S1/n=2 and S5/n=8
a bug, or the single-sortie budget doing its job?

Code trace (hv_dccpp/allocate.py:706-724, hv_dccpp/mission.py:1045):
    denom     = kappa_load * T_use - t_in - t_RTB          # usable time in ONE sortie
    lam_floor = ceil(T_single / denom)                     # aircraft needed for this cell
    if lam_floor > lam_max (= min(n_UAV, n_strips)):
        if not cfg.multi_sortie:                           # PlannerConfig.multi_sortie = False (default)
            ... cut the strip list at the budget and push the remainder into res.partial
        -> the remainder surfaces as MissionPlan.uncovered[(cell, "no_feasible_insertion")]

So the prediction is: flip cfg.multi_sortie to True (reachable from the adapter via
params["hv_cfg"], hv_dccpp/integration/aeroplan_adapter.py:774-777) and the same
fleet should cover the same area, using a second sortie.

usage: python stepH_multisortie_probe.py
"""
import json
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
from hv_dccpp.mission import PlannerConfig                       # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "out"
OUT.mkdir(parents=True, exist_ok=True)

print("PlannerConfig defaults that matter here:")
_c = PlannerConfig()
for k in ("multi_sortie", "kappa_load", "t_swap_s"):
    print(f"  {k} = {getattr(_c, k, '<absent>')!r}")
print("")


def run(scen, n, multi):
    polys, _ = M.SCEN[scen]
    drones, cls = _drones(n, 0, M.PITCH)
    c = SwarmCoordinator()
    c.use_harvest = True
    c.hv_class_of = lambda d: cls[int(d["drone_id"])]
    c.hv_home = HOME_LL
    c.hv_wind = (5.0, 270.0)
    c.hv_fence_zones = []
    params = {"hfov_deg": 60.0, "vfov_deg": 45.0, "hv_c2_range_km": 25.0,
              "hv_waiver_max_agl_m": 200.0, "hv_dem_margin_m": 3000.0}
    if multi:
        params["hv_cfg"] = {"multi_sortie": True}
    c.harvest_params = params
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
    rec = dict(scenario=scen, n=n, multi_sortie=bool(multi), sec=round(sec, 1),
               coverage=m["coverage"], max_km=m["max_km"], total_km=m["total_km"],
               n_tracks=len(tracks))
    if plan is not None:
        rec["uncovered"] = [[a, b] for a, b in (plan.uncovered or [])]
        rec["n_uncovered"] = len(plan.uncovered or [])
        rec["plan_ids"] = sorted((plan.plans or {}).keys())
        rec["per_uav_cruise_equiv_km"] = {
            uid: round(float(p.cruise_equiv_km), 2) for uid, p in (plan.plans or {}).items()}
        rec["per_uav_strips"] = {uid: len(p.strips or []) for uid, p in (plan.plans or {}).items()}
        rec["makespan_planner_s"] = round(float(plan.makespan_s), 1)
    if val is not None:
        rec["validate_passed"] = bool(getattr(val, "passed", False))
    return rec


ALL = []
for scen, n in (("S1", 2), ("S5", 8)):
    for multi in (False, True):
        print(f"--- HARVEST {scen}/n={n} multi_sortie={multi} ---", flush=True)
        try:
            r = run(scen, n, multi)
        except Exception as e:                                   # noqa: BLE001
            import traceback
            r = dict(scenario=scen, n=n, multi_sortie=bool(multi),
                     error=f"{type(e).__name__}: {e}", tb=traceback.format_exc()[-1500:])
            print("    ERROR", r["error"])
            ALL.append(r)
            continue
        ALL.append(r)
        print(f"    coverage={r['coverage']} max_km={r['max_km']} "
              f"uncovered={r.get('n_uncovered')} validate={r.get('validate_passed')} "
              f"plans={r.get('plan_ids')}")
        print(f"    cruise_equiv_km={r.get('per_uav_cruise_equiv_km')}")
        print(f"    strips={r.get('per_uav_strips')}  makespan={r.get('makespan_planner_s')} s"
              f"  ({r['sec']} s to plan)")

f = OUT / "stepH_multisortie_probe.json"
f.write_text(json.dumps(dict(
    planner_config_defaults={k: getattr(_c, k, None) for k in ("multi_sortie", "kappa_load", "t_swap_s")},
    runs=ALL), indent=2, ensure_ascii=False, default=str), encoding="utf-8")
print("")
print("wrote", f)
