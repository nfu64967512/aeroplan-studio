# -*- coding: utf-8 -*-
"""prep_baseline_control step J -- pin down WHICH budget makes HARVEST decline
S1/n=2 and S5/n=8.

step I showed the baseline's route for those two cells needs only 125.0 / 126.9 km
of the 133 km energy budget, so ENERGY is not what stops HARVEST. The remaining
candidate is the TIME budget in hv_dccpp/allocate.py:699-724:

    denom     = kappa_load * T_use - t_in - t_RTB
    lam_floor = ceil(T_single / denom)
    lam_max   = min(n_UAV, n_strips)
    lam_floor > lam_max  and  not multi_sortie  ->  cut the strip list at the budget

with PlannerConfig.kappa_load = 0.85 (hv_dccpp/mission.py), i.e. a further 15 %
loading margin ON TOP of the already-conservative D_use lower edge.

Prediction: raising kappa_load towards 1.0 should let the same fleet keep more
strips, and at some value coverage should jump to 1.0. If it does, the exclusion
of these two cells is a CONFIGURED MARGIN, not a capability gap. If it does not,
the diagnosis is wrong and the limitation is deeper.

Every run here is pure Python planning. usage: python stepJ_kappa_load_probe.py
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

OUT = Path(__file__).resolve().parents[1] / "out"
OUT.mkdir(parents=True, exist_ok=True)


def run(scen, n, cfg_over):
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
    if cfg_over:
        params["hv_cfg"] = dict(cfg_over)
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
    eff = {k: getattr(getattr(hv, "cfg", None), k, "<absent>") for k in (cfg_over or {})}
    rec = dict(scenario=scen, n=n, cfg_requested=dict(cfg_over or {}), cfg_effective=eff,
               sec=round(sec, 1), coverage=m["coverage"], max_km=m["max_km"],
               total_km=m["total_km"])
    if plan is not None:
        rec["n_uncovered"] = len(plan.uncovered or [])
        rec["uncovered"] = [[a, b] for a, b in (plan.uncovered or [])]
        rec["strips_flown"] = sum(len(p.strips or []) for p in (plan.plans or {}).values())
        rec["strips_total"] = len(plan.strips or {})
        rec["max_cruise_equiv_km"] = round(
            max((float(p.cruise_equiv_km) for p in (plan.plans or {}).values()), default=0.0), 2)
        rec["makespan_planner_s"] = round(float(plan.makespan_s), 1)
    if val is not None:
        rec["validate_passed"] = bool(getattr(val, "passed", False))
    return rec


GRID = [None, {"kappa_load": 0.90}, {"kappa_load": 0.95}, {"kappa_load": 1.0},
        {"kappa_load": 1.2}, {"kappa_load": 1.5}]
ALL = []
for scen, n in (("S1", 2), ("S5", 8)):
    print(f"\n########## HARVEST {scen}/n={n} ##########")
    hdr = (f"{'kappa_load':>11} {'effective':>10} | {'cov':>7} {'strips':>9} "
           f"{'uncov':>6} {'max D_plan':>10} {'valid':>6} {'sec':>6}")
    print(hdr)
    print("-" * len(hdr))
    for over in GRID:
        try:
            r = run(scen, n, over)
        except Exception as e:                                   # noqa: BLE001
            print(f"{str(over):>11} ERROR {type(e).__name__}: {e}")
            ALL.append(dict(scenario=scen, n=n, cfg_requested=over,
                            error=f"{type(e).__name__}: {e}"))
            continue
        ALL.append(r)
        req = "default(0.85)" if over is None else str(over["kappa_load"])
        eff = r["cfg_effective"].get("kappa_load", "0.85(default)")
        print(f"{req:>11} {str(eff):>10} | {r['coverage']:7} "
              f"{str(r.get('strips_flown'))+'/'+str(r.get('strips_total')):>9} "
              f"{r.get('n_uncovered'):6} {r.get('max_cruise_equiv_km'):10} "
              f"{str(r.get('validate_passed')):>6} {r['sec']:6}")

f = OUT / "stepJ_kappa_load_probe.json"
f.write_text(json.dumps(dict(grid=[g for g in GRID], runs=ALL), indent=2,
                        ensure_ascii=False, default=str), encoding="utf-8")
print("")
print("wrote", f)
