# -*- coding: utf-8 -*-
"""prep_baseline_control step K -- read the allocator's OWN bookkeeping for the
two excluded cells, instead of inferring it.

Wraps (read-only, no source edit):
  hv_dccpp.allocate.assign_config_and_count -> CountResult(by_cell, partial, infeasible)
  hv_dccpp.allocate.allocate                -> AllocationResult(uncoverable, leftover_strips,
                                                                lambda_by_cell, plans)
and prints what each stage decided, so "why is half the area unflown" is answered
by the planner's own reason codes.

usage: python stepK_allocator_trace.py
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
import hv_dccpp.allocate as AL                                   # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "out"
OUT.mkdir(parents=True, exist_ok=True)

TRACE = {"count": [], "alloc": []}
_o_count = AL.assign_config_and_count
_o_alloc = AL.allocate


def count_spy(*a, **k):
    r = _o_count(*a, **k)
    TRACE["count"].append(dict(
        by_cell={kk: [vv[0], vv[1]] for kk, vv in (r.by_cell or {}).items()},
        partial={kk: len(vv) for kk, vv in (r.partial or {}).items()},
        partial_ids={kk: list(vv)[:4] for kk, vv in (r.partial or {}).items()},
        infeasible=dict(r.infeasible or {}),
        J={kk: round(float(vv), 3) for kk, vv in (r.J or {}).items()}))
    return r


def alloc_spy(*a, **k):
    r = _o_alloc(*a, **k)
    try:
        TRACE["alloc"].append(dict(
            uncoverable=[[x, y] for x, y in (r.uncoverable or [])],
            n_leftover=len(r.leftover_strips or []),
            leftover_sample=list(r.leftover_strips or [])[:6],
            lambda_by_cell={kk: [vv[0], vv[1]] for kk, vv in (r.lambda_by_cell or {}).items()},
            plans={u: len(p) for u, p in (r.plans or {}).items()},
            residual_k=dict(r.residual_k or {})))
    except Exception as e:                                       # noqa: BLE001
        TRACE["alloc"].append({"spy_error": f"{type(e).__name__}: {e}"})
    return r


AL.assign_config_and_count = count_spy
AL.allocate = alloc_spy
# planner.py may have imported the names directly
try:
    import hv_dccpp.planner as PL
    for nm, fn in (("assign_config_and_count", count_spy), ("allocate", alloc_spy)):
        if hasattr(PL, nm):
            setattr(PL, nm, fn)
            print(f"  (also patched hv_dccpp.planner.{nm})")
except Exception as e:                                           # noqa: BLE001
    print("  planner patch skipped:", e)


def run(scen, n):
    TRACE["count"].clear()
    TRACE["alloc"].clear()
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
    out = dict(scenario=scen, n=n, sec=round(sec, 1), coverage=m["coverage"],
               count_calls=json.loads(json.dumps(TRACE["count"], default=str)),
               alloc_calls=json.loads(json.dumps(TRACE["alloc"], default=str)))
    if plan is not None:
        flown = {uid: [s for s in (p.strips or [])] for uid, p in (plan.plans or {}).items()}
        out["strips_flown_per_uav"] = {u: len(v) for u, v in flown.items()}
        out["strips_total"] = len(plan.strips or {})
        out["plan_uncovered"] = [[a, b] for a, b in (plan.uncovered or [])]
        flown_ids = {s if isinstance(s, str) else getattr(s, "strip_id", str(s))
                     for v in flown.values() for s in v}
        by_cell = {}
        for sid, s in (plan.strips or {}).items():
            cid = str(getattr(s, "cell_id", "?"))
            d = by_cell.setdefault(cid, dict(total=0, flown=0))
            d["total"] += 1
            if sid in flown_ids:
                d["flown"] += 1
        out["strips_by_cell"] = by_cell
    return out


ALL = {}
for scen, n in (("S1", 2), ("S1", 4), ("S5", 8)):
    print(f"\n================ HARVEST {scen}/n={n} ================")
    r = run(scen, n)
    ALL[f"{scen}_n{n}"] = r
    print(f"coverage={r['coverage']}  strips flown per uav={r.get('strips_flown_per_uav')}  "
          f"total strips={r.get('strips_total')}")
    print(f"strips per cell (flown/total): "
          + ", ".join(f"{k}:{v['flown']}/{v['total']}" for k, v in sorted((r.get('strips_by_cell') or {}).items())))
    print(f"MissionPlan.uncovered = {r.get('plan_uncovered')}")
    for i, cc in enumerate(r["count_calls"]):
        print(f"  ASSIGN_CONFIG_AND_COUNT call {i}: by_cell(lambda)={cc['by_cell']}")
        print(f"      partial(strips cut for energy)={cc['partial']}  infeasible={cc['infeasible']}")
    for i, ac in enumerate(r["alloc_calls"]):
        if "spy_error" in ac:
            print(f"  ALLOCATE call {i}: spy_error {ac['spy_error']}")
            continue
        print(f"  ALLOCATE call {i}: lambda_by_cell={ac['lambda_by_cell']}  plans(tasks per uav)={ac['plans']}")
        print(f"      uncoverable={ac['uncoverable']}")
        print(f"      leftover_strips={ac['n_leftover']} e.g. {ac['leftover_sample']}")

f = OUT / "stepK_allocator_trace.json"
f.write_text(json.dumps(ALL, indent=2, ensure_ascii=False, default=str), encoding="utf-8")
print("")
print("wrote", f)
