# -*- coding: utf-8 -*-
"""prep_baseline_control step O -- VERIFY THE VERIFIER on the HARVEST side.

step N measured HARVEST by re-lifting its assembled_paths into a fresh UAVPlan,
exactly as c5_baseline/step5 does for the baseline. That lift sets
    t_takeoff_s = 0.0  and recomputes every waypoint's t_s from zero,
which THROWS AWAY HARVEST's staggered takeoff schedule (takeoff_schedule() in
hv_dccpp/allocate.py) and its own per-waypoint timestamps. For the DCCPP baseline
that is harmless -- every DCCPP plan really does start at t=0 and carries no
schedule. For HARVEST it measures a plan HARVEST never produced.

So here we take HARVEST's NATIVE UAVPlan objects out of hv.last_plan.plans --
real t_s, real t_takeoff_s, real landing_site -- and push THOSE through the same
two checkers. The gap between step N and step O is the size of the instrument
error, and it is reported, not hidden.

usage: python stepO_harvest_native.py
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
from hv_dccpp.context import PlannerConfig                       # noqa: E402
from hv_dccpp import deconflict as HDEC                          # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "out"
OUT.mkdir(parents=True, exist_ok=True)
CFG = PlannerConfig()
PAPER10 = [("S1", 4), ("S1", 8), ("S1", 16), ("S2", 4), ("S2", 8),
           ("S3", 4), ("S3", 8), ("S4", 4), ("S4", 8), ("S5", 16)]


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
    row = dict(scenario=scen, n=n, sec=round(sec, 1), coverage=m["coverage"],
               max_km=m["max_km"])
    if plan is None or not plan.plans:
        row["error"] = "no native plan"
        return row
    native = dict(plan.plans)
    row["n_plans"] = len(native)
    row["t_takeoff_s"] = {u: round(float(p.t_takeoff_s), 1) for u, p in native.items()}
    row["t_land_s"] = {u: round(float(p.t_land_s), 1) for u, p in native.items()}
    row["landing_sites"] = sorted({p.landing_site for p in native.values()})
    row["max_agl_m"] = round(max(float(w.agl_m) for p in native.values()
                                 for w in p.waypoints), 2)
    row["max_z_m"] = round(max(float(w.z_m) for p in native.values()
                               for w in p.waypoints), 2)
    try:
        conf = HDEC.check_separation(native, CFG, t_sync_s=0.0)
        row["sep"] = len(conf)
        row["sep_sample"] = [f"{x.kind} {x.uav_a}-{x.uav_b} t={x.t_s:.0f} d_h={x.d_h_m:.0f}"
                             for x in conf[:3]]
    except Exception as e:                                       # noqa: BLE001
        row["sep"] = -1
        row["sep_error"] = f"{type(e).__name__}: {e}"
    try:
        sch = HDEC.CorridorSchedule(CFG)
        for u, p in native.items():
            sch.reserve_plan(p, p.landing_site)
        row["pad"] = len(sch.conflicts())
    except Exception as e:                                       # noqa: BLE001
        row["pad"] = -1
        row["pad_error"] = f"{type(e).__name__}: {e}"
    return row


ALL = []
print("=" * 112)
print("O. HARVEST measured on its OWN plan objects (real timestamps, real takeoff schedule)")
print(f"   checker config: d_sep_m={CFG.d_sep_m}  dh_sep_m={CFG.dh_sep_m}")
print("=" * 112)
hdr = (f"{'cell':>8} | {'4D':>5} {'pad':>5} | {'cov':>6} {'max_km':>7} {'maxAGL':>7} "
       f"{'t_takeoff spread (s)':>22} {'pads':>6} {'sec':>6}")
print(hdr)
print("-" * len(hdr))
for scen, n in PAPER10:
    try:
        r = run(scen, n)
    except Exception as e:                                       # noqa: BLE001
        print(f"{scen+'/'+str(n):>8} ERROR {type(e).__name__}: {e}")
        ALL.append(dict(scenario=scen, n=n, error=f"{type(e).__name__}: {e}"))
        continue
    ALL.append(r)
    ts = list((r.get("t_takeoff_s") or {}).values())
    spread = f"{min(ts):.0f}..{max(ts):.0f}" if ts else "-"
    print(f"{scen+'/'+str(n):>8} | {r.get('sep', -1):5} {r.get('pad', -1):5} | "
          f"{r['coverage']:6} {r['max_km']:7} {r.get('max_agl_m'):7} "
          f"{spread:>22} {','.join(r.get('landing_sites') or []):>6} {r['sec']:6}", flush=True)
print("-" * len(hdr))
print(f"{'TOTAL':>8} | {sum(r.get('sep', 0) for r in ALL if r.get('sep', -1) >= 0):5} "
      f"{sum(r.get('pad', 0) for r in ALL if r.get('pad', -1) >= 0):5}")

f = OUT / "stepO_harvest_native.json"
f.write_text(json.dumps(dict(checker_config=dict(d_sep_m=float(CFG.d_sep_m),
                                                 dh_sep_m=float(CFG.dh_sep_m)),
                             rows=ALL), indent=2, ensure_ascii=False, default=str),
             encoding="utf-8")
print("")
print("wrote", f)
