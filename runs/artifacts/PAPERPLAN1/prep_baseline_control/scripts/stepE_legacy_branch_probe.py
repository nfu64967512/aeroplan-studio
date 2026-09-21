# -*- coding: utf-8 -*-
"""prep_baseline_control step E -- "was the baseline's coordination code ever
given the chance?"

Step C proved coordination_mode is inert on the DCCPP *v2* pipeline. That only
says something about THIS INTEGRATION. To say something about the METHOD we have
to force the branch where CoordinatedPlanner is actually wired in
(swarm_coordinator.py:2463/2473, the legacy DCCPPSolver), and then ask whether
running it reduces pad / 4-D conflicts at all.

The branch is selected at swarm_coordinator.py:1605-1613:
    try:    from core.dccpp.task_allocator import UAVTaskAllocator ...
            _use_new_dccpp = True
    except ImportError:
            _use_new_dccpp = False
so we force it by poisoning sys.modules for the duration of the run. NO SOURCE
EDIT: the repo is untouched, the poison is reverted in a finally block.

CAVEAT recorded up front: the legacy branch is a DIFFERENT planner (GreedyAllocator
+ IDPSolver + AltitudePlanner + DCCPPPathAssembler), so its geometry is NOT the
paper's baseline. This probe answers only one question: "when CoordinatedPlanner
actually runs, does it touch the pad / 4-D axes?"

usage: python stepE_legacy_branch_probe.py
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
from bench import _drones, polys_to_areas, xy, metrics           # noqa: E402
from mission.swarm_coordinator import SwarmCoordinator           # noqa: E402
from hv_dccpp.mission import Waypoint, UAVPlan, Phase, Mode      # noqa: E402
from hv_dccpp.context import PlannerConfig                       # noqa: E402
from hv_dccpp.platform import PLATFORM_PIONEER, PLATFORM_HERO    # noqa: E402
from hv_dccpp import deconflict as HDEC                          # noqa: E402
import core.global_planner.mdtsp_solver as MD                    # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "out"
OUT.mkdir(parents=True, exist_ok=True)

COUNT = {"init": 0, "synchronize": 0, "insert_holding": 0}
_o_i, _o_s, _o_h = (MD.CoordinatedPlanner.__init__, MD.CoordinatedPlanner.synchronize,
                    MD.CoordinatedPlanner._insert_holding_pattern)
MD.CoordinatedPlanner.__init__ = lambda s, *a, **k: (COUNT.__setitem__("init", COUNT["init"] + 1), _o_i(s, *a, **k))[1]
MD.CoordinatedPlanner.synchronize = lambda s, *a, **k: (COUNT.__setitem__("synchronize", COUNT["synchronize"] + 1), _o_s(s, *a, **k))[1]
MD.CoordinatedPlanner._insert_holding_pattern = lambda s, *a, **k: (COUNT.__setitem__("insert_holding", COUNT["insert_holding"] + 1), _o_h(s, *a, **k))[1]

SPEC = {"A": PLATFORM_PIONEER, "B": PLATFORM_HERO}
D_USE_KM = {"A": PLATFORM_PIONEER.usable_distance_km[0], "B": PLATFORM_HERO.usable_distance_km[0]}
PHASE_OF = {"TAKEOFF": Phase.TAKEOFF, "ENTRY": Phase.ENTRY, "OPERATION": Phase.OPERATE,
            "TRANSFER": Phase.TRANSFER, "LANDING": Phase.LAND}
_HOVER_EPS, V_VERT = 1.0, 2.5
CFG = PlannerConfig()
POISON = ["core.dccpp.task_allocator", "core.dccpp.idp_solver", "core.dccpp.altitude_planner"]


def _hdg(w):
    """legacy AssembledWaypoint exposes heading_deg; v2 BuiltWaypoint exposes
    heading_compass_deg. Take whichever exists."""
    for a in ("heading_compass_deg", "heading_deg", "heading"):
        v = getattr(w, a, None)
        if v is not None:
            return float(v)
    return 0.0


def lift(built, uav_id, cls_letter, v_cruise):
    wps_in = list(built.waypoints)
    out, t, d_tot = [], 0.0, 0.0
    for i, w in enumerate(wps_in):
        if i:
            a = wps_in[i - 1]
            dn = (w.lat - a.lat) * 111320.0
            de = (w.lon - a.lon) * 111320.0 * math.cos(math.radians((a.lat + w.lat) / 2.0))
            horiz = math.hypot(dn, de)
            vert = abs(float(w.alt) - float(a.alt))
            t += (vert / V_VERT) if horiz <= _HOVER_EPS else (math.hypot(horiz, vert) / v_cruise)
            d_tot += math.hypot(horiz, vert)
        x_, y_ = xy(w.lat, w.lon)
        ph = PHASE_OF.get(getattr(w.segment_type, "name", "TRANSFER"), Phase.TRANSFER)
        vert_leg = ph in (Phase.TAKEOFF, Phase.LAND)
        out.append(Waypoint(
            x_m=x_, y_m=y_, z_m=float(w.alt), lat_deg=w.lat, lon_deg=w.lon,
            alt_msl_m=float(w.alt), agl_m=float(w.alt), t_s=t,
            mode=Mode.HOVER if vert_leg else Mode.CRUISE, phase=ph,
            sensor_on=(ph is Phase.OPERATE), strip_id=None, cell_id=None, dir=None,
            airspeed_mps=0.0 if vert_leg else v_cruise,
            ground_speed_mps=0.0 if vert_leg else v_cruise,
            track_deg=float(_hdg(w)), heading_deg=float(_hdg(w)),
            energy_remaining_wh=0.0, nearest_alternate="H", alt_reachable=True,
            lost_link_action="RTB"))
    return UAVPlan(uav_id=uav_id, cls=cls_letter, waypoints=out, strips=[],
                   t_takeoff_s=0.0, t_arr_ptc_s=out[-1].t_s if out else 0.0,
                   t_land_s=out[-1].t_s if out else 0.0, landing_site="H",
                   energy_used_wh=0.0, energy_margin_wh=0.0, distance_m=d_tot,
                   cruise_equiv_km=d_tot / 1000.0, orbits_inserted=0, warnings=[],
                   takeoff_site="H"), d_tot


def run_cell(scen, n, mode, legacy):
    polys, _ = M.SCEN[scen]
    drones, cls = _drones(n, 0, M.PITCH)
    saved = {}
    if legacy:
        for mname in POISON:
            saved[mname] = sys.modules.get(mname, "__absent__")
            sys.modules[mname] = None          # -> ImportError at the try/except
    before = dict(COUNT)
    try:
        c = SwarmCoordinator()
        t0 = time.perf_counter()
        res = c.plan_coverage_dccpp(areas=polys_to_areas(polys), drones=drones,
                                    coverage_width_m=M.PITCH, overlap_rate=0.0,
                                    coverage_altitude=M.ALT, mounting_angle_deg=0.0,
                                    coordination_mode=mode)
        sec = time.perf_counter() - t0
    finally:
        for mname, v in saved.items():
            if v == "__absent__":
                sys.modules.pop(mname, None)
            else:
                sys.modules[mname] = v
    ap = res.get("assembled_paths") or {}
    tracks = [t for t in ([xy(w.lat, w.lon) for w in p.waypoints] for p in ap.values())
              if len(t) >= 2]
    m = metrics(polys, tracks, M.SWATH)
    plans = {}
    for uid, built in ap.items():
        letter = cls[int(uid)]
        p, _d = lift(built, f"D{uid}", letter, float(SPEC[letter].cruise_airspeed_mps))
        plans[p.uav_id] = p
    out = dict(scenario=scen, n=n, mode=mode, branch="legacy" if legacy else "v2",
               sec=round(sec, 1), coverage=m["coverage"], max_km=m["max_km"],
               total_km=m["total_km"], makespan_s=res.get("total_makespan"),
               n_wp_total=sum(len(p.waypoints) for p in plans.values()),
               n_plans=len(plans))
    try:
        out["sep_conflicts"] = len(HDEC.check_separation(dict(plans), CFG, t_sync_s=0.0))
    except Exception as e:                                       # noqa: BLE001
        out["sep_conflicts"] = -1
        out["sep_error"] = f"{type(e).__name__}: {e}"
    try:
        sch = HDEC.CorridorSchedule(CFG)
        for p in plans.values():
            sch.reserve_plan(p, "H")
        out["pad_conflicts"] = len(sch.conflicts())
    except Exception as e:                                       # noqa: BLE001
        out["pad_conflicts"] = -1
        out["pad_error"] = f"{type(e).__name__}: {e}"
    out["coord_calls"] = {k: COUNT[k] - before[k] for k in COUNT}
    return out


PROBE = [("S1", 4), ("S1", 8), ("S1", 16), ("S3", 8), ("S4", 8)]
rows = []
hdr = (f"{'cell':>8} {'branch':>7} {'mode':>14} | {'cov':>6} {'maxkm':>7} {'wp':>5} "
       f"{'plans':>5} {'4D':>4} {'pad':>4} | {'CPini':>5} {'CPsyn':>5} {'CPhold':>6} {'sec':>6}")
print(hdr)
print("-" * len(hdr))
for scen, n in PROBE:
    for legacy in (False, True):
        for mode in ("uncoordinated", "coordinated"):
            try:
                r = run_cell(scen, n, mode, legacy)
            except Exception as e:                               # noqa: BLE001
                print(f"{scen+'/'+str(n):>8} {'legacy' if legacy else 'v2':>7} {mode:>14} "
                      f"ERROR {type(e).__name__}: {e}")
                rows.append(dict(scenario=scen, n=n, mode=mode,
                                 branch="legacy" if legacy else "v2",
                                 error=f"{type(e).__name__}: {e}"))
                continue
            rows.append(r)
            cc = r["coord_calls"]
            print(f"{scen+'/'+str(n):>8} {r['branch']:>7} {mode:>14} | {r['coverage']:6} "
                  f"{r['max_km']:7} {r['n_wp_total']:5} {r['n_plans']:5} "
                  f"{r['sep_conflicts']:4} {r['pad_conflicts']:4} | "
                  f"{cc['init']:5} {cc['synchronize']:5} {cc['insert_holding']:6} {r['sec']:6}")

f = OUT / "stepE_legacy_branch_probe.json"
f.write_text(json.dumps(dict(probe_cells=[list(x) for x in PROBE],
                             coord_call_totals=COUNT, rows=rows),
                        indent=2, ensure_ascii=False, default=str), encoding="utf-8")
print("")
print("coordination-code call totals:", COUNT)
print("wrote", f)
