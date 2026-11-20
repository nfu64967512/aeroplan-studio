# -*- coding: utf-8 -*-
"""prep_baseline_control step C -- the fair-fight control.

Re-runs the 24-configuration DCCPP KPI scan TWICE in one process:
  arm 1: coordination_mode="uncoordinated"  (what the paper's baseline used)
  arm 2: coordination_mode="coordinated"    (the baseline's own coordination feature)

Everything else is byte-identical to c5_baseline/scripts/step5_kpi_scan.py, so the
uncoordinated arm doubles as a reproduction check of that run.

Instrumentation: CoordinatedPlanner.__init__ and .synchronize are wrapped with
counters, so "the flag is inert" is a MEASURED fact, not a source-reading claim.

usage: python stepC_kpi_scan_control.py
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
from hv_dccpp.mission import Waypoint, UAVPlan, Phase, Mode      # noqa: E402
from hv_dccpp.context import PlannerConfig                       # noqa: E402
from hv_dccpp.platform import PLATFORM_PIONEER, PLATFORM_HERO    # noqa: E402
from hv_dccpp import deconflict as HDEC                          # noqa: E402
import core.global_planner.mdtsp_solver as MD                    # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "out"
OUT.mkdir(parents=True, exist_ok=True)

# ---- instrumentation: did the baseline's coordination code ever run? -------
COUNT = {"init": 0, "synchronize": 0, "insert_holding": 0}
_orig_init = MD.CoordinatedPlanner.__init__
_orig_sync = MD.CoordinatedPlanner.synchronize
_orig_hold = MD.CoordinatedPlanner._insert_holding_pattern


def _init(self, *a, **k):
    COUNT["init"] += 1
    return _orig_init(self, *a, **k)


def _sync(self, *a, **k):
    COUNT["synchronize"] += 1
    return _orig_sync(self, *a, **k)


def _hold(self, *a, **k):
    COUNT["insert_holding"] += 1
    return _orig_hold(self, *a, **k)


MD.CoordinatedPlanner.__init__ = _init
MD.CoordinatedPlanner.synchronize = _sync
MD.CoordinatedPlanner._insert_holding_pattern = _hold

SPEC = {"A": PLATFORM_PIONEER, "B": PLATFORM_HERO}
D_USE_KM = {"A": PLATFORM_PIONEER.usable_distance_km[0], "B": PLATFORM_HERO.usable_distance_km[0]}
R_PLAN = {"A": PLATFORM_PIONEER.turn_radius_planning_m, "B": PLATFORM_HERO.turn_radius_planning_m}
CIVIL_AGL_M = 120.0
PHASE_OF = {"TAKEOFF": Phase.TAKEOFF, "ENTRY": Phase.ENTRY, "OPERATION": Phase.OPERATE,
            "TRANSFER": Phase.TRANSFER, "LANDING": Phase.LAND}
_HOVER_EPS, V_VERT = 1.0, 2.5
CFG = PlannerConfig()


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
            track_deg=float(w.heading_compass_deg), heading_deg=float(w.heading_compass_deg),
            energy_remaining_wh=0.0, nearest_alternate="H", alt_reachable=True,
            lost_link_action="RTB"))
    return UAVPlan(uav_id=uav_id, cls=cls_letter, waypoints=out, strips=[],
                   t_takeoff_s=0.0, t_arr_ptc_s=out[-1].t_s if out else 0.0,
                   t_land_s=out[-1].t_s if out else 0.0, landing_site="H",
                   energy_used_wh=0.0, energy_margin_wh=0.0, distance_m=d_tot,
                   cruise_equiv_km=d_tot / 1000.0, orbits_inserted=0, warnings=[],
                   takeoff_site="H"), d_tot


def run_cell(scen, n, mix, mode):
    polys, _ = M.SCEN[scen]
    n_a, n_b = (n - n // 2, n // 2) if mix else (n, 0)
    drones, cls = _drones(n_a, n_b, M.PITCH)
    c = SwarmCoordinator()
    before = dict(COUNT)
    t0 = time.perf_counter()
    res = c.plan_coverage_dccpp(areas=polys_to_areas(polys), drones=drones,
                                coverage_width_m=M.PITCH, overlap_rate=0.0,
                                coverage_altitude=M.ALT, mounting_angle_deg=0.0,
                                coordination_mode=mode)
    sec = time.perf_counter() - t0
    ap = res.get("assembled_paths") or {}
    tracks = [t for t in ([xy(w.lat, w.lon) for w in p.waypoints] for p in ap.values())
              if len(t) >= 2]
    m = metrics(polys, tracks, M.SWATH)

    plans, rows = {}, []
    for uid, built in ap.items():
        letter = cls[int(uid)]
        p, d = lift(built, f"D{uid}", letter, float(SPEC[letter].cruise_airspeed_mps))
        plans[p.uav_id] = p
        agl_max = max((w.z_m for w in p.waypoints), default=0.0)
        rows.append(dict(uav=p.uav_id, cls=letter, d_km=round(d / 1000.0, 2),
                         d_use_km=D_USE_KM[letter],
                         endurance_over=bool(d / 1000.0 > D_USE_KM[letter]),
                         endurance_frac=round(d / 1000.0 / D_USE_KM[letter], 3),
                         n_wp=len(p.waypoints),
                         agl_max_m=round(agl_max, 1),
                         agl_over_120=bool(agl_max > CIVIL_AGL_M + 1e-6),
                         t_land_s=round(p.waypoints[-1].t_s, 1)))

    out = dict(scenario=scen, n=n, mix=mix, mode=mode, sec=round(sec, 1),
               makespan_s=res.get("total_makespan"), max_km=m["max_km"],
               total_km=m["total_km"], coverage=m["coverage"],
               n_wp_total=sum(r["n_wp"] for r in rows), per_uav=rows)
    try:
        conf = HDEC.check_separation(dict(plans), CFG, t_sync_s=0.0)
        out["sep_ran"] = True
        out["sep_conflicts"] = len(conf)
    except Exception as e:                                       # noqa: BLE001
        out["sep_ran"] = False
        out["sep_error"] = f"{type(e).__name__}: {e}"
    try:
        sch = HDEC.CorridorSchedule(CFG)
        for p in plans.values():
            sch.reserve_plan(p, "H")
        out["pad_conflicts"] = len(sch.conflicts())
        out["pad_ran"] = True
    except Exception as e:                                       # noqa: BLE001
        out["pad_ran"] = False
        out["pad_error"] = f"{type(e).__name__}: {e}"
    out["n_endurance_over"] = sum(1 for r in rows if r["endurance_over"])
    out["n_agl_over_120"] = sum(1 for r in rows if r["agl_over_120"])
    out["coord_calls"] = {k: COUNT[k] - before[k] for k in COUNT}
    return out


CELLS = [(s, n) for s, (_p, ns) in M.SCEN.items() for n in ns]
arms = {}
for mode in ("uncoordinated", "coordinated"):
    allr = []
    print("")
    print("########## arm: coordination_mode=" + repr(mode) + " ##########")
    print(f"{'cell':>10} {'fleet':>5} {'cov':>6} {'maxkm':>7} {'wp':>5} {'end>':>4} "
          f"{'sep':>4} {'pad':>4} {'agl>120':>8} {'CPini':>5} {'CPsyn':>5} {'sec':>6}")
    for scen, n in CELLS:
        for mix in (False, True):
            try:
                r = run_cell(scen, n, mix, mode)
            except Exception as e:                               # noqa: BLE001
                print(f"{scen}/{n:<3} {'mix' if mix else 'homo':>5} ERROR {type(e).__name__}: {e}")
                allr.append(dict(scenario=scen, n=n, mix=mix, mode=mode,
                                 error=f"{type(e).__name__}: {e}"))
                continue
            allr.append(r)
            cc = r["coord_calls"]
            print(f"{scen+'/'+str(n):>10} {'mix' if mix else 'homo':>5} {r['coverage']:6} "
                  f"{r['max_km']:7} {r['n_wp_total']:5} {r['n_endurance_over']:4} "
                  f"{r.get('sep_conflicts', -1):4} {r.get('pad_conflicts', -1):4} "
                  f"{r['n_agl_over_120']:8} {cc['init']:5} {cc['synchronize']:5} {r['sec']:6}")
    arms[mode] = allr

f = OUT / "stepC_kpi_scan_control.json"
f.write_text(json.dumps(dict(
    d_use_km=D_USE_KM, r_plan_m=R_PLAN, civil_agl_m=CIVIL_AGL_M,
    bench_alt_m=M.ALT, bench_pitch_m=M.PITCH,
    coord_call_totals=COUNT, arms=arms), indent=2, ensure_ascii=False, default=str),
    encoding="utf-8")
print("")
print("coordination-code call totals over BOTH arms:", COUNT)
print("wrote", f)
