# -*- coding: utf-8 -*-
"""prep_baseline_control step N -- the honesty pass on every KPI row where
HARVEST "wins".

Two questions, both answered by measurement:

N1. Was the BASELINE ever given the chance on the conflict axes?
    core/dccpp/collision_avoidance.py:150 CollisionAvoidance exists in this repo
    and is NEVER called by plan_coverage_dccpp (grep over core/ mission/ hv_dccpp/
    gui/ cli/ research/ scripts/ finds the class only in its own module). So the
    published conflict counts describe THIS INTEGRATION, not the method's ceiling.
    Here we run detect()+resolve() on the baseline's own paths and re-measure with
    the identical HARVEST checkers, so the paper can quote the baseline's BEST
    number rather than its default one.

N2. Is HARVEST actually clean on those axes, or was it simply never measured?
    Run HARVEST on the paper's 10 cells and push its routes through the SAME two
    checkers (hv_dccpp.deconflict.check_separation and CorridorSchedule).

usage: python stepN_fair_fight.py
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
from core.dccpp.collision_avoidance import CollisionAvoidance    # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "out"
OUT.mkdir(parents=True, exist_ok=True)

SPEC = {"A": PLATFORM_PIONEER, "B": PLATFORM_HERO}
PHASE_OF = {"TAKEOFF": Phase.TAKEOFF, "ENTRY": Phase.ENTRY, "OPERATION": Phase.OPERATE,
            "TRANSFER": Phase.TRANSFER, "LANDING": Phase.LAND}
_HOVER_EPS, V_VERT = 1.0, 2.5
CFG = PlannerConfig()
PAPER10 = [("S1", 4), ("S1", 8), ("S1", 16), ("S2", 4), ("S2", 8),
           ("S3", 4), ("S3", 8), ("S4", 4), ("S4", 8), ("S5", 16)]


def _hdg(w):
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
            track_deg=_hdg(w), heading_deg=_hdg(w),
            energy_remaining_wh=0.0, nearest_alternate="H", alt_reachable=True,
            lost_link_action="RTB"))
    return UAVPlan(uav_id=uav_id, cls=cls_letter, waypoints=out, strips=[],
                   t_takeoff_s=0.0, t_arr_ptc_s=out[-1].t_s if out else 0.0,
                   t_land_s=out[-1].t_s if out else 0.0, landing_site="H",
                   energy_used_wh=0.0, energy_margin_wh=0.0, distance_m=d_tot,
                   cruise_equiv_km=d_tot / 1000.0, orbits_inserted=0, warnings=[],
                   takeoff_site="H")


def checkers(plans):
    r = {}
    try:
        r["sep"] = len(HDEC.check_separation(dict(plans), CFG, t_sync_s=0.0))
    except Exception as e:                                       # noqa: BLE001
        r["sep"] = -1
        r["sep_error"] = f"{type(e).__name__}: {e}"
    try:
        sch = HDEC.CorridorSchedule(CFG)
        for p in plans.values():
            sch.reserve_plan(p, "H")
        r["pad"] = len(sch.conflicts())
    except Exception as e:                                       # noqa: BLE001
        r["pad"] = -1
        r["pad_error"] = f"{type(e).__name__}: {e}"
    return r


def plan(use_hv, scen, n):
    polys, _ = M.SCEN[scen]
    drones, cls = _drones(n, 0, M.PITCH)
    c = SwarmCoordinator()
    if use_hv:
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
    ap = res.get("assembled_paths") or {}
    tracks = [t for t in ([xy(w.lat, w.lon) for w in p.waypoints] for p in ap.values())
              if len(t) >= 2]
    m = metrics(polys, tracks, M.SWATH)
    return ap, cls, m, sec


ALL = []
print("=" * 108)
print("N. FAIR FIGHT on the conflict axes -- same two checkers for both planners,")
print("   and the baseline run BOTH without and WITH its own CollisionAvoidance.")
print("=" * 108)
hdr = (f"{'cell':>8} | {'DCCPP 4D':>8} {'DCCPP pad':>9} | {'+CA 4D':>7} {'+CA pad':>8} "
       f"{'CA fired':>9} | {'HARVEST 4D':>10} {'HARVEST pad':>11} {'H cov':>7} "
       f"{'H maxAGL':>8}")
print(hdr)
print("-" * len(hdr))
for scen, n in PAPER10:
    row = dict(scenario=scen, n=n)
    # ---- baseline, as published -------------------------------------------
    ap, cls, m, sec = plan(False, scen, n)
    plans = {f"D{u}": lift(b, f"D{u}", cls[int(u)],
                           float(SPEC[cls[int(u)]].cruise_airspeed_mps))
             for u, b in ap.items()}
    base = checkers(plans)
    row.update(dccpp_sep=base["sep"], dccpp_pad=base["pad"], dccpp_coverage=m["coverage"],
               dccpp_sec=round(sec, 1))
    # ---- baseline + its own CollisionAvoidance ----------------------------
    try:
        ca = CollisionAvoidance(min_separation_m=float(CFG.d_sep_m),
                                alt_offset_m=float(CFG.dh_sep_m),
                                time_step_s=1.0,
                                cruise_speed_mps=float(PLATFORM_PIONEER.cruise_airspeed_mps))
        conf = ca.detect(dict(ap))
        acts = ca.resolve(dict(ap), conf)
        plans2 = {f"D{u}": lift(b, f"D{u}", cls[int(u)],
                                float(SPEC[cls[int(u)]].cruise_airspeed_mps))
                  for u, b in ap.items()}
        ca_res = checkers(plans2)
        row.update(ca_detected=len(conf), ca_actions=len(acts),
                   dccpp_ca_sep=ca_res["sep"], dccpp_ca_pad=ca_res["pad"])
    except Exception as e:                                       # noqa: BLE001
        row.update(ca_error=f"{type(e).__name__}: {e}",
                   ca_detected=-1, ca_actions=-1, dccpp_ca_sep=-1, dccpp_ca_pad=-1)
    # ---- HARVEST -----------------------------------------------------------
    try:
        aph, clsh, mh, sech = plan(True, scen, n)
        plansh = {f"H{u}": lift(b, f"H{u}", clsh[int(u)],
                                float(SPEC[clsh[int(u)]].cruise_airspeed_mps))
                  for u, b in aph.items()}
        hv = checkers(plansh)
        agl = max((float(w.alt) for b in aph.values() for w in b.waypoints), default=0.0)
        row.update(harvest_sep=hv["sep"], harvest_pad=hv["pad"],
                   harvest_coverage=mh["coverage"], harvest_max_agl_m=round(agl, 2),
                   harvest_sec=round(sech, 1))
    except Exception as e:                                       # noqa: BLE001
        row.update(harvest_error=f"{type(e).__name__}: {e}",
                   harvest_sep=-1, harvest_pad=-1, harvest_coverage=None,
                   harvest_max_agl_m=None)
    ALL.append(row)
    print(f"{scen+'/'+str(n):>8} | {row['dccpp_sep']:8} {row['dccpp_pad']:9} | "
          f"{row['dccpp_ca_sep']:7} {row['dccpp_ca_pad']:8} "
          f"{str(row['ca_detected'])+'/'+str(row['ca_actions']):>9} | "
          f"{row['harvest_sep']:10} {row['harvest_pad']:11} "
          f"{str(row.get('harvest_coverage')):>7} "
          f"{str(row.get('harvest_max_agl_m')):>8}", flush=True)

print("-" * len(hdr))
tot = lambda k: sum(r[k] for r in ALL if r.get(k, -1) >= 0)          # noqa: E731
print(f"{'TOTAL':>8} | {tot('dccpp_sep'):8} {tot('dccpp_pad'):9} | "
      f"{tot('dccpp_ca_sep'):7} {tot('dccpp_ca_pad'):8} "
      f"{str(tot('ca_detected'))+'/'+str(tot('ca_actions')):>9} | "
      f"{tot('harvest_sep'):10} {tot('harvest_pad'):11}")

f = OUT / "stepN_fair_fight.json"
f.write_text(json.dumps(dict(
    paper10=[list(x) for x in PAPER10],
    checker_config=dict(d_sep_m=float(CFG.d_sep_m), dh_sep_m=float(CFG.dh_sep_m)),
    rows=ALL), indent=2, ensure_ascii=False, default=str), encoding="utf-8")
print("")
print("wrote", f)
