# -*- coding: utf-8 -*-
"""C5 step 5 -- KPI scan over every benchmark cell, DCCPP side only.

The three axes below need NO MissionContext, so they scale to all 12 cells x
{homo, mix} without paying for a HARVEST run per cell:

  endurance   : D_plan vs EnergyModel.usable_distance_cap_km()  (133 km A / 79 km B)
  shared pad  : hv_dccpp.deconflict.CorridorSchedule            (verbatim library code)
  4-D sep     : hv_dccpp.deconflict.check_separation            (verbatim library code)
  turn radius : corner-fit test -- a fly-by arc of radius R needs R*tan(delta/2)
                of tangent on BOTH adjacent legs; shorter legs = unflyable corner.
                (This is the same quantity matrix._turn_charge bills for.)
  altitude    : max planned AGL vs the 120 m civil limit.

usage: python step5_kpi_scan.py
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

OUT = Path(__file__).resolve().parents[1] / "out"
OUT.mkdir(parents=True, exist_ok=True)

SPEC = {"A": PLATFORM_PIONEER, "B": PLATFORM_HERO}
# usable_distance_cap_km: battery_monitor_validated is False for both -> lower edge
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


def corner_fit(track_xy, R):
    """Unflyable corners: a fly-by arc of radius R needs R*tan(delta/2) of
    tangent on each adjacent leg. Returns (n_unflyable, n_corners, worst_deg)."""
    pts = [p for i, p in enumerate(track_xy) if i == 0 or p != track_xy[i - 1]]
    bad, tot, worst = 0, 0, 0.0
    for i in range(1, len(pts) - 1):
        ax, ay = pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]
        bx, by = pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]
        na, nb = math.hypot(ax, ay), math.hypot(bx, by)
        if na < 1e-6 or nb < 1e-6:
            continue
        c = max(-1.0, min(1.0, (ax * bx + ay * by) / (na * nb)))
        d = math.acos(c)                      # deflection
        if math.degrees(d) < 1.0:
            continue
        tot += 1
        worst = max(worst, math.degrees(d))
        need = R * math.tan(min(d, math.radians(179.0)) / 2.0)
        if need > min(na, nb) / 2.0 + 1e-6:
            bad += 1
    return bad, tot, round(worst, 1)


def run_cell(scen, n, mix):
    polys, _ = M.SCEN[scen]
    n_a, n_b = (n - n // 2, n // 2) if mix else (n, 0)
    drones, cls = _drones(n_a, n_b, M.PITCH)
    c = SwarmCoordinator()
    t0 = time.perf_counter()
    res = c.plan_coverage_dccpp(areas=polys_to_areas(polys), drones=drones,
                                coverage_width_m=M.PITCH, overlap_rate=0.0,
                                coverage_altitude=M.ALT, mounting_angle_deg=0.0)
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
        tr = [(w.x_m, w.y_m) for w in p.waypoints]
        bad, tot, worst = corner_fit(tr, R_PLAN[letter])
        agl_max = max((w.z_m for w in p.waypoints), default=0.0)
        rows.append(dict(uav=p.uav_id, cls=letter, d_km=round(d / 1000.0, 2),
                         d_use_km=D_USE_KM[letter],
                         endurance_over=bool(d / 1000.0 > D_USE_KM[letter]),
                         endurance_frac=round(d / 1000.0 / D_USE_KM[letter], 3),
                         R_plan_m=R_PLAN[letter], corners=tot, corners_unflyable=bad,
                         worst_deflection_deg=worst,
                         agl_max_m=round(agl_max, 1),
                         agl_over_120=bool(agl_max > CIVIL_AGL_M + 1e-6),
                         t_land_s=round(p.waypoints[-1].t_s, 1)))

    out = dict(scenario=scen, n=n, mix=mix, sec=round(sec, 1),
               makespan_s=res.get("total_makespan"), max_km=m["max_km"],
               total_km=m["total_km"], coverage=m["coverage"], per_uav=rows)
    try:
        conf = HDEC.check_separation(dict(plans), CFG, t_sync_s=0.0)
        out["sep_ran"] = True
        out["sep_conflicts"] = len(conf)
        out["sep_sample"] = [f"{x.kind} {x.uav_a}-{x.uav_b} t={x.t_s:.0f} d_h={x.d_h_m:.0f}"
                             for x in conf[:4]]
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
    out["n_corners_unflyable"] = sum(r["corners_unflyable"] for r in rows)
    out["n_corners"] = sum(r["corners"] for r in rows)
    out["n_agl_over_120"] = sum(1 for r in rows if r["agl_over_120"])
    out["simultaneous_takeoff"] = len(rows) > 1      # every DCCPP plan starts at t=0
    return out


CELLS = [(s, n) for s, (_p, ns) in M.SCEN.items() for n in ns]
allr = []
print(f"{'cell':>10} {'fleet':>5} {'cov':>6} {'maxkm':>7} {'end>':>4} {'sep':>4} "
      f"{'pad':>4} {'badcorn':>8} {'/corn':>6} {'agl>120':>8} {'sec':>6}")
for scen, n in CELLS:
    for mix in (False, True):
        try:
            r = run_cell(scen, n, mix)
        except Exception as e:                                   # noqa: BLE001
            print(f"{scen}/{n:<3} {'mix' if mix else 'homo':>5} ERROR {type(e).__name__}: {e}")
            allr.append(dict(scenario=scen, n=n, mix=mix, error=f"{type(e).__name__}: {e}"))
            continue
        allr.append(r)
        print(f"{scen+'/'+str(n):>10} {'mix' if mix else 'homo':>5} {r['coverage']:6} "
              f"{r['max_km']:7} {r['n_endurance_over']:4} {r.get('sep_conflicts', -1):4} "
              f"{r.get('pad_conflicts', -1):4} {r['n_corners_unflyable']:8} {r['n_corners']:6} "
              f"{r['n_agl_over_120']:8} {r['sec']:6}")

f = OUT / "step5_kpi_scan.json"
f.write_text(json.dumps(dict(
    d_use_km=D_USE_KM, r_plan_m=R_PLAN, civil_agl_m=CIVIL_AGL_M,
    bench_alt_m=M.ALT, bench_pitch_m=M.PITCH, cells=allr), indent=2, ensure_ascii=False,
    default=str), encoding="utf-8")
print("\nwrote", f)
