# -*- coding: utf-8 -*-
"""C5 step 3 -- THE KPI EXPERIMENT.

Run one benchmark cell through BOTH planners in one process, then feed the
DCCPP baseline's own output into HARVEST's own post-hoc checkers, using
HARVEST's own MissionContext for that same cell. Same area, same fleet, same
wind, same checkers -- only the planner differs. Count violations.

Checks reused verbatim from the vendored library:
  hv_dccpp.sim.check_kinematics        -> curvature_below_R_eff (heterogeneous turn
                                          radius), slope_exceeded, agl_*, above_ceiling
  hv_dccpp.deconflict.check_separation -> 4-D separation
  hv_dccpp.deconflict.CorridorSchedule -> shared takeoff/landing pad
  hv_dccpp.platform EnergyModel.usable_distance_cap_km -> endurance

usage: python step3_kpi_audit.py <SCEN> <N> [mix]
"""
import json
import math
import sys
import time
import traceback
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
from hv_dccpp import sim as HSIM, deconflict as HDEC             # noqa: E402

SCEN, N = sys.argv[1], int(sys.argv[2])
MIX = len(sys.argv) > 3 and sys.argv[3] == "mix"
OUT = Path(__file__).resolve().parents[1] / "out"
OUT.mkdir(parents=True, exist_ok=True)
POLYS, _ = M.SCEN[SCEN]
N_A, N_B = (N - N // 2, N // 2) if MIX else (N, 0)
WIND = (5.0, 270.0)

PHASE_OF = {"TAKEOFF": Phase.TAKEOFF, "ENTRY": Phase.ENTRY, "OPERATION": Phase.OPERATE,
            "TRANSFER": Phase.TRANSFER, "LANDING": Phase.LAND}
_HOVER_EPS = 1.0          # core/dccpp/dccpp_path_builder._HOVER_LEG_EPS_M
V_VERT = 2.5              # vertical speed for the DCCPP makespan estimator


def run(use_hv):
    drones, cls = _drones(N_A, N_B, M.PITCH)
    c = SwarmCoordinator()
    if use_hv:
        c.use_harvest = True
        c.hv_class_of = lambda d: cls[int(d["drone_id"])]
        c.hv_home = HOME_LL
        c.hv_wind = WIND
        c.hv_fence_zones = []
        c.harvest_params = {"hfov_deg": 60.0, "vfov_deg": 45.0, "hv_c2_range_km": 25.0,
                            "hv_waiver_max_agl_m": 200.0, "hv_dem_margin_m": 3000.0}
    res = c.plan_coverage_dccpp(areas=polys_to_areas(POLYS), drones=drones,
                                coverage_width_m=M.PITCH, overlap_rate=0.0,
                                coverage_altitude=M.ALT, mounting_angle_deg=0.0)
    return c, res, cls


def lift(built, uav_id, cls_letter, v_cruise):
    """DCCPP BuiltPath -> hv_dccpp UAVPlan.

    t_s uses the PROJECT'S OWN leg model (core/dccpp/dccpp_path_builder.
    estimate_flight_time_s): pure-vertical legs at V_VERT, everything else
    3-D length / cruise. Nothing invented -- this is the timing DCCPP itself
    reports as makespan.
    """
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
        x, y = xy(w.lat, w.lon)
        ph = PHASE_OF.get(getattr(w.segment_type, "name", "TRANSFER"), Phase.TRANSFER)
        vert_leg = ph in (Phase.TAKEOFF, Phase.LAND)
        out.append(Waypoint(
            x_m=x, y_m=y, z_m=float(w.alt), lat_deg=w.lat, lon_deg=w.lon,
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


def kinds(lines):
    d = {}
    for s in lines:
        if s.startswith("..."):
            continue
        k = s.split(":", 1)[0]
        d[k] = d.get(k, 0) + 1
    return d


rep = {"scenario": SCEN, "n": N, "mix": MIX, "wind_mps_fromdeg": list(WIND),
       "alt_m": M.ALT, "pitch_m": M.PITCH, "n_a": N_A, "n_b": N_B}

# ---- HARVEST ---------------------------------------------------------------
t0 = time.perf_counter()
ch, rh, cls_map = run(True)
rep["harvest_sec"] = round(time.perf_counter() - t0, 1)
hvc = getattr(ch, "hv_coordinator", None)
ctx, cfg = hvc.last_ctx, hvc.cfg
v = ch.hv_validation
h_tracks = [t for t in ([xy(w.lat, w.lon) for w in p.waypoints]
                        for p in (rh.get("assembled_paths") or {}).values()) if len(t) >= 2]
hm = metrics(POLYS, h_tracks, M.SWATH)
rep["harvest"] = dict(
    makespan_s=rh.get("total_makespan"), max_km=hm["max_km"], total_km=hm["total_km"],
    coverage=hm["coverage"],
    passed=bool(v.passed), coverage_ok=bool(v.coverage_ok), energy_ok=bool(v.energy_ok),
    curvature_ok=bool(v.curvature_ok), agl_ok=bool(v.agl_ok), slope_ok=bool(v.slope_ok),
    airspace_ok=bool(v.airspace_ok), link_ok=bool(v.link_ok), integrity_ok=bool(v.integrity_ok),
    separation_ok=bool(v.separation_ok), n_conflicts=len(v.conflicts),
    min_sep_h_m=round(v.min_sep_h_m, 1), min_sep_v_m=round(v.min_sep_v_m, 1),
    total_distance_m=round(v.total_distance_m, 1),
    uncovered=len(hvc.last_plan.uncovered or ()),
    per_uav_violations={u: kinds(v.details.get(u, [])) for u in hvc.last_plan.plans},
    max_leg_km=round(max((p.distance_m for p in hvc.last_plan.plans.values()), default=0) / 1000.0, 2),
)

# ---- DCCPP -----------------------------------------------------------------
t0 = time.perf_counter()
cd, rd, _ = run(False)
rep["dccpp_sec"] = round(time.perf_counter() - t0, 1)
ap = rd.get("assembled_paths") or {}
d_tracks = [t for t in ([xy(w.lat, w.lon) for w in p.waypoints] for p in ap.values()) if len(t) >= 2]
dm = metrics(POLYS, d_tracks, M.SWATH)
plans, dist, seg_hist, alt_rng = {}, {}, {}, []
for uid, built in ap.items():
    letter = cls_map[int(uid)]
    spec = ctx.specs[letter]
    p, d = lift(built, f"D{uid}", letter, float(spec.cruise_airspeed_mps))
    plans[p.uav_id] = p
    dist[p.uav_id] = d
    for w in built.waypoints:
        k = getattr(w.segment_type, "name", "?")
        seg_hist[k] = seg_hist.get(k, 0) + 1
    alt_rng += [float(w.alt) for w in built.waypoints]

first = next(iter(ap.values()), None)
rep["dccpp_raw"] = dict(
    makespan_s=rd.get("total_makespan"), **dm, segment_types=seg_hist,
    alt_min_m=round(min(alt_rng), 2) if alt_rng else None,
    alt_max_m=round(max(alt_rng), 2) if alt_rng else None,
    builtwaypoint_fields=(sorted(vars(first.waypoints[0])) if first and first.waypoints else []),
    t_land_s={u: round(p.waypoints[-1].t_s, 1) for u, p in plans.items()},
)

# ---- HARVEST's own checks applied to the DCCPP output ----------------------
kin, kin_sample = {}, {}
for uid, p in plans.items():
    spec = ctx.specs[p.cls]
    try:
        lines = HSIM.check_kinematics(p, spec, ctx, cfg, strips=None)
    except Exception as e:                                       # noqa: BLE001
        lines = [f"check_kinematics_raised: {type(e).__name__}: {e}"]
    kin[uid] = kinds(lines)
    if len(kin_sample) < 2:
        kin_sample[uid] = lines[:8]
rep["dccpp_kinematics"] = kin
rep["dccpp_kinematics_sample"] = kin_sample

try:
    conf = HDEC.check_separation(dict(plans), cfg, t_sync_s=0.0)
    mh, mv, tmin, pair = HSIM.min_separation(dict(plans), cfg)
    kk = {}
    for c in conf:
        kk[c.kind] = kk.get(c.kind, 0) + 1
    rep["dccpp_separation"] = dict(
        ran=True, n_conflicts=len(conf), kinds=kk,
        sample=[f"{c.kind} {c.uav_a}-{c.uav_b} t={c.t_s:.0f} d_h={c.d_h_m:.0f} d_v={c.d_v_m:.0f}"
                for c in conf[:8]],
        min_sep_h_m=round(mh, 1), min_sep_v_m=round(mv, 1),
        at_t_s=(None if not math.isfinite(tmin) else round(tmin, 1)),
        pair=(list(pair) if pair else None))
except Exception as e:                                           # noqa: BLE001
    rep["dccpp_separation"] = dict(ran=False, error=f"{type(e).__name__}: {e}",
                                   tb=traceback.format_exc()[-900:])

try:
    sch = HDEC.CorridorSchedule(cfg)
    for p in plans.values():
        sch.reserve_plan(p, "H")
    pc = sch.conflicts()
    rep["dccpp_pad"] = dict(ran=True, n_conflicts=len(pc),
                            sample=[f"{c.kind} {c.uav_a}-{c.uav_b} t={c.t_s:.0f}" for c in pc[:8]])
except Exception as e:                                           # noqa: BLE001
    rep["dccpp_pad"] = dict(ran=False, error=f"{type(e).__name__}: {e}",
                            tb=traceback.format_exc()[-900:])

end = {}
for uid, p in plans.items():
    m = ctx.models.get(uid) or ctx.models_rep[p.cls]
    cap_km = float(m.usable_distance_cap_km())
    end[uid] = dict(cls=p.cls, d_plan_km=round(dist[uid] / 1000.0, 2), d_use_km=cap_km,
                    over=bool(dist[uid] / 1000.0 > cap_km),
                    frac=round(dist[uid] / 1000.0 / cap_km, 3))
rep["dccpp_endurance"] = end
rep["dccpp_endurance_n_over"] = sum(1 for e in end.values() if e["over"])

tag = f"{SCEN}_n{N}" + ("_mix" if MIX else "")
f = OUT / f"step3_kpi_{tag}.json"
f.write_text(json.dumps(rep, indent=2, ensure_ascii=False, default=str), encoding="utf-8")
print(json.dumps(rep, indent=2, ensure_ascii=False, default=str))
print("\nwrote", f)
