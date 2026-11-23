# -*- coding: utf-8 -*-
"""C5 step 4 -- VERIFY THE VERIFIER.

step3 reported ZERO curvature/AGL/slope violations for the DCCPP baseline.
An instrument that cannot fire is worthless as evidence, so before that zero
goes in a paper it has to be shown to be a real zero:

  A. count how many waypoint triples check_kinematics actually EXAMINED
     (FW-phase, chord >= 0.5 m, turn >= 0.5 deg) -- if 0, the check no-op'd;
  B. inject a deliberate 90-deg hairpin into the same lifted plan and confirm
     curvature_below_R_eff fires;
  C. raise the altitude above the ceiling and confirm above_ceiling fires;
  D. print the R_eff the check is measuring against.

usage: python step4_verify_the_verifier.py <SCEN> <N> [mix]
"""
import json
import math
import sys
from pathlib import Path

REPO = Path(r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
BENCH = REPO / "research/cpp_benchmark"
for p in (str(REPO), str(BENCH)):
    if p not in sys.path:
        sys.path.insert(0, p)

import logging                                               # noqa: E402
logging.disable(logging.CRITICAL)

import matrix as M                                           # noqa: E402
from bench import _drones, polys_to_areas, xy, HOME_LL       # noqa: E402
from mission.swarm_coordinator import SwarmCoordinator       # noqa: E402
from hv_dccpp.mission import Waypoint, UAVPlan, Phase, Mode, FW_PHASES   # noqa: E402
from hv_dccpp import sim as HSIM                             # noqa: E402

# NOTE: lift() is copied verbatim from step3_kpi_audit.py rather than imported,
# because importing that module would re-run its whole main body.
PHASE_OF = {"TAKEOFF": Phase.TAKEOFF, "ENTRY": Phase.ENTRY, "OPERATION": Phase.OPERATE,
            "TRANSFER": Phase.TRANSFER, "LANDING": Phase.LAND}
_HOVER_EPS = 1.0
V_VERT = 2.5


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
        ph_ = PHASE_OF.get(getattr(w.segment_type, "name", "TRANSFER"), Phase.TRANSFER)
        vert_leg = ph_ in (Phase.TAKEOFF, Phase.LAND)
        out.append(Waypoint(
            x_m=x_, y_m=y_, z_m=float(w.alt), lat_deg=w.lat, lon_deg=w.lon,
            alt_msl_m=float(w.alt), agl_m=float(w.alt), t_s=t,
            mode=Mode.HOVER if vert_leg else Mode.CRUISE, phase=ph_,
            sensor_on=(ph_ is Phase.OPERATE), strip_id=None, cell_id=None, dir=None,
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

SCEN, N = sys.argv[1], int(sys.argv[2])
MIX = len(sys.argv) > 3 and sys.argv[3] == "mix"
OUT = Path(__file__).resolve().parents[1] / "out"
POLYS, _ = M.SCEN[SCEN]
N_A, N_B = (N - N // 2, N // 2) if MIX else (N, 0)


def run(use_hv):
    drones, cls = _drones(N_A, N_B, M.PITCH)
    c = SwarmCoordinator()
    if use_hv:
        c.use_harvest = True
        c.hv_class_of = lambda d: cls[int(d["drone_id"])]
        c.hv_home = HOME_LL
        c.hv_wind = (5.0, 270.0)
        c.hv_fence_zones = []
        c.harvest_params = {"hfov_deg": 60.0, "vfov_deg": 45.0, "hv_c2_range_km": 25.0,
                            "hv_waiver_max_agl_m": 200.0, "hv_dem_margin_m": 3000.0}
    return c, c.plan_coverage_dccpp(areas=polys_to_areas(POLYS), drones=drones,
                                    coverage_width_m=M.PITCH, overlap_rate=0.0,
                                    coverage_altitude=M.ALT, mounting_angle_deg=0.0), cls


ch, rh, cls_map = run(True)
hvc = ch.hv_coordinator
ctx, cfg = hvc.last_ctx, hvc.cfg
cd, rd, _ = run(False)
ap = rd.get("assembled_paths") or {}

uid0 = next(iter(ap))
spec = ctx.specs[cls_map[int(uid0)]]
plan, _d = lift(ap[uid0], f"D{uid0}", cls_map[int(uid0)], float(spec.cruise_airspeed_mps))
rep = {"scenario": SCEN, "n": N, "mix": MIX, "uav": plan.uav_id, "cls": plan.cls,
       "n_waypoints": len(plan.waypoints)}

# ---- A. how many triples were examined? (mirrors sim.check_kinematics lines 655-672)
wps = plan.waypoints
x = [w.x_m for w in wps]
y = [w.y_m for w in wps]
ph = [w.phase for w in wps]
dx = [x[i + 1] - x[i] for i in range(len(x) - 1)]
dy = [y[i + 1] - y[i] for i in range(len(y) - 1)]
chord = [math.hypot(a, b) for a, b in zip(dx, dy)]
examined, skipped = 0, {"phase": 0, "chord": 0, "turn": 0, "degenerate": 0}
radii = []
for j in range(1, len(wps) - 1):
    if ph[j - 1] not in FW_PHASES or ph[j] not in FW_PHASES:
        skipped["phase"] += 1
        continue
    a, b = chord[j - 1], chord[j]
    if a < 0.5 or b < 0.5:
        skipped["chord"] += 1
        continue
    h0 = math.atan2(dx[j - 1], dy[j - 1])
    h1 = math.atan2(dx[j], dy[j])
    turn = abs((h1 - h0 + math.pi) % (2 * math.pi) - math.pi)
    if turn < math.radians(0.5):
        skipped["turn"] += 1
        continue
    area2 = abs(dx[j - 1] * dy[j] - dy[j - 1] * dx[j])
    if area2 < 1e-12:
        skipped["degenerate"] += 1
        continue
    c = math.hypot(x[j + 1] - x[j - 1], y[j + 1] - y[j - 1])
    radii.append(a * b * c / (2.0 * area2))
    examined += 1

R_eff = HSIM._r_eff_for_plan(spec, ctx, wps)
phase_hist = {}
for p_ in ph:
    phase_hist[p_.name] = phase_hist.get(p_.name, 0) + 1
rep["A_examined_triples"] = examined
rep["A_skipped"] = skipped
rep["A_phase_histogram"] = phase_hist
rep["A_R_eff_m"] = round(R_eff, 1)
rep["A_min_circumradius_m"] = round(min(radii), 1) if radii else None
rep["A_pctile_circumradius_m"] = ([round(sorted(radii)[int(q * (len(radii) - 1))], 1)
                                   for q in (0.0, 0.05, 0.5, 0.95, 1.0)] if radii else None)
rep["A_baseline_violations"] = HSIM.check_kinematics(plan, spec, ctx, cfg, strips=None)

# ---- B. inject a REAL corner -----------------------------------------------
# B1 first attempt (kept for the record): a 40 m kink between two OPERATE
# waypoints whose chords are ~2 km deflects the path by only ~1.6 deg, so the
# circumradius stays enormous and NOTHING fires. That is not evidence the
# checker is broken -- it is evidence the injection was too weak.
import copy                                                   # noqa: E402
bad = copy.deepcopy(plan)
idx = next((j for j in range(1, len(bad.waypoints) - 1)
            if bad.waypoints[j].phase in FW_PHASES and bad.waypoints[j - 1].phase in FW_PHASES
            and chord[j - 1] > 20 and chord[j] > 20), None)
rep["B1_inject_index"] = idx
rep["B1_kink_m"] = 40.0
if idx is not None:
    bad.waypoints[idx].x_m += 40.0
    bad.waypoints[idx].y_m += 40.0
    o = HSIM.check_kinematics(bad, spec, ctx, cfg, strips=None)
    rep["B1_fired"] = any("curvature_below_R_eff" in s for s in o)
    rep["B1_chords_m"] = [round(chord[idx - 1], 1), round(chord[idx], 1)]
    rep["B1_deflection_deg"] = round(math.degrees(math.atan2(56.6, chord[idx])), 2)

# B2: displace the SAME waypoint perpendicular by half the chord -> ~53 deg turn.
bad2 = copy.deepcopy(plan)
if idx is not None:
    L = min(chord[idx - 1], chord[idx])
    ux, uy = dx[idx - 1] / chord[idx - 1], dy[idx - 1] / chord[idx - 1]
    bad2.waypoints[idx].x_m += -uy * 0.5 * L        # perpendicular
    bad2.waypoints[idx].y_m += ux * 0.5 * L
    o2 = HSIM.check_kinematics(bad2, spec, ctx, cfg, strips=None)
    rep["B2_offset_m"] = round(0.5 * L, 1)
    rep["B2_violations"] = [s for s in o2 if "curvature" in s][:3]
    rep["B2_fired"] = any("curvature_below_R_eff" in s for s in o2)

# B3: synthetic ground truth -- three OPERATE waypoints, 200 m legs, 90 deg
# corner. Circumradius = 200*200*282.8/(2*40000) = 141.4 m < R_eff 200 m.
syn = copy.deepcopy(plan)
w0 = copy.deepcopy(syn.waypoints[0])
pts = [(0.0, 0.0), (0.0, 200.0), (200.0, 200.0)]
syn.waypoints = []
for k, (px, py) in enumerate(pts):
    w = copy.deepcopy(w0)
    w.x_m, w.y_m, w.z_m = px, py, 140.0
    w.agl_m = w.alt_msl_m = 140.0
    w.t_s = k * 200.0 / float(spec.cruise_airspeed_mps)
    w.phase, w.mode = Phase.OPERATE, Mode.CRUISE
    w.airspeed_mps = w.ground_speed_mps = float(spec.cruise_airspeed_mps)
    syn.waypoints.append(w)
o3 = HSIM.check_kinematics(syn, spec, ctx, cfg, strips=None)
rep["B3_expected_circumradius_m"] = round(200 * 200 * math.hypot(200, 200) / (2 * 40000), 1)
rep["B3_violations"] = [s for s in o3 if "curvature" in s][:3]
rep["B3_fired"] = any("curvature_below_R_eff" in s for s in o3)

# ---- C. raise altitude above the ceiling ----------------------------------
hi = copy.deepcopy(plan)
for w in hi.waypoints:
    if w.phase in FW_PHASES:
        w.z_m += 200.0
        w.agl_m += 200.0
        w.alt_msl_m += 200.0
out_hi = HSIM.check_kinematics(hi, spec, ctx, cfg, strips=None)
rep["C_violations"] = out_hi[:5]
rep["C_fired"] = any("above_ceiling" in s for s in out_hi)
rep["C_h_ceil_m"] = round(HSIM._h_ceil_of(ctx, cls=plan.cls), 1)
rep["C_min_fw_operation_agl_m"] = float(spec.min_fw_operation_agl_m)

# ---- D. what does the DCCPP path's own geometry look like? ----------------
rep["D_note"] = ("R_eff at 5 m/s equals R_plan for both classes (see step0_reff.log); "
                 "DCCPP builds Dubins arcs at min_turn_radius = the same R_plan it was handed.")

tag = f"{SCEN}_n{N}" + ("_mix" if MIX else "")
f = OUT / f"step4_verifier_{tag}.json"
f.write_text(json.dumps(rep, indent=2, ensure_ascii=False, default=str), encoding="utf-8")
print(json.dumps(rep, indent=2, ensure_ascii=False, default=str))
print("\nwrote", f)
