# -*- coding: utf-8 -*-
"""prep_baseline_control step I -- apply HARVEST'S OWN acceptance rule to the
DCCPP baseline's routes.

HARVEST's rule (hv_dccpp/sim.py:913-917):
    D_plan = EnergyModel.cruise_equiv_km(plan.energy_used_wh)  <=  D_use
with D_use = usable_distance_cap_km() = 133 km (A) / 79 km (B).

energy_used_wh is a GROSS sortie figure: cruise + VTOL takeoff + VTOL landing.
So the like-for-like estimator for any route is

    D_plan ~= D_ceq(cruise, wind triangle)  +  cruise_equiv_km(e_TO + e_LDG)

The second term is a per-platform constant. This script computes it from the
library, then CALIBRATES the estimator on HARVEST's own routes (where the true
D_plan is known from UAVPlan.cruise_equiv_km) before using it on the baseline.

usage: python stepI_dplan_rule.py
"""
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

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
from hv_dccpp.platform import (PLATFORM_PIONEER, PLATFORM_HERO,  # noqa: E402
                               EnergyModel)
from hv_dccpp import wind as HW                                  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "out"
OUT.mkdir(parents=True, exist_ok=True)

SPEC = {"A": PLATFORM_PIONEER, "B": PLATFORM_HERO}
# AUW / battery from the planner's own preflight per_uav block (stepF evidence):
#   class A: auw 24.6 kg, E_nom 1500 Wh, E_avail 1350 Wh
BATT = {"A": dict(auw_kg=24.6, nominal_wh=1500.0, available_wh=1350.0,
                  reference_wh=1500.0)}
WIND_SPEED, WIND_FROM = 5.0, 270.0
WIND_XY = HW.wind_vector(WIND_SPEED, WIND_FROM)


def model_for(cls):
    if cls not in BATT:
        raise KeyError(f"no battery record for class {cls}")
    return EnergyModel(spec=SPEC[cls], **BATT[cls])


OVERHEAD = {}
for cls in BATT:
    m = model_for(cls)
    e_fixed = float(m.e_takeoff_wh()) + float(m.e_landing_wh())
    OVERHEAD[cls] = dict(e_TO_wh=round(float(m.e_takeoff_wh()), 2),
                         e_LDG_wh=round(float(m.e_landing_wh()), 2),
                         e_fixed_wh=round(e_fixed, 2),
                         d_fixed_km=round(float(m.cruise_equiv_km(e_fixed)), 3),
                         D_use_km=float(m.usable_distance_cap_km()),
                         k_unc=round(float(m.k_unc), 4),
                         P_c_w=round(float(m.p_cruise_w()), 1))
print("=" * 90)
print("I0. per-platform fixed VTOL overhead, expressed in cruise-equivalent km")
print("=" * 90)
for cls, d in OVERHEAD.items():
    print(f"  class {cls}: e_TO={d['e_TO_wh']} Wh  e_LDG={d['e_LDG_wh']} Wh  "
          f"-> {d['d_fixed_km']} km of the {d['D_use_km']:.0f} km budget "
          f"(k_unc={d['k_unc']}, P_c={d['P_c_w']} W)")


def d_ceq_km(track_xy, v_air, min_leg=0.5):
    t_s, L = 0.0, 0.0
    for i in range(1, len(track_xy)):
        de = track_xy[i][0] - track_xy[i - 1][0]
        dn = track_xy[i][1] - track_xy[i - 1][1]
        seg = math.hypot(de, dn)
        if seg < min_leg:
            continue
        vg = float(HW.ground_speed(math.atan2(de, dn), v_air, WIND_XY)[0])
        if not np.isfinite(vg) or vg < 3.0:
            vg = 3.0
        t_s += seg / vg
        L += seg
    return v_air * t_s / 1000.0, L / 1000.0


def plan_cell(use_hv, scen, n):
    polys, _ = M.SCEN[scen]
    drones, cls = _drones(n, 0, M.PITCH)
    c = SwarmCoordinator()
    if use_hv:
        c.use_harvest = True
        c.hv_class_of = lambda d: cls[int(d["drone_id"])]
        c.hv_home = HOME_LL
        c.hv_wind = (WIND_SPEED, WIND_FROM)
        c.hv_fence_zones = []
        c.harvest_params = {"hfov_deg": 60.0, "vfov_deg": 45.0, "hv_c2_range_km": 25.0,
                            "hv_waiver_max_agl_m": 200.0, "hv_dem_margin_m": 3000.0}
    t0 = time.perf_counter()
    res = c.plan_coverage_dccpp(areas=polys_to_areas(polys), drones=drones,
                                coverage_width_m=M.PITCH, overlap_rate=0.0,
                                coverage_altitude=M.ALT, mounting_angle_deg=0.0)
    sec = time.perf_counter() - t0
    ap = res.get("assembled_paths") or {}
    tracks = {uid: [xy(w.lat, w.lon) for w in p.waypoints] for uid, p in ap.items()}
    m = metrics(polys, [t for t in tracks.values() if len(t) >= 2], M.SWATH)
    selfrep = {}
    if use_hv:
        hv = getattr(c, "hv_coordinator", None)
        plan = getattr(hv, "last_plan", None) if hv else None
        if plan is not None:
            selfrep = {uid: round(float(p.cruise_equiv_km), 3)
                       for uid, p in (plan.plans or {}).items()}
    return tracks, cls, m, sec, selfrep


# ---------------- I1: calibrate the estimator on HARVEST's own routes -------
print("")
print("=" * 90)
print("I1. CALIBRATION -- estimator vs HARVEST's own D_plan, on HARVEST's own routes")
print("=" * 90)
cal = []
for scen, n in (("S1", 4), ("S3", 4), ("S4", 4)):
    tracks, cls, m, sec, selfrep = plan_cell(True, scen, n)
    print(f"\n  HARVEST {scen}/n={n} (coverage {m['coverage']}, {sec:.0f} s)")
    print(f"  {'uav':>4} {'D_ceq':>8} {'+fixed':>8} {'=est':>8} {'HARVEST':>8} {'err%':>7}")
    for uid, tr in sorted(tracks.items()):
        letter = cls[int(uid)]
        dceq, still = d_ceq_km(tr, float(SPEC[letter].cruise_airspeed_mps))
        est = dceq + OVERHEAD[letter]["d_fixed_km"]
        truth = selfrep.get(f"{letter}{uid}")
        err = (est / truth - 1.0) * 100.0 if truth else float("nan")
        print(f"  {letter}{uid:>3} {dceq:8.2f} {OVERHEAD[letter]['d_fixed_km']:8.2f} "
              f"{est:8.2f} {truth if truth is not None else '-':>8} {err:7.2f}")
        cal.append(dict(scenario=scen, n=n, uav=f"{letter}{uid}", cls=letter,
                        d_ceq_km=round(dceq, 3), est_km=round(est, 3),
                        harvest_self_km=truth,
                        err_pct=None if truth is None else round(err, 3),
                        still_air_km=round(still, 3)))
errs = [c["err_pct"] for c in cal if c["err_pct"] is not None]
if errs:
    print(f"\n  estimator error over {len(errs)} HARVEST aircraft: "
          f"min {min(errs):+.2f}%  max {max(errs):+.2f}%  mean {sum(errs)/len(errs):+.2f}%")
    print("  (positive = estimator OVERstates the cost, i.e. it is the STRICT direction)")

# ---------------- I2: the rule applied to the DCCPP baseline ---------------
print("")
print("=" * 90)
print("I2. HARVEST's rule D_plan <= D_use applied to the DCCPP baseline (class-A fleet)")
print("=" * 90)
CELLS = [(s, n) for s, (_p, ns) in M.SCEN.items() for n in ns]
rows = []
hdr = (f"{'cell':>8} | {'still-air':>9} {'D_ceq':>8} {'D_plan est':>10} {'D_use':>6} "
       f"{'frac':>6} {'over/N':>7} {'verdict':>10}")
print(hdr)
print("-" * len(hdr))
for scen, n in CELLS:
    tracks, cls, m, sec, _ = plan_cell(False, scen, n)
    per = []
    for uid, tr in tracks.items():
        letter = cls[int(uid)]
        dceq, still = d_ceq_km(tr, float(SPEC[letter].cruise_airspeed_mps))
        est = dceq + OVERHEAD[letter]["d_fixed_km"]
        duse = OVERHEAD[letter]["D_use_km"]
        per.append(dict(uav=uid, cls=letter, still_air_km=round(still, 3),
                        d_ceq_km=round(dceq, 3), d_plan_est_km=round(est, 3),
                        d_use_km=duse, frac=round(est / duse, 4), over=bool(est > duse)))
    mx = max(per, key=lambda r: r["frac"])
    n_over = sum(1 for r in per if r["over"])
    rows.append(dict(scenario=scen, n=n, coverage=m["coverage"], n_uav=len(per),
                     n_over=n_over, max_frac=mx["frac"], max_d_plan_est_km=mx["d_plan_est_km"],
                     max_still_km=max(r["still_air_km"] for r in per), per_uav=per))
    print(f"{scen+'/'+str(n):>8} | {max(r['still_air_km'] for r in per):9.2f} "
          f"{mx['d_ceq_km']:8.2f} {mx['d_plan_est_km']:10.2f} {mx['d_use_km']:6.0f} "
          f"{mx['frac']:6.3f} {str(n_over)+'/'+str(len(per)):>7} "
          f"{'EXCEEDS' if n_over else 'within':>10}")

f = OUT / "stepI_dplan_rule.json"
f.write_text(json.dumps(dict(overhead=OVERHEAD, battery_record=BATT,
                             wind=dict(speed_mps=WIND_SPEED, from_deg=WIND_FROM),
                             calibration=cal, dccpp_cells=rows),
                        indent=2, ensure_ascii=False, default=str), encoding="utf-8")
print("")
print("wrote", f)
