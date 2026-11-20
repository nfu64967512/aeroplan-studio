# -*- coding: utf-8 -*-
"""prep_baseline_control step G -- one ruler, applied to BOTH planners:
"how much cruise-equivalent distance does this route actually cost in the wind
the paper plans for?"

Why this step exists
--------------------
c5_baseline step5 compared the raw 3-D polyline length against D_use (133/79 km).
That ruler gives the baseline a free pass: D_use is a STILL-AIR number, but the
paper plans in a 5 m/s wind, and a wind triangle costs time even on a closed
out-and-back route (the headwind leg loses more than the tailwind leg gains).
Charging one planner for the wind and not the other is not a fair fight.

Ruler (identical for both planners)
-----------------------------------
    D_ceq = V_a * sum_i ( L_i / V_g(theta_i) )      [m] -> km
with L_i the horizontal length of leg i, theta_i its track bearing, and
V_g from hv_dccpp.wind.ground_speed (the library's own wind triangle,
hv_dccpp/wind.py:278-291) at the benchmark wind (5.0 m/s from 270 deg,
bench.run_planner default). Compare against
    D_use = EnergyModel.usable_distance_cap_km()  = 133 km (A) / 79 km (B)
(lower edge, because battery_monitor_validated is False for both platforms,
hv_dccpp/platform.py:451,480).

Verify-the-verifier
-------------------
Before trusting it on the baseline, the same ruler is applied to HARVEST's OWN
routes and compared against the number HARVEST itself reports
(UAVPlan.cruise_equiv_km, derived from its energy model). If the two disagree
badly the instrument is not trustworthy and the row does not go in the paper.

usage: python stepG_windfair_endurance.py
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
from hv_dccpp.platform import PLATFORM_PIONEER, PLATFORM_HERO    # noqa: E402
from hv_dccpp import wind as HW                                  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "out"
OUT.mkdir(parents=True, exist_ok=True)

SPEC = {"A": PLATFORM_PIONEER, "B": PLATFORM_HERO}
WIND_SPEED, WIND_FROM = 5.0, 270.0
WIND_XY = HW.wind_vector(WIND_SPEED, WIND_FROM)      # blowing-toward vector (east, north)
D_USE = {}
for k, s in SPEC.items():
    # usable_distance_cap_km needs an EnergyModel; the value is a pure spec lookup
    lo, hi = s.usable_distance_km
    D_USE[k] = float(hi) if s.battery_monitor_validated else float(lo)


def d_ceq_km(track_xy, v_air, min_leg=0.5):
    """cruise-equivalent km of a horizontal track under the wind triangle."""
    t_s, L_still = 0.0, 0.0
    nan_legs, slow_legs = 0, 0
    for i in range(1, len(track_xy)):
        de = track_xy[i][0] - track_xy[i - 1][0]
        dn = track_xy[i][1] - track_xy[i - 1][1]
        L = math.hypot(de, dn)
        if L < min_leg:
            continue
        theta = math.atan2(de, dn)                   # compass bearing, rad
        vg = float(HW.ground_speed(theta, v_air, WIND_XY)[0])
        if not np.isfinite(vg):
            nan_legs += 1
            continue
        if vg < 3.0:
            slow_legs += 1
            vg = 3.0
        t_s += L / vg
        L_still += L
    return dict(d_ceq_km=round(v_air * t_s / 1000.0, 3),
                d_still_km=round(L_still / 1000.0, 3),
                t_air_s=round(t_s, 1), nan_legs=nan_legs, slow_legs=slow_legs)


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
            selfrep = {uid: dict(cls=p.cls, cruise_equiv_km=round(float(p.cruise_equiv_km), 3),
                                 distance_m=round(float(p.distance_m), 1))
                       for uid, p in (plan.plans or {}).items()}
    return tracks, cls, m, sec, selfrep


# =============================== part 1: calibrate ==========================
print("=" * 96)
print("G1. VERIFY THE VERIFIER -- my wind ruler vs HARVEST's own cruise_equiv_km")
print("=" * 96)
cal = []
for scen, n in (("S1", 4), ("S3", 4)):
    tracks, cls, m, sec, selfrep = plan_cell(True, scen, n)
    # HARVEST uav ids are 'A1'.. ; assembled_paths keys are the numeric drone ids
    mine = {}
    for uid, tr in tracks.items():
        letter = cls[int(uid)]
        mine[f"{letter}{uid}"] = d_ceq_km(tr, float(SPEC[letter].cruise_airspeed_mps))
    print(f"\n  HARVEST {scen}/n={n}  (planned in {sec:.1f}s, coverage {m['coverage']})")
    print(f"  {'uav':>5} {'mine D_ceq':>11} {'HARVEST self':>13} {'ratio':>7} {'still-air km':>13}")
    for uid in sorted(mine):
        s = selfrep.get(uid, {})
        sv = s.get("cruise_equiv_km")
        r = (mine[uid]["d_ceq_km"] / sv) if sv else float("nan")
        print(f"  {uid:>5} {mine[uid]['d_ceq_km']:11.3f} {sv if sv is not None else '-':>13} "
              f"{r:7.3f} {mine[uid]['d_still_km']:13.3f}")
        cal.append(dict(scenario=scen, n=n, uav=uid, mine_km=mine[uid]["d_ceq_km"],
                        harvest_self_km=sv, ratio=None if sv is None else round(r, 4),
                        still_air_km=mine[uid]["d_still_km"]))

ratios = [c["ratio"] for c in cal if c["ratio"]]
if ratios:
    print(f"\n  ratio mine/HARVEST-self over {len(ratios)} aircraft: "
          f"min {min(ratios):.3f} max {max(ratios):.3f} mean {sum(ratios)/len(ratios):.3f}")
    print("  NOTE: my ruler charges the CRUISE leg only. HARVEST's own number comes from")
    print("  energy_used_wh, which also carries climb/descent/orbit power, so mine is")
    print("  expected to sit BELOW HARVEST's. A ratio < 1 means my ruler is the LENIENT")
    print("  one, i.e. any overrun it reports for the baseline is a lower bound.")

# =============================== part 2: baseline ===========================
print("")
print("=" * 96)
print("G2. THE SAME RULER ON THE DCCPP BASELINE (homogeneous class-A fleet)")
print("=" * 96)
CELLS = [(s, n) for s, (_p, ns) in M.SCEN.items() for n in ns]
rows = []
hdr = (f"{'cell':>8} | {'still-air max':>13} {'D_ceq max':>10} {'D_use':>6} "
       f"{'frac':>6} {'over/N':>7} {'cov':>6}")
print(hdr)
print("-" * len(hdr))
for scen, n in CELLS:
    tracks, cls, m, sec, _ = plan_cell(False, scen, n)
    per = []
    for uid, tr in tracks.items():
        letter = cls[int(uid)]
        r = d_ceq_km(tr, float(SPEC[letter].cruise_airspeed_mps))
        r.update(uav=uid, cls=letter, d_use_km=D_USE[letter],
                 frac=round(r["d_ceq_km"] / D_USE[letter], 4),
                 over=bool(r["d_ceq_km"] > D_USE[letter]))
        per.append(r)
    mx = max(per, key=lambda r: r["frac"])
    n_over = sum(1 for r in per if r["over"])
    rows.append(dict(scenario=scen, n=n, coverage=m["coverage"],
                     max_still_km=max(r["d_still_km"] for r in per),
                     max_d_ceq_km=mx["d_ceq_km"], d_use_km=mx["d_use_km"],
                     max_frac=mx["frac"], n_over=n_over, n_uav=len(per), per_uav=per))
    print(f"{scen+'/'+str(n):>8} | {max(r['d_still_km'] for r in per):13.2f} "
          f"{mx['d_ceq_km']:10.2f} {mx['d_use_km']:6.0f} {mx['frac']:6.3f} "
          f"{str(n_over)+'/'+str(len(per)):>7} {m['coverage']:6}")

f = OUT / "stepG_windfair_endurance.json"
f.write_text(json.dumps(dict(
    wind=dict(speed_mps=WIND_SPEED, from_deg=WIND_FROM, xy=[float(x) for x in np.asarray(WIND_XY).ravel()]),
    d_use_km=D_USE, ruler="D_ceq = Va * sum(L_i / Vg(theta_i))",
    calibration=cal, dccpp_cells=rows), indent=2, ensure_ascii=False, default=str),
    encoding="utf-8")
print("")
print("wrote", f)
