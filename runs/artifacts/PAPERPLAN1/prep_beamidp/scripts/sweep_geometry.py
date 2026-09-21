# -*- coding: utf-8 -*-
"""prep_beamidp step 1: find a 1 Pioneer + 2 HERO geometry at the 119.5 m ceiling
whose per-class cells genuinely exceed the skip-k threshold
(Pioneer cell n >= 6 with k=5, HERO cell n >= 4 with k=3).

Pure Python planner runs only.  No SITL, no repo writes outside this folder.
"""
from __future__ import annotations
import json, os, sys, time
import numpy as np

ROOT = r"C:/Users/joy46/Documents/Projects/aeroplan-studio"
sys.path.insert(0, ROOT); os.chdir(ROOT)
HERE = os.path.join(ROOT, "runs/artifacts/PAPERPLAN1/prep_beamidp")

from scripts.hv_sitl_scenarios import Scenario, rect          # noqa: E402
from scripts.hv_sitl_run import _plan_gui                     # noqa: E402
from hv_dccpp.dubins import skip_k                            # noqa: E402
from hv_dccpp.decompose import r_eff_for_class                # noqa: E402

CAP = 119.5
# identical to gui_legal_120_1a2b except coverage_altitude_m / waiver_max_agl_m
AIRSPACE = dict(coverage_altitude_m=CAP, coverage_width_m=150.0, overlap_rate=0.30,
                waiver_max_agl_m=CAP, min_fw_agl_m={"A": 100.0, "B": 100.0},
                dh_band_min_m=15.0, target_gsd=0.05, spawn_spread_m=10.0)


def grid_areas(n, half_e, half_n, pitch, row=0.0):
    x0 = -pitch * (n - 1) / 2.0
    return [{"area_id": i + 1, "priority": 1.0,
             "polygon": rect(x0 + i * pitch, row, half_e, half_n)}
            for i in range(n)]


def strips_of(plan, ctx):
    got = []

    def _flat(x):
        if isinstance(x, dict):
            [_flat(v) for v in x.values()]
        elif isinstance(x, (list, tuple, set)):
            [_flat(v) for v in x]
        elif hasattr(x, "spacing_m") and hasattr(x, "cell_id"):
            got.append(x)
    _flat(getattr(plan, "strips", None) or getattr(ctx, "strips", None))
    return got


def run(key, areas):
    sc = Scenario(key=key, trial="PAPERPLAN1_PREP", path="gui", n_a=1, n_b=2,
                  areas=areas, note="prep_beamidp sweep", **AIRSPACE)
    t0 = time.perf_counter()
    hv, report = _plan_gui(sc)
    dt = time.perf_counter() - t0
    plan, ctx = hv.last_plan, hv.last_ctx
    ss = strips_of(plan, ctx)
    cells, groups = {}, {}
    for s in ss:
        groups.setdefault((s.cell_id, s.cls), []).append(s)
    max_a, max_b = 0, 0
    for (cid, cls), lst in sorted(groups.items()):
        s_mean = float(np.mean([x.spacing_m for x in lst]))
        R = float(r_eff_for_class(ctx, cls))
        k = skip_k(s_mean, R)
        n = len(lst)
        cells["%s|%s" % (cid, cls)] = dict(
            n=n, s=round(s_mean, 2), L=round(float(np.mean([x.length_m for x in lst])), 1),
            R_eff=R, k=k, triggers=bool(n > k))
        if cls == "A":
            max_a = max(max_a, n)
        else:
            max_b = max(max_b, n)
    out = dict(key=key, area_km2=None, plan_s=round(dt, 1),
               status=report.get("status"), makespan_s=round(float(plan.makespan_s), 1),
               uncovered=len(plan.uncovered), n_areas=len(areas),
               max_n_A=max_a, max_n_B=max_b,
               meets=bool(max_a >= 6 and max_b >= 4), cells=cells,
               uavs={u: dict(cls=p.cls, strips=len(p.strips),
                             km=round(p.distance_m / 1000.0, 2),
                             wh=round(float(p.energy_used_wh), 1),
                             margin_wh=round(float(p.energy_margin_wh), 1))
                     for u, p in sorted(plan.plans.items())})
    return out


CASES = [
    ("A_three_1560x800_p2000", grid_areas(3, 780, 400, 2000)),
    ("B_three_1560x800_p1800", grid_areas(3, 780, 400, 1800)),
    ("C_three_1200x1000_p1700", grid_areas(3, 600, 500, 1700)),
    ("D_three_1600x1200_p2100", grid_areas(3, 800, 600, 2100)),
    ("E_three_1600x1400_p2100", grid_areas(3, 800, 700, 2100)),
    ("F_three_1400x1300_p1900", grid_areas(3, 700, 650, 1900)),
    ("G_three_1600x1300_p2100", grid_areas(3, 800, 650, 2100)),
    ("Z_one_3000x3000", [{"area_id": 1, "priority": 1.0, "polygon": rect(0, 0, 1500, 1500)}]),
]

if __name__ == "__main__":
    only = sys.argv[1:] or None
    res = []
    for key, areas in CASES:
        if only and key not in only:
            continue
        try:
            r = run(key, areas)
        except BaseException as e:                              # noqa: BLE001
            r = dict(key=key, error="%s: %s" % (type(e).__name__, e))
        res.append(r)
        print("=" * 96)
        print("%s  plan=%ss status=%s makespan=%s uncovered=%s  maxA=%s maxB=%s MEETS=%s"
              % (r.get("key"), r.get("plan_s"), r.get("status"), r.get("makespan_s"),
                 r.get("uncovered"), r.get("max_n_A"), r.get("max_n_B"), r.get("meets")))
        for cid, c in (r.get("cells") or {}).items():
            print("    cell %-24s n=%-3s s=%-7s L=%-8s R=%-6s k=%s trig=%s"
                  % (cid, c["n"], c["s"], c["L"], c["R_eff"], c["k"], c["triggers"]))
        for u, d in (r.get("uavs") or {}).items():
            print("    %s %s strips=%s %.2f km  E=%s/%s Wh"
                  % (u, d["cls"], d["strips"], d["km"], d["wh"], d["margin_wh"]))
        if "error" in r:
            print("    ERROR:", r["error"])
        sys.stdout.flush()
    dst = os.path.join(HERE, "sweep_geometry.json")
    json.dump(res, open(dst, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("\nwrote " + dst)
