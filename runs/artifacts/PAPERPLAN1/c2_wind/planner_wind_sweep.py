"""C2 evidence: how much does the PLAN change when the wind assumption changes?

Read-only: plans the HV4SITL16 scenario (gui_legal_120_1a2b, 1 Pioneer + 2 HERO)
at W = 0 / 5 / 8 m/s from 270 deg and prints makespan, per-UAV distance/time,
R_eff per class and the planned minimum separation.  Nothing is exported and no
runs/artifacts/<TRIAL> directory is touched.

    python runs/artifacts/PAPERPLAN1/c2_wind/planner_wind_sweep.py
"""
from __future__ import annotations

import dataclasses
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))

from scripts.hv_sitl_run import _plan_gui                    # noqa: E402
from scripts.hv_sitl_scenarios import SCENARIOS              # noqa: E402

KEY = "gui_legal_120_1a2b"          # == trial HV4SITL16
def _arg(a):
    """'5' -> (5, 270);  '5@180' -> (5, 180)."""
    s, _, d = str(a).partition("@")
    return (float(s), float(d or 270.0))


WINDS = [_arg(x) for x in (sys.argv[1:] or [0, 5, 8])]


def one(sc, wind):
    import hv_dccpp as hvlib
    sc2 = dataclasses.replace(sc, wind=wind)
    t0 = time.perf_counter()
    hv, report = _plan_gui(sc2)
    dt = time.perf_counter() - t0
    plan, ctx, cfg = hv.last_plan, hv.last_ctx, hv.cfg
    cache = hvlib.build_validation_cache(plan, ctx, budget_only=False)
    v = hvlib.validate(plan, ctx, cache, cfg)
    w = float(wind[0])
    return {
        "wind_mps": w, "wind_dir_from_deg": float(wind[1]),
        "plan_wall_s": round(dt, 2),
        "status": report.get("status"),
        "makespan_s": round(plan.makespan_s, 1),
        "total_distance_km": round(sum(p.distance_m for p in plan.plans.values()) / 1000.0, 3),
        "per_uav": {u: {"cls": p.cls, "strips": len(p.strips), "wp": len(p.waypoints),
                        "distance_km": round(p.distance_m / 1000.0, 3),
                        "t_takeoff_s": round(p.t_takeoff_s, 1),
                        "t_land_s": round(p.t_land_s, 1),
                        "flight_time_s": round(p.t_land_s - p.t_takeoff_s, 1),
                        "energy_used_wh": round(p.energy_used_wh, 1),
                        "energy_margin_wh": round(p.energy_margin_wh, 1)}
                    for u, p in sorted(plan.plans.items())},
        "r_eff_m": {"A": round(hv.platform_a.r_eff(w), 2),
                    "B": round(hv.platform_b.r_eff(w), 2)},
        "r_plan_m": {"A": round(hv.platform_a.r_eff(0.0), 2),
                     "B": round(hv.platform_b.r_eff(0.0), 2)},
        "validate": {"passed": v.passed, "conflicts": len(v.conflicts),
                     "min_sep_h_m": round(float(v.min_sep_h_m), 2),
                     "min_sep_v_m": (None if v.min_sep_v_m != v.min_sep_v_m
                                     or v.min_sep_v_m == float("inf")
                                     else round(float(v.min_sep_v_m), 2)),
                     "coverage_ok": v.coverage_ok, "energy_ok": v.energy_ok},
    }


if __name__ == "__main__":
    sc = SCENARIOS[KEY]
    out = {"scenario": KEY, "trial_of_record": sc.trial,
           "scenario_default_wind": list(sc.wind), "rows": []}
    for w in WINDS:
        try:
            out["rows"].append(one(sc, w))
        except SystemExit as e:
            out["rows"].append({"wind_mps": w[0], "error": f"SystemExit: {e}"})
        except Exception as e:                                   # noqa: BLE001
            out["rows"].append({"wind_mps": w[0], "error": f"{type(e).__name__}: {e}"})
    print(json.dumps(out, ensure_ascii=False, indent=1))
