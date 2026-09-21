"""prep_text probe: does the planned strip HEADING stay east-west when the plan wind
direction changes?  Pure Python planner run (no SITL).  Writes only into this folder.

Why it matters: the C2 direction control ("5 m/s from the south") is only a crosswind
test if the STRIPS stay east-west.  If the planner re-orients the strips to the new
wind, the flown aircraft sees along-track wind again and the control tests nothing.
"""
from __future__ import annotations
import json, math, os, sys, time
from dataclasses import replace

ROOT = r"C:/Users/joy46/Documents/Projects/aeroplan-studio"
sys.path.insert(0, ROOT)
os.chdir(ROOT)

from scripts.hv_sitl_scenarios import SCENARIOS                      # noqa: E402
from scripts.hv_sitl_run import _plan_gui                            # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
KEY = "gui_legal_120_1a2b"


def strip_headings(plan) -> dict:
    """Heading of every long sensor-on (OPERATE) leg, from the planner's own waypoints."""
    out = {}
    for u, p in plan.plans.items():
        hs = []
        wps = list(getattr(p, "waypoints", []) or [])
        for a, b in zip(wps, wps[1:]):
            if not (bool(a.sensor_on) and bool(b.sensor_on)):
                continue
            if "OPERATE" not in str(a.phase).upper():
                continue
            dx = float(b.x_m) - float(a.x_m)
            dy = float(b.y_m) - float(a.y_m)
            L = math.hypot(dx, dy)
            if L < 200.0:                    # long scan legs only, not turn chords
                continue
            th = math.degrees(math.atan2(dx, dy)) % 180.0   # 90 = east-west, 0 = north-south
            hs.append((round(th, 2), round(L, 1)))
        if hs:
            tot = sum(L for _, L in hs)
            out[u] = {"n_legs": len(hs),
                      "total_len_m": round(tot, 1),
                      "len_weighted_heading_mod180_deg": round(sum(t * L for t, L in hs) / tot, 2),
                      "headings": sorted({t for t, _ in hs})}
        else:
            out[u] = {"n_legs": 0}
    return out


rows = []
for w in [(5.0, 270.0), (5.0, 180.0), (0.0, 270.0)]:
    sc = replace(SCENARIOS[KEY], wind=w)
    t0 = time.perf_counter()
    try:
        hv, report = _plan_gui(sc)
    except Exception as e:                                   # noqa: BLE001
        rows.append({"wind": w, "error": f"{type(e).__name__}: {e}"})
        continue
    plan = hv.last_plan
    rows.append({
        "wind_mps_dir_from_deg": list(w),
        "plan_wall_s": round(time.perf_counter() - t0, 2),
        "status": report.get("status"),
        "makespan_s": round(float(plan.makespan_s), 2),
        "total_km": round(sum(p.distance_m for p in plan.plans.values()) / 1000.0, 3),
        "per_uav_km": {u: round(p.distance_m / 1000.0, 3) for u, p in plan.plans.items()},
        "operate_leg_headings": strip_headings(plan),
    })
    print(json.dumps(rows[-1], ensure_ascii=False)[:600], flush=True)

with open(os.path.join(HERE, "strip_heading_vs_wind_dir.json"), "w", encoding="utf-8") as f:
    json.dump(rows, f, ensure_ascii=False, indent=1)
print("wrote strip_heading_vs_wind_dir.json")
