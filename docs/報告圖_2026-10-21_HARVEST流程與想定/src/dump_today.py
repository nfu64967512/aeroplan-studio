"""Re-plan gui_legal_120_1a2b with today's code (ceiling 119.5 m, same as HV4W55) and dump the 4-D plan."""
import dataclasses
import json
import os
import sys
from pathlib import Path

ROOT = Path(sys.argv[1]).resolve()
OUT = Path(sys.argv[2])
os.environ.setdefault("AEROPLAN_NO_FILE_LOG", "1")
sys.path.insert(0, str(ROOT))
os.chdir(ROOT)

import hv_dccpp as hvlib  # noqa: E402
from scripts.hv_sitl_run import _plan_gui  # noqa: E402
from scripts.hv_sitl_scenarios import SCENARIOS  # noqa: E402

sc = dataclasses.replace(SCENARIOS["gui_legal_120_1a2b"], waiver_max_agl_m=119.5, coverage_altitude_m=119.5)
hv, report = _plan_gui(sc, None)
plan, ctx, cfg = hv.last_plan, hv.last_ctx, hv.cfg
cache = hvlib.build_validation_cache(plan, ctx, budget_only=False)
v = hvlib.validate(plan, ctx, cache, cfg)

out = {
    "makespan_s": plan.makespan_s,
    "validate": {"passed": v.passed, "conflicts": len(v.conflicts), "min_sep_h_m": v.min_sep_h_m,
                 "coverage_ok": v.coverage_ok, "energy_ok": v.energy_ok},
    "uncovered": [list(u) for u in plan.uncovered],
    "notes": [str(n) for n in (report.get("notes") or [])],
    "warnings": [str(w) for w in (report.get("warnings") or [])],
    "sites": {sid: [float(xy[0]), float(xy[1]), float(z)] for sid, (xy, z) in
              ((k, (val[0], val[1])) for k, val in ctx.sites_enu.items())},
    "strips": {sid: {"cell": s.cell_id, "z": float(s.z_op_m),
                     "p0": [float(c) for c in s.start_state(1)[:2]] if False else None}
               for sid, s in plan.strips.items()},
    "uavs": {},
}
for u, p in sorted(plan.plans.items()):
    out["uavs"][u] = {
        "cls": p.cls, "t_takeoff_s": p.t_takeoff_s, "t_arr_ptc_s": p.t_arr_ptc_s, "t_land_s": p.t_land_s,
        "takeoff_site": p.takeoff_site, "landing_site": p.landing_site, "distance_m": p.distance_m,
        "energy_used_wh": p.energy_used_wh, "energy_margin_wh": p.energy_margin_wh,
        "strips": [[s, int(d)] for s, d in p.strips], "orbits": p.orbits_inserted,
        "wp": [[round(w.x_m, 2), round(w.y_m, 2), round(w.z_m, 2), round(w.agl_m, 2), round(w.t_s, 2),
                w.phase.name, w.mode.name, bool(w.sensor_on), w.strip_id, round(w.airspeed_mps, 2)]
               for w in p.waypoints],
    }
OUT.write_text(json.dumps(out, ensure_ascii=False, default=str), encoding="utf-8")
print("makespan", round(plan.makespan_s, 1), "validate", out["validate"])
print("sites", {k: [round(c, 1) for c in val] for k, val in out["sites"].items()})
for u, d in out["uavs"].items():
    ph = []
    for w in d["wp"]:
        if not ph or ph[-1][0] != w[5]:
            ph.append([w[5], w[4], w[4], w[2], w[2]])
        else:
            ph[-1][2] = w[4]; ph[-1][4] = w[2]
    print(u, d["cls"], d["takeoff_site"], "->", d["landing_site"], "t", round(d["t_takeoff_s"]), round(d["t_land_s"], 1),
          "km", round(d["distance_m"] / 1000, 2), "strips", d["strips"])
    print("   ", [(a, round(b), round(c), round(z0), round(z1)) for a, b, c, z0, z1 in ph])
