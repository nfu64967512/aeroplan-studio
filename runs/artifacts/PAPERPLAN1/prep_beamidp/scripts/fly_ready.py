# -*- coding: utf-8 -*-
"""prep_beamidp step 5: is the new scenario fly-ready?

Reads back what `python scripts/hv_sitl_run.py plan beamidp_1a2b_1195` already
wrote to runs/artifacts/BEAMIDP1, checks the three things that silently break a
sortie (exported altitude vs the cap, the 34-parameter gate, the runner's own
"every aircraft has work" gate), and re-plans once to record the per-aircraft
strip order so the per-cell table's "Beam-IDP as planned" row can be audited for
cell contiguity.

NO SITL IS STARTED.  Planner + file reads only.
"""
from __future__ import annotations
import json, os, sys
import numpy as np

ROOT = r"C:/Users/joy46/Documents/Projects/aeroplan-studio"
sys.path.insert(0, ROOT); os.chdir(ROOT)
HERE = os.path.join(ROOT, "runs/artifacts/PAPERPLAN1/prep_beamidp")
OUT = os.path.join(ROOT, "runs/artifacts/BEAMIDP1")

from scripts.hv_sitl_scenarios import SCENARIOS                       # noqa: E402
from scripts.hv_sitl_run import (_plan_gui, expected_params,          # noqa: E402
                                 _param_file_values, EXPECT_FALLBACK,
                                 FIRMWARE_OVERRIDE)

SC = SCENARIOS["beamidp_1a2b_1195"]
CAP = float(SC.waiver_max_agl_m)


def waypoint_alts(path):
    """QGC WPL 110: idx cur frame cmd p1..p4 x y z autocont (tab separated)."""
    zs, frames = [], {}
    for ln in open(path, encoding="utf-8").read().splitlines()[1:]:
        f = ln.split("\t")
        if len(f) < 12:
            continue
        frame, cmd, z = int(f[2]), int(f[3]), float(f[10])
        frames[frame] = frames.get(frame, 0) + 1
        if frame == 3 and z > 1.0:            # GLOBAL_RELATIVE_ALT, skip ground items
            zs.append((z, cmd))
    return zs, frames


def main():
    rep = {"scenario": SC.key, "trial": SC.trial, "out_dir": OUT, "cap_m": CAP}
    rep["report"] = json.load(open(os.path.join(OUT, "report.json"), encoding="utf-8"))
    rep["validation"] = json.load(open(os.path.join(OUT, "validation.json"), encoding="utf-8"))
    rep["scenario_json"] = json.load(open(os.path.join(OUT, "scenario.json"), encoding="utf-8"))
    ms = json.load(open(os.path.join(OUT, "plan", "mission_summary.json"), encoding="utf-8"))
    rep["mission_summary_uavs"] = {u: {k: v for k, v in d.items() if k != "items"}
                                   for u, d in ms.get("uavs", {}).items()}

    # ---- 1. exported altitude vs the 119.5 m planning cap (C1's metric) ----
    alt = {}
    for u in ("A1", "B2", "B3"):
        zs, frames = waypoint_alts(os.path.join(OUT, "plan", "%s.waypoints" % u))
        zmax, cmd = max(zs) if zs else (0.0, 0)
        alt[u] = dict(n_items_rel=len(zs), max_rel_alt_m=round(zmax, 4),
                      excess_vs_cap_m=round(zmax - CAP, 4),
                      excess_vs_120_m=round(zmax - 120.0, 4),
                      at_cmd=cmd, frames=frames,
                      n_over_120=int(sum(1 for z, _ in zs if z > 120.0)))
    rep["exported_altitude"] = alt

    # ---- 2. the 34 x 3 parameter gate (runs with no vehicle) ----
    gate = {}
    for u in ("A1", "B2", "B3"):
        from_file = _param_file_values(SC, u)
        want = expected_params(from_file)
        gate[u] = dict(n_in_param_file=len(from_file), n_compared=len(want),
                       from_fallback=sorted(set(want) - set(from_file)),
                       overridden=sorted(set(want) & set(FIRMWARE_OVERRIDE)),
                       names=sorted(want))
    rep["param_gate"] = gate
    rep["param_gate_total"] = sum(g["n_compared"] for g in gate.values())
    rep["param_gate_note"] = ("expected_params() is the fly path's only decision point; it "
                              "raises SystemExit if an exported parameter escapes the "
                              "comparison set.  It ran clean here, so the 34 x 3 gate is intact.")

    # ---- 3. per-aircraft planned order, and per-cell contiguity ----
    hv, report = _plan_gui(SC)
    plan, ctx = hv.last_plan, hv.last_ctx
    got = []

    def _flat(x):
        if isinstance(x, dict):
            [_flat(v) for v in x.values()]
        elif isinstance(x, (list, tuple, set)):
            [_flat(v) for v in x]
        elif hasattr(x, "spacing_m") and hasattr(x, "cell_id"):
            got.append(x)
    _flat(getattr(plan, "strips", None) or getattr(ctx, "strips", None))
    cell_of = {s.strip_id: s.cell_id for s in got}
    tours = {}
    for u, p in sorted(plan.plans.items()):
        ids = [t[0] if isinstance(t, (list, tuple)) else str(t) for t in p.strips]
        cells = [cell_of.get(i, "?") for i in ids]
        runs = [cells[0]]
        for c in cells[1:]:
            if c != runs[-1]:
                runs.append(c)
        tours[u] = dict(cls=p.cls, n=len(ids), cells_in_visit_order=runs,
                        contiguous_per_cell=bool(len(runs) == len(set(cells))),
                        strip_ids=ids, cell_labels=cells,
                        t_takeoff_s=round(float(p.t_takeoff_s), 1),
                        t_land_s=round(float(p.t_land_s), 1),
                        distance_m=round(float(p.distance_m), 1),
                        energy_used_wh=round(float(p.energy_used_wh), 2),
                        energy_margin_wh=round(float(p.energy_margin_wh), 2),
                        takeoff_site=getattr(p, "takeoff_site", ""))
    rep["tours"] = tours
    rep["makespan_s"] = round(float(plan.makespan_s), 2)
    rep["all_aircraft_have_work"] = bool(len(plan.plans) == len(SC.fleet)
                                         and all(t["n"] > 0 for t in tours.values()))
    rep["fly_command"] = ("cd C:/Users/joy46/Documents/Projects/aeroplan-studio  &&  "
                          "python scripts/hv_sitl_run.py fly beamidp_1a2b_1195")

    json.dump(rep, open(os.path.join(HERE, "fly_ready.json"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)

    print("=" * 96)
    print("scenario %s  trial %s  cap %.1f m" % (SC.key, SC.trial, CAP))
    print("status %s  validate passed=%s conflicts=%s  min_sep_h=%.1f m  min_sep_v=%.1f m  uncovered=%s"
          % (rep["report"].get("status"), rep["validation"]["passed"],
             rep["validation"]["conflicts"], rep["validation"]["min_sep_h_m"],
             rep["validation"]["min_sep_v_m"], rep["report"].get("uncovered")))
    print("makespan %.1f s  (speedup 5 -> %.1f min wall clock)"
          % (rep["makespan_s"], rep["makespan_s"] / 5.0 / 60.0))
    print("all aircraft have work: %s" % rep["all_aircraft_have_work"])
    print("-" * 96)
    for u, d in alt.items():
        print("  %s exported max rel alt %.4f m  (cap %+.4f, vs 120 m %+.4f)  items>120 m: %d"
              % (u, d["max_rel_alt_m"], d["excess_vs_cap_m"], d["excess_vs_120_m"], d["n_over_120"]))
    print("-" * 96)
    for u, g in gate.items():
        print("  %s param file %d entries -> gate compares %d  (fallback %s, override %s)"
              % (u, g["n_in_param_file"], g["n_compared"], g["from_fallback"], g["overridden"]))
    print("  TOTAL gate items: %d" % rep["param_gate_total"])
    print("-" * 96)
    for u, t in tours.items():
        print("  %s (%s) %d strips  cells in visit order %s  contiguous=%s  "
              "t_off=%.0fs t_land=%.0fs  %.2f km  E=%.0f/%.0f Wh  site=%s"
              % (u, t["cls"], t["n"], t["cells_in_visit_order"], t["contiguous_per_cell"],
                 t["t_takeoff_s"], t["t_land_s"], t["distance_m"] / 1000.0,
                 t["energy_used_wh"], t["energy_margin_wh"], t["takeoff_site"]))
    print("-" * 96)
    print("FLY COMMAND (later, when the machine is free):")
    print("  " + rep["fly_command"])
    print("")
    print("wrote " + os.path.join(HERE, "fly_ready.json"))


if __name__ == "__main__":
    main()
