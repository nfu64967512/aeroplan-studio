"""C3 probe 2: run the REAL HARVEST planner (pure Python, no SITL) and report,
per cell and per class, how many strips exist, the spacing, R_eff and skip_k.

Reuses scripts/hv_sitl_run._plan_gui so the pipeline is bit-identical to the
one that produced runs/artifacts/HV4SITL15. Writes NOTHING outside this folder.
"""
from __future__ import annotations
import json, math, os, sys, time
from dataclasses import replace

ROOT = r"C:/Users/joy46/Documents/Projects/aeroplan-studio"
sys.path.insert(0, ROOT)
os.chdir(ROOT)

from scripts.hv_sitl_scenarios import SCENARIOS, Scenario, rect          # noqa: E402
from scripts.hv_sitl_run import _plan_gui                                 # noqa: E402
from hv_dccpp.dubins import skip_k                                        # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def describe(key: str, sc: Scenario) -> dict:
    t0 = time.perf_counter()
    try:
        hv, report = _plan_gui(sc)
    except SystemExit as e:
        return {"key": key, "error": f"SystemExit: {e}"}
    except Exception as e:                                   # noqa: BLE001
        return {"key": key, "error": f"{type(e).__name__}: {e}"}
    dt = time.perf_counter() - t0
    plan, ctx, cfg = hv.last_plan, hv.last_ctx, hv.cfg
    out = {"key": key, "plan_s": round(dt, 2), "status": report.get("status"),
           "makespan_s": round(float(plan.makespan_s), 1),
           "uncovered": len(plan.uncovered), "cells": {}, "uavs": {}}
    # R_eff per class as the planner sees it
    reff = {}
    try:
        from hv_dccpp.decompose import r_eff_for_class
        for cls in ("A", "B"):
            try:
                reff[cls] = round(float(r_eff_for_class(ctx, cls)), 2)
            except Exception:                                # noqa: BLE001
                pass
    except Exception:                                        # noqa: BLE001
        pass
    out["R_eff_m"] = reff
    # strips grouped by (cell_id, cls)
    groups: dict[tuple[str, str], list] = {}
    strips_all = []
    for cls, d in getattr(ctx, "strips", {}).items() if isinstance(getattr(ctx, "strips", None), dict) else []:
        pass
    # ctx may not expose strips; go via plan
    for u, p in plan.plans.items():
        out["uavs"][u] = {"cls": p.cls, "n_strips": len(p.strips),
                          "strip_ids": [s[0] if isinstance(s, (list, tuple)) else str(s) for s in p.strips],
                          "distance_km": round(p.distance_m / 1000.0, 3),
                          "t_takeoff_s": round(float(p.t_takeoff_s), 1),
                          "t_land_s": round(float(p.t_land_s), 1)}
    # try to reach the strip objects
    cand = None
    for attr in ("strips", "all_strips"):
        v = getattr(plan, attr, None) or getattr(ctx, attr, None)
        if v:
            cand = v
            break
    if cand is None:
        # last resort: rebuild from cells
        cells = getattr(ctx, "cells", None) or getattr(plan, "cells", None)
        out["_note"] = f"no strip registry found; ctx attrs={sorted(a for a in dir(ctx) if not a.startswith('_'))[:60]}"
        return out
    seq = cand.values() if isinstance(cand, dict) else cand
    flat = []
    for x in seq:
        if isinstance(x, dict):
            flat.extend(x.values())
        elif isinstance(x, (list, tuple)):
            flat.extend(x)
        else:
            flat.append(x)
    for st in flat:
        key2 = (getattr(st, "cell_id", "?"), getattr(st, "cls", "?"))
        groups.setdefault(key2, []).append(st)
    for (cid, cls), lst in sorted(groups.items()):
        s_mean = sum(float(x.spacing_m) for x in lst) / len(lst)
        R = reff.get(cls, float("nan"))
        out["cells"][f"{cid}|{cls}"] = {
            "n_strips": len(lst),
            "spacing_m": round(s_mean, 2),
            "length_m": round(sum(float(x.length_m) for x in lst) / len(lst), 1),
            "width_m": round(sum(float(x.width_m) for x in lst) / len(lst), 1),
            "h_op_m": round(sum(float(x.h_op_m) for x in lst) / len(lst), 1),
            "R_eff_m": R,
            "k": (skip_k(s_mean, R) if (s_mean > 0 and R == R) else None),
            "skip_k_differs_from_sequential": bool(
                s_mean > 0 and R == R and len(lst) > max(1, skip_k(s_mean, R))),
        }
    return out


def main():
    keys = sys.argv[1:] or ["gui_legal_120_1a2b"]
    res = []
    for k in keys:
        sc = SCENARIOS[k]
        res.append(describe(k, sc))
    dst = os.path.join(HERE, "plan_strips_%s.json" % "_".join(keys)[:40])
    with open(dst, "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=1)
    print(json.dumps(res, ensure_ascii=False, indent=1))
    print("wrote", dst)


if __name__ == "__main__":
    main()
