"""C3 probe 5: mixed 1 Pioneer + 2 HERO fleet on a geometry big enough that the
Pioneer cell also exceeds k=5. Per aircraft, compare the tour HARVEST actually
planned over its own strips against the plain geometric (sequential) tour over
the same strips, with the planner's own Dubins-in-wind cost model.

Pure Python, no SITL.
"""
from __future__ import annotations
import json, math, os, sys, time
import numpy as np

ROOT = r"C:/Users/joy46/Documents/Projects/aeroplan-studio"
sys.path.insert(0, ROOT)
os.chdir(ROOT)

from scripts.hv_sitl_scenarios import Scenario, rect                    # noqa: E402
from scripts.hv_sitl_run import _plan_gui                               # noqa: E402
from hv_dccpp.dubins import dubins_all_times, skip_k                    # noqa: E402
from hv_dccpp.decompose import (r_eff_for_class, _skip_sequence,        # noqa: E402
                                _best_sequence, _held_karp_path)
from hv_dccpp.mission import PlannerConfig                              # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
AIRSPACE = dict(coverage_altitude_m=120.0, coverage_width_m=150.0, overlap_rate=0.30,
                waiver_max_agl_m=None, min_fw_agl_m={"A": 100.0, "B": 100.0},
                dh_band_min_m=15.0, target_gsd=0.05, spawn_spread_m=10.0)


def mk(key, n_a, n_b, half_e, half_n):
    return Scenario(key=key, trial="PAPERPLAN1_C3", path="gui", n_a=n_a, n_b=n_b,
                    areas=[{"area_id": 1, "priority": 1.0,
                            "polygon": rect(0, 0, half_e, half_n)}],
                    note="C3 probe", **AIRSPACE)


def matrices(strips, r_eff, v_air, wind_xy, dth):
    n = len(strips)
    S, E, W, slots = [], [], [], []
    for a in range(n):
        for b in range(n):
            if a == b:
                continue
            sa, sb = strips[a], strips[b]
            slots.append((a, b, len(S)))
            S.extend([sa.end_state(-1).as_array(), sa.end_state(+1).as_array()])
            E.extend([sb.start_state(+1).as_array(), sb.start_state(-1).as_array()])
            W.extend([0.5, 0.5])
    t, l, _ = dubins_all_times(np.array(S), np.array(E), r_eff, v_air, wind_xy, dth)
    tw = np.asarray(t) * np.asarray(W)
    lw = np.asarray(l) * np.asarray(W)
    Ct = np.full((n, n), math.inf)
    Cl = np.full((n, n), math.inf)
    for a, b, i0 in slots:
        Ct[a, b] = float(tw[i0:i0 + 2].sum())
        Cl[a, b] = float(lw[i0:i0 + 2].sum())
    return Ct, Cl


def chain(C, seq):
    return float(sum(C[a, b] for a, b in zip(seq[:-1], seq[1:])))


def run(key, n_a, n_b, he, hn):
    sc = mk(key, n_a, n_b, he, hn)
    t0 = time.perf_counter()
    hv, report = _plan_gui(sc)
    dt = time.perf_counter() - t0
    plan, ctx, cfg = hv.last_plan, hv.last_ctx, hv.cfg
    seqreg = []
    reg = getattr(plan, "strips", None) or getattr(ctx, "strips", None)

    def _flat(x):
        if isinstance(x, dict):
            for v in x.values():
                _flat(v)
        elif isinstance(x, (list, tuple, set)):
            for v in x:
                _flat(v)
        elif hasattr(x, "spacing_m") and hasattr(x, "cell_id"):
            seqreg.append(x)
    _flat(reg)
    by_id = {s.strip_id: s for s in seqreg}
    dth = math.radians(cfg.dtheta_int_deg)
    out = {"key": key, "rect_m": [2 * he, 2 * hn], "area_km2": round(4 * he * hn / 1e6, 3),
           "plan_s": round(dt, 1), "status": report.get("status"),
           "makespan_s": round(float(plan.makespan_s), 1),
           "uncovered": len(plan.uncovered), "per_uav": {}, "per_cell": {}}

    # ---- per aircraft: planner tour vs geometric-sequential tour ----
    for u, p in sorted(plan.plans.items()):
        cls = p.cls
        planned_ids = [t[0] if isinstance(t, (list, tuple)) else str(t) for t in p.strips]
        strips = [by_id[i] for i in planned_ids if i in by_id]
        if len(strips) < 3:
            out["per_uav"][u] = {"cls": cls, "n": len(strips),
                                 "note": "fewer than 3 strips: no ordering freedom"}
            continue
        geo = sorted(strips, key=lambda s: float(s.v_lo_m))
        ids = [s.strip_id for s in geo]
        pos = {sid: i for i, sid in enumerate(ids)}
        r_eff = float(r_eff_for_class(ctx, cls))
        spec = ctx.specs[cls]
        z_op = float(np.mean([s.z_op_m for s in geo]))
        wind_xy = np.asarray(ctx.wind.wind_vec(z_op), dtype=float).reshape(-1, 2)[0]
        Ct, Cl = matrices(geo, r_eff, spec.cruise_airspeed_mps, wind_xy, dth)
        n = len(geo)
        s_mean = float(np.mean([s.spacing_m for s in geo]))
        k = skip_k(s_mean, r_eff)
        orders = {"sequential": list(range(n)),
                  "skip_k_fixed": _skip_sequence(n, k),
                  "harvest_as_planned": [pos[i] for i in planned_ids if i in pos]}
        hk = _held_karp_path(Ct) if n <= 14 else None
        if hk:
            orders["held_karp_opt"] = hk
        bt, bl = chain(Ct, orders["sequential"]), chain(Cl, orders["sequential"])
        rows = {}
        model = (getattr(ctx, "models_rep", {}) or {}).get(cls)
        p_w = float(model.p_cruise_w() * model.k_unc) if model is not None else float("nan")
        for nm, sq in orders.items():
            t_, l_ = chain(Ct, sq), chain(Cl, sq)
            rows[nm] = {"order": [int(i) for i in sq], "turn_time_s": round(t_, 2),
                        "turn_dist_m": round(l_, 1),
                        "d_time_s": round(t_ - bt, 2),
                        "d_time_pct": round(100 * (t_ - bt) / bt, 2) if bt else 0,
                        "d_dist_m": round(l_ - bl, 1),
                        "d_dist_pct": round(100 * (l_ - bl) / bl, 2) if bl else 0,
                        "d_energy_wh": (round(p_w * (t_ - bt) / 3600, 2) if p_w == p_w else None)}
        out["per_uav"][u] = {"cls": cls, "n": n, "spacing_m": round(s_mean, 2),
                             "R_eff_m": round(r_eff, 2), "k": k,
                             "strip_len_m": round(float(np.mean([s.length_m for s in geo])), 1),
                             "cells": sorted({s.cell_id for s in geo}),
                             "planned_distance_m": round(float(p.distance_m), 1),
                             "planned_energy_wh": round(float(p.energy_used_wh), 2),
                             "t_takeoff_s": round(float(p.t_takeoff_s), 1),
                             "t_land_s": round(float(p.t_land_s), 1),
                             "orders": rows}

    # ---- per cell trigger table ----
    groups = {}
    for s in seqreg:
        groups.setdefault((s.cell_id, s.cls), []).append(s)
    for (cid, cls), lst in sorted(groups.items()):
        s_mean = float(np.mean([x.spacing_m for x in lst]))
        R = float(r_eff_for_class(ctx, cls))
        k = skip_k(s_mean, R)
        out["per_cell"]["%s|%s" % (cid, cls)] = {
            "n": len(lst), "s": round(s_mean, 2), "R_eff": R, "k": k,
            "triggers": len(lst) > k}
    return out


def main():
    cases = [("big_3000x3000_1a2b", 1, 2, 1500, 1500)]
    res = []
    for c in cases:
        try:
            r = run(*c)
        except BaseException as e:                       # noqa: BLE001
            r = {"key": c[0], "error": "%s: %s" % (type(e).__name__, e)}
        res.append(r)
        print("")
        print("=== %s  %s  %.2f km2  plan %ss  status=%s  makespan=%s s  uncovered=%s" % (
            r.get("key"), r.get("rect_m"), r.get("area_km2", 0), r.get("plan_s"),
            r.get("status"), r.get("makespan_s"), r.get("uncovered")))
        for cid, c2 in (r.get("per_cell") or {}).items():
            print("    cell %-22s n=%-3s k=%s trigger=%s" % (cid, c2["n"], c2["k"], c2["triggers"]))
        for u, d in (r.get("per_uav") or {}).items():
            if "orders" not in d:
                print("    %s (%s) %s" % (u, d.get("cls"), d.get("note")))
                continue
            print("    %s cls=%s n=%s k=%s cells=%s  planned %sm %sWh" % (
                u, d["cls"], d["n"], d["k"], d["cells"],
                d["planned_distance_m"], d["planned_energy_wh"]))
            for nm, v in d["orders"].items():
                print("        %-20s t=%8.1fs d=%9.1fm  dt=%7.2f%% dd=%7.2f%% dE=%s Wh  order=%s" % (
                    nm, v["turn_time_s"], v["turn_dist_m"], v["d_time_pct"],
                    v["d_dist_pct"], v["d_energy_wh"], v["order"]))
        sys.stdout.flush()
    dst = os.path.join(HERE, "real_order_mixed.json")
    json.dump(res, open(dst, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("")
    print("wrote " + dst)


if __name__ == "__main__":
    main()
