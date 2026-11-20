"""C3 probe 4: on a geometry that REALLY triggers skip-k (n=9 strips in one cell,
Pioneer k=5 / Hero k=3), compare the order HARVEST actually planned against a
plain sequential (boustrophedon) scan, using the planner's own Dubins-in-wind
cost model + its own energy model.

Outputs turning TIME, turning DISTANCE and ENERGY for:
  sequential | fixed skip-k | best-search | Held-Karp optimum | HARVEST as planned
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
                dh_band_min_m=15.0, target_gsd=0.05, spawn_spread_m=0.0)


def mk(key, n_a, n_b, half_e, half_n):
    return Scenario(key=key, trial="PAPERPLAN1_C3", path="gui", n_a=n_a, n_b=n_b,
                    areas=[{"area_id": 1, "priority": 1.0,
                            "polygon": rect(0, 0, half_e, half_n)}],
                    note="C3 probe", **AIRSPACE)


def pair_poses(sa, sb):
    return ([sa.end_state(-1).as_array(), sa.end_state(+1).as_array()],
            [sb.start_state(+1).as_array(), sb.start_state(-1).as_array()],
            [0.5, 0.5])


def matrices(strips, r_eff, v_air, wind_xy, dth):
    n = len(strips)
    S, E, W, slots = [], [], [], []
    for a in range(n):
        for b in range(n):
            if a == b:
                continue
            st, en, w = pair_poses(strips[a], strips[b])
            slots.append((a, b, len(S)))
            S.extend(st)
            E.extend(en)
            W.extend(w)
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


def analyse(key, n_a, n_b, he, hn):
    sc = mk(key, n_a, n_b, he, hn)
    t0 = time.perf_counter()
    hv, report = _plan_gui(sc)
    dt = time.perf_counter() - t0
    plan, ctx, cfg = hv.last_plan, hv.last_ctx, hv.cfg
    cls = "A" if n_a else "B"
    uav = next(iter(plan.plans))
    up = plan.plans[uav]
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
    strips = [s for s in seqreg if s.cls == cls]
    strips.sort(key=lambda s: float(s.v_lo_m))          # geometric (sequential) order
    ids = [s.strip_id for s in strips]
    n = len(strips)
    r_eff = float(r_eff_for_class(ctx, cls))
    spec = ctx.specs[cls]
    z_op = float(np.mean([s.z_op_m for s in strips]))
    wind_xy = np.asarray(ctx.wind.wind_vec(z_op), dtype=float).reshape(-1, 2)[0]
    s_mean = float(np.mean([s.spacing_m for s in strips]))
    k = skip_k(s_mean, r_eff)
    dth = math.radians(cfg.dtheta_int_deg)
    Ct, Cl = matrices(strips, r_eff, spec.cruise_airspeed_mps, wind_xy, dth)

    planned_ids = [t[0] if isinstance(t, (list, tuple)) else str(t) for t in up.strips]
    pos = {sid: i for i, sid in enumerate(ids)}
    planned = [pos[s] for s in planned_ids if s in pos]

    orders = {"sequential": list(range(n)),
              "skip_k_fixed": _skip_sequence(n, k),
              "best_search": _best_sequence(n, k, Ct, PlannerConfig(sweep_sequence="search")),
              "harvest_as_planned": planned}
    hk = _held_karp_path(Ct) if n <= 14 else None
    if hk:
        orders["held_karp_opt"] = hk

    model = (getattr(ctx, "models_rep", {}) or {}).get(cls)
    p_w = float(model.p_cruise_w() * model.k_unc) if model is not None else float("nan")

    base_t = chain(Ct, orders["sequential"])
    base_l = chain(Cl, orders["sequential"])
    rows = {}
    for nm, sq in orders.items():
        if len(sq) != n:
            rows[nm] = {"error": "order len %d != %d" % (len(sq), n), "order": list(sq)}
            continue
        t_ = chain(Ct, sq)
        l_ = chain(Cl, sq)
        rows[nm] = {"order": [int(i) for i in sq],
                    "turn_time_s": round(t_, 2), "turn_dist_m": round(l_, 1),
                    "turn_energy_wh": (round(p_w * t_ / 3600.0, 2) if p_w == p_w else None),
                    "d_time_s": round(t_ - base_t, 2),
                    "d_time_pct": round(100 * (t_ - base_t) / base_t, 2) if base_t else 0,
                    "d_dist_m": round(l_ - base_l, 1),
                    "d_dist_pct": round(100 * (l_ - base_l) / base_l, 2) if base_l else 0,
                    "d_energy_wh": (round(p_w * (t_ - base_t) / 3600.0, 2) if p_w == p_w else None)}
    return {"key": key, "cls": cls, "plan_s": round(dt, 1), "status": report.get("status"),
            "makespan_s": round(float(plan.makespan_s), 1),
            "uav": uav, "planned_distance_m": round(float(up.distance_m), 1),
            "planned_energy_wh": round(float(up.energy_used_wh), 2),
            "n_strips": n, "spacing_m": round(s_mean, 2), "R_eff_m": round(r_eff, 2),
            "k": k, "strip_len_m": round(float(np.mean([s.length_m for s in strips])), 1),
            "Va_mps": spec.cruise_airspeed_mps,
            "wind_xy_mps": [round(float(wind_xy[0]), 3), round(float(wind_xy[1]), 3)],
            "p_cruise_w_x_kunc": round(p_w, 1) if p_w == p_w else None,
            "strip_ids_geometric": ids, "planned_strip_ids": planned_ids,
            "orders": rows}


def main():
    cases = [("solo_A_1560x800", 1, 0, 780, 400),
             ("solo_B_1560x800", 0, 1, 780, 400),
             ("solo_A_1560x1600", 1, 0, 780, 800),
             ("solo_B_1560x1600", 0, 1, 780, 800)]
    out = []
    for c in cases:
        try:
            r = analyse(*c)
        except BaseException as e:                              # noqa: BLE001
            r = {"key": c[0], "error": "%s: %s" % (type(e).__name__, e)}
        out.append(r)
        print("")
        print("=== %s cls=%s n=%s s=%s R_eff=%s k=%s makespan=%s err=%s" % (
            r.get("key"), r.get("cls"), r.get("n_strips"), r.get("spacing_m"),
            r.get("R_eff_m"), r.get("k"), r.get("makespan_s"), r.get("error", "")))
        for nm, v in (r.get("orders") or {}).items():
            if "error" in v:
                print("   %-20s %s" % (nm, v["error"]))
                continue
            print("   %-20s t=%8.1fs  d=%9.1fm  E=%7s Wh   dt=%7.2f%%  dd=%7.2f%%   order=%s" % (
                nm, v["turn_time_s"], v["turn_dist_m"], v["turn_energy_wh"],
                v["d_time_pct"], v["d_dist_pct"], v["order"]))
        sys.stdout.flush()
    dst = os.path.join(HERE, "real_order.json")
    json.dump(out, open(dst, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("")
    print("wrote " + dst)


if __name__ == "__main__":
    main()
