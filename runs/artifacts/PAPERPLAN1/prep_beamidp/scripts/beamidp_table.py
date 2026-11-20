# -*- coding: utf-8 -*-
"""prep_beamidp step 2+3+4: the SEQUENTIAL-SCAN BASELINE DRIVER and the per-cell
Beam-IDP comparison table.

WHY A DRIVER IS NEEDED
----------------------
No PlannerConfig produces a sequential (boustrophedon) strip order: the K-nearest
filter in hv_dccpp/planner.py:604-605 ranks candidates by `t_add` (transition +
operate TIME), so K=1 only makes the search greedier about SKIPPING, and
`legacy_mode` merely sets the ESTIMATOR's sweep_sequence="fixed".  So the
sequential arm has to be evaluated, not planned.

THE ONE RULER
-------------
This driver re-implements hv_dccpp/decompose.py exact_time() verbatim (the
planner's own per-cell cost) and then forces the strip order instead of letting
_best_sequence pick it.  Everything else is read back from the planner run:

  HELD FIXED (identical for every arm)
    * the strip set                     -- exactly the Strip objects the planner built
    * R_eff                             -- decompose.r_eff_for_class(ctx, cls, z_top_m=z_op)
    * wind                              -- ctx.wind.wind_vec(z_op), the same vector
    * airspeed                          -- ctx.specs[cls].cruise_airspeed_mps
    * Dubins discretisation             -- cfg.dtheta_int_rad
    * entry/exit endpoint convention    -- Strip.end_state(-1|+1) -> Strip.start_state(+1|-1)
                                           with the planner's own 0.5/0.5 direction
                                           average (1.0 for one-way-feasible strips)
    * operate time t_oper               -- decompose._op_times(); order-independent,
                                           so it cancels in every delta but is shown
    * energy                            -- EnergyModel.transfer_wh(t, dz=0), i.e. the
                                           planner's own k_unc*P_cruise*t

  CANNOT BE CONTROLLED (stated, not hidden)
    * inter-cell transit, takeoff/entry and RTB legs are re-scheduled by the planner;
      a truly flown sequential arm would get different ones, so the fleet makespan
      delta printed at the bottom is an ESTIMATE (turn-time delta only).
    * 4-D deconfliction / corridor queueing may insert holds that this cost model
      does not see.
    * the direction each strip is actually flown: the 0.5/0.5 average is the
      planner's estimator convention, so the "Beam-IDP as planned" row is scored
      under the estimator's ruler, not under its own committed directions.
    * takeoff/landing/climb energy: excluded -- only the in-cell segments are compared.

Pure Python.  No SITL.  Writes only under runs/artifacts/PAPERPLAN1/prep_beamidp/.
"""
from __future__ import annotations
import dataclasses, json, math, os, sys, time
import numpy as np

ROOT = r"C:/Users/joy46/Documents/Projects/aeroplan-studio"
sys.path.insert(0, ROOT); os.chdir(ROOT)
HERE = os.path.join(ROOT, "runs/artifacts/PAPERPLAN1/prep_beamidp")

from scripts.hv_sitl_scenarios import SCENARIOS                     # noqa: E402
from scripts.hv_sitl_run import _plan_gui                           # noqa: E402
from hv_dccpp.dubins import dubins_all_times, skip_k                # noqa: E402
from hv_dccpp.decompose import (r_eff_for_class, _skip_sequence,    # noqa: E402
                                _held_karp_path, _op_times,
                                _req_of_strips, resolve_sensor)


# ---- the planner's own per-cell cost, with the order forced ----------------
def cell_ruler(strips, cls, ctx):
    """Mirror of decompose.exact_time() up to where it picks a sequence.

    Returns (t_oper_s, Ct, Cl, meta); Ct/Cl are n x n transition time/length
    matrices under the planner's own endpoint convention.
    """
    spec, cfg = ctx.specs[cls], ctx.cfg
    found = resolve_sensor(ctx, cls, _req_of_strips(ctx, cls, strips))
    sensor = found[0] if found is not None else None
    z_op = float(np.mean([s.z_op_m for s in strips]))
    wind_xy = np.asarray(ctx.wind.wind_vec(z_op), dtype=float).reshape(-1, 2)[0]
    op = _op_times(strips, spec, sensor, wind_xy, cfg)
    both = np.isfinite(op).all(axis=1)
    t_oper = float(np.where(both, op.mean(axis=1),
                            np.nanmin(np.where(np.isfinite(op), op, np.nan), axis=1)).sum())
    n = len(strips)
    r_eff = r_eff_for_class(ctx, cls, z_top_m=z_op)
    s_mean = float(np.mean([s.spacing_m for s in strips]))
    k = skip_k(s_mean, r_eff)
    meta = dict(n=n, k=k, spacing_m=s_mean, r_eff_m=float(r_eff), z_op_m=z_op,
                strip_len_m=float(np.mean([s.length_m for s in strips])),
                wind_xy=[float(wind_xy[0]), float(wind_xy[1])],
                v_air_mps=float(spec.cruise_airspeed_mps),
                one_way=int((~both).sum()))
    if n < 2:
        return t_oper, np.zeros((n, n)), np.zeros((n, n)), meta

    def pair(a, b):
        sa, sb = strips[a], strips[b]
        if both[a] and both[b]:
            return ([sa.end_state(-1).as_array(), sa.end_state(+1).as_array()],
                    [sb.start_state(+1).as_array(), sb.start_state(-1).as_array()], [0.5, 0.5])
        d = -1 if (np.isfinite(op[a, 0]) and np.isfinite(op[b, 0])) else 1
        return ([sa.end_state(d).as_array()], [sb.start_state(d).as_array()], [1.0])

    S, E, W, slots = [], [], [], []
    for a in range(n):
        for b in range(n):
            if a == b:
                continue
            st, en, w = pair(a, b)
            slots.append((a, b, len(S), len(w)))
            S.extend(st); E.extend(en); W.extend(w)
    t, l, _ = dubins_all_times(np.array(S), np.array(E), r_eff,
                               spec.cruise_airspeed_mps, wind_xy, cfg.dtheta_int_rad)
    tw = np.asarray(t, float) * np.asarray(W, float)
    lw = np.asarray(l, float) * np.asarray(W, float)
    Ct = np.full((n, n), math.inf)
    Cl = np.full((n, n), math.inf)
    for a, b, i0, nw in slots:
        Ct[a, b] = float(tw[i0:i0 + nw].sum())
        Cl[a, b] = float(lw[i0:i0 + nw].sum())
    return t_oper, Ct, Cl, meta


def chain(C, seq):
    return float(sum(C[a, b] for a, b in zip(seq[:-1], seq[1:]))) if len(seq) > 1 else 0.0


def collect_strips(plan, ctx):
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


def analyse(sc, label):
    t0 = time.perf_counter()
    hv, report = _plan_gui(sc)
    dt = time.perf_counter() - t0
    plan, ctx, cfg = hv.last_plan, hv.last_ctx, hv.cfg
    ss = collect_strips(plan, ctx)
    planned_order, owner = [], {}
    for u, p in sorted(plan.plans.items()):
        for tk in p.strips:
            sid = tk[0] if isinstance(tk, (list, tuple)) else str(tk)
            planned_order.append(sid)
            owner[sid] = u
    rank = {sid: i for i, sid in enumerate(planned_order)}

    groups = {}
    for s in ss:
        groups.setdefault((s.cell_id, s.cls), []).append(s)

    out = dict(label=label, scenario=sc.key, trial=sc.trial,
               h_ceil_m=(120.0 if sc.waiver_max_agl_m is None else sc.waiver_max_agl_m),
               coverage_altitude_m=sc.coverage_altitude_m,
               fleet="%dA+%dB" % (sc.n_a, sc.n_b), wind=list(sc.wind),
               plan_s=round(dt, 1), status=report.get("status"),
               makespan_s=round(float(plan.makespan_s), 2),
               uncovered=len(plan.uncovered), cells={},
               uav={u: dict(cls=p.cls, strips=len(p.strips),
                            distance_m=round(float(p.distance_m), 1),
                            energy_wh=round(float(p.energy_used_wh), 2),
                            margin_wh=round(float(p.energy_margin_wh), 2))
                    for u, p in sorted(plan.plans.items())})

    tot = {a: 0.0 for a in ("seq_t", "bid_t", "seq_l", "bid_l", "hk_t", "skip_t", "skip_l", "hk_l")}
    for (cid, cls), lst in sorted(groups.items()):
        geo = sorted(lst, key=lambda s: float(s.v_lo_m))     # sequential = geometric order
        ids = [s.strip_id for s in geo]
        pos = {sid: i for i, sid in enumerate(ids)}
        t_oper, Ct, Cl, meta = cell_ruler(geo, cls, ctx)
        n, k = meta["n"], meta["k"]
        model = (getattr(ctx, "models_rep", {}) or {}).get(cls)
        if model is not None:
            def wh(t_s, _m=model):
                return round(float(_m.transfer_wh(float(t_s), 0.0)), 3)
        else:
            def wh(t_s):
                return None
        orders = {"sequential": list(range(n)), "skip_k_fixed": _skip_sequence(n, k)}
        seen = [sid for sid in sorted(ids, key=lambda x: rank.get(x, 10 ** 9)) if sid in rank]
        if len(seen) == n:
            orders["beam_idp_planned"] = [pos[s] for s in seen]
        hk = _held_karp_path(Ct) if 2 <= n <= 14 else None
        if hk is not None:
            orders["held_karp_opt"] = list(hk)
        base_t, base_l = chain(Ct, orders["sequential"]), chain(Cl, orders["sequential"])
        rows = {}
        for nm, sq in orders.items():
            tt, ll = chain(Ct, sq), chain(Cl, sq)
            rows[nm] = dict(order=[int(i) for i in sq],
                            turn_time_s=round(tt, 2), turn_dist_m=round(ll, 1),
                            turn_energy_wh=wh(tt),
                            cell_time_s=round(t_oper + tt, 2),
                            cell_energy_wh=wh(t_oper + tt),
                            d_turn_time_pct=(round(100 * (tt - base_t) / base_t, 2) if base_t else 0.0),
                            d_turn_dist_pct=(round(100 * (ll - base_l) / base_l, 2) if base_l else 0.0),
                            d_cell_time_pct=(round(100 * (tt - base_t) / (t_oper + base_t), 2)
                                             if (t_oper + base_t) else 0.0))
        out["cells"]["%s|%s" % (cid, cls)] = dict(
            cls=cls, cell=cid, uav=owner.get(ids[0], "?"),
            n=n, k=k, triggers=bool(n > k),
            spacing_m=round(meta["spacing_m"], 2), r_eff_m=round(meta["r_eff_m"], 1),
            strip_len_m=round(meta["strip_len_m"], 1), z_op_m=round(meta["z_op_m"], 2),
            one_way_strips=meta["one_way"], t_oper_s=round(t_oper, 2),
            p_cruise_k_unc_w=(round(float(model.p_cruise_w() * model.k_unc), 1) if model else None),
            orders=rows)
        tot["seq_t"] += chain(Ct, orders["sequential"])
        tot["seq_l"] += chain(Cl, orders["sequential"])
        tot["skip_t"] += chain(Ct, orders["skip_k_fixed"])
        tot["skip_l"] += chain(Cl, orders["skip_k_fixed"])
        bo = orders.get("beam_idp_planned", orders["skip_k_fixed"])
        tot["bid_t"] += chain(Ct, bo)
        tot["bid_l"] += chain(Cl, bo)
        ho = orders.get("held_karp_opt", bo)
        tot["hk_t"] += chain(Ct, ho)
        tot["hk_l"] += chain(Cl, ho)
    out["fleet_turn_totals"] = {k2: round(v, 2) for k2, v in tot.items()}
    out["makespan_sequential_estimate_s"] = round(
        float(plan.makespan_s) + (tot["seq_t"] - tot["bid_t"]) / max(1, len(plan.plans)), 1)
    out["makespan_estimate_note"] = (
        "ESTIMATE ONLY: adds the fleet turn-time delta spread over the fleet; "
        "entry/RTB legs and 4-D deconfliction are NOT re-planned.")
    return out


def show(r):
    print("=" * 118)
    print("%s  scenario=%s  h_ceil=%s m  %s  plan=%ss  status=%s  makespan=%s s  uncovered=%s"
          % (r["label"], r["scenario"], r["h_ceil_m"], r["fleet"], r["plan_s"],
             r["status"], r["makespan_s"], r["uncovered"]))
    print("  %-16s %-3s %-3s %-3s %-2s %-5s %-18s %9s %10s %8s %10s %8s %8s"
          % ("cell", "cls", "uav", "n", "k", "trig", "order", "turn_t", "turn_m",
             "turn_Wh", "cell_t", "d_t%", "d_m%"))
    for c in r["cells"].values():
        for nm, v in c["orders"].items():
            print("  %-16s %-3s %-3s %-3s %-2s %-5s %-18s %9.2f %10.1f %8.3f %10.2f %8.2f %8.2f"
                  % (c["cell"], c["cls"], c["uav"], c["n"], c["k"], c["triggers"], nm,
                     v["turn_time_s"], v["turn_dist_m"], (v["turn_energy_wh"] or 0.0),
                     v["cell_time_s"], v["d_turn_time_pct"], v["d_turn_dist_pct"]))
        print("  " + "-" * 114)
    t = r["fleet_turn_totals"]
    print("  FLEET turn totals: sequential %.1f s / %.1f m | Beam-IDP %.1f s / %.1f m | "
          "skip-k %.1f s | Held-Karp %.1f s"
          % (t["seq_t"], t["seq_l"], t["bid_t"], t["bid_l"], t["skip_t"], t["hk_t"]))
    if t["seq_t"]:
        print("  FLEET turn delta: time %+.2f%%  distance %+.2f%%"
              % (100 * (t["bid_t"] - t["seq_t"]) / t["seq_t"],
                 100 * (t["bid_l"] - t["seq_l"]) / t["seq_l"]))
    print("  makespan as planned %s s ; sequential ESTIMATE %s s"
          % (r["makespan_s"], r["makespan_sequential_estimate_s"]))
    sys.stdout.flush()


if __name__ == "__main__":
    jobs = [("NEW  (Beam-IDP active)", SCENARIOS["beamidp_1a2b_1195"]),
            ("PAPER (as flown HV4SITL15/16, cap 120)", SCENARIOS["gui_legal_120_1a2b"]),
            ("PAPER (same geometry at the 119.5 m cap)",
             dataclasses.replace(SCENARIOS["gui_legal_120_1a2b"],
                                 key="gui_legal_120_1a2b@119.5",
                                 trial="PAPERPLAN1_PREP", coverage_altitude_m=119.5,
                                 waiver_max_agl_m=119.5))]
    res = []
    for label, sc in jobs:
        try:
            r = analyse(sc, label)
        except BaseException as e:                                  # noqa: BLE001
            r = dict(label=label, scenario=sc.key, error="%s: %s" % (type(e).__name__, e))
            print("ERROR " + label + " " + r["error"])
        res.append(r)
        if "error" not in r:
            show(r)
    dst = os.path.join(HERE, "beamidp_table.json")
    json.dump(res, open(dst, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("")
    print("wrote " + dst)
