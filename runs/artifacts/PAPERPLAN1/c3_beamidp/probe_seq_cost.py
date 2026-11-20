"""C3 probe: sequential (boustrophedon) vs skip-k / Beam-IDP strip ordering.

Uses the planner's OWN cost model:
  - hv_dccpp.dubins.dubins_all_times  (wind-corrected Dubins transition time)
  - hv_dccpp.dubins.skip_k            (k = ceil(2 R_eff / s))
  - hv_dccpp.decompose._skip_sequence / _best_sequence / _held_karp_path
  - hv_dccpp.platform.PlatformSpec.r_eff (R_eff = max(R_plan, R_theo (1+W/Va)^2))

Pair poses are taken verbatim from decompose.exact_time._pair_poses (both-direction case):
  right end: a swept LR -> (p^R_a, th)  to  b start RL (p^R_b, th+pi)   weight 0.5
  left  end: a swept RL -> (p^L_a, th+pi) to b start LR (p^L_b, th)     weight 0.5

Read-only. No SITL. Deterministic.
"""
from __future__ import annotations
import json, math, sys, os
import numpy as np

ROOT = r"C:/Users/joy46/Documents/Projects/aeroplan-studio"
sys.path.insert(0, ROOT)

from hv_dccpp.dubins import dubins_all_times, skip_k, omega_length, uturn_length  # noqa: E402
from hv_dccpp.geometry import Strip, sweep_frame                                   # noqa: E402
from hv_dccpp.decompose import _skip_sequence, _best_sequence, _held_karp_path     # noqa: E402
from hv_dccpp.platform import PLATFORM_PIONEER as PIONEER_SPEC, PLATFORM_HERO as HERO_SPEC                              # noqa: E402
from hv_dccpp.mission import PlannerConfig                                         # noqa: E402


def make_strips(n: int, s: float, L: float, alpha: float = math.pi / 2.0, cls: str = "A"):
    """n parallel strips, spacing s along the right normal n_hat(alpha), heading alpha."""
    t_hat, n_hat = sweep_frame(alpha)
    out = []
    for i in range(n):
        base = (i * s) * n_hat
        pL = base.copy()
        pR = base + L * t_hat
        out.append(Strip(strip_id=f"S{i}", cell_id="C", index=i, cls=cls,
                         pL=pL, pR=pR, theta_LR_rad=alpha,
                         sweep_origin=np.zeros(2), width_m=s, spacing_m=s,
                         h_min_m=100.0, h_op_m=120.0, agl_max_m=120.0,
                         v_lo_m=i * s, v_hi_m=i * s + s, length_m=L, z_op_m=120.0))
    return out


def cost_matrices(strips, r_eff, v_air, wind_xy, dtheta=math.radians(5.0)):
    """(C_time, C_len): n x n transition cost, both-direction weighting 0.5/0.5."""
    n = len(strips)
    starts, ends, weights, slots = [], [], [], []
    for a in range(n):
        for b in range(n):
            if a == b:
                continue
            sa, sb = strips[a], strips[b]
            slots.append((a, b, len(starts)))
            starts.extend([sa.end_state(-1).as_array(), sa.end_state(+1).as_array()])
            ends.extend([sb.start_state(+1).as_array(), sb.start_state(-1).as_array()])
            weights.extend([0.5, 0.5])
    times, lengths, _idx = dubins_all_times(np.array(starts), np.array(ends), r_eff,
                                            v_air, wind_xy, dtheta)
    tw = np.asarray(times) * np.asarray(weights)
    lw = np.asarray(lengths) * np.asarray(weights)
    Ct = np.full((n, n), math.inf); Cl = np.full((n, n), math.inf)
    for a, b, i0 in slots:
        Ct[a, b] = float(tw[i0:i0 + 2].sum())
        Cl[a, b] = float(lw[i0:i0 + 2].sum())
    return Ct, Cl


def chain(C, seq):
    return float(sum(C[a, b] for a, b in zip(seq[:-1], seq[1:])))


def run_case(cls, spec, n, s, L, wind_mps, cfg):
    W = float(wind_mps)
    r_eff = spec.r_eff(W)
    # 5 m/s from the WEST = blowing toward the east => wind vector +x (east)
    wind_xy = np.array([W, 0.0])
    strips = make_strips(n, s, L, cls=cls)
    Ct, Cl = cost_matrices(strips, r_eff, spec.cruise_airspeed_mps, wind_xy)
    k = skip_k(s, r_eff)
    seq_seq = list(range(n))                       # boustrophedon / sequential
    seq_fix = _skip_sequence(n, k)                 # HARVEST fixed skip-k
    seq_srch = _best_sequence(n, k, Ct, cfg)       # HARVEST "search" (folded serpentine + HK)
    hk = _held_karp_path(Ct) if n <= 14 else None
    rows = {
        "sequential": seq_seq,
        "skip_k_fixed": seq_fix,
        "beam_search": seq_srch,
    }
    if hk is not None:
        rows["held_karp_opt"] = hk
    out = {"cls": cls, "n": n, "s_m": s, "L_m": L, "wind_mps": W,
           "R_eff_m": r_eff, "k": k, "Va_mps": spec.cruise_airspeed_mps,
           "orders": {}}
    base_t = chain(Ct, seq_seq); base_l = chain(Cl, seq_seq)
    for name, sq in rows.items():
        t = chain(Ct, sq); l = chain(Cl, sq)
        out["orders"][name] = {
            "order": list(map(int, sq)),
            "turn_time_s": round(t, 2),
            "turn_len_m": round(l, 1),
            "d_time_vs_seq_pct": round(100.0 * (t - base_t) / base_t, 2) if base_t else 0.0,
            "d_len_vs_seq_pct": round(100.0 * (l - base_l) / base_l, 2) if base_l else 0.0,
        }
    # closed-form sanity: single adjacent reversal transition
    out["closed_form"] = {
        "omega_len_adjacent_m": round(omega_length(s, r_eff), 1),
        "uturn_len_k_s_m": round(uturn_length(k * s, r_eff), 1),
    }
    return out


def main():
    cfg = PlannerConfig(sweep_sequence="search")
    results = []
    L = 1690.0          # strip length ~ HV4SITL15 measured (1685.5-1695.4 m)
    for wind in (0.0, 5.0):
        for cls, spec in (("A", PIONEER_SPEC), ("B", HERO_SPEC)):
            for s in (97.0, 138.6):
                for n in (2, 3, 4, 5, 6, 8, 9, 10, 12):
                    results.append(run_case(cls, spec, n, s, L, wind, cfg))
    out = {"note": "planner-native cost model; turn_* are TRANSITION cost only "
                   "(strip operating time is order-independent)",
           "specs": {
               "A_pioneer": {"Va": PIONEER_SPEC.cruise_airspeed_mps,
                             "R_theo": PIONEER_SPEC.turn_radius_theoretical_m,
                             "R_plan": PIONEER_SPEC.turn_radius_planning_m,
                             "w_star": round(PIONEER_SPEC.w_star(), 3)},
               "B_hero": {"Va": HERO_SPEC.cruise_airspeed_mps,
                          "R_theo": HERO_SPEC.turn_radius_theoretical_m,
                          "R_plan": HERO_SPEC.turn_radius_planning_m,
                          "w_star": round(HERO_SPEC.w_star(), 3)}},
           "cases": results}
    dst = os.path.join(os.path.dirname(os.path.abspath(__file__)), "seq_cost.json")
    with open(dst, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    # console table
    print(f"{'cls':>3} {'W':>4} {'s':>6} {'n':>3} {'Reff':>6} {'k':>2} "
          f"{'seq_t':>9} {'skipk_t':>9} {'srch_t':>9} {'HK_t':>9} "
          f"{'skipk%':>8} {'srch%':>8} {'HK%':>8}")
    for r in results:
        o = r["orders"]
        hk = o.get("held_karp_opt")
        print(f"{r['cls']:>3} {r['wind_mps']:>4.1f} {r['s_m']:>6.1f} {r['n']:>3} "
              f"{r['R_eff_m']:>6.1f} {r['k']:>2} "
              f"{o['sequential']['turn_time_s']:>9.1f} {o['skip_k_fixed']['turn_time_s']:>9.1f} "
              f"{o['beam_search']['turn_time_s']:>9.1f} "
              f"{(hk['turn_time_s'] if hk else float('nan')):>9.1f} "
              f"{o['skip_k_fixed']['d_time_vs_seq_pct']:>8.2f} "
              f"{o['beam_search']['d_time_vs_seq_pct']:>8.2f} "
              f"{(hk['d_time_vs_seq_pct'] if hk else float('nan')):>8.2f}")
    print("\nwrote", dst)


if __name__ == "__main__":
    main()
