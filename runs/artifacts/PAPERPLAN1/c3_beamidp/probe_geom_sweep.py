"""C3 probe 3: search for a geometry whose per-cell strip count really triggers
skip-k interlacing (n > k), with Pioneer k=5 and Hero k=3.

Same planner path as HV4SITL15 (scripts.hv_sitl_run._plan_gui), same airspace
(no waiver => 120 m AGL ceiling, min_fw 100/100, dh band 15, GSD 0.05).
Pure Python, no SITL. Writes only into this folder.
"""
from __future__ import annotations
import json, os, sys, time, traceback

ROOT = r"C:/Users/joy46/Documents/Projects/aeroplan-studio"
sys.path.insert(0, ROOT)
os.chdir(ROOT)

from scripts.hv_sitl_scenarios import Scenario, rect                      # noqa: E402
from scripts.hv_sitl_run import _plan_gui                                 # noqa: E402
from hv_dccpp.dubins import skip_k                                        # noqa: E402
from hv_dccpp.decompose import r_eff_for_class                            # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))

AIRSPACE = dict(coverage_altitude_m=120.0, coverage_width_m=150.0, overlap_rate=0.30,
                waiver_max_agl_m=None, min_fw_agl_m={"A": 100.0, "B": 100.0},
                dh_band_min_m=15.0, target_gsd=0.05, spawn_spread_m=10.0)


def mk(key, n_a, n_b, half_e, half_n):
    return Scenario(key=key, trial="PAPERPLAN1_C3", path="gui", n_a=n_a, n_b=n_b,
                    areas=[{"area_id": 1, "priority": 1.0,
                            "polygon": rect(0, 0, half_e, half_n)}],
                    note="C3 probe", **AIRSPACE)


def summarise(key, sc):
    t0 = time.perf_counter()
    rec = {"key": key, "n_a": sc.n_a, "n_b": sc.n_b,
           "area_km2": round(4 * sc.areas[0]["polygon"] and 0, 3)}
    try:
        hv, report = _plan_gui(sc)
    except BaseException as e:                                # noqa: BLE001
        rec["error"] = f"{type(e).__name__}: {e}"
        rec["plan_s"] = round(time.perf_counter() - t0, 1)
        return rec
    dt = time.perf_counter() - t0
    plan, ctx = hv.last_plan, hv.last_ctx
    reff = {}
    for cls in ("A", "B"):
        try:
            reff[cls] = round(float(r_eff_for_class(ctx, cls)), 2)
        except Exception:                                     # noqa: BLE001
            pass
    rec.update(plan_s=round(dt, 1), status=report.get("status"),
               makespan_s=round(float(plan.makespan_s), 1),
               uncovered=len(plan.uncovered), R_eff_m=reff,
               uavs={u: {"cls": p.cls, "n_strips": len(p.strips),
                         "km": round(p.distance_m / 1000, 2)}
                     for u, p in sorted(plan.plans.items())})
    groups = {}
    seq = []
    reg = (getattr(plan, "strips", None) or getattr(ctx, "strips", None)
           or getattr(plan, "all_strips", None) or getattr(ctx, "all_strips", None))
    def _flat(x):
        if isinstance(x, dict):
            for v in x.values():
                _flat(v)
        elif isinstance(x, (list, tuple, set)):
            for v in x:
                _flat(v)
        elif hasattr(x, "cell_id") and hasattr(x, "spacing_m"):
            seq.append(x)
    _flat(reg)
    for st in seq:
        groups.setdefault((st.cell_id, st.cls), []).append(st)
    cells = {}
    for (cid, cls), lst in sorted(groups.items()):
        s_mean = sum(float(x.spacing_m) for x in lst) / len(lst)
        R = reff.get(cls)
        k = skip_k(s_mean, R) if (R and s_mean > 0) else None
        cells[f"{cid}|{cls}"] = {"n": len(lst), "s": round(s_mean, 2),
                                 "L": round(sum(float(x.length_m) for x in lst) / len(lst), 1),
                                 "R_eff": R, "k": k,
                                 "triggers": bool(k and len(lst) > k)}
    rec["cells"] = cells
    rec["max_n_over_k"] = max([c["n"] - (c["k"] or 0) for c in cells.values()] or [0])
    rec["any_trigger"] = any(c["triggers"] for c in cells.values())
    return rec


CASES = [
    # key,             n_a, n_b, half_e, half_n   -> area = (2he x 2hn)
    ("base_1560x800_1a2b", 1, 2, 780, 400),
    ("solo_A_1560x800",    1, 0, 780, 400),
    ("solo_B_1560x800",    0, 1, 780, 400),
    ("solo_A_1560x1600",   1, 0, 780, 800),
    ("solo_B_1560x1600",   0, 1, 780, 800),
    ("big_1560x2400_1a2b", 1, 2, 780, 1200),
    ("big_2000x3000_1a2b", 1, 2, 1000, 1500),
    ("big_3000x3000_1a2b", 1, 2, 1500, 1500),
    ("big_2600x2600_1a2b", 1, 2, 1300, 1300),
    ("solo_A_2000x1000",   1, 0, 1000, 500),
]


def main():
    want = sys.argv[1:]
    out = []
    for key, na, nb, he, hn in CASES:
        if want and key not in want:
            continue
        sc = mk(key, na, nb, he, hn)
        sc_area = (2 * he) * (2 * hn) / 1e6
        rec = summarise(key, sc)
        rec["area_km2"] = round(sc_area, 3)
        rec["rect_m"] = [2 * he, 2 * hn]
        out.append(rec)
        print(f"--- {key}  {2*he}x{2*hn} m ({sc_area:.2f} km2)  "
              f"{rec.get('plan_s')}s  {rec.get('status')}  "
              f"makespan={rec.get('makespan_s')}  err={rec.get('error','')}")
        for cid, c in (rec.get("cells") or {}).items():
            print(f"      {cid:<24} n={c['n']:<3} s={c['s']:<7} L={c['L']:<8} "
                  f"R_eff={c['R_eff']} k={c['k']} trigger={c['triggers']}")
        sys.stdout.flush()
    dst = os.path.join(HERE, "geom_sweep.json")
    prev = []
    if os.path.exists(dst):
        try:
            prev = json.load(open(dst, encoding="utf-8"))
        except Exception:                                     # noqa: BLE001
            prev = []
    keys = {r["key"] for r in out}
    merged = [r for r in prev if r.get("key") not in keys] + out
    json.dump(merged, open(dst, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("wrote", dst)


if __name__ == "__main__":
    main()