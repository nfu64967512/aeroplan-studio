"""matrix.py -- scenario x n x method matrix runner.

Worker mode:  python matrix.py <method> <scenario> <n>   -> one JSON line on stdout
Driver mode:  python run_matrix.py                        (spawns workers, writes CSV)

All metrics come from bench.metrics (the single ruler). No method reports its own
length or coverage. Time = length / SPEED_REF for every method.
"""
from __future__ import annotations

from pathlib import Path
import math
import sys

# ⚠ 原本硬編 session scratchpad 的路徑（會消失），改成 repo 相對。
BENCH = str(Path(__file__).resolve().parent)
if BENCH not in sys.path:
    sys.path.insert(0, BENCH)

from shapely.geometry import Polygon                       # noqa: E402
from bench import metrics, rect_xy                         # noqa: E402

ALT, HFOV = 140.0, 60.0
SWATH = 2 * ALT * math.tan(math.radians(HFOV) / 2)         # 161.66 m
PITCH = SWATH * 0.7                                        # 113.16 m
OVERLAP = 0.0
START = (0.0, 0.0)

# ── scenarios ────────────────────────────────────────────────────────────
_L = Polygon([(-2000, -2000), (2000, -2000), (2000, 0),
              (0, 0), (0, 2000), (-2000, 2000)])           # 4x4 minus 2x2 top-right

SCEN = {
    "S1": ([rect_xy(0, 0, 2500, 2000)], [2, 4, 8, 16]),                 # 5.0x4.0 km
    "S2": ([rect_xy(cx, 0, 625, 2000) for cx in (-3750, -1250, 1250, 3750)],
           [4, 8]),                                                     # 4x 1.25x4.0
    "S3": ([rect_xy(0, 0, 4000, 500)], [4, 8]),                         # 8.0x1.0 km
    "S4": ([_L], [4, 8]),                                               # L concave
    "S5": ([rect_xy(0, 0, 5000, 4000)], [8, 16]),                       # 10x8 km
}
SCEN_DESC = {
    "S1": "single 5.0x4.0 km (20.0 km2)",
    "S2": "4 blocks 1.25x4.0 km, separated (20.0 km2)",
    "S3": "thin 8.0x1.0 km (8.0 km2)",
    "S4": "L concave, 4x4 minus 2x2 (12.0 km2)",
    "S5": "large 10x8 km (80.0 km2)",
}


# ── methods ──────────────────────────────────────────────────────────────
def _baseline(modname, fnname="plan"):
    def run(polys, n):
        mod = __import__(modname)
        fn = getattr(mod, fnname)
        return fn(polys, n, swath_m=SWATH, pitch_m=PITCH, start=START), None
    return run


def _bench_bous(polys, n):
    from bench import boustrophedon
    return boustrophedon(polys, n, swath_m=SWATH, pitch_m=PITCH, start=START), None


def _planner(use_hv, mixed):
    def run(polys, n):
        from bench import run_planner
        n_a, n_b = (n - n // 2, n // 2) if mixed else (n, 0)
        tracks, ms, err, self_sig = run_planner(use_hv, polys, n_a, n_b,
                                                alt=ALT, pitch=PITCH, overlap=OVERLAP)
        if err:
            raise RuntimeError(err)
        run.last_self = self_sig          # run_cell 讀它填欄位
        return tracks, ms
    run.last_self = {}
    return run


METHODS = {
    # family: boustrophedon
    "boustrophedon":     ("bous", "n/a",   _bench_bous),
    "bous_contiguous":   ("bous", "n/a",   _baseline("algo_bous_variants", "plan_contiguous")),
    "bous_axis_aligned": ("bous", "n/a",   _baseline("algo_bous_variants", "plan_axis_aligned")),
    "bous_interleaved":  ("bous", "n/a",   _baseline("algo_bous_variants", "plan_interleaved")),
    "bous_v2":           ("bous", "n/a",   _baseline("algo_bous_variants_v2", "plan")),
    "bous_v2_rev1":      ("bous", "n/a",   _baseline("algo_bous_variants_v2_rev1", "plan")),
    # family: inward spiral
    "spiral":            ("spiral", "n/a", _baseline("algo_spiral")),
    "spiral_v2":         ("spiral", "n/a", _baseline("algo_spiral_v2")),
    "spiral_v2_r1":      ("spiral", "n/a", _baseline("algo_spiral_v2_r1")),
    # family: grid + greedy nearest neighbour
    "grid_nn":           ("grid_nn", "n/a", _baseline("algo_grid_nn")),
    "grid_nn_v2":        ("grid_nn", "n/a", _baseline("algo_grid_nn_v2")),
    "grid_nn_v2b":       ("grid_nn", "n/a", _baseline("algo_grid_nn_v2b")),
    # family: DARP-style area partition + boustrophedon
    "darp":              ("darp", "n/a",  _baseline("algo_darp")),
    "darp_v2":           ("darp", "n/a",  _baseline("algo_darp_v2")),
    "darp_v2_r1":        ("darp", "n/a",  _baseline("algo_darp_v2_r1")),
    # this project
    "DCCPP":             ("project", "homo",  _planner(False, False)),
    "DCCPP_mix":         ("project", "mixed", _planner(False, True)),
    "HARVEST":           ("project", "homo",  _planner(True, False)),
    "HARVEST_mix":       ("project", "mixed", _planner(True, True)),
}

# -- radius-charged metrics -------------------------------------------------
# The pure-geometry ruler gives a free discount to any method that ignores turn
# radius. R200_* charges every corner the arc-vs-corner difference for a 200 m
# minimum turn radius (the class-A airframe in bench._drones), so a route that a
# fixed-wing could not actually fly stops looking cheap. Coverage is unchanged.
R_TURN = 200.0


def _turn_charge(track, R=R_TURN):
    extra, hairpin, sharp = 0.0, 0, 0
    for i in range(1, len(track) - 1):
        ax, ay = track[i][0] - track[i - 1][0], track[i][1] - track[i - 1][1]
        bx, by = track[i + 1][0] - track[i][0], track[i + 1][1] - track[i][1]
        na, nb = math.hypot(ax, ay), math.hypot(bx, by)
        if na < 1e-9 or nb < 1e-9:
            continue
        c = max(-1.0, min(1.0, (ax * bx + ay * by) / (na * nb)))
        d = math.acos(c)
        if math.degrees(d) > 60.0:
            sharp += 1
        if math.degrees(d) > 170.0:              # reversal needs a teardrop
            hairpin += 1
            extra += math.pi * R
        else:
            extra += 2.0 * R * math.tan(d / 2.0) - R * d
    return extra, hairpin, sharp


def r200_metrics(tracks, covered_m2, speed=20.0):
    tracks = [[p for i, p in enumerate(t) if i == 0 or p != t[i - 1]] for t in tracks]
    tracks = [t for t in tracks if len(t) >= 2]
    if not tracks:
        return dict(total_km_r200="", max_km_r200="", scan_rate_r200="",
                    turn_charge_pct="", sharp_turns="")
    lens, ch, hp, sh = [], [], 0, 0
    for t in tracks:
        L = sum(math.dist(t[i], t[i - 1]) for i in range(1, len(t)))
        e, h, s = _turn_charge(t)
        lens.append(L + e)
        ch.append(e)
        hp += h
        sh += s
    base = sum(sum(math.dist(t[i], t[i - 1]) for i in range(1, len(t))) for t in tracks)
    mx = max(lens)
    return dict(total_km_r200=round(sum(lens) / 1000.0, 2),
                max_km_r200=round(mx / 1000.0, 2),
                scan_rate_r200=round((covered_m2 / 1e6) / ((mx / speed) / 3600.0), 3),
                turn_charge_pct=round(sum(ch) / base * 100.0, 1) if base else "",
                sharp_turns=sh)


FIELDS = ["method", "family", "fleet", "scenario", "n_uav_req", "area_km2",
          "n_uav", "total_km", "max_km", "coverage", "overhead", "redundancy",
          "balance", "scan_rate", "waste",
          "total_km_r200", "max_km_r200", "scan_rate_r200",
          "turn_charge_pct", "sharp_turns",
          "sec", "self_makespan_s",
          # 規劃器自己的訊號（只有本專案的方法有；基準線沒有這個概念）
          "self_uncovered", "self_validate", "error"]


def run_cell(method: str, scen: str, n: int) -> dict:
    import time
    polys, _ = SCEN[scen]
    fam, fleet, fn = METHODS[method]
    area = sum(p.area for p in polys) / 1e6
    row = dict(method=method, family=fam, fleet=fleet, scenario=scen,
               n_uav_req=n, area_km2=round(area, 3), self_makespan_s="",
               self_uncovered="", self_validate="", error="")
    t0 = time.perf_counter()
    try:
        tracks, ms = fn(polys, n)
    except Exception as e:                                  # noqa: BLE001
        row["sec"] = round(time.perf_counter() - t0, 2)
        row["error"] = f"{type(e).__name__}: {e}"[:200]
        for k in ("n_uav", "total_km", "max_km", "coverage", "overhead",
                  "redundancy", "balance", "scan_rate", "waste",
                  "total_km_r200", "max_km_r200", "scan_rate_r200",
                  "turn_charge_pct", "sharp_turns"):
            row[k] = ""
        return row
    dt = time.perf_counter() - t0
    m = metrics(polys, tracks, SWATH)
    row.update(m)
    from shapely.geometry import LineString
    from shapely.ops import unary_union
    _area = unary_union(polys)
    _ok = [t for t in tracks if len(t) >= 2]
    _cov = (_area.intersection(unary_union(
        [LineString(t).buffer(SWATH / 2.0, quad_segs=8) for t in _ok])).area
        if _ok else 0.0)
    row.update(r200_metrics(tracks, _cov))
    row["sec"] = round(dt, 2)
    row["self_makespan_s"] = round(ms, 1) if ms else ""
    row.update(getattr(fn, "last_self", None) or {})
    return row


if __name__ == "__main__":
    import json
    method, scen, n = sys.argv[1], sys.argv[2], int(sys.argv[3])
    print("@@ROW@@" + json.dumps(run_cell(method, scen, n)))
