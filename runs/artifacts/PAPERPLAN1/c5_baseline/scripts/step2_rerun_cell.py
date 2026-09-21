"""C5 step 2 -- re-run ONE benchmark cell from today's working tree and dump
both the metrics (to compare against the committed CSV) and the raw tracks
(so step 3 can audit them without re-planning).

usage: python step2_rerun_cell.py <METHOD> <SCEN> <N>
Pure Python. No SITL, no servers, no repo writes outside runs/artifacts/.
"""
import json, sys, time, hashlib
from pathlib import Path

REPO = Path(r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
BENCH = REPO / "research/cpp_benchmark"
for p in (str(REPO), str(BENCH)):
    if p not in sys.path:
        sys.path.insert(0, p)

OUT = Path(__file__).resolve().parents[1] / "out" / "tracks"
OUT.mkdir(parents=True, exist_ok=True)

import matrix as M                                   # noqa: E402
from bench import metrics                            # noqa: E402

method, scen, n = sys.argv[1], sys.argv[2], int(sys.argv[3])
polys, _ = M.SCEN[scen]
fam, fleet, fn = M.METHODS[method]

t0 = time.perf_counter()
tracks, ms = fn(polys, n)
dt = time.perf_counter() - t0
m = metrics(polys, tracks, M.SWATH)
self_sig = getattr(fn, "last_self", None) or {}

rec = dict(method=method, scenario=scen, n_uav_req=n, sec=round(dt, 2),
           self_makespan_s=(round(ms, 1) if ms else ""), **m, **self_sig,
           n_tracks=len(tracks), wpts=[len(t) for t in tracks])
print("@@ROW@@" + json.dumps(rec, ensure_ascii=False))

tag = f"{method}_{scen}_n{n}"
payload = dict(meta=rec, swath_m=M.SWATH, pitch_m=M.PITCH, alt_m=M.ALT,
               speed_ref_mps=20.0, home_xy=[0.0, 0.0],
               wind_mps_fromdeg=[5.0, 270.0],   # bench.run_planner default
               tracks=[[[round(x, 3), round(y, 3)] for x, y in t] for t in tracks])
f = OUT / f"{tag}.json"
f.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
print("wrote", f, "md5", hashlib.md5(f.read_bytes()).hexdigest())
