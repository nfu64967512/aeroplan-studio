"""Driver: spawn one subprocess per (method, scenario, n) cell, collect CSV.

Subprocess isolation so one hang/crash cannot poison the rest.
Re-run is incremental: cells already present in the CSV are skipped.
"""
from __future__ import annotations

import csv
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from matrix import FIELDS, METHODS, SCEN                    # noqa: E402

OUT = os.path.join(HERE, "matrix_results.csv")
TIMEOUT = int(os.environ.get("CELL_TIMEOUT", "1500"))
JOBS = int(os.environ.get("JOBS", "8"))

# slow scenarios first so they are not the tail
ORDER = ["S5", "S1", "S2", "S4", "S3"]


def cells():
    out = []
    for s in ORDER:
        for n in SCEN[s][1]:
            for m in METHODS:
                out.append((m, s, n))
    return out


def done_set():
    if not os.path.exists(OUT):
        return set()
    with open(OUT, newline="", encoding="utf-8") as f:
        return {(r["method"], r["scenario"], int(r["n_uav_req"]))
                for r in csv.DictReader(f) if not r.get("error")}


def work(cell):
    m, s, n = cell
    env = dict(os.environ, PYTHONUTF8="1", PYTHONIOENCODING="utf-8")
    t0 = time.perf_counter()
    try:
        p = subprocess.run([sys.executable, "-u", os.path.join(HERE, "matrix.py"), m, s, str(n)],
                           capture_output=True, text=True, timeout=TIMEOUT,
                           cwd=HERE, env=env, encoding="utf-8", errors="replace")
        for line in (p.stdout or "").splitlines():
            if line.startswith("@@ROW@@"):
                return json.loads(line[7:])
        return dict(method=m, scenario=s, n_uav_req=n,
                    error=f"no row (rc={p.returncode}) {(p.stderr or '')[-160:]}",
                    sec=round(time.perf_counter() - t0, 1))
    except subprocess.TimeoutExpired:
        return dict(method=m, scenario=s, n_uav_req=n, error=f"TIMEOUT>{TIMEOUT}s",
                    sec=TIMEOUT)


def main():
    todo = [c for c in cells() if c not in done_set()]
    new = not os.path.exists(OUT)
    print(f"cells to run: {len(todo)}  jobs={JOBS}  timeout={TIMEOUT}s", flush=True)
    with open(OUT, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS, extrasaction="ignore")
        if new:
            w.writeheader()
        with ThreadPoolExecutor(max_workers=JOBS) as ex:
            for i, row in enumerate(ex.map(work, todo), 1):
                w.writerow(row)
                f.flush()
                print(f"[{i}/{len(todo)}] {row.get('method'):18s} {row.get('scenario')} "
                      f"n={row.get('n_uav_req'):<3} cov={row.get('coverage','-')} "
                      f"scan={row.get('scan_rate','-')} {row.get('sec','')}s "
                      f"{row.get('error','')}", flush=True)


if __name__ == "__main__":
    main()
