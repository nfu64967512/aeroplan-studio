"""只重跑本專案那 4 列（DCCPP / DCCPP_mix / HARVEST / HARVEST_mix）。

⚠⚠⚠ 為什麼要單獨重跑：封存表格的 provenance 警告寫著那 4 列是對**工作樹 +
未提交修改**量的，而 2026-09-08 之後 `mission/swarm_coordinator.py` 又大改過
（最低固定翼作業高度可調、高度帶下限、切區三個缺陷修正、禁航區子區改整數 id）。
15 個基準線是自足的、不受影響，所以不重跑，免得浪費幾十分鐘。

輸出：`results/project_rows_<gitsha>.csv`，並在檔頭記下**當下的**版本狀態
（HEAD、是否有未提交修改、兩個關鍵檔的 md5），這樣事後看得出這批數字是哪一版量的。
"""
from __future__ import annotations

import csv
import hashlib
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from matrix import FIELDS, SCEN                            # noqa: E402

ROOT = HERE.parent.parent
PROJECT_METHODS = ["DCCPP", "DCCPP_mix", "HARVEST", "HARVEST_mix"]
WATCH = ["mission/swarm_coordinator.py", "core/dccpp/dccpp_path_builder.py"]
TIMEOUT = int(os.environ.get("CELL_TIMEOUT", "1800"))
JOBS = int(os.environ.get("JOBS", "4"))


def _git(*args) -> str:
    try:
        return subprocess.run(["git", *args], cwd=str(ROOT), capture_output=True,
                              text=True, timeout=30).stdout.strip()
    except Exception:                                       # noqa: BLE001
        return "?"


def provenance() -> dict:
    head = _git("rev-parse", "HEAD")
    dirty = bool(_git("status", "--porcelain"))
    md5 = {}
    for rel in WATCH:
        p = ROOT / rel
        md5[rel] = (hashlib.md5(p.read_bytes()).hexdigest() if p.exists() else "missing")
    return {"head": head, "dirty_working_tree": dirty, "md5": md5}


def work(cell):
    m, s, n = cell
    env = dict(os.environ, PYTHONUTF8="1", PYTHONIOENCODING="utf-8")
    t0 = time.perf_counter()
    try:
        p = subprocess.run([sys.executable, "-u", str(HERE / "matrix.py"), m, s, str(n)],
                           capture_output=True, text=True, timeout=TIMEOUT,
                           cwd=str(HERE), env=env, encoding="utf-8", errors="replace")
        for line in (p.stdout or "").splitlines():
            if line.startswith("@@ROW@@"):
                return json.loads(line[7:])
        return dict(method=m, scenario=s, n_uav_req=n,
                    error=f"no row (rc={p.returncode}) {(p.stderr or '')[-200:]}",
                    sec=round(time.perf_counter() - t0, 1))
    except subprocess.TimeoutExpired:
        return dict(method=m, scenario=s, n_uav_req=n, error=f"TIMEOUT>{TIMEOUT}s",
                    sec=round(time.perf_counter() - t0, 1))


def main() -> int:
    prov = provenance()
    cells = [(m, s, n) for s in ("S5", "S1", "S2", "S4", "S3")
             for n in SCEN[s][1] for m in PROJECT_METHODS]
    print(f"重跑 {len(cells)} 格（{len(PROJECT_METHODS)} 個方法）")
    print(f"HEAD {prov['head'][:12]}  dirty={prov['dirty_working_tree']}")
    for k, v in prov["md5"].items():
        print(f"  {k}  md5 {v[:12]}")
    rows = []
    with ThreadPoolExecutor(max_workers=JOBS) as ex:
        for r in ex.map(work, cells):
            rows.append(r)
            tag = r.get("error") or f"{r.get('total_km')} km  cov {r.get('coverage')}"
            print(f"  {r['method']:12} {r['scenario']} n={r['n_uav_req']:<3} {tag}")
    out = HERE / "results" / f"project_rows_{prov['head'][:8]}{'_dirty' if prov['dirty_working_tree'] else ''}.csv"
    out.parent.mkdir(exist_ok=True)
    with out.open("w", newline="", encoding="utf-8") as f:
        f.write(f"# provenance: {json.dumps(prov, ensure_ascii=False)}\n")
        w = csv.DictWriter(f, fieldnames=FIELDS, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print("寫出", out)
    bad = [r for r in rows if r.get("error")]
    print(f"完成 {len(rows) - len(bad)}/{len(rows)}，失敗 {len(bad)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
