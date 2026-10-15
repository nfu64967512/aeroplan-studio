"""跨航次比較：把幾趟 HARVEST SITL 的規劃 vs 實飛擺在一起。

    python scripts/hv_sitl_compare.py HV4SITL2 HV4SITL3 HV4SITL4

只讀 `runs/artifacts/<TRIAL>/` 底下已經產生的 `flight_summary.json`、
`scenario.json`（新版航次才有）與 `plan/mission_summary.json`，不重跑任何東西。
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def load(trial: str) -> dict | None:
    d = ROOT / "runs" / "artifacts" / trial
    if not (d / "flight_summary.json").exists():
        print(f"{trial}: 沒有 flight_summary.json（跑過 hv_sitl_report.py 了嗎？）")
        return None
    rows = json.loads((d / "flight_summary.json").read_text(encoding="utf-8"))
    meta = {}
    if (d / "scenario.json").exists():
        meta = json.loads((d / "scenario.json").read_text(encoding="utf-8"))
    val = {}
    if (d / "validation.json").exists():
        val = json.loads((d / "validation.json").read_text(encoding="utf-8"))
    rel = {}
    if (d / "release_schedule.json").exists():
        rel = json.loads((d / "release_schedule.json").read_text(encoding="utf-8"))
    return {"trial": trial, "rows": rows, "meta": meta, "val": val, "rel": rel}


def main() -> int:
    trials = sys.argv[1:]
    if not trials:
        print("用法：python scripts/hv_sitl_compare.py HV4SITL2 HV4SITL3 ...")
        return 1
    runs = [r for r in (load(t) for t in trials) if r]
    if not runs:
        return 1

    print(f"\n{'航次':10s}{'路徑':8s}{'機隊':7s}{'高度':>7s}{'架':>4s}"
          f"{'規劃 km':>10s}{'實飛 km':>10s}{'差':>7s}{'落地':>6s}{'validate':>10s}")
    for r in runs:
        m, rows = r["meta"], r["rows"]
        p = sum(x["plan_km"] for x in rows)
        f = sum(x["flown_km"] for x in rows)
        landed = sum(1 for x in rows if x["flown_t_land_s"] > 0)
        fleet = (f"{m.get('n_a', '?')}A+{m.get('n_b', '?')}B" if m else "?")
        print(f"{r['trial']:10s}{m.get('path', '?'):8s}{fleet:7s}"
              f"{m.get('altitude_m', 0):7.0f}{len(rows):4d}"
              f"{p:10.2f}{f:10.2f}{100 * (f - p) / max(p, 1e-9):+6.1f}%"
              f"{landed:4d}/{len(rows):<2d}"
              f"{'passed' if r['val'].get('passed') else 'FAILED':>10s}")

    print(f"\n逐機：\n{'航次':10s}{'機':4s}{'構型':9s}"
          f"{'規劃起飛':>9s}{'放行(牆鐘)':>11s}"
          f"{'規劃落地':>9s}{'實飛落地':>9s}{'航程誤差':>9s}{'最高 m':>8s}")
    for r in runs:
        rel = r["rel"].get("actual_release_wall_s", {})
        for x in r["rows"]:
            print(f"{r['trial']:10s}{x['uav']:4s}{x['class']:9s}"
                  f"{x['plan_t_takeoff_s']:9.0f}{rel.get(x['uav'], float('nan')):11.1f}"
                  f"{x['plan_t_land_s']:9.0f}{x['flown_t_land_s']:9.0f}"
                  f"{x['err_pct']:+8.1f}%{x['peak_alt_m']:8.1f}")

    print("\n落地順序（規劃 → 實飛）：")
    for r in runs:
        plan_order = [x["uav"] for x in sorted(r["rows"], key=lambda x: x["plan_t_land_s"])]
        flown_order = [x["uav"] for x in sorted(r["rows"], key=lambda x: x["flown_t_land_s"])]
        same = "一致" if plan_order == flown_order else "★ 不一致"
        print(f"  {r['trial']:10s} {' → '.join(plan_order):24s} | "
              f"{' → '.join(flown_order):24s}  {same}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
