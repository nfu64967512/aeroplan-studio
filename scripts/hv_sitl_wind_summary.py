"""把多趟 hv_sitl_wind_check.py 的輸出（runs/artifacts/WINDFLOW_CHECK/<TRIAL>/wind_check.json）整理成一張對照表。

    python scripts/hv_sitl_wind_summary.py WINDFLOW_F_N_MATCH WINDFLOW_F_N_BLIND ... [--out 目錄]

只讀 JSON、不起 SITL、不寫 data/。表格欄位（逐機）：
  規劃風／模擬風、任務時間（第一次切 AUTO 到觸地）規劃 vs 實飛、逆風段與順風段的地速、條帶段時間
  （模型：規劃風 vs 實飛風）、機頭偏流角（BIN 實測 vs 預計）、最後進場相對模擬風、觸地誤差。
數字都從 wind_check.json 原樣取出，不重算；缺欄位寫 n/a。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CHECK = ROOT / "runs" / "artifacts" / "WINDFLOW_CHECK"


def _g(d, *path, default=None):
    for k in path:
        if not isinstance(d, dict) or k not in d:
            return default
        d = d[k]
    return d


def _f(x, nd=1, sign=False):
    if x is None:
        return "n/a"
    try:
        return f"{x:+.{nd}f}" if sign else f"{x:.{nd}f}"
    except (TypeError, ValueError):
        return "n/a"


def load(trial: str, check_dir: Path | None = None) -> dict:
    p = (check_dir or CHECK) / trial / "wind_check.json"
    if not p.exists():
        raise SystemExit(f"找不到 {p}（先跑 scripts/hv_sitl_wind_check.py）")
    return json.loads(p.read_text(encoding="utf-8"))


def rows_for(trial: str, check_dir: Path | None = None) -> list[dict]:
    d = load(trial, check_dir)
    pw, sw = _g(d, "winds", "plan") or {}, _g(d, "winds", "sim") or {}
    out = []
    for uav, a in (d.get("per_uav") or {}).items():
        t = a.get("times") or {}
        lt = a.get("leg_time_totals") or {}
        cls = a.get("leg_classes_vs_ref") or {}
        crab = _g(d, "plan_wind_handling", "measured_crab_from_bin", uav, "max_abs_crab_deg")
        ap = _g(a, "landing", "approach") or {}
        out.append({
            "trial": trial, "uav": uav, "class": a.get("class"),
            "plan_wind": f"{_f(pw.get('speed_mps'))}@{_f(pw.get('dir_from_deg'), 0)}",
            "sim_wind": f"{_f(sw.get('speed_mps'))}@{_f(sw.get('dir_from_deg'), 0)}",
            "dur_plan_s": t.get("duration_plan_s"), "dur_flown_s": t.get("duration_flown_s"),
            "dur_delta_s": t.get("duration_delta_s"), "dur_delta_pct": t.get("duration_delta_pct"),
            "gs_headwind": _g(cls, "headwind", "gs_pos_mean_mps"), "gs_tailwind": _g(cls, "tailwind", "gs_pos_mean_mps"),
            "gs_crosswind": _g(cls, "crosswind", "gs_pos_mean_mps"),
            "legs_flown_s": lt.get("flown_s"), "legs_plan_model_s": lt.get("plan_model_s"), "legs_sim_model_s": lt.get("sim_model_s"),
            "crab_bin_max_deg": crab,
            "approach_headwind_mps": _g(ap, "vs_wind", "sim_wind", "headwind_component_mps"),
            "approach_course_deg": ap.get("course_deg"), "approach_planned_deg": ap.get("planned_bearing_deg"),
            "touchdown_err_m": _g(a, "landing", "touchdown_err_m"),
            "path_delta_pct": _g(a, "path", "delta_pct_all_samples"),
        })
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("trials", nargs="+")
    ap.add_argument("--out", default="")
    ap.add_argument("--check-dir", default="", help="wind_check.json 所在的上層目錄（預設 runs/artifacts/WINDFLOW_CHECK）")
    args = ap.parse_args()
    cdir = Path(args.check_dir) if args.check_dir else None
    rows = [r for t in args.trials for r in rows_for(t, cdir)]
    head = ("| 航次 | 機 | 規劃風 | 模擬風 | 任務時間 規劃→實飛 (s) | 差 (s／%) | 逆風／順風／橫風地速 (m/s) | "
            "條帶段合計：規劃風模型／實飛 (s) | 偏流角 實測最大 (°) | 最後進場頂風 (m/s) | 觸地誤差 (m) |")
    lines = [head, "|" + "---|" * 11]
    for r in rows:
        lines.append(
            f"| {r['trial'].replace('WINDFLOW_F_', '')} | {r['uav']} | {r['plan_wind']} | {r['sim_wind']} | "
            f"{_f(r['dur_plan_s'], 0)}→{_f(r['dur_flown_s'], 0)} | {_f(r['dur_delta_s'], 1, True)}／{_f(r['dur_delta_pct'], 1, True)}% | "
            f"{_f(r['gs_headwind'], 2)}／{_f(r['gs_tailwind'], 2)}／{_f(r['gs_crosswind'], 2)} | "
            f"{_f(r['legs_plan_model_s'], 0)}／{_f(r['legs_flown_s'], 0)} | {_f(r['crab_bin_max_deg'], 1)} | "
            f"{_f(r['approach_headwind_mps'], 2)} | {_f(r['touchdown_err_m'], 2)} |")
    text = "\n".join(lines) + "\n"
    print(text)
    if args.out:
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        (out / "wind_summary.md").write_text(text, encoding="utf-8")
        (out / "wind_summary.json").write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
