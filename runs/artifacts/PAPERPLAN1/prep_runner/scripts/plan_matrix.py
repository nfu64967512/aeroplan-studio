# -*- coding: utf-8 -*-
"""PLANNING-ONLY sortie matrix for PAPERPLAN1 / prep_runner.

Runs `scripts/hv_sitl_run.py plan gui_legal_120_1a2b` once per matrix cell with
EXACTLY the environment the RUNBOOK publishes for that cell, then harvests the
metrics from the artifacts that run produced.  NO SITL / ArduPilot / MAVProxy
process is started anywhere in this file (`plan` never calls launch()).

Output: runs/artifacts/PAPERPLAN1/prep_runner/plan_matrix.json  (+ per-cell dirs)
"""
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
OUT = ROOT / "runs/artifacts/PAPERPLAN1/prep_runner"
MATRIX_DIR = "PAPERPLAN1/prep_runner/plan_matrix"
SCEN = "gui_legal_120_1a2b"
SEP_THRESHOLD_M = 100.0          # hv_dccpp horizontal separation threshold
FLAG_MARGIN_M = 5.0              # task: flag any cell within 5 m of it

# cell -> (plan W, plan dir, sim W, sim dir, ceiling or None)
CELLS = [
    ("W00_c1195",  0.0, 270.0, 0.0, 270.0, "119.5"),
    ("W05_c1195",  0.0, 270.0, 5.0, 270.0, "119.5"),
    ("W55_c1195",  5.0, 270.0, 5.0, 270.0, "119.5"),
    ("W55S_c1195", 5.0, 180.0, 5.0, 180.0, "119.5"),
    ("W00_c120",   0.0, 270.0, 0.0, 270.0, None),
    ("W05_c120",   0.0, 270.0, 5.0, 270.0, None),
    ("W55_c120",   5.0, 270.0, 5.0, 270.0, None),
    ("W55S_c120",  5.0, 180.0, 5.0, 180.0, None),
]


def max_rel_alt(wp_file: Path) -> tuple[float, int]:
    """Max altitude column of a QGC WPL 110 file, over frame-3 (rel-to-home) rows."""
    best, n = float("-inf"), 0
    for ln in wp_file.read_text(encoding="utf-8").splitlines():
        if ln.startswith("QGC") or ln.lstrip().startswith("#") or not ln.strip():
            continue
        f = ln.split("\t")
        if len(f) < 12:
            f = ln.split()
        if len(f) < 12:
            continue
        frame, z = int(float(f[2])), float(f[10])
        if frame == 3:                      # MAV_FRAME_GLOBAL_RELATIVE_ALT
            best = max(best, z)
            n += 1
    return (best, n)


def params_of(pf: Path) -> dict:
    out = {}
    for ln in pf.read_text(encoding="utf-8").splitlines():
        ln = ln.strip()
        if ln and not ln.startswith("#") and "," in ln:
            k, v = ln.split(",", 1)
            out[k.strip()] = float(v)
    return out


def run_cell(name, pw, pd, sw, sd, ceil):
    trial = f"{MATRIX_DIR}/{name}"
    env = dict(os.environ)
    env["PYTHONUTF8"] = "1"
    env["HV_TRIAL"] = trial
    env["HV_PLAN_WIND_SPD"] = f"{pw:g}"
    env["HV_PLAN_WIND_DIR"] = f"{pd:g}"
    env["HV_SIM_WIND_SPD"] = f"{sw:g}"
    env["HV_SIM_WIND_DIR"] = f"{sd:g}"
    for k in ("HV_PLAN_CEIL_M", "HV_PLAN_ALT_M"):
        env.pop(k, None)
    if ceil is not None:
        env["HV_PLAN_CEIL_M"] = ceil
    t0 = time.perf_counter()
    r = subprocess.run([sys.executable, "-u", "scripts/hv_sitl_run.py", "plan", SCEN],
                       cwd=str(ROOT), env=env, capture_output=True, text=True,
                       encoding="utf-8", errors="replace", timeout=900)
    dt = time.perf_counter() - t0
    d = ROOT / "runs/artifacts" / trial
    (OUT / "logs").mkdir(parents=True, exist_ok=True)
    (OUT / "logs" / f"plan_{name}.log").write_text(
        r.stdout + "\n--- STDERR ---\n" + r.stderr, encoding="utf-8")
    row = {"cell": name, "plan_wind_mps": pw, "plan_wind_dir_from_deg": pd,
           "sim_wind_mps": sw, "sim_wind_dir_from_deg": sd,
           "plan_ceil_env": ceil, "rc": r.returncode, "wall_s": round(dt, 1),
           "trial_dir": str(d)}
    if r.returncode != 0:
        row["error"] = (r.stdout + r.stderr)[-2000:]
        return row
    ms = json.loads((d / "plan/mission_summary.json").read_text(encoding="utf-8"))
    va = json.loads((d / "validation.json").read_text(encoding="utf-8").replace("Infinity", '"Infinity"'))
    sc = json.loads((d / "scenario.json").read_text(encoding="utf-8"))
    rep = json.loads((d / "report.json").read_text(encoding="utf-8"))
    per = {}
    for u, p in sorted(ms["uavs"].items()):
        mz, nz = max_rel_alt(d / "plan" / f"{u}.waypoints")
        pv = params_of(d / "plan" / f"{u}.param")
        per[u] = {"cls": p["cls"], "strips": len(p["strips"]), "n_items": p["n_items"],
                  "distance_m": round(p["distance_m"], 1),
                  "t_takeoff_s": round(p["t_takeoff_s"], 1),
                  "t_land_s": round(p["t_land_s"], 1),
                  "energy_used_wh": round(p["energy_used_wh"], 1),
                  "energy_margin_wh": round(p["energy_margin_wh"], 1),
                  "max_exported_rel_alt_m": round(mz, 4),
                  "n_rel_frame_items": nz,
                  "RTL_RADIUS_m_eq_R_eff": pv.get("RTL_RADIUS"),
                  "WP_LOITER_RAD_m": pv.get("WP_LOITER_RAD"),
                  "FENCE_ALT_MAX_m": pv.get("FENCE_ALT_MAX"),
                  "Q_RTL_ALT_m": pv.get("Q_RTL_ALT"),
                  "n_params_exported": len(pv)}
    sep_h = va["min_sep_h_m"]
    sep_v = va["min_sep_v_m"]
    row.update({
        "status": rep.get("status"),
        "go": rep.get("status") in ("GO", "GO_WITH_WARNINGS"),
        "validate_passed": va["passed"], "conflicts": va["conflicts"],
        "uncovered": len(rep.get("uncovered") or []),
        "makespan_s": round(ms["makespan_s"], 2),
        "total_distance_m": round(sum(p["distance_m"] for p in ms["uavs"].values()), 1),
        "min_sep_h_m": round(sep_h, 3) if isinstance(sep_h, (int, float)) else sep_h,
        "min_sep_v_m": round(sep_v, 3) if isinstance(sep_v, (int, float)) else sep_v,
        "sep_margin_m": (round(sep_h - SEP_THRESHOLD_M, 3)
                         if isinstance(sep_h, (int, float)) else None),
        "SEP_FLAG": (isinstance(sep_h, (int, float))
                     and sep_h - SEP_THRESHOLD_M < FLAG_MARGIN_M),
        "max_exported_rel_alt_m": round(max(p["max_exported_rel_alt_m"] for p in per.values()), 4),
        "scenario_h_ceil_m": sc.get("h_ceil_m"),
        "scenario_altitude_m": sc.get("altitude_m"),
        "scenario_waiver_max_agl_m": sc.get("waiver_max_agl_m"),
        "scenario_plan_wind": sc.get("wind"),
        "scenario_sim_wind": sc.get("sim_wind"),
        "R_eff_m": {u: per[u]["RTL_RADIUS_m_eq_R_eff"] for u in per},
        "n_params_exported": {u: per[u]["n_params_exported"] for u in per},
        "uavs": per,
        "warnings": rep.get("warnings"),
    })
    return row


def main():
    only = sys.argv[1:] or None
    rows = []
    for c in CELLS:
        if only and c[0] not in only:
            continue
        print(f"--- {c[0]} ...", flush=True)
        row = run_cell(*c)
        rows.append(row)
        print(f"    rc={row['rc']} {row.get('status')} makespan={row.get('makespan_s')} "
              f"dist={row.get('total_distance_m')} sep_h={row.get('min_sep_h_m')} "
              f"maxalt={row.get('max_exported_rel_alt_m')} ({row['wall_s']}s)", flush=True)
    p = OUT / "plan_matrix.json"
    old = json.loads(p.read_text(encoding="utf-8")) if p.exists() else {}
    for r in rows:
        old[r["cell"]] = r
    p.write_text(json.dumps(old, ensure_ascii=False, indent=1), encoding="utf-8")
    print("wrote", p)


if __name__ == "__main__":
    main()
