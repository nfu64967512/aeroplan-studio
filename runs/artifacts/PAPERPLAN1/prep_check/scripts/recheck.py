"""prep_check: re-verify the load-bearing claims of the four prep workstreams.

Pure Python. Starts NO SITL / ArduPilot / MAVProxy process, touches no WSL tree,
no COM port, no git index. Read-only except for its own output JSON.

Run:  python runs/artifacts/PAPERPLAN1/prep_check/scripts/recheck.py
"""
from __future__ import annotations
import csv, hashlib, json, math, subprocess, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT))
OUT = ROOT / "runs/artifacts/PAPERPLAN1/prep_check/recheck.json"
R: dict = {}


def wp_scan(p: Path) -> dict:
    """QGC WPL 110 reader. Item count, max REL-frame alt, items above 120.000."""
    n, mx, over = 0, -1e9, 0
    for line in p.read_text(encoding="utf-8").splitlines():
        s = line.strip()
        if not s or s.startswith("#") or s.startswith("QGC"):
            continue
        q = s.split("\t")
        if len(q) < 12:
            q = s.split()
        if len(q) < 12:
            continue
        n += 1
        if int(q[2]) == 3:                    # MAV_FRAME_GLOBAL_RELATIVE_ALT
            a = float(q[10])
            mx = max(mx, a)
            over += a > 120.0
    return {"items": n, "max_rel_m": round(mx, 4), "items_over_120": over}


def ignored(rel: str) -> bool:
    return subprocess.run(["git", "check-ignore", "-q", rel], cwd=ROOT).returncode == 0


# 1 -- C1 ablation vs the prep_runner planning matrix -----------------------
ab = json.loads((ROOT / "runs/artifacts/PAPERPLAN1/c1_altitude/ablation_cap.json").read_text("utf-8"))
pm = json.loads((ROOT / "runs/artifacts/PAPERPLAN1/prep_runner/plan_matrix.json").read_text("utf-8"))
R["c1_vs_matrix"] = {
    "ablation_119_5": {k: ab["119.5"][k] for k in ("makespan_s", "min_sep_h_m", "swath_m", "spacing_m")},
    "matrix_W55_c1195": {k: pm["W55_c1195"][k] for k in ("makespan_s", "min_sep_h_m")},
    "ablation_120": {k: ab["120"][k] for k in ("makespan_s", "min_sep_h_m", "swath_m", "spacing_m")},
    "matrix_W55_c120": {k: pm["W55_c120"][k] for k in ("makespan_s", "min_sep_h_m")},
    "agree": (ab["119.5"]["makespan_s"] == pm["W55_c1195"]["makespan_s"]
              and ab["119.5"]["min_sep_h_m"] == pm["W55_c1195"]["min_sep_h_m"]
              and ab["120"]["makespan_s"] == pm["W55_c120"]["makespan_s"]
              and ab["120"]["min_sep_h_m"] == pm["W55_c120"]["min_sep_h_m"]),
}
R["separation_flag_planned"] = {c: pm[c]["min_sep_h_m"] for c in pm}

# 2 -- exported altitude / item counts, parsed independently ---------------
R["exports"] = {}
for tag, d in (("HV4SITL16_cap120", "runs/artifacts/HV4SITL16/plan"),
               ("BEAMIDP1_cap1195", "runs/artifacts/BEAMIDP1/plan"),
               ("W55_c1195", "runs/artifacts/PAPERPLAN1/prep_runner/plan_matrix/W55_c1195/plan"),
               ("W00_c1195", "runs/artifacts/PAPERPLAN1/prep_runner/plan_matrix/W00_c1195/plan")):
    p = ROOT / d
    if p.exists():
        R["exports"][tag] = {f.name: wp_scan(f) for f in sorted(p.glob("*.waypoints"))}

# 3 -- skip-k threshold, live from the library -----------------------------
from hv_dccpp.dubins import skip_k                       # noqa: E402
from hv_dccpp.decompose import _skip_sequence            # noqa: E402
R["skip_k"] = {"s=%s,R=%s" % (s, Rm): skip_k(s, Rm)
               for s in (96.995, 96.591) for Rm in (200.0, 120.0)}
R["skip_sequence_identity"] = {
    "k=%d" % k: {
        "identity_up_to_n": max(n for n in range(1, 12) if _skip_sequence(n, k) == list(range(n))),
        "first_differing_n": min(n for n in range(1, 12) if _skip_sequence(n, k) != list(range(n))),
        "first_differing_seq": _skip_sequence(min(n for n in range(1, 12)
                                                  if _skip_sequence(n, k) != list(range(n))), k),
    } for k in (5, 3)}

# 4 -- wind triangle: the signed acceptance rule ---------------------------
import numpy as np                                        # noqa: E402
from hv_dccpp.wind import ground_speed                    # noqa: E402
w_toward = np.array([5.0, 0.0])          # dir_from 270 => air moves toward 090 (east)
R["wind_triangle_W5_from270"] = {}
for name, Va in (("A_pioneer_Va21.5", 21.5), ("B_hero_Va19.0", 19.0)):
    vs = {lbl: round(float(ground_speed(math.radians(deg), Va, w_toward)[0]), 3)
          for lbl, deg in (("east", 90), ("west", 270), ("north", 0), ("south", 180))}
    vs["east_minus_west"] = round(vs["east"] - vs["west"], 3)
    R["wind_triangle_W5_from270"][name] = vs

# 5 -- C4 parameter gate, recounted ----------------------------------------
rows = list(csv.DictReader((ROOT / "runs/artifacts/HV4SITL16/params_readback.csv").open(encoding="utf-8")))
per_uav: dict = {}
for r in rows:
    per_uav[r["uav"]] = per_uav.get(r["uav"], 0) + 1
R["c4_params"] = {
    "rows": len(rows), "per_uav": per_uav,
    "all_ok": all(r["ok"] == "True" for r in rows),
    "param_file_lines": {f.name: sum(1 for ln in f.read_text(encoding="utf-8").splitlines()
                                     if ln.strip() and not ln.startswith("#"))
                         for f in sorted((ROOT / "runs/artifacts/HV4SITL16/plan").glob("*.param"))},
}

# 6 -- flown separation: every artifact in the tree ------------------------
flown = {}
for f in sorted((ROOT / "runs/artifacts").glob("HV4SITL*/separation_from_tracks.json")):
    flown[f.parent.name] = json.loads(f.read_text("utf-8")).get("min_h_m")
R["flown_min_sep_all_trials"] = flown
R["flown_min_sep_smallest"] = min((v for v in flown.values() if v is not None), default=None)
R["HV4SITL16_has_separation_artifact"] = (
    ROOT / "runs/artifacts/HV4SITL16/separation_from_tracks.json").exists()

# 7 -- "no env => unchanged": NOENV vs the archived HV4SITL15 plan ---------
a = ROOT / "runs/artifacts/PAPERPLAN1/prep_runner/plan_matrix/NOENV/plan"
b = ROOT / "runs/artifacts/HV4SITL15/plan"
same, diff = [], []
if a.exists():
    for f in sorted(a.iterdir()):
        g = b / f.name
        if not g.exists():
            continue
        h1 = hashlib.sha256(f.read_bytes()).hexdigest()
        h2 = hashlib.sha256(g.read_bytes()).hexdigest()
        (same if h1 == h2 else diff).append(f.name)
R["noenv_vs_HV4SITL15"] = {"identical": same, "differing": diff}

# 8 -- beam-IDP: makespan estimate + per-cell trigger + paper-cell orders ---
bt = json.loads((ROOT / "runs/artifacts/PAPERPLAN1/prep_beamidp/beamidp_table.json").read_text("utf-8"))
new = bt[0]
per_ac: dict = {}
for cid, c in new["cells"].items():
    u = c["uav"]
    per_ac[u] = per_ac.get(u, 0.0) + (c["orders"]["sequential"]["turn_time_s"]
                                      - c["orders"]["beam_idp_planned"]["turn_time_s"])
fr = json.loads((ROOT / "runs/artifacts/PAPERPLAN1/prep_beamidp/fly_ready.json").read_text("utf-8"))
t_land = {u: v["t_land_s"] for u, v in fr["mission_summary_uavs"].items()} \
    if isinstance(fr.get("mission_summary_uavs"), dict) else {}
shifted = {u: round(t_land[u] + per_ac.get(u, 0.0), 2) for u in t_land if u in per_ac}
R["beamidp_makespan_estimate"] = {
    "artifact_field_makespan_sequential_estimate_s": new["makespan_sequential_estimate_s"],
    "per_aircraft_method_s": round(max(shifted.values()), 2) if shifted else None,
    "per_aircraft_shifted_landing_s": shifted,
    "per_aircraft_turn_time_deltas_s": {k: round(v, 2) for k, v in per_ac.items()},
    "planned_makespan_s": new["makespan_s"],
    "note": ("two estimators disagree: the JSON field spreads the fleet turn-time delta, "
             "the prose adds each aircraft's own delta to its own landing time"),
}
R["beamidp_trigger_per_cell"] = {cid: {"cls": c["cls"], "uav": c["uav"], "n": c["n"],
                                       "k": c["k"], "triggers": c["triggers"]}
                                 for cid, c in new["cells"].items()}
R["paper_cell_orders_n_gt_1"] = {cid: {k: v["order"] for k, v in c["orders"].items()}
                                 for cid, c in bt[1]["cells"].items() if c["n"] > 1}

# 9 -- git-ignore status of every trial name in play -----------------------
R["gitignore_status"] = {t: ("IGNORED" if ignored("runs/artifacts/%s/report.json" % t) else "trackable")
                         for t in ("HV4SITL15", "HV4SITL16", "HV4SITL17", "HV4W00", "HV4W05",
                                   "HV4W55", "HV4W55S", "HV4W55R", "BEAMIDP1", "PAPERPLAN1")}

# 10 - benchmark CSV provenance --------------------------------------------
csvp = "research/cpp_benchmark/results/project_rows_5e0295f5_dirty.csv"
R["benchmark_csv"] = {
    "path": csvp, "exists": (ROOT / csvp).exists(), "git_ignored": ignored(csvp),
    "tracked": subprocess.run(["git", "ls-files", "--error-unmatch", csvp], cwd=ROOT,
                              capture_output=True).returncode == 0,
    "bench_py_dirty": bool(subprocess.run(["git", "status", "--porcelain",
                                           "research/cpp_benchmark/bench.py"],
                                          cwd=ROOT, capture_output=True, text=True).stdout.strip()),
    "swarm_coordinator_md5": hashlib.md5((ROOT / "mission/swarm_coordinator.py").read_bytes()).hexdigest(),
}

# 11 - machine state (read-only; confirms nothing has flown with the patch) -
lastlog = ROOT / "sitl/vtol/i100/logs/LASTLOG.TXT"
R["machine"] = {
    "wind_parm_present": sorted(str(p.relative_to(ROOT)) for p in (ROOT / "sitl/vtol").glob("i1*/wind.parm")),
    "lastlog": lastlog.read_text().strip() if lastlog.exists() else None,
}

OUT.parent.mkdir(parents=True, exist_ok=True)
OUT.write_text(json.dumps(R, ensure_ascii=False, indent=1), encoding="utf-8")
print(json.dumps(R, ensure_ascii=False, indent=1))
print("\nwrote", OUT)
