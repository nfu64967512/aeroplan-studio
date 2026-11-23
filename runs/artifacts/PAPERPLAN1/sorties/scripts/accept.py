# -*- coding: utf-8 -*-
"""Acceptance checks for one HV4W* / BEAMIDP1 sortie (label: sorties).

Read-only with respect to every other label's evidence.  Everything it reports
is recomputed here from the files the run itself produced; nothing is copied
from a prior agent's prose.

    python accept.py <TRIAL> [--expect-items 82,53,84] [--sim-wind 5@270]
                              [--plan-wind 5@270] [--ceil 119.5]

Writes runs/artifacts/PAPERPLAN1/sorties/<TRIAL>_accept.json and prints a
human summary.  Exit code 0 always: the caller reads the JSON verdicts.
"""
from __future__ import annotations

import csv
import json
import math
import statistics
import sys
from pathlib import Path

ROOT = Path(r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
ART = ROOT / "runs" / "artifacts"
OUT = ART / "PAPERPLAN1" / "sorties"
M_LAT = 111132.0


def _load(p: Path):
    return json.loads(p.read_text(encoding="utf-8")) if p.exists() else None


# ───────────────────────── waypoint file: max relative altitude ────────────
def wp_max_rel_alt(p: Path):
    """QGC WPL 110 text.  col 3 = frame, col 10 = z.  frame 3 = REL_ALT."""
    mx, n, n_over = None, 0, 0
    for line in p.read_text(encoding="utf-8").splitlines()[1:]:
        f = line.split("\t")
        if len(f) < 12:
            f = line.split()
        if len(f) < 12:
            continue
        try:
            frame, z = int(f[2]), float(f[10])
        except ValueError:
            continue
        if frame != 3:          # only relative-altitude items are AGL here
            continue
        n += 1
        mx = z if mx is None else max(mx, z)
        if z > 120.0:
            n_over += 1
    return {"n_rel_items": n, "max_rel_alt_m": mx, "n_items_above_120": n_over}


def param_of(p: Path, name: str):
    for line in p.read_text(encoding="utf-8").splitlines():
        parts = line.replace(",", " ").split()
        if len(parts) >= 2 and parts[0].strip().upper() == name:
            return float(parts[1])
    return None


# ───────────────────────── flight.csv ──────────────────────────────────────
def read_flight(p: Path):
    rows = []
    for r in csv.DictReader(p.open(encoding="utf-8")):
        if not r.get("t_s") or not r.get("lat"):
            continue
        row = {"t": float(r["t_s"]), "lat": float(r["lat"]), "lon": float(r["lon"]),
               "alt": float(r["rel_alt_m"]), "as": float(r["airspeed"]),
               "gs": float(r["groundspeed"]), "mode": r.get("mode", ""),
               "armed": (r.get("armed", "") == "True")}
        for k, col in (("w_est", "wind_spd_est"), ("w_dir", "wind_dir_from_est")):
            v = (r.get(col) or "").strip()
            row[k] = float(v) if v else None
        rows.append(row)
    return rows


def track_km(rows):
    d = 0.0
    for a, b in zip(rows, rows[1:]):
        if a["alt"] < 2.0 and b["alt"] < 2.0:
            continue
        dy = (b["lat"] - a["lat"]) * M_LAT
        dx = (b["lon"] - a["lon"]) * 111412.0 * math.cos(math.radians(a["lat"]))
        d += math.hypot(dx, dy)
    return d / 1000.0


def wind_bins(rows):
    """Signed east-minus-west of (groundspeed - airspeed).  Same metric and
    same gates as c2_wind/wind_probe_from_flightcsv.py, plus N/S bins for the
    180 deg case."""
    samples = []
    for a, b in zip(rows, rows[1:]):
        if a["alt"] < 30.0 or b["alt"] < 30.0:
            continue
        if a["gs"] < 10.0 or a["as"] < 10.0:
            continue
        dt = b["t"] - a["t"]
        if not (0.1 < dt < 30.0):
            continue
        dy = (b["lat"] - a["lat"]) * M_LAT
        dx = (b["lon"] - a["lon"]) * 111412.0 * math.cos(math.radians(a["lat"]))
        if math.hypot(dx, dy) < 5.0:
            continue
        theta = math.degrees(math.atan2(dx, dy)) % 360.0
        samples.append((theta, a["gs"] - a["as"]))
    if not samples:
        return None

    def _bin(lo, hi):
        v = [s[1] for s in samples if (lo <= s[0] < hi if lo < hi
                                       else (s[0] >= lo or s[0] < hi))]
        return {"n": len(v),
                "median_gs_minus_as": round(statistics.median(v), 3) if v else None}

    e, w = _bin(45.0, 135.0), _bin(225.0, 315.0)
    n_, s_ = _bin(315.0, 45.0), _bin(135.0, 225.0)
    ew = (None if not (e["n"] and w["n"])
          else round(e["median_gs_minus_as"] - w["median_gs_minus_as"], 3))
    sn = (None if not (s_["n"] and n_["n"])
          else round(s_["median_gs_minus_as"] - n_["median_gs_minus_as"], 3))
    w_est = [r["w_est"] for r in rows if r.get("w_est") is not None and r["alt"] > 30.0]
    w_dir = [r["w_dir"] for r in rows if r.get("w_dir") is not None and r["alt"] > 30.0]
    return {
        "n_cruise_samples": len(samples),
        "median_gs_minus_as_all": round(statistics.median([s[1] for s in samples]), 3),
        "eastbound_45_135": e, "westbound_225_315": w,
        "northbound_315_45": n_, "southbound_135_225": s_,
        "east_minus_west": ew, "implied_wind_along_EW_mps":
            None if ew is None else round(ew / 2.0, 3),
        "south_minus_north": sn, "implied_wind_along_NS_mps":
            None if sn is None else round(sn / 2.0, 3),
        "fc_wind_est_median_mps": round(statistics.median(w_est), 3) if w_est else None,
        "fc_wind_dir_from_median_deg": round(statistics.median(w_dir), 1) if w_dir else None,
        "n_fc_wind_samples": len(w_est),
    }


def main():
    trial = sys.argv[1]
    args = {}
    for a in sys.argv[2:]:
        if a.startswith("--") and "=" in a:
            k, v = a[2:].split("=", 1)
            args[k] = v
    d = ART / trial
    res = {"trial": trial, "dir": str(d), "gates": {}, "measured": {}}

    # ── G1 params ────────────────────────────────────────────────────────
    pr = d / "params_readback.csv"
    if pr.exists():
        rows = list(csv.DictReader(pr.open(encoding="utf-8")))
        bad = [r for r in rows if r["ok"] != "True"]
        res["gates"]["G1_params"] = {
            "n_rows": len(rows), "n_ok": len(rows) - len(bad),
            "expected_rows": 102, "bad": bad[:10],
            "pass": len(rows) == 102 and not bad,
        }
        res["measured"]["fence_alt_max"] = next(
            (r["actual"] for r in rows if r["param"] == "FENCE_ALT_MAX"), None)
    else:
        res["gates"]["G1_params"] = {"pass": False, "reason": "params_readback.csv missing"}

    # ── G2 wind readback ────────────────────────────────────────────────
    wr = _load(d / "wind_readback.json")
    if wr is not None:
        rws = wr["rows"] if isinstance(wr, dict) and "rows" in wr else wr
        bad = [r for r in rws if not r.get("ok")]
        mg = [r for r in rws if r.get("param") == "MIN_GROUNDSPEED"]
        # NB: `x or 1` turns a legitimate 0.0 into 1 — test for None explicitly.
        mg_zero = bool(mg) and all(
            r.get("actual") is not None and float(r["actual"]) == 0.0 for r in mg)
        # every row's readback must equal what was commanded
        mism = [r for r in rws if r.get("actual") is None
                or float(r["actual"]) != float(r["expected"])]
        res["gates"]["G2_wind_readback"] = {
            "n_rows": len(rws), "n_ok": len(rws) - len(bad), "expected_rows": 15,
            "bad": bad, "mismatched": mism,
            "commanded": (wr.get("sim_wind") if isinstance(wr, dict) else None),
            "plan_wind": (wr.get("plan_wind") if isinstance(wr, dict) else None),
            "min_groundspeed_actuals": [r.get("actual") for r in mg],
            "min_groundspeed_all_zero": mg_zero,
            "rows": rws,
            "pass": len(rws) == 15 and not bad and not mism and mg_zero,
        }
    else:
        res["gates"]["G2_wind_readback"] = {"pass": False, "reason": "wind_readback.json missing"}

    # ── G3 spawn ────────────────────────────────────────────────────────
    sp = _load(d / "spawn_readback.json")
    if sp is not None:
        rws = sp["rows"] if isinstance(sp, dict) else sp
        errs = [r["err_m"] for r in rws]
        res["gates"]["G3_spawn"] = {"max_err_m": max(errs), "n": len(rws),
                                    "all_ok": all(r.get("ok") for r in rws),
                                    "pass": max(errs) <= 2.0 and all(r.get("ok") for r in rws)}

    # ── G4 mission upload ───────────────────────────────────────────────
    mu = _load(d / "mission_upload.json")
    if mu is not None:
        got = {r["uav"]: r["read_back"] for r in mu}
        sent = {r["uav"]: r["sent"] for r in mu}
        exp = None
        if "expect-items" in args:
            names = sorted(got)
            exp = dict(zip(names, [int(x) for x in args["expect-items"].split(",")]))
        res["gates"]["G4_mission_items"] = {
            "sent": sent, "read_back": got, "expected": exp,
            "all_ok": all(r.get("ok") for r in mu),
            "pass": all(r.get("ok") for r in mu) and (exp is None or got == exp),
        }

    # ── G5 landed ───────────────────────────────────────────────────────
    rel = _load(d / "release_schedule.json")
    rep = _load(d / "report.json")
    fs = _load(d / "flight_summary.json")
    g5 = {"report_status": (rep or {}).get("status"),
          "report_errors": (rep or {}).get("errors")}
    if isinstance(rel, dict):
        g5["mode_change_rejected"] = rel.get("mode_change_rejected")
        g5["link_lost"] = rel.get("link_lost")
    landed = {}
    for sub in sorted(x for x in d.iterdir() if x.is_dir() and (x / "flight.csv").exists()):
        rws = read_flight(sub / "flight.csv")
        landed[sub.name] = {"final_alt_m": rws[-1]["alt"] if rws else None,
                            "final_armed": rws[-1]["armed"] if rws else None}
    g5["final_state"] = landed
    g5["pass"] = bool(landed) and all(
        v["final_alt_m"] is not None and v["final_alt_m"] < 3.0 and not v["final_armed"]
        for v in landed.values())
    res["gates"]["G5_landed"] = g5

    # ── H1 exported altitude / H3 RTL_RADIUS ────────────────────────────
    plan = d / "plan"
    h1, rtl = {}, {}
    for wp in sorted(plan.glob("*.waypoints")):
        h1[wp.stem] = wp_max_rel_alt(wp)
    for pf in sorted(plan.glob("*.param")):
        rtl[pf.stem] = param_of(pf, "RTL_RADIUS")
    res["measured"]["H1_exported_max_rel_alt_m"] = h1
    res["gates"]["H3_rtl_radius"] = {"values": rtl,
                                     "pass": rtl.get("A1") == 200.0 and all(
                                         v == 120.0 for k, v in rtl.items() if k != "A1")}

    # ── H2 flown altitude (flight.csv, 1 Hz) ────────────────────────────
    ms = _load(plan / "mission_summary.json")
    flown, sampling, winds = {}, {}, {}
    for sub in sorted(x for x in d.iterdir() if x.is_dir() and (x / "flight.csv").exists()):
        rws = read_flight(sub / "flight.csv")
        flown[sub.name] = {
            "max_rel_alt_m": round(max(r["alt"] for r in rws), 4),
            "flown_km": round(track_km(rws), 4),
            "t_first_s": rws[0]["t"], "t_last_s": rws[-1]["t"],
            "n_samples": len(rws),
        }
        dts = [b["t"] - a["t"] for a, b in zip(rws, rws[1:]) if 0 < b["t"] - a["t"] < 30]
        sampling[sub.name] = {"median_dt_s": round(statistics.median(dts), 3) if dts else None,
                              "p95_dt_s": round(sorted(dts)[int(0.95 * (len(dts) - 1))], 3) if dts else None}
        winds[sub.name] = wind_bins(rws)
    res["measured"]["H2_flown_max_rel_alt_m_flightcsv"] = flown
    res["gates"]["H2_below_120"] = {
        "max_over_fleet_m": round(max(v["max_rel_alt_m"] for v in flown.values()), 4) if flown else None,
        "per_uav": {k: v["max_rel_alt_m"] for k, v in flown.items()},
        "pass": bool(flown) and all(v["max_rel_alt_m"] < 120.0 for v in flown.values()),
        "note": "flight.csv is ~1 Hz; the dataflash figure in *_binalt.json is the authoritative one",
    }
    res["gates"]["G6_sampling"] = {"per_uav": sampling,
                                   "pass": all(v["median_dt_s"] is not None and v["median_dt_s"] <= 1.0
                                               for v in sampling.values())}
    res["measured"]["wind_probe"] = winds

    # ── makespan / distance, planned vs flown ───────────────────────────
    if ms:
        plan_km = {k: round(v.get("distance_m", 0.0) / 1000.0, 4) for k, v in ms["uavs"].items()}
        res["measured"]["planned"] = {
            "makespan_s": ms["makespan_s"],
            "t_land_s": {k: v["t_land_s"] for k, v in ms["uavs"].items()},
            "t_takeoff_s": {k: v["t_takeoff_s"] for k, v in ms["uavs"].items()},
            "distance_km": plan_km,
            "total_km": round(sum(plan_km.values()), 4),
            "n_items": {k: v["n_items"] for k, v in ms["uavs"].items()},
            "strips": {k: len(v.get("strips", [])) for k, v in ms["uavs"].items()},
        }
        if "makespan" in args:
            exp = float(args["makespan"])
            res["gates"]["H1_planned_makespan"] = {
                "expected_s": exp, "actual_s": ms["makespan_s"],
                "delta_s": round(ms["makespan_s"] - exp, 4),
                "pass": abs(ms["makespan_s"] - exp) < 0.05}
    if fs:
        res["measured"]["flight_summary"] = fs
        # flown_t_takeoff_s / flown_t_land_s are already in the PLAN clock
        # (clock_offset_s has been subtracted by the reporter: HV4SITL15 has
        # plan_t_takeoff 656.0 -> flown 659.5, offset 67.5).  Do not subtract again.
        fl_land = {r["uav"]: r["flown_t_land_s"] for r in fs}
        fl_off = {r["uav"]: r["flown_t_takeoff_s"] for r in fs}
        off = fs[0].get("clock_offset_s", 0.0)
        res["measured"]["flown"] = {
            "makespan_s": round(max(fl_land.values()), 2),
            "t_land_s": fl_land, "t_takeoff_s": fl_off, "clock_offset_s": off,
            "flown_km_report": {r["uav"]: r["flown_km"] for r in fs},
            "err_pct_report": {r["uav"]: r["err_pct"] for r in fs},
        }
        if ms:
            res["measured"]["flown"]["makespan_vs_planned_s"] = round(
                max(fl_land.values()) - ms["makespan_s"], 2)
            res["measured"]["flown"]["makespan_err_pct"] = round(
                100.0 * (max(fl_land.values()) - ms["makespan_s"]) / ms["makespan_s"], 3)
            res["measured"]["flown"]["total_km_vs_planned"] = {
                "flown": round(sum(v["flown_km"] for v in flown.values()), 4),
                "planned": res["measured"]["planned"]["total_km"]}

    # ── separation ──────────────────────────────────────────────────────
    sep = _load(d / "separation_from_tracks.json")
    if sep:
        pv = _load(d / "validation.json") or {}
        planned_h = (pv.get("min_sep_h_m") if "min_sep_h_m" in pv
                     else (pv.get("separation", {}) or {}).get("min_sep_h_m"))
        res["gates"]["S_separation"] = {
            "flown_min_h_m": sep["min_h_m"], "d_v_at_min_m": sep["d_v_at_min_m"],
            "pair": sep["pair"], "n_violations": sep["n_violations"],
            "step_s": sep.get("step_s"),
            "planned_min_sep_h_m": planned_h,
            "planned_minus_flown_m": (None if planned_h is None or sep["min_h_m"] is None
                                      else round(planned_h - sep["min_h_m"], 3)),
            "min_sep_v_m_planned": (pv.get("min_sep_v_m") if "min_sep_v_m" in pv
                                    else (pv.get("separation", {}) or {}).get("min_sep_v_m")),
            "pass": sep["n_violations"] == 0 and (sep["min_h_m"] or 0) >= 100.0,
            "caveat": "step_s=2.0 with linear interpolation makes the horizontal distance an UPPER bound",
        }

    # ── sim_wind block as the runner recorded it ────────────────────────
    sc = _load(d / "scenario.json")
    if sc:
        res["measured"]["scenario"] = {
            "sim_wind": sc.get("sim_wind"),
            "wind": sc.get("wind"),
            "altitude_m": sc.get("altitude_m"),
            "h_ceil_m": sc.get("h_ceil_m"),
            "waiver_max_agl_m": sc.get("waiver_max_agl_m"),
            "trial": sc.get("trial"), "key": sc.get("key"),
        }

    res["all_gates_pass"] = all(v.get("pass") for v in res["gates"].values())
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"{trial}_accept.json").write_text(
        json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")

    print(f"=== {trial} ===")
    for k, v in res["gates"].items():
        print(f"  {'PASS' if v.get('pass') else 'FAIL'}  {k}")
    print(f"  ALL GATES: {'PASS' if res['all_gates_pass'] else 'FAIL'}")
    print(json.dumps({"gates": res["gates"], "measured": res["measured"]},
                     ensure_ascii=False, indent=1)[:12000])


if __name__ == "__main__":
    main()
