# -*- coding: utf-8 -*-
"""C4（參數讀回 102 項）證據重算。唯讀 repo；所有輸出寫在本目錄。

做兩件事：
 1. 參數清冊：匯出檔 32 項 -> 比對集合 34 項的來源分解，逐機逐項列出。
 2. 航次歸屬：HV4SITL15（舊 17 項閘門 + 已產報告）與 HV4SITL16（新 34 項閘門，
    有 flight.csv 但從未跑 hv_sitl_report.py）是同一份計畫的兩趟。
    這裡用 scripts/hv_sitl_report.py 的同一套公式重算 16 的航程/最高高度/最小間距，
    並先在 15 上驗證公式能重現該趟已存檔的 flight_summary.json。
"""
from __future__ import annotations
import csv, json, math, sys
from collections import Counter
from pathlib import Path

ROOT = Path(r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
ART = ROOT / "runs" / "artifacts"
HERE = Path(__file__).resolve().parent

# ---------- 與 scripts/hv_sitl_run.py 同步的常數（唯讀複製，勿改）----------
EXPECT_FALLBACK = {"Q_ENABLE": 1.0, "ARMING_REQUIRE": 1.0, "SCHED_LOOP_RATE": 200.0,
                   "RTL_AUTOLAND": 2.0}
FIRMWARE_OVERRIDE = {"Q_RTL_MODE": (0.0, "ArduPlane 4.8.0-dev PreArm rejects RTL_AUTOLAND + Q_RTL_MODE together")}


def param_file(p: Path) -> dict:
    d = {}
    for ln in p.read_text(encoding="utf-8").splitlines():
        ln = ln.strip()
        if ln and not ln.startswith("#") and "," in ln:
            k, v = ln.split(",", 1)
            d[k.strip()] = float(v)
    return d


# ---------- 報告公式（逐行取自 scripts/hv_sitl_report.py）----------
def track(out: Path, uav: str):
    rows = [r for r in csv.DictReader((out / uav / "flight.csv").open(encoding="utf-8"))
            if r["lat"] and r["t_s"]]
    armed = [(r["armed"].strip().lower() in ("true", "1")) if r.get("armed") else None for r in rows]
    return ([float(r["lon"]) for r in rows], [float(r["lat"]) for r in rows],
            [float(r["t_s"]) for r in rows], [float(r["rel_alt_m"]) for r in rows], armed)


def flown_km(lon, lat):
    d = 0.0
    for i in range(1, len(lat)):
        dy = (lat[i] - lat[i - 1]) * 111132.0
        dx = (lon[i] - lon[i - 1]) * 111412.0 * math.cos(math.radians(lat[i]))
        d += math.hypot(dx, dy)
    return d / 1000.0


def touchdown_s(t, alt, armed):
    peak = 0.0
    for ts, a, ar in zip(t, alt, armed):
        peak = max(peak, a)
        if peak > 20.0 and a < 2.0 and ar is False:
            return ts
    return float("nan")


def clock_offset(out: Path, uavs):
    f = out / "release_schedule.json"
    if not f.exists():
        return 0.0
    rel = json.loads(f.read_text(encoding="utf-8"))
    sp = float(rel.get("speedup") or 1.0)
    act = rel.get("actual_release_wall_s") or {}
    offs = []
    for uav in uavs:
        if uav not in act:
            continue
        rows = [r for r in csv.DictReader((out / uav / "flight.csv").open(encoding="utf-8")) if r["t_s"]]
        auto = [r for r in rows if r["mode"] == "AUTO"]
        if auto:
            offs.append(float(auto[0]["t_s"]) - float(act[uav]) * sp)
    if not offs:
        return 0.0
    offs.sort()
    return offs[len(offs) // 2]


def min_separation(series, step_s=2.0, d_sep=100.0, dh_sep=50.0):
    def interp(t, xs, q):
        i, out = 0, []
        for u in q:
            while i + 2 < len(t) and t[i + 1] < u:
                i += 1
            if u < t[0] or u > t[-1]:
                out.append(None)
                continue
            span = t[i + 1] - t[i]
            f = 0.0 if span <= 0 else (u - t[i]) / span
            out.append(xs[i] + f * (xs[i + 1] - xs[i]))
        return out

    uavs = sorted(series)
    lo = min(min(s[0]) for s in series.values())
    hi = max(max(s[0]) for s in series.values())
    grid = [lo + i * step_s for i in range(int((hi - lo) / step_s) + 1)]
    g = {}
    for u in uavs:
        t, lat, lon, alt = series[u]
        g[u] = (interp(t, lat, grid), interp(t, lon, grid), interp(t, alt, grid))
    best, worst_pair, worst_t, best_dv, pairs = float("inf"), None, None, None, {}
    n_viol, n_air = 0, 0
    for i, a in enumerate(uavs):
        for b in uavs[i + 1:]:
            pmin, pmin_dv, pviol = float("inf"), None, 0
            for k, tk in enumerate(grid):
                la, oa, aa = g[a][0][k], g[a][1][k], g[a][2][k]
                lb, ob, ab = g[b][0][k], g[b][1][k], g[b][2][k]
                if None in (la, oa, aa, lb, ob, ab) or aa <= 2.0 or ab <= 2.0:
                    continue
                n_air += 1
                dy = (la - lb) * 111132.0
                dx = (oa - ob) * 111412.0 * math.cos(math.radians(la))
                d = math.hypot(dx, dy)
                dv = abs(aa - ab)
                if d < d_sep and dv < dh_sep:
                    pviol += 1
                    n_viol += 1
                if d < pmin:
                    pmin, pmin_dv = d, dv
                if d < best:
                    best, worst_pair, worst_t, best_dv = d, (a, b), tk, dv
            pairs[f"{a}-{b}"] = {"min_h_m": round(pmin, 1) if pmin < float("inf") else None,
                                 "d_v_at_min_m": round(pmin_dv, 1) if pmin_dv is not None else None,
                                 "n_violations": pviol}
    return {"min_h_m": round(best, 1) if best < float("inf") else None,
            "d_v_at_min_m": round(best_dv, 1) if best_dv is not None else None,
            "pair": worst_pair, "t_s": worst_t, "per_pair": pairs,
            "n_airborne_samples": n_air, "n_violations": n_viol, "step_s": step_s}


def summarize(trial: str):
    out = ART / trial
    mission = json.loads((out / "plan" / "mission_summary.json").read_text(encoding="utf-8"))
    uavs = sorted(mission["uavs"])
    off = clock_offset(out, uavs)
    rows, series = [], {}
    for uav in uavs:
        lon, lat, t, alt, armed = track(out, uav)
        t = [x - off for x in t]
        series[uav] = (t, lat, lon, alt)
        km = flown_km(lon, lat)
        td = touchdown_s(t, alt, armed)
        u = mission["uavs"][uav]
        air = [x for x, a in zip(t, alt) if a > 2.0]
        rows.append({"uav": uav, "clock_offset_s": round(off, 1),
                     "flown_t_takeoff_s": round(air[0], 1) if air else None,
                     "plan_t_takeoff_s": round(u["t_takeoff_s"], 1),
                     "plan_t_land_s": round(u["t_land_s"], 1),
                     "flown_t_land_s": round(td, 1) if td == td else None,
                     "plan_km": round(u["distance_m"] / 1000.0, 2),
                     "flown_km": round(km, 2),
                     "peak_alt_m": round(max(alt) if alt else 0.0, 1),
                     "peak_alt_m_2dp": round(max(alt) if alt else 0.0, 2),
                     "n_samples": len(t),
                     "err_pct": round(100.0 * (km - u["distance_m"] / 1000.0) / max(u["distance_m"] / 1000.0, 1e-6), 1)})
    sep = min_separation(series)
    tot_p = sum(r["plan_km"] for r in rows)
    tot_f = sum(r["flown_km"] for r in rows)
    return {"trial": trial, "rows": rows, "sep": sep,
            "total_plan_km": round(tot_p, 2), "total_flown_km": round(tot_f, 2),
            "total_err_pct": round(100.0 * (tot_f - tot_p) / tot_p, 2)}


def main():
    res = {}

    # ---- 1. 參數清冊 ----
    inv = {}
    for trial in ("HV4SITL15", "HV4SITL16"):
        files = {f.stem: param_file(f) for f in sorted((ART / trial / "plan").glob("*.param"))}
        per = {}
        for uav, fv in files.items():
            want = EXPECT_FALLBACK | fv
            entries = []
            for name in sorted(want):
                exp, src, note = want[name], ("export_file" if name in fv else "EXPECT_FALLBACK"), ""
                if name in FIRMWARE_OVERRIDE:
                    ov, why = FIRMWARE_OVERRIDE[name]
                    note = f"deliberate override: exported {exp:g} -> expect {ov:g}; {why}"
                    exp, src = ov, "FIRMWARE_OVERRIDE"
                entries.append({"param": name, "expected": exp, "source": src, "note": note})
            per[uav] = {"n_file": len(fv), "n_compared": len(entries), "entries": entries}
        inv[trial] = {"n_aircraft": len(files), "per_uav": per,
                      "n_file_total": sum(v["n_file"] for v in per.values()),
                      "n_compared_total": sum(v["n_compared"] for v in per.values())}
    res["inventory"] = inv

    # ---- 2. 實際 CSV 的來源分解 ----
    csvbrk = {}
    for trial in sorted(p.name for p in ART.glob("HV4SITL*") if p.is_dir()):
        f = ART / trial / "params_readback.csv"
        if not f.exists():
            continue
        rows = list(csv.DictReader(f.open(encoding="utf-8")))
        ok = sum(r["ok"].lower() == "true" for r in rows)
        trunc = [f'{r["uav"]}.{r["param"]}({r["expected"]}->{r["actual"]})'
                 for r in rows if "整數型別" in (r.get("note") or "")]
        csvbrk[trial] = {"rows": len(rows), "ok": ok,
                         "has_source_column": "source" in (rows[0] if rows else {}),
                         "uavs": sorted({r["uav"] for r in rows}),
                         "n_params": len({r["param"] for r in rows}),
                         "by_source": dict(Counter(r.get("source", "(no source column)") for r in rows)),
                         "int_truncated": trunc,
                         "override_rows": [f'{r["uav"]}.{r["param"]}' for r in rows
                                           if r.get("source") == "FIRMWARE_OVERRIDE"]}
    res["readback_csv"] = csvbrk

    # ---- 3. 航次歸屬：重算 15（驗證公式）與 16（本來就沒算過）----
    flights = {}
    for trial in ("HV4SITL15", "HV4SITL16"):
        try:
            flights[trial] = summarize(trial)
        except Exception as e:                      # noqa: BLE001
            flights[trial] = {"error": f"{type(e).__name__}: {e}"}
    stored = json.loads((ART / "HV4SITL15" / "flight_summary.json").read_text(encoding="utf-8"))
    recon = []
    for s in stored:
        mine = next((r for r in flights["HV4SITL15"]["rows"] if r["uav"] == s["uav"]), None)
        recon.append({"uav": s["uav"],
                      "stored": {k: s.get(k) for k in ("flown_km", "peak_alt_m", "err_pct", "flown_t_land_s")},
                      "recomputed": {k: mine.get(k) for k in ("flown_km", "peak_alt_m", "err_pct", "flown_t_land_s")},
                      "match": all(s.get(k) == mine.get(k) for k in ("flown_km", "peak_alt_m", "err_pct"))})
    sep_stored = json.loads((ART / "HV4SITL15" / "separation_from_tracks.json").read_text(encoding="utf-8"))
    recon_sep = {"stored_min_h_m": sep_stored["min_h_m"],
                 "recomputed_min_h_m": flights["HV4SITL15"]["sep"]["min_h_m"],
                 "match": sep_stored["min_h_m"] == flights["HV4SITL15"]["sep"]["min_h_m"]}
    res["flights"] = flights
    res["method_check_on_HV4SITL15"] = {"per_uav": recon, "separation": recon_sep}

    (HERE / "c4_evidence.json").write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")

    L = []
    L.append("== 1. export file vs comparison set ==")
    for t, d in inv.items():
        L.append(f"{t}: {d['n_aircraft']} aircraft; export total {d['n_file_total']}; compared total {d['n_compared_total']}")
        for u, v in d["per_uav"].items():
            L.append(f"   {u}: exported {v['n_file']} -> compared {v['n_compared']}")
    L.append("")
    L.append("== 2. params_readback.csv per sortie ==")
    for t, d in csvbrk.items():
        L.append(f"{t:10s} {d['ok']}/{d['rows']}  {d['n_params']} params x {len(d['uavs'])} ac {d['uavs']}  source {d['by_source']}")
        if d["int_truncated"]:
            L.append(f"           int-truncated: {d['int_truncated']}")
        if d["override_rows"]:
            L.append(f"           fw-override:   {d['override_rows']}")
    L.append("")
    L.append("== 3. method self-check on HV4SITL15 (vs its stored flight_summary.json) ==")
    for r in recon:
        L.append(f"  {r['uav']}: stored={r['stored']}  recomputed={r['recomputed']}  match={r['match']}")
    L.append(f"  separation: {recon_sep}")
    L.append("")
    L.append("== 4. HV4SITL16 (has the 102/102 gate + flight.csv, never reported) recomputed ==")
    for t in ("HV4SITL15", "HV4SITL16"):
        f = flights[t]
        if "error" in f:
            L.append(f"  {t}: {f['error']}")
            continue
        L.append(f"  {t}: plan {f['total_plan_km']} km / flown {f['total_flown_km']} km ({f['total_err_pct']:+.2f}%)"
                 f"  min-h {f['sep']['min_h_m']} m (dv {f['sep']['d_v_at_min_m']} m, {f['sep']['pair']}), violations {f['sep']['n_violations']}")
        for r in f["rows"]:
            L.append(f"     {r['uav']}: plan {r['plan_km']} / flown {r['flown_km']} km ({r['err_pct']:+.1f}%)"
                     f"  peak {r['peak_alt_m_2dp']} m  land {r['flown_t_land_s']} (plan {r['plan_t_land_s']})  n={r['n_samples']}")
        L.append(f"     per-pair: {f['sep']['per_pair']}")
    txt = "\n".join(L)
    (HERE / "c4_summary.txt").write_text(txt, encoding="utf-8")
    print(txt)


if __name__ == "__main__":
    sys.exit(main())
