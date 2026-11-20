# -*- coding: utf-8 -*-
"""label update_paper / stage 2 -- every number that goes into a table.

Altitude, timing, distance, separation, wind kinematics, EKF wind, coverage,
exported-waypoint altitude census, parameter read-back -- all recomputed here
from my own .npz cache (up_extract.py) and from the trials' own plan/ files.

  python up_metrics.py            # all trials -> out/metrics.json
"""
from __future__ import annotations

import csv
import itertools
import math
import sys
from pathlib import Path

import numpy as np
import shapely
from shapely.geometry import Polygon
from shapely.ops import unary_union

from up_lib import (AIRBORNE_M, CEIL_LEGAL_M, CEIL_OPS_M, D_SEP_H, D_SEP_V, OUT,
                    ROOT, UAVS, airborne_window, enu, first_auto_t, gps_offset,
                    jdump, jload, load, m_per_deg, path_len_enu, read_waypoints,
                    time_above, vec_to_from)

TRIALS = ("HV4W55", "HV4W55R", "HV4W55S", "HV4W05", "HV4W00", "BEAMIDP1",
          "HV4SITL15", "HV4SITL16")
SURVEY_AGL_M = 100.0
CRUISE_AGL_M = 100.0
CRUISE_AS_MIN = 10.0


# ------------------------------------------------------------------ altitude
def altitude(trial: str) -> dict:
    run = ROOT / "runs" / "artifacts" / trial
    out = {"per_uav": {}}
    for u in UAVS:
        p = load(trial, u)["pos"]
        t, rel = p[:, 0], p[:, 4]
        ta, tb = airborne_window(t, rel)
        m = (t >= ta) & (t <= tb)
        wp = read_waypoints(run / "plan" / f"{u}.waypoints")
        relwp = [w for w in wp if w["frame"] == 3]
        out["per_uav"][u] = {
            "n_pos": int(len(t)),
            "rate_hz": round(float(len(t) / (t[-1] - t[0])), 3),
            "flown_max_rel_m": round(float(rel[m].max()), 4),
            "flown_max_rel_all_m": round(float(rel.max()), 4),
            "t_above_120_s": round(time_above(t[m], rel[m], CEIL_OPS_M), 3),
            "t_above_legal_s": round(time_above(t[m], rel[m], CEIL_LEGAL_M), 3),
            "margin_to_120_m": round(CEIL_OPS_M - float(rel[m].max()), 4),
            "exported_n_rel_items": len(relwp),
            "exported_max_rel_m": round(max(w["alt"] for w in relwp), 4),
            "exported_n_above_120": sum(1 for w in relwp if w["alt"] > CEIL_OPS_M),
            "tracking_excess_m": round(float(rel[m].max()) - max(w["alt"] for w in relwp), 4),
        }
    pu = out["per_uav"]
    out["fleet_flown_max_rel_m"] = round(max(v["flown_max_rel_m"] for v in pu.values()), 4)
    out["fleet_margin_to_120_m"] = round(CEIL_OPS_M - out["fleet_flown_max_rel_m"], 4)
    out["fleet_t_above_120_s"] = round(sum(v["t_above_120_s"] for v in pu.values()), 3)
    out["fleet_exported_max_rel_m"] = round(max(v["exported_max_rel_m"] for v in pu.values()), 4)
    out["fleet_exported_n_above_120"] = sum(v["exported_n_above_120"] for v in pu.values())
    out["fleet_exported_n_rel_items"] = sum(v["exported_n_rel_items"] for v in pu.values())
    return out


# -------------------------------------------------------------------- timing
def timing(trial: str) -> dict:
    run = ROOT / "runs" / "artifacts" / trial
    ms = jload(run / "plan" / "mission_summary.json")
    lat0, lon0 = float(ms["home"]["lat_deg"]), float(ms["home"]["lon_deg"])
    delays = {u: float(ms["uavs"][u]["t_takeoff_s"]) for u in UAVS}
    zero_uav = min(delays, key=lambda u: delays[u])
    off, auto_c, land_c, rows = {}, {}, {}, {}
    for u in UAVS:
        d = load(trial, u)
        off[u] = gps_offset(d)
        p = d["pos"]
        ta, tb = airborne_window(p[:, 0], p[:, 4])
        auto_c[u] = first_auto_t(d) + off[u]
        land_c[u] = tb + off[u]
    t0 = auto_c[zero_uav] - delays[zero_uav]
    for u in UAVS:
        pl = ms["uavs"][u]
        p = load(trial, u)["pos"]
        ta, tb = airborne_window(p[:, 0], p[:, 4])
        air = (p[:, 0] >= ta) & (p[:, 0] <= tb)
        km = path_len_enu(p[air, 1], p[air, 2], lat0, lon0) / 1000.0
        rows[u] = {
            "cls": pl["cls"],
            "plan_t_takeoff_s": pl["t_takeoff_s"],
            "flown_t_release_s": round(auto_c[u] - t0, 3),
            "plan_t_land_s": round(float(pl["t_land_s"]), 4),
            "flown_t_land_2m_s": round(land_c[u] - t0, 3),
            "land_err_pct": round(100.0 * ((land_c[u] - t0) / float(pl["t_land_s"]) - 1.0), 3),
            "airborne_s": round(tb - ta, 3),
            "plan_km": round(float(pl["distance_m"]) / 1000.0, 4),
            "flown_km": round(km, 4),
            "dist_err_pct": round(100.0 * (km * 1000.0 / float(pl["distance_m"]) - 1.0), 3),
            "plan_n_items": pl["n_items"],
            "gps_offset_s": round(off[u], 4),
        }
    plan_mk = float(ms["makespan_s"])
    mk = max(r["flown_t_land_2m_s"] for r in rows.values())
    return {"fleet_zero_uav": zero_uav,
            "planned_makespan_s": plan_mk,
            "flown_makespan_2m_s": round(mk, 3),
            "makespan_delta_s": round(mk - plan_mk, 3),
            "makespan_err_pct": round(100.0 * (mk / plan_mk - 1.0), 3),
            "planned_total_km": round(sum(r["plan_km"] for r in rows.values()), 4),
            "flown_total_km": round(sum(r["flown_km"] for r in rows.values()), 4),
            "total_dist_delta_km": round(sum(r["flown_km"] for r in rows.values())
                                         - sum(r["plan_km"] for r in rows.values()), 4),
            "total_dist_err_pct": round(100.0 * (sum(r["flown_km"] for r in rows.values())
                                                 / sum(r["plan_km"] for r in rows.values()) - 1.0), 3),
            "per_uav": rows}


# ---------------------------------------------------------------- separation
def _track(trial, uav, lat0, lon0, frame):
    d = load(trial, uav)
    p = d["pos"]
    o = gps_offset(d)
    if frame == "ecef":
        x, y = enu(p[:, 1], p[:, 2], lat0, lon0)
    else:
        ml, mo = m_per_deg(lat0, frame)
        x, y = (p[:, 2] - lon0) * mo, (p[:, 1] - lat0) * ml
    return p[:, 0] + o, np.asarray(x), np.asarray(y), p[:, 4]


def _pair_min(t1, x1, y1, z1, t2, x2, y2, z2, grid_s=None):
    lo, hi = max(t1[0], t2[0]), min(t1[-1], t2[-1])
    if hi <= lo:
        return {"n_samples": 0, "min_h_m": None, "note": "no overlapping span"}
    if grid_s:
        tu = np.arange(lo, hi + 1e-9, grid_s)
    else:
        tu = np.unique(np.concatenate([[lo], t1[(t1 >= lo) & (t1 <= hi)],
                                       t2[(t2 >= lo) & (t2 <= hi)], [hi]]))
    a = [np.interp(tu, t1, v) for v in (x1, y1, z1)]
    b = [np.interp(tu, t2, v) for v in (x2, y2, z2)]
    air = (a[2] > AIRBORNE_M) & (b[2] > AIRBORNE_M)
    if air.sum() < 2:
        return {"n_samples": int(air.sum()), "min_h_m": None,
                "note": "never airborne simultaneously"}
    dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    best = {"h": math.inf}
    n_viol = 0
    for i in np.flatnonzero(air[:-1] & air[1:]):
        p0 = np.array([dx[i], dy[i]])
        dd = np.array([dx[i + 1] - dx[i], dy[i + 1] - dy[i]])
        den = float(dd @ dd)
        cand = {0.0, 1.0}
        if den > 0:
            cand.add(float(np.clip(-(p0 @ dd) / den, 0.0, 1.0)))
        for s in cand:
            h = float(np.hypot(*(p0 + s * dd)))
            v = float(abs(dz[i] + s * (dz[i + 1] - dz[i])))
            if h < best["h"]:
                best = {"h": h, "v": v, "t": float(tu[i] + s * (tu[i + 1] - tu[i]))}
            if h < D_SEP_H and v < D_SEP_V:
                n_viol += 1
    return {"n_samples": int(air.sum()), "min_h_m": round(best["h"], 4),
            "vert_at_min_m": round(best["v"], 4), "n_violation_intervals": int(n_viol)}


def separation(trial: str) -> dict:
    run = ROOT / "runs" / "artifacts" / trial
    ms = jload(run / "plan" / "mission_summary.json")
    lat0, lon0 = float(ms["home"]["lat_deg"]), float(ms["home"]["lon_deg"])
    out = {"frames": {}}
    for frame in ("ecef", "planner", "repo"):
        tr = {u: _track(trial, u, lat0, lon0, frame) for u in UAVS}
        pairs = {}
        for a, b in itertools.combinations(UAVS, 2):
            pairs[f"{a}-{b}"] = _pair_min(*tr[a], *tr[b])
        got = [v for v in pairs.values() if v.get("min_h_m") is not None]
        out["frames"][frame] = {
            "pairs": pairs,
            "min_h_m": round(min(v["min_h_m"] for v in got), 4) if got else None,
            "binding_pair": min(got, key=lambda v: v["min_h_m"])["min_h_m"] if got else None,
            "n_violations": sum(v.get("n_violation_intervals", 0) for v in pairs.values()),
            "n_paired_samples": sum(v.get("n_samples", 0) for v in pairs.values()),
        }
        if got:
            out["frames"][frame]["binding_pair"] = min(
                pairs, key=lambda k: pairs[k].get("min_h_m") or math.inf)
    v = jload(run / "validation.json")
    out["planned_min_sep_h_m"] = v.get("min_sep_h_m")
    rep = run / "separation_from_tracks.json"
    if rep.exists():
        r = jload(rep)
        out["reporter_min_sep_h_m"] = r.get("min_sep_h_m") or r.get("min_h_m")
        out["reporter_raw"] = {k: r[k] for k in list(r)[:12]}
    return out


# ------------------------------------------------------------ wind kinematics
def windaxis(trial: str) -> dict:
    """median of (GPS groundspeed - interpolated true airspeed) per heading bin."""
    res = {}
    for u in UAVS:
        d = load(trial, u)
        g, a, p = d["gps"], d["arsp"], d["pos"]
        rel = np.interp(g[:, 0], p[:, 0], p[:, 4])
        asp = np.interp(g[:, 0], a[:, 0], a[:, 1])
        ok = (rel > CRUISE_AGL_M) & (asp > CRUISE_AS_MIN)
        crs, dv = g[ok, 4] % 360.0, g[ok, 3] - asp[ok]
        bins = {}
        for name, c in (("E", 90.0), ("W", 270.0), ("N", 0.0), ("S", 180.0)):
            dd = np.abs((crs - c + 180.0) % 360.0 - 180.0)
            sel = dd <= 45.0
            bins[name] = {"n": int(sel.sum()),
                          "median": round(float(np.median(dv[sel])), 4) if sel.sum() else None}
        ew = (bins["E"]["median"] - bins["W"]["median"]
              if bins["E"]["median"] is not None and bins["W"]["median"] is not None else None)
        ns = (bins["N"]["median"] - bins["S"]["median"]
              if bins["N"]["median"] is not None and bins["S"]["median"] is not None else None)
        halfsum_ew = ((bins["E"]["median"] + bins["W"]["median"]) / 2.0
                      if ew is not None else None)
        # EKF wind, median of N/E components over the cruise band
        w = d["xkf2"] if "xkf2" in d and len(d["xkf2"]) else d.get("nkf2")
        ekf = None
        if w is not None and len(w):
            relw = np.interp(w[:, 0], p[:, 0], p[:, 4])
            sel = relw > CRUISE_AGL_M
            if sel.sum() > 10:
                spd, frm = vec_to_from(float(np.median(w[sel, 1])), float(np.median(w[sel, 2])))
                ekf = {"spd_mps": round(spd, 4), "from_deg": round(frm, 2),
                       "n": int(sel.sum())}
        res[u] = {"bins": bins, "east_minus_west": round(ew, 4) if ew is not None else None,
                  "north_minus_south": round(ns, 4) if ns is not None else None,
                  "ew_half_sum": round(halfsum_ew, 4) if halfsum_ew is not None else None,
                  "ekf_wind": ekf}
    return res


# ------------------------------------------------------------------ coverage
def coverage(trial: str, survey_only: bool = False) -> list:
    run = ROOT / "runs" / "artifacts" / trial
    sc = jload(run / "scenario.json")
    ms = jload(run / "plan" / "mission_summary.json")
    lat0, lon0 = float(ms["home"]["lat_deg"]), float(ms["home"]["lon_deg"])
    swath = 2.0 * float(sc["altitude_m"]) * math.tan(math.radians(float(sc["hfov_deg"]) / 2.0))
    lines = []
    for u in UAVS:
        p = load(trial, u)["pos"]
        ta, tb = airborne_window(p[:, 0], p[:, 4])
        m = (p[:, 0] >= ta) & (p[:, 0] <= tb)
        if survey_only:
            m &= p[:, 4] > SURVEY_AGL_M
        e, n = enu(p[m, 1], p[m, 2], lat0, lon0)
        xy = np.column_stack([e, n])
        br = np.flatnonzero(np.diff(p[m, 0]) > 5.0)
        for seg in np.split(xy, br + 1):
            if len(seg) >= 2:
                lines.append(shapely.LineString(seg))
    band = unary_union([ln.buffer(swath / 2.0, cap_style=2) for ln in lines])
    out = []
    for a in sc["areas"]:
        e, n = enu([la for la, lo in a["polygon"]], [lo for la, lo in a["polygon"]], lat0, lon0)
        poly = Polygon(list(zip(e, n)))
        inter = poly.intersection(band).area
        out.append({"area": f"Z{a['area_id']}", "area_m2": round(poly.area, 1),
                    "area_ha": round(poly.area / 1e4, 2),
                    "covered_m2": round(inter, 1),
                    "uncovered_m2": round(poly.area - inter, 1),
                    "ratio": round(inter / poly.area, 6)})
    return {"swath_m": round(swath, 4), "areas": out}


# ---------------------------------------------------------------- parameters
def params(trial: str) -> dict:
    """Re-does the 102/102 and the 'strict vs raw planned value' counts.

    Three comparisons, deliberately separated, because K2 says the published
    "strict 91/96" is not a byte comparison:
      ok_flag        the runner's own verdict column (tolerance max(0.01, 0.1%),
                     against the FIRMWARE-NORMALISED expectation)
      strict_exact   exact float equality between the exported .param value and
                     the read-back value  -> the literal byte comparison
      strict_float32 the same, but both sides rounded to float32 first, which is
                     what the firmware actually stores.
    """
    run = ROOT / "runs" / "artifacts" / trial
    rows = list(csv.DictReader((run / "params_readback.csv").open(encoding="utf-8")))
    got_col = "actual" if rows and "actual" in rows[0] else "got"
    n_ok = sum(1 for r in rows if str(r.get("ok", "")).strip().lower() in ("true", "1"))
    exported = {}
    for u in UAVS:
        exported[u] = {}
        for ln in (run / "plan" / f"{u}.param").read_text(encoding="utf-8").splitlines():
            ln = ln.split("#")[0].strip()
            if not ln:
                continue
            parts = ln.replace(",", " ").split()
            if len(parts) >= 2:
                exported[u][parts[0]] = parts[1]
    strict_exact, strict_f32, tot = 0, 0, 0
    f32_only, real_diff = [], []
    for r in rows:
        u, nm = r.get("uav"), r.get("param")
        if u not in exported or nm not in exported[u]:
            continue
        tot += 1
        want_s = exported[u][nm]
        want = float(want_s)
        try:
            got = float(r.get(got_col))
        except (TypeError, ValueError):
            real_diff.append({"uav": u, "param": nm, "exported": want_s,
                              "readback": r.get(got_col), "why": "unparsable"})
            continue
        if got == want:
            strict_exact += 1
            strict_f32 += 1
        elif np.float32(want) == np.float32(got):
            strict_f32 += 1
            f32_only.append({"uav": u, "param": nm, "exported": want_s,
                             "readback": r.get(got_col)})
        else:
            real_diff.append({"uav": u, "param": nm, "exported": want_s,
                              "readback": r.get(got_col),
                              "source": r.get("source"), "ok": r.get("ok")})
    src = {}
    for r in rows:
        src[r.get("source", "")] = src.get(r.get("source", ""), 0) + 1
    return {"n_rows": len(rows), "n_ok": n_ok, "readback_column": got_col,
            "source_counts": src,
            "n_exported_per_uav": {u: len(v) for u, v in exported.items()},
            "strict_exact": f"{strict_exact}/{tot}",
            "strict_float32": f"{strict_f32}/{tot}",
            "n_float32_roundtrip_only": len(f32_only),
            "float32_only_rows": f32_only,
            "genuinely_different_rows": real_diff}


def main() -> None:
    sel = sys.argv[1:] or list(TRIALS)
    f = OUT / "metrics.json"
    res = jload(f) if f.exists() else {}
    for trial in sel:
        e = {"altitude": altitude(trial), "timing": timing(trial),
             "separation": separation(trial), "windaxis": windaxis(trial),
             "coverage": coverage(trial), "params": params(trial)}
        res[trial] = e
        a, t, s = e["altitude"], e["timing"], e["separation"]
        print(f"{trial:10s} maxAGL {a['fleet_flown_max_rel_m']:9.4f} "
              f"t>120 {a['fleet_t_above_120_s']:7.2f}s  expAbove {a['fleet_exported_n_above_120']:3d}  "
              f"mk {t['planned_makespan_s']:9.3f}->{t['flown_makespan_2m_s']:9.3f} "
              f"({t['makespan_err_pct']:+6.3f}%)  km {t['planned_total_km']:7.3f}->{t['flown_total_km']:7.3f}  "
              f"sep {s['frames']['ecef']['min_h_m']}  viol {s['frames']['ecef']['n_violations']}")
    jdump(f, res)


if __name__ == "__main__":
    main()
