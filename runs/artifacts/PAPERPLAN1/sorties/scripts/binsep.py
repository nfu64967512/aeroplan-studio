# -*- coding: utf-8 -*-
"""Minimum inter-aircraft separation recomputed from DATAFLASH, not telemetry.

Why this exists: hv_sitl_report.py interpolates the ~1 Hz telemetry onto a 2.0 s
grid, so its min_h_m is an UPPER bound (its own docstring says so).  HV4W05 came
out at 100.8 m -- only 0.8 m above the 100 m floor -- so the bound is not good
enough to stand behind in a paper.

POS in the dataflash runs at ~6 Hz.  The three SITL instances each count TimeUS
from their own boot, so they are aligned here on GPS time-of-week (GPS.GWk/GMS),
which is a genuinely common clock, NOT on per-instance TimeUS.

Violation, per the vendored rule the reporter uses: horizontal < 100 m AND
vertical < 50 m, evaluated only where both aircraft are above 2 m AGL.

    python binsep.py <TRIAL> <lognum>

Writes <TRIAL>_binsep.json next to the other sortie evidence.
"""
from __future__ import annotations

import json
import math
import statistics
import sys
from pathlib import Path

from pymavlink import mavutil

ROOT = Path(r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
OUT = ROOT / "runs" / "artifacts" / "PAPERPLAN1" / "sorties"
INST = {"A1": 100, "B2": 101, "B3": 102}
M_LAT = 111132.0
D_SEP_H, D_SEP_V, AIRBORNE_AGL = 100.0, 50.0, 2.0


def read_track(path: Path):
    """-> (gps_t[], lat[], lon[], agl[]) with gps_t = GPS seconds of week."""
    m = mavutil.mavlink_connection(str(path), dialect="ardupilotmega")
    pos, offs = [], []
    while True:
        msg = m.recv_match()
        if msg is None:
            break
        ty = msg.get_type()
        if ty == "POS":
            a = getattr(msg, "RelHomeAlt", None)
            if a is not None:
                pos.append((float(msg.TimeUS) / 1e6, float(msg.Lat),
                            float(msg.Lng), float(a)))
        elif ty == "GPS":
            gms, gwk = getattr(msg, "GMS", None), getattr(msg, "GWk", None)
            if gms and gwk:
                # GPS time of week (s) minus this instance's own boot clock (s)
                offs.append(gwk * 604800.0 + gms / 1000.0 - float(msg.TimeUS) / 1e6)
    if not pos or not offs:
        raise SystemExit(f"{path}: no POS/GPS")
    off = statistics.median(offs)
    spread = max(offs) - min(offs)
    pos.sort()
    return ([p[0] + off for p in pos], [p[1] for p in pos],
            [p[2] for p in pos], [p[3] for p in pos], off, spread, len(offs))


def interp_at(t, xs, q):
    """Linear interpolation of xs(t) at the sorted query times q; None outside."""
    out, i, n = [], 0, len(t)
    for u in q:
        if u < t[0] or u > t[-1]:
            out.append(None)
            continue
        while i + 1 < n - 1 and t[i + 1] < u:
            i += 1
        while i > 0 and t[i] > u:
            i -= 1
        span = t[i + 1] - t[i]
        f = 0.0 if span <= 0 else (u - t[i]) / span
        out.append(xs[i] + f * (xs[i + 1] - xs[i]))
    return out


def main():
    trial, lognum = sys.argv[1], sys.argv[2]
    tracks, meta = {}, {}
    for uav, i in INST.items():
        p = ROOT / f"sitl/vtol/i{i}/logs/{int(lognum):08d}.BIN"
        t, la, lo, ag, off, spread, ngps = read_track(p)
        tracks[uav] = (t, la, lo, ag)
        dts = [b - a for a, b in zip(t, t[1:]) if 0 < b - a < 5]
        meta[uav] = {"log": p.name, "n_pos": len(t),
                     "median_dt_s": round(statistics.median(dts), 4),
                     "p95_dt_s": round(sorted(dts)[int(0.95 * (len(dts) - 1))], 4),
                     "max_dt_s": round(max(dts), 4),
                     "gps_offset_s": round(off, 4),
                     "gps_offset_spread_s": round(spread, 4), "n_gps": ngps,
                     "t_span_s": round(t[-1] - t[0], 1)}
        print(f"  {uav}: {len(t)} POS @ {meta[uav]['median_dt_s']}s "
              f"(p95 {meta[uav]['p95_dt_s']}s), clock spread {spread * 1e3:.1f} ms")

    pairs, worst = {}, None
    names = sorted(tracks)
    for ai in range(len(names)):
        for bi in range(ai + 1, len(names)):
            A, B = names[ai], names[bi]
            ta, laa, loa, aga = tracks[A]
            tb, lab, lob, agb = tracks[B]
            lo_t, hi_t = max(ta[0], tb[0]), min(ta[-1], tb[-1])
            # evaluate at EVERY native sample of both aircraft in the overlap
            q = sorted(u for u in list(ta) + list(tb) if lo_t <= u <= hi_t)
            La, Lo, Ag = (interp_at(ta, laa, q), interp_at(ta, loa, q),
                          interp_at(ta, aga, q))
            Lb, Ob, Bg = (interp_at(tb, lab, q), interp_at(tb, lob, q),
                          interp_at(tb, agb, q))
            best, nv, n_air = None, 0, 0
            for k, u in enumerate(q):
                if None in (La[k], Lb[k], Ag[k], Bg[k]):
                    continue
                if Ag[k] <= AIRBORNE_AGL or Bg[k] <= AIRBORNE_AGL:
                    continue
                n_air += 1
                dy = (La[k] - Lb[k]) * M_LAT
                dx = (Lo[k] - Ob[k]) * 111412.0 * math.cos(math.radians(La[k]))
                h = math.hypot(dx, dy)
                v = abs(Ag[k] - Bg[k])
                if h < D_SEP_H and v < D_SEP_V:
                    nv += 1
                if best is None or h < best[0]:
                    best = (h, v, u)
            if best is None:
                # Staggered releases: these two are never airborne at the same
                # time, so they have no separation to measure.  That is a real
                # outcome, not a failure -- record it and move on.
                pairs[f"{A}-{B}"] = {"min_h_m": None, "d_v_at_min_m": None,
                                     "t_gps_s": None, "n_airborne_samples": 0,
                                     "n_violations": 0,
                                     "note": "never airborne simultaneously"}
                print(f"  {A}-{B}: never airborne at the same time "
                      f"(no separation to measure)")
                continue
            pairs[f"{A}-{B}"] = {
                "min_h_m": round(best[0], 3), "d_v_at_min_m": round(best[1], 3),
                "t_gps_s": round(best[2], 2), "n_airborne_samples": n_air,
                "n_violations": nv}
            print(f"  {A}-{B}: min_h {best[0]:.2f} m (dv {best[1]:.2f} m), "
                  f"{n_air} airborne samples, {nv} violations")
            if worst is None or best[0] < worst[1]["min_h_m"]:
                worst = (f"{A}-{B}", pairs[f"{A}-{B}"])

    res = {"trial": trial, "lognum": int(lognum), "source": "dataflash POS + GPS ToW",
           "rule": {"d_sep_h_m": D_SEP_H, "dh_sep_v_m": D_SEP_V,
                    "airborne_agl_m": AIRBORNE_AGL,
                    "violation": "horizontal < 100 AND vertical < 50"},
           "per_uav": meta, "pairs": pairs,
           "min_h_m": None if worst is None else worst[1]["min_h_m"],
           "pair": None if worst is None else worst[0],
           "d_v_at_min_m": None if worst is None else worst[1]["d_v_at_min_m"],
           "pairs_never_concurrent": [k for k, v in pairs.items()
                                      if v["min_h_m"] is None],
           "n_violations": sum(p["n_violations"] for p in pairs.values()),
           "caveat": ("evaluated at every native POS sample of both aircraft "
                      "(no 2 s coarsening); residual error is only the "
                      "between-sample chord at ~6 Hz")}
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"{trial}_binsep.json").write_text(
        json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"=> {trial}: min_h {res['min_h_m']} m on {res['pair']}, "
          f"{res['n_violations']} violations")


if __name__ == "__main__":
    main()
