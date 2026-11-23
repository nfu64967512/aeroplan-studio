# -*- coding: utf-8 -*-
"""Full-rate flown altitude + EKF wind + airspeed/groundspeed from dataflash.

The 1 Hz flight.csv can miss the peak, so H2 ("flown max AGL < 120.000 m") is
decided here, on the POS.RelHomeAlt stream (~6 Hz).  Run IMMEDIATELY after a
sortie: sitl/vtol/i10x/logs/ is overwritten by the next one.

    python binalt.py <TRIAL>            # uses each instance's LASTLOG.TXT
    python binalt.py <TRIAL> 15         # or an explicit log number

Writes runs/artifacts/PAPERPLAN1/sorties/<TRIAL>_binalt.json and copies nothing.
"""
from __future__ import annotations

import collections
import json
import statistics
import sys
from pathlib import Path

from pymavlink import mavutil

ROOT = Path(r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
OUT = ROOT / "runs" / "artifacts" / "PAPERPLAN1" / "sorties"
INST = {"A1": 100, "B2": 101, "B3": 102}


def main():
    trial = sys.argv[1]
    forced = sys.argv[2] if len(sys.argv) > 2 else None
    res = {"trial": trial, "per_uav": {}}
    for uav, i in INST.items():
        logdir = ROOT / f"sitl/vtol/i{i}/logs"
        n = forced or (logdir / "LASTLOG.TXT").read_text().strip()
        p = logdir / f"{int(n):08d}.BIN"
        if not p.exists():
            res["per_uav"][uav] = {"error": f"missing {p}"}
            continue
        m = mavutil.mavlink_connection(str(p), dialect="ardupilotmega")
        inv = collections.Counter()
        pos, wind, arsp = [], [], []
        while True:
            msg = m.recv_match()
            if msg is None:
                break
            ty = msg.get_type()
            inv[ty] += 1
            if ty == "POS":
                a = getattr(msg, "RelHomeAlt", None)
                if a is not None:
                    pos.append((float(msg.TimeUS) / 1e6, float(a)))
            elif ty == "NKF2" or ty == "XKF2":
                # EKF wind estimate, NE components (m/s)
                vwn, vwe = getattr(msg, "VWN", None), getattr(msg, "VWE", None)
                if vwn is not None and vwe is not None:
                    wind.append((float(msg.TimeUS) / 1e6, float(vwn), float(vwe)))
            elif ty == "ARSP":
                v = getattr(msg, "Airspeed", None)
                if v is not None:
                    arsp.append((float(msg.TimeUS) / 1e6, float(v)))
        pos.sort()
        if not pos:
            res["per_uav"][uav] = {"error": "no POS messages", "log": str(p)}
            continue
        ts = [x[0] for x in pos]
        al = [x[1] for x in pos]
        dts = [b - a for a, b in zip(ts, ts[1:]) if 0 < b - a < 5]
        hz = len(dts) / sum(dts) if dts else 0.0
        k = max(range(len(al)), key=lambda j: al[j])
        t_over = 0.0
        for (t0, a0), (t1, a1) in zip(pos, pos[1:]):
            dt = t1 - t0
            if dt <= 0 or dt > 2.0:
                continue
            if a0 > 120.0 and a1 > 120.0:
                t_over += dt
            elif (a0 > 120.0) != (a1 > 120.0) and a1 != a0:
                t_over += dt * (max(a0, a1) - 120.0) / abs(a1 - a0)
        srt = sorted(al)
        row = {
            "log": str(p.relative_to(ROOT)).replace("\\", "/"),
            "log_bytes": p.stat().st_size,
            "n_pos": len(pos), "pos_rate_hz": round(hz, 2),
            "max_rel_home_alt_m": round(al[k], 4),
            "t_at_max_s": round(ts[k], 1),
            "excess_over_120_m": round(al[k] - 120.0, 4),
            "time_above_120_s": round(t_over, 2),
            "p99_m": round(srt[int(0.99 * (len(srt) - 1))], 3),
            "p999_m": round(srt[int(0.999 * (len(srt) - 1))], 3),
            "log_span_s": round(ts[-1] - ts[0], 1),
        }
        # EKF wind: median over the cruise half of the log, magnitude + FROM-direction
        if wind:
            cruise = [w for w in wind if w[0] > ts[0] + 0.25 * (ts[-1] - ts[0])]
            cruise = cruise or wind
            import math
            mag = [math.hypot(w[1], w[2]) for w in cruise]
            # VWN/VWE = wind velocity NE (the direction the air is going TO);
            # meteorological FROM-direction = atan2(-VWE, -VWN)
            frm = [math.degrees(math.atan2(-w[2], -w[1])) % 360.0 for w in cruise]
            # circular median via the unit-vector mean
            sx = sum(math.cos(math.radians(a)) for a in frm) / len(frm)
            sy = sum(math.sin(math.radians(a)) for a in frm) / len(frm)
            row["ekf_wind"] = {
                "n": len(cruise),
                "median_speed_mps": round(statistics.median(mag), 3),
                "mean_from_dir_deg": round(math.degrees(math.atan2(sy, sx)) % 360.0, 1),
                "median_VWN_mps": round(statistics.median([w[1] for w in cruise]), 3),
                "median_VWE_mps": round(statistics.median([w[2] for w in cruise]), 3),
                "source": "NKF2/XKF2 VWN,VWE",
            }
        row["msg_inventory_top"] = dict(inv.most_common(10))
        res["per_uav"][uav] = row

    vals = [v["max_rel_home_alt_m"] for v in res["per_uav"].values()
            if "max_rel_home_alt_m" in v]
    res["fleet_max_rel_alt_m"] = max(vals) if vals else None
    res["H2_below_120_pass"] = bool(vals) and all(v < 120.0 for v in vals)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"{trial}_binalt.json").write_text(
        json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps(res, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
