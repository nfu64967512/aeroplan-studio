# -*- coding: utf-8 -*-
"""Stage 1: dataflash .BIN -> compact per-(trial,uav) .npz cache.

Independent of scripts/hv_sitl_report.py, binalt.py and binsep.py: this file
re-reads the raw logs and keeps every stream the analysis needs, at native rate.

  python ana_extract.py            # all trials in TRIALS
  python ana_extract.py HV4W55     # one

Log<->trial mapping is asserted here from two independent sources:
  * LASTLOG-era numbering recorded in the runbook (P6: LASTLOG=14 before the set)
  * .BIN mtime vs the sortie wall-clock epochs in ../../sorties/logs/<TRIAL>.t0/.t1
"""
from __future__ import annotations

import collections
import json
import sys
from pathlib import Path

import numpy as np
from pymavlink import mavutil

ROOT = Path(r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
HERE = Path(__file__).resolve().parent
CACHE = HERE.parent / "cache"
INST = {"A1": 100, "B2": 101, "B3": 102}
TRIALS = {"HV4W55": 15, "HV4W05": 16, "HV4W00": 17, "BEAMIDP1": 18,
          "HV4SITL15": 13, "HV4SITL16": 14}

WANT = ["POS", "GPS", "ARSP", "XKF2", "NKF2", "ATT", "MODE", "EV", "STAT", "SIM"]


def extract(trial: str, lognum: int, uav: str) -> dict:
    p = ROOT / f"sitl/vtol/i{INST[uav]}/logs/{lognum:08d}.BIN"
    m = mavutil.mavlink_connection(str(p), dialect="ardupilotmega")
    acc = collections.defaultdict(list)
    n = collections.Counter()
    while True:
        msg = m.recv_match(type=WANT)
        if msg is None:
            break
        ty = msg.get_type()
        n[ty] += 1
        t = float(msg.TimeUS) / 1e6
        if ty == "POS":
            acc["pos"].append((t, float(msg.Lat), float(msg.Lng),
                               float(msg.Alt), float(msg.RelHomeAlt)))
        elif ty == "GPS":
            acc["gps"].append((t, float(msg.GWk), float(msg.GMS), float(msg.Spd),
                               float(msg.GCrs), float(msg.Alt), float(msg.Status)))
        elif ty == "ARSP":
            if int(getattr(msg, "I", 0)) == 0:
                acc["arsp"].append((t, float(msg.Airspeed)))
        elif ty in ("XKF2", "NKF2"):
            if int(getattr(msg, "C", 0)) == 0:
                acc[ty.lower()].append((t, float(msg.VWN), float(msg.VWE)))
        elif ty == "ATT":
            acc["att"].append((t, float(msg.Yaw), float(msg.Roll), float(msg.Pitch)))
        elif ty == "MODE":
            acc["mode"].append((t, float(msg.ModeNum)))
        elif ty == "EV":
            acc["ev"].append((t, float(msg.Id)))
        elif ty == "STAT":
            acc["stat"].append((t, float(getattr(msg, "Armed", -1)),
                                float(getattr(msg, "isFlying", -1))))
        elif ty == "SIM":
            acc["sim"].append((t, float(msg.Lat), float(msg.Lng), float(msg.Alt)))
    out = {k: np.asarray(v, dtype=float) for k, v in acc.items()}
    meta = {"trial": trial, "uav": uav, "lognum": lognum,
            "log": str(p.relative_to(ROOT)).replace("\\", "/"),
            "log_bytes": p.stat().st_size,
            "log_mtime_epoch": p.stat().st_mtime,
            "counts": {k: int(v) for k, v in sorted(n.items())}}
    return out, meta


def main() -> None:
    sel = sys.argv[1:] or list(TRIALS)
    index = {}
    for trial in sel:
        lognum = TRIALS[trial]
        index[trial] = {}
        for uav in INST:
            arrays, meta = extract(trial, lognum, uav)
            f = CACHE / f"{trial}_{uav}.npz"
            np.savez_compressed(f, **{k: v for k, v in arrays.items()
                                      if not k.startswith("_")})
            meta["cache"] = f.name
            meta["rows"] = {k: int(v.shape[0]) for k, v in arrays.items()
                            if not k.startswith("_")}
            index[trial][uav] = meta
            print(f"{trial} {uav} log={lognum:08d} "
                  f"pos={meta['rows'].get('pos', 0)} gps={meta['rows'].get('gps', 0)} "
                  f"arsp={meta['rows'].get('arsp', 0)} xkf2={meta['rows'].get('xkf2', 0)}")
    (CACHE / "index.json").write_text(
        json.dumps(index, indent=1, sort_keys=True), encoding="utf-8")


if __name__ == "__main__":
    main()
