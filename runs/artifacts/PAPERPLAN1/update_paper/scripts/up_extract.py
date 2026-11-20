# -*- coding: utf-8 -*-
"""label update_paper / stage 1 -- dataflash .BIN -> compact .npz, my own reader.

Deliberately does NOT import or reuse:
  scripts/hv_sitl_report.py, sorties/scripts/bin{alt,sep}.py,
  sorties2/scripts/*, analysis/scripts/ana_*.py, or any of their caches.
It re-reads the raw logs and re-derives the log<->trial mapping from the
wall-clock epoch files written by the runner.

  python up_extract.py              # every trial
  python up_extract.py HV4W55R      # one
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
OUT = HERE.parent / "out"
INST = {"A1": 100, "B2": 101, "B3": 102}

# claimed mapping; every entry is re-checked against the .t0/.t1 epoch files below
TRIALS = {
    "HV4SITL15": 13, "HV4SITL16": 14,
    "HV4W55": 15, "HV4W05": 16, "HV4W00": 17, "BEAMIDP1": 18,
    "HV4W55S": 19, "HV4W55R": 20,
}
EPOCH_DIRS = [ROOT / "runs/artifacts/PAPERPLAN1/sorties2/logs",
              ROOT / "runs/artifacts/PAPERPLAN1/sorties/logs"]

WANT = ["POS", "GPS", "ARSP", "XKF2", "NKF2", "ATT", "MODE", "EV", "STAT", "SIM", "BAT"]


def extract(lognum: int, uav: str):
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
            acc["sim"].append((t, float(msg.Lat), float(msg.Lng), float(msg.Alt),
                               float(getattr(msg, "Yaw", float("nan")))))
        elif ty == "BAT":
            if int(getattr(msg, "Inst", 0)) == 0:
                acc["bat"].append((t, float(getattr(msg, "Volt", float("nan"))),
                                   float(getattr(msg, "Curr", float("nan"))),
                                   float(getattr(msg, "EnrgTot", float("nan")))))
    arrays = {k: np.asarray(v, dtype=float) for k, v in acc.items()}
    meta = {"log": str(p.relative_to(ROOT)).replace("\\", "/"),
            "log_bytes": p.stat().st_size,
            "log_mtime_epoch": p.stat().st_mtime,
            "counts": {k: int(v) for k, v in sorted(n.items())}}
    return arrays, meta


def epochs(trial: str):
    for d in EPOCH_DIRS:
        t0, t1 = d / f"{trial}.t0", d / f"{trial}.t1"
        if t0.exists() and t1.exists():
            return float(t0.read_text().strip()), float(t1.read_text().strip())
    return None, None


def main() -> None:
    sel = sys.argv[1:] or list(TRIALS)
    CACHE.mkdir(parents=True, exist_ok=True)
    OUT.mkdir(parents=True, exist_ok=True)
    idx = {}
    for trial in sel:
        lognum = TRIALS[trial]
        t0, t1 = epochs(trial)
        idx[trial] = {"lognum": lognum, "t0_epoch": t0, "t1_epoch": t1, "uavs": {}}
        for uav in INST:
            arrays, meta = extract(lognum, uav)
            np.savez_compressed(CACHE / f"{trial}_{uav}.npz", **arrays)
            meta["rows"] = {k: int(v.shape[0]) for k, v in arrays.items()}
            # independent mapping check: the .BIN must have been closed inside
            # the sortie's own wall-clock window (with a small slack).
            if t1 is not None:
                meta["mtime_minus_t1_s"] = meta["log_mtime_epoch"] - t1
                meta["mtime_inside_window"] = bool(t0 - 60 <= meta["log_mtime_epoch"] <= t1 + 300)
            idx[trial]["uavs"][uav] = meta
            print(f"{trial:10s} {uav} log={lognum:08d} pos={meta['rows'].get('pos',0):6d} "
                  f"gps={meta['rows'].get('gps',0):6d} arsp={meta['rows'].get('arsp',0):6d} "
                  f"bat={meta['rows'].get('bat',0):6d} "
                  f"dmt={meta.get('mtime_minus_t1_s')}")
    f = OUT / "extract_index.json"
    old = json.loads(f.read_text(encoding="utf-8")) if f.exists() else {}
    old.update(idx)
    f.write_text(json.dumps(old, indent=1, sort_keys=True), encoding="utf-8")


if __name__ == "__main__":
    main()
