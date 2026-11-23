#!/usr/bin/env python3
"""Verify trial<->dataflash-log identity and solve the CSV->log time offset.

The tot_*.csv clock (t, seconds since node start) and the dataflash clock
(TimeUS, microseconds since FC boot) are unrelated.  We solve one scalar tau
per sortie by minimising the horizontal distance between the node's recorded
track and the FC's own POS track over the DIVE segment.
Residual RMS is reported so the identity claim is falsifiable.
"""
import csv, sys, numpy as np

REPO = "/mnt/c/Users/joy46/Documents/Projects/aeroplan-studio"
CACHE = "/home/windows-wsl/gammadiag_cache"
EARTH = 111319.4917

UAV1 = dict(zip([f"RRD{i}" for i in range(1, 9)] + [f"RRE{i}" for i in range(1, 9)],
                [405, 406, 407, 408, 409, 410, 411, 412,
                 413, 414, 415, 416, 417, 418, 419, 420]))
UAV2 = dict(zip([f"RRD{i}" for i in range(1, 9)] + [f"RRE{i}" for i in range(1, 9)],
                [393, 394, 395, 396, 397, 398, 399, 400,
                 401, 402, 403, 404, 405, 406, 407, 408]))


def load_csv(trial, sysid):
    """CSV rows for one sortie, with the 0.05 s grid restored.

    ~50% of adjacent rows carry an identical timestamp (the writer stamps at
    1/20 s but the node ticks at 1/20 s with two aircraft per tick); the k-th
    row sharing a stamp is placed at t + 0.05*k.  This is the documented
    measurement discipline for this dataset.
    """
    rows = [r for r in csv.DictReader(open(f"{REPO}/runs/artifacts/{trial}/tot_{trial}.csv"))
            if r["sysid"] == str(sysid)]
    t, seen = [], {}
    for r in rows:
        t0 = float(r["t"])
        k = seen.get(t0, 0); seen[t0] = k + 1
        t.append(t0 + 0.05 * k)
    return np.array(t), rows


def load_log(uav, num):
    return np.load(f"{CACHE}/u{uav}_{num}.npz", allow_pickle=True)


def solve(trial, sysid):
    t, rows = load_csv(trial, sysid)
    dive = np.array([r["phase"] == "DIVE" for r in rows])
    lat = np.array([float(r["lat"]) for r in rows])
    lon = np.array([float(r["lon"]) for r in rows])
    num = (UAV1 if sysid == 1 else UAV2)[trial]
    z = load_log(sysid, num)
    lt = z["POS.TimeUS"] / 1e6
    lla = z["POS.Lat"]; llo = z["POS.Lng"]
    ts, las, los = t[dive], lat[dive], lon[dive]
    ml = np.cos(np.radians(las.mean()))

    def rms(tau):
        a = np.interp(ts + tau, lt, lla, left=np.nan, right=np.nan)
        b = np.interp(ts + tau, lt, llo, left=np.nan, right=np.nan)
        d = np.hypot((a - las) * EARTH, (b - los) * EARTH * ml)
        return np.nanmean(d ** 2) ** .5 if np.isfinite(d).sum() > 50 else 1e9

    best = min(np.arange(-200, 201, 1.0), key=rms)
    best = min(np.arange(best - 1.5, best + 1.5, 0.02), key=rms)
    return best, rms(best), num


if __name__ == "__main__":
    print(f"{'trial':6s} {'sys':3s} {'log':5s} {'tau_s':>9s} {'rms_m':>9s}")
    for trial in list(UAV1):
        for sysid in (1, 2):
            tau, r, num = solve(trial, sysid)
            print(f"{trial:6s} {sysid:<3d} {num:<5d} {tau:9.2f} {r:9.2f}")
