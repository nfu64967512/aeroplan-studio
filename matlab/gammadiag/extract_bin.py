#!/usr/bin/env python3
"""M2 signal-chain dissection: extract per-sortie dataflash channels.

Runs INSIDE WSL (pymavlink lives there).  Writes one npz per (log, uav) into
OUT.  Nothing in the repo is modified; this file is new.

Channels kept (all are MEASURED, straight out of the flight controller):
  ATT   Roll, Pitch                 -- ahrs attitude (deg)
        DesRoll, DesPitch           -- nav_roll_cd/100, nav_pitch_cd/100
                                       i.e. the command AFTER the firmware clamp
  XKF1  VN, VE, VD                  -- EKF3 velocity (m/s, NED)
  POS   Lat, Lng, Alt               -- EKF3 position
  IMU   AccX/Y/Z, GyrY              -- body specific force / pitch rate
  AOA   AOA, SSA                    -- firmware's own angle-of-attack estimate
  ARSP  Airspeed
  PIDP  Tar, Act, P,I,D,FF, Limit   -- pitch RATE loop target/achieved (deg/s)
  AETR  Elev                        -- elevator output (-4500..4500)
  CTUN  NavPitch (== DesPitch)      -- cross-check
"""
import sys, os, numpy as np
from pymavlink import mavutil

TYPES = ["ATT", "XKF1", "POS", "IMU", "AOA", "ARSP", "PIDP", "AETR", "CTUN", "MODE", "PARM"]


def extract(path):
    m = mavutil.mavlink_connection(path)
    acc = {t: [] for t in TYPES}
    parm = {}
    parm_hist = []
    while True:
        msg = m.recv_match(type=TYPES)
        if msg is None:
            break
        t = msg.get_type()
        if t == "PARM":
            parm[msg.Name] = msg.Value
            if msg.Name in ("PTCH_LIM_MIN_DEG", "PTCH_LIM_MAX_DEG", "ROLL_LIMIT_DEG"):
                parm_hist.append((msg.TimeUS / 1e6, msg.Name, msg.Value))
            continue
        if t == "IMU" and getattr(msg, "I", 0) != 0:
            continue          # primary IMU only
        if t == "XKF1" and getattr(msg, "C", 0) != 0:
            continue          # primary EKF core only
        d = msg.to_dict()
        d.pop("mavpackettype", None)
        acc[t].append(d)

    out = {}
    for t, rows in acc.items():
        if not rows:
            continue
        keys = [k for k in rows[0] if isinstance(rows[0][k], (int, float))]
        for k in keys:
            out[f"{t}.{k}"] = np.array([r.get(k, np.nan) for r in rows], dtype=float)
    out["_parm_names"] = np.array(list(parm.keys()))
    out["_parm_vals"] = np.array(list(parm.values()), dtype=float)
    if parm_hist:
        out["_ph_t"] = np.array([p[0] for p in parm_hist])
        out["_ph_n"] = np.array([p[1] for p in parm_hist])
        out["_ph_v"] = np.array([p[2] for p in parm_hist])
    return out


if __name__ == "__main__":
    src, dst = sys.argv[1], sys.argv[2]
    if os.path.exists(dst):
        print("skip", dst); sys.exit(0)
    o = extract(src)
    np.savez_compressed(dst, **o)
    print("ok", dst, len(o), "arrays")
