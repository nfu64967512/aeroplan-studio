# -*- coding: utf-8 -*-
"""Re-derive the true altitude maxima from the ArduPilot dataflash logs that the
HV4SITL16 SITL instances left behind (READ ONLY; no SITL is started).

flight.csv is 1 Hz MAVLink telemetry, so its max is a LOWER bound on the real peak.
CTUN.Alt / POS.RelHomeAlt are logged at the controller rate.
"""
import os, sys, json, math
from pymavlink import mavutil
ROOT = r"C:/Users/joy46/Documents/Projects/aeroplan-studio"
OUT = os.path.join(ROOT, "runs/artifacts/PAPERPLAN1/c1_altitude")
LOGS = {"A1": r"sitl/vtol/i100/logs/00000014.BIN",
        "B2": r"sitl/vtol/i101/logs/00000014.BIN",
        "B3": r"sitl/vtol/i102/logs/00000014.BIN"}
WANT = ["CTUN", "POS", "MSG", "PARM", "MODE", "CMD"]
res = {}
for uav, rel in LOGS.items():
    path = os.path.join(ROOT, rel)
    m = mavutil.mavlink_connection(path, dialect="ardupilotmega")
    ctun_alt, ctun_t, pos_alt, pos_t, nav_alt = [], [], [], [], []
    fence_alt_max = None
    modes = []
    n = 0
    while True:
        msg = m.recv_match(type=WANT)
        if msg is None:
            break
        n += 1
        t = msg.__dict__.get("TimeUS", 0) / 1e6
        ty = msg.get_type()
        if ty == "CTUN":
            a = getattr(msg, "Alt", None)
            if a is not None:
                ctun_alt.append(float(a)); ctun_t.append(t)
            na = getattr(msg, "NavAlt", None)
            if na is not None:
                nav_alt.append(float(na))
        elif ty == "POS":
            a = getattr(msg, "RelHomeAlt", None)
            if a is not None:
                pos_alt.append(float(a)); pos_t.append(t)
        elif ty == "PARM" and getattr(msg, "Name", "") == "FENCE_ALT_MAX":
            fence_alt_max = float(msg.Value)
        elif ty == "MODE":
            modes.append((t, getattr(msg, "Mode", None), getattr(msg, "ModeNum", None)))
    def rate(ts):
        d = [b - a for a, b in zip(ts, ts[1:]) if 0 < b - a < 5]
        return (len(d) / sum(d)) if d else 0.0
    print("=" * 92)
    print(f"{uav}  {rel}   msgs_read={n}  FENCE_ALT_MAX(in log)={fence_alt_max}")
    out = dict(log=rel, fence_alt_max=fence_alt_max)
    if ctun_alt:
        i = max(range(len(ctun_alt)), key=lambda k: ctun_alt[k])
        over = [a for a in ctun_alt if a > 120.0]
        print(f"   CTUN.Alt (height above home, controller rate): n={len(ctun_alt)} "
              f"rate={rate(ctun_t):.1f} Hz")
        print(f"      max = {max(ctun_alt):.4f} m  at t={ctun_t[i]:.1f}s ; "
              f"samples >120.000 = {len(over)} ({100.0*len(over)/len(ctun_alt):.2f}%)")
        s = sorted(ctun_alt)
        print(f"      p50={s[len(s)//2]:.3f} p99={s[int(.99*(len(s)-1))]:.3f} "
              f"p999={s[int(.999*(len(s)-1))]:.3f}")
        out["ctun"] = dict(n=len(ctun_alt), rate_hz=round(rate(ctun_t), 2),
                           max=max(ctun_alt), t_max=ctun_t[i], n_over_120=len(over))
    if pos_alt:
        i = max(range(len(pos_alt)), key=lambda k: pos_alt[k])
        over = [a for a in pos_alt if a > 120.0]
        print(f"   POS.RelHomeAlt: n={len(pos_alt)} rate={rate(pos_t):.1f} Hz")
        print(f"      max = {max(pos_alt):.4f} m at t={pos_t[i]:.1f}s ; "
              f"samples >120.000 = {len(over)} ({100.0*len(over)/len(pos_alt):.2f}%)")
        out["pos"] = dict(n=len(pos_alt), rate_hz=round(rate(pos_t), 2),
                          max=max(pos_alt), t_max=pos_t[i], n_over_120=len(over))
    if nav_alt:
        print(f"   CTUN.NavAlt (demanded): max = {max(nav_alt):.4f} m")
        out["nav_alt_max"] = max(nav_alt)
    res[uav] = out
json.dump(res, open(os.path.join(OUT, "binlog_alt.json"), "w"), indent=1)
print("\nwrote binlog_alt.json")
