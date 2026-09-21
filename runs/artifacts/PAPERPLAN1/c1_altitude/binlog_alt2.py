# -*- coding: utf-8 -*-
"""Pass 2: correct TimeUS handling + message inventory + time-above-120 budget."""
import os, json, collections
from pymavlink import mavutil
ROOT = r"C:/Users/joy46/Documents/Projects/aeroplan-studio"
OUT = os.path.join(ROOT, "runs/artifacts/PAPERPLAN1/c1_altitude")
LOGS = {"A1": r"sitl/vtol/i100/logs/00000014.BIN",
        "B2": r"sitl/vtol/i101/logs/00000014.BIN",
        "B3": r"sitl/vtol/i102/logs/00000014.BIN"}
res = {}
for uav, rel in LOGS.items():
    m = mavutil.mavlink_connection(os.path.join(ROOT, rel), dialect="ardupilotmega")
    inv = collections.Counter()
    pos = []          # (t_s, RelHomeAlt)
    while True:
        msg = m.recv_match()
        if msg is None:
            break
        ty = msg.get_type()
        inv[ty] += 1
        if ty == "POS":
            t = float(getattr(msg, "TimeUS", 0)) / 1e6
            a = getattr(msg, "RelHomeAlt", None)
            if a is not None:
                pos.append((t, float(a)))
    pos.sort()
    ts = [p[0] for p in pos]; al = [p[1] for p in pos]
    dts = [b - a for a, b in zip(ts, ts[1:]) if 0 < b - a < 5]
    hz = len(dts) / sum(dts) if dts else 0.0
    i = max(range(len(al)), key=lambda k: al[k])
    # time above 120 m (trapezoid over consecutive samples both >120)
    t_over = 0.0
    for (t0, a0), (t1, a1) in zip(pos, pos[1:]):
        dt = t1 - t0
        if dt <= 0 or dt > 2.0:
            continue
        if a0 > 120.0 and a1 > 120.0:
            t_over += dt
        elif a0 > 120.0 or a1 > 120.0:
            t_over += dt * (max(a0, a1) - 120.0) / abs(a1 - a0) if a1 != a0 else 0.0
    air = [a for a in al if a > 2.0]
    s = sorted(al)
    print("=" * 92)
    print(f"{uav}  {rel}")
    print(f"   top message types: {dict(inv.most_common(8))}")
    print(f"   POS n={len(pos)} rate={hz:.2f} Hz  span={ts[0]:.1f}..{ts[-1]:.1f} s (log clock)")
    print(f"   max RelHomeAlt = {al[i]:.4f} m at log t={ts[i]:.1f}s  (excess over 120 = {al[i]-120:+.4f} m)")
    print(f"   cumulative time above 120.000 m = {t_over:.1f} s of {ts[-1]-ts[0]:.0f} s logged")
    print(f"   p99={s[int(.99*(len(s)-1))]:.3f}  p999={s[int(.999*(len(s)-1))]:.3f}  max={s[-1]:.3f}")
    res[uav] = dict(log=rel, n_pos=len(pos), rate_hz=round(hz, 2),
                    max_rel_home_alt_m=round(al[i], 4),
                    excess_m=round(al[i] - 120.0, 4),
                    time_above_120_s=round(t_over, 1),
                    log_span_s=round(ts[-1] - ts[0], 1),
                    msg_inventory=dict(inv.most_common(12)))
json.dump(res, open(os.path.join(OUT, "binlog_alt.json"), "w"), indent=1)
print("\nwrote binlog_alt.json (overwrote pass-1)")
