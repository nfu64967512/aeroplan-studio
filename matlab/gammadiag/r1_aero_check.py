#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""R1 side-check: is the C_L(alpha) line that path A INVERTS actually calibrated?

Path A sends  alpha_cmd = (n_z*W/(q*S) - C_L0) / C_Lalpha .
dn_z/dalpha is ~4 g/deg here, so a 0.5 deg calibration error is 2 g of command
error -- i.e. the whole authority band.  This re-derives (C_Lalpha, C_L0)
INDEPENDENTLY of B1 and checks the inversion error directly.

⚠️ alpha is taken from SITL TRUTH (SIM quaternion + SIM2 true velocity),
   NOT from the AOA log message.  The AOA message is biased (this script
   prints by how much) and the task brief quotes the biased value.

Usage: r1_aero_check.py <BIN> [<BIN> ...]     Read-only.
"""
from __future__ import annotations

import math
import statistics as st
import sys

from pymavlink import mavutil

G, RHO0, ISA_EXP = 9.80665, 1.225, 4.25588
CL_A, CL_0 = 6.9503, 0.56381          # tot_node.py defaults, under test
MASS, S = 22.0, 0.45
W = MASS * G
WANT = ["SIM", "SIM2", "AOA", "POS", "IMU"]


def near(rows, t):
    lo, hi = 0, len(rows) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if rows[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    return rows[lo][1]


def one(path):
    m = mavutil.mavlink_connection(path)
    buf = {k: [] for k in WANT}
    while True:
        msg = m.recv_match(type=WANT)
        if msg is None:
            break
        d = msg.to_dict()
        t = d.get("TimeUS")
        if t is not None:
            buf[msg.get_type()].append((t / 1e6, d))
    if not buf["SIM"] or not buf["POS"]:
        print("  %s: no SIM/POS" % path)
        return None
    alts = [(t, d["Alt"]) for t, d in buf["POS"]]
    tmax = max(alts, key=lambda x: x[1])[0]
    td = next((t for t, a in alts if t > tmax and a < 20.0), alts[-1][0])
    t0 = td - 45.0
    rows = []
    for t, s in buf["SIM"]:
        if not (t0 <= t <= td):
            continue
        q0, q1, q2, q3 = s["Q1"], s["Q2"], s["Q3"], s["Q4"]
        s2 = near(buf["SIM2"], t)
        vn, ve, vd = s2["VN"], s2["VE"], s2["VD"]
        u = ((1 - 2 * (q2 * q2 + q3 * q3)) * vn + 2 * (q1 * q2 + q0 * q3) * ve
             + 2 * (q1 * q3 - q0 * q2) * vd)
        w = (2 * (q1 * q3 + q0 * q2) * vn + 2 * (q2 * q3 - q0 * q1) * ve
             + (1 - 2 * (q1 * q1 + q2 * q2)) * vd)
        a_true = math.atan2(w, u)
        h = max(0.0, near(buf["POS"], t)["Alt"])
        sig = max(1e-3, (1.0 - 2.25577e-5 * h) ** ISA_EXP)
        vtas = math.sqrt(vn * vn + ve * ve + vd * vd)
        qp = 0.5 * RHO0 * (sig ** 2.0) * vtas * vtas
        nz = -near(buf["IMU"], t)["AccZ"] / G
        ao = math.radians(near(buf["AOA"], t)["AOA"]) if buf["AOA"] else float("nan")
        rows.append((a_true, ao, nz, qp, vtas, h))
    if len(rows) < 200:
        print("  %s: only %d rows" % (path, len(rows)))
        return None
    x = [r[0] for r in rows]
    y = [r[2] * W / max(r[3] * S, 1e-6) for r in rows]
    mx, my = st.mean(x), st.mean(y)
    k = sum((a - mx) * (b - my) for a, b in zip(x, y)) / sum((a - mx) ** 2 for a in x)
    c = my - k * mx
    res = [abs(((yy - CL_0) / CL_A) - xx) for xx, yy in zip(x, y)]
    dn = st.median([r[3] * S * CL_A * math.radians(1.0) / W for r in rows])
    name = path.replace("\\", "/").split("/")[-1]
    print("  %-14s n=%4d  alpha_true med %+6.3f deg   AOA-log med %+6.3f deg"
          "   bias %+.3f deg" % (name, len(rows), math.degrees(st.median(x)),
                                 math.degrees(st.median([r[1] for r in rows])),
                                 math.degrees(st.median([r[1] - r[0] for r in rows]))))
    print("                 fit  C_Lalpha %8.4f (%+.2f %%)   C_L0 %+8.5f (%+.2f %%)"
          "   R2 %.6f" % (k, (k - CL_A) / CL_A * 100, c, (c - CL_0) / CL_0 * 100,
                          st.correlation(x, y) ** 2))
    print("                 inversion error |alpha_model - alpha_true| med %.4f deg"
          "  p95 %.4f deg   (= %.3f g at %.2f g/deg)"
          % (math.degrees(st.median(res)),
             math.degrees(sorted(res)[int(0.95 * len(res))]),
             math.degrees(st.median(res)) * dn, dn))
    return k, c, math.degrees(st.median(res)) * dn


def main():
    print("tot_node.py under test:  C_Lalpha = %.4f /rad   C_L0 = %.5f" % (CL_A, CL_0))
    o = [z for z in (one(p) for p in sys.argv[1:]) if z]
    if o:
        print("")
        print("pooled %d logs: C_Lalpha med %.4f   C_L0 med %+.5f"
              "   inversion bias med %.3f g"
              % (len(o), st.median([z[0] for z in o]), st.median([z[1] for z in o]),
                 st.median([z[2] for z in o])))


if __name__ == "__main__":
    main()
