# -*- coding: utf-8 -*-
import os, collections
ROOT = r"C:/Users/joy46/Documents/Projects/aeroplan-studio"
D = os.path.join(ROOT, "runs/artifacts/HV4SITL16/plan")
FRAME = {0: "ABS_MSL", 3: "REL_HOME", 10: "TERRAIN", 6: "GLOBAL_REL_INT"}
CMD = {16: "NAV_WAYPOINT", 22: "NAV_TAKEOFF", 21: "NAV_LAND", 84: "VTOL_TAKEOFF",
       85: "VTOL_LAND", 19: "LOITER_TIME", 17: "LOITER_UNLIM", 18: "LOITER_TURNS",
       178: "DO_CHANGE_SPEED", 206: "DO_SET_CAM_TRIGG_DIST", 189: "DO_LAND_START",
       3000: "DO_VTOL_TRANSITION", 177: "DO_JUMP", 20: "NAV_RETURN_TO_LAUNCH"}
for uav in ("A1", "B2", "B3"):
    p = os.path.join(D, uav + ".waypoints")
    rows = []
    for ln in open(p, encoding="utf-8"):
        ln = ln.strip()
        if not ln or ln.startswith("#") or ln.startswith("QGC"):
            continue
        q = ln.split("\t")
        if len(q) < 12:
            q = ln.split()
        rows.append((int(q[0]), int(q[2]), int(q[3]), float(q[8]), float(q[9]), float(q[10])))
    print("=" * 62)
    print(uav, "items:", len(rows))
    c = collections.Counter((FRAME.get(r[1], r[1]), CMD.get(r[2], r[2])) for r in rows)
    for k, v in sorted(c.items(), key=lambda kv: -kv[1]):
        print(f"   {k[0]:>10s} {k[1]:<22s} x{v}")
    real = [r for r in rows if not (r[3] == 0.0 and r[4] == 0.0)]
    over = [r for r in real if r[5] > 120.0]
    print(f"   alt>120: {len(over)} of {len(real)} positional items; "
          f"max={max(r[5] for r in real):.3f}  excess_max={max(r[5] for r in real)-120.0:+.3f} m")
    print(f"   frames of the >120 items: {sorted({FRAME.get(r[1],r[1]) for r in over})}")
    print(f"   cmds   of the >120 items: {sorted({CMD.get(r[2],r[2]) for r in over})}")
    # distribution of excess
    ex = sorted(round(r[5] - 120.0, 4) for r in over)
    print(f"   excess distribution (m): min {ex[0]:+.4f} med {ex[len(ex)//2]:+.4f} max {ex[-1]:+.4f}")
    # all distinct alts
    alts = sorted({round(r[5], 3) for r in real})
    print(f"   distinct alts: n={len(alts)}  lowest5={alts[:5]}  highest5={alts[-5:]}")
