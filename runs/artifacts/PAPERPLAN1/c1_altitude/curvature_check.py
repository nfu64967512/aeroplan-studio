# -*- coding: utf-8 -*-
"""Test the hypothesis that the plan's >120 m waypoints come from the flat-ENU
tangent-plane vs geodetic-height curvature term d^2/(2R), not from any cap bug.
Pure geometry; no SITL, no network."""
import os, sys, math, json
ROOT = r"C:/Users/joy46/Documents/Projects/aeroplan-studio"
sys.path.insert(0, ROOT)
from hv_dccpp.geo import GeoFrame          # noqa: E402

HOME = (23.95, 120.90)                      # from plan/A1.waypoints seq 0 (cmd 16, frame 0)
OUT = os.path.join(ROOT, "runs/artifacts/PAPERPLAN1/c1_altitude")
R = 6371000.0
frame = GeoFrame(HOME[0], HOME[1], 0.0)

rows_all = {}
for uav in ("A1", "B2", "B3"):
    p = os.path.join(ROOT, "runs/artifacts/HV4SITL16/plan", uav + ".waypoints")
    rows = []
    for ln in open(p, encoding="utf-8"):
        ln = ln.strip()
        if not ln or ln.startswith("#") or ln.startswith("QGC"):
            continue
        q = ln.split("\t")
        if len(q) < 12:
            q = ln.split()
        seq, fr, cmd = int(q[0]), int(q[2]), int(q[3])
        lat, lon, alt = float(q[8]), float(q[9]), float(q[10])
        if lat == 0.0 and lon == 0.0:
            continue
        rows.append((seq, fr, cmd, lat, lon, alt))
    rows_all[uav] = rows

print("home =", HOME, " (seq 0 of every plan file, cmd 16 frame 0)")
print()
summary = {}
for uav, rows in rows_all.items():
    recs = []
    for seq, fr, cmd, lat, lon, alt in rows:
        e, n, _z = frame.to_enu([lat], [lon], [0.0])[0]
        d = math.hypot(e, n)
        curv = d * d / (2.0 * R)
        # exact: what MSL height does ENU z=120 map to at this (e,n)?
        msl_at_z120 = float(frame.enu_z_to_msl([[e, n]], 120.0)[0])
        recs.append((seq, cmd, d, alt, curv, msl_at_z120))
    top = sorted([r for r in recs if r[3] > 119.0], key=lambda r: -r[3])[:8]
    print("=" * 96)
    print(f"{uav}: positional items {len(recs)};  max horiz dist from home = {max(r[2] for r in recs):.1f} m")
    print(f"{'seq':>4} {'cmd':>4} {'d_home_m':>9} {'wp_alt_m':>10} {'wp-120':>8} "
          f"{'d^2/2R':>8} {'exact_msl(z=120)':>17} {'wp-exact':>9}")
    for seq, cmd, d, alt, curv, mz in top:
        print(f"{seq:>4} {cmd:>4} {d:9.1f} {alt:10.4f} {alt-120:+8.4f} "
              f"{curv:8.4f} {mz:17.4f} {alt-mz:+9.5f}")
    # residual statistics over ALL items whose alt is in the 120 band
    band = [r for r in recs if 119.5 < r[3] < 120.5]
    if band:
        res = [r[3] - r[5] for r in band]
        print(f"  -- over the {len(band)} items in the 120 m band: "
              f"|wp_alt - exact_msl(enu_z=120)| max = {max(abs(x) for x in res):.6f} m, "
              f"mean = {sum(res)/len(res):+.6f} m")
        summary[uav] = dict(n_band=len(band),
                            max_excess=max(r[3] for r in band) - 120.0,
                            max_abs_residual=max(abs(x) for x in res),
                            max_d_home_m=max(r[2] for r in recs))
json.dump(summary, open(os.path.join(OUT, "curvature_check.json"), "w"), indent=1)
print("\nwrote curvature_check.json")
