# -*- coding: utf-8 -*-
"""Where do the >120 m EXPORTED waypoints come from?  Re-plans gui_legal_120_1a2b
(pure Python) and runs the same reduce_waypoints() the exporter uses, tagging each
output point as an original plan sample or an arc-re-discretisation vertex."""
import os, sys, math, json, time
ROOT = r"C:/Users/joy46/Documents/Projects/aeroplan-studio"
sys.path.insert(0, ROOT); os.chdir(ROOT)
OUT = os.path.join(ROOT, "runs/artifacts/PAPERPLAN1/c1_altitude")
import numpy as np
from scripts.hv_sitl_scenarios import SCENARIOS
import scripts.hv_sitl_run as R
from hv_dccpp.ardupilot import reduce_waypoints, accept_radius_m, _home_msl, _wgs

sc = SCENARIOS["gui_legal_120_1a2b"]
hv, _rep = R._plan_gui(sc)
plan, ctx, cfg = hv.last_plan, hv.last_ctx, hv.cfg
home_msl = _home_msl(ctx)
print("cfg.delta_prof_m =", cfg.delta_prof_m, " cfg.e_max_m =", cfg.e_max_m,
      " cfg.psi_max_deg =", math.degrees(cfg.psi_max_rad))
print("home_msl =", home_msl)
res = {}
for u, p in sorted(plan.plans.items()):
    spec = ctx.specs[p.cls]
    w_ref = ctx.wind.max_speed() if ctx.wind is not None else 0.0
    r_acc = accept_radius_m(spec.r_eff(w_ref), None)
    pts = reduce_waypoints(p.waypoints, spec, cfg, r_acc)
    xy = np.array([[q.x, q.y] for q in pts], dtype=float)
    wgs = _wgs(ctx, xy, np.array([q.z for q in pts]))
    rel = wgs[:, 2] - home_msl
    d = np.hypot(xy[:, 0], xy[:, 1])
    over = [k for k in range(len(pts)) if rel[k] > 120.0]
    n_vertex_over = sum(1 for k in over if pts[k].vertex)
    print("=" * 90)
    print(f"{u}: reduced pts={len(pts)}  (plan samples={len(p.waypoints)})  "
          f"max rel={rel.max():.4f} (excess {rel.max()-120:+.4f} m)")
    print(f"   points with rel>120: {len(over)};  of those, arc-re-discretisation "
          f"vertices (vertex=True): {n_vertex_over}  "
          f"({100.0*n_vertex_over/max(1,len(over)):.0f}%)")
    # top offenders
    print(f"   {'k':>4} {'vertex':>6} {'kind':>10} {'d_home':>8} {'agl_field':>10} "
          f"{'z_enu':>11} {'rel_exported':>12} {'excess':>8}")
    for k in sorted(over, key=lambda i: -rel[i])[:6]:
        q = pts[k]
        print(f"   {k:>4} {str(q.vertex):>6} {q.kind:>10} {d[k]:8.1f} {q.agl:10.4f} "
              f"{q.z:11.5f} {rel[k]:12.5f} {rel[k]-120:+8.4f}")
    # control: exclude vertices entirely
    non_v = [k for k in range(len(pts)) if not pts[k].vertex]
    print(f"   max rel over NON-vertex points only = {rel[non_v].max():.6f} "
          f"(excess {rel[non_v].max()-120:+.6f} m)")
    res[u] = dict(n_pts=len(pts), max_rel=float(rel.max()),
                  n_over=len(over), n_vertex_over=int(n_vertex_over),
                  max_rel_nonvertex=float(rel[non_v].max()),
                  max_agl_field=float(max(q.agl for q in pts)))
json.dump(res, open(os.path.join(OUT, "export_probe.json"), "w"), indent=1)
print("\nwrote export_probe.json")
