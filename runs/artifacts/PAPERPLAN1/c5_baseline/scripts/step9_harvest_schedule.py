import sys, json, math
from pathlib import Path
REPO = Path(r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
for p in (str(REPO), str(REPO / "research/cpp_benchmark")):
    sys.path.insert(0, p)
import logging; logging.disable(logging.CRITICAL)
import matrix as M
from bench import _drones, polys_to_areas, HOME_LL
from mission.swarm_coordinator import SwarmCoordinator
SCEN, N = sys.argv[1], int(sys.argv[2])
POLYS, _ = M.SCEN[SCEN]
drones, cls = _drones(N, 0, M.PITCH)
c = SwarmCoordinator(); c.use_harvest = True
c.hv_class_of = lambda d: cls[int(d["drone_id"])]
c.hv_home = HOME_LL; c.hv_wind = (5.0, 270.0); c.hv_fence_zones = []
c.harvest_params = {"hfov_deg":60.0,"vfov_deg":45.0,"hv_c2_range_km":25.0,
                    "hv_waiver_max_agl_m":200.0,"hv_dem_margin_m":3000.0}
res = c.plan_coverage_dccpp(areas=polys_to_areas(POLYS), drones=drones,
        coverage_width_m=M.PITCH, overlap_rate=0.0, coverage_altitude=M.ALT,
        mounting_angle_deg=0.0)
pl = c.hv_coordinator.last_plan
rows=[dict(uav=u, cls=p.cls, t_takeoff_s=round(p.t_takeoff_s,1), t_land_s=round(p.t_land_s,1),
           d_km=round(p.distance_m/1000,2), landing_site=p.landing_site, takeoff_site=p.takeoff_site,
           orbits=p.orbits_inserted, e_used_wh=round(p.energy_used_wh,1),
           e_margin_wh=round(p.energy_margin_wh,1))
      for u,p in sorted(pl.plans.items(), key=lambda kv: kv[1].t_takeoff_s)]
out=dict(scenario=SCEN,n=N,makespan_s=round(pl.makespan_s,1),
         takeoff_span_s=round(max(r['t_takeoff_s'] for r in rows)-min(r['t_takeoff_s'] for r in rows),1),
         land_gaps_s=[round(b['t_land_s']-a['t_land_s'],1) for a,b in
                      zip(sorted(rows,key=lambda r:r['t_land_s']), sorted(rows,key=lambda r:r['t_land_s'])[1:])],
         plans=rows)
d = REPO/"runs/artifacts/PAPERPLAN1/c5_baseline/out"
(d/f"step9_harvest_schedule_{SCEN}_n{N}.json").write_text(json.dumps(out,indent=2,ensure_ascii=False),encoding="utf-8")
print(json.dumps(out,indent=1,ensure_ascii=False))
