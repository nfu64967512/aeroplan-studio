# -*- coding: utf-8 -*-
"""Across every committed HV4SITL trial that used the 120 m cap: how far above
120 m did (a) the exported plan and (b) the flight actually go?"""
import json, os
ROOT = r"C:/Users/joy46/Documents/Projects/aeroplan-studio"
OUT = os.path.join(ROOT, "runs/artifacts/PAPERPLAN1/c1_altitude")
d = json.load(open(os.path.join(OUT, "alt_evidence.json"), encoding="utf-8"))
print(f"{'trial':<11}{'uav':<5}{'plan max':>10}{'plan exc':>10}{'flown max':>11}"
      f"{'flown exc':>11}{'track exc':>11}{'csv dt':>8}")
rowsout = []
for tr, ent in d["trials"].items():
    plan, fly = ent["plan"], ent["flight"]
    if not plan:
        continue
    if max(p["max_alt"] for p in plan.values()) > 125.0:   # not a 120 m-cap trial
        continue
    for u in sorted(set(plan) | set(fly)):
        p = plan.get(u); f = fly.get(u)
        if not p:
            continue
        pm = p["max_alt"]
        if f is None:
            print(f"{tr:<11}{u:<5}{pm:>10.3f}{pm-120:>+10.3f}{'-':>11}{'-':>11}{'-':>11}{'-':>8}")
            continue
        fm = f["peak_alt"]
        print(f"{tr:<11}{u:<5}{pm:>10.3f}{pm-120:>+10.3f}{fm:>11.2f}{fm-120:>+11.2f}"
              f"{fm-pm:>+11.3f}{f['dt_median']:>8}")
        rowsout.append(dict(trial=tr, uav=u, plan_max=pm, plan_excess=round(pm-120, 4),
                            flown_max=fm, flown_excess=round(fm-120, 3),
                            tracking_excess=round(fm-pm, 3), csv_dt_s=f["dt_median"]))
print()
pe = [r["plan_excess"] for r in rowsout]; fe = [r["flown_excess"] for r in rowsout]
te = [r["tracking_excess"] for r in rowsout]
print(f"n={len(rowsout)} aircraft-sorties at the 120 m cap")
print(f"  export artefact (exported plan - 120): max {max(pe):+.4f}  min {min(pe):+.4f}")
print(f"  flown excess    (flown    - 120)     : max {max(fe):+.3f}  min {min(fe):+.3f}")
print(f"  tracking excess (flown - exported)   : max {max(te):+.3f}  min {min(te):+.3f}")
json.dump(rowsout, open(os.path.join(OUT, "trial_table.json"), "w"), indent=1)
