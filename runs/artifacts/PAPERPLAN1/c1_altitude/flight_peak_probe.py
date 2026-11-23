# -*- coding: utf-8 -*-
"""How much of the flown >120 m is the export defect and how much is tracking?
Reads HV4SITL16 flight.csv only (no SITL run)."""
import csv, math, os, json, statistics
ROOT = r"C:/Users/joy46/Documents/Projects/aeroplan-studio"
OUT = os.path.join(ROOT, "runs/artifacts/PAPERPLAN1/c1_altitude")
HOME = (23.95, 120.90)
Re = 6371000.0
def ll2xy(lat, lon):
    y = (lat - HOME[0]) * math.pi / 180.0 * Re
    x = (lon - HOME[1]) * math.pi / 180.0 * Re * math.cos(math.radians(HOME[0]))
    return x, y

# planned per-aircraft max exported relative altitude (from export_probe / wp files)
PLAN_MAX = {"A1": 120.0526, "B2": 120.0331, "B3": 120.0331}
res = {}
for uav in ("A1", "B2", "B3"):
    rows = []
    with open(os.path.join(ROOT, "runs/artifacts/HV4SITL16", uav, "flight.csv"),
              encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if not r["lat"]:
                continue
            a = float(r["rel_alt_m"])
            x, y = ll2xy(float(r["lat"]), float(r["lon"]))
            rows.append(dict(t=float(r["t_s"]), a=a, d=math.hypot(x, y),
                             mode=r["mode"], seq=r["wp_seq"], gs=float(r["groundspeed"] or 0)))
    air = [r for r in rows if r["a"] > 2.0]
    band = [r for r in air if r["a"] > 110.0]                # cruise-at-ceiling band
    over = [r for r in band if r["a"] > 120.0]
    over_plan = [r for r in band if r["a"] > PLAN_MAX[uav]]
    pk = max(rows, key=lambda r: r["a"])
    i = rows.index(pk)
    print("=" * 92)
    print(f"{uav}  samples={len(rows)} airborne={len(air)}  in 110-band={len(band)}")
    print(f"   peak rel_alt = {pk['a']:.2f} m at t={pk['t']:.0f}s  d_home={pk['d']:.0f} m "
          f"mode={pk['mode']} seq={pk['seq']} gs={pk['gs']:.1f}")
    print(f"   excess over 120.00 planning cap : {pk['a']-120.0:+.2f} m")
    print(f"   excess over exported plan max ({PLAN_MAX[uav]:.4f}) : {pk['a']-PLAN_MAX[uav]:+.4f} m")
    print(f"   samples >120.00 : {len(over)} / {len(band)} in-band ({100*len(over)/max(1,len(band)):.1f}%)")
    print(f"   samples >plan max: {len(over_plan)} / {len(band)} ({100*len(over_plan)/max(1,len(band)):.1f}%)")
    q = sorted(r["a"] for r in band)
    print(f"   in-band alt: min {q[0]:.2f}  p50 {q[len(q)//2]:.2f}  p95 {q[int(.95*(len(q)-1))]:.2f} "
          f" p99 {q[int(.99*(len(q)-1))]:.2f}  max {q[-1]:.2f}")
    if over:
        print(f"   d_home of the >120 samples: min {min(r['d'] for r in over):.0f} "
              f"med {statistics.median([r['d'] for r in over]):.0f} "
              f"max {max(r['d'] for r in over):.0f} m")
        near = [r for r in band if r["d"] < 600.0]
        far = [r for r in band if r["d"] >= 1000.0]
        if near:
            print(f"   in-band max alt with d_home<600 m : {max(r['a'] for r in near):.2f} "
                  f"(n={len(near)})")
        if far:
            print(f"   in-band max alt with d_home>=1000 m: {max(r['a'] for r in far):.2f} "
                  f"(n={len(far)})")
    print("   peak window (t, alt, d_home, mode, seq):")
    for r in rows[max(0, i-5): i+6]:
        print(f"      {r['t']:7.1f} {r['a']:8.2f} {r['d']:8.0f} {r['mode']:>6} {r['seq']:>4}")
    res[uav] = dict(peak=pk["a"], peak_t=pk["t"], peak_d=pk["d"],
                    plan_max=PLAN_MAX[uav],
                    excess_over_cap=round(pk["a"] - 120.0, 4),
                    excess_over_plan=round(pk["a"] - PLAN_MAX[uav], 4),
                    n_over_120=len(over), n_band=len(band),
                    p95_band=q[int(.95*(len(q)-1))], p99_band=q[int(.99*(len(q)-1))])
json.dump(res, open(os.path.join(OUT, "flight_peak_probe.json"), "w"), indent=1)
print("\nwrote flight_peak_probe.json")
