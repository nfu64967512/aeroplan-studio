# -*- coding: utf-8 -*-
"""C1 altitude evidence. READ-ONLY on repo; writes only into this folder.
Extracts: planned waypoint altitudes, flown rel_alt_m peaks, sampling cadence,
excess-over-120 statistics, and the shape of the peak (transient vs sustained).
"""
import csv, json, math, os, statistics, sys

ROOT = r"C:/Users/joy46/Documents/Projects/aeroplan-studio"
OUT  = os.path.join(ROOT, "runs/artifacts/PAPERPLAN1/c1_altitude")
TRIALS = ["HV4SITL16", "HV4SITL15", "HV4SITL14", "HV4SITL13", "HV4SITL12",
          "HV4SITL11", "HV4SITL10", "HV4SITL9", "HV4SITL8", "HV4SITL7",
          "HV4SITL6", "HV4SITL5", "HV4SITL4", "HV4SITL3", "HV4SITL2", "HV4SITL1"]

FRAME = {0: "ABS(MSL)", 3: "REL(home)", 10: "TERRAIN"}

def read_wp(path):
    rows = []
    with open(path, encoding="utf-8") as f:
        for ln in f:
            ln = ln.rstrip("\n")
            if not ln or ln.startswith("#") or ln.startswith("QGC"):
                continue
            p = ln.split("\t")
            if len(p) < 12:
                p = ln.split()
            if len(p) < 12:
                continue
            rows.append(dict(seq=int(p[0]), cur=int(p[1]), frame=int(p[2]),
                             cmd=int(p[3]),
                             p1=float(p[4]), p2=float(p[5]), p3=float(p[6]),
                             p4=float(p[7]),
                             lat=float(p[8]), lon=float(p[9]), alt=float(p[10])))
    return rows

def read_flight(path):
    out = []
    with open(path, encoding="utf-8") as f:
        for r in csv.DictReader(f):
            try:
                out.append(dict(t=float(r["t_s"]), alt=float(r["rel_alt_m"]),
                                lat=float(r["lat"]) if r["lat"] else None,
                                lon=float(r["lon"]) if r["lon"] else None,
                                gs=float(r["groundspeed"] or 0),
                                asp=float(r["airspeed"] or 0),
                                seq=r["wp_seq"], mode=r["mode"],
                                armed=(r["armed"] == "True")))
            except Exception:
                pass
    return out

report = {"trials": {}}

for tr in TRIALS:
    tdir = os.path.join(ROOT, "runs/artifacts", tr)
    if not os.path.isdir(tdir):
        continue
    ent = {"plan": {}, "flight": {}}
    plandir = os.path.join(tdir, "plan")
    if os.path.isdir(plandir):
        for fn in sorted(os.listdir(plandir)):
            if not fn.endswith(".waypoints"):
                continue
            uav = fn[:-len(".waypoints")]
            rows = read_wp(os.path.join(plandir, fn))
            cand = [r for r in rows if not (r["lat"] == 0.0 and r["lon"] == 0.0)]
            if not cand:
                continue
            mx = max(cand, key=lambda r: r["alt"])
            frames = sorted({FRAME.get(r["frame"], str(r["frame"])) for r in cand})
            over = [r for r in cand if r["alt"] > 120.0]
            ent["plan"][uav] = dict(
                n_items=len(rows), max_alt=mx["alt"], max_alt_cmd=mx["cmd"],
                max_alt_seq=mx["seq"], frames=frames,
                n_over_120=len(over),
                over_alts=sorted({round(r["alt"], 3) for r in over}),
                over_cmds=sorted({r["cmd"] for r in over}),
                distinct_alts_top=sorted({round(r["alt"], 3) for r in cand})[-6:])
    for uav in ("A1", "B2", "B3", "B4", "A2"):
        fp = os.path.join(tdir, uav, "flight.csv")
        if not os.path.isfile(fp):
            continue
        rows = read_flight(fp)
        if not rows:
            continue
        alts = [r["alt"] for r in rows]
        ts = [r["t_s"] if "t_s" in r else r["t"] for r in rows]
        dts = [b - a for a, b in zip(ts, ts[1:]) if b > a]
        pk = max(rows, key=lambda r: r["alt"])
        ipk = rows.index(pk)
        over = [r for r in rows if r["alt"] > 120.0]
        # air segment (>2 m)
        air = [r for r in rows if r["alt"] > 2.0]
        ent["flight"][uav] = dict(
            n=len(rows), t0=ts[0], t1=ts[-1],
            dt_median=round(statistics.median(dts), 3) if dts else None,
            dt_mean=round(sum(dts) / len(dts), 3) if dts else None,
            dt_min=round(min(dts), 3) if dts else None,
            dt_max=round(max(dts), 3) if dts else None,
            peak_alt=pk["alt"], peak_t=pk["t"], peak_mode=pk["mode"],
            peak_seq=pk["seq"], peak_gs=pk["gs"], peak_asp=pk["asp"],
            n_over_120=len(over),
            frac_air_over_120=round(len(over) / max(1, len(air)), 4),
            over_t=[r["t"] for r in over][:40],
            over_alt=[r["alt"] for r in over][:40],
            window=[(r["t"], r["alt"], r["mode"], r["seq"])
                    for r in rows[max(0, ipk - 6): ipk + 7]],
            n_air=len(air),
            alt_p95_air=round(sorted(a["alt"] for a in air)[int(0.95 * (len(air) - 1))], 3) if air else None,
            alt_median_air=round(statistics.median([a["alt"] for a in air]), 3) if air else None,
        )
    if ent["plan"] or ent["flight"]:
        report["trials"][tr] = ent

with open(os.path.join(OUT, "alt_evidence.json"), "w", encoding="utf-8") as f:
    json.dump(report, f, ensure_ascii=False, indent=1)

# console summary
for tr, ent in report["trials"].items():
    print("=" * 70)
    print(tr)
    for u, p in ent["plan"].items():
        print(f"  PLAN {u}: items={p['n_items']} maxalt={p['max_alt']:.3f} "
              f"(cmd={p['max_alt_cmd']} seq={p['max_alt_seq']}) frames={p['frames']} "
              f"over120={p['n_over_120']} {p['over_alts'][:6]} cmds={p['over_cmds']}")
    for u, fl in ent["flight"].items():
        print(f"  FLY  {u}: n={fl['n']} dt med/min/max={fl['dt_median']}/{fl['dt_min']}/{fl['dt_max']} "
              f"peak={fl['peak_alt']:.2f}@t={fl['peak_t']:.1f} mode={fl['peak_mode']} seq={fl['peak_seq']} "
              f"over120={fl['n_over_120']}/{fl['n_air']}air p95={fl['alt_p95_air']}")
print("\nwrote", os.path.join(OUT, "alt_evidence.json"))
