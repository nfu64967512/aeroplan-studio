#!/usr/bin/env python3
"""Measure dive-phase speed-command saturation, handover state, and impact spread
for the RPK/RPL n=2 baseline (6 trials, 12 sorties)."""
import csv, math, os, sys

ROOT = r"C:\Users\joy46\Documents\Projects\aeroplan-studio\runs\artifacts"
TRIALS = ["RPK1", "RPK2", "RPK3", "RPL1", "RPL2", "RPL3"]
VMIN, DIVE_SPEED, GATE_R, DIVE_ANGLE, PACE = 95.0, 210.0, 4800.0, 27.0, 150.0


def load(tag):
    rows = []
    with open(os.path.join(ROOT, tag, f"tot_{tag}.csv"), newline="") as f:
        for r in csv.DictReader(f):
            try:
                rows.append(dict(t=float(r["t"]), i=int(r["sysid"]),
                                 lat=float(r["lat"]), lon=float(r["lon"]),
                                 alt=float(r["alt"]), gs=float(r["gs"]),
                                 d=float(r["d_slot"]), mode=r["mode"],
                                 v=float(r["v_cmd"]), phase=r["phase"],
                                 agl=float(r["agl"])))
            except (ValueError, KeyError):
                pass
    return rows


def touchdown(rs):
    """first AGL<=0 crossing, linear interpolated; else last dive row."""
    prev = None
    for r in rs:
        if r["mode"] in ("dive", "expended", "impact"):
            if prev is not None and prev["agl"] > 0 >= r["agl"]:
                f = prev["agl"] / (prev["agl"] - r["agl"])
                return prev["t"] + f * (r["t"] - prev["t"]), True
            prev = r
    return (prev["t"] if prev else float("nan")), False


out = {}
print(f"{'trial':6} {'uav':3} {'dive_n':>6} {'sat_hi%':>7} {'sat_lo%':>7} {'int%':>6} "
      f"{'v_med':>6} {'gsMax':>6} {'h@rel':>6} {'d@rel':>6} {'g@rel':>6} "
      f"{'h@gate':>7} {'d@gate':>7} {'g@gate':>6} {'gs@gate':>7} {'t_td':>7} {'td?':>4}")
for tag in TRIALS:
    rows = load(tag)
    per = {}
    for r in rows:
        per.setdefault(r["i"], []).append(r)
    rec = {}
    for i in sorted(per):
        rs = per[i]
        dv = [r for r in rs if r["mode"] == "dive"]
        if not dv:
            continue
        hi = sum(1 for r in dv if r["v"] >= DIVE_SPEED - 0.5)
        lo = sum(1 for r in dv if r["v"] <= VMIN + 0.5)
        interior = len(dv) - hi - lo
        vs = sorted(r["v"] for r in dv)
        vmed = vs[len(vs) // 2]
        rel = dv[0]
        gate = next((r for r in dv if r["d"] <= GATE_R), None)
        td, real = touchdown(rs)
        gmax = max(r["gs"] for r in dv)
        g_rel = math.degrees(math.atan2(max(rel["agl"], 0), max(rel["d"], 1)))
        if gate:
            g_gate = math.degrees(math.atan2(max(gate["agl"], 0), max(gate["d"], 1)))
        else:
            g_gate = float("nan")
        rec[i] = dict(n=len(dv), hi=100 * hi / len(dv), lo=100 * lo / len(dv),
                      it=100 * interior / len(dv), vmed=vmed, gmax=gmax,
                      h_rel=rel["agl"], d_rel=rel["d"], g_rel=g_rel,
                      h_gate=gate["agl"] if gate else float("nan"),
                      d_gate=gate["d"] if gate else float("nan"),
                      g_gate=g_gate, gs_gate=gate["gs"] if gate else float("nan"),
                      td=td, real=real, t_rel=rel["t"])
        r_ = rec[i]
        print(f"{tag:6} {i:3d} {r_['n']:6d} {r_['hi']:7.1f} {r_['lo']:7.1f} {r_['it']:6.1f} "
              f"{r_['vmed']:6.1f} {r_['gmax']:6.1f} {r_['h_rel']:6.0f} {r_['d_rel']:6.0f} "
              f"{r_['g_rel']:6.1f} {r_['h_gate']:7.0f} {r_['d_gate']:7.0f} {r_['g_gate']:6.1f} "
              f"{r_['gs_gate']:7.1f} {r_['td']:7.1f} {str(r_['real']):>4}")
    out[tag] = rec

print()
print(f"{'trial':6} {'spread_s':>9} {'asym_hi':>8} {'mean_hi':>8} {'asym_int':>9} "
      f"{'mean_int':>9} {'dh@gate':>8} {'dg@gate':>8} {'dt_rel':>7} {'landed':>7}")
summ = []
for tag in TRIALS:
    rec = out[tag]
    ids = [i for i in rec if rec[i]["real"]]
    tds = [rec[i]["td"] for i in ids]
    spread = (max(tds) - min(tds)) if len(tds) >= 2 else float("nan")
    his = [rec[i]["hi"] for i in rec]
    its = [rec[i]["it"] for i in rec]
    hg = [rec[i]["h_gate"] for i in rec]
    gg = [rec[i]["g_gate"] for i in rec]
    trel = [rec[i]["t_rel"] for i in rec]
    asym_hi = max(his) - min(his)
    asym_it = max(its) - min(its)
    dh = max(hg) - min(hg) if not any(math.isnan(x) for x in hg) else float("nan")
    dg = max(gg) - min(gg) if not any(math.isnan(x) for x in gg) else float("nan")
    print(f"{tag:6} {spread:9.2f} {asym_hi:8.1f} {sum(his)/len(his):8.1f} "
          f"{asym_it:9.1f} {sum(its)/len(its):9.1f} {dh:8.0f} {dg:8.1f} "
          f"{max(trel)-min(trel):7.1f} {len(ids):7d}")
    summ.append((tag, spread, asym_hi, sum(his)/len(his), asym_it,
                 sum(its)/len(its), dh, dg, max(trel)-min(trel)))


def spearman(a, b):
    n = len(a)
    if n < 3:
        return float("nan")
    def rank(x):
        s = sorted(range(n), key=lambda k: x[k])
        r = [0.0] * n
        k = 0
        while k < n:
            j = k
            while j + 1 < n and x[s[j + 1]] == x[s[k]]:
                j += 1
            avg = (k + j) / 2.0 + 1
            for m in range(k, j + 1):
                r[s[m]] = avg
            k = j + 1
        return r
    ra, rb = rank(a), rank(b)
    ma, mb = sum(ra) / n, sum(rb) / n
    num = sum((ra[i] - ma) * (rb[i] - mb) for i in range(n))
    da = math.sqrt(sum((x - ma) ** 2 for x in ra))
    db = math.sqrt(sum((x - mb) ** 2 for x in rb))
    return num / (da * db) if da and db else float("nan")


print()
ok = [s for s in summ if not math.isnan(s[1])]
sp = [s[1] for s in ok]
for idx, name in ((2, "asym_hi"), (3, "mean_hi"), (4, "asym_int"),
                  (5, "mean_int"), (6, "dh@gate"), (7, "dgamma@gate"), (8, "dt_release")):
    v = [s[idx] for s in ok]
    if any(math.isnan(x) for x in v):
        print(f"spearman(spread, {name:12}) =  nan  (missing)")
        continue
    print(f"spearman(spread, {name:12}) = {spearman(sp, v):+.3f}   n={len(ok)}  vals={[round(x,1) for x in v]}")
print(f"spreads = {[round(x,2) for x in sp]}")
