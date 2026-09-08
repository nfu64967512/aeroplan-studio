"""make_table.py -- turn matrix_results_clean.csv into the paper comparison tables."""
from __future__ import annotations

import csv
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from matrix import SCEN, SCEN_DESC, SWATH, PITCH             # noqa: E402

CSV = os.path.join(HERE, "matrix_results_clean.csv")
COV_MIN = 0.99
CELLS = [(s, n) for s in ("S1", "S2", "S3", "S4", "S5") for n in SCEN[s][1]]
COLS = [f"{s}/{n}" for s, n in CELLS]
OVH_FLOOR = SWATH / PITCH

ORDER = ["boustrophedon", "bous_contiguous", "bous_axis_aligned", "bous_interleaved",
         "bous_v2", "bous_v2_rev1", "spiral", "spiral_v2", "spiral_v2_r1",
         "grid_nn", "grid_nn_v2", "grid_nn_v2b", "darp", "darp_v2", "darp_v2_r1",
         "DCCPP", "DCCPP_mix", "HARVEST", "HARVEST_mix"]
FAMILY = {"boustrophedon": "boustrophedon", "bous_contiguous": "boustrophedon",
          "bous_axis_aligned": "boustrophedon", "bous_interleaved": "boustrophedon",
          "bous_v2": "boustrophedon", "bous_v2_rev1": "boustrophedon",
          "spiral": "spiral", "spiral_v2": "spiral", "spiral_v2_r1": "spiral",
          "grid_nn": "grid+NN", "grid_nn_v2": "grid+NN", "grid_nn_v2b": "grid+NN",
          "darp": "DARP", "darp_v2": "DARP", "darp_v2_r1": "DARP",
          "DCCPP": "**this project**", "DCCPP_mix": "**this project**",
          "HARVEST": "**this project**", "HARVEST_mix": "**this project**"}
NL = chr(10)


def load():
    rows = {}
    with open(CSV, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            rows[(r["method"], r["scenario"], int(r["n_uav_req"]))] = r
    return rows


def num(r, k):
    try:
        return float((r or {}).get(k, ""))
    except (TypeError, ValueError):
        return None


def valid(r):
    c = num(r, "coverage")
    return c is not None and c >= COV_MIN


def best_of(rows, s, n, key, lower):
    cand = [(num(rows[(m, s, n)], key), m) for m in ORDER
            if (m, s, n) in rows and valid(rows[(m, s, n)])
            and num(rows[(m, s, n)], key) is not None]
    if not cand:
        return None, None
    return (min(cand) if lower else max(cand))


def ranks(rows, key, lower):
    out = {}
    for s, n in CELLS:
        vals = [(num(rows[(m, s, n)], key), m) for m in ORDER
                if (m, s, n) in rows and valid(rows[(m, s, n)])
                and num(rows[(m, s, n)], key) is not None]
        vals.sort(reverse=not lower)
        for i, (_v, m) in enumerate(vals, 1):
            out[(m, s, n)] = i
    return out


def cell_str(r, key, prec, win):
    if r is None:
        return "-"
    if r.get("error"):
        return "err"
    v = num(r, key)
    if v is None:
        return "-"
    if not num(r, "n_uav"):
        return "*empty*"
    s = f"{v:.{prec}f}"
    if not valid(r):
        s += "+"
    return f"**{s}**" if win else s


def table(rows, key, prec, lower, title, note):
    rk = ranks(rows, key, lower)
    wins = {(s, n): best_of(rows, s, n, key, lower)[1] for s, n in CELLS}
    mrank = {}
    for m in ORDER:
        rs = [rk[(m, s, n)] for s, n in CELLS if (m, s, n) in rk]
        mrank[m] = (sum(rs) / len(rs), len(rs)) if rs else (99.0, 0)
    order = sorted(ORDER, key=lambda m: (mrank[m][0], -mrank[m][1]))
    out = [f"### {title}", "", note, "",
           "| method | " + " | ".join(COLS) + " | mean rank | valid cells |",
           "|:--|" + "--:|" * (len(COLS) + 2)]
    for m in order:
        cs = [cell_str(rows.get((m, s, n)), key, prec, wins.get((s, n)) == m)
              for s, n in CELLS]
        mr, nv = mrank[m]
        mrs = f"{mr:.2f}" if nv else "-"
        out.append(f"| `{m}` | " + " | ".join(cs) + f" | {mrs} | {nv}/{len(CELLS)} |")
    return NL.join(out)


def headline(rows, key="scan_rate", title=None, note=None):
    best = {(s, n): best_of(rows, s, n, key, False)[0] for s, n in CELLS}
    stat = []
    for m in ORDER:
        ratios, ovhs, w, badcov, empty, idle = [], [], 0, 0, 0, 0
        for s, n in CELLS:
            r = rows.get((m, s, n))
            if r is None or r.get("error"):
                ratios.append(0.0)
                empty += 1
                continue
            sc, nu = num(r, key), num(r, "n_uav")
            if not nu:
                ratios.append(0.0)
                empty += 1
                continue
            if nu < n:
                idle += 1
            ratios.append((sc or 0.0) / best[(s, n)])
            if valid(r):
                ovhs.append(num(r, "overhead"))
                if abs(sc - best[(s, n)]) < 1e-9:
                    w += 1
            else:
                badcov += 1
        pos = [x for x in ratios if x > 0]
        tc = [num(rows[(m, s, n)], "turn_charge_pct") for s, n in CELLS
              if (m, s, n) in rows and num(rows[(m, s, n)], "turn_charge_pct") is not None]
        stat.append((sum(ratios) / len(ratios), sum(pos) / len(pos) if pos else 0.0,
                     m, sum(ovhs) / len(ovhs) if ovhs else None, w, badcov, empty, idle,
                     sum(tc) / len(tc) if tc else None))
    stat.sort(reverse=True)
    out = [f"### {title}", "",
           note, "",
           "`eff index` = mean over all 12 cells of (this method's scan rate / the best "
           "scan rate in that cell). A cell where the method returns an empty plan scores "
           "0 -- not covering is not rewarded with a missing value. `eff|ok` is the same "
           "mean restricted to the cells where it did produce a plan, so the two columns "
           "separate *how good it is* from *how often it works*. `mean ovh` averages "
           "route overhead over the cells that pass the coverage gate; the floor at "
           f"pitch = 0.7 x swath is **{OVH_FLOOR:.3f}**, not 1.0.", "",
           "| rank | method | family | eff index | eff-when-ok | mean ovh | turn charge % "
           "| cell wins | cov<0.99 | empty plan | cells w/ idle aircraft |",
           "|--:|:--|:--|--:|--:|--:|--:|--:|--:|--:|--:|"]
    for i, (gm, gp, m, ov, w, badcov, empty, idle, tc) in enumerate(stat, 1):
        ovs = f"{ov:.3f}" if ov else "-"
        tcs = f"{tc:.1f}%" if tc is not None else "-"
        out.append(f"| {i} | `{m}` | {FAMILY[m]} | {gm:.3f} | {gp:.3f} | {ovs} | {tcs} "
                   f"| {w} | {badcov} | {empty} | {idle} |")
    return NL.join(out)


def main():
    rows = load()
    o = ["# Coverage-path-planning efficiency benchmark", "",
         f"Common settings for every method: alt 140 m, HFOV 60 deg => swath "
         f"{SWATH:.1f} m, strip pitch {PITCH:.1f} m (30% side overlap), reference speed "
         f"20 m/s, every track starts and ends at home (0,0) (audited: max |first-home| "
         f"= max |last-home| = 0.0 m for every method). Every number below comes from "
         f"`bench.metrics`; no planner reports its own length or coverage.", "",
         "Reproduce: `JOBS=10 PYTHONUTF8=1 python -u run_matrix.py` (one subprocess per "
         "cell, 228 cells) then `python -u make_table.py`. Two independent full runs of "
         "the matrix agreed on **all 228 cells to the last printed digit** for both "
         "`total_km` and `scan_rate`, so nothing here is run-to-run noise.", "",
         "**Provenance warning.** The two project planners were measured against the "
         "*working tree*, not a commit: git HEAD `c565319` plus uncommitted edits, with "
         "`mission/swarm_coordinator.py` md5 `0c9cb713...` and "
         "`core/dccpp/dccpp_path_builder.py` md5 `82463450...`. Another session was "
         "editing those same files while this ran, so re-running later can legitimately "
         "give different DCCPP/HARVEST numbers. The 15 baselines are self-contained and "
         "import nothing from the project.", "",
         "| id | area | n |", "|:--|:--|:--|"]
    for s in ("S1", "S2", "S3", "S4", "S5"):
        o.append(f"| {s} | {SCEN_DESC[s]} | {', '.join(map(str, SCEN[s][1]))} |")
    o += ["",
          f"`+` = coverage < {COV_MIN} in that cell, so its efficiency numbers are "
          "meaningless: the cell is excluded from ranks and winners. `*empty*` = the "
          "planner returned no flyable path at all. **bold** = winner of that column.", "",
          headline(rows, "scan_rate",
                   "Table 0 - headline comparison, pure geometry (the paper table)",
                   "Zero-radius polylines: this is the ruler the brief defines."), "",
          headline(rows, "scan_rate_r200",
                   "Table 0b - headline comparison, charged for a 200 m turn radius",
                   "Same ruler, but every corner pays the arc it would actually have to "
                   "fly. Read this one if the deliverable is a fixed-wing/VTOL mission "
                   "rather than a geometry result."), "",
          table(rows, "scan_rate", 2, False,
                "Table 1 - scan rate (km2/h, higher is better)",
                "scan_rate = covered area / (longest track / 20 m/s). This is the "
                "makespan metric: it is set by the slowest aircraft, so it punishes both "
                "long routes and bad load balance."), "",
          table(rows, "overhead", 3, True,
                "Table 2 - route overhead (total length / ideal, lower is better)",
                f"overhead = total flown / (covered area / swath). The floor at 30% "
                f"overlap is {OVH_FLOOR:.3f}; everything above that is turns, transit to "
                f"and from home, and double-scanned ground."), "",
          table(rows, "scan_rate_r200", 2, False,
                "Table 2b - scan rate after charging a 200 m turn radius "
                "(km2/h, higher is better)",
                "Tables 0-2 are pure geometry, which silently subsidises any method that "
                "plans corners a real aircraft cannot fly. Here every vertex is charged "
                "the arc-vs-corner difference for a 200 m minimum turn radius (the "
                "class-A airframe in `bench._drones`); a >170 deg reversal is charged a "
                "pi*R teardrop. Coverage is unchanged. This is a *lower bound* on the "
                "penalty, not a re-plan: a Dubins-aware planner would have chosen a "
                "different route rather than pay it."), ""]
    o += ["### Table 3 - per-cell winner vs this project's two planners", "",
          "| cell | winner | scan | ovh | best DCCPP variant | best HARVEST variant | "
          "project's best rank |", "|:--|:--|--:|--:|--:|--:|--:|"]
    rk = ranks(rows, "scan_rate", False)
    for s, n in CELLS:
        bv, bm = best_of(rows, s, n, "scan_rate", False)
        if bm is None:
            continue
        bo = num(rows[(bm, s, n)], "overhead")

        def pick(ms, s=s, n=n, bv=bv):
            got = []
            for m in ms:
                r = rows.get((m, s, n))
                if r and num(r, "scan_rate") is not None:
                    got.append((num(r, "scan_rate"), m, r))
            if not got:
                return "err"
            v, m, r = max(got)
            if not num(r, "n_uav"):
                return f"*empty plan* (`{m}`)"
            tag = "" if valid(r) else "+"
            return f"{v:.2f}{tag} ({(v - bv) / bv * 100:+.1f}%, `{m}`)"
        prj = [rk[(m, s, n)] for m in ("DCCPP", "DCCPP_mix", "HARVEST", "HARVEST_mix")
               if (m, s, n) in rk]
        nv = len([1 for m in ORDER if (m, s, n) in rk])
        rank_s = f"{min(prj)} of {nv}" if prj else f"none valid of {nv}"
        o.append(f"| {s}/n={n} | `{bm}` | {bv:.2f} | {bo:.3f} | "
                 f"{pick(['DCCPP', 'DCCPP_mix'])} | {pick(['HARVEST', 'HARVEST_mix'])} | "
                 f"{rank_s} |")
    o.append("")
    # head-to-head
    BASE = [m for m in ORDER if FAMILY[m] != "**this project**"]
    o += ["### Table 5 - head-to-head: does either project planner ever win?", "",
          "Cells where both methods produced a valid plan (coverage >= 0.99, non-empty). "
          "`w/c` = cells won / cells comparable; margin is the median of "
          "(project scan rate - baseline scan rate) / baseline.", "",
          "| baseline | DCCPP w/c (geom) | median | DCCPP w/c (R200) | median | "
          "HARVEST w/c (geom) | median | HARVEST w/c (R200) | median |",
          "|:--|--:|--:|--:|--:|--:|--:|--:|--:|"]

    def h2h(prj, b, key):
        w, tot, marg = 0, 0, []
        for s, n in CELLS:
            a, c = rows.get((prj, s, n)), rows.get((b, s, n))
            if not (a and c and valid(a) and valid(c) and num(a, "n_uav")
                    and num(c, "n_uav")):
                continue
            va, vc = num(a, key), num(c, key)
            if va is None or vc is None or not vc:
                continue
            tot += 1
            marg.append((va - vc) / vc * 100.0)
            w += va > vc
        marg.sort()
        return (f"{w}/{tot}", f"{marg[len(marg) // 2]:+.1f}%" if marg else "-")
    for b in BASE:
        cells = []
        for prj in ("DCCPP", "HARVEST"):
            for key in ("scan_rate", "scan_rate_r200"):
                cells += list(h2h(prj, b, key))
        # reorder to DCCPP geom, DCCPP r200, HARVEST geom, HARVEST r200
        o.append(f"| `{b}` | " + " | ".join(cells) + " |")
    o.append("")
    o += ["### Table 4 - failures (empty plan, coverage < 0.99, or idle aircraft)", "",
          "| method | cell | aircraft used / asked | coverage | what happened |",
          "|:--|:--|--:|--:|:--|"]
    for m in ORDER:
        for s, n in CELLS:
            r = rows.get((m, s, n))
            if r is None:
                continue
            nu, c = num(r, "n_uav"), num(r, "coverage")
            if r.get("error"):
                note = r["error"][:70]
            elif not nu:
                note = "planner returned no flyable path"
            elif c is not None and c < COV_MIN:
                note = "coverage gate failed -- efficiency numbers void"
            elif nu < n:
                note = f"{int(n - nu)} aircraft got no work"
            else:
                continue
            o.append(f"| `{m}` | {s}/n={n} | {int(nu or 0)} / {n} | "
                     f"{'-' if c is None else f'{c:.4f}'} | {note} |")
    o.append("")
    # loss decomposition
    o += ["### Table 6 - where DCCPP's makespan gap comes from", "",
          "scan rate is (covered area) / (longest track), and longest track = "
          "(total / n) x balance. So the gap against the cell winner factorises into a "
          "**route-length** term and a **load-balance** term. `waste` is the share of "
          "swept ground that lies outside the work area -- ferry legs that scan nothing "
          "useful.", "",
          "| cell | DCCPP total km | winner total km | length gap | DCCPP balance | "
          "winner balance | DCCPP waste | winner waste | winner |",
          "|:--|--:|--:|--:|--:|--:|--:|--:|:--|"]
    for s, n in CELLS:
        d = rows[("DCCPP", s, n)]
        bv, bm = best_of(rows, s, n, "scan_rate", False)
        if bm is None or bm.startswith(("DCCPP", "HARVEST")):
            continue
        b = rows[(bm, s, n)]
        dt = (num(d, "total_km") - num(b, "total_km")) / num(b, "total_km") * 100.0
        o.append(f"| {s}/n={n} | {num(d, 'total_km'):.1f} | {num(b, 'total_km'):.1f} | "
                 f"{dt:+.1f}% | {num(d, 'balance'):.3f} | {num(b, 'balance'):.3f} | "
                 f"{num(d, 'waste') * 100:.1f}% | {num(b, 'waste') * 100:.1f}% | `{bm}` |")
    o.append("")
    print(NL.join(o))


if __name__ == "__main__":
    main()
