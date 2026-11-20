# -*- coding: utf-8 -*-
"""prep_beamidp step 3/4 output: turn beamidp_table.json + fly_ready.json into the
markdown tables the paper needs, and compute a per-aircraft makespan estimate for
the sequential arm (better than spreading the fleet delta evenly).

Reads only; runs no planner.
"""
from __future__ import annotations
import json, os, sys

ROOT = r"C:/Users/joy46/Documents/Projects/aeroplan-studio"
HERE = os.path.join(ROOT, "runs/artifacts/PAPERPLAN1/prep_beamidp")
sys.path.insert(0, ROOT)

T = json.load(open(os.path.join(HERE, "beamidp_table.json"), encoding="utf-8"))
F = json.load(open(os.path.join(HERE, "fly_ready.json"), encoding="utf-8"))
NEW, PAPER = T[0], T[1]

LBL = {"sequential": "sequential", "skip_k_fixed": "fixed skip-k",
       "beam_idp_planned": "Beam-IDP (this work)", "held_karp_opt": "Held-Karp optimum"}
ZH = {"sequential": "順序來回掃描", "skip_k_fixed": "固定跳帶 skip-k",
      "beam_idp_planned": "Beam-IDP（本文）", "held_karp_opt": "Held–Karp 最佳解"}
CLS = {"A": "Pioneer", "B": "HERO"}


def cell_rows(r, zh=False):
    lines = []
    for c in r["cells"].values():
        first = True
        for nm, v in c["orders"].items():
            lab = (ZH if zh else LBL)[nm]
            head = ("| %s %s | %d | %d | %d |" % (CLS[c["cls"]], c["cell"], c["n"], c["k"],
                                                  round(c["r_eff_m"]))) if first else "|  |  |  |  |"
            lines.append("%s %s | %.1f | %.1f | %.2f | %.1f | %s |"
                         % (head, lab, v["turn_dist_m"], v["turn_time_s"],
                            v["turn_energy_wh"], v["cell_time_s"],
                            ("base" if nm == "sequential"
                             else "%.1f%% / %.1f%%" % (v["d_turn_dist_pct"], v["d_turn_time_pct"]))))
            first = False
    return lines


HDR_EN = ("| cell (class) | n | k | R_eff (m) | strip order | turning distance (m) | "
          "turning time (s) | turning energy (Wh) | cell mission time (s) | vs sequential (dist / time) |")
SEP = "|---|---|---|---|---|---|---|---|---|---|"
HDR_ZH = ("| 胞（構型） | n | k | R_eff (m) | 條帶序 | 迴轉距離 (m) | 迴轉時間 (s) | "
          "迴轉能量 (Wh) | 胞內任務時間 (s) | 相對順序掃描（距離／時間） |")

# ---- per-aircraft sequential makespan estimate --------------------------------
per_uav_delta = {}
for c in NEW["cells"].values():
    d = c["orders"]["sequential"]["turn_time_s"] - c["orders"]["beam_idp_planned"]["turn_time_s"]
    per_uav_delta[c["uav"]] = per_uav_delta.get(c["uav"], 0.0) + d
est = {}
for u, t in F["tours"].items():
    est[u] = dict(t_land_s=t["t_land_s"], turn_delta_s=round(per_uav_delta.get(u, 0.0), 2),
                  t_land_sequential_s=round(t["t_land_s"] + per_uav_delta.get(u, 0.0), 1))
mk_seq = max(v["t_land_sequential_s"] for v in est.values())
mk_plan = NEW["makespan_s"]

out = {
    "new_scenario": NEW["scenario"], "trial": NEW["trial"],
    "makespan_planned_s": mk_plan,
    "makespan_sequential_estimate_s": round(mk_seq, 1),
    "makespan_delta_pct": round(100 * (mk_seq - mk_plan) / mk_plan, 2),
    "per_uav_estimate": est,
    "fleet": NEW["fleet_turn_totals"],
    "fleet_delta_turn_time_pct": round(100 * (NEW["fleet_turn_totals"]["bid_t"]
                                              - NEW["fleet_turn_totals"]["seq_t"])
                                       / NEW["fleet_turn_totals"]["seq_t"], 2),
    "fleet_delta_turn_dist_pct": round(100 * (NEW["fleet_turn_totals"]["bid_l"]
                                              - NEW["fleet_turn_totals"]["seq_l"])
                                       / NEW["fleet_turn_totals"]["seq_l"], 2),
    "fleet_delta_turn_energy_wh": None,
}
e_seq = sum(c["orders"]["sequential"]["turn_energy_wh"] for c in NEW["cells"].values())
e_bid = sum(c["orders"]["beam_idp_planned"]["turn_energy_wh"] for c in NEW["cells"].values())
out["fleet_turn_energy_sequential_wh"] = round(e_seq, 2)
out["fleet_turn_energy_beamidp_wh"] = round(e_bid, 2)
out["fleet_delta_turn_energy_wh"] = round(e_bid - e_seq, 2)
out["fleet_delta_turn_energy_pct"] = round(100 * (e_bid - e_seq) / e_seq, 2)

md_new = [HDR_EN, SEP] + cell_rows(NEW)
md_new_zh = [HDR_ZH, SEP] + cell_rows(NEW, zh=True)
out["table_md_en"] = "\n".join(md_new)
out["table_md_zh"] = "\n".join(md_new_zh)

# ---- the paper's own scenario -------------------------------------------------
pl = ["| cell (class) | n | k | R_eff (m) | n > k ? | strip order actually used | "
      "turning distance (m) | turning time (s) |", "|---|---|---|---|---|---|---|---|"]
for c in PAPER["cells"].values():
    o = c["orders"]
    same = all(abs(v["turn_time_s"] - o["sequential"]["turn_time_s"]) < 1e-6 for v in o.values())
    pl.append("| %s %s | %d | %d | %d | %s | %s | %.1f | %.2f |"
              % (CLS[c["cls"]], c["cell"], c["n"], c["k"], round(c["r_eff_m"]),
                 "yes" if c["triggers"] else "**no**",
                 "identical to sequential" if same else "differs",
                 o["sequential"]["turn_dist_m"], o["sequential"]["turn_time_s"]))
out["paper_table_md"] = "\n".join(pl)
out["paper_makespan_s"] = PAPER["makespan_s"]
out["paper_fleet_turn"] = PAPER["fleet_turn_totals"]
out["paper_all_orders_identical"] = all(
    abs(v["turn_time_s"] - c["orders"]["sequential"]["turn_time_s"]) < 1e-6
    for c in PAPER["cells"].values() for v in c["orders"].values())

# strip census
out["strip_counts_new"] = {k: dict(n=c["n"], k=c["k"], cls=CLS[c["cls"]], uav=c["uav"],
                                   triggers=c["triggers"])
                           for k, c in NEW["cells"].items()}
out["strip_counts_paper"] = {k: dict(n=c["n"], k=c["k"], cls=CLS[c["cls"]], uav=c["uav"],
                                     triggers=c["triggers"])
                             for k, c in PAPER["cells"].items()}

json.dump(out, open(os.path.join(HERE, "tables.json"), "w", encoding="utf-8"),
          ensure_ascii=False, indent=1)
open(os.path.join(HERE, "table_new_en.md"), "w", encoding="utf-8").write(out["table_md_en"] + "\n")
open(os.path.join(HERE, "table_new_zh.md"), "w", encoding="utf-8").write(out["table_md_zh"] + "\n")
open(os.path.join(HERE, "table_paper.md"), "w", encoding="utf-8").write(out["paper_table_md"] + "\n")

print(out["table_md_en"])
print("")
print(out["paper_table_md"])
print("")
print("fleet turn: %.1f -> %.1f s (%.2f%%), %.1f -> %.1f m (%.2f%%), %.2f -> %.2f Wh (%.2f%%)"
      % (out["fleet"]["seq_t"], out["fleet"]["bid_t"], out["fleet_delta_turn_time_pct"],
         out["fleet"]["seq_l"], out["fleet"]["bid_l"], out["fleet_delta_turn_dist_pct"],
         e_seq, e_bid, out["fleet_delta_turn_energy_pct"]))
print("makespan planned %.1f s ; sequential ESTIMATE %.1f s (%+.2f%%)  per-uav: %s"
      % (mk_plan, mk_seq, out["makespan_delta_pct"], est))
print("paper: all four orders identical = %s ; makespan %.1f s"
      % (out["paper_all_orders_identical"], out["paper_makespan_s"]))
print("")
print("wrote " + os.path.join(HERE, "tables.json"))
