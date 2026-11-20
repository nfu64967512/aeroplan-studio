"""C5 step 1 — recompute the paper's "HARVEST is 0.9%-22.9% longer" claim
straight out of the committed benchmark CSV. No planner is run here.

Source: research/cpp_benchmark/results/project_rows_5e0295f5_dirty.csv
        (first line is a `# provenance:` comment, NOT a header -- see
         docs/hv_dccpp/封閉基準_2026-09-10.md section 5)
Ruler : max_km, produced by bench.metrics (the single ruler).
"""
import csv, json, sys
from pathlib import Path

REPO = Path(r"C:/Users/joy46/Documents/Projects/aeroplan-studio")
CSV = REPO / "research/cpp_benchmark/results/project_rows_5e0295f5_dirty.csv"
OUT = Path(__file__).resolve().parents[1] / "out"
OUT.mkdir(parents=True, exist_ok=True)

lines = CSV.read_text(encoding="utf-8").splitlines()
prov = json.loads(lines[0].split("# provenance:", 1)[1])
rows = list(csv.DictReader(lines[1:]))

by = {(r["method"], r["scenario"], int(r["n_uav_req"])): r for r in rows}
cells = sorted({(r["scenario"], int(r["n_uav_req"])) for r in rows})

print("provenance:", json.dumps(prov, ensure_ascii=False))
print(f"rows={len(rows)} cells={len(cells)}")
print()
hdr = ("scen n  DCCPP_km HARVEST_km ratio  pct   D_cov H_cov H_uncov H_valid  included")
print(hdr); print("-" * len(hdr))

tbl, included = [], []
for scen, n in cells:
    d = by.get(("DCCPP", scen, n)); h = by.get(("HARVEST", scen, n))
    if not d or not h:
        continue
    def f(r, k):
        v = r.get(k, "")
        return float(v) if v not in ("", None) else None
    dk, hk, dc, hc = f(d, "max_km"), f(h, "max_km"), f(d, "coverage"), f(h, "coverage")
    if dk is None or hk is None:
        print(f"{scen} {n:<2} <missing: DCCPP err={d['error'][:30]!r} HARVEST err={h['error'][:30]!r}>")
        continue
    ratio = hk / dk
    # paper's inclusion rule (README): coverage < 0.99 -> efficiency numbers meaningless
    ok = (dc is not None and dc >= 0.99) and (hc is not None and hc >= 0.99)
    print(f"{scen} {n:<2} {dk:8.2f} {hk:10.2f} {ratio:6.3f} {(ratio-1)*100:+6.1f}% "
          f"{dc:5} {hc:5} {h['self_uncovered']:>7} {h['self_validate'] or '-':>7}  {'YES' if ok else 'no'}")
    tbl.append(dict(scenario=scen, n=n, dccpp_max_km=dk, harvest_max_km=hk,
                    ratio=round(ratio, 4), pct_longer=round((ratio - 1) * 100, 2),
                    dccpp_coverage=dc, harvest_coverage=hc,
                    harvest_self_uncovered=h["self_uncovered"],
                    harvest_self_validate=h["self_validate"], included=ok))
    if ok:
        included.append((ratio - 1) * 100)

print()
print(f"included cells = {len(included)}")
print(f"min  = {min(included):+.2f}%   max = {max(included):+.2f}%")
print(f"mean = {sum(included)/len(included):+.2f}%")
print(f'=> paper claim "0.9%-22.9% longer over 10 scenarios": '
      f'{"REPRODUCED" if len(included)==10 and abs(min(included)-0.9)<0.05 and abs(max(included)-22.9)<0.05 else "MISMATCH"}')

(OUT / "step1_ratio_from_csv.json").write_text(
    json.dumps(dict(provenance=prov, source=str(CSV), rows=tbl,
                    n_included=len(included), min_pct=min(included), max_pct=max(included),
                    mean_pct=sum(included)/len(included)), indent=2, ensure_ascii=False),
    encoding="utf-8")
print("\nwrote", OUT / "step1_ratio_from_csv.json")
