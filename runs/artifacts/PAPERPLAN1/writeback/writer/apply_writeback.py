# -*- coding: utf-8 -*-
"""Apply the C1-C7 write-back to cowork_brief.md and generate CHANGELOG.md.

    python runs/artifacts/PAPERPLAN1/writeback/writer/apply_writeback.py

Reads ONLY the backup (writeback/cowork_brief.before.md), so it is idempotent.
Writes: docs/hv_dccpp/AASRC2026_abstract/cowork_brief.md,
        runs/artifacts/PAPERPLAN1/writeback/writer/CHANGELOG.md,
        runs/artifacts/PAPERPLAN1/writeback/writer/length_estimate.json
Every edit's `old` must occur exactly once at the moment it is applied.
"""
from __future__ import annotations

import hashlib
import json
import math
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.dont_write_bytecode = True

from edits_part1 import BEFORE, ROOT, E as E1  # noqa: E402
from edits_part2 import E as E2  # noqa: E402
from edits_part3 import E as E3  # noqa: E402
from edits_part4 import E as E4  # noqa: E402

TARGET = ROOT / "docs/hv_dccpp/AASRC2026_abstract/cowork_brief.md"
CHANGELOG = HERE / "CHANGELOG.md"
LENGTH_JSON = HERE / "length_estimate.json"

EDITS = E1 + E2 + E3 + E4

# ------------------------------------------------------------------ length estimator
CM_PER_LINE = 0.494          # 11 pt, single spacing, grid snapping OFF (~14 pt per line)
CM_PER_LINE_GRID = 0.635     # 18 pt document grid (grid snapping ON)
PAGE_CM = 25.7               # A4 297 mm - 2 x 20 mm margins
UNITS_PER_LINE = 43.8        # 17 cm / 11 pt em (0.388 cm)
# measured: PNG height px / 300 dpi (all 2007 px = 17.0 cm wide)
FIG_CM = {"fig1_architecture.png": 6.80, "fig2_flowchart.png": 7.59, "fig3_results.png": 6.20,
          "fig5_compliance.png": 6.20, "fig4_wind.png": 5.99, "fig1_architecture_compact.png": 5.33}


def _units(s: str) -> float:
    s = re.sub(r"\\[a-zA-Z]+", "x", s)          # LaTeX command -> one glyph
    s = s.replace("**", "").replace("`", "").replace("$", "")
    s = re.sub(r"[{}^_]", "", s)
    u = 0.0
    for ch in s:
        u += 1.0 if ord(ch) >= 0x2E80 else 0.5
    return u


def _para_lines(s: str) -> int:
    return max(1, math.ceil(_units(s) / UNITS_PER_LINE))


def _table_lines(rows: list[list[str]]) -> int:
    ncol = max(len(r) for r in rows)
    maxu = [max((_units(r[j]) if j < len(r) else 0.0) for r in rows) for j in range(ncol)]
    avail = UNITS_PER_LINE - 0.8 * ncol
    tot = sum(maxu) or 1.0
    if tot <= avail:
        w = maxu
    else:
        w = [max(min(m, 5.0), avail * m / tot) for m in maxu]
        scale = avail / sum(w)
        w = [x * scale for x in w]
    n = 0
    for r in rows:
        n += max(math.ceil(_units(c) / max(w[j], 1.0)) if c.strip() else 1 for j, c in enumerate(r))
    return n


def estimate(text: str) -> dict:
    lines = text.split("\n")
    i0 = next(i for i, x in enumerate(lines) if x.startswith("## 1. 封面資訊"))
    i1 = next(i for i, x in enumerate(lines) if x.startswith("## 附錄 A"))
    body = lines[i0 + 1:i1]
    text_lines = 0.0
    fig_cm = 0.0
    figs = []
    in_code = False
    table: list[list[str]] = []
    breakdown: dict[str, float] = {}
    section = "封面"

    def add(n):
        nonlocal text_lines
        text_lines += n
        breakdown[section] = breakdown.get(section, 0.0) + n

    def flush_table():
        nonlocal table
        if table:
            add(_table_lines(table))
            table = []

    for raw in body:
        x = raw.rstrip()
        if x.startswith("```"):
            in_code = not in_code
            continue
        if in_code:
            continue
        if x.startswith("|"):
            if re.match(r"^\|[-| ]+\|$", x):
                continue
            table.append([c.strip() for c in x.strip("|").split("|")])
            continue
        flush_table()
        if not x or x == "---" or x.startswith("## "):
            continue
        if x.startswith("### 題目") or x.startswith("### 作者與服務單位"):
            continue
        if x.startswith("### 摘要"):
            section = "摘要"
            add(1 + 2)             # heading + the two mandated blank lines around it
            continue
        if x.startswith("備選"):
            continue
        if x.startswith("**主推：**"):
            add(1.3 * _para_lines(x.replace("**主推：**", "")))
            continue
        if x.startswith("### ") or x.startswith("#### "):
            section = x.lstrip("# ").split("（")[0][:12]
            add(1.5 if x.startswith("### ") else 1.3)
            continue
        if x.startswith("> 【插入圖"):
            m = re.search(r"`([^`]+\.png)`", x)
            name = m.group(1) if m else "?"
            figs.append(name)
            fig_cm += FIG_CM.get(name, 6.2)
            add(1.5)               # caption line(s) printed under the figure
            continue
        if x.startswith("> 圖說"):
            continue
        if x.startswith("Word 線性格式"):
            continue
        if x.startswith("$$"):
            add(3.0 if "aligned" in x else (2.5 if "tag{4}" in x and "R_{theo" in x else 2.0))
            continue
        if x.startswith("（**待使用者填寫**"):
            continue
        add(_para_lines(x))
    flush_table()
    cm = text_lines * CM_PER_LINE + fig_cm
    cm_grid = text_lines * CM_PER_LINE_GRID + fig_cm
    return {
        "text_lines": round(text_lines, 1),
        "figures": figs,
        "figure_cm": round(fig_cm, 1),
        "pages_no_grid": round(cm / PAGE_CM, 2),
        "pages_with_grid": round(cm_grid / PAGE_CM, 2),
        "printed_chars": sum(len(x) for x in body),
        "lines_by_section": {k: round(v, 1) for k, v in breakdown.items()},
    }


# ------------------------------------------------------------------ apply
def apply(text: str) -> str:
    for e in EDITS:
        n = text.count(e["old"])
        if n != 1:
            raise SystemExit(f"{e['id']}: old text occurs {n} times (must be 1)")
        text = text.replace(e["old"], e["new"], 1)
    return text


def sha(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


def apply_cuts(t: str, upto: int) -> str:
    """Simulate the cut order written in section 0.4 (items 1..upto)."""
    out = []
    lines = t.split("\n")
    k = 0
    while k < len(lines):
        x = lines[k]
        if upto >= 1 and x.startswith("系統架構如圖1。"):
            x = "系統架構如圖1。"
        if upto >= 1 and x.startswith("規劃流程如圖2。"):
            x = "規劃流程如圖2。"
        if upto >= 2 and x.startswith("**表1 "):
            out.append("Pioneer／HERO 巡航空速 21.5／19.0 m/s，規劃轉彎半徑 200／120 m（規劃裕度可吸收之風速 W* = 5.1675／5.0333 m/s），"
                       "可用航程 133–145／79–86 km（估計值），兩型共用 HFOV 60° 相機。")
            k += 1
            while k < len(lines) and (lines[k].startswith("|") or not lines[k].strip()):
                k += 1
            continue
        if upto >= 3 and x.startswith("於 1 架 Pioneer 與 3 架 HERO"):
            k += 1
            continue
        if upto >= 4 and x.startswith("表2 情境之各胞條帶數"):
            a = x.index("另設觸發情境")
            b = x.index("均為規劃層模型值。") + len("均為規劃層模型值。")
            x = x[a:b]
        if upto >= 5 and (x.startswith("| HV4SITL16 |") or x.startswith("| W ≥ 6.16 m/s |")):
            k += 1
            continue
        if upto >= 6 and x.startswith("> 【插入圖1："):
            x = x.replace("fig1_architecture.png", "fig1_architecture_compact.png")
        if upto >= 7 and x.startswith("> 【插入圖2："):
            k += 1
            continue
        if upto >= 8 and x.startswith("**表4 "):
            out.append("另一受控對照中，走廊豁免以場坪座標判定時最小水平間距降至 5.0 m（垂直 19.9 m；2 s 網格上界），以場坪身分判定則為 52.7 m 且無違規，"
                       "而規劃驗證與覆蓋率兩者皆無法察覺（單一情境、單次執行；原始資料未納管）。")
            k += 1
            while k < len(lines) and (lines[k].startswith("|") or not lines[k].strip() or lines[k].startswith("表註：回收站")):
                k += 1
            continue
        out.append(x)
        k += 1
    return "\n".join(out)


def main() -> None:
    before = BEFORE.read_text(encoding="utf-8")
    after = apply(before)
    est_b = estimate(before)
    est_a0 = estimate(after)
    est_c0 = estimate(apply_cuts(after, 5))
    est_c8 = estimate(apply_cuts(after, 8))
    est_bc = estimate(apply_cuts(before, 3))
    # the page estimates are written into section 0.4 of the paper itself
    after = after.replace("__PAGES_AFTER__", f"{est_a0['pages_no_grid']:.1f}")
    after = after.replace("__PAGES_BEFORE__", f"{est_b['pages_no_grid']:.1f}")
    after = after.replace("__PAGES_CUT__", f"{est_c0['pages_no_grid']:.1f}")
    after = after.replace("__PAGES_CUT8__", f"{est_c8['pages_no_grid']:.1f}")
    after = after.replace("__PAGES_BEFORE_CUT__", f"{est_bc['pages_no_grid']:.1f}")
    est_a = estimate(after)
    assert "__PAGES_" not in after
    TARGET.write_text(after, encoding="utf-8", newline="\n")

    scen = {"as_written": est_a["pages_no_grid"]}
    for n in range(1, 9):
        scen[f"after_cut_items_1_to_{n}"] = estimate(apply_cuts(after, n))["pages_no_grid"]
    scen["before_after_its_own_cut_items_1_to_3"] = est_bc["pages_no_grid"]
    LENGTH_JSON.write_text(json.dumps({"before": est_b, "after": est_a, "cut_scenarios": scen,
                                       "assumptions": {"cm_per_line": CM_PER_LINE, "cm_per_line_grid": CM_PER_LINE_GRID,
                                                       "page_cm": PAGE_CM, "units_per_line": UNITS_PER_LINE,
                                                       "figure_cm": FIG_CM,
                                                       "unit": "CJK/fullwidth = 1.0, ASCII = 0.5 (Times New Roman)"}},
                                      ensure_ascii=False, indent=1, sort_keys=True), encoding="utf-8")

    import changelog_text as ct  # noqa: E402
    CHANGELOG.write_text(ct.render(EDITS, before, after, est_b, est_a, scen,
                                   sha(before.encode("utf-8")), sha(after.encode("utf-8"))),
                         encoding="utf-8", newline="\n")
    print("edits applied:", len(EDITS))
    print("before:", est_b)
    print("after :", est_a)
    print("cuts  :", scen)


if __name__ == "__main__":
    main()
