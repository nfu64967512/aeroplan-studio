"""HARVEST vs 舊 DCCPP：投影片用的清楚版（固定 4 台 = SITL 實飛機隊）。

    python scripts/harvest_vs_dccpp_slide.py            # 算一次 + 畫兩張圖
    python scripts/harvest_vs_dccpp_slide.py --cached   # 只重畫，不重算

輸出
----
* ``exports/slide_harvest_map.png``      四格地圖，投影片字級
* ``exports/slide_harvest_summary.png``  覆蓋率與航程長條圖（不看地圖也能懂）

為什麼固定 4 台
--------------
機隊固定成 **1 Pioneer + 3 HERO**（HV4SITL3 真的飛過的那一組），
四格就變成一個乾淨的受控對照，**每一對相鄰格只差一個變因**：

    (a) → (c)   作業區形狀：4 塊小區 → 單一大區
    (b) ↔ (c)   條帶參數：pitch 150 / overlap 0.30 → pitch 113.2 / overlap 0
    (c) ↔ (d)   切區方式：系統自動切 → 人工預先切 4 塊

⚠ 這正是原腳本 ``--fleet 1+3`` 想做的事，但它第 183-184 行只保留 title 第一行，
把第二行的 pitch / overlap 說明蓋掉 ⇒ 讀圖的人會以為 (b) 只是換了機隊，
而其實 (b) 變的是條帶參數。本腳本自己組標題，不會有那個問題。

量法沿用 ``harvest_vs_dccpp_figure.py``（同一把尺）：
全航點不做航段篩選、距離用航點折線、兩邊套同一個物理刈幅 2·h·tan(HFOV/2)。
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                      # noqa: E402
from matplotlib.lines import Line2D                  # noqa: E402
from matplotlib.patches import Patch                 # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from scripts.harvest_vs_dccpp_figure import (        # noqa: E402
    _grid4, _xy, coverage, plan, rect, swept,
)
from shapely.geometry import Polygon                 # noqa: E402

CACHE = ROOT / "exports" / ".slide_harvest_cache.json"
OUT_MAP = ROOT / "exports" / "slide_harvest_map.png"
OUT_SUM = ROOT / "exports" / "slide_harvest_summary.png"

# ── 版面 ────────────────────────────────────────────────────────────
plt.rcParams["font.family"] = ["Microsoft JhengHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

INK, MUTED, RULE = "#1a1f26", "#5d6773", "#c8cfd8"
C_OLD, C_NEW = "#1c6fd6", "#e8590c"          # DCCPP 藍 / HARVEST 橘
F_BOTH, F_OLD, F_NEW, F_NONE = "#cdd3da", "#a8c8ef", "#f7c8a8", "#e03131"

N_A, N_B = 1, 3                              # 固定機隊：1 Pioneer + 3 HERO
ALT = 140.0
SWATH = 2.0 * ALT * math.tan(math.radians(60.0) / 2.0)   # 161.66 m

PANELS = [
    dict(key="a", label="(a)",
         head="4 塊小區　實飛幾何",
         sub="pitch 113.2 m ・ overlap 0",
         takeaway="HV4SITL3 實飛幾何。⚠ 那趟飛的是 150 / 0.30，\n"
                  "本格用面板慣例；換回實飛參數 DCCPP 是 29 km / 93.6%",
         areas=_grid4(), width=113.16, overlap=0.0),
    dict(key="b", label="(b)",
         head="單一 5.0 × 4.0 km 大區",
         sub="pitch 150 m ・ overlap 0.30　← 參數頁慣例",
         takeaway="唯一漏掃的一格。根因是我們自己的實作\n"
                  "area_processor.py:351 step=w_i+w_ovl，不是原論文的式子",
         areas=[{"area_id": 1, "priority": 1.0, "polygon": rect(0, 0, 2500, 2000)}],
         width=150.0, overlap=0.30),
    dict(key="c", label="(c)",
         head="單一 5.0 × 4.0 km 大區",
         sub="pitch 113.2 m ・ overlap 0",
         takeaway="同一塊區、同一組參數，交給系統自動切區",
         areas=[{"area_id": 1, "priority": 1.0, "polygon": rect(0, 0, 2500, 2000)}],
         width=113.16, overlap=0.0),
    dict(key="d", label="(d)",
         head="同 20 km²　人工預切 4 區",
         sub="pitch 113.2 m ・ overlap 0",
         takeaway="人工先切開，與 (c) 差 +1.3%，小於重畫變異 ±3.2%",
         areas=[{"area_id": i + 1, "priority": 1.0,
                 "polygon": rect(-1875 + 1250 * i, 0, 625, 2000)} for i in range(4)],
         width=113.16, overlap=0.0),
]


# ── 計算 ────────────────────────────────────────────────────────────
def compute() -> list[dict]:
    rows = []
    for p in PANELS:
        polys = [Polygon([_xy(la, lo) for la, lo in a["polygon"]]) for a in p["areas"]]
        kw = dict(alt=ALT, width=p["width"], overlap=p["overlap"])
        row = {"key": p["key"]}
        for tag, hv in (("old", False), ("new", True)):
            tr, km, n = plan(hv, p["areas"], N_A, N_B, **kw)
            row[tag] = {"tracks": tr, "km": km, "n": n,
                        "cov": 100.0 * coverage(polys, swept(tr, SWATH))}
        row["areas_xy"] = [list(pp.exterior.coords) for pp in polys]
        rows.append(row)
        print(f"  {p['key']}   DCCPP {row['old']['n']} 台 {row['old']['km']:7.1f} km "
              f"{row['old']['cov']:6.1f}%   |   HARVEST {row['new']['n']} 台 "
              f"{row['new']['km']:7.1f} km {row['new']['cov']:6.1f}%")
    CACHE.write_text(json.dumps(rows), encoding="utf-8")
    return rows


# ── 地圖 ────────────────────────────────────────────────────────────
def draw_map(rows: list[dict]) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(16.0, 11.0), dpi=150)
    fig.patch.set_facecolor("white")

    for ax, p, r in zip(axes.ravel(), PANELS, rows):
        polys = [Polygon(c) for c in r["areas_xy"]]
        sw_o = swept([[tuple(q) for q in t] for t in r["old"]["tracks"]], SWATH)
        sw_n = swept([[tuple(q) for q in t] for t in r["new"]["tracks"]], SWATH)
        area = polys[0]
        for q in polys[1:]:
            area = area.union(q)

        # 填色：兩邊都掃到 / 只有一邊 / 都沒掃到（都沒掃到最重要，畫最鮮）
        def _fill(geom, color, z, alpha=1.0):
            if geom is None or geom.is_empty:
                return
            gs = getattr(geom, "geoms", [geom])
            for g in gs:
                if g.is_empty or g.geom_type != "Polygon":
                    continue
                ax.fill(*g.exterior.xy, color=color, zorder=z, alpha=alpha, lw=0)
                for ring in g.interiors:
                    ax.fill(*ring.xy, color="white", zorder=z + 0.1, lw=0)

        if sw_o and sw_n:
            _fill(area.intersection(sw_o).intersection(sw_n), F_BOTH, 1)
            _fill(area.intersection(sw_o).difference(sw_n), F_OLD, 1.2)
            _fill(area.intersection(sw_n).difference(sw_o), F_NEW, 1.2)
            miss = area.difference(sw_o).difference(sw_n)
            _fill(miss, F_NONE, 1.4)

        # DCCPP 沒掃到的區域：粗紅框，這是本圖唯一要人一眼看到的東西
        if sw_o is not None:
            gap = area.difference(sw_o)
            if not gap.is_empty and gap.area > 1e4:
                for g in getattr(gap, "geoms", [gap]):
                    if g.geom_type == "Polygon" and g.area > 1e4:
                        ax.plot(*g.exterior.xy, color=F_NONE, lw=3.2, zorder=6)

        # 航跡：壓低到背景，不要跟填色搶
        for t in r["old"]["tracks"]:
            xs, ys = zip(*t)
            ax.plot(xs, ys, color=C_OLD, lw=0.65, alpha=0.5, zorder=3)
        for t in r["new"]["tracks"]:
            xs, ys = zip(*t)
            ax.plot(xs, ys, color=C_NEW, lw=0.65, alpha=0.5, zorder=3)

        for q in polys:
            ax.plot(*q.exterior.xy, color=INK, lw=2.0, ls=(0, (6, 4)), zorder=5)

        # 標題：兩行一起給，避免與副標疊字
        ax.set_title(f"{p['label']}　{p['head']}\n{p['sub']}", fontsize=16.5,
                     fontweight="bold", color=INK, pad=10, loc="left", linespacing=1.5)

        # 數字全部移到圖外，地圖本身保持乾淨
        o, n = r["old"], r["new"]
        cov_ok = o["cov"] >= 99.5
        pct = 100.0 * (n["km"] / o["km"] - 1.0) if o["km"] else 0.0
        ax.text(0.0, -0.105, f"DCCPP　　{o['km']:.0f} km", transform=ax.transAxes,
                fontsize=15, color=C_OLD, fontweight="bold", ha="left", va="top")
        ax.text(0.0, -0.185,
                f"HARVEST　{n['km']:.0f} km" + ("" if not cov_ok else f"　({pct:+.0f}%)"),
                transform=ax.transAxes, fontsize=15, color=C_NEW, fontweight="bold",
                ha="left", va="top")
        ax.text(0.0, -0.272,
                "兩邊都 100% 覆蓋" if cov_ok
                else f"DCCPP 只掃到 {o['cov']:.1f}%，漏 {100.0 - o['cov']:.1f}%",
                transform=ax.transAxes, fontsize=15, ha="left", va="top",
                fontweight="bold", color="#2f9e44" if cov_ok else F_NONE)
        ax.text(0.0, -0.360, p["takeaway"], transform=ax.transAxes, fontsize=12.5,
                color=MUTED, ha="left", va="top")

        ax.set_aspect("equal")
        ax.tick_params(labelsize=10, colors=MUTED)
        ax.grid(True, color=RULE, lw=.5, alpha=.55)
        for s in ax.spines.values():
            s.set_color(RULE)

    fig.suptitle("同一組輸入、同一把幾何尺：HARVEST 與既有 DCCPP 實作的對照",
                 fontsize=19, fontweight="bold", color=INK, y=.992)
    fig.text(.5, .950, f"覆蓋以幾何判定：全航點折線緩衝相機刈幅 2h·tan(HFOV/2) = {SWATH:.1f} m @ {ALT:.0f} m AGL", fontsize=12.5, color=MUTED, ha="center")
    fig.text(.5, .924, "機隊固定 1 Pioneer + 3 HERO（SITL 實飛機隊）　每一對相鄰格只差一個變因　"
                       "⚠ 跑在 140 m AGL 並假設 200 m 豁免，非 120 m 無豁免體制　"
                       "⚠ 覆蓋率是上界：轉場與降落段也被緩衝進去",
             fontsize=11.5, color=MUTED, ha="center")

    fig.legend(handles=[
        Patch(fc=F_BOTH, label="兩邊都掃到"),
        Patch(fc=F_OLD, label="只有 DCCPP 掃到"),
        Patch(fc=F_NEW, label="只有 HARVEST 掃到"),
        Line2D([], [], color=F_NONE, lw=3.2, label="DCCPP 沒掃到的範圍"),
        Line2D([], [], color=C_OLD, lw=2, label="DCCPP 航跡"),
        Line2D([], [], color=C_NEW, lw=2, label="HARVEST 航跡"),
        Line2D([], [], color=INK, lw=2, ls=(0, (6, 4)), label="作業區"),
    ], loc="lower center", ncol=7, fontsize=12.5, frameon=False,
        bbox_to_anchor=(.5, .004))

    fig.tight_layout(rect=(0, .055, 1, .905))
    fig.subplots_adjust(hspace=.92, wspace=.22)
    fig.savefig(OUT_MAP, dpi=150, facecolor="white")
    print(f"寫出 {OUT_MAP}")


# ── 長條圖（不看地圖也能懂的那一張）─────────────────────────────────
def draw_summary(rows: list[dict]) -> None:
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16.0, 6.4), dpi=150)
    fig.patch.set_facecolor("white")
    names = [f"{p['label']} {p['head'].split()[0]}" for p in PANELS]
    names = ["(a) 4 塊小區\n實飛幾何", "(b) 單一大區\npitch 150 / ovl 0.30",
             "(c) 單一大區\npitch 113 / ovl 0", "(d) 人工切 4 區\npitch 113 / ovl 0"]
    y = range(len(rows))
    h = .34

    # 左：覆蓋率
    for i, r in enumerate(rows):
        ax1.barh(i + h / 2, r["old"]["cov"], height=h, color=C_OLD)
        ax1.barh(i - h / 2, r["new"]["cov"], height=h, color=C_NEW)
        ax1.text(r["old"]["cov"] - 1.5, i + h / 2, f"{r['old']['cov']:.1f}%", va="center",
                 ha="right", color="white", fontsize=13, fontweight="bold")
        ax1.text(r["new"]["cov"] - 1.5, i - h / 2, f"{r['new']['cov']:.1f}%", va="center",
                 ha="right", color="white", fontsize=13, fontweight="bold")
        if r["old"]["cov"] < 99.5:
            ax1.text(r["old"]["cov"] + 1.5, i + h / 2, "← 漏掃", va="center", ha="left",
                     color=F_NONE, fontsize=13, fontweight="bold")
    ax1.axvline(100, color=RULE, lw=1.2, ls="--")
    ax1.set_xlim(70, 104)
    ax1.set_title("覆蓋率　—　完備性", fontsize=17, fontweight="bold", color=INK, pad=12, loc="left")
    ax1.set_xlabel("作業區被掃到的比例 [%]", fontsize=12.5, color=MUTED)

    # 右：航程
    for i, r in enumerate(rows):
        ax2.barh(i + h / 2, r["old"]["km"], height=h, color=C_OLD)
        ax2.barh(i - h / 2, r["new"]["km"], height=h, color=C_NEW)
        ax2.text(r["old"]["km"] + 2, i + h / 2, f"{r['old']['km']:.0f}", va="center",
                 fontsize=13, color=C_OLD, fontweight="bold")
        pct = 100.0 * (r["new"]["km"] / r["old"]["km"] - 1.0) if r["old"]["km"] else 0.0
        tail = "　不可比 ⚠" if r["old"]["cov"] < 99.5 else f"　({pct:+.0f}%)"
        ax2.text(r["new"]["km"] + 2, i - h / 2, f"{r['new']['km']:.0f}{tail}",
                 va="center", fontsize=13, color=C_NEW, fontweight="bold")
    ax2.set_xlim(0, max(r["new"]["km"] for r in rows) * 1.34)
    ax2.set_title("總航程　—　代價", fontsize=17, fontweight="bold", color=INK, pad=12, loc="left")
    ax2.set_xlabel("四機航點折線總長 [km]　（越短越省）", fontsize=12.5, color=MUTED)

    for ax in (ax1, ax2):
        ax.set_yticks(list(y))
        ax.set_yticklabels(names, fontsize=12.5, color=INK)
        ax.invert_yaxis()
        ax.tick_params(axis="x", labelsize=11, colors=MUTED)
        ax.grid(True, axis="x", color=RULE, lw=.6, alpha=.6)
        ax.set_axisbelow(True)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
        for s in ("left", "bottom"):
            ax.spines[s].set_color(RULE)

    fig.suptitle("同一組輸入、同一把尺：完備性打平，HARVEST 用航程換其他東西",
                 fontsize=19, fontweight="bold", color=INK, y=.975)
    fig.text(.5, .052, "⚠ (b) 的航程不可直接比：DCCPP 沒有把區域掃完，少飛的那一段是漏掃換來的",
             fontsize=12.5, color=F_NONE, ha="center")
    fig.legend(handles=[Patch(fc=C_OLD, label="原先 DCCPP"), Patch(fc=C_NEW, label="HARVEST")],
               loc="lower center", ncol=2, fontsize=13.5, frameon=False, bbox_to_anchor=(.5, .002))
    fig.tight_layout(rect=(0, .09, 1, .92))
    fig.savefig(OUT_SUM, dpi=150, facecolor="white")
    print(f"寫出 {OUT_SUM}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cached", action="store_true", help="讀快取，只重畫")
    a = ap.parse_args()
    if a.cached and CACHE.exists():
        rows = json.loads(CACHE.read_text(encoding="utf-8"))
        print("讀快取")
    else:
        print(f"機隊固定 {N_A} Pioneer + {N_B} HERO，刈幅 {SWATH:.1f} m")
        rows = compute()
    draw_map(rows)
    draw_summary(rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
