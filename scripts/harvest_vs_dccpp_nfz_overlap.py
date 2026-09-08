"""第二張比較圖：禁航區避繞 與 重疊率反應。

    python scripts/harvest_vs_dccpp_nfz_overlap.py [--pdf]

左半：**重疊率反應**。把重疊率從 0 調到 0.5，正確的反應是「條帶變密 → 航程變長
→ 覆蓋率不變」。三條基準線都這樣，只有舊 DCCPP 在「參數」頁的慣例下反過來：
航程**變少**、覆蓋率**掉到 68.8%**。

右半：**禁航區**。三種處理方式疊在同一張圖上，並且分開量「航跡有沒有進去」與
「相機有沒有拍到裡面」—— 事後繞行只管得了前者。

⚠⚠ 兩條路徑在不同層做避繞，這是比較的前提：
    舊 DCCPP  規劃器**完全不知道** NFZ（`core/dccpp/` grep 不到 nfz/fence），
              避繞是 GUI 的事後修正（`main_window:3058` → `FixedWingNFZPlanner`）。
    HARVEST   NFZ 是規劃時的約束（`fence_zones`）。
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
# ⚠ 原本硬編 session scratchpad 的路徑（會消失），改成 repo 相對。
BENCH = str(Path(__file__).resolve().parent.parent
             / "research" / "cpp_benchmark")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(BENCH))

from bench import (boustrophedon, metrics, nfz_correct, nfz_metrics,  # noqa: E402
                   rect_xy, run_planner, run_planner_nfz)

C_BOU, C_OLD, C_NEW = "#2f9e44", "#1c6fd6", "#e8590c"
ALT, HFOV = 140.0, 60.0
SWATH = 2 * ALT * math.tan(math.radians(HFOV) / 2)


def overlap_curves(polys, n=8):
    rs = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
    out = {k: {"cov": [], "km": []} for k in ("bou", "old_param", "old_panel", "new")}
    for r in rs:
        pitch = SWATH * (1.0 - r)
        m = metrics(polys, boustrophedon(polys, n, swath_m=SWATH, pitch_m=pitch), SWATH)
        out["bou"]["cov"].append(m["coverage"] * 100); out["bou"]["km"].append(m["total_km"])
        # 「參數」頁慣例：pitch 固定、overlap_rate 直接送
        t, _s, _e = run_planner(False, polys, n, 0, alt=ALT, pitch=SWATH, overlap=r)
        m = metrics(polys, t, SWATH)
        out["old_param"]["cov"].append(m["coverage"] * 100)
        out["old_param"]["km"].append(m["total_km"])
        # 面板慣例：pitch 吃掉重疊、overlap_rate 送 0
        t, _s, _e = run_planner(False, polys, n, 0, alt=ALT, pitch=pitch, overlap=0.0)
        m = metrics(polys, t, SWATH)
        out["old_panel"]["cov"].append(m["coverage"] * 100)
        out["old_panel"]["km"].append(m["total_km"])
        t, _s, _e = run_planner(True, polys, n, 0, alt=ALT, pitch=pitch, overlap=0.0)
        m = metrics(polys, t, SWATH)
        out["new"]["cov"].append(m["coverage"] * 100); out["new"]["km"].append(m["total_km"])
    return rs, out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pdf", action="store_true")
    args = ap.parse_args()

    polys = [rect_xy(0, 0, 2500, 2000)]
    nfz = [rect_xy(1200, 800, 700, 600)]
    pitch = SWATH * 0.7

    fig = plt.figure(figsize=(16.0, 10.0))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.0, 1.15], hspace=0.30, wspace=0.22)
    ax1, ax2 = fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[1, 0])
    ax3, ax4 = fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[1, 1])

    # ── 左：重疊率反應 ────────────────────────────────────────────
    rs, cur = overlap_curves(polys)
    style = [("bou", C_BOU, "-", "o", "Boustrophedon (pitch absorbs overlap)"),
             ("old_panel", C_OLD, "--", "s", "DCCPP, panel convention"),
             ("old_param", C_OLD, "-", "^", "DCCPP, parameters-page convention"),
             ("new", C_NEW, "-", "d", "HARVEST, panel convention")]
    for k, c, ls, mk, lab in style:
        ax1.plot(rs, cur[k]["cov"], ls, color=c, marker=mk, ms=5, lw=1.6, label=lab)
        ax2.plot(rs, cur[k]["km"], ls, color=c, marker=mk, ms=5, lw=1.6, label=lab)
    ax1.axhline(100, color="0.5", lw=0.7, ls=":")
    ax1.set_ylabel("geometric coverage [%]")
    ax1.set_title("(a) response to the overlap setting\n"
                  "correct behaviour: coverage flat, distance up", fontsize=10)
    ax1.set_ylim(60, 104)
    ax2.set_ylabel("total flown distance [km]")
    ax2.set_xlabel("overlap setting r")
    for a in (ax1, ax2):
        a.grid(alpha=0.25, lw=0.5)
        a.tick_params(labelsize=8)
    ax1.legend(fontsize=7.6, loc="lower left")
    ax2.annotate("raising overlap makes DCCPP\nfly LESS and cover LESS",
                 xy=(0.5, cur["old_param"]["km"][-1]), xytext=(0.28, 250),
                 fontsize=8.2, color=C_OLD,
                 arrowprops=dict(arrowstyle="->", color=C_OLD, lw=1.0))

    # ── 右：禁航區 ────────────────────────────────────────────────
    rows = []
    t_blind, _e, _a, _r = run_planner_nfz(False, polys, 8, 0, alt=ALT, pitch=pitch,
                                          overlap=0.0, nfz_polys=nfz, post_correct=False)
    t_fix, _e, added, reroutes = run_planner_nfz(False, polys, 8, 0, alt=ALT, pitch=pitch,
                                                 overlap=0.0, nfz_polys=nfz)
    t_new, _e, _a, _r = run_planner_nfz(True, polys, 8, 0, alt=ALT, pitch=pitch,
                                        overlap=0.0, nfz_polys=nfz)
    for name, tr in (("DCCPP, planner only", t_blind),
                     ("DCCPP + GUI post-hoc detour", t_fix),
                     ("HARVEST, NFZ as a planning constraint", t_new)):
        m = nfz_metrics(polys, tr, SWATH, nfz)
        rows.append((name, m))

    for ax, (name, m), tr, col in ((ax3, rows[1], t_fix, C_OLD),
                                   (ax4, rows[2], t_new, C_NEW)):
        for p in polys:
            ax.plot(*p.exterior.xy, "k--", lw=1.1)
        for z in nfz:
            ax.fill(*z.exterior.xy, color="#e03131", alpha=0.28, lw=0)
            ax.plot(*z.exterior.xy, color="#e03131", lw=1.4)
        for t in tr:
            ax.plot([q[0] for q in t], [q[1] for q in t], color=col, lw=0.7, alpha=0.9)
        ax.set_aspect("equal", "datalim")
        ax.grid(alpha=0.22, lw=0.5)
        ax.tick_params(labelsize=7)
        ax.set_title(f"({'b' if ax is ax3 else 'c'}) {name}", fontsize=10)
        ax.text(0.015, 0.985,
                f"coverage {m['coverage'] * 100:5.1f} %   {m['total_km']:6.1f} km\n"
                f"track inside NFZ   {m['nfz_track_km']:6.3f} km\n"
                f"imaged inside NFZ  {m['nfz_swept_km2']:6.4f} km$^2$",
                transform=ax.transAxes, va="top", fontsize=7.8, family="monospace",
                bbox=dict(fc="white", ec="0.7", alpha=0.93, boxstyle="round,pad=0.32"))
    ax4.set_xlabel("east [m]")
    for a in (ax3, ax4):
        a.set_ylabel("north [m]", fontsize=8)

    fig.legend(handles=[
        Patch(fc="#e03131", alpha=0.28, ec="#e03131", label="no-fly zone"),
        Line2D([], [], color=C_OLD, lw=1.2, label="DCCPP track"),
        Line2D([], [], color=C_NEW, lw=1.2, label="HARVEST track"),
        Line2D([], [], color="k", ls="--", lw=1.1, label="survey area"),
    ], loc="lower center", ncol=4, fontsize=8.5, frameon=False, bbox_to_anchor=(0.5, 0.005))
    fig.suptitle("Overlap response and no-fly-zone handling\n"
                 "coverage judged geometrically against the survey area minus the NFZ; "
                 "camera swath 161.7 m at 140 m AGL", fontsize=12)
    fig.tight_layout(rect=(0, 0.04, 1, 0.94))

    out = ROOT / "exports"
    out.mkdir(exist_ok=True)
    fig.savefig(out / "harvest_vs_dccpp_nfz_overlap.png", dpi=170)
    print("寫出", out / "harvest_vs_dccpp_nfz_overlap.png")
    if args.pdf:
        fig.savefig(out / "harvest_vs_dccpp_nfz_overlap.pdf")
        print("寫出", out / "harvest_vs_dccpp_nfz_overlap.pdf")

    print(f"\n禁航區（GUI 事後繞行加了 {added / 1000:.2f} km、繞了 {reroutes} 段）")
    print(f"{'處理方式':38s}{'覆蓋':>8s}{'總km':>8s}{'NFZ內航跡km':>12s}{'NFZ內拍到km²':>13s}")
    for name, m in rows:
        print(f"{name:38s}{m['coverage'] * 100:7.1f}%{m['total_km']:8.1f}"
              f"{m['nfz_track_km']:12.3f}{m['nfz_swept_km2']:13.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
