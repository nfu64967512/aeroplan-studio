"""AASRC 2026 加長型摘要 圖5：高度合規（C1／決策 D2）與實飛間距。

    python docs/hv_dccpp/AASRC2026_abstract/figures_src/make_fig5_compliance.py

輸出
    docs/hv_dccpp/AASRC2026_abstract/fig5_compliance.png   (17 cm 寬, 300 dpi)
    docs/hv_dccpp/AASRC2026_abstract/fig5_compliance.svg
    docs/hv_dccpp/AASRC2026_abstract/figures_src/fig5_data.json

原則同 make_fig_results.py／make_fig4_wind.py。**不動 fig3_results 的任何檔案。**
"""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

sys.dont_write_bytecode = True

import matplotlib  # noqa: E402

matplotlib.use("Agg")
from matplotlib import font_manager as fm  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

HERE = Path(__file__).resolve().parent
OUT_DIR = HERE.parent
ROOT = HERE.parents[3]
ANA = ROOT / "runs" / "artifacts" / "PAPERPLAN1" / "analysis" / "out"
CM = 1 / 2.54

KAIU = Path("C:/Windows/Fonts/kaiu.ttf")
TIMES = Path("C:/Windows/Fonts/times.ttf")
for f in (KAIU, TIMES):
    fm.fontManager.addfont(str(f))
matplotlib.rcParams.update({
    "font.family": [fm.FontProperties(fname=str(TIMES)).get_name(),
                    fm.FontProperties(fname=str(KAIU)).get_name()],
    "font.size": 8.0, "axes.titlesize": 8.5, "axes.labelsize": 8.0,
    "xtick.labelsize": 7.0, "ytick.labelsize": 7.5, "legend.fontsize": 7.0,
    "axes.unicode_minus": True, "axes.linewidth": 0.6,
    "xtick.major.width": 0.6, "ytick.major.width": 0.6,
    "xtick.major.size": 2.5, "ytick.major.size": 2.5,
    "xtick.direction": "in", "ytick.direction": "in",
    "svg.fonttype": "path", "svg.hashsalt": "fig5_compliance", "pdf.fonttype": 42,
})


def rel(p: Path) -> str:
    return p.resolve().relative_to(ROOT).as_posix()


def jload(p: Path):
    return json.loads(p.read_text(encoding="utf-8"))


A = jload(ANA / "altitude.json")
S = jload(ANA / "separation.json")

UAVS = ("A1", "B2", "B3")
MK = {"A1": "o", "B2": "s", "B3": "^"}
ORDER = ["HV4SITL15", "HV4SITL16", "HV4W55", "HV4W05", "HV4W00", "BEAMIDP1"]
XLAB = ["HV4SITL15", "HV4SITL16", "HV4W55", "HV4W05", "HV4W00", "BEAMIDP1"]
C_OK, C_BAD, C_PLAN = "#1b4965", "#bc4b51", "#9db4c0"

fig = plt.figure(figsize=(17.0 * CM, 6.2 * CM))
gs = fig.add_gridspec(1, 2, wspace=0.26, left=0.075, right=0.985,
                      top=0.875, bottom=0.215)

# ------------------------------------------------- (a) 天花板 120.0 vs 119.5 --
ax = fig.add_subplot(gs[0, 0])
ax.axhline(120.0, color=C_BAD, lw=0.9, zorder=1)
ax.annotate("120.000 m 上限", xy=(-0.45, 120.012), ha="left", va="bottom",
            fontsize=6.8, color=C_BAD)
data_a = []
for i, tr in enumerate(ORDER):
    e = A[tr]
    ceil = e["scenario_altitude_m"]
    ax.plot([i - 0.32, i + 0.32], [ceil, ceil], color="0.55", lw=0.7, ls=(0, (4, 2)),
            zorder=1)
    t_tot = 0.0
    for u in UAVS:
        r = e["per_uav"][u]
        y = r["max_rel_alt_m_flown"]
        c = C_BAD if y > 120.0 else C_OK
        x = i + {"A1": -0.20, "B2": 0.0, "B3": 0.20}[u]
        ax.plot(x, r["max_exported_rel_alt_m"], "_", ms=6, color="0.45", mew=1.0, zorder=2)
        ax.plot([x, x], [r["max_exported_rel_alt_m"], y], color="0.75", lw=0.6, zorder=1)
        ax.plot(x, y, MK[u], ms=3.6, mfc=c, mec=c, zorder=3)
        t_tot += r["time_above_120_000_s"]
        data_a.append({"trial": tr, "uav": u, "ceiling_m": ceil,
                       "max_exported_rel_alt_m": r["max_exported_rel_alt_m"],
                       "n_exported_items_above_120": r["n_items_above_120"],
                       "max_flown_rel_alt_m": y,
                       "time_above_120_000_s": r["time_above_120_000_s"],
                       "margin_to_120_000_m": r["margin_to_120_000_m"],
                       "margin_to_121_92_m": r["margin_to_121_92_m"]})
    lab = f"{t_tot:.0f} s" if t_tot > 0 else "0 s"
    ax.annotate(lab, xy=(i, 119.452), ha="center", va="bottom", fontsize=6.6,
                color=(C_BAD if t_tot > 0 else C_OK))
ax.annotate("機隊累計超過 120.000 m 的時間：", xy=(0.02, 0.085), xycoords="axes fraction",
            ha="left", va="bottom", fontsize=6.6, color="0.35")
ax.set_xticks(range(len(ORDER)))
ax.set_xticklabels(XLAB, rotation=18, ha="right")
ax.set_xlim(-0.5, 5.5)
ax.set_ylim(119.44, 120.24)
ax.set_ylabel("實飛最高相對高度 (m AGL)")
ax.set_title("(a) 壓低規劃天花板 120.0 → 119.5 m 的效果", loc="left")
ax.legend(handles=[
    Line2D([], [], marker="_", ls="", color="0.45", ms=6, mew=1.0, label="匯出航點最高"),
    Line2D([], [], marker="o", ls="", mfc=C_OK, mec=C_OK, ms=3.6, label="實飛最高（未超標）"),
    Line2D([], [], marker="o", ls="", mfc=C_BAD, mec=C_BAD, ms=3.6, label="實飛最高（超標）"),
    Line2D([], [], ls=(0, (4, 2)), color="0.55", lw=0.7, label="該架次規劃天花板"),
], loc="upper right", frameon=False, handletextpad=0.5, borderpad=0.2,
    labelspacing=0.22, handlelength=1.4)

# ------------------------------------------------------------- (b) 最小間距 --
ax2 = fig.add_subplot(gs[0, 1])
ax2.axhline(100.0, color=C_BAD, lw=0.9, zorder=1)
ax2.axhspan(98.6, 100.0, color=C_BAD, alpha=0.07, lw=0, zorder=0)
ax2.annotate("100 m 水平間距下限（本區為違規區，六趟皆 0 筆）",
             xy=(-0.42, 99.82), ha="left", va="top", fontsize=6.8, color=C_BAD)
data_b = []
ORDER_B = [t for t in ORDER if t != "BEAMIDP1"]
for i, tr in enumerate(ORDER_B):
    s = S[tr]
    pl = s["planned_min_sep_h_m"]
    ex = s["frames"]["ecef"]["fleet_min_h_m"]
    rp = s["frames"]["repo"]["fleet_min_h_m"]
    ax2.bar(i, pl - 100.0, 0.52, bottom=100.0, color=C_PLAN, ec="none", zorder=1)
    ax2.plot(i, rp, "x", ms=4.2, color="0.45", mew=0.9, zorder=3)
    ax2.plot(i, ex, "o", ms=4.4, mfc=C_OK, mec=C_OK, zorder=4)
    ax2.annotate(f"{ex - 100.0:+.2f}", xy=(i + 0.16, ex), ha="left", va="center",
                 fontsize=6.6, color=C_OK)
    data_b.append({"trial": tr, "planned_min_sep_h_m": pl,
                   "planned_source": s["planned_source"],
                   "flown_min_sep_h_m_ecef": ex, "flown_min_sep_h_m_repo_frame": rp,
                   "binding_pair": s["frames"]["ecef"]["binding_pair"],
                   "d_v_at_min_h_m": s["frames"]["ecef"]["pairs"][
                       s["frames"]["ecef"]["binding_pair"]]["d_v_at_min_h_m"],
                   "n_violations": s["frames"]["ecef"]["n_violations"],
                   "reporter_min_h_m": s["reporter_min_h_m"]})
sb = S["BEAMIDP1"]
data_b.append({"trial": "BEAMIDP1", "planned_min_sep_h_m": sb["planned_min_sep_h_m"],
               "planned_source": sb["planned_source"],
               "flown_min_sep_h_m_ecef": sb["frames"]["ecef"]["fleet_min_h_m"],
               "flown_min_sep_h_m_repo_frame": sb["frames"]["repo"]["fleet_min_h_m"],
               "binding_pair": sb["frames"]["ecef"]["binding_pair"],
               "n_violations": sb["frames"]["ecef"]["n_violations"],
               "reporter_min_h_m": sb["reporter_min_h_m"],
               "not_plotted": "off the axis range; stated in the panel annotation"})
ax2.annotate("BEAMIDP1（軸外）：規劃 "
             f"{sb['planned_min_sep_h_m']:.2f} → 實飛 "
             f"{sb['frames']['ecef']['fleet_min_h_m']:.2f} m（A1–B2）",
             xy=(len(ORDER_B) - 0.58, 98.72), ha="right", va="bottom",
             fontsize=6.6, color="0.35")
ax2.set_xticks(range(len(ORDER_B)))
ax2.set_xticklabels(ORDER_B, rotation=18, ha="right")
ax2.set_xlim(-0.5, len(ORDER_B) - 0.5)
ax2.set_ylim(98.6, 107.2)
ax2.set_ylabel("最小水平間距 (m)")
ax2.set_title("(b) 規劃間距 vs 實飛間距（B2–B3 為約束機對）", loc="left")
ax2.legend(handles=[
    matplotlib.patches.Patch(fc=C_PLAN, ec="none", label="規劃值（2 s 時間格，上界）"),
    Line2D([], [], marker="o", ls="", mfc=C_OK, mec=C_OK, ms=4.4,
           label="實飛（WGS84 ECEF，連續時間精確最小）"),
    Line2D([], [], marker="x", ls="", color="0.45", ms=4.2, mew=0.9,
           label="實飛（既有 111132 m/deg 平面近似）"),
], loc="upper right", frameon=False, handletextpad=0.5, borderpad=0.2,
    labelspacing=0.22, handlelength=1.2)

PNG = OUT_DIR / "fig5_compliance.png"
SVG = OUT_DIR / "fig5_compliance.svg"
fig.savefig(PNG, dpi=300, metadata={"Software": None})
fig.savefig(SVG, metadata={"Date": None, "Creator": None})

srcs = {rel(p): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(ANA.glob("*.json"))}
(HERE / "fig5_data.json").write_text(json.dumps({
    "figure": {"png": rel(PNG), "svg": rel(SVG), "width_cm": 17.0,
               "height_cm": 6.2, "dpi": 300},
    "panel_a_altitude": data_a,
    "panel_b_separation": data_b,
    "grid_cost_of_the_2s_planner_grid": {t: S[t].get("grid_cost_ecef_frame")
                                         for t in ORDER},
    "definitions": {
        "flown_max_rel_alt": "dataflash POS.RelHomeAlt, ~6 Hz, whole log",
        "time_above": "linear interpolation of the level crossings, not sample counting",
        "flown_min_sep": ("both tracks resampled onto the union of their native POS "
                          "times, then the exact minimum of the piecewise-linear "
                          "relative position per interval; violation = horizontal "
                          "< 100 m AND vertical < 50 m with both above 2 m AGL"),
        "ecef_vs_repo": ("hv_sitl_report.py and binsep.py use a flat 111132 m/deg of "
                         "latitude; the planner's mission geometry uses the exact "
                         "WGS84 GeoFrame, in which the same encounter is 0.34 m closer"),
    },
    "caveats": [
        "121.92 m（400 ft）法規上限落在圖 (a) 的縱軸範圍之外：六趟的最小餘裕為 "
        f"{min(r['margin_to_121_92_m'] for r in data_a):.2f} m，全部合規。",
        "規劃間距本身也是 2 s 時間格上的上界（hv_dccpp/sim.py:1007-1034、"
        "mission.py:1092 dt_4d_s = 2.0），所以圖 (b) 的灰柱與圓點不是同一種量測；"
        "grid_cost_of_the_2s_planner_grid 給出同一條航跡上 2 s 格會高估多少。",
        "每格 1 趟；SITL 為決定性模擬，沒有重複量測的散布。",
        "A1–B2 在 gui_legal 四趟中從未同時在空中，故約束機對恆為 B2–B3。",
    ],
    "sources": srcs,
}, indent=1, ensure_ascii=False, sort_keys=True), encoding="utf-8")
print("wrote", rel(PNG), rel(SVG), rel(HERE / "fig5_data.json"))
