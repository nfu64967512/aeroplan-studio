"""AASRC 2026 加長型摘要 圖4：風場對照（C2）。

    python docs/hv_dccpp/AASRC2026_abstract/figures_src/make_fig4_wind.py

輸出
    docs/hv_dccpp/AASRC2026_abstract/fig4_wind.png   (17 cm 寬, 300 dpi)
    docs/hv_dccpp/AASRC2026_abstract/fig4_wind.svg
    docs/hv_dccpp/AASRC2026_abstract/figures_src/fig4_data.json

原則（與 make_fig_results.py 相同）
- 圖中每個數值都來自 runs/artifacts/PAPERPLAN1/analysis/out/ 的產物，而那些產物
  由 analysis/scripts/ 直接讀 .BIN 與 runs/artifacts/<trial>/ 的原始檔算出；
  本腳本不手抄任何數字，也不 import 任何專案套件。
- 唯讀：只寫上面三個輸出檔。**不動 fig3_results 的任何檔案。**
- 決定性：json 以 sort_keys 輸出、不含時間戳；SVG 去除日期並固定 hashsalt。
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
KAIU_NAME = fm.FontProperties(fname=str(KAIU)).get_name()
TIMES_NAME = fm.FontProperties(fname=str(TIMES)).get_name()
matplotlib.rcParams.update({
    "font.family": [TIMES_NAME, KAIU_NAME], "font.size": 8.0,
    "axes.titlesize": 8.5, "axes.labelsize": 8.0, "xtick.labelsize": 7.5,
    "ytick.labelsize": 7.5, "legend.fontsize": 7.0, "axes.unicode_minus": True,
    "axes.linewidth": 0.6, "xtick.major.width": 0.6, "ytick.major.width": 0.6,
    "xtick.major.size": 2.5, "ytick.major.size": 2.5,
    "xtick.direction": "in", "ytick.direction": "in",
    "svg.fonttype": "path", "svg.hashsalt": "fig4_wind", "pdf.fonttype": 42,
})


def rel(p: Path) -> str:
    return p.resolve().relative_to(ROOT).as_posix()


def sha256(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def jload(p: Path):
    return json.loads(p.read_text(encoding="utf-8"))


K = jload(ANA / "uav_kinematics.json")
T = jload(ANA / "timing.json")
D = jload(ANA / "derived.json")

UAVS = ("A1", "B2", "B3")
MK = {"A1": "o", "B2": "s", "B3": "^"}
ORDER = ["HV4SITL16", "HV4W00", "HV4W05", "HV4W55"]
ROWLAB = {"HV4SITL16": "HV4SITL16\n規劃 5 / 模擬 —",
          "HV4W00": "HV4W00\n規劃 0 / 模擬 0",
          "HV4W05": "HV4W05\n規劃 0 / 模擬 5",
          "HV4W55": "HV4W55\n規劃 5 / 模擬 5"}
C_E, C_W, C_PLAN, C_FLOWN = "#1b4965", "#bc4b51", "#9db4c0", "#1b4965"

fig = plt.figure(figsize=(17.0 * CM, 6.0 * CM))
gs = fig.add_gridspec(1, 2, width_ratios=[1.35, 1.0], wspace=0.30,
                      left=0.115, right=0.985, top=0.88, bottom=0.175)

# ---------------------------------------------------------------- (a) 風證據 --
ax = fig.add_subplot(gs[0, 0])
data_a = []
for i, tr in enumerate(ORDER):
    for u in UAVS:
        l1 = K[tr]["per_uav"][u]["wind"]["L1_kinematic_dataflash"]
        e = l1["by_sector_median_gs_minus_as_mps"]["east"]
        w = l1["by_sector_median_gs_minus_as_mps"]["west"]
        off = l1["scale_offset_removed_mps"]
        y = i + {"A1": 0.22, "B2": 0.0, "B3": -0.22}[u]
        ax.plot([w, e], [y, y], color="0.55", lw=0.6, zorder=1)
        ax.plot(e, y, MK[u], ms=3.4, mfc=C_E, mec=C_E, zorder=3)
        ax.plot(w, y, MK[u], ms=3.4, mfc="white", mec=C_W, mew=0.9, zorder=3)
        ax.plot(off, y, "|", ms=5, color="0.25", mew=0.8, zorder=2)
        data_a.append({"trial": tr, "uav": u, "east_median_gs_minus_as_mps": e,
                       "west_median_gs_minus_as_mps": w,
                       "heading_independent_offset_mps": off,
                       "east_minus_west_mps": l1["east_minus_west_mps"],
                       "n_cruise_gps_samples": l1["n_cruise_gps_samples"]})
for x in (0.0,):
    ax.axvline(x, color="0.3", lw=0.6, ls=":", zorder=0)
for i, tr in enumerate(ORDER):
    ew = K[tr]["per_uav"]["B3"]["wind"]["L1_kinematic_dataflash"]["east_minus_west_mps"]
    ax.annotate(f"東−西 = {ew:+.2f} m/s", xy=(0.985, i + 0.40), xycoords=("axes fraction", "data"),
                ha="right", va="center", fontsize=6.8, color="0.25")
ax.set_yticks(range(len(ORDER)))
ax.set_yticklabels([ROWLAB[t] for t in ORDER])
ax.set_ylim(-0.65, len(ORDER) - 0.20)
ax.invert_yaxis()
ax.set_xlabel("巡航段 地速 − 空速 之中位數 (m/s)")
ax.set_title("(a) 風的動力學證據：航向分向後的地速−空速", loc="left")
ax.legend(handles=[
    Line2D([], [], marker="o", ls="", mfc=C_E, mec=C_E, ms=3.4, label="東向 45–135°"),
    Line2D([], [], marker="o", ls="", mfc="white", mec=C_W, mew=0.9, ms=3.4, label="西向 225–315°"),
    Line2D([], [], marker="|", ls="", color="0.25", ms=5, label="兩者中點＝空速刻度偏置"),
    Line2D([], [], marker="", ls="", label="形狀：○ A1、□ B2、△ B3"),
], loc="upper left", frameon=False, handletextpad=0.5, borderpad=0.2, labelspacing=0.25)

# ------------------------------------------------------- (b) 忽略風場的代價 --
ax2 = fig.add_subplot(gs[0, 1])
cells = ["HV4W00", "HV4W05", "HV4W55"]
data_b = []
for i, tr in enumerate(cells):
    p, fl = T[tr]["planned_makespan_s"], T[tr]["flown_makespan_2m_s"]
    ax2.bar(i - 0.19, p, 0.36, color=C_PLAN, ec="none")
    ax2.bar(i + 0.19, fl, 0.36, color=C_FLOWN, ec="none")
    ax2.annotate(f"{T[tr]['makespan_err_2m_pct']:+.2f}%", xy=(i + 0.19, fl + 6),
                 ha="center", va="bottom", fontsize=7.2, color=C_FLOWN)
    data_b.append({"trial": tr, "planned_makespan_s": p, "flown_makespan_s": fl,
                   "prediction_error_pct": T[tr]["makespan_err_2m_pct"]})
d = D["wind_cost"]["A_cost_of_the_air_same_plan_HV4W05_minus_HV4W00"]
y0 = T["HV4W00"]["flown_makespan_2m_s"]
y1 = T["HV4W05"]["flown_makespan_2m_s"]
ax2.hlines(y0, 0.19, 1.42, lw=0.6, ls=(0, (3, 2)), color="#bc4b51", zorder=4)
ax2.annotate("", xy=(1.42, y1), xytext=(1.42, y0), zorder=4,
             arrowprops=dict(arrowstyle="<->", lw=0.7, color="#bc4b51",
                             shrinkA=0, shrinkB=0))
ax2.annotate(f"同一份計畫，只換空氣\n+{d['d_flown_makespan_s']:.1f} s ({d['d_flown_makespan_pct']:+.2f}%)",
             xy=(1.42, 0.5 * (y0 + y1)), xytext=(0.50, 1356), ha="left", va="top",
             fontsize=6.8, color="#bc4b51",
             arrowprops=dict(arrowstyle="-", lw=0.5, color="#bc4b51",
                             connectionstyle="arc3,rad=-0.2", shrinkB=2))
ax2.set_xticks(range(len(cells)))
ax2.set_xticklabels(["規劃 0\n模擬 0", "規劃 0\n模擬 5", "規劃 5\n模擬 5"])
ax2.set_ylim(1150, 1372)
ax2.set_ylabel("機隊完工時間 makespan (s)")
ax2.set_title("(b) 忽略風場的代價", loc="left")
ax2.legend(handles=[matplotlib.patches.Patch(fc=C_PLAN, ec="none", label="規劃值"),
                    matplotlib.patches.Patch(fc=C_FLOWN, ec="none", label="SITL 實飛")],
           loc="upper left", frameon=False, handlelength=1.2, handletextpad=0.5,
           borderpad=0.2, labelspacing=0.25)

PNG, SVG = OUT_DIR / "fig4_wind.png", OUT_DIR / "fig4_wind.svg"
fig.savefig(PNG, dpi=300, metadata={"Software": None})
fig.savefig(SVG, metadata={"Date": None, "Creator": None})

srcs = {rel(p): sha256(p) for p in sorted(ANA.glob("*.json"))}
(HERE / "fig4_data.json").write_text(json.dumps({
    "figure": {"png": rel(PNG), "svg": rel(SVG), "width_cm": 17.0,
               "height_cm": 6.0, "dpi": 300},
    "panel_a_wind_evidence": data_a,
    "panel_b_makespan": data_b,
    "panel_b_derived": D["wind_cost"],
    "definitions": {
        "gs_minus_as": ("dataflash GPS.Spd minus ARSP.Airspeed, interpolated onto the "
                        "GPS samples, restricted to RelHomeAlt > 100 m and airspeed > "
                        "10 m/s, median per heading sector"),
        "east_minus_west": ("cancels any heading-independent groundspeed/airspeed scale "
                            "offset exactly; equals 2W for a pure east-west wind"),
        "makespan_flown": ("last downward crossing of RelHomeAlt = 2.0 m across the fleet, "
                           "on the GPS time-of-week clock, minus the plan's own t = 0"),
    },
    "caveats": [
        "SITL 內建 --model quadplane；SIM_WIND_TURB = 0、SIM_WIND_T = 1（等高度常值風），"
        "陣風、紊流與風切完全沒有模擬。",
        "5 m/s 是 Scenario.wind 的預設值，不是場址實測風。",
        "作業段條帶為東西向而風亦為東西向 ⇒ 沿航跡分量在往返之間相消；"
        "本圖不能推廣到條帶與風不平行的情況。",
        "HV4SITL16 規劃假設 5 m/s 但模擬從未設風（本圖 (a) 直接量到東−西 ≈ 0），"
        "這正是圖 (b) 三格要取代它的理由。",
        "每格 1 趟，SITL 為決定性模擬 ⇒ 沒有重複量測的散布。",
    ],
    "sources": srcs,
}, indent=1, ensure_ascii=False, sort_keys=True), encoding="utf-8")
print("wrote", rel(PNG), rel(SVG), rel(HERE / "fig4_data.json"))
