"""AASRC 2026 加長型摘要 圖3：HV4SITL16（1 Pioneer + 2 HERO）規劃 vs SITL 實飛。

    python docs/hv_dccpp/AASRC2026_abstract/figures_src/make_fig_results.py

輸出
    docs/hv_dccpp/AASRC2026_abstract/fig3_results.png   (17 cm 寬, 300 dpi)
    docs/hv_dccpp/AASRC2026_abstract/fig3_results.svg
    docs/hv_dccpp/AASRC2026_abstract/figures_src/fig3_data.json  (圖中每個數值 + 出處)

原則
- 圖中每個數值都由本腳本**直接讀 runs/artifacts/HV4SITL16/ 下的原始檔**算出，
  不手抄任何資料數字。演算法與 runs/artifacts/HV4SITL15/hv_sitl_report.py 相同
  （逐行標註出處行號），但**不 import** 那支腳本（它會連帶 import scripts/ 並在
  main() 內寫檔）。
- 間距門檻 d_sep_m / dh_sep_m 以 ast 解析 hv_dccpp/mission.py 的 PlannerConfig 預設值，
  不 import 套件（避免在 repo 內產生 __pycache__）。
- 唯讀：只寫上面三個輸出檔。
- 決定性：json 以 sort_keys 輸出、不含時間戳；SVG 去除日期並固定 hashsalt。
"""
from __future__ import annotations

import ast
import csv
import hashlib
import json
import math
import sys
from pathlib import Path

sys.dont_write_bytecode = True

import matplotlib  # noqa: E402

matplotlib.use("Agg")
from matplotlib import font_manager as fm  # noqa: E402
import matplotlib.ticker  # noqa: E402,F401
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

HERE = Path(__file__).resolve().parent                 # .../AASRC2026_abstract/figures_src
OUT_DIR = HERE.parent                                  # .../AASRC2026_abstract
ROOT = HERE.parents[3]                                 # repo root
RUN = ROOT / "runs" / "artifacts" / "HV4SITL16"
REPORT_ALGO = "runs/artifacts/HV4SITL15/hv_sitl_report.py"
MISSION_PY = ROOT / "hv_dccpp" / "mission.py"


def rel(p: Path) -> str:
    return p.resolve().relative_to(ROOT).as_posix()


def sha256(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def jload(p: Path):
    return json.loads(p.read_text(encoding="utf-8"))


# ----------------------------------------------------------------------------------------
# 字型：中文標楷體、英文數字 Times New Roman（matplotlib >= 3.6 逐字形 fallback）
# ----------------------------------------------------------------------------------------
KAIU = Path("C:/Windows/Fonts/kaiu.ttf")
TIMES = Path("C:/Windows/Fonts/times.ttf")
for f in (KAIU, TIMES):
    fm.fontManager.addfont(str(f))
KAIU_NAME = fm.FontProperties(fname=str(KAIU)).get_name()     # "DFKai-SB"
TIMES_NAME = fm.FontProperties(fname=str(TIMES)).get_name()   # "Times New Roman"
matplotlib.rcParams.update({
    "font.family": [TIMES_NAME, KAIU_NAME],
    "font.size": 8.0,
    "axes.titlesize": 8.5,
    "axes.labelsize": 8.0,
    "xtick.labelsize": 8.0,
    "ytick.labelsize": 8.0,
    "legend.fontsize": 8.0,
    "axes.unicode_minus": True,
    "axes.linewidth": 0.6,
    "xtick.major.width": 0.6,
    "ytick.major.width": 0.6,
    "xtick.major.size": 2.5,
    "ytick.major.size": 2.5,
    "xtick.direction": "in",
    "ytick.direction": "in",
    "svg.fonttype": "path",
    "svg.hashsalt": "fig3_results",
    "pdf.fonttype": 42,
})


# ----------------------------------------------------------------------------------------
# 原始資料
# ----------------------------------------------------------------------------------------
SCEN_F = RUN / "scenario.json"
MS_F = RUN / "plan" / "mission_summary.json"
REL_F = RUN / "release_schedule.json"
VAL_F = RUN / "validation.json"
SPAWN_F = RUN / "spawn_readback.json"

scen = jload(SCEN_F)
ms = jload(MS_F)
rel_sched = jload(REL_F)
val = jload(VAL_F)
spawn = jload(SPAWN_F)
UAVS = sorted(ms["uavs"])
CLS_NAME = {"A": "Pioneer", "B": "HERO"}


def planner_default(field: str) -> tuple[float, int]:
    """ast 解析 hv_dccpp/mission.py 的 PlannerConfig 欄位預設值與行號。"""
    tree = ast.parse(MISSION_PY.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == "PlannerConfig":
            for st in node.body:
                if (isinstance(st, ast.AnnAssign) and isinstance(st.target, ast.Name)
                        and st.target.id == field):
                    return float(ast.literal_eval(st.value)), st.lineno
    raise KeyError(field)


D_SEP, D_SEP_LINE = planner_default("d_sep_m")
DH_SEP, DH_SEP_LINE = planner_default("dh_sep_m")


def read_track(uav: str):
    """同 hv_sitl_report.py:_track（:59-65）：只留 lat、t_s 非空的列。"""
    f = RUN / uav / "flight.csv"
    rows = [r for r in csv.DictReader(f.open(encoding="utf-8")) if r["lat"] and r["t_s"]]
    return {
        "t": [float(r["t_s"]) for r in rows],
        "lat": [float(r["lat"]) for r in rows],
        "lon": [float(r["lon"]) for r in rows],
        "alt": [float(r["rel_alt_m"]) for r in rows],
        "armed": [(r["armed"].strip().lower() in ("true", "1")) if r.get("armed") else None
                  for r in rows],
        "mode": [r["mode"] for r in rows],
    }


def read_planned(uav: str):
    """同 hv_sitl_report.py:_planned（:116-125）：QGC WPL 第 9/10 欄 lat/lon，排除 lat=0；
    另取第 11 欄高度的最大值。"""
    lat, lon, alt = [], [], []
    for ln in (RUN / "plan" / f"{uav}.waypoints").read_text(encoding="utf-8").splitlines()[1:]:
        if ln.startswith("#"):
            continue
        f = ln.split("\t")
        if len(f) >= 11 and float(f[8]) != 0.0:
            lat.append(float(f[8]))
            lon.append(float(f[9]))
            alt.append(float(f[10]))
    return lat, lon, alt


def clock_offset(tracks) -> tuple[float, dict]:
    """同 hv_sitl_report.py:_clock_offset（:83-113）。"""
    sp = float(rel_sched.get("speedup") or 1.0)
    act = rel_sched.get("actual_release_wall_s") or {}
    per = {}
    for uav in UAVS:
        if uav not in act:
            continue
        tr = tracks[uav]
        i = next((k for k, m in enumerate(tr["mode"]) if m == "AUTO"), None)
        if i is not None:
            per[uav] = tr["t"][i] - float(act[uav]) * sp
    offs = sorted(per.values())
    return offs[len(offs) // 2], per


def flown_km(lon, lat) -> float:
    """同 hv_sitl_report.py:_flown_km（:128-135）。"""
    d = 0.0
    for i in range(1, len(lat)):
        dy = (lat[i] - lat[i - 1]) * 111132.0
        dx = (lon[i] - lon[i - 1]) * 111412.0 * math.cos(math.radians(lat[i]))
        d += math.hypot(dx, dy)
    return d / 1000.0


def touchdown_armed(t, alt, armed) -> float:
    """同 hv_sitl_report.py:_touchdown_s（:68-80）：爬過 20 m 後 <2 m 且已上鎖。"""
    peak = 0.0
    for ts, a, ar in zip(t, alt, armed):
        peak = max(peak, a)
        if peak > 20.0 and a < 2.0 and ar is False:
            return ts
    return float("nan")


def min_separation(series: dict, step_s: float = 2.0) -> dict:
    """同 hv_sitl_report.py:_min_separation（:179-262）；另回傳逐點樣本供作圖。"""

    def interp(t, xs, q):
        i = 0
        out = []
        for u in q:
            while i + 2 < len(t) and t[i + 1] < u:
                i += 1
            if u < t[0] or u > t[-1]:
                out.append(None)
                continue
            span = t[i + 1] - t[i]
            f = 0.0 if span <= 0 else (u - t[i]) / span
            out.append(xs[i] + f * (xs[i + 1] - xs[i]))
        return out

    uavs = sorted(series)
    lo = min(min(s[0]) for s in series.values())
    hi = max(max(s[0]) for s in series.values())
    grid = [lo + i * step_s for i in range(int((hi - lo) / step_s) + 1)]
    g = {u: tuple(interp(series[u][0], series[u][k], grid) for k in (1, 2, 3)) for u in uavs}

    best, pair_best, t_best, dv_best = float("inf"), None, None, None
    n_viol, n_air, per_pair, samples = 0, 0, {}, {}
    for i, a in enumerate(uavs):
        for b in uavs[i + 1:]:
            key = f"{a}-{b}"
            pmin, pdv, pv, pts = float("inf"), None, 0, []
            for k, tk in enumerate(grid):
                la, oa, aa = g[a][0][k], g[a][1][k], g[a][2][k]
                lb, ob, ab = g[b][0][k], g[b][1][k], g[b][2][k]
                if None in (la, oa, aa, lb, ob, ab) or aa <= 2.0 or ab <= 2.0:
                    continue
                n_air += 1
                dy = (la - lb) * 111132.0
                dx = (oa - ob) * 111412.0 * math.cos(math.radians(la))
                d = math.hypot(dx, dy)
                dv = abs(aa - ab)
                pts.append((d, dv))
                if d < D_SEP and dv < DH_SEP:
                    pv += 1
                    n_viol += 1
                if d < pmin:
                    pmin, pdv = d, dv
                if d < best:
                    best, pair_best, t_best, dv_best = d, key, tk, dv
            per_pair[key] = {"n_samples": len(pts), "n_violations": pv,
                             "min_h_m": pmin if pts else None,
                             "d_v_at_min_m": pdv}
            samples[key] = pts
    return {"min_h_m": best, "d_v_at_min_m": dv_best, "pair": pair_best, "t_s": t_best,
            "n_airborne_samples": n_air, "n_violations": n_viol,
            "per_pair": per_pair, "samples": samples, "step_s": step_s}


tracks = {u: read_track(u) for u in UAVS}
offset, offset_per = clock_offset(tracks)
for u in UAVS:
    tracks[u]["t"] = [x - offset for x in tracks[u]["t"]]

# 局部平面：同 hv_sitl_report.py:_coverage_from_tracks（:153-157）
# 原點取 mission_summary.json home（=報告腳本寫死的 23.95, 120.90）
LAT_H = float(ms["home"]["lat_deg"])
LON_H = float(ms["home"]["lon_deg"])
LAT0 = float(scen["areas"][0]["polygon"][0][0])
M_LAT, M_LON = 111132.0, 111412.0 * math.cos(math.radians(LAT0))


def xy_km(lat, lon):
    return [(o - LON_H) * M_LON / 1000.0 for o in lon], [(a - LAT_H) * M_LAT / 1000.0 for a in lat]


stats = {}
for u in UAVS:
    tr = tracks[u]
    plat, plon, palt = read_planned(u)
    um = ms["uavs"][u]
    t_air2 = next((ts for ts, a in zip(tr["t"], tr["alt"]) if a > 2.0), None)
    peak = 0.0
    t_lt2 = float("nan")
    for ts, a in zip(tr["t"], tr["alt"]):
        peak = max(peak, a)
        if peak > 20.0 and a < 2.0:
            t_lt2 = ts
            break
    stats[u] = {
        "cls": um["cls"],
        "plan_km": um["distance_m"] / 1000.0,
        "flown_km": flown_km(tr["lon"], tr["lat"]),
        "plan_wp_max_alt_m": max(palt),
        "flown_max_alt_m": max(tr["alt"]),
        "plan_t_takeoff_s": float(um["t_takeoff_s"]),
        "plan_t_land_s": float(um["t_land_s"]),
        "flown_t_first_above2m_s": t_air2,
        "flown_t_touchdown_armed_s": touchdown_armed(tr["t"], tr["alt"], tr["armed"]),
        "flown_t_touchdown_lt2m_s": t_lt2,
        "plan_latlon": (plat, plon),
    }

sep = min_separation({u: (tracks[u]["t"], tracks[u]["lat"], tracks[u]["lon"], tracks[u]["alt"])
                      for u in UAVS})
H_CEIL = float(scen["h_ceil_m"])
PLAN_MIN_SEP = float(val["min_sep_h_m"])

# ----------------------------------------------------------------------------------------
# 作圖
# ----------------------------------------------------------------------------------------
CM = 1 / 2.54
# 逐機：顏色 + 線型（黑白列印時靠線型分辨）。Pioneer 暖色、HERO 冷色。
STYLE = {
    "A1": {"color": "#B5470B", "ls": "-"},
    "B2": {"color": "#1B4F91", "ls": (0, (3.2, 1.4))},
    "B3": {"color": "#127A5C", "ls": (0, (5.0, 1.3, 1.1, 1.3))},
}

fig = plt.figure(figsize=(17.0 * CM, 6.2 * CM))
gs = fig.add_gridspec(1, 3, width_ratios=[1.52, 1.27, 1.0], wspace=0.30,
                      left=0.052, right=0.988, bottom=0.175, top=0.80)
axA = fig.add_subplot(gs[0])
axB = fig.add_subplot(gs[1])
axC = fig.add_subplot(gs[2])

# ---- (a) 平面航跡 ----
poly = scen["areas"][0]["polygon"]
px, py = xy_km([p[0] for p in poly] + [poly[0][0]], [p[1] for p in poly] + [poly[0][1]])
axA.plot(px, py, color="0.25", lw=0.7, ls=(0, (2.0, 1.2)), zorder=1)
for u in UAVS:
    s, st = stats[u], STYLE[u]
    x, y = xy_km(*s["plan_latlon"])
    axA.plot(x, y, color=st["color"], lw=2.6, alpha=0.22, solid_capstyle="round",
             solid_joinstyle="round", zorder=2)
for u in UAVS:
    st = STYLE[u]
    x, y = xy_km(tracks[u]["lat"], tracks[u]["lon"])
    axA.plot(x, y, color=st["color"], lw=0.65, ls=st["ls"], zorder=3)
hx, hy = xy_km([LAT_H], [LON_H])
axA.plot(hx, hy, marker="s", ms=3.6, mfc="white", mec="black", mew=0.8, ls="none", zorder=5)
all_x = [v for u in UAVS for v in xy_km(tracks[u]["lat"], tracks[u]["lon"])[0]]
all_y = [v for u in UAVS for v in xy_km(tracks[u]["lat"], tracks[u]["lon"])[1]]
ybot = min(all_y) - 0.22                      # 下方留白放標註，不壓航跡
axA.annotate("H 起降點", (hx[0], hy[0]), xytext=(min(all_x) + 0.02, ybot + 0.07),
             fontsize=8, ha="left", va="center", zorder=6,
             arrowprops=dict(arrowstyle="-", lw=0.5, color="black", shrinkA=1, shrinkB=3))
axA.text(max(px), min(all_y) - 0.04, "作業區（虛線框）", fontsize=8, ha="right", va="top",
         color="0.15", zorder=6)
axA.set_xlim(min(all_x) - 0.05, max(all_x) + 0.05)
axA.set_ylim(ybot, max(all_y) + 0.06)
axA.set_aspect("equal", adjustable="datalim")
axA.set_xlabel("東 E (km)", labelpad=1.5)
axA.set_ylabel("北 N (km)", labelpad=1.5)
axA.set_title("(a) 平面航跡（寬淡線：規劃；細線：飛行）", loc="left", pad=3)

# ---- (b) 高度對時間 ----
axB.axhline(H_CEIL, color="black", lw=0.8, ls=(0, (1.0, 1.0)), zorder=1)
for u in UAVS:
    # 只畫空中段（第一筆 >2 m 的前一筆 → 上鎖觸地那一筆），地面等待段貼在 x 軸上會蓋住 ▲▼
    st = STYLE[u]
    t, a = tracks[u]["t"], tracks[u]["alt"]
    i0 = max(0, next(k for k, v in enumerate(a) if v > 2.0) - 1)
    td = stats[u]["flown_t_touchdown_armed_s"]
    i1 = max(k for k, ts in enumerate(t) if ts <= td) if td == td else len(t) - 1
    axB.plot(t[i0:i1 + 1], a[i0:i1 + 1], color=st["color"], lw=0.8, ls=st["ls"], zorder=3)
for u in UAVS:
    s, st = stats[u], STYLE[u]
    axB.plot([s["plan_t_takeoff_s"]], [0], marker="^", ms=3.8, mfc="white", mec=st["color"],
             mew=0.9, ls="none", zorder=4, clip_on=False)
    axB.plot([s["plan_t_land_s"]], [0], marker="v", ms=3.8, mfc=st["color"], mec=st["color"],
             mew=0.9, ls="none", zorder=4, clip_on=False)
peaks = "/".join(f"{stats[u]['flown_max_alt_m']:.2f}" for u in UAVS)
axB.text(0.02, H_CEIL + 3, f"法規上限 {H_CEIL:.0f} m，不豁免", fontsize=8, ha="left",
         va="bottom", transform=axB.get_yaxis_transform())
axB.text(0.02, H_CEIL + 21, f"飛行最高 {peaks} m", fontsize=8, ha="left",
         va="bottom", transform=axB.get_yaxis_transform())
axB.set_ylim(-6, 172)
axB.set_yticks([0, 40, 80, 120])
tmax = max(stats[u]["flown_t_touchdown_armed_s"] for u in UAVS)
axB.set_xlim(-30, tmax + 40)
axB.set_xlabel("任務時間 (s)", labelpad=1.5)
axB.set_ylabel("相對高度 AGL (m)", labelpad=1.5)
axB.set_title("(b) 高度（△▼：規劃起降時刻）", loc="left", pad=3)

# ---- (c) 水平 × 垂直間距 ----
# x 取對數軸：線性軸到 2 km 時 100 m 違規框只剩一條細縫，看不出最小點貼著門檻
_dmin = min(p[0] for v in sep["samples"].values() for p in v)
X_LO = D_SEP / 2 if _dmin > D_SEP / 2 else 10 ** math.floor(math.log10(max(_dmin, 1.0) / 2))
X_HI = 1.2 * max(p[0] for v in sep["samples"].values() for p in v)
axC.set_xscale("log")
axC.add_patch(Rectangle((X_LO, 0), D_SEP - X_LO, DH_SEP, fc="0.90", ec="black", lw=0.6,
                        hatch="////", zorder=1))
axC.text(X_LO * 1.08, DH_SEP + 3, "違規區", fontsize=8, ha="left", va="bottom", zorder=6)
PAIR_MK = {"A1-B3": dict(marker="^", ms=2.6), "B2-B3": dict(marker="o", ms=2.4)}
for key, pts in sep["samples"].items():
    if not pts:
        continue
    mk = PAIR_MK.get(key, dict(marker="s", ms=2.4))
    axC.plot([p[0] for p in pts], [p[1] for p in pts], ls="none", mfc="none", mec="0.45",
             mew=0.45, zorder=3, **mk)
axC.plot([sep["min_h_m"]], [sep["d_v_at_min_m"]], marker="o", ms=4.2, mfc="black", mec="black",
         ls="none", zorder=5)
axC.annotate(f"最小 {sep['min_h_m']:.1f} m\n垂直 {sep['d_v_at_min_m']:.1f} m\n"
             f"{sep['pair'].replace('-', '–')}",
             (sep["min_h_m"], sep["d_v_at_min_m"]), xytext=(0.03, 0.97),
             textcoords="axes fraction", fontsize=8, ha="left", va="top", zorder=6,
             arrowprops=dict(arrowstyle="-", lw=0.6, color="black", shrinkA=1, shrinkB=2.5),
             bbox=dict(boxstyle="square,pad=0.12", fc="white", ec="none", alpha=0.95))
pair_h = [Line2D([], [], ls="none", mfc="none", mec="0.30", mew=0.6,
                 label=k.replace("-", "–"), **{**PAIR_MK.get(k, dict(marker="s", ms=2.4)),
                                               "ms": PAIR_MK.get(k, dict(ms=2.4))["ms"] + 1.2})
          for k in sorted(sep["samples"]) if sep["samples"][k]]
lg = axC.legend(handles=pair_h, loc="upper right", frameon=True, fancybox=False,
                framealpha=0.95, edgecolor="none", borderpad=0.25, handletextpad=0.2,
                labelspacing=0.15, handlelength=1.0, borderaxespad=0.2,
                title=f"違規 {sep['n_violations']}/{sep['n_airborne_samples']}",
                title_fontsize=8)
lg._legend_box.align = "left"
axC.set_xlim(X_LO, X_HI)
# 不標軸下限 50：避免讀成「違規區自 50 m 起」，也避免與 y 軸的 0 相撞
axC.set_xticks([t for t in (100, 300, 1000, 3000) if X_LO < t <= X_HI])
axC.xaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f"{v:g}"))
axC.xaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
axC.set_xlabel("水平間距 (m，對數軸)", labelpad=1.5)
axC.set_ylabel("垂直間距 (m)", labelpad=1.5)
_vmax = max(p[1] for v in sep["samples"].values() for p in v)
axC.set_ylim(0, _vmax + 42)                    # 上方留白放圖例與最小點標註，不遮散點
axC.set_yticks([0, 25, 50, 75, 100])
axC.set_title("(c) 同時在空中的機間間距", loc="left", pad=3)

for ax in (axA, axB, axC):
    ax.tick_params(pad=1.8)

# ---- 共用圖例（頂端） ----
handles = []
for u in UAVS:
    s, st = stats[u], STYLE[u]
    handles.append(Line2D([], [], color=st["color"], lw=1.0, ls=st["ls"],
                          label=f"{u} {CLS_NAME[s['cls']]} {s['plan_km']:.2f}→{s['flown_km']:.2f} km"))
LEG_Y = 0.955
lead = fig.text(0.012, LEG_Y, "各機航程（規劃→飛行）：", ha="left", va="center", fontsize=8)
fig.canvas.draw()
lead_bb = lead.get_window_extent().transformed(fig.transFigure.inverted())
fig.legend(handles=handles, loc="center left", bbox_to_anchor=(lead_bb.x1 + 0.004, LEG_Y),
           ncol=len(handles), frameon=False, handlelength=2.6, handletextpad=0.45,
           columnspacing=1.6, borderaxespad=0.0, borderpad=0.0)

PNG = OUT_DIR / "fig3_results.png"
SVG = OUT_DIR / "fig3_results.svg"
fig.savefig(PNG, dpi=300, metadata={"Software": None})
fig.savefig(SVG, metadata={"Date": None, "Creator": None})

# ----------------------------------------------------------------------------------------
# fig3_data.json：圖中每個數值與出處
# ----------------------------------------------------------------------------------------
def src(path: Path, field: str, algo: str | None = None) -> dict:
    d = {"file": rel(path), "field": field}
    if algo:
        d["algorithm"] = algo
    return d


items = []
for u in UAVS:
    s = stats[u]
    items += [
        {"panel": "legend", "label": f"{u} 規劃航程", "value": round(s["plan_km"], 3), "unit": "km",
         "shown_as": f"{s['plan_km']:.2f}", "raw": s["plan_km"],
         "source": src(MS_F, f"uavs.{u}.distance_m", "÷1000")},
        {"panel": "legend", "label": f"{u} 實飛航程", "value": round(s["flown_km"], 3), "unit": "km",
         "shown_as": f"{s['flown_km']:.2f}", "raw": s["flown_km"],
         "source": src(RUN / u / "flight.csv", "lat, lon（全部列，lat 與 t_s 非空）",
                       f"{REPORT_ALGO}:128-135 _flown_km")},
        {"panel": "b", "label": f"{u} 實飛最高相對高度", "value": round(s["flown_max_alt_m"], 2),
         "unit": "m", "shown_as": f"{s['flown_max_alt_m']:.2f}", "raw": s["flown_max_alt_m"],
         "source": src(RUN / u / "flight.csv", "max(rel_alt_m)")},
        {"panel": "b", "label": f"{u} 規劃起飛時刻（空心△）", "value": s["plan_t_takeoff_s"], "unit": "s",
         "shown_as": "marker", "source": src(MS_F, f"uavs.{u}.t_takeoff_s")},
        {"panel": "b", "label": f"{u} 規劃降落時刻（▼）", "value": round(s["plan_t_land_s"], 2),
         "unit": "s", "shown_as": "marker", "raw": s["plan_t_land_s"],
         "source": src(MS_F, f"uavs.{u}.t_land_s")},
        {"panel": "b", "label": f"{u} 時基偏移（逐機）", "value": round(offset_per[u], 2), "unit": "s",
         "shown_as": "not shown (用於 x 軸校正)", "raw": offset_per[u],
         "source": src(RUN / u / "flight.csv", "t_s of first mode==AUTO row − "
                       "release_schedule.json:actual_release_wall_s[uav]×speedup",
                       f"{REPORT_ALGO}:83-113 _clock_offset")},
    ]
items += [
    {"panel": "b", "label": "時基偏移（中位數，已自所有實飛時刻扣除）", "value": round(offset, 2),
     "unit": "s", "shown_as": "not shown (x 軸 = t_s − offset)", "raw": offset,
     "source": src(REL_F, "speedup, actual_release_wall_s", f"{REPORT_ALGO}:83-113 _clock_offset")},
    {"panel": "b", "label": "法規上限線 h_ceil", "value": H_CEIL, "unit": "m", "shown_as": f"{H_CEIL:.0f}",
     "source": src(SCEN_F, "h_ceil_m（waiver_max_agl_m = null ⇒ 不豁免）")},
    {"panel": "c", "label": "違規門檻 水平 d_sep_m", "value": D_SEP, "unit": "m",
     "shown_as": "違規區矩形寬", "source": {"file": "hv_dccpp/mission.py", "line": D_SEP_LINE,
                                           "field": "PlannerConfig.d_sep_m"}},
    {"panel": "c", "label": "違規門檻 垂直 dh_sep_m", "value": DH_SEP, "unit": "m",
     "shown_as": "違規區矩形高", "source": {"file": "hv_dccpp/mission.py", "line": DH_SEP_LINE,
                                           "field": "PlannerConfig.dh_sep_m"}},
    {"panel": "c", "label": "實飛最小水平間距", "value": round(sep["min_h_m"], 1), "unit": "m",
     "shown_as": f"{sep['min_h_m']:.1f}", "raw": sep["min_h_m"],
     "source": src(RUN, "{A1,B2,B3}/flight.csv t_s, lat, lon, rel_alt_m",
                   f"{REPORT_ALGO}:179-262 _min_separation（2 s 格點線性內插、兩機皆 >2 m AGL）")},
    {"panel": "c", "label": "最小點的垂直間距", "value": round(sep["d_v_at_min_m"], 1), "unit": "m",
     "shown_as": f"{sep['d_v_at_min_m']:.1f}", "raw": sep["d_v_at_min_m"],
     "source": src(RUN, "同上", f"{REPORT_ALGO}:179-262")},
    {"panel": "c", "label": "最小點機對", "value": sep["pair"], "unit": None,
     "shown_as": sep["pair"].replace("-", "–"), "source": src(RUN, "同上", f"{REPORT_ALGO}:179-262")},
    {"panel": "c", "label": "最小點時刻（任務時間）", "value": round(sep["t_s"], 1), "unit": "s",
     "shown_as": "not shown", "raw": sep["t_s"], "source": src(RUN, "同上", f"{REPORT_ALGO}:179-262")},
    {"panel": "c", "label": "違規樣本數", "value": sep["n_violations"], "unit": "samples",
     "shown_as": str(sep["n_violations"]), "source": src(RUN, "同上", f"{REPORT_ALGO}:179-262")},
    {"panel": "c", "label": "兩機皆在空中的樣本數（所有機對合計）", "value": sep["n_airborne_samples"],
     "unit": "samples", "shown_as": str(sep["n_airborne_samples"]),
     "source": src(RUN, "同上", f"{REPORT_ALGO}:179-262")},
]
for key, pp in sorted(sep["per_pair"].items()):
    items.append({"panel": "c", "label": f"機對 {key}", "value": {
        "n_samples": pp["n_samples"], "n_violations": pp["n_violations"],
        "min_h_m": None if pp["min_h_m"] is None else round(pp["min_h_m"], 1),
        "d_v_at_min_m": None if pp["d_v_at_min_m"] is None else round(pp["d_v_at_min_m"], 1)},
        "unit": "m / samples",
        "shown_as": ("散點" if pp["n_samples"] else "無散點：兩機從未同時在空中（>2 m AGL）"),
        "source": src(RUN, "同上", f"{REPORT_ALGO}:179-262")})

context = [
    {"label": "規劃最小水平間距（未畫於圖，供圖說/表對照）", "value": round(PLAN_MIN_SEP, 2), "unit": "m",
     "raw": PLAN_MIN_SEP, "source": src(VAL_F, "min_sep_h_m")},
    {"label": "航點最高高度（未畫於圖）", "value": {u: round(stats[u]["plan_wp_max_alt_m"], 3) for u in UAVS},
     "unit": "m", "source": src(RUN / "plan", "{uav}.waypoints 第 11 欄最大值（排除 lat=0 列）")},
    {"label": "實飛觸地時刻（上鎖判準；未標於圖）",
     "value": {u: stats[u]["flown_t_touchdown_armed_s"] for u in UAVS}, "unit": "s",
     "source": src(RUN, "{uav}/flight.csv", f"{REPORT_ALGO}:68-80 _touchdown_s")},
    {"label": "實飛觸地時刻（<2 m 判準；未標於圖）",
     "value": {u: stats[u]["flown_t_touchdown_lt2m_s"] for u in UAVS}, "unit": "s",
     "source": src(RUN, "{uav}/flight.csv", "同 _touchdown_s 但不看 armed")},
    {"label": "航程合計 規劃→實飛（未畫於圖）",
     "value": {"plan_km": round(sum(stats[u]["plan_km"] for u in UAVS), 3),
               "flown_km": round(sum(stats[u]["flown_km"] for u in UAVS), 3)},
     "unit": "km", "source": "同 legend 各機"},
]

data = {
    "figure": {"png": rel(PNG), "svg": rel(SVG), "width_cm": 17.0, "height_cm": 6.2, "dpi": 300,
               "script": rel(Path(__file__))},
    "run": rel(RUN),
    "projection": {"formula": "x=(lon−lon_H)·111412·cos(lat0), y=(lat−lat_H)·111132",
                   "lat_H": LAT_H, "lon_H": LON_H, "lat0": LAT0,
                   "source": [src(MS_F, "home.lat_deg, home.lon_deg"),
                              src(SCEN_F, "areas[0].polygon[0][0]"),
                              f"{REPORT_ALGO}:153-157"]},
    "items": items,
    "context_not_plotted": context,
    "caveats": [
        "SITL 內建 --model quadplane；單一平地矩形作業區；5 倍速；每條件 1 趟（HV4SITL15 為同規劃重飛）。",
        "SITL 未設風而規劃假設 wind=[5.0, 270.0]（scenario.json）⇒ 時間類對照可能有東西向抵銷假象。",
        "最小間距為約 1 Hz 遙測在 2 s 格點線性內插的上界；主結果僅高出門檻約 1 m，重飛（HV4SITL15）為 104.2 m。",
        "B2–B3 為 HERO–HERO，依設計沒有垂直隔離通道（hv_dccpp/mission.py h_transit_offset_A_m 只作用於 A 類）。",
        "實飛航程 = 遙測點逐段累加；時間軸已扣除時基偏移（逐機偏移差 ≤0.5 s，另有最多 6 s 取樣空洞）。",
        "HV4SITL16 目錄未納管（git ??）；下列 sha256 供核對檔案未被改動。",
        "作圖取捨：(b) 只畫各機空中段（第一筆 >2 m 的前一筆至上鎖觸地那一筆），地面等待段未畫；"
        "(c) 水平軸為對數軸，違規區實際自 0 m 起，框左緣只是軸下限；A1–B2 從未同時在空中故無散點。",
    ],
    "inputs_sha256": {rel(p): sha256(p) for p in sorted(
        [SCEN_F, MS_F, REL_F, VAL_F, SPAWN_F, MISSION_PY]
        + [RUN / u / "flight.csv" for u in UAVS]
        + [RUN / "plan" / f"{u}.waypoints" for u in UAVS])},
}
(HERE / "fig3_data.json").write_text(
    json.dumps(data, ensure_ascii=False, indent=1, sort_keys=True, allow_nan=True) + "\n",
    encoding="utf-8")

print("offset", offset, offset_per)
for u in UAVS:
    s = stats[u]
    print(u, {k: v for k, v in s.items() if k != "plan_latlon"})
print("sep", {k: v for k, v in sep.items() if k != "samples"})
print("wrote", PNG, SVG, HERE / "fig3_data.json")
