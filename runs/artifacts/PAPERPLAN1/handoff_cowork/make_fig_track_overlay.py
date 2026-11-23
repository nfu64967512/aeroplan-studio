"""有風／無風 SITL 航跡疊圖：HV4W05（規劃不含風、模擬 5 m/s 自 270°）對 HV4W00（規劃不含風、模擬無風）。

兩趟上傳的任務檔逐位元組相同（plan/*.waypoints），唯一差別是模擬風 ⇒ 航跡差異全部來自風。
資料：runs/artifacts/{HV4W00,HV4W05}/{A1,B2,B3}/flight.csv（約 1 Hz 遙測）。只取解鎖且相對高度 > 2 m 的點。
座標：以起飛點 HOME (23.95, 120.9) 為原點的局部東北座標，WGS84 橢球之子午／卯酉曲率半徑換算（非平面 111 km/°）。
偏移：有風航跡每一點到無風航跡（以 1 m 間距內插之折線）的最近距離；只算巡航段（相對高度 > 100 m）。
用法：python make_fig_track_overlay.py [--a HV4W00] [--b HV4W05]
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
from matplotlib import font_manager as fm          # noqa: E402
import matplotlib.pyplot as plt                     # noqa: E402
from matplotlib.patches import Rectangle            # noqa: E402

sys.stdout.reconfigure(encoding="utf-8")
HERE = Path(__file__).resolve().parent
ART = HERE.parents[1]                               # runs/artifacts
CM = 1 / 2.54
KAIU, TIMES = Path("C:/Windows/Fonts/kaiu.ttf"), Path("C:/Windows/Fonts/times.ttf")
for f in (KAIU, TIMES):
    fm.fontManager.addfont(str(f))
matplotlib.rcParams.update({
    "font.family": [fm.FontProperties(fname=str(TIMES)).get_name(),
                    fm.FontProperties(fname=str(KAIU)).get_name()],
    "font.size": 8.0, "axes.titlesize": 9.0, "axes.labelsize": 8.5,
    "xtick.labelsize": 8.0, "ytick.labelsize": 8.0, "legend.fontsize": 8.0,
    "axes.unicode_minus": True, "axes.linewidth": 0.6,
    "xtick.major.width": 0.6, "ytick.major.width": 0.6,
    "xtick.major.size": 2.5, "ytick.major.size": 2.5,
    "xtick.direction": "in", "ytick.direction": "in",
    "svg.fonttype": "path", "svg.hashsalt": "track_overlay", "pdf.fonttype": 42,
})
HOME = (23.95, 120.9)
UAVS = ["A1", "B2", "B3"]
NAME = {"A1": "A1 Pioneer", "B2": "B2 HERO", "B3": "B3 HERO"}
COL = {"A1": "#bc4b51", "B2": "#1b4965", "B3": "#2a9d8f"}
A, E2 = 6378137.0, 6.69437999014e-3


def enu(lat, lon):
    phi = math.radians(HOME[0])
    s = math.sin(phi)
    m_rad = A * (1 - E2) / (1 - E2 * s * s) ** 1.5
    n_rad = A / math.sqrt(1 - E2 * s * s)
    return ((np.asarray(lon) - HOME[1]) * math.radians(1) * n_rad * math.cos(phi),
            (np.asarray(lat) - HOME[0]) * math.radians(1) * m_rad)


def track(trial, uav):
    t, la, lo, alt = [], [], [], []
    with open(ART / trial / uav / "flight.csv", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if r["armed"] != "True" or float(r["rel_alt_m"]) <= 2.0:
                continue
            t.append(float(r["t_s"]))
            la.append(float(r["lat"]))
            lo.append(float(r["lon"]))
            alt.append(float(r["rel_alt_m"]))
    x, y = enu(la, lo)
    return np.array(t), x, y, np.array(alt)


def plan_xy(trial, uav):
    la, lo = [], []
    for line in (ART / trial / "plan" / f"{uav}.waypoints").read_text(encoding="utf-8").splitlines()[1:]:
        p = line.split("\t")
        if len(p) >= 11 and float(p[8]) != 0.0 and float(p[9]) != 0.0 and int(p[3]) in (16, 84, 85):
            la.append(float(p[8]))
            lo.append(float(p[9]))
    return enu(la, lo)


def densify(x, y, step=1.0):
    px, py = [x[0]], [y[0]]
    for i in range(1, len(x)):
        d = math.hypot(x[i] - x[i - 1], y[i] - y[i - 1])
        n = max(1, int(d / step))
        for k in range(1, n + 1):
            px.append(x[i - 1] + (x[i] - x[i - 1]) * k / n)
            py.append(y[i - 1] + (y[i] - y[i - 1]) * k / n)
    return np.array(px), np.array(py)


def deviation(xb, yb, xa, ya):
    """b 的每一點到 a（內插折線）的最近距離。"""
    dx, dy = densify(xa, ya)
    out = np.empty(len(xb))
    for i, (x, y) in enumerate(zip(xb, yb)):
        out[i] = np.min(np.hypot(dx - x, dy - y))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--a", default="HV4W00", help="基準（無風）")
    ap.add_argument("--b", default="HV4W05", help="比較（有風）")
    args = ap.parse_args()
    sa, sb = json.load(open(ART / args.a / "scenario.json", encoding="utf-8")), \
        json.load(open(ART / args.b / "scenario.json", encoding="utf-8"))
    same_plan = all((ART / args.a / "plan" / f"{u}.waypoints").read_bytes()
                    == (ART / args.b / "plan" / f"{u}.waypoints").read_bytes() for u in UAVS)

    tr = {(tt, u): track(tt, u) for tt in (args.a, args.b) for u in UAVS}
    stats, dev_all, worst = {}, {}, {"east": (0.0, None, None), "west": (0.0, None, None)}
    for u in UAVS:
        _, xa, ya, _ = tr[(args.a, u)]
        _, xb, yb, altb = tr[(args.b, u)]
        cruise = altb > 100.0
        d = deviation(xb[cruise], yb[cruise], xa, ya)
        dev_all[u] = d
        xs, ys = xb[cruise], yb[cruise]
        for side, mask in (("east", xs > 0), ("west", xs < 0)):
            if mask.any():
                i = int(np.argmax(np.where(mask, d, -1)))
                if d[i] > worst[side][0]:
                    worst[side] = (float(d[i]), float(xs[i]), float(ys[i]))
        stats[u] = {"n": int(d.size), "median_m": round(float(np.median(d)), 1),
                    "p95_m": round(float(np.percentile(d, 95)), 1), "max_m": round(float(d.max()), 1)}

    fig = plt.figure(figsize=(17.0 * CM, 12.0 * CM))
    ax1 = fig.add_axes([0.075, 0.635, 0.905, 0.315])
    axe = fig.add_axes([0.075, 0.07, 0.25, 0.30])
    axw = fig.add_axes([0.385, 0.07, 0.25, 0.30])
    ax4 = fig.add_axes([0.765, 0.07, 0.215, 0.30])

    for ax in (ax1, axe, axw):
        for u in UAVS:
            px, py = plan_xy(args.a, u)
            ax.plot(px, py, ls=":", lw=0.5, color="0.55", zorder=1)
            _, xa, ya, _ = tr[(args.a, u)]
            _, xb, yb, _ = tr[(args.b, u)]
            ax.plot(xa, ya, lw=1.8, color=COL[u], alpha=0.30, zorder=2, solid_capstyle="round")
            ax.plot(xb, yb, lw=0.75, color=COL[u], alpha=1.0, zorder=3)
        ax.set_aspect("equal", adjustable="datalim")
    h = [ax1.plot([], [], lw=1.8, color="0.3", alpha=0.35)[0], ax1.plot([], [], lw=0.75, color="0.1")[0],
         ax1.plot([], [], ls=":", lw=0.6, color="0.55")[0]] + [ax1.plot([], [], lw=2.0, color=COL[u])[0] for u in UAVS]
    def desc(sc):
        pw = (sc.get("wind") or [0, 0])[0]
        sw = (sc.get("sim_wind") or {}).get("spd_mps", 0)
        return f"規劃{'含' if pw else '不含'}風，模擬{f' {sw:g} m/s 西風' if sw else '無風'}"
    lab = [f"{args.a}（{desc(sa)}）", f"{args.b}（{desc(sb)}）", "規劃航點連線"] + [NAME[u] for u in UAVS]
    ax1.legend(h, lab, loc="upper center", bbox_to_anchor=(0.5, -0.24), ncol=3, frameon=False,
               handlelength=1.6, columnspacing=1.0)
    ax1.set_xlabel("東 (m)", labelpad=1)
    ax1.set_ylabel("北 (m)")
    ax1.set_title("(a) 航跡俯視：同一份任務檔，只差模擬風" if same_plan
                  else "(a) 航跡俯視：任務檔與模擬風皆不同（非單變因對照）", loc="left")
    ax1.annotate("", xy=(0.985, 0.10), xytext=(0.90, 0.10), xycoords="axes fraction",
                 arrowprops=dict(arrowstyle="-|>", lw=0.9, color="0.2"))
    ax1.text(0.9425, 0.13, "風 5 m/s", transform=ax1.transAxes, ha="center", va="bottom")

    half = 160.0
    for ax, side, title in ((axe, "east", "(b) 東端迴轉（順風側）"), (axw, "west", "(c) 西端迴轉（逆風側）")):
        dv, wx, wy = worst[side]
        ax.set_xlim(wx - half, wx + half)
        ax.set_ylim(wy - half, wy + half)
        ax.set_title(title, loc="left")
        ax.text(0.03, 0.03, f"最大偏移 {dv:.1f} m", transform=ax.transAxes, fontsize=8, va="bottom")
        ax.set_xlabel("東 (m)", labelpad=1)
        ax1.add_patch(Rectangle((wx - half, wy - half), 2 * half, 2 * half, fill=False, lw=0.6, ec="0.2", zorder=4))
        ax1.text(wx, wy + half + 15, title[1], ha="center", va="bottom", fontsize=8)
    axe.set_ylabel("北 (m)")

    dmax = 0.0
    for k, u in enumerate(UAVS):
        d = dev_all[u]
        ax4.boxplot([d], positions=[k], vert=False, widths=0.55, showfliers=True,
                    flierprops=dict(marker=".", ms=2, mec=COL[u], alpha=0.6),
                    medianprops=dict(color=COL[u], lw=1.2),
                    boxprops=dict(color=COL[u], lw=0.7), whiskerprops=dict(color=COL[u], lw=0.7),
                    capprops=dict(color=COL[u], lw=0.7))
        dmax = max(dmax, float(d.max()))
    ax4.set_yticks(range(len(UAVS)), UAVS)
    ax4.set_xlim(0, dmax * 1.08)
    ax4.set_xlabel("偏離無風航跡 (m)", labelpad=1)
    ax4.set_title("(d) 巡航段偏移", loc="left")
    for k, u in enumerate(UAVS):
        ax4.text(dmax * 1.06, k + 0.33, f"中位 {stats[u]['median_m']:.1f}，最大 {stats[u]['max_m']:.1f}",
                 ha="right", va="bottom", fontsize=8)
    ax4.set_ylim(-0.6, len(UAVS) - 0.1)

    stem = f"fig_track_overlay_{args.b}_vs_{args.a}"
    for ext in ("png", "svg"):
        fig.savefig(HERE / f"{stem}.{ext}", dpi=300)
    rec = {"a": args.a, "b": args.b, "same_plan_files": same_plan,
           "plan_wind": {args.a: sa.get("wind"), args.b: sb.get("wind")},
           "sim_wind": {args.a: sa.get("sim_wind"), args.b: sb.get("sim_wind")},
           "deviation_cruise_m": stats, "worst_points_m": {k: {"dev": round(v[0], 1), "x": round(v[1], 1), "y": round(v[2], 1)} for k, v in worst.items()},
           "method": "nearest distance from each b point (rel_alt>100 m) to the a track densified at 1 m; ENU from WGS84 radii at HOME"}
    (HERE / f"{stem}.json").write_text(json.dumps(rec, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps(rec, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
