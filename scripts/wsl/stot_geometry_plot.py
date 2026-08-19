#!/usr/bin/env python3
"""stot_geometry_plot.py — 把 STOT 表演的幾何畫出來(俯視圖 / 側視剖面 / 動畫)。

讀 stot_fast.env 之類的組態(或直接吃參數),畫出:
  1. 俯視圖:扇形 slot、pushover 環、各機徑向線、目標,並與舊組態同比例對照
  2. 側視剖面:perch 高度、俯衝角、PN 交接距、終端凍結半徑、速度標註
  3. GIF:ALIGN → 環上就位 → 同步俯衝的全程動畫(依設計參數解析生成)

⚠️ 圖上一律用英文標籤:SITL 環境沒有裝 CJK 字型,中文會變成方框。

用法:
  python3 stot_geometry_plot.py --out-dir ~/fixwing_ws/runs/figs
  python3 stot_geometry_plot.py --no-gif        # 只出靜態圖
"""
import argparse
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge, Circle, FancyArrowPatch
import matplotlib.animation as animation
import numpy as np

# ── 兩組幾何 ─────────────────────────────────────────────────────────
OLD = dict(name="baseline (stock plane-jet)", r_fan=4000.0, half=25.0, n=6,
           perch=400.0, dive_deg=10.0, ground=-48.0, gate_r=1200.0,
           freeze_r=100.0, loiter=180.0, reach=260.0,
           v_cruise=70.0, v_impact_total=63.0, v_level_top=105.7)
NEW = dict(name="fast airframe (136 / 204 m/s)", r_fan=11000.0, half=25.0, n=6,
           perch=3100.0, dive_deg=20.0, ground=-48.0, gate_r=3400.0,
           freeze_r=280.0, loiter=1088.0, reach=1600.0,
           v_cruise=130.0, v_impact_total=204.0, v_level_top=136.0)

COL = ["#e6194b", "#3cb44b", "#4363d8", "#f58231", "#911eb4", "#42d4f4"]


def ring_radius(cfg):
    return (cfg["perch"] - cfg["ground"]) / math.tan(math.radians(cfg["dive_deg"]))


def slot_bearings(cfg):
    """approach_from=180 ⇒ 扇形在目標南方;回傳每個 slot 的方位角(度)。"""
    h, n = cfg["half"], cfg["n"]
    return [(180.0 - h + 2 * h * k / (n - 1)) % 360.0 for k in range(n)]


def polar(brg_deg, r):
    """方位角(北為 0、東為 90)→ (x=East, y=North)。"""
    b = math.radians(brg_deg)
    return r * math.sin(b), r * math.cos(b)


# ══════════════════════════════════════════════════════════════════════
#  1. 俯視圖
# ══════════════════════════════════════════════════════════════════════
def topview(cfg, ax, show_legend=True):
    R = ring_radius(cfg)
    brgs = slot_bearings(cfg)

    # 扇形涵蓋區
    ax.add_patch(Wedge((0, 0), cfg["r_fan"],
                       90 - (180 + cfg["half"]), 90 - (180 - cfg["half"]),
                       facecolor="#9ecae1", alpha=0.16, edgecolor="none"))
    # pushover 環
    ax.add_patch(Circle((0, 0), R, fill=False, ls="--", lw=1.6,
                        ec="#2166ac", alpha=0.9))
    # PN 交接圈 / 終端凍結圈
    ax.add_patch(Circle((0, 0), cfg["gate_r"], fill=False, ls=":", lw=1.4,
                        ec="#b2182b", alpha=0.9))
    ax.add_patch(Circle((0, 0), cfg["freeze_r"], fill=False, lw=1.2,
                        ec="#67001f", alpha=0.7))

    for k, b in enumerate(brgs):
        sx, sy = polar(b, cfg["r_fan"])
        rx, ry = polar(b, R)
        ax.plot([0, sx], [0, sy], color=COL[k], lw=0.8, alpha=0.45)
        ax.plot([rx], [ry], "o", color=COL[k], ms=7, zorder=5)
        ax.plot([sx], [sy], "s", color=COL[k], ms=6, alpha=0.85, zorder=5)
        # 每機的繞圈半徑(說明為什麼 reach 必須比它大)
        ax.add_patch(Circle((sx, sy), cfg["loiter"], fill=False, lw=0.7,
                            ec=COL[k], alpha=0.35))

    ax.plot(0, 0, "*", color="#d62728", ms=22, zorder=6)
    ax.annotate("BULLSEYE", (0, 0), textcoords="offset points",
                xytext=(10, 10), fontsize=9, color="#d62728", weight="bold")

    lim = cfg["r_fan"] * 1.12
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim * 1.05, lim * 0.35)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25, ls=":")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    sp = cfg["r_fan"] * math.radians(2 * cfg["half"]) / (cfg["n"] - 1)
    ax.set_title("%s\nfan R=%.0f km  ring R=%.1f km  slot spacing=%.0f m"
                 % (cfg["name"], cfg["r_fan"] / 1000, R / 1000, sp),
                 fontsize=10)
    if show_legend:
        ax.plot([], [], "s", color="0.4", label="fan slot (hold / align)")
        ax.plot([], [], "o", color="0.4", label="pushover ring point")
        ax.plot([], [], "--", color="#2166ac", label="pushover ring")
        ax.plot([], [], ":", color="#b2182b", label="PN handover (gate_r)")
        ax.plot([], [], "-", color="0.5", lw=0.7,
                label="turn radius (WP_LOITER_RAD)")
        ax.legend(loc="lower right", fontsize=7.5, framealpha=0.9)


# ══════════════════════════════════════════════════════════════════════
#  2. 側視剖面
# ══════════════════════════════════════════════════════════════════════
def profile(cfg, ax):
    R = ring_radius(cfg)
    g = cfg["ground"]
    ax.axhspan(g - 400, g, color="#8c6d31", alpha=0.25)
    ax.axhline(g, color="#8c6d31", lw=1.4)
    ax.annotate("bullseye ground  z=%.0f m" % g, (R * 0.62, g - 130),
                fontsize=8, color="#8c6d31")

    # 下滑線
    ax.plot([R, 0], [cfg["perch"], g], color="#2166ac", lw=2.4,
            label="%.0f° glide slope" % cfg["dive_deg"])
    # perch 平台
    ax.plot([R, R * 1.16], [cfg["perch"]] * 2, color="#2166ac", lw=2.4)
    ax.plot([R], [cfg["perch"]], "o", color="#2166ac", ms=9)
    ax.annotate("perch  %.0f m" % cfg["perch"], (R * 1.0, cfg["perch"] + 130),
                fontsize=9, color="#2166ac", weight="bold")

    for r, lab, c in ((cfg["gate_r"], "PN handover\ngate_r=%.0f m" % cfg["gate_r"], "#b2182b"),
                      (cfg["freeze_r"], "terminal freeze\n%.0f m" % cfg["freeze_r"], "#67001f")):
        h = g + r * math.tan(math.radians(cfg["dive_deg"]))
        ax.plot([r, r], [g, h], ls=":", color=c, lw=1.3)
        ax.plot([r], [h], "o", color=c, ms=6)
        ax.annotate(lab, (r, h + 90), fontsize=8, color=c, ha="center")

    ax.plot(0, g, "*", color="#d62728", ms=20, zorder=6)
    ax.set_xlim(R * 1.2, -R * 0.06)          # 由遠而近(飛行方向由左到右)
    ax.set_ylim(g - 400, cfg["perch"] * 1.22)
    ax.grid(alpha=0.25, ls=":")
    ax.set_xlabel("horizontal range to bullseye (m)   →  direction of flight")
    ax.set_ylabel("altitude above HOME (m)")
    ax.set_title("dive profile — entry %.0f m/s → impact %.0f m/s total "
                 "(%.0f m/s ground)"
                 % (cfg["v_cruise"] * 0.55, cfg["v_impact_total"],
                    cfg["v_impact_total"] * math.cos(math.radians(cfg["dive_deg"]))),
                 fontsize=10)
    ax.legend(loc="upper right", fontsize=8)


# ══════════════════════════════════════════════════════════════════════
#  3. 動畫
# ══════════════════════════════════════════════════════════════════════
def build_tracks(cfg, n_frames=170):
    """解析生成六機的 ALIGN → 環上就位 → 同步俯衝軌跡(依設計參數)。"""
    R = ring_radius(cfg)
    brgs = slot_bearings(cfg)
    f_align, f_hold = 0.42, 0.56          # 相位切點(佔全片比例)
    tracks = []
    for k, b in enumerate(brgs):
        sx, sy = polar(b, cfg["r_fan"])
        rx, ry = polar(b, R)
        xs, ys, zs, vs = [], [], [], []
        for i in range(n_frames):
            u = i / (n_frames - 1)
            if u < f_align:                       # 由扇形 slot 內切到環
                s = u / f_align
                x = sx + (rx - sx) * s
                y = sy + (ry - sy) * s
                z = 1200.0 + (cfg["perch"] - 1200.0) * s
                v = cfg["v_cruise"] * (1 - 0.45 * s)
            elif u < f_hold:                      # 環上等高度收斂
                x, y, z = rx, ry, cfg["perch"]
                v = cfg["v_cruise"] * 0.55
            else:                                 # 同步俯衝
                s = (u - f_hold) / (1 - f_hold)
                x, y = rx * (1 - s), ry * (1 - s)
                z = cfg["perch"] + (cfg["ground"] - cfg["perch"]) * s
                v0 = cfg["v_cruise"] * 0.55
                v = math.sqrt(v0 ** 2 + (cfg["v_impact_total"] ** 2 - v0 ** 2) * s)
            xs.append(x); ys.append(y); zs.append(z); vs.append(v)
        tracks.append((xs, ys, zs, vs))
    return tracks, f_align, f_hold


def make_gif(cfg, path, n_frames=170, fps=18):
    R = ring_radius(cfg)
    tracks, f_align, f_hold = build_tracks(cfg, n_frames)

    fig = plt.figure(figsize=(13, 5.6), dpi=110)
    axt = fig.add_subplot(1, 2, 1)
    axp = fig.add_subplot(1, 2, 2)
    fig.subplots_adjust(left=0.06, right=0.985, top=0.86, bottom=0.12, wspace=0.22)

    topview(cfg, axt, show_legend=False)
    dots = [axt.plot([], [], "o", color=COL[k], ms=9, zorder=8)[0]
            for k in range(cfg["n"])]
    trails = [axt.plot([], [], "-", color=COL[k], lw=1.5, alpha=0.75)[0]
              for k in range(cfg["n"])]

    g = cfg["ground"]
    axp.axhspan(g - 400, g, color="#8c6d31", alpha=0.25)
    axp.axhline(g, color="#8c6d31", lw=1.4)
    axp.plot([R, 0], [cfg["perch"], g], color="#2166ac", lw=1.6, alpha=0.5)
    axp.plot(0, g, "*", color="#d62728", ms=18, zorder=6)
    axp.set_xlim(cfg["r_fan"] * 1.05, -cfg["r_fan"] * 0.04)
    axp.set_ylim(g - 400, cfg["perch"] * 1.25)
    axp.grid(alpha=0.25, ls=":")
    axp.set_xlabel("horizontal range to bullseye (m)")
    axp.set_ylabel("altitude above HOME (m)")
    pdots = [axp.plot([], [], "o", color=COL[k], ms=8, zorder=8)[0]
             for k in range(cfg["n"])]
    ptrails = [axp.plot([], [], "-", color=COL[k], lw=1.3, alpha=0.7)[0]
               for k in range(cfg["n"])]
    banner = fig.text(0.5, 0.945, "", ha="center", fontsize=13, weight="bold")
    vtxt = axp.text(0.02, 0.06, "", transform=axp.transAxes, fontsize=9,
                    family="monospace",
                    bbox=dict(fc="white", ec="0.7", alpha=0.85))

    def frame(i):
        u = i / (n_frames - 1)
        if u < f_align:
            ph = "ALIGN  →  cut in to pushover ring"
        elif u < f_hold:
            ph = "PERCH  →  hold ring, let altitudes converge"
        else:
            ph = "DIVE   →  synchronised 20° terminal dive"
        banner.set_text("%s     |     %s" % (cfg["name"], ph))
        for k, (xs, ys, zs, vs) in enumerate(tracks):
            dots[k].set_data([xs[i]], [ys[i]])
            trails[k].set_data(xs[:i + 1], ys[:i + 1])
            d = [math.hypot(x, y) for x, y in zip(xs[:i + 1], ys[:i + 1])]
            pdots[k].set_data([d[-1]], [zs[i]])
            ptrails[k].set_data(d, zs[:i + 1])
        vtxt.set_text("speed  %5.1f m/s\nalt    %5.0f m"
                      % (tracks[0][3][i], tracks[0][2][i]))
        return dots + trails + pdots + ptrails + [banner, vtxt]

    anim = animation.FuncAnimation(fig, frame, frames=n_frames,
                                   interval=1000 / fps, blit=False)
    anim.save(path, writer=animation.PillowWriter(fps=fps))
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", default=os.path.expanduser("~/fixwing_ws/runs/figs"))
    ap.add_argument("--no-gif", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)

    # 俯視對照(同比例尺,才看得出放大了多少)
    fig, axes = plt.subplots(1, 2, figsize=(14, 7.4), dpi=130)
    for ax, cfg in zip(axes, (OLD, NEW)):
        topview(cfg, ax)
        ax.set_xlim(-12500, 12500)
        ax.set_ylim(-13200, 4400)
    fig.suptitle("STOT show geometry — same scale on both panels", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.965))
    p1 = os.path.join(a.out_dir, "geometry_topview.png")
    fig.savefig(p1); plt.close(fig)

    # 側視剖面對照
    fig, axes = plt.subplots(2, 1, figsize=(11, 9), dpi=130)
    for ax, cfg in zip(axes, (OLD, NEW)):
        profile(cfg, ax)
    fig.tight_layout()
    p2 = os.path.join(a.out_dir, "geometry_profile.png")
    fig.savefig(p2); plt.close(fig)

    outs = [p1, p2]
    if not a.no_gif:
        p3 = os.path.join(a.out_dir, "mission_new_geometry.gif")
        make_gif(NEW, p3)
        outs.append(p3)
    for p in outs:
        print(p, "%.1f KB" % (os.path.getsize(p) / 1024))


if __name__ == "__main__":
    main()
