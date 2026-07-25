#!/usr/bin/env python3
"""_show_anim.py — 用「真實錄製軌跡」做整場表演動畫。

素材(都是實飛錄下來的,不是模擬畫的):
  /tmp/show_track.csv  GATHER / CRUISE / SCATTER(jet_show_cruise.py 錄)
  /tmp/tot_node.csv    ALIGN / PERCH / DIVE   (tot_node 錄)
兩者時間軸各自從 0 起算,依先後串接。

輸出:上為俯視(含紅心靶 + 扇形),下為高度剖面。
用法: python3 _show_anim.py <out.gif|out.mp4>
"""
import csv, math, sys, collections
import matplotlib
matplotlib.use("Agg")
from matplotlib import font_manager
for f in ("/mnt/c/Windows/Fonts/msjh.ttc", "/mnt/c/Windows/Fonts/msyh.ttc"):
    try:
        font_manager.fontManager.addfont(f)
        matplotlib.rcParams["font.family"] = [
            font_manager.FontProperties(fname=f).get_name(), "DejaVu Sans"]
        break
    except Exception:
        pass
matplotlib.rcParams["axes.unicode_minus"] = False
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/show.gif"
EARTH = 111320.0
T = (25.1406, 121.2325)
GROUND = -48.0          # 紅心地表(相對 HOME)
R_FAN, HALF, APPROACH, N = 4000.0, 25.0, 180.0, 6
COL = {1: "#2a7de1", 2: "#7b52d1", 3: "#1fa87a",
       4: "#d17f2a", 5: "#c23b3b", 6: "#00b3b3"}
PHASE_LABEL = {
    "GATHER": "① 彈射起飛後靠攏", "CRUISE": "② 雁型編隊長程巡航",
    "SCATTER": "③ 目標前四散進扇形", "ALIGN": "④ STOT 時間對齊",
    "PERCH": "⑤ 對正紅心徑向 + 佔位", "DIVE": "⑥ 終端俯衝命中紅心",
}


def mlon(lat):
    return EARTH * math.cos(math.radians(lat))


def en(lat, lon):
    return ((lon - T[1]) * mlon(T[0]), (lat - T[0]) * EARTH)


def load(path, tshift):
    """回傳 [(t, sysid, e, n, alt, phase)]"""
    out = []
    try:
        for r in csv.DictReader(open(path)):
            try:
                e, n = en(float(r["lat"]), float(r["lon"]))
                out.append((float(r["t"]) + tshift, int(r["sysid"]), e, n,
                            float(r["alt"]), r["phase"]))
            except (ValueError, KeyError):
                continue
    except FileNotFoundError:
        pass
    return out


rows = load("/tmp/show_track.csv", 0.0)
t_end = max((r[0] for r in rows), default=0.0)
rows += load("/tmp/tot_node.csv", t_end + 5.0)
rows.sort(key=lambda r: r[0])
if not rows:
    print("no data"); sys.exit(1)

# 依時間分格(每 2 秒一格,避免動畫過長)
STEP = 2.0
frames = collections.OrderedDict()
for t, sid, e, n, a, ph in rows:
    k = int(t // STEP)
    frames.setdefault(k, {})[sid] = (e, n, a, ph)
keys = sorted(frames)
# 累積軌跡
trail = {i: ([], []) for i in range(1, N + 1)}
alt_hist = {i: ([], []) for i in range(1, N + 1)}

fig, (ax, ax2) = plt.subplots(2, 1, figsize=(11, 12),
                              gridspec_kw={"height_ratios": [2.1, 1]})
fig.suptitle("蜂群長程表演:彈射 → 雁型巡航 → STOT → 命中紅心", fontsize=15)

# 靜態:紅心靶 + 扇形
for r_, c_ in ((400, "#ffffff"), (300, "#ff2b2b"), (200, "#ffffff"), (100, "#ff0000")):
    ax.add_patch(plt.Circle((0, 0), r_, color=c_, zorder=2, alpha=0.9))
ax.plot(0, 0, "*", ms=16, color="#8b0000", zorder=6)
ax.annotate("❤ 紅心", (0, 0), textcoords="offset points", xytext=(10, 10),
            fontsize=12, color="#b00000", zorder=7)
arc_e, arc_n = [], []
for k in range(41):
    b = math.radians(APPROACH - HALF + 2 * HALF * k / 40)
    arc_e.append(R_FAN * math.sin(b)); arc_n.append(R_FAN * math.cos(b))
ax.plot(arc_e, arc_n, ls="--", color="#d11", lw=1.0, alpha=0.5, label="扇形閘門 4km")
for k in range(N):
    b = math.radians(APPROACH - HALF + 2 * HALF * k / (N - 1))
    ax.plot([R_FAN * math.sin(b), 0], [R_FAN * math.cos(b), 0],
            ls=":", color="#d11", lw=0.6, alpha=0.35)

lines = {i: ax.plot([], [], "-", color=COL[i], lw=1.4, alpha=0.85,
                    label=f"UAV{i}")[0] for i in range(1, N + 1)}
dots = {i: ax.plot([], [], "o", color=COL[i], ms=7, zorder=8)[0]
        for i in range(1, N + 1)}
alines = {i: ax2.plot([], [], "-", color=COL[i], lw=1.3)[0]
          for i in range(1, N + 1)}
ax2.axhline(GROUND, color="#8b0000", ls="--", lw=1.2)
ax2.annotate("紅心地表", (0, GROUND), textcoords="offset points",
             xytext=(6, 6), color="#8b0000", fontsize=10)

ax.set_xlabel("東 (m)"); ax.set_ylabel("北 (m)")
ax.axis("equal"); ax.grid(alpha=0.25); ax.legend(fontsize=8, loc="upper left", ncol=2)
ax2.set_xlabel("時間 (s)"); ax2.set_ylabel("高度 (m)"); ax2.grid(alpha=0.25)
title = ax.set_title("")


def upd(idx):
    k = keys[idx]
    d = frames[k]
    ph = ""
    for i, (e, n, a, p) in d.items():
        trail[i][0].append(e); trail[i][1].append(n)
        alt_hist[i][0].append(k * STEP); alt_hist[i][1].append(a)
        lines[i].set_data(trail[i][0], trail[i][1])
        dots[i].set_data([e], [n])
        alines[i].set_data(alt_hist[i][0], alt_hist[i][1])
        ph = p
    title.set_text(f"t = {k*STEP:5.0f}s    {PHASE_LABEL.get(ph, ph)}")
    ax.relim(); ax.autoscale_view()
    ax2.relim(); ax2.autoscale_view()
    return list(lines.values()) + list(dots.values()) + list(alines.values())


ani = FuncAnimation(fig, upd, frames=len(keys), interval=90, blit=False)
fig.tight_layout(rect=[0, 0, 1, 0.97])
if OUT.endswith(".gif"):
    ani.save(OUT, writer=PillowWriter(fps=12))
else:
    ani.save(OUT, fps=12, dpi=110)
print("saved", OUT, "frames", len(keys))
