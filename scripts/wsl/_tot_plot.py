#!/usr/bin/env python3
"""TOT 航次可視化:目標 + 扇形 slot + 六機軌跡(S 型機動段標紅)+ 抵達時間。
用法: python3 _tot_plot.py <tot.csv> <tot_geom.csv> <out.png>"""
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

CSV, GEOM, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
EARTH = 111320.0
COL = {1: "#2a7de1", 2: "#7b52d1", 3: "#1fa87a", 4: "#d17f2a", 5: "#c23b3b", 6: "#00b3b3"}

geom = {}
slots = []
for r in csv.reader(open(GEOM)):
    if r[0] == "kind":
        continue
    if r[0] == "target":
        geom["T"] = (float(r[1]), float(r[2]))
    else:
        slots.append((float(r[1]), float(r[2])))
T = geom["T"]
ref = T
def en(lat, lon):
    return ((lon - ref[1]) * EARTH * math.cos(math.radians(ref[0])),
            (lat - ref[0]) * EARTH)

tracks = collections.defaultdict(list)   # sysid -> [(t,e,n,mode,gs,vcmd)]
for r in csv.reader(open(CSV)):
    if r[0] == "t":
        continue
    sid = int(r[1]); e, n = en(float(r[2]), float(r[3]))
    tracks[sid].append((float(r[0]), e, n, r[7], float(r[5]), float(r[8])))

fig, (ax, ax2) = plt.subplots(1, 2, figsize=(15, 8),
                              gridspec_kw={"width_ratios": [1.5, 1]})

# ── 左:俯視軌跡 + 扇形 ──
te, tn = en(*T)
ax.scatter(te, tn, s=400, c="#d11", marker="*", zorder=6, label="目標 T")
# 扇形弧
sxy = [en(*s) for s in slots]
ax.plot([x for x, y in sxy], [y for x, y in sxy], color="#d11", ls="--",
        lw=1.2, alpha=0.6, zorder=3)
for k, (sx, sy) in enumerate(sxy):
    ax.scatter(sx, sy, s=150, facecolors="none", edgecolors="#d11", lw=1.8, zorder=4)
    ax.plot([sx, te], [sy, tn], color="#d11", ls=":", lw=0.7, alpha=0.4)  # 切線
    ax.annotate(f"slot{k}", (sx, sy), textcoords="offset points",
                xytext=(4, 4), fontsize=8, color="#a11")
for sid, tr in sorted(tracks.items()):
    xs = [e for _, e, n, m, g, v in tr]; ys = [n for _, e, n, m, g, v in tr]
    ax.plot(xs, ys, color=COL[sid], lw=1.3, alpha=0.85, label=f"UAV{sid}")
    # S 型機動段標紅點
    wx = [e for _, e, n, m, g, v in tr if m == "weave"]
    wy = [n for _, e, n, m, g, v in tr if m == "weave"]
    if wx:
        ax.scatter(wx, wy, s=6, c="#c23b3b", alpha=0.5, zorder=2)
    ax.scatter(xs[0], ys[0], s=40, c=COL[sid], marker="o", zorder=5)
ax.set_aspect("equal"); ax.grid(ls=":", alpha=0.4)
ax.set_xlabel("東 (m,相對 T)"); ax.set_ylabel("北 (m,相對 T)")
ax.set_title("TOT 俯視 — ★目標、○扇形 slot、虛線=切線、紅點=S 型機動段")
ax.legend(fontsize=8, loc="lower left", ncol=2)

# ── 右上:各機 slot 剩餘距離 vs t(應同時歸零)──
ax2.set_title("各機到 slot 剩餘距離(同時歸零=同時抵達)")
for sid, tr in sorted(tracks.items()):
    # d_slot 在 CSV col6 但這裡用軌跡重算太麻煩;改讀原始 d(存在 col6)
    pass
# 直接從 CSV 讀 d_slot
dd = collections.defaultdict(list)
for r in csv.reader(open(CSV)):
    if r[0] == "t":
        continue
    dd[int(r[1])].append((float(r[0]), float(r[6]), r[7]))
for sid, seq in sorted(dd.items()):
    ax2.plot([t for t, d, m in seq], [d for t, d, m in seq],
             color=COL[sid], lw=1.4, label=f"UAV{sid}")
    wt = [t for t, d, m in seq if m == "weave"]
    wd = [d for t, d, m in seq if m == "weave"]
    if wt:
        ax2.scatter(wt, wd, s=5, c="#c23b3b", alpha=0.5)
ax2.axhline(120, color="#888", ls="--", lw=1)
ax2.text(0, 120, " reach 120m", fontsize=8, va="bottom")
ax2.set_xlabel("t (s)"); ax2.set_ylabel("到 slot 距離 (m)")
ax2.grid(ls=":", alpha=0.4); ax2.legend(fontsize=8)

fig.suptitle("蜂群同時抵達(TOT)— 扇形閘門 + S型機動/速度時間對齊", fontsize=13)
fig.tight_layout()
fig.savefig(OUT, dpi=115, bbox_inches="tight")
print("saved", OUT)
