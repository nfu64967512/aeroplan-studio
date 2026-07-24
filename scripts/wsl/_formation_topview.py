#!/usr/bin/env python3
"""編隊耦合俯視圖:把整段錄製的僚機位置全部轉進「長機座標框」(機頭朝上、
長機=原點)。每架僚機成為一團點雲 —— 雲團貼著自己 slot 越緊 = 耦合越緊。
用法: python3 _formation_topview.py <report.csv> <out.png> [spacing angle tail]
"""
import bisect, csv, math, sys
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

EARTH = 111320.0
CSVP, OUT = sys.argv[1], sys.argv[2]
S = float(sys.argv[3]) if len(sys.argv) > 3 else 60.0
TH = float(sys.argv[4]) if len(sys.argv) > 4 else 25.0
TAIL = float(sys.argv[5]) if len(sys.argv) > 5 else 120.0

# slot(長機框,機頭朝上):x=右、y=前;off=相對機鼻方位
OFFS = {2: (1 * S, 180 + TH), 3: (2 * S, 180 + TH),
        4: (1 * S, 180 - TH), 5: (2 * S, 180 - TH), 6: (TAIL, 180.0)}
SLOT = {i: (d * math.sin(math.radians(o)), d * math.cos(math.radians(o)))
        for i, (d, o) in OFFS.items()}
COL = {2: "#2a7de1", 3: "#7b52d1", 4: "#1fa87a", 5: "#d17f2a", 6: "#c23b3b"}

leader, pos = [], {}
for row in csv.reader(open(CSVP)):
    if row[0] == "leader":
        leader.append((float(row[1]), int(row[2]), float(row[3]),
                       float(row[4]), float(row[6])))          # t,id,lat,lon,hdg
    elif row[0] == "pos":
        pos.setdefault(int(row[2]), []).append(
            (float(row[1]), float(row[3]), float(row[4])))
ltimes = [l[0] for l in leader]
lead_id = leader[-1][1]


def to_frame(t, lat, lon):
    """該時刻長機框座標 (x=右, y=前);找不到 ±0.5 s 內的長機樣本回 None。"""
    k = bisect.bisect_left(ltimes, t)
    for kk in (k - 1, k):
        if 0 <= kk < len(leader) and abs(leader[kk][0] - t) <= 0.5:
            _, _, la, lo, hdg = leader[kk]
            e = (lon - lo) * EARTH * math.cos(math.radians(la))
            n = (lat - la) * EARTH
            r = math.radians(-hdg)
            return (e * math.cos(r) - n * math.sin(r),
                    e * math.sin(r) + n * math.cos(r))
    return None


clouds = {}          # {uav: [(t,x,y)]}
for i, tr in pos.items():
    if i == lead_id:
        continue
    for t, la, lo in tr:
        p = to_frame(t, la, lo)
        if p:
            clouds.setdefault(i, []).append((t, p[0], p[1]))

fig = plt.figure(figsize=(15, 9.5))
gs = fig.add_gridspec(2, 3, width_ratios=[2.6, 1, 1], hspace=0.28, wspace=0.3)

# ── 主圖:全程點雲(耦合的直接證據)────────────────────────────────
ax = fig.add_subplot(gs[:, 0])
stats = []
t_end = max(t for c in clouds.values() for t, _, _ in c)
for i in sorted(clouds):
    pts = clouds[i]
    xs = [x for _, x, y in pts]; ys = [y for _, x, y in pts]
    al = [0.12 + 0.5 * (t / t_end) for t, _, _ in pts]     # 越晚越不透明
    ax.scatter(xs, ys, s=7, c=COL[i], alpha=al, lw=0, zorder=2)
    sx, sy = SLOT[i]
    late = [(x, y) for t, x, y in pts if t >= t_end * 0.5]  # 後半段=已收斂態
    rms = math.sqrt(sum((x - sx) ** 2 + (y - sy) ** 2
                        for x, y in late) / max(len(late), 1))
    stats.append((i, rms))
    ax.add_patch(plt.Circle((sx, sy), rms, fill=False, color=COL[i],
                            ls="--", lw=1.2, zorder=3))
    ax.scatter(sx, sy, s=210, facecolors="none", edgecolors=COL[i],
               lw=2, zorder=4)
    ax.annotate(f"slot-{i}", (sx, sy), textcoords="offset points",
                xytext=(8, 8), fontsize=9, color=COL[i], fontweight="bold")
ax.scatter(0, 0, s=300, c="#c23b3b", marker="^", edgecolors="black", zorder=5)
ax.annotate(f"UAV{lead_id}(長機)", (0, 0), textcoords="offset points",
            xytext=(10, 10), fontsize=10, fontweight="bold")
ax.arrow(0, 20, 0, 45, head_width=12, head_length=14, fc="#c23b3b",
         ec="#c23b3b", alpha=0.6, zorder=5)
lines = ["RMS 耦合半徑(後半段=收斂態):"] + \
        [f"  UAV{i}: {r:5.0f} m ({r/math.hypot(*SLOT[i]):.1f}×slot距)"
         for i, r in stats] + \
        [f"  平均: {sum(r for _, r in stats)/len(stats):5.0f} m"]
ax.text(0.02, 0.02, "\n".join(lines), transform=ax.transAxes, va="bottom",
        fontsize=10, bbox=dict(fc="#fffbe6", ec="#999", alpha=0.9))
ax.set_aspect("equal"); ax.grid(ls=":", alpha=0.4)
ax.set_xlabel("右 ← 0 → 左  (m,長機框)"); ax.set_ylabel("後 ← 0 → 前  (m)")
ax.set_title(f"全程 {t_end:.0f} s 點雲(顏色越實=越晚;虛線圓=RMS 耦合半徑)",
             fontsize=11)

# ── 右側:四個時刻的瞬時 V 快照 ──────────────────────────────────
snaps = [t_end * f for f in (0.25, 0.5, 0.75, 0.98)]
for k, ts in enumerate(snaps):
    ax = fig.add_subplot(gs[k % 2, 1 + k // 2])
    for i in sorted(clouds):
        best = min(clouds[i], key=lambda p: abs(p[0] - ts))
        if abs(best[0] - ts) > 2.0:
            continue
        sx, sy = SLOT[i]
        ax.plot([sx, best[1]], [sy, best[2]], color=COL[i], ls="--", lw=1)
        ax.scatter(sx, sy, s=90, facecolors="none", edgecolors=COL[i], lw=1.5)
        ax.scatter(best[1], best[2], s=110, c=COL[i], marker="^",
                   edgecolors="black", zorder=4)
    ax.scatter(0, 0, s=140, c="#c23b3b", marker="^", edgecolors="black", zorder=5)
    ax.set_aspect("equal"); ax.grid(ls=":", alpha=0.4)
    ax.set_xlim(-2.4 * S, 2.4 * S)                 # 隨幾何縮放(修:原 S=60 硬編碼)
    ax.set_ylim(-(TAIL + 1.6 * S), 1.6 * S)
    ax.set_title(f"t = {ts:.0f} s", fontsize=10)

fig.suptitle("編隊耦合俯視 — 長機座標框(機頭朝上)· 修後 240 s 真飛"
             f"(S={S:.0f} m、θ={TH:.0f}°、tail={TAIL:.0f} m)", fontsize=13)
fig.savefig(OUT, dpi=115, bbox_inches="tight")
print("saved", OUT, "| RMS:", {i: round(r) for i, r in stats})
