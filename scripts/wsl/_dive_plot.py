#!/usr/bin/env python3
"""終端俯衝齊射可視化(4 面板):
  ① 俯視:六機軌跡收斂目標 T + 扇形 slot + pushover 環 + 穿越切線。
  ② 側視俯衝剖面:各機「距T vs 高度」—— 直觀看爬到高點→俯衝砸向 T。
  ③ 同時性:DIVE 段各機「距T vs 時間」同時歸零 = 齊射。
  ④ 終端加速:各機地速 vs 時間(俯衝段拉高)。
用法: python3 _dive_plot.py <tot.csv> <tot_geom.csv> <out.png>"""
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

geom = {"slot": [], "pp": []}
for r in csv.reader(open(GEOM)):
    if r[0] == "kind":
        continue
    if r[0] == "target":
        geom["T"] = (float(r[1]), float(r[2]))
    elif r[0].startswith("slot"):
        geom["slot"].append((float(r[1]), float(r[2])))
    elif r[0].startswith("pp"):
        geom["pp"].append((float(r[1]), float(r[2])))
T = geom["T"]; ref = T


def en(lat, lon):
    return ((lon - ref[1]) * EARTH * math.cos(math.radians(ref[0])),
            (lat - ref[0]) * EARTH)


def dT(lat, lon):
    e, n = en(lat, lon)
    return math.hypot(e, n)


# sysid -> [(t,lat,lon,alt,gs,mode,phase)]
tr = collections.defaultdict(list)
for r in csv.reader(open(CSV)):
    if r[0] == "t":
        continue
    tr[int(r[1])].append((float(r[0]), float(r[2]), float(r[3]),
                          float(r[4]), float(r[5]), r[7], r[11]))

fig, axs = plt.subplots(2, 2, figsize=(17, 12))
ax1, ax2, ax3, ax4 = axs[0, 0], axs[0, 1], axs[1, 0], axs[1, 1]

# ── ① 俯視 ──
te, tn = en(*T)
ax1.scatter(te, tn, s=520, c="#d11", marker="*", zorder=8, label="目標 T")
sxy = [en(*s) for s in geom["slot"]]
if sxy:
    ax1.plot([x for x, y in sxy], [y for x, y in sxy], color="#d11", ls="--",
             lw=1.1, alpha=0.5, zorder=3, label="扇形弧 (2km)")
    for k, (sx, sy) in enumerate(sxy):
        ax1.scatter(sx, sy, s=90, facecolors="none", edgecolors="#d11", lw=1.4, zorder=4)
        ax1.plot([sx, te], [sy, tn], color="#d11", ls=":", lw=0.6, alpha=0.35)
pxy = [en(*s) for s in geom["pp"]]
if pxy:
    ax1.scatter([x for x, y in pxy], [y for x, y in pxy], s=60, marker="s",
                facecolors="none", edgecolors="#888", lw=1.2, zorder=4,
                label="pushover 環")
for sid, rows in sorted(tr.items()):
    xy = [en(la, lo) for _, la, lo, _, _, _, _ in rows]
    ax1.plot([x for x, y in xy], [y for x, y in xy], color=COL[sid], lw=1.3,
             alpha=0.85, label=f"UAV{sid}")
    ex, ey = xy[-1]
    ax1.scatter(ex, ey, s=40, color=COL[sid], zorder=6)
ax1.set_title("① 俯視:六機收斂目標 T(扇形 slot → pushover 環 → 穿越)", fontsize=12)
ax1.set_xlabel("東 (m)"); ax1.set_ylabel("北 (m)")
ax1.legend(fontsize=8, loc="upper right", ncol=2); ax1.axis("equal"); ax1.grid(alpha=0.25)

# ── ② 側視俯衝剖面(距T vs 高度),只畫 DIVE/PERCH 段 ──
for sid, rows in sorted(tr.items()):
    seg = [(dT(la, lo), al) for _, la, lo, al, _, _, ph in rows if ph in ("PERCH", "DIVE")]
    if not seg:
        continue
    ax2.plot([d for d, a in seg], [a for d, a in seg], color=COL[sid], lw=1.6,
             alpha=0.9, label=f"UAV{sid}")
ax2.axvline(0, color="#d11", ls="--", lw=1.2)
ax2.annotate("目標 T", (0, 5), color="#d11", fontsize=10, ha="center")
ax2.set_title("② 側視俯衝剖面:爬到高點 → 俯衝砸向 T(距T→0)", fontsize=12)
ax2.set_xlabel("距目標 T 水平距離 (m)"); ax2.set_ylabel("相對高度 (m)")
ax2.invert_xaxis()   # 由遠(左)到 T(右=0)
ax2.legend(fontsize=8); ax2.grid(alpha=0.25)

# ── ③ 同時性:DIVE 段 距T vs 時間 ──
dive_t0 = min((rows[0][0] for rows in tr.values()), default=0)
for sid, rows in sorted(tr.items()):
    seg = [(t, dT(la, lo)) for t, la, lo, _, _, _, ph in rows if ph == "DIVE"]
    if not seg:
        continue
    t0 = seg[0][0]
    ax3.plot([t - t0 for t, d in seg], [d for t, d in seg], color=COL[sid], lw=1.6,
             label=f"UAV{sid}")
ax3.axhline(0, color="#d11", ls="--", lw=1.0)
ax3.set_title("③ 同時性:俯衝段 距T vs 時間 → 同時歸零(齊射)", fontsize=12)
ax3.set_xlabel("俯衝起算時間 (s)"); ax3.set_ylabel("距目標 T (m)")
ax3.legend(fontsize=8); ax3.grid(alpha=0.25)

# ── ④ 終端加速:地速 vs 時間(PERCH+DIVE)──
for sid, rows in sorted(tr.items()):
    seg = [(t, gs) for t, _, _, _, gs, _, ph in rows if ph in ("PERCH", "DIVE")]
    if not seg:
        continue
    t0 = seg[0][0]
    ax4.plot([t - t0 for t, gs in seg], [gs for t, gs in seg], color=COL[sid], lw=1.3,
             alpha=0.9, label=f"UAV{sid}")
ax4.set_title("④ 終端加速:各機地速 vs 時間(俯衝段拉高)", fontsize=12)
ax4.set_xlabel("PERCH 起算時間 (s)"); ax4.set_ylabel("地速 (m/s)")
ax4.legend(fontsize=8); ax4.grid(alpha=0.25)

fig.suptitle("蜂群終端俯衝齊射(Time-on-Target Terminal Dive)", fontsize=15, y=0.995)
fig.tight_layout(rect=[0, 0, 1, 0.98])
fig.savefig(OUT, dpi=115)
print("saved", OUT)
