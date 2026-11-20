#!/usr/bin/env python3
"""A/B 對照圖:修前(report_coldstart.csv) vs 修後(report_fixed2.csv)。"""
import csv, math, sys
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


def load(path):
    leader, errs, pos = [], {}, {}
    for row in csv.reader(open(path)):
        if row[0] == "leader":
            leader.append(tuple(float(x) for x in row[1:7]))
        elif row[0] == "err":
            errs.setdefault(int(row[2]), []).append((float(row[1]), float(row[3])))
        elif row[0] == "pos":
            pos.setdefault(int(row[2]), []).append(
                (float(row[1]), float(row[3]), float(row[4])))
    return leader, errs, pos


A = load(sys.argv[1])   # before
B = load(sys.argv[2])   # after
colors = {2: "#2a7de1", 3: "#7b52d1", 4: "#1fa87a", 5: "#d17f2a", 6: "#c23b3b"}

fig, axes = plt.subplots(2, 2, figsize=(14.5, 10))
for col, (name, (leader, errs, pos)) in enumerate(
        (("修前:腦裂 + 無手風琴(UAV6 代理,UAV1 出走)", A),
         ("修後:資格閘 + 讓權回收 + 手風琴 + QoS(UAV1 全程領導)", B))):
    # 上排:誤差時間線
    ax = axes[0][col]
    for i in sorted(errs):
        ax.plot([t for t, _ in errs[i]], [e for _, e in errs[i]],
                color=colors.get(i, "#888"), lw=1.2, label=f"UAV{i}")
    ax.axhline(95, color="#333", ls="--", lw=1)
    ax.axhline(171, color="#c23b3b", ls=":", lw=1)
    ax.set_ylim(0, 260)
    ax.set_title(name, fontsize=11)
    ax.set_xlabel("t (s)"); ax.set_ylabel("slot err (m)")
    ax.grid(ls=":", alpha=0.5); ax.legend(fontsize=7, ncol=3, loc="upper right")
    allv = [e for i in errs for _, e in errs[i]]
    worst_ts = {}
    for i in errs:
        for t, e in errs[i]:
            k = round(t)
            worst_ts[k] = max(worst_ts.get(k, 0), e)
    formed = 100.0 * sum(1 for v in worst_ts.values() if v <= 95) / max(len(worst_ts), 1)
    ax.text(0.02, 0.02, f"mean {sum(allv)/len(allv):.0f} m   "
                        f"max {max(allv):.0f} m   成形佔比 {formed:.0f}%",
            transform=ax.transAxes, fontsize=10, va="bottom",
            bbox=dict(fc="#fffbe6", ec="#999"))

    # 下排:俯視(以長機末位置為原點、航向朝上)
    ax = axes[1][col]
    tL, lid, la, lo, _, hdg = leader[-1]
    ch, sh = math.cos(math.radians(-hdg)), math.sin(math.radians(-hdg))

    def frame(lat, lon):
        e = (lon - lo) * EARTH * math.cos(math.radians(la))
        n = (lat - la) * EARTH
        return e * ch - n * sh, e * sh + n * ch

    for i, tr in sorted(pos.items()):
        tr2 = [p for p in tr if tL - p[0] <= 60.0]
        if not tr2:
            continue
        xs, ys = zip(*[frame(p[1], p[2]) for p in tr2])
        c = "#c23b3b" if i == int(lid) else colors.get(i, "#666")
        ax.plot(xs, ys, color=c, lw=0.7, alpha=0.4)
        ax.scatter(xs[-1], ys[-1], s=150, c=c, marker="^",
                   edgecolors="black", zorder=4)
        ax.annotate(f"UAV{i}", (xs[-1], ys[-1]), textcoords="offset points",
                    xytext=(6, 6), fontsize=8, fontweight="bold")
    ax.set_aspect("equal"); ax.grid(ls=":", alpha=0.5)
    ax.set_xlabel("cross-track (m)"); ax.set_ylabel("along-track (m)")
    ax.set_title("俯視(60 s 航跡;紅=當時長機)", fontsize=10)

fig.suptitle("六機 V 編隊 A/B 對照 — 同條件冷啟動、各 240 s 真飛錄製"
             "(WSL SITL+ROS2,PRESET=max)", fontsize=13)
fig.tight_layout()
fig.savefig(sys.argv[3], dpi=115, bbox_inches="tight")
print("saved", sys.argv[3])
