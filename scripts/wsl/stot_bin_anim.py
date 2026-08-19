#!/usr/bin/env python3
"""stot_bin_anim.py — 直接讀飛控 dataflash (.BIN) 產生任務 GIF。

⚠️⚠️ 為什麼要有這支(而不是沿用 stot_mission_anim.py):
   stot_mission_anim.py 讀的是 `runs/tot_<TRIAL>.csv` —— 那是 **tot_node 以為**
   飛機在哪裡(MAVROS 遙測、10 Hz、可能凍結、可能落後)。本專案已經吃過
   「遙測說 X、飛機做 Y」至少三次(長機位姿凍結、AGL 基準不同步、
   二次穿越污染)。.BIN 是**飛控自己寫的**,是獨立來源。
   兩者不一致的時候,以 .BIN 為準。

⚠️ 跨機時間對齊一律用 GPS 時間,不能用 TimeUS ——
   TimeUS 是各自的開機微秒數,兩個 SITL 實例的開機時刻差幾十秒到幾分鐘。
   DFReader 會在有 GPS 訊息時把 `msg._timestamp` 換算成 UTC epoch,用那個。

⚠️ 圖上一律英文標籤:SITL 環境沒有 CJK 字型,中文會變方框。

用法:
  python3 stot_bin_anim.py --uav 1 2 --target 25.1261649,120.615648 \
      --out ~/fixwing_ws/runs/figs/RPY3.gif
  # 不給 --logs 就自動取每架**最新**的 .BIN
"""
import argparse
import bisect
import glob
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation

EARTH = 111320.0
COL = ["#e6194b", "#3cb44b", "#4363d8", "#f58231", "#911eb4", "#00b8d4"]
SITL = os.path.expanduser("~/fixwing_ws/sitl_run")


def latest_bin(uav):
    pats = sorted(glob.glob(SITL + "/uav%d/logs/*.BIN" % uav),
                  key=os.path.getmtime)
    return pats[-1] if pats else None


def read_bin(path):
    """回傳 [[t_epoch, lat, lon, alt_amsl, gs, climb], ...],時間單調遞增。

    ⚠️ 位置只取 POS(EKF 融合後的位置)。GPS 訊息的 Lat/Lng 帶量測噪聲,
       畫俯衝末段會抖成鋸齒;POS 才是飛控實際拿來導引的量。
    """
    from pymavlink import mavutil
    mlog = mavutil.mavlink_connection(path, dialect="ardupilotmega")
    rows, spd = [], []
    while True:
        m = mlog.recv_match(type=["POS", "GPS"])
        if m is None:
            break
        ts = getattr(m, "_timestamp", None)
        if not ts or ts < 1e9:        # GPS 未鎖定 ⇒ 時戳不可信,整筆丟掉
            continue
        if m.get_type() == "GPS":
            spd.append((ts, float(getattr(m, "Spd", 0.0))))
        else:
            rows.append([ts, float(m.Lat), float(m.Lng), float(m.Alt), 0.0, 0.0])
    if not rows:
        return []
    rows.sort(key=lambda r: r[0])
    spd.sort()
    st = [s[0] for s in spd]
    for r in rows:                    # 最近鄰配地速
        if st:
            k = min(bisect.bisect_left(st, r[0]), len(st) - 1)
            r[4] = spd[k][1]
    for a, b in zip(rows, rows[1:]):  # 爬升率
        dt = b[0] - a[0]
        if dt > 1e-3:
            b[5] = (b[3] - a[3]) / dt
    return rows


def sample(rows, t):
    """線性內插到時刻 t。超出記錄範圍回 None —— **不外推**,外推會憑空生航跡。"""
    if not rows or t < rows[0][0] or t > rows[-1][0]:
        return None
    ts = [r[0] for r in rows]
    k = bisect.bisect_left(ts, t)
    if k == 0:
        return rows[0]
    a, b = rows[k - 1], rows[min(k, len(rows) - 1)]
    span = b[0] - a[0]
    f = 0.0 if span <= 0 else (t - a[0]) / span
    return [a[i] + (b[i] - a[i]) * f for i in range(6)]


def dive_start(rows, alt_floor=200.0):
    """俯衝起點 = 落地前的**最後一個**高度極大值。

    ⚠️ 刻意不用「首次降率超過門檻」:盤旋段高度來回擺,first-crossing 會抓到
       ALIGN 的某次擺盪,害俯衝段的影格預算花在錯的地方。
    """
    if not rows:
        return None
    below = [i for i, r in enumerate(rows) if r[3] < alt_floor]
    end = below[-1] if below else len(rows) - 1
    top = max(range(0, end + 1), key=lambda i: rows[i][3])
    return rows[top][0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--uav", nargs="+", type=int, default=[1, 2])
    ap.add_argument("--logs", nargs="*", default=None,
                    help=".BIN 路徑,順序對應 --uav;不給就自動取最新")
    ap.add_argument("--target", required=True, help="lat,lon")
    ap.add_argument("--out", required=True)
    ap.add_argument("--frames", type=int, default=240)
    ap.add_argument("--dive-share", type=float, default=0.55,
                    help="分給俯衝段的影格比例(俯衝只佔全程約 7%,等比例會一閃而過)")
    ap.add_argument("--fps", type=int, default=12)
    a = ap.parse_args()

    tlat, tlon = (float(x) for x in a.target.split(","))
    coslat = math.cos(math.radians(tlat))

    tracks = {}
    for n, u in enumerate(a.uav):
        p = (a.logs[n] if a.logs and n < len(a.logs) else latest_bin(u))
        if not p or not os.path.exists(p):
            print("[warn] UAV%d：找不到 .BIN" % u, file=sys.stderr)
            continue
        r = read_bin(p)
        print("[bin] UAV%d %s  %d POS  %.0fs"
              % (u, os.path.basename(p), len(r),
                 (r[-1][0] - r[0][0]) if r else 0), file=sys.stderr)
        if r:
            tracks[u] = r
    if not tracks:
        sys.exit("沒有可用的 .BIN")

    # ⚠️⚠️ 用**聯集**不是交集。交集窗會在最早結束的那架落地時就截斷,
    #    而散布正好長在「最早落地」到「最晚落地」之間 —— 交集等於把要看的
    #    東西剪掉(RPY3 第一版就切掉了 UAV1 的最後 84 s,剛好是 78 s 散布)。
    #    落在自己記錄範圍外的機體,sample() 回 None、標 no data,不會捏造航跡。
    t0 = min(r[0][0] for r in tracks.values())
    t1 = max(r[-1][0] for r in tracks.values())
    tdv = max((dive_start(r) or t0) for r in tracks.values())
    tdv = min(max(tdv, t0), t1)
    print("[win] 聯集 %.0fs；俯衝起點 T+%.0fs" % (t1 - t0, tdv - t0),
          file=sys.stderr)

    # 非線性時間軸:前段/俯衝段各自固定影格數。畫面上顯示的是**真實**經過時間。
    nd = max(int(a.frames * a.dive_share), 2)
    nc = max(a.frames - nd, 2)
    times = ([t0 + (tdv - t0) * i / nc for i in range(nc)]
             + [tdv + (t1 - tdv) * i / nd for i in range(nd + 1)])

    fig = plt.figure(figsize=(13, 6.5), dpi=110)
    gs = fig.add_gridspec(2, 2, width_ratios=[1.35, 1], hspace=0.32, wspace=0.22)
    axm = fig.add_subplot(gs[:, 0])
    axa = fig.add_subplot(gs[0, 1])
    axv = fig.add_subplot(gs[1, 1])

    xy = dict((u, [((r[2] - tlon) * EARTH * coslat, (r[1] - tlat) * EARTH)
                   for r in rows]) for u, rows in tracks.items())
    allx = [p[0] for v in xy.values() for p in v]
    ally = [p[1] for v in xy.values() for p in v]
    pad = 0.06 * max(max(allx) - min(allx), max(ally) - min(ally), 1000)
    axm.set_xlim(min(allx) - pad, max(allx) + pad)
    axm.set_ylim(min(ally) - pad, max(ally) + pad)
    axm.set_aspect("equal")
    axm.set_xlabel("East of target (m)")
    axm.set_ylabel("North of target (m)")
    axm.grid(alpha=0.25)
    axm.plot(0, 0, marker="*", ms=22, color="#d40000", zorder=6)
    axm.annotate("TARGET", (0, 0), textcoords="offset points", xytext=(12, 8),
                 color="#d40000", fontsize=9, weight="bold")

    for ax, lab in ((axa, "Altitude AMSL (m)"), (axv, "Ground speed (m/s)")):
        ax.set_xlim(0, t1 - t0)
        ax.set_xlabel("Mission time (s)")
        ax.set_ylabel(lab)
        ax.grid(alpha=0.25)
    axa.set_ylim(0, max(r[3] for rows in tracks.values() for r in rows) * 1.08)
    axv.set_ylim(0, max(r[4] for rows in tracks.values() for r in rows) * 1.1)
    axa.axvline(tdv - t0, color="k", ls="--", lw=0.9, alpha=0.5)
    axa.annotate("dive", (tdv - t0, axa.get_ylim()[1] * 0.94), fontsize=8,
                 ha="right", alpha=0.7)

    art = {}
    for n, u in enumerate(sorted(tracks)):
        c = COL[n % len(COL)]
        art[u] = {
            "trail": axm.plot([], [], "-", color=c, lw=1.5, alpha=0.85,
                              label="UAV%d" % u)[0],
            "head": axm.plot([], [], "o", color=c, ms=8, zorder=5)[0],
            "alt": axa.plot([], [], "-", color=c, lw=1.4)[0],
            "vel": axv.plot([], [], "-", color=c, lw=1.4)[0],
        }
    axm.legend(loc="upper left", fontsize=9)
    ttl = axm.set_title("", fontsize=10, loc="left", family="monospace")
    fig.suptitle("STOT mission — source: flight-controller dataflash (.BIN)",
                 fontsize=11, weight="bold", x=0.02, ha="left")

    def upd(k):
        t = times[k]
        lines = ["T+%6.1fs   (frame %d/%d)" % (t - t0, k + 1, len(times))]
        for u in sorted(tracks):
            rows = tracks[u]
            cut = bisect.bisect_left([r[0] for r in rows], t)
            seg = rows[:cut]
            art[u]["trail"].set_data([p[0] for p in xy[u][:cut]],
                                     [p[1] for p in xy[u][:cut]])
            art[u]["alt"].set_data([r[0] - t0 for r in seg], [r[3] for r in seg])
            art[u]["vel"].set_data([r[0] - t0 for r in seg], [r[4] for r in seg])
            s = sample(rows, t)
            if s:
                x = (s[2] - tlon) * EARTH * coslat
                y = (s[1] - tlat) * EARTH
                art[u]["head"].set_data([x], [y])
                lines.append("UAV%d  alt %6.0f m   gs %5.1f m/s   dist %8.0f m"
                             % (u, s[3], s[4], math.hypot(x, y)))
            else:
                art[u]["head"].set_data([], [])
                lines.append("UAV%d  (no data at this time)" % u)
        ttl.set_text("\n".join(lines))
        return []

    an = animation.FuncAnimation(fig, upd, frames=len(times),
                                 interval=1000 // a.fps)
    out_dir = os.path.dirname(os.path.abspath(a.out))
    if out_dir:
        try:
            os.makedirs(out_dir)
        except OSError:
            pass
    an.save(a.out, writer=animation.PillowWriter(fps=a.fps))
    print("[ok] %s  (%d frames)" % (a.out, len(times)), file=sys.stderr)


if __name__ == "__main__":
    main()
