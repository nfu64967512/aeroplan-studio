#!/usr/bin/env python3
"""stot_mission_anim.py — 全流程動畫:彈射起飛 → 巡航四散 → STOT 對齊 → 終端俯衝。

資料來源(混合,畫面上會標明):
  ALIGN / PERCH / DIVE  ── **真實飛行資料**,讀 runs/tot_<TRIAL>.csv
  彈射 + 四散            ── 重建。tot_node 是四散完才啟動的,那兩段不在 CSV 裡;
                            但它們是確定性的:HOME 縱列(間距 30 m、航向 52°)、
                            彈射爬到 TKOFF_ALT、再飛到 jet_rescatter.py 寫死的
                            OFF 表(目標南方 3.6–4.4 km、高度 130 m)。
                            重建段一律在畫面標 "(reconstructed)"。

⚠️ 時間軸是**非線性**的:整趟約 14 分鐘而俯衝只有 55 秒,等比例壓縮的話俯衝會
   一閃而過。所以各階段分配固定的影格數(見 BUDGET),畫面右上角會顯示真實時間。

⚠️ 圖上一律用英文標籤:SITL 環境沒有 CJK 字型,中文會變成方框。

用法:
  python3 stot_mission_anim.py --trial S1 --out ~/fixwing_ws/runs/figs/mission.gif
"""
import argparse
import csv
import math
import os
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Wedge, Circle

EARTH = 111320.0
RUNS = os.path.expanduser("~/fixwing_ws/runs")
COL = ["#e6194b", "#3cb44b", "#4363d8", "#f58231", "#911eb4", "#00b8d4"]

HOME = (25.0777, 121.2325)          # LOC=TAOYUAN
HOME_HDG = 52.0
COLUMN_SPACING = 30.0
TKOFF_ALT = 300.0
# jet_rescatter.py 的 OFF 表:(南向, 東向) 公尺,相對目標;巡航高度 130 m
SCATTER_OFF = {1: (3600, -800), 2: (4400, -400), 3: (3800, 0),
               4: (4200, 500), 5: (3700, 1000), 6: (4300, -1300)}
SCATTER_ALT = 130.0

# 各階段分配的影格數(而不是照真實時間等比例)
BUDGET = [("LAUNCH", 26), ("SCATTER", 30), ("ALIGN", 34),
          ("PERCH", 44), ("DIVE", 76)]


def mlon(lat):
    return EARTH * math.cos(math.radians(lat))


def enu(lat, lon, ref):
    """相對參考點的 (East, North) 公尺。"""
    return (lon - ref[1]) * mlon(ref[0]), (lat - ref[0]) * EARTH


def load(trial):
    geom = os.path.join(RUNS, f"tot_{trial}_geom.csv")
    T = None
    slots, pps = [], []
    with open(geom) as f:
        for r in csv.DictReader(f):
            p = (float(r["lat"]), float(r["lon"]))
            if r["kind"] == "target":
                T = p
            elif r["kind"].startswith("slot"):
                slots.append(p)
            elif r["kind"].startswith("pp"):
                pps.append(p)
    zg = {}
    logp = os.path.join(RUNS, f"trial_{trial}.log")
    if os.path.exists(logp):
        pat = re.compile(r"UAV(\d+) 紅心地表 z_g = ([+-]?[\d.]+)")
        for ln in open(logp, encoding="utf-8", errors="replace"):
            m = pat.search(ln)
            if m:
                zg[int(m.group(1))] = float(m.group(2))

    tr = {}
    with open(os.path.join(RUNS, f"tot_{trial}.csv")) as f:
        for r in csv.DictReader(f):
            i = int(r["sysid"])
            tr.setdefault(i, []).append(
                (float(r["t"]), float(r["lat"]), float(r["lon"]),
                 float(r["alt"]), float(r["gs"] or 0), r["phase"], r["mode"]))
    for i in tr:
        tr[i].sort()
    return T, slots, pps, zg, tr


def phase_window(tr, name):
    lo, hi = 1e18, -1e18
    for pts in tr.values():
        for p in pts:
            if p[5] == name:
                lo = min(lo, p[0]); hi = max(hi, p[0])
    return (None if lo > hi else (lo, hi))


def sample(pts, t):
    """線性內插到時間 t(超出範圍就夾到端點)。"""
    if t <= pts[0][0]:
        p = pts[0]
        return p[1], p[2], p[3], p[4]
    if t >= pts[-1][0]:
        p = pts[-1]
        return p[1], p[2], p[3], p[4]
    lo, hi = 0, len(pts) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if pts[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    a, b = pts[lo], pts[hi]
    f = (t - a[0]) / max(1e-6, b[0] - a[0])
    return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]),
            a[3] + f * (b[3] - a[3]), a[4] + f * (b[4] - a[4]))


def load_df(trial):
    """全程真實航跡(stot_extract_df.py 產出);沒有就回 None。"""
    p = os.path.join(RUNS, f"df_{trial}.csv")
    if not os.path.exists(p):
        return None
    tr = {}
    for r in csv.DictReader(open(p)):
        tr.setdefault(int(r["sysid"]), []).append(
            (float(r["t"]), float(r["lat"]), float(r["lon"]),
             float(r["alt"]), float(r["gs"])))
    for i in tr:
        tr[i].sort()
    return tr


def build_frames_auto(T, df):
    """階段直接由**軌跡幾何**推導,不依賴 tot_node 的 CSV。

    給「編隊表演」用:那種航次會先南下到起跑線、再編隊北上,而且 tot_node 可能
    中途掛掉(階段界線就取不到)。用幾何推導的好處是只依賴一份檔案,不必做
    跨檔案的時間對齊 —— 對齊出錯的話整支動畫都是錯的。
    """
    ids = sorted(df)
    ref = df[ids[0]]
    t_end = min(p[-1][0] for p in df.values())

    def dT(rec):
        return math.hypot(*enu(rec[1], rec[2], T))

    # 彈射:高度首次超過 250 m
    t_climb = next((p[0] for p in ref if p[3] > 250.0), 60.0)
    # 最南點:距紅心最遠的時刻 = 抵達起跑線
    t_south = max(ref, key=dT)[0]
    # 四散:距紅心首次進入 12 km(且在最南點之後)
    t_disp = next((p[0] for p in ref if p[0] > t_south and dT(p) < 12000.0), t_end)
    segs = [("LAUNCH — catapult, climb-out", 0.0, t_climb, 22),
            ("TRANSIT — run out to the start line, 30 km south",
             t_climb, t_south, 34),
            ("FORMATION CRUISE — V formation translating north",
             t_south, t_disp, 84),
            ("SCATTER → STOT — break into the 11 km fan",
             t_disp, t_end, 50)]
    frames = []
    for title, t0, t1, n in segs:
        if t1 <= t0:
            continue
        for k in range(n):
            t = t0 + (t1 - t0) * k / (n - 1)
            st = {}
            for i in ids:
                la, lo, al, gs = sample(df[i], t)
                x, y = enu(la, lo, T)
                st[i] = (x, y, al, gs)
            frames.append((title + "   [flight data]", t, st))
    return frames


def build_frames_df(T, df, tot):
    """全程都用 dataflash 的真實資料;階段界線由 tot_node 的 CSV 提供。

    ⚠️ 兩者的時間原點不同(dataflash 是飛控開機起算,tot CSV 是節點啟動起算),
       用「觸地時刻」對齊:兩邊都取各機最接近紅心的那一刻,取中位數差當偏移。
    """
    def closest_t(pts, latlon=(1, 2)):
        best = min(pts, key=lambda p: math.hypot(
            (p[latlon[0]] - T[0]) * EARTH, (p[latlon[1]] - T[1]) * mlon(T[0])))
        return best[0]

    offs = []
    for i in df:
        if i in tot:
            offs.append(closest_t(df[i]) - closest_t(tot[i]))
    offs.sort()
    off = offs[len(offs) // 2] if offs else 0.0

    wins = {}
    for n in ("ALIGN", "PERCH", "DIVE"):
        w = phase_window(tot, n)
        wins[n] = None if w is None else (w[0] + off, w[1] + off)

    t_end = max(p[-1][0] for p in df.values())
    align0 = wins["ALIGN"][0]
    # 彈射段:從記錄開頭到高度首次超過 250 m
    t_climb = 0.0
    for p in df[min(df)]:
        if p[3] > 250.0:
            t_climb = p[0]
            break
    segs = [("LAUNCH — catapult, climb-out", 0.0, max(t_climb, 30.0)),
            ("CRUISE / SCATTER — spread out ahead of the target",
             max(t_climb, 30.0), align0),
            ("ALIGN — fan slots, time-on-target planning", *wins["ALIGN"]),
            ("PERCH — climb to 3100 m, cut in to ring", *wins["PERCH"]),
            ("DIVE — synchronised 20° terminal strike",
             wins["DIVE"][0], min(wins["DIVE"][1], t_end))]
    budget = dict(BUDGET)
    names = [b[0] for b in BUDGET]
    frames = []
    for (title, t0, t1), key in zip(segs, names):
        n = budget[key]
        for k in range(n):
            t = t0 + (t1 - t0) * k / (n - 1)
            st = {}
            for i in sorted(df):
                la, lo, al, gs = sample(df[i], t)
                x, y = enu(la, lo, T)
                st[i] = (x, y, al, gs)
            frames.append((title + "   [flight data]", t, st))
    return frames


def build_frames(T, tr):
    """回傳 [(phase, label_real_time, {uav: (E, N, alt, gs)})]。"""
    wins = {n: phase_window(tr, n) for n in ("ALIGN", "PERCH", "DIVE")}
    t_align0 = wins["ALIGN"][0]
    frames = []

    # ── 重建:彈射 ───────────────────────────────────────────────────
    # ⚠️⚠️ 這兩個迴圈原本寫死 range(1, 7)。實測(FUL1,N=2)後果:
    #    `la, lo, al, gs = end0[i]` 在 i=3 拋 **KeyError: 3**,整個素材產製失敗
    #    (make_artifacts 只印一行「✗ mission_anim」,航次照常結束)。
    #    機號一律從 tr 取 —— 它就是本趟真正有資料的機體。
    _ids = sorted(tr)
    n = dict(BUDGET)["LAUNCH"]
    for k in range(n):
        s = k / (n - 1)
        st = {}
        for i in _ids:
            # HOME 縱列:沿 heading+180 每台間隔 30 m
            b = math.radians(HOME_HDG + 180.0)
            hx = (i - 1) * COLUMN_SPACING * math.sin(b)
            hy = (i - 1) * COLUMN_SPACING * math.cos(b)
            hx0, hy0 = enu(HOME[0], HOME[1], T)
            # 沿跑道航向前衝並爬升
            fx = hx0 + hx + 900.0 * s * math.sin(math.radians(HOME_HDG))
            fy = hy0 + hy + 900.0 * s * math.cos(math.radians(HOME_HDG))
            st[i] = (fx, fy, 10.0 + (TKOFF_ALT - 10.0) * s ** 0.7, 30 + 70 * s)
        frames.append(("LAUNCH  (reconstructed)", 0.0 + 60 * s, st))

    # ── 重建:四散到巡航點 ─────────────────────────────────────────
    n = dict(BUDGET)["SCATTER"]
    end0 = {i: sample(tr[i], t_align0) for i in tr}
    for k in range(n):
        s = k / (n - 1)
        st = {}
        for i in _ids:
            sx, sy, sa = frames[dict(BUDGET)["LAUNCH"] - 1][2][i][:3]
            # ⚠️ SCATTER_OFF 是 1..6 的字典;機號超出時退回第一組,不要 KeyError。
            dn, de = SCATTER_OFF.get(i, SCATTER_OFF[min(SCATTER_OFF)])
            cx, cy = de, -dn                       # 相對目標的 ENU
            la, lo, al, gs = end0[i]
            ex, ey = enu(la, lo, T)
            # 前半段飛到四散點,後半段接到 tot_node 起始位置
            if s < 0.72:
                u = s / 0.72
                x = sx + (cx - sx) * u
                y = sy + (cy - sy) * u
                z = sa + (SCATTER_ALT - sa) * u
            else:
                u = (s - 0.72) / 0.28
                x = cx + (ex - cx) * u
                y = cy + (ey - cy) * u
                z = SCATTER_ALT + (al - SCATTER_ALT) * u
            st[i] = (x, y, z, 70.0)
        frames.append(("CRUISE / SCATTER  (reconstructed)", 60 + 300 * s, st))

    # ── 真實資料:ALIGN / PERCH / DIVE ──────────────────────────────
    for name, title in (("ALIGN", "ALIGN — fan slots, time-on-target planning"),
                        ("PERCH", "PERCH — climb to 3100 m, cut in to ring"),
                        ("DIVE", "DIVE — synchronised 20° terminal strike")):
        w = wins[name]
        if w is None:
            continue
        n = dict(BUDGET)[name]
        for k in range(n):
            t = w[0] + (w[1] - w[0]) * k / (n - 1)
            st = {}
            for i in sorted(tr):
                la, lo, al, gs = sample(tr[i], t)
                x, y = enu(la, lo, T)
                st[i] = (x, y, al, gs)
            frames.append((title + "   [flight data]", 360 + t - t_align0, st))
    return frames


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trial", default="S1")
    ap.add_argument("--out", default=os.path.join(RUNS, "figs", "mission_full.gif"))
    ap.add_argument("--auto-phase", action="store_true",
                    help="階段由軌跡幾何推導,不讀 tot_node 的 CSV")
    ap.add_argument("--span", type=float, default=0.0,
                    help="俯視圖的半邊範圍(m);0 = 依資料自動")
    ap.add_argument("--fps", type=int, default=16)
    ap.add_argument("--dpi", type=int, default=80)
    a = ap.parse_args()
    os.makedirs(os.path.dirname(a.out), exist_ok=True)

    T, slots, pps, zg, tr = load(a.trial)
    df = load_df(a.trial)
    if df and a.auto_phase:
        print("階段由軌跡幾何推導(適合編隊表演 / tot_node 中途掛掉的航次)")
        frames = build_frames_auto(T, df)
    elif df:
        print("使用全程 dataflash 真實航跡(df_%s.csv)" % a.trial)
        frames = build_frames_df(T, df, tr)
    else:
        print("⚠️ 找不到 df_%s.csv,彈射/四散段改用重建 —— "
              "先跑 stot_extract_df.py 才會全程都是真實資料" % a.trial)
        frames = build_frames(T, tr)
    ring = math.hypot(*enu(pps[0][0], pps[0][1], T)) if pps else 8649.0

    fig = plt.figure(figsize=(14.5, 7.6), dpi=a.dpi)
    gs_ = fig.add_gridspec(2, 2, width_ratios=[1.55, 1], height_ratios=[1, 1],
                           hspace=0.30, wspace=0.20,
                           left=0.055, right=0.985, top=0.885, bottom=0.075)
    axm = fig.add_subplot(gs_[:, 0])
    axz = fig.add_subplot(gs_[0, 1])
    axd = fig.add_subplot(gs_[1, 1])

    # ── 主圖:俯視 ────────────────────────────────────────────────
    axm.add_patch(Wedge((0, 0), 11000, 90 - 205, 90 - 155,
                        facecolor="#9ecae1", alpha=0.13, ec="none"))
    axm.add_patch(Circle((0, 0), ring, fill=False, ls="--", lw=1.4,
                         ec="#2166ac", alpha=0.8))
    axm.add_patch(Circle((0, 0), 3400, fill=False, ls=":", lw=1.2,
                         ec="#b2182b", alpha=0.8))
    for s in slots:
        x, y = enu(s[0], s[1], T)
        axm.plot(x, y, "s", color="0.55", ms=5, alpha=0.7)
    hx, hy = enu(HOME[0], HOME[1], T)
    axm.plot(hx, hy, "^", color="0.25", ms=10)
    axm.annotate("HOME / catapult", (hx, hy), textcoords="offset points",
                 xytext=(12, -4), fontsize=8, color="0.25")
    axm.plot(0, 0, "*", color="#d62728", ms=24, zorder=9)
    axm.annotate("BULLSEYE", (0, 0), textcoords="offset points",
                 xytext=(14, 10), fontsize=10, color="#d62728", weight="bold")
    # 編隊表演的航程遠得多(起跑線在南方 30 km),範圍要依資料自動撐開
    if a.span > 0:
        span = a.span
    else:
        span = max(2000.0, max(max(abs(v[0]), abs(v[1]))
                               for _, _, st in frames for v in st.values()) * 1.08)
    axm.set_xlim(-span * 0.55, span * 0.55)
    axm.set_ylim(-span * 1.02, span * 0.30)
    axm.set_aspect("equal")
    axm.grid(alpha=0.22, ls=":")
    axm.set_xlabel("East (m)")
    axm.set_ylabel("North (m)")

    dots = [axm.plot([], [], "o", color=COL[k], ms=8, zorder=10)[0] for k in range(6)]
    trails = [axm.plot([], [], "-", color=COL[k], lw=1.3, alpha=0.7)[0] for k in range(6)]

    # ── 右上:高度 ────────────────────────────────────────────────
    axz.set_xlim(0, len(frames))
    axz.set_ylim(-200, 3500)
    axz.grid(alpha=0.22, ls=":")
    axz.set_ylabel("altitude (m)")
    axz.set_title("altitude", fontsize=9)
    axz.tick_params(labelbottom=False)
    zl = [axz.plot([], [], "-", color=COL[k], lw=1.3)[0] for k in range(6)]

    # ── 右下:距目標 ──────────────────────────────────────────────
    axd.set_xlim(0, len(frames))
    axd.set_ylim(0, max(12500.0, span * 1.05))
    axd.grid(alpha=0.22, ls=":")
    axd.set_ylabel("range to bullseye (m)")
    axd.set_xlabel("animation frame  (time axis is non-linear — see clock)")
    axd.set_title("range to bullseye — convergence to a single point", fontsize=9)
    dl = [axd.plot([], [], "-", color=COL[k], lw=1.3)[0] for k in range(6)]

    banner = fig.text(0.5, 0.955, "", ha="center", fontsize=14, weight="bold")
    clock = fig.text(0.985, 0.955, "", ha="right", fontsize=10,
                     family="monospace", color="0.3")
    stats = axm.text(0.02, 0.02, "", transform=axm.transAxes, fontsize=9,
                     family="monospace", va="bottom",
                     bbox=dict(fc="white", ec="0.7", alpha=0.9))

    hx_, hy_, hz_, hd_ = ([[] for _ in range(6)], [[] for _ in range(6)],
                          [[] for _ in range(6)], [[] for _ in range(6)])

    def frame(fi):
        ph, tt, st = frames[fi]
        banner.set_text(ph)
        clock.set_text("mission clock  %02d:%02d" % (int(tt) // 60, int(tt) % 60))
        # ⚠️⚠️ 原本 range spread 寫成 `max(hd_[k][-1] for k in range(6))`,
        #    而 hd_ 是 6 條空 list、只有本趟真的有的機體會被 append
        #    ⇒ N=2 時 hd_[2] 仍是空的,`[-1]` 直接 IndexError,整支素材腳本掛掉
        #    (FUL1 實測)。用**本幀真的有資料**的那幾條算,不要用固定 6。
        vs, zs, ds = [], [], []
        for k in range(6):
            i = k + 1
            if i not in st:
                continue
            x, y, z, v = st[i]
            hx_[k].append(x); hy_[k].append(y); hz_[k].append(z)
            hd_[k].append(math.hypot(x, y))
            dots[k].set_data([x], [y])
            trails[k].set_data(hx_[k], hy_[k])
            zl[k].set_data(range(len(hz_[k])), hz_[k])
            dl[k].set_data(range(len(hd_[k])), hd_[k])
            vs.append(v); zs.append(z); ds.append(hd_[k][-1])
        if vs:
            stats.set_text("ground speed  %5.1f – %5.1f m/s\n"
                           "altitude      %5.0f – %5.0f m\n"
                           "range spread  %5.0f m"
                           % (min(vs), max(vs), min(zs), max(zs),
                              max(ds) - min(ds)))
        return dots + trails + zl + dl + [banner, clock, stats]

    anim = animation.FuncAnimation(fig, frame, frames=len(frames),
                                   interval=1000 / a.fps, blit=False)
    anim.save(a.out, writer=animation.PillowWriter(fps=a.fps))
    plt.close(fig)
    print(a.out, "%.1f MB" % (os.path.getsize(a.out) / 1048576))


if __name__ == "__main__":
    main()
