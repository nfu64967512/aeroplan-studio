#!/usr/bin/env python3
"""swarm_test_report.py — 編隊測試可視化報告(每個測試結束自動出圖)。

用法(WSL,ROS2 環境;在測試「開始前」啟動、跑滿 --duration 自動出圖):
    source /opt/ros/humble/setup.bash
    python3 swarm_test_report.py --duration 240 --label "T2-起飛即進隊" \
        --out /tmp/report_t2
產出:
    <out>.png  四面板報告:A 各機 slot 誤差時間線(成形門檻/釋放門檻)
                B 長機地速(手風琴:等待/巡航線)
                C 俯視 V 隊形快照(實際 vs slot 目標 + 誤差向量 + 航跡)
                D worst-err 趨勢 + 統計(成形佔比/巡航佔比/各機 p95)
    <out>.csv  原始樣本(可重繪/歸檔)

資料源(sar_swarm 既有 topic,零侵入):
    /swarm/whoLeader          {id,lat,lon,alt,heading}  10 Hz(長機位姿)
    /swarm/formation_status   {"id":n,"err":m}          各僚機自報 slot 誤差
    /uavN/mavros/global_position/global                 各機軌跡
slot 目標由 whoLeader 位姿 + V 幾何(V_RANK_SIDE 同 wingman_node)重算,
僅用於作圖;誤差一律以僚機自報值為準(與驗收判準同源)。
"""
import argparse, csv, json, math, os, time

EARTH = 111320.0
# 與 wingman_node.V_RANK_SIDE 一致:uav_id -> (rank, side);side +1=後左、-1=後右
V_RANK_SIDE = {2: (1, +1), 3: (2, +1), 4: (1, -1), 5: (2, -1)}


def slot_of(uav_id, L, spacing, angle, tail):
    """由長機位姿算 uav_id 的 slot(lat, lon)。UAV6 = 正後 V 尾。"""
    if uav_id == 6:
        dist, brg_off = tail, 180.0
    else:
        rank, side = V_RANK_SIDE.get(uav_id, (1, 0))
        dist, brg_off = rank * spacing, 180.0 + side * angle
    brg = math.radians(L["heading"] + brg_off)
    dn, de = dist * math.cos(brg), dist * math.sin(brg)
    return (L["lat"] + dn / EARTH,
            L["lon"] + de / (EARTH * math.cos(math.radians(L["lat"]))))


def en_m(lat, lon, ref_lat, ref_lon):
    """(lat,lon) 相對 (ref) 的 (east, north) 公尺。"""
    return ((lon - ref_lon) * EARTH * math.cos(math.radians(ref_lat)),
            (lat - ref_lat) * EARTH)


def make_recorder(n_uav):
    """錄製模式才需要 ROS2;--replay 純離線重繪,不 import rclpy。"""
    from rclpy.node import Node
    from rclpy.qos import QoSPresetProfiles
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import String

    class Recorder(Node):
        def __init__(self, n):
            super().__init__("swarm_test_report")
            self.t0 = time.monotonic()
            self.leader, self.errs, self.pos = [], {}, {}
            qos = QoSPresetProfiles.SENSOR_DATA.value
            # /swarm/* 用 HEARTBEAT_QOS(BEST_EFFORT/depth1)發布 — 訂閱端
            # 必須同為 BEST_EFFORT(sensor 檔),否則 QoS 不相容、靜默收不到
            self.create_subscription(String, "/swarm/whoLeader",
                                     self._on_leader, qos)
            self.create_subscription(String, "/swarm/formation_status",
                                     self._on_err, qos)
            for i in range(1, n + 1):
                self.create_subscription(
                    NavSatFix, f"/uav{i}/mavros/global_position/global",
                    lambda m, i=i: self._on_fix(i, m), qos)

        def _t(self):
            return time.monotonic() - self.t0

        def _on_leader(self, msg):
            try:
                d = json.loads(msg.data)
                self.leader.append((self._t(), int(d["id"]), float(d["lat"]),
                                    float(d["lon"]), float(d.get("alt", 0.0)),
                                    float(d["heading"])))
            except (ValueError, KeyError):
                pass

        def _on_err(self, msg):
            try:
                d = json.loads(msg.data)
                e = float(d["err"])
                if e >= 0:
                    self.errs.setdefault(int(d["id"]), []).append((self._t(), e))
            except (ValueError, KeyError):
                pass

        def _on_fix(self, i, m):
            if abs(m.latitude) < 1e-3 and abs(m.longitude) < 1e-3:
                return
            self.pos.setdefault(i, []).append(
                (self._t(), m.latitude, m.longitude, m.altitude))

    return Recorder(n_uav)


def leader_speed(leader):
    """whoLeader 位置差分 → 地速樣本 [(t, m/s)](2 s 窗平滑)。"""
    out, win = [], 2.0
    j = 0
    for i in range(len(leader)):
        t1, _, la1, lo1, _, _ = leader[i]
        while j < i and t1 - leader[j][0] > win:
            j += 1
        t0, _, la0, lo0, _, _ = leader[j]
        if t1 - t0 >= 0.8:
            de, dn = en_m(la1, lo1, la0, lo0)
            out.append((t1, math.hypot(de, dn) / (t1 - t0)))
    return out


def _setup_cjk():
    """WSL 無中文字型 → 從 Windows 掛載點載入(微軟正黑等),圖表才不會滿版豆腐。"""
    import matplotlib
    from matplotlib import font_manager
    for f in ("/mnt/c/Windows/Fonts/msjh.ttc", "/mnt/c/Windows/Fonts/msyh.ttc",
              "/mnt/c/Windows/Fonts/mingliu.ttc", "/mnt/c/Windows/Fonts/simsun.ttc"):
        if os.path.exists(f):
            font_manager.fontManager.addfont(f)
            name = font_manager.FontProperties(fname=f).get_name()
            matplotlib.rcParams["font.family"] = [name, "DejaVu Sans"]
            matplotlib.rcParams["axes.unicode_minus"] = False
            return name
    return None


def render(args, rec, png, csvp):
    import matplotlib
    matplotlib.use("Agg")
    _setup_cjk()
    import matplotlib.pyplot as plt

    gate, rel = args.formed_r, args.formed_r * args.hyst
    fig = plt.figure(figsize=(15, 10))
    gs = fig.add_gridspec(2, 2, hspace=0.32, wspace=0.24)
    ids = sorted(rec.errs)
    colors = {2: "#2a7de1", 3: "#7b52d1", 4: "#1fa87a", 5: "#d17f2a", 6: "#c23b3b"}

    # ── A:各機 slot 誤差時間線 ───────────────────────────────────
    ax = fig.add_subplot(gs[0, 0])
    for i in ids:
        ts = rec.errs[i]
        ax.plot([t for t, _ in ts], [e for _, e in ts],
                color=colors.get(i, "#888"), lw=1.4, label=f"UAV{i}")
    ax.axhline(gate, color="#333", ls="--", lw=1)
    ax.text(0, gate, f" 成形門檻 {gate:.0f}", va="bottom", fontsize=8)
    ax.axhline(rel, color="#c23b3b", ls=":", lw=1)
    ax.text(0, rel, f" 釋放門檻 {rel:.0f}", va="bottom", fontsize=8, color="#c23b3b")
    ax.set_xlabel("t (s)"); ax.set_ylabel("slot err (m)")
    ax.set_title("A. 各機 slot 誤差(自報,與驗收同源)")
    ax.grid(ls=":", alpha=0.5); ax.legend(fontsize=8, ncol=3)

    # ── B:長機地速(手風琴)─────────────────────────────────────
    ax = fig.add_subplot(gs[0, 1])
    sp = leader_speed(rec.leader)
    if sp:
        ax.plot([t for t, _ in sp], [v for _, v in sp], color="#c23b3b", lw=1.5)
    for v, lbl, c in ((args.wait, f"wait {args.wait:.0f}", "#888"),
                      (args.nominal, f"nominal {args.nominal:.0f}", "#2a7de1")):
        ax.axhline(v, ls="--", lw=1, color=c)
        ax.text(0, v, " " + lbl, va="bottom", fontsize=8, color=c)
    cruise = [v for _, v in sp if v > (args.wait + args.nominal) / 2]
    frac = 100.0 * len(cruise) / max(len(sp), 1)
    ax.set_xlabel("t (s)"); ax.set_ylabel("leader groundspeed (m/s)")
    ax.set_title(f"B. 長機地速 — 手風琴(巡航佔比 {frac:.0f}%)")
    ax.grid(ls=":", alpha=0.5)

    # ── C:俯視 V 快照(長機航向朝上)──────────────────────────────
    ax = fig.add_subplot(gs[1, 0])
    if rec.leader:
        tL, lid, la, lo, _, hdg = rec.leader[-1]
        L = {"lat": la, "lon": lo, "heading": hdg}
        ch, sh = math.cos(math.radians(-hdg)), math.sin(math.radians(-hdg))

        def frame(lat, lon):     # ENU → 航向朝上座標
            e, n = en_m(lat, lon, la, lo)
            return e * ch - n * sh, e * sh + n * ch

        for i, tr in sorted(rec.pos.items()):
            tr = [p for p in tr if tL - p[0] <= 90.0]
            if not tr:
                continue
            xs, ys = zip(*[frame(p[1], p[2]) for p in tr])
            c = "#c23b3b" if i == lid else colors.get(i, "#888")
            ax.plot(xs, ys, color=c, lw=0.7, alpha=0.45)
            ax.scatter(xs[-1], ys[-1], s=200 if i == lid else 130, c=c,
                       marker="^", edgecolors="black", zorder=4)
            ax.annotate(f"UAV{i}", (xs[-1], ys[-1]), textcoords="offset points",
                        xytext=(7, 7), fontsize=9, fontweight="bold")
            if i != lid and i in rec.errs:   # 只給「實際回報編隊誤差」的僚機
                sx, sy = frame(*slot_of(i, L, args.spacing, args.angle, args.tail))
                ax.scatter(sx, sy, s=130, facecolors="none",
                           edgecolors=c, lw=1.5, zorder=3)
                ax.plot([sx, xs[-1]], [sy, ys[-1]], color=c, ls="--", lw=1)
            elif i != lid:                   # 在空中但未入隊(如腦裂的前長機)
                ax.annotate("(獨立/未入隊)", (xs[-1], ys[-1]),
                            textcoords="offset points", xytext=(7, -14),
                            fontsize=8, color="#a33")
        ax.arrow(0, 0, 0, args.spacing * 0.8, head_width=args.spacing * 0.12,
                 color="#c23b3b", alpha=0.5)
    ax.set_aspect("equal"); ax.grid(ls=":", alpha=0.5)
    ax.set_xlabel("cross-track (m)"); ax.set_ylabel("along-track (m)")
    ax.set_title("C. 俯視快照 — 實心=實際、空心=slot 目標、虛線=誤差向量(90 s 航跡)")

    # ── D:worst-err 趨勢 + 統計 ─────────────────────────────────
    ax = fig.add_subplot(gs[1, 1])
    allts = sorted({round(t) for i in ids for t, _ in rec.errs[i]})
    worst = []
    for tb in allts:
        vals = [e for i in ids for t, e in rec.errs[i] if abs(t - tb) <= 0.5]
        if vals:
            worst.append((tb, max(vals)))
    if worst:
        ax.plot([t for t, _ in worst], [e for _, e in worst],
                color="#333", lw=1.6, label="worst follower err")
        formed_frac = 100.0 * sum(1 for _, e in worst if e <= gate) / len(worst)
    else:
        formed_frac = 0.0
    ax.axhline(gate, color="#333", ls="--", lw=1)
    ax.axhline(rel, color="#c23b3b", ls=":", lw=1)
    lines = [f"成形佔比(worst≤{gate:.0f}): {formed_frac:.0f}%"]
    for i in ids:
        es = sorted(e for _, e in rec.errs[i])
        if es:
            p95 = es[min(len(es) - 1, int(0.95 * len(es)))]
            lines.append(f"UAV{i}: mean {sum(es)/len(es):5.0f}  "
                         f"p95 {p95:5.0f}  max {es[-1]:5.0f}")
    ax.text(0.02, 0.97, "\n".join(lines), transform=ax.transAxes, va="top",
            fontsize=9,  # 不指定 monospace — 會蓋掉 CJK 字型變豆腐
            bbox=dict(fc="white", alpha=0.85, ec="#999"))
    ax.set_xlabel("t (s)"); ax.set_ylabel("worst err (m)")
    ax.set_title("D. worst-err 趨勢 + 統計")
    ax.grid(ls=":", alpha=0.5); ax.legend(fontsize=8, loc="lower right")

    fig.suptitle(f"編隊測試報告 — {args.label}   "
                 f"(S={args.spacing:.0f} θ={args.angle:.0f}° tail={args.tail:.0f} "
                 f"gate={gate:.0f} hyst={args.hyst})", fontsize=13)
    fig.savefig(png, dpi=120, bbox_inches="tight")

    with open(csvp, "w", newline="") as f:      # 原始樣本歸檔
        w = csv.writer(f)
        w.writerow(["type", "t", "id", "a", "b", "c", "d"])
        for row in rec.leader:
            w.writerow(["leader", *row])
        for i in ids:
            for t, e in rec.errs[i]:
                w.writerow(["err", t, i, e, "", "", ""])
        for i, tr in sorted(rec.pos.items()):
            for t, la, lo, al in tr:
                w.writerow(["pos", t, i, la, lo, al, ""])
    print(f"[report] {png}\n[report] {csvp}")


class Replay:
    """從 --replay 的 CSV 重建 Recorder 資料(免重飛即可重繪/改圖)。"""
    def __init__(self, path):
        self.leader, self.errs, self.pos = [], {}, {}
        with open(path) as f:
            for row in csv.reader(f):
                if row[0] == "leader":
                    self.leader.append((float(row[1]), int(row[2]), float(row[3]),
                                        float(row[4]), float(row[5]), float(row[6])))
                elif row[0] == "err":
                    self.errs.setdefault(int(row[2]), []).append(
                        (float(row[1]), float(row[3])))
                elif row[0] == "pos":
                    self.pos.setdefault(int(row[2]), []).append(
                        (float(row[1]), float(row[3]), float(row[4]), float(row[5])))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--replay", default=None, help="從既有 CSV 重繪(不錄製)")
    p.add_argument("--duration", type=float, default=240.0)
    p.add_argument("--label", default="test")
    p.add_argument("--out", default="/tmp/swarm_report")
    p.add_argument("--n", type=int, default=6)
    p.add_argument("--spacing", type=float, default=60.0)   # V_SPACING
    p.add_argument("--angle", type=float, default=25.0)     # V_ANGLE
    p.add_argument("--tail", type=float, default=120.0)     # V_TAIL
    p.add_argument("--formed-r", type=float, default=95.0)  # FORMED_RADIUS
    p.add_argument("--hyst", type=float, default=1.8)       # FORMED_HYST
    p.add_argument("--wait", type=float, default=16.0)
    p.add_argument("--nominal", type=float, default=27.0)
    args = p.parse_args()

    if args.replay:
        render(args, Replay(args.replay), args.out + ".png", args.out + ".csv")
        return

    import rclpy
    rclpy.init()
    rec = make_recorder(args.n)
    end = time.monotonic() + args.duration
    print(f"[report] recording {args.duration:.0f}s — label: {args.label}")
    while time.monotonic() < end:
        rclpy.spin_once(rec, timeout_sec=0.2)
    rec.destroy_node()
    rclpy.shutdown()
    n_err = sum(len(v) for v in rec.errs.values())
    print(f"[report] samples: leader={len(rec.leader)} err={n_err} "
          f"pos={sum(len(v) for v in rec.pos.values())}")
    render(args, rec, args.out + ".png", args.out + ".csv")


if __name__ == "__main__":
    main()
