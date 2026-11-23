#!/usr/bin/env python3
"""stot_stage2_report.py — 階段 2(巡航編隊)的逐機驗收。

⚠️ 這支是重寫的:交接文件引用的 `coupling.py` / `formation_check.py` /
   `cmd_speed_check.py` 都只存在於臨時 shell,換一輪 session 就消失了。
   放進 repo 才不會每次都要重造。

驗收門檻(使用者定調):4 km 間距下 **worst slot 誤差 ≤ 200 m**(耦合度 5%)。

資料來源
  runs/show_<T>.csv        全隊軌跡 `elapsed_s,uav_id,lat,lon,alt,phase`(~2 Hz)
  runs/cmdlog/<T>/*.csv    指令記錄器(逐趟;舊版是覆寫式的單一目錄)

做的事
  1. 由長機(UAV1)軌跡平滑地估航向,重建每架僚機的**真 slot**(極座標 V)
  2. 誤差分解成沿航向 / 垂直航向 —— 兩者的權限完全不同:
       沿航向靠速度差(有權限);垂直航向靠橫向導引(也有權限);
       但「目標在身後」時飛機會轉圈,兩者都拿不到。
  3. 指令記錄器:身後航點比例、CLIMB/CRUISE 相位翻轉次數
     ⇒ 翻轉次數是高度閘門遲滯的直接驗收量(見 sar_swarm/phase_gate.py)

用法:
    python3 stot_stage2_report.py Q1
    python3 stot_stage2_report.py Q1 --v-spacing 4000 --v-angle 35
"""
import argparse
import csv
import glob
import math
import os
import statistics as st

EARTH = 111320.0
V_RANK_SIDE = {2: (1, +1), 3: (2, +1), 4: (1, -1), 5: (2, -1)}
LEADER = 1
SLOT_TOL_M = 200.0


def mlon(lat):
    return EARTH * math.cos(math.radians(lat))


def enu(p, origin):
    """(lat,lon) → (east, north) metres relative to origin."""
    return ((p[1] - origin[1]) * mlon(origin[0]),
            (p[0] - origin[0]) * EARTH)


def polar_slot(lat, lon, heading_deg, distance_m, bearing_off_deg):
    brg = math.radians(heading_deg + bearing_off_deg)
    return (lat + distance_m * math.cos(brg) / EARTH,
            lon + distance_m * math.sin(brg) / mlon(lat))


def load_show(path):
    """→ {t: {uav: (lat, lon, alt)}} sorted by t."""
    frames = {}
    with open(path, newline="") as fh:
        for row in csv.reader(fh):
            if len(row) < 5:
                continue
            try:
                t, uav = float(row[0]), int(row[1])
                lat, lon, alt = float(row[2]), float(row[3]), float(row[4])
            except ValueError:
                continue
            frames.setdefault(round(t, 1), {})[uav] = (lat, lon, alt)
    return dict(sorted(frames.items()))


def leader_headings(frames, smooth_s=6.0):
    """由長機軌跡估航向(度)。用 smooth_s 秒的位移向量,壓掉取樣雜訊。"""
    ts = [t for t, f in frames.items() if LEADER in f]
    out = {}
    for i, t in enumerate(ts):
        j = i
        while j + 1 < len(ts) and ts[j + 1] - t < smooth_s:
            j += 1
        if j == i:
            j = min(len(ts) - 1, i + 1)
        if j == i:
            continue
        a, b = frames[ts[i]][LEADER], frames[ts[j]][LEADER]
        de = (b[1] - a[1]) * mlon(a[0])
        dn = (b[0] - a[0]) * EARTH
        if math.hypot(de, dn) < 1.0:
            continue
        out[t] = math.degrees(math.atan2(de, dn)) % 360.0
    return out


def slot_errors(frames, hdgs, v_spacing, v_angle, v_tail):
    """→ {uav: [(t, err, along, cross)]}; along>0 = 在 slot 前方。"""
    out = {}
    for t, f in frames.items():
        if LEADER not in f or t not in hdgs:
            continue
        L, h = f[LEADER], hdgs[t]
        for uav, p in f.items():
            if uav == LEADER:
                continue
            if uav in V_RANK_SIDE:
                rank, side = V_RANK_SIDE[uav]
                dist, boff = rank * v_spacing, 180.0 + side * v_angle
            else:                       # UAV6 = 備長,在 V 尾正後方
                dist, boff = v_tail, 180.0
            s = polar_slot(L[0], L[1], h, dist, boff)
            de, dn = enu(p, s)          # slot → 飛機 的向量
            hb = math.radians(h)
            along = dn * math.cos(hb) + de * math.sin(hb)
            cross = -dn * math.sin(hb) + de * math.cos(hb)
            out.setdefault(uav, []).append((t, math.hypot(de, dn),
                                            along, cross))
    return out


def cmdlog_dir(trial, runs):
    """逐趟目錄優先;`cmdlog_<TRIAL>` 是舊版覆寫式記錄的手動備份命名。
    都沒有才退回共用目錄 —— 那裡幾乎一定是**別趟**的資料,要標出來。"""
    for cand in (os.path.join(runs, "cmdlog", trial),
                 os.path.join(runs, f"cmdlog_{trial}")):
        if os.path.isdir(cand) and glob.glob(os.path.join(cand, "*.csv")):
            return cand, True
    return os.path.join(runs, "cmdlog"), False


def cmdlog_stats(path):
    """→ (rows, behind_pct, phase_flips, climb_rows, alt_min, alt_p50).

    ⚠️ 翻轉次數要**配著最低高度一起看**,兩種成因完全不同:
      - 淺(離門檻只有幾十 m)= 閘門在抖,遲滯沒生效 → 是 bug
      - 深(掉了幾百 m 以上)= 飛機真的掉高度 → 遲滯正確地放行,是能量問題
    只看次數會把後者誤判成前者(實測 U1 就是:掉了 1000 m,不是抖動)。
    """
    rows, behind, flips, climb, prev = 0, 0, 0, 0, None
    alts = []
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            try:
                brg = float(r["bearing_to_wp_deg"])
            except (ValueError, KeyError):
                continue
            rows += 1
            ph = r.get("flight_phase", "")
            if abs(brg) > 90.0:
                behind += 1
            if ph == "CLIMB":
                climb += 1
            if prev is not None and ph != prev:
                flips += 1
            prev = ph
            try:
                alts.append(float(r["current_alt_amsl"]))
            except (ValueError, KeyError):
                pass
    return (rows, (behind / rows * 100 if rows else 0.0), flips, climb,
            (min(alts) if alts else float("nan")),
            (st.median(alts) if alts else float("nan")))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trial")
    ap.add_argument("--runs", default=os.path.expanduser("~/fixwing_ws/runs"))
    ap.add_argument("--v-spacing", type=float,
                    default=float(os.environ.get("V_SPACING", 4000.0)))
    ap.add_argument("--v-angle", type=float,
                    default=float(os.environ.get("V_ANGLE", 35.0)))
    ap.add_argument("--v-tail", type=float, default=None)
    ap.add_argument("--tail-window", type=float, default=60.0,
                    help="末端取多少秒做驗收(展開斜坡 120 s 之後才有意義)")
    ap.add_argument("--tol", type=float, default=SLOT_TOL_M)
    a = ap.parse_args()
    v_tail = a.v_tail if a.v_tail is not None else a.v_spacing * 1.5

    show = os.path.join(a.runs, f"show_{a.trial}.csv")
    if not os.path.exists(show):
        raise SystemExit(f"✗ 找不到 {show} —— 這趟的階段 2 沒有跑或沒有記錄")
    frames = load_show(show)
    if not frames:
        raise SystemExit(f"✗ {show} 是空的")
    hdgs = leader_headings(frames)
    errs = slot_errors(frames, hdgs, a.v_spacing, a.v_angle, v_tail)
    t_end = max(frames)
    t_cut = t_end - a.tail_window

    print(f"══ 階段 2 驗收:{a.trial} ══")
    print(f"   幾何 間距 {a.v_spacing:.0f} m / 半角 {a.v_angle:.0f}° / "
          f"V尾 {v_tail:.0f} m;巡航窗口 {t_end:.0f} s")
    print(f"   ⚠️ 側向展開斜坡 120 s;末端 {a.tail_window:.0f} s "
          f"(t {t_cut:.0f}–{t_end:.0f}) 才是收斂後的成績\n")

    print(f"{'uav':>4} {'err_end':>9} {'along':>9} {'cross':>9} "
          f"{'err_min':>9} {'err_p50':>9} {'趨勢':>8}")
    worst, worst_uav = -1.0, None
    for uav in sorted(errs):
        seq = errs[uav]
        tail = [e for e in seq if e[0] >= t_cut] or seq[-5:]
        e_end = st.median(e[1] for e in tail)
        al = st.median(e[2] for e in tail)
        cr = st.median(e[3] for e in tail)
        head = st.median(e[1] for e in seq[:max(1, len(seq) // 4)])
        trend = "收斂" if e_end < head else "發散"
        if e_end > worst:
            worst, worst_uav = e_end, uav
        print(f"{uav:>4} {e_end:>9.0f} {al:>+9.0f} {cr:>+9.0f} "
              f"{min(e[1] for e in seq):>9.0f} "
              f"{st.median(e[1] for e in seq):>9.0f} {trend:>8}")

    ok = worst <= a.tol
    print(f"\n   worst slot 誤差 = {worst:.0f} m (UAV{worst_uav});"
          f"門檻 {a.tol:.0f} m → {'✅ 通過' if ok else '❌ 未通過'}")

    # ── 指令記錄器 ────────────────────────────────────────────────────
    cdir, per_trial = cmdlog_dir(a.trial, a.runs)
    paths = sorted(glob.glob(os.path.join(cdir, "uav*_cmd_telemetry.csv")))
    if paths:
        tag = "逐趟" if per_trial else "⚠️ 共用目錄(可能是別趟的資料)"
        print(f"\n── 指令記錄器({tag}:{cdir})──")
        print(f"{'uav':>5} {'rows':>6} {'behind%':>8} {'相位翻轉':>9} "
              f"{'CLIMB列':>8} {'alt_p50':>8} {'alt_min':>8} {'判讀':>12}")
        for p in paths:
            uav = os.path.basename(p).split("_")[0]
            rows, behind, flips, climb, amin, ap50 = cmdlog_stats(p)
            # 淺 vs 深:離巡航高度掉了多少
            sag = (ap50 - amin) if amin == amin else 0.0
            if flips <= 2:
                verdict = "正常"
            elif sag < 300.0:
                verdict = "閘門抖動"      # 淺 → 遲滯沒生效,是 bug
            else:
                verdict = "真的掉高度"    # 深 → 遲滯正確放行,是能量問題
            print(f"{uav:>5} {rows:>6} {behind:>8.1f} {flips:>9} "
                  f"{climb:>8} {ap50:>8.0f} {amin:>8.0f} {verdict:>12}")
        print("   ⚠️ 翻轉次數要配著 alt_min 看:淺(<300 m)才是閘門抖動;")
        print("      深的是飛機真的掉高度 —— 遲滯正確放行,問題在能量不在閘門。")
    else:
        print(f"\n   (沒有指令記錄器資料:{cdir})")

    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
