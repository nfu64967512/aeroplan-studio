#!/usr/bin/env python3
"""jet_show_cruise.py — 長程表演的「編隊巡航 → 四散」段(STOT/俯衝交給 tot_node)。

為什麼不用 sar_swarm 編隊節點:那是「追逐律」(僚機追移動中的 slot),噴射機
最小可持速 ~45-75 m/s、迴轉半徑 ~500 m,追逐律只能收斂成 ~300 m 鬆散團,
本 repo 已實測且記錄。表演要的是「看起來整齊」,所以這裡改成**絕對位置編隊**:
以虛擬長機沿航線等速前進,每 tick 直接把各機指到它自己的 V slot 絕對座標。
飛控只要飛到指定點,不必追別人 → 直線段的視覺整齊度高得多。

階段:
  FORMUP  各機飛到起跑線上的 V slot(就位)
  CRUISE  虛擬長機沿航線北進,V 陣形整體平移(長程巡航,表演主秀)
  SCATTER 到達散開點 → 各機飛向目標前扇形弧上的自己那格(四散)

用法: python3 jet_show_cruise.py [cruise_speed] [v_spacing]
"""
import sys, math, time, csv
from pymavlink import mavutil

EARTH = 111320.0
# 分流分配(MAVProxy 的 --out tcpin 只把遙測餵給「先連上的」client,所以三方要分家):
#   5860+10i → 本腳本(MAVROS 沒跑時是空的)   6760+10i → AeroPlan   6960+10i → tot_node
# 之前本腳本和 tot_node 搶 6960:腳本被 SIGTERM 殺掉後 socket 殘留,
# 下一輪就讀不到位置(worst 被算成 0m、評分段無樣本),量測整個失真。
BASE, STRIDE, N = 5860, 10, 6

T = (25.1406, 121.2325)        # 紅心
START = (25.0000, 121.2325)    # 起跑線(T 南方 ~15.6 km)
DISPERSE_D = 5000.0            # 距 T 這麼遠時開始四散
CRUISE_ALT = 400.0
R_FAN, HALF = 4000.0, 25.0     # 扇形(與 tot_node 一致)
APPROACH = 180.0
V_HALF_ANGLE = 35.0            # V 字半角(自機尾方向量起)


def mlon(lat):
    return EARTH * math.cos(math.radians(lat))


def dest(lat, lon, brg, d):
    b = math.radians(brg)
    return (lat + d * math.cos(b) / EARTH, lon + d * math.sin(b) / mlon(lat))


def dist_m(a, b):
    return math.hypot((b[0] - a[0]) * EARTH, (b[1] - a[1]) * mlon(a[0]))


def bearing(a, b):
    return math.degrees(math.atan2((b[1] - a[1]) * mlon(a[0]),
                                   (b[0] - a[0]) * EARTH)) % 360.0


def conn(i):
    c = mavutil.mavlink_connection(f"tcp:127.0.0.1:{BASE + STRIDE * (i - 1)}",
                                   source_system=190 + i, dialect="ardupilotmega")
    c.wait_heartbeat(timeout=10)
    return c


WP_LAG = 3.5      # GUIDED 航點有效滯後(s)。⚠️ 實測不要調大:曾把它從 3.5 調到 6.8
                  # (以為殘餘 206m 全是未補的滯後),結果誤差從 245m 惡化成 1288m 且
                  # 單調發散 —— 殘餘誤差不是純滯後,過度前饋會讓瞄準點跑到追不到的
                  # 前方、速度律持續飽和。3.5 是實測最佳。
V_MAX = 100.0
V_MIN = 48.0      # 噴射可持續下限(實測 ~45-75);低於此會掉高/失速
KP_ALONG = 0.45   # 縱向(沿航線)誤差 → 速度修正。只有縱向誤差該用速度修
JOIN_D = 4000.0   # 巡航前段留給「動態編隊集合」的距離,不列入整齊度評分


def goto(c, lat, lon, alt, v):
    c.mav.command_long_send(c.target_system, 1,
                            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
                            0, float(v), -1, 0, 0, 0, 0)
    c.mav.command_int_send(
        c.target_system, 1,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0, -1.0, 0.0, 0.0,
        float("nan"), int(lat * 1e7), int(lon * 1e7), float(alt))


_last_pos = {}    # id(conn) -> 最後一次成功讀到的位置


def pos_of(c, window=0.35):
    """讀最新位置。⚠️ 保留上一次結果:單一 tick 沒收到就回 None 的話,量測會變成
    「時有時無」,GATHER 甚至會在連線剛建立、位置流還沒進來時就把 worst 算成 0m
    而立刻判定靠攏完成(實測踩到)。改成快取最後已知值,量測才可信。"""
    last = None; t0 = time.time()
    while time.time() - t0 < window:
        m = c.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if m:
            last = m
        elif last:
            break                      # 已拿到本輪最新的就不必等滿
        else:
            time.sleep(0.01)
    if last:
        _last_pos[id(c)] = (last.lat / 1e7, last.lon / 1e7,
                            last.relative_alt / 1000.0)
    return _last_pos.get(id(c))


def prime(links, timeout=25.0):
    """等每條連線都至少收到一次位置,再開始量測/決策。"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        if all(pos_of(c, window=0.15) for c in links.values()):
            return True
        time.sleep(0.2)
    return False


def v_slot(lead, hdg, rank, spacing):
    """rank 0 = 長機;1..5 依序左右交錯後掠。回傳該 slot 的絕對座標。"""
    if rank == 0:
        return lead
    side = -1 if rank % 2 == 1 else 1          # 奇數在左、偶數在右
    tier = (rank + 1) // 2                      # 第幾層
    back = tier * spacing * math.cos(math.radians(V_HALF_ANGLE))
    lat_off = tier * spacing * math.sin(math.radians(V_HALF_ANGLE)) * side
    p = dest(lead[0], lead[1], (hdg + 180.0) % 360.0, back)     # 後掠
    return dest(p[0], p[1], (hdg + 90.0) % 360.0, lat_off)      # 側向


REC = "/tmp/show_track.csv"


def main():
    v_cruise = float(sys.argv[1]) if len(sys.argv) > 1 else 70.0
    spacing = float(sys.argv[2]) if len(sys.argv) > 2 else 300.0
    rec = open(REC, "w", newline="")
    rw = csv.writer(rec)
    rw.writerow(["t", "sysid", "lat", "lon", "alt", "phase"])
    t_rec0 = time.time()

    def snap(phase, links_):
        for k, cc in links_.items():
            p = pos_of(cc, window=0.02)
            if p:
                rw.writerow([round(time.time() - t_rec0, 1), k,
                             p[0], p[1], round(p[2], 1), phase])
        rec.flush()
    links = {i: conn(i) for i in range(1, N + 1)}
    for i, c in links.items():                              # 確保 GUIDED
        c.mav.set_mode_send(c.target_system,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 15)
    time.sleep(1.0)
    if not prime(links):
        print("[WARN] 位置流未全部就緒,量測可能不可信", flush=True)
    else:
        print("[PRIME] 六機位置流就緒", flush=True)

    order = sorted(links)                                    # UAV1 當長機
    # 起跑線:預設 START;RESUME=1 時改用「目前蜂群重心」當起點(不必飛回南邊重跑)
    start = START
    if len(sys.argv) > 3 and sys.argv[3] == "resume":
        pts = [pos_of(links[i]) for i in order]
        pts = [p for p in pts if p]
        if pts:
            start = (sum(p[0] for p in pts) / len(pts),
                     sum(p[1] for p in pts) / len(pts))
            print(f"[RESUME] 以目前蜂群重心為起跑線 {start}", flush=True)
    globals()["START"] = start
    hdg = bearing(start, T)                                  # 航線方位(南→北)

    # ── GATHER:粗略靠攏到起跑線附近(不要求精準)──
    # ⚠️ 不要用「靜止 slot」逼精準:固定翼無法停在一點,會繞著它盤旋。實測噴射在
    #    52 m/s、45° 傾角的迴轉半徑 ≈ v²/(g·tanφ) ≈ 276m,所以對「靜止 slot」量到的
    #    誤差永遠卡在 ~190m(實測 953→189m 後就不動了)。編隊要「在動態中編成」:
    #    slot 跟著長機一起前進,飛機就變成「跟在後面直線飛」而不是繞圈。
    # ⚠️ 逾時要由「實際距離」推算,不能寫死。曾用固定 90s,但蜂群當時距起跑線 18 km
    #    (65 m/s 要 ~280s),於是 GATHER 直接逾時、虛擬長機自顧自從起跑線衝出去,
    #    飛機一開始就落後 12 km,整段巡航再也追不上(評分誤差 2542m)。
    d0 = max((dist_m(p[:2], start) for p in
              (pos_of(links[i]) for i in order) if p), default=0.0)
    budget = max(120.0, d0 / max(v_cruise * 0.6, 1.0) + 90.0)
    print(f"[GATHER] 靠攏起跑線 {start} 航向 {hdg:.0f}° "
          f"(目前最遠 {d0/1000:.1f} km,預算 {budget:.0f}s)", flush=True)
    t0 = time.time()
    while time.time() - t0 < budget:
        worst = 0.0
        for r, i in enumerate(order):
            s = v_slot(start, hdg, r, spacing)
            p = pos_of(links[i])
            if p:
                worst = max(worst, dist_m(p[:2], s))
            goto(links[i], s[0], s[1], CRUISE_ALT, v_cruise)
        snap("GATHER", links)
        if worst < 700:
            print(f"[GATHER] 靠攏完成 @+{time.time()-t0:.0f}s worst={worst:.0f}m", flush=True)
            break
        if int(time.time() - t0) % 30 == 0:
            print(f"  gather t+{time.time()-t0:4.0f}s worst={worst:6.0f}m", flush=True)
        time.sleep(1.0)

    # ── CRUISE:虛擬長機沿航線北進,V 整體平移 ──
    # ⚠️ 若只把各機指到「以巡航速前進的 slot」且同樣命令巡航速,初始落後永遠補不回來
    #    (slot 跑多快、飛機就飛多快)。所以要兩件事:
    #    (a) 前饋:瞄準 WP_LAG 秒後的 slot 位置(抵銷 guided-WP 滯後)
    #    (b) 追上增益:命令速度 = 巡航 + KP·誤差,落後越多飛越快 → 誤差收斂
    total = dist_m(START, T) - DISPERSE_D
    join_d = min(JOIN_D, total * 0.45)      # 前段是「動態編隊集合」,不列入評分
    print(f"[CRUISE] 長程巡航 {total/1000:.1f} km @ {v_cruise:.0f} m/s "
          f"(前饋 {WP_LAG}s、縱向增益 {KP_ALONG}、集合段 {join_d/1000:.1f} km)", flush=True)
    graded = []                              # 集合段之後的誤差樣本(評分用)
    tc = time.time()
    while True:
        el = time.time() - tc
        s_along = min(total, v_cruise * el)
        s_ff = min(total, v_cruise * (el + WP_LAG))       # (a) 前饋後的長機位置
        lead = dest(START[0], START[1], hdg, s_along)
        lead_ff = dest(START[0], START[1], hdg, s_ff)
        errs = []
        for r, i in enumerate(order):
            s_now = v_slot(lead, hdg, r, spacing)          # 真實 slot(量誤差用)
            s_aim = v_slot(lead_ff, hdg, r, spacing)       # 前饋 slot(發送用)
            p = pos_of(links[i])
            if not p:
                continue
            err = dist_m(p[:2], s_now)
            errs.append(err)
            # 縱/橫分離:誤差沿航線方向的分量才用「速度」修,橫向分量由航點自己修。
            # 用純量距離去催速度會讓「只是側偏」的飛機無謂加速,反而衝過頭再回擺,
            # 陣形永遠在震盪 —— 這是舊版 417-567m 誤差的主因之一。
            brg_es = bearing(p[:2], s_now)                  # 由飛機指向 slot
            along = err * math.cos(math.radians(brg_es - hdg))   # >0 = 落後
            v_cmd = v_cruise + KP_ALONG * along
            v_cmd = max(V_MIN, min(V_MAX, v_cmd))
            goto(links[i], s_aim[0], s_aim[1], CRUISE_ALT, v_cmd)
        snap("CRUISE", links)
        if errs and s_along > join_d:
            graded.append((max(errs), sum(errs) / len(errs)))
        if int(el) % 20 == 0 and errs:
            tag = "評分" if s_along > join_d else "集合"
            print(f"  t+{el:5.0f}s [{tag}] 距T {dist_m(lead,T)/1000:5.2f}km "
                  f"slot誤差 max={max(errs):.0f}m avg={sum(errs)/len(errs):.0f}m",
                  flush=True)
        if s_along >= total:
            break
        time.sleep(1.0)
    if graded:
        gmax = sum(g[0] for g in graded) / len(graded)
        gavg = sum(g[1] for g in graded) / len(graded)
        gate = 0.5 * spacing
        print(f"[CRUISE] 抵達散開點。評分段({len(graded)}樣本): "
              f"持續 max={gmax:.0f}m avg={gavg:.0f}m  門檻={gate:.0f}m  "
              f"{'✅通過' if gmax <= gate else '❌未達'}", flush=True)
    else:
        print("[CRUISE] 抵達散開點(評分段無樣本)", flush=True)

    # ── SCATTER:飛向扇形弧上自己那格 ──
    slots = []
    for k in range(N):
        off = -HALF + 2 * HALF * k / (N - 1)
        slots.append(dest(T[0], T[1], (APPROACH + off) % 360.0, R_FAN))
    print("[SCATTER] 四散進扇形", flush=True)
    ts = time.time()
    while time.time() - ts < 140:
        done = 0
        for r, i in enumerate(order):
            goto(links[i], slots[r][0], slots[r][1], CRUISE_ALT, v_cruise)
            p = pos_of(links[i])
            if p and dist_m(p[:2], slots[r]) < 350:
                done += 1
        snap("SCATTER", links)
        if done == N:
            break
        time.sleep(1.0)
    print(f"[SCATTER] 完成 ({done}/{N} 就位)", flush=True)
    for i in order:
        p = pos_of(links[i])
        if p:
            print(f"  UAV{i} 距T={dist_m(p[:2], T):.0f}m alt={p[2]:.0f}m", flush=True)


if __name__ == "__main__":
    main()
