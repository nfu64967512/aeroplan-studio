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
import sys, math, time
from pymavlink import mavutil

EARTH = 111320.0
BASE, STRIDE, N = 6960, 10, 6

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


WP_LAG = 3.5      # GUIDED 航點有效滯後(s)。不前饋的話穩態誤差 ≈ v·lag(70×3.5≈245m)
KP_CLOSE = 0.6    # 追上增益:指令速度 = 巡航 + KP·誤差(上限 V_MAX)
V_MAX = 100.0


def goto(c, lat, lon, alt, v):
    c.mav.command_long_send(c.target_system, 1,
                            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
                            0, float(v), -1, 0, 0, 0, 0)
    c.mav.command_int_send(
        c.target_system, 1,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0, -1.0, 0.0, 0.0,
        float("nan"), int(lat * 1e7), int(lon * 1e7), float(alt))


def pos_of(c):
    last = None; t0 = time.time()
    while time.time() - t0 < 0.5:
        m = c.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if m:
            last = m
        else:
            time.sleep(0.01)
    return (last.lat / 1e7, last.lon / 1e7, last.relative_alt / 1000.0) if last else None


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


def main():
    v_cruise = float(sys.argv[1]) if len(sys.argv) > 1 else 70.0
    spacing = float(sys.argv[2]) if len(sys.argv) > 2 else 300.0
    links = {i: conn(i) for i in range(1, N + 1)}
    for i, c in links.items():                              # 確保 GUIDED
        c.mav.set_mode_send(c.target_system,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 15)
    time.sleep(1.0)

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

    # ── FORMUP:飛到起跑線的 V slot ──
    print(f"[FORMUP] 起跑線 {start} 航向 {hdg:.0f}° V間距 {spacing:.0f}m", flush=True)
    t0 = time.time()
    while time.time() - t0 < 150:
        allin = True
        for r, i in enumerate(order):
            s = v_slot(start, hdg, r, spacing)
            goto(links[i], s[0], s[1], CRUISE_ALT, v_cruise)
            p = pos_of(links[i])
            if p and dist_m(p[:2], s) > 400:
                allin = False
        if allin:
            print(f"[FORMUP] 全員就位 @+{time.time()-t0:.0f}s", flush=True)
            break
        time.sleep(1.0)

    # ── CRUISE:虛擬長機沿航線北進,V 整體平移 ──
    # ⚠️ 若只把各機指到「以巡航速前進的 slot」且同樣命令巡航速,初始落後永遠補不回來
    #    (slot 跑多快、飛機就飛多快)。所以要兩件事:
    #    (a) 前饋:瞄準 WP_LAG 秒後的 slot 位置(抵銷 guided-WP 滯後)
    #    (b) 追上增益:命令速度 = 巡航 + KP·誤差,落後越多飛越快 → 誤差收斂
    total = dist_m(START, T) - DISPERSE_D
    print(f"[CRUISE] 長程巡航 {total/1000:.1f} km @ {v_cruise:.0f} m/s "
          f"(前饋 {WP_LAG}s + 追上增益 {KP_CLOSE})", flush=True)
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
            err = dist_m(p[:2], s_now) if p else 0.0
            errs.append(err)
            v_cmd = min(V_MAX, v_cruise + KP_CLOSE * err)  # (b) 落後就加速
            goto(links[i], s_aim[0], s_aim[1], CRUISE_ALT, v_cmd)
        if int(el) % 20 == 0 and errs:
            print(f"  t+{el:5.0f}s 距T {dist_m(lead,T)/1000:5.2f}km "
                  f"slot誤差 max={max(errs):.0f}m avg={sum(errs)/len(errs):.0f}m",
                  flush=True)
        if s_along >= total:
            break
        time.sleep(1.0)
    print("[CRUISE] 抵達散開點", flush=True)

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
