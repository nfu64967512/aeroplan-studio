#!/usr/bin/env python3
"""scatter.py — 把 6 機四散到目標 T 前方 ~2-2.7km(異距,製造 weave/直線混合),
供 tot_node 終端俯衝 demo 的初始態。走 6760 分流,DO_REPOSITION。"""
import sys, math, time
from pymavlink import mavutil

EARTH = 111320.0
T = (25.1406, 121.2325)          # 目標
# (南向距離 m, 東西橫移 m) — 異距,製造 near/far 混合
OFF = {1: (2000, -600), 2: (2700, -300), 3: (2200, 0),
       4: (2500, 300), 5: (2100, 600), 6: (2400, -900)}
ALT = 90.0


def mlon(lat):
    return EARTH * math.cos(math.radians(lat))


def conn(i):
    c = mavutil.mavlink_connection(f"tcp:127.0.0.1:{6760 + 10 * (i - 1)}",
                                   source_system=234, dialect="ardupilotmega")
    c.wait_heartbeat(timeout=8)
    return c


def goto(c, i, lat, lon, alt):
    c.mav.command_long_send(c.target_system or i, 1,
                            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
                            0, 28.0, -1, 0, 0, 0, 0)
    c.mav.command_int_send(
        c.target_system or i, 1,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0, -1.0, 0.0, 0.0,
        float("nan"), int(lat * 1e7), int(lon * 1e7), float(alt))


def main():
    hold = float(sys.argv[1]) if len(sys.argv) > 1 else 70.0
    links = {i: conn(i) for i in range(1, 7)}
    tgt = {}
    for i, (dn, de) in OFF.items():
        lat = T[0] - dn / EARTH               # 南 = 減 lat
        lon = T[1] + de / mlon(T[0])
        tgt[i] = (lat, lon)
    t0 = time.time()
    while time.time() - t0 < hold:
        for i, c in links.items():
            goto(c, i, tgt[i][0], tgt[i][1], ALT)
        time.sleep(1.0)
    # 印最終位置
    print("--- 四散後 ---", flush=True)
    for i, c in links.items():
        m = None
        for _ in range(60):
            m = c.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
            if m:
                break
        if m:
            d = math.hypot((T[0] - m.lat / 1e7) * EARTH,
                           (T[1] - m.lon / 1e7) * mlon(T[0]))
            print(f"UAV{i}: ({m.lat/1e7:.6f}, {m.lon/1e7:.7f}, "
                  f"{m.relative_alt/1000:.0f}m) 距T={d:.0f}m", flush=True)


if __name__ == "__main__":
    main()
