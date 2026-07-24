#!/usr/bin/env python3
"""jet_prep.py — plane-jet 蜂群:起飛 → 爬升 → GUIDED → 四散到噴射尺度扇形前緣。
走 6760 分流。用法: python3 jet_prep.py [climb_wait_s]"""
import sys, math, time
from pymavlink import mavutil

EARTH = 111320.0
T = (25.1406, 121.2325)
# 噴射尺度四散(南距 m, 東西 m):3.5-4.5km、橫向 ±1.3km(異距混合)
OFF = {1: (3600, -800), 2: (4400, -400), 3: (3800, 0),
       4: (4200, 500), 5: (3700, 1000), 6: (4300, -1300)}
CRUISE_ALT = 130.0
TAKEOFF_MODE = 13  # ArduPlane TAKEOFF


def mlon(lat):
    return EARTH * math.cos(math.radians(lat))


def conn(i):
    c = mavutil.mavlink_connection(f"tcp:127.0.0.1:{6760 + 10 * (i - 1)}",
                                   source_system=250, dialect="ardupilotmega")
    c.wait_heartbeat(timeout=8)
    return c


def set_mode(c, i, mode):
    c.mav.set_mode_send(c.target_system or i,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode)


def arm(c, i):
    c.mav.command_long_send(c.target_system or i, 1,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                            1, 0, 0, 0, 0, 0, 0)


def goto(c, i, lat, lon, alt, v):
    c.mav.command_long_send(c.target_system or i, 1,
                            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
                            0, float(v), -1, 0, 0, 0, 0)
    c.mav.command_int_send(
        c.target_system or i, 1,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0, -1.0, 0.0, 0.0,
        float("nan"), int(lat * 1e7), int(lon * 1e7), float(alt))


def alt_of(c):
    last = None; t0 = time.time()
    while time.time() - t0 < 1.5:
        m = c.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if m:
            last = m
        else:
            time.sleep(0.02)
    return last.relative_alt / 1000.0 if last else 0.0


def main():
    climb_wait = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
    links = {i: conn(i) for i in range(1, 7)}
    # 1) 起飛:TAKEOFF 模式 + ARM,錯開 4s(跑道縱列防撞)
    for i in range(1, 7):
        set_mode(links[i], i, TAKEOFF_MODE); time.sleep(0.4)
        arm(links[i], i)
        print(f"UAV{i} TAKEOFF+ARM", flush=True)
        time.sleep(4.0)
    # 2) 等爬升
    t0 = time.time()
    while time.time() - t0 < climb_wait:
        alts = {i: round(alt_of(links[i])) for i in links}
        print(f"climb… alts={alts}", flush=True)
        if all(a > 70 for a in alts.values()):
            print("全員 >70m", flush=True); break
        time.sleep(3)
    # 3) GUIDED + 飛去四散點(爬到 CRUISE_ALT)
    tgt = {}
    for i, (dn, de) in OFF.items():
        tgt[i] = (T[0] - dn / EARTH, T[1] + de / mlon(T[0]))
    for i in links:
        set_mode(links[i], i, 15)  # ArduPlane GUIDED=15
    time.sleep(1.0)
    hold = 95.0
    t0 = time.time()
    while time.time() - t0 < hold:
        for i, c in links.items():
            goto(c, i, tgt[i][0], tgt[i][1], CRUISE_ALT, 70.0)
        time.sleep(1.0)
    print("--- 四散完成(讀最終) ---", flush=True)
    for i, c in links.items():
        last = None; tt = time.time()
        while time.time() - tt < 2.0:
            m = c.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
            if m:
                last = m
            else:
                time.sleep(0.02)
        if last:
            la, lo = last.lat / 1e7, last.lon / 1e7
            d = math.hypot((T[0] - la) * EARTH, (T[1] - lo) * mlon(la))
            print(f"UAV{i}: dT={d:.0f}m alt={last.relative_alt/1000:.0f}m", flush=True)


if __name__ == "__main__":
    main()
