#!/usr/bin/env python3
"""jet_rescatter.py — 已在空中的 plane-jet 蜂群重新四散(GUIDED+goto,不重起飛)。
用法: python3 jet_rescatter.py [hold_s]"""
import sys, math, time
from pymavlink import mavutil

EARTH = 111320.0
T = (25.1406, 121.2325)
OFF = {1: (3600, -800), 2: (4400, -400), 3: (3800, 0),
       4: (4200, 500), 5: (3700, 1000), 6: (4300, -1300)}
CRUISE_ALT = 130.0


def mlon(lat):
    return EARTH * math.cos(math.radians(lat))


def conn(i):
    c = mavutil.mavlink_connection(f"tcp:127.0.0.1:{6760 + 10 * (i - 1)}",
                                   source_system=251, dialect="ardupilotmega")
    c.wait_heartbeat(timeout=8)
    return c


def goto(c, i, lat, lon, alt, v):
    c.mav.command_long_send(c.target_system or i, 1,
                            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
                            0, float(v), -1, 0, 0, 0, 0)
    c.mav.command_int_send(
        c.target_system or i, 1,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0, -1.0, 0.0, 0.0,
        float("nan"), int(lat * 1e7), int(lon * 1e7), float(alt))


def main():
    hold = float(sys.argv[1]) if len(sys.argv) > 1 else 100.0
    links = {i: conn(i) for i in range(1, 7)}
    for i in links:
        links[i].mav.set_mode_send(links[i].target_system or i,
                                   mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 15)
    time.sleep(1.0)
    tgt = {i: (T[0] - dn / EARTH, T[1] + de / mlon(T[0])) for i, (dn, de) in OFF.items()}
    t0 = time.time()
    while time.time() - t0 < hold:
        for i, c in links.items():
            goto(c, i, tgt[i][0], tgt[i][1], CRUISE_ALT, 70.0)
        time.sleep(1.0)
    print("--- rescatter 完成(讀最終) ---", flush=True)
    ok = 0
    for i, c in links.items():
        last = None; tt = time.time()
        while time.time() - tt < 2.0:
            m = c.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
            if m:
                last = m
            else:
                time.sleep(0.02)
        la, lo = last.lat / 1e7, last.lon / 1e7
        d = math.hypot((T[0] - la) * EARTH, (T[1] - lo) * mlon(la))
        if d > 3000:
            ok += 1
        print(f"UAV{i}: dT={d:.0f}m alt={last.relative_alt/1000:.0f}m", flush=True)
    print(f"scattered_ok={ok}/6", flush=True)


if __name__ == "__main__":
    main()
