#!/usr/bin/env python3
"""jet_trace.py — 彈射 → 轉 GUIDED → 平飛,逐秒記錄姿態/油門/速度。

用途:高速改裝之後飛機在轉 GUIDED 的瞬間掉下來,而點質量算式說推力綽綽有餘
(80 m/s 平飛只需 21.5 N,可用 58.6 N)。所以問題在控制暫態,要看時間序列才知道
是俯仰指令、油門、還是空速估計出的問題。

刻意**只下一次**航點:每 tick 重發 DO_REPOSITION 會走 set_target_altitude_current()
把高度目標重錨在當下高度(這是本專案 d7a377a 修過的坑),會污染診斷。
"""
import argparse
import math
import time

from pymavlink import mavutil

TAKEOFF_MODE = 13
GUIDED_MODE = 15
EARTH = 111320.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--link", type=int, default=6960)
    ap.add_argument("--secs", type=float, default=90.0)
    ap.add_argument("--speed", type=float, default=80.0)
    ap.add_argument("--climb", type=float, default=0.0,
                    help="轉 GUIDED 後要求的爬升量(m);0 = 維持當前高度")
    a = ap.parse_args()

    c = mavutil.mavlink_connection(f"tcp:127.0.0.1:{a.link}", source_system=202,
                                   dialect="ardupilotmega")
    c.wait_heartbeat(timeout=15)
    c.mav.request_data_stream_send(c.target_system, 1,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    def setp(name, val):
        c.mav.param_set_send(c.target_system, 1, name.encode(), float(val),
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    st = {}

    def pump(sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            m = c.recv_match(blocking=False)
            if m is None:
                time.sleep(0.01)
                continue
            t = m.get_type()
            if t == "STATUSTEXT":
                print("    [FC] %s" % m.text, flush=True)
            elif t in ("GLOBAL_POSITION_INT", "VFR_HUD", "ATTITUDE", "HEARTBEAT",
                       "NAV_CONTROLLER_OUTPUT"):
                st[t] = m

    setp("SERVO7_FUNCTION", 0)
    pump(2.0)
    for k in range(20):
        c.mav.set_mode_send(c.target_system,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            TAKEOFF_MODE)
        pump(0.5)
        c.mav.command_long_send(c.target_system, 1,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                1, 0, 0, 0, 0, 0, 0)
        pump(3.0)
        hb = st.get("HEARTBEAT")
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("已解鎖(第 %d 次)" % (k + 1), flush=True)
            break
    else:
        raise SystemExit("解不了鎖")

    t0 = time.time()
    while time.time() - t0 < 3.0:
        c.mav.command_long_send(c.target_system, 1,
                                mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                                7, 2000, 0, 0, 0, 0, 0)
        pump(0.2)
    c.mav.command_long_send(c.target_system, 1,
                            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                            7, 1000, 0, 0, 0, 0, 0)

    # 等爬到 150 m 再交接,別在剛離地就換模式
    t0 = time.time()
    while time.time() - t0 < 120.0:
        pump(1.0)
        g = st.get("GLOBAL_POSITION_INT")
        if g and g.relative_alt / 1000.0 > 150.0:
            break
    g = st.get("GLOBAL_POSITION_INT")
    alt0 = g.relative_alt / 1000.0 if g else 0.0
    lat0, lon0 = g.lat / 1e7, g.lon / 1e7
    print("轉 GUIDED @ alt=%.0f m" % alt0, flush=True)

    c.mav.set_mode_send(c.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        GUIDED_MODE)
    pump(0.5)
    c.mav.command_long_send(c.target_system, 1,
                            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0,
                            a.speed, -1, 0, 0, 0, 0)
    # 只送一次航點(見檔頭說明)
    tgt = (lat0 + 60000.0 / EARTH, lon0)
    c.mav.command_int_send(
        c.target_system, 1,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0, -1.0, 0.0, 0.0,
        float("nan"), int(tgt[0] * 1e7), int(tgt[1] * 1e7),
        float(alt0 + a.climb))

    print("%6s %8s %8s %8s %8s %8s %8s %8s" %
          ("t", "alt", "gs", "airspd", "pitch°", "thr%", "climb", "alt_err"))
    t0 = time.time()
    while time.time() - t0 < a.secs:
        pump(1.0)
        g = st.get("GLOBAL_POSITION_INT")
        v = st.get("VFR_HUD")
        at = st.get("ATTITUDE")
        nc = st.get("NAV_CONTROLLER_OUTPUT")
        if not (g and v and at):
            continue
        gs = math.hypot(g.vx, g.vy) / 100.0
        print("%6.0f %8.1f %8.1f %8.1f %8.1f %8d %8.1f %8s" %
              (time.time() - t0, g.relative_alt / 1000.0, gs, v.airspeed,
               math.degrees(at.pitch), v.throttle, v.climb,
               "%.0f" % nc.alt_error if nc else "-"), flush=True)


if __name__ == "__main__":
    main()
