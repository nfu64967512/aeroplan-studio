#!/usr/bin/env python3
"""驗收航次駕駛器:對 6960+10i 的 mavproxy 分流下指令(與節點/AeroPlan 並存)。

子命令:
  goto SYSID LAT LON ALT_REL      # DO_REPOSITION(guided 點)
  goto_rel SYSID DN DE ALT_REL    # 以該機當前位置為基準,北/東位移 m
  mode SYSID MODE                 # GUIDED=15 RTL=11 AUTO=10(名稱或數字)
  setp SYSID NAME VALUE           # param set(RTL 分層高度用 RTL_ALTITUDE,公尺;
                                  #  舊名 ALT_HOLD_RTL 已於 post-4.4 移除、會被靜默忽略)
  home SYSID                      # 讀 HOME_POSITION
  land SYSID [APPROACH_M] [ALT]   # 上傳 2 航點降落任務(進場點+NAV_LAND@HOME)→ AUTO
  pos SYSID                       # 當前位置
"""
import sys, time, math
from pymavlink import mavutil

MODES = {"GUIDED": 15, "RTL": 11, "AUTO": 10, "TAKEOFF": 13, "LOITER": 12,
         "FBWA": 5, "MANUAL": 0}
EARTH = 111320.0


def conn(sysid):
    # 用 6760 家族(AeroPlan 分流;tcpin 多 client 可並存)——6960 對晚加入
    # 的 client 不推 telemetry(實測 no GLOBAL_POSITION_INT)。
    port = 6760 + 10 * (sysid - 1)
    c = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}", source_system=244,
                                   dialect="ardupilotmega")
    c.wait_heartbeat(timeout=6)
    return c


def get_pos(c):
    for _ in range(30):
        m = c.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if m:
            return m.lat / 1e7, m.lon / 1e7, m.relative_alt / 1000.0
    raise SystemExit("no GLOBAL_POSITION_INT")


def get_home(c):
    c.mav.command_long_send(c.target_system, 1,
                            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                            mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION,
                            0, 0, 0, 0, 0, 0)
    for _ in range(30):
        m = c.recv_match(type="HOME_POSITION", blocking=True, timeout=1)
        if m:
            return m.latitude / 1e7, m.longitude / 1e7
    raise SystemExit("no HOME_POSITION")


def do_goto(c, lat, lon, alt):
    c.mav.command_int_send(
        c.target_system, 1, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0,
        -1.0, 0.0, 0.0, float("nan"),
        int(lat * 1e7), int(lon * 1e7), float(alt))
    ack = c.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    print(f"goto ack={getattr(ack, 'result', None)}", flush=True)


def do_mode(c, mode):
    mid = MODES.get(str(mode).upper(), None)
    mid = int(mode) if mid is None and str(mode).isdigit() else mid
    c.mav.command_long_send(c.target_system, 1,
                            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            float(mid), 0, 0, 0, 0, 0)
    ack = c.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    print(f"mode {mode} ack={getattr(ack, 'result', None)}", flush=True)


def upload_land(c, approach_m=1200.0, app_alt=60.0):
    """2 航點降落任務:進場點(HOME 反航向 approach_m、app_alt)→ NAV_LAND@HOME。
    進場方位 = 跑道航向 52°(TAOYUAN 生成),由 HOME 往回推。"""
    hlat, hlon = get_home(c)
    hdg = 52.0
    back = math.radians(hdg + 180.0)
    alat = hlat + approach_m * math.cos(back) / EARTH
    alon = hlon + approach_m * math.sin(back) / (EARTH * math.cos(math.radians(hlat)))
    items = [
        # seq0: home 佔位(ArduPilot 慣例)
        (0, mavutil.mavlink.MAV_FRAME_GLOBAL, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
         hlat, hlon, 0.0),
        (1, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
         mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, alat, alon, app_alt),
        (2, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
         mavutil.mavlink.MAV_CMD_NAV_LAND, hlat, hlon, 0.0),
    ]
    tgt = c.target_system
    c.mav.mission_count_send(tgt, 1, len(items),
                             mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    sent, t0 = set(), time.time()
    while len(sent) < len(items) and time.time() - t0 < 20:
        req = c.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT"],
                           blocking=True, timeout=3)
        if not req:
            continue
        seq, frame, cmd, lat, lon, alt = items[req.seq]
        c.mav.mission_item_int_send(
            tgt, 1, seq, frame, cmd, 0, 1, 0, 0, 0, 0,
            int(lat * 1e7), int(lon * 1e7), alt,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        sent.add(req.seq)
    ack = c.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
    print(f"mission ack type={getattr(ack, 'type', None)} "
          f"(approach {approach_m:.0f} m @ {app_alt:.0f} m → LAND "
          f"{hlat:.6f},{hlon:.6f})", flush=True)
    return getattr(ack, "type", 255) == 0


def main():
    cmd, sysid = sys.argv[1], int(sys.argv[2])
    c = conn(sysid)
    if cmd == "goto":
        do_goto(c, float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5]))
    elif cmd == "goto_rel":
        lat, lon, _ = get_pos(c)
        dn, de, alt = float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])
        do_goto(c, lat + dn / EARTH,
                lon + de / (EARTH * math.cos(math.radians(lat))), alt)
    elif cmd == "mode":
        do_mode(c, sys.argv[3])
    elif cmd == "setp":
        c.mav.param_set_send(c.target_system, 1, sys.argv[3].encode(),
                             float(sys.argv[4]),
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.3)
        print(f"setp {sys.argv[3]}={sys.argv[4]}", flush=True)
    elif cmd == "home":
        print(get_home(c), flush=True)
    elif cmd == "pos":
        print(get_pos(c), flush=True)
    elif cmd == "land":
        am = float(sys.argv[3]) if len(sys.argv) > 3 else 1200.0
        aa = float(sys.argv[4]) if len(sys.argv) > 4 else 60.0
        if upload_land(c, am, aa):
            do_mode(c, "AUTO")
    else:
        raise SystemExit("unknown cmd")


if __name__ == "__main__":
    main()
