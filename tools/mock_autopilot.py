"""mock_autopilot.py — 以 pymavlink 模擬一台 ArduPilot QuadPlane 載具的最小協定行為。

用途：在沒有 ArduPilot SITL 的環境下，驗證 HV-DCCPP 的 MAVLink 整合層
（任務 / 圍籬 / 集結點上傳、參數設定、遙測讀取、動態重規劃）。

支援的協定：
- HEARTBEAT（1 Hz，MAV_TYPE_FIXED_WING + MAV_AUTOPILOT_ARDUPILOTMEGA，QuadPlane 韌體的回報方式）
- 任務上傳：MISSION_COUNT → MISSION_REQUEST_INT × n → MISSION_ITEM_INT × n → MISSION_ACK
  （mission_type 0=任務、1=圍籬、2=集結點皆支援；也接受舊式 MISSION_ITEM）
- 任務下載：MISSION_REQUEST_LIST → MISSION_COUNT，MISSION_REQUEST_INT → MISSION_ITEM_INT
- MISSION_CLEAR_ALL、MISSION_SET_CURRENT → MISSION_CURRENT
- PARAM_SET → PARAM_VALUE；PARAM_REQUEST_LIST / PARAM_REQUEST_READ
- COMMAND_LONG：MAV_CMD_COMPONENT_ARM_DISARM、MAV_CMD_MISSION_START、MAV_CMD_DO_SET_MODE、
  MAV_CMD_REQUEST_MESSAGE → COMMAND_ACK；SET_MODE
- 簡化飛行模型：AUTO 模式下沿 NAV 航點以 AIRSPEED_CRUISE 直線飛行（不含轉彎動力學），
  發送 GLOBAL_POSITION_INT、MISSION_CURRENT、BATTERY_STATUS、SYS_STATUS、WIND、EXTENDED_SYS_STATE、
  MISSION_ITEM_REACHED；NAV_DELAY(93) 依 param1 在原地等待（配合 ardupilot.export_all 之
  ``takeoff_delay='nav_delay'``）。

電量模型（`--battery-wh` / `--cruise-power-w` / `--hover-power-w` / `--reserve-frac` / `--energy-profile`）
    預設是「巡航功率 × 時間」線性下降，垂直（VTOL）段改用懸停功率。**預設值（800 Wh、350 W）與任何實際平台
    都不一致**，會讓 `mavlink_link.detect_events` 一直看到能量偏差；要讓閉迴路示範誠實，請用該機的實際值：

        --battery-wh 1350 --cruise-power-w 330 --hover-power-w 2600 --reserve-frac 0.30

    或直接餵計畫的能量剖面（最誠實，遙測會完全貼合計畫）：

        --energy-profile A1_energy.json        # {"t_s": [...], "e_wh": [...]} 或 [[t_s, e_wh], ...]
                                               # 亦接受 UAVPlan/MissionPlan JSON（waypoints 之 t_s / energy_remaining_wh）
        --energy-error-frac 0.1                # 蓄意讓實際耗電比計畫多 10%（測試 ENERGY_UPDATE 用）

    產生剖面：
        python -c "import json;from hv_dccpp import*;from hv_dccpp.scenario import build_mini_example;
        i,c=build_mini_example();p=plan_mission(i,c);u=p.plans['A1'];
        json.dump({'t_s':[w.t_s for w in u.waypoints],'e_wh':[w.energy_remaining_wh for w in u.waypoints]},
        open('A1_energy.json','w'))"

用法：
    python tools/mock_autopilot.py --listen udpin:127.0.0.1:14550 --home 23.9,120.7,100,0 \
        --speed 21.5 --time-scale 20 --dump received_mission.json

    然後 HV-DCCPP 的整合層以 'udpout:127.0.0.1:14550' 連上來。

本檔只依賴 pymavlink（pip install pymavlink）。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import socket
import sys
import time

# ArduPilot 使用 MAVLink 2；mission_type 等欄位是 MAVLink 2 的擴充欄位，必須在 import 前設定。
os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402

mav = mavutil.mavlink

EARTH_R = 6371000.0


def _dist_m(lat1, lon1, lat2, lon2):
    """簡易等距投影距離（公尺），適用於短距離。"""
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1) * math.cos(math.radians(0.5 * (lat1 + lat2)))
    return EARTH_R * math.hypot(dlat, dlon)


def _disable_windows_udp_reset(sock):
    """Windows 的 UDP socket 在對端回 ICMP port unreachable 後，recvfrom 會拋 WinError 10054；關掉它。"""
    if sock is None or not hasattr(socket, "SIO_UDP_CONNRESET"):
        return
    try:
        sock.ioctl(socket.SIO_UDP_CONNRESET, False)
    except OSError:
        pass


def load_energy_profile(path):
    """讀計畫能量剖面 → (t_s ndarray-like list, e_wh list)。

    接受 {"t_s": [...], "e_wh": [...]}、[[t_s, e_wh], ...]，或含 waypoints（t_s／energy_remaining_wh）之
    UAVPlan/MissionPlan JSON（MissionPlan 取第一架，或 {"plans": {...}} 之單機）。
    """
    with open(path, "r", encoding="utf-8") as f:
        d = json.load(f)
    if isinstance(d, dict) and "plans" in d and isinstance(d["plans"], dict) and d["plans"]:
        d = next(iter(d["plans"].values()))
    if isinstance(d, dict) and "waypoints" in d:
        wps = d["waypoints"]
        ts = [float(w["t_s"]) for w in wps]
        es = [float(w.get("energy_remaining_wh", w.get("e_wh", 0.0))) for w in wps]
    elif isinstance(d, dict) and "t_s" in d:
        ts = [float(x) for x in d["t_s"]]
        es = [float(x) for x in (d.get("e_wh") or d["energy_remaining_wh"])]
    elif isinstance(d, list):
        ts = [float(r[0]) for r in d]
        es = [float(r[1]) for r in d]
    else:
        raise ValueError(f"unrecognised energy profile: {path}")
    if len(ts) != len(es) or len(ts) < 2:
        raise ValueError(f"energy profile needs >= 2 aligned samples: {path}")
    return ts, es


def _interp(x, xs, ys):
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    lo, hi = 0, len(xs) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if xs[mid] <= x:
            lo = mid
        else:
            hi = mid
    span = xs[hi] - xs[lo]
    f = 0.0 if span <= 0 else (x - xs[lo]) / span
    return ys[lo] + f * (ys[hi] - ys[lo])


NAV_CMDS = {
    mav.MAV_CMD_NAV_WAYPOINT,
    mav.MAV_CMD_NAV_DELAY,
    mav.MAV_CMD_NAV_VTOL_TAKEOFF,
    mav.MAV_CMD_NAV_VTOL_LAND,
    mav.MAV_CMD_NAV_LOITER_TO_ALT,
    mav.MAV_CMD_NAV_LOITER_TURNS,
    mav.MAV_CMD_NAV_LOITER_TIME,
    mav.MAV_CMD_NAV_RETURN_TO_LAUNCH,
    mav.MAV_CMD_NAV_TAKEOFF,
    mav.MAV_CMD_NAV_LAND,
}

# ArduPlane 模式編號（custom_mode）
PLANE_MODES = {"MANUAL": 0, "CIRCLE": 1, "STABILIZE": 2, "FBWA": 5, "AUTO": 10, "RTL": 11,
               "LOITER": 12, "GUIDED": 15, "QSTABILIZE": 17, "QHOVER": 18, "QLOITER": 19,
               "QLAND": 20, "QRTL": 21}


class MockAutopilot:
    def __init__(self, listen: str, home, speed: float, time_scale: float, dump: str | None,
                 sysid: int = 1, battery_wh: float = 800.0, cruise_power_w: float = 350.0,
                 wind=(0.0, 0.0), verbose: bool = True, hover_power_w: float | None = None,
                 reserve_frac: float = 0.0, energy_profile=None, energy_error_frac: float = 0.0):
        self.conn = mavutil.mavlink_connection(listen, source_system=sysid,
                                               source_component=mav.MAV_COMP_ID_AUTOPILOT1)
        _disable_windows_udp_reset(getattr(self.conn, "port", None))
        self.home = list(home)  # lat, lon, alt_msl, yaw
        self.lat, self.lon, self.alt = home[0], home[1], home[2]
        self.speed = speed
        self.time_scale = time_scale
        self.dump = dump
        self.verbose = verbose
        self.missions = {0: [], 1: [], 2: []}  # mission_type → list of dict
        self.pending = None  # (mission_type, count, next_seq, items)
        self.params = {"AIRSPEED_CRUISE": speed, "WP_RADIUS": 90.0, "WP_LOITER_RAD": 200.0,
                       "Q_ENABLE": 1.0, "FENCE_ENABLE": 0.0, "BATT_CAPACITY": 20000.0}
        self.mode = PLANE_MODES["QLOITER"]
        self.armed = False
        self.current_seq = 0
        self.vtol_state = mav.MAV_VTOL_STATE_MC
        self.landed_state = mav.MAV_LANDED_STATE_ON_GROUND
        self.battery_wh = battery_wh
        self.battery_wh_full = battery_wh
        self.cruise_power_w = cruise_power_w
        # 懸停功率預設 = 8 × 巡航（四旋翼 VTOL 段之量級）；--hover-power-w 覆寫
        self.hover_power_w = float(hover_power_w) if hover_power_w is not None else 8.0 * cruise_power_w
        self.reserve_frac = max(0.0, float(reserve_frac))
        self.reserve_wh = self.reserve_frac * battery_wh
        self.energy_profile = energy_profile          # (t_s[], e_wh[]) 或 None
        self.energy_error_frac = float(energy_error_frac)
        if energy_profile is not None:                # 起飛時之剩餘能量（可低於標稱）
            self.battery_wh = min(battery_wh, float(energy_profile[1][0]))
        self._reserve_warned = False
        self._t_arm = None                            # 進入 AUTO 之模擬時刻（能量剖面之時間原點）
        self._delay_until = None                      # NAV_DELAY 之結束模擬時刻
        self.wind = wind  # (speed m/s, direction deg from which it blows)
        self.gcs_sys = None
        self.gcs_comp = None
        self._last_hb = 0.0
        self._last_pos = 0.0
        self._last_slow = 0.0
        self._sim_t = 0.0
        self.trace = []  # (t, lat, lon, alt, seq)

    # ------------------------------------------------------------------ helpers
    def log(self, *a):
        if self.verbose:
            print("[mock]", *a, flush=True)

    def _target(self, msg):
        # 回覆給發訊者
        return msg.get_srcSystem(), msg.get_srcComponent()

    # ------------------------------------------------------------------ periodic
    def send_heartbeat(self):
        base_mode = mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (mav.MAV_MODE_FLAG_SAFETY_ARMED if self.armed else 0)
        self.conn.mav.heartbeat_send(mav.MAV_TYPE_FIXED_WING, mav.MAV_AUTOPILOT_ARDUPILOTMEGA,
                                     base_mode, self.mode, mav.MAV_STATE_ACTIVE if self.armed else mav.MAV_STATE_STANDBY)

    def send_position(self):
        rel = int((self.alt - self.home[2]) * 1000)
        self.conn.mav.global_position_int_send(int(self._sim_t * 1000), int(self.lat * 1e7), int(self.lon * 1e7),
                                               int(self.alt * 1000), rel, 0, 0, 0, 0)
        self.conn.mav.mission_current_send(self.current_seq)

    def send_slow(self):
        pct = max(0, min(100, int(100 * self.battery_wh / self.battery_wh_full)))
        self.conn.mav.sys_status_send(0, 0, 0, 500, 22000, 15000, pct, 0, 0, 0, 0, 0, 0)
        # energy_consumed 之單位是 hJ（100 J）：1 Wh = 36 hJ，且是「已消耗」而非「剩餘」
        used_hj = int(max(0.0, self.battery_wh_full - self.battery_wh) * 36.0)
        self.conn.mav.battery_status_send(0, mav.MAV_BATTERY_FUNCTION_ALL, mav.MAV_BATTERY_TYPE_LIPO, 25,
                                          [22000] + [65535] * 9, 15000, -1, used_hj, pct)
        self.conn.mav.wind_send(self.wind[1], self.wind[0], 0.0)
        self.conn.mav.extended_sys_state_send(self.vtol_state, self.landed_state)

    # ------------------------------------------------------------------ energy model
    def _burn(self, power_w: float, dt: float):
        """扣電量：有 `--energy-profile` 時直接照計畫剖面（× (1+error)），否則 power × dt。"""
        if self.energy_profile is not None:
            ts, es = self.energy_profile
            t0 = self._t_arm if self._t_arm is not None else 0.0
            e = _interp(self._sim_t - t0 + ts[0], ts, es)
            used = max(0.0, es[0] - e) * (1.0 + self.energy_error_frac)
            # 剖面之起點 es[0] 是「起飛時的剩餘能量」，未必等於標稱 battery_wh_full
            self.battery_wh = max(0.0, es[0] - used)
        else:
            self.battery_wh -= power_w * (1.0 + self.energy_error_frac) * dt / 3600.0
        if self.reserve_wh > 0.0 and self.battery_wh <= self.reserve_wh and not self._reserve_warned:
            self._reserve_warned = True
            self.log(f"battery {self.battery_wh:.0f} Wh at/below reserve "
                     f"{self.reserve_wh:.0f} Wh ({self.reserve_frac:.0%} of {self.battery_wh_full:.0f} Wh)")

    # ------------------------------------------------------------------ flight model
    def step_flight(self, dt: float):
        if not self.armed or self.mode != PLANE_MODES["AUTO"]:
            return
        if self._t_arm is None:
            self._t_arm = self._sim_t
        items = self.missions[0]
        nav = [it for it in items if it["command"] in NAV_CMDS]
        if not nav:
            return
        # 找目前 seq 對應的 NAV 項目（跳過 DO_ 指令）
        while self.current_seq < len(items) and items[self.current_seq]["command"] not in NAV_CMDS:
            self.log("DO cmd", items[self.current_seq]["command"], "at seq", self.current_seq)
            self.conn.mav.mission_item_reached_send(self.current_seq)
            self.current_seq += 1
        if self.current_seq >= len(items):
            self.log("mission complete → QLAND")
            self.mode = PLANE_MODES["QLAND"]
            self.landed_state = mav.MAV_LANDED_STATE_ON_GROUND
            self.armed = False
            return
        it = items[self.current_seq]
        cmd = it["command"]
        if cmd == mav.MAV_CMD_NAV_DELAY:
            # param1 = 秒（< 0 表「當日時刻」形式，本模擬不支援 → 視為 0）
            if self._delay_until is None:
                self._delay_until = self._sim_t + max(0.0, float(it["param1"]))
                self.log(f"NAV_DELAY {max(0.0, float(it['param1'])):.0f} s at seq {self.current_seq} "
                         f"(sim t={self._sim_t:.0f} s)")
            self._burn(self.hover_power_w if self.landed_state != mav.MAV_LANDED_STATE_ON_GROUND else 0.0, dt)
            if self._sim_t >= self._delay_until:
                self._delay_until = None
                self._reached()
            return
        tlat, tlon = it["x"], it["y"]
        if tlat == 0 and tlon == 0:  # 0,0 = 當前位置（VTOL_TAKEOFF / LAND 常見）
            tlat, tlon = self.lat, self.lon
        talt = self.home[2] + it["z"] if it["frame"] == mav.MAV_FRAME_GLOBAL_RELATIVE_ALT else it["z"]
        if cmd == mav.MAV_CMD_NAV_VTOL_TAKEOFF:
            self.vtol_state = mav.MAV_VTOL_STATE_MC
            self.landed_state = mav.MAV_LANDED_STATE_TAKEOFF
            self._burn(self.hover_power_w, dt)
            self.alt = min(talt, self.alt + 2.5 * dt)  # 2.5 m/s 垂直爬升
            if self.alt >= talt - 0.5:
                self._reached()
                self.vtol_state = mav.MAV_VTOL_STATE_TRANSITION_TO_FW
            return
        if cmd == mav.MAV_CMD_NAV_VTOL_LAND:
            d = _dist_m(self.lat, self.lon, tlat, tlon)
            if d > 5:
                self._move_towards(tlat, tlon, talt if talt > self.home[2] else self.alt, dt)
            else:
                self.vtol_state = mav.MAV_VTOL_STATE_MC
                self.landed_state = mav.MAV_LANDED_STATE_LANDING
                self._burn(self.hover_power_w, dt)
                self.alt = max(self.home[2], self.alt - 1.5 * dt)
                if self.alt <= self.home[2] + 0.2:
                    self._reached()
            return
        # 一般航點 / 盤旋到高度 / RTL
        if cmd == mav.MAV_CMD_NAV_RETURN_TO_LAUNCH:
            tlat, tlon, talt = self.home[0], self.home[1], self.alt
        self.vtol_state = mav.MAV_VTOL_STATE_FW
        self.landed_state = mav.MAV_LANDED_STATE_IN_AIR
        self._move_towards(tlat, tlon, talt, dt)
        acc = it["param2"] if it["param2"] > 0 else self.params.get("WP_RADIUS", 90.0)
        if cmd == mav.MAV_CMD_NAV_LOITER_TO_ALT:
            if abs(self.alt - talt) < 1.0:
                self._reached()
        elif _dist_m(self.lat, self.lon, tlat, tlon) <= acc:
            self._reached()

    def _move_towards(self, tlat, tlon, talt, dt):
        d = _dist_m(self.lat, self.lon, tlat, tlon)
        step = self.params.get("AIRSPEED_CRUISE", self.speed) * dt
        if d > 1e-6:
            f = min(1.0, step / d)
            self.lat += (tlat - self.lat) * f
            self.lon += (tlon - self.lon) * f
        # 高度：受 3° 爬升 / 4° 下降限制
        max_up = step * math.tan(math.radians(3.0))
        max_dn = step * math.tan(math.radians(4.0))
        dz = talt - self.alt
        self.alt += max(-max_dn, min(max_up, dz))
        self._burn(self.cruise_power_w, dt)

    def _reached(self):
        self.conn.mav.mission_item_reached_send(self.current_seq)
        self.log(f"reached seq {self.current_seq} at t={self._sim_t:.0f}s alt={self.alt:.0f}")
        self.current_seq += 1

    # ------------------------------------------------------------------ message handling
    def handle(self, msg):
        t = msg.get_type()
        if t == "BAD_DATA":
            return
        if t == "HEARTBEAT":
            if msg.type == mav.MAV_TYPE_GCS:
                self.gcs_sys, self.gcs_comp = msg.get_srcSystem(), msg.get_srcComponent()
            return
        src, comp = self._target(msg)
        if t == "MISSION_COUNT":
            mt = getattr(msg, "mission_type", 0)
            self.pending = [mt, msg.count, 0, []]
            self.log(f"MISSION_COUNT type={mt} count={msg.count}")
            if msg.count == 0:
                self.missions[mt] = []
                self.pending = None
                self.conn.mav.mission_ack_send(src, comp, mav.MAV_MISSION_ACCEPTED, mt)
            else:
                self.conn.mav.mission_request_int_send(src, comp, 0, mt)
        elif t in ("MISSION_ITEM_INT", "MISSION_ITEM"):
            if self.pending is None:
                self.conn.mav.mission_ack_send(src, comp, mav.MAV_MISSION_ERROR, getattr(msg, "mission_type", 0))
                return
            mt, count, nxt, items = self.pending
            if msg.seq != nxt:
                # 重送請求
                self.conn.mav.mission_request_int_send(src, comp, nxt, mt)
                return
            if t == "MISSION_ITEM_INT":
                x, y = msg.x / 1e7, msg.y / 1e7
            else:
                x, y = msg.x, msg.y
            items.append({"seq": msg.seq, "frame": msg.frame, "command": msg.command, "current": msg.current,
                          "autocontinue": msg.autocontinue, "param1": msg.param1, "param2": msg.param2,
                          "param3": msg.param3, "param4": msg.param4, "x": x, "y": y, "z": msg.z,
                          "mission_type": mt})
            nxt += 1
            self.pending[2] = nxt
            if nxt < count:
                self.conn.mav.mission_request_int_send(src, comp, nxt, mt)
            else:
                self.missions[mt] = items
                self.pending = None
                self.conn.mav.mission_ack_send(src, comp, mav.MAV_MISSION_ACCEPTED, mt)
                self.log(f"mission type {mt} accepted: {count} items")
                if mt == 0:
                    self.current_seq = 0
                if self.dump:
                    with open(self.dump, "w", encoding="utf-8") as f:
                        json.dump(self.missions, f, indent=1)
        elif t == "MISSION_REQUEST_LIST":
            mt = getattr(msg, "mission_type", 0)
            self.conn.mav.mission_count_send(src, comp, len(self.missions[mt]), mt)
        elif t in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
            mt = getattr(msg, "mission_type", 0)
            items = self.missions[mt]
            if 0 <= msg.seq < len(items):
                it = items[msg.seq]
                self.conn.mav.mission_item_int_send(src, comp, it["seq"], it["frame"], it["command"], it["current"],
                                                    it["autocontinue"], it["param1"], it["param2"], it["param3"],
                                                    it["param4"], int(it["x"] * 1e7), int(it["y"] * 1e7), it["z"], mt)
        elif t == "MISSION_ACK":
            pass
        elif t == "MISSION_CLEAR_ALL":
            mt = getattr(msg, "mission_type", 0)
            self.missions[mt] = []
            self.conn.mav.mission_ack_send(src, comp, mav.MAV_MISSION_ACCEPTED, mt)
        elif t == "MISSION_SET_CURRENT":
            self.current_seq = msg.seq
            self.conn.mav.mission_current_send(self.current_seq)
        elif t == "PARAM_SET":
            name = msg.param_id.rstrip("\x00") if isinstance(msg.param_id, str) else msg.param_id.decode().rstrip("\x00")
            self.params[name] = float(msg.param_value)
            self.conn.mav.param_value_send(name.encode(), float(msg.param_value), mav.MAV_PARAM_TYPE_REAL32,
                                           len(self.params), list(self.params).index(name))
        elif t == "PARAM_REQUEST_READ":
            name = msg.param_id.rstrip("\x00") if isinstance(msg.param_id, str) else msg.param_id.decode().rstrip("\x00")
            if name in self.params:
                self.conn.mav.param_value_send(name.encode(), self.params[name], mav.MAV_PARAM_TYPE_REAL32,
                                               len(self.params), list(self.params).index(name))
        elif t == "PARAM_REQUEST_LIST":
            for i, (k, v) in enumerate(self.params.items()):
                self.conn.mav.param_value_send(k.encode(), v, mav.MAV_PARAM_TYPE_REAL32, len(self.params), i)
        elif t == "SET_MODE":
            self.mode = msg.custom_mode
            self.log("mode →", self.mode)
        elif t == "COMMAND_LONG":
            res = mav.MAV_RESULT_ACCEPTED
            if msg.command == mav.MAV_CMD_COMPONENT_ARM_DISARM:
                self.armed = msg.param1 >= 0.5
                self.log("armed" if self.armed else "disarmed")
            elif msg.command == mav.MAV_CMD_MISSION_START:
                self.mode = PLANE_MODES["AUTO"]
                self.armed = True
                self.current_seq = max(1, int(msg.param1)) if msg.param1 > 0 else 1
                self.log("AUTO start at seq", self.current_seq)
            elif msg.command == mav.MAV_CMD_DO_SET_MODE:
                self.mode = int(msg.param2)
                self.log("mode →", self.mode)
            elif msg.command == mav.MAV_CMD_REQUEST_MESSAGE:
                pass
            elif msg.command == mav.MAV_CMD_DO_REPOSITION:
                pass
            else:
                res = mav.MAV_RESULT_UNSUPPORTED
            self.conn.mav.command_ack_send(msg.command, res)

    # ------------------------------------------------------------------ main loop
    def run(self, duration: float | None = None):
        wall0 = time.time()
        last_wall = wall0
        self.log("listening on", self.conn.address)
        while True:
            now = time.time()
            wall_dt = now - last_wall
            last_wall = now
            dt = wall_dt * self.time_scale
            self._sim_t += dt
            self.step_flight(dt)
            if self.armed:
                self.trace.append((round(self._sim_t, 1), self.lat, self.lon, round(self.alt, 1), self.current_seq))
            if now - self._last_hb >= 1.0:
                self.send_heartbeat(); self._last_hb = now
            if now - self._last_pos >= 0.25:
                self.send_position(); self._last_pos = now
            if now - self._last_slow >= 1.0:
                self.send_slow(); self._last_slow = now
            while True:
                try:
                    msg = self.conn.recv_match(blocking=False)
                except ConnectionResetError:
                    msg = None
                if msg is None:
                    break
                self.handle(msg)
            if duration is not None and now - wall0 > duration:
                break
            time.sleep(0.02)
        if self.dump:
            with open(self.dump.replace(".json", "_trace.json"), "w", encoding="utf-8") as f:
                json.dump(self.trace, f)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--listen", default="udpin:127.0.0.1:14550")
    ap.add_argument("--home", default="23.9,120.7,100,0", help="lat,lon,alt_msl,yaw")
    ap.add_argument("--speed", type=float, default=21.5)
    ap.add_argument("--time-scale", type=float, default=10.0, help="模擬時間 / 牆鐘時間")
    ap.add_argument("--dump", default=None, help="收到的任務寫到此 JSON")
    ap.add_argument("--duration", type=float, default=None, help="牆鐘秒數後結束")
    ap.add_argument("--wind", default="0,0", help="風速 m/s,風向 deg")
    ap.add_argument("--sysid", type=int, default=1)
    ap.add_argument("--battery-wh", type=float, default=None,
                    help="電池標稱能量 Wh（預設 800；給 --energy-profile 時預設為剖面起點）。"
                         "應設為該機 EnergyModel.nominal_wh（Pioneer 1350、HERO 380），否則遙測電量%% 與計畫不一致")
    ap.add_argument("--cruise-power-w", type=float, default=350.0, help="固定翼巡航功率 W")
    ap.add_argument("--hover-power-w", type=float, default=None, help="垂直（VTOL）段懸停功率 W（預設 8×巡航）")
    ap.add_argument("--reserve-frac", type=float, default=0.0,
                    help="保留電量比例 r（僅用於到達保留量時的記錄與告警；規劃器之 r·E_nom）")
    ap.add_argument("--energy-profile", default=None,
                    help="計畫能量剖面 JSON（{'t_s':[],'e_wh':[]}／[[t,e],…]／含 waypoints 之 plan JSON）；"
                         "給定時電量直接照剖面走，遙測與計畫一致")
    ap.add_argument("--energy-error-frac", type=float, default=0.0,
                    help="讓實際耗電比模型／剖面多此比例（例 0.1 = 多 10%%），用來測試 ENERGY_UPDATE")
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args(argv)
    home = [float(x) for x in a.home.split(",")]
    wind = tuple(float(x) for x in a.wind.split(","))
    prof = load_energy_profile(a.energy_profile) if a.energy_profile else None
    batt = a.battery_wh
    if batt is None:
        batt = prof[1][0] if prof is not None else 800.0
    MockAutopilot(a.listen, home, a.speed, a.time_scale, a.dump, sysid=a.sysid, wind=wind,
                  battery_wh=float(batt), cruise_power_w=a.cruise_power_w, hover_power_w=a.hover_power_w,
                  reserve_frac=a.reserve_frac, energy_profile=prof, energy_error_frac=a.energy_error_frac,
                  verbose=not a.quiet).run(a.duration)


if __name__ == "__main__":
    sys.exit(main())
