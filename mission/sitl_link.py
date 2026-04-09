"""
SITL / MAVLink 即時遙測連結
與 Mission Planner 相容：支援 ArduPlane（固定翼）、ArduCopter（多旋翼）。

連線字串範例：
    udp:127.0.0.1:14550        # SITL 預設廣播
    udpin:0.0.0.0:14550        # 接收任意來源
    tcp:127.0.0.1:5760         # SITL TCP
    COM7,57600                 # 實機 USB
"""

from __future__ import annotations

import time
import queue
import threading
from dataclasses import dataclass, field
from typing import Optional, List, Tuple

from PyQt6.QtCore import QThread, pyqtSignal

from utils.logger import get_logger

logger = get_logger()


# ──────────────────────────────────────────────────────────────────────
# 資料結構
# ──────────────────────────────────────────────────────────────────────
@dataclass
class TelemetryFrame:
    """單次遙測快照（與 Mission Planner HUD 對應的核心欄位）"""
    sysid: int = 1            # 飛行器 system id（多機支援）
    lat: float = 0.0
    lon: float = 0.0
    alt_msl: float = 0.0      # 海拔（m）
    alt_rel: float = 0.0      # 相對起飛點高度（m）
    heading: float = 0.0      # 航向（deg, 0=N）
    ground_speed: float = 0.0 # 對地速度（m/s）
    air_speed: float = 0.0    # 空速（m/s, 固定翼）
    climb: float = 0.0        # 爬升率（m/s）
    roll: float = 0.0         # 滾轉（deg）
    pitch: float = 0.0        # 俯仰（deg）
    yaw: float = 0.0          # 偏航（deg）
    throttle: int = 0         # 油門 %
    battery_v: float = 0.0    # 電壓 V
    battery_pct: int = -1     # 電量 %（-1 = 未知）
    battery_a: float = 0.0    # 電流 A
    gps_fix: int = 0          # 0=No, 2=2D, 3=3D, 4=DGPS, 5=RTK
    gps_sats: int = 0
    hdop: float = 99.9
    mode: str = '---'         # 飛行模式（STABILIZE/AUTO/RTL...）
    armed: bool = False
    vehicle_type: str = '---' # 'COPTER' / 'PLANE' / 'ROVER' / ...
    system_status: str = ''   # ACTIVE/STANDBY/...
    mav_time_us: int = 0
    last_update: float = field(default_factory=time.time)

    def is_valid_gps(self) -> bool:
        return self.gps_fix >= 3 and abs(self.lat) > 1e-6


# ──────────────────────────────────────────────────────────────────────
# MAVLink 飛行模式對照（ArduPilot）
# ──────────────────────────────────────────────────────────────────────
_COPTER_MODES = {
    0: 'STABILIZE', 1: 'ACRO', 2: 'ALT_HOLD', 3: 'AUTO', 4: 'GUIDED',
    5: 'LOITER', 6: 'RTL', 7: 'CIRCLE', 9: 'LAND', 11: 'DRIFT',
    13: 'SPORT', 14: 'FLIP', 15: 'AUTOTUNE', 16: 'POSHOLD',
    17: 'BRAKE', 18: 'THROW', 19: 'AVOID_ADSB', 20: 'GUIDED_NOGPS',
    21: 'SMART_RTL', 22: 'FLOWHOLD', 23: 'FOLLOW', 24: 'ZIGZAG',
    25: 'SYSTEMID', 26: 'AUTOROTATE', 27: 'AUTO_RTL',
}
_PLANE_MODES = {
    0: 'MANUAL', 1: 'CIRCLE', 2: 'STABILIZE', 3: 'TRAINING', 4: 'ACRO',
    5: 'FBWA', 6: 'FBWB', 7: 'CRUISE', 8: 'AUTOTUNE', 10: 'AUTO',
    11: 'RTL', 12: 'LOITER', 13: 'TAKEOFF', 14: 'AVOID_ADSB',
    15: 'GUIDED', 16: 'INITIALISING', 17: 'QSTABILIZE', 18: 'QHOVER',
    19: 'QLOITER', 20: 'QLAND', 21: 'QRTL', 22: 'QAUTOTUNE', 23: 'QACRO',
    24: 'THERMAL',
}
_GPS_FIX_NAMES = {0: 'NO_GPS', 1: 'NO_FIX', 2: '2D', 3: '3D', 4: 'DGPS', 5: 'RTK_F', 6: 'RTK_FX'}


def _decode_mode(vehicle_type: str, custom_mode: int) -> str:
    if 'PLANE' in vehicle_type:
        return _PLANE_MODES.get(custom_mode, f'MODE_{custom_mode}')
    if 'COPTER' in vehicle_type or 'ROTOR' in vehicle_type:
        return _COPTER_MODES.get(custom_mode, f'MODE_{custom_mode}')
    return f'MODE_{custom_mode}'


def _decode_vehicle_type(mav_type: int) -> str:
    # MAV_TYPE 部分對照
    table = {
        1: 'PLANE',         # FIXED_WING
        2: 'COPTER',        # QUADROTOR
        13: 'COPTER',       # HEXAROTOR
        14: 'COPTER',       # OCTOROTOR
        15: 'COPTER',       # TRICOPTER
        10: 'ROVER',
        11: 'BOAT',
        20: 'VTOL_DUOROTOR',
        21: 'VTOL_QUADROTOR',
        22: 'VTOL_TILTROTOR',
    }
    return table.get(mav_type, f'TYPE_{mav_type}')


# ──────────────────────────────────────────────────────────────────────
# SITLLink — MAVLink 接收背景執行緒
# ──────────────────────────────────────────────────────────────────────
class SITLLink(QThread):
    """
    背景執行緒接收 MAVLink，emit telemetry 信號。
    對外完全透過 Qt 信號 → 主執行緒安全。
    """

    # 信號
    connected      = pyqtSignal(str)              # 連線字串
    disconnected   = pyqtSignal(str)              # 原因
    telemetry      = pyqtSignal(object)           # TelemetryFrame
    status_text    = pyqtSignal(int, str)         # severity, text
    error          = pyqtSignal(str)

    def __init__(self, conn_str: str = 'udpin:0.0.0.0:14550',
                 sysid_label: int = 1, parent=None):
        super().__init__(parent)
        self.conn_str = conn_str
        self._stop = False
        self._mav = None
        self._frame = TelemetryFrame(sysid=sysid_label)
        self._cmd_queue: 'queue.Queue' = queue.Queue()  # 主執行緒下指令給接收執行緒
        self._mav_lock = threading.Lock()

    # ── 公開控制 ──────────────────────────────────────────────────────
    def stop(self):
        self._stop = True
        if self._mav is not None:
            try:
                self._mav.close()
            except Exception:
                pass

    # ── 主執行緒 → 接收執行緒指令 ────────────────────────────────────
    def arm(self):       self._cmd_queue.put(('arm', None))
    def disarm(self):    self._cmd_queue.put(('disarm', None))
    def set_mode(self, mode_name: str):
        self._cmd_queue.put(('set_mode', mode_name))
    def takeoff(self, alt: float = 30.0):
        self._cmd_queue.put(('takeoff', alt))
    def upload_mission(self, waypoints: List[Tuple[float, float, float]]):
        """waypoints: [(lat, lon, alt_rel), ...]"""
        self._cmd_queue.put(('upload_mission', list(waypoints)))
    def mission_start(self):
        self._cmd_queue.put(('mission_start', None))
    def auto_start(self):
        """一鍵：AUTO + ARM + MISSION_START"""
        self._cmd_queue.put(('auto_start', None))
    def set_param(self, name: str, value: float, ptype: str = 'REAL32'):
        """寫入單一參數 (name, value, ptype: REAL32/INT32/INT16/INT8)"""
        self._cmd_queue.put(('set_param', (name, float(value), ptype)))
    def set_params(self, items):
        """批次寫入：items = [(name, value, ptype?), ...]"""
        self._cmd_queue.put(('set_params', list(items)))
    def get_param(self, name: str):
        self._cmd_queue.put(('get_param', name))

    # ── 執行緒主迴圈 ──────────────────────────────────────────────────
    def run(self):
        try:
            from pymavlink import mavutil
        except ImportError:
            self.error.emit('找不到 pymavlink，請執行: pip install pymavlink')
            return

        logger.info(f'[SITL] 嘗試連線: {self.conn_str}')
        try:
            self._mav = mavutil.mavlink_connection(
                self.conn_str,
                autoreconnect=True,
                source_system=255,   # GCS system id
                dialect='ardupilotmega',
            )
        except Exception as e:
            self.error.emit(f'MAVLink 連線失敗: {e}')
            logger.error(f'[SITL] 連線失敗: {e}', exc_info=True)
            return

        # 等 heartbeat
        try:
            hb = self._mav.wait_heartbeat(timeout=10)
        except Exception as e:
            self.error.emit(f'等待 heartbeat 失敗: {e}')
            return

        if hb is None:
            self.error.emit('10 秒內未收到 heartbeat — 確認 SITL 已啟動')
            return

        self._frame.vehicle_type = _decode_vehicle_type(hb.type)
        logger.info(f'[SITL] 已連線，飛行器類型: {self._frame.vehicle_type}')
        self.connected.emit(self.conn_str)

        # 要求高頻串流（與 Mission Planner 一致：4 Hz 位置/姿態）
        try:
            self._mav.mav.request_data_stream_send(
                self._mav.target_system, self._mav.target_component,
                0,  # MAV_DATA_STREAM_ALL
                4, 1
            )
        except Exception:
            pass

        # 主迴圈
        _last_hb = 0.0
        while not self._stop:
            # 處理待發送的命令
            try:
                while True:
                    cmd, arg = self._cmd_queue.get_nowait()
                    self._dispatch_command(cmd, arg)
            except queue.Empty:
                pass

            # 週期送 GCS heartbeat (1 Hz) 給 SITL，避免觸發 FS_GCS 失效保護
            # 多機情境下尤其重要 — 沒有 heartbeat → ArduPlane 會自動切 RTL
            _now = time.time()
            if _now - _last_hb >= 1.0:
                try:
                    from pymavlink import mavutil as _mu
                    self._mav.mav.heartbeat_send(
                        _mu.mavlink.MAV_TYPE_GCS,
                        _mu.mavlink.MAV_AUTOPILOT_INVALID,
                        0, 0, 0,
                    )
                except Exception:
                    pass
                _last_hb = _now

            try:
                msg = self._mav.recv_match(blocking=True, timeout=0.5)
            except Exception as e:
                self.error.emit(f'接收錯誤: {e}')
                break
            if msg is None:
                continue

            try:
                self._handle_message(msg)
            except Exception as e:
                logger.error(f'[SITL] 處理訊息失敗 {msg.get_type()}: {e}', exc_info=True)

        self.disconnected.emit('使用者中斷' if self._stop else '連線中斷')
        logger.info('[SITL] 執行緒結束')

    # ── 訊息分派 ──────────────────────────────────────────────────────
    def _handle_message(self, msg):
        mtype = msg.get_type()
        f = self._frame
        emit = False

        if mtype == 'HEARTBEAT':
            f.vehicle_type = _decode_vehicle_type(msg.type)
            f.armed = bool(msg.base_mode & 128)  # MAV_MODE_FLAG_SAFETY_ARMED
            f.mode  = _decode_mode(f.vehicle_type, msg.custom_mode)
            f.system_status = {0:'UNINIT',1:'BOOT',2:'CALIB',3:'STANDBY',
                                4:'ACTIVE',5:'CRITICAL',6:'EMERGENCY',
                                7:'POWEROFF',8:'TERMINATION'}.get(msg.system_status, '?')
            emit = True

        elif mtype == 'GLOBAL_POSITION_INT':
            f.lat = msg.lat / 1e7
            f.lon = msg.lon / 1e7
            f.alt_msl = msg.alt / 1000.0
            f.alt_rel = msg.relative_alt / 1000.0
            f.heading = msg.hdg / 100.0 if msg.hdg != 65535 else f.heading
            # vx,vy 是 cm/s
            vx = msg.vx / 100.0
            vy = msg.vy / 100.0
            f.ground_speed = (vx*vx + vy*vy) ** 0.5
            f.climb = -msg.vz / 100.0
            emit = True

        elif mtype == 'ATTITUDE':
            import math
            f.roll  = math.degrees(msg.roll)
            f.pitch = math.degrees(msg.pitch)
            f.yaw   = math.degrees(msg.yaw)
            emit = True

        elif mtype == 'VFR_HUD':
            f.air_speed = msg.airspeed
            f.ground_speed = msg.groundspeed
            f.heading = msg.heading
            f.throttle = msg.throttle
            f.alt_msl = msg.alt
            f.climb = msg.climb
            emit = True

        elif mtype == 'GPS_RAW_INT':
            f.gps_fix  = msg.fix_type
            f.gps_sats = msg.satellites_visible
            f.hdop     = msg.eph / 100.0 if msg.eph != 65535 else 99.9

        elif mtype == 'SYS_STATUS':
            f.battery_v   = msg.voltage_battery / 1000.0
            f.battery_a   = msg.current_battery / 100.0 if msg.current_battery != -1 else 0.0
            f.battery_pct = msg.battery_remaining

        elif mtype == 'PARAM_VALUE':
            try:
                pname = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode('utf-8','ignore')
                self.status_text.emit(6, f'PARAM {pname.strip(chr(0))} = {msg.param_value:g}')
            except Exception:
                pass

        elif mtype == 'STATUSTEXT':
            text = msg.text if isinstance(msg.text, str) else msg.text.decode('utf-8', 'ignore')
            self.status_text.emit(int(msg.severity), text.strip('\x00'))

        if emit:
            f.last_update = time.time()
            self.telemetry.emit(f)

    # ─────────────────────────────────────────────────────────────────
    # 命令發送（在接收執行緒中執行，與接收共用 self._mav）
    # ─────────────────────────────────────────────────────────────────
    def _dispatch_command(self, cmd: str, arg):
        if self._mav is None:
            return
        try:
            with self._mav_lock:
                if cmd == 'arm':
                    self._send_arm(True)
                elif cmd == 'disarm':
                    self._send_arm(False)
                elif cmd == 'set_mode':
                    self._send_set_mode(arg)
                elif cmd == 'takeoff':
                    self._send_takeoff(float(arg))
                elif cmd == 'upload_mission':
                    self._send_mission(arg)
                elif cmd == 'mission_start':
                    self._send_mission_start()
                elif cmd == 'auto_start':
                    self._send_auto_start()
                elif cmd == 'set_param':
                    self._send_param_set(*arg)
                elif cmd == 'set_params':
                    for it in arg:
                        if len(it) == 2:
                            self._send_param_set(it[0], it[1], 'REAL32')
                        else:
                            self._send_param_set(it[0], it[1], it[2])
                        time.sleep(0.05)
                elif cmd == 'get_param':
                    self._send_param_request(arg)
                logger.info(f'[SITL] 已發送命令: {cmd} {arg if cmd != "upload_mission" else f"{len(arg)} 點"}')
        except Exception as e:
            logger.error(f'[SITL] 命令 {cmd} 失敗: {e}', exc_info=True)
            self.status_text.emit(3, f'命令失敗: {cmd} - {e}')

    def _send_arm(self, arm: bool):
        from pymavlink import mavutil
        # 先嘗試關閉 pre-arm 檢查（SITL 測試用；實機請勿這樣做）
        if arm:
            try:
                self._mav.mav.param_set_send(
                    self._mav.target_system, self._mav.target_component,
                    b'ARMING_CHECK', 0.0,
                    mavutil.mavlink.MAV_PARAM_TYPE_INT32,
                )
                time.sleep(0.2)
            except Exception as e:
                logger.warning(f'[SITL] 關閉 ARMING_CHECK 失敗: {e}')
        # param2 = 21196 → force arm/disarm，繞過剩餘安全檢查
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1 if arm else 0,
            21196,
            0, 0, 0, 0, 0
        )

    def _send_set_mode(self, mode_name: str):
        """ArduPilot set_mode：用 mode_mapping 把名稱轉為 custom_mode"""
        mode_name = mode_name.upper()
        mapping = self._mav.mode_mapping() or {}
        if mode_name not in mapping:
            self.status_text.emit(3, f'未知飛行模式: {mode_name}')
            return
        mode_id = mapping[mode_name]
        self._mav.set_mode(mode_id)

    def _send_takeoff(self, alt: float):
        from pymavlink import mavutil
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, alt
        )

    def _send_mission_start(self):
        """MAV_CMD_MISSION_START：觸發 AUTO 任務執行（ArduPlane SITL 必要）"""
        from pymavlink import mavutil
        # 確認目前是 AUTO
        try:
            mapping = self._mav.mode_mapping() or {}
            if 'AUTO' in mapping:
                self._mav.set_mode(mapping['AUTO'])
                time.sleep(0.2)
        except Exception:
            pass
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    def _send_auto_start(self):
        """一鍵起飛：關 ARMING_CHECK → AUTO → ARM → MISSION_START"""
        from pymavlink import mavutil
        # 1. 關閉 prearm
        try:
            self._mav.mav.param_set_send(
                self._mav.target_system, self._mav.target_component,
                b'ARMING_CHECK', 0.0,
                mavutil.mavlink.MAV_PARAM_TYPE_INT32,
            )
            time.sleep(0.3)
        except Exception:
            pass
        # 2. 設 AUTO
        mapping = self._mav.mode_mapping() or {}
        if 'AUTO' in mapping:
            self._mav.set_mode(mapping['AUTO'])
            time.sleep(0.3)
        # 3. ARM (force)
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 21196, 0, 0, 0, 0, 0
        )
        time.sleep(0.5)
        # 4. 觸發任務
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    _PARAM_TYPE_MAP = {
        'INT8':   1,  # MAV_PARAM_TYPE_INT8
        'INT16':  3,
        'INT32':  6,
        'REAL32': 9,
    }

    def _send_param_set(self, name: str, value: float, ptype: str = 'REAL32'):
        from pymavlink import mavutil
        pid = name.encode('utf-8')[:16]
        tcode = self._PARAM_TYPE_MAP.get(ptype.upper(), 9)
        self._mav.mav.param_set_send(
            self._mav.target_system, self._mav.target_component,
            pid, float(value), tcode,
        )
        self.status_text.emit(6, f'PARAM_SET {name} = {value} ({ptype})')

    def _send_param_request(self, name: str):
        pid = name.encode('utf-8')[:16]
        self._mav.mav.param_request_read_send(
            self._mav.target_system, self._mav.target_component,
            pid, -1,
        )

    def _send_mission(self, waypoints):
        """上傳任務航點到飛控（MISSION_COUNT + MISSION_ITEM_INT 序列）。

        waypoints 支援兩種元素格式：
          (lat, lon, alt)                             → 視為 NAV_WAYPOINT
          (lat, lon, alt, cmd, p1, p2, p3, p4)        → 直接套用 MAVLink 指令
        第 0 項固定複製一份做為 Home（ArduPilot 規範）。
        """
        from pymavlink import mavutil
        if not waypoints:
            return

        WP = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT

        def _norm(item):
            if len(item) == 3:
                lat, lon, alt = item
                return (float(lat), float(lon), float(alt), WP, 0.0, 0.0, 0.0, 0.0)
            lat, lon, alt, cmd, p1, p2, p3, p4 = item
            return (float(lat), float(lon), float(alt),
                    int(cmd), float(p1), float(p2), float(p3), float(p4))

        items_norm = [_norm(w) for w in waypoints]
        # Home 固定為 WAYPOINT @ 第一個具有有效經緯度的點（跳過 DO_* 等
        # lat=lon=0 的純指令項）
        home_lat, home_lon = 0.0, 0.0
        for it in items_norm:
            if abs(it[0]) > 1e-9 or abs(it[1]) > 1e-9:
                home_lat, home_lon = it[0], it[1]
                break
        home = (home_lat, home_lon, 0.0, WP, 0.0, 0.0, 0.0, 0.0)
        items = [home] + items_norm
        n = len(items)

        tgt_sys, tgt_comp = self._mav.target_system, self._mav.target_component

        # 1. 清空現有任務
        self._mav.mav.mission_clear_all_send(tgt_sys, tgt_comp)
        time.sleep(0.1)

        # 2. 通知總數
        self._mav.mav.mission_count_send(tgt_sys, tgt_comp, n)

        # 3. 等飛控逐一索取
        for seq in range(n):
            req = self._mav.recv_match(
                type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'],
                blocking=True, timeout=3
            )
            if req is None:
                self.status_text.emit(3, f'上傳任務逾時 @seq={seq}')
                return
            lat, lon, alt, cmd, p1, p2, p3, p4 = items[req.seq]
            # DO_* 指令 (非 NAV) 使用 MISSION frame；NAV 指令用 relative alt
            if cmd in (178, 179, 177, 20, 115):  # DO_CHANGE_SPEED/SET_HOME/JUMP/RTL/CONDITION_YAW
                frame = mavutil.mavlink.MAV_FRAME_MISSION
            else:
                frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            current = 0   # ArduPilot 會自行決定 current；全部設 0 較安全
            autocontinue = 1
            self._mav.mav.mission_item_int_send(
                tgt_sys, tgt_comp, req.seq, frame, cmd,
                current, autocontinue,
                p1, p2, p3, p4,
                int(lat * 1e7), int(lon * 1e7),
                float(alt), mavutil.mavlink.MAV_MISSION_TYPE_MISSION
            )

        # 4. 等 MISSION_ACK
        ack = self._mav.recv_match(type='MISSION_ACK', blocking=True, timeout=3)
        if ack is None:
            self.status_text.emit(4, '上傳完成但未收到 ACK')
        else:
            self.status_text.emit(6, f'任務上傳完成（{n} 點）')
