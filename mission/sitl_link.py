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
from typing import List, Optional, Tuple

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
    # NED 速度分量（m/s，vel_d 向下為正，與 MAVLink vz 同號）。
    # 投放彈道要的是**向量**不是純量：只有地速+航向的話，有風時航向 != 航跡，
    # 前拋方向會算錯。GLOBAL_POSITION_INT 本來就帶這三個分量，直接留著。
    vel_n: float = 0.0
    vel_e: float = 0.0
    vel_d: float = 0.0
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
    # SERVO_OUTPUT_RAW：16 通道 PWM（μs），未收到時為 0
    servo_pwm: List[int] = field(default_factory=lambda: [0] * 16)
    last_update: float = field(default_factory=time.time)

    def is_valid_gps(self) -> bool:
        return self.gps_fix >= 3 and abs(self.lat) > 1e-6


# ──────────────────────────────────────────────────────────────────────
# ArduPilot SERVO_FUNCTION 對照表（節選 — 涵蓋 VTOL / QuadPlane / 雲台常見功能）
# 完整列表見 ArduPilot/libraries/SRV_Channel/SRV_Channel.h
# ──────────────────────────────────────────────────────────────────────
SERVO_FUNCTIONS = {
    0:  'Disabled',
    1:  'RCPassThru',
    4:  'Aileron',
    6:  'Mount1Yaw',
    7:  'Mount1Pitch',
    8:  'Mount1Roll',
    19: 'Elevator',
    21: 'Rudder',
    24: 'Flap',
    25: 'FlapAuto',
    26: 'Air Brake',
    33: 'Motor1', 34: 'Motor2', 35: 'Motor3', 36: 'Motor4',
    37: 'Motor5', 38: 'Motor6', 39: 'Motor7', 40: 'Motor8',
    51: 'RCIN1', 52: 'RCIN2', 53: 'RCIN3', 54: 'RCIN4',
    55: 'RCIN5', 56: 'RCIN6', 57: 'RCIN7', 58: 'RCIN8',
    59: 'RCIN9', 60: 'RCIN10', 61: 'RCIN11', 62: 'RCIN12',
    63: 'RCIN13', 64: 'RCIN14', 65: 'RCIN15', 66: 'RCIN16',
    70: 'Throttle',
    73: 'ThrottleLeft',
    74: 'ThrottleRight',
    75: 'TiltMotorLeft',
    76: 'TiltMotorRight',
    77: 'ElevonLeft',
    78: 'ElevonRight',
    79: 'VTailLeft',
    80: 'VTailRight',
    81: 'BoostThrottle',
    82: 'Motor9', 83: 'Motor10', 84: 'Motor11', 85: 'Motor12',
    91: 'TiltMotorRear',
    92: 'TiltMotorRearLeft',
    93: 'TiltMotorRearRight',
    94: 'Mount2Yaw',
    95: 'Mount2Pitch',
    96: 'Mount2Roll',
}


def servo_function_name(code: int) -> str:
    """把 SERVO_FUNCTION 數字代碼轉為人類可讀名稱。"""
    return SERVO_FUNCTIONS.get(int(code), f'Func{int(code)}')


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
    if 'PLANE' in vehicle_type or 'VTOL' in vehicle_type:
        # QuadPlane (VTOL) 基於 ArduPlane，共用固定翼模式表（含 Q* 模式）
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
    servo_param    = pyqtSignal(int, str, float)  # ch (1-16), key (FUNCTION/MIN/TRIM/MAX/REVERSED), value
    # 任務下載結果（sysid, [item dict, ...]）— 見 _recv_mission 的 docstring
    mission_downloaded    = pyqtSignal(int, list)
    # ⚠️ 失敗**必須**走自己的信號，不可以用「成功但空清單」代表失敗。
    #    空清單是一個合法且常見的答案（飛控裡真的沒有任務），兩者混在同一條
    #    路上，UI 就分不出「沒有航線」與「讀不回來」——而那兩者要做的事相反。
    mission_download_failed = pyqtSignal(int, str)   # sysid, 原因

    def __init__(self, conn_str: str = 'udpin:0.0.0.0:14550',
                 sysid_label: int = 1, vehicle_hint: str = '',
                 parent=None):
        super().__init__(parent)
        self.conn_str = conn_str
        self.sysid_label = int(sysid_label)   # ★ 公開的 sysid 識別碼，供外部
                                              # 邏輯（如 DCCPP 上傳分配）按 sysid
                                              # 而非 list index 對應 UAV 路徑使用
        self._stop = False
        self._mav = None
        # 多來源遙測分流：每個 MAVLink source system 各自一個 TelemetryFrame。
        # 一般「一條連線一架」拓樸（內建 SITL 各佔一個 TCP 埠、嵌入式蜂群各佔一個
        # mavproxy --out 埠）下只會有一個 key（== sysid_label）；但若連線背後的
        # mavproxy 把多架 fan-in 到同一個 --out，這裡會自動依真實 sysid 拆成多個
        # frame 各自 emit，HUD 便分出多張正確卡片，而非全部疊在一張、互相覆蓋閃爍。
        self._frames: dict[int, TelemetryFrame] = {}
        self._cmd_queue: 'queue.Queue' = queue.Queue()  # 主執行緒下指令給接收執行緒
        self._mav_lock = threading.Lock()
        # vehicle_hint: 由 UI 啟動時傳入的載具類型（'VTOL'/'PLANE'/'COPTER'）
        # 用途：ArduPlane QuadPlane 的 MAV_TYPE 仍回報 FIXED_WING (1)，
        #       無法從 heartbeat 區分純固定翼與 VTOL，因此必須從外部傳入。
        self._vehicle_hint = vehicle_hint.upper()
        self._q_enable_confirmed = False  # Q_ENABLE=1 已確認 → 此連線為 QuadPlane VTOL
        # ── 來源分類：哪些 source system 是**飛機**，哪些只是同網路上的節點 ──
        # ⚠️ 實測（2026-09-02）：接 WSL 蜂群時每條鏈路都冒出一張 UAV191 卡片，
        #    顯示 MODE_0 / 0 sat / 0.0 V / 座標 0,0。那不是飛機 —— 是我們自己的
        #    MAVROS（swarm_strike/launch_helpers.py: GCS_SYSTEM_ID = 191）。
        #    飛控把它的心跳轉發到 SERIAL0，於是地面站替一台機載電腦開了卡片。
        #    判準用 MAVLink 自己的約定，不要用「191」這個魔術數字 —— 對方換個
        #    system_id 就失效的守衛等於沒有守衛。
        self._vehicle_srcs: set = set()      # 已確認是飛機的來源 system
        self._nonvehicle_srcs: set = set()   # 已判定不是飛機（log 只記一次）

    # ── 公開控制 ──────────────────────────────────────────────────────
    def stop(self):
        self._stop = True
        if self._mav is not None:
            try:
                self._mav.close()
            except Exception:
                pass

    # ── 內部：送出 GCS heartbeat ──────────────────────────────────────
    def _send_gcs_heartbeat(self) -> None:
        """以 GCS 身分送一個 heartbeat 給對端。

        兩個用途：
          1. 維持 GCS 連線心跳，避免飛控觸發 FS_GCS 失效保護（自動 RTL）。
          2. ★ udpout / udp client 連線「破冰」：
             - Windows 上 udpout socket 在「首次 send」之前不會 bind 本地埠，
               此時呼叫 recvfrom 會直接拋 WinError 10022（無效引數）。
             - 被動 udpin 對端（例如嵌入式 mavproxy 的 `--out udpin:...`）也必須
               先收到我方封包，才知道要把遙測回送到哪個位址。
             因此第一次 wait_heartbeat 之前一定要先送出至少一包。
        """
        if self._mav is None:
            return
        try:
            from pymavlink import mavutil as _mu
            self._mav.mav.heartbeat_send(
                _mu.mavlink.MAV_TYPE_GCS,
                _mu.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )
        except Exception:
            pass

    # ── 主執行緒 → 接收執行緒指令 ────────────────────────────────────
    def arm(self):       self._cmd_queue.put(('arm', None))
    def disarm(self):    self._cmd_queue.put(('disarm', None))
    def set_mode(self, mode_name: str):
        self._cmd_queue.put(('set_mode', mode_name))
    def takeoff(self, alt: float = 30.0):
        self._cmd_queue.put(('takeoff', alt))

    def set_servo(self, channel: int, pwm: int):
        """MAV_CMD_DO_SET_SERVO：直接把某個伺服通道打到指定 PWM。

        用於投放機構致動（掛架伺服 / 電磁鎖）。

        ⚠️ 飛控端該通道的 SERVOn_FUNCTION 必須是 0 (Disabled)，
        否則這個指令會被飛行控制邏輯覆蓋掉、看起來「送出去了但沒作用」。
        ⚠️ SERVOn_TRIM 是上電時輸出的值 —— 若 TRIM 設在「放」的位置，
        飛控一上電就會投放。這兩件事都要在台架上實測確認。
        """
        self._cmd_queue.put(('set_servo', (int(channel), int(pwm))))

    def guided_takeoff(self, alt: float = 60.0):
        """固定翼相容的 GUIDED 起飛：切 ArduPlane TAKEOFF 模式 + force ARM
        （滑跑爬升）。純固定翼 GUIDED 不吃 NAV_TAKEOFF，故用 TAKEOFF 模式。"""
        self._cmd_queue.put(('guided_takeoff', float(alt)))

    def guided_goto(self, lat: float, lon: float, alt: float = 60.0):
        """GUIDED 飛往 (lat, lon, alt_rel)。用 guided 航點（MISSION_ITEM_INT
        current=2），因為 ArduPlane 固定翼 GUIDED 不執行 SET_POSITION_TARGET。"""
        self._cmd_queue.put(('guided_goto', (float(lat), float(lon), float(alt))))

    def change_speed(self, speed_mps: float, speed_type: int = 0):
        """DO_CHANGE_SPEED：即時設定目標空速 (type=0) / 地速 (type=1)。

        供終端同步打擊的飛行中 Time-on-Target 速度修正（[[tot_controller]]）使用：
        GCS 端每 ~1s 依各機 ETA 回算應有空速 → 呼叫此法 → 收斂同一命中時刻。
        """
        self._cmd_queue.put(('change_speed', (float(speed_mps), int(speed_type))))

    def get_latest_telemetry(self) -> Optional['TelemetryFrame']:
        """本連線主 sysid 的最新遙測快照（無則 None）。供外部讀取，避免直接碰 _frames。"""
        return self._frames.get(self.sysid_label)
    def upload_mission(self, waypoints: List[Tuple[float, float, float]]):
        """waypoints: [(lat, lon, alt_rel), ...]"""
        self._cmd_queue.put(('upload_mission', list(waypoints)))

    def download_mission(self):
        """向飛控索取目前的任務航線（= Mission Planner 的 Ctrl+F → Read WPs）。

        結果非同步走 `mission_downloaded` / `mission_download_failed` 信號。

        ⚠️⚠️ **一定要走 _cmd_queue，不可以在 GUI 執行緒直接碰 self._mav。**
        `mission/param_service.py:91-95` 就是反例：它從 GUI 執行緒直接
        `link._mav.mav.param_request_list_send(...)`，和接收執行緒的
        `recv_match` 搶同一個 pymavlink 連線物件，沒有拿 `_mav_lock`。
        那是一個沒有症狀的資料競爭（pymavlink 的 buffer 不是執行緒安全的）。
        """
        self._cmd_queue.put(('download_mission', None))

    def upload_fence_zones(self, zones, fence_type_bits=4,
                           alt_max_m=200.0, alt_min_m=-10.0,
                           fence_action=1):
        """上傳多區 Fence (NFZ + 威脅 + 圍籬整合) 至飛控 EEPROM。

        使用 ArduPilot 4.2+ 多區 Polygon Fence 協定：
            MAV_MISSION_TYPE_FENCE 內可同時包含：
              • POLYGON inclusion 群組（同 polygon 所有頂點 param1 = 該 poly 點數）
              • POLYGON exclusion 群組（同上但用 CMD 5002）
              • CIRCLE inclusion / exclusion 各為一個單點 mission item

        Args:
            zones            : List[FenceZone]
            fence_type_bits  : FENCE_TYPE bitmask (1=alt, 2=circle, 4=poly, 8=min_alt)
            alt_max_m        : FENCE_ALT_MAX
            alt_min_m        : FENCE_ALT_MIN（ArduCopter 4.0+ 有效）
            fence_action     : FENCE_ACTION (0=Report, 1=RTL/Land, 2=Land, 4=Brake)
        """
        self._cmd_queue.put((
            'upload_fence_zones',
            {
                'zones': list(zones),
                'fence_type_bits': int(fence_type_bits),
                'alt_max_m': float(alt_max_m),
                'alt_min_m': float(alt_min_m),
                'fence_action': int(fence_action),
            },
        ))

    def upload_fence(self, bundle):
        """
        上傳電子圍籬。
        bundle: mission.geofence_manager.MissionBundle
        """
        self._cmd_queue.put(('upload_fence', bundle))

    def clear_fence(self):
        """清除飛控 EEPROM 中所有 polygon fence 頂點 + 強制 FENCE_ENABLE=0。

        用於蜂群打擊上傳前，徹底排除任何殘留 fence 干擾自殺攻擊俯衝。
        """
        self._cmd_queue.put(('clear_fence', None))
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

    # ── VTOL (QuadPlane) 專用指令 ─────────────────────────────────
    def vtol_transition(self, state: int):
        """VTOL 飛行模式轉換。state: 3=多旋翼(MC), 4=固定翼(FW)"""
        self._cmd_queue.put(('vtol_transition', state))

    def set_mode_qhover(self):
        """切換至 QHOVER（QuadPlane 多旋翼懸停模式，custom_mode=18）"""
        self._cmd_queue.put(('set_mode', 'QHOVER'))

    def set_mode_qloiter(self):
        """切換至 QLOITER（QuadPlane 多旋翼定點模式，custom_mode=19）"""
        self._cmd_queue.put(('set_mode', 'QLOITER'))

    def _effective_vehicle_type(self, mav_type: int) -> str:
        """把 MAV_TYPE 解成顯示用類型，並補上 QuadPlane 的盲點。

        ⚠ QuadPlane 回報 MAV_TYPE=1（FIXED_WING）—— 光看 heartbeat
        **分不出** VTOL 與純固定翼，於是 3D 圖台永遠拿到 'PLANE'、
        永遠畫固定翼模型。這條連線其實知道真相：UI 傳入的 vehicle_hint
        與 Q_ENABLE 讀回結果。兩者任一確認就把 1 升格成 'VTOL'。
        """
        decoded = _decode_vehicle_type(mav_type)
        if decoded == 'PLANE' and (
                self._vehicle_hint == 'VTOL' or self._q_enable_confirmed):
            return 'VTOL'
        return decoded

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

        # ★ 先送一包 GCS heartbeat「破冰」再等對方 heartbeat。
        #   udpout（主動連嵌入式 mavproxy）必須先送，否則：
        #     - Windows udpout socket 未 bind → wait_heartbeat 的 recvfrom
        #       直接拋 WinError 10022；
        #     - 被動 udpin 對端不知我方位址 → 永遠不回送遙測 → 逾時。
        #   對 tcp / udpin / serial 連線而言這只是提早送一包心跳，無副作用。
        self._send_gcs_heartbeat()

        # 等 heartbeat
        try:
            hb = self._mav.wait_heartbeat(timeout=10)
        except Exception as e:
            self.error.emit(f'等待 heartbeat 失敗: {e}')
            return

        if hb is None:
            self.error.emit('10 秒內未收到 heartbeat — 確認 SITL 已啟動')
            return

        primary_type = self._effective_vehicle_type(hb.type)
        hb_src = int(hb.get_srcSystem())
        # 預先建立第一個 heartbeat 來源的 frame（跳過我方 GCS 回送 255 / 無效 0，
        # 以及非飛機節點 —— wait_heartbeat 不分來源，接 WSL 蜂群時很可能先收到
        # MAVROS 的心跳，那會在主迴圈守衛生效之前就先建立一張假卡片）。
        if hb_src not in (0, 255):
            if self._heartbeat_is_vehicle(hb) or hb_src == self.sysid_label:
                self._vehicle_srcs.add(hb_src)
                self._frame_for(hb_src).vehicle_type = primary_type
            else:
                self._note_non_vehicle(hb_src, hb)
        logger.info(
            f'[SITL] 已連線，飛行器類型: {primary_type} (來源 system {hb_src})'
        )
        self.connected.emit(self.conn_str)

        # ── VTOL QuadPlane 驗證 ─────────────────────────────────────
        # QuadPlane (VTOL) 回報 MAV_TYPE=FIXED_WING，無法只憑 heartbeat 區分
        # 純固定翼與 VTOL，因此對「可能是 VTOL」的連線主動讀 Q_ENABLE 驗證。
        # 但若 UI 已明確指定 vehicle_hint='PLANE'（例如嵌入式固定翼蜂群監看），
        # 就確定不是 VTOL → 跳過驗證，省去每條連線最多 5 秒的同步參數讀取阻塞
        # （多機監看時尤其重要：4 條連線 = 最多 20 秒延遲首筆遙測）。
        # _verify_vtol_q_enable 只設定 _q_enable_confirmed，跳過不影響固定翼
        # 起飛序列（_send_auto_start 對 hint='PLANE' 一律走固定翼分支）。
        if self._vehicle_hint != 'PLANE' and (
            'VTOL' in primary_type or 'PLANE' in primary_type
        ):
            self._verify_vtol_q_enable()

        # 要求高頻串流（與 Mission Planner 一致）
        try:
            self._mav.mav.request_data_stream_send(
                self._mav.target_system, self._mav.target_component,
                0,  # MAV_DATA_STREAM_ALL
                4, 1
            )
            # 額外要求 RC_CHANNELS 串流（含 SERVO_OUTPUT_RAW），10 Hz 以利 PWM 條圖即時
            self._mav.mav.request_data_stream_send(
                self._mav.target_system, self._mav.target_component,
                8,  # MAV_DATA_STREAM_RC_CHANNELS
                10, 1
            )
        except Exception:
            pass

        # 連線後讀取 SERVO{1..16}_FUNCTION 參數，供 ServoOutputPanel 顯示功能名稱
        self._request_servo_function_params()

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
                self._send_gcs_heartbeat()
                _last_hb = _now

            try:
                msg = self._mav.recv_match(blocking=True, timeout=0.5)
            except OSError as e:
                # 防禦性處理：對「尚未 bind 的 Windows udpout socket」呼叫 recvfrom
                # 會丟 WinError 10022（WSAEINVAL）。正常情況下連線啟動時的 pre-send
                # 已 bind 該 socket，且 pymavlink 不會重建 udpout socket（mavudp 無
                # autoreconnect 重連路徑），故此分支幾乎不會觸發 —— 保留為廉價保險：
                # 萬一遇到未 bind 狀態，補送一包 heartbeat 重新 bind 後續傳，
                # 不要當成致命錯誤把整條連線收掉。
                if getattr(e, 'winerror', None) == 10022:
                    self._send_gcs_heartbeat()
                    _last_hb = time.time()
                    time.sleep(0.1)
                    continue
                self.error.emit(f'接收錯誤: {e}')
                break
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

    # ── VTOL Q_ENABLE 驗證 ──────────────────────────────────────────
    def _verify_vtol_q_enable(self):
        """連線後驗證飛控是否啟用 QuadPlane (Q_ENABLE=1)。

        驗證流程：
          1. 發送 PARAM_REQUEST_READ 請求 Q_ENABLE 參數
          2. 等待 PARAM_VALUE 回應（最多 5 秒）
          3. 檢查 param_value 是否為 1
             ✓ Q_ENABLE=1 → VTOL QuadPlane 物理模型啟動成功
             ✗ Q_ENABLE=0 或未回應 → ArduPlane 以純固定翼模式運行

        ArduPilot 參數說明：
          Q_ENABLE (QuadPlane Enable):
            0 = 純固定翼模式（標準 ArduPlane）
            1 = QuadPlane 模式（啟用垂直起降馬達 + Q* 飛行模式）
        """
        try:
            # 發送 PARAM_REQUEST_READ: 讀取 Q_ENABLE
            param_name = b'Q_ENABLE\x00\x00\x00\x00\x00\x00\x00\x00'[:16]
            self._mav.mav.param_request_read_send(
                self._mav.target_system, self._mav.target_component,
                param_name, -1  # param_index=-1 → 依名稱查詢
            )
            logger.info('[SITL] 已發送 Q_ENABLE 參數讀取請求')

            # 等待 PARAM_VALUE 回應（最多 5 秒）
            start_time = time.time()
            while time.time() - start_time < 5.0:
                msg = self._mav.recv_match(
                    type='PARAM_VALUE', blocking=True, timeout=1.0
                )
                if msg is None:
                    continue

                # 解析參數名稱（去除 null padding）
                pname = msg.param_id
                if isinstance(pname, bytes):
                    pname = pname.decode('utf-8', 'ignore')
                pname = pname.strip('\x00').strip()

                if pname == 'Q_ENABLE':
                    q_val = msg.param_value
                    if q_val == 1.0:
                        # ✓ QuadPlane 啟用成功 → 標記此連線為 VTOL
                        self._q_enable_confirmed = True
                        logger.info(
                            f'[SITL] Q_ENABLE={q_val:g} → '
                            f'VTOL QuadPlane 物理模型已確認啟用'
                        )
                        self.status_text.emit(
                            6,
                            '[System] VTOL QuadPlane 物理模型啟動成功！'
                            '（ArduPlane 核心 + Q_ENABLE=1）'
                        )
                    else:
                        # ✗ Q_ENABLE ≠ 1 → 純固定翼
                        # ⚠ 使用者若是把它當 VTOL 在用（vehicle_hint='VTOL'），
                        #   這不是「資訊」而是**故障**：四軸馬達根本不存在，
                        #   任務會照跑、畫面一切正常，飛機就是不會垂直起飛。
                        #   啟動器已經注入 vtol_enforced.parm 保底，走到這裡代表
                        #   注入沒生效，必須大聲講。
                        if self._vehicle_hint == 'VTOL':
                            logger.error(
                                f'[SITL] Q_ENABLE={q_val:g}，但這條連線被當成 VTOL '
                                f'⇒ 四軸馬達不存在，不可能垂直起飛。'
                                f'vtol_enforced.parm 應該要把它設成 1 —— 沒生效。'
                            )
                            self.status_text.emit(
                                3,
                                f'[System] ⚠ VTOL 但 Q_ENABLE={q_val:g} — '
                                f'QuadPlane 未啟用，無法垂直起飛'
                            )
                        else:
                            logger.warning(
                                f'[SITL] Q_ENABLE={q_val:g} → '
                                f'QuadPlane 未啟用，當前為純固定翼模式'
                            )
                            self.status_text.emit(
                                4,
                                f'[System] Q_ENABLE={q_val:g}，'
                                f'QuadPlane 未啟用 — 以純固定翼模式運行'
                            )
                    return

            # 超時：未收到 Q_ENABLE 回應
            if self._vehicle_hint == 'VTOL':
                logger.error(
                    '[SITL] Q_ENABLE 讀取逾時，而這條連線被當成 VTOL '
                    '⇒ 無法確認四軸是否啟用，垂直起飛可能整段不會發生。'
                )
                self.status_text.emit(
                    3, '[System] ⚠ VTOL 但 Q_ENABLE 讀不到 — 無法確認 QuadPlane 已啟用'
                )
            else:
                logger.warning('[SITL] Q_ENABLE 參數讀取逾時（5 秒）')
                self.status_text.emit(
                    5, '[System] Q_ENABLE 參數讀取逾時 — 無法確認 VTOL 狀態'
                )

        except Exception as e:
            logger.error(f'[SITL] Q_ENABLE 驗證失敗: {e}', exc_info=True)
            self.status_text.emit(
                4, f'[System] Q_ENABLE 驗證失敗: {e}'
            )

    # ── 來源分類：是飛機，還是同一條鏈路上的其他節點 ─────────────────
    #: 非載具的 MAV_TYPE：這些東西會發心跳，但它們不是飛機。
    #: 5=ANTENNA_TRACKER 6=GCS 18=ONBOARD_CONTROLLER 26=GIMBAL 27=ADSB
    #: 30=FLARM 31=SERVO 32=ODID 36=LOG 37=OSD 38=IMU 39=GPS 40=WINCH
    _NON_VEHICLE_TYPES = frozenset({5, 6, 18, 26, 27, 30, 31, 32,
                                    36, 37, 38, 39, 40})
    _MAV_AUTOPILOT_INVALID = 8

    def _heartbeat_is_vehicle(self, msg) -> bool:
        """這包 HEARTBEAT 是飛機發的嗎？

        兩個判準取聯集，任一成立就**不是**飛機：
          * ``autopilot == MAV_AUTOPILOT_INVALID`` —— MAVLink 對「我不是飛控」
            的正式講法。MAVROS / 地面站 / 機載電腦都這樣填。
          * ``type`` 落在非載具清單（GCS、機載電腦、雲台、ADSB…）。

        兩條都查是刻意的：只查 autopilot，某個節點填了非 INVALID 的值就漏；
        只查 type，填 GENERIC(0) 的節點也會漏。
        實測基準（2026-09-02）：ArduPilot SITL 是 ``type=2 autopilot=3``
        （兩條都不咬）；MAVROS 是機載電腦節點（兩條都咬）。
        """
        try:
            if int(msg.autopilot) == self._MAV_AUTOPILOT_INVALID:
                return False
            if int(msg.type) in self._NON_VEHICLE_TYPES:
                return False
        except Exception:
            return True          # 讀不到欄位就別亂丟資料
        return True

    def _note_non_vehicle(self, src: int, msg) -> None:
        """只記一次 —— 1 Hz 心跳會把日誌洗掉。"""
        if src in self._nonvehicle_srcs:
            return
        self._nonvehicle_srcs.add(src)
        # ⚠️ 這個專案的 logger 是自訂包裝，簽名是 info(message, exc_info=False)
        #    —— 傳 %-args 會被當成 exc_info 而 TypeError。一律用 f-string。
        logger.info(
            f'[SITL] {self.conn_str} 忽略非飛機來源 system {src}'
            f'（type={getattr(msg, "type", "?")} '
            f'autopilot={getattr(msg, "autopilot", "?")}）'
            f' —— 機載電腦/地面站節點，不建立 UAV 卡片')

    # ── 來源 system → TelemetryFrame 取得/建立 ────────────────────────
    def _frame_for(self, src: int) -> TelemetryFrame:
        """取得（或建立）來源 system `src` 對應的 TelemetryFrame。

        first-seen 時建立並 log，讓「同一條連線冒出非預期 sysid」（mavproxy
        fan-in 多架到同一 --out）能被看見，而非無聲疊在 sysid_label 那張卡片上。
        """
        f = self._frames.get(src)
        if f is None:
            f = TelemetryFrame(sysid=src)
            self._frames[src] = f
            extra = '' if src == self.sysid_label else f'（與標示 sysid={self.sysid_label} 不同）'
            logger.info(f'[SITL] {self.conn_str} 偵測到來源 system {src}{extra}')
        return f

    # ── 訊息分派 ──────────────────────────────────────────────────────
    def _handle_message(self, msg):
        mtype = msg.get_type()

        # ── 與來源無關的訊息：直接路由到對應信號，不進 frame 分流 ──
        # （PARAM/STATUSTEXT 走 servo_param / status_text 等連線層級信號）
        if mtype == 'PARAM_VALUE':
            try:
                pname = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode('utf-8','ignore')
                pname = pname.strip('\x00').upper()
                pval = float(msg.param_value)
                # 攔截 SERVOn_FUNCTION/MIN/TRIM/MAX/REVERSED → 推送到 ServoOutputPanel
                # 例如 SERVO5_FUNCTION → ch=5, key=FUNCTION
                import re as _re
                m = _re.match(r'^SERVO(\d+)_(FUNCTION|MIN|TRIM|MAX|REVERSED)$', pname)
                if m:
                    ch = int(m.group(1))
                    key = m.group(2)
                    if 1 <= ch <= 16:
                        self.servo_param.emit(ch, key, pval)
                else:
                    self.status_text.emit(6, f'PARAM {pname} = {pval:g}')
            except Exception:
                pass
            return

        if mtype == 'STATUSTEXT':
            text = msg.text if isinstance(msg.text, str) else msg.text.decode('utf-8', 'ignore')
            self.status_text.emit(int(msg.severity), text.strip('\x00'))
            return

        # ── 遙測類：依來源 system 分流到各自的 TelemetryFrame ──
        # 跳過我方 GCS 心跳回送 (255) 與無效來源 (0)，避免冒出假卡片。
        src = int(msg.get_srcSystem())
        if src in (0, 255):
            return

        # ── 只有飛機才配一張 UAV 卡片 ───────────────────────────────
        # 心跳是唯一帶得出「我是什麼」的訊息，所以分類在這裡做。其餘訊息一律
        # 要等該來源先被確認成飛機才收，否則某個節點只送遙測不送心跳一樣會
        # 生出卡片。sysid_label 是 UI 明確標示的本鏈路機號，永遠放行。
        if mtype == 'HEARTBEAT':
            if not self._heartbeat_is_vehicle(msg) and src != self.sysid_label:
                self._note_non_vehicle(src, msg)
                return
            self._vehicle_srcs.add(src)
        elif src not in self._vehicle_srcs and src != self.sysid_label:
            return

        f = self._frame_for(src)
        emit = False

        if mtype == 'HEARTBEAT':
            f.vehicle_type = self._effective_vehicle_type(msg.type)
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
            # vx,vy,vz 是 cm/s（NED，vz 向下為正）
            vx = msg.vx / 100.0
            vy = msg.vy / 100.0
            f.ground_speed = (vx*vx + vy*vy) ** 0.5
            f.climb = -msg.vz / 100.0
            f.vel_n, f.vel_e, f.vel_d = vx, vy, msg.vz / 100.0
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

        elif mtype == 'SERVO_OUTPUT_RAW':
            # MAVLink 提供 servo1_raw ~ servo16_raw（μs）
            for ch in range(1, 17):
                attr = f'servo{ch}_raw'
                if hasattr(msg, attr):
                    f.servo_pwm[ch - 1] = int(getattr(msg, attr))
            emit = True

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
                elif cmd == 'download_mission':
                    self._recv_mission()
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
                elif cmd == 'guided_takeoff':
                    self._send_guided_takeoff(float(arg))
                elif cmd == 'guided_goto':
                    self._send_guided_goto(*arg)
                elif cmd == 'change_speed':
                    self._send_change_speed(*arg)
                elif cmd == 'vtol_transition':
                    self._send_vtol_transition(int(arg))
                elif cmd == 'set_servo':
                    self._send_set_servo(*arg)
                elif cmd == 'upload_fence':
                    self._send_fence(arg)
                elif cmd == 'upload_fence_zones':
                    self._send_fence_zones(arg)
                elif cmd == 'clear_fence':
                    self._send_clear_fence()
                logger.info(f'[SITL] 已發送命令: {cmd} {arg if cmd != "upload_mission" else f"{len(arg)} 點"}')
        except Exception as e:
            logger.error(f'[SITL] 命令 {cmd} 失敗: {e}', exc_info=True)
            self.status_text.emit(3, f'命令失敗: {cmd} - {e}')

    def _arm_with_prearm(self, force_fallback: bool = True,
                         wait_s: float = 3.0) -> bool:
        """解鎖：**先走真的 PreArm 檢查**，被擋才視情況 force。

        真實開發模擬的原則：
          * 不再預先把 ARMING_CHECK 歸零 —— 那個參數在 ArduPilot 4.6+ 已移除，
            舊寫法是對不存在的參數 param_set，本來就是**無作用**的迷信碼。
          * 先發正常 ARM（param2=0），讓 PreArm 檢查真的跑。過得了 = 真機也
            過得了。
          * 只有在 force_fallback=True 且正常解鎖失敗時才 force（21196），
            並且**大聲**講：這代表這份任務/組態在真機上會被擋。

        回傳是否最後成功解鎖。
        """
        from pymavlink import mavutil
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0,          # param2=0 → 正常解鎖，吃 PreArm
        )
        deadline = time.time() + max(0.5, wait_s)
        prearm_msgs: list = []
        armed = False
        while time.time() < deadline:
            msg = self._mav.recv_match(
                type=['HEARTBEAT', 'STATUSTEXT', 'COMMAND_ACK'],
                blocking=True, timeout=0.5,
            )
            if msg is None:
                continue
            t = msg.get_type()
            if t == 'HEARTBEAT':
                if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    armed = True
                    break
            elif t == 'STATUSTEXT':
                txt = msg.text if isinstance(msg.text, str) else msg.text.decode('utf-8', 'ignore')
                if 'PreArm' in txt or 'Arm' in txt:
                    prearm_msgs.append(txt)
        if armed:
            logger.info('[SITL] 正常解鎖成功（PreArm 檢查全數通過）')
            self.status_text.emit(6, '[System] ✅ 解鎖成功 — PreArm 檢查通過（真機同樣可解鎖）')
            return True
        # 正常解鎖失敗
        reasons = '; '.join(prearm_msgs[-4:]) if prearm_msgs else '（沒收到 PreArm 訊息）'
        logger.error(f'[SITL] 正常解鎖失敗，PreArm 擋下: {reasons}')
        self.status_text.emit(3, f'[System] ⚠ PreArm 擋住解鎖: {reasons}')
        if not force_fallback:
            return False
        logger.warning(
            '[SITL] 改用 FORCE ARM (21196) 繞過 —— ⚠ 這份任務/組態在真機上會被擋，'
            '請修正上面列的 PreArm 原因而不是依賴 force。'
        )
        self.status_text.emit(
            4, '[System] ⚠ 已 FORCE ARM 繞過 — 真機會被擋，請修 PreArm 原因')
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 21196, 0, 0, 0, 0, 0,
        )
        time.sleep(0.5)
        return True

    def _send_arm(self, arm: bool):
        from pymavlink import mavutil
        if arm:
            # 真實開發模擬：走 PreArm 檢查的正常解鎖，被擋才 force（會大聲警告）。
            # 舊版先對 ARMING_CHECK 做 param_set —— 該參數 4.6+ 已移除，那行
            # 從來沒有作用，只是讓人誤以為檢查被關掉了。
            self._arm_with_prearm(force_fallback=True)
            return
        # DISARM 維持 force（落地後要能立刻上鎖）
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 21196, 0, 0, 0, 0, 0
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

    # ── 起飛 / 飛到：依機型分流 ─────────────────────────────────────
    # ⚠️⚠️ 這裡原本寫死 ArduPlane 的模式編號（TAKEOFF=13、GUIDED=15）。
    #      在 ArduCopter 上 13 = SPORT、15 = AUTOTUNE ⇒ 右鍵起飛會把多旋翼
    #      切進 SPORT，既不是 GUIDED 也不會起飛，**而且完全沒有錯誤訊息**：
    #      set_mode 成功了，只是成功切到了錯的模式。
    #      模式一律用 mode_mapping() 依名稱查（pymavlink 依 heartbeat 的機型
    #      給出正確對照表），查不到就大聲報錯，不要退回寫死的數字。
    _APM_MODE_TAKEOFF = 13      # 僅供 ArduPlane 沒有 TAKEOFF 別名時的退路
    _APM_MODE_GUIDED = 15       # 同上

    def _current_vehicle_type(self) -> str:
        f = self.get_latest_telemetry()
        return (getattr(f, 'vehicle_type', '') or '').upper() if f else ''

    def _set_mode_by_name(self, name: str, fallback_id=None) -> bool:
        """依名稱切模式。回傳是否送得出去。

        名稱查不到時只有在呼叫端明確給 fallback_id 才用數字，
        並且會警告 —— 數字是機型相關的，猜錯就是切到別的模式。
        """
        mapping = self._mav.mode_mapping() or {}
        if name in mapping:
            self._mav.set_mode(mapping[name])
            return True
        if fallback_id is None:
            self.status_text.emit(
                3, f'這台機沒有 {name} 模式（mode_mapping 查不到），指令未送出')
            return False
        from pymavlink import mavutil
        self.status_text.emit(
            4, f'mode_mapping 沒有 {name}，改用寫死的 custom_mode={fallback_id}'
               f'（機型相關，若模式跑掉先查這裡）')
        self._mav.mav.set_mode_send(
            self._mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, fallback_id)
        return True

    def _send_guided_takeoff(self, alt: float):
        vt = self._current_vehicle_type()
        if vt == 'COPTER':
            # 多旋翼的原生起飛：GUIDED -> ARM -> MAV_CMD_NAV_TAKEOFF。
            # 這台機沒有「TAKEOFF 飛行模式」，硬切 13 會落到 SPORT。
            if not self._set_mode_by_name('GUIDED'):
                return
            time.sleep(0.4)
            if not self._arm_with_prearm(force_fallback=True):
                self.status_text.emit(3, '解鎖失敗，起飛中止')
                return
            # 等飛控真的進 ARMED 再送起飛，否則 NAV_TAKEOFF 會被丟掉
            t0 = time.time()
            while time.time() - t0 < 5.0:
                f = self.get_latest_telemetry()
                if f is not None and f.armed:
                    break
                time.sleep(0.2)
            self._send_takeoff(alt)
            self.status_text.emit(
                6, f'🛫 多旋翼 GUIDED 起飛（NAV_TAKEOFF ~{alt:.0f}m）')
            return
        # 固定翼 / QuadPlane 走 TAKEOFF 模式滑跑爬升：
        # ArduPlane 純固定翼 GUIDED 不執行 SET_POSITION_TARGET、也拒絕
        # GUIDED+NAV_TAKEOFF。
        self._set_mode_by_name('TAKEOFF', fallback_id=self._APM_MODE_TAKEOFF)
        time.sleep(0.3)
        self._arm_with_prearm(force_fallback=True)
        self.status_text.emit(
            6, f'🛫 固定翼 GUIDED 起飛（TAKEOFF 模式爬升 ~{alt:.0f}m）')

    def _send_guided_goto(self, lat: float, lon: float, alt: float):
        from pymavlink import mavutil
        # 確保在 GUIDED。⚠️ 退路的 custom_mode=15 是 ArduPlane 的 GUIDED，
        # 在 ArduCopter 上是 AUTOTUNE —— 只有 mode_mapping 真的沒有 GUIDED
        # 時才會用到，而且會警告。
        try:
            self._set_mode_by_name('GUIDED', fallback_id=self._APM_MODE_GUIDED)
            time.sleep(0.2)
        except Exception:
            pass
        # guided 航點：current=2 = 「飛到這」
        self._mav.mav.mission_item_int_send(
            self._mav.target_system, self._mav.target_component, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0, 0, 0, 0,
            int(lat * 1e7), int(lon * 1e7), float(alt),
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        self.status_text.emit(6, f'🎯 GUIDED 飛往 ({lat:.5f},{lon:.5f}) @ {alt:.0f}m')

    def _send_change_speed(self, speed_mps: float, speed_type: int):
        """DO_CHANGE_SPEED：param1=速度類型(0=空速,1=地速)、param2=目標速度、param3=油門(-1 不變)。

        AUTO / GUIDED 皆生效；終端同步打擊在 GUIDED 平飛段以此即時調速收斂命中時刻。
        """
        from pymavlink import mavutil
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
            float(speed_type), float(speed_mps), -1, 0, 0, 0, 0)

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
        """一鍵起飛：(VTOL: QLOITER→真解鎖→AUTO) / (固定翼: AUTO→真解鎖) → MISSION_START

        ArduPilot QuadPlane AUTO 起飛流程：
          1. 先切 QLOITER（啟動四軸馬達控制迴路）
          2. ARM（四軸馬達就緒）
          3. 切 AUTO（執行任務，第一個 NAV_VTOL_TAKEOFF 會讓飛機垂直升空）
          4. MISSION_START

        固定翼 AUTO 起飛流程：
          1. 切 AUTO
          2. ARM
          3. MISSION_START（NAV_TAKEOFF 觸發跑道起飛）
        """
        from pymavlink import mavutil
        # 真實開發模擬：不再嘗試關 PreArm（ARMING_CHECK 4.6+ 已移除，舊寫法
        # 本來就無作用）。解鎖一律先走真檢查，被擋才 force + 大聲警告。
        mapping = self._mav.mode_mapping() or {}
        # QuadPlane 回報 MAV_TYPE=1 (FIXED_WING)，無法從 heartbeat 區分
        # 優先用 UI 傳入的 vehicle_hint，其次用 Q_ENABLE 驗證結果
        is_vtol = (self._vehicle_hint == 'VTOL') or self._q_enable_confirmed
        logger.info(f'[SITL] auto_start: is_vtol={is_vtol} '
                    f'(hint={self._vehicle_hint}, q_enable={self._q_enable_confirmed})')

        if is_vtol:
            # ── VTOL QuadPlane 起飛序列 ──
            # 先進 QLOITER 讓四軸馬達控制器啟動，否則直接 AUTO 可能四軸馬達不轉
            if 'QLOITER' in mapping:
                self._mav.set_mode(mapping['QLOITER'])
                self.status_text.emit(6, '[VTOL] 切換 QLOITER → 四軸馬達就緒')
                time.sleep(0.5)
            # ARM：先走真 PreArm 檢查，被擋才 force（狀態列會警告）
            self._arm_with_prearm(force_fallback=True)
            # 切 AUTO → 執行任務（NAV_VTOL_TAKEOFF 開始垂直起飛）
            if 'AUTO' in mapping:
                self._mav.set_mode(mapping['AUTO'])
                self.status_text.emit(6, '[VTOL] 切換 AUTO → 執行 VTOL 任務')
                time.sleep(0.3)
        else:
            # ── 固定翼起飛序列 ──
            if 'AUTO' in mapping:
                self._mav.set_mode(mapping['AUTO'])
                time.sleep(0.3)
            # ARM：同上，先真檢查
            self._arm_with_prearm(force_fallback=True)

        # 觸發任務
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

    def _send_clear_fence(self):
        """清除飛控 EEPROM 中所有 polygon fence 頂點，並強制 FENCE_ENABLE=0。

        用法：蜂群末端飽和打擊（自殺撞擊任務）上傳前呼叫，徹底排除
        任何殘留 fence 攔截俯衝。
        """
        from pymavlink import mavutil
        if self._mav is None:
            return
        try:
            # 1) FENCE_ENABLE=0 → 立即停用 fence 動作（即便仍有頂點殘留）
            self._send_param_set('FENCE_ENABLE', 0, 'INT8')
            time.sleep(0.05)
            # 2) FENCE_TOTAL=0 → 標記 polygon 頂點數為 0
            self._send_param_set('FENCE_TOTAL', 0, 'INT8')
            time.sleep(0.05)
            # 3) FENCE_TYPE=0 → bitmask 清空，不啟用任何 fence 類型
            self._send_param_set('FENCE_TYPE', 0, 'INT8')
            time.sleep(0.05)
            # 4) 用 mission_clear_all_send 對 FENCE mission type 清空頂點
            self._mav.mav.mission_clear_all_send(
                self._mav.target_system, self._mav.target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
            )
            self.status_text.emit(6, '🚫 已清除所有 fence (蜂群打擊不使用)')
            logger.info(f'[SITL] sysid={self.sysid_label} 清除 fence 完成')
        except Exception as e:
            logger.warning(f'[SITL] 清除 fence 失敗: {e}')

    def _send_fence(self, bundle):
        """
        上傳電子圍籬到飛控。
        bundle: mission.geofence_manager.MissionBundle（含 .geofence 與 .fence_params）

        實作步驟（依 ArduPilot 4.2+ AC_PolyFence_loader）：
            1) 寫入所有 FENCE_* 參數（FENCE_ENABLE 最後寫入避免半成品 trigger）
            2) 多邊形頂點透過 MISSION_COUNT + MISSION_ITEM_INT 上傳，
               mission_type = MAV_MISSION_TYPE_FENCE (=1)
        """
        from pymavlink import mavutil
        if bundle is None or self._mav is None:
            return
        g = bundle.geofence
        fp = dict(bundle.fence_params)
        # 先停用 fence，等所有頂點/參數上傳完才重啟，避免中途 RTL
        self._send_param_set('FENCE_ENABLE', 0, 'INT8')
        time.sleep(0.05)

        # 1) 上傳頂點（mission type = 1 = FENCE）
        try:
            verts = g.vertices
            count = len(verts)
            self._mav.mav.mission_count_send(
                self._mav.target_system, self._mav.target_component,
                count,
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
            )
            # ArduPilot 4.2+ Polygon Fence 用
            #   MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION (5001)
            CMD_INCL = mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
            for seq, (lat, lon) in enumerate(verts):
                # 等待 MISSION_REQUEST(_INT) 再回送對應 seq
                msg = self._mav.recv_match(
                    type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'],
                    blocking=True, timeout=2,
                )
                if msg is None:
                    logger.warning(f'[Fence] 上傳超時 seq={seq}')
                    break
                self._mav.mav.mission_item_int_send(
                    self._mav.target_system, self._mav.target_component,
                    seq,
                    mavutil.mavlink.MAV_FRAME_GLOBAL,
                    CMD_INCL,
                    0,    # current
                    1,    # autocontinue
                    float(count),  # param1: 頂點總數
                    0, 0, 0,
                    int(round(lat * 1e7)), int(round(lon * 1e7)),
                    0.0,
                    mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                )
            # 等 MISSION_ACK
            ack = self._mav.recv_match(type='MISSION_ACK', blocking=True, timeout=3)
            if ack is None:
                logger.warning('[Fence] 未收到 MISSION_ACK')
        except Exception as e:
            logger.error(f'[Fence] 頂點上傳失敗: {e}', exc_info=True)

        # 2) 寫入 FENCE_* 參數（高度上下限、ACTION、TYPE 等）
        for name, val in fp.items():
            if name == 'FENCE_ENABLE':
                continue
            ptype = 'INT8' if isinstance(val, int) and -128 <= val <= 127 \
                else ('INT16' if isinstance(val, int) and abs(val) < 32768 else 'REAL32')
            try:
                self._send_param_set(name, val, ptype)
                time.sleep(0.04)
            except Exception:
                pass

        # 3) 最後啟用 fence
        time.sleep(0.1)
        self._send_param_set('FENCE_ENABLE', 1, 'INT8')
        self.status_text.emit(6,
            f'🛡️ 圍籬已上傳: {len(g.vertices)} 頂點, '
            f'alt=[{g.alt_min_m:.0f},{g.alt_max_m:.0f}]m')

    def _send_fence_zones(self, args: dict):
        """上傳「多區 fence（NFZ + 威脅 + 圍籬整合）」至飛控 EEPROM。

        實作策略：
            1) FENCE_ENABLE=0 暫停（避免上傳半成品觸發 RTL）
            2) 統計所有 mission item 總數：
                 polygon: N 頂點各一個 item（同 polygon 共用 param1=該 poly 點數）
                 circle:  1 個 item
            3) MISSION_COUNT + 依序回應 MISSION_REQUEST → MISSION_ITEM_INT
            4) MISSION_ACK 確認
            5) 寫 FENCE_TYPE / FENCE_ACTION / FENCE_ALT_MAX / FENCE_ALT_MIN
            6) FENCE_ENABLE=1 啟用
        """
        from pymavlink import mavutil
        if self._mav is None:
            return
        zones = args.get('zones') or []
        if not zones:
            return

        # ── 把所有 zone 展開成「待上傳的 mission item 描述」陣列 ──
        # 每個元素：(cmd_id, param1, lat_deg, lon_deg)
        # 其中 cmd_id ∈ {5001, 5002, 5003, 5004}
        items = []
        for z in zones:
            cmd_id = z.mavlink_cmd_id  # 5001/5002/5003/5004
            if cmd_id in (5001, 5002):
                # Polygon：把所有頂點（含 CIRCLE 展開後的近似多邊形）依序送
                verts = z.polygon_vertices(circle_segments=16)
                if len(verts) < 3:
                    continue
                n = len(verts)
                for lat, lon in verts:
                    items.append((cmd_id, float(n), float(lat), float(lon)))
            elif cmd_id in (5003, 5004):
                # Circle：lat/lon = 中心，param1 = 半徑
                if z.center is None or z.radius_m <= 0:
                    continue
                lat, lon = z.center
                items.append((cmd_id, float(z.radius_m), float(lat), float(lon)))

        if not items:
            logger.warning('[Fence] 沒有有效 fence item 可上傳')
            return

        # 1) 先 disable fence 避免半成品觸發 action
        try:
            self._send_param_set('FENCE_ENABLE', 0, 'INT8')
            time.sleep(0.05)
        except Exception:
            pass

        # 2) 清除舊 fence（避免新舊混雜）
        try:
            self._mav.mav.mission_clear_all_send(
                self._mav.target_system, self._mav.target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
            )
            time.sleep(0.1)
        except Exception:
            pass

        # 3) MISSION_COUNT
        try:
            self._mav.mav.mission_count_send(
                self._mav.target_system, self._mav.target_component,
                len(items),
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
            )
        except Exception as e:
            logger.error(f'[Fence] mission_count_send 失敗: {e}')
            return

        # 4) 依序回應 MISSION_REQUEST(_INT) → mission_item_int_send
        try:
            for seq, (cmd_id, p1, lat, lon) in enumerate(items):
                msg = self._mav.recv_match(
                    type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'],
                    blocking=True, timeout=2,
                )
                if msg is None:
                    logger.warning(f'[Fence] 上傳超時 seq={seq} cmd={cmd_id}')
                    break
                self._mav.mav.mission_item_int_send(
                    self._mav.target_system, self._mav.target_component,
                    seq,
                    mavutil.mavlink.MAV_FRAME_GLOBAL,
                    cmd_id,
                    0,    # current
                    1,    # autocontinue
                    p1,   # param1 = (polygon 點數) 或 (circle 半徑 m)
                    0, 0, 0,
                    int(round(lat * 1e7)),
                    int(round(lon * 1e7)),
                    0.0,
                    mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                )
            ack = self._mav.recv_match(
                type='MISSION_ACK', blocking=True, timeout=3
            )
            if ack is None:
                logger.warning('[Fence] 多區 fence 未收到 MISSION_ACK')
        except Exception as e:
            logger.error(f'[Fence] 多區 fence 上傳失敗: {e}', exc_info=True)

        # 5) 寫 FENCE_* 參數
        try:
            self._send_param_set('FENCE_TYPE', int(args['fence_type_bits']), 'INT8')
            time.sleep(0.04)
            self._send_param_set('FENCE_ACTION', int(args['fence_action']), 'INT8')
            time.sleep(0.04)
            self._send_param_set('FENCE_ALT_MAX', float(args['alt_max_m']), 'REAL32')
            time.sleep(0.04)
            # FENCE_ALT_MIN 僅部分機型支援 — 寫入失敗不影響整體上傳
            try:
                self._send_param_set('FENCE_ALT_MIN', float(args['alt_min_m']), 'REAL32')
                time.sleep(0.04)
            except Exception:
                pass
        except Exception as e:
            logger.warning(f'[Fence] FENCE_* 參數寫入部分失敗: {e}')

        # 6) 最後啟用 fence
        time.sleep(0.1)
        try:
            self._send_param_set('FENCE_ENABLE', 1, 'INT8')
        except Exception:
            pass

        # 統計 polygon / circle 數量給狀態列
        n_poly = sum(1 for z in zones if z.shape.value == 'POLYGON')
        n_circ = sum(1 for z in zones if z.shape.value == 'CIRCLE')
        self.status_text.emit(
            6,
            f'🛡️ 多區 fence 已上傳: {len(items)} items '
            f'(polygons={n_poly}, circles={n_circ}, type=0x{int(args["fence_type_bits"]):02X})'
        )
        logger.info(
            f'[Fence] sysid={self.sysid_label} 多區上傳完成: '
            f'{len(items)} items, type_bits={args["fence_type_bits"]}'
        )

    def _request_servo_function_params(self):
        """連線後讀取 SERVO{1..16}_FUNCTION/MIN/TRIM/MAX/REVERSED，
        供 UI 端對應 PWM 通道顯示功能名稱與設定範圍。"""
        if self._mav is None:
            return
        try:
            with self._mav_lock:
                for ch in range(1, 17):
                    for key in ('FUNCTION', 'MIN', 'TRIM', 'MAX', 'REVERSED'):
                        pname = f'SERVO{ch}_{key}'
                        try:
                            self._send_param_request(pname)
                        except Exception:
                            pass
                        time.sleep(0.02)  # 避免一次塞爆 buffer
        except Exception as e:
            logger.warning(f'[SITL] SERVO 參數讀取失敗: {e}')

    def _send_vtol_transition(self, state: int):
        """MAV_CMD_DO_VTOL_TRANSITION (3000)：VTOL 飛行模式轉換。

        state: 3 = MAV_VTOL_STATE_MC（多旋翼模式）
               4 = MAV_VTOL_STATE_FW（固定翼模式）
        """
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            3000,        # MAV_CMD_DO_VTOL_TRANSITION
            0,           # confirmation
            float(state),  # param1: 目標狀態
            0, 0, 0, 0, 0, 0
        )
        state_name = {3: '多旋翼 (MC)', 4: '固定翼 (FW)'}.get(state, f'state={state}')
        self.status_text.emit(6, f'VTOL 轉換 → {state_name}')

    def _send_set_servo(self, channel: int, pwm: int):
        """MAV_CMD_DO_SET_SERVO (183)：param1 = 通道 (1-16)，param2 = PWM (μs)。

        沒有回讀確認 —— 要確認伺服真的動了，看 SERVO OUTPUT 面板上該通道的
        PWM 是否跳到目標值（那是飛控回報的 SERVO_OUTPUT_RAW，不是我們自己
        送出去的值，所以它是真的讀回）。
        """
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            183,          # MAV_CMD_DO_SET_SERVO
            0,            # confirmation
            float(channel),   # param1: servo 通道
            float(pwm),       # param2: PWM (μs)
            0, 0, 0, 0, 0
        )
        self.status_text.emit(6, f'SERVO ch{channel} → {pwm} μs')

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
            # 3000 = DO_VTOL_TRANSITION 也屬於 DO 類指令
            # 93 = NAV_DELAY（地面/空中等待）— 不需要實際 lat/lon/alt 用 MISSION
            if cmd in (93, 112, 113, 114, 178, 179, 177, 20, 115, 3000):
                # NAV_DELAY/CONDITION_DELAY/DISTANCE/CHANGE_ALT/DO_CHANGE_SPEED/...
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
        elif int(ack.type) != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            # ⚠️ 這個分支以前不存在：無論 ack.type 是什麼都印「上傳完成」。
            #    飛控**明確拒收**（NO_SPACE / INVALID / UNSUPPORTED_FRAME…）
            #    和成功長得一模一樣，而下一步「切 AUTO」會照常送出去，
            #    飛的是**上一份**任務。這是本 repo 反覆出現的靜默失敗形狀。
            name = self._mission_result_name(ack.type)
            self.status_text.emit(3, f'⚠️ 飛控拒收任務：{name}（{n} 點未生效）')
            logger.error(f'[SITL] 任務上傳被拒: {name} (type={ack.type})')
        else:
            self.status_text.emit(6, f'任務上傳完成（{n} 點）')

    # ── 任務下載（MISSION_REQUEST_LIST → COUNT → ITEM_INT × n → ACK）──
    @staticmethod
    def _mission_result_name(code) -> str:
        """把 MISSION_ACK.type 解成看得懂的名字。

        ⚠️ 不要自己抄一份對照表 —— pymavlink 本來就帶著完整的
           MAV_MISSION_RESULT 列舉，抄表只會在協定加值時默默過期，
           而「默默過期的解碼表」會把新的錯誤碼顯示成舊的錯誤原因。
        """
        try:
            from pymavlink import mavutil
            return mavutil.mavlink.enums['MAV_MISSION_RESULT'][int(code)].name
        except Exception:
            return f'UNKNOWN({code})'

    def _recv_mission(self):
        """從飛控下載目前的任務航線（= Mission Planner 的 Ctrl+F → Read WPs）。

        走 MAVLink mission 協定的讀取半邊：

            MISSION_REQUEST_LIST   →  MISSION_COUNT(n)
            MISSION_REQUEST_INT(i) →  MISSION_ITEM_INT(i)      × n
            MISSION_ACK(ACCEPTED)                              ← 我方收尾

        和 `_send_mission` 一樣，**在接收執行緒裡、持著 `_mav_lock` 同步阻塞**
        地做完一整筆交易，中途自己 recv_match 把 mission 訊息從主迴圈手上接走。
        mission 協定是嚴格的一問一答，拆成非同步狀態機要多一整層，而這條路徑
        只在使用者按下按鈕時跑一次。

        ⚠️ 產出的是**純 dict**，不是 `mission.waypoint.Waypoint`。
           `Waypoint.__post_init__` 會做 `MAVCommand(command)`，而那個列舉裡
           沒有 DO_JUMP(177)、DO_SET_MODE(176)、DO_LAND_START(189)、
           NAV_CONTINUE_AND_CHANGE_ALT(30) 等常見指令（諷刺的是 `_send_mission`
           自己的 frame 判斷清單就寫著 177）⇒ 飛控只要回一筆我們列舉外的指令，
           整趟下載就會連同它的 ValueError 一起消失。**下載端不可以對飛控
           回來的資料做封閉列舉驗證** —— 那一側的內容不是我們決定的。
        """
        from pymavlink import mavutil
        sysid = self.sysid_label
        tgt_sys, tgt_comp = self._mav.target_system, self._mav.target_component
        MT = mavutil.mavlink.MAV_MISSION_TYPE_MISSION

        def _fail(reason: str):
            logger.warning(f'[SITL] 任務下載失敗: {reason}')
            self.status_text.emit(3, f'任務下載失敗：{reason}')
            self.mission_download_failed.emit(sysid, reason)

        # ⚠️ 整段期間主迴圈是停住的 —— 它的 1 Hz GCS heartbeat（run() 裡
        #    `if _now - _last_hb >= 1.0`）跑不到。航點多時累計阻塞可以到數十秒，
        #    而沒有 GCS 心跳會觸發 FS_GCS ⇒ 飛控自動 RTL。所以這裡自己補送。
        #    （上傳路徑有同樣的問題，只是它從沒跑過大任務。）
        last_hb = time.time()

        def _pump_heartbeat():
            nonlocal last_hb
            if time.time() - last_hb >= 1.0:
                self._send_gcs_heartbeat()
                last_hb = time.time()

        # 1. 要清單
        self._mav.mav.mission_request_list_send(tgt_sys, tgt_comp, MT)
        cnt = self._mav.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
        if cnt is None:
            _fail('等 MISSION_COUNT 逾時（5 s）')
            return
        n = int(cnt.count)

        # n == 0 是**合法**答案：飛控裡真的沒有任務。走成功路徑，讓 UI 顯示
        # 「0 個航點」而不是紅字 —— 那正是 auto_mission=False 時的正常樣子。
        if n == 0:
            self.status_text.emit(6, '飛控目前沒有任務（0 點）')
            self.mission_downloaded.emit(sysid, [])
            return

        # 2. 逐項索取
        items = []
        for seq in range(n):
            got = None
            for _attempt in range(3):
                _pump_heartbeat()
                self._mav.mav.mission_request_int_send(tgt_sys, tgt_comp, seq, MT)
                msg = self._mav.recv_match(
                    type=['MISSION_ITEM_INT', 'MISSION_ITEM'],
                    blocking=True, timeout=3,
                )
                if msg is None:
                    continue
                # ⚠️ 飛控可能回別的 seq（重送、亂序）。「收到就算數」會讓航線
                #    默默錯位 —— 產出一張畫得出來、但**是錯的**地圖，而那比
                #    畫不出來更糟：它不會有人去查。
                if int(msg.seq) != seq:
                    continue
                got = msg
                break
            if got is None:
                _fail(f'seq {seq} 讀不回來（重試 3 次；共 {n} 項）')
                return

            # MISSION_ITEM_INT 的 x/y 是 int×1e7；MISSION_ITEM 是 float 度。
            if got.get_type() == 'MISSION_ITEM_INT':
                lat, lon = got.x / 1e7, got.y / 1e7
            else:
                lat, lon = float(got.x), float(got.y)
            items.append({
                'seq':          int(got.seq),
                'command':      int(got.command),
                'frame':        int(got.frame),
                'lat':          lat,
                'lon':          lon,
                'alt':          float(got.z),
                'param1':       float(got.param1),
                'param2':       float(got.param2),
                'param3':       float(got.param3),
                'param4':       float(got.param4),
                'current':      int(getattr(got, 'current', 0)),
                'autocontinue': int(getattr(got, 'autocontinue', 1)),
            })

        # 3. 收尾 ACK：協定要求，不送的話部分飛控會停在傳輸狀態，
        #    下一次下載直接逾時。
        try:
            self._mav.mav.mission_ack_send(
                tgt_sys, tgt_comp, mavutil.mavlink.MAV_MISSION_ACCEPTED, MT)
        except Exception:
            pass

        logger.info(f'[SITL] 任務下載完成：{len(items)} 項 (sysid={sysid})')
        self.status_text.emit(6, f'任務下載完成（{len(items)} 點）')
        self.mission_downloaded.emit(sysid, items)
