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

import copy
import functools
import math
import os
import struct
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


@functools.lru_cache(maxsize=1)
def _mavlink_enums():
    """pymavlink 的 MAVLink 常數模組（延遲載入：本檔不在 import 時就拉 pymavlink）。"""
    from pymavlink import mavutil
    return mavutil.mavlink


def _mav_const(*names):
    """依名稱取 MAVLink 常數；第一個存在的名字勝出，全都不存在回 None。

    pymavlink 沒有鎖版本（requirements 是 >=），MAVLink 上游又會改名
    （例如 VTOL_DUOROTOR → VTOL_TAILSITTER_DUOROTOR）—— 名字查不到時寧可讓
    那一列不存在，也不要讓每一包心跳都丟 AttributeError。
    """
    m = _mavlink_enums()
    for n in names:
        v = getattr(m, n, None)
        if v is not None:
            return int(v)
    return None


@functools.lru_cache(maxsize=1)
def _vehicle_type_table() -> dict:
    """MAV_TYPE → 顯示用類型。

    ⚠️ 數字一律取自 pymavlink 的列舉，不手抄。舊版手抄表把三種 VTOL 寫成 20/21/22，
       實際是 19/20/21（pymavlink 2.4.49：19 VTOL_DUOROTOR、20 VTOL_QUADROTOR、
       21 VTOL_TILTROTOR、22 VTOL_RESERVED2）—— 整排錯一格：尾座雙旋翼顯示成
       TYPE_19、尾座四旋翼顯示成 VTOL_DUOROTOR、傾轉旋翼顯示成 VTOL_QUADROTOR。
    """
    rows = (
        (('MAV_TYPE_FIXED_WING',), 'PLANE'),
        (('MAV_TYPE_QUADROTOR',), 'COPTER'),
        (('MAV_TYPE_HEXAROTOR',), 'COPTER'),
        (('MAV_TYPE_OCTOROTOR',), 'COPTER'),
        (('MAV_TYPE_TRICOPTER',), 'COPTER'),
        (('MAV_TYPE_GROUND_ROVER',), 'ROVER'),
        (('MAV_TYPE_SURFACE_BOAT',), 'BOAT'),
        (('MAV_TYPE_VTOL_DUOROTOR', 'MAV_TYPE_VTOL_TAILSITTER_DUOROTOR'), 'VTOL_DUOROTOR'),
        (('MAV_TYPE_VTOL_QUADROTOR', 'MAV_TYPE_VTOL_TAILSITTER_QUADROTOR'), 'VTOL_QUADROTOR'),
        (('MAV_TYPE_VTOL_TILTROTOR',), 'VTOL_TILTROTOR'),
    )
    table = {}
    for names, label in rows:
        v = _mav_const(*names)
        if v is not None:
            table[v] = label
    return table


def _decode_vehicle_type(mav_type: int) -> str:
    return _vehicle_type_table().get(mav_type, f'TYPE_{mav_type}')


def _mav_type_name(mav_type) -> str:
    """MAV_TYPE 數字 → 列舉名（去掉 MAV_TYPE_ 前綴），查不到回 TYPE_n。"""
    try:
        return _mavlink_enums().enums['MAV_TYPE'][int(mav_type)].name.replace('MAV_TYPE_', '', 1)
    except Exception:
        return f'TYPE_{mav_type}'


#: 非載具的 MAV_TYPE（名稱 → 數字由 pymavlink 查）：這些東西會發心跳，但它們不是飛機。
#: ⚠️ 舊版手抄的數字註解從 30 起整排錯兩格（寫 30=FLARM 31=SERVO 32=ODID 36=LOG
#:    37=OSD 38=IMU 39=GPS 40=WINCH），實際是 30=CAMERA 31=CHARGING_STATION 32=FLARM
#:    33=SERVO 34=ODID 36=BATTERY 37=PARACHUTE 38=LOG 39=OSD 40=IMU 41=GPS 42=WINCH
#:    ⇒ 舊集合裡根本沒有 SERVO/ODID/GPS/WINCH（33/34/41/42）。
_NON_VEHICLE_TYPE_NAMES = (
    'ANTENNA_TRACKER', 'GCS', 'ONBOARD_CONTROLLER', 'GIMBAL', 'ADSB', 'CAMERA',
    'CHARGING_STATION', 'FLARM', 'SERVO', 'ODID', 'BATTERY', 'PARACHUTE',
    'LOG', 'OSD', 'IMU', 'GPS', 'WINCH',
)


@functools.lru_cache(maxsize=1)
def _non_vehicle_types() -> frozenset:
    vals = (_mav_const(f'MAV_TYPE_{n}') for n in _NON_VEHICLE_TYPE_NAMES)
    return frozenset(v for v in vals if v is not None)


# ──────────────────────────────────────────────────────────────────────
# 參數讀回比對（float32 感知）
# ──────────────────────────────────────────────────────────────────────
def _as_f32(v: float) -> float:
    """壓成 float32 實際能表示的那個 double（超出範圍飽和成 ±inf，同 C 的轉型）。"""
    d = float(v)
    try:
        return struct.unpack('<f', struct.pack('<f', d))[0]
    except OverflowError:
        return math.inf if d > 0 else -math.inf


def _f32_ordinal(f: float) -> int:
    """float32 的「格點序號」：相鄰的兩個 float32 差 1；+0 與 -0 同為 0。"""
    i = struct.unpack('<i', struct.pack('<f', f))[0]
    return i if i >= 0 else -(i & 0x7FFFFFFF)


def _param_type_code(ptype) -> Optional[int]:
    """ptype → MAV_PARAM_TYPE 數字。None / 0 = 不知道型別。

    接受數字或名稱（'INT8'、'REAL32'、'MAV_PARAM_TYPE_UINT16'…，名稱取自 pymavlink
    列舉）。認不得的名稱**拒收**（ValueError），不可默默當成 REAL32。
    """
    if ptype is None:
        return None
    if isinstance(ptype, str):
        key = ptype.strip().upper()
        if not key.startswith('MAV_PARAM_TYPE_'):
            key = 'MAV_PARAM_TYPE_' + key
        v = _mav_const(key)
        if v is None or key.endswith('_ENUM_END'):
            raise ValueError(f'不認得的參數型別：{ptype!r}')
        return v
    code = int(ptype)
    return code if code else None


def values_match(expected, actual, ptype=None) -> bool:
    """讀回值 `actual` 是否等於預期值 `expected`（在 MAVLink 的 float32 格上比）。

    PARAM_VALUE.param_value 是 float32；整數型參數由 ArduPilot 以 C 轉型送成 float
    （不是逐位元編碼）。所以：
      * 整數型（MAV_PARAM_TYPE 1..8）：兩邊壓成 float32 後**完全相等**。預期 2.4、
        飛控存 2 ⇒ 不符（使用者給的值沒有生效）。|值| > 2^24 時 float32 本身就分不出
        相鄰整數，這是協定的極限，不是這裡放水。
      * REAL32 / REAL64 / 型別未知（None、0）：兩邊壓成 float32 後差 ≤ 1 個 float32
        格點（ULP）。1 格是給「十進位文字只印 7~8 位有效數字」的往返誤差；
        `%.6f` 把 4e-07 印成 0 這種錯誤差了上億格，一定會被抓到。
      * 任一邊是 None / NaN ⇒ 不符；±inf 只等於同號 inf。
    """
    if expected is None or actual is None:
        return False
    try:
        e, a = float(expected), float(actual)
    except (TypeError, ValueError):
        return False
    if math.isnan(e) or math.isnan(a):
        return False
    code = _param_type_code(ptype)
    e32, a32 = _as_f32(e), _as_f32(a)
    if code is not None and 1 <= code <= 8:
        return e32 == a32
    if math.isinf(e32) or math.isinf(a32):
        return e32 == a32
    return abs(_f32_ordinal(e32) - _f32_ordinal(a32)) <= 1


# ──────────────────────────────────────────────────────────────────────
# 「寫入任務」(upload_mission_items) 用的 ArduPilot 事實表
#
# 來源：~/ardupilot-airframe（唯讀）的 libraries/AP_Mission/AP_Mission.cpp。
# ⚠️ 這些表存在的理由：飛控**不會**把寫進去的東西原樣讀回來。讀回驗證如果
#    寫成「全欄位嚴格相等」，第一個 NAV_WAYPOINT 就會誤報（Plane 只存 param2/3
#    的 uint8、非位置指令根本不存 lat/lon/alt/frame）；如果寫成「全部忽略」，
#    就等於沒有驗證。所以表裡每一條都是讀過原始碼才寫的。
# ──────────────────────────────────────────────────────────────────────

# AP_Mission::stored_in_location()：只有這些指令會把 x/y/z/frame 存進飛控。
# 其餘指令（NAV_DELAY 93、RTL 20、DO_* 、CONDITION_*）在
# mavlink_int_to_mission_cmd() 裡根本不讀 x/y/z/frame，
# mission_cmd_to_mavlink_int() 讀回時 x=y=z=0、frame=0（MAV_FRAME_GLOBAL）
# ⇒ 這些欄位「不可驗證」，比對時跳過而不是拿 0 去比。
_MISSION_CMDS_STORED_IN_LOCATION = frozenset({
    16, 17, 18, 19, 21, 22, 30, 31, 36, 82, 84, 85, 92, 94,
    179, 188, 189, 191, 195, 201,
    5000, 5001, 5002, 5003, 5004, 5100,
})

# 位置類指令：寫入的 frame → 讀回的 frame
#   AP_Mission::mavlink_int_to_mission_cmd()  寫入：
#     MISSION(2)/GLOBAL(0)/GLOBAL_INT(5)                → relative_alt=0
#     GLOBAL_RELATIVE_ALT(3)/_INT(6)                    → relative_alt=1
#     GLOBAL_TERRAIN_ALT(10)/_INT(11)                   → relative_alt=1, terrain_alt=1
#     其他                                              → MAV_MISSION_UNSUPPORTED_FRAME
#   AP_Mission::mission_cmd_to_mavlink_int()  讀回：
#     relative_alt=0 → GLOBAL(0)；=1 → GLOBAL_RELATIVE_ALT(3)；terrain → TERRAIN_ALT(10)
#   ⇒ 「MISSION frame 寫進去、GLOBAL 讀回來」是正常的，但 3 讀回成 0
#      （相對高度被吃掉）不是 —— 那會讓飛機在 AGL 與 MSL 之間差一個 home 高度。
_MISSION_LOCATION_FRAME_READBACK = {0: 0, 2: 0, 5: 0, 3: 3, 6: 3, 10: 10, 11: 10}

# 參數在飛控端以整數 / 部分欄位儲存的指令（見 mavlink_int_to_mission_cmd 的
# 各 case：cmd.p1 是 uint16，Plane 的 NAV_WAYPOINT 只存 param2/3 的 uint8…）。
# 這些指令的 param 比對採「寬鬆」：讀回 0（該欄位飛控不存）、或差 < 1（整數截斷）、
# 或被夾到 255/65535，都算相符；差更多就是真的不符。
# 不在這個集合的指令一律嚴格比對（float32 相對誤差 1e-6）。
_MISSION_CMDS_LOSSY_PARAMS = frozenset({
    16, 17, 18, 19, 20, 21, 22, 30, 31, 36, 82, 84, 85, 92, 93, 94,
    112, 114, 115, 177, 178, 181, 182, 183, 184, 189, 201, 3000,
})

# 飛控讀回時「不是你寫的那個值」的欄位：NAV_LOITER_TIME 的 param3 讀回固定 ±1
# （mission_cmd_to_mavlink_int: packet.param3 = ccw ? -1 : 1）。
_MISSION_PARAMS_NOT_ECHOED = {19: (3,)}


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
    # 任務**寫入**結果（sysid, ok, 訊息）。每次 upload_mission_items() 恰好一次，
    # 成功或失敗都是；ok=True 只在飛控 ACK(ACCEPTED)（且 verify 時讀回相符）之後。
    mission_uploaded = pyqtSignal(int, bool, str)
    # 每一則 PARAM_VALUE（不分來源、不分是誰要的）：
    #   (sysid, name, value, MAV_PARAM_TYPE, param_index, param_count)
    # 前四個是對外約定的簽名；後兩個是附加的（ParamService 要靠 count 判斷整表收齊）。
    # PyQt 對 Python slot 會丟掉多出來的引數 ⇒ 只收 4 個參數的 slot 照樣能接。
    # ⚠️ 不帶 compid：同 sysid 的相機若也回 PARAM_VALUE，一樣從這裡出去；
    #    要「飛控的值」請用 read_params()（只收自駕儀元件的回覆）。
    param_value = pyqtSignal(int, str, float, int, int, int)
    # CAMERA_FEEDBACK（只收自駕儀元件送的）：(sysid, dict) —— 欄位見 _on_camera_feedback
    camera_feedback = pyqtSignal(int, dict)
    # 同 sysid 底下**非自駕儀**元件（相機、雲台、伴隨電腦…）的心跳：(sysid, compid, 資訊)
    # 這些心跳不會更新機卡；資訊欄位見 _note_component。
    component_heartbeat = pyqtSignal(int, int, dict)

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
        # ── 同一個 sysid 底下的元件（compid）──
        # ⚠️ 2026-10-14 離線重現：相機 / 伴隨電腦（例如 compid 100、MAV_TYPE_CAMERA）
        #    與飛控共用 sysid 時，它的心跳搭上「src == sysid_label 永遠放行」這條規則，
        #    直接寫進機卡 ⇒ 機卡被蓋成 TYPE_30 / MODE_0 / disarmed。
        #    只有自駕儀元件能更新機卡；其他元件記在 _components，經
        #    component_heartbeat 信號與 get_components() 對外。
        self._autopilot_compid: dict = {}    # sysid → 自駕儀 compid
        self._components: dict = {}          # sysid → {compid → 資訊 dict}
        self._cam_stats: dict = {}           # (sysid, cam_idx) → CAMERA_FEEDBACK 統計
        self._state_lock = threading.Lock()  # 保護上面兩個（接收執行緒寫、GUI 執行緒讀）
        # ── 參數讀回（read_params）：等待者清單，由接收執行緒的 PARAM_VALUE 分支填值 ──
        self._param_cv = threading.Condition()
        self._param_waiters: list = []
        self._recv_thread_ident: Optional[int] = None   # run() 所在執行緒
        self._link_closed = False
        # ── tlog（opt-in，預設不寫任何檔案；見 start_tlog）──
        self._tlog_lock = threading.Lock()
        self._tlog_file = None
        self._tlog_path: Optional[str] = None
        self._tlog_msgs = 0
        self._tlog_bytes = 0
        self._tlog_error: Optional[str] = None
        self._tlog_last_flush = 0.0

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

    def upload_mission_items(self, items, verify: bool = True):
        """把一份任務**寫入**飛控（= Mission Planner 的 Write WPs）。

        `items`：與 `_recv_mission` 產出相同形狀的 dict 清單
        （seq/command/frame/lat/lon/alt/param1-4/current/autocontinue；seq 0 是 Home 槽）。
        結果非同步走 `mission_uploaded(sysid, ok, 訊息)`，**恰好一次**，
        成功 / 失敗 / 例外都一樣。`verify=True` 時 ACK 之後會讀回逐項比對。

        ⚠️ 只做「深拷貝快照 + 放進 _cmd_queue」，**不碰 socket**（理由同
           download_mission）。快照讓呼叫端之後改動 / 重用自己的清單不會影響
           已排隊的寫入 —— 確認對話框顯示的就是實際送出的內容。
        ⚠️ 與舊的 upload_mission()（收 (lat,lon,alt) 元組、猜 frame、先 clear_all）
           是兩條不同的路，舊的行為不變。
        """
        try:
            snapshot = copy.deepcopy(list(items))
        except Exception as e:
            # 連快照都做不出來（None / 不可迭代 / 不可複製）：仍然要**恰好一次**回報，
            # 否則 UI 會永遠等一個不會來的結果。
            self._report_mission_upload(False, f'寫入前檢查失敗：任務資料無法讀取（{e}）')
            return
        self._cmd_queue.put(('upload_mission_items', (snapshot, bool(verify))))

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

    def read_params(self, names, timeout_s: float = 3.0, retries: int = 2,
                    types_out: Optional[dict] = None) -> dict:
        """**同步**讀回一批參數：回傳 ``{name: float | None}``，None = 讀不到。

        給工作執行緒（飛前閘門）用，仿 _verify_vtol_q_enable 的作法：以名稱送
        PARAM_REQUEST_READ（param_index=-1），等名稱相符的 PARAM_VALUE。不同的是
        這裡**不在呼叫端碰 self._mav**：請求走 _cmd_queue 由接收執行緒送出，回覆由
        接收執行緒的 PARAM_VALUE 分支交回 —— 讀的期間遙測照常流動，也沒有跟
        recv_match 搶連線物件的資料競爭（見 download_mission 的說明）。

        * 只收**請求送出之後**、來自目標 sysid 的**自駕儀元件**的回覆；同 sysid 的
          相機 / 別架飛控回的同名參數一律不算。請求目標是 (target_system, 自駕儀
          compid)，不是廣播 —— 免得飛控把讀參數請求轉發給伴隨電腦。
        * ``timeout_s``：每一輪的等待上限（秒）。``retries``：讀不齊時再補送幾輪
          （只補缺的）。最壞總等待 ≈ timeout_s × (retries + 1)。飛控沒有的參數
          **不會有任何回覆**（GCS_Param.cpp param_io_timer：找不到就 return），
          所以它一定會等滿全部輪數後才回 None。
        * 回覆只收第一則；ArduPilot 送 PARAM_VALUE 不帶「是回哪個請求」，若同名
          參數剛被別人改過，可能拿到改之前那一則 —— 那只會造成「不符」（再讀一次即可），
          不會把不符讀成相符。
        * ``types_out``：給一個 dict 就順便填 ``{name: MAV_PARAM_TYPE}``（只填讀到的），
          可直接交給 values_match()。
        * 不用 MAVFTP —— CubeBlack 每次 MAVFTP 永久漏 1.7 KB 記憶體。
        * ⚠️ 不可在接收執行緒（run() 內）呼叫：那會自己等自己。也別在 GUI 執行緒
          呼叫（會卡住畫面最多上面那個總時間）。接收執行緒正在做同步交易（任務
          上傳 / 讀回）時，請求要等交易做完才送得出去 —— 那段時間也算在逾時裡。
        """
        if isinstance(names, (str, bytes)):
            raise TypeError('names 要是參數名稱的清單，不是單一字串')
        names = list(names)
        if not names:
            raise ValueError('names 是空的')
        for n in names:
            if not isinstance(n, str) or not n or '\x00' in n:
                raise ValueError(f'參數名稱不合法：{n!r}')
            try:
                nb = n.encode('ascii')
            except UnicodeEncodeError:
                raise ValueError(f'參數名稱必須是 ASCII：{n!r}') from None
            if len(nb) > 16:
                # MAVLink param_id 只有 16 位元組；送出去會被截斷成另一個名字，
                # 回覆永遠對不上 —— 直接拒收，不要讓它變成一個「讀不到」。
                raise ValueError(f'參數名稱超過 16 字元（MAVLink 上限）：{n!r}')
        timeout_s = float(timeout_s)
        if not (math.isfinite(timeout_s) and timeout_s > 0):
            raise ValueError(f'timeout_s 必須是正數：{timeout_s!r}')
        if isinstance(retries, bool) or int(retries) != retries or retries < 0:
            raise ValueError(f'retries 必須是 >= 0 的整數：{retries!r}')
        retries = int(retries)
        if threading.get_ident() == self._recv_thread_ident:
            raise RuntimeError('read_params 不可在 SITLLink 的接收執行緒裡呼叫（會自己等自己）')

        uniq = list(dict.fromkeys(names))
        if self._mav is None:
            logger.warning(f'[SITL] read_params：尚未連線，{len(uniq)} 個參數全部視為讀不到')
            return {n: None for n in names}

        waiter = {'want': set(uniq), 'got': {}, 'sysid': None, 'compid': None}
        with self._param_cv:
            self._param_waiters.append(waiter)
        rounds = 0
        try:
            for _attempt in range(retries + 1):
                with self._param_cv:
                    missing = [n for n in uniq if n not in waiter['got']]
                if not missing or self._stop or self._link_closed:
                    break
                rounds += 1
                self._cmd_queue.put(('read_params', (waiter, missing)))
                deadline = time.monotonic() + timeout_s
                with self._param_cv:
                    while not all(n in waiter['got'] for n in missing):
                        remain = deadline - time.monotonic()
                        if remain <= 0 or self._stop or self._link_closed:
                            break
                        self._param_cv.wait(min(remain, 0.25))
        finally:
            with self._param_cv:
                self._param_waiters.remove(waiter)
                got = dict(waiter['got'])
        lost = [n for n in uniq if n not in got]
        if lost:
            shown = ', '.join(lost[:8]) + ('…' if len(lost) > 8 else '')
            logger.warning(
                f'[SITL] read_params：{len(lost)}/{len(uniq)} 個參數讀不到'
                f'（{rounds} 輪 × {timeout_s:g} s，sysid={waiter["sysid"]}）：{shown}')
        if types_out is not None:
            for n, (_v, t) in got.items():
                types_out[n] = t
        return {n: (got[n][0] if n in got else None) for n in names}

    def get_components(self, sysid: Optional[int] = None) -> dict:
        """有發過心跳的元件：``{sysid: {compid: 資訊}}``；給 sysid 就只回 ``{compid: 資訊}``。

        回的是複本。資訊欄位見 _note_component；`autopilot_component=True` 的那一個
        才會更新機卡。
        """
        with self._state_lock:
            snap = {s: {c: dict(i) for c, i in comps.items()}
                    for s, comps in self._components.items()}
        if sysid is None:
            return snap
        return snap.get(int(sysid), {})

    def get_camera_stats(self, sysid: Optional[int] = None, cam_idx: int = 0) -> Optional[dict]:
        """某架（預設本鏈路 sysid_label）某相機的 CAMERA_FEEDBACK 統計複本；沒收過回 None。

        欄位意義見 _on_camera_feedback。
        """
        key = (self.sysid_label if sysid is None else int(sysid), int(cam_idx))
        with self._state_lock:
            st = self._cam_stats.get(key)
            if st is None:
                return None
            out = dict(st)
            out['gaps'] = list(st['gaps'])
        return out

    # ── tlog（opt-in）────────────────────────────────────────────────
    def start_tlog(self, path) -> str:
        """開始把**收到的** MAVLink 原始封包記成 tlog；預設不記、不寫任何檔案。

        格式是標準 .tlog（pymavlink / MAVProxy / Mission Planner 都讀得懂）：每則訊息 =
        8 位元組 big-endian uint64 主機時間（Unix epoch µs，低 2 位清零 —— 與 pymavlink
        mavfile.recv_msg 自己寫 tlog 的方式相同）＋ 該則訊息收到時的原始位元組。
        只記收到的，不記我方送出的。以 append 開檔（同路徑再開不會蓋掉舊資料）；
        已在記錄時換路徑會先關掉舊檔。回傳實際寫入的絕對路徑。

        記錄點是 pymavlink 的 message_hooks（連線建立時就裝好）⇒ 同步交易裡被
        recv_match(type=...) 濾掉的訊息也會記到。寫檔失敗會停止記錄並大聲報一次
        （status_text 嚴重度 3），不會把連線帶掉；狀態見 tlog_status()。
        """
        p = os.path.abspath(os.fspath(path))
        parent = os.path.dirname(p)
        if parent:
            os.makedirs(parent, exist_ok=True)
        fh = open(p, 'ab')
        with self._tlog_lock:
            old, old_path = self._tlog_file, self._tlog_path
            self._tlog_file, self._tlog_path = fh, p
            self._tlog_msgs = self._tlog_bytes = 0
            self._tlog_error = None
            self._tlog_last_flush = time.time()
        if old is not None:
            try:
                old.close()
            except Exception:
                pass
            logger.info(f'[SITL] tlog 換檔：{old_path} → {p}')
        logger.info(f'[SITL] tlog 開始記錄 (sysid={self.sysid_label})：{p}')
        return p

    def stop_tlog(self) -> dict:
        """停止 tlog 並關檔；回傳 tlog_status()（沒在記錄也可以呼叫）。"""
        with self._tlog_lock:
            fh = self._tlog_file
            self._tlog_file = None
            status = self._tlog_status_locked()
        if fh is not None:
            try:
                fh.close()
            except Exception as e:
                logger.error(f'[SITL] tlog 關檔失敗：{e}')
            logger.info(f'[SITL] tlog 停止：{status["path"]}'
                        f'（{status["messages"]} 則，{status["bytes"]} B）')
        return status

    def tlog_status(self) -> dict:
        """{'active', 'path', 'messages', 'bytes', 'error'} —— 實際生效的 tlog 狀態。"""
        with self._tlog_lock:
            return self._tlog_status_locked()

    def _tlog_status_locked(self) -> dict:
        return {'active': self._tlog_file is not None, 'path': self._tlog_path,
                'messages': self._tlog_msgs, 'bytes': self._tlog_bytes,
                'error': self._tlog_error}

    def _tlog_hook(self, _mavfile, msg) -> None:
        """pymavlink message_hooks：**每一則**解析出來的訊息都會經過這裡。

        ⚠️ 例外絕對不可以往外丟：hook 是在 recv_msg 裡被呼叫的，丟出去會變成主迴圈的
           「接收錯誤」把整條連線收掉。寫檔失敗 → 停止 tlog、大聲報一次。
        """
        if self._tlog_file is None:
            return
        try:
            if msg.get_type() == 'BAD_DATA':
                return
            buf = msg.get_msgbuf()
            if not buf:
                return
            rec = struct.pack('>Q', int(time.time() * 1.0e6) & ~3) + bytes(buf)
        except Exception:
            return
        err = None
        with self._tlog_lock:
            fh = self._tlog_file
            if fh is None:
                return
            try:
                fh.write(rec)
                self._tlog_msgs += 1
                self._tlog_bytes += len(rec)
                now = time.time()
                if now - self._tlog_last_flush >= 1.0:
                    fh.flush()
                    self._tlog_last_flush = now
            except Exception as e:
                err = f'{type(e).__name__}: {e}'
                self._tlog_error = err
                self._tlog_file = None
                try:
                    fh.close()
                except Exception:
                    pass
            path = self._tlog_path
        if err is not None:
            logger.error(f'[SITL] tlog 寫入失敗，已停止記錄（{path}）：{err}')
            self.status_text.emit(3, f'⚠️ tlog 寫入失敗，已停止記錄：{err}（{path}）')

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
        self._recv_thread_ident = threading.get_ident()
        self._link_closed = False
        try:
            self._run_link()
        finally:
            # 連線結束：叫醒還在 read_params 裡等的工作執行緒（別讓它們白等到逾時），
            # 並關掉 tlog（使用者有開才會有檔案）。
            self._link_closed = True
            self._recv_thread_ident = None
            with self._param_cv:
                self._param_cv.notify_all()
            if self._tlog_file is not None:
                self.stop_tlog()

    def _run_link(self):
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

        # tlog 記錄點：裝在 pymavlink 的 message_hooks 上（沒開 tlog 時 hook 立刻返回）。
        # 越早裝越好 —— 下面的 wait_heartbeat / Q_ENABLE 讀取也會被記到。
        _hooks = getattr(self._mav, 'message_hooks', None)
        if isinstance(_hooks, list):
            _hooks.append(self._tlog_hook)

        # ★ 先送一包 GCS heartbeat「破冰」再等對方 heartbeat。
        #   udpout（主動連嵌入式 mavproxy）必須先送，否則：
        #     - Windows udpout socket 未 bind → wait_heartbeat 的 recvfrom
        #       直接拋 WinError 10022；
        #     - 被動 udpin 對端不知我方位址 → 永遠不回送遙測 → 逾時。
        #   對 tcp / udpin / serial 連線而言這只是提早送一包心跳，無副作用。
        self._send_gcs_heartbeat()

        # 等 heartbeat —— 要等到**自駕儀元件**的心跳。
        # ⚠️ wait_heartbeat 不分 compid：同 sysid 的相機（compid 100）若比飛控先開口，
        #    舊版會拿它的 MAV_TYPE_CAMERA 當機型（機卡 TYPE_30），連下面的 Q_ENABLE
        #    驗證都因為「不是 PLANE」而被跳過。非自駕儀元件的心跳記進元件表，繼續等。
        #    （0 / 255 照舊直接收下 —— 下面本來就會跳過，不建卡。）
        hb = None
        _hb_deadline = time.time() + 10
        try:
            while True:
                _remain = _hb_deadline - time.time()
                if _remain <= 0:
                    break
                cand = self._mav.wait_heartbeat(timeout=_remain)
                if cand is None:
                    break
                c_src = int(cand.get_srcSystem())
                if c_src in (0, 255):
                    hb = cand
                    break
                c_comp = self._src_component(cand)
                if self._claim_autopilot(c_src, c_comp, cand):
                    hb = cand
                    break
                info = self._note_component(c_src, c_comp, cand, False)
                self.component_heartbeat.emit(c_src, c_comp, info)
        except Exception as e:
            self.error.emit(f'等待 heartbeat 失敗: {e}')
            return

        if hb is None:
            seen = self.get_components()
            if seen:
                parts = ', '.join(f'sys{s}/comp{c}={i["type_name"]}'
                                  for s, comps in seen.items() for c, i in comps.items())
                self.error.emit(f'10 秒內只收到非自駕儀元件的 heartbeat（{parts}）'
                                f' — 飛控本身沒有回應')
            else:
                self.error.emit('10 秒內未收到 heartbeat — 確認 SITL 已啟動')
            return

        primary_type = self._effective_vehicle_type(hb.type)
        hb_src = int(hb.get_srcSystem())
        # 預先建立第一個 heartbeat 來源的 frame（跳過我方 GCS 回送 255 / 無效 0，
        # 以及非飛機節點 —— wait_heartbeat 不分來源，接 WSL 蜂群時很可能先收到
        # MAVROS 的心跳，那會在主迴圈守衛生效之前就先建立一張假卡片）。
        if hb_src not in (0, 255):
            self._note_component(hb_src, self._src_component(hb), hb, True)
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
    # 非載具 MAV_TYPE 見模組層的 _NON_VEHICLE_TYPE_NAMES / _non_vehicle_types()。

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
            if int(msg.autopilot) == _mavlink_enums().MAV_AUTOPILOT_INVALID:
                return False
            if int(msg.type) in _non_vehicle_types():
                return False
        except Exception:
            return True          # 讀不到欄位就別亂丟資料
        return True

    # ── 元件（compid）分類：同一個 sysid 底下，誰是飛控 ─────────────────
    @staticmethod
    def _src_component(msg) -> int:
        """訊息的來源 compid。拿不到（測試替身等）就當成自駕儀 1 —— 與修正前的行為一致。"""
        get = getattr(msg, 'get_srcComponent', None)
        if not callable(get):
            return 1
        try:
            return int(get())
        except Exception:
            return 1

    def _claim_autopilot(self, src: int, comp: int, msg) -> bool:
        """這包 HEARTBEAT 是不是 sysid `src` 的**自駕儀元件**發的？是就登記下來。

          * compid 1（MAV_COMP_ID_AUTOPILOT1）一律算 —— ArduPilot / PX4 飛控都是 1。
          * 其他 compid：只有在這個 sysid 還沒有自駕儀、而且這包心跳本身像飛機
            （autopilot ≠ INVALID 且 type 不在非載具清單）時，**第一個**出現的才算。
          * 之後出現的其他 compid 一律不算 —— 相機（100）、雲台（154）、伴隨電腦
            都走這條，即使它們把 autopilot 欄位填成 ARDUPILOTMEGA。
        """
        ap1 = _mavlink_enums().MAV_COMP_ID_AUTOPILOT1
        known = self._autopilot_compid.get(src)
        if comp == ap1:
            if known is not None and known != ap1:
                logger.warning(f'[SITL] {self.conn_str} system {src}：compid {ap1} 出現，'
                               f'取代原先登記的自駕儀 compid {known}')
            self._autopilot_compid[src] = ap1
            return True
        if known is not None:
            return comp == known
        if self._heartbeat_is_vehicle(msg):
            self._autopilot_compid[src] = comp
            logger.info(f'[SITL] {self.conn_str} system {src}：以 compid {comp} 為自駕儀'
                        f'（type={getattr(msg, "type", "?")} autopilot={getattr(msg, "autopilot", "?")}）')
            return True
        return False

    def _is_autopilot_component(self, src: int, comp: int) -> bool:
        return comp == _mavlink_enums().MAV_COMP_ID_AUTOPILOT1 or \
            comp == self._autopilot_compid.get(src)

    def _note_component(self, src: int, comp: int, msg, is_autopilot: bool) -> dict:
        """把一包心跳記進元件表，回傳該元件資訊的複本：

        {'compid', 'type', 'type_name', 'autopilot', 'system_status',
         'autopilot_component'（True = 這個 compid 才會更新機卡）,
         'first_seen', 'last_seen'（主機 time.time()）, 'count'（收到幾包心跳）}
        """
        now = time.time()
        mtype = int(getattr(msg, 'type', -1))
        with self._state_lock:
            comps = self._components.setdefault(src, {})
            info = comps.get(comp)
            first = info is None
            if first:
                info = {'compid': comp, 'first_seen': now, 'count': 0}
                comps[comp] = info
            info.update(
                type=mtype, type_name=_mav_type_name(mtype),
                autopilot=int(getattr(msg, 'autopilot', -1)),
                system_status=int(getattr(msg, 'system_status', -1)),
                autopilot_component=bool(is_autopilot), last_seen=now)
            info['count'] += 1
            snap = dict(info)
        if first and not is_autopilot:
            logger.info(f'[SITL] {self.conn_str} system {src} 的元件 compid={comp}'
                        f'（{snap["type_name"]}）不是自駕儀 —— 另外記錄，不更新機卡')
        return snap

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
            try:
                self._on_param_value(msg)
            except Exception as e:
                logger.error(f'[SITL] PARAM_VALUE 處理失敗: {e}', exc_info=True)
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
        # ⚠️ 但「放行的是這個 sysid 的**自駕儀元件**」：同 sysid 的相機 / 伴隨電腦
        #    心跳先分流到元件表（見 __init__ 的 _components 說明），不碰機卡。
        if mtype == 'HEARTBEAT':
            comp = self._src_component(msg)
            is_ap = self._claim_autopilot(src, comp, msg)
            info = self._note_component(src, comp, msg, is_ap)
            if not is_ap:
                self.component_heartbeat.emit(src, comp, info)
                return
            if not self._heartbeat_is_vehicle(msg) and src != self.sysid_label:
                self._note_non_vehicle(src, msg)
                return
            self._vehicle_srcs.add(src)
        elif src not in self._vehicle_srcs and src != self.sysid_label:
            return
        elif not self._is_autopilot_component(src, self._src_component(msg)):
            return   # 同 sysid 非自駕儀元件送的遙測，一樣不寫進機卡

        if mtype == 'CAMERA_FEEDBACK':
            self._on_camera_feedback(src, msg)
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

    # ── PARAM_VALUE → param_value 信號 + read_params 等待者 ─────────────
    def _on_param_value(self, msg) -> None:
        raw = msg.param_id
        name = (raw if isinstance(raw, str) else bytes(raw).decode('utf-8', 'ignore'))
        name = name.split('\x00', 1)[0].strip()
        value = float(msg.param_value)
        ptype = int(getattr(msg, 'param_type', 0) or 0)     # 0 = 訊息沒帶型別
        pidx = int(getattr(msg, 'param_index', -1))
        pcount = int(getattr(msg, 'param_count', 0))
        get_src = getattr(msg, 'get_srcSystem', None)
        src = int(get_src()) if callable(get_src) else 0
        comp = self._src_component(msg)
        self.param_value.emit(src, name, value, ptype, pidx, pcount)
        with self._param_cv:
            hit = False
            for w in self._param_waiters:
                # sysid 是 None = 這一批請求還沒送出：之前到的值不算這次讀回
                if w['sysid'] is None or src != w['sysid'] or comp != w['compid']:
                    continue
                if name in w['want'] and name not in w['got']:
                    w['got'][name] = (value, ptype)
                    hit = True
            if hit:
                self._param_cv.notify_all()

    # ── CAMERA_FEEDBACK → 拍照計數 ──────────────────────────────────────
    def _on_camera_feedback(self, src: int, msg) -> None:
        """CAMERA_FEEDBACK：更新 (sysid, cam_idx) 的統計並 emit camera_feedback。

        ⚠️ **張數看 img_idx 的跳號，不看訊息則數。** ArduPilot 每條鏈路對
           CAMERA_FEEDBACK 只有一個待送位元＋一份會被覆寫的內容：鏈路忙時連拍幾張
           只會送出最後一則（img_idx 跳號）。另外同步交易（任務上傳 / 讀回）期間主迴圈
           收不到訊息，也會表現成跳號。所以：
             * triggered：飛控觸發計數（= 最新 img_idx；飛控重開機後接著累加）。
             * received / missing / gaps：有收到 / 沒收到回饋的 img_idx（gaps 是
               (第一個缺的, 最後一個缺的) 清單）。**缺回饋 ≠ 漏拍**：飛控照樣觸發了，
               只是那張的回饋訊息被合併或掉了；真正漏拍要落地後拿 .BIN CAM 與 Pi 的
               frames.jsonl 對。
             * completed：completed_captures（回授腳確認的張數）依跳號累加。沒設
               CAM1_FEEDBAK_PIN 時飛控永遠送 0。鏈路鎖在 MAVLink1 時這個擴充欄位
               也收不到（恆為 0）。有回授腳時同一個 img_idx 可能出現兩次（第二則帶著
               新的 completed_captures）⇒ duplicates 不是異常。
             * img_idx 倒退（且不是 65535→0 的回捲）= 飛控重開機：resets+1，
               新一輪從 0 起算。
           first_img_idx > 1 代表連上之前已經拍過（那些不算缺）。
        """
        cam = int(getattr(msg, 'cam_idx', 0))
        img = int(msg.img_idx) & 0xFFFF
        cc = int(getattr(msg, 'completed_captures', 0) or 0) & 0xFFFF
        now = time.time()
        gap = None
        reset = duplicate = False
        with self._state_lock:
            st = self._cam_stats.get((src, cam))
            if st is None:
                st = {'sysid': src, 'cam_idx': cam, 'messages': 0,
                      'first_img_idx': img, 'last_img_idx': img,
                      'triggered': img, 'received': 1, 'missing': 0, 'gaps': [],
                      'duplicates': 0, 'resets': 0,
                      'completed_captures': cc, 'completed': cc}
                self._cam_stats[(src, cam)] = st
                new = img
            else:
                d = (img - st['last_img_idx']) & 0xFFFF
                if d == 0:
                    duplicate = True
                    st['duplicates'] += 1
                    new = 0
                elif d < 0x8000:
                    new = d
                    st['received'] += 1
                    if d > 1:
                        gap = ((st['last_img_idx'] + 1) & 0xFFFF, (img - 1) & 0xFFFF)
                else:
                    reset = True
                    st['resets'] += 1
                    new = img
                    st['received'] += 1
                    if img > 1:
                        gap = (1, img - 1)
                st['triggered'] += new
                if gap is not None:
                    st['gaps'].append(gap)
                    st['missing'] += (gap[1] - gap[0]) % 0x10000 + 1
                dc = (cc - st['completed_captures']) & 0xFFFF
                if reset:
                    st['completed'] += cc
                elif dc < 0x8000:
                    st['completed'] += dc
                st['last_img_idx'] = img
                st['completed_captures'] = cc
            st['messages'] += 1
            st['last_time_usec'] = int(msg.time_usec)
            st['last_seen'] = now
            payload = {
                'sysid': src, 'compid': self._src_component(msg), 'cam_idx': cam,
                'img_idx': img,
                'lat': int(msg.lat) / 1e7, 'lng': int(msg.lng) / 1e7,    # 度
                'lat_e7': int(msg.lat), 'lng_e7': int(msg.lng),
                'alt_msl': float(msg.alt_msl), 'alt_rel': float(msg.alt_rel),   # m
                'roll': float(msg.roll), 'pitch': float(msg.pitch),
                'yaw': float(msg.yaw),                                   # 度
                'foc_len': float(getattr(msg, 'foc_len', 0.0)),
                'completed_captures': cc, 'flags': int(msg.flags),
                # ⚠️ time_usec 原樣轉交、不解讀：沒設 CAM1_FEEDBAK_PIN 時是外推的
                #    GPS epoch µs（gps.time_epoch_usec()），設了回授腳時是 32 位元
                #    開機 µs（約 71.6 分鐘回捲）。要用它得先知道那個參數。
                'time_usec': int(msg.time_usec),
                'new_photos': new, 'gap': gap, 'reset': reset, 'duplicate': duplicate,
                'triggered': st['triggered'], 'missing': st['missing'],
                'completed': st['completed'],
            }
        if gap is not None:
            logger.info(f'[SITL] sysid={src} cam{cam} CAMERA_FEEDBACK 跳號：'
                        f'img_idx {gap[0]}..{gap[1]} 沒收到回饋（合併或遺失；照樣計入觸發數）')
        if reset:
            logger.warning(f'[SITL] sysid={src} cam{cam} img_idx 倒退到 {img}'
                           f' —— 飛控重開機？觸發計數接著累加')
            self.status_text.emit(4, f'⚠️ UAV{src} 相機 img_idx 倒退到 {img}（飛控重開機？）')
        self.camera_feedback.emit(src, payload)

    # ─────────────────────────────────────────────────────────────────
    # 命令發送（在接收執行緒中執行，與接收共用 self._mav）
    # ─────────────────────────────────────────────────────────────────
    @staticmethod
    def _cmd_arg_summary(cmd: str, arg):
        """log 用：任務類命令只印點數，不要把整份清單倒進 log。"""
        if cmd == 'upload_mission':
            return f'{len(arg)} 點'
        if cmd == 'upload_mission_items':
            return f'{len(arg[0])} 項 verify={arg[1]}'
        if cmd == 'read_params':
            return f'{len(arg[1])} 個參數'
        return arg

    def _dispatch_command(self, cmd: str, arg):
        if self._mav is None:
            if cmd == 'upload_mission_items':
                # ⚠️ 不可以靜默 return：呼叫端在等 mission_uploaded，零次 = 永遠等。
                self._report_mission_upload(False, '尚未連線到飛控（沒有可用的 MAVLink 連線）')
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
                elif cmd == 'upload_mission_items':
                    self._send_mission_items(*arg)
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
                elif cmd == 'read_params':
                    self._send_param_read_requests(*arg)
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
                logger.info(f'[SITL] 已發送命令: {cmd} {self._cmd_arg_summary(cmd, arg)}')
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
        self._warn_if_auto_while_disarmed(mode_name)

    def _warn_if_auto_while_disarmed(self, mode_name: str) -> None:
        """未解鎖就切 AUTO：飛控會照樣印 `Mission: 2 VTOLTakeoff`，但**不會起飛**。

        2026-10-08 使用者回報「進 AUTO 顯示 vtol_takeoff 但飛機沒有起飛」。真正原因是
        ARMING_REQUIRE=0 讓飛機永遠「已解鎖」卻馬達鎖死；修好之後 VTOL 開機是未解鎖，
        只按 AUTO 會呈現**同樣的畫面**（訊息有、飛機不動）。所以在這裡明講：要先 ARM
        （或用「一鍵起飛」）。ArduPilot 在 AUTO 下再解鎖會立刻開始任務，不用再切一次。
        只警告、不擋：行為與 Mission Planner 一致。
        """
        if mode_name != 'AUTO':
            return
        f = self._frames.get(self.sysid_label)
        if f is not None and not f.armed:
            self.status_text.emit(
                4, '[System] ⚠ 尚未解鎖：AUTO 不會起飛。請先按 ARM（或用「一鍵起飛」）；'
                   'AUTO 下解鎖後會立刻開始任務。')

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

    def _send_param_read_requests(self, waiter: dict, names) -> None:
        """read_params 的送出端（接收執行緒、持 _mav_lock）。

        目標 = (target_system, 該 sysid 的自駕儀 compid)，**不是** target_component
        （pymavlink 預設 0 = 廣播，飛控會把讀參數請求轉發給同 sysid 的伴隨電腦）。
        送出前先在 waiter 記下「只收誰的回覆」—— 回覆也由這條執行緒處理，所以不會有
        回覆比記錄先到的競爭。每則間隔 20 ms：ArduPilot 的參數請求佇列只有 20 格，
        一次塞滿會被丟掉（丟掉的由 read_params 下一輪補送）。
        """
        tgt_sys = int(self._mav.target_system or 0) or self.sysid_label
        tgt_comp = self._autopilot_compid.get(
            tgt_sys, _mavlink_enums().MAV_COMP_ID_AUTOPILOT1)
        with self._param_cv:
            waiter['sysid'], waiter['compid'] = tgt_sys, tgt_comp
        pump = self._make_heartbeat_pump()
        for i, n in enumerate(names):
            if i:
                time.sleep(0.02)
            self._mav.mav.param_request_read_send(tgt_sys, tgt_comp, n.encode('ascii'), -1)
            pump()

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
        sysid = self.sysid_label

        items, err = self._fetch_mission_items()
        if err is not None:
            logger.warning(f'[SITL] 任務下載失敗: {err}')
            self.status_text.emit(3, f'任務下載失敗：{err}')
            self.mission_download_failed.emit(sysid, err)
            return

        # n == 0 是**合法**答案：飛控裡真的沒有任務。走成功路徑，讓 UI 顯示
        # 「0 個航點」而不是紅字 —— 那正是 auto_mission=False 時的正常樣子。
        if not items:
            self.status_text.emit(6, '飛控目前沒有任務（0 點）')
            self.mission_downloaded.emit(sysid, [])
            return

        logger.info(f'[SITL] 任務下載完成：{len(items)} 項 (sysid={sysid})')
        self.status_text.emit(6, f'任務下載完成（{len(items)} 點）')
        self.mission_downloaded.emit(sysid, items)

    def _make_heartbeat_pump(self):
        """回傳一個 `pump()`：距上次 >= 1 s 才補送一包 GCS heartbeat。

        ⚠️ 任務協定（下載 / 寫入 / 讀回驗證）都是在接收執行緒裡同步阻塞地做完，
           主迴圈自己的 1 Hz heartbeat（run() 裡 `_now - _last_hb >= 1.0`）
           在這段期間跑不到。航點多時累計阻塞可以到數十秒，而沒有 GCS 心跳
           會觸發 FS_GCS ⇒ 飛控自動 RTL。所以每個阻塞迴圈都要自己補送。
        """
        last_hb = time.time()

        def _pump():
            nonlocal last_hb
            if time.time() - last_hb >= 1.0:
                self._send_gcs_heartbeat()
                last_hb = time.time()

        return _pump

    def _fetch_mission_items(self):
        """讀一次飛控任務，回傳 `(items | None, error | None)`。

        **不 emit 任何信號、不寫 status_text** —— 呼叫端決定失敗怎麼呈現：
        `_recv_mission`（使用者按「讀取」）走 mission_download_failed，
        `_write_mission_items` 的讀回驗證則把它併進 mission_uploaded 的失敗訊息。
        兩條路共用同一份協定實作，才不會出現「驗證讀回」和「使用者讀取」
        各自有一份、其中一份默默過期的情況。

        `([], None)` = 飛控裡真的沒有任務（合法答案）；`(None, str)` = 讀不回來。
        """
        from pymavlink import mavutil
        tgt_sys, tgt_comp = self._mav.target_system, self._mav.target_component
        MT = mavutil.mavlink.MAV_MISSION_TYPE_MISSION

        # ⚠️ 整段期間主迴圈是停住的 —— 見 _make_heartbeat_pump。
        #    （上傳路徑有同樣的問題，只是它從沒跑過大任務。）
        _pump_heartbeat = self._make_heartbeat_pump()

        # 0. 清掉還留在接收緩衝裡的舊 MISSION_COUNT / MISSION_ITEM(_INT) / MISSION_ACK：
        #    _mission_msg_is_for_us() 只濾「哪一架飛控」，不濾「哪一筆交易」——
        #    同一架飛控、同一個 mission_type 的一則舊回覆（例如使用者上一次按了
        #    「讀取」但客戶端已經逾時放棄、或上一次讀回驗證被中斷）會跟這次全新
        #    請求的回覆長得一模一樣，若不先清空就會被當成**這次**的答案：讀取路徑
        #    會把過期任務當成飛控「目前」的任務顯示成功、零錯誤文字；寫入後的讀回
        #    驗證則可能拿它跟剛寫入的內容比對，把飛控已經接受的寫入誤判成「不符」，
        #    或反過來湊巧判成「相符」但其實沒有真的驗到飛控上現在的內容。
        #    寫入路徑（_write_mission_items）已經對自己用到的訊息種類做過同一件事
        #    （REQUEST/ACK，見那裡的註解），這裡是讀取路徑自己的一份，不能少。
        for _ in range(64):
            stale = self._mav.recv_match(
                type=['MISSION_COUNT', 'MISSION_ITEM_INT', 'MISSION_ITEM',
                      'MISSION_ACK'],
                blocking=False)
            if stale is None:
                break

        # 1. 要清單
        #    ⚠️ fan-in 拓樸（見 __init__ docstring／_frame_for）下同一條連線上可能
        #    同時有別架飛控的流量。recv_match(type=...) 只濾訊息種類、不濾來源，
        #    這裡要另外用 _mission_msg_is_for_us() 擋掉別架的 MISSION_COUNT，
        #    否則會把別台飛控剛好在同時間送出的任務清單，讀成「我方」讀回結果
        #    ——包括寫入路徑拿它做讀回驗證，會把別架的任務誤判成「已驗證相符」。
        self._mav.mav.mission_request_list_send(tgt_sys, tgt_comp, MT)
        cnt = None
        _cnt_deadline = time.time() + 5
        while True:
            _remain = _cnt_deadline - time.time()
            if _remain <= 0:
                break
            cnt = self._mav.recv_match(
                type='MISSION_COUNT', blocking=True, timeout=_remain)
            if cnt is None:
                break
            if self._mission_msg_is_for_us(cnt, tgt_sys, MT):
                break
            cnt = None  # 別架飛控的 MISSION_COUNT：不算數，繼續等
        if cnt is None:
            return None, '等 MISSION_COUNT 逾時（5 s）'
        n = int(cnt.count)

        if n == 0:
            return [], None

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
                # ⚠️ 同一條 fan-in 連線上別架飛控剛好也在傳任務時，訊息種類會
                #    一樣（MISSION_ITEM_INT），但來源不是我們要讀的那架 ——
                #    一樣要擋，否則讀回會混進別台飛控的座標。
                if not self._mission_msg_is_for_us(msg, tgt_sys, MT):
                    continue
                # ⚠️ 飛控可能回別的 seq（重送、亂序）。「收到就算數」會讓航線
                #    默默錯位 —— 產出一張畫得出來、但**是錯的**地圖，而那比
                #    畫不出來更糟：它不會有人去查。
                if int(msg.seq) != seq:
                    continue
                got = msg
                break
            if got is None:
                return None, f'seq {seq} 讀不回來（重試 3 次；共 {n} 項）'

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

        return items, None

    # ── 任務寫入（Mission Planner 的「Write WPs」）─────────────────────
    #
    # ArduPilot 端的協定（唯讀查證於 ~/ardupilot-airframe，下面每條都有出處）：
    #
    # ① 收到 MISSION_COUNT 就**自己截斷**舊任務、然後逐項向我們要
    #    （MissionItemProtocol_Waypoints.cpp::truncate → AP_Mission::truncate(count)）。
    #    所以**不可以**先送 MISSION_CLEAR_ALL：
    #      * 沒有好處 —— 舊任務本來就會被 COUNT 截成新長度；
    #      * 有壞處 —— AP_Mission::clear() 在「已解鎖且任務執行中」回 false，
    #        MissionItemProtocol::handle_mission_clear_all 會回 ERROR，
    #        於是「飛行中改航線」在還沒開始寫之前就先失敗；
    #      * 且 clear 與 COUNT 之間飛控是空任務，那個空窗裡切 AUTO 會直接完成任務。
    #    （舊路徑 _send_mission 有 clear，那是舊行為，這裡刻意不沿用。）
    # ② **請求永遠是 MISSION_REQUEST（非 INT）**：MissionItemProtocol::
    #    queued_request_send() 只送 mavlink_msg_mission_request_send
    #    （MissionItemProtocol.cpp）。所以迴圈兩種都要收；我們回的一律是
    #    MISSION_ITEM_INT（GCS_MAVLINK::handle_mission_item 兩種都收，
    #    收 MISSION_ITEM 會多印一行 "GCS should send MISSION_ITEM_INT" 警告）。
    # ③ 飛控自己每 1 s 重送一次請求（MissionItemProtocol::update():
    #    wp_recv_timeout_ms = 1000 + slowdown），連續 8 s 沒收到項目就取消並送
    #    ACK(OPERATION_CANCELLED)（upload_timeout_ms = 8000）。
    #    ⇒ 封包遺失時「同一個 seq 被再問一次」是**正常**的，重答即可；
    #      項目遺失由飛控重問驅動，客戶端不必自己重送項目；只有 COUNT 本身
    #      遺失時飛控尚未開始，才需要我們重送 COUNT。
    # ④ 重複送項目的副作用：MissionItemProtocol::handle_mission_item 對
    #    seq != request_i 回 ACK(INVALID_SEQUENCE) 但**不中止**傳輸（receiving 維持
    #    true）。所以「我方剛好重答過某 seq」之後收到的 INVALID_SEQUENCE 是我方
    #    造成的良性噪音；沒有重答過卻收到，才是真的錯。
    # ⑤ seq 0（Home）：AP_Mission::replace_cmd(0) 在任務非空時**直接回 true 而不寫入**
    #    （"Writing index zero is not allowed, it must be home"）。飛控端 Home 由
    #    自己的 ahrs 決定 ⇒ 我們送的 seq 0 內容不會被採用，但仍會先過
    #    mavlink_int_to_mission_cmd 的檢查（座標/frame 不合法會被 NAK），所以
    #    seq 0 必須是合法項目。讀回時 seq 0 是飛控的 Home ⇒ 驗證跳過 seq 0。
    #    另：空任務時 add_cmd() 會先寫 Home 再把我們的 seq 0 放到 index 1，
    #    之後被 seq 1 的 replace 蓋掉 ⇒ **只有 seq 0 一項**的任務在空飛控上會讀回 2 項，
    #    所以寫入要求 >= 2 項（Home + 至少一個航點）。
    # ⑥ `current` 欄位：GCS_MAVLINK::handle_mission_item 對 MISSION_TYPE_MISSION 的項目，
    #    current == 2 是「GUIDED 單點」、current == 3 是「只改高度」—— 不入任務、
    #    立刻 ACK，**即使在上傳途中**！所以 current 一律送 0：
    #    檔案裡 QGC WPL 的 current=1（Home 行常見）在 mavlink_int_to_mission_cmd
    #    裡不被讀取，沒有任何好處，而 2/3 會劫持整個傳輸。
    #    讀回時 current=1 只表示「這是目前執行中的 nav 指令」
    #    （MissionItemProtocol_Waypoints::get_item），與檔案內容無關 ⇒ 驗證不比對 current。
    # ⑦ 讀回的 frame：見 _MISSION_LOCATION_FRAME_READBACK。非位置指令的
    #    frame/lat/lon/alt 飛控不存（stored_in_location），讀回是 0 ⇒ 不比對。
    # ⑧ MISSION_ACK.type 一律用 pymavlink 的列舉解碼（_mission_result_name）。
    #    ArduPilot 在寫入路徑上實際會回：ACCEPTED、NO_SPACE（count > num_commands_max）、
    #    INVALID_SEQUENCE、UNSUPPORTED_FRAME / UNSUPPORTED（指令或 frame 不認得）、
    #    INVALID_PARAM1..7（NaN/Inf、座標超範圍）、DENIED（別的 GCS 正在傳）、
    #    ERROR（DO_JUMP 目標不合法、儲存失敗）、OPERATION_CANCELLED（8 s 逾時）。
    # ⑨ 飛行中改任務：AP_Mission::replace_cmd 的註解 "replacing the current active
    #    command will have no effect until the command is restarted" —— 寫入**不會**
    #    停止或重啟正在跑的 AUTO 任務，但內容會被換掉。這裡不替呼叫端做這個決定
    #    （呼叫端應在 UI 擋 AUTO 中寫入）。
    @staticmethod
    def _preflight_mission_items(items):
        """寫入前的本機檢查，回傳 `(規範化後的 item 清單 | None, 失敗原因 | None)`。

        ⚠️ 全部在碰 socket **之前**做完：壞資料一旦開始傳，飛控端已經被 COUNT
           截斷了舊任務，傳到一半失敗 = 飛控上是一份半新半舊的任務。

        規則：非空且 >= 2 項（見上面 ⑤）；seq 必須恰為 0..n-1 各一次（順序亂但
        齊全就依 seq 排序，不重編號；缺號 / 重複 / 超出一律失敗）；每個數值欄位
        有限；command 在 [2, 65535]（0/1 是 ArduPilot 內部保留，會被 NAK）；
        frame 在 [0, 255]；|lat| <= 90、|lon| <= 180。
        """
        if not isinstance(items, (list, tuple)) or len(items) == 0:
            return None, '任務是空的（沒有任何項目可寫）'

        def _int_field(it, key, lo, hi, pos):
            v = it.get(key)
            if isinstance(v, bool) or v is None:
                raise ValueError(f'第 {pos} 項的 {key} 不是整數（{v!r}）')
            try:
                f = float(v)
            except (TypeError, ValueError):
                raise ValueError(f'第 {pos} 項的 {key} 不是整數（{v!r}）')
            if not math.isfinite(f) or f != int(f):
                raise ValueError(f'第 {pos} 項的 {key} 不是整數（{v!r}）')
            iv = int(f)
            if not lo <= iv <= hi:
                raise ValueError(f'第 {pos} 項的 {key}={iv} 超出 {lo}..{hi}')
            return iv

        def _num_field(it, key, pos):
            v = it.get(key)
            if isinstance(v, bool) or v is None:
                raise ValueError(f'第 {pos} 項的 {key} 缺漏或不是數字（{v!r}）')
            try:
                f = float(v)
            except (TypeError, ValueError):
                raise ValueError(f'第 {pos} 項的 {key} 缺漏或不是數字（{v!r}）')
            if not math.isfinite(f):
                raise ValueError(f'第 {pos} 項的 {key} 不是有限數（{v!r}）')
            return f

        norm = []
        try:
            for pos, it in enumerate(items):
                if not isinstance(it, dict):
                    raise ValueError(f'第 {pos} 項不是 dict')
                seq = _int_field(it, 'seq', 0, 65534, pos)
                cmd = _int_field(it, 'command', 2, 65535, pos)
                frame = _int_field(it, 'frame', 0, 255, pos)
                lat = _num_field(it, 'lat', pos)
                lon = _num_field(it, 'lon', pos)
                alt = _num_field(it, 'alt', pos)
                p = [_num_field(it, f'param{i}', pos) for i in (1, 2, 3, 4)]
                if abs(lat) > 90.0:
                    raise ValueError(f'第 {pos} 項 lat={lat} 超出 ±90')
                if abs(lon) > 180.0:
                    raise ValueError(f'第 {pos} 項 lon={lon} 超出 ±180')
                ac = it.get('autocontinue', 1)
                ac = 1 if ac is None else _int_field({'autocontinue': ac},
                                                     'autocontinue', 0, 255, pos)
                norm.append({
                    'seq': seq, 'command': cmd, 'frame': frame,
                    'lat': lat, 'lon': lon, 'alt': alt,
                    'param1': p[0], 'param2': p[1], 'param3': p[2], 'param4': p[3],
                    'autocontinue': ac,
                })
        except ValueError as e:
            return None, str(e)

        seqs = sorted(x['seq'] for x in norm)
        if seqs != list(range(len(norm))):
            seen, dup = set(), []
            for s in seqs:
                if s in seen:
                    dup.append(s)
                seen.add(s)
            missing = [s for s in range(len(norm)) if s not in seen]
            extra = [s for s in seqs if s >= len(norm)]
            return None, (f'seq 必須恰為 0..{len(norm) - 1} 各一次'
                          f'（重複 {dup[:5]}、缺 {missing[:5]}、超出 {extra[:5]}）')
        norm.sort(key=lambda x: x['seq'])
        if len(norm) < 2:
            return None, ('任務只有 Home（seq 0）一項、沒有任何航點；'
                          'ArduPilot 在空任務上寫入單一 seq 0 會多出一筆而讀回不符')
        return norm, None

    def _mission_msg_is_for_us(self, msg, tgt_sys: int, mission_type: int) -> bool:
        """過濾同一條鏈路上不屬於這筆交易的訊息（別架的、fence / rally 的）。"""
        mt = getattr(msg, 'mission_type', mission_type)
        if mt is not None and int(mt) != int(mission_type):
            return False
        get_src = getattr(msg, 'get_srcSystem', None)
        if tgt_sys and callable(get_src):
            try:
                if int(get_src()) != int(tgt_sys):
                    return False
            except Exception:
                pass
        return True

    def _write_mission_items(self, items, verify: bool):
        """做一整筆「寫入任務」交易，回傳 `(ok, 訊息)`。**自己不 emit 任何信號**
        （由 `_send_mission_items` 統一 emit 一次，見那裡的說明）。

        流程：本機檢查 → MISSION_COUNT → **事件迴圈**回答 MISSION_REQUEST(_INT)
        → 等 MISSION_ACK(ACCEPTED) →（verify）讀回逐項比對。

        ⚠️ 不是 `for seq in range(n)`：那種寫法假設飛控恰好一次一個、依序來。
           實際上請求會重複（封包遺失）、可能是 INT 或非 INT、ACK 可能提早到，
           `for` 迴圈遇到任何一種都會 IndexError、少送、或把「被拒收」當成功。

        seq 0 / current 的規則見上方 ⑤⑥：seq 0 照送（飛控不採用其內容，但要合法），
        current 一律送 0。
        """
        from pymavlink import mavutil
        norm, err = self._preflight_mission_items(items)
        if err is not None:
            return False, f'寫入前檢查失敗：{err}（尚未送出任何資料）'
        n = len(norm)
        by_seq = {x['seq']: x for x in norm}
        tgt_sys, tgt_comp = self._mav.target_system, self._mav.target_component
        MT = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        ACCEPTED = mavutil.mavlink.MAV_MISSION_ACCEPTED
        INVALID_SEQ = mavutil.mavlink.MAV_MISSION_INVALID_SEQUENCE
        pump = self._make_heartbeat_pump()

        # 清掉還留在接收緩衝裡的舊 REQUEST / ACK：例如上一次讀取留下的
        # ACK(ACCEPTED) 若被當成這次的 ACK，會在還沒送任何項目時就宣告成功。
        for _ in range(64):
            stale = self._mav.recv_match(
                type=['MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK'],
                blocking=False)
            if stale is None:
                break

        t0 = time.time()
        # 整體期限隨項數放大：每項一來一回，慢速數傳（57600 baud）約 0.2~0.5 s。
        deadline = t0 + 15.0 + 1.5 * n
        count_retry_s, max_count_resends = 2.0, 3

        self._mav.mav.mission_count_send(tgt_sys, tgt_comp, n, MT)
        last_count_t = time.time()
        count_resends = 0
        answered = {}           # seq → 已回答次數
        dup_answers = 0         # 我方重複回答的次數（見 ④）
        tolerated_inv_seq = 0
        unexplained_inv_seq = 0  # 見下方「未解釋的 INVALID_SEQUENCE」

        while True:
            now = time.time()
            if now >= deadline:
                # 見下方：一筆「未解釋」的 INVALID_SEQUENCE 不會立刻判定失敗，
                # 而是留到這裡 —— 逾時訊息要把它帶出來，不能讓失敗看起來
                # 只是單純沒回應（那樣使用者看不出跟 INVALID_SEQUENCE 有關）。
                inv_note = (
                    f'；期間收到過 {unexplained_inv_seq} 次未解釋的 '
                    'INVALID_SEQUENCE（飛控之後沒有再繼續問下一項，判定為真實'
                    '失敗，不是純傳輸層重複）'
                ) if unexplained_inv_seq else ''
                if not answered:
                    return False, (f'飛控無回應：{count_resends} 次重送 MISSION_COUNT 後'
                                   f'仍沒有任何請求（{deadline - t0:.0f} s）{inv_note}')
                return False, (f'逾時：已回答 {len(answered)}/{n} 項的請求，'
                               f'但 {deadline - t0:.0f} s 內沒有收到 ACK（未驗證＝失敗）'
                               f'{inv_note}')
            pump()
            msg = self._mav.recv_match(
                type=['MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK'],
                blocking=True, timeout=min(1.0, max(0.05, deadline - now)))
            if msg is None:
                # 第一個請求都還沒來 ⇒ 飛控可能沒收到 COUNT（飛控自己會重問項目，
                # 但不會重問「還沒開始的」傳輸），所以只在這個階段重送 COUNT。
                if not answered and time.time() - last_count_t >= count_retry_s:
                    if count_resends >= max_count_resends:
                        return False, (f'飛控無回應：MISSION_COUNT 送了 {count_resends + 1} 次'
                                       f'都沒有請求')
                    count_resends += 1
                    logger.warning(f'[SITL] 寫入任務：第 {count_resends} 次重送 MISSION_COUNT')
                    self._mav.mav.mission_count_send(tgt_sys, tgt_comp, n, MT)
                    last_count_t = time.time()
                continue
            if not self._mission_msg_is_for_us(msg, tgt_sys, MT):
                continue

            mtype = msg.get_type()
            if mtype == 'MISSION_ACK':
                res = int(msg.type)
                if res == ACCEPTED:
                    if len(answered) < n:
                        # 「提早 ACK」= 飛控認為傳完了，但我們還沒送完 ⇒ 不是成功。
                        return False, (f'飛控在只索取 {len(answered)}/{n} 項時就回了 ACK'
                                       f'（不可信，視為失敗）')
                    break
                if res == INVALID_SEQ and tolerated_inv_seq < dup_answers:
                    tolerated_inv_seq += 1
                    logger.warning('[SITL] 寫入任務：收到 INVALID_SEQUENCE，'
                                   '對應我方重複回答的項目（見 ④），繼續等')
                    continue
                if res == INVALID_SEQ and answered:
                    # ⚠️ ArduPilot 對 INVALID_SEQUENCE 的協定語意本來就是
                    # 「這筆資料是舊的，我略過，但傳輸沒中止」
                    # （MissionItemProtocol::handle_mission_item：seq!=request_i
                    # 只回 ACK 就 return，不動 receiving/link；update() 仍會在
                    # ~1s 內重新要 request_i，直到 upload_timeout_ms 才真的放棄）
                    # ——不論這筆重複是「我方自己重送」（上面 dup_answers 那支）
                    # 還是「傳輸層本身把我方送出的同一筆複製了一份」（UDP、或本
                    # GCS 鎖定的半雙工數傳鏈路常見），飛控端反應完全一樣，客戶端
                    # 也天生無法區分兩者。所以已經送出至少一項之後收到的
                    # INVALID_SEQUENCE，不再立刻判失敗，而是繼續等：
                    #   * 飛控自己沒事 ⇒ 之後還會再問下一項，正常往下走；
                    #   * 飛控真的壞了/拒收 ⇒ 之後不會再有任何 REQUEST，
                    #     最終在上面的 deadline 逾時失敗（訊息仍帶著這次的
                    #     INVALID_SEQUENCE，不會被吃掉、不會看起來像成功）。
                    # 一項都還沒送出時收到的 INVALID_SEQUENCE 沒有「複製了哪一筆」
                    # 可以解釋，維持原本「立刻判定為真實錯誤」（見下面）。
                    unexplained_inv_seq += 1
                    logger.warning('[SITL] 寫入任務：收到未解釋的 INVALID_SEQUENCE'
                                   f'（已回答 {len(answered)}/{n} 項），繼續等飛控'
                                   '是否自行恢復，而非立刻判定失敗')
                    continue
                name = self._mission_result_name(res)
                return False, (f'飛控拒收任務：{name}'
                               f'（已回答 {len(answered)}/{n} 項）')

            # MISSION_REQUEST / MISSION_REQUEST_INT
            seq = int(msg.seq)
            if seq not in by_seq:
                return False, f'飛控請求了範圍外的 seq {seq}（任務共 {n} 項，合法 0..{n - 1}）'
            if answered.get(seq, 0) >= 1:
                dup_answers += 1
            answered[seq] = answered.get(seq, 0) + 1
            it = by_seq[seq]
            # frame 逐項照檔案原樣送（不猜、不改寫）；current 一律 0（見 ⑥）；
            # 座標 ×1e7 用 round()：int() 會截斷，
            # 例如 25.0777*1e7 = 250776999.99999997 → int 得 250776999 而不是 250777000。
            self._mav.mav.mission_item_int_send(
                tgt_sys, tgt_comp, seq, it['frame'], it['command'],
                0, it['autocontinue'],
                it['param1'], it['param2'], it['param3'], it['param4'],
                int(round(it['lat'] * 1e7)), int(round(it['lon'] * 1e7)),
                float(it['alt']), MT)

        if not verify:
            return True, f'已寫入 {n} 項（飛控 ACK 接受；未讀回驗證）'

        got, rerr = self._fetch_mission_items()
        if rerr is not None:
            return False, f'飛控已 ACK 接受，但讀回驗證失敗：{rerr}（無法確認內容）'
        diffs = self._mission_readback_diffs(norm, got)
        if diffs:
            shown = '；'.join(diffs[:5])
            more = f'（共 {len(diffs)} 處，僅列前 5）' if len(diffs) > 5 else ''
            return False, f'讀回驗證不符：{shown}{more}'
        return True, f'已寫入並讀回驗證 {n} 項相符'

    @staticmethod
    def _mission_param_matches(cmd: int, idx: int, sent: float, got: float) -> bool:
        if idx in _MISSION_PARAMS_NOT_ECHOED.get(cmd, ()):
            return True
        if not math.isfinite(got):
            return False
        if abs(got - sent) <= 1e-6 * max(1.0, abs(sent)):
            return True
        if cmd not in _MISSION_CMDS_LOSSY_PARAMS:
            return False
        if got == 0.0:                       # 該欄位飛控不存
            return True
        if abs(got - sent) <= max(1.0, 0.1 * abs(sent)):   # 整數截斷 / 定點
            return True
        if sent > got > 0 and got in (255.0, 65535.0):     # uint8 / uint16 夾值
            return True
        return False

    @staticmethod
    def _mission_readback_diffs(sent, got):
        """逐項比對「送出」與「讀回」，回傳人看得懂的差異字串清單（空 = 相符）。

        規則全部來自 ArduPilot 原始碼（見表的註解）：
          * 筆數必須相同；
          * seq 0 跳過（飛控自己的 Home，本來就會不同）；
          * command 必須完全相同；
          * 位置類指令：lat/lon 差 <= 1 格（1e-7°；x 是 int32×1e7）、alt 差 <= 1.1 cm 加
            float32 誤差（飛控以 cm 的 int32 存：`alt = z*100.0f` 截斷）、
            frame 依正規化表；
          * 非位置類指令：lat/lon/alt/frame 飛控不存，不比；
          * param1..4：見 _mission_param_matches。
        `current` 不比（讀回的 1 只表示「正在執行」）。
        """
        diffs = []
        if len(got) != len(sent):
            diffs.append(f'項數：送出 {len(sent)}、讀回 {len(got)}')
        got_by_seq = {}
        for g in got:
            got_by_seq[int(g['seq'])] = g
        for s in sent:
            seq = int(s['seq'])
            if seq == 0:
                continue
            g = got_by_seq.get(seq)
            if g is None:
                diffs.append(f'seq {seq} 讀回缺漏')
                continue
            cmd = int(s['command'])
            if int(g['command']) != cmd:
                diffs.append(f'seq {seq} command（送 {cmd}、讀回 {int(g["command"])}）')
                continue
            if cmd in _MISSION_CMDS_STORED_IN_LOCATION:
                sf = int(s['frame'])
                exp_frame = _MISSION_LOCATION_FRAME_READBACK.get(sf, sf)
                if int(g['frame']) != exp_frame:
                    diffs.append(f'seq {seq} frame（送 {sf}、預期讀回 {exp_frame}、'
                                 f'實際 {int(g["frame"])}）')
                for key in ('lat', 'lon'):
                    # 以 x = int32×1e7 的整數格比：差 1 格（1e-7°）以內算相符。
                    # 直接比浮點 `> 1e-7` 會在剛好差 1 格時被浮點誤差誤判。
                    if abs(round(float(g[key]) * 1e7) - round(float(s[key]) * 1e7)) > 1:
                        diffs.append(f'seq {seq} {key}（送 {s[key]}、讀回 {g[key]}）')
                tol = 0.011 + 2.4e-7 * abs(float(s['alt']))
                if abs(float(g['alt']) - float(s['alt'])) > tol:
                    diffs.append(f'seq {seq} alt（送 {s["alt"]}、讀回 {g["alt"]}）')
            for i in (1, 2, 3, 4):
                key = f'param{i}'
                if not SITLLink._mission_param_matches(
                        cmd, i, float(s[key]), float(g[key])):
                    diffs.append(f'seq {seq} {key}（送 {s[key]}、讀回 {g[key]}）')
        return diffs

    def _report_mission_upload(self, ok: bool, message: str) -> None:
        """**唯一**發出 mission_uploaded 的地方。

        「每次 upload_mission_items() 恰好一次」靠結構保證，而不是靠各路徑自己
        記得：`_write_mission_items` 只回傳 (ok, msg)、不碰信號，`_send_mission_items`
        呼叫這個函式**一次**；例外也在同一個 try/except 裡轉成 (False, msg)。
        零次 = UI 永遠停在「寫入中…」；兩次 = 先顯示成功又改成失敗（或反過來）。
        """
        sysid = self.sysid_label
        if ok:
            logger.info(f'[SITL] 任務寫入成功 (sysid={sysid}): {message}')
            self.status_text.emit(6, f'任務寫入完成：{message}')
        else:
            logger.error(f'[SITL] 任務寫入失敗 (sysid={sysid}): {message}')
            self.status_text.emit(3, f'⚠️ 任務寫入失敗：{message}')
        self.mission_uploaded.emit(sysid, bool(ok), str(message))

    def _send_mission_items(self, items, verify: bool = True):
        """`_dispatch_command('upload_mission_items')` 的落點：跑交易、恰好 emit 一次。

        在接收執行緒裡、持著 `_mav_lock`（由 _dispatch_command 取得，with 區塊
        離開時一定釋放，任何路徑都不會留著鎖）。
        """
        try:
            ok, message = self._write_mission_items(items, bool(verify))
        except Exception as e:
            logger.error(f'[SITL] 任務寫入例外: {e}', exc_info=True)
            ok, message = False, f'寫入過程發生例外：{type(e).__name__}: {e}'
        self._report_mission_upload(ok, message)
