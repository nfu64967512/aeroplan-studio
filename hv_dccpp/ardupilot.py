"""ardupilot.py — HV-DCCPP `UAVPlan` → ArduPilot QuadPlane（ArduPlane + Q_ENABLE）任務輸出（純輸出層）。

依 `scratchpad/ardupilot_notes.md` 與 `integration_notes.md`：
* 任務項目（MAV_MISSION_TYPE_MISSION）：NAV_VTOL_TAKEOFF(84) → NAV_WAYPOINT(16)／NAV_LOITER_TO_ALT(31)／
  NAV_LOITER_TURNS(18)／NAV_LOITER_TIME(19) → DO_LAND_START(189) → NAV_VTOL_LAND(85)；
  DO_CHANGE_SPEED(178) 於空速改變處；作業條帶以 DO_SET_CAM_TRIGG_DIST(206) dist／0 包住（rgb），
  LiDAR 以 DO_SET_SERVO(183) 佔位（見 :data:`LIDAR_SERVO_CHANNEL`）。
* 弧段（`Waypoint.arc`）以外切多邊形重新離散（規格 §8.5、論文 Eq.(28)(29)、D27：
  Δψ = min(2·acos(R/(R+e_max)), ψ_max)），直線段只留端點（高度剖面以 Douglas–Peucker 保留 > `delta_prof_m` 之轉折），
  並丟棄距前一航點小於接受半徑 r_acc = min(R_eff/2, 30) 之非必要點。
* 高度框架：`relative`（MAV_FRAME_GLOBAL_RELATIVE_ALT=3，alt = MSL − home MSL）、`msl`（MAV_FRAME_GLOBAL=0）、
  `terrain`（MAV_FRAME_GLOBAL_TERRAIN_ALT=10，alt = AGL；需 TERRAIN_ENABLE=1）。
* 第 0 列 home（frame 0、cmd 16、current 1）；QGC WPL 110（12 欄 tab 分隔，lat/lon 7 位小數）；
  rally（NAV_RALLY_POINT 5100）、fence（5000/5001/5002）、Mission Planner `.param`、QGC `.plan` JSON。

本模組只依賴 numpy／shapely 與 hv_dccpp 之 mission/context/platform/dubins；pymavlink 為**可選**（僅用於常數自檢，
未安裝時使用本檔內建之同值常數）。所有 MAV_CMD 常數以 `mavutil.mavlink.MAV_CMD_*` 取得（不手寫數字）。
"""
from __future__ import annotations

import json
import logging
import math
import os
import re
from dataclasses import dataclass, asdict
from typing import Any, Iterable, Literal, Mapping, Sequence

import numpy as np
import shapely
from shapely.geometry import MultiPolygon, Polygon
from shapely.ops import unary_union

from .context import MissionContext
from .decompose import d_turn_from_r
from .dubins import discretize_step_rad
from .geometry import is_cross_strip
from .mission import FW_PHASES, MissionInputs, MissionPlan, Mode, Phase, PlannerConfig, UAVPlan, Waypoint
from .platform import EnergyModel, PlatformSpec, Sensor

os.environ.setdefault("MAVLINK20", "1")

_log = logging.getLogger(__name__)

try:  # pragma: no cover - 依環境
    from pymavlink import mavutil as _mavutil
    _M = _mavutil.mavlink
    HAVE_PYMAVLINK = True
except Exception:  # noqa: BLE001
    _M = None
    HAVE_PYMAVLINK = False


def _const(name: str, fallback: int) -> int:
    """`mavutil.mavlink.<name>`；pymavlink 不可用時用 fallback（值已以 pymavlink 2.4.49 驗證）。"""
    if _M is not None and hasattr(_M, name):
        return int(getattr(_M, name))          # 常數不一致時以 pymavlink 為準
    return fallback


# ---------------------------------------------------------------------------
# MAVLink 常數（值皆自 pymavlink 取得；fallback 僅供無 pymavlink 之環境）
# ---------------------------------------------------------------------------
MAV_CMD_NAV_WAYPOINT = _const("MAV_CMD_NAV_WAYPOINT", 16)
MAV_CMD_NAV_DELAY = _const("MAV_CMD_NAV_DELAY", 93)
MAV_CMD_NAV_LOITER_TURNS = _const("MAV_CMD_NAV_LOITER_TURNS", 18)
MAV_CMD_NAV_LOITER_TIME = _const("MAV_CMD_NAV_LOITER_TIME", 19)
MAV_CMD_NAV_RETURN_TO_LAUNCH = _const("MAV_CMD_NAV_RETURN_TO_LAUNCH", 20)
MAV_CMD_NAV_LOITER_TO_ALT = _const("MAV_CMD_NAV_LOITER_TO_ALT", 31)
MAV_CMD_NAV_VTOL_TAKEOFF = _const("MAV_CMD_NAV_VTOL_TAKEOFF", 84)
MAV_CMD_NAV_VTOL_LAND = _const("MAV_CMD_NAV_VTOL_LAND", 85)
MAV_CMD_DO_CHANGE_SPEED = _const("MAV_CMD_DO_CHANGE_SPEED", 178)
MAV_CMD_DO_SET_SERVO = _const("MAV_CMD_DO_SET_SERVO", 183)
MAV_CMD_DO_LAND_START = _const("MAV_CMD_DO_LAND_START", 189)
MAV_CMD_DO_SET_CAM_TRIGG_DIST = _const("MAV_CMD_DO_SET_CAM_TRIGG_DIST", 206)
MAV_CMD_DO_VTOL_TRANSITION = _const("MAV_CMD_DO_VTOL_TRANSITION", 3000)
MAV_CMD_NAV_FENCE_RETURN_POINT = _const("MAV_CMD_NAV_FENCE_RETURN_POINT", 5000)
MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = _const("MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION", 5001)
MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = _const("MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION", 5002)
MAV_CMD_NAV_RALLY_POINT = _const("MAV_CMD_NAV_RALLY_POINT", 5100)

MAV_FRAME_GLOBAL = _const("MAV_FRAME_GLOBAL", 0)
MAV_FRAME_GLOBAL_RELATIVE_ALT = _const("MAV_FRAME_GLOBAL_RELATIVE_ALT", 3)
MAV_FRAME_GLOBAL_TERRAIN_ALT = _const("MAV_FRAME_GLOBAL_TERRAIN_ALT", 10)

MAV_MISSION_TYPE_MISSION = _const("MAV_MISSION_TYPE_MISSION", 0)
MAV_MISSION_TYPE_FENCE = _const("MAV_MISSION_TYPE_FENCE", 1)
MAV_MISSION_TYPE_RALLY = _const("MAV_MISSION_TYPE_RALLY", 2)

NAV_COMMANDS: frozenset[int] = frozenset({
    MAV_CMD_NAV_WAYPOINT, MAV_CMD_NAV_LOITER_TURNS, MAV_CMD_NAV_LOITER_TIME, MAV_CMD_NAV_RETURN_TO_LAUNCH,
    MAV_CMD_NAV_LOITER_TO_ALT, MAV_CMD_NAV_VTOL_TAKEOFF, MAV_CMD_NAV_VTOL_LAND, MAV_CMD_NAV_DELAY,
})
"""會推進任務序列之 NAV 指令（DO_* 為附掛於前一 NAV 之即時指令）。"""

AltFrame = Literal["relative", "msl", "terrain"]
FRAME_OF: dict[str, int] = {"relative": MAV_FRAME_GLOBAL_RELATIVE_ALT, "msl": MAV_FRAME_GLOBAL,
                            "terrain": MAV_FRAME_GLOBAL_TERRAIN_ALT}

# LiDAR 佔位：ArduPilot 無通用「LiDAR 開／關」指令，依酬載以 AUX 伺服／繼電器觸發（待實機驗證，ardupilot_notes §1）。
LIDAR_SERVO_CHANNEL: int = 9
"""DO_SET_SERVO param1：LiDAR 錄製開關之 AUX 輸出通道（佔位；依 SERVOx_FUNCTION 配置調整）。"""
LIDAR_SERVO_PWM_ON: int = 1900
LIDAR_SERVO_PWM_OFF: int = 1100
CAMERA_SERVO_CHANNEL: int = 10
"""相機觸發輸出通道（SERVOx_FUNCTION = 10 CameraTrigger）；佔位，依實機接線調整。"""

QGC_WPL_HEADER = "QGC WPL 110"
_QGC_FIRMWARE_ARDUPILOT = 3
_QGC_VEHICLE_FIXED_WING = 2

_LANDING_PHASES = frozenset({Phase.RTB, Phase.DESCENT, Phase.DECEL, Phase.TRANS_IN, Phase.LAND})
_FULL_LAP_TOL_RAD = 0.1


# ---------------------------------------------------------------------------
# 資料型別
# ---------------------------------------------------------------------------
@dataclass(slots=True)
class MissionItem:
    """一列 MISSION_ITEM_INT 等價資料（度／公尺；`x=lat_deg`、`y=lon_deg`、`z=alt_m` 依 `frame` 解讀）。"""
    seq: int
    frame: int
    command: int
    current: int
    autocontinue: int
    param1: float
    param2: float
    param3: float
    param4: float
    x: float
    y: float
    z: float
    mission_type: int = 0

    @property
    def is_nav(self) -> bool:
        return self.command in NAV_COMMANDS

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    def as_mission_item_int_kwargs(self, target_system: int = 1, target_component: int = 1) -> dict[str, Any]:
        """`mav.mission_item_int_send(**kw)` 之關鍵字（lat/lon ×1e7 取整）。"""
        return dict(target_system=target_system, target_component=target_component, seq=self.seq, frame=self.frame,
                    command=self.command, current=self.current, autocontinue=self.autocontinue,
                    param1=float(self.param1), param2=float(self.param2), param3=float(self.param3),
                    param4=float(self.param4), x=int(round(self.x * 1e7)), y=int(round(self.y * 1e7)),
                    z=float(self.z), mission_type=self.mission_type)


def _item(seq: int, command: int, frame: int, p: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0),
          x: float = 0.0, y: float = 0.0, z: float = 0.0, current: int = 0, mission_type: int = 0) -> MissionItem:
    return MissionItem(int(seq), int(frame), int(command), int(current), 1, float(p[0]), float(p[1]), float(p[2]),
                       float(p[3]), float(x), float(y), float(z), int(mission_type))


def renumber(items: Sequence[MissionItem]) -> list[MissionItem]:
    """依序重編 `seq`（就地修改並回傳同一 list）。"""
    out = list(items)
    for i, it in enumerate(out):
        it.seq = i
    return out


# ---------------------------------------------------------------------------
# 相機觸發設定（飛控相機後端參數 ＋ 任務中 206 之擺放方式）
#   參數名／列舉皆對過 ArduPilot 原始碼（Plane-4.4.4／4.5.7／4.7.2）：
#   AP_Camera_Params.cpp:8-75、AP_Camera.cpp:20-44、AP_Camera_Relay.cpp、AP_Relay.cpp／AP_Relay_Params.cpp
# ---------------------------------------------------------------------------
CAMERA_BACKEND_TYPE: dict[str, int] = {"servo": 1, "relay": 2, "mavlink_v2": 6}
"""backend → CAM1_TYPE（AP_Camera_Params.cpp:11；4.4.4 只有 1–5，6 = MAVLinkCamV2 自 4.5 起）。"""
RELAY_FUNCTION_CAMERA: int = 4
"""RELAYn_FUNCTION 之 Camera 列舉（AP_Relay_Params.cpp:12，Plane-4.5.7 與 4.7.2 相同；4.4.x 沒有 RELAYn_FUNCTION）。"""
RELAY_NUM_RELAYS: int = 6
"""AP_RELAY_NUM_RELAYS 預設值（AP_Relay.h：4.4.4:14、4.5.7/4.7.2:20）。"""
_SERVO_CH_MAX = 32


def parse_firmware_version(fw: str) -> tuple[int, int, int]:
    """'4.5.7'／'V4.8.0-dev'／'4.6' → (4, 5, 7)；看不懂 → ValueError（不猜）。"""
    m = re.match(r"^\s*[Vv]?(\d+)\.(\d+)(?:\.(\d+))?", str(fw))
    if m is None:
        raise ValueError(f"看不懂的韌體版本 {fw!r}（應為 '4.5.7' 這種形式）")
    return int(m.group(1)), int(m.group(2)), int(m.group(3) or 0)


def _check_int(name: str, v: Any, lo: int, hi: int) -> None:
    if isinstance(v, bool) or not isinstance(v, (int, np.integer)):
        raise ValueError(f"{name} 須為整數，得 {v!r}")
    if not lo <= int(v) <= hi:
        raise ValueError(f"{name} 須在 [{lo}, {hi}]，得 {v}")


def _check_float(name: str, v: Any, lo: float, hi: float, *, lo_open: bool = False, hi_open: bool = False) -> None:
    if isinstance(v, bool) or not isinstance(v, (int, float, np.integer, np.floating)) or not math.isfinite(float(v)):
        raise ValueError(f"{name} 須為有限實數，得 {v!r}")
    x = float(v)
    if (x <= lo if lo_open else x < lo) or (x >= hi if hi_open else x > hi):
        raise ValueError(f"{name} 須在 {'(' if lo_open else '['}{lo}, {hi}{')' if hi_open else ']'}，得 {x}")


@dataclass(frozen=True, slots=True)
class CameraTriggerConfig:
    """相機觸發設定：飛控相機後端參數（:func:`recommended_params`）與任務中 206 之擺放（:func:`plan_to_mission_items`）。

    * ``backend``：``"servo"``（CAM1_TYPE 1，SERVO{servo_channel}_FUNCTION=10）、``"relay"``（2，GPIO 脈衝，建議實機用）、
      ``"mavlink_v2"``（6，需 ArduPlane ≥ 4.5 且 flash > 1 MB；相機須在「開機 > 60 s 且已解鎖」之前被找到，否則整架次
      靜默零張——AP_Camera_MAVLinkCamV2.cpp:186-216）。
    * ``max_roll_deg`` 必須是整數度（CAM_MAX_ROLL 是 AP_Int16；12.5 會被飛控存成 12 ⇒ 拒絕，不替使用者取整）。
    * ``firmware``：該機 ArduPlane 版本（如 ``"4.5.7"``）。relay 之參數名依版本而異（4.4.x：RELAY_PIN＝relay instance 0，
      4.4 的相機後端只驅動它；4.5 起：RELAYn_FUNCTION=4＋RELAYn_PIN），**版本未知時 relay／mavlink_v2 一律拒絕**，不猜名字。
    * ``duration_s``：CAM1_DURATION（秒，4.4.0 起單位為秒；期間拒絕新觸發）。``interval_min_s``：CAM1_INTRVAL_MIN（秒；
      觸發早於此值時飛控**靜默延後**，須等於規劃器所用之 1/最大拍照率）。``auto_only``：CAM_AUTO_ONLY。
      ``max_roll_deg``：CAM_MAX_ROLL（0 = 不限；只延後距離觸發，206 p3=1 的首張不受限）。
    * ``relay_instance``／``relay_pin``：RELAYn 與其 GPIO 腳號（如 Cube AUX1 = 50）；``aux_servo_channel_for_relay``：
      該腳所屬之 SERVOn（ChibiOS 上 PWM 腳須 SERVOn_FUNCTION=-1 才是 GPIO；Cube AUX1 = SERVO9）。
    * ``feedback_pin``／``feedback_polarity``／``feedback_servo_channel``：CAM1_FEEDBAK_PIN／POL 與其 SERVOn（設 -1）。
    * ``first_shot_immediate``：每條帶 206 之 p3（1 = 開啟時立即拍一張）。
    * ``camera_lead_in_m`` > 0：在條帶起點後 lead_in 處插一個共線 NAV_WAYPOINT（接受半徑 1 m，見
      :data:`LEAD_IN_ACCEPT_RADIUS_M`），206 開改掛在它之後（轉彎改出後才開拍）。有效引入段 ＝ lead_in − 1 m 到
      lead_in + v·0.1 s（10 Hz 任務更新）；飛前模擬實測 50 m 引入段的首張落在帶起點後 49.0–51.1 m。任何 > 0 的值都照用
      （不另設下限）：小於 1 m 的引入段等於在帶起點開拍。
    * ``terrain_retrigger_tol``：條帶中途之 OPERATE 航點 AGL 與目前 d 所依據之 AGL 相對差 > tol 時，於該點後重發 206(d_new)。
    """
    backend: str = "servo"
    firmware: str | None = None
    duration_s: float = 0.1
    interval_min_s: float = 0.0
    auto_only: bool = True
    max_roll_deg: float = 0.0
    servo_channel: int = CAMERA_SERVO_CHANNEL
    relay_instance: int = 1
    relay_pin: int | None = None
    aux_servo_channel_for_relay: int | None = None
    feedback_pin: int = -1
    feedback_polarity: int = 1
    feedback_servo_channel: int | None = None
    first_shot_immediate: bool = True
    camera_lead_in_m: float = 0.0
    terrain_retrigger_tol: float | None = None

    def __post_init__(self) -> None:
        if self.backend not in CAMERA_BACKEND_TYPE:
            raise ValueError(f"backend 須為 {tuple(CAMERA_BACKEND_TYPE)}，得 {self.backend!r}")
        fw = self.firmware_version
        if fw is not None and fw < (4, 4):
            raise ValueError(f"firmware {self.firmware}：CAM_* 在 ArduPlane 4.4.0 才改名為 CAM1_*（且 DURATION 單位由 0.1 s "
                             "改為秒），4.4 以前的參數組本匯出器不支援")
        _check_float("duration_s", self.duration_s, 0.0, 5.0, lo_open=True)       # AP_Camera_Params.cpp:19 @Range 0 5
        _check_float("interval_min_s", self.interval_min_s, 0.0, 10.0)            # :58 @Range 0 10
        _check_float("max_roll_deg", self.max_roll_deg, 0.0, 180.0)               # AP_Camera.cpp:27 @Range 0 180
        if not float(self.max_roll_deg).is_integer():
            # CAM_MAX_ROLL 在 ArduPlane 是 AP_Int16（AP_Camera.h 4.5.7:227 `AP_Int16 _max_roll`）：小數會被飛控截掉
            raise ValueError(f"max_roll_deg 須為整數度：CAM_MAX_ROLL 在 ArduPlane 以 AP_Int16 儲存，"
                             f"{float(self.max_roll_deg):g}° 會被飛控存成 {int(float(self.max_roll_deg))}°")
        for name in ("auto_only", "first_shot_immediate"):
            if not isinstance(getattr(self, name), bool):
                raise ValueError(f"{name} 須為 bool，得 {getattr(self, name)!r}")
        _check_int("servo_channel", self.servo_channel, 1, _SERVO_CH_MAX)
        _check_int("relay_instance", self.relay_instance, 1, RELAY_NUM_RELAYS)
        _check_int("feedback_pin", self.feedback_pin, -1, 127)                    # 4.7.2 :66 @Range -1 127
        _check_int("feedback_polarity", self.feedback_polarity, 0, 1)
        for name in ("aux_servo_channel_for_relay", "feedback_servo_channel"):
            v = getattr(self, name)
            if v is not None:
                _check_int(name, v, 1, _SERVO_CH_MAX)
        _check_float("camera_lead_in_m", self.camera_lead_in_m, 0.0, 1e5)
        if self.terrain_retrigger_tol is not None:
            _check_float("terrain_retrigger_tol", self.terrain_retrigger_tol, 0.0, 1.0, lo_open=True, hi_open=True)
        # ---- 只對某一後端有意義的值：給了卻用不到 ⇒ 拒絕（有效值契約：不可靜默忽略）----
        if self.backend != "servo" and self.servo_channel != CAMERA_SERVO_CHANNEL:
            raise ValueError(f"servo_channel={self.servo_channel} 只用於 backend='servo'（目前 {self.backend!r}）")
        if self.backend != "relay":
            for name, dflt in (("relay_pin", None), ("aux_servo_channel_for_relay", None), ("relay_instance", 1)):
                if getattr(self, name) != dflt:
                    raise ValueError(f"{name}={getattr(self, name)} 只用於 backend='relay'（目前 {self.backend!r}）")
        if self.feedback_servo_channel is not None and self.feedback_pin < 0:
            raise ValueError("feedback_servo_channel 給了但 feedback_pin = -1（沒有回授腳）")
        if self.backend == "servo" and self.feedback_servo_channel == self.servo_channel:
            raise ValueError(f"SERVO{self.servo_channel} 不能同時是相機觸發輸出（FUNCTION 10）與回授 GPIO（-1）")
        # ---- 依版本 ----
        if self.backend == "mavlink_v2":
            if fw is None:
                raise ValueError("backend='mavlink_v2'（CAM1_TYPE 6）需 ArduPlane ≥ 4.5，firmware 未知時拒絕")
            if fw < (4, 5):
                raise ValueError(f"backend='mavlink_v2' 需 ArduPlane ≥ 4.5（4.4.4 的 CAM1_TYPE 沒有 6），得 {self.firmware}")
        if self.backend == "relay":
            if fw is None:
                raise ValueError("backend='relay' 必須給 firmware：relay 參數名依版本而異（4.4.x 為 RELAY_PIN，"
                                 "4.5 起為 RELAYn_FUNCTION=4＋RELAYn_PIN），版本未知時不猜")
            if self.relay_pin is None:
                raise ValueError("backend='relay' 必須給 relay_pin（GPIO 腳號，如 Cube AUX1 = 50）")
            _check_int("relay_pin", self.relay_pin, 0, 127 if fw < (4, 5) else 1015)   # 4.4 AP_Int8；4.7.2 @Range -1 1015
            if fw < (4, 5) and self.relay_instance != 1:
                raise ValueError(f"ArduPlane {self.firmware}：4.4 的相機後端只驅動 relay instance 0（RELAY_PIN；"
                                 f"AP_Camera_Relay.cpp 4.4.4:17-21、42-46），relay_instance 必須為 1，得 {self.relay_instance}")
            if self.aux_servo_channel_for_relay is not None \
                    and self.aux_servo_channel_for_relay == self.feedback_servo_channel:
                raise ValueError(f"SERVO{self.aux_servo_channel_for_relay} 不能同時是 relay 輸出腳與回授腳")
            if self.feedback_pin >= 0 and self.feedback_pin == self.relay_pin:
                raise ValueError(f"relay_pin 與 feedback_pin 同為 {self.relay_pin}")

    @property
    def firmware_version(self) -> tuple[int, int, int] | None:
        return None if self.firmware is None else parse_firmware_version(self.firmware)

    @property
    def cam_type(self) -> int:
        """CAM1_TYPE 值。"""
        return CAMERA_BACKEND_TYPE[self.backend]


def _camera_params(cc: CameraTriggerConfig, warn: list[str]) -> dict[str, float]:
    """`cc` → CAM*／RELAY*／SERVOn 參數（每鍵說明見 :data:`PARAM_COMMENTS` 與 :func:`camera_param_comments`）。"""
    p: dict[str, float] = {
        "CAM1_TYPE": float(cc.cam_type),
        "CAM1_DURATION": float(cc.duration_s),
        "CAM1_INTRVAL_MIN": float(cc.interval_min_s),
        "CAM_AUTO_ONLY": 1.0 if cc.auto_only else 0.0,
        "CAM_MAX_ROLL": float(cc.max_roll_deg),
        "CAM1_FEEDBAK_PIN": float(cc.feedback_pin),
        "CAM1_FEEDBAK_POL": float(cc.feedback_polarity),
        "CAM1_TRIGG_DIST": 0.0,                            # 距離由任務每條帶之 206 開／關
    }
    if cc.backend == "servo":
        p[f"SERVO{cc.servo_channel}_FUNCTION"] = 10.0      # CameraTrigger
    elif cc.backend == "relay":
        fw = cc.firmware_version
        assert fw is not None and cc.relay_pin is not None          # __post_init__ 已擋
        p["CAM1_RELAY_ON"] = 1.0                           # 觸發時拉高（樹莓派偵測上升緣）
        if fw >= (4, 5):
            p[f"RELAY{cc.relay_instance}_FUNCTION"] = float(RELAY_FUNCTION_CAMERA)
            p[f"RELAY{cc.relay_instance}_PIN"] = float(cc.relay_pin)
        else:
            p["RELAY_PIN"] = float(cc.relay_pin)           # 4.4.x：relay instance 0
        if cc.aux_servo_channel_for_relay is not None:
            p[f"SERVO{cc.aux_servo_channel_for_relay}_FUNCTION"] = -1.0       # GPIO
        else:
            warn.append(f"relay pin {cc.relay_pin}: aux_servo_channel_for_relay not given, so no SERVOn_FUNCTION=-1 "
                        "is exported. On ChibiOS a PWM-capable AUX pin is a GPIO only with SERVOn_FUNCTION=-1 "
                        "(Cube AUX1 = SERVO9); otherwise the relay does not move while CAM is still logged")
    if cc.feedback_servo_channel is not None:
        p[f"SERVO{cc.feedback_servo_channel}_FUNCTION"] = -1.0           # GPIO（回授輸入）
    elif cc.feedback_pin >= 0:
        warn.append(f"CAM1_FEEDBAK_PIN {cc.feedback_pin}: feedback_servo_channel not given, so no SERVOn_FUNCTION=-1 "
                    "is exported for the feedback pin (needed on ChibiOS for a PWM-capable AUX pin)")
    fw = cc.firmware_version
    if fw is not None and fw < (4, 5):
        warn.append(f"firmware {cc.firmware}: only the CAM*/RELAY* names were adapted to 4.4; the other parameters in "
                    "this file use ArduPlane 4.5+ names (e.g. AIRSPEED_CRUISE was TRIM_ARSPD_CM, ROLL_LIMIT_DEG was "
                    "LIM_ROLL_CD) and are rejected by a 4.4 vehicle")
    return p


def camera_param_comments(cc: CameraTriggerConfig | None) -> dict[str, str]:
    """`cc` 會輸出之 SERVOn_FUNCTION 鍵（通道由設定決定，無法列在 :data:`PARAM_COMMENTS`）之說明。"""
    if cc is None:
        return {}
    out: dict[str, str] = {}
    if cc.backend == "servo":
        out[f"SERVO{cc.servo_channel}_FUNCTION"] = ("10 = CameraTrigger — the output the servo camera backend drives to "
                                                    "CAM1_SERVO_ON for CAM1_DURATION")
    if cc.backend == "relay" and cc.aux_servo_channel_for_relay is not None:
        out[f"SERVO{cc.aux_servo_channel_for_relay}_FUNCTION"] = (
            f"-1 = GPIO — the output carrying relay pin {cc.relay_pin}; on ChibiOS a PWM-capable pin is only a GPIO "
            "with -1 (ChibiOS_GPIO.cpp:66-77; Cube AUX1 = SERVO9)")
    if cc.feedback_servo_channel is not None:
        out[f"SERVO{cc.feedback_servo_channel}_FUNCTION"] = (
            f"-1 = GPIO — the input wired to CAM1_FEEDBAK_PIN {cc.feedback_pin}")
    return out


# ---------------------------------------------------------------------------
# 幾何輔助
# ---------------------------------------------------------------------------
def _home_msl(ctx: MissionContext) -> float:
    xy, z = ctx.sites_enu[ctx.inputs.home.site_id]
    return float(ctx.frame.enu_z_to_msl(np.asarray(xy, dtype=float).reshape(1, 2), float(z))[0])


def _site_msl(ctx: MissionContext, site_id: str) -> float:
    xy, z = ctx.sites_enu[site_id]
    return float(ctx.frame.enu_z_to_msl(np.asarray(xy, dtype=float).reshape(1, 2), float(z))[0])


def fc_home(ctx: MissionContext, uav_plan: UAVPlan) -> tuple[str, float, float, float]:
    """這架飛控**實際**的 home ＝ 它自己的起飛坪 → ``(site_id, lat_deg, lon_deg, msl_m)``。

    ArduPlane 解鎖時把 home 重設為飛機當下的位置（AP_Arming_Plane.cpp 4.5.7:307-313 → ``Plane::update_home``；
    解鎖前 commands.cpp:119-150 也一直讓 home 追著目前位置）⇒ MAV_FRAME_GLOBAL_RELATIVE_ALT 的高度、rally 高度、
    FENCE_ALT_MAX 都是相對**起飛坪**，不是規劃 home。逐機起飛站（``TO_<uav>``）在斜坡上時兩者差 (坪 − home) MSL，
    整份任務就整體偏移這麼多（R1 實測 ±4.2 m，4° 坡）。平地（坪與 home 同 MSL）時與舊基準逐值相同。

    起飛站：``uav_plan.takeoff_site`` → ``ctx.start_site_id(uav_id)``；兩者都不在站點表 ⇒ 退回規劃 home 並記 WARNING。
    """
    sid = str(getattr(uav_plan, "takeoff_site", "") or "")
    if not sid:
        try:
            sid = str(ctx.start_site_id(uav_plan.uav_id))
        except Exception:  # noqa: BLE001 - 舊 ctx／無此機
            sid = ""
    sites = ctx.inputs.sites
    if sid and sid in ctx.sites_enu and sid in sites:
        s = sites[sid]
        msl = _site_msl(ctx, sid)
        h_plan = _home_msl(ctx)
        if abs(msl - h_plan) < 1e-6:
            # 同一個地面高度：ENU↔MSL 來回換算有 1e-9 m 級的雜訊（F1 實測 9.3e-10 m），不是真的高度差 ⇒ 用規劃 home 的值，
            # 平地時相對高度與舊版逐位元相同
            msl = h_plan
        return sid, float(s.lat_deg), float(s.lon_deg), msl
    h = ctx.inputs.home
    _log.warning("%s: 起飛站 %r 不在站點表 ⇒ 相對高度改以規劃 home %s 為基準；飛控解鎖時會把 home 重設到實際起飛位置，"
                 "兩者 MSL 不同時整份任務的高度會偏移這個差值", uav_plan.uav_id, sid, h.site_id)
    return h.site_id, float(h.lat_deg), float(h.lon_deg), _home_msl(ctx)


def _wgs(ctx: MissionContext, xy: np.ndarray, z: np.ndarray | float) -> np.ndarray:
    """ENU (N,2)+z → (N,3) [lat, lon, msl]。"""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    zz = np.broadcast_to(np.asarray(z, dtype=float), (xy.shape[0],))
    return ctx.frame.to_wgs84(np.column_stack([xy, zz]))


def accept_radius_m(r_eff_m: float, wp_radius_m: float | None = None) -> float:
    """r_acc = min(R_eff/2, 30) m（規格 §8.5）；`wp_radius_m` 覆寫。"""
    if wp_radius_m is not None:
        return float(wp_radius_m)
    return float(min(0.5 * float(r_eff_m), 30.0))


LEAD_IN_ACCEPT_RADIUS_M: float = 1.0
"""引入段 NAV_WAYPOINT 的 param2（該航點的接受半徑，m）。

ArduPlane ``verify_nav_wp`` 取 ``LOWBYTE(param2)`` 為接受距離（commands_logic.cpp Plane-4.5.7:651、681-689）；
**0** 才退回 ``WP_RADIUS`` 經 ``turn_distance``（依下一轉角提前），所以不能給 0。引入點與帶起點共線，給最小的非零值
1 m：飛機到引入點 1 m 內、或越過它的終點線（``past_interval_finish_line``）才算抵達，掛在其後的 206 開就在帶起點後
≈ lead_in 處執行（加上 10 Hz 任務更新的 ≤ 0.1 s）。舊值 r_acc（30 m）讓 206 開提早 30 m：開拍點在帶起點後
lead_in − 30（50 m 引入段只剩 20 m；≤ 30 m 的引入段首張在轉彎裡，且 p3=1 的首張不受 CAM_MAX_ROLL 擋）。"""

AGL_SNAP_TOL_M: float = 0.5
"""206 觸發距離所依據之高度（只在給了 ``camera_cfg`` 時）：條帶起點 AGL 與該感測器的規劃作業高度
（``PayloadRequirement.h_target_agl_m``）相差 ≤ 本值 ⇒ 用作業高度。平地條帶的 AGL 不會剛好等於作業高度：
沒有 DEM 時 ENU↔WGS84 換算有 µm 級抖動（R2：一條帶 18.847990619763777，其餘 float32(18.848)，差 5 ulp）；有 DEM
（連平的 100 m 也一樣）時規劃器沿帶保持 ENU z、地面卻隨地球曲率 d²/2R 下降，再加 DEM 內插 ⇒ 帶起點 AGL 高出
4–42 mm（F1 實測），每條帶的 d 都不同。0.5 m 涵蓋規劃原點 2.5 km 內的曲率（2500²/(2·6.37e6) ≈ 0.49 m）；
d ∝ h，0.5 m／120 m 對 d 的影響 ≤ 0.42 %（比 50 Hz tick 造成的 +0.8 % 間距量化還小），且帶起點 AGL ≥ 作業高度時
用作業高度算出的 d 較短（重疊略多，保守）。差超過本值（真的地形或高度帶差）照舊用該點 AGL；作業高度被高度帶夾離
目標時同理（照舊）。"""


def _trigger_agl(sensor: Any, agl_m: float) -> float:
    """206 觸發距離用的高度：與規劃作業高度相差 ≤ :data:`AGL_SNAP_TOL_M` ⇒ 作業高度；否則該點 AGL。"""
    h_t = getattr(getattr(sensor, "r", None), "h_target_agl_m", None)
    if h_t is None:
        h_t = getattr(getattr(sensor, "req", None), "h_target_agl_m", None)
    try:
        h = float(h_t) if h_t is not None else None
    except (TypeError, ValueError):
        h = None
    if h is not None and math.isfinite(h) and abs(float(agl_m) - h) <= AGL_SNAP_TOL_M:
        return h
    return float(agl_m)


def _arc_sweep(pts: np.ndarray, center: tuple[float, float]) -> float:
    """依序取樣點之累積掃掠角（rad，帶號：+ 逆時針（幾何角增）、− 順時針）。"""
    ang = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
    d = np.diff(ang)
    d = (d + math.pi) % (2.0 * math.pi) - math.pi
    return float(d.sum())


def arc_vertices(p_start: np.ndarray, p_end: np.ndarray, center: tuple[float, float], R: float, turn_dir: int,
                 dpsi_rad: float, sweep_rad: float | None = None, force_single: bool = False) -> np.ndarray:
    """外切多邊形頂點（論文 Eq.(29)）：自 `p_start` 沿圓 (center, R) 轉向 `turn_dir`（+1 順時針／右轉、−1 逆時針／左轉）
    至 `p_end`，n = ⌈|Δ|/Δψ⌉ 段，頂點在每段中點方向、徑向距 R/cos(δ/2)；回傳 (n,2)，**不含** p_end。
    `sweep_rad` 可給定（帶號幾何掃掠角）以消除 2π 模糊。|Δ| ≤ Δψ 時弦之矢高已 ≤ e_max，回傳空陣列
    （`force_single=True` 仍給 1 個頂點）。"""
    cx, cy = center
    a0 = math.atan2(p_start[1] - cy, p_start[0] - cx)
    if sweep_rad is None:
        a1 = math.atan2(p_end[1] - cy, p_end[0] - cx)
        if turn_dir > 0:      # 順時針：幾何角遞減
            sweep = -((a0 - a1) % (2.0 * math.pi))
        else:
            sweep = (a1 - a0) % (2.0 * math.pi)
    else:
        sweep = float(sweep_rad)
    if abs(sweep) < 1e-9:
        return np.zeros((0, 2))
    n = max(1, int(math.ceil(abs(sweep) / dpsi_rad - 1e-9)))
    if n == 1 and not force_single:
        # |Δ| ≤ Δψ：弦之矢高 R(1 − cos(Δ/2)) ≤ R·e_max/(R + e_max) < e_max，直接以起訖點連線即可，不需頂點
        return np.zeros((0, 2))
    delta = sweep / n
    r_v = R / math.cos(0.5 * abs(delta))
    ang = a0 + (np.arange(n) + 0.5) * delta
    return np.column_stack([cx + r_v * np.cos(ang), cy + r_v * np.sin(ang)])


def _dp_keep(s: np.ndarray, z: np.ndarray, tol: float) -> np.ndarray:
    """(s, z) 折線之 Douglas–Peucker 保留遮罩（端點恆保留）。"""
    n = len(s)
    keep = np.zeros(n, dtype=bool)
    if n == 0:
        return keep
    keep[0] = keep[-1] = True
    stack = [(0, n - 1)]
    while stack:
        i, j = stack.pop()
        if j - i < 2:
            continue
        zi = z[i] + (z[j] - z[i]) * (s[i + 1:j] - s[i]) / max(s[j] - s[i], 1e-9)
        dev = np.abs(z[i + 1:j] - zi)
        k = int(np.argmax(dev))
        if dev[k] > tol:
            m = i + 1 + k
            keep[m] = True
            stack.append((i, m))
            stack.append((m, j))
    return keep


# ---------------------------------------------------------------------------
# 航點縮減（規格 §8.5；ardupilot_notes §1）
# ---------------------------------------------------------------------------
@dataclass(slots=True)
class _NavPoint:
    """縮減後之導航點（ENU）。`kind`: 'wp' | 'takeoff' | 'loiter_alt' | 'loiter_turns' | 'loiter_time' | 'land'。"""
    x: float
    y: float
    z: float
    agl: float
    t: float
    phase: Phase
    mode: Mode
    v_air: float
    sensor_on: bool
    strip_id: str | None
    essential: bool
    kind: str = "wp"
    radius_signed: float = 0.0
    laps: int = 0
    duration_s: float = 0.0
    src: int = -1                      # 來源 Waypoint 索引
    vertex: bool = False               # 外切多邊形頂點（在相鄰接點之切線上；不因接受半徑被丟棄）
    protected: bool = False            # 無頂點短弧之起點（弦矢高不可忽略；不因接受半徑被丟棄，但不在切線上）
    into_line: bool = False            # 弧起點且入航段為直線（與前一保留點、首頂點共線 → 可省略）


def _runs(wps: Sequence[Waypoint]) -> list[tuple[int, int, str]]:
    """分割為 [(i0, i1, kind)]（含 i1）：同一弧（arc tuple 相同）為 'arc'；直線為 'line'；
    分割於 arc／strip／sensor／mode 變化處及走廊／ORBIT 階段邊界（FW 階段標籤 CLIMB→ENTRY 等中途改名不切分）。"""
    runs: list[tuple[int, int, str]] = []
    n = len(wps)
    i = 0

    def key_of(w: Waypoint) -> tuple:
        ph = w.phase if (w.phase is Phase.ORBIT or w.phase not in FW_PHASES) else None
        return (w.arc, ph, w.strip_id, w.sensor_on, w.mode)

    while i < n:
        w = wps[i]
        key = key_of(w)
        j = i
        while j + 1 < n:
            if key_of(wps[j + 1]) != key:
                break
            j += 1
        runs.append((i, j, "arc" if w.arc is not None else "line"))
        i = j + 1
    return runs


def _boundary(wps: Sequence[Waypoint], i: int) -> bool:
    """第 i 點是否為必要點：strip／sensor／mode 邊界或緊接轉換段之首點（FW 階段標籤之變化本身不構成必要點）。"""
    if i == 0 or i >= len(wps):
        return True
    p, q = wps[i - 1], wps[i]
    return (p.strip_id != q.strip_id or p.sensor_on != q.sensor_on or p.mode is not q.mode
            or p.phase in (Phase.TRANS_OUT, Phase.TRANS_IN, Phase.TAKEOFF, Phase.LAND))


def reduce_waypoints(wps: Sequence[Waypoint], spec: PlatformSpec, cfg: PlannerConfig, r_acc_m: float,
                     z_tol_m: float | None = None, heading_tol_deg: float = 1.0) -> list[_NavPoint]:
    """`UAVPlan.waypoints`（build_3d 之密集取樣）→ autopilot 導航點：

    * 弧段：同一 `arc` 之連續點以外切多邊形重離散（Δψ 依 D27，e_max = cfg.e_max_m、ψ_max = cfg.psi_max_deg），
      ORBIT 階段之整圈（|Δ| ≥ 2π − 0.1）改為盤旋指令（loiter_alt／loiter_turns／loiter_time）。
    * 直線段：保留端點、航向轉折（> heading_tol_deg）、高度剖面 DP 轉折（> z_tol_m；預設 cfg.delta_prof_m）。
    * TRANS_OUT／TRANS_IN 之中間取樣點全部略去（ArduPilot 自行轉換）；TAKEOFF → 'takeoff'、LAND → 'land'。
    * 非必要點距前一保留點 < r_acc 時丟棄。
    """
    n = len(wps)
    if n == 0:
        return []
    z_tol = float(cfg.delta_prof_m if z_tol_m is None else z_tol_m)
    hd_tol = math.radians(heading_tol_deg)
    xy = np.array([[w.x_m, w.y_m] for w in wps], dtype=float)
    z = np.array([w.z_m for w in wps], dtype=float)
    agl = np.array([w.agl_m for w in wps], dtype=float)
    t = np.array([w.t_s for w in wps], dtype=float)
    out: list[_NavPoint] = []

    def mk(i: int, kind: str = "wp", essential: bool = False, **kw: Any) -> _NavPoint:
        w = wps[i]
        return _NavPoint(float(xy[i, 0]), float(xy[i, 1]), float(z[i]), float(agl[i]), float(t[i]), w.phase, w.mode,
                         float(w.airspeed_mps), bool(w.sensor_on), w.strip_id, essential, kind, src=i, **kw)

    for (i0, i1, kind) in _runs(wps):
        w0 = wps[i0]
        ph = w0.phase
        if ph is Phase.TAKEOFF:
            out.append(mk(i0, "takeoff", True))
            continue
        if ph is Phase.LAND:
            out.append(mk(i0, "land", True))
            continue
        if ph in (Phase.TRANS_OUT, Phase.TRANS_IN):
            continue                                   # ArduPilot 自行執行轉換；端點由相鄰 ACCEL／LAND 供給
        i_end = min(i1 + 1, n - 1)                     # 本 run 之出航段終點（下一 run 首點）
        if kind == "arc":
            assert w0.arc is not None
            cx, cy, R, tdir = float(w0.arc[0]), float(w0.arc[1]), float(w0.arc[2]), int(w0.arc[3])
            pts = xy[i0:i_end + 1]
            sweep = _arc_sweep(pts, (cx, cy))
            single_closed = (i_end == i0) or (np.hypot(*(pts[-1] - pts[0])) < 1.0 and abs(sweep) < 1e-6)
            dz = float(z[i_end] - z[i0])
            dt = float(t[i_end] - t[i0]) if i_end > i0 else 0.0
            if ph is Phase.ORBIT and (abs(sweep) >= 2.0 * math.pi - _FULL_LAP_TOL_RAD or single_closed):
                # 整圈盤旋（起飛／進入螺旋、去衝突延遲盤旋）
                va = max(float(w0.airspeed_mps), 1e-6)
                t_lap = 2.0 * math.pi * R / va
                laps = int(round(abs(sweep) / (2.0 * math.pi))) if abs(sweep) >= 2.0 * math.pi - _FULL_LAP_TOL_RAD \
                    else max(1, int(round(dt / t_lap))) if dt > 0 else 1
                # ⚠⚠⚠ vendored 修改 #6：先把**真正的**進入點補回去，再接 loiter。
                #   下面那行 `p.x, p.y = cx, cy` 會把這個點的座標改成盤旋**圓心**，
                #   而直線 run 又固定 `keep[-1] = False`（見本函式下方）丟掉自己的終點、
                #   指望「下一 run 首點」補上。兩件事湊在一起 ⇒ **條帶終點從上傳序列裡
                #   整個消失**，飛機從條帶的一端直接飛向圓心。
                #   實測（疊圖 (c) 格、5×4 km、4A+4B）：5 126 m 的作業條帶被斜掉
                #   r_orb = **120.00 m**，漏掃 87 851 m²＝**0.4393%**；缺口高度 65.42 m
                #   恰等於 2s − w，也就是「少掃一條」的精確值。關掉航點精簡則是
                #   精確的 100.0000% ⇒ 規劃沒問題，是壓成 ArduPilot 航點這一步壞的。
                #   ⚠ 三把獨立的尺（shapely quad_segs 8/64、格網法 5 m/10 m）一致，
                #     且航跡加密到每 1 m 補點數字完全不動 ⇒ 位置錯，不是取樣粗。
                if (not out
                        or math.hypot(out[-1].x - float(xy[i0, 0]),
                                      out[-1].y - float(xy[i0, 1])) > 1e-6):
                    out.append(mk(i0, "wp", True))
                p = mk(i0, "loiter_alt" if abs(dz) >= 1.0 else ("loiter_time" if dt > 0 else "loiter_turns"), True,
                       radius_signed=(R if tdir > 0 else -R), laps=laps, duration_s=dt)
                p.x, p.y = cx, cy
                p.z, p.agl = float(z[i_end]), float(agl[i_end])
                out.append(p)
                continue
            dpsi = discretize_step_rad(R, cfg.e_max_m, cfg.psi_max_rad)
            # |Δ| ≤ Δψ 之短弧不產生頂點（弦矢高 ≤ e_max）；矢高不可忽略（≥ 0.25 m）時其起點承載幾何 → 保護
            short_arc = 1e-9 < abs(sweep) <= dpsi + 1e-9
            sagitta = R * (1.0 - math.cos(0.5 * abs(sweep)))
            out.append(mk(i0, "wp", _boundary(wps, i0), into_line=(i0 > 0 and wps[i0 - 1].arc is None),
                          protected=(short_arc and sagitta >= 0.25)))
            if abs(sweep) < 1e-9:
                continue
            verts = arc_vertices(xy[i0], xy[i_end], (cx, cy), R, tdir, dpsi, sweep_rad=sweep)
            if len(verts):
                frac = (np.arange(len(verts)) + 0.5) / len(verts)
                zs = z[i0] + frac * dz
                ts = t[i0] + frac * dt
                agls = agl[i0] + frac * (agl[i_end] - agl[i0])
                for k, v in enumerate(verts):
                    out.append(_NavPoint(float(v[0]), float(v[1]), float(zs[k]), float(agls[k]), float(ts[k]), ph,
                                         w0.mode, float(w0.airspeed_mps), bool(w0.sensor_on), w0.strip_id, False,
                                         "wp", src=i0, vertex=True))
            continue
        # ---- 直線 run：i0..i1，出航段終點 i_end ----
        idx = np.arange(i0, i_end + 1)
        seg = xy[idx]
        d = np.diff(seg, axis=0)
        s = np.concatenate([[0.0], np.cumsum(np.hypot(d[:, 0], d[:, 1]))])
        keep = _dp_keep(s, z[idx], z_tol)
        if len(idx) > 2:
            hdg = np.arctan2(d[:, 0], d[:, 1])
            dh = np.abs((np.diff(hdg) + math.pi) % (2.0 * math.pi) - math.pi)
            keep[1:-1] |= dh > hd_tol
        keep[-1] = False                               # 終點由下一 run 首點供給
        for k, i in enumerate(idx):
            if keep[k]:
                out.append(mk(int(i), "wp", _boundary(wps, int(i)) if k == 0 else False))
    # 末點（LAND 已處理；若計畫無 LAND，補最後一點）
    if wps[-1].phase is not Phase.LAND and (not out or out[-1].src != n - 1):
        out.append(mk(n - 1, "wp", True))
    # ---- 接受半徑過濾：距前一保留點 < r_acc 時，非必要／非頂點／非保護之點丟棄；必要點取代前一可丟棄點；
    #      弧頂點取代前一可丟棄之弧起點，惟須共線（入航為直線或前一保留點為頂點——切線構造）；否則並存 ----
    filtered: list[_NavPoint] = []
    for p in out:
        if filtered and p.kind == "wp" and filtered[-1].kind in ("wp", "takeoff"):
            if math.hypot(p.x - filtered[-1].x, p.y - filtered[-1].y) < r_acc_m:
                if not (p.essential or p.vertex or p.protected):
                    continue
                last = filtered[-1]
                collinear = last.into_line or (len(filtered) >= 2 and filtered[-2].vertex)
                droppable = last.kind == "wp" and not (last.essential or last.vertex or last.protected)
                if droppable and (p.essential or collinear):
                    filtered.pop()
        filtered.append(p)
    return filtered


# ---------------------------------------------------------------------------
# 感測器觸發距離
# ---------------------------------------------------------------------------
def _sensor_for(uav_plan: UAVPlan, ctx: MissionContext, strip_id: str | None,
                strip_sensors: Mapping[str, Sensor] | None) -> Sensor | None:
    """條帶之感測器：`strip_sensors[strip_id]` → 同構型主感測種類之 ctx.sensors → 該構型任一。"""
    if strip_sensors is not None and strip_id is not None and strip_id in strip_sensors:
        return strip_sensors[strip_id]
    cls = uav_plan.cls
    cands = [(k, s) for k, s in ctx.sensors.items() if k[0] == cls]
    if not cands:
        return None
    try:
        primary_kind = ctx.uav(uav_plan.uav_id).primary_sensor.kind
    except Exception:  # noqa: BLE001
        primary_kind = None
    for _k, s in cands:
        if getattr(s, "kind", None) == primary_kind:
            return s
    return cands[0][1]


LOITER_TURNS_MAX_RADIUS_M: int = 255
"""NAV_LOITER_TURNS 之 |param3|：ArduPlane 將半徑存入 `cmd.p1` 高位元組（整數公尺，上限 255）。"""


def _loiter_turns_radius(radius_signed: float) -> float:
    """NAV_LOITER_TURNS param3：取整並夾至 ±255 m（ArduPlane `p1 = (radius_m << 8) | turns`）。"""
    r = int(round(abs(float(radius_signed))))
    r = max(0, min(LOITER_TURNS_MAX_RADIUS_M, r))
    return float(-r if radius_signed < 0 else r)


def orbit_radius_m(items: Sequence[MissionItem]) -> float | None:
    """任務中盤旋指令實際使用之半徑（m）；無盤旋 → None。

    ArduPlane 對 NAV_LOITER_TIME(19) **不讀** param3 之半徑（只讀正負號＝方向），一律以 `WP_LOITER_RAD` 飛；
    因此匯出時須把 `WP_LOITER_RAD` 設為本值，否則延遲盤旋之圈時／半徑與去衝突所保留者不符。
    """
    rs: list[float] = []
    for it in items:
        if it.command in (MAV_CMD_NAV_LOITER_TIME, MAV_CMD_NAV_LOITER_TURNS) and abs(it.param3) > 1e-6:
            rs.append(abs(float(it.param3)))
        elif it.command == MAV_CMD_NAV_LOITER_TO_ALT and abs(it.param2) > 1e-6:
            rs.append(abs(float(it.param2)))
    return max(rs) if rs else None


def min_nav_spacing_m(items: Sequence[MissionItem], ctx: MissionContext) -> float | None:
    """相鄰（有座標之）NAV 項目之最小水平間距（m）；不足兩點 → None。

    ArduPlane 的 NAV_WAYPOINT **會用** param2：非 0 時取其整數公尺（低位元組，≤ 255）為該航點之接受半徑，只有
    param2 = 0 時才退回 `WP_RADIUS`（經 turn_distance；commands_logic.cpp verify_nav_wp，Plane-4.4.4:665-670、
    4.5.7:681-686、4.7.2:673-678；AP_Mission.cpp 4.5.7:1019-1027）。本匯出器的 NAV_WAYPOINT 一律帶 param2 = r_acc，
    `WP_RADIUS` 只作用於 param2 = 0 之航點與其他用到它的 NAV（如盤旋）；仍令其小於本值，避免後一航點在前一航點
    被判抵達的同一瞬間即被判抵達、掛在其後之 DO_ 指令提早觸發。
    """
    pts = [(it.x, it.y) for it in items
           if it.is_nav and it.seq > 0 and not (it.x == 0.0 and it.y == 0.0)]
    if len(pts) < 2:
        return None
    xy = ctx.frame.to_enu(np.array([p[0] for p in pts]), np.array([p[1] for p in pts]))[:, :2]
    d = np.hypot(*np.diff(xy, axis=0).T)
    return float(d.min()) if d.size else None


def trigger_distance_m(sensor: Sensor, h_agl_m: float) -> float:
    """相機等距觸發距離 = 足跡前向長 l(h)·(1 − o_fwd)（規格 §4.2）；LiDAR（l=0）→ 0。"""
    l_fp = float(sensor.footprint_len_m(float(h_agl_m)))
    fwd = float(getattr(getattr(sensor, "r", None), "forward_overlap", getattr(getattr(sensor, "req", None),
                                                                              "forward_overlap", 0.7)))
    return max(0.0, l_fp * (1.0 - fwd))


# ---------------------------------------------------------------------------
# Plan → 任務項目
# ---------------------------------------------------------------------------
def plan_to_mission_items(uav_plan: UAVPlan, ctx: MissionContext, cfg: PlannerConfig,
                          alt_frame: AltFrame = "relative", camera_trigger: bool = True,
                          wp_radius_m: float | None = None, *,
                          strip_sensors: Mapping[str, Sensor] | None = None,
                          z_tol_m: float | None = None,
                          takeoff_delay_s: float | None = None,
                          camera_cfg: CameraTriggerConfig | None = None) -> list[MissionItem]:
    """`UAVPlan` → ArduPilot QuadPlane 任務項目（第 0 列 home）。

    對應：TAKEOFF → NAV_VTOL_TAKEOFF（alt = transition_start_agl）；TRANS_OUT 略；ACCEL／CLIMB／ENTRY／TRANSFER／RTB／
    DESCENT／DECEL → NAV_WAYPOINT（弧段外切離散、直線端點）；ORBIT 整圈 → NAV_LOITER_TO_ALT（param2 = ±R，順時針正）
    ／NAV_LOITER_TURNS／NAV_LOITER_TIME；空速改變 → DO_CHANGE_SPEED(airspeed)；OPERATE 條帶起／訖 →
    DO_SET_CAM_TRIGG_DIST(dist)／(0)（rgb）或 DO_SET_SERVO 開／關（lidar）。**DO_CHANGE_SPEED 與酬載開／關一律置於
    其所描述之「出發腿」起點 NAV 之後**（ArduPilot 於抵達前一個 NAV 時即執行其後之 DO_；`Waypoint.airspeed_mps`／
    `Waypoint.sensor_on` 描述該點之出發腿），與 `integration/mqtt_bridge` 之順序一致（D45）；
    第一個降落段航點前 → DO_LAND_START；
    TRANS_IN 略；LAND → NAV_VTOL_LAND（落點；alt 0 相對 home）。

    `alt_frame`：relative（alt = MSL − **本機起飛坪** MSL）、msl、terrain（alt = AGL）。`wp_radius_m` None → min(R_eff/2, 30)。

    **高度基準與第 0 列**：ArduPlane 解鎖時把 home 重設到飛機自己的位置（見 :func:`fc_home`），所以相對高度一律以
    **這架的起飛坪**（``uav_plan.takeoff_site``）MSL 為基準，第 0 列 home 也寫這個坪的經緯度與 MSL（frame 0）。
    逐機起飛站在斜坡上時，舊版以規劃 home 為基準會讓整份任務偏移 (坪 − home)；平地時兩者相同、輸出不變。
    NAV_VTOL_LAND 的高度 = 降落站地面 − 起飛坪（降回自己的起飛坪 ⇒ 0）。
    `strip_sensors`（strip_id → Sensor，planner 之 decompose.resolve_sensor 結果）決定觸發距離；缺省依機隊主感測種類。
    `takeoff_delay_s` > 0 時於 home 列之後、NAV_VTOL_TAKEOFF 之前插入 MAV_CMD_NAV_DELAY(93)（param1 = 秒；
    param2–4 = −1 表「不使用時刻」），使多機任務保留計畫之錯開起飛時刻（D46）。

    **DO_LAND_START 與相機**：`RTL_AUTOLAND=2` 下失效保護 RTL 會跳到 DO_LAND_START 並維持 AUTO（CAM_AUTO_ONLY 擋不住），
    跳轉會略過其前所有條帶的 206(0)。因此只要本任務開過相機，DO_LAND_START **緊接**一個 206(0)。若到達第一個降落段
    航點時相機條帶仍開著（該點即條帶終點＝RTB 首點），DO_LAND_START 改放在該點 NAV（及其 DO_CHANGE_SPEED）**之後**，
    其後的 206(0) 兼作該條帶之關閉——放在該點 NAV 之前會落在開相機那一腿之內，且其後的 206(0) 會在抵達條帶起點時就
    一併執行（同一 NAV 之後的 DO_ 依序連續執行），整條帶就只剩一張。代價：跳轉時略過該點之 DO_CHANGE_SPEED。

    `camera_cfg`（:class:`CameraTriggerConfig`；None ＝ 現行行為）：
    * ``first_shot_immediate`` → 開啟之 206 的 p3；
    * ``camera_lead_in_m`` > 0 → 於條帶起點 NAV 之後、沿條帶 lead_in 處插一個共線 NAV_WAYPOINT（同高度、同框架、
      param2 = :data:`LEAD_IN_ACCEPT_RADIUS_M` = 1 m），206 開改掛在它之後。ArduPlane 於「抵達」該 NAV 時執行其後的
      DO_，抵達＝進入 LOWBYTE(param2) 公尺內或越過終點線 ⇒ 開拍點在帶起點後 ≈ lead_in（param2 若用 r_acc 30 m，
      開拍點只剩 lead_in − 30）。條帶短於 2·lead_in 或首段短於 lead_in 時不插並記 WARNING；
    * 給了 ``camera_cfg`` 時，206 的 d 用 :func:`_trigger_agl`（AGL 與規劃作業高度差 ≤ :data:`AGL_SNAP_TOL_M` ⇒ 作業高度），
      平地各條帶的 p1 逐位元相同；``camera_cfg`` None ＝ 舊行為（該點 AGL）；
    * ``terrain_retrigger_tol`` → 條帶**中途既有**之 OPERATE 航點，其 AGL 與目前 d 所依據之 AGL 相對差 > tol 時，於該點
      NAV 之後重發 206(d_new, 0, 0, 0)。不另造航點：只有起訖兩點的條帶（平直條帶的常態）不會重發。
    """
    if alt_frame not in FRAME_OF:
        raise ValueError(f"alt_frame must be one of {tuple(FRAME_OF)}, got {alt_frame!r}")
    if camera_cfg is not None and not isinstance(camera_cfg, CameraTriggerConfig):
        raise TypeError(f"camera_cfg 須為 CameraTriggerConfig，得 {type(camera_cfg).__name__}")
    if camera_cfg is not None and not camera_trigger:
        _log.warning("%s: camera_trigger=False ⇒ camera_cfg 之任務項目設定（p3／lead-in／地形重發）不套用",
                     uav_plan.uav_id)
    cc = camera_cfg if camera_trigger else None
    p3_first = 1.0 if (cc is None or cc.first_shot_immediate) else 0.0
    lead_in = float(cc.camera_lead_in_m) if cc is not None else 0.0
    rt_tol = cc.terrain_retrigger_tol if cc is not None else None
    frame = FRAME_OF[alt_frame]
    spec = ctx.specs[uav_plan.cls]
    w_ref = ctx.wind.max_speed() if ctx.wind is not None else 0.0
    r_eff = spec.r_eff(w_ref)
    r_acc = accept_radius_m(r_eff, wp_radius_m)
    # 相對高度基準＝本機起飛坪（飛控解鎖時 home 會被重設到這裡；見 fc_home），不是規劃 home
    ref_site, ref_lat, ref_lon, home_msl = fc_home(ctx, uav_plan)
    _plan_home_msl = _home_msl(ctx)
    if alt_frame == "relative" and abs(home_msl - _plan_home_msl) > 1e-6:
        _log.info("%s: 相對高度以起飛坪 %s（MSL %.2f m）為基準；規劃 home MSL %.2f m，差 %+.2f m（舊版以規劃 home 為"
                  "基準時整份任務會偏移這麼多）", uav_plan.uav_id, ref_site, home_msl, _plan_home_msl,
                  home_msl - _plan_home_msl)
    items: list[MissionItem] = [_item(0, MAV_CMD_NAV_WAYPOINT, MAV_FRAME_GLOBAL, x=ref_lat, y=ref_lon,
                                      z=home_msl, current=1)]
    wps = uav_plan.waypoints
    if not wps:
        return items
    if takeoff_delay_s is not None and float(takeoff_delay_s) > 0.0:
        # NAV_DELAY：param1 = 延遲秒數；param2/3/4 = −1（不使用「當日時刻」形式）
        items.append(_item(0, MAV_CMD_NAV_DELAY, MAV_FRAME_GLOBAL_RELATIVE_ALT,
                           (float(takeoff_delay_s), -1.0, -1.0, -1.0)))
    pts = reduce_waypoints(wps, spec, cfg, r_acc, z_tol_m=z_tol_m)
    xy = np.array([[p.x, p.y] for p in pts], dtype=float)
    wgs = _wgs(ctx, xy, np.array([p.z for p in pts]))

    def alt_of(k: int) -> float:
        if alt_frame == "relative":
            return float(wgs[k, 2] - home_msl)
        if alt_frame == "msl":
            return float(wgs[k, 2])
        return float(pts[k].agl)

    v_cmd: float | None = None
    land_start_done = False
    cam_on = False
    cam_used = False                  # 本任務已開過相機（DO_LAND_START 之後須補 206(0)）
    lidar_on = False
    cur_strip: str | None = None
    trig_kind: str | None = None
    cam_subs: tuple[Any, ...] = ()    # 目前條帶之 rgb 感測器（地形重發 206 用）
    cam_agl_ref = 0.0                 # 目前 206 之 d 所依據之 AGL

    def strip_subs(k: int) -> tuple[Any, tuple[Any, ...]]:
        sensor = _sensor_for(uav_plan, ctx, pts[k].strip_id, strip_sensors)
        # 規格 §6.2：同機雙感測（`platform.CombinedSensor`）一次飛行同時開兩個酬載 → 兩個 DO_ 皆送出。
        return sensor, tuple(getattr(sensor, "sensors", None) or ((sensor,) if sensor is not None else ()))

    def cam_on_item(dist: float, p3: float) -> None:
        nonlocal cam_on, cam_used
        if land_start_done:
            # 失效保護跳到 DO_LAND_START 後會依序執行到這一項 ⇒ 降落途中開相機。正常計畫不會發生（OPERATE 不屬降落段）
            raise ValueError(f"{uav_plan.uav_id}: 條帶 {cur_strip} 的 206 開位於 DO_LAND_START 之後（計畫在降落段之後"
                             "又有作業段？）；拒絕輸出")
        items.append(_item(0, MAV_CMD_DO_SET_CAM_TRIGG_DIST, MAV_FRAME_GLOBAL_RELATIVE_ALT, (dist, 0.0, p3, 0.0)))
        cam_on = True
        cam_used = True

    def open_strip(k: int, defer_cam: bool = False) -> list[float]:
        """送出本條帶之酬載開指令；`defer_cam` 時相機之 206 不送，回傳其距離（由呼叫端掛在 lead-in NAV 之後）。"""
        nonlocal lidar_on, cur_strip, trig_kind, cam_subs, cam_agl_ref
        p = pts[k]
        sensor, subs = strip_subs(k)
        cur_strip = p.strip_id
        trig_kind = getattr(sensor, "kind", None) if sensor is not None else None
        if not subs:
            items.append(_item(0, MAV_CMD_DO_SET_SERVO, MAV_FRAME_GLOBAL_RELATIVE_ALT,
                               (float(LIDAR_SERVO_CHANNEL), float(LIDAR_SERVO_PWM_ON), 0.0, 0.0)))
            lidar_on = True
            return []
        pending: list[float] = []
        cam_subs = tuple(s for s in subs if getattr(s, "kind", None) == "rgb")
        agl_ref = float(p.agl)
        if is_cross_strip(p.strip_id):
            # 建圖模式交叉航帶（可與主條帶不同高）：帶上若有升降（build_3d 的包絡；縮點後飛控在帶起點與帶終點之間
            # 線性升降，2×2 km 修前實測帶起點 115.4 m、帶上 100 m）⇒ 用相機開著時會飛到的**最低** AGL（本帶 OPERATE
            # 導航點與帶終點）算 d（足跡最小 ⇒ d 最短 ⇒ 前向重疊不低於規劃）。主條帶不走這裡（舊行為逐位元不變）。
            j = k
            while j < len(pts) and pts[j].phase is Phase.OPERATE and pts[j].strip_id == p.strip_id:
                agl_ref = min(agl_ref, float(pts[j].agl))
                j += 1
            if j < len(pts):
                # 帶終點 ＝ 下一個導航點（206(0) 掛在它之後；飛控在兩個 NAV 之間線性升降）⇒ 相機開著時飛到的最低高度
                agl_ref = min(agl_ref, float(pts[j].agl))
        cam_agl_ref = agl_ref
        for sub in subs:
            if getattr(sub, "kind", None) == "rgb":
                dist = trigger_distance_m(sub, _trigger_agl(sub, agl_ref) if cc is not None else agl_ref)
                if defer_cam:
                    pending.append(dist)
                else:
                    cam_on_item(dist, p3_first)
            else:   # lidar（或未知）→ 伺服佔位
                items.append(_item(0, MAV_CMD_DO_SET_SERVO, MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                   (float(LIDAR_SERVO_CHANNEL), float(LIDAR_SERVO_PWM_ON), 0.0, 0.0)))
                lidar_on = True
        return pending

    def lead_in_xy(k: int) -> np.ndarray | None:
        """條帶起點 k 沿首段 lead_in 處之 ENU 座標；條帶 < 2·lead_in 或首段 ≤ lead_in → None（記 WARNING）。"""
        sid = pts[k].strip_id
        j = k + 1                                       # 條帶終點＝第一個不屬本條帶之點（下一 run 首點）
        while j < len(pts) and pts[j].kind == "wp" and pts[j].sensor_on and pts[j].phase is Phase.OPERATE \
                and pts[j].strip_id == sid:
            j += 1
        j = min(j, len(pts) - 1)
        if j <= k:
            _log.warning("%s: 條帶 %s 找不到終點，不插 lead-in（%.1f m）", uav_plan.uav_id, sid, lead_in)
            return None
        length = float(np.hypot(*np.diff(xy[k:j + 1], axis=0).T).sum())
        seg = xy[k + 1] - xy[k]
        s1 = float(np.hypot(*seg))
        if length < 2.0 * lead_in or s1 <= lead_in:
            _log.warning("%s: 條帶 %s 長 %.1f m（首段 %.1f m）< 2×lead-in %.1f m（或首段 ≤ lead-in）⇒ 不插 lead-in，"
                         "206 開仍在條帶起點；建圖模式應把條帶外延同樣距離", uav_plan.uav_id, sid, length, s1, lead_in)
            return None
        return xy[k] + (lead_in / s1) * seg

    def close_strip() -> None:
        nonlocal cam_on, lidar_on, cur_strip
        if cam_on:
            items.append(_item(0, MAV_CMD_DO_SET_CAM_TRIGG_DIST, MAV_FRAME_GLOBAL_RELATIVE_ALT, (0.0, 0.0, 0.0, 0.0)))
            cam_on = False
        if lidar_on:
            items.append(_item(0, MAV_CMD_DO_SET_SERVO, MAV_FRAME_GLOBAL_RELATIVE_ALT,
                               (float(LIDAR_SERVO_CHANNEL), float(LIDAR_SERVO_PWM_OFF), 0.0, 0.0)))
            lidar_on = False
        cur_strip = None

    def land_start() -> None:
        """DO_LAND_START；任務開過相機 ⇒ 緊接 206(0)（失效保護跳到此處時會略過所有條帶的關閉；若此刻條帶仍開著，
        這個 206(0) 就是它的關閉）。"""
        nonlocal land_start_done, cam_on
        items.append(_item(0, MAV_CMD_DO_LAND_START, MAV_FRAME_GLOBAL_RELATIVE_ALT))
        land_start_done = True
        if camera_trigger and cam_used:
            items.append(_item(0, MAV_CMD_DO_SET_CAM_TRIGG_DIST, MAV_FRAME_GLOBAL_RELATIVE_ALT, (0.0, 0.0, 0.0, 0.0)))
            cam_on = False

    for k, p in enumerate(pts):
        lat, lon = float(wgs[k, 0]), float(wgs[k, 1])
        if p.kind == "takeoff":
            h_ts = float(spec.transition_start_agl_m)
            if alt_frame == "terrain":
                z_to = h_ts
            elif alt_frame == "msl":
                z_to = float(wgs[k, 2]) + h_ts
            else:
                z_to = float(wgs[k, 2]) + h_ts - home_msl
            items.append(_item(0, MAV_CMD_NAV_VTOL_TAKEOFF, frame, (0.0, 0.0, 0.0, 0.0), lat, lon, z_to))
            continue
        if p.kind == "land":
            if not land_start_done and cam_on:
                land_start()                           # 其後之 206(0) 即本條帶之關閉
            if camera_trigger and (cam_on or lidar_on):
                close_strip()
            if not land_start_done:
                land_start()
            # 落點高度 = 該站地面（LAND 航點本身位於 H_ts 之懸停下降起點，不可直接用其 z）
            if uav_plan.landing_site == ref_site:
                site_msl = home_msl                    # 降回自己的起飛坪（預設）⇒ 相對高 0（不是 1e-9 m 級換算雜訊）
            elif uav_plan.landing_site in ctx.sites_enu:
                site_msl = _site_msl(ctx, uav_plan.landing_site)
            else:
                site_msl = float(wgs[k, 2]) - float(spec.transition_start_agl_m)
            z_ld = 0.0 if alt_frame == "terrain" else (site_msl if alt_frame == "msl" else site_msl - home_msl)
            items.append(_item(0, MAV_CMD_NAV_VTOL_LAND, frame, (0.0, 0.0, 0.0, 0.0), lat, lon, z_ld))
            continue
        # ---- 條帶開／關（實際 append 在本點之 NAV 之後，見下）----
        want_open = bool(camera_trigger and p.sensor_on and p.phase is Phase.OPERATE and p.strip_id is not None
                         and cur_strip != p.strip_id)
        want_close = bool(camera_trigger and (cam_on or lidar_on) and (want_open or not (
            p.sensor_on and p.phase is Phase.OPERATE and p.strip_id == cur_strip)))
        # ---- 降落序列：相機條帶仍開著（本點＝條帶終點）⇒ DO_LAND_START 延到本點 NAV 之後（見 docstring）----
        land_start_after = False
        if not land_start_done and p.phase in _LANDING_PHASES:
            if cam_on:
                land_start_after = True
            else:
                land_start()
        # ---- 空速（描述本點之出發腿 ⇒ 與酬載開關同樣置於本點 NAV 之後，見下）----
        want_speed = bool(p.mode is Mode.CRUISE and p.v_air > 0 and (v_cmd is None or abs(p.v_air - v_cmd) > 0.05))
        # ---- 導航點 ----
        alt = alt_of(k)
        if p.kind == "loiter_alt":
            # NAV_LOITER_TO_ALT：ArduPlane 讀 param2 為半徑（帶號＝方向）
            items.append(_item(0, MAV_CMD_NAV_LOITER_TO_ALT, frame, (0.0, p.radius_signed, 0.0, 1.0), lat, lon, alt))
        elif p.kind == "loiter_turns":
            # NAV_LOITER_TURNS：ArduPlane 以 |param3| 取整（≤ 255 m）為半徑，param3 < 0 為逆時針
            items.append(_item(0, MAV_CMD_NAV_LOITER_TURNS, frame,
                               (float(p.laps), 0.0, _loiter_turns_radius(p.radius_signed), 1.0), lat, lon, alt))
        elif p.kind == "loiter_time":
            # NAV_LOITER_TIME：ArduPlane 只讀 param3 之「正負號」（方向），半徑一律用 WP_LOITER_RAD
            # ⇒ recommended_params 之 WP_LOITER_RAD 取本計畫實際盤旋半徑（見 orbit_radius_m）
            items.append(_item(0, MAV_CMD_NAV_LOITER_TIME, frame, (float(p.duration_s), 0.0, p.radius_signed, 1.0),
                               lat, lon, alt))
        else:
            # 給了 camera_cfg 而沒有引入段（lead_in = 0）：206 開直接掛在本點（帶起點）之後 ⇒ 本點也用 1 m 接受半徑，
            # 否則 r_acc（≤ 30 m）會讓相機在帶起點前 30 m（轉彎裡）就開、p3=1 那張也不受 CAM_MAX_ROLL 擋
            r_here = r_acc
            if (want_open and cc is not None and lead_in <= 0.0
                    and any(getattr(s, "kind", None) == "rgb" for s in strip_subs(k)[1])):
                r_here = LEAD_IN_ACCEPT_RADIUS_M
            items.append(_item(0, MAV_CMD_NAV_WAYPOINT, frame, (0.0, r_here, 0.0, 0.0), lat, lon, alt))
        # ArduPilot 於「抵達前一個 NAV」時執行其後之 DO_ 指令 ⇒ 描述本點**出發腿**的指令必須放在本點 NAV **之後**
        if want_speed:
            items.append(_item(0, MAV_CMD_DO_CHANGE_SPEED, MAV_FRAME_GLOBAL_RELATIVE_ALT, (0.0, float(p.v_air), -1.0, 0.0)))
            v_cmd = float(p.v_air)
        if land_start_after:
            land_start()                               # 其後之 206(0) 即本條帶相機之關閉
        if want_close:
            close_strip()
        if want_open:
            lead = None
            if lead_in > 0.0 and any(getattr(s, "kind", None) == "rgb" for s in strip_subs(k)[1]):
                lead = lead_in_xy(k)
            pending = open_strip(k, defer_cam=lead is not None)
            if lead is not None:
                # 共線 lead-in 航點：同高度、同框架；206 開掛在它之後（轉彎改出後才開拍）。接受半徑 1 m（不是 r_acc）：
                # ArduPlane 在 LOWBYTE(param2) 公尺內或越過終點線才算抵達，r_acc 30 m 會讓 206 開提早 30 m 執行
                ll = _wgs(ctx, lead, p.z)[0]
                items.append(_item(0, MAV_CMD_NAV_WAYPOINT, frame, (0.0, LEAD_IN_ACCEPT_RADIUS_M, 0.0, 0.0),
                                   float(ll[0]), float(ll[1]), alt))
                for dist in pending:
                    cam_on_item(dist, p3_first)
        elif cam_on and rt_tol is not None and not want_close and p.strip_id == cur_strip:
            # 條帶中途既有之 OPERATE 航點：AGL 偏離目前 d 所依據者 > tol ⇒ 重發 206(d_new)（p3 = 0，不另拍）
            if abs(float(p.agl) - cam_agl_ref) > float(rt_tol) * max(abs(cam_agl_ref), 1e-9):
                for sub in cam_subs:
                    items.append(_item(0, MAV_CMD_DO_SET_CAM_TRIGG_DIST, MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                       (trigger_distance_m(sub, _trigger_agl(sub, p.agl)), 0.0, 0.0, 0.0)))
                cam_agl_ref = float(p.agl)
    if camera_trigger and (cam_on or lidar_on):
        close_strip()
    return renumber(items)


# ---------------------------------------------------------------------------
# QGC WPL 110
# ---------------------------------------------------------------------------
def _fmt(v: float) -> str:
    s = f"{v:.8f}".rstrip("0").rstrip(".")
    return s if s not in ("", "-0") else "0"


def write_qgc_wpl(items: Sequence[MissionItem], path: str | os.PathLike, comments: Sequence[str] = ()) -> str:
    """寫出 QGC WPL 110（12 欄 tab 分隔：seq current frame cmd p1 p2 p3 p4 lat lon alt autocontinue；lat/lon 7 位）。

    `comments` 於版本列之後以 `#` 開頭寫出（pymavlink `mavwp.MAVWPLoader` 與本模組 :func:`read_qgc_wpl` 皆會略過）。
    **一律轉為 ASCII**：`mavwp.MAVWPLoader.load` 以系統預設編碼（Windows 為 cp950）開檔，非 ASCII 註解會讓它
    丟 UnicodeDecodeError。
    """
    lines = [QGC_WPL_HEADER]
    for c in comments:
        for ln in str(c).splitlines():
            lines.append("# " + ln.encode("ascii", "replace").decode("ascii"))
    for it in items:
        lines.append("\t".join([str(int(it.seq)), str(int(it.current)), str(int(it.frame)), str(int(it.command)),
                                _fmt(it.param1), _fmt(it.param2), _fmt(it.param3), _fmt(it.param4),
                                f"{it.x:.7f}", f"{it.y:.7f}", f"{it.z:.6f}", str(int(it.autocontinue))]))
    text = "\n".join(lines) + "\n"
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    return str(path)


def read_qgc_wpl(path: str | os.PathLike, mission_type: int = 0) -> list[MissionItem]:
    """讀回 QGC WPL 110 → `list[MissionItem]`（格式錯誤丟 ValueError）。"""
    with open(path, "r", encoding="utf-8") as f:
        header = f.readline().strip()
        if header != QGC_WPL_HEADER:
            raise ValueError(f"unsupported waypoint file header {header!r}")
        out: list[MissionItem] = []
        for ln, line in enumerate(f, start=2):
            if not line.strip() or line.startswith("#"):
                continue
            a = line.split()
            if len(a) != 12:
                raise ValueError(f"line {ln}: expected 12 columns, got {len(a)}")
            out.append(MissionItem(int(a[0]), int(a[2]), int(a[3]), int(a[1]), int(a[11]), float(a[4]), float(a[5]),
                                   float(a[6]), float(a[7]), float(a[8]), float(a[9]), float(a[10]), int(mission_type)))
    return out


# ---------------------------------------------------------------------------
# Rally / Fence
# ---------------------------------------------------------------------------
def rally_items(inputs: MissionInputs, ctx: MissionContext, alt_frame: AltFrame = "relative", *,
                ref_msl_m: float | None = None) -> list[MissionItem]:
    """NAV_RALLY_POINT(5100)：home + 全部備降點；alt = 該站地面 + max_cls(H_tc,cmd)。

    ArduPilot 之 `RallyLocation` **沒有 MSL 框架**（高度為 int16「相對 home」，terrain 另以旗標表示），故
    `alt_frame='msl'` 亦輸出相對 home 高度並使用 MAV_FRAME_GLOBAL_RELATIVE_ALT；只有 `'terrain'` 用
    MAV_FRAME_GLOBAL_TERRAIN_ALT（AGL）。D47。

    `ref_msl_m`：「相對 home」的 home MSL。飛控的 home 是它自己的起飛坪（:func:`fc_home`），逐機上傳／匯出時要給
    該機起飛坪的 MSL；None ＝ 規劃 home（舊行為，平地兩者相同）。
    """
    frame = FRAME_OF["terrain"] if alt_frame == "terrain" else MAV_FRAME_GLOBAL_RELATIVE_ALT
    home_msl = _home_msl(ctx) if ref_msl_m is None else float(ref_msl_m)
    h_tc = max(sp.transition_cmd(transition_profile=ctx.cfg.transition_profile).h_tc_cmd_m for sp in ctx.specs.values()) \
        if ctx.specs else 100.0
    out: list[MissionItem] = []
    for site in (inputs.home, *inputs.alternates):
        if site.site_id not in ctx.sites_enu:
            continue
        msl = _site_msl(ctx, site.site_id)
        alt = h_tc if alt_frame == "terrain" else (msl + h_tc - home_msl)
        out.append(_item(len(out), MAV_CMD_NAV_RALLY_POINT, frame, x=site.lat_deg, y=site.lon_deg, z=alt,
                         mission_type=MAV_MISSION_TYPE_RALLY))
    return out


def _largest_polygon(g: shapely.Geometry) -> Polygon | None:
    if g is None or g.is_empty:
        return None
    if isinstance(g, Polygon):
        return g
    if isinstance(g, MultiPolygon):
        return max(g.geoms, key=lambda p: p.area)
    polys = [p for p in getattr(g, "geoms", []) if isinstance(p, Polygon)]
    return max(polys, key=lambda p: p.area) if polys else None


def _ring_latlon(ctx: MissionContext, poly: Polygon, simplify_m: float = 5.0) -> list[tuple[float, float]]:
    ring = shapely.Polygon(poly.exterior).simplify(simplify_m, preserve_topology=True)
    coords = np.asarray(ring.exterior.coords, dtype=float)[:-1]
    if len(coords) >= 3 and shapely.LinearRing(coords).is_ccw is False:
        coords = coords[::-1]
    ll = _wgs(ctx, coords[:, :2], 0.0)
    return [(float(a), float(b)) for a, b in ll[:, :2]]


def fence_buffer_m(ctx: MissionContext, cls: str) -> float:
    """inclusion 外擴量 = max(2·R_eff, d_turn(s→0) = R_eff(1+√3) + e_ext,max)（D38：Ω 轉場超出端線量）。"""
    spec = ctx.specs[cls]
    w_ref = ctx.wind.max_speed() if ctx.wind is not None else 0.0
    r_eff = spec.r_eff(w_ref)
    v_g_max = spec.cruise_airspeed_mps + w_ref
    e_ext = 0.0
    for (c, _rid), sensor in ctx.sensors.items():
        if c == cls:
            try:
                e_ext = max(e_ext, float(sensor.end_extension_m(v_g_max, ctx.cfg.t_settle_s)))
            except Exception:  # noqa: BLE001
                pass
    if ctx.cfg.e_ext_override_m is not None:
        e_ext = float(ctx.cfg.e_ext_override_m)
    return max(2.0 * r_eff, d_turn_from_r(r_eff, 0.0, e_ext))


def fence_polygons(inputs: MissionInputs, ctx: MissionContext, cls: str,
                   plans: Iterable[UAVPlan] | None = None) -> tuple[Polygon | None, list[Polygon]]:
    """(inclusion, exclusions)（ENU）：inclusion = ConvexHull(區域 ∪ 站點 ⊕ 起降走廊 ∪ 航線 ⊕ b_trk) ⊕ max(2·R_eff, d_turn)
    ∩ P_link,c；exclusion = 有效禁飛區（未加 b_trk；航線已由規劃器保持 ⊕ b_trk 淨空）。"""
    spec = ctx.specs[cls]
    w_ref = ctx.wind.max_speed() if ctx.wind is not None else 0.0
    r_eff = spec.r_eff(w_ref)
    tc = spec.transition_cmd(transition_profile=ctx.cfg.transition_profile)
    d_corr = tc.d_tr_m + tc.d_acc_m + 2.0 * r_eff          # 起降走廊（轉換＋加速段＋進入／下降迴圈）可及範圍
    geoms: list[shapely.Geometry] = list(ctx.areas_enu.values())
    geoms += [shapely.Point(float(xy[0]), float(xy[1])).buffer(d_corr, quad_segs=8) for xy, _z in ctx.sites_enu.values()]
    for up in (plans or []):
        if up.cls == cls and len(up.waypoints) >= 2:
            geoms.append(shapely.LineString([(w.x_m, w.y_m) for w in up.waypoints]).buffer(ctx.cfg.b_trk_m, quad_segs=4))
    hull = unary_union(geoms).convex_hull.buffer(fence_buffer_m(ctx, cls), quad_segs=16)
    link = ctx.link_poly.get(cls) if ctx.link_poly else None
    incl = hull if (link is None or link.is_empty) else hull.intersection(link)
    inclusion = _largest_polygon(incl)
    excl: list[Polygon] = []
    for nf in inputs.airspace.active_no_fly(ctx.t_window) if ctx.cfg.enforce_time_windows else inputs.airspace.no_fly:
        ring = np.asarray(nf.polygon_wgs84, dtype=float)
        xy = ctx.frame.to_enu(ring[:, 0], ring[:, 1])[:, :2]
        p = Polygon(xy)
        if p.is_valid and not p.is_empty:
            excl.append(p)
    return inclusion, excl


def fence_items(inputs: MissionInputs, ctx: MissionContext, cls: str,
                plans: Iterable[UAVPlan] | None = None) -> list[MissionItem]:
    """FENCE 項目：NAV_FENCE_RETURN_POINT(5000)=home → inclusion 頂點 5001（param1=頂點數） → exclusion 5002。
    `plans`（該構型之 UAVPlan）給定時 inclusion 亦涵蓋航線 ⊕ b_trk。"""
    inclusion, exclusions = fence_polygons(inputs, ctx, cls, plans)
    out: list[MissionItem] = [_item(0, MAV_CMD_NAV_FENCE_RETURN_POINT, MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                    x=inputs.home.lat_deg, y=inputs.home.lon_deg, z=0.0,
                                    mission_type=MAV_MISSION_TYPE_FENCE)]
    if inclusion is not None:
        ll = _ring_latlon(ctx, inclusion)
        for lat, lon in ll:
            out.append(_item(len(out), MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, MAV_FRAME_GLOBAL_RELATIVE_ALT,
                             (float(len(ll)), 0.0, 0.0, 0.0), lat, lon, 0.0, mission_type=MAV_MISSION_TYPE_FENCE))
    for poly in exclusions:
        ll = _ring_latlon(ctx, poly, simplify_m=0.5)
        for lat, lon in ll:
            out.append(_item(len(out), MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, MAV_FRAME_GLOBAL_RELATIVE_ALT,
                             (float(len(ll)), 0.0, 0.0, 0.0), lat, lon, 0.0, mission_type=MAV_MISSION_TYPE_FENCE))
    return out


# ---------------------------------------------------------------------------
# 參數建議
# ---------------------------------------------------------------------------
PARAM_COMMENTS: dict[str, str] = {
    "AIRSPEED_CRUISE": "cruise_airspeed_mps (Table 1) — TECS target airspeed in AUTO [m/s] (old TRIM_ARSPD_CM cm/s)",
    "AIRSPEED_MIN": "v_min_mission (min_mission_airspeed_mps upper bound) [m/s] (old ARSPD_FBW_MIN). AP_Int16 on the "
                    "vehicle: a fractional value is exported rounded UP (never below the planned minimum)",
    "AIRSPEED_MAX": "max_airspeed_mps; provisional when Table 1 value is None (1.4*cruise) [m/s] (old ARSPD_FBW_MAX). "
                    "AP_Int16 on the vehicle: a fractional value is exported rounded DOWN (26.6 -> 26)",
    "AIRSPEED_STALL": "stall_airspeed_mps [m/s] (ArduPlane 4.6.0+: added in 4.6.0-beta1; a 4.5.x vehicle has no "
                      "such parameter and refuses it)",
    "TECS_CLMB_MAX": "max_fw_climb_rate_mps = Va*sin(planning_climb_angle) [m/s]",
    "TECS_SINK_MAX": "max_fw_descent_rate_mps = Va*sin(planning_descent_angle) [m/s]",
    "TECS_SINK_MIN": "minimum sink rate at cruise (0.9*planning descent rate) [m/s]",
    "ROLL_LIMIT_DEG": "max_bank_angle_deg [deg] (old LIM_ROLL_CD centideg)",
    "WP_LOITER_RAD": "loiter radius [m]; positive = clockwise. ArduPlane flies NAV_LOITER_TIME/TURNS at THIS radius "
                     "(NAV_LOITER_TIME param3 only carries the direction sign), so it is set to the radius the "
                     "planner used for the delay orbits when the mission has any, else R_eff (wind-inflated). "
                     "AP_Int16: exported rounded UP in magnitude (never tighter than R_eff)",
    "WP_RADIUS": "waypoint acceptance radius [m]. ArduPlane uses a NON-ZERO NAV_WAYPOINT param2 (integer m, <= 255) "
                 "as that waypoint's acceptance radius and falls back to WP_RADIUS only when param2 = 0 "
                 "(commands_logic.cpp verify_nav_wp); this exporter sets param2 = r_acc on every NAV_WAYPOINT "
                 "except the camera lead-in NAV (param2 = 1). AP_Int16: exported rounded DOWN. "
                 "Still kept below 0.8x the smallest consecutive NAV spacing of this mission "
                 "(cap r_acc = min(R_eff/2, 30), floor 10) so a waypoint is never 'reached' together with its "
                 "predecessor and its attached DO_ commands fire on position",
    "RTL_RADIUS": "RTL loiter radius = R_eff [m]; AP_Int16: exported rounded UP (never tighter than R_eff)",
    "Q_ENABLE": "QuadPlane enabled (lift-and-cruise 4+1)",
    "Q_TRANSITION_MS": "forward transition timeout = t_tr_out_s * 1000 [ms]",
    "Q_ASSIST_SPEED": "VTOL assist below this airspeed [m/s]; MUST stay below AIRSPEED_MIN (else the lift motors "
                      "spin up whenever TECS regulates down to its minimum) and above stall: "
                      "min(v_tr, AIRSPEED_MIN - 1) floored at 1.15*AIRSPEED_STALL",
    "Q_RTL_ALT": "QRTL approach altitude = min_fw_operation_agl_m [m AGL]; AP_Int16: exported rounded UP",
    "Q_RTL_MODE": "1 = an RTL becomes QRTL within RTL_RADIUS of home (a QuadPlane RTL otherwise ends in a "
                  "fixed-wing loiter over home). Required for the RTL selected by FENCE_ACTION / FS_LONG_ACTN / "
                  "BATT_FS_LOW_ACT to end in a VTOL landing",
    "Q_LAND_FINAL_ALT": "final vertical descent phase altitude (10 m rule of hover_descent_time_s) [m]",
    "RTL_AUTOLAND": "2 = go directly to the landing sequence at DO_LAND_START (every exported mission contains one; "
                    "with the default 0 the sequence is never used on RTL and arming is refused)",
    "FENCE_ENABLE": "geofence enabled",
    "FENCE_TYPE": "bitmask: 1 = max alt, 4 = polygon (inclusion/exclusion)",
    "FENCE_ALT_MAX": "max altitude fence [m ABOVE HOME] (AC_Fence compares relative alt, NOT AGL): "
                     "max(plan alt_msl - home_msl) + b_v, at least the airspace ceiling h_ceil",
    "FENCE_ACTION": "1 = RTL on breach",
    "FENCE_RET_RALLY": "1 = return to the nearest rally point (this export ships a rally set) instead of the fence "
                       "return point",
    "BATT_CAPACITY": "pack nominal capacity = nominal_wh / pack_voltage * 1000 [mAh]",
    "BATT_LOW_MAH": "low battery threshold = (reserve + e_LDG) in mAh — planner reserve r*E_nom plus landing energy",
    "BATT_CRT_MAH": "critical battery threshold = reserve r*E_nom in mAh",
    "BATT_FS_LOW_ACT": "low battery action, ArduPLANE enum (0:None 1:RTL 2:Land 3:Terminate 4:QLand 6:Loiter to "
                       "QLand — NOT the Copter enum): 1 = RTL. Needs BATT_LOW_MAH (or BATT_LOW_VOLT) to trigger",
    "BATT_FS_CRT_ACT": "critical battery action, ArduPlane enum: 4 = QLand (3 is Terminate — flight termination). "
                       "Needs BATT_CRT_MAH (or BATT_CRT_VOLT) to trigger",
    "FS_LONG_ACTN": "long GCS failsafe action from lost_link_policy: 0 = continue (CONTINUE_THEN_RTB), 1 = RTL (RTB/NEAREST_ALT)",
    "FS_LONG_TIMEOUT": "long failsafe timeout = c2.t_ll_s [s]",
    "TERRAIN_ENABLE": "1 when waypoints use MAV_FRAME_GLOBAL_TERRAIN_ALT (alt_frame='terrain')",
    "TERRAIN_FOLLOW": "terrain following in auto modes (1 with terrain frame)",
    "CAM1_TYPE": "camera backend: 1 = Servo, 2 = Relay, 6 = MAVLinkCamV2 (6 needs ArduPlane >= 4.5 and > 1 MB flash) "
                 "(AP_Camera_Params.cpp:11). CAM_* became CAM1_* in ArduPlane 4.4.0 (pre-4.4: CAM_TRIGG_TYPE). Without "
                 "a camera backend the mission's DO_SET_CAM_TRIGG_DIST(206) items are a no-op. CAM/TRIG log records "
                 "also need LOG_BITMASK bit 12 (Camera; ArduPlane Parameters.cpp 4.5.7:627) — NOT set by this file — "
                 "and are not written while disarmed when LOG_DISARMED=0",
    "CAM1_DURATION": "shutter/relay hold time [s] (seconds since ArduPlane 4.4.0; deciseconds before). The backend "
                     "refuses new triggers for DURATION*50 ticks (AP_Camera_Servo/Relay.cpp), so 1.0 would cap the "
                     "rate at ~1 Hz; 0.1 = ArduPilot default",
    "CAM1_INTRVAL_MIN": "minimum time between photos [s]; an earlier trigger is postponed SILENTLY "
                        "(AP_Camera_Backend.cpp 4.5.7:131-136). Must be 1/(proven sustainable capture rate), the same "
                        "rate the planner uses",
    "CAM_AUTO_ONLY": "1 = distance triggering only in AUTO (AP_Camera.cpp 4.7.2:30-35). A failsafe RTL that jumps to "
                     "DO_LAND_START stays in AUTO, so this does not stop it — the 206(0) right after DO_LAND_START does",
    "CAM_MAX_ROLL": "max |roll| [deg] for distance triggers, 0 = no limit (AP_Camera.cpp 4.7.2:22-28); postpones distance "
                    "triggers only (the 206 p3=1 first shot bypasses it)",
    "CAM1_FEEDBAK_PIN": "-1 = no feedback; else the GPIO pin (50 = AUX1 ...) read for the hot-shoe edge: CAM is then "
                        "logged at the edge and TRIG at the command (AP_Camera_Params.cpp:62-68; reboot required)",
    "CAM1_FEEDBAK_POL": "feedback edge polarity: 0 = TriggerLow, 1 = TriggerHigh (AP_Camera_Params.cpp:70-75)",
    "CAM1_TRIGG_DIST": "0: distance triggering is switched on/off per strip by the mission's DO_SET_CAM_TRIGG_DIST(206), "
                       "which writes this value in RAM only",
    "CAM1_RELAY_ON": "1 = relay output High while triggering (AP_Camera_Params.cpp:47-52)",
    "RELAY_PIN": "ArduPlane 4.4.x: GPIO pin of relay instance 0, the only relay the 4.4 camera backend drives "
                 "(AP_Camera_Relay.cpp 4.4.4:17-21, 42-46; AP_Relay.cpp 4.4.4:57-62)",
    f"SERVO{CAMERA_SERVO_CHANNEL}_FUNCTION": "10 = CameraTrigger — the output the camera backend pulses. "
                                             "PLACEHOLDER channel; match the payload wiring",
    f"SERVO{LIDAR_SERVO_CHANNEL}_FUNCTION": "0 = Disabled, i.e. available to mission DO_SET_SERVO(183). Without "
                                            "this ArduPilot refuses the LiDAR record on/off items with "
                                            "'Channel N is already in use'. PLACEHOLDER channel; match the wiring",
}
for _n in range(1, RELAY_NUM_RELAYS + 1):
    PARAM_COMMENTS[f"RELAY{_n}_FUNCTION"] = (
        "4 = Camera (ArduPlane >= 4.5; AP_Relay_Params.cpp:12). With no relay set to Camera the backend still logs "
        "CAM but no pulse leaves the board")
    PARAM_COMMENTS[f"RELAY{_n}_PIN"] = (
        "GPIO pin of this relay (ArduPlane >= 4.5; 50 = AUXOUT1 ...; AP_Relay_Params.cpp:38-59); on ChibiOS its "
        "output must be SERVOx_FUNCTION=-1")
del _n


# ---------------------------------------------------------------------------
# 參數在飛控上的儲存型別（有效值契約：AP_Int* 參數收到小數會被飛控**靜默截斷**——PARAM_SET 經 AP_Param::set_float
# 轉成整數，26.6 存成 26——所以匯出前就取整並記錄有效值）。型別對過 ArduPilot 原始碼（Plane-4.5.7 與 master）：
#   AP_FixedWing.h（airspeed_min/max AP_Int16、loiter_radius AP_Int16、airspeed_cruise/roll_limit AP_Float、
#   airspeed_stall AP_Float〔4.6+〕）、ArduPlane/Parameters.h（waypoint_radius/rtl_radius AP_Int16、
#   terrain_follow AP_Int32、fs_action_long AP_Int8、fs_timeout_long AP_Float、rtl_autoland AP_Enum）、
#   quadplane.h（enable/rtl_mode AP_Int8、qrtl_alt/transition_time_ms AP_Int16、assist_speed/land_final_alt AP_Float）、
#   AP_TECS.h（_maxClimbRate/_maxSinkRate/_minSinkRate AP_Float）、AC_Fence.h（_enabled/_enabled_fences/_action/_ret_rally
#   AP_Int8、_alt_max AP_Float）、AP_BattMonitor_Params.h（_pack_capacity AP_Int32、_low/_critical_capacity AP_Float、
#   _failsafe_*_action AP_Int8）、AP_Terrain.h（enable AP_Int8）、AP_Camera_Params.h（type/relay_on/feedback_pin/
#   feedback_polarity AP_Int8、trigger_duration/interval_min/trigg_dist AP_Float）、AP_Camera.h（_auto_mode_only AP_Int8、
#   _max_roll AP_Int16）、AP_Relay_Params.h（function AP_Enum、pin AP_Int16；4.4 RELAY_PIN AP_Int8）、
#   SRV_Channel.h（function AP_Enum16）。
# 規則：上限往下取（floor）、下限往上取（ceil）——取整後的範圍一定落在平台規格的範圍「之內」；半徑往大取（不要求比
# 有風 R_eff 更急的轉彎）；接受半徑往小取（維持 ≤ r_acc 且 < 0.8×最小 NAV 間距）；高度下限往上取。
# "exact" ＝ 本匯出器只會輸出整數（旗標／列舉／腳號／毫秒）——若出現小數就是內部錯誤，拒絕輸出而不是偷偷截斷。
# ---------------------------------------------------------------------------
_INT_PARAM_RULES: dict[str, tuple[str, str]] = {
    "AIRSPEED_MIN": ("AP_Int16", "ceil"),
    "AIRSPEED_MAX": ("AP_Int16", "floor"),
    "WP_LOITER_RAD": ("AP_Int16", "ceil"),
    "RTL_RADIUS": ("AP_Int16", "ceil"),
    "WP_RADIUS": ("AP_Int16", "floor"),
    "Q_RTL_ALT": ("AP_Int16", "ceil"),
    "Q_TRANSITION_MS": ("AP_Int16", "exact"),
    "CAM_MAX_ROLL": ("AP_Int16", "exact"),
    "Q_ENABLE": ("AP_Int8", "exact"),
    "Q_RTL_MODE": ("AP_Int8", "exact"),
    "RTL_AUTOLAND": ("AP_Enum", "exact"),
    "FENCE_ENABLE": ("AP_Int8", "exact"),
    "FENCE_TYPE": ("AP_Int8", "exact"),
    "FENCE_ACTION": ("AP_Int8", "exact"),
    "FENCE_RET_RALLY": ("AP_Int8", "exact"),
    "FS_LONG_ACTN": ("AP_Int8", "exact"),
    "TERRAIN_ENABLE": ("AP_Int8", "exact"),
    "TERRAIN_FOLLOW": ("AP_Int32", "exact"),
    "BATT_FS_LOW_ACT": ("AP_Int8", "exact"),
    "BATT_FS_CRT_ACT": ("AP_Int8", "exact"),
    "BATT_CAPACITY": ("AP_Int32", "exact"),
    "CAM1_TYPE": ("AP_Int8", "exact"),
    "CAM_AUTO_ONLY": ("AP_Int8", "exact"),
    "CAM1_FEEDBAK_PIN": ("AP_Int8", "exact"),
    "CAM1_FEEDBAK_POL": ("AP_Int8", "exact"),
    "CAM1_RELAY_ON": ("AP_Int8", "exact"),
    "RELAY_PIN": ("AP_Int8", "exact"),
}
_INT_PARAM_PATTERNS: tuple[tuple[re.Pattern[str], tuple[str, str]], ...] = (
    (re.compile(r"RELAY\d+_FUNCTION"), ("AP_Enum", "exact")),
    (re.compile(r"RELAY\d+_PIN"), ("AP_Int16", "exact")),
    (re.compile(r"SERVO\d+_FUNCTION"), ("AP_Enum16", "exact")),
)
FLOAT_PARAMS: frozenset[str] = frozenset({
    "AIRSPEED_CRUISE", "AIRSPEED_STALL", "TECS_CLMB_MAX", "TECS_SINK_MAX", "TECS_SINK_MIN", "ROLL_LIMIT_DEG",
    "Q_ASSIST_SPEED", "Q_LAND_FINAL_ALT", "FENCE_ALT_MAX", "FS_LONG_TIMEOUT", "BATT_LOW_MAH", "BATT_CRT_MAH",
    "CAM1_DURATION", "CAM1_INTRVAL_MIN", "CAM1_TRIGG_DIST",
})
"""本匯出器輸出、在 ArduPlane 上是 AP_Float 的參數（小數原樣有效）。"""
_INT_ROUND_WHY: dict[str, str] = {
    "AIRSPEED_MIN": "a MIN limit is rounded up so the stored envelope never allows less than the planned minimum "
                    "mission airspeed",
    "AIRSPEED_MAX": "a MAX limit is rounded down so the stored envelope never exceeds the platform maximum",
    "WP_LOITER_RAD": "a radius is rounded up (in magnitude) so the vehicle is never asked to turn tighter than R_eff",
    "RTL_RADIUS": "a radius is rounded up so the vehicle is never asked to turn tighter than R_eff",
    "WP_RADIUS": "an acceptance radius is rounded down so it stays <= r_acc and below 0.8x the smallest NAV spacing",
    "Q_RTL_ALT": "an altitude floor is rounded up so QRTL never approaches below min_fw_operation_agl",
}


def _int_rule(name: str) -> tuple[str, str] | None:
    r = _INT_PARAM_RULES.get(name)
    if r is not None:
        return r
    for pat, rule in _INT_PARAM_PATTERNS:
        if pat.fullmatch(name):
            return rule
    return None


def param_storage_type(name: str) -> str | None:
    """本匯出器輸出之參數在 ArduPlane 上的儲存型別（'AP_Float'／'AP_Int8'／'AP_Int16'／'AP_Int32'／'AP_Enum'／
    'AP_Enum16'）；不認得 → None。"""
    if name in FLOAT_PARAMS:
        return "AP_Float"
    r = _int_rule(name)
    return r[0] if r is not None else None


def _round_int_params(params: Mapping[str, float], warn: list[str]) -> dict[str, float]:
    """AP_Int* 參數取整成飛控會存的值（規則見上），每個被改的值記 WARNING（含原值、有效值、型別、理由）。

    浮點誤差（|x − round(x)| < 1e-6）視為整數、不記錄；"exact" 規則的參數出現小數、或參數不在型別表 ⇒ ValueError
    （拒絕輸出，不猜、不偷偷截斷）。"""
    out: dict[str, float] = {}
    for k, v in params.items():
        typ = param_storage_type(k)
        if typ is None:
            raise ValueError(f"{k}：匯出器不知道它在 ArduPlane 的儲存型別（AP_Int*／AP_Float），拒絕輸出——"
                             "請先在 _INT_PARAM_RULES／FLOAT_PARAMS 登記（對過飛控原始碼）")
        x = float(v)
        if typ == "AP_Float":
            out[k] = x
            continue
        if abs(x - round(x)) < 1e-6:
            out[k] = float(round(x))
            continue
        rule = _int_rule(k)
        assert rule is not None
        if rule[1] == "exact":
            raise ValueError(f"{k}={x!r}：ArduPlane 以 {typ} 儲存，小數會被飛控截斷；這個值本應是整數（匯出器內部錯誤），"
                             "拒絕輸出")
        mag = math.ceil(abs(x)) if rule[1] == "ceil" else math.floor(abs(x))
        eff = float(math.copysign(mag, x))
        out[k] = eff
        msg = (f"{k} {x:g} -> {eff:g}: ArduPlane stores {k} as {typ} (a fractional value is silently truncated by "
               f"the vehicle); exported {rule[1]}({x:g}) = {eff:g} because {_INT_ROUND_WHY.get(k, 'see rules')}")
        warn.append(msg)
        _log.warning("參數取整（有效值）：%s %g → %g（飛控以 %s 儲存，小數會被截斷；規則 %s）", k, x, eff, typ, rule[1])
    return out


def _check_speed_window(p: Mapping[str, float], uav_plan: UAVPlan | None, warn: list[str]) -> None:
    """取整後的 AIRSPEED_MIN／MAX 是否仍容得下巡航空速與任務的空速指令。

    ArduPlane ``do_change_speed`` 只接受 ``AIRSPEED_MIN <= v <= AIRSPEED_MAX``（commands_logic.cpp 4.5.7:972-973），
    區間外的 DO_CHANGE_SPEED 被**靜默丟掉**、維持前一個空速 ⇒ 4D 計畫的時序跟著偏。"""
    lo, hi = p.get("AIRSPEED_MIN"), p.get("AIRSPEED_MAX")
    if lo is None or hi is None:
        return
    lo, hi = float(lo), float(hi)
    va = p.get("AIRSPEED_CRUISE")
    if va is not None and not (lo <= float(va) <= hi):
        msg = (f"AIRSPEED_CRUISE {float(va):g} m/s is outside the stored AIRSPEED_MIN {lo:g} / AIRSPEED_MAX {hi:g}")
        warn.append(msg)
        _log.warning("巡航空速 %g m/s 不在取整後的 AIRSPEED_MIN %g／AIRSPEED_MAX %g 之內", float(va), lo, hi)
    if uav_plan is None:
        return
    cmds = sorted({round(float(w.airspeed_mps), 3) for w in uav_plan.waypoints
                   if w.mode is Mode.CRUISE and float(w.airspeed_mps) > 0.0})
    for v in cmds:
        if not (lo <= v <= hi):
            msg = (f"{uav_plan.uav_id}: DO_CHANGE_SPEED {v:g} m/s is outside the stored AIRSPEED_MIN {lo:g} / "
                   f"AIRSPEED_MAX {hi:g} (integer effective values): ArduPlane do_change_speed only accepts "
                   "min <= v <= max (commands_logic.cpp 4.5.7:972-973) and silently keeps the previous airspeed")
            warn.append(msg)
            _log.warning("%s：任務空速指令 %g m/s 落在飛控會存的 AIRSPEED_MIN %g／AIRSPEED_MAX %g 之外 ⇒ 飛控會靜默"
                         "不套用這筆 DO_CHANGE_SPEED", uav_plan.uav_id, v, lo, hi)


def recommended_params(spec: PlatformSpec, model: EnergyModel | None, cfg: PlannerConfig,
                       uav_plan: UAVPlan | None = None, *, pack_voltage_v: float | None = None,
                       wind_speed_mps: float = 0.0, h_ceil_agl_m: float | None = None,
                       lost_link_policy: str = "RTB", alt_frame: AltFrame = "relative",
                       t_ll_s: float = 60.0, home_msl_m: float | None = None,
                       min_nav_spacing_m: float | None = None, loiter_radius_m: float | None = None,
                       sensor_kinds: Iterable[str] = (), warnings: list[str] | None = None,
                       camera_cfg: CameraTriggerConfig | None = None) -> dict[str, float]:
    """表 1 → ArduPilot 參數建議（ardupilot_notes §3）。回傳 {參數名: 值}；每鍵之說明見 :data:`PARAM_COMMENTS`。

    BATT_CAPACITY／BATT_LOW_MAH／BATT_CRT_MAH 只在 `pack_voltage_v` 已知（SortieEnergy.pack_voltage_nom_v）且模型非
    基準分鐘模式時輸出；無法輸出門檻時 BATT_FS_*_ACT 仍輸出（動作正確才不會誤觸發終止），但於 `warnings` 記一筆，
    由操作者於機上設定門檻。

    * `home_msl_m` + `uav_plan`：FENCE_ALT_MAX 依 **相對 home 高度**（AC_Fence 之判準）計算，而非 AGL（D48）。
    * `min_nav_spacing_m`（:func:`min_nav_spacing_m`）：WP_RADIUS 取 0.8×該值（夾在 [10, r_acc]）。
    * `loiter_radius_m`（:func:`orbit_radius_m`）：WP_LOITER_RAD 取本任務實際盤旋半徑。
    * `sensor_kinds`：本構型之感測種類（'rgb'／'lidar'）→ 輸出使 DO_SET_CAM_TRIGG_DIST／DO_SET_SERVO 生效之酬載參數。
    * `camera_cfg`（:class:`CameraTriggerConfig`）：有 'rgb' 時依其後端輸出 CAM1_TYPE／DURATION／INTRVAL_MIN、
      CAM_AUTO_ONLY／MAX_ROLL、FEEDBAK_PIN／POL、CAM1_TRIGG_DIST=0 與後端參數（servo：SERVOn_FUNCTION=10；relay：
      依韌體版本之 RELAY 參數＋SERVOn_FUNCTION=-1）；None ＝ servo 後端 CAM1_TYPE=1、CAM1_DURATION=0.1、SERVO10=10。
      本構型沒有 'rgb' 時不輸出相機參數並於 `warnings` 記一筆。同一個 SERVOn 被要求兩種 FUNCTION → ValueError。
    * **整數型參數**（AP_Int8/16/32、AP_Enum；見 :func:`param_storage_type`）：飛控收到小數會靜默截斷（HERO 的
      AIRSPEED_MAX 1.4×19 = 26.6 會存成 26），所以這裡就輸出飛控會存的值並於 `warnings` 記錄原值 → 有效值：上限
      （AIRSPEED_MAX、WP_RADIUS）往下取、下限／半徑／高度下限（AIRSPEED_MIN、WP_LOITER_RAD、RTL_RADIUS、Q_RTL_ALT）往上取
      ——取整後的範圍一定落在平台規格之內、轉彎半徑不小於 R_eff。取整後巡航空速或 `uav_plan` 的空速指令落在
      [AIRSPEED_MIN, AIRSPEED_MAX] 之外（飛控會靜默丟掉該 DO_CHANGE_SPEED）⇒ `warnings` 記一筆。
    """
    warn = warnings if warnings is not None else []
    if camera_cfg is not None and not isinstance(camera_cfg, CameraTriggerConfig):
        raise TypeError(f"camera_cfg 須為 CameraTriggerConfig，得 {type(camera_cfg).__name__}")
    va = float(spec.cruise_airspeed_mps)
    r_eff = spec.r_eff(wind_speed_mps)
    v_max = float(spec.max_airspeed_mps) if spec.max_airspeed_mps is not None else round(1.4 * va, 1)
    tc = spec.transition_cmd(transition_profile=cfg.transition_profile)
    v_min = float(spec.v_min_mission())
    # Q_ASSIST_SPEED 必須 < AIRSPEED_MIN（否則巡航中 TECS 一降到下限就啟動升力馬達）且 > 失速
    q_assist = min(float(tc.v_tr_mps), v_min - 1.0)
    q_assist = max(q_assist, 1.15 * float(spec.stall_airspeed_mps))
    if q_assist >= v_min:                                   # 極窄之速度窗（理論上不會發生）
        q_assist = 0.5 * (float(spec.stall_airspeed_mps) + v_min)
        warn.append("Q_ASSIST_SPEED squeezed between AIRSPEED_STALL and AIRSPEED_MIN; verify on the vehicle")
    r_acc = accept_radius_m(r_eff)
    if min_nav_spacing_m is not None and math.isfinite(min_nav_spacing_m) and min_nav_spacing_m > 0:
        wp_rad = max(10.0, min(r_acc, math.floor(0.8 * float(min_nav_spacing_m))))
        if wp_rad >= float(min_nav_spacing_m):
            warn.append(f"WP_RADIUS {wp_rad:.0f} m >= min NAV spacing {float(min_nav_spacing_m):.1f} m; "
                        "consecutive waypoints may be consumed in one cycle")
    else:
        wp_rad = r_acc
    p: dict[str, float] = {
        "AIRSPEED_CRUISE": va,
        "AIRSPEED_MIN": v_min,
        "AIRSPEED_MAX": v_max,
        "AIRSPEED_STALL": float(spec.stall_airspeed_mps),
        "TECS_CLMB_MAX": float(spec.max_fw_climb_rate_mps),
        "TECS_SINK_MAX": float(spec.max_fw_descent_rate_mps),
        "TECS_SINK_MIN": round(0.9 * va * math.sin(math.radians(spec.planning_descent_angle_deg)), 2),
        "ROLL_LIMIT_DEG": float(spec.max_bank_angle_deg),
        "WP_LOITER_RAD": float(round(loiter_radius_m if loiter_radius_m else r_eff, 1)),
        "WP_RADIUS": float(round(wp_rad, 1)),
        "RTL_RADIUS": float(round(r_eff, 1)),
        "Q_ENABLE": 1.0,
        "Q_TRANSITION_MS": float(int(round(spec.t_tr_out_s * 1000.0))),
        "Q_ASSIST_SPEED": float(round(q_assist, 1)),
        "Q_RTL_ALT": float(spec.min_fw_operation_agl_m),
        "Q_RTL_MODE": 1.0,
        "Q_LAND_FINAL_ALT": 10.0,
        "RTL_AUTOLAND": 2.0,
        "FENCE_ENABLE": 1.0,
        "FENCE_TYPE": 5.0,
        "FENCE_ACTION": 1.0,
        "FENCE_RET_RALLY": 1.0,
        "FS_LONG_ACTN": 0.0 if lost_link_policy == "CONTINUE_THEN_RTB" else 1.0,
        "FS_LONG_TIMEOUT": float(t_ll_s),
        "TERRAIN_ENABLE": 1.0 if alt_frame == "terrain" else 0.0,
        "TERRAIN_FOLLOW": 1.0 if alt_frame == "terrain" else 0.0,
        # ArduPlane 之列舉（非 Copter）：1 = RTL、4 = QLand（3 是 Terminate）
        "BATT_FS_LOW_ACT": 1.0,
        "BATT_FS_CRT_ACT": 4.0,
    }
    # ---- 酬載參數（沒有這些，任務中的 DO_SET_CAM_TRIGG_DIST／DO_SET_SERVO 不會有任何作用）----
    kinds = {str(k) for k in sensor_kinds}
    if "rgb" in kinds:
        if camera_cfg is None:
            p["CAM1_TYPE"] = 1.0                           # servo backend（CAM1_* 自 4.4.0；4.4 以前為 CAM_TRIGG_TYPE）
            # 單位是秒（4.4.0 起）。舊值 1.0 ＝ 1 s 脈衝，期間拒絕新觸發 ⇒ 觸發率被壓在 ~1 Hz；0.1 為 ArduPilot 預設
            p["CAM1_DURATION"] = 0.1
            p[f"SERVO{CAMERA_SERVO_CHANNEL}_FUNCTION"] = 10.0  # CameraTrigger
        else:
            p.update(_camera_params(camera_cfg, warn))
    elif camera_cfg is not None:
        warn.append(f"camera_cfg (backend {camera_cfg.backend}) NOT applied: sensor kinds {sorted(kinds)} contain no "
                    "'rgb' camera, so no CAM*/RELAY* parameters were exported")
    if "lidar" in kinds or not kinds:
        key = f"SERVO{LIDAR_SERVO_CHANNEL}_FUNCTION"
        if key in p and p[key] != 0.0:
            raise ValueError(f"{key}: 相機設定要求 {p[key]:g}，LiDAR 佔位要求 0（Disabled）——同一輸出不能兩用，"
                             "請改 camera_cfg 之通道或 LIDAR_SERVO_CHANNEL")
        p[key] = 0.0                                       # Disabled ⇒ 開放給任務 DO_SET_SERVO
    # ---- FENCE_ALT_MAX：AC_Fence 比對「相對 home 高度」，不是 AGL ----
    h_ceil = float(h_ceil_agl_m) if h_ceil_agl_m is not None and math.isfinite(h_ceil_agl_m) else None
    rel_max: float | None = None
    if uav_plan is not None and uav_plan.waypoints:
        if home_msl_m is not None and math.isfinite(home_msl_m):
            rel_max = max(float(w.alt_msl_m) for w in uav_plan.waypoints) - float(home_msl_m) + cfg.b_v_m
        else:                                              # 無 home MSL：退回 AGL（平地才等價）並警告
            rel_max = max(float(w.agl_m) for w in uav_plan.waypoints) + cfg.b_v_m
            warn.append("FENCE_ALT_MAX derived from AGL because home_msl_m was not supplied; over terrain higher "
                        "than the launch site the fence can trip mid-mission")
    if rel_max is not None:
        h_ceil = rel_max if h_ceil is None else max(h_ceil, rel_max)
    if h_ceil is None:
        h_ceil = float(spec.min_fw_operation_agl_m) + 100.0
    p["FENCE_ALT_MAX"] = float(math.ceil(h_ceil))
    if model is not None and pack_voltage_v is not None and pack_voltage_v > 0 and not model.base_minute_mode \
            and model.nominal_wh is not None:
        wh2mah = 1000.0 / float(pack_voltage_v)
        p["BATT_CAPACITY"] = float(round(float(model.nominal_wh) * wh2mah))
        p["BATT_CRT_MAH"] = float(round(model.reserve_wh() * wh2mah))
        p["BATT_LOW_MAH"] = float(round((model.reserve_wh() + model.e_landing_wh()) * wh2mah))
    else:
        warn.append("BATT_LOW_MAH/BATT_CRT_MAH not exported (pack voltage or energy model unknown): "
                    "BATT_FS_LOW_ACT/BATT_FS_CRT_ACT have no threshold until you set BATT_LOW_MAH/BATT_CRT_MAH "
                    "(or BATT_LOW_VOLT/BATT_CRT_VOLT) on the vehicle")
    # ---- 有效值：AP_Int* 參數取整成飛控會存的值（記 WARNING），再檢查空速指令是否仍在取整後的範圍內 ----
    p = _round_int_params(p, warn)
    _check_speed_window(p, uav_plan, warn)
    return p


def write_param_file(params: Mapping[str, float], path: str | os.PathLike, comments: Mapping[str, str] | None = None,
                     warnings: Sequence[str] = ()) -> str:
    """Mission Planner `.param` 格式：每列 `NAME,VALUE`；`#` 開頭為註解（含每鍵說明與 `warnings`）。"""
    cm = PARAM_COMMENTS if comments is None else comments
    lines = ["# HV-DCCPP recommended ArduPilot (ArduPlane/QuadPlane) parameters",
             "# Verify against the vehicle firmware version before upload (names are ArduPlane 4.5+)."]
    for w in warnings:
        lines.append(f"# WARNING: {w}")
    for k in sorted(params):
        v = params[k]
        if k in cm:
            lines.append(f"# {k}: {cm[k]}")
        vs = str(int(v)) if float(v).is_integer() else f"{float(v):.6g}"
        lines.append(f"{k},{vs}")
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    return str(path)


def read_param_file(path: str | os.PathLike) -> dict[str, float]:
    """讀回 `.param`（`NAME,VALUE` 或 `NAME VALUE`；忽略 `#` 註解）。"""
    out: dict[str, float] = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            parts = s.replace(",", " ").replace("\t", " ").split()
            if len(parts) < 2:
                raise ValueError(f"bad param line: {line!r}")
            out[parts[0]] = float(parts[1])
    return out


# ---------------------------------------------------------------------------
# QGC .plan JSON
# ---------------------------------------------------------------------------
def _fence_groups(fence: Sequence[MissionItem]) -> list[dict[str, Any]]:
    polys: list[dict[str, Any]] = []
    cur: list[list[float]] = []
    cur_cmd: int | None = None
    for it in fence:
        if it.command in (MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION):
            if cur_cmd is not None and (it.command != cur_cmd or len(cur) >= int(it.param1)):
                polys.append({"inclusion": cur_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, "polygon": cur,
                              "version": 1})
                cur = []
            cur_cmd = it.command
            cur.append([it.x, it.y])
        else:
            if cur:
                polys.append({"inclusion": cur_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, "polygon": cur,
                              "version": 1})
                cur, cur_cmd = [], None
    if cur:
        polys.append({"inclusion": cur_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, "polygon": cur, "version": 1})
    return polys


def to_qgc_plan_json(items: Sequence[MissionItem], rally: Sequence[MissionItem] | None,
                     fence: Sequence[MissionItem] | None, path: str | os.PathLike | None = None,
                     cruise_speed_mps: float | None = None) -> dict[str, Any]:
    """QGC `.plan` v1 JSON（vehicleType 2 固定翼、firmwareType 3 ArduPilot）；`path` 給定時亦寫檔。"""
    if not items:
        raise ValueError("items must contain the home row")
    home = items[0]
    mission_items: list[dict[str, Any]] = []
    for it in items[1:]:
        # QGC (>= 4.3) AltitudeMode：0 None、1 Relative、2 Absolute、3 CalcAboveTerrain（由地形資料算 AMSL）、
        # 4 TerrainFrame（= MAV_FRAME_GLOBAL_TERRAIN_ALT）。
        alt_mode = {MAV_FRAME_GLOBAL_RELATIVE_ALT: 1, MAV_FRAME_GLOBAL: 2, MAV_FRAME_GLOBAL_TERRAIN_ALT: 4}.get(it.frame, 1)
        mission_items.append({
            "type": "SimpleItem", "autoContinue": bool(it.autocontinue), "command": int(it.command),
            "doJumpId": int(it.seq), "frame": int(it.frame),
            "params": [it.param1, it.param2, it.param3, it.param4, it.x, it.y, it.z],
            "AMSLAltAboveTerrain": None, "Altitude": it.z, "AltitudeMode": alt_mode,
        })
    if cruise_speed_mps is None:
        sp = [it.param2 for it in items if it.command == MAV_CMD_DO_CHANGE_SPEED and it.param2 > 0]
        cruise_speed_mps = float(sp[0]) if sp else 20.0
    plan = {
        "fileType": "Plan", "version": 1, "groundStation": "QGroundControl",
        "mission": {"cruiseSpeed": float(cruise_speed_mps), "hoverSpeed": 5.0, "firmwareType": _QGC_FIRMWARE_ARDUPILOT,
                    "vehicleType": _QGC_VEHICLE_FIXED_WING, "version": 2,
                    "plannedHomePosition": [home.x, home.y, home.z], "items": mission_items},
        "geoFence": {"version": 2, "circles": [], "polygons": _fence_groups(fence or [])},
        "rallyPoints": {"version": 2, "points": [[r.x, r.y, r.z] for r in (rally or [])
                                                 if r.command == MAV_CMD_NAV_RALLY_POINT]},
    }
    if path is not None:
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            json.dump(plan, f, indent=2)
    return plan


# ---------------------------------------------------------------------------
# 一鍵匯出
# ---------------------------------------------------------------------------
def export_all(plan: MissionPlan, ctx: MissionContext, cfg: PlannerConfig, out_dir: str | os.PathLike, *,
               alt_frame: AltFrame = "relative", camera_trigger: bool = True, wp_radius_m: float | None = None,
               strip_sensors: Mapping[str, Sensor] | None = None, write_plan_json: bool = True,
               takeoff_delay: Literal["operator", "nav_delay"] = "operator", t0_s: float | None = None,
               camera_cfg: CameraTriggerConfig | None = None) -> dict[str, Any]:
    """每機寫出 `<uav_id>.waypoints`、`<uav_id>_rally.txt`、`<uav_id>_fence.txt`、`<uav_id>.param`
    （及 `<uav_id>.plan` QGC JSON），加 `mission_summary.json`；回傳 summary dict。

    **起飛時刻**（D46）：計畫之 `t_takeoff_s` 是 4D 去衝突與走廊排程的結果，任務檔本身不帶時間原點，若同時 AUTO
    所有機，同墊 60 s 間隔與全部去衝突延遲都會失效。`takeoff_delay`：

    * ``'operator'``（預設，與 `mavlink_link.FleetLink` 之「由地面站依 t_takeoff 逐機 MISSION_START」一致）：
      不插入任何項目，但把 `t0 + Δt` 寫進 `<uav>.waypoints` 的檔頭註解與 `mission_summary.json` 之 `takeoff` 區塊；
    * ``'nav_delay'``：於 NAV_VTOL_TAKEOFF 之前插入 MAV_CMD_NAV_DELAY(93)（param1 = t_takeoff − t0 秒），
      任務自帶錯開；此時**不可**再由地面站延後 MISSION_START（會重複延遲）。

    `t0_s` 預設為全隊最小之 `t_takeoff_s`。摘要的 ``takeoff.offset_s``／``uavs[uid].takeoff_offset_s`` 是未四捨五入的
    t_takeoff − t0（秒；與 ``aeroplan_adapter.HvDccppCoordinator.takeoff_schedule`` 同值），``delay_s`` 是 0.1 s 取整的
    顯示值。

    **高度基準**：每架的相對高度、第 0 列 home、`<uav>_rally.txt` 的高度與 `.param` 的 FENCE_ALT_MAX 都以**該機起飛坪**
    為 home（飛控解鎖時 home 重設到這裡；見 :func:`fc_home`），摘要 ``uavs[uid].fc_home`` 記坪、MSL 與它和規劃 home
    的差（``home`` 區塊仍是規劃 home）。

    `camera_cfg`（:class:`CameraTriggerConfig`）同時傳給 :func:`plan_to_mission_items`（206 擺放）與
    :func:`recommended_params`（`.param` 之相機參數），有效值另寫進 summary 之 ``camera_cfg``。
    """
    out_dir = os.fspath(out_dir)
    os.makedirs(out_dir, exist_ok=True)
    if takeoff_delay not in ("operator", "nav_delay"):
        raise ValueError(f"takeoff_delay must be 'operator' or 'nav_delay', got {takeoff_delay!r}")
    inputs = ctx.inputs
    home_msl = _home_msl(ctx)
    fences: dict[str, list[MissionItem]] = {}
    t0 = float(t0_s) if t0_s is not None else (min(float(u.t_takeoff_s) for u in plan.plans.values())
                                               if plan.plans else 0.0)
    delays = {uid: max(0.0, float(up.t_takeoff_s) - t0) for uid, up in plan.plans.items()}
    summary: dict[str, Any] = {
        "home": {"site_id": inputs.home.site_id, "lat_deg": inputs.home.lat_deg, "lon_deg": inputs.home.lon_deg,
                 "alt_msl_m": home_msl},
        "alt_frame": alt_frame, "makespan_s": float(plan.makespan_s), "feasibility": plan.feasibility.status,
        "uncovered": [list(u) for u in plan.uncovered],
        "takeoff": {"mode": takeoff_delay, "t0_s": t0, "spread_s": (max(delays.values()) if delays else 0.0),
                    "delay_s": {u: round(d, 1) for u, d in delays.items()},
                    "offset_s": dict(delays),
                    "note": ("每機任務第 1 列為 NAV_DELAY(93)，Δt 已內含於任務；地面站請一次 MISSION_START，勿再延後。"
                             if takeoff_delay == "nav_delay" else
                             "任務不含時間原點：地面站須於 t0 + delay_s[uav] 才對該機 MISSION_START"
                             "（規格 13.5 同墊 60 s 間隔與 4D 去衝突延遲全靠此）。")},
        "uavs": {},
    }
    if camera_cfg is not None:
        summary["camera_cfg"] = asdict(camera_cfg)
    p_comments = {**PARAM_COMMENTS, **camera_param_comments(camera_cfg)}
    for uid, up in plan.plans.items():
        cls = up.cls
        spec = ctx.specs[cls]
        uav = ctx.uav(uid)
        model = ctx.models.get(uid) or ctx.models_rep.get(cls)
        items = plan_to_mission_items(up, ctx, cfg, alt_frame, camera_trigger, wp_radius_m, strip_sensors=strip_sensors,
                                      takeoff_delay_s=delays[uid] if takeoff_delay == "nav_delay" else None,
                                      camera_cfg=camera_cfg)
        if cls not in fences:
            fences[cls] = fence_items(inputs, ctx, cls, [q for q in plan.plans.values() if q.cls == cls])
        fence = fences[cls]
        # 飛控 home ＝ 本機起飛坪：rally 高度與 FENCE_ALT_MAX 都「相對 home」⇒ 同一個基準
        ref_site, ref_lat, ref_lon, ref_msl = fc_home(ctx, up)
        rally = rally_items(inputs, ctx, alt_frame, ref_msl_m=ref_msl)
        w_ref = ctx.wind.max_speed() if ctx.wind is not None else 0.0
        kinds = {getattr(s, "kind", None) for (c, _r), s in ctx.sensors.items() if c == cls}
        if strip_sensors:
            kinds |= {getattr(strip_sensors[s], "kind", None) for s, _d in up.strips if s in strip_sensors}
        p_warn: list[str] = []
        params = recommended_params(spec, model, cfg, up, pack_voltage_v=uav.sortie.pack_voltage_nom_v,
                                    wind_speed_mps=w_ref, h_ceil_agl_m=ctx.h_ceil_m.get(cls),
                                    lost_link_policy=inputs.c2.policy_for(uid, cls), alt_frame=alt_frame,
                                    t_ll_s=inputs.c2.t_ll_s, home_msl_m=ref_msl,
                                    min_nav_spacing_m=min_nav_spacing_m(items, ctx),
                                    loiter_radius_m=orbit_radius_m(items),
                                    sensor_kinds=[k for k in kinds if k], warnings=p_warn, camera_cfg=camera_cfg)
        files = {
            "waypoints": os.path.join(out_dir, f"{uid}.waypoints"),
            "rally": os.path.join(out_dir, f"{uid}_rally.txt"),
            "fence": os.path.join(out_dir, f"{uid}_fence.txt"),
            "param": os.path.join(out_dir, f"{uid}.param"),
        }
        wp_comments = [f"HV-DCCPP {uid} ({cls}); plan t_takeoff = {float(up.t_takeoff_s):.1f} s, t_land = "
                       f"{float(up.t_land_s):.1f} s (mission time origin t0 = {t0:.1f} s)"]
        if takeoff_delay == "nav_delay":
            wp_comments.append(f"seq 1 = NAV_DELAY {delays[uid]:.1f} s: start the mission once, do NOT delay "
                               f"MISSION_START as well")
        else:
            wp_comments.append(f"START THIS MISSION AT t0 + {delays[uid]:.1f} s. The mission carries no time origin; "
                               f"starting all vehicles together voids the 60 s pad spacing and every 4D "
                               f"deconfliction delay.")
        wp_comments.append(f"home (seq 0) = takeoff pad {ref_site} at {ref_msl:.2f} m MSL; relative altitudes are "
                           f"above this pad (ArduPlane resets home to the arming position); planning home "
                           f"{inputs.home.site_id} is {home_msl:.2f} m MSL ({ref_msl - home_msl:+.2f} m)")
        write_qgc_wpl(items, files["waypoints"], wp_comments)
        write_qgc_wpl(rally, files["rally"])
        write_qgc_wpl(fence, files["fence"])
        write_param_file(params, files["param"], comments=p_comments, warnings=p_warn)
        if write_plan_json:
            files["plan"] = os.path.join(out_dir, f"{uid}.plan")
            to_qgc_plan_json(items, rally, fence, files["plan"], cruise_speed_mps=spec.cruise_airspeed_mps)
        summary["uavs"][uid] = {
            "cls": cls, "n_items": len(items), "n_nav": sum(1 for it in items if it.is_nav),
            "n_waypoints_in": len(up.waypoints), "strips": [[s, int(d)] for s, d in up.strips],
            "t_takeoff_s": float(up.t_takeoff_s), "t_arr_ptc_s": float(up.t_arr_ptc_s), "t_land_s": float(up.t_land_s),
            "takeoff_site": up.takeoff_site, "landing_site": up.landing_site,
            "energy_used_wh": float(up.energy_used_wh),
            "energy_margin_wh": float(up.energy_margin_wh), "distance_m": float(up.distance_m),
            "orbits_inserted": int(up.orbits_inserted), "warnings": list(up.warnings),
            "takeoff_delay_s": round(delays[uid], 1), "takeoff_offset_s": delays[uid], "param_warnings": p_warn,
            "fc_home": {"site_id": ref_site, "lat_deg": ref_lat, "lon_deg": ref_lon, "alt_msl_m": ref_msl,
                        "offset_from_planning_home_m": ref_msl - home_msl},
            "params": params, "files": files,
        }
    with open(os.path.join(out_dir, "mission_summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    summary["files"] = {"summary": os.path.join(out_dir, "mission_summary.json")}
    return summary


__all__ = [
    "MissionItem", "plan_to_mission_items", "reduce_waypoints", "arc_vertices", "accept_radius_m", "trigger_distance_m",
    "write_qgc_wpl", "read_qgc_wpl", "rally_items", "fence_items", "fence_polygons", "fence_buffer_m", "recommended_params",
    "write_param_file", "read_param_file", "to_qgc_plan_json", "export_all", "renumber", "PARAM_COMMENTS",
    "NAV_COMMANDS", "FRAME_OF", "LIDAR_SERVO_CHANNEL", "LIDAR_SERVO_PWM_ON", "LIDAR_SERVO_PWM_OFF",
    "CAMERA_SERVO_CHANNEL", "LOITER_TURNS_MAX_RADIUS_M", "orbit_radius_m", "min_nav_spacing_m", "HAVE_PYMAVLINK",
    "CameraTriggerConfig", "CAMERA_BACKEND_TYPE", "RELAY_FUNCTION_CAMERA", "RELAY_NUM_RELAYS", "camera_param_comments",
    "parse_firmware_version", "fc_home", "LEAD_IN_ACCEPT_RADIUS_M", "AGL_SNAP_TOL_M", "FLOAT_PARAMS",
    "param_storage_type",
]
