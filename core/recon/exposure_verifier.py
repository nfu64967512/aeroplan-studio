"""曝光層級（攝影測量）覆蓋驗證 —— 每張照片的足跡、每格被幾張照片看到、前向／側向／接縫重疊。

為什麼要有這個
--------------
``core/recon/coverage_verifier.py`` 量的是 2-D「航跡 ± 刈幅/2」：全隊共用一個刈幅、假設連續成像。
它量不到前向重疊、照片張數、側風偏流（crab）造成的足跡旋轉，也看不到 ``CAM1_INTRVAL_MIN``
**靜默延後**把間距拉大。SfM 要的是「每個地面點被幾張照片看到」（視數），本模組直接算這個。

兩種用法，共用同一套度量（:func:`exposure_metrics`）
-----------------------------------------------
* **飛前**（:func:`verify_preflight`）：吃每架的任務項目（``hv_dccpp.ardupilot.plan_to_mission_items``
  的輸出形狀），模擬 ArduPilot AP_Camera 的距離觸發，得到每張照片的位置與航向（航跡 + 偏流）。
* **飛後**（:func:`verify_postflight`）：吃 photos.csv（F2）的實際位姿；用實際 yaw，
  可選 roll/pitch 做傾斜足跡（針孔投影四角到該張天底點地形高的水平面）。

ArduPilot 語意（Plane-4.5.7 原始碼核對；master 的觸發邏輯相同，只差 4.5 多一個 GPS 3D fix 門檻）
--------------------------------------------------------------------------------------
* ``AP_Camera_Backend::update`` 50 Hz：先處理 ``trigger_pending``；``TRIGG_DIST <= 0`` 時清掉
  ``last_location``；``last_location`` 為空就先記下目前位置（**第一張在 d 公尺之後**）；
  與上一張的**水平直線距離** ≥ d 才拍（Backend.cpp:53-93）。
* ``take_picture``：距上一張不足 ``INTRVAL_MIN`` → ``trigger_pending``，**不告警**，之後每個 tick 重試
  （Backend.cpp:126-152）⇒ 間距變成 v·Tmin（量化到 20 ms）。
* Relay／Servo 後端 ``trigger_pic``：脈衝未結束（``trigger_counter = DURATION·50`` 個 tick）時回傳 false
  （Relay.cpp:24-42）；距離觸發會在下個 tick 重試，但 206 p3=1 的那一張會**直接丟掉**。
* 206：``set_trigger_distance(p1)``，``p3 == 1`` 立刻 ``take_picture()``（Mission_Commands.cpp:138-143）。
  203（DO_DIGICAM_CONTROL）只有 param5 == 1 會拍。
* DO 指令在「抵達前一個 NAV」時執行；ArduPlane 的 NAV_WAYPOINT **會用 param2 當接受半徑**
  （AP_Mission.cpp:1018-1027 存進 p1 低位元組、commands_logic.cpp:681-689 使用，上限 255 m），
  所以帶尾的 206(0) 會在離帶尾 param2 公尺處就執行。同一 NAV 之後的多個 DO 一次只跑一個：
  第 1 個與抵達同一次 mission.update，第 k≥2 個晚 (2k−3) 次（AP_Mission.cpp:306-356、2060-2088）。
  mission.update 的頻率取 10 Hz（ArduPlane navigate 排程）。⚠ 呼叫鏈在 mode_auto.cpp，手上的原始碼
  副本沒有這個檔 ⇒ 10 Hz 是**未核對**的假設；誤差 ≤ 0.1–0.3 s（20 m/s 下 2–6 m）。
* DO_LAND_START：失效保護（RTL_AUTOLAND）跳到它時，它與下一個 NAV 之間的 DO 會依序執行
  （master AP_Mission.cpp:2573 jump_to_landing_sequence → 608 set_current_cmd → 2157 advance_current_nav_cmd
  先載入 DO_LAND_START 本身；之後 2258 advance_current_do_cmd／2380 get_next_do_cmd 從下一列逐個執行到 NAV 為止）
  ⇒ 緊接其後的 206(0) 會關相機。靜態檢查以「這串 DO
  跑完相機是否仍開著」判定，只有真的開著才寫 note（effective ``land_start_in_open_206`` 記每個的判定）。
* DO_CHANGE_SPEED：param1=0 空速；超出 [AIRSPEED_MIN, AIRSPEED_MAX] 時飛控**靜默不套用**
  （commands_logic.cpp:965-991）—— 給了 ``airspeed_limits_mps`` 才模擬這件事，否則照單全收並寫進 notes。
  param1=1 只改 MIN_GROUNDSPEED，不改空速 ⇒ 本模擬不改空速並寫進 notes。

未建模（報告 ``notes`` 會逐條列出，不靜默）
-----------------------------------------
* **CAM_MAX_ROLL**：要動態模型才知道轉彎改出時的滾轉；本模擬不擋 ⇒ 實際張數可能少
  （SITL 約 4% 觸發落在 |roll|>10°，集中在每帶前 5 張）。
* 轉彎切角：路徑一律是相鄰 NAV 之間的 3-D 直線；只有 DO 的「執行點」依接受半徑提前。
* NAV_WAYPOINT param2 = 0 時 ArduPlane 用 WP_RADIUS 依轉角提前判定抵達（turn_distance），本模擬以 0 計。
* 盤旋（LOITER_*）期間的照片；起降垂直段的時間（無水平位移，不影響距離觸發）。
* 機身俯仰：飛前一律 pitch = roll = 0（平飛配平攻角約數度，未計）。
* CAM1_TRIGG_DIST 參數初值假設為 0、全程 AUTO（CAM_AUTO_ONLY 不擋）、GPS 3D fix 正常。

座標
----
本地切平面：以測區中心 (lat0, lon0) 為原點，x = N(φ0)·cosφ0·Δλ、y = M(φ0)·Δφ
（WGS84 卯酉圈／子午圈曲率半徑）。對 pyproj aeqd 實測（25.10N 121.20E）：±1 km（2 km 見方）內
位置誤差 ≤ 0.08 m、兩點距離誤差 ≤ 0.12 m；±2 km 內 ≤ 0.32 m／0.54 m。
（若用球半徑 6378137 m，北向尺度會差 0.49%，2 km 就是 10 m。）

高度基準
--------
相機高度與地形**必須同一個垂直基準**；本模組不做大地起伏換算。飛前：任務高度（frame 0 = AMSL、
3 = 相對 home、10 = 相對地形）一律換成 MSL，地形函式也要回 MSL。飛後：``height_field`` 選
``h_orth_m``（地形給正高）或 ``h_ell_m``（地形給橢球高）；選的欄位有空值就丟例外，**不會拿 N=0 補**。
已知 AGL 時可傳 ``terrain=0.0`` 並把 AGL 當高度。

度量定義
--------
* 視數：足跡（凸四邊形）光柵化到格網，格心落在足跡內就算一次；只統計格心在測區內的格。
* 前向重疊：同一 (uav, strip) 依 seq 相鄰兩張，交集面積 / 兩者中較大的足跡面積（保守）。
  足跡同尺寸、未旋轉時等於 1 − d/l；有偏流時足跡斜著排，會比 1 − d/l 小。
* 側向重疊：同一架、平行且相鄰的兩條帶；A 帶每張照片與 B 帶足跡聯集的交集面積 / 該張面積
  （只取沿帶位置落在 B 帶範圍內的照片）。等於 1 − s/w，且與兩帶照片的前後相位無關。
* 接縫重疊：只算**相鄰**的兩架。重疊帶 = 格心在測區內、**恰好只被 X、Y 兩架**看到的格（第三架也看到的格
  表示第三架夾在中間，X、Y 不必直接相接，那裡的偶然重疊不算接縫；兩架的重疊全都有第三架 ⇒ 不相鄰、不列，
  記在 ``effective["seam_pairs_not_adjacent"]``）。局部寬度 = 到「X 有、Y 沒有」區的距離 + 到「Y 有、X 沒有」
  區的距離 − 1 格（兩架之間的重疊寬，與第三架無關），除以兩架照片足跡寬的中位數（ref_swath_m）。只取離
  「兩架都沒看到」的格 ≥ 一個參考刈幅的帶內格（避開帶的兩端與破洞旁的端點效應）。光柵量化約 ±1 格。
  沒有只屬於兩架的重疊，但兩架之間隔著「沒有任何一架看到」的格（該格最近的兩架就是 X、Y）且縫 < 參考
  刈幅，或兩架的「只有自己」區直接相接 ⇒ 相鄰而重疊為 0，記為寬度 0 並給 gap_m。
* 視數不足區（給補飛規劃）：格心在測區內且視數 < k 的格，每個 4-連通分量一個多邊形
  （rasterio／GDAL polygonize），再裁到測區內；附經緯度外環／內環、面積、格數、最小視數，依面積排序。
* 邊緣帶：離測區邊界（含內環）≤ ``edge_band_m``（預設 w/2，w = 照片橫向足跡寬中位數）的格。最外側條帶
  只有一條蓋到邊緣、觸發間距又被 50 Hz tick 量化（d' > d），邊緣帶的視數會規律地少一張 ⇒ 邊緣帶與內部
  分開統計（``edge_band``／``interior``：格數、frac_ge、視數不足格比例、最少視數）；視數不足分量整個落在
  邊緣帶內者標 ``region="edge_band"``，只要伸入內部就標 ``"interior"``（當成真破洞）。整體的
  ``frac_ge``／``undercovered``／``undercovered_area_m2`` 照舊包含全部，不因此變好看。

偏流對重疊數字的影響（讀報告時要知道）
------------------------------------
有偏流時足跡斜著排：前向重疊（面積）會下降，例如 HQ6、d = 0.2·l、偏流 15.26° → 0.773；
側向重疊（對鄰帶聯集的面積）反而略升，因為斜放的照片角落伸得更遠。評估報告裡「側向 0.70 實剩
0.62–0.65」是**並排兩張照片**的面積重疊（保守口徑、含沿帶相位），與這裡的定義不同。
驗收應看每格視數（``frac_ge``／``undercovered``），重疊百分比只是輔助。
"""

from __future__ import annotations

import csv
import logging
import math
import os
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Mapping, Sequence

import numpy as np
import shapely
import shapely.geometry
from scipy import ndimage

__all__ = [
    "CAMERA_UPDATE_HZ",
    "MISSION_UPDATE_HZ",
    "F2_COLUMNS",
    "LocalFrame",
    "FrameCamera",
    "WindSolution",
    "wind_triangle",
    "PhotoPose",
    "SimulationResult",
    "simulate_mission_photos",
    "footprint_corners_enu",
    "OverlapStats",
    "SeamStats",
    "UnderCoveredArea",
    "RegionStats",
    "ExposureGrid",
    "ExposureReport",
    "exposure_metrics",
    "verify_preflight",
    "read_photos_csv",
    "verify_postflight",
]

_log = logging.getLogger(__name__)

CAMERA_UPDATE_HZ = 50
"""AP_Camera::update 排程頻率（ArduPlane.cpp SCHED_TASK_CLASS(AP_Camera, …, update, 50, …)）。
Relay／Servo 的 trigger_counter 也是以 50 Hz tick 計（Relay.cpp:38 註解 assumes 50hz）。"""

MISSION_UPDATE_HZ = 10
"""mission.update 頻率（ArduPlane navigate 10 Hz；呼叫鏈未在原始碼副本核對，見模組說明）。"""

F2_COLUMNS = ("image_id", "file", "uav", "strip", "seq", "t_fc_us", "t_gpst_s", "lat_deg", "lon_deg", "h_ell_m",
              "h_orth_m", "yaw_deg", "pitch_deg", "roll_deg", "cog_deg", "speed_n_mps", "speed_e_mps", "speed_d_mps",
              "sigma_h_m", "sigma_v_m", "source", "crs_h", "crs_v", "flags")
"""photos.csv（F2）欄位，順序即標頭。"""

_WGS84_A = 6378137.0
_WGS84_F = 1.0 / 298.257223563
_MAX_CELLS = 25_000_000            # 格網上限：2 m 格距約 10 km × 10 km
_TICKS_PER_NAV = CAMERA_UPDATE_HZ // MISSION_UPDATE_HZ
_HORIZON_COS = math.cos(math.radians(85.0))   # 角點視線離鉛垂 > 85° 視為看到地平線

# ---- MAV_CMD（數字與 hv_dccpp/ardupilot.py 相同；本模組不 import 規劃器）----
_NAV_WAYPOINT = 16
_NAV_LOITER_UNLIM = 17
_NAV_LOITER_TURNS = 18
_NAV_LOITER_TIME = 19
_NAV_RETURN_TO_LAUNCH = 20
_NAV_LAND = 21
_NAV_TAKEOFF = 22
_NAV_LOITER_TO_ALT = 31
_NAV_VTOL_TAKEOFF = 84
_NAV_VTOL_LAND = 85
_NAV_DELAY = 93
_DO_JUMP = 177
_DO_CHANGE_SPEED = 178
_DO_LAND_START = 189
_DO_DIGICAM_CONTROL = 203
_DO_SET_CAM_TRIGG_DIST = 206
_IMAGE_START_CAPTURE = 2000
_IMAGE_STOP_CAPTURE = 2001

_NAV_SUPPORTED = frozenset({_NAV_WAYPOINT, _NAV_LOITER_UNLIM, _NAV_LOITER_TURNS, _NAV_LOITER_TIME,
                            _NAV_RETURN_TO_LAUNCH, _NAV_LAND, _NAV_TAKEOFF, _NAV_LOITER_TO_ALT,
                            _NAV_VTOL_TAKEOFF, _NAV_VTOL_LAND, _NAV_DELAY})
_NAV_LOITERS = frozenset({_NAV_LOITER_UNLIM, _NAV_LOITER_TURNS, _NAV_LOITER_TIME, _NAV_LOITER_TO_ALT})
_UNSUPPORTED = {
    _DO_JUMP: "DO_JUMP(177)：跳轉後的航線不在本模擬範圍",
    _IMAGE_START_CAPTURE: "IMAGE_START_CAPTURE(2000)：時間制連拍未建模",
    _IMAGE_STOP_CAPTURE: "IMAGE_STOP_CAPTURE(2001)：時間制連拍未建模",
}
# MAV_FRAME → 高度基準
_FRAME_KIND = {0: "msl", 5: "msl", 3: "rel", 6: "rel", 10: "terrain", 11: "terrain"}


# ---------------------------------------------------------------------------
# 本地切平面
# ---------------------------------------------------------------------------
class LocalFrame:
    """以 (lat0, lon0) 為原點的本地東北切平面（公尺）；誤差見模組說明。"""

    def __init__(self, lat0: float, lon0: float):
        self.lat0 = float(lat0)
        self.lon0 = float(lon0)
        e2 = _WGS84_F * (2.0 - _WGS84_F)
        s = math.sin(math.radians(self.lat0))
        w = 1.0 - e2 * s * s
        m = _WGS84_A * (1.0 - e2) / w ** 1.5          # 子午圈曲率半徑
        n = _WGS84_A / math.sqrt(w)                    # 卯酉圈曲率半徑
        self._kx = n * math.cos(math.radians(self.lat0)) * math.pi / 180.0
        self._ky = m * math.pi / 180.0

    def to_xy(self, lat, lon) -> tuple[np.ndarray, np.ndarray]:
        lat = np.asarray(lat, dtype=float)
        lon = np.asarray(lon, dtype=float)
        return (lon - self.lon0) * self._kx, (lat - self.lat0) * self._ky

    def to_latlon(self, x, y) -> tuple[np.ndarray, np.ndarray]:
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        return self.lat0 + y / self._ky, self.lon0 + x / self._kx


# ---------------------------------------------------------------------------
# 相機、風
# ---------------------------------------------------------------------------
@dataclass(frozen=True, slots=True)
class FrameCamera:
    """針孔框幅相機（正下視）。影像寬方向 = 橫向（機翼），高方向 = 縱向（機頭）。

    例：RPi HQ IMX477 + 6 mm → ``FrameCamera(6.0, 1.55, 4056, 3040)``，120 m 足跡 125.7 × 94.2 m。
    """
    focal_mm: float
    pixel_um: float
    width_px: int
    height_px: int

    def __post_init__(self):
        for name in ("focal_mm", "pixel_um", "width_px", "height_px"):
            v = getattr(self, name)
            if not (isinstance(v, (int, float)) and math.isfinite(v) and v > 0):
                raise ValueError(f"FrameCamera.{name} 必須是正數，收到 {v!r}")

    def footprint_m(self, agl_m: float) -> tuple[float, float]:
        """(橫向寬 w, 縱向長 l)，公尺。"""
        k = float(agl_m) * (self.pixel_um * 1e-6) / (self.focal_mm * 1e-3)
        return k * self.width_px, k * self.height_px

    def gsd_m(self, agl_m: float) -> float:
        return float(agl_m) * (self.pixel_um * 1e-6) / (self.focal_mm * 1e-3)


_IMAGE_SIZE_ATTRS = (("width_px", "height_px"), ("image_width_px", "image_height_px"), ("image_width", "image_height"))
_SENSOR_SIZE_RTOL = 1e-3      # 感測器尺寸（mm）與 像素間距×像素數 兩種給法的容許相對差


def _pos_attr(camera: Any, name: str) -> float:
    v = getattr(camera, name)
    try:
        f = float(v)
    except (TypeError, ValueError):
        f = float("nan")
    if not (math.isfinite(f) and f > 0):
        raise ValueError(f"相機 {type(camera).__name__}.{name} 必須是正數，收到 {v!r}")
    return f


def _footprint_model(camera: Any) -> tuple[Callable[[float], tuple[float, float]], str]:
    """相機 → (agl ↦ (w 橫向, l 縱向), 說明用了哪個公式)。接受：

    * :class:`FrameCamera` 或任何有 ``footprint_m(agl)`` 的物件；
    * hv_dccpp 的 Sensor（``swath_m`` / ``footprint_len_m``）；
    * hv_dccpp 的 ``CameraHardware``（規劃器本身用的相機）：``focal_length_mm`` + ``sensor_width_mm`` /
      ``sensor_height_mm`` ⇒ w = AGL·sensor_width_mm/focal_length_mm（與 platform.sensor_model 的 swath 同式）；
      或 ``focal_length_mm`` + ``pixel_pitch_um`` + 影像像素寬高（``width_px``/``height_px``、``image_width_px``/
      ``image_height_px`` 或 ``image_width``/``image_height``）。兩種都給時必須一致（相對差 ≤ 1e-3），
      不一致 → ValueError（不靜默挑一個）。
    """
    if hasattr(camera, "footprint_m"):
        return camera.footprint_m, f"{type(camera).__name__}.footprint_m"
    if hasattr(camera, "swath_m") and hasattr(camera, "footprint_len_m"):
        return (lambda agl: (camera.swath_m(agl), camera.footprint_len_m(agl))), \
            f"{type(camera).__name__}.swath_m/footprint_len_m"
    if hasattr(camera, "focal_length_mm"):
        name = type(camera).__name__
        f_mm = _pos_attr(camera, "focal_length_mm")
        sensor = None
        if hasattr(camera, "sensor_width_mm") and hasattr(camera, "sensor_height_mm"):
            sensor = (_pos_attr(camera, "sensor_width_mm"), _pos_attr(camera, "sensor_height_mm"))
        pixels = None
        size_keys = next((k for k in _IMAGE_SIZE_ATTRS if hasattr(camera, k[0]) and hasattr(camera, k[1])), None)
        if hasattr(camera, "pixel_pitch_um") and size_keys is not None:
            pix = _pos_attr(camera, "pixel_pitch_um")
            pixels = (pix * 1e-3 * _pos_attr(camera, size_keys[0]), pix * 1e-3 * _pos_attr(camera, size_keys[1]))
        if sensor is not None and pixels is not None:
            for what, a, b in (("寬", sensor[0], pixels[0]), ("高", sensor[1], pixels[1])):
                if abs(a - b) > _SENSOR_SIZE_RTOL * max(a, b):
                    raise ValueError(f"相機 {name} 的感測器{what} {a:.6g} mm 與 像素間距×像素數 {b:.6g} mm 不一致"
                                     f"（相對差 {abs(a - b) / max(a, b):.2%} > {_SENSOR_SIZE_RTOL:.1%}）；"
                                     "請修正其中一個，不會自行挑選")
        dims = sensor if sensor is not None else pixels
        if dims is None:
            raise TypeError(f"相機 {name} 有 focal_length_mm，但沒有 sensor_width_mm/sensor_height_mm，"
                            "也沒有 pixel_pitch_um + 影像像素寬高 ⇒ 算不出足跡")
        sw, sh = dims
        how = "sensor_width_mm/sensor_height_mm" if sensor is not None else f"pixel_pitch_um×{'/'.join(size_keys)}"
        return (lambda agl: (float(agl) * sw / f_mm, float(agl) * sh / f_mm)), \
            f"{name}: w = AGL·{sw:.6g}/{f_mm:g}、l = AGL·{sh:.6g}/{f_mm:g}（{how}）"
    raise TypeError("相機物件要有 footprint_m(agl)、swath_m/footprint_len_m，或 focal_length_mm + "
                    "sensor_width_mm/sensor_height_mm（或 pixel_pitch_um + 影像像素寬高），"
                    f"收到 {type(camera).__name__}")


def _footprint_wl(camera: Any, agl_m: float, model: Callable[[float], tuple[float, float]] | None = None
                  ) -> tuple[float, float]:
    """相機在 AGL 的 (w 橫向, l 縱向)；接受的相機形狀見 :func:`_footprint_model`。"""
    fn = model if model is not None else _footprint_model(camera)[0]
    w, l_ = fn(agl_m)
    w, l_ = float(w), float(l_)
    if not (math.isfinite(w) and math.isfinite(l_) and w > 0 and l_ > 0):
        raise ValueError(f"相機在 AGL {agl_m:.2f} m 的足跡無效：w={w}, l={l_}")
    return w, l_


@dataclass(frozen=True, slots=True)
class WindSolution:
    heading_deg: float       # 機頭（＝相機）方向，真北順時針
    groundspeed_mps: float
    crab_deg: float          # heading − track；右偏為正


def wind_triangle(track_deg: float, airspeed_mps: float, wind_speed_mps: float,
                  wind_from_deg: float) -> WindSolution:
    """風三角：要沿 ``track_deg`` 飛，機頭要偏多少、地速多少。

    ``wind_from_deg`` 是氣象慣例的「來向」（真北順時針）。側風 ≥ 空速或地速 ≤ 0 時無解 → ValueError。
    例：航跡 0°、風從 270°（西風）5 m/s、空速 19 m/s → 機頭 344.74°（偏流 −15.26°）、地速 18.33 m/s。
    """
    va = float(airspeed_mps)
    w = float(wind_speed_mps)
    if not (math.isfinite(va) and va > 0):
        raise ValueError(f"空速必須為正，收到 {airspeed_mps!r}")
    if not (math.isfinite(w) and w >= 0):
        raise ValueError(f"風速必須 ≥ 0，收到 {wind_speed_mps!r}")
    chi = math.radians(float(track_deg))
    th = math.radians(float(wind_from_deg))
    we, wn = -w * math.sin(th), -w * math.cos(th)            # 風「吹往」的東、北分量
    w_along = we * math.sin(chi) + wn * math.cos(chi)
    w_cross = we * math.cos(chi) - wn * math.sin(chi)         # 正 = 往航跡右側吹
    if abs(w_cross) >= va:
        raise ValueError(f"側風 {abs(w_cross):.2f} m/s ≥ 空速 {va:.2f} m/s，航跡 {track_deg:.1f}° 無法維持")
    delta = math.asin(-w_cross / va)
    gs = va * math.cos(delta) + w_along
    if gs <= 0:
        raise ValueError(f"地速 {gs:.2f} m/s ≤ 0（逆風 ≥ 空速），航跡 {track_deg:.1f}° 無法前進")
    crab = math.degrees(delta)
    return WindSolution((float(track_deg) + crab) % 360.0, gs, crab)


# ---------------------------------------------------------------------------
# 照片位姿
# ---------------------------------------------------------------------------
@dataclass(slots=True)
class PhotoPose:
    """一張照片的拍攝位姿（飛前模擬或飛後 photos.csv）。"""
    uav: str
    strip: str | None        # None = 不屬於任何條帶（不進前向／側向重疊）
    seq: int                 # 每架 1 起
    lat_deg: float
    lon_deg: float
    h_m: float               # 相機高度；與地形同一垂直基準（飛前 = MSL）
    yaw_deg: float           # 機身／相機航向，真北順時針（不是航跡）
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    cog_deg: float | None = None
    groundspeed_mps: float | None = None
    t_s: float | None = None          # 飛前：自任務起點的模擬秒數
    trigger: str = ""                 # 飛前：distance / immediate / pending / digicam
    image_id: str = ""


def _ground_fn(terrain: Any, what: str) -> Callable[[float, float], float]:
    """terrain：callable(lat, lon) → 地面高，或常數。回 None/NaN 一律丟例外（不拿 0 補）。"""
    if terrain is None:
        raise ValueError(f"{what}：需要地形（callable 或常數）")
    if isinstance(terrain, (int, float)):
        g = float(terrain)
        if not math.isfinite(g):
            raise ValueError(f"{what}：地形常數無效 {terrain!r}")
        return lambda lat, lon: g

    def f(lat: float, lon: float) -> float:
        v = terrain(lat, lon)
        if v is None or not math.isfinite(float(v)):
            raise ValueError(f"{what}：terrain({lat:.7f}, {lon:.7f}) 沒有值（{v!r}）；不會以 0 代替")
        return float(v)
    return f


# ---------------------------------------------------------------------------
# 任務項目正規化
# ---------------------------------------------------------------------------
@dataclass(frozen=True, slots=True)
class _Item:
    seq: int
    command: int
    frame: int
    p1: float
    p2: float
    p3: float
    p4: float
    lat: float
    lon: float
    alt: float


def _as_item(obj: Any) -> _Item:
    """MissionItem（x/y/z）、dict（lat/lon/alt 或 x/y/z；param1-4 或 p1-4）、或 10-tuple
    ``(seq, command, frame, p1, p2, p3, p4, lat, lon, alt)``。"""
    if isinstance(obj, Mapping):
        def g(*keys, default=None):
            for k in keys:
                if k in obj and obj[k] is not None:
                    return obj[k]
            if default is None:
                raise ValueError(f"任務項目缺 {keys[0]}：{dict(obj)!r}")
            return default
        return _Item(int(g("seq")), int(g("command")), int(g("frame")),
                     float(g("param1", "p1", default=0.0)), float(g("param2", "p2", default=0.0)),
                     float(g("param3", "p3", default=0.0)), float(g("param4", "p4", default=0.0)),
                     float(g("lat", "x", default=0.0)), float(g("lon", "y", default=0.0)),
                     float(g("alt", "z", default=0.0)))
    if hasattr(obj, "command") and hasattr(obj, "frame") and hasattr(obj, "param1"):
        return _Item(int(obj.seq), int(obj.command), int(obj.frame), float(obj.param1), float(obj.param2),
                     float(obj.param3), float(obj.param4), float(obj.x), float(obj.y), float(obj.z))
    if isinstance(obj, (tuple, list)) and len(obj) == 10:
        s, c, f, p1, p2, p3, p4, la, lo, al = obj
        return _Item(int(s), int(c), int(f), float(p1), float(p2), float(p3), float(p4),
                     float(la), float(lo), float(al))
    raise TypeError("任務項目要是 MissionItem、dict 或 (seq, command, frame, p1, p2, p3, p4, lat, lon, alt)，"
                    f"收到 {obj!r}")


# ---------------------------------------------------------------------------
# 飛前模擬
# ---------------------------------------------------------------------------
@dataclass
class SimulationResult:
    uav: str
    photos: list[PhotoPose]
    notes: list[str]
    effective: dict[str, Any]


@dataclass
class _Nav:
    seq: int
    command: int
    s: float                  # 該 NAV 頂點的水平累積距離
    accept_m: float
    dos: list[_Item] = field(default_factory=list)


def _per_uav(value: Any, uav: str, what: str) -> Any:
    if isinstance(value, Mapping):
        if uav not in value:
            raise ValueError(f"{what} 沒有給 {uav!r} 的值（給了 {sorted(value)}）")
        return value[uav]
    return value


def _land_start_closer(rest: Sequence[_Item], i: int) -> int | None:
    """``rest[i]`` 是 DO_LAND_START、且照任務順序相機此時開著：失效保護跳到這裡後，它與下一個 NAV 之間的 DO
    會依序執行。這串 DO 跑完相機是關的 → 回傳關掉它的那個 206(p1 ≤ 0) 的 seq；仍開著（沒有 206(0)，或 206(0)
    之後又有 206(d>0)）→ None。"""
    closer: int | None = None
    for it in rest[i + 1:]:
        if it.command in _NAV_SUPPORTED:
            break
        if it.command == _DO_SET_CAM_TRIGG_DIST:
            closer = it.seq if it.p1 <= 0 else None
    return closer


def _check_wind(wind: Any) -> tuple[float, float]:
    try:
        spd, frm = (float(v) for v in wind)
    except Exception as e:  # noqa: BLE001
        raise ValueError(f"wind 要是 (風速 m/s, 來向°)，收到 {wind!r}") from e
    if not (math.isfinite(spd) and spd >= 0 and math.isfinite(frm)):
        raise ValueError(f"wind 無效：{wind!r}")
    return spd, frm % 360.0


def simulate_mission_photos(items: Sequence[Any], *, uav: str, airspeed_mps: float,
                            wind: tuple[float, float], terrain: Any = None,
                            home_alt_msl: float | None = None, intrval_min_s: float = 0.0,
                            trigger_duration_s: float = 0.1, accept_radius: str | float = "mission",
                            airspeed_limits_mps: tuple[float, float] | None = None) -> SimulationResult:
    """模擬一架的任務：哪裡拍、拍幾張、每張的航向。

    參數
    ----
    items : 任務項目，第 0 列（seq 0）視為 home 槽（與 ArduPilot 相同）。
    airspeed_mps : 任務開始時的巡航空速（AIRSPEED_CRUISE）；任務中的 DO_CHANGE_SPEED 會改它。
    wind : (風速 m/s, 來向°)；無風請明確給 (0, 0)。
    terrain : callable(lat, lon) → 地面 MSL、常數，或 None（平地，地面 = home 高度）。
    home_alt_msl : home 地面 MSL；None → 用第 0 列（frame 0）的 z，來源記在 effective。
    intrval_min_s : CAM1_INTRVAL_MIN（秒，0–10）。
    trigger_duration_s : CAM1_DURATION（秒，ArduPilot 預設 0.1）。
    accept_radius : "mission" → 用每個 NAV_WAYPOINT 的 param2（ArduPlane 語意）；數字 → 全部改用它。
    airspeed_limits_mps : (AIRSPEED_MIN, AIRSPEED_MAX)；給了才模擬「超出範圍靜默不套用」。
    """
    notes: list[str] = []
    va0 = float(airspeed_mps)
    if not (math.isfinite(va0) and va0 > 0):
        raise ValueError(f"{uav}：airspeed_mps 必須為正，收到 {airspeed_mps!r}")
    w_spd, w_from = _check_wind(wind)
    tmin = float(intrval_min_s)
    if not (0.0 <= tmin <= 10.0):
        raise ValueError(f"{uav}：CAM1_INTRVAL_MIN 範圍 0–10 s，收到 {intrval_min_s!r}")
    dur = float(trigger_duration_s)
    if not (0.0 <= dur <= 50.0):
        raise ValueError(f"{uav}：CAM1_DURATION 範圍 0–50 s，收到 {trigger_duration_s!r}")
    if not (accept_radius == "mission" or (isinstance(accept_radius, (int, float)) and accept_radius >= 0)):
        raise ValueError(f"{uav}：accept_radius 要是 'mission' 或 ≥ 0 的數字，收到 {accept_radius!r}")
    # 飛控以 float32 運算再截斷成整數（Backend.cpp:131、Relay.cpp:39）
    tmin_ms = int(np.float32(tmin) * np.float32(1000.0))
    dur_ticks = int(min(np.float32(dur) * np.float32(CAMERA_UPDATE_HZ), 65535.0))

    its = [_as_item(o) for o in items]
    effective: dict[str, Any] = {
        "airspeed_initial_mps": va0, "wind": (w_spd, w_from), "intrval_min_s": tmin, "intrval_min_ms": tmin_ms,
        "trigger_duration_s": dur, "trigger_duration_ticks": dur_ticks, "accept_radius": accept_radius,
        "airspeed_limits_mps": airspeed_limits_mps, "airspeed_changes": [], "trigger_distances_m": [],
    }
    if not its:
        return SimulationResult(uav, [], ["任務是空的"], effective)

    home = its[0] if its[0].seq == 0 else None
    rest = its[1:] if home is not None else its
    if home_alt_msl is not None:
        home_alt, src = float(home_alt_msl), "argument"
    elif home is not None and _FRAME_KIND.get(home.frame) == "msl":
        home_alt, src = float(home.alt), "mission_item0"
    else:
        home_alt, src = None, "none"
    effective["home_alt_msl"] = home_alt
    effective["home_alt_source"] = src

    if terrain is None:
        if home_alt is None:
            raise ValueError(f"{uav}：沒給地形也沒有 home 高度，無法定義平地")
        ground = _ground_fn(home_alt, uav)
        effective["terrain"] = f"flat@{home_alt:.2f}m (home)"
    else:
        ground = _ground_fn(terrain, uav)
        effective["terrain"] = "constant" if isinstance(terrain, (int, float)) else "callable"

    # 本地座標以 home（或第一個有座標的 NAV）為原點
    anchor = home if home is not None and (home.lat or home.lon) else next(
        (it for it in rest if it.command in _NAV_SUPPORTED and (it.lat or it.lon)), None)
    if anchor is None:
        return SimulationResult(uav, [], ["任務沒有任何帶座標的航點"], effective)
    frame = LocalFrame(anchor.lat, anchor.lon)

    def alt_msl(it: _Item) -> float:
        kind = _FRAME_KIND.get(it.frame)
        if kind == "msl":
            return it.alt
        if kind == "rel":
            if home_alt is None:
                raise ValueError(f"{uav}：seq {it.seq} 是相對 home 高度（frame {it.frame}），但沒有 home 高度")
            return home_alt + it.alt
        if kind == "terrain":
            return ground(it.lat, it.lon) + it.alt
        raise ValueError(f"{uav}：seq {it.seq} 的 frame {it.frame} 不支援")

    # ---- 1) 走一遍任務：航線頂點與 NAV／DO 結構 ----
    if home is not None and (home.lat or home.lon):
        hx, hy = frame.to_xy(home.lat, home.lon)
        vx, vy = [float(hx)], [float(hy)]
        va = [ground(home.lat, home.lon)]                  # 起點在 home 地面
    else:
        vx, vy, va = [], [], []
    vs = [0.0] * len(vx)
    navs: list[_Nav] = []
    pre_dos: list[_Item] = []
    nav_p2_zero = 0
    passby = 0

    def add_vertex(x: float, y: float, a: float) -> float:
        if vx:
            vs.append(vs[-1] + math.hypot(x - vx[-1], y - vy[-1]))
        else:
            vs.append(0.0)
        vx.append(x)
        vy.append(y)
        va.append(a)
        return vs[-1]

    stopped = False
    for it in rest:
        c = it.command
        if c in _UNSUPPORTED:
            raise ValueError(f"{uav}：seq {it.seq} {_UNSUPPORTED[c]}")
        if c not in _NAV_SUPPORTED and 16 <= c < 100:
            raise ValueError(f"{uav}：seq {it.seq} NAV 指令 {c} 未支援，航線無法模擬")
        if c not in _NAV_SUPPORTED:
            (navs[-1].dos if navs else pre_dos).append(it)
            continue
        if stopped:
            continue
        if c == _NAV_RETURN_TO_LAUNCH:
            notes.append(f"seq {it.seq} RTL：航線模擬到此為止")
            navs.append(_Nav(it.seq, c, vs[-1] if vs else 0.0, 0.0))
            stopped = True
            continue
        has_pos = bool(it.lat or it.lon)
        if not vx and not has_pos:
            continue
        if has_pos:
            x, y = (float(v) for v in frame.to_xy(it.lat, it.lon))
        else:
            x, y = vx[-1], vy[-1]
        if c == _NAV_DELAY:
            s = add_vertex(x, y, va[-1]) if vx else 0.0
        elif c == _NAV_VTOL_LAND:
            add_vertex(x, y, va[-1] if va else alt_msl(it))   # 先平飛到落點上空
            s = add_vertex(x, y, alt_msl(it))
        else:
            s = add_vertex(x, y, alt_msl(it))
        acc = 0.0
        if c == _NAV_WAYPOINT:
            if accept_radius == "mission":
                acc = float(min(int(max(it.p2, 0.0)), 255))     # AP_Mission：uint16 截斷、上限 255
                if acc == 0.0:
                    nav_p2_zero += 1
            else:
                acc = float(accept_radius)
            if it.p3 > 0:
                passby += 1
        navs.append(_Nav(it.seq, c, s, acc))

    if nav_p2_zero:
        notes.append(f"{nav_p2_zero} 個 NAV_WAYPOINT param2=0：ArduPlane 會用 WP_RADIUS 依轉角提前判定抵達，"
                     "本模擬以 0 計（DO 執行點可能更早）")
    if passby:
        notes.append(f"{passby} 個 NAV_WAYPOINT 有 pass-by（param3>0），未建模")
    effective["nav_p2_zero_count"] = nav_p2_zero

    # ---- 靜態檢查：相機開著時遇到盤旋／降落序列／任務結束 ----
    on = False
    land_start_checks: list[tuple[int, str]] = []
    for i_it, it in enumerate(rest):
        if it.command == _DO_SET_CAM_TRIGG_DIST:
            on = it.p1 > 0
            if on:
                effective["trigger_distances_m"].append(it.p1)
        elif on and it.command in _NAV_LOITERS:
            notes.append(f"seq {it.seq} 盤旋時相機開著：盤旋中的照片未建模")
        elif on and it.command == _DO_LAND_START:
            # 跳到 DO_LAND_START 後，它與下一個 NAV 之間的 DO 會依序執行（AP_Mission advance_current_do_cmd）
            # ⇒ 其後緊接的 206(0) 就會關相機；那一串 DO 跑完相機仍開著才是真的開著。
            closer = _land_start_closer(rest, i_it)
            if closer is None:
                land_start_checks.append((it.seq, "open"))
                notes.append(f"seq {it.seq} DO_LAND_START 落在 206 開啟區間內：失效保護 RTL 跳到這裡時相機不會關")
            else:
                land_start_checks.append((it.seq, f"closed_by_seq_{closer}"))
        elif on and it.command in (_NAV_VTOL_LAND, _NAV_LAND, _NAV_RETURN_TO_LAUNCH):
            notes.append(f"seq {it.seq} 降落／RTL 時相機仍開著")
    if on:
        notes.append("任務結束時相機仍開著（最後沒有 206(0)）")
    effective["trigger_distances_m"] = sorted(set(effective["trigger_distances_m"]))
    effective["land_start_in_open_206"] = land_start_checks

    if len(vx) < 1:
        return SimulationResult(uav, [], notes + ["任務沒有可模擬的航線"], effective)
    VX, VY, VA, VS = (np.asarray(v, dtype=float) for v in (vx, vy, va, vs))
    s_end = float(VS[-1])
    seg_len = np.diff(VS)
    seg_track = np.degrees(np.arctan2(np.diff(VX), np.diff(VY))) % 360.0 if len(VX) > 1 else np.zeros(0)
    horiz = np.nonzero(seg_len > 1e-9)[0]

    def seg_at(s: float) -> int:
        """含 s 的水平線段索引（s 在垂直段上時取其後第一個水平段；都沒有 → -1）。"""
        if horiz.size == 0:
            return -1
        j = int(np.searchsorted(VS[horiz + 1], s, side="right"))
        return int(horiz[min(j, horiz.size - 1)])

    lim = airspeed_limits_mps

    def change_speed(it: _Item, cur: float) -> float:
        kind, val = int(it.p1), float(it.p2)
        if kind == 0:
            if abs(val - (-2.0)) < 1e-6:
                effective["airspeed_changes"].append((it.seq, va0, "default"))
                return va0
            if val <= 0:
                return cur
            if lim is not None and not (lim[0] <= val <= lim[1]):
                effective["airspeed_changes"].append((it.seq, val, "ignored_out_of_range"))
                notes.append(f"seq {it.seq} DO_CHANGE_SPEED {val:.2f} m/s 超出 AIRSPEED_MIN/MAX {lim}："
                             "飛控會靜默不套用，本模擬照做")
                return cur
            effective["airspeed_changes"].append((it.seq, val, "applied" if lim is not None else "assumed"))
            return val
        if kind == 1:
            effective["airspeed_changes"].append((it.seq, val, "groundspeed_not_modelled"))
            notes.append(f"seq {it.seq} DO_CHANGE_SPEED 地速型（param1=1）只改 MIN_GROUNDSPEED，空速不變")
        return cur

    # ---- 2) DO 事件：抵達點（依接受半徑提前）與空速變化點 ----
    cam_events: list[tuple[float, int, _Item]] = []      # (抵達點 s, DO 序號 k, 項目)
    speed_pts: list[tuple[float, float]] = [(0.0, va0)]  # (生效 s, 空速)
    cur_va = va0

    def schedule(s_reach: float, dos: list[_Item]) -> None:
        nonlocal cur_va
        for k, d in enumerate(dos, start=1):
            if d.command == _DO_CHANGE_SPEED:
                lag_s = 0.0 if k == 1 else (2 * k - 3) / MISSION_UPDATE_HZ
                j = seg_at(s_reach)
                gs = cur_va if j < 0 else wind_triangle(float(seg_track[j]), cur_va, w_spd, w_from).groundspeed_mps
                new_va = change_speed(d, cur_va)
                if new_va != cur_va:
                    speed_pts.append((min(s_reach + lag_s * gs, s_end), new_va))
                    cur_va = new_va
            elif d.command in (_DO_SET_CAM_TRIGG_DIST, _DO_DIGICAM_CONTROL):
                cam_events.append((s_reach, k, d))

    schedule(0.0, pre_dos)
    s_prev = 0.0
    for nv in navs:
        s_reach = max(s_prev, nv.s - nv.accept_m)
        schedule(s_reach, nv.dos)
        s_prev = s_reach

    # ---- 3) 分段：每段固定航跡與空速 → 地速、航向、時間 ----
    sp_s = np.array([p[0] for p in speed_pts])
    sp_v = np.array([p[1] for p in speed_pts])
    order = np.argsort(sp_s, kind="stable")
    sp_s, sp_v = sp_s[order], sp_v[order]
    p_s0, p_s1, p_seg, p_gs, p_hd, p_tr, p_t0 = [], [], [], [], [], [], []
    t_acc = 0.0
    for j in horiz:
        a, b = float(VS[j]), float(VS[j + 1])
        cuts = [a] + [float(s) for s in sp_s if a < s < b] + [b]
        for s0, s1 in zip(cuts[:-1], cuts[1:]):
            if s1 - s0 <= 1e-9:
                continue
            v_air = float(sp_v[int(np.searchsorted(sp_s, s0, side="right")) - 1])
            ws = wind_triangle(float(seg_track[j]), v_air, w_spd, w_from)
            p_s0.append(s0)
            p_s1.append(s1)
            p_seg.append(int(j))
            p_gs.append(ws.groundspeed_mps)
            p_hd.append(ws.heading_deg)
            p_tr.append(float(seg_track[j]))
            p_t0.append(t_acc)
            t_acc += (s1 - s0) / ws.groundspeed_mps
    t_end = t_acc
    P_s0, P_gs, P_t0 = np.array(p_s0), np.array(p_gs), np.array(p_t0)

    def time_at(s: float) -> float:
        if P_s0.size == 0:
            return 0.0
        i = max(int(np.searchsorted(P_s0, s, side="right")) - 1, 0)
        return float(P_t0[i] + max(min(s, p_s1[i]) - P_s0[i], 0.0) / P_gs[i])

    # 抵達偵測在 10 Hz 的 mission.update；第 k≥2 個 DO 晚 (2k−3) 次
    ev_ticks: list[tuple[int, int, _Item]] = []
    for n, (s_r, k, d) in enumerate(cam_events):
        tick = int(math.ceil(time_at(s_r) * MISSION_UPDATE_HZ - 1e-9)) * _TICKS_PER_NAV
        if k >= 2:
            tick += (2 * k - 3) * _TICKS_PER_NAV
        ev_ticks.append((tick, n, d))
    ev_ticks.sort(key=lambda e: (e[0], e[1]))
    if not ev_ticks:
        return SimulationResult(uav, [], notes, effective)

    # ---- 4) 50 Hz tick 位置表 ----
    tail = int(math.ceil(tmin * CAMERA_UPDATE_HZ)) + dur_ticks + 2
    n_ticks = max(int(math.ceil(t_end * CAMERA_UPDATE_HZ)), ev_ticks[-1][0]) + tail + 1
    tt = np.arange(n_ticks) / CAMERA_UPDATE_HZ
    if P_s0.size:
        pi = np.clip(np.searchsorted(P_t0, tt, side="right") - 1, 0, P_s0.size - 1)
        ss = np.minimum(P_s0[pi] + (tt - P_t0[pi]) * P_gs[pi], np.array(p_s1)[pi])
        jj = np.array(p_seg)[pi]
        fr = (ss - VS[jj]) / seg_len[jj]
        TX = VX[jj] + fr * (VX[jj + 1] - VX[jj])
        TY = VY[jj] + fr * (VY[jj + 1] - VY[jj])
        TA = VA[jj] + fr * (VA[jj + 1] - VA[jj])
        THD, TTR, TGS = np.array(p_hd)[pi], np.array(p_tr)[pi], P_gs[pi]
        # 航線結束後停在終點
        past = tt > t_end
        TX[past], TY[past], TA[past] = VX[-1], VY[-1], VA[-1]
    else:
        TX = np.full(n_ticks, VX[-1])
        TY = np.full(n_ticks, VY[-1])
        TA = np.full(n_ticks, VA[-1])
        THD = TTR = np.zeros(n_ticks)
        TGS = np.zeros(n_ticks)
    TXl, TYl = TX.tolist(), TY.tolist()

    # ---- 5) AP_Camera 狀態機 ----
    photos: list[PhotoPose] = []
    trigg = 0.0
    last: tuple[float, float] | None = None
    last_ms = -10 ** 12
    pending = False
    pending_strip: str | None = None
    counter = 0
    strip_no = 0
    strip_on = False

    def take(k: int, how: str) -> bool:
        nonlocal pending, pending_strip, counter, last_ms, last
        now_ms = k * (1000 // CAMERA_UPDATE_HZ)
        if now_ms - last_ms < tmin_ms:      # INTRVAL_MIN：延後、不告警
            if not pending:
                pending_strip = str(strip_no) if strip_on else None
            pending = True
            return False
        was_pending = pending
        pending = False
        if counter > 0:                     # 脈衝未結束：trigger_pic 失敗
            return False
        counter = dur_ticks
        last_ms = now_ms
        last = (TXl[k], TYl[k])
        lat, lon = frame.to_latlon(TXl[k], TYl[k])
        strip = pending_strip if was_pending else (str(strip_no) if strip_on else None)
        photos.append(PhotoPose(uav=uav, strip=strip, seq=len(photos) + 1, lat_deg=float(lat), lon_deg=float(lon),
                                h_m=float(TA[k]), yaw_deg=float(THD[k]), cog_deg=float(TTR[k]),
                                groundspeed_mps=float(TGS[k]), t_s=k / CAMERA_UPDATE_HZ,
                                trigger="pending" if was_pending else how))
        return True

    ei = 0
    k = ev_ticks[0][0]
    n_ev = len(ev_ticks)
    while k < n_ticks:
        while ei < n_ev and ev_ticks[ei][0] <= k:
            d = ev_ticks[ei][2]
            if d.command == _DO_SET_CAM_TRIGG_DIST:
                if d.p1 > 0 and not strip_on:
                    strip_no += 1
                strip_on = d.p1 > 0
                trigg = d.p1
                if int(d.p3) == 1:
                    take(k, "immediate")
            elif d.command == _DO_DIGICAM_CONTROL and int(d.lat) == 1:   # param5 == 1 才拍
                take(k, "digicam")
            ei += 1
        # AP_Camera_Relay::update → AP_Camera_Backend::update
        if counter > 0:
            counter -= 1
        if pending:
            take(k, "pending")
        if trigg <= 0:
            last = None
            if not pending:
                if ei >= n_ev:
                    break
                nxt = ev_ticks[ei][0]
                counter = max(0, counter - (nxt - k - 1))
                k = nxt
                continue
            k += 1
            continue
        x, y = TXl[k], TYl[k]
        if last is None:
            last = (x, y)
        elif (x != last[0] or y != last[1]) and math.hypot(x - last[0], y - last[1]) >= trigg:
            take(k, "distance")
        k += 1
    return SimulationResult(uav, photos, notes, effective)


# ---------------------------------------------------------------------------
# 足跡投影
# ---------------------------------------------------------------------------
def _rotation_ned(yaw_deg: np.ndarray, pitch_deg: np.ndarray, roll_deg: np.ndarray) -> np.ndarray:
    """機體 FRD → NED 方向餘弦矩陣（ZYX：yaw、pitch、roll），(N, 3, 3)。"""
    y, p, r = (np.radians(np.asarray(a, dtype=float)) for a in (yaw_deg, pitch_deg, roll_deg))
    cy, sy, cp, sp, cr, sr = np.cos(y), np.sin(y), np.cos(p), np.sin(p), np.cos(r), np.sin(r)
    R = np.empty(y.shape + (3, 3))
    R[..., 0, 0] = cy * cp
    R[..., 0, 1] = cy * sp * sr - sy * cr
    R[..., 0, 2] = cy * sp * cr + sy * sr
    R[..., 1, 0] = sy * cp
    R[..., 1, 1] = sy * sp * sr + cy * cr
    R[..., 1, 2] = sy * sp * cr - cy * sr
    R[..., 2, 0] = -sp
    R[..., 2, 1] = cp * sr
    R[..., 2, 2] = cp * cr
    return R


def footprint_corners_enu(x: np.ndarray, y: np.ndarray, agl: np.ndarray, tan_half_w: np.ndarray,
                          tan_half_l: np.ndarray, yaw_deg: np.ndarray, pitch_deg: np.ndarray,
                          roll_deg: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """針孔投影：四個影像角的視線打到「相機正下方地面高」的水平面。

    回傳 (corners (N,4,2) ENU, ok (N,) bool)；ok=False 表示有角點視線接近或高於地平線（> 85°）。
    角點順序：左前、右前、右後、左後（凸四邊形）。pitch = roll = 0 時就是 w × l 矩形依 yaw 旋轉。
    """
    n = len(x)
    f = np.array([1.0, 1.0, -1.0, -1.0])          # 前(+)/後(−)
    r = np.array([-1.0, 1.0, 1.0, -1.0])          # 右(+)/左(−)
    rays = np.stack([f[None, :] * tan_half_l[:, None], r[None, :] * tan_half_w[:, None],
                     np.ones((n, 4))], axis=-1)    # (N,4,3) 機體 FRD
    R = _rotation_ned(yaw_deg, pitch_deg, roll_deg)
    d = np.einsum("nij,nkj->nki", R, rays)         # (N,4,3) NED
    norm = np.linalg.norm(d, axis=-1)
    ok = np.all(d[..., 2] > _HORIZON_COS * norm, axis=1)
    dz = np.where(d[..., 2] > 1e-9, d[..., 2], np.nan)
    t = agl[:, None] / dz
    corners = np.stack([x[:, None] + t * d[..., 1], y[:, None] + t * d[..., 0]], axis=-1)
    return corners, ok


# ---------------------------------------------------------------------------
# 度量
# ---------------------------------------------------------------------------
@dataclass(frozen=True, slots=True)
class OverlapStats:
    n: int
    minimum: float | None
    p05: float | None
    median: float | None
    maximum: float | None

    @staticmethod
    def of(values: Sequence[float] | np.ndarray) -> "OverlapStats":
        v = np.asarray(values, dtype=float)
        if v.size == 0:
            return OverlapStats(0, None, None, None, None)
        return OverlapStats(int(v.size), float(v.min()), float(np.percentile(v, 5)), float(np.median(v)),
                            float(v.max()))


@dataclass(frozen=True, slots=True)
class SeamStats:
    """**相鄰**兩架覆蓋之間的接縫（重疊帶）寬度。frac = 寬度 / ref_swath_m。

    只算「恰好只有這兩架」看到的格（第三架也看到的格不算：那裡是第三架夾在中間，兩架不必直接相接）。
    兩架的重疊全都有第三架也看到 ⇒ 不相鄰，不列出（記在 ``effective["seam_pairs_not_adjacent"]``）。
    """
    uav_a: str
    uav_b: str
    cells: int                       # 恰好只被這兩架看到、且在測區內的格數
    width_min_m: float
    width_p05_m: float | None
    width_median_m: float | None
    frac_min: float
    frac_p05: float | None
    frac_median: float | None
    ref_swath_m: float
    gap_m: float | None = None       # 沒有重疊時，兩架覆蓋之間（經由兩架都沒看到的格）的最近距離
    shared_cells: int = 0            # 兩架都看到、但第三架也看到的格數（不進寬度統計）


@dataclass(frozen=True)
class UnderCoveredArea:
    """視數 < k 的連通區（已裁到測區內），供補飛規劃。

    ``region``：整個分量都在邊緣帶（離測區邊界 ≤ ``ExposureReport.edge_band_m``）內 → ``"edge_band"``
    （多半是最外側條帶只蓋到一次的邊緣效應）；只要有一格伸進內部 → ``"interior"``（當成真破洞）。
    ``min_views`` = 0 表示有完全沒拍到的格，即使在邊緣帶也不是單純的邊緣效應。
    """
    area_m2: float
    cells: int
    min_views: int
    centroid_latlon: tuple[float, float]
    exterior_latlon: list[tuple[float, float]]
    holes_latlon: list[list[tuple[float, float]]]
    geometry_enu: Any = field(repr=False, default=None)
    region: str = "interior"
    edge_band_cells: int = 0         # 此分量落在邊緣帶內的格數


@dataclass(frozen=True, slots=True)
class RegionStats:
    """測區的一部分（邊緣帶或內部）的逐格統計；格心在測區內的格才算。"""
    cells: int
    area_m2: float                   # cells × 格面積
    frac_ge: dict[int, float]        # 視數 ≥ k 的格比例（分母 = 本區格數）
    undercovered_cells: int          # 視數 < undercover_k 的格數
    undercovered_area_m2: float
    undercovered_frac: float         # undercovered_cells / cells（本區沒有格時 0）
    min_views: int | None


@dataclass
class ExposureGrid:
    """驗算格網（本地 ENU）。格 (iy, ix) 的格心 = (x0 + (ix+0.5)c, y0 + (iy+0.5)c)。"""
    frame: LocalFrame
    x0: float
    y0: float
    cell_size_m: float
    views: np.ndarray            # (ny, nx) int32，全部照片
    inside: np.ndarray           # (ny, nx) bool，格心在測區內
    views_by_uav: dict[str, np.ndarray] = field(default_factory=dict)


@dataclass
class ExposureReport:
    cell_size_m: float
    ks: tuple[int, ...]
    undercover_k: int
    total_cells: int
    area_m2: float
    views_histogram: dict[int, int]
    frac_ge: dict[int, float]
    forward_overlap: OverlapStats
    side_overlap: OverlapStats
    seams: list[SeamStats]
    seam_overlap_min: float | None
    photos_per_uav: dict[str, int]
    photos_used: int
    photos_excluded: dict[str, int]
    undercovered: list[UnderCoveredArea]
    undercovered_area_m2: float
    notes: list[str]
    effective: dict[str, Any]
    elapsed_s: float
    frame_origin: tuple[float, float]
    photos: list[PhotoPose] = field(default_factory=list, repr=False)
    grid: ExposureGrid | None = field(default=None, repr=False)
    simulations: dict[str, SimulationResult] = field(default_factory=dict, repr=False)
    # 邊緣帶：離測區邊界（含內環）≤ edge_band_m 的格。預設 edge_band_m = w/2（w = 照片橫向足跡寬中位數）。
    # 沒有可用照片且沒給 edge_band_m 時寬度無從定義 ⇒ 三者皆 None、所有視數不足區都算 interior。
    edge_band_m: float | None = None
    edge_band: RegionStats | None = None
    interior: RegionStats | None = None
    undercovered_edge_band_area_m2: float = 0.0      # region == "edge_band" 的分量面積總和
    undercovered_interior_area_m2: float = 0.0       # region == "interior" 的分量面積總和
    # 方向分視數（主條帶 vs 交叉航帶；2026-10-15 交叉航帶與加密 GCP 對照用）。見 :func:`_direction_stats`。
    # 新欄位、只加不改：其餘欄位（含 effective）與加入前逐值相同。
    directions: dict[str, Any] | None = None


_PREFLIGHT_ASSUMPTIONS = (
    "CAM_MAX_ROLL 未建模（需要動態模型）：轉彎改出時 |roll| 超過門檻的距離觸發會被延後，實際張數可能較少",
    "飛前足跡一律 pitch = roll = 0（平飛配平攻角未計）；路徑為相鄰 NAV 間直線，轉彎切角未建模",
    "假設全程 AUTO、GPS 3D fix 正常、CAM1_TRIGG_DIST 參數初值 0",
)


def _survey_geometry(polygons: Any) -> list[tuple[list[tuple[float, float]], list[list[tuple[float, float]]]]]:
    """polygons：單一 [(lat, lon), …]、多個 ring 的串列，或 {"exterior": ring, "holes": [ring, …]} 的串列。"""
    if not polygons:
        raise ValueError("沒有測區多邊形")
    first = polygons[0]
    if isinstance(first, Mapping):
        out = [(list(p["exterior"]), [list(h) for h in p.get("holes", ())]) for p in polygons]
    elif len(first) == 2 and all(isinstance(v, (int, float)) for v in first):
        out = [(list(polygons), [])]
    else:
        out = [(list(p), []) for p in polygons]
    for ext, _h in out:
        if len(ext) < 3:
            raise ValueError(f"多邊形至少需要 3 個點，收到 {len(ext)}")
    return out


def _raster_views(quads: np.ndarray, nx: int, ny: int, chunk: int = 2000) -> np.ndarray:
    """凸四邊形（格網索引座標：格心在整數點）→ 每格被幾個四邊形蓋到，(ny, nx) int32。掃描線 + 差分。"""
    acc = np.zeros(ny * (nx + 1), dtype=np.int64)
    for c0 in range(0, len(quads), chunk):
        q = quads[c0:c0 + chunk]
        u, v = q[..., 0], q[..., 1]
        r0 = np.maximum(np.ceil(v.min(axis=1)).astype(np.int64), 0)
        r1 = np.minimum(np.floor(v.max(axis=1)).astype(np.int64), ny - 1)
        cnt = np.maximum(r1 - r0 + 1, 0)
        tot = int(cnt.sum())
        if tot == 0:
            continue
        idx = np.repeat(np.arange(len(q)), cnt)
        rows = np.repeat(r0, cnt) + (np.arange(tot) - np.repeat(np.cumsum(cnt) - cnt, cnt))
        ua, va_ = u[idx], v[idx]
        ub, vb = np.roll(u, -1, axis=1)[idx], np.roll(v, -1, axis=1)[idx]
        rr = rows[:, None].astype(float)
        lo, hi = np.minimum(va_, vb), np.maximum(va_, vb)
        hit = (rr >= lo) & (rr <= hi) & (hi > lo)
        with np.errstate(divide="ignore", invalid="ignore"):
            xs = ua + (rr - va_) * (ub - ua) / (vb - va_)
        xl = np.where(hit, xs, np.inf).min(axis=1)
        xh = np.where(hit, xs, -np.inf).max(axis=1)
        good = np.isfinite(xl) & np.isfinite(xh)
        cl = np.maximum(np.ceil(xl[good]).astype(np.int64), 0)
        ch = np.minimum(np.floor(xh[good]).astype(np.int64), nx - 1)
        rg = rows[good]
        ok = cl <= ch
        rg, cl, ch = rg[ok], cl[ok], ch[ok]
        acc += np.bincount(rg * (nx + 1) + cl, minlength=acc.size)
        acc -= np.bincount(rg * (nx + 1) + ch + 1, minlength=acc.size)
    return np.cumsum(acc.reshape(ny, nx + 1), axis=1)[:, :nx].astype(np.int32)


def _seam_widths(mx: np.ndarray, my: np.ndarray, where: np.ndarray, ref_cells: float) -> tuple[np.ndarray, bool]:
    """X、Y 兩架覆蓋（bool 格網）重疊帶的局部寬度（格數）。

    寬度 = 到「只有 X」區的距離 + 到「只有 Y」區的距離 − 1 格（量的是橫越重疊帶、兩側各自延伸到對方
    的距離；斜排足跡的鋸齒邊會讓它在齒尖處變窄，那是真的窄）。覆蓋結束處（帶的兩端）與覆蓋破洞旁，
    「到對側的最近距離」會繞到端點而偏小 ⇒ 只取離「兩架都沒看到」的格 ≥ 一個參考刈幅的帶內格；
    一格都沒有時退回全部帶內格，回傳第二個值 True 讓呼叫端寫 note。
    """
    xo, yo = mx & ~my, my & ~mx
    inf = np.full(mx.shape, np.inf)
    dx = ndimage.distance_transform_edt(~xo) if xo.any() else inf
    dy = ndimage.distance_transform_edt(~yo) if yo.any() else inf
    neither = ~(mx | my)
    dn = ndimage.distance_transform_edt(~neither) if neither.any() else inf
    core = where & (dn >= ref_cells)
    fallback = not core.any()
    sel = where if fallback else core
    return dx[sel] + dy[sel] - 1.0, fallback


def _strip_key(strip: Any) -> str | None:
    if strip is None:
        return None
    s = str(strip).strip()
    return None if s in ("", "0", "None") else s


#: 方向分視數的門檻（「這格同時被主條帶 ≥ k 張、交叉航帶 ≥ k 張照片看到」）
DIRECTION_KS: tuple[int, ...] = (1, 2, 3)


def window_direction_labels(strip_ids_by_uav: Mapping[str, Sequence[str]],
                            is_cross: Callable[[str], bool]) -> dict[tuple[str, str], str]:
    """飛前模擬的條帶鍵是「第 k 個 206 開窗」（1 起）；規劃器每條帶開一次窗 ⇒ 第 k 窗 = 該機第 k 條帶。

    ``strip_ids_by_uav``：{uav: [該機條帶 id 依飛行順序]}（例如 ``{u: [s for s, _d in plan.plans[u].strips]}``）；
    ``is_cross``：條帶 id → 是否交叉航帶（例如 ``hv_dccpp.geometry.is_cross_strip``）。
    回傳 {(uav, "k"): "cross" | "main"}，可直接當 :func:`exposure_metrics` 的 ``direction_of``。
    ⚠ 只對「每條帶恰好一個 206 開窗」的任務成立（hv 匯出器的建圖輸出就是）；別的任務請自己給對照。"""
    out: dict[tuple[str, str], str] = {}
    for u, sids in strip_ids_by_uav.items():
        for k, sid in enumerate(sids, 1):
            out[(str(u), str(k))] = "cross" if is_cross(str(sid)) else "main"
    return out


def _classify_directions(photos: Sequence[PhotoPose], idx_use: np.ndarray, strips: Mapping[tuple[str, str], list[int]],
                         info: Mapping[tuple[str, str], tuple], direction_of: Any, tol_deg: float
                         ) -> tuple[np.ndarray, str, dict[str, float]]:
    """每張（可用）照片 → "main"／"cross"／"none"（不屬任何條帶或無法判定）。

    * ``direction_of`` 是 mapping：鍵 (uav, 條帶鍵) → "main"／"cross"（例如 :func:`window_direction_labels`）；
    * ``direction_of`` 是 callable：PhotoPose → "main"／"cross"／None；
    * None（預設、自動）：每架以「照片數最多的條帶軸向（mod 180°）」為主方向，條帶軸向與它差 > tol_deg ⇒ cross。
      條帶軸向 = 該帶首末照片連線（與側向重疊同一個 ``info``；照片 < 2 張的帶判 none）。⚠ 交叉照片比主條帶還多時
      （例如面積比例 1.0 又飛得更低）自動判定會把兩者對調 ⇒ 請給明確的 ``direction_of``。"""
    lab = np.full(len(idx_use), "none", dtype=object)
    axes: dict[str, float] = {}
    if direction_of is not None:
        method = "callable" if callable(direction_of) else "mapping"
        for j, i in enumerate(idx_use):
            p = photos[int(i)]
            if callable(direction_of):
                v = direction_of(p)
            else:
                sk = _strip_key(p.strip)
                v = direction_of.get((p.uav, sk)) if sk is not None else None
            lab[j] = v if v in ("main", "cross") else "none"
        return lab, method, axes

    def d180(a: float, b: float) -> float:
        d = abs(a - b) % 180.0
        return min(d, 180.0 - d)

    per_uav: dict[str, list[tuple[tuple[str, str], float, int]]] = {}
    for key, (_a0, uvec, _nv, js, _w) in info.items():
        ang = math.degrees(math.atan2(float(uvec[0]), float(uvec[1]))) % 180.0
        per_uav.setdefault(key[0], []).append((key, ang, len(js)))
    for u, lst in per_uav.items():
        best = max(lst, key=lambda t: (sum(n for _k, a2, n in lst if d180(t[1], a2) <= tol_deg), t[2], -t[1]))
        axes[u] = float(best[1])
        for key, ang, _n in lst:
            v = "main" if d180(ang, best[1]) <= tol_deg else "cross"
            for j in strips[key]:
                lab[j] = v
    return lab, "axis", axes


def _direction_stats(lab: np.ndarray, method: str, axes: Mapping[str, float], quads_uv: np.ndarray,
                     uav_of: np.ndarray, views: np.ndarray, by_uav: Mapping[str, np.ndarray], inside: np.ndarray,
                     nx: int, ny: int, cell: float) -> dict[str, Any]:
    """方向分視數：主條帶視數 = 總視數 − 交叉 − 未分類；只統計格心在測區內的格。

    ``frac_seen``：被 ≥ 1 張主條帶／交叉航帶／兩者都有的照片看到的面積比例；``both_ge[k]``：主、交叉各 ≥ k 張；
    ``per_uav[u]["cross_on_own_main"]``：該架自己主條帶看到的格中，也被該架交叉航帶看到的比例。"""
    is_x = lab == "cross"
    is_n = lab == "none"
    vx = _raster_views(quads_uv[is_x], nx, ny) if is_x.any() else np.zeros((ny, nx), dtype=np.int32)
    vn = _raster_views(quads_uv[is_n], nx, ny) if is_n.any() else np.zeros((ny, nx), dtype=np.int32)
    vm = views - vx - vn
    m, x = vm[inside], vx[inside]
    tot = int(m.size)

    def frac(mask: np.ndarray) -> float:
        return float(mask.mean()) if tot else 0.0

    per_uav: dict[str, dict[str, Any]] = {}
    for u in sorted(by_uav):
        sel_x = is_x & (uav_of == u)
        sel_n = is_n & (uav_of == u)
        vxu = _raster_views(quads_uv[sel_x], nx, ny) if sel_x.any() else np.zeros((ny, nx), dtype=np.int32)
        vnu = _raster_views(quads_uv[sel_n], nx, ny) if sel_n.any() else np.zeros((ny, nx), dtype=np.int32)
        vmu = (by_uav[u] - vxu - vnu)[inside]
        own = vmu >= 1
        n_own = int(own.sum())
        per_uav[u] = {"photos_main": int(((lab == "main") & (uav_of == u)).sum()), "photos_cross": int(sel_x.sum()),
                      "photos_unclassified": int(sel_n.sum()), "main_axis_deg": axes.get(u),
                      "cross_on_own_main": (float((vxu[inside][own] >= 1).mean()) if n_own else 0.0)}
    return {
        "method": method,
        "photos": {"main": int((lab == "main").sum()), "cross": int(is_x.sum()), "unclassified": int(is_n.sum())},
        "frac_seen": {"main": frac(m >= 1), "cross": frac(x >= 1), "both": frac((m >= 1) & (x >= 1))},
        "both_ge": {k: frac((m >= k) & (x >= k)) for k in DIRECTION_KS},
        "cross_ge": {k: frac(x >= k) for k in DIRECTION_KS},
        "area_m2": {"cross": float((x >= 1).sum()) * cell * cell,
                    "both": float(((m >= 1) & (x >= 1)).sum()) * cell * cell},
        "per_uav": per_uav,
    }


def exposure_metrics(photos: Sequence[PhotoPose], polygons: Any, camera: Any, *, terrain: Any,
                     cell_size_m: float = 2.0, ks: Sequence[int] = (5, 9), undercover_k: int | None = None,
                     tilt: bool = True, parallel_tol_deg: float = 20.0,
                     notes: Sequence[str] = (), effective: Mapping[str, Any] | None = None,
                     edge_band_m: float | None = None, direction_of: Any = None) -> ExposureReport:
    """由照片位姿算曝光層級度量（飛前、飛後共用）。

    terrain：callable(lat, lon) 或常數，與 ``PhotoPose.h_m`` 同一垂直基準（必填）。
    tilt=False 時忽略 roll/pitch（只用 yaw）。AGL ≤ 0 或視線看到地平線的照片不進度量，數量記在
    ``photos_excluded`` 並寫進 notes。
    edge_band_m：邊緣帶寬度（公尺，≥ 0）；None → w/2（w = 照片橫向足跡寬中位數）。邊緣帶與內部分開統計
    （``edge_band``／``interior``），視數不足區逐一標 ``region``；整體的 ``frac_ge``／``undercovered`` 不變。
    direction_of：主條帶／交叉航帶的分類（見 :func:`_classify_directions`；None ＝ 依條帶軸向自動判定），
    結果在 ``ExposureReport.directions``（新欄位；其餘輸出不變）。
    """
    t_start = time.perf_counter()
    cell = float(cell_size_m)
    if not (math.isfinite(cell) and cell > 0):
        raise ValueError(f"格距必須為正，收到 {cell_size_m!r}")
    ks_t = tuple(int(k) for k in ks)
    if not ks_t or any(k < 1 for k in ks_t):
        raise ValueError(f"ks 要是 ≥ 1 的整數，收到 {ks!r}")
    uk = int(undercover_k) if undercover_k is not None else min(ks_t)
    if uk < 1:
        raise ValueError(f"undercover_k 要 ≥ 1，收到 {undercover_k!r}")
    if edge_band_m is not None:
        try:
            eb_arg = float(edge_band_m)
        except (TypeError, ValueError):
            eb_arg = float("nan")
        if not (math.isfinite(eb_arg) and eb_arg >= 0):
            raise ValueError(f"edge_band_m 要是 ≥ 0 的公尺數（None = w/2），收到 {edge_band_m!r}")
    ground = _ground_fn(terrain, "exposure_metrics")
    cam_fn, cam_desc = _footprint_model(camera)
    out_notes = list(notes)
    eff = dict(effective or {})
    eff.update({"cell_size_m": cell, "ks": ks_t, "undercover_k": uk, "tilt": bool(tilt),
                "parallel_tol_deg": float(parallel_tol_deg), "camera_model": cam_desc})

    # ---- 測區 ----
    rings = _survey_geometry(polygons)
    all_ll = np.array([p for ext, _ in rings for p in ext], dtype=float)
    frame = LocalFrame(float((all_ll[:, 0].min() + all_ll[:, 0].max()) / 2),
                       float((all_ll[:, 1].min() + all_ll[:, 1].max()) / 2))

    def ring_xy(ring):
        a = np.asarray(ring, dtype=float)
        x, y = frame.to_xy(a[:, 0], a[:, 1])
        return np.column_stack([x, y])

    survey = shapely.union_all([shapely.Polygon(ring_xy(ext), [ring_xy(h) for h in holes]).buffer(0)
                                for ext, holes in rings])
    if survey.is_empty or survey.area <= 0:
        raise ValueError("測區多邊形面積為 0")

    # ---- 照片 → 足跡 ----
    photos = list(photos)
    per_uav_all: dict[str, int] = {}
    for p in photos:
        per_uav_all[p.uav] = per_uav_all.get(p.uav, 0) + 1
    n = len(photos)
    lat = np.array([p.lat_deg for p in photos], dtype=float)
    lon = np.array([p.lon_deg for p in photos], dtype=float)
    px, py = frame.to_xy(lat, lon) if n else (np.zeros(0), np.zeros(0))
    agl = np.array([p.h_m - ground(p.lat_deg, p.lon_deg) for p in photos], dtype=float)
    excluded: dict[str, int] = {}
    good = agl > 0
    if (~good).any():
        excluded["agl_le_0"] = int((~good).sum())
        out_notes.append(f"{excluded['agl_le_0']} 張照片 AGL ≤ 0（地面試拍？），不進度量")
    tw = np.ones(n)
    tl = np.ones(n)
    for i in np.nonzero(good)[0]:
        w, l_ = _footprint_wl(camera, float(agl[i]), cam_fn)
        tw[i], tl[i] = w / (2 * agl[i]), l_ / (2 * agl[i])
    yaw = np.array([p.yaw_deg for p in photos], dtype=float)
    pitch = np.array([p.pitch_deg if tilt else 0.0 for p in photos], dtype=float)
    roll = np.array([p.roll_deg if tilt else 0.0 for p in photos], dtype=float)
    corners, ok = footprint_corners_enu(px, py, np.where(good, agl, 1.0), tw, tl, yaw, pitch, roll) if n else \
        (np.zeros((0, 4, 2)), np.zeros(0, bool))
    bad_tilt = good & ~ok
    if bad_tilt.any():
        excluded["horizon"] = int(bad_tilt.sum())
        out_notes.append(f"{excluded['horizon']} 張照片傾斜到角點視線 > 85°，不進度量")
    use = good & ok
    idx_use = np.nonzero(use)[0]
    cq = corners[idx_use]

    # ---- 格網 ----
    sx0, sy0, sx1, sy1 = survey.bounds
    margin = cell * 2
    if idx_use.size:
        ext = np.abs(cq - np.stack([px[idx_use], py[idx_use]], axis=-1)[:, None, :]).max()
        margin = min(float(ext) + cell * 2, 1000.0)
    gx0, gy0 = sx0 - margin, sy0 - margin
    nx = int(math.ceil((sx1 + margin - gx0) / cell))
    ny = int(math.ceil((sy1 + margin - gy0) / cell))
    if nx * ny > _MAX_CELLS:
        raise ValueError(f"格網 {nx}×{ny} = {nx * ny:,} 格超過上限 {_MAX_CELLS:,}；請把格距從 {cell:g} m 調大")
    gxs = gx0 + (np.arange(nx) + 0.5) * cell
    gys = gy0 + (np.arange(ny) + 0.5) * cell
    shapely.prepare(survey)
    inside = shapely.contains_xy(survey, *np.meshgrid(gxs, gys))

    uavs = sorted({photos[i].uav for i in idx_use})
    uav_of = np.array([photos[i].uav for i in idx_use], dtype=object)
    quads_uv = np.empty_like(cq)
    quads_uv[..., 0] = (cq[..., 0] - gx0) / cell - 0.5
    quads_uv[..., 1] = (cq[..., 1] - gy0) / cell - 0.5
    views = np.zeros((ny, nx), dtype=np.int32)
    by_uav: dict[str, np.ndarray] = {}
    for u in uavs:
        vu = _raster_views(quads_uv[uav_of == u], nx, ny)
        by_uav[u] = vu
        views += vu

    vin = views[inside]
    total = int(vin.size)
    hist = {int(k): int(c) for k, c in enumerate(np.bincount(vin)) if c} if total else {}
    frac_ge = {k: (float((vin >= k).mean()) if total else 0.0) for k in ks_t}

    # ---- 前向重疊 ----
    polys = shapely.polygons(cq) if idx_use.size else np.zeros(0, dtype=object)
    areas = shapely.area(polys) if idx_use.size else np.zeros(0)
    strips: dict[tuple[str, str], list[int]] = {}
    for j, i in enumerate(idx_use):
        sk = _strip_key(photos[i].strip)
        if sk is not None:
            strips.setdefault((photos[i].uav, sk), []).append(j)
    for key in strips:
        strips[key].sort(key=lambda j: photos[idx_use[j]].seq)
    fa, fb = [], []
    for js in strips.values():
        fa.extend(js[:-1])
        fb.extend(js[1:])
    if fa:
        fa_, fb_ = np.array(fa), np.array(fb)
        inter = shapely.area(shapely.intersection(polys[fa_], polys[fb_]))
        fwd = inter / np.maximum(areas[fa_], areas[fb_])
    else:
        fwd = np.zeros(0)

    # ---- 側向重疊（同一架、平行相鄰條帶）----
    cxy = np.stack([px[idx_use], py[idx_use]], axis=-1) if idx_use.size else np.zeros((0, 2))
    wid = 2 * tw[idx_use] * agl[idx_use]                    # 各張橫向足跡寬（m）
    info = {}
    for key, js in strips.items():
        if len(js) < 2:
            continue
        c = cxy[js]
        dvec = c[-1] - c[0]
        L = float(np.hypot(*dvec))
        if L < 1e-6:
            continue
        uvec = dvec / L
        info[key] = (c[0], uvec, np.array([-uvec[1], uvec[0]]), js, float(np.median(wid[js])))
    side_vals: list[float] = []
    band_cache: dict[tuple[str, str], Any] = {}
    cos_tol = math.cos(math.radians(parallel_tol_deg))
    for ka, (a0, ua, na, ja, wa) in info.items():
        along_a = (cxy[ja] - a0) @ ua
        nbrs: dict[int, tuple[float, tuple[str, str]]] = {}
        for kb, (_b0, ub, _nb, jb, _wb) in info.items():
            if kb == ka or kb[0] != ka[0] or abs(float(ua @ ub)) < cos_tol:
                continue
            off = float(np.median((cxy[jb] - a0) @ na))
            if abs(off) < 0.1 * wa or abs(off) >= wa:
                continue
            side = 1 if off > 0 else -1
            if side not in nbrs or abs(off) < abs(nbrs[side][0]):
                nbrs[side] = (off, kb)
        for _off, kb in nbrs.values():
            jb = info[kb][3]
            along_b = (cxy[jb] - a0) @ ua
            sel = [j for j, s in zip(ja, along_a) if along_b.min() <= s <= along_b.max()]
            if not sel:
                continue
            if kb not in band_cache:
                band_cache[kb] = shapely.union_all(polys[jb])
            sel_a = np.array(sel)
            inter = shapely.area(shapely.intersection(polys[sel_a], band_cache[kb]))
            side_vals.extend((inter / areas[sel_a]).tolist())

    # ---- 方向分視數（主條帶 vs 交叉航帶）----
    d_lab, d_method, d_axes = _classify_directions(photos, idx_use, strips, info, direction_of, float(parallel_tol_deg))
    directions = _direction_stats(d_lab, d_method, d_axes, quads_uv, uav_of, views, by_uav, inside, nx, ny, cell)

    # ---- 接縫（相鄰兩架）----
    # 只看「恰好只有這兩架」看到的格：第三架也看到的格表示第三架夾在中間（兩架靠它各自相接），
    # 那裡兩架足跡的偶然重疊不是接縫（評估 R2：B2/B3 之間夾著 A1，舊算法報 0.223）。
    # 寬度仍是這兩架之間的重疊寬（到「X 有 Y 沒有」＋到「Y 有 X 沒有」− 1 格，與第三架無關）。
    seams: list[SeamStats] = []
    not_adjacent: list[tuple[str, str, int]] = []
    if len(uavs) >= 2:
        masks = {u: by_uav[u] > 0 for u in uavs}
        n_cov = np.zeros((ny, nx), dtype=np.int16)
        for m_ in masks.values():
            n_cov += m_
        uav_w = {u: wid[uav_of == u] for u in uavs}
        hole = inside & (n_cov == 0)
        hole_nearest: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None

        def _hole_nearest_two() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
            """測區內沒有任何一架看到的格：最近的兩架（索引）與到兩架覆蓋的距離和（格）。"""
            hy, hx = np.nonzero(hole)
            dist = np.stack([ndimage.distance_transform_edt(~masks[u])[hy, hx] if masks[u].any()
                             else np.full(hy.size, np.inf) for u in uavs])
            order = np.argsort(dist, axis=0, kind="stable")[:2]
            dsum = np.take_along_axis(dist, order, axis=0).sum(axis=0)
            return order[0], order[1], dsum

        for ia, ua_ in enumerate(uavs):
            for ib in range(ia + 1, len(uavs)):
                ub_ = uavs[ib]
                ref = float(np.median(np.concatenate([uav_w[ua_], uav_w[ub_]])))
                both = inside & masks[ua_] & masks[ub_]
                band = both & (n_cov == 2)
                shared = int((both & (n_cov > 2)).sum())
                if band.any():
                    wc, fallback = _seam_widths(masks[ua_], masks[ub_], band, ref / cell)
                    wloc = wc * cell
                    if fallback:
                        out_notes.append(f"接縫 {ua_}/{ub_} 太短（整條都離覆蓋端點不到一個刈幅），寬度含端點效應、偏小")
                    seams.append(SeamStats(ua_, ub_, int(band.sum()), float(wloc.min()),
                                           float(np.percentile(wloc, 5)), float(np.median(wloc)),
                                           float(wloc.min() / ref), float(np.percentile(wloc, 5) / ref),
                                           float(np.median(wloc) / ref), ref, shared_cells=shared))
                    continue
                # 沒有只屬於這兩架的重疊：兩架之間隔著「沒人看到」的格（縫）且縫 < 參考刈幅，
                # 或兩架的「只有自己」區直接相接（零重疊、零縫）⇒ 相鄰而重疊為 0
                gaps: list[float] = []
                if hole.any():
                    if hole_nearest is None:
                        hole_nearest = _hole_nearest_two()
                    i1, i2, dsum = hole_nearest
                    sel = ((i1 == ia) & (i2 == ib)) | ((i1 == ib) & (i2 == ia))
                    if sel.any():
                        gaps.append((float(dsum[sel].min()) - 1.0) * cell)
                only_a = inside & masks[ua_] & (n_cov == 1)
                only_b = inside & masks[ub_] & (n_cov == 1)
                if only_a.any() and only_b.any() and (ndimage.binary_dilation(only_a) & only_b).any():
                    gaps.append(0.0)
                if gaps and min(gaps) < ref:
                    seams.append(SeamStats(ua_, ub_, 0, 0.0, None, None, 0.0, None, None, ref,
                                           gap_m=max(min(gaps), 0.0), shared_cells=shared))
                elif shared:
                    not_adjacent.append((ua_, ub_, shared))
    seam_min = min((s.frac_min for s in seams), default=None)
    eff["seam_pairs_not_adjacent"] = not_adjacent

    # ---- 邊緣帶（離測區邊界 ≤ edge_band_m）與內部分開統計 ----
    w_ref = float(np.median(wid)) if idx_use.size else None
    if edge_band_m is not None:
        eb: float | None = eb_arg
        eb_src = "argument"
    elif w_ref is not None:
        eb, eb_src = w_ref / 2.0, f"w/2（w = 照片橫向足跡寬中位數 {w_ref:.3f} m）"
    else:
        eb, eb_src = None, "沒有可用照片，w 無從定義 ⇒ 不分邊緣帶"
    eff.update({"edge_band_m": eb, "edge_band_source": eb_src})
    edge: np.ndarray | None = None
    edge_stats = interior_stats = None
    if eb is not None:
        edge = np.zeros_like(inside)
        if eb > 0:
            core_geom = survey.buffer(-eb)
            iy, ix = np.nonzero(inside)
            if core_geom.is_empty:
                edge[iy, ix] = True
            else:
                shapely.prepare(core_geom)
                edge[iy, ix] = ~shapely.contains_xy(core_geom, gxs[ix], gys[iy])

        def _region(mask: np.ndarray) -> RegionStats:
            v = views[mask]
            nc = int(v.size)
            und = int((v < uk).sum())
            return RegionStats(nc, nc * cell * cell, {k: (float((v >= k).mean()) if nc else 0.0) for k in ks_t},
                               und, und * cell * cell, (und / nc) if nc else 0.0, int(v.min()) if nc else None)
        edge_stats = _region(edge)
        interior_stats = _region(inside & ~edge)

    # ---- 視數不足區 → 多邊形 ----
    under: list[UnderCoveredArea] = []
    mask_u = inside & (views < uk)
    if mask_u.any():
        # 4-連通分量 → 每個分量一個多邊形（rasterio＝GDAL polygonize；比逐格聯集快 20 倍以上）
        from rasterio import features as rio_features
        from rasterio.transform import Affine
        lab, nlab = ndimage.label(mask_u)
        mins = ndimage.minimum(views, lab, index=np.arange(1, nlab + 1))
        cnts = np.bincount(lab.ravel(), minlength=nlab + 1)
        ecnts = np.bincount(lab[edge & mask_u], minlength=nlab + 1) if edge is not None else \
            np.zeros(nlab + 1, dtype=np.int64)
        tr = Affine(cell, 0.0, gx0, 0.0, cell, gy0)
        for geo, val in rio_features.shapes(lab.astype(np.int32), mask=mask_u, transform=tr, connectivity=4):
            li = int(val) - 1
            geom = shapely.geometry.shape(geo).intersection(survey)
            for part in getattr(geom, "geoms", [geom]):
                if part.is_empty or part.geom_type != "Polygon" or part.area <= 0:
                    continue
                ex, ey = np.asarray(part.exterior.coords).T
                elat, elon = frame.to_latlon(ex, ey)
                holes = []
                for h in part.interiors:
                    hx, hy = np.asarray(h.coords).T
                    hlat, hlon = frame.to_latlon(hx, hy)
                    holes.append(list(zip(hlat.tolist(), hlon.tolist())))
                c = part.centroid
                clat, clon = frame.to_latlon(c.x, c.y)
                in_edge = int(ecnts[li + 1])
                region = "edge_band" if edge is not None and in_edge == int(cnts[li + 1]) else "interior"
                under.append(UnderCoveredArea(float(part.area), int(cnts[li + 1]), int(mins[li]),
                                              (float(clat), float(clon)), list(zip(elat.tolist(), elon.tolist())),
                                              holes, part, region=region, edge_band_cells=in_edge))
        under.sort(key=lambda a: -a.area_m2)
    a_edge = float(sum(a.area_m2 for a in under if a.region == "edge_band"))
    a_int = float(sum(a.area_m2 for a in under if a.region == "interior"))
    if under:
        n_edge = sum(1 for a in under if a.region == "edge_band")
        if edge_stats is not None:
            msg = (f"視數 < {uk}：邊緣帶（離測區邊界 ≤ {eb:.1f} m）內 {n_edge} 處 {a_edge:,.0f} m²"
                   f"（邊緣帶視數不足格佔邊緣帶 {edge_stats.undercovered_frac:.2%}，最少 {edge_stats.min_views} 視）；"
                   f"伸入內部 {len(under) - n_edge} 處 {a_int:,.0f} m²（真破洞）")
            if edge_stats.min_views == 0:
                msg += "；邊緣帶內有視數 0 的格：完全沒拍到，不只是邊緣效應"
        else:
            msg = f"視數 < {uk}：{len(under)} 處 {a_int:,.0f} m²；{eb_src}"
        out_notes.append(msg)

    if not strips:
        out_notes.append("沒有任何照片帶條帶編號：前向／側向重疊無法計算")
    rep = ExposureReport(
        cell_size_m=cell, ks=ks_t, undercover_k=uk, total_cells=total, area_m2=float(survey.area),
        views_histogram=hist, frac_ge=frac_ge, forward_overlap=OverlapStats.of(fwd),
        side_overlap=OverlapStats.of(side_vals), seams=seams, seam_overlap_min=seam_min,
        photos_per_uav=per_uav_all, photos_used=int(idx_use.size), photos_excluded=excluded,
        undercovered=under, undercovered_area_m2=float(sum(a.area_m2 for a in under)), notes=out_notes,
        effective=eff, elapsed_s=0.0, frame_origin=(frame.lat0, frame.lon0), photos=photos,
        grid=ExposureGrid(frame, gx0, gy0, cell, views, inside, by_uav),
        edge_band_m=eb, edge_band=edge_stats, interior=interior_stats,
        undercovered_edge_band_area_m2=a_edge, undercovered_interior_area_m2=a_int, directions=directions)
    rep.elapsed_s = time.perf_counter() - t_start
    return rep


# ---------------------------------------------------------------------------
# 入口：飛前、飛後
# ---------------------------------------------------------------------------
def verify_preflight(missions: Mapping[str, Sequence[Any]], polygons: Any, camera: Any, *,
                     airspeed_mps: float | Mapping[str, float], wind: tuple[float, float],
                     terrain: Any = None, home_alt_msl: float | Mapping[str, float] | None = None,
                     intrval_min_s: float | Mapping[str, float] = 0.0,
                     trigger_duration_s: float | Mapping[str, float] = 0.1,
                     accept_radius: str | float = "mission",
                     airspeed_limits_mps: tuple[float, float] | Mapping[str, tuple[float, float]] | None = None,
                     cell_size_m: float = 2.0, ks: Sequence[int] = (5, 9),
                     undercover_k: int | None = None, edge_band_m: float | None = None,
                     direction_of: Any = None) -> ExposureReport:
    """飛前驗證：模擬每架的拍照點，再算曝光層級度量。

    missions：{uav: 任務項目}。每架參數可給單一值或 {uav: 值}；給 mapping 但缺某架 → ValueError。
    terrain=None 表示平地，地面 = 各架 home 高度；各架 home 高度差 > 0.5 m 時平地不成立 → ValueError。
    camera：FrameCamera、hv_dccpp Sensor，或規劃器的 CameraHardware（見 :func:`_footprint_model`）。
    edge_band_m：見 :func:`exposure_metrics`（None = w/2）。
    """
    t0 = time.perf_counter()
    sims: dict[str, SimulationResult] = {}
    for uav, items in missions.items():
        lim = airspeed_limits_mps
        if isinstance(lim, Mapping):
            lim = _per_uav(lim, uav, "airspeed_limits_mps")
        sims[uav] = simulate_mission_photos(
            items, uav=uav, airspeed_mps=_per_uav(airspeed_mps, uav, "airspeed_mps"), wind=wind, terrain=terrain,
            home_alt_msl=_per_uav(home_alt_msl, uav, "home_alt_msl"),
            intrval_min_s=_per_uav(intrval_min_s, uav, "intrval_min_s"),
            trigger_duration_s=_per_uav(trigger_duration_s, uav, "trigger_duration_s"),
            accept_radius=accept_radius, airspeed_limits_mps=lim)
    if terrain is None:
        homes = {u: s.effective.get("home_alt_msl") for u, s in sims.items()}
        vals = [v for v in homes.values() if v is not None]
        if not vals:
            raise ValueError("平地模式需要 home 高度")
        if max(vals) - min(vals) > 0.5:
            raise ValueError(f"沒給地形，但各架 home 高度不同 {homes}；平地假設不成立，請給 terrain")
        ground = float(np.mean(vals))
    else:
        ground = terrain
    photos = [p for s in sims.values() for p in s.photos]
    notes = list(_PREFLIGHT_ASSUMPTIONS) + [f"{u}: {m}" for u, s in sims.items() for m in s.notes]
    rep = exposure_metrics(photos, polygons, camera, terrain=ground, cell_size_m=cell_size_m, ks=ks,
                           undercover_k=undercover_k, tilt=False, notes=notes,
                           effective={"mode": "preflight", "per_uav": {u: s.effective for u, s in sims.items()}},
                           edge_band_m=edge_band_m, direction_of=direction_of)
    rep.simulations = sims
    rep.elapsed_s = time.perf_counter() - t0
    for m in rep.notes:
        _log.warning(f"[曝光驗證] {m}")
    return rep


def read_photos_csv(path: str | os.PathLike, *, height_field: str) -> tuple[list[PhotoPose], dict[str, Any]]:
    """讀 photos.csv（F2）。``height_field`` 必須明講用 ``h_orth_m`` 還是 ``h_ell_m``；
    選的欄位有空值 → ValueError（不拿 N=0 補）。回傳 (照片, 讀到的 crs_v／source 統計)。"""
    if height_field not in ("h_orth_m", "h_ell_m"):
        raise ValueError(f"height_field 要是 'h_orth_m' 或 'h_ell_m'，收到 {height_field!r}")
    photos: list[PhotoPose] = []
    crs_v: dict[str, int] = {}
    sources: dict[str, int] = {}
    empty = 0
    with open(path, newline="", encoding="utf-8") as fh:
        rd = csv.DictReader(fh)
        missing = [c for c in ("uav", "strip", "seq", "lat_deg", "lon_deg", height_field, "yaw_deg")
                   if c not in (rd.fieldnames or [])]
        if missing:
            raise ValueError(f"{path}：缺欄位 {missing}")

        def num(row, key, default=None):
            v = (row.get(key) or "").strip()
            return float(v) if v else default
        for row in rd:
            h = num(row, height_field)
            if h is None:
                empty += 1
                continue
            yaw = num(row, "yaw_deg")
            if yaw is None:
                raise ValueError(f"{path}：{row.get('image_id')!r} 沒有 yaw_deg")
            cv = (row.get("crs_v") or "").strip()
            crs_v[cv] = crs_v.get(cv, 0) + 1
            src = (row.get("source") or "").strip()
            sources[src] = sources.get(src, 0) + 1
            photos.append(PhotoPose(uav=row["uav"], strip=_strip_key(row["strip"]), seq=int(float(row["seq"])),
                                    lat_deg=float(row["lat_deg"]), lon_deg=float(row["lon_deg"]), h_m=h,
                                    yaw_deg=yaw, pitch_deg=num(row, "pitch_deg", 0.0),
                                    roll_deg=num(row, "roll_deg", 0.0), cog_deg=num(row, "cog_deg"),
                                    image_id=row.get("image_id") or ""))
    if empty:
        raise ValueError(f"{path}：{empty} 列的 {height_field} 是空的（沒有大地起伏模型就不會有 h_orth_m）；"
                         "不會以 N=0 代替，請改用 h_ell_m 並給橢球高地形")
    if height_field == "h_orth_m" and len([k for k in crs_v if k not in ("",)]) > 1:
        raise ValueError(f"{path}：h_orth_m 混了多個垂直基準 {crs_v}")
    return photos, {"height_field": height_field, "crs_v": crs_v, "source": sources}


def verify_postflight(photos: str | os.PathLike | Sequence[PhotoPose], polygons: Any, camera: Any, *,
                      terrain: Any, height_field: str | None = None, tilt: bool = True,
                      cell_size_m: float = 2.0, ks: Sequence[int] = (5, 9),
                      undercover_k: int | None = None, edge_band_m: float | None = None,
                      direction_of: Any = None) -> ExposureReport:
    """飛後驗證：photos.csv（F2，需 ``height_field``）或 PhotoPose 串列 → 曝光層級度量。

    terrain 必填，且與相機高度同一垂直基準（h_orth_m ↔ 正高 DEM；h_ell_m ↔ 橢球高）。
    edge_band_m：見 :func:`exposure_metrics`（None = w/2）。
    """
    t0 = time.perf_counter()
    eff: dict[str, Any] = {"mode": "postflight"}
    if isinstance(photos, (str, os.PathLike)):
        if height_field is None:
            raise ValueError("讀 photos.csv 要明講 height_field（'h_orth_m' 或 'h_ell_m'）")
        poses, meta = read_photos_csv(photos, height_field=height_field)
        eff.update(meta)
    else:
        poses = list(photos)
        eff["height_field"] = height_field or "PhotoPose.h_m"
    rep = exposure_metrics(poses, polygons, camera, terrain=terrain, cell_size_m=cell_size_m, ks=ks,
                           undercover_k=undercover_k, tilt=tilt, effective=eff, edge_band_m=edge_band_m,
                           direction_of=direction_of)
    rep.elapsed_s = time.perf_counter() - t0
    for m in rep.notes:
        _log.warning(f"[曝光驗證] {m}")
    return rep
