"""mqtt_bridge.py — HV-DCCPP ↔ 使用者 MQTT 系統之橋接層（整合筆記 §「HV-DCCPP 要提供的整合層」）。

本模組分兩層：

1. **純函式（不需 broker、不需 paho-mqtt）**
   * :func:`request_to_mission_inputs`：任務請求 JSON → ``(MissionInputs, PlannerConfig)``（無 DEM 時以
     :func:`scenario.synthetic_dem` 產生平地 DEM）。
   * :func:`plan_to_payloads`：``MissionPlan`` → 每架無人機一份回應 JSON（QGC WPL 110 文字 + 航點清單 + 摘要）。
   * :func:`handle_request` / :func:`handle_request_full`：請求 → 規劃 → 回應（後者另回傳 plan/ctx/狀態）。
   * :func:`legacy_to_request`：舊系統 ``{"waypoints":[{"lat","lon"}…]}``（恰 4 角點；本模組放寬為 ≥ 3）→ 新請求。
2. **:class:`MqttBridge`**（可選依賴 paho-mqtt，同時支援 v1 回呼 API 與 v2 ``CallbackAPIVersion``）：
   connect → subscribe → on_message → handle_request → publish。

座標約定：對外一律 WGS84 十進位度 ``(lat, lon)``、高度同時給 MSL 與相對 home（ArduPilot
``MAV_FRAME_GLOBAL_RELATIVE_ALT``）；內部 ENU 公尺（契約 §0）。角度為度、時間為秒。

主題（可於 :class:`MqttBridge` 建構時改寫）
-----------------------------------------
* 請求：``hvdccpp/mission/request``（:data:`TOPIC_REQUEST`）；舊系統相容主題 ``drone/waypoints``。
* 回應：``hvdccpp/mission/<mission_id>/<drone_id>``（:data:`TOPIC_RESPONSE_FMT`），每架一則。
* 狀態：``hvdccpp/status``（:data:`TOPIC_STATUS`）：``online`` / ``planning`` / ``planned`` / ``error`` 事件。

請求 JSON（topic ``hvdccpp/mission/request``）
--------------------------------------------
所有 ``?`` 標示者可省略；點一律 ``{"lat": …, "lon": …}``（亦接受 ``[lat, lon]`` 或 ``{"latitude","longitude"}``）。
省略之關鍵欄位由 :class:`BridgeDefaults` 補值並記入回應之 ``summary.assumptions``／狀態之 ``assumptions``。

.. code-block:: json

   {
     "mission_id": "m001",
     "home": {"lat": 23.7027, "lon": 120.4193, "alt_msl"?: 100.0, "id"?: "H",
              "clear_radius_m"?: 8, "approach_heading_deg"?: 270},
     "alternates"?: [{"id": "ALT1", "lat": …, "lon": …, "alt_msl"?: …}],
     "areas": [{"area_id": "Z1", "polygon": [{"lat","lon"}, … ≥3], "holes"?: [[{"lat","lon"}…]],
                "sensor_req"?: "lidar"|"rgb"|"both"|"any", "priority"?: 1.0,
                "time_window_s"?: [0, 7200], "payload_req_id"?: "default"}],
     "drones": [{"id": "A1", "platform": "pioneer_pnp"|"hero_pnp", "auw_kg"?: 24.6, "payload_kg"?: 5.0,
                 "available_wh"?: 1350, "nominal_wh"?: 1500, "soc"?: 0.9, "battery_temp_c"?: 25,
                 "sensors"?: ["lidar", "rgb"], "lost_link_policy"?: "RTB"|"CONTINUE_THEN_RTB"|"NEAREST_ALT"}],
     "wind"?: {"speed_mps": 5, "dir_from_deg": 270, "gust_mps"?: 7}
              | [{"z_bottom_m", "z_top_m", "speed_mps", "dir_from_deg", "gust_mps"?, "z_ref"?: "agl"|"msl"}…],
     "weather"?: {"mean_wind_mps", "gust_mps", "wind_dir_from_deg", "visibility_m", "precip_mmh", "temp_c"},
     "c2_range_km"?: 25 | {"A": 25, "B": 30},
     "c2"?: {"range_km"?, "coverage_polygon"?: [{"lat","lon"}…], "lost_link_policy"?: {"A1": "RTB"}, "t_ll_s"?: 60},
     "terrain_clearance_m"?: 36,
     "airspace"?: {"max_agl_m"?: 120, "waiver_max_agl_m"?: 200, "max_msl_m"?: …, "time_window_s"?: [0, 7200],
                   "no_fly"?: [{"zone_id", "polygon": [{"lat","lon"}…], "z_floor_msl_m"?, "z_ceiling_msl_m"?,
                                "active_window_s"?}]},
     "obstacles"?: [{"obs_id", "geom_type": "point"|"line"|"polygon", "coords": [{"lat","lon"}…],
                     "height_agl_m" | "top_msl_m", "category"?: "tower"|"building"|"tree"|"powerline"|"other",
                     "keepout_buffer_m"?}],
     "dem"?: {"lat0_deg", "lon0_deg", "dlat_deg", "dlon_deg", "z_msl": [[…]], "sigma_z_m"?, "is_dsm"?}
             | {"flat_msl_m"?: 100, "step_m"?: 50, "margin_m"?: 4000, "sigma_z_m"?: 2},
     "payload_reqs"?: {"default": {"kind": "lidar", "density_pt_m2", "side_overlap", "h_target_agl_m", …},
                       "default_rgb": {"kind": "rgb", "gsd_required_m", …}},
     "planner"?: {"dem_step_m": 50, "d_sep_m": 100, …任何 PlannerConfig 欄位},
     "platform_overrides"?: {"A": {"c2_link_range_km": 30}},
     "recovery_site_id"?: "ALT1", "t0_epoch_s"?: 0
   }

舊系統相容（legacy）：``{"waypoints": [{"lat","lon"}, …]}``（≥ 3 點、原系統恰 4 點）、``{"polygon": [...]}``
單區簡寫、分段訊息 ``{"waypoint": {...}, "index": i, "total": n}``（由 :class:`MqttBridge` 組裝）與單點
``{"lat","lon"}``（累積 ``legacy_n_points`` 點）皆視為一個 ``sensor_req="any"`` 區域，機隊採建構子之
``default_drones``，home 採 ``default_home``（無則自動置於區域西側 ``home_offset_m`` 處，記 ``home_auto_placed``）。

回應 JSON（topic ``hvdccpp/mission/<mission_id>/<drone_id>``）
------------------------------------------------------------
.. code-block:: json

   {
     "mission_id": "m001", "drone_id": "A1", "platform": "pioneer_pnp", "cls": "A",
     "qgc_wpl": "QGC WPL 110\\n0\\t1\\t0\\t16\\t…",
     "waypoints": [{"seq": 0, "lat": …, "lon": …, "alt_msl": …, "alt_rel": …, "alt_agl": …, "t_s": …,
                    "phase": "OPERATE", "mode": "CRUISE", "sensor_on": true, "strip_id": "Z1_1_A_1"|null,
                    "dir": -1|1|null, "airspeed_mps": …, "ground_speed_mps": …, "track_deg": …,
                    "energy_remaining_wh": …, "accept_radius_m": 30,
                    "arc": null | {"lat": …, "lon": …, "radius_m": …, "dir": 1}}],
     "summary": {"status": "GO"|"GO_WITH_WARNINGS"|"NO_GO"|"NOT_USED"|"ERROR", "makespan_s": …,
                 "t_takeoff_s", "t_arr_ptc_s", "t_land_s", "landing_site", "energy_used_wh", "energy_margin_wh",
                 "distance_m", "cruise_equiv_km", "n_waypoints", "n_qgc_items", "strips": [["Z1_1_A_1", -1], …],
                 "warnings": [...], "errors": [...], "assumptions": [...], "uncovered": [[id, reason], …],
                 "home": {"lat", "lon", "alt_msl"}, "alt_reference": "home_msl", "frame": "MAV_FRAME_GLOBAL_RELATIVE_ALT"},
     "rally": [{"id": "ALT1", "lat", "lon", "alt_rel"}],
     "geofence": {"inclusion": [{"lat","lon"}…], "exclusion": [[{"lat","lon"}…]], "alt_max_rel_m": …}
   }

``qgc_wpl`` 為 Mission Planner／QGC 可直接開啟之 ``.waypoints`` 文字（QGC WPL 110、tab 分隔 12 欄；第 0 列 home：
frame 0、cmd 16、current 1）。指令：NAV_VTOL_TAKEOFF(84) → DO_CHANGE_SPEED(178) → NAV_WAYPOINT(16)…，作業條帶以
DO_SET_CAM_TRIGG_DIST(206)（RGB）／DO_SET_RELAY(181)（LiDAR，繼電器號 :attr:`QgcOptions.lidar_relay`，依酬載線路待驗證）
包住；去衝突／起降盤旋（phase ORBIT 且帶 arc）→ NAV_LOITER_TIME(19)（param3 帶號半徑：ArduPlane 的 `AP_Mission`
**只**讀 ``p1`` = 盤旋秒數與 ``location.loiter_ccw = (param3 < 0)`` 之轉向，半徑大小被丟棄，實際以 ``WP_LOITER_RAD``
飛；故 param3 僅供 QGC／人工判讀轉向與規劃半徑。本橋接不輸出 .param，若下游要飛這段盤旋，須自行把
``WP_LOITER_RAD`` 設成規劃半徑，可由 :func:`hv_dccpp.ardupilot.orbit_radius_m` 取得）；降落序列
DO_LAND_START(189) → NAV_WAYPOINT（DECEL／TRANS_IN）→ NAV_VTOL_LAND(85)。MAV_CMD 常數取自 pymavlink
（``MAVLINK20=1``）；pymavlink 不存在時退回同值之內建表（:data:`_MAV_FALLBACK`）。

狀態 JSON（topic ``hvdccpp/status``）
------------------------------------
``{"event": "online"|"planning"|"planned"|"error", "mission_id"?, "status"?, "errors", "warnings", "assumptions",
"makespan_s"?, "drones"?: {"A1": "GO", …}, "uncovered"?, "timing_ms"?, "elapsed_s"?, "ts": epoch}``。
"""
from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
import uuid
from dataclasses import dataclass, field, fields
from typing import Any, Callable, Mapping, Sequence

import numpy as np

from ..geo import DemGrid, GeoFrame
from ..mission import (
    Airspace,
    AreaSpec,
    C2Link,
    MissionInputError,
    MissionInputs,
    MissionPlan,
    Mode,
    NoFlyZone,
    Obstacle,
    PayloadRequirement,
    Phase,
    PlannerConfig,
    Site,
    SortieEnergy,
    UAVInstance,
    UAVPlan,
    Waypoint,
    WeatherObs,
    WeatherThresholds,
    WindLayer,
    load_mission,
)
from ..platform import PLATFORM_HERO, PLATFORM_PIONEER, CameraHardware, LidarHardware, PlatformSpec
from ..scenario import metres_per_degree, synthetic_dem

__all__ = [
    "TOPIC_REQUEST", "TOPIC_STATUS", "TOPIC_RESPONSE_FMT", "LEGACY_TOPICS", "DEFAULT_DRONES", "PLATFORM_KEYS",
    "BridgeDefaults", "BridgeResult", "QgcOptions",
    "request_to_mission_inputs", "plan_to_payloads", "handle_request", "handle_request_full",
    "legacy_to_request", "is_legacy_payload", "status_payload", "qgc_wpl_from_plan", "qgc_items_from_plan",
    "MqttBridge",
]

_log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# MAVLink 常數（優先 pymavlink；否則同值退回表）
# ---------------------------------------------------------------------------

os.environ.setdefault("MAVLINK20", "1")


class _MavFallback:
    """pymavlink 缺席時之最小常數表（值 = MAVLink common.xml；只用於 QGC 文字輸出）。"""
    MAV_CMD_NAV_WAYPOINT = 16
    MAV_CMD_NAV_LOITER_TURNS = 18
    MAV_CMD_NAV_LOITER_TIME = 19
    MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
    MAV_CMD_NAV_LOITER_TO_ALT = 31
    MAV_CMD_NAV_VTOL_TAKEOFF = 84
    MAV_CMD_NAV_VTOL_LAND = 85
    MAV_CMD_DO_CHANGE_SPEED = 178
    MAV_CMD_DO_SET_RELAY = 181
    MAV_CMD_DO_LAND_START = 189
    MAV_CMD_DO_SET_CAM_TRIGG_DIST = 206
    MAV_CMD_NAV_RALLY_POINT = 5100
    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001
    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = 5002
    MAV_FRAME_GLOBAL = 0
    MAV_FRAME_GLOBAL_RELATIVE_ALT = 3


_MAV_FALLBACK = _MavFallback()

try:  # pragma: no cover - 依環境而異
    from pymavlink import mavutil as _mavutil  # type: ignore[import-not-found]
    _MAV: Any = _mavutil.mavlink
    HAVE_PYMAVLINK = True
except Exception:  # noqa: BLE001
    _MAV = _MAV_FALLBACK
    HAVE_PYMAVLINK = False


# ---------------------------------------------------------------------------
# 常數／預設
# ---------------------------------------------------------------------------

TOPIC_REQUEST: str = "hvdccpp/mission/request"
TOPIC_STATUS: str = "hvdccpp/status"
TOPIC_RESPONSE_FMT: str = "hvdccpp/mission/{mission_id}/{drone_id}"
LEGACY_TOPICS: tuple[str, ...] = ("drone/waypoints",)

PLATFORM_KEYS: dict[str, PlatformSpec] = {
    "pioneer_pnp": PLATFORM_PIONEER, "pioneer": PLATFORM_PIONEER, "a": PLATFORM_PIONEER,
    "hero_pnp": PLATFORM_HERO, "hero": PLATFORM_HERO, "b": PLATFORM_HERO,
}
"""``drones[i].platform`` 之鍵（不分大小寫；``Pioneer PNP``／``HERO PNP`` 亦可）。"""

PLATFORM_KEY_BY_CLS: dict[str, str] = {"A": "pioneer_pnp", "B": "hero_pnp"}

DEFAULT_DRONES: tuple[dict[str, Any], ...] = (
    {"id": "A1", "platform": "pioneer_pnp"},
    {"id": "B1", "platform": "hero_pnp"},
)
"""legacy payload 之預設機隊（1 Pioneer + 1 HERO）。"""

_DEFAULT_SENSORS_BY_CLS: dict[str, tuple[str, ...]] = {"A": ("lidar",), "B": ("rgb",)}
_PHASE_LAND_SEQ: frozenset[Phase] = frozenset({Phase.DECEL, Phase.TRANS_IN, Phase.LAND})


@dataclass(frozen=True, slots=True)
class BridgeDefaults:
    """請求缺省欄位之補值（每次套用皆記入 ``assumptions``）。"""
    c2_range_km: float = 25.0
    terrain_clearance_m: float = 36.0
    max_agl_m: float = 120.0
    waiver_max_agl_m: float | None = 200.0
    flat_dem_msl_m: float | None = None          # None → home.alt_msl → 0.0
    dem_step_m: float = 50.0                     # 合成平地 DEM 節點距
    dem_margin_m: float = 4000.0                 # 合成 DEM 涵蓋 = 幾何外接矩形 ⊕ 此值
    dem_sigma_z_m: float = 2.0
    planner_dem_step_synthetic_m: float = 50.0   # 平地 DEM → TerrainGrid 步長（無地形細節可失）
    soc_fraction: float = 0.9                    # 未給電量 → soc 0.9 × 標稱
    wind_speed_mps: float = 0.0
    wind_dir_from_deg: float = 0.0
    wind_layer_top_agl_m: float = 3000.0
    visibility_m: float = 8000.0
    precip_mmh: float = 0.0
    temp_c: float = 25.0
    home_offset_m: float = 1500.0                # legacy 自動 home：區域西側距離
    alternate_offset_m: float = 1000.0           # 無 alternates → 自動備降點與 home 之距離（0 → 不自動建立）
    lidar_h_target_agl_m: float = 160.0
    rgb_h_target_agl_m: float = 100.0


@dataclass(frozen=True, slots=True)
class QgcOptions:
    """QGC WPL 110 輸出選項。"""
    frame: str = "relative"                      # "relative" → MAV_FRAME_GLOBAL_RELATIVE_ALT；"msl" → MAV_FRAME_GLOBAL
    speed_cmd: bool = True                       # 空速變化 > speed_tol 時插 DO_CHANGE_SPEED
    speed_tol_mps: float = 0.05
    cam_trigger: bool = True                     # RGB 條帶 DO_SET_CAM_TRIGG_DIST；LiDAR 條帶 DO_SET_RELAY
    lidar_relay: int | None = 0                  # None → LiDAR 條帶不輸出開關指令
    # 注意：本橋接用 DO_SET_RELAY(181)，`hv_dccpp.ardupilot` 用 DO_SET_SERVO(183) on LIDAR_SERVO_CHANNEL。
    # 兩者需要的機上參數不同（RELAY{n+1}_PIN vs SERVO{n}_FUNCTION=0）；全案應擇一慣例，見 CONTRACT_DEVIATIONS。
    default_trigger_dist_m: float = 25.0         # 無法由感測器推得時之觸發距
    orbit_as_loiter: bool = True                 # ORBIT+arc 航點 → NAV_LOITER_TIME
    land_start: bool = True                      # 降落序列前插 DO_LAND_START
    autocontinue: int = 1


@dataclass(slots=True)
class BridgeResult:
    """:func:`handle_request_full` 之完整結果。"""
    mission_id: str
    payloads: dict[str, dict[str, Any]]
    status: dict[str, Any]
    plan: MissionPlan | None = None
    ctx: Any = None
    inputs: MissionInputs | None = None
    cfg: PlannerConfig | None = None
    assumptions: list[str] = field(default_factory=list)
    elapsed_s: float = 0.0

    @property
    def ok(self) -> bool:
        return self.status.get("status") in ("GO", "GO_WITH_WARNINGS")


# ---------------------------------------------------------------------------
# 小工具：點／數值解析
# ---------------------------------------------------------------------------

def _num(v: Any, name: str) -> float:
    try:
        f = float(v)
    except (TypeError, ValueError) as e:
        raise MissionInputError(f"{name} 須為數值，得 {v!r}") from e
    if not math.isfinite(f):
        raise MissionInputError(f"{name} 須為有限數值，得 {v!r}")
    return f


def _opt(d: Mapping[str, Any], *keys: str, default: Any = None) -> Any:
    for k in keys:
        if k in d and d[k] is not None:
            return d[k]
    return default


def _point(p: Any, name: str) -> tuple[float, float, float | None]:
    """``{"lat","lon"[,"alt_msl"|"alt"]}`` / ``{"latitude","longitude"}`` / ``[lat, lon(, alt)]`` → (lat, lon, alt|None)。"""
    if isinstance(p, Mapping):
        lat = _opt(p, "lat", "latitude", "lat_deg")
        lon = _opt(p, "lon", "lng", "longitude", "lon_deg")
        alt = _opt(p, "alt_msl", "alt_msl_m", "alt", "altitude")
        if lat is None or lon is None:
            raise MissionInputError(f"{name}: 點須含 lat/lon，得 {dict(p)!r}")
    elif isinstance(p, Sequence) and not isinstance(p, (str, bytes)) and len(p) >= 2:
        lat, lon = p[0], p[1]
        alt = p[2] if len(p) >= 3 else None
    else:
        raise MissionInputError(f"{name}: 點須為 {{lat, lon}} 或 [lat, lon]，得 {p!r}")
    lat_f, lon_f = _num(lat, f"{name}.lat"), _num(lon, f"{name}.lon")
    if not (-90.0 <= lat_f <= 90.0 and -180.0 <= lon_f <= 360.0):
        raise MissionInputError(f"{name}: lat/lon 超出範圍 ({lat_f}, {lon_f})")
    return lat_f, lon_f, (None if alt is None else _num(alt, f"{name}.alt"))


def _ring(pts: Any, name: str, min_n: int = 3) -> tuple[tuple[float, float], ...]:
    if pts is None or isinstance(pts, (str, bytes, Mapping)):
        raise MissionInputError(f"{name} 須為點序列")
    ring = [(_point(p, f"{name}[{i}]")[0], _point(p, f"{name}[{i}]")[1]) for i, p in enumerate(list(pts))]
    if len(ring) >= 2 and ring[0] == ring[-1]:
        ring = ring[:-1]
    if len(ring) < min_n:
        raise MissionInputError(f"{name} 須至少 {min_n} 個頂點，得 {len(ring)}")
    return tuple(ring)


def _window(w: Any, name: str) -> tuple[float, float] | None:
    if w is None:
        return None
    if isinstance(w, Mapping):
        w = (_opt(w, "start", "t0", "from"), _opt(w, "end", "t1", "to"))
    try:
        a, b = float(w[0]), float(w[1])
    except (TypeError, ValueError, IndexError) as e:
        raise MissionInputError(f"{name} 須為 [t_start, t_end]") from e
    return (a, b)


def _platform_of(key: Any, name: str) -> PlatformSpec:
    k = "".join(ch for ch in str(key).lower() if ch.isalnum() or ch == "_")
    if k in PLATFORM_KEYS:
        return PLATFORM_KEYS[k]
    k2 = k.replace("_", "")
    for cand, spec in PLATFORM_KEYS.items():
        if cand.replace("_", "") == k2:
            return spec
    raise MissionInputError(f"{name}: 未知平台 {key!r}（可用：{sorted(PLATFORM_KEYS)}）")


def is_legacy_payload(payload: Mapping[str, Any]) -> bool:
    """舊系統格式：無 ``areas`` 而有 ``waypoints``／``polygon``／單點 ``lat,lon``／分段 ``waypoint``。"""
    if not isinstance(payload, Mapping) or "areas" in payload:
        return False
    return any(k in payload for k in ("waypoints", "polygon", "waypoint")) or ("lat" in payload and "lon" in payload)


def legacy_to_request(payload: Mapping[str, Any], *, default_drones: Sequence[Mapping[str, Any]] | None = None,
                      default_home: Mapping[str, Any] | None = None, mission_id: str | None = None,
                      defaults: BridgeDefaults | None = None, notes: list[str] | None = None) -> dict[str, Any]:
    """舊系統 payload → 本模組請求 JSON（單一 ``any`` 區、預設機隊、home 自動放置）。

    接受 ``{"waypoints": [pt…]}``、``{"polygon": [pt…]}``、``[pt, …]`` 清單；``FlightParameters``（altitude/angle/
    spacing/speed）鍵忽略並記 ``legacy_flight_params_ignored``。
    """
    dfl = defaults or BridgeDefaults()
    if isinstance(payload, Sequence) and not isinstance(payload, (str, bytes, Mapping)):
        payload = {"waypoints": list(payload)}
    if not isinstance(payload, Mapping):
        raise MissionInputError("legacy payload 須為 mapping 或點清單")
    pts = _opt(payload, "waypoints", "polygon", "corners", "points")
    if pts is None:
        raise MissionInputError("legacy payload 缺 waypoints/polygon")
    ring = _ring(pts, "waypoints")
    req: dict[str, Any] = {k: v for k, v in payload.items()
                           if k not in ("waypoints", "polygon", "corners", "points", "waypoint", "index", "total")}
    ignored = [k for k in ("altitude", "angle", "spacing", "speed", "yaw_speed", "safety_distance") if k in req]
    for k in ignored:
        req.pop(k)
    if ignored and notes is not None:
        notes.append(f"legacy_flight_params_ignored({','.join(ignored)})")
    req["mission_id"] = str(mission_id or req.get("mission_id") or f"legacy-{uuid.uuid4().hex[:8]}")
    req["areas"] = [{"area_id": str(req.pop("area_id", "Z1")),
                     "polygon": [{"lat": a, "lon": b} for a, b in ring],
                     "sensor_req": str(req.pop("sensor_req", "any"))}]
    if "drones" not in req:
        req["drones"] = [dict(d) for d in (default_drones or DEFAULT_DRONES)]
    home = _opt(req, "home", "takeoff")
    req.pop("takeoff", None)
    if home is None and default_home is not None:
        home = dict(default_home)
    if home is None:
        lat_c = float(np.mean([a for a, _ in ring]))
        lon_min = min(b for _, b in ring)
        _m_lat, m_lon = metres_per_degree(lat_c)
        home = {"lat": lat_c, "lon": lon_min - dfl.home_offset_m / m_lon, "id": "H"}
        if notes is not None:
            notes.append(f"home_auto_placed({dfl.home_offset_m:.0f}m west of area)")
    req["home"] = home
    return req


# ---------------------------------------------------------------------------
# 請求 → MissionInputs / PlannerConfig
# ---------------------------------------------------------------------------

def _parse_site(obj: Any, name: str, kind: str, fallback_id: str) -> Site:
    if isinstance(obj, Site):
        return obj
    if isinstance(obj, Sequence) and not isinstance(obj, (str, bytes)):
        lat, lon, alt = _point(obj, name)
        obj = {"lat": lat, "lon": lon}
        if alt is not None:
            obj["alt_msl"] = alt
    if not isinstance(obj, Mapping):
        raise MissionInputError(f"{name} 須為 {{lat, lon, alt_msl?}} 或 [lat, lon(, alt_msl)]")
    lat, lon, alt = _point(obj, name)
    kw: dict[str, Any] = {}
    if alt is not None:
        kw["alt_msl_m"] = alt
    if _opt(obj, "alt_agl_m", "alt_agl") is not None:
        kw["alt_agl_m"] = _num(_opt(obj, "alt_agl_m", "alt_agl"), f"{name}.alt_agl_m")
        kw.pop("alt_msl_m", None)
    if _opt(obj, "clear_radius_m") is not None:
        kw["clear_radius_m"] = _num(obj["clear_radius_m"], f"{name}.clear_radius_m")
    if _opt(obj, "approach_heading_deg") is not None:
        kw["approach_heading_deg"] = _num(obj["approach_heading_deg"], f"{name}.approach_heading_deg")
    sid = str(_opt(obj, "id", "site_id", "name", default=fallback_id))
    return Site(sid, lat, lon, kind=kind, **kw)


def _auto_alternate(home: Site, area: AreaSpec, dist_m: float, site_id: str = "ALT1") -> Site:
    """無備降點時之自動備降點：自 home 朝區域質心方向 0.6·d̂ + 側向 0.8·n̂（左側）共 ``dist_m``（平地假設）。"""
    m_lat, m_lon = metres_per_degree(home.lat_deg)
    c_lat = float(np.mean([a for a, _ in area.polygon_wgs84]))
    c_lon = float(np.mean([b for _, b in area.polygon_wgs84]))
    dx, dy = (c_lon - home.lon_deg) * m_lon, (c_lat - home.lat_deg) * m_lat
    L = math.hypot(dx, dy)
    ux, uy = ((dx / L, dy / L) if L > 1e-6 else (1.0, 0.0))
    ox, oy = dist_m * (0.6 * ux - 0.8 * uy), dist_m * (0.6 * uy + 0.8 * ux)
    kw = {"alt_msl_m": home.alt_msl_m} if home.alt_msl_m is not None else {}
    return Site(site_id, home.lat_deg + oy / m_lat, home.lon_deg + ox / m_lon, kind="alternate", **kw)


def _parse_area(obj: Any, i: int) -> AreaSpec:
    if isinstance(obj, AreaSpec):
        return obj
    if not isinstance(obj, Mapping):
        raise MissionInputError(f"areas[{i}] 須為 mapping")
    aid = str(_opt(obj, "area_id", "id", "name", default=f"Z{i + 1}"))
    poly = _opt(obj, "polygon", "polygon_wgs84", "waypoints", "corners")
    if poly is None:
        raise MissionInputError(f"areas[{i}]({aid}) 缺 polygon")
    holes = _opt(obj, "holes", "holes_wgs84", default=())
    kw: dict[str, Any] = {
        "sensor_req": str(_opt(obj, "sensor_req", "sensor", default="any")).lower(),
        "priority": _num(_opt(obj, "priority", default=1.0), f"areas[{i}].priority"),
        "time_window_s": _window(_opt(obj, "time_window_s", "time_window"), f"areas[{i}].time_window_s"),
        "payload_req_id": str(_opt(obj, "payload_req_id", default="default")),
    }
    return AreaSpec(aid, _ring(poly, f"areas[{i}].polygon"),
                    holes_wgs84=tuple(_ring(h, f"areas[{i}].holes[{j}]") for j, h in enumerate(list(holes or ()))),
                    **kw)


def _parse_wind(obj: Any, dfl: BridgeDefaults, notes: list[str]) -> tuple[WindLayer, ...]:
    if obj is None:
        notes.append(f"wind_assumed({dfl.wind_speed_mps:.1f} m/s from {dfl.wind_dir_from_deg:.0f} deg)")
        return (WindLayer(0.0, dfl.wind_layer_top_agl_m, dfl.wind_speed_mps, dfl.wind_dir_from_deg, 0.0, "agl"),)
    if isinstance(obj, WindLayer):
        return (obj,)
    if isinstance(obj, Mapping):
        if "z_top_m" in obj or "z_bottom_m" in obj:
            obj = [obj]
        else:
            spd = _num(_opt(obj, "speed_mps", "speed", default=0.0), "wind.speed_mps")
            d = _num(_opt(obj, "dir_from_deg", "dir_from", "direction_deg", default=0.0), "wind.dir_from_deg")
            g = _num(_opt(obj, "gust_mps", "gust", default=spd), "wind.gust_mps")
            return (WindLayer(0.0, dfl.wind_layer_top_agl_m, spd, d, max(g, spd), "agl"),)
    out: list[WindLayer] = []
    for j, it in enumerate(list(obj)):
        if isinstance(it, WindLayer):
            out.append(it)
            continue
        if not isinstance(it, Mapping):
            raise MissionInputError(f"wind[{j}] 須為 mapping")
        out.append(WindLayer(
            _num(_opt(it, "z_bottom_m", default=0.0), f"wind[{j}].z_bottom_m"),
            float(_opt(it, "z_top_m", default=math.inf)) if _opt(it, "z_top_m") is not None else math.inf,
            _num(_opt(it, "speed_mps", "speed", default=0.0), f"wind[{j}].speed_mps"),
            _num(_opt(it, "dir_from_deg", "dir_from", default=0.0), f"wind[{j}].dir_from_deg"),
            _num(_opt(it, "gust_mps", "gust", default=0.0), f"wind[{j}].gust_mps"),
            str(_opt(it, "z_ref", default="agl")),
        ))
    return tuple(out)


def _parse_weather(obj: Any, wind: tuple[WindLayer, ...], dfl: BridgeDefaults, notes: list[str]) -> WeatherObs:
    if isinstance(obj, WeatherObs):
        return obj
    w0 = wind[0] if wind else None
    spd = w0.speed_mps if w0 else dfl.wind_speed_mps
    gust = max(w0.gust_mps, spd) if w0 else spd
    ddeg = w0.dir_from_deg if w0 else dfl.wind_dir_from_deg
    d = dict(obj) if isinstance(obj, Mapping) else {}
    if not isinstance(obj, Mapping):
        notes.append("weather_obs_assumed(from wind; visibility/precip/temp defaults)")
    return WeatherObs(
        mean_wind_mps=_num(_opt(d, "mean_wind_mps", "wind_mps", default=spd), "weather.mean_wind_mps"),
        gust_mps=_num(_opt(d, "gust_mps", default=gust), "weather.gust_mps"),
        wind_dir_from_deg=_num(_opt(d, "wind_dir_from_deg", "dir_from_deg", default=ddeg), "weather.wind_dir_from_deg"),
        visibility_m=_num(_opt(d, "visibility_m", default=dfl.visibility_m), "weather.visibility_m"),
        precip_mmh=_num(_opt(d, "precip_mmh", default=dfl.precip_mmh), "weather.precip_mmh"),
        temp_c=_num(_opt(d, "temp_c", default=dfl.temp_c), "weather.temp_c"),
        t_s=_num(_opt(d, "t_s", default=0.0), "weather.t_s"),
    )


def _parse_drone(obj: Any, i: int, dfl: BridgeDefaults, notes: list[str]) -> tuple[UAVInstance, str | None]:
    """→ (UAVInstance, lost_link_policy|None)。"""
    if isinstance(obj, UAVInstance):
        return obj, None
    if not isinstance(obj, Mapping):
        raise MissionInputError(f"drones[{i}] 須為 mapping")
    uid = str(_opt(obj, "id", "uav_id", "drone_id", "name", default=f"UAV{i + 1}"))
    spec = _platform_of(_opt(obj, "platform", "cls", "spec", default="pioneer_pnp"), f"drones[{i}]({uid}).platform")
    kinds = _opt(obj, "sensors", "sensor_kinds", default=_DEFAULT_SENSORS_BY_CLS[spec.cls])
    if isinstance(kinds, str):
        kinds = [kinds]
    hw: list[Any] = []
    for k in kinds:
        kk = str(k).lower()
        if kk == "lidar":
            hw.append(LidarHardware())
        elif kk in ("rgb", "camera"):
            hw.append(CameraHardware())
        else:
            raise MissionInputError(f"drones[{i}]({uid}).sensors: 未知種類 {k!r}（lidar/rgb）")
    auw = _num(_opt(obj, "auw_kg", default=spec.reference_auw_kg), f"drones[{i}].auw_kg")
    payload_kg = _opt(obj, "payload_kg")
    if payload_kg is None and "auw_kg" not in obj:
        payload_kg = spec.reference_payload_kg
    nominal = _opt(obj, "nominal_wh", "pack_nominal_wh", "battery_nominal_wh")
    avail = _opt(obj, "available_wh", "energy_wh")
    soc = _opt(obj, "soc", "soc_fraction")
    if avail is None and soc is None:
        soc = dfl.soc_fraction
        notes.append(f"energy_assumed({uid}: soc {dfl.soc_fraction:.2f} x nominal)")
    sortie = SortieEnergy(
        uid, auw,
        payload_kg=None if payload_kg is None else _num(payload_kg, f"drones[{i}].payload_kg"),
        available_wh=None if avail is None else _num(avail, f"drones[{i}].available_wh"),
        available_mah=None if _opt(obj, "available_mah") is None else _num(obj["available_mah"], f"drones[{i}].available_mah"),
        pack_voltage_nom_v=None if _opt(obj, "pack_voltage_nom_v") is None else _num(obj["pack_voltage_nom_v"], f"drones[{i}].pack_voltage_nom_v"),
        soc_fraction=None if soc is None else _num(soc, f"drones[{i}].soc"),
        pack_nominal_wh=None if nominal is None else _num(nominal, f"drones[{i}].nominal_wh"),
        battery_temp_c=None if _opt(obj, "battery_temp_c") is None else _num(obj["battery_temp_c"], f"drones[{i}].battery_temp_c"),
    )
    pol = _opt(obj, "lost_link_policy", "link_policy")
    return UAVInstance(uid, spec, tuple(hw), sortie), (None if pol is None else str(pol).upper())


def _parse_obstacle(obj: Any, i: int) -> Obstacle:
    if isinstance(obj, Obstacle):
        return obj
    if not isinstance(obj, Mapping):
        raise MissionInputError(f"obstacles[{i}] 須為 mapping")
    oid = str(_opt(obj, "obs_id", "id", "name", default=f"OBS{i + 1}"))
    coords = _opt(obj, "coords", "coords_wgs84", "polygon", "line", "points")
    pt = _opt(obj, "point")
    if coords is None and pt is not None:
        coords = [pt]
    if coords is None and "lat" in obj and "lon" in obj:
        coords = [obj]
    if coords is None:
        raise MissionInputError(f"obstacles[{i}]({oid}) 缺 coords")
    if isinstance(coords, Mapping):
        coords = [coords]
    pts = [(_point(p, f"obstacles[{i}].coords[{j}]")[0], _point(p, f"obstacles[{i}].coords[{j}]")[1])
           for j, p in enumerate(list(coords))]
    gt = _opt(obj, "geom_type", "type")
    if gt is None:
        gt = "point" if len(pts) == 1 else ("line" if len(pts) == 2 or "line" in obj else "polygon")
    gt = str(gt).lower()
    if gt == "polygon" and len(pts) >= 2 and pts[0] == pts[-1]:
        pts = pts[:-1]
    top = _opt(obj, "top_msl_m", "top_msl")
    hgt = _opt(obj, "height_agl_m", "height_m", "height")
    return Obstacle(oid, gt, tuple(pts),
                    top_msl_m=None if top is None else _num(top, f"obstacles[{i}].top_msl_m"),
                    height_agl_m=None if hgt is None else _num(hgt, f"obstacles[{i}].height_agl_m"),
                    category=str(_opt(obj, "category", default="other")).lower(),
                    keepout_buffer_m=None if _opt(obj, "keepout_buffer_m") is None
                    else _num(obj["keepout_buffer_m"], f"obstacles[{i}].keepout_buffer_m"))


def _parse_nofly(obj: Any, i: int) -> NoFlyZone:
    if isinstance(obj, NoFlyZone):
        return obj
    if not isinstance(obj, Mapping):
        raise MissionInputError(f"airspace.no_fly[{i}] 須為 mapping")
    zid = str(_opt(obj, "zone_id", "id", "name", default=f"NF{i + 1}"))
    poly = _opt(obj, "polygon", "polygon_wgs84", "coords")
    if poly is None:
        raise MissionInputError(f"airspace.no_fly[{i}]({zid}) 缺 polygon")
    fl, ce = _opt(obj, "z_floor_msl_m", "floor_msl_m"), _opt(obj, "z_ceiling_msl_m", "ceiling_msl_m")
    return NoFlyZone(zid, _ring(poly, f"airspace.no_fly[{i}].polygon"),
                     z_floor_msl_m=None if fl is None else _num(fl, f"no_fly[{i}].z_floor_msl_m"),
                     z_ceiling_msl_m=None if ce is None else _num(ce, f"no_fly[{i}].z_ceiling_msl_m"),
                     active_window_s=_window(_opt(obj, "active_window_s", "window"), f"no_fly[{i}].active_window_s"))


def _default_payload_reqs(dfl: BridgeDefaults) -> dict[str, PayloadRequirement]:
    return {
        "default": PayloadRequirement("default", "lidar", density_pt_m2=20.0, side_overlap=0.25,
                                      h_target_agl_m=dfl.lidar_h_target_agl_m),
        "default_rgb": PayloadRequirement("default_rgb", "rgb", gsd_required_m=0.05, side_overlap=0.30,
                                          forward_overlap=0.70, h_target_agl_m=dfl.rgb_h_target_agl_m),
    }


def _parse_payload_reqs(obj: Any, dfl: BridgeDefaults) -> dict[str, PayloadRequirement]:
    reqs = _default_payload_reqs(dfl)
    if obj is None:
        return reqs
    if not isinstance(obj, Mapping):
        raise MissionInputError("payload_reqs 須為 {req_id: {...}}")
    names = {f.name for f in fields(PayloadRequirement)}
    for rid, spec in obj.items():
        if isinstance(spec, PayloadRequirement):
            reqs[str(rid)] = spec
            continue
        if not isinstance(spec, Mapping):
            raise MissionInputError(f"payload_reqs[{rid}] 須為 mapping")
        d = {k: v for k, v in spec.items() if k in names and k != "req_id"}
        if "kind" not in d:
            raise MissionInputError(f"payload_reqs[{rid}] 缺 kind（lidar/rgb）")
        try:
            reqs[str(rid)] = PayloadRequirement(req_id=str(rid), **d)
        except TypeError as e:
            raise MissionInputError(f"payload_reqs[{rid}]: {e}") from e
    return reqs


def _all_points(home: Site, alternates: Sequence[Site], areas: Sequence[AreaSpec], obstacles: Sequence[Obstacle],
                nofly: Sequence[NoFlyZone], cov: Any) -> np.ndarray:
    pts: list[tuple[float, float]] = [home.latlon] + [s.latlon for s in alternates]
    for a in areas:
        pts.extend(a.polygon_wgs84)
        for h in a.holes_wgs84:
            pts.extend(h)
    for o in obstacles:
        pts.extend(o.coords_wgs84)
    for z in nofly:
        pts.extend(z.polygon_wgs84)
    if isinstance(cov, Mapping):
        for ring in cov.values():
            pts.extend(ring)
    elif cov:
        pts.extend(cov)
    return np.asarray(pts, dtype=np.float64).reshape(-1, 2)


def _synthetic_flat_dem(pts: np.ndarray, base_msl_m: float, step_m: float, margin_m: float,
                        sigma_z_m: float | None) -> DemGrid:
    lat_min, lon_min = float(pts[:, 0].min()), float(pts[:, 1].min())
    lat_max, lon_max = float(pts[:, 0].max()), float(pts[:, 1].max())
    m_lat, m_lon = metres_per_degree(0.5 * (lat_min + lat_max))
    lat0 = lat_min - margin_m / m_lat
    lon0 = lon_min - margin_m / m_lon
    size_km = (((lon_max - lon_min) * m_lon + 2.0 * margin_m) / 1000.0,
               ((lat_max - lat_min) * m_lat + 2.0 * margin_m) / 1000.0)
    return synthetic_dem(lat0, lon0, size_km, step_m, base_msl_m=base_msl_m, sigma_z_m=sigma_z_m)


def request_to_mission_inputs(payload: Mapping[str, Any], *, default_drones: Sequence[Mapping[str, Any]] | None = None,
                              default_home: Mapping[str, Any] | None = None, defaults: BridgeDefaults | None = None,
                              notes: list[str] | None = None) -> tuple[MissionInputs, PlannerConfig]:
    """請求 JSON（模組說明之 schema；legacy 格式自動經 :func:`legacy_to_request`）→ ``(MissionInputs, PlannerConfig)``。

    * 無 ``dem``（或 ``dem`` 只含 ``flat_msl_m/step_m/margin_m``）→ :func:`scenario.synthetic_dem` 平地 DEM，
      涵蓋全部幾何外接矩形 ⊕ ``defaults.dem_margin_m``；高程 = ``dem.flat_msl_m`` → ``home.alt_msl`` → 0。
    * 缺省之 c2 範圍／地形淨空／空域上限／風／氣象／電量皆依 :class:`BridgeDefaults` 補值並 append 至 ``notes``。
    * ``planner`` 區段以 :func:`dataclasses.replace` 套用於 PlannerConfig（平地 DEM 時 ``dem_step_m`` 預設 50 m）。
    * 輸入錯誤丟 :class:`MissionInputError`。
    """
    dfl = defaults or BridgeDefaults()
    nts: list[str] = notes if notes is not None else []
    if not isinstance(payload, Mapping):
        if isinstance(payload, Sequence) and not isinstance(payload, (str, bytes)):
            payload = legacy_to_request(payload, default_drones=default_drones, default_home=default_home,
                                        defaults=dfl, notes=nts)
        else:
            raise MissionInputError("請求須為 JSON 物件")
    if is_legacy_payload(payload):
        payload = legacy_to_request(payload, default_drones=default_drones, default_home=default_home,
                                    defaults=dfl, notes=nts)
        nts.append("legacy_payload")
    p = dict(payload)

    # ---- 站點 ---------------------------------------------------------------
    home_obj = _opt(p, "home", "takeoff")
    if home_obj is None:
        if default_home is None:
            raise MissionInputError("請求缺 home（{lat, lon, alt_msl?}）")
        home_obj = dict(default_home)
    home = _parse_site(home_obj, "home", "home", "H")
    alts_obj = _opt(p, "alternates", "alternate_sites", "rally", default=())
    if isinstance(alts_obj, Mapping):
        alts_obj = [alts_obj]
    alternates = tuple(_parse_site(s, f"alternates[{i}]", "alternate", f"ALT{i + 1}") for i, s in enumerate(list(alts_obj)))

    # ---- 區域 ---------------------------------------------------------------
    areas_obj = p.get("areas")
    if areas_obj is None:
        raise MissionInputError("請求缺 areas")
    if isinstance(areas_obj, Mapping):
        areas_obj = [dict(v, area_id=v.get("area_id", k)) if isinstance(v, Mapping) else v for k, v in areas_obj.items()]
    areas = tuple(_parse_area(a, i) for i, a in enumerate(list(areas_obj)))
    if not areas:
        raise MissionInputError("areas 不可為空")
    if not alternates and dfl.alternate_offset_m > 0.0:
        alternates = (_auto_alternate(home, areas[0], dfl.alternate_offset_m),)
        nts.append(f"alternate_auto_placed({alternates[0].site_id}: {dfl.alternate_offset_m:.0f} m from home; "
                   f"spec alternate_landing_sites_required)")

    # ---- 機隊 ---------------------------------------------------------------
    drones_obj = _opt(p, "drones", "fleet", "uavs")
    if drones_obj is None:
        drones_obj = [dict(d) for d in (default_drones or DEFAULT_DRONES)]
        nts.append("default_drones")
    if isinstance(drones_obj, Mapping):
        drones_obj = [dict(v, id=v.get("id", k)) if isinstance(v, Mapping) else v for k, v in drones_obj.items()]
    fleet: list[UAVInstance] = []
    policies: dict[str, str] = {}
    for i, d in enumerate(list(drones_obj)):
        u, pol = _parse_drone(d, i, dfl, nts)
        fleet.append(u)
        if pol is not None:
            policies[u.uav_id] = pol
    if not fleet:
        raise MissionInputError("drones 不可為空")

    # ---- 風／氣象 -----------------------------------------------------------
    wind = _parse_wind(p.get("wind"), dfl, nts)
    weather = _parse_weather(p.get("weather", p.get("weather_obs")), wind, dfl, nts)
    th_obj = p.get("weather_thresholds")
    thresholds = th_obj if isinstance(th_obj, WeatherThresholds) else (
        WeatherThresholds() if th_obj is None else WeatherThresholds(**dict(th_obj)))

    # ---- C2 -----------------------------------------------------------------
    c2_obj = p.get("c2") if isinstance(p.get("c2"), Mapping) else {}
    rng = _opt(p, "c2_range_km", default=_opt(c2_obj, "range_km"))
    if rng is None:
        rng = dfl.c2_range_km
        nts.append(f"c2_range_assumed({dfl.c2_range_km:.0f} km)")
    if isinstance(rng, Mapping):
        rng = {str(k).upper(): _num(v, f"c2_range_km[{k}]") for k, v in rng.items()}
    else:
        rng = _num(rng, "c2_range_km")
    cov = _opt(c2_obj, "coverage_polygon", "coverage_polygon_wgs84")
    if isinstance(cov, Mapping) and not ("lat" in cov):
        cov = {str(k).upper(): _ring(v, f"c2.coverage_polygon[{k}]") for k, v in cov.items()}
    elif cov is not None:
        cov = _ring(cov, "c2.coverage_polygon")
    pol_map = dict(policies)
    for k, v in (c2_obj.get("lost_link_policy") or {}).items():
        pol_map[str(k)] = str(v).upper()
    gs_obj = _opt(c2_obj, "ground_station")
    sites_by_id = {home.site_id: home, **{s.site_id: s for s in alternates}}
    if gs_obj is None:
        gs = home
    elif isinstance(gs_obj, str):
        if gs_obj not in sites_by_id:
            raise MissionInputError(f"c2.ground_station: 未知站點 {gs_obj!r}")
        gs = sites_by_id[gs_obj]
    else:
        gs = _parse_site(gs_obj, "c2.ground_station", "alternate", "GS")
    c2 = C2Link(range_km=rng, coverage_polygon_wgs84=cov, ground_station=gs, lost_link_policy=pol_map,
                t_ll_s=_num(_opt(c2_obj, "t_ll_s", default=60.0), "c2.t_ll_s"))

    # ---- 空域 ---------------------------------------------------------------
    as_obj = p.get("airspace") if isinstance(p.get("airspace"), Mapping) else {}
    tc = _opt(p, "terrain_clearance_m", default=_opt(as_obj, "terrain_clearance_m"))
    if tc is None:
        tc = dfl.terrain_clearance_m
        nts.append(f"terrain_clearance_assumed({dfl.terrain_clearance_m:.0f} m)")
    max_agl = _opt(as_obj, "max_agl_m", "max_agl")
    max_msl = _opt(as_obj, "max_msl_m", "max_msl")
    waiver = _opt(as_obj, "waiver_max_agl_m", "waiver")
    if max_agl is None and max_msl is None:
        max_agl = dfl.max_agl_m
        if waiver is None and dfl.waiver_max_agl_m is not None:
            waiver = dfl.waiver_max_agl_m
            nts.append(f"airspace_assumed(max_agl {dfl.max_agl_m:.0f} m, waiver {dfl.waiver_max_agl_m:.0f} m)")
        else:
            nts.append(f"airspace_assumed(max_agl {dfl.max_agl_m:.0f} m)")
    nf_obj = _opt(as_obj, "no_fly", "no_fly_zones", "nofly", default=p.get("no_fly", ()))
    if isinstance(nf_obj, Mapping):
        nf_obj = [nf_obj]
    nofly = tuple(_parse_nofly(z, i) for i, z in enumerate(list(nf_obj or ())))
    airspace = Airspace(
        max_agl_m=None if max_agl is None else _num(max_agl, "airspace.max_agl_m"),
        waiver_max_agl_m=None if waiver is None else _num(waiver, "airspace.waiver_max_agl_m"),
        max_msl_m=None if max_msl is None else _num(max_msl, "airspace.max_msl_m"),
        no_fly=nofly,
        time_window_s=_window(_opt(as_obj, "time_window_s", "time_window", default=p.get("time_window_s")),
                              "airspace.time_window_s"),
        terrain_clearance_m=_num(tc, "terrain_clearance_m"),
    )

    # ---- 障礙／酬載需求 -------------------------------------------------------
    obs_obj = p.get("obstacles") or ()
    if isinstance(obs_obj, Mapping):
        obs_obj = [obs_obj]
    obstacles = tuple(_parse_obstacle(o, i) for i, o in enumerate(list(obs_obj)))
    reqs = _parse_payload_reqs(p.get("payload_reqs"), dfl)

    # ---- DEM ----------------------------------------------------------------
    dem_obj = p.get("dem")
    synthetic = not (isinstance(dem_obj, DemGrid) or (isinstance(dem_obj, Mapping) and "z_msl" in dem_obj))
    if synthetic:
        d = dict(dem_obj) if isinstance(dem_obj, Mapping) else {}
        base = _opt(d, "flat_msl_m", "base_msl_m")
        if base is None:
            base = home.alt_msl_m if home.alt_msl_m is not None else (dfl.flat_dem_msl_m if dfl.flat_dem_msl_m is not None else 0.0)
        pts = _all_points(home, alternates, areas, obstacles, nofly, cov)
        dem: Any = _synthetic_flat_dem(
            pts, float(base), _num(_opt(d, "step_m", default=dfl.dem_step_m), "dem.step_m"),
            _num(_opt(d, "margin_m", default=dfl.dem_margin_m), "dem.margin_m"),
            None if _opt(d, "sigma_z_m", default=dfl.dem_sigma_z_m) is None else float(_opt(d, "sigma_z_m", default=dfl.dem_sigma_z_m)))
        nts.append(f"dem_synthetic_flat({float(base):.1f} m MSL)")
    else:
        dem = dem_obj

    # ---- 組裝（經 load_mission 統一驗證）-----------------------------------------
    mission_dict: dict[str, Any] = {
        "home": home, "alternates": list(alternates), "areas": list(areas), "dem": dem, "obstacles": list(obstacles),
        "wind": list(wind), "fleet": list(fleet), "payload_reqs": reqs, "c2": c2, "airspace": airspace,
        "weather_thresholds": thresholds, "weather_obs": weather,
        "t0_epoch_s": float(_opt(p, "t0_epoch_s", default=0.0) or 0.0),
        "platform_overrides": dict(p.get("platform_overrides") or {}),
        "recovery_site_id": p.get("recovery_site_id"),
    }
    inputs = load_mission(mission_dict)

    # ---- PlannerConfig --------------------------------------------------------
    cfg_kw: dict[str, Any] = {}
    if synthetic:
        cfg_kw["dem_step_m"] = dfl.planner_dem_step_synthetic_m
    planner_obj = p.get("planner") or p.get("planner_config") or {}
    if isinstance(planner_obj, PlannerConfig):
        cfg = planner_obj
    else:
        if not isinstance(planner_obj, Mapping):
            raise MissionInputError("planner 須為 {PlannerConfig 欄位: 值}")
        names = {f.name for f in fields(PlannerConfig)}
        bad = sorted(set(planner_obj) - names)
        if bad:
            raise MissionInputError(f"planner 未知參數：{bad}")
        cfg_kw.update({k: v for k, v in planner_obj.items()})
        try:
            cfg = PlannerConfig.from_dict(cfg_kw)
        except (TypeError, ValueError) as e:
            raise MissionInputError(f"planner: {e}") from e
    return inputs, cfg


# ---------------------------------------------------------------------------
# QGC WPL 110（私有序列化；ardupilot.py 尚未存在 → 記於 CONTRACT_DEVIATIONS）
# ---------------------------------------------------------------------------

@dataclass(slots=True)
class QgcItem:
    seq: int
    current: int
    frame: int
    command: int
    p1: float
    p2: float
    p3: float
    p4: float
    x: float
    y: float
    z: float
    autocontinue: int = 1

    def line(self) -> str:
        return (f"{self.seq}\t{self.current}\t{self.frame}\t{self.command}\t{self.p1:.6f}\t{self.p2:.6f}\t"
                f"{self.p3:.6f}\t{self.p4:.6f}\t{self.x:.8f}\t{self.y:.8f}\t{self.z:.6f}\t{self.autocontinue}")

    def to_dict(self) -> dict[str, Any]:
        return {f.name: getattr(self, f.name) for f in fields(self)}


def _trigger_dist_m(plan: MissionPlan, ctx: Any, strip_id: str | None, cls: str, opt: QgcOptions
                    ) -> tuple[str, float]:
    """(kind, 觸發距 m)：RGB → l(h_op)(1−o_fwd)；LiDAR → (lidar, 0)。"""
    if strip_id is None or ctx is None:
        return "rgb", opt.default_trigger_dist_m
    strip = plan.strips.get(strip_id) if isinstance(plan.strips, Mapping) else None
    if strip is None:
        return "rgb", opt.default_trigger_dist_m
    cell = next((c for c in plan.cells if c.cell_id == strip.cell_id), None)
    req_id = cell.payload_req_id if cell is not None else "default"
    sensors = getattr(ctx, "sensors", {}) or {}
    sensor = sensors.get((cls, req_id)) or sensors.get((cls, req_id + "_rgb"))
    if sensor is None:
        for (c, _r), s in sensors.items():
            if c == cls:
                sensor = s
                break
    if sensor is None:
        return "rgb", opt.default_trigger_dist_m
    kind = str(getattr(sensor, "kind", "rgb"))
    if kind == "lidar":
        return "lidar", 0.0
    try:
        h = float(strip.h_op_m)
        o_fwd = float(getattr(getattr(sensor, "r", None), "forward_overlap", 0.70))
        d = float(sensor.footprint_len_m(h)) * (1.0 - o_fwd)
        return "rgb", (d if d > 0.5 else opt.default_trigger_dist_m)
    except Exception:  # noqa: BLE001
        return "rgb", opt.default_trigger_dist_m


def qgc_items_from_plan(uplan: UAVPlan, spec: PlatformSpec, home_latlonalt: tuple[float, float, float],
                        frame: GeoFrame | None = None, *, plan: MissionPlan | None = None, ctx: Any = None,
                        opt: QgcOptions = QgcOptions()) -> list[QgcItem]:
    """UAVPlan → ArduPilot QuadPlane 任務項目（整合筆記 §1；MAV_CMD 常數取自 pymavlink）。

    * seq 0 = home（frame GLOBAL、cmd NAV_WAYPOINT、current 1）；seq 1 = NAV_VTOL_TAKEOFF（alt = ``transition_start_agl_m``）。
    * 每個非 TAKEOFF／LAND 航點 → NAV_WAYPOINT（p2 = accept_radius）；航點之 ``airspeed_mps``／``sensor_on`` 為其**出發航段**
      屬性，故 DO_CHANGE_SPEED／觸發指令緊接在該 NAV 項目之後。
    * ORBIT 且帶 ``arc`` → NAV_LOITER_TIME（p1 = 至下一航點之 Δt、p3 = ±R、位置 = 圓心）。
      **注意**：ArduPlane 的 `AP_Mission` 對 MAV_CMD_NAV_LOITER_TIME(19) 只儲存 p1（盤旋秒數）與
      ``location.loiter_ccw = (param3 < 0)``；**半徑大小被丟棄**，實際以 ``WP_LOITER_RAD`` 飛。此處寫入
      ±R 只是給 QGC／人工判讀轉向與規劃半徑用；要飛出規劃半徑必須另外把機上的 ``WP_LOITER_RAD``
      設成該值（本橋接不輸出 .param）。
    * 首個 DECEL（或 TRANS_IN）前插 DO_LAND_START；LAND 航點 → 單一 NAV_VTOL_LAND（alt 0）。
    """
    M = _MAV
    rel = opt.frame != "msl"
    frame_id = int(M.MAV_FRAME_GLOBAL_RELATIVE_ALT if rel else M.MAV_FRAME_GLOBAL)
    h_lat, h_lon, h_msl = home_latlonalt
    items: list[QgcItem] = []

    def add(cmd: int, p1: float = 0.0, p2: float = 0.0, p3: float = 0.0, p4: float = 0.0, x: float = 0.0,
            y: float = 0.0, z: float = 0.0, fr: int | None = None, current: int = 0) -> None:
        items.append(QgcItem(len(items), current, frame_id if fr is None else fr, int(cmd), float(p1), float(p2),
                             float(p3), float(p4), float(x), float(y), float(z), opt.autocontinue))

    def alt_of(w: Waypoint) -> float:
        return float(w.alt_msl_m) - h_msl if rel else float(w.alt_msl_m)

    add(M.MAV_CMD_NAV_WAYPOINT, x=h_lat, y=h_lon, z=h_msl, fr=int(M.MAV_FRAME_GLOBAL), current=1)
    wps = list(uplan.waypoints)
    if not wps:
        return items
    w0 = wps[0]
    add(M.MAV_CMD_NAV_VTOL_TAKEOFF, p4=0.0, x=w0.lat_deg, y=w0.lon_deg,
        z=(float(spec.transition_start_agl_m) if rel else float(w0.alt_msl_m) + float(spec.transition_start_agl_m)))
    speed_cur: float | None = None
    if opt.speed_cmd:
        v0 = next((float(w.airspeed_mps) for w in wps
                   if w.mode is Mode.CRUISE and math.isfinite(float(w.airspeed_mps)) and w.airspeed_mps > 0), None)
        if v0 is not None:
            add(M.MAV_CMD_DO_CHANGE_SPEED, p1=0.0, p2=v0, p3=-1.0)
            speed_cur = v0
    sensor_cur = False
    land_started = False
    landed = False
    takeoff_item = next(it for it in items if it.command == int(M.MAV_CMD_NAV_VTOL_TAKEOFF))
    last_nav: tuple[float, float, float] | None = (float(w0.lat_deg), float(w0.lon_deg), float(takeoff_item.z))
    n = len(wps)
    for j, w in enumerate(wps):
        if w.phase is Phase.TAKEOFF:
            continue
        if w.phase is Phase.LAND:
            if not landed:
                if opt.land_start and not land_started:
                    add(M.MAV_CMD_DO_LAND_START, x=w.lat_deg, y=w.lon_deg)
                    land_started = True
                add(M.MAV_CMD_NAV_VTOL_LAND, x=w.lat_deg, y=w.lon_deg, z=(0.0 if rel else float(w.alt_msl_m)))
                landed = True
            continue
        if opt.land_start and not land_started and w.phase in _PHASE_LAND_SEQ:
            add(M.MAV_CMD_DO_LAND_START, x=w.lat_deg, y=w.lon_deg)
            land_started = True
        if opt.orbit_as_loiter and w.phase is Phase.ORBIT and w.arc is not None and frame is not None:
            cx, cy, r, tdir = w.arc
            c = frame.to_wgs84(np.array([[float(cx), float(cy), float(w.z_m)]], dtype=np.float64))[0]
            dt = max(0.0, float(wps[j + 1].t_s) - float(w.t_s)) if j + 1 < n else 0.0
            add(M.MAV_CMD_NAV_LOITER_TIME, p1=dt, p3=float(r) * (1.0 if int(tdir) >= 0 else -1.0),
                x=float(c[0]), y=float(c[1]), z=alt_of(w))
            last_nav = (float(w.lat_deg), float(w.lon_deg), alt_of(w))
        else:
            cur = (float(w.lat_deg), float(w.lon_deg), alt_of(w))
            if last_nav is None or not _colocated(last_nav, cur):
                add(M.MAV_CMD_NAV_WAYPOINT, p2=float(w.accept_radius_m), x=w.lat_deg, y=w.lon_deg, z=alt_of(w))
                last_nav = cur
        if opt.speed_cmd and w.mode is Mode.CRUISE and math.isfinite(float(w.airspeed_mps)) and w.airspeed_mps > 0:
            if speed_cur is None or abs(float(w.airspeed_mps) - speed_cur) > opt.speed_tol_mps:
                add(M.MAV_CMD_DO_CHANGE_SPEED, p1=0.0, p2=float(w.airspeed_mps), p3=-1.0)
                speed_cur = float(w.airspeed_mps)
        if opt.cam_trigger and bool(w.sensor_on) != sensor_cur:
            kind, dist = _trigger_dist_m(plan, ctx, w.strip_id if w.sensor_on else _prev_strip(wps, j), uplan.cls, opt) \
                if plan is not None else ("rgb", opt.default_trigger_dist_m)
            if kind == "lidar":
                if opt.lidar_relay is not None:
                    add(M.MAV_CMD_DO_SET_RELAY, p1=float(opt.lidar_relay), p2=1.0 if w.sensor_on else 0.0)
            else:
                add(M.MAV_CMD_DO_SET_CAM_TRIGG_DIST, p1=(dist if w.sensor_on else 0.0), p2=0.0,
                    p3=(1.0 if w.sensor_on else 0.0))
            sensor_cur = bool(w.sensor_on)
    if not landed:
        wl = wps[-1]
        if opt.land_start and not land_started:
            add(M.MAV_CMD_DO_LAND_START, x=wl.lat_deg, y=wl.lon_deg)
        add(M.MAV_CMD_NAV_VTOL_LAND, x=wl.lat_deg, y=wl.lon_deg, z=(0.0 if rel else float(wl.alt_msl_m)))
    return items


def _colocated(a: tuple[float, float, float], b: tuple[float, float, float], tol_m: float = 1.0) -> bool:
    """兩 (lat, lon, alt) 水平距與高差皆 < tol_m（去除與起飛點重合之 NAV 項目）。"""
    m_lat, m_lon = metres_per_degree(a[0])
    d = math.hypot((b[0] - a[0]) * m_lat, (b[1] - a[1]) * m_lon)
    return d < tol_m and abs(b[2] - a[2]) < tol_m


def _prev_strip(wps: Sequence[Waypoint], j: int) -> str | None:
    for k in range(j - 1, -1, -1):
        if wps[k].strip_id is not None:
            return wps[k].strip_id
    return None


def qgc_wpl_from_items(items: Sequence[QgcItem]) -> str:
    return "QGC WPL 110\n" + "\n".join(it.line() for it in items) + "\n"


def qgc_wpl_from_plan(uplan: UAVPlan, spec: PlatformSpec, home_latlonalt: tuple[float, float, float],
                      frame: GeoFrame | None = None, *, plan: MissionPlan | None = None, ctx: Any = None,
                      opt: QgcOptions = QgcOptions()) -> str:
    """UAVPlan → QGC WPL 110 文字（Mission Planner ``.waypoints``）。"""
    return qgc_wpl_from_items(qgc_items_from_plan(uplan, spec, home_latlonalt, frame, plan=plan, ctx=ctx, opt=opt))


# ---------------------------------------------------------------------------
# MissionPlan → 回應 payload
# ---------------------------------------------------------------------------

def _f(v: Any) -> Any:
    """JSON 安全浮點（inf/nan → None）。"""
    try:
        x = float(v)
    except (TypeError, ValueError):
        return v
    return x if math.isfinite(x) else None


def _waypoint_json(seq: int, w: Waypoint, home_msl: float, frame: GeoFrame | None) -> dict[str, Any]:
    arc = None
    if w.arc is not None and frame is not None:
        cx, cy, r, tdir = w.arc
        c = frame.to_wgs84(np.array([[float(cx), float(cy), float(w.z_m)]], dtype=np.float64))[0]
        arc = {"lat": float(c[0]), "lon": float(c[1]), "radius_m": float(r), "dir": int(tdir)}
    return {
        "seq": int(seq), "lat": float(w.lat_deg), "lon": float(w.lon_deg),
        "alt_msl": float(w.alt_msl_m), "alt_rel": float(w.alt_msl_m) - home_msl, "alt_agl": _f(w.agl_m),
        "t_s": float(w.t_s), "phase": w.phase.name, "mode": w.mode.name, "sensor_on": bool(w.sensor_on),
        "strip_id": w.strip_id, "cell_id": w.cell_id, "dir": (None if w.dir is None else int(w.dir)),
        "airspeed_mps": _f(w.airspeed_mps), "ground_speed_mps": _f(w.ground_speed_mps),
        "track_deg": _f(w.track_deg), "heading_deg": _f(w.heading_deg),
        "energy_remaining_wh": _f(w.energy_remaining_wh), "accept_radius_m": _f(w.accept_radius_m),
        "nearest_alternate": w.nearest_alternate, "alt_reachable": bool(w.alt_reachable),
        "lost_link_action": w.lost_link_action, "arc": arc,
    }


def _home_of(ctx: Any, inputs: MissionInputs | None) -> tuple[float, float, float]:
    if ctx is not None:
        fr = ctx.frame
        return float(fr.lat0_deg), float(fr.lon0_deg), float(fr.h0_msl_m)
    if inputs is not None:
        h = inputs.home
        return float(h.lat_deg), float(h.lon_deg), float(h.alt_msl_m if h.alt_msl_m is not None else 0.0)
    return 0.0, 0.0, 0.0


def _rally_json(ctx: Any, inputs: MissionInputs, home_msl: float) -> list[dict[str, Any]]:
    out = []
    for s in inputs.alternates:
        alt_rel = 0.0
        if ctx is not None and s.site_id in ctx.sites_enu:
            xy, z = ctx.sites_enu[s.site_id]
            msl = float(ctx.frame.enu_z_to_msl(np.asarray(xy, dtype=np.float64).reshape(1, 2), float(z))[0])
            alt_rel = msl - home_msl
        out.append({"id": s.site_id, "lat": float(s.lat_deg), "lon": float(s.lon_deg), "alt_rel": alt_rel})
    return out


def _geofence_json(ctx: Any, inputs: MissionInputs, cls: str, buffer_m: float = 1000.0) -> dict[str, Any] | None:
    if ctx is None:
        return None
    try:
        from shapely.geometry import Point
        from shapely.ops import unary_union
        geoms = [g for g in ctx.areas_enu.values() if g is not None and not g.is_empty]
        geoms += [Point(float(xy[0]), float(xy[1])) for xy, _z in ctx.sites_enu.values()]
        hull = unary_union(geoms).convex_hull.buffer(float(buffer_m), quad_segs=4)
        ring = np.asarray(hull.exterior.coords, dtype=np.float64)[:-1]
        w = ctx.frame.to_wgs84(np.column_stack([ring, np.zeros(len(ring))]))
        incl = [{"lat": float(a), "lon": float(b)} for a, b in w[:, :2]]
        excl = [[{"lat": float(a), "lon": float(b)} for a, b in z.polygon_wgs84] for z in inputs.airspace.no_fly]
        h_ceil = ctx.h_ceil_m.get(cls) if isinstance(ctx.h_ceil_m, Mapping) else None
        return {"inclusion": incl, "exclusion": excl, "alt_max_rel_m": _f(h_ceil)}
    except Exception:  # noqa: BLE001
        return None


def _empty_payload(mission_id: str, uav: UAVInstance, status: str, home: tuple[float, float, float],
                   errors: Sequence[str], warnings: Sequence[str], assumptions: Sequence[str]) -> dict[str, Any]:
    return {
        "mission_id": mission_id, "drone_id": uav.uav_id, "platform": PLATFORM_KEY_BY_CLS.get(uav.cls, uav.cls),
        "cls": uav.cls, "qgc_wpl": "", "waypoints": [],
        "summary": {
            "status": status, "makespan_s": 0.0, "t_takeoff_s": None, "t_arr_ptc_s": None, "t_land_s": None,
            "landing_site": None, "energy_used_wh": 0.0, "energy_margin_wh": None, "distance_m": 0.0,
            "cruise_equiv_km": 0.0, "n_waypoints": 0, "n_qgc_items": 0, "strips": [],
            "warnings": list(warnings), "errors": list(errors), "assumptions": list(assumptions), "uncovered": [],
            "home": {"lat": home[0], "lon": home[1], "alt_msl": home[2]}, "alt_reference": "home_msl",
            "frame": "MAV_FRAME_GLOBAL_RELATIVE_ALT",
        },
        "rally": [], "geofence": None,
    }


def plan_to_payloads(plan: MissionPlan, ctx: Any, cfg: PlannerConfig, *, mission_id: str = "",
                     assumptions: Sequence[str] = (), qgc: QgcOptions = QgcOptions(),
                     inputs: MissionInputs | None = None) -> dict[str, dict[str, Any]]:
    """``MissionPlan`` → ``{drone_id: 回應 JSON}``（模組說明之回應 schema）。

    機隊中未飛之 UAV 亦有一則（``summary.status`` = ``NOT_USED``；NO_GO 時 = ``NO_GO`` 並附 errors）。
    ``alt_rel`` = ``alt_msl − home_msl``（home_msl = ``ctx.frame.h0_msl_m``）。
    """
    inp: MissionInputs = inputs if inputs is not None else ctx.inputs
    home = _home_of(ctx, inp)
    h_msl = home[2]
    rep = plan.feasibility
    fleet_status = "NO_GO" if rep.status == "NO_GO" else "NOT_USED"
    uncovered = [[str(a), str(b)] for a, b in plan.uncovered]
    out: dict[str, dict[str, Any]] = {}
    rally = _rally_json(ctx, inp, h_msl)
    for uav in inp.fleet:
        up = plan.plans.get(uav.uav_id)
        if up is None:
            out[uav.uav_id] = _empty_payload(mission_id, uav, fleet_status, home, rep.errors, rep.warnings, assumptions)
            out[uav.uav_id]["rally"] = rally
            out[uav.uav_id]["summary"]["uncovered"] = uncovered
            continue
        spec = ctx.specs.get(uav.cls, uav.spec) if ctx is not None and isinstance(getattr(ctx, "specs", None), Mapping) else uav.spec
        frame = ctx.frame if ctx is not None else None
        items = qgc_items_from_plan(up, spec, home, frame, plan=plan, ctx=ctx, opt=qgc)
        wps = [_waypoint_json(j, w, h_msl, frame) for j, w in enumerate(up.waypoints)]
        status = rep.status if rep.status in ("GO", "GO_WITH_WARNINGS") else "GO_WITH_WARNINGS"
        if up.warnings and status == "GO":
            status = "GO_WITH_WARNINGS"
        out[uav.uav_id] = {
            "mission_id": mission_id, "drone_id": uav.uav_id, "platform": PLATFORM_KEY_BY_CLS.get(uav.cls, uav.cls),
            "cls": uav.cls, "qgc_wpl": qgc_wpl_from_items(items), "waypoints": wps,
            "summary": {
                "status": status, "makespan_s": _f(plan.makespan_s),
                "t_takeoff_s": _f(up.t_takeoff_s), "t_arr_ptc_s": _f(up.t_arr_ptc_s), "t_land_s": _f(up.t_land_s),
                "landing_site": up.landing_site, "energy_used_wh": _f(up.energy_used_wh),
                "energy_margin_wh": _f(up.energy_margin_wh), "distance_m": _f(up.distance_m),
                "cruise_equiv_km": _f(up.cruise_equiv_km), "orbits_inserted": int(up.orbits_inserted),
                "n_waypoints": len(wps), "n_qgc_items": len(items),
                "strips": [[str(s), int(d)] for s, d in up.strips],
                "warnings": list(up.warnings) + [w for w in rep.warnings if w not in up.warnings],
                "errors": list(rep.errors), "assumptions": list(assumptions), "uncovered": uncovered,
                "home": {"lat": home[0], "lon": home[1], "alt_msl": h_msl}, "alt_reference": "home_msl",
                "frame": ("MAV_FRAME_GLOBAL_RELATIVE_ALT" if qgc.frame != "msl" else "MAV_FRAME_GLOBAL"),
                "cruise_airspeed_mps": float(spec.cruise_airspeed_mps),
                "transition_start_agl_m": float(spec.transition_start_agl_m),
            },
            "rally": rally,
            "geofence": _geofence_json(ctx, inp, uav.cls),
        }
    return out


def status_payload(mission_id: str, plan: MissionPlan | None, *, event: str = "planned", errors: Sequence[str] = (),
                   assumptions: Sequence[str] = (), payloads: Mapping[str, Mapping[str, Any]] | None = None,
                   elapsed_s: float = 0.0) -> dict[str, Any]:
    """``hvdccpp/status`` 訊息。"""
    d: dict[str, Any] = {"event": event, "mission_id": mission_id, "ts": time.time(), "elapsed_s": float(elapsed_s),
                         "errors": list(errors), "warnings": [], "assumptions": list(assumptions)}
    if plan is not None:
        rep = plan.feasibility
        d["status"] = rep.status
        d["errors"] = list(rep.errors) + [e for e in errors if e not in rep.errors]
        d["warnings"] = list(rep.warnings)
        d["makespan_s"] = _f(plan.makespan_s)
        d["uncovered"] = [[str(a), str(b)] for a, b in plan.uncovered]
        d["timing_ms"] = {k: _f(v) for k, v in plan.timing_ms.items()}
    else:
        d["status"] = "ERROR" if errors else "PENDING"
    if payloads:
        d["drones"] = {k: v.get("summary", {}).get("status") for k, v in payloads.items()}
        d["topics"] = {k: TOPIC_RESPONSE_FMT.format(mission_id=mission_id, drone_id=k) for k in payloads}
    return d


# ---------------------------------------------------------------------------
# 請求 → 規劃 → 回應
# ---------------------------------------------------------------------------

def handle_request_full(payload: Mapping[str, Any] | Sequence[Any] | str | bytes, *,
                        default_drones: Sequence[Mapping[str, Any]] | None = None,
                        default_home: Mapping[str, Any] | None = None, defaults: BridgeDefaults | None = None,
                        terrain_margin_m: float = 2000.0, qgc: QgcOptions = QgcOptions(),
                        raise_errors: bool = False) -> BridgeResult:
    """請求（dict／JSON 字串／bytes／legacy）→ :class:`BridgeResult`（每架 payload、狀態、plan、ctx）。

    輸入錯誤／規劃例外 → ``status.status == "ERROR"``（``raise_errors=True`` 則重新拋出）；preflight NO_GO →
    每架 payload ``summary.status == "NO_GO"`` 與 errors。
    """
    from ..context import build_context
    from ..planner import plan_mission

    t0 = time.perf_counter()
    if isinstance(payload, (bytes, bytearray)):
        payload = payload.decode("utf-8")
    if isinstance(payload, str):
        try:
            payload = json.loads(payload)
        except json.JSONDecodeError as e:
            if raise_errors:
                raise MissionInputError(f"JSON 解析失敗：{e}") from e
            mid = f"bad-{uuid.uuid4().hex[:8]}"
            return BridgeResult(mid, {}, status_payload(mid, None, event="error", errors=[f"json: {e}"]))
    mission_id = str(payload.get("mission_id") if isinstance(payload, Mapping) and payload.get("mission_id")
                     else f"m-{uuid.uuid4().hex[:8]}")
    notes: list[str] = []
    try:
        inputs, cfg = request_to_mission_inputs(payload, default_drones=default_drones, default_home=default_home,
                                                defaults=defaults, notes=notes)
    except MissionInputError as e:
        if raise_errors:
            raise
        st = status_payload(mission_id, None, event="error", errors=[f"input: {e}"], assumptions=notes)
        return BridgeResult(mission_id, {}, st, assumptions=notes, elapsed_s=time.perf_counter() - t0)
    home = _home_of(None, inputs)
    try:
        ctx = build_context(inputs, cfg, terrain_margin_m=float(terrain_margin_m))
    except MissionInputError as e:
        if raise_errors:
            raise
        errs = [f"build_context: {e}"]
        payloads = {u.uav_id: _empty_payload(mission_id, u, "NO_GO", home, errs, [], notes) for u in inputs.fleet}
        st = status_payload(mission_id, None, event="planned", errors=errs, assumptions=notes, payloads=payloads,
                            elapsed_s=time.perf_counter() - t0)
        st["status"] = "NO_GO"
        return BridgeResult(mission_id, payloads, st, None, None, inputs, cfg, notes, time.perf_counter() - t0)
    try:
        plan = plan_mission(inputs, cfg, ctx=ctx)
        payloads = plan_to_payloads(plan, ctx, cfg, mission_id=mission_id, assumptions=notes, qgc=qgc, inputs=inputs)
    except Exception as e:  # noqa: BLE001
        if raise_errors:
            raise
        _log.exception("plan failed for mission %s", mission_id)
        errs = [f"planner: {type(e).__name__}: {e}"]
        payloads = {u.uav_id: _empty_payload(mission_id, u, "ERROR", home, errs, [], notes) for u in inputs.fleet}
        st = status_payload(mission_id, None, event="error", errors=errs, assumptions=notes, payloads=payloads,
                            elapsed_s=time.perf_counter() - t0)
        return BridgeResult(mission_id, payloads, st, None, ctx, inputs, cfg, notes, time.perf_counter() - t0)
    dt = time.perf_counter() - t0
    st = status_payload(mission_id, plan, event="planned", assumptions=notes, payloads=payloads, elapsed_s=dt)
    return BridgeResult(mission_id, payloads, st, plan, ctx, inputs, cfg, notes, dt)


def handle_request(payload: Mapping[str, Any] | Sequence[Any] | str | bytes, *,
                   default_drones: Sequence[Mapping[str, Any]] | None = None,
                   default_home: Mapping[str, Any] | None = None, defaults: BridgeDefaults | None = None,
                   terrain_margin_m: float = 2000.0, qgc: QgcOptions = QgcOptions(),
                   raise_errors: bool = False) -> dict[str, dict[str, Any]]:
    """請求 → ``{drone_id: 回應 JSON}``（:func:`handle_request_full` 之 ``payloads``）。"""
    return handle_request_full(payload, default_drones=default_drones, default_home=default_home, defaults=defaults,
                               terrain_margin_m=terrain_margin_m, qgc=qgc, raise_errors=raise_errors).payloads


# ---------------------------------------------------------------------------
# MQTT 橋接（可選依賴 paho-mqtt v1／v2）
# ---------------------------------------------------------------------------

def _make_paho_client(client_id: str, userdata: Any = None) -> Any:
    """建立 paho Client：v2 → ``Client(CallbackAPIVersion.VERSION2, client_id)``；v1 → ``Client(client_id)``。"""
    try:
        import paho.mqtt.client as mqtt  # type: ignore[import-not-found]
    except Exception as e:  # noqa: BLE001
        raise ImportError("MqttBridge 需要 paho-mqtt（pip install paho-mqtt）") from e
    api = getattr(mqtt, "CallbackAPIVersion", None)
    if api is not None:
        return mqtt.Client(api.VERSION2, client_id=client_id, userdata=userdata)
    return mqtt.Client(client_id=client_id, userdata=userdata)


class MqttBridge:
    """MQTT 橋：訂閱請求主題 → :func:`handle_request_full` → 逐機發布回應與狀態。

    ``client`` 可注入任何具 ``connect/subscribe/publish/loop_*/disconnect`` 之物件（測試用假 client）；
    ``handler`` 可替換為任何 ``payload → BridgeResult`` 之函式。回呼簽章同時相容 paho v1 與 v2
    （``on_connect(client, userdata, flags, rc, properties=None)``；``on_message(client, userdata, msg)`` 兩版相同）。
    ``async_handling=True`` 時每則請求於獨立執行緒規劃（避免阻塞網路迴圈超過 keepalive）。
    """

    def __init__(self, host: str = "localhost", port: int = 1883, *, client_id: str = "hvdccpp-bridge", qos: int = 1,
                 keepalive_s: int = 60, default_drones: Sequence[Mapping[str, Any]] | None = None,
                 default_home: Mapping[str, Any] | None = None, defaults: BridgeDefaults | None = None,
                 topic_request: str = TOPIC_REQUEST, topic_status: str = TOPIC_STATUS,
                 topic_response_fmt: str = TOPIC_RESPONSE_FMT, legacy_topics: Sequence[str] = LEGACY_TOPICS,
                 legacy_n_points: int = 4, terrain_margin_m: float = 2000.0, qgc: QgcOptions = QgcOptions(),
                 handler: Callable[..., BridgeResult] | None = None, on_result: Callable[[BridgeResult], None] | None = None,
                 client: Any = None, async_handling: bool = False, username: str | None = None,
                 password: str | None = None, retain_response: bool = False) -> None:
        self.host, self.port, self.qos, self.keepalive_s = host, int(port), int(qos), int(keepalive_s)
        self.client_id = client_id
        self.default_drones = [dict(d) for d in (default_drones or DEFAULT_DRONES)]
        self.default_home = None if default_home is None else dict(default_home)
        self.defaults = defaults or BridgeDefaults()
        self.topic_request, self.topic_status, self.topic_response_fmt = topic_request, topic_status, topic_response_fmt
        self.legacy_topics = tuple(legacy_topics)
        self.legacy_n_points = int(legacy_n_points)
        self.terrain_margin_m = float(terrain_margin_m)
        self.qgc = qgc
        self.handler = handler
        self.on_result = on_result
        self.async_handling = bool(async_handling)
        self.retain_response = bool(retain_response)
        self._username, self._password = username, password
        self._legacy_chunks: dict[int, Any] = {}
        self._legacy_points: list[Any] = []
        self._lock = threading.Lock()
        self.results: list[BridgeResult] = []
        self.published: list[tuple[str, dict[str, Any]]] = []
        self.client = client if client is not None else _make_paho_client(client_id, userdata=self)
        self._wire_callbacks()

    # -- paho 接線 -------------------------------------------------------------
    def _wire_callbacks(self) -> None:
        c = self.client
        for name in ("on_connect", "on_message", "on_disconnect"):
            try:
                setattr(c, name, getattr(self, name))
            except Exception:  # noqa: BLE001
                pass
        if self._username is not None and hasattr(c, "username_pw_set"):
            c.username_pw_set(self._username, self._password)

    def connect(self) -> None:
        """連線（不阻塞）；訂閱在 ``on_connect`` 完成（重連時自動重訂閱）。"""
        self.client.connect(self.host, self.port, self.keepalive_s)

    def subscribe(self) -> None:
        for t in (self.topic_request, *self.legacy_topics):
            self.client.subscribe(t, self.qos)

    def loop_forever(self) -> None:
        self.client.loop_forever()

    def loop_start(self) -> None:
        self.client.loop_start()

    def loop_stop(self) -> None:
        if hasattr(self.client, "loop_stop"):
            self.client.loop_stop()

    def disconnect(self) -> None:
        self.client.disconnect()

    def publish(self, topic: str, payload: Mapping[str, Any], *, retain: bool | None = None) -> None:
        data = json.dumps(payload, ensure_ascii=False, allow_nan=False, default=_json_default)
        self.published.append((topic, dict(payload)))
        self.client.publish(topic, data, qos=self.qos, retain=(self.retain_response if retain is None else retain))

    # -- 回呼（v1／v2 相容）----------------------------------------------------
    def on_connect(self, client: Any, userdata: Any, flags: Any, rc: Any, properties: Any = None) -> None:
        self.subscribe()
        self.publish(self.topic_status, {"event": "online", "client_id": self.client_id, "ts": time.time(),
                                         "topics": {"request": self.topic_request, "legacy": list(self.legacy_topics),
                                                    "response": self.topic_response_fmt}}, retain=False)

    def on_disconnect(self, client: Any, userdata: Any, *args: Any) -> None:
        _log.info("mqtt disconnected: %s", args)

    def on_message(self, client: Any, userdata: Any, msg: Any) -> None:
        topic = getattr(msg, "topic", "")
        data = getattr(msg, "payload", b"")
        if self.async_handling:
            threading.Thread(target=self.handle_bytes, args=(topic, data), daemon=True).start()
        else:
            self.handle_bytes(topic, data)

    # -- 純處理（可離線測試）------------------------------------------------------
    def handle_bytes(self, topic: str, data: bytes | str) -> list[tuple[str, dict[str, Any]]]:
        """一則訊息 → 發布之 ``[(topic, payload), …]``（legacy 分段訊息未齊時回 []）。"""
        try:
            obj = json.loads(data.decode("utf-8") if isinstance(data, (bytes, bytearray)) else data)
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            st = {"event": "error", "errors": [f"json: {e}"], "topic": topic, "ts": time.time()}
            self.publish(self.topic_status, st, retain=False)
            return [(self.topic_status, st)]
        assembled = self._assemble_legacy(obj) if isinstance(obj, Mapping) and self._is_chunk(obj) else obj
        if assembled is None:
            return []
        return self.handle_payload(assembled)

    def _is_chunk(self, obj: Mapping[str, Any]) -> bool:
        if "areas" in obj or "waypoints" in obj or "polygon" in obj:
            return False
        return "waypoint" in obj or ("lat" in obj and "lon" in obj)

    def _assemble_legacy(self, obj: Mapping[str, Any]) -> dict[str, Any] | None:
        """``{"waypoint":{...},"index":i,"total":n}`` 或單點 ``{"lat","lon"}`` → 齊全時回 ``{"waypoints":[…]}``。"""
        with self._lock:
            if "waypoint" in obj:
                total = int(obj.get("total", 0) or 0)
                idx = int(obj.get("index", len(self._legacy_chunks)))
                self._legacy_chunks[idx] = obj["waypoint"]
                if total > 0 and len(self._legacy_chunks) >= total:
                    pts = [self._legacy_chunks[k] for k in sorted(self._legacy_chunks)]
                    self._legacy_chunks = {}
                    return {"waypoints": pts, "mission_id": obj.get("mission_id")}
                return None
            self._legacy_points.append({"lat": obj["lat"], "lon": obj["lon"]})
            if len(self._legacy_points) >= self.legacy_n_points:
                pts = list(self._legacy_points)
                self._legacy_points = []
                return {"waypoints": pts, "mission_id": obj.get("mission_id")}
            return None

    def handle_payload(self, obj: Any) -> list[tuple[str, dict[str, Any]]]:
        mid = str(obj.get("mission_id")) if isinstance(obj, Mapping) and obj.get("mission_id") else None
        if isinstance(obj, Mapping) and mid is None:
            obj = dict(obj)
            obj["mission_id"] = mid = f"m-{uuid.uuid4().hex[:8]}"
        self.publish(self.topic_status, {"event": "planning", "mission_id": mid, "ts": time.time()}, retain=False)
        if self.handler is not None:
            res = self.handler(obj)
        else:
            res = handle_request_full(obj, default_drones=self.default_drones, default_home=self.default_home,
                                      defaults=self.defaults, terrain_margin_m=self.terrain_margin_m, qgc=self.qgc)
        out: list[tuple[str, dict[str, Any]]] = []
        for drone_id, pl in res.payloads.items():
            t = self.topic_response_fmt.format(mission_id=res.mission_id, drone_id=drone_id)
            self.publish(t, pl)
            out.append((t, pl))
        st = dict(res.status)
        st.setdefault("event", "planned")
        self.publish(self.topic_status, st, retain=False)
        out.append((self.topic_status, st))
        with self._lock:
            self.results.append(res)
        if self.on_result is not None:
            try:
                self.on_result(res)
            except Exception:  # noqa: BLE001
                _log.exception("on_result callback failed")
        return out


def _json_default(o: Any) -> Any:
    if isinstance(o, (np.floating, np.integer)):
        v = o.item()
        return v if not (isinstance(v, float) and not math.isfinite(v)) else None
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (set, frozenset, tuple)):
        return list(o)
    if hasattr(o, "to_dict"):
        return o.to_dict()
    return str(o)
