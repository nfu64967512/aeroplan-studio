"""aeroplan_adapter.py — AeroPlan Studio（PyQt6 地面站，v2.8.1）之 HV-DCCPP 轉接層。

用途
----
把 `hv_dccpp` 的異構雙構型 VTOL 覆蓋規劃，**原地**接到 AeroPlan Studio 既有的 DCCPP 接縫上：

    REPO/mission/swarm_coordinator.py:975  ``SwarmCoordinator.plan_coverage_dccpp(...)``

:class:`HvDccppCoordinator.plan_coverage_dccpp` 之簽章與該方法**逐字相同**，回傳同樣的 6 鍵字典
（``results / allocation / coverage_paths / uav_states / total_makespan / assembled_paths``，
見 REPO/mission/swarm_coordinator.py:1291-1298），因此 AeroPlan 的全部下游消費端不需修改：

    * 地圖顯示   REPO/ui/main_window.py:5015 ``map_widget.display_swarm_raw`` 與 :4877-4932
    * NFZ 重繞   REPO/ui/main_window.py:4844
    * 防撞       REPO/ui/main_window.py:4852-4870
    * .waypoints 匯出  REPO/ui/main_window.py:5213
    * SITL 上傳  REPO/ui/main_window.py:6041-6210 → REPO/mission/sitl_link.py:264 ``upload_mission``
    * spawn home REPO/ui/main_window.py:6330-6356

**本模組不 import AeroPlan Studio 的任何模組**（`hv_dccpp` 不得相依於它）。所有回傳物件皆為本模組自建的
duck-typed 型別（``Hv*``），欄位名與 AeroPlan 一致；若呼叫端已經 import 了 AeroPlan 的真型別，可用
:func:`use_native_types` 讓本模組改用它們。

型別對照（欄位名照抄 AeroPlan，不可改）
--------------------------------------
========================  ===================================================================
本模組                    AeroPlan Studio 原型
========================  ===================================================================
:class:`HvSegmentLabel`   ``SegmentLabel``（REPO/core/dccpp/dccpp_path_builder.py:45-51）
:class:`HvBuiltWaypoint`  ``BuiltWaypoint``（同上 :54-62）＋ ``AssembledWaypoint.heading_deg``
                          （REPO/core/trajectory/dccpp_path_assembler.py:47-87）
:class:`HvBuiltPath`      ``BuiltPath``（同上 :65-78）＋ ``AssembledPath`` 之
                          ``takeoff_length_m`` / ``landing_length_m`` / ``to_latlon_list()``
:class:`HvUavState`       ``UAVState``（REPO/core/global_planner/mdtsp_solver.py:43-63）
:class:`HvCoveragePath`   ``CoveragePath``（REPO/mission/coverage_path.py:57-71）
:class:`HvOperationSegment` ``OperationSegment``（同上 :25-51）
:class:`HvMdtspResult`    ``MDTSPResult``（REPO/core/global_planner/mdtsp_solver.py:81-99）
========================  ===================================================================

消費端以 duck typing 同時支援 live 與 legacy 兩套欄位名
（``getattr(wp, 'heading_compass_deg', getattr(wp, 'heading_deg', 0.0))``，REPO/ui/main_window.py:5308；
``getattr(apath, 'takeoff_length_m', 0.0)``，:5361），故本模組之航點**同時**提供
``heading_compass_deg`` 與 ``heading_deg``，路徑**同時**提供五個 ``*_length_m``、``to_latlon()``、
``to_latlon_list()`` 與 ``to_latlon_3d_list()``。

階段（Phase）→ 區段標籤（SegmentLabel）映射
-------------------------------------------
`hv_dccpp` 有 14 種 :class:`hv_dccpp.mission.Phase`（規格 §3.4、§11.1），AeroPlan 只有 5 種
``SegmentLabel``；本模組採下列映射（:data:`PHASE_TO_SEGMENT`），原則是「地面／垂直段 → TAKEOFF/LANDING、
出海段 → ENTRY、掃描 → OPERATION、機動 → TRANSFER、回收 → LANDING」：

====================================  ==========  ==========================================
Phase                                 標籤        理由
====================================  ==========  ==========================================
TAKEOFF, TRANS_OUT                    TAKEOFF     垂直起飛與 MC→FW 轉換皆屬起飛序列
ENTRY, CLIMB, ORBIT, ACCEL            ENTRY       進場／爬升／盤旋等高／加速皆為「進入作業」段
OPERATE                               OPERATION   條帶掃描（感測器 ON）
TRANSFER, HOLD                        TRANSFER    條帶間轉移與等待皆為機動段
RTB, DESCENT, DECEL, TRANS_IN, LAND   LANDING     返場、下降、減速、FW→MC 轉換、垂直降落
====================================  ==========  ==========================================

高度基準（重要）
----------------
AeroPlan 的匯出／上傳一律採 **AGL 相對 home**（``MAV_FRAME_GLOBAL_RELATIVE_ALT`` = 3；
REPO/utils/file_io.py:226、REPO/mission/vtol_mission_exporter.py:32、REPO/mission/sitl_link.py:1274）。
故 :attr:`HvBuiltWaypoint.alt` 一律輸出 ``alt_msl_m − 規劃 home MSL``（全機共用基準；**不是** AGL 對當地地形，
也不是 AMSL）；hv_dccpp 內部之 ``Waypoint.agl_m`` 為對當地地形之 AGL，兩者在起伏地形不相等。
⚠ 飛控的 home 是**這架自己的起飛坪**（``UAVPlan.takeoff_site``），不是規劃 home：ArduPlane 解鎖時把 home
重設到飛機當下位置（見 :func:`hv_dccpp.ardupilot.fc_home`）。路徑上的 ``fc_alt_offset_m`` ＝ (坪 − 規劃 home)，
送飛控的相對高度＝``alt − fc_alt_offset_m``（:func:`to_upload_tuples` 已處理）；平地時偏移 0。``alt`` 本身保留
共用基準，是因為避撞（core/dccpp/collision_avoidance.py）與 2D/3D 顯示要跨機比較高度。

已知落差（手冊必須明講；詳見 CONTRACT_DEVIATIONS.md「AeroPlan Studio 轉接層」一節）
------------------------------------------------------------------------------------
1. AeroPlan 的 ``fov_width`` 欄位從未被讀（REPO/mission/swarm_coordinator.py:1090 只寫不讀），
   區域分解用單一 ``coverage_width_m``；A 的 LiDAR 刈幅與 B 的相機刈幅無法用該介面表達 →
   本模組改由 :class:`hv_dccpp.mission.PayloadRequirement` + 機上硬體逐構型推導刈幅，
   ``coverage_width_m`` 只作為**核對值**（差異 > 25% 時記警告）。
2. AeroPlan 全隊共用單一 R_min 與單一高度（:1184, :1204-1205）；本模組逐構型採
   ``spec.turn_radius_planning_m``（A 200 m、B 120 m）與逐構型作業高度（規格 §9 分層）。
3. AeroPlan 之 ``'vtol'`` 靜默降級為 ``MULTIROTOR``（:1080-1082）；本模組回傳的
   :class:`HvUavState.vehicle_type` 預設為 ``FIXED_WING``（QuadPlane 巡航段為固定翼），
   避免 REPO/core/dccpp/task_allocator.py:136 對 QuadPlane 誤用多旋翼 haversine 進入成本。
4. ``enable_altitude=False`` 無效：hv_dccpp 一律做高度規劃（規格 §9、Alg.3），關不掉。
5. 地球半徑／投影不一致：AeroPlan 部分模組用 111111.0 m/deg（REPO/core/geometry/region_divider.py:260-283），
   hv_dccpp 用 WGS84 橢球（``geo.GeoFrame``）→ 距離與 makespan 與 AeroPlan 舊值差約 0.1%。
6. ``dem_path`` 不在此讀取（需 rasterio）：請由呼叫端傳 ``terrain_func``
   （REPO/sensors/dem_terrain.py:175 ``DEMTerrainManager.get_elevation``），本模組以
   :func:`terrain_func_to_dem` 取樣建網格。

執行緒
------
規劃為同步阻塞（大情境數十秒），AeroPlan 的規劃在 GUI 執行緒（REPO/ui/main_window.py:4753-4776，
只有 :4749-4751 一次 ``processEvents()``）→ **請在 QThread 內呼叫**，並用 ``progress`` 回呼回報進度。
Qt 端寫法（本套件不 import PyQt6，僅示意）::

    # REPO/ui/workers/planning_worker.py（比照 REPO/mission/sitl_link.py:163 SITLLink 寫法）
    class HvPlanningWorker(QThread):
        progress = pyqtSignal(float, str)
        finished_ok = pyqtSignal(dict)
        failed = pyqtSignal(str)

        def __init__(self, areas, drones, params, parent=None):
            super().__init__(parent)
            self._areas, self._drones, self._params = areas, drones, params

        def run(self):
            try:
                from hv_dccpp.integration.aeroplan_adapter import HvDccppCoordinator, to_planner_config
                coord = HvDccppCoordinator(cfg=to_planner_config(self._params),
                                           progress=lambda f, m: self.progress.emit(f, m))
                res = coord.plan_coverage_dccpp(
                    areas=self._areas, drones=self._drones,
                    coverage_width_m=float(self._params['fov_width_m']),
                    overlap_rate=float(self._params['overlap_rate']),
                    coverage_altitude=float(self._params['altitude']))
                self.finished_ok.emit(res)
            except Exception as e:
                self.failed.emit(str(e))
"""
from __future__ import annotations

import logging
import math
import re
from dataclasses import dataclass, field, fields, is_dataclass, replace
from enum import Enum, auto
from typing import Any, Callable, Iterable, Literal, Mapping, Sequence

import numpy as np

from ..ardupilot import CameraTriggerConfig, MissionItem, fc_home, plan_to_mission_items
from ..context import MissionContext, build_context
from ..geo import DemGrid, GeoFrame
from ..geometry import is_cross_strip
from ..mission import (
    AreaSpec, Airspace, C2Link, MappingMissionInputs, MappingModeConfig, MissionInputError, MissionInputs, MissionPlan,
    NoFlyZone, PayloadRequirement,
    Phase, PlannerConfig as HvPlannerConfig, Site, SortieEnergy, UAVInstance, UAVPlan, Waypoint, WeatherObs,
    WeatherThresholds, WindLayer,
)
from ..planner import plan_mission
from ..platform import (
    PLATFORM_HERO, PLATFORM_PIONEER, CameraHardware, LidarHardware, PlatformSpec, SensorHardware, TrapezoidFovSpec,
)

__all__ = [
    "AeroPlanAdapterError",
    "HvSegmentLabel", "HvVehicleType", "HvBuiltWaypoint", "HvBuiltPath",
    "HvUavState", "HvOperationSegment", "HvCoveragePath", "HvMdtspResult",
    "HvTypeRegistry", "use_native_types", "default_types",
    "HvDccppCoordinator", "HvPlannerConfig",
    "PHASE_TO_SEGMENT", "SEGMENT_NAMES", "PARAM_KEY_NOTES",
    "DEFAULT_ENERGY_A", "DEFAULT_ENERGY_B",
    "to_mission_inputs", "to_planner_config", "from_mission_plan", "to_upload_tuples",
    "mission_item_dicts", "expected_photo_count",
    "fence_zones_to_nofly", "terrain_func_to_dem", "telemetry_frames_to_events", "plan_to_aeroplan_waypoints",
    "qgc_wpl_lines", "MAV_CMD",
]


_log = logging.getLogger(__name__)


class AeroPlanAdapterError(RuntimeError):
    """轉接層輸入錯誤或規劃 NO_GO（`errors` 保留 preflight 之錯誤清單）。"""

    def __init__(self, msg: str, errors: Sequence[str] = ()) -> None:
        super().__init__(msg)
        self.errors = list(errors)


# ===========================================================================
# 1. duck-typed 輸出型別（欄位名對齊 AeroPlan；本模組不 import AeroPlan）
# ===========================================================================

class HvSegmentLabel(Enum):
    """對應 ``SegmentLabel``（REPO/core/dccpp/dccpp_path_builder.py:45-51）。

    消費端一律比對 ``.segment_type.name`` 字串（REPO/ui/main_window.py:6060-6066、:6113-6136），
    故成員名必須恰為這五個字串。
    """
    TAKEOFF = auto()
    ENTRY = auto()
    OPERATION = auto()
    TRANSFER = auto()
    LANDING = auto()


SEGMENT_NAMES: tuple[str, ...] = ("TAKEOFF", "ENTRY", "OPERATION", "TRANSFER", "LANDING")
"""AeroPlan 消費端唯一接受的五個標籤字串。"""


class HvVehicleType(Enum):
    """對應 ``VehicleType``（REPO/core/global_planner/mdtsp_solver.py:37-40，**原型無 VTOL**）。

    本模組多提供 ``VTOL`` 成員供新程式使用；但 :class:`HvUavState.vehicle_type` 預設填 ``FIXED_WING``，
    因為 QuadPlane 之巡航段為固定翼、且 REPO/core/dccpp/task_allocator.py:136-142 以
    ``vehicle_type is FIXED_WING`` 決定是否採 Dubins 進入成本（填 MULTIROTOR 會低估轉彎成本）。
    """
    MULTIROTOR = auto()
    FIXED_WING = auto()
    VTOL = auto()


PHASE_TO_SEGMENT: dict[Phase, HvSegmentLabel] = {
    Phase.TAKEOFF: HvSegmentLabel.TAKEOFF,
    Phase.TRANS_OUT: HvSegmentLabel.TAKEOFF,
    Phase.ACCEL: HvSegmentLabel.ENTRY,
    Phase.CLIMB: HvSegmentLabel.ENTRY,
    Phase.ORBIT: HvSegmentLabel.ENTRY,
    Phase.ENTRY: HvSegmentLabel.ENTRY,
    Phase.OPERATE: HvSegmentLabel.OPERATION,
    Phase.TRANSFER: HvSegmentLabel.TRANSFER,
    Phase.HOLD: HvSegmentLabel.TRANSFER,
    Phase.RTB: HvSegmentLabel.LANDING,
    Phase.DESCENT: HvSegmentLabel.LANDING,
    Phase.DECEL: HvSegmentLabel.LANDING,
    Phase.TRANS_IN: HvSegmentLabel.LANDING,
    Phase.LAND: HvSegmentLabel.LANDING,
}
"""14 種 :class:`hv_dccpp.mission.Phase` → 5 種 :class:`HvSegmentLabel`（見模組 docstring 之對照表）。"""


@dataclass
class HvBuiltWaypoint:
    """對應 ``BuiltWaypoint``（REPO/core/dccpp/dccpp_path_builder.py:54-62）。

    參數
    ----
    lat, lon
        WGS84 度（AeroPlan 全域慣例為 ``(lat, lon)``；REPO/mission/geofence_manager.py:60）。
    alt
        相對**規劃 home**（m）= ``alt_msl_m − 規劃 home MSL``（全機共用基準）。送飛控
        （``MAV_FRAME_GLOBAL_RELATIVE_ALT``，frame 3；飛控 home ＝ 這架的起飛坪）前要減掉路徑的
        ``fc_alt_offset_m``，:func:`to_upload_tuples` 已處理。
    heading_compass_deg
        航跡方位（度，0 = 北、順時針），已正規化至 ``[0, 360)``。
    segment_type
        :class:`HvSegmentLabel`。

    ``heading_deg`` 為 ``heading_compass_deg`` 之別名（legacy ``AssembledWaypoint`` 欄位名，
    REPO/core/trajectory/dccpp_path_assembler.py:47-63），消費端 REPO/ui/main_window.py:5308 兩者皆讀。

    sensor_on
        相機／感測器在這一點是不是開著。⚠⚠⚠ vendored 修改 #5：這個欄位原本**沒有
        被帶出來**，於是下游只剩 ``segment_type`` 可以判斷「哪一段算覆蓋」。而兩個
        規劃器的段落標籤慣例不同 —— 實測同一份幾何：DCCPP 73 個航點裡 14 個是
        ``OPERATION``（19.2%），HARVEST 306 個裡只有 16 個（**5.2%**，170 個標成
        ``TRANSFER``）。用 ``OPERATION`` 篩選去驗算覆蓋率，HARVEST 從 **100.00%**
        掉到 **75.10%**，而 DCCPP 只從 76.68% 掉到 74.04% ⇒ **同一把尺對兩個規劃器
        不對等**，會讓人以為 HARVEST 漏掃三分之一。
        相機開沒開才是「這一段算不算覆蓋」的語意判準，而且與規劃器無關。
    """
    lat: float
    lon: float
    alt: float
    heading_compass_deg: float
    segment_type: HvSegmentLabel
    sensor_on: bool = False

    @property
    def heading_deg(self) -> float:
        """legacy ``AssembledWaypoint.heading_deg`` 別名（與 ``heading_compass_deg`` 恆等）。"""
        return self.heading_compass_deg

    def to_tuple(self) -> tuple[float, float]:
        """``(lat, lon)``（同 ``BuiltWaypoint.to_tuple``）。"""
        return (self.lat, self.lon)


@dataclass
class HvBuiltPath:
    """對應 ``BuiltPath``（REPO/core/dccpp/dccpp_path_builder.py:65-78）∪ ``AssembledPath``
    （REPO/core/trajectory/dccpp_path_assembler.py:66-87）之聯集欄位。

    五個 ``*_length_m`` 皆為**水平**長度（ENU 平面，m），依航點所屬區段標籤累加：第 i 段（wp[i]→wp[i+1]）
    計入 ``PHASE_TO_SEGMENT[wp[i].phase]``（hv_dccpp 之 Waypoint 欄位描述「出發腿」，D45 同慣例）。
    ``total_length_m`` 為五者之和。

    尾端 ``hv_uav_id`` / ``cls`` / ``makespan_s`` 為本轉接層附加（AeroPlan 原型無，以 getattr 讀取者不受影響）。
    ``alt_ref_site`` / ``alt_ref_msl_m``：各航點 ``alt`` 的基準＝**規劃 home**（全機共用；避撞、2D/3D 顯示跨機比高度靠它）。
    ``fc_home_site`` / ``fc_home_msl_m``：這架的起飛坪＝飛控解鎖後的 home；``fc_alt_offset_m``＝坪 − 規劃 home。
    送給飛控的相對高度＝``alt − fc_alt_offset_m``（:func:`to_upload_tuples`、legacy 匯出器）；平地時偏移 0。
    ``land_alt_m``：降落站地面相對**飛控 home**（起飛坪）的高度（降回自己的起飛坪 ⇒ 0），NAV_VTOL_LAND 用它。
    """
    uav_id: int
    waypoints: list[HvBuiltWaypoint] = field(default_factory=list)
    entry_length_m: float = 0.0
    operation_length_m: float = 0.0
    transfer_length_m: float = 0.0
    takeoff_length_m: float = 0.0
    landing_length_m: float = 0.0
    hv_uav_id: str = ""
    cls: str = ""
    makespan_s: float = 0.0
    alt_ref_site: str = ""
    alt_ref_msl_m: float = 0.0
    fc_home_site: str = ""
    fc_home_msl_m: float = 0.0
    fc_alt_offset_m: float = 0.0
    land_alt_m: float = 0.0

    @property
    def total_length_m(self) -> float:
        """五段長度之和（m）。"""
        return (self.takeoff_length_m + self.entry_length_m + self.operation_length_m
                + self.transfer_length_m + self.landing_length_m)

    def to_latlon(self) -> list[tuple[float, float]]:
        """``[(lat, lon), ...]``（live ``BuiltPath.to_latlon``）。"""
        return [(w.lat, w.lon) for w in self.waypoints]

    def to_latlon_list(self) -> list[tuple[float, float]]:
        """``[(lat, lon), ...]``（legacy ``AssembledPath.to_latlon_list``；與 :meth:`to_latlon` 同）。"""
        return self.to_latlon()

    def to_latlon_3d_list(self) -> list[tuple[float, float, float]]:
        """``[(lat, lon, alt_rel_home), ...]``。"""
        return [(w.lat, w.lon, w.alt) for w in self.waypoints]


@dataclass
class HvUavState:
    """對應 ``UAVState``（REPO/core/global_planner/mdtsp_solver.py:43-63）。

    ``fov_width`` 在 AeroPlan 中**從未被讀取**（只在 REPO/mission/swarm_coordinator.py:1090 寫入），
    此處填該機主感測器於作業高度之實際刈幅，僅供顯示／除錯。
    """
    uav_id: int
    position: tuple[float, float]
    heading: float = 0.0
    turn_radius: float = 0.0
    speed: float = 15.0
    vehicle_type: Any = HvVehicleType.FIXED_WING
    fov_width: float = 100.0


@dataclass
class HvOperationSegment:
    """對應 ``OperationSegment``（REPO/mission/coverage_path.py:25-51）。

    ``left_point`` / ``right_point`` 為 hv_dccpp 條帶（:class:`hv_dccpp.geometry.Strip`）之
    ``pL`` / ``pR`` 端點轉回 ``(lat, lon)``；若該條帶之飛行方向為 RL（``dir=+1``），比照
    REPO/core/dccpp/idp_solver.py:386-397 之慣例交換兩端，使 ``left_point`` 恆為實際進入端。
    """
    index: int
    left_point: tuple[float, float]
    right_point: tuple[float, float]
    waypoints: list[tuple[float, float]] = field(default_factory=list)
    length_m: float = 0.0
    covered: bool = False
    assigned_drone_id: int | None = None

    def entry_points(self) -> tuple[tuple[float, float], tuple[float, float]]:
        """``(left_point, right_point)``（同原型）。"""
        return self.left_point, self.right_point


@dataclass
class HvCoveragePath:
    """對應 ``CoveragePath``（REPO/mission/coverage_path.py:57-71）。

    ``scan_angle_deg`` 為條帶 ``pL → pR`` 之羅盤方位（度，0 = 北、順時針），取該區域全部條帶之中位數；
    hv_dccpp 允許同一區域內不同胞／不同構型有不同掃描角（規格 §7.3），單一數值僅為 AeroPlan 顯示用。
    """
    area_id: int
    area_polygon: list[tuple[float, float]]
    scan_angle_deg: float
    operations: list[HvOperationSegment] = field(default_factory=list)
    priority: float = 1.0

    def get_uncovered_operations(self) -> list[HvOperationSegment]:
        """未完成之作業路徑（同原型查詢方法）。"""
        return [o for o in self.operations if not o.covered]


@dataclass
class HvMdtspResult:
    """對應 ``MDTSPResult``（REPO/core/global_planner/mdtsp_solver.py:81-99）。

    ``makespan`` 依原型註解為「最大完工時間（秒）」，但 REPO/core/dccpp/idp_solver.py:407 實際填的是
    ``idp_result.makespan_length``（公尺）。本模組填**秒**（規格 §10 之 makespan），與欄位文件一致；
    若需與 AeroPlan 舊行為逐值比對，請改讀 ``total_distances``。
    """
    area_id: int
    uav_assignments: dict[int, list[HvOperationSegment]] = field(default_factory=dict)
    path_matrix: Any = None
    decision_matrix: Any = None
    total_distances: dict[int, float] = field(default_factory=dict)
    makespan: float = 0.0
    assembled_paths: dict[int, Any] | None = None


# --------------------------------------------------------------------------
# 型別註冊表：可換成 AeroPlan 的真型別
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class HvTypeRegistry:
    """本模組建構輸出物件時所用的類別；預設全部為上面的 ``Hv*``。"""
    segment_label: type = HvSegmentLabel
    built_waypoint: type = HvBuiltWaypoint
    built_path: type = HvBuiltPath
    uav_state: type = HvUavState
    vehicle_type: type = HvVehicleType
    coverage_path: type = HvCoveragePath
    operation_segment: type = HvOperationSegment
    mdtsp_result: type = HvMdtspResult


default_types: HvTypeRegistry = HvTypeRegistry()
"""模組層級之預設型別註冊表；:func:`use_native_types` 會改寫它。"""

_NATIVE_KEYS: dict[str, str] = {
    "SegmentLabel": "segment_label",
    "BuiltWaypoint": "built_waypoint",
    "BuiltPath": "built_path",
    "AssembledPath": "built_path",
    "AssembledWaypoint": "built_waypoint",
    "UAVState": "uav_state",
    "VehicleType": "vehicle_type",
    "CoveragePath": "coverage_path",
    "OperationSegment": "operation_segment",
    "MDTSPResult": "mdtsp_result",
}


def use_native_types(module_map: Mapping[str, Any] | None, *, set_default: bool = True) -> HvTypeRegistry:
    """改用 AeroPlan Studio 的真型別建構回傳物件（可選；不呼叫時用本模組的 ``Hv*``）。

    參數
    ----
    module_map
        ``{名稱: 類別或模組}``。名稱可為 AeroPlan 的類別名
        （``SegmentLabel`` / ``BuiltWaypoint`` / ``BuiltPath`` / ``AssembledPath`` / ``UAVState`` /
        ``VehicleType`` / ``CoveragePath`` / ``OperationSegment`` / ``MDTSPResult``），
        值可以直接是類別，也可以是含有該類別的模組（會自動 ``getattr``）。
        傳 ``None`` 或空 mapping → 還原為本模組的 ``Hv*``。
    set_default
        True（預設）→ 同時改寫模組層級的 :data:`default_types`。

    返回
    ----
    :class:`HvTypeRegistry`

    用例（在 AeroPlan 端呼叫，本套件不 import 它）::

        import core.dccpp.dccpp_path_builder as pb
        import core.global_planner.mdtsp_solver as ms
        import mission.coverage_path as cp
        use_native_types({"SegmentLabel": pb, "BuiltWaypoint": pb, "BuiltPath": pb,
                          "UAVState": ms, "VehicleType": ms, "MDTSPResult": ms,
                          "CoveragePath": cp, "OperationSegment": cp})

    註：AeroPlan 的 live ``BuiltPath``（REPO/core/dccpp/dccpp_path_builder.py:65-78）**沒有**
    ``takeoff_length_m`` / ``landing_length_m`` / ``to_latlon_list()``；本模組建構後會嘗試 ``setattr``
    補上兩個長度欄位（該 dataclass 未用 ``__slots__``，可行），但 ``to_latlon_list()`` 無法補
    → 使用原生型別時請確認下游只呼叫 ``to_latlon()``（REPO/ui/main_window.py:5361 即以 getattr 取長度）。
    """
    global default_types
    reg = HvTypeRegistry()
    if module_map:
        kw: dict[str, type] = {}
        for name, obj in module_map.items():
            attr = _NATIVE_KEYS.get(str(name))
            if attr is None:
                continue
            cls = obj if isinstance(obj, type) else getattr(obj, str(name), None)
            if isinstance(cls, type):
                kw.setdefault(attr, cls)
        reg = replace(reg, **kw) if kw else reg
    if set_default:
        default_types = reg
    return reg


def _construct(target_cls: type, /, **kw: Any) -> Any:
    """以 dataclass 欄位過濾後建構；未支援之欄位改用 ``setattr`` 補（失敗則忽略）。

    第一參數為 positional-only，避免與 :class:`HvBuiltPath` 的 ``cls`` 欄位撞名。
    """
    cls = target_cls
    names: set[str] | None = None
    if is_dataclass(cls):
        names = {f.name for f in fields(cls)}
    if names is None:
        obj = cls(**kw)
        return obj
    accepted = {k: v for k, v in kw.items() if k in names}
    extra = {k: v for k, v in kw.items() if k not in names}
    obj = cls(**accepted)
    for k, v in extra.items():
        try:
            setattr(obj, k, v)
        except Exception:                      # __slots__ 或 frozen → 放棄補欄位
            pass
    return obj


def _segment(label: HvSegmentLabel, reg: HvTypeRegistry) -> Any:
    """把本模組之 :class:`HvSegmentLabel` 轉為註冊表中的標籤型別（依 ``name`` 對應）。"""
    cls = reg.segment_label
    if cls is HvSegmentLabel:
        return label
    return getattr(cls, label.name)


def _vehicle_type(name: str, reg: HvTypeRegistry) -> Any:
    """取註冊表之 VehicleType 成員；原生 ``VehicleType`` 無 ``VTOL`` 時退回 ``FIXED_WING``。"""
    cls = reg.vehicle_type
    v = getattr(cls, name, None)
    if v is None:
        v = getattr(cls, "FIXED_WING", None)
    if v is None:                              # pragma: no cover - 不可能的原生型別
        v = name
    return v


# ===========================================================================
# 2. 小工具
# ===========================================================================

_EARTH_R_M = 6371000.0
"""球面近似半徑，只用於 :func:`fence_zones_to_nofly` 圓形展開（與 REPO/utils/math_utils.EARTH_RADIUS_M 對齊）。"""

DEFAULT_ENERGY_A: dict[str, float] = {"auw_kg": 24.6, "payload_kg": 5.0, "available_wh": 1350.0,
                                      "pack_nominal_wh": 1500.0}
"""構型 A（Pioneer PNP + 5 kg LiDAR）當架次能量預設（同 `scenario.build_mini_example`）。"""
DEFAULT_ENERGY_B: dict[str, float] = {"auw_kg": 6.65, "payload_kg": 0.6, "available_wh": 380.0,
                                      "pack_nominal_wh": 420.0}
"""構型 B（HERO PNP + 0.6 kg 相機）當架次能量預設。"""

_SPEC_OF: dict[str, PlatformSpec] = {"A": PLATFORM_PIONEER, "B": PLATFORM_HERO}

_NAME_CLS_RE = re.compile(r"(?:^|[^A-Z])([AB])(?=\d)")
"""由機名推構型：取「後面緊接數字」的 A/B（``"UAV-B2"`` → B、``"A1"`` → A；``"UAV"`` 不匹配）。"""


def _compass(deg: float) -> float:
    """正規化羅盤方位至 ``[0, 360)``。"""
    d = float(deg) % 360.0
    return d + 360.0 if d < 0.0 else d


def _latlon(p: Any, name: str) -> tuple[float, float]:
    """``(lat, lon)`` 二元組（容忍 list / tuple / numpy / 含第三元素）。"""
    try:
        seq = list(p)
    except TypeError as e:
        raise AeroPlanAdapterError(f"{name} 須為 (lat, lon)，得 {p!r}") from e
    if len(seq) < 2:
        raise AeroPlanAdapterError(f"{name} 須為 (lat, lon)，得 {p!r}")
    return (float(seq[0]), float(seq[1]))


def _get(obj: Any, name: str, default: Any = None) -> Any:
    """duck typing 讀屬性或 mapping 鍵。"""
    if isinstance(obj, Mapping):
        return obj.get(name, default)
    return getattr(obj, name, default)


def _enum_name(v: Any) -> str:
    """Enum → ``name``；字串 → 原值（大寫）；其他 → ``str``。"""
    n = getattr(v, "name", None)
    if isinstance(n, str):
        return n.upper()
    if isinstance(v, str):
        return v.upper()
    val = getattr(v, "value", None)
    return str(val if val is not None else v).upper()


def _metres_per_deg(lat_deg: float) -> tuple[float, float]:
    """該緯度之 (m/°lat, m/°lon)（WGS84 一階近似；只用於建 DEM 邊界）。"""
    lat = math.radians(lat_deg)
    m_lat = 111132.92 - 559.82 * math.cos(2 * lat) + 1.175 * math.cos(4 * lat)
    m_lon = 111412.84 * math.cos(lat) - 93.5 * math.cos(3 * lat)
    return (m_lat, max(m_lon, 1.0))


# ===========================================================================
# 3. 轉換函式（公開；手冊逐段引用）
# ===========================================================================

def fence_zones_to_nofly(zones: Iterable[Any] | None, *, alt_ref: Literal["msl", "agl"] = "msl",
                         home_msl_m: float = 0.0, circle_segments: int = 16,
                         include_categories: Sequence[str] = ("NFZ", "THREAT")) -> tuple[NoFlyZone, ...]:
    """AeroPlan ``FenceZone`` 序列 → :class:`hv_dccpp.mission.NoFlyZone` 元組（duck typing，不 import AeroPlan）。

    來源：REPO/mission/fence_zone.py:48 ``FenceZone``；全域單例
    ``FenceZoneRegistry.instance().by_category(ZoneCategory.NFZ)``（:181、:220）。

    參數
    ----
    zones
        任何具 ``category`` / ``shape`` / ``vertices`` / ``center`` / ``radius_m`` / ``alt_min_m`` /
        ``alt_max_m`` 屬性之物件序列；有 ``polygon_vertices(circle_segments)`` 方法者優先使用之
        （REPO/mission/fence_zone.py:87，圓形展開為 N 邊形）。``None`` → 空元組。
    alt_ref
        ``FenceZone.alt_min_m`` / ``alt_max_m`` 之基準（原始碼註解為「AMSL 或 AGL 依專案慣例」，
        REPO/mission/fence_zone.py:61-63 未定）。``"msl"``（預設）直接採用；``"agl"`` 時加上 ``home_msl_m``，
        因為 :class:`hv_dccpp.mission.NoFlyZone` 之樓層／天花板一律為 **MSL**（規格 §6.1、D42）。
    circle_segments
        圓形展開邊數（同原型預設 16，精度 < 2% R）。
    include_categories
        取哪些 ``category``；預設只取 exclusion 類 ``NFZ`` / ``THREAT``。``GEOFENCE`` 為 inclusion
        （UAV 必須留在**內**），語意與 NoFlyZone 相反，預設略過。

    返回
    ----
    ``tuple[NoFlyZone, ...]``；``zone_id`` 取 ``id`` → ``name`` → ``NFZ{i}``（去重時加序號）。
    頂點少於 3、或圓形缺 center/radius 者略過。
    """
    if zones is None:
        return ()
    keep = {str(c).upper() for c in include_categories}
    out: list[NoFlyZone] = []
    used: set[str] = set()
    for i, z in enumerate(zones):
        cat = _enum_name(_get(z, "category", "NFZ"))
        if keep and cat not in keep:
            continue
        verts: list[tuple[float, float]] = []
        pv = _get(z, "polygon_vertices")
        if callable(pv):
            try:
                verts = [_latlon(p, "FenceZone.polygon_vertices") for p in pv(circle_segments)]
            except TypeError:                                  # 無參數版本
                verts = [_latlon(p, "FenceZone.polygon_vertices") for p in pv()]
        if not verts:
            shape = _enum_name(_get(z, "shape", "POLYGON"))
            if shape == "CIRCLE":
                c = _get(z, "center")
                r = float(_get(z, "radius_m", 0.0) or 0.0)
                if c is None or r <= 0.0:
                    continue
                lat0, lon0 = _latlon(c, "FenceZone.center")
                ang = r / _EARTH_R_M
                lat0r, lon0r = math.radians(lat0), math.radians(lon0)
                for k in range(int(circle_segments)):
                    brg = 2.0 * math.pi * k / float(circle_segments)
                    lat_r = math.asin(math.sin(lat0r) * math.cos(ang)
                                      + math.cos(lat0r) * math.sin(ang) * math.cos(brg))
                    lon_r = lon0r + math.atan2(math.sin(brg) * math.sin(ang) * math.cos(lat0r),
                                               math.cos(ang) - math.sin(lat0r) * math.sin(lat_r))
                    verts.append((math.degrees(lat_r), math.degrees(lon_r)))
            else:
                raw = _get(z, "vertices") or ()
                verts = [_latlon(p, "FenceZone.vertices") for p in raw]
        if len(verts) < 3:
            continue
        zid = str(_get(z, "id", "") or _get(z, "name", "") or f"NFZ{i + 1}")
        if zid in used:
            zid = f"{zid}#{i + 1}"
        used.add(zid)
        off = float(home_msl_m) if alt_ref == "agl" else 0.0
        z_lo = _get(z, "alt_min_m")
        z_hi = _get(z, "alt_max_m")
        floor = None if z_lo is None else float(z_lo) + off
        ceil = None if z_hi is None else float(z_hi) + off
        if floor is not None and ceil is not None and ceil <= floor:
            ceil = None                                        # 無效區間 → 視為無上限
        out.append(NoFlyZone(zone_id=zid, polygon_wgs84=tuple(verts), z_floor_msl_m=floor, z_ceiling_msl_m=ceil))
    return tuple(out)


def terrain_func_to_dem(terrain_func: Callable[[float, float], float] | None,
                        bounds_latlon: tuple[float, float, float, float], step_m: float = 50.0, *,
                        base_msl_m: float = 0.0, sigma_z_m: float = 2.0, is_dsm: bool = False,
                        max_nodes: int = 4_000_000) -> DemGrid:
    """AeroPlan 的地形回呼 → :class:`hv_dccpp.geo.DemGrid`（規則經緯網格，雙線性內插）。

    參數
    ----
    terrain_func
        ``f(lat, lon) -> 高程（公尺 AMSL）``，即 REPO/sensors/dem_terrain.py:175
        ``DEMTerrainManager.get_elevation``（AeroPlan 於 REPO/mission/swarm_coordinator.py:975
        以 ``terrain_func`` 參數傳入）。``None`` → 建 ``base_msl_m`` 的**平地** DEM。
        回呼回 ``None`` / ``nan`` 之處以 ``base_msl_m`` 填補。
    bounds_latlon
        ``(lat_min, lon_min, lat_max, lon_max)``，須已含足夠邊界裕度
        （建議 ≥ ``dem_margin_m``，見 :class:`HvDccppCoordinator`）。
    step_m
        目標節點距（公尺）；換算為 ``dlat_deg`` / ``dlon_deg``（以中心緯度）。
    base_msl_m
        平地高程／填補值。
    sigma_z_m
        DEM 1σ 高程誤差，進入 ``h_clr ≥ 3σ + 30`` 下限（D11，由 preflight 使用）。
    is_dsm
        True → tree/building 類障礙不再燒入（D24）。
    max_nodes
        節點數上限；超過時自動放大 ``step_m`` 並不丟例外（避免 GUI 因使用者給了整張圖而爆記憶體）。

    返回
    ----
    :class:`hv_dccpp.geo.DemGrid`（``z_msl`` 為 float32，rows 沿 lat、cols 沿 lon）。
    """
    lat_min, lon_min, lat_max, lon_max = (float(x) for x in bounds_latlon)
    if not (lat_max > lat_min and lon_max > lon_min):
        raise AeroPlanAdapterError(f"terrain_func_to_dem: 邊界須 lat_max>lat_min 且 lon_max>lon_min，得 {bounds_latlon!r}")
    lat_c = 0.5 * (lat_min + lat_max)
    m_lat, m_lon = _metres_per_deg(lat_c)
    step = max(float(step_m), 1.0)
    for _ in range(12):
        dlat = step / m_lat
        dlon = step / m_lon
        rows = max(2, int(math.ceil((lat_max - lat_min) / dlat)) + 1)
        cols = max(2, int(math.ceil((lon_max - lon_min) / dlon)) + 1)
        if rows * cols <= max_nodes:
            break
        step *= 2.0
    lats = lat_min + np.arange(rows, dtype=np.float64) * dlat
    lons = lon_min + np.arange(cols, dtype=np.float64) * dlon
    if terrain_func is None:
        z = np.full((rows, cols), float(base_msl_m), dtype=np.float32)
    else:
        z = np.empty((rows, cols), dtype=np.float32)
        for i, la in enumerate(lats):
            for j, lo in enumerate(lons):
                try:
                    v = terrain_func(float(la), float(lo))
                except Exception:                              # 回呼失敗 → 以平地值填補
                    v = None
                if v is None or not np.isfinite(float(v)):
                    v = base_msl_m
                z[i, j] = float(v)
    return DemGrid(lat0_deg=lat_min, lon0_deg=lon_min, dlat_deg=float(dlat), dlon_deg=float(dlon),
                   z_msl=z, is_dsm=bool(is_dsm), sigma_z_m=float(sigma_z_m))


PARAM_KEY_NOTES: dict[str, str] = {
    "vehicle_type": "忽略：hv_dccpp 只支援 lift-and-cruise 4+1 VTOL（構型 A/B），載具由 class_of 決定",
    "num_drones": "忽略：機隊由 plan_coverage_dccpp 的 drones 參數決定",
    "fov_width_m": "核對值：實際刈幅由感測器硬體 + 作業高度推導（規格 §4.2/§4.3）",
    "overlap_rate": "→ PayloadRequirement.side_overlap（不在 PlannerConfig）",
    "altitude": "→ PayloadRequirement.h_target_agl_m（不在 PlannerConfig）",
    "speed": "忽略：巡航空速為平台常數 spec.cruise_airspeed_mps（A 21.5、B 19.0 m/s）",
    "turn_radius": "忽略：逐構型採 spec.turn_radius_planning_m（A 200、B 120 m）＋風修正 r_eff",
    "auto_scan_angle": "忽略：hv_dccpp 一律最佳化掃描方向（規格 §7.3）",
    "enable_altitude": "忽略：高度規劃不可關閉（規格 §9、Alg.3）",
    "auto_landing": "忽略：一律規劃 RTB + 降落走廊（規格 §11）",
    "mounting_angle_deg": "→ TrapezoidFovSpec.mount_angle_deg（需同時給 hfov/vfov 才生效）",
    "hfov_deg": "→ TrapezoidFovSpec.h_fov_deg",
    "vfov_deg": "→ TrapezoidFovSpec.v_fov_deg",
    "coordination_mode": "忽略：hv_dccpp 一律做 4-D 去衝突（規格 §10.6）",
    "dem_path": "忽略：本層不讀 raster，請改傳 terrain_func",
    "takeoff_bearing": "→ runway_bearing_deg（只影響 UI 佈機，不影響 hv_dccpp 起飛走廊）",
    "runway_length": "忽略：VTOL 垂直起飛不需跑道",
    "landing_bearing": "忽略：逆風進場由 D19 決定",
    "pattern_alt": "忽略",
    "landing_rollout": "忽略：VTOL 垂直降落無滑行",
    "takeoff_spacing": "→ 只影響 UI 佈機；起飛錯開時距為 PlannerConfig.dt_dep_s",
    "collision_avoidance": "忽略：分離約束一律啟用（規格 §10.6、D14）",
    "min_separation_m": "→ PlannerConfig.d_sep_m",
    "alt_offset_m": "→ PlannerConfig.dh_sep_m",
    "avoid_strategy": "忽略",
}
"""REPO/ui/widgets/parameter_panel.py:2488 ``get_dccpp_params()`` 之 22+ 鍵在本層的處置（手冊逐鍵引用）。"""


def to_planner_config(params: Mapping[str, Any] | None, *, base: HvPlannerConfig | None = None) -> HvPlannerConfig:
    """AeroPlan ``get_dccpp_params()`` 字典 → :class:`hv_dccpp.mission.PlannerConfig`。

    來源：REPO/ui/widgets/parameter_panel.py:2488（22 鍵，經 :56 ``dccpp_coverage_requested`` signal
    送到 REPO/ui/main_window.py:4673 ``_on_dccpp_coverage_requested``）。

    只有兩個鍵真正落在 :class:`PlannerConfig`（規格 §16.2）上：

    * ``min_separation_m``（> 0）→ ``d_sep_m``（水平分離下限，規格 §10.6）
    * ``alt_offset_m``（> 0）→ ``dh_sep_m``（垂直分層間距，規格 §9.4）

    其餘鍵屬於任務輸入（→ :func:`to_mission_inputs`）或平台常數（不可調），逐鍵處置見
    :data:`PARAM_KEY_NOTES`。另可用 ``params["hv_cfg"] = {...}`` 直接覆寫任意 PlannerConfig 欄位
    （交給 ``PlannerConfig.from_dict``，未知鍵會丟 ``ValueError``）。

    參數
    ----
    params
        參數字典（``None`` → 全預設）。
    base
        基底設定（``None`` → ``PlannerConfig()``）。

    返回
    ----
    :class:`hv_dccpp.mission.PlannerConfig`（frozen；本函式回傳新物件）。
    """
    cfg = base if base is not None else HvPlannerConfig()
    if not params:
        return cfg
    kw: dict[str, Any] = {}
    sep = params.get("min_separation_m")
    if sep is not None and float(sep) > 0.0:
        kw["d_sep_m"] = float(sep)
    off = params.get("alt_offset_m")
    if off is not None and float(off) > 0.0:
        kw["dh_sep_m"] = float(off)
    extra = params.get("hv_cfg")
    if isinstance(extra, Mapping):
        kw.update(dict(extra))
    if not kw:
        return cfg
    d = cfg.to_dict()
    d.update(kw)
    return HvPlannerConfig.from_dict(d)


def to_mission_inputs(areas: Sequence[Mapping[str, Any]], drones: Sequence[Mapping[str, Any]], *,
                      coverage_width_m: float, overlap_rate: float = 0.1, coverage_altitude: float = 100.0,
                      terrain_func: Callable[[float, float], float] | None = None, dem_path: str = "",
                      home: tuple[float, float] | Site | None = None,
                      fence_zones: Iterable[Any] | None = None,
                      class_of: Callable[[Mapping[str, Any]], str] | None = None,
                      platform_a: PlatformSpec = PLATFORM_PIONEER, platform_b: PlatformSpec = PLATFORM_HERO,
                      sensor_a: Sequence[SensorHardware] | None = None,
                      sensor_b: Sequence[SensorHardware] | None = None,
                      payload_reqs: Mapping[str, PayloadRequirement] | None = None,
                      alternates: Sequence[Any] | None = None,
                      recovery_sites: Mapping[str, str] | None = None,
                      per_uav_takeoff_sites: bool = True,
                      land_at_takeoff_site: bool = True,
                      wind: tuple[float, float] = (0.0, 0.0), c2_range_km: float = 25.0,
                      terrain_clearance_m: float | None = None, waiver_max_agl_m: float | None = 200.0,
                      max_agl_m: float = 120.0, dem_step_m: float = 50.0, dem_margin_m: float = 1500.0,
                      base_msl_m: float = 0.0, weather_temp_c: float = 25.0,
                      mounting_angle_deg: float = 0.0, hfov_deg: float = 0.0, vfov_deg: float = 0.0,
                      energy: Mapping[str, Mapping[str, float]] | None = None,
                      warnings: list[str] | None = None,
                      mapping_mode: MappingModeConfig | None = None,
                      forward_overlap: float = 0.70) -> tuple[MissionInputs, dict[int, str]]:
    """AeroPlan 的 ``areas`` / ``drones`` → :class:`hv_dccpp.mission.MissionInputs`。

    參數（前段照 REPO/mission/swarm_coordinator.py:975 之同名參數）
    -------------------------------------------------------------
    areas
        ``[{'area_id': int, 'polygon': [(lat, lon), ...], 'priority': float}, ...]``
        （AeroPlan 組裝處 REPO/ui/main_window.py:4709）。``area_id`` → hv_dccpp 之 ``"Z{area_id}"``。
    drones
        ``[{'drone_id', 'name', 'position': (lat, lon), 'speed', 'heading', 'turn_radius',
        'vehicle_type', 'fov_width'}, ...]``（REPO/ui/main_window.py:4721-4744）。
        ``drone_id`` → hv_dccpp 之 ``"{cls}{drone_id}"``（例如 ``"A1"`` / ``"B2"``）。
    coverage_width_m
        UI 的單一刈幅（REPO/ui/widgets/parameter_panel.py:2496 ``fov_width_m``）。**僅作核對**：
        實際刈幅由感測器硬體與作業高度推導（障礙 1），差異 > 25% 時記警告 ``fov_width_mismatch``。
    overlap_rate
        側向重疊率 → :class:`PayloadRequirement.side_overlap`（須在 ``[0, 1)``；否則採規格 §16.2 預設）。
    coverage_altitude
        目標作業高度（AGL, m）→ ``h_target_agl_m``；會被夾到
        ``[max_c spec.min_fw_operation_agl_m, 天花板]``（A 120 m、B 100 m；D12 豁免上限）並記警告。
    terrain_func
        ``f(lat, lon) -> AMSL m``；``None`` → 平地 DEM（見 :func:`terrain_func_to_dem`）。
    dem_path
        **不讀取**（本層無 rasterio 相依）；非空時記警告 ``dem_path_ignored``。
    home
        起降點；``None`` → 第一架 drone 的 ``position``（AeroPlan 於 REPO/ui/main_window.py:4712-4718
        用 ``home_point`` 或多邊形形心，此處以佈機中心代之）。
    fence_zones
        AeroPlan ``FenceZone`` 序列 → :func:`fence_zones_to_nofly`。
    class_of
        ``drone dict -> "A" | "B"``；``None`` → :func:`default_class_of`。
    sensor_a / sensor_b
        逐構型機上硬體；``None`` → A ``(LidarHardware(),)``、B ``(CameraHardware(),)``；
        給了 ``hfov_deg`` 與 ``vfov_deg``（皆 > 0）時改用 :class:`TrapezoidFovSpec`
        （論文 Eq.(2)(3)，須配合 ``cfg.fov_model="trapezoid"`` 才會生效）。
    payload_reqs
        直接指定酬載需求字典（``req_id -> PayloadRequirement``）；``None`` → 由本函式推導單一 ``"default"``。
    alternates
        備降點 ``[(lat, lon) | Site]``；``None`` → 自動於 home 背向區域形心 400 m 處置一點
        （表 1 要求必有備降點，缺則 preflight NO_GO）。
    recovery_sites
        逐機回收站 ``{drone_id 或 uav_id: site_id}``；**明確指定者優先**於下面的預設。
    land_at_takeoff_site
        ``True``（預設）：逐機起飛站生效時（n 台 ⇒ n 個 ``TO_*`` 站），每台預設降回**自己的**起飛站，
        不是全隊擠進同一個 home。逐機起飛站沒生效（單機／座標全相同）時是 no-op。
    terrain_clearance_m
        對地淨空（D11 必填）；``None`` → ``max(36, 3σ_DEM + 30)``。
    dem_margin_m
        DEM 邊界外擴（m）；須大於 hv_dccpp 的 ``terrain_margin_m``（預設 1500）否則地形取樣會落在網格外。
    warnings
        傳入 list 時，本函式把警告字串附加進去。
    mapping_mode
        建圖模式（:class:`hv_dccpp.mission.MappingModeConfig`）→ 回傳 :class:`hv_dccpp.mission.MappingMissionInputs`
        （表 2 ＋ ``mapping_mode``）；``None``（預設）＝ 原本的 ``MissionInputs``，舊行為逐位元不變。其他型別丟
        :class:`AeroPlanAdapterError`（不默默當成沒開）。``lead_in_m`` 要跟匯出器的 camera_lead_in_m 同一個數。
    forward_overlap
        相機前向重疊 → 本函式**自己推導**的 :class:`PayloadRequirement.forward_overlap`（預設 0.70 ＝ 舊行為；
        建圖模式由面板給 0.80）。須在 ``[0, 1)``。``payload_reqs`` 是呼叫端給的時候不推導；此時若改了這個值卻跟
        ``payload_reqs`` 裡的前向重疊不一致 ⇒ :class:`AeroPlanAdapterError`（不默默以哪一個為準）。

    返回
    ----
    ``(MissionInputs, {drone_id: uav_id})``——第二項為 AeroPlan 整數 id 與 hv_dccpp 字串 id 的對照表。
    """
    warn = warnings if warnings is not None else []
    if mapping_mode is not None and not isinstance(mapping_mode, MappingModeConfig):
        raise AeroPlanAdapterError(f"mapping_mode 須為 MappingModeConfig 或 None，得 {type(mapping_mode).__name__}")
    fwd = _check_forward_overlap(forward_overlap)
    if not areas:
        raise AeroPlanAdapterError("areas 不可為空")
    if not drones:
        raise AeroPlanAdapterError("drones 不可為空")
    if dem_path:
        warn.append(f"dem_path_ignored({dem_path})")

    cls_fn = class_of or default_class_of
    id_map: dict[int, str] = {}
    fleet: list[UAVInstance] = []
    specs_used: dict[str, PlatformSpec] = {}
    hw_used: dict[str, tuple[SensorHardware, ...]] = {}
    for d in drones:
        did = int(_get(d, "drone_id", len(id_map) + 1))
        c = str(cls_fn(d)).upper()
        if c not in ("A", "B"):
            raise AeroPlanAdapterError(f"class_of 須回傳 'A' 或 'B'，drone_id={did} 得 {c!r}")
        spec = platform_a if c == "A" else platform_b
        hw = _hardware_for(c, sensor_a, sensor_b, mounting_angle_deg, hfov_deg, vfov_deg)
        uid = f"{c}{did}"
        if uid in id_map.values():
            raise AeroPlanAdapterError(f"drone_id 重複：{did}")
        id_map[did] = uid
        specs_used[c] = spec
        hw_used[c] = hw
        e = dict((energy or {}).get(c) or (DEFAULT_ENERGY_A if c == "A" else DEFAULT_ENERGY_B))
        e.pop("uav_id", None)
        fleet.append(UAVInstance(uid, spec, hw, SortieEnergy(uid, **e)))

    # -- home / 備降點 -------------------------------------------------------
    if isinstance(home, Site):
        home_site = home if home.kind == "home" else replace(home, kind="home")
    else:
        if home is None:
            lat_h, lon_h = _latlon(_get(drones[0], "position", (0.0, 0.0)), "drones[0].position")
        else:
            lat_h, lon_h = _latlon(home, "home")
        home_site = Site("H", lat_h, lon_h, kind="home")
    poly_pts: list[tuple[float, float]] = []
    area_specs: list[AreaSpec] = []

    # -- 酬載需求（先算 h_target，供刈幅核對）--------------------------------
    ceiling = float(waiver_max_agl_m if waiver_max_agl_m is not None else max_agl_m)
    h_minfw = max(float(s.min_fw_operation_agl_m) for s in specs_used.values())
    h_target = float(coverage_altitude)
    h_clamped = min(max(h_target, h_minfw), ceiling)
    if abs(h_clamped - h_target) > 1e-9:
        warn.append(f"coverage_altitude_clamped({h_target:.1f}->{h_clamped:.1f}; "
                    f"min_fw_operation_agl={h_minfw:.0f}, ceiling={ceiling:.0f})")
    h_target = h_clamped

    if payload_reqs is None:
        reqs, req_id, kind_req = _default_payload_reqs(hw_used, overlap_rate, h_target, warn, forward_overlap=fwd)
    else:
        reqs = dict(payload_reqs)
        if abs(fwd - 0.70) > 1e-12:
            bad = {rid: r.forward_overlap for rid, r in reqs.items() if abs(float(r.forward_overlap) - fwd) > 1e-9}
            if bad:
                raise AeroPlanAdapterError(f"forward_overlap={fwd:g} 與 payload_reqs 的前向重疊 {bad} 不一致"
                                           "（給了 payload_reqs 就以它為準；兩邊要給同一個數）")
        req_id = next(iter(reqs))
        kind_req = {}
        for rid, r in reqs.items():
            kind_req.setdefault(r.kind, rid)

    for i, a in enumerate(areas):
        aid = int(_get(a, "area_id", i + 1))
        ring = [_latlon(p, f"areas[{i}].polygon") for p in (_get(a, "polygon") or ())]
        if len(ring) < 3:
            raise AeroPlanAdapterError(f"areas[{i}] 之 polygon 至少 3 點，得 {len(ring)}")
        poly_pts.extend(ring)
        pr = _get(a, "priority", 1.0)
        sreq = str(_get(a, "sensor_req", "any"))
        rid = str(_get(a, "payload_req_id", "") or "")
        if rid not in reqs:
            # 區域指定 rgb / lidar 時挑同 kind 的需求，否則用第一個（"default"）
            rid = kind_req.get(sreq, req_id)
        area_specs.append(AreaSpec(f"Z{aid}", tuple(ring), sensor_req=sreq,
                                   priority=float(pr if pr is not None else 1.0), payload_req_id=rid))

    alt_sites = _resolve_alternates(alternates, home_site, poly_pts, warn)

    # -- DEM ------------------------------------------------------------------
    pts = poly_pts + [home_site.latlon] + [s.latlon for s in alt_sites]
    lats = [p[0] for p in pts]
    lons = [p[1] for p in pts]
    m_lat, m_lon = _metres_per_deg(0.5 * (min(lats) + max(lats)))
    dlat = float(dem_margin_m) / m_lat
    dlon = float(dem_margin_m) / m_lon
    dem = terrain_func_to_dem(terrain_func, (min(lats) - dlat, min(lons) - dlon, max(lats) + dlat, max(lons) + dlon),
                              step_m=dem_step_m, base_msl_m=base_msl_m)

    # -- 刈幅核對（障礙 1）----------------------------------------------------
    for c, hws in hw_used.items():
        w = _nominal_swath_m(hws[0], h_target)
        if w is not None and coverage_width_m > 0.0 and abs(w - float(coverage_width_m)) > 0.25 * float(coverage_width_m):
            warn.append(f"fov_width_mismatch(cls={c}: {w:.0f} m @ {h_target:.0f} m AGL "
                        f"vs UI coverage_width_m={float(coverage_width_m):.0f} m)")

    # -- 空域／風／氣象 --------------------------------------------------------
    sigma = float(dem.sigma_z_m or 0.0)
    tc = float(terrain_clearance_m) if terrain_clearance_m is not None else max(36.0, 3.0 * sigma + 30.0)
    home_msl = float(dem.sample_msl(home_site.lat_deg, home_site.lon_deg)[0])
    if not math.isfinite(home_msl):
        home_msl = float(base_msl_m)
    nofly = fence_zones_to_nofly(fence_zones, home_msl_m=home_msl)
    airspace = Airspace(max_agl_m=float(max_agl_m), waiver_max_agl_m=waiver_max_agl_m,
                        terrain_clearance_m=tc, no_fly=nofly)
    speed, dir_from = (float(wind[0]), float(wind[1]))
    winds = (WindLayer(0.0, 3000.0, speed, dir_from, gust_mps=speed * 1.3, z_ref="agl"),)
    obs = WeatherObs(mean_wind_mps=speed, gust_mps=speed * 1.3, wind_dir_from_deg=dir_from,
                     visibility_m=10000.0, precip_mmh=0.0, temp_c=float(weather_temp_c))
    c2 = C2Link(range_km=float(c2_range_km), coverage_polygon_wgs84=None, ground_station=home_site,
                lost_link_policy={})
    # ── 逐機起飛站（vendored #7）：n 台 ⇒ n 個起降點，比照 DCCPP ────────
    #   ⚠⚠⚠ 為什麼要接：AeroPlan 的 GUI 早就用 `spawn_positions` 把 n 台沿垂直於
    #   起飛方向排開，逐台寫進 `drones[i]['position']`。**DCCPP 直接吃那個欄位**，
    #   所以它的計畫真的有 n 個起飛點；HARVEST 這一側卻整個忽略它、全隊一律用 home
    #   ⇒ 畫面上四台疊在同一個起降點，SITL 起出來也對不上計畫的第一個航點。
    #
    #   ⚠ 只在座標**真的不同**時才動作。`scripts/hv_sitl_scenarios.py` 每一台的
    #     position 都是同一個 HOME 常數 ⇒ 既有 SITL 情境逐位元不變、不受影響。
    #
    #   ⚠⚠ 效能上**幾乎沒有幫助**：起飛站分流的精確天花板實測只有 −52.6 s（−3.6%）。
    #     槓桿在**降落**側（見 vendored #4）。接這一條的價值是「與 DCCPP 對齊、方便起
    #     SITL」，不是壓 makespan —— 修好 vendored #8 之後它其實會**變慢** 7～26%。
    #
    #   ⚠⚠⚠ 這個功能一度不安全，見 vendored #8：同坪豁免比的是場坪**座標**而不是站點
    #     **身分**，兩個相距 < 50 m 的不同站被豁免卻沒被走廊排程互斥 ⇒ 起飛與降落
    #     可以同時發生在相鄰坪上（實測 30 m 水平／2 m 垂直，而 `validate` 回報通過）。
    #     修好後 10 m 間距也合法，最小水平間距 7.5 → 107.2 m。
    #   ⚠ 修前那張「間距越小 makespan 越好」的表是**非單調的雜訊**，不是優化結果。
    _to_sites: list[Site] = []
    _own_pad: dict[str, str] = {}            # uav_id → 該機自己的起飛站 id（下面「回收站」預設用）
    if per_uav_takeoff_sites and len(fleet) > 1:
        _pos: dict[str, tuple[float, float]] = {}
        for d in drones:
            did = int(_get(d, "drone_id", 0))
            uid = id_map.get(did)
            p = _get(d, "position", None)
            if uid is None or not p:
                continue
            try:
                _pos[uid] = (float(p[0]), float(p[1]))
            except Exception:                               # noqa: BLE001
                continue
        _vals = list(_pos.values())
        _spread = 0.0
        for i in range(len(_vals)):
            for j in range(i + 1, len(_vals)):
                dN = (_vals[j][0] - _vals[i][0]) * 111132.0
                dE = ((_vals[j][1] - _vals[i][1]) * 111412.0
                      * math.cos(math.radians(_vals[i][0])))
                _spread = max(_spread, math.hypot(dN, dE))
        if len(_pos) == len(fleet) and _spread > 1.0:
            _by_uid: dict[str, str] = {}
            for u in fleet:
                la, lo = _pos[u.uav_id]
                sid = f"TO_{u.uav_id}"
                _to_sites.append(Site(sid, la, lo, kind="alternate"))
                _by_uid[u.uav_id] = sid
            fleet = [replace(u, start_site_id=_by_uid[u.uav_id]) for u in fleet]
            _own_pad = dict(_by_uid)
            # ⚠⚠ 站點間距 < 50 m 是**三不管地帶**：`deconflict.SAME_PAD_TOL_M = 50`
            #   會讓 4D 套用同坪豁免，而走廊時刻表又因為 site_id 不同而不互斥
            #   ⇒ 兩層保護同時關掉（實測 30 m 坪距下兩機水平只差 30.0 m）。
            #   GUI 的生成點預設間距是 50–60 m，剛好壓在這條線上，所以要說出來。
            _min_gap = float("inf")
            for i in range(len(_vals)):
                for j in range(i + 1, len(_vals)):
                    dN = (_vals[j][0] - _vals[i][0]) * 111132.0
                    dE = ((_vals[j][1] - _vals[i][1]) * 111412.0
                          * math.cos(math.radians(_vals[i][0])))
                    _min_gap = min(_min_gap, math.hypot(dN, dE))
            if warnings is not None:
                warnings.append(f"per_uav_takeoff_sites({len(_to_sites)} 個；"
                                f"最小間距 {_min_gap:.1f} m)")
                if _min_gap < 50.0:
                    # ⚠ 這句在 vendored #8 之前是「兩層保護同時失效」——那個缺陷已修，
                    #   現在 10 m 間距也合法（實測最小水平間距 107.2 m、validate 通過）。
                    #   留著是因為代價變成 makespan：4D 修復必須把起降錯開。
                    warnings.append(
                        f"takeoff_sites_close(最小間距 {_min_gap:.1f} m < 50 m ⇒ "
                        f"起飛與降落走廊要靠 4D 修復錯開，makespan 約 +7～26%；"
                        f"分離本身安全，見 vendored #8)")
        alt_sites = tuple(alt_sites) + tuple(_to_sites)

    # ── 逐機回收站（vendored #4）────────────────────────────────────────
    #   ⚠ 排序陷阱：機隊在上面就組好了，但 home / alt_sites 要到這裡才有合法的
    #     site_id，所以不能在建 UAVInstance 當下傳 —— 在這裡用 replace 補上。
    #   ⚠⚠ 鍵是 hv 的 uav_id（"A1"/"B2"…），不是 AeroPlan 的 drone_id；
    #     `id_map` 提供兩者的對照，兩種鍵都收，認不得的鍵**大聲拒絕**而不是默默忽略
    #     —— 默默忽略會讓「指派了但沒生效」看起來跟「沒指派」一樣。
    #
    #   ⭐⭐ 預設：各機降回**自己的起飛站**（2026-10-09 使用者定調「三架 land 會在同一點而不是起飛點，
    #     這必須修正」）。之前逐機起飛站（vendored #7）只接了**起飛**側，降落仍全部落在 home：
    #     GUI 實測 3 架（坪相距 60 m）全部降在中間那架（UAV-2）的坪，最後進場腿三條疊在同一條線上、
    #     UAV-1／UAV-3 起飛點與降落點差 60 m。降落側的機制早就有（vendored #4），只是沒人預設去用。
    #     明確指定的 ``recovery_sites`` 優先；`land_at_takeoff_site=False` 退回舊行為（全降 home）。
    #   ⚠ 「宣告了偏好」不保證真的落在那裡（`_site_order` 能量吃不下會退回）⇒ 規劃後由
    #     `HarvestCoordinator.plan_coverage_dccpp` 讀回 `UAVPlan.landing_site` 比對並發警告。
    _explicit: dict[str, str] = {}
    if recovery_sites:
        _known = {home_site.site_id} | {s_.site_id for s_ in alt_sites}
        for _k, _v in recovery_sites.items():
            _uid = id_map.get(int(_k)) if str(_k).lstrip("-").isdigit() else str(_k)
            if _uid not in {u.uav_id for u in fleet}:
                raise AeroPlanAdapterError(
                    f"recovery_sites 的鍵 {_k!r} 不是機隊裡的載具（uav_id {sorted(u.uav_id for u in fleet)}"
                    f"／drone_id {sorted(id_map)}）")
            if str(_v) not in _known:
                raise AeroPlanAdapterError(
                    f"recovery_sites[{_k!r}]={_v!r} 不是已知站點 {sorted(_known)}")
            _explicit[_uid] = str(_v)
    _rec_by_uid: dict[str, str] = dict(_own_pad) if land_at_takeoff_site else {}
    _defaulted = sorted(set(_rec_by_uid) - set(_explicit))
    _rec_by_uid.update(_explicit)
    if _rec_by_uid:
        fleet = [replace(u, recovery_site_id=_rec_by_uid.get(u.uav_id)) for u in fleet]
        if warnings is not None:
            warnings.append("per_uav_recovery_sites(" + ",".join(
                f"{k}@{v}" for k, v in sorted(_rec_by_uid.items())) + ")")
            if _defaulted:
                warnings.append("landing_at_takeoff_sites(" + ",".join(_defaulted)
                                + " 各自降回自己的起飛點)")

    kw_in = dict(home=home_site, alternates=alt_sites, areas=tuple(area_specs), dem=dem,
                 obstacles=(), wind=winds, fleet=tuple(fleet), payload_reqs=reqs, c2=c2,
                 airspace=airspace, weather_thresholds=WeatherThresholds(), weather_obs=obs)
    # 建圖模式 → MappingMissionInputs（表 2 + mapping_mode）；沒開就是原本的 MissionInputs
    inputs = MissionInputs(**kw_in) if mapping_mode is None else MappingMissionInputs(**kw_in, mapping_mode=mapping_mode)
    return inputs, id_map


def default_class_of(drone: Mapping[str, Any]) -> str:
    """預設的 drone dict → 構型（``"A"`` = Pioneer PNP、``"B"`` = HERO PNP）判定。

    依序嘗試：``hv_class`` / ``class`` 鍵 → ``platform`` 鍵（含 pioneer → A、hero → B）→
    ``payload`` / ``sensor`` 鍵（lidar → A、rgb/camera → B）→ ``name`` 中「後面緊接數字」的
    ``A`` / ``B``（例如 ``"A1"``、``"UAV-B2"``；``"UAV-1"`` 不匹配）→ 皆無 → ``"A"``。

    AeroPlan 的 UI 以同一組 spinbox 產生所有機（REPO/ui/main_window.py:4722-4744），dict 中並無構型資訊，
    因此**實際整合時應自行提供 ``class_of``**（例如依 ``drone_id`` 奇偶、或新 UI 分頁的逐機設定）。
    """
    for key in ("hv_class", "class"):
        v = _get(drone, key)
        if isinstance(v, str) and v.strip().upper() in ("A", "B"):
            return v.strip().upper()
    plat = str(_get(drone, "platform", "") or "").lower()
    if "pioneer" in plat:
        return "A"
    if "hero" in plat:
        return "B"
    for key in ("payload", "sensor", "sensor_kind"):
        s = str(_get(drone, key, "") or "").lower()
        if "lidar" in s:
            return "A"
        if "rgb" in s or "cam" in s:
            return "B"
    name = str(_get(drone, "name", "") or "").upper()
    m = _NAME_CLS_RE.search(name)
    if m is not None:
        return m.group(1)
    return "A"


def _check_forward_overlap(v: Any) -> float:
    """前向重疊須為 [0, 1) 的有限實數（PayloadRequirement 自己也會擋，這裡先給轉接層的訊息）。"""
    if isinstance(v, bool) or not isinstance(v, (int, float, np.integer, np.floating)) \
            or not math.isfinite(float(v)) or not 0.0 <= float(v) < 1.0:
        raise AeroPlanAdapterError(f"forward_overlap 須為 [0, 1) 的實數，得 {v!r}")
    return float(v)


def _default_payload_reqs(hw_used: Mapping[str, tuple[SensorHardware, ...]], overlap_rate: float,
                          h_target_agl_m: float, warn: list[str], forward_overlap: float = 0.70
                          ) -> tuple[dict[str, PayloadRequirement], str, dict[str, str]]:
    """由機隊硬體推導 :class:`PayloadRequirement` 字典。

    ``forward_overlap``：相機前向重疊（預設 0.70 ＝ 舊行為；建圖模式由面板給，例如 0.80）。

    **每一種機上感測種類都必須有對應 kind 的需求**，否則 ``context.build_sensors`` 不會替該構型建
    :class:`hv_dccpp.platform.Sensor`（``ctx.sensors`` 之鍵為 ``(cls, req_id)``），該構型便完全無法被分配
    （症狀：異構機隊只有一種構型飛）。故本函式在 A（LiDAR）與 B（相機）並存時同時產生
    ``"default"``（主種類）與另一個 ``"default_rgb"`` / ``"default_lidar"``，與
    `scenario.build_mini_example` 的兩個需求同慣例。

    返回 ``(reqs, 主 req_id, {kind: req_id})``。
    """
    kinds = [hw[0].kind for hw in hw_used.values()]
    primary: Literal["lidar", "rgb"] = "lidar" if "lidar" in kinds else "rgb"
    side = float(overlap_rate) if 0.0 <= float(overlap_rate) < 1.0 else None
    if side is None:
        warn.append(f"overlap_rate_out_of_range({overlap_rate}) -> 採規格 §16.2 預設")
    cam = next((h for hws in hw_used.values() for h in hws if isinstance(h, CameraHardware)), None)
    # h_max = GSD·f/p ≥ 1.05·h_target（platform.CameraSensor.agl_window）
    gsd = (1.05 * h_target_agl_m * (cam.pixel_pitch_um * 1e-6) / (cam.focal_length_mm * 1e-3)) if cam else 0.05

    def _mk(rid: str, kind: str) -> PayloadRequirement:
        return PayloadRequirement(rid, kind,  # type: ignore[arg-type]
                                  gsd_required_m=(gsd if kind == "rgb" else None),
                                  density_pt_m2=(20.0 if kind == "lidar" else None),
                                  side_overlap=side, forward_overlap=float(forward_overlap),
                                  h_target_agl_m=h_target_agl_m)

    reqs = {"default": _mk("default", primary)}
    kind_req = {primary: "default"}
    for other in ("lidar", "rgb"):
        if other != primary and other in kinds:
            rid = f"default_{other}"
            reqs[rid] = _mk(rid, other)
            kind_req[other] = rid
    return reqs, "default", kind_req


def _hardware_for(cls: str, sensor_a: Sequence[SensorHardware] | None, sensor_b: Sequence[SensorHardware] | None,
                  mounting_angle_deg: float, hfov_deg: float, vfov_deg: float) -> tuple[SensorHardware, ...]:
    """逐構型機上硬體；給了 hfov/vfov 時改用論文梯形 FOV（Eq.(2)(3)）。"""
    given = sensor_a if cls == "A" else sensor_b
    if given:
        return tuple(given)
    if float(hfov_deg) > 0.0 and float(vfov_deg) > 0.0:
        kind: Literal["lidar", "rgb"] = "lidar" if cls == "A" else "rgb"
        return (TrapezoidFovSpec(kind=kind, mount_angle_deg=float(mounting_angle_deg),
                                 v_fov_deg=float(vfov_deg), h_fov_deg=float(hfov_deg)),)
    return (LidarHardware(),) if cls == "A" else (CameraHardware(),)


def _nominal_swath_m(hw: SensorHardware, h_agl_m: float) -> float | None:
    """該硬體於 ``h_agl_m`` 之標稱地面刈幅（m）；未知型別 → None。僅用於與 UI ``fov_width_m`` 核對。"""
    if isinstance(hw, LidarHardware):
        return 2.0 * h_agl_m * math.tan(math.radians(0.5 * hw.fov_usable_deg))
    if isinstance(hw, CameraHardware):
        return h_agl_m * hw.sensor_width_mm / hw.focal_length_mm
    if isinstance(hw, TrapezoidFovSpec):
        return 2.0 * h_agl_m * math.tan(math.radians(0.5 * hw.h_fov_deg))
    return None


def _resolve_alternates(alternates: Sequence[Any] | None, home: Site, poly_pts: Sequence[tuple[float, float]],
                        warn: list[str]) -> tuple[Site, ...]:
    """備降點：給定值 → Site 元組；未給 → home 背向區域形心 400 m 自動置一點（並記警告）。"""
    if alternates:
        out: list[Site] = []
        for i, a in enumerate(alternates):
            if isinstance(a, Site):
                out.append(a if a.kind == "alternate" else replace(a, kind="alternate"))
            else:
                la, lo = _latlon(a, f"alternates[{i}]")
                out.append(Site(f"ALT{i + 1}", la, lo, kind="alternate"))
        return tuple(out)
    c_lat = sum(p[0] for p in poly_pts) / len(poly_pts)
    c_lon = sum(p[1] for p in poly_pts) / len(poly_pts)
    frame = GeoFrame(home.lat_deg, home.lon_deg, 0.0)
    c = frame.to_enu_xy(c_lat, c_lon)[0]
    d = float(math.hypot(c[0], c[1]))
    ux, uy = (-c[0] / d, -c[1] / d) if d > 1.0 else (0.0, -1.0)
    w = frame.to_wgs84(np.array([ux * 400.0, uy * 400.0, 0.0]))[0]
    warn.append("alternate_auto_placed(ALT1, 400 m from home)")
    return (Site("ALT1", float(w[0]), float(w[1]), kind="alternate"),)


# ---------------------------------------------------------------------------
# 3.2 MissionPlan → AeroPlan 6 鍵字典
# ---------------------------------------------------------------------------

def from_mission_plan(plan: MissionPlan, ctx: MissionContext, *, areas: Sequence[Mapping[str, Any]],
                      drones: Sequence[Mapping[str, Any]], id_map: Mapping[int, str] | None = None,
                      types: HvTypeRegistry | None = None, reduce: bool = True,
                      wp_radius_m: float | None = None) -> dict[str, Any]:
    """:class:`hv_dccpp.mission.MissionPlan` → AeroPlan ``plan_coverage_dccpp`` 之 6 鍵回傳字典。

    回傳鍵（與 REPO/mission/swarm_coordinator.py:1291-1298 完全相同）
    --------------------------------------------------------------
    ``results``
        ``{area_id(int): HvMdtspResult}``
    ``allocation``
        ``{area_id(int): [drone_id(int), ...]}``（對應 ``alloc_res.area_assignments``）
    ``coverage_paths``
        ``[HvCoveragePath, ...]``（每區一個）
    ``uav_states``
        ``[HvUavState, ...]``（順序同 ``drones``）
    ``total_makespan``
        ``float``（**秒**；= ``MissionPlan.makespan_s``。AeroPlan 原路徑於 :1290 是「總長 ÷ 速度」，
        不含起降、轉換、盤旋與錯開起飛，兩者不可逐值比較。）
    ``assembled_paths``
        ``{drone_id(int): HvBuiltPath}``；無任何路徑時為 ``None``（同原型）。

    參數
    ----
    id_map
        ``{drone_id: uav_id}``（:func:`to_mission_inputs` 的第二個回傳值）；``None`` → 以
        ``uav_id`` 尾端數字反推。
    types
        型別註冊表（見 :func:`use_native_types`）；``None`` → :data:`default_types`。
    reduce
        True（預設）→ 以 :func:`hv_dccpp.ardupilot.reduce_waypoints` 把 build_3d 之密集取樣
        （每機數百點）縮成自駕儀導航點（弧段外切離散、直線端點與航向／高度轉折），
        避免超過 ArduPilot 任務項目上限並讓地圖圖層可用；False → 輸出全部取樣點。
    wp_radius_m
        接受半徑（縮點門檻）；``None`` → ``min(R_eff/2, 30)``（規格 §8.5）。
    """
    reg = types or default_types
    cfg = ctx.cfg
    idm = dict(id_map or {})
    if not idm:
        for uid in plan.plans:
            digits = "".join(ch for ch in uid if ch.isdigit())
            if digits:
                idm[int(digits)] = uid
    rev: dict[str, int] = {v: k for k, v in idm.items()}

    # -- assembled_paths ------------------------------------------------------
    assembled: dict[int, Any] = {}
    for uid, up in plan.plans.items():
        did = rev.get(uid)
        if did is None:
            continue
        # 航點 alt 的基準＝規劃 home（全機共用：避撞與顯示要跨機比高度）；飛控解鎖後的 home 是這架的起飛坪，
        # 兩者差 (坪 − home) 記在路徑上，上傳／匯出時才換算（斜坡起降坪時不為 0）
        pad_site, _lat, _lon, pad_msl = fc_home(ctx, up)
        bp = _built_path(up, ctx, cfg, did, reg, reduce=reduce, wp_radius_m=wp_radius_m, home_msl=_home_msl(ctx),
                         alt_ref_site=ctx.inputs.home.site_id, fc_home_site=pad_site, fc_home_msl=pad_msl)
        if bp is not None:
            assembled[did] = bp

    # -- uav_states -----------------------------------------------------------
    uav_states: list[Any] = []
    for d in drones:
        did = int(_get(d, "drone_id", 0))
        uid = idm.get(did)
        pos = _latlon(_get(d, "position", (ctx.inputs.home.lat_deg, ctx.inputs.home.lon_deg)), "drone.position")
        if uid is not None and uid in plan.plans:
            up = plan.plans[uid]
            spec = ctx.specs[up.cls]
            uav = ctx.inputs.uav_by_id(uid)
            w_ref = ctx.wind.max_speed() if ctx.wind is not None else 0.0
            turn_r = float(spec.r_eff(w_ref))
            speed = float(spec.cruise_airspeed_mps)
            h_op = float(np.median([w.agl_m for w in up.waypoints if w.phase is Phase.OPERATE])) \
                if any(w.phase is Phase.OPERATE for w in up.waypoints) else 0.0
            fov = _nominal_swath_m(uav.primary_sensor, h_op) or float(_get(d, "fov_width", 100.0) or 100.0)
            hdg = float(up.waypoints[0].track_deg) if up.waypoints else float(_get(d, "heading", 0.0) or 0.0)
        else:
            turn_r = float(_get(d, "turn_radius", 0.0) or 0.0)
            speed = float(_get(d, "speed", 15.0) or 15.0)
            fov = float(_get(d, "fov_width", 100.0) or 100.0)
            hdg = float(_get(d, "heading", 0.0) or 0.0)
        uav_states.append(_construct(reg.uav_state, uav_id=did, position=pos, heading=_compass(hdg),
                                     turn_radius=turn_r, speed=speed,
                                     vehicle_type=_vehicle_type("FIXED_WING", reg), fov_width=fov))

    # -- coverage_paths / results / allocation --------------------------------
    strip_area: dict[str, str] = {}
    cell_of: dict[str, Any] = {c.cell_id: c for c in plan.cells}
    for sid, st in plan.strips.items():
        cell = cell_of.get(st.cell_id)
        strip_area[sid] = cell.area_id if cell is not None else ""
    # 每機的條帶順序（UAVPlan.strips = [(strip_id, dir), ...]）
    seq_of: dict[str, list[tuple[str, int]]] = {uid: list(up.strips) for uid, up in plan.plans.items()}

    area_ids: list[int] = []
    area_poly: dict[int, list[tuple[float, float]]] = {}
    area_prio: dict[int, float] = {}
    for i, a in enumerate(areas):
        aid = int(_get(a, "area_id", i + 1))
        area_ids.append(aid)
        area_poly[aid] = [_latlon(p, "area.polygon") for p in (_get(a, "polygon") or ())]
        pr = _get(a, "priority", 1.0)
        area_prio[aid] = float(pr if pr is not None else 1.0)

    coverage_paths: list[Any] = []
    results: dict[int, Any] = {}
    allocation: dict[int, list[int]] = {}
    for aid in area_ids:
        key = f"Z{aid}"
        sids = [sid for sid, a in strip_area.items() if a == key]
        headings: list[float] = []
        ops_by_sid: dict[str, Any] = {}
        for sid in sorted(sids):
            st = plan.strips[sid]
            pL = _latlon(_wgs_xy(ctx, st.pL), "strip.pL")
            pR = _latlon(_wgs_xy(ctx, st.pR), "strip.pR")
            if not is_cross_strip(sid):          # 建圖模式交叉航帶不參與「這區的掃描角」（否則比例大時中位數翻轉）
                headings.append(_compass(math.degrees(math.atan2(float(st.pR[0] - st.pL[0]),
                                                                 float(st.pR[1] - st.pL[1])))))
            ops_by_sid[sid] = _construct(reg.operation_segment, index=int(st.index), left_point=pL,
                                         right_point=pR, waypoints=[pL, pR], length_m=float(st.length_m),
                                         covered=False, assigned_drone_id=None)
        assign: dict[int, list[Any]] = {}
        dist: dict[int, float] = {}
        for uid, seq in seq_of.items():
            did = rev.get(uid)
            if did is None:
                continue
            ordered: list[Any] = []
            for sid, dirn in seq:
                if sid not in ops_by_sid:
                    continue
                op = ops_by_sid[sid]
                op.assigned_drone_id = did
                op.covered = True
                if int(dirn) == 1:                       # RL：進入端為 pR（同 idp_solver.py:386-397 慣例）
                    op.left_point, op.right_point = op.right_point, op.left_point
                    op.waypoints = [op.left_point, op.right_point]
                ordered.append(op)
            if ordered:
                assign[did] = ordered
                dist[did] = float(sum(o.length_m for o in ordered))
        if assign:
            allocation[aid] = sorted(assign)
        mk = max((getattr(assembled[d], "makespan_s", 0.0) for d in assign if d in assembled), default=0.0)
        results[aid] = _construct(reg.mdtsp_result, area_id=aid, uav_assignments=assign, path_matrix=None,
                                  decision_matrix=None, total_distances=dist, makespan=float(mk),
                                  assembled_paths={d: assembled[d] for d in assign if d in assembled} or None)
        scan = float(np.median(headings)) if headings else 0.0
        coverage_paths.append(_construct(reg.coverage_path, area_id=aid, area_polygon=area_poly.get(aid, []),
                                         scan_angle_deg=scan,
                                         operations=[ops_by_sid[s] for s in sorted(ops_by_sid)],
                                         priority=area_prio.get(aid, 1.0)))

    return {
        "results": results,
        "allocation": allocation,
        "coverage_paths": coverage_paths,
        "uav_states": uav_states,
        "total_makespan": float(plan.makespan_s),
        "assembled_paths": assembled if assembled else None,
    }


def _home_msl(ctx: MissionContext) -> float:
    """**規劃** home 地面之 MSL 高（m）＝ ``HvBuiltWaypoint.alt`` 的基準。⚠ 飛控的 home 是各機自己的起飛坪
    （``fc_home``）；送飛控前要減掉路徑的 ``fc_alt_offset_m``。"""
    return _site_ground_msl(ctx, ctx.inputs.home.site_id)


def _site_ground_msl(ctx: MissionContext, site_id: str) -> float:
    """站點地面之 MSL 高（m；與 hv_dccpp.ardupilot 同一個換算）。"""
    xy, z = ctx.sites_enu[site_id]
    return float(ctx.frame.enu_z_to_msl(np.asarray(xy, dtype=float).reshape(1, 2), float(z))[0])


def _wgs_xy(ctx: MissionContext, xy: Any) -> tuple[float, float]:
    """ENU ``(x, y)`` → ``(lat, lon)``（z 取 0，供多邊形／條帶端點用）。"""
    p = np.asarray(xy, dtype=float).reshape(2)
    w = ctx.frame.to_wgs84(np.array([[p[0], p[1], 0.0]]))[0]
    return (float(w[0]), float(w[1]))


def _built_path(up: UAVPlan, ctx: MissionContext, cfg: HvPlannerConfig, drone_id: int, reg: HvTypeRegistry, *,
                reduce: bool, wp_radius_m: float | None, home_msl: float, alt_ref_site: str = "",
                fc_home_site: str = "", fc_home_msl: float | None = None) -> Any | None:
    """單機 :class:`UAVPlan` → :class:`HvBuiltPath`（或註冊表中的原生 ``BuiltPath``）。

    ``home_msl``：航點 ``alt`` 的基準 MSL（規劃 home，全機共用）。``fc_home_msl``：這架飛控解鎖後的 home
    （起飛坪；``None`` ⇒ 與 ``home_msl`` 相同）。``alt = max(0, z − 坪) + (坪 − home)``：減掉偏移就是送飛控的
    相對高度（貼地的點夾在坪面而不是 home 面，與 hv 匯出器同一個值）；平地時偏移 0、與舊值逐位元相同。"""
    fc_msl = float(home_msl) if fc_home_msl is None else float(fc_home_msl)
    fc_off = fc_msl - float(home_msl)
    wps = up.waypoints
    if not wps:
        return None
    spec = ctx.specs[up.cls]

    # 五段長度：以**全部**取樣點計算水平長度（縮點只影響輸出航點數，不影響長度統計）
    lengths = {name: 0.0 for name in SEGMENT_NAMES}
    for i in range(len(wps) - 1):
        a, b = wps[i], wps[i + 1]
        lab = PHASE_TO_SEGMENT.get(a.phase, HvSegmentLabel.TRANSFER)
        lengths[lab.name] += math.hypot(b.x_m - a.x_m, b.y_m - a.y_m)

    if reduce:
        from ..ardupilot import accept_radius_m, reduce_waypoints
        w_ref = ctx.wind.max_speed() if ctx.wind is not None else 0.0
        r_acc = accept_radius_m(spec.r_eff(w_ref), wp_radius_m)
        pts = reduce_waypoints(wps, spec, cfg, r_acc)
        xy = np.array([[p.x, p.y] for p in pts], dtype=float)
        z = np.array([p.z for p in pts], dtype=float)
        src = [int(getattr(p, "src", 0)) for p in pts]
    else:
        xy = np.array([[w.x_m, w.y_m] for w in wps], dtype=float)
        z = np.array([w.z_m for w in wps], dtype=float)
        src = list(range(len(wps)))
    if xy.size == 0:
        return None
    wgs = ctx.frame.to_wgs84(np.column_stack([xy, z]))

    out_wps: list[Any] = []
    for k in range(wgs.shape[0]):
        w0 = wps[min(max(src[k], 0), len(wps) - 1)]
        lab = _segment(PHASE_TO_SEGMENT.get(w0.phase, HvSegmentLabel.TRANSFER), reg)
        out_wps.append(_construct(reg.built_waypoint, lat=float(wgs[k, 0]), lon=float(wgs[k, 1]),
                                  alt=max(0.0, float(wgs[k, 2]) - fc_msl) + fc_off,
                                  heading_compass_deg=_compass(w0.track_deg), segment_type=lab,
                                  # vendored #5：把「相機開沒開」帶出來。下游的覆蓋率驗算
                                  # 只剩 segment_type 可用時，會因為兩個規劃器的標籤慣例
                                  # 不同而對 HARVEST 嚴重低估（100.00% → 75.10%）。
                                  sensor_on=bool(getattr(w0, "sensor_on", False))))
    # 降落站地面相對**飛控 home**（起飛坪）（降回自己的起飛坪 ⇒ 0；與 hv 匯出器 NAV_VTOL_LAND 的 z 同一條）
    land_sid = getattr(up, "landing_site", "") or ""
    if land_sid and land_sid == (fc_home_site or alt_ref_site):
        land_alt = 0.0                                      # 降回自己的起飛坪（預設）
    elif land_sid in ctx.sites_enu:
        land_alt = _site_ground_msl(ctx, land_sid) - fc_msl
    else:
        land_alt = 0.0
    return _construct(reg.built_path, uav_id=int(drone_id), waypoints=out_wps,
                      entry_length_m=lengths["ENTRY"], operation_length_m=lengths["OPERATION"],
                      transfer_length_m=lengths["TRANSFER"], takeoff_length_m=lengths["TAKEOFF"],
                      landing_length_m=lengths["LANDING"], hv_uav_id=up.uav_id, cls=up.cls,
                      makespan_s=float(up.t_land_s), alt_ref_site=str(alt_ref_site), alt_ref_msl_m=float(home_msl),
                      fc_home_site=str(fc_home_site or alt_ref_site), fc_home_msl_m=fc_msl, fc_alt_offset_m=fc_off,
                      land_alt_m=float(land_alt))


# ---------------------------------------------------------------------------
# 3.3 上傳 / 匯出
# ---------------------------------------------------------------------------

MAV_CMD: dict[str, int] = {
    "NAV_WAYPOINT": 16, "NAV_LAND": 21, "NAV_TAKEOFF": 22,
    "NAV_VTOL_TAKEOFF": 84, "NAV_VTOL_LAND": 85,
    "DO_CHANGE_SPEED": 178, "DO_VTOL_TRANSITION": 3000,
}
"""AeroPlan 於 REPO/ui/main_window.py:6017-6018 使用的指令碼（與 ``pymavlink.mavutil.mavlink.MAV_CMD_*`` 一致）。

VTOL 三個指令在 AeroPlan 全鏈路皆已支援：白名單 REPO/mission/mission_validator.py:38-51；
frame 選擇 REPO/mission/sitl_link.py:1270-1274（3000 → ``MAV_FRAME_MISSION``，其餘 → ``GLOBAL_RELATIVE_ALT``）。
"""


def to_upload_tuples(built_path: Any, *, is_vtol: bool = True, cruise_speed_mps: float = 15.0,
                     turn_radius_m: float = 50.0, vtol_transition_alt_m: float = 100.0,
                     default_alt_m: float = 100.0, emit_change_speed: bool = True,
                     accept_radius_m: float | None = None) -> list[tuple]:
    """:class:`HvBuiltPath`（或原生 ``BuiltPath``）→ ``SITLLink.upload_mission`` 的 8-tuple 串列。

    餵給 REPO/mission/sitl_link.py:264 ``upload_mission(waypoints)``；該方法接受 3-tuple 或
    ``(lat, lon, alt, cmd, p1, p2, p3, p4)`` 8-tuple（REPO/mission/sitl_link.py:1214-1235）。
    本函式比照 REPO/ui/main_window.py:6045-6159 ``_build_wps_for_path`` 的序列，
    使既有上傳鏈路（:6188-6210 依 ``link.sysid_label`` 派送）行為不變：

    VTOL（QuadPlane）序列::

        DO_CHANGE_SPEED → NAV_VTOL_TAKEOFF(@transition_alt) → DO_VTOL_TRANSITION(4=FW)
        → NAV_WAYPOINT × N
        → NAV_WAYPOINT(@transition_alt) → DO_VTOL_TRANSITION(3=MC) → NAV_VTOL_LAND

    固定翼序列::

        DO_CHANGE_SPEED → NAV_TAKEOFF(pitch 10°) → NAV_WAYPOINT × N → NAV_LAND

    參數
    ----
    built_path
        任何具 ``waypoints``（元素有 ``lat`` / ``lon`` / ``alt`` / ``segment_type`` /
        ``heading_compass_deg``）的物件。
    is_vtol
        True（預設）→ QuadPlane 序列。兩構型皆為 4+1 複合翼，正常情況不應改。
    cruise_speed_mps
        ``DO_CHANGE_SPEED`` 之 param2（param1=0 → 空速；param3=−1 → 不改油門）。
    turn_radius_m
        該機的有效轉彎半徑 R_eff（幾何量）。**不再**直接寫進 ``NAV_WAYPOINT`` 的 param2：
        ArduPlane 把非零的 param2（整數公尺）當**該航點的接受半徑**（ArduPlane_commands_logic.cpp
        verify_nav_wp），舊版把 R_eff（Pioneer ≈200 m、HERO ≈120 m）填進去 ⇒ 條帶尾端提早最多 200 m 被判到達。
        param2 改用 hv 匯出器同一條 :func:`hv_dccpp.ardupilot.accept_radius_m` ＝ min(R_eff/2, 30) m。
    accept_radius_m
        直接指定 ``NAV_WAYPOINT`` param2（接受半徑，m）；``None``（預設）＝ 由 ``turn_radius_m`` 推導如上。
    vtol_transition_alt_m
        MC↔FW 轉換高度（AGL 相對 home）。AeroPlan 由 ``parameter_panel.vtol_transition_alt_spin`` 取得（:6030）。
    default_alt_m
        航點 ``alt`` 缺值時的替代高度。
    emit_change_speed
        False → 不輸出開頭的 ``DO_CHANGE_SPEED``。

    返回
    ----
    ``[(lat, lon, alt, cmd, p1, p2, p3, p4), ...]``；``alt`` 為 **AGL 相對 home**
    （``sitl_link`` 對 NAV 指令一律用 ``MAV_FRAME_GLOBAL_RELATIVE_ALT``）。home ＝ 這架的起飛坪（飛控解鎖時重設），
    :func:`from_mission_plan` 的路徑已以它為基準；降落點高度取路徑的 ``land_alt_m``（降回自己的起飛坪 ⇒ 0）。
    ``DO_*`` 項的 lat/lon/alt 皆為 0（該處 frame 為 ``MAV_FRAME_MISSION``）。

    與 AeroPlan 原實作的**唯一差異**（刻意；記於 CONTRACT_DEVIATIONS.md）
    ------------------------------------------------------------------
    REPO/ui/main_window.py:6113-6115 對 ``seg_name == 'TAKEOFF'`` 且 ``idx == 0`` 一律 ``continue``；
    若路徑只有**一個** TAKEOFF 航點（正是 hv_dccpp 縮點後的情形——``NAV_VTOL_TAKEOFF`` 只需一點），
    ``takeoff_emitted`` 永遠為 False，整個任務就**沒有起飛指令**（飛控會直接從第一個 NAV_WAYPOINT 開始）。
    本函式在 ``idx == 0`` 且尚未輸出起飛指令時照樣輸出 ``NAV_VTOL_TAKEOFF`` + ``DO_VTOL_TRANSITION(4)``，
    確保「VTOL 起降指令齊全」。其餘序列逐項相同。

    另一個刻意差異（bug fix，2026-10-14）：``NAV_WAYPOINT`` param2 是接受半徑 min(R_eff/2, 30) m，
    不再是轉彎半徑（見參數 ``turn_radius_m``）。
    """
    CMD_WP, CMD_TO, CMD_LAND, CMD_SPD = MAV_CMD["NAV_WAYPOINT"], MAV_CMD["NAV_TAKEOFF"], MAV_CMD["NAV_LAND"], MAV_CMD["DO_CHANGE_SPEED"]
    CMD_VTOL_TO, CMD_VTOL_LAND, CMD_VTOL_TRANS = MAV_CMD["NAV_VTOL_TAKEOFF"], MAV_CMD["NAV_VTOL_LAND"], MAV_CMD["DO_VTOL_TRANSITION"]
    from ..ardupilot import accept_radius_m as _accept_radius
    out: list[tuple] = []
    wp_list = list(getattr(built_path, "waypoints", None) or [])
    if not wp_list:
        return out
    # NAV_WAYPOINT param2 ＝ 接受半徑（不是轉彎半徑；見 docstring）
    r_acc = float(accept_radius_m) if accept_radius_m is not None else _accept_radius(float(turn_radius_m))
    # 降落點高度：降落站地面相對飛控 home（＝起飛坪）；降回自己的起飛坪 ⇒ 0（舊行為）。沒有這個欄位的原生路徑 ⇒ 0
    land_alt = float(getattr(built_path, "land_alt_m", 0.0) or 0.0)
    # 路徑 alt 的基準是規劃 home；飛控 home 是這架的起飛坪 ⇒ 送出前減掉 (坪 − home)。原生路徑沒有這欄 ⇒ 0
    fc_off = float(getattr(built_path, "fc_alt_offset_m", 0.0) or 0.0)
    last_landing_idx = -1
    has_takeoff_seg = False
    for i, w in enumerate(wp_list):
        nm = _enum_name(getattr(w, "segment_type", ""))
        if nm == "LANDING":
            last_landing_idx = i
        if nm == "TAKEOFF":
            has_takeoff_seg = True

    if emit_change_speed:
        out.append((0.0, 0.0, 0.0, CMD_SPD, 0.0, float(cruise_speed_mps), -1.0, 0.0))

    takeoff_emitted = False
    if not has_takeoff_seg:
        w0 = wp_list[0]
        _a0 = getattr(w0, "alt", None)
        alt0 = (float(_a0) - fc_off) if _a0 else float(default_alt_m)
        if alt0 < 20.0:
            alt0 = max(float(default_alt_m), 50.0)
        if is_vtol:
            out.append((float(w0.lat), float(w0.lon), float(vtol_transition_alt_m), CMD_VTOL_TO, 0.0, 0.0, 0.0, 0.0))
            out.append((0.0, 0.0, 0.0, CMD_VTOL_TRANS, 4.0, 0.0, 0.0, 0.0))
        else:
            out.append((float(w0.lat), float(w0.lon), alt0, CMD_TO, 10.0, 0.0, 0.0, 0.0))
        takeoff_emitted = True

    for idx, wp in enumerate(wp_list):
        seg = _enum_name(getattr(wp, "segment_type", ""))
        lat = getattr(wp, "lat", None)
        lon = getattr(wp, "lon", None)
        if lat is None or lon is None:
            continue
        lat, lon = float(lat), float(lon)
        _alt = getattr(wp, "alt", None)
        alt = (float(_alt) - fc_off) if _alt else float(default_alt_m)
        hdg = float(getattr(wp, "heading_compass_deg", getattr(wp, "heading_deg", 0.0)) or 0.0)
        if seg == "TAKEOFF":
            if idx == 0 and not takeoff_emitted:
                if is_vtol:
                    out.append((lat, lon, float(vtol_transition_alt_m), CMD_VTOL_TO, 0.0, 0.0, 0.0, 0.0))
                    out.append((0.0, 0.0, 0.0, CMD_VTOL_TRANS, 4.0, 0.0, 0.0, 0.0))
                else:
                    out.append((lat, lon, alt, CMD_TO, 10.0, 0.0, 0.0, 0.0))
                takeoff_emitted = True
            elif idx == 0:
                continue
            elif not takeoff_emitted:
                if is_vtol:
                    out.append((lat, lon, float(vtol_transition_alt_m), CMD_VTOL_TO, 0.0, 0.0, 0.0, 0.0))
                    out.append((0.0, 0.0, 0.0, CMD_VTOL_TRANS, 4.0, 0.0, 0.0, 0.0))
                else:
                    out.append((lat, lon, alt, CMD_TO, 10.0, 0.0, 0.0, 0.0))
                takeoff_emitted = True
            else:
                out.append((lat, lon, alt, CMD_WP, 0.0, r_acc, 0.0, 0.0))
        elif seg == "LANDING":
            if idx == last_landing_idx:
                if is_vtol:
                    out.append((lat, lon, float(vtol_transition_alt_m), CMD_WP, 0.0, 0.0, 0.0, 0.0))
                    out.append((0.0, 0.0, 0.0, CMD_VTOL_TRANS, 3.0, 0.0, 0.0, 0.0))
                    out.append((lat, lon, land_alt, CMD_VTOL_LAND, 0.0, 0.0, 0.0, hdg))
                else:
                    out.append((lat, lon, land_alt, CMD_LAND, 0.0, 0.0, 0.0, hdg))
            else:
                out.append((lat, lon, alt, CMD_WP, 0.0, r_acc, 0.0, 0.0))
        else:
            out.append((lat, lon, alt, CMD_WP, 0.0, r_acc, 0.0, 0.0))
    return out


def qgc_wpl_lines(items: Sequence[MissionItem], *, header: bool = True) -> list[str]:
    """:class:`hv_dccpp.ardupilot.MissionItem` 串列 → QGC WPL 110 文字行。

    格式與 REPO/utils/file_io.py:222 ``create_waypoint_line`` **完全一致**（tab 分隔、12 欄、
    ``lat:.6f`` ``lon:.6f`` ``alt:.2f``），欄序為::

        seq  current  frame  command  p1  p2  p3  p4  lat  lon  alt  autocontinue

    ``header=True`` 時第一行為 ``"QGC WPL 110"``（REPO/utils/file_io.py:157 之寫檔慣例）。
    """
    lines: list[str] = ["QGC WPL 110"] if header else []
    for it in items:
        lines.append(f"{int(it.seq)}\t{int(it.current)}\t{int(it.frame)}\t{int(it.command)}\t"
                     f"{float(it.param1)}\t{float(it.param2)}\t{float(it.param3)}\t{float(it.param4)}\t"
                     f"{float(it.x):.6f}\t{float(it.y):.6f}\t{float(it.z):.2f}\t{int(it.autocontinue)}")
    return lines


def plan_to_aeroplan_waypoints(plan: MissionPlan, ctx: MissionContext, cfg: HvPlannerConfig | None = None, *,
                               header: bool = True, camera_trigger: bool = True,
                               id_map: Mapping[int, str] | None = None,
                               key: Literal["uav_id", "drone_id"] = "uav_id",
                               camera_cfg: CameraTriggerConfig | None = None) -> dict[str, list[str]]:
    """每機 QGC WPL 110 文字行（可直接寫成 AeroPlan 的 ``.waypoints`` 檔）。

    項目由 :func:`hv_dccpp.ardupilot.plan_to_mission_items` 產生（``alt_frame="relative"`` →
    ``alt = MSL − home MSL``，對應 frame 3），含 QuadPlane 的 ``NAV_VTOL_TAKEOFF`` /
    ``DO_VTOL_TRANSITION`` / ``NAV_VTOL_LAND``、``DO_CHANGE_SPEED``、以及 LiDAR/相機的
    ``DO_SET_SERVO`` / ``DO_SET_CAM_TRIGG_DIST``（AeroPlan 的 ``MAVCommand`` 白名單見
    REPO/mission/waypoint.py:46-49、REPO/mission/mission_validator.py:38-51）。

    行格式與 REPO/utils/file_io.py:222 逐位元相同（見 :func:`qgc_wpl_lines`），因此
    REPO/utils/file_io.py:129/185 的讀取器可原樣解析、REPO/ui/main_window.py:5213 的匯出流程可直接沿用。

    參數
    ----
    key
        ``"uav_id"``（預設）→ 鍵為 hv_dccpp 的字串 id（如 ``"A1"``）；``"drone_id"`` → 鍵為
        AeroPlan 整數 id 的字串形式（需給 ``id_map``）。
    camera_cfg
        :class:`hv_dccpp.ardupilot.CameraTriggerConfig`，原樣交給 ``plan_to_mission_items``（206 的 p3、引入段 NAV、
        地形重發）；``None`` ＝ 舊行為。建圖模式要給**上傳用的同一份**，匯出才會等於上傳。

    返回
    ----
    ``{key: [行, ...]}``。
    """
    cfg = cfg if cfg is not None else ctx.cfg
    rev: dict[str, int] = {v: k for k, v in dict(id_map or {}).items()}
    out: dict[str, list[str]] = {}
    for uid, up in plan.plans.items():
        items = plan_to_mission_items(up, ctx, cfg, alt_frame="relative", camera_trigger=camera_trigger,
                                      camera_cfg=camera_cfg)
        name = uid if key == "uav_id" or uid not in rev else str(rev[uid])
        out[name] = qgc_wpl_lines(items, header=header)
    return out


def mission_item_dicts(items: Sequence[MissionItem]) -> list[dict[str, Any]]:
    """:class:`hv_dccpp.ardupilot.MissionItem` 串列 → ``SITLLink.upload_mission_items`` 吃的 dict 串列。

    形狀與 REPO/mission/sitl_link.py ``_fetch_mission_items``（讀回）相同：
    ``seq/command/frame/lat/lon/alt/param1-4/current/autocontinue``；seq 0 是 Home 槽（飛控不採用其內容，
    但要合法——hv 匯出器的第 0 列就是 home，frame 0、alt = home MSL）。``x/y/z`` → ``lat/lon/alt``，逐值不改。
    """
    return [{"seq": int(it.seq), "command": int(it.command), "frame": int(it.frame),
             "lat": float(it.x), "lon": float(it.y), "alt": float(it.z),
             "param1": float(it.param1), "param2": float(it.param2), "param3": float(it.param3),
             "param4": float(it.param4), "current": int(it.current), "autocontinue": int(it.autocontinue)}
            for it in items]


#: 帶座標、會「抵達」的 NAV 指令（206 的開關在抵達前一個 NAV 時執行）
_PHOTO_NAV_CMDS = frozenset({16, 17, 18, 19, 21, 22, 31, 84, 85})


def _ground_dist_m(a: tuple[float, float], b: tuple[float, float]) -> float:
    """兩點水平距離（WGS84 子午／卯酉曲率半徑的局部平面近似；條帶尺度內誤差 < 0.1%）。"""
    lat = math.radians(0.5 * (a[0] + b[0]))
    e2 = 6.69437999014e-3
    w = math.sqrt(1.0 - e2 * math.sin(lat) ** 2)
    m_rad = 6378137.0 * (1.0 - e2) / w ** 3
    n_rad = 6378137.0 / w
    dn = math.radians(b[0] - a[0]) * m_rad
    de = math.radians(b[1] - a[1]) * n_rad * math.cos(lat)
    return math.hypot(dn, de)


#: AP_Camera 的更新率（AP_Camera::update 由 scheduler 以 50 Hz 呼叫；距離觸發只在這些 tick 檢查）
CAMERA_UPDATE_HZ: float = 50.0


def _nav_accept_m(it: Mapping[str, Any]) -> float:
    """該 NAV 的接受距離（m）：NAV_WAYPOINT 取 ``int(param2)``（ArduPlane LOWBYTE，≤ 255）；其他 NAV 與 param2 = 0 → 0。"""
    if int(it["command"]) != 16:
        return 0.0
    try:
        p2 = float(it.get("param2", 0.0) or 0.0)
    except (TypeError, ValueError):
        return 0.0
    return float(min(int(max(p2, 0.0)), 255)) if math.isfinite(p2) else 0.0


def expected_photo_count(items: Sequence[Mapping[str, Any]], *, speed_mps: float | None = None,
                         intrval_min_s: float = 0.0, tick_hz: float = CAMERA_UPDATE_HZ) -> int:
    """依任務項目（:func:`mission_item_dicts` 的 dict）**估**這架會拍幾張。

    模型（與 core.recon.exposure_verifier.simulate_mission_photos 同一套 AP_Camera／AP_Mission 語意，但不跑逐 tick 模擬）：

    * 206（DO_SET_CAM_TRIGG_DIST）在「抵達它前一個 NAV」時執行；抵達點 = 該 NAV 前 ``int(param2)`` 公尺
      （NAV_WAYPOINT 的接受距離；其他 NAV 為 0），且不早於前一個 NAV 的抵達點。相機開著的距離就是兩個抵達點之間的
      折線長 —— 引入段 NAV（param2 = 1）與帶終點 NAV（param2 = 30）的差會讓有效帶長短 29 m。
    * 50 Hz tick：距離觸發只在 tick 上檢查「離上一張 ≥ d」⇒ 實際間距 = ceil(d / (v·Δt))·v·Δt（Δt = 1/``tick_hz``）；
      R2 實測 HERO 19 m/s、d = 18.848 ⇒ 19.0 m（每條 1.15 km 的帶少 1 張）。v = 任務裡的 DO_CHANGE_SPEED 空速
      （param1 = 0）；``speed_mps`` 給了就整份用它。無風：地速 ＝ 空速。
    * ``intrval_min_s``（CAM1_INTRVAL_MIN）：觸發早於它時延後到下一個允許的 tick ⇒ 間距至少 ceil(t_min/Δt)·v·Δt。
    * 每條帶：p3 = 1 開啟即拍一張；之後每走一個間距一張；條帶中途的 206(d', 0, 0, 0)（地形重發）以新的 d 接續累積距離。
      任何 p3 = 1 的 206（包括 p1 = 0 的關）都會立即拍一張——ArduPlane 的行為（AP_Mission_Commands.cpp 4.5.7:138-142）。

    沒有任何 DO_CHANGE_SPEED 空速、也沒給 ``speed_mps`` ⇒ 無法量化 tick，退回連續模型（間距 = d）並記 WARNING
    （可能每條帶多估 1 張）。仍不含：轉彎切角、CAM_MAX_ROLL 延後、風、10 Hz 任務更新造成的 ≤ 0.1 s 開／關延遲
    （對帶長 ±2 m ⇒ 個別條帶 ±1 張，無系統偏差）。
    """
    if speed_mps is not None:
        sp = float(speed_mps)
        if not (math.isfinite(sp) and sp > 0.0):
            raise ValueError(f"speed_mps 須為正的有限值（m/s），得 {speed_mps!r}")
    t_min = float(intrval_min_s)
    if not (math.isfinite(t_min) and 0.0 <= t_min <= 10.0):
        raise ValueError(f"intrval_min_s（CAM1_INTRVAL_MIN）須在 0–10 s，得 {intrval_min_s!r}")
    hz = float(tick_hz)
    if not (math.isfinite(hz) and hz > 0.0):
        raise ValueError(f"tick_hz 須為正的有限值，得 {tick_hz!r}")
    dt = 1.0 / hz
    v: float | None = float(speed_mps) if speed_mps is not None else None
    warned = False

    def spacing(d: float) -> float:
        nonlocal warned
        if v is None:
            if not warned:
                _log.warning(
                    "expected_photo_count：任務沒有 DO_CHANGE_SPEED 空速、也沒給 speed_mps ⇒ 不做 %g Hz tick 量化，"
                    "間距用 d 本身（每條帶可能多估 1 張）%s", hz,
                    "；CAM1_INTRVAL_MIN 也無法換算成距離" if t_min > 0 else "")
                warned = True
            return d
        step = v * dt
        m = max(1, math.ceil(d / step - 1e-9))
        if t_min > 0.0:
            m = max(m, math.ceil(t_min / dt - 1e-9))
        return m * step

    count = 0
    d = 0.0                 # 0 ＝ 相機關
    acc = 0.0               # 上一張之後走過的距離
    s = 0.0                 # 航線累積水平距離（到最近一個 NAV 頂點）
    s_cam = 0.0             # 相機狀態已推進到的位置（最近一個抵達點）
    reach_prev = 0.0
    prev: tuple[float, float] | None = None
    for it in items:
        cmd = int(it["command"])
        if int(it.get("seq", 1)) == 0:                      # Home 槽：航線起點
            lat, lon = float(it.get("lat", 0.0) or 0.0), float(it.get("lon", 0.0) or 0.0)
            if not (lat == 0.0 and lon == 0.0):
                prev = (lat, lon)
            continue
        if cmd in _PHOTO_NAV_CMDS:
            lat, lon = float(it["lat"]), float(it["lon"])
            if lat == 0.0 and lon == 0.0:
                continue
            here = (lat, lon)
            if prev is not None:
                s += _ground_dist_m(prev, here)
            prev = here
            reach = max(reach_prev, s - _nav_accept_m(it))  # 抵達點：其後的 DO_ 在這裡執行
            reach_prev = reach
            if d > 0.0 and reach > s_cam:
                acc += reach - s_cam
                q = spacing(d)
                n = int(math.floor(acc / q + 1e-9))
                count += n
                acc -= n * q
            s_cam = max(s_cam, reach)
        elif cmd == 178:
            if speed_mps is None and float(it.get("param1", 0.0) or 0.0) == 0.0:
                p2 = float(it.get("param2", 0.0) or 0.0)
                if math.isfinite(p2) and p2 > 0.0:
                    v = p2                                  # 空速型 DO_CHANGE_SPEED（無風：地速 ＝ 空速）
        elif cmd == 206:
            dist = float(it["param1"])
            if dist > 0.0:
                if d <= 0.0:                                # 開：距離從這裡起算
                    acc = 0.0
                d = dist
            else:
                d, acc = 0.0, 0.0                           # 關
            if float(it["param3"]) == 1.0:
                # ArduPlane：p3 = 1 一律立即拍一張（開、地形重發、甚至關都拍；AP_Mission_Commands.cpp 4.5.7:138-142）
                count += 1
                acc = 0.0
    return count


# ---------------------------------------------------------------------------
# 3.4 遙測 → 動態事件
# ---------------------------------------------------------------------------

def telemetry_frames_to_events(frames: Iterable[Any], plan: MissionPlan, ctx: MissionContext,
                               cfg: HvPlannerConfig, *, id_map: Mapping[int, str] | None = None,
                               uav_id_of: Callable[[Any], str | None] | None = None,
                               t0_epoch_s: float | None = None,
                               link_timeout_s: float = 5.0, now_s: float | None = None,
                               last_link_ok: Mapping[str, bool] | None = None,
                               return_telemetry: bool = False) -> list[Any] | tuple[list[Any], dict[str, Any]]:
    """AeroPlan ``FleetRegistry.snapshot()`` 的 ``TelemetryFrame`` → hv_dccpp 動態事件。

    來源：REPO/mission/fleet_registry.py:24 ``FleetRegistry``（單例 :49、``snapshot()`` :106、
    ``latest_by_sysid()`` :101、``latest_age_sec()`` :121）；``TelemetryFrame`` 定義於
    REPO/mission/sitl_link.py:30-62。以 duck typing 讀 ``sysid`` / ``lat`` / ``lon`` / ``alt_msl`` /
    ``alt_rel`` / ``heading`` / ``ground_speed`` / ``battery_pct`` / ``last_update``（另讀 ``air_speed`` /
    ``armed`` / ``mode`` 若有）。

    對應規則（規格 §15.5.2；實作沿用 :func:`hv_dccpp.mavlink_link.to_planner_telemetry` 與
    :func:`hv_dccpp.mavlink_link.detect_events`）：

    * WGS84 + AMSL → ENU（``ctx.frame.to_enu``，D42）；θ 取航跡（地速 ≥ 3 m/s）否則機首方位；
    * 能量：``battery_pct/100 × E_nom``（E_nom 取該機 EnergyModel 標稱）→ 與計畫能量偏差
      > ``cfg.delta_1 × E_nom`` 時發 :class:`hv_dccpp.dynamic.EnergyUpdate`；
    * 鏈路：``now − last_update > link_timeout_s`` → ``link_ok=False`` → :class:`LinkLoss`；
      恢復 → :class:`LinkRestored`（附該筆 :class:`hv_dccpp.dynamic.Telemetry`）。

    **限制**：``TelemetryFrame`` 沒有 ``landed_state``、``mission_seq`` 與風速欄位，故不產生
    ``UAVExit(landed_unexpected)`` 與 ``WindUpdate``；條帶進度改以計畫時間軸估計（誤差較 MISSION_CURRENT 大）。
    需要完整規則請改走 :class:`hv_dccpp.mavlink_link.FleetLink`（自建 MAVLink 連線）。

    參數
    ----
    id_map
        ``{drone_id/sysid: uav_id}``；AeroPlan 以 ``sysid = drone_id``（REPO/mission/sitl_launcher.py:431
        ``sysid=i+1``、REPO/ui/main_window.py:6188-6210 依 ``link.sysid_label`` 派送）。
    uav_id_of
        自訂對照（優先於 ``id_map``）；回 ``None`` 表示略過該筆。
    t0_epoch_s
        任務起始 epoch 秒；``None`` → 取全部 frame 之最小 ``last_update``。
        事件時間 ``t_s = last_update − t0_epoch_s``。
    last_link_ok
        ``{uav_id: 上一輪 link_ok}``；未給則視為 True（首輪失聯即發 LinkLoss）。
    return_telemetry
        True → 回 ``(events, {uav_id: Telemetry})``，方便直接餵給
        :func:`hv_dccpp.dynamic.replan` 的 ``telemetry`` 參數。

    返回
    ----
    ``list[Event]``（或 ``(events, telemetry_dict)``）。
    """
    from ..mavlink_link import VehicleTelemetry, detect_events, to_planner_telemetry

    idm = dict(id_map or {})
    fl = list(frames or [])
    if not fl:
        return ([], {}) if return_telemetry else []
    ups = [float(_get(f, "last_update", 0.0) or 0.0) for f in fl]
    t0 = float(t0_epoch_s) if t0_epoch_s is not None else min(ups)
    now = float(now_s) if now_s is not None else max(ups)
    events: list[Any] = []
    tels: dict[str, Any] = {}
    for f, up_t in zip(fl, ups):
        uid: str | None
        if uav_id_of is not None:
            uid = uav_id_of(f)
        else:
            uid = idm.get(int(_get(f, "sysid", 0) or 0))
        if uid is None or uid not in plan.plans:
            continue
        pct = _get(f, "battery_pct", None)
        pct_f = None if pct is None or float(pct) < 0.0 else float(pct)
        air = _get(f, "air_speed", None)
        vt = VehicleTelemetry(
            t_s=up_t - t0,
            lat_deg=float(_get(f, "lat", 0.0) or 0.0), lon_deg=float(_get(f, "lon", 0.0) or 0.0),
            alt_msl_m=float(_get(f, "alt_msl", 0.0) or 0.0), rel_alt_m=float(_get(f, "alt_rel", 0.0) or 0.0),
            heading_deg=float(_get(f, "heading", 0.0) or 0.0),
            groundspeed_mps=float(_get(f, "ground_speed", 0.0) or 0.0),
            airspeed_mps=None if air is None or float(air) <= 0.0 else float(air),
            battery_pct=pct_f, battery_wh_remaining=None, wind_speed_mps=None, wind_dir_from_deg=None,
            vtol_state=0, landed_state=0, mission_seq=0, mode=str(_get(f, "mode", "") or ""),
            armed=bool(_get(f, "armed", False)), link_ok=(now - up_t) <= float(link_timeout_s),
            track_deg=None, sysid=int(_get(f, "sysid", 0) or 0), t_epoch_s=up_t)
        ptel = to_planner_telemetry(vt, plan, ctx, uid)
        tels[uid] = ptel
        prev_ok = True if last_link_ok is None else bool(last_link_ok.get(uid, True))
        prev = replace(ptel, link_ok=prev_ok)
        events.extend(detect_events(ptel, plan, ctx, cfg, prev, uav_id=uid))
    return (events, tels) if return_telemetry else events


# ===========================================================================
# 4. 主接縫：HvDccppCoordinator
# ===========================================================================

class HvDccppCoordinator:
    """可取代 ``mission.swarm_coordinator.SwarmCoordinator`` 的 DCCPP 部分。

    在 AeroPlan 端只需把 REPO/ui/main_window.py:4754 的::

        coordinator = SwarmCoordinator()

    換成::

        from hv_dccpp.integration.aeroplan_adapter import HvDccppCoordinator
        coordinator = HvDccppCoordinator(class_of=lambda d: 'A' if d['drone_id'] % 2 else 'B')

    其餘（:4760-4776 的呼叫、:5014 的 ``self._dccpp_result``、以及全部下游）**一字不改**。
    建議在 QThread 內呼叫（見模組 docstring 之 ``HvPlanningWorker`` 範例）。

    參數
    ----
    platform_a / platform_b
        構型 A（Pioneer PNP，LiDAR）／B（HERO PNP，相機）之 :class:`hv_dccpp.platform.PlatformSpec`
        （**數值取自 docs/平台參數表.md，不可改**）。
    class_of
        ``drone dict -> "A" | "B"``；``None`` → :func:`default_class_of`
        （AeroPlan 的 drone dict 無構型資訊，實務上應自行提供，見障礙 1/2）。
    sensor_a / sensor_b
        逐構型機上硬體序列（主感測器在前）；``None`` → A ``(LidarHardware(),)``、B ``(CameraHardware(),)``。
    payload_reqs
        ``{req_id: PayloadRequirement}``；``None`` → 由 ``coverage_width_m`` / ``overlap_rate`` /
        ``coverage_altitude`` 推導單一 ``"default"``。
    cfg
        :class:`hv_dccpp.mission.PlannerConfig`（``from hv_dccpp import PlannerConfig as HvPlannerConfig``；
        AeroPlan 內有兩個同名 ``PlannerConfig``——REPO/core/base/planner_base.py:70 與
        REPO/core/planning/heterogeneous_nfz_planner.py:80——**務必用別名匯入**）。
        ``None`` → ``HvPlannerConfig(dem_step_m=25.0)``。
    fence_zones
        AeroPlan ``FenceZone`` 序列（duck typing）；取自
        ``FenceZoneRegistry.instance().by_category(ZoneCategory.NFZ)``（REPO/mission/fence_zone.py:181, :220）。
    alternates
        備降點 ``[(lat, lon) | Site]``；``None`` → 自動置一點（表 1 要求必有，否則 preflight NO_GO）。
    wind
        ``(speed_mps, dir_from_deg)`` 單層風（AGL 0–3000 m）；``None`` → 無風。
    c2_range_km
        C2 資料鏈路半徑（表 2 必填，D11）。
    terrain_clearance_m
        對地淨空（D11 必填）；``None`` → ``max(36, 3σ_DEM + 30)``。
    waiver_max_agl_m
        豁免後 AGL 上限（D12）；``None`` → 只用 120 m（構型 A 之 ``min_fw_operation_agl_m`` 為 120 m，
        平地將無高度帶可用 → 建議保留預設 200）。
    dem_step_m / dem_margin_m
        由 ``terrain_func`` 取樣建 DEM 的節點距與邊界外擴（後者須 ≥ hv_dccpp 的 ``terrain_margin_m`` 1500 m）。
    progress
        ``f(fraction: float, message: str)``；於 build_context / preflight / plan_mission / 轉換四階段回報。
    on_report
        ``f(payload: dict)``；規劃結束時以 ``{"status", "errors", "warnings", "notes",
        "makespan_s", "timing_ms", "uncovered"}`` 呼叫一次，供 GUI 顯示（AeroPlan 已有
        ``_dccpp_metrics`` 欄位，REPO/ui/main_window.py:5046-5054）。建圖模式時另含 ``"mapping_mode"``。
    mapping_mode
        建圖模式 :class:`hv_dccpp.mission.MappingModeConfig`（接縫共用重疊、不分層、偏流餘裕、引入段、碎區塊警告）；
        ``None``（預設）＝ 舊行為逐位元不變。經 :func:`to_mission_inputs` 變成 ``MappingMissionInputs.mapping_mode``。
        有效值記在 ``notes`` 的 ``mapping_mode_effective(...)``、``on_report`` 的 ``"mapping_mode"`` 與
        :attr:`mapping_effective`；匯出器的 camera_lead_in_m 請直接用 ``self.mapping_mode.lead_in_m``。
    forward_overlap
        轉接層**自己推導** PayloadRequirement 時用的前向重疊（預設 0.70 ＝ 舊行為）；見 :func:`to_mission_inputs`。
    camera_cfg
        這份計畫上傳／匯出時用的 :class:`hv_dccpp.ardupilot.CameraTriggerConfig`（只是存放，規劃不讀它）。
        與 ``mapping_mode`` 同給時，``camera_lead_in_m`` 必須等於 ``mapping_mode.lead_in_m``，否則丟例外
        （規劃器把條帶外延 lead_in_m、匯出器在帶起點後 camera_lead_in_m 才開相機——兩個數不同就會掉覆蓋）。

    屬性
    ----
    last_inputs / last_ctx / last_plan / last_result / notes / id_map
        最近一次規劃的中間物件與警告，供除錯與 A/B 比較。
    mapping_effective
        建圖模式最近一次規劃的有效值（``{"config", "per_class", "seam_by_area"}``）；未開建圖模式 → ``None``。
    """

    def __init__(self, *, platform_a: PlatformSpec = PLATFORM_PIONEER, platform_b: PlatformSpec = PLATFORM_HERO,
                 class_of: Callable[[Mapping[str, Any]], str] | None = None,
                 sensor_a: Sequence[SensorHardware] | None = None,
                 sensor_b: Sequence[SensorHardware] | None = None,
                 payload_reqs: Mapping[str, PayloadRequirement] | None = None,
                 cfg: HvPlannerConfig | None = None,
                 fence_zones: Iterable[Any] | None = None,
                 alternates: Sequence[Any] | None = None,
                 recovery_sites: Mapping[str, str] | None = None,
                 per_uav_takeoff_sites: bool = True,
                 land_at_takeoff_site: bool = True,
                 wind: tuple[float, float] | None = None,
                 c2_range_km: float = 25.0,
                 terrain_clearance_m: float | None = None,
                 waiver_max_agl_m: float | None = 200.0,
                 dem_step_m: float = 50.0,
                 dem_margin_m: float = 1500.0,
                 progress: Callable[[float, str], None] | None = None,
                 on_report: Callable[[dict[str, Any]], None] | None = None,
                 energy: Mapping[str, Mapping[str, float]] | None = None,
                 home: tuple[float, float] | Site | None = None,
                 types: HvTypeRegistry | None = None,
                 reduce_waypoints: bool = True,
                 raise_on_no_go: bool = True,
                 mapping_mode: MappingModeConfig | None = None,
                 forward_overlap: float = 0.70,
                 camera_cfg: CameraTriggerConfig | None = None,
                 mapping_max_wind_mps: float | None = None) -> None:
        if mapping_mode is not None and not isinstance(mapping_mode, MappingModeConfig):
            raise AeroPlanAdapterError(f"mapping_mode 須為 MappingModeConfig 或 None，得 {type(mapping_mode).__name__}")
        self.mapping_mode = mapping_mode
        # 建圖作業風速上限（m/s；2026-10-15 首飛 ≤ 3）：只在建圖模式有意義 ⇒ 沒開建圖卻給了 → 拒絕（不靜默忽略）
        if mapping_max_wind_mps is not None:
            if mapping_mode is None:
                raise AeroPlanAdapterError(
                    f"mapping_max_wind_mps={mapping_max_wind_mps!r} 只在建圖模式（mapping_mode）有作用；沒開建圖不要給")
            _w = float(mapping_max_wind_mps)
            if not (math.isfinite(_w) and _w > 0.0):
                raise AeroPlanAdapterError(f"建圖作業風速上限須為 > 0 的有限數，得 {mapping_max_wind_mps!r}")
            mapping_max_wind_mps = _w
        self.mapping_max_wind_mps = mapping_max_wind_mps
        self.mapping_effective: dict[str, Any] | None = None
        self.forward_overlap = _check_forward_overlap(forward_overlap)
        if camera_cfg is not None:
            self._check_camera_cfg(camera_cfg)
        self.camera_cfg = camera_cfg
        self.platform_a = platform_a
        self.platform_b = platform_b
        self.class_of = class_of
        self.sensor_a = tuple(sensor_a) if sensor_a else None
        self.sensor_b = tuple(sensor_b) if sensor_b else None
        self.payload_reqs = dict(payload_reqs) if payload_reqs else None
        self.cfg = cfg if cfg is not None else HvPlannerConfig(dem_step_m=25.0)
        self.fence_zones = list(fence_zones) if fence_zones else None
        self.alternates = list(alternates) if alternates else None
        # vendored #4：逐機回收站（{uav_id 或 drone_id: site_id}）；None ⇒ 全員走全域 recovery
        self.recovery_sites = dict(recovery_sites) if recovery_sites else None
        # vendored #7：n 台 ⇒ n 個起飛站（由每台的 position 推），比照 DCCPP。
        #   座標全部相同時自動 no-op，所以既有情境逐位元不變。
        # ⚠⚠⚠ 這個功能一度**不安全**（2026-09-09 實測）；根因與修法記在 vendored #8 ——
        #   同坪豁免比場坪**座標** ≤ SAME_PAD_TOL_M(50 m)，而走廊排程以 site_id 字串為鍵
        #   ⇒ 兩個相距 < 50 m 的**不同**站被豁免卻沒被互斥。改比站點**身分**後：
        #     站點間距   最小水平間距(修前→修後)   makespan(修前→修後)   被豁免的真違規
        #       全部同點      148.2 → 148.2 m       1401.3 → 1401.3 s      0 → 0
        #       200 m          17.9 →  17.9 m       1177.3 → 1177.3 s      0 → 0
        #        60 m          30.0 →  52.5 m       1259.0 → 1589.2 s    496 → 0
        #        30 m          15.0 → 101.2 m       1188.3 → 1277.9 s   1037 → 0
        #        15 m           7.5 → 101.6 m       1190.1 → 1279.0 s   1062 → 0
        #        10 m          15.0 → 107.2 m       1192.0 → 1281.3 s      — → 0
        #   ⇒ 現在 10 m 間距也合法（4D 修復自己把起降錯開），代價是 makespan +7～26%。
        self.per_uav_takeoff_sites = bool(per_uav_takeoff_sites)
        # 各機預設降回自己的起飛站（2026-10-09 使用者定調）；只在逐機起飛站生效時有作用
        self.land_at_takeoff_site = bool(land_at_takeoff_site)
        self.wind = (float(wind[0]), float(wind[1])) if wind else (0.0, 0.0)
        self.c2_range_km = float(c2_range_km)
        self.terrain_clearance_m = terrain_clearance_m
        self.waiver_max_agl_m = waiver_max_agl_m
        self.dem_step_m = float(dem_step_m)
        self.dem_margin_m = float(dem_margin_m)
        self.progress = progress
        self.on_report = on_report
        self.energy = dict(energy) if energy else None
        self.home = home
        self.types = types
        self.reduce_waypoints = bool(reduce_waypoints)
        self.raise_on_no_go = bool(raise_on_no_go)
        # 最近一次結果
        self.notes: list[str] = []
        self.id_map: dict[int, str] = {}
        self.last_inputs: MissionInputs | None = None
        self.last_ctx: MissionContext | None = None
        self.last_plan: MissionPlan | None = None
        self.last_result: dict[str, Any] | None = None

    # -- 內部 ---------------------------------------------------------------
    def _check_camera_cfg(self, camera_cfg: Any) -> CameraTriggerConfig:
        """型別 + 引入段值鎖：``camera_lead_in_m`` 必須等於規劃時生效的 ``mapping_mode.lead_in_m``（沒開建圖 ＝ 0）。

        規劃器把每條帶兩端外延 lead_in_m，匯出器在帶起點後 camera_lead_in_m 才開相機；兩者不同 ⇒ 條帶頭
        少拍（外延不夠）或多飛（外延白費），而且沒有任何地方會報錯，所以在這裡拒絕。
        """
        if not isinstance(camera_cfg, CameraTriggerConfig):
            raise AeroPlanAdapterError(f"camera_cfg 須為 CameraTriggerConfig，得 {type(camera_cfg).__name__}")
        lead = float(self.mapping_mode.lead_in_m) if self.mapping_mode is not None else 0.0
        if abs(float(camera_cfg.camera_lead_in_m) - lead) > 1e-9:
            raise AeroPlanAdapterError(
                f"相機引入段 camera_lead_in_m={camera_cfg.camera_lead_in_m:g} m ≠ 規劃時生效的引入段 lead_in_m="
                f"{lead:g} m（{'建圖模式' if self.mapping_mode is not None else '沒開建圖模式'}）；兩者必須是同一個數")
        return camera_cfg

    def _plan_of(self, drone_id: Any) -> tuple[str, UAVPlan]:
        """AeroPlan drone_id → (hv uav_id, UAVPlan)；沒規劃或不認得 ⇒ AeroPlanAdapterError。"""
        if self.last_plan is None or self.last_ctx is None:
            raise AeroPlanAdapterError("尚未規劃：請先呼叫 plan_coverage_dccpp")
        try:
            did = int(drone_id)
        except (TypeError, ValueError):
            raise AeroPlanAdapterError(f"drone_id 須為整數，得 {drone_id!r}") from None
        uid = self.id_map.get(did)
        up = self.last_plan.plans.get(uid) if uid is not None else None
        if up is None:
            flying = sorted(d for d, u in self.id_map.items() if u in self.last_plan.plans)
            raise AeroPlanAdapterError(f"drone_id={did} 在這份計畫裡沒有任務（機隊 {sorted(self.id_map)}，"
                                       f"有任務的 {flying}）")
        return uid, up

    def _tick(self, frac: float, msg: str) -> None:
        if self.progress is not None:
            try:
                self.progress(float(frac), str(msg))
            except Exception:                              # 回呼失敗不得影響規劃
                pass

    # -- 主接縫 --------------------------------------------------------------
    def plan_coverage_dccpp(self, areas: list[dict[str, Any]], drones: list[dict[str, Any]],
                            coverage_width_m: float, overlap_rate: float = 0.1,
                            auto_scan_angle: bool = True, coverage_altitude: float = 100.0,
                            terrain_func=None, enable_altitude: bool = True,
                            mounting_angle_deg: float = 0.0, dem_path: str = "",
                            coordination_mode: str = "uncoordinated",
                            runway_bearing_deg: float | None = None, runway_length_m: float | None = None,
                            landing_bearing_deg: float | None = None,
                            landing_rollout_m: float = 0.0) -> dict[str, Any]:
        """與 REPO/mission/swarm_coordinator.py:975 **逐字相同**的簽章；回傳同樣的 6 鍵字典。

        流程：:func:`to_mission_inputs` → :func:`hv_dccpp.build_context` → :func:`hv_dccpp.preflight`
        → :func:`hv_dccpp.plan_mission` → :func:`from_mission_plan`。

        參數之處置（逐項；完整清單見 :data:`PARAM_KEY_NOTES`）
        ----------------------------------------------------
        ``coverage_width_m``
            僅作核對（障礙 1）；實際刈幅由感測器硬體 + 作業高度推導。
        ``overlap_rate`` / ``coverage_altitude``
            → :class:`PayloadRequirement` 的 ``side_overlap`` / ``h_target_agl_m``。
        ``auto_scan_angle``
            忽略：hv_dccpp 一律最佳化掃描方向 α*（規格 §7.3、論文 §2.3.1）。
        ``terrain_func``
            → :func:`terrain_func_to_dem`（``None`` → 平地 DEM）。
        ``enable_altitude``
            忽略：高度規劃不可關閉（規格 §9、論文 Alg.3）。``False`` 時記 note。
        ``mounting_angle_deg``
            與建構子的 ``hfov/vfov``（若有）合成 :class:`TrapezoidFovSpec`；單獨給時記 note。
        ``dem_path``
            不讀取（無 rasterio 相依）；非空時記 note ``dem_path_ignored``。
        ``coordination_mode``
            忽略：hv_dccpp 一律做 4-D 時空去衝突（規格 §10.6、D14）。
        ``runway_bearing_deg`` / ``runway_length_m`` / ``landing_bearing_deg`` / ``landing_rollout_m``
            忽略：兩構型皆為 VTOL，垂直起降不需跑道；進場方位由逆風規則決定（D19）。
            ``runway_bearing_deg`` 只在 drone dict 缺 ``heading`` 時作為初始航向。

        返回
        ----
        ``{'results', 'allocation', 'coverage_paths', 'uav_states', 'total_makespan', 'assembled_paths'}``
        （鍵名／型別見 :func:`from_mission_plan`）。

        例外
        ----
        :class:`AeroPlanAdapterError`
            輸入不合法，或 preflight ``NO_GO`` 且 ``raise_on_no_go=True``（預設）。
        """
        self.notes = []
        note = self.notes
        if not enable_altitude:
            note.append("enable_altitude_ignored(hv_dccpp 一律做高度規劃；規格 §9)")
        if not auto_scan_angle:
            note.append("auto_scan_angle_ignored(hv_dccpp 一律最佳化掃描方向；規格 §7.3)")
        if str(coordination_mode) != "uncoordinated":
            note.append(f"coordination_mode_ignored({coordination_mode}; hv_dccpp 一律 4-D 去衝突)")
        for name, val in (("runway_bearing_deg", runway_bearing_deg), ("runway_length_m", runway_length_m),
                          ("landing_bearing_deg", landing_bearing_deg), ("landing_rollout_m", landing_rollout_m)):
            if val:
                note.append(f"{name}_ignored({val}; VTOL 垂直起降不需跑道)")
        if mounting_angle_deg:
            note.append(f"mounting_angle_deg={mounting_angle_deg} 僅在同時提供 hfov/vfov（TrapezoidFovSpec）時生效")

        self._tick(0.02, "組裝任務輸入")
        inputs, id_map = to_mission_inputs(
            areas, drones, coverage_width_m=float(coverage_width_m), overlap_rate=float(overlap_rate),
            coverage_altitude=float(coverage_altitude), terrain_func=terrain_func, dem_path=str(dem_path or ""),
            home=self.home, fence_zones=self.fence_zones, class_of=self.class_of,
            platform_a=self.platform_a, platform_b=self.platform_b,
            sensor_a=self.sensor_a, sensor_b=self.sensor_b, payload_reqs=self.payload_reqs,
            alternates=self.alternates, recovery_sites=self.recovery_sites,
            per_uav_takeoff_sites=self.per_uav_takeoff_sites,
            land_at_takeoff_site=self.land_at_takeoff_site,
            wind=self.wind, c2_range_km=self.c2_range_km,
            terrain_clearance_m=self.terrain_clearance_m, waiver_max_agl_m=self.waiver_max_agl_m,
            dem_step_m=self.dem_step_m, dem_margin_m=self.dem_margin_m,
            mounting_angle_deg=float(mounting_angle_deg), energy=self.energy, warnings=note,
            mapping_mode=self.mapping_mode, forward_overlap=self.forward_overlap)
        self.last_inputs = inputs
        self.id_map = id_map

        self._tick(0.15, "建立情境（地形／風場／能量模型）")
        try:
            ctx = build_context(inputs, self.cfg)
        except MissionInputError as e:
            raise AeroPlanAdapterError(f"HV-DCCPP build_context 失敗：{e}", [str(e)]) from e
        self.last_ctx = ctx

        self._tick(0.35, "前置檢查（preflight）")
        self._tick(0.40, "規劃中（分解 → 配置 → IDP → 組裝 → 去衝突）")
        plan = plan_mission(inputs, self.cfg, ctx=ctx)
        self.last_plan = plan
        # 讀回：偏好不等於保證（能量吃不下會退回別的站）。沒降回自己的起飛站就明說，
        # 不要讓使用者在 3D 圖上才發現「又降到同一點」。
        for _u in inputs.fleet:
            _want = getattr(_u, "recovery_site_id", None)
            _p = plan.plans.get(_u.uav_id)
            _got = getattr(_p, "landing_site", None) if _p is not None else None
            if _want and _p is not None and _got is not None and str(_got) != str(_want):
                note.append(f"landing_not_at_requested_site({_u.uav_id}: 要降 {_want}，實際降 {_got}；"
                            f"能量或走廊不允許)")
        rep = plan.feasibility
        note.extend(f"preflight:{w}" for w in rep.warnings)
        self.mapping_effective = None
        if self.mapping_mode is not None:
            # 有效值契約：使用者開了建圖模式，就把實際用的偏流角／間距／接縫重疊／外擴／引入段寫出來
            per_class = {c: d.get("mapping") for c, d in (rep.derived or {}).items() if isinstance(d, dict)}
            seam = {aid: r.get("mapping_seam") for aid, r in (rep.per_area or {}).items() if isinstance(r, dict)}
            self.mapping_effective = {"config": self.mapping_mode.to_dict(), "per_class": per_class,
                                      "seam_by_area": seam}
            if getattr(self.mapping_mode, "cross_strips", False):
                # 交叉航帶有效值：規劃器的 mapping_cross(...) 摘要（逐機逐區塊）＋略過／退回的區塊
                self.mapping_effective["cross"] = [str(w) for w in (rep.warnings or [])
                                                   if str(w).startswith(("mapping_cross(", "mapping_cross_skipped(",
                                                                         "mapping_cross_dropped("))]
            if self.mapping_max_wind_mps is not None:
                w_plan = float(ctx.wind.max_speed()) if getattr(ctx, "wind", None) is not None else float("nan")
                self.mapping_effective["wind"] = {"limit_mps": self.mapping_max_wind_mps, "planning_wind_mps": w_plan}
                note.append(f"mapping_wind(規劃風速 {w_plan:g} m/s {'≤' if w_plan <= self.mapping_max_wind_mps + 1e-9 else '＞'}"
                            f" 建圖作業風速上限 {self.mapping_max_wind_mps:g} m/s"
                            + ("" if w_plan <= self.mapping_max_wind_mps + 1e-9 else " ⇒ 不可上傳")
                            + "；上限指**現場實際風速**，起飛前必須實測)")
            note.append("mapping_mode_effective(" + "; ".join(
                f"{c}: ψ={m.get('crab_deg', float('nan')):.2f}°, w={m.get('w_m', float('nan')):.1f} m, "
                f"w_eff={m.get('w_eff_m', float('nan')):.1f} m, s={m.get('s_m', float('nan')):.1f} m, "
                f"e_ext(含 lead_in {m.get('lead_in_m', 0.0):g})={m.get('e_ext_m', float('nan')):.1f} m"
                for c, m in sorted(per_class.items()) if m) + "; " + "; ".join(
                f"{aid}: 接縫重疊目標 {s_['overlap_target_m']:.1f} m、胞外擴 {s_['delta_cell_m']:.1f} m、"
                f"往鄰區長 {s_['neighbor_growth_m']:.1f} m、殘帶外擴 {s_['delta_residual_m']:.1f} m、"
                f"區域外緣 {s_['delta_area_outer_m']:.1f} m"
                for aid, s_ in sorted(seam.items()) if s_ and s_.get("overlap_target_m") is not None) + ")")
        payload = {"status": rep.status, "errors": list(rep.errors), "warnings": list(rep.warnings),
                   "notes": list(note), "makespan_s": float(plan.makespan_s),
                   "timing_ms": dict(plan.timing_ms),
                   "uncovered": [list(u) for u in plan.uncovered]}
        if self.mapping_effective is not None:
            payload["mapping_mode"] = self.mapping_effective
        if self.on_report is not None:
            try:
                self.on_report(payload)
            except Exception:
                pass
        if rep.status == "NO_GO":
            if self.raise_on_no_go:
                raise AeroPlanAdapterError("HV-DCCPP preflight NO_GO：" + "; ".join(rep.errors), rep.errors)
            note.append("NO_GO(回傳空結果)")

        self._tick(0.90, "轉換為 AeroPlan 結果格式")
        result = from_mission_plan(plan, ctx, areas=areas, drones=drones, id_map=id_map,
                                  types=self.types, reduce=self.reduce_waypoints)
        self.last_result = result
        self._tick(1.0, "完成")
        return result

    # -- 便利方法 -------------------------------------------------------------
    def upload_tuples(self, *, is_vtol: bool = True, cruise_speed_mps: float | None = None,
                      turn_radius_m: float | None = None,
                      vtol_transition_alt_m: float = 100.0) -> dict[int, list[tuple]]:
        """最近一次規劃 → ``{drone_id: 8-tuple 串列}``，直接餵
        REPO/mission/sitl_link.py:264 ``upload_mission``（派送方式同 REPO/ui/main_window.py:6188-6210）。

        ``cruise_speed_mps`` / ``turn_radius_m`` 為 ``None`` 時逐機取該構型之
        ``spec.cruise_airspeed_mps`` 與 ``spec.r_eff(W)``。``NAV_WAYPOINT`` 的 param2 是由轉彎半徑推導的
        **接受半徑** min(R_eff/2, 30) m（與 hv 匯出器同一條，見 :func:`to_upload_tuples`）。
        ⚠ 這條路徑沒有拍照指令（206）；建圖模式請用 :meth:`mission_items`。
        """
        if self.last_result is None or self.last_ctx is None:
            raise AeroPlanAdapterError("尚未規劃：請先呼叫 plan_coverage_dccpp")
        ctx = self.last_ctx
        w_ref = ctx.wind.max_speed() if ctx.wind is not None else 0.0
        out: dict[int, list[tuple]] = {}
        for did, bp in (self.last_result.get("assembled_paths") or {}).items():
            cls = getattr(bp, "cls", "") or "A"
            spec = ctx.specs.get(cls, _SPEC_OF.get(cls, self.platform_a))
            v = float(cruise_speed_mps) if cruise_speed_mps is not None else float(spec.cruise_airspeed_mps)
            r = float(turn_radius_m) if turn_radius_m is not None else float(spec.r_eff(w_ref))
            out[int(did)] = to_upload_tuples(bp, is_vtol=is_vtol, cruise_speed_mps=v, turn_radius_m=r,
                                             vtol_transition_alt_m=vtol_transition_alt_m)
        return out

    def mission_items(self, drone_id: int, *, camera_cfg: CameraTriggerConfig,
                      alt_frame: Literal["relative", "msl", "terrain"] = "relative") -> list[dict[str, Any]]:
        """某架的任務項目 → ``SITLLink.upload_mission_items`` 吃的 dict 串列（形狀見 :func:`mission_item_dicts`）。

        內容就是 hv 匯出器 :func:`hv_dccpp.ardupilot.plan_to_mission_items` ``(camera_trigger=True,
        camera_cfg=camera_cfg)`` 的輸出 —— 與 ``export_all(..., camera_cfg=同一份)`` 寫出的 ``.waypoints`` 逐項相同
        （206 每條帶開／關、DO_LAND_START 後緊接 206(0)、接受半徑 min(R_eff/2, 30)、空速型 DO_CHANGE_SPEED）。

        ``camera_cfg`` 必給；其 ``camera_lead_in_m`` ≠ 規劃時生效的 ``mapping_mode.lead_in_m``（沒開建圖 ＝ 0）
        ⇒ :class:`AeroPlanAdapterError`。

        第 0 列 home 與所有相對高度以**這架的起飛坪**為基準（飛控解鎖時 home 重設到那裡；見
        :func:`hv_dccpp.ardupilot.fc_home`）。任務**不帶**錯開起飛時間（不插 NAV_DELAY：武裝的飛機在地上等幾十分鐘
        不安全）⇒ 放行時刻見 :meth:`takeoff_schedule`。
        """
        _uid, up = self._plan_of(drone_id)
        cc = self._check_camera_cfg(camera_cfg)
        items = plan_to_mission_items(up, self.last_ctx, self.cfg, alt_frame=alt_frame, camera_trigger=True,
                                      camera_cfg=cc)
        return mission_item_dicts(items)

    def takeoff_schedule(self) -> dict[int, float]:
        """最近一次計畫的錯開起飛：``{drone_id: t_takeoff_s − min(t_takeoff_s)}``（秒；只列有任務的機）。

        hv_dccpp 的 4D 去衝突有一部分靠**延後起飛**（``UAVPlan.t_takeoff_s``）；建圖模式不分高度層，時間錯開就是唯一的
        分隔（R1：GUI 情境同時放行時最小間距 293 m → 12 m，要求 100 m）。:meth:`mission_items` 的任務本身不帶時間
        （預設不插 NAV_DELAY：武裝的飛機在地上等幾十分鐘不安全），所以地面站必須在 t0 + offset 才放行／MISSION_START
        該機；要把錯開寫進任務請用 ``hv_dccpp.ardupilot.export_all(..., takeoff_delay='nav_delay')``（此時就不要再由地面站
        延後）。同一份數字也寫在 ``export_all`` 摘要的 ``takeoff.offset_s``。
        """
        if self.last_plan is None:
            raise AeroPlanAdapterError("尚未規劃：請先呼叫 plan_coverage_dccpp")
        plans = self.last_plan.plans
        if not plans:
            return {}
        t0 = min(p.t_takeoff_s for p in plans.values())
        return {int(did): plans[uid].t_takeoff_s - t0 for did, uid in sorted(self.id_map.items()) if uid in plans}

    def expected_camera_params(self, drone_id: int, *, camera_cfg: CameraTriggerConfig) -> dict[str, float]:
        """上傳前讀回閘門要比對的**相機**參數（CAM1_*／CAM_*／RELAY*／相機用的 SERVOn_FUNCTION）。

        值由 hv 匯出器同一支 :func:`hv_dccpp.ardupilot.recommended_params` ``(sensor_kinds=('rgb',),
        camera_cfg=camera_cfg)`` 算出 ⇒ 與 ``export_all`` 寫出的 ``.param`` 同值（含 CAM1_TRIGG_DIST = 0：206 會把
        距離寫進 RAM，任務被打斷時殘值留在飛控上）。其餘飛控參數（空速、圍籬、電池…）不在這裡。
        """
        from ..ardupilot import recommended_params
        _uid, up = self._plan_of(drone_id)
        cc = self._check_camera_cfg(camera_cfg)
        spec = self.last_ctx.specs[up.cls]
        p = recommended_params(spec, None, self.cfg, sensor_kinds=("rgb",), camera_cfg=cc, warnings=[])
        return {k: float(v) for k, v in p.items()
                if k.startswith(("CAM1_", "CAM_", "RELAY")) or re.fullmatch(r"SERVO\d+_FUNCTION", k)}

    def mapping_gate(self, *, max_wind_mps: float | None = None) -> dict[str, Any]:
        """建圖上傳閘門：``{"uploadable": bool, "reasons": [原因, ...]}``（形狀不變；風速有效值見
        ``mapping_effective["wind"]``）。

        ``max_wind_mps`` 缺省 ⇒ 用建構時的 ``mapping_max_wind_mps``（GUI：面板「建圖作業風速上限」；
        MainWindow 的 ``hv_upload_gate`` 呼叫 ``mapping_gate()`` 不帶參數，所以上限一定要放在建構參數裡）。

        ⚠ preflight 的 GO_WITH_WARNINGS **不夠**：實測有 GO_WITH_WARNINGS 卻丟掉約 35% 面積的計畫。拼圖要的是
        「整區都有人拍」，所以下列任一 ⇒ 不可上傳：

        * 還沒規劃／preflight NO_GO；
        * ``plan.uncovered`` 非空（有區塊沒人飛；含能量排不進的交叉航帶 ``cross_energy_infeasible``）；
        * 規劃器或轉接層 notes 裡有 ``mapping_incomplete(``（建圖模式的破洞警告）；
        * ``max_wind_mps``（建圖作業風速上限，2026-10-15 使用者定調首飛 ≤ 3 m/s）給了，而**規劃實際用的風速**
          （``last_ctx.wind`` 各層最大值 ＝ 面板／呼叫端送進來的 HARVEST 風速）超過它。比較容差 1e-9 m/s
          （3.0 過、3.1 擋）。⚠ 這只檢查「規劃用的風」；現場實際風速要操作員起飛前實測 —— 上限適用的是現場的風。
        """
        reasons: list[str] = []
        plan = self.last_plan
        wind_eff: dict[str, Any] | None = None
        if max_wind_mps is None:
            max_wind_mps = getattr(self, "mapping_max_wind_mps", None)
        if max_wind_mps is not None:
            lim = float(max_wind_mps)
            if not (math.isfinite(lim) and lim > 0.0):
                raise AeroPlanAdapterError(f"建圖作業風速上限須為 > 0 的有限數，得 {max_wind_mps!r}")
            w_plan = None
            if self.last_ctx is not None and getattr(self.last_ctx, "wind", None) is not None:
                w_plan = float(self.last_ctx.wind.max_speed())
            wind_eff = {"limit_mps": lim, "planning_wind_mps": w_plan,
                        "ok": (w_plan is not None and w_plan <= lim + 1e-9)}
        if plan is None:
            return {"uploadable": False, "reasons": ["尚未規劃"]}
        if wind_eff is not None and not wind_eff["ok"]:
            reasons.append(
                f"建圖作業風速上限 {wind_eff['limit_mps']:g} m/s：規劃風速 "
                + ("未知" if wind_eff["planning_wind_mps"] is None else f"{wind_eff['planning_wind_mps']:g} m/s")
                + " 超過上限（有風時轉彎後改平不夠、照片側傾，SITL 5 m/s 實測 2.5% 照片在 10–15° 側傾）；"
                  "請在風速 ≤ 上限時再規劃與上傳（上限指現場實際風速，起飛前要實測）")
        feas = getattr(plan, "feasibility", None)
        if str(getattr(feas, "status", "")) == "NO_GO":
            reasons.append("preflight NO_GO")
        unc = list(getattr(plan, "uncovered", None) or [])
        if unc:
            reasons.append(f"未覆蓋 {len(unc)} 項：" + ", ".join(":".join(map(str, u)) if isinstance(u, (tuple, list))
                                                              else str(u) for u in unc))
        for w in list(getattr(feas, "warnings", None) or []) + list(self.notes or []):
            s = str(w)
            i = s.find("mapping_incomplete(")
            if i >= 0:
                reasons.append(s[i:])
                break
        return {"uploadable": not reasons, "reasons": reasons}

    def qgc_waypoints(self, *, key: Literal["uav_id", "drone_id"] = "drone_id",
                      camera_cfg: CameraTriggerConfig | None = None) -> dict[str, list[str]]:
        """最近一次規劃 → ``{key: QGC WPL 110 行串列}``（見 :func:`plan_to_aeroplan_waypoints`）。

        ``camera_cfg`` 原樣交給匯出器（給了就套用 :meth:`mission_items` 同一條引入段值鎖）。
        """
        if self.last_plan is None or self.last_ctx is None:
            raise AeroPlanAdapterError("尚未規劃：請先呼叫 plan_coverage_dccpp")
        if camera_cfg is not None:
            self._check_camera_cfg(camera_cfg)
        return plan_to_aeroplan_waypoints(self.last_plan, self.last_ctx, self.cfg, id_map=self.id_map, key=key,
                                          camera_cfg=camera_cfg)

    def events_from_telemetry(self, frames: Iterable[Any], **kw: Any) -> list[Any]:
        """最近一次規劃 + AeroPlan 遙測 → 動態事件（見 :func:`telemetry_frames_to_events`）。"""
        if self.last_plan is None or self.last_ctx is None:
            raise AeroPlanAdapterError("尚未規劃：請先呼叫 plan_coverage_dccpp")
        kw.setdefault("id_map", self.id_map)
        return telemetry_frames_to_events(frames, self.last_plan, self.last_ctx, self.cfg, **kw)
