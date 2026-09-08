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
故 :attr:`HvBuiltWaypoint.alt` 一律輸出 ``alt_msl_m − home_msl``（**不是** AGL 對當地地形，
也不是 AMSL）；hv_dccpp 內部之 ``Waypoint.agl_m`` 為對當地地形之 AGL，兩者在起伏地形不相等。

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

import math
import re
from dataclasses import dataclass, field, fields, is_dataclass, replace
from enum import Enum, auto
from typing import Any, Callable, Iterable, Literal, Mapping, Sequence

import numpy as np

from ..ardupilot import MissionItem, plan_to_mission_items
from ..context import MissionContext, build_context
from ..geo import DemGrid, GeoFrame
from ..mission import (
    AreaSpec, Airspace, C2Link, MissionInputError, MissionInputs, MissionPlan, NoFlyZone, PayloadRequirement,
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
    "fence_zones_to_nofly", "terrain_func_to_dem", "telemetry_frames_to_events", "plan_to_aeroplan_waypoints",
    "qgc_wpl_lines", "MAV_CMD",
]


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
        **AGL 相對 home**（m）= ``alt_msl_m − home_msl``，對應 ``MAV_FRAME_GLOBAL_RELATIVE_ALT``（frame 3）。
    heading_compass_deg
        航跡方位（度，0 = 北、順時針），已正規化至 ``[0, 360)``。
    segment_type
        :class:`HvSegmentLabel`。

    ``heading_deg`` 為 ``heading_compass_deg`` 之別名（legacy ``AssembledWaypoint`` 欄位名，
    REPO/core/trajectory/dccpp_path_assembler.py:47-63），消費端 REPO/ui/main_window.py:5308 兩者皆讀。
    """
    lat: float
    lon: float
    alt: float
    heading_compass_deg: float
    segment_type: HvSegmentLabel

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
                      wind: tuple[float, float] = (0.0, 0.0), c2_range_km: float = 25.0,
                      terrain_clearance_m: float | None = None, waiver_max_agl_m: float | None = 200.0,
                      max_agl_m: float = 120.0, dem_step_m: float = 50.0, dem_margin_m: float = 1500.0,
                      base_msl_m: float = 0.0, weather_temp_c: float = 25.0,
                      mounting_angle_deg: float = 0.0, hfov_deg: float = 0.0, vfov_deg: float = 0.0,
                      energy: Mapping[str, Mapping[str, float]] | None = None,
                      warnings: list[str] | None = None) -> tuple[MissionInputs, dict[int, str]]:
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
    terrain_clearance_m
        對地淨空（D11 必填）；``None`` → ``max(36, 3σ_DEM + 30)``。
    dem_margin_m
        DEM 邊界外擴（m）；須大於 hv_dccpp 的 ``terrain_margin_m``（預設 1500）否則地形取樣會落在網格外。
    warnings
        傳入 list 時，本函式把警告字串附加進去。

    返回
    ----
    ``(MissionInputs, {drone_id: uav_id})``——第二項為 AeroPlan 整數 id 與 hv_dccpp 字串 id 的對照表。
    """
    warn = warnings if warnings is not None else []
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
        reqs, req_id, kind_req = _default_payload_reqs(hw_used, overlap_rate, h_target, warn)
    else:
        reqs = dict(payload_reqs)
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
    inputs = MissionInputs(home=home_site, alternates=alt_sites, areas=tuple(area_specs), dem=dem,
                           obstacles=(), wind=winds, fleet=tuple(fleet), payload_reqs=reqs, c2=c2,
                           airspace=airspace, weather_thresholds=WeatherThresholds(), weather_obs=obs)
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


def _default_payload_reqs(hw_used: Mapping[str, tuple[SensorHardware, ...]], overlap_rate: float,
                          h_target_agl_m: float, warn: list[str]
                          ) -> tuple[dict[str, PayloadRequirement], str, dict[str, str]]:
    """由機隊硬體推導 :class:`PayloadRequirement` 字典。

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
                                  side_overlap=side, forward_overlap=0.70, h_target_agl_m=h_target_agl_m)

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
    home_msl = _home_msl(ctx)
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
        bp = _built_path(up, ctx, cfg, did, reg, reduce=reduce, wp_radius_m=wp_radius_m, home_msl=home_msl)
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
    """home 地面之 MSL 高（m）——``HvBuiltWaypoint.alt`` 之基準（frame 3）。"""
    xy, z = ctx.sites_enu[ctx.inputs.home.site_id]
    return float(ctx.frame.enu_z_to_msl(np.asarray(xy, dtype=float).reshape(1, 2), float(z))[0])


def _wgs_xy(ctx: MissionContext, xy: Any) -> tuple[float, float]:
    """ENU ``(x, y)`` → ``(lat, lon)``（z 取 0，供多邊形／條帶端點用）。"""
    p = np.asarray(xy, dtype=float).reshape(2)
    w = ctx.frame.to_wgs84(np.array([[p[0], p[1], 0.0]]))[0]
    return (float(w[0]), float(w[1]))


def _built_path(up: UAVPlan, ctx: MissionContext, cfg: HvPlannerConfig, drone_id: int, reg: HvTypeRegistry, *,
                reduce: bool, wp_radius_m: float | None, home_msl: float) -> Any | None:
    """單機 :class:`UAVPlan` → :class:`HvBuiltPath`（或註冊表中的原生 ``BuiltPath``）。"""
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
                                  alt=max(0.0, float(wgs[k, 2]) - home_msl),
                                  heading_compass_deg=_compass(w0.track_deg), segment_type=lab))
    return _construct(reg.built_path, uav_id=int(drone_id), waypoints=out_wps,
                      entry_length_m=lengths["ENTRY"], operation_length_m=lengths["OPERATION"],
                      transfer_length_m=lengths["TRANSFER"], takeoff_length_m=lengths["TAKEOFF"],
                      landing_length_m=lengths["LANDING"], hv_uav_id=up.uav_id, cls=up.cls,
                      makespan_s=float(up.t_land_s))


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
                     default_alt_m: float = 100.0, emit_change_speed: bool = True) -> list[tuple]:
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
        ``NAV_WAYPOINT`` 之 param2（AeroPlan 於 :6122 以此傳轉彎半徑）。
    vtol_transition_alt_m
        MC↔FW 轉換高度（AGL 相對 home）。AeroPlan 由 ``parameter_panel.vtol_transition_alt_spin`` 取得（:6030）。
    default_alt_m
        航點 ``alt`` 缺值時的替代高度。
    emit_change_speed
        False → 不輸出開頭的 ``DO_CHANGE_SPEED``。

    返回
    ----
    ``[(lat, lon, alt, cmd, p1, p2, p3, p4), ...]``；``alt`` 為 **AGL 相對 home**
    （``sitl_link`` 對 NAV 指令一律用 ``MAV_FRAME_GLOBAL_RELATIVE_ALT``）。
    ``DO_*`` 項的 lat/lon/alt 皆為 0（該處 frame 為 ``MAV_FRAME_MISSION``）。

    與 AeroPlan 原實作的**唯一差異**（刻意；記於 CONTRACT_DEVIATIONS.md）
    ------------------------------------------------------------------
    REPO/ui/main_window.py:6113-6115 對 ``seg_name == 'TAKEOFF'`` 且 ``idx == 0`` 一律 ``continue``；
    若路徑只有**一個** TAKEOFF 航點（正是 hv_dccpp 縮點後的情形——``NAV_VTOL_TAKEOFF`` 只需一點），
    ``takeoff_emitted`` 永遠為 False，整個任務就**沒有起飛指令**（飛控會直接從第一個 NAV_WAYPOINT 開始）。
    本函式在 ``idx == 0`` 且尚未輸出起飛指令時照樣輸出 ``NAV_VTOL_TAKEOFF`` + ``DO_VTOL_TRANSITION(4)``，
    確保「VTOL 起降指令齊全」。其餘序列逐項相同。
    """
    CMD_WP, CMD_TO, CMD_LAND, CMD_SPD = MAV_CMD["NAV_WAYPOINT"], MAV_CMD["NAV_TAKEOFF"], MAV_CMD["NAV_LAND"], MAV_CMD["DO_CHANGE_SPEED"]
    CMD_VTOL_TO, CMD_VTOL_LAND, CMD_VTOL_TRANS = MAV_CMD["NAV_VTOL_TAKEOFF"], MAV_CMD["NAV_VTOL_LAND"], MAV_CMD["DO_VTOL_TRANSITION"]
    out: list[tuple] = []
    wp_list = list(getattr(built_path, "waypoints", None) or [])
    if not wp_list:
        return out
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
        alt0 = float(getattr(w0, "alt", default_alt_m) or default_alt_m)
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
        alt = float(getattr(wp, "alt", default_alt_m) or default_alt_m)
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
                out.append((lat, lon, alt, CMD_WP, 0.0, float(turn_radius_m), 0.0, 0.0))
        elif seg == "LANDING":
            if idx == last_landing_idx:
                if is_vtol:
                    out.append((lat, lon, float(vtol_transition_alt_m), CMD_WP, 0.0, 0.0, 0.0, 0.0))
                    out.append((0.0, 0.0, 0.0, CMD_VTOL_TRANS, 3.0, 0.0, 0.0, 0.0))
                    out.append((lat, lon, 0.0, CMD_VTOL_LAND, 0.0, 0.0, 0.0, hdg))
                else:
                    out.append((lat, lon, 0.0, CMD_LAND, 0.0, 0.0, 0.0, hdg))
            else:
                out.append((lat, lon, alt, CMD_WP, 0.0, float(turn_radius_m), 0.0, 0.0))
        else:
            out.append((lat, lon, alt, CMD_WP, 0.0, float(turn_radius_m), 0.0, 0.0))
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
                               key: Literal["uav_id", "drone_id"] = "uav_id") -> dict[str, list[str]]:
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

    返回
    ----
    ``{key: [行, ...]}``。
    """
    cfg = cfg if cfg is not None else ctx.cfg
    rev: dict[str, int] = {v: k for k, v in dict(id_map or {}).items()}
    out: dict[str, list[str]] = {}
    for uid, up in plan.plans.items():
        items = plan_to_mission_items(up, ctx, cfg, alt_frame="relative", camera_trigger=camera_trigger)
        name = uid if key == "uav_id" or uid not in rev else str(rev[uid])
        out[name] = qgc_wpl_lines(items, header=header)
    return out


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
        ``_dccpp_metrics`` 欄位，REPO/ui/main_window.py:5046-5054）。

    屬性
    ----
    last_inputs / last_ctx / last_plan / last_result / notes / id_map
        最近一次規劃的中間物件與警告，供除錯與 A/B 比較。
    """

    def __init__(self, *, platform_a: PlatformSpec = PLATFORM_PIONEER, platform_b: PlatformSpec = PLATFORM_HERO,
                 class_of: Callable[[Mapping[str, Any]], str] | None = None,
                 sensor_a: Sequence[SensorHardware] | None = None,
                 sensor_b: Sequence[SensorHardware] | None = None,
                 payload_reqs: Mapping[str, PayloadRequirement] | None = None,
                 cfg: HvPlannerConfig | None = None,
                 fence_zones: Iterable[Any] | None = None,
                 alternates: Sequence[Any] | None = None,
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
                 raise_on_no_go: bool = True) -> None:
        self.platform_a = platform_a
        self.platform_b = platform_b
        self.class_of = class_of
        self.sensor_a = tuple(sensor_a) if sensor_a else None
        self.sensor_b = tuple(sensor_b) if sensor_b else None
        self.payload_reqs = dict(payload_reqs) if payload_reqs else None
        self.cfg = cfg if cfg is not None else HvPlannerConfig(dem_step_m=25.0)
        self.fence_zones = list(fence_zones) if fence_zones else None
        self.alternates = list(alternates) if alternates else None
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
            alternates=self.alternates, wind=self.wind, c2_range_km=self.c2_range_km,
            terrain_clearance_m=self.terrain_clearance_m, waiver_max_agl_m=self.waiver_max_agl_m,
            dem_step_m=self.dem_step_m, dem_margin_m=self.dem_margin_m,
            mounting_angle_deg=float(mounting_angle_deg), energy=self.energy, warnings=note)
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
        rep = plan.feasibility
        note.extend(f"preflight:{w}" for w in rep.warnings)
        payload = {"status": rep.status, "errors": list(rep.errors), "warnings": list(rep.warnings),
                   "notes": list(note), "makespan_s": float(plan.makespan_s),
                   "timing_ms": dict(plan.timing_ms),
                   "uncovered": [list(u) for u in plan.uncovered]}
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
        ``spec.cruise_airspeed_mps`` 與 ``spec.r_eff(W)``。
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

    def qgc_waypoints(self, *, key: Literal["uav_id", "drone_id"] = "drone_id") -> dict[str, list[str]]:
        """最近一次規劃 → ``{key: QGC WPL 110 行串列}``（見 :func:`plan_to_aeroplan_waypoints`）。"""
        if self.last_plan is None or self.last_ctx is None:
            raise AeroPlanAdapterError("尚未規劃：請先呼叫 plan_coverage_dccpp")
        return plan_to_aeroplan_waypoints(self.last_plan, self.last_ctx, self.cfg, id_map=self.id_map, key=key)

    def events_from_telemetry(self, frames: Iterable[Any], **kw: Any) -> list[Any]:
        """最近一次規劃 + AeroPlan 遙測 → 動態事件（見 :func:`telemetry_frames_to_events`）。"""
        if self.last_plan is None or self.last_ctx is None:
            raise AeroPlanAdapterError("尚未規劃：請先呼叫 plan_coverage_dccpp")
        kw.setdefault("id_map", self.id_map)
        return telemetry_frames_to_events(frames, self.last_plan, self.last_ctx, self.cfg, **kw)
