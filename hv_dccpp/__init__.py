"""HV-DCCPP — Heterogeneous-VTOL Dynamic Complete Coverage Path Planning.

兩構型（A = Pioneer PNP、B = HERO PNP，皆為 lift-and-cruise 4+1 VTOL）群飛偵查／區域覆蓋路徑規劃，
以 Wang et al. 2025 (Defence Technology 47:197-215) 的 DCCPP 框架為基礎擴展。

頂層 API（直接自套件取用）：

    import hv_dccpp as hv
    inputs = hv.load_mission("mission.json")          # 表 2 任務輸入（JSON / dict）→ MissionInputs
    cfg    = hv.PlannerConfig()                        # 規劃參數（規格 §16）
    ctx    = hv.build_context(inputs, cfg)             # ENU 框架、地形、風場、能量模型、感測器
    report = hv.preflight(inputs, cfg, ctx)            # GO / GO_WITH_WARNINGS / NO_GO
    plan   = hv.plan_mission(inputs, cfg, ctx=ctx)     # MissionPlan（每機 4-D 航點）
    cache  = hv.build_validation_cache(plan, ctx, budget_only=True)
    vrep   = hv.validate(plan, ctx, cache, cfg)        # 覆蓋／能量／坡度／AGL／空域／分離
    hv.export_all(plan, ctx, cfg, "out/")              # ArduPilot QGC WPL 110 + rally + fence + .param + .plan
    res    = hv.replan(plan, event, telemetry, ctx, hv.cache_from_plan(plan, ctx), cfg, [])   # 動態事件重規劃

整合層（延遲載入，首次存取時才 import；不需要 paho-mqtt / pymavlink 也能 import 本套件）：
    hv.HvDccppCoordinator / hv.to_upload_tuples / ...          ← integration.aeroplan_adapter（AeroPlan Studio PyQt6 地面站）
    hv.HVDCCPPGeneratorAdapter / hv.LegacyCollisionAvoidance   ← integration.legacy_adapter（既有 Tkinter 系統）
    hv.MqttBridge / hv.handle_request / hv.handle_request_full ← integration.mqtt_bridge（既有 MQTT 地面站）
    hv.VehicleLink / hv.FleetLink                              ← mavlink_link（ArduPilot QuadPlane / mock / SITL）
    hv.build_mini_example / hv.build_hills_scenario            ← scenario（內建示範情境，免外部 DEM）
    hv.save_all / hv.viz                                       ← viz（存取即把 matplotlib 後端設為 Agg）

子模組（`from hv_dccpp import viz` 等）：geo, platform, dubins, geometry, wind, mission, context, routing,
altitude, decompose, costs, preflight, allocate, deconflict, planner, dynamic, sim, viz, scenario, ardupilot,
mavlink_link, integration.legacy_adapter, integration.mqtt_bridge, integration.aeroplan_adapter。`viz` 會把 matplotlib 後端設為 Agg，
故不在此處預先載入（既有 Tkinter 系統以 TkAgg 繪圖）。
"""
from __future__ import annotations

import importlib
from typing import TYPE_CHECKING, Any

__version__ = "0.1.0"

# ---------------------------------------------------------------------------
# 核心 API（預先載入；只依賴 numpy / scipy / shapely，約 1 s）
# ---------------------------------------------------------------------------
from .platform import (  # noqa: E402
    PLATFORM_HERO, PLATFORM_PIONEER, EnergyModel, MissionInputError, PlatformSpec, PlatformSpecError,
)
from .mission import (  # noqa: E402
    FeasibilityReport, MappingMissionInputs, MappingModeConfig, MissionInputs, MissionPlan, Mode, Phase, PlannerConfig,
    UAVPlan, Waypoint, load_mission,
)
from .context import MissionContext, build_context  # noqa: E402
from .preflight import preflight  # noqa: E402
from .planner import plan_mission  # noqa: E402
from .dynamic import (  # noqa: E402
    EnergyUpdate, LinkLoss, LinkRestored, NewArea, PriorityChange, ReplanResult, TargetCue, Telemetry, UAVExit,
    WeatherAbort, WindUpdate, cache_from_plan, replan,
)
from .sim import ValidationReport, build_validation_cache, validate  # noqa: E402
from .ardupilot import export_all  # noqa: E402

# ---------------------------------------------------------------------------
# 整合層：延遲載入（PEP 562）。`from hv_dccpp import MqttBridge` 亦適用。
# ---------------------------------------------------------------------------
_LAZY_ATTRS: dict[str, tuple[str, str]] = {
    # 既有 Tkinter 系統（main_app.py）之 waypoint_generator 替換件
    "HVDCCPPGeneratorAdapter": ("hv_dccpp.integration.legacy_adapter", "HVDCCPPGeneratorAdapter"),
    "LegacyCollisionAvoidance": ("hv_dccpp.integration.legacy_adapter", "LegacyCollisionAvoidance"),
    "LegacyAdapterError": ("hv_dccpp.integration.legacy_adapter", "LegacyAdapterError"),
    # AeroPlan Studio（PyQt6 地面站）之 SwarmCoordinator.plan_coverage_dccpp 替換件
    "HvDccppCoordinator": ("hv_dccpp.integration.aeroplan_adapter", "HvDccppCoordinator"),
    "AeroPlanAdapterError": ("hv_dccpp.integration.aeroplan_adapter", "AeroPlanAdapterError"),
    "to_mission_inputs": ("hv_dccpp.integration.aeroplan_adapter", "to_mission_inputs"),
    "to_planner_config": ("hv_dccpp.integration.aeroplan_adapter", "to_planner_config"),
    "from_mission_plan": ("hv_dccpp.integration.aeroplan_adapter", "from_mission_plan"),
    "to_upload_tuples": ("hv_dccpp.integration.aeroplan_adapter", "to_upload_tuples"),
    "fence_zones_to_nofly": ("hv_dccpp.integration.aeroplan_adapter", "fence_zones_to_nofly"),
    "terrain_func_to_dem": ("hv_dccpp.integration.aeroplan_adapter", "terrain_func_to_dem"),
    "telemetry_frames_to_events": ("hv_dccpp.integration.aeroplan_adapter", "telemetry_frames_to_events"),
    "plan_to_aeroplan_waypoints": ("hv_dccpp.integration.aeroplan_adapter", "plan_to_aeroplan_waypoints"),
    "use_native_types": ("hv_dccpp.integration.aeroplan_adapter", "use_native_types"),
    "aeroplan_adapter": ("hv_dccpp.integration.aeroplan_adapter", None),
    # 既有 MQTT 地面站
    "MqttBridge": ("hv_dccpp.integration.mqtt_bridge", "MqttBridge"),
    "BridgeResult": ("hv_dccpp.integration.mqtt_bridge", "BridgeResult"),
    "BridgeDefaults": ("hv_dccpp.integration.mqtt_bridge", "BridgeDefaults"),
    "handle_request": ("hv_dccpp.integration.mqtt_bridge", "handle_request"),
    "handle_request_full": ("hv_dccpp.integration.mqtt_bridge", "handle_request_full"),
    # ArduPilot MAVLink 鏈路（需 pymavlink）
    "VehicleLink": ("hv_dccpp.mavlink_link", "VehicleLink"),
    "FleetLink": ("hv_dccpp.mavlink_link", "FleetLink"),
    "VehicleTelemetry": ("hv_dccpp.mavlink_link", "VehicleTelemetry"),
    "to_planner_telemetry": ("hv_dccpp.mavlink_link", "to_planner_telemetry"),
    "detect_events": ("hv_dccpp.mavlink_link", "detect_events"),
    # 內建情境（測試／示範用；不需外部 DEM）
    "build_mini_example": ("hv_dccpp.scenario", "build_mini_example"),
    "build_hills_scenario": ("hv_dccpp.scenario", "build_hills_scenario"),
    "build_paper_table2_scenario": ("hv_dccpp.scenario", "build_paper_table2_scenario"),
    # 繪圖（存取即載入 matplotlib 並設為 Agg 後端；Tkinter 系統請勿在主執行緒先行存取）
    "save_all": ("hv_dccpp.viz", "save_all"),
    "viz": ("hv_dccpp.viz", None),
    # 子模組本身
    "legacy_adapter": ("hv_dccpp.integration.legacy_adapter", None),
    "mqtt_bridge": ("hv_dccpp.integration.mqtt_bridge", None),
    "mavlink_link": ("hv_dccpp.mavlink_link", None),
    "scenario": ("hv_dccpp.scenario", None),
    "ardupilot": ("hv_dccpp.ardupilot", None),
}
"""屬性名 → (模組, 模組內名稱或 None=模組本身)。"""


def __getattr__(name: str) -> Any:
    """延遲載入整合層物件（避免 `import hv_dccpp` 就得安裝 paho-mqtt / pymavlink 或載入 matplotlib）。"""
    target = _LAZY_ATTRS.get(name)
    if target is None:
        raise AttributeError(f"module 'hv_dccpp' has no attribute {name!r}")
    module_name, attr = target
    module = importlib.import_module(module_name)
    value = module if attr is None else getattr(module, attr)
    globals()[name] = value          # 之後直接命中，不再經過 __getattr__
    return value


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(_LAZY_ATTRS))


if TYPE_CHECKING:  # 讓型別檢查器 / IDE 看得到延遲載入的名稱
    from .integration.aeroplan_adapter import (
        AeroPlanAdapterError, HvDccppCoordinator, fence_zones_to_nofly, from_mission_plan,
        plan_to_aeroplan_waypoints, telemetry_frames_to_events, terrain_func_to_dem, to_mission_inputs,
        to_planner_config, to_upload_tuples, use_native_types,
    )
    from .integration.legacy_adapter import HVDCCPPGeneratorAdapter, LegacyAdapterError, LegacyCollisionAvoidance
    from .integration.mqtt_bridge import BridgeDefaults, BridgeResult, MqttBridge, handle_request, handle_request_full
    from .mavlink_link import FleetLink, VehicleLink, VehicleTelemetry, detect_events, to_planner_telemetry
    from .scenario import build_hills_scenario, build_mini_example, build_paper_table2_scenario
    from .viz import save_all

__all__ = [
    "__version__",
    # 平台 / 輸入型別
    "PLATFORM_PIONEER", "PLATFORM_HERO", "PlatformSpec", "PlatformSpecError", "EnergyModel", "MissionInputError",
    "MissionInputs", "PlannerConfig", "MissionPlan", "UAVPlan", "Waypoint", "Mode", "Phase", "FeasibilityReport",
    "load_mission", "MappingModeConfig", "MappingMissionInputs",
    # 規劃流程
    "MissionContext", "build_context", "preflight", "plan_mission",
    # 動態重規劃
    "replan", "cache_from_plan", "ReplanResult", "Telemetry",
    "UAVExit", "NewArea", "EnergyUpdate", "LinkLoss", "LinkRestored", "WindUpdate", "TargetCue", "PriorityChange",
    "WeatherAbort",
    # 驗證與輸出
    "validate", "build_validation_cache", "ValidationReport", "export_all",
    # 整合層（延遲載入）
    "HvDccppCoordinator", "AeroPlanAdapterError", "to_mission_inputs", "to_planner_config", "from_mission_plan",
    "to_upload_tuples", "fence_zones_to_nofly", "terrain_func_to_dem", "telemetry_frames_to_events",
    "plan_to_aeroplan_waypoints", "use_native_types",
    "HVDCCPPGeneratorAdapter", "LegacyCollisionAvoidance", "LegacyAdapterError",
    "MqttBridge", "BridgeResult", "BridgeDefaults", "handle_request", "handle_request_full",
    "VehicleLink", "FleetLink", "VehicleTelemetry", "to_planner_telemetry", "detect_events",
    # 內建情境與繪圖（延遲載入）
    "build_mini_example", "build_hills_scenario", "build_paper_table2_scenario", "save_all",
    # 子模組（延遲載入）
    "legacy_adapter", "mqtt_bridge", "mavlink_link", "scenario", "ardupilot", "viz", "aeroplan_adapter",
]
