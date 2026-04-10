"""
DCCPP 核心模組 v2 (重寫版)
==========================
忠實實作論文 (Wang et al., Defence Technology 2025) Sec 2.3 / 3。

組成
----
- area_processor       : 凸化 + 最小寬度 + 論文 Eq.6/7 掃描線
- dccpp_path_builder   : 真實 Dubins (含 CCC/loiter) 路徑建構器
- idp_solver           : Algorithm 2 IDP，含方向約束 (Eq.23–26)
- task_allocator       : Algorithm 1 多區域 UAV 分配 (Eq.30–31)
- altitude_planner     : Algorithm 3 GDA 3D 高度規劃 (沿用)
"""

from core.dccpp.area_processor import (
    AreaProcessor,
    AreaProcessingResult,
    MinWidthResult,
    ScanLine,
    find_min_width,
    generate_paper_scan_lines,
    douglas_peucker,
    convex_hull,
    compute_min_width_direction,
)
from core.dccpp.dccpp_path_builder import (
    DCCPPPathBuilder,
    BuiltPath,
    BuiltWaypoint,
    SegmentLabel,
)
from core.dccpp.idp_solver import (
    IDP_Solver,
    IDPResult,
    UAVPlan,
    to_legacy_mdtsp_result,
)
from core.dccpp.task_allocator import (
    UAVTaskAllocator,
    AllocationResult,
)

# UAV 配置資料結構（沿用舊檔）
from core.dccpp.uav_models import (
    UAVType,
    FOVConfig,
    DCCPPUAVConfig,
    make_fixed_wing_uav,
    make_multi_rotor_uav,
)

# 高度規劃（沿用舊檔）
from core.dccpp.altitude_planner import (
    AltitudePlanner,
    AltitudePlanResult,
)

__all__ = [
    "AreaProcessor", "AreaProcessingResult", "MinWidthResult", "ScanLine",
    "find_min_width", "generate_paper_scan_lines",
    "douglas_peucker", "convex_hull", "compute_min_width_direction",
    "DCCPPPathBuilder", "BuiltPath", "BuiltWaypoint", "SegmentLabel",
    "IDP_Solver", "IDPResult", "UAVPlan", "to_legacy_mdtsp_result",
    "UAVTaskAllocator", "AllocationResult",
    "UAVType", "FOVConfig", "DCCPPUAVConfig",
    "make_fixed_wing_uav", "make_multi_rotor_uav",
    "AltitudePlanner", "AltitudePlanResult",
]
