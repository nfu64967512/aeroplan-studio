"""
Global Planner 全局規劃器模組
"""

from .grid_generator import (
    GridSurveyGenerator,
    SurveyConfig,
    ScanPattern,
    EntryLocation,
    CameraConfig,
    SurveyStatistics
)

from .astar import (
    AStarPlanner,
    AStarNode,
    HeuristicType
)

from .coverage_planner import (
    CoveragePlanner,
    CoverageParameters,
    ScanPattern as CoverageScanPattern
)

from .nfz_planner import (
    FixedWingNFZPlanner,
    NoFlyZone,
    PathCorrectionResult,
    NFZType
)

from .advanced_scan_patterns import (
    AdvancedScanGenerator,
    SpiralParams,
    ConcentricParams,
    AdvancedScanResult,
    ScanPattern as AdvancedScanPattern
)

from .mdtsp_solver import (
    DCCPPSolver,
    IDPSolver,
    GreedyAllocator,
    AltitudePlanner,
    PathMatrixBuilder,
    UAVState,
    VehicleType,
    MDTSPResult,
)

__all__ = [
    # Grid Survey
    'GridSurveyGenerator',
    'SurveyConfig',
    'ScanPattern',
    'EntryLocation',
    'CameraConfig',
    'SurveyStatistics',

    # A*
    'AStarPlanner',
    'AStarNode',
    'HeuristicType',

    # Coverage Planner
    'CoveragePlanner',
    'CoverageParameters',
    'CoverageScanPattern',

    # NFZ Planner（固定翼禁航區）
    'FixedWingNFZPlanner',
    'NoFlyZone',
    'PathCorrectionResult',
    'NFZType',

    # Advanced Scan Patterns（固定翼進階掃描）
    'AdvancedScanGenerator',
    'SpiralParams',
    'ConcentricParams',
    'AdvancedScanResult',
    'AdvancedScanPattern',

    # MDTSP / IDP / GDA（多機覆蓋最佳化）
    'DCCPPSolver',
    'IDPSolver',
    'GreedyAllocator',
    'AltitudePlanner',
    'PathMatrixBuilder',
    'UAVState',
    'VehicleType',
    'MDTSPResult',
]
