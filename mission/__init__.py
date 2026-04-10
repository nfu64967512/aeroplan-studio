"""
Mission 任務管理模組
"""

from .mission_manager import (
    MissionManager,
    Mission,
)

from .coverage_path import (
    CoveragePath,
    OperationSegment,
)

from .waypoint import (
    Waypoint,
    WaypointSequence,
    MAVCommand,
    CoordinateFrame,
    create_home_waypoint,
    create_takeoff_waypoint,
    create_rtl_waypoint,
    create_change_speed_command,
)

__all__ = [
    # 任務管理
    'MissionManager',
    'Mission',
    # 結構化覆蓋路徑
    'CoveragePath',
    'OperationSegment',
    # 航點
    'Waypoint',
    'WaypointSequence',
    'MAVCommand',
    'CoordinateFrame',
    # 便捷函數
    'create_home_waypoint',
    'create_takeoff_waypoint',
    'create_rtl_waypoint',
    'create_change_speed_command',
]
