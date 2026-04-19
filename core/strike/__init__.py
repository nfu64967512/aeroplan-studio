"""蜂群打擊規劃模組 (2026 架構)

模組分層：

    terminal_strike_planner
        經典固定翼三段式軌跡 (起飛爬升 / Dubins 巡航 / 末端俯衝)
        提供 plan_auto (DTOT 異地) / plan_stot (STOT 同地)

    dtot_coordinator
        時間協同演算 (STOT 同秒 / DTOT 間隔)
        反推各機空速 + Loiter 補時

    swarm_strike_planner          (standalone, 單目標多方向 reference)
    advanced_swarm_strike_planner (standalone, + DTOT-interval 實驗版)
    vtol_swarm_strike_planner     (VTOL 專用：Phase 2/3 + CEP + AI 尋標)
    recon_to_strike_manager       (DCCPP → Strike 動態切換)
"""
from .terminal_strike_planner import (
    TerminalStrikePlanner,
    StrikeTarget,
    StrikeUAV,
    StrikeTrajectory,
    StrikeWaypoint,
)
from .dtot_coordinator import (
    DTOTCoordinator,
    TimingPlan,
)
from .vtol_swarm_strike_planner import (
    VTOLSwarmStrikePlanner,
    AttackMode as VTOLAttackMode,
    Target as VTOLTarget,
    VTOLUAV,
)
from .recon_to_strike_manager import (
    ReconToStrikeManager,
    UAVState,
    TaskMode,
    StrikeAssignment,
    CoalitionReport,
)
# 保留 (作為教學 reference，被 VTOL / Advanced 版超集)
from .swarm_strike_planner import SwarmStrikePlanner
from .advanced_swarm_strike_planner import (
    AdvancedSwarmStrikePlanner,
    AttackMode as AdvancedAttackMode,
)

__all__ = [
    # 主要整合到 UI
    'TerminalStrikePlanner', 'StrikeTarget', 'StrikeUAV',
    'StrikeTrajectory', 'StrikeWaypoint',
    'DTOTCoordinator', 'TimingPlan',
    'VTOLSwarmStrikePlanner', 'VTOLAttackMode', 'VTOLTarget', 'VTOLUAV',
    'ReconToStrikeManager', 'UAVState', 'TaskMode',
    'StrikeAssignment', 'CoalitionReport',
    # Reference / experimental
    'SwarmStrikePlanner',
    'AdvancedSwarmStrikePlanner', 'AdvancedAttackMode',
]
