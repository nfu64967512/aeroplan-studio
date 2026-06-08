"""ui/widgets/indicators — ADOS-style status indicators bundle.

對應 ADOS `src/components/indicators/*`，僅移植 6 個高價值低成本元件。
全部訂閱 `mission.fleet_registry.FleetRegistry.telemetry_updated`。
"""
from .connection_quality_meter import ConnectionQualityMeter
from .ekf_status_bars import EkfStatusBars
from .gps_sky_view import GpsSkyView
from .fence_breach_indicator import FenceBreachIndicator
from .nav_state_pill import NavStatePill
from .prearm_checks import PreArmChecks

__all__ = [
    "ConnectionQualityMeter",
    "EkfStatusBars",
    "GpsSkyView",
    "FenceBreachIndicator",
    "NavStatePill",
    "PreArmChecks",
]
