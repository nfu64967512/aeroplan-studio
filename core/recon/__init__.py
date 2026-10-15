"""偵察任務支援模組：感測器足跡建模與覆蓋率驗算。

供 VTOL 群飛偵察頁面使用；底層路徑規劃仍沿用 core.dccpp。
"""

from core.recon.coverage_verifier import CoverageReport, verify_coverage
from core.recon.sensor_footprint import (
    CameraSpec,
    Footprint,
    FootprintError,
    altitude_for_gsd_m,
    compute_footprint,
    effective_sweep_width_m,
    gsd_m_per_px,
    swath_width_m,
)

__all__ = [
    "CameraSpec",
    "Footprint",
    "FootprintError",
    "altitude_for_gsd_m",
    "compute_footprint",
    "effective_sweep_width_m",
    "gsd_m_per_px",
    "swath_width_m",
    "CoverageReport",
    "verify_coverage",
]
