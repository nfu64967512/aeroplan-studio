"""ui/widgets/indicators/ekf_status_bars.py — EKF 健康度 5 條。

對應 ADOS `EkfStatusBars.tsx`：
velocity_variance / pos_horiz / pos_vert / compass / terrain。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from PyQt6.QtWidgets import QGridLayout, QLabel, QProgressBar, QWidget

from ui.resources.aeroplan_theme import tokens as T


@dataclass
class EkfStatus:
    """EKF_STATUS_REPORT 子集；值愈低愈好（variance）。"""
    velocity_variance: float = 0.0
    pos_horiz_variance: float = 0.0
    pos_vert_variance: float = 0.0
    compass_variance: float = 0.0
    terrain_alt_variance: float = 0.0


class EkfStatusBars(QWidget):
    """5 條健康度水平棒。variance 0.0 → 100%、0.5 → 50%、≥1.0 → 0%。"""

    _LABELS = ["VEL", "POS-H", "POS-V", "MAG", "TERR"]

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        grid = QGridLayout(self)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(2)

        self._bars: list[QProgressBar] = []
        for i, name in enumerate(self._LABELS):
            lab = QLabel(name, self)
            lab.setStyleSheet(
                f"font-family: {T.FONT_DISPLAY_STACK}; font-size: 9px; "
                f"color: {T.FG_SECONDARY}; letter-spacing: 0.5px;"
            )
            bar = QProgressBar(self)
            bar.setRange(0, 100)
            bar.setTextVisible(False)
            bar.setFixedHeight(6)
            bar.setProperty("status", "nominal")
            self._bars.append(bar)
            grid.addWidget(lab, i, 0)
            grid.addWidget(bar, i, 1)
        grid.setColumnStretch(1, 1)

    def update_from_status(self, st: EkfStatus) -> None:
        """以 EkfStatus 更新 5 條。"""
        variances = [
            st.velocity_variance, st.pos_horiz_variance, st.pos_vert_variance,
            st.compass_variance, st.terrain_alt_variance,
        ]
        for bar, v in zip(self._bars, variances):
            pct = max(0, min(100, int(round((1.0 - v) * 100))))
            bar.setValue(pct)
            if pct >= 60:
                bar.setProperty("status", "nominal")
            elif pct >= 30:
                bar.setProperty("status", "warning")
            else:
                bar.setProperty("status", "critical")
            bar.style().unpolish(bar)
            bar.style().polish(bar)


__all__ = ["EkfStatus", "EkfStatusBars"]
