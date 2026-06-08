"""ui/widgets/indicators/connection_quality_meter.py — 鏈路品質計。

合成 RSSI / packet loss / 延遲為 0–100% 條。對應 ADOS `ConnectionQualityMeter.tsx`。
"""
from __future__ import annotations

from typing import Optional

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QHBoxLayout, QLabel, QProgressBar, QWidget

from mission.fleet_registry import FleetRegistry
from ui.resources.aeroplan_theme import tokens as T


class ConnectionQualityMeter(QWidget):
    """鏈路品質：以最近一筆 telemetry 的 age + 收到頻率合成 0–100%。

    無真實 RSSI 欄位時退化使用 freshness（age sec）：
    - age < 0.5s → 100%
    - 0.5 ≤ age < 2 → 60%
    - 2 ≤ age < 5 → 30%
    - age ≥ 5 → 0%（顯示 LOST）
    """

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)

        self._label = QLabel("LNK", self)
        self._label.setStyleSheet(
            f"font-family: {T.FONT_DISPLAY_STACK}; font-size: 10px; "
            f"color: {T.FG_SECONDARY}; letter-spacing: 0.5px;"
        )
        self._bar = QProgressBar(self)
        self._bar.setRange(0, 100)
        self._bar.setValue(0)
        self._bar.setTextVisible(False)
        self._bar.setFixedHeight(8)
        self._bar.setProperty("status", "nominal")

        self._pct = QLabel("--%", self)
        self._pct.setStyleSheet(
            f"font-family: {T.FONT_MONO_STACK}; font-weight: 700; "
            f"font-size: 11px; color: {T.FG};"
        )
        self._pct.setFixedWidth(36)
        self._pct.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)

        lay.addWidget(self._label)
        lay.addWidget(self._bar, 1)
        lay.addWidget(self._pct)

    def update_for(self, callsign: str) -> None:
        """以 FleetRegistry 對應 callsign 的最新 freshness 更新顯示。"""
        age = FleetRegistry.instance().latest_age_sec(callsign)
        if age == float("inf"):
            pct = 0
        elif age < 0.5:
            pct = 100
        elif age < 2.0:
            pct = 60
        elif age < 5.0:
            pct = 30
        else:
            pct = 0

        self._bar.setValue(pct)
        if pct >= 60:
            self._bar.setProperty("status", "nominal")
        elif pct >= 30:
            self._bar.setProperty("status", "warning")
        else:
            self._bar.setProperty("status", "critical")
        self._bar.style().unpolish(self._bar)
        self._bar.style().polish(self._bar)
        self._pct.setText(f"{pct}%" if pct > 0 else "LOST")


__all__ = ["ConnectionQualityMeter"]
