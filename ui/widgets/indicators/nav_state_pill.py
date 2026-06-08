"""ui/widgets/indicators/nav_state_pill.py — 飛行模式 + armed 緊湊膠囊。

對應 ADOS `NavStatePill.tsx`。
"""
from __future__ import annotations

from typing import Optional

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QHBoxLayout, QLabel, QWidget

from mission.sitl_link import TelemetryFrame
from ui.resources.aeroplan_theme import tokens as T


class NavStatePill(QWidget):
    """單顆「模式 + 武裝狀態」膠囊。

    用法::

        pill = NavStatePill()
        pill.update_from_frame(frame)   # 接 telemetry_updated signal
    """

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 2, 8, 2)
        lay.setSpacing(6)

        self._dot = QLabel(self)
        self._dot.setProperty("role", "statusdot")
        self._dot.setProperty("severity", "info")
        self._dot.setFixedSize(8, 8)

        self._mode = QLabel("---", self)
        self._mode.setStyleSheet(
            f"font-family: {T.FONT_DISPLAY_STACK}; font-weight: 700;"
            f"font-size: 11px; color: {T.FG}; letter-spacing: 0.5px;"
        )
        self._armed = QLabel("DISARMED", self)
        self._armed.setStyleSheet(
            f"font-family: {T.FONT_MONO_STACK}; font-weight: 700;"
            f"font-size: 10px; color: {T.FRIENDLY};"
        )

        lay.addWidget(self._dot, 0, Qt.AlignmentFlag.AlignVCenter)
        lay.addWidget(self._mode)
        lay.addStretch(1)
        lay.addWidget(self._armed)

        self.setStyleSheet(
            f"QWidget {{ background: {T.BG_ELEVATED}; "
            f"border: 1px solid {T.BORDER_SUBTLE}; border-radius: 10px; }}"
        )
        self.setMinimumHeight(20)

    def update_from_frame(self, frame: TelemetryFrame) -> None:
        """以 TelemetryFrame 更新 pill 顯示。"""
        self._mode.setText(frame.mode or "---")
        if frame.armed:
            self._armed.setText("ARMED")
            self._armed.setStyleSheet(
                f"font-family: {T.FONT_MONO_STACK}; font-weight: 700;"
                f"font-size: 10px; color: {T.HOSTILE};"
            )
            self._dot.setProperty("severity", "critical")
        else:
            self._armed.setText("DISARMED")
            self._armed.setStyleSheet(
                f"font-family: {T.FONT_MONO_STACK}; font-weight: 700;"
                f"font-size: 10px; color: {T.FRIENDLY};"
            )
            self._dot.setProperty("severity", "success")
        # 重新套用 QSS 屬性
        self._dot.style().unpolish(self._dot)
        self._dot.style().polish(self._dot)


__all__ = ["NavStatePill"]
