"""ui/widgets/indicators/fence_breach_indicator.py — Geofence 越界警示。

對應 ADOS `FenceBreachIndicator.tsx`。
越界時亮紅 + 觸 `AlertEngine.play(FAILSAFE)`（1472H §5.7.3.6.5）。
若當前 mission 無 geofence，顯示 "NO FENCE"（區別於誤觸警報）。
"""
from __future__ import annotations

from typing import Optional

from PyQt6.QtWidgets import QHBoxLayout, QLabel, QWidget

from ui.audio import AlertEngine, AlertId
from ui.resources.aeroplan_theme import tokens as T


class FenceBreachIndicator(QWidget):
    """Geofence 越界指示器。"""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 4, 8, 4)
        lay.setSpacing(6)

        self._dot = QLabel(self)
        self._dot.setProperty("role", "statusdot")
        self._dot.setProperty("severity", "success")
        self._dot.setFixedSize(10, 10)

        self._text = QLabel("FENCE OK", self)
        self._text.setStyleSheet(
            f"font-family: {T.FONT_DISPLAY_STACK}; font-weight: 700; "
            f"font-size: 11px; color: {T.FG}; letter-spacing: 0.5px;"
        )
        lay.addWidget(self._dot)
        lay.addWidget(self._text)
        lay.addStretch(1)

        self._has_fence: bool = True
        self._breached: bool = False

    def set_fence_present(self, present: bool) -> None:
        """通知是否掛載 geofence（mission/GeofenceConstraintManager 來源）。"""
        self._has_fence = bool(present)
        self._refresh()

    def set_breached(self, breached: bool) -> None:
        """設定越界狀態；首次轉為 True 時觸 FAILSAFE 警示。"""
        prev = self._breached
        self._breached = bool(breached)
        self._refresh()
        if self._breached and not prev:
            # 1472H §5.7.3.6.5：強制播放，不受 cooldown / mute 影響
            AlertEngine.instance().play(AlertId.FAILSAFE)

    def _refresh(self) -> None:
        if not self._has_fence:
            self._text.setText("NO FENCE")
            self._text.setStyleSheet(
                f"font-family: {T.FONT_DISPLAY_STACK}; font-weight: 700; "
                f"font-size: 11px; color: {T.WARNING}; letter-spacing: 0.5px;"
            )
            self._dot.setProperty("severity", "warning")
        elif self._breached:
            self._text.setText("FENCE BREACH")
            self._text.setStyleSheet(
                f"font-family: {T.FONT_DISPLAY_STACK}; font-weight: 700; "
                f"font-size: 11px; color: {T.HOSTILE}; letter-spacing: 0.5px;"
            )
            self._dot.setProperty("severity", "critical")
        else:
            self._text.setText("FENCE OK")
            self._text.setStyleSheet(
                f"font-family: {T.FONT_DISPLAY_STACK}; font-weight: 700; "
                f"font-size: 11px; color: {T.FG}; letter-spacing: 0.5px;"
            )
            self._dot.setProperty("severity", "success")
        self._dot.style().unpolish(self._dot)
        self._dot.style().polish(self._dot)


__all__ = ["FenceBreachIndicator"]
