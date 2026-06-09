"""ui/widgets/fleet_rail.py — ADOS 風左側垂直 Fleet rail。

對應 ADOS Mission Control 主畫面左緣的 mini-card 列：每架 UAV 一格，
顯示 2 字母代號（callsign 縮寫）+ 狀態色條，點擊切換目前作戰機。
"""
from __future__ import annotations

from typing import Dict, Optional

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QMouseEvent
from PyQt6.QtWidgets import (
    QFrame,
    QLabel,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from mission.fleet_registry import FleetRegistry
from mission.sitl_link import TelemetryFrame
from ui.qt_utils import repolish
from ui.resources.aeroplan_theme import tokens as T


class _RailCell(QFrame):
    """單格 mini card：2 字母縮寫 + 左側狀態條。"""

    clicked = pyqtSignal(str)

    def __init__(
        self,
        callsign: str,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self._callsign = callsign
        self._selected = False
        self.setFixedSize(48, 44)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self._severity = "info"
        self._abbr = self._make_abbr(callsign)
        self._refresh_style()

    @staticmethod
    def _make_abbr(callsign: str) -> str:
        cs = callsign.upper()
        # 取首字母 + 第二段第一字（如 "HAWK-01" → "H1"、"ECHO-5" → "E5"、"FOX" → "FO"）
        if "-" in cs:
            head, tail = cs.split("-", 1)
            return f"{head[0]}{tail[0] if tail else ''}"[:2]
        if len(cs) >= 2:
            return cs[:2]
        return cs

    def _refresh_style(self) -> None:
        accent = {
            "critical": T.HOSTILE,
            "warning": T.WARNING,
            "success": T.FRIENDLY,
            "info": T.NEUTRAL,
        }.get(self._severity, T.NEUTRAL)
        if self._selected:
            border = f"1px solid {T.ACCENT_PRIMARY}"
            bg = T.BG_ELEVATED
        else:
            border = f"1px solid {T.BORDER_SUBTLE}"
            bg = T.BG_SECONDARY
        self.setStyleSheet(
            f"QFrame {{ background: {bg}; "
            f"border: {border}; "
            f"border-left: 3px solid {accent}; "
            f"border-radius: 4px; }}"
            f"QFrame:hover {{ background: {T.BG_ELEVATED}; }}"
            f"QLabel {{ background: transparent; border: none; "
            f"font-family: {T.FONT_DISPLAY_STACK}; font-weight: 700; "
            f"font-size: 16px; color: {T.FG}; letter-spacing: 1px; }}"
        )
        # 強制重設子標籤（避免 QSS 殘留）
        for lab in self.findChildren(QLabel):
            repolish(lab)

    def _setup_label(self) -> None:
        if self.layout() is None:
            v = QVBoxLayout(self)
            v.setContentsMargins(0, 0, 0, 0)
            v.setSpacing(0)
            lab = QLabel(self._abbr, self)
            lab.setAlignment(Qt.AlignmentFlag.AlignCenter)
            v.addWidget(lab)

    def set_selected(self, selected: bool) -> None:
        self._selected = bool(selected)
        self._refresh_style()

    def set_severity(self, sev: str) -> None:
        self._severity = sev
        self._refresh_style()

    def callsign(self) -> str:
        return self._callsign

    def mousePressEvent(self, evt: QMouseEvent) -> None:  # noqa: N802
        if evt.button() == Qt.MouseButton.LeftButton:
            self.clicked.emit(self._callsign)
        super().mousePressEvent(evt)

    def showEvent(self, evt) -> None:  # noqa: N802
        self._setup_label()
        super().showEvent(evt)


class FleetRail(QWidget):
    """左側 fleet 垂直導覽。

    Signals:
        current_uav_changed(str): 選機事件。
    """

    current_uav_changed = pyqtSignal(str)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setFixedWidth(60)
        self._cells: Dict[str, _RailCell] = {}
        self._current: Optional[str] = None

        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 8, 4, 8)
        outer.setSpacing(6)

        # 抬頭：FLEET
        title = QLabel("FLEET", self)
        title.setStyleSheet(
            f"font-family: {T.FONT_DISPLAY_STACK}; font-weight: 700; "
            f"font-size: 9px; color: {T.FG_MUTED}; letter-spacing: 1px; "
            f"background: transparent;"
        )
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        outer.addWidget(title)

        # 可捲動容器
        self._scroll = QScrollArea(self)
        self._scroll.setWidgetResizable(True)
        self._scroll.setFrameShape(QFrame.Shape.NoFrame)
        self._scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self._scroll.setStyleSheet("background: transparent;")
        self._cell_host = QWidget(self._scroll)
        self._cell_layout = QVBoxLayout(self._cell_host)
        self._cell_layout.setContentsMargins(0, 0, 0, 0)
        self._cell_layout.setSpacing(6)
        self._cell_layout.addStretch(1)
        self._scroll.setWidget(self._cell_host)
        outer.addWidget(self._scroll, 1)

        # 「+」加入按鈕（觸發 SITL Launcher 等流程）
        self._add_btn = QPushButton("+", self)
        self._add_btn.setFixedSize(36, 36)
        self._add_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._add_btn.setStyleSheet(
            f"QPushButton {{ background: {T.BG_ELEVATED}; "
            f"color: {T.FG_SECONDARY}; border: 1px solid {T.BORDER}; "
            f"border-radius: 4px; font-size: 18px; font-weight: 700; }}"
            f"QPushButton:hover {{ color: {T.ACCENT_PRIMARY}; "
            f"border-color: {T.ACCENT_PRIMARY}; }}"
        )
        outer.addWidget(self._add_btn, 0, Qt.AlignmentFlag.AlignHCenter)

        self.setStyleSheet(
            f"FleetRail {{ background: {T.BG_SECONDARY}; "
            f"border-right: 1px solid {T.BORDER_SUBTLE}; }}"
        )

        # 訂閱 registry
        reg = FleetRegistry.instance()
        reg.uav_registered.connect(self._on_uav_registered)
        reg.uav_unregistered.connect(self._on_uav_unregistered)
        reg.telemetry_updated.connect(self._on_telemetry)
        reg.link_status.connect(self._on_link_status)

    # ── 公開 ──────────────────────────────────────────
    def add_button_handler(self, handler) -> None:
        """為 "+" 按鈕綁定 callback（通常觸發 SITL Launcher）。"""
        self._add_btn.clicked.connect(handler)

    def current_callsign(self) -> Optional[str]:
        return self._current

    def set_current(self, callsign: str) -> None:
        if callsign not in self._cells or self._current == callsign:
            return
        self._current = callsign
        for cs, cell in self._cells.items():
            cell.set_selected(cs == callsign)
        self.current_uav_changed.emit(callsign)

    # ── Registry callbacks ─────────────────────────────
    def _on_uav_registered(self, callsign: str) -> None:
        if callsign in self._cells:
            return
        cell = _RailCell(callsign, self._cell_host)
        cell.clicked.connect(self.set_current)
        # 插入 stretch 之前
        self._cell_layout.insertWidget(self._cell_layout.count() - 1, cell)
        self._cells[callsign] = cell
        if self._current is None:
            self.set_current(callsign)

    def _on_uav_unregistered(self, callsign: str) -> None:
        cell = self._cells.pop(callsign, None)
        if cell is not None:
            self._cell_layout.removeWidget(cell)
            cell.deleteLater()
        if self._current == callsign:
            self._current = next(iter(self._cells), None)
            if self._current:
                self.current_uav_changed.emit(self._current)

    def _on_telemetry(self, callsign: str, frame: TelemetryFrame) -> None:
        cell = self._cells.get(callsign)
        if cell is None:
            return
        if frame.armed:
            cell.set_severity("critical")    # armed = 警示色
        elif frame.battery_pct >= 0 and frame.battery_pct < T.THRESHOLDS.battery_critical:
            cell.set_severity("critical")
        elif frame.battery_pct >= 0 and frame.battery_pct < T.THRESHOLDS.battery_low:
            cell.set_severity("warning")
        else:
            cell.set_severity("success")

    def _on_link_status(self, callsign: str, status: str) -> None:
        cell = self._cells.get(callsign)
        if cell is None:
            return
        if status in ("disconnected", "lost"):
            cell.set_severity("critical")


__all__ = ["FleetRail"]
