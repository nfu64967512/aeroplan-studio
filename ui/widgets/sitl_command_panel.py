"""ui/widgets/sitl_command_panel.py — ADOS 風 SITL 分頁主控版面。

對應截圖 Command 頁的「rail + drone detail」精簡版（去掉 map / flight logs
兩欄，因為 legacy MainWindow 已有獨立 map 與 status bar）。

對齊截圖視覺：
  ┌────────────────────────────────────────┐
  │ FLEET  [AL][BR][E5][CH][DE][FO][GO][+] │  ← 橫向 mini rail
  ├────────────────────────────────────────┤
  │ ECHO-5  IN MISSION    ...  Reboot FC   │
  │ Overview Flights Calibrate Param Conf  │
  │ ─────────────────────────────────────  │
  │   62.6m   36.0    304°    -2.4         │  ← 大讀數
  │   [  DISARM  ]   [ AUTO  ▾ ]           │
  │   [‖][⌂][↑10][↥][↧][✕]                │
  │   [ Follow Me ]                        │
  │   HEALTH chips ... VEHICLE INFO ...    │
  └────────────────────────────────────────┘

互動：所有按鈕透過 `FleetRegistry` 取 `SITLLink`，發 MAVLink 指令到 SITL。
此 widget 由 `main_window.MainWindow` 在 SITL 分頁中載入；
SITLLink 由 `MainWindow.on_sitl_connect()` 同時註冊到 FleetRegistry。
"""
from __future__ import annotations

from typing import Dict, Optional

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QScrollArea,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from mission.fleet_registry import FleetRegistry
from mission.sitl_link import TelemetryFrame
from ui.resources.aeroplan_theme import tokens as T
from ui.widgets.drone_detail_panel import DroneDetailPanel
from ui.widgets.fleet_rail import _RailCell


class _HorizontalRail(QWidget):
    """橫向版 fleet rail：mini cards 一字排開。

    與 ui.widgets.fleet_rail.FleetRail 不同：rail 直向適合主視窗左緣，
    本元件橫向、適合放在 SITL 分頁頂部。
    """

    current_uav_changed = pyqtSignal(str)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._cells: Dict[str, _RailCell] = {}
        self._current: Optional[str] = None
        self.setFixedHeight(56)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setStyleSheet(
            f"_HorizontalRail {{ background: {T.BG_SECONDARY}; "
            f"border-bottom: 1px solid {T.BORDER_SUBTLE}; }}"
        )

        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 6, 8, 6)
        lay.setSpacing(6)

        head = QLabel("FLEET", self)
        head.setStyleSheet(
            f"font-family: {T.FONT_DISPLAY_STACK}; font-weight: 700; "
            f"font-size: 10px; color: {T.FG_MUTED}; letter-spacing: 1px; "
            f"background: transparent;"
        )
        lay.addWidget(head)

        # cells container
        self._cells_host = QWidget(self)
        self._cells_lay = QHBoxLayout(self._cells_host)
        self._cells_lay.setContentsMargins(0, 0, 0, 0)
        self._cells_lay.setSpacing(4)
        self._cells_lay.addStretch(1)
        lay.addWidget(self._cells_host, 1)

        # 訂閱 registry
        reg = FleetRegistry.instance()
        reg.uav_registered.connect(self._on_uav_registered)
        reg.uav_unregistered.connect(self._on_uav_unregistered)
        reg.telemetry_updated.connect(self._on_telemetry)
        reg.link_status.connect(self._on_link_status)
        # 預先補進已註冊的 UAV
        for cs in reg.callsigns():
            self._on_uav_registered(cs)

    def current_callsign(self) -> Optional[str]:
        return self._current

    def set_current(self, callsign: str) -> None:
        if callsign not in self._cells or self._current == callsign:
            return
        self._current = callsign
        for cs, cell in self._cells.items():
            cell.set_selected(cs == callsign)
        self.current_uav_changed.emit(callsign)

    def _on_uav_registered(self, callsign: str) -> None:
        if callsign in self._cells:
            return
        cell = _RailCell(callsign, self._cells_host)
        cell.clicked.connect(self.set_current)
        # 插到 stretch 前面
        self._cells_lay.insertWidget(self._cells_lay.count() - 1, cell)
        self._cells[callsign] = cell
        if self._current is None:
            self.set_current(callsign)

    def _on_uav_unregistered(self, callsign: str) -> None:
        cell = self._cells.pop(callsign, None)
        if cell is not None:
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
            cell.set_severity("critical")
        elif (frame.battery_pct >= 0
                and frame.battery_pct < T.THRESHOLDS.battery_critical):
            cell.set_severity("critical")
        elif (frame.battery_pct >= 0
                and frame.battery_pct < T.THRESHOLDS.battery_low):
            cell.set_severity("warning")
        else:
            cell.set_severity("success")

    def _on_link_status(self, callsign: str, status: str) -> None:
        cell = self._cells.get(callsign)
        if cell is None:
            return
        if status in ("disconnected", "lost"):
            cell.set_severity("critical")


class SitlCommandPanel(QWidget):
    """SITL 分頁主控版面：橫向 rail + DroneDetailPanel。

    Signals:
        current_uav_changed(str): 對外通知選機。
    """

    current_uav_changed = pyqtSignal(str)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # 頂部橫向 rail
        self.rail = _HorizontalRail(self)
        root.addWidget(self.rail)

        # DroneDetailPanel 包進 scroll
        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.detail = DroneDetailPanel(scroll)
        scroll.setWidget(self.detail)
        root.addWidget(scroll, 1)

        # rail → detail 串接
        self.rail.current_uav_changed.connect(self._on_rail_selected)

    def _on_rail_selected(self, callsign: str) -> None:
        self.detail.set_callsign(callsign)
        self.current_uav_changed.emit(callsign)


__all__ = ["SitlCommandPanel"]
