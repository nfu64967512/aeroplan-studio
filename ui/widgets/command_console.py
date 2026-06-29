"""ui/widgets/command_console.py — ADOS 風 Command 主控台。

對應截圖整體版面：
    ┌──────┬────────────────────────────────┬──────────────┐
    │      │ Echo-5  [IN MISSION] ... [Reboot]│              │
    │ FLEET│ Overview Flights ... Configure   │ FLIGHT LOGS  │
    │ rail │ ───────────────────────────────  │              │
    │      │ [DetailPanel │ Map (Map/Fly)]   │              │
    └──────┴────────────────────────────────┴──────────────┘

DetailPanel 內含大型讀數 + 模式切換 + 動作鈕 + HEALTH + IDENTITY。
Map 區位由主視窗注入既有的 `DualMapWidget` 或 `MapWidget`；缺席時以 placeholder。

互動 = GCS：所有按鈕（ARM/DISARM/Mode/Pause/RTL/Loiter/Reboot/Follow/ChangeAlt）
經 `DroneDetailPanel.command_sent` signal 由內部已連到對應 SITLLink 方法，
真實對著 SITL 飛機發 MAVLink。
"""
from __future__ import annotations

from typing import Optional

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from ui.resources.aeroplan_theme import tokens as T
from ui.widgets.drone_detail_panel import DroneDetailPanel
from ui.widgets.fleet_rail import FleetRail
from ui.widgets.flight_logs_panel import FlightLogsPanel


class CommandConsole(QWidget):
    """ADOS 風 Command 主控台。

    Signals:
        current_uav_changed(str): 對外通知選機（給其他 page 同步）。
    """

    current_uav_changed = pyqtSignal(str)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)

        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ── 左：FleetRail ───────────────────────────
        self.rail = FleetRail(self)
        root.addWidget(self.rail)

        # ── 中：detail + map splitter ─────────────────
        center = QWidget(self)
        center_lay = QVBoxLayout(center)
        center_lay.setContentsMargins(0, 0, 0, 0)
        center_lay.setSpacing(0)

        # 上方 horizontal splitter（detail | map）
        h_split = QSplitter(Qt.Orientation.Horizontal, center)
        h_split.setHandleWidth(1)
        h_split.setChildrenCollapsible(False)

        self.detail = DroneDetailPanel(h_split)
        self.detail.setMinimumWidth(360)
        self.detail.setMaximumWidth(440)
        h_split.addWidget(self.detail)

        # Map host：允許外部 set_map_widget 注入
        self._map_host = QWidget(h_split)
        self._map_layout = QVBoxLayout(self._map_host)
        self._map_layout.setContentsMargins(0, 0, 0, 0)
        self._map_layout.setSpacing(0)
        self._map_placeholder = QLabel(
            "MAP / FLY VIEW\n\n— assign via set_map_widget() —", self._map_host
        )
        self._map_placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._map_placeholder.setStyleSheet(
            f"color: {T.FG_MUTED}; background: {T.BG_SECONDARY}; "
            f"border: 1px dashed {T.BORDER}; "
            f"font-family: {T.FONT_DISPLAY_STACK}; font-size: 11px; "
            f"letter-spacing: 0.8px;"
        )
        self._map_layout.addWidget(self._map_placeholder)
        h_split.addWidget(self._map_host)

        h_split.setStretchFactor(0, 0)
        h_split.setStretchFactor(1, 1)
        h_split.setSizes([400, 1000])

        center_lay.addWidget(h_split, 1)
        root.addWidget(center, 1)

        # ── 右：FlightLogsPanel ─────────────────────
        self.logs = FlightLogsPanel(self)
        self.logs.setMinimumWidth(280)
        self.logs.setMaximumWidth(420)
        root.addWidget(self.logs)

        # 對接 rail → detail
        self.rail.current_uav_changed.connect(self._on_rail_selected)

    # ── 公開 ─────────────────────────────────────────
    def set_map_widget(self, w: QWidget) -> None:
        """主視窗注入既有 map widget。"""
        self._map_layout.removeWidget(self._map_placeholder)
        self._map_placeholder.hide()
        self._map_layout.addWidget(w)

    def add_button_handler(self, handler) -> None:
        """為 rail "+" 按鈕綁定 callback（呼叫 SITL Launcher）。"""
        self.rail.add_button_handler(handler)

    # ── 內部 ─────────────────────────────────────────
    def _on_rail_selected(self, callsign: str) -> None:
        self.detail.set_callsign(callsign)
        self.current_uav_changed.emit(callsign)


__all__ = ["CommandConsole"]
