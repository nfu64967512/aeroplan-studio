"""ui/widgets/flight_logs_panel.py — ADOS 風 Flight Logs 終端機。

對應截圖右側 FLIGHT LOGS 面板：時間欄 + severity 標籤 + 訊息。
訂閱所有 SITLLink 的 `status_text` signal（MAVLink severity 0-7）。

severity → 顏色（1472H 對齊）：
    0/1/2/3 (EMERGENCY/ALERT/CRITICAL/ERROR) → 紅
    4 (WARNING) → 琥珀
    5 (NOTICE) → 青
    6 (INFO) → 灰 / debug 綠
    7 (DEBUG) → 灰
"""
from __future__ import annotations

from datetime import datetime
from typing import Optional

from PyQt6.QtCore import Qt, pyqtSlot
from PyQt6.QtWidgets import (
    QComboBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QVBoxLayout,
    QWidget,
)

from mission.fleet_registry import FleetRegistry
from ui.resources.aeroplan_theme import tokens as T
from ui.resources.aeroplan_theme.buttons import (
    ButtonSize,
    ButtonVariant,
    make_button,
)


_SEV_NAMES = {
    0: ("EMERG", T.HOSTILE),
    1: ("ALERT", T.HOSTILE),
    2: ("CRIT",  T.HOSTILE),
    3: ("ERROR", T.HOSTILE),
    4: ("WARN",  T.WARNING),
    5: ("NOTICE", T.NEUTRAL),
    6: ("INFO",  T.FG_SECONDARY),
    7: ("DEBUG", T.FG_MUTED),
}


class FlightLogsPanel(QWidget):
    """Flight logs 顯示器。"""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._connected_callsigns: set[str] = set()

        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(6)

        head = QLabel("FLIGHT LOGS", self)
        head.setStyleSheet(
            f"font-family: {T.FONT_DISPLAY_STACK}; font-weight: 700; "
            f"font-size: 12px; color: {T.FG}; letter-spacing: 0.8px; "
            f"background: transparent;"
        )
        outer.addWidget(head)

        # 工具列：severity filter + search + auto / clear
        tools = QHBoxLayout()
        tools.setContentsMargins(0, 0, 0, 0)
        tools.setSpacing(4)
        self._cmb_level = QComboBox(self)
        self._cmb_level.addItems(["DEBUG", "INFO", "NOTICE", "WARN", "ERROR"])
        self._cmb_level.setCurrentText("DEBUG")
        self._cmb_level.currentIndexChanged.connect(lambda *_: self._refilter())
        tools.addWidget(self._cmb_level)

        self._cmb_source = QComboBox(self)
        self._cmb_source.addItems(["All Messages"])
        tools.addWidget(self._cmb_source)

        tools.addStretch(1)

        self._btn_auto = make_button("Auto", variant=ButtonVariant.OUTLINE, size=ButtonSize.XS)
        self._btn_auto.setCheckable(True)
        self._btn_auto.setChecked(True)
        tools.addWidget(self._btn_auto)

        self._btn_clear = make_button("Clear", variant=ButtonVariant.GHOST, size=ButtonSize.XS)
        self._btn_clear.clicked.connect(self.clear)
        tools.addWidget(self._btn_clear)

        outer.addLayout(tools)

        # 搜尋
        self._search = QLineEdit(self)
        self._search.setPlaceholderText("Search messages …")
        self._search.textChanged.connect(self._refilter)
        outer.addWidget(self._search)

        # 列表
        self._list = QListWidget(self)
        self._list.setStyleSheet(
            f"QListWidget {{ background: {T.BG_PRIMARY}; "
            f"color: {T.FG}; border: 1px solid {T.BORDER_SUBTLE}; "
            f"border-radius: 4px; "
            f"font-family: {T.FONT_MONO_STACK}; font-size: 11px; }}"
            f"QListWidget::item {{ padding: 2px 6px; }}"
            f"QListWidget::item:alternate {{ background: {T.BG_SECONDARY}; }}"
        )
        self._list.setAlternatingRowColors(True)
        self._list.setSelectionMode(QListWidget.SelectionMode.NoSelection)
        outer.addWidget(self._list, 1)

        # 訂閱現有 + 未來的 link
        reg = FleetRegistry.instance()
        reg.uav_registered.connect(self._on_uav_registered)
        # 對既有的也接一次
        for cs in reg.callsigns():
            self._on_uav_registered(cs)

        self.setStyleSheet(
            f"FlightLogsPanel {{ background: {T.BG_PRIMARY}; "
            f"border-left: 1px solid {T.BORDER_SUBTLE}; }}"
        )

    # ── 公開 ─────────────────────────────────────────
    def clear(self) -> None:
        self._list.clear()

    # ── Registry hook ────────────────────────────────
    def _on_uav_registered(self, callsign: str) -> None:
        if callsign in self._connected_callsigns:
            return
        link = FleetRegistry.instance().get_link(callsign)
        if link is None:
            return
        link.status_text.connect(
            lambda sev, text, cs=callsign: self._append(cs, sev, text)
        )
        self._connected_callsigns.add(callsign)
        # 補進 source filter
        if self._cmb_source.findText(callsign) < 0:
            self._cmb_source.addItem(callsign)

    # ── 寫入 ─────────────────────────────────────────
    @pyqtSlot(str, int, str)
    def _append(self, callsign: str, severity: int, text: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S")
        sev_name, sev_color = _SEV_NAMES.get(int(severity), ("INFO", T.FG_SECONDARY))
        item = QListWidgetItem(f"{ts}  {sev_name:<6}  {callsign:<8}  {text}")
        item.setForeground(sev_color and item.foreground())  # 整列維持預設色
        item.setData(Qt.ItemDataRole.UserRole, (severity, callsign, text))
        # 重要訊息用顏色標
        if severity <= 3:
            from PyQt6.QtGui import QColor
            item.setForeground(QColor(T.HOSTILE))
        elif severity == 4:
            from PyQt6.QtGui import QColor
            item.setForeground(QColor(T.WARNING))
        elif severity == 5:
            from PyQt6.QtGui import QColor
            item.setForeground(QColor(T.NEUTRAL))
        self._list.addItem(item)
        # 自動捲到底
        if self._btn_auto.isChecked():
            self._list.scrollToBottom()
        # 上限 2000 筆
        if self._list.count() > 2000:
            self._list.takeItem(0)
        self._apply_filter_to(item)

    # ── 篩選 ─────────────────────────────────────────
    def _refilter(self) -> None:
        for i in range(self._list.count()):
            self._apply_filter_to(self._list.item(i))

    def _apply_filter_to(self, item: QListWidgetItem) -> None:
        if item is None:
            return
        meta = item.data(Qt.ItemDataRole.UserRole)
        if not meta:
            return
        sev, cs, text = meta
        min_sev = {
            "DEBUG": 7, "INFO": 6, "NOTICE": 5, "WARN": 4, "ERROR": 3,
        }.get(self._cmb_level.currentText(), 7)
        show = int(sev) <= min_sev
        if self._cmb_source.currentText() != "All Messages":
            show = show and (cs == self._cmb_source.currentText())
        q = self._search.text().strip().lower()
        if q and q not in text.lower():
            show = False
        item.setHidden(not show)


__all__ = ["FlightLogsPanel"]
