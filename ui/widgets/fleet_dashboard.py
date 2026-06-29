"""ui/widgets/fleet_dashboard.py — ADOS 風格機隊儀錶板聚合器。

包含工具列（搜尋 / 篩選 / 排序 / Grid|List|Map 切換）、5 張統計卡、
與 Grid / List / Map 三種主體呈現。

訂閱 `mission.fleet_registry.FleetRegistry.telemetry_updated`，
依 callsign 將最新 telemetry 路由到對應 `FleetCard`。

主視窗點卡片 → 發 `current_uav_changed(callsign)`。
"""
from __future__ import annotations

from typing import Dict, Optional

from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QShortcut, QKeySequence
from PyQt6.QtWidgets import (
    QComboBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QStackedWidget,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from mission.fleet_registry import FleetRegistry
from mission.sitl_link import TelemetryFrame
from ui.resources.aeroplan_theme import tokens as T
from ui.resources.aeroplan_theme.buttons import (
    ButtonSize,
    ButtonVariant,
    make_button,
)
from ui.widgets.fleet_card import FleetCard


class _StatCard(QFrame):
    """單張統計卡：標籤 + 大型 stat 值 + 可選 delta。"""

    def __init__(self, label: str, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setProperty("role", "card")
        self.setFixedHeight(70)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 8, 12, 8)
        lay.setSpacing(2)
        self._label = QLabel(label, self)
        self._label.setProperty("role", "stat-label")
        self._value = QLabel("--", self)
        self._value.setProperty("role", "stat-value")
        lay.addWidget(self._label)
        lay.addWidget(self._value)

    def set_value(self, value: str) -> None:
        self._value.setText(value)


class FleetDashboard(QWidget):
    """機隊儀錶板（Dashboard 頁）。"""

    current_uav_changed = pyqtSignal(str)   # callsign

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._cards: Dict[str, FleetCard] = {}
        self._current: Optional[str] = None

        root = QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        # ── 工具列 ─────────────────────────────────────────
        toolbar = QHBoxLayout()
        toolbar.setContentsMargins(0, 0, 0, 0)
        toolbar.setSpacing(6)

        self._search = QLineEdit(self)
        self._search.setPlaceholderText("Search callsign / mode …")
        self._search.setMinimumWidth(220)
        self._search.textChanged.connect(self._refresh_cards)
        toolbar.addWidget(self._search)

        self._filter = QComboBox(self)
        self._filter.addItems(["All", "Armed", "Failsafe", "Lost"])
        self._filter.currentIndexChanged.connect(self._refresh_cards)
        toolbar.addWidget(self._filter)

        self._sort = QComboBox(self)
        self._sort.addItems(["Callsign", "Battery", "Link", "Mode"])
        self._sort.currentIndexChanged.connect(self._refresh_cards)
        toolbar.addWidget(self._sort)

        toolbar.addStretch(1)

        for label, idx in (("Grid", 0), ("List", 1), ("Map", 2)):
            btn = make_button(
                label, variant=ButtonVariant.OUTLINE, size=ButtonSize.SM,
            )
            btn.setCheckable(True)
            btn.clicked.connect(lambda _=False, i=idx: self._set_view(i))
            toolbar.addWidget(btn)
            if idx == 0:
                btn.setChecked(True)
                self._grid_btn = btn

        root.addLayout(toolbar)

        # ── 統計列 ─────────────────────────────────────────
        stats_row = QHBoxLayout()
        stats_row.setContentsMargins(0, 0, 0, 0)
        stats_row.setSpacing(8)
        self._stat_active = _StatCard("ACTIVE DRONES", self)
        self._stat_battery = _StatCard("AVG BATTERY", self)
        self._stat_missions = _StatCard("ACTIVE MISSIONS", self)
        self._stat_alerts = _StatCard("ALERTS (24H)", self)
        self._stat_link = _StatCard("AVG LINK", self)
        for c in (
            self._stat_active, self._stat_battery, self._stat_missions,
            self._stat_alerts, self._stat_link,
        ):
            stats_row.addWidget(c)
        root.addLayout(stats_row)

        # ── 主體 stacked ───────────────────────────────────
        self._stack = QStackedWidget(self)

        # Grid 頁
        self._grid_host = QWidget(self)
        self._grid_layout = QGridLayout(self._grid_host)
        self._grid_layout.setContentsMargins(0, 0, 0, 0)
        self._grid_layout.setHorizontalSpacing(10)
        self._grid_layout.setVerticalSpacing(10)
        self._grid_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        self._stack.addWidget(self._grid_host)

        # List 頁
        self._table = QTableWidget(self)
        self._table.setColumnCount(7)
        self._table.setHorizontalHeaderLabels(
            ["Callsign", "Mode", "Armed", "BAT%", "GPS", "LNK%", "Mission"]
        )
        self._table.verticalHeader().setVisible(False)
        self._table.setEditTriggers(self._table.EditTrigger.NoEditTriggers)
        self._table.cellClicked.connect(self._on_table_clicked)
        self._stack.addWidget(self._table)

        # Map 頁（保留為 stub，主視窗會餵真實 map widget 過來）
        self._map_host = QWidget(self)
        self._map_layout = QVBoxLayout(self._map_host)
        self._map_layout.setContentsMargins(0, 0, 0, 0)
        self._map_placeholder = QLabel("Map view — assign via set_map_widget()", self)
        self._map_placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._map_placeholder.setStyleSheet(
            f"color: {T.FG_MUTED}; background: {T.BG_SECONDARY}; "
            f"border: 1px dashed {T.BORDER}; border-radius: 6px;"
        )
        self._map_layout.addWidget(self._map_placeholder)
        self._stack.addWidget(self._map_host)

        root.addWidget(self._stack, 1)

        # ── 訂閱 FleetRegistry ─────────────────────────────
        reg = FleetRegistry.instance()
        reg.uav_registered.connect(self._on_uav_registered)
        reg.uav_unregistered.connect(self._on_uav_unregistered)
        reg.telemetry_updated.connect(self._on_telemetry)
        reg.link_status.connect(self._on_link_status)

        # 響應式欄數：resizeEvent 動態算
        self._cols = 4

        # 統計列輪詢
        self._stats_timer = QTimer(self)
        self._stats_timer.setInterval(500)
        self._stats_timer.timeout.connect(self._refresh_stats)
        self._stats_timer.start()

        # Ctrl+1..9 快速切換
        for i in range(1, 10):
            QShortcut(QKeySequence(f"Ctrl+{i}"), self, activated=lambda idx=i - 1: self._quick_select(idx))

    # ── 公開 API ──────────────────────────────────────────────
    def set_map_widget(self, w: QWidget) -> None:
        """主視窗注入既有 map widget（dual_map_widget 等）。"""
        self._map_layout.removeWidget(self._map_placeholder)
        self._map_placeholder.hide()
        self._map_layout.addWidget(w)

    # ── Registry 訊號處理 ──────────────────────────────────────
    def _on_uav_registered(self, callsign: str) -> None:
        link = FleetRegistry.instance().get_link(callsign)
        sysid = 0
        if link is not None:
            # 公開 sysid 來自 link.sysid_label，非不存在的 _frame 屬性。
            sysid = int(getattr(link, "sysid_label", 0) or 0)
        card = FleetCard(callsign, sysid=sysid, parent=self)
        card.clicked.connect(self._on_card_clicked)
        card.context_action.connect(self._on_context_action)
        self._cards[callsign] = card
        self._refresh_cards()
        self._refresh_table()
        # 預設第一機為 current
        if self._current is None:
            self._set_current(callsign)

    def _on_uav_unregistered(self, callsign: str) -> None:
        card = self._cards.pop(callsign, None)
        if card is not None:
            card.deleteLater()
        if self._current == callsign:
            self._current = next(iter(self._cards), None)
            if self._current:
                self.current_uav_changed.emit(self._current)
        self._refresh_cards()
        self._refresh_table()

    def _on_telemetry(self, callsign: str, frame: TelemetryFrame) -> None:
        card = self._cards.get(callsign)
        if card is None:
            return
        card.update_from_frame(frame)
        # 鏈路品質：以 latest_age 推算
        age = FleetRegistry.instance().latest_age_sec(callsign)
        if age < 0.5:
            pct = 100
        elif age < 2.0:
            pct = 60
        elif age < 5.0:
            pct = 30
        else:
            pct = 0
        card.set_link_quality(pct)

    def _on_link_status(self, callsign: str, status: str) -> None:
        card = self._cards.get(callsign)
        if card is None:
            return
        if status in ("disconnected", "lost"):
            card.set_lost()

    # ── UI 互動 ───────────────────────────────────────────────
    def _on_card_clicked(self, callsign: str) -> None:
        self._set_current(callsign)

    def _on_context_action(self, callsign: str, action: str) -> None:
        if action == "set_active":
            self._set_current(callsign)
        # 其他動作交給上層由 Strike controller 或主視窗處理（保留訊號鉤）

    def _set_current(self, callsign: str) -> None:
        if self._current == callsign:
            return
        self._current = callsign
        for cs, card in self._cards.items():
            card.set_selected(cs == callsign)
        self.current_uav_changed.emit(callsign)

    def _quick_select(self, idx: int) -> None:
        cs_list = list(self._cards.keys())
        if 0 <= idx < len(cs_list):
            self._set_current(cs_list[idx])

    def _set_view(self, idx: int) -> None:
        self._stack.setCurrentIndex(idx)
        if idx == 1:
            self._refresh_table()

    def _on_table_clicked(self, row: int, _col: int) -> None:
        item = self._table.item(row, 0)
        if item:
            self._set_current(item.text())

    # ── 重排 ──────────────────────────────────────────────────
    def resizeEvent(self, evt) -> None:  # noqa: N802
        super().resizeEvent(evt)
        # 響應式欄數：1280/960/640/480 → 4/3/2/1
        w = self.width()
        if w >= 1280:
            cols = 4
        elif w >= 960:
            cols = 3
        elif w >= 640:
            cols = 2
        else:
            cols = 1
        if cols != self._cols:
            self._cols = cols
            self._refresh_cards()

    def _refresh_cards(self) -> None:
        # 清空 grid
        while self._grid_layout.count():
            it = self._grid_layout.takeAt(0)
            if it.widget():
                it.widget().hide()
        # 篩選
        q = (self._search.text() or "").lower()
        fmode = self._filter.currentText()
        items = []
        for cs, card in self._cards.items():
            text_blob = f"{cs}".lower()
            if q and q not in text_blob:
                continue
            items.append((cs, card))
        # 排序：簡化版（只 by callsign）
        items.sort(key=lambda x: x[0])
        # 放入 grid
        for i, (_cs, card) in enumerate(items):
            r, c = divmod(i, self._cols)
            self._grid_layout.addWidget(card, r, c)
            card.show()

    def _refresh_table(self) -> None:
        self._table.setRowCount(len(self._cards))
        for row, (cs, _card) in enumerate(self._cards.items()):
            frame = FleetRegistry.instance().latest(cs)
            self._table.setItem(row, 0, QTableWidgetItem(cs))
            if frame is None:
                continue
            self._table.setItem(row, 1, QTableWidgetItem(frame.mode))
            self._table.setItem(row, 2, QTableWidgetItem("ARMED" if frame.armed else "DISARMED"))
            self._table.setItem(row, 3, QTableWidgetItem(str(frame.battery_pct)))
            self._table.setItem(row, 4, QTableWidgetItem(f"{frame.gps_fix}/{frame.gps_sats}"))
            age = FleetRegistry.instance().latest_age_sec(cs)
            lnk = 0 if age == float("inf") else (100 if age < 0.5 else 60 if age < 2 else 30 if age < 5 else 0)
            self._table.setItem(row, 5, QTableWidgetItem(str(lnk)))
            self._table.setItem(row, 6, QTableWidgetItem(""))

    def _refresh_stats(self) -> None:
        active = len(self._cards)
        self._stat_active.set_value(str(active))
        if active == 0:
            self._stat_battery.set_value("--%")
            self._stat_link.set_value("--%")
            self._stat_missions.set_value("0")
            self._stat_alerts.set_value("0")
            return
        bat = 0
        bat_n = 0
        link = 0
        link_n = 0
        for cs in self._cards:
            f = FleetRegistry.instance().latest(cs)
            if f and f.battery_pct >= 0:
                bat += f.battery_pct
                bat_n += 1
            age = FleetRegistry.instance().latest_age_sec(cs)
            if age != float("inf"):
                pct = 100 if age < 0.5 else 60 if age < 2 else 30 if age < 5 else 0
                link += pct
                link_n += 1
        self._stat_battery.set_value(
            f"{int(bat / max(1, bat_n))}%" if bat_n else "--%"
        )
        self._stat_link.set_value(
            f"{int(link / max(1, link_n))}%" if link_n else "--%"
        )
        # 任務 / 警報先擺 placeholder（後續由 main window 注入）
        self._stat_missions.set_value("0")
        self._stat_alerts.set_value("0")


__all__ = ["FleetDashboard"]
