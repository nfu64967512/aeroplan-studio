"""ui/widgets/fleet_card.py — ADOS 風格 Fleet Card。

單機卡片：callsign / mode / 飛時 / 電池 / GPS / 鏈路 / RSSI / armed / 任務。
純 QSS + QLabel / QProgressBar，無 QPainter 自繪（與舊的 TacticalUAVCard 區分）。

舊的 `TacticalUAVCard` 保留給 `tactical_swarm_strike_panel.py` 使用，
本元件供新的 `FleetDashboard`。
"""
from __future__ import annotations

from typing import Optional

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QMouseEvent
from PyQt6.QtWidgets import (
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMenu,
    QProgressBar,
    QSizePolicy,
    QWidget,
)

from mission.sitl_link import TelemetryFrame
from ui.qt_utils import repolish
from ui.resources.aeroplan_theme import tokens as T


class FleetCard(QFrame):
    """單機卡片：260×148 px、QGridLayout。

    Signals:
        clicked(str): 點擊；參數為 callsign。
        context_action(str, str): 右鍵動作；(callsign, action_key)。
    """

    clicked = pyqtSignal(str)
    context_action = pyqtSignal(str, str)

    def __init__(
        self,
        callsign: str,
        sysid: int = 0,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self._callsign = callsign
        self._sysid = sysid
        self._selected = False
        self.setProperty("role", "card")
        self.setProperty("selected", "false")
        self.setMinimumSize(260, 148)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setCursor(Qt.CursorShape.PointingHandCursor)

        grid = QGridLayout(self)
        grid.setContentsMargins(10, 8, 10, 8)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(4)

        # Row 0：狀態點 + callsign + sysid
        head = QHBoxLayout()
        head.setContentsMargins(0, 0, 0, 0)
        head.setSpacing(6)
        self._status_dot = QLabel(self)
        self._status_dot.setProperty("role", "statusdot")
        self._status_dot.setProperty("severity", "info")
        self._status_dot.setFixedSize(10, 10)

        self._cs_label = QLabel(callsign, self)
        self._cs_label.setStyleSheet(
            f"font-family: {T.FONT_DISPLAY_STACK}; font-weight: 700; "
            f"font-size: 14px; color: {T.FG}; letter-spacing: 0.5px; "
            f"background: transparent; border: none;"
        )
        self._sysid_label = QLabel(f"sys {sysid}" if sysid else "", self)
        self._sysid_label.setStyleSheet(
            f"font-family: {T.FONT_MONO_STACK}; font-size: 11px; "
            f"color: {T.FG_MUTED}; background: transparent; border: none;"
        )
        head.addWidget(self._status_dot)
        head.addWidget(self._cs_label, 1)
        head.addWidget(self._sysid_label)
        grid.addLayout(head, 0, 0, 1, 2)

        # Row 1：MODE + FLT
        self._mode_label = QLabel("MODE: ---", self)
        self._flt_label = QLabel("FLT --:--:--", self)
        for lab in (self._mode_label, self._flt_label):
            lab.setStyleSheet(
                f"font-family: {T.FONT_MONO_STACK}; font-weight: 700; "
                f"font-size: 11px; color: {T.HUD_GREEN}; "
                f"background: transparent; border: none;"
            )
        grid.addWidget(self._mode_label, 1, 0)
        grid.addWidget(self._flt_label, 1, 1, Qt.AlignmentFlag.AlignRight)

        # Row 2：BAT bar + GPS
        bat_row = QHBoxLayout()
        bat_row.setContentsMargins(0, 0, 0, 0)
        bat_row.setSpacing(4)
        bat_lab = QLabel("BAT", self)
        bat_lab.setStyleSheet(
            f"font-family: {T.FONT_DISPLAY_STACK}; font-size: 10px; "
            f"color: {T.FG_SECONDARY}; background: transparent; border: none;"
        )
        self._bat_bar = QProgressBar(self)
        self._bat_bar.setRange(0, 100)
        self._bat_bar.setValue(0)
        self._bat_bar.setTextVisible(False)
        self._bat_bar.setFixedHeight(8)
        self._bat_bar.setProperty("status", "nominal")
        self._bat_pct = QLabel("--%", self)
        self._bat_pct.setStyleSheet(
            f"font-family: {T.FONT_MONO_STACK}; font-weight: 700; "
            f"font-size: 11px; color: {T.FG}; background: transparent; border: none;"
        )
        self._bat_pct.setFixedWidth(36)
        self._bat_pct.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        bat_row.addWidget(bat_lab)
        bat_row.addWidget(self._bat_bar, 1)
        bat_row.addWidget(self._bat_pct)
        grid.addLayout(bat_row, 2, 0)

        self._gps_label = QLabel("GPS --- · 0 SV", self)
        self._gps_label.setStyleSheet(
            f"font-family: {T.FONT_MONO_STACK}; font-weight: 700; "
            f"font-size: 11px; color: {T.FG_SECONDARY}; "
            f"background: transparent; border: none;"
        )
        grid.addWidget(self._gps_label, 2, 1, Qt.AlignmentFlag.AlignRight)

        # Row 3：LNK bar + RSSI
        lnk_row = QHBoxLayout()
        lnk_row.setContentsMargins(0, 0, 0, 0)
        lnk_row.setSpacing(4)
        lnk_lab = QLabel("LNK", self)
        lnk_lab.setStyleSheet(
            f"font-family: {T.FONT_DISPLAY_STACK}; font-size: 10px; "
            f"color: {T.FG_SECONDARY}; background: transparent; border: none;"
        )
        self._lnk_bar = QProgressBar(self)
        self._lnk_bar.setRange(0, 100)
        self._lnk_bar.setValue(0)
        self._lnk_bar.setTextVisible(False)
        self._lnk_bar.setFixedHeight(8)
        self._lnk_bar.setProperty("status", "nominal")
        self._lnk_pct = QLabel("--%", self)
        self._lnk_pct.setStyleSheet(
            f"font-family: {T.FONT_MONO_STACK}; font-weight: 700; "
            f"font-size: 11px; color: {T.FG}; background: transparent; border: none;"
        )
        self._lnk_pct.setFixedWidth(36)
        self._lnk_pct.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        lnk_row.addWidget(lnk_lab)
        lnk_row.addWidget(self._lnk_bar, 1)
        lnk_row.addWidget(self._lnk_pct)
        grid.addLayout(lnk_row, 3, 0)

        self._rssi_label = QLabel("RSSI --- dBm", self)
        self._rssi_label.setStyleSheet(
            f"font-family: {T.FONT_MONO_STACK}; font-weight: 700; "
            f"font-size: 11px; color: {T.FG_SECONDARY}; "
            f"background: transparent; border: none;"
        )
        grid.addWidget(self._rssi_label, 3, 1, Qt.AlignmentFlag.AlignRight)

        # Row 4：ARMED + MSN
        self._armed_label = QLabel("● DISARMED", self)
        self._armed_label.setStyleSheet(
            f"font-family: {T.FONT_MONO_STACK}; font-weight: 700; "
            f"font-size: 11px; color: {T.FRIENDLY}; "
            f"background: transparent; border: none;"
        )
        self._msn_label = QLabel("MSN ---", self)
        self._msn_label.setStyleSheet(
            f"font-family: {T.FONT_MONO_STACK}; font-weight: 700; "
            f"font-size: 11px; color: {T.FG_SECONDARY}; "
            f"background: transparent; border: none;"
        )
        grid.addWidget(self._armed_label, 4, 0)
        grid.addWidget(self._msn_label, 4, 1, Qt.AlignmentFlag.AlignRight)

    # ── 公開 API ──────────────────────────────────────────────
    def callsign(self) -> str:
        return self._callsign

    def set_selected(self, selected: bool) -> None:
        self._selected = bool(selected)
        self.setProperty("selected", "true" if selected else "false")
        repolish(self)

    def update_from_frame(self, frame: TelemetryFrame, mission_label: str = "") -> None:
        """以 TelemetryFrame 更新所有讀數。"""
        self._mode_label.setText(f"MODE: {frame.mode or '---'}")
        # 飛時來自 telemetry — 暫以 0 顯示，後續接 main window 接管。
        # GPS
        gps_names = {0: "NO", 1: "NF", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK"}
        self._gps_label.setText(
            f"GPS {gps_names.get(frame.gps_fix, '?')} · {frame.gps_sats} SV"
        )
        # Battery
        pct = max(0, min(100, int(frame.battery_pct))) if frame.battery_pct >= 0 else 0
        self._bat_bar.setValue(pct)
        if pct >= T.THRESHOLDS.battery_low:
            self._bat_bar.setProperty("status", "nominal")
            self._bat_pct.setStyleSheet(
                f"font-family: {T.FONT_MONO_STACK}; font-weight: 700; "
                f"font-size: 11px; color: {T.FG}; background: transparent; border: none;"
            )
        elif pct >= T.THRESHOLDS.battery_critical:
            self._bat_bar.setProperty("status", "warning")
            self._bat_pct.setStyleSheet(
                f"font-family: {T.FONT_MONO_STACK}; font-weight: 700; "
                f"font-size: 11px; color: {T.WARNING}; background: transparent; border: none;"
            )
        else:
            self._bat_bar.setProperty("status", "critical")
            self._bat_pct.setStyleSheet(
                f"font-family: {T.FONT_MONO_STACK}; font-weight: 700; "
                f"font-size: 11px; color: {T.HOSTILE}; background: transparent; border: none;"
            )
        repolish(self._bat_bar)
        self._bat_pct.setText(f"{pct}%" if frame.battery_pct >= 0 else "--%")

        # Armed
        if frame.armed:
            self._armed_label.setText("● ARMED")
            self._armed_label.setStyleSheet(
                f"font-family: {T.FONT_MONO_STACK}; font-weight: 700; "
                f"font-size: 11px; color: {T.HOSTILE}; "
                f"background: transparent; border: none;"
            )
            self._status_dot.setProperty("severity", "critical")
        else:
            self._armed_label.setText("● DISARMED")
            self._armed_label.setStyleSheet(
                f"font-family: {T.FONT_MONO_STACK}; font-weight: 700; "
                f"font-size: 11px; color: {T.FRIENDLY}; "
                f"background: transparent; border: none;"
            )
            self._status_dot.setProperty("severity", "success")
        repolish(self._status_dot)

        if mission_label:
            self._msn_label.setText(f"MSN {mission_label}")

    def set_link_quality(self, pct: int, rssi_dbm: Optional[int] = None) -> None:
        """更新鏈路條與 RSSI。"""
        pct = max(0, min(100, int(pct)))
        self._lnk_bar.setValue(pct)
        if pct >= 60:
            self._lnk_bar.setProperty("status", "nominal")
        elif pct >= 30:
            self._lnk_bar.setProperty("status", "warning")
        else:
            self._lnk_bar.setProperty("status", "critical")
        repolish(self._lnk_bar)
        self._lnk_pct.setText(f"{pct}%" if pct > 0 else "LOST")
        if rssi_dbm is not None:
            self._rssi_label.setText(f"RSSI {rssi_dbm} dBm")

    def set_lost(self) -> None:
        """鏈路失聯狀態：燈號轉紅、armed 顯示 LOST。"""
        self._status_dot.setProperty("severity", "critical")
        repolish(self._status_dot)
        self._armed_label.setText("● LOST")
        self._armed_label.setStyleSheet(
            f"font-family: {T.FONT_MONO_STACK}; font-weight: 700; "
            f"font-size: 11px; color: {T.HOSTILE}; "
            f"background: transparent; border: none;"
        )

    # ── Qt 事件 ────────────────────────────────────────────────
    def mousePressEvent(self, evt: QMouseEvent) -> None:  # noqa: N802
        if evt.button() == Qt.MouseButton.LeftButton:
            self.clicked.emit(self._callsign)
        super().mousePressEvent(evt)

    def contextMenuEvent(self, evt) -> None:  # noqa: N802
        menu = QMenu(self)
        actions = [
            ("Set Active", "set_active"),
            ("Center Map", "center_map"),
            ("Disarm Now", "disarm"),
            ("Open SITL Console", "open_console"),
        ]
        for label, key in actions:
            act = menu.addAction(label)
            act.triggered.connect(lambda _=False, k=key: self.context_action.emit(self._callsign, k))
        menu.exec(evt.globalPos())


__all__ = ["FleetCard"]
