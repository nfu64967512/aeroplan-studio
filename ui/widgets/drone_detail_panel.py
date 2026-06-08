"""ui/widgets/drone_detail_panel.py — ADOS 風單機詳細面板（Overview tab）。

對應截圖左欄：大型讀數（ALT/SPD/HDG/VS）→ GPS+電池 → LOADOUT
→ Pre-Flight Check → DISARM + Mode → 快速動作鈕 → Follow Me
→ HEALTH chips → 健康度 → GPS sats/Fix → 電池條
→ VEHICLE INFO / IDENTITY / STATISTICS。

GCS 級指令均對接 `mission.fleet_registry.FleetRegistry.get_link(callsign)`
取 `SITLLink` 並呼叫 arm/disarm/set_mode/rtl/land/loiter/pause/reboot_fc/change_altitude。
"""
from __future__ import annotations

import time
from datetime import datetime
from typing import Optional

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QStackedWidget,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from mission.fleet_registry import FleetRegistry
from mission.sitl_link import TelemetryFrame
from ui.widgets.flight_logs_panel import FlightLogsPanel
from ui.widgets.parameters_browser import ParametersBrowser
from ui.resources.aeroplan_theme import tokens as T
from ui.resources.aeroplan_theme.buttons import (
    ButtonSize,
    ButtonVariant,
    make_button,
)


# 飛行模式選單
_MODE_LIST = [
    "STABILIZE", "ACRO", "ALT_HOLD", "AUTO", "GUIDED",
    "LOITER", "RTL", "LAND", "POSHOLD", "BRAKE",
    "SMART_RTL", "FOLLOW", "AUTO_RTL",
]


class _BigReadout(QFrame):
    """大型數值塊：value + 緊鄰小單位、下方灰標籤。

    版面：
        ┌─────────────────┐
        │   62.6 m        │  ← value（HUD 綠 22px）+ unit（灰 9px，緊鄰右側）
        │   ALT           │  ← label（灰 11px）
        └─────────────────┘
    """

    def __init__(
        self,
        label: str,
        unit: str = "",
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.setProperty("role", "card")
        self._value_label = QLabel("--", self)
        self._value_label.setProperty("role", "stat-value")
        self._value_label.setAlignment(
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter
        )
        self._unit_label = QLabel(unit, self)
        self._unit_label.setStyleSheet(
            f"font-family: {T.FONT_DISPLAY_STACK}; font-size: 10px; "
            f"color: {T.FG_MUTED}; background: transparent; border: none;"
        )
        self._unit_label.setAlignment(
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignBottom
        )
        self._label_label = QLabel(label, self)
        self._label_label.setProperty("role", "stat-label")

        v = QVBoxLayout(self)
        v.setContentsMargins(8, 6, 8, 6)
        v.setSpacing(0)
        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(3)
        # value 在左、unit 緊鄰右側、stretch 在最後吃剩餘空間
        row.addWidget(self._value_label, 0)
        row.addWidget(self._unit_label, 0, Qt.AlignmentFlag.AlignBottom)
        row.addStretch(1)
        v.addLayout(row)
        v.addWidget(self._label_label)

    def set_value(self, text: str) -> None:
        self._value_label.setText(text)


class _Section(QFrame):
    """區塊標題 + 內容容器。"""

    def __init__(
        self,
        title: str,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.setProperty("role", "card")
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)

        v = QVBoxLayout(self)
        v.setContentsMargins(10, 8, 10, 8)
        v.setSpacing(6)

        head = QLabel(title.upper(), self)
        head.setProperty("role", "stat-label")
        v.addWidget(head)

        self._body = QWidget(self)
        self._body_layout = QVBoxLayout(self._body)
        self._body_layout.setContentsMargins(0, 0, 0, 0)
        self._body_layout.setSpacing(4)
        v.addWidget(self._body)

    def body_layout(self) -> QVBoxLayout:
        return self._body_layout


def _make_kv(key: str, value: str, parent: QWidget) -> QWidget:
    """單列 key/value 卡片（VEHICLE INFO 等用）。"""
    w = QFrame(parent)
    w.setProperty("role", "card")
    v = QVBoxLayout(w)
    v.setContentsMargins(8, 4, 8, 4)
    v.setSpacing(0)
    val_lab = QLabel(value, w)
    val_lab.setStyleSheet(
        f"font-family: {T.FONT_MONO_STACK}; font-weight: 700; "
        f"font-size: 13px; color: {T.FG}; background: transparent; border: none;"
    )
    key_lab = QLabel(key, w)
    key_lab.setStyleSheet(
        f"font-family: {T.FONT_DISPLAY_STACK}; font-size: 9px; "
        f"color: {T.FG_MUTED}; background: transparent; border: none; "
        f"letter-spacing: 0.5px;"
    )
    v.addWidget(val_lab)
    v.addWidget(key_lab)
    return w


class DroneDetailPanel(QWidget):
    """ADOS 風單機詳細面板。"""

    # 對外通知（呼叫端可選擇做提示音、log 等）
    command_sent = pyqtSignal(str, str)   # callsign, action

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._callsign: Optional[str] = None

        # ── 整體 scroll wrapper ─────────────────────────
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        # ── 頂部 breadcrumb ─────────────────────────
        crumb = QWidget(self)
        crumb.setStyleSheet(
            f"background: {T.BG_SECONDARY}; "
            f"border-bottom: 1px solid {T.BORDER_SUBTLE};"
        )
        crumb_lay = QHBoxLayout(crumb)
        crumb_lay.setContentsMargins(12, 6, 12, 6)
        crumb_lay.setSpacing(8)

        self._name_lab = QLabel("---", crumb)
        self._name_lab.setStyleSheet(
            f"font-family: {T.FONT_DISPLAY_STACK}; font-weight: 700; "
            f"font-size: 14px; color: {T.FG}; background: transparent; border: none;"
        )
        crumb_lay.addWidget(self._name_lab)

        self._state_pill = QLabel("IDLE", crumb)
        self._state_pill.setStyleSheet(
            f"background: {T.BG_ELEVATED}; color: {T.HUD_GREEN}; "
            f"border: 1px solid {T.BORDER}; border-radius: 8px; "
            f"padding: 1px 8px; font-family: {T.FONT_DISPLAY_STACK}; "
            f"font-weight: 700; font-size: 10px; letter-spacing: 0.8px;"
        )
        crumb_lay.addWidget(self._state_pill)

        crumb_lay.addStretch(1)

        # 小型狀態 icons（純文字 chip 表示，無 SVG 依賴）
        self._mavstate_chip = QLabel("MAV", crumb)
        self._mavstate_chip.setStyleSheet(
            f"font-family: {T.FONT_MONO_STACK}; font-size: 10px; "
            f"color: {T.FG_SECONDARY}; background: transparent;"
        )
        crumb_lay.addWidget(self._mavstate_chip)

        self._btn_reboot = make_button(
            "Reboot FC",
            variant=ButtonVariant.DESTRUCTIVE,
            size=ButtonSize.SM,
        )
        self._btn_reboot.clicked.connect(self._on_reboot)
        crumb_lay.addWidget(self._btn_reboot)

        outer.addWidget(crumb)

        # ── tab strip：Overview / Flights / Calibrate / Parameters / Configure ──
        # 5 個 QToolButton 互斥分頁；點擊切換下方 QStackedWidget。
        tabs = QWidget(self)
        tabs.setStyleSheet(
            f"background: {T.BG_PRIMARY}; "
            f"border-bottom: 1px solid {T.BORDER_SUBTLE};"
        )
        tabs_lay = QHBoxLayout(tabs)
        tabs_lay.setContentsMargins(8, 0, 8, 0)
        tabs_lay.setSpacing(0)
        self._tab_buttons: list[QToolButton] = []
        for i, name in enumerate(
            ("Overview", "Flights", "Calibrate", "Parameters", "Configure")
        ):
            tb = QToolButton(tabs)
            tb.setText(name)
            tb.setCheckable(True)
            tb.setProperty("role", "navtab")
            tb.setChecked(i == 0)
            tb.setAutoExclusive(True)
            tb.style().unpolish(tb)
            tb.style().polish(tb)
            # 用 lambda 抓 index，切換 stack 對應頁
            tb.clicked.connect(lambda _checked, idx=i: self._on_tab_changed(idx))
            self._tab_buttons.append(tb)
            tabs_lay.addWidget(tb)
        tabs_lay.addStretch(1)
        outer.addWidget(tabs)

        # ── 主內容：QStackedWidget 5 頁 ─────────────────────────
        self._stack = QStackedWidget(self)
        outer.addWidget(self._stack, 1)

        # Page 0：Overview（包在 scroll 內）
        scroll = QScrollArea(self._stack)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        body = QWidget(scroll)
        body_lay = QVBoxLayout(body)
        body_lay.setContentsMargins(10, 10, 10, 10)
        body_lay.setSpacing(8)

        # ── 大型 4 讀數 ─────────────────────────
        readout_row = QHBoxLayout()
        readout_row.setContentsMargins(0, 0, 0, 0)
        readout_row.setSpacing(6)
        self._alt = _BigReadout("ALT", "m", body)
        self._spd = _BigReadout("SPD", "m/s", body)
        self._hdg = _BigReadout("HDG", "°", body)
        self._vs = _BigReadout("VS", "m/s", body)
        for r in (self._alt, self._spd, self._hdg, self._vs):
            readout_row.addWidget(r, 1)
        body_lay.addLayout(readout_row)

        # ── 主動作列：DISARM + Mode ───────────────────────
        action_row = QHBoxLayout()
        action_row.setContentsMargins(0, 0, 0, 0)
        action_row.setSpacing(6)
        self._btn_arm = make_button(
            "DISARM", variant=ButtonVariant.DESTRUCTIVE, size=ButtonSize.MD,
        )
        self._btn_arm.clicked.connect(self._on_toggle_arm)
        action_row.addWidget(self._btn_arm, 1)

        self._cmb_mode = QComboBox(body)
        self._cmb_mode.addItems(_MODE_LIST)
        self._cmb_mode.setCurrentText("GUIDED")
        self._cmb_mode.activated.connect(self._on_mode_change)
        action_row.addWidget(self._cmb_mode, 1)
        body_lay.addLayout(action_row)

        # ── 快速動作鈕 6 顆（Pause / Home / Alt / Upload / Download / Cancel）──
        quick_row = QHBoxLayout()
        quick_row.setContentsMargins(0, 0, 0, 0)
        quick_row.setSpacing(4)
        self._btn_pause = make_button("‖", variant=ButtonVariant.OUTLINE, size=ButtonSize.SM, tooltip="Pause/Continue")
        self._btn_home = make_button("⌂", variant=ButtonVariant.OUTLINE, size=ButtonSize.SM, tooltip="Return to Launch")
        self._btn_alt = make_button("↑10", variant=ButtonVariant.OUTLINE, size=ButtonSize.SM, tooltip="Change altitude")
        self._btn_upload = make_button("↥", variant=ButtonVariant.OUTLINE, size=ButtonSize.SM, tooltip="Upload mission")
        self._btn_download = make_button("↧", variant=ButtonVariant.OUTLINE, size=ButtonSize.SM, tooltip="Download mission")
        self._btn_cancel = make_button("✕", variant=ButtonVariant.OUTLINE, size=ButtonSize.SM, tooltip="Cancel current command")
        for b in (self._btn_pause, self._btn_home, self._btn_alt,
                  self._btn_upload, self._btn_download, self._btn_cancel):
            quick_row.addWidget(b)
        body_lay.addLayout(quick_row)
        self._btn_pause.clicked.connect(self._on_pause)
        self._btn_home.clicked.connect(self._on_rtl)
        self._btn_alt.clicked.connect(self._on_change_alt)
        self._btn_cancel.clicked.connect(self._on_loiter)

        # Follow Me
        self._btn_follow = make_button(
            "Follow Me", variant=ButtonVariant.PRIMARY, size=ButtonSize.SM,
        )
        self._btn_follow.clicked.connect(self._on_follow_me)
        body_lay.addWidget(self._btn_follow)

        # ── HEALTH chips ────────────────────────────────
        health_section = _Section("HEALTH", body)
        chips_row = QHBoxLayout()
        chips_row.setContentsMargins(0, 0, 0, 0)
        chips_row.setSpacing(4)
        self._health_chips: dict[str, QLabel] = {}
        for name in ("Gyro", "Accel", "Compass", "Baro", "GPS", "Motors", "RC", "AHRS"):
            chip = QLabel(name, health_section)
            chip.setStyleSheet(self._chip_style(ok=True))
            self._health_chips[name] = chip
            chips_row.addWidget(chip)
        chips_row.addStretch(1)
        health_section.body_layout().addLayout(chips_row)

        # 健康度數值（92% Health / 19.6V Voltage / 19 GPS Sats / 3D Fix）
        kv_grid = QGridLayout()
        kv_grid.setContentsMargins(0, 0, 0, 0)
        kv_grid.setHorizontalSpacing(6)
        kv_grid.setVerticalSpacing(6)
        self._kv_health = _make_kv("HEALTH", "--%", health_section)
        self._kv_volt = _make_kv("VOLTAGE", "--.- V", health_section)
        self._kv_sats = _make_kv("GPS SATS", "--", health_section)
        self._kv_fix = _make_kv("FIX TYPE", "---", health_section)
        kv_grid.addWidget(self._kv_health, 0, 0)
        kv_grid.addWidget(self._kv_volt, 0, 1)
        kv_grid.addWidget(self._kv_sats, 1, 0)
        kv_grid.addWidget(self._kv_fix, 1, 1)
        health_section.body_layout().addLayout(kv_grid)

        # 電池條
        self._bat_bar = QProgressBar(health_section)
        self._bat_bar.setRange(0, 100)
        self._bat_bar.setValue(0)
        self._bat_bar.setTextVisible(True)
        self._bat_bar.setFormat("Battery %p%")
        self._bat_bar.setProperty("status", "nominal")
        health_section.body_layout().addWidget(self._bat_bar)
        body_lay.addWidget(health_section)

        # ── VEHICLE INFO ────────────────────────────────
        veh = _Section("VEHICLE INFO", body)
        veh_grid = QGridLayout()
        veh_grid.setContentsMargins(0, 0, 0, 0)
        veh_grid.setHorizontalSpacing(6)
        veh_grid.setVerticalSpacing(6)
        self._kv_frame = _make_kv("FRAME", "copter", veh)
        self._kv_firmware = _make_kv("FIRMWARE", "ArduCopter", veh)
        self._kv_compute = _make_kv("COMPUTE", "RPi CM4", veh)
        self._kv_weight = _make_kv("WEIGHT", "Micro", veh)
        veh_grid.addWidget(self._kv_frame, 0, 0)
        veh_grid.addWidget(self._kv_firmware, 0, 1)
        veh_grid.addWidget(self._kv_compute, 1, 0)
        veh_grid.addWidget(self._kv_weight, 1, 1)
        veh.body_layout().addLayout(veh_grid)
        body_lay.addWidget(veh)

        # ── IDENTITY ────────────────────────────────────
        ident = _Section("IDENTITY", body)
        ident_grid = QGridLayout()
        ident_grid.setContentsMargins(0, 0, 0, 0)
        ident_grid.setHorizontalSpacing(6)
        ident_grid.setVerticalSpacing(6)
        self._kv_name = _make_kv("NAME", "---", ident)
        self._kv_id = _make_kv("ID", "---", ident)
        self._kv_serial = _make_kv("SERIAL", "---", ident)
        self._kv_reg = _make_kv("REGISTRATION", "—", ident)
        ident_grid.addWidget(self._kv_name, 0, 0)
        ident_grid.addWidget(self._kv_id, 0, 1)
        ident_grid.addWidget(self._kv_serial, 1, 0)
        ident_grid.addWidget(self._kv_reg, 1, 1)
        ident.body_layout().addLayout(ident_grid)
        body_lay.addWidget(ident)

        body_lay.addStretch(1)
        scroll.setWidget(body)
        self._stack.addWidget(scroll)  # idx 0 — Overview

        # Page 1：Flights — 直接嵌入 FlightLogsPanel（已自訂閱所有 SITLLink）
        self._flights_panel = FlightLogsPanel(self._stack)
        self._stack.addWidget(self._flights_panel)  # idx 1

        # Page 2：Calibrate — 校正動作 + 狀態資訊
        self._calibrate_page = self._build_calibrate_page()
        self._stack.addWidget(self._calibrate_page)  # idx 2

        # Page 3：Parameters — ParametersBrowser（set_callsign 時同步 sysid）
        self._params_browser = ParametersBrowser(self._stack)
        self._stack.addWidget(self._params_browser)  # idx 3

        # Page 4：Configure — 載具識別 + 常用參數快速編輯
        self._configure_page = self._build_configure_page()
        self._stack.addWidget(self._configure_page)  # idx 4

        # 訂閱 registry
        FleetRegistry.instance().telemetry_updated.connect(self._on_telemetry)

    # ── 公開 ─────────────────────────────────────────
    def set_callsign(self, callsign: str) -> None:
        self._callsign = callsign
        self._name_lab.setText(callsign)
        reg = FleetRegistry.instance()
        link = reg.get_link(callsign)
        sysid: Optional[int] = None
        if link is not None:
            sysid = int(getattr(link, "_frame").sysid)
            # _make_kv() 內部 children 順序：[val_lab, key_lab]
            self._kv_id.findChildren(QLabel)[0].setText(f"sysid {sysid}")
            self._kv_name.findChildren(QLabel)[0].setText(callsign)
        # Parameters tab：同步該 UAV 的參數
        if sysid is not None and hasattr(self, "_params_browser"):
            self._params_browser.set_current_sysid(sysid)
        # Configure tab：更新識別與框架資訊
        if hasattr(self, "_cfg_kv_callsign"):
            self._cfg_kv_callsign.findChildren(QLabel)[0].setText(callsign)
            self._cfg_kv_sysid.findChildren(QLabel)[0].setText(
                f"sysid {sysid}" if sysid is not None else "—"
            )
        # 立即套用最新一筆 telemetry（若有）
        latest = reg.latest(callsign)
        if latest is not None:
            self._on_telemetry(callsign, latest)

    # ── Tab 切換 ─────────────────────────────────────
    def _on_tab_changed(self, idx: int) -> None:
        """切換主內容 stack 並維持互斥按鈕勾選狀態（autoExclusive 已處理）。"""
        self._stack.setCurrentIndex(idx)

    # ── Page 2：Calibrate ─────────────────────────────
    def _build_calibrate_page(self) -> QWidget:
        """Calibrate 分頁：4 個校正動作鈕 + 校正狀態說明。

        校正用 MAVLink COMMAND_LONG (MAV_CMD_PREFLIGHT_CALIBRATION, #241)：
            param1=陀螺、param2=羅盤磁偏、param3=氣壓、param4=遙控、
            param5=加速度（1=全、4=水平、2=Trim）。
        透過 SITLLink._cmd_queue 推一個 'cmd_long' 任務（若 SITLLink 不支援
        則 fallback 為 status 訊息），SITL 環境下校正多半立即完成。
        """
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        page = QWidget(scroll)
        lay = QVBoxLayout(page)
        lay.setContentsMargins(10, 10, 10, 10)
        lay.setSpacing(8)

        notice = QLabel(
            "感測器校正 — SITL 環境下校正多為立即完成、實機需依提示移動載具。",
            page,
        )
        notice.setWordWrap(True)
        notice.setProperty("role", "caption")
        lay.addWidget(notice)

        # 4 顆校正鈕
        cal_section = _Section("CALIBRATION ACTIONS", page)
        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(6)

        btn_gyro = make_button(
            "Gyro", variant=ButtonVariant.OUTLINE, size=ButtonSize.MD,
            tooltip="校正陀螺儀偏移（保持載具靜止）",
        )
        btn_accel = make_button(
            "Accel (Level)", variant=ButtonVariant.OUTLINE, size=ButtonSize.MD,
            tooltip="水平校正加速度計（載具置於水平面）",
        )
        btn_compass = make_button(
            "Compass", variant=ButtonVariant.OUTLINE, size=ButtonSize.MD,
            tooltip="羅盤磁偏校正",
        )
        btn_baro = make_button(
            "Baro", variant=ButtonVariant.OUTLINE, size=ButtonSize.MD,
            tooltip="氣壓計地面參考校正",
        )
        btn_gyro.clicked.connect(lambda: self._calibrate("gyro"))
        btn_accel.clicked.connect(lambda: self._calibrate("accel_level"))
        btn_compass.clicked.connect(lambda: self._calibrate("compass"))
        btn_baro.clicked.connect(lambda: self._calibrate("baro"))
        grid.addWidget(btn_gyro, 0, 0)
        grid.addWidget(btn_accel, 0, 1)
        grid.addWidget(btn_compass, 1, 0)
        grid.addWidget(btn_baro, 1, 1)
        cal_section.body_layout().addLayout(grid)
        lay.addWidget(cal_section)

        # 校正即時狀態（mirror Overview health chips）
        status_section = _Section("SENSOR STATUS", page)
        chips_row = QHBoxLayout()
        chips_row.setContentsMargins(0, 0, 0, 0)
        chips_row.setSpacing(4)
        self._cal_chips: dict[str, QLabel] = {}
        for name in ("Gyro", "Accel", "Compass", "Baro", "GPS"):
            chip = QLabel(name, status_section)
            chip.setStyleSheet(self._chip_style(ok=True))
            self._cal_chips[name] = chip
            chips_row.addWidget(chip)
        chips_row.addStretch(1)
        status_section.body_layout().addLayout(chips_row)
        lay.addWidget(status_section)

        lay.addStretch(1)
        scroll.setWidget(page)
        return scroll

    def _calibrate(self, kind: str) -> None:
        """送 PREFLIGHT_CALIBRATION (CMD#241) 至 SITLLink。
        若該 link 不支援 cmd_long 介面，發 command_sent 給 status bar。
        """
        link = self._link()
        if link is None:
            return
        # MAV_CMD_PREFLIGHT_CALIBRATION 各參數對應
        params = {
            "gyro":         (1, 0, 0, 0, 0, 0, 0),
            "compass":      (0, 1, 0, 0, 0, 0, 0),
            "baro":         (0, 0, 1, 0, 0, 0, 0),
            "accel_level":  (0, 0, 0, 0, 4, 0, 0),
        }.get(kind)
        if params is None:
            return
        # 嘗試多種 SITLLink API（前向相容）
        if hasattr(link, "send_command_long"):
            link.send_command_long(241, *params)
        elif hasattr(link, "_cmd_queue"):
            link._cmd_queue.put(("cmd_long", (241, params)))
        self.command_sent.emit(self._callsign or "", f"CAL→{kind.upper()}")

    # ── Page 4：Configure ─────────────────────────────
    def _build_configure_page(self) -> QWidget:
        """Configure 分頁：載具識別 + 常用 ArduPilot 參數快速調整。"""
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        page = QWidget(scroll)
        lay = QVBoxLayout(page)
        lay.setContentsMargins(10, 10, 10, 10)
        lay.setSpacing(8)

        # 識別資訊（從 Overview 抽出概觀）
        ident = _Section("IDENTITY", page)
        ig = QGridLayout()
        ig.setContentsMargins(0, 0, 0, 0)
        ig.setHorizontalSpacing(6)
        ig.setVerticalSpacing(6)
        self._cfg_kv_callsign = _make_kv("CALLSIGN", "—", ident)
        self._cfg_kv_sysid = _make_kv("SYSID", "—", ident)
        ig.addWidget(self._cfg_kv_callsign, 0, 0)
        ig.addWidget(self._cfg_kv_sysid, 0, 1)
        ident.body_layout().addLayout(ig)
        lay.addWidget(ident)

        # 常用參數快速編輯
        qp = _Section("QUICK PARAMETERS", page)
        qp_form = QGridLayout()
        qp_form.setContentsMargins(0, 0, 0, 0)
        qp_form.setHorizontalSpacing(6)
        qp_form.setVerticalSpacing(6)

        # RTL_ALT — 返航高度（cm）
        self._cfg_rtl_alt = QDoubleSpinBox(qp)
        self._cfg_rtl_alt.setRange(200.0, 50000.0)   # 2–500 m
        self._cfg_rtl_alt.setValue(3000.0)
        self._cfg_rtl_alt.setSuffix(" cm")
        self._cfg_rtl_alt.setDecimals(0)
        self._cfg_rtl_alt.setToolTip("RTL_ALT — 返航爬升高度（單位：公分）")

        # WPNAV_SPEED — 自動模式水平巡航速度（cm/s）
        self._cfg_wp_speed = QDoubleSpinBox(qp)
        self._cfg_wp_speed.setRange(20.0, 2000.0)
        self._cfg_wp_speed.setValue(500.0)
        self._cfg_wp_speed.setSuffix(" cm/s")
        self._cfg_wp_speed.setDecimals(0)
        self._cfg_wp_speed.setToolTip("WPNAV_SPEED — 自動模式水平速度（公分/秒）")

        # FENCE_ENABLE — 圍籬啟用 (0/1)
        self._cfg_fence_enable = QComboBox(qp)
        self._cfg_fence_enable.addItem("Disabled (0)", 0)
        self._cfg_fence_enable.addItem("Enabled (1)", 1)
        self._cfg_fence_enable.setToolTip("FENCE_ENABLE — 電子圍籬啟用")

        qp_form.addWidget(QLabel("RTL Altitude:", qp), 0, 0)
        qp_form.addWidget(self._cfg_rtl_alt, 0, 1)
        qp_form.addWidget(QLabel("WPNAV Speed:", qp), 1, 0)
        qp_form.addWidget(self._cfg_wp_speed, 1, 1)
        qp_form.addWidget(QLabel("Fence Enable:", qp), 2, 0)
        qp_form.addWidget(self._cfg_fence_enable, 2, 1)
        qp.body_layout().addLayout(qp_form)

        # 套用 / 完整參數鈕
        btn_row = QHBoxLayout()
        btn_apply = make_button(
            "Apply Quick Params", variant=ButtonVariant.PRIMARY, size=ButtonSize.MD,
        )
        btn_apply.clicked.connect(self._on_apply_quick_params)
        btn_full = make_button(
            "Full Parameter Editor →", variant=ButtonVariant.OUTLINE, size=ButtonSize.MD,
            tooltip="切換到 Parameters 分頁編輯所有 ArduPilot 參數",
        )
        btn_full.clicked.connect(lambda: self._tab_buttons[3].click())
        btn_row.addWidget(btn_apply, 1)
        btn_row.addWidget(btn_full, 1)
        qp.body_layout().addLayout(btn_row)
        lay.addWidget(qp)

        lay.addStretch(1)
        scroll.setWidget(page)
        return scroll

    def _on_apply_quick_params(self) -> None:
        """Quick Parameters：批次寫入 SITLLink。"""
        link = self._link()
        if link is None:
            return
        items = [
            ("RTL_ALT",     float(self._cfg_rtl_alt.value()),     "REAL32"),
            ("WPNAV_SPEED", float(self._cfg_wp_speed.value()),    "REAL32"),
            ("FENCE_ENABLE", float(self._cfg_fence_enable.currentData()), "INT8"),
        ]
        if hasattr(link, "set_params"):
            link.set_params(items)
        else:
            for n, v, t in items:
                link.set_param(n, v, t)
        self.command_sent.emit(self._callsign or "", "APPLY_QUICK_PARAMS")

    def callsign(self) -> Optional[str]:
        return self._callsign

    # ── Telemetry 渲染 ──────────────────────────────
    def _on_telemetry(self, callsign: str, frame: TelemetryFrame) -> None:
        if callsign != self._callsign:
            return
        self._alt.set_value(f"{frame.alt_rel:.1f}")
        self._spd.set_value(f"{frame.ground_speed:.1f}")
        self._hdg.set_value(f"{int(frame.heading) % 360}")
        self._vs.set_value(f"{frame.climb:+.1f}")

        # state pill
        if frame.armed:
            self._state_pill.setText("IN MISSION" if frame.mode in ("AUTO", "GUIDED", "RTL") else "ARMED")
            self._state_pill.setStyleSheet(
                f"background: {T.BG_ELEVATED}; color: {T.HOSTILE}; "
                f"border: 1px solid {T.HOSTILE}; border-radius: 8px; "
                f"padding: 1px 8px; font-family: {T.FONT_DISPLAY_STACK}; "
                f"font-weight: 700; font-size: 10px; letter-spacing: 0.8px;"
            )
            self._btn_arm.setText("DISARM")
        else:
            self._state_pill.setText("IDLE")
            self._state_pill.setStyleSheet(
                f"background: {T.BG_ELEVATED}; color: {T.HUD_GREEN}; "
                f"border: 1px solid {T.BORDER}; border-radius: 8px; "
                f"padding: 1px 8px; font-family: {T.FONT_DISPLAY_STACK}; "
                f"font-weight: 700; font-size: 10px; letter-spacing: 0.8px;"
            )
            self._btn_arm.setText("ARM")
            self._btn_arm.setProperty("btnVariant", "success")
            self._btn_arm.style().unpolish(self._btn_arm)
            self._btn_arm.style().polish(self._btn_arm)
            return  # avoid double-style below
        self._btn_arm.setProperty("btnVariant", "destructive")
        self._btn_arm.style().unpolish(self._btn_arm)
        self._btn_arm.style().polish(self._btn_arm)

        # mode combo
        if frame.mode and frame.mode != "---":
            idx = self._cmb_mode.findText(frame.mode)
            if idx >= 0:
                self._cmb_mode.blockSignals(True)
                self._cmb_mode.setCurrentIndex(idx)
                self._cmb_mode.blockSignals(False)

        # health KVs
        self._kv_health.findChildren(QLabel)[0].setText(
            f"{frame.battery_pct}%" if frame.battery_pct >= 0 else "--%"
        )
        self._kv_volt.findChildren(QLabel)[0].setText(f"{frame.battery_v:.1f} V")
        self._kv_sats.findChildren(QLabel)[0].setText(str(frame.gps_sats))
        fix_names = {0: "NO_GPS", 1: "NO_FIX", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK_F", 6: "RTK_FX"}
        self._kv_fix.findChildren(QLabel)[0].setText(fix_names.get(frame.gps_fix, "—"))

        # 電池條
        pct = max(0, min(100, frame.battery_pct))
        self._bat_bar.setValue(pct if frame.battery_pct >= 0 else 0)
        if pct < T.THRESHOLDS.battery_critical:
            self._bat_bar.setProperty("status", "critical")
        elif pct < T.THRESHOLDS.battery_low:
            self._bat_bar.setProperty("status", "warning")
        else:
            self._bat_bar.setProperty("status", "nominal")
        self._bat_bar.style().unpolish(self._bat_bar)
        self._bat_bar.style().polish(self._bat_bar)

        # health chips（簡化版：GPS 與 Compass 由 frame 推；其餘預設 OK）
        gps_ok = frame.gps_fix >= 3
        self._health_chips["GPS"].setStyleSheet(self._chip_style(ok=gps_ok))
        # Calibrate tab 的感測器 chip 同步（若已建立）
        if hasattr(self, "_cal_chips") and "GPS" in self._cal_chips:
            self._cal_chips["GPS"].setStyleSheet(self._chip_style(ok=gps_ok))

    # ── 指令分派 ────────────────────────────────────
    def _link(self):
        if self._callsign is None:
            return None
        return FleetRegistry.instance().get_link(self._callsign)

    def _on_toggle_arm(self) -> None:
        link = self._link()
        if link is None:
            return
        if self._btn_arm.text() == "ARM":
            link.arm()
            self.command_sent.emit(self._callsign or "", "ARM")
        else:
            link.disarm()
            self.command_sent.emit(self._callsign or "", "DISARM")

    def _on_mode_change(self, _idx: int) -> None:
        link = self._link()
        if link is None:
            return
        mode = self._cmb_mode.currentText()
        link.set_mode(mode)
        self.command_sent.emit(self._callsign or "", f"MODE→{mode}")

    def _on_pause(self) -> None:
        link = self._link()
        if link is None:
            return
        # 若 SITLLink 有 pause() 方法則用之；否則切 LOITER 暫停運動。
        if hasattr(link, "pause"):
            link.pause(True)
            self.command_sent.emit(self._callsign or "", "PAUSE")
        else:
            link.set_mode("LOITER")
            self.command_sent.emit(self._callsign or "", "PAUSE→LOITER")

    def _on_rtl(self) -> None:
        link = self._link()
        if link is None:
            return
        # 透過 set_mode 切 RTL（與既有 SITLLink API 對齊）
        link.set_mode("RTL")
        self.command_sent.emit(self._callsign or "", "RTL")

    def _on_loiter(self) -> None:
        link = self._link()
        if link is None:
            return
        link.set_mode("LOITER")
        self.command_sent.emit(self._callsign or "", "LOITER")

    def _on_change_alt(self) -> None:
        link = self._link()
        if link is None:
            return
        val, ok = QInputDialog.getDouble(
            self, "Change Altitude",
            "New target altitude (m AGL):",
            value=100.0, min=1.0, max=5000.0, decimals=1,
        )
        if not ok:
            return
        # 若有 change_altitude() 方法則用之；否則退化為 takeoff(alt)
        if hasattr(link, "change_altitude"):
            link.change_altitude(val)
        else:
            link.takeoff(val)
        self.command_sent.emit(self._callsign or "", f"CHANGE_ALT {val:.1f}")

    def _on_follow_me(self) -> None:
        link = self._link()
        if link is None:
            return
        link.set_mode("FOLLOW")
        self.command_sent.emit(self._callsign or "", "FOLLOW")

    def _on_reboot(self) -> None:
        link = self._link()
        if link is None:
            return
        from PyQt6.QtWidgets import QMessageBox
        if QMessageBox.question(
            self, "Reboot Flight Controller",
            f"Reboot FC on {self._callsign}?\n\nMAVLink will reconnect after reboot.",
        ) != QMessageBox.StandardButton.Yes:
            return
        if hasattr(link, "reboot_fc"):
            link.reboot_fc()
        else:
            # 沒 reboot_fc 方法：在 status bar 給提示，不嘗試送指令
            self.command_sent.emit(
                self._callsign or "",
                "REBOOT_FC (not supported by this SITLLink)",
            )
            return
        self.command_sent.emit(self._callsign or "", "REBOOT_FC")

    # ── chip style helper ───────────────────────────
    @staticmethod
    def _chip_style(ok: bool = True) -> str:
        color = T.FRIENDLY if ok else T.HOSTILE
        return (
            f"background: {T.BG_ELEVATED}; color: {color}; "
            f"border: 1px solid {color}; border-radius: 8px; "
            f"padding: 1px 6px; font-family: {T.FONT_MONO_STACK}; "
            f"font-weight: 700; font-size: 10px;"
        )


__all__ = ["DroneDetailPanel"]
