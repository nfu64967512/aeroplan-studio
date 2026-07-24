"""ui/widgets/hil_connect_panel.py — HIL（Hardware-in-the-Loop）多飛控連線面板。

SITL 分頁的第三個內頁：一次把最多 6 顆「實體飛控」接進 GCS，
每顆走獨立 MAVLink 鏈路（序列埠 COM 或 udp:/tcp: 端點），
與內建 SITL 共用同一套 SITLLink / FleetRegistry 遙測與指令管線 —
連上後 fleet rail、HUD 多機卡片、任務上傳、broadcast 指令全部可用。

UI 結構：
    ┌ 說明列（HIL 定位 + 本機序列埠掃描結果）
    ┌ 6 列連線表格：啟用 / 連接埠 / 鮑率 / SYSID / 載具
    ┌ 按鈕列：掃描序列埠 / 全部連線 / 全部斷線
    └ 狀態列（已連線數量）

對外訊號：
    connect_requested(list)   # [{'conn': 'COM7,115200', 'sysid': 1, 'vehicle_hint': ''}, ...]
    disconnect_requested()

設定持久化：config/settings.json → 'hil'（HILSettings.instances）。
"""
from __future__ import annotations

from typing import List, Optional

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QComboBox, QFrame, QHBoxLayout, QHeaderView, QLabel, QSpinBox,
    QTableWidget, QVBoxLayout, QWidget,
)

from config.settings import get_settings
from ui.resources.tactical_theme import TacticalColors as TC, TacticalFonts as TF
from ui.resources.aeroplan_theme.widgets import IconButton
from utils.logger import get_logger

logger = get_logger()

MAX_FC = 6  # 一次最多接 6 顆飛控

_BAUD_RATES = ['57600', '115200', '230400', '460800', '921600', '1500000']
_VEHICLE_HINTS = ['自動', 'PLANE', 'COPTER', 'VTOL']


def _scan_serial_ports() -> List[str]:
    """列出目前系統可見的序列埠（pyserial；失敗回空 list）。"""
    try:
        from serial.tools import list_ports
        return [p.device for p in list_ports.comports()]
    except Exception as e:
        logger.warning(f'[HIL] 掃描序列埠失敗: {e}')
        return []


class HILConnectPanel(QWidget):
    """HIL 多飛控連線面板（最多 6 顆）。"""

    connect_requested = pyqtSignal(list)
    disconnect_requested = pyqtSignal()

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._connected = False
        self._init_ui()
        self._populate_from_saved()
        self.refresh_ports()

    # ──────────────────────────────────────────────────────────────
    # UI 建構
    # ──────────────────────────────────────────────────────────────
    def _init_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # ── 說明列 ────────────────────────────────────────────
        info = QLabel(
            'HIL 硬體在環：把實體飛控（USB 序列埠或 udp:/tcp: 端點）直接接進 GCS，'
            f'一次最多 {MAX_FC} 顆。連上後與 SITL 共用 HUD 多機卡片、fleet rail、'
            '任務上傳與 ARM / 模式廣播指令。'
        )
        info.setWordWrap(True)
        info.setStyleSheet(
            f'color:{TC.FG_SECONDARY};font-size:11px;font-family:{TF.css_sans()};'
        )
        root.addWidget(info)

        # ── 連線表格 ──────────────────────────────────────────
        self.table = QTableWidget(MAX_FC, 5)
        self.table.setHorizontalHeaderLabels(['啟用', '連接埠', '鮑率', 'SYSID', '載具'])
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(4, QHeaderView.ResizeMode.ResizeToContents)
        self.table.setStyleSheet(
            f'QTableWidget{{background:{TC.BG_SUNKEN};color:{TC.FG_PRIMARY};'
            f'border:1px solid {TC.BORDER_DEFAULT};font-size:11px;'
            f'font-family:{TF.css_sans()};}}'
        )

        self._row_enable: List[QComboBox] = []   # 用兩態 combo 當 checkbox（樣式一致）
        self._row_port: List[QComboBox] = []
        self._row_baud: List[QComboBox] = []
        self._row_sysid: List[QSpinBox] = []
        self._row_vehicle: List[QComboBox] = []

        combo_qss = (
            f'QComboBox{{background:{TC.BG_SUNKEN};color:{TC.FG_PRIMARY};'
            f'border:1px solid {TC.BORDER_DEFAULT};border-radius:0;'
            f'padding:2px 4px;font-size:11px;font-family:{TF.css_mono()};}}'
        )
        for row in range(MAX_FC):
            en = QComboBox()
            en.addItems(['—', '✓'])
            en.setCurrentIndex(1 if row == 0 else 0)
            en.setStyleSheet(combo_qss)
            self.table.setCellWidget(row, 0, en)
            self._row_enable.append(en)

            port = QComboBox()
            port.setEditable(True)
            port.setStyleSheet(combo_qss)
            self.table.setCellWidget(row, 1, port)
            self._row_port.append(port)

            baud = QComboBox()
            baud.addItems(_BAUD_RATES)
            baud.setCurrentText('115200')
            baud.setStyleSheet(combo_qss)
            self.table.setCellWidget(row, 2, baud)
            self._row_baud.append(baud)

            sysid = QSpinBox()
            sysid.setRange(1, 255)
            sysid.setValue(row + 1)
            sysid.setStyleSheet(
                f'QSpinBox{{background:{TC.BG_SUNKEN};color:{TC.FG_PRIMARY};'
                f'border:1px solid {TC.BORDER_DEFAULT};border-radius:0;'
                f'padding:2px 4px;font-size:11px;font-family:{TF.css_mono()};}}'
            )
            self.table.setCellWidget(row, 3, sysid)
            self._row_sysid.append(sysid)

            veh = QComboBox()
            veh.addItems(_VEHICLE_HINTS)
            veh.setStyleSheet(combo_qss)
            self.table.setCellWidget(row, 4, veh)
            self._row_vehicle.append(veh)

        self.table.setMinimumHeight(220)
        root.addWidget(self.table)

        # ── 按鈕列 ────────────────────────────────────────────
        btn_frame = QFrame()
        btn_frame.setStyleSheet(
            f'QFrame{{background:{TC.BG_SECONDARY};border:1px solid {TC.BORDER_DEFAULT};border-radius:0;}}'
        )
        brow = QHBoxLayout(btn_frame)
        brow.setContentsMargins(8, 6, 8, 6)
        brow.setSpacing(6)

        self.btn_scan = IconButton('radar', '掃描序列埠', tone='neutral', compact=True)
        self.btn_scan.clicked.connect(self.refresh_ports)
        brow.addWidget(self.btn_scan)
        brow.addStretch(1)

        self.btn_connect_all = IconButton('link', '全部連線', tone='primary', compact=True)
        self.btn_connect_all.clicked.connect(self._on_connect_all_clicked)
        brow.addWidget(self.btn_connect_all)

        self.btn_disconnect_all = IconButton('unlink', '全部斷線', tone='danger', compact=True)
        self.btn_disconnect_all.clicked.connect(self._on_disconnect_all_clicked)
        self.btn_disconnect_all.setEnabled(False)
        brow.addWidget(self.btn_disconnect_all)

        root.addWidget(btn_frame)

        # ── 狀態列 ────────────────────────────────────────────
        self.lbl_status = QLabel('未連線')
        self.lbl_status.setStyleSheet(
            f'color:{TC.FG_MUTED};font-size:11px;font-family:{TF.css_mono()};'
        )
        root.addWidget(self.lbl_status)
        root.addStretch(1)

    # ──────────────────────────────────────────────────────────────
    # 序列埠掃描 / 設定回填
    # ──────────────────────────────────────────────────────────────
    def refresh_ports(self) -> None:
        """重新掃描序列埠，回填各列 combo（保留使用者手打的值）。"""
        ports = _scan_serial_ports()
        for combo in self._row_port:
            current = combo.currentText()
            combo.blockSignals(True)
            combo.clear()
            combo.addItems(ports)
            # 常用網路端點樣板（可手改 IP/port）
            combo.addItems(['udp:0.0.0.0:14550', 'tcp:127.0.0.1:5760'])
            if current:
                combo.setCurrentText(current)
            combo.blockSignals(False)
        self.lbl_status.setText(
            f'序列埠掃描：{", ".join(ports) if ports else "（無）"}'
            if not self._connected else self.lbl_status.text()
        )

    def _populate_from_saved(self) -> None:
        """從 settings.json::hil 回填上次的連線配置。"""
        try:
            saved = get_settings().hil.instances or []
        except Exception:
            saved = []
        for row, inst in enumerate(saved[:MAX_FC]):
            try:
                self._row_enable[row].setCurrentIndex(1 if inst.get('enabled') else 0)
                if inst.get('port'):
                    self._row_port[row].setCurrentText(str(inst['port']))
                if inst.get('baud'):
                    self._row_baud[row].setCurrentText(str(inst['baud']))
                if inst.get('sysid'):
                    self._row_sysid[row].setValue(int(inst['sysid']))
                hint = inst.get('vehicle_hint') or '自動'
                if hint in _VEHICLE_HINTS:
                    self._row_vehicle[row].setCurrentText(hint)
            except Exception as e:
                logger.warning(f'[HIL] 回填設定第 {row + 1} 列失敗: {e}')

    def _save_to_settings(self) -> None:
        """把目前表格內容存回 settings.json::hil。"""
        try:
            s = get_settings()
            s.hil.instances = [
                {
                    'enabled': self._row_enable[r].currentIndex() == 1,
                    'port': self._row_port[r].currentText().strip(),
                    'baud': int(self._row_baud[r].currentText()),
                    'sysid': self._row_sysid[r].value(),
                    'vehicle_hint': self._row_vehicle[r].currentText(),
                }
                for r in range(MAX_FC)
            ]
            s.save()
        except Exception as e:
            logger.warning(f'[HIL] 儲存設定失敗: {e}')

    # ──────────────────────────────────────────────────────────────
    # 連線 / 斷線
    # ──────────────────────────────────────────────────────────────
    def _gather_rows(self) -> List[dict]:
        """收集啟用列 → SITLLink 連線字串；序列埠附上鮑率。"""
        rows: List[dict] = []
        seen_sysids: set = set()
        seen_ports: set = set()
        for r in range(MAX_FC):
            if self._row_enable[r].currentIndex() != 1:
                continue
            port = self._row_port[r].currentText().strip()
            if not port:
                continue
            sysid = self._row_sysid[r].value()
            if sysid in seen_sysids or port in seen_ports:
                logger.warning(f'[HIL] 第 {r + 1} 列 SYSID/埠重複，略過: {port} sysid={sysid}')
                continue
            seen_sysids.add(sysid)
            seen_ports.add(port)
            # udp:/tcp:/udpin: 端點原樣使用；其餘視為序列埠，補鮑率
            if any(port.lower().startswith(p) for p in ('udp:', 'udpin:', 'udpout:', 'tcp:', 'tcpin:')):
                conn = port
            else:
                conn = f'{port},{self._row_baud[r].currentText()}'
            hint = self._row_vehicle[r].currentText()
            rows.append({
                'conn': conn,
                'sysid': sysid,
                'vehicle_hint': '' if hint == '自動' else hint,
            })
        return rows

    def _on_connect_all_clicked(self) -> None:
        rows = self._gather_rows()
        if not rows:
            self.lbl_status.setText('沒有可連線的列：請勾選啟用並填入連接埠')
            self.lbl_status.setStyleSheet(
                f'color:{TC.WARNING};font-size:11px;font-family:{TF.css_mono()};'
            )
            return
        self._save_to_settings()
        self.btn_connect_all.setEnabled(False)
        self.lbl_status.setText(f'連線中…（{len(rows)} 顆飛控）')
        self.connect_requested.emit(rows)

    def _on_disconnect_all_clicked(self) -> None:
        self.disconnect_requested.emit()

    # ──────────────────────────────────────────────────────────────
    # MainWindow 回呼
    # ──────────────────────────────────────────────────────────────
    def on_connect_result(self, ok: int, total: int) -> None:
        """MainWindow 完成連線嘗試後回報（ok = 成功建立鏈路數）。"""
        self._connected = ok > 0
        self.btn_connect_all.setEnabled(not self._connected)
        self.btn_disconnect_all.setEnabled(self._connected)
        self._set_table_enabled(not self._connected)
        if ok > 0:
            self.lbl_status.setText(f'已連線 {ok}/{total} 顆飛控（遙測見 HUD / Command 頁）')
            self.lbl_status.setStyleSheet(
                f'color:{TC.FRIENDLY};font-size:11px;font-family:{TF.css_mono()};'
            )
        else:
            self.lbl_status.setText(f'連線失敗（0/{total}）— 檢查埠名 / 鮑率 / 線材')
            self.lbl_status.setStyleSheet(
                f'color:{TC.HOSTILE};font-size:11px;font-family:{TF.css_mono()};'
            )

    def on_disconnected(self) -> None:
        """MainWindow 完成斷線後回報。"""
        self._connected = False
        self.btn_connect_all.setEnabled(True)
        self.btn_disconnect_all.setEnabled(False)
        self._set_table_enabled(True)
        self.lbl_status.setText('未連線')
        self.lbl_status.setStyleSheet(
            f'color:{TC.FG_MUTED};font-size:11px;font-family:{TF.css_mono()};'
        )

    def _set_table_enabled(self, enabled: bool) -> None:
        self.table.setEnabled(enabled)
        self.btn_scan.setEnabled(enabled)
