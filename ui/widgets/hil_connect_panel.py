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


#: 下拉項的「埠 — 描述」分隔字串。連線只取前半段（見 :func:`_port_of`）。
_PORT_SEP = '  —  '


def _port_of(text: str) -> str:
    """下拉項可能是 ``'COM26  —  Cube Orange+ Mavlink'``，連線只要 ``COM26``。

    ⚠ 這個 combo 是 ``setEditable(True)``，使用者可能直接手打 ``udp:...``；
    那種字串沒有分隔符，原樣回傳。
    """
    return (text or '').split(_PORT_SEP)[0].strip()


def _scan_serial_ports() -> List[tuple]:
    """列出序列埠：``[(device, label, serial_number), ...]``；失敗回空 list。

    ⚠⚠ 為什麼不只回 ``device``：六顆同型飛控在 Windows 上全叫 ``COMxx``，
    而 **COM 編號會在重插／重開機後換**。只看編號認飛機，接錯**沒有任何徵狀**。
    實測一顆 Cube Orange+ 會列舉出兩個埠、共用同一個序號：
        COM26  Cube Orange+ Mavlink  VID:PID=0x2DAE:0x1058  serial=25003A000E51323039383833
        COM27  Cube Orange+ SLCAN    （CAN 介面，**不是** MAVLink，不要接這個）
    ⇒ 描述與序號才是穩定的識別，編號不是。
    """
    try:
        from serial.tools import list_ports
        out = []
        for p in list_ports.comports():
            desc = (p.description or '').strip()
            # pyserial 在 Windows 會把 "(COM26)" 併進 description，去掉免得重複
            if desc.endswith(f'({p.device})'):
                desc = desc[: -len(f'({p.device})')].strip()
            label = f'{p.device}{_PORT_SEP}{desc}' if desc else p.device
            out.append((p.device, label, p.serial_number or ''))
        return out
    except Exception as e:
        logger.warning(f'[HIL] 掃描序列埠失敗: {e}')
        return []


def probe_serial_port(port: str, baud: int = 115200, seconds: float = 2.0) -> tuple:
    """連線前診斷一個序列埠，回 ``(verdict, 給人看的說明)``。

    ``verdict``：``'ok' | 'busy' | 'silent' | 'not_mavlink' | 'error'``

    ⚠⚠⚠ 為什麼需要這個：沒有它，連不上只會得到一個**沉默的逾時**，
    而逾時的原因至少有四種、處置完全不同。2026-09-14 實測一顆 Cube Orange+：
    埠開得起來、有資料進來，但 **0xFD/0xFE 都是 0**，內容是 ASCII
    ``"Unable to init RAMTRON storage"`` 以 2 Hz 重複 —— 板子卡在參數儲存初始化、
    根本沒跑到 MAVLink。那是硬體/韌體問題，在 GCS 這端怎麼調都沒用。
    把這個判別做進來，才不會把「板子沒開機」誤診成「埠名或鮑率錯」。
    """
    try:
        import serial as _pyserial
    except Exception as e:                                    # noqa: BLE001
        return 'error', f'pyserial 不可用：{e}'
    try:
        s = _pyserial.Serial(port, baud, timeout=0.5)
    except Exception as e:                                    # noqa: BLE001
        name = type(e).__name__
        if 'Permission' in name or '存取' in str(e) or 'Access' in str(e):
            return 'busy', f'{port} 被其他程式佔用（Mission Planner／QGC？）：{e}'
        return 'error', f'{port} 開啟失敗（{name}）：{e}'
    try:
        import time as _time
        buf = b''
        t0 = _time.time()
        while _time.time() - t0 < max(0.5, float(seconds)):
            chunk = s.read(4096)
            if chunk:
                buf += chunk
    finally:
        try:
            s.close()
        except Exception:                                     # noqa: BLE001
            pass

    if not buf:
        return 'silent', (f'{port} 開得起來但 {seconds:g} 秒內 0 bytes —— '
                          f'飛控沒有在這個埠送資料（SERIAL_PROTOCOL？選到 SLCAN 埠？）')
    n2, n1 = buf.count(b'\xfd'), buf.count(b'\xfe')
    if n2 or n1:
        return 'ok', f'{port} 收到 {len(buf)} bytes，MAVLink 起始位元組 v2×{n2} / v1×{n1}'
    sample = buf.decode('ascii', errors='replace')[:80].replace('\r', ' ').replace('\n', ' ')
    return 'not_mavlink', (f'{port} 收到 {len(buf)} bytes 但**沒有任何 MAVLink 框頭**'
                           f'（0xFD/0xFE 皆 0）。內容像主控台文字：「{sample}」'
                           f' ⇒ 飛控沒開機到 MAVLink 階段，先修板子')


class HILConnectPanel(QWidget):
    """HIL 多飛控連線面板（最多 6 顆）。"""

    connect_requested = pyqtSignal(list)
    disconnect_requested = pyqtSignal()

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._connected = False
        self._port_serials: dict = {}      # device -> USB 序號（穩定識別）
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
        found = _scan_serial_ports()
        self._port_serials = {dev: sn for dev, _lbl, sn in found}
        for combo in self._row_port:
            current = combo.currentText()
            combo.blockSignals(True)
            combo.clear()
            for dev, label, sn in found:
                combo.addItem(label)
                if sn:
                    # 序號才是穩定識別；COM 編號會在重插後換（見 _scan_serial_ports）
                    combo.setItemData(combo.count() - 1, f'{dev}  序號 {sn}',
                                      Qt.ItemDataRole.ToolTipRole)
            # 常用網路端點樣板（可手改 IP/port）
            combo.addItems(['udp:0.0.0.0:14550', 'tcp:127.0.0.1:5760'])
            if current:
                combo.setCurrentText(current)
            combo.blockSignals(False)
        if not self._connected:
            names = ', '.join(lbl for _d, lbl, _s in found)
            self.lbl_status.setText(f'序列埠掃描：{names or "（無）"}')

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
                    'port': _port_of(self._row_port[r].currentText()),
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
            port = _port_of(self._row_port[r].currentText())
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
            # ⚠⚠⚠ 不可以只說「檢查埠名 / 鮑率 / 線材」——那是四種完全不同處置的
            #   集合名詞，而使用者無從分辨。失敗時**實際去問那個埠**，把原因講出來。
            #   實測價值：一顆 Cube Orange+ 連不上，原因是它以 2 Hz 重複吐
            #   "Unable to init RAMTRON storage"、零個 MAVLink 框頭 ——
            #   板子沒開機到 MAVLink 階段，怎麼改埠名鮑率都沒用。
            self.lbl_status.setText(f'連線失敗（0/{total}）— 診斷中…')
            reasons = self._diagnose_failed_rows()
            detail = ('；'.join(reasons) if reasons
                      else '沒有可診斷的序列埠列（網路端點請檢查 IP / 防火牆）')
            self.lbl_status.setText(f'連線失敗（0/{total}）— {detail}')
            self.lbl_status.setToolTip(detail)
            logger.error(f'[HIL] 連線失敗 0/{total}：{detail}')
            self.lbl_status.setStyleSheet(
                f'color:{TC.HOSTILE};font-size:11px;font-family:{TF.css_mono()};'
            )

    def _diagnose_failed_rows(self, max_rows: int = 3) -> List[str]:
        """連線失敗後，逐個序列埠實際探測並回傳人看得懂的原因。

        只在失敗路徑跑（成功時完全不花時間），且最多探 ``max_rows`` 個埠、
        每個 1.5 s，避免 6 顆全失敗時把 UI 卡住十幾秒。
        """
        out: List[str] = []
        for row in self._gather_rows()[:max_rows]:
            conn = str(row.get('conn', ''))
            if any(conn.lower().startswith(x) for x in
                   ('udp:', 'udpin:', 'udpout:', 'tcp:', 'tcpin:')):
                continue                       # 網路端點不走序列埠探測
            port, _, baud = conn.partition(',')
            try:
                verdict, msg = probe_serial_port(port, int(baud or 115200), seconds=1.5)
            except Exception as e:             # noqa: BLE001
                out.append(f'{port} 探測失敗：{e}')
                continue
            out.append(msg if verdict != 'ok' else
                       f'{msg}（有 MAVLink 但鏈路仍未建立 —— 檢查 SYSID 是否相符）')
        return out

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
