"""嵌入式蜂群連線設定彈窗（模式 A1：AeroPlan 當監看 GCS）

對應固定翼蜂群 SITL 文件 §4.1：
    嵌入式裝置（Jetson）跑「原生 ArduPlane SITL」，每個 SITL 實例 -I<N> 會在
    TCP `5760 + 10*N`（5760 / 5770 / 5780 ...）開一個 TCP server，AeroPlan Studio
    透過 TCP 連這些埠（每機一條），只做即時顯示／監看，導引由嵌入式 ROS 蜂群負責。

與「內建 SITL 啟動」(SITLLaunchDialog) 的差別：
    ● 本對話框不啟動任何本機 SITL 子行程，純粹建立 N 條 MAVLink 監看連線。
    ● 連線方向二選一：
        dial   — AeroPlan 主動連往嵌入式  → tcp:<host>:<port>
                 （對端為原生 ArduPlane SITL -I<N>，或 mavproxy --out tcpin:0.0.0.0:<port>）
        listen — AeroPlan 當 TCP server     → tcpin:0.0.0.0:<port>
                 （需 mavproxy --out tcp:<Windows_IP>:<port> 主動連入）

對外提供：
    EmbeddedSwarmDialog.get_config(parent=None) -> Optional[dict]
        modal 執行，回傳：
            None                              # 使用者取消
            {                                 # 使用者按連線
                'host': str,                  # 嵌入式 IP（listen 模式為空字串）
                'count': int,
                'base_port': int,
                'port_stride': int,
                'sysid_base': int,
                'direction': 'dial' | 'listen',
                'save_as_default': bool,
                'connections': [              # 已組好的每機連線描述
                    {'sysid': int, 'conn_str': str},
                    ...
                ],
            }
"""

from __future__ import annotations

from typing import List, Optional

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QComboBox, QCheckBox, QDialog, QFormLayout, QGroupBox, QHBoxLayout,
    QHeaderView, QLabel, QLineEdit, QMessageBox, QPushButton, QSpinBox,
    QTableWidget, QTableWidgetItem, QTextEdit, QVBoxLayout, QWidget,
)

from config.settings import get_settings
from mission.sitl_launcher import _get_lan_ip
from ui.resources.aeroplan_theme.widgets import IconButton
from utils.logger import get_logger

logger = get_logger()


def _is_valid_ipv4(text: str) -> bool:
    """嚴格 IPv4 驗證：4 段、每段 0–255。"""
    parts = text.strip().split('.')
    if len(parts) != 4:
        return False
    try:
        return all(0 <= int(p) <= 255 for p in parts)
    except ValueError:
        return False


class EmbeddedSwarmDialog(QDialog):
    """嵌入式蜂群連線設定彈窗。

    UI 結構：
        ┌ 標題 / 本機 LAN IP 說明
        ┌ 參數表單（IP / 機數 / 基準 port / port 間隔 / sysid 起始 / 連線方向）
        ┌ 連線預覽表格（# / SYSID / 連線字串）
        ┌ 選項（儲存為下次預設）
        ┌ mavproxy --out 對應提示（依方向動態產生）
        └ 取消 / 連線 按鈕
    """

    # 連線方向選項：(顯示文字, 內部代碼)
    _DIRECTIONS = (
        ('主動連線到 SITL／mavproxy  (tcp，推薦)', 'dial'),
        ('被動等待對端連入 (tcpin)', 'listen'),
    )

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setWindowTitle('嵌入式蜂群連線設定')
        self.setMinimumSize(640, 560)
        self.setModal(True)

        self._lan_ip = _get_lan_ip()
        self._saved = get_settings().embedded_swarm

        self._init_ui()
        self._populate_from_saved()
        self._refresh_preview()

    # ──────────────────────────────────────────────────────────────
    # UI 建構
    # ──────────────────────────────────────────────────────────────
    def _init_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        # ── 標題 ───────────────────────────────────────────────
        title = QLabel('連線到嵌入式蜂群（Jetson 原生 ArduPlane SITL，TCP 監看）')
        title.setProperty('role', 'emphasis')
        title.style().polish(title)
        root.addWidget(title)

        desc = QLabel(
            '本對話框不啟動內建 SITL，只建立 N 條 MAVLink 監看連線。'
            '導引由嵌入式 ROS 蜂群負責，AeroPlan 僅顯示遙測。'
        )
        desc.setProperty('role', 'caption')
        desc.setWordWrap(True)
        desc.style().polish(desc)
        root.addWidget(desc)

        lan_lbl = QLabel(f'本機 LAN IP：{self._lan_ip}    （listen 模式下對端 TCP 主動連入的目標）')
        lan_lbl.setProperty('role', 'caption')
        lan_lbl.style().polish(lan_lbl)
        root.addWidget(lan_lbl)

        # ── 參數表單 ───────────────────────────────────────────
        form_group = QGroupBox('連線參數')
        form = QFormLayout(form_group)
        form.setContentsMargins(8, 8, 8, 8)
        form.setSpacing(6)

        self.edit_host = QLineEdit()
        self.edit_host.setPlaceholderText('例如 192.168.1.50（嵌入式 Jetson 的 LAN IP）')
        self.edit_host.textChanged.connect(self._refresh_preview)
        form.addRow('嵌入式主機 IP：', self.edit_host)

        self.spin_count = QSpinBox()
        self.spin_count.setRange(1, 16)
        self.spin_count.setValue(4)
        self.spin_count.setSuffix(' 機')
        self.spin_count.valueChanged.connect(self._refresh_preview)
        form.addRow('監看機數：', self.spin_count)

        self.spin_base_port = QSpinBox()
        self.spin_base_port.setRange(1024, 65535)
        self.spin_base_port.setValue(5760)  # 原生 ArduPlane SITL -I0 的 TCP 埠
        self.spin_base_port.valueChanged.connect(self._refresh_preview)
        form.addRow('基準 TCP Port：', self.spin_base_port)

        self.spin_stride = QSpinBox()
        self.spin_stride.setRange(1, 1000)
        self.spin_stride.setValue(10)
        self.spin_stride.valueChanged.connect(self._refresh_preview)
        form.addRow('Port 間隔：', self.spin_stride)

        self.spin_sysid_base = QSpinBox()
        self.spin_sysid_base.setRange(1, 250)
        self.spin_sysid_base.setValue(1)  # 原生 SITL -I0..N 慣例：sysid = 1..N+1
        self.spin_sysid_base.valueChanged.connect(self._refresh_preview)
        form.addRow('SYSID 起始：', self.spin_sysid_base)

        self.combo_direction = QComboBox()
        for text, code in self._DIRECTIONS:
            self.combo_direction.addItem(text, code)
        self.combo_direction.currentIndexChanged.connect(self._on_direction_changed)
        form.addRow('連線方向：', self.combo_direction)

        root.addWidget(form_group)

        # ── 連線預覽 ───────────────────────────────────────────
        preview_group = QGroupBox('連線預覽（每機一條 MAVLink 監看）')
        pv_layout = QVBoxLayout(preview_group)
        pv_layout.setContentsMargins(8, 8, 8, 8)

        self.table = QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(['#', 'SYSID', '連線字串'])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.Stretch)
        pv_layout.addWidget(self.table)
        root.addWidget(preview_group)

        # ── 選項 ───────────────────────────────────────────────
        self.chk_save_default = QCheckBox('儲存為下次預設')
        self.chk_save_default.setChecked(True)
        root.addWidget(self.chk_save_default)

        # ── mavproxy --out 對應提示 ────────────────────────────
        hint_group = QGroupBox('嵌入式 SITL／mavproxy TCP 對應（貼到 Jetson 端腳本）')
        hint_layout = QVBoxLayout(hint_group)
        hint_layout.setContentsMargins(8, 8, 8, 8)
        self.mavproxy_hint = QTextEdit()
        self.mavproxy_hint.setReadOnly(True)
        self.mavproxy_hint.setFixedHeight(96)
        hint_layout.addWidget(self.mavproxy_hint)
        root.addWidget(hint_group)

        # ── 按鈕列 ─────────────────────────────────────────────
        btn_row = QHBoxLayout()
        btn_row.addStretch()
        self.btn_cancel = QPushButton('取消')
        self.btn_cancel.clicked.connect(self.reject)
        btn_row.addWidget(self.btn_cancel)

        self.btn_connect = IconButton('drone', '連線', tone='primary')
        self.btn_connect.setDefault(True)
        self.btn_connect.clicked.connect(self._on_connect_clicked)
        btn_row.addWidget(self.btn_connect)
        root.addLayout(btn_row)

    # ──────────────────────────────────────────────────────────────
    # 預設值載入
    # ──────────────────────────────────────────────────────────────
    # 舊版 UDP 監看的基準埠預設值（14550）；改用 TCP 後不再適用，
    # 載入到此值時一律遷移為原生 SITL 的 TCP 基準埠，免得預覽顯示無效的 tcp:…:14550。
    _LEGACY_UDP_BASE_PORT = 14550
    _TCP_DEFAULT_BASE_PORT = 5760

    def _populate_from_saved(self) -> None:
        s = self._saved
        if s.host:
            self.edit_host.setText(str(s.host))
        self.spin_count.setValue(int(s.count))
        base_port = int(s.base_port)
        if base_port == self._LEGACY_UDP_BASE_PORT:
            base_port = self._TCP_DEFAULT_BASE_PORT
        self.spin_base_port.setValue(base_port)
        self.spin_stride.setValue(int(s.port_stride))
        self.spin_sysid_base.setValue(int(s.sysid_base))
        # 還原方向下拉選擇
        idx = self.combo_direction.findData(s.direction)
        if idx >= 0:
            self.combo_direction.setCurrentIndex(idx)
        self._on_direction_changed()

    # ──────────────────────────────────────────────────────────────
    # 內部計算
    # ──────────────────────────────────────────────────────────────
    def _current_direction(self) -> str:
        """回傳目前選取的連線方向代碼 'dial' / 'listen'。"""
        return self.combo_direction.currentData() or 'dial'

    def _build_connections(self) -> List[dict]:
        """依目前 UI 狀態組出每機連線描述 [{'sysid', 'conn_str'}, ...]。"""
        host = self.edit_host.text().strip()
        count = self.spin_count.value()
        base_port = self.spin_base_port.value()
        stride = self.spin_stride.value()
        sysid_base = self.spin_sysid_base.value()
        direction = self._current_direction()

        conns: List[dict] = []
        for i in range(count):
            port = base_port + i * stride
            if direction == 'dial':
                conn_str = f'tcp:{host}:{port}'
            else:
                conn_str = f'tcpin:0.0.0.0:{port}'
            conns.append({'sysid': sysid_base + i, 'conn_str': conn_str})
        return conns

    # ──────────────────────────────────────────────────────────────
    # Signal handlers
    # ──────────────────────────────────────────────────────────────
    def _on_direction_changed(self, *_args) -> None:
        """dial 模式才需要嵌入式 IP；listen 模式停用 IP 欄位。"""
        is_dial = self._current_direction() == 'dial'
        self.edit_host.setEnabled(is_dial)
        self._refresh_preview()

    def _refresh_preview(self) -> None:
        """重建預覽表格 + mavproxy 提示。"""
        conns = self._build_connections()
        self.table.setRowCount(len(conns))
        for i, c in enumerate(conns):
            idx_item = QTableWidgetItem(str(i))
            idx_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.table.setItem(i, 0, idx_item)

            sysid_item = QTableWidgetItem(str(c['sysid']))
            sysid_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.table.setItem(i, 1, sysid_item)

            self.table.setItem(i, 2, QTableWidgetItem(c['conn_str']))

        self._refresh_mavproxy_hint(conns)

    def _refresh_mavproxy_hint(self, conns: List[dict]) -> None:
        """依連線方向產生對端 SITL／mavproxy TCP 範例。"""
        direction = self._current_direction()
        lines: List[str] = []
        if direction == 'dial':
            lines.append('# AeroPlan 主動連往對端 TCP server（原生 SITL -I<N> 自動開埠，'
                         '或 mavproxy --out tcpin）：')
            for i, c in enumerate(conns):
                port = c['conn_str'].rsplit(':', 1)[1]
                lines.append(
                    f'#  第 {i} 機 (sysid={c["sysid"]}) → '
                    f'SITL -I{i}（TCP {port}）或 mavproxy --out tcpin:0.0.0.0:{port}'
                )
        else:
            lines.append('# AeroPlan 當 TCP server → 對端 mavproxy 主動連入本機 LAN IP：')
            for i, c in enumerate(conns):
                port = c['conn_str'].rsplit(':', 1)[1]
                lines.append(
                    f'#  第 {i} 機 (sysid={c["sysid"]}) → '
                    f'--out tcp:{self._lan_ip}:{port}'
                )
        self.mavproxy_hint.setPlainText('\n'.join(lines))

    # ──────────────────────────────────────────────────────────────
    # 連線按鈕：驗證 → 收集 → （可選）持久化 → accept
    # ──────────────────────────────────────────────────────────────
    def _on_connect_clicked(self) -> None:
        direction = self._current_direction()
        host = self.edit_host.text().strip()

        # dial 模式必須有合法 IP
        if direction == 'dial' and not _is_valid_ipv4(host):
            QMessageBox.warning(
                self, 'IP 無效',
                '主動連線模式需要嵌入式主機的合法 IPv4 位址\n'
                '（例如 192.168.1.50）。\n'
                '若要讓嵌入式主動連入本機，請改選「被動等待對端連入」。',
            )
            self.edit_host.setFocus()
            return

        # port 範圍溢位檢查（base + (count-1)*stride 不可超過 65535）
        max_port = self.spin_base_port.value() + (self.spin_count.value() - 1) * self.spin_stride.value()
        if max_port > 65535:
            QMessageBox.warning(
                self, 'Port 超出範圍',
                f'最後一機的 port 會是 {max_port}，超過 65535。\n'
                f'請降低機數、基準 port 或 port 間隔。',
            )
            return

        result = self.collect_result()

        # 持久化
        if result.get('save_as_default'):
            try:
                gs = get_settings()
                es = gs.embedded_swarm
                es.host = result['host']
                es.count = result['count']
                es.base_port = result['base_port']
                es.port_stride = result['port_stride']
                es.sysid_base = result['sysid_base']
                es.direction = result['direction']
                if gs.save():
                    logger.info('[Embedded Swarm] 已儲存嵌入式蜂群連線設定為預設')
                else:
                    logger.warning('[Embedded Swarm] 儲存設定失敗（save() 回傳 False）')
            except Exception as e:
                logger.warning(f'[Embedded Swarm] 持久化例外: {e}')

        self._result = result
        self.accept()

    def collect_result(self) -> dict:
        """從目前 UI 狀態組出回傳 dict。"""
        direction = self._current_direction()
        host = self.edit_host.text().strip() if direction == 'dial' else ''
        return {
            'host': host,
            'count': self.spin_count.value(),
            'base_port': self.spin_base_port.value(),
            'port_stride': self.spin_stride.value(),
            'sysid_base': self.spin_sysid_base.value(),
            'direction': direction,
            'save_as_default': bool(self.chk_save_default.isChecked()),
            'connections': self._build_connections(),
        }

    # ──────────────────────────────────────────────────────────────
    # Convenience static entry
    # ──────────────────────────────────────────────────────────────
    @staticmethod
    def get_config(parent: Optional[QWidget] = None) -> Optional[dict]:
        """便利方法：執行 modal 並回傳結果 dict，使用者取消則回 None。"""
        dlg = EmbeddedSwarmDialog(parent=parent)
        if dlg.exec() == QDialog.DialogCode.Accepted:
            return getattr(dlg, '_result', dlg.collect_result())
        return None
