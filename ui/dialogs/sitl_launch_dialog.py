"""SITL 啟動前的配置彈窗
讓使用者設定每台 SITL 的 sysid、本機 UDP 監聽 port，
並支援儲存為下次預設（持久化到 config/settings.json）。

架構：
    Windows SITL 被動 listen（--serial1=udpin:0.0.0.0:<port>）
    嵌入式 MAVROS apm.launch 主動 send：
        fcu_url:=udp://:<local_port>@<windows_ip>:<port>
    （避免 SITL 啟動時對嵌入式 IP 做 ARP 解析而 hang 住）

對外提供：
    SITLLaunchDialog.get_config(vehicle, count, parent=None) -> Optional[dict]
        modal 執行，回傳：
            None                             # 使用者取消
            {                                # 使用者按啟動
                'enable_fanout': bool,
                'auto_firewall': bool,
                'save_as_default': bool,
                'instances': [
                    {
                        'sysid': int,
                        'embedded_port': int,   # SITL 本地 UDP listen port
                        'embedded_ip': str,     # 已保留欄位但不再使用（向下相容）
                    },
                    ...
                ],
            }
"""

from __future__ import annotations

import re
from typing import Optional, List

from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout, QLabel,
    QSpinBox, QLineEdit, QCheckBox, QPushButton, QGroupBox,
    QTableWidget, QTableWidgetItem, QHeaderView, QTextEdit,
    QMessageBox, QWidget,
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QRegularExpressionValidator
from PyQt6.QtCore import QRegularExpression

from config.settings import get_settings
from mission.sitl_launcher import _get_lan_ip
from ui.resources.aeroplan_theme.widgets import IconButton
from utils.logger import get_logger

logger = get_logger()


# IPv4 簡易 regex（不嚴格驗 255 上限，但能擋掉非數字輸入）
_IPV4_REGEX = QRegularExpression(
    r'^(\d{1,3})\.(\d{1,3})\.(\d{1,3})\.(\d{1,3})$'
)


def _is_valid_ipv4(text: str) -> bool:
    """嚴格 IPv4 驗證：4 段、每段 0–255"""
    parts = text.strip().split('.')
    if len(parts) != 4:
        return False
    try:
        return all(0 <= int(p) <= 255 for p in parts)
    except ValueError:
        return False


class SITLLaunchDialog(QDialog):
    """SITL 啟動前的配置彈窗。
    UI 結構：
        ┌ 標題列：載具類型 / 數量
        ┌ 每台 SITL 設定表格（sysid / 嵌入式 IP / 嵌入式 Port）
        ┌ 全域選項：啟用 fan-out / 自動防火牆 / 儲存為預設
        ┌ MAVROS apm.launch 提示框
        └ 取消 / 啟動 按鈕
    """

    def __init__(self,
                 vehicle: str,
                 count: int,
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._vehicle = vehicle
        self._count = max(1, int(count))

        self.setWindowTitle('SITL 啟動設定')
        self.setMinimumSize(640, 520)
        self.setModal(True)

        # 讀取上次儲存的設定當作預設值
        self._saved = get_settings().sitl_launch

        self._init_ui()
        self._populate_from_saved()
        self._refresh_apm_hint()

    # ──────────────────────────────────────────────────────────────
    # UI 建構
    # ──────────────────────────────────────────────────────────────
    def _init_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        # ── 標題列 ─────────────────────────────────────────────
        title = QLabel(f'載具：{self._vehicle}    數量：{self._count} 台')
        # 用 emphasis role 取得 amber mono-font 強調樣式
        title.setProperty('role', 'emphasis')
        title.style().polish(title)
        root.addWidget(title)

        # 顯示本機 LAN IP — MAVROS apm.launch 需要這個位址當作 send 目標
        self._lan_ip = _get_lan_ip()
        lan_lbl = QLabel(
            f'本機 LAN IP：{self._lan_ip}    '
            f'（嵌入式 MAVROS apm.launch 的 fcu_url remote host）'
        )
        lan_lbl.setProperty('role', 'caption')
        lan_lbl.style().polish(lan_lbl)
        root.addWidget(lan_lbl)

        # ── 表格：每台 SITL 設定 ────────────────────────────────
        # 3 欄結構：#（instance 編號）/ SYSID / SITL UDP 監聽 port
        # （SITL 改為被動 udpin，不再需要嵌入式 IP；嵌入式只要主動 send 到
        #  Windows LAN IP 的對應 port 即可）
        table_group = QGroupBox('每台 SITL 設定')
        tg_layout = QVBoxLayout(table_group)
        tg_layout.setContentsMargins(8, 8, 8, 8)

        self.table = QTableWidget(self._count, 3)
        self.table.setHorizontalHeaderLabels(['#', 'SYSID', 'SITL UDP 監聽 Port'])
        self.table.verticalHeader().setVisible(False)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.Stretch)

        for i in range(self._count):
            self._build_row(i)

        tg_layout.addWidget(self.table)
        root.addWidget(table_group)

        # ── 全域選項 ───────────────────────────────────────────
        opts_group = QGroupBox('選項')
        opts_layout = QVBoxLayout(opts_group)
        opts_layout.setContentsMargins(8, 8, 8, 8)

        self.chk_enable_fanout = QCheckBox('啟用嵌入式 fan-out（取消 = 純本機 SITL，沿用舊行為）')
        self.chk_enable_fanout.toggled.connect(self._on_fanout_toggled)
        opts_layout.addWidget(self.chk_enable_fanout)

        self.chk_auto_firewall = QCheckBox('啟動時自動建立 Windows Firewall 入站規則（首次需 UAC 同意）')
        opts_layout.addWidget(self.chk_auto_firewall)

        self.chk_save_default = QCheckBox('儲存為下次啟動預設')
        opts_layout.addWidget(self.chk_save_default)

        root.addWidget(opts_group)

        # ── MAVROS apm.launch 提示 ─────────────────────────────
        hint_group = QGroupBox('MAVROS apm.launch 對應（複製到嵌入式 ROS workspace）')
        hint_layout = QVBoxLayout(hint_group)
        hint_layout.setContentsMargins(8, 8, 8, 8)

        self.apm_hint = QTextEdit()
        self.apm_hint.setReadOnly(True)
        self.apm_hint.setFixedHeight(110)
        # QTextEdit 全域 QSS 已使用 FONT_MONO；無需 inline
        hint_layout.addWidget(self.apm_hint)

        root.addWidget(hint_group)

        # ── 按鈕列 ─────────────────────────────────────────────
        btn_row = QHBoxLayout()
        btn_row.addStretch()
        self.btn_cancel = QPushButton('取消')
        self.btn_cancel.clicked.connect(self.reject)
        btn_row.addWidget(self.btn_cancel)

        # 主動作按鈕：用 launch SVG icon + orange tone（一鍵啟動 hero）
        self.btn_launch = IconButton('launch', '啟動', tone='orange')
        self.btn_launch.setDefault(True)
        self.btn_launch.clicked.connect(self._on_launch_clicked)
        btn_row.addWidget(self.btn_launch)
        root.addLayout(btn_row)

    def _build_row(self, i: int) -> None:
        """建立第 i 列的表格欄位（# / SYSID spinbox / Port spinbox）。"""
        # 第 0 欄：instance index（唯讀）
        idx_item = QTableWidgetItem(str(i))
        idx_item.setFlags(Qt.ItemFlag.ItemIsEnabled)
        idx_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.table.setItem(i, 0, idx_item)

        # 第 1 欄：SYSID spinbox（覆寫 MAV_SYSID，需與 MAVROS tgt_system 對齊）
        sysid_spin = QSpinBox()
        sysid_spin.setRange(1, 255)
        sysid_spin.setValue(i + 1)
        sysid_spin.valueChanged.connect(self._refresh_apm_hint)
        self.table.setCellWidget(i, 1, sysid_spin)

        # 第 2 欄：SITL 本地 UDP 監聽 port（預設 14550 + 10i）
        # SITL --serial1=udpin:0.0.0.0:<port>，嵌入式 MAVROS 主動送過來
        port_spin = QSpinBox()
        port_spin.setRange(1024, 65535)
        port_spin.setValue(14550 + 10 * i)
        port_spin.valueChanged.connect(self._refresh_apm_hint)
        self.table.setCellWidget(i, 2, port_spin)

    # ──────────────────────────────────────────────────────────────
    # 預設值載入：把 settings.json 內的 sitl_launch 套回 UI
    # ──────────────────────────────────────────────────────────────
    def _populate_from_saved(self) -> None:
        s = self._saved
        self.chk_enable_fanout.setChecked(bool(s.enable_fanout))
        self.chk_auto_firewall.setChecked(bool(s.auto_firewall))
        self.chk_save_default.setChecked(True)  # 預設勾選，避免每次都要手動勾

        # 把上次儲存的每列設定塞回表格（只覆蓋上次儲存過的列數）
        # 注意：embedded_ip 欄位向下相容保留於 settings.json 但不再寫回 UI
        saved_instances = s.instances or []
        for i in range(min(self._count, len(saved_instances))):
            cfg = saved_instances[i] or {}
            sysid = cfg.get('sysid')
            port = cfg.get('embedded_port')
            if sysid is not None:
                w = self.table.cellWidget(i, 1)
                if isinstance(w, QSpinBox):
                    w.setValue(int(sysid))
            if port is not None:
                w = self.table.cellWidget(i, 2)
                if isinstance(w, QSpinBox):
                    w.setValue(int(port))

        # 同步啟用狀態（fanout 沒勾 → 表格 disabled）
        self._on_fanout_toggled(self.chk_enable_fanout.isChecked())

    # ──────────────────────────────────────────────────────────────
    # Signal handlers
    # ──────────────────────────────────────────────────────────────
    def _on_fanout_toggled(self, checked: bool) -> None:
        """fanout 沒勾選時 disable 整個表格與防火牆選項，讓使用者只能跑本機 SITL。"""
        self.table.setEnabled(checked)
        self.chk_auto_firewall.setEnabled(checked)
        self._refresh_apm_hint()

    def _refresh_apm_hint(self) -> None:
        """根據當前表格內容組出 apm.launch 範例片段，給使用者直接複製。"""
        if not self.chk_enable_fanout.isChecked():
            self.apm_hint.setPlainText(
                '<!-- 已停用嵌入式 fan-out，純本機 SITL。-->\n'
                '<!-- 若要讓嵌入式接 SITL，請勾選上方「啟用嵌入式 fan-out」。-->'
            )
            return

        # MAVROS 主動 send 模式 — apm.launch 用：
        #   fcu_url:=udp://:<local_port>@<windows_lan_ip>:<sitl_port>
        # MAVROS bind local_port（建議 14555+10i），主動送到 Windows:<sitl_port>
        # SITL 端 --serial1=udpin:0.0.0.0:<sitl_port> 被動接收
        win_ip = self._lan_ip
        lines: List[str] = [
            '<!-- 嵌入式 ROS workspace：roslaunch <pkg> apm.launch ... -->',
            '<!-- Windows SITL = 被動 udpin listen；MAVROS = 主動 send -->',
        ]
        for i in range(self.table.rowCount()):
            sysid_w = self.table.cellWidget(i, 1)
            port_w = self.table.cellWidget(i, 2)
            if not isinstance(sysid_w, QSpinBox) or not isinstance(port_w, QSpinBox):
                continue
            sitl_port = port_w.value()
            sysid = sysid_w.value()
            local_port = 14555 + 10 * i  # MAVROS 本地 bind port（任意未占用即可）
            lines.append(
                f'<!-- 第 {i} 台 (sysid={sysid}) → -->'
                f' fcu_url:=udp://:{local_port}@{win_ip}:{sitl_port}'
                f' tgt_system:={sysid}'
            )
        self.apm_hint.setPlainText('\n'.join(lines))

    # ──────────────────────────────────────────────────────────────
    # 啟動按鈕：驗證 → 收集結果 → （可選）持久化 → accept
    # ──────────────────────────────────────────────────────────────
    def _on_launch_clicked(self) -> None:
        # SITL 改為被動 udpin 後，不再需要驗證嵌入式 IP；port 由 QSpinBox
        # 限制範圍即可。port 衝突檢查（同一機器多個 instance 撞 port）保留：
        if self.chk_enable_fanout.isChecked():
            seen_ports: dict = {}
            for i in range(self.table.rowCount()):
                port_w = self.table.cellWidget(i, 2)
                if not isinstance(port_w, QSpinBox):
                    continue
                p = port_w.value()
                if p in seen_ports:
                    QMessageBox.warning(
                        self, 'Port 重複',
                        f'第 {i} 列與第 {seen_ports[p]} 列的 SITL UDP port '
                        f'都是 {p}，請改用不同 port（建議 14550 / 14560 / 14570 ...）。',
                    )
                    port_w.setFocus()
                    return
                seen_ports[p] = i

        # 收集結果
        result = self.collect_result()

        # 持久化：勾選「儲存為預設」就寫進 settings.json
        if result.get('save_as_default'):
            try:
                gs = get_settings()
                gs.sitl_launch.enable_fanout = result['enable_fanout']
                gs.sitl_launch.auto_firewall = result['auto_firewall']
                gs.sitl_launch.instances = list(result['instances'])  # 深拷貝避免共用
                ok = gs.save()
                if ok:
                    logger.info('[SITL Dialog] 已儲存 SITL 啟動設定為下次預設')
                else:
                    logger.warning('[SITL Dialog] 儲存設定失敗（save() 回傳 False）')
            except Exception as e:
                logger.warning(f'[SITL Dialog] 持久化例外: {e}')

        self._result = result
        self.accept()

    def collect_result(self) -> dict:
        """從目前 UI 狀態組出回傳 dict。
        instances 內 `embedded_ip` 欄位保留空字串以維持 schema 向下相容
        （sitl_launcher 不再讀此欄；舊版若還在用則收到空字串會 fallback）。
        """
        instances: List[dict] = []
        for i in range(self.table.rowCount()):
            sysid_w = self.table.cellWidget(i, 1)
            port_w = self.table.cellWidget(i, 2)
            if (not isinstance(sysid_w, QSpinBox)
                    or not isinstance(port_w, QSpinBox)):
                continue
            instances.append({
                'sysid': int(sysid_w.value()),
                'embedded_ip': '',  # deprecated；新版 SITL 為 udpin 被動模式
                'embedded_port': int(port_w.value()),
            })
        return {
            'enable_fanout': bool(self.chk_enable_fanout.isChecked()),
            'auto_firewall': bool(self.chk_auto_firewall.isChecked()),
            'save_as_default': bool(self.chk_save_default.isChecked()),
            'instances': instances,
        }

    # ──────────────────────────────────────────────────────────────
    # Convenience static entry
    # ──────────────────────────────────────────────────────────────
    @staticmethod
    def get_config(vehicle: str,
                   count: int,
                   parent: Optional[QWidget] = None) -> Optional[dict]:
        """便利方法：執行 modal 並回傳結果 dict，使用者取消則回 None。"""
        dlg = SITLLaunchDialog(vehicle=vehicle, count=count, parent=parent)
        if dlg.exec() == QDialog.DialogCode.Accepted:
            return getattr(dlg, '_result', dlg.collect_result())
        return None
