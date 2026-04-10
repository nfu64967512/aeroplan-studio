"""
禁航區管理對話框
顯示目前所有 NFZ，提供刪除、清除功能。
新增 NFZ 請直接在地圖上繪製（主視窗的「繪製多邊形」/「拖曳圓形」按鈕）。
"""

from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QGroupBox,
    QListWidget, QListWidgetItem, QMessageBox,
)
from PyQt6.QtCore import pyqtSignal

from utils.logger import get_logger

logger = get_logger()


class NFZManagerDialog(QDialog):
    """
    禁航區管理對話框

    顯示目前禁航區清單，支援刪除/清除。
    在關閉時透過 nfz_changed 信號通知主視窗更新：
        [{'type': 'polygon', 'vertices': [(lat, lon), ...], 'name': str}, ...]
        [{'type': 'circle',  'center':   (lat, lon),       'radius': float, 'name': str}, ...]
    """

    nfz_changed = pyqtSignal(list)

    def __init__(self, parent=None, nfz_zones=None):
        super().__init__(parent)
        self.setWindowTitle("禁航區管理 (NFZ)")
        self.setMinimumSize(480, 380)

        self.nfz_zones = list(nfz_zones) if nfz_zones else []

        self._init_ui()
        self._refresh_list()
        logger.info("禁航區管理對話框初始化完成")

    def _init_ui(self):
        root = QVBoxLayout(self)
        root.setSpacing(8)

        hint = QLabel(
            "💡 若要新增禁航區，請關閉此視窗後\n"
            "使用主介面的「✏️ 繪製多邊形」或「⭕ 拖曳圓形」按鈕直接在地圖上畫。"
        )
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #FF9800; font-size: 11px; padding: 6px; "
                           "background: #FFF8E1; border-radius: 4px;")
        root.addWidget(hint)

        list_group = QGroupBox("目前禁航區清單")
        list_layout = QVBoxLayout(list_group)
        self.nfz_list_widget = QListWidget()
        self.nfz_list_widget.setMinimumHeight(180)
        list_layout.addWidget(self.nfz_list_widget)

        btn_row = QHBoxLayout()
        self.del_btn = QPushButton("刪除選取")
        self.del_btn.setStyleSheet(
            "background-color: #E53935; color: white; padding: 5px; border-radius: 3px;"
        )
        self.del_btn.clicked.connect(self._on_delete)
        self.clear_btn = QPushButton("清除全部")
        self.clear_btn.setStyleSheet(
            "background-color: #78909C; color: white; padding: 5px; border-radius: 3px;"
        )
        self.clear_btn.clicked.connect(self._on_clear_all)
        btn_row.addWidget(self.del_btn)
        btn_row.addWidget(self.clear_btn)
        list_layout.addLayout(btn_row)
        root.addWidget(list_group)

        footer = QHBoxLayout()
        ok_btn = QPushButton("確定")
        ok_btn.setStyleSheet(
            "background-color: #1976D2; color: white; padding: 6px; border-radius: 3px;"
        )
        ok_btn.clicked.connect(self._on_ok)
        cancel_btn = QPushButton("取消")
        cancel_btn.clicked.connect(self.reject)
        footer.addStretch()
        footer.addWidget(ok_btn)
        footer.addWidget(cancel_btn)
        root.addLayout(footer)

    def _on_delete(self):
        row = self.nfz_list_widget.currentRow()
        if row < 0:
            return
        name = self.nfz_zones[row]['name']
        reply = QMessageBox.question(
            self, "確認刪除", f"刪除禁航區「{name}」？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self.nfz_zones.pop(row)
            self._refresh_list()

    def _on_clear_all(self):
        if not self.nfz_zones:
            return
        reply = QMessageBox.question(
            self, "清除全部", "確定清除所有禁航區？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self.nfz_zones.clear()
            self._refresh_list()

    def _on_ok(self):
        self.nfz_changed.emit(list(self.nfz_zones))
        self.accept()

    def _refresh_list(self):
        self.nfz_list_widget.clear()
        for zone in self.nfz_zones:
            if zone['type'] == 'polygon':
                text = f"🟥 多邊形  {zone['name']}  ({len(zone['vertices'])} 頂點)"
            else:
                lat, lon = zone['center']
                text = (
                    f"🔴 圓形  {zone['name']}  "
                    f"圓心({lat:.5f},{lon:.5f})  r={zone['radius']:.0f}m"
                )
            self.nfz_list_widget.addItem(QListWidgetItem(text))

        count = len(self.nfz_zones)
        self.del_btn.setEnabled(count > 0)
        self.clear_btn.setEnabled(count > 0)
