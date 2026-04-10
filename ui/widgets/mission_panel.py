"""
任務面板模組
提供任務預覽、匯出、清除等操作界面
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QPushButton, QLabel, QProgressBar, QFrame, QGridLayout
)
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QFont, QColor

from utils.logger import get_logger

# 獲取日誌實例
logger = get_logger()


class MissionPanel(QWidget):
    """
    任務面板
    
    提供任務相關操作的控制界面
    """
    
    # 信號定義
    preview_requested = pyqtSignal()  # 預覽請求
    export_requested = pyqtSignal()   # 匯出請求
    clear_requested = pyqtSignal()    # 清除請求
    
    def __init__(self, parent=None):
        """初始化任務面板"""
        super().__init__(parent)
        
        # 初始化變數
        self.mission_stats = {}
        
        # 建立 UI
        self.init_ui()
        
        logger.info("任務面板初始化完成")
    
    def init_ui(self):
        """初始化 UI"""
        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        
        # 操作按鈕群組
        operation_group = self.create_operation_buttons()
        layout.addWidget(operation_group)
        
        # 任務資訊群組
        info_group = self.create_mission_info()
        layout.addWidget(info_group)
        
        # 添加彈性空間
        layout.addStretch()
    
    def create_operation_buttons(self):
        """創建操作按鈕群組"""
        group = QGroupBox("任務操作")
        layout = QVBoxLayout(group)
        layout.setSpacing(5)
        
        # 預覽路徑按鈕
        self.preview_btn = QPushButton("👁 預覽路徑")
        self.preview_btn.setMinimumHeight(40)
        self.preview_btn.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                border-radius: 5px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:pressed {
                background-color: #0D47A1;
            }
        """)
        self.preview_btn.clicked.connect(self.on_preview_clicked)
        layout.addWidget(self.preview_btn)
        
        # 匯出航點按鈕
        self.export_btn = QPushButton("📤 匯出航點")
        self.export_btn.setMinimumHeight(40)
        self.export_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                border-radius: 5px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #388E3C;
            }
            QPushButton:pressed {
                background-color: #1B5E20;
            }
        """)
        self.export_btn.clicked.connect(self.on_export_clicked)
        self.export_btn.setEnabled(False)  # 預設停用
        layout.addWidget(self.export_btn)
        
        # 分隔線
        layout.addSpacing(10)
        
        # 清除操作按鈕組
        clear_layout = QHBoxLayout()
        
        self.clear_paths_btn = QPushButton("清除路徑")
        self.clear_paths_btn.clicked.connect(lambda: self.on_clear_clicked('paths'))
        clear_layout.addWidget(self.clear_paths_btn)
        
        self.clear_corners_btn = QPushButton("清除邊界")
        self.clear_corners_btn.clicked.connect(lambda: self.on_clear_clicked('corners'))
        clear_layout.addWidget(self.clear_corners_btn)
        
        layout.addLayout(clear_layout)
        
        # 清除全部按鈕
        self.clear_all_btn = QPushButton("🗑 清除全部")
        self.clear_all_btn.setStyleSheet("""
            QPushButton {
                background-color: #FF5722;
                color: white;
                border: none;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #E64A19;
            }
        """)
        self.clear_all_btn.clicked.connect(lambda: self.on_clear_clicked('all'))
        layout.addWidget(self.clear_all_btn)
        
        return group
    
    def create_mission_info(self):
        """創建任務資訊群組（卡片式設計）"""
        group = QGroupBox("任務資訊")
        layout = QVBoxLayout(group)
        layout.setSpacing(8)
        layout.setContentsMargins(8, 10, 8, 10)

        # ── 空白提示卡（無資料時顯示）──────────────────────────────
        self._empty_card = QFrame()
        self._empty_card.setStyleSheet("""
            QFrame {
                background: rgba(33, 150, 243, 0.06);
                border: 1px dashed #90CAF9;
                border-radius: 8px;
            }
        """)
        empty_inner = QVBoxLayout(self._empty_card)
        empty_inner.setContentsMargins(12, 16, 12, 16)
        empty_inner.setSpacing(5)

        empty_icon = QLabel("🗺️")
        empty_icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
        empty_icon.setStyleSheet(
            "font-size: 30px; border: none; background: transparent;"
        )
        empty_inner.addWidget(empty_icon)

        empty_title = QLabel("尚無任務資訊")
        empty_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        empty_title.setStyleSheet(
            "font-weight: bold; color: #1976D2; font-size: 13px;"
            " border: none; background: transparent;"
        )
        empty_inner.addWidget(empty_title)

        for step in [
            "① 在地圖上點擊設置邊界點",
            "② 調整右側飛行參數",
            "③ 點擊「預覽路徑」生成任務",
            "④ 點擊「匯出航點」儲存檔案",
        ]:
            lbl = QLabel(step)
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setStyleSheet(
                "color: #607D8B; font-size: 11px;"
                " border: none; background: transparent;"
            )
            empty_inner.addWidget(lbl)

        layout.addWidget(self._empty_card)

        # ── 統計卡片容器（有資料時顯示）────────────────────────────
        self._stats_card = QWidget()
        self._stats_card.setVisible(False)
        stats_outer = QVBoxLayout(self._stats_card)
        stats_outer.setContentsMargins(0, 0, 0, 0)
        stats_outer.setSpacing(6)

        # 標題列
        header = QLabel("📊  任務統計")
        header.setStyleSheet(
            "font-weight: bold; color: #37474F; font-size: 12px;"
        )
        stats_outer.addWidget(header)

        # 指標定義：(key, icon, 名稱, 單位, 主色)
        metrics = [
            ('waypoints', '📍', '航點數量', '個',   '#1E88E5'),
            ('distance',  '📏', '飛行距離', 'm',    '#43A047'),
            ('time',      '⏱️',  '預估時間', 'min',  '#FB8C00'),
            ('area',      '🗺️',  '測繪面積', 'm²',   '#8E24AA'),
            ('regions',   '🔲', '子區域數', '個',   '#00ACC1'),
        ]

        self._metric_val_labels: dict = {}
        grid = QGridLayout()
        grid.setSpacing(6)

        for i, (key, icon, name, unit, color) in enumerate(metrics):
            card = QFrame()
            card.setStyleSheet(f"""
                QFrame {{
                    background-color: {color}18;
                    border-left: 3px solid {color};
                    border-radius: 5px;
                }}
            """)
            card_h = QHBoxLayout(card)
            card_h.setContentsMargins(8, 7, 8, 7)
            card_h.setSpacing(7)

            icon_lbl = QLabel(icon)
            icon_lbl.setStyleSheet(
                "font-size: 17px; background: transparent; border: none;"
            )
            icon_lbl.setFixedWidth(24)
            icon_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            card_h.addWidget(icon_lbl)

            text_col = QVBoxLayout()
            text_col.setSpacing(1)

            name_lbl = QLabel(name)
            name_lbl.setStyleSheet(
                "color: #78909C; font-size: 10px;"
                " background: transparent; border: none;"
            )
            val_lbl = QLabel("—")
            val_lbl.setStyleSheet(
                f"color: {color}; font-weight: bold; font-size: 14px;"
                f" background: transparent; border: none;"
            )
            text_col.addWidget(name_lbl)
            text_col.addWidget(val_lbl)
            card_h.addLayout(text_col)
            card_h.addStretch()

            self._metric_val_labels[key] = (val_lbl, unit)

            # 最後一個若為奇數則橫跨兩欄
            if i == len(metrics) - 1 and len(metrics) % 2 == 1:
                grid.addWidget(card, i // 2, 0, 1, 2)
            else:
                grid.addWidget(card, i // 2, i % 2)

        stats_outer.addLayout(grid)
        layout.addWidget(self._stats_card)

        # ── 進度條 ────────────────────────────────────────────────
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        self.progress_bar.setTextVisible(True)
        self.progress_bar.setFixedHeight(16)
        self.progress_bar.setStyleSheet("""
            QProgressBar {
                border: none;
                border-radius: 4px;
                background: #E3F2FD;
                text-align: center;
                font-size: 10px;
                color: #1565C0;
            }
            QProgressBar::chunk {
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:0,
                    stop:0 #1976D2, stop:1 #42A5F5
                );
                border-radius: 4px;
            }
        """)
        layout.addWidget(self.progress_bar)

        # 初始化顯示
        self.update_info_display()

        return group
    
    def on_preview_clicked(self):
        """處理預覽按鈕點擊"""
        self.preview_requested.emit()
        logger.info("請求預覽路徑")
    
    def on_export_clicked(self):
        """處理匯出按鈕點擊"""
        self.export_requested.emit()
        logger.info("請求匯出航點")
    
    def on_clear_clicked(self, clear_type: str):
        """
        處理清除按鈕點擊
        
        參數:
            clear_type: 清除類型 ('paths', 'corners', 'all')
        """
        self.clear_requested.emit()
        logger.info(f"請求清除: {clear_type}")
    
    def update_mission_stats(self, stats: dict):
        """
        更新任務統計資訊
        
        參數:
            stats: 統計資訊字典
        """
        self.mission_stats = stats
        self.update_info_display()
        
        # 如果有統計資訊，啟用匯出按鈕
        if stats:
            self.export_btn.setEnabled(True)
        
        logger.debug(f"更新任務統計: {stats}")
    
    def update_info_display(self):
        """更新資訊顯示（切換空白卡 / 統計卡）"""
        if not hasattr(self, '_empty_card'):
            return  # UI 尚未建立

        if not self.mission_stats:
            self._empty_card.setVisible(True)
            self._stats_card.setVisible(False)
            return

        self._empty_card.setVisible(False)
        self._stats_card.setVisible(True)

        waypoint_count = self.mission_stats.get('waypoint_count', 0)
        total_distance = self.mission_stats.get('total_distance', 0.0)
        estimated_time = self.mission_stats.get('estimated_time', 0.0)
        area           = self.mission_stats.get('area', 0.0)
        regions        = self.mission_stats.get('regions', 1)

        values = {
            'waypoints': f"{waypoint_count}",
            'distance':  f"{total_distance:.1f}",
            'time':      f"{estimated_time / 60:.1f}",
            'area':      f"{area:.1f}",
            'regions':   f"{regions}",
        }
        for key, (lbl, unit) in self._metric_val_labels.items():
            lbl.setText(f"{values[key]} {unit}")
    
    def show_progress(self, visible: bool, value: int = 0, text: str = ""):
        """
        顯示/隱藏進度條
        
        參數:
            visible: 是否顯示
            value: 進度值 (0-100)
            text: 進度文字
        """
        self.progress_bar.setVisible(visible)
        
        if visible:
            self.progress_bar.setValue(value)
            if text:
                self.progress_bar.setFormat(text + " %p%")
        
        logger.debug(f"進度條: visible={visible}, value={value}, text={text}")
    
    def set_buttons_enabled(self, enabled: bool):
        """
        設置按鈕啟用狀態
        
        參數:
            enabled: 是否啟用
        """
        self.preview_btn.setEnabled(enabled)
        self.clear_paths_btn.setEnabled(enabled)
        self.clear_corners_btn.setEnabled(enabled)
        self.clear_all_btn.setEnabled(enabled)
        
        logger.debug(f"按鈕啟用狀態: {enabled}")
    
    def reset(self):
        """重置面板"""
        self.mission_stats = {}
        self.update_info_display()
        self.export_btn.setEnabled(False)
        self.progress_bar.setVisible(False)
        logger.info("任務面板已重置")
