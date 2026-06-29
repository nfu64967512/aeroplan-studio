"""
匯出對話框模組
提供任務匯出的友善 UI 界面
支援多種格式、批次匯出和進階選項
"""

from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QGroupBox, QFormLayout,
    QCheckBox, QLineEdit, QPushButton, QFileDialog,
    QLabel, QSpinBox, QTabWidget,
    QWidget, QRadioButton, QButtonGroup, QListWidget, QMessageBox
)
from PyQt6.QtCore import pyqtSignal
from typing import Optional, List
from pathlib import Path
import os

from mission.mission_manager import Mission
from mission.mavlink_exporter import MAVLinkExporter, ExportFormat
from ui.resources.aeroplan_theme.widgets import IconButton


class ExportDialog(QDialog):
    """
    匯出對話框
    
    提供任務匯出的完整選項配置：
    - 匯出格式選擇（QGC、KML、GPX、JSON）
    - 單一/批次匯出模式
    - 檔案命名規則
    - 進階選項（簡報、元數據、壓縮）
    """
    
    # 信號
    export_completed = pyqtSignal(list)  # 匯出完成，發送檔案路徑列表
    
    def __init__(self, mission: Mission = None, missions: List[Mission] = None, parent=None):
        """
        初始化匯出對話框
        
        參數:
            mission: 單一任務實例
            missions: 任務列表（批次匯出）
            parent: 父視窗
        """
        super().__init__(parent)
        
        self.mission = mission
        self.missions = missions or ([mission] if mission else [])
        self.is_batch_mode = len(self.missions) > 1
        
        self.exporter = MAVLinkExporter()
        self.output_files = []
        
        self.setWindowTitle("匯出任務" + (" (批次模式)" if self.is_batch_mode else ""))
        self.setMinimumWidth(600)
        self.setMinimumHeight(500)
        
        self.setup_ui()
        self.load_default_settings()
    
    def setup_ui(self):
        """建立 UI"""
        layout = QVBoxLayout()
        
        # 分頁控制
        self.tab_widget = QTabWidget()
        
        # 分頁 1: 基本設定
        self.tab_widget.addTab(self.create_basic_tab(), "基本設定")
        
        # 分頁 2: 格式選項
        self.tab_widget.addTab(self.create_format_tab(), "格式選項")
        
        # 分頁 3: 進階選項
        self.tab_widget.addTab(self.create_advanced_tab(), "進階選項")
        
        layout.addWidget(self.tab_widget)
        
        # 預覽區域
        preview_group = self.create_preview_group()
        layout.addWidget(preview_group)
        
        # 按鈕列
        button_layout = self.create_button_layout()
        layout.addLayout(button_layout)
        
        self.setLayout(layout)
    
    def create_basic_tab(self) -> QWidget:
        """創建基本設定分頁"""
        widget = QWidget()
        layout = QVBoxLayout()
        
        # 任務資訊
        info_group = QGroupBox("任務資訊")
        info_layout = QFormLayout()
        
        # 任務數量
        mission_count = len(self.missions)
        count_label = QLabel(f"<b>{mission_count}</b> 個任務")
        info_layout.addRow("任務數量:", count_label)
        
        # 顯示任務名稱（單一任務時）
        if not self.is_batch_mode and self.mission:
            name_label = QLabel(self.mission.name)
            info_layout.addRow("任務名稱:", name_label)
            
            # 統計資訊
            stats = self.mission.get_statistics()
            stats_text = (
                f"航點: {stats.get('waypoint_count', 0)} 個 | "
                f"距離: {stats.get('total_distance_m', 0):.1f} m | "
                f"時間: {stats.get('estimated_time_s', 0):.0f} s"
            )
            stats_label = QLabel(stats_text)
            stats_label.setProperty('role', 'caption')
            stats_label.style().polish(stats_label)
            info_layout.addRow("統計:", stats_label)
        
        info_group.setLayout(info_layout)
        layout.addWidget(info_group)
        
        # 輸出設定
        output_group = QGroupBox("輸出設定")
        output_layout = QFormLayout()
        
        # 輸出目錄
        dir_layout = QHBoxLayout()
        self.output_dir_edit = QLineEdit()
        self.output_dir_edit.setPlaceholderText("選擇輸出目錄...")
        browse_btn = QPushButton("瀏覽...")
        browse_btn.clicked.connect(self.browse_output_directory)
        dir_layout.addWidget(self.output_dir_edit)
        dir_layout.addWidget(browse_btn)
        output_layout.addRow("輸出目錄:", dir_layout)
        
        # 檔案命名
        self.filename_pattern_edit = QLineEdit()
        self.filename_pattern_edit.setPlaceholderText("{mission_name}_{timestamp}")
        self.filename_pattern_edit.textChanged.connect(self.update_preview)
        output_layout.addRow("命名模式:", self.filename_pattern_edit)
        
        # 提示
        hint_label = QLabel(
            "可用變數: {mission_name}, {timestamp}, {date}, {index}\n"
            "範例: mission_{date}_{index}"
        )
        hint_label.setProperty('role', 'caption')
        hint_label.style().polish(hint_label)
        hint_label.setWordWrap(True)
        output_layout.addRow("", hint_label)
        
        output_group.setLayout(output_layout)
        layout.addWidget(output_group)
        
        # 匯出模式（僅批次時顯示）
        if self.is_batch_mode:
            mode_group = QGroupBox("匯出模式")
            mode_layout = QVBoxLayout()
            
            self.mode_group = QButtonGroup()
            
            self.separate_files_radio = QRadioButton("分別匯出（每個任務獨立檔案）")
            self.separate_files_radio.setChecked(True)
            self.mode_group.addButton(self.separate_files_radio)
            mode_layout.addWidget(self.separate_files_radio)
            
            self.combined_file_radio = QRadioButton("合併匯出（所有任務合併為一個檔案）")
            self.mode_group.addButton(self.combined_file_radio)
            mode_layout.addWidget(self.combined_file_radio)
            
            mode_group.setLayout(mode_layout)
            layout.addWidget(mode_group)
        
        layout.addStretch()
        widget.setLayout(layout)
        return widget
    
    def create_format_tab(self) -> QWidget:
        """創建格式選項分頁"""
        widget = QWidget()
        layout = QVBoxLayout()
        
        # 格式選擇
        format_group = QGroupBox("匯出格式")
        format_layout = QVBoxLayout()
        
        # 格式選項
        self.format_checkboxes = {}
        
        formats = [
            (ExportFormat.QGC_WPL, "QGC WPL 110", 
             "QGroundControl / Mission Planner 標準格式", True),
            (ExportFormat.JSON, "JSON", 
             "JSON 格式，包含完整任務資料", False),
            (ExportFormat.KML, "KML", 
             "Google Earth 格式，可視化顯示", False),
            (ExportFormat.GPX, "GPX", 
             "GPS 軌跡格式，通用 GPS 裝置", False),
        ]
        
        for fmt, name, desc, default in formats:
            cb = QCheckBox(f"{name} (.{self.exporter._get_extension_for_format(fmt)[1:]})")
            cb.setChecked(default)
            cb.toggled.connect(self.update_preview)
            
            # 說明文字
            desc_label = QLabel(f"  └─ {desc}")
            desc_label.setProperty('role', 'caption')
            desc_label.style().polish(desc_label)
            
            format_layout.addWidget(cb)
            format_layout.addWidget(desc_label)
            
            self.format_checkboxes[fmt] = cb
        
        format_group.setLayout(format_layout)
        layout.addWidget(format_group)
        
        # 格式特定選項
        options_group = QGroupBox("格式選項")
        options_layout = QFormLayout()
        
        # JSON 選項
        self.json_include_metadata_cb = QCheckBox("包含元數據")
        self.json_include_metadata_cb.setChecked(True)
        options_layout.addRow("JSON:", self.json_include_metadata_cb)
        
        # KML 選項
        kml_layout = QHBoxLayout()
        self.kml_name_edit = QLineEdit()
        self.kml_name_edit.setPlaceholderText("顯示名稱（預設為任務名稱）")
        kml_layout.addWidget(self.kml_name_edit)
        options_layout.addRow("KML:", kml_layout)
        
        options_group.setLayout(options_layout)
        layout.addWidget(options_group)
        
        layout.addStretch()
        widget.setLayout(layout)
        return widget
    
    def create_advanced_tab(self) -> QWidget:
        """創建進階選項分頁"""
        widget = QWidget()
        layout = QVBoxLayout()
        
        # 附加檔案
        additional_group = QGroupBox("附加檔案")
        additional_layout = QVBoxLayout()
        
        self.export_briefing_cb = QCheckBox("匯出任務簡報 (.txt)")
        self.export_briefing_cb.setChecked(True)
        self.export_briefing_cb.setToolTip("生成包含任務詳細資訊的文字檔案")
        additional_layout.addWidget(self.export_briefing_cb)
        
        self.export_statistics_cb = QCheckBox("匯出統計報告 (.csv)")
        self.export_statistics_cb.setToolTip("生成包含任務統計數據的 CSV 檔案")
        additional_layout.addWidget(self.export_statistics_cb)
        
        additional_group.setLayout(additional_layout)
        layout.addWidget(additional_group)
        
        # 壓縮選項
        compression_group = QGroupBox("壓縮選項")
        compression_layout = QVBoxLayout()
        
        self.compress_output_cb = QCheckBox("壓縮輸出檔案為 .zip")
        self.compress_output_cb.setToolTip("將所有匯出檔案打包為 ZIP 壓縮檔")
        compression_layout.addWidget(self.compress_output_cb)
        
        compression_group.setLayout(compression_layout)
        layout.addWidget(compression_group)
        
        # 座標精度
        precision_group = QGroupBox("座標精度")
        precision_layout = QFormLayout()
        
        self.coord_precision_spin = QSpinBox()
        self.coord_precision_spin.setRange(4, 10)
        self.coord_precision_spin.setValue(6)
        self.coord_precision_spin.setSuffix(" 位小數")
        precision_layout.addRow("經緯度:", self.coord_precision_spin)
        
        self.alt_precision_spin = QSpinBox()
        self.alt_precision_spin.setRange(0, 4)
        self.alt_precision_spin.setValue(2)
        self.alt_precision_spin.setSuffix(" 位小數")
        precision_layout.addRow("高度:", self.alt_precision_spin)
        
        precision_group.setLayout(precision_layout)
        layout.addWidget(precision_group)
        
        # 匯出後動作
        post_export_group = QGroupBox("匯出後動作")
        post_export_layout = QVBoxLayout()
        
        self.open_folder_cb = QCheckBox("開啟輸出資料夾")
        self.open_folder_cb.setChecked(True)
        post_export_layout.addWidget(self.open_folder_cb)
        
        self.show_summary_cb = QCheckBox("顯示匯出摘要")
        self.show_summary_cb.setChecked(True)
        post_export_layout.addWidget(self.show_summary_cb)
        
        post_export_group.setLayout(post_export_layout)
        layout.addWidget(post_export_group)
        
        layout.addStretch()
        widget.setLayout(layout)
        return widget
    
    def create_preview_group(self) -> QGroupBox:
        """創建預覽群組"""
        group = QGroupBox("檔案預覽")
        layout = QVBoxLayout()
        
        # 提示文字
        hint = QLabel("將生成以下檔案：")
        hint.setProperty('role', 'emphasis')
        hint.style().polish(hint)
        layout.addWidget(hint)

        # 檔案列表（QListWidget 走全域 QSS 預設樣式，與 dark theme 統一）
        self.preview_list = QListWidget()
        self.preview_list.setMaximumHeight(120)
        layout.addWidget(self.preview_list)
        
        group.setLayout(layout)
        return group
    
    def create_button_layout(self) -> QHBoxLayout:
        """創建按鈕列"""
        layout = QHBoxLayout()
        
        # 左側資訊
        self.status_label = QLabel("")
        self.status_label.setProperty('role', 'caption')
        self.status_label.style().polish(self.status_label)
        layout.addWidget(self.status_label)

        layout.addStretch()

        # 匯出按鈕（IconButton 走 success tone，hover/disabled 由全域 QSS 處理）
        self.export_btn = IconButton('export', '匯出', tone='success')
        self.export_btn.setMinimumWidth(100)
        self.export_btn.clicked.connect(self.do_export)
        layout.addWidget(self.export_btn)
        
        # 取消按鈕
        cancel_btn = QPushButton("取消")
        cancel_btn.setMinimumWidth(80)
        cancel_btn.clicked.connect(self.reject)
        layout.addWidget(cancel_btn)
        
        return layout
    
    def load_default_settings(self):
        """載入預設設定"""
        # 設定預設輸出目錄
        from pathlib import Path
        default_dir = str(Path.home() / "Documents" / "UAV_Missions")
        self.output_dir_edit.setText(default_dir)
        
        # 設定預設命名模式
        self.filename_pattern_edit.setText("{mission_name}_{timestamp}")
        
        # 更新預覽
        self.update_preview()
    
    def browse_output_directory(self):
        """瀏覽輸出目錄"""
        directory = QFileDialog.getExistingDirectory(
            self,
            "選擇輸出目錄",
            self.output_dir_edit.text() or str(Path.home())
        )
        
        if directory:
            self.output_dir_edit.setText(directory)
            self.update_preview()
    
    def update_preview(self):
        """更新檔案預覽"""
        self.preview_list.clear()
        
        if not self.missions:
            return
        
        # 獲取選中的格式
        selected_formats = [
            fmt for fmt, cb in self.format_checkboxes.items() if cb.isChecked()
        ]
        
        if not selected_formats:
            self.preview_list.addItem("⚠ 請至少選擇一種匯出格式")
            self.export_btn.setEnabled(False)
            return
        
        self.export_btn.setEnabled(True)
        
        # 生成檔案列表
        from datetime import datetime
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        file_count = 0
        
        for idx, mission in enumerate(self.missions, 1):
            # 替換命名模式
            pattern = self.filename_pattern_edit.text() or "{mission_name}"
            filename_base = pattern.replace("{mission_name}", mission.name)
            filename_base = filename_base.replace("{timestamp}", timestamp)
            filename_base = filename_base.replace("{date}", datetime.now().strftime('%Y%m%d'))
            filename_base = filename_base.replace("{index}", str(idx))
            
            # 為每種格式生成檔名
            for fmt in selected_formats:
                ext = self.exporter._get_extension_for_format(fmt)
                filename = f"{filename_base}{ext}"
                self.preview_list.addItem(f"📄 {filename}")
                file_count += 1
            
            # 簡報檔案
            if self.export_briefing_cb.isChecked():
                briefing_file = f"{filename_base}_briefing.txt"
                self.preview_list.addItem(f"📋 {briefing_file}")
                file_count += 1
            
            # 統計檔案
            if self.export_statistics_cb.isChecked():
                stats_file = f"{filename_base}_statistics.csv"
                self.preview_list.addItem(f"📊 {stats_file}")
                file_count += 1
        
        # 壓縮檔案
        if self.compress_output_cb.isChecked():
            self.preview_list.addItem("")
            self.preview_list.addItem(f"📦 missions_export_{timestamp}.zip (包含上述所有檔案)")
        
        # 更新狀態
        self.status_label.setText(f"將生成 {file_count} 個檔案")
    
    def do_export(self):
        """執行匯出"""
        # 驗證輸出目錄
        output_dir = self.output_dir_edit.text()
        if not output_dir:
            QMessageBox.warning(self, "警告", "請選擇輸出目錄")
            return
        
        # 創建目錄
        try:
            os.makedirs(output_dir, exist_ok=True)
        except Exception as e:
            QMessageBox.critical(self, "錯誤", f"無法創建輸出目錄：{e}")
            return
        
        # 獲取選中的格式
        selected_formats = [
            fmt for fmt, cb in self.format_checkboxes.items() if cb.isChecked()
        ]
        
        if not selected_formats:
            QMessageBox.warning(self, "警告", "請至少選擇一種匯出格式")
            return
        
        # 開始匯出
        self.export_btn.setEnabled(False)
        self.export_btn.setText("匯出中...")
        self.status_label.setText("正在匯出...")
        
        try:
            from datetime import datetime
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            
            self.output_files = []
            
            for idx, mission in enumerate(self.missions, 1):
                # 生成檔名
                pattern = self.filename_pattern_edit.text() or "{mission_name}"
                filename_base = pattern.replace("{mission_name}", mission.name)
                filename_base = filename_base.replace("{timestamp}", timestamp)
                filename_base = filename_base.replace("{date}", datetime.now().strftime('%Y%m%d'))
                filename_base = filename_base.replace("{index}", str(idx))
                
                # 匯出各種格式
                for fmt in selected_formats:
                    ext = self.exporter._get_extension_for_format(fmt)
                    filepath = os.path.join(output_dir, f"{filename_base}{ext}")
                    
                    # 格式特定選項
                    kwargs = {}
                    if fmt == ExportFormat.JSON:
                        kwargs['include_metadata'] = self.json_include_metadata_cb.isChecked()
                    elif fmt == ExportFormat.KML:
                        kml_name = self.kml_name_edit.text()
                        if kml_name:
                            kwargs['name'] = kml_name
                    
                    if self.exporter.export_mission(mission, filepath, fmt, **kwargs):
                        self.output_files.append(filepath)
                
                # 匯出簡報
                if self.export_briefing_cb.isChecked():
                    briefing_path = os.path.join(output_dir, f"{filename_base}_briefing.txt")
                    if self.exporter.export_mission_briefing(mission, briefing_path):
                        self.output_files.append(briefing_path)
                
                # 匯出統計（TODO: 實現統計匯出）
                if self.export_statistics_cb.isChecked():
                    stats_path = os.path.join(output_dir, f"{filename_base}_statistics.csv")
                    self._export_statistics(mission, stats_path)
                    self.output_files.append(stats_path)
            
            # 壓縮（如果需要）
            if self.compress_output_cb.isChecked():
                zip_path = self._compress_files(output_dir, timestamp)
                if zip_path:
                    self.output_files = [zip_path]  # 只保留 ZIP 檔案
            
            # 完成
            self.status_label.setText(f"✓ 成功匯出 {len(self.output_files)} 個檔案")
            
            # 匯出後動作
            if self.open_folder_cb.isChecked():
                self._open_folder(output_dir)
            
            if self.show_summary_cb.isChecked():
                self._show_summary()
            
            # 發送信號
            self.export_completed.emit(self.output_files)
            
            # 關閉對話框
            self.accept()
            
        except Exception as e:
            QMessageBox.critical(self, "匯出失敗", f"匯出過程中發生錯誤：{e}")
            self.status_label.setText("✗ 匯出失敗")
        finally:
            self.export_btn.setEnabled(True)
            self.export_btn.setText("匯出")
    
    def _export_statistics(self, mission: Mission, filepath: str):
        """匯出統計資料為 CSV"""
        try:
            stats = mission.get_statistics()
            
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write("統計項目,數值,單位\n")
                f.write(f"任務名稱,{mission.name},\n")
                f.write(f"航點數量,{stats.get('waypoint_count', 0)},個\n")
                f.write(f"總距離,{stats.get('total_distance_m', 0):.2f},公尺\n")
                f.write(f"預估時間,{stats.get('estimated_time_s', 0):.0f},秒\n")
                f.write(f"測繪面積,{stats.get('survey_area_m2', 0):.2f},平方公尺\n")
                f.write(f"平均高度,{stats.get('average_altitude_m', 0):.2f},公尺\n")
                f.write(f"最大高度,{stats.get('max_altitude_m', 0):.2f},公尺\n")
        except Exception as e:
            print(f"匯出統計失敗: {e}")
    
    def _compress_files(self, output_dir: str, timestamp: str) -> Optional[str]:
        """壓縮檔案"""
        try:
            import zipfile
            
            zip_path = os.path.join(output_dir, f"missions_export_{timestamp}.zip")
            
            with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
                for filepath in self.output_files:
                    arcname = os.path.basename(filepath)
                    zipf.write(filepath, arcname)
                    # 刪除原始檔案
                    os.remove(filepath)
            
            return zip_path
        except Exception as e:
            print(f"壓縮失敗: {e}")
            return None
    
    def _open_folder(self, folder_path: str):
        """開啟資料夾"""
        try:
            import subprocess
            import platform
            
            system = platform.system()
            if system == "Windows":
                os.startfile(folder_path)
            elif system == "Darwin":  # macOS
                subprocess.run(["open", folder_path])
            else:  # Linux
                subprocess.run(["xdg-open", folder_path])
        except Exception as e:
            print(f"開啟資料夾失敗: {e}")
    
    def _show_summary(self):
        """顯示匯出摘要"""
        summary = "匯出完成！\n\n"
        summary += f"共匯出 {len(self.output_files)} 個檔案：\n\n"
        
        for filepath in self.output_files:
            filename = os.path.basename(filepath)
            size = os.path.getsize(filepath)
            summary += f"• {filename} ({size:,} bytes)\n"
        
        QMessageBox.information(self, "匯出成功", summary)


# ==========================================
# 使用範例
# ==========================================
if __name__ == '__main__':
    from PyQt6.QtWidgets import QApplication
    import sys
    
    app = QApplication(sys.argv)
    
    # 創建測試任務
    from mission.mission_manager import MissionManager
    manager = MissionManager()
    mission = manager.create_mission("Test Mission")
    
    # 顯示對話框
    dialog = ExportDialog(mission=mission)
    
    if dialog.exec() == QDialog.DialogCode.Accepted:
        print(f"匯出的檔案: {dialog.output_files}")
    
    sys.exit(app.exec())
