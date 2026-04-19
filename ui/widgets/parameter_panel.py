"""
參數面板模組
提供飛行參數、測繪參數的設置界面
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QSlider, QSpinBox, QDoubleSpinBox,
    QComboBox, QCheckBox, QPushButton, QFormLayout, QFrame,
    QFileDialog, QTabWidget, QScrollArea, QSplitter, QSizePolicy
)
from PyQt6.QtCore import Qt, pyqtSignal

from config import get_settings
from utils.logger import get_logger

# 獲取配置和日誌實例
settings = get_settings()
logger = get_logger()


class ParameterPanel(QWidget):
    """
    參數面板
    
    提供各種飛行和測繪參數的設置
    """
    
    # 信號定義
    parameters_changed = pyqtSignal(dict)  # 參數變更信號
    corner_added = pyqtSignal(float, float)  # 新增邊界點信號
    clear_corners_requested = pyqtSignal()  # 清除邊界點信號
    remove_last_corner_requested = pyqtSignal()  # 刪除最後一點信號
    open_click_map_requested = pyqtSignal()  # 打開點擊地圖視窗
    pick_center_requested = pyqtSignal()    # 從地圖點選圓心
    drag_circle_requested = pyqtSignal()    # 地圖拖曳定義圓形區域
    pick_home_point_requested  = pyqtSignal()  # 從地圖點選起飛點
    clear_home_point_requested = pyqtSignal()  # 清除起飛點
    manage_nfz_requested       = pyqtSignal()  # 開啟禁航區管理對話框
    nfz_draw_polygon_requested = pyqtSignal()  # 在地圖繪製 NFZ 多邊形
    nfz_draw_circle_requested  = pyqtSignal()  # 在地圖拖曳定義 NFZ 圓形
    nfz_poly_finish_requested  = pyqtSignal()  # 完成 NFZ 多邊形繪製
    dccpp_coverage_requested = pyqtSignal(dict)    # 發送 DCCPP 最佳化參數
    dccpp_export_requested = pyqtSignal()             # 請求匯出 DCCPP 任務
    vtol_export_requested = pyqtSignal(dict)           # 請求匯出 VTOL 任務 (含 transition_alt 等參數)
    delete_last_corner_requested = pyqtSignal()       # 請求刪除最後一個邊界點
    dem_loaded = pyqtSignal(str)                       # DEM 檔案已選取，附路徑

    # ── 戰術模組信號 ──────────────────────────────────────────────────
    elevation_slicer_changed = pyqtSignal(float, float)  # (minAlt, maxAlt)
    elevation_slicer_cleared = pyqtSignal()               # 清除高程切片
    sar_heatmap_init_requested = pyqtSignal(dict)         # 初始化搜救熱力圖
    sar_heatmap_reset_requested = pyqtSignal()            # 重置熱力圖
    sar_heatmap_clear_requested = pyqtSignal()            # 清除熱力圖
    fov_cone_toggle_requested = pyqtSignal(bool)          # 開關 FOV 光錐
    radar_sim_requested = pyqtSignal(dict)                # 模擬雷達威脅掃描
    radar_clear_requested = pyqtSignal()                  # 清除雷達穹頂
    rcs_toggle_requested = pyqtSignal(bool)               # 開關 RCS 渲染

    # ── 蜂群打擊模組信號 ──────────────────────────────────────────────
    strike_mark_targets_requested = pyqtSignal()          # 開始標記打擊目標
    strike_mark_base_requested = pyqtSignal()             # STOT 模式：標記共用發射基地
    strike_mode_changed = pyqtSignal(str)                 # 發射模式切換 ('DTOT' / 'STOT')
    strike_execute_requested = pyqtSignal(dict)           # 執行蜂群打擊 (params dict)
    strike_clear_requested = pyqtSignal()                 # 清除打擊視覺化
    strike_export_requested = pyqtSignal()                # 匯出蜂群打擊 QGC WPL 航點
    strike_dtot_export_requested = pyqtSignal(dict)       # DTOT/STOT 飽和攻擊協同匯出
    strike_owa_parm_requested = pyqtSignal()              # 產生 OWA-UAV .parm 檔
    strike_sitl_upload_requested = pyqtSignal(bool)       # 上傳蜂群打擊任務至 SITL (bool = 是否使用 DTOT 空速)
    strike_recon_trigger_requested = pyqtSignal(dict)     # DCCPP → Strike 動態切換觸發
    strike_vtol_toggle_changed = pyqtSignal(bool)         # VTOL 模式開關切換

    def __init__(self, parent=None):
        """初始化參數面板"""
        super().__init__(parent)
        
        # 初始化參數字典
        self.parameters = {
            'altitude': 10.0,
            'speed': 3.0,
            'angle': 0.0,
            'spacing': 20.0,
            'yaw_speed': 60.0,
            'subdivisions': 1,
            'region_spacing': 3.0,
            'gap_spacings_m': [],        # 各邊界獨立間隔（條帶模式）
            'v_spacing_m': 3.0,          # 網格垂直間隔
            'transit_stagger_enabled': False,  # 轉場高度錯層
            'transit_stagger_alt': 10.0,       # 每台無人機高度錯層間距 (m)
            'reduce_overlap': True,
            'flight_mode': 'smart_collision',
            'algorithm': 'grid',
            'vehicle_type': '多旋翼',
            'vehicle_model': 'DJI Mavic 3',
            'turn_radius': 50.0,  # 固定翼轉彎半徑
            'lock_heading': False,
            'heading_angle': 0.0,
            # ── 固定翼專用參數 ──────────────────────────────────
            'fw_takeoff_bearing': 0.0,       # 起飛方向（度，0=北）
            'fw_runway_length': 50.0,        # 跑道長度（公尺）
            'fw_landing_bearing': 180.0,     # 降落進場方向（度）
            'fw_pattern_alt': 80.0,          # 五邊飛行高度（公尺）
            'fw_downwind_offset': 200.0,     # 下風邊側偏距離（公尺）
            'fw_pattern_leg': 300.0,         # 五邊腿長（公尺）
            'fw_final_dist': 400.0,          # 最終進場距離（公尺）
            'fw_scan_mode': 'spiral',        # 掃描模式: spiral / circular
            'fw_max_bank_deg': 45.0,         # 最大傾斜角 (ArduPlane LIM_ROLL_CD = 4500 cd)
            # ── 螺旋/同心圓圓心模式 ─────────────────────────────────────
            'circle_center_lat': 0.0,
            'circle_center_lon': 0.0,
            'circle_max_radius': 200.0,
            'circle_alt_step': 0.0,
            'circle_min_radius': 0.0,
            # ── 螺旋專用 ─────────────────────────────────────────────────
            'spiral_curvature': 1.0,         # 螺旋曲率倍數
            'spiral_alt_step': 0.0,          # 螺旋每圈高度步階 (m/rev)
            'grid_alt_step': 0.0,            # 網格每條掃描線高度步階 (m/line)
            # ── 起飛點（固定翼跑道位置）────────────────────────────────
            'home_point_lat': 0.0,
            'home_point_lon': 0.0,
            # ── AutoLand ─────────────────────────────────────────────────
            'fw_landing_rollout': 0.0,    # 觸地後滑行+煞車距離（公尺）
            'fw_autoland': False,          # True = 跳過五邊，直接 NAV_LAND
        }
        # 動態間隔 spinbox 列表（隨 n_regions 重建）
        self._gap_spinboxes: list = []
        
        # 建立 UI
        self.init_ui()
        
        logger.info("參數面板初始化完成")
    
    def init_ui(self):
        """初始化 UI"""
        root_layout = QVBoxLayout(self)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        self._tabs = QTabWidget()
        self._tabs.setDocumentMode(True)
        root_layout.addWidget(self._tabs)

        # 紀錄所有 splitter，供「重置版面」功能使用
        self._splitters: list[QSplitter] = []

        # ── Tab 1: 基本演算法 ──────────────────────────────────────────
        self.corner_group = self.create_corner_management()
        algo_vehicle_group = self.create_algorithm_vehicle_selection()
        flight_group = self.create_flight_parameters()
        self.fixed_wing_group = self.create_fixed_wing_panel()
        self.fixed_wing_group.setVisible(False)
        survey_group = self.create_survey_parameters()
        self.circle_center_group = self.create_circle_center_panel()
        self.circle_center_group.setVisible(False)
        self.region_safety_group = self.create_region_safety_group()
        self.region_safety_group.setVisible(False)
        advanced_group = self.create_advanced_parameters()

        basic_scroll = self._make_resizable_tab([
            self.corner_group, algo_vehicle_group, flight_group,
            self.fixed_wing_group, survey_group, self.circle_center_group,
            self.region_safety_group, advanced_group,
        ])
        self._tabs.addTab(basic_scroll, "基本演算法")

        # ── Tab 2: DCCPP ──────────────────────────────────────────────
        dccpp_group = self.create_dccpp_panel()
        dccpp_scroll = self._make_resizable_tab([dccpp_group])
        self._tabs.addTab(dccpp_scroll, "DCCPP")

        # ── Tab 4: 戰術模組 ──────────────────────────────────────────
        elev_group = self._create_elevation_slicer_panel()
        sar_group = self._create_sar_heatmap_panel()
        radar_group = self._create_radar_rcs_panel()
        tactical_scroll = self._make_resizable_tab(
            [elev_group, sar_group, radar_group]
        )
        self._tabs.addTab(tactical_scroll, "戰術模組")

        # ── Tab 5: 蜂群打擊 ──────────────────────────────────────────
        strike_group = self._create_strike_command_panel()
        strike_scroll = self._make_resizable_tab([strike_group])
        self._tabs.addTab(strike_scroll, "蜂群打擊")
    
    # ─────────────────────────────────────────────────────────────────
    #  可拖拉佈局 helpers — 讓每個 GroupBox 可由 Splitter 手動決定大小
    # ─────────────────────────────────────────────────────────────────
    _SPLITTER_QSS = """
        QSplitter::handle {
            background-color: #37474F;
            border: 1px solid #263238;
        }
        QSplitter::handle:horizontal {
            width: 6px;
        }
        QSplitter::handle:vertical {
            height: 6px;
        }
        QSplitter::handle:hover {
            background-color: #FFB74D;
        }
        QSplitter::handle:pressed {
            background-color: #FF9800;
        }
    """

    def _make_resizable_tab(self, widgets: list) -> QScrollArea:
        """將一組 QGroupBox widgets 包進 QSplitter(Vertical) → QScrollArea。

        - 每個 GroupBox 之間有可拖拉的分隔線（~6px，hover 時變橘色）
        - 隱藏狀態的 GroupBox 不會佔空間，切換顯示時自動分配空間
        - QScrollArea 仍保留，內容超過視窗高度時可捲動
        """
        splitter = QSplitter(Qt.Orientation.Vertical)
        splitter.setStyleSheet(self._SPLITTER_QSS)
        splitter.setChildrenCollapsible(False)    # 避免拖到 0 讓 widget 消失
        splitter.setHandleWidth(6)

        for w in widgets:
            # 允許 widget 依需求收縮/擴張
            w.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
            splitter.addWidget(w)
            # 給每個 widget 一個合理的預設 stretch
            splitter.setStretchFactor(splitter.count() - 1, 1)

        # 記錄，供「重置版面」與 QSettings 持久化使用
        self._splitters.append(splitter)

        # 包進 QScrollArea：垂直超出時可捲動，水平自動
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(splitter)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        return scroll

    def _make_vertical_splitter(self, widgets: list,
                                 stretch: list = None) -> QSplitter:
        """通用版本：把一組 widgets 放進 QSplitter(Vertical) 並回傳 splitter。

        Parameters
        ----------
        widgets : list
            要放入 splitter 的 widget 清單
        stretch : list, optional
            各 widget 的 stretch factor；預設為 1 (平均分配)
        """
        splitter = QSplitter(Qt.Orientation.Vertical)
        splitter.setStyleSheet(self._SPLITTER_QSS)
        splitter.setChildrenCollapsible(False)
        splitter.setHandleWidth(6)

        for i, w in enumerate(widgets):
            w.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
            splitter.addWidget(w)
            s = (stretch[i] if stretch and i < len(stretch) else 1)
            splitter.setStretchFactor(i, s)

        self._splitters.append(splitter)
        return splitter

    def reset_resizable_layout(self):
        """重置所有 Splitter 的比例為平均分配（供工具列「重置版面」按鈕）"""
        for sp in self._splitters:
            n = sp.count()
            if n == 0:
                continue
            total = max(sp.width() if sp.orientation() == Qt.Orientation.Horizontal
                        else sp.height(), 100)
            per = total // n
            sp.setSizes([per] * n)

    def create_corner_management(self):
        """創建邊界點管理群組"""
        group = QGroupBox("邊界點管理")
        layout = QVBoxLayout(group)

        # 座標輸入區（隱藏 - 改用地圖直接點擊）
        coord_layout = QHBoxLayout()

        # 緯度輸入
        lat_layout = QVBoxLayout()
        lat_layout.addWidget(QLabel("緯度:"))
        self.lat_input = QDoubleSpinBox()
        self.lat_input.setRange(-90.0, 90.0)
        self.lat_input.setDecimals(6)
        self.lat_input.setValue(settings.map.default_lat)
        lat_layout.addWidget(self.lat_input)
        coord_layout.addLayout(lat_layout)

        # 經度輸入
        lon_layout = QVBoxLayout()
        lon_layout.addWidget(QLabel("經度:"))
        self.lon_input = QDoubleSpinBox()
        self.lon_input.setRange(-180.0, 180.0)
        self.lon_input.setDecimals(6)
        self.lon_input.setValue(settings.map.default_lon)
        lon_layout.addWidget(self.lon_input)
        coord_layout.addLayout(lon_layout)

        # 隱藏手動輸入區域
        coord_widget = QWidget()
        coord_widget.setLayout(coord_layout)
        coord_widget.setVisible(False)  # 隱藏
        layout.addWidget(coord_widget)

        # 提示：使用地圖點擊添加
        click_hint = QLabel("🖱️ 請直接在地圖上左鍵點擊添加角點（右鍵開啟工具選單）")
        click_hint.setStyleSheet("""
            background-color: rgba(33, 150, 243, 0.1);
            color: #2196F3;
            padding: 10px;
            border-radius: 5px;
            font-weight: bold;
            font-size: 12px;
        """)
        click_hint.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(click_hint)

        # 刪除最後一點按鈕
        self.remove_last_corner_btn = QPushButton("↩️ 刪除最後一點")
        self.remove_last_corner_btn.setStyleSheet("background-color: #FF9800; color: white; font-weight: bold;")
        self.remove_last_corner_btn.setToolTip("刪除最後一個新增的角點（快捷鍵: Backspace）")
        self.remove_last_corner_btn.clicked.connect(self.on_remove_last_corner)
        layout.addWidget(self.remove_last_corner_btn)

        # 角點數量顯示
        self.corner_count_label = QLabel("目前角點: 0 個")
        self.corner_count_label.setStyleSheet("color: #2196F3; font-weight: bold;")
        layout.addWidget(self.corner_count_label)

        return group

    def create_circle_center_panel(self) -> QGroupBox:
        """創建螺旋/同心圓圓心掃描設定群組"""
        group = QGroupBox("螺旋 / 同心圓掃描設定")
        group.setObjectName("circleCenterGroup")
        group.setStyleSheet(
            "QGroupBox#circleCenterGroup { font-weight: bold; color: #2196F3; }"
        )
        layout = QVBoxLayout(group)
        layout.setSpacing(6)

        # ── 圓心座標顯示 ──
        center_frame = QFrame()
        center_frame.setFrameShape(QFrame.Shape.StyledPanel)
        center_frame_layout = QHBoxLayout(center_frame)
        center_frame_layout.setContentsMargins(6, 4, 6, 4)
        self.circle_center_label = QLabel("圓心：尚未設定")
        self.circle_center_label.setStyleSheet(
            "color: #888; font-size: 11px; font-style: italic;"
        )
        self.circle_center_label.setWordWrap(True)
        center_frame_layout.addWidget(self.circle_center_label, 1)
        layout.addWidget(center_frame)

        # ── 操作按鈕（拖曳定義 / 點選圓心）──
        btn_row = QHBoxLayout()
        drag_btn = QPushButton("地圖拖曳定義圓形")
        drag_btn.setStyleSheet(
            "background-color: #1565C0; color: white; font-weight: bold; padding: 5px;"
        )
        drag_btn.setToolTip(
            "按住滑鼠左鍵並在地圖上拖曳\n"
            "即可同時設定圓心與半徑（Mission Planner 風格）"
        )
        drag_btn.clicked.connect(self.drag_circle_requested.emit)
        btn_row.addWidget(drag_btn, 2)

        pick_btn = QPushButton("僅點選圓心")
        pick_btn.setStyleSheet(
            "background-color: #455A64; color: white; padding: 5px;"
        )
        pick_btn.setToolTip("點擊此鈕後，在地圖上左鍵點一下即可設定圓心（半徑手動調整）")
        pick_btn.clicked.connect(self.pick_center_requested.emit)
        btn_row.addWidget(pick_btn, 1)
        layout.addLayout(btn_row)

        # ── 共用參數表單 ──
        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)

        # 最大半徑
        self.circle_max_radius_spin = QDoubleSpinBox()
        self.circle_max_radius_spin.setRange(10.0, 10000.0)
        self.circle_max_radius_spin.setValue(self.parameters['circle_max_radius'])
        self.circle_max_radius_spin.setSuffix(" m")
        self.circle_max_radius_spin.setDecimals(0)
        self.circle_max_radius_spin.setToolTip("從圓心向外掃描的最大半徑（拖曳時自動填入）")
        self.circle_max_radius_spin.valueChanged.connect(
            lambda v: self.update_parameter('circle_max_radius', v)
        )
        form.addRow("最大半徑:", self.circle_max_radius_spin)

        # 起始半徑（同心圓最小圈）
        self.circle_min_radius_spin = QDoubleSpinBox()
        self.circle_min_radius_spin.setRange(0.0, 5000.0)
        self.circle_min_radius_spin.setValue(self.parameters['circle_min_radius'])
        self.circle_min_radius_spin.setSuffix(" m")
        self.circle_min_radius_spin.setDecimals(0)
        self.circle_min_radius_spin.setToolTip("掃描起始半徑（0 = 從圓心開始）")
        self.circle_min_radius_spin.valueChanged.connect(
            lambda v: self.update_parameter('circle_min_radius', v)
        )
        form.addRow("起始半徑:", self.circle_min_radius_spin)

        # ── 同心圓專用：高度步階 ──
        self.circle_alt_step_label_row = QLabel("高度步階:")
        self.circle_alt_step_spin = QDoubleSpinBox()
        self.circle_alt_step_spin.setRange(0.0, 200.0)
        self.circle_alt_step_spin.setValue(self.parameters['circle_alt_step'])
        self.circle_alt_step_spin.setSuffix(" m/圈")
        self.circle_alt_step_spin.setDecimals(1)
        self.circle_alt_step_spin.setToolTip(
            "【同心圓】每圈高度遞增量（公尺）\n"
            "0 = 所有圈等高，正值 = 向外擴張同時爬升"
        )
        self.circle_alt_step_spin.valueChanged.connect(
            lambda v: self.update_parameter('circle_alt_step', v)
        )
        form.addRow(self.circle_alt_step_label_row, self.circle_alt_step_spin)

        layout.addLayout(form)

        # ── 螺旋專用參數（僅 spiral 模式顯示）──
        self.spiral_params_frame = QFrame()
        self.spiral_params_frame.setFrameShape(QFrame.Shape.StyledPanel)
        self.spiral_params_frame.setStyleSheet(
            "QFrame { border: 1px solid #1565C0; border-radius: 4px; }"
        )
        spiral_form_layout = QVBoxLayout(self.spiral_params_frame)
        spiral_form_layout.setContentsMargins(6, 4, 6, 4)
        spiral_title = QLabel("螺旋路徑參數")
        spiral_title.setStyleSheet("font-weight: bold; color: #1565C0; font-size: 11px;")
        spiral_form_layout.addWidget(spiral_title)

        spiral_form = QFormLayout()
        spiral_form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)

        # 螺旋曲率
        self.spiral_curvature_spin = QDoubleSpinBox()
        self.spiral_curvature_spin.setRange(0.1, 5.0)
        self.spiral_curvature_spin.setValue(self.parameters['spiral_curvature'])
        self.spiral_curvature_spin.setSingleStep(0.1)
        self.spiral_curvature_spin.setDecimals(2)
        self.spiral_curvature_spin.setToolTip(
            "螺旋曲率倍數\n"
            "1.0 = 標準（圈距 = 航線間距）\n"
            "< 1.0 = 緊密螺旋（圈距較小）\n"
            "> 1.0 = 鬆散螺旋（圈距較大）"
        )
        self.spiral_curvature_spin.valueChanged.connect(
            lambda v: self.update_parameter('spiral_curvature', v)
        )
        spiral_form.addRow("螺旋曲率:", self.spiral_curvature_spin)

        # 螺旋高度步階
        self.spiral_alt_step_spin = QDoubleSpinBox()
        self.spiral_alt_step_spin.setRange(0.0, 200.0)
        self.spiral_alt_step_spin.setValue(self.parameters['spiral_alt_step'])
        self.spiral_alt_step_spin.setSuffix(" m/圈")
        self.spiral_alt_step_spin.setDecimals(1)
        self.spiral_alt_step_spin.setToolTip(
            "【螺旋】每旋轉一圈（360°）的高度增量\n"
            "0 = 等高螺旋，正值 = 螺旋上升（3D 螺旋）"
        )
        self.spiral_alt_step_spin.valueChanged.connect(
            lambda v: self.update_parameter('spiral_alt_step', v)
        )
        spiral_form.addRow("高度步階:", self.spiral_alt_step_spin)

        spiral_form_layout.addLayout(spiral_form)
        layout.addWidget(self.spiral_params_frame)
        self.spiral_params_frame.setVisible(False)  # 預設隱藏，切到 spiral 時顯示

        # ── 提示說明 ──
        hint = QLabel(
            "圈間間距請使用上方「航線間距」調整。\n"
            "飛行高度請使用上方「飛行高度」設定。"
        )
        hint.setStyleSheet(
            "background-color: rgba(33,150,243,0.08);"
            "color: #2196F3; padding: 6px; border-radius: 4px; font-size: 10px;"
        )
        hint.setWordWrap(True)
        layout.addWidget(hint)

        return group

    def set_circle_radius_display(self, radius_m: float):
        """從拖曳結果更新最大半徑 spinbox"""
        if hasattr(self, 'circle_max_radius_spin') and radius_m > 1.0:
            self.circle_max_radius_spin.blockSignals(True)
            self.circle_max_radius_spin.setValue(round(radius_m))
            self.circle_max_radius_spin.blockSignals(False)
            self.update_parameter('circle_max_radius', round(radius_m))

    def create_algorithm_vehicle_selection(self):
        """創建演算法與載具選擇群組"""
        group = QGroupBox("演算法與載具")
        layout = QFormLayout(group)

        # 路徑演算法選擇
        self.algorithm_combo = QComboBox()
        self.algorithm_combo.addItems([
            "網格掃描 (Grid) - 覆蓋任務",
            "螺旋掃描 (Spiral) - 覆蓋任務",
            "同心圓掃描 (Circular) - 覆蓋任務",
            "RRT 路徑規劃 - 點對點",
            "RRT* 路徑規劃 - 最優路徑",
            "Dijkstra 路徑規劃 - 最短路徑",
            "DWA 動態窗口 - 即時避障"
        ])
        self.algorithm_combo.setCurrentIndex(0)
        self.algorithm_combo.currentIndexChanged.connect(self.on_algorithm_changed)
        self.algorithm_combo.setToolTip(
            "Grid/Spiral/Circular: 適用於區域覆蓋任務\n"
            "RRT/Dijkstra: 適用於點對點路徑規劃\n"
            "DWA: 適用於即時避障"
        )
        layout.addRow("路徑演算法:", self.algorithm_combo)

        # 載具類型選擇
        self.vehicle_type_combo = QComboBox()
        self.vehicle_type_combo.addItems(["多旋翼", "固定翼", "VTOL"])
        self.vehicle_type_combo.setCurrentIndex(0)
        self.vehicle_type_combo.currentIndexChanged.connect(self.on_vehicle_type_changed)
        layout.addRow("載具類型:", self.vehicle_type_combo)

        # 載具型號選擇
        self.vehicle_model_combo = QComboBox()
        self._update_vehicle_models("多旋翼")
        self.vehicle_model_combo.currentIndexChanged.connect(self.on_vehicle_model_changed)
        layout.addRow("載具型號:", self.vehicle_model_combo)

        # 載具資訊標籤
        self.vehicle_info_label = QLabel("選擇載具以顯示資訊")
        self.vehicle_info_label.setStyleSheet("color: #888; font-size: 10px;")
        self.vehicle_info_label.setWordWrap(True)
        layout.addRow("", self.vehicle_info_label)

        return group

    def _update_vehicle_models(self, vehicle_type: str):
        """更新載具型號列表"""
        self.vehicle_model_combo.clear()

        if vehicle_type == "多旋翼":
            self.vehicle_model_combo.addItems([
                "DJI Mavic 3",
                "DJI Phantom 4 Pro",
                "DJI Mini 3 Pro",
                "Generic Quadcopter"
            ])
        elif vehicle_type == "固定翼":
            self.vehicle_model_combo.addItems([
                "衝浪者 (Surfer)",
                "Generic Fixed Wing",
            ])
        elif vehicle_type == "VTOL":
            self.vehicle_model_combo.addItems([
                "Generic VTOL"
            ])

    def on_algorithm_changed(self, index):
        """處理演算法變更"""
        algorithms = ['grid', 'spiral', 'circular', 'rrt', 'rrt_star', 'dijkstra', 'dwa']
        algorithm = algorithms[index] if index < len(algorithms) else 'grid'
        self.update_parameter('algorithm', algorithm)

        # 螺旋/同心圓模式：顯示圓心掃描設定
        is_circle_mode = algorithm in ('spiral', 'circular')
        is_spiral = algorithm == 'spiral'
        is_circular = algorithm == 'circular'
        if hasattr(self, 'circle_center_group'):
            self.circle_center_group.setVisible(is_circle_mode)
        # 螺旋專用參數區塊（曲率 / 高度步階）
        if hasattr(self, 'spiral_params_frame'):
            self.spiral_params_frame.setVisible(is_spiral)
        # 同心圓高度步階（螺旋模式用 spiral_alt_step，同心圓用 circle_alt_step）
        if hasattr(self, 'circle_alt_step_spin'):
            self.circle_alt_step_spin.setVisible(is_circular)
        if hasattr(self, 'circle_alt_step_label_row'):
            self.circle_alt_step_label_row.setVisible(is_circular)

        # 更新主視窗的演算法設定
        main_window = self.parent()
        if main_window:
            main_window.current_algorithm = algorithm

        # 顯示演算法說明
        algorithm_info = {
            'grid': "網格掃描：適合覆蓋測繪任務，之字形路徑",
            'spiral': "螺旋掃描：從外圍向中心螺旋掃描",
            'circular': "同心圓掃描：從中心向外擴張的同心圓環路徑，可逐圈上升",
            'rrt': "RRT 演算法：快速探索隨機樹，適合複雜環境",
            'rrt_star': "RRT* 演算法：RRT 的最優化版本",
            'dijkstra': "Dijkstra 演算法：保證最短路徑",
            'dwa': "DWA 動態窗口：即時避障，適合動態環境"
        }
        info = algorithm_info.get(algorithm, "")
        self.algorithm_combo.setToolTip(info)

        logger.info(f"演算法變更: {algorithm} - {info}")

    def on_vehicle_type_changed(self, index):
        """處理載具類型變更"""
        vehicle_types = ["多旋翼", "固定翼", "VTOL"]
        vehicle_type = vehicle_types[index] if index < len(vehicle_types) else "多旋翼"
        self._update_vehicle_models(vehicle_type)
        self.update_parameter('vehicle_type', vehicle_type)

        is_fixed_wing = (vehicle_type == "固定翼")

        # 顯示/隱藏轉彎半徑（飛行參數群組內）
        if hasattr(self, 'turn_radius_spin'):
            self.turn_radius_spin.setVisible(is_fixed_wing)
            self.turn_radius_label.setVisible(is_fixed_wing)

        # 顯示/隱藏固定翼專用設定群組
        if hasattr(self, 'fixed_wing_group'):
            self.fixed_wing_group.setVisible(is_fixed_wing)
            if is_fixed_wing:
                # 切換到固定翼時立即執行轉彎半徑檢測
                self._update_turn_radius_check()

        logger.info(f"載具類型變更: {vehicle_type}")

    def on_vehicle_model_changed(self, index):
        """處理載具型號變更"""
        model = self.vehicle_model_combo.currentText()
        self.update_parameter('vehicle_model', model)

        # 更新載具資訊
        vehicle_info = self._get_vehicle_info(model)
        self.vehicle_info_label.setText(vehicle_info)

        # 如果有對應的參數，自動更新飛行參數
        self._apply_vehicle_defaults(model)

        logger.info(f"載具型號變更: {model}")

    def _get_vehicle_info(self, model: str) -> str:
        """獲取載具資訊"""
        info_db = {
            "DJI Mavic 3": "最大速度: 19m/s | 飛行時間: 46min | 抗風: 12m/s",
            "DJI Phantom 4 Pro": "最大速度: 20m/s | 飛行時間: 30min | 抗風: 10m/s",
            "DJI Mini 3 Pro": "最大速度: 16m/s | 飛行時間: 34min | 抗風: 10.7m/s",
            "Generic Quadcopter": "最大速度: 15m/s | 飛行時間: 25min | 抗風: 10m/s",
            "衝浪者 (Surfer)": "巡航: 18m/s | 飛行時間: 90min | 最小轉彎半徑: 30m | 跑道: 50m",
            "Generic Fixed Wing": "最大速度: 25m/s | 飛行時間: 120min | 最小轉彎半徑: 50m",
            "Generic VTOL": "最大速度: 30m/s | 飛行時間: 90min | 抗風: 12m/s",
        }
        return info_db.get(model, "無資訊")

    def _apply_vehicle_defaults(self, model: str):
        """根據載具型號應用預設參數（含固定翼五邊參數）"""
        defaults_db = {
            "DJI Mavic 3":       {'speed': 15.0, 'altitude': 60.0,  'turn_radius': 0},
            "DJI Phantom 4 Pro": {'speed': 12.0, 'altitude': 50.0,  'turn_radius': 0},
            "DJI Mini 3 Pro":    {'speed': 10.0, 'altitude': 40.0,  'turn_radius': 0},
            "Generic Quadcopter":{'speed': 3.0,  'altitude': 10.0,  'turn_radius': 0},
            "衝浪者 (Surfer)": {
                'speed': 18.0, 'altitude': 100.0, 'turn_radius': 50.0,
                'fw_runway_length': 50.0, 'fw_pattern_alt': 80.0,
                'fw_downwind_offset': 200.0, 'fw_pattern_leg': 300.0,
                'fw_final_dist': 400.0,
            },
            "Generic Fixed Wing": {
                'speed': 18.0, 'altitude': 100.0, 'turn_radius': 80.0,
                'fw_runway_length': 30.0, 'fw_pattern_alt': 100.0,
                'fw_downwind_offset': 250.0, 'fw_pattern_leg': 400.0,
                'fw_final_dist': 500.0,
            },
            "Generic VTOL": {'speed': 15.0, 'altitude': 80.0, 'turn_radius': 30.0},
        }

        defaults = defaults_db.get(model)
        if not defaults:
            return

        self.speed_spin.setValue(defaults['speed'])
        self.altitude_spin.setValue(defaults['altitude'])

        if defaults.get('turn_radius', 0) > 0:
            self.turn_radius_spin.setValue(defaults['turn_radius'])

        # 固定翼五邊降落參數
        if hasattr(self, 'fw_runway_spin') and 'fw_runway_length' in defaults:
            self.fw_runway_spin.setValue(defaults['fw_runway_length'])
        if hasattr(self, 'fw_pattern_alt_spin') and 'fw_pattern_alt' in defaults:
            self.fw_pattern_alt_spin.setValue(defaults['fw_pattern_alt'])
        if hasattr(self, 'fw_downwind_offset_spin') and 'fw_downwind_offset' in defaults:
            self.fw_downwind_offset_spin.setValue(defaults['fw_downwind_offset'])
        if hasattr(self, 'fw_pattern_leg_spin') and 'fw_pattern_leg' in defaults:
            self.fw_pattern_leg_spin.setValue(defaults['fw_pattern_leg'])
        if hasattr(self, 'fw_final_dist_spin') and 'fw_final_dist' in defaults:
            self.fw_final_dist_spin.setValue(defaults['fw_final_dist'])
        if hasattr(self, 'fw_landing_rollout_spin') and 'fw_landing_rollout' in defaults:
            self.fw_landing_rollout_spin.setValue(defaults['fw_landing_rollout'])

        # 套用後立即更新轉彎半徑檢測
        self._update_turn_radius_check()

    def on_add_preset_area(self):
        """添加預設測試區域"""
        # 清除現有角點
        self.clear_corners_requested.emit()

        # 在預設位置周圍創建一個約 200m x 200m 的矩形
        center_lat = settings.map.default_lat
        center_lon = settings.map.default_lon

        # 大約 0.0018 度 ≈ 200m
        offset = 0.0009

        corners = [
            (center_lat + offset, center_lon - offset),  # 左上
            (center_lat + offset, center_lon + offset),  # 右上
            (center_lat - offset, center_lon + offset),  # 右下
            (center_lat - offset, center_lon - offset),  # 左下
        ]

        for lat, lon in corners:
            self.corner_added.emit(lat, lon)

        logger.info("已添加預設測試區域（4個角點）")

    def on_add_corner(self):
        """新增角點"""
        lat = self.lat_input.value()
        lon = self.lon_input.value()
        self.corner_added.emit(lat, lon)
        logger.info(f"手動新增角點: ({lat:.6f}, {lon:.6f})")

    # ──────────────────────────────────────────────────────────────────
    # 子區域安全設定（動態 UI）
    # ──────────────────────────────────────────────────────────────────

    def create_region_safety_group(self) -> QGroupBox:
        """建立「子區域安全設定」群組（分割 > 1 時才顯示）"""
        group = QGroupBox("子區域安全設定")
        outer = QVBoxLayout(group)
        outer.setSpacing(6)

        # ── 說明標籤 ────────────────────────────────────────────────
        hint = QLabel("設定各子區域邊界間隔，防止相鄰無人機物理碰撞")
        hint.setStyleSheet("color:#888; font-size:10px;")
        hint.setWordWrap(True)
        outer.addWidget(hint)

        # ── 動態間隔區塊（由 _rebuild_region_gaps 填充）────────────
        self._gap_container = QWidget()
        self._gap_vbox = QVBoxLayout(self._gap_container)
        self._gap_vbox.setContentsMargins(0, 0, 0, 0)
        self._gap_vbox.setSpacing(4)
        outer.addWidget(self._gap_container)

        # ── 轉場高度錯層 ─────────────────────────────────────────────
        self._transit_check = QCheckBox("啟用轉場高度錯層（防止路線交錯碰撞）")
        self._transit_check.setToolTip(
            "各無人機從起飛點飛往各子區域時，\n"
            "以不同高度轉場，避免路線在空中交錯"
        )
        self._transit_check.stateChanged.connect(self._on_transit_stagger_toggled)
        outer.addWidget(self._transit_check)

        # 錯層間距（預設隱藏）
        stagger_row = QHBoxLayout()
        stagger_row.addWidget(QLabel("各機高度間距:"))
        self._stagger_spin = QDoubleSpinBox()
        self._stagger_spin.setRange(5.0, 100.0)
        self._stagger_spin.setValue(10.0)
        self._stagger_spin.setSuffix(" m")
        self._stagger_spin.setDecimals(1)
        self._stagger_spin.setToolTip("相鄰無人機轉場時的高度差（公尺）")
        self._stagger_spin.valueChanged.connect(
            lambda v: self.update_parameter('transit_stagger_alt', v)
        )
        stagger_row.addWidget(self._stagger_spin)
        stagger_row.addStretch()

        self._stagger_detail = QLabel("")
        self._stagger_detail.setStyleSheet("color:#2196F3; font-size:10px;")
        self._stagger_detail.setWordWrap(True)

        self._stagger_widget = QWidget()
        stagger_inner = QVBoxLayout(self._stagger_widget)
        stagger_inner.setContentsMargins(16, 0, 0, 0)
        stagger_inner.addLayout(stagger_row)
        stagger_inner.addWidget(self._stagger_detail)
        self._stagger_widget.setVisible(False)
        outer.addWidget(self._stagger_widget)

        return group

    def _rebuild_region_gaps(self, n_regions: int):
        """依分割數量重建動態間隔 spinbox"""
        # 清除舊 spinbox
        self._gap_spinboxes.clear()
        while self._gap_vbox.count():
            item = self._gap_vbox.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        if n_regions <= 1:
            self.region_safety_group.setVisible(False)
            self.parameters['gap_spacings_m'] = []
            return

        self.region_safety_group.setVisible(True)
        default_gap = self.parameters.get('region_spacing', 3.0)

        if n_regions in (2, 3, 5):
            # 條帶模式：n-1 個邊界間隔
            n_gaps = n_regions - 1
            self.parameters['gap_spacings_m'] = [default_gap] * n_gaps
            for i in range(n_gaps):
                row_w = QWidget()
                row = QHBoxLayout(row_w)
                row.setContentsMargins(0, 0, 0, 0)
                lbl = QLabel(f"區域 {i+1}–{i+2} 間距:")
                lbl.setMinimumWidth(95)
                row.addWidget(lbl)
                sb = QDoubleSpinBox()
                sb.setRange(0.0, 100.0)
                sb.setValue(default_gap)
                sb.setSuffix(" m")
                sb.setDecimals(1)
                sb.valueChanged.connect(
                    lambda v, idx=i: self._on_gap_changed(idx, v)
                )
                row.addWidget(sb)
                row.addStretch()
                self._gap_vbox.addWidget(row_w)
                self._gap_spinboxes.append(sb)

        elif n_regions == 4:
            # 2×2 網格：水平間距 + 垂直間距
            self.parameters['gap_spacings_m'] = []
            for label_text, param_key in [
                ("水平（列）間距:", 'region_spacing'),
                ("垂直（行）間距:", 'v_spacing_m'),
            ]:
                row_w = QWidget()
                row = QHBoxLayout(row_w)
                row.setContentsMargins(0, 0, 0, 0)
                lbl = QLabel(label_text)
                lbl.setMinimumWidth(95)
                row.addWidget(lbl)
                sb = QDoubleSpinBox()
                sb.setRange(0.0, 100.0)
                sb.setValue(self.parameters.get(param_key, default_gap))
                sb.setSuffix(" m")
                sb.setDecimals(1)
                key = param_key
                sb.valueChanged.connect(
                    lambda v, k=key: self.update_parameter(k, v)
                )
                row.addWidget(sb)
                row.addStretch()
                self._gap_vbox.addWidget(row_w)
                self._gap_spinboxes.append(sb)

        elif n_regions == 6:
            # 3×2 網格：水平間距 + 垂直間距
            self.parameters['gap_spacings_m'] = []
            for label_text, param_key in [
                ("水平（列）間距:", 'region_spacing'),
                ("垂直（行）間距:", 'v_spacing_m'),
            ]:
                row_w = QWidget()
                row = QHBoxLayout(row_w)
                row.setContentsMargins(0, 0, 0, 0)
                lbl = QLabel(label_text)
                lbl.setMinimumWidth(95)
                row.addWidget(lbl)
                sb = QDoubleSpinBox()
                sb.setRange(0.0, 100.0)
                sb.setValue(self.parameters.get(param_key, default_gap))
                sb.setSuffix(" m")
                sb.setDecimals(1)
                key = param_key
                sb.valueChanged.connect(
                    lambda v, k=key: self.update_parameter(k, v)
                )
                row.addWidget(sb)
                row.addStretch()
                self._gap_vbox.addWidget(row_w)
                self._gap_spinboxes.append(sb)

        # 更新轉場高度錯層說明
        self._update_stagger_detail(n_regions)

    def _on_gap_changed(self, idx: int, value: float):
        """某條邊界間隔變更"""
        gaps = self.parameters.get('gap_spacings_m', [])
        if idx < len(gaps):
            gaps[idx] = value
        else:
            while len(gaps) <= idx:
                gaps.append(value)
            gaps[idx] = value
        self.parameters['gap_spacings_m'] = gaps
        self.parameters_changed.emit({'gap_spacings_m': gaps})
        logger.debug(f"邊界間隔更新: index={idx}, value={value}m, all={gaps}")

    def _on_transit_stagger_toggled(self, state):
        """轉場錯層開關"""
        enabled = (state == Qt.CheckState.Checked.value)
        self._stagger_widget.setVisible(enabled)
        self.update_parameter('transit_stagger_enabled', enabled)
        if enabled:
            self._update_stagger_detail(self.parameters.get('subdivisions', 1))

    def _update_stagger_detail(self, n_regions: int):
        """更新轉場高度說明標籤"""
        if not self._transit_check.isChecked() or n_regions <= 1:
            self._stagger_detail.setText("")
            return
        base = self.parameters.get('altitude', 10.0)
        step = self._stagger_spin.value()
        lines = []
        for i in range(n_regions):
            lines.append(f"區域 {i+1}: {base + i * step:.0f} m")
        self._stagger_detail.setText("轉場高度: " + "  |  ".join(lines))

    def on_remove_last_corner(self):
        """刪除最後一個角點"""
        self.remove_last_corner_requested.emit()
        logger.info("請求刪除最後一個角點")

    def on_clear_corners(self):
        """清除所有角點"""
        self.clear_corners_requested.emit()
        logger.info("請求清除所有角點")

    def update_corner_count(self, count: int):
        """更新角點數量顯示"""
        self.corner_count_label.setText(f"目前角點: {count} 個")
        if count >= 3:
            self.corner_count_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
        else:
            self.corner_count_label.setStyleSheet("color: #f44336; font-weight: bold;")

    def set_circle_center_display(self, lat: float, lon: float, radius_m: float = 0.0):
        """更新圓心座標顯示（拖曳時同時帶入半徑）"""
        radius_str = f"  ／  半徑 {radius_m:.0f} m" if radius_m > 1.0 else ""
        if hasattr(self, 'circle_center_label'):
            self.circle_center_label.setText(
                f"圓心：{lat:.6f}, {lon:.6f}{radius_str}"
            )
            self.circle_center_label.setStyleSheet(
                "color: #4CAF50; font-size: 11px; font-weight: bold;"
            )
        self.update_parameter('circle_center_lat', lat)
        self.update_parameter('circle_center_lon', lon)
        if radius_m > 1.0:
            self.set_circle_radius_display(radius_m)

    def set_home_point_display(self, lat: float, lon: float):
        """更新起飛點座標顯示"""
        txt = f"起飛點：{lat:.6f}, {lon:.6f}"
        style = "color: #4CAF50; font-size: 11px; font-weight: bold;"
        if hasattr(self, 'home_point_label'):
            self.home_point_label.setText(txt)
            self.home_point_label.setStyleSheet(style)
        if hasattr(self, 'dccpp_home_label'):
            self.dccpp_home_label.setText(txt)
            self.dccpp_home_label.setStyleSheet(style)

    def clear_home_point_display(self):
        """清除起飛點顯示（由 main_window 呼叫，純 UI 更新）"""
        txt = "起飛點：尚未設定（預設使用掃描區中心）"
        style = "color: #888; font-size: 11px; font-style: italic;"
        if hasattr(self, 'home_point_label'):
            self.home_point_label.setText(txt)
            self.home_point_label.setStyleSheet(style)
        if hasattr(self, 'dccpp_home_label'):
            self.dccpp_home_label.setText(txt)
            self.dccpp_home_label.setStyleSheet(style)

    def create_flight_parameters(self):
        """創建飛行參數群組"""
        group = QGroupBox("飛行參數")
        layout = QFormLayout(group)
        
        # 飛行高度
        self.altitude_spin = QDoubleSpinBox()
        self.altitude_spin.setRange(settings.safety.min_altitude_m, 
                                settings.safety.max_altitude_m)
        self.altitude_spin.setValue(self.parameters['altitude'])
        self.altitude_spin.setSuffix(" m")
        self.altitude_spin.setDecimals(1)
        self.altitude_spin.valueChanged.connect(lambda v: self.update_parameter('altitude', v))
        layout.addRow("飛行高度:", self.altitude_spin)

        # 網格高度遞增（每條掃描線疊加，0=等高）
        self.grid_alt_step_spin = QDoubleSpinBox()
        self.grid_alt_step_spin.setRange(-50.0, 50.0)
        self.grid_alt_step_spin.setValue(self.parameters.get('grid_alt_step', 0.0))
        self.grid_alt_step_spin.setSuffix(" m/條")
        self.grid_alt_step_spin.setDecimals(1)
        self.grid_alt_step_spin.setSingleStep(0.5)
        self.grid_alt_step_spin.setToolTip(
            "每條掃描線高度遞增量（公尺）\n"
            "0 = 等高；正值=逐條爬升；負值=逐條下降\n"
            "基礎高度使用上方「飛行高度」"
        )
        self.grid_alt_step_spin.valueChanged.connect(
            lambda v: self.update_parameter('grid_alt_step', v)
        )
        layout.addRow("網格高度遞增:", self.grid_alt_step_spin)

        # 飛行速度
        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(settings.safety.min_speed_mps,
                                settings.safety.max_speed_mps)
        self.speed_spin.setValue(self.parameters['speed'])
        self.speed_spin.setSuffix(" m/s")
        self.speed_spin.setDecimals(1)
        self.speed_spin.valueChanged.connect(lambda v: self.update_parameter('speed', v))
        self.speed_spin.valueChanged.connect(lambda _: self._update_turn_radius_check())
        layout.addRow("飛行速度:", self.speed_spin)
        
        # 轉向速度
        self.yaw_speed_spin = QDoubleSpinBox()
        self.yaw_speed_spin.setRange(10.0, 360.0)
        self.yaw_speed_spin.setValue(self.parameters['yaw_speed'])
        self.yaw_speed_spin.setSuffix(" °/s")
        self.yaw_speed_spin.setDecimals(1)
        self.yaw_speed_spin.valueChanged.connect(lambda v: self.update_parameter('yaw_speed', v))
        layout.addRow("轉向速度:", self.yaw_speed_spin)

        # 固定翼轉彎半徑（預設隱藏）
        self.turn_radius_label = QLabel("轉彎半徑:")
        self.turn_radius_spin = QDoubleSpinBox()
        self.turn_radius_spin.setRange(10.0, 500.0)
        self.turn_radius_spin.setValue(self.parameters['turn_radius'])
        self.turn_radius_spin.setSuffix(" m")
        self.turn_radius_spin.setDecimals(1)
        self.turn_radius_spin.setToolTip("固定翼飛機的最小轉彎半徑，用於生成平滑路徑")
        self.turn_radius_spin.valueChanged.connect(lambda v: self.update_parameter('turn_radius', v))
        self.turn_radius_spin.valueChanged.connect(lambda _: self._update_turn_radius_check())
        layout.addRow(self.turn_radius_label, self.turn_radius_spin)

        # 預設隱藏固定翼參數
        self.turn_radius_label.setVisible(False)
        self.turn_radius_spin.setVisible(False)

        # ── Heading Lock（多旋翼專用）────────────────────────────────────────
        self.lock_heading_check = QCheckBox("鎖定機頭朝向 (Heading Lock)")
        self.lock_heading_check.setToolTip(
            "勾選後，匯出 Mission Planner 檔案時所有\n"
            "CONDITION_YAW 指令都會使用固定方向，\n"
            "而非自動計算到下一航點的方位角。"
        )
        self.lock_heading_check.stateChanged.connect(self.on_lock_heading_changed)
        layout.addRow("", self.lock_heading_check)

        # Heading 滑條（預設隱藏，勾選後顯示）
        self.heading_row_label = QLabel("機頭方向:")
        heading_layout = QHBoxLayout()
        self.heading_slider = QSlider(Qt.Orientation.Horizontal)
        self.heading_slider.setRange(0, 359)
        self.heading_slider.setValue(0)
        self.heading_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.heading_slider.setTickInterval(45)
        self.heading_value_label = QLabel("0° 北")
        self.heading_value_label.setMinimumWidth(55)
        self.heading_value_label.setStyleSheet("font-weight: bold; color: #2196F3;")
        self.heading_slider.valueChanged.connect(self.on_heading_changed)
        heading_layout.addWidget(self.heading_slider)
        heading_layout.addWidget(self.heading_value_label)
        self.heading_container = QWidget()
        self.heading_container.setLayout(heading_layout)
        layout.addRow(self.heading_row_label, self.heading_container)

        # 預設隱藏 heading 滑條
        self.heading_row_label.setVisible(False)
        self.heading_container.setVisible(False)

        return group
    
    def create_fixed_wing_panel(self) -> QGroupBox:
        """
        創建固定翼專用設定群組

        包含：
          1. 轉彎半徑合規性檢測（即時更新）
          2. 起飛設定（方向、跑道長度）
          3. 降落設定（五邊進場各參數）
        """
        group = QGroupBox("固定翼專用設定")
        group.setObjectName("fixedWingGroup")
        # 使用 object-name 選擇器，防止樣式向下繼承至子 QGroupBox
        group.setStyleSheet("QGroupBox#fixedWingGroup { font-weight: bold; color: #FF9800; }")
        outer = QVBoxLayout(group)
        outer.setSpacing(8)

        # ── 0. 起飛點設定 ─────────────────────────────────────
        home_group = QGroupBox("起飛點設定")
        home_layout = QVBoxLayout(home_group)
        home_layout.setSpacing(4)

        self.home_point_label = QLabel("起飛點：尚未設定（預設使用掃描區中心）")
        self.home_point_label.setStyleSheet(
            "color: #888; font-size: 11px; font-style: italic;"
        )
        self.home_point_label.setWordWrap(True)
        home_layout.addWidget(self.home_point_label)

        pick_home_btn = QPushButton("📍 從地圖點選起飛點")
        pick_home_btn.setStyleSheet(
            "background-color: #455A64; color: white; padding: 6px; border-radius: 3px;"
        )
        pick_home_btn.clicked.connect(self.pick_home_point_requested.emit)
        home_layout.addWidget(pick_home_btn)

        clear_home_btn = QPushButton("✖ 清除起飛點")
        clear_home_btn.setStyleSheet(
            "background-color: #78909C; color: white; padding: 4px; border-radius: 3px;"
        )
        clear_home_btn.clicked.connect(self.clear_home_point_requested.emit)
        home_layout.addWidget(clear_home_btn)

        outer.addWidget(home_group)

        # ── 0.5. 禁航區設定 (NFZ) ─────────────────────────────
        nfz_group = QGroupBox("禁航區設定 (NFZ)")
        nfz_layout = QVBoxLayout(nfz_group)
        nfz_layout.setSpacing(4)

        self.nfz_count_label = QLabel("目前禁航區：0 個")
        self.nfz_count_label.setStyleSheet("color: #888; font-size: 11px;")
        nfz_layout.addWidget(self.nfz_count_label)

        # 地圖繪製按鈕列
        nfz_draw_row = QHBoxLayout()
        nfz_draw_poly_btn = QPushButton("✏️ 繪製多邊形")
        nfz_draw_poly_btn.setToolTip("在地圖上點擊新增頂點，雙擊完成")
        nfz_draw_poly_btn.setStyleSheet(
            "background-color: #E53935; color: white; padding: 5px; border-radius: 3px;"
        )
        nfz_draw_poly_btn.clicked.connect(self.nfz_draw_polygon_requested.emit)
        nfz_draw_row.addWidget(nfz_draw_poly_btn)

        nfz_draw_circle_btn = QPushButton("⭕ 拖曳圓形")
        nfz_draw_circle_btn.setToolTip("在地圖上按住拖曳定義圓形禁航區")
        nfz_draw_circle_btn.setStyleSheet(
            "background-color: #E53935; color: white; padding: 5px; border-radius: 3px;"
        )
        nfz_draw_circle_btn.clicked.connect(self.nfz_draw_circle_requested.emit)
        nfz_draw_row.addWidget(nfz_draw_circle_btn)
        nfz_layout.addLayout(nfz_draw_row)

        self.nfz_finish_poly_btn = QPushButton("✅ 完成多邊形")
        self.nfz_finish_poly_btn.setStyleSheet(
            "background-color: #388E3C; color: white; padding: 5px; border-radius: 3px;"
        )
        self.nfz_finish_poly_btn.setVisible(False)
        self.nfz_finish_poly_btn.clicked.connect(self.nfz_poly_finish_requested.emit)
        nfz_layout.addWidget(self.nfz_finish_poly_btn)

        manage_nfz_btn = QPushButton("🗂 管理 / 刪除禁航區...")
        manage_nfz_btn.setStyleSheet(
            "background-color: #B71C1C; color: white; padding: 6px; border-radius: 3px;"
        )
        manage_nfz_btn.clicked.connect(self.manage_nfz_requested.emit)
        nfz_layout.addWidget(manage_nfz_btn)

        outer.addWidget(nfz_group)

        # ── 1. 轉彎半徑合規性檢測 ────────────────────────────
        check_group = QGroupBox("轉彎半徑合規性檢測")
        check_layout = QVBoxLayout(check_group)
        check_layout.setSpacing(4)

        # 狀態燈 + 摘要（一行）
        status_row = QHBoxLayout()
        self.fw_status_dot = QLabel("●")
        self.fw_status_dot.setStyleSheet("color: #888; font-size: 20px;")
        self.fw_status_dot.setFixedWidth(24)
        self.fw_status_summary = QLabel("請先設定轉彎半徑與速度")
        self.fw_status_summary.setStyleSheet("font-weight: bold; font-size: 11px;")
        self.fw_status_summary.setWordWrap(True)
        status_row.addWidget(self.fw_status_dot)
        status_row.addWidget(self.fw_status_summary, 1)
        check_layout.addLayout(status_row)

        # 詳細說明
        self.fw_status_detail = QLabel("")
        self.fw_status_detail.setStyleSheet("color: #888; font-size: 10px;")
        self.fw_status_detail.setWordWrap(True)
        check_layout.addWidget(self.fw_status_detail)

        # ── ArduPlane LIM_ROLL_CD 設定 ─────────────────────────
        bank_form = QFormLayout()
        bank_form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        bank_form.setContentsMargins(0, 4, 0, 4)

        # 最大傾斜角欄位
        bank_row = QHBoxLayout()
        self.fw_max_bank_spin = QDoubleSpinBox()
        self.fw_max_bank_spin.setRange(10.0, 60.0)
        self.fw_max_bank_spin.setValue(self.parameters.get('fw_max_bank_deg', 45.0))
        self.fw_max_bank_spin.setSuffix("°")
        self.fw_max_bank_spin.setSingleStep(1.0)
        self.fw_max_bank_spin.setDecimals(1)
        self.fw_max_bank_spin.setToolTip(
            "最大傾斜角（對應 ArduPlane LIM_ROLL_CD）\n"
            "Mission Planner SITL 預設: 45° (= 4500 cd)\n"
            "衝浪者建議: 45°   一般固定翼: 30–45°\n\n"
            "物理公式：R_min = V² / (g × tan(φ))\n"
            "  V=巡航速度, g=9.81, φ=傾斜角"
        )
        self.fw_max_bank_spin.valueChanged.connect(
            lambda v: self.update_parameter('fw_max_bank_deg', v)
        )
        self.fw_max_bank_spin.valueChanged.connect(self._on_bank_angle_changed)

        # 即時顯示對應的最小轉彎半徑
        self.fw_min_radius_label = QLabel("→ 最小半徑: ─ m")
        self.fw_min_radius_label.setStyleSheet(
            "color: #2196F3; font-size: 10px; font-weight: bold;"
        )
        bank_row.addWidget(self.fw_max_bank_spin)
        bank_row.addWidget(self.fw_min_radius_label, 1)
        bank_form.addRow("最大傾斜角\n(LIM_ROLL_CD):", bank_row)

        # 套用：由傾斜角→最小半徑→填入轉彎半徑欄位
        apply_bank_btn = QPushButton("套用最小半徑至轉彎半徑設定")
        apply_bank_btn.setStyleSheet(
            "background-color: #37474F; color: white; font-size: 10px; padding: 3px;"
        )
        apply_bank_btn.setToolTip(
            "依目前「最大傾斜角」和「速度」計算最小轉彎半徑，\n"
            "並自動填入上方「轉彎半徑」欄位"
        )
        apply_bank_btn.clicked.connect(self._apply_min_radius_from_bank)
        bank_form.addRow("", apply_bank_btn)

        check_layout.addLayout(bank_form)

        # 手動觸發檢測按鈕
        check_btn = QPushButton("立即檢測轉彎半徑")
        check_btn.setStyleSheet(
            "background-color: #1565C0; color: white; font-weight: bold;"
        )
        check_btn.clicked.connect(self._update_turn_radius_check)
        check_layout.addWidget(check_btn)

        outer.addWidget(check_group)

        # ── 2. 起飛設定 ──────────────────────────────────────
        takeoff_group = QGroupBox("起飛設定")
        takeoff_layout = QFormLayout(takeoff_group)

        # 起飛方向（跑道方向）
        takeoff_bearing_layout = QHBoxLayout()
        self.fw_takeoff_bearing_spin = QDoubleSpinBox()
        self.fw_takeoff_bearing_spin.setRange(0.0, 359.9)
        self.fw_takeoff_bearing_spin.setValue(self.parameters['fw_takeoff_bearing'])
        self.fw_takeoff_bearing_spin.setSuffix("°")
        self.fw_takeoff_bearing_spin.setDecimals(1)
        self.fw_takeoff_bearing_spin.setToolTip(
            "跑道方向（度，0=北，90=東，180=南，270=西）\n"
            "建議設定為逆風方向"
        )
        self.fw_takeoff_bearing_spin.valueChanged.connect(
            lambda v: self.update_parameter('fw_takeoff_bearing', v)
        )
        self.fw_takeoff_bearing_label = QLabel("")
        self.fw_takeoff_bearing_label.setStyleSheet("color: #2196F3; font-size: 10px; min-width: 30px;")
        self.fw_takeoff_bearing_spin.valueChanged.connect(
            lambda v: self.fw_takeoff_bearing_label.setText(self._bearing_to_compass(v))
        )
        self.fw_takeoff_bearing_label.setText(
            self._bearing_to_compass(self.parameters['fw_takeoff_bearing'])
        )
        takeoff_bearing_layout.addWidget(self.fw_takeoff_bearing_spin)
        takeoff_bearing_layout.addWidget(self.fw_takeoff_bearing_label)
        takeoff_layout.addRow("起飛方向:", takeoff_bearing_layout)

        # 跑道長度
        self.fw_runway_spin = QDoubleSpinBox()
        self.fw_runway_spin.setRange(20.0, 500.0)
        self.fw_runway_spin.setValue(self.parameters['fw_runway_length'])
        self.fw_runway_spin.setSuffix(" m")
        self.fw_runway_spin.setDecimals(0)
        self.fw_runway_spin.setToolTip(
            "起飛滑跑距離（公尺）\n"
            "衝浪者建議: 50m 以上"
        )
        self.fw_runway_spin.valueChanged.connect(
            lambda v: self.update_parameter('fw_runway_length', v)
        )
        takeoff_layout.addRow("跑道長度:", self.fw_runway_spin)

        outer.addWidget(takeoff_group)

        # ── 3. 五邊進場降落設定 ───────────────────────────────
        land_group = QGroupBox("五邊進場降落設定")
        land_layout = QFormLayout(land_group)

        # 降落進場方向
        land_bearing_layout = QHBoxLayout()
        self.fw_landing_bearing_spin = QDoubleSpinBox()
        self.fw_landing_bearing_spin.setRange(0.0, 359.9)
        self.fw_landing_bearing_spin.setValue(self.parameters['fw_landing_bearing'])
        self.fw_landing_bearing_spin.setSuffix("°")
        self.fw_landing_bearing_spin.setDecimals(1)
        self.fw_landing_bearing_spin.setToolTip(
            "飛機降落時的飛行方向（進場方向）\n"
            "通常與起飛方向相反或逆風"
        )
        self.fw_landing_bearing_spin.valueChanged.connect(
            lambda v: self.update_parameter('fw_landing_bearing', v)
        )
        self.fw_landing_bearing_label = QLabel("")
        self.fw_landing_bearing_label.setStyleSheet("color: #2196F3; font-size: 10px; min-width: 30px;")
        self.fw_landing_bearing_spin.valueChanged.connect(
            lambda v: self.fw_landing_bearing_label.setText(self._bearing_to_compass(v))
        )
        self.fw_landing_bearing_label.setText(
            self._bearing_to_compass(self.parameters['fw_landing_bearing'])
        )
        land_bearing_layout.addWidget(self.fw_landing_bearing_spin)
        land_bearing_layout.addWidget(self.fw_landing_bearing_label)
        land_layout.addRow("進場方向:", land_bearing_layout)

        # 五邊飛行高度
        self.fw_pattern_alt_spin = QDoubleSpinBox()
        self.fw_pattern_alt_spin.setRange(30.0, 500.0)
        self.fw_pattern_alt_spin.setValue(self.parameters['fw_pattern_alt'])
        self.fw_pattern_alt_spin.setSuffix(" m")
        self.fw_pattern_alt_spin.setDecimals(0)
        self.fw_pattern_alt_spin.setToolTip("五邊飛行高度 AGL（衝浪者建議: 80m）")
        self.fw_pattern_alt_spin.valueChanged.connect(
            lambda v: self.update_parameter('fw_pattern_alt', v)
        )
        land_layout.addRow("五邊高度:", self.fw_pattern_alt_spin)

        # 下風邊側偏距離
        self.fw_downwind_offset_spin = QDoubleSpinBox()
        self.fw_downwind_offset_spin.setRange(50.0, 1000.0)
        self.fw_downwind_offset_spin.setValue(self.parameters['fw_downwind_offset'])
        self.fw_downwind_offset_spin.setSuffix(" m")
        self.fw_downwind_offset_spin.setDecimals(0)
        self.fw_downwind_offset_spin.setToolTip(
            "下風邊與跑道的側向偏移距離（公尺）\n"
            "衝浪者建議: 200m（右手模式）"
        )
        self.fw_downwind_offset_spin.valueChanged.connect(
            lambda v: self.update_parameter('fw_downwind_offset', v)
        )
        land_layout.addRow("側偏距離:", self.fw_downwind_offset_spin)

        # 下風邊腿長
        self.fw_pattern_leg_spin = QDoubleSpinBox()
        self.fw_pattern_leg_spin.setRange(100.0, 2000.0)
        self.fw_pattern_leg_spin.setValue(self.parameters['fw_pattern_leg'])
        self.fw_pattern_leg_spin.setSuffix(" m")
        self.fw_pattern_leg_spin.setDecimals(0)
        self.fw_pattern_leg_spin.setToolTip("下風邊腿長（公尺）")
        self.fw_pattern_leg_spin.valueChanged.connect(
            lambda v: self.update_parameter('fw_pattern_leg', v)
        )
        land_layout.addRow("下風邊腿長:", self.fw_pattern_leg_spin)

        # 最終進場距離
        self.fw_final_dist_spin = QDoubleSpinBox()
        self.fw_final_dist_spin.setRange(100.0, 2000.0)
        self.fw_final_dist_spin.setValue(self.parameters['fw_final_dist'])
        self.fw_final_dist_spin.setSuffix(" m")
        self.fw_final_dist_spin.setDecimals(0)
        self.fw_final_dist_spin.setToolTip(
            "最終進場起始距離（Final Approach Fix 到跑道頭的距離）"
        )
        self.fw_final_dist_spin.valueChanged.connect(
            lambda v: self.update_parameter('fw_final_dist', v)
        )
        land_layout.addRow("最終進場距:", self.fw_final_dist_spin)

        # 降落滑行距離（觸地後煞車滾停）
        self.fw_landing_rollout_spin = QDoubleSpinBox()
        self.fw_landing_rollout_spin.setRange(0.0, 9999.0)
        self.fw_landing_rollout_spin.setValue(self.parameters.get('fw_landing_rollout', 0.0))
        self.fw_landing_rollout_spin.setSuffix(" m")
        self.fw_landing_rollout_spin.setDecimals(0)
        self.fw_landing_rollout_spin.setSingleStep(10.0)
        self.fw_landing_rollout_spin.setToolTip(
            "觸地後滑行＋煞車所需距離（公尺）\n"
            "觸地點會提前此距離，確保飛機減速停止時\n"
            "不超過跑道末端（Home 點）\n"
            "設 0 = 不偏移，觸地點在 Home"
        )
        self.fw_landing_rollout_spin.valueChanged.connect(
            lambda v: self.update_parameter('fw_landing_rollout', v)
        )
        land_layout.addRow("降落滑行距:", self.fw_landing_rollout_spin)

        # ── AutoLand 開關 ──────────────────────────────────────
        # 分隔線
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet("color: #CFD8DC;")
        land_layout.addRow(sep)

        self.fw_autoland_check = QCheckBox("啟用 AutoLand")
        self.fw_autoland_check.setChecked(self.parameters['fw_autoland'])
        self.fw_autoland_check.setStyleSheet("""
            QCheckBox {
                font-weight: bold;
                color: #E65100;
                font-size: 12px;
                padding: 4px 0px;
            }
            QCheckBox::indicator {
                width: 18px; height: 18px;
            }
            QCheckBox::indicator:checked {
                background-color: #E65100;
                border: 2px solid #BF360C;
                border-radius: 3px;
            }
        """)
        self.fw_autoland_check.setToolTip(
            "【開啟】匯出時跳過五邊電路，在最後任務點後直接插入\n"
            "  DO_LAND_START (189) + NAV_LAND (21)\n"
            "  ArduPlane 將自動執行降落程序（需飛控支援 AUTOLAND）\n\n"
            "【關閉】使用標準五邊進場電路（下風→底邊→最終進場→觸地）"
        )
        self.fw_autoland_check.stateChanged.connect(
            lambda s: self._on_autoland_toggled(
                s == Qt.CheckState.Checked.value
            )
        )
        land_layout.addRow("降落模式:", self.fw_autoland_check)

        # AutoLand 說明標籤（動態顯示/隱藏）
        self._autoland_hint = QLabel(
            "⚡ AutoLand 已啟用：匯出時跳過五邊電路\n"
            "   五邊設定僅供地圖預覽參考"
        )
        self._autoland_hint.setStyleSheet(
            "background-color: rgba(230,81,0,0.10);"
            "color: #E65100; padding: 6px 8px; border-radius: 4px;"
            "font-size: 10px; border: 1px solid #FFCCBC;"
        )
        self._autoland_hint.setWordWrap(True)
        self._autoland_hint.setVisible(False)
        land_layout.addRow(self._autoland_hint)

        outer.addWidget(land_group)

        # ── 4. 固定翼掃描模式說明 ─────────────────────────────
        hint = QLabel(
            "固定翼建議使用「螺旋掃描」或「同心圓掃描」。\n"
            "路徑已自動考慮最小轉彎半徑，若轉彎半徑不足\n"
            "將顯示警告並調整掃描間距。"
        )
        hint.setStyleSheet(
            "background-color: rgba(255,152,0,0.1);"
            "color: #FF9800; padding: 8px; border-radius: 4px;"
            "font-size: 10px;"
        )
        hint.setWordWrap(True)
        outer.addWidget(hint)

        # 連接 turn_radius 和 speed 變更到即時檢測
        # （在 create_flight_parameters 建立後由 init_ui 串接）

        return group

    def _on_autoland_toggled(self, enabled: bool):
        """AutoLand 開關切換：灰化五邊設定、顯示提示"""
        self.update_parameter('fw_autoland', enabled)

        # 五邊設定控件在 AutoLand 模式下不影響匯出，標灰提示
        for w in [
            self.fw_landing_bearing_spin,
            self.fw_pattern_alt_spin,
            self.fw_downwind_offset_spin,
            self.fw_pattern_leg_spin,
            self.fw_final_dist_spin,
        ]:
            w.setEnabled(not enabled)

        # 顯示/隱藏說明標籤
        if hasattr(self, '_autoland_hint'):
            self._autoland_hint.setVisible(enabled)

        logger.info(f"AutoLand {'啟用' if enabled else '關閉'}")

    @staticmethod
    def _bearing_to_compass(bearing_deg: float) -> str:
        """將方位角轉換為羅盤文字"""
        dirs = ['北', '東北', '東', '東南', '南', '西南', '西', '西北', '北']
        idx = int((bearing_deg + 22.5) / 45) % 8
        return dirs[idx]

    def _update_turn_radius_check(self):
        """
        即時更新轉彎半徑合規性狀態

        讀取當前的 turn_radius 和 speed，
        查詢機體規格（衝浪者 / Generic），
        更新 fw_status_dot, fw_status_summary, fw_status_detail 的顏色與文字。
        """
        if not hasattr(self, 'fw_status_dot'):
            return

        from core.global_planner.fixed_wing_planner import TurnRadiusChecker

        turn_r = self.parameters.get('turn_radius', 50.0)
        speed  = self.parameters.get('speed', 18.0)
        model  = self.parameters.get('vehicle_model', '衝浪者 (Surfer)')

        result = TurnRadiusChecker.check(turn_r, speed, model)

        color = result['color']
        status_icons = {'ok': '●', 'warning': '▲', 'critical': '✖'}
        icon = status_icons.get(result['status'], '●')

        self.fw_status_dot.setText(icon)
        self.fw_status_dot.setStyleSheet(f"color: {color}; font-size: 18px; font-weight: bold;")

        # 摘要
        status_text = {'ok': '符合規格', 'warning': '接近極限', 'critical': '超出極限！'}
        summary = f"{status_text.get(result['status'], '')}  |  轉彎半徑: {turn_r:.0f} m"
        self.fw_status_summary.setText(summary)
        self.fw_status_summary.setStyleSheet(f"color: {color}; font-weight: bold; font-size: 11px;")

        # 詳細說明
        detail = (
            f"理論所需傾斜角: {result['required_bank']:.1f}°  "
            f"(機體上限: {result['max_bank']:.0f}°)\n"
            f"機體最小轉彎半徑: {result['min_radius']:.0f} m  "
            f"@速度 {speed:.1f} m/s"
        )
        self.fw_status_detail.setText(detail)
        self.fw_status_detail.setStyleSheet(f"color: {color}; font-size: 10px;")

        # 同步更新「最大傾斜角 → 最小半徑」顯示
        self._refresh_min_radius_label()

        logger.info(
            f"轉彎半徑檢測: 半徑={turn_r:.0f}m, 速度={speed:.1f}m/s, "
            f"狀態={result['status']}, 傾斜角={result['required_bank']:.1f}°"
        )

    def _refresh_min_radius_label(self):
        """依目前「最大傾斜角」和「速度」更新最小半徑顯示標籤"""
        import math
        if not hasattr(self, 'fw_min_radius_label'):
            return
        bank_deg = self.parameters.get('fw_max_bank_deg', 45.0)
        speed    = self.parameters.get('speed', 18.0)
        if bank_deg <= 0:
            return
        tan_b = math.tan(math.radians(bank_deg))
        if abs(tan_b) < 1e-9:
            return
        min_r = speed ** 2 / (9.81 * tan_b)
        self.fw_min_radius_label.setText(f"→ 最小半徑: {min_r:.0f} m")

    def _on_bank_angle_changed(self, _value: float = 0.0):
        """
        傾斜角改變時：
          1. 即時更新最小半徑顯示
          2. 觸發合規性檢測（以更新狀態燈）
        """
        self._refresh_min_radius_label()
        self._update_turn_radius_check()

    def _apply_min_radius_from_bank(self):
        """
        由目前「最大傾斜角 + 速度」計算最小轉彎半徑，
        並填入 turn_radius_spin（供路徑規劃使用）。

        公式（ArduPlane 同款）：
            R_min = V² / (g × tan(φ))
        """
        import math
        bank_deg = self.parameters.get('fw_max_bank_deg', 45.0)
        speed    = self.parameters.get('speed', 18.0)
        if bank_deg <= 0:
            return
        tan_b = math.tan(math.radians(bank_deg))
        if abs(tan_b) < 1e-9:
            return
        min_r = speed ** 2 / (9.81 * tan_b)
        if hasattr(self, 'turn_radius_spin'):
            self.turn_radius_spin.setValue(round(min_r, 1))
        logger.info(
            f"由傾斜角套用最小轉彎半徑: 傾斜角={bank_deg:.1f}°, "
            f"速度={speed:.1f}m/s → 最小半徑={min_r:.0f}m"
        )

    def create_survey_parameters(self):
        """創建測繪參數群組"""
        group = QGroupBox("測繪參數")
        layout = QFormLayout(group)
        
        # 掃描角度
        angle_layout = QHBoxLayout()
        self.angle_slider = QSlider(Qt.Orientation.Horizontal)
        self.angle_slider.setRange(-180, 180)
        self.angle_slider.setValue(int(self.parameters['angle']))
        self.angle_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.angle_slider.setTickInterval(30)
        self.angle_label = QLabel(f"{self.parameters['angle']:.0f}°")
        self.angle_slider.valueChanged.connect(self.on_angle_changed)
        angle_layout.addWidget(self.angle_slider)
        angle_layout.addWidget(self.angle_label)
        layout.addRow("掃描角度:", angle_layout)
        
        # 航線間距
        self.spacing_spin = QDoubleSpinBox()
        self.spacing_spin.setRange(settings.safety.min_spacing_m, 
                                settings.safety.max_spacing_m)
        self.spacing_spin.setValue(self.parameters['spacing'])
        self.spacing_spin.setSuffix(" m")
        self.spacing_spin.setDecimals(1)
        self.spacing_spin.valueChanged.connect(lambda v: self.update_parameter('spacing', v))
        layout.addRow("航線間距:", self.spacing_spin)
        
        # 子區域分割
        self.subdivision_combo = QComboBox()
        self.subdivision_combo.addItems([
            "1 (不分割)", 
            "2 區域", 
            "3 區域", 
            "4 區域 (2x2)", 
            "5 區域",
            "6 區域 (2x3)"
        ])
        self.subdivision_combo.setCurrentIndex(0)
        self.subdivision_combo.currentIndexChanged.connect(self.on_subdivision_changed)
        layout.addRow("區域分割:", self.subdivision_combo)
        
        return group
    
    def create_advanced_parameters(self):
        """創建進階參數群組"""
        group = QGroupBox("進階設定")
        layout = QVBoxLayout(group)
        
        # 減少重疊
        self.reduce_overlap_check = QCheckBox("減少重疊（互補掃描）")
        self.reduce_overlap_check.setChecked(self.parameters['reduce_overlap'])
        self.reduce_overlap_check.stateChanged.connect(
            lambda state: self.update_parameter('reduce_overlap', state == Qt.CheckState.Checked)
        )
        layout.addWidget(self.reduce_overlap_check)
        
        # 飛行模式
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("飛行模式:"))
        self.flight_mode_combo = QComboBox()
        self.flight_mode_combo.addItems(["同步飛行", "智能避撞"])
        self.flight_mode_combo.setCurrentIndex(1)  # 預設智能避撞
        self.flight_mode_combo.currentTextChanged.connect(self.on_flight_mode_changed)
        mode_layout.addWidget(self.flight_mode_combo)
        layout.addLayout(mode_layout)
        
        # 安全距離顯示（只讀）
        safety_layout = QHBoxLayout()
        safety_layout.addWidget(QLabel("安全距離:"))
        safety_label = QLabel(f"{settings.safety.default_safety_distance_m} m")
        safety_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
        safety_layout.addWidget(safety_label)
        safety_layout.addStretch()
        layout.addLayout(safety_layout)
        
        return group
    
    def on_angle_changed(self, value):
        """處理角度變更"""
        self.angle_label.setText(f"{value}°")
        self.update_parameter('angle', float(value))
    
    def on_subdivision_changed(self, index):
        """處理分割數量變更"""
        subdivisions = index + 1  # 1, 2, 3, 4
        self.update_parameter('subdivisions', subdivisions)
        self._rebuild_region_gaps(subdivisions)
    
    def on_flight_mode_changed(self, text):
        """處理飛行模式變更"""
        mode = 'smart_collision' if text == "智能避撞" else 'synchronous'
        self.update_parameter('flight_mode', mode)

    @staticmethod
    def _heading_to_compass(angle: int) -> str:
        """將角度轉換為羅盤方向文字"""
        dirs = ['北', '東北', '東', '東南', '南', '西南', '西', '西北']
        return dirs[round(angle / 45) % 8]

    def on_lock_heading_changed(self, state):
        """處理 Heading Lock 勾選狀態變更"""
        locked = (state == Qt.CheckState.Checked.value)
        self.heading_row_label.setVisible(locked)
        self.heading_container.setVisible(locked)
        self.update_parameter('lock_heading', locked)
        logger.info(f"Heading Lock: {'啟用' if locked else '停用'}")

    def on_heading_changed(self, value: int):
        """處理機頭方向滑條變更"""
        compass = self._heading_to_compass(value)
        self.heading_value_label.setText(f"{value}° {compass}")
        self.update_parameter('heading_angle', float(value))
    
    def update_parameter(self, key: str, value):
        """
        更新參數並發送信號
        
        參數:
            key: 參數名稱
            value: 參數值
        """
        self.parameters[key] = value
        self.parameters_changed.emit({key: value})
        logger.debug(f"參數更新: {key} = {value}")
    
    def get_parameters(self):
        """
        獲取所有參數
        
        返回:
            參數字典
        """
        return self.parameters.copy()
    
    def set_parameters(self, params: dict):
        """
        設置參數
        
        參數:
            params: 參數字典
        """
        for key, value in params.items():
            if key in self.parameters:
                self.parameters[key] = value
                
                # 更新 UI
                if key == 'altitude':
                    self.altitude_spin.setValue(value)
                elif key == 'speed':
                    self.speed_spin.setValue(value)
                elif key == 'angle':
                    self.angle_slider.setValue(int(value))
                elif key == 'spacing':
                    self.spacing_spin.setValue(value)
                elif key == 'yaw_speed':
                    self.yaw_speed_spin.setValue(value)
                elif key == 'subdivisions':
                    self.subdivision_combo.setCurrentIndex(value - 1)

                elif key == 'reduce_overlap':
                    self.reduce_overlap_check.setChecked(value)
                elif key == 'flight_mode':
                    index = 1 if value == 'smart_collision' else 0
                    self.flight_mode_combo.setCurrentIndex(index)
                elif key == 'turn_radius':
                    self.turn_radius_spin.setValue(value)
                elif key == 'fw_max_bank_deg':
                    if hasattr(self, 'fw_max_bank_spin'):
                        self.fw_max_bank_spin.setValue(value)
                elif key == 'lock_heading':
                    self.lock_heading_check.setChecked(bool(value))
                elif key == 'heading_angle':
                    self.heading_slider.setValue(int(value))

        logger.info("參數已設置")
    
    # ══════════════════════════════════════════════════════════════════
    #  DCCPP 最佳化規劃面板（論文 MDTSP + IDP + GDA）
    # ══════════════════════════════════════════════════════════════════

    def create_dccpp_panel(self) -> QGroupBox:
        """建立 DCCPP 最佳化規劃設定群組（多機覆蓋最佳化 + 高度平滑）"""
        group = QGroupBox("=== DCCPP 最佳化規劃 ===")
        group.setObjectName("dccppGroup")
        group.setStyleSheet(
            "QGroupBox#dccppGroup { font-weight: bold; color: #00695C; }"
        )
        layout = QVBoxLayout(group)
        layout.setSpacing(6)

        # 說明標籤
        hint = QLabel(
            "使用論文 MDTSP + IDP 最佳化模型進行\n"
            "多機協同覆蓋路徑規劃，含 GDA 高度平滑。"
        )
        hint.setStyleSheet(
            "background-color: rgba(0,105,92,0.08);"
            "color: #00695C; padding: 6px; border-radius: 4px; font-size: 10px;"
        )
        hint.setWordWrap(True)
        layout.addWidget(hint)

        form = QFormLayout()
        form.setSpacing(4)

        # 載具類型（多旋翼/固定翼）
        self.dccpp_vehicle_combo = QComboBox()
        self.dccpp_vehicle_combo.addItem("多旋翼 (Multirotor)", "multirotor")
        self.dccpp_vehicle_combo.addItem("固定翼 (Fixed-Wing)", "fixed_wing")
        self.dccpp_vehicle_combo.addItem("VTOL (4+1 QuadPlane)", "vtol")
        self.dccpp_vehicle_combo.setToolTip(
            "多旋翼：自由轉向（turn_radius=0）\n"
            "固定翼：Dubins 曲線約束（turn_radius>0）\n"
            "VTOL：垂直起降 + 固定翼巡航（QuadPlane 混飛）"
        )
        self.dccpp_vehicle_combo.currentIndexChanged.connect(
            self._on_dccpp_vehicle_changed
        )
        form.addRow("載具類型:", self.dccpp_vehicle_combo)

        # 無人機數量
        self.dccpp_num_drones_spin = QSpinBox()
        self.dccpp_num_drones_spin.setRange(2, 12)
        self.dccpp_num_drones_spin.setValue(3)
        self.dccpp_num_drones_spin.setToolTip("參與 DCCPP 最佳化的無人機數量（2–12 台）")
        form.addRow("無人機數量:", self.dccpp_num_drones_spin)

        # FOV 覆蓋寬度
        self.dccpp_fov_spin = QDoubleSpinBox()
        self.dccpp_fov_spin.setRange(5.0, 5000.0)
        self.dccpp_fov_spin.setDecimals(1)
        self.dccpp_fov_spin.setValue(100.0)
        self.dccpp_fov_spin.setSuffix(" m")
        self.dccpp_fov_spin.setToolTip("相機 FOV 地面投影寬度（公尺）")
        form.addRow("FOV 覆蓋寬度:", self.dccpp_fov_spin)

        # 重疊率
        self.dccpp_overlap_spin = QDoubleSpinBox()
        self.dccpp_overlap_spin.setRange(0.0, 0.5)
        self.dccpp_overlap_spin.setDecimals(2)
        self.dccpp_overlap_spin.setSingleStep(0.05)
        self.dccpp_overlap_spin.setValue(0.1)
        self.dccpp_overlap_spin.setToolTip("相鄰掃描條帶的重疊率（0.0–0.5）")
        form.addRow("重疊率:", self.dccpp_overlap_spin)

        # 覆蓋高度
        self.dccpp_altitude_spin = QDoubleSpinBox()
        self.dccpp_altitude_spin.setRange(10.0, 5000.0)
        self.dccpp_altitude_spin.setDecimals(1)
        self.dccpp_altitude_spin.setValue(100.0)
        self.dccpp_altitude_spin.setSuffix(" m")
        self.dccpp_altitude_spin.setToolTip("覆蓋飛行高度（公尺）")
        form.addRow("覆蓋高度:", self.dccpp_altitude_spin)

        # 巡航速度
        self.dccpp_speed_spin = QDoubleSpinBox()
        self.dccpp_speed_spin.setRange(1.0, 100.0)
        self.dccpp_speed_spin.setDecimals(1)
        self.dccpp_speed_spin.setValue(15.0)
        self.dccpp_speed_spin.setSuffix(" m/s")
        self.dccpp_speed_spin.setToolTip("無人機巡航速度（公尺/秒）")
        form.addRow("巡航速度:", self.dccpp_speed_spin)

        # 最小轉彎半徑（固定翼專用）
        self.dccpp_turn_radius_label = QLabel("最小轉彎半徑:")
        self.dccpp_turn_radius_spin = QDoubleSpinBox()
        self.dccpp_turn_radius_spin.setRange(10.0, 1000.0)
        self.dccpp_turn_radius_spin.setDecimals(1)
        self.dccpp_turn_radius_spin.setValue(50.0)
        self.dccpp_turn_radius_spin.setSuffix(" m")
        self.dccpp_turn_radius_spin.setToolTip(
            "固定翼最小轉彎半徑（公尺）\n影響 Dubins 曲線路徑長度"
        )
        form.addRow(self.dccpp_turn_radius_label, self.dccpp_turn_radius_spin)
        self.dccpp_turn_radius_label.setVisible(False)
        self.dccpp_turn_radius_spin.setVisible(False)

        layout.addLayout(form)

        # 自動掃描方向
        self.dccpp_auto_scan_check = QCheckBox("自動掃描方向（依多邊形最佳角度）")
        self.dccpp_auto_scan_check.setChecked(True)
        layout.addWidget(self.dccpp_auto_scan_check)

        # 啟用 GDA 高度平滑
        self.dccpp_altitude_check = QCheckBox("啟用 GDA 高度平滑（2D→3D）")
        self.dccpp_altitude_check.setChecked(True)
        self.dccpp_altitude_check.setToolTip(
            "使用梯度下降演算法平滑飛行高度\n"
            "考量地形與爬升/俯衝角限制"
        )
        layout.addWidget(self.dccpp_altitude_check)

        # 自動起降（固定翼）
        self.dccpp_auto_landing_check = QCheckBox("啟用自動起飛/降落（固定翼 Dubins）")
        self.dccpp_auto_landing_check.setChecked(True)
        self.dccpp_auto_landing_check.setToolTip(
            "為每架固定翼 UAV 在作業段前後加入：\n"
            "  • 起飛爬升段（從 Home 起飛 + Dubins 進場）\n"
            "  • 五邊降落段（Dubins 出場 + 下滑落地）\n"
            "關閉時 DCCPP 結果只包含純作業 + 轉移段"
        )
        self.dccpp_auto_landing_check.stateChanged.connect(
            self._on_dccpp_auto_landing_changed
        )
        layout.addWidget(self.dccpp_auto_landing_check)

        # ── 跑道 / 起降設定（固定翼專用）──────────────────
        self.dccpp_runway_group = QGroupBox("跑道 / 起降設定")
        self.dccpp_runway_group.setStyleSheet(
            "QGroupBox { font-weight: bold; color: #FF6D00; }"
        )
        rwy_layout = QVBoxLayout(self.dccpp_runway_group)
        rwy_layout.setSpacing(4)

        # 起飛點顯示
        self.dccpp_home_label = QLabel("起飛點：尚未設定（預設使用掃描區中心）")
        self.dccpp_home_label.setStyleSheet(
            "color: #888; font-size: 11px; font-style: italic;"
        )
        self.dccpp_home_label.setWordWrap(True)
        rwy_layout.addWidget(self.dccpp_home_label)

        home_btn_row = QHBoxLayout()
        dccpp_pick_home_btn = QPushButton("📍 從地圖點選起飛點")
        dccpp_pick_home_btn.setStyleSheet(
            "background-color: #455A64; color: white; padding: 5px; border-radius: 3px;"
        )
        dccpp_pick_home_btn.clicked.connect(self.pick_home_point_requested.emit)
        home_btn_row.addWidget(dccpp_pick_home_btn)

        dccpp_clear_home_btn = QPushButton("✖ 清除")
        dccpp_clear_home_btn.setStyleSheet(
            "background-color: #78909C; color: white; padding: 5px; border-radius: 3px;"
        )
        dccpp_clear_home_btn.clicked.connect(self.clear_home_point_requested.emit)
        dccpp_clear_home_btn.setFixedWidth(60)
        home_btn_row.addWidget(dccpp_clear_home_btn)
        rwy_layout.addLayout(home_btn_row)

        rwy_form = QFormLayout()
        rwy_form.setSpacing(4)

        # 起飛方向（跑道方向）
        dccpp_takeoff_row = QHBoxLayout()
        self.dccpp_takeoff_bearing_spin = QDoubleSpinBox()
        self.dccpp_takeoff_bearing_spin.setRange(0.0, 359.9)
        self.dccpp_takeoff_bearing_spin.setValue(0.0)
        self.dccpp_takeoff_bearing_spin.setSuffix("°")
        self.dccpp_takeoff_bearing_spin.setDecimals(1)
        self.dccpp_takeoff_bearing_spin.setToolTip(
            "跑道起飛方向（度，0=北，90=東，180=南，270=西）\n"
            "桃園機場 05L/23R 跑道方向約 050°/230°"
        )
        self.dccpp_takeoff_compass_label = QLabel("北")
        self.dccpp_takeoff_compass_label.setStyleSheet("color: #2196F3; font-size: 10px; min-width: 30px;")
        self.dccpp_takeoff_bearing_spin.valueChanged.connect(
            lambda v: self.dccpp_takeoff_compass_label.setText(self._bearing_to_compass(v))
        )
        dccpp_takeoff_row.addWidget(self.dccpp_takeoff_bearing_spin)
        dccpp_takeoff_row.addWidget(self.dccpp_takeoff_compass_label)
        rwy_form.addRow("起飛方向:", dccpp_takeoff_row)

        # 跑道長度
        self.dccpp_runway_length_spin = QDoubleSpinBox()
        self.dccpp_runway_length_spin.setRange(20.0, 4000.0)
        self.dccpp_runway_length_spin.setValue(50.0)
        self.dccpp_runway_length_spin.setSuffix(" m")
        self.dccpp_runway_length_spin.setDecimals(0)
        self.dccpp_runway_length_spin.setToolTip(
            "起飛滑跑距離（公尺）\n"
            "桃園機場跑道長 3660m，測試時視需要設定"
        )
        rwy_form.addRow("跑道長度:", self.dccpp_runway_length_spin)

        # 降落進場方向
        dccpp_landing_row = QHBoxLayout()
        self.dccpp_landing_bearing_spin = QDoubleSpinBox()
        self.dccpp_landing_bearing_spin.setRange(0.0, 359.9)
        self.dccpp_landing_bearing_spin.setValue(180.0)
        self.dccpp_landing_bearing_spin.setSuffix("°")
        self.dccpp_landing_bearing_spin.setDecimals(1)
        self.dccpp_landing_bearing_spin.setToolTip(
            "降落進場方向（度）\n"
            "通常與起飛方向相反或逆風方向"
        )
        self.dccpp_landing_compass_label = QLabel("南")
        self.dccpp_landing_compass_label.setStyleSheet("color: #2196F3; font-size: 10px; min-width: 30px;")
        self.dccpp_landing_bearing_spin.valueChanged.connect(
            lambda v: self.dccpp_landing_compass_label.setText(self._bearing_to_compass(v))
        )
        dccpp_landing_row.addWidget(self.dccpp_landing_bearing_spin)
        dccpp_landing_row.addWidget(self.dccpp_landing_compass_label)
        rwy_form.addRow("進場方向:", dccpp_landing_row)

        # 五邊飛行高度
        self.dccpp_pattern_alt_spin = QDoubleSpinBox()
        self.dccpp_pattern_alt_spin.setRange(30.0, 500.0)
        self.dccpp_pattern_alt_spin.setValue(80.0)
        self.dccpp_pattern_alt_spin.setSuffix(" m")
        self.dccpp_pattern_alt_spin.setDecimals(0)
        self.dccpp_pattern_alt_spin.setToolTip("五邊進場飛行高度 AGL")
        rwy_form.addRow("五邊高度:", self.dccpp_pattern_alt_spin)

        # 降落滑行距離（觸地後煞車滾停）
        self.dccpp_landing_rollout_spin = QDoubleSpinBox()
        self.dccpp_landing_rollout_spin.setRange(0.0, 9999.0)
        self.dccpp_landing_rollout_spin.setValue(0.0)
        self.dccpp_landing_rollout_spin.setSuffix(" m")
        self.dccpp_landing_rollout_spin.setDecimals(0)
        self.dccpp_landing_rollout_spin.setSingleStep(10.0)
        self.dccpp_landing_rollout_spin.setToolTip(
            "觸地後滑行＋煞車所需距離（公尺）\n"
            "觸地點會提前此距離，確保飛機減速停止時\n"
            "不超過跑道末端（Home 點）\n"
            "設 0 = 不偏移，觸地點在 Home"
        )
        rwy_form.addRow("降落滑行距:", self.dccpp_landing_rollout_spin)

        # 起飛點間距（多機時沿跑道方向排列的間距）
        self.dccpp_takeoff_spacing_spin = QDoubleSpinBox()
        self.dccpp_takeoff_spacing_spin.setRange(5.0, 500.0)
        self.dccpp_takeoff_spacing_spin.setValue(50.0)
        self.dccpp_takeoff_spacing_spin.setSuffix(" m")
        self.dccpp_takeoff_spacing_spin.setDecimals(0)
        self.dccpp_takeoff_spacing_spin.setToolTip(
            "多機起飛時，各 UAV 起飛點沿跑道方向的間距（公尺）\n"
            "避免多架 UAV 起飛點重疊在一起"
        )
        rwy_form.addRow("起飛點間距:", self.dccpp_takeoff_spacing_spin)

        rwy_layout.addLayout(rwy_form)
        layout.addWidget(self.dccpp_runway_group)
        # 初始狀態：多旋翼時隱藏跑道設定
        self.dccpp_runway_group.setVisible(False)

        # ── 進階設定（論文深度整合）──────────────────────
        adv_form = QFormLayout()
        adv_form.setContentsMargins(0, 4, 0, 4)

        # 感測器掛載角
        self.dccpp_mounting_angle_spin = QDoubleSpinBox()
        self.dccpp_mounting_angle_spin.setRange(0.0, 85.0)
        self.dccpp_mounting_angle_spin.setDecimals(1)
        self.dccpp_mounting_angle_spin.setValue(0.0)
        self.dccpp_mounting_angle_spin.setSuffix(" °")
        self.dccpp_mounting_angle_spin.setToolTip(
            "感測器掛載角 α_m（度）\n"
            "0° = 垂直朝下（矩形 FOV）\n"
            "> 0° = 前傾安裝（梯形 FOV，論文 Eq. 2-3）"
        )
        adv_form.addRow("感測器掛載角:", self.dccpp_mounting_angle_spin)

        # 協調模式
        self.dccpp_coord_combo = QComboBox()
        self.dccpp_coord_combo.addItem("非協調", "uncoordinated")
        self.dccpp_coord_combo.addItem("協調（同步進入）", "coordinated")
        self.dccpp_coord_combo.setToolTip(
            "非協調：各 UAV 獨立規劃\n"
            "協調：同步進入覆蓋區域，較慢 UAV 插入等待盤旋"
        )
        adv_form.addRow("協調模式:", self.dccpp_coord_combo)

        # DEM 地形檔案
        dem_row = QHBoxLayout()
        self.dccpp_dem_path_label = QLabel("（無）")
        self.dccpp_dem_path_label.setStyleSheet("color: #888; font-size: 11px;")
        self._dccpp_dem_path = ""
        dem_btn = QPushButton("載入 DEM")
        dem_btn.setFixedWidth(80)
        dem_btn.setToolTip(
            "載入 DEM 地形檔案（GeoTIFF 或 .npy）\n"
            "用於 3D 高度規劃與地形迴避（論文 Eq. 4）"
        )
        dem_btn.clicked.connect(self._on_dccpp_load_dem)
        dem_row.addWidget(self.dccpp_dem_path_label, 1)
        dem_row.addWidget(dem_btn)
        adv_form.addRow("DEM 地形:", dem_row)

        layout.addLayout(adv_form)

        # ── 避撞設定 ──────────────────────────────────────
        collision_group = QGroupBox("避撞模擬")
        collision_group.setStyleSheet(
            "QGroupBox { font-weight: bold; border: 1px solid #555; "
            "border-radius: 4px; margin-top: 8px; padding-top: 16px; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 8px; }"
        )
        collision_layout = QFormLayout()
        collision_layout.setSpacing(4)

        self.dccpp_collision_check = QCheckBox("啟用多機避撞偵測")
        self.dccpp_collision_check.setChecked(False)
        self.dccpp_collision_check.setToolTip(
            "路徑規劃後模擬所有 UAV 同時飛行，\n"
            "偵測距離過近的衝突並自動解衝突"
        )
        collision_layout.addRow(self.dccpp_collision_check)

        self.dccpp_min_separation_spin = QDoubleSpinBox()
        self.dccpp_min_separation_spin.setRange(10.0, 1000.0)
        self.dccpp_min_separation_spin.setValue(100.0)
        self.dccpp_min_separation_spin.setSuffix(" m")
        self.dccpp_min_separation_spin.setDecimals(0)
        self.dccpp_min_separation_spin.setToolTip("兩機最小安全距離（公尺）")
        collision_layout.addRow("最小安全距離:", self.dccpp_min_separation_spin)

        self.dccpp_alt_offset_spin = QDoubleSpinBox()
        self.dccpp_alt_offset_spin.setRange(10.0, 200.0)
        self.dccpp_alt_offset_spin.setValue(30.0)
        self.dccpp_alt_offset_spin.setSuffix(" m")
        self.dccpp_alt_offset_spin.setDecimals(0)
        self.dccpp_alt_offset_spin.setToolTip(
            "衝突時其中一架 UAV 的高度偏移量\n"
            "例如 30m = 衝突區段升高 30m 通過"
        )
        collision_layout.addRow("高度錯開量:", self.dccpp_alt_offset_spin)

        self.dccpp_avoid_strategy_combo = QComboBox()
        self.dccpp_avoid_strategy_combo.addItem("高度錯開", "altitude")
        self.dccpp_avoid_strategy_combo.addItem("時間延遲（盤旋等待）", "temporal")
        self.dccpp_avoid_strategy_combo.setToolTip(
            "高度錯開：衝突區段升高/降低通過\n"
            "時間延遲：插入盤旋等待航點，錯開通過時間"
        )
        collision_layout.addRow("避撞策略:", self.dccpp_avoid_strategy_combo)

        collision_group.setLayout(collision_layout)
        layout.addWidget(collision_group)

        # 刪除上一點按鈕
        del_corner_btn = QPushButton("⌫ 刪除上一個邊界點")
        del_corner_btn.setStyleSheet(
            "background-color: #E53935; color: white; "
            "padding: 6px; font-size: 11px; border-radius: 3px;"
        )
        del_corner_btn.setToolTip("刪除最後一個邊界點（等同 Delete 鍵）")
        del_corner_btn.clicked.connect(self.delete_last_corner_requested.emit)
        layout.addWidget(del_corner_btn)

        gen_btn = QPushButton("DCCPP 最佳化規劃")
        gen_btn.setStyleSheet(
            "background-color: #00695C; color: white; "
            "font-weight: bold; padding: 8px; font-size: 12px;"
        )
        gen_btn.setToolTip(
            "執行完整 DCCPP 流程：\n"
            "① GreedyAllocator 分配 UAV → 各區域\n"
            "② IDPSolver 最佳化各區域作業路徑序列\n"
            "③ AltitudePlanner + GDA 高度平滑"
        )
        gen_btn.clicked.connect(self._on_dccpp_btn_clicked)
        layout.addWidget(gen_btn)

        # ── 任務匯出模式 ──────────────────────────────────────
        from PyQt6.QtWidgets import QFormLayout as _FL2
        export_group = QGroupBox("任務匯出")
        export_group.setStyleSheet(
            "QGroupBox{font-weight:bold;border:1px solid #455a64;"
            "border-radius:4px;margin-top:6px;padding-top:14px;}"
            "QGroupBox::title{color:#90caf9;}"
        )
        export_form = _FL2()
        export_form.setContentsMargins(6, 4, 6, 4)
        export_form.setSpacing(4)

        # 匯出模式下拉
        self.dccpp_export_mode_combo = QComboBox()
        self.dccpp_export_mode_combo.addItem("常規多旋翼", "copter")
        self.dccpp_export_mode_combo.addItem("常規固定翼", "plane")
        self.dccpp_export_mode_combo.addItem("4+1 VTOL 混飛模式", "vtol")
        self.dccpp_export_mode_combo.setToolTip(
            "常規多旋翼/固定翼：使用原有匯出邏輯\n"
            "4+1 VTOL 混飛模式：VTOL_TAKEOFF → FW 巡航 → MC → VTOL_LAND"
        )
        self.dccpp_export_mode_combo.currentIndexChanged.connect(
            self._on_dccpp_export_mode_changed
        )
        export_form.addRow("匯出模式:", self.dccpp_export_mode_combo)

        # VTOL 轉換安全高度
        self.vtol_transition_alt_spin = QSpinBox()
        self.vtol_transition_alt_spin.setRange(10, 500)
        self.vtol_transition_alt_spin.setValue(100)
        self.vtol_transition_alt_spin.setSuffix(" m")
        self.vtol_transition_alt_spin.setToolTip(
            "VTOL 垂直起飛爬升到此高度後觸發固定翼轉換；\n"
            "降落前在此高度轉回多旋翼模式再垂直降落"
        )
        self.vtol_transition_alt_label = QLabel("VTOL 轉換高度:")
        self.vtol_transition_alt_label.setVisible(False)
        self.vtol_transition_alt_spin.setVisible(False)
        export_form.addRow(self.vtol_transition_alt_label, self.vtol_transition_alt_spin)

        export_group.setLayout(export_form)
        layout.addWidget(export_group)

        # 匯出按鈕
        export_btn = QPushButton("匯出群飛任務")
        export_btn.setStyleSheet(
            "background-color: #4A148C; color: white; "
            "font-weight: bold; padding: 8px; font-size: 12px;"
        )
        export_btn.setToolTip(
            "將 DCCPP 最佳化結果匯出為每架 UAV 獨立的 .waypoints 檔案\n"
            "格式：QGC WPL 110（Mission Planner 相容）"
        )
        export_btn.clicked.connect(self._on_dccpp_export_clicked)
        layout.addWidget(export_btn)

        return group

    def _on_dccpp_vehicle_changed(self):
        """DCCPP 載具類型切換時，顯示/隱藏轉彎半徑與跑道設定"""
        vtype = self.dccpp_vehicle_combo.currentData()
        is_fixed = vtype == 'fixed_wing'
        is_vtol = vtype == 'vtol'
        # VTOL 巡航段也是固定翼，需要轉彎半徑
        needs_turn = is_fixed or is_vtol
        self.dccpp_turn_radius_label.setVisible(needs_turn)
        self.dccpp_turn_radius_spin.setVisible(needs_turn)
        # 自動起降：固定翼需要跑道，VTOL 垂直起降不需要
        self.dccpp_auto_landing_check.setVisible(is_fixed)
        # 跑道設定：僅固定翼 + 啟用自動起降時才顯示
        show_rwy = is_fixed and self.dccpp_auto_landing_check.isChecked()
        self.dccpp_runway_group.setVisible(show_rwy)
        # VTOL 選中時自動切換匯出模式為 VTOL
        if is_vtol and hasattr(self, 'dccpp_export_mode_combo'):
            idx = self.dccpp_export_mode_combo.findData('vtol')
            if idx >= 0:
                self.dccpp_export_mode_combo.setCurrentIndex(idx)

    def _on_dccpp_auto_landing_changed(self):
        """DCCPP 自動起降開關切換時，顯示/隱藏跑道設定"""
        is_fixed = self.dccpp_vehicle_combo.currentData() == 'fixed_wing'
        show_rwy = is_fixed and self.dccpp_auto_landing_check.isChecked()
        self.dccpp_runway_group.setVisible(show_rwy)

    def _on_dccpp_export_mode_changed(self):
        """匯出模式切換：VTOL 模式時顯示轉換高度"""
        is_vtol = self.dccpp_export_mode_combo.currentData() == 'vtol'
        self.vtol_transition_alt_label.setVisible(is_vtol)
        self.vtol_transition_alt_spin.setVisible(is_vtol)

    def _on_dccpp_export_clicked(self):
        """匯出按鈕：根據模式分派到對應的匯出流程"""
        mode = self.dccpp_export_mode_combo.currentData()
        if mode == 'vtol':
            params = {
                'transition_alt': self.vtol_transition_alt_spin.value(),
            }
            self.vtol_export_requested.emit(params)
        else:
            # 常規多旋翼 / 固定翼 → 原有匯出邏輯
            self.dccpp_export_requested.emit()

    def _on_dccpp_load_dem(self):
        """選擇 DEM 地形檔案"""
        path, _ = QFileDialog.getOpenFileName(
            self, "載入 DEM 地形檔案", "",
            "GeoTIFF (*.tif *.tiff);;Numpy (*.npy);;所有檔案 (*)"
        )
        if path:
            self._dccpp_dem_path = path
            # 顯示簡短檔名
            from pathlib import Path
            short = Path(path).name
            self.dccpp_dem_path_label.setText(short)
            self.dccpp_dem_path_label.setStyleSheet("color: #00695C; font-size: 11px;")
            self.dccpp_dem_path_label.setToolTip(path)
            self.dem_loaded.emit(path)

    def get_dccpp_params(self) -> dict:
        """取得 DCCPP 最佳化參數"""
        vehicle_type = self.dccpp_vehicle_combo.currentData()
        return {
            'vehicle_type': vehicle_type,
            'num_drones': self.dccpp_num_drones_spin.value(),
            'fov_width_m': self.dccpp_fov_spin.value(),
            'overlap_rate': self.dccpp_overlap_spin.value(),
            'altitude': self.dccpp_altitude_spin.value(),
            'speed': self.dccpp_speed_spin.value(),
            'turn_radius': (
                self.dccpp_turn_radius_spin.value()
                if vehicle_type in ('fixed_wing', 'vtol') else 0.0
            ),
            'auto_scan_angle': self.dccpp_auto_scan_check.isChecked(),
            'enable_altitude': self.dccpp_altitude_check.isChecked(),
            'auto_landing': self.dccpp_auto_landing_check.isChecked(),
            'mounting_angle_deg': self.dccpp_mounting_angle_spin.value(),
            'coordination_mode': self.dccpp_coord_combo.currentData(),
            'dem_path': self._dccpp_dem_path,
            # 跑道 / 起降參數
            'takeoff_bearing': self.dccpp_takeoff_bearing_spin.value(),
            'runway_length': self.dccpp_runway_length_spin.value(),
            'landing_bearing': self.dccpp_landing_bearing_spin.value(),
            'pattern_alt': self.dccpp_pattern_alt_spin.value(),
            'landing_rollout': self.dccpp_landing_rollout_spin.value(),
            'takeoff_spacing': self.dccpp_takeoff_spacing_spin.value(),
            # 避撞參數
            'collision_avoidance': self.dccpp_collision_check.isChecked(),
            'min_separation_m': self.dccpp_min_separation_spin.value(),
            'alt_offset_m': self.dccpp_alt_offset_spin.value(),
            'avoid_strategy': self.dccpp_avoid_strategy_combo.currentData(),
        }

    def _on_dccpp_btn_clicked(self):
        """點擊「DCCPP 最佳化規劃」按鈕"""
        params = self.get_dccpp_params()
        self.dccpp_coverage_requested.emit(params)

    # ══════════════════════════════════════════════════════════════════
    #  戰術模組 UI 面板
    # ══════════════════════════════════════════════════════════════════

    def _create_elevation_slicer_panel(self):
        """模組一：FSDM 高程切片分析面板"""
        group = QGroupBox("FSDM 高程切片分析")
        group.setStyleSheet(
            "QGroupBox{font-weight:bold;color:#4CAF50;border:1px solid #388E3C;"
            "border-radius:4px;margin-top:8px;padding-top:14px;}"
            "QGroupBox::title{subcontrol-position:top left;padding:2px 8px;}"
        )
        layout = QFormLayout(group)

        # 最低安全高度滑桿
        self._elev_min_slider = QSlider(Qt.Orientation.Horizontal)
        self._elev_min_slider.setRange(0, 5000)
        self._elev_min_slider.setValue(500)
        self._elev_min_slider.setTickInterval(100)
        self._elev_min_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self._elev_min_label = QLabel("500 m")
        self._elev_min_label.setMinimumWidth(50)
        min_row = QHBoxLayout()
        min_row.addWidget(self._elev_min_slider, 1)
        min_row.addWidget(self._elev_min_label)
        layout.addRow("最低安全高度:", min_row)

        # 最高突防高度滑桿
        self._elev_max_slider = QSlider(Qt.Orientation.Horizontal)
        self._elev_max_slider.setRange(0, 5000)
        self._elev_max_slider.setValue(1500)
        self._elev_max_slider.setTickInterval(100)
        self._elev_max_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self._elev_max_label = QLabel("1500 m")
        self._elev_max_label.setMinimumWidth(50)
        max_row = QHBoxLayout()
        max_row.addWidget(self._elev_max_slider, 1)
        max_row.addWidget(self._elev_max_label)
        layout.addRow("最高突防高度:", max_row)

        # 滑桿值更新
        self._elev_min_slider.valueChanged.connect(
            lambda v: self._elev_min_label.setText(f"{v} m")
        )
        self._elev_max_slider.valueChanged.connect(
            lambda v: self._elev_max_label.setText(f"{v} m")
        )

        # 啟用按鈕
        btn_row = QHBoxLayout()
        apply_btn = QPushButton("啟用切片分析")
        apply_btn.setStyleSheet(
            "background-color:#2E7D32;color:white;font-weight:bold;"
            "padding:6px;border-radius:3px;"
        )
        apply_btn.setToolTip(
            "將地形依設定的高度區間分色渲染\n"
            "綠色 = 盲區走廊（安全飛行區間）\n"
            "暗色 = 障礙地形（超出安全範圍）"
        )
        apply_btn.clicked.connect(self._on_elev_slicer_apply)
        btn_row.addWidget(apply_btn)

        clear_btn = QPushButton("清除切片")
        clear_btn.setStyleSheet(
            "background-color:#455A64;color:white;padding:6px;border-radius:3px;"
        )
        clear_btn.clicked.connect(self.elevation_slicer_cleared.emit)
        btn_row.addWidget(clear_btn)
        layout.addRow(btn_row)

        return group

    def _on_elev_slicer_apply(self):
        """高程切片：取得滑桿值並發送信號"""
        min_alt = float(self._elev_min_slider.value())
        max_alt = float(self._elev_max_slider.value())
        if min_alt >= max_alt:
            logger.warning("[FSDM] 最低高度必須小於最高高度")
            return
        self.elevation_slicer_changed.emit(min_alt, max_alt)

    def _create_sar_heatmap_panel(self):
        """模組二：FOV 光錐 + SAR 搜救機率熱力圖面板"""
        group = QGroupBox("SAR 搜救機率熱力圖")
        group.setStyleSheet(
            "QGroupBox{font-weight:bold;color:#FF9800;border:1px solid #F57C00;"
            "border-radius:4px;margin-top:8px;padding-top:14px;}"
            "QGroupBox::title{subcontrol-position:top left;padding:2px 8px;}"
        )
        layout = QFormLayout(group)

        # FOV 光錐開關
        self._fov_cone_check = QCheckBox("啟用 FOV 光錐")
        self._fov_cone_check.setToolTip(
            "在 UAV 下方投射視野光錐\n"
            "光錐姿態與 UAV Pitch/Roll/Yaw 同步"
        )
        self._fov_cone_check.stateChanged.connect(
            lambda s: self.fov_cone_toggle_requested.emit(s == Qt.CheckState.Checked.value)
        )
        layout.addRow(self._fov_cone_check)

        # FOV 半徑
        self._fov_radius_spin = QDoubleSpinBox()
        self._fov_radius_spin.setRange(5.0, 500.0)
        self._fov_radius_spin.setValue(50.0)
        self._fov_radius_spin.setSuffix(" m")
        self._fov_radius_spin.setDecimals(0)
        self._fov_radius_spin.setToolTip("光錐地面投影半徑（公尺）")
        layout.addRow("FOV 半徑:", self._fov_radius_spin)

        # 掃描有效寬度 W（用於機率計算）
        self._sweep_width_spin = QDoubleSpinBox()
        self._sweep_width_spin.setRange(1.0, 500.0)
        self._sweep_width_spin.setValue(50.0)
        self._sweep_width_spin.setSuffix(" m")
        self._sweep_width_spin.setDecimals(0)
        self._sweep_width_spin.setToolTip(
            "掃描有效寬度 W（公尺）\n"
            "影響探測機率：P = 1 - exp(-W × q)"
        )
        layout.addRow("掃描寬度 W:", self._sweep_width_spin)

        # 探測品質 q
        self._detect_quality_spin = QDoubleSpinBox()
        self._detect_quality_spin.setRange(0.01, 1.0)
        self._detect_quality_spin.setValue(0.8)
        self._detect_quality_spin.setSingleStep(0.05)
        self._detect_quality_spin.setDecimals(2)
        self._detect_quality_spin.setToolTip(
            "探測品質 q (0~1)\n"
            "影響單次探測機率：P = 1 - exp(-W × q)"
        )
        layout.addRow("探測品質 q:", self._detect_quality_spin)

        # 網格解析度
        self._heatmap_grid_spin = QSpinBox()
        self._heatmap_grid_spin.setRange(5, 100)
        self._heatmap_grid_spin.setValue(20)
        self._heatmap_grid_spin.setToolTip("網格行列數（越大越精細，但消耗越多）")
        layout.addRow("網格解析度:", self._heatmap_grid_spin)

        # 初始化 / 重置 / 清除按鈕列
        btn_row = QHBoxLayout()

        init_btn = QPushButton("初始化熱力圖")
        init_btn.setStyleSheet(
            "background-color:#E65100;color:white;font-weight:bold;"
            "padding:6px;border-radius:3px;"
        )
        init_btn.setToolTip(
            "在目前邊界區域內建立搜救機率網格\n"
            "紅色 = 高殘餘機率（未搜索）\n"
            "藍色 = 已充分搜索"
        )
        init_btn.clicked.connect(self._on_sar_heatmap_init)
        btn_row.addWidget(init_btn)

        reset_btn = QPushButton("重置")
        reset_btn.setStyleSheet(
            "background-color:#F57C00;color:white;padding:6px;border-radius:3px;"
        )
        reset_btn.setToolTip("保留網格但重設所有 COS 為 0")
        reset_btn.clicked.connect(self.sar_heatmap_reset_requested.emit)
        btn_row.addWidget(reset_btn)

        clear_btn = QPushButton("清除")
        clear_btn.setStyleSheet(
            "background-color:#455A64;color:white;padding:6px;border-radius:3px;"
        )
        clear_btn.clicked.connect(self.sar_heatmap_clear_requested.emit)
        btn_row.addWidget(clear_btn)

        layout.addRow(btn_row)

        return group

    def _on_sar_heatmap_init(self):
        """SAR 熱力圖初始化：發送參數"""
        params = {
            'fov_radius': self._fov_radius_spin.value(),
            'sweep_width': self._sweep_width_spin.value(),
            'quality': self._detect_quality_spin.value(),
            'grid_size': self._heatmap_grid_spin.value(),
        }
        self.sar_heatmap_init_requested.emit(params)

    def _create_radar_rcs_panel(self):
        """模組三：雷達威脅穹頂 + RCS 敏感度面板"""
        group = QGroupBox("雷達威脅穹頂 / RCS 敏感度")
        group.setStyleSheet(
            "QGroupBox{font-weight:bold;color:#F44336;border:1px solid #D32F2F;"
            "border-radius:4px;margin-top:8px;padding-top:14px;}"
            "QGroupBox::title{subcontrol-position:top left;padding:2px 8px;}"
        )
        layout = QFormLayout(group)

        # 雷達緯度
        self._radar_lat_spin = QDoubleSpinBox()
        self._radar_lat_spin.setRange(-90.0, 90.0)
        self._radar_lat_spin.setValue(23.70)
        self._radar_lat_spin.setDecimals(6)
        self._radar_lat_spin.setToolTip("敵方防空雷達位置（緯度）")
        layout.addRow("雷達緯度:", self._radar_lat_spin)

        # 雷達經度
        self._radar_lon_spin = QDoubleSpinBox()
        self._radar_lon_spin.setRange(-180.0, 180.0)
        self._radar_lon_spin.setValue(120.42)
        self._radar_lon_spin.setDecimals(6)
        self._radar_lon_spin.setToolTip("敵方防空雷達位置（經度）")
        layout.addRow("雷達經度:", self._radar_lon_spin)

        # 雷達探測半徑
        self._radar_radius_spin = QDoubleSpinBox()
        self._radar_radius_spin.setRange(100.0, 100000.0)
        self._radar_radius_spin.setValue(5000.0)
        self._radar_radius_spin.setSuffix(" m")
        self._radar_radius_spin.setDecimals(0)
        self._radar_radius_spin.setSingleStep(500.0)
        self._radar_radius_spin.setToolTip("雷達探測半徑（公尺）")
        layout.addRow("探測半徑:", self._radar_radius_spin)

        # 雷達名稱
        from PyQt6.QtWidgets import QLineEdit
        self._radar_name_edit = QLineEdit("SAM-1")
        self._radar_name_edit.setToolTip("雷達識別名稱")
        self._radar_name_edit.setMaxLength(20)
        layout.addRow("雷達名稱:", self._radar_name_edit)

        # RCS 即時渲染開關
        self._rcs_check = QCheckBox("啟用 RCS 敏感度渲染")
        self._rcs_check.setToolTip(
            "即時計算 UAV 相對雷達的 RCS 威脅等級\n"
            "紅色 = 高被偵測風險\n"
            "藍色 = 安全距離/角度"
        )
        self._rcs_check.stateChanged.connect(
            lambda s: self.rcs_toggle_requested.emit(s == Qt.CheckState.Checked.value)
        )
        layout.addRow(self._rcs_check)

        # 按鈕列
        btn_row = QHBoxLayout()

        sim_btn = QPushButton("模擬雷達威脅掃描")
        sim_btn.setStyleSheet(
            "background-color:#C62828;color:white;font-weight:bold;"
            "padding:6px;border-radius:3px;"
        )
        sim_btn.setToolTip(
            "在地圖上建立雷達威脅穹頂\n"
            "並啟動脈衝掃描動畫"
        )
        sim_btn.clicked.connect(self._on_radar_sim)
        btn_row.addWidget(sim_btn)

        clear_btn = QPushButton("清除雷達")
        clear_btn.setStyleSheet(
            "background-color:#455A64;color:white;padding:6px;border-radius:3px;"
        )
        clear_btn.clicked.connect(self.radar_clear_requested.emit)
        btn_row.addWidget(clear_btn)

        layout.addRow(btn_row)

        return group

    def _on_radar_sim(self):
        """模擬雷達威脅掃描按鈕"""
        params = {
            'lat': self._radar_lat_spin.value(),
            'lon': self._radar_lon_spin.value(),
            'radius': self._radar_radius_spin.value(),
            'name': self._radar_name_edit.text().strip() or 'Radar',
        }
        self.radar_sim_requested.emit(params)

    def get_fov_radius(self) -> float:
        """取得目前 FOV 半徑設定值"""
        return self._fov_radius_spin.value()

    def is_fov_cone_enabled(self) -> bool:
        """FOV 光錐是否啟用"""
        return self._fov_cone_check.isChecked()

    def is_rcs_enabled(self) -> bool:
        """RCS 渲染是否啟用"""
        return self._rcs_check.isChecked()

    # ─────────────────────────────────────────────────────────────────
    # 蜂群打擊控制面板
    # ─────────────────────────────────────────────────────────────────
    def _create_strike_command_panel(self) -> QGroupBox:
        """蜂群分佈式協同打擊與末端俯衝 (Swarm Distributed Strike & Terminal Dive)"""
        group = QGroupBox("UCAV 蜂群協同打擊")
        group.setStyleSheet(
            "QGroupBox{font-weight:bold;color:#F44336;border:1px solid #D32F2F;"
            "border-radius:4px;margin-top:8px;padding-top:14px;}"
            "QGroupBox::title{subcontrol-position:top left;padding:2px 8px;}"
        )
        layout = QVBoxLayout(group)
        layout.setSpacing(8)

        # ── 說明 ─────────────────────────────────────────────────────
        desc = QLabel(
            "多架固定翼巡飛彈 (Loitering Munitions) 在巡航高度集結後，"
            "依匈牙利演算法分配地面目標，四散飛行並在末端執行大角度俯衝攻擊。"
        )
        desc.setWordWrap(True)
        desc.setStyleSheet("color:#BDBDBD; font-size:10px; padding:4px;")
        layout.addWidget(desc)

        # ══════════════════════════════════════════════════════════════
        #  發射位置 (Launch Position) — 與 STOT/DTOT 時間協同正交的概念
        # ══════════════════════════════════════════════════════════════
        mode_form = QFormLayout()
        mode_form.setSpacing(4)
        self._strike_launch_mode = QComboBox()
        self._strike_launch_mode.addItems([
            '異地發射 (各 UCAV 自動分散起飛點)',
            '同地發射 (共用發射基地)',
        ])
        self._strike_launch_mode.setStyleSheet(
            "QComboBox{font-weight:bold;color:#FFB74D;padding:4px;}"
        )
        self._strike_launch_mode.setToolTip(
            "異地發射: 各 UCAV 從不同位置出發 → 圍剿目標\n"
            "同地發射: 所有 UCAV 從同一基地起飛 → 扇形散開"
        )
        self._strike_launch_mode.currentIndexChanged.connect(
            self._on_strike_launch_mode_changed
        )
        mode_form.addRow("發射位置:", self._strike_launch_mode)
        layout.addLayout(mode_form)

        # ── STOT 專用：共用發射基地（僅 STOT 模式顯示）─────────────
        self._strike_base_group = QGroupBox("STOT 共用發射基地")
        self._strike_base_group.setStyleSheet(
            "QGroupBox{color:#4CAF50;border:1px dashed #4CAF50;"
            "border-radius:3px;margin-top:6px;padding-top:12px;}"
            "QGroupBox::title{subcontrol-position:top left;padding:0 6px;}"
        )
        base_layout = QVBoxLayout(self._strike_base_group)
        base_layout.setSpacing(4)

        self._strike_mark_base_btn = QPushButton("📍 從地圖標記共用發射基地")
        self._strike_mark_base_btn.setCheckable(True)
        self._strike_mark_base_btn.setStyleSheet(
            "QPushButton{background-color:#2E7D32;color:white;font-weight:bold;"
            "padding:6px;border-radius:3px;font-size:11px;}"
            "QPushButton:hover{background-color:#388E3C;}"
            "QPushButton:checked{background-color:#E65100;}"
        )
        self._strike_mark_base_btn.setToolTip(
            "啟用後左鍵點擊 3D 地圖任一處設定共用起飛點；\n"
            "所有 UCAV 將從該點同時起飛。"
        )
        self._strike_mark_base_btn.clicked.connect(
            lambda: self.strike_mark_base_requested.emit()
        )
        base_layout.addWidget(self._strike_mark_base_btn)

        self._strike_base_label = QLabel("未標記 — 必須先設定基地才能 EXECUTE")
        self._strike_base_label.setStyleSheet(
            "color:#FF5252;font-size:10px;padding:2px 4px;"
        )
        self._strike_base_label.setWordWrap(True)
        base_layout.addWidget(self._strike_base_label)

        layout.addWidget(self._strike_base_group)
        self._strike_base_group.setVisible(False)  # 預設 DTOT 模式隱藏

        # ── 標記目標按鈕 ─────────────────────────────────────────────
        self._strike_mark_btn = QPushButton("🎯 標記多重打擊目標 (Mark Targets)")
        self._strike_mark_btn.setStyleSheet(
            "QPushButton{background-color:#1565C0;color:white;font-weight:bold;"
            "padding:8px;border-radius:4px;font-size:12px;}"
            "QPushButton:hover{background-color:#1976D2;}"
            "QPushButton:checked{background-color:#E65100;}"
        )
        self._strike_mark_btn.setCheckable(True)
        self._strike_mark_btn.setToolTip(
            "啟用後在 3D 地圖上左鍵點擊新增敵方地面目標\n"
            "再次點擊結束標記模式"
        )
        self._strike_mark_btn.clicked.connect(
            lambda checked: self.strike_mark_targets_requested.emit()
        )
        layout.addWidget(self._strike_mark_btn)

        # ── 目標計數 + 自動生成提示 ─────────────────────────────────
        self._strike_target_count = QLabel("已標記目標: 0 個")
        self._strike_target_count.setStyleSheet(
            "color:#FF5252; font-weight:bold; font-size:11px; padding:2px 4px;"
        )
        layout.addWidget(self._strike_target_count)

        auto_hint = QLabel(
            "N 個目標 = 自動生成 N 架 UCAV\n"
            "每架 UCAV 從不同方位角進場、不同高度巡航\n"
            "航線完全錯開，不會交叉"
        )
        auto_hint.setWordWrap(True)
        auto_hint.setStyleSheet(
            "background-color:rgba(33,150,243,0.08); color:#64B5F6;"
            "padding:6px; border-radius:4px; font-size:10px;"
        )
        layout.addWidget(auto_hint)

        # ── 分隔線 ──────────────────────────────────────────────────
        sep1 = QFrame()
        sep1.setFrameShape(QFrame.Shape.HLine)
        sep1.setStyleSheet("color:#424242;")
        layout.addWidget(sep1)

        # ── 參數區域 ─────────────────────────────────────────────────
        param_form = QFormLayout()
        param_form.setSpacing(6)

        # 巡航高度（基準）
        self._strike_cruise_alt = QDoubleSpinBox()
        self._strike_cruise_alt.setRange(50.0, 5000.0)
        self._strike_cruise_alt.setValue(500.0)
        self._strike_cruise_alt.setSuffix(" m")
        self._strike_cruise_alt.setDecimals(0)
        self._strike_cruise_alt.setToolTip(
            "第 1 架 UCAV 的基準巡航高度\n"
            "後續各機依「高度錯層」遞增"
        )
        param_form.addRow("基準巡航高度:", self._strike_cruise_alt)

        # 巡航速度
        self._strike_cruise_speed = QDoubleSpinBox()
        self._strike_cruise_speed.setRange(10.0, 300.0)
        self._strike_cruise_speed.setValue(60.0)
        self._strike_cruise_speed.setSuffix(" m/s")
        self._strike_cruise_speed.setDecimals(1)
        self._strike_cruise_speed.setToolTip("UCAV 巡航速度")
        param_form.addRow("巡航速度:", self._strike_cruise_speed)

        # 高度錯層間距
        self._strike_alt_step = QDoubleSpinBox()
        self._strike_alt_step.setRange(0.0, 200.0)
        self._strike_alt_step.setValue(30.0)
        self._strike_alt_step.setSuffix(" m")
        self._strike_alt_step.setDecimals(0)
        self._strike_alt_step.setToolTip(
            "各 UCAV 巡航高度的遞增間距\n"
            "第 i 架 UCAV 高度 = 基準高度 + i × 錯層間距\n"
            "例：500, 530, 560, 590 m ..."
        )
        self._strike_alt_step.setStyleSheet(
            "QDoubleSpinBox{font-weight:bold;color:#42A5F5;}"
        )
        param_form.addRow("高度錯層間距:", self._strike_alt_step)

        # 高度預覽標籤
        self._strike_alt_preview = QLabel("")
        self._strike_alt_preview.setStyleSheet(
            "color:#81D4FA; font-size:10px; padding:2px 4px;"
        )
        self._strike_alt_preview.setWordWrap(True)
        param_form.addRow("", self._strike_alt_preview)

        # 高度預覽自動更新
        self._strike_cruise_alt.valueChanged.connect(self._update_strike_alt_preview)
        self._strike_alt_step.valueChanged.connect(self._update_strike_alt_preview)

        # 最大俯衝角
        self._strike_max_dive_angle = QDoubleSpinBox()
        self._strike_max_dive_angle.setRange(10.0, 89.0)
        self._strike_max_dive_angle.setValue(45.0)
        self._strike_max_dive_angle.setSuffix(" °")
        self._strike_max_dive_angle.setDecimals(1)
        self._strike_max_dive_angle.setToolTip(
            "固定翼末端俯衝時的物理極限角度\n"
            "角度越大，俯衝越陡峭（最大 89°）"
        )
        self._strike_max_dive_angle.setStyleSheet(
            "QDoubleSpinBox{font-weight:bold;color:#FF6D00;}"
        )
        param_form.addRow("最大俯衝角 (θ_max):", self._strike_max_dive_angle)

        # 俯衝起始距離
        self._strike_dive_dist = QDoubleSpinBox()
        self._strike_dive_dist.setRange(100.0, 5000.0)
        self._strike_dive_dist.setValue(800.0)
        self._strike_dive_dist.setSuffix(" m")
        self._strike_dive_dist.setDecimals(0)
        self._strike_dive_dist.setToolTip(
            "距離目標多遠時開始從巡航高度轉為俯衝姿態\n"
            "若此距離不足以滿足最大俯衝角，系統會自動後推"
        )
        self._strike_dive_dist.setStyleSheet(
            "QDoubleSpinBox{font-weight:bold;color:#FF6D00;}"
        )
        param_form.addRow("俯衝起始距離:", self._strike_dive_dist)

        # 動畫速度
        self._strike_anim_speed = QDoubleSpinBox()
        self._strike_anim_speed.setRange(0.5, 20.0)
        self._strike_anim_speed.setValue(3.0)
        self._strike_anim_speed.setSuffix(" x")
        self._strike_anim_speed.setDecimals(1)
        self._strike_anim_speed.setToolTip("動畫播放速度倍率")
        param_form.addRow("動畫速度:", self._strike_anim_speed)

        layout.addLayout(param_form)

        # ── 分隔線 ──────────────────────────────────────────────────
        sep2 = QFrame()
        sep2.setFrameShape(QFrame.Shape.HLine)
        sep2.setStyleSheet("color:#424242;")
        layout.addWidget(sep2)

        # ── 執行蜂群打擊按鈕 ─────────────────────────────────────────
        self._strike_execute_btn = QPushButton("EXECUTE SWARM STRIKE")
        self._strike_execute_btn.setStyleSheet(
            "QPushButton{"
            "  background-color:#D32F2F;color:white;font-weight:bold;"
            "  font-size:14px;padding:12px;border-radius:6px;"
            "  border:2px solid #B71C1C;"
            "  letter-spacing:2px;"
            "}"
            "QPushButton:hover{background-color:#F44336;border-color:#D32F2F;}"
            "QPushButton:pressed{background-color:#B71C1C;}"
            "QPushButton:disabled{background-color:#616161;border-color:#424242;color:#9E9E9E;}"
        )
        self._strike_execute_btn.setToolTip(
            "觸發後台演算法進行 UAV-目標分配\n"
            "並在 3D 地圖上播放俯衝攻擊動畫"
        )
        self._strike_execute_btn.clicked.connect(self._on_strike_execute)
        layout.addWidget(self._strike_execute_btn)

        # ── 匯出打擊任務按鈕 (DCCPP 風格，產生每機獨立 QGC WPL) ──────
        self._strike_export_btn = QPushButton("💾 匯出打擊任務 (QGC WPL)")
        self._strike_export_btn.setStyleSheet(
            "QPushButton{background-color:#1B5E20;color:white;padding:8px;"
            "font-weight:bold;border-radius:4px;}"
            "QPushButton:hover{background-color:#2E7D32;}"
            "QPushButton:disabled{background-color:#555;color:#999;}"
        )
        self._strike_export_btn.setToolTip(
            "將規劃完成的蜂群打擊路徑匯出為 QGC WPL 110 航點檔案\n"
            "每架 UCAV 一個 .waypoints，附任務簡報 TXT\n"
            "包含：DO_SET_HOME / NAV_TAKEOFF / 巡航/俯衝 NAV_WAYPOINT"
        )
        self._strike_export_btn.setEnabled(False)
        self._strike_export_btn.clicked.connect(self.strike_export_requested.emit)
        layout.addWidget(self._strike_export_btn)

        # ══════════════════════════════════════════════════════════════
        #  上傳至 SITL (每架 UCAV 綁定一條 SITL link)
        # ══════════════════════════════════════════════════════════════
        sitl_group = QGroupBox("蜂群打擊 → SITL 模擬")
        sitl_group.setStyleSheet(
            "QGroupBox{color:#00BCD4;border:1px dashed #00BCD4;"
            "border-radius:3px;margin-top:8px;padding-top:12px;}"
            "QGroupBox::title{subcontrol-position:top left;padding:0 6px;}"
        )
        sitl_layout = QVBoxLayout(sitl_group)
        sitl_layout.setSpacing(6)

        sitl_hint = QLabel(
            "需先在「🛰 SITL」分頁啟動 N 台 ArduPlane 實例，\n"
            "每架 UCAV 會依 uav_id 綁定對應的 SITL link。"
        )
        sitl_hint.setWordWrap(True)
        sitl_hint.setStyleSheet("color:#80DEEA;font-size:10px;padding:2px 4px;")
        sitl_layout.addWidget(sitl_hint)

        # DTOT 空速核取方塊 — 勾選則用各機協同空速，否則統一用 cruise_speed
        self._strike_sitl_use_dtot = QCheckBox(
            "使用 DTOT 協同空速 (各機專屬 V_i)"
        )
        self._strike_sitl_use_dtot.setChecked(True)
        self._strike_sitl_use_dtot.setToolTip(
            "勾選：上傳時每架 UCAV 插入自己的 DO_CHANGE_SPEED(178)，\n"
            "      達成同秒同時命中 (Time On Target)\n"
            "未勾：所有 UCAV 使用統一 cruise_speed (傳統打擊)"
        )
        self._strike_sitl_use_dtot.setStyleSheet("color:#B2EBF2;font-size:11px;")
        sitl_layout.addWidget(self._strike_sitl_use_dtot)

        self._strike_sitl_upload_btn = QPushButton("🚀 上傳蜂群打擊任務至 SITL")
        self._strike_sitl_upload_btn.setStyleSheet(
            "QPushButton{background-color:#006064;color:white;font-weight:bold;"
            "padding:10px;border-radius:4px;font-size:12px;"
            "border:2px solid #00838F;}"
            "QPushButton:hover{background-color:#00838F;border-color:#00ACC1;}"
            "QPushButton:pressed{background-color:#004D40;}"
            "QPushButton:disabled{background-color:#555;color:#999;border-color:#444;}"
        )
        self._strike_sitl_upload_btn.setToolTip(
            "將蜂群打擊任務上傳到所有已連線的 SITL 實例\n"
            "每架 UCAV 獨立路徑：DO_CHANGE_SPEED → NAV_TAKEOFF →\n"
            "  (可選) NAV_LOITER_TIME 補時 → 巡航 NAV_WAYPOINT → 俯衝 NAV_WAYPOINT\n"
            "（需先完成 EXECUTE SWARM STRIKE 且已有 SITL 連線）"
        )
        self._strike_sitl_upload_btn.setEnabled(False)
        self._strike_sitl_upload_btn.clicked.connect(
            lambda: self.strike_sitl_upload_requested.emit(
                self._strike_sitl_use_dtot.isChecked()
            )
        )
        sitl_layout.addWidget(self._strike_sitl_upload_btn)

        layout.addWidget(sitl_group)

        # ══════════════════════════════════════════════════════════════
        #  動態切換：DCCPP → 蜂群打擊 (ReconToStrikeManager)
        # ══════════════════════════════════════════════════════════════
        recon_group = QGroupBox("⚡ 動態偵打切換 (DCCPP → Strike)")
        recon_group.setStyleSheet(
            "QGroupBox{color:#FFD54F;border:1px dashed #FFD54F;"
            "border-radius:3px;margin-top:8px;padding-top:12px;}"
            "QGroupBox::title{subcontrol-position:top left;padding:0 6px;}"
        )
        recon_layout = QVBoxLayout(recon_group)
        recon_layout.setSpacing(6)

        recon_hint = QLabel(
            "情境：DCCPP 覆蓋掃描中偵測到目標 → 動態重構為蜂群打擊\n"
            "需求：(1) 已完成 DCCPP 規劃  (2) 已標記至少 1 個打擊目標\n"
            "流程：選 N 架最近的 UAV → 平滑銜接 → IAPF 避障 → STOT 匯聚"
        )
        recon_hint.setWordWrap(True)
        recon_hint.setStyleSheet("color:#FFE082;font-size:10px;padding:2px 4px;")
        recon_layout.addWidget(recon_hint)

        recon_form = QFormLayout()
        recon_form.setSpacing(4)

        # 聯盟規模
        self._strike_coalition_size = QSpinBox()
        self._strike_coalition_size.setRange(1, 12)
        self._strike_coalition_size.setValue(3)
        self._strike_coalition_size.setSuffix(" 架")
        self._strike_coalition_size.setToolTip(
            "從 DCCPP 掃描機群中挑選幾架轉打擊\n"
            "其餘 UAV 繼續執行 DCCPP 覆蓋任務"
        )
        recon_form.addRow("打擊聯盟規模:", self._strike_coalition_size)

        # IAPF 安全距離
        self._strike_iapf_dist = QDoubleSpinBox()
        self._strike_iapf_dist.setRange(100.0, 5000.0)
        self._strike_iapf_dist.setValue(1500.0)
        self._strike_iapf_dist.setSuffix(" m")
        self._strike_iapf_dist.setDecimals(0)
        self._strike_iapf_dist.setToolTip(
            "3D IAPF 避障觸發距離\n"
            "兩機 3D 距離 < 此值啟動切線斥力"
        )
        recon_form.addRow("IAPF 安全距離:", self._strike_iapf_dist)

        recon_layout.addLayout(recon_form)

        self._strike_recon_btn = QPushButton("⚡ 觸發偵測事件 → 動態切換打擊")
        self._strike_recon_btn.setStyleSheet(
            "QPushButton{background-color:#F57F17;color:white;font-weight:bold;"
            "padding:10px;border-radius:4px;font-size:12px;"
            "border:2px solid #FF6F00;}"
            "QPushButton:hover{background-color:#FF8F00;border-color:#FF6F00;}"
            "QPushButton:disabled{background-color:#555;color:#999;border-color:#444;}"
        )
        self._strike_recon_btn.setToolTip(
            "模擬 DCCPP 掃描中 UAV 偵測到目標的事件\n"
            "系統自動：篩選聯盟 → 平滑銜接 → IAPF 避障 → 上傳 SITL\n"
            "（需先執行 DCCPP + 標記至少 1 個目標）"
        )
        self._strike_recon_btn.clicked.connect(self._on_strike_recon_trigger)
        recon_layout.addWidget(self._strike_recon_btn)

        layout.addWidget(recon_group)

        # ══════════════════════════════════════════════════════════════
        #  VTOL 模式開關
        # ══════════════════════════════════════════════════════════════
        vtol_group = QGroupBox("🛩 VTOL 模式 (垂直起降)")
        vtol_group.setStyleSheet(
            "QGroupBox{color:#00ACC1;border:1px dashed #00ACC1;"
            "border-radius:3px;margin-top:8px;padding-top:12px;}"
            "QGroupBox::title{subcontrol-position:top left;padding:0 6px;}"
        )
        vtol_layout = QVBoxLayout(vtol_group)
        vtol_layout.setSpacing(4)

        self._strike_vtol_enabled = QCheckBox(
            "啟用 VTOL 蜂群打擊 (NAV_VTOL_TAKEOFF + Phase 2/3)"
        )
        self._strike_vtol_enabled.setChecked(False)
        self._strike_vtol_enabled.setStyleSheet("color:#80DEEA;font-size:11px;")
        self._strike_vtol_enabled.setToolTip(
            "勾選後：\n"
            "  - 起飛改用 NAV_VTOL_TAKEOFF (垂直) + DO_VTOL_TRANSITION\n"
            "  - Phase 2 巡航速度 (40-60 kts)\n"
            "  - 距目標 2 km 切換 Phase 3 末端衝刺 (80-100 kts)\n"
            "  - 可選觸發 IMAGE_START_CAPTURE AI 尋標\n"
            "  - CEP 多機散佈涵蓋誤差圓"
        )
        self._strike_vtol_enabled.toggled.connect(self._on_strike_vtol_toggled)
        vtol_layout.addWidget(self._strike_vtol_enabled)

        vtol_form = QFormLayout()
        vtol_form.setSpacing(4)

        self._strike_vtol_cruise_kts = QDoubleSpinBox()
        self._strike_vtol_cruise_kts.setRange(30.0, 80.0)
        self._strike_vtol_cruise_kts.setValue(50.0)
        self._strike_vtol_cruise_kts.setSuffix(" kts")
        self._strike_vtol_cruise_kts.setDecimals(0)
        self._strike_vtol_cruise_kts.setToolTip("Phase 2 巡航空速 (建議 40-60 kts)")
        self._strike_vtol_cruise_kts.setEnabled(False)
        vtol_form.addRow("Phase 2 巡航:", self._strike_vtol_cruise_kts)

        self._strike_vtol_terminal_kts = QDoubleSpinBox()
        self._strike_vtol_terminal_kts.setRange(60.0, 150.0)
        self._strike_vtol_terminal_kts.setValue(90.0)
        self._strike_vtol_terminal_kts.setSuffix(" kts")
        self._strike_vtol_terminal_kts.setDecimals(0)
        self._strike_vtol_terminal_kts.setToolTip("Phase 3 末端衝刺 (建議 80-100 kts)")
        self._strike_vtol_terminal_kts.setEnabled(False)
        vtol_form.addRow("Phase 3 末端:", self._strike_vtol_terminal_kts)

        self._strike_vtol_boundary_m = QDoubleSpinBox()
        self._strike_vtol_boundary_m.setRange(500.0, 10000.0)
        self._strike_vtol_boundary_m.setValue(2000.0)
        self._strike_vtol_boundary_m.setSuffix(" m")
        self._strike_vtol_boundary_m.setDecimals(0)
        self._strike_vtol_boundary_m.setToolTip("Phase 3 切換距離 (預設 2 km)")
        self._strike_vtol_boundary_m.setEnabled(False)
        vtol_form.addRow("邊界距離:", self._strike_vtol_boundary_m)

        self._strike_vtol_cep_m = QDoubleSpinBox()
        self._strike_vtol_cep_m.setRange(1.0, 100.0)
        self._strike_vtol_cep_m.setValue(12.0)
        self._strike_vtol_cep_m.setSuffix(" m")
        self._strike_vtol_cep_m.setDecimals(1)
        self._strike_vtol_cep_m.setToolTip("CEP 末端分佈半徑 (覆蓋目標誤差圓)")
        self._strike_vtol_cep_m.setEnabled(False)
        vtol_form.addRow("CEP 散佈:", self._strike_vtol_cep_m)

        self._strike_vtol_ai_seeker = QCheckBox("Phase 3 觸發 AI 尋標 (IMAGE_START_CAPTURE)")
        self._strike_vtol_ai_seeker.setChecked(True)
        self._strike_vtol_ai_seeker.setEnabled(False)
        self._strike_vtol_ai_seeker.setStyleSheet("color:#80DEEA;font-size:10px;")
        vtol_form.addRow("", self._strike_vtol_ai_seeker)

        vtol_layout.addLayout(vtol_form)
        layout.addWidget(vtol_group)

        # ══════════════════════════════════════════════════════════════
        #  時間協同 (Time Coordination) — STOT 同時命中 / DTOT 間隔命中
        # ══════════════════════════════════════════════════════════════
        sep_dtot = QFrame()
        sep_dtot.setFrameShape(QFrame.Shape.HLine)
        sep_dtot.setStyleSheet("color:#FF6F00;")
        layout.addWidget(sep_dtot)

        dtot_header = QLabel("時間協同 (Time On Target)")
        dtot_header.setStyleSheet(
            "font-weight:bold;color:#FF6F00;font-size:12px;padding:4px 0;"
        )
        layout.addWidget(dtot_header)

        dtot_desc = QLabel(
            "STOT (Simultaneous): 全體 UCAV 同一秒命中 (飽和攻擊)\n"
            "DTOT (Distributed): slot k 於 T+k·Δ 秒命中 (波次打擊)\n\n"
            "兩種模式皆自動反推各機所需空速；\n"
            "V_req < V_stall 時強制插入 Loiter 盤旋補時。"
        )
        dtot_desc.setWordWrap(True)
        dtot_desc.setStyleSheet(
            "color:#BDBDBD;font-size:10px;padding:2px 4px;"
        )
        layout.addWidget(dtot_desc)

        dtot_form = QFormLayout()
        dtot_form.setSpacing(6)

        # ── 時間協同模式選擇 ───────────────────────────────────
        self._strike_timing_mode = QComboBox()
        self._strike_timing_mode.addItems([
            'STOT — 同時命中',
            'DTOT — 間隔命中',
        ])
        self._strike_timing_mode.setStyleSheet(
            "QComboBox{font-weight:bold;color:#FF8A65;padding:4px;}"
        )
        self._strike_timing_mode.setToolTip(
            "STOT: 全體 UCAV 同一秒突破末端/命中目標\n"
            "DTOT: 依 slot 序依序命中 (間隔由下方 interval 決定)"
        )
        self._strike_timing_mode.currentIndexChanged.connect(
            self._on_strike_timing_mode_changed
        )
        dtot_form.addRow("時間協同:", self._strike_timing_mode)

        # ── DTOT 間隔時間 (僅 DTOT 模式顯示) ────────────────────
        self._strike_interval_sec = QDoubleSpinBox()
        self._strike_interval_sec.setRange(1.0, 300.0)
        self._strike_interval_sec.setValue(15.0)
        self._strike_interval_sec.setSuffix(" s")
        self._strike_interval_sec.setDecimals(1)
        self._strike_interval_sec.setSingleStep(1.0)
        self._strike_interval_sec.setToolTip(
            "DTOT 間隔秒數 Δ：slot 0 命中時刻為 T，\n"
            "slot 1 於 T+Δ，slot 2 於 T+2Δ ... 依序命中\n"
            "（STOT 模式此欄位不生效）"
        )
        self._strike_interval_sec.setStyleSheet(
            "QDoubleSpinBox{font-weight:bold;color:#FFAB91;}"
        )
        self._strike_interval_label = QLabel("間隔 Δ:")
        dtot_form.addRow(self._strike_interval_label, self._strike_interval_sec)
        # 預設 STOT → 隱藏 interval
        self._strike_interval_sec.setEnabled(False)

        # 最大空速
        self._strike_max_speed = QDoubleSpinBox()
        self._strike_max_speed.setRange(30.0, 300.0)
        self._strike_max_speed.setValue(85.0)
        self._strike_max_speed.setSuffix(" m/s")
        self._strike_max_speed.setDecimals(1)
        self._strike_max_speed.setToolTip(
            "飛控允許最大空速 (ARSPD_FBW_MAX)\n"
            "超過此值的 TOT 計算為不可行"
        )
        dtot_form.addRow("最大空速 (V_max):", self._strike_max_speed)

        # 失速速度
        self._strike_stall_speed = QDoubleSpinBox()
        self._strike_stall_speed.setRange(10.0, 100.0)
        self._strike_stall_speed.setValue(25.0)
        self._strike_stall_speed.setSuffix(" m/s")
        self._strike_stall_speed.setDecimals(1)
        self._strike_stall_speed.setToolTip(
            "失速速度下限 (ARSPD_FBW_MIN)\n"
            "低於此值 = 需插入 Loiter 盤旋補時"
        )
        dtot_form.addRow("失速速度 (V_stall):", self._strike_stall_speed)

        # 最小轉彎半徑
        self._strike_turn_radius = QDoubleSpinBox()
        self._strike_turn_radius.setRange(30.0, 1000.0)
        self._strike_turn_radius.setValue(150.0)
        self._strike_turn_radius.setSuffix(" m")
        self._strike_turn_radius.setDecimals(0)
        self._strike_turn_radius.setToolTip(
            "固定翼最小轉彎半徑 R_min\n"
            "用於 S-turn/Loiter 補時盤旋與巡航段 Dubins 曲線"
        )
        dtot_form.addRow("最小轉彎半徑:", self._strike_turn_radius)

        # TOT 結果預覽
        self._strike_dtot_preview = QLabel("")
        self._strike_dtot_preview.setWordWrap(True)
        self._strike_dtot_preview.setStyleSheet(
            "color:#FFD54F;font-size:10px;padding:2px 4px;"
        )
        dtot_form.addRow("", self._strike_dtot_preview)

        layout.addLayout(dtot_form)

        # DTOT 匯出按鈕
        self._strike_dtot_export_btn = QPushButton("DTOT/STOT 飽和攻擊匯出 (QGC WPL)")
        self._strike_dtot_export_btn.setStyleSheet(
            "QPushButton{background-color:#E65100;color:white;padding:8px;"
            "font-weight:bold;border-radius:4px;font-size:11px;}"
            "QPushButton:hover{background-color:#F57C00;}"
            "QPushButton:disabled{background-color:#555;color:#999;}"
        )
        self._strike_dtot_export_btn.setToolTip(
            "執行 DTOT/STOT 時空協同演算 → 反推各機空速 →\n"
            "匯出含 DO_CHANGE_SPEED(178) 的 QGC WPL 航點檔\n"
            "（需先完成 EXECUTE SWARM STRIKE）"
        )
        self._strike_dtot_export_btn.setEnabled(False)
        self._strike_dtot_export_btn.clicked.connect(self._on_dtot_export)
        layout.addWidget(self._strike_dtot_export_btn)

        # ══════════════════════════════════════════════════════════════
        #  OWA-UAV 戰術 .parm 生成
        # ══════════════════════════════════════════════════════════════
        sep_owa = QFrame()
        sep_owa.setFrameShape(QFrame.Shape.HLine)
        sep_owa.setStyleSheet("color:#424242;")
        layout.addWidget(sep_owa)

        owa_header = QLabel("OWA-UAV 飛控參數")
        owa_header.setStyleSheet(
            "font-weight:bold;color:#78909C;font-size:12px;padding:4px 0;"
        )
        layout.addWidget(owa_header)

        owa_desc = QLabel(
            "為 ArduPlane SITL 產生 OWA (One-Way Attack) 戰術參數：\n"
            "Failsafe=繼續任務 / ICE 內燃機 / 地形跟隨突防"
        )
        owa_desc.setWordWrap(True)
        owa_desc.setStyleSheet("color:#BDBDBD;font-size:10px;padding:2px 4px;")
        layout.addWidget(owa_desc)

        self._strike_owa_btn = QPushButton("產生 OWA-UAV .parm 參數檔")
        self._strike_owa_btn.setStyleSheet(
            "QPushButton{background-color:#37474F;color:white;padding:8px;"
            "font-weight:bold;border-radius:4px;}"
            "QPushButton:hover{background-color:#455A64;}"
        )
        self._strike_owa_btn.setToolTip(
            "在 sitl/default_params/ 產生 owa_uav_default.parm\n"
            "包含 Failsafe 戰術覆寫 / ICE 引擎 / 地形跟隨\n"
            "SITL 啟動時加 --defaults plane.parm,owa_uav_default.parm"
        )
        self._strike_owa_btn.clicked.connect(self.strike_owa_parm_requested.emit)
        layout.addWidget(self._strike_owa_btn)

        # ── 清除按鈕 ─────────────────────────────────────────────────
        clear_btn = QPushButton("清除打擊視覺化")
        clear_btn.setStyleSheet(
            "QPushButton{background-color:#455A64;color:white;padding:6px;"
            "border-radius:3px;}"
            "QPushButton:hover{background-color:#546E7A;}"
        )
        clear_btn.clicked.connect(self.strike_clear_requested.emit)
        layout.addWidget(clear_btn)

        return group

    def set_strike_export_enabled(self, enabled: bool):
        """規劃完成 / 清除後由主視窗呼叫，切換匯出與 SITL 按鈕啟用狀態"""
        if hasattr(self, '_strike_export_btn'):
            self._strike_export_btn.setEnabled(enabled)
        if hasattr(self, '_strike_dtot_export_btn'):
            self._strike_dtot_export_btn.setEnabled(enabled)
        if hasattr(self, '_strike_sitl_upload_btn'):
            self._strike_sitl_upload_btn.setEnabled(enabled)

    def _on_dtot_export(self):
        """收集時間協同匯出參數並發射信號。

        2026 重構：明確拆分兩個正交概念，不再混用 'mode' key：
          - launch_mode: 'SAME' 同地 / 'DIFF' 異地  (發射位置)
          - timing_mode: 'STOT' 同時 / 'DTOT' 間隔  (命中時間)
        """
        params = {
            'launch_mode':      self.get_strike_launch_mode(),
            'timing_mode':      self.get_strike_timing_mode(),
            'interval_sec':     self.get_strike_interval_sec(),
            'cruise_speed':     self._strike_cruise_speed.value(),
            'max_speed':        self._strike_max_speed.value(),
            'stall_speed':      self._strike_stall_speed.value(),
            'min_turn_radius':  self._strike_turn_radius.value(),
        }
        self.strike_dtot_export_requested.emit(params)

    def update_dtot_preview(self, text: str):
        """由主視窗呼叫，更新 DTOT 結果預覽文字"""
        if hasattr(self, '_strike_dtot_preview'):
            self._strike_dtot_preview.setText(text)

    def update_collision_report(self, safe: bool, conflicts: list):
        """顯示避障報告（Task 3：CollisionReport UI）"""
        if not hasattr(self, '_strike_dtot_preview'):
            return
        current = self._strike_dtot_preview.text()
        if safe:
            extra = '\n[避障] SAFE — 無衝突'
            color = '#FFD54F'
        else:
            extra = f'\n[避障] {len(conflicts)} 項衝突:'
            for c in conflicts[:3]:   # 最多顯示 3 條
                extra += f'\n  ! {c}'
            if len(conflicts) > 3:
                extra += f'\n  ... +{len(conflicts) - 3} more'
            color = '#FF8A65'
        self._strike_dtot_preview.setText(current + extra)
        self._strike_dtot_preview.setStyleSheet(
            f'color:{color};font-size:10px;padding:2px 4px;'
        )

    # ─── ReconToStrike / VTOL 動態切換 handlers ─────────────────────
    def _on_strike_recon_trigger(self):
        """觸發 DCCPP → 蜂群打擊動態切換"""
        params = {
            'coalition_size':   self._strike_coalition_size.value(),
            'iapf_safe_dist':   self._strike_iapf_dist.value(),
            'cruise_speed':     self._strike_cruise_speed.value(),
            'stall_speed':      self._strike_stall_speed.value(),
            'max_speed':        self._strike_max_speed.value(),
            'turn_radius':      self._strike_turn_radius.value(),
            'base_alt':         self._strike_cruise_alt.value(),
            'alt_step':         self._strike_alt_step.value(),
        }
        self.strike_recon_trigger_requested.emit(params)

    def _on_strike_vtol_toggled(self, checked: bool):
        """VTOL 模式切換 → 啟用/禁用 VTOL 相關 SpinBox + 發射信號"""
        for w in ('_strike_vtol_cruise_kts', '_strike_vtol_terminal_kts',
                  '_strike_vtol_boundary_m', '_strike_vtol_cep_m',
                  '_strike_vtol_ai_seeker'):
            if hasattr(self, w):
                getattr(self, w).setEnabled(checked)
        self.strike_vtol_toggle_changed.emit(checked)

    def get_strike_vtol_params(self) -> dict:
        """取得 VTOL 模式全部參數（main_window 讀取用）"""
        if not hasattr(self, '_strike_vtol_enabled'):
            return {'enabled': False}
        return {
            'enabled':        self._strike_vtol_enabled.isChecked(),
            'cruise_kts':     self._strike_vtol_cruise_kts.value(),
            'terminal_kts':   self._strike_vtol_terminal_kts.value(),
            'boundary_m':     self._strike_vtol_boundary_m.value(),
            'cep_m':          self._strike_vtol_cep_m.value(),
            'ai_seeker':      self._strike_vtol_ai_seeker.isChecked(),
        }

    # ─────────────────────────────────────────────────────────────
    #  發射模式 / 時間協同 getters
    #
    #  2026 重構 — 徹底消除 STOT/DTOT 舊/新版歧義：
    #      launch_mode : 'SAME' (同地) / 'DIFF' (異地)  ← 僅「發射位置」
    #      timing_mode : 'STOT' (同時命中) / 'DTOT' (間隔命中)  ← 唯一 STOT/DTOT
    #
    #  `get_strike_launch_mode()` 的回傳值從 'STOT'/'DTOT' 改為 'SAME'/'DIFF'，
    #  所有下游 (main_window / strike_controller) 也同步使用新值。
    # ─────────────────────────────────────────────────────────────
    def get_strike_launch_mode(self) -> str:
        """取得發射位置模式 ('SAME' 同地 / 'DIFF' 異地)"""
        if not hasattr(self, '_strike_launch_mode'):
            return 'DIFF'
        text = self._strike_launch_mode.currentText()
        return 'SAME' if '同地' in text else 'DIFF'

    def get_strike_timing_mode(self) -> str:
        """取得時間協同模式 ('STOT' 同時命中 / 'DTOT' 間隔命中)"""
        if not hasattr(self, '_strike_timing_mode'):
            return 'STOT'
        return 'DTOT' if 'DTOT' in self._strike_timing_mode.currentText() else 'STOT'

    def get_strike_interval_sec(self) -> float:
        """取得 DTOT 間隔秒數 (STOT 模式此值無效)"""
        if not hasattr(self, '_strike_interval_sec'):
            return 0.0
        return float(self._strike_interval_sec.value())

    def _on_strike_launch_mode_changed(self, index: int):
        """發射位置切換 → 顯示/隱藏 STOT 基地區塊 + 發射信號"""
        mode = self.get_strike_launch_mode()
        if hasattr(self, '_strike_base_group'):
            # SAME (同地) 才需要共用發射基地
            self._strike_base_group.setVisible(mode == 'SAME')
        self.strike_mode_changed.emit(mode)

    def _on_strike_timing_mode_changed(self, index: int):
        """時間協同模式切換 → 啟用/禁用 interval 輸入欄位"""
        is_dtot = self.get_strike_timing_mode() == 'DTOT'
        if hasattr(self, '_strike_interval_sec'):
            self._strike_interval_sec.setEnabled(is_dtot)
        if hasattr(self, '_strike_interval_label'):
            color = '#FFAB91' if is_dtot else '#555555'
            self._strike_interval_label.setStyleSheet(f'color:{color};')

    def set_strike_base_marking_mode(self, active: bool):
        """由主視窗呼叫，同步 STOT 基地標記按鈕狀態"""
        if hasattr(self, '_strike_mark_base_btn'):
            self._strike_mark_base_btn.setChecked(active)

    def update_strike_base_label(self, lat: float = None, lon: float = None):
        """更新 STOT 基地座標顯示；傳 None 代表尚未標記"""
        if not hasattr(self, '_strike_base_label'):
            return
        if lat is None or lon is None:
            self._strike_base_label.setText("未標記 — 必須先設定基地才能 EXECUTE")
            self._strike_base_label.setStyleSheet(
                "color:#FF5252;font-size:10px;padding:2px 4px;"
            )
        else:
            self._strike_base_label.setText(
                f"✓ 基地: ({lat:.6f}, {lon:.6f})"
            )
            self._strike_base_label.setStyleSheet(
                "color:#81C784;font-size:10px;padding:2px 4px;font-weight:bold;"
            )

    def _on_strike_execute(self):
        """蜂群打擊執行按鈕 — 同時攜帶發射位置 + 時間協同模式

        2026 重構：內部值徹底分離歧義：
          * launch_mode = 'SAME' / 'DIFF'  (發射位置)
          * timing_mode = 'STOT' / 'DTOT'  (命中時間)
        為相容舊程式，同時保留 'mode' key (= launch_mode)。
        """
        launch = self.get_strike_launch_mode()    # 'SAME' / 'DIFF'
        timing = self.get_strike_timing_mode()    # 'STOT' / 'DTOT'
        params = {
            # 新命名 (推薦使用)
            'launch_mode':          launch,         # 'SAME' 同地 / 'DIFF' 異地
            'timing_mode':          timing,         # 'STOT' 同時 / 'DTOT' 間隔
            'interval_sec':         self.get_strike_interval_sec(),
            # 舊 key 'mode' 保留以相容已寫好的下游程式；值亦為 SAME/DIFF
            'mode':                 launch,
            # 幾何
            'cruise_alt':           self._strike_cruise_alt.value(),
            'cruise_speed':         self._strike_cruise_speed.value(),
            'altitude_step':        self._strike_alt_step.value(),
            'max_dive_angle':       self._strike_max_dive_angle.value(),
            'dive_initiation_dist': self._strike_dive_dist.value(),
            'anim_speed':           self._strike_anim_speed.value(),
            # 時空協同邊界
            'max_speed':            self._strike_max_speed.value(),
            'stall_speed':          self._strike_stall_speed.value(),
            'min_turn_radius':      self._strike_turn_radius.value(),
        }
        self.strike_execute_requested.emit(params)
        logger.info(
            f'[Strike] 執行蜂群打擊: launch={launch} '
            f'({"同地" if launch == "SAME" else "異地"}), '
            f'timing={timing} ({"同時" if timing == "STOT" else "間隔"})'
            + (f' Δ={params["interval_sec"]}s' if timing == 'DTOT' else '')
        )

    def update_strike_target_count(self, count: int):
        """更新打擊目標計數顯示"""
        self._strike_target_count.setText(
            f"已標記目標: {count} 個 → 自動生成 {count} 架 UCAV"
            if count > 0 else "已標記目標: 0 個"
        )
        self._strike_execute_btn.setEnabled(count > 0)
        self._strike_target_n = count
        self._update_strike_alt_preview()

    def _update_strike_alt_preview(self):
        """更新高度錯層預覽"""
        n = getattr(self, '_strike_target_n', 0)
        if n <= 0:
            self._strike_alt_preview.setText("")
            return
        base = self._strike_cruise_alt.value()
        step = self._strike_alt_step.value()
        alts = [f'{base + i * step:.0f}' for i in range(min(n, 8))]
        suffix = ' ...' if n > 8 else ''
        self._strike_alt_preview.setText(
            f'各機高度: {", ".join(alts)}{suffix} m'
        )

    def set_strike_marking_mode(self, active: bool):
        """設定標記模式按鈕狀態"""
        self._strike_mark_btn.setChecked(active)

    def reset_to_default(self):
        """重置為預設參數"""
        default_params = {
            'altitude': 10.0,
            'speed': 3.0,
            'angle': 0.0,
            'spacing': 20.0,
            'yaw_speed': 60.0,
            'subdivisions': 1,
            'region_spacing': 3.0,
            'reduce_overlap': True,
            'flight_mode': 'smart_collision',
            'turn_radius': 50.0,
            'lock_heading': False,
            'heading_angle': 0.0,
        }

        self.set_parameters(default_params)
        self.parameters_changed.emit(default_params)
        logger.info("參數已重置為預設值")
