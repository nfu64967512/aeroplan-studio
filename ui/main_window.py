"""
主視窗模組
整合地圖、參數面板、任務面板等核心 UI 組件
"""

import sys
import math
from typing import Optional, Tuple
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QSplitter, QStatusBar, QToolBar, QMessageBox,
    QFileDialog, QLabel, QScrollArea, QApplication
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QAction, QIcon, QKeySequence, QShortcut

# 修正導入路徑
from config import get_settings
from utils.logger import get_logger
from utils.file_io import write_waypoints, create_waypoint_line
from mission import MissionManager
from core.global_planner.coverage_planner import CoveragePlanner, CoverageParameters, ScanPattern
from core.global_planner.astar import AStarPlanner
from core.global_planner.rrt import RRTPlanner, RRTStarPlanner
from core.global_planner.fixed_wing_planner import (
    FixedWingPlanner, FixedWingParameters, TurnRadiusChecker,
)
from core.collision import CollisionChecker
from mission.swarm_coordinator import SwarmCoordinator, SwarmMission, DroneInfo
from mission.coverage_path import CoveragePath

# 2026 重構：Swarm Strike 邏輯已抽出至 Controller Mixin (ui/controllers/)
from ui.controllers import StrikeControllerMixin

# 戰術蜂群打擊面板（VTOL 全任務生命週期）— 彈出式對話框
from ui.widgets.tactical_swarm_strike_panel import TacticalSwarmStrikeDialog

# 獲取配置和日誌實例
settings = get_settings()
logger = get_logger()

# 常數定義
MIN_CORNERS = 3   # 最少邊界點數量
MAX_CORNERS = 100  # 最大邊界點數量
_WP_LIMIT = 655   # Mission Planner 單次最大匯入航點數


class MainWindow(QMainWindow, StrikeControllerMixin):
    """
    主視窗類
    
    整合地圖顯示、參數控制、任務管理等核心功能
    """
    
    # 信號定義
    mission_changed = pyqtSignal(object)  # 任務變更信號
    waypoints_updated = pyqtSignal(list)  # 航點更新信號
    
    def __init__(self):
        """初始化主視窗"""
        super().__init__()

        # 視窗基本設置（使用 settings 替代 Config）
        self.setWindowTitle(settings.ui.window_title)
        self._apply_responsive_geometry()
        self.setMinimumSize(settings.ui.min_window_width, settings.ui.min_window_height)
        
        # 初始化核心組件
        self.mission_manager = MissionManager()
        
        # 初始化變數
        self.init_variables()
        
        # 建立 UI
        self.init_ui()
        
        # 載入樣式表
        self.load_stylesheet()
        
        # 顯示歡迎信息
        self.statusBar().showMessage("無人機路徑規劃工具已就緒", 5000)
        
        logger.info("主視窗初始化完成")
    
    def _apply_responsive_geometry(self,
                                    target_ratio: float = 0.85,
                                    min_w: int = 1280,
                                    min_h: int = 720):
        """依螢幕尺寸動態決定主視窗大小，跨 1080p / 2K / 4K 不跑版。

        規則：
          - 取得主螢幕 availableGeometry (扣掉工作列)
          - 預設佔 85% × 85% 空間
          - 水平置中、垂直偏上 (讓標題列不會被遮住)
          - 視 DPR 縮放：4K 上也會維持合理視覺比例
          - 夾底：不小於 min_w × min_h (1280×720)

        Parameters
        ----------
        target_ratio : float
            視窗佔螢幕可用區域的比例 (0~1)
        min_w, min_h : int
            視窗最小像素尺寸 (logical px)
        """
        try:
            from PyQt6.QtGui import QGuiApplication
            screen = (self.screen() if hasattr(self, 'screen') and self.screen()
                      else QGuiApplication.primaryScreen())
            if screen is None:
                # fallback: 使用 settings 固定值
                self.setGeometry(100, 100,
                                 settings.ui.window_width,
                                 settings.ui.window_height)
                return

            avail = screen.availableGeometry()   # QRect (logical px，已扣工作列)
            scr_w, scr_h = avail.width(), avail.height()

            # 計算目標尺寸 (依比例 + 下限)
            win_w = max(int(scr_w * target_ratio), min_w)
            win_h = max(int(scr_h * target_ratio), min_h)
            # 上限不能超過螢幕可用區 (4K 下會卡到 3400 左右)
            win_w = min(win_w, scr_w)
            win_h = min(win_h, scr_h)

            # 居中對齊 (垂直偏上 8%)
            x = avail.left() + (scr_w - win_w) // 2
            y = avail.top() + max(0, (scr_h - win_h) // 3)

            self.setGeometry(x, y, win_w, win_h)

            # DPI 資訊記錄 (供除錯用)
            try:
                dpr = screen.devicePixelRatio()
                logger.info(
                    f'[Window] 螢幕 {scr_w}x{scr_h} (DPR={dpr:.1f}), '
                    f'視窗尺寸 {win_w}x{win_h} @ ({x},{y})'
                )
            except Exception:
                pass
        except Exception as e:
            logger.warning(f'[Window] 自適應尺寸失敗，使用 settings 預設: {e}')
            self.setGeometry(100, 100,
                             settings.ui.window_width,
                             settings.ui.window_height)

    def init_variables(self):
        """初始化變數"""
        self.current_mission = None
        self.corners = []  # 邊界點
        self.waypoints = []  # 航點（扁平化，供單台匯出使用）
        self.spiral_waypoint_altitudes: list = []  # 螺旋掃描每航點高度（alt_step > 0 時有效）
        self.sub_paths = []  # 各子區域路徑（分割模式時使用）
        self.obstacles = []  # 障礙物

        # 當前演算法
        self.current_algorithm = 'grid'

        # 當前載具設定
        self.current_vehicle_type = '多旋翼'
        self.current_vehicle_model = 'DJI Mavic 3'

        # 飛行參數
        self.flight_params = {
            'altitude': 10.0,
            'speed': 3.0,
            'angle': 0.0,
            'spacing': 20.0,
            'yaw_speed': 60.0,
            'subdivisions': 1,
            'region_spacing': 3.0,
            'gap_spacings_m': [],        # 各邊界獨立間隔
            'v_spacing_m': 3.0,          # 網格垂直間隔
            'transit_stagger_enabled': False,
            'transit_stagger_alt': 10.0,
            'turn_radius': 50.0,  # 固定翼轉彎半徑 (m)
            'lock_heading': False,
            'heading_angle': 0.0,
            # ── 固定翼專用 ─────────────────────────────────────
            'fw_takeoff_bearing': 0.0,
            'fw_runway_length': 50.0,
            'fw_landing_bearing': 180.0,
            'fw_pattern_alt': 80.0,
            'fw_downwind_offset': 200.0,
            'fw_pattern_leg': 300.0,
            'fw_final_dist': 400.0,
            'fw_scan_mode': 'spiral',
            'fw_autoland': False,
            # ── 螺旋專用 ────────────────────────────────────────
            'spiral_curvature': 1.0,
            'spiral_alt_step': 0.0,
        }

        # 固定翼任務結果（三階段路徑）
        self.fw_mission_result: dict = {}

        # 禁航區清單（FixedWingNFZPlanner 使用）
        # 每筆格式：{'type': 'polygon'/'circle', 'name': str,
        #            'vertices': [(lat,lon),...] | 'center':(lat,lon), 'radius': float}
        self.nfz_zones: list = []

        # 螺旋/同心圓掃描圓心模式
        self.circle_center: Optional[Tuple[float, float]] = None  # (lat, lon)
        self.picking_circle_center: bool = False

        # 固定翼起飛點（跑道位置，獨立於掃描區中心）
        self.home_point: Optional[Tuple[float, float]] = None  # (lat, lon)
        self.picking_home_point: bool = False

        self._dccpp_result = None  # 當前 DCCPP 最佳化結果

        # 即時路徑生成設定（已停用：改由「預覽」按鈕 / Enter 鍵手動觸發，見 on_preview_paths）
        # 保留屬性以維持 _schedule_path_generation / _auto_generate_path 方法的向後相容
        self.auto_generate_path = False
        self.path_generation_timer = None  # 延遲生成計時器
        self.path_generation_delay = 300   # 延遲時間 (ms)

        # 戰術蜂群打擊面板（VTOL 全任務生命週期）— 單例浮動對話框
        self._swarm_strike_dialog: Optional[TacticalSwarmStrikeDialog] = None
        self.picking_swarm_launch: bool = False   # 地圖點擊設為蜂群起飛點
        self.picking_swarm_target: bool = False   # 地圖點擊設為蜂群目標
    
    def init_ui(self):
        """初始化 UI 組件"""
        # 創建中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主佈局
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # 創建分割器（地圖 | 控制面板）— 可拖拉決定比例
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setHandleWidth(6)
        splitter.setChildrenCollapsible(False)
        splitter.setStyleSheet("""
            QSplitter::handle {
                background-color: #37474F; border: 1px solid #263238;
            }
            QSplitter::handle:horizontal { width: 6px; }
            QSplitter::handle:hover      { background-color: #FFB74D; }
            QSplitter::handle:pressed    { background-color: #FF9800; }
        """)

        # 左側：地圖區域
        self.map_widget = self.create_map_widget()
        splitter.addWidget(self.map_widget)

        # 右側：控制面板
        control_panel = self.create_control_panel()
        splitter.addWidget(control_panel)
        
        # 依「視窗寬度」自適應分割比例（地圖 60%，控制面板 40%）
        # 使用視窗寬度而非螢幕寬度，避免 4K 上 splitter 超出視窗
        win_w = self.width() or 1280
        map_w   = int(win_w * 0.60)
        panel_w = int(win_w * 0.40)
        splitter.setStretchFactor(0, 60)
        splitter.setStretchFactor(1, 40)
        splitter.setSizes([map_w, panel_w])
        # 快取 splitter 以便 resizeEvent 時重新分配
        self._main_splitter = splitter
        
        main_layout.addWidget(splitter)
        
        # 創建工具列
        self.create_toolbar()
        
        # 創建狀態列
        self.create_statusbar()
        
        # 創建選單
        self.create_menus()

        # 設置快捷鍵
        self.setup_shortcuts()

        # 建立 Undo/Redo Stack（Ctrl+Z / Ctrl+Y）
        try:
            from ui.undo_commands import setup_undo_stack
            setup_undo_stack(self)
        except Exception as e:
            logger.warning(f'[Undo] 初始化失敗: {e}')
    
    def create_map_widget(self):
        """創建地圖組件（DualMapWidget：2D Folium + 3D Cesium 雙模式）"""
        from ui.widgets.dual_map_widget import DualMapWidget

        map_widget = DualMapWidget(self)

        # 連接信號
        map_widget.corner_added.connect(self.on_corner_added)
        map_widget.corner_moved.connect(self.on_corner_moved)

        return map_widget

    def open_click_map_window(self):
        """打開可點擊的地圖視窗（使用 tkintermapview）"""
        try:
            import tkinter as tk
            import tkintermapview

            # 創建 Tkinter 視窗
            self.tk_map_window = tk.Toplevel()
            self.tk_map_window.title("點擊地圖添加角點 - 左鍵點擊添加")
            self.tk_map_window.geometry("900x700")

            # 創建地圖
            map_widget = tkintermapview.TkinterMapView(
                self.tk_map_window,
                width=900,
                height=650,
                corner_radius=0
            )
            map_widget.pack(fill="both", expand=True)

            # 設置位置
            map_widget.set_position(
                settings.map.default_lat,
                settings.map.default_lon
            )
            map_widget.set_zoom(settings.map.default_zoom)

            # 嘗試設置 Google 衛星圖
            try:
                map_widget.set_tile_server(
                    "https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}",
                    max_zoom=20
                )
            except:
                pass

            # 儲存標記列表
            self.tk_markers = []
            self.tk_polygon = None

            def on_click(coords):
                lat, lon = coords
                # 添加標記
                point_num = len(self.corners) + 1
                marker = map_widget.set_marker(
                    lat, lon,
                    text=f"P{point_num}",
                    marker_color_circle="green"
                )
                self.tk_markers.append(marker)

                # 添加到主視窗
                self.on_manual_corner_added(lat, lon)

                # 更新多邊形
                if len(self.corners) >= 3:
                    if hasattr(self, 'tk_polygon') and self.tk_polygon:
                        self.tk_polygon.delete()
                    self.tk_polygon = map_widget.set_polygon(
                        self.corners,
                        fill_color="green",
                        outline_color="darkgreen",
                        border_width=2
                    )

                logger.info(f"Tkinter 地圖點擊: ({lat:.6f}, {lon:.6f})")

            map_widget.add_left_click_map_command(on_click)

            # 添加說明標籤
            info_frame = tk.Frame(self.tk_map_window)
            info_frame.pack(fill="x", pady=5)
            tk.Label(
                info_frame,
                text="左鍵點擊地圖添加角點 | 滾輪縮放 | 右鍵拖曳移動",
                font=("Arial", 10)
            ).pack()

            logger.info("已打開 Tkinter 地圖視窗")

        except ImportError:
            QMessageBox.warning(
                self, "缺少套件",
                "請先安裝 tkintermapview:\n\npip install tkintermapview"
            )
        except Exception as e:
            logger.error(f"打開 Tkinter 地圖失敗: {e}")
            QMessageBox.critical(self, "錯誤", f"打開地圖視窗失敗:\n{str(e)}")
    
    def create_control_panel(self):
        """創建控制面板（右側）"""
        from ui.widgets.parameter_panel import ParameterPanel
        from ui.widgets.mission_panel import MissionPanel

        # 外層容器：垂直排列 [ScrollArea(參數面板)] + [任務面板]
        panel_widget = QWidget()
        panel_layout = QVBoxLayout(panel_widget)
        panel_layout.setContentsMargins(4, 4, 4, 4)
        panel_layout.setSpacing(6)

        # 參數面板
        self.parameter_panel = ParameterPanel(self)
        self.parameter_panel.corner_added.connect(self.on_manual_corner_added)
        self.parameter_panel.clear_corners_requested.connect(self.on_clear_corners)
        self.parameter_panel.remove_last_corner_requested.connect(self.on_delete_last_corner)
        self.parameter_panel.open_click_map_requested.connect(self.open_click_map_window)
        self.parameter_panel.pick_center_requested.connect(self.on_pick_circle_center)
        self.parameter_panel.drag_circle_requested.connect(self.on_drag_circle_mode)
        self.map_widget.circle_defined.connect(self.on_circle_drag_defined)
        self.parameter_panel.pick_home_point_requested.connect(self.on_pick_home_point)
        self.parameter_panel.clear_home_point_requested.connect(self.on_clear_home_point)
        self.parameter_panel.manage_nfz_requested.connect(self.open_nfz_dialog)
        self.parameter_panel.nfz_draw_polygon_requested.connect(self.on_nfz_draw_polygon)
        self.parameter_panel.nfz_draw_circle_requested.connect(self.on_nfz_draw_circle)
        self.parameter_panel.nfz_poly_finish_requested.connect(self.on_nfz_poly_finish)
        self.map_widget.nfz_polygon_drawn.connect(self.on_nfz_polygon_drawn)
        self.map_widget.nfz_circle_drawn.connect(self.on_nfz_circle_drawn)
        self.parameter_panel.dccpp_coverage_requested.connect(self._on_dccpp_coverage_requested)
        self.parameter_panel.dccpp_export_requested.connect(self._on_dccpp_export_requested)
        self.parameter_panel.vtol_export_requested.connect(self._on_vtol_export_requested)
        self.parameter_panel.delete_last_corner_requested.connect(self.on_delete_last_corner)
        self.parameter_panel.dem_loaded.connect(self._on_dem_loaded)

        # 將參數面板包進 QScrollArea，解決高解析度以外螢幕時內容被截斷的問題
        scroll = QScrollArea()
        scroll.setWidget(self.parameter_panel)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        # 自適應最小寬度：螢幕寬度的 22%，至少 280px
        screen = QApplication.primaryScreen()
        min_panel_w = max(280, int((screen.availableGeometry().width() if screen else 1280) * 0.22))
        scroll.setMinimumWidth(min_panel_w)

        # 任務面板
        self.mission_panel = MissionPanel(self)

        # 以 QSplitter 取代 VBox → 使用者可拖拉決定「參數面板 / 任務面板」比例
        from PyQt6.QtWidgets import QSplitter, QSizePolicy
        self._right_panel_splitter = QSplitter(Qt.Orientation.Vertical)
        self._right_panel_splitter.setChildrenCollapsible(False)
        self._right_panel_splitter.setHandleWidth(6)
        self._right_panel_splitter.setStyleSheet("""
            QSplitter::handle {
                background-color: #37474F; border: 1px solid #263238;
            }
            QSplitter::handle:vertical { height: 6px; }
            QSplitter::handle:hover    { background-color: #FFB74D; }
            QSplitter::handle:pressed  { background-color: #FF9800; }
        """)
        scroll.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.mission_panel.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred
        )
        self._right_panel_splitter.addWidget(scroll)
        self._right_panel_splitter.addWidget(self.mission_panel)
        # 預設比例：參數面板 78% / 任務面板 22%
        self._right_panel_splitter.setStretchFactor(0, 78)
        self._right_panel_splitter.setStretchFactor(1, 22)
        panel_layout.addWidget(self._right_panel_splitter, 1)

        # SITL HUD 分頁（Mission Planner 風格即時遙測）
        from ui.widgets.sitl_hud import SITLHud
        self.sitl_hud = SITLHud(self)
        self.sitl_hud.connect_requested.connect(self.on_sitl_connect)
        self.sitl_hud.disconnect_requested.connect(self.on_sitl_disconnect)
        self.sitl_hud.launch_sitl_requested.connect(self.on_sitl_launch)
        self.sitl_hud.stop_sitl_requested.connect(self.on_sitl_stop_local)
        self.sitl_hud.cmd_arm.connect(lambda: self._sitl_broadcast('arm'))
        self.sitl_hud.cmd_disarm.connect(lambda: self._sitl_broadcast('disarm'))
        self.sitl_hud.cmd_set_mode.connect(lambda m: self._sitl_broadcast('set_mode', m))
        self.sitl_hud.cmd_takeoff.connect(lambda a: self._sitl_broadcast('takeoff', a))
        self.sitl_hud.cmd_upload.connect(self.on_sitl_upload_mission)
        self.sitl_hud.cmd_auto_start.connect(lambda: self._sitl_broadcast('auto_start'))
        self.sitl_hud.cmd_param_set.connect(
            lambda n, v, t: self._sitl_broadcast('set_param', (n, v, t))
        )
        self.sitl_hud.cmd_param_get.connect(
            lambda n: self._sitl_broadcast('get_param', n)
        )
        self.sitl_hud.cmd_params_batch.connect(
            lambda items: self._sitl_broadcast('set_params', items)
        )
        self.sitl_hud.cmd_vtol_transition.connect(
            lambda state: self._sitl_broadcast('vtol_transition', state)
        )

        # 加到參數面板的 QTabWidget 作為第 4 個分頁
        sitl_scroll = QScrollArea()
        sitl_scroll.setWidget(self.sitl_hud)
        sitl_scroll.setWidgetResizable(True)
        sitl_scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        self.parameter_panel._tabs.addTab(sitl_scroll, "🛰 SITL")

        # 任務操作面板隨分頁切換可見元件：只有「基本演算法」分頁才顯示完整預覽/匯出
        self.parameter_panel._tabs.currentChanged.connect(self._on_main_tab_changed)
        self._on_main_tab_changed(0)  # 初始化

        self._sitl_link = None        # 主鏈路（向後相容）
        self._sitl_links: list = []   # 多機鏈路列表
        self._sitl_launcher = None
        
        # 連接信號
        self.parameter_panel.parameters_changed.connect(self.on_parameters_changed)
        self.mission_panel.preview_requested.connect(self.on_preview_paths)
        self.mission_panel.export_requested.connect(self.on_export_waypoints)
        self.mission_panel.clear_requested.connect(self.on_clear_all)

        # ── 戰術模組信號連接 ──────────────────────────────────────────
        self.parameter_panel.elevation_slicer_changed.connect(self._on_elev_slicer_changed)
        self.parameter_panel.elevation_slicer_cleared.connect(self._on_elev_slicer_cleared)
        self.parameter_panel.sar_heatmap_init_requested.connect(self._on_sar_heatmap_init)
        self.parameter_panel.sar_heatmap_reset_requested.connect(self._on_sar_heatmap_reset)
        self.parameter_panel.sar_heatmap_clear_requested.connect(self._on_sar_heatmap_clear)
        self.parameter_panel.fov_cone_toggle_requested.connect(self._on_fov_cone_toggle)
        self.parameter_panel.radar_sim_requested.connect(self._on_radar_sim)
        self.parameter_panel.radar_clear_requested.connect(self._on_radar_clear)
        self.parameter_panel.rcs_toggle_requested.connect(self._on_rcs_toggle)

        # 戰術模組內部狀態
        self._fov_cone_enabled = False   # FOV 光錐是否啟用
        self._rcs_enabled = False        # RCS 渲染是否啟用

        # ── 蜂群打擊模組信號連接 ──────────────────────────────────────
        self.parameter_panel.strike_mark_targets_requested.connect(self._on_strike_mark_targets)
        self.parameter_panel.strike_mark_base_requested.connect(self._on_strike_mark_base)
        self.parameter_panel.strike_mode_changed.connect(self._on_strike_mode_changed)
        self.parameter_panel.strike_execute_requested.connect(self._on_strike_execute)
        self.parameter_panel.strike_clear_requested.connect(self._on_strike_clear)
        self.parameter_panel.strike_export_requested.connect(self._on_strike_export)
        self.parameter_panel.strike_dtot_export_requested.connect(self._on_strike_dtot_export)
        self.parameter_panel.strike_owa_parm_requested.connect(self._on_strike_owa_parm)
        self.parameter_panel.strike_sitl_upload_requested.connect(self._on_strike_sitl_upload)
        self.parameter_panel.strike_recon_trigger_requested.connect(self._on_strike_recon_trigger)
        self.parameter_panel.strike_vtol_toggle_changed.connect(self._on_strike_vtol_toggled)
        self.parameter_panel.strike_open_vtol_mission_planner_requested.connect(
            self.on_open_swarm_strike_panel
        )

        # 蜂群打擊內部狀態
        self._strike_marking_mode = False
        self._strike_base_marking_mode = False         # STOT 基地標記模式
        self._strike_launch_base = None                # (lat, lon) 或 None (STOT 模式才用)
        self._strike_targets: list = []                # [(lat, lon), ...]
        # 快取最近一次規劃結果，供匯出使用
        self._strike_result = None   # {'trajectories': [...], 'targets': [...], 'params': {...}, 'mode': 'DTOT'/'STOT', 'is_vtol': bool}
        # VTOL 模式專屬快取
        self._vtol_strike_planner = None   # VTOLSwarmStrikePlanner 實例
        self._vtol_strike_plans = None     # VTOLPlan list
        # ReconToStrikeManager 快取
        self._recon_strike_manager = None
        self._recon_strike_report = None

        return panel_widget
    
    def create_toolbar(self):
        """創建工具列"""
        toolbar = QToolBar("主工具列")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)
        
        # 新建任務
        new_action = QAction("🆕 新建", self)
        new_action.setStatusTip("創建新任務")
        new_action.triggered.connect(self.on_new_mission)
        toolbar.addAction(new_action)
        
        # 開啟任務
        open_action = QAction("📂 開啟", self)
        open_action.setStatusTip("開啟現有任務")
        open_action.triggered.connect(self.on_open_mission)
        toolbar.addAction(open_action)
        
        # 儲存任務
        save_action = QAction("💾 儲存", self)
        save_action.setStatusTip("儲存當前任務")
        save_action.triggered.connect(self.on_save_mission)
        toolbar.addAction(save_action)
        
        toolbar.addSeparator()
        
        # 預覽路徑
        preview_action = QAction("👁 預覽", self)
        preview_action.setStatusTip("預覽飛行路徑")
        preview_action.triggered.connect(self.on_preview_paths)
        toolbar.addAction(preview_action)
        
        # 匯出航點
        export_action = QAction("📤 匯出", self)
        export_action.setStatusTip("匯出航點檔案")
        export_action.triggered.connect(self.on_export_waypoints)
        toolbar.addAction(export_action)
        
        toolbar.addSeparator()

        # 蜂群打擊（VTOL 全任務生命週期）— 開啟浮動戰術面板
        swarm_action = QAction("🎯 蜂群打擊", self)
        swarm_action.setStatusTip("開啟 VTOL 蜂群打擊任務規劃面板（起飛 → 巡航 → 2km 決斷圈 → ROE）")
        swarm_action.triggered.connect(self.on_open_swarm_strike_panel)
        toolbar.addAction(swarm_action)

        toolbar.addSeparator()

        # 清除全部
        clear_action = QAction("🗑 清除", self)
        clear_action.setStatusTip("清除所有標記和路徑")
        clear_action.triggered.connect(self.on_clear_all)
        toolbar.addAction(clear_action)
    
    def create_statusbar(self):
        """創建狀態列"""
        statusbar = QStatusBar()
        self.setStatusBar(statusbar)
        
        # 添加永久顯示的資訊
        self.coord_label = QLabel("座標: --")
        statusbar.addPermanentWidget(self.coord_label)
        
        self.waypoint_label = QLabel("航點: 0")
        statusbar.addPermanentWidget(self.waypoint_label)
        
        self.distance_label = QLabel("距離: 0.0m")
        statusbar.addPermanentWidget(self.distance_label)
    
    def create_menus(self):
        """創建選單列（PyQt6 兼容版本）"""
        menubar = self.menuBar()
        
        # === 檔案選單 ===
        file_menu = menubar.addMenu("檔案(&F)")
        
        action = file_menu.addAction("新建任務")
        action.setShortcut(QKeySequence("Ctrl+N"))
        action.triggered.connect(self.on_new_mission)
        
        action = file_menu.addAction("開啟任務")
        action.setShortcut(QKeySequence("Ctrl+O"))
        action.triggered.connect(self.on_open_mission)
        
        action = file_menu.addAction("儲存任務")
        action.setShortcut(QKeySequence("Ctrl+S"))
        action.triggered.connect(self.on_save_mission)

        file_menu.addSeparator()

        # 專案檔 (.aeroplan) — 包含目標標記、邊界、STOT 基地等 UI 狀態
        action = file_menu.addAction("儲存專案 (.aeroplan)")
        action.setShortcut(QKeySequence("Ctrl+Shift+S"))
        action.setStatusTip("儲存當前所有 UI 狀態 (邊界 + 打擊目標 + 參數)")
        action.triggered.connect(self.on_save_project)

        action = file_menu.addAction("讀取專案 (.aeroplan)")
        action.setShortcut(QKeySequence("Ctrl+Shift+O"))
        action.setStatusTip("載入先前儲存的專案檔")
        action.triggered.connect(self.on_load_project)

        file_menu.addSeparator()
        
        action = file_menu.addAction("匯出航點")
        action.setShortcut(QKeySequence("Ctrl+E"))
        action.triggered.connect(self.on_export_waypoints)
        
        file_menu.addSeparator()
        
        action = file_menu.addAction("退出")
        action.setShortcut(QKeySequence("Ctrl+Q"))
        action.triggered.connect(self.close)
        
        # === 編輯選單 ===
        edit_menu = menubar.addMenu("編輯(&E)")
        
        action = edit_menu.addAction("清除路徑")
        action.triggered.connect(self.on_clear_paths)
        
        action = edit_menu.addAction("清除邊界")
        action.triggered.connect(self.on_clear_corners)
        
        action = edit_menu.addAction("清除全部")
        action.setShortcut(QKeySequence("Ctrl+R"))
        action.triggered.connect(self.on_clear_all)
        
        # === 檢視選單 ===
        view_menu = menubar.addMenu("檢視(&V)")

        action = view_menu.addAction("重置視圖")
        action.triggered.connect(self.on_reset_view)

        action = view_menu.addAction("顯示網格")
        action.triggered.connect(self.on_toggle_grid)

        view_menu.addSeparator()

        # 重置版面 (所有 Splitter 恢復預設比例)
        action = view_menu.addAction("重置版面比例")
        action.setShortcut(QKeySequence("Ctrl+Shift+R"))
        action.setStatusTip("將所有可拖拉分隔線恢復為預設比例")
        action.triggered.connect(self.on_reset_layout)

        # 切換視窗自適應尺寸
        action = view_menu.addAction("自適應視窗尺寸 (85%)")
        action.triggered.connect(self._apply_responsive_geometry)
        
        # === 工具選單 ===
        tools_menu = menubar.addMenu("工具(&T)")

        action = tools_menu.addAction("🗺️ 多邊形編輯器")
        action.setShortcut(QKeySequence("Ctrl+M"))
        action.triggered.connect(self.on_open_polygon_editor)

        action = tools_menu.addAction("🖱️ 點擊地圖視窗 (Tkinter)")
        action.triggered.connect(self.open_click_map_window)

        tools_menu.addSeparator()

        action = tools_menu.addAction("相機配置")
        action.triggered.connect(self.on_camera_config)

        action = tools_menu.addAction("飛行器配置")
        action.triggered.connect(self.on_vehicle_config)

        tools_menu.addSeparator()

        action = tools_menu.addAction("障礙物管理")
        action.triggered.connect(self.on_obstacle_manager)
        
        # === 說明選單 ===
        help_menu = menubar.addMenu("說明(&H)")
        
        action = help_menu.addAction("使用說明")
        action.triggered.connect(self.on_show_help)
        
        action = help_menu.addAction("關於")
        action.triggered.connect(self.on_about)
    
    def _on_enter_key(self):
        """Enter 鍵依當前分頁決定行為：
        - 基本演算法 (tab 0)：生成路徑
        - DCCPP (tab 2)：觸發 DCCPP 最佳化
        - 其他分頁（SITL 等）：不做任何動作（僅用於數字輸入）
        """
        tab_index = self.parameter_panel._tabs.currentIndex()
        if tab_index == 0:
            self.on_preview_paths()
        elif tab_index == 2:
            self.parameter_panel._on_dccpp_btn_clicked()
        # 其他分頁不執行任何動作

    def setup_shortcuts(self):
        """設置快捷鍵"""
        # Enter 鍵 - 依當前分頁決定行為
        enter_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Return), self)
        enter_shortcut.activated.connect(self._on_enter_key)

        # 也支援小鍵盤的 Enter
        enter_shortcut2 = QShortcut(QKeySequence(Qt.Key.Key_Enter), self)
        enter_shortcut2.activated.connect(self._on_enter_key)

        # Space 鍵 - 也可以生成路徑（備選）
        space_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Space), self)
        space_shortcut.activated.connect(self.on_preview_paths)

        # Escape 鍵 - 清除路徑
        esc_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Escape), self)
        esc_shortcut.activated.connect(self.on_clear_paths)

        # Delete 鍵 - 刪除最後一個角點
        del_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Delete), self)
        del_shortcut.activated.connect(self.on_delete_last_corner)

        # Backspace 鍵 - 也可以刪除最後一個角點
        backspace_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Backspace), self)
        backspace_shortcut.activated.connect(self.on_delete_last_corner)

        logger.info("快捷鍵設置完成: Enter=依分頁動作, Delete=刪除角點, Esc=清除路徑")

    def _update_transit_paths(self, planner, sub_paths):
        """計算並顯示轉場路徑（若啟用高度錯層）"""
        if not self.flight_params.get('transit_stagger_enabled', False):
            self.map_widget.clear_transit_paths()
            return
        if not self.corners or not sub_paths:
            return
        # 以多邊形質心作為預設起飛點
        home_lat = sum(c[0] for c in self.corners) / len(self.corners)
        home_lon = sum(c[1] for c in self.corners) / len(self.corners)
        home = (home_lat, home_lon)

        stagger_alt = self.flight_params.get('transit_stagger_alt', 10.0)
        base_alt = self.flight_params.get('altitude', 10.0)
        transit_paths = planner.generate_transit_paths(
            home=home,
            sub_paths=sub_paths,
            base_alt=base_alt,
            stagger_alt=stagger_alt,
        )
        self.map_widget.display_transit_paths(transit_paths)
        logger.info(f"轉場路徑已更新: {len(transit_paths)} 條，高度錯層 {stagger_alt}m")

    def on_pick_circle_center(self):
        """進入圓心點選模式：下次地圖點擊將設定為圓心"""
        self.picking_circle_center = True
        self.statusBar().showMessage("點擊地圖上任意位置以設定螺旋/圓心掃描的圓心...", 0)

    def on_drag_circle_mode(self):
        """進入地圖拖曳定義圓形模式（Mission Planner 風格）"""
        self.picking_circle_center = True   # 確保點擊攔截
        self.map_widget.set_circle_draw_mode(True)
        self.statusBar().showMessage(
            "按住滑鼠左鍵並拖曳以定義圓形掃描區域（放開滑鼠確認）...", 0
        )
        logger.info("進入地圖拖曳定義圓形模式")

    def on_circle_drag_defined(self, lat: float, lon: float, radius_m: float):
        """接收地圖拖曳圓形結果（同時設定圓心 + 半徑）"""
        self.circle_center = (lat, lon)
        self.picking_circle_center = False
        # 若拖曳半徑有效，更新 flight_params
        if radius_m > 1.0:
            self.flight_params['circle_max_radius'] = radius_m
        # 更新參數面板（顯示圓心 + 半徑，並更新 spinbox）
        self.parameter_panel.set_circle_center_display(lat, lon, radius_m)
        # 地圖圓形已由 map_widget.on_circle_draw_complete 渲染，不重複渲染
        r_disp = radius_m if radius_m > 1.0 else self.flight_params.get('circle_max_radius', 200.0)
        self.statusBar().showMessage(
            f"圓形已定義：圓心 ({lat:.6f}, {lon:.6f})，半徑 {r_disp:.0f} m", 5000
        )
        logger.info(f"拖曳圓形定義：圓心={lat:.6f},{lon:.6f}, r={radius_m:.1f}m")

    def on_circle_center_set(self, lat: float, lon: float):
        """設定圓心座標並更新 UI（僅點選模式）"""
        self.circle_center = (lat, lon)
        self.picking_circle_center = False
        # 更新參數面板顯示
        self.parameter_panel.set_circle_center_display(lat, lon)
        # 在地圖上繪製掃描範圍圓形
        radius = self.flight_params.get('circle_max_radius', 200.0)
        if hasattr(self.map_widget, 'draw_circle_overlay'):
            self.map_widget.draw_circle_overlay(lat, lon, radius)
        self.statusBar().showMessage(f"圓心已設定: ({lat:.6f}, {lon:.6f})，半徑 {radius:.0f}m", 5000)
        logger.info(f"圓心掃描中心點設定: ({lat:.6f}, {lon:.6f})")

    def on_pick_home_point(self):
        """進入起飛點點選模式：下次地圖點擊將設定為起飛點"""
        self.picking_home_point = True
        self.statusBar().showMessage("點擊地圖上任意位置以設定起飛點（跑道位置）...", 0)
        logger.info("進入起飛點選模式")

    def on_home_point_set(self, lat: float, lon: float):
        """設定起飛點並更新 UI"""
        self.home_point = (lat, lon)
        self.picking_home_point = False
        self.parameter_panel.set_home_point_display(lat, lon)
        self.map_widget.set_home_point_overlay(lat, lon)
        self.statusBar().showMessage(f"起飛點已設定: ({lat:.6f}, {lon:.6f})", 5000)
        logger.info(f"起飛點設定: ({lat:.6f}, {lon:.6f})")

    def on_clear_home_point(self):
        """清除起飛點"""
        self.home_point = None
        self.picking_home_point = False
        self.parameter_panel.clear_home_point_display()
        if hasattr(self.map_widget, 'clear_home_point_overlay'):
            self.map_widget.clear_home_point_overlay()
        self.statusBar().showMessage("起飛點已清除", 3000)
        logger.info("起飛點已清除")

    # ═════════════════════════════════════════════════════════════
    #  蜂群打擊面板（VTOL 全任務生命週期）— 浮動對話框整合
    # ═════════════════════════════════════════════════════════════
    def on_open_swarm_strike_panel(self):
        """開啟戰術蜂群打擊面板（單例浮動對話框）。

        - 首次開啟建立對話框並接線
        - 預填座標：起飛點用已設定的 home_point，目標用第一個邊界角點（若存在）
        - 非模態 .show()：視窗保持浮在主視窗上方但不阻擋地圖互動
        """
        if self._swarm_strike_dialog is None:
            dlg = TacticalSwarmStrikeDialog(self)
            panel = dlg.panel
            # ── 信號接線 ──
            panel.pick_launch_requested.connect(self.on_swarm_pick_launch_request)
            panel.pick_target_requested.connect(self.on_swarm_pick_target_request)
            panel.mission_planned.connect(self.on_swarm_mission_planned)
            panel.mission_exported.connect(self.on_swarm_mission_exported)
            panel.mission_deployed.connect(self.on_swarm_mission_deployed)
            self._swarm_strike_dialog = dlg

            # ── 預填座標（若主視窗已有對應狀態） ──
            if self.home_point is not None:
                panel.set_launch_point(*self.home_point)
            if self.corners:
                panel.set_target_point(*self.corners[0])

            logger.info("戰術蜂群打擊面板：已建立")

        self._swarm_strike_dialog.show()
        self._swarm_strike_dialog.raise_()
        self._swarm_strike_dialog.activateWindow()

    def on_swarm_pick_launch_request(self):
        """面板 LAUNCH PICK 按鈕 → 進入地圖拾取模式，暫時隱藏面板。"""
        self.picking_swarm_launch = True
        self.picking_swarm_target = False
        if self._swarm_strike_dialog is not None:
            self._swarm_strike_dialog.hide()
        self.statusBar().showMessage(
            "⚡ SWARM STRIKE：點擊地圖以設定起飛點（LAUNCH POINT）...", 0
        )
        logger.info("進入蜂群起飛點拾取模式")

    def on_swarm_pick_target_request(self):
        """面板 TARGET PICK 按鈕 → 進入地圖拾取模式。"""
        self.picking_swarm_target = True
        self.picking_swarm_launch = False
        if self._swarm_strike_dialog is not None:
            self._swarm_strike_dialog.hide()
        self.statusBar().showMessage(
            "⚡ SWARM STRIKE：點擊地圖以設定打擊目標（TARGET）...", 0
        )
        logger.info("進入蜂群目標拾取模式")

    def on_swarm_launch_picked(self, lat: float, lon: float):
        """使用者在地圖上點擊後，將座標回填至面板並重新顯示對話框。"""
        self.picking_swarm_launch = False
        if self._swarm_strike_dialog is not None:
            self._swarm_strike_dialog.panel.set_launch_point(lat, lon)
            self._swarm_strike_dialog.show()
            self._swarm_strike_dialog.raise_()
            self._swarm_strike_dialog.activateWindow()
        self.statusBar().showMessage(
            f"⚡ 蜂群起飛點已設定: ({lat:.6f}, {lon:.6f})", 4000
        )
        logger.info(f"蜂群起飛點設定: ({lat:.6f}, {lon:.6f})")

    def on_swarm_target_picked(self, lat: float, lon: float):
        """同上，目標版本。"""
        self.picking_swarm_target = False
        if self._swarm_strike_dialog is not None:
            self._swarm_strike_dialog.panel.set_target_point(lat, lon)
            self._swarm_strike_dialog.show()
            self._swarm_strike_dialog.raise_()
            self._swarm_strike_dialog.activateWindow()
        self.statusBar().showMessage(
            f"⚡ 蜂群打擊目標已設定: ({lat:.6f}, {lon:.6f})", 4000
        )
        logger.info(f"蜂群打擊目標設定: ({lat:.6f}, {lon:.6f})")

    def on_swarm_mission_planned(self, params, waypoints):
        """蜂群任務已規劃完成（已顯示於面板預覽區）。"""
        logger.info(
            f"蜂群任務規劃完成 — ROE={params.roe_mode.value} WP={len(waypoints)}"
        )

    def on_swarm_mission_exported(self, file_path: str):
        """蜂群任務已匯出 .waypoints 檔案。"""
        self.statusBar().showMessage(f"⚡ 蜂群任務已匯出: {file_path}", 5000)
        logger.info(f"蜂群任務匯出: {file_path}")

    def on_swarm_mission_deployed(self, waypoints):
        """面板 DEPLOY 按鈕 — 此處僅 log 與狀態提示；實際 MAVLink 上傳由連線層處理。

        若後續要串接 SITL/實機上傳，請在此：
            1. 取得當前 MAVLink 連線 (self.sitl_connection 或等效)
            2. 呼叫 mission upload (mission_clear_all + mission_item_int × N)
            3. 確認 MISSION_ACK 後發 status 回饋
        """
        logger.info(f"[SwarmStrike] DEPLOY 請求 — {len(waypoints)} 個航點")
        self.statusBar().showMessage(
            f"⚡ SWARM STRIKE DEPLOYED — {len(waypoints)} WP（MAVLink 上傳待接線）",
            8000,
        )

    @staticmethod
    def _haversine(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """兩點間大地距離（公尺）"""
        R = 6371000.0
        lat1, lon1 = math.radians(p1[0]), math.radians(p1[1])
        lat2, lon2 = math.radians(p2[0]), math.radians(p2[1])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
        return R * 2 * math.asin(math.sqrt(a))

    def on_delete_last_corner(self):
        """刪除最後一個角點"""
        if self.corners:
            removed = self.corners.pop()
            # 同步地圖 widget 的 corners 與 markers 列表
            if self.map_widget.corners:
                self.map_widget.corners.pop()
            if self.map_widget.markers:
                self.map_widget.markers.pop()
            # 重新渲染地圖
            self.map_widget._render_map()
            # 更新 UI
            self.parameter_panel.update_corner_count(len(self.corners))
            self.update_statusbar()
            logger.info(f"刪除角點: ({removed[0]:.6f}, {removed[1]:.6f}), 剩餘 {len(self.corners)} 個")

    def _schedule_path_generation(self):
        """排程延遲路徑生成（防止頻繁更新）"""
        if self.path_generation_timer:
            self.path_generation_timer.stop()

        self.path_generation_timer = QTimer()
        self.path_generation_timer.setSingleShot(True)
        self.path_generation_timer.timeout.connect(self._auto_generate_path)
        self.path_generation_timer.start(self.path_generation_delay)

    def _auto_generate_path(self):
        """自動生成路徑（靜默模式，不顯示對話框）"""
        algorithm = getattr(self, 'current_algorithm', 'grid')
        _circle_mode = algorithm in ('spiral', 'circular') and self.circle_center is not None
        if not _circle_mode and len(self.corners) < MIN_CORNERS:
            return

        try:
            # 使用 CoveragePlanner 生成路徑
            planner = CoveragePlanner()

            # 獲取當前選擇的演算法模式
            algorithm = getattr(self, 'current_algorithm', 'grid')
            if algorithm == 'grid':
                pattern = ScanPattern.GRID
            elif algorithm == 'spiral':
                pattern = ScanPattern.SPIRAL
            elif algorithm == 'circular':
                pattern = ScanPattern.CIRCULAR
            else:
                pattern = ScanPattern.GRID

            # 判斷是否為固定翼
            is_fixed_wing = self.current_vehicle_type == '固定翼'

            # 建立覆蓋參數
            params = CoverageParameters(
                spacing=self.flight_params['spacing'],
                angle=self.flight_params['angle'],
                pattern=pattern,
                is_fixed_wing=is_fixed_wing,
                turn_radius=self.flight_params.get('turn_radius', 50.0),
                smooth_turns=is_fixed_wing,
                circle_base_altitude=self.flight_params['altitude'],
                grid_alt_step=self.flight_params.get('grid_alt_step', 0.0),
                grid_base_altitude=self.flight_params['altitude'],
            )

            # 生成覆蓋路徑（支援分區）
            n_regions = self.flight_params.get('subdivisions', 1)
            spacing_m = self.flight_params.get('region_spacing', 0.0)
            gap_spacings_m = self.flight_params.get('gap_spacings_m') or None
            v_spacing_m = self.flight_params.get('v_spacing_m', 0.0) or None

            if n_regions > 1:
                sub_paths = planner.plan_subdivided_coverage(
                    self.corners, params,
                    n_regions=n_regions,
                    spacing_m=spacing_m,
                    gap_spacings_m=gap_spacings_m,
                    v_spacing_m=v_spacing_m,
                )
                self.sub_paths = sub_paths  # 保存各子區域路徑供匯出使用
                path = [pt for sp in sub_paths for pt in sp]
                if not path:
                    return
                self.map_widget.display_paths(sub_paths, self.flight_params['altitude'])
                # 顯示轉場路徑（如啟用高度錯層）
                self._update_transit_paths(planner, sub_paths)
            else:
                self.sub_paths = []
                path = planner.plan_coverage(self.corners, params)
                if not path:
                    return
                self.map_widget.display_path(path, self.flight_params['altitude'])
                self.map_widget.clear_transit_paths()

            # 計算總飛行距離
            total_distance = 0.0
            for i in range(len(path) - 1):
                lat1, lon1 = path[i]
                lat2, lon2 = path[i + 1]
                dlat = (lat2 - lat1) * 111111.0
                dlon = (lon2 - lon1) * 111111.0 * math.cos(math.radians((lat1 + lat2) / 2))
                total_distance += math.sqrt(dlat**2 + dlon**2)

            # 儲存航點
            self.waypoints = path

            # 更新狀態列
            self.waypoint_label.setText(f"航點: {len(path)}")
            self.distance_label.setText(f"距離: {total_distance:.0f}m")

            # 啟用匯出按鈕並更新任務統計
            self.mission_panel.update_mission_stats({
                'waypoint_count': len(path),
                'total_distance': total_distance,
                'estimated_time': total_distance / max(self.flight_params['speed'], 0.1),
                'area': 0,
                'regions': self.flight_params.get('subdivisions', 1),
            })

            self.statusBar().showMessage(f"即時生成: {len(path)} 個航點, {total_distance:.0f}m", 2000)
            logger.info(f"即時路徑生成: {len(path)} 個航點")

            # ── 自動同步到 Mission Planner ──────────────────────────
            self._sync_to_mission_planner(path)

        except Exception as e:
            logger.error(f"即時路徑生成失敗: {e}")

    def _sync_to_mission_planner(self, path):
        """將航點寫入同步檔，供 Mission Planner 的 FileSystemWatcher 偵測"""
        import os
        SYNC_DIR  = r"C:\Users\joy46\Documents\mp_sync"
        SYNC_FILE = os.path.join(SYNC_DIR, "aeroplan_sync.waypoints")
        try:
            os.makedirs(SYNC_DIR, exist_ok=True)
            alt = self.flight_params.get('altitude', 50)
            lines = ["QGC WPL 110"]
            # Home point
            lines.append("0\t1\t0\t16\t0\t0\t0\t0\t0\t0\t0\t1")
            for i, wp in enumerate(path, start=1):
                lat = wp[0] if len(wp) > 0 else 0
                lon = wp[1] if len(wp) > 1 else 0
                a   = wp[2] if len(wp) > 2 else alt
                lines.append(
                    f"{i}\t0\t3\t16\t0\t0\t0\t0\t{lat:.8f}\t{lon:.8f}\t{a:.2f}\t1"
                )
            with open(SYNC_FILE, "w", encoding="utf-8") as f:
                f.write("\n".join(lines))
            logger.info(f"[MP Sync] 已同步 {len(path)} 個航點到 {SYNC_FILE}")
        except Exception as e:
            logger.warning(f"[MP Sync] 同步失敗: {e}")

    def load_stylesheet(self):
        """載入戰術 HUD 主題（Tactical HUD Theme）

        標準啟動路徑中 main.py 已於建立 QApplication 時呼叫 apply_tactical_theme。
        此方法重複套用無副作用 (app.setStyleSheet 冪等)，用途：
          * 保留 __init__ 既有呼叫鏈的相容性
          * 支援直接實例化 MainWindow 的測試 / 除錯情境
        """
        try:
            from ui.resources.tactical_theme import apply_tactical_theme
            app = QApplication.instance()
            if app is not None:
                apply_tactical_theme(app)
                logger.info("戰術主題 (Tactical HUD) 已套用")
            else:
                logger.warning("QApplication 尚未建立，跳過主題套用")
        except Exception as e:
            logger.error(f"載入戰術主題失敗: {e}")
    
    # ==========================================
    # 信號處理函數
    # ==========================================
    
    def on_corner_added(self, lat, lon):
        """處理新增邊界點（從地圖點擊）"""
        # 起飛點點選模式：優先攔截（在圓心模式之前）
        if self.picking_home_point:
            if self.map_widget.corners:
                self.map_widget.corners.pop()
            if self.map_widget.markers:
                self.map_widget.markers.pop()
            self.map_widget._render_map()
            self.on_home_point_set(lat, lon)
            return

        # 圓心點選模式：攔截點擊作為掃描圓心
        if self.picking_circle_center:
            # 撤銷 map_widget 已自動加入的角點
            if self.map_widget.corners:
                self.map_widget.corners.pop()
            if self.map_widget.markers:
                self.map_widget.markers.pop()
            self.map_widget._render_map()
            self.on_circle_center_set(lat, lon)
            return

        # 蜂群打擊：起飛點拾取模式
        if self.picking_swarm_launch:
            if self.map_widget.corners:
                self.map_widget.corners.pop()
            if self.map_widget.markers:
                self.map_widget.markers.pop()
            self.map_widget._render_map()
            self.on_swarm_launch_picked(lat, lon)
            return

        # 蜂群打擊：目標拾取模式
        if self.picking_swarm_target:
            if self.map_widget.corners:
                self.map_widget.corners.pop()
            if self.map_widget.markers:
                self.map_widget.markers.pop()
            self.map_widget._render_map()
            self.on_swarm_target_picked(lat, lon)
            return

        # 檢查是否超過最大數量
        if len(self.corners) >= MAX_CORNERS:
            QMessageBox.warning(
                self, "已達上限",
                f"已達到最大邊界點數量 ({MAX_CORNERS} 個)！"
            )
            return

        self.corners.append((lat, lon))
        remaining = MAX_CORNERS - len(self.corners)
        logger.info(f"新增邊界點 #{len(self.corners)}: ({lat:.6f}, {lon:.6f}) [剩餘: {remaining}]")
        self.parameter_panel.update_corner_count(len(self.corners))
        self.update_statusbar()

    def on_manual_corner_added(self, lat, lon):
        """處理手動新增邊界點（從參數面板）"""
        # 檢查是否超過最大數量
        if len(self.corners) >= MAX_CORNERS:
            QMessageBox.warning(
                self, "已達上限",
                f"已達到最大邊界點數量 ({MAX_CORNERS} 個)！"
            )
            return

        self.corners.append((lat, lon))
        # 在地圖上添加標記
        self.map_widget.add_corner(lat, lon)
        remaining = MAX_CORNERS - len(self.corners)
        logger.info(f"手動新增邊界點 #{len(self.corners)}: ({lat:.6f}, {lon:.6f}) [剩餘: {remaining}]")
        self.parameter_panel.update_corner_count(len(self.corners))
        self.update_statusbar()

    def on_corner_moved(self, index, lat, lon):
        """處理移動邊界點"""
        if 0 <= index < len(self.corners):
            self.corners[index] = (lat, lon)
            logger.info(f"移動邊界點 #{index+1}: ({lat:.6f}, {lon:.6f})")
            self.update_statusbar()
    
    def on_parameters_changed(self, params):
        """處理參數變更"""
        # 處理演算法變更
        if 'algorithm' in params:
            self.current_algorithm = params['algorithm']
            logger.info(f"演算法變更: {self.current_algorithm}")

        # 處理載具變更
        if 'vehicle_type' in params:
            self.current_vehicle_type = params['vehicle_type']
        if 'vehicle_model' in params:
            self.current_vehicle_model = params['vehicle_model']

        # 更新飛行參數
        flight_keys = [
            'altitude', 'speed', 'angle', 'spacing', 'yaw_speed',
            'subdivisions', 'region_spacing', 'gap_spacings_m', 'v_spacing_m',
            'transit_stagger_enabled', 'transit_stagger_alt',
            'turn_radius', 'lock_heading', 'heading_angle',
            # 固定翼專用
            'fw_takeoff_bearing', 'fw_runway_length',
            'fw_landing_bearing', 'fw_pattern_alt',
            'fw_downwind_offset', 'fw_pattern_leg', 'fw_final_dist',
            'fw_scan_mode',
            # 螺旋/同心圓圓心模式
            'circle_center_lat', 'circle_center_lon',
            'circle_max_radius', 'circle_alt_step', 'circle_min_radius',
            # 螺旋專用
            'spiral_curvature', 'spiral_alt_step',
            # 固定翼 AutoLand
            'fw_autoland',
        ]
        for key in flight_keys:
            if key in params:
                self.flight_params[key] = params[key]

        # 圓心半徑變更時更新地圖圓形
        if self.circle_center and 'circle_max_radius' in params:
            c_lat, c_lon = self.circle_center
            if hasattr(self.map_widget, 'draw_circle_overlay'):
                self.map_widget.draw_circle_overlay(
                    c_lat, c_lon, self.flight_params['circle_max_radius']
                )

        logger.info(f"參數已更新: {params}")
    
    def on_preview_paths(self):
        """預覽飛行路徑"""
        algorithm = getattr(self, 'current_algorithm', 'grid')
        # 螺旋/同心圓圓心模式：不需要邊界點，只需設定圓心
        _is_circle_center_mode = (
            algorithm in ('spiral', 'circular') and self.circle_center is not None
        )
        if not _is_circle_center_mode and len(self.corners) < MIN_CORNERS:
            QMessageBox.warning(
                self, "邊界不足",
                f"需要至少 {MIN_CORNERS} 個邊界點才能生成路徑\n"
                f"（螺旋/同心圓模式請先點擊「地圖拖曳定義圓形」設定圓心）"
            )
            return

        # ── Schema 驗證飛行參數 (2026 重構：fail-fast 防無效輸入)──
        try:
            from config.schemas import FlightParameters
            fp = FlightParameters.from_dict({
                'altitude':     self.flight_params.get('altitude', 50.0),
                'speed':        self.flight_params.get('speed', 15.0),
                'angle':        self.flight_params.get('angle', 0.0),
                'spacing':      self.flight_params.get('spacing', 20.0),
                'turn_radius':  self.flight_params.get('turn_radius', 50.0),
                'vehicle_type': {
                    '多旋翼': 'multirotor', '固定翼': 'fixed_wing', 'VTOL': 'vtol',
                }.get(self.current_vehicle_type, 'multirotor'),
            })
            fp.validate()
        except ValueError as e:
            QMessageBox.warning(
                self, '飛行參數錯誤',
                f'預覽前參數驗證失敗，請修正設定:\n\n{e}'
            )
            logger.warning(f'[Preview] flight_params 驗證失敗: {e}')
            return
        except Exception as e:
            logger.debug(f'[Preview] Schema 驗證異常 (忽略): {e}')

        try:
            # 獲取當前選擇的演算法
            algorithm = getattr(self, 'current_algorithm', 'grid')

            # 判斷是否為固定翼
            is_fixed_wing = self.current_vehicle_type == '固定翼'

            path = None

            # ── 固定翼三階段路徑規劃（螺旋/同心圓/網格均啟用）──────────
            # _preview_fixed_wing_mission 內部已完整處理：
            #   地圖顯示、self.waypoints、self.fw_mission_result、統計、對話框
            # 成功後直接 return，避免下方的多旋翼後處理覆蓋地圖與航點
            if is_fixed_wing and algorithm in ['spiral', 'circular', 'grid']:
                path = self._preview_fixed_wing_mission(algorithm)
                if path is None:
                    return  # 已在內部顯示錯誤
                return  # ← 固定翼任務已完整處理，跳過後續多旋翼流程

            # 根據演算法類型生成路徑
            elif algorithm in ['grid', 'spiral', 'circular']:
                # ── 螺旋/同心圓：圓心模式 ──────────────────────────────────────
                if algorithm in ('spiral', 'circular'):
                    if self.circle_center is None:
                        QMessageBox.information(
                            self, "提示",
                            "螺旋/同心圓掃描需要先設定圓心。\n\n"
                            "方法：\n"
                            "  1. 在右側參數面板按「從地圖點選圓心」\n"
                            "  2. 在地圖上點擊想要的中心位置\n\n"
                            "現已自動進入點選圓心模式，請點擊地圖。"
                        )
                        self.on_pick_circle_center()
                        return

                    # 圓心模式：直接用圓心 + 最大半徑生成路徑
                    c_lat, c_lon = self.circle_center
                    max_r    = self.flight_params.get('circle_max_radius', 200.0)
                    alt_step = self.flight_params.get('circle_alt_step', 0.0)
                    min_r    = self.flight_params.get('circle_min_radius', 0.0)
                    spacing  = self.flight_params['spacing']
                    fw       = is_fixed_wing
                    tr       = self.flight_params.get('turn_radius', 50.0)
                    alt      = self.flight_params['altitude']

                    spiral_curvature = self.flight_params.get('spiral_curvature', 1.0)
                    spiral_alt_step  = self.flight_params.get('spiral_alt_step', 0.0)

                    circle_planner = CoveragePlanner()
                    if algorithm == 'spiral':
                        path = circle_planner.plan_spiral_from_center(
                            c_lat, c_lon, max_r, spacing,
                            is_fixed_wing=fw, turn_radius=tr,
                            curvature=spiral_curvature,
                            alt_step=spiral_alt_step,
                            base_alt=alt,
                        )
                        _spiral_alts = circle_planner.get_spiral_altitudes()
                    else:  # circular
                        path = circle_planner.plan_circle_survey_from_center(
                            c_lat, c_lon, max_r, spacing,
                            base_alt=alt, alt_step=alt_step,
                            min_radius_m=min_r,
                        )
                        _spiral_alts = []

                    # ── NFZ 繞行修正（多旋翼圓心模式）────────────────────
                    if self.nfz_zones and path:
                        path = self._apply_nfz_correction_multirotor_latlon(
                            path, c_lat, c_lon
                        )

                    if not path:
                        QMessageBox.warning(self, "路徑為空", "無法生成路徑，請檢查參數設定（半徑/間距）。")
                        return

                    _orig_n = len(path)
                    if _orig_n > _WP_LIMIT:
                        path, _spiral_alts = CoveragePlanner.limit_path_waypoints(
                            path, _WP_LIMIT, _spiral_alts if _spiral_alts else None
                        )
                        _trimmed = True
                    else:
                        _trimmed = False
                    self.waypoints = path
                    self.spiral_waypoint_altitudes = _spiral_alts
                    self.sub_paths = []

                    # 確保圓形資料是最新的（不觸發額外渲染）
                    self.map_widget._circle_center = (c_lat, c_lon)
                    self.map_widget._circle_radius_m = max_r
                    # 顯示路徑（同時包含圓形，只渲染一次）
                    self.map_widget.display_path(path, alt)

                    # 統計
                    total_distance = sum(
                        self._haversine(path[i], path[i+1]) for i in range(len(path)-1)
                    )
                    _wp_txt = f"航點: {len(path)}"
                    if _trimmed:
                        _wp_txt += f" (原{_orig_n},已簡化)"
                    self.waypoint_label.setText(_wp_txt)
                    self.distance_label.setText(f"距離: {total_distance:.0f}m")
                    self.mission_panel.update_mission_stats({
                        'waypoint_count': len(path),
                        'total_distance': total_distance,
                        'estimated_time': total_distance / max(self.flight_params['speed'], 0.1),
                        'area': math.pi * max_r ** 2,
                        'regions': 1,
                    })
                    self.statusBar().showMessage(f"路徑生成完成：{len(path)} 個航點", 5000)

                    # ── 任務預覽彈窗 ──────────────────────────────────────
                    _speed = max(self.flight_params.get('speed', 15.0), 0.1)
                    _est_s = total_distance / _speed
                    _mode_name = '螺旋掃描' if algorithm == 'spiral' else '同心圓掃描'
                    _wp_note = (f"（原 {_orig_n} 點，已簡化至 {len(path)}）" if _trimmed else "")
                    QMessageBox.information(
                        self, "任務預覽",
                        f"模式: {_mode_name}\n"
                        f"中心: {c_lat:.6f}°, {c_lon:.6f}°\n"
                        f"最大半徑: {max_r:.0f} m   間距: {spacing:.0f} m\n\n"
                        f"航點數: {len(path)} {_wp_note}\n"
                        f"總飛行距離: {total_distance:.0f} m\n"
                        f"預估飛行時間: {_est_s / 60:.1f} min\n"
                        f"覆蓋面積: {math.pi * max_r ** 2 / 10000:.2f} ha"
                    )
                    return  # skip the rest of the elif block
                # ── END 圓心模式 ──────────────────────────────────────────────

                # 覆蓋路徑規劃（Grid/Spiral/Circular，多邊形模式）
                planner = CoveragePlanner()
                if algorithm == 'grid':
                    pattern = ScanPattern.GRID
                elif algorithm == 'spiral':
                    pattern = ScanPattern.SPIRAL
                else:
                    pattern = ScanPattern.CIRCULAR

                params = CoverageParameters(
                    spacing=self.flight_params['spacing'],
                    angle=self.flight_params['angle'],
                    pattern=pattern,
                    is_fixed_wing=is_fixed_wing,
                    turn_radius=self.flight_params.get('turn_radius', 50.0),
                    smooth_turns=is_fixed_wing,
                    circle_base_altitude=self.flight_params['altitude'],
                )

                n_regions = self.flight_params.get('subdivisions', 1)
                spacing_m = self.flight_params.get('region_spacing', 0.0)
                gap_spacings_m = self.flight_params.get('gap_spacings_m') or None
                v_spacing_m = self.flight_params.get('v_spacing_m', 0.0) or None

                if n_regions > 1:
                    # 分區覆蓋：各子區域獨立規劃，顯示不同顏色
                    sub_paths = planner.plan_subdivided_coverage(
                        self.corners, params,
                        n_regions=n_regions,
                        spacing_m=spacing_m,
                        gap_spacings_m=gap_spacings_m,
                        v_spacing_m=v_spacing_m,
                    )
                    # ── NFZ 繞行修正（各子區域）──────────────────────────
                    if self.nfz_zones and self.corners:
                        _ref_lat = sum(p[0] for p in self.corners) / len(self.corners)
                        _ref_lon = sum(p[1] for p in self.corners) / len(self.corners)
                        sub_paths = [
                            self._apply_nfz_correction_multirotor_latlon(sp, _ref_lat, _ref_lon)
                            for sp in sub_paths
                        ]
                    self.sub_paths = sub_paths  # 保存各子區域路徑供匯出使用
                    # 扁平化供匯出使用，各子區域之間不插入多餘航點
                    path = [pt for sp in sub_paths for pt in sp]
                    self.map_widget.display_paths(sub_paths, self.flight_params['altitude'])
                    # 顯示轉場路徑
                    self._update_transit_paths(planner, sub_paths)
                else:
                    self.sub_paths = []
                    path = planner.plan_coverage(self.corners, params)
                    # ── NFZ 繞行修正（單區域）────────────────────────────
                    if self.nfz_zones and self.corners:
                        _ref_lat = sum(p[0] for p in self.corners) / len(self.corners)
                        _ref_lon = sum(p[1] for p in self.corners) / len(self.corners)
                        path = self._apply_nfz_correction_multirotor_latlon(
                            path, _ref_lat, _ref_lon
                        )
                    self.map_widget.clear_transit_paths()

            elif algorithm == 'astar':
                # A* 路徑規劃（點對點）
                if len(self.corners) >= 2:
                    astar_planner = AStarPlanner(
                        collision_checker=None,
                        step_size=self.flight_params['spacing'],
                        heuristic='euclidean'
                    )
                    path = astar_planner.plan(
                        start=self.corners[0],
                        goal=self.corners[-1],
                        boundary=self.corners
                    )

            elif algorithm in ['rrt', 'rrt_star']:
                # RRT/RRT* 路徑規劃
                if len(self.corners) >= 2:
                    # 計算搜索區域
                    lats = [c[0] for c in self.corners]
                    lons = [c[1] for c in self.corners]
                    search_area = (min(lats), min(lons), max(lats), max(lons))

                    # 創建空的碰撞檢測器（無障礙物）
                    collision_checker = CollisionChecker()

                    if algorithm == 'rrt':
                        rrt_planner = RRTPlanner(
                            collision_checker=collision_checker,
                            step_size=0.0001,  # 經緯度單位
                            goal_sample_rate=0.1,
                            max_iter=1000
                        )
                    else:
                        rrt_planner = RRTStarPlanner(
                            collision_checker=collision_checker,
                            step_size=0.0001,
                            goal_sample_rate=0.1,
                            max_iter=1000,
                            search_radius=0.0005
                        )

                    path = rrt_planner.plan(
                        start=self.corners[0],
                        goal=self.corners[-1],
                        search_area=search_area
                    )

            elif algorithm == 'dijkstra':
                # Dijkstra 使用與 A* 相同的邏輯
                if len(self.corners) >= 2:
                    astar_planner = AStarPlanner(
                        collision_checker=None,
                        step_size=self.flight_params['spacing'],
                        heuristic='euclidean',
                        heuristic_weight=0.0  # weight=0 等同於 Dijkstra
                    )
                    path = astar_planner.plan(
                        start=self.corners[0],
                        goal=self.corners[-1],
                        boundary=self.corners
                    )

            elif algorithm == 'dwa':
                # DWA 需要即時規劃，這裡生成直線路徑作為全域參考
                QMessageBox.information(
                    self, "DWA 演算法",
                    "DWA (動態窗口) 是局域即時規劃演算法，\n"
                    "需要配合飛行控制器使用。\n\n"
                    "目前生成直線路徑作為參考。"
                )
                path = self.corners.copy()

            else:
                # 預設使用 Grid
                planner = CoveragePlanner()
                params = CoverageParameters(
                    spacing=self.flight_params['spacing'],
                    angle=self.flight_params['angle'],
                    pattern=ScanPattern.GRID
                )
                path = planner.plan_coverage(self.corners, params)

            if not path:
                QMessageBox.warning(self, "路徑生成失敗", "無法生成覆蓋路徑，請檢查邊界點設定")
                return

            # 計算統計資訊（使用 CoveragePlanner 工具函數）
            coverage_planner = CoveragePlanner()
            area = coverage_planner.calculate_coverage_area(self.corners)
            mission_time = coverage_planner.estimate_mission_time(path, self.flight_params['speed'])

            # 計算總飛行距離
            total_distance = 0.0
            for i in range(len(path) - 1):
                lat1, lon1 = path[i]
                lat2, lon2 = path[i + 1]
                dlat = (lat2 - lat1) * 111111.0
                dlon = (lon2 - lon1) * 111111.0 * math.cos(math.radians((lat1 + lat2) / 2))
                total_distance += math.sqrt(dlat**2 + dlon**2)

            # 若超出 MP 匯入上限，自動簡化
            path, _trimmed, _orig_n = self._trim_to_wp_limit(path, "覆蓋路徑")

            # 儲存航點
            self.waypoints = path

            # 在地圖上顯示路徑
            # 多區域時 display_paths 已在規劃迴圈中呼叫，此處只處理單區域
            n_reg = self.flight_params.get('subdivisions', 1)
            if n_reg <= 1:
                self.map_widget.display_path(path, self.flight_params['altitude'])

            # 更新狀態列
            _wp_txt = f"航點: {len(path)}"
            if _trimmed:
                _wp_txt += f" (原{_orig_n},已簡化)"
            self.waypoint_label.setText(_wp_txt)
            self.distance_label.setText(f"距離: {total_distance:.0f}m")

            # 顯示結果
            region_info = (f"  分割區域: {n_reg} 個\n" if n_reg > 1 else "")
            _trim_note = (f" (原 {_orig_n}，已簡化至 {_WP_LIMIT} 上限)" if _trimmed else "")
            QMessageBox.information(
                self, "路徑生成完成",
                f"覆蓋路徑已生成！\n\n"
                f"邊界點: {len(self.corners)} 個\n"
                f"航點數: {len(path)} 個{_trim_note}\n"
                f"覆蓋面積: {area:.0f} m²\n"
                f"總飛行距離: {total_distance:.0f} m\n"
                f"預估飛行時間: {mission_time/60:.1f} 分鐘\n\n"
                f"參數:\n"
                f"  高度: {self.flight_params['altitude']} m\n"
                f"  速度: {self.flight_params['speed']} m/s\n"
                f"  間距: {self.flight_params['spacing']} m\n"
                f"  角度: {self.flight_params['angle']}°\n"
                f"{region_info}"
                f"  演算法: {algorithm}"
            )

            # 啟用匯出按鈕並更新任務統計
            self.mission_panel.update_mission_stats({
                'waypoint_count': len(path),
                'total_distance': total_distance,
                'estimated_time': mission_time,
                'area': area,
                'regions': self.flight_params.get('subdivisions', 1),
            })

            self.statusBar().showMessage(f"路徑生成完成：{len(path)} 個航點", 5000)
            logger.info(f"路徑生成完成：{len(path)} 個航點，距離 {total_distance:.0f}m")

        except Exception as e:
            logger.error(f"預覽失敗: {e}")
            import traceback
            traceback.print_exc()
            QMessageBox.critical(self, "預覽錯誤", f"生成路徑時發生錯誤：\n{str(e)}")
    
    @staticmethod
    def _calc_bearing(lat1, lon1, lat2, lon2) -> float:
        """計算兩點間方位角（度，0=北，順時針）"""
        lat1r = math.radians(lat1)
        lat2r = math.radians(lat2)
        dlon  = math.radians(lon2 - lon1)
        y = math.sin(dlon) * math.cos(lat2r)
        x = math.cos(lat1r) * math.sin(lat2r) - math.sin(lat1r) * math.cos(lat2r) * math.cos(dlon)
        return (math.degrees(math.atan2(y, x)) + 360) % 360

    # ──────────────────────────────────────────────────────────────────────────
    # 固定翼專用：三階段任務規劃
    # ──────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _trim_to_wp_limit(path, label: str = "") -> tuple:
        """
        若路徑超過 _WP_LIMIT，使用 RDP 自動簡化後回傳。
        回傳 (trimmed_path, was_trimmed, original_count)
        """
        orig_n = len(path)
        if orig_n <= _WP_LIMIT:
            return list(path), False, orig_n
        trimmed, _ = CoveragePlanner.limit_path_waypoints(path, _WP_LIMIT)
        if label:
            logger.warning(
                f"[WP 限制] {label}: {orig_n} → {len(trimmed)} 航點"
                f"（Mission Planner 上限 {_WP_LIMIT}）"
            )
        return trimmed, True, orig_n

    def _build_fw_params(self) -> FixedWingParameters:
        """從 flight_params 建立 FixedWingParameters"""
        import math
        fp       = self.flight_params
        speed    = fp.get('speed', 18.0)
        bank_deg = fp.get('fw_max_bank_deg', 45.0)
        tan_b    = math.tan(math.radians(max(bank_deg, 1.0)))
        r_min    = speed ** 2 / (9.81 * tan_b)          # R_min = V²/(g·tan φ)
        turn_radius_m = max(fp.get('turn_radius', 50.0), r_min)  # 不得低於 R_min
        return FixedWingParameters(
            cruise_speed_mps=speed,
            mission_altitude_m=fp.get('altitude', 100.0),
            turn_radius_m=turn_radius_m,
            takeoff_bearing_deg=fp.get('fw_takeoff_bearing', 0.0),
            takeoff_runway_m=fp.get('fw_runway_length', 50.0),
            takeoff_climb_alt_m=fp.get('altitude', 100.0) * 0.3,
            landing_bearing_deg=fp.get('fw_landing_bearing', 180.0),
            pattern_altitude_m=fp.get('fw_pattern_alt', 80.0),
            downwind_offset_m=fp.get('fw_downwind_offset', 200.0),
            pattern_leg_length_m=fp.get('fw_pattern_leg', 300.0),
            final_approach_dist_m=fp.get('fw_final_dist', 400.0),
            landing_rollout_m=fp.get('fw_landing_rollout', 0.0),
            scan_spacing_m=fp.get('spacing', 50.0),
            use_autoland=bool(fp.get('fw_autoland', False)),
        )

    def _preview_fixed_wing_mission(self, algorithm: str):
        """
        固定翼三階段任務規劃與地圖顯示

        顯示三種顏色路徑：
          黃色 - 起飛路徑
          藍色 - 主任務（螺旋/同心圓）
          橙色 - 五邊進場降落

        返回扁平化的路徑列表（lat, lon），供統計使用；
        出錯時返回 None。
        """
        try:
            fw_params = self._build_fw_params()
            scan_mode = algorithm  # 'spiral' | 'circular' | 'grid'
            scan_angle = self.flight_params.get('angle', 0.0)   # 網格掃描角度（度）

            # 決定中心點與任務多邊形：優先使用圓心模式，其次使用邊界點
            # 網格掃描必須使用邊界多邊形，圓心模式下跳過圓形多邊形邏輯
            if scan_mode == 'grid':
                # 網格掃描需要實際多邊形邊界
                if len(self.corners) < 3:
                    QMessageBox.warning(
                        self, "缺少區域定義",
                        "固定翼網格掃描需要在地圖上點選至少 3 個邊界點。"
                    )
                    return None
                center_lat = sum(c[0] for c in self.corners) / len(self.corners)
                center_lon = sum(c[1] for c in self.corners) / len(self.corners)
                mission_polygon = self.corners
            elif self.circle_center is not None:
                center_lat, center_lon = self.circle_center
                max_r = self.flight_params.get('circle_max_radius', 200.0)
                # 以 16 邊形近似圓形多邊形，供 plan_coverage 計算半徑用
                mission_polygon = []
                for i in range(16):
                    angle = math.radians(i * 22.5)
                    dlat = max_r * math.cos(angle) / 111111.0
                    dlon = max_r * math.sin(angle) / (
                        111111.0 * math.cos(math.radians(center_lat)))
                    mission_polygon.append(
                        (center_lat + dlat, center_lon + dlon))
            elif len(self.corners) >= 3:
                center_lat = sum(c[0] for c in self.corners) / len(self.corners)
                center_lon = sum(c[1] for c in self.corners) / len(self.corners)
                mission_polygon = self.corners
            else:
                QMessageBox.warning(
                    self, "缺少區域定義",
                    "請先設定飛行區域：\n"
                    "  • 螺旋/同心圓：按「從地圖點選圓心」後點擊地圖\n"
                    "  • 網格：在地圖上點選至少 3 個邊界點"
                )
                return None

            planner = FixedWingPlanner()
            n_regions = self.flight_params.get('subdivisions', 1)
            spacing_m = self.flight_params.get('region_spacing', 0.0)

            # 起飛/降落點：優先使用獨立設定的 home_point，否則沿用掃描區中心
            if self.home_point is not None:
                home_lat, home_lon = self.home_point
            else:
                home_lat, home_lon = center_lat, center_lon

            # 進場方向 = home → survey_center（兩點距離 > 10m 才有意義）
            _dist_to_center = self._haversine((home_lat, home_lon), (center_lat, center_lon))
            if _dist_to_center > 10.0:
                _dlat_r = math.radians(center_lat - home_lat)
                _dlon_r = math.radians(center_lon - home_lon)
                _x = math.sin(_dlon_r) * math.cos(math.radians(center_lat))
                _y = (math.cos(math.radians(home_lat)) * math.sin(math.radians(center_lat))
                      - math.sin(math.radians(home_lat)) * math.cos(math.radians(center_lat))
                      * math.cos(_dlon_r))
                approach_bearing = (math.degrees(math.atan2(_x, _y)) + 360) % 360
            else:
                approach_bearing = fw_params.takeoff_bearing_deg

            # 共用起飛 / 降落路徑（不論分區與否皆相同）
            takeoff_path = planner.generate_takeoff_path(home_lat, home_lon, fw_params)
            landing_path = planner.generate_landing_path(home_lat, home_lon, fw_params)
            takeoff_ll = [(lat, lon) for lat, lon, _ in takeoff_path]
            landing_ll = [(lat, lon) for lat, lon, _ in landing_path]

            # 轉彎半徑合規性檢測（共用）
            check = TurnRadiusChecker.check(
                fw_params.turn_radius_m,
                fw_params.cruise_speed_mps,
                self.current_vehicle_model,
            )
            turn_status = {
                'ok': '符合規格',
                'warning': '接近極限（建議增大）',
                'critical': '超出極限！請立即修改',
            }.get(check['status'], '')

            def _path_dist(pts):
                return sum(self._haversine(pts[i], pts[i + 1]) for i in range(len(pts) - 1))

            if n_regions > 1:
                # ── 多分區固定翼掃描 ────────────────────────────────────────
                if scan_mode == 'spiral':
                    _pattern = ScanPattern.SPIRAL
                elif scan_mode == 'grid':
                    _pattern = ScanPattern.GRID
                else:
                    _pattern = ScanPattern.CIRCULAR
                coverage_params = CoverageParameters(
                    spacing=fw_params.scan_spacing_m,
                    angle=scan_angle,
                    pattern=_pattern,
                    is_fixed_wing=True,
                    turn_radius=fw_params.turn_radius_m,
                    smooth_turns=True,
                    circle_base_altitude=fw_params.mission_altitude_m,
                    circle_direction=fw_params.circle_direction,
                )
                cov_planner = CoveragePlanner()
                sub_scan_paths = cov_planner.plan_subdivided_coverage(
                    mission_polygon, coverage_params,
                    n_regions=n_regions,
                    spacing_m=spacing_m,
                )
                if not sub_scan_paths:
                    QMessageBox.warning(self, "路徑為空", "分區掃描路徑生成失敗，請檢查區域定義。")
                    return None

                self.fw_mission_result = None
                self.sub_paths = sub_scan_paths
                all_scan_pts = [pt for sp in sub_scan_paths for pt in sp]
                path = takeoff_ll + all_scan_pts + landing_ll
                path, _trimmed, _orig_n = self._trim_to_wp_limit(path, "固定翼多分區")
                self.waypoints = path

                self.map_widget.display_fw_paths(takeoff_ll, sub_scan_paths, landing_ll)

                takeoff_dist = _path_dist(takeoff_ll)
                landing_dist = _path_dist(landing_ll)
                scan_dists   = [_path_dist(sp) for sp in sub_scan_paths if len(sp) >= 2]
                total_dist   = takeoff_dist + sum(scan_dists) + landing_dist

                _wp_txt = f"航點: {len(path)}"
                if _trimmed:
                    _wp_txt += f" (原{_orig_n},已簡化)"
                self.waypoint_label.setText(_wp_txt)
                self.distance_label.setText(f"距離: {total_dist:.0f}m")
                self.mission_panel.update_mission_stats({
                    'waypoint_count': len(path),
                    'total_distance': total_dist,
                    'estimated_time': total_dist / max(self.flight_params.get('speed', 18.0), 0.1),
                    'area': CoveragePlanner().calculate_coverage_area(mission_polygon),
                    'regions': n_regions,
                })

                _fw_speed = max(self.flight_params.get('speed', 18.0), 0.1)
                scan_lines = "".join(
                    f"  區域 {i + 1}: {sd:.0f} m  ({sd / _fw_speed / 60:.1f} min)\n"
                    for i, sd in enumerate(scan_dists)
                )
                _takeoff_time = takeoff_dist / _fw_speed / 60
                _landing_time = landing_dist / _fw_speed / 60
                QMessageBox.information(
                    self, "固定翼多分區任務生成完成",
                    f"任務區域已分割為 {n_regions} 個子區域，各區域獨立掃描路徑。\n\n"
                    f"[起飛] {takeoff_dist:.0f} m  ({_takeoff_time:.1f} min)\n"
                    f"[掃描]\n{scan_lines}"
                    f"[降落] {landing_dist:.0f} m  ({_landing_time:.1f} min)\n\n"
                    f"總飛行距離: {total_dist:.0f} m\n"
                    f"預估總時間: {total_dist / _fw_speed / 60:.1f} min\n\n"
                    f"[轉彎半徑] {fw_params.turn_radius_m:.0f}m  "
                    f"傾斜: {check['required_bank']:.1f}°  {turn_status}"
                )
                logger.info(
                    f"固定翼多分區任務: {n_regions} 區, {len(path)} 航點, {total_dist:.0f}m"
                )
                return path

            else:
                # ── 單區域固定翼任務（三階段完整生成）──────────────────────
                # 螺旋/同心圓 + 圓心模式：使用 AdvancedScanGenerator（固定翼曲率約束）
                # 避免多邊形裁切把圓環切成不連續段，再直線相連形成八字路徑
                if scan_mode in ('spiral', 'circular') and self.circle_center is not None:
                    _alt = fw_params.mission_altitude_m
                    _max_r = self.flight_params.get('circle_max_radius', 200.0)
                    _mission_path, mission_latlon = self._generate_advanced_scan_mission(
                        scan_mode, center_lat, center_lon, _max_r,
                        fw_params, approach_bearing,
                    )
                    if _mission_path and fw_params.turn_radius_m > 0:
                        _mission_path = planner.smooth_path_with_arcs(
                            _mission_path, fw_params.turn_radius_m
                        )
                        mission_latlon = [(lat, lon) for lat, lon, _ in _mission_path]
                    result = {
                        'takeoff_path': takeoff_path,
                        'mission_path': _mission_path,
                        'mission_latlon': mission_latlon,
                        'landing_path': landing_path,
                        'full_path': takeoff_ll + mission_latlon + landing_ll,
                    }
                else:
                    if scan_mode == 'grid':
                        # 使用 CoveragePathPlanner（燈泡型轉彎）取代預設網格生成
                        result = self._generate_grid_with_coverage_planner(
                            mission_polygon, fw_params, scan_angle,
                            planner, takeoff_path, landing_path,
                            home_lat, home_lon,
                        )
                    else:
                        result = planner.generate_full_mission(
                            polygon=mission_polygon,
                            home_lat=home_lat,
                            home_lon=home_lon,
                            params=fw_params,
                            scan_pattern=scan_mode,
                            scan_angle_deg=scan_angle,
                        )

                # ── NFZ 路徑修正（繞行禁航區）───────────────────────────
                if self.nfz_zones:
                    _raw_mission_ll = result.get('mission_latlon', [])
                    if _raw_mission_ll:
                        _corrected_ll = self._apply_nfz_correction_latlon(
                            _raw_mission_ll, center_lat, center_lon
                        )
                        _alt_nfz = fw_params.mission_altitude_m
                        result['mission_latlon'] = _corrected_ll
                        result['mission_path'] = [
                            (lat, lon, _alt_nfz) for lat, lon in _corrected_ll
                        ]
                        result['full_path'] = (
                            takeoff_ll + _corrected_ll + landing_ll
                        )

                # ── 插入進場弧：起飛末端 → 任務第一點（Dubins 雙約束）─────
                _mission_ll = result.get('mission_latlon', [])
                _tkoff_path = result.get('takeoff_path', [])
                if _mission_ll and _tkoff_path:
                    _p_end   = (_tkoff_path[-1][0], _tkoff_path[-1][1])
                    _p_first = _mission_ll[0]

                    # 由任務前兩點推算進入航向（羅盤方位）
                    if len(_mission_ll) >= 2:
                        _dx = _mission_ll[1][1] - _mission_ll[0][1]
                        _dy = _mission_ll[1][0] - _mission_ll[0][0]
                        _cos0 = math.cos(math.radians(_mission_ll[0][0]))
                        _first_hdg = (math.degrees(
                            math.atan2(_dx * _cos0, _dy)
                        ) + 360) % 360
                    else:
                        _first_hdg = fw_params.takeoff_bearing_deg

                    # 優先使用 Dubins（同時滿足出發 + 到達航向約束）
                    _entry_arc = planner.connect_with_dubins_latlon(
                        _p_end[0], _p_end[1], fw_params.takeoff_bearing_deg,
                        _p_first[0], _p_first[1], _first_hdg,
                        fw_params.turn_radius_m,
                        fw_params.mission_altitude_m,
                        step_size_m=8.0,
                    )
                    # Fallback：若 Dubins 失敗則用原圓弧平滑
                    if not _entry_arc:
                        _entry_arc = planner.generate_entry_arc(
                            _p_end,
                            fw_params.takeoff_bearing_deg,
                            _p_first,
                            fw_params.turn_radius_m,
                            fw_params.mission_altitude_m,
                            min_turn_deg=5.0,
                        )

                    if _entry_arc:
                        _arc_ll = [(_la, _lo) for _la, _lo, _ in _entry_arc]
                        result['mission_latlon'] = _arc_ll + _mission_ll
                        result['mission_path'] = (
                            _entry_arc + result.get('mission_path', [])
                        )
                        result['full_path'] = (
                            [(la, lo) for la, lo, _ in result['takeoff_path']]
                            + result['mission_latlon']
                            + [(la, lo) for la, lo, _ in result['landing_path']]
                        )
                        logger.info(
                            f"插入進場弧：{len(_entry_arc)} 個弧段點，"
                            f"起飛→任務轉向平滑化"
                        )

                # ── 插入離場弧：任務最後一點 → 對齊下風邊 → 五邊進場 ────
                # 幾何關係：
                #   飛機在下風邊飛行方向 = runway_hdg（downwind_start → downwind_end）
                #   ∴ lead_in 須在 downwind_start 的 opp_hdg 方向（飛行路徑的前方）
                #   弧段以 lead_in 為目標，飛機抵達後航向 ≈ runway_hdg，
                #   再直線飛入 downwind_start，完全消除急轉彎。
                _mission_ll_ex = result.get('mission_latlon', [])
                _landing_path  = result.get('landing_path', [])
                if len(_mission_ll_ex) >= 2 and _landing_path:
                    _p_last           = _mission_ll_ex[-1]
                    _p_downwind_start = (_landing_path[0][0], _landing_path[0][1])
                    _pat_alt          = fw_params.pattern_altitude_m

                    # 飛機進入下風邊的飛行方向 = runway_hdg
                    # lead_in 在 downwind_start 的 opp_hdg 方向（= 路徑上游）
                    _runway_hdg = fw_params.landing_bearing_deg
                    _opp_hdg    = (_runway_hdg + 180.0) % 360.0

                    _lead_dist = max(2.0 * fw_params.turn_radius_m, 150.0)
                    _dlat_m = 1.0 / 111_111.0
                    _dlon_m = 1.0 / (111_111.0 * math.cos(
                        math.radians(_p_downwind_start[0])) + 1e-12)
                    # lead_in = downwind_start + lead_dist * opp_hdg（上游方向）
                    _lead_lat = (_p_downwind_start[0]
                                 + _lead_dist * math.cos(math.radians(_opp_hdg)) * _dlat_m)
                    _lead_lon = (_p_downwind_start[1]
                                 + _lead_dist * math.sin(math.radians(_opp_hdg)) * _dlon_m)
                    _p_lead_in = (_lead_lat, _lead_lon)

                    # 最後一段航向（倒數第二點 → 最後一點）
                    _lx = (math.sin(math.radians(_p_last[1] - _mission_ll_ex[-2][1]))
                           * math.cos(math.radians(_p_last[0])))
                    _ly = (math.cos(math.radians(_mission_ll_ex[-2][0]))
                           * math.sin(math.radians(_p_last[0]))
                           - math.sin(math.radians(_mission_ll_ex[-2][0]))
                           * math.cos(math.radians(_p_last[0]))
                           * math.cos(math.radians(_p_last[1] - _mission_ll_ex[-2][1])))
                    _h_last = (math.degrees(math.atan2(_lx, _ly)) + 360) % 360

                    # 弧段：任務末端 → lead_in（Dubins 雙約束：出發=_h_last, 到達=runway_hdg）
                    _exit_arc = planner.connect_with_dubins_latlon(
                        _p_last[0], _p_last[1], _h_last,
                        _p_lead_in[0], _p_lead_in[1], _runway_hdg,
                        fw_params.turn_radius_m,
                        _pat_alt,
                        step_size_m=8.0,
                    )
                    # Fallback：Dubins 失敗時用原圓弧平滑
                    if not _exit_arc:
                        _exit_arc = planner.generate_entry_arc(
                            _p_last,
                            _h_last,
                            _p_lead_in,
                            fw_params.turn_radius_m,
                            _pat_alt,
                            min_turn_deg=5.0,
                        )
                    _exit_ll   = ([(_la, _lo) for _la, _lo, _ in _exit_arc]
                                  + [_p_lead_in])
                    _exit_path = (_exit_arc
                                  + [(_lead_lat, _lead_lon, _pat_alt)])
                    result['mission_latlon'] = _mission_ll_ex + _exit_ll
                    result['mission_path']   = (
                        result.get('mission_path', []) + _exit_path
                    )
                    result['full_path'] = (
                        [(la, lo) for la, lo, _ in result['takeoff_path']]
                        + result['mission_latlon']
                        + [(la, lo) for la, lo, _ in result['landing_path']]
                    )
                    logger.info(
                        f"插入離場弧：{len(_exit_arc)} 個弧段點 + lead_in，"
                        f"runway_hdg={_runway_hdg:.0f}°，opp={_opp_hdg:.0f}°→五邊進場"
                    )

                # ── 最終任務段弧段平滑：消除進/離場弧接入殘留銳角 ─────
                if result.get('mission_path') and fw_params.turn_radius_m > 0:
                    _mp_final = planner.smooth_path_with_arcs(
                        result['mission_path'], fw_params.turn_radius_m
                    )
                    result['mission_path']   = _mp_final
                    result['mission_latlon'] = [(la, lo) for la, lo, _ in _mp_final]
                    result['full_path'] = (
                        [(la, lo) for la, lo, _ in result['takeoff_path']]
                        + result['mission_latlon']
                        + [(la, lo) for la, lo, _ in result['landing_path']]
                    )

                # ── 任務末端 → 五邊進場橋接 + lead_in 轉角弧平滑 ────────────────────
                # 問題：離場弧使飛機到達 lead_in 時的航向 = bearing(p_last→lead_in)，
                #        但 lead_in→downwind_start 的航向 = runway_hdg，
                #        兩者不同 → lead_in 處出現銳角。
                # 修正：
                #   1. 以 [arc_exit, lead_in, downwind_start] 做 3 點弧平滑，
                #      將 lead_in 轉角替換為內切圓弧，弧出口與 runway_hdg 對齊。
                #   2. 弧尾接上 downwind_start，消除藍橙路徑視覺斷線。
                _mp_tail = result.get('mission_path', [])
                _lp_head = result.get('landing_path', [])
                if len(_mp_tail) >= 2 and len(_lp_head) >= 1:
                    _bridge_pt3d  = _lp_head[0]          # downwind_start
                    _bridge_ll    = (_bridge_pt3d[0], _bridge_pt3d[1])

                    # 只對末端 3 點做弧平滑，避免干擾已平滑的前段路徑
                    _tail3        = [_mp_tail[-2], _mp_tail[-1], _bridge_pt3d]
                    _tail3_smooth = planner.smooth_path_with_arcs(
                        _tail3, fw_params.turn_radius_m
                    )
                    # _tail3_smooth[0] = arc_exit（已在 _mp_tail[:-1] 末端，跳過避免重複）
                    # _tail3_smooth[1:] = [t_in, arc_pts..., t_out, downwind_start]
                    _extended = _mp_tail[:-1] + _tail3_smooth[1:]

                    result['mission_path']   = _extended
                    result['mission_latlon'] = [(la, lo) for la, lo, _ in _extended]
                    result['full_path'] = (
                        [(la, lo) for la, lo, _ in result['takeoff_path']]
                        + result['mission_latlon']
                        + landing_ll
                    )
                    _arc_n = max(0, len(_tail3_smooth) - 3)
                    logger.info(
                        f"任務→五邊接縫弧：lead_in 轉角平滑，"
                        f"插入 {_arc_n} 個弧段點，"
                        f"downwind_start ({_bridge_ll[0]:.6f}, {_bridge_ll[1]:.6f})"
                    )

                self.fw_mission_result = result
                self.sub_paths = []
                path = result['full_path']
                path, _trimmed, _orig_n = self._trim_to_wp_limit(path, "固定翼單區任務")
                self.waypoints = path

                # 確保 MAVLink 匯出的任務航點也在 _WP_LIMIT 以內
                # 預算 = 限制 - 起飛點數 - 降落點數 - 固定指令頭(1)
                _tkoff_n = len(result.get('takeoff_path', []))
                _land_n  = len(result.get('landing_path', []))
                _mission_budget = max(10, _WP_LIMIT - _tkoff_n - _land_n - 1)
                _mp = result.get('mission_path', [])
                if len(_mp) > _mission_budget:
                    _mp_ll  = [(la, lo) for la, lo, _ in _mp]
                    _mp_alt = [al for _, _, al in _mp]
                    _mp_ll_trim, _mp_alt_trim = CoveragePlanner.limit_path_waypoints(
                        _mp_ll, _mission_budget, _mp_alt
                    )
                    if not _mp_alt_trim:
                        _mp_alt_trim = [(_mp[0][2] if _mp else 100.0)] * len(_mp_ll_trim)
                    result['mission_path']   = [(la, lo, al) for (la, lo), al
                                                in zip(_mp_ll_trim, _mp_alt_trim)]
                    result['mission_latlon'] = _mp_ll_trim
                    _trimmed = True

                mission_ll = result['mission_latlon']
                self.map_widget.display_fw_paths(takeoff_ll, mission_ll, landing_ll)
                # 提供完整 3D 高度資訊給 Cesium 地圖
                if hasattr(self.map_widget, 'set_fw_result'):
                    self.map_widget.set_fw_result(result)
                if self.nfz_zones:
                    self.map_widget.display_nfz_zones(self.nfz_zones)

                stats = FixedWingPlanner.estimate_mission_stats(
                    result, self.flight_params.get('speed', 18.0)
                )

                _wp_txt = f"航點: {stats['waypoint_count']}"
                if _trimmed:
                    _wp_txt = f"航點: {len(path)} (原{_orig_n},已簡化)"
                self.waypoint_label.setText(_wp_txt)
                total_dist = stats['total_dist_m']
                self.distance_label.setText(f"距離: {total_dist:.0f}m")
                self.mission_panel.update_mission_stats({
                    'waypoint_count': stats['waypoint_count'],
                    'total_distance': total_dist,
                    'estimated_time': stats['estimated_time_s'],
                    'area': CoveragePlanner().calculate_coverage_area(mission_polygon),
                    'regions': 1,
                })

                QMessageBox.information(
                    self, "固定翼任務生成完成",
                    f"三階段任務路徑已生成！\n\n"
                    f"[起飛階段]\n"
                    f"  方向: {fw_params.takeoff_bearing_deg:.0f}°  "
                    f"跑道: {fw_params.takeoff_runway_m:.0f}m\n"
                    f"  起飛距離: {stats['takeoff_dist_m']:.0f}m\n\n"
                    f"[主任務 ({scan_mode.upper()} 掃描)]\n"
                    f"  航點數: {len(result['mission_path'])} 個\n"
                    f"  掃描距離: {stats['mission_dist_m']:.0f}m\n\n"
                    f"[五邊進場降落]\n"
                    f"  進場方向: {fw_params.landing_bearing_deg:.0f}°  "
                    f"五邊高度: {fw_params.pattern_altitude_m:.0f}m\n"
                    f"  降落距離: {stats['landing_dist_m']:.0f}m\n\n"
                    f"總飛行距離: {total_dist:.0f}m\n"
                    f"預估時間: {stats['estimated_time_s'] / 60:.1f} min\n\n"
                    f"[轉彎半徑檢測]\n"
                    f"  半徑: {fw_params.turn_radius_m:.0f}m  "
                    f"所需傾斜: {check['required_bank']:.1f}°\n"
                    f"  狀態: {turn_status}"
                )
                logger.info(
                    f"固定翼任務生成: {stats['waypoint_count']} 個航點, "
                    f"總距離 {total_dist:.0f}m, 轉彎半徑={fw_params.turn_radius_m:.0f}m"
                )
                return path

        except Exception as e:
            logger.error(f"固定翼任務生成失敗: {e}")
            import traceback
            traceback.print_exc()
            QMessageBox.critical(self, "固定翼任務錯誤", f"生成路徑時發生錯誤：\n{str(e)}")
            return None

    # ══════════════════════════════════════════════════════════════════
    # 整合 3：FixedWingNFZPlanner — 禁航區管理與路徑修正
    # ══════════════════════════════════════════════════════════════════
    # ── NFZ 地圖繪製 ──────────────────────────────────────────
    def on_nfz_draw_polygon(self):
        """啟用 NFZ 多邊形繪製模式：點擊地圖新增頂點，雙擊或按完成按鈕結束"""
        self.map_widget.set_nfz_polygon_draw_mode(True)
        if hasattr(self.parameter_panel, 'nfz_finish_poly_btn'):
            self.parameter_panel.nfz_finish_poly_btn.setVisible(True)
        self.statusBar().showMessage("🚫 NFZ 模式：點擊地圖新增頂點，雙擊或按「完成多邊形」完成")

    def on_nfz_poly_finish(self):
        """完成 NFZ 多邊形繪製"""
        self.map_widget.finish_nfz_polygon()
        if hasattr(self.parameter_panel, 'nfz_finish_poly_btn'):
            self.parameter_panel.nfz_finish_poly_btn.setVisible(False)

    def on_nfz_draw_circle(self):
        """啟用 NFZ 圓形拖曳繪製模式"""
        self.map_widget.set_nfz_circle_draw_mode(True)
        self.statusBar().showMessage("🚫 NFZ 圓形模式：按住滑鼠拖曳定義圓形禁航區")

    def on_nfz_polygon_drawn(self, vertices: list):
        """接收地圖繪製完成的 NFZ 多邊形"""
        if hasattr(self.parameter_panel, 'nfz_finish_poly_btn'):
            self.parameter_panel.nfz_finish_poly_btn.setVisible(False)
        name = f"NFZ_Poly_{len(self.nfz_zones) + 1}"
        zone = {'type': 'polygon', 'vertices': vertices, 'name': name}
        self.nfz_zones.append(zone)
        self.on_nfz_changed(self.nfz_zones)
        self.statusBar().showMessage(f"✅ 已新增 NFZ 多邊形「{name}」({len(vertices)} 頂點)")

    def on_nfz_circle_drawn(self, lat: float, lon: float, radius_m: float):
        """接收地圖拖曳完成的 NFZ 圓形"""
        name = f"NFZ_Circle_{len(self.nfz_zones) + 1}"
        zone = {'type': 'circle', 'center': (lat, lon), 'radius': radius_m, 'name': name}
        self.nfz_zones.append(zone)
        self.on_nfz_changed(self.nfz_zones)
        self.statusBar().showMessage(f"✅ 已新增 NFZ 圓形「{name}」(r={radius_m:.0f}m)")

    def open_nfz_dialog(self):
        """開啟禁航區管理對話框"""
        from ui.dialogs.nfz_manager_dialog import NFZManagerDialog
        dialog = NFZManagerDialog(self, self.nfz_zones)
        dialog.nfz_changed.connect(self.on_nfz_changed)
        dialog.exec()

    def on_nfz_changed(self, nfz_zones: list):
        """更新禁航區清單，更新地圖顯示，並重新生成路徑"""
        self.nfz_zones = nfz_zones
        count = len(nfz_zones)
        logger.info(f"禁航區更新：{count} 個")
        # 更新 parameter_panel 計數顯示
        if hasattr(self.parameter_panel, 'nfz_count_label'):
            self.parameter_panel.nfz_count_label.setText(f"目前禁航區：{count} 個")
        # 更新地圖上的 NFZ 視覺化
        self.map_widget.display_nfz_zones(nfz_zones)
        # 重新生成路徑（若已有路徑）
        if self.waypoints:
            self.on_preview_paths()

    def _apply_nfz_correction_multirotor_latlon(
        self,
        path_latlon: list,
        ref_lat: float,
        ref_lon: float,
        safety_buffer_m: float = 5.0,
    ) -> list:
        """
        多旋翼路徑的 NFZ 繞行修正（Visibility Graph + Dijkstra）。

        多旋翼不需要固定翼的大轉彎半徑緩衝，
        使用 safety_buffer_m（預設 5m）作為 NFZ 邊界安全餘量。

        Args:
            path_latlon     : [(lat, lon), ...] 原始路徑
            ref_lat/ref_lon : 座標轉換參考原點
            safety_buffer_m : NFZ 邊界向外膨脹距離（公尺）

        Returns:
            修正後的 [(lat, lon), ...]
        """
        if not self.nfz_zones or not path_latlon:
            return path_latlon

        import math as _math
        from core.global_planner.nfz_planner import FixedWingNFZPlanner
        from core.base.fixed_wing_constraints import FixedWingConstraints
        from utils.math_utils import latlon_to_meters, meters_to_latlon

        # 建立等效約束：讓 r_min ≈ safety_buffer_m
        # r_min = V² / (g·tan(φ))，取 V=10m/s，解出 φ
        V = 10.0
        tan_phi = V ** 2 / (9.81 * max(safety_buffer_m, 0.5))
        bank_deg = min(80.0, _math.degrees(_math.atan(tan_phi)))
        constraints = FixedWingConstraints(
            cruise_airspeed_mps=V,
            max_bank_angle_deg=bank_deg,
            stall_speed_mps=4.0,
            safety_factor=1.0,
        )
        # buffer_factor=1.0 → buffer_distance = r_min ≈ safety_buffer_m
        nfz_planner = FixedWingNFZPlanner(constraints, buffer_factor=1.0)

        for zone in self.nfz_zones:
            if zone['type'] == 'polygon':
                nfz_planner.add_polygon_nfz(
                    zone['vertices'], zone['name'],
                    coord_type='latlon',
                    ref_latlon=(ref_lat, ref_lon),
                )
            elif zone['type'] == 'circle':
                clat, clon = zone['center']
                cx, cy = latlon_to_meters(clat, clon, ref_lat, ref_lon)
                nfz_planner.add_circle_nfz(
                    (cx, cy), zone['radius'], zone['name']
                )

        metric_path = [
            latlon_to_meters(lat, lon, ref_lat, ref_lon)
            for lat, lon in path_latlon
        ]

        correction = nfz_planner.correct_path(metric_path)

        if correction.is_modified:
            logger.info(
                f"多旋翼 NFZ 路徑修正：{correction.segments_rerouted} 段繞行，"
                f"增加 {correction.added_length:.0f}m，"
                f"航點 {len(path_latlon)} → {len(correction.corrected_path)}"
            )
        else:
            logger.info("多旋翼 NFZ 檢查：路徑無需修正")

        return [
            meters_to_latlon(x, y, ref_lat, ref_lon)
            for x, y in correction.corrected_path
        ]

    def _apply_nfz_correction_latlon(
        self,
        path_latlon: list,
        ref_lat: float,
        ref_lon: float,
    ) -> list:
        """
        對 lat/lon 路徑套用禁航區繞行修正。

        內部使用公尺座標計算，完成後轉回 lat/lon 返回。

        Args:
            path_latlon: [(lat, lon), ...] 原始路徑
            ref_lat, ref_lon: 座標轉換參考原點

        Returns:
            修正後的 [(lat, lon), ...]（若無 NFZ 或無交叉則原樣返回）
        """
        if not self.nfz_zones or not path_latlon:
            return path_latlon

        from core.global_planner.nfz_planner import FixedWingNFZPlanner
        from utils.math_utils import latlon_to_meters, meters_to_latlon

        fw_params = self._build_fw_params()
        constraints = self._build_fw_constraints(fw_params)
        nfz_planner = FixedWingNFZPlanner(constraints, buffer_factor=1.5)

        # 加入所有禁航區（轉為公尺座標）
        for zone in self.nfz_zones:
            if zone['type'] == 'polygon':
                nfz_planner.add_polygon_nfz(
                    zone['vertices'], zone['name'],
                    coord_type='latlon',
                    ref_latlon=(ref_lat, ref_lon),
                )
            elif zone['type'] == 'circle':
                clat, clon = zone['center']
                cx, cy = latlon_to_meters(clat, clon, ref_lat, ref_lon)
                nfz_planner.add_circle_nfz(
                    (cx, cy), zone['radius'], zone['name']
                )

        # 將路徑轉為公尺座標
        metric_path = [
            latlon_to_meters(lat, lon, ref_lat, ref_lon)
            for lat, lon in path_latlon
        ]

        correction = nfz_planner.correct_path(metric_path)

        if correction.is_modified:
            logger.info(
                f"NFZ 路徑修正：{correction.segments_rerouted} 段繞行，"
                f"增加 {correction.added_length:.0f}m，"
                f"航點 {len(path_latlon)} → {len(correction.corrected_path)}"
            )
        else:
            logger.info("NFZ 檢查：路徑無需修正")

        return [
            meters_to_latlon(x, y, ref_lat, ref_lon)
            for x, y in correction.corrected_path
        ]

    # ══════════════════════════════════════════════════════════════════
    # 輔助：固定翼約束建立
    # ══════════════════════════════════════════════════════════════════
    def _build_fw_constraints(self, fw_params: 'FixedWingParameters'):
        """
        從 FixedWingParameters 建立 FixedWingConstraints。

        以使用者設定的 turn_radius_m 反推等效傾斜角：
            R = V²/(g·tan φ)  →  φ = atan(V²/(g·R))
        safety_factor=1.0（不額外放大），使 get_min_turn_radius() 直接返回 turn_radius_m，
        確保 AdvancedScanGenerator / CoveragePathPlanner 的轉彎幾何與 UI 設定一致。
        """
        from core.base.fixed_wing_constraints import FixedWingConstraints
        import math as _math
        speed = fw_params.cruise_speed_mps
        stall = min(12.0, speed * 0.6)
        r = max(fw_params.turn_radius_m, 10.0)
        # tan φ = V²/(g·R)
        tan_phi = speed ** 2 / (9.81 * r)
        bank_deg = _math.degrees(_math.atan(tan_phi))
        bank_deg = max(5.0, min(bank_deg, 80.0))  # 安全夾限
        return FixedWingConstraints(
            cruise_airspeed_mps=max(speed, stall + 1.0),
            max_bank_angle_deg=bank_deg,
            stall_speed_mps=stall,
            safety_factor=1.0,  # 不放大：R_min 直接等於 turn_radius_m
        )

    def _poses_to_latlon(self, poses, ref_lat: float, ref_lon: float):
        """將 List[Pose3D]（公尺座標）轉回 [(lat, lon), ...]"""
        from utils.math_utils import meters_to_latlon
        return [meters_to_latlon(p.x, p.y, ref_lat, ref_lon) for p in poses]

    # ══════════════════════════════════════════════════════════════════
    # 整合 1：AdvancedScanGenerator — 螺旋 / 同心圓掃描
    # ══════════════════════════════════════════════════════════════════
    def _generate_advanced_scan_mission(
        self,
        scan_mode: str,
        center_lat: float,
        center_lon: float,
        max_radius: float,
        fw_params: 'FixedWingParameters',
        approach_bearing: float,
    ):
        """
        使用 AdvancedScanGenerator 生成螺旋或同心圓掃描路徑。

        Args:
            scan_mode: 'spiral' 或 'circular'
            center_lat/lon: 掃描圓心（地理座標）
            max_radius: 最大半徑 [m]
            fw_params: 固定翼飛行參數
            approach_bearing: 進場航向 [度]，用於螺旋起始角

        Returns:
            (_mission_path, mission_latlon)
            _mission_path: [(lat, lon, alt), ...]
            mission_latlon: [(lat, lon), ...]
        """
        from core.global_planner.advanced_scan_patterns import (
            AdvancedScanGenerator, SpiralParams, ConcentricParams
        )
        from core.trajectory.dubins_trajectory import DubinsTrajectoryGenerator

        constraints = self._build_fw_constraints(fw_params)
        dubins_gen = DubinsTrajectoryGenerator(constraints)
        adv_gen = AdvancedScanGenerator(
            constraints=constraints,
            dubins_gen=dubins_gen,
            altitude_m=fw_params.mission_altitude_m,
        )
        _alt = fw_params.mission_altitude_m
        spacing = fw_params.scan_spacing_m

        if scan_mode == 'spiral':
            direction = fw_params.circle_direction  # 1=CCW, -1=CW
            params = SpiralParams(
                center=(0.0, 0.0),
                max_radius=max_radius,
                spacing=spacing,
                direction=direction,
                start_angle_deg=approach_bearing,
            )
            adv_result = adv_gen.generate_spiral(params, inward=True)
        else:  # circular (同心圓)
            params = ConcentricParams(
                center=(0.0, 0.0),
                max_radius=max_radius,
                spacing=spacing,
                arc_step_deg=5.0,
            )
            adv_result = adv_gen.generate_concentric(params)

        if adv_result.warnings:
            for w in adv_result.warnings:
                logger.warning(f"AdvancedScan: {w}")

        if not adv_result.poses:
            # 退回舊方法
            logger.warning("AdvancedScanGenerator 無法生成路徑，退回 CoveragePlanner")
            _cov = CoveragePlanner()
            if scan_mode == 'spiral':
                _sp_curv = self.flight_params.get('spiral_curvature', 1.0)
                _sp_alt_step = self.flight_params.get('spiral_alt_step', 0.0)
                mission_latlon = _cov.plan_spiral_from_center(
                    center_lat, center_lon, max_radius,
                    spacing, is_fixed_wing=True,
                    turn_radius=fw_params.turn_radius_m,
                    curvature=_sp_curv, alt_step=_sp_alt_step,
                    base_alt=_alt, entry_bearing_deg=approach_bearing,
                )
            else:
                _min_r = self.flight_params.get('circle_min_radius', 0.0)
                mission_latlon = _cov.plan_circle_survey_from_center(
                    center_lat, center_lon, max_radius, spacing,
                    base_alt=_alt, direction=fw_params.circle_direction,
                    min_radius_m=_min_r, start_angle_deg=approach_bearing,
                )
            _mission_path = [(lat, lon, _alt) for lat, lon in mission_latlon]
            return _mission_path, mission_latlon

        mission_latlon = self._poses_to_latlon(adv_result.poses, center_lat, center_lon)
        _mission_path = [(lat, lon, _alt) for lat, lon in mission_latlon]
        logger.info(
            f"AdvancedScan ({scan_mode}): {len(mission_latlon)} 航點, "
            f"總長={adv_result.total_length:.0f}m, "
            f"可行={'是' if adv_result.is_feasible else '否'}"
        )
        return _mission_path, mission_latlon

    # ══════════════════════════════════════════════════════════════════
    # 整合 2：CoveragePathPlanner — 固定翼網格掃描（燈泡型轉彎）
    # ══════════════════════════════════════════════════════════════════
    def _generate_grid_with_coverage_planner(
        self,
        mission_polygon,
        fw_params: 'FixedWingParameters',
        scan_angle: float,
        planner: 'FixedWingPlanner',
        takeoff_path,
        landing_path,
        home_lat: float,
        home_lon: float,
    ) -> dict:
        """
        使用 CoveragePathPlanner（Bulb Turn 燈泡型轉彎）生成固定翼網格掃描路徑。
        取代原本 generate_full_mission() 中的網格覆蓋邏輯。

        Returns:
            與 generate_full_mission() 相容的 result dict
        """
        from core.global_planner.coverage_path_planner import CoveragePathPlanner
        from core.trajectory.dubins_trajectory import DubinsTrajectoryGenerator
        from utils.math_utils import latlon_to_meters, meters_to_latlon

        constraints = self._build_fw_constraints(fw_params)
        dubins_gen = DubinsTrajectoryGenerator(constraints)
        cov_planner = CoveragePathPlanner(
            constraints=constraints,
            dubins_gen=dubins_gen,
            altitude_m=fw_params.mission_altitude_m,
            airspeed_mps=fw_params.cruise_speed_mps,
        )

        # 以第一個角點為參考原點轉換到公尺座標
        ref_lat = sum(p[0] for p in mission_polygon) / len(mission_polygon)
        ref_lon = sum(p[1] for p in mission_polygon) / len(mission_polygon)
        metric_poly = [
            latlon_to_meters(lat, lon, ref_lat, ref_lon)
            for lat, lon in mission_polygon
        ]

        cov_result = cov_planner.plan_coverage(
            polygon=metric_poly,
            scan_angle_deg=scan_angle,
            spacing_m=fw_params.scan_spacing_m,
            coord_type="metric",
        )

        if cov_result.warnings:
            for w in cov_result.warnings:
                logger.warning(f"CoveragePathPlanner: {w}")

        _alt = fw_params.mission_altitude_m
        takeoff_ll = [(lat, lon) for lat, lon, _ in takeoff_path]
        landing_ll = [(lat, lon) for lat, lon, _ in landing_path]

        if not cov_result.poses_3d:
            # 退回原方法
            logger.warning("CoveragePathPlanner 無可用路徑，退回 generate_full_mission")
            return planner.generate_full_mission(
                polygon=mission_polygon,
                home_lat=home_lat,
                home_lon=home_lon,
                params=fw_params,
                scan_pattern='grid',
                scan_angle_deg=scan_angle,
            )

        mission_latlon = self._poses_to_latlon(cov_result.poses_3d, ref_lat, ref_lon)
        _mission_path = [(lat, lon, _alt) for lat, lon in mission_latlon]

        logger.info(
            f"CoveragePathPlanner: {cov_result.num_scan_lines} 掃描線, "
            f"{len(mission_latlon)} 航點, 總長={cov_result.total_length:.0f}m"
        )

        return {
            'takeoff_path': takeoff_path,
            'mission_path': _mission_path,
            'mission_latlon': mission_latlon,
            'landing_path': landing_path,
            'full_path': takeoff_ll + mission_latlon + landing_ll,
        }

    def on_export_waypoints(self):
        """匯出航點"""
        if not self.waypoints:
            QMessageBox.warning(self, "無資料", "請先設定邊界點並預覽路徑生成航點")
            return

        # ── 固定翼三階段任務匯出 ──────────────────────────────────────
        if self.current_vehicle_type == '固定翼' and self.fw_mission_result:
            self._export_fixed_wing_waypoints()
            return

        # ── 多台無人機分區匯出 ──────────────────────────────────────────
        if self.sub_paths and len(self.sub_paths) > 1:
            self._export_multi_uav_waypoints()
            return

        # ── 單台無人機：以高度為預設檔名 ────────────────────────────────
        altitude = self.flight_params['altitude']
        default_name = f"高度{altitude:.0f}m"

        # 開啟匯出對話框
        filepath, selected_filter = QFileDialog.getSaveFileName(
            self, "儲存航點檔案",
            default_name,
            "Mission Planner (*.waypoints);;QGC Waypoint Files (*.waypoints);;CSV Files (*.csv);;All Files (*)"
        )

        if not filepath:
            return

        try:
            speed    = self.flight_params['speed']
            yaw_spd  = self.flight_params.get('yaw_speed', 60.0)

            # ── CSV ──────────────────────────────────────────────────────────
            if selected_filter == "CSV Files (*.csv)" or filepath.endswith('.csv'):
                if not filepath.endswith('.csv'):
                    filepath += '.csv'
                with open(filepath, 'w', encoding='utf-8') as f:
                    f.write("sequence,latitude,longitude,altitude,speed\n")
                    for i, (lat, lon) in enumerate(self.waypoints):
                        _wp_alt = (self.spiral_waypoint_altitudes[i]
                                   if self.spiral_waypoint_altitudes and i < len(self.spiral_waypoint_altitudes)
                                   else altitude)
                        f.write(f"{i},{lat:.8f},{lon:.8f},{_wp_alt:.1f},{speed:.1f}\n")
                format_name = "CSV"

            # ── Mission Planner ──────────────────────────────────────────────
            elif selected_filter == "Mission Planner (*.waypoints)":
                if not filepath.endswith('.waypoints'):
                    filepath += '.waypoints'

                waypoint_lines = ['QGC WPL 110']
                seq = 0

                # seq 0: DO_SET_HOME（以目前位置為 Home）
                waypoint_lines.append(create_waypoint_line(
                    seq=seq, command=179,  # MAV_CMD_DO_SET_HOME
                    lat=0.0, lon=0.0, alt=0.0,
                    current=0, autocontinue=1
                ))
                seq += 1

                # seq 1: DO_CHANGE_SPEED（設定巡航速度）
                waypoint_lines.append(create_waypoint_line(
                    seq=seq, command=178,  # MAV_CMD_DO_CHANGE_SPEED
                    param1=0.0,    # 0=Airspeed（旋翼機等效地速）
                    param2=speed,  # 目標速度 (m/s)
                    param3=0.0,
                    current=0, autocontinue=1
                ))
                seq += 1

                # seq 2+: 航點 + CONDITION_YAW 交錯
                lock_heading  = self.flight_params.get('lock_heading', False)
                fixed_heading = self.flight_params.get('heading_angle', 0.0)

                for i, (lat, lon) in enumerate(self.waypoints):
                    _wp_alt = (self.spiral_waypoint_altitudes[i]
                               if self.spiral_waypoint_altitudes and i < len(self.spiral_waypoint_altitudes)
                               else altitude)
                    waypoint_lines.append(create_waypoint_line(
                        seq=seq, command=16,  # MAV_CMD_NAV_WAYPOINT
                        lat=lat, lon=lon, alt=_wp_alt,
                        param1=0.0,  # hold time
                        param2=2.0,  # acceptance radius
                        current=0, autocontinue=1
                    ))
                    seq += 1

                    # 鎖定 Heading：每個航點都插入固定方向的 CONDITION_YAW
                    # 自動 Heading：只在相鄰航點之間插入，計算到下一點的方位角
                    if lock_heading:
                        heading = fixed_heading
                        waypoint_lines.append(create_waypoint_line(
                            seq=seq, command=115,  # MAV_CMD_CONDITION_YAW
                            param1=round(heading, 1),
                            param2=yaw_spd,
                            param3=0.0,  # 0=絕對角度
                            param4=0.0,  # 0=順時針
                            current=0, autocontinue=1
                        ))
                        seq += 1
                    elif i < len(self.waypoints) - 1:
                        next_lat, next_lon = self.waypoints[i + 1]
                        heading = self._calc_bearing(lat, lon, next_lat, next_lon)
                        waypoint_lines.append(create_waypoint_line(
                            seq=seq, command=115,  # MAV_CMD_CONDITION_YAW
                            param1=round(heading, 1),
                            param2=yaw_spd,
                            param3=0.0,  # 0=絕對角度
                            param4=0.0,  # 0=順時針
                            current=0, autocontinue=1
                        ))
                        seq += 1

                # 返航（RTL）
                waypoint_lines.append(create_waypoint_line(
                    seq=seq, command=20,  # MAV_CMD_NAV_RETURN_TO_LAUNCH
                    current=0, autocontinue=1
                ))

                write_waypoints(filepath, waypoint_lines)
                format_name = "Mission Planner"

            # ── QGC WPL 110 ──────────────────────────────────────────────────
            else:
                if not filepath.endswith('.waypoints'):
                    filepath += '.waypoints'

                home_lat, home_lon = self.waypoints[0]
                waypoint_lines = ['QGC WPL 110']

                # HOME 點
                waypoint_lines.append(create_waypoint_line(
                    seq=0, command=16,
                    lat=home_lat, lon=home_lon, alt=0.0,
                    current=1, autocontinue=1
                ))
                # 起飛點
                waypoint_lines.append(create_waypoint_line(
                    seq=1, command=22,  # MAV_CMD_NAV_TAKEOFF
                    lat=home_lat, lon=home_lon, alt=altitude,
                    param1=15.0,
                    current=0, autocontinue=1
                ))
                # 航點
                for i, (lat, lon) in enumerate(self.waypoints):
                    _wp_alt = (self.spiral_waypoint_altitudes[i]
                               if self.spiral_waypoint_altitudes and i < len(self.spiral_waypoint_altitudes)
                               else altitude)
                    waypoint_lines.append(create_waypoint_line(
                        seq=2 + i, command=16,
                        lat=lat, lon=lon, alt=_wp_alt,
                        param1=0.0, param2=2.0,
                        current=0, autocontinue=1
                    ))
                # RTL
                waypoint_lines.append(create_waypoint_line(
                    seq=2 + len(self.waypoints), command=20,
                    current=0, autocontinue=1
                ))

                write_waypoints(filepath, waypoint_lines)
                format_name = "QGC"

            QMessageBox.information(
                self, "匯出成功",
                f"航點檔案已匯出！\n\n"
                f"格式：{format_name}\n"
                f"檔案：{filepath}\n"
                f"航點數：{len(self.waypoints)}\n"
                f"高度：{altitude} m\n"
                f"速度：{speed} m/s"
            )
            self.statusBar().showMessage(f"已匯出 {len(self.waypoints)} 個航點至 {filepath}", 5000)
            logger.info(f"匯出航點 ({format_name}): {filepath}")

        except Exception as e:
            logger.error(f"匯出失敗: {e}")
            import traceback
            traceback.print_exc()
            QMessageBox.critical(self, "匯出錯誤", f"匯出時發生錯誤：\n{str(e)}")

    def _export_fixed_wing_waypoints(self):
        """匯出固定翼三階段任務 .waypoints 檔（Mission Planner 格式）"""
        altitude = self.flight_params['altitude']
        default_name = f"固定翼任務_{altitude:.0f}m"

        filepath, _ = QFileDialog.getSaveFileName(
            self, "儲存固定翼任務檔案",
            default_name,
            "Mission Planner (*.waypoints);;All Files (*)"
        )
        if not filepath:
            return
        if not filepath.endswith('.waypoints'):
            filepath += '.waypoints'

        try:
            fw_params = self._build_fw_params()
            planner = FixedWingPlanner()
            lines = planner.generate_mavlink_waypoints(
                mission_result=self.fw_mission_result,
                params=fw_params,
                speed_mps=self.flight_params.get('speed', 18.0),
            )

            from utils.file_io import write_waypoints
            write_waypoints(filepath, lines)

            stats = FixedWingPlanner.estimate_mission_stats(
                self.fw_mission_result, self.flight_params.get('speed', 18.0)
            )

            QMessageBox.information(
                self, "固定翼任務匯出成功",
                f"已匯出固定翼三階段任務！\n\n"
                f"檔案：{filepath}\n"
                f"總航點：{stats['waypoint_count']} 個\n"
                f"總距離：{stats['total_dist_m']:.0f} m\n"
                f"預估時間：{stats['estimated_time_s'] / 60:.1f} min\n\n"
                f"包含：起飛路徑 + {self.current_algorithm.upper()} 掃描 + 五邊進場降落"
            )
            self.statusBar().showMessage(f"已匯出固定翼任務至 {filepath}", 5000)
            logger.info(f"固定翼任務匯出: {filepath}")

        except Exception as e:
            logger.error(f"固定翼匯出失敗: {e}")
            import traceback
            traceback.print_exc()
            QMessageBox.critical(self, "匯出錯誤", f"匯出時發生錯誤：\n{str(e)}")

    def _export_multi_uav_waypoints(self):
        """多台無人機分區匯出：每個子區域生成一個獨立 .waypoints 檔案"""
        import os

        output_dir = QFileDialog.getExistingDirectory(
            self, "選擇輸出資料夾", "",
            QFileDialog.Option.ShowDirsOnly
        )
        if not output_dir:
            return

        altitude        = self.flight_params['altitude']
        speed           = self.flight_params['speed']
        yaw_spd         = self.flight_params.get('yaw_speed', 60.0)
        stagger_enabled = self.flight_params.get('transit_stagger_enabled', False)
        stagger_alt     = self.flight_params.get('transit_stagger_alt', 10.0)
        lock_heading    = self.flight_params.get('lock_heading', False)
        fixed_heading   = self.flight_params.get('heading_angle', 0.0)

        exported_files = []
        try:
            for i, sub_path in enumerate(self.sub_paths):
                if not sub_path:
                    continue

                # 此台無人機的實際飛行高度
                uav_alt = altitude + i * stagger_alt if stagger_enabled else altitude

                # 自動命名
                if stagger_enabled:
                    filename = f"區域{i+1}_高度{uav_alt:.0f}m.waypoints"
                else:
                    filename = f"區域{i+1}_高度{altitude:.0f}m.waypoints"
                filepath = os.path.join(output_dir, filename)

                # 生成 Mission Planner 格式航點
                waypoint_lines = ['QGC WPL 110']
                seq = 0

                # HOME
                waypoint_lines.append(create_waypoint_line(
                    seq=seq, command=179,
                    lat=0.0, lon=0.0, alt=0.0,
                    current=0, autocontinue=1
                ))
                seq += 1

                # DO_CHANGE_SPEED
                waypoint_lines.append(create_waypoint_line(
                    seq=seq, command=178,
                    param1=0.0, param2=speed, param3=0.0,
                    current=0, autocontinue=1
                ))
                seq += 1

                # 航點 + CONDITION_YAW
                for j, (lat, lon) in enumerate(sub_path):
                    waypoint_lines.append(create_waypoint_line(
                        seq=seq, command=16,
                        lat=lat, lon=lon, alt=uav_alt,
                        param1=0.0, param2=2.0,
                        current=0, autocontinue=1
                    ))
                    seq += 1

                    if lock_heading:
                        waypoint_lines.append(create_waypoint_line(
                            seq=seq, command=115,
                            param1=round(fixed_heading, 1),
                            param2=yaw_spd, param3=0.0, param4=0.0,
                            current=0, autocontinue=1
                        ))
                        seq += 1
                    elif j < len(sub_path) - 1:
                        next_lat, next_lon = sub_path[j + 1]
                        heading = self._calc_bearing(lat, lon, next_lat, next_lon)
                        waypoint_lines.append(create_waypoint_line(
                            seq=seq, command=115,
                            param1=round(heading, 1),
                            param2=yaw_spd, param3=0.0, param4=0.0,
                            current=0, autocontinue=1
                        ))
                        seq += 1

                # RTL
                waypoint_lines.append(create_waypoint_line(
                    seq=seq, command=20,
                    current=0, autocontinue=1
                ))

                write_waypoints(filepath, waypoint_lines)
                exported_files.append(filename)
                logger.info(f"匯出子區域 {i+1}: {filepath}")

            QMessageBox.information(
                self, "匯出成功",
                f"已匯出 {len(exported_files)} 個航點檔案至：\n{output_dir}\n\n"
                + "\n".join(exported_files)
            )
            self.statusBar().showMessage(
                f"已匯出 {len(exported_files)} 個子區域航點至 {output_dir}", 5000
            )
            logger.info(f"多台無人機匯出完成: {len(exported_files)} 個檔案")

        except Exception as e:
            logger.error(f"多台無人機匯出失敗: {e}")
            import traceback
            traceback.print_exc()
            QMessageBox.critical(self, "匯出錯誤", f"匯出時發生錯誤：\n{str(e)}")

    def on_new_mission(self):
        """創建新任務"""
        # 如果有未儲存的變更，詢問是否儲存
        if self.current_mission and self.has_unsaved_changes():
            reply = QMessageBox.question(
                self, "未儲存的變更",
                "當前任務有未儲存的變更，是否儲存？",
                QMessageBox.StandardButton.Yes | 
                QMessageBox.StandardButton.No | 
                QMessageBox.StandardButton.Cancel
            )
            
            if reply == QMessageBox.StandardButton.Yes:
                self.on_save_mission()
            elif reply == QMessageBox.StandardButton.Cancel:
                return
        
        # 清除當前任務
        self.on_clear_all_silent()
        
        # 創建新任務
        self.current_mission = self.mission_manager.create_mission("新任務")
        
        self.statusBar().showMessage("已創建新任務", 3000)
        logger.info("創建新任務")
    
    def on_open_mission(self):
        """開啟任務"""
        filepath, _ = QFileDialog.getOpenFileName(
            self, "開啟任務檔案",
            "",
            "Mission Files (*.json);;All Files (*)"
        )
        
        if filepath:
            try:
                mission = self.mission_manager.load_mission(filepath)
                self.current_mission = mission
                
                # TODO: 載入任務參數到 UI
                
                self.statusBar().showMessage(f"已載入任務：{mission.name}", 3000)
                logger.info(f"載入任務: {filepath}")
                
            except Exception as e:
                logger.error(f"載入任務失敗: {e}")
                QMessageBox.critical(self, "載入錯誤", f"載入任務時發生錯誤：\n{str(e)}")
    
    # ═════════════════════════════════════════════════════════════════
    #  專案檔 (.aeroplan) 保存 / 讀取
    # ═════════════════════════════════════════════════════════════════
    def on_save_project(self):
        """儲存當前 UI 狀態為 .aeroplan 專案檔"""
        from mission.project_io import (
            build_project_from_main_window, save_project,
        )
        path, _ = QFileDialog.getSaveFileName(
            self, '儲存 AeroPlan 專案', '', 'AeroPlan Project (*.aeroplan)'
        )
        if not path:
            return
        try:
            proj = build_project_from_main_window(self)
            save_project(proj, path)
            self.statusBar().showMessage(f'專案已儲存: {path}', 4000)
            logger.info(f'[Project] 儲存至 {path}')
        except Exception as e:
            logger.error(f'[Project] 儲存失敗: {e}', exc_info=True)
            QMessageBox.critical(self, '儲存失敗', str(e))

    def on_load_project(self):
        """從 .aeroplan 檔案復原 UI 狀態"""
        from mission.project_io import load_project, apply_project_to_main_window
        path, _ = QFileDialog.getOpenFileName(
            self, '讀取 AeroPlan 專案', '', 'AeroPlan Project (*.aeroplan)'
        )
        if not path:
            return
        try:
            proj = load_project(path)
            apply_project_to_main_window(proj, self)
            self.statusBar().showMessage(
                f'專案已載入: {path} '
                f'(邊界 {len(proj.corners)} 點, 打擊目標 {len(proj.strike_targets)} 個)',
                5000,
            )
            logger.info(f'[Project] 從 {path} 載入')
        except Exception as e:
            logger.error(f'[Project] 載入失敗: {e}', exc_info=True)
            QMessageBox.critical(self, '載入失敗', str(e))

    def on_save_mission(self):
        """儲存任務"""
        if not self.current_mission:
            QMessageBox.warning(self, "無任務", "沒有任務可儲存")
            return
        
        try:
            filepath = self.mission_manager.save_mission(self.current_mission)
            
            if filepath:
                self.statusBar().showMessage(f"任務已儲存", 3000)
                logger.info(f"儲存任務: {filepath}")
            else:
                QMessageBox.warning(self, "儲存失敗", "無法儲存任務")
                
        except Exception as e:
            logger.error(f"儲存任務失敗: {e}")
            QMessageBox.critical(self, "儲存錯誤", f"儲存時發生錯誤：\n{str(e)}")
    
    def on_clear_paths(self):
        """清除路徑"""
        self.map_widget.clear_paths()
        self.waypoints.clear()
        self.spiral_waypoint_altitudes.clear()
        self.waypoint_label.setText("航點: 0")
        self.distance_label.setText("距離: 0.0m")
        logger.info("已清除路徑")
    
    def on_clear_corners(self):
        """清除邊界"""
        self.map_widget.clear_corners()
        self.corners.clear()
        self.parameter_panel.update_corner_count(0)
        # 同時清除圓心設定
        self.circle_center = None
        self.picking_circle_center = False
        if hasattr(self, 'parameter_panel') and hasattr(self.parameter_panel, 'circle_center_label'):
            self.parameter_panel.circle_center_label.setText("圓心：尚未設定")
            self.parameter_panel.circle_center_label.setStyleSheet(
                "color: #888; font-size: 11px; font-style: italic;"
            )
        if hasattr(self.map_widget, 'clear_circle_overlay'):
            self.map_widget.clear_circle_overlay()
        # 同時清除起飛點設定
        self.home_point = None
        self.picking_home_point = False
        if hasattr(self, 'parameter_panel'):
            self.parameter_panel.clear_home_point_display()
        if hasattr(self.map_widget, 'clear_home_point_overlay'):
            self.map_widget.clear_home_point_overlay()
        logger.info("已清除邊界")
    
    def on_clear_all(self):
        """清除全部（帶確認）"""
        reply = QMessageBox.question(
            self, "確認清除",
            "確定要清除所有標記和路徑嗎？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            self.on_clear_all_silent()
    
    def on_clear_all_silent(self):
        """清除全部（不帶確認）"""
        self.on_clear_corners()
        self.on_clear_paths()
        self.obstacles.clear()
        logger.info("已清除全部")
    
    def on_reset_view(self):
        """重置視圖"""
        self.map_widget.reset_view()
    
    def on_toggle_grid(self):
        """切換網格顯示"""
        # TODO: 實現網格顯示切換
        QMessageBox.information(self, "網格顯示", "網格顯示功能開發中")
    
    def on_camera_config(self):
        """相機配置"""
        try:
            from ui.dialogs.camera_config import CameraConfigDialog
            dialog = CameraConfigDialog(self)
            dialog.exec()
        except ImportError:
            QMessageBox.information(self, "相機配置", "相機配置功能開發中")
    
    def on_vehicle_config(self):
        """飛行器配置"""
        try:
            from ui.dialogs.vehicle_config import VehicleConfigDialog
            dialog = VehicleConfigDialog(self)
            dialog.exec()
        except ImportError:
            QMessageBox.information(self, "飛行器配置", "飛行器配置功能開發中")
    
    def on_obstacle_manager(self):
        """障礙物管理"""
        try:
            from ui.dialogs.obstacle_manager import ObstacleManagerDialog

            dialog = ObstacleManagerDialog(self, self.obstacles)
            dialog.obstacles_changed.connect(self.on_obstacles_changed)
            dialog.exec()

        except ImportError as e:
            logger.error(f"無法載入障礙物管理對話框: {e}")
            QMessageBox.warning(self, "載入失敗", "無法載入障礙物管理功能")

    def on_open_polygon_editor(self):
        """開啟多邊形編輯器"""
        try:
            from ui.widgets.polygon_editor import PolygonEditorWindow

            # 創建編輯器視窗
            self.polygon_editor_window = PolygonEditorWindow(max_corners=MAX_CORNERS)

            # 如果已有角點，載入到編輯器
            if self.corners:
                self.polygon_editor_window.editor.set_corners(self.corners)

            # 連接信號 - 當編輯完成時同步角點
            self.polygon_editor_window.polygon_completed.connect(self._on_polygon_editor_completed)
            self.polygon_editor_window.editor.corners_changed.connect(self._on_polygon_editor_corners_changed)

            self.polygon_editor_window.show()
            logger.info("已開啟多邊形編輯器")

        except Exception as e:
            logger.error(f"開啟多邊形編輯器失敗: {e}")
            import traceback
            traceback.print_exc()
            QMessageBox.critical(self, "錯誤", f"無法開啟多邊形編輯器：\n{str(e)}")

    def _on_polygon_editor_completed(self, corners):
        """多邊形編輯器完成編輯"""
        self._sync_corners_from_editor(corners)
        QMessageBox.information(
            self, "角點已同步",
            f"已從編輯器同步 {len(corners)} 個角點到主視窗"
        )

    def _on_polygon_editor_corners_changed(self, corners):
        """多邊形編輯器角點變更（即時同步）"""
        self._sync_corners_from_editor(corners)

    def _sync_corners_from_editor(self, corners):
        """從編輯器同步角點"""
        # 清除現有角點
        self.corners.clear()
        self.map_widget.corners.clear()
        self.map_widget.markers.clear()

        # 添加新角點
        for lat, lon in corners:
            self.corners.append((lat, lon))

        # 重新初始化地圖並添加角點
        self.map_widget.init_map()
        for lat, lon in self.corners:
            self.map_widget.add_corner(lat, lon)

        # 更新 UI
        self.parameter_panel.update_corner_count(len(self.corners))
        self.update_statusbar()
        logger.info(f"已同步 {len(corners)} 個角點")

    def on_obstacles_changed(self, obstacles):
        """處理障礙物變更"""
        self.obstacles = obstacles
        logger.info(f"障礙物已更新: {len(obstacles)} 個")
    
    def on_show_help(self):
        """顯示說明"""
        help_text = """
        <h2>AeroPlan Studio — 使用說明</h2>
        <p><i>Collaborative UAV Mission Planning Suite</i></p>

        <h3>① 定義作業區域</h3>
        <ul>
            <li><b>新增邊界點：</b>在 2D 地圖上左鍵點擊（至少 3 點）</li>
            <li><b>移動邊界點：</b>直接拖曳地圖標記</li>
            <li><b>多邊形編輯器：</b>Ctrl+M 開啟，可精確輸入經緯度</li>
            <li><b>螺旋/同心圓中心：</b>右側面板「地圖拖曳定義圓形」於地圖上拖曳</li>
        </ul>

        <h3>② 規劃路徑</h3>
        <ul>
            <li><b>選擇載具：</b>右側面板選 多旋翼 / 固定翼 / VTOL 與型號</li>
            <li><b>選擇演算法：</b>Grid / Spiral / Circular / A* / RRT / RRT* / Dijkstra / DWA</li>
            <li><b>調整參數：</b>高度、速度、間距、重疊率、掃描角度…</li>
            <li><b>預覽：</b>按 Enter 或「預覽」按鈕</li>
            <li><b>2D ↔ 3D：</b>地圖頂端 🗺 2D 衛星 / 🌐 3D Cesium 切換</li>
        </ul>

        <h3>③ DCCPP 多機協同（固定翼/多旋翼）</h3>
        <ul>
            <li>切到 <b>DCCPP</b> 分頁，設定 UAV 數量、FOV、重疊率、轉彎半徑</li>
            <li>可載入 DEM 地形（GeoTIFF / .npy）→ 啟用 GDA 高度平滑</li>
            <li>按「DCCPP 最佳化」執行 GreedyAllocator → IDP → Dubins → AltitudePlanner</li>
            <li>3D 地圖會以分色顯示每架 UAV 的完整三維航線</li>
        </ul>

        <h3>④ SITL 模擬（單機 / 多機）</h3>
        <ul>
            <li>切到 <b>🛰 SITL</b> 分頁</li>
            <li><b>單機：</b>選 PLANE / COPTER → 🚀 啟動 → 自動 tcp:5760 連線</li>
            <li><b>多機：</b>先做完 DCCPP，再按 🚀 啟動，會依每台 UAV 起飛點生成 N 個獨立 SITL（各自 console、SYSID、無漂移）</li>
            <li><b>上傳任務：</b>按 uav_id 將每台對應路徑分發到對應 SITL</li>
            <li><b>📂 參數檔：</b>批次匯入外部 .param 設定</li>
            <li>HUD 即時顯示 模式 / ARMED / 姿態 / 速度 / 電量 / GPS / FC log</li>
        </ul>

        <h3>⑤ 匯出</h3>
        <ul>
            <li>Ctrl+E 匯出 QGC WPL 110 (.waypoints)，可直接給 Mission Planner / QGC</li>
            <li>群飛模式可逐機匯出獨立任務檔</li>
        </ul>

        <h3>快捷鍵</h3>
        <ul>
            <li><b>Enter / Space</b>：生成路徑</li>
            <li><b>Esc</b>：清除路徑</li>
            <li><b>Delete / Backspace</b>：刪除最後一個角點</li>
            <li><b>Ctrl+N / O / S</b>：新建 / 開啟 / 儲存任務</li>
            <li><b>Ctrl+E</b>：匯出航點</li>
            <li><b>Ctrl+R</b>：清除全部</li>
            <li><b>Ctrl+M</b>：多邊形編輯器</li>
        </ul>
        """

        QMessageBox.information(self, "AeroPlan Studio 使用說明", help_text)

    def on_about(self):
        """關於"""
        about_text = """
        <h2>AeroPlan Studio</h2>
        <p><b>Collaborative UAV Mission Planning Suite</b></p>
        <p>版本 2.5.0 · PyQt6 · MIT License</p>
        <p>面向多機協同作業的專業級無人機任務規劃平台，整合：</p>
        <ul>
            <li>2D Folium / 3D Cesium 雙模式地圖（離線可用）</li>
            <li>Grid / Spiral / Circular 覆蓋路徑與 A* / RRT / DWA 規劃</li>
            <li>DCCPP 多機協同最佳化（GreedyAllocator + IDP + Dubins + GDA）</li>
            <li>固定翼三階段任務（起飛 → 螺旋/同心圓 → 五邊進場降落）</li>
            <li>ArduPilot SITL 多機模擬整合 + Mission Planner 風格 HUD</li>
            <li>QGC WPL 110 / MAVLink 任務匯出</li>
            <li>DEM 真實 3D 地形顯示（CustomHeightmapTerrainProvider）</li>
        </ul>
        <p>舊名：UAV Path Planner / DWA_path_planner</p>
        <p>© 2026 AeroPlan Studio</p>
        """

        QMessageBox.about(self, "關於 AeroPlan Studio", about_text)
    
    # ==========================================
    # 輔助函數
    # ==========================================
    
    def has_unsaved_changes(self):
        """檢查是否有未儲存的變更"""
        # TODO: 實現變更檢測
        return False
    
    def update_statusbar(self):
        """更新狀態列"""
        # 更新航點數量
        self.waypoint_label.setText(f"航點: {len(self.waypoints)}")
        
        # 更新邊界點數量
        if self.corners:
            self.statusBar().showMessage(f"邊界點: {len(self.corners)} 個", 2000)
    
    def _on_dccpp_coverage_requested(self, params: dict):
        """
        處理 DCCPP 最佳化覆蓋請求

        流程：
        1. 驗證邊界點 >= 3
        2. 組裝 areas / drones 資料結構
        3. 呼叫 SwarmCoordinator.plan_coverage_dccpp()
        4. 將結果轉為 swarm_raw 格式並在地圖顯示
        5. 更新狀態列
        """
        from PyQt6.QtWidgets import QMessageBox
        try:
            if len(self.corners) < 3:
                QMessageBox.warning(
                    self, "邊界點不足",
                    "請先在地圖上設定至少 3 個邊界點，再執行 DCCPP 最佳化。"
                )
                return

            import math

            num_drones    = int(params.get('num_drones', 3))
            fov_width_m   = float(params.get('fov_width_m', 100.0))
            overlap_rate  = float(params.get('overlap_rate', 0.1))
            altitude      = float(params.get('altitude', 100.0))
            speed         = float(params.get('speed', 15.0))
            turn_radius   = float(params.get('turn_radius', 0.0))
            vehicle_type  = params.get('vehicle_type', 'multirotor')
            auto_scan     = bool(params.get('auto_scan_angle', True))
            enable_alt    = bool(params.get('enable_altitude', True))
            mounting_angle = float(params.get('mounting_angle_deg', 0.0))
            coord_mode    = params.get('coordination_mode', 'uncoordinated')
            dem_path      = params.get('dem_path', '')

            # ── 組裝區域 ──
            areas = [{'area_id': 1, 'polygon': list(self.corners), 'priority': 1.0}]

            # ── 組裝無人機 ──
            centroid_lat = sum(c[0] for c in self.corners) / len(self.corners)
            centroid_lon = sum(c[1] for c in self.corners) / len(self.corners)
            # 優先使用使用者設定的起飛點，否則用多邊形質心
            if self.home_point is not None:
                base_lat, base_lon = self.home_point
            else:
                base_lat, base_lon = centroid_lat, centroid_lon
            user_takeoff_hdg = float(params.get('takeoff_bearing', 0.0))
            takeoff_spacing_m = float(params.get('takeoff_spacing', 50.0))
            drones = []
            for i in range(num_drones):
                # 多機時沿跑道方向（垂直方向）排列，間距由使用者設定
                if num_drones > 1:
                    # 沿跑道的垂直方向（左右排列），以起飛點為中心對稱展開
                    perp_bearing_rad = math.radians((user_takeoff_hdg + 90.0) % 360.0)
                    offset_m = (i - (num_drones - 1) / 2.0) * takeoff_spacing_m
                    dN = offset_m * math.cos(perp_bearing_rad)
                    dE = offset_m * math.sin(perp_bearing_rad)
                    cos_lat = max(math.cos(math.radians(base_lat)), 1e-6)
                    pos = (base_lat + dN / 111320.0,
                           base_lon + dE / (111320.0 * cos_lat))
                else:
                    pos = (base_lat, base_lon)
                drones.append({
                    'drone_id': i + 1,
                    'name': f'UAV-{i + 1}',
                    'position': pos,
                    'speed': speed,
                    'heading': user_takeoff_hdg,
                    'turn_radius': turn_radius,
                    'vehicle_type': vehicle_type,
                    'fov_width': fov_width_m,
                })

            # ── 清除舊路徑（JS 方式，不重載頁面），避免與新 DCCPP 路徑重疊 ──
            self.map_widget.clear_all_display_layers_js()

            self.statusBar().showMessage("DCCPP 最佳化計算中…", 0)
            from PyQt6.QtWidgets import QApplication
            QApplication.processEvents()

            # ── 呼叫 DCCPP 求解器 ──
            coordinator = SwarmCoordinator()
            user_runway_length = float(params.get('runway_length', 500.0))
            user_landing_hdg = float(params.get('landing_bearing', 180.0))
            # 降落滑行距離：將觸地點沿進場反方向提前此距離，
            # 確保 DCCPP 規劃的降落點不會和起飛點重疊
            user_landing_rollout = float(params.get('landing_rollout', 0.0))
            dccpp_result = coordinator.plan_coverage_dccpp(
                areas=areas,
                drones=drones,
                coverage_width_m=fov_width_m,
                overlap_rate=overlap_rate,
                auto_scan_angle=auto_scan,
                coverage_altitude=altitude,
                terrain_func=None,
                enable_altitude=enable_alt,
                mounting_angle_deg=mounting_angle,
                dem_path=dem_path,
                coordination_mode=coord_mode,
                runway_bearing_deg=user_takeoff_hdg,
                runway_length_m=user_runway_length,
                landing_bearing_deg=user_landing_hdg,
                landing_rollout_m=user_landing_rollout,
            )

            # 儲存 DCCPP 參數供匯出時使用
            self.flight_params.update({
                'vehicle_type': vehicle_type,
                'altitude': altitude,
                'speed': speed,
                'turn_radius': turn_radius,
            })

            # ── 將 DCCPP 結果轉為 display_swarm_raw 格式 ──
            results = dccpp_result.get('results', {})
            total_makespan = dccpp_result.get('total_makespan', 0.0)
            allocation = dccpp_result.get('allocation', {})
            assembled_paths = dccpp_result.get('assembled_paths', None)

            # ── 為固定翼 UAV 添加起飛 / 降落路徑（受 auto_landing 開關控制）──
            auto_landing = bool(params.get('auto_landing', True))
            # 新版 swarm_coordinator 已在 BuiltPath 中自動補上 TAKEOFF/LANDING；
            # 偵測到第一個航點已是 TAKEOFF 時跳過舊 assembler 補丁，避免重複。
            _already_has_to_ld = bool(
                assembled_paths
                and any(
                    apath.waypoints
                    and getattr(apath.waypoints[0].segment_type, 'name', '') == 'TAKEOFF'
                    for apath in assembled_paths.values()
                )
            )
            # VTOL 不需要跑道式起降，垂直起降由匯出器處理
            if (auto_landing and assembled_paths and not _already_has_to_ld
                    and vehicle_type == 'fixed_wing' and turn_radius > 0):
                from core.trajectory.dccpp_path_assembler import DCCPPPathAssembler as _Asm
                _asm = _Asm(
                    default_altitude=altitude,
                    turn_radius=turn_radius,
                )
                # 使用起飛點或多邊形質心作為 Home
                home = self.home_point if self.home_point else (centroid_lat, centroid_lon)
                # 使用使用者設定的跑道參數
                user_takeoff_hdg = float(params.get('takeoff_bearing', 0.0))
                user_landing_hdg = float(params.get('landing_bearing', 180.0))
                user_pattern_alt = float(params.get('pattern_alt', 80.0))
                user_landing_rollout = float(params.get('landing_rollout', 0.0))
                for uav_id, apath in assembled_paths.items():
                    if not apath.waypoints:
                        continue
                    _asm.prepend_takeoff(
                        apath, home[0], home[1],
                        takeoff_bearing_deg=user_takeoff_hdg,
                        climb_alt_m=min(30.0, altitude * 0.3),
                    )
                    _asm.append_landing(
                        apath, home[0], home[1],
                        landing_bearing_deg=user_landing_hdg,
                        pattern_alt_m=user_pattern_alt,
                        landing_rollout_m=user_landing_rollout,
                    )

            # ── 避撞偵測與解衝突 ──
            _collision_enabled = bool(params.get('collision_avoidance', False))
            _collision_conflicts = []
            _collision_actions = []
            if _collision_enabled and assembled_paths and len(assembled_paths) >= 2:
                try:
                    from core.dccpp.collision_avoidance import CollisionAvoidance
                    _ca = CollisionAvoidance(
                        min_separation_m=float(params.get('min_separation_m', 100.0)),
                        alt_offset_m=float(params.get('alt_offset_m', 30.0)),
                        time_step_s=1.0,
                        cruise_speed_mps=speed,
                        strategy=params.get('avoid_strategy', 'altitude'),
                    )
                    _collision_conflicts, _collision_actions = _ca.run(assembled_paths)
                    if _collision_conflicts:
                        self.statusBar().showMessage(
                            f"避撞偵測: {len(_collision_conflicts)} 個衝突, "
                            f"已執行 {len(_collision_actions)} 個避撞動作",
                            5000,
                        )
                except Exception as _ca_err:
                    logger.warning(f"避撞模組失敗: {_ca_err}", exc_info=True)

            drones_data = []
            total_wps = 0
            total_dist = 0.0

            # ── 優先使用 assembled_paths（含 Dubins 曲線）──
            if assembled_paths:
                for uav_id, apath in assembled_paths.items():
                    # 按 SegmentLabel 分組：起飛/進入/作業歸 operation_paths，轉移/降落歸 transfer_paths
                    takeoff_pts = []
                    entry_pts = []
                    op_pts = []
                    xfer_pts = []
                    landing_pts = []
                    current_seg = []
                    current_label = None

                    for wp in apath.waypoints:
                        pt = (wp.lat, wp.lon, getattr(wp, 'alt', altitude))
                        if wp.segment_type != current_label:
                            if current_seg and current_label is not None:
                                lname = current_label.name
                                if lname == 'TAKEOFF':
                                    takeoff_pts.append(current_seg)
                                elif lname == 'ENTRY':
                                    entry_pts.append(current_seg)
                                elif lname == 'OPERATION':
                                    op_pts.append(current_seg)
                                elif lname == 'TRANSFER':
                                    xfer_pts.append(current_seg)
                                elif lname == 'LANDING':
                                    landing_pts.append(current_seg)
                            current_seg = [pt]
                            current_label = wp.segment_type
                        else:
                            current_seg.append(pt)
                    # flush last segment
                    if current_seg and current_label is not None:
                        lname = current_label.name
                        if lname == 'TAKEOFF':
                            takeoff_pts.append(current_seg)
                        elif lname == 'ENTRY':
                            entry_pts.append(current_seg)
                        elif lname == 'OPERATION':
                            op_pts.append(current_seg)
                        elif lname == 'TRANSFER':
                            xfer_pts.append(current_seg)
                        elif lname == 'LANDING':
                            landing_pts.append(current_seg)

                    # 各段分別保留，讓 map_widget 以不同線型/顏色繪製
                    total_wps += len(apath.waypoints)
                    start_pos = (apath.waypoints[0].lat, apath.waypoints[0].lon) if apath.waypoints else (centroid_lat, centroid_lon)
                    drones_data.append({
                        'drone_id': uav_id,
                        'start_position': start_pos,
                        'takeoff_paths': takeoff_pts,
                        'entry_paths': entry_pts,
                        'operation_paths': op_pts,
                        'transfer_paths': xfer_pts,
                        'landing_paths': landing_pts,
                    })
            else:
                # ── 備用：從 MDTSPResult 提取 ──
                for area_id, mdtsp_result in results.items():
                    assigned_uavs = allocation.get(area_id, [])
                    if hasattr(mdtsp_result, 'uav_paths') and mdtsp_result.uav_paths:
                        for uav_id, path_seq in mdtsp_result.uav_paths.items():
                            op_paths = []
                            xfer_paths = []
                            for seg in path_seq:
                                if hasattr(seg, 'waypoints') and seg.waypoints:
                                    op_paths.append(seg.waypoints)
                                    total_wps += len(seg.waypoints)
                                elif isinstance(seg, (list, tuple)):
                                    if len(seg) >= 2:
                                        op_paths.append(list(seg))
                                        total_wps += len(seg)

                            start_pos = op_paths[0][0] if op_paths and op_paths[0] else (centroid_lat, centroid_lon)
                            drones_data.append({
                                'drone_id': uav_id,
                                'start_position': start_pos,
                                'operation_paths': op_paths,
                                'transfer_paths': xfer_paths,
                            })
                    elif hasattr(mdtsp_result, 'ordered_segments'):
                        for idx, uav_id in enumerate(assigned_uavs):
                            seg_per_uav = []
                            for seg in mdtsp_result.ordered_segments:
                                pts = []
                                if hasattr(seg, 'left_point') and hasattr(seg, 'right_point'):
                                    pts = [seg.left_point, seg.right_point]
                                elif isinstance(seg, (list, tuple)) and len(seg) >= 2:
                                    pts = list(seg)
                                if pts:
                                    seg_per_uav.append(pts)
                                    total_wps += len(pts)

                            if seg_per_uav:
                                start_pos = seg_per_uav[0][0]
                                drones_data.append({
                                    'drone_id': uav_id,
                                    'start_position': start_pos,
                                    'operation_paths': seg_per_uav,
                                    'transfer_paths': [],
                                })

            # 計算總距離
            def _haversine(a, b):
                dlat = (b[0] - a[0]) * 111320
                dlon = (b[1] - a[1]) * 111320 * math.cos(math.radians(a[0]))
                return math.hypot(dlat, dlon)

            for dd in drones_data:
                for path in dd.get('operation_paths', []):
                    for j in range(len(path) - 1):
                        total_dist += _haversine(path[j], path[j + 1])

            # 衝突標記
            conflict_markers = []
            if _collision_conflicts:
                for cf in _collision_conflicts:
                    conflict_markers.append({
                        'lat': cf.pos_a[0], 'lon': cf.pos_a[1],
                        'min_dist': cf.min_dist_m,
                        'duration': cf.duration_s,
                        'uav_a': cf.uav_a, 'uav_b': cf.uav_b,
                    })

            swarm_data = {
                'drones': drones_data,
                'areas': [{'area_id': 1, 'polygon': list(self.corners)}],
                'stats': {
                    'total_drones': len(drones_data),
                    'total_waypoints': total_wps,
                    'total_distance': total_dist,
                    'estimated_time': total_makespan,
                    'conflicts': len(_collision_conflicts),
                    'avoidance_actions': len(_collision_actions),
                },
                'conflicts': conflict_markers,
            }
            self._dccpp_result = dccpp_result
            self.map_widget.display_swarm_raw(swarm_data)

            vtype_label = {"fixed_wing": "固定翼", "vtol": "VTOL (4+1)", "multirotor": "多旋翼"}.get(vehicle_type, "多旋翼")

            # ── 更新任務統計面板 ──
            try:
                _area_m2 = CoveragePlanner().calculate_coverage_area(list(self.corners))
            except Exception:
                _area_m2 = 0.0
            self.waypoint_label.setText(f"航點: {total_wps}")
            self.distance_label.setText(f"距離: {total_dist:.0f}m")
            self.mission_panel.update_mission_stats({
                'waypoint_count': total_wps,
                'total_distance': total_dist,
                'estimated_time': total_makespan,
                'area': _area_m2,
                'regions': len(drones_data),
            })

            self.statusBar().showMessage(
                f"DCCPP 最佳化完成 | {vtype_label} × {len(drones_data)} 台 | "
                f"共 {total_wps} 航點 | 總距離 {total_dist:.0f}m | "
                f"Makespan {total_makespan:.1f}s ({total_makespan / 60:.1f}min)",
                10000
            )
            logger.info(
                f"DCCPP 最佳化完成: {len(drones_data)} 台 {vtype_label} | "
                f"makespan={total_makespan:.1f}s"
            )

            # 儲存 DCCPP metrics 供比較
            self._dccpp_metrics = {
                'algorithm': 'DCCPP (最佳化)',
                'num_drones': len(drones_data),
                'vehicle_type': vtype_label,
                'total_waypoints': total_wps,
                'total_distance': total_dist,
                'estimated_time': total_makespan,
                'makespan': total_makespan,
            }

        except Exception as e:
            logger.error(f"DCCPP 最佳化失敗: {e}", exc_info=True)
            from PyQt6.QtWidgets import QMessageBox
            QMessageBox.warning(self, "DCCPP 失敗", f"DCCPP 最佳化路徑規劃失敗：\n{e}")

    def _on_dem_loaded(self, dem_path: str):
        """使用者選取 DEM → 載入並在 3D 地圖上疊加 heatmap"""
        try:
            from sensors.dem_terrain import DEMTerrainManager
            dem = DEMTerrainManager()
            if not dem.load_dem(dem_path):
                QMessageBox.warning(self, 'DEM', f'載入失敗：\n{dem_path}')
                return
            self._dem_manager = dem
            # 嘗試在 3D 地圖顯示
            cesium = None
            if hasattr(self, 'map_widget'):
                cesium = getattr(self.map_widget, 'map_3d', None) or \
                         (self.map_widget if hasattr(self.map_widget, 'show_dem_overlay') else None)
            if cesium is None or not hasattr(cesium, 'show_dem_overlay'):
                QMessageBox.information(
                    self, 'DEM',
                    f'✅ DEM 已載入\n'
                    f'緯度範圍: {dem._lat_min:.4f} ~ {dem._lat_max:.4f}\n'
                    f'經度範圍: {dem._lon_min:.4f} ~ {dem._lon_max:.4f}\n'
                    f'解析度: {dem._cols}x{dem._rows}\n\n'
                    f'（3D 地圖不可用，無法視覺化）'
                )
                return
            ok = cesium.show_dem_overlay(dem, alpha=0.6)
            # 同步建立 3D 立體地形網格（真實起伏）
            try:
                if hasattr(cesium, 'show_dem_terrain_3d'):
                    cesium.show_dem_terrain_3d(dem, max_size=80)
            except Exception as _e3:
                logger.warning(f'[DEM3D] 3D 地形網格生成失敗: {_e3}')
            # 自動切換到 3D 頁
            try:
                if hasattr(self.map_widget, '_switch_mode'):
                    self.map_widget._switch_mode(1)  # _MODE_3D
            except Exception:
                pass
            if ok:
                QMessageBox.information(
                    self, 'DEM',
                    f'✅ DEM 已疊加到 3D 地圖\n'
                    f'緯度: {dem._lat_min:.4f} ~ {dem._lat_max:.4f}\n'
                    f'經度: {dem._lon_min:.4f} ~ {dem._lon_max:.4f}\n'
                    f'解析度: {dem._cols}x{dem._rows}'
                )
            else:
                QMessageBox.warning(self, 'DEM', '3D 疊加失敗，請查看日誌（可能缺 Pillow）')
        except Exception as e:
            logger.error(f'[DEM] 載入/顯示失敗: {e}', exc_info=True)
            QMessageBox.critical(self, 'DEM', f'錯誤: {e}')

    def _on_dccpp_export_requested(self):
        """處理 DCCPP 匯出按鈕請求

        若 DCCPP 載具類型為 VTOL，自動轉派到 VTOL 匯出流程。
        """
        from PyQt6.QtWidgets import QMessageBox
        if getattr(self, '_dccpp_result', None) is None:
            QMessageBox.warning(
                self, "無 DCCPP 結果",
                "請先執行 DCCPP 最佳化規劃，再匯出任務。"
            )
            return

        # 偵測 DCCPP 載具類型：若為 VTOL 則轉派
        vehicle_type = self.flight_params.get('vehicle_type', '')
        if vehicle_type == 'vtol':
            transition_alt = 40.0
            try:
                transition_alt = self.parameter_panel.vtol_transition_alt_spin.value()
            except Exception:
                pass
            self._on_vtol_export_requested({'transition_alt': transition_alt})
            return

        try:
            self._export_dccpp_waypoints()
        except Exception as e:
            logger.error(f"匯出 DCCPP 任務失敗: {e}", exc_info=True)
            QMessageBox.warning(self, "匯出失敗", f"DCCPP 任務匯出失敗：\n{e}")

    def _on_vtol_export_requested(self, params: dict):
        """處理 VTOL 匯出按鈕請求"""
        from PyQt6.QtWidgets import QMessageBox, QFileDialog
        if getattr(self, '_dccpp_result', None) is None:
            QMessageBox.warning(
                self, "無 DCCPP 結果",
                "請先執行 DCCPP 最佳化規劃，再匯出 VTOL 任務。"
            )
            return

        assembled_paths = self._dccpp_result.get('assembled_paths', {})
        if not assembled_paths:
            QMessageBox.warning(self, "無可匯出路徑", "DCCPP 結果中沒有組裝路徑資料。")
            return

        export_dir = QFileDialog.getExistingDirectory(
            self, "選擇 VTOL 任務匯出目錄", ""
        )
        if not export_dir:
            return

        try:
            from mission.vtol_mission_exporter import VTOLMissionExporter

            altitude = self.flight_params.get('altitude', 100.0)
            speed = self.flight_params.get('speed', 15.0)
            turn_radius = self.flight_params.get('turn_radius', 50.0)
            transition_alt = params.get('transition_alt', 40.0)

            exporter = VTOLMissionExporter(
                transition_alt_m=transition_alt,
                cruise_speed_ms=speed,
                turn_radius_m=turn_radius,
            )

            # 匯出航點檔案
            exported_files = exporter.export_all(
                assembled_paths=assembled_paths,
                export_dir=export_dir,
                altitude=altitude,
                speed=speed,
            )

            # 在 Cesium 3D 圖台繪製 VTOL 軌跡視覺化
            try:
                cesium = self._get_cesium_widget()
                if cesium:
                    vtol_data = exporter.to_cesium_data(assembled_paths, altitude)
                    cesium.draw_vtol_swarm_paths(vtol_data)
            except Exception as ve:
                logger.warning(f'[VTOL] 3D 視覺化失敗: {ve}')

            QMessageBox.information(
                self, "VTOL 匯出成功",
                f"VTOL (4+1 混飛模式) 任務匯出完成！\n"
                f"目錄：{export_dir}\n"
                f"轉換高度：{transition_alt}m\n\n"
                + "\n".join(exported_files)
            )
            self.statusBar().showMessage(
                f"已匯出 {len(exported_files)} 個 VTOL 任務檔案至 {export_dir}", 5000
            )
            logger.info(f"VTOL 匯出完成: {len(exported_files)} 個檔案")

        except Exception as e:
            logger.error(f"匯出 VTOL 任務失敗: {e}", exc_info=True)
            QMessageBox.warning(self, "匯出失敗", f"VTOL 任務匯出失敗：\n{e}")

    def _export_dccpp_waypoints(self):
        """
        匯出 DCCPP 最佳化結果為每架 UAV 獨立的 .waypoints 檔案
        格式：QGC WPL 110（Mission Planner 相容）

        航點指令對應（符合論文三類路徑 + 起飛降落）：
          DO_SET_HOME (179)     - Home 佔位
          DO_CHANGE_SPEED (178) - 空速設定
          NAV_TAKEOFF (22)      - 起飛到指定高度 (param1=爬升角, param4=yaw)
          NAV_WAYPOINT (16)     - 爬升/進入/作業/轉移/進場航點
                                  param2=turn_radius（讓飛控提前轉彎）
          NAV_LAND (21)         - 降落觸地 (param4=著陸方向)
        """
        import os, math
        from PyQt6.QtWidgets import QFileDialog, QMessageBox

        export_dir = QFileDialog.getExistingDirectory(
            self, "選擇 DCCPP 任務匯出目錄", ""
        )
        if not export_dir:
            return

        assembled_paths = self._dccpp_result.get('assembled_paths', {})
        if not assembled_paths:
            QMessageBox.warning(self, "無可匯出路徑", "DCCPP 結果中沒有組裝路徑資料。")
            return

        altitude = self.flight_params.get('altitude', 100.0)
        speed = self.flight_params.get('speed', 15.0)
        turn_radius = self.flight_params.get('turn_radius', 50.0)
        exported_files = []

        for uav_id, apath in assembled_paths.items():
            if not apath.waypoints:
                continue

            waypoint_lines = ['QGC WPL 110']
            seq = 0

            # ── seq 0: DO_SET_HOME ──
            home_wp = apath.waypoints[0]
            waypoint_lines.append(create_waypoint_line(
                seq=seq, command=179,
                lat=home_wp.lat, lon=home_wp.lon, alt=0.0,
                current=0, autocontinue=1
            ))
            seq += 1

            # ── seq 1: DO_CHANGE_SPEED (空速) ──
            waypoint_lines.append(create_waypoint_line(
                seq=seq, command=178,
                param1=1.0, param2=speed, param3=0.0,
                current=0, autocontinue=1
            ))
            seq += 1

            # ── 依 SegmentLabel 分派 MAVLink 指令 ──
            # 先找到最後一個 LANDING 的索引 → 只在那一點發 NAV_LAND
            last_landing_idx = -1
            for _i, _w in enumerate(apath.waypoints):
                if getattr(getattr(_w, 'segment_type', None), 'name', '') == 'LANDING':
                    last_landing_idx = _i

            takeoff_emitted = False
            for idx, wp in enumerate(apath.waypoints):
                wp_alt = wp.alt if wp.alt > 0 else altitude

                if getattr(wp.segment_type, 'name', '') == 'TAKEOFF':
                    if idx == 0:
                        # Home 地面點已在 DO_SET_HOME 處理，跳過
                        continue
                    if not takeoff_emitted and idx >= 1:
                        # 跑道末端 → NAV_TAKEOFF（param4=0 使用當前航向）
                        waypoint_lines.append(create_waypoint_line(
                            seq=seq, command=22,  # NAV_TAKEOFF
                            param1=10.0,  # 最小爬升俯仰角
                            param4=0.0,
                            lat=wp.lat, lon=wp.lon, alt=wp_alt,
                            current=0, autocontinue=1
                        ))
                        seq += 1
                        takeoff_emitted = True
                    else:
                        # 爬升過渡點
                        waypoint_lines.append(create_waypoint_line(
                            seq=seq, command=16,
                            lat=wp.lat, lon=wp.lon, alt=wp_alt,
                            param2=turn_radius,
                            current=0, autocontinue=1
                        ))
                        seq += 1

                elif getattr(wp.segment_type, 'name', '') == 'LANDING':
                    if idx == last_landing_idx:
                        waypoint_lines.append(create_waypoint_line(
                            seq=seq, command=21,  # NAV_LAND
                            param4=getattr(wp, 'heading_compass_deg', getattr(wp, 'heading_deg', 0.0)),
                            lat=wp.lat, lon=wp.lon, alt=0.0,
                            current=0, autocontinue=1
                        ))
                        seq += 1
                    else:
                        waypoint_lines.append(create_waypoint_line(
                            seq=seq, command=16,
                            lat=wp.lat, lon=wp.lon, alt=wp_alt,
                            param2=turn_radius,
                            current=0, autocontinue=1
                        ))
                        seq += 1

                else:
                    # ENTRY / OPERATION / TRANSFER → NAV_WAYPOINT
                    # acceptance_radius = turn_radius（讓固定翼提前轉彎）
                    accept_r = turn_radius
                    if getattr(wp.segment_type, 'name', '') == 'OPERATION':
                        # 作業段密集航點：減小 acceptance_radius 避免跳點
                        if idx < len(apath.waypoints) - 1:
                            nxt = apath.waypoints[idx + 1]
                            dist_next = math.hypot(
                                (nxt.lat - wp.lat) * 111320,
                                (nxt.lon - wp.lon) * 111320 * math.cos(math.radians(wp.lat))
                            )
                            accept_r = max(turn_radius / 10.0,
                                           min(dist_next / 3.0, turn_radius))

                    waypoint_lines.append(create_waypoint_line(
                        seq=seq, command=16,
                        lat=wp.lat, lon=wp.lon, alt=wp_alt,
                        param2=accept_r,
                        current=0, autocontinue=1
                    ))
                    seq += 1

            filename = f"DCCPP_UAV{uav_id}_高度{altitude:.0f}m.waypoints"
            filepath = os.path.join(export_dir, filename)
            write_waypoints(filepath, waypoint_lines)
            exported_files.append(filename)
            logger.info(f"匯出 DCCPP UAV-{uav_id}: {filepath} ({seq} 航點)")

        # 匯出簡報
        briefing_path = os.path.join(export_dir, "DCCPP_briefing.txt")
        with open(briefing_path, 'w', encoding='utf-8') as f:
            f.write("DCCPP 最佳化覆蓋任務簡報\n")
            f.write("=" * 40 + "\n\n")
            total_makespan = self._dccpp_result.get('total_makespan', 0.0)
            f.write(f"無人機數量：{len(assembled_paths)}\n")
            f.write(f"預估 Makespan：{total_makespan:.1f}s ({total_makespan/60:.1f}min)\n")
            f.write(f"飛行高度：{altitude:.0f}m\n")
            f.write(f"飛行速度：{speed:.1f}m/s\n")
            f.write(f"轉彎半徑：{turn_radius:.0f}m\n\n")
            for uav_id, apath in assembled_paths.items():
                total_len = apath.total_length_m
                f.write(
                    f"UAV-{uav_id}：{len(apath.waypoints)} 航點，"
                    f"總長 {total_len:.0f}m\n"
                    f"  起飛 {getattr(apath, 'takeoff_length_m', 0.0):.0f}m + "
                    f"進場 {getattr(apath, 'entry_length_m', 0.0):.0f}m + "
                    f"作業 {getattr(apath, 'operation_length_m', 0.0):.0f}m + "
                    f"轉移 {getattr(apath, 'transfer_length_m', 0.0):.0f}m + "
                    f"降落 {getattr(apath, 'landing_length_m', 0.0):.0f}m\n"
                )
        exported_files.append("DCCPP_briefing.txt")

        QMessageBox.information(
            self, "匯出成功",
            f"DCCPP 任務匯出完成！\n目錄：{export_dir}\n\n"
            + "\n".join(exported_files)
        )
        self.statusBar().showMessage(
            f"已匯出 {len(exported_files)} 個 DCCPP 任務檔案至 {export_dir}", 5000
        )
        logger.info(f"DCCPP 匯出完成: {len(exported_files)} 個檔案")

    # ─────────────────────────────────────────────────────────────────
    # SITL / MAVLink 即時遙測
    # ─────────────────────────────────────────────────────────────────
    def on_sitl_connect(self, conn_str: str, sysid_label: int = 1,
                        vehicle_hint: str = ''):
        """啟動 SITL 背景接收執行緒（單一連線）

        vehicle_hint: 由 UI 傳入的載具類型（'VTOL'/'PLANE'/'COPTER'）
                      用於區分 QuadPlane VTOL 與純固定翼（兩者 MAV_TYPE 相同）
        """
        from mission.sitl_link import SITLLink
        link = SITLLink(conn_str, sysid_label=sysid_label,
                        vehicle_hint=vehicle_hint, parent=self)
        link.connected.connect(self.sitl_hud.on_connected)
        link.disconnected.connect(self._on_sitl_disconnected)
        link.telemetry.connect(self._on_sitl_telemetry)
        link.status_text.connect(self.sitl_hud.on_status_text)
        link.error.connect(self.sitl_hud.on_error)
        link.start()
        self._sitl_links.append(link)
        if self._sitl_link is None:
            self._sitl_link = link
        logger.info(f'[SITL] 啟動連線執行緒: {conn_str} (sysid={sysid_label})')

    def on_sitl_disconnect(self):
        """斷開所有 SITL 連線"""
        for link in self._sitl_links:
            try:
                link.stop()
                link.wait(2000)
            except Exception:
                pass
        self._sitl_links.clear()
        self._sitl_link = None
        self.sitl_hud.on_disconnected('使用者中斷')
        if hasattr(self.map_widget, 'clear_uav'):
            self.map_widget.clear_uav()

    def _on_sitl_disconnected(self, reason: str):
        self.sitl_hud.on_disconnected(reason)

    def _on_sitl_telemetry(self, frame):
        """收到一幀遙測 → 更新 HUD + 推到 3D 地圖（多機支援）"""
        # 主機 (sysid_label==1) 才更新 HUD，避免多機資訊互相覆蓋
        # 所有 sysid 都送進 HUD（多機卡片）
        self.sitl_hud.on_telemetry(frame)
        if frame.is_valid_gps() and hasattr(self.map_widget, 'update_uav_position'):
            _pitch = getattr(frame, 'pitch', 0.0)
            _roll = getattr(frame, 'roll', 0.0)
            self.map_widget.update_uav_position(
                frame.lat, frame.lon, frame.alt_rel,
                frame.heading, frame.ground_speed,
                sysid=frame.sysid, mode=frame.mode, armed=frame.armed,
                vehicle_type=getattr(frame, 'vehicle_type', ''),
                pitch_deg=_pitch,
                roll_deg=_roll,
            )
            # 同步更新戰術模組（FOV 光錐、熱力圖、RCS）
            self._update_tactical_on_uav_move(
                frame.lat, frame.lon, frame.alt_rel,
                frame.heading, _pitch, _roll,
                sysid=frame.sysid,
            )

    def _sitl_broadcast(self, cmd: str, arg=None):
        """廣播指令到所有 SITL 連線"""
        if not self._sitl_links:
            QMessageBox.information(self, 'SITL', '尚未連線任何 SITL')
            return
        for link in self._sitl_links:
            try:
                if cmd == 'arm':       link.arm()
                elif cmd == 'disarm':  link.disarm()
                elif cmd == 'set_mode':link.set_mode(arg)
                elif cmd == 'takeoff': link.takeoff(arg)
                elif cmd == 'auto_start': link.auto_start()
                elif cmd == 'mission_start': link.mission_start()
                elif cmd == 'set_param':
                    name, value, ptype = arg
                    link.set_param(name, value, ptype)
                elif cmd == 'set_params':
                    link.set_params(arg)
                elif cmd == 'get_param':
                    link.get_param(arg)
            except Exception as e:
                logger.error(f'[SITL] {cmd} 失敗: {e}')

    def on_sitl_upload_mission(self):
        """把當前路徑上傳到所有連線的 SITL"""
        if not self._sitl_links:
            QMessageBox.information(self, 'SITL', '尚未連線任何 SITL')
            return
        # 從 map_widget.paths 收集所有航點；若無則改抓 DCCPP / swarm 結果
        wps = []
        try:
            default_alt = float(self.parameter_panel.get_parameters().get('altitude', 50))
            paths = getattr(self.map_widget, 'paths', [])
            for path in paths:
                for p in path:
                    if len(p) >= 3:
                        wps.append((float(p[0]), float(p[1]), float(p[2])))
                    elif len(p) == 2:
                        wps.append((float(p[0]), float(p[1]), default_alt))

            # DCCPP / 群飛結果（display_swarm_raw 不會塞進 self.paths）
            if not wps:
                swarm = getattr(self.map_widget, '_swarm_data', None) \
                        or getattr(getattr(self.map_widget, 'map_2d', None), '_swarm_data', None)
                if swarm:
                    for d in swarm.get('drones', []):
                        for p in d.get('path', []):
                            if len(p) >= 3:
                                wps.append((float(p[0]), float(p[1]), float(p[2])))
                            elif len(p) == 2:
                                wps.append((float(p[0]), float(p[1]), default_alt))

            # DCCPP 多機：每架 SITL 綁到各自的 UAV 路徑（dict: link_index → wps list）
            per_link_wps: dict = {}

            if not wps and getattr(self, '_dccpp_result', None):
                # DCCPP：輸出完整任務指令序列，讓 SITL AUTO 模式完整起飛→任務→降落。
                # 每個 tuple: (lat, lon, alt, cmd, p1, p2, p3, p4)
                #
                # ── MAVLink 任務指令對照 ──
                # CMD_WP   = 16  (MAV_CMD_NAV_WAYPOINT)       → 普通航點
                # CMD_TO   = 22  (MAV_CMD_NAV_TAKEOFF)        → 固定翼跑道起飛
                # CMD_LAND = 21  (MAV_CMD_NAV_LAND / RTL)     → 固定翼降落
                # CMD_SPD  = 178 (MAV_CMD_DO_CHANGE_SPEED)    → 設定巡航速度
                # ── VTOL QuadPlane 專用 ──
                # CMD_VTOL_TO   = 84   (MAV_CMD_NAV_VTOL_TAKEOFF)    → 垂直起飛
                # CMD_VTOL_LAND = 85   (MAV_CMD_NAV_VTOL_LAND)       → 垂直降落
                # CMD_VTOL_TRANS= 3000 (MAV_CMD_DO_VTOL_TRANSITION)  → 飛行模式轉換
                #   param1=3 → MC（多旋翼）  param1=4 → FW（固定翼）
                CMD_WP, CMD_TO, CMD_LAND, CMD_SPD = 16, 22, 21, 178
                CMD_VTOL_TO, CMD_VTOL_LAND, CMD_VTOL_TRANS = 84, 85, 3000
                params = self.parameter_panel.get_parameters()
                turn_radius = float(params.get('turn_radius', 50.0))
                cruise_speed = float(params.get('speed', 15.0))

                # 判斷是否為 VTOL 載具 → 使用垂直起降指令
                is_vtol = self.flight_params.get('vehicle_type', '') == 'vtol'

                # VTOL 轉換高度：多旋翼垂直爬升到此高度後，再轉換為固定翼巡航
                # 預設 100m；優先讀取 UI 參數面板的設定值
                vtol_transition_alt = 100.0
                if is_vtol:
                    try:
                        vtol_transition_alt = float(
                            self.parameter_panel.vtol_transition_alt_spin.value()
                        )
                    except Exception:
                        pass
                    logger.info(
                        f'[SITL] VTOL 轉換高度: {vtol_transition_alt}m '
                        f'(多旋翼垂直爬升 → 轉換 FW → 巡航)'
                    )

                ap = self._dccpp_result.get('assembled_paths') or {}
                # 依 uav_id 排序 → 對應 SITL instance 0/1/2...
                sorted_paths = sorted(ap.items(), key=lambda kv: kv[0])

                def _build_wps_for_path(first_path):
                    """把一個 BuiltPath 轉成 (lat,lon,alt,cmd,p1,p2,p3,p4) 列表。

                    VTOL 模式任務序列（ArduPilot QuadPlane）：
                      DO_CHANGE_SPEED → NAV_VTOL_TAKEOFF → DO_VTOL_TRANSITION(FW)
                      → NAV_WAYPOINT × N → DO_VTOL_TRANSITION(MC) → NAV_VTOL_LAND

                    固定翼模式任務序列：
                      DO_CHANGE_SPEED → NAV_TAKEOFF → NAV_WAYPOINT × N → NAV_LAND
                    """
                    wps_local = []
                    wp_list = getattr(first_path, 'waypoints', None) or []
                    if not wp_list:
                        return wps_local
                    last_landing_idx = -1
                    has_takeoff_seg = False
                    for _i, _w in enumerate(wp_list):
                        _nm = getattr(getattr(_w, 'segment_type', None), 'name', '')
                        if _nm == 'LANDING':
                            last_landing_idx = _i
                        if _nm == 'TAKEOFF':
                            has_takeoff_seg = True

                    # 1) 設定巡航速度
                    wps_local.append((0.0, 0.0, 0.0, CMD_SPD,
                                      0.0, cruise_speed, -1.0, 0.0))

                    # 2) 起飛指令
                    takeoff_emitted = False
                    if not has_takeoff_seg and wp_list:
                        _w0 = wp_list[0]
                        _alt0 = float(getattr(_w0, 'alt', default_alt) or default_alt)
                        if _alt0 < 20.0:
                            _alt0 = max(default_alt, 50.0)
                        if is_vtol:
                            # ── VTOL QuadPlane 起飛序列 ──────────────────
                            # 階段 1: NAV_VTOL_TAKEOFF (cmd=84)
                            #   多旋翼模式垂直爬升到「轉換高度」
                            #   alt = vtol_transition_alt（例如 100m）
                            #   到達此高度後飛控會自動懸停等待下一指令
                            wps_local.append((float(_w0.lat), float(_w0.lon),
                                              vtol_transition_alt,
                                              CMD_VTOL_TO, 0.0, 0.0, 0.0, 0.0))
                            # 階段 2: DO_VTOL_TRANSITION → 固定翼 (FW)
                            #   param1=4 → MAV_VTOL_STATE_FW
                            #   在轉換高度懸停狀態下啟動後推馬達，
                            #   前進加速到失速速度以上後，四軸馬達關閉
                            wps_local.append((0.0, 0.0, 0.0, CMD_VTOL_TRANS,
                                              4.0, 0.0, 0.0, 0.0))
                            # 階段 3: 第一個 NAV_WAYPOINT 會帶飛機爬升到任務高度
                            #   例如任務高度 200m → 固定翼以爬升姿態飛向第一航點
                            #   （這一步由後面的 for 迴圈自動產生）
                        else:
                            # 固定翼: NAV_TAKEOFF (cmd=22) → 跑道/手拋起飛
                            # param1 = pitch_angle (最低仰角 10°)
                            wps_local.append((float(_w0.lat), float(_w0.lon), _alt0,
                                              CMD_TO, 10.0, 0.0, 0.0, 0.0))
                        takeoff_emitted = True

                    for idx, wp in enumerate(wp_list):
                        seg_name = getattr(
                            getattr(wp, 'segment_type', None), 'name', ''
                        ).upper()
                        lat = getattr(wp, 'lat', None)
                        lon = getattr(wp, 'lon', None)
                        alt = float(getattr(wp, 'alt', default_alt) or default_alt)
                        hdg = float(getattr(wp, 'heading_compass_deg', 0.0) or 0.0)
                        if lat is None or lon is None:
                            continue
                        lat = float(lat); lon = float(lon)

                        if seg_name == 'TAKEOFF':
                            if idx == 0:
                                continue
                            if not takeoff_emitted:
                                if is_vtol:
                                    # VTOL: 垂直起飛到轉換高度 → 轉 FW
                                    wps_local.append((lat, lon, vtol_transition_alt,
                                                      CMD_VTOL_TO, 0.0, 0.0, 0.0, 0.0))
                                    wps_local.append((0.0, 0.0, 0.0, CMD_VTOL_TRANS,
                                                      4.0, 0.0, 0.0, 0.0))
                                else:
                                    wps_local.append((lat, lon, alt, CMD_TO,
                                                      10.0, 0.0, 0.0, 0.0))
                                takeoff_emitted = True
                            else:
                                wps_local.append((lat, lon, alt, CMD_WP,
                                                  0.0, turn_radius, 0.0, 0.0))
                        elif seg_name == 'LANDING':
                            if idx == last_landing_idx:
                                if is_vtol:
                                    # ── VTOL QuadPlane 降落序列 ──────────────
                                    # 階段 1: 先飛到降落點上空，高度=轉換高度
                                    #   確保在安全高度進行 FW→MC 轉換
                                    wps_local.append((lat, lon, vtol_transition_alt,
                                                      CMD_WP, 0.0, 0.0, 0.0, 0.0))
                                    # 階段 2: DO_VTOL_TRANSITION → 多旋翼 (MC)
                                    #   param1=3 → MAV_VTOL_STATE_MC
                                    #   後推馬達關閉，四軸馬達啟動接管
                                    wps_local.append((0.0, 0.0, 0.0, CMD_VTOL_TRANS,
                                                      3.0, 0.0, 0.0, 0.0))
                                    # 階段 3: NAV_VTOL_LAND (cmd=85)
                                    #   四軸馬達垂直降落到地面
                                    wps_local.append((lat, lon, 0.0, CMD_VTOL_LAND,
                                                      0.0, 0.0, 0.0, hdg))
                                else:
                                    wps_local.append((lat, lon, 0.0, CMD_LAND,
                                                      0.0, 0.0, 0.0, hdg))
                            else:
                                wps_local.append((lat, lon, alt, CMD_WP,
                                                  0.0, turn_radius, 0.0, 0.0))
                        else:
                            wps_local.append((lat, lon, alt, CMD_WP,
                                              0.0, turn_radius, 0.0, 0.0))
                    return wps_local

                # 每個 UAV 建立獨立 wps，對應到相同 index 的 SITL link
                for link_idx, (uav_id, bpath) in enumerate(sorted_paths):
                    path_wps = _build_wps_for_path(bpath)
                    if path_wps:
                        per_link_wps[link_idx] = (uav_id, path_wps)
                        logger.info(
                            f'[SITL] UAV-{uav_id} → link#{link_idx}: {len(path_wps)} 點'
                        )
                # 若只有 1 筆路徑，維持舊行為（讓 wps 非空觸發上傳判斷）
                if per_link_wps and 0 in per_link_wps:
                    wps = per_link_wps[0][1]
        except Exception as e:
            logger.error(f'[SITL] 收集航點失敗: {e}', exc_info=True)
        if not wps and not per_link_wps:
            QMessageBox.warning(self, 'SITL', '目前沒有可上傳的路徑，請先預覽生成路徑')
            return

        # 分派：每架 link 使用對應 uav 的路徑；若 link 數 > 路徑數則剩餘 link 共用最後一份
        if per_link_wps:
            for link_idx, link in enumerate(self._sitl_links):
                key = link_idx if link_idx in per_link_wps else max(per_link_wps.keys())
                uav_id, path_wps = per_link_wps[key]
                link.upload_mission(path_wps)
                logger.info(
                    f'[SITL] link#{link_idx} ← UAV-{uav_id} ({len(path_wps)} 點)'
                )
            self.statusBar().showMessage(
                f'已為 {len(self._sitl_links)} 台 SITL 各自上傳獨立 DCCPP 路徑', 5000
            )
        else:
            for link in self._sitl_links:
                link.upload_mission(wps)
            self.statusBar().showMessage(
                f'已對 {len(self._sitl_links)} 個 SITL 上傳 {len(wps)} 個航點', 5000
            )

    def _on_main_tab_changed(self, index: int):
        """切換右側分頁時，調整任務面板顯示內容
        index 0 = 基本演算法 → 顯示完整預覽 / 匯出
        其他   → 隱藏預覽 / 匯出（避免重複），保留清除類按鈕
        """
        try:
            mp = self.mission_panel
            is_basic = (index == 0)
            mp.preview_btn.setVisible(is_basic)
            mp.export_btn.setVisible(is_basic)
            if hasattr(mp, '_stats_card'):
                mp._stats_card.setVisible(is_basic)
        except Exception as e:
            logger.warning(f'切換分頁時調整 mission_panel 失敗: {e}')

    def on_sitl_launch(self, vehicle: str, count: int = 1):
        """啟動 N 台專案內 SITL binary

        若已執行過 DCCPP，則自動依任務的載具類型覆寫 vehicle，
        避免使用者選錯 binary 導致 ArduPlane 跑 Copter 任務或反之。
        """
        from mission.sitl_launcher import SITLLauncher
        if self._sitl_launcher is None:
            self._sitl_launcher = SITLLauncher()

        try:
            dccpp = getattr(self, '_dccpp_result', None)
            if dccpp:
                ap = dccpp.get('assembled_paths') or {}
                first_path = next(iter(ap.values()), None)
                if first_path is not None:
                    wp_list = getattr(first_path, 'waypoints', None) or []
                    has_takeoff = any(
                        getattr(getattr(w, 'segment_type', None), 'name', '') == 'TAKEOFF'
                        for w in wp_list
                    )
                    inferred = 'PLANE' if has_takeoff else 'COPTER'
                    # VTOL 由使用者明確選擇，不被 DCCPP 推斷覆寫
                    if vehicle.upper() != 'VTOL' and inferred != vehicle.upper():
                        logger.warning(
                            f'[SITL] 依 DCCPP 結果自動切換載具：{vehicle} → {inferred}'
                        )
                        vehicle = inferred
        except Exception as _e:
            logger.warning(f'[SITL] 推斷載具類型失敗: {_e}')

        if not SITLLauncher.is_available():
            QMessageBox.warning(
                self, 'SITL 不可用',
                '找不到 sitl/ArduPlane.exe 或 ArduCopter.exe\n'
                '請從 Mission Planner sitl 資料夾複製到專案 sitl/ 目錄'
            )
            return

        try:
            from config.settings import get_settings
            _s = get_settings()
            lat = _s.map.default_lat
            lon = _s.map.default_lon

            # ── 優先使用當前任務的原點，讓 SITL 飛機生在任務附近 ──
            # 否則 AUTO 後會試圖飛幾十公里到任務點 → 極端 bank/pitch 墜機
            origin_src = None
            spawn_heading = 0.0
            per_uav_homes: list = []  # [(lat, lon, heading), ...] 依 uav_id 排序
            try:
                # 1. DCCPP 結果中每架 UAV 的起飛點
                dccpp = getattr(self, '_dccpp_result', None)
                if dccpp:
                    ap = dccpp.get('assembled_paths') or {}
                    import math as _m
                    for uav_id, bpath in sorted(ap.items(), key=lambda kv: kv[0]):
                        wps_u = getattr(bpath, 'waypoints', None) or []
                        if not wps_u:
                            continue
                        u_wp0 = wps_u[0]
                        u_lat = float(u_wp0.lat)
                        u_lon = float(u_wp0.lon)
                        u_hdg = 0.0
                        if len(wps_u) >= 2:
                            u_wp1 = wps_u[1]
                            dN = (float(u_wp1.lat) - u_lat) * 111320.0
                            dE = (float(u_wp1.lon) - u_lon) * 111320.0 * _m.cos(_m.radians(u_lat))
                            if abs(dN) + abs(dE) > 1e-6:
                                u_hdg = (_m.degrees(_m.atan2(dE, dN))) % 360.0
                        per_uav_homes.append((u_lat, u_lon, u_hdg))
                        logger.info(
                            f'[SITL] UAV-{uav_id} 起飛點: '
                            f'({u_lat:.6f}, {u_lon:.6f}) hdg={u_hdg:.1f}°'
                        )
                    if per_uav_homes:
                        lat, lon = per_uav_homes[0][0], per_uav_homes[0][1]
                        spawn_heading = per_uav_homes[0][2]
                        origin_src = f'DCCPP 每架獨立起飛點 ({len(per_uav_homes)} 個)'
                # 2. 當前多邊形的第一個角點
                if origin_src is None and getattr(self, 'corners', None):
                    c0 = self.corners[0]
                    lat, lon = float(c0[0]), float(c0[1])
                    origin_src = '多邊形第一點'
                # 3. Home point
                if origin_src is None and getattr(self, 'home_point', None):
                    lat, lon = float(self.home_point[0]), float(self.home_point[1])
                    origin_src = 'Home 點'
            except Exception as _e:
                logger.warning(f'[SITL] 讀取任務原點失敗，改用預設座標: {_e}')

            logger.info(f'[SITL] 生成座標: ({lat:.6f}, {lon:.6f}) heading={spawn_heading:.1f}°'
                        + (f' (來源: {origin_src})' if origin_src else ' (預設)'))

            # 若有 DCCPP 每架獨立起飛點，count 自動對齊路徑數量
            if per_uav_homes:
                if count != len(per_uav_homes):
                    logger.info(
                        f'[SITL] 依 DCCPP 結果將 SITL 數量 {count} → {len(per_uav_homes)}'
                    )
                count = len(per_uav_homes)

            results = self._sitl_launcher.start_multi(
                vehicle=vehicle, count=count,
                lat=lat, lon=lon, alt=0.0, heading=spawn_heading,
                spacing_deg=0.0008,
                homes=per_uav_homes if per_uav_homes else None,
            )
            first_conn = results[0][1]
            self.sitl_hud.on_sitl_launched(f'{vehicle} x{count}', first_conn)
            logger.info(f'[SITL] 內建 {vehicle} x{count} 已啟動')

            # 對每台啟動 MAVLink 連線（sysid_label = instance+1）
            QApplication.processEvents()
            import time as _t; _t.sleep(0.5)
            for instance, conn in results:
                self.on_sitl_connect(conn, sysid_label=instance + 1,
                                     vehicle_hint=vehicle)
        except Exception as e:
            logger.error(f'[SITL] 啟動失敗: {e}', exc_info=True)
            QMessageBox.critical(self, 'SITL 啟動失敗', str(e))
            self.sitl_hud.on_sitl_stopped()

    def on_sitl_stop_local(self):
        """停止本地 SITL（含所有實例與連線）"""
        if self._sitl_links:
            self.on_sitl_disconnect()
        if self._sitl_launcher is not None:
            self._sitl_launcher.stop()
        self.sitl_hud.on_sitl_stopped()
        logger.info('[SITL] 內建 SITL 已停止')

    # ══════════════════════════════════════════════════════════════════
    #  戰術模組事件處理
    # ══════════════════════════════════════════════════════════════════

    def _on_elev_slicer_changed(self, min_alt: float, max_alt: float):
        """高程切片參數變更 → 呼叫 3D 地圖更新"""
        self.map_widget.update_elevation_slicer(min_alt, max_alt)
        self.statusBar().showMessage(
            f'[FSDM] 高程切片啟用: {min_alt:.0f}m ~ {max_alt:.0f}m', 5000
        )

    def _on_elev_slicer_cleared(self):
        """清除高程切片"""
        self.map_widget.clear_elevation_slicer()
        self.statusBar().showMessage('[FSDM] 高程切片已清除', 3000)

    def _on_sar_heatmap_init(self, params: dict):
        """初始化搜救機率熱力圖（使用目前邊界區域）"""
        corners = self.map_widget.corners
        if len(corners) < 3:
            from PyQt6.QtWidgets import QMessageBox
            QMessageBox.warning(self, "無邊界區域",
                                "請先在地圖上定義至少 3 個邊界點作為搜索區域")
            return

        # 計算邊界矩形
        lats = [c[0] for c in corners]
        lons = [c[1] for c in corners]
        lat_min, lat_max = min(lats), max(lats)
        lon_min, lon_max = min(lons), max(lons)

        grid_size = params.get('grid_size', 20)
        sweep_w = params.get('sweep_width', 50.0)
        quality = params.get('quality', 0.8)

        self.map_widget.init_sar_heatmap(
            lat_min, lat_max, lon_min, lon_max,
            rows=grid_size, cols=grid_size,
            sweep_width=sweep_w, quality=quality,
        )
        self.statusBar().showMessage(
            f'[SAR] 熱力圖初始化: {grid_size}×{grid_size} 網格', 5000
        )

    def _on_sar_heatmap_reset(self):
        """重置搜救熱力圖"""
        self.map_widget.reset_sar_heatmap()
        self.statusBar().showMessage('[SAR] 熱力圖已重置', 3000)

    def _on_sar_heatmap_clear(self):
        """清除搜救熱力圖"""
        self.map_widget.clear_sar_heatmap()
        self.map_widget.clear_fov_cone()
        self.statusBar().showMessage('[SAR] 熱力圖已清除', 3000)

    def _on_fov_cone_toggle(self, enabled: bool):
        """FOV 光錐開關"""
        self._fov_cone_enabled = enabled
        if not enabled:
            self.map_widget.clear_fov_cone()
        self.statusBar().showMessage(
            f'[SAR] FOV 光錐 {"啟用" if enabled else "關閉"}', 3000
        )

    def _on_radar_sim(self, params: dict):
        """模擬雷達威脅掃描：建立穹頂 + 播放脈衝動畫"""
        lat = params.get('lat', 23.70)
        lon = params.get('lon', 120.42)
        radius = params.get('radius', 5000.0)
        name = params.get('name', 'Radar')

        self.map_widget.add_radar_dome(lat, lon, 0.0, radius, name)
        # 飛向雷達位置
        self.map_widget.fly_to_position(lat, lon, 0.0, radius * 2.5)
        # 播放掃描脈衝動畫（需要延遲以等待穹頂建立完成）
        from PyQt6.QtCore import QTimer
        QTimer.singleShot(500, lambda: self.map_widget.animate_radar_scan(0, 3000))

        self.statusBar().showMessage(
            f'[Radar] 威脅穹頂已建立: {name} R={radius/1000:.1f}km', 5000
        )

    def _on_radar_clear(self):
        """清除所有雷達穹頂"""
        self.map_widget.clear_radar_domes()
        # 同時關閉 RCS 渲染
        self._rcs_enabled = False
        self.map_widget.clear_rcs_sensitivity(1)
        self.statusBar().showMessage('[Radar] 所有雷達穹頂已清除', 3000)

    def _on_rcs_toggle(self, enabled: bool):
        """RCS 渲染開關"""
        self._rcs_enabled = enabled
        if not enabled:
            self.map_widget.clear_rcs_sensitivity(1)
        self.statusBar().showMessage(
            f'[RCS] 敏感度渲染 {"啟用" if enabled else "關閉"}', 3000
        )

    def _update_tactical_on_uav_move(self, lat: float, lon: float, alt: float,
                                      heading_deg: float, pitch_deg: float,
                                      roll_deg: float, sysid: int = 1):
        """SITL UAV 位置更新時，同步更新戰術模組視覺化

        此方法應在 UAV 位置更新後呼叫，以驅動：
        1. FOV 光錐位置同步
        2. SAR 熱力圖掃描更新
        3. RCS 敏感度動態計算
        """
        if self._fov_cone_enabled:
            fov_r = self.parameter_panel.get_fov_radius()
            self.map_widget.update_fov_cone(
                lat, lon, alt, fov_r,
                heading_deg, pitch_deg, roll_deg,
            )
            # 同時更新熱力圖（光錐掃過的區域）
            self.map_widget.update_heatmap(lat, lon, fov_r)

        if self._rcs_enabled:
            self.map_widget.update_rcs_sensitivity(
                lat, lon, alt, heading_deg, sysid,
            )

    # ═════════════════════════════════════════════════════════════════
    # Swarm Strike 處理 — 已抽出至 ui.controllers.StrikeControllerMixin
    # (MainWindow 透過多重繼承自動獲得所有 _on_strike_* 方法)
    # ═════════════════════════════════════════════════════════════════

    def _get_cesium_widget(self):
        """取得 CesiumMapWidget 實例（通過 DualMapWidget）"""
        if hasattr(self.map_widget, 'map_3d'):
            return self.map_widget.map_3d
        return None

    def on_reset_layout(self):
        """重置所有 Splitter 為預設比例 (Ctrl+Shift+R)

        涵蓋三層：
          1. 主水平 splitter (地圖 ⇄ 右側控制面板) → 60:40
          2. 右側垂直 splitter (參數面板 ⇄ 任務面板) → 78:22
          3. 各 tab 內的垂直 splitter (所有 GroupBox 平均分配)
        """
        try:
            # (1) 主 splitter
            if hasattr(self, '_main_splitter') and self._main_splitter:
                w = self.width() or 1280
                self._main_splitter.setSizes([int(w * 0.60), int(w * 0.40)])
            # (2) 右側面板 splitter
            if hasattr(self, '_right_panel_splitter') and self._right_panel_splitter:
                h = self.height() or 720
                self._right_panel_splitter.setSizes([int(h * 0.78), int(h * 0.22)])
            # (3) ParameterPanel 內的所有 tab splitter
            if hasattr(self, 'parameter_panel') and hasattr(
                self.parameter_panel, 'reset_resizable_layout'
            ):
                self.parameter_panel.reset_resizable_layout()
            self.statusBar().showMessage('已重置版面比例', 3000)
            logger.info('[Layout] 版面比例已重置')
        except Exception as e:
            logger.warning(f'[Layout] 重置失敗: {e}')

    def resizeEvent(self, event):
        """視窗大小變更時，保持地圖/控制面板 60:40 比例。

        這讓使用者在 1080p / 2K / 4K 之間拖拉視窗到不同螢幕時，
        splitter 會自動重新分配，不會跑版。
        """
        super().resizeEvent(event)
        splitter = getattr(self, '_main_splitter', None)
        if splitter is None:
            return
        try:
            w = event.size().width()
            if w < 200:
                return
            # 讀當前使用者是否手動改過比例；若變動 <5% 則視為自動，重新調整
            cur = splitter.sizes()
            if len(cur) == 2 and sum(cur) > 0:
                cur_ratio = cur[0] / sum(cur)
                if abs(cur_ratio - 0.60) > 0.05:
                    # 使用者自己拖過分隔線 → 尊重其設定，不覆寫
                    return
            splitter.setSizes([int(w * 0.60), int(w * 0.40)])
        except Exception:
            pass

    def closeEvent(self, event):
        """視窗關閉事件"""
        # 確保 SITL 執行緒安全停止
        for _link in self._sitl_links:
            try:
                _link.stop()
                _link.wait(2000)
            except Exception:
                pass
        self._sitl_links.clear()
        # 停止本地 SITL 子行程
        if self._sitl_launcher is not None:
            try:
                self._sitl_launcher.stop()
            except Exception:
                pass

        if self.current_mission and self.has_unsaved_changes():
            reply = QMessageBox.question(
                self, "未儲存的變更",
                "當前任務有未儲存的變更，確定要退出嗎？",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
            )
            
            if reply == QMessageBox.StandardButton.No:
                event.ignore()
                return
        
        logger.info("應用程式關閉")
        event.accept()