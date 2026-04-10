"""
雙模式地圖容器
將 2D (Folium/Leaflet) 與 3D (Cesium) 地圖整合在同一個 QStackedWidget 中，
提供無縫切換功能，對外介面與原 MapWidget 完全相容。
"""

from typing import List, Tuple, Optional

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QStackedWidget,
    QPushButton, QLabel, QFrame,
)
from PyQt6.QtCore import pyqtSignal, Qt

from ui.widgets.map_widget import MapWidget
from ui.widgets.cesium_map_widget import CesiumMapWidget
from utils.logger import get_logger

logger = get_logger()

_MODE_2D = 0
_MODE_3D = 1


class DualMapWidget(QWidget):
    """
    2D / 3D 雙模式地圖容器。

    ── 使用方式 ────────────────────────────────────────────────────────
    與原 MapWidget 完全相容：所有信號 / 方法呼叫不需修改。

    固定翼任務需額外呼叫：
        self.map_widget.set_fw_result(result)   # result = fw_mission_result
    以供 3D 地圖取得完整高度資訊。
    """

    # ── 轉發 MapWidget 的所有信號 ─────────────────────────────────────
    corner_added      = pyqtSignal(float, float)
    corner_moved      = pyqtSignal(int, float, float)
    circle_defined    = pyqtSignal(float, float, float)
    nfz_polygon_drawn = pyqtSignal(list)
    nfz_circle_drawn  = pyqtSignal(float, float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._mode = _MODE_2D
        self._init_ui()
        self._connect_signals()
        logger.info('DualMapWidget 初始化完成')

    # ─────────────────────────────────────────────────────────────────
    # UI 初始化
    # ─────────────────────────────────────────────────────────────────
    def _init_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ── 頂部工具列 ──────────────────────────────────────────────
        toolbar = QFrame()
        toolbar.setFixedHeight(34)
        toolbar.setStyleSheet(
            'QFrame{background:#1a1f2e;border-bottom:1px solid #2a3248;}'
        )
        tb_layout = QHBoxLayout(toolbar)
        tb_layout.setContentsMargins(8, 0, 8, 0)
        tb_layout.setSpacing(6)

        # 模式指示標籤
        self._mode_label = QLabel('🗺️ 2D 地圖')
        self._mode_label.setStyleSheet(
            'color:#90caf9;font-size:12px;font-weight:600;'
        )
        tb_layout.addWidget(self._mode_label)
        tb_layout.addStretch()

        # 2D 按鈕
        self._btn_2d = QPushButton('🗺️  2D 衛星')
        self._btn_2d.setCheckable(True)
        self._btn_2d.setChecked(True)
        self._btn_2d.setFixedHeight(24)
        self._btn_2d.setStyleSheet(self._btn_style(active=True))
        self._btn_2d.clicked.connect(lambda: self._switch_mode(_MODE_2D))
        tb_layout.addWidget(self._btn_2d)

        # 3D 按鈕
        self._btn_3d = QPushButton('🌐  3D Cesium')
        self._btn_3d.setCheckable(True)
        self._btn_3d.setChecked(False)
        self._btn_3d.setFixedHeight(24)
        self._btn_3d.setStyleSheet(self._btn_style(active=False))
        self._btn_3d.clicked.connect(lambda: self._switch_mode(_MODE_3D))
        tb_layout.addWidget(self._btn_3d)

        # 飛向路徑捷徑
        self._btn_fly = QPushButton('🎯 飛向')
        self._btn_fly.setFixedHeight(24)
        self._btn_fly.setToolTip('在 3D 模式中飛向目前路徑')
        self._btn_fly.setStyleSheet(
            'QPushButton{background:#0d47a1;color:#e3f2fd;border:none;'
            'border-radius:4px;padding:0 8px;font-size:11px;}'
            'QPushButton:hover{background:#1565C0;}'
        )
        self._btn_fly.clicked.connect(self._on_fly_clicked)
        tb_layout.addWidget(self._btn_fly)

        root.addWidget(toolbar)

        # ── 地圖堆疊 ────────────────────────────────────────────────
        self._stack = QStackedWidget()
        self.map_2d = MapWidget()
        self.map_3d = CesiumMapWidget()
        self._stack.addWidget(self.map_2d)   # index 0
        self._stack.addWidget(self.map_3d)   # index 1
        self._stack.setCurrentIndex(_MODE_2D)
        root.addWidget(self._stack, 1)

    @staticmethod
    def _btn_style(active: bool) -> str:
        if active:
            return (
                'QPushButton{background:#1565C0;color:#e3f2fd;border:none;'
                'border-radius:4px;padding:0 10px;font-size:11px;font-weight:600;}'
                'QPushButton:hover{background:#1976D2;}'
            )
        return (
            'QPushButton{background:#2a3248;color:#8ba3c7;border:1px solid #3a4a60;'
            'border-radius:4px;padding:0 10px;font-size:11px;}'
            'QPushButton:hover{background:#37475f;color:#cfd8dc;}'
        )

    # ─────────────────────────────────────────────────────────────────
    # 信號連接
    # ─────────────────────────────────────────────────────────────────
    def _connect_signals(self):
        # 2D 地圖信號 → 本身信號
        self.map_2d.corner_added.connect(self.corner_added)
        self.map_2d.corner_moved.connect(self.corner_moved)
        self.map_2d.circle_defined.connect(self.circle_defined)
        self.map_2d.nfz_polygon_drawn.connect(self.nfz_polygon_drawn)
        self.map_2d.nfz_circle_drawn.connect(self.nfz_circle_drawn)

        # 3D 地圖信號 → 本身信號（若 3D 模式下點擊也要有效）
        self.map_3d.corner_added.connect(self.corner_added)
        self.map_3d.corner_moved.connect(self.corner_moved)
        self.map_3d.circle_defined.connect(self.circle_defined)
        self.map_3d.nfz_polygon_drawn.connect(self.nfz_polygon_drawn)
        self.map_3d.nfz_circle_drawn.connect(self.nfz_circle_drawn)

    # ─────────────────────────────────────────────────────────────────
    # 模式切換
    # ─────────────────────────────────────────────────────────────────
    def _switch_mode(self, mode: int):
        if mode == self._mode:
            return
        self._mode = mode

        if mode == _MODE_3D:
            # 切到 3D：把 2D 的當前狀態同步過去
            self._sync_to_3d()
            self._stack.setCurrentIndex(_MODE_3D)
            self._mode_label.setText('🌐 3D Cesium')
            self._btn_2d.setChecked(False)
            self._btn_3d.setChecked(True)
            self._btn_2d.setStyleSheet(self._btn_style(active=False))
            self._btn_3d.setStyleSheet(self._btn_style(active=True))
        else:
            self._stack.setCurrentIndex(_MODE_2D)
            self._mode_label.setText('🗺️ 2D 地圖')
            self._btn_2d.setChecked(True)
            self._btn_3d.setChecked(False)
            self._btn_2d.setStyleSheet(self._btn_style(active=True))
            self._btn_3d.setStyleSheet(self._btn_style(active=False))

        logger.info(f'地圖模式切換: {"3D" if mode == _MODE_3D else "2D"}')

    def _sync_to_3d(self):
        """將 2D 地圖的完整狀態同步到 3D 地圖"""
        m = self.map_2d
        self.map_3d.sync_state(
            corners       = m.corners,
            paths         = m.paths,
            path_colors   = m.path_colors,
            path_tooltips = getattr(m, '_path_tooltips', []),
            transit_paths = m.transit_paths,
            circle_center = getattr(m, '_circle_center', None),
            circle_radius = getattr(m, '_circle_radius_m', 0.0),
            home_point    = getattr(m, '_home_point', None),
            nfz_zones     = getattr(m, '_nfz_zones', []),
            swarm_data    = getattr(m, '_swarm_data', None),
        )

    def _on_fly_clicked(self):
        """飛向路徑（3D 模式才有效果）"""
        if self._mode == _MODE_3D:
            self.map_3d._js('flyToScene()')
        else:
            self._switch_mode(_MODE_3D)

    # ─────────────────────────────────────────────────────────────────
    # 以下全部代理到 2D 和 3D（兩者同時更新，切換瞬間完成）
    # ─────────────────────────────────────────────────────────────────

    # ── 路徑顯示 ─────────────────────────────────────────────────────
    def display_path(self, path, altitude: float = 50.0):
        self.map_2d.display_path(path, altitude)
        self.map_3d.display_path(path, altitude)

    def display_paths(self, paths_list, altitude: float = 50.0):
        self.map_2d.display_paths(paths_list, altitude)
        self.map_3d.display_paths(paths_list, altitude)

    def display_fw_paths(self, takeoff, mission, landing):
        self.map_2d.display_fw_paths(takeoff, mission, landing)
        self.map_3d.display_fw_paths(takeoff, mission, landing)

    def set_fw_result(self, result: dict):
        """固定翼 3D 高度資訊（只給 3D 地圖用）"""
        self.map_3d.set_fw_result(result)

    # ── NFZ / 覆蓋層 ─────────────────────────────────────────────────
    def display_nfz_zones(self, nfz_zones: list):
        self.map_2d.display_nfz_zones(nfz_zones)
        self.map_3d.display_nfz_zones(nfz_zones)

    def draw_circle_overlay(self, center_lat, center_lon, radius_m, color='#2196F3'):
        self.map_2d.draw_circle_overlay(center_lat, center_lon, radius_m, color)
        self.map_3d.draw_circle_overlay(center_lat, center_lon, radius_m, color)

    def clear_circle_overlay(self):
        self.map_2d.clear_circle_overlay()
        self.map_3d.clear_circle_overlay()

    def set_home_point_overlay(self, lat, lon):
        self.map_2d.set_home_point_overlay(lat, lon)
        self.map_3d.set_home_point_overlay(lat, lon)

    def clear_home_point_overlay(self):
        self.map_2d.clear_home_point_overlay()
        self.map_3d.clear_home_point_overlay()

    # ── 群飛 ─────────────────────────────────────────────────────────
    def display_swarm_coverage(self, swarm_mission, coverage_paths=None):
        self.map_2d.display_swarm_coverage(swarm_mission, coverage_paths)
        self.map_3d.display_swarm_coverage(swarm_mission, coverage_paths)

    def display_swarm_raw(self, swarm_data: dict):
        self.map_2d.display_swarm_raw(swarm_data)
        self.map_3d.display_swarm_raw(swarm_data)

    def display_survey(self, survey_mission):
        self.map_2d.display_survey(survey_mission)

    # ── 轉場路徑 ─────────────────────────────────────────────────────
    def display_transit_paths(self, transit_paths: list):
        self.map_2d.display_transit_paths(transit_paths)
        self.map_3d.display_transit_paths(transit_paths)

    def clear_transit_paths(self):
        self.map_2d.clear_transit_paths()
        self.map_3d.clear_transit_paths()

    # ── 邊界角點操作 ─────────────────────────────────────────────────
    def add_corner(self, lat, lon):
        self.map_2d.add_corner(lat, lon)
        self.map_3d.add_corner(lat, lon)

    def move_corner(self, index, lat, lon):
        self.map_2d.move_corner(index, lat, lon)
        self.map_3d.move_corner(index, lat, lon)

    def clear_all(self):
        self.map_2d.clear_all()
        self.map_3d.clear_all()

    # ── 屬性代理（讓 main_window 直接存取 corners 等）─────────────────
    @property
    def corners(self) -> List[Tuple[float, float]]:
        return self.map_2d.corners

    @corners.setter
    def corners(self, value):
        self.map_2d.corners = value

    @property
    def paths(self):
        return self.map_2d.paths

    @property
    def edit_mode(self) -> bool:
        return self.map_2d.edit_mode

    # ── 模式設定（NFZ 繪製等特殊模式，只在 2D 實作）────────────────
    def set_edit_mode(self, enabled: bool):
        self.map_2d.set_edit_mode(enabled)

    def set_nfz_poly_draw_mode(self, enabled: bool):
        if hasattr(self.map_2d, 'set_nfz_poly_draw_mode'):
            self.map_2d.set_nfz_poly_draw_mode(enabled)

    def set_nfz_circle_draw_mode(self, enabled: bool):
        if hasattr(self.map_2d, 'set_nfz_circle_draw_mode'):
            self.map_2d.set_nfz_circle_draw_mode(enabled)

    def set_circle_draw_mode(self, enabled: bool):
        if hasattr(self.map_2d, 'set_circle_draw_mode'):
            self.map_2d.set_circle_draw_mode(enabled)

    def set_circle_center_display(self, lat, lon, radius):
        if hasattr(self.map_2d, 'set_circle_center_display'):
            self.map_2d.set_circle_center_display(lat, lon, radius)

    def set_home_point_pick_mode(self, enabled: bool):
        if hasattr(self.map_2d, 'set_home_point_pick_mode'):
            self.map_2d.set_home_point_pick_mode(enabled)

    # ── 其餘屬性（回傳 2D 地圖的值）────────────────────────────────
    def __getattr__(self, name):
        # 任何未定義的方法/屬性，先找 2D 地圖
        try:
            return getattr(self.map_2d, name)
        except AttributeError:
            raise AttributeError(f"'DualMapWidget' object has no attribute '{name}'")

    # ─────────────────────────────────────────────────────────────────
    # SITL 接口（直接代理到 3D 地圖）
    # ─────────────────────────────────────────────────────────────────
    def update_uav_position(self, lat: float, lon: float, alt: float,
                             heading_deg: float = 0.0, speed_ms: float = 0.0,
                             sysid: int = 1, mode: str = '', armed: bool = False,
                             vehicle_type: str = ''):
        """
        SITL / MAVLink 即時 UAV 位置更新（多機支援）。
        只更新 3D 地圖內部狀態，不強制切換模式（否則使用者切到 2D 會一直被踢回 3D）。
        """
        self.map_3d.update_uav_position(lat, lon, alt, heading_deg, speed_ms,
                                         sysid=sysid, mode=mode, armed=armed,
                                         vehicle_type=vehicle_type)
        if hasattr(self.map_2d, 'update_uav_position'):
            self.map_2d.update_uav_position(lat, lon, alt, heading_deg, speed_ms,
                                             sysid=sysid, mode=mode, armed=armed,
                                             vehicle_type=vehicle_type)

    def clear_uav(self):
        self.map_3d.clear_uav()

    def fly_to_position(self, lat, lon, alt=0.0, range_m=600.0):
        if self._mode == _MODE_3D:
            self.map_3d.fly_to_position(lat, lon, alt, range_m)
