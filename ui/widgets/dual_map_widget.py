"""
雙模式地圖容器
將 2D (Folium/Leaflet) 與 3D (Cesium) 地圖整合在同一個 QStackedWidget 中，
提供無縫切換功能，對外介面與原 MapWidget 完全相容。
"""

from typing import List, Tuple, Optional

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QStackedWidget,
    QPushButton, QLabel, QFrame, QMenu,
)
from PyQt6.QtGui import QAction
from PyQt6.QtCore import pyqtSignal, Qt

from ui.widgets.map_widget import MapWidget
from ui.widgets.cesium_map_widget import CesiumMapWidget
from ui.resources.tactical_theme import TacticalColors as TC, TacticalFonts as TF
from ui.resources.aeroplan_theme.widgets import IconButton
from utils.logger import get_logger


# ── MIL-STD-1472H 工具列按鈕 QSS（單一事實來源）──────────────────────
def _btn_qss(active: bool = False, accent: str = None) -> str:
    """
    工具列按鈕統一 QSS，全直角、TacticalColors，無漸層、無圓角。

    active=True ：使用者選定狀態（NEUTRAL 高亮）
    accent      ：覆寫文字色（語意：FRIENDLY/HOSTILE/WARNING/NEUTRAL/AMBER）
    """
    if active:
        bg, fg, hov_bg = TC.NEUTRAL, TC.BG_PRIMARY, TC.BORDER_STRONG
        weight = '600'
    else:
        bg = TC.BG_SECONDARY
        fg = accent or TC.FG_PRIMARY
        hov_bg = TC.BG_ELEVATED
        weight = '500'
    return (
        f'QPushButton{{background:{bg};color:{fg};border:1px solid {TC.BORDER_DEFAULT};'
        f'border-radius:0;padding:0 10px;font-size:11px;font-weight:{weight};'
        f'font-family:{TF.css_condensed()};letter-spacing:1px;}}'
        f'QPushButton:hover{{background:{hov_bg};color:{TC.FG_PRIMARY};}}'
        f'QPushButton:disabled{{background:{TC.BG_SUNKEN};color:{TC.FG_MUTED};}}'
    )

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
    fence_built       = pyqtSignal(object)  # 自動建構 Geofence 完成（傳 MissionBundle 或 None）
    nfz_polygon_drawn = pyqtSignal(list)
    nfz_circle_drawn  = pyqtSignal(float, float, float)
    strike_target_added = pyqtSignal(float, float)  # 打擊目標標記
    # ── Fence Zone 工具列觸發信號（NFZ + 威脅 + 圍籬統一） ──
    fence_zone_draw_polygon_requested = pyqtSignal()
    fence_zone_draw_circle_requested  = pyqtSignal()
    fence_zone_manage_requested       = pyqtSignal()

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
            f'QFrame{{background:{TC.BG_SECONDARY};'
            f'border-bottom:1px solid {TC.BORDER_DEFAULT};border-radius:0;}}'
        )
        tb_layout = QHBoxLayout(toolbar)
        tb_layout.setContentsMargins(8, 0, 8, 0)
        tb_layout.setSpacing(6)

        # 模式指示標籤（採用 AeroPlan Design System emphasis 角色：琥珀色高亮）
        self._mode_label = QLabel('2D 地圖')
        self._mode_label.setProperty('role', 'emphasis')
        self._mode_label.style().polish(self._mode_label)
        tb_layout.addWidget(self._mode_label)
        tb_layout.addStretch()

        # 2D 按鈕（AeroPlan IconButton + 自訂 active 狀態 QSS 覆寫）
        self._btn_2d = IconButton('map_2d', '2D 衛星', tone='ghost', compact=True)
        self._btn_2d.setCheckable(True)
        self._btn_2d.setChecked(True)
        self._btn_2d.setFixedHeight(24)
        self._btn_2d.setStyleSheet(self._btn_style(active=True))
        self._btn_2d.clicked.connect(lambda: self._switch_mode(_MODE_2D))
        tb_layout.addWidget(self._btn_2d)

        # 3D 按鈕
        self._btn_3d = IconButton('map_3d', '3D Cesium', tone='ghost', compact=True)
        self._btn_3d.setCheckable(True)
        self._btn_3d.setChecked(False)
        self._btn_3d.setFixedHeight(24)
        self._btn_3d.setStyleSheet(self._btn_style(active=False))
        self._btn_3d.clicked.connect(lambda: self._switch_mode(_MODE_3D))
        tb_layout.addWidget(self._btn_3d)

        # 飛向路徑捷徑
        self._btn_fly = IconButton('fly_to', '飛向', tone='ghost', compact=True)
        self._btn_fly.setFixedHeight(24)
        self._btn_fly.setToolTip('在 3D 模式中飛向目前路徑')
        self._btn_fly.setStyleSheet(_btn_qss(accent=TC.FG_EMPHASIS))
        self._btn_fly.clicked.connect(self._on_fly_clicked)
        tb_layout.addWidget(self._btn_fly)

        # 跟隨相機（追尾第三人稱視角）
        self._chase_sysid = 0   # 0 = 關閉
        self._fpv_sysid   = 0   # 0 = 關閉
        self._fpv_mode    = 'forward'  # 'forward' | 'down'
        self._btn_chase = IconButton('drone', '跟隨', tone='ghost', compact=True)
        self._btn_chase.setFixedHeight(24)
        self._btn_chase.setToolTip('模擬飛行鏡頭：鎖定飛機後方跟隨（僅 3D 模式）')
        self._btn_chase.setStyleSheet(_btn_qss())
        self._btn_chase.clicked.connect(self._on_chase_clicked)
        tb_layout.addWidget(self._btn_chase)

        # 電子圍籬（Geofence）— 強制飛安政策，路徑生成自動建構
        self._fence_visible = True   # 預設顯示
        self._btn_fence = IconButton('polygon', '圍籬', tone='ghost', compact=True)
        self._btn_fence.setCheckable(True)
        self._btn_fence.setChecked(True)
        self._btn_fence.setFixedHeight(24)
        self._btn_fence.setToolTip(
            '電子圍籬：每次規劃路徑會自動建立 4 頂點矩形圍籬\n'
            'FENCE_TYPE=7 (MaxAlt+Circle+Polygon)、FENCE_ACTION=1 (RTL)\n'
            '點擊切換顯示 / 隱藏'
        )
        self._btn_fence.setStyleSheet(_btn_qss(active=True))
        self._btn_fence.clicked.connect(self._on_fence_toggle)
        tb_layout.addWidget(self._btn_fence)

        # ── Fence Zone（NFZ + 威脅 + 圍籬）3 個按鈕：地圖永久可見 ──
        # 主面板 / 各分頁切換時也仍然能直接從這裡新增區域
        self._btn_fz_poly = IconButton(
            'polygon', '⊕ NFZ/威脅 多邊形', tone='ghost', compact=True,
        )
        self._btn_fz_poly.setFixedHeight(24)
        self._btn_fz_poly.setToolTip(
            '在 3D 地圖點擊新增頂點，雙擊或按「完成多邊形」結束\n'
            '完成後自動開對話框：選分類 (NFZ / 威脅 / 圍籬) + 海拔上下限 + 命名'
        )
        self._btn_fz_poly.setStyleSheet(_btn_qss(accent=TC.HOSTILE))
        self._btn_fz_poly.clicked.connect(
            self.fence_zone_draw_polygon_requested.emit
        )
        tb_layout.addWidget(self._btn_fz_poly)

        self._btn_fz_circ = IconButton(
            'circle', '⊕ NFZ/威脅 圓形', tone='ghost', compact=True,
        )
        self._btn_fz_circ.setFixedHeight(24)
        self._btn_fz_circ.setToolTip(
            '按住滑鼠從圓心往外拖曳定義圓形 → 完成後自動開對話框預填'
        )
        self._btn_fz_circ.setStyleSheet(_btn_qss(accent=TC.HOSTILE))
        self._btn_fz_circ.clicked.connect(
            self.fence_zone_draw_circle_requested.emit
        )
        tb_layout.addWidget(self._btn_fz_circ)

        self._btn_fz_manage = IconButton(
            'settings', '管理 Fence', tone='ghost', compact=True,
        )
        self._btn_fz_manage.setFixedHeight(24)
        self._btn_fz_manage.setToolTip(
            '開啟統一 Fence 對話框 — 手動輸入頂點 / 圓心、編輯既有區域'
        )
        self._btn_fz_manage.setStyleSheet(_btn_qss(accent=TC.WARNING))
        self._btn_fz_manage.clicked.connect(
            self.fence_zone_manage_requested.emit
        )
        tb_layout.addWidget(self._btn_fz_manage)

        # 地形跟隨（terrain-following at constant AGL）— 解決路徑撞山問題
        self._btn_tf = IconButton('tool', '地形跟隨', tone='ghost', compact=True)
        self._btn_tf.setFixedHeight(24)
        self._btn_tf.setToolTip(
            '地形跟隨：每個航點高度 = 地形海拔 + AGL\n'
            '保持固定離地高度，避免撞山（需先載入 DEM）'
        )
        self._btn_tf.setStyleSheet(_btn_qss())
        self._btn_tf.clicked.connect(self._on_tf_clicked)
        tb_layout.addWidget(self._btn_tf)

        # FPV 機上相機（第一人稱 / 雲台偵查）
        self._btn_fpv = IconButton('fpv', 'FPV', tone='ghost', compact=True)
        self._btn_fpv.setFixedHeight(24)
        self._btn_fpv.setToolTip('機上相機視角：前視 / 俯視偵查 + 可調 FOV（僅 3D 模式）')
        self._btn_fpv.setStyleSheet(_btn_qss())
        self._btn_fpv.clicked.connect(self._on_fpv_clicked)
        tb_layout.addWidget(self._btn_fpv)

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
        return _btn_qss(active=active)

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
        # 3D 點擊加角點時，同步寫回 map_2d.corners，這樣：
        #   (a) SAR 熱力圖讀取 map_widget.corners (= map_2d.corners) 能拿到完整資料；
        #   (b) 日後切回 2D 時，2D 地圖已有正確的角點狀態。
        self.map_3d.corner_added.connect(self._on_3d_corner_added)
        self.map_3d.corner_moved.connect(self._on_3d_corner_moved)
        self.map_3d.circle_defined.connect(self.circle_defined)
        self.map_3d.nfz_polygon_drawn.connect(self.nfz_polygon_drawn)
        self.map_3d.nfz_circle_drawn.connect(self.nfz_circle_drawn)
        self.map_3d.strike_target_added.connect(self.strike_target_added)

    def _on_3d_corner_added(self, lat: float, lon: float):
        """3D 點擊加角點：把資料鏡射到 2D 內部狀態（不重新 emit 2D 訊號以避免重複）"""
        if (lat, lon) not in self.map_2d.corners:
            self.map_2d.add_corner(lat, lon)
        self.corner_added.emit(lat, lon)

    def _on_3d_corner_moved(self, index: int, lat: float, lon: float):
        """3D 移動角點：同步到 2D"""
        if 0 <= index < len(self.map_2d.corners):
            self.map_2d.move_corner(index, lat, lon)
        self.corner_moved.emit(index, lat, lon)

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
            self._mode_label.setText('3D Cesium')
            self._btn_2d.setChecked(False)
            self._btn_3d.setChecked(True)
            self._btn_2d.setStyleSheet(self._btn_style(active=False))
            self._btn_3d.setStyleSheet(self._btn_style(active=True))
        else:
            self._stack.setCurrentIndex(_MODE_2D)
            self._mode_label.setText('2D 地圖')
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
    # 跟隨相機（Chase Camera）
    # ─────────────────────────────────────────────────────────────────
    def _on_chase_clicked(self):
        """
        點擊跟隨按鈕：彈出選單選擇要鎖定的 UAV（若當前已鎖定則顯示關閉選項）。
        僅列出目前 SITL 面板上已註冊的機體；若未啟動則預設 UAV1~UAV3。
        """
        # 切到 3D 模式，否則跟隨無畫面可看
        if self._mode != _MODE_3D:
            self._switch_mode(_MODE_3D)

        menu = QMenu(self._btn_chase)
        active = self._chase_sysid

        if active > 0:
            act_off = QAction(f'🛑 關閉跟隨 (當前: UAV{active})', self)
            act_off.triggered.connect(lambda: self._apply_chase(0))
            menu.addAction(act_off)
            act_reset = QAction('🎯 歸零視角（回飛機正後方）', self)
            act_reset.triggered.connect(self.map_3d.reset_chase_view)
            menu.addAction(act_reset)
            menu.addSeparator()

        # 從 SITL 即時狀態取得已註冊的 sysid；fallback 到 1..5
        sysids = self._known_sysids() or [1, 2, 3, 4, 5]
        for sid in sysids:
            mark = '✓ ' if sid == active else '   '
            act = QAction(f'{mark}鎖定 UAV{sid}', self)
            act.triggered.connect(lambda _, s=sid: self._apply_chase(s))
            menu.addAction(act)

        # 參數預設值子選單
        menu.addSeparator()
        for label, dist, hgt, pit in [
            ('距離 60m / 俯角 -8°',   60, 15, -8),
            ('距離 80m / 俯角 -10°',  80, 20, -10),
            ('距離 120m / 俯角 -15°', 120, 30, -15),
            ('距離 200m / 俯角 -20°', 200, 50, -20),
        ]:
            a = QAction(label, self)
            a.triggered.connect(
                lambda _, d=dist, h=hgt, p=pit: self.map_3d.set_chase_camera_params(
                    distance=d, height=h, pitch_deg=p
                )
            )
            menu.addAction(a)

        # 彈出選單在按鈕下方
        pos = self._btn_chase.mapToGlobal(self._btn_chase.rect().bottomLeft())
        menu.exec(pos)

    def _apply_chase(self, sysid: int):
        self._chase_sysid = max(0, int(sysid))
        self.map_3d.set_chase_camera(self._chase_sysid)
        if self._chase_sysid > 0:
            # 兩模式互斥：chase 啟動就關掉 FPV 的 UI 狀態
            self._fpv_sysid = 0
            self._set_btn_style(self._btn_fpv, False, 'FPV')
            self._set_btn_style(self._btn_chase, True, f'跟隨 UAV{self._chase_sysid}')
        else:
            self._set_btn_style(self._btn_chase, False, '跟隨')

    # ── FPV 第一人稱相機 ──────────────────────────────────────────
    def _on_fpv_clicked(self):
        """
        FPV 選單：選擇鎖定的 UAV + 雲台模式 + FOV 預設。
        """
        if self._mode != _MODE_3D:
            self._switch_mode(_MODE_3D)

        menu = QMenu(self._btn_fpv)
        active = self._fpv_sysid

        if active > 0:
            act_off = QAction(f'🛑 關閉 FPV (當前: UAV{active})', self)
            act_off.triggered.connect(lambda: self._apply_fpv(0))
            menu.addAction(act_off)
            menu.addSeparator()

        sysids = self._known_sysids() or [1, 2, 3, 4, 5]

        # 前視（機頭方向 FPV）
        fwd_menu = menu.addMenu('🚁 機頭前視 (FPV)')
        for sid in sysids:
            mark = '✓ ' if (sid == active and self._fpv_mode == 'forward') else '   '
            act = QAction(f'{mark}UAV{sid}', self)
            act.triggered.connect(
                lambda _, s=sid: self._apply_fpv(s, mode='forward')
            )
            fwd_menu.addAction(act)

        # 俯視偵查（雲台 -90°）
        down_menu = menu.addMenu('📷 俯視偵查鏡頭 (雲台 -90°)')
        for sid in sysids:
            mark = '✓ ' if (sid == active and self._fpv_mode == 'down') else '   '
            act = QAction(f'{mark}UAV{sid}', self)
            act.triggered.connect(
                lambda _, s=sid: self._apply_fpv(s, mode='down')
            )
            down_menu.addAction(act)

        # FOV 預設值
        menu.addSeparator()
        fov_menu = menu.addMenu('🔭 FOV 視角')
        for label, fov in [
            ('窄角 60°（長焦偵查）', 60),
            ('標準 75°（一般無人機）', 75),
            ('廣角 100°（GoPro 類）', 100),
            ('魚眼 130°（競速 FPV）', 130),
        ]:
            a = QAction(label, self)
            a.triggered.connect(
                lambda _, f=fov: self.map_3d.set_fpv_camera_params(fov_deg=f)
            )
            fov_menu.addAction(a)

        # 雲台 Roll 跟隨
        roll_menu = menu.addMenu('🎚️ 機體 Roll 跟隨')
        for label, rf in [
            ('開啟（沉浸式 / 會傾斜）', True),
            ('關閉（雲台穩定 / 地平線水平）', False),
        ]:
            a = QAction(label, self)
            a.triggered.connect(
                lambda _, r=rf: self.map_3d.set_fpv_camera_params(roll_follow=r)
            )
            roll_menu.addAction(a)

        pos = self._btn_fpv.mapToGlobal(self._btn_fpv.rect().bottomLeft())
        menu.exec(pos)

    def _apply_fpv(self, sysid: int, mode: str = None):
        """套用 FPV 相機設定。mode='forward' 為平視，'down' 為雲台俯視偵查。"""
        sysid = max(0, int(sysid))
        if mode is not None:
            self._fpv_mode = mode
        gimbal = -90.0 if self._fpv_mode == 'down' else 0.0

        self._fpv_sysid = sysid
        self.map_3d.set_fpv_camera(sysid, gimbal_pitch_deg=gimbal)

        if self._fpv_sysid > 0:
            # 關掉 chase 的 UI（JS 端已自動互斥）
            self._chase_sysid = 0
            self._set_btn_style(self._btn_chase, False, '跟隨')
            tag = '俯視' if self._fpv_mode == 'down' else '前視'
            self._set_btn_style(self._btn_fpv, True, f'FPV {tag} UAV{self._fpv_sysid}')
        else:
            self._set_btn_style(self._btn_fpv, False, 'FPV')

    @staticmethod
    def _set_btn_style(btn: QPushButton, active: bool, text: str):
        btn.setText(text)
        if active:
            btn.setStyleSheet(_btn_qss(active=True))
        else:
            btn.setStyleSheet(_btn_qss())

    def _known_sysids(self) -> list:
        """回傳目前已出現在 SITL 的 sysid 列表（由 update_uav_position 累積）。"""
        return sorted(getattr(self, '_active_sysids', set()))

    # ─────────────────────────────────────────────────────────────────
    # 以下全部代理到 2D 和 3D（兩者同時更新，切換瞬間完成）
    # ─────────────────────────────────────────────────────────────────

    # ── 路徑顯示（每次自動建構 Geofence —— 強制飛安政策）───────────
    def display_path(self, path, altitude: float = 50.0):
        self.map_2d.display_path(path, altitude)
        self.map_3d.display_path(path, altitude)
        self._auto_build_fence([path], altitude)

    def display_paths(self, paths_list, altitude: float = 50.0):
        self.map_2d.display_paths(paths_list, altitude)
        self.map_3d.display_paths(paths_list, altitude)
        self._auto_build_fence(paths_list, altitude)

    def display_fw_paths(self, takeoff, mission, landing):
        self.map_2d.display_fw_paths(takeoff, mission, landing)
        self.map_3d.display_fw_paths(takeoff, mission, landing)
        # 三段都要納入圍籬範圍；固定翼建議 buffer ≥ 100m
        all_paths = []
        for p in (takeoff, mission, landing):
            if p:
                all_paths.append(p)
        self._auto_build_fence(all_paths, altitude=None, buffer_m=120.0)

    # ── Geofence 自動建構 ─────────────────────────────────────────
    def _auto_build_fence(self, paths_list, altitude=None, buffer_m: float = 30.0):
        """
        強制飛安：將所有航點包進矩形圍籬並上 3D 地圖渲染。
        失敗（航點 <2 個、shapely 計算錯誤等）僅記錄，不阻斷顯示。
        """
        try:
            from mission.geofence_manager import GeofenceConstraintManager
        except ImportError:
            return
        # 把所有 path 攤平成 (lat, lon[, alt]) 串列
        flat = []
        for path in (paths_list or []):
            for p in (path or []):
                if len(p) >= 3:
                    flat.append((float(p[0]), float(p[1]), float(p[2])))
                elif len(p) >= 2:
                    a = float(altitude) if altitude is not None else 50.0
                    flat.append((float(p[0]), float(p[1]), a))
        if len(flat) < 2:
            self.map_3d.clear_geofence()
            self._last_fence_bundle = None
            self.fence_built.emit(None)
            return
        try:
            mgr = GeofenceConstraintManager(buffer_radius_m=buffer_m)
            bundle = mgr.build(waypoints=flat)
        except Exception as e:
            logger.warning(f'[Geofence] 建構失敗: {e}')
            self.fence_built.emit(None)
            return
        self._last_fence_bundle = bundle
        if self._fence_visible:
            self.map_3d.set_geofence(bundle.geofence)
        logger.info(bundle.summary())
        self.fence_built.emit(bundle)

    def _on_fence_toggle(self):
        self._fence_visible = self._btn_fence.isChecked()
        if self._fence_visible and getattr(self, '_last_fence_bundle', None):
            self.map_3d.set_geofence(self._last_fence_bundle.geofence)
            self._btn_fence.setStyleSheet(_btn_qss(active=True))
            self._btn_fence.setText('圍籬')
        else:
            self.map_3d.clear_geofence()
            self._btn_fence.setStyleSheet(_btn_qss())
            self._btn_fence.setText('圍籬 (隱藏)')

    @property
    def last_fence_bundle(self):
        """供 main_window 取得最近一次自動建構的 MissionBundle（含 fence_params）"""
        return getattr(self, '_last_fence_bundle', None)

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

    def clear_paths(self):
        """清除所有飛行路徑 (2D + 3D 同步)

        主視窗的「清除路徑」按鈕與 Esc 快捷鍵會呼叫此方法。
        若只代理到 2D 地圖，3D Cesium 會殘留路徑視覺，造成錯覺。
        """
        if hasattr(self.map_2d, 'clear_paths'):
            self.map_2d.clear_paths()
        if hasattr(self.map_3d, 'clear_paths'):
            self.map_3d.clear_paths()

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

    # ── 模式設定 ─────────────────────────────────────────────────────
    def set_edit_mode(self, enabled: bool):
        self.map_2d.set_edit_mode(enabled)

    def set_nfz_poly_draw_mode(self, enabled: bool):
        # Legacy 名稱（poly）— 同時啟用 2D + 3D 繪製模式
        if hasattr(self.map_2d, 'set_nfz_poly_draw_mode'):
            self.map_2d.set_nfz_poly_draw_mode(enabled)
        if hasattr(self.map_3d, 'set_nfz_polygon_draw_mode'):
            self.map_3d.set_nfz_polygon_draw_mode(enabled)

    def set_nfz_polygon_draw_mode(self, enabled: bool):
        """NFZ 多邊形繪製：2D + 3D 同時啟用，使用者在哪邊都能畫。"""
        if hasattr(self.map_2d, 'set_nfz_polygon_draw_mode'):
            self.map_2d.set_nfz_polygon_draw_mode(enabled)
        elif hasattr(self.map_2d, 'set_nfz_poly_draw_mode'):
            self.map_2d.set_nfz_poly_draw_mode(enabled)
        if hasattr(self.map_3d, 'set_nfz_polygon_draw_mode'):
            self.map_3d.set_nfz_polygon_draw_mode(enabled)

    def finish_nfz_polygon(self):
        """完成多邊形：2D + 3D 都呼叫（哪個在 draw 模式就完成哪個）。"""
        if hasattr(self.map_2d, 'finish_nfz_polygon'):
            self.map_2d.finish_nfz_polygon()
        if hasattr(self.map_3d, 'finish_nfz_polygon'):
            self.map_3d.finish_nfz_polygon()

    def set_nfz_circle_draw_mode(self, enabled: bool):
        """NFZ 圓形拖曳：2D + 3D 同時啟用。"""
        if hasattr(self.map_2d, 'set_nfz_circle_draw_mode'):
            self.map_2d.set_nfz_circle_draw_mode(enabled)
        if hasattr(self.map_3d, 'set_nfz_circle_draw_mode'):
            self.map_3d.set_nfz_circle_draw_mode(enabled)

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
                             vehicle_type: str = '',
                             pitch_deg: float = 0.0, roll_deg: float = 0.0):
        """
        SITL / MAVLink 即時 UAV 位置更新（多機支援，含姿態同步）。
        只更新 3D 地圖內部狀態，不強制切換模式（否則使用者切到 2D 會一直被踢回 3D）。
        """
        # 記錄出現過的 sysid，供跟隨選單列出
        if not hasattr(self, '_active_sysids'):
            self._active_sysids = set()
        self._active_sysids.add(int(sysid))

        self.map_3d.update_uav_position(lat, lon, alt, heading_deg, speed_ms,
                                         sysid=sysid, mode=mode, armed=armed,
                                         vehicle_type=vehicle_type,
                                         pitch_deg=pitch_deg, roll_deg=roll_deg)
        if hasattr(self.map_2d, 'update_uav_position'):
            self.map_2d.update_uav_position(lat, lon, alt, heading_deg, speed_ms,
                                             sysid=sysid, mode=mode, armed=armed,
                                             vehicle_type=vehicle_type)

    def set_chase_camera(self, sysid: int):
        """公開 API：程式化啟用/關閉跟隨相機。"""
        self._apply_chase(sysid)

    def set_chase_camera_params(self, distance: float = None, height: float = None,
                                 pitch_deg: float = None, smoothing: float = None):
        """公開 API：調整跟隨相機距離 / 高度 / 俯角 / 平滑係數。"""
        self.map_3d.set_chase_camera_params(distance, height, pitch_deg, smoothing)

    def reset_chase_view(self):
        """公開 API：歸零跟隨鏡頭視角回飛機正後方。"""
        self.map_3d.reset_chase_view()

    # ─────────────────────────────────────────────────────────────
    # 地形跟隨（Terrain-Following at constant AGL）
    # ─────────────────────────────────────────────────────────────
    def _on_tf_clicked(self):
        """彈出選單選擇 AGL 高度，啟用地形跟隨。"""
        if self._mode != _MODE_3D:
            self._switch_mode(_MODE_3D)

        # 先檢查 DEM 是否載入
        dem_loaded = (getattr(self.map_3d, '_dem_manager', None) is not None
                      and getattr(self.map_3d._dem_manager, '_loaded', False))

        menu = QMenu(self._btn_tf)
        if not dem_loaded:
            from PyQt6.QtGui import QAction as _QA
            warn = _QA('⚠ 尚未載入 DEM — 請先載入地形檔', self)
            warn.setEnabled(False)
            menu.addAction(warn)
            pos = self._btn_tf.mapToGlobal(self._btn_tf.rect().bottomLeft())
            menu.exec(pos)
            return

        active = float(getattr(self.map_3d, 'terrain_following_agl', 0.0))

        if active > 0:
            act_off = QAction(f'🛑 關閉地形跟隨（當前 AGL={active:.0f}m）', self)
            act_off.triggered.connect(lambda: self._apply_tf(0))
            menu.addAction(act_off)
            menu.addSeparator()

        for label, agl in [
            ('30 m AGL（低空巡查）',  30),
            ('50 m AGL（標準掃描）',  50),
            ('80 m AGL（一般測繪）',  80),
            ('100 m AGL（高空覆蓋）', 100),
            ('150 m AGL',           150),
            ('200 m AGL',           200),
        ]:
            mark = '✓ ' if abs(active - agl) < 0.5 else '   '
            act = QAction(f'{mark}{label}', self)
            act.triggered.connect(lambda _, a=agl: self._apply_tf(a))
            menu.addAction(act)

        pos = self._btn_tf.mapToGlobal(self._btn_tf.rect().bottomLeft())
        menu.exec(pos)

    def _apply_tf(self, agl: float):
        """套用地形跟隨 AGL 設定。agl<=0 為關閉。"""
        agl = max(0.0, float(agl))
        if agl > 0:
            self.map_3d.set_terrain_following(agl)
            self._btn_tf.setText(f'跟隨 AGL={agl:.0f}m')
            self._btn_tf.setStyleSheet(_btn_qss(active=True))
        else:
            self.map_3d.clear_terrain_following()
            self._btn_tf.setText('地形跟隨')
            self._btn_tf.setStyleSheet(_btn_qss())

    def set_terrain_following(self, agl: float):
        """公開 API：程式化啟用 / 關閉地形跟隨。"""
        self._apply_tf(agl)

    def set_dem_manager(self, dem_manager):
        """公開 API：注入 DEMTerrainManager（main_window 在 DEM 載入後呼叫）。"""
        self.map_3d.set_dem_manager(dem_manager)

    def set_fpv_camera(self, sysid: int, mode: str = 'forward'):
        """公開 API：程式化啟用 FPV。mode='forward' 平視 / 'down' 俯視偵查。"""
        self._apply_fpv(sysid, mode=mode)

    def set_fpv_camera_params(self, gimbal_pitch_deg: float = None,
                               fov_deg: float = None, roll_follow: bool = None,
                               forward_offset: float = None):
        """公開 API：微調 FPV 雲台俯角 / FOV / roll 跟隨 / 機頭前偏移。"""
        self.map_3d.set_fpv_camera_params(gimbal_pitch_deg, fov_deg,
                                           roll_follow, forward_offset)

    def clear_uav(self):
        self.map_3d.clear_uav()

    def fly_to_position(self, lat, lon, alt=0.0, range_m=600.0):
        if self._mode == _MODE_3D:
            self.map_3d.fly_to_position(lat, lon, alt, range_m)

    # ─────────────────────────────────────────────────────────────────
    # 戰術模組代理（只作用於 3D Cesium 地圖）
    # ─────────────────────────────────────────────────────────────────
    def update_elevation_slicer(self, min_alt: float, max_alt: float):
        self.map_3d.update_elevation_slicer(min_alt, max_alt)

    def clear_elevation_slicer(self):
        self.map_3d.clear_elevation_slicer()

    def update_fov_cone(self, lat, lon, alt, fov_radius=50.0,
                        heading_deg=0.0, pitch_deg=0.0, roll_deg=0.0,
                        sysid: int = 1,
                        hfov_deg: float = 0.0,
                        vfov_deg: float = 0.0,
                        mount_angle_deg: float = 0.0):
        """更新指定 UAV 的 FOV 光錐（多機支援）

        傳入 hfov_deg/vfov_deg/mount_angle_deg（皆 > 0）時，3D 地圖切換為
        梯形角錐 frustum 模式；否則保持 legacy 圓錐視覺。
        """
        self.map_3d.update_fov_cone(lat, lon, alt, fov_radius,
                                     heading_deg, pitch_deg, roll_deg,
                                     sysid=sysid,
                                     hfov_deg=hfov_deg,
                                     vfov_deg=vfov_deg,
                                     mount_angle_deg=mount_angle_deg)

    def clear_fov_cone(self, sysid: int = None):
        """清除 FOV 光錐 (sysid=None 清全部)"""
        self.map_3d.clear_fov_cone(sysid=sysid)

    def init_sar_heatmap(self, lat_min, lat_max, lon_min, lon_max,
                         rows=20, cols=20, sweep_width=50.0, quality=0.8):
        self.map_3d.init_sar_heatmap(lat_min, lat_max, lon_min, lon_max,
                                      rows, cols, sweep_width, quality)

    def update_heatmap(self, uav_lat, uav_lon, fov_radius=50.0, sysid: int = 0):
        """sysid > 0 → 改用 frustum 梯形覆蓋（隨 HFOV/VFOV/掛載角即時變化）"""
        self.map_3d.update_heatmap(uav_lat, uav_lon, fov_radius, sysid=sysid)

    def clear_sar_heatmap(self):
        self.map_3d.clear_sar_heatmap()

    def reset_sar_heatmap(self):
        self.map_3d.reset_sar_heatmap()

    def add_radar_dome(self, lat, lon, alt=0.0, radius=5000.0, name=''):
        self.map_3d.add_radar_dome(lat, lon, alt, radius, name)

    def clear_radar_domes(self):
        self.map_3d.clear_radar_domes()

    def update_rcs_sensitivity(self, uav_lat, uav_lon, uav_alt,
                               uav_heading=0.0, sysid=1):
        self.map_3d.update_rcs_sensitivity(uav_lat, uav_lon, uav_alt,
                                            uav_heading, sysid)

    def clear_rcs_sensitivity(self, sysid=1):
        self.map_3d.clear_rcs_sensitivity(sysid)

    def animate_radar_scan(self, radar_idx=0, duration_ms=2000):
        self.map_3d.animate_radar_scan(radar_idx, duration_ms)

    # ── Fence Zone (unified NFZ / Threat / Geofence) ────────────────
    def add_fence_zone(self, zone_id, vertices, alt_min, alt_max,
                       name, color_hex, category, inclusion):
        self.map_3d.add_fence_zone(zone_id, vertices, alt_min, alt_max,
                                   name, color_hex, category, inclusion)

    def remove_fence_zone(self, zone_id):
        self.map_3d.remove_fence_zone(zone_id)

    def clear_fence_zones(self):
        self.map_3d.clear_fence_zones()
