"""
SITL HUD 面板 — Mission Planner 風格的即時遙測顯示
顯示：飛行模式、武裝狀態、姿態、速度、電量、GPS、飛行器類型
含連線/斷線控制與連線字串輸入。
"""

import math
from PyQt6.QtCore import Qt, pyqtSignal, QRectF, QPointF
from PyQt6.QtGui import QFont, QPainter, QColor, QPen, QBrush, QPolygonF
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QLineEdit, QFrame, QComboBox, QScrollArea, QSizePolicy,
)


# ══════════════════════════════════════════════════════════════════════
#  AttitudeIndicator — 自繪人工地平儀（姿態球）
# ══════════════════════════════════════════════════════════════════════
class AttitudeIndicator(QWidget):
    """Roll/Pitch/Yaw 姿態球（QPainter 自繪）"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self.setMinimumSize(110, 110)
        self.setMaximumSize(140, 140)

    def set_attitude(self, roll_deg: float, pitch_deg: float, yaw_deg: float = 0.0):
        self._roll = float(roll_deg)
        self._pitch = float(pitch_deg)
        self._yaw = float(yaw_deg)
        self.update()

    def paintEvent(self, _evt):
        w = self.width()
        h = self.height()
        size = min(w, h)
        cx = w / 2
        cy = h / 2
        r = size / 2 - 4

        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        # 圓形裁切
        p.save()
        path_rect = QRectF(cx - r, cy - r, 2 * r, 2 * r)
        p.setClipRect(path_rect)

        # 旋轉以畫出 roll/pitch
        p.translate(cx, cy)
        p.rotate(-self._roll)
        # 1° pitch 約 = r / 45
        pitch_offset = (self._pitch / 45.0) * r

        # 天空（藍）
        sky_rect = QRectF(-r * 2, -r * 2 + pitch_offset, r * 4, r * 2)
        p.fillRect(sky_rect, QColor('#1976D2'))
        # 地面（棕）
        ground_rect = QRectF(-r * 2, pitch_offset, r * 4, r * 2)
        p.fillRect(ground_rect, QColor('#6D4C41'))
        # 地平線
        p.setPen(QPen(QColor('#FFFFFF'), 2))
        p.drawLine(int(-r * 2), int(pitch_offset), int(r * 2), int(pitch_offset))

        # Pitch 刻度
        p.setPen(QPen(QColor('#FFFFFF'), 1))
        for d in (-30, -20, -10, 10, 20, 30):
            y = pitch_offset - (d / 45.0) * r
            wlen = 14 if d % 20 == 0 else 8
            p.drawLine(int(-wlen), int(y), int(wlen), int(y))

        p.restore()

        # 中央十字 + 飛機符號
        p.setPen(QPen(QColor('#FFEB3B'), 3))
        p.drawLine(int(cx - 18), int(cy), int(cx - 6), int(cy))
        p.drawLine(int(cx + 6), int(cy), int(cx + 18), int(cy))
        p.drawLine(int(cx), int(cy - 4), int(cx), int(cy + 4))
        p.setBrush(QColor('#FFEB3B'))
        p.drawEllipse(QPointF(cx, cy), 2.5, 2.5)

        # 外框
        p.setPen(QPen(QColor('#37474F'), 2))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawEllipse(path_rect)

        # Roll 指示三角（頂端）
        p.save()
        p.translate(cx, cy)
        p.rotate(-self._roll)
        tri = QPolygonF([
            QPointF(0, -r + 2),
            QPointF(-5, -r + 12),
            QPointF(5, -r + 12),
        ])
        p.setBrush(QColor('#FFEB3B'))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawPolygon(tri)
        p.restore()

        p.end()


# ══════════════════════════════════════════════════════════════════════
#  UavInfoCard — 單台 UAV 的資訊欄（姿態球 + 遙測）
# ══════════════════════════════════════════════════════════════════════
class UavInfoCard(QFrame):
    """單台無人機的卡片：標題 + 姿態球 + 簡要遙測"""

    def __init__(self, sysid: int, parent=None):
        super().__init__(parent)
        self.sysid = sysid
        self.setFixedWidth(190)
        self.setStyleSheet(
            'QFrame{background:#0d1117;border:1px solid #2a3248;border-radius:6px;}'
            'QLabel{background:transparent;border:none;}'
        )
        v = QVBoxLayout(self)
        v.setContentsMargins(6, 4, 6, 6)
        v.setSpacing(3)

        # 標題列：UAV# + 機型 + 模式
        head = QHBoxLayout()
        self.lbl_title = QLabel(f'UAV{sysid}')
        self.lbl_title.setStyleSheet('color:#FFD700;font-weight:bold;font-size:13px;')
        self.lbl_vehicle = QLabel('---')
        self.lbl_vehicle.setStyleSheet('color:#80cbc4;font-size:10px;')
        head.addWidget(self.lbl_title)
        head.addStretch()
        head.addWidget(self.lbl_vehicle)
        v.addLayout(head)

        mode_row = QHBoxLayout()
        self.lbl_mode = QLabel('---')
        self.lbl_mode.setStyleSheet('color:#FFD700;font-weight:bold;font-size:14px;')
        self.lbl_armed = QLabel('DIS')
        self.lbl_armed.setStyleSheet(
            'color:#888;background:#1a1f2c;padding:1px 6px;'
            'border-radius:3px;font-size:10px;font-weight:bold;'
        )
        mode_row.addWidget(self.lbl_mode)
        mode_row.addStretch()
        mode_row.addWidget(self.lbl_armed)
        v.addLayout(mode_row)

        # 姿態球
        self.attitude = AttitudeIndicator()
        att_row = QHBoxLayout()
        att_row.addStretch()
        att_row.addWidget(self.attitude)
        att_row.addStretch()
        v.addLayout(att_row)

        # 遙測表格
        grid = QGridLayout()
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(2)
        def cap(t):
            l = QLabel(t)
            l.setStyleSheet('color:#78909c;font-size:9px;')
            return l
        def val():
            l = QLabel('--')
            l.setStyleSheet('color:#cfd8dc;font-size:11px;font-family:Consolas,monospace;')
            return l
        self.lbl_alt = val()
        self.lbl_gs  = val()
        self.lbl_hdg = val()
        self.lbl_roll = val()
        self.lbl_pitch = val()
        self.lbl_thr = val()
        grid.addWidget(cap('ALT'), 0, 0); grid.addWidget(self.lbl_alt, 0, 1)
        grid.addWidget(cap('SPD'), 0, 2); grid.addWidget(self.lbl_gs, 0, 3)
        grid.addWidget(cap('HDG'), 1, 0); grid.addWidget(self.lbl_hdg, 1, 1)
        grid.addWidget(cap('THR'), 1, 2); grid.addWidget(self.lbl_thr, 1, 3)
        grid.addWidget(cap('ROLL'),2, 0); grid.addWidget(self.lbl_roll, 2, 1)
        grid.addWidget(cap('PIT'), 2, 2); grid.addWidget(self.lbl_pitch, 2, 3)
        v.addLayout(grid)

        # 底部：GPS / 電池 / 位置
        self.lbl_gps = QLabel('GPS: --')
        self.lbl_gps.setStyleSheet('color:#aed581;font-size:9px;')
        self.lbl_bat = QLabel('🔋 --')
        self.lbl_bat.setStyleSheet('color:#ffb74d;font-size:9px;')
        self.lbl_pos = QLabel('--, --')
        self.lbl_pos.setStyleSheet('color:#90caf9;font-size:9px;')
        v.addWidget(self.lbl_gps)
        v.addWidget(self.lbl_bat)
        v.addWidget(self.lbl_pos)

    def update_from_frame(self, frame):
        self.lbl_mode.setText(frame.mode or '---')
        self.lbl_vehicle.setText(frame.vehicle_type or '---')
        if frame.armed:
            self.lbl_armed.setText('ARM')
            self.lbl_armed.setStyleSheet(
                'color:#fff;background:#2e7d32;padding:1px 6px;'
                'border-radius:3px;font-size:10px;font-weight:bold;'
            )
        else:
            self.lbl_armed.setText('DIS')
            self.lbl_armed.setStyleSheet(
                'color:#888;background:#1a1f2c;padding:1px 6px;'
                'border-radius:3px;font-size:10px;font-weight:bold;'
            )
        self.lbl_alt.setText(f'{frame.alt_rel:5.0f}m')
        self.lbl_gs.setText(f'{frame.ground_speed:4.1f}')
        self.lbl_hdg.setText(f'{frame.heading:3.0f}°')
        self.lbl_roll.setText(f'{frame.roll:+5.1f}°')
        self.lbl_pitch.setText(f'{frame.pitch:+5.1f}°')
        self.lbl_thr.setText(f'{frame.throttle:3d}%')
        self.lbl_gps.setText(f'GPS:{frame.gps_sats}sat HDOP{frame.hdop:.1f}')
        if frame.battery_pct >= 0:
            self.lbl_bat.setText(f'🔋{frame.battery_v:.1f}V {frame.battery_pct}%')
        else:
            self.lbl_bat.setText(f'🔋{frame.battery_v:.1f}V')
        self.lbl_pos.setText(f'📍{frame.lat:.5f},{frame.lon:.5f}')
        self.attitude.set_attitude(frame.roll, frame.pitch, frame.heading)


class SITLHud(QWidget):
    """SITL 連線 + HUD 顯示面板"""

    connect_requested    = pyqtSignal(str)   # 連線字串
    disconnect_requested = pyqtSignal()
    launch_sitl_requested = pyqtSignal(str, int)   # vehicle, count
    stop_sitl_requested   = pyqtSignal()
    # MAVLink 命令
    cmd_arm        = pyqtSignal()
    cmd_disarm     = pyqtSignal()
    cmd_set_mode   = pyqtSignal(str)         # 模式名稱
    cmd_takeoff    = pyqtSignal(float)       # 高度
    cmd_upload     = pyqtSignal()            # 上傳當前任務
    cmd_auto_start = pyqtSignal()            # 一鍵 AUTO+ARM+MISSION_START
    cmd_param_set  = pyqtSignal(str, float, str)   # name, value, ptype
    cmd_param_get  = pyqtSignal(str)
    cmd_params_batch = pyqtSignal(list)            # [(name,value,ptype), ...]

    def __init__(self, parent=None):
        super().__init__(parent)
        self._connected = False
        self._sitl_running = False
        self._init_ui()

    def _init_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        # ── 連線控制區 ─────────────────────────────────────
        ctrl_frame = QFrame()
        ctrl_frame.setStyleSheet(
            'QFrame{background:#161b27;border:1px solid #2a3248;border-radius:6px;}'
        )
        ctrl = QHBoxLayout(ctrl_frame)
        ctrl.setContentsMargins(8, 6, 8, 6)
        ctrl.setSpacing(6)

        ctrl.addWidget(self._lbl('SITL:', '#90caf9', bold=True))

        self.preset_combo = QComboBox()
        self.preset_combo.addItems([
            'udpin:0.0.0.0:14550',     # 接收 SITL/Mission Planner 廣播
            'udp:127.0.0.1:14550',     # 主動 UDP
            'tcp:127.0.0.1:5760',      # SITL TCP
            'tcp:127.0.0.1:5762',      # SITL 第二 TCP
            'COM3,57600',
            'COM7,57600',
        ])
        self.preset_combo.setEditable(True)
        self.preset_combo.setMinimumWidth(180)
        self.preset_combo.setStyleSheet(
            'QComboBox{background:#0d1117;color:#cfd8dc;border:1px solid #37475f;'
            'border-radius:4px;padding:3px 6px;font-size:11px;}'
        )
        ctrl.addWidget(self.preset_combo, 1)

        self.btn_connect = QPushButton('🔌 連線')
        self.btn_connect.setFixedHeight(26)
        self.btn_connect.setStyleSheet(self._btn_style('#1565C0'))
        self.btn_connect.clicked.connect(self._on_connect_clicked)
        ctrl.addWidget(self.btn_connect)

        root.addWidget(ctrl_frame)

        # ── 內建 SITL 啟動列 ──────────────────────────────
        launch_frame = QFrame()
        launch_frame.setStyleSheet(
            'QFrame{background:#161b27;border:1px solid #2a3248;border-radius:6px;}'
        )
        lrow = QHBoxLayout(launch_frame)
        lrow.setContentsMargins(8, 6, 8, 6)
        lrow.setSpacing(6)

        lrow.addWidget(self._lbl('內建 SITL:', '#ffb74d', bold=True))

        self.vehicle_combo = QComboBox()
        self.vehicle_combo.addItems(['PLANE (固定翼)', 'COPTER (多旋翼)'])
        self.vehicle_combo.setStyleSheet(
            'QComboBox{background:#0d1117;color:#cfd8dc;border:1px solid #37475f;'
            'border-radius:4px;padding:3px 6px;font-size:11px;}'
        )
        lrow.addWidget(self.vehicle_combo, 1)

        # 數量
        from PyQt6.QtWidgets import QSpinBox
        self.count_spin = QSpinBox()
        self.count_spin.setRange(1, 6)
        self.count_spin.setValue(1)
        self.count_spin.setSuffix(' 台')
        self.count_spin.setStyleSheet(
            'QSpinBox{background:#0d1117;color:#cfd8dc;border:1px solid #37475f;'
            'border-radius:4px;padding:3px 6px;font-size:11px;}'
        )
        lrow.addWidget(self.count_spin)

        self.btn_launch = QPushButton('🚀 啟動')
        self.btn_launch.setFixedHeight(26)
        self.btn_launch.setStyleSheet(self._btn_style('#2e7d32'))
        self.btn_launch.clicked.connect(self._on_launch_clicked)
        lrow.addWidget(self.btn_launch)

        root.addWidget(launch_frame)

        # ── 多 UAV 卡片橫向滾動區 ─────────────────────────────
        self._uav_cards: dict = {}
        cards_scroll = QScrollArea()
        cards_scroll.setWidgetResizable(True)
        cards_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        cards_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        cards_scroll.setFixedHeight(330)
        cards_scroll.setStyleSheet('QScrollArea{border:none;background:transparent;}')
        cards_holder = QWidget()
        self._cards_row = QHBoxLayout(cards_holder)
        self._cards_row.setContentsMargins(2, 2, 2, 2)
        self._cards_row.setSpacing(6)
        self._cards_row.addStretch()
        cards_scroll.setWidget(cards_holder)
        root.addWidget(cards_scroll)

        # ── 舊版單機 HUD 框架（已停用，但保留 lbl_* 變數避免外部呼叫崩潰）──
        hud_frame = QFrame()
        hud_frame.setVisible(False)
        hud = QVBoxLayout(hud_frame)
        hud.setContentsMargins(10, 8, 10, 8)
        hud.setSpacing(4)

        # 第一行：模式 + 武裝 + 飛行器類型
        row1 = QHBoxLayout()
        self.lbl_mode    = self._big('---', '#FFD700', 18)
        self.lbl_armed   = self._big('DISARMED', '#888', 12)
        self.lbl_vehicle = self._lbl('---', '#80cbc4', bold=True)
        row1.addWidget(self.lbl_mode)
        row1.addStretch()
        row1.addWidget(self.lbl_vehicle)
        row1.addSpacing(8)
        row1.addWidget(self.lbl_armed)
        hud.addLayout(row1)

        # 分隔線
        sep = QFrame(); sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet('color:#2a3248;')
        hud.addWidget(sep)

        # Grid: 主要遙測欄位（HUD 風格）
        grid = QGridLayout()
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(3)

        self.lbl_alt_rel = self._val('-- m')
        self.lbl_alt_msl = self._val('-- m')
        self.lbl_gs      = self._val('-- m/s')
        self.lbl_as      = self._val('-- m/s')
        self.lbl_climb   = self._val('-- m/s')
        self.lbl_hdg     = self._val('--°')
        self.lbl_roll    = self._val('--°')
        self.lbl_pitch   = self._val('--°')
        self.lbl_thr     = self._val('-- %')

        r = 0
        grid.addWidget(self._cap('高度(REL)'), r, 0); grid.addWidget(self.lbl_alt_rel, r, 1)
        grid.addWidget(self._cap('高度(MSL)'), r, 2); grid.addWidget(self.lbl_alt_msl, r, 3)
        r += 1
        grid.addWidget(self._cap('地速'),   r, 0); grid.addWidget(self.lbl_gs,    r, 1)
        grid.addWidget(self._cap('空速'),   r, 2); grid.addWidget(self.lbl_as,    r, 3)
        r += 1
        grid.addWidget(self._cap('爬升率'), r, 0); grid.addWidget(self.lbl_climb, r, 1)
        grid.addWidget(self._cap('航向'),   r, 2); grid.addWidget(self.lbl_hdg,   r, 3)
        r += 1
        grid.addWidget(self._cap('Roll'),   r, 0); grid.addWidget(self.lbl_roll,  r, 1)
        grid.addWidget(self._cap('Pitch'),  r, 2); grid.addWidget(self.lbl_pitch, r, 3)
        r += 1
        grid.addWidget(self._cap('油門'),   r, 0); grid.addWidget(self.lbl_thr,   r, 1)
        hud.addLayout(grid)

        # 分隔線
        sep2 = QFrame(); sep2.setFrameShape(QFrame.Shape.HLine)
        sep2.setStyleSheet('color:#2a3248;')
        hud.addWidget(sep2)

        # 底部：GPS + 電量
        bottom = QHBoxLayout()
        self.lbl_gps = self._lbl('GPS: --', '#aed581')
        self.lbl_bat = self._lbl('🔋 --V', '#ffb74d')
        self.lbl_pos = self._lbl('--, --', '#90caf9')
        bottom.addWidget(self.lbl_gps)
        bottom.addStretch()
        bottom.addWidget(self.lbl_bat)
        hud.addLayout(bottom)

        pos_row = QHBoxLayout()
        pos_row.addWidget(self.lbl_pos)
        pos_row.addStretch()
        hud.addLayout(pos_row)

        root.addWidget(hud_frame)

        # ── 飛控指令區（ARM / 模式切換 / 上傳任務）─────────
        cmd_frame = QFrame()
        cmd_frame.setStyleSheet(
            'QFrame{background:#161b27;border:1px solid #2a3248;border-radius:6px;}'
        )
        cmd_outer = QVBoxLayout(cmd_frame)
        cmd_outer.setContentsMargins(8, 6, 8, 6)
        cmd_outer.setSpacing(5)

        cmd_outer.addWidget(self._lbl('飛控指令', '#90caf9', bold=True))

        # ARM/DISARM 列
        arm_row = QHBoxLayout()
        arm_row.setSpacing(4)
        self.btn_arm    = self._cmd_btn('🔓 ARM',    '#2e7d32')
        self.btn_disarm = self._cmd_btn('🔒 DISARM', '#c62828')
        self.btn_takeoff= self._cmd_btn('🛫 起飛',    '#0288D1')
        self.btn_arm.clicked.connect(self.cmd_arm.emit)
        self.btn_disarm.clicked.connect(self.cmd_disarm.emit)
        self.btn_takeoff.clicked.connect(lambda: self.cmd_takeoff.emit(30.0))
        arm_row.addWidget(self.btn_arm)
        arm_row.addWidget(self.btn_disarm)
        arm_row.addWidget(self.btn_takeoff)
        cmd_outer.addLayout(arm_row)

        # 飛行模式列
        mode_row1 = QHBoxLayout()
        mode_row1.setSpacing(4)
        for mode in ['GUIDED', 'AUTO', 'LOITER']:
            b = self._cmd_btn(mode, '#37474f')
            b.clicked.connect(lambda _, m=mode: self.cmd_set_mode.emit(m))
            mode_row1.addWidget(b)
        cmd_outer.addLayout(mode_row1)

        mode_row2 = QHBoxLayout()
        mode_row2.setSpacing(4)
        for mode in ['RTL', 'LAND', 'STABILIZE']:
            b = self._cmd_btn(mode, '#37474f')
            b.clicked.connect(lambda _, m=mode: self.cmd_set_mode.emit(m))
            mode_row2.addWidget(b)
        cmd_outer.addLayout(mode_row2)

        # 上傳任務
        self.btn_upload = self._cmd_btn('📤 上傳當前任務到 SITL', '#6A1B9A')
        self.btn_upload.clicked.connect(self.cmd_upload.emit)
        cmd_outer.addWidget(self.btn_upload)

        self.btn_auto_start = self._cmd_btn('🚀 一鍵起飛 (AUTO+ARM+START)', '#D84315')
        self.btn_auto_start.clicked.connect(self.cmd_auto_start.emit)
        cmd_outer.addWidget(self.btn_auto_start)

        root.addWidget(cmd_frame)

        # ── 參數寫入區 ────────────────────────────────────
        param_frame = QFrame()
        param_frame.setStyleSheet(
            'QFrame{background:#161b27;border:1px solid #2a3248;border-radius:6px;}'
            'QLineEdit,QComboBox{background:#0d1117;color:#cfd8dc;border:1px solid #37475f;'
            'border-radius:4px;padding:2px 5px;font-size:11px;}'
        )
        pv = QVBoxLayout(param_frame)
        pv.setContentsMargins(8, 6, 8, 6)
        pv.setSpacing(5)
        pv.addWidget(self._lbl('參數寫入 (PARAM_SET)', '#90caf9', bold=True))

        # 常用參數預設按鈕（一鍵套用 SITL 友善設定）
        preset_row = QHBoxLayout()
        preset_row.setSpacing(4)
        btn_preset_sitl = self._cmd_btn('🛠 SITL 友善預設', '#37474f')
        btn_preset_sitl.setToolTip(
            'ARMING_CHECK=0, FS_*=0, GPS 容忍寬鬆 — 方便模擬測試'
        )
        btn_preset_sitl.clicked.connect(self._on_preset_sitl_friendly)
        btn_preset_tune = self._cmd_btn('⚡ 速度/航點調校', '#37474f')
        btn_preset_tune.setToolTip(
            'WPNAV_SPEED / AIRSPEED_CRUISE / WP_RADIUS 調整'
        )
        btn_preset_tune.clicked.connect(self._on_preset_speed_tune)
        btn_load_param = self._cmd_btn('📂 讀取參數檔', '#455a64')
        btn_load_param.setToolTip(
            '讀取 MP 匯出的 .param / .parm 檔並批次寫入 SITL'
        )
        btn_load_param.clicked.connect(self._on_load_param_file)
        preset_row.addWidget(btn_preset_sitl)
        preset_row.addWidget(btn_preset_tune)
        preset_row.addWidget(btn_load_param)
        pv.addLayout(preset_row)

        # 單筆寫入列
        single_row = QHBoxLayout()
        single_row.setSpacing(4)
        self.param_name_edit = QLineEdit()
        self.param_name_edit.setPlaceholderText('參數名稱 (e.g. WPNAV_SPEED)')
        self.param_value_edit = QLineEdit()
        self.param_value_edit.setPlaceholderText('數值')
        self.param_value_edit.setFixedWidth(70)
        self.param_type_combo = QComboBox()
        self.param_type_combo.addItems(['REAL32', 'INT32', 'INT16', 'INT8'])
        self.param_type_combo.setFixedWidth(72)
        single_row.addWidget(self.param_name_edit, 1)
        single_row.addWidget(self.param_value_edit)
        single_row.addWidget(self.param_type_combo)
        pv.addLayout(single_row)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(4)
        btn_set = self._cmd_btn('📝 寫入', '#1565C0')
        btn_set.clicked.connect(self._on_param_set_clicked)
        btn_get = self._cmd_btn('🔍 讀取', '#37474f')
        btn_get.clicked.connect(self._on_param_get_clicked)
        btn_row.addWidget(btn_set)
        btn_row.addWidget(btn_get)
        pv.addLayout(btn_row)

        # 常用參數下拉（點選即填入名稱）
        common_row = QHBoxLayout()
        common_row.setSpacing(4)
        common_row.addWidget(self._lbl('常用:', '#78909c'))
        self.common_param_combo = QComboBox()
        self.common_param_combo.addItems([
            '--選擇--',
            'ARMING_CHECK', 'WPNAV_SPEED', 'WPNAV_SPEED_UP', 'WPNAV_SPEED_DN',
            'WPNAV_ACCEL', 'WPNAV_RADIUS', 'WP_RADIUS', 'WP_LOITER_RAD',
            'AIRSPEED_CRUISE', 'AIRSPEED_MIN', 'AIRSPEED_MAX',
            'TRIM_THROTTLE', 'THR_MAX', 'THR_MIN',
            'RTL_ALT', 'RTL_ALT_MIN', 'LAND_SPEED',
            'FS_THR_ENABLE', 'FS_GCS_ENABLE', 'FS_BATT_ENABLE', 'FS_EKF_ACTION',
            'GPS_HDOP_GOOD', 'EK3_GPS_CHECK',
            'SYSID_THISMAV', 'SR0_POSITION', 'SR0_EXTRA1',
        ])
        self.common_param_combo.currentTextChanged.connect(
            lambda t: self.param_name_edit.setText(t) if t and not t.startswith('--') else None
        )
        common_row.addWidget(self.common_param_combo, 1)
        pv.addLayout(common_row)

        root.addWidget(param_frame)

        # ── 訊息區（多行滾動 log，類似 MP messages）────────
        from PyQt6.QtWidgets import QPlainTextEdit
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(500)
        self.log_view.setStyleSheet(
            'QPlainTextEdit{background:#0e1216;color:#cfd8dc;'
            'font-family:"Consolas",monospace;font-size:10px;'
            'border:1px solid #263238;padding:3px;}'
        )
        self.log_view.setFixedHeight(140)
        root.addWidget(self.log_view)

        # 相容舊程式：lbl_status.setText / setStyleSheet → 轉為 append log
        class _LogShim:
            def __init__(self, view):
                self._view = view
                self._color = '#78909c'
            def setText(self, txt):
                import time as _t
                ts = _t.strftime('%H:%M:%S')
                self._view.appendHtml(
                    f'<span style="color:#607d8b">{ts}</span> '
                    f'<span style="color:{self._color}">{txt}</span>'
                )
                sb = self._view.verticalScrollBar()
                sb.setValue(sb.maximum())
            def setStyleSheet(self, css):
                import re
                m = re.search(r'color:\s*(#[0-9a-fA-F]{3,6})', css)
                if m:
                    self._color = m.group(1)
        self.lbl_status = _LogShim(self.log_view)
        self.lbl_status.setText('未連線')

        root.addStretch()

    # ── 樣式輔助 ─────────────────────────────────────────
    @staticmethod
    def _lbl(text, color='#cfd8dc', bold=False):
        l = QLabel(text)
        weight = '600' if bold else '400'
        l.setStyleSheet(f'color:{color};font-size:11px;font-weight:{weight};')
        return l

    @staticmethod
    def _big(text, color, size):
        l = QLabel(text)
        l.setStyleSheet(
            f'color:{color};font-size:{size}px;font-weight:700;'
            f'font-family:"Consolas","Segoe UI",monospace;'
        )
        return l

    @staticmethod
    def _cap(text):
        l = QLabel(text)
        l.setStyleSheet('color:#78909c;font-size:10px;')
        return l

    @staticmethod
    def _val(text):
        l = QLabel(text)
        l.setStyleSheet(
            'color:#e3f2fd;font-size:13px;font-weight:600;'
            'font-family:"Consolas",monospace;'
        )
        return l

    @staticmethod
    def _cmd_btn(text, bg):
        b = QPushButton(text)
        b.setFixedHeight(26)
        b.setStyleSheet(
            f'QPushButton{{background:{bg};color:#fff;border:none;'
            f'border-radius:4px;padding:0 8px;font-size:11px;font-weight:600;}}'
            f'QPushButton:hover{{background:#455a64;}}'
            f'QPushButton:disabled{{background:#1a1f2e;color:#546e7a;}}'
        )
        return b

    @staticmethod
    def _btn_style(bg):
        return (
            f'QPushButton{{background:{bg};color:#fff;border:none;'
            f'border-radius:4px;padding:0 12px;font-size:11px;font-weight:600;}}'
            f'QPushButton:hover{{background:#1976D2;}}'
            f'QPushButton:disabled{{background:#37475f;color:#90a4ae;}}'
        )

    # ── 公開介面 ─────────────────────────────────────────
    def _on_connect_clicked(self):
        if not self._connected:
            conn = self.preset_combo.currentText().strip()
            if conn:
                self.connect_requested.emit(conn)
                self.lbl_status.setText(f'連線中：{conn} ...')
                self.btn_connect.setEnabled(False)
        else:
            self.disconnect_requested.emit()

    def _on_launch_clicked(self):
        if not self._sitl_running:
            txt = self.vehicle_combo.currentText()
            vehicle = 'PLANE' if 'PLANE' in txt else 'COPTER'
            count = self.count_spin.value()
            self.launch_sitl_requested.emit(vehicle, count)
        else:
            self.stop_sitl_requested.emit()

    def on_sitl_launched(self, vehicle: str, conn_str: str):
        self._sitl_running = True
        self.btn_launch.setText('⏹ 停止')
        self.btn_launch.setStyleSheet(self._btn_style('#c62828'))
        self.lbl_status.setText(f'🚀 SITL {vehicle} 已啟動 → {conn_str}')
        # 自動填入連線字串
        self.preset_combo.setEditText(conn_str)

    def on_sitl_stopped(self):
        self._sitl_running = False
        self.btn_launch.setText('🚀 啟動')
        self.btn_launch.setStyleSheet(self._btn_style('#2e7d32'))

    def on_connected(self, conn_str: str):
        self._connected = True
        self.btn_connect.setText('⛔ 斷線')
        self.btn_connect.setStyleSheet(self._btn_style('#c62828'))
        self.btn_connect.setEnabled(True)
        self.lbl_status.setText(f'✅ 已連線：{conn_str}')

    def on_disconnected(self, reason: str = ''):
        self._connected = False
        self.btn_connect.setText('🔌 連線')
        self.btn_connect.setStyleSheet(self._btn_style('#1565C0'))
        self.btn_connect.setEnabled(True)
        self.lbl_status.setText(f'已斷線：{reason}')
        self.lbl_mode.setText('---')
        self.lbl_armed.setText('DISARMED')
        self.lbl_armed.setStyleSheet(
            'color:#888;font-size:12px;font-weight:700;'
            'font-family:"Consolas",monospace;'
        )

    def on_error(self, msg: str):
        self.btn_connect.setEnabled(True)
        self.lbl_status.setText(f'❌ {msg}')

    def on_status_text(self, severity: int, text: str):
        # severity 0=EMERG ... 7=DEBUG
        color = '#ff5252' if severity <= 3 else '#ffb74d' if severity <= 5 else '#90caf9'
        self.lbl_status.setStyleSheet(f'color:{color};font-size:10px;padding:3px;')
        self.lbl_status.setText(f'[FC] {text}')

    # ── 參數寫入 ─────────────────────────────────────────
    def _on_param_set_clicked(self):
        name = self.param_name_edit.text().strip().upper()
        val_txt = self.param_value_edit.text().strip()
        if not name or not val_txt:
            self.lbl_status.setText('⚠ 請輸入參數名稱與數值')
            return
        try:
            value = float(val_txt)
        except ValueError:
            self.lbl_status.setText('⚠ 數值格式錯誤')
            return
        ptype = self.param_type_combo.currentText()
        self.cmd_param_set.emit(name, value, ptype)
        self.lbl_status.setText(f'📝 PARAM_SET {name} = {value} ({ptype})')

    def _on_param_get_clicked(self):
        name = self.param_name_edit.text().strip().upper()
        if not name:
            self.lbl_status.setText('⚠ 請輸入參數名稱')
            return
        self.cmd_param_get.emit(name)
        self.lbl_status.setText(f'🔍 讀取 {name} ...')

    def _on_preset_sitl_friendly(self):
        items = [
            ('ARMING_CHECK', 0, 'INT32'),
            ('FS_THR_ENABLE', 0, 'INT32'),
            ('FS_GCS_ENABLE', 0, 'INT32'),
            ('FS_BATT_ENABLE', 0, 'INT32'),
            ('FS_EKF_ACTION', 1, 'INT32'),
            ('GPS_HDOP_GOOD', 900, 'INT16'),
            ('SR0_POSITION', 5, 'INT16'),
            ('SR0_EXTRA1', 5, 'INT16'),
            # ── ArduPlane SITL 地面起飛必備 ─────────────
            # 拋投加速度門檻設 0 → 解除武裝後直接給油起飛，不等待手拋
            ('TKOFF_THR_MINACC', 0.0, 'REAL32'),
            # 起飛油門最低 60%、最高 100%
            ('TKOFF_THR_MINSPD', 0.0, 'REAL32'),
            ('TKOFF_THR_MAX', 100, 'INT16'),
            ('TKOFF_THR_DELAY', 0, 'INT8'),
            # 起飛最低俯仰角 10°
            ('TKOFF_LVL_PITCH', 10.0, 'REAL32'),
            # 沒有輪組 → 關閉地面轉向（避免在地上扭翻）
            ('GROUND_STEER_ALT', -1.0, 'REAL32'),
            # 起飛時直接解除地面模式
            ('TKOFF_TDRAG_ELEV', 0, 'INT8'),
            ('TKOFF_TDRAG_SPD1', 0.0, 'REAL32'),
            ('TKOFF_ROTATE_SPD', 0.0, 'REAL32'),
            # 預設巡航/最低空速，避免失速保護誤觸發
            ('AIRSPEED_MIN', 8, 'INT8'),
            ('AIRSPEED_CRUISE', 18, 'INT8'),
            ('AIRSPEED_MAX', 30, 'INT8'),
            # 失速保護關閉（SITL 地面起飛時常誤觸發）
            ('STALL_PREVENTION', 0, 'INT8'),
        ]
        self.cmd_params_batch.emit(items)
        self.lbl_status.setText(f'🛠 已套用 SITL 友善預設 ({len(items)} 參數)')

    def _on_load_param_file(self):
        """讀取 MP 匯出的 .param / .parm 檔並批次寫入 SITL。
        檔案格式每行：  PARAM_NAME<TAB or space>VALUE  ( # 註解略過 )
        """
        from PyQt6.QtWidgets import QFileDialog
        path, _ = QFileDialog.getOpenFileName(
            self, '選擇 ArduPilot 參數檔', '',
            'Param files (*.param *.parm *.txt);;All files (*)'
        )
        if not path:
            return
        items = []
        skipped = 0
        try:
            with open(path, 'r', encoding='utf-8', errors='ignore') as f:
                for raw in f:
                    line = raw.strip()
                    if not line or line.startswith('#'):
                        continue
                    # 支援空白、逗號、tab 分隔
                    parts = line.replace(',', ' ').split()
                    if len(parts) < 2:
                        skipped += 1
                        continue
                    name = parts[0].upper()
                    try:
                        value = float(parts[1])
                    except ValueError:
                        skipped += 1
                        continue
                    # 依值型別自動判斷 ptype
                    if value != int(value):
                        ptype = 'REAL32'
                    else:
                        iv = int(value)
                        if -128 <= iv <= 127:
                            ptype = 'INT8'
                        elif -32768 <= iv <= 32767:
                            ptype = 'INT16'
                        else:
                            ptype = 'INT32'
                    items.append((name, value, ptype))
        except Exception as e:
            self.lbl_status.setText(f'❌ 讀檔失敗: {e}')
            return
        if not items:
            self.lbl_status.setText('⚠ 參數檔為空或格式錯誤')
            return
        self.cmd_params_batch.emit(items)
        self.lbl_status.setText(
            f'📂 已送出 {len(items)} 個參數 (跳過 {skipped} 行) → {path}'
        )

    def _on_preset_speed_tune(self):
        items = [
            ('WPNAV_SPEED', 800, 'REAL32'),     # cm/s, copter
            ('WPNAV_SPEED_UP', 300, 'REAL32'),
            ('WPNAV_SPEED_DN', 200, 'REAL32'),
            ('WPNAV_RADIUS', 100, 'REAL32'),
            ('WP_RADIUS', 5, 'REAL32'),
            ('AIRSPEED_CRUISE', 18, 'REAL32'),  # plane m/s
            ('TRIM_THROTTLE', 50, 'INT8'),
            ('WP_LOITER_RAD', 60, 'REAL32'),
        ]
        self.cmd_params_batch.emit(items)
        self.lbl_status.setText(f'⚡ 已套用速度/航點調校 ({len(items)} 參數)')

    def _get_or_create_card(self, sysid: int) -> 'UavInfoCard':
        card = self._uav_cards.get(sysid)
        if card is None:
            card = UavInfoCard(sysid, self)
            self._uav_cards[sysid] = card
            # 插在最後 stretch 之前
            self._cards_row.insertWidget(self._cards_row.count() - 1, card)
        return card

    def clear_cards(self):
        for card in list(self._uav_cards.values()):
            card.setParent(None)
            card.deleteLater()
        self._uav_cards.clear()

    def on_telemetry(self, frame):
        # 多機：每個 sysid 一個卡片
        try:
            card = self._get_or_create_card(int(frame.sysid))
            card.update_from_frame(frame)
        except Exception:
            pass
        # 仍保留舊版單機路徑（外部 reset 用）
        return self._on_telemetry_legacy(frame)

    def _on_telemetry_legacy(self, frame):
        """frame: TelemetryFrame"""
        # 模式 / 武裝 / 飛行器
        self.lbl_mode.setText(frame.mode)
        if frame.armed:
            self.lbl_armed.setText('ARMED')
            self.lbl_armed.setStyleSheet(
                'color:#ff5252;font-size:12px;font-weight:700;'
                'font-family:"Consolas",monospace;'
            )
        else:
            self.lbl_armed.setText('DISARMED')
            self.lbl_armed.setStyleSheet(
                'color:#888;font-size:12px;font-weight:700;'
                'font-family:"Consolas",monospace;'
            )
        self.lbl_vehicle.setText(f'✈ {frame.vehicle_type}')

        # 數值
        self.lbl_alt_rel.setText(f'{frame.alt_rel:6.1f} m')
        self.lbl_alt_msl.setText(f'{frame.alt_msl:6.1f} m')
        self.lbl_gs.setText(f'{frame.ground_speed:5.1f} m/s')
        self.lbl_as.setText(f'{frame.air_speed:5.1f} m/s')
        self.lbl_climb.setText(f'{frame.climb:+5.1f} m/s')
        self.lbl_hdg.setText(f'{frame.heading:5.0f}°')
        self.lbl_roll.setText(f'{frame.roll:+5.1f}°')
        self.lbl_pitch.setText(f'{frame.pitch:+5.1f}°')
        self.lbl_thr.setText(f'{frame.throttle:3d} %')

        # GPS
        from mission.sitl_link import _GPS_FIX_NAMES
        fix_name = _GPS_FIX_NAMES.get(frame.gps_fix, '?')
        gps_color = '#aed581' if frame.gps_fix >= 3 else '#ff7043'
        self.lbl_gps.setText(f'GPS: {fix_name} · {frame.gps_sats}sat · HDOP {frame.hdop:.1f}')
        self.lbl_gps.setStyleSheet(f'color:{gps_color};font-size:11px;')

        # 電量
        if frame.battery_pct >= 0:
            bat_color = '#ff5252' if frame.battery_pct < 20 else '#ffb74d' if frame.battery_pct < 50 else '#aed581'
            self.lbl_bat.setText(f'🔋 {frame.battery_v:.1f}V · {frame.battery_pct}% · {frame.battery_a:.1f}A')
        else:
            bat_color = '#ffb74d'
            self.lbl_bat.setText(f'🔋 {frame.battery_v:.1f}V · {frame.battery_a:.1f}A')
        self.lbl_bat.setStyleSheet(f'color:{bat_color};font-size:11px;')

        # 座標
        self.lbl_pos.setText(f'📍 {frame.lat:.6f}, {frame.lon:.6f}')
