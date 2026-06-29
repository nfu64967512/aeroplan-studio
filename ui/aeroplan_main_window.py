"""
AeroPlanMainWindow — MIL-STD-1472H 合規主視窗框架

本模組提供符合美國國防部人機工程設計標準 MIL-STD-1472H 的標準軍規
主視窗骨架，作為 AeroPlan Studio 全系統 UI 的統一外殼。

版面分區 (Layout Regions)
--------------------------
    ┌─────────────────────────────────────────────────────────────┐
    │  ①  TopStatusStrip          MIL-STD §5.2.3 / §5.17.14       │
    ├───────────┬──────────────────────────────────┬─────────────┤
    │           │                                  │ ③ Master    │
    │  ②        │  ④ Central Display Area          │   Warning   │
    │  Left     │     (Map / Video / 3D View)      │   §5.7.3.8  │
    │  Nav      │     §5.1.2.2.1 Functional        │             │
    │  Rail     │         Grouping                 ├─────────────┤
    │           │                                  │ ⑤ Telemetry │
    │           │                                  │   §5.17.18  │
    │           │                                  │             │
    │           │                                  │ ⑥ Arm/Comm  │
    │           │                                  │             │
    ├───────────┴──────────────────────────────────┴─────────────┤
    │  ⑦  System Status Bar       §5.17.10.7 Constructive Msgs    │
    └─────────────────────────────────────────────────────────────┘

MIL-STD-1472H 條款對應
------------------------
- §5.1.2.2.1  功能分組 (Functional grouping) —— ②⑤⑥ 皆獨立 QGroupBox
- §5.2.3      顯示器易讀性 —— 深背景淺文字、1px 邊線
- §5.7.3.8    主告警面板獨立 —— ③ 永遠可見、不被分頁遮蔽
- §5.17.10.7  建設性錯誤訊息 —— ⑦ 狀態列 + MilStdMessageBox
- §5.17.14    狀態指示器 —— ① 頂部狀態列
- §5.17.18.7  字型：無襯線+數字等寬 —— 由 Global_MIL_STD.qss 統一規範
- §5.17.25    色彩語意 —— 紅/黃/綠/琥珀，禁裝飾用
- §5.17.25.16 暗適應 —— 深藏青背景 #0A0F14
"""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Optional

from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QAction, QKeySequence, QShortcut
from PyQt6.QtWidgets import (
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMenuBar,
    QSizePolicy,
    QSplitter,
    QStackedWidget,
    QStatusBar,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from ui.qt_utils import repolish
from ui.resources.tactical_theme import TacticalColors, TacticalFonts
from ui.widgets.master_warning_panel import (
    AlertLevel,
    MasterWarningPanel,
)

# ── ADOS 對齊新元件（Phase A/B/C/J/K/M/N） ─────────────────────────
from ui.widgets.alert_banner import AlertBannerStack
from ui.widgets.fleet_dashboard import FleetDashboard
from ui.widgets.hud_composer import HudComposer, HudFrame
from ui.widgets.parameters_browser import ParametersBrowser
from ui.widgets.command_console import CommandConsole
from mission.fleet_registry import FleetRegistry
from mission.sitl_link import TelemetryFrame


# ══════════════════════════════════════════════════════════════════════
#  TopStatusStrip — ① 頂部狀態列（§5.17.14）
# ══════════════════════════════════════════════════════════════════════
class TopStatusStrip(QWidget):
    """頂部全局狀態列。

    顯示：系統模式 / GPS 狀態 / LINK 品質 / 任務時間 / UTC 時間。
    所有讀值為等寬字型，確保跳動時視覺穩定。
    """

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setObjectName("TopStatusStrip")
        self._build_ui()

        # UTC 時鐘以 1 Hz 更新
        self._clock = QTimer(self)
        self._clock.setInterval(1000)
        self._clock.timeout.connect(self._update_clock)
        self._clock.start()
        self._update_clock()

    def _build_ui(self) -> None:
        row = QHBoxLayout(self)
        row.setContentsMargins(10, 0, 10, 0)
        row.setSpacing(0)

        # ── 系統模式 ─────────────────────────
        self.mode_caption = QLabel("MODE")
        self.mode_value = QLabel("STANDBY")
        self.mode_value.setProperty("indicator", "value")

        # ── GPS 狀態 ─────────────────────────
        self.gps_caption = QLabel("GPS")
        self.gps_value = QLabel("—")
        self.gps_value.setProperty("indicator", "value")

        # ── Link 品質 ────────────────────────
        self.link_caption = QLabel("LINK")
        self.link_value = QLabel("—%")
        self.link_value.setProperty("indicator", "value")

        # ── 任務時間 ─────────────────────────
        self.mission_caption = QLabel("MISSION T+")
        self.mission_value = QLabel("00:00:00")
        self.mission_value.setProperty("indicator", "value")

        # ── UTC 時鐘 ─────────────────────────
        self.utc_caption = QLabel("UTC")
        self.utc_value = QLabel("--:--:--")
        self.utc_value.setProperty("indicator", "value")

        groups: list[tuple[QLabel, QLabel]] = [
            (self.mode_caption, self.mode_value),
            (self.gps_caption, self.gps_value),
            (self.link_caption, self.link_value),
            (self.mission_caption, self.mission_value),
            (self.utc_caption, self.utc_value),
        ]
        for i, (cap, val) in enumerate(groups):
            row.addWidget(cap)
            row.addWidget(val)
            if i < len(groups) - 1:
                sep = QFrame()
                sep.setProperty("role", "separator")
                sep.setFrameShape(QFrame.Shape.VLine)
                sep.setFixedWidth(1)
                row.addWidget(sep)
                row.addSpacing(4)
        row.addStretch(1)

    # ------------------------------------------------------------------
    # 公開 setter
    # ------------------------------------------------------------------
    def set_mode(self, mode: str, color: Optional[str] = None) -> None:
        """設定系統模式文字與可選色彩。"""
        self.mode_value.setText(mode.upper())
        if color:
            self.mode_value.setStyleSheet(f"color: {color};")

    def set_gps(self, fix: str, sats: Optional[int] = None) -> None:
        """設定 GPS 定位資訊（如 3D / 15 sats）。"""
        txt = fix.upper()
        if sats is not None:
            txt = f"{txt}  {sats:02d} SAT"
        self.gps_value.setText(txt)
        # 語意色：3D=綠, 2D=黃, NO FIX=紅
        color_map = {
            "3D": TacticalColors.FRIENDLY,
            "2D": TacticalColors.WARNING,
            "NO FIX": TacticalColors.HOSTILE,
        }
        c = color_map.get(fix.upper(), TacticalColors.FG_EMPHASIS)
        self.gps_value.setStyleSheet(f"color: {c};")

    def set_link(self, quality_pct: float) -> None:
        """設定通訊品質百分比，依閾值自動上色。"""
        self.link_value.setText(f"{quality_pct:3.0f}%")
        if quality_pct < 30:
            c = TacticalColors.HOSTILE
        elif quality_pct < 60:
            c = TacticalColors.WARNING
        else:
            c = TacticalColors.FRIENDLY
        self.link_value.setStyleSheet(f"color: {c};")

    def set_mission_elapsed(self, seconds: int) -> None:
        """設定任務已執行時間（秒）。"""
        h, rem = divmod(max(0, seconds), 3600)
        m, s = divmod(rem, 60)
        self.mission_value.setText(f"{h:02d}:{m:02d}:{s:02d}")

    def _update_clock(self) -> None:
        """每秒更新 UTC 時間顯示。"""
        now = datetime.now(timezone.utc)
        self.utc_value.setText(now.strftime("%H:%M:%S"))


# ══════════════════════════════════════════════════════════════════════
#  NavigationRail — ② 左側功能導航
# ══════════════════════════════════════════════════════════════════════
class NavigationRail(QWidget):
    """左側功能導航欄（可切換主視覺區頁面）。

    以 QToolButton(checkable) + QButtonGroup 行為呈現分頁選項，
    點擊發出 page_selected(index)。階層標籤使用窄體大寫字型。
    """

    page_selected = pyqtSignal(int)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setObjectName("NavigationRail")
        self.setFixedWidth(176)

        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(0, 8, 0, 8)
        self._layout.setSpacing(0)

        self._buttons: list[QToolButton] = []

        # 預設分頁：後續由 add_page() 補充
        self._layout.addStretch(1)

    def add_page(self, label: str, tooltip: str = "") -> int:
        """新增一個導航按鈕，回傳其索引。"""
        btn = QToolButton(self)
        btn.setText(label.upper())
        if tooltip:
            btn.setToolTip(tooltip)
        btn.setCheckable(True)
        btn.setAutoExclusive(True)
        btn.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextOnly)
        btn.setSizePolicy(QSizePolicy.Policy.Expanding,
                          QSizePolicy.Policy.Fixed)

        idx = len(self._buttons)
        btn.clicked.connect(lambda _, i=idx: self.page_selected.emit(i))

        # 插入到 stretch 之前
        self._layout.insertWidget(self._layout.count() - 1, btn)
        self._buttons.append(btn)

        if idx == 0:
            btn.setChecked(True)
        return idx

    def set_current(self, idx: int) -> None:
        """程式化選取指定分頁。"""
        if 0 <= idx < len(self._buttons):
            self._buttons[idx].setChecked(True)
            self.page_selected.emit(idx)


# ══════════════════════════════════════════════════════════════════════
#  TelemetryReadoutGroup — ⑤ 遙測讀值群組（§5.17.18.7.2）
# ══════════════════════════════════════════════════════════════════════
class TelemetryReadoutGroup(QGroupBox):
    """功能分組：遙測讀值（§5.1.2.2.1）。

    以三欄格網呈現：姿態 (ATT) / 位置 (POS) / 能量 (PWR)。
    所有數字強制等寬字型，避免跳動時抖動。
    """

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__("TELEMETRY / 遙測", parent)
        self._readouts: dict[str, QLabel] = {}
        self._build_ui()

    def _build_ui(self) -> None:
        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 18, 8, 8)
        outer.setSpacing(8)

        # 三個子分組：姿態、位置、能量
        outer.addWidget(self._make_subgroup("ATTITUDE / 姿態", [
            ("ROLL",  "ROLL",  "°"),
            ("PITCH", "PITCH", "°"),
            ("YAW",   "YAW",   "°"),
        ]))
        outer.addWidget(self._make_subgroup("POSITION / 位置", [
            ("ALT",    "ALT",   "m"),
            ("GSPD",   "GSPD",  "m/s"),
            ("HDG",    "HDG",   "°"),
        ]))
        outer.addWidget(self._make_subgroup("POWER / 能量", [
            ("BATT",   "BATT",  "%"),
            ("VOLT",   "VOLT",  "V"),
            ("CURR",   "CURR",  "A"),
        ]))

    def _make_subgroup(self, title: str,
                       rows: list[tuple[str, str, str]]) -> QWidget:
        """建立一組階層式標籤 + 3 個讀值列。"""
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(2)

        cap = QLabel(title)
        cap.setProperty("role", "subsection-title")
        v.addWidget(cap)

        for key, label, unit in rows:
            row = QHBoxLayout()
            row.setContentsMargins(0, 0, 0, 0)
            row.setSpacing(6)

            lbl = QLabel(label)
            lbl.setProperty("role", "telemetry-label")
            lbl.setFixedWidth(48)
            row.addWidget(lbl)

            val = QLabel("—")
            val.setProperty("role", "telemetry-value")
            val.setAlignment(
                Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter
            )
            row.addWidget(val, 1)

            u = QLabel(unit)
            u.setProperty("role", "telemetry-unit")
            u.setFixedWidth(28)
            row.addWidget(u)

            v.addLayout(row)
            self._readouts[key] = val

        return w

    # ------------------------------------------------------------------
    # 公開 setter — 上層僅需 set_value(key, value, status)
    # ------------------------------------------------------------------
    def set_value(self, key: str, value: float,
                  status: str = "") -> None:
        """更新指定遙測項的數值與狀態色。

        Parameters
        ----------
        key : str
            遙測鍵（ROLL, PITCH, YAW, ALT, GSPD, HDG, BATT, VOLT, CURR）。
        value : float
            數值。格式化依 key 自動選擇小數位數。
        status : {"", "nominal", "caution", "critical"}
            語意狀態；觸發 QSS 色彩切換。
        """
        lbl = self._readouts.get(key)
        if lbl is None:
            return
        fmt = {"ROLL": "{:+6.1f}", "PITCH": "{:+6.1f}", "YAW": "{:+6.1f}",
               "ALT": "{:6.1f}", "GSPD": "{:5.1f}", "HDG": "{:5.1f}",
               "BATT": "{:5.1f}", "VOLT": "{:5.2f}", "CURR": "{:5.2f}"}
        lbl.setText(fmt.get(key, "{:.2f}").format(value))
        lbl.setProperty("status", status or "")
        repolish(lbl)


# ══════════════════════════════════════════════════════════════════════
#  ArmCommGroup — ⑥ 武裝與通訊面板
# ══════════════════════════════════════════════════════════════════════
class ArmCommGroup(QGroupBox):
    """功能分組：武裝狀態與通訊狀態（§5.1.2.2.1）。

    嚴格遵循色彩語意：
        ARM=綠 / SAFE=黃 / FAIL=紅
    """

    arm_requested = pyqtSignal(bool)    # True=ARM, False=SAFE
    abort_requested = pyqtSignal()

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__("ARMAMENT & COMMS / 武裝 & 通訊", parent)
        self._build_ui()

    def _build_ui(self) -> None:
        from PyQt6.QtWidgets import QPushButton

        v = QVBoxLayout(self)
        v.setContentsMargins(8, 18, 8, 8)
        v.setSpacing(8)

        # ── 武裝狀態指示 ──
        arm_cap = QLabel("ARMAMENT STATUS / 武裝狀態")
        arm_cap.setProperty("role", "subsection-title")
        v.addWidget(arm_cap)

        self.arm_status = QLabel("SAFE")
        self.arm_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.arm_status.setFont(TacticalFonts.condensed(size=16, bold=True,
                                                        letter_spacing=3.0))
        self.arm_status.setStyleSheet(
            f"color: {TacticalColors.WARNING}; "
            f"border: 1px solid {TacticalColors.WARNING}; padding: 10px;"
        )
        v.addWidget(self.arm_status)

        # ── 武裝操作按鈕 ──
        btn_row = QHBoxLayout()
        btn_row.setSpacing(4)

        self.arm_btn = QPushButton("ARM")
        self.arm_btn.setProperty("tacticalRole", "execute")
        self.arm_btn.clicked.connect(lambda: self.arm_requested.emit(True))

        self.safe_btn = QPushButton("SAFE")
        self.safe_btn.setProperty("tacticalRole", "caution")
        self.safe_btn.clicked.connect(lambda: self.arm_requested.emit(False))

        self.abort_btn = QPushButton("ABORT")
        self.abort_btn.setProperty("tacticalRole", "abort")
        self.abort_btn.clicked.connect(self.abort_requested.emit)

        btn_row.addWidget(self.arm_btn)
        btn_row.addWidget(self.safe_btn)
        btn_row.addWidget(self.abort_btn)
        v.addLayout(btn_row)

        # ── 通訊狀態 ──
        comm_cap = QLabel("COMMS / 通訊")
        comm_cap.setProperty("role", "subsection-title")
        v.addWidget(comm_cap)

        self.comm_rf = self._make_comm_row("RF LINK", "—")
        self.comm_gcs = self._make_comm_row("GCS LINK", "—")
        self.comm_video = self._make_comm_row("VIDEO", "—")
        v.addLayout(self.comm_rf[0])
        v.addLayout(self.comm_gcs[0])
        v.addLayout(self.comm_video[0])

        v.addStretch(1)

    def _make_comm_row(self, label: str, value: str) -> tuple[QHBoxLayout, QLabel]:
        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        lbl = QLabel(label)
        lbl.setProperty("role", "telemetry-label")
        val = QLabel(value)
        val.setProperty("role", "telemetry-value")
        val.setAlignment(
            Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter
        )
        row.addWidget(lbl, 0)
        row.addWidget(val, 1)
        return row, val

    # ------------------------------------------------------------------
    # 公開 setter
    # ------------------------------------------------------------------
    def set_armed(self, armed: bool, failed: bool = False) -> None:
        """設定武裝狀態指示燈。"""
        if failed:
            self.arm_status.setText("FAIL")
            c = TacticalColors.HOSTILE
        elif armed:
            self.arm_status.setText("ARMED")
            c = TacticalColors.FRIENDLY
        else:
            self.arm_status.setText("SAFE")
            c = TacticalColors.WARNING
        self.arm_status.setStyleSheet(
            f"color: {c}; border: 1px solid {c}; padding: 10px;"
        )


# ══════════════════════════════════════════════════════════════════════
#  AeroPlanMainWindow — 主視窗
# ══════════════════════════════════════════════════════════════════════
class AeroPlanMainWindow(QMainWindow):
    """AeroPlan Studio 軍規主視窗（MIL-STD-1472H 合規骨架）。

    使用方式
    ----------
    >>> win = AeroPlanMainWindow()
    >>> win.nav.add_page("MISSION", "任務規劃")
    >>> win.central_stack.addWidget(my_mission_page)
    >>> win.show()
    """

    # 供上層 controller 連接的高階信號
    mission_mode_changed = pyqtSignal(str)

    def __init__(self, parent: Optional[QWidget] = None,
                 *,
                 window_title: str = (
                     "AeroPlan Studio — MIL-STD-1472H Compliant UI"
                 ),
                 min_size: tuple[int, int] = (1280, 720)) -> None:
        super().__init__(parent)
        self.setWindowTitle(window_title)
        self.setMinimumSize(*min_size)

        # 建構各分區
        self._build_menubar()
        self._build_central()
        self._build_statusbar()
        self._install_shortcuts()

    # ------------------------------------------------------------------
    # ① + 選單列
    # ------------------------------------------------------------------
    def _build_menubar(self) -> None:
        """標準軍規選單列（階層式分類，§5.1.2.2.1）。"""
        mb: QMenuBar = self.menuBar()

        m_file = mb.addMenu("&FILE  /  檔案")
        self.act_new = QAction("NEW MISSION  (Ctrl+N)", self)
        self.act_open = QAction("OPEN MISSION  (Ctrl+O)", self)
        self.act_save = QAction("SAVE MISSION  (Ctrl+S)", self)
        self.act_export = QAction("EXPORT WPL    (Ctrl+E)", self)
        self.act_exit = QAction("EXIT          (Ctrl+Q)", self)
        for a in (self.act_new, self.act_open, self.act_save,
                  self.act_export):
            m_file.addAction(a)
        m_file.addSeparator()
        m_file.addAction(self.act_exit)
        self.act_exit.triggered.connect(self.close)

        m_view = mb.addMenu("&VIEW  /  檢視")
        self.act_fullscreen = QAction("FULLSCREEN    (F11)", self)
        self.act_fullscreen.setCheckable(True)
        self.act_fullscreen.triggered.connect(self._toggle_fullscreen)
        m_view.addAction(self.act_fullscreen)

        m_help = mb.addMenu("&HELP  /  說明")
        self.act_about = QAction("ABOUT", self)
        m_help.addAction(self.act_about)

    # ------------------------------------------------------------------
    # 中央分區（①②③④⑤⑥）
    # ------------------------------------------------------------------
    def _build_central(self) -> None:
        """建構主要版面：頂部狀態列 + 三欄式（左導航/中央/右側面板）。

        相對於 ADOS，本視窗額外保留 §5.1.2.2.1 左側 NavigationRail（軍規功能分組），
        Top bar 僅對齊 ADOS 視覺風格（wordmark + status indicators）。
        參見 plans/...risks/8 的差異說明。
        """
        root = QWidget()
        self.setCentralWidget(root)
        v = QVBoxLayout(root)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)

        # ── ADOS Command Shell 頂欄（wordmark） ───────────────
        topbar = QWidget(self)
        topbar.setObjectName("CommandShellTopBar")
        tb_layout = QHBoxLayout(topbar)
        tb_layout.setContentsMargins(0, 0, 0, 0)
        tb_layout.setSpacing(0)
        wordmark = QLabel("AEROPLAN  STUDIO", topbar)
        wordmark.setObjectName("WordmarkLabel")
        tb_layout.addWidget(wordmark)
        tb_layout.addStretch(1)
        v.addWidget(topbar)

        # ── ① 頂部狀態列 ─────────────────────
        self.top_strip = TopStatusStrip(self)
        v.addWidget(self.top_strip)

        # ── Alert Banner Stack（位於頂欄之下、splitter 之上）─
        self.alert_stack = AlertBannerStack(self)
        v.addWidget(self.alert_stack)

        # ── 三欄 Splitter：左=Nav / 中=Stack / 右=Side ──
        splitter = QSplitter(Qt.Orientation.Horizontal, self)
        splitter.setHandleWidth(1)
        splitter.setChildrenCollapsible(False)

        # ② Navigation Rail
        self.nav = NavigationRail(self)
        splitter.addWidget(self.nav)

        # ④ Central Stack
        self.central_stack = QStackedWidget(self)
        self.central_stack.setContentsMargins(0, 0, 0, 0)
        # 預設 placeholder
        placeholder = QLabel(
            "CENTRAL DISPLAY AREA\n\n"
            "— attach map / 3D / video widgets via central_stack.addWidget() —"
        )
        placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        # 灰色 placeholder 文字（背景沿用全域 QWidget BG_PRIMARY 預設）
        placeholder.setProperty('role', 'caption')
        placeholder.style().polish(placeholder)
        placeholder.setFont(TacticalFonts.condensed(size=11, bold=True,
                                                    letter_spacing=2.0))
        self.central_stack.addWidget(placeholder)
        splitter.addWidget(self.central_stack)

        # 右側欄：③ Master Warning + ⑤ Telemetry + ⑥ Arm/Comm
        self.right_panel = QWidget(self)
        rp_layout = QVBoxLayout(self.right_panel)
        rp_layout.setContentsMargins(6, 6, 6, 6)
        rp_layout.setSpacing(6)

        # ③ Master Warning — 永遠置於最上方
        self.master_warning = MasterWarningPanel(self.right_panel)
        self.master_warning.setMinimumHeight(160)
        rp_layout.addWidget(self.master_warning)

        # ⑤ Telemetry
        self.telemetry = TelemetryReadoutGroup(self.right_panel)
        rp_layout.addWidget(self.telemetry)

        # ⑥ Arm / Comm
        self.armcomm = ArmCommGroup(self.right_panel)
        rp_layout.addWidget(self.armcomm)

        rp_layout.addStretch(1)

        self.right_panel.setMinimumWidth(320)
        splitter.addWidget(self.right_panel)

        # Splitter 初始比例：左 176px | 中 stretch | 右 340px
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setStretchFactor(2, 0)
        splitter.setSizes([176, 1000, 340])

        v.addWidget(splitter, 1)

        # 導航信號 → 切換中央 stack
        self.nav.page_selected.connect(self._on_nav_selected)

        # ── ADOS 對齊：Dashboard / Parameters / HUD 三個 nav 頁 ──
        # 取代原 placeholder：addWidget 後保留 placeholder 在 index 0，
        # 真實頁面從 index 1 開始。
        self._setup_ados_pages()

    def _setup_ados_pages(self) -> None:
        """掛入 ADOS 風格的 Dashboard / Parameters / HUD 頁面。

        FleetDashboard.current_uav_changed → 同步 ParametersBrowser sysid
        與 HudComposer 內部的 callsign 顯示；HUD 由 FleetRegistry 推送 telemetry。
        """
        # Dashboard
        self.fleet_dashboard = FleetDashboard(self)
        self.add_nav_page("DASHBOARD", self.fleet_dashboard,
                          tooltip="Fleet overview · multi-UAV monitoring")

        # Command Console（ADOS 主視角：Rail + DroneDetail + Logs；
        # 是 SITL 真實 GCS 互動的核心頁）
        self.command_console = CommandConsole(self)
        self.add_nav_page("COMMAND", self.command_console,
                          tooltip="Tactical command console · GCS for SITL")

        # Parameters
        self.parameters_browser = ParametersBrowser(self)
        self.add_nav_page("PARAMETERS", self.parameters_browser,
                          tooltip="ArduPilot parameters · searchable")

        # HUD
        self.hud_composer = HudComposer(parent=self)
        self.add_nav_page("HUD", self.hud_composer,
                          tooltip="Tactical HUD · 60 FPS")

        # 串接：dashboard 或 console 選機 → parameters / HUD context 切換
        self.fleet_dashboard.current_uav_changed.connect(self._on_current_uav_changed)
        self.command_console.current_uav_changed.connect(self._on_current_uav_changed)
        # console rail 「+」按鈕 → 觸發 SITL Launcher（如可從主視窗呼叫）
        self.command_console.add_button_handler(self._on_console_add_uav_clicked)

        # 訂閱 FleetRegistry 推 HUD 用的 telemetry
        FleetRegistry.instance().telemetry_updated.connect(self._on_uav_telemetry)
        self._hud_seq: int = 0
        self._current_callsign: Optional[str] = None

    def _on_current_uav_changed(self, callsign: str) -> None:
        """Dashboard → 切換 current UAV 給 Parameters / HUD。"""
        self._current_callsign = callsign
        # 同步右側 telemetry strip 模式欄
        try:
            link = FleetRegistry.instance().get_link(callsign)
            sysid = 0
            if link is not None:
                # 公開 sysid 來自 link.sysid_label，非不存在的 _frame 屬性。
                sysid = int(getattr(link, "sysid_label", 0) or 0)
            if sysid:
                self.parameters_browser.set_current_sysid(sysid)
        except Exception:
            pass

    def _on_console_add_uav_clicked(self) -> None:
        """Rail 「+」按鈕：開 SITLLaunchDialog 啟動新機。"""
        try:
            from ui.dialogs.sitl_launch_dialog import SITLLaunchDialog
            cfg = SITLLaunchDialog.get_config("COPTER", 1, self)
            if cfg is not None:
                self.show_status(
                    f"SITL launch configured: {len(cfg.get('instances', []))} instance(s)",
                    severity="info",
                )
                # 真實 spawn 由 上層 strike_controller / mission_manager 處理；
                # 此處只負責顯示對話框，避免主視窗承擔 launcher 生命週期管理。
        except Exception as exc:
            self.show_status(f"SITL launch error: {exc}", severity="critical")

    def _on_uav_telemetry(self, callsign: str, frame: TelemetryFrame) -> None:
        """把當前 UAV 的 telemetry 映射為 HudFrame 推 HUD。"""
        if callsign != self._current_callsign:
            return
        self._hud_seq += 1
        hud = HudFrame(
            frame_sequence=self._hud_seq,
            callsign=callsign,
            mode=frame.mode,
            armed=frame.armed,
            roll_deg=frame.roll,
            pitch_deg=frame.pitch,
            yaw_deg=frame.yaw,
            ground_speed_ms=frame.ground_speed,
            air_speed_ms=frame.air_speed,
            alt_msl_m=frame.alt_msl,
            alt_rel_m=frame.alt_rel,
            heading_deg=frame.heading,
            throttle_pct=int(frame.throttle),
            battery_pct=int(frame.battery_pct),
            battery_v=frame.battery_v,
            gps_fix=int(frame.gps_fix),
            gps_sats=int(frame.gps_sats),
        )
        self.hud_composer.set_frame(hud)
        # 同步 top strip 模式欄
        try:
            self.top_strip.mode_value.setText(frame.mode or "—")
        except Exception:
            pass

    # ------------------------------------------------------------------
    # ⑦ 狀態列
    # ------------------------------------------------------------------
    def _build_statusbar(self) -> None:
        """系統狀態列：最近一則訊息 + 操作提示。"""
        bar: QStatusBar = self.statusBar()
        self._status_label = QLabel("SYSTEM READY  /  系統就緒")
        self._status_label.setProperty("msgSeverity", "info")
        bar.addWidget(self._status_label, 1)

        # 右側固定提示（灰色 caption 字體）
        hint = QLabel("F1: HELP   |   Ctrl+Q: EXIT")
        hint.setProperty('role', 'caption')
        hint.style().polish(hint)
        bar.addPermanentWidget(hint)

    # ------------------------------------------------------------------
    # 快捷鍵
    # ------------------------------------------------------------------
    def _install_shortcuts(self) -> None:
        """綁定標準操作鍵。"""
        QShortcut(QKeySequence("Ctrl+Q"), self, activated=self.close)
        QShortcut(QKeySequence("F11"), self,
                  activated=self._toggle_fullscreen)

    # ------------------------------------------------------------------
    # 公開 API
    # ------------------------------------------------------------------
    def add_nav_page(self, label: str, page: QWidget,
                     tooltip: str = "") -> int:
        """便利方法：同時新增左側導航與中央 stack 頁面。

        Parameters
        ----------
        label : str
            導航鈕文字（會自動大寫）。
        page : QWidget
            中央顯示頁面。
        tooltip : str
            Tooltip 說明文字。

        Returns
        -------
        int
            該頁面在 stack 中的索引。
        """
        idx_nav = self.nav.add_page(label, tooltip)
        idx_stack = self.central_stack.addWidget(page)
        # 若為第一個真實頁面，移除 placeholder 並選取
        if idx_nav == 0:
            self.central_stack.setCurrentIndex(idx_stack)
        return idx_stack

    def show_status(self, message: str,
                    severity: str = "info",
                    timeout_ms: int = 5000) -> None:
        """於狀態列顯示訊息，採建設性/中立語氣（§5.17.10.7）。

        Parameters
        ----------
        message : str
            訊息內容。建議格式：「現象；建議操作」。
        severity : {"info", "ok", "warning", "critical"}
            語意色，由 QSS 的 msgSeverity property 自動套用。
        timeout_ms : int
            自動清除毫秒數；0 表示常駐。
        """
        self._status_label.setText(message)
        self._status_label.setProperty("msgSeverity", severity)
        repolish(self._status_label)
        if timeout_ms > 0:
            QTimer.singleShot(timeout_ms, self._clear_status)

    def _clear_status(self) -> None:
        # (1-7) reuse show_status 避免重複狀態列設定樣板；timeout_ms=0 常駐不再排程
        self.show_status("SYSTEM READY  /  系統就緒", "info", timeout_ms=0)

    # ------------------------------------------------------------------
    # 事件
    # ------------------------------------------------------------------
    def _on_nav_selected(self, idx: int) -> None:
        """左側導航切換 → 中央 stack 切換。

        stack 第 0 項是 placeholder，由 add_nav_page() 加入的真實頁面從 index 1 起，
        所以 nav idx N → stack idx N+1。
        """
        target = idx + 1   # 跳過 placeholder
        if 0 <= target < self.central_stack.count():
            self.central_stack.setCurrentIndex(target)

    def _toggle_fullscreen(self) -> None:
        """F11 全螢幕切換。"""
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()


# ══════════════════════════════════════════════════════════════════════
#  Main Block — 獨立可執行的示範（展示標準軍規佈局）
# ══════════════════════════════════════════════════════════════════════
def _demo() -> int:
    """獨立啟動 demo，模擬遙測注入與告警觸發。"""
    import sys

    from PyQt6.QtWidgets import QApplication

    from ui.resources.tactical_theme import apply_tactical_theme

    app = QApplication(sys.argv)
    apply_tactical_theme(app)

    win = AeroPlanMainWindow()

    # 建立幾個示範頁面
    def _demo_page(title: str, color: str) -> QWidget:
        w = QWidget()
        w.setStyleSheet(f"background: {TacticalColors.BG_PRIMARY};")
        lay = QVBoxLayout(w)
        lay.setContentsMargins(20, 20, 20, 20)
        t = QLabel(title)
        t.setProperty("role", "section-title")
        t.setStyleSheet(f"color: {color};")
        lay.addWidget(t)
        lay.addStretch(1)
        return w

    win.add_nav_page("MISSION", _demo_page("MISSION PLANNER",
                                           TacticalColors.FG_EMPHASIS),
                     "任務規劃與航點編輯")
    win.add_nav_page("VTOL", _demo_page("VTOL OPERATIONS",
                                        TacticalColors.NEUTRAL),
                     "垂直起降控制")
    win.add_nav_page("SWARM", _demo_page("SWARM STRIKE",
                                         TacticalColors.FRIENDLY),
                     "蜂群打擊")
    win.add_nav_page("SITL", _demo_page("SITL SIMULATOR",
                                        TacticalColors.WARNING),
                     "軟體迴路模擬")

    # 示範遙測與狀態
    win.top_strip.set_mode("AUTO", TacticalColors.FRIENDLY)
    win.top_strip.set_gps("3D", 14)
    win.top_strip.set_link(82.0)
    win.telemetry.set_value("ROLL", 2.3, "nominal")
    win.telemetry.set_value("PITCH", -1.1, "nominal")
    win.telemetry.set_value("YAW", 142.8, "nominal")
    win.telemetry.set_value("ALT", 85.4, "nominal")
    win.telemetry.set_value("GSPD", 12.6, "nominal")
    win.telemetry.set_value("HDG", 142.0, "nominal")
    win.telemetry.set_value("BATT", 68.0, "nominal")
    win.telemetry.set_value("VOLT", 22.8, "nominal")
    win.telemetry.set_value("CURR", 18.4, "nominal")
    win.armcomm.set_armed(False)

    # 示範告警
    win.master_warning.push_alert(
        AlertLevel.CAUTION,
        code="BAT-LOW",
        message="UAV-2 電量 38% 低於警告閾值 40%",
        action="安排替換電池或安排返航；若任務可中止，請按 ABORT。",
    )
    win.master_warning.push_alert(
        AlertLevel.ADVISORY,
        code="WX-001",
        message="風速預報 10 分鐘後超過 8 m/s",
        action="評估是否提前結束任務或切至抗風模式。",
    )

    win.show_status(
        "NAV: ACTIVE   |   已載入示範任務，按 MISSION 分頁開始規劃",
        severity="info",
    )

    win.show()
    return app.exec()


if __name__ == "__main__":
    import sys
    sys.exit(_demo())


__all__ = [
    "AeroPlanMainWindow",
    "TopStatusStrip",
    "NavigationRail",
    "TelemetryReadoutGroup",
    "ArmCommGroup",
]
