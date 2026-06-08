"""
MasterWarningPanel — 主告警面板（MIL-STD-1472H §5.7.3.8）

依據美國國防部人機工程設計標準 MIL-STD-1472H 第 5.7.3.8 節：
    "Master Warning/Caution Panel shall be physically or logically
     independent of the general status display area, shall be
     continuously visible, and shall not be occludable by other
     panels, tabs, or dialogs."

設計重點
----------
1. **獨立性 (Independence)**
   此面板以 Qt 自有 Widget 呈現，放置於主視窗右上角，使用 QSplitter
   鎖定最小寬度，確保即便其他分頁切換也不會被遮蓋。

2. **三級嚴重度 (Severity Levels)** — 對應 MIL-STD-1472H §5.7.3.1
   - advisory  藍/青 — 建議性資訊，不影響任務
   - caution   黃   — 應注意，接近閾值
   - warning   紅   — 立即處置，危及任務或安全

3. **閃爍 (Flashing)** — 依 MIL-STD-411E
   當最高嚴重度為 warning 時，標題列以 2 Hz 方波閃爍，
   操作員確認 (ACK) 後轉為穩態。

4. **建設性訊息 (Constructive Message)** — §5.17.10.7.3
   每筆告警包含「現象」與「建議操作」兩部分，避免單純指責操作員。

使用範例
----------
>>> panel = MasterWarningPanel()
>>> panel.push_alert(AlertLevel.WARNING,
...                  code="BAT-001",
...                  message="UAV-3 電量 18% 低於臨界閾值",
...                  action="立即下達 RTL 指令返航，或切換至備援機")
>>> panel.acknowledge("BAT-001")

"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Optional

from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

# 由既有主題取得標準色與字型，避免色彩語意重複定義
try:
    from ui.resources.tactical_theme import TacticalColors, TacticalFonts, THRESHOLDS
except Exception:  # pragma: no cover — 主題模組載入失敗時提供保守 fallback
    class TacticalColors:  # type: ignore[no-redef]
        HOSTILE = "#FF003C"
        WARNING = "#FFB703"
        FRIENDLY = "#00E676"
        NEUTRAL = "#00B4D8"
        FG_MUTED = "#6B7A8C"

    class TacticalFonts:  # type: ignore[no-redef]
        _SANS_FALLBACK = ("Segoe UI", "Inter", "sans-serif")
        _CONDENSED_FALLBACK = ("Rajdhani", "Roboto Condensed",
                                "Segoe UI Semibold", "Arial Narrow", "sans-serif")
        _MONO_FALLBACK = ("JetBrains Mono", "Cascadia Mono", "Consolas",
                           "Courier New", "monospace")

        @staticmethod
        def _pick(stack, size):
            from PyQt6.QtGui import QFontDatabase
            families = QFontDatabase.families()
            for n in stack:
                if n in families:
                    return QFont(n, size)
            return QFont(stack[-1], size)

        @classmethod
        def condensed(cls, size: int = 10, bold: bool = True,
                      letter_spacing: float = 1.0) -> QFont:
            f = cls._pick(cls._CONDENSED_FALLBACK, size)
            f.setBold(bold)
            return f

        @classmethod
        def mono(cls, size: int = 10, bold: bool = False) -> QFont:
            f = cls._pick(cls._MONO_FALLBACK, size)
            f.setBold(bold)
            return f

    class _Thresh:
        blink_interval_ms = 250

    THRESHOLDS = _Thresh()  # type: ignore[assignment]


# ══════════════════════════════════════════════════════════════════════
#  Alert 資料模型
# ══════════════════════════════════════════════════════════════════════
class AlertLevel(Enum):
    """告警嚴重度。依 MIL-STD-1472H §5.7.3.1 分級。"""

    ADVISORY = "advisory"   # 資訊性（藍/青）
    CAUTION = "caution"     # 警告（黃）
    WARNING = "warning"     # 警報（紅，2 Hz 閃爍）


@dataclass
class Alert:
    """單筆告警記錄。

    Attributes
    ----------
    code : str
        告警代碼（如 'BAT-001', 'LINK-003'），作為唯一鍵。
    level : AlertLevel
        嚴重度。
    message : str
        現象描述（發生什麼事）。
    action : str
        建議操作（MIL-STD-1472H §5.17.10.7.3 — 必須具建設性）。
    timestamp : datetime
        產生時間（UTC）。
    acknowledged : bool
        操作員是否已按 ACK 確認。確認後仍顯示，但不再閃爍。
    """

    code: str
    level: AlertLevel
    message: str
    action: str = ""
    timestamp: datetime = field(default_factory=datetime.utcnow)
    acknowledged: bool = False

    def format_line(self) -> str:
        """格式化為列表顯示字串（等寬字型對齊）。"""
        ts = self.timestamp.strftime("%H:%M:%S")
        ack_flag = "[ACK]" if self.acknowledged else "     "
        return f"{ts}Z {ack_flag} {self.code:<10s} {self.message}"


# ══════════════════════════════════════════════════════════════════════
#  MasterWarningPanel
# ══════════════════════════════════════════════════════════════════════
class MasterWarningPanel(QWidget):
    """主告警面板 — MIL-STD-1472H §5.7.3.8 合規實作。

    Signals
    -------
    alert_acknowledged(str)
        操作員按 ACK 時發出，帶告警代碼。
    alert_cleared(str)
        告警清除（由上層程式呼叫 clear_alert）時發出。
    all_cleared()
        所有告警清空時發出。
    """

    alert_acknowledged = pyqtSignal(str)
    alert_cleared = pyqtSignal(str)
    all_cleared = pyqtSignal()

    # 物件名稱 — 對應 Global_MIL_STD.qss §11
    OBJECT_NAME = "MasterWarningPanel"

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        """建構面板。"""
        super().__init__(parent)
        self.setObjectName(self.OBJECT_NAME)

        # ── 內部狀態 ──────────────────────────────────────────
        self._alerts: dict[str, Alert] = {}  # code -> Alert
        self._blink_on: bool = False

        # ── UI 建立 ──────────────────────────────────────────
        self._build_ui()

        # ── 閃爍計時器（2 Hz = 250 ms，依 MIL-STD-411E） ──────
        self._blink_timer = QTimer(self)
        self._blink_timer.setInterval(THRESHOLDS.blink_interval_ms)
        self._blink_timer.timeout.connect(self._on_blink_tick)

        # ── 初始化樣式屬性 ──────────────────────────────────
        self._refresh_severity()

    # ------------------------------------------------------------------
    # UI 建置
    # ------------------------------------------------------------------
    def _build_ui(self) -> None:
        """建立面板視覺結構：標題 + 告警列表 + ACK 按鈕列。"""
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ── 標題列 ──────────────────────────────────────────
        self.title_label = QLabel("MASTER  CAUTION / WARNING")
        self.title_label.setObjectName("MasterTitle")
        self.title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.title_label.setFont(TacticalFonts.condensed(size=10, bold=True,
                                                         letter_spacing=2.0))
        root.addWidget(self.title_label)

        # ── 告警列表 ────────────────────────────────────────
        self.alert_list = QListWidget(self)
        self.alert_list.setFont(TacticalFonts.mono(size=9))
        self.alert_list.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.alert_list.setSelectionMode(
            QListWidget.SelectionMode.SingleSelection
        )
        self.alert_list.itemDoubleClicked.connect(self._on_item_double_clicked)
        root.addWidget(self.alert_list, 1)

        # ── 操作列 ─────────────────────────────────────────
        btn_bar = QHBoxLayout()
        btn_bar.setContentsMargins(4, 4, 4, 4)
        btn_bar.setSpacing(4)

        self.ack_btn = QPushButton("ACK")
        self.ack_btn.setToolTip(
            "確認選取的告警（停止閃爍，保留於列表）\n"
            "Acknowledge selected alert (stop flashing, keep on list)"
        )
        self.ack_btn.setProperty("tacticalRole", "caution")
        self.ack_btn.clicked.connect(self._on_ack_clicked)

        self.ack_all_btn = QPushButton("ACK ALL")
        self.ack_all_btn.setToolTip("確認所有告警")
        self.ack_all_btn.setProperty("tacticalRole", "caution")
        self.ack_all_btn.clicked.connect(self.acknowledge_all)

        self.clear_btn = QPushButton("CLEAR")
        self.clear_btn.setToolTip(
            "從列表移除選取告警（僅當告警條件已解除時）\n"
            "Remove selected alert from list (only after condition cleared)"
        )
        self.clear_btn.setProperty("tacticalRole", "execute")
        self.clear_btn.clicked.connect(self._on_clear_clicked)

        btn_bar.addWidget(self.ack_btn)
        btn_bar.addWidget(self.ack_all_btn)
        btn_bar.addStretch(1)
        btn_bar.addWidget(self.clear_btn)

        btn_frame = QFrame()
        btn_frame.setLayout(btn_bar)
        root.addWidget(btn_frame)

    # ------------------------------------------------------------------
    # 公開 API
    # ------------------------------------------------------------------
    def push_alert(
        self,
        level: AlertLevel,
        code: str,
        message: str,
        action: str = "",
    ) -> None:
        """新增或更新一筆告警。

        若 code 已存在，則更新訊息與嚴重度並清除 ACK 狀態；否則新增。

        Parameters
        ----------
        level : AlertLevel
            告警嚴重度。
        code : str
            告警唯一代碼（如 'BAT-001'）。
        message : str
            現象描述。
        action : str, optional
            建設性建議操作（MIL-STD-1472H §5.17.10.7.3）。
        """
        alert = Alert(code=code, level=level, message=message, action=action)
        self._alerts[code] = alert
        self._rebuild_list()
        self._refresh_severity()

    def acknowledge(self, code: str) -> None:
        """標記單筆告警為已確認。"""
        a = self._alerts.get(code)
        if a is None or a.acknowledged:
            return
        a.acknowledged = True
        self.alert_acknowledged.emit(code)
        self._rebuild_list()
        self._refresh_severity()

    def acknowledge_all(self) -> None:
        """確認所有未 ACK 的告警。"""
        changed = False
        for a in self._alerts.values():
            if not a.acknowledged:
                a.acknowledged = True
                self.alert_acknowledged.emit(a.code)
                changed = True
        if changed:
            self._rebuild_list()
            self._refresh_severity()

    def clear_alert(self, code: str) -> None:
        """從列表移除一筆告警（告警條件已解除時呼叫）。"""
        if code in self._alerts:
            del self._alerts[code]
            self.alert_cleared.emit(code)
            self._rebuild_list()
            self._refresh_severity()
            if not self._alerts:
                self.all_cleared.emit()

    def clear_all(self) -> None:
        """清空全部告警。"""
        if not self._alerts:
            return
        for code in list(self._alerts):
            self.alert_cleared.emit(code)
        self._alerts.clear()
        self._rebuild_list()
        self._refresh_severity()
        self.all_cleared.emit()

    def highest_level(self) -> Optional[AlertLevel]:
        """回傳目前最高嚴重度；無告警時為 None。"""
        if not self._alerts:
            return None
        rank = {AlertLevel.ADVISORY: 0,
                AlertLevel.CAUTION: 1,
                AlertLevel.WARNING: 2}
        return max(self._alerts.values(), key=lambda a: rank[a.level]).level

    def active_alerts(self) -> list[Alert]:
        """回傳目前所有告警（含已 ACK 者）。"""
        return list(self._alerts.values())

    # ------------------------------------------------------------------
    # 內部：列表重建 / 嚴重度刷新 / 閃爍
    # ------------------------------------------------------------------
    def _rebuild_list(self) -> None:
        """依當前告警重繪 QListWidget。"""
        # 排序：未 ACK 優先，再依嚴重度遞減，最後依時間遞減
        rank = {AlertLevel.WARNING: 3, AlertLevel.CAUTION: 2,
                AlertLevel.ADVISORY: 1}
        sorted_alerts = sorted(
            self._alerts.values(),
            key=lambda a: (a.acknowledged, -rank[a.level], -a.timestamp.timestamp()),
        )

        self.alert_list.clear()
        for a in sorted_alerts:
            item = QListWidgetItem(a.format_line())
            # 供 QSS 依嚴重度上色
            item.setData(Qt.ItemDataRole.UserRole, a.code)
            item.setData(Qt.ItemDataRole.ToolTipRole,
                         self._format_tooltip(a))
            # QSS 透過 property 選擇器無法直接套用到 QListWidgetItem；
            # 以前景色直接設置，避免樣式失效
            color = {
                AlertLevel.WARNING: TacticalColors.HOSTILE,
                AlertLevel.CAUTION: TacticalColors.WARNING,
                AlertLevel.ADVISORY: TacticalColors.NEUTRAL,
            }[a.level]
            if a.acknowledged:
                color = TacticalColors.FG_MUTED
            from PyQt6.QtGui import QBrush, QColor
            item.setForeground(QBrush(QColor(color)))
            self.alert_list.addItem(item)

    @staticmethod
    def _format_tooltip(a: Alert) -> str:
        """產生 Tooltip：現象 + 建議操作（建設性語氣）。"""
        ack = "（已確認）" if a.acknowledged else "（未確認）"
        lvl_zh = {AlertLevel.WARNING: "警報",
                  AlertLevel.CAUTION: "警告",
                  AlertLevel.ADVISORY: "資訊"}[a.level]
        parts = [
            f"[{lvl_zh} {a.code}] {ack}",
            f"現象：{a.message}",
        ]
        if a.action:
            parts.append(f"建議操作：{a.action}")
        parts.append(f"時間：{a.timestamp.strftime('%Y-%m-%d %H:%M:%S')} UTC")
        return "\n".join(parts)

    def _refresh_severity(self) -> None:
        """依當前最高嚴重度刷新 property、觸發 QSS 重繪、啟停閃爍。"""
        highest = self.highest_level()
        # 僅「存在未 ACK 的 warning」才閃爍
        has_unacked_warning = any(
            (a.level == AlertLevel.WARNING and not a.acknowledged)
            for a in self._alerts.values()
        )

        if highest is None:
            severity = "nominal"
        elif highest == AlertLevel.WARNING:
            severity = "warning"
        elif highest == AlertLevel.CAUTION:
            severity = "caution"
        else:
            severity = "nominal"

        self.setProperty("severity", severity)
        # 強制重新 polish 以套用 QSS 屬性選擇器
        self.style().unpolish(self)
        self.style().polish(self)
        self.title_label.style().unpolish(self.title_label)
        self.title_label.style().polish(self.title_label)
        self.update()

        # 閃爍控制
        if has_unacked_warning:
            if not self._blink_timer.isActive():
                self._blink_timer.start()
        else:
            if self._blink_timer.isActive():
                self._blink_timer.stop()
            self._blink_on = False
            self.title_label.setVisible(True)

    def _on_blink_tick(self) -> None:
        """閃爍回呼：切換標題可見性。"""
        self._blink_on = not self._blink_on
        self.title_label.setVisible(not self._blink_on)

    # ------------------------------------------------------------------
    # 按鈕事件
    # ------------------------------------------------------------------
    def _on_ack_clicked(self) -> None:
        """ACK 按鈕：確認目前選取的告警。"""
        item = self.alert_list.currentItem()
        if item is None:
            return
        code = item.data(Qt.ItemDataRole.UserRole)
        if isinstance(code, str):
            self.acknowledge(code)

    def _on_clear_clicked(self) -> None:
        """CLEAR 按鈕：移除已選取告警。"""
        item = self.alert_list.currentItem()
        if item is None:
            return
        code = item.data(Qt.ItemDataRole.UserRole)
        if isinstance(code, str):
            self.clear_alert(code)

    def _on_item_double_clicked(self, item: QListWidgetItem) -> None:
        """雙擊列表項：等同 ACK。"""
        code = item.data(Qt.ItemDataRole.UserRole)
        if isinstance(code, str):
            self.acknowledge(code)


__all__ = ["MasterWarningPanel", "Alert", "AlertLevel"]
