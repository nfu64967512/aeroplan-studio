"""ui/widgets/alert_banner.py — Alert Banner Stack（視覺+聽覺一體）。

對應 ADOS Mission Control 的 alert banner pattern。
進入時依 severity 自動觸發對應 `AlertEngine` 音效。

- INFO  → 自動 5 秒消失，無音
- CAUTION → 自動 15 秒消失，ERROR 音
- WARNING → 自動 60 秒消失，LOW_BATTERY 音
- CRITICAL → **不自動消失**（需手動 ACK），FAILSAFE 音

最多堆疊 5 筆；超出後最舊的非 CRITICAL 自動讓位。
"""
from __future__ import annotations

from enum import Enum
from typing import Optional

from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QProgressBar,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from ui.audio import AlertEngine, AlertId
from ui.resources.aeroplan_theme import tokens as T


class BannerSeverity(str, Enum):
    INFO = "info"
    CAUTION = "caution"
    WARNING = "warning"
    CRITICAL = "critical"


# 自動消失時間（毫秒）；CRITICAL = 0 表示不自動消失
_AUTO_DISMISS_MS = {
    BannerSeverity.INFO: 5_000,
    BannerSeverity.CAUTION: 15_000,
    BannerSeverity.WARNING: 60_000,
    BannerSeverity.CRITICAL: 0,
}

# severity → AlertId（None 表示不播音）
_AUDIO_MAP: dict[BannerSeverity, Optional[AlertId]] = {
    BannerSeverity.INFO: None,
    BannerSeverity.CAUTION: AlertId.ERROR,
    BannerSeverity.WARNING: AlertId.LOW_BATTERY,
    BannerSeverity.CRITICAL: AlertId.FAILSAFE,
}

# severity → 主色
_COLOR_MAP = {
    BannerSeverity.INFO: T.NEUTRAL,
    BannerSeverity.CAUTION: T.WARNING,
    BannerSeverity.WARNING: T.WARNING,
    BannerSeverity.CRITICAL: T.HOSTILE,
}


class AlertBanner(QFrame):
    """單筆 banner。"""

    dismissed = pyqtSignal(int)   # banner id

    def __init__(
        self,
        banner_id: int,
        severity: BannerSeverity,
        title: str,
        message: str = "",
        source: Optional[str] = None,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self._id = banner_id
        self._severity = severity
        self._auto_ms = _AUTO_DISMISS_MS[severity]
        self.setObjectName(f"AlertBanner_{banner_id}")
        self.setProperty("severity", severity.value)
        self.setProperty("role", "card")
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)

        color = _COLOR_MAP[severity]
        self.setStyleSheet(
            f"QFrame {{ background: {T.BG_SECONDARY}; "
            f"border: 1px solid {color}; border-left: 4px solid {color}; "
            f"border-radius: 6px; }}"
        )

        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 8, 8, 8)
        lay.setSpacing(4)

        head = QHBoxLayout()
        head.setContentsMargins(0, 0, 0, 0)
        head.setSpacing(6)

        sev_label = QLabel(severity.value.upper(), self)
        sev_label.setStyleSheet(
            f"font-family: {T.FONT_DISPLAY_STACK}; font-weight: 700; "
            f"font-size: 10px; color: {color}; letter-spacing: 1px; "
            f"background: transparent; border: none;"
        )
        title_label = QLabel(title, self)
        title_label.setStyleSheet(
            f"font-family: {T.FONT_SANS_STACK}; font-weight: 700; "
            f"font-size: 13px; color: {T.FG}; "
            f"background: transparent; border: none;"
        )
        src_label = QLabel(source or "", self)
        src_label.setStyleSheet(
            f"font-family: {T.FONT_MONO_STACK}; font-size: 10px; "
            f"color: {T.FG_MUTED}; background: transparent; border: none;"
        )

        self._close_btn = QPushButton("×", self)
        self._close_btn.setFixedSize(20, 20)
        self._close_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._close_btn.setStyleSheet(
            f"QPushButton {{ background: transparent; color: {T.FG_SECONDARY}; "
            f"border: none; font-size: 16px; font-weight: 700; }}"
            f"QPushButton:hover {{ color: {T.FG}; }}"
        )
        self._close_btn.clicked.connect(self._on_close)

        head.addWidget(sev_label)
        head.addWidget(title_label, 1)
        head.addWidget(src_label)
        head.addWidget(self._close_btn)
        lay.addLayout(head)

        if message:
            msg_label = QLabel(message, self)
            msg_label.setWordWrap(True)
            msg_label.setStyleSheet(
                f"font-family: {T.FONT_SANS_STACK}; font-size: 11px; "
                f"color: {T.FG_SECONDARY}; background: transparent; border: none;"
            )
            lay.addWidget(msg_label)

        # 倒數計時條（CRITICAL 不顯示）
        self._countdown: Optional[QProgressBar] = None
        if self._auto_ms > 0:
            self._countdown = QProgressBar(self)
            self._countdown.setRange(0, self._auto_ms)
            self._countdown.setValue(self._auto_ms)
            self._countdown.setTextVisible(False)
            self._countdown.setFixedHeight(2)
            self._countdown.setStyleSheet(
                f"QProgressBar {{ background: transparent; border: none; }}"
                f"QProgressBar::chunk {{ background: {color}; }}"
            )
            lay.addWidget(self._countdown)
            self._tick_timer = QTimer(self)
            self._tick_timer.setInterval(100)
            self._tick_timer.timeout.connect(self._tick)
            self._remaining_ms = self._auto_ms
            self._tick_timer.start()

        # 播音
        aid = _AUDIO_MAP[severity]
        if aid is not None:
            AlertEngine.instance().play(aid)

    @property
    def banner_id(self) -> int:
        return self._id

    @property
    def severity(self) -> BannerSeverity:
        return self._severity

    def _tick(self) -> None:
        self._remaining_ms = max(0, self._remaining_ms - 100)
        if self._countdown:
            self._countdown.setValue(self._remaining_ms)
        if self._remaining_ms <= 0:
            self._tick_timer.stop()
            self._on_close()

    def _on_close(self) -> None:
        if self._severity == BannerSeverity.CRITICAL:
            # CRITICAL ACK：停掉持續循環的 FAILSAFE 音
            AlertEngine.instance().stop_alert(AlertId.FAILSAFE)
        self.dismissed.emit(self._id)


class AlertBannerStack(QWidget):
    """Banner 容器；最多 5 筆，最舊的非 CRITICAL 先讓位。"""

    MAX_BANNERS = 5

    pushed = pyqtSignal(int)        # banner id
    cleared = pyqtSignal(int)       # banner id

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet("background: transparent;")

        self._lay = QVBoxLayout(self)
        self._lay.setContentsMargins(8, 4, 8, 4)
        self._lay.setSpacing(6)
        self._lay.addStretch(1)

        self._banners: list[AlertBanner] = []
        self._next_id = 1

    def push(
        self,
        severity: BannerSeverity,
        title: str,
        message: str = "",
        source: Optional[str] = None,
    ) -> int:
        """新增 banner；回傳 id。"""
        # 滿了 → 讓位
        if len(self._banners) >= self.MAX_BANNERS:
            self._evict_oldest_non_critical()

        bid = self._next_id
        self._next_id += 1
        banner = AlertBanner(bid, severity, title, message, source, self)
        banner.dismissed.connect(self._remove_banner)
        self._lay.insertWidget(self._lay.count() - 1, banner)
        self._banners.append(banner)
        self.pushed.emit(bid)
        return bid

    def dismiss(self, banner_id: int) -> None:
        for b in list(self._banners):
            if b.banner_id == banner_id:
                b._on_close()
                return

    def _evict_oldest_non_critical(self) -> None:
        for b in self._banners:
            if b.severity != BannerSeverity.CRITICAL:
                b._on_close()
                return
        # 全是 CRITICAL，移除最舊
        if self._banners:
            self._banners[0]._on_close()

    def _remove_banner(self, banner_id: int) -> None:
        for b in list(self._banners):
            if b.banner_id == banner_id:
                self._banners.remove(b)
                self._lay.removeWidget(b)
                b.deleteLater()
                self.cleared.emit(banner_id)
                return


__all__ = ["AlertBanner", "AlertBannerStack", "BannerSeverity"]
