"""
TacticalUAVCard — 戰術 UAV 資產狀態卡

設計參考：MIL-STD-1472H（人機工程）、STANAG APP-6（戰術符號語意）、
         Stephen Few《Information Dashboard Design》之 Bullet Graph。

核心理念：
    * 捨棄圓形儀表（Gauge）— 資料墨水比低、佔空間
    * 改用線性 Bullet Graph — 一格 30-40px 高即可表達 current + target + zones
    * 閃爍採 QTimer 方波（2Hz）— 人眼敏感頻段、零動畫延遲
    * 閾值判定集中於 TacticalThresholds（單一事實來源）

欄位：
    Callsign | Status | Battery | Link | Armed | Mission Tag

使用範例：
    card = TacticalUAVCard()
    card.set_telemetry(UAVTelemetry(
        callsign="HAWK-01",
        status=UAVStatus.ENROUTE,
        battery_pct=72.5,
        link_pct=88.0,
        armed=True,
        mission_tag="STRIKE-03",
    ))
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from PyQt6.QtCore import QRectF, QSize, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import (
    QBrush, QColor, QFont, QFontMetrics, QPainter, QPaintEvent, QPen,
)
from PyQt6.QtWidgets import QFrame, QSizePolicy, QWidget

from ui.resources.tactical_theme import (
    THRESHOLDS, TacticalColors, TacticalFonts,
)


# ══════════════════════════════════════════════════════════════════════
#  Status Enum + Telemetry Dataclass
# ══════════════════════════════════════════════════════════════════════
class UAVStatus(str, Enum):
    """UAV 任務狀態。對應顯示色與是否觸發警示。"""
    IDLE     = "IDLE"      # 待機
    READY    = "READY"     # 就緒
    ENROUTE  = "ENROUTE"   # 前往目標
    ENGAGED  = "ENGAGED"   # 交戰中
    RTB      = "RTB"       # 返航 (Return To Base)
    LOST     = "LOST"      # 失聯
    DOWN     = "DOWN"      # 墜毀／失效


_STATUS_COLOR: dict[UAVStatus, str] = {
    UAVStatus.IDLE:    TacticalColors.FG_MUTED,
    UAVStatus.READY:   TacticalColors.NEUTRAL,
    UAVStatus.ENROUTE: TacticalColors.FRIENDLY,
    UAVStatus.ENGAGED: TacticalColors.FG_EMPHASIS,  # 琥珀 — 關注但非告警
    UAVStatus.RTB:     TacticalColors.WARNING,
    UAVStatus.LOST:    TacticalColors.HOSTILE,
    UAVStatus.DOWN:    TacticalColors.HOSTILE,
}


@dataclass
class UAVTelemetry:
    """UAV 即時遙測資料。"""
    callsign: str = "UAV-XX"
    status: UAVStatus = UAVStatus.IDLE
    battery_pct: float = 100.0      # 0-100
    link_pct: float = 100.0         # 0-100
    armed: bool = False
    mission_tag: str = ""           # e.g. "STRIKE-03" 可選
    # 可擴充欄位（保留）
    extras: dict[str, str] = field(default_factory=dict)


# ══════════════════════════════════════════════════════════════════════
#  BulletGraph — 線性水平條（QPainter 自繪）
# ══════════════════════════════════════════════════════════════════════
class BulletGraph(QWidget):
    """極簡 Bullet Graph：
    [背景軌跡]  [實心填充]  [閾值分界刻度]

    - 填充色依 value 與閾值自動切換 (FRIENDLY/WARNING/HOSTILE)
    - 支援 blink_on 狀態由外部 card 統一控制，避免各 bar 各自起 QTimer
    """

    BAR_HEIGHT = 8  # px — 極薄條帶，節省垂直空間

    def __init__(
        self,
        label: str,
        warning_threshold: float,
        critical_threshold: float,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._label = label
        self._warn = warning_threshold
        self._crit = critical_threshold
        self._value: float = 100.0
        self._blink_on: bool = False  # 由外部切換
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setMinimumHeight(22)
        self.setMaximumHeight(22)

    # ─── public API ─────────────────────────────────────
    def set_value(self, value: float) -> None:
        clamped = max(0.0, min(100.0, float(value)))
        if clamped != self._value:
            self._value = clamped
            self.update()

    def set_blink(self, on: bool) -> None:
        if on != self._blink_on:
            self._blink_on = on
            self.update()

    def is_critical(self) -> bool:
        return self._value < self._crit

    def is_warning(self) -> bool:
        return self._crit <= self._value < self._warn

    # ─── drawing ─────────────────────────────────────
    def paintEvent(self, _evt: QPaintEvent) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, False)  # 直角不抗鋸齒 → 更銳利

        w = self.width()
        h = self.height()

        # --- Label (左側 3 字元) + 百分比 (右側) 皆為等寬 Mono ---
        font_label = TacticalFonts.mono(8, bold=True)
        p.setFont(font_label)
        fm = QFontMetrics(font_label)
        label_w = fm.horizontalAdvance("XXX") + 4      # 預留 3 字元寬度
        pct_text = f"{self._value:5.1f}%"
        pct_w = fm.horizontalAdvance(pct_text) + 4

        # 文字色：告警時切換
        fill_color = self._resolve_fill_color()
        text_color = fill_color if self.is_critical() or self.is_warning() \
                     else QColor(TacticalColors.FG_SECONDARY)
        p.setPen(QPen(text_color))
        p.drawText(0, 0, label_w, h,
                   Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                   self._label)

        p.setPen(QPen(QColor(TacticalColors.FG_PRIMARY)))
        p.drawText(w - pct_w, 0, pct_w, h,
                   Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                   pct_text)

        # --- Bar 範圍 ---
        bar_x = label_w
        bar_w = w - label_w - pct_w
        bar_y = (h - self.BAR_HEIGHT) // 2
        bar_rect = QRectF(bar_x, bar_y, bar_w, self.BAR_HEIGHT)

        # 背景軌跡（sunken + 邊框）
        p.setPen(QPen(QColor(TacticalColors.BORDER_DEFAULT), 1))
        p.setBrush(QBrush(QColor(TacticalColors.BG_SUNKEN)))
        p.drawRect(bar_rect)

        # 填充（依 value）— critical 閃爍時半幀消失
        if self.is_critical() and not self._blink_on:
            pass  # 閃爍：此幀不畫填充 → 製造閃動
        else:
            fill_w = (bar_w - 2) * (self._value / 100.0)
            if fill_w > 0:
                p.setPen(Qt.PenStyle.NoPen)
                p.setBrush(QBrush(fill_color))
                p.drawRect(QRectF(bar_x + 1, bar_y + 1, fill_w, self.BAR_HEIGHT - 2))

        # 閾值刻度線（warning / critical）—— 極細白線標示
        tick_pen = QPen(QColor(TacticalColors.BORDER_STRONG), 1)
        p.setPen(tick_pen)
        for thr in (self._crit, self._warn):
            tx = bar_x + bar_w * (thr / 100.0)
            p.drawLine(int(tx), bar_y - 2, int(tx), bar_y + self.BAR_HEIGHT + 2)

    def _resolve_fill_color(self) -> QColor:
        if self._value < self._crit:
            return QColor(TacticalColors.HOSTILE)
        if self._value < self._warn:
            return QColor(TacticalColors.WARNING)
        return QColor(TacticalColors.FRIENDLY)


# ══════════════════════════════════════════════════════════════════════
#  TacticalUAVCard — UAV 資產狀態卡主元件
# ══════════════════════════════════════════════════════════════════════
class TacticalUAVCard(QFrame):
    """UAV 資產狀態卡（戰術 HUD 風格）。

    Signals:
        clicked(callsign): 點擊整張卡片時觸發，便於外部切換選中態
    """

    CARD_WIDTH = 260
    CARD_HEIGHT = 112
    CHAMFER = 10  # 四角瞄準框 L 型線段長度（px）

    clicked = pyqtSignal(str)  # callsign

    def __init__(self, telemetry: UAVTelemetry | None = None,
                 parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._telemetry = telemetry or UAVTelemetry()
        self._selected = False
        self._blink_on = False
        self._battery_bar = BulletGraph(
            "BAT",
            warning_threshold=THRESHOLDS.battery_low,
            critical_threshold=THRESHOLDS.battery_critical,
            parent=self,
        )
        self._link_bar = BulletGraph(
            "LNK",
            warning_threshold=THRESHOLDS.link_low,
            critical_threshold=THRESHOLDS.link_critical,
            parent=self,
        )

        self._blink_timer = QTimer(self)
        self._blink_timer.setInterval(THRESHOLDS.blink_interval_ms)
        self._blink_timer.timeout.connect(self._on_blink_tick)

        self._init_ui()
        self._apply_telemetry()

    # ─── UI init ──────────────────────────────────────
    def _init_ui(self) -> None:
        self.setFixedSize(self.CARD_WIDTH, self.CARD_HEIGHT)
        self.setFrameShape(QFrame.Shape.NoFrame)  # 邊框自繪
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setProperty("selected", "false")
        self.setProperty("alarm", "false")

        # Bullet Graphs 於 paintEvent 中以幾何定位繪製 → 不走 layout
        # 這裡只預先 resize bars 到卡片寬度
        pad = 10
        bar_x = pad
        bar_w = self.CARD_WIDTH - 2 * pad
        self._battery_bar.setGeometry(bar_x, 58, bar_w, 22)
        self._link_bar.setGeometry(bar_x,    82, bar_w, 22)

    # ─── public API ────────────────────────────────────
    def set_telemetry(self, telemetry: UAVTelemetry) -> None:
        self._telemetry = telemetry
        self._apply_telemetry()

    def telemetry(self) -> UAVTelemetry:
        return self._telemetry

    def set_selected(self, selected: bool) -> None:
        if selected == self._selected:
            return
        self._selected = selected
        self.setProperty("selected", "true" if selected else "false")
        self.style().unpolish(self)
        self.style().polish(self)
        self.update()

    # ─── internal ─────────────────────────────────────
    def _apply_telemetry(self) -> None:
        self._battery_bar.set_value(self._telemetry.battery_pct)
        self._link_bar.set_value(self._telemetry.link_pct)

        # 更新閃爍計時器狀態
        needs_blink = (
            self._telemetry.status in (UAVStatus.LOST, UAVStatus.DOWN)
            or self._battery_bar.is_critical()
            or self._link_bar.is_critical()
        )
        self.setProperty("alarm", "true" if needs_blink else "false")
        self.style().unpolish(self)
        self.style().polish(self)

        if needs_blink and not self._blink_timer.isActive():
            self._blink_timer.start()
        elif not needs_blink and self._blink_timer.isActive():
            self._blink_timer.stop()
            self._blink_on = False
            self._battery_bar.set_blink(False)
            self._link_bar.set_blink(False)
        self.update()

    def _on_blink_tick(self) -> None:
        self._blink_on = not self._blink_on
        # 同步到 bars（僅對 critical 的 bar 有效）
        self._battery_bar.set_blink(self._blink_on)
        self._link_bar.set_blink(self._blink_on)
        self.update()

    # ─── sizing ───────────────────────────────────────
    def sizeHint(self) -> QSize:
        return QSize(self.CARD_WIDTH, self.CARD_HEIGHT)

    # ─── input ────────────────────────────────────────
    def mousePressEvent(self, event) -> None:  # noqa: ANN001
        if event.button() == Qt.MouseButton.LeftButton:
            self.clicked.emit(self._telemetry.callsign)
        super().mousePressEvent(event)

    # ─── drawing ──────────────────────────────────────
    def paintEvent(self, evt: QPaintEvent) -> None:
        super().paintEvent(evt)   # 讓 QSS 先畫背景 + 邊框
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, False)

        w = self.width()
        h = self.height()

        # ── (1) 四角瞄準框（Chamfer L-corners）— HUD 經典裝飾 ──
        self._draw_crosshair_corners(p, w, h)

        # ── (2) 頂部狀態條 ──
        self._draw_header(p, w)

        # ── (3) Armed / Mission Tag ──
        self._draw_footer(p, w, h)

    def _draw_crosshair_corners(self, p: QPainter, w: int, h: int) -> None:
        """在四角繪製 L 型短線，模仿瞄準框。告警時改為紅色閃爍。"""
        in_alarm = self._telemetry.status in (UAVStatus.LOST, UAVStatus.DOWN) \
                   or self._battery_bar.is_critical() \
                   or self._link_bar.is_critical()

        if in_alarm:
            color = QColor(TacticalColors.HOSTILE) if self._blink_on \
                    else QColor(TacticalColors.BG_SECONDARY)
        elif self._selected:
            color = QColor(TacticalColors.NEUTRAL)
        else:
            color = QColor(TacticalColors.BORDER_STRONG)

        pen = QPen(color, 2)
        pen.setCapStyle(Qt.PenCapStyle.FlatCap)
        p.setPen(pen)
        c = self.CHAMFER
        # 左上
        p.drawLine(0, 0, c, 0)
        p.drawLine(0, 0, 0, c)
        # 右上
        p.drawLine(w - c, 0, w, 0)
        p.drawLine(w - 1, 0, w - 1, c)
        # 左下
        p.drawLine(0, h - 1, c, h - 1)
        p.drawLine(0, h - c, 0, h - 1)
        # 右下
        p.drawLine(w - c, h - 1, w - 1, h - 1)
        p.drawLine(w - 1, h - c, w - 1, h - 1)

    def _draw_header(self, p: QPainter, w: int) -> None:
        """頂部：[STATUS 標籤]  CALLSIGN"""
        t = self._telemetry
        pad = 10
        y = 6

        # Status tag box（左）
        status_font = TacticalFonts.condensed(8, bold=True, letter_spacing=1.2)
        p.setFont(status_font)
        fm = QFontMetrics(status_font)
        tag_text = t.status.value
        tag_w = fm.horizontalAdvance(tag_text) + 12
        tag_h = fm.height() + 4
        tag_rect = QRectF(pad, y, tag_w, tag_h)

        status_color = QColor(_STATUS_COLOR[t.status])
        # LOST / DOWN 時底色做閃爍
        if t.status in (UAVStatus.LOST, UAVStatus.DOWN) and self._blink_on:
            p.setBrush(QBrush(status_color))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawRect(tag_rect)
            p.setPen(QPen(QColor(TacticalColors.BG_PRIMARY)))
        else:
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.setPen(QPen(status_color, 1))
            p.drawRect(tag_rect)
            p.setPen(QPen(status_color))
        p.drawText(tag_rect, Qt.AlignmentFlag.AlignCenter, tag_text)

        # Callsign（右）— 等寬大字
        call_font = TacticalFonts.mono(13, bold=True)
        p.setFont(call_font)
        p.setPen(QPen(QColor(TacticalColors.FG_PRIMARY)))
        call_rect = QRectF(tag_w + pad + 8, y - 1, w - tag_w - pad * 2 - 8, tag_h + 2)
        p.drawText(call_rect,
                   Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                   t.callsign)

        # 分隔線（頂部條下緣）
        sep_y = y + tag_h + 4
        p.setPen(QPen(QColor(TacticalColors.BORDER_SUBTLE), 1))
        p.drawLine(pad, sep_y, w - pad, sep_y)

    def _draw_footer(self, p: QPainter, w: int, h: int) -> None:
        """底部：[■ ARMED]  MSN: STRIKE-03"""
        t = self._telemetry
        pad = 10
        y = h - 18

        # Armed 指示
        armed_font = TacticalFonts.condensed(8, bold=True, letter_spacing=1.0)
        p.setFont(armed_font)
        fm = QFontMetrics(armed_font)

        if t.armed:
            armed_color = QColor(TacticalColors.HOSTILE)
            armed_text = "● ARMED"
        else:
            armed_color = QColor(TacticalColors.FG_MUTED)
            armed_text = "○ SAFE"

        p.setPen(QPen(armed_color))
        p.drawText(pad, y, fm.horizontalAdvance(armed_text) + 2,
                   fm.height() + 2,
                   Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                   armed_text)

        # Mission Tag（右）
        if t.mission_tag:
            msn_font = TacticalFonts.mono(8, bold=False)
            p.setFont(msn_font)
            p.setPen(QPen(QColor(TacticalColors.FG_SECONDARY)))
            msn_text = f"MSN · {t.mission_tag}"
            mfm = QFontMetrics(msn_font)
            msn_w = mfm.horizontalAdvance(msn_text) + 2
            p.drawText(w - pad - msn_w, y, msn_w, fm.height() + 2,
                       Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                       msn_text)


# ══════════════════════════════════════════════════════════════════════
#  Demo — 直接執行本模組觀看效果
#  $ python -m ui.widgets.tactical_uav_card
# ══════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    import sys
    from PyQt6.QtWidgets import QApplication, QGridLayout, QWidget

    from ui.resources.tactical_theme import apply_tactical_theme

    app = QApplication(sys.argv)
    apply_tactical_theme(app)

    root = QWidget()
    root.setWindowTitle("AeroPlan Studio — Tactical UAV Asset Cards")
    root.resize(880, 360)
    grid = QGridLayout(root)
    grid.setContentsMargins(16, 16, 16, 16)
    grid.setSpacing(10)

    demo_feed: list[UAVTelemetry] = [
        UAVTelemetry("HAWK-01", UAVStatus.ENROUTE, 82.0, 94.0, True,  "STRIKE-03"),
        UAVTelemetry("HAWK-02", UAVStatus.ENGAGED, 58.0, 71.0, True,  "STRIKE-03"),
        UAVTelemetry("HAWK-03", UAVStatus.READY,   99.0, 100.0, False,""),
        UAVTelemetry("HAWK-04", UAVStatus.RTB,     31.0, 82.0, False, "RTB-A"),
        UAVTelemetry("HAWK-05", UAVStatus.LOST,    12.0, 18.0, True,  "STRIKE-01"),
        UAVTelemetry("HAWK-06", UAVStatus.IDLE,    88.0, 90.0, False, ""),
        UAVTelemetry("HAWK-07", UAVStatus.DOWN,     0.0,  0.0, False, "—"),
        UAVTelemetry("HAWK-08", UAVStatus.ENROUTE, 45.0, 55.0, True,  "RECON-2"),
    ]

    for idx, tlm in enumerate(demo_feed):
        card = TacticalUAVCard(tlm)
        card.clicked.connect(lambda cs: print(f"[demo] selected {cs}"))
        grid.addWidget(card, idx // 3, idx % 3)

    root.show()
    sys.exit(app.exec())
