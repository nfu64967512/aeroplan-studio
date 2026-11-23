"""
SITL HUD 面板 — Mission Planner 風格的即時遙測顯示
顯示：飛行模式、武裝狀態、姿態、速度、電量、GPS、飛行器類型
含連線/斷線控制與連線字串輸入。

Mission Planner-style Head-Up Display：依 MIL-STD-1472H 5.12.6.5.1
（顯示介面對比與字型）原則設計：
  - 上方羅盤 tape（航向）
  - 左側空速 tape / 右側高度 tape（中央當前值用 boxed 高亮）
  - 中央 pitch ladder + 人工地平
  - 底部聚合遙測（GS / VS / BAT / GPS / ALT）
  - 全 sans-serif 高對比配色（白/黃/品紅）以利強光與夜間任務
"""

import html as _html
import math
import time as _time
from typing import Optional
from PyQt6.QtCore import (Qt, pyqtSignal, pyqtSlot, QRectF, QPointF, QSize, QTimer, QRect, QPoint,
                          QMetaObject)
from PyQt6.QtGui import QPainter, QColor, QPen, QPolygonF, QFontMetrics
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QLineEdit, QFrame, QComboBox, QScrollArea, QSizePolicy, QSpinBox,
    QProgressBar, QCheckBox, QButtonGroup, QPlainTextEdit, QLayout,
)

# 群控／單機兩種模式的訊息緩衝（每則帶 sysid；純 Python）
from ui.widgets.sitl_messages import SitlMessageLog

# MIL-STD-1472H 戰術色彩 / 字型常數（單一事實來源 — QPainter / 動態樣式仍需）
from ui.resources.tactical_theme import TacticalColors as TC, TacticalFonts as TF, THRESHOLDS

# AeroPlan Studio 全域設計系統：emoji 按鈕替換為 SVG IconButton
from ui.resources.aeroplan_theme.widgets import IconButton
from ui.resources.aeroplan_theme.icons import get_icon

# ── ADI（人工地平儀）物理色 ──────────────────────────────────────
# 這兩色為飛行儀表「物理符號」，非裝飾色，故不放在 TacticalColors。
# 取自 MIL-STD-1787E ADI 慣用低飽和、護暗視覺色階。
_ADI_SKY    = '#1E3A5F'   # 暗藍灰天空
_ADI_GROUND = '#3D2914'   # 暗棕大地


class _FlowLayout(QLayout):
    """會換行的橫排（單機「機號」chip 用）：6 台、三位數 sysid、再加未讀標記「UAV103 ●12」時一排放不下，
    不換行就把整頁撐出水平捲軸（2026-10-22 審查實測 481 px）。高度依寬度算（heightForWidth）。"""

    def __init__(self, parent=None, spacing: int = 4):
        super().__init__(parent)
        self._items = []
        self._sp = int(spacing)
        self.setContentsMargins(0, 0, 0, 0)

    def addItem(self, item):                     # noqa: N802 — Qt 覆寫
        self._items.append(item)

    def count(self):
        return len(self._items)

    def itemAt(self, i):                         # noqa: N802
        return self._items[i] if 0 <= i < len(self._items) else None

    def takeAt(self, i):                         # noqa: N802
        return self._items.pop(i) if 0 <= i < len(self._items) else None

    def hasHeightForWidth(self):                 # noqa: N802
        return True

    def heightForWidth(self, w):                 # noqa: N802
        return self._place(QRect(0, 0, int(w), 0), apply=False)

    def setGeometry(self, rect):                 # noqa: N802
        super().setGeometry(rect)
        self._place(rect, apply=True)

    def sizeHint(self):                          # noqa: N802
        return self.minimumSize()

    def minimumSize(self):                       # noqa: N802
        s = QSize(0, 0)
        for it in self._items:
            s = s.expandedTo(it.minimumSize())
        m = self.contentsMargins()
        return s + QSize(m.left() + m.right(), m.top() + m.bottom())

    def _place(self, rect, apply: bool) -> int:
        m = self.contentsMargins()
        r = rect.adjusted(m.left(), m.top(), -m.right(), -m.bottom())
        x, y, line_h = r.x(), r.y(), 0
        for it in self._items:
            w = it.widget()
            if w is not None and w.isHidden():
                continue
            hint = it.sizeHint()
            if x > r.x() and x + hint.width() > r.right() + 1:
                x, y, line_h = r.x(), y + line_h + self._sp, 0
            if apply:
                it.setGeometry(QRect(QPoint(x, y), hint))
            x += hint.width() + self._sp
            line_h = max(line_h, hint.height())
        return y + line_h - rect.y() + m.bottom()


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
#  MissionPlannerHud — 完整 MIL-STD-1472H 風格 HUD（compass+tapes+horizon）
# ══════════════════════════════════════════════════════════════════════
class MissionPlannerHud(QWidget):
    """
    Mission Planner Head-Up Display 全景 HUD：

        ┌─── compass tape ──────────────┐
        │ 27 29 31  N  3  6  9          │
        ├──┬─────────────────────┬──────┤
        │SP│   pitch ladder      │ ALT  │
        │EE│   ─── ▽ ───         │      │
        │ED│   人工地平          │      │
        │  │                     │      │
        │  │  AUTO [ARMED] ✈     │      │
        ├──┴─────────────────────┴──────┤
        │ GS:120  VS:+3  BAT:85%  GPS:12│
        └───────────────────────────────┘

    色彩語意（依 MIL-STD-1472H §5.17.25.8 + TABLE XL）：
      - FG_PRIMARY (#E0E1DD) — 結構線、刻度、地平（中性）
      - FG_EMPHASIS (#FFB703 琥珀) — 重點數字、固定中央指標、boxed 當前值
      - HOSTILE  (#FF003C) — TABLE XL「Equipment: malfunction/critical」 + ARMED 框
      - WARNING  (#FFB703) — TABLE XL「Caution/abnormal」(BAT<50%)
      - FRIENDLY (#00E676) — TABLE XL「Equipment: normal/in-tolerance/ready」(BAT≥50%, GPS 3D)
      - NEUTRAL  (#00B4D8) — TABLE XL「Cyan: Advisory/friendly affiliation」(標籤)
      - BG_PRIMARY (#0A0F14) — 主背景（非純黑，保暗視覺）

    字型（依 §5.17.18.7 — sans-serif under adverse lighting）：
      - 數字 → MONO_STACK（JetBrains Mono → Cascadia → Consolas）等寬避免抖動
      - 標籤 → CONDENSED_STACK（Rajdhani → Roboto Condensed → Segoe UI Semibold）
      - 一般 → SANS_STACK（Segoe UI → Inter → Arial — 全部為 §5.17.18.7.1/.2 列名合規字型）

    更新頻率（依 §5.17.22.1.1）：
      - **精讀數值** (boxed 速度/高度/底部 GS/VS/BAT/GPS) → ≤ 1 Hz （§5.17.22.1.1.1）
      - **變化率指示** (人工地平、pitch ladder、compass tape) → 3-4 Hz （§5.17.22.1.1.2）

    其他規定：
      - 數字無前導零 (§5.17.18.10.6)
      - 全直角 border-radius:0 (戰術介面鐵則)
    """

    PITCH_FOV       = 50.0    # 中央顯示區域 ±50° 俯仰（決定 pitch ladder 縮放）
    SLOW_REFRESH_S  = 1.0     # 精讀數值最小刷新週期（§5.17.22.1.1.1）

    def __init__(self, parent=None):
        super().__init__(parent)
        # ── 即時狀態（每幀更新；繪製姿態球/ladder/compass 用）──
        self._roll  = 0.0
        self._pitch = 0.0
        self._yaw   = 0.0
        self._airspeed = 0.0
        self._altitude = 0.0
        self._gs       = 0.0
        self._vs       = 0.0
        self._throttle = 0
        self._mode  = '---'
        self._armed = False
        self._vehicle = ''
        self._bat_pct = -1
        self._bat_v   = 0.0
        self._gps_sats = 0
        self._gps_fix  = 0

        # ── 慢速顯示快照（≤1 Hz；§5.17.22.1.1.1 精讀數值上限）──
        # 這些快照是「使用者必須可靠讀取」的數值，不能跟著 SITL 4-10 Hz 跳動
        self._slow_t      = 0.0
        self._d_airspeed  = 0.0
        self._d_altitude  = 0.0
        self._d_gs        = 0.0
        self._d_vs        = 0.0
        self._d_throttle  = 0
        self._d_bat_pct   = -1
        self._d_bat_v     = 0.0
        self._d_gps_sats  = 0
        self._d_gps_fix   = 0
        # 模式/ARMED 雖也屬「精讀」，但屬離散狀態變更 — 變更時立即顯示
        self._d_mode      = '---'
        self._d_armed     = False
        self._d_vehicle   = ''

        self.setMinimumSize(260, 230)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    # ── 公開資料寫入 ─────────────────────────────────────────────
    def set_attitude(self, roll: float, pitch: float, yaw: float = 0.0):
        # 姿態屬「rate-of-change」(§5.17.22.1.1.2) → 全速更新
        self._roll, self._pitch, self._yaw = float(roll), float(pitch), float(yaw)
        self.update()

    def set_flight(self, airspeed: float, altitude: float,
                    gs: float = 0.0, vs: float = 0.0, throttle: int = 0):
        self._airspeed, self._altitude = float(airspeed), float(altitude)
        self._gs, self._vs, self._throttle = float(gs), float(vs), int(throttle)
        self._maybe_refresh_slow()
        self.update()

    def set_status(self, mode: str, armed: bool, vehicle: str = ''):
        self._mode, self._armed, self._vehicle = mode or '---', bool(armed), vehicle or ''
        # 模式/ARMED 為離散狀態，立即推到顯示快照（不受 1Hz 限制）
        if self._mode != self._d_mode or self._armed != self._d_armed \
                or self._vehicle != self._d_vehicle:
            self._d_mode, self._d_armed, self._d_vehicle = \
                self._mode, self._armed, self._vehicle
        self.update()

    def set_health(self, bat_pct: int, bat_v: float, gps_sats: int, gps_fix: int):
        self._bat_pct, self._bat_v = int(bat_pct), float(bat_v)
        self._gps_sats, self._gps_fix = int(gps_sats), int(gps_fix)
        self._maybe_refresh_slow()
        # GPS fix 等級變更為離散 → 立即同步避免使用者錯失告警
        if self._gps_fix != self._d_gps_fix and \
                ((self._d_gps_fix >= 3) != (self._gps_fix >= 3)):
            self._d_gps_fix = self._gps_fix
        self.update()

    def _maybe_refresh_slow(self):
        """
        精讀數值快照：每 SLOW_REFRESH_S (§5.17.22.1.1.1=1s) 才從即時值刷新一次，
        避免 4-10 Hz SITL 直接打到顯示讓使用者讀不清。
        """
        import time as _t
        now = _t.time()
        if (now - self._slow_t) < self.SLOW_REFRESH_S:
            return
        self._slow_t      = now
        self._d_airspeed  = self._airspeed
        self._d_altitude  = self._altitude
        self._d_gs        = self._gs
        self._d_vs        = self._vs
        self._d_throttle  = self._throttle
        self._d_bat_pct   = self._bat_pct
        self._d_bat_v     = self._bat_v
        self._d_gps_sats  = self._gps_sats
        self._d_gps_fix   = self._gps_fix

    def update_from_frame(self, frame):
        """一次性套用整個 TelemetryFrame（只排一次重繪：四個 set_* 各自的 update() 在這裡被壓掉）。"""
        self._in_batch = True
        try:
            self.set_attitude(frame.roll, frame.pitch, frame.heading)
            self.set_flight(getattr(frame, 'air_speed', frame.ground_speed),
                             frame.alt_rel, frame.ground_speed,
                             getattr(frame, 'climb', 0.0), frame.throttle)
            self.set_status(frame.mode, frame.armed, frame.vehicle_type)
            self.set_health(frame.battery_pct, frame.battery_v,
                             frame.gps_sats, frame.gps_fix)
            self._yaw = float(frame.heading)
            # invalidate_slow() 之後：等**整幀都寫進去**才刷新精讀快照（set_flight 裡那次刷新時
            # 電量／GPS 還是上一幀的 ⇒ 單機換台後會顯示上一台的電量，2026-10-22 截圖查到）
            if self.__dict__.pop('_slow_force', False):
                self._slow_t = 0.0
                self._maybe_refresh_slow()
        finally:
            self._in_batch = False
        self.update()

    def update(self, *args):                     # noqa: D401 — QWidget.update 的薄包裝
        """批次套用期間不排重繪（update_from_frame 最後會排一次）。"""
        if self.__dict__.get('_in_batch'):
            return None
        return super().update(*args)

    def invalidate_slow(self) -> None:
        """下一幀（update_from_frame）整幀寫完後立刻刷新精讀數值。

        換看另一台時用：不然最多 1 s 內顯示的還是上一台的高度／速度／電量／衛星數。
        """
        self._slow_t = 0.0
        self._slow_force = True

    # ── 主繪製 ─────────────────────────────────────────────────────
    def paintEvent(self, _evt):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        # 區塊配置
        TAPE_TOP    = 22                            # 頂部 compass tape 高度
        TAPE_SIDE   = 50                            # 兩側 tape 寬度
        BOTTOM_H    = 56                            # 底部遙測區
        horizon = QRectF(TAPE_SIDE, TAPE_TOP,
                          w - 2 * TAPE_SIDE,
                          h - TAPE_TOP - BOTTOM_H)

        # 背景：BG_PRIMARY（深藏青非純黑，保視桿細胞 — MIL-STD 1472H 5.8.4.4）
        p.fillRect(self.rect(), QColor(TC.BG_PRIMARY))

        # 1) 中央人工地平 + pitch ladder
        self._draw_horizon(p, horizon)
        # 2) 頂部 compass tape
        self._draw_compass(p, QRectF(0, 0, w, TAPE_TOP))
        # 3) 左側空速 tape
        self._draw_speed_tape(p, QRectF(0, TAPE_TOP, TAPE_SIDE, horizon.height()))
        # 4) 右側高度 tape
        self._draw_alt_tape(p, QRectF(w - TAPE_SIDE, TAPE_TOP, TAPE_SIDE, horizon.height()))
        # 5) 中央底部模式 + ARMED
        self._draw_mode_armed(p, horizon)
        # 6) 底部聚合遙測
        self._draw_telemetry(p, QRectF(0, h - BOTTOM_H, w, BOTTOM_H))
        # 7) 邊框（BORDER_DEFAULT，全直角）
        p.setPen(QPen(QColor(TC.BORDER_DEFAULT), 1))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRect(QRectF(0.5, 0.5, w - 1, h - 1))
        p.end()

    # ── 人工地平 ──────────────────────────────────────────────────
    def _draw_horizon(self, p: QPainter, rect: QRectF):
        cx = rect.center().x()
        cy = rect.center().y()
        p.save()
        p.setClipRect(rect)
        p.translate(cx, cy)
        p.rotate(-self._roll)
        # 1° pitch 對應像素：rect.height / FOV
        ppd = rect.height() / self.PITCH_FOV
        pitch_px = self._pitch * ppd

        # 天空 / 地面 — ADI 物理符號色（MIL-STD-1787E）
        p.fillRect(QRectF(-rect.width(), -rect.height() + pitch_px,
                            rect.width() * 2, rect.height()),
                    QColor(_ADI_SKY))
        p.fillRect(QRectF(-rect.width(), pitch_px,
                            rect.width() * 2, rect.height()),
                    QColor(_ADI_GROUND))
        # 地平線（FG_PRIMARY 主結構線）
        p.setPen(QPen(QColor(TC.FG_PRIMARY), 2))
        p.drawLine(int(-rect.width()), int(pitch_px),
                    int(rect.width()),  int(pitch_px))

        # Pitch ladder（每 10° 主刻度，每 5° 副刻度）
        ladder_font = TF.mono(8)
        p.setFont(ladder_font)
        for d in range(-30, 31, 5):
            if d == 0:
                continue
            y = pitch_px - d * ppd
            major = (d % 10 == 0)
            length = 28 if major else 14
            p.setPen(QPen(QColor(TC.FG_PRIMARY), 1.5 if major else 1.0))
            p.drawLine(int(-length), int(y), int(length), int(y))
            if major:
                txt = str(abs(d))
                p.setPen(QPen(QColor(TC.FG_PRIMARY)))
                fm = QFontMetrics(ladder_font)
                tw = fm.horizontalAdvance(txt)
                p.drawText(int(-length - tw - 4), int(y + 4), txt)
                p.drawText(int(length + 4), int(y + 4), txt)
        p.restore()

        # 中央飛機十字（FG_EMPHASIS 琥珀 — 重點視覺焦點）
        cx_i, cy_i = int(cx), int(cy)
        p.setPen(QPen(QColor(TC.FG_EMPHASIS), 2.5))
        p.drawLine(cx_i - 24, cy_i, cx_i - 8, cy_i)
        p.drawLine(cx_i + 8,  cy_i, cx_i + 24, cy_i)
        tri = QPolygonF([
            QPointF(cx_i - 7, cy_i),
            QPointF(cx_i + 7, cy_i),
            QPointF(cx_i,     cy_i + 6),
        ])
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawPolygon(tri)

        # Roll 指示三角（頂端，琥珀）
        p.save()
        p.translate(cx, rect.top() + 14)
        p.rotate(-self._roll)
        rtri = QPolygonF([
            QPointF(0, 0),
            QPointF(-5, 9),
            QPointF(5, 9),
        ])
        p.setBrush(QColor(TC.FG_EMPHASIS))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawPolygon(rtri)
        p.restore()

    # ── Compass tape（航向帶）────────────────────────────────────
    def _draw_compass(self, p: QPainter, rect: QRectF):
        p.fillRect(rect, QColor(TC.BG_SUNKEN))
        p.setPen(QPen(QColor(TC.BORDER_SUBTLE), 1))
        p.drawLine(int(rect.left()), int(rect.bottom()),
                    int(rect.right()), int(rect.bottom()))

        cx = rect.center().x()
        ppd = 4.0     # pixel per degree
        font = TF.condensed(9, bold=True, letter_spacing=0.5)
        p.setFont(font)
        fm = QFontMetrics(font)

        hdg = self._yaw % 360
        for d in range(int(hdg) - 60, int(hdg) + 61, 5):
            x = cx + (d - hdg) * ppd
            if x < rect.left() + 4 or x > rect.right() - 4:
                continue
            major = (d % 30 == 0)
            tlen = 7 if major else 3
            p.setPen(QPen(QColor(TC.FG_PRIMARY), 1.2 if major else 0.8))
            p.drawLine(int(x), int(rect.top() + 2),
                        int(x), int(rect.top() + 2 + tlen))
            if major:
                dn = d % 360
                label = {0: 'N', 90: 'E', 180: 'S', 270: 'W'}.get(dn, str(dn // 10))
                tw = fm.horizontalAdvance(label)
                p.setPen(QPen(QColor(TC.FG_PRIMARY)))
                p.drawText(int(x - tw / 2), int(rect.top() + 18), label)

        # 中央 ▼ 指針（FG_EMPHASIS 琥珀）
        ptr = QPolygonF([
            QPointF(cx,     rect.bottom() - 1),
            QPointF(cx - 5, rect.bottom() + 4),
            QPointF(cx + 5, rect.bottom() + 4),
        ])
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor(TC.FG_EMPHASIS))
        p.drawPolygon(ptr)

    # ── Speed tape (左側) ────────────────────────────────────────
    def _draw_speed_tape(self, p: QPainter, rect: QRectF):
        # boxed 當前值用「慢速顯示快照」(§5.17.22.1.1.1)，背景刻度跟即時值即可
        self._draw_vertical_tape(
            p, rect, value=self._d_airspeed, step=20, range_units=80,
            unit_label='m/s', side='left', min_value=0,
        )

    # ── Altitude tape (右側) ─────────────────────────────────────
    def _draw_alt_tape(self, p: QPainter, rect: QRectF):
        self._draw_vertical_tape(
            p, rect, value=self._d_altitude, step=20, range_units=120,
            unit_label='m', side='right',
        )

    def _draw_vertical_tape(self, p, rect, value, step, range_units,
                             unit_label, side, min_value=None):
        """
        通用垂直 tape。range_units 是 tape 上下總範圍（單位）。
        刻度為 NEUTRAL/FG_PRIMARY；boxed 當前值用 FG_EMPHASIS 琥珀。
        """
        p.fillRect(rect, QColor(TC.BG_SUNKEN))
        # 內側邊線
        p.setPen(QPen(QColor(TC.BORDER_SUBTLE), 1))
        if side == 'left':
            p.drawLine(int(rect.right()), int(rect.top()),
                        int(rect.right()), int(rect.bottom()))
        else:
            p.drawLine(int(rect.left()), int(rect.top()),
                        int(rect.left()), int(rect.bottom()))

        cy = rect.center().y()
        ppu = rect.height() / range_units
        font_tick = TF.mono(8)
        font_box  = TF.mono(12, bold=True)
        p.setFont(font_tick)
        fm = QFontMetrics(font_tick)

        v0 = math.floor(value / step) * step - range_units / 2
        v1 = v0 + range_units
        v = v0
        while v <= v1:
            if min_value is not None and v < min_value:
                v += step
                continue
            y = cy + (value - v) * ppu
            if y < rect.top() or y > rect.bottom():
                v += step
                continue
            major = (int(v) % step == 0)
            tlen = 6 if major else 3
            p.setPen(QPen(QColor(TC.FG_SECONDARY), 1.2 if major else 0.8))
            if side == 'left':
                p.drawLine(int(rect.right() - tlen), int(y),
                            int(rect.right()), int(y))
            else:
                p.drawLine(int(rect.left()), int(y),
                            int(rect.left() + tlen), int(y))
            if major:
                txt = str(int(v))
                tw = fm.horizontalAdvance(txt)
                p.setPen(QPen(QColor(TC.FG_PRIMARY)))
                if side == 'left':
                    p.drawText(int(rect.right() - tlen - tw - 4), int(y + 4), txt)
                else:
                    p.drawText(int(rect.left() + tlen + 4), int(y + 4), txt)
            v += step

        # 中央 boxed 當前值（FG_EMPHASIS 琥珀，全直角邊框）
        p.setFont(font_box)
        fmb = QFontMetrics(font_box)
        val_txt = f'{value:.0f}'
        bw = max(fmb.horizontalAdvance(val_txt) + 12, 38)
        bh = 22
        if side == 'left':
            box = QRectF(rect.right() - bw, cy - bh / 2, bw, bh)
            arrow_x = box.right()
        else:
            box = QRectF(rect.left(), cy - bh / 2, bw, bh)
            arrow_x = box.left()
        p.setPen(QPen(QColor(TC.FG_EMPHASIS), 1.5))
        p.setBrush(QColor(TC.BG_PRIMARY))
        p.drawRect(box)
        p.setPen(QPen(QColor(TC.FG_EMPHASIS)))
        p.drawText(box, Qt.AlignmentFlag.AlignCenter, val_txt)
        if side == 'left':
            arrow = QPolygonF([
                QPointF(arrow_x,     cy - 6),
                QPointF(arrow_x + 8, cy),
                QPointF(arrow_x,     cy + 6),
            ])
        else:
            arrow = QPolygonF([
                QPointF(arrow_x,     cy - 6),
                QPointF(arrow_x - 8, cy),
                QPointF(arrow_x,     cy + 6),
            ])
        p.setBrush(QColor(TC.FG_EMPHASIS))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawPolygon(arrow)

    # ── 模式 + ARMED 視覺中心置中 ───────────────────────────────
    def _draw_mode_armed(self, p: QPainter, horizon: QRectF):
        cx = horizon.center().x()
        y  = horizon.bottom() - 12
        font_mode = TF.condensed(11, bold=True, letter_spacing=1.2)
        font_arm  = TF.condensed(10, bold=True, letter_spacing=1.0)
        fm_mode = QFontMetrics(font_mode)
        fm_arm  = QFontMetrics(font_arm)

        # 模式/ARMED 用 _d_* 快照（離散變更時立即同步，§5.17.22.1.1 不限制離散事件）
        mode_txt = self._d_mode
        arm_txt  = 'ARMED' if self._d_armed else 'DISARMED'
        # ARMED 框色：HOSTILE（武裝即危險，符合語意） / 停用為灰
        arm_color = QColor(TC.FG_PRIMARY) if self._armed else QColor(TC.FG_MUTED)
        bg_color  = QColor(TC.HOSTILE)    if self._armed else QColor(TC.BG_ELEVATED)

        mode_w = fm_mode.horizontalAdvance(mode_txt)
        aw = fm_arm.horizontalAdvance(arm_txt) + 10
        ah = 18
        gap = 8
        total_w = mode_w + gap + aw
        start_x = cx - total_w / 2

        # 1) 模式文字（FG_EMPHASIS 琥珀重點）
        p.setFont(font_mode)
        p.setPen(QPen(QColor(TC.FG_EMPHASIS)))
        p.drawText(int(start_x), int(y), mode_txt)

        # 2) ARMED 框
        bx = start_x + mode_w + gap
        box = QRectF(bx, y - ah + 4, aw, ah)
        p.setBrush(bg_color)
        p.setPen(QPen(arm_color, 1))
        p.drawRect(box)
        p.setFont(font_arm)
        p.setPen(QPen(arm_color))
        p.drawText(box, Qt.AlignmentFlag.AlignCenter, arm_txt)

        # 3) 載具圖示（NEUTRAL 青色，標示 vehicle 類型）
        vt = (self._d_vehicle or '').upper()
        sym_x = bx + aw + 12
        sym_y = box.center().y()
        p.setPen(QPen(QColor(TC.NEUTRAL), 1.5))
        p.setBrush(Qt.BrushStyle.NoBrush)
        if 'PLANE' in vt or 'FIXED' in vt or 'VTOL' in vt:
            # 機翼簡化符號
            p.drawLine(int(sym_x - 8), int(sym_y), int(sym_x + 8), int(sym_y))
            p.drawLine(int(sym_x), int(sym_y - 4), int(sym_x), int(sym_y + 4))
        elif vt:
            # 多旋翼 X
            p.drawLine(int(sym_x - 6), int(sym_y - 6), int(sym_x + 6), int(sym_y + 6))
            p.drawLine(int(sym_x + 6), int(sym_y - 6), int(sym_x - 6), int(sym_y + 6))

    # ── 底部聚合遙測 ─────────────────────────────────────────────
    def _draw_telemetry(self, p: QPainter, rect: QRectF):
        """
        關鍵遙測聚合：GS/VS/BAT/GPS。
        全用 _d_* 慢速顯示快照（§5.17.22.1.1.1：精讀數值 ≤ 1 Hz）。
        色彩語意按 TABLE XL 嚴格上色：
          BAT < 20% → HOSTILE (紅)；< 50% → WARNING (黃)；≥ 50% → FRIENDLY (綠)
          GPS < 3D fix → HOSTILE；≥ 3D → FRIENDLY
          VS 上升 → FRIENDLY；下降 → 中性 FG_PRIMARY
        """
        p.fillRect(rect, QColor(TC.BG_SUNKEN))
        p.setPen(QPen(QColor(TC.BORDER_SUBTLE), 1))
        p.drawLine(int(rect.left()), int(rect.top()),
                    int(rect.right()), int(rect.top()))

        font_label = TF.condensed(9, bold=True, letter_spacing=0.8)
        # ⚠ 不用粗體：Cascadia Mono Bold 9pt 的「5」在螢幕上跟「6」幾乎一樣（24.5V 看成 24.6V；2026-10-22 截圖查到）
        font_value = TF.mono(9, bold=False)
        x = rect.left() + 6
        y = rect.top() + 14

        def line(label, value, vcolor=TC.FG_PRIMARY, lcolor=TC.NEUTRAL):
            nonlocal y
            p.setFont(font_label)
            p.setPen(QPen(QColor(lcolor)))
            p.drawText(int(x), int(y), label)
            fm = QFontMetrics(font_label)
            lw = fm.horizontalAdvance(label)
            p.setFont(font_value)
            p.setPen(QPen(QColor(vcolor)))
            p.drawText(int(x + lw + 2), int(y), value)
            y += 12

        # GS（地速）— §5.17.18.10.6 不使用前導零
        line('GS', f'{self._d_gs:5.1f} m/s')
        # VS（升降率）— 上升綠，下降中性
        vs_color = TC.FRIENDLY if self._d_vs > 0.1 else TC.FG_PRIMARY
        sign = '+' if self._d_vs >= 0 else ''
        line('VS', f'{sign}{self._d_vs:.1f} m/s', vcolor=vs_color)
        # BAT（電量）— 三段語意色
        if self._d_bat_pct >= 0:
            bcol = TC.HOSTILE  if self._d_bat_pct < THRESHOLDS.battery_critical else \
                    TC.WARNING  if self._d_bat_pct < THRESHOLDS.battery_low else \
                    TC.FRIENDLY
            line('BAT', f'{self._d_bat_v:.1f}V {self._d_bat_pct}%', vcolor=bcol)
        else:
            line('BAT', f'{self._d_bat_v:.1f}V', vcolor=TC.WARNING)
        # GPS（fix）— 3D 以上綠，不足紅
        fix_name = {0: 'NoFix', 1: 'NoFix', 2: '2D', 3: '3D',
                     4: 'DGPS', 5: 'RTK-Flt', 6: 'RTK-Fix'}.get(self._d_gps_fix, '?')
        gcol = TC.FRIENDLY if self._d_gps_fix >= 3 else TC.HOSTILE
        line('GPS', f'{self._d_gps_sats}sat {fix_name}', vcolor=gcol)


# ══════════════════════════════════════════════════════════════════════
#  UavInfoCard — 單台 UAV 的資訊欄（姿態球 + 遙測）
# ══════════════════════════════════════════════════════════════════════
class UavInfoCard(QFrame):
    """單台無人機的卡片：標題 + 姿態球 + 簡要遙測。

    額外互動（多機指揮）：
      - 點擊卡片本體 → 發出 ``selected(sysid)``，讓 SITLHud 將其設為「指令對象」。
      - 標題列右側「✕」鈕 → 發出 ``disconnect_requested(sysid)``，單獨斷線此機。
      - ``set_selected(bool)`` 切換選取高亮邊框（FG_EMPHASIS 琥珀）。
    """

    # 卡片被點選（要求成為指令對象）；參數為 sysid
    selected = pyqtSignal(int)
    # 要求單獨斷線此機；參數為 sysid
    disconnect_requested = pyqtSignal(int)
    # 下載這台飛完的 DataFlash 日誌（.bin）／取消下載；參數為 sysid
    log_download_requested = pyqtSignal(int)
    log_cancel_requested = pyqtSignal(int)

    def __init__(self, sysid: int, parent=None):
        super().__init__(parent)
        self.sysid: int = sysid
        self._selected: bool = False          # 是否為目前指令對象
        # 卡片寬 290 容得下完整 HUD；全直角設計（border-radius:0）
        self.setFixedWidth(290)
        # objectName 化 → 邊框樣式只作用在卡片本身，不外溢到子 QFrame（標題/座標列）
        self.setObjectName('uavCard')
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        # 點選行為提示（toggle 取消不易自我說明 → 用 tooltip 揭露）
        self.setToolTip('點擊選取此 UAV 為指令對象；再點一下取消（回全部）。')
        self._apply_card_style()
        v = QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)

        # 視窗式標題列（NEUTRAL 青色標籤）
        title_bar = QFrame()
        title_bar.setStyleSheet(
            f'QFrame{{background:{TC.BG_SUNKEN};border:none;'
            f'border-bottom:1px solid {TC.BORDER_SUBTLE};border-radius:0;}}'
            f'QLabel{{background:transparent;}}'
        )
        title_bar.setFixedHeight(20)
        tb = QHBoxLayout(title_bar)
        tb.setContentsMargins(8, 0, 4, 0)
        tb.setSpacing(6)
        self.lbl_title = QLabel(f'UAV{sysid} — Head-Up Display')
        self.lbl_title.setStyleSheet(
            f'color:{TC.FG_SECONDARY};font-size:10px;'
            f'font-family:{TF.css_condensed()};letter-spacing:1px;'
        )
        tb.addWidget(self.lbl_title)
        tb.addStretch()

        # 下載此機飛行日誌（.bin）鈕（icon-only；飛完上鎖後才有用，解鎖中按下會被 MainWindow 擋並說明）
        self.btn_logdl = QPushButton()
        self.btn_logdl.setIcon(get_icon('download', color=TC.NEUTRAL, size=12))
        self.btn_logdl.setIconSize(QSize(12, 12))
        self.btn_logdl.setFixedSize(22, 18)
        self.btn_logdl.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_logdl.setToolTip(f'下載 UAV{sysid} 飛控裡最新一份飛行日誌（.bin）到 runs/flight_logs/。\n'
                                  '飛控只在上鎖（落地）後才給；數傳 57600 很慢（數十 MB 要數十分鐘），USB／SITL 快。')
        self.btn_logdl.setStyleSheet(
            'QPushButton{background:transparent;border:none;border-radius:0;}'
            f'QPushButton:hover{{background:{TC.BG_ELEVATED};}}'
        )
        self.btn_logdl.clicked.connect(lambda: self.log_download_requested.emit(self.sysid))
        tb.addWidget(self.btn_logdl)

        # 單獨斷線鈕（icon-only，HOSTILE 語意：移除此連線）
        self.btn_disconnect = QPushButton()
        self.btn_disconnect.setIcon(get_icon('unlink', color=TC.HOSTILE, size=12))
        self.btn_disconnect.setIconSize(QSize(12, 12))
        # 22x18：放大點擊目標降低誤觸（標題列 20px，仍容得下）
        self.btn_disconnect.setFixedSize(22, 18)
        self.btn_disconnect.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_disconnect.setToolTip(f'單獨斷線 UAV{sysid}（不影響其他機）')
        # hover 用 BG_ELEVATED（深色提亮），而非 HOSTILE 填滿 —
        # 否則紅色 unlink icon 疊在紅底上會看不見。
        self.btn_disconnect.setStyleSheet(
            'QPushButton{background:transparent;border:none;border-radius:0;}'
            f'QPushButton:hover{{background:{TC.BG_ELEVATED};}}'
        )
        # 用 lambda 帶入 sysid；按鈕本身吃掉點擊，不會誤觸卡片選取
        self.btn_disconnect.clicked.connect(
            lambda: self.disconnect_requested.emit(self.sysid))
        tb.addWidget(self.btn_disconnect)
        v.addWidget(title_bar)

        # MIL-STD-1472H 完整 HUD
        self.hud = MissionPlannerHud()
        self.hud.setMinimumHeight(240)
        v.addWidget(self.hud, 1)

        # 底部位置欄（NEUTRAL 中性資訊）
        pos_bar = QFrame()
        pos_bar.setStyleSheet(
            f'QFrame{{background:{TC.BG_SUNKEN};border:none;'
            f'border-top:1px solid {TC.BORDER_SUBTLE};border-radius:0;}}'
            f'QLabel{{background:transparent;}}'
        )
        pos_bar.setFixedHeight(18)
        pb = QHBoxLayout(pos_bar)
        pb.setContentsMargins(6, 0, 6, 0)
        pb.setSpacing(6)
        self.lbl_pos = QLabel('--, --')
        self.lbl_pos.setStyleSheet(
            f'color:{TC.NEUTRAL};font-size:9px;font-family:{TF.css_mono()};'
        )
        # 固定尺寸：每次 setText 文字變寬就 updateGeometry → 整排卡片重新排版（2026-10-17 效能）；
        # 座標列只有一行等寬字，固定 190×14 容得下「📍 23.45123,120.28123」
        self.lbl_pos.setFixedSize(190, 14)
        pb.addWidget(self.lbl_pos)
        pb.addStretch()
        v.addWidget(pos_bar)

        # 日誌下載列（平常藏起來；下載中顯示進度＋取消，完成／失敗顯示一行結果）
        self.log_bar = QFrame()
        self.log_bar.setStyleSheet(
            f'QFrame{{background:{TC.BG_SUNKEN};border:none;'
            f'border-top:1px solid {TC.BORDER_SUBTLE};border-radius:0;}}'
            f'QLabel{{background:transparent;}}'
        )
        self.log_bar.setFixedHeight(18)
        lb = QHBoxLayout(self.log_bar)
        lb.setContentsMargins(6, 0, 4, 0)
        lb.setSpacing(4)
        self.log_progress = QProgressBar()
        self.log_progress.setRange(0, 1000)
        self.log_progress.setValue(0)
        self.log_progress.setTextVisible(False)
        self.log_progress.setFixedHeight(8)
        lb.addWidget(self.log_progress, 1)
        self.lbl_log = QLabel('')
        self.lbl_log.setStyleSheet(f'color:{TC.FG_SECONDARY};font-size:9px;font-family:{TF.css_mono()};')
        lb.addWidget(self.lbl_log, 2)
        self.btn_log_cancel = QPushButton('✕')
        self.btn_log_cancel.setFixedSize(16, 14)
        self.btn_log_cancel.setToolTip('取消這台的日誌下載')
        self.btn_log_cancel.setStyleSheet(
            f'QPushButton{{background:transparent;border:none;color:{TC.HOSTILE};font-size:9px;}}'
            f'QPushButton:hover{{background:{TC.BG_ELEVATED};}}')
        self.btn_log_cancel.clicked.connect(lambda: self.log_cancel_requested.emit(self.sysid))
        lb.addWidget(self.btn_log_cancel)
        self.log_bar.hide()
        v.addWidget(self.log_bar)
        self._log_state = 'idle'            # 'idle' | 'running' | 'done' | 'failed'

        # 為了向後相容：保留別名（其他模組若直接讀 attitude / lbl_mode 等不會崩）
        self.attitude = self.hud
        self.lbl_mode    = QLabel()   # 啞元 — 實際資料在 self.hud
        self.lbl_armed   = QLabel()
        self.lbl_vehicle = QLabel()
        self.lbl_alt = self.lbl_gs = self.lbl_hdg = QLabel()
        self.lbl_roll = self.lbl_pitch = self.lbl_thr = QLabel()
        self.lbl_gps = self.lbl_bat = QLabel()

    def update_from_frame(self, frame):
        # 全部資料推進 HUD widget
        self.hud.update_from_frame(frame)
        # 只剩座標列在 HUD 之外更新
        self.lbl_pos.setText(f'📍 {frame.lat:.5f},{frame.lon:.5f}')

    # ── 日誌下載列 ───────────────────────────────────────────────
    @staticmethod
    def _fmt_mb(n: int) -> str:
        return f'{int(n) / 1048576:.1f} MB'

    def on_log_progress(self, info: dict) -> None:
        """SITLLink.log_download_progress 的 payload：{'id','size','received','bytes_per_s','eta_s','path'}。"""
        size = int(info.get('size', 0) or 0)
        got = int(info.get('received', 0) or 0)
        bps = float(info.get('bytes_per_s', 0.0) or 0.0)
        eta = info.get('eta_s')
        self._log_state = 'running'
        self.log_progress.setValue(int(1000 * got / size) if size > 0 else 0)
        eta_txt = f'，剩 {int(eta) // 60}:{int(eta) % 60:02d}' if isinstance(eta, (int, float)) and eta >= 0 else ''
        self.lbl_log.setText(f'⬇ #{info.get("id", "?")} {self._fmt_mb(got)}/{self._fmt_mb(size)} '
                             f'{bps / 1024:.0f} KiB/s{eta_txt}')
        self.lbl_log.setStyleSheet(f'color:{TC.NEUTRAL};font-size:9px;font-family:{TF.css_mono()};')
        self.btn_log_cancel.show()
        self.log_progress.show()
        self.log_bar.show()

    def on_log_done(self, info: dict) -> None:
        self._log_state = 'done'
        self.log_progress.setValue(1000)
        name = str(info.get('path', '')).replace('\\', '/').rsplit('/', 1)[-1]
        self.lbl_log.setText(f'✔ {name}（{self._fmt_mb(int(info.get("size", 0) or 0))}）')
        self.lbl_log.setStyleSheet(f'color:{TC.FRIENDLY};font-size:9px;font-family:{TF.css_mono()};')
        self.lbl_log.setToolTip(str(info.get('path', '')))
        self.btn_log_cancel.hide()
        self.log_bar.show()

    def on_log_failed(self, reason: str) -> None:
        self._log_state = 'failed'
        self.lbl_log.setText(f'✖ {reason}')
        self.lbl_log.setToolTip(str(reason))
        self.lbl_log.setStyleSheet(f'color:{TC.HOSTILE};font-size:9px;font-family:{TF.css_mono()};')
        self.btn_log_cancel.hide()
        self.log_bar.show()

    def log_state(self) -> str:
        return self._log_state

    # ── 選取（指令對象）視覺狀態 ─────────────────────────────────
    def _apply_card_style(self) -> None:
        """依 ``_selected`` 套用卡片外框。

        選取 → FG_EMPHASIS 琥珀 2px 粗框；未選 → 1px 預設框，且 hover 時邊框轉
        NEUTRAL 青提示「可點選」（選取態不加 hover 規則，以免覆蓋選取框）。
        """
        if self._selected:
            frame_rule = (f'QFrame#uavCard{{background:{TC.BG_PRIMARY};'
                          f'border:2px solid {TC.FG_EMPHASIS};border-radius:0;}}')
        else:
            frame_rule = (f'QFrame#uavCard{{background:{TC.BG_PRIMARY};'
                          f'border:1px solid {TC.BORDER_DEFAULT};border-radius:0;}}'
                          f'QFrame#uavCard:hover{{border:1px solid {TC.NEUTRAL};}}')
        self.setStyleSheet(
            frame_rule + 'QLabel{background:transparent;border:none;}'
        )

    def set_selected(self, selected: bool) -> None:
        """設定本卡片是否為目前指令對象（切換高亮邊框與標題色）。"""
        self._selected = bool(selected)
        self._apply_card_style()
        # 標題列：選取時改用琥珀重點色，未選回中性次要色
        self.lbl_title.setStyleSheet(
            f'color:{TC.FG_EMPHASIS if self._selected else TC.FG_SECONDARY};'
            f'font-size:10px;font-family:{TF.css_condensed()};letter-spacing:1px;'
        )

    def mousePressEvent(self, evt):
        """點擊卡片任一處（斷線鈕除外）→ 要求成為指令對象。"""
        if evt.button() == Qt.MouseButton.LeftButton:
            self.selected.emit(self.sysid)
        super().mousePressEvent(evt)


class SITLHud(QWidget):
    """SITL 連線 + HUD 顯示面板

    2026-10-22「群控／單機」兩種顯示模式（LINK 標頭右側切換；使用者：servo output 跟 message
    應該是選哪一台就列哪一台）：
      * 群控（預設）：所有 UAV 機卡 ＋ 合併的飛控訊息（每行標 UAVn）；指令依 FLIGHT 標頭「對象」送出
        （點機卡＝單獨、「全部」＝廣播；語意與以前相同）。
      * 單機：只看選定的那一台 —— 大 HUD、詳細數值、SERVO OUTPUT、這台的飛控訊息；指令只送給這一台。
        選定的那台斷線 ⇒「未選定」，**不會**自動改看別台（2026-09-02 教訓：指令對象不能靜默換掉）。
    「看哪一台」（_single_sysid）與群控的指令對象（_selected_sysid）是兩份狀態；Command 分頁的機卡列與
    單機選擇互相同步（main_window.wire_uav_selection_sync）。
    """

    MODE_SWARM = 'swarm'
    MODE_SINGLE = 'single'
    #: 單機模式但還沒選飛機時 get_target_sysid() 回這個：對不上任何鏈路 ⇒ 既有守衛會拒絕送出（fail-closed）
    NO_TARGET = -1
    SINGLE_LOG_MAX = 450        # 單機訊息區行數上限；換看另一台時最多重畫這麼多行
    PARAM_LIVE_PER_S = 10       # 每台每秒最多即時列出幾則 PARAM 回報（其餘只存，之後補一行摘要）
    TARGET_ALL_TIP = '指令對象切回「全部 UAV」（取消單機選取）'
    TARGET_ALL_SINGLE_TIP = '單機模式下指令只送給選定的這一台；要對全部下指令請切回「群控」。'
    #: 單機 chip 的說明 —— 逐項對得上 main_window 的實際行為（2026-10-22 審查：以前寫「所有指令只送給這一台」，
    #: 但「上傳任務」等幾顆其實送給每一台）。
    SINGLE_MODE_TIP = (
        '單機：只看選定的一台——大 HUD、這台的飛控訊息、SERVO OUTPUT、詳細數值。\n'
        'FLIGHT／MISSION／LOG 的按鈕只對這一台：上傳任務、讀回、圍籬、寫入（含「寫入每架匯入」只寫這台那份）、'
        '下載 .bin（只下載這台）。\n'
        '「依時程放行」是整批起飛，單機模式不送（請切回群控）。\n'
        '別台斷線、別台的嚴重訊息仍會在這裡提示。')

    connect_requested    = pyqtSignal(str)   # 連線字串
    connect_embedded_requested = pyqtSignal()  # 連線到嵌入式蜂群（模式 A1 監看）
    disconnect_requested = pyqtSignal()       # 斷開全部連線
    uav_disconnect_requested = pyqtSignal(int)  # 單獨斷線指定 sysid（卡片快速斷線鈕）
    launch_sitl_requested = pyqtSignal(str, int)   # vehicle, count
    stop_sitl_requested   = pyqtSignal()
    # MAVLink 命令
    cmd_arm        = pyqtSignal()
    cmd_disarm     = pyqtSignal()
    cmd_set_mode   = pyqtSignal(str)         # 模式名稱
    cmd_takeoff    = pyqtSignal(float)       # 高度
    cmd_upload     = pyqtSignal()            # 上傳當前任務
    cmd_download   = pyqtSignal()            # 從飛控讀回任務航線（Read WPs）
    # Mission Planner 風格航點檔三兄弟（Load / Write / Save；Read 就是上面的 cmd_download）。
    # 對象一律由 main_window._target_links() 決定，這裡不帶參數。
    cmd_load_wp    = pyqtSignal()            # 從檔案載入航點檔（純本機讀檔，不需要連線）
    cmd_write_wp   = pyqtSignal()            # 把已載入的航點檔寫入飛控（安全關鍵，見 on_sitl_write_wp）
    cmd_save_wp    = pyqtSignal()            # 把「剛讀回」的任務存成航點檔
    # 多檔匯入：一次選多個 .waypoints／.plan，每檔明確指派給一架（對話框），再逐機寫入
    cmd_import_wps = pyqtSignal()            # 開匯入對話框（純本機讀檔＋預覽，不需要連線）
    cmd_write_imported = pyqtSignal()        # 把「每架匯入的那一份」各自寫入對應 sysid（每架各跳一次確認）
    # 飛完後下載 DataFlash 日誌（.bin）：逐機（卡片鈕，帶 sysid）／全部連線／取消
    cmd_log_download = pyqtSignal(int)
    cmd_log_download_all = pyqtSignal()
    cmd_log_cancel = pyqtSignal(int)
    cmd_upload_fence = pyqtSignal()          # 上傳電子圍籬
    cmd_auto_start = pyqtSignal()            # 一鍵 AUTO+ARM+MISSION_START
    cmd_param_set  = pyqtSignal(str, float, str)   # name, value, ptype
    cmd_param_get  = pyqtSignal(str)
    cmd_params_batch = pyqtSignal(list)            # [(name,value,ptype), ...]
    cmd_vtol_transition = pyqtSignal(int)          # VTOL 轉換: 3=MC, 4=FW
    cmd_release_store  = pyqtSignal(int, str)      # 投彈：掛架編號(1/2), 彈種
    cmd_guided_takeoff = pyqtSignal(float)         # 固定翼相容 GUIDED 起飛（TAKEOFF 模式）
    cmd_guided_goto_mode = pyqtSignal(bool)        # 點圖飛到模式開關（長機 GUIDED 飛到，僚機跟）
    cmd_guided_takeoff_mode = pyqtSignal(bool)     # 點圖起飛模式開關（點地圖 → 整群 GUIDED 起飛）
    # 群控／單機（2026-10-22）
    view_mode_changed = pyqtSignal(str)            # 'swarm' / 'single'
    single_uav_changed = pyqtSignal(int)           # 單機看的 sysid；被清掉（斷線／全部清空）時發 NO_TARGET

    def __init__(self, parent=None):
        super().__init__(parent)
        self._connected = False
        self._sitl_running = False
        # 目前「指令對象」sysid；None = 全部 UAV（廣播）。
        # 由卡片點選設定，main_window._sitl_broadcast 會讀取以決定派送範圍。
        self._selected_sysid: Optional[int] = None
        # ── 群控／單機（2026-10-22）──
        self._mode = self.MODE_SWARM
        self._single_sysid: Optional[int] = None   # 單機看的那一台（群控時也記著：切過去就是它）
        self._single_hold = False                  # 單機選定的那台斷線 ⇒ 等使用者重選，不自動接下一台
        self._single_lost_sysid: Optional[int] = None
        self._last_frame: dict = {}                # sysid → 最近一幀（切換對象／模式時重畫用）
        self._last_frame_t: dict = {}              # sysid → 收到最近一幀的 time.monotonic()
        self._servo_cache: dict = {}               # sysid → {(ch, key): value}（SERVOn_* 讀回值）
        self._msgs = SitlMessageLog()
        self._unread_crit: dict = {}               # 單機時「別台」的嚴重訊息（severity ≤ 3）未讀數
        self._unread_last: dict = {}               # 同上，最後一則的內文（那台斷線時留一行說明用）
        self._single_src = 'init'                  # 最近一次 single_uav_changed 的來源（'auto' ＝ 第一台遙測自動選）
        self._servo_purge_sids: set = set()        # 斷線／清空後要再清一次 SERVO 快取的 sysid（見 _schedule_servo_purge）
        self._servo_purge_posted = False
        self._param_burst: dict = {}               # sysid → [視窗起點, 視窗內筆數, 被壓下的筆數]
        self._servo_autoexpanded = False           # 第一次切到單機自動展開 SERVO OUTPUT（只一次）
        self._single_chips: dict = {}              # sysid → 單機選台 chip
        self._link_line_state = None
        self._param_flush_timer = QTimer(self)     # PARAM 洪水：壓下的筆數 1.1 s 後補一行摘要
        self._param_flush_timer.setSingleShot(True)
        self._param_flush_timer.setInterval(1100)
        self._param_flush_timer.timeout.connect(self._flush_param_summaries)
        self._init_ui()

    def _init_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)
        # 戰術風格重排（2026-10-17）：面板真實寬度只有螢幕 22%（約 420–560 px），以前 25 顆全寬
        # 飽和色長條疊成 1600 px 高；現在分成 LINK / FLIGHT / MISSION / LOG / PARAMS 五個區段，
        # 區段內用 3–4 欄的扁平「chip」鈕（深底、左側 3 px 語意色條、SVG 圖示、無 emoji）。
        # ⚠ 所有屬性名、信號、工具提示逐字不變（測試鎖著）；蜂群攻擊的投放列與「嵌入式蜂群」鈕
        #   仍會建構，但只有 config.features.STRIKE_ENABLED 時才進版面（使用者 2026-10-17：先拿掉）。
        from config import features as _features
        strike_on = bool(_features.strike_enabled())

        # ── LINK：連線 + 內建 SITL（合成一個區段、兩列）─────────────────
        link_frame, link_body, link_head = self._panel('LINK', '連線', TC.NEUTRAL)
        # 顯示模式（2026-10-22）：群控／單機，放 LINK 標頭右側（root 第 0 項仍是 LINK、區段順序不變）
        link_head.addWidget(self._cap('顯示'))
        self.btn_mode_swarm = self._chip('群控', 'info', 'radar', checkable=True)
        self.btn_mode_swarm.setToolTip(
            '群控：所有 UAV 機卡＋合併飛控訊息（每行標 UAVn）；指令依 FLIGHT 標頭「對象」送出。')
        self.btn_mode_single = self._chip('單機', 'info', 'drone', checkable=True)
        self.btn_mode_single.setToolTip(self.SINGLE_MODE_TIP)
        self._mode_group = QButtonGroup(self)
        self._mode_group.setExclusive(True)
        for b, m in ((self.btn_mode_swarm, self.MODE_SWARM), (self.btn_mode_single, self.MODE_SINGLE)):
            self._mode_group.addButton(b)
            # clicked（不是 toggled）：程式同步勾選狀態時不會回頭觸發切換
            b.clicked.connect(lambda _=False, mm=m: self.set_view_mode(mm))
            link_head.addWidget(b)
        self.btn_mode_swarm.setChecked(True)
        ctrl = QHBoxLayout()
        ctrl.setSpacing(6)
        ctrl.addWidget(self._cap('SITL'))

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
        self.preset_combo.setMinimumWidth(160)
        self.preset_combo.setStyleSheet(self._input_css())
        ctrl.addWidget(self.preset_combo, 1)

        # 連線按鈕 — IconButton('link') 走全域 btnPrimary tone；
        # 連線後動態切換為 'unlink' + btnDanger（見 on_connected / on_disconnected）
        self.btn_connect = IconButton('link', '連線', tone='primary', compact=True)
        self.btn_connect.clicked.connect(self._on_connect_clicked)
        ctrl.addWidget(self.btn_connect)

        # 嵌入式蜂群監看連線（模式 A1）— 不啟動內建 SITL，
        # 連到嵌入式 Jetson 原生 ArduPlane SITL 的 TCP server（N 機 TCP）。
        # 屬蜂群攻擊那一系：預設不進版面（features.STRIKE_ENABLED）。
        self.btn_embedded = IconButton('drone', '嵌入式蜂群', tone='neutral', compact=True)
        self.btn_embedded.setToolTip(
            '連線到嵌入式蜂群（Jetson 原生 ArduPlane SITL，TCP 監看）\n'
            '不啟動內建 SITL — 純監看 N 機遙測'
        )
        self.btn_embedded.clicked.connect(self.connect_embedded_requested.emit)
        if strike_on:
            ctrl.addWidget(self.btn_embedded)
        else:
            self.btn_embedded.setParent(self)
            self.btn_embedded.hide()
        link_body.addLayout(ctrl)

        lrow = QHBoxLayout()
        lrow.setSpacing(6)
        lrow.addWidget(self._cap('內建'))

        self.vehicle_combo = QComboBox()
        self.vehicle_combo.addItems(['PLANE (固定翼)', 'COPTER (多旋翼)', 'VTOL (垂直起降)'])
        self.vehicle_combo.setStyleSheet(self._input_css())
        lrow.addWidget(self.vehicle_combo, 1)

        # 數量
        self.count_spin = QSpinBox()
        self.count_spin.setRange(1, 6)
        self.count_spin.setValue(1)
        self.count_spin.setSuffix(' 台')
        self.count_spin.setStyleSheet(
            f'QSpinBox{{background:{TC.BG_SUNKEN};color:{TC.FG_PRIMARY};border:1px solid {TC.BORDER_DEFAULT};'
            f'border-radius:0;padding:3px 6px;font-size:11px;font-family:{TF.css_mono()};}}'
        )
        lrow.addWidget(self.count_spin)

        # SITL 啟動按鈕 — 'launch' icon + orange tone（一鍵英雄按鈕）；
        # 啟動後切換為 'stop' + btnDanger（見 on_sitl_launched / on_sitl_stopped）
        self.btn_launch = IconButton('launch', '啟動', tone='orange', compact=True)
        self.btn_launch.clicked.connect(self._on_launch_clicked)
        lrow.addWidget(self.btn_launch)
        link_body.addLayout(lrow)
        root.addWidget(link_frame)

        # ── 多 UAV 卡片橫向滾動區 ─────────────────────────────
        self._uav_cards: dict = {}
        cards_scroll = QScrollArea()
        cards_scroll.setWidgetResizable(True)
        cards_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        cards_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        cards_scroll.setFixedHeight(320)
        # 全域 QSS 已將 QScrollArea 設為 transparent + 無邊框，無需 inline
        cards_holder = QWidget()
        self._cards_row = QHBoxLayout(cards_holder)
        self._cards_row.setContentsMargins(2, 2, 2, 2)
        self._cards_row.setSpacing(6)
        self._cards_row.addStretch()
        cards_scroll.setWidget(cards_holder)
        self._cards_scroll = cards_scroll        # 群控才顯示（單機模式整排收起、也不更新）
        root.addWidget(cards_scroll)

        # ── UAV 單機（2026-10-22）：選台 ＋ 這台的快捷動作 ＋ 大 HUD ＋ 遙測新鮮度（單機模式才顯示）──
        # 單機模式的區段順序（群控藏起來的區段插在中間，群控的順序不變）：
        #   LINK → UAV（這裡）→ FLIGHT → MSG → MISSION → LOG → SERVO OUTPUT → DETAIL → PARAMS
        #   FLIGHT 緊接在 HUD 下面：單機模式指令只送這一台，DISARM／RTL 不能被 SERVO 表（16 列）擠到畫面外
        #   （2026-10-22 審查實測：以前 DISARM 在單機模式 y=1367 px，1080p 面板只看得到約 900 px）。
        self._single_frame, single_body, _single_head = self._panel('UAV', '單機', TC.NEUTRAL)
        sel_row = QHBoxLayout()
        sel_row.setSpacing(4)
        sel_row.addWidget(self._cap('機號'), 0, Qt.AlignmentFlag.AlignTop)
        chips_host = QWidget()
        # 每台一顆 chip（_rebuild_single_selector 依 sysid 排序重建）；會換行（_FlowLayout），不撐寬整頁
        self._single_chip_row = _FlowLayout(chips_host, spacing=4)
        sel_row.addWidget(chips_host, 1)
        single_body.addLayout(sel_row)
        # 這一台的快捷動作：群控時在機卡標題列（⬇／斷線鈕＋下載進度），單機時機卡整排收起 ⇒ 搬到這裡
        self._single_actions = QWidget()
        act_row = QHBoxLayout(self._single_actions)
        act_row.setContentsMargins(0, 0, 0, 0)
        act_row.setSpacing(4)
        self.btn_single_logdl = self._chip('下載 .bin', 'emphasis', 'download')
        self.btn_single_logdl.setToolTip(
            '下載這一台飛控裡最新一份飛行日誌（.bin）到 runs/flight_logs/（同機卡標題列的 ⬇ 鈕）。\n'
            '飛控只在上鎖（落地）後才給；數傳 57600 很慢，USB／SITL 快。')
        self.btn_single_logdl.clicked.connect(lambda: self._emit_for_single(self.cmd_log_download))
        self.btn_single_disconnect = self._chip('斷線', 'stop', 'unlink')
        self.btn_single_disconnect.setToolTip('單獨斷線這一台（不影響其他機；同機卡標題列的斷線鈕）')
        self.btn_single_disconnect.clicked.connect(
            lambda: self._emit_for_single(self.uav_disconnect_requested))
        act_row.addWidget(self.btn_single_logdl)
        act_row.addWidget(self.btn_single_disconnect)
        act_row.addStretch(1)
        single_body.addWidget(self._single_actions)
        # 日誌下載列：跟這台機卡的下載列同一份狀態（進度／完成／失敗），換台時從機卡複製
        self.single_log_bar = QWidget()
        lb = QHBoxLayout(self.single_log_bar)
        lb.setContentsMargins(0, 0, 0, 0)
        lb.setSpacing(4)
        self.single_log_progress = QProgressBar()
        self.single_log_progress.setRange(0, 1000)
        self.single_log_progress.setTextVisible(False)
        self.single_log_progress.setFixedHeight(8)
        lb.addWidget(self.single_log_progress, 1)
        self.lbl_single_log = QLabel('')
        self.lbl_single_log.setStyleSheet(f'color:{TC.FG_SECONDARY};font-size:9px;font-family:{TF.css_mono()};')
        lb.addWidget(self.lbl_single_log, 2)
        self.btn_single_log_cancel = self._chip('取消', 'muted')
        self.btn_single_log_cancel.setToolTip('取消這台的日誌下載')
        self.btn_single_log_cancel.clicked.connect(lambda: self._emit_for_single(self.cmd_log_cancel))
        lb.addWidget(self.btn_single_log_cancel)
        self.single_log_bar.hide()
        single_body.addWidget(self.single_log_bar)
        self.lbl_single_empty = self._lbl('尚無連線的 UAV（連線後自動選第一台）', TC.FG_MUTED)
        self.lbl_single_empty.setWordWrap(True)
        single_body.addWidget(self.lbl_single_empty)
        self.single_hud = MissionPlannerHud()
        self.single_hud.setMinimumHeight(250)
        single_body.addWidget(self.single_hud)
        # 「UAVn · 遙測 <1 s 前 · ACTIVE · PLANE」：> 3 s 轉琥珀、> 10 s 紅＋STALE（1 Hz，只在單機模式跑）
        self.lbl_single_link = QLabel('')
        self.lbl_single_link.setWordWrap(True)  # 窄面板換行，不撐寬整頁
        self.lbl_single_link.setStyleSheet(
            f'color:{TC.FG_SECONDARY};font-size:10px;font-family:{TF.css_sans()};')
        single_body.addWidget(self.lbl_single_link)
        self._single_age_timer = QTimer(self)
        self._single_age_timer.setInterval(1000)
        self._single_age_timer.timeout.connect(self._refresh_single_link_line)

        # 詳細數值：沿用舊版單機 HUD 框架（lbl_* 屬性名不變），自成一個 DETAIL 區段（單機模式、放在 SERVO 下面）
        self._single_detail_frame, detail_body, _detail_head = self._panel('DETAIL', '詳細數值', TC.NEUTRAL)
        hud_frame = QFrame()
        self._legacy_hud_frame = hud_frame      # on_telemetry 用它判斷「沒顯示就不更新那些 label」
        hud = QVBoxLayout(hud_frame)
        hud.setContentsMargins(0, 2, 0, 0)
        hud.setSpacing(4)

        # 第一行：模式 + 武裝 + 飛行器類型
        row1 = QHBoxLayout()
        self.lbl_mode    = self._big('---', '#FFD700', 18)
        self.lbl_armed   = self._big('DISARMED', '#888', 12)
        self.lbl_vehicle = self._lbl('---', TC.NEUTRAL, bold=True)
        row1.addWidget(self.lbl_mode)
        row1.addStretch()
        row1.addWidget(self.lbl_vehicle)
        row1.addSpacing(8)
        row1.addWidget(self.lbl_armed)
        hud.addLayout(row1)

        # 分隔線
        sep = QFrame(); sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet(f'color:{TC.BORDER_DEFAULT};')
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
        sep2.setStyleSheet(f'color:{TC.BORDER_DEFAULT};')
        hud.addWidget(sep2)

        # 底部：GPS + 電量
        bottom = QHBoxLayout()
        self.lbl_gps = self._lbl('GPS: --', TC.FRIENDLY)
        self.lbl_bat = self._lbl('🔋 --V', TC.WARNING)
        self.lbl_pos = self._lbl('--, --', TC.NEUTRAL)
        bottom.addWidget(self.lbl_gps)
        bottom.addStretch()
        bottom.addWidget(self.lbl_bat)
        hud.addLayout(bottom)

        pos_row = QHBoxLayout()
        pos_row.addWidget(self.lbl_pos)
        pos_row.addStretch()
        hud.addLayout(pos_row)

        hud_frame.setVisible(True)              # 框本身不藏：顯不顯示由 DETAIL 區段（祖先）決定
        detail_body.addWidget(hud_frame)
        root.addWidget(self._single_frame)

        # ── MSG 飛控訊息（單機模式：選定那一台 ＋ 本機訊息 ＋ 別台斷線警示；群控的合併訊息在頁面最下方）──
        #    先建好，FLIGHT 之後才放進版面（見上面「單機模式的區段順序」）
        self._single_msg_frame, msg_body, _msg_head = self._panel('MSG', '飛控訊息', TC.NEUTRAL)
        self.single_log_view = QPlainTextEdit()
        self.single_log_view.setReadOnly(True)
        self.single_log_view.setMaximumBlockCount(self.SINGLE_LOG_MAX)
        self.single_log_view.setProperty('role', 'log')
        self.single_log_view.style().polish(self.single_log_view)
        self.single_log_view.setFixedHeight(150)
        msg_body.addWidget(self.single_log_view)

        # ── SERVO OUTPUT 面板（Mission Planner 風格 16 通道 PWM 表格）──
        # 2026-10-22：只在單機模式顯示，內容是**選定那一台**的（PWM 與 SERVOn_* 都依 sysid 分開）；LOG 之後才放進版面
        from ui.widgets.servo_output_panel import ServoOutputPanel
        self.servo_panel = ServoOutputPanel()
        # 預設可摺疊（節省空間）；點按「▶ SERVO OUTPUT」開合；第一次切到單機自動展開一次
        self._servo_visible = False
        self._btn_servo_toggle = QPushButton('▶ SERVO OUTPUT  (16 通道 PWM)')
        self._btn_servo_toggle.setStyleSheet(self._toggle_css())
        self._btn_servo_toggle.clicked.connect(self._toggle_servo_panel)
        self.servo_panel.setVisible(False)

        # ── FLIGHT：解鎖／起飛／模式（標頭右側＝指令對象）──────────────────
        flight_frame, flight_body, flight_head = self._panel('FLIGHT', '飛行', TC.FRIENDLY)
        flight_head.addWidget(self._cap('對象'))
        # 指令對象值標籤（全部 → 琥珀；單機 → NEUTRAL 青框，見 _update_target_label）
        self.lbl_cmd_target = self._lbl('全部 UAV', TC.FG_EMPHASIS, bold=True)
        flight_head.addWidget(self.lbl_cmd_target)
        # 「全部」復位鈕：把指令對象切回全部 UAV
        self.btn_target_all = QPushButton('全部')
        self.btn_target_all.setFixedHeight(20)
        self.btn_target_all.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_target_all.setToolTip(self.TARGET_ALL_TIP)
        self.btn_target_all.setStyleSheet(
            f'QPushButton{{background:{TC.BG_SUNKEN};color:{TC.FG_SECONDARY};'
            f'border:1px solid {TC.BORDER_DEFAULT};border-radius:0;'
            f'padding:0 8px;min-height:18px;max-height:18px;font-size:10px;font-family:{TF.css_condensed()};'
            f'letter-spacing:1px;}}'
            f'QPushButton:hover{{background:{TC.BORDER_STRONG};color:{TC.FG_PRIMARY};}}'
            # 單機模式停用（指令只送選定那台）：看得出是停用的
            f'QPushButton:disabled{{color:{TC.FG_MUTED};border-color:{TC.BORDER_SUBTLE};}}'
        )
        self.btn_target_all.clicked.connect(lambda: self._set_target(None))
        flight_head.addWidget(self.btn_target_all)

        # ARM / DISARM / 起飛
        arm_row = QHBoxLayout()
        arm_row.setSpacing(4)
        self.btn_arm    = self._chip('ARM', 'go', 'arm')
        self.btn_disarm = self._chip('DISARM', 'stop', 'disarm')
        self.btn_takeoff= self._chip('起飛', 'info', 'takeoff')
        self.btn_arm.clicked.connect(self.cmd_arm.emit)
        self.btn_disarm.clicked.connect(self.cmd_disarm.emit)
        self.btn_takeoff.clicked.connect(lambda: self.cmd_takeoff.emit(30.0))
        arm_row.addWidget(self.btn_arm)
        arm_row.addWidget(self.btn_disarm)
        arm_row.addWidget(self.btn_takeoff)
        flight_body.addLayout(arm_row)

        # ── GUIDED 群飛指令列（固定翼相容）＋ 點圖指令 ────────────────────
        # ArduPlane 純固定翼 GUIDED 不吃 NAV_TAKEOFF/SET_POSITION_TARGET，
        # 故起飛走 TAKEOFF 模式、飛到走 guided 航點（MISSION_ITEM current=2）。
        # 點圖兩者互斥（開一個自動關另一個）；2D Leaflet 與 3D Cesium 點擊皆會
        # 走到 main_window.on_corner_added，故同一攔截即同時支援兩種地圖。
        guided_row = QHBoxLayout()
        guided_row.setSpacing(4)
        self.btn_guided_takeoff = self._chip('GUIDED 起飛', 'go', 'takeoff')
        self.btn_guided_takeoff.setToolTip(
            '固定翼相容 GUIDED 起飛：切 TAKEOFF 模式滑跑爬升 + 自動 ARM（整群一次起飛）。')
        self.btn_guided_takeoff.clicked.connect(
            lambda: self.cmd_guided_takeoff.emit(60.0))
        guided_row.addWidget(self.btn_guided_takeoff)
        self.btn_guided_takeoff_pt = self._chip('點圖起飛', 'go', None, checkable=True)
        self.btn_guided_takeoff_pt.setToolTip(
            '開啟後在 2D 或 3D 地圖點一下 → 長機帶領的分階段 GUIDED 起飛：\n'
            '長機單獨起飛 → 確認長機升空且進入 GUIDED → 僚機才跟著起飛、組 V 編隊。\n'
            '再按一次關閉。與「點圖飛到」互斥。')
        self.btn_guided_takeoff_pt.toggled.connect(self.cmd_guided_takeoff_mode.emit)
        self.btn_guided_goto = self._chip('點圖飛到', 'warn', None, checkable=True)
        self.btn_guided_goto.setToolTip(
            '開啟後在 2D 或 3D 地圖點一下 → 長機 GUIDED 飛往該點（guided 航點），\n'
            '僚機由蜂群節點自動跟隨成 V 字。再按一次關閉。與「點圖起飛」互斥。')
        self.btn_guided_goto.toggled.connect(self.cmd_guided_goto_mode.emit)
        guided_row.addWidget(self.btn_guided_takeoff_pt)
        guided_row.addWidget(self.btn_guided_goto)
        flight_body.addLayout(guided_row)

        # 飛行模式（4 欄；QuadPlane 的 Q 模式與 MC/FW 轉換同一格網）
        flight_body.addWidget(self._subcap('MODE'))
        mode_grid = QGridLayout()
        mode_grid.setHorizontalSpacing(4)
        mode_grid.setVerticalSpacing(4)
        self._mode_buttons: dict = {}
        modes = ['GUIDED', 'AUTO', 'LOITER', 'RTL',
                 'LAND', 'STABILIZE', 'QHOVER', 'QLOITER',
                 'QLAND', 'QRTL']
        for i, mode in enumerate(modes):
            b = self._chip(mode, 'muted')
            b.clicked.connect(lambda _, m=mode: self.cmd_set_mode.emit(m))
            self._mode_buttons[mode] = b
            mode_grid.addWidget(b, i // 4, i % 4)
        # VTOL 轉換（MAV_CMD_DO_VTOL_TRANSITION）
        self.btn_vtol_mc = self._chip('→ MC 多旋翼', 'info')
        self.btn_vtol_mc.setToolTip('MAV_CMD_DO_VTOL_TRANSITION → state=3 (MC)')
        self.btn_vtol_mc.clicked.connect(lambda: self.cmd_vtol_transition.emit(3))
        self.btn_vtol_fw = self._chip('→ FW 固定翼', 'info')
        self.btn_vtol_fw.setToolTip('MAV_CMD_DO_VTOL_TRANSITION → state=4 (FW)')
        self.btn_vtol_fw.clicked.connect(lambda: self.cmd_vtol_transition.emit(4))
        mode_grid.addWidget(self.btn_vtol_mc, 2, 2)
        mode_grid.addWidget(self.btn_vtol_fw, 2, 3)
        flight_body.addLayout(mode_grid)
        root.addWidget(flight_frame)

        # ── RELEASE（投放）：蜂群攻擊那一系；建構但預設不進版面 ─────────────
        self._build_release_group()
        if strike_on:
            root.addWidget(self._release_group)
        else:
            self._release_group.setParent(self)
            self._release_group.hide()

        # 單機：這台的飛控訊息緊接在 FLIGHT 下面（按了指令馬上看得到回應）
        root.addWidget(self._single_msg_frame)

        # ── MISSION：任務上傳／讀回／航點檔／圍籬／一鍵起飛 ────────────────
        mission_frame, mission_body, _mission_head = self._panel('MISSION', '任務', TC.FG_EMPHASIS)
        m_row1 = QHBoxLayout()
        m_row1.setSpacing(4)
        # 上傳任務
        self.btn_upload = self._chip('上傳任務', 'emphasis', 'upload')
        self.btn_upload.clicked.connect(self.cmd_upload.emit)
        m_row1.addWidget(self.btn_upload)

        # 從飛控讀回任務（= Mission Planner 的 Ctrl+F → Read WPs）
        self.btn_download = self._chip('讀回任務', 'emphasis', 'download')
        self.btn_download.setToolTip(
            '向飛控索取目前存著的任務並畫到地圖上。\n'
            '看 AUTO 架構的預劃航線用（起飛 → 巡航 → 攻擊發起點決斷圈）。\n'
            '⚠️ 讀回來的是**飛控裡實際存著的那一份**，不是規劃器算出來的那份 —— '
            '兩者不同就是上傳沒生效。'
        )
        self.btn_download.clicked.connect(self.cmd_download.emit)
        m_row1.addWidget(self.btn_download)

        self.btn_save_wp = self._chip('存成航點檔', 'emphasis', 'export')
        self.btn_save_wp.setToolTip(
            '把「剛剛從飛控讀回」的那份任務（「讀回任務」的結果）存成 .waypoints 檔。\n'
            '⚠️ 存的是**讀回結果**，不是尚未寫入飛控的載入檔 —— 兩者不同就是還沒寫入成功。\n'
            '尚未讀回任何任務時會說明原因，不會偷偷存成別的東西。'
        )
        self.btn_save_wp.clicked.connect(self.cmd_save_wp.emit)
        m_row1.addWidget(self.btn_save_wp)
        mission_body.addLayout(m_row1)

        # ── Mission Planner 風格航點檔：Load / Import / Fence ──────
        m_row2 = QHBoxLayout()
        m_row2.setSpacing(4)
        self.btn_load_wp = self._chip('載入航點檔', 'emphasis', 'import')
        self.btn_load_wp.setToolTip(
            '從 .waypoints（QGC WPL）或 .plan（QGroundControl）檔案載入一份任務，\n'
            '預覽到地圖上 —— 純讀檔 + 畫圖，不需要連線、也不會自動寫入飛控。\n'
            '載入時若檔案有格式或語意錯誤（如缺欄位、DO_JUMP 目標不存在），\n'
            '會擋下並列出原因；載入成功後才能按「寫入飛控」。'
        )
        self.btn_load_wp.clicked.connect(self.cmd_load_wp.emit)
        m_row2.addWidget(self.btn_load_wp)

        self.btn_import_wps = self._chip('多檔匯入', 'emphasis', 'import')
        self.btn_import_wps.setToolTip(
            '一次選多個 .waypoints / .plan（Mission Planner 存的 QGC WPL 110 也行），\n'
            '在對話框裡把每個檔**明確指派給一架**（sysid；檔名有 UAV-3 / sys3 之類會先幫你填），\n'
            '確定後每架各自預覽到 2D / 3D 地圖（不同顏色）。\n'
            '純讀檔 + 畫圖，不需要連線、不會自動寫入飛控；要寫入按下面的「寫入每架匯入」。\n'
            '有錯誤的檔、沒指派的檔、兩個檔指到同一架 ⇒ 對話框不准按確定。'
        )
        self.btn_import_wps.clicked.connect(self.cmd_import_wps.emit)
        m_row2.addWidget(self.btn_import_wps)

        self.btn_upload_fence = self._chip('上傳圍籬', 'warn', 'polygon')
        self.btn_upload_fence.setToolTip(
            'FENCE_ENABLE=1, FENCE_TYPE=7 (MaxAlt+Circle+Polygon),\n'
            'FENCE_ACTION=1 (RTL/QRTL)，4 頂點矩形 + 高度上下限'
        )
        self.btn_upload_fence.clicked.connect(self.cmd_upload_fence.emit)
        m_row2.addWidget(self.btn_upload_fence)
        mission_body.addLayout(m_row2)

        # ── 寫入飛控（安全關鍵，紅色語意；兩顆都預設停用）──────────────
        m_row3 = QHBoxLayout()
        m_row3.setSpacing(4)
        self.btn_write_wp = self._chip('寫入飛控', 'stop', 'write')
        self.btn_write_wp.setToolTip(
            '把「已載入的航點檔」寫入飛控。\n'
            '⚠️ 這顆鈕本身只依「有沒有連線」與「有沒有寫入還在等回報」兩個條件\n'
            '決定能不能按；「指令對象要恰好選中單一台 UAV」是送出時另外檢查的\n'
            '——選成「全部」或選到 0/多台的話，按下去會先跳出提示，不會真的送出。\n'
            '⚠️ 送出前一定會跳出確認對話框，列出目標機、連線方式、航點摘要與所有警告，\n'
            '對真實飛控（COM/序列埠）會用更強的字句提醒 —— 只有明確按「是」才會真的送出，\n'
            '按「取消」什麼都不會發生。\n'
            '寫入期間本鈕會暫時停用，避免連點造成兩筆寫入互相競爭。'
        )
        # 預設無連線 → 不可用；之後由 main_window._update_write_wp_enabled()
        # 依「有連線 且 沒有正在等待回報的寫入」這兩個條件統一決定，見那裡的註解。
        # ⚠️ 這裡刻意不額外檢查「指令對象是否恰好一台」——那個檢查在
        # on_sitl_write_wp() 的入口做（見那裡 (2) 對象面），保持「按不按得下去」
        # 跟「按下去會不會被拒絕」分開兩層，鈕本身的 enabled 狀態不需要跟著
        # HUD 上「指令對象」下拉選單的即時變動重算。
        self.btn_write_wp.setEnabled(False)
        self.btn_write_wp.clicked.connect(self.cmd_write_wp.emit)
        m_row3.addWidget(self.btn_write_wp)

        self.btn_write_imported = self._chip('寫入每架匯入', 'stop', 'write')
        self.btn_write_imported.setToolTip(
            '把「多檔匯入」指派好的每一份，各自寫入**對應 sysid** 的那一架：\n'
            'UAV-1 拿 UAV-1 的檔、UAV-2 拿 UAV-2 的檔，不看 HUD 的「指令對象」。\n'
            '⚠️ 每一架都會各跳一次確認對話框（跟「寫入飛控」同一套守衛），按取消就跳過那架；\n'
            '指派了但沒連線的 sysid 會列出來、不會寫到別台。\n'
            '沒有匯入任何檔、沒有連線、或有寫入還在等回報時本鈕停用。'
        )
        self.btn_write_imported.setEnabled(False)
        self.btn_write_imported.clicked.connect(self.cmd_write_imported.emit)
        m_row3.addWidget(self.btn_write_imported)
        mission_body.addLayout(m_row3)

        # 一鍵起飛：全寬填色英雄鈕。⚠ 必須是 mission_body（QVBoxLayout）的**直接**子項：
        # main_window 的「依時程放行」小元件用 lay.indexOf(btn_auto_start)+1 插在它正下方。
        self.btn_auto_start = self._chip('一鍵起飛  AUTO + ARM + START', 'warn', 'rocket', fill=True)
        self.btn_auto_start.clicked.connect(self.cmd_auto_start.emit)
        mission_body.addWidget(self.btn_auto_start)
        root.addWidget(mission_frame)

        # ── LOG：飛完後下載每台的飛行日誌（.bin）─────────────────────────
        log_frame, log_body, _log_head = self._panel('LOG', '日誌', TC.NEUTRAL)
        log_row = QHBoxLayout()
        log_row.setSpacing(6)
        self.btn_log_download_all = self._chip('下載 .bin（全部連線）', 'emphasis', 'download')
        self.btn_log_download_all.setToolTip(
            '對每一條連線索取飛控的日誌清單、挑最新一份下載到 runs/flight_logs/<日期>/UAV-<sysid>_log<id>_<時間>.bin。\n'
            '逐機下載用卡片標題列的 ⬇ 鈕。飛控只在上鎖（落地）後才給日誌；數傳 57600 約 8 KiB/s（數十 MB 要數十分鐘），\n'
            'USB 直連與 SITL 很快。不會刪飛控上的日誌、不用 MAVFTP。'
        )
        self.btn_log_download_all.clicked.connect(self.cmd_log_download_all.emit)
        log_row.addWidget(self.btn_log_download_all, 1)
        self.chk_auto_logdl = QCheckBox('落地後自動下載')
        self.chk_auto_logdl.setChecked(False)
        self.chk_auto_logdl.setToolTip('勾了：每台解鎖→上鎖幾秒後自動開始下載最新日誌。數傳慢的實機建議不勾，手動按。')
        self.chk_auto_logdl.setStyleSheet(f'color:{TC.FG_SECONDARY};font-size:10px;')
        log_row.addWidget(self.chk_auto_logdl)
        log_body.addLayout(log_row)
        root.addWidget(log_frame)

        # 單機：SERVO OUTPUT（16 列）與 DETAIL 放在 FLIGHT／MISSION／LOG 之後，不把指令鈕擠到畫面外
        root.addWidget(self._btn_servo_toggle)
        root.addWidget(self.servo_panel)
        root.addWidget(self._single_detail_frame)

        # ── PARAMS：參數寫入（預設摺起；少用）────────────────────────────
        self._params_visible = False
        self._btn_params_toggle = QPushButton('▶ PARAMS  參數寫入 (PARAM_SET)')
        self._btn_params_toggle.setStyleSheet(self._toggle_css())
        self._btn_params_toggle.clicked.connect(self._toggle_params_panel)
        root.addWidget(self._btn_params_toggle)

        param_frame = QFrame()
        param_frame.setObjectName('hudPanel')
        param_frame.setStyleSheet(
            self._panel_css()
            + f'QLineEdit,QComboBox{{background:{TC.BG_SUNKEN};color:{TC.FG_PRIMARY};'
              f'border:1px solid {TC.BORDER_DEFAULT};border-radius:0;padding:2px 5px;'
              f'font-size:11px;font-family:{TF.css_sans()};}}'
        )
        pv = QVBoxLayout(param_frame)
        pv.setContentsMargins(8, 6, 8, 6)
        pv.setSpacing(5)

        # 常用參數預設按鈕（一鍵套用 SITL 友善設定）
        preset_row = QHBoxLayout()
        preset_row.setSpacing(4)
        btn_preset_sitl = self._chip('SITL 友善預設', 'muted', 'tool')
        btn_preset_sitl.setToolTip(
            'ARMING_CHECK=0, FS_*=0, GPS 容忍寬鬆 — 方便模擬測試'
        )
        btn_preset_sitl.clicked.connect(self._on_preset_sitl_friendly)
        btn_preset_tune = self._chip('速度/航點調校', 'muted', 'settings')
        btn_preset_tune.setToolTip(
            'WPNAV_SPEED / AIRSPEED_CRUISE / WP_RADIUS 調整'
        )
        btn_preset_tune.clicked.connect(self._on_preset_speed_tune)
        btn_load_param = self._chip('讀取參數檔', 'muted', 'import')
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
        btn_set = self._chip('寫入', 'info', 'write')
        btn_set.clicked.connect(self._on_param_set_clicked)
        btn_get = self._chip('讀取', 'muted', 'read')
        btn_get.clicked.connect(self._on_param_get_clicked)
        btn_row.addWidget(btn_set)
        btn_row.addWidget(btn_get)
        pv.addLayout(btn_row)

        # 常用參數下拉（點選即填入名稱）
        common_row = QHBoxLayout()
        common_row.setSpacing(4)
        common_row.addWidget(self._cap('常用'))
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

        self._params_frame = param_frame
        param_frame.setVisible(False)
        root.addWidget(param_frame)

        # ── 訊息區（群控：全部 UAV 合併的時間線，每行標 UAVn；類似 MP messages）────────
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(500)
        # role="log" → 全域 QSS QPlainTextEdit[role="log"] 提供 log well 樣式
        self.log_view.setProperty('role', 'log')
        self.log_view.style().polish(self.log_view)
        self.log_view.setFixedHeight(120)
        root.addWidget(self.log_view)

        # 相容舊程式：lbl_status.setText / setStyleSheet → 一則「本機訊息」（不屬於任何一台，兩種模式都看得到）
        class _LogShim:
            def __init__(self, hud):
                self._hud = hud
                self._color = TC.FG_SECONDARY
            def setText(self, txt):
                self._hud._append_msg(None, None, str(txt), 'local', color=self._color)
            def setStyleSheet(self, css):
                import re
                m = re.search(r'color:\s*(#[0-9a-fA-F]{3,6})', css)
                if m:
                    self._color = m.group(1)
        self.lbl_status = _LogShim(self)
        self.lbl_status.setText('未連線')

        self._apply_mode_visibility()
        self._sync_mode_chips()
        root.addStretch()

    # ── 樣式輔助（MIL-STD-1472H 合規）─────────────────────
    # 全部走 TacticalColors / TacticalFonts，全直角，無漸層
    @staticmethod
    def _lbl(text, color=None, bold=False):
        l = QLabel(text)
        weight = '600' if bold else '400'
        c = color or TC.FG_PRIMARY
        l.setStyleSheet(
            f'color:{c};font-size:11px;font-weight:{weight};'
            f'font-family:{TF.css_sans()};'
        )
        return l

    @staticmethod
    def _big(text, color, size):
        l = QLabel(text)
        l.setStyleSheet(
            f'color:{color};font-size:{size}px;font-weight:700;'
            f'font-family:{TF.css_mono()};letter-spacing:0.5px;'
        )
        return l

    @staticmethod
    def _cap(text):
        l = QLabel(text)
        l.setStyleSheet(
            f'color:{TC.FG_SECONDARY};font-size:10px;'
            f'font-family:{TF.css_condensed()};letter-spacing:1px;'
        )
        return l

    @staticmethod
    def _val(text):
        l = QLabel(text)
        l.setStyleSheet(
            f'color:{TC.FG_PRIMARY};font-size:13px;font-weight:600;'
            f'font-family:{TF.css_mono()};'
        )
        return l

    @staticmethod
    def _tiny_label(text):
        lb = QLabel(text)
        lb.setStyleSheet(f'color:{TC.FG_MUTED};font-size:10px;')
        return lb

    # ── 戰術風格版面元件（2026-10-17 重排）────────────────────────────
    _TONE_COLORS = None     # 延後建立（TC 在模組載入時就有，但放方法裡讀比較不易被覆寫層打斷）

    @classmethod
    def _tone_color(cls, tone: str) -> str:
        """語意色：go=綠（正常／放行）、stop=紅（危險／停止）、warn／emphasis=琥珀、info=青、muted=次要灰。"""
        return {
            'go': TC.FRIENDLY, 'stop': TC.HOSTILE, 'warn': TC.WARNING, 'emphasis': TC.FG_EMPHASIS,
            'info': TC.NEUTRAL, 'muted': TC.FG_SECONDARY,
        }.get(tone, TC.FG_SECONDARY)

    @staticmethod
    def _panel_css() -> str:
        return (f'QFrame#hudPanel{{background:{TC.BG_SECONDARY};border:1px solid {TC.BORDER_SUBTLE};'
                f'border-radius:0;}}'
                'QLabel{background:transparent;border:none;}')

    @staticmethod
    def _input_css() -> str:
        return (f'QComboBox{{background:{TC.BG_SUNKEN};color:{TC.FG_PRIMARY};border:1px solid {TC.BORDER_DEFAULT};'
                f'border-radius:0;padding:3px 6px;font-size:11px;font-family:{TF.css_sans()};}}')

    @staticmethod
    def _toggle_css() -> str:
        """摺疊區段的標頭鈕（SERVO OUTPUT / PARAMS）：左對齊、壓縮字體、細框。"""
        return (f'QPushButton{{background:{TC.BG_SECONDARY};color:{TC.FG_SECONDARY};'
                f'border:1px solid {TC.BORDER_SUBTLE};border-radius:0;padding:0 8px;'
                f'min-height:20px;max-height:20px;font-size:10px;font-weight:600;text-align:left;'
                f'letter-spacing:2px;font-family:{TF.css_condensed()};}}'
                f'QPushButton:hover{{background:{TC.BG_ELEVATED};color:{TC.FG_PRIMARY};}}')

    def _panel(self, code: str, title: str, accent: str):
        """一個區段：標頭（3 px 色條 + 英文代碼 + 中文名 + 右側可加元件）＋ 細線 ＋ 內容。

        回 (frame, body_layout, header_layout)。frame 的 objectName 是 'hudPanel'（樣式只作用在框本身）。
        """
        frame = QFrame()
        frame.setObjectName('hudPanel')
        frame.setStyleSheet(self._panel_css())
        body = QVBoxLayout(frame)
        body.setContentsMargins(8, 5, 8, 7)
        body.setSpacing(5)
        head = QHBoxLayout()
        head.setSpacing(6)
        bar = QFrame()
        bar.setFixedSize(3, 12)
        bar.setStyleSheet(f'background:{accent};border:none;border-radius:0;')
        head.addWidget(bar)
        lbl = QLabel(f'{code}  <span style="color:{TC.FG_SECONDARY};letter-spacing:0px;">{title}</span>')
        lbl.setTextFormat(Qt.TextFormat.RichText)
        lbl.setStyleSheet(f'color:{accent};font-size:10px;font-weight:700;letter-spacing:2px;'
                          f'font-family:{TF.css_condensed()};')
        head.addWidget(lbl)
        head.addStretch()
        body.addLayout(head)
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setFixedHeight(1)
        line.setStyleSheet(f'background:{TC.BORDER_SUBTLE};border:none;')
        body.addWidget(line)
        return frame, body, head

    @staticmethod
    def _subcap(text: str) -> QLabel:
        lb = QLabel(text)
        lb.setStyleSheet(f'color:{TC.FG_MUTED};font-size:9px;letter-spacing:2px;'
                         f'font-family:{TF.css_condensed()};padding-top:2px;')
        return lb

    @classmethod
    def _chip_css(cls, tone: str = 'muted', fill: bool = False) -> str:
        """_chip 的樣式表（拆出來：單機選台 chip 有未讀嚴重訊息時要換成 stop 色調）。"""
        c = cls._tone_color(tone)
        if fill:
            return (f'QPushButton{{background:{c};color:{TC.BG_PRIMARY};border:1px solid {c};'
                    f'border-radius:0;padding:0 8px;min-height:22px;max-height:22px;'
                    f'font-size:11px;font-weight:700;font-family:{TF.css_condensed()};letter-spacing:1.5px;}}'
                    f'QPushButton:hover{{background:{TC.FG_PRIMARY};border-color:{TC.FG_PRIMARY};}}'
                    f'QPushButton:pressed{{background:{c};}}'
                    f'QPushButton:disabled{{background:{TC.BG_SUNKEN};color:{TC.FG_MUTED};border-color:{TC.BORDER_SUBTLE};}}')
        return (f'QPushButton{{background:{TC.BG_ELEVATED};color:{c};border:1px solid {TC.BORDER_DEFAULT};'
                f'border-left:3px solid {c};border-radius:0;padding:0 6px;min-height:22px;max-height:22px;'
                f'font-size:11px;font-weight:600;font-family:{TF.css_condensed()};letter-spacing:1px;}}'
                f'QPushButton:hover{{background:{TC.BORDER_SUBTLE};border-color:{c};}}'
                f'QPushButton:pressed{{background:{TC.BG_SUNKEN};}}'
                f'QPushButton:checked{{background:{c};color:{TC.BG_PRIMARY};border-color:{c};}}'
                f'QPushButton:disabled{{background:{TC.BG_SUNKEN};color:{TC.FG_MUTED};'
                f'border-color:{TC.BORDER_SUBTLE};border-left-color:{TC.BORDER_DEFAULT};}}')

    @classmethod
    def _chip(cls, text: str, tone: str = 'muted', icon=None, *, fill: bool = False,
              checkable: bool = False) -> QPushButton:
        """扁平指令鈕（24 px）：深底、左側 3 px 語意色條、語意色文字、SVG 圖示；fill=True 是填色英雄鈕。

        取代以前的全寬飽和色長條（_cmd_btn）；_cmd_btn 仍保留給舊呼叫端。
        checked（點圖起飛／點圖飛到）時整顆填成語意色。
        """
        b = QPushButton(text)
        b.setFixedHeight(24)
        b.setCursor(Qt.CursorShape.PointingHandCursor)
        b.setCheckable(bool(checkable))
        b.setStyleSheet(cls._chip_css(tone, fill))
        b.setProperty('chipTone', tone)
        c = cls._tone_color(tone)
        if icon:
            try:
                b.setIcon(get_icon(icon, color=(TC.BG_PRIMARY if fill else c), size=12))
                b.setIconSize(QSize(12, 12))
            except Exception:       # noqa: BLE001 — 沒圖示就只剩字，不能讓面板建不起來
                pass
        return b

    def _build_release_group(self) -> None:
        """投放（蜂群攻擊）那一列：保險 + 一號彈／二號彈 + 通道／PWM／彈種。

        建構但**不一定進版面**（config.features.STRIKE_ENABLED）；屬性名與工具提示逐字保留，
        tests/test_store_release.py 直接操作這些 widget。
        """
        frame, body, _head = self._panel('RELEASE', '投放', TC.HOSTILE)
        self._release_group = frame
        # 兩顆投彈鈕預設 disabled，必須先扳開「投放保險」才會亮。
        # 這不是多餘的手續：這兩顆鈕會直接把伺服打到 HI 位置放掉掛載，
        # 誤點沒有 undo。實機上的機械保險銷是同一個道理，這裡做成軟體版。
        rel_row = QHBoxLayout()
        rel_row.setSpacing(4)
        self.btn_release_safety = self._cmd_btn('🔐 投放保險', TC.BG_SECONDARY)
        self.btn_release_safety.setCheckable(True)
        self.btn_release_safety.setToolTip(
            '投放保險。扳開後「一號彈 / 二號彈」才可按。\n'
            '投放後會自動復位，避免連續誤觸。')
        self.btn_release_safety.toggled.connect(self._on_release_safety)
        rel_row.addWidget(self.btn_release_safety)

        self.btn_release_1 = self._cmd_btn('💣 一號彈', TC.HOSTILE)
        self.btn_release_2 = self._cmd_btn('💣 二號彈', TC.HOSTILE)
        for b, idx in ((self.btn_release_1, 1), (self.btn_release_2, 2)):
            b.setEnabled(False)
            b.setToolTip(
                f'投放 {idx} 號掛架：MAV_CMD_DO_SET_SERVO 把該通道打到 HI (2000 μs)。\n'
                '通道與 PWM 見下方「投放設定」；\n'
                '注意：該通道若被指派給飛控會主動驅動的功能，'
                '指令會被擋掉、PWM 完全不動。\n'
                '2026-09-01 對 ArduCopter SITL 實測：\n'
                '  0 Disabled / 1 RCPassThru / 51 RCIN1 -> 打得上去且停住\n'
                '  33 Motor1 / 24 Flap -> 完全沒上去\n'
                '投放通道請設為 0 (Disabled)，其餘不要依賴。\n'
                '按下後看 SERVO OUTPUT 面板該通道的 PWM 是否真的跳上去；'
                '那是飛控回報的值，不是我們送出去的值。\n'
                '沒有跳上去就不會有落點動畫（動畫由讀回觸發，不是按鈕觸發）。')
            b.clicked.connect(
                lambda _, i=idx: self._on_release_clicked(i))
        rel_row.addWidget(self.btn_release_1)
        rel_row.addWidget(self.btn_release_2)
        body.addLayout(rel_row)

        # 投放設定：通道與彈種（彈種只影響 3D 地圖的落點模擬，不影響送出的指令）
        rel_cfg = QHBoxLayout()
        rel_cfg.setSpacing(4)
        rel_cfg.addWidget(self._tiny_label('通道'))
        self.spin_release_ch1 = QSpinBox()
        self.spin_release_ch1.setRange(1, 16)
        self.spin_release_ch1.setValue(9)
        self.spin_release_ch1.setToolTip('一號彈的伺服通道（預設 9，與蜂群節點的左掛架一致）')
        self.spin_release_ch2 = QSpinBox()
        self.spin_release_ch2.setRange(1, 16)
        self.spin_release_ch2.setValue(10)
        self.spin_release_ch2.setToolTip('二號彈的伺服通道（預設 10，右掛架）')
        for sp in (self.spin_release_ch1, self.spin_release_ch2):
            sp.setFixedHeight(24)
            sp.setFixedWidth(74)      # 52/62 都會把兩位數的通道號(10)截成 1
            rel_cfg.addWidget(sp)
        rel_cfg.addWidget(self._tiny_label('HI'))
        self.spin_release_pwm = QSpinBox()
        self.spin_release_pwm.setRange(800, 2200)
        self.spin_release_pwm.setSingleStep(50)
        self.spin_release_pwm.setValue(2000)
        self.spin_release_pwm.setFixedHeight(24)
        self.spin_release_pwm.setFixedWidth(84)   # 64 會把 2000 截成 2
        self.spin_release_pwm.setToolTip('投放位置的 PWM（μs）。預設 2000 = HI。')
        rel_cfg.addWidget(self.spin_release_pwm)
        rel_cfg.addWidget(self._tiny_label('彈種'))
        self.combo_store_kind = QComboBox()
        self.combo_store_kind.addItem('傻瓜彈', 'dumb')
        self.combo_store_kind.addItem('煙霧彈', 'smoke')
        self.combo_store_kind.setFixedHeight(24)
        self.combo_store_kind.setToolTip(
            '只影響 3D 地圖上的落點模擬（阻力係數），不影響送給飛控的指令。\n'
            '傻瓜彈：低阻力，前拋距離明顯；煙霧彈：高阻力，幾乎垂直落下 + 煙柱。')
        rel_cfg.addWidget(self.combo_store_kind, 1)
        body.addLayout(rel_cfg)

    def _toggle_params_panel(self):
        """PARAMS 區段開合（預設摺起：參數寫入是少用的進階操作）。"""
        self._params_visible = not self._params_visible
        self._params_frame.setVisible(self._params_visible)
        self._btn_params_toggle.setText(
            ('▼' if self._params_visible else '▶') + ' PARAMS  參數寫入 (PARAM_SET)')

    def _on_release_safety(self, on: bool):
        """投放保險開關：只有扳開時兩顆投彈鈕才可按。"""
        self.btn_release_1.setEnabled(bool(on))
        self.btn_release_2.setEnabled(bool(on))
        self.btn_release_safety.setText('🔓 保險已開' if on else '🔐 投放保險')

    def _on_release_clicked(self, index: int):
        """送出投放請求，然後**立刻把保險復位** —— 一次開保險只放一發。"""
        kind = self.combo_store_kind.currentData() or 'dumb'
        self.cmd_release_store.emit(int(index), str(kind))
        self.btn_release_safety.setChecked(False)

    def release_channel(self, index: int) -> int:
        return int((self.spin_release_ch1 if index == 1
                    else self.spin_release_ch2).value())

    def release_pwm(self) -> int:
        return int(self.spin_release_pwm.value())

    @staticmethod
    def _cmd_btn(text, bg):
        """
        命令按鈕。bg 參數為「語意色」hint：
            HOSTILE/WARNING/FRIENDLY/NEUTRAL/AMBER/BG_SECONDARY
        會自動套用配對的文字色與 hover/disabled 狀態。
        """
        b = QPushButton(text)
        b.setFixedHeight(26)
        # 統一文字色 — 在飽和語意色上用主背景色，其他用主前景色
        text_color = TC.BG_PRIMARY if bg in (
            TC.HOSTILE, TC.WARNING, TC.FRIENDLY, TC.NEUTRAL, TC.FG_EMPHASIS
        ) else TC.FG_PRIMARY
        b.setStyleSheet(
            f'QPushButton{{background:{bg};color:{text_color};'
            f'border:1px solid {TC.BORDER_DEFAULT};border-radius:0;'
            f'padding:0 8px;font-size:11px;font-weight:600;'
            f'font-family:{TF.css_condensed()};letter-spacing:1px;}}'
            f'QPushButton:hover{{background:{TC.BORDER_STRONG};color:{TC.FG_PRIMARY};}}'
            f'QPushButton:disabled{{background:{TC.BG_SUNKEN};color:{TC.FG_MUTED};}}'
        )
        return b

    @staticmethod
    def _btn_style(bg):
        text_color = TC.BG_PRIMARY if bg in (
            TC.HOSTILE, TC.WARNING, TC.FRIENDLY, TC.NEUTRAL, TC.FG_EMPHASIS
        ) else TC.FG_PRIMARY
        return (
            f'QPushButton{{background:{bg};color:{text_color};'
            f'border:1px solid {TC.BORDER_DEFAULT};border-radius:0;'
            f'padding:0 12px;font-size:11px;font-weight:600;'
            f'font-family:{TF.css_condensed()};letter-spacing:1px;}}'
            f'QPushButton:hover{{background:{TC.BORDER_STRONG};color:{TC.FG_PRIMARY};}}'
            f'QPushButton:disabled{{background:{TC.BG_SUNKEN};color:{TC.FG_MUTED};}}'
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

    def set_vehicle(self, vehicle: str):
        """把載具下拉同步到 vehicle（'PLANE'/'COPTER'/'VTOL'）。

        啟動流程若因任務不符而改了載具，必須把下拉一起改過來，
        否則畫面會顯示一個沒有生效的選項。
        """
        target = (vehicle or '').upper()
        for i in range(self.vehicle_combo.count()):
            if self.vehicle_combo.itemText(i).upper().startswith(target):
                self.vehicle_combo.setCurrentIndex(i)
                return

    def _on_launch_clicked(self):
        if not self._sitl_running:
            txt = self.vehicle_combo.currentText()
            if 'VTOL' in txt:
                vehicle = 'VTOL'
            elif 'PLANE' in txt:
                vehicle = 'PLANE'
            else:
                vehicle = 'COPTER'
            count = self.count_spin.value()
            self.launch_sitl_requested.emit(vehicle, count)
        else:
            self.stop_sitl_requested.emit()

    def on_sitl_launched(self, vehicle: str, conn_str: str):
        self._sitl_running = True
        # 切換為「停止」狀態：danger tone + stop icon（IconButton objectName swap）
        self.btn_launch.setText('停止')
        self.btn_launch.setIcon(get_icon('stop', color='#FFFFFF', size=16))
        self.btn_launch.setObjectName('btnDanger')
        self.btn_launch.style().polish(self.btn_launch)
        # VTOL 特殊提示：顯示使用 ArduPlane 核心 + QuadPlane 物理模型
        # vehicle 可能是 'VTOL' 或 'VTOL x2' 等格式
        if 'VTOL' in vehicle.upper():
            self.lbl_status.setStyleSheet(f'color:{TC.FG_EMPHASIS};font-size:10px;padding:3px;')
            self.lbl_status.setText(
                f'🚀 VTOL SITL 已啟動 → {conn_str}\n'
                f'    使用 ArduPlane 核心模擬 VTOL QuadPlane\n'
                f'    (ArduPlane.exe --model quadplane)'
            )
        else:
            self.lbl_status.setText(f'🚀 SITL {vehicle} 已啟動 → {conn_str}')
        # 自動填入連線字串
        self.preset_combo.setEditText(conn_str)

    def on_sitl_stopped(self):
        self._sitl_running = False
        # 回到「啟動」狀態：orange tone + launch icon
        self.btn_launch.setText('啟動')
        self.btn_launch.setIcon(get_icon('launch', color='#FFFFFF', size=16))
        self.btn_launch.setObjectName('btnOrange')
        self.btn_launch.style().polish(self.btn_launch)

    def on_connected(self, conn_str: str):
        self._connected = True
        # 切換為「斷線」狀態：danger tone + unlink icon
        self.btn_connect.setText('斷線')
        self.btn_connect.setIcon(get_icon('unlink', color='#FFFFFF', size=16))
        self.btn_connect.setObjectName('btnDanger')
        self.btn_connect.style().polish(self.btn_connect)
        self.btn_connect.setEnabled(True)
        self.lbl_status.setText(f'✅ 已連線：{conn_str}')

    def on_disconnected(self, reason: str = ''):
        self._connected = False
        # 全部斷線 → 清掉所有 UAV 卡片並復位指令對象（避免殘留離線卡片）
        self.clear_cards()
        # 回到「連線」狀態：primary tone + link icon
        self.btn_connect.setText('連線')
        self.btn_connect.setIcon(get_icon('link', color='#FFFFFF', size=16))
        self.btn_connect.setObjectName('btnPrimary')
        self.btn_connect.style().polish(self.btn_connect)
        self.btn_connect.setEnabled(True)
        self.lbl_status.setText(f'已斷線：{reason}')
        self.lbl_mode.setText('---')
        self.lbl_armed.setText('DISARMED')
        self.lbl_armed.setStyleSheet(
            f'color:{TC.FG_MUTED};font-size:12px;font-weight:700;'
            f'font-family:{TF.css_mono()};'
        )

    def on_error(self, msg: str):
        self.btn_connect.setEnabled(True)
        self.lbl_status.setText(f'❌ {msg}')

    # ── 訊息（2026-10-22：每則帶 sysid 存進 SitlMessageLog；只畫進「看得到的那一區」）──────
    #   群控 → log_view（合併、每行標 UAVn）；單機 → single_log_view（只列選定那台 ＋ 本機訊息，不標 UAVn）。
    #   看不到的那一區不 append（切過去時從緩衝重畫）。severity 0=EMERG … 7=DEBUG。
    _KIND_TAG = {'fc': 'FC', 'gcs': 'GCS', 'log': 'LOG', 'strip': '條帶', 'link': 'LINK'}

    @staticmethod
    def _sev_color(severity) -> str:
        if severity is None:
            return TC.FG_SECONDARY
        s = int(severity)
        return TC.HOSTILE if s <= 3 else TC.WARNING if s <= 5 else TC.NEUTRAL

    def on_status_text(self, severity: int, text: str):
        """舊介面（不帶 sysid）：當成不屬於任何一台的飛控訊息。鏈路請接 on_status_text_sysid。"""
        self._append_msg(None, severity, text, 'fc')

    def on_status_text_sysid(self, sysid: int, severity: int, text: str, kind: str = 'gcs') -> None:
        """SITLLink.status_text_sysid(sysid, severity, text, kind)。sysid 0 ＝ 無法歸屬（當本機訊息）。"""
        self._append_msg(sysid, severity, text, kind)

    def on_uav_message(self, sysid: int, text: str, severity: int = 6, kind: str = 'gcs') -> None:
        """地面站針對某一台的訊息（斷線、航測條帶警報…）。"""
        self._append_msg(sysid, severity, text, kind)

    # ── 飛行日誌（.bin）下載：進度進卡片、訊息進 log 區 ──────────────
    def on_log_message(self, sysid: int, text: str, severity: int = 6) -> None:
        self._append_msg(sysid, severity, text, 'log')

    def _append_msg(self, sysid, severity, text, kind, color=None):
        """存一則訊息，並只在「現在看得到的那一區」即時加一行。回傳存進緩衝的 HudMsg。"""
        d = self.__dict__
        log = d.get('_msgs')
        if log is None:                       # 建構途中（理論上不會）
            return None
        e = log.add(sysid, severity, text, kind, color=color)
        single = d.get('_mode') == self.MODE_SINGLE
        view = d.get('single_log_view') if single else d.get('log_view')
        if view is None:
            return e
        if kind == 'param' and not self._param_live_ok(e.sysid):
            return e                          # PARAM 洪水：只存不列（1.1 s 後補一行摘要）
        if single:
            if e.kind == 'ext':
                return e                      # 非飛機自駕儀來源：不屬於任何一台（群控合併時間線看得到）
            own = e.sysid is None or e.sysid == self._view_sid()
            if not own and e.kind != 'link':
                # 單機只看選定那台；但別台的嚴重訊息不能就這樣沒人看見 ⇒ 在選台 chip 上標未讀
                if e.severity is not None and e.severity <= 3:
                    self._unread_crit[e.sysid] = self._unread_crit.get(e.sysid, 0) + 1
                    self._unread_last[e.sysid] = e.text
                    self._style_single_chip(e.sysid)
                return e
            # 別台斷線（link）是全隊警示：單機看哪一台都列出來，標 UAVn
            view.appendHtml(self._msg_html(e, with_uav_tag=not own))
        else:
            view.appendHtml(self._msg_html(e, with_uav_tag=True))
        sb = view.verticalScrollBar()
        sb.setValue(sb.maximum())
        return e

    def _view_sid(self):
        """單機訊息區正在列哪一台：選定的那台；選定那台斷線（停住等重選）時 ＝ 斷線的那台（歷史不清掉）。"""
        d = self.__dict__
        sid = d.get('_single_sysid')
        if sid is None and d.get('_single_hold'):
            sid = d.get('_single_lost_sysid')
        return sid

    def _msg_html(self, e, with_uav_tag: bool) -> str:
        """一行：HH:MM:SS [UAVn] [FC/GCS/LOG/條帶/LINK] 內文（內文 HTML 跳脫；PARAM 回報與本機訊息不加種類標）。
        ext（非飛機自駕儀來源）標 [EXT n]、不標 UAVn —— 它不是那一台的飛控說的。"""
        parts = [f'<span style="color:{TC.FG_MUTED}">{_time.strftime("%H:%M:%S", _time.localtime(e.t))}</span>']
        if e.kind == 'ext':
            src = f' {e.sysid}' if e.sysid is not None else ''
            parts.append(f'<span style="color:{TC.FG_SECONDARY}">[EXT{src}]</span>')
        else:
            if with_uav_tag and e.sysid is not None:
                parts.append(f'<span style="color:{TC.FG_PRIMARY}">[UAV{e.sysid}]</span>')
            tag = self._KIND_TAG.get(e.kind)
            if tag:
                parts.append(f'<span style="color:{TC.FG_SECONDARY}">[{tag}]</span>')
        if e.color:                           # 本機訊息（lbl_status 設的色）、PARAM 摘要
            color = e.color
        else:
            color = TC.FG_SECONDARY if e.kind == 'local' else self._sev_color(e.severity)
        body = _html.escape(e.text).replace('\n', '<br>')
        parts.append(f'<span style="color:{color}">{body}</span>')
        return ' '.join(parts)

    def _render_msg_view(self, view, entries, with_uav_tag: bool) -> None:
        """整區重畫（切模式／換看另一台）：最多 SINGLE_LOG_MAX 行（最新的那些）。

        with_uav_tag=False（單機區）時，不是這台的（別台斷線警示）仍標 UAVn。
        呼叫端要先 _flush_param_summaries(append=False)：被壓下的 PARAM 筆數先存成摘要訊息，重畫才列得到
        （以前重畫直接把每一台的計數歸零 ⇒ 換一次台，超出 50 筆緩衝的那幾百筆就再也沒有摘要；2026-10-22 審查）。
        """
        own = None if with_uav_tag else self._view_sid()
        view.setUpdatesEnabled(False)
        try:
            view.clear()
            for e in list(entries)[-self.SINGLE_LOG_MAX:]:
                tag = with_uav_tag or (e.sysid is not None and e.sysid != own)
                view.appendHtml(self._msg_html(e, tag))
        finally:
            view.setUpdatesEnabled(True)
        sb = view.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _param_live_ok(self, sid) -> bool:
        """這則 PARAM 回報要不要即時列出：每台每秒最多 PARAM_LIVE_PER_S 則（PARAM_REQUEST_LIST 會灌約 900 則）。"""
        now = _time.monotonic()
        st = self._param_burst.get(sid)
        if st is None or now - st[0] >= 1.0:
            st = [now, 0, st[2] if st is not None else 0]
            self._param_burst[sid] = st
        st[1] += 1
        if st[1] <= self.PARAM_LIVE_PER_S:
            return True
        st[2] += 1
        if not self._param_flush_timer.isActive():
            self._param_flush_timer.start()
        return False

    def _flush_param_summaries(self, append: bool = True) -> None:
        """被壓下的 PARAM 回報存成一則摘要「（參數回報 N 筆未逐筆列出）」（kind 'param'、帶那台的 sysid ⇒
        跟著那台的歷史、重畫也列得到）；append=True 時看得到的那一區即時加一行（群控標 UAVn）。
        每一台都存（單機看別台時也存：換過去才看得到）。"""
        d = self.__dict__
        log = d.get('_msgs')
        if log is None:
            return
        single = d.get('_mode') == self.MODE_SINGLE
        view = d.get('single_log_view') if single else d.get('log_view')
        cur = self._view_sid()
        wrote = False
        for sid, st in list(d.get('_param_burst', {}).items()):
            n, st[2] = st[2], 0
            if n <= 0:
                continue
            e = log.add(sid, None, f'（參數回報 {n} 筆未逐筆列出）', 'param', color=TC.FG_MUTED)
            if not append or view is None or (single and e.sysid is not None and e.sysid != cur):
                continue
            view.appendHtml(self._msg_html(e, with_uav_tag=not single))
            wrote = True
        if wrote:
            sb = view.verticalScrollBar()
            sb.setValue(sb.maximum())

    def on_log_progress(self, sysid: int, info: dict) -> None:
        card = self._uav_cards.get(int(sysid))
        if card is not None:
            card.on_log_progress(info)
            self._log_bar_changed(int(sysid))

    def on_log_done(self, sysid: int, info: dict) -> None:
        card = self._uav_cards.get(int(sysid))
        if card is not None:
            card.on_log_done(info)
            self._log_bar_changed(int(sysid))

    def on_log_failed(self, sysid: int, reason: str) -> None:
        card = self._uav_cards.get(int(sysid))
        if card is not None:
            card.on_log_failed(reason)
            self._log_bar_changed(int(sysid))

    def _log_bar_changed(self, sid: int) -> None:
        if sid == self.__dict__.get('_single_sysid'):
            self._refresh_single_log_bar()

    def _refresh_single_log_bar(self) -> None:
        """單機的日誌下載列 ＝ 選定那台機卡的下載列（同一份狀態：進度、結果、能不能取消）。"""
        d = self.__dict__
        bar = d.get('single_log_bar')
        if bar is None:
            return
        sid = d.get('_single_sysid')
        card = d.get('_uav_cards', {}).get(sid) if sid is not None else None
        if card is None or card.log_bar.isHidden():
            bar.hide()
            return
        self.single_log_progress.setValue(card.log_progress.value())
        self.lbl_single_log.setText(card.lbl_log.text())
        self.lbl_single_log.setStyleSheet(card.lbl_log.styleSheet())
        self.lbl_single_log.setToolTip(card.lbl_log.toolTip())
        self.btn_single_log_cancel.setVisible(not card.btn_log_cancel.isHidden())
        bar.show()

    def _emit_for_single(self, sig) -> None:
        """單機區的快捷動作：對「選定那台」發 sysid 信號；沒選定就什麼都不送。"""
        sid = self.__dict__.get('_single_sysid')
        if sid is not None:
            sig.emit(int(sid))

    def auto_log_download_enabled(self) -> bool:
        chk = getattr(self, 'chk_auto_logdl', None)
        return bool(chk is not None and chk.isChecked())

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
            card.selected.connect(self._on_card_selected)
            card.disconnect_requested.connect(self.uav_disconnect_requested.emit)
            card.log_download_requested.connect(self.cmd_log_download.emit)
            card.log_cancel_requested.connect(self.cmd_log_cancel.emit)
            # ── 依 sysid 數字大小插入（與連線順序無關）→ 卡片永遠按編號排列 ──
            # 插入索引 = 既有卡片中 sysid 比它小的數量（stretch 永遠在最後）。
            insert_idx = sum(1 for sid in self._uav_cards if sid < sysid)
            self._uav_cards[sysid] = card
            self._cards_row.insertWidget(insert_idx, card)
            # 套用目前選取高亮（新卡片預設未選，除非剛好等於目前指令對象）
            card.set_selected(sysid == self._selected_sysid)
            # 機卡同時是「已連線 sysid」的清單 ⇒ 單機選台 chip 跟著重建
            self._rebuild_single_selector()
        return card

    def clear_cards(self):
        for card in list(self._uav_cards.values()):
            card.setParent(None)
            card.deleteLater()
        self._uav_cards.clear()
        # 卡片清空 → 指令對象一併復位為「全部」
        self._selected_sysid = None
        # 單機狀態一併清掉（訊息歷史保留）。
        # 單機模式下選定的那台也在這次清空裡（＝它是最後一條連線）⇒ 一樣**停住**等使用者重選，跟「還有別台時
        # 選定那台斷線」（remove_card）同一套規則 —— 以前這裡把停住解除，下一台有遙測的就靜默變成單機指令對象
        # （2026-10-22 審查）。使用者自己按「全部斷線」才解除（main_window → release_single_hold）。
        d = self.__dict__
        had = d.get('_single_sysid')
        was_lost = d.get('_single_lost_sysid') if d.get('_single_hold') else None
        gone = set(d.get('_servo_cache') or {}) | set(d.get('_last_frame') or {})
        for sid in sorted(set(d.get('_unread_crit') or {})):
            self._note_unread_on_loss(sid)
        for k in ('_last_frame', '_last_frame_t', '_servo_cache', '_unread_crit', '_unread_last', '_param_burst'):
            if isinstance(d.get(k), dict):
                d[k].clear()
        self._schedule_servo_purge(gone)
        d['_single_sysid'] = None
        lost = had if had is not None else was_lost
        if d.get('_mode') == self.MODE_SINGLE and lost is not None:
            d['_single_hold'] = True
            d['_single_lost_sysid'] = lost
        else:
            d['_single_hold'] = False
            d['_single_lost_sysid'] = None
        if hasattr(self, 'lbl_cmd_target'):
            self._update_target_label()
        if '_single_chip_row' in d:
            self._rebuild_single_selector()
            self._apply_single_view()
        if had is not None:
            d['_single_src'] = 'remove'
            self.single_uav_changed.emit(self.NO_TARGET)

    def release_single_hold(self) -> bool:
        """使用者明確「全部斷線」之後呼叫：之後重新連線是新的一輪 ⇒ 解除「選定那台斷線、等重選」，
        第一台有遙測的照常自動選（main_window.on_sitl_disconnect）。有解除才回 True。"""
        d = self.__dict__
        if not d.get('_single_hold'):
            return False
        d['_single_hold'] = False
        d['_single_lost_sysid'] = None
        self._refresh_single_empty()
        self._apply_single_view()
        self._update_target_label()
        return True

    def _note_unread_on_loss(self, sid) -> None:
        """單機時別台有未讀的嚴重訊息、那台就斷線了（chip 跟著消失）⇒ 留一行全隊警示（每一台的單機畫面都列），
        不然那則嚴重訊息就完全沒人看見（2026-10-22 審查）。"""
        d = self.__dict__
        n = int((d.get('_unread_crit') or {}).get(sid, 0) or 0)
        if n <= 0 or d.get('_mode') != self.MODE_SINGLE:
            return
        last = (d.get('_unread_last') or {}).get(sid, '')
        self._append_msg(sid, 3, f'UAV{sid} 斷線前有 {n} 則未讀的嚴重訊息'
                         + (f'（最後一則：{last}）' if last else '') + '；切回「群控」可看完整訊息', 'link')

    def _schedule_servo_purge(self, sids) -> None:
        """斷線／清空後，那幾台的 SERVO 快取在**目前已排隊的事件都處理完之後**再清一次。

        全部斷線（main_window.on_sitl_disconnect／HIL 全部斷線）是 link.stop(); link.wait() 之後同步 clear_cards()：
        接收執行緒結束前已經排進 GUI 佇列的 servo_param_sysid（連線時那 80 筆讀回）會在清空**之後**才送到、把快取
        又建回來 ⇒ 同 sysid 重連時「參數 80/80」其實是上一顆板子的值（有效值契約；2026-10-22 審查）。
        用 QueuedConnection 排一個清理：Qt 同一執行緒的已排事件先進先出，它一定在那些舊事件之後跑。
        只清這幾台、而且清的時候已經又有機卡的不清（不會誤清新連上的別台）。"""
        d = self.__dict__
        pend = d.get('_servo_purge_sids')
        if pend is None or not sids:
            return
        pend.update(int(s) for s in sids)
        if not d.get('_servo_purge_posted'):
            d['_servo_purge_posted'] = True
            QMetaObject.invokeMethod(self, '_purge_removed_servo_cache', Qt.ConnectionType.QueuedConnection)

    @pyqtSlot()
    def _purge_removed_servo_cache(self) -> None:
        d = self.__dict__
        d['_servo_purge_posted'] = False
        sids, d['_servo_purge_sids'] = set(d.get('_servo_purge_sids') or ()), set()
        cards = d.get('_uav_cards') or {}
        for sid in sids:
            if sid not in cards:
                (d.get('_servo_cache') or {}).pop(sid, None)

    def remove_card(self, sysid: int) -> None:
        """移除單一 UAV 卡片（單獨斷線時呼叫）。

        若被移除者正是目前指令對象 → 指令對象自動復位為「全部」，
        避免指令繼續指向已離線的機。
        若它正是單機模式看的那台 ⇒ 變成「未選定」並**停住**（_single_hold）：不自動改看下一台，
        指令也送不出去（get_target_sysid() 回 NO_TARGET），直到使用者自己選（2026-09-02 教訓）。
        群控時不停住：下一幀自動接手（群控的「看哪台」不是指令對象）。
        """
        sysid = int(sysid)
        card = self._uav_cards.pop(sysid, None)
        if card is not None:
            card.setParent(None)
            card.deleteLater()
        if self._selected_sysid == sysid:
            # 不走 _set_target(None)：單機模式下那條會切回群控
            self._selected_sysid = None
        d = self.__dict__
        # 單機時這台還有未讀的嚴重訊息 ⇒ chip 要消失了，先留一行全隊警示（不然那則訊息沒人看見）
        self._note_unread_on_loss(sysid)
        for k in ('_last_frame', '_last_frame_t', '_servo_cache', '_unread_crit', '_unread_last', '_param_burst'):
            if isinstance(d.get(k), dict):
                d[k].pop(sysid, None)
        self._schedule_servo_purge([sysid])
        if d.get('_single_sysid') == sysid:
            d['_single_sysid'] = None
            if d.get('_mode') == self.MODE_SINGLE:
                # 停住：MSG 區繼續列這台的歷史（_view_sid），斷線那行也會列進來
                d['_single_hold'] = True
                d['_single_lost_sysid'] = sysid
            self._rebuild_single_selector()
            self._apply_single_view()
            d['_single_src'] = 'remove'
            self.single_uav_changed.emit(self.NO_TARGET)
        else:
            self._rebuild_single_selector()
        if hasattr(self, 'lbl_cmd_target'):
            self._update_target_label()

    # ── 單機選台（2026-10-22）────────────────────────────────────────
    def _rebuild_single_selector(self) -> None:
        """依目前的機卡（＝已連線、有遙測的 sysid）重建「機號」chip，依 sysid 排序。"""
        d = self.__dict__
        row = d.get('_single_chip_row')
        if row is None:
            return
        for b in d.get('_single_chips', {}).values():
            row.removeWidget(b)
            b.setParent(None)
            b.deleteLater()
        self._single_chips = {}
        for sid in sorted(self._uav_cards):
            b = self._chip(f'UAV{sid}', 'info', None, checkable=True)
            b.clicked.connect(lambda _=False, s=sid: self._on_single_chip_clicked(s))
            row.addWidget(b)
            self._single_chips[sid] = b
        self._sync_single_chips()
        self._refresh_single_empty()

    def _sync_single_chips(self) -> None:
        cur = self.__dict__.get('_single_sysid')
        for sid, b in self.__dict__.get('_single_chips', {}).items():
            b.blockSignals(True)
            b.setChecked(sid == cur)
            b.blockSignals(False)
            self._style_single_chip(sid)

    def _style_single_chip(self, sid) -> None:
        """未讀嚴重訊息 ⇒ chip 換 stop 色調＋「UAVn ●k」；選了它就清掉（set_single_sysid）。"""
        b = self.__dict__.get('_single_chips', {}).get(sid)
        if b is None:
            return
        n = int(self._unread_crit.get(sid, 0))
        text = f'UAV{sid} ●{n}' if n else f'UAV{sid}'
        tone = 'stop' if n else 'info'
        if b.text() != text:
            b.setText(text)
        if b.property('chipTone') != tone:
            b.setStyleSheet(self._chip_css(tone))
            b.setProperty('chipTone', tone)
        b.setToolTip(f'UAV{sid} 有 {n} 則未讀的嚴重訊息（點選查看）' if n
                     else f'單機模式看 UAV{sid}（指令也只送給它）')

    def _on_single_chip_clicked(self, sid: int) -> None:
        self.set_single_sysid(sid, source='ui')
        self._sync_single_chips()             # 點已選的那顆：Qt 已把它切成未勾 ⇒ 勾回來

    def _refresh_single_empty(self) -> None:
        """沒有選定的那台時：收起 HUD／DETAIL，顯示為什麼（還沒連線／選定的那台斷線）。"""
        d = self.__dict__
        lbl = d.get('lbl_single_empty')
        if lbl is None:
            return
        has = d.get('_single_sysid') is not None
        if not has:
            lost = d.get('_single_lost_sysid')
            if d.get('_single_hold') and lost is not None:
                txt = f'UAV{lost} 已斷線，請重新選擇（不會自動改看別台；指令暫時送不出去）'
            elif self._uav_cards:
                txt = '尚未選定（在上面選一台）'
            else:
                txt = '尚無連線的 UAV（連線後自動選第一台）'
            if lbl.text() != txt:
                lbl.setText(txt)
        lbl.setVisible(not has)
        for w in (d.get('single_hud'), d.get('_legacy_hud_frame'), d.get('lbl_single_link'),
                  d.get('_single_actions')):
            if w is not None:
                w.setVisible(has)
        det = d.get('_single_detail_frame')
        if det is not None:
            det.setVisible(has and d.get('_mode') == self.MODE_SINGLE)
        self._refresh_single_log_bar()

    def view_mode(self) -> str:
        return self.__dict__.get('_mode', self.MODE_SWARM)

    def single_sysid(self) -> Optional[int]:
        """單機模式看的（或切過去會看的）那一台；None ＝ 未選定。"""
        return self.__dict__.get('_single_sysid')

    def connected_sysids(self) -> list:
        return sorted(self._uav_cards)

    def set_view_mode(self, mode: str) -> bool:
        """切換「群控／單機」。實際有切換才回 True（並發 view_mode_changed 一次）。"""
        if mode not in (self.MODE_SWARM, self.MODE_SINGLE) or mode == self.view_mode():
            self._sync_mode_chips()
            return False
        self._mode = mode
        picked = None
        if mode == self.MODE_SINGLE:
            if self._single_sysid is None and self._uav_cards:
                # 切到單機是使用者的明確動作 ⇒ 還沒看過任何一台就看編號最小那台
                picked = self._single_sysid = min(self._uav_cards)
                self._unread_crit.pop(picked, None)
            if not self._servo_autoexpanded:
                self._servo_autoexpanded = True
                self._servo_visible = True
        else:
            self._single_hold = False
            self._single_lost_sysid = None
            self._unread_crit.clear()         # 群控的合併訊息看得到全部 ⇒ 未讀標記作廢
        self._apply_mode_visibility()
        self._update_servo_toggle_text()
        if mode == self.MODE_SINGLE:
            self._sync_single_chips()
            self._apply_single_view()
            self._single_age_timer.start()
        else:
            self._single_age_timer.stop()
            # 單機時機卡不更新 ⇒ 回群控先用每台最後一幀補畫一次；合併訊息從緩衝重畫
            for sid, card in self._uav_cards.items():
                f = self._last_frame.get(sid)
                if f is not None:
                    try:
                        card.update_from_frame(f)
                    except Exception:
                        pass
            self._flush_param_summaries(append=False)
            self._render_msg_view(self.log_view, self._msgs.merged(), True)
        self._update_target_label()
        self._sync_mode_chips()
        self.view_mode_changed.emit(mode)
        if mode == self.MODE_SINGLE and self._single_sysid is not None:
            # 進單機 ＝ 使用者明確選了「看這台、指令給這台」⇒ 通知一次（Command 分頁的 rail 跟著對齊；
            # 群控時 HUD 自動選的那台不帶 rail，見 main_window.sync_rail_to_hud）
            self.__dict__['_single_src'] = 'mode'
            self.single_uav_changed.emit(int(self._single_sysid))
        return True

    def set_single_sysid(self, sysid, source: str = 'ui') -> bool:
        """改看 sysid 這一台。沒變、或這台沒有機卡（還沒遙測）⇒ 不動、回 False。

        不碰群控的指令對象（_selected_sysid）；單機模式下它就是指令對象（見 get_target_sysid）。
        source='rail'（Command 分頁的機卡列）⇒ 在訊息區留一行，讓使用者知道對象是從別頁改的。
        """
        try:
            sid = int(sysid)
        except (TypeError, ValueError):
            return False
        d = self.__dict__
        if sid == d.get('_single_sysid') or sid not in self._uav_cards:
            return False
        d['_single_sysid'] = sid
        d['_single_hold'] = False
        d['_single_lost_sysid'] = None
        self._unread_crit.pop(sid, None)
        self._sync_single_chips()
        if self.view_mode() == self.MODE_SINGLE:
            self._apply_single_view()
            self._update_target_label()
            if source == 'rail':
                self._append_msg(None, None, f'單機對象改為 UAV{sid}（來自 Command 分頁）', 'local',
                                 color=TC.NEUTRAL)
        d['_single_src'] = str(source)
        self.single_uav_changed.emit(sid)
        return True

    def single_change_source(self) -> str:
        """最近一次 single_uav_changed 的來源：'ui'／'rail'／'auto'（第一台遙測自動選）／'mode'（切到單機）／
        'remove'（斷線清掉）。main_window.sync_rail_to_hud 用它：群控時自動選的那台不帶 Command 分頁的 rail。"""
        return str(self.__dict__.get('_single_src', ''))

    def _apply_mode_visibility(self) -> None:
        single = self.view_mode() == self.MODE_SINGLE
        self._cards_scroll.setVisible(not single)
        self.log_view.setVisible(not single)
        self._single_frame.setVisible(single)
        self._single_msg_frame.setVisible(single)
        self._btn_servo_toggle.setVisible(single)
        self.servo_panel.setVisible(single and bool(self._servo_visible))
        # DETAIL：單機而且有選定那台才顯示（沒選 ⇒ 一張空表沒意義，見 _refresh_single_empty）
        self._single_detail_frame.setVisible(single and self.__dict__.get('_single_sysid') is not None)

    def _sync_mode_chips(self) -> None:
        """模式 chip 的勾選與圖示色（勾選時整顆填色 ⇒ 圖示改深色，不然會跟底色同色看不見）。"""
        d = self.__dict__
        mode = self.view_mode()
        info = self._tone_color('info')
        for name, m, icon in (('btn_mode_swarm', self.MODE_SWARM, 'radar'),
                              ('btn_mode_single', self.MODE_SINGLE, 'drone')):
            b = d.get(name)
            if b is None:
                continue
            on = (mode == m)
            if on and not b.isChecked():
                b.blockSignals(True)
                b.setChecked(True)            # 互斥群組：勾這顆會自動取消另一顆
                b.blockSignals(False)
            try:
                b.setIcon(get_icon(icon, color=(TC.BG_PRIMARY if on else info), size=12))
            except Exception:                 # noqa: BLE001 — 沒圖示就只剩字
                pass

    def _apply_single_view(self) -> None:
        """單機區整個換成目前選定那台：HUD、DETAIL、SERVO OUTPUT、訊息、遙測新鮮度。群控時不做事。"""
        d = self.__dict__
        if self.view_mode() != self.MODE_SINGLE:
            return
        sid = d.get('_single_sysid')
        self._refresh_single_empty()
        f = self._last_frame.get(sid) if sid is not None else None
        if f is not None:
            try:
                self.single_hud.invalidate_slow()
                self.single_hud.update_from_frame(f)
                # 換對象一律重畫一次 DETAIL（頁面沒顯示也要）：不然切回來先看到的是上一台的數值
                self._on_telemetry_legacy(f)
            except Exception:
                pass
        self._reload_servo_panel()
        # 訊息：選定那台；選定那台斷線（停住）⇒ 照樣列**斷線那台**的歷史（斷線前的飛控訊息正是要看的；
        # 以前這裡變成只剩本機訊息、連它自己的斷線那行都被濾掉；2026-10-22 審查）
        self._flush_param_summaries(append=False)
        self._render_msg_view(self.single_log_view, self._msgs.for_uav(self._view_sid()), False)
        self._refresh_single_link_line()
        self._update_servo_toggle_text()

    def _reload_servo_panel(self) -> None:
        """SERVO OUTPUT 換成選定那台：先 reset（沒讀回的通道顯示「—」），再灌它的 SERVOn_* 快取與最後一幀 PWM。"""
        d = self.__dict__
        if self.view_mode() != self.MODE_SINGLE or not d.get('_servo_visible'):
            return
        panel = self.servo_panel
        panel.reset()
        sid = d.get('_single_sysid')
        if sid is None:
            panel.set_source(None, 0)
            return
        cache = self._servo_cache.get(sid, {})
        for (ch, key), val in sorted(cache.items()):
            panel.on_servo_param(ch, key, val)
        f = self._last_frame.get(sid)
        if f is not None:
            panel.on_telemetry(f)
        panel.set_source(sid, len(cache))

    def _update_servo_toggle_text(self) -> None:
        arrow = '▼' if self._servo_visible else '▶'
        sid = self.__dict__.get('_single_sysid')
        # 有機號時不再重複「(16 通道 PWM)」（面板標題已寫）：按鈕的最小寬就是字寬，太長會撐出水平捲軸
        tail = f'  UAV{sid}' if sid is not None else '  (16 通道 PWM)'
        self._btn_servo_toggle.setText(f'{arrow} SERVO OUTPUT{tail}')

    def _refresh_single_link_line(self) -> None:
        """「UAVn · 遙測 <1 s 前 · ACTIVE · PLANE」；> 3 s 琥珀、> 10 s 紅＋STALE。文字／顏色沒變就不 setText。"""
        d = self.__dict__
        lbl = d.get('lbl_single_link')
        if lbl is None:
            return
        sid = d.get('_single_sysid')
        f = d.get('_last_frame', {}).get(sid) if sid is not None else None
        if f is None:
            text, color = '', TC.FG_SECONDARY
        else:
            age = max(0.0, _time.monotonic() - d.get('_last_frame_t', {}).get(sid, _time.monotonic()))
            parts = [f'UAV{sid}', f'遙測 {"<1" if age < 1.0 else f"{age:.0f}"} s 前',
                     str(getattr(f, 'system_status', '') or '—'), str(getattr(f, 'vehicle_type', '') or '—')]
            if age > 10.0:
                parts.append('STALE')
                color = TC.HOSTILE
            elif age > 3.0:
                color = TC.WARNING
            else:
                color = TC.FG_SECONDARY
            text = ' · '.join(parts)
        state = (text, color)
        if state == d.get('_link_line_state'):
            return
        d['_link_line_state'] = state
        lbl.setText(text)
        lbl.setStyleSheet(f'color:{color};font-size:10px;font-family:{TF.css_sans()};')

    # ── 指令對象（單機 / 全部）選取 ──────────────────────────────
    def _on_card_selected(self, sysid: int) -> None:
        """卡片被點選：再次點同一台 → 取消（回全部）；否則選取該台（單機模式「看哪台」也跟著換）。"""
        new_target = None if self._selected_sysid == sysid else sysid
        self._set_target(new_target)
        if new_target is not None:
            self.set_single_sysid(new_target)

    def _set_target(self, sysid: Optional[int]) -> None:
        """設定指令對象 sysid（None = 全部），同步所有卡片高亮與標籤。

        單機模式下要「全部」（main_window 的「改成全部 UAV」）⇒ 先明確切回群控（畫面看得到），
        不在單機模式裡偷偷廣播。
        """
        if sysid is None and self.view_mode() == self.MODE_SINGLE:
            self.set_view_mode(self.MODE_SWARM)
        self._selected_sysid = sysid
        for sid, card in self._uav_cards.items():
            card.set_selected(sid == sysid)
        self._update_target_label()

    def _update_target_label(self) -> None:
        """更新「指令對象」標籤文字與語意色。

        全部 → FG_EMPHASIS 琥珀；單機 → NEUTRAL 青（advisory/mode，非告警語意，
        符合 MIL-STD-1472H TABLE XL），並加底色框出，與右側「全部」鈕成對。
        單機模式：對象就是選定看的那台（「UAVn（單機模式）」）；還沒選 ⇒ 琥珀「未選定」，
        「全部」鈕停用（要對全部下指令請切回群控）。
        """
        btn = self.__dict__.get('btn_target_all')
        if self.view_mode() == self.MODE_SINGLE:
            sid = self.__dict__.get('_single_sysid')
            if sid is None:
                lost = self.__dict__.get('_single_lost_sysid') if self.__dict__.get('_single_hold') else None
                self.lbl_cmd_target.setText(f'UAV{lost} 已斷線（單機模式）' if lost is not None
                                            else '未選定（單機模式）')
                self.lbl_cmd_target.setStyleSheet(
                    f'color:{TC.WARNING};font-size:11px;font-weight:600;'
                    f'font-family:{TF.css_sans()};'
                )
            else:
                self.lbl_cmd_target.setText(f'UAV{sid}（單機模式）')
                self.lbl_cmd_target.setStyleSheet(
                    f'color:{TC.NEUTRAL};font-size:11px;font-weight:600;'
                    f'font-family:{TF.css_sans()};'
                    f'background:{TC.BG_ELEVATED};padding:1px 6px;'
                )
            if btn is not None:
                btn.setEnabled(False)
                btn.setToolTip(self.TARGET_ALL_SINGLE_TIP)
            return
        if btn is not None and not btn.isEnabled():
            btn.setEnabled(True)
            btn.setToolTip(self.TARGET_ALL_TIP)
        if self._selected_sysid is None:
            self.lbl_cmd_target.setText('全部 UAV')
            self.lbl_cmd_target.setStyleSheet(
                f'color:{TC.FG_EMPHASIS};font-size:11px;font-weight:600;'
                f'font-family:{TF.css_sans()};'
            )
        else:
            self.lbl_cmd_target.setText(f'UAV{self._selected_sysid}（單獨）')
            self.lbl_cmd_target.setStyleSheet(
                f'color:{TC.NEUTRAL};font-size:11px;font-weight:600;'
                f'font-family:{TF.css_sans()};'
                f'background:{TC.BG_ELEVATED};padding:1px 6px;'
            )

    def get_target_sysid(self) -> Optional[int]:
        """目前指令對象 sysid；None 代表「全部 UAV」。

        供 main_window._sitl_broadcast / 上傳處理判斷指令派送範圍使用。
        單機模式：回選定看的那台；還沒選 ⇒ NO_TARGET（-1，對不上任何鏈路 ⇒ 指令送不出去，fail-closed）。
        """
        d = self.__dict__
        if d.get('_mode') == self.MODE_SINGLE:
            sid = d.get('_single_sysid')
            return self.NO_TARGET if sid is None else sid
        return self._selected_sysid

    def on_telemetry(self, frame):
        """一幀遙測（main_window 合幀後、每架 ≤ 15 Hz）。

        群控：只更新機卡。單機：機卡只建不更新；只有選定那台更新大 HUD／SERVO（展開時）／DETAIL（看得到時）。
        """
        try:
            sid = int(frame.sysid)
        except Exception:
            return None
        d = self.__dict__
        lf = d.get('_last_frame')
        if lf is not None:
            lf[sid] = frame
            d['_last_frame_t'][sid] = _time.monotonic()
        # 機卡永遠要建（它也是「已連線 sysid」的清單，單機選台 chip 從這裡來）
        try:
            card = self._get_or_create_card(sid)
        except Exception:
            card = None
        # 沒選過任何一台 ⇒ 預設看第一台有遙測的；選定的那台斷線（_single_hold）⇒ 等使用者選
        if d.get('_single_sysid') is None and not d.get('_single_hold') and '_mode' in d:
            self.set_single_sysid(sid, source='auto')
        mode = d.get('_mode', self.MODE_SWARM)
        if mode == self.MODE_SWARM and card is not None:
            try:
                card.update_from_frame(frame)
            except Exception:
                pass
        if '_mode' in d and sid != d.get('_single_sysid'):
            return None
        if mode == self.MODE_SINGLE:
            try:
                self.single_hud.update_from_frame(frame)
                # SERVO OUTPUT：摺起時 16 條 PWM 條圖不用更新（展開時 _reload_servo_panel 會補最後一幀）
                if d.get('_servo_visible'):
                    self.servo_panel.on_telemetry(frame)
            except Exception:
                pass
        # DETAIL（舊版單機框）只在真的顯示時才跑：14 次 setText ＋ 3 次 setStyleSheet 每幀打在看不見的
        # label 上，6 架時是每秒上百次樣式重算（2026-10-17 效能）。它在單機區裡 ⇒ 群控時一定看不見。
        legacy = d.get('_legacy_hud_frame')
        if legacy is not None and not legacy.isVisible():
            return None
        return self._on_telemetry_legacy(frame)

    def on_servo_param(self, ch: int, key: str, value: float):
        """舊介面（不帶 sysid）：直接轉發到 ServoOutputPanel。main_window 已改接 on_servo_param_sysid。"""
        if hasattr(self, 'servo_panel'):
            self.servo_panel.on_servo_param(ch, key, value)

    def on_servo_param_sysid(self, sysid: int, ch: int, key: str, value: float) -> None:
        """SITLLink.servo_param_sysid：依 sysid 快取 SERVOn_*；只有單機模式、正在看這台、面板展開時才寫面板。"""
        try:
            sid = int(sysid)
        except (TypeError, ValueError):
            return
        if sid <= 0:
            return
        cache = self._servo_cache.setdefault(sid, {})
        cache[(int(ch), str(key))] = float(value)
        d = self.__dict__
        if d.get('_mode') == self.MODE_SINGLE and sid == d.get('_single_sysid') and d.get('_servo_visible'):
            self.servo_panel.on_servo_param(int(ch), str(key), float(value))
            self.servo_panel.set_source(sid, len(cache))

    def _toggle_servo_panel(self):
        """開合 SERVO OUTPUT 面板（只在單機模式顯示；展開時換成選定那台的內容）。"""
        self._servo_visible = not self._servo_visible
        self.servo_panel.setVisible(self._servo_visible and self.view_mode() == self.MODE_SINGLE)
        self._update_servo_toggle_text()
        if self._servo_visible:
            self._reload_servo_panel()

    @staticmethod
    def _set_css(lbl, css: str) -> None:
        """樣式沒變就不設：setStyleSheet 每次都會重新 polish（單機 DETAIL 每幀 3 次，2026-10-22 效能）。"""
        if lbl.styleSheet() != css:
            lbl.setStyleSheet(css)

    def _on_telemetry_legacy(self, frame):
        """frame: TelemetryFrame"""
        # 模式 / 武裝 / 飛行器
        self.lbl_mode.setText(frame.mode)
        if frame.armed:
            self.lbl_armed.setText('ARMED')
            self._set_css(self.lbl_armed,
                          f'color:{TC.HOSTILE};font-size:12px;font-weight:700;'
                          f'font-family:{TF.css_mono()};')
        else:
            self.lbl_armed.setText('DISARMED')
            self._set_css(self.lbl_armed,
                          f'color:{TC.FG_MUTED};font-size:12px;font-weight:700;'
                          'font-family:"Consolas",monospace;')
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
        self._set_css(self.lbl_gps, f'color:{gps_color};font-size:11px;')

        # 電量
        if frame.battery_pct >= 0:
            bat_color = '#ff5252' if frame.battery_pct < 20 else '#ffb74d' if frame.battery_pct < 50 else '#aed581'
            self.lbl_bat.setText(f'🔋 {frame.battery_v:.1f}V · {frame.battery_pct}% · {frame.battery_a:.1f}A')
        else:
            bat_color = '#ffb74d'
            self.lbl_bat.setText(f'🔋 {frame.battery_v:.1f}V · {frame.battery_a:.1f}A')
        self._set_css(self.lbl_bat, f'color:{bat_color};font-size:11px;')

        # 座標
        self.lbl_pos.setText(f'📍 {frame.lat:.6f}, {frame.lon:.6f}')
