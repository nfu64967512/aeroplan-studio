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

import math
from typing import Optional
from PyQt6.QtCore import Qt, pyqtSignal, QRectF, QPointF, QSize
from PyQt6.QtGui import QPainter, QColor, QPen, QPolygonF, QFontMetrics
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QLineEdit, QFrame, QComboBox, QScrollArea, QSizePolicy, QSpinBox,
)

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
        """一次性套用整個 TelemetryFrame。"""
        self.set_attitude(frame.roll, frame.pitch, frame.heading)
        self.set_flight(getattr(frame, 'air_speed', frame.ground_speed),
                         frame.alt_rel, frame.ground_speed,
                         getattr(frame, 'climb', 0.0), frame.throttle)
        self.set_status(frame.mode, frame.armed, frame.vehicle_type)
        self.set_health(frame.battery_pct, frame.battery_v,
                         frame.gps_sats, frame.gps_fix)
        self._yaw = float(frame.heading)
        self.update()

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
        font_value = TF.mono(9, bold=True)
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
        pb.addWidget(self.lbl_pos)
        pb.addStretch()
        v.addWidget(pos_bar)

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
    """SITL 連線 + HUD 顯示面板"""

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

    def __init__(self, parent=None):
        super().__init__(parent)
        self._connected = False
        self._sitl_running = False
        # 目前「指令對象」sysid；None = 全部 UAV（廣播）。
        # 由卡片點選設定，main_window._sitl_broadcast 會讀取以決定派送範圍。
        self._selected_sysid: Optional[int] = None
        self._init_ui()

    def _init_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        # ── 連線控制區 ─────────────────────────────────────
        ctrl_frame = QFrame()
        ctrl_frame.setStyleSheet(
            f'QFrame{{background:{TC.BG_SECONDARY};border:1px solid {TC.BORDER_DEFAULT};border-radius:0;}}'
        )
        ctrl = QHBoxLayout(ctrl_frame)
        ctrl.setContentsMargins(8, 6, 8, 6)
        ctrl.setSpacing(6)

        ctrl.addWidget(self._lbl('SITL:', TC.NEUTRAL, bold=True))

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
            f'QComboBox{{background:{TC.BG_SUNKEN};color:{TC.FG_PRIMARY};border:1px solid {TC.BORDER_DEFAULT};'
            f'border-radius:0;padding:3px 6px;font-size:11px;font-family:{TF.css_sans()};}}'
        )
        ctrl.addWidget(self.preset_combo, 1)

        # 連線按鈕 — IconButton('link') 走全域 btnPrimary tone；
        # 連線後動態切換為 'unlink' + btnDanger（見 on_connected / on_disconnected）
        self.btn_connect = IconButton('link', '連線', tone='primary', compact=True)
        self.btn_connect.clicked.connect(self._on_connect_clicked)
        ctrl.addWidget(self.btn_connect)

        # 嵌入式蜂群監看連線（模式 A1）— 不啟動內建 SITL，
        # 連到嵌入式 Jetson 原生 ArduPlane SITL 的 TCP server（N 機 TCP）。
        self.btn_embedded = IconButton('drone', '嵌入式蜂群', tone='neutral', compact=True)
        self.btn_embedded.setToolTip(
            '連線到嵌入式蜂群（Jetson 原生 ArduPlane SITL，TCP 監看）\n'
            '不啟動內建 SITL — 純監看 N 機遙測'
        )
        self.btn_embedded.clicked.connect(self.connect_embedded_requested.emit)
        ctrl.addWidget(self.btn_embedded)

        root.addWidget(ctrl_frame)

        # ── 內建 SITL 啟動列 ──────────────────────────────
        launch_frame = QFrame()
        launch_frame.setStyleSheet(
            f'QFrame{{background:{TC.BG_SECONDARY};border:1px solid {TC.BORDER_DEFAULT};border-radius:0;}}'
        )
        lrow = QHBoxLayout(launch_frame)
        lrow.setContentsMargins(8, 6, 8, 6)
        lrow.setSpacing(6)

        lrow.addWidget(self._lbl('內建 SITL:', TC.WARNING, bold=True))

        self.vehicle_combo = QComboBox()
        self.vehicle_combo.addItems(['PLANE (固定翼)', 'COPTER (多旋翼)', 'VTOL (垂直起降)'])
        self.vehicle_combo.setStyleSheet(
            f'QComboBox{{background:{TC.BG_SUNKEN};color:{TC.FG_PRIMARY};border:1px solid {TC.BORDER_DEFAULT};'
            f'border-radius:0;padding:3px 6px;font-size:11px;font-family:{TF.css_sans()};}}'
        )
        lrow.addWidget(self.vehicle_combo, 1)

        # 數量
        from PyQt6.QtWidgets import QSpinBox
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

        root.addWidget(launch_frame)

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
        root.addWidget(cards_scroll)

        # ── SERVO OUTPUT 面板（Mission Planner 風格 16 通道 PWM 表格）──
        from ui.widgets.servo_output_panel import ServoOutputPanel
        self.servo_panel = ServoOutputPanel()
        # 預設可摺疊（節省空間）；點按「▶ SERVO OUTPUT」開合
        self._servo_visible = False
        self._btn_servo_toggle = QPushButton('▶ SERVO OUTPUT  (16 通道 PWM)')
        self._btn_servo_toggle.setStyleSheet(
            f'QPushButton{{background:{TC.BG_SECONDARY};color:{TC.FG_EMPHASIS};'
            f'border:1px solid {TC.BORDER_DEFAULT};border-radius:0;padding:4px 8px;'
            f'font-size:11px;font-weight:bold;text-align:left;letter-spacing:1px;'
            f'font-family:{TF.css_condensed()};}}'
            f'QPushButton:hover{{background:{TC.BG_ELEVATED};}}'
        )
        self._btn_servo_toggle.clicked.connect(self._toggle_servo_panel)
        root.addWidget(self._btn_servo_toggle)
        self.servo_panel.setVisible(False)
        root.addWidget(self.servo_panel)

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

        root.addWidget(hud_frame)

        # ── 飛控指令區（ARM / 模式切換 / 上傳任務）─────────
        cmd_frame = QFrame()
        cmd_frame.setStyleSheet(
            f'QFrame{{background:{TC.BG_SECONDARY};border:1px solid {TC.BORDER_DEFAULT};border-radius:0;}}'
        )
        cmd_outer = QVBoxLayout(cmd_frame)
        cmd_outer.setContentsMargins(8, 6, 8, 6)
        cmd_outer.setSpacing(5)

        # 標題列：左「飛控指令」+ 右「指令對象」指示（全部 / 單機）
        cmd_header = QHBoxLayout()
        cmd_header.setSpacing(6)
        cmd_header.addWidget(self._lbl('飛控指令', TC.NEUTRAL, bold=True))
        cmd_header.addStretch()
        cmd_header.addWidget(self._lbl('指令對象:', TC.FG_SECONDARY))
        # 指令對象值標籤（全部 → 琥珀；單機 → NEUTRAL 青框，見 _update_target_label）
        self.lbl_cmd_target = self._lbl('全部 UAV', TC.FG_EMPHASIS, bold=True)
        cmd_header.addWidget(self.lbl_cmd_target)
        # 「全部」復位鈕：把指令對象切回全部 UAV
        self.btn_target_all = QPushButton('全部')
        self.btn_target_all.setFixedHeight(20)
        self.btn_target_all.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_target_all.setToolTip('指令對象切回「全部 UAV」（取消單機選取）')
        self.btn_target_all.setStyleSheet(
            f'QPushButton{{background:{TC.BG_SUNKEN};color:{TC.FG_SECONDARY};'
            f'border:1px solid {TC.BORDER_DEFAULT};border-radius:0;'
            f'padding:0 8px;font-size:10px;font-family:{TF.css_condensed()};'
            f'letter-spacing:1px;}}'
            f'QPushButton:hover{{background:{TC.BORDER_STRONG};color:{TC.FG_PRIMARY};}}'
        )
        self.btn_target_all.clicked.connect(lambda: self._set_target(None))
        cmd_header.addWidget(self.btn_target_all)
        cmd_outer.addLayout(cmd_header)

        # ARM/DISARM 列
        arm_row = QHBoxLayout()
        arm_row.setSpacing(4)
        self.btn_arm    = self._cmd_btn('🔓 ARM',    TC.FRIENDLY)
        self.btn_disarm = self._cmd_btn('🔒 DISARM', TC.HOSTILE)
        self.btn_takeoff= self._cmd_btn('🛫 起飛',    TC.NEUTRAL)
        self.btn_arm.clicked.connect(self.cmd_arm.emit)
        self.btn_disarm.clicked.connect(self.cmd_disarm.emit)
        self.btn_takeoff.clicked.connect(lambda: self.cmd_takeoff.emit(30.0))
        arm_row.addWidget(self.btn_arm)
        arm_row.addWidget(self.btn_disarm)
        arm_row.addWidget(self.btn_takeoff)
        cmd_outer.addLayout(arm_row)

        # ── 投放列：保險 + 一號彈 / 二號彈 ────────────────────────────
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
        cmd_outer.addLayout(rel_row)

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
        cmd_outer.addLayout(rel_cfg)

        # ── GUIDED 群飛指令列（固定翼相容）──────────────────────────
        # ArduPlane 純固定翼 GUIDED 不吃 NAV_TAKEOFF/SET_POSITION_TARGET，
        # 故起飛走 TAKEOFF 模式、飛到走 guided 航點（MISSION_ITEM current=2）。
        guided_row = QHBoxLayout()
        guided_row.setSpacing(4)
        self.btn_guided_takeoff = self._cmd_btn('🛫 GUIDED 起飛', TC.FRIENDLY)
        self.btn_guided_takeoff.setToolTip(
            '固定翼相容 GUIDED 起飛：切 TAKEOFF 模式滑跑爬升 + 自動 ARM（整群一次起飛）。')
        self.btn_guided_takeoff.clicked.connect(
            lambda: self.cmd_guided_takeoff.emit(60.0))
        guided_row.addWidget(self.btn_guided_takeoff)
        cmd_outer.addLayout(guided_row)

        # ── 點圖指令列：在 2D 或 3D 地圖點一下即下令（起飛 / 飛到）──────────
        # 兩者互斥（開一個自動關另一個）；2D Leaflet 與 3D Cesium 點擊皆會
        # 走到 main_window.on_corner_added，故同一攔截即同時支援兩種地圖。
        guided_row2 = QHBoxLayout()
        guided_row2.setSpacing(4)
        self.btn_guided_takeoff_pt = self._cmd_btn('🛫 點圖起飛', TC.FRIENDLY)
        self.btn_guided_takeoff_pt.setCheckable(True)
        self.btn_guided_takeoff_pt.setToolTip(
            '開啟後在 2D 或 3D 地圖點一下 → 長機帶領的分階段 GUIDED 起飛：\n'
            '長機單獨起飛 → 確認長機升空且進入 GUIDED → 僚機才跟著起飛、組 V 編隊。\n'
            '再按一次關閉。與「點圖飛到」互斥。')
        self.btn_guided_takeoff_pt.toggled.connect(self.cmd_guided_takeoff_mode.emit)
        self.btn_guided_goto = self._cmd_btn('🎯 點圖飛到', TC.WARNING)
        self.btn_guided_goto.setCheckable(True)
        self.btn_guided_goto.setToolTip(
            '開啟後在 2D 或 3D 地圖點一下 → 長機 GUIDED 飛往該點（guided 航點），\n'
            '僚機由蜂群節點自動跟隨成 V 字。再按一次關閉。與「點圖起飛」互斥。')
        self.btn_guided_goto.toggled.connect(self.cmd_guided_goto_mode.emit)
        guided_row2.addWidget(self.btn_guided_takeoff_pt)
        guided_row2.addWidget(self.btn_guided_goto)
        cmd_outer.addLayout(guided_row2)

        # 飛行模式列
        mode_row1 = QHBoxLayout()
        mode_row1.setSpacing(4)
        for mode in ['GUIDED', 'AUTO', 'LOITER']:
            b = self._cmd_btn(mode, TC.BG_SECONDARY)
            b.clicked.connect(lambda _, m=mode: self.cmd_set_mode.emit(m))
            mode_row1.addWidget(b)
        cmd_outer.addLayout(mode_row1)

        mode_row2 = QHBoxLayout()
        mode_row2.setSpacing(4)
        for mode in ['RTL', 'LAND', 'STABILIZE']:
            b = self._cmd_btn(mode, TC.BG_SECONDARY)
            b.clicked.connect(lambda _, m=mode: self.cmd_set_mode.emit(m))
            mode_row2.addWidget(b)
        cmd_outer.addLayout(mode_row2)

        # VTOL QuadPlane 模式列
        vtol_row = QHBoxLayout()
        vtol_row.setSpacing(4)
        # QuadPlane 飛行模式按鈕
        for mode in ['QHOVER', 'QLOITER', 'QLAND', 'QRTL']:
            b = self._cmd_btn(mode, TC.BG_SECONDARY)
            b.clicked.connect(lambda _, m=mode: self.cmd_set_mode.emit(m))
            vtol_row.addWidget(b)
        cmd_outer.addLayout(vtol_row)

        # VTOL 轉換按鈕列
        trans_row = QHBoxLayout()
        trans_row.setSpacing(4)
        self.btn_vtol_mc = self._cmd_btn('轉換為多旋翼 (MC Mode)', TC.NEUTRAL)
        self.btn_vtol_mc.setToolTip('MAV_CMD_DO_VTOL_TRANSITION → state=3 (MC)')
        self.btn_vtol_mc.clicked.connect(lambda: self.cmd_vtol_transition.emit(3))
        self.btn_vtol_fw = self._cmd_btn('轉換為固定翼 (FW Mode)', TC.NEUTRAL)
        self.btn_vtol_fw.setToolTip('MAV_CMD_DO_VTOL_TRANSITION → state=4 (FW)')
        self.btn_vtol_fw.clicked.connect(lambda: self.cmd_vtol_transition.emit(4))
        trans_row.addWidget(self.btn_vtol_mc)
        trans_row.addWidget(self.btn_vtol_fw)
        cmd_outer.addLayout(trans_row)

        # 上傳任務
        self.btn_upload = self._cmd_btn('📤 上傳當前任務到 SITL', TC.FG_EMPHASIS)
        self.btn_upload.clicked.connect(self.cmd_upload.emit)
        cmd_outer.addWidget(self.btn_upload)

        # 從飛控讀回任務（= Mission Planner 的 Ctrl+F → Read WPs）
        self.btn_download = self._cmd_btn('📥 從飛控讀回任務航線', TC.FG_EMPHASIS)
        self.btn_download.setToolTip(
            '向飛控索取目前存著的任務並畫到地圖上。\n'
            '看 AUTO 架構的預劃航線用（起飛 → 巡航 → 攻擊發起點決斷圈）。\n'
            '⚠️ 讀回來的是**飛控裡實際存著的那一份**，不是規劃器算出來的那份 —— '
            '兩者不同就是上傳沒生效。'
        )
        self.btn_download.clicked.connect(self.cmd_download.emit)
        cmd_outer.addWidget(self.btn_download)

        # ── Mission Planner 風格航點檔三兄弟：Load / Write / Save ──────
        # （Read WPs 就是上面剛接好的 btn_download，這裡補齊另外三顆）
        self.btn_load_wp = self._cmd_btn('📂 載入航點檔 (Load WP File)', TC.FG_EMPHASIS)
        self.btn_load_wp.setToolTip(
            '從 .waypoints（QGC WPL）或 .plan（QGroundControl）檔案載入一份任務，\n'
            '預覽到地圖上 —— 純讀檔 + 畫圖，不需要連線、也不會自動寫入飛控。\n'
            '載入時若檔案有格式或語意錯誤（如缺欄位、DO_JUMP 目標不存在），\n'
            '會擋下並列出原因；載入成功後才能按「✍️ 寫入飛控」。'
        )
        self.btn_load_wp.clicked.connect(self.cmd_load_wp.emit)
        cmd_outer.addWidget(self.btn_load_wp)

        self.btn_write_wp = self._cmd_btn('✍️ 寫入飛控 (Write WPs)', TC.HOSTILE)
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
        cmd_outer.addWidget(self.btn_write_wp)

        self.btn_save_wp = self._cmd_btn('💾 存成航點檔 (Save WP File)', TC.FG_EMPHASIS)
        self.btn_save_wp.setToolTip(
            '把「剛剛從飛控讀回」的那份任務（📥 從飛控讀回任務航線的結果）存成 .waypoints 檔。\n'
            '⚠️ 存的是**讀回結果**，不是尚未寫入飛控的載入檔 —— 兩者不同就是還沒寫入成功。\n'
            '尚未讀回任何任務時會說明原因，不會偷偷存成別的東西。'
        )
        self.btn_save_wp.clicked.connect(self.cmd_save_wp.emit)
        cmd_outer.addWidget(self.btn_save_wp)

        self.btn_upload_fence = self._cmd_btn('🛡️ 上傳電子圍籬', TC.WARNING)
        self.btn_upload_fence.setToolTip(
            'FENCE_ENABLE=1, FENCE_TYPE=7 (MaxAlt+Circle+Polygon),\n'
            'FENCE_ACTION=1 (RTL/QRTL)，4 頂點矩形 + 高度上下限'
        )
        self.btn_upload_fence.clicked.connect(self.cmd_upload_fence.emit)
        cmd_outer.addWidget(self.btn_upload_fence)

        self.btn_auto_start = self._cmd_btn('🚀 一鍵起飛 (AUTO+ARM+START)', TC.WARNING)
        self.btn_auto_start.clicked.connect(self.cmd_auto_start.emit)
        cmd_outer.addWidget(self.btn_auto_start)

        root.addWidget(cmd_frame)

        # ── 參數寫入區 ────────────────────────────────────
        param_frame = QFrame()
        param_frame.setStyleSheet(
            f'QFrame{{background:{TC.BG_SECONDARY};border:1px solid {TC.BORDER_DEFAULT};border-radius:0;}}'
            f'QLineEdit,QComboBox{{background:{TC.BG_SUNKEN};color:{TC.FG_PRIMARY};'
            f'border:1px solid {TC.BORDER_DEFAULT};border-radius:0;padding:2px 5px;'
            f'font-size:11px;font-family:{TF.css_sans()};}}'
        )
        pv = QVBoxLayout(param_frame)
        pv.setContentsMargins(8, 6, 8, 6)
        pv.setSpacing(5)
        pv.addWidget(self._lbl('參數寫入 (PARAM_SET)', TC.NEUTRAL, bold=True))

        # 常用參數預設按鈕（一鍵套用 SITL 友善設定）
        preset_row = QHBoxLayout()
        preset_row.setSpacing(4)
        btn_preset_sitl = self._cmd_btn('🛠 SITL 友善預設', TC.BG_SECONDARY)
        btn_preset_sitl.setToolTip(
            'ARMING_CHECK=0, FS_*=0, GPS 容忍寬鬆 — 方便模擬測試'
        )
        btn_preset_sitl.clicked.connect(self._on_preset_sitl_friendly)
        btn_preset_tune = self._cmd_btn('⚡ 速度/航點調校', TC.BG_SECONDARY)
        btn_preset_tune.setToolTip(
            'WPNAV_SPEED / AIRSPEED_CRUISE / WP_RADIUS 調整'
        )
        btn_preset_tune.clicked.connect(self._on_preset_speed_tune)
        btn_load_param = self._cmd_btn('📂 讀取參數檔', TC.BG_ELEVATED)
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
        btn_set = self._cmd_btn('📝 寫入', TC.NEUTRAL)
        btn_set.clicked.connect(self._on_param_set_clicked)
        btn_get = self._cmd_btn('🔍 讀取', TC.BG_SECONDARY)
        btn_get.clicked.connect(self._on_param_get_clicked)
        btn_row.addWidget(btn_set)
        btn_row.addWidget(btn_get)
        pv.addLayout(btn_row)

        # 常用參數下拉（點選即填入名稱）
        common_row = QHBoxLayout()
        common_row.setSpacing(4)
        common_row.addWidget(self._lbl('常用:', TC.FG_SECONDARY))
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
        # role="log" → 全域 QSS QPlainTextEdit[role="log"] 提供 log well 樣式
        self.log_view.setProperty('role', 'log')
        self.log_view.style().polish(self.log_view)
        self.log_view.setFixedHeight(140)
        root.addWidget(self.log_view)

        # 相容舊程式：lbl_status.setText / setStyleSheet → 轉為 append log
        class _LogShim:
            def __init__(self, view):
                self._view = view
                self._color = TC.FG_SECONDARY
            def setText(self, txt):
                import time as _t
                ts = _t.strftime('%H:%M:%S')
                self._view.appendHtml(
                    f'<span style="color:{TC.FG_MUTED}">{ts}</span> '
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

    def on_status_text(self, severity: int, text: str):
        # severity 0=EMERG ... 7=DEBUG
        color = TC.HOSTILE if severity <= 3 else TC.WARNING if severity <= 5 else TC.NEUTRAL
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
            card.selected.connect(self._on_card_selected)
            card.disconnect_requested.connect(self.uav_disconnect_requested.emit)
            # ── 依 sysid 數字大小插入（與連線順序無關）→ 卡片永遠按編號排列 ──
            # 插入索引 = 既有卡片中 sysid 比它小的數量（stretch 永遠在最後）。
            insert_idx = sum(1 for sid in self._uav_cards if sid < sysid)
            self._uav_cards[sysid] = card
            self._cards_row.insertWidget(insert_idx, card)
            # 套用目前選取高亮（新卡片預設未選，除非剛好等於目前指令對象）
            card.set_selected(sysid == self._selected_sysid)
        return card

    def clear_cards(self):
        for card in list(self._uav_cards.values()):
            card.setParent(None)
            card.deleteLater()
        self._uav_cards.clear()
        # 卡片清空 → 指令對象一併復位為「全部」
        self._selected_sysid = None
        if hasattr(self, 'lbl_cmd_target'):
            self._update_target_label()

    def remove_card(self, sysid: int) -> None:
        """移除單一 UAV 卡片（單獨斷線時呼叫）。

        若被移除者正是目前指令對象 → 指令對象自動復位為「全部」，
        避免指令繼續指向已離線的機。
        """
        card = self._uav_cards.pop(sysid, None)
        if card is not None:
            card.setParent(None)
            card.deleteLater()
        if self._selected_sysid == sysid:
            self._set_target(None)

    # ── 指令對象（單機 / 全部）選取 ──────────────────────────────
    def _on_card_selected(self, sysid: int) -> None:
        """卡片被點選：再次點同一台 → 取消（回全部）；否則選取該台。"""
        new_target = None if self._selected_sysid == sysid else sysid
        self._set_target(new_target)

    def _set_target(self, sysid: Optional[int]) -> None:
        """設定指令對象 sysid（None = 全部），同步所有卡片高亮與標籤。"""
        self._selected_sysid = sysid
        for sid, card in self._uav_cards.items():
            card.set_selected(sid == sysid)
        self._update_target_label()

    def _update_target_label(self) -> None:
        """更新「指令對象」標籤文字與語意色。

        全部 → FG_EMPHASIS 琥珀；單機 → NEUTRAL 青（advisory/mode，非告警語意，
        符合 MIL-STD-1472H TABLE XL），並加底色框出，與右側「全部」鈕成對。
        """
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
        """
        return self._selected_sysid

    def on_telemetry(self, frame):
        # 多機：每個 sysid 一個卡片
        try:
            card = self._get_or_create_card(int(frame.sysid))
            card.update_from_frame(frame)
        except Exception:
            pass
        # SERVO OUTPUT 面板（只取主機 sysid==1，避免多機混疊）
        try:
            if int(getattr(frame, 'sysid', 1)) == 1 and hasattr(self, 'servo_panel'):
                self.servo_panel.on_telemetry(frame)
        except Exception:
            pass
        # 仍保留舊版單機路徑（外部 reset 用）
        return self._on_telemetry_legacy(frame)

    def on_servo_param(self, ch: int, key: str, value: float):
        """SITLLink.servo_param 信號 → 轉發到 ServoOutputPanel。"""
        if hasattr(self, 'servo_panel'):
            self.servo_panel.on_servo_param(ch, key, value)

    def _toggle_servo_panel(self):
        """開合 SERVO OUTPUT 面板。"""
        self._servo_visible = not self._servo_visible
        self.servo_panel.setVisible(self._servo_visible)
        arrow = '▼' if self._servo_visible else '▶'
        self._btn_servo_toggle.setText(f'{arrow} SERVO OUTPUT  (16 通道 PWM)')

    def _on_telemetry_legacy(self, frame):
        """frame: TelemetryFrame"""
        # 模式 / 武裝 / 飛行器
        self.lbl_mode.setText(frame.mode)
        if frame.armed:
            self.lbl_armed.setText('ARMED')
            self.lbl_armed.setStyleSheet(
                f'color:{TC.HOSTILE};font-size:12px;font-weight:700;'
                f'font-family:{TF.css_mono()};'
            )
        else:
            self.lbl_armed.setText('DISARMED')
            self.lbl_armed.setStyleSheet(
                f'color:{TC.FG_MUTED};font-size:12px;font-weight:700;'
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
