"""
Servo Output Panel — Mission Planner 風格的 16 通道 PWM 輸出表格

依 MIL-STD-1472H 配色（TacticalColors）顯示 ArduPilot SITL/實機的 SERVO_OUTPUT_RAW，
搭配從 PARAM_VALUE 解析出的 SERVO{n}_FUNCTION/MIN/TRIM/MAX/REVERSED 即時更新。

每行包含：
    # | Position bar | Reverse | Function       | Min | Trim | Max
    1 | [████ 1505 ] |   ☐    | Aileron        | 1000| 1505 | 2000
    2 | [██   1417 ] |   ☑    | Elevator       | 1000| 1461 | 2000
    ...

PWM bar 用 QPainter 自繪，依 PWM 值在 Min~Max 範圍內以 FRIENDLY 綠填充；
Disabled 通道（FUNCTION=0 或 PWM=0）顯示為 BG_ELEVATED 灰底。
"""

from __future__ import annotations

from typing import Optional

from PyQt6.QtCore import Qt, pyqtSignal, QRectF
from PyQt6.QtGui import QPainter, QColor, QPen, QFont, QFontMetrics
from PyQt6.QtWidgets import (
    QWidget, QFrame, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QCheckBox, QComboBox, QSpinBox, QSizePolicy,
)

from ui.resources.tactical_theme import TacticalColors as TC, TacticalFonts as TF
from mission.sitl_link import SERVO_FUNCTIONS, servo_function_name


# ══════════════════════════════════════════════════════════════════════
#  PwmBar — 自繪 PWM 條圖
# ══════════════════════════════════════════════════════════════════════
class PwmBar(QWidget):
    """顯示 PWM 在 [min, max] 區間中的位置；中央顯示數值。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._pwm = 0
        self._min = 1000
        self._max = 2000
        self._enabled_ch = True
        self.setMinimumSize(120, 22)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

    def set_pwm(self, pwm: int):
        if pwm != self._pwm:
            self._pwm = int(pwm)
            self.update()

    def set_range(self, pmin: int, pmax: int):
        self._min, self._max = int(pmin), int(pmax)
        self.update()

    def set_enabled_channel(self, en: bool):
        self._enabled_ch = bool(en)
        self.update()

    def paintEvent(self, _evt):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        rect = self.rect().adjusted(0, 1, -1, -1)

        # 背景（凹陷層）
        p.fillRect(rect, QColor(TC.BG_SUNKEN))

        # 填充比例
        if self._max > self._min and self._pwm > 0:
            ratio = (self._pwm - self._min) / (self._max - self._min)
            ratio = max(0.0, min(1.0, ratio))
            fill_w = int(rect.width() * ratio)
            if fill_w > 0:
                fill_rect = QRectF(rect.left(), rect.top(), fill_w, rect.height())
                color = QColor(TC.FRIENDLY) if self._enabled_ch else QColor(TC.FG_MUTED)
                p.fillRect(fill_rect, color)

        # 邊框
        p.setPen(QPen(QColor(TC.BORDER_DEFAULT), 1))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRect(rect)

        # 中央 PWM 文字
        font = TF.mono(9, bold=True)
        p.setFont(font)
        text = f'{self._pwm}' if self._pwm > 0 else '---'
        # 文字色：在綠色填充上用深底色，灰色填充上用主色
        p.setPen(QPen(QColor(TC.BG_PRIMARY) if self._enabled_ch and self._pwm > 0
                      else QColor(TC.FG_PRIMARY)))
        p.drawText(rect, Qt.AlignmentFlag.AlignCenter, text)
        p.end()


# ══════════════════════════════════════════════════════════════════════
#  ServoChannelRow — 單一通道列
# ══════════════════════════════════════════════════════════════════════
class ServoChannelRow(QWidget):
    """
    一個 servo 通道列：通道號 / PWM bar / Reverse / Function / Min / Trim / Max。
    所有控制均為「顯示」用，邊改邊發 param_set 由外層 ServoOutputPanel 處理。
    """

    param_change_requested = pyqtSignal(int, str, float)
    # 參數編輯權限（false 為唯讀，避免使用者誤改 SITL 參數）
    EDITABLE = False

    def __init__(self, ch: int, parent=None):
        super().__init__(parent)
        self.ch = ch
        self._init_ui()

    def _init_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(2, 0, 2, 0)
        layout.setSpacing(6)

        # 通道號
        self.lbl_ch = QLabel(f'{self.ch}')
        self.lbl_ch.setFixedWidth(20)
        self.lbl_ch.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_ch.setStyleSheet(
            f'color:{TC.FG_SECONDARY};font-size:10px;'
            f'font-family:{TF.css_mono()};font-weight:bold;'
        )
        layout.addWidget(self.lbl_ch)

        # PWM bar
        self.bar = PwmBar()
        self.bar.setFixedWidth(140)
        layout.addWidget(self.bar)

        # Reverse 顯示（唯讀 checkbox 樣式）
        self.lbl_rev = QLabel('☐')
        self.lbl_rev.setFixedWidth(20)
        self.lbl_rev.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_rev.setStyleSheet(
            f'color:{TC.FG_MUTED};font-size:14px;'
        )
        self.lbl_rev.setToolTip('SERVOn_REVERSED')
        layout.addWidget(self.lbl_rev)

        # Function 名稱
        self.lbl_func = QLabel('Disabled')
        self.lbl_func.setFixedWidth(120)
        self.lbl_func.setStyleSheet(
            f'background:{TC.BG_SUNKEN};color:{TC.FG_PRIMARY};'
            f'border:1px solid {TC.BORDER_DEFAULT};border-radius:0;'
            f'padding:2px 6px;font-size:10px;'
            f'font-family:{TF.css_condensed()};letter-spacing:0.5px;'
        )
        layout.addWidget(self.lbl_func)

        # Min / Trim / Max（QSpinBox 樣式但唯讀）
        for label, name in [('Min', 'min'), ('Trim', 'trim'), ('Max', 'max')]:
            sb = QSpinBox()
            sb.setRange(800, 2200)
            sb.setValue({'min': 1000, 'trim': 1500, 'max': 2000}[name])
            sb.setFixedWidth(56)
            sb.setReadOnly(not self.EDITABLE)
            sb.setButtonSymbols(QSpinBox.ButtonSymbols.NoButtons if not self.EDITABLE
                                  else QSpinBox.ButtonSymbols.UpDownArrows)
            sb.setStyleSheet(
                f'QSpinBox{{background:{TC.BG_SUNKEN};color:{TC.FG_PRIMARY};'
                f'border:1px solid {TC.BORDER_DEFAULT};border-radius:0;'
                f'padding:1px 4px;font-size:10px;'
                f'font-family:{TF.css_mono()};}}'
            )
            setattr(self, f'sb_{name}', sb)
            layout.addWidget(sb)

        layout.addStretch(1)
        self.setFixedHeight(24)

    # ── 公開資料寫入 ─────────────────────────────────────────────
    def set_pwm(self, pwm: int):
        self.bar.set_pwm(pwm)

    def set_function(self, func_code: int):
        name = servo_function_name(func_code)
        self.lbl_func.setText(name)
        is_disabled = (func_code == 0)
        self.bar.set_enabled_channel(not is_disabled)
        # 停用通道整列文字降亮度
        self.lbl_func.setStyleSheet(
            f'background:{TC.BG_SUNKEN};'
            f'color:{TC.FG_MUTED if is_disabled else TC.FG_PRIMARY};'
            f'border:1px solid {TC.BORDER_DEFAULT};border-radius:0;'
            f'padding:2px 6px;font-size:10px;'
            f'font-family:{TF.css_condensed()};letter-spacing:0.5px;'
        )

    def set_min_trim_max(self, pmin: int = None, ptrim: int = None, pmax: int = None):
        if pmin is not None:
            self.sb_min.setValue(int(pmin))
        if ptrim is not None:
            self.sb_trim.setValue(int(ptrim))
        if pmax is not None:
            self.sb_max.setValue(int(pmax))
        # 同步 PWM bar 範圍（取 min/max）
        self.bar.set_range(self.sb_min.value(), self.sb_max.value())

    def set_reversed(self, rev: bool):
        if rev:
            self.lbl_rev.setText('☑')
            self.lbl_rev.setStyleSheet(
                f'color:{TC.NEUTRAL};font-size:14px;font-weight:bold;'
            )
        else:
            self.lbl_rev.setText('☐')
            self.lbl_rev.setStyleSheet(
                f'color:{TC.FG_MUTED};font-size:14px;'
            )


# ══════════════════════════════════════════════════════════════════════
#  ServoOutputPanel — 16 通道 PWM 表格主面板
# ══════════════════════════════════════════════════════════════════════
class ServoOutputPanel(QFrame):
    """
    Mission Planner 風格 PWM 輸出面板。

    用法：
        panel = ServoOutputPanel()
        sitl_link.telemetry.connect(panel.on_telemetry)        # 灌入 SERVO_OUTPUT_RAW
        sitl_link.servo_param.connect(panel.on_servo_param)    # 灌入 SERVOn_*
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(
            f'QFrame{{background:{TC.BG_PRIMARY};'
            f'border:1px solid {TC.BORDER_DEFAULT};border-radius:0;}}'
            f'QLabel{{background:transparent;border:none;}}'
        )
        self._init_ui()

    def _init_ui(self):
        v = QVBoxLayout(self)
        v.setContentsMargins(6, 6, 6, 6)
        v.setSpacing(0)

        # 標題列
        title = QLabel('▎SERVO OUTPUT  —  16 通道 PWM (μs)')
        title.setStyleSheet(
            f'color:{TC.FG_EMPHASIS};'
            f'font-family:{TF.css_condensed()};letter-spacing:1.5px;'
            f'font-size:11px;font-weight:bold;padding:2px 0 4px 2px;'
        )
        v.addWidget(title)

        # 表頭（與 ServoChannelRow 對齊）
        header = QHBoxLayout()
        header.setContentsMargins(2, 0, 2, 0)
        header.setSpacing(6)
        # §5.17.20.3.10 — units 必須包含於 column label
        for txt, w, align in [
            ('#',          20,  Qt.AlignmentFlag.AlignCenter),
            ('PWM (μs)',  140, Qt.AlignmentFlag.AlignCenter),
            ('Rev',        20,  Qt.AlignmentFlag.AlignCenter),
            ('Function',  120, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
            ('Min (μs)',   56,  Qt.AlignmentFlag.AlignCenter),
            ('Trim (μs)',  56,  Qt.AlignmentFlag.AlignCenter),
            ('Max (μs)',   56,  Qt.AlignmentFlag.AlignCenter),
        ]:
            lbl = QLabel(txt)
            lbl.setFixedWidth(w)
            lbl.setAlignment(align)
            lbl.setStyleSheet(
                f'color:{TC.FG_SECONDARY};font-size:9px;font-weight:bold;'
                f'font-family:{TF.css_condensed()};letter-spacing:1.2px;'
                f'border-bottom:1px solid {TC.BORDER_SUBTLE};padding:0 0 2px 0;'
            )
            header.addWidget(lbl)
        header.addStretch(1)
        head_w = QWidget()
        head_w.setLayout(header)
        head_w.setFixedHeight(18)
        v.addWidget(head_w)

        # 16 個通道列；§5.17.20.3.14 — 每 ≤5 列要有視覺分組分隔
        # 採用每 4 列一組（4-4-4-4 = 16）：
        #   1-4 = 主控制翼面 (Aileron/Elevator/Throttle/Rudder)
        #   5-8 = lift motors (VTOL)
        #   9-12 = 雲台 / 輔助
        #   13-16 = 預留
        self._rows: list[ServoChannelRow] = []
        for ch in range(1, 17):
            row = ServoChannelRow(ch, parent=self)
            v.addWidget(row)
            self._rows.append(row)
            # 每 4 個通道後加薄分隔線（除最後一組）
            if ch % 4 == 0 and ch < 16:
                sep = QFrame()
                sep.setFrameShape(QFrame.Shape.HLine)
                sep.setStyleSheet(
                    f'QFrame{{background:{TC.BORDER_SUBTLE};max-height:1px;border:none;}}'
                )
                sep.setFixedHeight(1)
                v.addWidget(sep)

        v.addStretch(1)

    # ── 信號接收 ─────────────────────────────────────────────────
    def on_telemetry(self, frame):
        """從 TelemetryFrame 取 servo_pwm，更新 16 通道條圖。"""
        if not hasattr(frame, 'servo_pwm') or not frame.servo_pwm:
            return
        for i, pwm in enumerate(frame.servo_pwm[:16]):
            self._rows[i].set_pwm(int(pwm))

    def on_servo_param(self, ch: int, key: str, value: float):
        """SITLLink.servo_param 信號處理（FUNCTION/MIN/TRIM/MAX/REVERSED）。"""
        if not (1 <= ch <= 16):
            return
        row = self._rows[ch - 1]
        if key == 'FUNCTION':
            row.set_function(int(value))
        elif key == 'MIN':
            row.set_min_trim_max(pmin=int(value))
        elif key == 'TRIM':
            row.set_min_trim_max(ptrim=int(value))
        elif key == 'MAX':
            row.set_min_trim_max(pmax=int(value))
        elif key == 'REVERSED':
            row.set_reversed(bool(value))

    # ── 手動測試用 ───────────────────────────────────────────────
    def set_demo_state(self):
        """填入範例樣式（VTOL QuadPlane）— 供無 SITL 連線時預覽。"""
        demo = [
            (1, 'FUNCTION', 4),    (1, 'MIN', 1000), (1, 'TRIM', 1505), (1, 'MAX', 2000),
            (2, 'FUNCTION', 19),   (2, 'MIN', 1000), (2, 'TRIM', 1461), (2, 'MAX', 2000),
                                   (2, 'REVERSED', 1),
            (3, 'FUNCTION', 70),   (3, 'MIN', 1000), (3, 'TRIM', 1000), (3, 'MAX', 2000),
            (4, 'FUNCTION', 21),   (4, 'MIN', 1000), (4, 'TRIM', 1500), (4, 'MAX', 2000),
            (5, 'FUNCTION', 33),   (5, 'MIN', 1000), (5, 'TRIM', 1500), (5, 'MAX', 2000),
            (6, 'FUNCTION', 34),   (6, 'MIN', 1000), (6, 'TRIM', 1500), (6, 'MAX', 2000),
            (7, 'FUNCTION', 35),   (7, 'MIN', 1100), (7, 'TRIM', 1500), (7, 'MAX', 1900),
            (8, 'FUNCTION', 36),   (8, 'MIN', 1100), (8, 'TRIM', 1500), (8, 'MAX', 1900),
            (9, 'FUNCTION', 6),    (10, 'FUNCTION', 7), (10, 'REVERSED', 1),
            (11, 'FUNCTION', 8),   (11, 'REVERSED', 1),
        ]
        for it in demo:
            ch, key, val = it
            self.on_servo_param(ch, key, float(val))
        # 模擬 PWM 值
        from mission.sitl_link import TelemetryFrame
        f = TelemetryFrame()
        f.servo_pwm = [1505, 1417, 1000, 1500, 1950, 1150, 1949, 1156,
                        0, 0, 0, 0, 0, 0, 0, 0]
        self.on_telemetry(f)
