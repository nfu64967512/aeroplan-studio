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


from PyQt6.QtCore import Qt, pyqtSignal, QRectF
from PyQt6.QtGui import QPainter, QColor, QPen
from PyQt6.QtWidgets import (
    QWidget, QFrame, QVBoxLayout, QHBoxLayout, QLabel,
    QSpinBox, QSizePolicy,
)

from ui.resources.tactical_theme import TacticalColors as TC, TacticalFonts as TF
from mission.sitl_link import servo_function_name


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
        fill_w = 0
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

        # 中央 PWM 文字：填色那一段用深色字、沒填到的那一段用主色字（分段裁切）——
        # 以前整串都用深色，填充沒蓋到中央時（例如 1100 μs）數字是黑底黑字看不見（2026-10-22）。
        # 10pt：Cascadia Mono Bold 9pt 的「5」在螢幕上跟「6」幾乎一樣（1150 看成 1160；2026-10-22 截圖查到）
        font = TF.mono(10, bold=True)
        p.setFont(font)
        text = f'{self._pwm}' if self._pwm > 0 else '---'
        rf = QRectF(rect)
        filled = QRectF(rf.left(), rf.top(), float(fill_w), rf.height())
        empty = QRectF(rf.left() + fill_w, rf.top(), rf.width() - fill_w, rf.height())
        for clip, col in ((empty, TC.FG_PRIMARY),
                          (filled, TC.BG_PRIMARY if self._enabled_ch else TC.FG_PRIMARY)):
            if clip.width() <= 0:
                continue
            p.save()
            p.setClipRect(clip)
            p.setPen(QPen(QColor(col)))
            p.drawText(rect, Qt.AlignmentFlag.AlignCenter, text)
            p.restore()
        p.end()


# 欄寬（表頭與每一列共用，兩邊才對得齊）。2026-10-22 壓窄：SITL 頁「單機」模式第一次進入會自動展開這張表，
# 而 SITL 頁實際寬度約 420–450 px（螢幕 22%），扣掉直向捲軸與分頁框只剩約 400 px；以前 140／120／56 加起來整張表
# 要 522 px、第一次壓到 424 px 仍然冒水平捲軸（審查實測：422 px 面板 ⇒ 捲軸 32 px）。
# 現在 PWM 條是**彈性欄**（最少 _W_BAR_MIN、多的寬度都給它；表頭同一套設定所以對得齊），整張表最少約 360 px。
# 功能欄 98 px 容得下最長的名稱（TiltMotorRearRight 87 px ＋ 內距／框）；Min/Trim/Max 44 px 容得下四位數（23 px）
# 與表頭「Trim (μs)」（實測 44 px；§5.17.20.3.10 單位要寫在欄名裡，所以不縮欄名）。
_W_CH, _W_BAR_MIN, _W_BAR_MAX, _W_REV, _W_FUNC, _W_NUM, _SPACING = 18, 60, 260, 18, 98, 44, 4
_W_BAR = _W_BAR_MIN       # 舊名（外部若有人讀）：現在是彈性欄的最小寬


def _elastic_bar_column(w: QWidget) -> None:
    """PWM 欄（表頭標籤與每列的條圖）用同一套尺寸設定：忽略 sizeHint、最少 _W_BAR_MIN、最多 _W_BAR_MAX。
    兩邊設定完全一樣 ⇒ 同寬的列裡分到的寬度一樣 ⇒ 表頭對得齊。"""
    w.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Fixed)
    w.setMinimumWidth(_W_BAR_MIN)
    w.setMaximumWidth(_W_BAR_MAX)


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
        layout.setSpacing(_SPACING)

        # 通道號
        self.lbl_ch = QLabel(f'{self.ch}')
        self.lbl_ch.setFixedWidth(_W_CH)
        self.lbl_ch.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_ch.setStyleSheet(
            f'color:{TC.FG_SECONDARY};font-size:10px;'
            f'font-family:{TF.css_mono()};font-weight:bold;'
        )
        layout.addWidget(self.lbl_ch)

        # PWM bar（彈性欄：見 _elastic_bar_column）
        self.bar = PwmBar()
        _elastic_bar_column(self.bar)
        layout.addWidget(self.bar, 1)

        # Reverse 顯示（唯讀 checkbox 樣式）
        self.lbl_rev = QLabel('☐')
        self.lbl_rev.setFixedWidth(_W_REV)
        self.lbl_rev.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_rev.setStyleSheet(
            f'color:{TC.FG_MUTED};font-size:14px;'
        )
        self.lbl_rev.setToolTip('SERVOn_REVERSED')
        layout.addWidget(self.lbl_rev)

        # Function 名稱
        self.lbl_func = QLabel('Disabled')
        self.lbl_func.setFixedWidth(_W_FUNC)
        self.lbl_func.setStyleSheet(self._func_css(muted=False))
        layout.addWidget(self.lbl_func)

        # Min / Trim / Max（QSpinBox 樣式但唯讀）
        for label, name in [('Min', 'min'), ('Trim', 'trim'), ('Max', 'max')]:
            sb = QSpinBox()
            sb.setRange(800, 2200)
            sb.setValue({'min': 1000, 'trim': 1500, 'max': 2000}[name])
            sb.setFixedWidth(_W_NUM)
            sb.setReadOnly(not self.EDITABLE)
            sb.setButtonSymbols(QSpinBox.ButtonSymbols.NoButtons if not self.EDITABLE
                                  else QSpinBox.ButtonSymbols.UpDownArrows)
            sb.setStyleSheet(self._sb_css(False))
            setattr(self, f'sb_{name}', sb)
            layout.addWidget(sb)

        layout.addStretch(0)                  # PWM 欄到最大寬之後，多的寬度留在最右邊（表頭同一套）
        self.setFixedHeight(24)
        # 一開始就是「尚未讀回」：以前預設顯示 Disabled／1000／1500／2000，看起來像讀回值（有效值契約）
        self.reset()

    @staticmethod
    def _sb_css(muted: bool) -> str:
        return (f'QSpinBox{{background:{TC.BG_SUNKEN};'
                f'color:{TC.FG_MUTED if muted else TC.FG_PRIMARY};'
                f'border:1px solid {TC.BORDER_DEFAULT};border-radius:0;'
                f'padding:1px 2px;font-size:10px;'
                f'font-family:{TF.css_mono()};}}')

    @staticmethod
    def _func_css(muted: bool) -> str:
        return (f'background:{TC.BG_SUNKEN};'
                f'color:{TC.FG_MUTED if muted else TC.FG_PRIMARY};'
                f'border:1px solid {TC.BORDER_DEFAULT};border-radius:0;'
                f'padding:2px 4px;font-size:10px;'
                f'font-family:{TF.css_condensed()};letter-spacing:0px;')

    def reset(self) -> None:
        """回到「這台還沒讀回任何 SERVOn_*」：PWM 0（顯示 ---）、功能「—」、Min/Trim/Max 灰字（數字只是佔位）。

        換看另一台時一定要先 reset：不然上一台讀回的功能名會留在沒收到的通道上，被當成這台的值。
        """
        self.bar.set_pwm(0)
        self.bar.set_enabled_channel(True)
        self.lbl_func.setText('—')
        self.lbl_func.setToolTip(f'尚未讀回 SERVO{self.ch}_FUNCTION')
        self.lbl_func.setStyleSheet(self._func_css(muted=True))
        for name, default in (('min', 1000), ('trim', 1500), ('max', 2000)):
            sb = getattr(self, f'sb_{name}')
            sb.setValue(default)
            sb.setStyleSheet(self._sb_css(True))
            sb.setToolTip(f'尚未讀回 SERVO{self.ch}_{name.upper()}（{default} 只是佔位）')
        self.bar.set_range(1000, 2000)
        self.set_reversed(False)
        self.lbl_rev.setToolTip(f'SERVO{self.ch}_REVERSED（尚未讀回）')

    # ── 公開資料寫入 ─────────────────────────────────────────────
    def set_pwm(self, pwm: int):
        self.bar.set_pwm(pwm)

    def set_function(self, func_code: int):
        name = servo_function_name(func_code)
        self.lbl_func.setText(name)
        self.lbl_func.setToolTip(f'SERVO{self.ch}_FUNCTION = {int(func_code)}（{name}）')
        is_disabled = (func_code == 0)
        self.bar.set_enabled_channel(not is_disabled)
        # 停用通道整列文字降亮度
        self.lbl_func.setStyleSheet(self._func_css(muted=is_disabled))

    def set_min_trim_max(self, pmin: int = None, ptrim: int = None, pmax: int = None):
        for name, v in (('min', pmin), ('trim', ptrim), ('max', pmax)):
            if v is None:
                continue
            sb = getattr(self, f'sb_{name}')
            sb.setValue(int(v))
            sb.setStyleSheet(self._sb_css(False))
            sb.setToolTip(f'SERVO{self.ch}_{name.upper()} = {int(v)}')
        # 同步 PWM bar 範圍（取 min/max）
        self.bar.set_range(self.sb_min.value(), self.sb_max.value())

    def set_reversed(self, rev: bool):
        self.lbl_rev.setToolTip(f'SERVO{self.ch}_REVERSED = {int(bool(rev))}')
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

    多機時（SITLHud 單機模式）由 SITLHud 依 sysid 快取 SERVOn_*，換看另一台時
    reset() → 重灌那台的快取 → set_source(sysid, 筆數)；面板本身不分 sysid。
    """

    TITLE = '▎SERVO OUTPUT  —  16 通道 PWM (μs)'
    #: 每台要讀回的 SERVOn_* 筆數（16 通道 × FUNCTION/MIN/TRIM/MAX/REVERSED）
    N_PARAMS = 16 * 5

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

        # 標題列（set_source 會改成「— UAVn —」並帶參數讀回筆數）
        self.lbl_title = QLabel(self.TITLE)
        # 標題帶機號＋讀回筆數時很長：可換行，不讓它把整張表的最小寬撐大（單機頁寬約 400 px）
        self.lbl_title.setWordWrap(True)
        self.lbl_title.setStyleSheet(
            f'color:{TC.FG_EMPHASIS};'
            f'font-family:{TF.css_condensed()};letter-spacing:1.5px;'
            f'font-size:11px;font-weight:bold;padding:2px 0 4px 2px;'
        )
        v.addWidget(self.lbl_title)

        # 表頭（與 ServoChannelRow 對齊）
        header = QHBoxLayout()
        header.setContentsMargins(2, 0, 2, 0)
        header.setSpacing(_SPACING)
        # §5.17.20.3.10 — units 必須包含於 column label
        for txt, w, align in [
            ('#',         _W_CH,   Qt.AlignmentFlag.AlignCenter),
            ('PWM (μs)',  None,    Qt.AlignmentFlag.AlignCenter),     # None ＝ 彈性欄（同條圖）
            ('Rev',       _W_REV,  Qt.AlignmentFlag.AlignCenter),
            ('Function',  _W_FUNC, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
            ('Min (μs)',  _W_NUM,  Qt.AlignmentFlag.AlignCenter),
            ('Trim (μs)', _W_NUM,  Qt.AlignmentFlag.AlignCenter),
            ('Max (μs)',  _W_NUM,  Qt.AlignmentFlag.AlignCenter),
        ]:
            lbl = QLabel(txt)
            if w is None:
                _elastic_bar_column(lbl)
            else:
                lbl.setFixedWidth(w)
            lbl.setAlignment(align)
            lbl.setStyleSheet(
                f'color:{TC.FG_SECONDARY};font-size:9px;font-weight:bold;'
                f'font-family:{TF.css_condensed()};letter-spacing:0.3px;'
                f'border-bottom:1px solid {TC.BORDER_SUBTLE};padding:0 0 2px 0;'
            )
            header.addWidget(lbl, 1 if w is None else 0)
        header.addStretch(0)
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

    # ── 來源（哪一台）─────────────────────────────────────────────
    def reset(self) -> None:
        """16 列全部回到「尚未讀回」，標題回到不帶機號。"""
        for row in self._rows:
            row.reset()
        self.lbl_title.setText(self.TITLE)
        self.lbl_title.setToolTip('')

    def set_source(self, sysid, n_params: int) -> None:
        """標題標出這是哪一台、SERVOn_* 讀回了幾筆（共 N_PARAMS）。sysid None ⇒ 不帶機號。"""
        if sysid is None:
            self.lbl_title.setText(self.TITLE)
            self.lbl_title.setToolTip('')
            return
        k = int(n_params)
        self.lbl_title.setText(
            f'▎SERVO OUTPUT — UAV{int(sysid)} — 16 通道 PWM (μs)　參數 {k}/{self.N_PARAMS}')
        self.lbl_title.setToolTip(
            f'UAV{int(sysid)} 的 SERVOn_FUNCTION/MIN/TRIM/MAX/REVERSED 已讀回 {k} 筆（共 {self.N_PARAMS}）；'
            '連線時讀一次。功能欄「—」＝那個通道還沒讀回。')

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
