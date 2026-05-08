"""
TacticalSwarmStrikePanel — 蜂群打擊全任務生命週期面板

涵蓋四個階段：
    Phase 1  VTOL TAKEOFF     ──  在起飛點垂直爬升並完成轉態
    Phase 2  CRUISE TRANSIT   ──  依 bearing 經「轉態 WP → 對準 WP → 決斷圈」
    Phase 3  TERMINAL (ROE)   ──  依指揮官授權決定：
                                     ▸ ARMED STRIKE：加速俯衝命中
                                     ▸ RECON & RTL ：低空飛越 + QRTL 返航
    （Phase 4）QRTL LANDING    ──  RECON 模式由飛控的 QRTL 邏輯自動完成

相關 ArduPilot 參數（必須於地面站預先設定）：
    Q_RTL_MODE        = 1    # 收到 NAV_RTL 時改走 QRTL（垂直降落）
    Q_RTL_ALT         = 15   # QRTL 返航高度 (m AGL)
    Q_TRANSITION_MS   = 5000 # VTOL→FW 轉態時間（決定所需 transition_distance）
    Q_ASSIST_SPEED    = 18   # 低於此空速 VTOL 馬達自動介入防失速
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from PyQt6.QtCore import QPoint, QRect, QSize, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import (
    QBrush, QColor, QFont, QFontMetrics, QPainter, QPaintEvent, QPen,
)
from PyQt6.QtWidgets import (
    QDialog, QDoubleSpinBox, QFileDialog, QFrame, QGridLayout, QHBoxLayout,
    QLabel, QMessageBox, QPlainTextEdit, QPushButton, QSizePolicy, QVBoxLayout,
    QWidget,
)

from ui.resources.tactical_theme import TacticalColors, TacticalFonts


# ══════════════════════════════════════════════════════════════════════
#  Geodesy helpers — 大地量測
# ══════════════════════════════════════════════════════════════════════
_EARTH_R_M: float = 6_371_000.0


def haversine_m(p1: tuple[float, float], p2: tuple[float, float]) -> float:
    """Great-circle 距離（公尺）。"""
    lat1, lon1 = math.radians(p1[0]), math.radians(p1[1])
    lat2, lon2 = math.radians(p2[0]), math.radians(p2[1])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return _EARTH_R_M * 2.0 * math.asin(math.sqrt(a))


def bearing_deg(p1: tuple[float, float], p2: tuple[float, float]) -> float:
    """由 p1 指向 p2 的初始方位角（0-360，0=North，順時針）。"""
    lat1, lat2 = math.radians(p1[0]), math.radians(p2[0])
    dlon = math.radians(p2[1] - p1[1])
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def destination(origin: tuple[float, float], brg: float, dist_m: float) -> tuple[float, float]:
    """由 origin 沿 bearing 前進 dist_m 後的 (lat, lon)。"""
    lat1, lon1 = math.radians(origin[0]), math.radians(origin[1])
    brg_r = math.radians(brg)
    ang = dist_m / _EARTH_R_M
    lat2 = math.asin(
        math.sin(lat1) * math.cos(ang) + math.cos(lat1) * math.sin(ang) * math.cos(brg_r)
    )
    lon2 = lon1 + math.atan2(
        math.sin(brg_r) * math.sin(ang) * math.cos(lat1),
        math.cos(ang) - math.sin(lat1) * math.sin(lat2),
    )
    return math.degrees(lat2), (math.degrees(lon2) + 540.0) % 360.0 - 180.0


# ══════════════════════════════════════════════════════════════════════
#  ROE + Mission Params
# ══════════════════════════════════════════════════════════════════════
class ROEMode(str, Enum):
    """交戰規則模式。"""
    ARMED_STRIKE = "ARMED_STRIKE"   # 授權終端俯衝打擊
    RECON_RTL    = "RECON_RTL"      # 低空飛越後 QRTL 返航


@dataclass
class VTOLMissionParams:
    """VTOL 全任務參數。所有高度為 AGL（相對 Home 高度）。"""

    # ── 座標 ──
    launch_lat: float = 0.0
    launch_lon: float = 0.0
    target_lat: float = 0.0
    target_lon: float = 0.0

    # ── 高度 (m AGL) ──
    takeoff_alt_m: float = 50.0       # VTOL 爬升完成高度
    cruise_alt_m: float = 150.0       # 巡航/對準高度
    flyover_alt_m: float = 30.0       # RECON 模式飛越目標高度
    strike_alt_m: float = 5.0         # ARMED 模式終端命中高度（幾乎貼地）

    # ── 速度 (m/s) ──
    cruise_speed_ms: float = 25.0     # ~50 kts
    strike_speed_ms: float = 45.0     # ~87 kts 俯衝
    flyover_speed_ms: float = 28.0    # 飛越微幅加速

    # ── 幾何 (m) ──
    transition_distance_m: float = 600.0    # VTOL→FW 轉態所需直線距離
    approach_alignment_m: float = 4000.0    # 距目標多遠開始直線對準
    terminal_boundary_m: float = 2000.0     # 決斷圈半徑

    # ── ROE（由面板設定，生成時傳入） ──
    roe_mode: ROEMode = ROEMode.RECON_RTL

    def validate(self) -> list[str]:
        """回傳人類可讀的驗證錯誤訊息；空陣列代表通過。"""
        errs: list[str] = []
        dist = haversine_m((self.launch_lat, self.launch_lon),
                           (self.target_lat, self.target_lon))
        if dist < self.approach_alignment_m + self.transition_distance_m + 200:
            errs.append(
                f"起飛點至目標距離 {dist:.0f}m 不足，"
                f"需 > {self.approach_alignment_m + self.transition_distance_m + 200:.0f}m"
            )
        if self.takeoff_alt_m <= 0:
            errs.append("起飛高度必須 > 0")
        if self.cruise_alt_m < self.takeoff_alt_m:
            errs.append("巡航高度應高於起飛完成高度")
        if self.strike_speed_ms <= self.cruise_speed_ms:
            errs.append("打擊速度應大於巡航速度")
        return errs


# ══════════════════════════════════════════════════════════════════════
#  MAVLink Waypoint + Mission Generator
# ══════════════════════════════════════════════════════════════════════
@dataclass
class MAVWaypoint:
    """QGC WPL 110 單行結構。欄位順序即為匯出順序。"""
    seq: int
    current: int                # 1 = 起始點 (HOME)，其餘 0
    frame: int                  # MAV_FRAME
    command: int                # MAV_CMD
    p1: float = 0.0
    p2: float = 0.0
    p3: float = 0.0
    p4: float = 0.0
    x: float  = 0.0             # lat（或保留 0）
    y: float  = 0.0             # lon
    z: float  = 0.0             # alt (m, 依 frame 解讀)
    autocontinue: int = 1

    # 說明（僅供 UI 預覽顯示，不寫入檔案）
    note: str = ""

    def to_wpl_line(self) -> str:
        """輸出一行 QGC WPL 110 格式（tab 分隔）。"""
        return "\t".join([
            str(self.seq),
            str(self.current),
            str(self.frame),
            str(self.command),
            f"{self.p1:.8f}",
            f"{self.p2:.8f}",
            f"{self.p3:.8f}",
            f"{self.p4:.8f}",
            f"{self.x:.8f}",
            f"{self.y:.8f}",
            f"{self.z:.6f}",
            str(self.autocontinue),
        ])


class VTOLMissionGenerator:
    """依 VTOLMissionParams 生成完整 QuadPlane 任務航點。"""

    # MAV_CMD 編號（ArduPilot 相容）
    NAV_WAYPOINT:       int = 16
    NAV_RTL:            int = 20   # MAV_CMD_NAV_RETURN_TO_LAUNCH
    NAV_VTOL_TAKEOFF:   int = 84
    NAV_VTOL_LAND:      int = 85
    DO_CHANGE_SPEED:    int = 178

    # MAV_FRAME
    FRAME_GLOBAL:          int = 0  # 絕對 AMSL 高度（供 HOME 使用）
    FRAME_GLOBAL_REL_ALT:  int = 3  # 相對 HOME 高度（AGL）

    # DO_CHANGE_SPEED: param1 speed type
    SPEED_AIRSPEED:    int = 0
    SPEED_GROUNDSPEED: int = 1

    def generate(self, p: VTOLMissionParams) -> list[MAVWaypoint]:
        """依 ROE 分支生成完整航點序列。"""
        wps: list[MAVWaypoint] = []
        seq = 0

        # Bearing launch → target（整段任務的主軸）
        brg_fwd = bearing_deg((p.launch_lat, p.launch_lon),
                              (p.target_lat, p.target_lon))
        brg_back = (brg_fwd + 180.0) % 360.0   # 由目標回看起飛點

        # ─── WP 0 : HOME ─────────────────────────────────
        wps.append(MAVWaypoint(
            seq=seq, current=1, frame=self.FRAME_GLOBAL,
            command=self.NAV_WAYPOINT,
            x=p.launch_lat, y=p.launch_lon, z=0.0,
            note="HOME (Launch Point)",
        ))
        seq += 1

        # ─── WP 1 : VTOL TAKEOFF ─────────────────────────
        wps.append(MAVWaypoint(
            seq=seq, current=0, frame=self.FRAME_GLOBAL_REL_ALT,
            command=self.NAV_VTOL_TAKEOFF,
            p1=0.0,     # min takeoff pitch (deg)
            p4=brg_fwd, # 起飛後即朝目標方位
            x=p.launch_lat, y=p.launch_lon, z=p.takeoff_alt_m,
            note=f"VTOL_TAKEOFF  alt={p.takeoff_alt_m:.0f}m  yaw={brg_fwd:.1f}°",
        ))
        seq += 1

        # ─── WP 2 : 轉態 WP（給 FW 轉態充分直線距離） ────
        t_lat, t_lon = destination(
            (p.launch_lat, p.launch_lon), brg_fwd, p.transition_distance_m,
        )
        wps.append(MAVWaypoint(
            seq=seq, current=0, frame=self.FRAME_GLOBAL_REL_ALT,
            command=self.NAV_WAYPOINT,
            p2=25.0,  # pass radius (m)
            x=t_lat, y=t_lon, z=p.cruise_alt_m,
            note=f"TRANSITION_WP  +{p.transition_distance_m:.0f}m  climb→{p.cruise_alt_m:.0f}m",
        ))
        seq += 1

        # ─── WP 3 : DO_CHANGE_SPEED (巡航速度) ─────────
        wps.append(MAVWaypoint(
            seq=seq, current=0, frame=self.FRAME_GLOBAL_REL_ALT,
            command=self.DO_CHANGE_SPEED,
            p1=float(self.SPEED_AIRSPEED),
            p2=p.cruise_speed_ms,
            p3=-1.0,   # throttle: keep
            note=f"DO_CHANGE_SPEED  {p.cruise_speed_ms:.1f} m/s (cruise)",
        ))
        seq += 1

        # ─── WP 4 : 對準 WP（距目標 4km，沿 bearing 直線起點） ─
        a_lat, a_lon = destination(
            (p.target_lat, p.target_lon), brg_back, p.approach_alignment_m,
        )
        wps.append(MAVWaypoint(
            seq=seq, current=0, frame=self.FRAME_GLOBAL_REL_ALT,
            command=self.NAV_WAYPOINT,
            p2=30.0,
            x=a_lat, y=a_lon, z=p.cruise_alt_m,
            note=f"APPROACH_ALIGN  -{p.approach_alignment_m:.0f}m from target",
        ))
        seq += 1

        # ─── WP 5 : 終端決斷圈（2km） ──────────────────
        b_lat, b_lon = destination(
            (p.target_lat, p.target_lon), brg_back, p.terminal_boundary_m,
        )
        wps.append(MAVWaypoint(
            seq=seq, current=0, frame=self.FRAME_GLOBAL_REL_ALT,
            command=self.NAV_WAYPOINT,
            p2=20.0,
            x=b_lat, y=b_lon, z=p.cruise_alt_m,
            note=f"TERMINAL_BOUNDARY  -{p.terminal_boundary_m:.0f}m  [DECISION CIRCLE]",
        ))
        seq += 1

        # ─── Phase 3 : ROE 分支 ──────────────────────────
        if p.roe_mode == ROEMode.ARMED_STRIKE:
            wps.extend(self._emit_armed_strike(seq, p))
        else:
            wps.extend(self._emit_recon_rtl(seq, p))

        return wps

    # ─── Phase 3-A : ARMED STRIKE ───────────────────────
    def _emit_armed_strike(self, seq: int, p: VTOLMissionParams) -> list[MAVWaypoint]:
        out: list[MAVWaypoint] = []

        # 加速至打擊速度
        out.append(MAVWaypoint(
            seq=seq, current=0, frame=self.FRAME_GLOBAL_REL_ALT,
            command=self.DO_CHANGE_SPEED,
            p1=float(self.SPEED_AIRSPEED),
            p2=p.strike_speed_ms,
            p3=-1.0,
            note=f"DO_CHANGE_SPEED  {p.strike_speed_ms:.1f} m/s  [ARMED STRIKE ACCEL]",
        ))
        seq += 1

        # 目標命中 WP — 低高度差 + 高速 = 俯衝幾何
        # 飛控 L1 會自然形成 ~4° 下降角（150m→5m 跨越 2km）
        out.append(MAVWaypoint(
            seq=seq, current=0, frame=self.FRAME_GLOBAL_REL_ALT,
            command=self.NAV_WAYPOINT,
            p1=0.0,   # accept radius
            p2=3.0,   # pass radius (接近 0 = 命中)
            x=p.target_lat, y=p.target_lon, z=p.strike_alt_m,
            note=f"⚠  TARGET IMPACT  alt={p.strike_alt_m:.0f}m  [KINETIC STRIKE]",
        ))
        return out

    # ─── Phase 3-B : RECON & RTL ─────────────────────────
    def _emit_recon_rtl(self, seq: int, p: VTOLMissionParams) -> list[MAVWaypoint]:
        out: list[MAVWaypoint] = []

        out.append(MAVWaypoint(
            seq=seq, current=0, frame=self.FRAME_GLOBAL_REL_ALT,
            command=self.DO_CHANGE_SPEED,
            p1=float(self.SPEED_AIRSPEED),
            p2=p.flyover_speed_ms,
            p3=-1.0,
            note=f"DO_CHANGE_SPEED  {p.flyover_speed_ms:.1f} m/s  [FLYOVER PACE]",
        ))
        seq += 1

        out.append(MAVWaypoint(
            seq=seq, current=0, frame=self.FRAME_GLOBAL_REL_ALT,
            command=self.NAV_WAYPOINT,
            p2=10.0,
            x=p.target_lat, y=p.target_lon, z=p.flyover_alt_m,
            note=f"FLYOVER_WP  alt={p.flyover_alt_m:.0f}m  [RECON PASS]",
        ))
        seq += 1

        # NAV_RTL — 觸發 QRTL（需 Q_RTL_MODE=1）
        out.append(MAVWaypoint(
            seq=seq, current=0, frame=self.FRAME_GLOBAL_REL_ALT,
            command=self.NAV_RTL,
            note="NAV_RTL → QRTL (Q_RTL_MODE=1)  [vertical landing at HOME]",
        ))
        return out

    # ── 輸出 ───────────────────────────────────────────
    @staticmethod
    def to_qgc_wpl(waypoints: list[MAVWaypoint]) -> str:
        header = "QGC WPL 110"
        body = "\n".join(w.to_wpl_line() for w in waypoints)
        return header + "\n" + body + "\n"


# ══════════════════════════════════════════════════════════════════════
#  ROECard — 單一 ROE 選項卡（自繪）
# ══════════════════════════════════════════════════════════════════════
class ROECard(QFrame):
    """一個可點擊的 ROE 模式卡片。選中態以四角 L 瞄準框＋色彩強調。
    ARMED 模式選中時以 2Hz 方波脈衝（告警），RECON 則靜態（安全狀態）。
    """

    clicked = pyqtSignal()
    CHAMFER = 14

    def __init__(
        self,
        title: str,
        subtitle: str,
        accent_hex: str,
        is_lethal: bool,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._title = title
        self._subtitle = subtitle
        self._accent = QColor(accent_hex)
        self._is_lethal = is_lethal
        self._selected = False
        self._blink_on = False

        self.setMinimumHeight(96)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)

        self._blink_timer = QTimer(self)
        self._blink_timer.setInterval(500)  # 1 Hz 切換 = 2Hz 視覺頻率
        self._blink_timer.timeout.connect(self._on_blink_tick)

    def set_selected(self, selected: bool) -> None:
        if selected == self._selected:
            return
        self._selected = selected
        if selected and self._is_lethal:
            self._blink_timer.start()
        else:
            self._blink_timer.stop()
            self._blink_on = False
        self.update()

    def _on_blink_tick(self) -> None:
        self._blink_on = not self._blink_on
        self.update()

    # ── Mouse ─────────────────────────────────────────
    def mousePressEvent(self, event):  # noqa: ANN001
        if event.button() == Qt.MouseButton.LeftButton:
            self.clicked.emit()
        super().mousePressEvent(event)

    # ── Paint ─────────────────────────────────────────
    def paintEvent(self, _evt: QPaintEvent) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, False)
        w, h = self.width(), self.height()

        # 背景
        if self._selected:
            tint = QColor(self._accent)
            tint.setAlpha(60 if self._blink_on else 35)
            p.fillRect(0, 0, w, h, QColor(TacticalColors.BG_SECONDARY))
            p.fillRect(0, 0, w, h, tint)
        else:
            p.fillRect(0, 0, w, h, QColor(TacticalColors.BG_SECONDARY))

        # 四角 L 型瞄準框
        if self._selected:
            corner_col = self._accent
            corner_w = 3 if self._blink_on else 2
        else:
            corner_col = QColor(TacticalColors.BORDER_SUBTLE)
            corner_w = 1
        self._draw_chamfer(p, w, h, corner_col, corner_w)

        # 左上角狀態指示圓
        center = QPoint(18, 18)
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(self._accent if self._selected else QColor(TacticalColors.FG_MUTED), 2))
        p.drawEllipse(center, 6, 6)
        if self._selected:
            p.setBrush(QBrush(self._accent))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawEllipse(center, 3, 3)

        # 主標題
        title_font = TacticalFonts.condensed(14, bold=True, letter_spacing=2.5)
        p.setFont(title_font)
        title_color = self._accent if self._selected else QColor(TacticalColors.FG_SECONDARY)
        p.setPen(QPen(title_color))
        p.drawText(QRect(0, 34, w, 24), Qt.AlignmentFlag.AlignHCenter, self._title)

        # 副標題（任務描述）
        sub_font = TacticalFonts.mono(8, bold=False)
        p.setFont(sub_font)
        sub_color = QColor(TacticalColors.FG_PRIMARY) if self._selected \
                    else QColor(TacticalColors.FG_MUTED)
        p.setPen(QPen(sub_color))
        p.drawText(QRect(0, 62, w, 18), Qt.AlignmentFlag.AlignHCenter, self._subtitle)

        # 底部狀態文字
        status_text = "● AUTHORIZED" if self._selected else "○ STANDBY"
        status_col = self._accent if self._selected else QColor(TacticalColors.FG_MUTED)
        p.setPen(QPen(status_col))
        p.setFont(TacticalFonts.condensed(8, bold=True, letter_spacing=1.2))
        p.drawText(QRect(0, h - 20, w, 16), Qt.AlignmentFlag.AlignHCenter, status_text)

    def _draw_chamfer(self, p: QPainter, w: int, h: int,
                      color: QColor, width: int) -> None:
        pen = QPen(color, width)
        pen.setCapStyle(Qt.PenCapStyle.FlatCap)
        p.setPen(pen)
        c = self.CHAMFER
        p.drawLine(0, 0, c, 0);                 p.drawLine(0, 0, 0, c)
        p.drawLine(w - c, 0, w - 1, 0);         p.drawLine(w - 1, 0, w - 1, c)
        p.drawLine(0, h - 1, c, h - 1);         p.drawLine(0, h - c, 0, h - 1)
        p.drawLine(w - c, h - 1, w - 1, h - 1); p.drawLine(w - 1, h - c, w - 1, h - 1)


# ══════════════════════════════════════════════════════════════════════
#  ROEToggle — 雙卡片互斥切換 + ARMED 二次確認
# ══════════════════════════════════════════════════════════════════════
class ROEToggle(QWidget):
    """兩張並排的 ROE 模式卡片。選取 ARMED 時彈出致命授權確認。"""

    mode_changed = pyqtSignal(str)   # emits ROEMode.value

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._mode: ROEMode = ROEMode.RECON_RTL   # 安全預設
        self._init_ui()
        self._apply_mode()

    def _init_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(6)

        header = QLabel("RULES OF ENGAGEMENT  ·  ROE")
        header.setFont(TacticalFonts.condensed(10, bold=True, letter_spacing=2.0))
        header.setStyleSheet(f"color: {TacticalColors.FG_EMPHASIS};")
        header.setAlignment(Qt.AlignmentFlag.AlignCenter)
        root.addWidget(header)

        row = QHBoxLayout()
        row.setSpacing(8)

        self.armed_card = ROECard(
            "ARMED STRIKE", "TERMINAL DIVE · KINETIC KILL",
            TacticalColors.HOSTILE, is_lethal=True, parent=self,
        )
        self.armed_card.clicked.connect(
            lambda: self._request_mode(ROEMode.ARMED_STRIKE)
        )

        self.recon_card = ROECard(
            "RECON & RTL", "FLYOVER 30M · QRTL RECOVER",
            TacticalColors.FRIENDLY, is_lethal=False, parent=self,
        )
        self.recon_card.clicked.connect(
            lambda: self._request_mode(ROEMode.RECON_RTL)
        )

        row.addWidget(self.armed_card, 1)
        row.addWidget(self.recon_card, 1)
        root.addLayout(row)

    def _request_mode(self, mode: ROEMode) -> None:
        if mode == self._mode:
            return
        # 切換至 ARMED → 強制二次確認（MIL-STD-1472H 致命決策雙動作）
        if mode == ROEMode.ARMED_STRIKE:
            ret = QMessageBox.warning(
                self,
                "CONFIRM LETHAL AUTHORIZATION",
                "■  ROE MODE CHANGE  ■\n\n"
                "You are about to arm TERMINAL DIVE mode.\n"
                "Aircraft will execute kinetic self-destruct strike\n"
                "upon crossing the 2 km decision boundary.\n\n"
                "This authorization is IRREVERSIBLE once uploaded.\n"
                "Do you confirm?",
                QMessageBox.StandardButton.Cancel | QMessageBox.StandardButton.Yes,
                QMessageBox.StandardButton.Cancel,
            )
            if ret != QMessageBox.StandardButton.Yes:
                return
        self._mode = mode
        self._apply_mode()
        self.mode_changed.emit(mode.value)

    def _apply_mode(self) -> None:
        self.armed_card.set_selected(self._mode == ROEMode.ARMED_STRIKE)
        self.recon_card.set_selected(self._mode == ROEMode.RECON_RTL)

    def mode(self) -> ROEMode:
        return self._mode

    def set_mode(self, mode: ROEMode, skip_confirm: bool = False) -> None:
        """外部程式化設定模式，可選擇跳過確認（僅用於載入儲存檔）。"""
        if skip_confirm:
            self._mode = mode
            self._apply_mode()
            self.mode_changed.emit(mode.value)
        else:
            self._request_mode(mode)


# ══════════════════════════════════════════════════════════════════════
#  CoordinateInput — 座標雙欄輸入 + PICK
# ══════════════════════════════════════════════════════════════════════
class CoordinateInput(QWidget):
    """LAT/LON 等寬雙欄輸入，含從地圖拾取按鈕。"""

    pick_requested = pyqtSignal()
    changed = pyqtSignal(float, float)

    def __init__(
        self,
        title: str,
        accent_hex: str = TacticalColors.NEUTRAL,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._title = title
        self._accent = accent_hex
        self._init_ui()

    def _init_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(10, 8, 10, 8)
        root.setSpacing(4)

        hdr = QLabel(self._title)
        hdr.setFont(TacticalFonts.condensed(9, bold=True, letter_spacing=1.5))
        hdr.setStyleSheet(f"color: {self._accent};")
        root.addWidget(hdr)

        row = QHBoxLayout()
        row.setSpacing(4)

        self.lat = QDoubleSpinBox()
        self.lat.setRange(-90.0, 90.0)
        self.lat.setDecimals(7)
        self.lat.setSingleStep(0.0001)
        self.lat.setPrefix("LAT  ")
        self.lat.setFont(TacticalFonts.mono(9))
        self.lat.valueChanged.connect(self._emit_changed)

        self.lon = QDoubleSpinBox()
        self.lon.setRange(-180.0, 180.0)
        self.lon.setDecimals(7)
        self.lon.setSingleStep(0.0001)
        self.lon.setPrefix("LON  ")
        self.lon.setFont(TacticalFonts.mono(9))
        self.lon.valueChanged.connect(self._emit_changed)

        row.addWidget(self.lat, 3)
        row.addWidget(self.lon, 3)

        pick = QPushButton("PICK")
        pick.setFixedWidth(56)
        pick.setToolTip("Pick this point on the map")
        pick.clicked.connect(self.pick_requested)
        row.addWidget(pick)

        root.addLayout(row)

    # ── API ───────────────────────────────────────────
    def set_coords(self, lat: float, lon: float) -> None:
        for sb, v in ((self.lat, lat), (self.lon, lon)):
            sb.blockSignals(True)
            sb.setValue(v)
            sb.blockSignals(False)
        self._emit_changed()

    def coords(self) -> tuple[float, float]:
        return self.lat.value(), self.lon.value()

    def _emit_changed(self) -> None:
        self.changed.emit(self.lat.value(), self.lon.value())


# ══════════════════════════════════════════════════════════════════════
#  ParamRow — 標籤 + SpinBox 水平組
# ══════════════════════════════════════════════════════════════════════
class ParamRow(QWidget):
    """右對齊的「標籤 數值 單位」列。"""

    changed = pyqtSignal(float)

    def __init__(
        self,
        label: str, value: float, unit: str,
        minimum: float, maximum: float, step: float = 1.0,
        decimals: int = 1, parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)

        lbl = QLabel(label)
        lbl.setFont(TacticalFonts.condensed(9, bold=True, letter_spacing=1.2))
        lbl.setStyleSheet(f"color: {TacticalColors.FG_SECONDARY};")
        lbl.setMinimumWidth(120)
        lay.addWidget(lbl)

        self.sb = QDoubleSpinBox()
        self.sb.setRange(minimum, maximum)
        self.sb.setDecimals(decimals)
        self.sb.setSingleStep(step)
        self.sb.setValue(value)
        self.sb.setFont(TacticalFonts.mono(9))
        self.sb.setSuffix(f"  {unit}")
        self.sb.valueChanged.connect(self.changed)
        lay.addWidget(self.sb, 1)

    def value(self) -> float:
        return self.sb.value()

    def set_value(self, v: float) -> None:
        self.sb.setValue(v)


# ══════════════════════════════════════════════════════════════════════
#  TacticalSwarmStrikePanel — 主面板
# ══════════════════════════════════════════════════════════════════════
class TacticalSwarmStrikePanel(QWidget):
    """蜂群打擊任務面板。整合座標輸入 + ROE 切換 + 任務預覽/匯出。

    Signals:
        pick_launch_requested()            — 使用者按「起飛點 PICK」
        pick_target_requested()            — 使用者按「目標點 PICK」
        mission_planned(params, waypoints) — 按 PLAN 後計算完成
        mission_exported(file_path)        — 匯出 .waypoints 後
        mission_deployed(waypoints)        — 按 DEPLOY（外部接此進行 MAVLink 上傳）
    """

    pick_launch_requested = pyqtSignal()
    pick_target_requested = pyqtSignal()
    mission_planned = pyqtSignal(object, object)  # (VTOLMissionParams, list[MAVWaypoint])
    mission_exported = pyqtSignal(str)
    mission_deployed = pyqtSignal(object)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._generator = VTOLMissionGenerator()
        self._last_waypoints: list[MAVWaypoint] = []
        self._init_ui()

    # ─── UI Layout ────────────────────────────────────
    def _init_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        # ────── Zone 1 : TARGETING（座標 + 參數） ──────
        targeting = self._build_targeting_zone()
        root.addWidget(targeting, 0)

        # ────── Zone 2 : ROE ──────
        self.roe_toggle = ROEToggle(self)
        self.roe_toggle.mode_changed.connect(self._on_roe_changed)
        roe_frame = self._wrap_in_panel(self.roe_toggle, "ENGAGEMENT AUTHORITY")
        root.addWidget(roe_frame, 0)

        # ────── Zone 3 : PLAN 預覽 + 指令按鈕 ──────
        root.addWidget(self._build_plan_zone(), 1)

        # ────── Status bar ──────
        self.status_label = QLabel("● READY — ROE: RECON & RTL")
        self.status_label.setFont(TacticalFonts.mono(9))
        self.status_label.setStyleSheet(
            f"color: {TacticalColors.FG_SECONDARY}; padding: 4px;"
        )
        root.addWidget(self.status_label, 0)

    def _build_targeting_zone(self) -> QWidget:
        panel = QFrame()
        panel.setProperty("tacticalPanel", "true")
        lay = QGridLayout(panel)
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setHorizontalSpacing(10)
        lay.setVerticalSpacing(6)

        self.launch_input = CoordinateInput("LAUNCH POINT",
                                            accent_hex=TacticalColors.NEUTRAL)
        self.launch_input.pick_requested.connect(self.pick_launch_requested)

        self.target_input = CoordinateInput("STRIKE TARGET",
                                            accent_hex=TacticalColors.FG_EMPHASIS)
        self.target_input.pick_requested.connect(self.pick_target_requested)

        lay.addWidget(self.launch_input, 0, 0)
        lay.addWidget(self.target_input, 0, 1)

        # ── 關鍵任務參數（可展開，這裡放常用幾項） ──
        self.takeoff_alt = ParamRow("TAKEOFF ALT",  50.0, "m", 10.0, 300.0, 5.0, 0)
        self.cruise_alt  = ParamRow("CRUISE ALT",  150.0, "m", 30.0, 500.0, 10.0, 0)
        self.cruise_spd  = ParamRow("CRUISE SPD",   25.0, "m/s", 15.0, 40.0, 1.0, 1)
        self.strike_spd  = ParamRow("STRIKE SPD",   45.0, "m/s", 25.0, 80.0, 1.0, 1)
        self.flyover_alt = ParamRow("FLYOVER ALT",  30.0, "m", 10.0, 150.0, 5.0, 0)
        self.trans_dist  = ParamRow("TRANS DIST",  600.0, "m", 150.0, 2000.0, 50.0, 0)

        lay.addWidget(self.takeoff_alt, 1, 0)
        lay.addWidget(self.cruise_alt,  1, 1)
        lay.addWidget(self.cruise_spd,  2, 0)
        lay.addWidget(self.strike_spd,  2, 1)
        lay.addWidget(self.flyover_alt, 3, 0)
        lay.addWidget(self.trans_dist,  3, 1)

        return self._wrap_in_panel(panel, "TARGETING · PARAMETERS")

    def _build_plan_zone(self) -> QWidget:
        panel = QFrame()
        lay = QVBoxLayout(panel)
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setSpacing(6)

        # ── 指令按鈕列 ──
        btn_row = QHBoxLayout()
        btn_row.setSpacing(6)

        self.plan_btn = QPushButton("▶  PLAN MISSION")
        self.plan_btn.setProperty("tacticalRole", "execute")
        self.plan_btn.clicked.connect(self._on_plan_clicked)

        self.export_btn = QPushButton("⬇  EXPORT .WAYPOINTS")
        self.export_btn.setEnabled(False)
        self.export_btn.clicked.connect(self._on_export_clicked)

        self.deploy_btn = QPushButton("⚡  DEPLOY → SWARM")
        self.deploy_btn.setProperty("tacticalRole", "caution")
        self.deploy_btn.setEnabled(False)
        self.deploy_btn.clicked.connect(self._on_deploy_clicked)

        btn_row.addWidget(self.plan_btn, 2)
        btn_row.addWidget(self.export_btn, 2)
        btn_row.addWidget(self.deploy_btn, 2)
        lay.addLayout(btn_row)

        # ── 任務預覽（QGC WPL） ──
        self.preview = QPlainTextEdit()
        self.preview.setReadOnly(True)
        self.preview.setFont(TacticalFonts.mono(9))
        self.preview.setPlaceholderText(
            "// MISSION PLAN — press PLAN MISSION to generate QGC WPL 110\n"
        )
        self.preview.setMinimumHeight(200)
        lay.addWidget(self.preview, 1)

        return self._wrap_in_panel(panel, "MISSION PLAN  ·  QGC WPL 110")

    def _wrap_in_panel(self, inner: QWidget, title: str) -> QFrame:
        """將 widget 包入一個帶標題的戰術面板（上方小標籤 + 邊框）。"""
        frame = QFrame()
        frame.setFrameShape(QFrame.Shape.NoFrame)
        frame.setStyleSheet(
            f"QFrame {{ background-color: {TacticalColors.BG_SECONDARY};"
            f" border: 1px solid {TacticalColors.BORDER_DEFAULT}; }}"
        )
        lay = QVBoxLayout(frame)
        lay.setContentsMargins(1, 1, 1, 1)
        lay.setSpacing(0)

        header = QLabel(f"◤ {title} ◥")
        header.setFont(TacticalFonts.condensed(9, bold=True, letter_spacing=1.8))
        header.setStyleSheet(
            f"color: {TacticalColors.FG_EMPHASIS};"
            f" background-color: {TacticalColors.BG_PRIMARY};"
            f" padding: 4px 8px;"
            f" border-bottom: 1px solid {TacticalColors.BORDER_DEFAULT};"
        )
        lay.addWidget(header)
        lay.addWidget(inner, 1)
        return frame

    # ─── Parameter helpers ────────────────────────────
    def current_params(self) -> VTOLMissionParams:
        lat1, lon1 = self.launch_input.coords()
        lat2, lon2 = self.target_input.coords()
        return VTOLMissionParams(
            launch_lat=lat1, launch_lon=lon1,
            target_lat=lat2, target_lon=lon2,
            takeoff_alt_m=self.takeoff_alt.value(),
            cruise_alt_m=self.cruise_alt.value(),
            flyover_alt_m=self.flyover_alt.value(),
            cruise_speed_ms=self.cruise_spd.value(),
            strike_speed_ms=self.strike_spd.value(),
            transition_distance_m=self.trans_dist.value(),
            roe_mode=self.roe_toggle.mode(),
        )

    def set_launch_point(self, lat: float, lon: float) -> None:
        self.launch_input.set_coords(lat, lon)

    def set_target_point(self, lat: float, lon: float) -> None:
        self.target_input.set_coords(lat, lon)

    # ─── Event handlers ───────────────────────────────
    def _on_roe_changed(self, mode_str: str) -> None:
        mode = ROEMode(mode_str)
        text = ("⚠  ROE: ARMED STRIKE  —  TERMINAL DIVE AUTHORIZED"
                if mode == ROEMode.ARMED_STRIKE
                else "● ROE: RECON & RTL  —  SAFE RECOVERY")
        color = (TacticalColors.HOSTILE if mode == ROEMode.ARMED_STRIKE
                 else TacticalColors.FRIENDLY)
        self.status_label.setText(text)
        self.status_label.setStyleSheet(f"color: {color}; padding: 4px;")
        # 若已有預覽，清空以防誤用舊 ROE 的任務
        self._last_waypoints = []
        self.preview.setPlainText(
            "// ROE changed — re-plan mission\n"
        )
        self.export_btn.setEnabled(False)
        self.deploy_btn.setEnabled(False)

    def _on_plan_clicked(self) -> None:
        p = self.current_params()
        errs = p.validate()
        if errs:
            QMessageBox.warning(self, "PARAMETER VALIDATION",
                                "Mission parameters invalid:\n\n• "
                                + "\n• ".join(errs))
            return

        wps = self._generator.generate(p)
        wpl = self._generator.to_qgc_wpl(wps)

        # 預覽：純 WPL 後附帶註解區塊
        note_lines = [f"// [{w.seq:02d}] {w.note}" for w in wps if w.note]
        preview_text = wpl + "\n" + "\n".join(note_lines)
        self.preview.setPlainText(preview_text)

        self._last_waypoints = wps
        self.export_btn.setEnabled(True)
        self.deploy_btn.setEnabled(True)

        total_m = haversine_m(
            (p.launch_lat, p.launch_lon),
            (p.target_lat, p.target_lon),
        )
        self.status_label.setText(
            f"● PLAN READY — {len(wps)} WP · "
            f"launch→target {total_m/1000:.2f} km · "
            f"ROE: {p.roe_mode.value}"
        )
        self.mission_planned.emit(p, wps)

    def _on_export_clicked(self) -> None:
        if not self._last_waypoints:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Export mission as QGC WPL", "swarm_strike.waypoints",
            "Waypoint files (*.waypoints);;All files (*)"
        )
        if not path:
            return
        try:
            content = self._generator.to_qgc_wpl(self._last_waypoints)
            with open(path, "w", encoding="utf-8", newline="\n") as f:
                f.write(content)
            self.status_label.setText(
                f"● EXPORTED — {os.path.basename(path)}"
            )
            self.mission_exported.emit(path)
        except OSError as e:
            QMessageBox.critical(self, "Export failed", str(e))

    def _on_deploy_clicked(self) -> None:
        if not self._last_waypoints:
            return
        mode = self.roe_toggle.mode()
        # Deploy 需二次確認（無論 ROE），ARMED 用紅色警示
        icon = (QMessageBox.Icon.Critical if mode == ROEMode.ARMED_STRIKE
                else QMessageBox.Icon.Warning)
        box = QMessageBox(self)
        box.setIcon(icon)
        box.setWindowTitle("CONFIRM DEPLOY")
        box.setText(
            f"■ DEPLOY MISSION TO SWARM ■\n\n"
            f"ROE           : {mode.value}\n"
            f"Waypoint count: {len(self._last_waypoints)}\n"
            f"Action        : Upload to connected UAV(s)\n\n"
            "Proceed?"
        )
        box.setStandardButtons(QMessageBox.StandardButton.Cancel
                               | QMessageBox.StandardButton.Yes)
        box.setDefaultButton(QMessageBox.StandardButton.Cancel)
        if box.exec() != QMessageBox.StandardButton.Yes:
            return
        self.status_label.setText(
            f"⚡ DEPLOYING — {len(self._last_waypoints)} WP · ROE: {mode.value}"
        )
        self.mission_deployed.emit(self._last_waypoints)


# ══════════════════════════════════════════════════════════════════════
#  TacticalSwarmStrikeDialog — 彈出視窗包裝
# ══════════════════════════════════════════════════════════════════════
class TacticalSwarmStrikeDialog(QDialog):
    """將 TacticalSwarmStrikePanel 包裝為獨立浮動視窗。

    採非模態（NonModal）：視窗開著時仍可操作背後的地圖，
    讓「從地圖 PICK 起飛點/目標」流程順暢無阻。

    使用方式：
        dlg = TacticalSwarmStrikeDialog(parent=main_window)
        dlg.panel.pick_launch_requested.connect(main_window.on_swarm_pick_launch)
        dlg.panel.pick_target_requested.connect(main_window.on_swarm_pick_target)
        dlg.panel.mission_deployed.connect(main_window.on_swarm_deploy)
        dlg.show()   # NOT exec() — 要允許背景地圖互動
    """

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("SWARM STRIKE  ·  Full Mission Lifecycle")
        self.setMinimumSize(980, 820)
        # 非模態 + Qt.Tool 讓視窗在主視窗上方浮動但不霸佔焦點
        self.setWindowFlag(Qt.WindowType.Window, True)
        self.setModal(False)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)

        self.panel = TacticalSwarmStrikePanel(self)
        lay.addWidget(self.panel)


# ══════════════════════════════════════════════════════════════════════
#  Demo — python -m ui.widgets.tactical_swarm_strike_panel
# ══════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    import sys
    from PyQt6.QtWidgets import QApplication, QMainWindow

    from ui.resources.tactical_theme import apply_tactical_theme

    app = QApplication(sys.argv)
    apply_tactical_theme(app)

    win = QMainWindow()
    win.setWindowTitle("AeroPlan Studio — Tactical Swarm Strike (VTOL)")
    win.resize(960, 820)

    panel = TacticalSwarmStrikePanel()
    # 範例：台灣中部的一對座標（起飛→打擊約 7 km）
    panel.set_launch_point(24.1469, 120.6839)
    panel.set_target_point(24.2050, 120.6850)
    win.setCentralWidget(panel)

    # 示範外部接線：PICK 按鈕回呼
    panel.pick_launch_requested.connect(
        lambda: print("[demo] pick_launch_requested")
    )
    panel.pick_target_requested.connect(
        lambda: print("[demo] pick_target_requested")
    )
    panel.mission_planned.connect(
        lambda params, wps: print(
            f"[demo] planned  ROE={params.roe_mode.value}  N={len(wps)}"
        )
    )
    panel.mission_exported.connect(lambda p: print(f"[demo] exported → {p}"))
    panel.mission_deployed.connect(
        lambda wps: print(f"[demo] deploy N={len(wps)}")
    )

    win.show()
    sys.exit(app.exec())


__all__ = [
    "ROEMode",
    "VTOLMissionParams",
    "MAVWaypoint",
    "VTOLMissionGenerator",
    "ROECard",
    "ROEToggle",
    "CoordinateInput",
    "ParamRow",
    "TacticalSwarmStrikePanel",
    "TacticalSwarmStrikeDialog",
    "haversine_m",
    "bearing_deg",
    "destination",
]
