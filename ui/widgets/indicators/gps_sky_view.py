"""ui/widgets/indicators/gps_sky_view.py — GPS 天空圖。

對應 ADOS `GpsSkyView.tsx`：圓形天空圖（衛星方位角 + 仰角 + SNR）。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

from PyQt6.QtCore import Qt, QPointF
from PyQt6.QtGui import QColor, QPainter, QPen
from PyQt6.QtWidgets import QWidget

from ui.resources.aeroplan_theme import tokens as T


@dataclass
class Sat:
    """單顆衛星觀測值。"""
    prn: int = 0
    elevation_deg: float = 0.0   # 0=地平、90=正上
    azimuth_deg: float = 0.0     # 0=N，clockwise
    snr_db: int = 0              # 0–60


@dataclass
class GpsSnapshot:
    """GPS_RAW_INT + GPS_RTK 合成快照。"""
    fix_type: int = 0
    satellites_visible: int = 0
    satellites: List[Sat] = field(default_factory=list)


class GpsSkyView(QWidget):
    """圓形 sky view，外圈 = 地平、中心 = 天頂。"""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._snapshot = GpsSnapshot()
        self.setMinimumSize(120, 120)

    def update_from_snapshot(self, snap: GpsSnapshot) -> None:
        self._snapshot = snap
        self.update()

    def paintEvent(self, _evt) -> None:   # noqa: N802 — Qt 簽章
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)

        w = min(self.width(), self.height())
        cx, cy = self.width() / 2, self.height() / 2
        r = w / 2 - 6

        # 背景
        p.fillRect(self.rect(), QColor(T.BG_SUNKEN))

        # 同心圓 0° / 30° / 60°
        pen = QPen(QColor(T.BORDER_STRONG))
        pen.setWidth(1)
        p.setPen(pen)
        for frac in (1.0, 2 / 3, 1 / 3):
            rr = r * frac
            p.drawEllipse(QPointF(cx, cy), rr, rr)

        # 十字 N/E/S/W
        p.drawLine(QPointF(cx - r, cy), QPointF(cx + r, cy))
        p.drawLine(QPointF(cx, cy - r), QPointF(cx, cy + r))

        # N 標記
        p.setPen(QColor(T.FG_SECONDARY))
        p.drawText(int(cx - 4), int(cy - r + 12), "N")

        # 衛星
        import math
        for sat in self._snapshot.satellites:
            # 仰角 → 半徑（0° 在最外、90° 在中心）
            rho = r * (1.0 - max(0.0, min(90.0, sat.elevation_deg)) / 90.0)
            # 方位角 → 角度（0=N=上，clockwise）
            theta = math.radians(sat.azimuth_deg - 90.0)
            x = cx + rho * math.cos(theta)
            y = cy + rho * math.sin(theta)

            # SNR → 顏色
            if sat.snr_db >= 40:
                c = QColor(T.FRIENDLY)
            elif sat.snr_db >= 25:
                c = QColor(T.WARNING)
            else:
                c = QColor(T.HOSTILE)
            p.setBrush(c)
            p.setPen(Qt.PenStyle.NoPen)
            p.drawEllipse(QPointF(x, y), 4, 4)

        # 中央：可見衛星數
        p.setPen(QColor(T.HUD_GREEN))
        f = self.font()
        f.setBold(True)
        f.setPointSize(14)
        p.setFont(f)
        p.drawText(
            self.rect(), Qt.AlignmentFlag.AlignCenter,
            f"{self._snapshot.satellites_visible}",
        )
        p.end()


__all__ = ["GpsSkyView", "GpsSnapshot", "Sat"]
