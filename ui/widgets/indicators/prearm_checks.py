"""ui/widgets/indicators/prearm_checks.py — Pre-arm 檢查列表。

對應 ADOS `PreArmChecks.tsx`：列 SYS_STATUS_SENSOR 與 EKF_STATUS_REPORT，
逐項顯示 pass/fail icon。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from PyQt6.QtWidgets import (
    QLabel,
    QVBoxLayout,
    QWidget,
    QHBoxLayout,
)

from ui.resources.aeroplan_theme import tokens as T


@dataclass
class PreArmStatus:
    """Pre-arm 各項目布林結果。True = pass。"""
    gyro_ok: bool = False
    accel_ok: bool = False
    mag_ok: bool = False
    baro_ok: bool = False
    gps_ok: bool = False
    rc_ok: bool = False
    battery_ok: bool = False
    ekf_ok: bool = False
    fence_ok: bool = False
    notes: list[str] = field(default_factory=list)

    @property
    def all_ok(self) -> bool:
        return all([
            self.gyro_ok, self.accel_ok, self.mag_ok, self.baro_ok,
            self.gps_ok, self.rc_ok, self.battery_ok, self.ekf_ok,
            self.fence_ok,
        ])


class PreArmChecks(QWidget):
    """Pre-arm 檢查清單顯示器。"""

    _ROWS = [
        ("GYRO", "gyro_ok"),
        ("ACCEL", "accel_ok"),
        ("MAG", "mag_ok"),
        ("BARO", "baro_ok"),
        ("GPS", "gps_ok"),
        ("RC", "rc_ok"),
        ("BATT", "battery_ok"),
        ("EKF", "ekf_ok"),
        ("FENCE", "fence_ok"),
    ]

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 6, 8, 6)
        lay.setSpacing(2)

        self._title = QLabel("PRE-ARM", self)
        self._title.setStyleSheet(
            f"font-family: {T.FONT_DISPLAY_STACK}; font-weight: 700; "
            f"font-size: 11px; color: {T.FG_SECONDARY}; letter-spacing: 0.8px;"
        )
        lay.addWidget(self._title)

        self._labels: dict[str, QLabel] = {}
        for label_text, attr in self._ROWS:
            row = QWidget(self)
            hl = QHBoxLayout(row)
            hl.setContentsMargins(0, 0, 0, 0)
            hl.setSpacing(6)

            dot = QLabel(row)
            dot.setProperty("role", "statusdot")
            dot.setProperty("severity", "warning")
            dot.setFixedSize(8, 8)
            self._labels[f"{attr}_dot"] = dot

            lab = QLabel(label_text, row)
            lab.setStyleSheet(
                f"font-family: {T.FONT_MONO_STACK}; font-weight: 700; "
                f"font-size: 11px; color: {T.FG};"
            )
            self._labels[attr] = lab

            hl.addWidget(dot)
            hl.addWidget(lab)
            hl.addStretch(1)
            lay.addWidget(row)

    def update_from_status(self, st: PreArmStatus) -> None:
        for _label_text, attr in self._ROWS:
            ok = bool(getattr(st, attr, False))
            dot = self._labels[f"{attr}_dot"]
            dot.setProperty("severity", "success" if ok else "critical")
            dot.style().unpolish(dot)
            dot.style().polish(dot)
            lab = self._labels[attr]
            lab.setStyleSheet(
                f"font-family: {T.FONT_MONO_STACK}; font-weight: 700; "
                f"font-size: 11px; color: "
                f"{T.FG if ok else T.HOSTILE};"
            )


__all__ = ["PreArmStatus", "PreArmChecks"]
