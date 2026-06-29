"""ui/dialogs/strike_bda_dialog.py — 蜂群打擊 BDA (Battle Damage Assessment) 對話框。

由 StrikeControllerMixin._finalize_strike_visualization 在所有 UAV 抵達 IMPACT
階段（或 600s 超時）後彈出。內容由 JS 端 strikeVizEnd() 收集回傳：

stats: {
  uavs: [
    {sysid, callsign, missM, bearingDeg, ttImpactS, impactLat, impactLon},
    ...
  ],
  summary: {meanMissM, maxMissM, ttSpreadS, targetLat, targetLon}
}

提供：
    • 標頭：目標座標、平均 miss、最大 miss、TTT spread
    • 每機表格：呼號 / sysid / miss / 進入方位 / 命中時間 / 命中座標
    • 動作鈕：匯出 CSV、關閉
"""
from __future__ import annotations

import csv
import math
from datetime import datetime
from pathlib import Path
from typing import Optional

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QDialog,
    QFileDialog,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMessageBox,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ui.resources.aeroplan_theme import tokens as T

# 命中等級色帶（依 miss distance）
_MISS_EXCELLENT = 10.0    # < 10m → 綠
_MISS_GOOD = 30.0         # < 30m → 青
_MISS_FAIR = 80.0         # < 80m → 黃，> 80m → 紅


def _miss_color(miss_m: float) -> str:
    if not math.isfinite(miss_m):
        return T.FG_MUTED
    if miss_m < _MISS_EXCELLENT:
        return T.FRIENDLY
    if miss_m < _MISS_GOOD:
        return T.NEUTRAL
    if miss_m < _MISS_FAIR:
        return T.WARNING
    return T.HOSTILE


class StrikeBDADialog(QDialog):
    """BDA 對話框 — 蜂群打擊命中後統計報告"""

    def __init__(self, stats: dict, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._stats = stats or {'uavs': [], 'summary': {}}

        self.setWindowTitle("Battle Damage Assessment — 蜂群打擊命中報告")
        self.setMinimumSize(720, 460)
        self.setStyleSheet(f"background: {T.BG_PRIMARY}; color: {T.FG};")

        self._build_ui()

    # ── UI ───────────────────────────────────────────────────────────
    def _build_ui(self) -> None:
        v = QVBoxLayout(self)
        v.setContentsMargins(16, 14, 16, 14)
        v.setSpacing(10)

        summary = self._stats.get('summary', {})
        uavs = self._stats.get('uavs', [])
        n_uavs = len(uavs)
        n_impacted = sum(
            1 for u in uavs
            if math.isfinite(float(u.get('ttImpactS', float('nan'))))
        )

        # ── 標題列 ──
        title = QLabel("⚔  BATTLE DAMAGE ASSESSMENT", self)
        title.setStyleSheet(
            f"font-family: {T.FONT_DISPLAY_STACK}; font-weight: 800; "
            f"font-size: 18px; color: {T.HOSTILE}; "
            f"letter-spacing: 1.2px; padding: 4px 0;"
        )
        v.addWidget(title)

        tgt_lat = float(summary.get('targetLat', 0.0))
        tgt_lon = float(summary.get('targetLon', 0.0))
        subt = QLabel(
            f"Target  {tgt_lat:.6f}°,  {tgt_lon:.6f}°    │    "
            f"Time  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
            self,
        )
        subt.setStyleSheet(
            f"font-family: {T.FONT_MONO_STACK}; font-size: 11px; "
            f"color: {T.FG_SECONDARY};"
        )
        v.addWidget(subt)

        # ── 摘要 KPI 行 ──
        v.addLayout(self._build_kpi_row(summary, n_uavs, n_impacted))

        # ── 每機表格 ──
        v.addWidget(self._build_table(uavs), 1)

        # ── 按鈕列 ──
        btn_row = QHBoxLayout()
        btn_row.addStretch(1)
        btn_export = QPushButton("匯出 CSV", self)
        btn_export.setStyleSheet(self._btn_style(T.NEUTRAL))
        btn_export.clicked.connect(self._on_export_csv)
        btn_close = QPushButton("關閉", self)
        btn_close.setStyleSheet(self._btn_style(T.BORDER))
        btn_close.clicked.connect(self.accept)
        btn_row.addWidget(btn_export)
        btn_row.addWidget(btn_close)
        v.addLayout(btn_row)

    def _build_kpi_row(self, summary: dict, n_uavs: int, n_impacted: int) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(8)

        def _kpi(label: str, value: str, color: str) -> QWidget:
            card = QWidget()
            card.setStyleSheet(
                f"background: {T.BG_ELEVATED}; "
                f"border: 1px solid {T.BORDER_SUBTLE}; border-radius: 6px;"
            )
            lay = QVBoxLayout(card)
            lay.setContentsMargins(10, 6, 10, 6)
            lay.setSpacing(0)
            val = QLabel(value, card)
            val.setStyleSheet(
                f"font-family: {T.FONT_DISPLAY_STACK}; font-weight: 800; "
                f"font-size: 22px; color: {color}; background: transparent;"
            )
            lab = QLabel(label, card)
            lab.setStyleSheet(
                f"font-family: {T.FONT_DISPLAY_STACK}; font-size: 9px; "
                f"color: {T.FG_MUTED}; background: transparent; "
                f"letter-spacing: 0.6px;"
            )
            lay.addWidget(val)
            lay.addWidget(lab)
            return card

        mean_miss = float(summary.get('meanMissM', 0.0))
        max_miss = float(summary.get('maxMissM', 0.0))
        tt_spread = float(summary.get('ttSpreadS', 0.0))

        row.addWidget(_kpi(
            "IMPACT RATE",
            f"{n_impacted}/{n_uavs}",
            T.FRIENDLY if n_impacted == n_uavs and n_uavs > 0 else T.WARNING,
        ), 1)
        row.addWidget(_kpi(
            "MEAN MISS",
            f"{mean_miss:.1f} m",
            _miss_color(mean_miss),
        ), 1)
        row.addWidget(_kpi(
            "MAX MISS",
            f"{max_miss:.1f} m",
            _miss_color(max_miss),
        ), 1)
        row.addWidget(_kpi(
            "TTT SPREAD",
            f"{tt_spread:.2f} s",
            T.FRIENDLY if tt_spread < 1.0 else (T.WARNING if tt_spread < 5.0 else T.HOSTILE),
        ), 1)
        return row

    def _build_table(self, uavs: list) -> QTableWidget:
        cols = ["Callsign", "sysid", "Miss (m)", "Bearing (°)",
                "Time-to-Impact (s)", "Impact Lat", "Impact Lon"]
        tbl = QTableWidget(self)
        tbl.setColumnCount(len(cols))
        tbl.setHorizontalHeaderLabels(cols)
        tbl.setRowCount(len(uavs))
        tbl.setStyleSheet(
            f"QTableWidget {{ background: {T.BG_ELEVATED}; "
            f"font-family: {T.FONT_MONO_STACK}; font-size: 11px; "
            f"color: {T.FG}; gridline-color: {T.BORDER_SUBTLE}; "
            f"border: 1px solid {T.BORDER_SUBTLE}; }} "
            f"QHeaderView::section {{ background: {T.BG_SECONDARY}; "
            f"color: {T.FG_SECONDARY}; padding: 4px 6px; border: none; "
            f"border-bottom: 1px solid {T.BORDER_SUBTLE}; font-weight: 700; }}"
        )
        tbl.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        tbl.setAlternatingRowColors(False)
        tbl.verticalHeader().setVisible(False)

        # 依 miss 由小到大排，最佳命中在頂端
        sorted_uavs = sorted(
            uavs,
            key=lambda u: float(u.get('missM', float('inf')))
            if math.isfinite(float(u.get('missM', float('inf')))) else float('inf'),
        )

        for r, u in enumerate(sorted_uavs):
            miss = float(u.get('missM', float('nan')))
            brg = float(u.get('bearingDeg', float('nan')))
            tti = float(u.get('ttImpactS', float('nan')))

            def _txt(val, fmt='{:.2f}'):
                return fmt.format(val) if math.isfinite(val) else '—'

            cells = [
                str(u.get('callsign', '—')),
                str(int(u.get('sysid', 0))),
                _txt(miss, '{:.1f}'),
                _txt(brg, '{:.1f}'),
                _txt(tti, '{:.2f}'),
                _txt(float(u.get('impactLat', float('nan'))), '{:.6f}'),
                _txt(float(u.get('impactLon', float('nan'))), '{:.6f}'),
            ]
            for c, text in enumerate(cells):
                item = QTableWidgetItem(text)
                item.setTextAlignment(
                    Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignRight
                    if c >= 2 else
                    Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft
                )
                if c == 2:  # miss 欄上色
                    from PyQt6.QtGui import QColor
                    item.setForeground(QColor(_miss_color(miss)))
                tbl.setItem(r, c, item)

        hdr = tbl.horizontalHeader()
        hdr.setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        hdr.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        tbl.setMinimumHeight(220)
        return tbl

    # ── 動作 ─────────────────────────────────────────────────────────
    def _on_export_csv(self) -> None:
        """匯出 BDA 統計為 CSV，含每機列 + 摘要列。"""
        default_name = f"bda_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        path, _ = QFileDialog.getSaveFileName(
            self, "匯出 BDA 報告", default_name,
            "CSV files (*.csv);;All files (*.*)",
        )
        if not path:
            return
        try:
            with open(path, 'w', newline='', encoding='utf-8-sig') as f:
                w = csv.writer(f)
                summary = self._stats.get('summary', {})
                # 摘要區
                w.writerow(['# BDA Summary'])
                w.writerow(['Generated', datetime.now().isoformat(timespec='seconds')])
                w.writerow(['Target Lat', f"{float(summary.get('targetLat', 0)):.6f}"])
                w.writerow(['Target Lon', f"{float(summary.get('targetLon', 0)):.6f}"])
                w.writerow(['Mean Miss (m)', f"{float(summary.get('meanMissM', 0)):.2f}"])
                w.writerow(['Max Miss (m)', f"{float(summary.get('maxMissM', 0)):.2f}"])
                w.writerow(['TTT Spread (s)', f"{float(summary.get('ttSpreadS', 0)):.2f}"])
                w.writerow([])
                # 每機資料
                w.writerow(['# Per-UAV'])
                w.writerow([
                    'Callsign', 'sysid', 'Miss(m)', 'Bearing(deg)',
                    'TTI(s)', 'Impact Lat', 'Impact Lon',
                ])
                for u in self._stats.get('uavs', []):
                    w.writerow([
                        u.get('callsign', ''),
                        int(u.get('sysid', 0)),
                        f"{float(u.get('missM', float('nan'))):.2f}",
                        f"{float(u.get('bearingDeg', float('nan'))):.2f}",
                        f"{float(u.get('ttImpactS', float('nan'))):.2f}",
                        f"{float(u.get('impactLat', 0)):.6f}",
                        f"{float(u.get('impactLon', 0)):.6f}",
                    ])
            QMessageBox.information(
                self, "匯出成功",
                f"BDA 報告已儲存：\n{Path(path).name}"
            )
        except Exception as e:
            QMessageBox.critical(self, "匯出失敗", f"寫入 CSV 失敗：\n{e}")

    # ── helpers ─────────────────────────────────────────────────────
    @staticmethod
    def _btn_style(color: str) -> str:
        return (
            f"QPushButton {{ background: {T.BG_ELEVATED}; "
            f"color: {T.FG}; border: 1px solid {color}; "
            f"border-radius: 4px; padding: 6px 16px; "
            f"font-family: {T.FONT_DISPLAY_STACK}; font-weight: 700; "
            f"font-size: 11px; letter-spacing: 0.8px; }} "
            f"QPushButton:hover {{ background: {color}; color: {T.BG_PRIMARY}; }}"
        )


__all__ = ['StrikeBDADialog']
