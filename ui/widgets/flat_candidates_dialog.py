"""「找平坦位置」結果對話框：候選表 + 平移（可復原）。

⚠ 表上的「預估不用豁免」是**篩選**，不是保證：
  * 起伏是以作業區外框（外擴一個轉彎距離）算的，規劃器是逐胞判定；
  * 實測（桃園、僅 HERO、最低設定）預測可行的窗口大多「全覆蓋」，少數只覆蓋到 1 個胞以外 ⇒ 平移後請看規劃結果；
  * 不檢查起降場坡度（≤5°）與禁航區，平移後要重新確認起降場；
  * 地形是地表高度（含樹與建物），約 35 m 解析度。
"""
from __future__ import annotations

from typing import Any, Dict, List, Optional

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (QDialog, QHBoxLayout, QLabel, QPushButton, QTableWidget, QTableWidgetItem,
                             QVBoxLayout, QAbstractItemView, QHeaderView)

CLASS_NAME = {"A": "Pioneer", "B": "HERO"}
HEADERS = ["#", "中心（緯度, 經度）", "距離", "起伏", "需要 AGL 上限", "預估不用豁免", "餘裕"]


def summarize(payload: Dict[str, Any]) -> str:
    """結果標題（純文字，測試用）。"""
    cands: List[Dict[str, Any]] = payload.get("candidates") or []
    ceil = payload.get("ceil_m", 120.0)
    here = payload.get("here_relief_m")
    scale = float((payload.get("request") or {}).get("scale", 1.0))
    head = (f"搜尋窗口 {payload.get('window_h_m', 0):.0f} × {payload.get('window_w_m', 0):.0f} m"
            f"（目前作業區的 {scale * 100:.0f}%）；AGL 上限 {ceil:.0f} m"
            + (f"；目前位置起伏 {here:.0f} m" if here is not None else ""))
    fits = payload.get("any_fits") or {}
    if not cands:
        return head + "\n⚠ 搜尋範圍內沒有完整的陸地窗口。"
    if any(fits.values()):
        ok = "、".join(CLASS_NAME.get(c, c) for c, v in sorted(fits.items()) if v)
        return head + f"\n✔ 有位置預估 {ok} 不用豁免就容得下（見下表）。預估不是保證，平移後請看規劃結果。"
    best = min(cands, key=lambda c: min(c["need_m"].values()))
    cls = min(best["need_m"], key=best["need_m"].get)
    return (head + f"\n⚠ 範圍內沒有任何位置能不用豁免：最好的起伏 {best['relief_m']:.0f} m，"
            f"{CLASS_NAME.get(cls, cls)} 仍需 AGL 上限 {best['need_m'][cls]:.0f} m（> {ceil:.0f}）。"
            "可以把右上的「大小」調小再找，或取得豁免。")


def row_texts(c: Dict[str, Any]) -> List[str]:
    need = c.get("need_m") or {}
    fits = c.get("fits") or {}
    ok = "、".join(CLASS_NAME.get(k, k) for k, v in sorted(fits.items()) if v) or "—"
    slack = c.get("ceil_m", 120.0) - min(need.values()) if need else 0.0
    return [str(c["rank"]), f"{c['lat']:.4f}, {c['lon']:.4f}", f"{c['distance_km']:.1f} km", f"{c['relief_m']:.0f} m",
            " / ".join(f"{CLASS_NAME.get(k, k)} {v:.0f}" for k, v in sorted(need.items())), ok, f"{slack:+.0f} m"]


class FlatCandidatesDialog(QDialog):
    apply_requested = pyqtSignal(float, float, float)      # 中心緯度、經度、縮放倍率
    undo_requested = pyqtSignal()

    def __init__(self, payload: Dict[str, Any], parent=None, *, can_undo: bool = False) -> None:
        super().__init__(parent)
        self.setWindowTitle("找平坦位置")
        self.setMinimumWidth(780)
        self._payload = payload
        self._cands: List[Dict[str, Any]] = list(payload.get("candidates") or [])
        self._scale = float((payload.get("request") or {}).get("scale", 1.0))

        lay = QVBoxLayout(self)
        self.summary = QLabel(summarize(payload))
        self.summary.setWordWrap(True)
        lay.addWidget(self.summary)

        self.table = QTableWidget(len(self._cands), len(HEADERS))
        self.table.setHorizontalHeaderLabels(HEADERS)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        for r, c in enumerate(self._cands):
            for k, txt in enumerate(row_texts(c)):
                it = QTableWidgetItem(txt)
                if k:
                    it.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.table.setItem(r, k, it)
        self.table.itemSelectionChanged.connect(self._sync)
        lay.addWidget(self.table)

        self.note = QLabel(
            "起伏 = 作業區外框（外擴一個轉彎距離）內的最高 − 最低；地形為 Cesium World Terrain 地表高度（約 35 m 解析度）。\n"
            "平移會一併搬動起降場並清除目前的規劃結果；起降場坡度（≤5°）與禁航區不在這裡檢查，請平移後確認。\n"
            + ("資料來源：" + payload["attribution"] if payload.get("attribution") else ""))
        self.note.setWordWrap(True)
        self.note.setProperty("role", "caption")
        lay.addWidget(self.note)

        row = QHBoxLayout()
        self.btn_apply = QPushButton("平移作業區到此處" + ("（並縮成 %d%%）" % round(self._scale * 100) if self._scale < 0.999 else ""))
        self.btn_apply.setEnabled(False)
        self.btn_apply.clicked.connect(self._emit_apply)
        self.btn_undo = QPushButton("復原上次平移")
        self.btn_undo.setEnabled(bool(can_undo))
        self.btn_undo.clicked.connect(self.undo_requested.emit)
        self.btn_close = QPushButton("關閉")
        self.btn_close.clicked.connect(self.close)
        row.addWidget(self.btn_apply)
        row.addWidget(self.btn_undo)
        row.addStretch(1)
        row.addWidget(self.btn_close)
        lay.addLayout(row)
        if self._cands:
            self.table.selectRow(0)

    def selected(self) -> Optional[Dict[str, Any]]:
        rows = self.table.selectionModel().selectedRows() if self.table.selectionModel() else []
        return self._cands[rows[0].row()] if rows else None

    def set_can_undo(self, v: bool) -> None:
        self.btn_undo.setEnabled(bool(v))

    def _sync(self) -> None:
        self.btn_apply.setEnabled(self.selected() is not None)

    def _emit_apply(self) -> None:
        c = self.selected()
        if c is not None:
            self.apply_requested.emit(float(c["lat"]), float(c["lon"]), self._scale)
