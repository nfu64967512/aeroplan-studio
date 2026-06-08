"""ui/widgets/parameters_browser.py — ADOS 風格 ArduPilot 參數瀏覽器。

對應 ADOS `src/components/fc/parameters/`：
- 左 180px 類別側欄（"All" + 由 PARAM_VALUE 字首動態建類）
- 右 QTreeView + 8 欄（⭐ / # / Name / Description / Value / Range / Units / Type）
- 上方工具列：搜尋（debounce 150 ms）、Modified / Non-Default / Favorites 篩選
- 右上動作群：Export / Compare / Defaults Diff / Revert / Reset / Save / Refresh
- 模組層 cache TTL 5min（由 `mission.param_service.ParamService` 提供）

檔案 I/O 沿用 `sitl/plane/i1/identity.parm` 的 `NAME=VALUE` 一行一筆格式。
"""
from __future__ import annotations

import json
import logging
import time
from pathlib import Path
from typing import Dict, List, Optional

from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtWidgets import (
    QCheckBox,
    QFileDialog,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QListWidget,
    QMessageBox,
    QProgressBar,
    QSplitter,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

from mission.param_service import ParamRecord, ParamService
from ui.resources.aeroplan_theme import tokens as T
from ui.resources.aeroplan_theme.buttons import (
    ButtonSize,
    ButtonVariant,
    make_button,
)

logger = logging.getLogger(__name__)


class ParametersBrowser(QWidget):
    """ArduPilot 參數瀏覽器。"""

    save_requested = pyqtSignal(int, dict)   # sysid, {name: new_value}

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._sysid: Optional[int] = None
        self._params: Dict[str, ParamRecord] = {}
        self._pending_writes: Dict[str, float] = {}
        self._favorites: set[str] = set()
        self._metadata: Dict[str, dict] = self._load_metadata()
        self._last_sync_ts: Optional[float] = None

        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        # ── 上方工具列：搜尋 + 篩選 + 動作群 ─────────────────────
        toolbar = QHBoxLayout()
        toolbar.setContentsMargins(0, 0, 0, 0)
        toolbar.setSpacing(6)

        self._search = QLineEdit(self)
        self._search.setPlaceholderText("Search parameter name …")
        self._search.setMinimumWidth(240)
        self._search_timer = QTimer(self)
        self._search_timer.setSingleShot(True)
        self._search_timer.setInterval(150)
        self._search_timer.timeout.connect(self._refilter)
        self._search.textChanged.connect(lambda _: self._search_timer.start())
        toolbar.addWidget(self._search)

        self._chk_modified = QCheckBox("Modified", self)
        self._chk_modified.toggled.connect(self._refilter)
        toolbar.addWidget(self._chk_modified)

        self._chk_nondefault = QCheckBox("Non-Default", self)
        self._chk_nondefault.toggled.connect(self._refilter)
        toolbar.addWidget(self._chk_nondefault)

        self._chk_favorites = QCheckBox("★ Favorites", self)
        self._chk_favorites.toggled.connect(self._refilter)
        toolbar.addWidget(self._chk_favorites)

        toolbar.addStretch(1)

        # 右上動作群
        self._btn_export  = make_button("Export",  variant=ButtonVariant.OUTLINE, size=ButtonSize.SM)
        self._btn_import  = make_button("Import",  variant=ButtonVariant.OUTLINE, size=ButtonSize.SM)
        self._btn_revert  = make_button("Revert",  variant=ButtonVariant.OUTLINE, size=ButtonSize.SM)
        self._btn_save    = make_button("Save",    variant=ButtonVariant.PRIMARY, size=ButtonSize.SM)
        self._btn_refresh = make_button("Refresh", variant=ButtonVariant.SECONDARY, size=ButtonSize.SM)
        for b in (self._btn_export, self._btn_import, self._btn_revert, self._btn_save, self._btn_refresh):
            toolbar.addWidget(b)

        self._btn_export.clicked.connect(self._on_export)
        self._btn_import.clicked.connect(self._on_import)
        self._btn_revert.clicked.connect(self._on_revert)
        self._btn_save.clicked.connect(self._on_save)
        self._btn_refresh.clicked.connect(self._on_refresh)

        root.addLayout(toolbar)

        # ── 主體：QSplitter 左類別 / 右 grid ─────────────────────
        body = QSplitter(Qt.Orientation.Horizontal, self)

        self._categories = QListWidget(body)
        self._categories.setFixedWidth(180)
        self._categories.currentItemChanged.connect(lambda *_: self._refilter())
        body.addWidget(self._categories)

        self._tree = QTreeWidget(body)
        self._tree.setColumnCount(7)
        self._tree.setHeaderLabels(
            ["★", "Name", "Value", "Description", "Range", "Units", "Type"]
        )
        self._tree.setRootIsDecorated(False)
        self._tree.setAlternatingRowColors(True)
        self._tree.setSortingEnabled(True)
        self._tree.itemDoubleClicked.connect(self._on_double_click)
        hdr = self._tree.header()
        hdr.setSectionResizeMode(0, QHeaderView.ResizeMode.Fixed)
        hdr.resizeSection(0, 28)
        hdr.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        hdr.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        hdr.setSectionResizeMode(3, QHeaderView.ResizeMode.Stretch)
        body.addWidget(self._tree)

        body.setStretchFactor(0, 0)
        body.setStretchFactor(1, 1)
        root.addWidget(body, 1)

        # ── 底部狀態列 ─────────────────────────────────────
        bottom = QHBoxLayout()
        bottom.setContentsMargins(0, 0, 0, 0)
        bottom.setSpacing(6)
        self._status_label = QLabel("No data", self)
        self._status_label.setStyleSheet(
            f"font-family: {T.FONT_MONO_STACK}; font-size: 10px; "
            f"color: {T.FG_MUTED};"
        )
        self._progress = QProgressBar(self)
        self._progress.setRange(0, 100)
        self._progress.setFixedHeight(8)
        self._progress.setTextVisible(False)
        self._progress.hide()
        bottom.addWidget(self._status_label)
        bottom.addStretch(1)
        bottom.addWidget(self._progress, 0)
        root.addLayout(bottom)

        # ── 接 ParamService ─────────────────────────────
        ps = ParamService.instance()
        ps.param_list_progress.connect(self._on_progress)
        ps.param_list_complete.connect(self._on_complete)
        ps.param_set_ack.connect(self._on_set_ack)
        ps.param_error.connect(self._on_error)

    # ── 公開 API ──────────────────────────────────────────────
    def set_current_sysid(self, sysid: int) -> None:
        """切換目標 UAV；自動使用快取或觸發 refresh。"""
        self._sysid = sysid
        cache = ParamService.instance().cached(sysid)
        if cache:
            self._params = dict(cache)
            self._rebuild_categories()
            self._refilter()
            self._update_status()
        else:
            self._on_refresh()

    # ── ParamService callbacks ────────────────────────────
    def _on_progress(self, sysid: int, total: int, received: int) -> None:
        if sysid != self._sysid:
            return
        self._progress.show()
        self._progress.setRange(0, max(1, total))
        self._progress.setValue(received)
        self._status_label.setText(f"Loading {received}/{total} parameters …")

    def _on_complete(self, sysid: int, params: dict) -> None:
        if sysid != self._sysid:
            return
        self._progress.hide()
        self._params = dict(params)
        self._last_sync_ts = time.monotonic()
        self._rebuild_categories()
        self._refilter()
        self._update_status()

    def _on_set_ack(self, sysid: int, name: str, accepted: bool) -> None:
        if sysid != self._sysid:
            return
        if accepted:
            self._pending_writes.pop(name, None)
            self._update_status()

    def _on_error(self, sysid: int, msg: str) -> None:
        if sysid != self._sysid:
            return
        self._progress.hide()
        self._status_label.setText(f"Error: {msg}")

    # ── 重建 UI ──────────────────────────────────────────────
    def _rebuild_categories(self) -> None:
        self._categories.clear()
        prefixes: Dict[str, int] = {}
        for name in self._params:
            head = name.split("_", 1)[0] if "_" in name else name
            prefixes[head] = prefixes.get(head, 0) + 1
        all_item = self._categories.addItem(f"All ({len(self._params)})")
        for head in sorted(prefixes):
            self._categories.addItem(f"{head}_* ({prefixes[head]})")
        self._categories.setCurrentRow(0)

    def _refilter(self) -> None:
        """重畫 tree。依分類 + 搜尋 + checkbox 篩選。"""
        self._tree.clear()
        if not self._params:
            return
        q = (self._search.text() or "").upper()
        only_modified = self._chk_modified.isChecked()
        only_nondefault = self._chk_nondefault.isChecked()
        only_favorites = self._chk_favorites.isChecked()
        cur = self._categories.currentItem()
        cat_filter = cur.text() if cur else ""
        cat_prefix: Optional[str] = None
        if cat_filter and not cat_filter.startswith("All"):
            cat_prefix = cat_filter.split("_*")[0] + "_"

        for name in sorted(self._params):
            if cat_prefix and not name.startswith(cat_prefix):
                continue
            if q and q not in name:
                continue
            if only_modified and name not in self._pending_writes:
                continue
            if only_favorites and name not in self._favorites:
                continue
            rec = self._params[name]
            meta = self._metadata.get(name, {})
            default_val = meta.get("default")
            if only_nondefault and default_val is not None and abs(rec.value - float(default_val)) < 1e-9:
                continue

            item = QTreeWidgetItem([
                "★" if name in self._favorites else "",
                name,
                self._format_value(rec.value),
                meta.get("description", ""),
                meta.get("range", ""),
                meta.get("units", ""),
                self._format_type(rec.type_code),
            ])
            if name in self._pending_writes:
                item.setForeground(2, item.foreground(2))  # 視覺標記留給 stylesheet 處理
                item.setText(2, f"{self._format_value(self._pending_writes[name])}*")
            self._tree.addTopLevelItem(item)

    # ── 互動 ──────────────────────────────────────────────────
    def _on_double_click(self, item: QTreeWidgetItem, col: int) -> None:
        if col != 2:
            # 雙擊 ★ 欄位切 favorite
            if col == 0:
                name = item.text(1)
                if name in self._favorites:
                    self._favorites.remove(name)
                else:
                    self._favorites.add(name)
                self._refilter()
            return
        # 雙擊 value 欄 → 改值
        name = item.text(1)
        rec = self._params.get(name)
        if rec is None:
            return
        from PyQt6.QtWidgets import QInputDialog
        new_val, ok = QInputDialog.getDouble(
            self, "Edit parameter",
            f"{name}\n\nCurrent: {rec.value}",
            value=rec.value, decimals=6,
        )
        if not ok:
            return
        self._pending_writes[name] = float(new_val)
        self._refilter()
        self._update_status()

    def _on_save(self) -> None:
        if self._sysid is None:
            return
        if not self._pending_writes:
            QMessageBox.information(self, "Save", "No pending changes.")
            return
        # 確認對話框
        lines = [
            f"{name}:  {self._params[name].value}  →  {new}"
            for name, new in self._pending_writes.items()
        ]
        if QMessageBox.question(
            self, "Confirm parameter writes",
            "Write the following parameter changes?\n\n" + "\n".join(lines),
        ) != QMessageBox.StandardButton.Yes:
            return
        for name, value in list(self._pending_writes.items()):
            ParamService.instance().set_param(self._sysid, name, value)
        self.save_requested.emit(self._sysid, dict(self._pending_writes))

    def _on_revert(self) -> None:
        self._pending_writes.clear()
        self._refilter()
        self._update_status()

    def _on_refresh(self) -> None:
        if self._sysid is None:
            self._status_label.setText("No active UAV selected.")
            return
        self._params.clear()
        self._pending_writes.clear()
        self._tree.clear()
        ok = ParamService.instance().request_all(self._sysid)
        if ok:
            self._progress.show()
            self._status_label.setText("Requesting parameters …")

    def _on_export(self) -> None:
        if not self._params:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Export parameters",
            f"params_sysid{self._sysid}.parm", "Parameter (*.parm)"
        )
        if not path:
            return
        lines = [
            f"{name}\t{rec.value}\n"
            for name, rec in sorted(self._params.items())
        ]
        Path(path).write_text("".join(lines), encoding="utf-8")
        QMessageBox.information(self, "Export", f"Wrote {len(lines)} params.")

    def _on_import(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Import parameters", "", "Parameter (*.parm *.txt)"
        )
        if not path:
            return
        loaded: Dict[str, float] = {}
        for line in Path(path).read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.replace("=", " ").split()
            if len(parts) < 2:
                continue
            try:
                loaded[parts[0].upper()] = float(parts[1])
            except ValueError:
                continue
        # 比對成 pending_writes
        diff_n = 0
        for name, value in loaded.items():
            cur = self._params.get(name)
            if cur is not None and cur.value != value:
                self._pending_writes[name] = value
                diff_n += 1
        QMessageBox.information(
            self, "Import",
            f"Loaded {len(loaded)} params; {diff_n} differ — press Save to apply.",
        )
        self._refilter()
        self._update_status()

    # ── helpers ──────────────────────────────────────────────
    @staticmethod
    def _format_value(v: float) -> str:
        if v == int(v):
            return str(int(v))
        return f"{v:.6f}".rstrip("0").rstrip(".")

    @staticmethod
    def _format_type(tc: int) -> str:
        return {
            1: "i8", 2: "u8", 3: "i16", 4: "u16",
            5: "i32", 6: "u32", 7: "i64", 8: "u64",
            9: "f32", 10: "f64",
        }.get(int(tc), f"t{tc}")

    def _update_status(self) -> None:
        total = len(self._params)
        modified = len(self._pending_writes)
        age = (
            int(time.monotonic() - self._last_sync_ts)
            if self._last_sync_ts else None
        )
        age_str = f" · synced {age}s ago" if age is not None else ""
        self._status_label.setText(
            f"{total} params · {modified} modified{age_str}"
        )

    @staticmethod
    def _load_metadata() -> Dict[str, dict]:
        p = Path(__file__).resolve().parents[2] / "data" / "parameter_metadata.json"
        if not p.exists():
            return {}
        try:
            return json.loads(p.read_text(encoding="utf-8"))
        except Exception as exc:
            logger.warning("ParametersBrowser: failed to load metadata: %s", exc)
            return {}


__all__ = ["ParametersBrowser"]
