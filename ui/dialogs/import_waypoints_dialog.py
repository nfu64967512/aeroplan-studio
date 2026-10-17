# -*- coding: utf-8 -*-
"""匯入多個 Mission Planner .waypoints（QGC WPL 110）／QGC .plan，每檔**明確指定給一架**（sysid）。

``ImportWaypointsDialog(parent=None, paths=(), links_sysids=(), last_dir='')``

* ``paths``：一開始就載入的檔案；之後可按「加入檔案…」再加（QFileDialog 多選）。
* ``links_sysids``：目前連線中的 sysid。下拉只列這些（＋檔名猜到但沒連線的那個，標「未連線」）；
  **沒有任何連線**時可指派 1–255 任一個，但每一項都標「未連線」。
* ``last_dir``：QFileDialog 的起始資料夾。

表格：檔名｜sysid 下拉｜項目數｜起降｜錯誤／警告；底下是每個檔的 issues 全文。
「確定」只有在 ``mission.import_missions.assignment_problems()`` 回空時才能按，理由逐條列在紅字。
``result()`` 回傳已指派的 ``list[ImportedMission]``（**只有按了確定才有東西**，取消是空清單）。

有效值契約：表格顯示的 sysid 就是 ``result()`` 送出去的 sysid；檔名猜到的只是下拉的**預設值**，
一定要經過這個對話框讓使用者看到、按確定才算指派（這個專案發生過「兩架拿到同一份任務」）。
有錯誤的檔**不會**被靜靜丟掉或修正 —— 它擋住整個「確定」，使用者得自己把它移除或修好再來。

⚠ ``result()`` 遮掉了 ``QDialog.result()``（那個回傳 int 的對話框代碼）。對話框代碼請用
``exec()`` 的回傳值或 ``accepted`` 屬性；這裡照呼叫端的約定回傳任務清單。
"""
from __future__ import annotations

import os
from typing import Iterable, List, Optional, Sequence

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (QAbstractItemView, QComboBox, QDialog, QFileDialog, QHBoxLayout,
                             QHeaderView, QLabel, QPlainTextEdit, QPushButton, QTableWidget,
                             QTableWidgetItem, QVBoxLayout)

from mission import import_missions as im
from mission import mission_view as _mv
from ui.resources.aeroplan_theme.widgets import IconButton
from utils.logger import get_logger

logger = get_logger()

HEADERS = ['檔名', 'sysid', '項目數', '起降', '錯誤／警告']
FILE_FILTER = ('Mission Planner / QGC waypoints (*.waypoints *.txt);;'
               'QGroundControl plan (*.plan);;All files (*)')
PLACEHOLDER = '— 請選擇 —'
COL_NAME, COL_SYSID, COL_ITEMS, COL_TOL, COL_ISSUES = range(5)


class ImportWaypointsDialog(QDialog):
    def __init__(self, parent=None, paths: Iterable = (), links_sysids: Sequence[int] = (),
                 last_dir: str = ''):
        super().__init__(parent)
        self.setWindowTitle('匯入多個航點檔（每檔指定一架）')
        self.setMinimumSize(860, 520)
        self._links: List[int] = sorted({int(s) for s in (links_sysids or ())})
        self._last_dir = str(last_dir or '')
        self._missions: List[im.ImportedMission] = []
        self._accepted: List[im.ImportedMission] = []
        self._combos: List[QComboBox] = []
        # 使用者碰過下拉的檔（路徑鍵）。_refresh() 重建表格時，碰過的列一律照 m.sysid 還原 ——
        # 包括使用者**明確選回「請選擇」(None)** 的；沒碰過的才拿檔名猜的當預設。
        # 以前只看 `m.sysid is not None`，於是清成 None 的列在加入／移除檔案後又被猜回去。
        self._user_set: set = set()
        self._build_ui()
        self.add_paths(paths)

    @staticmethod
    def _pkey(path) -> str:
        return os.path.normcase(os.path.abspath(os.fspath(path)))

    # ─────────────────────────────────────────────────────────────────
    # UI
    # ─────────────────────────────────────────────────────────────────
    def _build_ui(self):
        lay = QVBoxLayout(self)
        hint = QLabel('每個檔案要**明確**指定給一架（sysid）。檔名猜到的只是預設值，請逐一確認；'
                      '同一個 sysid 不能指派兩個檔，有錯誤的檔要先移除或修正。')
        hint.setTextFormat(Qt.TextFormat.PlainText)
        hint.setWordWrap(True)
        lay.addWidget(hint)
        self.lbl_links = QLabel('')
        self.lbl_links.setWordWrap(True)
        lay.addWidget(self.lbl_links)
        self._refresh_links_label()

        self.table = QTableWidget(0, len(HEADERS))
        self.table.setHorizontalHeaderLabels(HEADERS)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        hdr = self.table.horizontalHeader()
        hdr.setSectionResizeMode(COL_NAME, QHeaderView.ResizeMode.Stretch)
        hdr.setSectionResizeMode(COL_ISSUES, QHeaderView.ResizeMode.Stretch)
        lay.addWidget(self.table, 2)

        row = QHBoxLayout()
        self.btn_add = IconButton('import', '加入檔案…', tone='neutral', compact=True)
        self.btn_add.clicked.connect(self._on_add_files)
        self.btn_remove = IconButton('clear', '移除選取', tone='danger', compact=True)
        self.btn_remove.clicked.connect(self.remove_selected)
        row.addWidget(self.btn_add)
        row.addWidget(self.btn_remove)
        row.addStretch(1)
        lay.addLayout(row)

        self.lbl_problems = QLabel('')
        self.lbl_problems.setWordWrap(True)
        self.lbl_problems.setStyleSheet('color:#E53935;')
        lay.addWidget(self.lbl_problems)

        self.issues_view = QPlainTextEdit()
        self.issues_view.setReadOnly(True)
        self.issues_view.setPlaceholderText('每個檔案的驗證結果（錯誤／警告）會列在這裡')
        lay.addWidget(self.issues_view, 1)

        brow = QHBoxLayout()
        brow.addStretch(1)
        self.btn_ok = IconButton('confirm', '確定', tone='primary')
        self.btn_ok.clicked.connect(self._on_ok)
        self.btn_cancel = QPushButton('取消')
        self.btn_cancel.clicked.connect(self.reject)
        brow.addWidget(self.btn_ok)
        brow.addWidget(self.btn_cancel)
        lay.addLayout(brow)

    def _refresh_links_label(self):
        if self._links:
            self.lbl_links.setText('連線中的 UAV：' + '、'.join(f'UAV-{s}' for s in self._links))
            self.lbl_links.setStyleSheet('')
        else:
            self.lbl_links.setText('目前沒有任何連線：可先指派 1–255 任一個 sysid（全部標「未連線」），'
                                   '寫入飛控時仍要有對應的連線。')
            self.lbl_links.setStyleSheet('color:#FB8C00;')

    # ─────────────────────────────────────────────────────────────────
    # 檔案進出
    # ─────────────────────────────────────────────────────────────────
    def add_paths(self, paths: Iterable) -> int:
        """載入並加進表格；同一個路徑已在清單裡就跳過。回傳實際加入的檔數。"""
        known = {self._pkey(m.path) for m in self._missions}
        fresh = []
        for p in (paths or ()):
            key = self._pkey(p)
            if key in known:
                logger.info(f'[匯入航點] 已在清單裡，略過：{p}')
                continue
            known.add(key)
            fresh.append(os.fspath(p))
        if fresh:
            self._missions.extend(im.load_many(fresh))
            self._last_dir = os.path.dirname(fresh[-1]) or self._last_dir
        self._refresh()
        return len(fresh)

    def remove_selected(self) -> bool:
        row = self.table.currentRow()
        if row < 0 or row >= len(self._missions):
            return False
        gone = self._missions.pop(row)
        self._user_set.discard(self._pkey(gone.path))
        logger.info(f'[匯入航點] 移除 {gone.label}')
        self._refresh()
        return True

    def _on_add_files(self):
        paths, _flt = QFileDialog.getOpenFileNames(self, '加入航點檔', self._last_dir, FILE_FILTER)
        if paths:
            self.add_paths(paths)

    # ─────────────────────────────────────────────────────────────────
    # 表格
    # ─────────────────────────────────────────────────────────────────
    def _sysid_choices(self, suggested: Optional[int]) -> List[tuple]:
        """(sysid, 顯示文字) 清單；第一項是「請選擇」(None)。"""
        choices: List[tuple] = [(None, PLACEHOLDER)]
        if self._links:
            for s in self._links:
                choices.append((s, f'UAV-{s}'))
            if suggested is not None and suggested not in self._links:
                choices.append((suggested, f'UAV-{suggested}（未連線）'))
        else:
            for s in range(im.SYSID_MIN, im.SYSID_MAX + 1):
                choices.append((s, f'UAV-{s}（未連線）'))
        return choices

    def _refresh(self):
        """整張表依 self._missions 重建（含下拉），再重算問題清單與「確定」。"""
        # 同名檔（不同資料夾）要分得出來：撞名的 label 變 `<上層資料夾>/<檔名>`。
        # 撞名的對象可能是早就在清單裡的那個，所以每次重建都對**整份**清單算一次。
        im.disambiguate_labels(self._missions)
        sums = im.summaries(self._missions)
        self.table.blockSignals(True)
        self.table.setRowCount(len(self._missions))
        self._combos = []
        for r, (m, s) in enumerate(zip(self._missions, sums)):
            name = QTableWidgetItem(m.label)
            name.setToolTip(m.path)
            self.table.setItem(r, COL_NAME, name)

            cmb = QComboBox()
            for sid, text in self._sysid_choices(m.suggested_sysid):
                cmb.addItem(text, sid)
            # 預設值：使用者碰過的列照 m.sysid 還原（**含明確選回 None 的**，重建表格不可以
            # 把使用者的選擇洗掉或猜回去）；沒碰過的才拿檔名猜到的當預設。
            touched = self._pkey(m.path) in self._user_set
            want = m.sysid if (touched or m.sysid is not None) else m.suggested_sysid
            idx = cmb.findData(want) if want is not None else 0
            cmb.setCurrentIndex(idx if idx >= 0 else 0)
            m.sysid = cmb.currentData()
            if m.suggested_sysid is not None:
                cmb.setToolTip(f'檔名猜到 sysid {m.suggested_sysid}（只是預設值，請確認）')
            cmb.currentIndexChanged.connect(lambda _i, row=r: self._on_sysid_changed(row))
            self.table.setCellWidget(r, COL_SYSID, cmb)
            self._combos.append(cmb)

            self.table.setItem(r, COL_ITEMS, QTableWidgetItem(str(s['n_items'])))
            tol = f"起飛{'✓' if s['has_takeoff'] else '✗'}／降落{'✓' if s['has_land'] else '✗'}"
            self.table.setItem(r, COL_TOL, QTableWidgetItem(tol))
            iss = QTableWidgetItem(f"{s['issues_error']} 錯誤／{s['issues_warning']} 警告")
            if s['issues_error']:
                iss.setForeground(Qt.GlobalColor.red)
            elif s['issues_warning']:
                iss.setForeground(Qt.GlobalColor.darkYellow)
            self.table.setItem(r, COL_ISSUES, iss)
            color = _mv.sysid_color(m.sysid) if m.sysid is not None else None
            if color:
                name.setToolTip(f'{m.path}\n地圖顏色 {color}')
        self.table.blockSignals(False)
        self._refresh_issues()
        self._refresh_problems()

    def _on_sysid_changed(self, row: int):
        if 0 <= row < len(self._missions) and row < len(self._combos):
            m = self._missions[row]
            m.sysid = self._combos[row].currentData()
            self._user_set.add(self._pkey(m.path))     # 從此這一列只聽使用者的，不再猜
        self._refresh_problems()

    def _refresh_issues(self):
        blocks = []
        for m in self._missions:
            lines = [m.loaded.summary_text()]
            if m.suggested_sysid is not None:
                lines.append(f'  [提示] 檔名猜到 sysid {m.suggested_sysid}（預設值，請確認）')
            blocks.append('\n'.join(lines))
        self.issues_view.setPlainText('\n\n'.join(blocks))

    def _refresh_problems(self):
        probs = self.problems()
        self.lbl_problems.setText('\n'.join(f'✘ {p}' for p in probs))
        self.btn_ok.setEnabled(not probs)
        self.btn_remove.setEnabled(bool(self._missions))

    # ─────────────────────────────────────────────────────────────────
    # 對外
    # ─────────────────────────────────────────────────────────────────
    def problems(self) -> List[str]:
        """目前擋住「確定」的理由（`mission.import_missions.assignment_problems`）。"""
        return im.assignment_problems(self._missions)

    def missions(self) -> List[im.ImportedMission]:
        """表格裡目前的清單（含未指派／有錯誤的），給測試與主視窗看狀態用。"""
        return list(self._missions)

    def set_sysid(self, row: int, sysid: Optional[int]) -> bool:
        """程式化地替第 row 列選 sysid（等同使用者操作下拉）；選項裡沒有就回 False、不改。"""
        if not 0 <= row < len(self._combos):
            return False
        cmb = self._combos[row]
        idx = cmb.findData(sysid) if sysid is not None else 0
        if idx < 0:
            return False
        cmb.setCurrentIndex(idx)
        self._on_sysid_changed(row)
        return True

    def result(self) -> List[im.ImportedMission]:  # noqa: D401 - 呼叫端約定；遮掉 QDialog.result()
        """按了「確定」後的已指派清單；取消或還沒按是空清單。"""
        return list(self._accepted)

    def _on_ok(self):
        probs = self.problems()
        if probs:
            # 按鈕本來就該是灰的；這裡再擋一次是為了「兩個地方算的是同一份規則」
            self.lbl_problems.setText('\n'.join(f'✘ {p}' for p in probs))
            self.btn_ok.setEnabled(False)
            return
        self._accepted = [im.ImportedMission(path=m.path, sysid=int(m.sysid), loaded=m.loaded,
                                             label=m.label, suggested_sysid=m.suggested_sysid)
                          for m in self._missions]
        logger.info('[匯入航點] 指派：' + '、'.join(f'{m.label}→UAV-{m.sysid}' for m in self._accepted))
        self.accept()
