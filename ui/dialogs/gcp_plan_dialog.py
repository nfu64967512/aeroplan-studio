"""GCP／檢核點規劃對話框（工具 →「GCP／檢核點規劃…」，或航測處理對話框裡的按鈕）。

``GcpPlanDialog(parent=None, map_widget=None, polygons=None, blocks=None)``

* ``polygons``：作業區 —— ring 的串列，或回傳它的可呼叫物件（建議傳 ``MainWindow.survey_area_polygons``；
  非模態，每次按「產生」時才取最新值）。None ⇒ 用地圖上正在畫的角點（``map_widget.corners``）。
* ``blocks``：每架的 UAV 區塊 —— None（沒有 HARVEST 規劃）、``{標籤: ring 的串列}``，或回傳
  ``(blocks, info, 來源說明)``／``(blocks, info)``／``blocks`` 的可呼叫物件（MainWindow 用
  ``survey.gcp_plan.blocks_from_hv`` 從 HvDccppCoordinator 唯讀取出）。

選方案（交叉航帶＋最少 GCP／加密 GCP）與點數 → 產生 → 看點數、警告、逐條規則核對、兩方案比較 → 匯出
（CSV、KML、GeoJSON、ODM gcp_list 樣板、外業表、外業填寫 CSV）→ 地圖上顯示（GCP 紅、檢核點藍，帶編號）。

有效值契約：空白欄位＝預設值（由足跡推得），實際用的值與來源寫在「有效設定」；做不到的規則以紅字警告列出，
不靜默放寬；HARVEST 區塊取不到時**不**默默改成沒有區塊 —— 要使用者自己取消勾選「使用 HARVEST 區塊」。
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (QAbstractItemView, QCheckBox, QComboBox, QDialog, QDoubleSpinBox, QFileDialog,
                             QFormLayout, QGroupBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit, QPlainTextEdit,
                             QPushButton, QSpinBox, QTableWidget, QTableWidgetItem, QTabWidget, QVBoxLayout, QWidget)

from utils.logger import get_logger

logger = get_logger()

MAP_LAYER = "gcp_plan"
POINT_HEADERS = ["編號", "類型", "緯度", "經度", "區塊", "用途", "備註"]
COMPARE_HEADERS = ["方案", "GCP", "檢核點", "總點數", "接縫", "接縫檢核點對", "警告", "加飛"]
TRI = ("方案預設", "是", "否")
AREAS_MAIN_SRC = "主視窗的作業區（已完成的區＋正在畫且 ≥3 角點的區）"
AREAS_MAP_SRC = "地圖上正在畫的測區角點"


def _opt_float(text: str, what: str, errors: List[str], *, allow_zero: bool = True) -> Optional[float]:
    s = (text or "").strip()
    if not s:
        return None
    try:
        v = float(s)
    except ValueError:
        errors.append(f"{what} 不是數字：{s!r}")
        return None
    if not math.isfinite(v) or v < 0 or (v == 0 and not allow_zero):
        errors.append(f"{what} 要是 {'≥' if allow_zero else '>'} 0 的有限數值：{s!r}")
        return None
    return v


class GcpPlanDialog(QDialog):
    def __init__(self, parent=None, map_widget=None, polygons=None, blocks=None):
        super().__init__(parent)
        from survey import gcp_plan
        self._gp = gcp_plan
        self.setWindowTitle("GCP／檢核點規劃（交叉航帶 vs 加密 GCP）")
        self.setMinimumWidth(980)
        self.map_widget = map_widget
        self.polygons_source = polygons
        self.blocks_source = blocks
        self.plan = None
        self.comparison = None
        self.exported: Optional[dict] = None
        self._build_ui()
        self._on_preset_changed()
        self._refresh_sources()

    # ─────────────────────────────────────────────────────────────────
    # UI
    # ─────────────────────────────────────────────────────────────────
    def _build_ui(self):
        gp = self._gp
        lay = QVBoxLayout(self)
        top = QHBoxLayout()
        lay.addLayout(top)

        grp = QGroupBox("方案與點數")
        f = QFormLayout(grp)
        self.cmb_preset = QComboBox()
        for name, pr in gp.PRESETS.items():
            self.cmb_preset.addItem(pr.label, name)
        self.cmb_preset.currentIndexChanged.connect(lambda _: self._on_preset_changed())
        f.addRow("方案", self.cmb_preset)
        self.lbl_preset = QLabel("")
        self.lbl_preset.setWordWrap(True)
        f.addRow(self.lbl_preset)
        crow = QHBoxLayout()
        self.spn_chk = QSpinBox()
        self.spn_chk.setRange(0, 200)
        self.spn_chk.setValue(20)
        self.chk_asprs30 = QCheckBox("ASPRS 第 2 版：30 個")
        self.chk_asprs30.toggled.connect(self._on_asprs_toggled)
        crow.addWidget(self.spn_chk)
        crow.addWidget(self.chk_asprs30)
        f.addRow("檢核點數", crow)
        self.cmb_per_block = QComboBox()
        self.cmb_per_block.addItems(TRI)
        self.cmb_seam_gcp = QComboBox()
        self.cmb_seam_gcp.addItems(TRI)
        f.addRow("每個 UAV 區塊 +1 GCP", self.cmb_per_block)
        f.addRow("每條跨機接縫 +1 GCP", self.cmb_seam_gcp)
        self.spn_split = QSpinBox()
        self.spn_split.setRange(0, 50)
        self.spn_split.setSpecialValueText("不分割")
        f.addRow("ODM split 子模型數", self.spn_split)
        self.spn_seed = QSpinBox()
        self.spn_seed.setRange(0, 2_000_000_000)
        f.addRow("隨機種子（同種子同結果）", self.spn_seed)
        top.addWidget(grp, 1)

        grp2 = QGroupBox("足跡與間距（空白 = 預設，括號內是預設的由來）")
        g = QFormLayout(grp2)
        self.spn_w = QDoubleSpinBox()
        self.spn_w.setRange(1.0, 5000.0)
        self.spn_w.setDecimals(1)
        self.spn_w.setSuffix(" m")
        self.spn_w.setValue(gp.DEFAULT_FOOTPRINT_W_M)
        self.spn_l = QDoubleSpinBox()
        self.spn_l.setRange(1.0, 5000.0)
        self.spn_l.setDecimals(1)
        self.spn_l.setSuffix(" m")
        self.spn_l.setValue(gp.DEFAULT_FOOTPRINT_L_M)
        g.addRow("單張足跡寬 w（⟂ 航向）", self.spn_w)
        g.addRow("單張足跡長 l（∥ 航向）", self.spn_l)
        g.addRow(QLabel("預設是 HQ6 相機、120 m AGL（125.7 × 94.2 m）；換相機或高度請改這兩格。"))
        self.ed_edge_band = QLineEdit()
        self.ed_edge_band.setPlaceholderText("空白 = w/2（外緣低視數帶）")
        self.spn_min_inset = QDoubleSpinBox()
        self.spn_min_inset.setRange(0.0, 1000.0)
        self.spn_min_inset.setDecimals(1)
        self.spn_min_inset.setSuffix(" m")
        self.spn_min_inset.setValue(30.0)
        self.ed_gcp_spacing = QLineEdit()
        self.ed_gcp_spacing.setPlaceholderText("空白 = max(w, l)（ODM：大於單張足跡）")
        self.ed_chk_gcp = QLineEdit()
        self.ed_chk_gcp.setPlaceholderText("空白 = l（一個足跡長）")
        self.ed_seam_offset = QLineEdit()
        self.ed_seam_offset.setPlaceholderText("空白 = w/2")
        g.addRow("邊緣帶寬", self.ed_edge_band)
        g.addRow("最小內縮", self.spn_min_inset)
        g.addRow("GCP 最小間距", self.ed_gcp_spacing)
        g.addRow("檢核點離 GCP 至少", self.ed_chk_gcp)
        g.addRow("接縫檢核點離對方區塊", self.ed_seam_offset)
        top.addWidget(grp2, 1)

        src = QGroupBox("來源")
        sf = QFormLayout(src)
        self.lbl_areas = QLabel("")
        self.lbl_areas.setWordWrap(True)
        self.chk_use_blocks = QCheckBox("使用 HARVEST 區塊（每架的條帶；接縫檢核點對需要它）")
        self.chk_use_blocks.setChecked(True)
        self.chk_use_blocks.toggled.connect(lambda _: self._refresh_sources())
        self.lbl_blocks = QLabel("")
        self.lbl_blocks.setWordWrap(True)
        sf.addRow("作業區", self.lbl_areas)
        sf.addRow(self.chk_use_blocks)
        sf.addRow("UAV 區塊", self.lbl_blocks)
        lay.addWidget(src)

        row = QHBoxLayout()
        self.btn_generate = QPushButton("產生")
        self.btn_generate.clicked.connect(self.generate)
        self.chk_show_map = QCheckBox("顯示在地圖上")
        self.chk_show_map.setChecked(True)
        self.btn_clear_map = QPushButton("清除地圖上的點")
        self.btn_clear_map.clicked.connect(self.clear_map)
        row.addWidget(self.btn_generate)
        row.addWidget(self.chk_show_map)
        row.addWidget(self.btn_clear_map)
        row.addStretch(1)
        lay.addLayout(row)

        self.lbl_errors = QLabel("")
        self.lbl_errors.setWordWrap(True)
        self.lbl_errors.setStyleSheet("color:#E53935;")
        lay.addWidget(self.lbl_errors)
        self.lbl_counts = QLabel("尚未產生")
        self.lbl_counts.setWordWrap(True)
        lay.addWidget(self.lbl_counts)
        self.lbl_warnings = QLabel("")
        self.lbl_warnings.setWordWrap(True)
        self.lbl_warnings.setStyleSheet("color:#E53935;")
        lay.addWidget(self.lbl_warnings)

        self.tabs = QTabWidget()
        lay.addWidget(self.tabs, 1)
        self.points_table = QTableWidget(0, len(POINT_HEADERS))
        self.points_table.setHorizontalHeaderLabels(POINT_HEADERS)
        self.points_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.points_table.horizontalHeader().setSectionResizeMode(6, QHeaderView.ResizeMode.Stretch)
        self.tabs.addTab(self.points_table, "點位")
        self.compare_table = QTableWidget(0, len(COMPARE_HEADERS))
        self.compare_table.setHorizontalHeaderLabels(COMPARE_HEADERS)
        self.compare_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.compare_table.horizontalHeader().setSectionResizeMode(7, QHeaderView.ResizeMode.Stretch)
        self.tabs.addTab(self.compare_table, "兩方案比較")
        self.checks_view = QPlainTextEdit()
        self.checks_view.setReadOnly(True)
        self.tabs.addTab(self.checks_view, "規則核對與有效設定")
        self.tabs.addTab(self._build_export_tab(), "匯出")

        brow = QHBoxLayout()
        brow.addStretch(1)
        self.btn_close = QPushButton("關閉")
        self.btn_close.clicked.connect(self.close)
        brow.addWidget(self.btn_close)
        lay.addLayout(brow)

    def _build_export_tab(self) -> QWidget:
        gp = self._gp
        w = QWidget()
        f = QFormLayout(w)
        self.cmb_projection = QComboBox()
        self.cmb_projection.setEditable(True)
        self.cmb_projection.addItems(["EPSG:4326"])
        f.addRow("gcp_list 標頭（geo_x geo_y 的座標系）", self.cmb_projection)
        self.lbl_projection = QLabel("ODM 只認 WGS84 UTM <區><N|S>、+proj=…、EPSG:xxxx 三種寫法。EPSG:4326 時 geo_x = 經度。")
        self.lbl_projection.setWordWrap(True)
        f.addRow(self.lbl_projection)
        self.cmb_vertical = QComboBox()
        self.cmb_vertical.addItem("橢球高（e-GNSS 給的高）", "ellipsoidal")
        self.cmb_vertical.addItem("正高", "orthometric")
        f.addRow("geo_z 高程基準", self.cmb_vertical)
        self.ed_survey_frame = QLineEdit(gp.DEFAULT_SURVEY_FRAME)
        f.addRow("外業量測框架", self.ed_survey_frame)
        orow = QHBoxLayout()
        self.ed_out_dir = QLineEdit()
        b = QPushButton("…")
        b.setFixedWidth(32)
        b.clicked.connect(self._browse_out)
        orow.addWidget(self.ed_out_dir, 1)
        orow.addWidget(b)
        f.addRow("輸出資料夾", orow)
        self.chk_overwrite = QCheckBox("已有檔案時覆寫")
        f.addRow(self.chk_overwrite)
        self.btn_export = QPushButton("匯出（CSV／KML／GeoJSON／gcp_list 樣板／外業表）")
        self.btn_export.clicked.connect(lambda: self.export())
        f.addRow(self.btn_export)
        f.addRow(QLabel("外業量完之後："))
        srow = QHBoxLayout()
        self.ed_survey_csv = QLineEdit()
        self.ed_survey_csv.setPlaceholderText("填好實測值的 field_survey_template.csv")
        b2 = QPushButton("…")
        b2.setFixedWidth(32)
        b2.clicked.connect(lambda: self._browse_file(self.ed_survey_csv, "CSV (*.csv)"))
        srow.addWidget(self.ed_survey_csv, 1)
        srow.addWidget(b2)
        f.addRow("實測 CSV", srow)
        self.ed_frame_crs = QLineEdit()
        self.ed_frame_crs.setPlaceholderText("實測框架名不是 PROJ 座標系時必填，例如 +proj=longlat +ellps=GRS80 +no_defs")
        f.addRow("實測框架對應的座標系", self.ed_frame_crs)
        self.btn_from_survey = QPushButton("由實測 CSV 產生 gcp_list.txt（只差影像座標）")
        self.btn_from_survey.clicked.connect(lambda: self.gcp_list_from_survey())
        f.addRow(self.btn_from_survey)
        self.export_log = QPlainTextEdit()
        self.export_log.setReadOnly(True)
        f.addRow(self.export_log)
        return w

    def _browse_out(self):
        p = QFileDialog.getExistingDirectory(self, "選輸出資料夾", self.ed_out_dir.text())
        if p:
            self.ed_out_dir.setText(p)

    def _browse_file(self, edit: QLineEdit, filt: str):
        p, _ = QFileDialog.getOpenFileName(self, "選檔案", edit.text(), filt)
        if p:
            edit.setText(p)

    # ─────────────────────────────────────────────────────────────────
    # 來源
    # ─────────────────────────────────────────────────────────────────
    def areas(self) -> Tuple[Optional[list], str, Optional[str]]:
        """(ring 的串列, 來源說明, 錯誤)。給了 polygons 就只用它（不退回地圖角點）。"""
        if self.polygons_source is not None:
            try:
                src = self.polygons_source
                raw = src() if callable(src) else src
            except Exception as e:                       # noqa: BLE001 - 主視窗狀態異常要說清楚
                return None, AREAS_MAIN_SRC, f"讀主視窗作業區失敗：{type(e).__name__}: {e}"
            rings = [list(r) for r in (raw or [])]
            if not rings:
                return None, AREAS_MAIN_SRC, "主視窗沒有任何作業區：請先圈好作業區"
            return rings, f"{AREAS_MAIN_SRC}，{len(rings)} 塊", None
        corners = list(getattr(self.map_widget, "corners", None) or []) if self.map_widget is not None else []
        if len(corners) < 3:
            return None, AREAS_MAP_SRC, "沒有作業區（地圖上的角點不到 3 個）"
        return [corners], f"{AREAS_MAP_SRC}（1 塊）", None

    def blocks(self) -> Tuple[Optional[dict], dict, str, Optional[str]]:
        """(區塊, 說明, 來源說明, 錯誤)。沒有提供者 ⇒ (None, {}, '沒有…', None)。"""
        if not self.chk_use_blocks.isChecked():
            return None, {}, "使用者取消勾選：不分 UAV 區塊", None
        src = self.blocks_source
        if src is None:
            return None, {}, "沒有 HARVEST 規劃：不分 UAV 區塊（每區塊 GCP 與接縫檢核點對不會產生）", None
        try:
            got = src() if callable(src) else src
        except Exception as e:                           # noqa: BLE001 - 取不到就照實說，不默默改成沒有區塊
            return None, {}, "HARVEST 區塊", (f"取 HARVEST 區塊失敗：{type(e).__name__}: {e}"
                                              "（要不分區塊就取消勾選「使用 HARVEST 區塊」）")
        info, label = {}, ""
        if isinstance(got, tuple):
            blocks = got[0]
            info = got[1] if len(got) > 1 else {}
            label = got[2] if len(got) > 2 else ""
        else:
            blocks = got
        if not blocks:
            return None, {}, label or "沒有 HARVEST 規劃：不分 UAV 區塊（每區塊 GCP 與接縫檢核點對不會產生）", None
        label = label or f"HARVEST 規劃：{len(blocks)} 架（{'、'.join(sorted(blocks))}）"
        return blocks, dict(info or {}), label, None

    def _refresh_sources(self):
        rings, src, err = self.areas()
        self.lbl_areas.setText(f"✘ {err}" if err else f"{src}")
        self.lbl_areas.setStyleSheet("color:#E53935;" if err else "")
        if not self.chk_use_blocks.isChecked():
            self.lbl_blocks.setText("不分 UAV 區塊（使用者取消勾選）")
            self.lbl_blocks.setStyleSheet("")
            return
        b, _info, bsrc, berr = self.blocks()
        self.lbl_blocks.setText(f"✘ {berr}" if berr else bsrc)
        self.lbl_blocks.setStyleSheet("color:#E53935;" if berr else "")

    # ─────────────────────────────────────────────────────────────────
    # 參數
    # ─────────────────────────────────────────────────────────────────
    def _on_preset_changed(self):
        pr = self._gp.PRESETS[self.cmb_preset.currentData()]
        self.lbl_preset.setText(pr.description)

    def _on_asprs_toggled(self, on: bool):
        if on:
            self.spn_chk.setValue(max(30, self.spn_chk.value()))

    def params(self) -> Tuple[Optional[Any], List[str]]:
        errors: List[str] = []
        tri = {"方案預設": None, "是": True, "否": False}
        kw = dict(preset=self.cmb_preset.currentData(), footprint_w_m=float(self.spn_w.value()),
                  footprint_l_m=float(self.spn_l.value()), min_inset_m=float(self.spn_min_inset.value()),
                  n_checkpoints=int(self.spn_chk.value()), split_submodels=int(self.spn_split.value()),
                  seed=int(self.spn_seed.value()), per_block=tri[self.cmb_per_block.currentText()],
                  seam_gcp=tri[self.cmb_seam_gcp.currentText()])
        kw["edge_band_m"] = _opt_float(self.ed_edge_band.text(), "邊緣帶寬", errors)
        kw["gcp_min_spacing_m"] = _opt_float(self.ed_gcp_spacing.text(), "GCP 最小間距", errors)
        kw["chk_gcp_min_m"] = _opt_float(self.ed_chk_gcp.text(), "檢核點離 GCP", errors)
        kw["seam_pair_offset_m"] = _opt_float(self.ed_seam_offset.text(), "接縫檢核點離對方區塊", errors)
        if self.chk_asprs30.isChecked() and kw["n_checkpoints"] < 30:
            errors.append(f"勾了 ASPRS 第 2 版（30 個）但檢核點數是 {kw['n_checkpoints']}")
        if errors:
            return None, errors
        try:
            p = self._gp.LayoutParams(**kw)
            p.check()
        except self._gp.GcpPlanError as e:
            return None, [str(e)]
        return p, []

    # ─────────────────────────────────────────────────────────────────
    # 產生
    # ─────────────────────────────────────────────────────────────────
    def generate(self) -> bool:
        gp = self._gp
        self._refresh_sources()
        errors = []
        params, perr = self.params()
        errors += perr
        rings, asrc, aerr = self.areas()
        if aerr:
            errors.append(aerr)
        blocks, binfo, bsrc, berr = self.blocks()
        if berr:
            errors.append(berr)
        if errors:
            self.lbl_errors.setText("\n".join(f"✘ {e}" for e in errors))
            return False
        try:
            plan = gp.plan_layout(rings, blocks, params, blocks_source=bsrc)
            comp = gp.compare_presets(rings, blocks, params, blocks_source=bsrc)
        except gp.GcpPlanError as e:
            self.lbl_errors.setText(f"✘ {e}")
            return False
        self.lbl_errors.setText("")
        plan.effective["areas_source"] = asrc
        plan.effective["blocks_info"] = binfo
        self.plan, self.comparison, self.exported = plan, comp, None
        self._fill(plan, comp)
        self._fill_projection(plan)
        logger.info(f"[GCP 規劃] {plan.effective['preset']}：GCP {plan.counts['GCP']}、檢核點 {plan.counts['CHK']}、"
                    f"接縫 {plan.counts['seams']}；警告 {len(plan.warnings)}")
        if self.chk_show_map.isChecked():
            self.show_on_map()
        return True

    def _fill(self, plan, comp):
        gp = self._gp
        c, e = plan.counts, plan.effective
        lines = [f"{e['preset_label']}：GCP {c['GCP']} 個（{self._by(c['GCP_by_purpose'])}）、"
                 f"檢核點 {c['CHK']} 個（{self._by(c['CHK_by_purpose'])}），共 {c['total']} 點要外業量測",
                 f"作業區 {e['area_m2'] / 1e6:.3f} km²；UAV 區塊 {len(e['blocks'])} 個；跨機接縫 {c['seams']} 條、"
                 f"接縫檢核點對 {c['seam_pairs']} 對；內縮 {e['inset_m']:.1f} m、GCP 間距 ≥ {e['gcp_min_spacing_m']:.1f} m、"
                 f"檢核點離 GCP ≥ {e['chk_gcp_min_m']:.1f} m"]
        if c["CHK"] < 30:
            lines.append(f"檢核點 {c['CHK']} < 30：精度報告要附 ASPRS 第 2 版的小專案聲明")
        self.lbl_counts.setText("\n".join(lines))
        self.lbl_warnings.setText("\n".join(f"⚠ {w}" for w in plan.warnings))
        self.points_table.setRowCount(len(plan.points))
        for i, p in enumerate(plan.points):
            vals = (p.id, p.type, f"{p.lat:.7f}", f"{p.lon:.7f}", p.block or "—",
                    gp.PURPOSE_TEXT.get(p.purpose, p.purpose), p.note)
            for k, v in enumerate(vals):
                it = QTableWidgetItem(str(v))
                if k == 1:
                    it.setForeground(Qt.GlobalColor.red if p.type == "GCP" else Qt.GlobalColor.blue)
                self.points_table.setItem(i, k, it)
        self.compare_table.setRowCount(len(comp))
        for i, (name, s) in enumerate(comp.items()):
            vals = (s["label"], s["GCP"], s["CHK"], s["total"], s["seams"], s["seam_pairs"], s["n_warnings"],
                    s["flight"])
            for k, v in enumerate(vals):
                self.compare_table.setItem(i, k, QTableWidgetItem(str(v)))
        txt = ["【規則核對】"] + [f"{'✔' if ok else '✘'} {name}：{detail}" for name, ok, detail in plan.checks]
        txt += ["", "【備註】"] + [f"・{n}" for n in plan.notes]
        txt += ["", "【有效設定】", json.dumps(gp._jsonable(plan.effective), ensure_ascii=False, indent=1,
                                              sort_keys=True)]
        self.checks_view.setPlainText("\n".join(txt))

    @staticmethod
    def _by(d: dict) -> str:
        from survey.gcp_plan import PURPOSE_TEXT
        return "、".join(f"{PURPOSE_TEXT.get(k, k).split('（')[0]} {v}" for k, v in d.items()) or "—"

    def _fill_projection(self, plan):
        lat = sum(p.lat for p in plan.points) / max(1, len(plan.points))
        lon = sum(p.lon for p in plan.points) / max(1, len(plan.points))
        utm = self._gp.utm_header_for(lat, lon)
        cur = self.cmb_projection.currentText().strip()
        self.cmb_projection.blockSignals(True)
        self.cmb_projection.clear()
        self.cmb_projection.addItems([utm, "EPSG:4326"])
        self.cmb_projection.setCurrentText(cur if cur and cur not in (utm, "EPSG:4326") else utm)
        self.cmb_projection.blockSignals(False)

    # ─────────────────────────────────────────────────────────────────
    # 地圖
    # ─────────────────────────────────────────────────────────────────
    def show_on_map(self) -> Optional[dict]:
        if self.map_widget is None or self.plan is None:
            return None
        fn = getattr(self.map_widget, "set_survey_points", None)
        if fn is None:
            self._log("⚠ 這個地圖元件沒有 set_survey_points，點位不顯示")
            return None
        try:
            eff = fn(MAP_LAYER, self._gp.map_points(self.plan))
        except Exception as e:                           # noqa: BLE001 - 地圖失敗不能讓規劃結果消失
            self._log(f"⚠ 地圖顯示失敗：{type(e).__name__}: {e}")
            return None
        self._log(f"地圖：GCP／檢核點 {len(self.plan.points)} 點 → {eff}")
        return eff

    def clear_map(self) -> bool:
        fn = getattr(self.map_widget, "clear_survey_points", None) if self.map_widget is not None else None
        if fn is None:
            return False
        try:
            return bool(fn(MAP_LAYER))
        except Exception as e:                           # noqa: BLE001
            self._log(f"⚠ 清除地圖點位失敗：{type(e).__name__}: {e}")
            return False

    # ─────────────────────────────────────────────────────────────────
    # 匯出
    # ─────────────────────────────────────────────────────────────────
    def _log(self, msg: str):
        self.export_log.appendPlainText(msg)
        logger.info(f"[GCP 規劃] {msg}")

    def export(self, out_dir: Optional[str] = None) -> Optional[dict]:
        gp = self._gp
        if self.plan is None:
            self.lbl_errors.setText("✘ 還沒有產生佈設")
            return None
        out = (out_dir or self.ed_out_dir.text()).strip()
        if not out:
            self.lbl_errors.setText("✘ 輸出資料夾必填")
            return None
        try:
            paths = gp.export_layout(self.plan, out, projection=self.cmb_projection.currentText().strip(),
                                     vertical=self.cmb_vertical.currentData(),
                                     survey_frame=self.ed_survey_frame.text().strip() or gp.DEFAULT_SURVEY_FRAME,
                                     overwrite=self.chk_overwrite.isChecked())
        except gp.GcpPlanError as e:
            self.lbl_errors.setText(f"✘ {e}")
            return None
        self.lbl_errors.setText("")
        self.exported = paths
        self._log("匯出：" + "；".join(f"{k} → {v}" for k, v in paths.items()))
        return paths

    def gcp_list_from_survey(self, out_dir: Optional[str] = None) -> Optional[Path]:
        gp = self._gp
        src = self.ed_survey_csv.text().strip()
        out = (out_dir or self.ed_out_dir.text()).strip()
        errs = []
        if not src or not Path(src).is_file():
            errs.append(f"實測 CSV 不存在：{src or '（空白）'}")
        if not out:
            errs.append("輸出資料夾必填")
        if errs:
            self.lbl_errors.setText("\n".join(f"✘ {e}" for e in errs))
            return None
        dst = Path(out) / "gcp_list.txt"
        if dst.exists() and not self.chk_overwrite.isChecked():
            self.lbl_errors.setText(f"✘ {dst} 已存在：請換資料夾或勾「已有檔案時覆寫」")
            return None
        try:
            txt = gp.gcp_list_from_survey(src, projection=self.cmb_projection.currentText().strip(),
                                          vertical=self.cmb_vertical.currentData(),
                                          frame_crs=self.ed_frame_crs.text().strip() or None)
        except gp.GcpPlanError as e:
            self.lbl_errors.setText(f"✘ {e}")
            return None
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_text(txt, encoding="utf-8", newline="\n")
        self.lbl_errors.setText("")
        self._log(f"gcp_list.txt → {dst}（影像座標 <im_x> <im_y> <image_name> 要在 WebODM 的 GCP 編輯器或手動填，"
                  "每個 GCP ≥5 張）")
        return dst

    def showEvent(self, event):                          # noqa: N802 - Qt 介面
        self._refresh_sources()                          # 非模態：主視窗的作業區／規劃可能在關著時變了
        super().showEvent(event)
