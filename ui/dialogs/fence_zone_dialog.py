"""ui/dialogs/fence_zone_dialog.py — 統一 Fence Zone 設定對話框。

把原本分散的 NFZ Manager + 雷達威脅 spinbox 整合成單一 fence-style UX：
    1) 選類別：NFZ（禁航） / Threat（動態威脅） / Geofence（作業圍籬）
    2) 選形狀：多邊形（手動輸入 / 地圖點選） / 矩形（4 角輸入） / 圓形
    3) 設高度上下限
    4) 命名
    5) OK → 發 zone_accepted(FenceZone) 給 MainWindow 註冊 + 視覺 + FC 上傳

設計原則：
    - 對話框不接觸 registry / Cesium / SITLLink；只負責收集資料 → 發 signal
    - 地圖點選模式靠呼叫端（MainWindow）幫忙啟動，回填透過 set_vertices()
"""
from __future__ import annotations

from typing import Optional

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QPushButton,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from mission.fence_zone import FenceZone, ZoneCategory, ZoneShape
from ui.resources.aeroplan_theme import tokens as T


class FenceZoneDialog(QDialog):
    """新增 / 編輯一個 fence zone。"""

    # 主要輸出：使用者按 OK 後發
    zone_accepted = pyqtSignal(object)            # FenceZone

    # 從地圖繪製的入口已搬到主面板「Fence Zones」群組。地圖完成後 MainWindow
    # 會直接 new 對話框並呼叫 set_vertices(...) / set_circle(...) 預填。

    def __init__(self, parent: Optional[QWidget] = None,
                 default_category: ZoneCategory = ZoneCategory.NFZ) -> None:
        super().__init__(parent)
        self.setWindowTitle("新增 Fence 區域 — 統一 NFZ / 威脅 / 圍籬設定")
        self.setMinimumSize(540, 520)
        self.setStyleSheet(f"background: {T.BG_PRIMARY}; color: {T.FG};")

        self._default_category = default_category
        self._build_ui()

    # ── UI ───────────────────────────────────────────────────────────
    def _build_ui(self) -> None:
        v = QVBoxLayout(self)
        v.setContentsMargins(14, 12, 14, 12)
        v.setSpacing(10)

        # 標題
        title = QLabel("⊞  FENCE ZONE  ─  unified setup", self)
        title.setStyleSheet(
            f"font-family: {T.FONT_DISPLAY_STACK}; font-weight: 800; "
            f"font-size: 14px; color: {T.FG}; letter-spacing: 0.8px;"
        )
        v.addWidget(title)

        # ── 基本資料 ──
        form = QFormLayout()
        form.setHorizontalSpacing(10)
        form.setVerticalSpacing(8)

        self._name_edit = QLineEdit(self)
        self._name_edit.setPlaceholderText("ZONE-1")
        form.addRow("名稱:", self._name_edit)

        self._cat_combo = QComboBox(self)
        self._cat_combo.addItem("NFZ (禁航區・紅・拒絕進入)", ZoneCategory.NFZ)
        self._cat_combo.addItem("Threat (威脅區・橘・拒絕進入)", ZoneCategory.THREAT)
        self._cat_combo.addItem("Geofence (作業圍籬・琥珀・限制範圍內)",
                                ZoneCategory.GEOFENCE)
        # 預設選 default_category
        for i in range(self._cat_combo.count()):
            if self._cat_combo.itemData(i) == self._default_category:
                self._cat_combo.setCurrentIndex(i)
                break
        self._cat_combo.currentIndexChanged.connect(self._on_category_changed)
        form.addRow("分類:", self._cat_combo)

        self._shape_combo = QComboBox(self)
        self._shape_combo.addItem("多邊形 (Polygon)", ZoneShape.POLYGON)
        self._shape_combo.addItem("圓形 (Circle)", ZoneShape.CIRCLE)
        self._shape_combo.currentIndexChanged.connect(self._on_shape_changed)
        form.addRow("形狀:", self._shape_combo)

        # 海拔上下限
        alt_row = QHBoxLayout()
        self._alt_min_spin = QDoubleSpinBox(self)
        self._alt_min_spin.setRange(-100.0, 10000.0)
        self._alt_min_spin.setSuffix(" m")
        self._alt_min_spin.setValue(0.0)
        self._alt_min_spin.setDecimals(1)
        self._alt_max_spin = QDoubleSpinBox(self)
        self._alt_max_spin.setRange(0.0, 10000.0)
        self._alt_max_spin.setSuffix(" m")
        self._alt_max_spin.setValue(200.0)
        self._alt_max_spin.setDecimals(1)
        alt_row.addWidget(QLabel("min:"))
        alt_row.addWidget(self._alt_min_spin, 1)
        alt_row.addWidget(QLabel("  max:"))
        alt_row.addWidget(self._alt_max_spin, 1)
        form.addRow("海拔範圍:", alt_row)

        v.addLayout(form)

        # ── 幾何輸入區（多邊形 / 圓形 切換） ──
        v.addWidget(self._build_section_label("幾何設定"))

        self._geom_stack = QStackedWidget(self)
        self._geom_stack.addWidget(self._build_polygon_page())  # idx 0
        self._geom_stack.addWidget(self._build_circle_page())   # idx 1
        v.addWidget(self._geom_stack, 1)

        # ── 按鈕列 ──
        btn_row = QHBoxLayout()
        btn_row.addStretch(1)
        self._btn_cancel = QPushButton("取消", self)
        self._btn_cancel.setStyleSheet(self._btn_style(T.BORDER))
        self._btn_cancel.clicked.connect(self.reject)
        self._btn_ok = QPushButton("加入區域", self)
        self._btn_ok.setStyleSheet(self._btn_style(T.FRIENDLY))
        self._btn_ok.clicked.connect(self._on_accept)
        btn_row.addWidget(self._btn_cancel)
        btn_row.addWidget(self._btn_ok)
        v.addLayout(btn_row)

        # 初始狀態同步
        self._on_shape_changed()
        self._on_category_changed()

    def _build_polygon_page(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)

        # 頂點列表
        self._poly_list = QListWidget(w)
        self._poly_list.setMinimumHeight(140)
        self._poly_list.setStyleSheet(
            f"background: {T.BG_ELEVATED}; "
            f"font-family: {T.FONT_MONO_STACK}; font-size: 11px; "
            f"border: 1px solid {T.BORDER_SUBTLE};"
        )
        lay.addWidget(self._poly_list, 1)

        # 操作列：手動輸入 / 從地圖選 / 移除
        action_row = QHBoxLayout()
        self._poly_lat_spin = QDoubleSpinBox(w)
        self._poly_lat_spin.setRange(-90.0, 90.0)
        self._poly_lat_spin.setDecimals(6)
        self._poly_lat_spin.setSuffix(" °")
        self._poly_lon_spin = QDoubleSpinBox(w)
        self._poly_lon_spin.setRange(-180.0, 180.0)
        self._poly_lon_spin.setDecimals(6)
        self._poly_lon_spin.setSuffix(" °")
        btn_add = QPushButton("＋ 加入頂點", w)
        btn_add.setStyleSheet(self._btn_style(T.NEUTRAL))
        btn_add.clicked.connect(self._on_add_vertex)
        btn_rem = QPushButton("－ 移除選取", w)
        btn_rem.setStyleSheet(self._btn_style(T.BORDER))
        btn_rem.clicked.connect(self._on_remove_vertex)
        action_row.addWidget(QLabel("lat:"))
        action_row.addWidget(self._poly_lat_spin)
        action_row.addWidget(QLabel("lon:"))
        action_row.addWidget(self._poly_lon_spin)
        action_row.addWidget(btn_add)
        action_row.addWidget(btn_rem)
        lay.addLayout(action_row)

        # 提示：地圖繪製入口在主面板（避免使用者卡在這裡找）
        hint = QLabel(
            "提示：在 3D 地圖即時繪製請使用主面板 "
            "Fence Zones 群組的「⊕ 地圖繪製多邊形」按鈕",
            w,
        )
        hint.setWordWrap(True)
        hint.setStyleSheet(
            f"color: {T.FG_MUTED}; font-size: 10px; padding: 4px 0;"
        )
        lay.addWidget(hint)

        return w

    def _build_circle_page(self) -> QWidget:
        w = QWidget()
        lay = QFormLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setHorizontalSpacing(10)
        lay.setVerticalSpacing(8)

        self._circ_lat_spin = QDoubleSpinBox(w)
        self._circ_lat_spin.setRange(-90.0, 90.0)
        self._circ_lat_spin.setDecimals(6)
        self._circ_lat_spin.setSuffix(" °")
        self._circ_lon_spin = QDoubleSpinBox(w)
        self._circ_lon_spin.setRange(-180.0, 180.0)
        self._circ_lon_spin.setDecimals(6)
        self._circ_lon_spin.setSuffix(" °")
        self._circ_r_spin = QDoubleSpinBox(w)
        self._circ_r_spin.setRange(10.0, 50000.0)
        self._circ_r_spin.setDecimals(1)
        self._circ_r_spin.setSuffix(" m")
        self._circ_r_spin.setValue(500.0)
        self._circ_seg_spin = QDoubleSpinBox(w)
        self._circ_seg_spin.setRange(6, 64)
        self._circ_seg_spin.setDecimals(0)
        self._circ_seg_spin.setValue(16)
        self._circ_seg_spin.setSuffix(" 邊")
        self._circ_seg_spin.setToolTip(
            "圓形上傳至飛控前展開為 N 邊形（建議 16；越多越精確但 FC 記憶體吃緊）"
        )

        lay.addRow("圓心 lat:", self._circ_lat_spin)
        lay.addRow("圓心 lon:", self._circ_lon_spin)
        lay.addRow("半徑:", self._circ_r_spin)
        lay.addRow("多邊形展開:", self._circ_seg_spin)

        hint = QLabel(
            "提示：在 3D 地圖即時拖曳請使用主面板 "
            "Fence Zones 群組的「⊕ 地圖拖曳圓形」按鈕",
            w,
        )
        hint.setWordWrap(True)
        hint.setStyleSheet(
            f"color: {T.FG_MUTED}; font-size: 10px; padding: 4px 0;"
        )
        lay.addRow("", hint)

        return w

    def _build_section_label(self, text: str) -> QLabel:
        lab = QLabel(text, self)
        lab.setStyleSheet(
            f"font-family: {T.FONT_DISPLAY_STACK}; font-weight: 700; "
            f"font-size: 10px; color: {T.FG_SECONDARY}; "
            f"letter-spacing: 0.6px; padding-top: 4px;"
        )
        return lab

    @staticmethod
    def _btn_style(color: str) -> str:
        return (
            f"QPushButton {{ background: {T.BG_ELEVATED}; "
            f"color: {T.FG}; border: 1px solid {color}; "
            f"border-radius: 4px; padding: 5px 12px; "
            f"font-family: {T.FONT_DISPLAY_STACK}; font-weight: 700; "
            f"font-size: 10px; letter-spacing: 0.5px; }} "
            f"QPushButton:hover {{ background: {color}; color: {T.BG_PRIMARY}; }}"
        )

    # ── 槽函式 ───────────────────────────────────────────────────────
    def _on_shape_changed(self) -> None:
        idx = 0 if self._shape_combo.currentData() == ZoneShape.POLYGON else 1
        self._geom_stack.setCurrentIndex(idx)

    def _on_category_changed(self) -> None:
        cat = self._cat_combo.currentData()
        # 預設名稱依分類自動加 prefix
        if not self._name_edit.text().strip():
            self._name_edit.setPlaceholderText({
                ZoneCategory.NFZ: 'NFZ-1',
                ZoneCategory.THREAT: 'THREAT-1',
                ZoneCategory.GEOFENCE: 'GEOFENCE-1',
            }[cat])

    def _on_add_vertex(self) -> None:
        lat = self._poly_lat_spin.value()
        lon = self._poly_lon_spin.value()
        self._append_vertex_to_list(lat, lon)

    def _on_remove_vertex(self) -> None:
        row = self._poly_list.currentRow()
        if row >= 0:
            self._poly_list.takeItem(row)

    def _append_vertex_to_list(self, lat: float, lon: float) -> None:
        idx = self._poly_list.count() + 1
        item = QListWidgetItem(
            f"F{idx}   lat={lat:.6f}°    lon={lon:.6f}°"
        )
        item.setData(Qt.ItemDataRole.UserRole, (float(lat), float(lon)))
        self._poly_list.addItem(item)

    # ── 公開 API（呼叫端在地圖點選結束時回填用） ─────────────────
    def set_vertices(self, vertices: list[tuple[float, float]]) -> None:
        """多邊形地圖點選完成時呼叫，覆寫頂點列表並切到 POLYGON 形狀。"""
        # 切到 POLYGON
        for i in range(self._shape_combo.count()):
            if self._shape_combo.itemData(i) == ZoneShape.POLYGON:
                self._shape_combo.setCurrentIndex(i)
                break
        self._poly_list.clear()
        for lat, lon in vertices:
            self._append_vertex_to_list(lat, lon)

    def set_circle(self, lat: float, lon: float, radius_m: float) -> None:
        """圓形地圖拖曳完成時呼叫。"""
        for i in range(self._shape_combo.count()):
            if self._shape_combo.itemData(i) == ZoneShape.CIRCLE:
                self._shape_combo.setCurrentIndex(i)
                break
        self._circ_lat_spin.setValue(float(lat))
        self._circ_lon_spin.setValue(float(lon))
        self._circ_r_spin.setValue(float(radius_m))

    # ── OK 觸發 ─────────────────────────────────────────────────────
    def _on_accept(self) -> None:
        cat = self._cat_combo.currentData()
        shape = self._shape_combo.currentData()
        name = self._name_edit.text().strip() or self._name_edit.placeholderText()
        alt_min = self._alt_min_spin.value()
        alt_max = self._alt_max_spin.value()

        if alt_max <= alt_min:
            QMessageBox.warning(self, "海拔範圍錯誤", "alt max 必須大於 alt min")
            return

        if shape == ZoneShape.POLYGON:
            verts: list[tuple[float, float]] = []
            for i in range(self._poly_list.count()):
                data = self._poly_list.item(i).data(Qt.ItemDataRole.UserRole)
                if data:
                    verts.append(data)
            if len(verts) < 3:
                QMessageBox.warning(
                    self, "頂點不足",
                    "多邊形至少需要 3 個頂點，請手動加入或從地圖點選。"
                )
                return
            zone = FenceZone(
                name=name, category=cat, shape=shape,
                vertices=verts,
                alt_min_m=alt_min, alt_max_m=alt_max,
            )
        else:  # CIRCLE
            lat = self._circ_lat_spin.value()
            lon = self._circ_lon_spin.value()
            r = self._circ_r_spin.value()
            if r <= 0 or (lat == 0 and lon == 0):
                QMessageBox.warning(
                    self, "圓形未設定",
                    "請輸入有效的圓心經緯度與半徑（>0）。"
                )
                return
            zone = FenceZone(
                name=name, category=cat, shape=shape,
                center=(lat, lon), radius_m=r,
                alt_min_m=alt_min, alt_max_m=alt_max,
            )

        self.zone_accepted.emit(zone)
        self.accept()


__all__ = ['FenceZoneDialog']
