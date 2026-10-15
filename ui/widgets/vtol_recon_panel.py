"""VTOL 群飛偵察面板 —— 一頁走完「劃區 → 感測器足跡 → 規劃 → 驗算 → 輸出 → 起 SITL」。

與其他分頁的分工
----------------
* 路徑規劃**沿用現有 DCCPP**（不另建規劃器）：本面板組好參數後發
  :attr:`plan_requested`，由 MainWindow 轉呼叫既有的 DCCPP 流程，
  只是把 ``vehicle_type`` 釘死成 ``'vtol'``。
* 本面板自己負責的是 DCCPP **沒有**做的兩件事：
  1. 感測器足跡建模 —— 由相機規格與航高互算掃描寬度 / GSD，再把
     「有效條帶間距」餵給 DCCPP 的 ``fov_width_m``。
  2. 規劃後的覆蓋率驗算 —— 規劃器不會回報迴轉切角、避撞繞道造成的漏掃。

面板本身不做任何運算以外的副作用：所有實際動作都用信號往外送，
由 MainWindow 決定怎麼執行（與 ParameterPanel 的既有慣例一致）。
"""

from __future__ import annotations

from typing import Optional

from pathlib import Path

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QPixmap
import math

from PyQt6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QGroupBox, QSlider,
    QHBoxLayout, QHeaderView, QLabel, QLineEdit, QRadioButton, QSpinBox,
    QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from core.recon import CameraSpec, FootprintError, altitude_for_gsd_m, compute_footprint
from ui.resources.aeroplan_theme.widgets import IconButton
from utils.logger import get_logger

logger = get_logger()


# ══════════════════════════════════════════════════════════════════════
#  版面密度（只作用於本頁，不動全域 QSS）
# ══════════════════════════════════════════════════════════════════════
# 全域 Global_MIL_STD.qss 是**所有分頁共用**的，改它會動到別人的頁面；
# 這頁的擁擠是因為每個 QFormLayout 都吃 Qt 預設間距（幾乎沒有留白）。
# 所以間距一律在這裡用 layout API 設定 —— 影響範圍剛好是這一頁。
_ROOT_MARGIN = 14        # 整頁四周
_ROOT_SPACING = 14       # 群組與群組之間
_GROUP_MARGIN_LR = 14    # 群組內左右
_GROUP_MARGIN_TOP = 12   # 群組內上緣（QSS 已有 margin-top 14 + padding-top 10）
_GROUP_MARGIN_BOTTOM = 14
_FORM_V_SPACING = 11     # 欄位列與列之間
_FORM_H_SPACING = 16     # 標籤與輸入框之間
_FIELD_MIN_HEIGHT = 28   # 輸入框最小高度，避免上下擠在一起

# ── 兩個構型的機體照片 ────────────────────────────────────────────────
#   放在 ui/resources/images/（該資料夾整包會被 PyInstaller 收進去）。
#   檔案不存在時面板照常運作，只是那一格顯示檔名而不是照片。
_IMAGE_DIR = Path(__file__).resolve().parent.parent / "resources" / "images"
_HV_PLATFORM_IMAGES = {"A": "pioneer_pnp.png", "B": "hero_pnp.png"}
_PLATFORM_THUMB_W = 132      # 縮圖框寬（固定寬度讓兩列的文字對齊）
_PLATFORM_THUMB_H = 74       # 縮圖高（16:9 左右，看得出機體外形又不佔版面）

# ── 航測建圖：相機預設 ──────────────────────────────────────────────
#   焦距／像素間距／解析度是規格表值（評估報告 §5 已用這些數字獨立重算足跡）。
#   ⚠ 觸發率沒有預設的「證明值」：預設只填一個佔位（評估報告的台架目標 2 Hz），
#     面板顯示「未經台架驗證」直到使用者勾「已台架驗證」。讀出時間全部是推算值 ⇒ 不預填。
MAPPING_CAMERA_PRESETS = {
    "rpi_hq_6mm": {"label": "RPi HQ（IMX477）＋ 6 mm", "focal_mm": 6.0, "pixel_um": 1.55,
                   "width_px": 4056, "height_px": 3040},
    "rpi_hq_8mm": {"label": "RPi HQ（IMX477）＋ 8 mm", "focal_mm": 8.0, "pixel_um": 1.55,
                   "width_px": 4056, "height_px": 3040},
    "rpi_cm3": {"label": "RPi Camera Module 3（IMX708）4.74 mm", "focal_mm": 4.74, "pixel_um": 1.40,
                "width_px": 4608, "height_px": 2592},
    "rpi_gs_8mm": {"label": "RPi Global Shutter（IMX296）＋ 8 mm", "focal_mm": 8.0, "pixel_um": 3.45,
                   "width_px": 1456, "height_px": 1088},
}
MAPPING_PRESET_CUSTOM = "custom"
MAPPING_TRIGGER_HZ_PLACEHOLDER = 2.0     # 佔位，不是證明值
MAPPING_FORWARD_OVERLAP_DEFAULT = 0.80
MAPPING_LEAD_IN_DEFAULT_M = 50.0
MAPPING_MAX_ROLL_DEFAULT_DEG = 15.0
MAPPING_RETRIGGER_TOL_DEFAULT = 0.10
#: 交叉航帶（opt-in，預設不勾）：面積比例預設 0.20（Gerke & Przybilla 2016 的交叉塊約蓋 20% 面積）、高度差 0 m
MAPPING_CROSS_FRACTION_DEFAULT = 0.20
MAPPING_CROSS_ALT_OFFSET_DEFAULT_M = 0.0
#: 建圖作業風速上限（2026-10-15 使用者定調：首飛 ≤ 3 m/s）
MAPPING_MAX_WIND_DEFAULT_MPS = 3.0
#: 第一個 = 預設。relay 在前（R1 #4／需求 §4：實機要 relay；servo 在不觸發時仍持續輸出 PWM 脈衝串，
#: 樹莓派的上升緣擷取會把每個 PWM 週期當成一張照片）。relay 要填韌體版本與腳位，沒填 ⇒ 面板直接顯示「無法規劃」與原因。
MAPPING_BACKENDS = (("relay", "relay（GPIO）"),
                    ("servo", "servo（PWM，SITL）"),
                    ("mavlink_v2", "mavlink_v2"))
MAPPING_BACKEND_DEFAULT = MAPPING_BACKENDS[0][0]


def _platform_pixmap(filename: str, height: int):
    """載入機體照片並縮到指定高度；檔案不在或讀不出來回 None。"""
    path = _IMAGE_DIR / filename
    if not path.exists():
        return None
    pm = QPixmap(str(path))
    if pm.isNull():
        logger.warning(f"[VTOL 偵察] 機體照片讀不出來：{path}")
        return None
    return pm.scaledToHeight(height, Qt.TransformationMode.SmoothTransformation)



def _tune_form(form: QFormLayout, is_group_layout: bool = False) -> QFormLayout:
    """把一個 QFormLayout 調成本頁統一的密度。

    ⚠ 用 helper 而不是逐處填數字：本頁有 5 個 QFormLayout，散落填數字必然
    會漏掉一兩個，而「漏掉」在畫面上只是某一段比較擠，沒有任何錯誤訊息。

    Args:
        is_group_layout: 這個 form 是否**直接**當 QGroupBox 的 layout。
            ⚠ 是的話留白要自己給 —— 設成 0 會讓欄位直接貼在群組框線上。
            若它只是被塞進某個 QVBoxLayout(g)，留白由 `_tune_group_layout`
            負責，這裡就要歸零，否則兩層留白疊起來變成兩倍。
    """
    form.setVerticalSpacing(_FORM_V_SPACING)
    form.setHorizontalSpacing(_FORM_H_SPACING)
    if is_group_layout:
        form.setContentsMargins(_GROUP_MARGIN_LR, _GROUP_MARGIN_TOP,
                                _GROUP_MARGIN_LR, _GROUP_MARGIN_BOTTOM)
    else:
        form.setContentsMargins(0, 0, 0, 0)
    # ⚠ 必須是 AllNonFixedFieldsGrow(Windows 上的 Qt 預設)。
    #   用 ExpandingFieldsGrow 只有 sizePolicy 是 Expanding 的欄位會長,
    #   而 QDoubleSpinBox/QSpinBox 不是 ⇒ 輸入框會**縮成一小截**,
    #   看起來像「欄位變窄了」而不是「間距變鬆了」(我第一版就這樣弄壞)。
    form.setFieldGrowthPolicy(
        QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
    form.setLabelAlignment(Qt.AlignmentFlag.AlignRight
                           | Qt.AlignmentFlag.AlignVCenter)
    return form


def _tune_group_layout(lay) -> None:
    """群組內層 layout 的留白（標題與第一列之間最容易黏在一起）。"""
    lay.setContentsMargins(_GROUP_MARGIN_LR, _GROUP_MARGIN_TOP,
                           _GROUP_MARGIN_LR, _GROUP_MARGIN_BOTTOM)
    if lay.spacing() < 8:
        lay.setSpacing(10)


class VTOLReconPanel(QWidget):
    """VTOL 群飛偵察工作流面板。"""

    # ── 對外信號 ─────────────────────────────────────────────
    pick_area_requested = pyqtSignal()        # 從地圖框選偵察區
    clear_area_requested = pyqtSignal()       # 清除偵察區
    delete_last_corner_requested = pyqtSignal()  # 刪除上一個角點（誤點救援）
    commit_area_requested = pyqtSignal()      # 完成本區、開始畫下一區（多區模式）
    clear_areas_requested = pyqtSignal()      # 清除所有已完成的區
    pick_launch_site_requested = pyqtSignal()   # 從地圖點選起降場（= MainWindow.home_point）
    clear_launch_site_requested = pyqtSignal()  # 清除起降場
    plan_requested = pyqtSignal(dict)         # 執行 DCCPP（vehicle_type 已釘為 vtol）
    find_flat_requested = pyqtSignal(dict)    # 找平坦位置（參數；作業區由 MainWindow 補）
    flat_apply_requested = pyqtSignal(float, float, float)   # 平移作業區到（緯度, 經度），縮放倍率
    flat_undo_requested = pyqtSignal()        # 復原上次平移
    verify_coverage_requested = pyqtSignal(dict)   # 覆蓋率驗算
    show_missed_requested = pyqtSignal()      # 在地圖標出漏掃格（紅點）
    show_heatmap_requested = pyqtSignal()     # 用 SAR 熱力圖呈現覆蓋分布
    export_requested = pyqtSignal(dict)       # 匯出 VTOL 航點
    upload_requested = pyqtSignal()           # 上傳至飛控
    launch_sitl_requested = pyqtSignal(int)   # 一鍵起 N 台 VTOL SITL

    # FOV 光錐（本頁自己的參數來源；與戰術分析頁共用同一個 3D 光錐實體）
    fov_cone_toggle_requested = pyqtSignal(bool)          # 開/關
    #   payload: (mount_angle_deg, hfov_deg, vfov_deg) —— 與戰術頁同一組語意
    fov_visual_changed = pyqtSignal(float, float, float)
    # ── Fence Zones（NFZ / 威脅 / 圍籬）—— 與 parameter_panel 同名，接同一組
    #    main_window handler（open_fence_zone_dialog / on_fence_zone_draw_* /
    #    upload_fence_zones_to_fc / clear_all_fence_zones）。
    fence_zone_create_requested = pyqtSignal()        # 開統一對話框（手動輸入）
    fence_zone_draw_polygon_requested = pyqtSignal()  # 地圖繪多邊形
    fence_zone_draw_circle_requested = pyqtSignal()   # 地圖拖圓形
    fence_zone_upload_requested = pyqtSignal()        # 上傳至飛控
    fence_zone_clear_all_requested = pyqtSignal()     # 全部清除

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._corner_count = 0
        self._committed_areas = 0     # 已完成的作業區數（多區模式）
        self._has_plan = False
        self._has_missed = False
        self._has_coverage = False
        self._build_ui()
        self._refresh_footprint()
        self._sync_enabled()

    # ══════════════════════════════════════════════════════════
    #  UI
    # ══════════════════════════════════════════════════════════
    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(_ROOT_MARGIN, _ROOT_MARGIN,
                                _ROOT_MARGIN, _ROOT_MARGIN)
        root.setSpacing(_ROOT_SPACING)
        root.addWidget(self._build_area_group())
        root.addWidget(self._build_sensor_group())
        root.addWidget(self._build_vtol_group())
        root.addWidget(self._build_swarm_group())
        root.addWidget(self._build_plan_group())
        root.addWidget(self._build_coverage_group())
        root.addWidget(self._build_output_group())
        root.addWidget(self._build_fence_group())
        root.addStretch(1)

    # ── 1. 偵察區域 ───────────────────────────────────────────
    def _build_area_group(self) -> QGroupBox:
        g = QGroupBox("1. 偵察區域")
        lay = QVBoxLayout(g)
        _tune_group_layout(lay)

        self.area_label = QLabel("尚未劃定偵察區")
        self.area_label.setWordWrap(True)
        lay.addWidget(self.area_label)

        row = QHBoxLayout()
        self.btn_pick_area = IconButton("polygon", "從地圖框選", tone="primary", compact=True)
        self.btn_pick_area.clicked.connect(self.pick_area_requested.emit)
        row.addWidget(self.btn_pick_area)
        # 誤點救援：只退掉最後一個角點，不用整區重畫（2D + 3D 同步刪）
        self.btn_del_last = IconButton("undo", "刪除上一點", tone="warn", compact=True)
        self.btn_del_last.setToolTip(
            "刪除最後一個邊界角點（2D 與 3D 同時生效）——\n"
            "點錯位置時用這個，不必按「清除」重畫整個偵察區。")
        self.btn_del_last.clicked.connect(self.delete_last_corner_requested.emit)
        row.addWidget(self.btn_del_last)
        self.btn_clear_area = IconButton("clear", "清除", tone="ghost", compact=True)
        self.btn_clear_area.clicked.connect(self.clear_area_requested.emit)
        row.addWidget(self.btn_clear_area)
        row.addStretch(1)
        lay.addLayout(row)

        # ── 多區模式 ──────────────────────────────────────────
        #   圈好一塊按「完成本區」，畫布清空繼續圈下一塊。規劃時所有區一起送。
        #   ⚠ 只圈一塊時完全不用碰這兩顆鈕，行為與以前相同。
        row2 = QHBoxLayout()
        self.btn_commit_area = IconButton("plus", "完成本區，畫下一區",
                                          tone="primary", compact=True)
        self.btn_commit_area.setToolTip(
            "把目前圈好的這一塊收進清單，畫布清空讓你圈下一塊。\n"
            "規劃時所有已完成的區會一起送給規劃器，由它決定哪台飛哪一塊。\n"
            "只要圈一塊的話不用按這顆。")
        self.btn_commit_area.clicked.connect(self.commit_area_requested.emit)
        row2.addWidget(self.btn_commit_area)
        self.btn_clear_areas = IconButton("clear", "清除所有區", tone="ghost", compact=True)
        self.btn_clear_areas.setToolTip("清掉所有**已完成**的區；正在畫的那一塊不受影響。")
        self.btn_clear_areas.clicked.connect(self.clear_areas_requested.emit)
        row2.addWidget(self.btn_clear_areas)
        row2.addStretch(1)
        lay.addLayout(row2)

        self.areas_label = QLabel("")
        self.areas_label.setWordWrap(True)
        self.areas_label.setStyleSheet("color:#555; font-size:11px;")
        lay.addWidget(self.areas_label)
        # ⚠ 這裡**不能**呼叫 self.set_area_count()：它會走到 _sync_enabled()，
        #   而規劃鈕等元件此時還沒建好。直接寫初始文字就好。
        self.areas_label.setText(
            "多區模式：圈好一塊按「完成本區」就能再圈下一塊；只圈一塊時不必用。")

        # ── 起降場（使用者決定，2026-10-08）────────────────────
        #   各機的起降坪排在這一點（間距見規劃參數），降落也回這裡。
        #   ⚠ 規劃前**必須**選：沒選時以前會默默放在作業區中心（起降坪在作業區正中、
        #   進場與返場全擠在中央），現在由 MainWindow._on_vtol_recon_plan 擋下。
        #   狀態本身存在 MainWindow.home_point（與固定翼的「起飛點」同一份），這裡只負責顯示。
        self.launch_site_label = QLabel()
        self.launch_site_label.setWordWrap(True)
        lay.addWidget(self.launch_site_label)
        row3 = QHBoxLayout()
        self.btn_pick_launch_site = IconButton("pin", "從地圖選起降場", tone="primary", compact=True)
        self.btn_pick_launch_site.setToolTip(
            "按下後在地圖（2D 或 3D）點一下起降場位置。\n"
            "各機的起降坪會排在這一點，任務結束也降回這裡。")
        self.btn_pick_launch_site.clicked.connect(self.pick_launch_site_requested.emit)
        row3.addWidget(self.btn_pick_launch_site)
        self.btn_clear_launch_site = IconButton("clear", "清除起降場", tone="ghost", compact=True)
        self.btn_clear_launch_site.clicked.connect(self.clear_launch_site_requested.emit)
        row3.addWidget(self.btn_clear_launch_site)
        row3.addStretch(1)
        lay.addLayout(row3)
        self.clear_launch_site()
        return g

    # ── 起降場顯示（狀態在 MainWindow.home_point）────────────────
    def set_launch_site(self, lat: float, lon: float) -> None:
        self.launch_site_label.setText(f"起降場：{lat:.6f}, {lon:.6f}")
        self.launch_site_label.setStyleSheet("color:#2e7d32; font-weight:600;")

    def clear_launch_site(self) -> None:
        self.launch_site_label.setText("起降場：尚未選擇 —— 規劃前必須先選（各機的起降坪會排在這裡）")
        self.launch_site_label.setStyleSheet("color:#c62828;")

    # ── 2. 感測器足跡 ─────────────────────────────────────────
    def _build_sensor_group(self) -> QGroupBox:
        g = QGroupBox("2. 感測器足跡")
        lay = QVBoxLayout(g)
        _tune_group_layout(lay)
        form = _tune_form(QFormLayout())

        self.hfov_spin = QDoubleSpinBox()
        self.hfov_spin.setRange(1.0, 179.0)
        self.hfov_spin.setValue(60.0)
        self.hfov_spin.setSuffix(" °")
        self.hfov_spin.setToolTip("橫向（cross-track）視場角 — 決定掃描寬度")
        form.addRow("相機 HFOV:", self.hfov_spin)

        self.vfov_spin = QDoubleSpinBox()
        self.vfov_spin.setRange(1.0, 179.0)
        self.vfov_spin.setValue(45.0)
        self.vfov_spin.setSuffix(" °")
        self.vfov_spin.setToolTip("縱向（along-track）視場角 — 決定足跡長度")
        form.addRow("相機 VFOV:", self.vfov_spin)

        self.img_width_spin = QSpinBox()
        self.img_width_spin.setRange(320, 20000)
        self.img_width_spin.setValue(4000)
        self.img_width_spin.setSuffix(" px")
        self.img_width_spin.setToolTip("影像橫向像素數 — 決定 GSD")
        form.addRow("影像寬度:", self.img_width_spin)

        self.mount_spin = QDoubleSpinBox()
        self.mount_spin.setRange(-84.0, 84.0)
        self.mount_spin.setValue(0.0)
        self.mount_spin.setSuffix(" °")
        self.mount_spin.setToolTip("掛載傾斜角，0 = 正下視；傾斜會讓掃描變寬但邊緣 GSD 變差")
        form.addRow("掛載角:", self.mount_spin)

        self.overlap_spin = QDoubleSpinBox()
        self.overlap_spin.setRange(0.0, 0.9)
        self.overlap_spin.setDecimals(2)
        self.overlap_spin.setSingleStep(0.05)
        self.overlap_spin.setValue(0.20)
        self.overlap_spin.setToolTip("相鄰條帶重疊率 — 有效條帶間距 = 掃描寬度 ×(1−重疊率)")
        form.addRow("重疊率:", self.overlap_spin)

        # ── 掃描角度（條帶方向）──────────────────────────────
        # 慣例與 PolygonUtils.optimal_scan_angle 一致：0°=北，順時針為正，
        # 掃描線平行於此方向。
        self.auto_angle_check = QCheckBox("自動選最佳掃描角（垂直於最小寬度）")
        self.auto_angle_check.setChecked(True)
        self.auto_angle_check.setToolTip(
            "勾選：用旋轉卡尺法選能讓條帶數最少的方向。\n"
            "取消勾選：改用下面手動指定的角度。")
        self.auto_angle_check.toggled.connect(self._on_auto_angle_toggled)
        form.addRow("", self.auto_angle_check)

        angle_row = QHBoxLayout()
        self.scan_angle_slider = QSlider(Qt.Orientation.Horizontal)
        self.scan_angle_slider.setRange(-180, 180)
        self.scan_angle_slider.setValue(0)
        self.scan_angle_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.scan_angle_slider.setTickInterval(30)
        self.scan_angle_spin = QDoubleSpinBox()
        self.scan_angle_spin.setRange(-180.0, 180.0)
        self.scan_angle_spin.setDecimals(1)
        self.scan_angle_spin.setSingleStep(5.0)
        self.scan_angle_spin.setValue(0.0)
        self.scan_angle_spin.setSuffix(" °")
        self.scan_angle_spin.setToolTip("0°=北，順時針為正；掃描線平行於此方向")
        # 滑桿與數字框互相同步（各自都能改）
        self.scan_angle_slider.valueChanged.connect(
            lambda v: self.scan_angle_spin.setValue(float(v))
            if abs(self.scan_angle_spin.value() - v) > 0.05 else None)
        self.scan_angle_spin.valueChanged.connect(
            lambda v: self.scan_angle_slider.setValue(int(round(v)))
            if self.scan_angle_slider.value() != int(round(v)) else None)
        angle_row.addWidget(self.scan_angle_slider, 1)
        angle_row.addWidget(self.scan_angle_spin)
        form.addRow("掃描角度:", angle_row)
        # 規劃器回報的「實際生效角」。自動模式下使用者看不到幾何算出什麼，
        # 沒有這個標籤就無從判斷手動值該從哪裡起跳。
        self.effective_angle_label = QLabel("尚未規劃")
        self.effective_angle_label.setStyleSheet("color:#888;")
        form.addRow("實際生效角:", self.effective_angle_label)
        self._auto_scan_angle_hint = None
        self._on_auto_angle_toggled(True)
        lay.addLayout(form)

        # 驅動方向：由航高算 GSD，或由目標 GSD 反算航高
        mode_row = QHBoxLayout()
        self.rb_by_alt = QRadioButton("由航高決定 GSD")
        self.rb_by_alt.setChecked(True)
        self.rb_by_gsd = QRadioButton("由目標 GSD 決定航高")
        mode_row.addWidget(self.rb_by_alt)
        mode_row.addWidget(self.rb_by_gsd)
        mode_row.addStretch(1)
        lay.addLayout(mode_row)

        form2 = _tune_form(QFormLayout())
        self.altitude_spin = QDoubleSpinBox()
        self.altitude_spin.setRange(10.0, 5000.0)
        self.altitude_spin.setValue(120.0)
        self.altitude_spin.setSuffix(" m")
        self.altitude_spin.setToolTip("偵察航高（AGL）")
        form2.addRow("偵察航高:", self.altitude_spin)

        self.target_gsd_spin = QDoubleSpinBox()
        self.target_gsd_spin.setRange(0.001, 5.0)
        self.target_gsd_spin.setDecimals(3)
        self.target_gsd_spin.setSingleStep(0.01)
        self.target_gsd_spin.setValue(0.05)
        self.target_gsd_spin.setSuffix(" m/px")
        self.target_gsd_spin.setToolTip("要求的地面解析度；勾選右側模式後由它反算航高")
        form2.addRow("目標 GSD:", self.target_gsd_spin)
        lay.addLayout(form2)

        self.footprint_label = QLabel("—")
        self.footprint_label.setWordWrap(True)
        self.footprint_label.setTextFormat(Qt.TextFormat.RichText)
        lay.addWidget(self.footprint_label)

        # ── 3D FOV 光錐（吃本頁參數，不是戰術分析頁的）──────────────
        self.fov_cone_check = QCheckBox("在 3D 地圖顯示 FOV 光錐（用本頁參數）")
        self.fov_cone_check.setToolTip(
            "把上面的 HFOV / VFOV / 掛載角投影成 3D 梯形角錐。"
            "需要 3D Cesium 視圖 + 連線中的 UAV 才看得到。")
        self.fov_cone_check.toggled.connect(self.fov_cone_toggle_requested.emit)
        lay.addWidget(self.fov_cone_check)

        self.fov_cone_status = QLabel("")
        self.fov_cone_status.setWordWrap(True)
        lay.addWidget(self.fov_cone_status)

        for w in (self.hfov_spin, self.vfov_spin, self.mount_spin,
                  self.altitude_spin, self.target_gsd_spin, self.overlap_spin):
            w.valueChanged.connect(self._refresh_footprint)
        # 三個角度一改就重投影光錐（不必等下一筆遙測）
        for w in (self.hfov_spin, self.vfov_spin, self.mount_spin):
            w.valueChanged.connect(self._emit_fov_visual)
        self.img_width_spin.valueChanged.connect(self._refresh_footprint)
        self.rb_by_alt.toggled.connect(self._refresh_footprint)
        return g

    # ── 3. VTOL 參數 ──────────────────────────────────────────
    def _on_auto_angle_toggled(self, auto: bool) -> None:
        """自動選角時把手動控制鎖起來，避免以為改了其實沒生效。

        取消自動時把數字框**種**成上一次實際生效的角度：使用者要的是
        「從現在畫面上這條航線開始微調」，硬跳回 0° 等於每次都要重來。
        """
        for w in (getattr(self, "scan_angle_slider", None),
                  getattr(self, "scan_angle_spin", None)):
            if w is not None:
                w.setEnabled(not auto)
        hint = getattr(self, "_auto_scan_angle_hint", None)
        if not auto and hint is not None and self.scan_angle_spin is not None:
            self.scan_angle_spin.setValue(self._wrap_scan_angle(hint))

    @staticmethod
    def _wrap_scan_angle(deg: float) -> float:
        """compass 0..360 → 數字框的 [-180, 180]（同一個方向，換個寫法）。"""
        return ((float(deg) + 180.0) % 360.0) - 180.0

    def set_effective_scan_angle(self, deg, auto: bool = True) -> None:
        """由規劃器回填「這趟實際用了幾度」。

        ⚠ 只有在規劃器真的回報時才呼叫；拿不到就維持原樣，不要用面板自己
        重算一個數字充當——那會變成「顯示的角度」與「飛的角度」兩套值。
        """
        if deg is None:
            return
        wrapped = self._wrap_scan_angle(deg)
        if auto:
            self._auto_scan_angle_hint = float(deg)
        src = "自動" if auto else "手動"
        self.effective_angle_label.setText(f"{wrapped:.1f}°（{src}，0°=北順時針）")
        self.effective_angle_label.setStyleSheet("color:#2e7d32;")

    def _build_vtol_group(self) -> QGroupBox:
        g = QGroupBox("3. VTOL 參數")
        form = _tune_form(QFormLayout(g), is_group_layout=True)

        self.transition_alt_spin = QDoubleSpinBox()
        self.transition_alt_spin.setRange(10.0, 500.0)
        self.transition_alt_spin.setValue(40.0)
        self.transition_alt_spin.setSuffix(" m")
        self.transition_alt_spin.setToolTip("垂直爬升到此高度後轉為固定翼前飛")
        form.addRow("轉換高度:", self.transition_alt_spin)

        self.speed_spin = QDoubleSpinBox()
        # ⚠ 下限 20：飛控 AIRSPEED_MIN=20（vtol_default.parm），低於它的
        #   DO_CHANGE_SPEED 會被夾到 20 —— 面板顯示的值與實飛值悄悄不一致。
        #   預設 22 落在 [AIRSPEED_MIN 20, AIRSPEED_MAX 34] 的舒適帶。
        self.speed_spin.setRange(20.0, 34.0)
        self.speed_spin.setValue(22.0)
        self.speed_spin.setSuffix(" m/s")
        self.speed_spin.setToolTip(
            "固定翼段巡航空速（飛控允許 20–34 m/s，低於 20 會被夾掉）")
        form.addRow("巡航速度:", self.speed_spin)

        self.turn_radius_spin = QDoubleSpinBox()
        self.turn_radius_spin.setRange(10.0, 500.0)
        self.turn_radius_spin.setValue(60.0)
        self.turn_radius_spin.setSuffix(" m")
        self.turn_radius_spin.setToolTip("固定翼段最小迴轉半徑 — 影響條帶端點的切角漏掃")
        form.addRow("迴轉半徑:", self.turn_radius_spin)

        self.auto_landing_check = QCheckBox("自動加入垂直起降段")
        self.auto_landing_check.setChecked(True)
        form.addRow("", self.auto_landing_check)
        return g

    # ── ④ 群飛 ───────────────────────────────────────────────
    def _build_swarm_group(self) -> QGroupBox:
        g = QGroupBox("4. 群飛編組")
        form = _tune_form(QFormLayout(g), is_group_layout=True)

        self.num_drones_spin = QSpinBox()
        self.num_drones_spin.setRange(2, 12)
        self.num_drones_spin.setValue(3)
        self.num_drones_spin.setSuffix(" 台")
        form.addRow("VTOL 數量:", self.num_drones_spin)

        self.coord_combo = QComboBox()
        self.coord_combo.addItem("不協同 (各自獨立)", "uncoordinated")
        self.coord_combo.addItem("協同 (共享進度)", "coordinated")
        form.addRow("協同模式:", self.coord_combo)

        self.collision_check = QCheckBox("啟用避撞")
        self.collision_check.setChecked(True)
        form.addRow("", self.collision_check)

        self.min_sep_spin = QDoubleSpinBox()
        self.min_sep_spin.setRange(5.0, 500.0)
        self.min_sep_spin.setValue(50.0)
        self.min_sep_spin.setSuffix(" m")
        self.min_sep_spin.valueChanged.connect(self._on_avoid_params_changed)
        form.addRow("最小間距:", self.min_sep_spin)

        # ⚠⚠ 高度錯開量。這一格以前**根本沒送出去** ⇒ 規劃器吃寫死預設 30,
        #   而最小間距預設 50 ⇒ 30 < 50 無條件成立 ⇒ 高度錯開永遠拉不到門檻,
        #   每輪 +30 一路棘輪(實測 120→360 m)。預設改成跟最小間距一致。
        self.alt_offset_spin = QDoubleSpinBox()
        self.alt_offset_spin.setRange(5.0, 200.0)
        self.alt_offset_spin.setValue(50.0)
        self.alt_offset_spin.setSuffix(" m")
        self.alt_offset_spin.setToolTip(
            "衝突時把其中一架抬高多少。"
            "小於「最小間距」時,完全重疊的航跡永遠解不到門檻。")
        self.alt_offset_spin.valueChanged.connect(self._on_avoid_params_changed)
        form.addRow("高度錯開量:", self.alt_offset_spin)

        # 累加上限:抬到這個量還解不掉就停止抬升並大聲失敗,不再無界棘輪
        self.max_alt_offset_spin = QDoubleSpinBox()
        self.max_alt_offset_spin.setRange(0.0, 500.0)
        self.max_alt_offset_spin.setValue(100.0)
        self.max_alt_offset_spin.setSuffix(" m")
        # 0 = 交給演算法自己算(2 x (機數-1) x 錯開量),不設絕對天花板。
        # ⚠ 自動值會**跟著錯開量放大**,所以它保證不擋住正常收斂,
        #   但不保證高度低 —— 要壓住絕對高度就得自己填一個數字。
        self.max_alt_offset_spin.setSpecialValueText("自動（依機數）")
        self.max_alt_offset_spin.setToolTip(
            "每架累加抬升的上限。撞到上限就停止抬升並回報失敗,"
            "而不是繼續把飛機往上推(掃描寬度與 GSD 都是照規劃航高算的)。")
        form.addRow("抬升上限:", self.max_alt_offset_spin)

        self.avoid_warn_label = QLabel("")
        self.avoid_warn_label.setWordWrap(True)
        form.addRow("", self.avoid_warn_label)
        self._on_avoid_params_changed()
        return g

    def _on_avoid_params_changed(self, *_a) -> None:
        """高度錯開量 < 最小間距時當場示警。

        ⚠ 這裡**不**偷偷把值夾上去 —— 面板顯示的值必須就是送出去的值,
        否則又變成「顯示的角度」與「飛的角度」兩套值。只警告,不代改。
        """
        off = self.alt_offset_spin.value()
        sep = self.min_sep_spin.value()
        if off < sep:
            self.avoid_warn_label.setText(
                f"高度錯開量 {off:.0f} m < 最小間距 {sep:.0f} m:"
                f"完全重疊的航跡解不到門檻,會一路抬到上限才失敗。")
            self.avoid_warn_label.setStyleSheet("color:#e65100;")
        else:
            self.avoid_warn_label.setText("")
            self.avoid_warn_label.setStyleSheet("")

    # ── 5. 執行規劃 + 6. 結果 ──────────────────────────────────
    def _build_plan_group(self) -> QGroupBox:
        g = QGroupBox("5. 規劃")
        lay = QVBoxLayout(g)
        _tune_group_layout(lay)

        # ── 演算法選擇 ────────────────────────────────────────
        #   這一頁有兩套規劃器可以選，兩者吃的是**同一組**上面填的參數：
        #   DCCPP      = 專案既有實作（論文 Alg.1–3），全隊共用一個轉彎半徑與高度。
        #   HARVEST   = 本專案自行開發的異構雙構型版本（hv_dccpp 套件）。
        algo_form = _tune_form(QFormLayout())
        self.algo_combo = QComboBox()
        self.algo_combo.addItem("DCCPP（既有實作）", "dccpp")
        self.algo_combo.addItem("HARVEST 異構雙構型（本專案）", "hv_dccpp")
        self.algo_combo.setToolTip(
            "DCCPP：論文的忠實實作。全隊共用一個轉彎半徑與一個高度，\n"
            "        刈幅用上面算出來的條帶間距，不做能量／風／地形包絡。\n"
            "HARVEST：兩種構型各自的轉彎半徑、巡航空速、爬降角、續航與最低作業高度；\n"
            "        刈幅由感測器與作業高度推導；含能量閉合檢查、風修正、\n"
            "        坡度受限高度包絡、垂直起降走廊與 4-D 去衝突。\n"
            "        只支援 4+1 複合翼 VTOL 兩個構型（Pioneer / HERO）。"
        )
        self.algo_combo.currentIndexChanged.connect(self._on_algo_changed)
        algo_form.addRow("規劃演算法:", self.algo_combo)
        lay.addLayout(algo_form)

        self.hv_group = self._build_hv_group()
        self.hv_group.setVisible(False)
        lay.addWidget(self.hv_group)

        self.btn_plan = IconButton("play", "執行 DCCPP 偵察規劃", tone="orange")
        self.btn_plan.clicked.connect(self._emit_plan)
        lay.addWidget(self.btn_plan)

        # 預設演算法 = HARVEST（2026-10-08 使用者定調）；DCCPP 仍可從下拉手動切回。
        # ⚠ 一定要在 btn_plan 建好之後才切：currentIndexChanged → _on_algo_changed 會改按鈕
        #   文字、顯示 HARVEST 參數區、把「VTOL 數量」改成機隊組成的唯讀鏡子。
        self.algo_combo.setCurrentIndex(self.algo_combo.findData('hv_dccpp'))

        self.plan_status = QLabel("尚未規劃")
        self.plan_status.setWordWrap(True)
        lay.addWidget(self.plan_status)

        self.result_table = QTableWidget(0, 5)
        self.result_table.setHorizontalHeaderLabels(
            ["UAV", "航點", "航程 (km)", "預估 (min)", "構型"])
        self.result_table.verticalHeader().setVisible(False)
        self.result_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.result_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )
        self.result_table.setMinimumHeight(120)
        lay.addWidget(self.result_table)
        return g

    def _build_platform_row(self, *, image: str, title: str, specs: str,
                            spin: QSpinBox) -> QWidget:
        """一列＝一個構型：機體照片 ｜ 名稱與規格 ｜ 台數。

        兩個構型是不同的飛機，光看名字分不出來；放一張照片是最快的辨識方式。
        照片放在 ``ui/resources/images/``（該資料夾整包會被 PyInstaller 收進去，
        見 aeroplan_studio.spec 的 `_add_dir('ui/resources', ...)`）。
        ⚠ 檔案不在時**不要**留一個破圖：改顯示該放哪個檔名，並且整列照樣可用。
        """
        row = QWidget()
        lay = QHBoxLayout(row)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(10)

        pix = _platform_pixmap(image, _PLATFORM_THUMB_H)
        shot = QLabel()
        shot.setFixedWidth(_PLATFORM_THUMB_W)
        shot.setAlignment(Qt.AlignmentFlag.AlignCenter)
        if pix is not None:
            shot.setPixmap(pix)
        else:
            shot.setText(f"（{image}）")
            shot.setWordWrap(True)
            shot.setStyleSheet("color:#7a7a7a; font-size:10px;")
            shot.setToolTip(
                f"把機體照片放到 ui/resources/images/{image} 就會顯示在這裡。")
        lay.addWidget(shot)

        text = QVBoxLayout()
        text.setSpacing(2)
        name = QLabel(f"<b>{title}</b>")
        name.setTextFormat(Qt.TextFormat.RichText)
        text.addWidget(name)
        detail = QLabel(specs)
        detail.setWordWrap(True)
        detail.setStyleSheet("color:#9aa0a6;")
        text.addWidget(detail)
        lay.addLayout(text, 1)

        lay.addWidget(spin, 0, Qt.AlignmentFlag.AlignVCenter)

        # 規格同時放進 tooltip，滑到哪一格都看得到（含大圖）
        tip = f"{title}\n{specs}\n數值取自平台參數表，規劃時不可改。"
        if pix is not None:
            big = (_IMAGE_DIR / image).as_uri()
            tip = (f"<img src='{big}' width='260'><br><b>{title}</b><br>"
                   + specs.replace("\n", "<br>")
                   + "<br>數值取自平台參數表，規劃時不可改。")
        for w in (row, shot, name, detail, spin):
            w.setToolTip(tip)
        return row

    def _build_hv_group(self) -> QGroupBox:
        """HARVEST 專屬欄位（選到那個演算法時才顯示）。

        只放「既有 DCCPP 表達不了、而且不給就會出錯或被夾」的量：
        逐機構型、法規豁免天花板、C2 半徑、對地淨空、風。
        其餘（作業高度、條帶間距、最小間距、高度錯開量、協同模式）沿用上面的欄位。
        參數逐鍵去向見 vtol演篹法/docs/AEROPLAN_接入手冊.md §8。
        """
        g = QGroupBox("HARVEST 參數")
        outer = QVBoxLayout(g)
        _tune_group_layout(outer)

        # ── 機隊組成：兩種構型各自一格 ────────────────────────────
        #   它們是**不同的飛機**（轉彎半徑、巡航空速、續航、最低作業高度、酬載都不同），
        #   所以各給一個台數欄位。總台數 = 兩者相加；HV 模式下「4. 群飛編組」的
        #   VTOL 數量改為唯讀，免得畫面上出現兩個互相打架的總數。
        fleet = QGroupBox("機隊組成")
        fleet_lay = QVBoxLayout(fleet)
        _tune_group_layout(fleet_lay)

        # 兩型掛同一台相機（上面「感測器」那組的 HFOV／VFOV），所以酬載不是分工依據；
        # 誰做哪一塊完全由下面列的平台性能決定。見 mission/swarm_coordinator._uniform_camera。
        hint = QLabel("兩種構型掛同一台單光相機（上方感測器設定），"
                      "分工由續航、巡航速度、轉彎半徑與最低作業高度決定。")
        hint.setWordWrap(True)
        hint.setStyleSheet("color:#555; font-size:11px;")
        fleet_lay.addWidget(hint)

        self.hv_num_a_spin = QSpinBox()
        self.hv_num_a_spin.setRange(0, 16)
        self.hv_num_a_spin.setValue(1)
        self.hv_num_a_spin.setSuffix(" 台")
        self.hv_num_a_spin.valueChanged.connect(self._on_hv_fleet_changed)
        fleet_lay.addWidget(self._build_platform_row(
            image=_HV_PLATFORM_IMAGES['A'],
            title="Pioneer PNP",
            specs="巡航 21.5 m/s ｜ 轉彎半徑 200 m ｜ 最低作業高度 120 m AGL\n"
                  "續航 176 min ｜ 可用航程 133 km ｜ 酬載上限 5.0 kg",
            spin=self.hv_num_a_spin))

        self.hv_num_b_spin = QSpinBox()
        self.hv_num_b_spin.setRange(0, 16)
        self.hv_num_b_spin.setValue(1)
        self.hv_num_b_spin.setSuffix(" 台")
        self.hv_num_b_spin.valueChanged.connect(self._on_hv_fleet_changed)
        fleet_lay.addWidget(self._build_platform_row(
            image=_HV_PLATFORM_IMAGES['B'],
            title="HERO PNP",
            specs="巡航 19.0 m/s ｜ 轉彎半徑 120 m ｜ 最低作業高度 100 m AGL\n"
                  "續航 136 min ｜ 可用航程 79 km ｜ 酬載上限 1.0 kg",
            spin=self.hv_num_b_spin))

        self.hv_fleet_label = QLabel("")
        self.hv_fleet_label.setWordWrap(True)
        self.hv_fleet_label.setTextFormat(Qt.TextFormat.RichText)
        fleet_lay.addWidget(self.hv_fleet_label)
        outer.addWidget(fleet)

        # ── 任務限制：不給就會 NO_GO、或會被夾住的那幾項 ─────────────
        limits = QGroupBox("任務限制")
        form = _tune_form(QFormLayout(limits), is_group_layout=True)

        # ── AGL 上限：法規 120 m，只有拿到豁免才往上 ────────────────
        #   ⚠⚠⚠ 這個欄位餵的是 `Airspace.waiver_max_agl_m`，而規劃器用的是
        #   `h_ceil = waiver if given else max_agl_m`（max_agl_m 上游寫死 120，
        #   就是民航法規值）。所以「不勾」＝沒有豁免＝上限 120 m。
        #   以前預設 200 等於在使用者沒說的情況下**假設他拿到了 200 m 豁免**，
        #   規劃出來的任務違規而畫面上看不出來。
        self.hv_waiver_check = QCheckBox("已取得 AGL 豁免")
        self.hv_waiver_check.setChecked(False)
        self.hv_waiver_check.setToolTip(
            "不勾：AGL 上限 = 民航法規的 120 m。\n"
            "勾選：用右側的豁免值當上限（要真的持有豁免文件才勾）。")
        self.hv_waiver_spin = QDoubleSpinBox()
        self.hv_waiver_spin.setRange(121.0, 500.0)
        self.hv_waiver_spin.setDecimals(0)
        self.hv_waiver_spin.setSingleStep(10.0)
        self.hv_waiver_spin.setValue(200.0)
        self.hv_waiver_spin.setSuffix(" m AGL")
        self.hv_waiver_spin.setEnabled(False)
        self.hv_waiver_check.toggled.connect(self.hv_waiver_spin.setEnabled)
        self.hv_waiver_spin.setToolTip(
            "豁免後的 AGL 上限。可用高度帶 = 上限 − 最低固定翼作業高度，\n"
            "規劃器要求它至少 20 m 寬（還要再扣地形起伏）。\n"
            "\n"
            "實測（2.45 km²、1 Pioneer + 1 HERO、作業高度 120 m、GSD 0.05）：\n"
            "  上限 120 m（法規值）→ 整趟 NO_GO，連 HERO 都排不進去\n"
            "  上限 125–140 m → 只有 HERO（makespan 2085 s）\n"
            "  上限 ≥ 141 m   → 兩型都有（1424 s；200 m 時 1284 s）\n"
            "141 = Pioneer 的最低固定翼作業高度 120 + 高度帶下限 20 + 起伏。")
        _wrow = QHBoxLayout()
        _wrow.setContentsMargins(0, 0, 0, 0)
        _wrow.addWidget(self.hv_waiver_check)
        _wrow.addWidget(self.hv_waiver_spin, 1)
        _wwrap = QWidget()
        _wwrap.setLayout(_wrow)
        form.addRow("AGL 上限（法規 120 m）:", _wwrap)

        # ── 最低固定翼作業高度 ───────────────────────────────────
        #   ⚠⚠⚠ 表 1 的 120／100 **不是機體能力極限**：那一列的使用方式欄
        #   寫的是「任務前需依地形、感測器與法規調整」，而且它沒有像旁邊的
        #   轉換高度那樣標「暫定」—— 它是使用者給的規劃值，全 repo 找不到
        #   任何原廠或法規出處。規格 D35 更明文只讓它對 ENTRY/OPER/TRANS/RTB
        #   生效，CLIMB/DESCENT 不含它、ACCEL/DECEL 落在 80–100 m。
        #   ⚠ 下界綁在轉換完成高度上緣（A 100、B 90）：再低就是要求飛機在
        #   轉換完成前用固定翼巡航，那已經沒有依據了。
        self.hv_minfw_a_spin = QDoubleSpinBox()
        self.hv_minfw_a_spin.setRange(100.0, 300.0)
        self.hv_minfw_a_spin.setDecimals(0)
        self.hv_minfw_a_spin.setSingleStep(5.0)
        self.hv_minfw_a_spin.setValue(100.0)
        self.hv_minfw_a_spin.setSuffix(" m")
        self.hv_minfw_a_spin.setToolTip(
            "Pioneer 的最低固定翼作業高度。表 1 是 120，但那是規劃值不是機體極限，\n"
            "而且 120 剛好等於法規上限 ⇒ 可用高度帶 0，沒有豁免就一趟都飛不了。\n"
            "預設調成 100（與 HERO 相同，也正好是它的轉換完成高度上緣）。\n"
            "\n"
            "⚠ 這個值會寫進匯出參數的 Q_RTL_ALT ⇒ QRTL 進場高度跟著降\n"
            "（HV4SITL8 讀回 Q_RTL_ALT=100 已確認）。FENCE_ALT_MIN 也綁著它，\n"
            "但那只在走 mavlink_link 寫參數那條路時才送出。下界 100 不可再低。")
        self.hv_minfw_b_spin = QDoubleSpinBox()
        self.hv_minfw_b_spin.setRange(90.0, 300.0)
        self.hv_minfw_b_spin.setDecimals(0)
        self.hv_minfw_b_spin.setSingleStep(5.0)
        self.hv_minfw_b_spin.setValue(100.0)
        self.hv_minfw_b_spin.setSuffix(" m")
        self.hv_minfw_b_spin.setToolTip(
            "HERO 的最低固定翼作業高度（表 1 值 100）。下界 90 是它的轉換完成高度上緣。")
        _mrow = QHBoxLayout()
        _mrow.setContentsMargins(0, 0, 0, 0)
        _mrow.addWidget(QLabel("A"))
        _mrow.addWidget(self.hv_minfw_a_spin, 1)
        _mrow.addWidget(QLabel("B"))
        _mrow.addWidget(self.hv_minfw_b_spin, 1)
        _mwrap = QWidget()
        _mwrap.setLayout(_mrow)
        form.addRow("最低固定翼作業高度:", _mwrap)

        # ── 高度帶下限 ───────────────────────────────────────────
        #   規格 §4.6：「坡度包絡在起伏地形上需要的活動空間」，來源標 D（本文預設），
        #   不是任務輸入，而且沒有任何文件寫出它為什麼是 20。
        #   ⚠ h_ceil 120 配 min_fw 100 時可用窗恰好 20 m ⇒ 留 20 等於要求地形
        #     起伏完全為零，**任何**起伏都會讓整趟 NO_GO。上游自己在 h_ceil=120
        #     的測試就用 15（tests/hv_dccpp/test_preflight.py:354），這裡跟進。
        self.hv_band_spin = QDoubleSpinBox()
        self.hv_band_spin.setRange(5.0, 60.0)
        self.hv_band_spin.setDecimals(0)
        self.hv_band_spin.setSingleStep(1.0)
        self.hv_band_spin.setValue(15.0)
        self.hv_band_spin.setSuffix(" m")
        self.hv_band_spin.setToolTip(
            "可用高度帶的下限：h_ceil − 最低固定翼作業高度 − 地形起伏 要 ≥ 它。\n"
            "規格預設 20，但 120 m 法規上限配 100 m 最低作業高度時窗只有 20 m，\n"
            "留 20 等於不容許任何地形起伏。15 留 5 m 給起伏（上游測試同值）。\n"
            "\n"
            "⚠ 同一個數字還被相機的 GSD 可達性判準（h_max ≥ 最低作業高度 + 它）\n"
            "與相機預設目標高借用，調它等於三件事一起動。")
        form.addRow("高度帶下限:", self.hv_band_spin)

        self.hv_c2_spin = QDoubleSpinBox()
        self.hv_c2_spin.setRange(1.0, 200.0)
        self.hv_c2_spin.setDecimals(1)
        self.hv_c2_spin.setValue(25.0)
        self.hv_c2_spin.setSuffix(" km")
        self.hv_c2_spin.setToolTip(
            "C2 鏈路半徑。平台參數表列為「待填入」⇒ 必須由任務給，缺了會 NO_GO。\n"
            "請填實測值，不要沿用這個預設。")
        form.addRow("C2 鏈路半徑:", self.hv_c2_spin)

        self.hv_clearance_spin = QDoubleSpinBox()
        self.hv_clearance_spin.setRange(0.0, 200.0)
        self.hv_clearance_spin.setDecimals(0)
        self.hv_clearance_spin.setValue(0.0)
        self.hv_clearance_spin.setSuffix(" m")
        self.hv_clearance_spin.setSpecialValueText("自動（max(36, 3σ+30)）")
        self.hv_clearance_spin.setToolTip("對地淨空。0 = 由 DEM 高程誤差自動推導。")
        form.addRow("對地淨空:", self.hv_clearance_spin)

        # ── 地形來源：Cesium World Terrain（串流）──────────────────────
        from utils.cesium_ion import configured_ion_token
        _tok, _ = configured_ion_token()
        self.hv_world_terrain_chk = QCheckBox("使用 Cesium World Terrain 地形")
        self.hv_world_terrain_chk.setChecked(bool(_tok))
        self.hv_world_terrain_chk.setEnabled(bool(_tok))
        self.hv_world_terrain_chk.setToolTip(
            "規劃時從 Cesium ion 串流 World Terrain 高程（約 35 m 間距），取代「沒有 DEM ⇒ 當海平面 0 m」。\n"
            "⚠ 需要 ion token（config/settings.json 的 map.cesium_ion_token 或環境變數 CESIUM_ION_TOKEN）；"
            + ("目前沒有 token，已停用。\n" if not _tok else "\n")
            + "⚠ 免費 Community 方案：個人/非商用、串流 15 GB/月；有研究經費或商用要付費。\n"
            "⚠ 授權：高程只放記憶體、不存檔（條款禁止儲存供離線使用）。\n"
            "⚠ 高程是橢球高（台灣比海拔高約 15–22 m），規劃只用相對高度。\n"
            "⚠ 真實地形有起伏 ⇒ 120 m 法規上限配 100 m 最低作業高度會 NO_GO，\n"
            "   需要在上面勾選豁免並提高 AGL 上限（NO_GO 訊息會算出需要多少）。")
        form.addRow("地形:", self.hv_world_terrain_chk)

        # ── 找平坦位置（需要 token；用同一份 World Terrain 掃描作業區附近）───────
        #   為什麼：120 m 法規上限下，可用高度帶只容得下 ≤15 m（Pioneer）/ ≤25 m（HERO、最低設定）的起伏；
        #   不想申請豁免就得換到夠平的地方。結果是**預估**（見 flat_candidates_dialog 的說明）。
        self._flat_token_ok = bool(_tok)
        self._flat_dialog = None
        self.hv_flat_radius_spin = QDoubleSpinBox()
        self.hv_flat_radius_spin.setRange(3.0, 30.0)
        self.hv_flat_radius_spin.setDecimals(0)
        self.hv_flat_radius_spin.setValue(15.0)
        self.hv_flat_radius_spin.setSuffix(" km")
        self.hv_flat_radius_spin.setToolTip("以目前作業區中心為圓心的搜尋半徑。越大越慢（要下載更多地形圖磚）。")
        self.hv_flat_scale_spin = QDoubleSpinBox()
        self.hv_flat_scale_spin.setRange(30.0, 100.0)
        self.hv_flat_scale_spin.setDecimals(0)
        self.hv_flat_scale_spin.setSingleStep(10.0)
        self.hv_flat_scale_spin.setValue(100.0)
        self.hv_flat_scale_spin.setSuffix(" % 大小")
        self.hv_flat_scale_spin.setToolTip(
            "要找多大的窗口（相對目前作業區的邊長）。同尺寸找不到時調小：窗口越小越容易找到夠平的地方\n"
            "（桃園實測：原大小 0 個、70% 少數、50% 很多）。平移時作業區也會縮成這個大小。")
        self.btn_find_flat = IconButton("pin", "找平坦位置…", tone="primary", compact=True)
        self.btn_find_flat.setEnabled(False)
        self.btn_find_flat.setToolTip(
            "在作業區附近找起伏夠小、不用豁免也能規劃的位置（用 Cesium World Terrain）。\n"
            "需要先圈好作業區與 ion token；結果是預估，不是保證。")
        self.btn_find_flat.clicked.connect(self._emit_find_flat)
        _frow = QHBoxLayout()
        _frow.setContentsMargins(0, 0, 0, 0)
        _frow.addWidget(self.hv_flat_radius_spin)
        _frow.addWidget(self.hv_flat_scale_spin)
        _frow.addWidget(self.btn_find_flat, 1)
        _fwrap = QWidget()
        _fwrap.setLayout(_frow)
        form.addRow("平坦位置:", _fwrap)
        self.flat_status_label = QLabel("")
        self.flat_status_label.setWordWrap(True)
        self.flat_status_label.setProperty("role", "caption")
        form.addRow("", self.flat_status_label)

        self.hv_wind_speed_spin = QDoubleSpinBox()
        self.hv_wind_speed_spin.setRange(0.0, 25.0)
        self.hv_wind_speed_spin.setDecimals(1)
        self.hv_wind_speed_spin.setSingleStep(0.5)
        self.hv_wind_speed_spin.setValue(0.0)
        self.hv_wind_speed_spin.setSuffix(" m/s")
        self.hv_wind_speed_spin.setToolTip(
            "平均風速。會修正地速、耗時與**轉彎半徑**：\n"
            "實測 8 m/s 時 A 的 200 m → 244.7 m、B 的 120 m → 151.5 m。\n"
            "風速太大會撞到起降場地的陣風限制而直接 NO_GO。")
        form.addRow("平均風速:", self.hv_wind_speed_spin)

        self.hv_wind_dir_spin = QDoubleSpinBox()
        self.hv_wind_dir_spin.setRange(0.0, 359.0)
        self.hv_wind_dir_spin.setDecimals(0)
        self.hv_wind_dir_spin.setSingleStep(5.0)
        self.hv_wind_dir_spin.setValue(0.0)
        self.hv_wind_dir_spin.setSuffix(" °")
        self.hv_wind_dir_spin.setToolTip("風**從哪個方位吹來**（氣象慣例，0=北）。影響逆風進場方向。")
        form.addRow("風向（風從何方來）:", self.hv_wind_dir_spin)

        self.hv_note_label = QLabel(
            "作業高度、條帶間距、最小間距與高度錯開量沿用上方欄位。\n"
            "注意：民航法規的 AGL 上限是 120 m，而表 1 給 Pioneer 的最低固定翼作業"
            "高度剛好也是 120 m ⇒ 可用高度帶為零。但表 1 那一列的使用方式欄寫的是"
            "「任務前需依地形、感測器與法規調整」，它是規劃值不是機體極限，"
            "所以本頁預設把它調成 100 m（與 HERO 相同，也是它的轉換完成高度上緣）。\n"
            "注意：100 m 配 120 m 上限剛好只有 20 m 的窗，等於不容許任何地形起伏，"
            "所以高度帶下限預設 15（規格值 20）。實測 2.45 km²："
            "120/100/20 → NO_GO；100/100/20 → 仍 NO_GO（只差不到 1 m 起伏）；"
            "100/100/15 → 兩型都有工作，makespan 2085 → 1424 s。\n"
            "注意：另一個獨立的旋鈕是「目標 GSD」：它反算相機的高度上限 h_max，"
            "而規劃器要求 h_max ≥ 最低作業高度 + 20 m。不填時系統合成 1.05×作業高度"
            "（120 m 下只有 126 m）⇒ Pioneer 分不到工作，畫面只顯示「1/2 架」。"
            "填 0.05 m/px（h_max 205 m）就正常；嚴到 0.03（123 m）又會被排除。\n"
            "未提供備降點時系統會自動放一點，那個點不見得能降落。")
        self.hv_note_label.setWordWrap(True)
        form.addRow("", self.hv_note_label)
        outer.addWidget(limits)

        outer.addWidget(self._build_mapping_group())

        self._on_hv_fleet_changed()
        return g

    # ── 航測建圖（HARVEST 專屬；預設關閉）──────────────────────────
    def _build_mapping_group(self) -> QGroupBox:
        """「航測建圖」：真相機、前向重疊、拍照指令（206）與上傳前參數讀回。

        預設**不勾** ⇒ 不建圖的使用者畫面與送出的規劃完全不變（get_params 的 ``hv_map_enabled`` = False，
        規劃器忽略其餘 ``hv_map_*``）。勾選後：

        * 上面「感測器足跡」的 HFOV／VFOV／影像寬度／掛載角改成這台相機的**唯讀鏡子**（掛載角 0 ＝ 正下視），
          足跡、條帶間距與覆蓋率驗算才會跟規劃器用同一台相機；取消勾選就還原原本的值。
        * 側向重疊仍是上面的「重疊率」（< 0.60 會提醒）；前向重疊在這裡（預設 0.80）。
        * 觸發率沒有預設證明值：預設只是佔位，顯示「未經台架驗證」直到勾「已台架驗證」。
        """
        g = QGroupBox("航測建圖")
        lay = QVBoxLayout(g)
        _tune_group_layout(lay)

        self.hv_map_check = QCheckBox("建圖模式")
        self.hv_map_check.setChecked(False)
        self.hv_map_check.setToolTip(
            "勾選：規劃改用下面這台真相機與前向重疊、開接縫共用重疊／不分層／偏流餘裕／引入段，\n"
            "上傳改走 hv 匯出器（每條帶 DO_SET_CAM_TRIGG_DIST 開／關），上傳前先讀回飛控相機參數比對。\n"
            "不勾：一切照舊。")
        lay.addWidget(self.hv_map_check)

        form = _tune_form(QFormLayout())
        self.hv_map_preset_combo = QComboBox()
        for key, pre in MAPPING_CAMERA_PRESETS.items():
            self.hv_map_preset_combo.addItem(pre["label"], key)
        self.hv_map_preset_combo.addItem("自訂", MAPPING_PRESET_CUSTOM)
        self.hv_map_preset_combo.setToolTip("選一台預設相機會填入下面的焦距／像素／解析度；改了任何一項就變成「自訂」。")
        # 名稱很長（「RPi Camera Module 3（IMX708）4.74 mm」）：不讓下拉把整頁撐寬
        self.hv_map_preset_combo.setSizeAdjustPolicy(
            QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.hv_map_preset_combo.setMinimumContentsLength(14)
        form.addRow("相機:", self.hv_map_preset_combo)

        self.hv_map_focal_spin = QDoubleSpinBox()
        self.hv_map_focal_spin.setRange(1.0, 200.0)
        self.hv_map_focal_spin.setDecimals(2)
        self.hv_map_focal_spin.setSuffix(" mm")
        self.hv_map_pixel_spin = QDoubleSpinBox()
        self.hv_map_pixel_spin.setRange(0.5, 20.0)
        self.hv_map_pixel_spin.setDecimals(4)
        self.hv_map_pixel_spin.setSuffix(" µm")
        _fp = QHBoxLayout()
        _fp.setContentsMargins(0, 0, 0, 0)
        _fp.addWidget(self.hv_map_focal_spin, 1)
        _fp.addWidget(QLabel("像素"))
        _fp.addWidget(self.hv_map_pixel_spin, 1)
        _fpw = QWidget()
        _fpw.setLayout(_fp)
        form.addRow("焦距:", _fpw)

        self.hv_map_width_spin = QSpinBox()
        self.hv_map_width_spin.setRange(320, 20000)
        self.hv_map_width_spin.setSuffix(" px")
        self.hv_map_height_spin = QSpinBox()
        self.hv_map_height_spin.setRange(240, 20000)
        self.hv_map_height_spin.setSuffix(" px")
        _wh = QHBoxLayout()
        _wh.setContentsMargins(0, 0, 0, 0)
        _wh.addWidget(self.hv_map_width_spin, 1)
        _wh.addWidget(QLabel("×"))
        _wh.addWidget(self.hv_map_height_spin, 1)
        _whw = QWidget()
        _whw.setLayout(_wh)
        form.addRow("影像寬×高:", _whw)

        self.hv_map_rate_spin = QDoubleSpinBox()
        self.hv_map_rate_spin.setRange(0.1, 30.0)
        self.hv_map_rate_spin.setDecimals(2)
        self.hv_map_rate_spin.setSingleStep(0.1)
        self.hv_map_rate_spin.setSuffix(" Hz")
        self.hv_map_rate_spin.setToolTip(
            "台架實測、能連續撐完整個架次（全解析度 JPEG 寫入不掉張）的最大觸發率。\n"
            "CAM1_INTRVAL_MIN = 1/此值向上取整到 0.02 s，規劃器的拍照率上限用同一個數。\n"
            "relay／servo 後端在 CAM1_DURATION 0.1 s 時最高約 7.14 Hz（CAM1_INTRVAL_MIN ≥ 0.14 s），超過會擋規劃。\n"
            "預設值只是佔位，不是證明值。")
        self.hv_map_rate_verified_check = QCheckBox("已台架驗證")
        _rt = QHBoxLayout()
        _rt.setContentsMargins(0, 0, 0, 0)
        _rt.addWidget(self.hv_map_rate_spin, 1)
        _rt.addWidget(self.hv_map_rate_verified_check)
        _rtw = QWidget()
        _rtw.setLayout(_rt)
        form.addRow("實測最大觸發率:", _rtw)
        self.hv_map_rate_status_label = QLabel("")
        self.hv_map_rate_status_label.setWordWrap(True)
        form.addRow("", self.hv_map_rate_status_label)

        self.hv_map_readout_spin = QDoubleSpinBox()
        self.hv_map_readout_spin.setRange(0.0, 500.0)
        self.hv_map_readout_spin.setDecimals(1)
        self.hv_map_readout_spin.setSuffix(" ms")
        self.hv_map_readout_spin.setSpecialValueText("未填")
        self.hv_map_readout_spin.setToolTip(
            "滾動快門讀出時間（選填；給後處理的滾動快門修正）。公開數字全是推算值，請用 LED 頻閃實測。")
        form.addRow("讀出時間:", self.hv_map_readout_spin)

        self.hv_map_fwd_spin = QDoubleSpinBox()
        self.hv_map_fwd_spin.setRange(0.0, 0.95)
        self.hv_map_fwd_spin.setDecimals(2)
        self.hv_map_fwd_spin.setSingleStep(0.05)
        self.hv_map_fwd_spin.setToolTip("沿航向的照片重疊；觸發距離 = 足跡縱長 ×(1 − 前向重疊)。"
                                        "側向重疊是上方「感測器足跡」的「重疊率」。")
        form.addRow("前向重疊:", self.hv_map_fwd_spin)

        self.hv_map_backend_combo = QComboBox()
        for key, label in MAPPING_BACKENDS:
            self.hv_map_backend_combo.addItem(label, key)
        self.hv_map_backend_combo.setToolTip(
            "relay（預設）：CAM1_TYPE 2，GPIO 脈衝（實機用，需韌體版本與 relay 腳位）；\n"
            "servo：CAM1_TYPE 1，會持續輸出 PWM 脈衝，只適用 SITL（實機請用 relay）；\n"
            "mavlink_v2：CAM1_TYPE 6，MAVLink 相機（ArduPlane ≥ 4.5，需韌體版本）。")
        form.addRow("相機後端:", self.hv_map_backend_combo)

        self.hv_map_firmware_edit = QLineEdit()
        self.hv_map_firmware_edit.setPlaceholderText("例如 4.5.7（relay／mavlink_v2 必填）")
        self.hv_map_firmware_edit.setToolTip("每架飛控的 ArduPlane 版本（relay 參數名依版本而異，不知道就不猜）。")
        form.addRow("韌體版本:", self.hv_map_firmware_edit)

        self.hv_map_relay_pin_spin = QSpinBox()
        self.hv_map_relay_pin_spin.setRange(-1, 1015)
        self.hv_map_relay_pin_spin.setValue(-1)
        self.hv_map_relay_pin_spin.setSpecialValueText("未填")
        self.hv_map_relay_pin_spin.setToolTip("RELAYn_PIN（GPIO 腳號，例如 Cube AUX1 = 50）。")
        self.hv_map_relay_aux_spin = QSpinBox()
        self.hv_map_relay_aux_spin.setRange(0, 32)
        self.hv_map_relay_aux_spin.setSpecialValueText("未填")
        self.hv_map_relay_aux_spin.setToolTip("該腳所屬的 SERVOn（會設 SERVOn_FUNCTION = -1 當 GPIO；Cube AUX1 = SERVO9）。")
        _rl = QHBoxLayout()
        _rl.setContentsMargins(0, 0, 0, 0)
        _rl.addWidget(self.hv_map_relay_pin_spin, 1)
        _rl.addWidget(QLabel("AUX 伺服"))
        _rl.addWidget(self.hv_map_relay_aux_spin, 1)
        _rlw = QWidget()
        _rlw.setLayout(_rl)
        form.addRow("relay 腳位:", _rlw)

        self.hv_map_fb_pin_spin = QSpinBox()
        self.hv_map_fb_pin_spin.setRange(-1, 127)
        self.hv_map_fb_pin_spin.setValue(-1)
        self.hv_map_fb_pin_spin.setSpecialValueText("無")
        self.hv_map_fb_pin_spin.setToolTip("CAM1_FEEDBAK_PIN（相機回授腳；-1 = 無）。")
        self.hv_map_fb_servo_spin = QSpinBox()
        self.hv_map_fb_servo_spin.setRange(0, 32)
        self.hv_map_fb_servo_spin.setSpecialValueText("未填")
        self.hv_map_fb_servo_spin.setToolTip("回授腳所屬的 SERVOn（會設 SERVOn_FUNCTION = -1）。")
        _fb = QHBoxLayout()
        _fb.setContentsMargins(0, 0, 0, 0)
        _fb.addWidget(self.hv_map_fb_pin_spin, 1)
        _fb.addWidget(QLabel("通道"))
        _fb.addWidget(self.hv_map_fb_servo_spin, 1)
        _fbw = QWidget()
        _fbw.setLayout(_fb)
        form.addRow("回授腳位:", _fbw)

        self.hv_map_roll_spin = QDoubleSpinBox()
        self.hv_map_roll_spin.setRange(0.0, 90.0)
        self.hv_map_roll_spin.setDecimals(0)
        self.hv_map_roll_spin.setSuffix(" °")
        self.hv_map_roll_spin.setToolTip("CAM_MAX_ROLL：滾轉超過此角度時延後距離觸發（0 = 不限）。")
        form.addRow("CAM_MAX_ROLL:", self.hv_map_roll_spin)

        self.hv_map_lead_spin = QDoubleSpinBox()
        self.hv_map_lead_spin.setRange(0.0, 500.0)
        self.hv_map_lead_spin.setDecimals(0)
        self.hv_map_lead_spin.setSuffix(" m")
        self.hv_map_lead_spin.setToolTip(
            "引入段：條帶兩端各外延這麼多，且在帶起點後這麼遠才開相機（轉彎改出後才拍）。\n"
            "規劃器的外延與匯出器開相機的距離是同一個數。")
        form.addRow("引入段:", self.hv_map_lead_spin)

        self.hv_map_retrig_spin = QDoubleSpinBox()
        self.hv_map_retrig_spin.setRange(0.0, 0.99)
        self.hv_map_retrig_spin.setDecimals(2)
        self.hv_map_retrig_spin.setSingleStep(0.05)
        self.hv_map_retrig_spin.setSpecialValueText("關閉")
        self.hv_map_retrig_spin.setToolTip("條帶中途航點的 AGL 與觸發距離所依據的 AGL 相對差超過此值時，重發 206 改觸發距離。")
        form.addRow("地形重觸發容差:", self.hv_map_retrig_spin)

        # ── 交叉航帶（與加密 GCP 對照用；預設不勾）──────────────────
        self.hv_map_cross_check = QCheckBox("交叉航帶")
        self.hv_map_cross_check.setChecked(False)
        self.hv_map_cross_check.setToolTip(
            "每架在自己飛的每個區塊多飛一組與主條帶垂直的條帶（區塊中央、同一架飛）。\n"
            "文獻：平地、GCP 少時效益大（Gerke & Przybilla 2016：同時換方向與高度、約兩成面積）；\n"
            "同高度垂直重飛只有小幅改善（Štroner 2021）。代價是多飛、多轉彎，鄰區可能要錯開時間。")
        form.addRow("", self.hv_map_cross_check)
        self.hv_map_cross_frac_spin = QDoubleSpinBox()
        self.hv_map_cross_frac_spin.setRange(0.05, 1.0)
        self.hv_map_cross_frac_spin.setDecimals(2)
        self.hv_map_cross_frac_spin.setSingleStep(0.05)
        self.hv_map_cross_frac_spin.setToolTip("交叉航帶蓋住區塊面積的比例（中央橫帶）；至少 2 條，放不下就略過該區塊並警告。")
        self.hv_map_cross_dh_spin = QDoubleSpinBox()
        self.hv_map_cross_dh_spin.setRange(-100.0, 100.0)
        self.hv_map_cross_dh_spin.setDecimals(0)
        self.hv_map_cross_dh_spin.setSingleStep(5.0)
        self.hv_map_cross_dh_spin.setSuffix(" m")
        self.hv_map_cross_dh_spin.setToolTip(
            "交叉航帶高度 = 主條帶高度 + 此值；超出高度帶或低於最低固定翼高度 ⇒ 不規劃（不夾限）。")
        _cx = QHBoxLayout()
        _cx.addWidget(QLabel("面積比例"))
        _cx.addWidget(self.hv_map_cross_frac_spin, 1)
        _cx.addWidget(QLabel("高度差"))
        _cx.addWidget(self.hv_map_cross_dh_spin, 1)
        form.addRow("", _cx)

        self.hv_map_max_wind_spin = QDoubleSpinBox()
        self.hv_map_max_wind_spin.setRange(0.5, 15.0)
        self.hv_map_max_wind_spin.setDecimals(1)
        self.hv_map_max_wind_spin.setSingleStep(0.5)
        self.hv_map_max_wind_spin.setSuffix(" m/s")
        self.hv_map_max_wind_spin.setToolTip(
            "上方「平均風速」（規劃風速）超過此值 ⇒ 規劃後不可上傳。\n"
            "⚠ 這個上限指的是**現場實際風速**：規劃只檢查你輸入的風速，起飛前必須在場實測。")
        form.addRow("建圖作業風速上限:", self.hv_map_max_wind_spin)

        self.hv_map_tlog_check = QCheckBox("記錄 tlog")
        self.hv_map_tlog_check.setToolTip("上傳時開始記錄每架收到的 MAVLink（標準 .tlog），\n"
                                          "存到 runs/survey/<日期>/UAV-n_<時分秒>.tlog；斷線時關檔。")
        form.addRow("", self.hv_map_tlog_check)
        lay.addLayout(form)

        self.hv_map_warn_label = QLabel("")
        self.hv_map_warn_label.setWordWrap(True)
        lay.addWidget(self.hv_map_warn_label)
        self.hv_map_status_label = QLabel("")
        self.hv_map_status_label.setWordWrap(True)
        lay.addWidget(self.hv_map_status_label)
        self.hv_map_live_label = QLabel("")
        self.hv_map_live_label.setWordWrap(True)
        lay.addWidget(self.hv_map_live_label)

        # 預設值（相機預設 → 觸發率佔位）
        self._map_saved_sensor = None
        self._map_applying_preset = False
        self.hv_map_fwd_spin.setValue(MAPPING_FORWARD_OVERLAP_DEFAULT)
        self.hv_map_roll_spin.setValue(MAPPING_MAX_ROLL_DEFAULT_DEG)
        self.hv_map_lead_spin.setValue(MAPPING_LEAD_IN_DEFAULT_M)
        self.hv_map_retrig_spin.setValue(MAPPING_RETRIGGER_TOL_DEFAULT)
        self.hv_map_cross_frac_spin.setValue(MAPPING_CROSS_FRACTION_DEFAULT)
        self.hv_map_cross_dh_spin.setValue(MAPPING_CROSS_ALT_OFFSET_DEFAULT_M)
        self.hv_map_max_wind_spin.setValue(MAPPING_MAX_WIND_DEFAULT_MPS)
        self._apply_mapping_preset(next(iter(MAPPING_CAMERA_PRESETS)))

        self.hv_map_check.toggled.connect(self._on_mapping_toggled)
        self.hv_map_preset_combo.currentIndexChanged.connect(self._on_mapping_preset_changed)
        for w in (self.hv_map_focal_spin, self.hv_map_pixel_spin, self.hv_map_width_spin, self.hv_map_height_spin):
            w.valueChanged.connect(self._on_mapping_camera_edited)
        self.hv_map_rate_spin.valueChanged.connect(self._on_mapping_rate_edited)
        self.hv_map_rate_verified_check.toggled.connect(self._refresh_mapping_texts)
        self.hv_map_backend_combo.currentIndexChanged.connect(self._sync_mapping_enabled)
        self.hv_map_fb_pin_spin.valueChanged.connect(self._sync_mapping_enabled)
        for w in (self.hv_map_readout_spin, self.hv_map_fwd_spin, self.hv_map_relay_pin_spin,
                  self.hv_map_relay_aux_spin, self.hv_map_fb_servo_spin, self.hv_map_roll_spin,
                  self.hv_map_lead_spin, self.hv_map_retrig_spin, self.overlap_spin,
                  self.hv_map_cross_frac_spin, self.hv_map_cross_dh_spin, self.hv_map_max_wind_spin,
                  self.hv_wind_speed_spin):
            w.valueChanged.connect(self._refresh_mapping_texts)
        self.hv_map_cross_check.toggled.connect(self._sync_mapping_enabled)
        self.hv_map_firmware_edit.textChanged.connect(self._refresh_mapping_texts)
        self._sync_mapping_enabled()
        return g

    def _apply_mapping_preset(self, key: str) -> None:
        """把預設相機的規格填進欄位；觸發率填佔位並取消「已台架驗證」（換相機 ⇒ 舊的驗證不算數）。"""
        pre = MAPPING_CAMERA_PRESETS.get(key)
        if pre is None:
            return
        self._map_applying_preset = True
        try:
            idx = self.hv_map_preset_combo.findData(key)
            if idx >= 0 and idx != self.hv_map_preset_combo.currentIndex():
                blocked = self.hv_map_preset_combo.blockSignals(True)
                self.hv_map_preset_combo.setCurrentIndex(idx)
                self.hv_map_preset_combo.blockSignals(blocked)
            self.hv_map_focal_spin.setValue(pre["focal_mm"])
            self.hv_map_pixel_spin.setValue(pre["pixel_um"])
            self.hv_map_width_spin.setValue(pre["width_px"])
            self.hv_map_height_spin.setValue(pre["height_px"])
            self.hv_map_rate_spin.setValue(MAPPING_TRIGGER_HZ_PLACEHOLDER)
            self.hv_map_rate_verified_check.setChecked(False)
        finally:
            self._map_applying_preset = False
        self._sync_mapping_mirror()
        self._refresh_mapping_texts()

    def _on_mapping_preset_changed(self, *_a) -> None:
        key = self.hv_map_preset_combo.currentData()
        if key != MAPPING_PRESET_CUSTOM:
            self._apply_mapping_preset(key)
        self._refresh_mapping_texts()

    def _on_mapping_camera_edited(self, *_a) -> None:
        """手改焦距／像素／解析度 ⇒ 不再是那台預設相機，下拉改成「自訂」（不默默掛著預設的名字）。"""
        if self._map_applying_preset:
            return
        key = self.hv_map_preset_combo.currentData()
        pre = MAPPING_CAMERA_PRESETS.get(key)
        if pre is not None and (abs(self.hv_map_focal_spin.value() - pre["focal_mm"]) > 1e-9
                                or abs(self.hv_map_pixel_spin.value() - pre["pixel_um"]) > 1e-9
                                or self.hv_map_width_spin.value() != pre["width_px"]
                                or self.hv_map_height_spin.value() != pre["height_px"]):
            blocked = self.hv_map_preset_combo.blockSignals(True)
            self.hv_map_preset_combo.setCurrentIndex(self.hv_map_preset_combo.findData(MAPPING_PRESET_CUSTOM))
            self.hv_map_preset_combo.blockSignals(blocked)
        self._sync_mapping_mirror()
        self._refresh_mapping_texts()

    def _on_mapping_rate_edited(self, *_a) -> None:
        """觸發率改了 ⇒ 「已台架驗證」是對舊數字說的，取消勾選。"""
        if not self._map_applying_preset:
            self.hv_map_rate_verified_check.setChecked(False)
        self._refresh_mapping_texts()

    def mapping_enabled(self) -> bool:
        """建圖模式是否生效（只在 HARVEST 演算法下）。"""
        return bool(self.hv_map_check.isChecked() and self.algo_combo.currentData() == 'hv_dccpp')

    def _mapping_fov_deg(self) -> tuple:
        """這台相機的 (HFOV°, VFOV°)：2·atan(寬或高 × 像素 / 2f)。"""
        f = self.hv_map_focal_spin.value()
        p_mm = self.hv_map_pixel_spin.value() * 1e-3
        w_mm = self.hv_map_width_spin.value() * p_mm
        h_mm = self.hv_map_height_spin.value() * p_mm
        return (2.0 * math.degrees(math.atan(w_mm / (2.0 * f))),
                2.0 * math.degrees(math.atan(h_mm / (2.0 * f))))

    def _sync_mapping_mirror(self) -> None:
        """建圖模式下，感測器足跡的 HFOV／VFOV／影像寬度／掛載角是這台相機的鏡子（掛載角 0 ＝ 正下視）。"""
        if not getattr(self, "hv_map_check", None) or not self.hv_map_check.isChecked():
            return
        hfov, vfov = self._mapping_fov_deg()
        self.hfov_spin.setValue(hfov)
        self.vfov_spin.setValue(vfov)
        self.img_width_spin.setValue(self.hv_map_width_spin.value())
        self.mount_spin.setValue(0.0)

    def _on_mapping_toggled(self, on: bool) -> None:
        sensor = (self.hfov_spin, self.vfov_spin, self.img_width_spin, self.mount_spin)
        if on:
            if self._map_saved_sensor is None:
                self._map_saved_sensor = tuple(w.value() for w in sensor)
            self._sync_mapping_mirror()
            logger.info(f"[VTOL 偵察] 航測建圖開啟：感測器足跡改用面板相機 "
                        f"（HFOV {self.hfov_spin.value():.2f}°／VFOV {self.vfov_spin.value():.2f}°／"
                        f"寬 {self.img_width_spin.value()} px／掛載角 0°）")
        elif self._map_saved_sensor is not None:
            for w, v in zip(sensor, self._map_saved_sensor):
                w.setValue(v)
            self._map_saved_sensor = None
            logger.info("[VTOL 偵察] 航測建圖關閉：感測器足跡還原為開啟前的值")
        for w in sensor:
            w.setEnabled(not on)          # 建圖模式下由「航測建圖」的相機決定（唯讀鏡子）
        self._sync_mapping_enabled()

    def _sync_mapping_enabled(self, *_a) -> None:
        on = self.hv_map_check.isChecked()
        relay = self.hv_map_backend_combo.currentData() == "relay"
        for w in (self.hv_map_preset_combo, self.hv_map_focal_spin, self.hv_map_pixel_spin, self.hv_map_width_spin,
                  self.hv_map_height_spin, self.hv_map_rate_spin, self.hv_map_rate_verified_check,
                  self.hv_map_readout_spin, self.hv_map_fwd_spin, self.hv_map_backend_combo,
                  self.hv_map_firmware_edit, self.hv_map_fb_pin_spin, self.hv_map_roll_spin, self.hv_map_lead_spin,
                  self.hv_map_retrig_spin, self.hv_map_tlog_check, self.hv_map_cross_check, self.hv_map_max_wind_spin):
            w.setEnabled(on)
        # 交叉航帶的比例／高度差只在勾了交叉航帶時可改（沒勾時送 None，不送會被忽略的值）
        self.hv_map_cross_frac_spin.setEnabled(on and self.hv_map_cross_check.isChecked())
        self.hv_map_cross_dh_spin.setEnabled(on and self.hv_map_cross_check.isChecked())
        # 只對某一後端／回授腳有意義的欄位：用不到就鎖起來（送出的值也是 None，不會被默默吃掉）
        self.hv_map_relay_pin_spin.setEnabled(on and relay)
        self.hv_map_relay_aux_spin.setEnabled(on and relay)
        self.hv_map_fb_servo_spin.setEnabled(on and self.hv_map_fb_pin_spin.value() >= 0)
        self._refresh_mapping_texts()

    def mapping_problems(self, params: Optional[dict] = None) -> tuple:
        """(錯誤清單, 警告清單)。錯誤會擋規劃；判準與規劃器同一支 build_mapping_setup（不另寫一套）。"""
        p = params if params is not None else self.get_params()
        if not p.get("hv_map_enabled"):
            return [], []
        from mission.swarm_coordinator import build_mapping_setup
        try:
            setup = build_mapping_setup(p)
        except ValueError as e:
            return [str(e)], []
        return [], list(setup["warnings"])

    def _refresh_mapping_texts(self, *_a) -> None:
        if not hasattr(self, "hv_map_warn_label"):
            return
        verified = self.hv_map_rate_verified_check.isChecked()
        self.hv_map_rate_status_label.setText("已台架驗證" if verified else "未經台架驗證")
        self.hv_map_rate_status_label.setStyleSheet("color:#2e7d32;" if verified else "color:#e65100;")
        if not self.hv_map_check.isChecked():
            self.hv_map_warn_label.setText("")
            return
        errs, warns = self.mapping_problems({**self._mapping_params(True),
                                             "hv_wind": (self.hv_wind_speed_spin.value(), self.hv_wind_dir_spin.value())})
        lines = [f"<b style='color:#c62828;'>無法規劃：{e}</b>" for e in errs]
        lines += [f"<span style='color:#e65100;'>注意：{w}</span>" for w in warns]
        self.hv_map_warn_label.setText("<br>".join(lines))

    def set_mapping_plan_status(self, text: str, ok: bool = True) -> None:
        """規劃後由 MainWindow 回填：建圖有效值與上傳閘門結果。"""
        self.hv_map_status_label.setText(text)
        self.hv_map_status_label.setStyleSheet("color:#2e7d32;" if ok else "color:#c62828;")

    def set_mapping_live_status(self, text: str) -> None:
        """上傳後由 MainWindow 回填：每架「已拍 N / 預估 M」（CAMERA_FEEDBACK）。"""
        self.hv_map_live_label.setText(text)

    def apply_camera_values(self, values: dict, *, enable_mapping: bool) -> dict:
        """「相機配置」對話框套用：焦距／像素／解析度／前向與側向重疊填進來（下拉改成「自訂」）。

        任何一個值超出欄位範圍 ⇒ ValueError（不讓 spinbox 默默夾值）。``enable_mapping`` ⇒ 打開建圖模式
        （必要時把規劃演算法切到 HARVEST——建圖只在 HARVEST 有）。回傳**欄位讀回**的有效值。
        """
        spec = (("focal_mm", self.hv_map_focal_spin, "焦距"), ("pixel_um", self.hv_map_pixel_spin, "像素間距"),
                ("width_px", self.hv_map_width_spin, "影像寬"), ("height_px", self.hv_map_height_spin, "影像高"),
                ("front_overlap", self.hv_map_fwd_spin, "前向重疊"),
                ("side_overlap", self.overlap_spin, "側向重疊（重疊率）"))
        bad = []
        for key, w, label in spec:
            v = values.get(key)
            if v is None or not (w.minimum() <= float(v) <= w.maximum()):
                bad.append(f"{label} {v!r} 超出面板範圍 {w.minimum():g}–{w.maximum():g}")
        if bad:
            raise ValueError("；".join(bad))
        switched = False
        if enable_mapping and self.algo_combo.currentData() != 'hv_dccpp':
            self.algo_combo.setCurrentIndex(self.algo_combo.findData('hv_dccpp'))
            switched = True
        self._map_applying_preset = True
        try:
            blocked = self.hv_map_preset_combo.blockSignals(True)
            self.hv_map_preset_combo.setCurrentIndex(self.hv_map_preset_combo.findData(MAPPING_PRESET_CUSTOM))
            self.hv_map_preset_combo.blockSignals(blocked)
            self.hv_map_focal_spin.setValue(float(values["focal_mm"]))
            self.hv_map_pixel_spin.setValue(float(values["pixel_um"]))
            self.hv_map_width_spin.setValue(int(values["width_px"]))
            self.hv_map_height_spin.setValue(int(values["height_px"]))
            self.hv_map_fwd_spin.setValue(float(values["front_overlap"]))
            self.overlap_spin.setValue(float(values["side_overlap"]))
            # 換了相機 ⇒ 舊的「已台架驗證」不算數（對話框沒有觸發率，沿用欄位上的數字）
            self.hv_map_rate_verified_check.setChecked(False)
        finally:
            self._map_applying_preset = False
        if enable_mapping:
            self.hv_map_check.setChecked(True)
        self._sync_mapping_mirror()
        self._refresh_mapping_texts()
        eff = {"焦距 mm": self.hv_map_focal_spin.value(), "像素 µm": self.hv_map_pixel_spin.value(),
               "影像寬 px": self.hv_map_width_spin.value(), "影像高 px": self.hv_map_height_spin.value(),
               "前向重疊": self.hv_map_fwd_spin.value(), "側向重疊": self.overlap_spin.value(),
               "實測最大觸發率": f"{self.hv_map_rate_spin.value():g} Hz（沿用欄位值，未經台架驗證）",
               "建圖模式": self.hv_map_check.isChecked()}
        if switched:
            eff["規劃演算法"] = "已切到 HARVEST（建圖只在 HARVEST 有）"
        logger.info(f"[VTOL 偵察] 相機配置套用到航測建圖：{eff}")
        return eff

    def _mapping_params(self, is_hv: bool) -> dict:
        """``hv_map_*`` 鍵（get_params 的一部分）。用不到的欄位送 None，不送一個會被默默忽略的值。"""
        backend = self.hv_map_backend_combo.currentData()
        relay = backend == "relay"
        rp, aux = self.hv_map_relay_pin_spin.value(), self.hv_map_relay_aux_spin.value()
        fbp, fbs = self.hv_map_fb_pin_spin.value(), self.hv_map_fb_servo_spin.value()
        readout = self.hv_map_readout_spin.value()
        tol = self.hv_map_retrig_spin.value()
        return {
            "hv_map_enabled": bool(is_hv and self.hv_map_check.isChecked()),
            "hv_map_camera_preset": self.hv_map_preset_combo.currentData(),
            "hv_map_focal_mm": float(self.hv_map_focal_spin.value()),
            "hv_map_pixel_um": float(self.hv_map_pixel_spin.value()),
            "hv_map_width_px": int(self.hv_map_width_spin.value()),
            "hv_map_height_px": int(self.hv_map_height_spin.value()),
            "hv_map_trigger_hz": float(self.hv_map_rate_spin.value()),
            "hv_map_trigger_verified": bool(self.hv_map_rate_verified_check.isChecked()),
            "hv_map_readout_ms": (float(readout) if readout > 0.0 else None),
            "hv_map_forward_overlap": float(self.hv_map_fwd_spin.value()),
            # 側向重疊就是上面的「重疊率」（同一個欄位，不另開一格）
            "hv_map_side_overlap": float(self.overlap_spin.value()),
            "hv_map_backend": backend,
            "hv_map_firmware": (self.hv_map_firmware_edit.text().strip() or None),
            "hv_map_relay_pin": (int(rp) if relay and rp >= 0 else None),
            "hv_map_relay_aux_servo": (int(aux) if relay and aux > 0 else None),
            "hv_map_feedback_pin": int(fbp),
            "hv_map_feedback_servo": (int(fbs) if fbp >= 0 and fbs > 0 else None),
            "hv_map_max_roll_deg": float(self.hv_map_roll_spin.value()),
            "hv_map_lead_in_m": float(self.hv_map_lead_spin.value()),
            "hv_map_terrain_retrigger_tol": (float(tol) if tol > 0.0 else None),
            "hv_map_tlog": bool(self.hv_map_tlog_check.isChecked()),
            # 交叉航帶：沒勾 ⇒ 比例／高度差送 None（build_mapping_setup 對「沒勾卻給值」會拒絕）
            "hv_map_cross_enabled": bool(self.hv_map_cross_check.isChecked()),
            "hv_map_cross_fraction": (float(self.hv_map_cross_frac_spin.value())
                                      if self.hv_map_cross_check.isChecked() else None),
            "hv_map_cross_alt_offset_m": (float(self.hv_map_cross_dh_spin.value())
                                          if self.hv_map_cross_check.isChecked() else None),
            # 建圖作業風速上限（規劃風速＝上方「平均風速」超過 ⇒ 不可上傳；指現場實際風速，起飛前實測）
            "hv_map_max_wind_mps": float(self.hv_map_max_wind_spin.value()),
        }

    def _on_algo_changed(self) -> None:
        """切換規劃演算法：顯示對應欄位，按鈕文字也跟著改。"""
        is_hv = self.algo_combo.currentData() == 'hv_dccpp'
        self.hv_group.setVisible(is_hv)
        # 建圖只在 HARVEST 有：切到 DCCPP 就關掉（同時把感測器足跡還原、解除唯讀），免得畫面上
        # 留著一組被鎖住、而且這個演算法根本不讀的鏡子值。
        if not is_hv and getattr(self, "hv_map_check", None) is not None and self.hv_map_check.isChecked():
            self.hv_map_check.setChecked(False)
            logger.info("[VTOL 偵察] 切到 DCCPP：航測建圖只在 HARVEST 有，已關閉")
        self.btn_plan.setText(
            "執行 HARVEST 偵察規劃" if is_hv else "執行 DCCPP 偵察規劃")
        # HV 模式下總台數由 Pioneer + HERO 決定，把上面那格改成唯讀，
        # 免得同一個數字有兩個可以改的地方（改了哪一個算數？）。
        self.num_drones_spin.setEnabled(not is_hv)
        self.num_drones_spin.setToolTip(
            "HARVEST 模式下由「機隊組成」的 Pioneer ＋ HERO 台數決定。"
            if is_hv else "")
        self._on_hv_fleet_changed()

    def _on_hv_fleet_changed(self, *_a) -> None:
        """兩種構型的台數是總台數的唯一來源；同步上面的 VTOL 數量並更新提示。"""
        if not hasattr(self, 'hv_fleet_label'):
            return
        n_a = self.hv_num_a_spin.value()
        n_b = self.hv_num_b_spin.value()
        total = n_a + n_b
        # 只有 HV 模式才回寫，否則會蓋掉使用者在 DCCPP 模式下自己設的數量。
        # ⚠ 一定要 blockSignals：num_drones_spin 的 valueChanged 會再繞回來。
        if getattr(self, 'algo_combo', None) is not None \
                and self.algo_combo.currentData() == 'hv_dccpp':
            lo, hi = self.num_drones_spin.minimum(), self.num_drones_spin.maximum()
            if lo <= total <= hi:
                blocked = self.num_drones_spin.blockSignals(True)
                self.num_drones_spin.setValue(total)
                self.num_drones_spin.blockSignals(blocked)

        if total <= 0:
            self.hv_fleet_label.setText(
                "<b style='color:#e65100;'>機隊是空的</b> —— Pioneer 與 HERO 至少要有一台。")
            return
        parts = []
        if n_a:
            parts.append(f"<b>Pioneer×{n_a}</b> 巡航 21.5 m/s・轉彎 200 m・最低 120 m AGL")
        if n_b:
            parts.append(f"<b>HERO×{n_b}</b> 巡航 19.0 m/s・轉彎 120 m・最低 100 m AGL")
        warn = ""
        if n_a == 0 or n_b == 0:
            only = "HERO" if n_a == 0 else "Pioneer"
            warn = (f"<br><span style='color:#e65100;'>同構機隊（只有 {only}）——"
                    f"異構的好處在這個組合下看不出來。</span>")
        self.hv_fleet_label.setText(
            f"共 <b>{total}</b> 台　" + "　".join(parts) + warn)

    # ── ⑦ 覆蓋率驗算 ─────────────────────────────────────────
    def _build_coverage_group(self) -> QGroupBox:
        g = QGroupBox("6. 覆蓋率驗算")
        lay = QVBoxLayout(g)
        _tune_group_layout(lay)
        form = _tune_form(QFormLayout())

        self.cell_size_spin = QDoubleSpinBox()
        self.cell_size_spin.setRange(1.0, 200.0)
        self.cell_size_spin.setValue(20.0)
        self.cell_size_spin.setSuffix(" m")
        self.cell_size_spin.setToolTip("驗算格網解析度 — 越小越準也越慢")
        form.addRow("驗算格距:", self.cell_size_spin)
        lay.addLayout(form)

        row = QHBoxLayout()
        self.btn_verify = IconButton("confirm", "驗算覆蓋率", tone="primary", compact=True)
        self.btn_verify.clicked.connect(self._emit_verify)
        row.addWidget(self.btn_verify)
        self.btn_heatmap = IconButton("radar", "覆蓋熱力圖", tone="primary", compact=True)
        self.btn_heatmap.setToolTip(
            "用 SAR 熱力圖呈現覆蓋分布：紅 = 漏掃、藍 = 已充分掃過"
        )
        self.btn_heatmap.clicked.connect(self.show_heatmap_requested.emit)
        row.addWidget(self.btn_heatmap)
        self.btn_show_missed = IconButton("fov_cone", "標出漏掃", tone="ghost", compact=True)
        self.btn_show_missed.setToolTip("在 3D 圖台用紅點精確標出每個漏掃格心")
        self.btn_show_missed.clicked.connect(self.show_missed_requested.emit)
        row.addWidget(self.btn_show_missed)
        row.addStretch(1)
        lay.addLayout(row)

        self.coverage_label = QLabel("尚未驗算")
        self.coverage_label.setWordWrap(True)
        self.coverage_label.setTextFormat(Qt.TextFormat.RichText)
        lay.addWidget(self.coverage_label)
        return g

    # ── ⑧ 輸出 ───────────────────────────────────────────────
    def _build_fence_group(self) -> QGroupBox:
        """⑥ Fence 管理（NFZ / 威脅 / 圍籬）。

        VTOL 偵察也要能在**同一分頁**設定禁航區 / 威脅區 / 作業圍籬，不必切回
        基本演算法頁。按鈕發的是與 parameter_panel 同名的信號，接的是
        main_window 既有的同一組 handler，共用同一個 FenceZoneRegistry 單例
        —— 所以在這裡設的區域，DCCPP 規劃繞行、Cesium 3D 呈現、上傳飛控
        全都吃得到，不是另做一套。
        """
        g = QGroupBox("8. Fence 管理（NFZ / 威脅 / 圍籬）")
        v = QVBoxLayout(g)
        _tune_group_layout(v)
        # ⚠ 這裡原本是 setSpacing(4)，會蓋掉 _tune_group_layout 設的間距
        #   （helper 先跑、這行後跑 ⇒ 後者贏）。Fence 有 5 顆按鈕排成一列列，
        #   間距比一般欄位窄一點合理，但 4 太擠。

        self.fence_zone_count_label = QLabel("目前 Fence 區域：0 個")
        self.fence_zone_count_label.setProperty("role", "caption")
        self.fence_zone_count_label.style().polish(self.fence_zone_count_label)
        v.addWidget(self.fence_zone_count_label)

        # ⚠ 專案裡有**兩套**圍籬，不要混為一談（2026-08-25 使用者回報
        #   「地圖上有 fence 但這裡是 0」就是被這件事絆到）：
        #     (1) 上面那行數的是 FenceZoneRegistry —— 手動建立的
        #         NFZ / 威脅 / 圍籬，由這個群組管理與上傳。
        #     (2) 下面這行是**規劃時自動建構**的電子圍籬（把整條航跡框起來的
        #         AABB 矩形），畫在 2D/3D 地圖上、由 SITL 分頁的「上傳圍籬」
        #         送進飛控 —— 它不在 registry 裡，所以 (1) 永遠不會把它算進去。
        #   兩者都顯示出來，面板才不會對使用者說謊。
        self.auto_fence_label = QLabel("自動圍籬：尚未規劃")
        self.auto_fence_label.setWordWrap(True)
        self.auto_fence_label.setProperty("role", "caption")
        self.auto_fence_label.style().polish(self.auto_fence_label)
        self.auto_fence_label.setToolTip(
            "規劃路徑後系統自動把整條航跡框成矩形電子圍籬，"
            "顯示在地圖上。它與上面的手動 Fence 區域是兩套獨立機制："
            "自動圍籬由 SITL 分頁的「上傳圍籬」送進飛控。")
        v.addWidget(self.auto_fence_label)

        self.btn_fence_new = IconButton(
            "polygon", "＋ 新增 Fence（手動輸入）...", tone="warn", compact=True)
        self.btn_fence_new.setToolTip("開統一對話框，手動輸入頂點 / 圓心，選分類 + 海拔")
        self.btn_fence_new.clicked.connect(self.fence_zone_create_requested.emit)
        v.addWidget(self.btn_fence_new)

        draw_row = QHBoxLayout()
        self.btn_fence_poly = IconButton(
            "polygon", "地圖繪多邊形", tone="warn", compact=True)
        self.btn_fence_poly.setToolTip("在地圖上點頂點、雙擊完成，之後彈對話框選分類/海拔")
        self.btn_fence_poly.clicked.connect(self.fence_zone_draw_polygon_requested.emit)
        draw_row.addWidget(self.btn_fence_poly)
        self.btn_fence_circle = IconButton(
            "circle", "地圖拖圓形", tone="warn", compact=True)
        self.btn_fence_circle.setToolTip("在地圖上按住拖曳定義圓心半徑")
        self.btn_fence_circle.clicked.connect(self.fence_zone_draw_circle_requested.emit)
        draw_row.addWidget(self.btn_fence_circle)
        v.addLayout(draw_row)

        op_row = QHBoxLayout()
        self.btn_fence_upload = IconButton(
            "upload", "上傳至飛控", tone="success", compact=True)
        self.btn_fence_upload.setToolTip(
            "把所有 Fence 以 ArduPilot 4.2+ Polygon Fence 協定上傳到各連線飛控")
        self.btn_fence_upload.clicked.connect(self.fence_zone_upload_requested.emit)
        op_row.addWidget(self.btn_fence_upload)
        self.btn_fence_clear = IconButton(
            "clear", "全部清除", tone="danger", compact=True)
        self.btn_fence_clear.setToolTip("清除所有 fence 區域（視覺 + registry）")
        self.btn_fence_clear.clicked.connect(self.fence_zone_clear_all_requested.emit)
        op_row.addWidget(self.btn_fence_clear)
        v.addLayout(op_row)

        return g

    def update_fence_status(self, bundle) -> None:
        """顯示**自動建構**的電子圍籬狀態（與 parameter_panel 同名同簽名，
        讓 main_window 的 ``fence_built`` 信號可以同時餵兩個面板）。

        bundle: mission.geofence_manager.MissionBundle 或 None。
        """
        lbl = getattr(self, "auto_fence_label", None)
        if lbl is None:
            return
        if bundle is None:
            lbl.setText("自動圍籬：尚未規劃")
            lbl.setProperty("role", "caption")
        else:
            g = bundle.geofence
            lbl.setText(
                f"自動圍籬：✓ {len(g.vertices)} 頂點 {g.method.upper()} 矩形 ｜ "
                f"alt [{g.alt_min_m:.0f}, {g.alt_max_m:.0f}] m ｜ "
                f"buffer {g.buffer_radius_m:.0f} m（地圖上那個；由 SITL 分頁上傳）"
            )
            lbl.setProperty("role", "friendly")
        lbl.style().unpolish(lbl)
        lbl.style().polish(lbl)

    def update_fence_zone_count(self, n: int, nfz_count: int = -1) -> None:
        """由 main_window 的 FenceZoneRegistry 回呼更新計數（與 parameter_panel
        同簽名，這樣 main_window 可以對兩個面板一視同仁）。"""
        if hasattr(self, "fence_zone_count_label"):
            self.fence_zone_count_label.setText(f"目前 Fence 區域：{n} 個")

    def _build_output_group(self) -> QGroupBox:
        g = QGroupBox("7. 輸出 / 模擬")
        lay = QVBoxLayout(g)
        _tune_group_layout(lay)

        row = QHBoxLayout()
        self.btn_export = IconButton("export", "匯出航點", tone="primary", compact=True)
        self.btn_export.clicked.connect(self._emit_export)
        row.addWidget(self.btn_export)
        self.btn_upload = IconButton("upload", "上傳飛控", tone="ghost", compact=True)
        self.btn_upload.clicked.connect(self.upload_requested.emit)
        row.addWidget(self.btn_upload)
        row.addStretch(1)
        lay.addLayout(row)

        self.btn_launch = IconButton("launch", "一鍵起 VTOL SITL", tone="orange", compact=True)
        self.btn_launch.clicked.connect(
            lambda: self.launch_sitl_requested.emit(self.num_drones_spin.value())
        )
        lay.addWidget(self.btn_launch)
        return g

    # ══════════════════════════════════════════════════════════
    #  足跡即時換算
    # ══════════════════════════════════════════════════════════
    def _camera(self) -> CameraSpec:
        return CameraSpec(
            hfov_deg=self.hfov_spin.value(),
            vfov_deg=self.vfov_spin.value(),
            image_width_px=self.img_width_spin.value(),
            mount_angle_deg=self.mount_spin.value(),
        )

    def effective_altitude_m(self) -> float:
        """目前生效的偵察航高（依模式可能由目標 GSD 反算）。"""
        if self.rb_by_gsd.isChecked():
            return altitude_for_gsd_m(self._camera(), self.target_gsd_spin.value())
        return self.altitude_spin.value()

    # ══════════════════════════════════════════════════════════
    #  FOV 光錐：與 parameter_panel 同名同語意的取值介面
    #  ⚠ 刻意同名 —— main_window 用同一組呼叫處理兩個來源,
    #    少一個方法就會在切換來源時炸開,而不是靜靜吃到別頁的值。
    # ══════════════════════════════════════════════════════════
    def get_fov_visual_params(self) -> tuple:
        """(掛載角°, 水平 FOV°, 垂直 FOV°) —— 任一 > 0 即走梯形角錐 frustum。"""
        return (self.mount_spin.value(),
                self.hfov_spin.value(),
                self.vfov_spin.value())

    def get_fov_radius(self) -> float:
        """Legacy 圓錐半徑(公尺)。

        ⚠ 只有 HFOV/VFOV 都是 0 時才會被用到(見 cesium_map_widget.update_fov_cone)。
        本頁沒有「FOV 半徑」這個輸入,改用**感測器掃描寬度的一半**——
        那是同一個幾何量,而且會跟著航高與相機規格自動更新。
        """
        swath = getattr(self, "_raw_swath_width_m", 0.0) or 0.0
        return max(1.0, swath / 2.0)

    def is_fov_cone_enabled(self) -> bool:
        return self.fov_cone_check.isChecked()

    def _emit_fov_visual(self) -> None:
        self.fov_visual_changed.emit(*self.get_fov_visual_params())

    def set_fov_cone_status(self, text: str, ok: bool = True) -> None:
        """由 MainWindow 回填光錐現況（哪個來源在用、為什麼沒畫出來）。

        ⚠ 這個標籤是**唯一**能讓使用者看出「光錐吃的是哪一頁參數」的地方。
        兩頁共用同一個 3D 實體,沒有它就只能用猜的。
        """
        self.fov_cone_status.setText(text)
        self.fov_cone_status.setStyleSheet(
            "color:#2e7d32;" if ok else "color:#e65100;")

    def _refresh_footprint(self) -> None:
        """任何相機/航高參數變動就重算足跡摘要。"""
        by_gsd = self.rb_by_gsd.isChecked()
        self.altitude_spin.setEnabled(not by_gsd)
        self.target_gsd_spin.setEnabled(by_gsd)
        try:
            cam = self._camera()
            alt = self.effective_altitude_m()
            fp = compute_footprint(cam, alt)
            sweep = fp.swath_width_m * (1.0 - self.overlap_spin.value())
            self._sweep_width_m = sweep                    # 條帶間距（給規劃器）
            self._raw_swath_width_m = fp.swath_width_m     # 感測器掃描寬度（給驗算）
            self._footprint_ok = True
            self.footprint_label.setText(
                f"航高 <b>{alt:.1f} m</b> ｜ 掃描寬度 <b>{fp.swath_width_m:.1f} m</b> ｜ "
                f"有效條帶間距 <b>{sweep:.1f} m</b><br>"
                f"中心 GSD <b>{fp.gsd_m_per_px * 100:.2f} cm/px</b>"
                f"（邊緣最差 {fp.worst_gsd_m_per_px * 100:.2f}）｜ "
                f"足跡長度 {fp.along_track_m:.1f} m"
            )
            self.footprint_label.setStyleSheet("")
        except FootprintError as e:
            self._footprint_ok = False
            self._sweep_width_m = 0.0
            self._raw_swath_width_m = 0.0
            self.footprint_label.setText(f"相機幾何無解：{e}")
            self.footprint_label.setStyleSheet("color:#ff6b6b;")
        self._sync_enabled()

    # ══════════════════════════════════════════════════════════
    #  對外狀態更新（由 MainWindow 呼叫）
    # ══════════════════════════════════════════════════════════
    def set_area_count(self, n_committed: int, n_corners: int = 0) -> None:
        """更新「已完成幾塊區 / 正在畫幾個角點」。由 MainWindow 呼叫。"""
        self._committed_areas = int(n_committed)
        if n_committed <= 0:
            self.areas_label.setText(
                "多區模式：圈好一塊按「完成本區」就能再圈下一塊；只圈一塊時不必用。")
        else:
            cur = (f"，另有 {n_corners} 個角點正在畫" if n_corners else "")
            self.areas_label.setText(
                f"<b>已完成 {n_committed} 塊作業區</b>{cur}。按「開始規劃」會一起送出。")
        self._sync_enabled()

    def set_corner_count(self, n: int) -> None:
        """更新偵察區角點數。"""
        self._corner_count = int(n)
        if self._corner_count >= 3:
            self.area_label.setText(f"偵察區已劃定：{self._corner_count} 個角點")
        elif self._corner_count > 0:
            self.area_label.setText(
                f"已標 {self._corner_count} 點 —— 至少需要 3 點才能構成偵察區"
            )
        else:
            self.area_label.setText("尚未劃定偵察區")
        self._sync_enabled()

    def set_plan_result(self, rows: list, summary: str) -> None:
        """填入規劃結果摘要表。

        rows: ``[(uav_id, waypoint_count, distance_km, minutes[, 構型]), ...]``

        第 5 個元素（構型）是選用的：既有 DCCPP 是同構機隊，沒有這個概念，
        傳 4 元素即可，那一欄會顯示 ``—``。⚠ 新欄位一律**加在最後**，
        免得動到既有呼叫端與測試對前四欄的索引。
        """
        self.result_table.setRowCount(len(rows))
        for r, row in enumerate(rows):
            uid, wps, km, minutes = row[:4]
            cls = str(row[4]) if len(row) > 4 and row[4] else "—"
            for c, v in enumerate((f"UAV-{uid}", str(wps), f"{km:.2f}",
                                   f"{minutes:.1f}", cls)):
                self.result_table.setItem(r, c, QTableWidgetItem(v))
        self._has_plan = bool(rows)
        self.plan_status.setText(summary)
        # 新的規劃出來 ⇒ 舊的驗算結果作廢
        self._has_missed = False
        self._has_coverage = False
        self._sync_enabled()

    def set_coverage_result(self, report, warn: bool = False) -> None:
        """填入覆蓋率驗算結果（report 為 core.recon.CoverageReport）。"""
        pct = report.coverage_pct
        colour = "#51cf66" if pct >= 99.0 else ("#ffd43b" if pct >= 95.0 else "#ff6b6b")
        missed_n = len(report.missed_points)
        extra = ""
        if report.missed_truncated:
            extra = "（漏掃點已抽樣顯示）"
        self.coverage_label.setText(
            f"覆蓋率 <b style='color:{colour}'>{pct:.2f}%</b> "
            f"（{report.covered_cells}/{report.total_cells} 格，"
            f"格距 {report.cell_size_m:g} m）<br>"
            f"重掃比例 {report.redundant_ratio * 100:.1f}% ｜ "
            f"漏掃 {missed_n} 格{extra}"
        )
        self._has_missed = missed_n > 0
        self._has_coverage = True
        self._sync_enabled()

    def set_plan_status(self, text: str) -> None:
        self.plan_status.setText(text)

    # ══════════════════════════════════════════════════════════
    #  參數輸出
    # ══════════════════════════════════════════════════════════
    def get_params(self) -> dict:
        """組出送給 DCCPP 的參數；``vehicle_type`` 釘死為 ``'vtol'``。

        ``fov_width_m`` 帶的是**有效條帶間距**（已扣重疊），
        因為 DCCPP 是拿它當條帶間距用的。
        """
        alt = self.effective_altitude_m()
        is_hv = self.algo_combo.currentData() == 'hv_dccpp'
        n_a, n_b = self.hv_num_a_spin.value(), self.hv_num_b_spin.value()
        return {
            "vehicle_type": "vtol",          # 這一頁只規劃 VTOL，不從別處推斷
            # HV 模式的總台數**只**由兩個構型相加決定（上面那格是唯讀的鏡子）
            "num_drones": (n_a + n_b) if is_hv else self.num_drones_spin.value(),
            "fov_width_m": self._sweep_width_m,
            "overlap_rate": 0.0,             # 重疊已經吃進 fov_width_m，不可重複扣
            "altitude": alt,
            "speed": self.speed_spin.value(),
            "turn_radius": self.turn_radius_spin.value(),
            "auto_scan_angle": self.auto_angle_check.isChecked(),
            "scan_angle_deg": float(self.scan_angle_spin.value()),
            "enable_altitude": True,
            "auto_landing": self.auto_landing_check.isChecked(),
            "mounting_angle_deg": self.mount_spin.value(),
            "hfov_deg": self.hfov_spin.value(),
            "vfov_deg": self.vfov_spin.value(),
            "coordination_mode": self.coord_combo.currentData(),
            "collision_avoidance": self.collision_check.isChecked(),
            "min_separation_m": self.min_sep_spin.value(),
            # ⚠ 以前沒有這兩行 ⇒ 規劃器吃寫死預設 alt_offset_m=30(< 間距 50)
            "alt_offset_m": self.alt_offset_spin.value(),
            # 0 ⇒ None ⇒ 交給演算法自動決定(見 CollisionAvoidance._effective_cap)
            "max_alt_offset_m": (None if self.max_alt_offset_spin.value() <= 0.0
                                 else self.max_alt_offset_spin.value()),
            # VTOL 專屬
            "transition_alt": self.transition_alt_spin.value(),
            # 足跡衍生量，供結果面板與驗算共用
            "raw_swath_width_m": self._raw_swath_width_m,
            "sweep_width_m": self._sweep_width_m,
            "target_gsd": self.target_gsd_spin.value(),
            # ── 規劃演算法（見 _build_plan_group 的下拉）────────────────
            #   'dccpp'    → 既有實作，下面的 hv_* 全部不會被讀
            #   'hv_dccpp' → 本專案的異構雙構型版本
            "planner": self.algo_combo.currentData(),
            "hv_dccpp": is_hv,
            "hv_num_a": n_a,
            "hv_num_b": n_b,
            # None ＝ 沒有豁免 ⇒ 規劃器用民航法規的 120 m 當 AGL 上限。
            "hv_waiver_max_agl_m": (self.hv_waiver_spin.value()
                                    if self.hv_waiver_check.isChecked() else None),
            # 最低固定翼作業高度（表 1 的 120／100 是規劃值，不是機體極限）
            "hv_min_fw_agl_m": {"A": self.hv_minfw_a_spin.value(),
                                "B": self.hv_minfw_b_spin.value()},
            "hv_dh_band_min_m": self.hv_band_spin.value(),
            "hv_c2_range_km": self.hv_c2_spin.value(),
            # 0 = 自動（max(36, 3σ+30)）→ 必須送 None，送 0.0 會變成「淨空 0 公尺」
            "hv_terrain_clearance_m": (self.hv_clearance_spin.value()
                                       if self.hv_clearance_spin.value() > 0.0 else None),
            "hv_wind": (self.hv_wind_speed_spin.value(), self.hv_wind_dir_spin.value()),
            "hv_reduce_waypoints": True,
            # Cesium World Terrain（沒有 token 時勾選框停用 ⇒ False）
            "hv_world_terrain": bool(self.hv_world_terrain_chk.isChecked()
                                     and self.hv_world_terrain_chk.isEnabled()),
            # 航測建圖（hv_map_*）：hv_map_enabled=False 時規劃器一律不讀其餘鍵
            **self._mapping_params(is_hv),
        }

    # ══════════════════════════════════════════════════════════
    #  內部
    # ══════════════════════════════════════════════════════════
    def _sync_enabled(self) -> None:
        # 多區模式：已經完成的區也算數，不必再多圈一塊才能規劃
        _has_area = (self._corner_count >= 3
                     or getattr(self, "_committed_areas", 0) >= 1)
        ready = _has_area and getattr(self, "_footprint_ok", False)
        self.btn_plan.setEnabled(ready)
        _ff = getattr(self, "btn_find_flat", None)             # 面板建構中途也會走到這裡
        if _ff is not None and not getattr(self, "_flat_busy", False):
            _ff.setEnabled(bool(_has_area and getattr(self, "_flat_token_ok", False)))
        self.btn_verify.setEnabled(self._has_plan)
        # 熱力圖只要驗算過就能看（即使 100% 覆蓋，也能看出分布）；
        # 紅點標示則要真的有漏掃才有東西可標。
        self.btn_heatmap.setEnabled(self._has_coverage)
        self.btn_show_missed.setEnabled(self._has_missed)
        self.btn_export.setEnabled(self._has_plan)
        self.btn_upload.setEnabled(self._has_plan)

    # ── 找平坦位置 ──────────────────────────────────────────
    def get_flat_request(self) -> dict:
        """搜尋參數（作業區與 token 由 MainWindow 補）。``ceil_m``：有勾豁免就用豁免值，否則法規 120 m。"""
        p = self.get_params()
        wv = p.get("hv_waiver_max_agl_m")
        return {"radius_km": float(self.hv_flat_radius_spin.value()),
                "scale": float(self.hv_flat_scale_spin.value()) / 100.0,
                "min_fw_agl_m": dict(p["hv_min_fw_agl_m"]),
                "dh_band_min_m": float(p["hv_dh_band_min_m"]),
                "clearance_m": float(p["hv_terrain_clearance_m"] or 36.0),
                "ceil_m": float(wv) if wv else 120.0}

    def _emit_find_flat(self) -> None:
        self.find_flat_requested.emit(self.get_flat_request())

    def set_flat_busy(self, busy: bool, text: str = "") -> None:
        self._flat_busy = bool(busy)
        self.btn_find_flat.setEnabled(False if busy else bool(
            (self._corner_count >= 3 or getattr(self, "_committed_areas", 0) >= 1) and self._flat_token_ok))
        self.flat_status_label.setText(text)

    def show_flat_result(self, payload: dict, *, can_undo: bool = False) -> None:
        """結果對話框（非模態）。已開著的舊對話框先關掉。"""
        from ui.widgets.flat_candidates_dialog import FlatCandidatesDialog, summarize
        self.set_flat_busy(False, summarize(payload).split("\n")[-1])
        if self._flat_dialog is not None:
            try:
                self._flat_dialog.close()
            except RuntimeError:
                pass
        dlg = FlatCandidatesDialog(payload, self, can_undo=can_undo)
        dlg.apply_requested.connect(self.flat_apply_requested.emit)
        dlg.undo_requested.connect(self.flat_undo_requested.emit)
        self._flat_dialog = dlg
        dlg.show()

    def set_flat_can_undo(self, v: bool) -> None:
        if self._flat_dialog is not None:
            try:
                self._flat_dialog.set_can_undo(v)
            except RuntimeError:
                self._flat_dialog = None

    def _emit_plan(self) -> None:
        params = self.get_params()
        # 空機隊要當場說，不要送下去讓規劃器在幾秒後回一個看不懂的錯誤
        if int(params.get("num_drones", 0)) <= 0:
            self.plan_status.setText("機隊是空的：Pioneer 與 HERO 至少要有一台")
            return
        if params.get("hv_map_enabled"):
            # 建圖設定不合法（例如 relay 沒填韌體版本）要當場說，不送去背景規劃再失敗
            errs, warns = self.mapping_problems(params)
            if errs:
                self.plan_status.setText("航測建圖設定有誤：" + "；".join(errs))
                logger.warning(f"[VTOL 偵察] 航測建圖設定有誤，不規劃：{errs}")
                return
            logger.info("[VTOL 偵察] 航測建圖有效值：" + "、".join(
                f"{k}={params[k]!r}" for k in sorted(params) if k.startswith("hv_map_")))
            for w in warns:
                logger.warning(f"[VTOL 偵察] 航測建圖：{w}")
        self.plan_status.setText("規劃中 …")
        self.plan_requested.emit(params)

    def _emit_verify(self) -> None:
        # 兩個寬度不能混用：
        #   sweep_width_m = 條帶間距（已扣重疊）→ 規劃器排條帶用
        #   swath_width_m = 感測器實際掃描寬度   → 驗算覆蓋率用
        self.verify_coverage_requested.emit({
            "cell_size_m": self.cell_size_spin.value(),
            "sweep_width_m": self._sweep_width_m,
            "swath_width_m": self._raw_swath_width_m,
        })

    def _emit_export(self) -> None:
        self.export_requested.emit({
            "transition_alt": self.transition_alt_spin.value(),
        })
