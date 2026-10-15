"""航測後處理對話框：每架的飛控 .BIN ＋ Pi 會期（＋相機模組 F9P／PPK）→ I1、geo.txt、加標籤影像、QA、NodeODM，
以及 GCP（gcp_list.txt）送 ODM、檢核點精度報告（ASPRS 第 2 版的報告方式）。

``SurveyProcessDialog(parent=None, map_widget=None, polygons=None, gcp_blocks=None)``；main_window 以延遲匯入開啟。
``gcp_blocks``：給「GCP／檢核點規劃…」用的 UAV 區塊提供者（見 ui.dialogs.gcp_plan_dialog）。
處理跑在背景執行緒（ui.workers.survey_worker），對話框只收信號、填表、畫地圖。

``polygons``：視數用的測區。None = 舊行為（地圖上正在畫的角點 ``map_widget.corners``）；給了就**優先**用它、
不退回地圖角點 —— 接受 ``MainWindow.survey_area_polygons()`` 的回傳值（ring 的串列 [[(lat, lon), …], …]：
已完成的區＋正在畫且 ≥3 角點的區），或回傳它的可呼叫物件（建議傳 ``self.survey_area_polygons``：
對話框是非模態的，每次按「開始處理」時才取最新的作業區）。

有效值契約：
* SensorTimestamp 語意（台架 B3 未定案）沒有預設，不選就不能跑。
* 大地起伏 N 空白 = 未知（正高留空），絕不當 0。
* GPS 接收延遲（ms）空白 = 0（不補償），有效值與來源（空白／操作員輸入／取自飛控 GPS_DELAY_MS）顯示在欄位下、
  寫進 qa.json 的 effective.input_sources；從日誌帶入只在各架一致、> 0、飛行中沒改過時才帶。
* PPK 的基站框架名稱必填；框架不是 WGS84 時 geo.txt／EXIF 要明確給經緯度座標系，否則不寫（處理結果列為錯誤）。
* QA 未通過時「送 ODM」停用；要送必須勾「QA 未通過仍送出」，這個選擇寫進任務紀錄（odm/submission.json）。
* 建圖風速上限（預設 3 m/s，使用者 2026-10-15 的首飛決定）：超過只標示（QA 表與摘要的 ⚠），不擋送 ODM。
* gcp_list.txt：geo_z 的高程基準沒有預設 —— 必須明確選，而且要與 geo.txt 的高度相同；送出前逐行核對
  （樣板佔位、影像檔名、像素範圍、檢核點不可混入），核對摘要寫進任務紀錄。
* 檢核點報告：DSM 的高程基準要明確宣告（處理完成後預填「geo.txt 用的高度」並標明來源），與檢核點不同時
  除非給大地起伏 N，否則拒絕。
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (QAbstractItemView, QButtonGroup, QCheckBox, QComboBox, QDialog, QDoubleSpinBox,
                             QFileDialog, QFormLayout, QGroupBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit,
                             QPlainTextEdit, QProgressBar, QPushButton, QRadioButton, QTabWidget, QTableWidget,
                             QTableWidgetItem, QVBoxLayout, QWidget)

from utils.logger import get_logger

logger = get_logger()

SORTIE_HEADERS = ["呼號", "飛控 .BIN", "Pi 會期資料夾", "移動站 UBX（自動）", "既有 .pos（選填）"]
COL_CALLSIGN, COL_BIN, COL_SESSION, COL_UBX, COL_POS = range(5)
QA_HEADERS = ["機", "項目", "數值", "結果"]
DEFAULT_ODM_URL = "http://127.0.0.1:3000"
PHOTO_POSE_LAYER = "survey_photo_poses"
ORTHO_LAYER = "survey_ortho"

B3_TEXT = ("SensorTimestamp 的意義還沒有定案（台架 B3）：libcamera 文件說是第一列「開始曝光」的時刻，"
           "picamera2 手冊說是第一個像素「讀出」的時刻。兩者差一個曝光時間（20 m/s 下 1 ms ≈ 2 cm），"
           "所以沒有預設值 —— 必須依台架結果明確選一個；選錯時所有照片會沿航跡偏移同一個距離。")
GPS_LAG_TEXT = ("飛控 GPS 紀錄的時刻是「收到並處理」的時刻，比量測 epoch 晚一個接收機＋驅動延遲；不補償時，"
                "只用飛控內插（fc_interp）的照片 GPST 與 EXIF 時間會整體偏早這麼多（R2 實測 80 ms）。"
                "PPK 照片用相機模組的 TIM-TM2 時刻，不受影響。")
GPS_LAG_MAX_MS = 1000.0
GPS_LAG_BLANK_SRC = "空白＝0（不補償接收機延遲）"
GPS_LAG_USER_SRC = "操作員輸入"
POLY_MAIN_SRC = "主視窗的作業區（已完成的區＋正在畫且 ≥3 角點的區）"
POLY_MAP_SRC = "地圖上正在畫的測區角點"
GCP_VERTICAL_ITEMS = (("（請選 gcp_list.txt 的 geo_z 高程基準）", None), ("橢球高", "ellipsoidal"),
                      ("正高", "orthometric"))
DSM_VERTICAL_ITEMS = (("（請選 DSM 的高程基準）", None), ("橢球高", "ellipsoidal"), ("EGM96 正高", "EGM96"),
                      ("TWVD2001 正高", "TWVD2001"))
WIND_TEXT = ("使用者決定（2026-10-15）：首次建圖飛行只在風速 ≤ 3 m/s。處理時用飛控 EKF 的風估計（XKF2／NKF2）"
             "逐帶檢查；超過只標示（QA 表與摘要的 ⚠），不擋送 ODM。")
GPS_DELAY_NAMES = ("GPS_DELAY_MS", "GPS1_DELAY_MS")        # 4.6 起改名 GPS1_DELAY_MS


def read_log_params(path) -> Tuple[Dict[str, float], set]:
    """讀 .BIN 的參數：(第一次出現的值, 飛行中改過的參數名)。只掃 PARM／MSG／VER（survey.dflog）。"""
    from survey import dflog
    log = dflog.read_dflog(path, types=[])
    return dict(log.params), {n for _t, n, _v in log.param_changes}


def _float_or_none(text: str, what: str, errors: List[str]) -> Optional[float]:
    s = (text or "").strip()
    if not s:
        return None
    try:
        v = float(s)
    except ValueError:
        errors.append(f"{what} 不是數字：{s!r}")
        return None
    if not math.isfinite(v):
        errors.append(f"{what} 不是有限數值：{s!r}")
        return None
    return v


class SurveyProcessDialog(QDialog):
    def __init__(self, parent=None, map_widget=None, polygons=None, gcp_blocks=None):
        super().__init__(parent)
        self.setWindowTitle("航測後處理（照片位姿・PPK・ODM）")
        self.setMinimumWidth(900)
        self.map_widget = map_widget
        # 測區：None = 地圖角點；ring 的串列或回傳它的可呼叫物件（MainWindow.survey_area_polygons）優先
        self.polygons_source = polygons
        self.gcp_blocks_source = gcp_blocks          # 「GCP／檢核點規劃…」用的 UAV 區塊提供者
        self.checkpoint_report = None
        self._dsm_vertical_source = None
        self._gcp_dialog = None
        # 可替換的處理函式（測試用假的流水線／假 NodeODM／假的日誌參數讀取）；None = 正式實作
        self.process_fn = None
        self.odm_run_fn = None
        self.odm_client = None
        self.param_reader = None
        self.result = None
        self.odm_result = None
        self._worker = None
        self._odm_worker = None
        self._res_from_user = False
        self._gps_lag_source = None          # None = 空白或操作員輸入；從日誌帶入時是來源說明
        self._build_ui()
        self._refresh_rtklib_status()
        self._refresh_readout()
        self._refresh_gps_lag_label()
        self._refresh_polygon_label()
        self._update_buttons()

    # ─────────────────────────────────────────────────────────────────
    # UI
    # ─────────────────────────────────────────────────────────────────
    def _build_ui(self):
        lay = QVBoxLayout(self)
        self.tabs = QTabWidget()
        lay.addWidget(self.tabs)
        self.tabs.addTab(self._build_inputs_tab(), "架次與時間")
        self.tabs.addTab(self._build_ppk_tab(), "PPK")
        self.tabs.addTab(self._build_output_tab(), "輸出與 ODM")
        self.tabs.addTab(self._build_results_tab(), "結果")
        self.tabs.addTab(self._build_checkpoint_tab(), "檢核點精度")

        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 1000)
        self.progress_bar.setValue(0)
        self.lbl_progress = QLabel("")
        self.lbl_errors = QLabel("")
        self.lbl_errors.setWordWrap(True)
        self.lbl_errors.setStyleSheet("color:#E53935;")
        lay.addWidget(self.lbl_progress)
        lay.addWidget(self.progress_bar)
        lay.addWidget(self.lbl_errors)

        row = QHBoxLayout()
        self.btn_run = QPushButton("開始處理")
        self.btn_cancel = QPushButton("取消")
        self.btn_close = QPushButton("關閉")
        self.btn_run.clicked.connect(self.start_processing)
        self.btn_cancel.clicked.connect(self.cancel)
        self.btn_close.clicked.connect(self.close)
        row.addStretch(1)
        for b in (self.btn_run, self.btn_cancel, self.btn_close):
            row.addWidget(b)
        lay.addLayout(row)

    def _build_inputs_tab(self) -> QWidget:
        w = QWidget()
        v = QVBoxLayout(w)
        self.sortie_table = QTableWidget(0, len(SORTIE_HEADERS))
        self.sortie_table.setHorizontalHeaderLabels(SORTIE_HEADERS)
        self.sortie_table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.sortie_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.sortie_table.itemChanged.connect(self._on_sortie_item_changed)
        v.addWidget(QLabel("每架一列：飛控 SD 卡的 .BIN（落地後、飛控重開機之前取出）與同一架次的 Pi 會期資料夾。"))
        v.addWidget(self.sortie_table)
        row = QHBoxLayout()
        self.btn_add_row = QPushButton("新增架次")
        self.btn_remove_row = QPushButton("移除選取")
        self.btn_browse_bin = QPushButton("選 .BIN…")
        self.btn_browse_session = QPushButton("選會期資料夾…")
        self.btn_browse_pos = QPushButton("選 .pos…")
        self.btn_add_row.clicked.connect(lambda: self.add_sortie_row())
        self.btn_remove_row.clicked.connect(self._remove_selected_row)
        self.btn_browse_bin.clicked.connect(lambda: self._browse_cell(COL_BIN, "飛控日誌 (*.BIN *.bin)"))
        self.btn_browse_session.clicked.connect(lambda: self._browse_cell(COL_SESSION, None))
        self.btn_browse_pos.clicked.connect(lambda: self._browse_cell(COL_POS, "RTKLIB 軌跡 (*.pos)"))
        for b in (self.btn_add_row, self.btn_remove_row, self.btn_browse_bin, self.btn_browse_session,
                  self.btn_browse_pos):
            row.addWidget(b)
        row.addStretch(1)
        v.addLayout(row)

        grp = QGroupBox("SensorTimestamp 語意（必選）")
        g = QVBoxLayout(grp)
        lbl = QLabel(B3_TEXT)
        lbl.setWordWrap(True)
        g.addWidget(lbl)
        self.rb_exposure_start = QRadioButton("exposure_start：第一列開始曝光（曝光中心 = ts + E/2 + R/2）")
        self.rb_readout_start = QRadioButton("readout_start：第一個像素讀出（曝光中心 = ts − E/2 + R/2）")
        self.semantics_group = QButtonGroup(self)
        self.semantics_group.setExclusive(True)
        for rb in (self.rb_exposure_start, self.rb_readout_start):
            self.semantics_group.addButton(rb)
            g.addWidget(rb)
        v.addWidget(grp)

        grp_h = QGroupBox("高度與視數")
        f = QFormLayout(grp_h)
        self.cmb_height = QComboBox()
        self.cmb_height.addItem("橢球高（h_ell_m）", "ellipsoidal")
        self.cmb_height.addItem("正高（h_orth_m，需要大地起伏 N）", "orthometric")
        f.addRow("geo.txt／EXIF 高度", self.cmb_height)
        nrow = QHBoxLayout()
        self.ed_geoid_n = QLineEdit()
        self.ed_geoid_n.setPlaceholderText("空白 = 未知（正高留空，不會當 0）")
        self.cmb_geoid_datum = QComboBox()
        self.cmb_geoid_datum.addItems(["EGM96", "TWVD2001"])
        nrow.addWidget(self.ed_geoid_n, 1)
        nrow.addWidget(QLabel("正高基準"))
        nrow.addWidget(self.cmb_geoid_datum)
        f.addRow("大地起伏 N（m，h_ell − H）", nrow)
        self.ed_ground_h = QLineEdit()
        self.ed_ground_h.setPlaceholderText("空白 = 不算視數與 GSD；與上面所選高度同一基準")
        f.addRow("地面高（m）", self.ed_ground_h)
        self.chk_use_polygon = QCheckBox("以測區多邊形計算視數（否則在被拍到的範圍內算）")
        self.chk_use_polygon.toggled.connect(lambda _: self._refresh_polygon_label())
        f.addRow(self.chk_use_polygon)
        self.lbl_polygon_source = QLabel("")
        self.lbl_polygon_source.setWordWrap(True)
        f.addRow("測區來源", self.lbl_polygon_source)
        v.addWidget(grp_h)

        grp_t = QGroupBox("GPS 接收延遲（只影響飛控內插照片的 GPST／EXIF 時間）")
        tf = QFormLayout(grp_t)
        lbl = QLabel(GPS_LAG_TEXT)
        lbl.setWordWrap(True)
        tf.addRow(lbl)
        lrow = QHBoxLayout()
        self.ed_gps_lag = QLineEdit()
        self.ed_gps_lag.setPlaceholderText("空白 = 0 ms（不補償）")
        self.ed_gps_lag.textEdited.connect(self._on_gps_lag_edited)
        self.ed_gps_lag.textChanged.connect(lambda _: self._refresh_gps_lag_label())
        self.btn_gps_lag_from_log = QPushButton("從日誌讀 GPS_DELAY_MS")
        self.btn_gps_lag_from_log.setToolTip("讀每一列 .BIN 的 GPS_DELAY_MS／GPS1_DELAY_MS（EKF 的延遲模型值，不是實測）；"
                                             "各架一致且 > 0 才帶入")
        self.btn_gps_lag_from_log.clicked.connect(self.fill_gps_lag_from_logs)
        lrow.addWidget(self.ed_gps_lag, 1)
        lrow.addWidget(self.btn_gps_lag_from_log)
        tf.addRow("延遲（ms）", lrow)
        self.lbl_gps_lag = QLabel("")
        self.lbl_gps_lag.setWordWrap(True)
        tf.addRow(self.lbl_gps_lag)
        self.lbl_gps_lag_log = QLabel("")
        self.lbl_gps_lag_log.setWordWrap(True)
        tf.addRow(self.lbl_gps_lag_log)
        v.addWidget(grp_t)

        grp_w = QGroupBox("建圖風速上限（只標示，不擋）")
        wf = QFormLayout(grp_w)
        lbl = QLabel(WIND_TEXT)
        lbl.setWordWrap(True)
        wf.addRow(lbl)
        self.spn_wind_limit = QDoubleSpinBox()
        self.spn_wind_limit.setRange(0.5, 30.0)
        self.spn_wind_limit.setDecimals(1)
        self.spn_wind_limit.setSingleStep(0.5)
        self.spn_wind_limit.setSuffix(" m/s")
        self.spn_wind_limit.setValue(3.0)
        wf.addRow("上限", self.spn_wind_limit)
        v.addWidget(grp_w)
        return w

    def _build_ppk_tab(self) -> QWidget:
        w = QWidget()
        v = QVBoxLayout(w)
        self.grp_ppk = QGroupBox("PPK（公分級；相機模組 ZED-F9P 的 TIM-TM2 ＋ RTKLIB 軌跡）")
        self.grp_ppk.setCheckable(True)
        self.grp_ppk.setChecked(False)
        g = QVBoxLayout(self.grp_ppk)
        self.rb_ppk_rtklib = QRadioButton("用 RTKLIB 解算（rover.ubx ＋ 基站觀測）")
        self.rb_ppk_pos = QRadioButton("用架次表填的既有 .pos")
        self.rb_ppk_rtklib.setChecked(True)
        self.ppk_mode_group = QButtonGroup(self)
        for rb in (self.rb_ppk_rtklib, self.rb_ppk_pos):
            self.ppk_mode_group.addButton(rb)
            g.addWidget(rb)
        f = QFormLayout()
        self.ed_base_rinex = QLineEdit()
        self.ed_base_rinex.setPlaceholderText("基站 RINEX 觀測檔（與 UBX 二擇一）")
        self.ed_base_ubx = QLineEdit()
        self.ed_base_ubx.setPlaceholderText("或基站 UBX（先用 convbin 轉 RINEX）")
        f.addRow("基站 RINEX", self._with_browse(self.ed_base_rinex, "RINEX (*.obs *.*o *.rnx *.crx)"))
        f.addRow("基站 UBX", self._with_browse(self.ed_base_ubx, "UBX (*.ubx)"))
        self.ed_base_lat = QLineEdit()
        self.ed_base_lon = QLineEdit()
        self.ed_base_h = QLineEdit()
        self.ed_base_ant = QLineEdit()
        self.ed_base_ant.setPlaceholderText("量測值，必填")
        self.ed_base_frame = QLineEdit()
        self.ed_base_frame.setPlaceholderText("必填，例如 e-GNSS[2025]（會成為照片的 crs_h）")
        f.addRow("基站緯度（°）", self.ed_base_lat)
        f.addRow("基站經度（°）", self.ed_base_lon)
        f.addRow("基站橢球高（m，標石）", self.ed_base_h)
        f.addRow("天線高（m，ARP 高於標石）", self.ed_base_ant)
        f.addRow("座標框架名稱", self.ed_base_frame)
        self.ed_convbin = QLineEdit()
        self.ed_convbin.setPlaceholderText("空白 = 從 PATH 找")
        self.ed_rnx2rtkp = QLineEdit()
        self.ed_rnx2rtkp.setPlaceholderText("空白 = 從 PATH 找")
        f.addRow("convbin", self.ed_convbin)
        f.addRow("rnx2rtkp", self.ed_rnx2rtkp)
        rr = QHBoxLayout()
        self.btn_rtklib_detect = QPushButton("自動偵測 RTKLIB")
        self.btn_rtklib_detect.clicked.connect(self.detect_rtklib)
        self.lbl_rtklib = QLabel("")
        self.lbl_rtklib.setWordWrap(True)
        rr.addWidget(self.btn_rtklib_detect)
        rr.addWidget(self.lbl_rtklib, 1)
        f.addRow(rr)
        g.addLayout(f)
        v.addWidget(self.grp_ppk)

        grp_geo = QGroupBox("geo.txt／EXIF 的座標系")
        gf = QFormLayout(grp_geo)
        self.ed_geo_projection = QLineEdit()
        self.ed_geo_projection.setPlaceholderText("空白 = 只在全部照片都是 EPSG:4326 時寫出")
        lbl = QLabel("PPK 的位置落在基站框架（例如 e-GNSS[2025]），不是 WGS84。不會靜默當 WGS84 寫；要寫 geo.txt／EXIF "
                     "就明確給 PROJ 可解析的經緯度座標系（例如 +proj=longlat +ellps=GRS80 +no_defs），全部照片以它寫出。")
        lbl.setWordWrap(True)
        gf.addRow(lbl)
        gf.addRow("座標系", self.ed_geo_projection)
        v.addWidget(grp_geo)
        v.addStretch(1)
        return w

    def _build_output_tab(self) -> QWidget:
        w = QWidget()
        v = QVBoxLayout(w)
        f = QFormLayout()
        self.ed_out_dir = QLineEdit()
        f.addRow("輸出資料夾", self._with_browse(self.ed_out_dir, None))
        self.chk_overwrite = QCheckBox("輸出資料夾已有成果時覆寫")
        f.addRow(self.chk_overwrite)
        v.addLayout(f)

        grp = QGroupBox("NodeODM")
        of = QFormLayout(grp)
        self.ed_odm_url = QLineEdit(DEFAULT_ODM_URL)
        of.addRow("節點位址", self.ed_odm_url)
        self.spn_res = QDoubleSpinBox()
        self.spn_res.setRange(0.0, 1000.0)
        self.spn_res.setDecimals(1)
        self.spn_res.setSuffix(" cm")
        self.spn_res.setSpecialValueText("未設定（處理後以 GSD 帶入）")
        self.spn_res.valueChanged.connect(self._on_res_changed)
        of.addRow("正射解析度", self.spn_res)
        self.lbl_readout = QLabel("")
        self.lbl_readout.setWordWrap(True)
        of.addRow("滾動快門讀出", self.lbl_readout)
        self.chk_rolling = QCheckBox("開滾動快門校正（--rolling-shutter，讀出時間取自會期）")
        of.addRow(self.chk_rolling)
        self.chk_auto_submit = QCheckBox("處理完成且 QA 通過後自動送 ODM")
        of.addRow(self.chk_auto_submit)
        self.chk_qa_override = QCheckBox("QA 未通過仍送出（會寫進任務紀錄）")
        self.chk_qa_override.toggled.connect(lambda _: self._update_buttons())
        of.addRow(self.chk_qa_override)
        self.btn_odm = QPushButton("送 ODM")
        self.btn_odm.clicked.connect(self.submit_odm)
        of.addRow(self.btn_odm)
        v.addWidget(grp)

        grp_g = QGroupBox("GCP（地面控制點）")
        gf = QFormLayout(grp_g)
        self.btn_gcp_plan = QPushButton("GCP／檢核點規劃…")
        self.btn_gcp_plan.setToolTip("產生建議的 GCP 與檢核點位置（交叉航帶 vs 加密 GCP 兩方案）、匯出外業表與 gcp_list 樣板")
        self.btn_gcp_plan.clicked.connect(self.open_gcp_plan)
        gf.addRow(self.btn_gcp_plan)
        self.ed_gcp_txt = QLineEdit()
        self.ed_gcp_txt.setPlaceholderText("選填：已在 WebODM GCP 編輯器標好影像座標的 gcp_list.txt")
        self.ed_gcp_txt.textChanged.connect(lambda _: self._update_buttons())
        gf.addRow("gcp_list.txt", self._with_browse(self.ed_gcp_txt, "ODM GCP (*.txt)"))
        self.cmb_gcp_vertical = QComboBox()
        for text, data in GCP_VERTICAL_ITEMS:
            self.cmb_gcp_vertical.addItem(text, data)
        self.cmb_gcp_vertical.currentIndexChanged.connect(lambda _: self._update_buttons())
        gf.addRow("geo_z 高程基準", self.cmb_gcp_vertical)
        self.lbl_gcp = QLabel("沒有 GCP：只用影像的 GNSS 位置定位（公尺級，不能宣告車道級精度）")
        self.lbl_gcp.setWordWrap(True)
        gf.addRow(self.lbl_gcp)
        v.addWidget(grp_g)
        v.addStretch(1)
        return w

    def _build_checkpoint_tab(self) -> QWidget:
        w = QWidget()
        v = QVBoxLayout(w)
        lbl = QLabel("實測檢核點 vs 成果（正射上量的平面、DSM 取的高程）→ RMSE_H／RMSE_V、偏差、最大值，"
                     "與檢核點量測誤差合成（RSS）、逐 UAV 區塊與逐跨機接縫的高程階差。檢核點不能同時當 GCP。")
        lbl.setWordWrap(True)
        v.addWidget(lbl)
        f = QFormLayout()
        self.ed_chk_survey = QLineEdit()
        self.ed_chk_survey.setPlaceholderText("外業實測 CSV（id, lat, lon, h, frame, vertical_datum, sigma_h_m, sigma_v_m）")
        f.addRow("實測檢核點", self._with_browse(self.ed_chk_survey, "CSV (*.csv)"))
        self.ed_chk_measured = QLineEdit()
        self.ed_chk_measured.setPlaceholderText("選填：正射上量的 CSV（id, lat, lon, frame；可加 h, vertical_datum）")
        f.addRow("正射量測", self._with_browse(self.ed_chk_measured, "CSV (*.csv)"))
        self.ed_chk_layout = QLineEdit()
        self.ed_chk_layout.setPlaceholderText("選填：GCP／檢核點規劃匯出的 gcp_plan.csv（逐區塊、逐接縫）")
        f.addRow("佈設 CSV", self._with_browse(self.ed_chk_layout, "CSV (*.csv)"))
        self.ed_dsm = QLineEdit()
        self.ed_dsm.setPlaceholderText("選填：DSM GeoTIFF（ODM 完成後自動帶入 odm_dem/dsm.tif）")
        f.addRow("DSM", self._with_browse(self.ed_dsm, "GeoTIFF (*.tif *.tiff)"))
        self.cmb_dsm_vertical = QComboBox()
        for text, data in DSM_VERTICAL_ITEMS:
            self.cmb_dsm_vertical.addItem(text, data)
        self.cmb_dsm_vertical.activated.connect(lambda _: self._on_dsm_vertical_picked())
        f.addRow("DSM 高程基準", self.cmb_dsm_vertical)
        self.lbl_dsm_vertical = QLabel("")
        self.lbl_dsm_vertical.setWordWrap(True)
        f.addRow(self.lbl_dsm_vertical)
        self.ed_chk_undulation = QLineEdit()
        self.ed_chk_undulation.setPlaceholderText("空白 = 不換算（DSM 與檢核點基準不同就拒絕）；N = h_ell − H")
        f.addRow("大地起伏 N（m）", self.ed_chk_undulation)
        self.ed_chk_frame_crs = QLineEdit()
        self.ed_chk_frame_crs.setPlaceholderText("檢核點框架名不是 PROJ 座標系時必填，例如 +proj=longlat +ellps=GRS80 +no_defs")
        f.addRow("檢核點框架對應的座標系", self.ed_chk_frame_crs)
        trow = QHBoxLayout()
        self.spn_target_h = QDoubleSpinBox()
        self.spn_target_v = QDoubleSpinBox()
        for sp in (self.spn_target_h, self.spn_target_v):
            sp.setRange(0.0, 1000.0)
            sp.setDecimals(1)
            sp.setSuffix(" cm")
            sp.setSpecialValueText("未設定")
        trow.addWidget(QLabel("水平"))
        trow.addWidget(self.spn_target_h)
        trow.addWidget(QLabel("垂直"))
        trow.addWidget(self.spn_target_v)
        f.addRow("目標精度（RMSE）", trow)
        v.addLayout(f)
        self.btn_chk_report = QPushButton("計算檢核點精度")
        self.btn_chk_report.clicked.connect(lambda: self.compute_checkpoint_report())
        v.addWidget(self.btn_chk_report)
        self.chk_report_view = QPlainTextEdit()
        self.chk_report_view.setReadOnly(True)
        v.addWidget(self.chk_report_view, 1)
        return w

    def _build_results_tab(self) -> QWidget:
        w = QWidget()
        v = QVBoxLayout(w)
        self.lbl_summary = QLabel("尚未處理")
        self.lbl_summary.setWordWrap(True)
        v.addWidget(self.lbl_summary)
        self.qa_table = QTableWidget(0, len(QA_HEADERS))
        self.qa_table.setHorizontalHeaderLabels(QA_HEADERS)
        self.qa_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.qa_table.verticalHeader().setVisible(False)
        self.qa_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeMode.Stretch)
        v.addWidget(self.qa_table, 3)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        v.addWidget(self.log_view, 1)
        return w

    def _with_browse(self, edit: QLineEdit, file_filter: Optional[str]) -> QWidget:
        box = QWidget()
        h = QHBoxLayout(box)
        h.setContentsMargins(0, 0, 0, 0)
        h.addWidget(edit, 1)
        b = QPushButton("…")
        b.setFixedWidth(32)

        def pick():
            if file_filter is None:
                p = QFileDialog.getExistingDirectory(self, "選資料夾", edit.text())
            else:
                p, _ = QFileDialog.getOpenFileName(self, "選檔案", edit.text(), file_filter)
            if p:
                edit.setText(p)
        b.clicked.connect(pick)
        h.addWidget(b)
        return box

    # ─────────────────────────────────────────────────────────────────
    # 架次表
    # ─────────────────────────────────────────────────────────────────
    def _cell(self, row: int, col: int) -> str:
        it = self.sortie_table.item(row, col)
        return it.text().strip() if it is not None else ""

    def _set_cell(self, row: int, col: int, text: str) -> None:
        it = self.sortie_table.item(row, col)
        if it is None:
            self.sortie_table.setItem(row, col, QTableWidgetItem(text))
        else:
            it.setText(text)

    def add_sortie_row(self, callsign: str = "", bin_path="", session_dir="", pos_path="", rover_ubx="") -> int:
        r = self.sortie_table.rowCount()
        self.sortie_table.blockSignals(True)
        self.sortie_table.insertRow(r)
        for c, val in ((COL_CALLSIGN, callsign), (COL_BIN, bin_path), (COL_SESSION, session_dir),
                       (COL_UBX, rover_ubx), (COL_POS, pos_path)):
            self._set_cell(r, c, str(val or ""))
        self.sortie_table.blockSignals(False)
        if session_dir:
            self._autofill_from_session(r)
        return r

    def _remove_selected_row(self):
        rows = sorted({i.row() for i in self.sortie_table.selectedIndexes()}, reverse=True)
        for r in rows:
            self.sortie_table.removeRow(r)
        self._refresh_readout()

    def _browse_cell(self, col: int, file_filter: Optional[str]):
        r = self.sortie_table.currentRow()
        if r < 0:
            r = self.add_sortie_row()
        if file_filter is None:
            p = QFileDialog.getExistingDirectory(self, "選 Pi 會期資料夾", self._cell(r, col))
        else:
            p, _ = QFileDialog.getOpenFileName(self, "選檔案", self._cell(r, col), file_filter)
        if p:
            self._set_cell(r, col, p)

    def _on_sortie_item_changed(self, item):
        if item.column() == COL_SESSION:
            self._autofill_from_session(item.row())

    def _autofill_from_session(self, row: int) -> None:
        """會期資料夾一填：rover.ubx 自動偵測（gnss/rover.ubx），呼號空白時取 session.json 的 uav。"""
        sess = self._cell(row, COL_SESSION)
        if not sess:
            return
        self.sortie_table.blockSignals(True)
        try:
            ubx = Path(sess) / "gnss" / "rover.ubx"
            if not self._cell(row, COL_UBX) or not Path(self._cell(row, COL_UBX)).is_file():
                self._set_cell(row, COL_UBX, str(ubx) if ubx.is_file() else "")
            if not self._cell(row, COL_CALLSIGN):
                try:
                    uav = json.loads((Path(sess) / "session.json").read_text(encoding="utf-8")).get("uav")
                except (OSError, ValueError):
                    uav = None
                if uav:
                    self._set_cell(row, COL_CALLSIGN, str(uav))
        finally:
            self.sortie_table.blockSignals(False)
        self._refresh_readout()

    # ─────────────────────────────────────────────────────────────────
    # RTKLIB／讀出時間
    # ─────────────────────────────────────────────────────────────────
    def _rtklib_paths(self) -> Tuple[Optional[str], Optional[str], Optional[str]]:
        """(convbin, rnx2rtkp, 錯誤訊息)。只用 shutil.which／檔案存在判斷，不執行任何程式。"""
        from survey.ppk.rtklib import RtklibNotFound, find_executable
        try:
            cb = find_executable("convbin", self.ed_convbin.text().strip() or None)
            rk = find_executable("rnx2rtkp", self.ed_rnx2rtkp.text().strip() or None)
        except RtklibNotFound as e:
            return None, None, str(e)
        return cb, rk, None

    def _refresh_rtklib_status(self) -> None:
        cb, rk, err = self._rtklib_paths()
        if err:
            self.lbl_rtklib.setText(f"未安裝 RTKLIB：{err}")
            self.lbl_rtklib.setStyleSheet("color:#E53935;")
        else:
            self.lbl_rtklib.setText(f"RTKLIB：convbin = {cb}；rnx2rtkp = {rk}")
            self.lbl_rtklib.setStyleSheet("")

    def detect_rtklib(self) -> None:
        cb, rk, _ = self._rtklib_paths()
        if cb and not self.ed_convbin.text().strip():
            self.ed_convbin.setText(cb)
        if rk and not self.ed_rnx2rtkp.text().strip():
            self.ed_rnx2rtkp.setText(rk)
        self._refresh_rtklib_status()

    def _refresh_readout(self) -> None:
        from survey.odm_client import rolling_shutter_readout
        vals, n = [], 0
        for r in range(self.sortie_table.rowCount()):
            sess = self._cell(r, COL_SESSION)
            if not sess:
                continue
            try:
                cam = json.loads((Path(sess) / "session.json").read_text(encoding="utf-8"))["camera"]
            except (OSError, ValueError, KeyError):
                continue
            n += 1
            vals.append(cam.get("readout_ms"))
        if self.result is not None:
            ro, err = self.result.odm.get("rolling_shutter_readout_ms"), self.result.odm.get("rolling_shutter_error")
        elif n == 0:
            ro, err = None, "還沒有會期"
        else:
            try:
                ro, err = rolling_shutter_readout(vals), None
            except ValueError as e:
                ro, err = None, str(e)
        if ro is None:
            self.lbl_readout.setText(f"不能開：{err}")
            self.chk_rolling.setChecked(False)
            self.chk_rolling.setEnabled(False)
        else:
            self.lbl_readout.setText(f"{ro:.2f} ms（取自會期 session.json 的實測值）")
            if not self.chk_rolling.isEnabled():        # 剛變成可用才預設勾選；之後尊重操作員的選擇
                self.chk_rolling.setEnabled(True)
                self.chk_rolling.setChecked(True)

    # ─────────────────────────────────────────────────────────────────
    # 測區（視數）
    # ─────────────────────────────────────────────────────────────────
    def survey_rings(self) -> Tuple[Optional[List[List[Tuple[float, float]]]], str, Optional[str]]:
        """(ring 的串列, 來源說明, 錯誤)。給了 ``polygons_source`` 就只用它（不退回地圖角點）。"""
        if self.polygons_source is not None:
            try:
                src = self.polygons_source
                raw = src() if callable(src) else src
            except Exception as e:                   # noqa: BLE001 - 主視窗狀態異常要說清楚，不能當成沒有測區
                return None, POLY_MAIN_SRC, f"讀主視窗作業區失敗：{type(e).__name__}: {e}"
            rings = []
            for k, ring in enumerate(list(raw or []), start=1):
                try:
                    pts = [(float(a), float(b)) for a, b in ring]
                except (TypeError, ValueError):
                    return None, POLY_MAIN_SRC, f"第 {k} 塊作業區的角點不是（緯度, 經度）數字"
                if len(pts) < 3:
                    return None, POLY_MAIN_SRC, f"第 {k} 塊作業區只有 {len(pts)} 個角點（至少 3 個）"
                rings.append(pts)
            if not rings:
                return None, POLY_MAIN_SRC, "主視窗沒有任何作業區（已完成的區、或正在畫且 ≥3 角點的區），無法以測區計算視數"
            return rings, f"{POLY_MAIN_SRC}，{len(rings)} 塊", None
        corners = list(getattr(self.map_widget, "corners", None) or []) if self.map_widget is not None else []
        if len(corners) < 3:
            return None, POLY_MAP_SRC, "地圖上的測區角點不到 3 個，無法以測區計算視數"
        try:
            ring = [(float(a), float(b)) for a, b in corners]
        except (TypeError, ValueError):
            return None, POLY_MAP_SRC, "地圖上的測區角點不是（緯度, 經度）數字"
        return [ring], f"{POLY_MAP_SRC}（1 塊）", None

    def _refresh_polygon_label(self) -> None:
        if not self.chk_use_polygon.isChecked():
            self.lbl_polygon_source.setText("不用測區：在被拍到的範圍內算視數")
            self.lbl_polygon_source.setStyleSheet("")
            return
        rings, src, err = self.survey_rings()
        if err:
            self.lbl_polygon_source.setText(f"✘ {err}")
            self.lbl_polygon_source.setStyleSheet("color:#E53935;")
            return
        self.lbl_polygon_source.setText(f"{src}：{sum(len(r) for r in rings)} 個角點")
        self.lbl_polygon_source.setStyleSheet("")

    # ─────────────────────────────────────────────────────────────────
    # GPS 接收延遲
    # ─────────────────────────────────────────────────────────────────
    def gps_lag(self) -> Tuple[Optional[float], str, Optional[str]]:
        """(秒, 來源說明, 錯誤)。空白 = 0（明確顯示為有效值）。"""
        s = self.ed_gps_lag.text().strip()
        if not s:
            return 0.0, GPS_LAG_BLANK_SRC, None
        try:
            ms = float(s)
        except ValueError:
            return None, GPS_LAG_USER_SRC, f"GPS 接收延遲不是數字：{s!r}"
        if not math.isfinite(ms):
            return None, GPS_LAG_USER_SRC, f"GPS 接收延遲不是有限數值：{s!r}"
        if not (0.0 <= ms <= GPS_LAG_MAX_MS):
            return None, GPS_LAG_USER_SRC, f"GPS 接收延遲 {ms:g} ms 超出合理範圍 0–{GPS_LAG_MAX_MS:g} ms"
        return ms / 1000.0, self._gps_lag_source or GPS_LAG_USER_SRC, None

    def _on_gps_lag_edited(self, _text: str) -> None:
        """操作員動手改過 ⇒ 來源變成操作員輸入（程式 setText 不會觸發 textEdited）。"""
        self._gps_lag_source = None
        self.lbl_gps_lag_log.setText("")

    def _refresh_gps_lag_label(self) -> None:
        lag, src, err = self.gps_lag()
        if err:
            self.lbl_gps_lag.setText(f"✘ {err}")
            self.lbl_gps_lag.setStyleSheet("color:#E53935;")
        else:
            self.lbl_gps_lag.setText(f"有效值：{lag * 1000.0:g} ms（{src}）")
            self.lbl_gps_lag.setStyleSheet("")

    def fill_gps_lag_from_logs(self) -> Optional[float]:
        """讀每一列 .BIN 的 GPS_DELAY_MS（4.6 起 GPS1_DELAY_MS），各架一致且 > 0 才帶入欄位並標明來源。

        回傳帶入的毫秒數；不帶入時回傳 None、欄位不動，原因寫在欄位下。GPS_DELAY_MS = 0 表示飛控用
        GPS 驅動依接收機型號的預設延遲，那個數值不在日誌裡 ⇒ 不猜。
        """
        reader = self.param_reader or read_log_params
        rows = []
        for r in range(self.sortie_table.rowCount()):
            b = self._cell(r, COL_BIN)
            if b:
                rows.append((self._cell(r, COL_CALLSIGN) or f"第 {r + 1} 列", b))
        problems, vals = [], {}
        if not rows:
            problems.append("架次表沒有任何 .BIN")
        for name, path in rows:
            try:
                params, changed = reader(path)
            except Exception as e:                   # noqa: BLE001 - 讀檔失敗就照實說
                problems.append(f"{name}：讀不到日誌參數（{type(e).__name__}: {e}）")
                continue
            key = next((k for k in GPS_DELAY_NAMES if k in params), None)
            if key is None:
                problems.append(f"{name}：日誌沒有 {'／'.join(GPS_DELAY_NAMES)}")
            elif key in changed:
                problems.append(f"{name}：{key} 在飛行中改過")
            else:
                vals[name] = (key, float(params[key]))
        if not problems:
            distinct = sorted({v for _k, v in vals.values()})
            if len(distinct) != 1:
                problems.append("各架不同（" + "、".join(f"{n} {v:g} ms" for n, (_k, v) in vals.items())
                                + "）：流水線只吃單一值，不帶入")
            elif not distinct[0] > 0:
                keys = "／".join(sorted({k for k, _v in vals.values()}))
                problems.append(f"{keys} = 0：飛控用 GPS 驅動依接收機型號的預設延遲，數值不在日誌裡 ⇒ 不帶入，請自行填")
        if problems:
            msg = "不帶入：" + "；".join(problems)
            self.lbl_gps_lag_log.setText(msg)
            self._log(f"GPS 延遲：{msg}")
            return None
        ms = next(iter(vals.values()))[1]
        keys = "／".join(sorted({k for k, _v in vals.values()}))
        self._gps_lag_source = (f"取自飛控參數 {keys}（{'、'.join(vals)} 皆 {ms:g} ms；"
                                "EKF 的延遲模型值，不是實測）")
        self.ed_gps_lag.setText(f"{ms:g}")
        self._refresh_gps_lag_label()
        self.lbl_gps_lag_log.setText("")
        self._log(f"GPS 延遲：{self._gps_lag_source}")
        return ms

    # ─────────────────────────────────────────────────────────────────
    # 收集輸入
    # ─────────────────────────────────────────────────────────────────
    def sensor_ts_semantics(self) -> Optional[str]:
        if self.rb_exposure_start.isChecked():
            return "exposure_start"
        if self.rb_readout_start.isChecked():
            return "readout_start"
        return None

    def collect_inputs(self) -> Tuple[Optional[Dict[str, Any]], List[str]]:
        """回傳 ({'sorties', 'out_dir', 'kwargs'}, []) 或 (None, 錯誤清單)。"""
        from survey import datum
        from survey.pipeline import PipelineError, SortieInput, check_geo_projection
        errors: List[str] = []
        sem = self.sensor_ts_semantics()
        if sem is None:
            errors.append("請選擇 SensorTimestamp 語意（台架 B3 未定案，沒有預設值）")
        sorties = []
        for r in range(self.sortie_table.rowCount()):
            cs, bin_p, sess, ubx, pos = (self._cell(r, c) for c in range(len(SORTIE_HEADERS)))
            if not (cs or bin_p or sess or pos):
                continue
            if not cs:
                errors.append(f"第 {r + 1} 列：呼號必填")
            if not bin_p or not Path(bin_p).is_file():
                errors.append(f"第 {r + 1} 列：飛控 .BIN 不存在：{bin_p or '（空白）'}")
            if not sess or not (Path(sess) / "session.json").is_file():
                errors.append(f"第 {r + 1} 列：Pi 會期資料夾不對（缺 session.json）：{sess or '（空白）'}")
            if pos and not Path(pos).is_file():
                errors.append(f"第 {r + 1} 列：.pos 不存在：{pos}")
            sorties.append(SortieInput(cs, bin_p, sess, rover_ubx=ubx or None, pos_path=pos or None))
        if not sorties:
            errors.append("至少要有一個架次")

        height = self.cmb_height.currentData()
        n_err = len(errors)
        n = _float_or_none(self.ed_geoid_n.text(), "大地起伏 N", errors)
        geoid = None if n is None else datum.GeoidModel(self.cmb_geoid_datum.currentText(), n, "操作員輸入的常數")
        if height == "orthometric" and geoid is None and len(errors) == n_err:
            errors.append("選了正高但大地起伏 N 空白：無法算正高（不會用 N=0）")
        ground = _float_or_none(self.ed_ground_h.text(), "地面高", errors)
        sources: Dict[str, str] = {}
        lag, lag_src, lag_err = self.gps_lag()
        self._refresh_gps_lag_label()
        if lag_err:
            errors.append(lag_err)
        else:
            sources["gps_lag_s"] = lag_src
        rings = None
        self._refresh_polygon_label()
        if self.chk_use_polygon.isChecked():
            rings, poly_src, poly_err = self.survey_rings()
            if poly_err:
                errors.append(poly_err)
            else:
                sources["polygons"] = poly_src
        geo_proj = self.ed_geo_projection.text().strip() or None
        if geo_proj:
            try:
                check_geo_projection(geo_proj)
            except PipelineError as e:
                errors.append(str(e))
        ppk = None
        if self.grp_ppk.isChecked():
            ppk, perr = self._collect_ppk(sorties)
            errors += perr
        out = self.ed_out_dir.text().strip()
        if not out:
            errors.append("輸出資料夾必填")
        if errors:
            return None, errors
        kwargs = dict(sensor_ts_semantics=sem, height=height, geoid=geoid, geo_projection=geo_proj, ppk=ppk,
                      ground_h_m=ground, polygons=rings, gps_lag_s=lag, input_sources=sources,
                      overwrite=self.chk_overwrite.isChecked(),
                      wind_limit_mps=float(self.spn_wind_limit.value()))
        return {"sorties": sorties, "out_dir": out, "kwargs": kwargs}, []

    def _collect_ppk(self, sorties) -> Tuple[Optional[Any], List[str]]:
        from survey.pipeline import PipelineError, PpkSettings, rover_ubx_for
        from survey.ppk.rtklib import BaseStation
        errors: List[str] = []
        frame = self.ed_base_frame.text().strip()
        if not frame:
            errors.append("PPK：基站座標框架名稱必填（例如 e-GNSS[2025]）")
        run_rtklib = self.rb_ppk_rtklib.isChecked()
        base = base_obs = base_ubx = None
        if run_rtklib:
            vals = {}
            for key, edit, name in (("lat", self.ed_base_lat, "基站緯度"), ("lon", self.ed_base_lon, "基站經度"),
                                    ("h", self.ed_base_h, "基站橢球高"), ("ant", self.ed_base_ant, "天線高")):
                n_err = len(errors)
                vals[key] = _float_or_none(edit.text(), f"PPK：{name}", errors)
                if vals[key] is None and len(errors) == n_err:
                    errors.append(f"PPK：{name}必填")
            rinex, bubx = self.ed_base_rinex.text().strip(), self.ed_base_ubx.text().strip()
            if bool(rinex) == bool(bubx):
                errors.append("PPK：基站觀測檔要給 RINEX 或 UBX 其中一個")
            elif not Path(rinex or bubx).is_file():
                errors.append(f"PPK：基站觀測檔不存在：{rinex or bubx}")
            base_obs, base_ubx = rinex or None, bubx or None
            _, _, err = self._rtklib_paths()
            if err:
                errors.append(f"未安裝 RTKLIB：{err}")
            if frame and all(v is not None for v in vals.values()):
                try:
                    base = BaseStation(vals["lat"], vals["lon"], vals["h"], frame, antenna_height_m=vals["ant"])
                except ValueError as e:
                    errors.append(f"PPK：{e}")
        else:
            for s in sorties:
                if not s.pos_path:
                    errors.append(f"PPK：{s.callsign or s.session_dir} 沒有填 .pos（或改選 RTKLIB 解算）")
        for s in sorties:
            if s.session_dir and rover_ubx_for(s) is None:
                errors.append(f"PPK：{s.callsign or s.session_dir} 找不到 rover.ubx（會期的 gnss/rover.ubx）")
        if errors:
            return None, errors
        try:
            ppk = PpkSettings(frame=frame, base=base, base_obs=base_obs, base_ubx=base_ubx, run_rtklib=run_rtklib,
                              convbin_exe=self.ed_convbin.text().strip() or None,
                              rnx2rtkp_exe=self.ed_rnx2rtkp.text().strip() or None)
        except PipelineError as e:
            return None, [str(e)]
        return ppk, []

    # ─────────────────────────────────────────────────────────────────
    # 處理
    # ─────────────────────────────────────────────────────────────────
    def is_busy(self) -> bool:
        return self._worker is not None or self._odm_worker is not None

    def _show_errors(self, errors: List[str]) -> None:
        self.lbl_errors.setText("\n".join(f"✘ {e}" for e in errors))

    def _log(self, msg: str) -> None:
        self.log_view.appendPlainText(msg)
        logger.info(f"[航測處理] {msg}")

    def start_processing(self) -> bool:
        from ui.workers.survey_worker import SurveyProcessWorker
        if self.is_busy():
            return False
        inp, errors = self.collect_inputs()
        self._show_errors(errors)
        if errors:
            return False
        self.result = None
        self.odm_result = None
        self.qa_table.setRowCount(0)
        self.log_view.clear()
        self.lbl_summary.setText("處理中…")
        w = SurveyProcessWorker(inp["sorties"], inp["out_dir"], inp["kwargs"], process_fn=self.process_fn, parent=self)
        w.progress.connect(self._on_progress)
        w.finished_ok.connect(self._on_finished)
        w.failed.connect(self._on_failed)
        w.canceled.connect(self._on_canceled)
        self._worker = w
        self._update_buttons()
        w.start()
        return True

    def cancel(self) -> None:
        for w in (self._worker, self._odm_worker):
            if w is not None:
                w.cancel()
        if self.is_busy():
            self.lbl_progress.setText("取消中（等目前的步驟結束）…")

    def _on_progress(self, msg: str, frac: float) -> None:
        self.lbl_progress.setText(msg)
        if frac >= 0:
            self.progress_bar.setValue(int(round(frac * 1000)))
        self.log_view.appendPlainText(msg)

    def _end_worker(self) -> None:
        if self._worker is not None:
            self._worker.wait()
        self._worker = None
        self._update_buttons()

    def _on_failed(self, msg: str) -> None:
        self._end_worker()
        self.lbl_summary.setText(f"處理失敗：{msg}")
        self._show_errors([msg])
        self._log(f"✘ 失敗：{msg}")

    def _on_canceled(self) -> None:
        self._end_worker()
        self.lbl_summary.setText("已取消（輸出資料夾可能只寫了一部分）")
        self._log("已取消")

    def _on_finished(self, result) -> None:
        self._end_worker()
        self.result = result
        self.progress_bar.setValue(1000)
        self._fill_results(result)
        gsd = result.odm.get("gsd_cm")
        if gsd and not self._res_from_user:
            self.spn_res.blockSignals(True)
            self.spn_res.setValue(float(gsd))
            self.spn_res.blockSignals(False)
        self._refresh_readout()
        self._prefill_dsm_vertical(result)
        self._show_on_map(result)
        self._update_buttons()
        self.tabs.setCurrentIndex(3)
        if self.chk_auto_submit.isChecked() and result.passed and self.btn_odm.isEnabled():
            self.submit_odm()

    def _fill_results(self, result) -> None:
        from survey.pipeline import qa_table_rows
        rows = qa_table_rows(result)
        self.qa_table.setRowCount(len(rows))
        for i, (uav, item, value, ok) in enumerate(rows):
            mark = "—" if ok is None else ("✔ 通過" if ok else "✘ 未通過")
            for c, txt in enumerate((uav, item, value, mark)):
                it = QTableWidgetItem(str(txt))
                if c == 3 and ok is False:
                    it.setForeground(Qt.GlobalColor.red)
                self.qa_table.setItem(i, c, it)
        head = "QA 通過" if result.passed else f"QA 未通過（{len(result.qa['reasons'])} 項）"
        lines = [f"{head}｜照片 {len(result.rows)} 張｜加標籤影像 {len(result.images)} 張｜輸出 {result.out_dir}"]
        eff = result.effective or {}
        if eff.get("gps_lag_s") is not None:                      # 有效值（流水線實際用的），不是欄位文字
            src = (eff.get("input_sources") or {}).get("gps_lag_s", "未記錄來源")
            lines.append(f"GPS 接收延遲補償 {float(eff['gps_lag_s']) * 1000.0:g} ms（{src}）")
        if eff.get("polygons"):
            lines.append(f"視數測區：{len(eff['polygons'])} 塊")
        lines += [f"✘ {r}" for r in result.qa["reasons"]]
        lines += [f"⚠ {w}" for w in result.warnings]
        self.lbl_summary.setText("\n".join(lines))
        self._log("有效設定：" + json.dumps(result.effective, ensure_ascii=False, sort_keys=True, default=str)[:4000])

    def _show_on_map(self, result) -> None:
        csv_path = result.paths.get("photos_csv")
        if self.map_widget is None or not csv_path:
            return
        try:
            eff = self.map_widget.load_photo_poses_file(PHOTO_POSE_LAYER, str(csv_path))
            self._log(f"地圖：照片位姿 {csv_path} → {eff}")
        except Exception as e:                       # noqa: BLE001 - 地圖失敗不能讓成果消失
            self._log(f"⚠ 地圖顯示照片位姿失敗：{type(e).__name__}: {e}")

    # ─────────────────────────────────────────────────────────────────
    # ODM
    # ─────────────────────────────────────────────────────────────────
    def _on_res_changed(self, _v) -> None:
        self._res_from_user = True
        self._update_buttons()

    def odm_block_reason(self) -> Optional[str]:
        """「送 ODM」不能按的原因；None = 可以。"""
        if self.is_busy():
            return "處理中"
        r = self.result
        if r is None:
            return "還沒有處理結果"
        if r.paths.get("geo_txt") is None or not r.images:
            return "沒有 geo.txt 或加標籤影像（見處理錯誤）"
        if not r.passed and not self.chk_qa_override.isChecked():
            return "QA 未通過（要送出必須勾「QA 未通過仍送出」）"
        if self.spn_res.value() <= 0:
            return "正射解析度未設定"
        gcp = self.ed_gcp_txt.text().strip()
        if gcp:
            if not Path(gcp).is_file():
                return f"GCP 檔不存在：{gcp}"
            if self.cmb_gcp_vertical.currentData() is None:
                return "選了 GCP 檔，但還沒選它的 geo_z 高程基準"
        return None

    def _update_buttons(self) -> None:
        busy = self.is_busy()
        self.btn_run.setEnabled(not busy)
        self.btn_cancel.setEnabled(busy)
        reason = self.odm_block_reason()
        self.btn_odm.setEnabled(reason is None)
        self.btn_odm.setToolTip(reason or "上傳加標籤影像與 geo.txt 到 NodeODM")

    def submit_odm(self) -> bool:
        from survey.pipeline import PipelineError, odm_job
        from ui.workers.survey_worker import OdmSubmitWorker
        reason = self.odm_block_reason()
        if reason is not None:
            self._show_errors([f"不能送 ODM：{reason}"])
            return False
        override = self.chk_qa_override.isChecked() and not self.result.passed
        gcp_kw = {}
        gcp = self.ed_gcp_txt.text().strip()
        if gcp:
            gcp_kw = dict(gcp_txt=gcp, gcp_vertical=self.cmb_gcp_vertical.currentData(),
                          checkpoint_ids=self._checkpoint_ids_for_gcp_check())
        try:
            job = odm_job(self.result, node_url=self.ed_odm_url.text(), orthophoto_resolution_cm=self.spn_res.value(),
                          rolling_shutter=self.chk_rolling.isChecked(), qa_override=override, **gcp_kw)
        except PipelineError as e:
            self._show_errors([str(e)])
            if gcp:
                self.lbl_gcp.setText(f"✘ {e}")
                self.lbl_gcp.setStyleSheet("color:#E53935;")
            return False
        job["orthophoto_resolution_source"] = "user" if self._res_from_user else "gsd"
        self.result.effective["odm_submission"] = {k: job[k] for k in (
            "node_url", "options", "qa_passed", "qa_override", "qa_reasons", "rolling_shutter",
            "orthophoto_resolution_source")}
        if "gcp" in job:
            self.result.effective["odm_submission"].update(gcp_txt=job["gcp_txt"], gcp=job["gcp"])
            g = job["gcp"]
            self.lbl_gcp.setText(f"GCP：{g['n_gcps']} 個、{g['n_lines']} 行；標頭 {g['header']}；geo_z "
                                 f"{'橢球高' if g['gcp_vertical'] == 'ellipsoidal' else '正高'}"
                                 + ("；⚠ " + "；".join(g["warnings"][:3]) if g["warnings"] else ""))
            self.lbl_gcp.setStyleSheet("")
        self._log("送 ODM：" + json.dumps(self.result.effective["odm_submission"], ensure_ascii=False, default=str))
        w = OdmSubmitWorker(job, run_fn=self.odm_run_fn, client=self.odm_client, parent=self)
        w.progress.connect(self._on_progress)
        w.finished_ok.connect(self._on_odm_finished)
        w.failed.connect(self._on_odm_failed)
        w.canceled.connect(self._on_odm_canceled)
        self._odm_worker = w
        self._update_buttons()
        w.start()
        return True

    def _end_odm_worker(self) -> None:
        if self._odm_worker is not None:
            self._odm_worker.wait()
        self._odm_worker = None
        self._update_buttons()

    def _on_odm_failed(self, msg: str) -> None:
        self._end_odm_worker()
        self._show_errors([f"ODM：{msg}"])
        self._log(f"✘ ODM 失敗：{msg}")

    def _on_odm_canceled(self) -> None:
        self._end_odm_worker()
        self._log("ODM：已停止等待（NodeODM 上的任務仍在執行）")

    def _on_odm_finished(self, info: dict) -> None:
        self._end_odm_worker()
        self.odm_result = info
        tiles = info.get("ortho_tiles")
        self._log(f"ODM 完成：任務 {info.get('uuid')}，成果 {info.get('extract_dir')}")
        self._after_odm_checkpoints(info)
        if not tiles:
            self._log("⚠ 成果裡沒有 orthophoto_tiles（任務沒開 --tiles？），地圖不疊正射")
            return
        if self.map_widget is None:
            return
        try:
            eff = self.map_widget.add_ortho_layer(ORTHO_LAYER, str(tiles), scheme="tms")
            self._log(f"地圖：正射瓦片 {tiles} → {eff}")
        except Exception as e:                       # noqa: BLE001
            self._log(f"⚠ 地圖疊正射失敗：{type(e).__name__}: {e}")

    # ─────────────────────────────────────────────────────────────────
    # GCP 規劃、檢核點精度
    # ─────────────────────────────────────────────────────────────────
    def open_gcp_plan(self):
        """開「GCP／檢核點規劃」（非模態；同一個對話框重複開只是叫到前面）。"""
        from ui.dialogs.gcp_plan_dialog import GcpPlanDialog
        if self._gcp_dialog is None:
            self._gcp_dialog = GcpPlanDialog(parent=self, map_widget=self.map_widget, polygons=self.polygons_source,
                                             blocks=self.gcp_blocks_source)
        self._gcp_dialog.show()
        self._gcp_dialog.raise_()
        return self._gcp_dialog

    def _checkpoint_ids_for_gcp_check(self) -> List[str]:
        """實測檢核點的編號（送 ODM 前擋「檢核點混進 GCP 檔」用）；沒有 CSV 就是空的。"""
        p = self.ed_chk_survey.text().strip()
        if not p or not Path(p).is_file():
            return []
        from survey.checkpoints import checkpoint_ids
        try:
            return checkpoint_ids(p)
        except (OSError, KeyError, ValueError) as e:
            self._log(f"⚠ 讀檢核點編號失敗（GCP 檔不檢查檢核點混入）：{type(e).__name__}: {e}")
            return []

    def _prefill_dsm_vertical(self, result) -> None:
        """ODM 不改高程基準 ⇒ DSM 的基準 = geo.txt 用的高度。只在操作員還沒選過時預填，並標明來源。"""
        if self._dsm_vertical_source == "user":
            return
        eff = getattr(result, "effective", None) or {}
        h = eff.get("height")
        val = "ellipsoidal" if h == "ellipsoidal" else ((eff.get("geoid") or {}).get("datum") if h == "orthometric"
                                                        else None)
        idx = self.cmb_dsm_vertical.findData(val) if val else -1
        if idx < 0:
            return
        self.cmb_dsm_vertical.setCurrentIndex(idx)
        self._dsm_vertical_source = "geo.txt"
        self.lbl_dsm_vertical.setText(f"預填：取自本次處理 geo.txt 的高度（{self.cmb_dsm_vertical.currentText()}；"
                                      "ODM 不改高程基準）。GCP 用了別的基準時請自己改。")

    def _on_dsm_vertical_picked(self) -> None:
        self._dsm_vertical_source = "user"
        self.lbl_dsm_vertical.setText("操作員選擇")

    def compute_checkpoint_report(self) -> Optional[dict]:
        """用欄位裡的輸入算檢核點精度報告；寫 checkpoint_report.json／.md 到輸出資料夾的 checkpoints/。"""
        from survey import checkpoints as ck
        from survey import gcp_plan
        errors: List[str] = []
        sp = self.ed_chk_survey.text().strip()
        if not sp or not Path(sp).is_file():
            errors.append(f"實測檢核點 CSV 不存在：{sp or '（空白）'}")
        mp = self.ed_chk_measured.text().strip()
        if mp and not Path(mp).is_file():
            errors.append(f"正射量測 CSV 不存在：{mp}")
        dsm = self.ed_dsm.text().strip()
        if dsm and not Path(dsm).is_file():
            errors.append(f"DSM 不存在：{dsm}")
        lp = self.ed_chk_layout.text().strip()
        if lp and not Path(lp).is_file():
            errors.append(f"佈設 CSV 不存在：{lp}")
        und = _float_or_none(self.ed_chk_undulation.text(), "大地起伏 N", errors)
        if not mp and not dsm:
            errors.append("至少要給正射量測 CSV 或 DSM")
        dv = self.cmb_dsm_vertical.currentData()
        if dsm and dv is None:
            errors.append("請選 DSM 的高程基準（不猜）")
        if errors:
            self.chk_report_view.setPlainText("\n".join(f"✘ {e}" for e in errors))
            return None
        gcp_names = []
        gcp = self.ed_gcp_txt.text().strip()
        try:
            if gcp and Path(gcp).is_file():
                gcp_names = sorted({g.name for g in gcp_plan.read_gcp_list(gcp)[2]})
            rep = ck.accuracy_report(
                ck.read_surveyed_csv(sp), measured=ck.read_measured_csv(mp) if mp else None,
                dsm_path=dsm or None, dsm_vertical=dv if dsm else None, undulation=und,
                frame_crs=self.ed_chk_frame_crs.text().strip() or None,
                layout=gcp_plan.read_layout_csv(lp) if lp else None, gcp_names=gcp_names,
                target_rmse_h_m=(self.spn_target_h.value() / 100.0) or None,
                target_rmse_v_m=(self.spn_target_v.value() / 100.0) or None)
        except (ck.CheckpointError, gcp_plan.GcpPlanError, OSError) as e:
            self.chk_report_view.setPlainText(f"✘ {e}")
            return None
        if dsm:
            rep["effective"]["dsm_vertical_source"] = self._dsm_vertical_source or "user"
        self.checkpoint_report = rep
        lines = ck.report_lines(rep)
        out = self.ed_out_dir.text().strip() or (str(self.result.out_dir) if self.result is not None else "")
        if out:
            try:
                paths = ck.write_report(rep, Path(out) / "checkpoints")
                lines.append(f"報告：{paths['json']}、{paths['md']}")
            except OSError as e:
                lines.append(f"⚠ 報告寫檔失敗：{e}")
        self.chk_report_view.setPlainText("\n".join(lines))
        self._log("檢核點報告：" + lines[2] + "；" + lines[3])
        return rep

    def _after_odm_checkpoints(self, info: dict) -> None:
        """ODM 完成：DSM 欄位空白就帶入成果裡的 dsm.tif；有實測檢核點就順便算報告並切到那一頁。"""
        from survey.pipeline import find_odm_dsm
        if not self.ed_dsm.text().strip() and info.get("extract_dir"):
            dsm = find_odm_dsm(info["extract_dir"])
            if dsm is not None:
                self.ed_dsm.setText(str(dsm))
                self._log(f"DSM：{dsm}")
        if self.ed_chk_survey.text().strip():
            if self.compute_checkpoint_report() is not None:
                self.tabs.setCurrentIndex(4)

    # ─────────────────────────────────────────────────────────────────
    def showEvent(self, event):                      # noqa: N802 - Qt 介面
        self._refresh_polygon_label()                # 非模態：主視窗的作業區可能在對話框關著時變了
        super().showEvent(event)

    def closeEvent(self, event):                     # noqa: N802 - Qt 介面
        if self.is_busy():
            self.cancel()
            self._show_errors(["處理中：已要求取消，等目前的步驟結束後再關閉"])
            event.ignore()
            return
        super().closeEvent(event)

    def reject(self):
        if self.is_busy():
            self.cancel()
            self._show_errors(["處理中：已要求取消，等目前的步驟結束後再關閉"])
            return
        super().reject()
