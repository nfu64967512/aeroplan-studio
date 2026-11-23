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
* PPK 用既有 .pos（RTKLIB／Emlid Studio）時基站座標（緯、經、橢球高、天線高）一樣必填：流水線會核對 .pos 表頭的
  ``% ref pos``（各 5 mm），框架名稱才綁在解算實際用的那組座標上；Emlid Studio 的 ``_events.pos`` 直接擋（要整條軌跡）。
* QA 未通過時「送 ODM」停用；要送必須勾「QA 未通過仍送出」，這個選擇寫進任務紀錄（odm/submission.json）。
* 建圖風速上限（預設 3 m/s，使用者 2026-10-15 的首飛決定）：超過只標示（QA 表與摘要的 ⚠），不擋送 ODM。
* gcp_list.txt：geo_z 的高程基準沒有預設 —— 必須明確選，而且要與 geo.txt 的高度相同；送出前逐行核對
  （樣板佔位、影像檔名、像素範圍、檢核點不可混入），核對摘要寫進任務紀錄。
* 檢核點報告：DSM 的高程基準要明確宣告（處理完成後預填「geo.txt 用的高度」並標明來源），與檢核點不同時
  除非給大地起伏 N，否則拒絕。
* --force-gps（2026-10-20 Step 0）：選了 gcp_list.txt 就**必須**明確選「照片 GPS 也參與平差」或「只用 GCP」
  （有 GCP 時 ODX 預設照片 GPS 不進平差，會把 PPK 位置丟掉）；沒選不能送。沒有 GCP 時不適用、不送。
* --max-concurrency：空白／0 = 不送（引擎預設＝節點全部核心）；送不送、值與來源都寫進任務紀錄。
* NodeODX 權杖：只放在欄位與客戶端物件裡，不寫進任何檔案、紀錄或 log；接回／取消遠端任務要再填一次。
* 計畫來源：「目前規劃器」（即時狀態，舊行為）或「飛行計畫快照」（建圖任務上傳成功時存的那一份，survey.plan_snapshot）；
  用了哪一個寫進 qa.json 的 effective.input_sources（plan_source／plan_snapshot：檔名、plan／digest 雜湊）與 ODM 任務紀錄。
  選了快照卻沒載入成功 ⇒ 拒絕，不退回即時狀態。

ODM 任務（survey.pipeline 的 run／resume／cancel_odm_job）：節點與引擎版本（GET /info）、真的進度（上傳位元組、
節點 info.progress、下載位元組）、「接回既有任務…」（選輸出資料夾，讀 odm/submission.json 繼續等待／下載）、
「取消遠端任務」（確認後 POST /task/cancel；本機有工作在跑就先停）；完成後品質報告（odm_report/stats.json 等）列進「結果」頁。
"""
from __future__ import annotations

import json
import math
import re
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (QAbstractItemView, QButtonGroup, QCheckBox, QComboBox, QDialog, QDoubleSpinBox,
                             QFileDialog, QFormLayout, QGroupBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit,
                             QMessageBox, QPlainTextEdit, QProgressBar, QPushButton, QRadioButton, QSpinBox,
                             QTabWidget, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)

from utils.logger import get_logger

logger = get_logger()

SORTIE_HEADERS = ["呼號", "飛控 .BIN", "Pi 會期資料夾", "移動站 UBX（自動）", "既有 .pos（選填）"]
COL_CALLSIGN, COL_BIN, COL_SESSION, COL_UBX, COL_POS = range(5)
POS_FILE_FILTER = "RTKLIB／Emlid Studio 軌跡 (*.pos)"
BASE_HINT_RTKLIB = ("基站座標寫進 rnx2rtkp 設定（ant2-pos：標石；天線高另計）；解出的 .pos 表頭 % ref pos 也會再核對一次。")
BASE_HINT_POS = ("用既有 .pos（例如 Emlid Studio）也要填基站座標：要與解算軟體裡輸入的一致（標石的橢球高，"
                 "不是 RINEX 表頭的單點平均位置），流水線會核對 .pos 表頭的 % ref pos（水平、垂直各 ≤ 5 mm），"
                 "不一致就整架不用 PPK。Emlid Studio 的 _events.pos 不能用，請給同一次解算的整條軌跡 .pos。")
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

# --force-gps：沒有預設（第一項 = 未選）；文字照使用者 2026-10-20 的用語
FORCE_GPS_ITEMS = (("（有 GCP 時必選：照片 GPS 要不要進平差）", None),
                   ("照片 GPS 也參與平差（--force-gps；有 PPK 時建議）", True),
                   ("只用 GCP（照片 GPS 不參與平差）", False))
FORCE_GPS_NA_TEXT = "不適用：沒有 GCP 檔（平差只用照片 GPS，--force-gps 沒有作用、不送）"
FORCE_GPS_TEXT = {
    None: ("✘ 必選（沒有預設）：有 GCP 時 ODX 預設 bundle_use_gps: no，照片 GPS（PPK）只用來初始對位、不進平差；"
           "要送出必須明確選一個"),
    True: ("送 --force-gps：照片位置（PPK）與 GCP 一起平差；OpenSfM 另估每台相機的 GPS 偏差"
           "（常數偏移被吸收，看品質報告的「相機偏差」列）"),
    False: ("不送 --force-gps：只用 GCP 平差；照片位置只用於初始對位與配對 ⇒ 品質報告的 GPS 誤差是 PPK 對 GCP 解的"
            "獨立檢核"),
}
MAX_CONC_MAX = 1024
MAX_CONC_HINT = ("空白（0）＝不送，引擎預設用節點全部核心。峰值記憶體經驗值：每個執行緒、每 2 MP 影像約 1 GB"
                 "（ODX --max-concurrency 說明）⇒ 128 GB 工作站、約 2,500 張 12.3 MP：建議 10–12。"
                 "節點若設了更低的上限會被壓低，提交後的任務紀錄會記下實際值。")
ODM_POLL_S = 10.0                    # 等待節點時每幾秒查一次 /task/{uuid}/info
ODM_CANCEL_POLL_S = 2.0              # 取消遠端任務後，每幾秒查一次節點是否已停
ODM_OP_TEXT = {"submit": "送出", "reattach": "接回任務", "cancel_remote": "取消遠端任務", "probe": "測試連線"}
# 未提交（還在上傳）的任務：本機紀錄有 uuid、節點上有暫存上傳 ⇒ 「取消遠端任務」走 cancel_odm_job 的清理路徑
ODM_UNCOMMITTED_CANCELABLE = ("initialized", "uploading", "uploaded")
ODM_NOT_REATTACHABLE = ("failed", "canceled", "lost")
PLAN_LIVE_SRC = "即時狀態（目前規劃器與主視窗，不是飛行計畫快照）"
PLAN_LIVE_TEXT = ("目前用即時狀態：測區＝主視窗的作業區（或地圖角點）、UAV 區塊＝HARVEST 最近一次規劃。"
                  "飛完、重開程式、改了作業區或重新規劃之後，即時狀態就不一定是當初飛的那一份 ⇒ 建議載入上傳時存的飛行計畫快照。"
                  "（快照只在 AeroPlan 自己把建圖任務上傳成功時才存；用「匯出」的檔案在 Mission Planner 等外部工具上傳的任務"
                  "沒有快照，後處理只能用即時狀態。）")


def _probe_node(job, *, client, progress, should_cancel, poll_s):
    """「測試連線」（OdmSubmitWorker 的 run_fn）：GET /info ＋ GET /options；不建立任務、不上傳。"""
    from survey.odm_client import OdmError
    progress("查詢節點（GET /info）…", None)
    info = client.node_info()
    try:
        known, oerr = sorted(str(o.get("name")) for o in client.engine_options() if isinstance(o, dict)), None
    except OdmError as e:
        known, oerr = None, str(e)
    return {"node_url": client.base_url, "token_used": bool(getattr(client, "token", None)), "info": info,
            "options": known, "options_error": oerr}


def _cancel_remote_job(job, *, client, progress, should_cancel, poll_s, on_state=None):
    """「取消遠端任務」（OdmSubmitWorker 的 run_fn）：``survey.pipeline.cancel_odm_job``，等節點確認時每 0.2 s 看一次
    本機取消（關主視窗時不必等滿 120 s；取消請求已經送出，之後可再接回看狀態）。"""
    from survey.pipeline import PipelineCanceled, cancel_odm_job

    def sleep(dt):
        end = time.monotonic() + float(dt)
        while True:
            if should_cancel():
                raise PipelineCanceled("已停止等待節點確認取消（取消請求已送出；之後可用「接回既有任務…」查看狀態）")
            rem = end - time.monotonic()
            if rem <= 0:
                return
            time.sleep(min(0.2, rem))
    return cancel_odm_job(job, client=client, progress=progress, should_cancel=should_cancel, poll_s=poll_s,
                          sleep=sleep, on_state=on_state)


def _same_dir(a, b) -> bool:
    """兩個資料夾是不是同一個（解析過的絕對路徑，不分大小寫——Windows）。"""
    import os

    def norm(p):
        try:
            p = Path(p).resolve()
        except OSError:
            p = Path(p).absolute()
        return os.path.normcase(str(p))
    return norm(a) == norm(b)


def odm_node_text(info: dict, options=None, options_error=None) -> str:
    """GET /info（＋ /options）→ 給操作員看的一段：節點版本、引擎版本、上限、資源；引擎認不認得 force-gps／max-concurrency。"""
    gib = 2 ** 30

    def mem(v):
        try:
            return f"{float(v) / gib:.0f} GB"
        except (TypeError, ValueError):
            return "?"
    ev = info.get("engineVersion")
    parts = [f"NodeODX {info.get('version', '?')}",
             (f"引擎 ODX {ev}" if ev not in (None, "", "?") else f"⚠ 引擎版本未知（engineVersion={ev!r}）"),
             f"maxImages {info.get('maxImages', '?')}", f"CPU {info.get('cpuCores', '?')} 核",
             f"記憶體 可用 {mem(info.get('availableMemory'))}／共 {mem(info.get('totalMemory'))}",
             f"排隊 {info.get('taskQueueCount', '?')}"]
    txt = "；".join(parts) + "（info.engine 不是證據，版本以 engineVersion 與成果的 log.json 為準）"
    if options is not None:
        txt += "\n引擎參數：" + "、".join(f"{n} {'✔' if n in options else '✘ 不認得'}"
                                       for n in ("force-gps", "max-concurrency", "rolling-shutter-readout"))
    elif options_error:
        txt += f"\n⚠ 取不到引擎參數清單（GET /options）：{options_error}"
    return txt


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
        self._dsm_autofilled = None          # ODM 完成後自動帶入的 DSM 路徑（不是操作員選的）
        self._gcp_dialog = None
        # 可替換的處理函式（測試用假的流水線／假 NodeODM／假的日誌參數讀取）；None = 正式實作
        self.process_fn = None
        self.odm_run_fn = None               # 送出：None = survey.pipeline.run_odm_job
        self.odm_resume_fn = None            # 接回：None = survey.pipeline.resume_odm_job
        self.odm_cancel_fn = None            # 取消遠端任務：None = _cancel_remote_job（cancel_odm_job）
        self.odm_client = None               # 固定的客戶端（測試用）；None = 每次用欄位的位址＋權杖建
        self.odm_client_factory = None       # (位址, 權杖或 None) → 客戶端（測試用，例如不吃系統 proxy 的 session）
        self.odm_poll_s = ODM_POLL_S
        self.param_reader = None
        self.result = None
        self.odm_result = None
        self._worker = None
        self._odm_worker = None
        self._odm_op = None                  # 目前 _odm_worker 在做什麼（ODM_OP_TEXT 的鍵）
        self._odm_dir = None                 # 目前／上一個 ODM 任務的處理輸出資料夾（odm/submission.json 在它底下）
        self._odm_status = None              # survey.pipeline.odm_submission_status(紀錄)
        self._odm_status_error = None
        self._odm_node = None                # 「測試連線」的結果
        self._odm_last_phase = None
        self._odm_last_log_key = None
        self._pending_remote_cancel = None   # (job, client)：等本機的 ODM 工作停下來就送取消
        self._plan_snapshot = None           # survey.plan_snapshot.read_snapshot 的結果（已驗證）
        self._plan_snapshot_path = None
        self._plan_snapshot_error = None
        self._res_from_user = False
        self._gps_lag_source = None          # None = 空白或操作員輸入；從日誌帶入時是來源說明
        self._build_ui()
        self._refresh_rtklib_status()
        self._refresh_readout()
        self._refresh_gps_lag_label()
        self._refresh_plan_label()
        self._refresh_polygon_label()
        self._refresh_force_gps_label()
        self._refresh_max_conc_label()
        self._refresh_odm_task_label()
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
        self.btn_browse_pos.clicked.connect(lambda: self._browse_cell(COL_POS, POS_FILE_FILTER))
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

        grp_p = QGroupBox("計畫來源（視數用的測區、「GCP／檢核點規劃…」的 UAV 區塊）")
        pv = QVBoxLayout(grp_p)
        self.rb_plan_live = QRadioButton("目前規劃器（即時：主視窗的作業區、HARVEST 最近一次規劃）")
        self.rb_plan_snapshot = QRadioButton("飛行計畫快照（建圖任務上傳成功時存下的那一份）")
        self.rb_plan_live.setChecked(True)
        self.plan_source_group = QButtonGroup(self)
        self.plan_source_group.setExclusive(True)
        for rb in (self.rb_plan_live, self.rb_plan_snapshot):
            self.plan_source_group.addButton(rb)
            pv.addWidget(rb)
        srow = QHBoxLayout()
        self.ed_plan_snapshot = QLineEdit()
        self.ed_plan_snapshot.setReadOnly(True)
        self.ed_plan_snapshot.setPlaceholderText("尚未載入（預設在 runs/survey/<日期>/flown_plan_*.json）")
        self.btn_plan_snapshot = QPushButton("載入計畫快照…")
        self.btn_plan_snapshot.clicked.connect(self._browse_plan_snapshot)
        srow.addWidget(self.ed_plan_snapshot, 1)
        srow.addWidget(self.btn_plan_snapshot)
        pv.addLayout(srow)
        self.lbl_plan_snapshot = QLabel("")
        self.lbl_plan_snapshot.setWordWrap(True)
        pv.addWidget(self.lbl_plan_snapshot)
        self.rb_plan_live.toggled.connect(lambda _: self._on_plan_source_changed())
        v.addWidget(grp_p)

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
        self.rb_ppk_pos = QRadioButton("用架次表填的既有 .pos（RTKLIB／Emlid Studio 解好的整條軌跡）")
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
        self.box_base_rinex = self._with_browse(self.ed_base_rinex, "RINEX (*.obs *.*o *.rnx *.crx)")
        self.box_base_ubx = self._with_browse(self.ed_base_ubx, "UBX (*.ubx)")
        f.addRow("基站 RINEX", self.box_base_rinex)
        f.addRow("基站 UBX", self.box_base_ubx)
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
        self.lbl_base_hint = QLabel(BASE_HINT_RTKLIB)
        self.lbl_base_hint.setWordWrap(True)
        f.addRow(self.lbl_base_hint)
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
        self.rb_ppk_pos.toggled.connect(lambda _: self._on_ppk_mode_changed())
        self._on_ppk_mode_changed()

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

    def _on_ppk_mode_changed(self) -> None:
        """兩種模式都要基站座標（.pos 模式用來核對 % ref pos）；基站觀測檔與 RTKLIB 執行檔只有 RTKLIB 模式用得到。"""
        rtk = self.rb_ppk_rtklib.isChecked()
        for wdg in (self.ed_base_lat, self.ed_base_lon, self.ed_base_h, self.ed_base_ant, self.ed_base_frame):
            wdg.setEnabled(True)
        for wdg in (self.box_base_rinex, self.box_base_ubx, self.ed_convbin, self.ed_rnx2rtkp, self.btn_rtklib_detect):
            wdg.setEnabled(rtk)
        self.lbl_base_hint.setText(BASE_HINT_RTKLIB if rtk else BASE_HINT_POS)

    def _build_output_tab(self) -> QWidget:
        w = QWidget()
        v = QVBoxLayout(w)
        f = QFormLayout()
        self.ed_out_dir = QLineEdit()
        f.addRow("輸出資料夾", self._with_browse(self.ed_out_dir, None))
        self.chk_overwrite = QCheckBox("輸出資料夾已有成果時覆寫")
        f.addRow(self.chk_overwrite)
        v.addLayout(f)

        grp = QGroupBox("NodeODM／NodeODX")
        of = QFormLayout(grp)
        self.ed_odm_url = QLineEdit(DEFAULT_ODM_URL)
        of.addRow("節點位址", self.ed_odm_url)
        self.ed_odm_token = QLineEdit()
        self.ed_odm_token.setEchoMode(QLineEdit.EchoMode.Password)
        self.ed_odm_token.setPlaceholderText("選填：節點有設 --token 才要填；只在連線時使用，不寫進任何檔案、紀錄或 log")
        of.addRow("權杖", self.ed_odm_token)
        nrow = QHBoxLayout()
        self.btn_odm_test = QPushButton("測試連線")
        self.btn_odm_test.setToolTip("GET /info 與 GET /options：節點與引擎版本、maxImages、記憶體；不建立任務")
        self.btn_odm_test.clicked.connect(self.test_odm_connection)
        self.lbl_odm_node = QLabel("節點與引擎版本：尚未查詢（按「測試連線」；送出時也會先查 GET /info 並寫進任務紀錄）")
        self.lbl_odm_node.setWordWrap(True)
        nrow.addWidget(self.btn_odm_test)
        nrow.addWidget(self.lbl_odm_node, 1)
        of.addRow(nrow)
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
        self.spn_max_conc = QSpinBox()
        self.spn_max_conc.setRange(0, MAX_CONC_MAX)
        self.spn_max_conc.setSpecialValueText("不送（引擎預設＝節點全部核心）")
        self.spn_max_conc.valueChanged.connect(lambda _: self._refresh_max_conc_label())
        of.addRow("--max-concurrency", self.spn_max_conc)
        self.lbl_max_conc = QLabel("")
        self.lbl_max_conc.setWordWrap(True)
        of.addRow(self.lbl_max_conc)
        self.chk_auto_submit = QCheckBox("處理完成且 QA 通過後自動送 ODM")
        of.addRow(self.chk_auto_submit)
        self.chk_qa_override = QCheckBox("QA 未通過仍送出（會寫進任務紀錄）")
        self.chk_qa_override.toggled.connect(lambda _: self._update_buttons())
        of.addRow(self.chk_qa_override)
        brow = QHBoxLayout()
        self.btn_odm = QPushButton("送 ODM")
        self.btn_odm.clicked.connect(self.submit_odm)
        self.btn_odm_reattach = QPushButton("接回既有任務…")
        self.btn_odm_reattach.setToolTip("選一個送過 ODM 的處理輸出資料夾（裡面有 odm/submission.json），繼續等待／下載；"
                                         "權杖要再填一次")
        self.btn_odm_reattach.clicked.connect(self._browse_reattach)
        self.btn_odm_cancel_remote = QPushButton("取消遠端任務")
        self.btn_odm_cancel_remote.clicked.connect(lambda: self.cancel_remote_odm())
        for b in (self.btn_odm, self.btn_odm_reattach, self.btn_odm_cancel_remote):
            brow.addWidget(b)
        of.addRow(brow)
        self.odm_progress = QProgressBar()
        self.odm_progress.setRange(0, 1000)
        self.odm_progress.setValue(0)
        of.addRow("任務進度", self.odm_progress)
        self.lbl_odm_task = QLabel("")
        self.lbl_odm_task.setWordWrap(True)
        self.lbl_odm_task.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        of.addRow(self.lbl_odm_task)
        self.lbl_odm_msg = QLabel("")
        self.lbl_odm_msg.setWordWrap(True)
        of.addRow(self.lbl_odm_msg)
        v.addWidget(grp)

        grp_g = QGroupBox("GCP（地面控制點）")
        gf = QFormLayout(grp_g)
        self.btn_gcp_plan = QPushButton("GCP／檢核點規劃…")
        self.btn_gcp_plan.setToolTip("產生建議的 GCP 與檢核點位置（交叉航帶 vs 加密 GCP 兩方案）、匯出外業表與 gcp_list 樣板")
        self.btn_gcp_plan.clicked.connect(self.open_gcp_plan)
        gf.addRow(self.btn_gcp_plan)
        self.ed_gcp_txt = QLineEdit()
        self.ed_gcp_txt.setPlaceholderText("選填：已在 WebODM GCP 編輯器標好影像座標的 gcp_list.txt")
        self.ed_gcp_txt.textChanged.connect(lambda _: self._on_gcp_txt_changed())
        gf.addRow("gcp_list.txt", self._with_browse(self.ed_gcp_txt, "ODM GCP (*.txt)"))
        self.cmb_gcp_vertical = QComboBox()
        for text, data in GCP_VERTICAL_ITEMS:
            self.cmb_gcp_vertical.addItem(text, data)
        self.cmb_gcp_vertical.currentIndexChanged.connect(lambda _: self._update_buttons())
        gf.addRow("geo_z 高程基準", self.cmb_gcp_vertical)
        self.cmb_force_gps = QComboBox()
        for text, data in FORCE_GPS_ITEMS:
            self.cmb_force_gps.addItem(text, data)
        self.cmb_force_gps.currentIndexChanged.connect(lambda _: self._on_force_gps_changed())
        gf.addRow("照片 GPS 進平差？", self.cmb_force_gps)
        self.lbl_force_gps = QLabel("")
        self.lbl_force_gps.setWordWrap(True)
        gf.addRow(self.lbl_force_gps)
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
        self.lbl_odm_quality = QLabel("")           # ODM 完成後：品質報告檔的路徑、引擎版本（表裡是「機 = ODM」的列）
        self.lbl_odm_quality.setWordWrap(True)
        self.lbl_odm_quality.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        v.addWidget(self.lbl_odm_quality)
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
        """(ring 的串列, 來源說明, 錯誤)。計畫來源是快照 ⇒ 只用快照的作業區外框（沒載入成功就是錯誤，不退回即時狀態）；
        否則給了 ``polygons_source`` 就只用它（不退回地圖角點）。"""
        if self.plan_source_mode() == "snapshot":
            from survey import plan_snapshot as ps
            if self._plan_snapshot is None:
                return None, "飛行計畫快照", self._snapshot_missing_text()
            label = ps.polygons_label(self._plan_snapshot, self._plan_snapshot_path)
            rings = ps.snapshot_polygons(self._plan_snapshot)
            if not rings:
                return None, label, "飛行計畫快照裡沒有作業區"
            return rings, label, None
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
    # 計畫來源（即時狀態 vs 飛行計畫快照）
    # ─────────────────────────────────────────────────────────────────
    def plan_source_mode(self) -> str:
        """'live'（目前規劃器，舊行為）或 'snapshot'（飛行計畫快照）。"""
        return "snapshot" if self.rb_plan_snapshot.isChecked() else "live"

    def _snapshot_missing_text(self) -> str:
        if self._plan_snapshot_error:
            name = Path(self._plan_snapshot_path).name if self._plan_snapshot_path else ""
            return f"計畫來源選了「飛行計畫快照」，但快照 {name} 載入失敗：{self._plan_snapshot_error}"
        return "計畫來源選了「飛行計畫快照」，但還沒有載入快照（按「載入計畫快照…」，或改回「目前規劃器」）"

    def _browse_plan_snapshot(self) -> None:
        from survey import plan_snapshot as ps
        try:
            start = str(ps.default_root())
        except ps.PlanSnapshotError:
            start = ""
        p, _ = QFileDialog.getOpenFileName(self, "選飛行計畫快照", self.ed_plan_snapshot.text() or start,
                                           "飛行計畫快照 (flown_plan_*.json);;JSON (*.json)")
        if p:
            self.load_plan_snapshot(p)

    def load_plan_snapshot(self, path) -> bool:
        """讀＋嚴格驗證飛行計畫快照（survey.plan_snapshot.read_snapshot）並切到「飛行計畫快照」。

        載入失敗：大聲說（標籤＋log），計畫來源**維持**快照、不退回即時狀態 ⇒ 開始處理會被擋。"""
        from survey import plan_snapshot as ps
        p = Path(path)
        self.ed_plan_snapshot.setText(str(p))
        try:
            snap = ps.read_snapshot(p)
        except Exception as e:                       # noqa: BLE001 - 驗證失敗、讀檔失敗都照實說
            msg = str(e) if isinstance(e, ps.PlanSnapshotError) else f"{type(e).__name__}: {e}"
            self._plan_snapshot, self._plan_snapshot_path, self._plan_snapshot_error = None, p, msg
            self._log(f"✘ 飛行計畫快照 {p} 載入失敗：{msg}")
        else:
            self._plan_snapshot, self._plan_snapshot_path, self._plan_snapshot_error = snap, p, None
            self._log("飛行計畫快照：" + "；".join(ps.summary_lines(snap, p)) + f"（{p}）")
        self.rb_plan_snapshot.setChecked(True)
        self._on_plan_source_changed()
        return self._plan_snapshot is not None

    def plan_sources(self) -> Tuple[Any, Any, Optional[str]]:
        """(作業區來源, GCP 區塊來源, 錯誤)：即時 = 建構時給的 polygons／gcp_blocks；快照 = 快照的提供者。"""
        if self.plan_source_mode() == "live":
            return self.polygons_source, self.gcp_blocks_source, None
        if self._plan_snapshot is None:
            return None, None, self._snapshot_missing_text()
        from survey import plan_snapshot as ps
        return (ps.polygons_provider(self._plan_snapshot),
                ps.blocks_provider(self._plan_snapshot, self._plan_snapshot_path), None)

    def plan_polygons_label(self) -> Optional[str]:
        """給「GCP／檢核點規劃」記進 effective.areas_source 的作業區來源說明：即時 ⇒ None（它自己的「主視窗的作業區」）；
        快照 ⇒ 快照的說明（檔名、雜湊、塊數）；快照沒載入 ⇒ 照實說。"""
        if self.plan_source_mode() == "live":
            return None
        if self._plan_snapshot is None:
            return "飛行計畫快照（沒有載入成功）"
        from survey import plan_snapshot as ps
        return ps.polygons_label(self._plan_snapshot, self._plan_snapshot_path)

    def plan_source_record(self) -> Dict[str, Any]:
        """用了哪個計畫來源（給 ODM 任務紀錄；本機檔案，所以快照放完整路徑）。"""
        if self.plan_source_mode() == "live":
            return {"kind": "live", "label": PLAN_LIVE_SRC}
        if self._plan_snapshot is None:
            return {"kind": "plan_snapshot", "error": self._snapshot_missing_text()}
        from survey import plan_snapshot as ps
        rec = ps.source_record(self._plan_snapshot, self._plan_snapshot_path)
        rec.update(label=ps.source_label(self._plan_snapshot, self._plan_snapshot_path),
                   path=str(self._plan_snapshot_path))
        return rec

    def _refresh_plan_label(self) -> None:
        if self.plan_source_mode() == "live":
            self.lbl_plan_snapshot.setText(PLAN_LIVE_TEXT)
            self.lbl_plan_snapshot.setStyleSheet("")
            return
        if self._plan_snapshot is None:
            self.lbl_plan_snapshot.setText(f"✘ {self._snapshot_missing_text()}")
            self.lbl_plan_snapshot.setStyleSheet("color:#E53935;")
            return
        from survey import plan_snapshot as ps
        self.lbl_plan_snapshot.setText("\n".join(ps.summary_lines(self._plan_snapshot, self._plan_snapshot_path)))
        self.lbl_plan_snapshot.setStyleSheet("")

    def _on_plan_source_changed(self) -> None:
        self._refresh_plan_label()
        self._refresh_polygon_label()
        self._sync_gcp_dialog_sources()

    def _sync_gcp_dialog_sources(self) -> None:
        """已開著的「GCP／檢核點規劃」跟著換來源（下次按「產生」才重算）；快照沒載入 ⇒ 給會報錯的來源，不靜默用即時狀態。"""
        g = self._gcp_dialog
        if g is None:
            return
        polys, blocks, err = self.plan_sources()
        if err is not None:
            def fail(_msg=err):
                raise ValueError(_msg)
            polys = blocks = fail
        g.polygons_source, g.blocks_source = polys, blocks
        g.polygons_label = self.plan_polygons_label()
        try:
            g._refresh_sources()
        except Exception as e:                       # noqa: BLE001 - 標籤更新失敗不影響來源本身
            self._log(f"⚠ GCP 規劃對話框的來源標籤沒更新：{type(e).__name__}: {e}")

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
        from survey.pipeline import (EVENTS_POS_REFUSAL, PipelineError, SortieInput, check_geo_projection,
                                     is_events_pos)
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
            elif pos and is_events_pos(pos):
                errors.append(f"第 {r + 1} 列：{Path(pos).name}：{EVENTS_POS_REFUSAL}")
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
        # 計畫來源：快照 ⇒ 一律記（操作員明確選的；檔名＋plan／digest 雜湊，不放絕對路徑——effective 也會進 photos.meta.json）；
        # 即時狀態 ⇒ 只在它真的影響處理（以測區算視數）時記
        if self.plan_source_mode() == "snapshot":
            if self._plan_snapshot is None:
                msg = self._snapshot_missing_text()
                if msg not in errors:
                    errors.append(msg)
            else:
                from survey import plan_snapshot as ps
                sources["plan_source"] = ps.source_label(self._plan_snapshot, self._plan_snapshot_path)
                sources["plan_snapshot"] = json.dumps(ps.source_record(self._plan_snapshot, self._plan_snapshot_path),
                                                      ensure_ascii=False, sort_keys=True)
        elif "polygons" in sources:
            sources["plan_source"] = PLAN_LIVE_SRC
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
        # 基站座標兩種模式都必填：RTKLIB 模式寫進 rnx2rtkp 設定；.pos 模式用來核對 .pos 表頭的 % ref pos
        vals = {}
        for key, edit, name in (("lat", self.ed_base_lat, "基站緯度"), ("lon", self.ed_base_lon, "基站經度"),
                                ("h", self.ed_base_h, "基站橢球高"), ("ant", self.ed_base_ant, "天線高")):
            n_err = len(errors)
            vals[key] = _float_or_none(edit.text(), f"PPK：{name}", errors)
            if vals[key] is None and len(errors) == n_err:
                errors.append(f"PPK：{name}必填" + ("" if run_rtklib else "（要與解算 .pos 時輸入的一致，會核對 % ref pos）"))
        if run_rtklib:
            rinex, bubx = self.ed_base_rinex.text().strip(), self.ed_base_ubx.text().strip()
            if bool(rinex) == bool(bubx):
                errors.append("PPK：基站觀測檔要給 RINEX 或 UBX 其中一個")
            elif not Path(rinex or bubx).is_file():
                errors.append(f"PPK：基站觀測檔不存在：{rinex or bubx}")
            base_obs, base_ubx = rinex or None, bubx or None
            _, _, err = self._rtklib_paths()
            if err:
                errors.append(f"未安裝 RTKLIB：{err}")
        else:
            for s in sorties:
                if not s.pos_path:
                    errors.append(f"PPK：{s.callsign or s.session_dir} 沒有填 .pos（或改選 RTKLIB 解算）")
        if frame and all(v is not None for v in vals.values()):
            try:
                base = BaseStation(vals["lat"], vals["lon"], vals["h"], frame, antenna_height_m=vals["ant"])
            except ValueError as e:
                errors.append(f"PPK：{e}")
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

    def run_block_reason(self) -> Optional[str]:
        """「開始處理」不能按的原因（None = 可以）。ODM 等待／下載可能要好幾個小時：說清楚怎麼騰出來。"""
        if self._worker is not None:
            return "航測處理中"
        if self._odm_worker is not None:
            what = ODM_OP_TEXT.get(self._odm_op, self._odm_op)
            st = self._odm_status or {}
            task = f"任務 {st.get('uuid')}，{st.get('phase_text')}" if st.get("uuid") else "任務"
            return (f"ODM「{what}」進行中（{task}）：處理會改寫輸出資料夾與結果表，不能同時跑。要先處理別的架次："
                    "按「取消」停止本機的等待／下載——已提交的遠端任務在節點上繼續跑——處理完再用「接回既有任務…」"
                    "選原本的輸出資料夾接回（未提交的上傳停止後不能接續，要重新送出）")
        return None

    def start_processing(self) -> bool:
        from ui.workers.survey_worker import SurveyProcessWorker
        if self.is_busy():
            self._show_errors([f"不能開始處理：{self.run_block_reason()}"])
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
        if self.chk_auto_submit.isChecked() and result.passed:
            reason = self.odm_block_reason()
            if reason is None:
                self.submit_odm()
            else:                                    # 勾了自動送卻沒送：一定要說（例如 GCP 的 --force-gps 還沒選）
                msg = f"⚠ 自動送 ODM 沒有執行：{reason}"
                self._log(msg)
                self.lbl_odm_msg.setText(msg)
                self.lbl_summary.setText(self.lbl_summary.text() + "\n" + msg)

    def _set_qa_row(self, i: int, row) -> None:
        uav, item, value, ok = row
        mark = "—" if ok is None else ("✔ 通過" if ok else "✘ 未通過")
        for c, txt in enumerate((uav, item, value, mark)):
            it = QTableWidgetItem(str(txt))
            if c == 3 and ok is False:
                it.setForeground(Qt.GlobalColor.red)
            self.qa_table.setItem(i, c, it)

    def _fill_results(self, result) -> None:
        from survey.pipeline import qa_table_rows
        rows = qa_table_rows(result)
        self.qa_table.setRowCount(len(rows))
        for i, row in enumerate(rows):
            self._set_qa_row(i, row)
        head = "QA 通過" if result.passed else f"QA 未通過（{len(result.qa['reasons'])} 項）"
        lines = [f"{head}｜照片 {len(result.rows)} 張｜加標籤影像 {len(result.images)} 張｜輸出 {result.out_dir}"]
        eff = result.effective or {}
        if eff.get("gps_lag_s") is not None:                      # 有效值（流水線實際用的），不是欄位文字
            src = (eff.get("input_sources") or {}).get("gps_lag_s", "未記錄來源")
            lines.append(f"GPS 接收延遲補償 {float(eff['gps_lag_s']) * 1000.0:g} ms（{src}）")
        if eff.get("polygons"):
            lines.append(f"視數測區：{len(eff['polygons'])} 塊")
        plan_src = (eff.get("input_sources") or {}).get("plan_source")
        if plan_src:
            lines.append(f"計畫來源：{plan_src}")
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
            if self.cmb_force_gps.currentData() is None:
                return ("選了 GCP 檔，但還沒決定照片 GPS（PPK）要不要一起進平差（--force-gps）：沒有預設，"
                        "請在 GCP 區明確選一個")
        return None

    def remote_cancel_state(self) -> Tuple[bool, Optional[str]]:
        """「取消遠端任務」能不能按 → (可以, 不行的原因)。"""
        if self._odm_op == "cancel_remote":
            return False, "正在取消遠端任務"
        if self._pending_remote_cancel is not None:
            return False, "已要求取消：等本機的 ODM 工作停下來就送出"
        st = self._odm_status
        if not st or not st.get("uuid"):
            return False, "沒有已送出的 ODM 任務（先送出，或「接回既有任務…」）"
        if st.get("can_cancel_remote") or st.get("phase") in ODM_UNCOMMITTED_CANCELABLE:
            return True, None
        return False, f"任務 {st.get('uuid')} {st.get('phase_text')}：沒有可以取消的遠端任務"

    def _update_buttons(self) -> None:
        busy = self.is_busy()
        self.btn_run.setEnabled(not busy)
        self.btn_run.setToolTip(self.run_block_reason() or "")
        self.btn_cancel.setEnabled(busy)
        reason = self.odm_block_reason()
        self.btn_odm.setEnabled(reason is None)
        self.btn_odm.setToolTip(reason or "分批上傳加標籤影像與 geo.txt 到 NodeODX（init → upload → commit）")
        idle = self._odm_worker is None
        self.btn_odm_test.setEnabled(idle)
        self.btn_odm_reattach.setEnabled(idle and self._worker is None)
        ok, why = self.remote_cancel_state()
        self.btn_odm_cancel_remote.setEnabled(ok)
        self.btn_odm_cancel_remote.setToolTip(why or "確認後要求節點停止這個任務（POST /task/cancel）")

    # ── 選項 ────────────────────────────────────────────────────────────
    def _on_gcp_txt_changed(self) -> None:
        self._refresh_force_gps_label()
        self._update_buttons()

    def _on_force_gps_changed(self) -> None:
        self._refresh_force_gps_label()
        self._update_buttons()

    def force_gps_choice(self) -> Optional[bool]:
        """送給 odm_job 的 force_gps：沒有 GCP ⇒ None（不適用，流水線記 not_applicable_no_gcp）；有 GCP ⇒ 操作員的選擇。"""
        if not self.ed_gcp_txt.text().strip():
            return None
        return self.cmb_force_gps.currentData()

    def _refresh_force_gps_label(self) -> None:
        has_gcp = bool(self.ed_gcp_txt.text().strip())
        self.cmb_force_gps.setEnabled(has_gcp)
        if not has_gcp:
            self.lbl_force_gps.setText(FORCE_GPS_NA_TEXT)
            self.lbl_force_gps.setStyleSheet("")
            return
        v = self.cmb_force_gps.currentData()
        self.lbl_force_gps.setText(FORCE_GPS_TEXT[v])
        self.lbl_force_gps.setStyleSheet("color:#E53935;" if v is None else "")

    def max_concurrency_choice(self) -> Optional[int]:
        """None ＝ 不送（引擎預設）；否則送的整數。"""
        v = int(self.spn_max_conc.value())
        return v if v > 0 else None

    def _refresh_max_conc_label(self) -> None:
        v = self.max_concurrency_choice()
        eff = "有效值：不送（引擎預設＝節點全部核心）" if v is None else f"有效值：送 --max-concurrency {v}（操作員設定）"
        self.lbl_max_conc.setText(f"{eff}。{MAX_CONC_HINT}")

    # ── 節點 ────────────────────────────────────────────────────────────
    def _odm_client_for(self, url: str):
        """客戶端：測試注入的 ``odm_client`` 優先；否則用位址＋「權杖」欄建（權杖只活在客戶端物件裡）。
        位址不合法（帶查詢字串、帳密…）⇒ ValueError（訊息不回顯查詢字串）。"""
        if self.odm_client is not None:
            return self.odm_client
        token = self.ed_odm_token.text().strip() or None
        if self.odm_client_factory is not None:
            return self.odm_client_factory(url, token)
        from survey.odm_client import NodeOdmClient
        return NodeOdmClient(url, token=token)

    def test_odm_connection(self) -> bool:
        """「測試連線」：背景查 GET /info ＋ GET /options，顯示節點與引擎版本（不建立任務）。"""
        if self._odm_worker is not None:
            self._show_errors(["ODM 還有工作在跑，等它結束再測試連線"])
            return False
        try:
            client = self._odm_client_for(self.ed_odm_url.text())
        except ValueError as e:
            self.lbl_odm_node.setText(f"✘ {e}")
            self.lbl_odm_node.setStyleSheet("color:#E53935;")
            return False
        self.lbl_odm_node.setText(f"查詢 {client.base_url} …")
        self.lbl_odm_node.setStyleSheet("")
        return self._start_odm_worker("probe", {"node_url": client.base_url}, _probe_node, client,
                                      self._on_odm_probe_ok)

    def _on_odm_probe_ok(self, res: dict) -> None:
        self._end_odm_worker()
        self._odm_node = res
        txt = odm_node_text(res.get("info") or {}, res.get("options"), res.get("options_error"))
        self.lbl_odm_node.setText(f"{res.get('node_url')}：{txt}")
        self.lbl_odm_node.setStyleSheet("")
        self._log(f"節點：{res.get('node_url')}（權杖 {'有' if res.get('token_used') else '沒有'}）：{txt}")
        self._after_odm_worker()

    # ── 送出、接回、取消 ────────────────────────────────────────────────
    def submit_odm(self) -> bool:
        from survey.pipeline import PipelineError, odm_job
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
                          rolling_shutter=self.chk_rolling.isChecked(), qa_override=override,
                          force_gps=self.force_gps_choice(), max_concurrency=self.max_concurrency_choice(), **gcp_kw)
        except PipelineError as e:
            self._show_errors([str(e)])
            if gcp:
                self.lbl_gcp.setText(f"✘ {e}")
                self.lbl_gcp.setStyleSheet("color:#E53935;")
            return False
        try:
            client = self._odm_client_for(job["node_url"])
        except ValueError as e:
            self._show_errors([str(e)])
            return False
        job["orthophoto_resolution_source"] = "user" if self._res_from_user else "gsd"
        # ODM 本身不用計畫多邊形；記下送出時對話框選的計畫來源（處理時用的在 qa.json 的 effective.input_sources）
        job["plan_source"] = dict(self.plan_source_record(),
                                  note="送出時對話框選的計畫來源；處理（視數）用的記在 qa.json 的 effective.input_sources")
        self.result.effective["odm_submission"] = {k: job[k] for k in (
            "node_url", "options", "qa_passed", "qa_override", "qa_reasons", "rolling_shutter",
            "orthophoto_resolution_source", "force_gps", "max_concurrency", "gps_accuracy", "plan_source")}
        self.result.effective["odm_submission"]["token_used"] = bool(getattr(client, "token", None))
        if "gcp" in job:
            self.result.effective["odm_submission"].update(gcp_txt=job["gcp_txt"], gcp=job["gcp"])
            g = job["gcp"]
            self.lbl_gcp.setText(f"GCP：{g['n_gcps']} 個、{g['n_lines']} 行；標頭 {g['header']}；geo_z "
                                 f"{'橢球高' if g['gcp_vertical'] == 'ellipsoidal' else '正高'}"
                                 + ("；⚠ " + "；".join(g["warnings"][:3]) if g["warnings"] else ""))
            self.lbl_gcp.setStyleSheet("")
        self._log("送 ODM：" + json.dumps(self.result.effective["odm_submission"], ensure_ascii=False, default=str))
        self._odm_dir = Path(job["out_dir"])
        self._odm_status = None
        self._odm_last_phase = None
        self.odm_progress.setValue(0)
        self.lbl_odm_quality.setText("")
        self._refresh_odm_task_label()
        return self._start_odm_worker("submit", job, self.odm_run_fn, client, self._on_odm_finished)

    def _browse_reattach(self) -> None:
        start = self.ed_out_dir.text().strip() or (str(self._odm_dir) if self._odm_dir else "")
        p = QFileDialog.getExistingDirectory(self, "選送過 ODM 的處理輸出資料夾（裡面有 odm/submission.json）", start)
        if p:
            self.reattach_odm(p)

    def reattach_odm(self, where) -> bool:
        """「接回既有任務」：讀 ``where``（輸出資料夾、其下 odm/、或 submission.json）的任務紀錄，用同一個節點
        （位址取自紀錄、權杖取自欄位）繼續等待／下載／品質報告。不需要本次的處理結果。"""
        from survey.pipeline import PipelineError, load_submission, odm_submission_status, resume_odm_job, \
            submission_path
        if self._odm_worker is not None:
            self._show_errors(["ODM 還有工作在跑，等它結束（或取消）再接回"])
            return False
        if self._worker is not None:                  # 處理可能正在（覆寫）同一個輸出資料夾
            self._show_errors(["航測處理中：等處理結束再接回 ODM 任務"])
            return False
        try:
            path = submission_path(where)
            st = odm_submission_status(load_submission(path))
        except PipelineError as e:
            self._show_errors([f"接回任務：{e}"])
            self._log(f"✘ 接回任務：{e}")
            return False
        out_dir = path.parent.parent
        prev_odm = (self._odm_dir, self._odm_status, self._odm_status_error, self._odm_last_phase)
        self._odm_dir, self._odm_status, self._odm_status_error = out_dir, st, None
        self._odm_last_phase = st.get("phase")
        self._refresh_odm_task_label()
        self._update_buttons()
        errors = []
        if not st.get("uuid"):
            errors.append(f"任務紀錄 {path} 沒有 uuid：任務沒有建立過，請重新送出")
        elif st.get("phase") in ODM_NOT_REATTACHABLE:
            errors.append(f"任務 {st['uuid']} {st['phase_text']}，不能接回：{st.get('reason')}"
                          + (f"（{st['failure']}）" if st.get("failure") else ""))
        token = self.ed_odm_token.text().strip()
        if st.get("token_used") and not token and self.odm_client is None:
            errors.append("這個任務送出時用了權杖：請先在「權杖」欄填同一個權杖（權杖不存檔，所以要再填一次）")
        url = st.get("node_url") or ""
        if not errors:
            try:
                client = self._odm_client_for(url)
            except ValueError as e:
                errors.append(f"任務紀錄的節點位址不能用：{e}")
        if errors:
            self._show_errors(errors)
            for e in errors:
                self._log(f"✘ 接回任務：{e}")
            return False
        cur = self.ed_out_dir.text().strip() or (str(self.result.out_dir) if self.result is not None else "")
        if cur and not _same_dir(cur, out_dir):
            # 對話框還留著另一個架次（A）的輸出資料夾／處理結果／DSM：接回 B 會把 B 的成果和 A 的證據混在一起
            # （檢核點報告寫到 A、用 A 的 DSM、B 的品質列接在 A 的 QA 表後面）⇒ 先問，確定才整組換成 B（審查）
            text = (f"這個對話框目前的輸出資料夾是\n  {cur}\n要接回的任務在另一個資料夾\n  {out_dir}\n\n"
                    "「是」＝換成接回的資料夾：清掉目前的處理結果、QA 表、DSM 與它的高程基準（成果完成後改用接回任務的"
                    " DSM，高程基準要重新選）、檢核點報告；實測檢核點等 CSV 欄位保留（請確認是同一個場地）。\n"
                    "「否」＝不接回。")
            if not self._confirm("接回另一個輸出資料夾的任務", text):
                self._odm_dir, self._odm_status, self._odm_status_error, self._odm_last_phase = prev_odm
                self._refresh_odm_task_label()
                self._update_buttons()
                self._log(f"接回任務：操作員放棄（目前的輸出資料夾 {cur} 與任務的 {out_dir} 不同）")
                return False
            self._switch_campaign(out_dir, cur)
        if self.ed_odm_url.text().strip() != url:
            self.ed_odm_url.setText(url)
            self._log(f"節點位址改成任務紀錄的 {url}（接回只能連送出時的同一個節點）")
        if not self.ed_out_dir.text().strip():
            self.ed_out_dir.setText(str(out_dir))           # 檢核點報告寫在這裡（checkpoints/）
            self._log(f"輸出資料夾帶入接回的 {out_dir}")
        self._show_errors([])
        self.lbl_odm_quality.setText("")
        self.odm_progress.setValue(0)
        self._log(f"接回 ODM 任務 {st['uuid']}（{st['phase_text']}；紀錄 {path}；權杖 {'有' if token else '沒有'}）")
        self.tabs.setCurrentIndex(2)
        return self._start_odm_worker("reattach", {"record_path": str(path)}, self.odm_resume_fn or resume_odm_job,
                                      client, self._on_odm_finished)

    def _switch_campaign(self, out_dir, old) -> None:
        """接回另一個輸出資料夾的任務（操作員確認過）：整組換成那個資料夾，不留上一個架次的結果與 DSM。"""
        kept = [n for n, e in (("實測檢核點", self.ed_chk_survey), ("正射量測", self.ed_chk_measured),
                               ("佈設", self.ed_chk_layout)) if e.text().strip()]
        old_dsm = self.ed_dsm.text().strip()
        dsm_kind = "自動帶入的 " if old_dsm and old_dsm == self._dsm_autofilled else "操作員選的 "
        self.ed_out_dir.setText(str(out_dir))
        self.result = None
        self.odm_result = None
        self.qa_table.setRowCount(0)
        self.lbl_summary.setText(f"已換成接回任務的輸出資料夾 {out_dir}（{old} 的處理結果已從畫面清掉，檔案不動）")
        self.ed_dsm.setText("")
        self.cmb_dsm_vertical.setCurrentIndex(0)
        self._dsm_vertical_source = None
        self._dsm_autofilled = None
        self.lbl_dsm_vertical.setText("")
        self.checkpoint_report = None
        self.chk_report_view.setPlainText("")
        self.lbl_odm_quality.setText("")
        self._log(f"輸出資料夾換成接回的 {out_dir}（原本 {old}）：清掉處理結果、QA 表"
                  + (f"、{dsm_kind}DSM {old_dsm}" if old_dsm else "") + "、DSM 高程基準、檢核點報告"
                  + (f"；保留 {('、'.join(kept))} CSV 欄位（請確認是同一個場地）" if kept else ""))
        self._update_buttons()

    def _confirm(self, title: str, text: str) -> bool:
        """是／否確認（預設「否」）；測試換掉這支。"""
        r = QMessageBox.question(self, title, text, QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                                 QMessageBox.StandardButton.No)
        return r == QMessageBox.StandardButton.Yes

    def cancel_remote_odm(self, where=None, *, confirm: bool = True) -> bool:
        """「取消遠端任務」：確認後要求節點停止任務（survey.pipeline.cancel_odm_job：POST /task/cancel，等節點回報 50）。
        本機有 ODM 工作在跑（送出／接回）⇒ 先停它（兩邊寫同一份紀錄），停下來後才送取消。"""
        from survey.pipeline import PipelineError, submission_path
        if where is not None:
            try:
                self._odm_dir = submission_path(where).parent.parent
            except (OSError, ValueError) as e:
                self._show_errors([f"取消遠端任務：{e}"])
                return False
        self._refresh_odm_status()
        ok, why = self.remote_cancel_state()
        st = self._odm_status or {}
        if ok and st.get("token_used") and not self.ed_odm_token.text().strip() and self.odm_client is None:
            ok, why = False, "這個任務送出時用了權杖：請先在「權杖」欄填同一個權杖"
        if not ok:
            self._show_errors([f"不能取消遠端任務：{why}"])
            return False
        text = (f"取消 NodeODX（{st.get('node_url')}）上的任務 {st.get('uuid')}（{st.get('phase_text')}）？\n\n"
                "節點會停止處理（POST /task/cancel）；已處理的進度不能接續、成果不會下載。"
                "本機的 odm/submission.json 會記下取消。")
        if st.get("phase") in ODM_UNCOMMITTED_CANCELABLE:
            text += ("\n\n這個任務還沒提交（上傳中）：NodeODX 沒有刪除未提交上傳的 API，節點會在 cleanupUploadsAfter"
                     "（預設 48 h）後自己清掉暫存。")
        if self._odm_worker is not None:
            text += f"\n\n本機正在「{ODM_OP_TEXT.get(self._odm_op, self._odm_op)}」：會先停止它，再送取消。"
        if confirm and not self._confirm("取消遠端任務", text):
            self._log("取消遠端任務：操作員放棄")
            return False
        try:
            client = self._odm_client_for(st.get("node_url") or "")
        except ValueError as e:
            self._show_errors([f"取消遠端任務：{e}"])
            return False
        try:
            job = {"record_path": str(submission_path(self._odm_dir))}
        except PipelineError as e:
            self._show_errors([f"取消遠端任務：{e}"])
            return False
        self._show_errors([])
        if self._odm_worker is not None:
            self._pending_remote_cancel = (job, client)
            self._odm_worker.cancel()
            msg = f"取消遠端任務：先停止本機的「{ODM_OP_TEXT.get(self._odm_op, self._odm_op)}」，停下來後立刻送取消…"
            self.lbl_progress.setText(msg)
            self._log(msg)
            self._update_buttons()
            return True
        return self._start_remote_cancel(job, client)

    def _start_remote_cancel(self, job, client) -> bool:
        self._log(f"取消遠端任務：送出（紀錄 {job['record_path']}）")
        return self._start_odm_worker("cancel_remote", job, self.odm_cancel_fn or _cancel_remote_job, client,
                                      self._on_odm_cancel_done, poll_s=ODM_CANCEL_POLL_S)

    def _on_odm_cancel_done(self, res: dict) -> None:
        self._end_odm_worker()
        msg = res.get("message") or "已送出取消"
        self.lbl_progress.setText(msg)
        self.lbl_odm_msg.setText(msg)
        self._log(f"取消遠端任務：{msg}（階段 {res.get('phase')}，節點狀態 {res.get('status_code')}）")
        self._refresh_odm_status()
        self._after_odm_worker()

    # ── worker 共用 ─────────────────────────────────────────────────────
    def _start_odm_worker(self, op: str, job: dict, run_fn, client, on_ok, *, poll_s: Optional[float] = None) -> bool:
        from ui.workers.survey_worker import OdmSubmitWorker
        w = OdmSubmitWorker(job, run_fn=run_fn, client=client,
                            poll_s=self.odm_poll_s if poll_s is None else poll_s, parent=self)
        w.progress.connect(self._on_odm_progress)
        w.state.connect(self._on_odm_state)
        w.finished_ok.connect(on_ok)
        w.failed.connect(self._on_odm_failed)
        w.canceled.connect(self._on_odm_canceled)
        self._odm_worker, self._odm_op = w, op
        self._odm_last_log_key = None
        self._update_buttons()
        w.start()
        return True

    def _end_odm_worker(self) -> None:
        if self._odm_worker is not None:
            self._odm_worker.wait()
        self._odm_worker = None
        self._odm_op = None
        self._update_buttons()

    def _after_odm_worker(self) -> None:
        """一個 ODM 工作結束後：有等著的「取消遠端任務」就現在送。"""
        pend, self._pending_remote_cancel = self._pending_remote_cancel, None
        if pend is not None and self._odm_worker is None:
            self._refresh_odm_status()
            self._start_remote_cancel(*pend)
        else:
            self._update_buttons()

    def _on_odm_progress(self, msg: str, frac: float) -> None:
        self.lbl_progress.setText(msg)
        if frac >= 0:
            v = int(round(frac * 1000))
            self.progress_bar.setValue(v)
            self.odm_progress.setValue(v)
        # 上傳／下載的進度每 0.5 s 一則：log 只在訊息「種類」變了才寫（數字換成 # 比對），不洗版
        key = re.sub(r"\d+(?:\.\d+)?", "#", msg)
        if key != self._odm_last_log_key:
            self._odm_last_log_key = key
            self.log_view.appendPlainText(msg)

    def _on_odm_state(self, st: dict) -> None:
        frac = st.get("frac")
        if frac is not None:
            self.odm_progress.setValue(int(round(float(frac) * 1000)))
        if st.get("message"):
            self.lbl_odm_msg.setText(str(st["message"]))
        phase = st.get("phase")
        if phase != self._odm_last_phase:              # 階段換了 ⇒ 重讀紀錄（引擎版本、遠端狀態）
            self._odm_last_phase = phase
            self._refresh_odm_status()

    def _on_odm_failed(self, msg: str) -> None:
        op = self._odm_op
        self._end_odm_worker()
        what = ODM_OP_TEXT.get(op, "ODM")
        if op == "probe":
            self.lbl_odm_node.setText(f"✘ 連不上節點：{msg}")
            self.lbl_odm_node.setStyleSheet("color:#E53935;")
        self._show_errors([f"ODM {what}：{msg}"])
        self._log(f"✘ ODM {what}失敗：{msg}")
        self.lbl_odm_msg.setText(f"✘ {msg}")
        self._refresh_odm_status()
        self._after_odm_worker()

    def _on_odm_canceled(self) -> None:
        w, op = self._odm_worker, self._odm_op
        msg = (getattr(w, "cancel_message", None) if w is not None else None) or "已停止"
        self._end_odm_worker()
        self.lbl_progress.setText(msg)
        self.lbl_odm_msg.setText(msg)
        self._log(f"ODM {ODM_OP_TEXT.get(op, '')}：{msg}")
        self._refresh_odm_status()
        self._after_odm_worker()

    def _on_odm_finished(self, info: dict) -> None:
        self._end_odm_worker()
        self.odm_result = info
        tiles = info.get("ortho_tiles")
        self.odm_progress.setValue(1000)
        self._log(f"ODM 完成：任務 {info.get('uuid')}，成果 {info.get('extract_dir')}")
        for w in info.get("warnings") or []:
            self._log(f"⚠ ODM：{w}")
        self._show_odm_quality(info)
        self._refresh_odm_status()
        self._after_odm_checkpoints(info)
        self._overlay_ortho(tiles)
        self._after_odm_worker()

    def _overlay_ortho(self, tiles) -> None:
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

    def _show_odm_quality(self, info: dict) -> None:
        """引擎的品質報告摘要 → 「結果」頁表格（機 = ODM 的列，與 QA 同格式）＋報告檔路徑。沒有就照實說。"""
        from survey.pipeline import odm_quality_rows
        q = info.get("quality")
        rows = info.get("quality_rows")
        if rows is None and isinstance(q, dict):
            rows = odm_quality_rows(q)
        if rows is None:
            return                                   # 假的 run_fn／舊介面：沒有品質資料（不是缺檔）
        for r in reversed(range(self.qa_table.rowCount())):      # 換掉上一次的 ODM 列
            it = self.qa_table.item(r, 0)
            if it is not None and it.text() == "ODM":
                self.qa_table.removeRow(r)
        base = self.qa_table.rowCount()
        self.qa_table.setRowCount(base + len(rows))
        for i, row in enumerate(rows):
            self._set_qa_row(base + i, row)
        q = q or {}
        files = q.get("files") or {}
        ext = info.get("extract_dir")
        lines = [f"ODM 品質報告（任務 {info.get('uuid')}；引擎自己的數字，沒有我們自訂的門檻）："]
        for key, name in (("report_pdf", "report.pdf"), ("stats_json", "stats.json"), ("shots_geojson", "shots.geojson")):
            rel = files.get(key)
            lines.append(f"  {name}：{Path(ext) / rel if (rel and ext) else '⚠ 沒有'}")
        rec = info.get("record_path")
        if rec:
            lines.append(f"  品質摘要：{Path(rec).parent / 'quality_summary.json'}；任務紀錄：{rec}")
        eng, node = q.get("engine") or {}, info.get("node") or {}
        lines.append(f"  引擎：log.json {eng.get('engine') or '?'} {eng.get('version') or '?'}；"
                     f"節點 /info engineVersion {node.get('engineVersion') or eng.get('node_engine_version') or '?'}")
        n_warn = len(q.get("warnings") or [])
        if n_warn:
            lines.append(f"  ⚠ 品質報告 {n_warn} 則警告（見上表「警告」列）")
        self.lbl_odm_quality.setText("\n".join(lines))
        self._log("\n".join(lines))
        self.tabs.setCurrentIndex(3)

    # ── 任務狀態（odm/submission.json） ─────────────────────────────────
    def _refresh_odm_status(self) -> None:
        """重讀目前 ODM 任務的紀錄 → 狀態摘要（給標籤與按鈕）。紀錄是背景執行緒以原子取代寫的，讀到的一定完整。"""
        from survey.pipeline import PipelineError, load_submission, odm_submission_status
        st, err = None, None
        if self._odm_dir is not None:
            try:
                st = odm_submission_status(load_submission(self._odm_dir))
            except PipelineError as e:
                err = str(e)
        self._odm_status, self._odm_status_error = st, err
        self._refresh_odm_task_label()
        self._update_buttons()

    def _refresh_odm_task_label(self) -> None:
        st = self._odm_status
        if not st:
            self.lbl_odm_task.setText(f"⚠ {self._odm_status_error}" if self._odm_status_error
                                      else "沒有 ODM 任務（送出後，或「接回既有任務…」）")
            return
        prog = st.get("progress")
        try:
            prog_txt = f" {float(prog):.0f} %" if prog is not None else ""
        except (TypeError, ValueError):
            prog_txt = f" {prog}"
        parts = [f"任務 {st.get('uuid') or '（尚未建立）'}", st.get("phase_text") or "?",
                 f"節點 {st.get('status_text') or '—'}{prog_txt}",
                 f"引擎 ODX {st.get('engine_version') or '?'}",
                 f"{st.get('n_images') if st.get('n_images') is not None else '?'} 張",
                 f"權杖 {'有' if st.get('token_used') else '沒有'}", f"更新 {st.get('updated_at') or '—'}"]
        txt = "｜".join(parts)
        if st.get("failure"):
            txt += f"\n✘ {st['failure']}"
        elif st.get("reason") and not st.get("can_reattach") and self._odm_worker is None:
            txt += f"\n{st['reason']}"                 # 「能不能接回」的說明；正在上傳時不顯示（會誤導）
        self.lbl_odm_task.setText(txt)

    # ── 關主視窗用 ──────────────────────────────────────────────────────
    def busy_description(self) -> str:
        parts = []
        if self._worker is not None:
            parts.append("航測處理（照片位姿／PPK／QA）")
        if self._odm_worker is not None:
            st = self._odm_status or {}
            parts.append(f"ODM {ODM_OP_TEXT.get(self._odm_op, '')}"
                         + (f"（任務 {st.get('uuid')}，{st.get('phase_text')}）" if st.get("uuid") else ""))
        if self._pending_remote_cancel is not None:
            parts.append("已確認、還沒送出的「取消遠端任務」（等本機工作停下來才送）")
        return "、".join(parts) or "沒有"

    def pending_remote_cancel_text(self) -> Optional[str]:
        """操作員已確認、但還在等本機工作停下來才送的「取消遠端任務」（關主視窗時要講清楚它不會送出）。"""
        if self._pending_remote_cancel is None:
            return None
        job = self._pending_remote_cancel[0] or {}
        st = self._odm_status or {}
        return (f"你已確認要取消遠端任務 {st.get('uuid') or ''}（紀錄 {job.get('record_path')}），但取消請求還沒送出"
                "（在等本機的 ODM 工作停下來）")

    def shutdown_workers(self, timeout_ms: int = 10000) -> list:
        """主視窗關閉：要求所有背景工作停止、合計最多等 ``timeout_ms``，拆掉它們連到本對話框的信號。

        回傳還沒停下來的 QThread：已 ``setParent(None)``（否則對話框被刪時會解構一個還在跑的 QThread ⇒ qFatal），
        呼叫端要負責留住參考直到它結束（主視窗寄放到模組層級）。ODM 已提交的遠端任務不受影響，可以之後接回。
        已確認、還沒送出的「取消遠端任務」**不會**送出：照實記 log（主視窗的關閉提問已先講過）。"""
        pend = self.pending_remote_cancel_text()
        if pend:
            msg = f"⚠ 關閉：{pend}——不會送出，遠端任務會繼續跑；要取消請重開後用「取消遠端任務」"
            logger.warning(f"[航測處理] {msg}")
            try:
                self._log(msg)
            except RuntimeError:
                pass
        self._pending_remote_cancel = None
        workers = [w for w in (self._worker, self._odm_worker) if w is not None]
        for w in workers:
            w.cancel()
        deadline = time.monotonic() + max(0, int(timeout_ms)) / 1000.0
        left = []
        for w in workers:
            remain = max(0, int((deadline - time.monotonic()) * 1000))
            try:
                done = (not w.isRunning()) or w.wait(remain)
            except RuntimeError:                      # C++ 物件已經不在
                continue
            for sig in ("progress", "state", "finished_ok", "failed", "canceled"):
                try:
                    getattr(w, sig).disconnect()
                except (TypeError, RuntimeError, AttributeError):
                    pass                              # 本來就沒接／沒有這個信號
            if not done:
                w.setParent(None)
                left.append(w)
        self._worker = self._odm_worker = None
        self._odm_op = None
        try:
            self._update_buttons()
        except RuntimeError:
            pass
        if left:
            logger.warning(f"[航測處理] 關閉時還有 {len(left)} 個背景工作沒停下來（{timeout_ms} ms 內），交給主視窗寄放")
        return left

    # ─────────────────────────────────────────────────────────────────
    # GCP 規劃、檢核點精度
    # ─────────────────────────────────────────────────────────────────
    def open_gcp_plan(self):
        """開「GCP／檢核點規劃」（非模態；同一個對話框重複開只是叫到前面）。作業區與 UAV 區塊跟著「計畫來源」：
        即時 ⇒ 建構時給的提供者；飛行計畫快照 ⇒ 快照裡的（快照沒載入成功 ⇒ 拒絕開，不靜默改用即時狀態）。"""
        from ui.dialogs.gcp_plan_dialog import GcpPlanDialog
        polys, blocks, err = self.plan_sources()
        if err is not None:
            self._show_errors([err])
            return None
        if self._gcp_dialog is None:
            self._gcp_dialog = GcpPlanDialog(parent=self, map_widget=self.map_widget, polygons=polys, blocks=blocks)
        else:
            self._gcp_dialog.polygons_source, self._gcp_dialog.blocks_source = polys, blocks
        self._gcp_dialog.polygons_label = self.plan_polygons_label()   # effective.areas_source 要寫真的來源
        try:
            self._gcp_dialog._refresh_sources()
        except Exception as e:                       # noqa: BLE001 - 標籤更新失敗不影響來源本身
            self._log(f"⚠ GCP 規劃對話框的來源標籤沒更新：{type(e).__name__}: {e}")
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
                self._dsm_autofilled = str(dsm)
                self._log(f"DSM：{dsm}")
        if self.ed_chk_survey.text().strip():
            if self.compute_checkpoint_report() is not None:
                self.tabs.setCurrentIndex(4)

    # ─────────────────────────────────────────────────────────────────
    def showEvent(self, event):                      # noqa: N802 - Qt 介面
        self._refresh_polygon_label()                # 非模態：主視窗的作業區可能在對話框關著時變了
        if self._odm_worker is None and self._odm_dir is not None:
            self._refresh_odm_status()               # 關著時任務紀錄可能被別處（另一次接回）改過
        super().showEvent(event)

    def _close_requested(self) -> bool:
        """Esc／視窗的關閉鈕 → 可以關（隱藏）嗎？

        * 航測處理在跑：照舊——要求取消處理、不關（輸出資料夾正在寫）。
        * 只有 ODM 工作在跑（上傳／等待／下載／接回／取消遠端）：**只隱藏**，工作在背景繼續——未提交的上傳一旦中斷
          就不能接續（2,500 張要整批重傳），不能讓一個 Esc 就把它取消（審查）。再開「航測處理…」會叫回同一個對話框
          看進度；要停就按「取消」。主視窗關閉時另有 survey_close_decision 處理。"""
        if self._worker is not None:
            self._worker.cancel()
            self.lbl_progress.setText("取消中（等目前的步驟結束）…")
            self._show_errors(["處理中：已要求取消處理，等目前的步驟結束後再關閉"])
            return False
        if self._odm_worker is not None:
            what = ODM_OP_TEXT.get(self._odm_op, self._odm_op)
            self._log(f"對話框關閉（隱藏）：ODM「{what}」在背景繼續；再開「航測處理…」看進度，要停止請按「取消」")
        return True

    # 視窗的關閉鈕：QDialog.closeEvent 會呼叫 reject()（Qt 的虛擬函式 ⇒ 就是下面這支）；reject 沒有隱藏對話框時
    # QDialog 自己會忽略關閉事件 ⇒ 不另外覆寫 closeEvent（否則同一次關閉會判斷、記 log 兩次）
    def reject(self):
        if not self._close_requested():
            return
        super().reject()
