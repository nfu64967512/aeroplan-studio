"""航測架次後處理流水線：每架的飛控 .BIN ＋ 樹莓派會期（＋相機模組 F9P 的 PPK）→ I1 交付物、geo.txt、
加標籤影像、QA，以及可選的 NodeODM 任務。

串起第一階段的各模組（評估 §8 流程 1–6），本模組只做「接線」與「合併規則」，不重算任何東西::

    ingest.ingest_sortie                    # 每架：配對、時鐘擬合、FC 內插位姿（fc_interp）、現場 QA
    （PPK）ubx.read_ubx_file → events.extract_edges → events.pair_with_frames
           → rtklib.run_ppk 或使用者給的 .pos → pos.read_pos
           → interp.interpolate_at（槓桿臂 gnss_lever_arm_m、姿態取同一曝光時刻的 FC 姿態）
           → outputs.ppk_photo_rows
    合併（依 seq）→ deliverable.write_i1_deliverable → geotag.write_geo_txt
    → exif_write.export_tagged_images（與 geo.txt 同一個高度選擇）
    → 視數（core.recon.exposure_verifier，有測區多邊形時；否則 survey.qa 的足跡格網）

合併規則（有效值契約）
======================
* PPK 只在「前後 epoch 都是 fix、沒有缺口、槓桿臂已知、有姿態、時刻不是外插」時取代 fc_interp 的位置、
  速度、GPST 與 σ；``source='ppk'``、``crs_h`` = 基站框架名稱（例如 e-GNSS[2025]）。
* 其他照片保留 fc_interp，旗標加 ``ppk_rejected`` 與原因（``ppk_float``…）；完全沒有 PPK 事件的加
  ``ppk_missing``；整架 PPK 失敗的加 ``ppk_unavailable``，並列為 QA 未通過。
* PPK 數值鎖（2026-10-21，RTKLIB 與既有 .pos 兩條路線都套；不通過 ⇒ 整架 PPK 失敗）：
  .pos 表頭 ``% ref pos`` 必須等於填的基站座標（M2b，各 5 mm）；``% obs start／end`` 必須落在 rover.ubx
  的 RAWX 跨度內、第一筆解不能早於 obs start（M2a，抓時間偏 18 s 與別架的 .pos）；已採用 PPK 的照片
  與 fc_interp 的中位數差超過 ``fc_gross_h_m／fc_gross_v_m``（M3，提案 15 m）⇒ 該架 QA 未通過。
  Emlid Studio 的 ``_events.pos`` 直接拒絕（要給整條軌跡）。
* 框架不混用：只要有任何一列不是 EPSG:4326，就必須明確給 ``geo_projection``（PROJ 可解析的**經緯度**
  座標系；本流程寫的是經度／緯度，不做投影換算），否則 geo.txt 與 EXIF 影像都不寫、記成錯誤。給了的話
  所有列以該框架寫出，各列原本的框架與張數寫進 warnings 與 qa.json。
* SensorTimestamp 語意（台架 B3 未定案）沒有預設值；大地起伏未知時正高留空（不當 0）。

風（2026-10-15 使用者決定：首次建圖飛行只在風速 ≤ 3 m/s）
=========================================================
每架用飛控 EKF 的風估計（XKF2／NKF2）做逐帶風速、風向與「超過上限的照片比例」（survey.qa.wind_report）。
超過上限是**旗標**：寫進 warnings 與 QA 表，不改變 QA 通過與否；上限 ``wind_limit_mps``（預設 3 m/s）是有效值，
記在 qa.json 的 ``effective.wind_qa``（不寫進 photos.meta.json：風不影響照片位姿，I1 交付物與加入這個功能前逐位元相同）。

GCP（ODM gcp_list.txt）
=======================
``odm_job(..., gcp_txt=, gcp_vertical=)`` 把使用者給的 gcp_list.txt（已標好影像座標）一起送 NodeODM：
先用 survey.gcp_plan.validate_gcp_list 核對（標頭、樣板佔位、影像檔名都在上傳清單裡、像素在影像內、
高程基準與 geo.txt 相同、檢核點沒有混進來），有錯就不送；核對摘要記在任務紀錄（odm/submission.json）。
沒給 GCP 時任務內容與加入這個功能前完全相同。

輸出（out_dir 底下；同樣輸入 ⇒ 位元組相同，檔案內不寫當下時間、也不寫 out_dir 的絕對路徑）::

    photos.csv / photos.geojson / photos.meta.json   I1（F2）
    odm/geo.txt、odm/images/<image_id>.jpg           給 ODM
    qa.json                                          有效設定、每架 QA、PPK、視數、錯誤與警告
    ppk/<呼號>/、ppk/base/                           只有明確開啟 RTKLIB 時才有

送 NodeODX 之後（含時間戳與 uuid，不屬於「位元組相同」的交付物；格式見本檔 NodeODX 一節）::

    odm/submission.json                              任務紀錄：每個階段都更新，可「接回任務」；權杖永遠不寫進去
    odm/quality_summary.json                         引擎品質報告摘要（odm_report/stats.json、log.json…）
    odm/result_<uuid>/、odm/<uuid>_all.zip           成果
    odm/history/                                     同一個輸出資料夾先前送過的任務紀錄
"""

from __future__ import annotations

import json
import logging
import math
import os
import re
import shutil
import time
import uuid as uuid_mod
import zipfile
from collections import Counter
from dataclasses import asdict, dataclass, field, is_dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional, Sequence

import numpy as np

from survey import datum, deliverable, dflog, exif_write, geotag, ingest, odm_client
from survey import pose as pose_mod
from survey import qa as qa_mod
from survey.ppk import events as ppk_events
from survey.ppk import geodesy as ppk_geodesy
from survey.ppk import interp as ppk_interp
from survey.ppk import outputs as ppk_outputs
from survey.ppk import pos as ppk_pos
from survey.ppk import rtklib
from survey.ppk import ubx as ppk_ubx

_log = logging.getLogger(__name__)

HEIGHTS = ("ellipsoidal", "orthometric")
_HEIGHT_KEY = {"ellipsoidal": "h_ell_m", "orthometric": "h_orth_m"}
WGS84_CRS = pose_mod.CRS_H_WGS84
QA_FORMAT = "aeroplan-survey-qa/1"

# PPK 列有這些旗標就不取代 fc_interp（位置不是 fix、內插跨缺口、不是相機中心、時刻外插）
PPK_REJECT_FLAGS = frozenset(ppk_outputs.NOT_FIXED_FLAGS | {"lever_arm_unknown", "no_attitude", "fit_extrapolated"})
# 位置換成 PPK 後，描述飛控位置／速度／高度來源的旗標不再成立
_FC_POSITION_FLAGS = frozenset({"pos_gap", "pos_from_gps", "h_ell_unknown", "h_orth_needs_h_ell",
                                "h_orth_receiver_geoid", "height_unknown", "sigma_unknown", "vel_gap",
                                "velocity_unknown", "cog_undefined"})
_PPK_FIELDS = ("t_gpst_s", "lat_deg", "lon_deg", "h_ell_m", "cog_deg", "speed_n_mps", "speed_e_mps",
               "speed_d_mps", "sigma_h_m", "sigma_v_m")

# PPK 數值鎖（docs/評估_2026-10-21_EmlidStudio_PPK接入.md §4 M2／M3；兩條路線——RTKLIB 與既有 .pos——都套）
# M2a：.pos 表頭 obs start／end（GPST 週內秒印 %8.1f ⇒ 0.1 s）必須落在 rover.ubx 的 RAWX 跨度內；
#      第一筆解不能早於 obs start（時間欄是 UTC 卻標 GPST ⇒ 早 18 s）
RAWX_SPAN_TOL_S = 0.1
FIRST_SOLUTION_TOL_S = 0.05
AGE_WARN_MEDIAN_S = 1.0            # .pos 的 age(s) 中位數超過就警告（基站觀測與移動站差太遠）
# M2b：% ref pos（解算實際用的基站標石位置）vs 操作員填的基站座標；RTKLIB 印 9 位小數度（≈0.1 mm）、4 位小數 m
REF_POS_TOL_H_M = 0.005
REF_POS_TOL_V_M = 0.005
# M3：已採用 PPK 的照片 vs 同一張的 fc_interp（同一個相機中心），中位數水平或垂直差超過 ⇒ 該架 QA 不通過。
#     **提案值、未用首飛資料校準**；要抓的是基站高填成正高（差一個 N，本場址約 18.5 m）、時間偏 18 s
#     （20 m/s ⇒ 360 m）、多機同飛時拿到別架的 .pos。PpkSettings.fc_gross_h_m／fc_gross_v_m 可逐次調整
PPK_FC_MAX_MEDIAN_H_M = 15.0
PPK_FC_MAX_MEDIAN_V_M = 15.0


class PipelineError(ValueError):
    """輸入或設定不合理（訊息是給操作員看的繁中說明）。"""


class PipelineCanceled(RuntimeError):
    """使用者取消。"""


class PpkLockError(PipelineError):
    """PPK 數值鎖不通過（.pos 與 rover.ubx／基站座標對不上）。``partial`` 是已讀到的軌跡資訊，會寫進 qa.json。"""

    def __init__(self, msg: str, partial: Optional[dict] = None):
        super().__init__(msg)
        self.partial = dict(partial or {})


# ─────────────────────────────────────────────────────────────────────────────
# 輸入
# ─────────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class SortieInput:
    """一架次。``rover_ubx`` 省略時用會期的 gnss/rover.ubx；``pos_path`` = 已解好的 RTKLIB .pos。"""
    callsign: str
    bin_path: object
    session_dir: object
    rover_ubx: object = None
    pos_path: object = None


@dataclass(frozen=True)
class PpkSettings:
    """PPK 設定。``frame`` 必填：基站座標所屬框架，一路帶到 photos.csv 的 crs_h。

    ``run_rtklib=True`` 才會呼叫外部程式（survey.ppk.rtklib），此時要 ``base`` 與基站觀測
    （``base_obs`` RINEX 或 ``base_ubx`` 二擇一）。沒開 RTKLIB 時每架都要給 ``SortieInput.pos_path``，
    而且 ``base`` 一樣必填：既有 .pos（例如 Emlid Studio）表頭的 ``% ref pos`` 要與它一致（M2b），
    框架名稱才真的綁在解算用的那組基站座標上。
    """
    frame: str
    base: Optional[rtklib.BaseStation] = None
    base_obs: object = None
    base_ubx: object = None
    run_rtklib: bool = False
    convbin_exe: object = None
    rnx2rtkp_exe: object = None
    rtklib_overrides: Optional[dict] = None
    edge: str = "rising"
    fix_ratio_threshold: float = ppk_outputs.FIX_RATIO_THRESHOLD
    interp_method: str = "hermite"
    max_gap_s: float = ppk_interp.MAX_GAP_S
    pair_tol_s: float = 0.002
    timeout_s: float = 3600.0
    fc_gross_h_m: float = PPK_FC_MAX_MEDIAN_H_M
    fc_gross_v_m: float = PPK_FC_MAX_MEDIAN_V_M

    def __post_init__(self):
        if not isinstance(self.frame, str) or not self.frame.strip():
            raise PipelineError("PPK：基站座標框架名稱必填（例如 e-GNSS[2025]）——它會成為照片的 crs_h，不能留白")
        if self.base is not None and self.base.frame != self.frame:
            raise PipelineError(f"PPK：基站框架 {self.base.frame!r} 與設定的框架 {self.frame!r} 不同")
        if self.edge not in ("rising", "falling"):
            raise PipelineError(f"PPK：邊緣極性只能是 rising／falling，收到 {self.edge!r}")
        if not (0.0 < float(self.fix_ratio_threshold) <= 1.0):
            raise PipelineError(f"PPK：fix 比例門檻要在 (0, 1]，收到 {self.fix_ratio_threshold!r}")
        if self.interp_method not in ("hermite", "linear"):
            raise PipelineError(f"PPK：內插方法只能是 hermite／linear，收到 {self.interp_method!r}")
        for name, v in (("水平", self.fc_gross_h_m), ("垂直", self.fc_gross_v_m)):
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
                raise PipelineError(f"PPK：對飛控的{name}粗差門檻要是正的有限公尺數，收到 {v!r}")
        if self.run_rtklib:
            if self.base is None:
                raise PipelineError("PPK：要跑 RTKLIB 就必須給基站座標（緯度、經度、橢球高、天線高、框架）")
            if (self.base_obs is None) == (self.base_ubx is None):
                raise PipelineError("PPK：基站觀測要給 RINEX 或 UBX 其中一個（不能都沒有、也不能都給）")
        elif self.base is None:
            raise PipelineError("PPK：用既有 .pos 也必須給基站座標（緯度、經度、橢球高、天線高、框架）——要與解算軟體"
                                "（例如 Emlid Studio）裡輸入的一致，會核對 .pos 表頭的 % ref pos")

    def effective(self) -> dict:
        return {
            "frame": self.frame, "base": None if self.base is None else asdict(self.base),
            "base_obs": _s(self.base_obs), "base_ubx": _s(self.base_ubx), "run_rtklib": bool(self.run_rtklib),
            "rtklib_overrides": dict(self.rtklib_overrides or {}), "edge": self.edge,
            "fix_ratio_threshold": float(self.fix_ratio_threshold), "interp_method": self.interp_method,
            "max_gap_s": float(self.max_gap_s), "pair_tol_s": float(self.pair_tol_s),
            "fc_gross_h_m": float(self.fc_gross_h_m), "fc_gross_v_m": float(self.fc_gross_v_m),
        }


# ─────────────────────────────────────────────────────────────────────────────
# 結果
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class SortieResult:
    callsign: str
    ingest: object                  # survey.ingest.IngestResult
    rows: list                      # 合併後的 F2 列
    qa: dict                        # survey.qa.sortie_report
    ppk: Optional[dict]             # PPK 摘要（沒開 PPK 為 None）
    reasons: list                   # QA 未通過的原因
    warnings: list
    effective: dict
    wind: Optional[dict] = None     # survey.qa.wind_report（EKF 風估計；旗標，不影響 passed）

    @property
    def passed(self) -> bool:
        return not self.reasons


@dataclass
class CampaignResult:
    out_dir: Path
    sorties: list
    rows: list
    paths: dict                     # photos_csv, photos_geojson, photos_meta, geo_txt, images_dir, qa_json
    images: list                    # 寫出的加標籤影像（Path）
    effective: dict
    qa: dict                        # {'passed', 'reasons', 'per_sortie', 'exposure'}
    odm: dict                       # {'rolling_shutter_readout_ms', 'rolling_shutter_error', 'gsd_cm', ...}
    warnings: list = field(default_factory=list)
    errors: list = field(default_factory=list)

    @property
    def passed(self) -> bool:
        return bool(self.qa.get("passed"))


# ─────────────────────────────────────────────────────────────────────────────
# 小工具
# ─────────────────────────────────────────────────────────────────────────────

def _s(p) -> Optional[str]:
    return None if p is None else str(p)


def _check_cancel(should_cancel) -> None:
    if should_cancel is not None and should_cancel():
        raise PipelineCanceled("使用者取消")


def _jsonable(o):
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.floating):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, Path):
        return str(o)
    if is_dataclass(o):
        return asdict(o)
    if isinstance(o, (set, frozenset, tuple)):
        return list(o)
    return str(o)


def _dump_json(path: Path, obj) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj, ensure_ascii=False, indent=2, sort_keys=True, default=_jsonable) + "\n",
                    encoding="utf-8", newline="\n")
    return path


def _flags(r) -> list:
    return [f for f in (r.get("flags") or "").split(";") if f]


def _join_flags(flags) -> str:
    return ";".join(dict.fromkeys(f for f in flags if f))


def check_geo_projection(projection) -> str:
    """geo.txt 第一行的座標系：只接受 PROJ 可解析的**經緯度**（地理）座標系。

    本流程寫進 geo.txt 的是經度、緯度（ODM 以 ``transform3(srs, EPSG:4326, x, y)`` 讀）；
    投影座標系（例如 EPSG:3826 TWD97 TM2）會讓經緯度被當成公尺 ⇒ 直接拒絕。
    """
    from pyproj import CRS
    from pyproj.exceptions import CRSError
    s = str(projection or "").strip()
    if not s:
        raise PipelineError("geo.txt 投影是空的")
    try:
        crs = CRS.from_user_input(s)
    except CRSError as e:
        raise PipelineError(f"geo.txt 投影 {s!r} 無法由 PROJ 解析：{e}") from None
    if not crs.is_geographic:
        raise PipelineError(f"geo.txt 投影 {s!r} 不是經緯度座標系：本流程寫的是經度／緯度，不做投影換算")
    return s


def _session_meta(session_dir) -> dict:
    p = Path(session_dir) / "session.json"
    if not p.is_file():
        raise PipelineError(f"{session_dir} 不是 Pi 會期目錄（缺 session.json）")
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError) as e:
        raise PipelineError(f"{p} 讀不到：{e}") from None


def rover_ubx_for(sortie: SortieInput) -> Optional[Path]:
    """移動站 UBX：明確給的優先，否則會期的 gnss/rover.ubx（不存在則 None）。"""
    if sortie.rover_ubx:
        return Path(sortie.rover_ubx)
    p = Path(sortie.session_dir) / "gnss" / "rover.ubx"
    return p if p.is_file() else None


def _as_sortie(s) -> SortieInput:
    if isinstance(s, SortieInput):
        return s
    if isinstance(s, dict):
        return SortieInput(**s)
    raise PipelineError(f"架次輸入要是 SortieInput 或 dict，收到 {type(s).__name__}")


EVENTS_POS_REFUSAL = ("這是 Emlid Studio 的事件檔（_events.pos：每個時間標記一筆、只按順序對照片、遺失的觸發看不到、"
                      "沒有曝光中心偏移與槓桿臂），不是整條軌跡 ⇒ 請給同一次解算的移動站軌跡 .pos（檔名沒有 _events 的那個）")


def is_events_pos(path) -> bool:
    """Emlid Studio 的 ``<名稱>_events.pos``（不分大小寫）。"""
    return Path(str(path)).name.lower().endswith("_events.pos")


def _validate_sortie(s: SortieInput, ppk: Optional[PpkSettings]) -> None:
    cs = (s.callsign or "").strip() if isinstance(s.callsign, str) else ""
    if not cs:
        raise PipelineError(f"架次 {s.session_dir}：呼號必填")
    if not Path(s.bin_path).is_file():
        raise PipelineError(f"{cs}：飛控日誌不存在：{s.bin_path}")
    meta = _session_meta(s.session_dir)
    if str(meta.get("uav")) != cs:
        raise PipelineError(f"{cs}：會期 {s.session_dir} 的 uav 是 {meta.get('uav')!r}，與呼號不符（拿錯會期？）")
    if ppk is None:
        return
    ubx = rover_ubx_for(s)
    if ubx is None or not ubx.is_file():
        raise PipelineError(f"{cs}：PPK 需要相機模組的 rover.ubx（TIM-TM2 曝光時刻），找不到"
                            f"（{ubx or Path(s.session_dir) / 'gnss' / 'rover.ubx'}）")
    if s.pos_path:
        if not Path(s.pos_path).is_file():
            raise PipelineError(f"{cs}：.pos 不存在：{s.pos_path}")
        if is_events_pos(s.pos_path):
            raise PipelineError(f"{cs}：{EVENTS_POS_REFUSAL}（{Path(s.pos_path).name}）")
    elif not ppk.run_rtklib:
        raise PipelineError(f"{cs}：PPK 已開啟，但這架沒有 .pos，也沒有開啟 RTKLIB 解算")


# ─────────────────────────────────────────────────────────────────────────────
# 每架次
# ─────────────────────────────────────────────────────────────────────────────

def _qa_reasons(callsign: str, rep: dict) -> list:
    """把 sortie_report 的硬性條件翻成給人看的原因（與 rep['passed'] 同一組條件）。"""
    out = []
    if rep.get("n_strips", 0) == 0:
        out.append(f"{callsign}：日誌裡找不到任何作業帶（206 開／關），無法逐帶核對")
    for c in rep["conservation"]:
        if c["conserved"]:
            continue
        where = f"第 {c['strip']} 帶" if c["strip"] is not None else "帶外"
        trig = f"、TRIG {c['fc_trig']}" if c["fc_trig"] is not None else ""
        out.append(f"{callsign}：{where}張數不守恆（飛控 CAM {c['fc_cam']}{trig}、Pi 邊緣 {c['pi_edges']}、"
                   f"JPEG {c['jpeg_ok']}；未配對 飛控 {c['unmatched_fc']}、Pi {c['unmatched_pi']}）")
    for s in rep["spacing"]:
        if s["median_ok"] and s["p99_ok"]:
            continue
        if s["median_m"] is None:
            out.append(f"{callsign}：第 {s['strip']} 帶不到兩張，量不出間距")
        else:
            out.append(f"{callsign}：第 {s['strip']} 帶間距 中位數 {s['median_m']:.2f} m／p99 {s['p99_m']:.2f} m，"
                       f"與 d={s['d_m']:.2f} m 不符（中位數 ±2%、p99 ≤ 1.2d）")
    if not rep["fit"]["rms_ok"]:
        out.append(f"{callsign}：時鐘擬合殘差 RMS {rep['fit']['rms_ms']:.3f} ms 超過 "
                   f"{qa_mod.FIT_RMS_LIMIT_S * 1e3:.1f} ms")
    if rep["unmatched_fc"]:
        idx = ", ".join(str(u["index"]) for u in rep["unmatched_fc"][:10])
        out.append(f"{callsign}：{len(rep['unmatched_fc'])} 個飛控拍照事件沒有對應的 Pi 邊緣（index {idx}）")
    if rep["unmatched_pi"]:
        seq = ", ".join(str(u["seq"]) for u in rep["unmatched_pi"][:10])
        out.append(f"{callsign}：{len(rep['unmatched_pi'])} 個 Pi 邊緣沒有對應的飛控事件（seq {seq}）")
    if not rep.get("passed") and not out:
        out.append(f"{callsign}：架次 QA 未通過")
    return out


def _convbin_base(ppk: PpkSettings, work: Path) -> tuple:
    """基站給的是 UBX 時先轉 RINEX（convbin）。回傳 (obs, nav, 命令)。"""
    exe = rtklib.find_executable("convbin", ppk.convbin_exe)
    ubx = Path(ppk.base_ubx)
    work.mkdir(parents=True, exist_ok=True)
    obs, nav = work / (ubx.stem + ".obs"), work / (ubx.stem + ".nav")
    cmd = rtklib.convbin_command(ubx, obs, nav, exe=exe)
    rtklib._run(cmd, ppk.timeout_s)
    rtklib._must_exist(obs, "convbin 基站觀測檔")
    rtklib._must_exist(nav, "convbin 基站導航檔")
    return obs, nav, cmd


def _ppk_trajectory(sortie: SortieInput, ubx_path: Path, ppk: PpkSettings, base_rinex, work_dir):
    if sortie.pos_path:
        p = Path(sortie.pos_path)
        return ppk_pos.read_pos(p), {"source": "pos_file", "path": str(p)}
    obs, extra_nav = base_rinex
    wd = Path(work_dir) / sortie.callsign
    run = rtklib.run_ppk(ubx_path, obs, ppk.base, wd, extra_nav=extra_nav, overrides=ppk.rtklib_overrides,
                         convbin_exe=ppk.convbin_exe, rnx2rtkp_exe=ppk.rnx2rtkp_exe, timeout_s=ppk.timeout_s)
    return ppk_pos.read_pos(run.pos_path), {"source": "rtklib", "path": _rel(run.pos_path, Path(work_dir).parent),
                                            "commands": [list(map(str, c)) for c in run.commands],
                                            "options": dict(run.options)}


def _gpst_text(t: float) -> str:
    wk = int(t // ppk_pos.GPS_WEEK_S)
    return f"week{wk} {t - wk * ppk_pos.GPS_WEEK_S:.3f}s"


def _check_ref_pos(traj, base: rtklib.BaseStation, rec: dict) -> None:
    """M2b：.pos 表頭的 % ref pos（解算實際用的基站標石位置）必須等於填的基站座標（水平、垂直各 ≤ 5 mm）。

    Emlid Studio 的基站座標預設取 RINEX 表頭（通常是接收機單點平均），表頭框架永遠寫 WGS84 ⇒ 不核對的話，
    框架名稱填 e-GNSS[2025] 而照片其實落在單點平均的位置上，下游看不出來。天線高不在 ref pos 裡，不比對。
    """
    meta = traj.meta
    if meta.ref_pos_text is None:
        raise PpkLockError(
            f".pos 表頭沒有 % ref pos ⇒ 不是相對定位（單點／PPP？）或解算軟體沒寫出基站位置，無法核對它用的"
            f"基站座標是不是填的 {base.frame} 座標（Emlid Studio 請用 Kinematic、手動輸入基站座標）")
    if meta.ref_pos_llh is None:
        raise PpkLockError(f".pos 的 % ref pos 解不出來：{'；'.join(meta.problems) or meta.ref_pos_text}")
    la, lo, h = meta.ref_pos_llh
    d = ppk_geodesy.ecef_to_ned_delta(base.lat_deg, base.lon_deg,
                                      ppk_geodesy.llh_to_ecef(la, lo, h)
                                      - ppk_geodesy.llh_to_ecef(base.lat_deg, base.lon_deg, base.h_ell_m))
    dh, dv = float(np.hypot(d[0], d[1])), float(-d[2])
    ok = dh <= REF_POS_TOL_H_M and abs(dv) <= REF_POS_TOL_V_M
    rec["ref_pos_check"] = {"ref_pos_llh": [la, lo, h], "base_llh": [base.lat_deg, base.lon_deg, base.h_ell_m],
                            "base_frame": base.frame, "d_h_m": dh, "d_v_m": dv, "tol_h_m": REF_POS_TOL_H_M,
                            "tol_v_m": REF_POS_TOL_V_M, "antenna2": meta.antenna2, "passed": ok}
    if ok:
        return
    hint = ""
    if (dh <= REF_POS_TOL_H_M and base.antenna_height_m > REF_POS_TOL_V_M
            and abs(dv - base.antenna_height_m) <= REF_POS_TOL_V_M):
        hint = (f"；垂直差恰好等於天線高 {base.antenna_height_m:.4f} m ⇒ 解算時的基站高可能是天線參考點而不是標石，"
                "請確認兩邊的高度定義")
    raise PpkLockError(
        f".pos 解算用的基站位置（% ref pos {la:.9f}, {lo:.9f}, {h:.4f} m）與填的基站座標（{base.lat_deg:.9f}, "
        f"{base.lon_deg:.9f}, {base.h_ell_m:.4f} m，{base.frame}）差 水平 {dh * 1e3:.1f} mm／垂直 {dv * 1e3:+.1f} mm"
        f"（容許各 {REF_POS_TOL_H_M * 1e3:.0f} mm）⇒ 照片會落在另一組基站座標上（例如 RINEX 表頭的單點平均位置）{hint}")


def _check_rover_span(traj, ulog, rec: dict) -> None:
    """M2a：這個 .pos 必須是用這架 rover.ubx 的觀測解的，而且時間是 GPST。

    obs start ≥ 第一個 week>0 的 RAWX epoch − 0.1 s、obs end ≤ 最後一個 + 0.1 s、第一筆解 ≥ obs start − 0.05 s。
    obs start／end 缺任一個就丟錯（RTKLIB 兩行一起印；只剩一行代表表頭不是原樣，鎖不能靜默不檢查）。
    """
    meta = traj.meta
    ep = [r.week * ppk_pos.GPS_WEEK_S + r.rcv_tow_s for r in ulog.rawx if r.week > 0]
    t0 = float(traj.t_gpst_s[0])
    rec["rover_span"] = {
        "rawx_first_gpst_s": ep[0] if ep else None, "rawx_last_gpst_s": ep[-1] if ep else None,
        "n_rawx": len(ep), "n_rawx_week0": len(ulog.rawx) - len(ep),
        "obs_start_gpst_s": None if meta.obs_start is None else meta.obs_start.gpst_s,
        "obs_end_gpst_s": None if meta.obs_end is None else meta.obs_end.gpst_s,
        "first_solution_gpst_s": t0, "tol_s": RAWX_SPAN_TOL_S, "first_solution_tol_s": FIRST_SOLUTION_TOL_S}
    for key, o in (("obs start", meta.obs_start), ("obs end", meta.obs_end)):
        if o is None:
            raise PpkLockError(
                f".pos 表頭沒有可用的 % {key}（括號內的 GPST 週／週內秒）⇒ 無法核對這個 .pos 是不是用這架的 "
                f"rover.ubx 解的、時間是不是 GPST；請開啟輸出表頭重新輸出")
    if not ep:
        raise PpkLockError(f"rover.ubx 沒有週數有效（week>0）的 RXM-RAWX ⇒ 無法核對 .pos 的觀測時間跨度")
    s, e = meta.obs_start.gpst_s, meta.obs_end.gpst_s
    if s < ep[0] - RAWX_SPAN_TOL_S:
        raise PpkLockError(
            f".pos 的 obs start（{_gpst_text(s)}）比 rover.ubx 第一個 RAWX epoch（{_gpst_text(ep[0])}）早 "
            f"{ep[0] - s:.3f} s ⇒ 這個 .pos 不是用這個 rover.ubx 解的（別架、別次，或 rover.ubx 被截斷）")
    if e > ep[-1] + RAWX_SPAN_TOL_S:
        raise PpkLockError(
            f".pos 的 obs end（{_gpst_text(e)}）比 rover.ubx 最後一個 RAWX epoch（{_gpst_text(ep[-1])}）晚 "
            f"{e - ep[-1]:.3f} s ⇒ 這個 .pos 不是用這個 rover.ubx 解的（別架、別次，或 rover.ubx 被截斷）")
    if t0 < s - FIRST_SOLUTION_TOL_S:
        d = s - t0
        hint = ""
        if abs(d - round(d)) < 0.1 and 10 <= round(d) <= 25:
            hint = f"；差值是整數 {round(d)} s（GPS－UTC 閏秒目前 18 s）⇒ 時間欄很可能是 UTC 卻標成 GPST"
        raise PpkLockError(f".pos 第一筆解（{_gpst_text(t0)}）比表頭 obs start（{_gpst_text(s)}）早 {d:.3f} s"
                           f"——解不可能早於第一個觀測{hint}")


def _age_stats(cs: str, traj) -> tuple:
    """age(s) 統計（基站觀測與移動站的時間差）；中位數 > 1 s 出警告。回傳 (統計或 None, 警告)。"""
    a = None if traj.age_s is None else np.asarray(traj.age_s, float)
    a = None if a is None else a[np.isfinite(a)]
    if a is None or a.size == 0:
        return None, [f"{cs}：.pos 沒有 age(s) 欄 ⇒ 無法確認基站觀測與移動站的時間差"]
    st = {"n": int(a.size), "median_s": float(np.median(a)), "p95_s": float(np.percentile(a, 95)),
          "max_s": float(a.max()), "warn_median_s": AGE_WARN_MEDIAN_S}
    if st["median_s"] <= AGE_WARN_MEDIAN_S:
        return st, []
    return st, [f"{cs}：.pos 的 age(s) 中位數 {st['median_s']:.2f} s > {AGE_WARN_MEDIAN_S:g} s：基站觀測與移動站相差"
                "超過 1 s（基站 30 s 取樣被內插？）⇒ 公分級精度沒有保證"]


def _fc_gross_check(cs: str, adopted: list, ppk: PpkSettings) -> tuple:
    """M3：已採用 PPK 的照片，PPK 相機中心 − 同一張 fc_interp 相機中心的中位數水平／垂直差。

    比相機中心（不是天線）：兩邊都已加槓桿臂，差值只剩位置本身；槓桿臂打錯數十公尺也會被抓到。
    垂直取有號差的中位數（系統偏移，例如正高當橢球高），飛控沒有橢球高的照片不計入垂直。
    回傳 (紀錄, QA 未通過原因, 警告)。
    """
    rec = {"point": "相機中心（PPK 採用值 − fc_interp）", "n": len(adopted), "limit_h_m": float(ppk.fc_gross_h_m),
           "limit_v_m": float(ppk.fc_gross_v_m), "median_h_m": None, "max_h_m": None, "n_v": 0,
           "median_v_m": None, "passed": True}
    hz = [(fc, p) for fc, p in adopted if fc.get("lat_deg") is not None and fc.get("lon_deg") is not None]
    if hz:
        fl = np.asarray([[fc["lat_deg"], fc["lon_deg"]] for fc, _ in hz], float)
        pl = np.asarray([[p["lat_deg"], p["lon_deg"], p["h_ell_m"]] for _, p in hz], float)
        # 同一個高度算 ECEF 差 ⇒ 水平距離不受飛控高度有無影響
        d = ppk_geodesy.ecef_to_ned_delta(fl[:, 0], fl[:, 1],
                                          ppk_geodesy.llh_to_ecef(pl[:, 0], pl[:, 1], pl[:, 2])
                                          - ppk_geodesy.llh_to_ecef(fl[:, 0], fl[:, 1], pl[:, 2]))
        h = np.hypot(d[:, 0], d[:, 1])
        rec.update(median_h_m=float(np.median(h)), max_h_m=float(h.max()))
    dv = [p["h_ell_m"] - fc["h_ell_m"] for fc, p in adopted if fc.get("h_ell_m") is not None]
    if dv:
        rec.update(n_v=len(dv), median_v_m=float(np.median(dv)))
    reasons, warnings = [], []
    bad_h = rec["median_h_m"] is not None and rec["median_h_m"] > ppk.fc_gross_h_m
    bad_v = rec["median_v_m"] is not None and abs(rec["median_v_m"]) > ppk.fc_gross_v_m
    if bad_h or bad_v:
        rec["passed"] = False
        mh = "—" if rec["median_h_m"] is None else f"{rec['median_h_m']:.1f} m"
        mv = "—" if rec["median_v_m"] is None else f"{rec['median_v_m']:+.1f} m"
        reasons.append(f"{cs}：PPK 與飛控位置差 中位數 水平 {mh}／垂直 {mv}（{len(adopted)} 張；上限 水平 "
                       f"{ppk.fc_gross_h_m:g} m、垂直 {ppk.fc_gross_v_m:g} m）⇒ .pos 可能拿錯架次、時間不是 GPST，"
                       "或基站高填成正高（差一個大地起伏 N）")
    if adopted and not dv:
        warnings.append(f"{cs}：飛控位姿沒有橢球高 ⇒ PPK 對飛控的垂直粗差檢查沒做（基站高填成正高這類錯誤抓不到）")
    return rec, reasons, warnings


def _apply_ppk(res, rows: list, sortie: SortieInput, ppk: PpkSettings, *, sensor_ts_semantics: str,
               geoid, base_rinex, work_dir, should_cancel) -> tuple:
    """回傳 (合併後的列, PPK 摘要)。摘要含 reasons（QA 未通過的原因）與 warnings。"""
    session = res.session
    cs = sortie.callsign
    ubx_path = rover_ubx_for(sortie)
    traj, traj_info = _ppk_trajectory(sortie, ubx_path, ppk, base_rinex, work_dir)
    traj_rec = {**traj_info, "n_epochs": len(traj), "datum_label": traj.datum_label,
                "skipped_lines": list(traj.skipped_lines), "has_velocity": traj.vel_ned_mps is not None,
                "meta": traj.meta.as_dict()}
    try:
        _check_ref_pos(traj, ppk.base, traj_rec)
        _check_cancel(should_cancel)
        ulog = ppk_ubx.read_ubx_file(ubx_path)
        _check_rover_span(traj, ulog, traj_rec)
    except PpkLockError as e:
        e.partial.setdefault("trajectory", traj_rec)
        raise
    ex = ppk_events.extract_edges(ulog, edge=ppk.edge)
    warnings, reasons = [], []
    traj_rec["age_s"], age_warn = _age_stats(cs, traj)
    warnings += age_warn
    if not ex.pvt_checked:
        warnings.append(f"{cs}：rover.ubx 沒有可用的 NAV-PVT，TIM-TM2 的時間基準（閏秒／GNSS 系統）無法交叉核對")

    # 曝光中心相對邊緣的時間（Pi CLOCK_MONOTONIC 秒）：與 fc_interp 用同一個曝光中心定義
    # （pose.exposure_center_ns，含 SensorTimestamp 語意與讀出時間），BOOTTIME 以 clocks.jsonl 的偏移換回 MONOTONIC
    readout_ms = res.poses.effective["readout_ms"]
    frames, offs = [], []
    for f in session.frames:
        frames.append({"seq": f.seq, "edge_ns": f.edge_ns, "file": f.file, "status": f.status})
        if f.sensor_ts_ns is None:
            offs.append(None)
            continue
        c_boot, _ = pose_mod.exposure_center_ns(f.sensor_ts_ns, f.exposure_us, readout_ms, sensor_ts_semantics)
        offs.append((c_boot - session.clock_offset_ns - f.edge_ns) * 1e-9)
    pr = ppk_events.pair_with_frames(ex, frames, exposure_offset_s=offs, pair_tol_s=ppk.pair_tol_s)
    keep, skipped = ppk_outputs.photo_events(pr)

    fc_by_seq = {r["seq"]: r for r in rows}
    use = [e for e in keep if e.seq in fc_by_seq]
    no_fc_row = [e.seq for e in keep if e.seq not in fc_by_seq]
    lever = session.meta.get("gnss_lever_arm_m")
    if lever is None:
        warnings.append(f"{cs}：session.json 的 gnss_lever_arm_m 是 null ⇒ PPK 只有天線位置、不是相機中心，"
                        "全部照片保留 fc_interp（請量測相機中心 − 相機模組天線的槓桿臂）")
    prow_by_seq = {}
    fix_windows = []
    if use:
        # 姿態：同一個曝光時刻的 FC 姿態（fc_interp 列已經由 FC↔Pi 時鐘擬合內插到曝光中心）
        att = np.asarray([[fc_by_seq[e.seq]["yaw_deg"], fc_by_seq[e.seq]["pitch_deg"],
                           fc_by_seq[e.seq]["roll_deg"]] for e in use], float)
        ip = ppk_interp.interpolate_at(traj, [e.t_exposure_gpst_s for e in use], method=ppk.interp_method,
                                       max_gap_s=ppk.max_gap_s, lever_arm_frd_m=lever, attitude_deg=att)
        prows = ppk_outputs.ppk_photo_rows(
            use, ip, crs_h=ppk.frame, uav=session.uav, attitude_deg=att,
            strip_of_seq={e.seq: fc_by_seq[e.seq]["strip"] for e in use},
            image_id_of_seq={e.seq: fc_by_seq[e.seq]["image_id"] for e in use})
        prow_by_seq = {r["seq"]: r for r in prows}
        wins = ppk_outputs.strip_windows([r["t_gpst_s"] if r["t_gpst_s"] is not None else math.nan for r in prows],
                                         [r["strip"] for r in prows])
        fix_windows = ppk_outputs.fix_ratio_report(
            traj, wins, threshold=ppk.fix_ratio_threshold,
            photo_t=[r["t_gpst_s"] if r["t_gpst_s"] is not None else math.nan for r in prows],
            photo_flags=[_flags(r) for r in prows])
    for w in fix_windows:
        if w.below_threshold:
            ratio = "沒有 epoch" if w.fix_ratio is None else f"{w.fix_ratio:.1%}"
            reasons.append(f"{cs}：PPK 第 {w.label} 帶 fix 比例 {ratio} < {ppk.fix_ratio_threshold:.0%}"
                           f"（照片未 fix {w.n_photos_not_fixed}/{w.n_photos}）")

    skip_reason = dict(skipped)
    merged, used, rejected, adopted = [], 0, Counter(), []
    for r in rows:
        p = prow_by_seq.get(r["seq"])
        if p is None:
            why = skip_reason.get(r["seq"], "no_tm2_event")
            rejected[f"ppk_missing:{why}"] += 1
            merged.append({**r, "flags": _join_flags(_flags(r) + ["ppk_missing"])})
            continue
        pf = _flags(p)
        bad = sorted(set(pf) & PPK_REJECT_FLAGS)
        if p["lat_deg"] is None or p["lon_deg"] is None or p["h_ell_m"] is None:
            bad.append("ppk_no_position")
        if bad:
            for b in bad:
                rejected[b] += 1
            merged.append({**r, "flags": _join_flags(_flags(r) + ["ppk_rejected"] + bad)})
            continue
        h_ell, h_orth, crs_v, hfl = datum.ellipsoidal_heights(p["h_ell_m"], p["lat_deg"], p["lon_deg"], geoid=geoid)
        m = dict(r)
        for k in _PPK_FIELDS:
            m[k] = p[k]
        m.update(h_ell_m=h_ell, h_orth_m=h_orth, crs_v=crs_v, source="ppk", crs_h=ppk.frame,
                 flags=_join_flags([f for f in _flags(r) if f not in _FC_POSITION_FLAGS] + pf + hfl))
        merged.append(m)
        adopted.append((r, m))
        used += 1
    fc_check, fc_reasons, fc_warn = _fc_gross_check(cs, adopted, ppk)
    reasons += fc_reasons
    warnings += fc_warn

    info = {
        "trajectory": traj_rec,
        "fc_check": fc_check,
        "rover_ubx": str(ubx_path),
        "edges": {"n_edges": len(ex.edges), "n_lost": ex.n_lost, "invalid": dict(Counter(r for _, r in ex.invalid)),
                  "duplicates": len(ex.duplicates), "other_channel": ex.other_channel, "edge": ex.edge,
                  "pvt_checked": ex.pvt_checked, "rawx_leap_s": ex.rawx_leap_s},
        "pairing": {"n_paired": pr.n_paired, "rate_ppm": pr.fit.rate_ppm, "rms_ms": pr.fit.rms_s * 1e3,
                    "max_abs_ms": pr.fit.max_abs_s * 1e3, "counter_shifts": [list(x) for x in pr.counter_shifts],
                    "unmatched_tm2": len(pr.unmatched_tm2), "unmatched_pi_seq": list(pr.unmatched_pi_seq)},
        "lever_arm_frd_m": lever,
        "skipped": [list(x) for x in skipped],
        "no_fc_row": no_fc_row,
        "fix_windows": [asdict(w) for w in fix_windows],
        "used": used, "rejected": dict(sorted(rejected.items())),
        "reasons": reasons, "warnings": warnings,
    }
    if rows and used == 0:
        reasons.append(f"{cs}：PPK 已開啟，但沒有任何一張照片採用 PPK 位置（{dict(rejected)}）")
    if no_fc_row:
        warnings.append(f"{cs}：{len(no_fc_row)} 張有 PPK 時刻但飛控位姿被略過（seq {no_fc_row[:10]}），不輸出")
    _log.info("%s：PPK 採用 %d 張、退回 fc_interp %s", cs, used, dict(rejected))
    return merged, info


def process_sortie(sortie, *, sensor_ts_semantics: str, geoid: Optional[datum.GeoidModel] = None,
                   ppk: Optional[PpkSettings] = None, work_dir=None, base_rinex=None, gps_lag_s: float = 0.0,
                   verify_sha256: bool = True, should_cancel: Optional[Callable[[], bool]] = None,
                   wind_limit_mps: float = qa_mod.WIND_LIMIT_MPS) -> SortieResult:
    """一架次：ingest（＋PPK 合併）＋ QA（含 EKF 風速旗標）。不寫檔（RTKLIB 的工作檔除外，寫在 ``work_dir``）。

    ``base_rinex``：(基站觀測檔, 額外導航檔 tuple)；``ppk.run_rtklib`` 且這架沒有 .pos 時必填
    （process_campaign 會先準備好）。
    """
    sortie = _as_sortie(sortie)
    gps_lag_s = check_gps_lag_s(gps_lag_s)
    wind_limit_mps = check_wind_limit_mps(wind_limit_mps)
    if sensor_ts_semantics not in pose_mod.SENSOR_TS_SEMANTICS:
        raise PipelineError(f"SensorTimestamp 語意必須明確選 {pose_mod.SENSOR_TS_SEMANTICS} 之一"
                            f"（台架 B3 未定案，沒有預設），收到 {sensor_ts_semantics!r}")
    _validate_sortie(sortie, ppk)
    if ppk is not None and ppk.run_rtklib and not sortie.pos_path:
        if base_rinex is None or work_dir is None:
            raise PipelineError(f"{sortie.callsign}：要跑 RTKLIB 需要 work_dir 與基站 RINEX")
    _check_cancel(should_cancel)
    res = ingest.ingest_sortie(sortie.bin_path, sortie.session_dir, sensor_ts_semantics=sensor_ts_semantics,
                               verify_sha256=verify_sha256, geoid=geoid, gps_lag_s=gps_lag_s)
    rows = [dict(r) for r in res.poses.rows]
    reasons = _qa_reasons(sortie.callsign, res.qa)
    warnings = [f"{sortie.callsign}：會期旗標 {', '.join(res.session.flags)}"] if res.session.flags else []
    wind = qa_mod.wind_report(res.log, res.fc_events, dflog.strip_windows(res.log), limit_mps=wind_limit_mps)
    warnings += _wind_warnings(sortie.callsign, wind)
    ppk_info = None
    if ppk is not None:
        _check_cancel(should_cancel)
        try:
            rows, ppk_info = _apply_ppk(res, rows, sortie, ppk, sensor_ts_semantics=sensor_ts_semantics,
                                        geoid=geoid, base_rinex=base_rinex, work_dir=work_dir,
                                        should_cancel=should_cancel)
        except PipelineCanceled:
            raise
        except (ppk_events.PairingError, ppk_pos.PosFormatError, rtklib.RtklibError, rtklib.RtklibNotFound,
                ValueError) as e:
            msg = f"{sortie.callsign}：PPK 失敗（{type(e).__name__}）：{e}"
            _log.error(msg)
            reasons.append(msg)
            rows = [{**r, "flags": _join_flags(_flags(r) + ["ppk_unavailable"])} for r in rows]
            # 數值鎖擋下時，已讀到的軌跡資訊（表頭、RAWX 跨度、ref pos 差值）也寫進 qa.json 供查核
            ppk_info = {**getattr(e, "partial", {}), "error": msg, "reasons": [msg], "warnings": []}
        else:
            reasons += ppk_info["reasons"]
            warnings += ppk_info["warnings"]
    eff = {
        "bin": str(sortie.bin_path), "session": str(sortie.session_dir), "uav": res.session.uav,
        "session_id": res.session.session_id, "unit_model": res.session.unit_model,
        "rover_ubx": _s(rover_ubx_for(sortie)) if ppk is not None else None, "pos": _s(sortie.pos_path),
        "pose": res.poses.effective, "session_flags": list(res.session.flags),
        "n_frames": len(res.session.frames), "n_rows": len(rows),
        "skipped": dict(sorted(Counter(s["reason"] for s in res.poses.skipped).items())),
    }
    return SortieResult(callsign=sortie.callsign, ingest=res, rows=rows, qa=res.qa, ppk=ppk_info,
                        reasons=reasons, warnings=warnings, effective=eff, wind=wind)


def _wind_warnings(callsign: str, wind: dict) -> list:
    """風的旗標 → 給人看的警告（不是 QA 未通過的原因）。"""
    lim = wind["limit_mps"]
    if not wind.get("available"):
        return [f"{callsign}：無法確認作業時的風速（{wind.get('reason')}）；建圖風速上限 {lim:g} m/s"]
    if wind.get("flagged"):
        return [f"{callsign}：{wind['share_above']:.1%} 的照片（{wind['n_photos_above']}/{wind['n_photos_with_wind']}）"
                f"拍攝時 EKF 估計風速 > {lim:g} m/s（建圖風速上限；最大 {wind['photo_speed_max_mps']:.1f} m/s）"]
    return []


def check_wind_limit_mps(v) -> float:
    """建圖風速上限（m/s）：(0, 50] 的有限實數；其他一律拒絕。"""
    if isinstance(v, bool) or not isinstance(v, (int, float, np.integer, np.floating)):
        raise PipelineError(f"建圖風速上限 wind_limit_mps 要是 m/s（實數），收到 {type(v).__name__} {v!r}")
    f = float(v)
    if not (math.isfinite(f) and 0.0 < f <= 50.0):
        raise PipelineError(f"建圖風速上限 wind_limit_mps = {v!r} 不合理（要在 (0, 50] m/s）")
    return f


# ─────────────────────────────────────────────────────────────────────────────
# 整個作業（多架）
# ─────────────────────────────────────────────────────────────────────────────

def _geo_rows(rows: list, geo_projection: Optional[str]) -> tuple:
    """geo.txt／EXIF 要用的列與標頭。回傳 (標頭 projection 或 None, 列, 警告)。框架不一致且沒有明確投影就丟錯。"""
    frames = Counter(str(r.get("crs_h")) for r in rows)
    if geo_projection is None:
        if set(frames) - {WGS84_CRS}:
            raise PipelineError(f"照片的水平框架 {dict(frames)} 不全是 {WGS84_CRS}：不會把其他框架當 WGS84 寫進 "
                                "geo.txt／EXIF；請明確給 geo.txt 的座標系（PROJ 可解析的經緯度座標系）")
        return None, rows, []
    notes = [f"geo.txt／EXIF 以使用者指定的 {geo_projection!r} 寫出；原本框架 {k}：{n} 張"
             for k, n in sorted(frames.items()) if k != geo_projection]
    return geo_projection, [{**r, "crs_h": geo_projection} for r in rows], notes


def _views(rows: list, csv_path: Path, sessions: list, height: str, ground_h_m, rings, ks, cell_m) -> dict:
    """``rings``：normalize_polygons 的結果（ring 的串列，多區取聯集）或 None。"""
    if ground_h_m is None:
        return {"computed": False, "reason": "沒有給地面高（與所選高度同一基準）⇒ 不算視數"}
    cams = sorted({(float(s.camera["lens_mm"]), float(s.camera["pixel_um"]), int(s.camera["width"]),
                    int(s.camera["height"])) for s in sessions})
    if len(cams) != 1:
        return {"computed": False, "reason": f"各機相機參數不同 {cams}：視數需要單一相機模型"}
    focal, pixel, w, h = cams[0]
    key = _HEIGHT_KEY[height]
    try:
        if rings:
            from core.recon.exposure_verifier import FrameCamera, verify_postflight
            rep = verify_postflight(csv_path, [[tuple(map(float, p)) for p in ring] for ring in rings],
                                    FrameCamera(focal, pixel, w, h),
                                    terrain=float(ground_h_m), height_field=key, cell_size_m=float(cell_m),
                                    ks=tuple(int(k) for k in ks))
            return {"computed": True, "source": "core.recon.exposure_verifier", "scope": "測區多邊形內",
                    "n_polygons": len(rings),
                    "frac_ge": {str(k): float(v) for k, v in rep.frac_ge.items()},
                    "forward_overlap": asdict(rep.forward_overlap), "side_overlap": asdict(rep.side_overlap),
                    "seam_overlap_min": rep.seam_overlap_min, "area_m2": rep.area_m2,
                    "undercovered_area_m2": rep.undercovered_area_m2, "photos_used": rep.photos_used,
                    "photos_excluded": dict(rep.photos_excluded), "notes": list(rep.notes),
                    "cell_m": float(cell_m), "ground_h_m": float(ground_h_m), "height_field": key}
        grid = qa_mod.footprint_multiplicity(rows, focal_mm=focal, pixel_um=pixel, width_px=w, height_px=h,
                                             ground_h_m=float(ground_h_m), height=height, cell_m=float(cell_m))
        return {"computed": True, "source": "survey.qa.footprint_multiplicity", "scope": "至少被拍到一次的格子",
                "frac_ge": {str(k): grid.fraction_at_least(int(k)) for k in ks},
                "n_footprints": grid.n_footprints, "n_skipped": grid.n_skipped,
                "cell_m": float(cell_m), "ground_h_m": float(ground_h_m), "height_field": key}
    except ValueError as e:
        return {"computed": False, "reason": f"視數無法計算：{e}"}


def _gsd_cm(rows: list, sessions: list, height: str, ground_h_m) -> Optional[float]:
    """正射解析度的預設值 ≈ GSD（各機中位 AGL 下最粗的那個，cm，取到 0.1）。沒有地面高就不猜。"""
    if ground_h_m is None:
        return None
    key = _HEIGHT_KEY[height]
    out = []
    for s in sessions:
        agl = [r[key] - ground_h_m for r in rows if r["uav"] == s.uav and r.get(key) is not None]
        if agl:
            out.append(float(np.median(agl)) * float(s.camera["pixel_um"]) * 1e-6
                       / (float(s.camera["lens_mm"]) * 1e-3) * 100.0)
    return round(max(out), 1) if out else None


GPS_LAG_MAX_S = 1.0       # 接收機＋驅動延遲的合理上限（ArduPilot GPS_DELAY_MS 參數範圍 0–250 ms）


def _is_ring_list(obj) -> bool:
    """obj[0][0] 本身又是一個點（長度 ≥ 2 的序列）⇒ obj 是「ring 的串列」而不是單一 ring。"""
    try:
        inner = obj[0][0]
    except (TypeError, IndexError, KeyError):
        return False
    if isinstance(inner, (str, bytes)):
        return False
    try:
        return len(inner) >= 2
    except TypeError:
        return False


def normalize_polygons(polygon=None, polygons=None) -> Optional[list]:
    """測區：``polygon`` = 單一 ring [(lat, lon), …]（舊介面）；``polygons`` = ring 的串列（多區，例如
    MainWindow.survey_area_polygons() 的回傳值：已完成的區＋正在畫且 ≥3 角點的區）。兩個都沒給 ⇒ None。

    回傳 [[(lat, lon), …], …]。不猜：兩個都給、空串列、點不是 (緯度, 經度) 數字、超出範圍都拒絕。
    """
    if polygon is not None and polygons is not None:
        raise PipelineError("測區只能給一個：polygon（單一 ring）或 polygons（多個 ring），不能兩個都給")
    if polygon is None and polygons is None:
        return None
    if polygon is not None:
        if _is_ring_list(polygon):
            raise PipelineError("polygon 是單一 ring [(緯度, 經度), …]，收到的是多個 ring：多區請用 polygons=")
        raw = [polygon]
    else:
        if isinstance(polygons, (str, bytes)):
            raise PipelineError(f"polygons 要是 ring 的串列，收到字串 {polygons!r}")
        raw = list(polygons)
        if not raw:
            raise PipelineError("polygons 是空的：沒有任何測區（要在被拍到的範圍內算視數就不要給測區）")
    rings = []
    for k, ring in enumerate(raw, start=1):
        where = "" if polygons is None else f"第 {k} 塊"
        pts = []
        try:
            for p in ring:
                lat, lon = p
                if isinstance(lat, bool) or isinstance(lon, bool):
                    raise TypeError
                pts.append((float(lat), float(lon)))
        except (TypeError, ValueError):
            raise PipelineError(f"測區{where}的角點要是（緯度, 經度）數字，收到 {ring!r}"[:300]) from None
        if len(pts) < 3:
            raise PipelineError(f"測區多邊形{where}至少要 3 點，收到 {len(pts)}")
        for lat, lon in pts:
            if not (math.isfinite(lat) and math.isfinite(lon)):
                raise PipelineError(f"測區{where}的角點不是有限數值：{(lat, lon)}")
            if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                raise PipelineError(f"測區角點超出範圍：{(lat, lon)}")
        rings.append(pts)
    return rings


def check_gps_lag_s(gps_lag_s) -> float:
    """FC GPST 的接收機延遲補償（秒）：0 ≤ lag ≤ GPS_LAG_MAX_S 的有限實數；其他（字串、NaN、負值、
    毫秒誤當秒）一律拒絕。"""
    if isinstance(gps_lag_s, bool) or not isinstance(gps_lag_s, (int, float, np.integer, np.floating)):
        raise PipelineError(f"GPS 接收延遲 gps_lag_s 要是秒數（實數），收到 {type(gps_lag_s).__name__} {gps_lag_s!r}")
    v = float(gps_lag_s)
    if not math.isfinite(v):
        raise PipelineError(f"GPS 接收延遲 gps_lag_s 不是有限數值：{gps_lag_s!r}")
    if not (0.0 <= v <= GPS_LAG_MAX_S):
        raise PipelineError(f"GPS 接收延遲 gps_lag_s = {v!r} s 超出合理範圍 [0, {GPS_LAG_MAX_S:g}] s"
                            "（單位是秒；GPS_DELAY_MS 的 120 要寫成 0.12）")
    return v


def _rel(p, root: Path) -> Optional[str]:
    if p is None:
        return None
    try:
        return Path(p).resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        return str(p)


def _check_inputs(sorties, out: Path, *, sensor_ts_semantics, height, geoid, geo_projection, ppk, ground_h_m,
                  views_k, cell_m, overwrite, input_sources=None):
    if not sorties:
        raise PipelineError("沒有任何架次")
    if sensor_ts_semantics not in pose_mod.SENSOR_TS_SEMANTICS:
        raise PipelineError(f"SensorTimestamp 語意必須明確選 {pose_mod.SENSOR_TS_SEMANTICS} 之一"
                            f"（台架 B3 未定案，沒有預設），收到 {sensor_ts_semantics!r}")
    if height not in HEIGHTS:
        raise PipelineError(f"高度必須是 {HEIGHTS} 之一，收到 {height!r}")
    if geoid is not None and not isinstance(geoid, datum.GeoidModel):
        raise PipelineError(f"大地起伏要是 survey.datum.GeoidModel 或 None，收到 {type(geoid).__name__}")
    if height == "orthometric" and geoid is None:
        raise PipelineError("選了正高但沒有大地起伏 N：無法算正高（不會用 N=0）")
    if ppk is not None and not isinstance(ppk, PpkSettings):
        raise PipelineError(f"ppk 要是 PpkSettings 或 None，收到 {type(ppk).__name__}")
    if ground_h_m is not None and not math.isfinite(float(ground_h_m)):
        raise PipelineError(f"地面高無效：{ground_h_m!r}")
    if input_sources is not None and not (isinstance(input_sources, dict) and all(
            isinstance(k, str) and isinstance(v, str) for k, v in input_sources.items())):
        raise PipelineError(f"input_sources 要是 {{欄位: 來源說明}} 的字串對照，收到 {input_sources!r}"[:300])
    if not views_k or any(int(k) < 1 for k in views_k):
        raise PipelineError(f"視數門檻要是 ≥1 的整數，收到 {views_k!r}")
    if not float(cell_m) > 0:
        raise PipelineError(f"格網大小要 > 0，收到 {cell_m!r}")
    names = [s.callsign for s in sorties]
    if len(set(names)) != len(names):
        raise PipelineError(f"呼號重複：{sorted(n for n in set(names) if names.count(n) > 1)}")
    for s in sorties:
        _validate_sortie(s, ppk)
    if ppk is not None and ppk.run_rtklib:
        if ppk.base_obs is not None and not Path(ppk.base_obs).is_file():
            raise PipelineError(f"基站 RINEX 不存在：{ppk.base_obs}")
        if ppk.base_ubx is not None and not Path(ppk.base_ubx).is_file():
            raise PipelineError(f"基站 UBX 不存在：{ppk.base_ubx}")
        try:
            rtklib.find_executable("rnx2rtkp", ppk.rnx2rtkp_exe)
            rtklib.find_executable("convbin", ppk.convbin_exe)
        except rtklib.RtklibNotFound as e:
            raise PipelineError(f"未安裝 RTKLIB：{e}") from None
    if not overwrite and ((out / "photos.csv").exists() or (out / "odm" / "geo.txt").exists()):
        raise PipelineError(f"輸出目錄 {out} 已有成果（photos.csv／odm/geo.txt）：請換目錄或明確選擇覆寫")


def process_campaign(sorties: Sequence, out_dir, *, sensor_ts_semantics: str, height: str = "ellipsoidal",
                     geoid: Optional[datum.GeoidModel] = None, geo_projection: Optional[str] = None,
                     ppk: Optional[PpkSettings] = None, ground_h_m: Optional[float] = None, polygon=None,
                     polygons=None, views_k: Sequence[int] = (5, 9), cell_m: float = 2.0,
                     exif_make: str = "Raspberry Pi", gps_lag_s: float = 0.0, verify_sha256: bool = True,
                     overwrite: bool = False, input_sources: Optional[dict] = None,
                     wind_limit_mps: float = qa_mod.WIND_LIMIT_MPS,
                     progress: Optional[Callable[[str, Optional[float]], None]] = None,
                     should_cancel: Optional[Callable[[], bool]] = None) -> CampaignResult:
    """多架次 → 一份 I1、一份 geo.txt、一個加標籤影像目錄、一份 qa.json。

    ``height``：geo.txt 與 EXIF 共用的高度（'ellipsoidal' = h_ell_m；'orthometric' = h_orth_m，需要 ``geoid``）。
    ``ground_h_m``：地面高（與 ``height`` 同一基準），給了才算視數與 GSD。
    測區：``polygon`` = 單一 ring [(lat, lon), …]，或 ``polygons`` = ring 的串列（多區取聯集；
    MainWindow.survey_area_polygons() 的形狀），只能給一個；給了用 exposure_verifier 在測區內算，
    否則在「被拍到的範圍」內算。有效值記在 effective['polygons']（一律是 ring 的串列）。
    ``gps_lag_s``：FC GPST 的接收機延遲補償（秒，0–1；0 = 不補），記在 effective。
    ``input_sources``：{欄位: 來源說明}（例如 GUI 的「操作員輸入」「取自飛控 GPS_DELAY_MS」），給了才記進 effective。
    ``wind_limit_mps``：建圖風速上限（預設 3 m/s，使用者 2026-10-15 的首飛決定）；超過只是旗標，記在 qa.json。
    ``progress(訊息, 0–1 或 None)``、``should_cancel() -> bool`` 給背景執行緒用。
    """
    out = Path(out_dir)
    sorties = [_as_sortie(s) for s in sorties]
    views_k = tuple(int(k) for k in views_k)
    if geo_projection is not None:
        geo_projection = check_geo_projection(geo_projection)
    rings = normalize_polygons(polygon, polygons)
    gps_lag_s = check_gps_lag_s(gps_lag_s)
    wind_limit_mps = check_wind_limit_mps(wind_limit_mps)
    _check_inputs(sorties, out, sensor_ts_semantics=sensor_ts_semantics, height=height, geoid=geoid,
                  geo_projection=geo_projection, ppk=ppk, ground_h_m=ground_h_m,
                  views_k=views_k, cell_m=cell_m, overwrite=overwrite, input_sources=input_sources)
    n_steps = len(sorties) + 4

    def step(k, msg):
        _check_cancel(should_cancel)
        _log.info("[航測處理] %s", msg)
        if progress is not None:
            progress(msg, min(k / n_steps, 1.0))

    effective = {
        "sensor_ts_semantics": sensor_ts_semantics, "height": height,
        "geoid": None if geoid is None else {"datum": geoid.datum, "source": geoid.source,
                                             "undulation_m": (None if callable(geoid.undulation_m)
                                                              else float(geoid.undulation_m))},
        "geo_projection": geo_projection, "exif_make": exif_make,
        "ground_h_m": None if ground_h_m is None else float(ground_h_m),
        "ground_datum": _HEIGHT_KEY[height] if ground_h_m is not None else None,
        "polygons": None if rings is None else [[[a, b] for a, b in ring] for ring in rings],
        "views_k": list(views_k), "cell_m": float(cell_m), "gps_lag_s": gps_lag_s,
        "verify_sha256": bool(verify_sha256), "overwrite": bool(overwrite),
        "ppk": None if ppk is None else ppk.effective(),
    }
    if input_sources is not None:
        effective["input_sources"] = dict(sorted(input_sources.items()))
    _log.info("[航測處理] 有效設定：%s", json.dumps(effective, ensure_ascii=False, sort_keys=True, default=_jsonable))
    out.mkdir(parents=True, exist_ok=True)

    base_rinex, ppk_work = None, out / "ppk"
    if ppk is not None and ppk.run_rtklib and any(not s.pos_path for s in sorties):
        step(0, "PPK：準備基站觀測（RTKLIB）…")
        if ppk.base_ubx is not None:
            obs, nav, cmd = _convbin_base(ppk, ppk_work / "base")
            base_rinex = (obs, (nav,))
            effective["ppk"]["base_conversion"] = {"obs": _rel(obs, out), "nav": _rel(nav, out),
                                                   "command": list(map(str, cmd))}
        else:
            base_rinex = (Path(ppk.base_obs), ())

    results = []
    for k, s in enumerate(sorties):
        step(k, f"{s.callsign}：讀日誌、配對、算位姿{'、PPK' if ppk is not None else ''}…")
        results.append(process_sortie(s, sensor_ts_semantics=sensor_ts_semantics, geoid=geoid, ppk=ppk,
                                      work_dir=ppk_work, base_rinex=base_rinex, gps_lag_s=gps_lag_s,
                                      verify_sha256=verify_sha256, should_cancel=should_cancel,
                                      wind_limit_mps=wind_limit_mps))
    rows = [r for sr in results for r in sr.rows]
    sessions = [sr.ingest.session for sr in results]
    warnings = [w for sr in results for w in sr.warnings]
    errors = []
    effective["sorties"] = {sr.callsign: sr.effective for sr in results}

    step(len(sorties), "寫 I1 交付物（photos.csv／GeoJSON）…")
    i1 = deliverable.write_i1_deliverable(out, rows, meta=effective)
    # 風的設定只進 qa.json（I1 寫完才加）：它不影響照片位姿，photos.meta.json 與加入這個功能前逐位元相同
    effective["wind_qa"] = {"limit_mps": wind_limit_mps, "source": "飛控 EKF 風估計（XKF2／NKF2 的 VWN／VWE）",
                            "max_gap_s": qa_mod.WIND_MAX_GAP_S, "flag_only": True}
    paths = {"photos_csv": i1["csv"], "photos_geojson": i1["geojson"], "photos_meta": i1["meta"],
             "geo_txt": None, "images_dir": None, "qa_json": out / "qa.json"}

    step(len(sorties) + 1, "寫 geo.txt 與加標籤影像…")
    images = []
    try:
        header, geo_rows, notes = _geo_rows(rows, geo_projection)
        warnings += notes
        geo_p = out / "odm" / "geo.txt"
        geotag.write_geo_txt(geo_p, geo_rows, height=height, projection=header)
        paths["geo_txt"] = geo_p
        effective["geo_txt_accuracy"] = dict(geotag.ACCURACY_SEMANTICS)   # 精度欄的換算（水平 HRMS/√2）照實記
    except ValueError as e:
        errors.append(f"geo.txt 未寫：{e}")
    if paths["geo_txt"] is not None:
        img_dir = out / "odm" / "images"
        try:
            for sr in results:
                _check_cancel(should_cancel)
                images += exif_write.export_tagged_images(sr.ingest.session, sr.rows, img_dir, make=exif_make,
                                                          height=height, overwrite=overwrite)
            paths["images_dir"] = img_dir
        except (ValueError, OSError) as e:
            errors.append(f"加標籤影像未完成：{e}")

    step(len(sorties) + 2, "計算視數與 QA…")
    exposure = _views(rows, i1["csv"], sessions, height, ground_h_m, rings, views_k, cell_m)
    try:
        readout, readout_err = odm_client.rolling_shutter_readout([s.readout_ms for s in sessions]), None
    except ValueError as e:
        readout, readout_err = None, str(e)
    odm = {"rolling_shutter_readout_ms": readout, "rolling_shutter_error": readout_err,
           "gsd_cm": _gsd_cm(rows, sessions, height, ground_h_m), "n_images": len(images)}
    reasons = [r for sr in results for r in sr.reasons] + [f"交付物：{e}" for e in errors]
    qa = {"passed": not reasons, "reasons": reasons,
          "per_sortie": {sr.callsign: {"passed": sr.passed, "reasons": sr.reasons, "report": sr.qa, "ppk": sr.ppk,
                                       "wind": sr.wind}
                         for sr in results},
          "exposure": exposure}
    for w in warnings:
        _log.warning("[航測處理] %s", w)
    for e in errors:
        _log.error("[航測處理] %s", e)

    _dump_json(paths["qa_json"], {
        "format": QA_FORMAT, "effective": effective, "qa": qa, "odm": odm, "warnings": warnings, "errors": errors,
        "files": {k: _rel(v, out) for k, v in paths.items()}, "images": [p.name for p in images],
        "n_photos": len(rows), "sources": dict(sorted(Counter(r["source"] for r in rows).items())),
    })
    step(n_steps, "完成" + ("" if qa["passed"] else f"（QA 未通過：{len(reasons)} 項）"))
    return CampaignResult(out_dir=out, sorties=results, rows=rows, paths=paths, images=images, effective=effective,
                          qa=qa, odm=odm, warnings=warnings, errors=errors)


# ─────────────────────────────────────────────────────────────────────────────
# 結果表（對話框用；純資料，可測）
# ─────────────────────────────────────────────────────────────────────────────

def qa_table_rows(result: CampaignResult) -> list:
    """QA 表的列：(機, 項目, 數值, 結果)；結果 True＝通過、False＝未通過、None＝參考值。"""
    out = []
    for sr in result.sorties:
        cs, rep = sr.callsign, sr.qa
        for c in rep["conservation"]:
            where = f"第 {c['strip']} 帶" if c["strip"] is not None else "帶外"
            trig = f"／TRIG {c['fc_trig']}" if c["fc_trig"] is not None else ""
            out.append((cs, f"張數守恆 {where}",
                        f"飛控 {c['fc_cam']}{trig}／Pi {c['pi_edges']}／JPEG {c['jpeg_ok']}"
                        f"（未配對 飛控 {c['unmatched_fc']}、Pi {c['unmatched_pi']}）", bool(c["conserved"])))
        ufc = rep["unmatched_fc"]
        out.append((cs, "未配對飛控事件",
                    "無" if not ufc else "；".join(f"#{u['index']} TimeUS {u['TimeUS']}" for u in ufc[:20])
                    + ("…" if len(ufc) > 20 else ""), not ufc))
        upi = rep["unmatched_pi"]
        out.append((cs, "未配對 Pi 邊緣",
                    "無" if not upi else "；".join(f"seq {u['seq']}" for u in upi[:20]) + ("…" if len(upi) > 20 else ""),
                    not upi))
        for s in rep["spacing"]:
            val = ("不到兩張" if s["median_m"] is None else
                   f"中位數 {s['median_m']:.2f} m／p99 {s['p99_m']:.2f} m（d = {s['d_m']:.2f} m）")
            out.append((cs, f"間距 第 {s['strip']} 帶", val, bool(s["median_ok"] and s["p99_ok"])))
        ro = rep["roll"]
        out.append((cs, f"側傾 > {ro['limit_deg']:g}° 比例",
                    "—" if ro["share"] is None else f"{ro['share']:.1%}（{ro['n_gt']}/{ro['n']}，每帶第一張不計）", None))
        f = rep["fit"]
        out.append((cs, "時鐘擬合殘差", f"RMS {f['rms_ms']:.3f} ms／最大 {f['max_abs_ms']:.3f} ms／漂移 "
                    f"{f['drift_ppm']:+.1f} ppm／{f['n_pairs']} 對", bool(f["rms_ok"])))
        out += _wind_rows(cs, getattr(sr, "wind", None))
        if sr.ppk is not None:
            if sr.ppk.get("error"):
                out.append((cs, "PPK", sr.ppk["error"], False))
            else:
                for w in sr.ppk["fix_windows"]:
                    val = ("沒有 epoch" if w["fix_ratio"] is None else
                           f"{w['fix_ratio']:.1%}（{w['n_fix']}/{w['n_epochs']} epoch；照片未 fix "
                           f"{w['n_photos_not_fixed']}/{w['n_photos']}）")
                    out.append((cs, f"PPK fix 比例 第 {w['label']} 帶", val, not w["below_threshold"]))
                rej = "、".join(f"{k} {n}" for k, n in sr.ppk["rejected"].items()) or "無"
                out.append((cs, "PPK 採用", f"採用 {sr.ppk['used']} 張；保留 fc_interp：{rej}", None))
                fc = sr.ppk.get("fc_check")
                if fc and fc["n"]:
                    mh = "—" if fc["median_h_m"] is None else f"{fc['median_h_m']:.2f} m"
                    mv = "—" if fc["median_v_m"] is None else f"{fc['median_v_m']:+.2f} m"
                    out.append((cs, "PPK 對飛控粗差（中位數）", f"水平 {mh}／垂直 {mv}（{fc['n']} 張；上限 水平 "
                                f"{fc['limit_h_m']:g} m、垂直 {fc['limit_v_m']:g} m）", bool(fc["passed"])))
                tr = sr.ppk.get("trajectory") or {}
                rc = tr.get("ref_pos_check")
                if rc:
                    out.append((cs, "PPK 基站座標（% ref pos）", f"水平 {rc['d_h_m'] * 1e3:.1f} mm／垂直 "
                                f"{rc['d_v_m'] * 1e3:+.1f} mm（{rc['base_frame']}）", bool(rc["passed"])))
                sp = tr.get("rover_span")
                if sp and sp.get("obs_start_gpst_s") is not None and sp.get("rawx_first_gpst_s") is not None:
                    out.append((cs, "PPK 觀測跨度（obs start／end 對 rover.ubx）",
                                f"開始 {sp['obs_start_gpst_s'] - sp['rawx_first_gpst_s']:+.3f} s／結束 "
                                f"{sp['obs_end_gpst_s'] - sp['rawx_last_gpst_s']:+.3f} s；第一筆解 "
                                f"{sp['first_solution_gpst_s'] - sp['obs_start_gpst_s']:+.3f} s", True))
    ex = result.qa.get("exposure") or {}
    if ex.get("computed"):
        for k, v in ex["frac_ge"].items():
            out.append(("全部", f"視數 ≥ {k} 比例（{ex['scope']}）", f"{v:.1%}", None))
    else:
        out.append(("全部", "視數", ex.get("reason", "未計算"), None))
    for e in result.errors:
        out.append(("全部", "交付物", e, False))
    return out


def _wind_rows(cs: str, wind: Optional[dict]) -> list:
    """風的 QA 表列（結果欄一律 None＝參考值：超過上限是旗標，不是 QA 未通過；值前面加 ⚠ 標示）。"""
    if wind is None:
        return []
    lim = wind["limit_mps"]
    if not wind.get("available"):
        return [(cs, f"EKF 風速（上限 {lim:g} m/s）", f"⚠ 無法確認：{wind.get('reason')}", None)]
    share = wind["share_above"]
    head = "⚠ " if wind["flagged"] else ""
    share_txt = "—" if share is None else f"{share:.1%}"

    def _mps(v) -> str:
        return "—" if v is None else f"{v:.1f}"

    # 有 EKF 風但沒有任何照片取得到風值（零照片、或 XKF2 間隔 > max_gap）⇒ 中位／最大是 None，不能拿去格式化
    out = [(cs, f"EKF 風速（上限 {lim:g} m/s）",
            f"{head}照片 {share_txt} 超過（{wind['n_photos_above']}/{wind['n_photos_with_wind']}）；照片處中位 "
            f"{_mps(wind['photo_speed_median_mps'])}、最大 {_mps(wind['photo_speed_max_mps'])} m/s"
            f"（{wind['core_selection']}）"
            + ("；⚠ 沒有任何照片取得到風估計" if not wind['n_photos_with_wind'] else ""), None)]
    for r in wind["per_strip"]:
        if r["mean_speed_mps"] is None:
            val = "這段沒有風估計樣本"
        else:
            val = (f"{'⚠ ' if r['n_photos_above'] else ''}平均 {r['mean_speed_mps']:.1f} m/s、來自 "
                   f"{r['dir_from_deg']:.0f}°、最大 {r['max_speed_mps']:.1f} m/s；照片超過 "
                   f"{r['n_photos_above']}/{r['n_photos']}")
        out.append((cs, f"風 第 {r['strip']} 帶", val, None))
    return out


# ─────────────────────────────────────────────────────────────────────────────
# NodeODM
# ─────────────────────────────────────────────────────────────────────────────

def _image_sizes(result: CampaignResult) -> dict:
    """上傳影像檔名 → (寬, 高) 像素（取該架會期的相機設定；查不到的不列）。"""
    by_id = {}
    for sr in result.sorties or []:
        try:
            cam = sr.ingest.session.camera
            wh = (int(cam["width"]), int(cam["height"]))
        except (AttributeError, KeyError, TypeError, ValueError):
            continue
        for r in sr.rows:
            by_id[str(r.get("image_id"))] = wh
    return {Path(p).name: by_id[Path(p).stem] for p in result.images if Path(p).stem in by_id}


def check_gcp_for_job(result: CampaignResult, gcp_txt, *, gcp_vertical: Optional[str],
                      checkpoint_ids: Sequence[str] = ()) -> dict:
    """送 ODM 前核對 gcp_list.txt（survey.gcp_plan.validate_gcp_list）；有錯 ⇒ PipelineError。回傳核對摘要。"""
    from survey import gcp_plan
    try:
        rep = gcp_plan.validate_gcp_list(gcp_txt, image_names=[Path(p).name for p in result.images],
                                         image_sizes=_image_sizes(result), gcp_vertical=gcp_vertical,
                                         height=(result.effective or {}).get("height"),
                                         forbidden_names=list(checkpoint_ids))
    except gcp_plan.GcpPlanError as e:
        raise PipelineError(f"GCP 檔：{e}") from None
    if gcp_vertical == "orthometric":
        # 「正高」還要同一個系統：geo.txt 用的 GeoidModel.datum 與 gcp_list 宣告的（EGM96／TWVD2001）要相同
        geo_sys = ((result.effective or {}).get("geoid") or {}).get("datum")
        gcp_sys = gcp_plan.declared_ortho_system(gcp_txt)
        if gcp_sys is None:
            rep["warnings"].append(f"gcp_list.txt 沒宣告是哪一個正高系統（geo.txt 用 {geo_sys or '未知'}）："
                                   "請確認兩者同為 EGM96 或同為 TWVD2001")
        elif geo_sys is not None and gcp_sys != geo_sys:
            rep["errors"].append(f"gcp_list.txt 的正高是 {gcp_sys}，geo.txt 用的是 {geo_sys}：不同正高系統不混用")
    if rep["errors"]:
        raise PipelineError("GCP 檔不能送：" + "；".join(rep["errors"][:5])
                            + ("…" if len(rep["errors"]) > 5 else ""))
    return rep


#: 2026-10-20 之前的 geo.txt：水平精度欄寫的是二維 HRMS（√(σE²+σN²)），ODX 卻當成 x、y 各自的 σ ⇒ 先驗權重低估 2 倍
GEO_ACCURACY_LEGACY = {"horizontal": "hrms_2d", "vertical": "sigma_v_1d", "recorded": False,
                       "note": "處理結果沒有記錄 geo.txt 精度欄的意義（2026-10-20 之前的版本）：水平欄是二維 HRMS，"
                               "ODX 當成單軸 σ 用 ⇒ 照片位置的先驗權重被低估約 2 倍（σ 大了 √2）；重跑處理可改正"}

FORCE_GPS_UNDECIDED = (
    "選了 GCP 檔，但還沒決定照片位置（PPK）要不要一起進平差：有 GCP 時 ODX 預設 bundle_use_gps: no，照片 GPS "
    "只用來初始對位、不進平差（X modules/osfm.py:329-337）。請明確選「--force-gps：照片 GPS 與 GCP 一起平差」"
    "或「只用 GCP」")


def inspect_geo_txt(path, image_names) -> dict:
    """核對 geo.txt 與上傳影像：每張影像都要有一行；統計有幾行帶每張的精度欄。

    ODX 只有在 ≥ 9 欄（yaw/pitch/roll 佔位也要在）時才讀第 8–9 欄的水平／垂直精度（X modules/geo.py:42-51）；
    沒有 geo.txt 行的影像會靜默改用 EXIF GPS 與 --gps-accuracy 預設（X stages/dataset.py:177-183）。
    """
    p = Path(path)
    try:
        lines = p.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeDecodeError) as e:
        raise PipelineError(f"讀不到 geo.txt：{e}") from None
    entries, bad = {}, []
    for k, ln in enumerate(lines[1:], start=2):
        parts = ln.split()
        if not parts:
            continue
        entries[parts[0]] = len(parts)
        if len(parts) >= 9:
            # 精度欄要是有限的正數：nan／0／負數／文字會讓 ODX 拿到 NaN 或零 σ（權重無限大），或在讀檔時當掉
            try:
                ok = all(math.isfinite(v) and v > 0 for v in (float(parts[7]), float(parts[8])))
            except ValueError:
                ok = False
            if not ok:
                bad.append(f"第 {k} 行 {parts[0]}：{parts[7]} {parts[8]}")
    names = [str(n) for n in image_names]
    missing = sorted(set(names) - set(entries))
    n_acc = sum(1 for n in names if entries.get(n, 0) >= 9)
    accuracy = "per_image" if names and n_acc == len(names) else ("none" if n_acc == 0 else "partial")
    return {"header": lines[0].strip() if lines else "", "n_entries": len(entries), "n_images": len(names),
            "n_with_accuracy": n_acc, "accuracy": accuracy, "n_missing": len(missing), "missing_images": missing[:20],
            "n_extra_entries": len(set(entries) - set(names)), "n_bad_accuracy": len(bad), "bad_accuracy": bad[:20]}


def _force_gps_decision(has_gcp: bool, force_gps: Optional[bool]) -> dict:
    if not has_gcp:
        return {"decision": "not_applicable_no_gcp", "sent": False, "value": None, "source": "rule",
                "operator_choice": force_gps,
                "meaning": "沒有 GCP 檔：平差只用照片 GPS，--force-gps 沒有作用（X modules/osfm.py:329-337），不送"}
    if force_gps:
        return {"decision": "force_gps", "sent": True, "value": True, "source": "operator",
                "meaning": "送 --force-gps：照片位置（PPK）與 GCP 一起平差，OpenSfM 另估每台相機的 GPS 偏差"
                           "（bundle_compensate_gps_bias）⇒ 常數偏移被吸收、不出現在 gps_errors，要看 camera_errors.bias"
                           "（X modules/osfm.py:329-337；S opensfm/stats.py:53-62）"}
    return {"decision": "gcp_only", "sent": False, "value": False, "source": "operator",
            "meaning": "不送 --force-gps：有 GCP 時照片位置只用於初始對位與配對、不進平差（bundle_use_gps: no）⇒ "
                       "gps_errors 是 PPK 對 GCP 解的獨立檢核"}


def _max_concurrency_decision(max_concurrency) -> dict:
    if max_concurrency is None:
        return {"sent": False, "value": None, "source": "engine_default",
                "meaning": "不送：引擎預設＝節點的 CPU 數（X modules/context.py:44）；節點若設了 --max_concurrency 上限，"
                           "NodeODX 會自己補上（N odmInfo.js:327-334），提交後的 info.options 會記下實際值"}
    if isinstance(max_concurrency, bool) or not isinstance(max_concurrency, int) or max_concurrency < 1:
        raise PipelineError(f"max_concurrency 必須是 ≥ 1 的整數（不設定請給 None），收到 {max_concurrency!r}")
    return {"sent": True, "value": int(max_concurrency), "source": "operator",
            "meaning": "送 --max-concurrency；節點若有更低的上限會被靜默壓低（N odmInfo.js:308-315），提交後對照 info.options。"
                       "峰值記憶體經驗值：每個執行緒、每 2 MP 影像約 1 GB（X modules/config.py:309-316 說明）"}


def _gps_accuracy_decision(geo: dict) -> dict:
    if geo["accuracy"] == "per_image":
        return {"sent": False, "value": None, "source": "geo.txt",
                "semantics": geo.get("accuracy_semantics"),
                "meaning": "geo.txt 每張都有水平／垂直精度欄：明確送 --gps-accuracy 會蓋掉每張的值"
                           "（X stages/dataset.py:188-193），所以不送。ODX 把水平欄當成 x、y 各自的 σ"
                           "（gps_xy_stddev，R1 §6.2／§8.2）⇒ 我們寫的是單軸 σ（見 semantics）"}
    if geo["accuracy"] == "none":
        return {"sent": False, "value": None, "source": "engine_default",
                "meaning": "geo.txt 沒有精度欄：引擎用 --gps-accuracy 預設 3 m（水平），垂直 ×3（X modules/photo.py:836-857）"}
    return {"sent": False, "value": None, "source": "geo.txt+engine_default",
            "meaning": f"只有 {geo['n_with_accuracy']}/{geo['n_images']} 張在 geo.txt 有精度欄，其餘用引擎預設 3 m／9 m"}


def odm_job(result: CampaignResult, *, node_url: str, orthophoto_resolution_cm: float, rolling_shutter: bool,
            qa_override: bool = False, name: Optional[str] = None, gcp_txt=None, gcp_vertical: Optional[str] = None,
            checkpoint_ids: Sequence[str] = (), force_gps: Optional[bool] = None,
            max_concurrency: Optional[int] = None) -> dict:
    """組 NodeODX 任務。QA 未通過時必須 ``qa_override=True`` 才組得出來（並記在任務紀錄裡）。

    ``gcp_txt``：使用者標好影像座標的 gcp_list.txt（選填）；``gcp_vertical`` 是它 geo_z 的高程基準
    （'ellipsoidal'／'orthometric'，必填且要等於 geo.txt 用的高度）。核對摘要放在 ``job['gcp']``。

    ``force_gps``（三態）：有 GCP 時**必須**明確給 True（送 --force-gps）或 False（只用 GCP）；None ⇒ 拒絕組任務。
    沒有 GCP 時不適用、不送，記成 ``not_applicable_no_gcp``。``max_concurrency``：None ⇒ 不送（引擎預設），
    整數 ≥ 1 ⇒ 送。兩者與 gps-accuracy 的「送／不送與理由」都記在任務裡（``force_gps``、``max_concurrency``、
    ``gps_accuracy``），也就進了 odm/submission.json。沒給 GCP、沒設 max_concurrency 時送出的 options 與加入這些功能前相同。

    權杖不在任務裡：呼叫端用 ``NodeOdmClient(url, token=...)`` 建客戶端交給 :func:`run_odm_job`。
    """
    try:
        url = odm_client.sanitize_node_url(node_url)
    except ValueError as e:
        raise PipelineError(str(e)) from None
    if result.paths.get("geo_txt") is None or not result.images:
        raise PipelineError("沒有 geo.txt 或加標籤影像（見處理錯誤），不能送 ODM")
    if not result.passed and not qa_override:
        raise PipelineError("QA 未通過：" + "；".join(result.qa["reasons"][:3])
                            + ("…" if len(result.qa["reasons"]) > 3 else "")
                            + "（要送出必須明確勾選「QA 未通過仍送出」）")
    readout = result.odm.get("rolling_shutter_readout_ms")
    if rolling_shutter and readout is None:
        raise PipelineError(f"不能開滾動快門校正：{result.odm.get('rolling_shutter_error')}")
    has_gcp = gcp_txt is not None and bool(str(gcp_txt).strip())
    gcp_rep = None
    if has_gcp:
        gcp_rep = check_gcp_for_job(result, gcp_txt, gcp_vertical=gcp_vertical, checkpoint_ids=checkpoint_ids)
        if force_gps is None:
            raise PipelineError(FORCE_GPS_UNDECIDED)
    elif gcp_vertical is not None or checkpoint_ids:
        raise PipelineError("給了 GCP 高程基準或檢核點清單，卻沒有 gcp_list.txt")
    if force_gps is not None and not isinstance(force_gps, bool):
        raise PipelineError(f"force_gps 只能是 True／False／None，收到 {force_gps!r}")
    mc = _max_concurrency_decision(max_concurrency)
    geo = inspect_geo_txt(result.paths["geo_txt"], [Path(p).name for p in result.images])
    if geo["n_missing"]:
        raise PipelineError(f"geo.txt 缺 {geo['n_missing']} 張上傳影像的位置（例如 {geo['missing_images'][:3]}）："
                            "沒有 geo.txt 行的影像會被 ODX 靜默改用 EXIF GPS 與預設精度，不送")
    if geo["n_bad_accuracy"]:
        raise PipelineError(f"geo.txt 有 {geo['n_bad_accuracy']} 行的精度欄不是有限的正數（例如 {geo['bad_accuracy'][:3]}）："
                            "ODX 會拿到 NaN 或零 σ（權重無限大）或讀檔失敗，不送")
    # 精度欄的意義是「寫 geo.txt 那次處理」決定的（qa.json effective.geo_txt_accuracy）；舊的處理結果沒記 ⇒ 照實記成舊慣例
    geo["accuracy_semantics"] = (result.effective or {}).get("geo_txt_accuracy") or GEO_ACCURACY_LEGACY
    try:
        opts = odm_client.build_options(orthophoto_resolution_cm=float(orthophoto_resolution_cm),
                                        rolling_shutter_readout_ms=readout if rolling_shutter else None,
                                        force_gps=bool(has_gcp and force_gps), max_concurrency=mc["value"])
    except ValueError as e:
        raise PipelineError(f"ODM 參數：{e}") from None
    job = {"node_url": url, "images": [str(p) for p in result.images], "geo_txt": str(result.paths["geo_txt"]),
           "options": opts, "name": name, "qa_passed": result.passed, "qa_override": bool(qa_override),
           "qa_reasons": list(result.qa["reasons"]) if not result.passed else [],
           "rolling_shutter": bool(rolling_shutter), "out_dir": str(result.out_dir),
           "force_gps": _force_gps_decision(has_gcp, force_gps), "max_concurrency": mc,
           "gps_accuracy": _gps_accuracy_decision(geo), "geo_txt_check": geo}
    if has_gcp:
        job["gcp_txt"] = str(gcp_txt)
        job["gcp"] = {k: gcp_rep[k] for k in ("header", "crs", "n_lines", "n_gcps", "gcp_vertical", "images_per_gcp",
                                              "engine_checkpoints", "warnings")}
    return job


def _check_extract_space(zip_path, dest) -> None:
    """解壓前看磁碟：成員解開後的總大小 > 可用空間 ⇒ PipelineError（不要解到一半才 ENOSPC，留下半套成果）。"""
    try:
        with zipfile.ZipFile(zip_path) as zf:
            need = sum(int(m.file_size) for m in zf.infolist())
    except (OSError, zipfile.BadZipFile):
        return                                               # 讀不了 zip：交給 extract_odm_zip 報錯
    free = odm_client.free_bytes(dest)
    if free is not None and need > free:
        raise PipelineError(f"磁碟空間不足：解壓 {Path(zip_path).name} 需要 {need / 2**30:.2f} GiB，{Path(dest).parent} 只剩 "
                            f"{free / 2**30:.2f} GiB；沒有解壓（all.zip 留著，清出空間後「接回任務」會直接從本機 zip 解壓）")


def extract_odm_zip(zip_path, dest) -> Path:
    """解開 all.zip；任何成員路徑跳出 ``dest`` 就整個拒絕（zip slip）。"""
    dest = Path(dest)
    dest.mkdir(parents=True, exist_ok=True)
    root = dest.resolve()
    with zipfile.ZipFile(zip_path) as zf:
        for m in zf.infolist():
            if not (root / m.filename).resolve().is_relative_to(root):
                raise PipelineError(f"ODM 成果 zip 內有跳出目錄的路徑：{m.filename!r}，拒絕解壓")
        zf.extractall(root)
    return root


def find_ortho_tiles(root) -> Optional[Path]:
    """在 ODM 成果裡找正射瓦片資料夾（orthophoto_tiles/，底下是數字層級資料夾）。"""
    root = Path(root)
    cands = [root / "orthophoto_tiles"] + sorted(p for p in root.rglob("orthophoto_tiles") if p.is_dir())
    for c in cands:
        if c.is_dir() and any(e.is_dir() and e.name.isdigit() for e in c.iterdir()):
            return c
    return None


def find_odm_dsm(root) -> Optional[Path]:
    """在 ODM 成果裡找 DSM（odm_dem/dsm.tif；任務沒開 --dsm 就沒有）。"""
    root = Path(root)
    for c in [root / "odm_dem" / "dsm.tif"] + sorted(p for p in root.rglob("dsm.tif") if p.is_file()):
        if c.is_file():
            return c
    return None


# ─────────────────────────────────────────────────────────────────────────────
# NodeODX 任務紀錄（odm/submission.json）
# ─────────────────────────────────────────────────────────────────────────────
#
# 每一步都先寫紀錄再做下一步（原子寫入：先寫 .tmp 再取代）；權杖永遠不進紀錄（字串值逐一遮蔽、位址先去掉查詢字串）。
# phase：
#   planned            本機已產生 uuid，還沒對節點送任何請求
#   initialized        /task/new/init 成功（節點上有未提交的上傳）
#   uploading          上傳中（upload.batches 逐批記錄）
#   uploaded           全部上傳完成、還沒送 commit
#   committing         正要送 commit（停在這裡 ⇒ 接回時用 info 判斷其實有沒有提交）
#   committed          提交成功（節點在背景搬檔、檢查 maxImages）
#   running            節點回報排隊／執行中（status_history 記狀態改變與每 10 % 里程碑）
#   completed          節點回報完成、成果還沒下載
#   downloaded         all.zip 已下載、解壓、讀過品質報告（終點）
#   failed             節點回報失敗，或提交後核對不過（張數、被丟掉的參數）而由我們取消並刪除
#   abort_pending      提交後核對不過，但取消與刪除**都**沒成功（遠端可能還在用錯的輸入跑）⇒ 仍算進行中：
#                      擋新任務、可取消遠端、接回時再試一次取消＋刪除（審查 T6）
#   canceled           節點回報已取消
#   upload_failed／upload_canceled   提交前失敗／取消
#   postprocess_failed 已下載但解壓或後處理失敗（可接回重下）
#   lost               節點查不到任務（過期、被刪、節點當機時遺失）

SUBMISSION_FORMAT = "aeroplan-odm-submission/2"
QUALITY_FORMAT = "aeroplan-odm-quality/1"
ODM_CONSOLE_TAIL_LINES = 50
ODM_POLL_TRANSPORT_RETRIES = 6
_UNCOMMITTED_PHASES = frozenset({"planned", "initialized", "uploading", "uploaded", "upload_failed", "upload_canceled"})
_LIVE_PHASES = frozenset({"committing", "committed", "running", "completed", "abort_pending"})
_REATTACH_PHASES = frozenset({"committing", "committed", "running", "completed", "postprocess_failed", "downloaded",
                              "abort_pending"})
ODM_PHASE_TEXT = {
    "planned": "準備中（尚未連線）", "initialized": "已建立上傳（未提交）", "uploading": "上傳中（未提交）",
    "uploaded": "已上傳、未提交", "committing": "提交中", "committed": "已提交", "running": "節點處理中",
    "completed": "已完成、尚未下載", "downloaded": "已下載", "failed": "失敗", "canceled": "已取消",
    "upload_failed": "上傳失敗（未提交）", "upload_canceled": "上傳已取消（未提交）",
    "postprocess_failed": "已完成但解壓／後處理失敗", "lost": "節點上查不到任務",
    "abort_pending": "核對不過、取消遠端沒成功（遠端可能還在跑）",
}
# 進度條（0–1）的分段：上傳、節點處理（info.progress 0–100）、下載、解壓與品質報告
_F_UPLOAD, _F_REMOTE, _F_DOWNLOAD = (0.0, 0.30), (0.30, 0.90), (0.90, 0.98)
_UNCOMMITTED_NOTE = ("NodeODX 沒有刪除「未提交上傳」的 API（/task/remove 只認得已提交的任務），節點會在 "
                     "cleanupUploadsAfter（預設 2880 分＝48 h）後自動清掉 tmp/<uuid>")


def _now() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def _scrub(obj):
    """紀錄裡的每個字串值都遮蔽 ``token=...``（URL 帶的權杖）；鍵是我們自己的固定名稱。

    **不做**權杖原值的逐字替換：權杖從來不進任務資料（只在客戶端物件裡），例外訊息在客戶端就已經遮過
    （odm_client.redact）；對整份紀錄做原值替換時，短權杖（"odm"、"aeroplan"、"2026"）會改掉 format、
    路徑、階段名與時間戳，紀錄讀不回來 ⇒ 接回、取消遠端、防止遠端任務失聯全部失效（2026-10-20 審查 T9）。"""
    if isinstance(obj, str):
        return odm_client.redact(obj)
    if isinstance(obj, dict):
        return {k: _scrub(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_scrub(v) for v in obj]
    if isinstance(obj, Path):
        return odm_client.redact(str(obj))
    return obj


def _write_json_atomic(path: Path, obj) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(_scrub(obj), ensure_ascii=False, indent=2, sort_keys=True, default=_jsonable) + "\n"
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(text, encoding="utf-8", newline="\n")
    for k in range(10):
        try:
            os.replace(tmp, path)
            break
        except PermissionError:                      # Windows：別的程序剛好開著舊檔
            if k == 9:
                raise
            time.sleep(0.05)
    return path


def submission_path(where) -> Path:
    """``where`` 可以是處理輸出資料夾、其下的 odm/ 資料夾、或 submission.json 本身。"""
    p = Path(where)
    if p.is_file() or p.suffix.lower() == ".json":
        return p
    if p.name == "odm" and (p / "submission.json").exists():
        return p / "submission.json"
    return p / "odm" / "submission.json"


def _normalize_record(data: dict) -> dict:
    """舊版（/1，沒有 format 與 phase）紀錄：有 uuid 就是單一 POST 已提交的任務；有 extract_dir 就是已下載。"""
    if not isinstance(data, dict):
        raise PipelineError("odm/submission.json 不是物件")
    if data.get("format") in (None, "aeroplan-odm-submission/1"):
        data = dict(data)
        data["format"] = "aeroplan-odm-submission/1"
        if not data.get("phase"):
            data["phase"] = ("downloaded" if data.get("extract_dir") else "committed") if data.get("uuid") else "planned"
            data["phase_inferred_from_legacy_record"] = True
    elif data.get("format") != SUBMISSION_FORMAT:
        raise PipelineError(f"odm/submission.json 的格式 {data.get('format')!r} 不認得")
    return data


def load_submission(where) -> dict:
    """讀 odm/submission.json（含舊版）。找不到或壞掉 ⇒ PipelineError。"""
    p = submission_path(where)
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise PipelineError(f"找不到任務紀錄 {p}（不是 ODM 送出過的輸出資料夾？）") from None
    except (OSError, ValueError) as e:
        raise PipelineError(f"任務紀錄 {p} 讀不到或不是 JSON：{e}") from None
    return _normalize_record(data)


def odm_submission_status(record: dict) -> dict:
    """給對話框顯示用的摘要：uuid、階段、遠端狀態與進度、引擎版本、能不能接回／取消。純函式。"""
    phase = record.get("phase")
    remote = record.get("remote") or {}
    node = record.get("node") or {}
    code = remote.get("status_code")
    can_reattach = bool(record.get("uuid")) and phase in _REATTACH_PHASES
    up = record.get("upload") or {}
    upload_complete = (bool(up.get("n_batches_planned")) and isinstance(up.get("batches"), list)
                       and len(up["batches"]) == int(up["n_batches_planned"]))
    if bool(record.get("uuid")) and phase in ("uploading", "uploaded", "upload_failed") and upload_complete:
        can_reattach = True
        reason = "上傳已完整但沒有提交：「接回」時若節點上還有這個上傳就補送 commit（不必重傳）"
    elif phase == "abort_pending":
        reason = "提交後核對不過、取消遠端任務沒成功：「接回」或「取消遠端任務」會再試一次取消並刪除（不會下載成果）"
    elif can_reattach:
        reason = None
    elif phase in _UNCOMMITTED_PHASES:
        reason = "任務沒有提交，上傳無法接續：請重新送出"
    elif phase in ("failed", "canceled", "lost"):
        reason = f"任務{ODM_PHASE_TEXT.get(phase, phase)}：請重新送出"
    else:
        reason = f"紀錄階段 {phase!r} 不能接回"
    return {"uuid": record.get("uuid"), "phase": phase, "phase_text": ODM_PHASE_TEXT.get(phase, str(phase)),
            "node_url": record.get("node_url"), "token_used": bool(record.get("token_used")),
            "status_code": code, "status_text": odm_client.STATUS_NAMES.get(code) if code is not None else None,
            "progress": remote.get("progress"), "images_count": remote.get("imagesCount"),
            "engine_version": node.get("engineVersion"), "n_images": record.get("n_images"),
            "updated_at": record.get("updated_at"), "can_reattach": can_reattach,
            "can_cancel_remote": bool(record.get("uuid")) and phase in (_LIVE_PHASES | {"postprocess_failed"}),
            "reason": reason, "failure": (record.get("failure") or {}).get("message")}


class _OdmRecord:
    def __init__(self, path: Path, data: dict, secret: Optional[str]):
        self.path, self.data, self.secret = Path(path), data, secret

    @property
    def uuid(self) -> str:
        return self.data["uuid"]

    @property
    def phase(self) -> str:
        return self.data.get("phase")

    @property
    def odm_dir(self) -> Path:
        return self.path.parent

    def save(self) -> None:
        self.data["updated_at"] = _now()
        _write_json_atomic(self.path, self.data)

    def note(self, event: str, **fields) -> None:
        self.data.setdefault("events", []).append({"event": event, "at": _now(), **fields})
        self.save()

    def set_phase(self, phase: str, **fields) -> None:
        self.data["phase"] = phase
        self.data.setdefault("phase_history", []).append({"phase": phase, "at": _now()})
        self.data.update(fields)
        self.save()


def _archive_previous(odm_dir: Path) -> Optional[Path]:
    """新任務不覆蓋舊紀錄：舊任務收尾了就搬到 odm/history/；遠端還可能在跑或還沒下載 ⇒ 拒絕（以免失聯）。"""
    p = odm_dir / "submission.json"
    if not p.exists():
        return None
    try:
        old = _normalize_record(json.loads(p.read_text(encoding="utf-8")))
    except (OSError, ValueError, PipelineError):
        old = {"phase": "unreadable"}
    phase = old.get("phase")
    if phase in _LIVE_PHASES and old.get("uuid"):
        raise PipelineError(f"odm/submission.json 還有一個沒收尾的遠端任務 {old.get('uuid')}（{ODM_PHASE_TEXT.get(phase)}）："
                            "請先「接回任務」把成果下載完、或「取消遠端任務」，再送新的任務（不覆蓋紀錄，以免遠端任務失聯）")
    hist = odm_dir / "history"
    hist.mkdir(parents=True, exist_ok=True)
    stem = f"submission_{old.get('uuid') or 'none'}_{phase}"
    dest, k = hist / f"{stem}.json", 2
    while dest.exists():
        dest, k = hist / f"{stem}_{k}.json", k + 1
    os.replace(p, dest)
    q = odm_dir / "quality_summary.json"
    if q.exists():
        qd, k = hist / f"quality_summary_{old.get('uuid') or 'none'}.json", 2
        while qd.exists():
            qd, k = hist / f"quality_summary_{old.get('uuid') or 'none'}_{k}.json", k + 1
        os.replace(q, qd)
    return dest


class _OdmRun:
    """一次執行（送出或接回）的共用狀態：紀錄、客戶端、進度回報、取消。"""

    def __init__(self, rec: _OdmRecord, client, *, progress, should_cancel, sleep, on_state):
        self.rec, self.client = rec, client
        self._progress, self._should_cancel, self._sleep, self._on_state = progress, should_cancel, sleep, on_state
        self.warnings: list = []
        self.state = {"uuid": rec.data.get("uuid"), "node_url": rec.data.get("node_url")}
        self._last_status_key = None

    def report(self, msg: str, frac: Optional[float] = None, *, log: bool = True, **state) -> None:
        if frac is not None:                                 # 0.30 + 0.60×1.0 = 0.9000000000000001：分段相接處不能倒退
            frac = round(min(1.0, max(0.0, float(frac))), 6)
        self.state.update(state)
        self.state.update(message=msg, phase=self.rec.phase, frac=frac)
        if log:
            _log.info("[ODM] %s", msg)
        if self._progress is not None:
            self._progress(msg, frac)
        if self._on_state is not None:
            self._on_state(dict(self.state))

    def warn(self, msg: str) -> None:
        if msg not in self.warnings:
            self.warnings.append(msg)
            _log.warning("[ODM] %s", msg)
        ws = self.rec.data.setdefault("warnings", [])
        if msg not in ws:
            ws.append(msg)

    def canceled(self) -> bool:
        return self._should_cancel is not None and bool(self._should_cancel())

    def sleep(self, seconds: float) -> None:
        """分段睡，每 0.5 s 檢查取消（取消 ⇒ PipelineCanceled；遠端任務不受影響）。"""
        t = 0.0
        while True:
            if self.canceled():
                self.stop_waiting("等待")
            if t >= seconds:
                return
            dt = min(0.5, seconds - t)
            self._sleep(dt)
            t += dt

    def stop_waiting(self, what: str):
        self.rec.note("local_stopped", stage=what)
        msg = (f"已停止本機{what}；遠端任務 {self.rec.uuid} 仍在節點上（{ODM_PHASE_TEXT.get(self.rec.phase)}），"
               "可用「接回任務」繼續")
        self.report(msg, None)
        raise PipelineCanceled(msg)


def _client_for(record_or_job: dict, client):
    url = record_or_job["node_url"]
    if client is None:
        try:
            return odm_client.NodeOdmClient(url)
        except ValueError as e:
            raise PipelineError(str(e)) from None
    try:
        want = odm_client.sanitize_node_url(url)
    except ValueError as e:
        raise PipelineError(str(e)) from None
    if getattr(client, "base_url", want) != want:
        raise PipelineError(f"客戶端連的是 {client.base_url}，任務紀錄的節點是 {want}：不同節點不能接續同一個任務")
    return client


def option_sources(job: dict) -> dict:
    """每個送出的參數與每個刻意不送的參數 → 值與來源（有效值契約；節點自己加／改的另記在 options_check）。"""
    src = {}
    for o in job.get("options") or []:
        n = o.get("name")
        if n == "orthophoto-resolution":
            s = job.get("orthophoto_resolution_source") or "caller"
        elif n in ("dsm", "tiles"):
            s = "pipeline（odm_job 固定開啟）"
        elif n in ("rolling-shutter", "rolling-shutter-readout"):
            s = "operator（rolling_shutter）＋會期實測讀出時間"
        elif n == "force-gps":
            s = (job.get("force_gps") or {}).get("source", "caller")
        elif n == "max-concurrency":
            s = (job.get("max_concurrency") or {}).get("source", "caller")
        else:
            s = "caller"
        src[n] = {"sent": True, "value": o.get("value"), "source": s}
    for key, name in (("force_gps", "force-gps"), ("max_concurrency", "max-concurrency"),
                      ("gps_accuracy", "gps-accuracy")):
        d = job.get(key)
        if isinstance(d, dict) and not d.get("sent") and name not in src:
            src[name] = {"sent": False, "value": None, "source": d.get("source"), "reason": d.get("meaning")}
    return src


def _node_summary(info: dict) -> dict:
    keys = ("version", "engine", "engineVersion", "maxImages", "availableMemory", "totalMemory", "cpuCores",
            "maxParallelTasks", "taskQueueCount")
    return {k: info.get(k) for k in keys if k in info}


def _engine_defaults(opts: list, names) -> dict:
    by = {o.get("name"): o for o in opts if isinstance(o, dict)}
    return {n: {"default": (by.get(n) or {}).get("value"), "type": (by.get(n) or {}).get("type"),
                "known": n in by} for n in names}


def _same_value(a, b) -> bool:
    def norm(v):
        if isinstance(v, bool):
            return v
        if isinstance(v, str) and v.strip().lower() in ("true", "false"):
            return v.strip().lower() == "true"
        try:
            return float(v)
        except (TypeError, ValueError):
            return str(v)
    na, nb = norm(a), norm(b)
    if isinstance(na, float) and isinstance(nb, float):
        return math.isclose(na, nb, rel_tol=1e-9, abs_tol=1e-12)
    return na == nb


def diff_node_options(sent: list, effective: Optional[list]) -> dict:
    """提交後 info.options（節點過濾後真正要傳給引擎的）對照送出的 options（R1 §4）。

    ``dropped``：送了但節點丟掉（引擎不認得）⇒ 必須大聲失敗；``changed``：值被節點改了（例如 max-concurrency 上限）；
    ``added``：節點自己加的（pc-ept／cog／gltf、max-concurrency 上限）。--gcp／--geo 不在 info.options 裡（N Task.js:554-565）。
    """
    sent_map = {o["name"]: o.get("value") for o in sent or []}
    eff_map = {o.get("name"): o.get("value") for o in effective or [] if isinstance(o, dict)}
    return {"dropped": [n for n in sent_map if n not in eff_map],
            "changed": [{"name": n, "sent": sent_map[n], "node": eff_map[n]} for n in sent_map
                        if n in eff_map and not _same_value(sent_map[n], eff_map[n])],
            "added": [{"name": n, "value": v} for n, v in eff_map.items() if n not in sent_map]}


_ODX_INIT_RE = re.compile(r"Initializing\s+(ODX|ODM)\s+(\S+)", re.I)
_ODX_ARG_RE = re.compile(r"^\[INFO\]\s+([a-z][a-z0-9_]*):\s?(.*)$")


def parse_odx_args(lines) -> dict:
    """從主控台抓 ODX 啟動時印的有效參數區塊（"Initializing ODX 3.8.4" 之後一串 ``[INFO] <dest>: <value>``，
    X run.py:25,41-45；X modules/log.py:51-55）。回傳 ``{"engine_line", "args", "complete"}``；格式沒對真引擎驗證過。"""
    lines = [str(x).strip() for x in lines or []]
    start, engine = None, None
    for i, ln in enumerate(lines):
        m = _ODX_INIT_RE.search(ln)
        if m:
            start, engine = i + 1, f"{m.group(1)} {m.group(2)}"
            break
    if start is None:
        return {"engine_line": None, "args": {}, "complete": False}
    args, started = {}, False
    for ln in lines[start:]:
        m = _ODX_ARG_RE.match(ln)
        if m:
            args[m.group(1)] = m.group(2).strip()
            started = True
        elif started:
            return {"engine_line": engine, "args": args, "complete": True}
    return {"engine_line": engine, "args": args, "complete": False}


def _basename(v) -> Optional[str]:
    if v is None or str(v).strip() in ("", "None"):
        return None
    return str(v).replace("\\", "/").rstrip("/").rsplit("/", 1)[-1]


def check_engine_args(parsed: dict, record: dict) -> dict:
    """核對 ODX 實際用的 geo／gcp 檔與 force_gps、max_concurrency（只記錄、不當失敗：主控台格式沒對真引擎驗證過）。"""
    args = parsed.get("args") or {}
    aux = {a["uploaded_as"] for a in (record.get("upload") or {}).get("aux_files", [])}
    sent = {o["name"]: o.get("value") for o in record.get("options") or []}
    mism = []
    if "geo" in args and ("geo.txt" in aux) and _basename(args["geo"]) != "geo.txt":
        mism.append(f"引擎用的 geo 是 {args['geo']!r}，不是 geo.txt")
    if "gcp" in args:
        want = odm_client.GCP_UPLOAD_NAME if odm_client.GCP_UPLOAD_NAME in aux else None
        if _basename(args["gcp"]) != want:
            mism.append(f"引擎用的 gcp 是 {args['gcp']!r}，預期 {want!r}")
    if "force_gps" in args and not _same_value(args["force_gps"], bool(sent.get("force-gps"))):
        mism.append(f"引擎的 force_gps = {args['force_gps']}，我們的決定是 {bool(sent.get('force-gps'))}")
    eff_mc = None
    for o in (record.get("remote") or {}).get("options") or []:
        if isinstance(o, dict) and o.get("name") == "max-concurrency":
            eff_mc = o.get("value")
    if "max_concurrency" in args and eff_mc is not None and not _same_value(args["max_concurrency"], eff_mc):
        mism.append(f"引擎的 max_concurrency = {args['max_concurrency']}，節點 info.options 是 {eff_mc}")
    keys = ("gcp", "geo", "force_gps", "max_concurrency", "gps_accuracy", "use_exif", "orthophoto_resolution",
            "rolling_shutter", "rolling_shutter_readout", "dsm", "tiles", "skip_report")
    return {"engine_line": parsed.get("engine_line"), "complete": bool(parsed.get("complete")),
            "observed": {k: args[k] for k in keys if k in args}, "mismatches": mism}


def _console_tail(run: _OdmRun) -> list:
    try:
        lines = run.client.task_output(run.rec.uuid, 0)
    except odm_client.OdmError as e:
        return [f"（無法取得主控台輸出：{e}）"]
    return lines[-ODM_CONSOLE_TAIL_LINES:]


def _cancel_and_remove(run: _OdmRun) -> tuple:
    """POST /task/cancel ＋ /task/remove → (動作紀錄, 遠端已停或已不在)。節點說「not found」也算已不在。"""
    actions, stopped = [], False
    for what, fn in (("cancel", run.client.cancel_task), ("remove", run.client.remove_task)):
        try:
            fn(run.rec.uuid)
            actions.append(f"{what}: ok")
            stopped = True
        except odm_client.OdmError as e:
            actions.append(f"{what}: {e}")
            if e.not_found and not isinstance(e, odm_client.OdmTransportError):
                stopped = True                               # 節點上已經沒有這個任務
    return actions, stopped


def _abort_remote(run: _OdmRun, reason: str, kind: str):
    """提交後核對不過：取消並刪除遠端任務（不讓它用錯的輸入跑幾個小時），紀錄 failed，丟 PipelineError。

    取消與刪除**都**沒成功（連不上節點）⇒ 紀錄 ``abort_pending``（不是 failed）：遠端可能還在跑，要能再取消、
    能接回（接回時會再試一次取消＋刪除），也要擋住新任務（審查 T6）。"""
    actions, stopped = _cancel_and_remove(run)
    failure = {"kind": kind, "message": reason, "remote_actions": actions, "at": _now(), "pending_abort": not stopped}
    if stopped:
        run.rec.set_phase("failed", failure=failure)
        raise PipelineError(f"{reason}（已要求節點取消並刪除任務 {run.rec.uuid}：{'；'.join(actions)}）")
    run.rec.set_phase("abort_pending", failure=failure)
    raise PipelineError(f"{reason}。⚠ 取消與刪除遠端任務 {run.rec.uuid} 都沒成功（{'；'.join(actions)}）：遠端可能"
                        "還在用錯的輸入跑——請用「取消遠端任務」或「接回既有任務…」再試（紀錄停在「取消待完成」）")


def _retry_abort(run: _OdmRun) -> dict:
    """``abort_pending`` 的任務（接回或取消遠端時）：再試一次取消＋刪除。成功 ⇒ failed（原本的失敗原因保留）；
    還是不行 ⇒ 維持 abort_pending、丟 PipelineError。"""
    rec = run.rec
    failure = dict(rec.data.get("failure") or {})
    actions, stopped = _cancel_and_remove(run)
    failure["remote_actions"] = list(failure.get("remote_actions") or []) + [f"重試：{a}" for a in actions]
    failure["pending_abort"] = not stopped
    if not stopped:
        rec.data["failure"] = failure
        rec.note("abort_retry_failed", actions=actions)
        raise PipelineError(f"任務 {rec.uuid} 還是取消不了（{'；'.join(actions)}）：遠端可能仍在跑，稍後再試")
    rec.set_phase("failed", failure=failure)
    msg = (f"已取消並刪除提交後核對不過的任務 {rec.uuid}（{'；'.join(actions)}）；原因：{failure.get('message')}"
           "——請重新送出")
    run.report(msg, None)
    return {"uuid": rec.uuid, "phase": rec.phase, "status_code": None, "message": msg}


def _cleanup_uncommitted(run: _OdmRun) -> str:
    """提交前失敗／取消：試 /task/remove（萬一 commit 其實送到了）；未提交的上傳節點沒有刪除 API，照實記錄。"""
    try:
        run.client.remove_task(run.rec.uuid)
        res = {"endpoint": "/task/remove", "result": "removed",
               "note": "節點上已有這個任務（提交其實送到了），已刪除"}
    except odm_client.OdmError as e:
        res = {"endpoint": "/task/remove", "result": "not_found" if e.not_found else "error", "error": str(e),
               "note": _UNCOMMITTED_NOTE}
    run.rec.data["remote_cleanup"] = res
    return res["note"]


def _observe(run: _OdmRun, info: dict) -> None:
    st = info.get("status") or {}
    code = st.get("code")
    prog = info.get("progress")
    pt = info.get("processingTime")
    remote = run.rec.data.setdefault("remote", {})
    remote.update(status_code=code, progress=prog, processingTime=pt, imagesCount=info.get("imagesCount"),
                  checked_at=_now())
    if "options" in info:
        remote["options"] = info.get("options")
    try:
        bucket = int(float(prog) // 10) if prog is not None else None
    except (TypeError, ValueError):
        bucket = None
    key = (code, bucket)
    if key != run._last_status_key:
        run._last_status_key = key
        run.rec.data.setdefault("status_history", []).append(
            {"at": _now(), "code": code, "progress": prog, "processingTime": pt,
             **({"errorMessage": st.get("errorMessage")} if st.get("errorMessage") else {})})
        if run.rec.phase == "committed" and code in (odm_client.STATUS_QUEUED, odm_client.STATUS_RUNNING):
            run.rec.set_phase("running")
        else:
            run.rec.save()
    try:
        pf = max(0.0, min(100.0, float(prog))) if prog is not None else None
    except (TypeError, ValueError):
        pf = None
    frac = None if pf is None else _F_REMOTE[0] + (_F_REMOTE[1] - _F_REMOTE[0]) * pf / 100.0
    name = odm_client.STATUS_NAMES.get(code, f"狀態 {code}")
    init = " 初始化中" if code == odm_client.STATUS_RUNNING and isinstance(pt, (int, float)) and pt < 0 else ""
    run.report(f"節點{name}{init}（任務 {run.rec.uuid}，{'—' if pf is None else f'{pf:.0f} %'}）", frac, log=False,
               status_code=code, remote_progress=prog, processing_time_ms=pt, images_count=info.get("imagesCount"))


def _verify_options(run: _OdmRun, info: dict) -> None:
    if "options_check" in run.rec.data:
        return
    if "options" not in info:
        run.rec.data["options_check"] = {"available": False}
        run.warn("節點的 info 沒有 options：無法確認引擎實際收到的參數")
        run.rec.save()
        return
    d = diff_node_options(run.rec.data.get("options") or [], info.get("options"))
    d["available"] = True
    d["notes"] = ([f"{c['name']}：送 {c['sent']!r}、節點改成 {c['node']!r}（node-capped）" for c in d["changed"]]
                  + [f"{a['name']} = {a['value']!r}（節點自己加的，node auto-added）" for a in d["added"]])
    run.rec.data["options_check"] = d
    run.rec.save()
    if d["dropped"]:
        _abort_remote(run, f"節點丟掉了送出的參數 {d['dropped']}（它的引擎不認得，N odmInfo.js:290-325）：任務會用引擎預設跑",
                      "options_dropped")
    for c in d["changed"]:
        run.warn(f"參數 {c['name']} 被節點改成 {c['node']!r}（送的是 {c['sent']!r}）")


def _verify_images_count(run: _OdmRun, info: dict) -> None:
    if (run.rec.data.get("images_check") or {}).get("ok") is not None:
        return
    st = info.get("status") or {}
    code, pt, n = st.get("code"), info.get("processingTime"), info.get("imagesCount")
    settled = (isinstance(pt, (int, float)) and pt >= 0) or code in (odm_client.STATUS_QUEUED,
                                                                      odm_client.STATUS_COMPLETED)
    if not settled:
        return
    expected = run.rec.data.get("n_images")
    if n is None:
        run.rec.data["images_check"] = {"ok": None, "expected": expected, "reported": None}
        run.warn("節點的 info 沒有 imagesCount：無法核對上傳張數")
        return
    ok = expected is None or int(n) == int(expected)
    run.rec.data["images_check"] = {"ok": ok, "expected": expected, "reported": n, "at": _now()}
    run.rec.save()
    if not ok:
        amb = (run.rec.data.get("upload") or {}).get("ambiguous_retries")
        _abort_remote(run, f"節點上的影像數 {n} 不等於上傳的 {expected} 張"
                           + (f"（有 {amb} 次「送完沒回應」的重送，節點會把同名檔改名成 *_ 再存一次）" if amb else ""),
                      "images_count_mismatch")


def _capture_engine_args(run: _OdmRun, code) -> None:
    """抓 ODX 啟動參數區塊並核對（最多試 5 次；區塊完整、任務完成、或輸出已超過 400 行就定案）。"""
    ea = run.rec.data.get("engine_args") or {}
    if ea.get("final"):
        return
    attempts = int(ea.get("attempts", 0)) + 1
    try:
        lines = run.client.task_output(run.rec.uuid, 0)
    except odm_client.OdmError as e:
        run.rec.data["engine_args"] = {"final": attempts >= 5, "complete": False, "error": str(e), "attempts": attempts}
        if attempts >= 5:
            run.warn(f"取不到 ODX 主控台輸出（{e}）：無法從主控台確認引擎實際用的 geo／gcp 與參數；改看成果裡的 log.json")
        return
    parsed = parse_odx_args(lines)
    if not (parsed["complete"] or code == odm_client.STATUS_COMPLETED or len(lines) >= 400 or attempts >= 5):
        run.rec.data["engine_args"] = {"final": False, "complete": False, "attempts": attempts}
        return
    chk = check_engine_args(parsed, run.rec.data)
    chk.update(attempts=attempts, final=True)
    if not parsed["engine_line"]:
        chk["note"] = "主控台裡沒看到 ODX 啟動參數區塊（格式未對真引擎驗證，或輸出已被清掉）"
        run.warn("ODX 主控台裡沒看到啟動參數區塊：無法從主控台確認引擎實際用的 geo／gcp 與參數；改看成果裡的 log.json")
    run.rec.data["engine_args"] = chk
    for m in chk["mismatches"]:
        run.warn(f"ODX 主控台：{m}")
    run.rec.save()


def _remote_failed(run: _OdmRun, info: dict):
    st = info.get("status") or {}
    msg = st.get("errorMessage") or "（節點沒有給錯誤訊息）"
    tail = _console_tail(run)
    pt = info.get("processingTime")
    hint = ("（提交後初始化就失敗：常見是張數超過節點 maxImages 或搬檔失敗）"
            if isinstance(pt, (int, float)) and pt < 0 else "")
    run.rec.set_phase("failed", failure={"kind": "remote", "message": f"{msg}{hint}", "status": st,
                                         "progress": info.get("progress"), "processingTime": pt,
                                         "console_tail": tail, "at": _now()})
    raise PipelineError(f"ODX 任務 {run.rec.uuid} 失敗：{msg}{hint}\n主控台最後幾行：\n" + "\n".join(tail[-12:]))


def _follow(run: _OdmRun, *, poll_s: float, timeout_s: float, clock=time.monotonic) -> dict:
    """提交之後的共用路徑（送出與接回同一條）：輪詢 → 核對 → 下載 → 解壓 → 找成果 → 品質報告。"""
    rec, uid = run.rec, run.rec.uuid
    t0 = clock()
    fails = 0
    while True:
        if run.canceled():
            run.stop_waiting("等待")
        try:
            info = run.client.task_info(uid)
            fails = 0
        except odm_client.OdmTransportError as e:
            fails += 1
            if fails > ODM_POLL_TRANSPORT_RETRIES:
                rec.note("poll_failed", error=str(e))
                raise PipelineError(f"連不上節點（連續 {fails} 次）：{e}；遠端任務可能仍在執行，稍後可用「接回任務」") from None
            run.report(f"連不上節點（第 {fails} 次），稍後重試：{e}", None)
            run.sleep(poll_s)
            continue
        except odm_client.OdmError as e:
            if e.not_found:
                prev = rec.phase
                if prev == "downloaded":                    # 成果已下載過：節點過期不代表成果不見，不降級成 lost
                    rec.note("not_on_node_after_download", error=str(e))
                    raise PipelineError(f"節點上已查不到任務 {uid}（{e}），本機也找不到 {uid}_all.zip 或解壓後的"
                                        f" result_{uid}/：成果檔被搬走或刪掉了（紀錄維持「已下載」）") from None
                rec.set_phase("lost", lost={"previous_phase": prev, "error": str(e), "at": _now()})
                raise PipelineError(f"節點上查不到任務 {uid}（{e}）：可能已過期（完成後預設 48 h 自動刪除）、被刪除、"
                                    "或節點當機時遺失；需要重新送出") from None
            raise PipelineError(f"查詢任務 {uid} 狀態失敗：{e}") from None
        _observe(run, info)
        code = (info.get("status") or {}).get("code")
        _verify_options(run, info)
        _verify_images_count(run, info)
        pt = info.get("processingTime")
        if (isinstance(pt, (int, float)) and pt >= 0) or code == odm_client.STATUS_COMPLETED:
            _capture_engine_args(run, code)
        if code == odm_client.STATUS_COMPLETED:
            if rec.phase != "completed":
                rec.set_phase("completed")
            break
        if code == odm_client.STATUS_FAILED:
            _remote_failed(run, info)
        if code == odm_client.STATUS_CANCELED:
            rec.set_phase("canceled", failure={"kind": "remote_canceled", "message": "節點回報任務已取消", "at": _now()})
            raise PipelineError(f"遠端任務 {uid} 已被取消（狀態 50）")
        if clock() - t0 > timeout_s:
            rec.note("wait_timeout", timeout_s=timeout_s)
            raise PipelineError(f"等待任務 {uid} 逾時（{timeout_s:.0f} s）；遠端任務仍在執行，可用「接回任務」繼續")
        run.sleep(poll_s)
    return _download_and_finish(run, info)


def _download_and_finish(run: _OdmRun, info: dict) -> dict:
    rec, uid = run.rec, run.rec.uuid
    zip_path = rec.odm_dir / f"{uid}_all.zip"
    run.report("下載成果 all.zip…", _F_DOWNLOAD[0])
    span = _F_DOWNLOAD[1] - _F_DOWNLOAD[0]
    last = [0.0]

    def dl_prog(got, total):
        now = time.monotonic()
        if now - last[0] < 0.5:
            return
        last[0] = now
        frac = None if not total else _F_DOWNLOAD[0] + span * min(1.0, got / total)
        run.report(f"下載 all.zip：{got / 2**20:.0f}" + (f"／{total / 2**20:.0f}" if total else "") + " MiB", frac,
                   log=False, download_bytes=got, download_total=total)

    try:
        run.client.download_all(uid, zip_path, progress=dl_prog, should_cancel=run.canceled)
    except odm_client.OdmCanceled:
        run.stop_waiting("下載")
    except odm_client.OdmError as e:
        rec.data["download_error"] = {"message": str(e), "at": _now(),
                                      "kind": "local" if isinstance(e, odm_client.OdmLocalError) else "remote"}
        rec.save()
        raise PipelineError(f"下載任務 {uid} 的成果失敗：{e}（節點預設在完成後 48 h 才刪成果，可用「接回任務」重下）") from None
    rec.data.pop("download_error", None)
    rec.data["zip"] = str(zip_path)
    rec.data["zip_bytes"] = zip_path.stat().st_size
    rec.data["downloaded_at"] = _now()
    rec.save()
    return _finish_local(run, info, extract=True)


def _local_zip(rec: _OdmRecord) -> Optional[Path]:
    """本機已下載、完整的 all.zip（以紀錄檔所在資料夾為準，不用紀錄裡的絕對路徑——資料夾搬過也找得到）。

    download_all 只在位元組數與 Content-Length 相符後才把 .part 改名 ⇒ 檔案在就是完整的；紀錄有 zip_bytes 時再對一次大小。"""
    p = rec.odm_dir / f"{rec.uuid}_all.zip"
    if not p.is_file():
        return None
    want = rec.data.get("zip_bytes")
    if want is not None and p.stat().st_size != int(want):
        return None
    return p


def _resolve_geo_txt(rec: _OdmRecord) -> Optional[Path]:
    """送出時的 geo.txt：紀錄的路徑；資料夾搬過就用紀錄檔旁邊（odm/geo.txt）的同名檔。"""
    g = rec.data.get("geo_txt")
    if not g:
        return None
    p = Path(g)
    if p.is_file():
        return p
    alt = rec.odm_dir / p.name
    return alt if alt.is_file() else None


def _finish_local(run: _OdmRun, info: Optional[dict], *, extract: bool) -> dict:
    rec, uid = run.rec, run.rec.uuid
    zip_path = rec.odm_dir / f"{uid}_all.zip"               # 以紀錄檔位置為準（審查 T2b：資料夾搬過）
    extract_dir = rec.odm_dir / f"result_{uid}"
    run.report("解壓成果與讀品質報告…", _F_DOWNLOAD[1])
    if extract or not extract_dir.is_dir():
        existed = extract_dir.is_dir()
        try:
            _check_extract_space(zip_path, extract_dir)
            extract_dir = extract_odm_zip(zip_path, extract_dir)
        except (PipelineError, zipfile.BadZipFile, OSError) as e:
            if not existed and extract_dir.is_dir():      # 解到一半（例如磁碟滿）：不留半套成果
                shutil.rmtree(extract_dir, ignore_errors=True)
            rec.set_phase("postprocess_failed", postprocess_error={"message": str(e), "at": _now()})
            if isinstance(e, PipelineError):
                raise
            raise PipelineError(f"解壓 {zip_path.name} 失敗：{e}") from None
    else:
        extract_dir = extract_dir.resolve()
        if not zip_path.is_file():
            run.warn(f"本機已沒有 {zip_path.name}（只剩解壓後的 {extract_dir.name}/）：用解壓後的成果")
    tiles, dsm = find_ortho_tiles(extract_dir), find_odm_dsm(extract_dir)
    gcp_up = any(a.get("role") == "gcp" for a in (rec.data.get("upload") or {}).get("aux_files", []) or [])
    quality = read_odm_quality(extract_dir, n_uploaded=rec.data.get("n_images"), sent_options=rec.data.get("options"),
                               force_gps=rec.data.get("force_gps"),
                               node_engine_version=(rec.data.get("node") or {}).get("engineVersion"), uuid=uid,
                               gcp_uploaded=gcp_up, gps_accuracy=rec.data.get("gps_accuracy"),
                               geo_txt=_resolve_geo_txt(rec) if rec.data.get("geo_txt") else None,
                               geo_txt_expected=bool(rec.data.get("geo_txt")))
    sent = {o["name"] for o in rec.data.get("options") or []}
    if "tiles" in sent and tiles is None:
        quality["warnings"].append("送了 --tiles，成果裡卻沒有 orthophoto_tiles/")
    if "dsm" in sent and dsm is None:
        quality["warnings"].append("送了 --dsm，成果裡卻沒有 odm_dem/dsm.tif")
    q_path = _write_json_atomic(rec.odm_dir / "quality_summary.json", quality)
    for w in quality["warnings"]:
        run.warn(f"品質報告：{w}")
    # ⚠ NodeODX 在 ODX 根本沒啟動時（例如找不到 winrun.bat）也回報 status 40、progress 100、all.zip 只有
    #   task_output.txt（2026-10-20 本機實測：65 ms「完成」）⇒ 「完成」不是證據，沒有正射就是失敗
    if tiles is None and "skip-orthophoto" not in sent and not any(extract_dir.rglob("odm_orthophoto.tif")):
        out_txt = extract_dir / "task_output.txt"
        try:
            tail = out_txt.read_text(encoding="utf-8", errors="replace").splitlines()[-ODM_CONSOLE_TAIL_LINES:]
        except OSError:
            tail = []
        msg = ("節點回報完成，成果裡卻沒有正射（odm_orthophoto.tif 與 orthophoto_tiles/ 都沒有）"
               + ("、也沒有品質報告" if not quality.get("stats_available") else "")
               + "：引擎可能根本沒有執行，請看 task_output.txt")
        rec.set_phase("failed", failure={"kind": "empty_result", "message": msg, "console_tail": tail,
                                         "quality_summary": str(q_path), "at": _now()})
        raise PipelineError(f"ODX 任務 {uid}：{msg}\n主控台最後幾行：\n" + "\n".join(tail[-12:]))
    rec.data.pop("postprocess_error", None)
    rec.set_phase("downloaded", zip=str(zip_path) if zip_path.is_file() else None, extract_dir=str(extract_dir),
                  ortho_tiles=_s(tiles), dsm=_s(dsm), quality_summary=str(q_path),
                  downloaded_at=rec.data.get("downloaded_at") or _now())
    run.report("完成" + ("" if tiles else "（成果裡沒有 orthophoto_tiles：任務沒開 --tiles？）")
               + (f"；⚠ {len(run.warnings)} 則警告" if run.warnings else ""), 1.0)
    return {"uuid": uid, "info": info, "zip": zip_path, "extract_dir": extract_dir, "ortho_tiles": tiles, "dsm": dsm,
            "quality": quality, "quality_rows": odm_quality_rows(quality), "warnings": list(run.warnings),
            "record_path": rec.path, "node": rec.data.get("node"), "phase": rec.phase}


def check_job_consistency(job: dict) -> None:
    """送出前核對「要上傳的」與「紀錄說的」一致（有效值契約）：任務一定要由 :func:`odm_job` 組、之後沒被改過。

    * 有 ``gcp_txt`` ⇒ 要有核對摘要 ``gcp``、``force_gps.decision`` 是 force_gps／gcp_only；
    * 沒有 ``gcp_txt`` ⇒ ``force_gps.decision`` 必須是 not_applicable_no_gcp；
    * options 裡有沒有 force-gps 必須等於決定是不是 force_gps。
    不一致 ⇒ PipelineError（否則紀錄會說沒用 GCP、卻上傳了 GCP 檔，GCP 的高程基準與檢核點核對也被跳過；審查）。"""
    fg = job.get("force_gps")
    decision = fg.get("decision") if isinstance(fg, dict) else None
    has_gcp = bool(str(job.get("gcp_txt") or "").strip())
    sent = any(isinstance(o, dict) and o.get("name") == "force-gps" and o.get("value") for o in job.get("options") or [])
    problems = []
    if decision is None:
        problems.append("任務沒有 force_gps 的決定（請用 survey.pipeline.odm_job 組任務）")
    elif has_gcp:
        if not isinstance(job.get("gcp"), dict):
            problems.append("有 gcp_txt 卻沒有 GCP 核對摘要（gcp）：GCP 檔沒有經過 odm_job 的核對")
        if decision not in ("force_gps", "gcp_only"):
            problems.append(f"有 gcp_txt，但 force_gps 的決定是 {decision!r}（要 force_gps 或 gcp_only）")
    elif decision != "not_applicable_no_gcp":
        problems.append(f"沒有 gcp_txt，但 force_gps 的決定是 {decision!r}")
    if decision is not None and sent != (decision == "force_gps"):
        problems.append(f"options {'有' if sent else '沒有'} force-gps，但紀錄的決定是 {decision!r}")
    if problems:
        raise PipelineError("任務內容與紀錄不一致，不送：" + "；".join(problems))


def run_odm_job(job: dict, *, client=None, progress: Optional[Callable[[str, Optional[float]], None]] = None,
                should_cancel: Optional[Callable[[], bool]] = None, poll_s: float = 10.0,
                timeout_s: float = 24 * 3600.0, sleep=time.sleep, on_state: Optional[Callable[[dict], None]] = None,
                batch_max_files: int = odm_client.UPLOAD_BATCH_MAX_FILES,
                batch_max_bytes: int = odm_client.UPLOAD_BATCH_MAX_BYTES) -> dict:
    """查節點 → init → 分批上傳 → commit → 輪詢 → 核對 → 下載 all.zip → 解壓 → 找正射／DSM → 讀品質報告。

    任務紀錄寫在 ``out_dir/odm/submission.json``，每個階段都更新（見本節開頭的 phase 表）；權杖只在 ``client`` 裡。
    取消（``should_cancel``）：提交前 ⇒ 停在兩批之間、試著清掉遠端（見 :data:`_UNCOMMITTED_NOTE`）、丟 PipelineCanceled；
    提交後 ⇒ 只停本機的等待／下載（遠端任務繼續跑，可用 :func:`resume_odm_job` 接回、:func:`cancel_odm_job` 取消）。
    ``progress(訊息, 0–1 或 None)``：0–0.30 上傳（位元組）、0.30–0.90 節點處理（info.progress）、0.90–0.98 下載。
    ``on_state(dict)``：同一時刻的結構化狀態（phase、uuid、上傳檔數／位元組、遠端狀態與進度…），給對話框顯示用。
    """
    odm_dir = Path(job["out_dir"]) / "odm"
    check_job_consistency(job)
    client = _client_for(job, client)
    try:
        imgs, aux = odm_client.classify_uploads(job["images"], geo_txt=job["geo_txt"], gcp_txt=job.get("gcp_txt"))
    except ValueError as e:
        raise PipelineError(f"上傳清單：{e}") from None
    sizes = {}
    for p, _ in imgs + aux:
        try:
            sizes[str(p)] = Path(p).stat().st_size
        except OSError as e:
            raise PipelineError(f"讀不到要上傳的檔案 {p}：{e}") from None
    img_items = [(p, n, sizes[str(p)]) for p, n in imgs]
    aux_items = [(p, n, sizes[str(p)]) for p, n in aux]
    n_batches_img = len(odm_client.plan_batches(img_items, max_files=batch_max_files, max_bytes=batch_max_bytes))
    n_batches_aux = (len(odm_client.plan_batches(aux_items, max_files=batch_max_files, max_bytes=batch_max_bytes))
                     if aux else 0)
    _archive_previous(odm_dir)
    data = {k: v for k, v in job.items() if k != "images"}
    data.update(format=SUBMISSION_FORMAT, uuid=str(uuid_mod.uuid4()), node_url=client.base_url,
                token_used=bool(getattr(client, "token", None)), n_images=len(imgs),
                image_names=sorted(n for _, n in imgs), created_at=_now(),
                upload={"batch_max_files": int(batch_max_files), "batch_max_bytes": int(batch_max_bytes),
                        "n_files": len(imgs) + len(aux), "n_images": len(imgs),
                        "bytes_total": sum(sizes.values()), "bytes_images": sum(i[2] for i in img_items),
                        "aux_files": [{"local": str(p), "uploaded_as": n, "bytes": sizes[str(p)],
                                       "role": "geo" if n == "geo.txt" else "gcp"} for p, n in aux],
                        "n_batches_planned": n_batches_img + n_batches_aux, "batches": [],
                        "ambiguous_retries": 0, "streamed_from_disk": True})
    data["option_sources"] = option_sources(job)
    data["phase"] = "planned"
    data["phase_history"] = [{"phase": "planned", "at": _now()}]
    rec = _OdmRecord(odm_dir / "submission.json", data, getattr(client, "token", None))
    rec.save()
    run = _OdmRun(rec, client, progress=progress, should_cancel=should_cancel, sleep=sleep, on_state=on_state)
    _submit(run, imgs, aux, n_batches_aux, batch_max_files, batch_max_bytes)
    return _follow(run, poll_s=poll_s, timeout_s=timeout_s)


def _submit(run: _OdmRun, imgs, aux, n_batches_aux, batch_max_files, batch_max_bytes) -> None:
    rec, client, uid = run.rec, run.client, run.rec.uuid
    up = rec.data["upload"]
    n_img = len(imgs)
    # 1) 節點與引擎（GET /info、GET /options）——什麼都還沒上傳
    try:
        node = client.node_info()
    except odm_client.OdmError as e:
        rec.set_phase("upload_failed", failure={"kind": "node_info", "message": str(e), "at": _now()})
        raise PipelineError(f"連不上 NodeODX（GET /info）：{e}") from None
    rec.data["node"] = dict(_node_summary(node), fetched_at=_now())
    ev = node.get("engineVersion")
    if ev in (None, "", "?"):
        run.warn(f"節點回報的引擎版本未知（engineVersion={ev!r}）：無法確認是哪一版 ODX 處理的，下載後會再看 log.json")
    rec.data["node"]["engine_note"] = "info.engine 在 ODX 3.8.4 永遠是退回值 'odx'，不是證據（N odmRunner.js:95-100）"
    max_images = node.get("maxImages")
    if isinstance(max_images, (int, float)) and not isinstance(max_images, bool):
        if n_img > max_images:
            msg = (f"這個節點最多只處理 {int(max_images)} 張（GET /info 的 maxImages），本任務有 {n_img} 張："
                   "請分任務或換節點（沒有上傳任何檔案）")
            rec.set_phase("upload_failed", failure={"kind": "max_images", "message": msg, "at": _now()})
            raise PipelineError(msg)
        if n_img + len(aux) > max_images:
            # R1 §2.3：提交後的初始化是「搬進 images/ → 解 seed → 檢查 maxImages → 才把 *.txt 搬到 gcp/」
            # （N taskNew.js:201-210,321-340）⇒ 檢查時 geo.txt／gcp_list.txt 還在 images/ 裡、會被算進去
            msg = (f"這個節點最多只處理 {int(max_images)} 張（GET /info 的 maxImages），本任務 {n_img} 張影像加 {len(aux)} 個"
                   f" .txt 附檔共 {n_img + len(aux)} 個檔：節點檢查張數時 .txt 還在 images/ 裡（R1 §2.3），上傳完才會在"
                   "初始化失敗——請減少影像、分任務或換節點（沒有上傳任何檔案）")
            rec.set_phase("upload_failed", failure={"kind": "max_images", "message": msg, "at": _now()})
            raise PipelineError(msg)
    geo_chk = rec.data.get("geo_txt_check") or {}
    sem = geo_chk.get("accuracy_semantics") or {}
    if geo_chk.get("n_with_accuracy") and sem.get("recorded") is False:
        run.warn(str(sem.get("note")))
    if geo_chk.get("accuracy") in ("none", "partial"):
        n_acc, n_all = geo_chk.get("n_with_accuracy"), geo_chk.get("n_images")
        run.warn(f"geo.txt 只有 {n_acc}/{n_all} 張帶每張的精度欄：其餘的照片 GPS 先驗用引擎預設 --gps-accuracy 3 m"
                 "（垂直 9 m，X modules/photo.py:836-857）")
    names = [o["name"] for o in rec.data.get("options") or []]
    try:
        eopts = client.engine_options()
    except odm_client.OdmError as e:
        rec.data["engine_options"] = {"available": False, "error": str(e)}
        run.warn(f"取不到節點的參數清單（GET /options：{e}）：提交後仍會用 info.options 核對")
    else:
        known = {o.get("name") for o in eopts if isinstance(o, dict)}
        unknown = [n for n in names if n not in known]
        rec.data["engine_options"] = {"available": True, "n_known": len(known),
                                      "defaults": _engine_defaults(eopts, sorted(set(names) | {
                                          "force-gps", "max-concurrency", "gps-accuracy", "orthophoto-resolution"}))}
        if unknown:
            msg = (f"節點的引擎不認得這些參數：{unknown}（NodeODX 會靜默丟掉，N odmInfo.js:290-325）："
                   f"引擎版本 {ev!r}；沒有上傳任何檔案")
            rec.set_phase("upload_failed", failure={"kind": "options_unknown", "message": msg, "at": _now()})
            raise PipelineError(msg)
    rec.save()
    if run.canceled():
        rec.set_phase("upload_canceled", failure={"kind": "canceled", "message": "上傳前取消", "at": _now()})
        raise PipelineCanceled("已在上傳前取消（節點上沒有留下任何東西）")

    # 2) init → 附檔（自己一批、不重送模稜兩可的失敗）→ 影像分批 → commit
    total = up["bytes_total"]
    n_files = up["n_files"]
    n_batches = up["n_batches_planned"]
    last = [0.0]

    def up_prog(ev_):
        now = time.monotonic()
        if ev_.get("file") is None and now - last[0] < 0.5 and ev_["bytes_done"] < ev_["bytes_total"]:
            return
        last[0] = now
        frac = _F_UPLOAD[0] + (_F_UPLOAD[1] - _F_UPLOAD[0]) * (ev_["bytes_done"] / max(1, ev_["bytes_total"]))
        run.report(f"上傳 {ev_['files_done']}/{ev_['n_files']} 個檔（{ev_['bytes_done'] / 2**20:.0f}／"
                   f"{ev_['bytes_total'] / 2**20:.0f} MiB，第 {ev_['batch']}/{ev_['n_batches']} 批）", frac, log=False,
                   upload=dict(ev_))

    def on_batch(rep):
        up["batches"].append(rep)
        up["ambiguous_retries"] = sum(b.get("ambiguous_retries", 0) for b in up["batches"])
        up["attempts_total"] = sum(b.get("attempts", 0) for b in up["batches"])
        rec.save()
        _log.info("[ODM] 第 %d/%d 批完成（%d 個檔、%.1f MiB、試 %d 次）", rep["index"], n_batches, rep["n_files"],
                  rep["bytes"] / 2**20, rep["attempts"])

    try:
        run.report(f"建立任務 {uid}（NodeODX {rec.data['node'].get('version')}，引擎 {ev}）…", 0.0)
        client.init_task(uid, options=rec.data.get("options") or [], name=rec.data.get("name"), sleep=run._sleep)
        rec.set_phase("initialized")
        up["started_at"] = _now()
        t0 = time.monotonic()
        rec.set_phase("uploading")
        common = dict(max_files=batch_max_files, max_bytes=batch_max_bytes, should_cancel=run.canceled,
                      sleep=run._sleep, on_batch=on_batch, n_batches_total=n_batches, n_files_total=n_files,
                      bytes_total=total, progress=up_prog)
        if aux:
            client.upload_files(uid, aux, allow_ambiguous_retry=False, **common)
        client.upload_files(uid, imgs, batch_offset=n_batches_aux, files_offset=len(aux),
                            bytes_offset=sum(a["bytes"] for a in up["aux_files"]), **common)
        up["finished_at"] = _now()
        up["seconds"] = round(time.monotonic() - t0, 3)
        rec.set_phase("uploaded")
        if run.canceled():
            raise odm_client.OdmCanceled("上傳完成、提交前取消")
        rec.set_phase("committing")
        client.commit_task(uid)
        rec.set_phase("committed", committed_at=_now())
        run.report(f"已提交任務 {uid}（{n_img} 張、{total / 2**20:.0f} MiB）", _F_UPLOAD[1])
    except (odm_client.OdmCanceled, PipelineCanceled) as e:
        note = _cleanup_uncommitted(run)
        rec.set_phase("upload_canceled", failure={"kind": "canceled", "message": str(e), "at": _now()})
        raise PipelineCanceled(f"已取消上傳（任務 {uid} 未提交）：{note}") from None
    except odm_client.OdmError as e:
        if getattr(e, "commit_unknown", False):
            # commit 送了、回應沒收到、事後也查不到結果（節點暫時連不上）：任務可能正在節點上跑 ⇒ 紀錄**留在**
            # 「提交中」（算進行中：可接回確認、可取消遠端、擋住新任務），不清理、不說「未提交」（審查 T4）
            rec.data["commit_unknown"] = {"message": str(e), "at": _now()}
            rec.note("commit_unknown", error=str(e))
            raise PipelineError(f"任務 {uid} 的 commit 結果不明（{e}）：節點可能已經在處理——紀錄留在「提交中」，"
                                "請稍後用「接回既有任務…」確認（有就繼續等待／下載，沒有就照實說），"
                                "或用「取消遠端任務」；在那之前不能在同一個輸出資料夾送新任務") from None
        note = _cleanup_uncommitted(run)
        rec.set_phase("upload_failed", failure={"kind": "upload", "message": str(e), "at": _now(),
                                                "ambiguous": bool(getattr(e, "ambiguous", False)),
                                                "stage": "commit" if rec.phase == "committing" else "upload"})
        raise PipelineError(f"上傳到 NodeODX 失敗（任務 {uid} 未提交）：{e}。{note}") from None
    except Exception as e:                                  # noqa: BLE001 - 意外錯誤也要留下紀錄再往上丟
        _cleanup_uncommitted(run)
        rec.set_phase("upload_failed", failure={"kind": "unexpected", "at": _now(),
                                                "message": f"{type(e).__name__}: {odm_client.redact(e, rec.secret)}"})
        raise


def _load_for_resume(job) -> tuple:
    where = job if isinstance(job, (str, Path)) else (job.get("record_path") or job.get("out_dir"))
    if where is None:
        raise PipelineError("接回任務要給輸出資料夾（out_dir）或任務紀錄（record_path）")
    path = submission_path(where)
    return path, load_submission(path)


def resume_odm_job(job, *, client=None, progress: Optional[Callable[[str, Optional[float]], None]] = None,
                   should_cancel: Optional[Callable[[], bool]] = None, poll_s: float = 10.0,
                   timeout_s: float = 24 * 3600.0, sleep=time.sleep,
                   on_state: Optional[Callable[[dict], None]] = None) -> dict:
    """接回已送出的任務：讀 ``odm/submission.json`` 的 uuid 與節點位址，接著輪詢／下載／解壓／讀品質報告。

    ``job``：``{"out_dir": 處理輸出資料夾}``、``{"record_path": submission.json}`` 或直接給路徑；資料夾以紀錄檔的
    位置為準（不用紀錄裡的絕對路徑，資料夾搬過也能接）。``client`` 要連同一個節點（權杖只在這裡給）。
    簽章與 :func:`run_odm_job` 相同，可直接交給 ``OdmSubmitWorker(job, run_fn=resume_odm_job, client=...)``。
    """
    path, data = _load_for_resume(job)
    if not data.get("uuid"):
        raise PipelineError(f"任務紀錄 {path} 沒有 uuid：任務沒有建立過，請重新送出")
    client = _client_for(data, client)
    rec = _OdmRecord(path, data, getattr(client, "token", None))
    run = _OdmRun(rec, client, progress=progress, should_cancel=should_cancel, sleep=sleep, on_state=on_state)
    phase = rec.phase
    uid = rec.uuid
    rec.note("reattach", phase=phase, token_used=bool(getattr(client, "token", None)))
    if phase == "abort_pending":                            # 核對不過、上次取消失敗：再試取消＋刪除，不下載
        raise PipelineError(_retry_abort(run)["message"])
    # 成果已在本機：不問節點（節點完成 48 h 後會刪任務，「查不到」不代表成果不見；審查 T1–T3）
    local = _local_zip(rec)
    extract_dir = rec.odm_dir / f"result_{uid}"
    if local is not None and phase in ("completed", "postprocess_failed", "downloaded", "lost"):
        again = phase == "downloaded" and extract_dir.is_dir()
        run.report(f"任務 {uid} 的 all.zip 已在本機（{local.name}）：" + ("重新整理成果與品質報告" if again
                   else "直接解壓與讀品質報告，不重下") + "…", _F_DOWNLOAD[1])
        return _finish_local(run, None, extract=not again)
    if phase == "downloaded" and extract_dir.is_dir():
        run.report(f"任務 {uid} 已下載過（all.zip 不在了，用解壓後的 {extract_dir.name}/）：重新整理品質報告…",
                   _F_DOWNLOAD[1])
        return _finish_local(run, None, extract=False)
    if phase not in _REATTACH_PHASES and phase != "downloaded":
        try:
            info = client.task_info(uid)
        except odm_client.OdmError as e:
            failure = (rec.data.get("failure") or {}).get("message")
            if phase in _UNCOMMITTED_PHASES:
                probe = client.probe_upload_session(uid) if not isinstance(e, odm_client.OdmTransportError) else None
                if probe is True and phase in _COMMITTABLE_ON_REATTACH and _upload_complete(rec):
                    _commit_on_reattach(run, phase)
                    run.report(f"接回任務 {uid}（{ODM_PHASE_TEXT.get(rec.phase, rec.phase)}）…", None)
                    return _follow(run, poll_s=poll_s, timeout_s=timeout_s)
                where_ = ("節點上還有這個未提交的上傳（但紀錄顯示上傳沒有完整做完，不能補提交）" if probe else "")
                raise PipelineError(f"任務 {uid} 沒有提交（紀錄停在「{ODM_PHASE_TEXT.get(phase, phase)}」）：上傳無法接續，"
                                    f"請重新送出。{where_ + '；' if where_ else ''}{_UNCOMMITTED_NOTE}") from None
            raise PipelineError(f"任務 {uid}（{ODM_PHASE_TEXT.get(phase, phase)}）在節點上查不到：{e}"
                                + (f"；紀錄的原因：{failure}" if failure else "")) from None
        rec.note("found_on_node", previous_phase=phase, status=(info.get("status") or {}).get("code"))
        if phase in _UNCOMMITTED_PHASES:
            rec.set_phase("committed", committed_at=rec.data.get("committed_at") or _now(),
                          commit_confirmed_on_reattach=True)
    elif phase == "committing":
        try:
            client.task_info(uid)
        except odm_client.OdmError as e:
            if not e.not_found or isinstance(e, odm_client.OdmTransportError):
                raise PipelineError(f"查詢任務 {uid} 失敗：{e}（紀錄維持「提交中」，稍後再接回）") from None
            probe = client.probe_upload_session(uid)
            if probe is True and _upload_complete(rec):
                _commit_on_reattach(run, phase)
            elif probe is None:
                raise PipelineError(f"任務 {uid} 在節點上查不到，探測未提交的上傳也沒有結果：commit 有沒有送達還不確定"
                                    "（紀錄維持「提交中」，稍後再接回）") from None
            else:
                rec.set_phase("upload_failed", failure={"kind": "commit_not_received", "at": _now(),
                                                        "message": "commit 沒有送達節點"})
                raise PipelineError(f"任務 {uid} 的 commit 沒有送達節點：請重新送出。{_UNCOMMITTED_NOTE}") from None
        else:
            rec.set_phase("committed", committed_at=_now(), commit_confirmed_on_reattach=True)
    run.report(f"接回任務 {uid}（{ODM_PHASE_TEXT.get(rec.phase, rec.phase)}）…", None)
    return _follow(run, poll_s=poll_s, timeout_s=timeout_s)


# 接回時可以補送 commit 的階段：上傳中／已上傳／上傳失敗（commit 那一步失敗）——紀錄要顯示每一批都上傳成功，
# 節點也要還有這個未提交的上傳（探測回 "Need at least 1 file."）。操作員取消的（upload_canceled）不補。
_COMMITTABLE_ON_REATTACH = frozenset({"uploading", "uploaded", "upload_failed", "committing"})


def _upload_complete(rec: _OdmRecord) -> bool:
    """紀錄顯示每一批都上傳成功（附檔批沒有模稜兩可的失敗——那種會讓批次少一筆）。"""
    up = rec.data.get("upload") or {}
    n = up.get("n_batches_planned")
    return bool(n) and len(up.get("batches") or []) == int(n)


def _commit_on_reattach(run: _OdmRun, previous_phase: str) -> None:
    """整個上傳都完成、節點上也還有這個未提交的上傳 ⇒ 接回時補送 commit（不必重傳 50–100 GB；審查 T5）。"""
    rec, uid = run.rec, run.rec.uuid
    rec.note("commit_on_reattach", previous_phase=previous_phase)
    run.report(f"任務 {uid} 已完整上傳但沒有提交，節點上還有這個上傳：補送 commit…", _F_UPLOAD[1])
    rec.set_phase("committing")
    try:
        run.client.commit_task(uid)
    except odm_client.OdmError as e:
        if getattr(e, "commit_unknown", False):
            rec.note("commit_unknown", error=str(e))
            raise PipelineError(f"補送任務 {uid} 的 commit 結果不明（{e}）：紀錄留在「提交中」，稍後再接回") from None
        rec.set_phase("upload_failed", failure={"kind": "upload", "stage": "commit", "message": str(e), "at": _now()})
        raise PipelineError(f"補送任務 {uid} 的 commit 失敗：{e}。{_UNCOMMITTED_NOTE}") from None
    rec.set_phase("committed", committed_at=_now(), committed_on_reattach=True)


def cancel_odm_job(job, *, client=None, progress: Optional[Callable[[str, Optional[float]], None]] = None,
                   should_cancel: Optional[Callable[[], bool]] = None, poll_s: float = 2.0,
                   timeout_s: float = 120.0, sleep=time.sleep, remove: bool = False,
                   on_state: Optional[Callable[[dict], None]] = None) -> dict:
    """取消遠端任務（POST /task/cancel，``remove=True`` 再 POST /task/remove 刪掉節點上的資料），並等節點回報 50。

    呼叫前先停掉本機正在跑的 run／resume（兩邊都會寫同一份紀錄）。未提交的任務：試 /task/remove，照實記錄
    「沒有刪除 API、節點 48 h 後自清」。回傳 ``{"uuid", "phase", "status_code", "message"}``。
    """
    path, data = _load_for_resume(job)
    if not data.get("uuid"):
        raise PipelineError(f"任務紀錄 {path} 沒有 uuid：沒有遠端任務可取消")
    client = _client_for(data, client)
    rec = _OdmRecord(path, data, getattr(client, "token", None))
    run = _OdmRun(rec, client, progress=progress, should_cancel=should_cancel, sleep=sleep, on_state=on_state)
    uid, phase = rec.uuid, rec.phase
    if phase == "abort_pending":                            # 提交後核對不過、上次取消沒成功：取消＋刪除，原因保留
        return _retry_abort(run)
    if phase in _UNCOMMITTED_PHASES:
        note = _cleanup_uncommitted(run)
        rec.note("cancel_requested", phase=phase, result=rec.data["remote_cleanup"]["result"])
        if rec.data["remote_cleanup"]["result"] == "removed":
            rec.set_phase("canceled", failure={"kind": "operator_canceled", "message": note, "at": _now()})
        elif phase not in ("upload_failed", "upload_canceled"):
            rec.set_phase("upload_canceled", failure={"kind": "operator_canceled", "message": note, "at": _now()})
        msg = f"任務 {uid} 沒有提交：{note}"
        run.report(msg, None)
        return {"uuid": uid, "phase": rec.phase, "status_code": None, "message": msg}
    try:
        client.cancel_task(uid)
    except odm_client.OdmError as e:
        if e.not_found and not isinstance(e, odm_client.OdmTransportError) and phase == "committing":
            # commit 結果不明的任務：節點上沒有 ⇒ commit 沒送達（不是「遺失」）；未提交的上傳照實記
            note = _cleanup_uncommitted(run)
            rec.set_phase("upload_canceled", failure={"kind": "operator_canceled", "at": _now(),
                                                      "message": f"commit 沒有送達節點（{e}）；{note}"})
            msg = f"任務 {uid} 的 commit 沒有送達節點，沒有遠端任務可取消：{note}"
            run.report(msg, None)
            return {"uuid": uid, "phase": rec.phase, "status_code": None, "message": msg}
        if e.not_found:
            if phase in _LIVE_PHASES or phase == "postprocess_failed":
                rec.set_phase("lost", lost={"previous_phase": phase, "error": str(e), "at": _now()})
            else:
                rec.note("cancel_not_found", phase=phase, error=str(e))
            msg = f"節點上已經沒有任務 {uid}（{e}）"
            run.report(msg, None)
            return {"uuid": uid, "phase": rec.phase, "status_code": None, "message": msg}
        raise PipelineError(f"取消任務 {uid} 失敗：{e}") from None
    rec.note("cancel_requested", phase=phase)
    code = None
    for _ in range(max(1, int(timeout_s / max(poll_s, 0.5)) + 1)):    # 次數有上限（poll_s=0 也不會無限迴圈）
        try:
            info = client.task_info(uid)
            code = (info.get("status") or {}).get("code")
            rec.data.setdefault("remote", {}).update(status_code=code, progress=info.get("progress"), checked_at=_now())
        except odm_client.OdmError:
            code = None
        if code in (odm_client.STATUS_CANCELED, odm_client.STATUS_COMPLETED, odm_client.STATUS_FAILED):
            break
        run.report(f"等節點停止任務 {uid}（{odm_client.STATUS_NAMES.get(code, code)}）…", None)
        sleep(poll_s)
    removed = None
    if remove:
        try:
            client.remove_task(uid)
            removed = True
        except odm_client.OdmError as e:
            removed = f"remove 失敗：{e}"
    if code == odm_client.STATUS_CANCELED or removed is True:
        rec.set_phase("canceled", failure={"kind": "operator_canceled", "message": "操作員取消遠端任務", "at": _now(),
                                           "removed": removed})
        msg = f"已取消遠端任務 {uid}" + ("並刪除節點上的資料" if removed is True else "")
    elif code in (odm_client.STATUS_COMPLETED, odm_client.STATUS_FAILED):
        rec.note("cancel_ineffective", status=code)
        msg = f"任務 {uid} 已經是「{odm_client.STATUS_NAMES.get(code)}」，取消沒有作用" + (
            f"；{removed}" if isinstance(removed, str) else "")
    else:
        rec.note("cancel_pending", status=code)
        msg = f"已要求取消任務 {uid}，但 {timeout_s:.0f} s 內節點還沒回報已取消（終止不是立即的）"
    run.report(msg, None)
    return {"uuid": uid, "phase": rec.phase, "status_code": code, "message": msg}


# ─────────────────────────────────────────────────────────────────────────────
# 引擎品質報告（odm_report/stats.json 等；格式依 OpenSfM a125a3d stats.py 與 ODX v3.8.4，見 R1 §7–8）
# ─────────────────────────────────────────────────────────────────────────────

_QUALITY_FILES = {"stats_json": "odm_report/stats.json", "report_pdf": "odm_report/report.pdf",
                  "shots_geojson": "odm_report/shots.geojson", "log_json": "log.json", "images_json": "images.json",
                  "task_output": "task_output.txt"}


def _load_json(path: Path):
    try:
        return json.loads(path.read_text(encoding="utf-8")), None
    except FileNotFoundError:
        return None, "missing"
    except (OSError, ValueError) as e:
        return None, f"{type(e).__name__}: {e}"


def _f(v) -> Optional[float]:
    if isinstance(v, bool) or v is None:
        return None
    try:
        x = float(v)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def _avail(v) -> Optional[float]:
    """OpenSfM 用 -1 表示「沒有這個值」（S stats.py:754-841）。"""
    x = _f(v)
    return None if x is None or x == -1 else x


def _xyz(d) -> Optional[dict]:
    if not isinstance(d, dict):
        return None
    return {k: _f(d.get(k)) for k in ("x", "y", "z")}


def _err_block(b) -> Optional[dict]:
    """誤差區塊（S stats.py:115-156）：mean＝帶號平均（偏差）、std＝母體標準差、error＝各軸 RMS；單位 m（ENU）。"""
    if not isinstance(b, dict) or not b:
        return None
    rms = _xyz(b.get("error"))
    out = {"rms": rms, "mean": _xyz(b.get("mean")), "std": _xyz(b.get("std")),
           "average_error": _f(b.get("average_error")), "ce90": _f(b.get("ce90")), "le90": _f(b.get("le90"))}
    if rms and rms["x"] is not None and rms["y"] is not None:
        out["rms_horizontal"] = math.hypot(rms["x"], rms["y"])
    if "average_gps_std" in b:
        out["average_gps_std"] = _xyz(b.get("average_gps_std"))
    return out


def _geo_txt_sigmas(path) -> dict:
    """geo.txt → {影像檔名: (水平欄, 垂直欄)}（只收 ≥ 9 欄、兩欄都是數字的行）。"""
    out = {}
    for ln in Path(path).read_text(encoding="utf-8").splitlines()[1:]:
        parts = ln.split()
        if len(parts) >= 9:
            try:
                out[parts[0]] = (float(parts[7]), float(parts[8]))
            except ValueError:
                continue
    return out


_IDENTITY_TOL = 1e-9


def _bias_is_identity(b) -> Optional[bool]:
    """camera_errors[*].bias 是不是單位轉換（旋轉 0、平移 0、尺度 1）⇒ OpenSfM 沒有做 GPS 偏差補償。"""
    if not isinstance(b, dict):
        return None
    try:
        rot = [float(v) for v in b.get("rotation") or []]
        tr = [float(v) for v in b.get("translation") or []]
        sc = float(b.get("scale"))
    except (TypeError, ValueError):
        return None
    if len(rot) != 3 or len(tr) != 3:
        return None
    return all(abs(v) <= _IDENTITY_TOL for v in rot + tr) and abs(sc - 1.0) <= _IDENTITY_TOL


_ENGINE_WARN_RE = re.compile(r"(?i)force[-_ ]?gps|\bgcps?\b|gcp_list|ground control|checkpoint|\bbias\b")
_ENGINE_FORCE_GPS_RE = re.compile(r"(?i)force[-_ ]?gps")


def read_odm_quality(root, *, n_uploaded: Optional[int] = None, sent_options: Optional[list] = None,
                     force_gps: Optional[dict] = None, node_engine_version=None, uuid: Optional[str] = None,
                     gcp_uploaded: bool = False, gps_accuracy: Optional[dict] = None, geo_txt=None,
                     geo_txt_expected: bool = False) -> dict:
    """讀 ODX 成果的品質報告摘要（純讀檔）。缺檔或格式不符一律寫進 ``warnings``（不靜默略過）。

    檔案以壓縮檔裡的確切路徑找（7z -r 可能多出同名項目，R1 §7）：odm_report/stats.json、report.pdf、shots.geojson、
    log.json、images.json、task_output.txt。

    ``gcp_uploaded``：上傳了 GCP ⇒ 核對引擎真的把它當控制點用（has_gcp、gcp_only）；``gps_accuracy`` 的來源是
    geo.txt 而且給了 ``geo_txt`` ⇒ 逐張核對 images.json 的 gps_xy／z_stddev 與 geo.txt 的精度欄（R1 §6.2：images.json
    是 ODX 實際用值的證據）。``geo_txt_expected`` 而找不到 geo.txt ⇒ 警告（無法核對）。
    """
    root = Path(root)
    sent = {o.get("name"): o.get("value") for o in sent_options or [] if isinstance(o, dict)}
    q = {"format": QUALITY_FORMAT, "uuid": uuid, "root": str(root), "warnings": [], "stats_available": False,
         "files": {k: (rel if (root / rel).is_file() else None) for k, rel in _QUALITY_FILES.items()},
         "force_gps": (force_gps or {}).get("decision") if isinstance(force_gps, dict) else None}
    why = "（任務帶了 --skip-report）" if sent.get("skip-report") else "（原因請看 task_output.txt／log.json）"
    stats, err = _load_json(root / _QUALITY_FILES["stats_json"])
    if err == "missing":
        q["warnings"].append(f"找不到 odm_report/stats.json：沒有引擎的品質數字{why}")
    elif err is not None:
        q["warnings"].append(f"odm_report/stats.json 無法解析（{err}）")
    elif not isinstance(stats, dict) or not isinstance(stats.get("reconstruction_statistics"), dict):
        q["warnings"].append("odm_report/stats.json 格式不符（缺 reconstruction_statistics）")
    if q["files"]["report_pdf"] is None:
        q["warnings"].append(f"找不到 odm_report/report.pdf{why}")
    if q["files"]["shots_geojson"] is None:
        q["warnings"].append("找不到 odm_report/shots.geojson（重建沒有地理參考，或報告階段沒跑）")

    shots_n = None
    if q["files"]["shots_geojson"]:
        shots, serr = _load_json(root / _QUALITY_FILES["shots_geojson"])
        if isinstance(shots, dict) and isinstance(shots.get("features"), list):
            shots_n = len(shots["features"])
        else:
            q["warnings"].append(f"shots.geojson 無法解析（{serr or '不是 FeatureCollection'}）")

    if isinstance(stats, dict) and isinstance(stats.get("reconstruction_statistics"), dict):
        q["stats_available"] = True
        rs = stats["reconstruction_statistics"]
        initial, recon = _avail(rs.get("initial_shots_count")), _avail(rs.get("reconstructed_shots_count"))
        q["images"] = {
            "uploaded": n_uploaded, "initial_shots": None if initial is None else int(initial),
            "reconstructed_shots": None if recon is None else int(recon),
            "not_reconstructed": None if initial is None or recon is None else int(initial - recon),
            "missing_before_sfm": None if initial is None or n_uploaded is None else int(n_uploaded - initial),
            "shots_geojson_features": shots_n}
        comp = _avail(rs.get("components"))
        q["reconstruction"] = {
            "components": None if comp is None else int(comp), "has_gps": rs.get("has_gps"),
            "has_gcp": rs.get("has_gcp"), "reprojection_error_pixels": _avail(rs.get("reprojection_error_pixels")),
            "reprojection_error_normalized": _avail(rs.get("reprojection_error_normalized")),
            "average_track_length": _avail(rs.get("average_track_length")),
            "reconstructed_points_count": _avail(rs.get("reconstructed_points_count")),
            "observations_count": _avail(rs.get("observations_count")),
            "reprojection_note": "平均殘差、排除 > 4 px 的殘差（RESIDUAL_PIXEL_CUTOFF，S stats.py:34,597）⇒ 偏樂觀"}
        q["gps_errors"] = _err_block(stats.get("gps_errors"))
        ge = stats.get("gcp_errors") if isinstance(stats.get("gcp_errors"), dict) else {}
        details = []
        for d in ge.get("details") or []:
            if isinstance(d, dict):
                details.append({"id": d.get("id"), "role": d.get("role"),
                                "error": _xyz(d.get("error")) if d.get("error") is not None else None,
                                "n_inliers": d.get("n_inliers"), "n_total": d.get("n_total")})
        q["gcp"] = {"gcp_only": _err_block(ge.get("gcp_only")), "checkpoints": _err_block(ge.get("cp_only")),
                    "all": _err_block({k: v for k, v in ge.items() if k not in ("details", "gcp_only", "cp_only",
                                                                                 "coordinate_system")}),
                    "details": details, "n_not_triangulated": sum(1 for d in details if d["error"] is None),
                    "coordinate_system": ge.get("coordinate_system")}
        q["errors_3d"] = _err_block(stats.get("3d_errors"))
        cams = stats.get("camera_errors") if isinstance(stats.get("camera_errors"), dict) else {}
        q["camera_bias"] = {cid: (c.get("bias") if isinstance(c, dict) else None) for cid, c in cams.items()}
        ps = stats.get("processing_statistics") if isinstance(stats.get("processing_statistics"), dict) else {}
        ops = stats.get("odm_processing_statistics") if isinstance(stats.get("odm_processing_statistics"), dict) else {}
        steps = ps.get("steps_times") if isinstance(ps.get("steps_times"), dict) else {}
        q["processing"] = {"steps_times_s": {k: _avail(v) for k, v in steps.items()},
                           "total_time_s": _f(ops.get("total_time")), "total_time_human": ops.get("total_time_human"),
                           "start_date": ps.get("start_date"), "end_date": ps.get("end_date")}
        q["gsd_cm"] = _f(ops.get("average_gsd"))
        q["area_m2"] = _avail(ps.get("area"))
        q["area_note"] = "相機中心的外包矩形面積（S stats.py:844-919），不是測區面積"
        q["output_coordinate_system"] = stats.get("output_coordinate_system")
        if q["images"]["missing_before_sfm"]:
            q["warnings"].append(f"上傳 {n_uploaded} 張，進 OpenSfM 只有 {q['images']['initial_shots']} 張")
        if q["images"]["not_reconstructed"]:
            q["warnings"].append(f"{q['images']['not_reconstructed']} 張沒有重建進模型")
        if q["reconstruction"]["components"] is not None and q["reconstruction"]["components"] > 1:
            q["warnings"].append(f"模型分成 {q['reconstruction']['components']} 塊互不相連的部分")
        if q["gcp"]["n_not_triangulated"]:
            q["warnings"].append(f"{q['gcp']['n_not_triangulated']} 個控制／檢核點沒有三角化（沒用上）")
        if gcp_uploaded:
            # 上傳了 GCP 不代表引擎拿它當控制點：全是 CHK-（ODX 當檢核點並自己開 --force-gps）、影像名對不上被丟掉…
            if rs.get("has_gcp") is not True:
                q["warnings"].append(f"上傳了 GCP，但引擎的 reconstruction_statistics.has_gcp = {rs.get('has_gcp')!r}："
                                     "GCP 沒有當控制點進平差（全是 CHK- 檢核點、影像名對不上被丟掉、或少於 3 個點）")
            if not q["gcp"]["gcp_only"]:
                q["warnings"].append("上傳了 GCP，但 stats.json 沒有 gcp_only 誤差：沒有任何點被當成控制點用")
        idents = [_bias_is_identity(b) for b in q["camera_bias"].values()]
        q["camera_bias_identity"] = (all(idents) if idents and None not in idents else None)
        if (force_gps or {}).get("decision") == "force_gps" and q["camera_bias_identity"]:
            q["warnings"].append("送了 --force-gps，但每台相機的偏差都是單位轉換（旋轉 0、平移 0、尺度 1）："
                                 "OpenSfM 的 GPS 偏差補償可能沒有生效（對位失敗時它會自己關掉，S reconstruction.py:927-940）"
                                 "⇒ GPS 誤差列是沒扣偏差的值")

    log, lerr = _load_json(root / _QUALITY_FILES["log_json"])
    if isinstance(log, dict):
        opts = log.get("options") if isinstance(log.get("options"), dict) else {}
        warn_msgs = []
        for st in log.get("stages") or []:
            for m in (st.get("messages") or []) if isinstance(st, dict) else []:
                if isinstance(m, dict) and str(m.get("type", "")).lower().startswith("warn"):
                    warn_msgs.append(str(m.get("message")))
        q["engine"] = {"engine": log.get("engine"), "version": log.get("version"), "odmVersion": log.get("odmVersion"),
                       "success": log.get("success"), "error": log.get("error"), "totalTime": _f(log.get("totalTime")),
                       "node_engine_version": node_engine_version,
                       "options": {k: opts.get(k) for k in ("force_gps", "max_concurrency", "gps_accuracy", "gcp", "geo",
                                                             "use_exif", "orthophoto_resolution", "dsm", "tiles",
                                                             "skip_report") if k in opts},
                       "warnings": warn_msgs[:50],
                       "gsd_capped": [m for m in warn_msgs if "Maximum resolution set to" in m][:5]}
        if log.get("success") is False:
            q["warnings"].append(f"log.json 記錄引擎執行失敗：{log.get('error')}")
        if node_engine_version not in (None, "", "?") and log.get("version") not in (None, node_engine_version):
            q["warnings"].append(f"引擎版本不一致：節點 /info 說 {node_engine_version}，log.json 說 {log.get('version')}")
        if "force_gps" in opts and not _same_value(opts["force_gps"], bool(sent.get("force-gps"))):
            q["warnings"].append(f"log.json 的 force_gps = {opts['force_gps']}，與我們的決定 {bool(sent.get('force-gps'))} 不同")
        if q["engine"]["gsd_capped"]:
            q["warnings"].append("正射／DEM 解析度被引擎依 GSD 壓低：" + q["engine"]["gsd_capped"][0])
        # 引擎在執行中改的東西（例如 GCP 全是 CHK- 時自己開 --force-gps、GCP 行被丟掉、偏差補償關掉）只出現在
        # log.json 的警告裡——log.json 的 options 是開始時抓的，看不出來 ⇒ 相關的警告一律拉到品質報告上
        relevant = [m for m in warn_msgs if _ENGINE_WARN_RE.search(m) and "Maximum resolution set to" not in m]
        for m in relevant[:10]:
            q["warnings"].append(f"引擎警告：{m}")
        decision = (force_gps or {}).get("decision") if isinstance(force_gps, dict) else None
        if decision != "force_gps" and any(_ENGINE_FORCE_GPS_RE.search(m) for m in relevant):
            q["force_gps_overridden_by_engine"] = True
            q["warnings"].append(f"引擎的警告提到 force-gps，而記錄的決定是 {decision!r}：引擎可能自己開了 --force-gps"
                                 "（例如 GCP 全是 CHK- 檢核點，X stages/dataset.py:306-308）⇒ GPS 誤差不是獨立檢核")
    else:
        q["engine"] = {"node_engine_version": node_engine_version}
        q["warnings"].append("找不到 log.json：無法核對引擎版本與實際參數" if lerr == "missing"
                             else f"log.json 無法解析（{lerr}）")

    imgs, ierr = _load_json(root / _QUALITY_FILES["images_json"])
    if isinstance(imgs, list):
        xy = [_f(p.get("gps_xy_stddev")) for p in imgs if isinstance(p, dict)]
        z = [_f(p.get("gps_z_stddev")) for p in imgs if isinstance(p, dict)]
        xs, zs = [v for v in xy if v is not None], [v for v in z if v is not None]
        q["images_json"] = {"n": len(imgs), "n_without_xy_std": len(xy) - len(xs),
                            "gps_xy_stddev_m": [min(xs), max(xs)] if xs else None,
                            "gps_z_stddev_m": [min(zs), max(zs)] if zs else None}
        if n_uploaded is not None and len(imgs) != n_uploaded:
            q["warnings"].append(f"images.json 有 {len(imgs)} 張，上傳的是 {n_uploaded} 張")
        if (gps_accuracy or {}).get("source") == "geo.txt":
            _check_images_json_sigmas(q, imgs, geo_txt, geo_txt_expected)
    elif ierr == "missing":
        q["warnings"].append("找不到 images.json：無法確認 ODX 實際用的每張 GPS 精度（gps_xy／z_stddev，R1 §6.2）")
    else:
        q["warnings"].append(f"images.json 無法解析（{ierr}）")
    if q["files"]["task_output"] is None:
        q["warnings"].append("找不到 task_output.txt：引擎主控台全文不在成果裡（節點重開後也取不回）")
    return q


def _check_images_json_sigmas(q: dict, imgs: list, geo_txt, geo_txt_expected: bool) -> None:
    """images.json（ODX 套用 geo.txt 與 --gps-accuracy 之後的實際值）逐張對 geo.txt 的精度欄：不符 ⇒ 警告。

    引擎預設（水平 3 m、垂直 9 m）出現在本來有精度欄的影像上，代表 geo.txt 的精度沒有被讀到（欄數不對、或送了
    --gps-accuracy 蓋掉）。容差 0.0015 m（geo.txt 印到 mm）。"""
    if geo_txt is None:
        if geo_txt_expected:
            q["warnings"].append("找不到送出的 geo.txt：無法逐張核對 images.json 的 GPS 精度")
        return
    try:
        want = _geo_txt_sigmas(geo_txt)
    except (OSError, UnicodeDecodeError) as e:
        q["warnings"].append(f"讀不到送出的 geo.txt（{e}）：無法逐張核對 images.json 的 GPS 精度")
        return
    mism, missing, defaults, n_checked = [], [], 0, 0
    got = {str(p.get("filename")): p for p in imgs if isinstance(p, dict) and p.get("filename")}
    for name, (h, v) in sorted(want.items()):
        p = got.get(name)
        if p is None:
            missing.append(name)
            continue
        gx, gz = _f(p.get("gps_xy_stddev")), _f(p.get("gps_z_stddev"))
        n_checked += 1
        if gx is None or gz is None or abs(gx - h) > 0.0015 or abs(gz - v) > 0.0015:
            mism.append(f"{name}：geo.txt {h:g}/{v:g}，images.json {gx}/{gz}")
            if gx == 3.0 and gz in (3.0, 9.0):
                defaults += 1
    q["images_json"]["geo_txt_check"] = {"n_checked": n_checked, "n_mismatch": len(mism), "n_missing": len(missing),
                                         "n_engine_default": defaults, "examples": mism[:5]}
    if mism:
        q["warnings"].append(f"images.json 有 {len(mism)} 張的 GPS 精度與 geo.txt 不同（例如 {mism[:2]}）"
                             + (f"；其中 {defaults} 張是引擎預設 3 m／9 m：geo.txt 的精度欄沒被讀到" if defaults else ""))
    if missing:
        q["warnings"].append(f"geo.txt 的 {len(missing)} 張影像不在 images.json（例如 {missing[:3]}）")


def _fmt_m(v) -> str:
    return "—" if v is None else f"{v:.3f}"


def _err_text(b: Optional[dict]) -> str:
    if not b or not b.get("rms"):
        return "沒有樣本"
    r = b["rms"]
    s = f"RMS E {_fmt_m(r['x'])}／N {_fmt_m(r['y'])}／U {_fmt_m(r['z'])} m"
    if b.get("rms_horizontal") is not None:
        s += f"（水平 {b['rms_horizontal']:.3f}）"
    if b.get("ce90") is not None or b.get("le90") is not None:
        s += f"；CE90 {_fmt_m(b.get('ce90'))}、LE90 {_fmt_m(b.get('le90'))}"
    m = b.get("mean") or {}
    if any(m.get(k) is not None for k in ("x", "y", "z")):
        s += f"；平均（偏差）{_fmt_m(m.get('x'))}／{_fmt_m(m.get('y'))}／{_fmt_m(m.get('z'))}"
    return s


def odm_quality_rows(q: dict) -> list:
    """品質摘要 → QA 表的列 (機, 項目, 數值, 結果)，與 :func:`qa_table_rows` 同格式；
    結果 False 只用在明確的缺陷（沒有品質報告、模型分塊、引擎回報失敗），其餘是參考值 None（不自訂門檻）。"""
    who = "ODM"
    rows = []
    has = bool(q.get("stats_available"))
    rows.append((who, "品質報告 stats.json",
                 q["files"].get("stats_json") if has else "⚠ 沒有可用的 stats.json（見警告）", has))
    rows.append((who, "品質報告 report.pdf", q["files"].get("report_pdf") or "⚠ 沒有", None))
    if has:
        im = q.get("images") or {}
        rows.append((who, "影像（上傳／進 OpenSfM／重建）",
                     f"{im.get('uploaded', '—')}／{im.get('initial_shots', '—')}／{im.get('reconstructed_shots', '—')}"
                     + (f"（⚠ {im['not_reconstructed']} 張沒重建）" if im.get("not_reconstructed") else ""), None))
        rc = q.get("reconstruction") or {}
        comp = rc.get("components")
        rows.append((who, "模型分塊數", "—" if comp is None else str(comp), None if comp is None else comp == 1))
        rp = rc.get("reprojection_error_pixels")
        rows.append((who, "重投影誤差（平均，排除 > 4 px）", "—" if rp is None else f"{rp:.2f} px", None))
        if q.get("force_gps") == "force_gps":
            gps_note = ("（--force-gps，但⚠ 偏差補償可能沒生效：相機偏差都是單位轉換）" if q.get("camera_bias_identity")
                        else "（--force-gps：已扣掉每台相機的偏差，常數偏移看相機偏差列）")
        elif q.get("force_gps_overridden_by_engine"):
            gps_note = "（⚠ 引擎可能自己開了 --force-gps：不是獨立檢核，見警告）"
        elif q.get("force_gps") == "gcp_only":
            gps_note = "（只用 GCP 平差：PPK 對 GCP 解的獨立檢核）"
        else:
            gps_note = ""
        rows.append((who, "照片位置（geo.txt）對解算位置" + gps_note, _err_text(q.get("gps_errors")), None))
        g = q.get("gcp") or {}
        if g.get("gcp_only") or g.get("details"):
            rows.append((who, "GCP 誤差", _err_text(g.get("gcp_only")), None))
        if g.get("checkpoints"):
            rows.append((who, "檢核點誤差（CHK-，不進平差）", _err_text(g.get("checkpoints")), None))
        if q.get("force_gps") == "force_gps":
            for cid, b in (q.get("camera_bias") or {}).items():
                if isinstance(b, dict):
                    t = b.get("translation")
                    rows.append((who, f"相機偏差 {cid}",
                                 f"平移 {t}、旋轉 {b.get('rotation')}、尺度 {b.get('scale')}", None))
        gsd = q.get("gsd_cm")
        rows.append((who, "平均 GSD", "—" if gsd is None else f"{gsd:.2f} cm/px", None))
        area = q.get("area_m2")
        rows.append((who, "相機中心外包面積", "—" if area is None else f"{area / 1e4:.1f} ha", None))
        pr = q.get("processing") or {}
        rows.append((who, "處理時間", pr.get("total_time_human") or (
            "—" if pr.get("total_time_s") is None else f"{pr['total_time_s']:.0f} s"), None))
    eng = q.get("engine") or {}
    if eng.get("engine") or eng.get("version"):
        rows.append((who, "引擎（log.json）", f"{eng.get('engine')} {eng.get('version')}"
                     + (f"（節點 /info：{eng.get('node_engine_version')}）" if eng.get("node_engine_version") else ""),
                     False if eng.get("success") is False else None))
    for w in q.get("warnings") or []:
        rows.append((who, "警告", f"⚠ {w}", None))
    return rows


def odm_quality_text(q: dict) -> str:
    """品質摘要的純文字版（每列「項目：數值」）。"""
    return "\n".join(f"{item}：{val}" for _, item, val, _ in odm_quality_rows(q))
