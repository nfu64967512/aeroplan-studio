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
"""

from __future__ import annotations

import json
import logging
import math
import time
import zipfile
from collections import Counter
from dataclasses import asdict, dataclass, field, is_dataclass
from pathlib import Path
from typing import Callable, Optional, Sequence

import numpy as np

from survey import datum, deliverable, dflog, exif_write, geotag, ingest, odm_client
from survey import pose as pose_mod
from survey import qa as qa_mod
from survey.ppk import events as ppk_events
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


class PipelineError(ValueError):
    """輸入或設定不合理（訊息是給操作員看的繁中說明）。"""


class PipelineCanceled(RuntimeError):
    """使用者取消。"""


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
    （``base_obs`` RINEX 或 ``base_ubx`` 二擇一）。沒開 RTKLIB 時每架都要給 ``SortieInput.pos_path``。
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
        if self.run_rtklib:
            if self.base is None:
                raise PipelineError("PPK：要跑 RTKLIB 就必須給基站座標（緯度、經度、橢球高、天線高、框架）")
            if (self.base_obs is None) == (self.base_ubx is None):
                raise PipelineError("PPK：基站觀測要給 RINEX 或 UBX 其中一個（不能都沒有、也不能都給）")

    def effective(self) -> dict:
        return {
            "frame": self.frame, "base": None if self.base is None else asdict(self.base),
            "base_obs": _s(self.base_obs), "base_ubx": _s(self.base_ubx), "run_rtklib": bool(self.run_rtklib),
            "rtklib_overrides": dict(self.rtklib_overrides or {}), "edge": self.edge,
            "fix_ratio_threshold": float(self.fix_ratio_threshold), "interp_method": self.interp_method,
            "max_gap_s": float(self.max_gap_s), "pair_tol_s": float(self.pair_tol_s),
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


def _apply_ppk(res, rows: list, sortie: SortieInput, ppk: PpkSettings, *, sensor_ts_semantics: str,
               geoid, base_rinex, work_dir, should_cancel) -> tuple:
    """回傳 (合併後的列, PPK 摘要)。摘要含 reasons（QA 未通過的原因）與 warnings。"""
    session = res.session
    cs = sortie.callsign
    ubx_path = rover_ubx_for(sortie)
    traj, traj_info = _ppk_trajectory(sortie, ubx_path, ppk, base_rinex, work_dir)
    _check_cancel(should_cancel)
    ulog = ppk_ubx.read_ubx_file(ubx_path)
    ex = ppk_events.extract_edges(ulog, edge=ppk.edge)
    warnings, reasons = [], []
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
    merged, used, rejected = [], 0, Counter()
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
        used += 1

    info = {
        "trajectory": {**traj_info, "n_epochs": len(traj), "datum_label": traj.datum_label,
                       "skipped_lines": list(traj.skipped_lines), "has_velocity": traj.vel_ned_mps is not None},
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
            ppk_info = {"error": msg, "reasons": [msg], "warnings": []}
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


def odm_job(result: CampaignResult, *, node_url: str, orthophoto_resolution_cm: float, rolling_shutter: bool,
            qa_override: bool = False, name: Optional[str] = None, gcp_txt=None, gcp_vertical: Optional[str] = None,
            checkpoint_ids: Sequence[str] = ()) -> dict:
    """組 NodeODM 任務。QA 未通過時必須 ``qa_override=True`` 才組得出來（並記在任務紀錄裡）。

    ``gcp_txt``：使用者標好影像座標的 gcp_list.txt（選填）；``gcp_vertical`` 是它 geo_z 的高程基準
    （'ellipsoidal'／'orthometric'，必填且要等於 geo.txt 用的高度）。核對摘要放在 ``job['gcp']``；
    沒給 GCP 時任務內容與加入這個功能前完全相同。
    """
    url = (node_url or "").strip()
    if not url.lower().startswith(("http://", "https://")):
        raise PipelineError(f"NodeODM 位址要是 http(s)://…，收到 {node_url!r}")
    if result.paths.get("geo_txt") is None or not result.images:
        raise PipelineError("沒有 geo.txt 或加標籤影像（見處理錯誤），不能送 ODM")
    if not result.passed and not qa_override:
        raise PipelineError("QA 未通過：" + "；".join(result.qa["reasons"][:3])
                            + ("…" if len(result.qa["reasons"]) > 3 else "")
                            + "（要送出必須明確勾選「QA 未通過仍送出」）")
    readout = result.odm.get("rolling_shutter_readout_ms")
    if rolling_shutter and readout is None:
        raise PipelineError(f"不能開滾動快門校正：{result.odm.get('rolling_shutter_error')}")
    try:
        opts = odm_client.build_options(orthophoto_resolution_cm=float(orthophoto_resolution_cm),
                                        rolling_shutter_readout_ms=readout if rolling_shutter else None)
    except ValueError as e:
        raise PipelineError(f"ODM 參數：{e}") from None
    job = {"node_url": url, "images": [str(p) for p in result.images], "geo_txt": str(result.paths["geo_txt"]),
           "options": opts, "name": name, "qa_passed": result.passed, "qa_override": bool(qa_override),
           "qa_reasons": list(result.qa["reasons"]) if not result.passed else [],
           "rolling_shutter": bool(rolling_shutter), "out_dir": str(result.out_dir)}
    if gcp_txt is not None and str(gcp_txt).strip():
        rep = check_gcp_for_job(result, gcp_txt, gcp_vertical=gcp_vertical, checkpoint_ids=checkpoint_ids)
        job["gcp_txt"] = str(gcp_txt)
        job["gcp"] = {k: rep[k] for k in ("header", "crs", "n_lines", "n_gcps", "gcp_vertical", "images_per_gcp",
                                          "warnings")}
    elif gcp_vertical is not None or checkpoint_ids:
        raise PipelineError("給了 GCP 高程基準或檢核點清單，卻沒有 gcp_list.txt")
    return job


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


def run_odm_job(job: dict, *, client=None, progress: Optional[Callable[[str, Optional[float]], None]] = None,
                should_cancel: Optional[Callable[[], bool]] = None, poll_s: float = 10.0,
                timeout_s: float = 24 * 3600.0, sleep=time.sleep) -> dict:
    """上傳 → 等待 → 下載 all.zip → 解壓 → 找正射瓦片。任務紀錄寫在 out_dir/odm/submission.json。

    取消只停止本機的等待與下載；NodeODM 上的任務會繼續跑（客戶端沒有取消端點）。
    """
    out = Path(job["out_dir"]) / "odm"
    client = client or odm_client.NodeOdmClient(job["node_url"])
    record = {k: v for k, v in job.items() if k != "images"}
    record["n_images"] = len(job["images"])
    record["image_names"] = sorted(Path(p).name for p in job["images"])
    _dump_json(out / "submission.json", record)

    def report(msg, frac=None):
        _log.info("[ODM] %s", msg)
        if progress is not None:
            progress(msg, frac)

    _check_cancel(should_cancel)
    gcp_kw = {"gcp_txt": job["gcp_txt"]} if job.get("gcp_txt") else {}
    report(f"上傳 {len(job['images'])} 張影像與 geo.txt" + ("、gcp_list" if gcp_kw else "")
           + f" 到 {job['node_url']}…", 0.0)
    uuid = client.new_task(job["images"], options=job["options"], geo_txt=job["geo_txt"], name=job.get("name"),
                           **gcp_kw)
    record["uuid"] = uuid
    _dump_json(out / "submission.json", record)
    waited = [0.0]

    def _sleep(s):
        t = 0.0
        while t < s:
            _check_cancel(should_cancel)
            dt = min(0.5, s - t)
            sleep(dt)
            t += dt
        waited[0] += s
        report(f"NodeODM 處理中（任務 {uuid}，已等 {waited[0]:.0f} s）…", None)

    info = client.wait(uuid, poll_s=poll_s, timeout_s=timeout_s, sleep=_sleep)
    _check_cancel(should_cancel)
    report("下載成果 all.zip…", 0.8)
    zip_path = client.download_all(uuid, out / f"{uuid}_all.zip")
    extract_dir = extract_odm_zip(zip_path, out / f"result_{uuid}")
    tiles = find_ortho_tiles(extract_dir)
    record.update(zip=str(zip_path), extract_dir=str(extract_dir), ortho_tiles=_s(tiles))
    _dump_json(out / "submission.json", record)
    report("完成" + ("" if tiles else "（成果裡沒有 orthophoto_tiles：任務沒開 --tiles？）"), 1.0)
    return {"uuid": uuid, "info": info, "zip": zip_path, "extract_dir": extract_dir, "ortho_tiles": tiles}
