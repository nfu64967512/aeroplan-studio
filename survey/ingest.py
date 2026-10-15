"""一個架次的 ingest：飛控 .BIN ＋ 樹莓派會期 → 配對、時鐘擬合、每張照片的位姿、現場 QA。

落地後的順序（評估 §8 流程 1–3）::

    log     = dflog.read_dflog(bin)              # 飛控重開機之前先取 SD 卡
    session = pi_session.read_pi_session(dir, verify_sha256=True)
    fit, fc = fit_session(log, session)          # 逐張邊緣配對，不依序硬配
    poses   = pose.compute_photo_poses(log, session, fit, fc, sensor_ts_semantics=...)
    qa      = qa.sortie_report(log, session, fit, fc, poses.rows)

:func:`ingest_sortie` 把上面串起來；任何一步的有效值都留在回傳物件裡。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from survey import clock_fit, dflog, pi_session, pose, qa


def fit_session(log, session, *, kind: str = "CAM", instance: Optional[int] = 0, **fit_kw):
    """飛控事件（CAM.TimeUS）與 Pi 邊緣（BOOTTIME）配對擬合。回傳 (ClockFit, 飛控事件表)。

    Relay 後端沒接回授腳時 CAM 與 GPIO 邊緣同一個 tick；接了回授腳時 CAM 是回授中斷時刻 ——
    兩種情況 CAM 都是邊緣事件，所以預設用 CAM（``kind='TRIG'`` 只給特殊診斷用）。
    """
    fc = dflog.cam_events(log, kind=kind, instance=instance)
    if not fc:
        raise clock_fit.ClockFitError(f"日誌沒有 {kind} 紀錄（LOG_BITMASK 沒開 Camera 位元？）")
    t_fc = np.asarray(fc["TimeUS"], np.float64) * 1e-6
    t_pi = session.edge_boottime_ns().astype(np.float64) * 1e-9
    return clock_fit.fit_clock(t_fc, t_pi, **fit_kw), fc


@dataclass
class IngestResult:
    log: object
    session: object
    fit: object
    fc_events: dict
    poses: object
    qa: dict


def ingest_sortie(bin_path, session_dir, *, sensor_ts_semantics: str, verify_sha256: bool = True,
                  fit_kw: Optional[dict] = None, **pose_kw) -> IngestResult:
    """讀檔、配對、算位姿、出 QA。``pose_kw`` 直接傳給 :func:`survey.pose.compute_photo_poses`。"""
    log = dflog.read_dflog(bin_path)
    session = pi_session.read_pi_session(session_dir, verify_sha256=verify_sha256)
    fit, fc = fit_session(log, session, **(fit_kw or {}))
    windows = dflog.strip_windows(log)
    poses = pose.compute_photo_poses(log, session, fit, fc, sensor_ts_semantics=sensor_ts_semantics,
                                     windows=windows, **pose_kw)
    report = qa.sortie_report(log, session, fit, fc, poses.rows, windows=windows)
    return IngestResult(log=log, session=session, fit=fit, fc_events=fc, poses=poses, qa=report)
