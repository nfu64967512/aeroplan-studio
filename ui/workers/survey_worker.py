"""ui/workers/survey_worker.py — 航測後處理的背景執行緒（不卡 GUI）。

兩個 worker，寫法比照 ``flat_search_worker.FlatSearchWorker``：QThread、只走信號、失敗有自己的信號、
``run()`` 內不碰任何 widget。

* :class:`SurveyProcessWorker`：跑 ``survey.pipeline.process_campaign``（讀日誌、配對、PPK、交付物、QA）。
* :class:`OdmSubmitWorker`：跑 ``survey.pipeline.run_odm_job``（上傳、等待、下載、解壓、找正射瓦片）。

取消：設旗標，流水線在步驟之間（每架次、每架的影像匯出、ODM 等待的每 0.5 s）檢查；正在跑的單一步驟
（例如讀一份大日誌、RTKLIB 解算）要等它結束才會停。取消後輸出目錄可能只寫了一部分。
本模組不開任何子行程；只有使用者明確開啟 RTKLIB 時，流水線才經由 survey.ppk.rtklib 呼叫它。
"""
from __future__ import annotations

import threading
from typing import Any, Callable, Dict, Optional, Sequence

from PyQt6.QtCore import QThread, pyqtSignal

from utils.logger import get_logger

logger = get_logger()


def _message(e: Exception) -> str:
    """流水線自己的錯誤訊息是給人看的；其他例外加上型別。"""
    from survey.pipeline import PipelineError
    return str(e) if isinstance(e, PipelineError) else f"{type(e).__name__}: {e}"


class SurveyProcessWorker(QThread):
    """信號：``progress(訊息, 0–1；-1 = 不明)``、``finished_ok(CampaignResult)``、``failed(錯誤)``、``canceled()``。"""

    progress = pyqtSignal(str, float)
    finished_ok = pyqtSignal(object)
    failed = pyqtSignal(str)
    canceled = pyqtSignal()

    def __init__(self, sorties: Sequence, out_dir, kwargs: Dict[str, Any], *,
                 process_fn: Optional[Callable] = None, parent=None) -> None:
        super().__init__(parent)
        self._sorties = list(sorties)
        self._out_dir = out_dir
        self._kwargs = dict(kwargs)
        self._process_fn = process_fn
        self._cancel = threading.Event()

    def cancel(self) -> None:
        self._cancel.set()

    def is_canceled(self) -> bool:
        return self._cancel.is_set()

    def _emit_progress(self, msg: str, frac: Optional[float] = None) -> None:
        self.progress.emit(str(msg), -1.0 if frac is None else float(frac))

    def run(self) -> None:                                   # noqa: D401 - QThread 入口
        from survey.pipeline import PipelineCanceled, process_campaign
        fn = self._process_fn or process_campaign
        try:
            res = fn(self._sorties, self._out_dir, progress=self._emit_progress,
                     should_cancel=self._cancel.is_set, **self._kwargs)
        except PipelineCanceled:
            logger.info("[航測處理] 已取消")
            self.canceled.emit()
            return
        except Exception as e:                               # noqa: BLE001 - 一律走 failed，不讓執行緒帶著例外死掉
            msg = _message(e)
            logger.error(f"[航測處理] 失敗：{msg}")
            self.failed.emit(msg)
            return
        if self._cancel.is_set():
            self.canceled.emit()
            return
        self.finished_ok.emit(res)


class OdmSubmitWorker(QThread):
    """信號：``progress(訊息, 0–1；-1 = 不明)``、``finished_ok(dict)``、``failed(錯誤)``、``canceled()``。

    取消只停止本機的等待與下載；NodeODM 上的任務會繼續跑。
    """

    progress = pyqtSignal(str, float)
    finished_ok = pyqtSignal(dict)
    failed = pyqtSignal(str)
    canceled = pyqtSignal()

    def __init__(self, job: Dict[str, Any], *, run_fn: Optional[Callable] = None, client=None,
                 poll_s: float = 10.0, parent=None) -> None:
        super().__init__(parent)
        self._job = dict(job)
        self._run_fn = run_fn
        self._client = client
        self._poll_s = float(poll_s)
        self._cancel = threading.Event()

    def cancel(self) -> None:
        self._cancel.set()

    def _emit_progress(self, msg: str, frac: Optional[float] = None) -> None:
        self.progress.emit(str(msg), -1.0 if frac is None else float(frac))

    def run(self) -> None:                                   # noqa: D401 - QThread 入口
        from survey.pipeline import PipelineCanceled, run_odm_job
        fn = self._run_fn or run_odm_job
        try:
            res = fn(self._job, client=self._client, progress=self._emit_progress,
                     should_cancel=self._cancel.is_set, poll_s=self._poll_s)
        except PipelineCanceled:
            logger.info("[ODM] 已取消（NodeODM 上的任務仍在執行）")
            self.canceled.emit()
            return
        except Exception as e:                               # noqa: BLE001
            msg = _message(e)
            logger.error(f"[ODM] 失敗：{msg}")
            self.failed.emit(msg)
            return
        self.finished_ok.emit(dict(res))
