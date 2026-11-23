"""ui/workers/survey_worker.py — 航測後處理的背景執行緒（不卡 GUI）。

兩個 worker，寫法比照 ``flat_search_worker.FlatSearchWorker``：QThread、只走信號、失敗有自己的信號、
``run()`` 內不碰任何 widget。

* :class:`SurveyProcessWorker`：跑 ``survey.pipeline.process_campaign``（讀日誌、配對、PPK、交付物、QA）。
* :class:`OdmSubmitWorker`：跑 ``survey.pipeline.run_odm_job``（查節點、分批上傳、提交、等待、下載、解壓、品質報告），
  或同簽章的 ``resume_odm_job``（接回）、``cancel_odm_job``（取消遠端任務）、對話框的「測試連線」。

取消：設旗標，流水線在步驟之間（每架次、每架的影像匯出、ODM 上傳的每個讀取區塊、等待的每 0.5 s、下載的每個區塊）
檢查；正在跑的單一步驟（例如讀一份大日誌、RTKLIB 解算、一個卡住的 API 呼叫）要等它結束才會停。取消後輸出目錄可能
只寫了一部分。本模組不開任何子行程；只有使用者明確開啟 RTKLIB 時，流水線才經由 survey.ppk.rtklib 呼叫它。
"""
from __future__ import annotations

import inspect
import threading
from typing import Any, Callable, Dict, Optional, Sequence

from PyQt6.QtCore import QThread, pyqtSignal

from utils.logger import get_logger

logger = get_logger()


def _message(e: Exception) -> str:
    """流水線自己的錯誤訊息是給人看的；其他例外加上型別。"""
    from survey.pipeline import PipelineError
    return str(e) if isinstance(e, PipelineError) else f"{type(e).__name__}: {e}"


def _accepts_kw(fn, name: str) -> bool:
    """``fn`` 收不收 ``name=`` 這個關鍵字參數（有 **kwargs 也算）。看不到簽章 ⇒ 不送（舊的假函式簽章是固定的）。"""
    try:
        params = inspect.signature(fn).parameters
    except (TypeError, ValueError):
        return False
    return name in params or any(p.kind is p.VAR_KEYWORD for p in params.values())


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
    """信號：``progress(訊息, 0–1；-1 = 不明)``、``state(dict)``、``finished_ok(dict)``、``failed(錯誤)``、``canceled()``。

    ``run_fn(job, *, client, progress, should_cancel, poll_s)``（預設 ``run_odm_job``）；``run_fn`` 收 ``on_state``
    （或 **kwargs）時才送 ``on_state``，結構化狀態（phase、uuid、上傳檔數／位元組、遠端狀態與進度…）經 ``state`` 信號
    回到 GUI 執行緒。取消的意思由流水線決定：提交前 ⇒ 停止上傳（遠端未提交的上傳節點 48 h 後自清）；提交後 ⇒ 只停本機的
    等待與下載，NodeODX 上的任務繼續跑（可接回）。實際的取消訊息放在 ``cancel_message``（``canceled`` 信號沒有參數）。
    """

    progress = pyqtSignal(str, float)
    state = pyqtSignal(dict)
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
        self.cancel_message: Optional[str] = None

    def cancel(self) -> None:
        self._cancel.set()

    def is_canceled(self) -> bool:
        return self._cancel.is_set()

    def _emit_progress(self, msg: str, frac: Optional[float] = None) -> None:
        self.progress.emit(str(msg), -1.0 if frac is None else float(frac))

    def _emit_state(self, st: dict) -> None:
        self.state.emit(dict(st))

    def run(self) -> None:                                   # noqa: D401 - QThread 入口
        from survey.pipeline import PipelineCanceled, run_odm_job
        fn = self._run_fn or run_odm_job
        kw = dict(client=self._client, progress=self._emit_progress, should_cancel=self._cancel.is_set,
                  poll_s=self._poll_s)
        if _accepts_kw(fn, "on_state"):
            kw["on_state"] = self._emit_state
        try:
            res = fn(self._job, **kw)
        except PipelineCanceled as e:
            # 訊息由流水線說清楚是哪一種取消（提交前 ⇒ 上傳已放棄；提交後 ⇒ 遠端任務仍在跑、可接回）
            self.cancel_message = str(e) or "已取消"
            logger.info(f"[ODM] 已停止：{self.cancel_message}")
            self.canceled.emit()
            return
        except Exception as e:                               # noqa: BLE001
            msg = _message(e)
            logger.error(f"[ODM] 失敗：{msg}")
            self.failed.emit(msg)
            return
        self.finished_ok.emit(dict(res))
