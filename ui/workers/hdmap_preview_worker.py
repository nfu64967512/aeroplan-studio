"""ui/workers/hdmap_preview_worker.py — 「預覽高精地圖成果」背景執行緒（survey.preview.compute_hdmap_preview）。

建圖模式規劃完成後由 MainWindow 啟動：用**跟上傳閘門同一份有效輸入**（測區 − 禁航區、規劃 DEM、最強規劃風、
空速／觸發間隔）跑曝光驗證器，再把視數柵格分級成預計成果圖（規劃預測，非實際成果）。

寫法比照 ``wind_grid_worker`` / ``flat_search_worker``：QThread、只走信號、失敗有自己的信號、``run()`` 內不碰 widget。
* ``compute_hdmap_preview`` 照契約不丟例外、做不出來回 ``ok=False`` + 中文原因 ⇒ 這裡把它轉成 ``failed(原因)``
  （house rule：失敗不可以用「成功但空結果」表示）；萬一還是丟了例外也走 ``failed``。
* 取消：算圖沒有取消點，只能放棄結果（``stop()``）；還在跑的執行緒由 MainWindow 寄放到程式結束前
  （在還在跑的 QThread 上解構是 qFatal）。
* ``stamp``：啟動當時的 ``(hv, hv.last_result)``，MainWindow 收到結果時用它判斷計畫是不是已經換掉（換了就丟）。
  ``result`` / ``error_text`` 在發信號**之前**寫好，MainWindow 用它確認信號是目前這個 worker 發的。
"""
from __future__ import annotations

from typing import Any, Optional, Tuple

from PyQt6.QtCore import QThread, pyqtSignal

from utils.logger import get_logger

logger = get_logger()


class HdmapPreviewWorker(QThread):
    """信號：``finished_ok(HdmapPreview)``（``ok=True`` 的那種）、``failed(錯誤字串)``。"""

    finished_ok = pyqtSignal(object)
    failed = pyqtSignal(str)

    def __init__(self, hv, swarm, *, stamp: Optional[Tuple[Any, Any]] = None,
                 cell_size_m: Optional[float] = None, parent=None) -> None:
        super().__init__(parent)
        self._hv = hv
        self._swarm = swarm
        self._cell_size_m = None if cell_size_m is None else float(cell_size_m)
        #: 啟動當時的 (hv, hv.last_result)；沒給就自己拍一張
        self.stamp = stamp if stamp is not None else (hv, getattr(hv, 'last_result', None))
        self._abandoned = False
        #: 成功時的 HdmapPreview（發 finished_ok 之前寫好）
        self.result = None
        #: 失敗原因（發 failed 之前寫好）
        self.error_text: Optional[str] = None

    @property
    def abandoned(self) -> bool:
        return self._abandoned

    def stop(self) -> None:
        """放棄結果（計算本身停不下來）。"""
        self._abandoned = True

    def run(self) -> None:                                   # noqa: D401 - QThread 入口
        try:
            from survey.preview import compute_hdmap_preview
            kw = {} if self._cell_size_m is None else {'cell_size_m': self._cell_size_m}
            pv = compute_hdmap_preview(self._hv, self._swarm, **kw)
        except Exception as e:                               # noqa: BLE001 - 一律走 failed，不讓執行緒帶著例外死掉
            msg = f'{type(e).__name__}: {e}'
            logger.error(f'[高精地圖預覽] 計算失敗：{msg}', exc_info=True)
            self._emit_failed(msg)
            return
        if self._abandoned:
            logger.info('[高精地圖預覽] 結果已放棄（計畫已變更或視窗關閉）')
            return
        if pv is None or not bool(getattr(pv, 'ok', False)):
            reason = str(getattr(pv, 'reason', None) or '預覽沒有產生結果（原因不明）')
            logger.warning(f'[高精地圖預覽] 無法產生：{reason}')
            self._emit_failed(reason)
            return
        self.result = pv
        self.finished_ok.emit(pv)

    def _emit_failed(self, msg: str) -> None:
        if self._abandoned:
            return
        self.error_text = str(msg)
        self.failed.emit(self.error_text)
