"""ui/workers/planning_worker.py — HARVEST 覆蓋規劃背景執行緒。

為什麼要有這個檔：``ui/main_window.py`` 的 DCCPP 規劃是**同步跑在 GUI 執行緒**上的
（唯一的讓步是呼叫前一次 ``QApplication.processEvents()``），小情境 1 秒還能忍，
8 機 107 條帶的山區情境要數十秒 ⇒ Windows 會把標題列改成「沒有回應」。
理由與量測見 ``docs/AEROPLAN_接入手冊.md`` §7、``docs/adr/0008-planning-in-worker-thread.md``。

寫法比照 ``mission/sitl_link.py:169 SITLLink``：
    * 繼承 QThread，對外只走 pyqtSignal；
    * 失敗**有自己的信號**（``failed``），不用「成功但空 dict」代表失敗——
      空結果是一個合法答案（真的沒有可覆蓋區域），兩者混在同一條路上，UI 就分不出來；
    * ``run()`` 內不碰任何 widget，只做純運算並 emit。

⚠⚠ 取消：**做不到，只能放棄結果**。
兩層原因，兩層都要知道：
  1. ``hv_dccpp.planner.plan_mission`` 沒有取消點；
  2. 就算有，轉接層的 ``HvDccppCoordinator._tick``（``aeroplan_adapter.py:1705-1710``）把
     progress 回呼包在 ``try: ... except Exception: pass``（註解寫「回呼失敗不得影響規劃」）
     ⇒ **從回呼裡拋例外中止規劃這條路是死的**，例外根本傳不出來。
所以 ``stop()`` 只做兩件誠實的事：停止再發 progress、以及**跑完後不採用結果**
（不 emit ``finished_ok``，改 emit ``cancelled``）。執行緒本身會把那一趟規劃跑完才結束。
絕對不要改用 ``terminate()``：會在 numpy/scipy 的 C 層留下不一致狀態。
視窗關閉時的處理見 ``ui/main_window.py`` 的 ``_hv_park_running_worker``。
"""
from __future__ import annotations

from typing import Any, Callable, Optional

from PyQt6.QtCore import QThread, pyqtSignal

from utils.logger import get_logger

logger = get_logger()


class HarvestPlanningWorker(QThread):
    """在背景執行緒跑 ``SwarmCoordinator.plan_coverage_dccpp``。

    刻意接受**已經設定好旗標的** coordinator，而不是自己組 HvDccppCoordinator：
    這樣 GUI 路徑與 headless 路徑共用 ``mission/swarm_coordinator.py`` 的同一個接縫，
    不會出現「兩條路徑參數覆蓋不一致」那類靜默失效。

    信號：
        progress(fraction 0.0–1.0, 訊息)   — 六個階段邊界（見手冊 §7.4）
        finished_ok(6 鍵 dict)             — 與既有 DCCPP 回傳同形
        failed(錯誤字串)                    — 規劃失敗（含 preflight NO_GO）
        report(dict)                       — {'status','errors','warnings','notes',
                                              'makespan_s','timing_ms','uncovered'}
        cancelled()                        — 呼叫過 stop()：規劃已跑完但結果不採用
    """

    progress = pyqtSignal(float, str)
    finished_ok = pyqtSignal(dict)
    failed = pyqtSignal(str)
    report = pyqtSignal(dict)
    cancelled = pyqtSignal()

    def __init__(self, coordinator: Any, plan_kwargs: dict,
                 *, parent=None) -> None:
        super().__init__(parent)
        self._coordinator = coordinator
        self._plan_kwargs = dict(plan_kwargs)
        self._stop = False
        self.result: Optional[dict] = None      # 完成後填入，供主執行緒讀回

    # ── 公開控制 ──────────────────────────────────────────────────────
    def stop(self) -> None:
        """放棄這一趟規劃的結果。

        ⚠ **不會**讓規劃提早結束（見模組 docstring：hv 沒有取消點，而且會把
        progress 回呼的例外吞掉）。執行緒仍會把整趟跑完，只是不再發 progress，
        而且結束時發 ``cancelled`` 而不是 ``finished_ok``。
        """
        self._stop = True

    # 與 SITLLink.stop() 同名慣例，另留一個語意別名給呼叫端讀起來順一點
    cancel = stop

    @property
    def is_cancelled(self) -> bool:
        return self._stop

    @property
    def coordinator(self) -> Any:
        """規劃用的 SwarmCoordinator；完成後 ``.hv_coordinator`` 可取匯出／上傳／重規劃介面。"""
        return self._coordinator

    # ── 內部 ──────────────────────────────────────────────────────────
    def _tick(self, frac: float, msg: str) -> None:
        # ⚠ 這裡**不可以**用「拋例外」來中止規劃：轉接層把回呼包在
        #   try/except Exception: pass 裡，例外傳不出去（曾經這樣寫過，是死的）。
        if self._stop:
            return
        self.progress.emit(float(frac), str(msg))

    def run(self) -> None:                                  # noqa: D102 - QThread 入口
        coord = self._coordinator
        # progress / report 在 worker 執行緒內綁定，emit 由 Qt 自動排隊回主執行緒
        coord.hv_progress = self._tick
        coord.hv_on_report = self.report.emit
        try:
            result = coord.plan_coverage_dccpp(**self._plan_kwargs)
        except Exception as e:                              # noqa: BLE001 - 執行緒不得帶例外死掉
            detail = "\n".join(f"  • {x}" for x in (getattr(e, 'errors', None) or []))
            logger.error(f'[HARVEST] 規劃失敗：{e}', exc_info=True)
            if self._stop:
                # 已經被放棄了就不要再彈錯誤視窗（視窗多半正在關）
                self.cancelled.emit()
            else:
                self.failed.emit(f"{e}\n{detail}" if detail else str(e))
            return
        finally:
            # 不要讓 worker 的 bound method 留在 coordinator 上（執行緒結束後就不該再被呼叫）
            coord.hv_progress = None
            coord.hv_on_report = None

        if self._stop:
            # 規劃已經跑完，但呼叫端（多半是關視窗）已經不要這個結果了
            logger.info('[HARVEST] 規劃完成但已被要求放棄，結果不採用')
            self.cancelled.emit()
            return
        if not isinstance(result, dict):
            self.failed.emit(f"規劃器回傳型別異常：{type(result).__name__}")
            return
        self.result = result
        logger.info(
            f"[HARVEST] 完成：makespan={float(result.get('total_makespan', 0.0)):.1f}s "
            f"{len(result.get('assembled_paths') or {})} 機")
        self.finished_ok.emit(result)
