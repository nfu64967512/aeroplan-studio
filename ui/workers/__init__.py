"""背景執行緒 worker：把耗時運算移出 GUI 執行緒。

目前只有 :class:`ui.workers.planning_worker.HarvestPlanningWorker`（HARVEST 覆蓋規劃）。
寫法一律比照 ``mission/sitl_link.py`` 的 ``SITLLink``：繼承 QThread、對外只走
pyqtSignal、失敗走自己的信號（不可用「成功但空結果」代表失敗）、worker 內不碰任何 widget。
"""

from ui.workers.planning_worker import HarvestPlanningWorker

__all__ = ["HarvestPlanningWorker"]
