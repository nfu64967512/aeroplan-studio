"""ui/workers/flat_search_worker.py — 「找平坦位置」背景執行緒。

要連網下載地形圖磚（Cesium World Terrain）並整張掃描，約 10–20 s，不能跑在 GUI 執行緒。
寫法比照 ``planning_worker.HarvestPlanningWorker``：QThread、只走信號、失敗有自己的信號、``run()`` 內不碰 widget。

⚠ token 只在建構時收進來給取樣器用，**不放進任何信號、不記 log**。
⚠ 取消：做不到（下載與掃描沒有取消點），只能放棄結果；視窗關閉時由 MainWindow 等它跑完。
"""
from __future__ import annotations

from typing import Any, Dict

from PyQt6.QtCore import QThread, pyqtSignal

from utils.logger import get_logger

logger = get_logger()


class FlatSearchWorker(QThread):
    """信號：``progress(訊息)``、``finished_ok(結果 dict)``、``failed(錯誤字串)``。"""

    progress = pyqtSignal(str)
    finished_ok = pyqtSignal(dict)
    failed = pyqtSignal(str)

    def __init__(self, token: str, request: Dict[str, Any], *, parent=None) -> None:
        super().__init__(parent)
        self._token = token
        self._req = dict(request)
        self._abandoned = False

    def stop(self) -> None:
        """放棄結果（下載/掃描本身停不下來）。"""
        self._abandoned = True

    def run(self) -> None:                                   # noqa: D401 - QThread 入口
        try:
            from core.terrain import CesiumWorldTerrain, WorldTerrainError
            from core.terrain.flat_finder import FlatSearchError, areas_window, find_flat_windows
            req = self._req
            areas = req["areas"]
            h, w, center = areas_window(areas)
            scale = float(req.get("scale", 1.0))
            self.progress.emit("下載地形圖磚並掃描…（約 10–20 秒）")
            terrain = CesiumWorldTerrain(self._token)
            res = find_flat_windows(
                terrain, center, float(req.get("radius_km", 15.0)), h * scale, w * scale,
                min_fw_agl_m=req.get("min_fw_agl_m"), band_min_m=float(req.get("dh_band_min_m", 15.0)),
                clearance_m=float(req.get("clearance_m", 36.0)), ceil_m=float(req.get("ceil_m", 120.0)),
                top_n=int(req.get("top_n", 8)))
            payload = res.to_dict()
            payload["request"] = {k: v for k, v in req.items() if k != "areas"}
            payload["from_center"] = center
            payload["window_here_h_m"], payload["window_here_w_m"] = h, w
            payload["attribution"] = terrain.attribution_text
            if self._abandoned:
                return
            self.finished_ok.emit(payload)
        except Exception as e:                               # noqa: BLE001 - 一律走 failed，不讓執行緒帶著例外死掉
            kind = type(e).__name__
            # WorldTerrainError / FlatSearchError 的訊息本來就是給人看的（且不含 token）
            msg = str(e) if kind in ("WorldTerrainError", "FlatSearchError") else f"{kind}: {e}"
            logger.error(f"[找平坦位置] 失敗：{msg}")
            if not self._abandoned:
                self.failed.emit(msg)
