"""ui/workers/wind_grid_worker.py — 「抓氣象網格」背景執行緒（Open-Meteo → core.wind_field.WindField）。

要連網（約 1–3 s；逾時 20 s），不能跑在 GUI 執行緒。寫法比照 ``flat_search_worker``：QThread、只走信號、
失敗有自己的信號、``run()`` 內不碰 widget。沒有 token、沒有秘密；送出去的是作業區附近的格點經緯度。
⚠ 取消：做不到（urllib 沒有取消點），只能放棄結果；視窗關閉時由 MainWindow 等它跑完。
"""
from __future__ import annotations

from typing import Sequence

from PyQt6.QtCore import QThread, pyqtSignal

from utils.logger import get_logger

logger = get_logger()


class WindGridWorker(QThread):
    """信號：``finished_ok(WindField)``、``failed(錯誤字串)``。"""

    finished_ok = pyqtSignal(object)
    failed = pyqtSignal(str)

    def __init__(self, bbox: Sequence[float], *, height: str = "10m", nx: int = 6, ny: int = 6,
                 parent=None) -> None:
        super().__init__(parent)
        self._bbox = tuple(float(x) for x in bbox)
        self._height = str(height)
        self._nx, self._ny = int(nx), int(ny)
        self._abandoned = False

    def stop(self) -> None:
        """放棄結果（請求本身停不下來）。"""
        self._abandoned = True

    def run(self) -> None:                                   # noqa: D401 - QThread 入口
        try:
            from core import wind_field as wf
            field = wf.fetch_open_meteo(self._bbox, nx=self._nx, ny=self._ny, height=self._height)
            if self._abandoned:
                return
            self.finished_ok.emit(field)
        except Exception as e:                               # noqa: BLE001 - 一律走 failed，不讓執行緒帶著例外死掉
            kind = type(e).__name__
            msg = str(e) if kind in ("WindFetchError", "ValueError") else f"{kind}: {e}"
            logger.error(f"[氣象網格] 失敗：{msg}")
            if not self._abandoned:
                self.failed.emit(msg)


class CwaStationWorker(QThread):
    """中央氣象署自動氣象站觀測（core.wind_field.fetch_cwa_station）。
    信號：``finished_ok(StationWind)``、``failed(錯誤字串)``。⚠ 授權碼只收進來用，不放進信號、不記 log。"""

    finished_ok = pyqtSignal(object)
    failed = pyqtSignal(str)

    def __init__(self, key: str, *, station: str = "", near=None, parent=None) -> None:
        super().__init__(parent)
        self._key = str(key)
        self._station = str(station or "").strip()
        self._near = (float(near[0]), float(near[1])) if near is not None else None
        self._abandoned = False

    def stop(self) -> None:
        self._abandoned = True

    def run(self) -> None:                                   # noqa: D401 - QThread 入口
        try:
            from core import wind_field as wf
            obs = wf.fetch_cwa_station(self._key, station=self._station, near=self._near)
            if self._abandoned:
                return
            self.finished_ok.emit(obs)
        except Exception as e:                               # noqa: BLE001
            kind = type(e).__name__
            msg = str(e) if kind in ("WindFetchError", "ValueError") else f"{kind}: {e}"
            from core.wind_field import redact_cwa_url
            msg = redact_cwa_url(msg)
            logger.error(f"[CWA 測站] 失敗：{msg}")
            if not self._abandoned:
                self.failed.emit(msg)
