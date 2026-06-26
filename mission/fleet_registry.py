"""mission/fleet_registry.py — 全機隊註冊單例。

集中管理多個 `SITLLink` 與最新 `TelemetryFrame`，供：
- `FleetDashboard` 顯示多機卡片
- `ParametersBrowser` 依 sysid / callsign 取 link 發 PARAM_REQUEST_LIST
- `Indicators` 訂閱遙測

設計刻意輕量（~150 LOC），不取代 `MissionManager`。
"""
from __future__ import annotations

import logging
import time
from typing import Dict, Optional

from PyQt6.QtCore import QObject, pyqtSignal

from mission.sitl_link import SITLLink, TelemetryFrame


logger = logging.getLogger(__name__)


class FleetRegistry(QObject):
    """機隊註冊與遙測廣播中心。

    所有更新均在 Qt 主執行緒以 signal 形式廣播；
    呼叫端訂閱對應 signal 即可，不需自行去 `SITLLink` 連線。
    """

    # ── Signals ──────────────────────────────────────────────
    uav_registered    = pyqtSignal(str)                  # callsign
    uav_unregistered  = pyqtSignal(str)                  # callsign
    telemetry_updated = pyqtSignal(str, object)          # callsign, TelemetryFrame
    link_status       = pyqtSignal(str, str)             # callsign, status('connected'/'disconnected'/'lost')

    _instance: Optional["FleetRegistry"] = None

    def __init__(self, parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self._links: Dict[str, SITLLink] = {}
        self._latest: Dict[str, TelemetryFrame] = {}
        self._last_packet_ts: Dict[str, float] = {}
        # callsign ↔ sysid 對應；多數情境 callsign = "UAV-<sysid>"，
        # 但允許外部覆寫（如 HAWK-01）。
        self._sysid_index: Dict[int, str] = {}

    @classmethod
    def instance(cls) -> "FleetRegistry":
        if cls._instance is None:
            cls._instance = FleetRegistry()
        return cls._instance

    # ── Lifecycle ────────────────────────────────────────────
    def register(self, callsign: str, link: SITLLink, sysid: int) -> None:
        """註冊一個 SITLLink；同名 callsign 會覆蓋。"""
        if callsign in self._links:
            logger.info("FleetRegistry: replacing existing link for %s", callsign)
            self.unregister(callsign)
        self._links[callsign] = link
        self._sysid_index[sysid] = callsign
        # 接 SITLLink 既有 signals
        link.telemetry.connect(
            lambda frame, cs=callsign: self._on_telemetry(cs, frame)
        )
        link.connected.connect(
            lambda _conn, cs=callsign: self.link_status.emit(cs, "connected")
        )
        link.disconnected.connect(
            lambda _reason, cs=callsign: self.link_status.emit(cs, "disconnected")
        )
        self.uav_registered.emit(callsign)
        logger.info("FleetRegistry: registered %s (sysid=%d)", callsign, sysid)

    def unregister(self, callsign: str) -> None:
        link = self._links.pop(callsign, None)
        self._latest.pop(callsign, None)
        self._last_packet_ts.pop(callsign, None)
        # 移除反向索引
        for sysid, cs in list(self._sysid_index.items()):
            if cs == callsign:
                del self._sysid_index[sysid]
        if link is not None:
            self.uav_unregistered.emit(callsign)
            logger.info("FleetRegistry: unregistered %s", callsign)

    # ── Query ────────────────────────────────────────────────
    def callsigns(self) -> list[str]:
        return list(self._links.keys())

    def get_link(self, callsign: str) -> Optional[SITLLink]:
        return self._links.get(callsign)

    def get_link_by_sysid(self, sysid: int) -> Optional[SITLLink]:
        cs = self._sysid_index.get(int(sysid))
        return self._links.get(cs) if cs else None

    def latest(self, callsign: str) -> Optional[TelemetryFrame]:
        return self._latest.get(callsign)

    def latest_by_sysid(self, sysid: int) -> Optional[TelemetryFrame]:
        """以 sysid 取最新遙測（多機協調器用 sysid 為鍵）。"""
        cs = self._sysid_index.get(int(sysid))
        return self._latest.get(cs) if cs else None

    def snapshot(self) -> Dict[int, TelemetryFrame]:
        """全機最新遙測快照 {sysid: TelemetryFrame}。

        這是「各機之間的資訊互通」的共享黑板讀取點：終端同步打擊協調器每個 tick
        以此取得**全機**即時態勢，據以計算同步釋放閘 / ToT 速度修正 / 兩兩防撞間隔
        —— 每架的決策都是全機狀態的函數，而非各看各的。
        """
        out: Dict[int, TelemetryFrame] = {}
        # list(...) 快照鍵集，避免遍歷中 register/unregister 改動 dict 觸發 RuntimeError
        for sysid, cs in list(self._sysid_index.items()):
            fr = self._latest.get(cs)
            if fr is not None:
                out[int(sysid)] = fr
        return out

    def latest_age_sec(self, callsign: str) -> float:
        """最後一筆遙測距今秒數；無資料回傳 inf。"""
        ts = self._last_packet_ts.get(callsign)
        if ts is None:
            return float("inf")
        return max(0.0, time.monotonic() - ts)

    # ── 內部 ──────────────────────────────────────────────────
    def _on_telemetry(self, callsign: str, frame: TelemetryFrame) -> None:
        self._latest[callsign] = frame
        self._last_packet_ts[callsign] = time.monotonic()
        self.telemetry_updated.emit(callsign, frame)


__all__ = ["FleetRegistry"]
