"""mission/failover_monitor.py — 長機容錯監督者(GCS 端 Qt 轉接層)。

把純邏輯的 `core.swarm.consensus.RaftFailover` 接到實機遙測:
  - 以 `FleetRegistry`(共享黑板)為 liveness 來源 —— 每架最後遙測時刻 + 電量 + GPS。
  - 固定週期(QTimer)推進容錯核心,將長機更替 / 節點失聯以 Qt signal 廣播給 UI。

刻意只「讀」FleetRegistry(不改它),維持單例黑板職責單純(SRP)。
與打擊邏輯零耦合:選舉只看心跳與健康度(GPS/電量),不看任何目標或距離。

典型接線(GCS 啟動時):
    monitor = FailoverMonitor()                 # 預設用 FleetRegistry.instance()
    monitor.leader_changed.connect(ui.on_leader_changed)
    monitor.node_lost.connect(ui.on_node_lost)
    monitor.start()                             # 依 config.tick_period_s 起 QTimer
"""
from __future__ import annotations

import logging
import time
from typing import Callable, Optional, Protocol, runtime_checkable

from PyQt6.QtCore import Qt, QObject, QTimer, pyqtSignal

from core.swarm.consensus import (
    EventKind,
    FailoverConfig,
    FailoverEvent,
    NodeHealth,
    PriorityKey,
    RaftFailover,
    health_priority_key,
)

logger = logging.getLogger(__name__)

_NO_LEADER = -1  # signal 中以 -1 表示「無長機 / 舊長機不存在」(Qt signal 不傳 None)


@runtime_checkable
class LivenessSource(Protocol):
    """FailoverMonitor 對遙測來源的最小契約(FleetRegistry 與測試替身皆需滿足)。"""

    def callsigns(self) -> list[str]: ...
    def latest(self, callsign: str) -> Optional[object]: ...
    def latest_age_sec(self, callsign: str) -> float: ...


class FailoverMonitor(QObject):
    """長機容錯監督者:訂閱機隊 liveness,廣播長機更替與節點存活事件。"""

    # ── Signals(int 一律用 sysid;無長機以 -1 表示）─────────────────────────
    leader_changed = pyqtSignal(int, int, int)   # old_sysid(-1=無), new_sysid, term
    leader_lost    = pyqtSignal(int)              # old_sysid(-1=無);全機失聯,暫無長機
    node_lost      = pyqtSignal(int)              # sysid 失聯
    node_recovered = pyqtSignal(int)              # sysid 恢復
    fleet_liveness = pyqtSignal(dict)             # {sysid: alive_bool},每拍更新供 UI

    def __init__(
        self,
        registry: Optional[LivenessSource] = None,
        config: Optional[FailoverConfig] = None,
        priority_key: Optional[PriorityKey] = None,
        now_fn: Optional[Callable[[], float]] = None,
        parent: Optional[QObject] = None,
    ) -> None:
        """
        Args:
            registry:  liveness 來源(預設 FleetRegistry 單例)。須實作 LivenessSource。
            config:    容錯參數;預設 FailoverConfig()。
            priority_key: 選舉優先序;預設健康度(GPS→電量→id),不得用距目標距離。
            now_fn:    取時函式(預設 time.monotonic)。**契約**:它回傳的時間軸必須與
                       registry.latest_age_sec() 內部所用時鐘一致(FleetRegistry 用
                       time.monotonic)。注入不一致的時鐘會使「now - age」失去意義 →
                       僅在搭配同時間軸的替身 registry 時才覆寫(如單元測試)。
        """
        super().__init__(parent)
        if registry is None:
            # 延後 import,避免測試在無 SITL/pymavlink 環境硬載入。
            from mission.fleet_registry import FleetRegistry
            registry = FleetRegistry.instance()
        self._registry = registry
        # SAR 預設健康度優先序(GPS→電量→id);呼叫端可覆寫(仍不得用距目標距離)。
        self._core = RaftFailover(config=config, priority_key=priority_key or health_priority_key)
        self._now: Callable[[], float] = now_fn or time.monotonic
        self._last_tick_t: Optional[float] = None   # 上一拍時戳,用於偵測 tick 間隔過大
        self._timer = QTimer(self)
        # 用 PreciseTimer:預設 CoarseTimer 容許數十毫秒漂移/合併,會侵蝕 3 秒預算。
        self._timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._timer.timeout.connect(self._tick)

    # ── 唯讀存取 ────────────────────────────────────────────────────────────
    @property
    def core(self) -> RaftFailover:
        """底層容錯核心(供查詢 leader_id / term / 測試）。"""
        return self._core

    @property
    def leader_sysid(self) -> int:
        """目前長機 sysid;無長機回 -1。"""
        lid = self._core.leader_id
        return _NO_LEADER if lid is None else lid

    # ── 生命週期 ────────────────────────────────────────────────────────────
    def start(self, interval_ms: Optional[int] = None) -> None:
        """啟動週期性推進。預設週期取 config.tick_period_s。

        若呼叫端覆寫 interval_ms,會「重新驗證」它不致破壞 3 秒接管預算 —— 因為實際
        驅動 tick 的是 QTimer 週期而非 config.tick_period_s,放任覆寫等於繞過 config
        的安全驗證。

        Raises:
            ValueError: interval_ms 使最壞接管(逾時 + 該週期)達/超過 takeover_deadline_s。
        """
        cfg = self._core.cfg
        if interval_ms is None:
            ms = int(cfg.tick_period_s * 1000)
        else:
            worst = cfg.heartbeat_timeout_s + interval_ms / 1000.0
            if worst >= cfg.takeover_deadline_s:
                raise ValueError(
                    f"interval_ms={interval_ms} 使最壞接管 {worst:.3f}s 達/超過上界 "
                    f"{cfg.takeover_deadline_s:.3f}s;請改小 interval_ms 或放寬 config"
                )
            ms = interval_ms
        self._last_tick_t = None
        self._timer.start(max(1, ms))

    def stop(self) -> None:
        self._timer.stop()

    # ── 內部:每拍推進 ──────────────────────────────────────────────────────
    def _tick(self) -> None:
        """從 FleetRegistry 取 liveness 餵核心,推進一拍並廣播事件。"""
        now = self._now()

        # 偵測 tick 間隔過大(主執行緒被佔用):若真實間隔使最壞接管超過上界,
        # 發 warning 讓「被默默撐破的 3 秒預算」可被觀測,而非無聲違規。
        if self._last_tick_t is not None:
            dt = now - self._last_tick_t
            cfg = self._core.cfg
            if cfg.heartbeat_timeout_s + dt > cfg.takeover_deadline_s:
                logger.warning(
                    "FailoverMonitor: tick 間隔 %.3fs 過大,最壞接管 %.3fs 可能超過上界 "
                    "%.3fs(主執行緒被佔用?)",
                    dt, cfg.heartbeat_timeout_s + dt, cfg.takeover_deadline_s,
                )
        self._last_tick_t = now

        present: set[int] = set()

        # 1) 以每架最後遙測時刻 + 健康度當心跳餵入核心。
        for callsign in self._registry.callsigns():
            frame = self._registry.latest(callsign)
            if frame is None:
                continue
            sysid = int(getattr(frame, "sysid", 0))
            present.add(sysid)
            age = self._registry.latest_age_sec(callsign)   # 距今秒數(inf=無資料)
            bpct = getattr(frame, "battery_pct", -1)
            known = bpct is not None and bpct >= 0           # -1 = 未知(MAVLink 慣例)
            health = NodeHealth(
                battery_pct=float(bpct) if known else 0.0,   # 未知 → 0 且 known=False,絕不冒充滿電
                gps_ok=int(getattr(frame, "gps_fix", 0)) >= 3,
                battery_known=known,
            )
            # 心跳時戳 = now - age → 還原該筆遙測實際抵達時刻(age=inf 時自然永不新鮮)。
            self._core.heartbeat(sysid, now - age, health)

        # 2) 已從註冊表消失(解除註冊)的節點 → 從核心移除,避免幽靈長機。
        for nid in self._core.node_ids():
            if nid not in present:
                self._core.remove_node(nid)

        # 3) 推進並廣播。
        for ev in self._core.tick(now):
            self._emit_event(ev)
        self.fleet_liveness.emit(self._core.fleet_liveness())

    def _emit_event(self, ev: FailoverEvent) -> None:
        """把容錯事件翻成 Qt signal。"""
        if ev.kind is EventKind.LEADER_CHANGED:
            old = ev.old_leader if ev.old_leader is not None else _NO_LEADER
            self.leader_changed.emit(old, ev.new_leader, ev.term)
        elif ev.kind is EventKind.LEADER_LOST:
            old = ev.old_leader if ev.old_leader is not None else _NO_LEADER
            self.leader_lost.emit(old)
        elif ev.kind is EventKind.NODE_LOST:
            self.node_lost.emit(ev.node_id)
        elif ev.kind is EventKind.NODE_RECOVERED:
            self.node_recovered.emit(ev.node_id)


__all__ = ["FailoverMonitor", "LivenessSource"]
