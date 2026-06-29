"""FailoverMonitor(Qt 轉接層)單元測試 —— 用假註冊表 + 假時鐘確定性驗證。

不啟動 QTimer,直接驅動 `_tick()`,故無需事件迴圈;以一個 QApplication
fixture 滿足 QObject/pyqtSignal 建構需求。

執行:pytest tests/test_failover_monitor.py -v
"""
from types import SimpleNamespace

import pytest

from mission.failover_monitor import FailoverMonitor


# ═══════════════════════════════════════════════════════════════════════════
#  測試替身
# ═══════════════════════════════════════════════════════════════════════════
class FakeRegistry:
    """鴨子型別替身,僅實作 FailoverMonitor 用到的三個方法。"""

    def __init__(self):
        self._frames = {}   # callsign -> TelemetryFrame 替身
        self._age = {}      # callsign -> 距今秒數

    def set(self, callsign, sysid, age, battery_pct=100, gps_fix=3):
        self._frames[callsign] = SimpleNamespace(
            sysid=sysid, battery_pct=battery_pct, gps_fix=gps_fix)
        self._age[callsign] = age

    def drop(self, callsign):
        self._frames.pop(callsign, None)
        self._age.pop(callsign, None)

    # FailoverMonitor 介面 ----------------------------------------------------
    def callsigns(self):
        return list(self._frames.keys())

    def latest(self, callsign):
        return self._frames.get(callsign)

    def latest_age_sec(self, callsign):
        return self._age.get(callsign, float("inf"))


@pytest.fixture(scope="module", autouse=True)
def _qapp():
    """QObject/pyqtSignal 建構需要一個 QApplication 實例。"""
    from PyQt6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    yield app


@pytest.fixture
def setup():
    reg = FakeRegistry()
    clock = {"t": 0.0}
    mon = FailoverMonitor(registry=reg, now_fn=lambda: clock["t"])
    captured = {"leader": [], "lost": [], "recovered": [], "leader_lost": []}
    mon.leader_changed.connect(lambda o, n, term: captured["leader"].append((o, n, term)))
    mon.node_lost.connect(lambda s: captured["lost"].append(s))
    mon.node_recovered.connect(lambda s: captured["recovered"].append(s))
    mon.leader_lost.connect(lambda o: captured["leader_lost"].append(o))
    return reg, clock, mon, captured


# ═══════════════════════════════════════════════════════════════════════════
#  測試
# ═══════════════════════════════════════════════════════════════════════════
def test_monitor_initial_leader_health_based(setup):
    """三機上線,電量最高者(2)當選 —— 監督者預設健康度優先序。"""
    reg, clock, mon, cap = setup
    reg.set("UAV-1", 1, age=0.0, battery_pct=50)
    reg.set("UAV-2", 2, age=0.0, battery_pct=95)
    reg.set("UAV-3", 3, age=0.0, battery_pct=70)
    mon._tick()
    assert mon.core.leader_id == 2
    assert mon.leader_sysid == 2
    assert cap["leader"] == [(-1, 2, 1)]   # old=-1(無), new=2, term=1


def test_monitor_failover_on_stale_telemetry(setup):
    """長機遙測停更(age 變大)→ 監督者觸發接管並發 leader_changed。"""
    reg, clock, mon, cap = setup
    reg.set("UAV-1", 1, age=0.0, battery_pct=95)   # 1 電量最高 → 先當選
    reg.set("UAV-2", 2, age=0.0, battery_pct=90)
    mon._tick()
    assert mon.core.leader_id == 1

    # 1 遙測停更(age=5s),2 持續新鮮;時鐘前進。
    clock["t"] = 1.2
    reg.set("UAV-1", 1, age=5.0, battery_pct=95)
    reg.set("UAV-2", 2, age=0.0, battery_pct=90)
    mon._tick()
    assert mon.core.leader_id == 2
    assert (1, 2, 2) in cap["leader"]
    assert 1 in cap["lost"]


def test_monitor_unregister_removes_node(setup):
    """從註冊表消失的節點 → 監督者把它移出核心(不留幽靈)。"""
    reg, clock, mon, cap = setup
    reg.set("UAV-1", 1, age=0.0)
    reg.set("UAV-2", 2, age=0.0)
    mon._tick()
    assert set(mon.core.node_ids()) == {1, 2}

    reg.drop("UAV-2")
    mon._tick()
    assert set(mon.core.node_ids()) == {1}


def test_monitor_emits_fleet_liveness(setup):
    """每拍廣播 {sysid: alive} 供 UI 更新健康狀態。"""
    reg, clock, mon, cap = setup
    snapshots = []
    mon.fleet_liveness.connect(lambda d: snapshots.append(d))
    reg.set("UAV-1", 1, age=0.0)
    reg.set("UAV-2", 2, age=10.0)   # 已逾時
    mon._tick()
    assert snapshots[-1] == {1: True, 2: False}


def test_monitor_leader_lost_on_all_stale(setup):
    """全機遙測停更 → 監督者發 leader_lost(舊長機 sysid)、leader_sysid 回 -1。"""
    reg, clock, mon, cap = setup
    reg.set("UAV-1", 1, age=0.0)
    reg.set("UAV-2", 2, age=0.0)
    mon._tick()
    leader0 = mon.core.leader_id          # 等電量等 GPS → 最低 id = 1
    assert leader0 == 1

    clock["t"] = 2.0
    reg.set("UAV-1", 1, age=10.0)
    reg.set("UAV-2", 2, age=10.0)
    mon._tick()
    assert mon.leader_sysid == -1
    assert cap["leader_lost"] == [leader0]


def test_monitor_fleet_liveness_transitions(setup):
    """節點在正確的一拍由 True 翻成 False(跨拍轉變,非出生即死)。"""
    reg, clock, mon, cap = setup
    snaps = []
    mon.fleet_liveness.connect(lambda d: snaps.append(dict(d)))
    reg.set("UAV-1", 1, age=0.0)
    reg.set("UAV-2", 2, age=0.0)
    mon._tick()
    assert snaps[-1] == {1: True, 2: True}

    clock["t"] = 1.2
    reg.set("UAV-1", 1, age=0.0)
    reg.set("UAV-2", 2, age=5.0)          # 2 停更 → 該拍翻為 False
    mon._tick()
    assert snaps[-1] == {1: True, 2: False}


def test_monitor_start_rejects_budget_breaking_interval(setup):
    """start(interval_ms) 若使最壞接管達/超過上界 → 直接 raise;合法值放行。"""
    reg, clock, mon, cap = setup
    with pytest.raises(ValueError):
        mon.start(interval_ms=2500)       # 1.0 + 2.5 = 3.5 >= 3.0
    mon.start(interval_ms=500)            # 1.0 + 0.5 = 1.5 < 3.0 → 放行
    mon.stop()


def test_monitor_unknown_battery_not_elected(setup):
    """電量未知(-1)的機不得被當滿電優先;已知健康電量者(雖 id 較大)當選。"""
    reg, clock, mon, cap = setup
    reg.set("UAV-1", 1, age=0.0, battery_pct=-1, gps_fix=3)   # 未知電量、較小 id
    reg.set("UAV-2", 2, age=0.0, battery_pct=60, gps_fix=3)   # 已知 60%
    mon._tick()
    assert mon.core.leader_id == 2
