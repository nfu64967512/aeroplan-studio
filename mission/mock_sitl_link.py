"""
MockSITLLink — 不需 ArduPilot 即可測試 MAVLink 任務上傳的替身

用途
----
* CI/CD 無 pymavlink / 無網路環境下跑 unit test
* 開發蜂群打擊邏輯時不用真的啟 SITL
* 測試 STOT/DTOT/VTOL 各種上傳流程

與真 SITLLink 相容的介面：
    upload_mission(waypoints)       記錄上傳次數與內容
    connect() / disconnect() / stop()
    wait(timeout)
    send_heartbeat() / cmd_long(...)
    last_frame                      (TelemetryFrame 仿造)

用法::

    from mission.mock_sitl_link import MockSITLLink

    link = MockSITLLink(sysid_label='UAV-1')
    link.connect()   # no-op
    link.upload_mission([(25.0, 121.5, 100.0, 16, 0, 150, 0, 0), ...])
    assert link.upload_count == 1
    assert len(link.last_uploaded_waypoints) == 1
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Any


@dataclass
class _MockTelemetryFrame:
    """最小化 TelemetryFrame 替身，提供 MainWindow 讀取即時遙測用"""
    lat: Optional[float] = 25.0
    lon: Optional[float] = 121.5
    alt: float = 100.0
    heading_deg: float = 0.0
    groundspeed_mps: float = 15.0
    airspeed_mps: float = 15.0
    armed: bool = False
    mode: str = 'STABILIZE'
    battery_remaining: float = 100.0
    gps_fix_type: int = 3          # 3D Fix
    satellites_visible: int = 12


class MockSITLLink:
    """SITLLink 的 test double

    故意不繼承 QThread；純同步介面避免測試中的 Qt 事件循環依賴。

    Attributes
    ----------
    upload_count : int
        上傳 mission 的次數
    last_uploaded_waypoints : List[Tuple]
        最近一次上傳的 8-tuple 航點
    all_uploaded_missions : List[List]
        歷次上傳的完整記錄 (供回歸測試)
    cmd_log : List[str]
        所有接收過的指令 (arm / disarm / set_mode / takeoff ...)
    """

    def __init__(self,
                 connection_str: str = 'mock://localhost',
                 sysid_label: str = 'MOCK-1',
                 **kwargs):
        self.connection_str = connection_str
        self.sysid_label = sysid_label
        self._connected = False

        # 測試用觀察欄位
        self.upload_count: int = 0
        self.last_uploaded_waypoints: List[Tuple] = []
        self.all_uploaded_missions: List[List[Tuple]] = []
        self.cmd_log: List[str] = []

        # 模擬遙測
        self.last_frame = _MockTelemetryFrame()

        # 故意模擬的錯誤模式 (供 fault-injection 測試)
        self.simulate_upload_failure: bool = False
        self.simulate_timeout: bool = False

    # ─── 連線生命週期 ─────────────────────────────────
    def connect(self) -> bool:
        if self.simulate_timeout:
            return False
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False

    def stop(self) -> None:
        self.disconnect()

    def wait(self, timeout_ms: int = 2000) -> bool:
        return True

    def is_connected(self) -> bool:
        return self._connected

    # ─── MAVLink 任務 API ─────────────────────────────
    def upload_mission(self,
                       waypoints: List[Tuple[float, ...]]) -> bool:
        """記錄上傳內容 (不實際發 MAVLink)"""
        if self.simulate_upload_failure:
            raise RuntimeError('[Mock] 模擬上傳失敗')
        self.upload_count += 1
        wp_list = list(waypoints)
        self.last_uploaded_waypoints = wp_list
        self.all_uploaded_missions.append(wp_list)
        return True

    def cmd_long(self, cmd_id: int, *params) -> None:
        self.cmd_log.append(f'CMD_LONG(id={cmd_id}, params={params})')

    def send_heartbeat(self) -> None:
        self.cmd_log.append('HEARTBEAT')

    def set_mode(self, mode: str) -> None:
        self.cmd_log.append(f'SET_MODE({mode})')
        self.last_frame.mode = mode

    def arm(self) -> None:
        self.cmd_log.append('ARM')
        self.last_frame.armed = True

    def disarm(self) -> None:
        self.cmd_log.append('DISARM')
        self.last_frame.armed = False

    def takeoff(self, alt: float) -> None:
        self.cmd_log.append(f'TAKEOFF({alt}m)')

    # ─── Test 輔助 ─────────────────────────────────────
    def reset(self) -> None:
        """清空所有觀察記錄，重啟測試"""
        self.upload_count = 0
        self.last_uploaded_waypoints = []
        self.all_uploaded_missions = []
        self.cmd_log = []

    def summary(self) -> str:
        return (f'MockSITLLink({self.sysid_label}): '
                f'uploads={self.upload_count}, '
                f'cmds={len(self.cmd_log)}, '
                f'last_wp={len(self.last_uploaded_waypoints)}')


# ═══════════════════════════════════════════════════════════════════════
#  Fleet 輔助：一次建立 N 台 Mock
# ═══════════════════════════════════════════════════════════════════════

def make_mock_fleet(n: int) -> List[MockSITLLink]:
    """建立 N 台 MockSITLLink，sysid 從 1 編號"""
    return [
        MockSITLLink(
            connection_str=f'mock://127.0.0.1:{5760 + i * 10}',
            sysid_label=f'MOCK-{i + 1}',
        )
        for i in range(n)
    ]


if __name__ == '__main__':
    # 快速冒煙測試
    fleet = make_mock_fleet(3)
    for link in fleet:
        link.connect()
        link.upload_mission([
            (25.0, 121.5, 100.0, 16, 0, 150, 0, 0),
            (25.01, 121.51, 100.0, 16, 0, 150, 0, 0),
        ])
    for link in fleet:
        print(link.summary())
