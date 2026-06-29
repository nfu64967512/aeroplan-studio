"""mission.sitl_link — 多來源遙測分流 (per-source-system demux) 單元測試

驗證 SITLLink._handle_message 會依 MAVLink source system 把遙測拆到各自的
TelemetryFrame 並各別 emit，使得「一條連線背後 mavproxy fan-in 多架」時，
HUD 仍能分出多張正確卡片（而非全部疊在 sysid_label 那一張、互相覆蓋）。

不啟動 QThread：直接以假訊息餵 _handle_message，並擷取 telemetry 信號。
只用到 QtCore（QCoreApplication），無需顯示環境。
"""
from __future__ import annotations

import pytest
from PyQt6.QtCore import QCoreApplication

from mission.sitl_link import SITLLink

# QThread/pyqtSignal 需要一個 application 物件；同程序共用單例即可。
_app = QCoreApplication.instance() or QCoreApplication([])

# MAVLink 常數（避免 import mavutil 增加相依）
_MAV_TYPE_FIXED_WING = 1
_PLANE_AUTO = 10        # _PLANE_MODES[10] == 'AUTO'
_ARMED_BIT = 128        # MAV_MODE_FLAG_SAFETY_ARMED


class FakeMsg:
    """最小 MAVLink 訊息替身：提供 get_type()/get_srcSystem() 與欄位屬性。"""

    def __init__(self, mtype: str, src: int, **fields):
        self._mtype = mtype
        self._src = src
        for k, v in fields.items():
            setattr(self, k, v)

    def get_type(self) -> str:
        return self._mtype

    def get_srcSystem(self) -> int:
        return self._src


def _heartbeat(src: int, armed: bool = True, custom_mode: int = _PLANE_AUTO) -> FakeMsg:
    return FakeMsg(
        'HEARTBEAT', src,
        type=_MAV_TYPE_FIXED_WING,
        base_mode=_ARMED_BIT if armed else 0,
        custom_mode=custom_mode,
        system_status=4,  # ACTIVE
    )


def _gpos(src: int, lat: float, lon: float) -> FakeMsg:
    return FakeMsg(
        'GLOBAL_POSITION_INT', src,
        lat=int(lat * 1e7), lon=int(lon * 1e7),
        alt=100_000, relative_alt=50_000,
        hdg=9000,  # 90.00 deg
        vx=1000, vy=0, vz=0,  # 10 m/s north
    )


def _make_link(sysid_label: int = 21) -> SITLLink:
    """建立一條未啟動的 SITLLink 並接好 telemetry 收集器。

    回傳 (link, captured)；captured 為 list[dict]，每筆是一次 emit 的快照。
    """
    link = SITLLink(conn_str='udpin:0.0.0.0:14550', sysid_label=sysid_label)
    captured: list = []
    # 在 slot 內拍快照（直接連線 → emit 當下同步呼叫，欄位值即時正確）
    link.telemetry.connect(lambda f: captured.append(
        {'sysid': f.sysid, 'lat': round(f.lat, 5), 'lon': round(f.lon, 5),
         'mode': f.mode, 'vehicle_type': f.vehicle_type}
    ))
    link._captured = captured  # 方便測試取用
    return link


class TestPerSourceDemux:
    def test_two_sysids_on_one_link_demux(self):
        """同一條連線收到兩個來源 system → 拆成兩個 frame、資料不互染。"""
        link = _make_link(sysid_label=21)

        # 來源 21
        link._handle_message(_heartbeat(21))
        link._handle_message(_gpos(21, 24.1, 121.1))
        # 來源 22（mavproxy fan-in 到同一 --out 的另一架）
        link._handle_message(_heartbeat(22))
        link._handle_message(_gpos(22, 24.2, 121.2))

        # 兩個獨立 frame
        assert set(link._frames.keys()) == {21, 22}
        f21 = link._frames[21]
        f22 = link._frames[22]
        assert f21.sysid == 21 and f22.sysid == 22
        # 位置未互染
        assert round(f21.lat, 5) == 24.1 and round(f21.lon, 5) == 121.1
        assert round(f22.lat, 5) == 24.2 and round(f22.lon, 5) == 121.2

        # emit 過的快照同時涵蓋 21 與 22，且各自帶正確座標
        snaps_21 = [s for s in link._captured if s['sysid'] == 21]
        snaps_22 = [s for s in link._captured if s['sysid'] == 22]
        assert snaps_21 and snaps_22
        assert snaps_21[-1]['lat'] == 24.1 and snaps_21[-1]['lon'] == 121.1
        assert snaps_22[-1]['lat'] == 24.2 and snaps_22[-1]['lon'] == 121.2
        # 模式/類型有解出來（非預設）
        assert snaps_21[-1]['mode'] == 'AUTO'
        assert 'PLANE' in snaps_21[-1]['vehicle_type']

    def test_gcs_echo_and_invalid_source_filtered(self):
        """來源 255（我方 GCS 回送）與 0（無效）不得產生 frame/卡片。"""
        link = _make_link(sysid_label=21)
        link._handle_message(_heartbeat(255))         # 我方 GCS 回送
        link._handle_message(_gpos(255, 9.9, 9.9))
        link._handle_message(_heartbeat(0))           # 無效來源
        assert link._frames == {}
        assert link._captured == []

    def test_single_source_regression(self):
        """單一來源（一條連線一架）行為不變：恰一個 frame、sysid 正確。"""
        link = _make_link(sysid_label=1)
        link._handle_message(_heartbeat(1))
        link._handle_message(_gpos(1, 25.0, 121.5))
        assert list(link._frames.keys()) == [1]
        assert link._frames[1].sysid == 1
        assert round(link._frames[1].lat, 5) == 25.0
        # 至少 emit 過一次且 sysid==1
        assert link._captured and all(s['sysid'] == 1 for s in link._captured)

    def test_param_and_statustext_are_source_agnostic(self):
        """PARAM_VALUE / STATUSTEXT 走連線層級信號，不建立 frame。"""
        link = _make_link(sysid_label=21)
        statuses: list = []
        servo: list = []
        link.status_text.connect(lambda sev, txt: statuses.append((sev, txt)))
        link.servo_param.connect(lambda ch, key, val: servo.append((ch, key, val)))

        link._handle_message(FakeMsg(
            'PARAM_VALUE', 21, param_id='SERVO5_FUNCTION', param_value=33.0))
        link._handle_message(FakeMsg(
            'STATUSTEXT', 21, severity=6, text='hello'))

        # 不應因 PARAM/STATUSTEXT 而生出 frame
        assert link._frames == {}
        assert servo == [(5, 'FUNCTION', 33.0)]
        assert ('hello' in t for _, t in statuses)
