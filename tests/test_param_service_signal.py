"""ParamService 與 SITLLink.param_value 的參數對齊。

SITLLink.param_value 是 (來源 sysid, name, value, type, index, count) 六個參數。
ParamService 的 lambda 若照舊版五參數寫法接，第一個參數會變成 name，
_sys 預設值被第六個參數蓋掉，第二筆 PARAM_VALUE 就在槽裡丟 TypeError。
"""
from PyQt6.QtCore import QObject, pyqtSignal

from mission import param_service as ps_mod
from mission.param_service import ParamService


class _FakeMav:
    class mav:
        sent = []

        @classmethod
        def param_request_list_send(cls, *a):
            cls.sent.append(a)

    target_system = 7
    target_component = 1


class _FakeLink(QObject):
    param_value = pyqtSignal(int, str, float, int, int, int)

    def __init__(self):
        super().__init__()
        self._mav = _FakeMav()


class _FakeRegistry:
    def __init__(self, link):
        self._link = link

    def get_link_by_sysid(self, sysid):
        return self._link


def test_param_value_six_args_reach_records(monkeypatch):
    link = _FakeLink()
    monkeypatch.setattr(ps_mod.FleetRegistry, "instance", classmethod(lambda cls: _FakeRegistry(link)))
    svc = ParamService()
    done = []
    acks = []
    svc.param_list_complete.connect(lambda sysid, recs: done.append((sysid, recs)))
    svc.param_set_ack.connect(lambda sysid, name, ok: acks.append((sysid, name, ok)))

    assert svc.request_all(3) is True
    # 來源 sysid 故意與登錄的 3 不同：記錄仍要歸在登錄的 sysid 底下
    link.param_value.emit(42, "CAM1_TYPE", 2.0, 2, 0, 2)
    link.param_value.emit(42, "CAM1_DURATION", 0.1, 9, 1, 2)

    assert len(done) == 1
    sysid, recs = done[0]
    assert sysid == 3
    assert recs["CAM1_TYPE"].value == 2.0 and recs["CAM1_TYPE"].type_code == 2 and recs["CAM1_TYPE"].index == 0
    assert abs(recs["CAM1_DURATION"].value - 0.1) < 1e-6 and recs["CAM1_DURATION"].type_code == 9
    assert recs["CAM1_DURATION"].index == 1

    # 再收到不同值 ⇒ 視為寫入成功的 ACK，sysid 仍是登錄值
    link.param_value.emit(42, "CAM1_TYPE", 1.0, 2, 0, 2)
    assert acks == [(3, "CAM1_TYPE", True)]
