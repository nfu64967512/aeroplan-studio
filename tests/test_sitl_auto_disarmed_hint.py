"""AUTO 在未解鎖時要給警告（2026-10-08 使用者回報「進 AUTO 顯示 vtol_takeoff 但飛機沒有起飛」的後續）。

修好 ARMING_REQUIRE 之後 VTOL 開機是**未解鎖**；只按 AUTO 會得到同一個畫面（訊息有、飛機不動）。
這裡鎖：未解鎖 + AUTO ⇒ 警告；已解鎖 / 其他模式 / 還沒有遙測 ⇒ 不吵、也不丟例外。
不連任何埠：未綁定方法 + 假 self。
"""
import types

import pytest

from mission.sitl_link import SITLLink, TelemetryFrame


class _Sig:
    def __init__(self):
        self.out = []

    def emit(self, sev, txt):
        self.out.append((sev, txt))


def _link(frame):
    mav = types.SimpleNamespace(
        mode_mapping=lambda: {"AUTO": 10, "QLOITER": 19, "QHOVER": 18},
        set_mode=lambda mid: setattr(mav, "last_mode", mid),
        last_mode=None)
    fake = types.SimpleNamespace(_mav=mav, status_text=_Sig(), sysid_label=1,
                                 _frames={} if frame is None else {1: frame})
    fake._warn_if_auto_while_disarmed = types.MethodType(
        SITLLink._warn_if_auto_while_disarmed, fake)
    return fake


def _frame(armed):
    f = TelemetryFrame()
    f.armed = armed
    return f


def test_auto_while_disarmed_warns():
    fake = _link(_frame(False))
    SITLLink._send_set_mode(fake, "auto")
    assert fake._mav.last_mode == 10, "警告不能擋掉模式切換（行為與 Mission Planner 一致）"
    sev, txt = fake.status_text.out[-1]
    assert sev == 4 and "尚未解鎖" in txt and "ARM" in txt


@pytest.mark.parametrize("armed,mode", [(True, "AUTO"), (False, "QLOITER"), (False, "QHOVER")])
def test_no_warning_when_armed_or_not_auto(armed, mode):
    fake = _link(_frame(armed))
    SITLLink._send_set_mode(fake, mode)
    assert fake.status_text.out == []


def test_no_telemetry_yet_is_quiet_and_does_not_crash():
    fake = _link(None)
    SITLLink._send_set_mode(fake, "AUTO")
    assert fake._mav.last_mode == 10 and fake.status_text.out == []
