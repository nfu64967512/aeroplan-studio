# -*- coding: utf-8 -*-
"""同一個 MAVLink 指令**一次只准送一條路** —— 雙路送會殺死 MAVROS。

═══ 2026-08-07 事故 ═══
`change_speed_async` 同時走兩條路把 **同一個 cmd 178** 送給 **同一台飛控**:

    direct  : self._wp.mav.command_long_send(... DO_CHANGE_SPEED ...)
    MAVROS  : self.cli_cmd.call_async(CommandLong{command=178})

飛控對每一發各回一個 COMMAND_ACK,MAVProxy 把 ACK 扇出到所有輸出
⇒ MAVROS 的 command plugin 收到兩個 ACK,對同一個 promise `set_value` 兩次:

    [uav1.mavros.cmd]: CMD: Unexpected command 178, result 4
    terminate called after throwing an instance of 'std::future_error'
      what():  std::future_error: Promise already satisfied
    process has died [exit code -6, __ns:=/uav1/mavros]

⚠️ 後果不只是少一個節點:master_role 會抱著最後一筆定位以 10 Hz 繼續廣播
   (speed/omega 還在變,看起來完全正常)⇒ 僚機飛向永遠不動的 slot,
   ANM1 那趟飛到 68 km 外而全程 WARN/ERROR 計數 = 0。
   —— 見 tests/test_leader_state_freeze.py。那一輪只補了下游防護,
   **根因(雙路送)留到這一輪才修**,於是 2026-08-07 又復發一次,
   兩台 MAVROS 同時 SIGABRT,swarm_takeoff.sh 直接 arm 不起來。
"""
import pathlib
import re

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC = (ROOT / "scripts" / "wsl" / "sar_swarm_patches"
       / "mavros_interface.py").read_text(encoding="utf-8")


def _body(name):
    m = re.search(r"    def %s\(self.*?\n(?=    def )" % re.escape(name), SRC, re.S)
    assert m, "找不到 %s" % name
    return m.group(0)


class TestSpeedTakesExactlyOnePath:
    def test_direct_path_does_not_also_call_mavros(self):
        """⚠️ 核心不變式:走直連時,同一次呼叫裡不可以再有 MAVROS 送出。"""
        b = _body("change_speed_async")
        head = b[:b.index("# SPEED_DIRECT=0")]
        assert "call_async" not in head, \
            "直連分支裡仍有 MAVROS 送出 —— 會製造雙 ACK"

    def test_direct_branch_returns_early(self):
        b = _body("change_speed_async")
        assert re.search(r"if self\._speed_direct:.*?\n            return None", b, re.S)

    def test_mavros_branch_still_exists_for_ab(self):
        """A/B 對照組要留著:SPEED_DIRECT=0 時只走 MAVROS,同樣只有一發。"""
        b = _body("change_speed_async")
        assert "cli_cmd.call_async(req)" in b
        assert "MAV_CMD_DO_CHANGE_SPEED" in b

    def test_only_one_send_per_call_for_both_settings(self):
        """逐分支數送出次數:兩種設定都必須恰好 1。"""
        b = _body("change_speed_async")
        i = b.index("# SPEED_DIRECT=0")
        direct, mavros = b[:i], b[i:]
        assert direct.count("_change_speed_direct(") == 1
        assert direct.count("call_async") == 0
        assert mavros.count("_change_speed_direct(") == 0
        assert mavros.count("call_async") == 1

    def test_observability_kept_without_a_second_command(self):
        """⚠️ 不可以為了「檢驗回應」而再送一發真的指令 —— 那正是病灶。"""
        assert "def _speed_report(self" in SRC
        b = _body("_speed_report")
        # ⚠️ 只看**程式碼**:docstring 裡本來就會提到 command_long_send
        #    (那是在解釋為什麼直連沒有回應可等),不該被算成一次送出。
        code = b[b.index('"""', b.index('"""') + 3) + 3:]
        assert "call_async" not in code
        assert "command_long_send" not in code
        assert "_speed_ok_n" in code and "_speed_warn" in code

    def test_report_is_honest_about_what_it_knows(self):
        """直連沒有回應可等,不可以假裝知道飛控收不收。"""
        b = _body("_speed_report")
        assert "沒有回應可以等" in b or "沒有回應" in b


class TestTheIncidentIsDocumented:
    @pytest.mark.parametrize("needle", [
        "Promise already satisfied",
        "exit code -6",
        "COMMAND_ACK",
    ])
    def test_root_cause_written_down(self, needle):
        """下一個人要能從程式碼直接讀到「為什麼不能兩條都送」。"""
        assert needle in SRC

    def test_warns_against_reintroducing_it(self):
        b = _body("change_speed_async")
        assert "一次只准走一條路" in b
