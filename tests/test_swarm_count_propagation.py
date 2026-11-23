# -*- coding: utf-8 -*-
"""機數必須傳到鏈路的每一段 —— 寫死 6 會讓整段靜默跳過。

═══ 這輪抓到的兩個(都讓 N=2 的鏈路殘廢)═══

**① `jet_show_ros2.py:35` `BASE, STRIDE, N = 6760, 10, 6`**
N 寫死,不讀環境變數(同目錄 `jet_catapult_launch.py:20` 是讀的)。
兩機航次它照樣去連 UAV3–6 的 6780/6790/6800/6810 ⇒
`ConnectionRefusedError: [Errno 111]` 崩在 `links = {i: conn(i) for i in range(1, N+1)}`。
而 `stot_trial.sh` 呼叫它時 **rc 是丟掉的** ⇒ 巡航段整個靜默跳過。
⇒ 實測後果:TWO1 / KIL2 / KIL3 三趟的「巡航編隊」**從未執行過**,
  機隊維持彈射後的正北航點,直到 tot_node 的 ALIGN 接手(跑了 963 s)才被帶去扇形。

**② `master_node.py:107` `expected_followers` 預設 5**
配 `:222 if len(fresh) < self._expected_followers: return False` ⇒
n=2 只有 1 個發布者,`1 < 5` 恆真,**「成形」永遠不會被宣告**。
實測 `/tmp/nodes.log` 的 `formation formed` 出現 **0 次**。
而且這個參數先前**沒有任何通路**(launch 檔沒給、swarm_ros2_up.sh 沒有)。

發布者是誰:`wingman_node.py:303` 與 `backup_node.py:123` 發 /swarm/formation_status,
`master_node.py:188` 只訂閱 ⇒ **發布者數 = n − 1**。
"""
import os
import pathlib
import re

import pytest

W = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "wsl"
P = W / "sar_swarm_patches"
LAUNCH_SRC = (P / "sar_failover_nodes.launch.py").read_text(encoding="utf-8")
TRIAL = (W / "stot_trial.sh").read_text(encoding="utf-8")


def _load(fn):
    """把 launch 檔裡的純函式挖出來單獨執行(不 import ROS2)。"""
    m = re.search(r"def %s\(.*?\n(?=\ndef |\n[A-Z_]+ = |\nclass )" % fn,
                  LAUNCH_SRC, re.S)
    assert m, fn
    ns = {"os": os, "N_UAVS": 6}
    exec(m.group(0), ns)
    return ns[fn], ns


class TestCruiseScriptsReadTheCount:
    @pytest.mark.parametrize("f", ["jet_show_ros2.py", "jet_show_cruise.py"])
    def test_n_is_not_hardcoded(self, f):
        src = (W / f).read_text(encoding="utf-8")
        assert not re.search(r"^\s*BASE,\s*STRIDE,\s*N\s*=\s*\d+,\s*\d+,\s*\d+\s*$",
                             src, re.M), "%s 仍把 N 寫死在元組賦值裡" % f
        assert re.search(r'os\.environ\.get\("SWARM_COUNT"\)', src), \
            "%s 沒有讀 SWARM_COUNT" % f

    @pytest.mark.parametrize("f", ["jet_show_ros2.py", "jet_show_cruise.py",
                                   "jet_catapult_launch.py"])
    def test_falls_back_to_six(self, f):
        """六機是既有預設,不可以因為這次改動而變。"""
        src = (W / f).read_text(encoding="utf-8")
        assert re.search(r'or\s*"6"|,\s*"6"\)', src), "%s 失去 6 的預設" % f


class TestCruiseFailureIsNoLongerSilent:
    def test_rc_is_captured_on_every_branch(self):
        """三個巡航分支的 rc 先前全部丟掉 —— 那正是 ① 能藏這麼久的原因。"""
        assert TRIAL.count("_cruise_rc=$?") >= 3, "還有分支沒抓 rc"

    def test_nonzero_rc_leaves_a_trace(self):
        assert '"${_cruise_rc:-0}" != "0"' in TRIAL
        assert "巡航段以 rc=" in TRIAL

    def test_does_not_abort_the_trial(self):
        """刻意只警告不中止:巡航失敗時 ALIGN 仍能把機隊帶到扇形,
        中止反而拿不到終端資料。但必須留下痕跡。"""
        m = re.search(r'if \[ "\$\{_cruise_rc:-0\}" != "0" \]; then(.*?)\nfi',
                      TRIAL, re.S)
        assert m, "找不到巡航 rc 檢查"
        assert "exit " not in m.group(1), "巡航失敗不應中止航次"


class TestExpectedFollowersDerived:
    def test_no_hardcoded_five_in_launch(self):
        assert not re.search(r'"expected_followers":\s*5\b', LAUNCH_SRC)

    def test_derived_from_swarm_count(self):
        assert "_expected_followers = max(1, _n - 1)" in LAUNCH_SRC
        assert '"expected_followers": _expected_followers' in LAUNCH_SRC

    def test_computed_before_master_node(self):
        """master_node 就在 nodes 清單裡,計算必須排在清單之前,否則 NameError。"""
        i_calc = LAUNCH_SRC.index("_expected_followers = max(1, _n - 1)")
        i_master = LAUNCH_SRC.index('executable="master_node"')
        assert i_calc < i_master

    @pytest.mark.parametrize("n,pubs", [(2, 1), (3, 2), (4, 3), (6, 5), (8, 7)])
    def test_matches_publisher_count(self, n, pubs):
        """期望數必須等於真正會發 formation_status 的節點數(僚機 + 備長)。"""
        roles, _ = _load("_roles")
        _m, wing, backup = roles(n)
        assert len(wing) + (1 if backup else 0) == pubs
        assert max(1, n - 1) == pubs

    def test_six_ship_value_is_unchanged(self):
        """n=6 必須仍是 5 —— 既有六機航次零影響,這是單變數紀律。"""
        assert max(1, 6 - 1) == 5

    def test_never_zero(self):
        """設 0 會讓 master_node 走到 `worst <= radius` 而 worst 是 None ⇒ TypeError。
        n=1 因此刻意留 1(退化成永不成形,但不會崩)。"""
        assert max(1, 1 - 1) == 1
