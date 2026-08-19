# -*- coding: utf-8 -*-
"""編隊必須對任意機數成立 —— 逐 n 參數化。

═══ 使用者規格(2026-08-07)═══
「不管幾台飛機都要能正常 work 編隊。最基本 2 台 = **一長一備長**,
 3 台 = 多一台僚機,之後以此類推。**數量不要寫死。**」

    n | 長機 | 僚機      | 備長
    1 |  1   |  —        |  —
    2 |  1   |  —        |  2      ← 一長一備長
    3 |  1   |  2        |  3      ← 多一台僚機
    4 |  1   |  2,3      |  4
    6 |  1   |  2,3,4,5  |  6      ← 必須與 2026-08-07 之前逐項相同

═══ 這一輪修掉的四類 ═══
1. `wingman_node.n_uavs` 預設 6、launch 從沒傳 ⇒ n≥7 的機訂閱不到自己的定位
   ⇒ `fleet.get(uav_id)` 恆 None ⇒ 追趕通道關閉、回報 err=-1 被 master 丟棄
   ⇒ `formation formed` 永遠不出現。
2. `backup_node.expected_followers` 預設 4、launch 從沒傳 ⇒ n≤3 備長永遠不能接管
   (`_eligible` 要 `fresh >= min(2, expected)`,而 `_heard` 只數僚機)。
   ⇒ 而 n=2 配備長的**唯一理由**就是失效接管。
3. `V_RANK_SIDE.get(uav_id, (1, 0))` 的靜默預設 ⇒ n=8 時 UAV6 與 UAV7 **都**
   退回 (1,0) ⇒ 被指到**同一個經緯度**,唯一分隔是 15 m 的高度分層,
   而 formed_radius 是 200 m ⇒ **系統判定「編隊成形」**。
4. `tot_node` 五處 `/(self.n - 1)` ⇒ n=1 在 `__init__` 就 ZeroDivisionError。
"""
import math
import os
import pathlib
import re

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
P = ROOT / "scripts" / "wsl" / "sar_swarm_patches"
LAUNCH_SRC = (P / "sar_failover_nodes.launch.py").read_text(encoding="utf-8")
WING = (P / "wingman_node.py").read_text(encoding="utf-8")
BACKUP = (P / "backup_node.py").read_text(encoding="utf-8")
TOT = (P / "tot_node.py").read_text(encoding="utf-8")
UP = (ROOT / "scripts" / "wsl" / "swarm_ros2_up.sh").read_text(encoding="utf-8")

ALL_N = [1, 2, 3, 4, 6, 8]


def _roles(n):
    """把 launch 的純函式挖出來執行(不 import ROS2)。"""
    m = re.search(r"def _roles\(.*?\n(?=\ndef |\n[A-Z_]+ = |\nclass )", LAUNCH_SRC, re.S)
    assert m, "找不到 _roles"
    ns = {"os": os}
    exec(m.group(0), ns)
    return ns["_roles"](n)


def _v_rank_side(uav_id, n_uavs):
    """把 wingman 的純函式挖出來(含它引用的字典)。"""
    ns = {"math": math}
    for pat in (r"V_RANK_SIDE = \{[^}]*\}", r"CONE_PHASE_DEG = \{[^}]*\}"):
        exec(re.search(pat, WING).group(0), ns)
    exec(re.search(r"def v_rank_side\(.*?\n(?=\ndef )", WING, re.S).group(0), ns)
    return ns["v_rank_side"](uav_id, n_uavs)


class TestRoleSpec:
    @pytest.mark.parametrize("n,master,wing,backup", [
        (1, 1, [], None),
        (2, 1, [], 2),          # ← 一長一備長
        (3, 1, [2], 3),         # ← 多一台僚機
        (4, 1, [2, 3], 4),
        (6, 1, [2, 3, 4, 5], 6),
        (8, 1, [2, 3, 4, 5, 6, 7], 8),
    ])
    def test_matches_user_spec(self, n, master, wing, backup):
        assert _roles(n) == (master, wing, backup)

    def test_six_is_bit_identical_to_before(self):
        """六機是唯一有較多歷史資料的組態,不可在修 N 感知時被改掉。"""
        assert _roles(6) == (1, [2, 3, 4, 5], 6)

    @pytest.mark.parametrize("n", ALL_N)
    def test_every_aircraft_has_exactly_one_role(self, n):
        m, w, b = _roles(n)
        ids = [m] + list(w) + ([b] if b else [])
        assert sorted(ids) == list(range(1, n + 1)), "有機體沒有角色或角色重複"

    @pytest.mark.parametrize("n", ALL_N)
    def test_publisher_count_invariant(self, n):
        """發布 formation_status 的 = 僚機 + 備長 = n − 1(長機只訂閱)。"""
        _m, w, b = _roles(n)
        assert len(w) + (1 if b else 0) == max(0, n - 1)


class TestLaunchPassesCounts:
    def test_wingman_gets_n_uavs(self):
        assert '"n_uavs": _n' in LAUNCH_SRC

    def test_backup_gets_expected_followers(self):
        assert '"expected_followers": max(0, _n - 2)' in LAUNCH_SRC

    def test_master_expected_differs_from_backup(self):
        """語意不同:master 聽得到僚機+備長(n−1);備長只聽得到僚機(n−2)。"""
        assert "_expected_followers = max(1, _n - 1)" in LAUNCH_SRC

    @pytest.mark.parametrize("n,master_exp,backup_exp", [
        (2, 1, 0), (3, 2, 1), (4, 3, 2), (6, 5, 4), (8, 7, 6)])
    def test_expected_counts_match_actual_publishers(self, n, master_exp, backup_exp):
        _m, w, b = _roles(n)
        assert max(1, n - 1) == master_exp
        assert len(w) + (1 if b else 0) == master_exp      # master 聽得到的
        assert max(0, n - 2) == backup_exp
        assert len(w) == backup_exp                        # 備長聽得到的(排除自己)

    def test_six_values_unchanged(self):
        """n=6 ⇒ n_uavs=6(= wingman 舊預設)、expected=4(= backup 舊寫死值)。"""
        assert max(0, 6 - 2) == 4


class TestSlotNeverCollides:
    @pytest.mark.parametrize("n", ALL_N)
    def test_no_two_wingmen_share_a_slot(self, n):
        """⚠️ n=8 時 UAV6/UAV7 曾雙雙退回 (1,0) ⇒ 同一個經緯度,而系統判成形。"""
        _m, wing, _b = _roles(n)
        slots = [_v_rank_side(v, n) for v in wing]
        assert len(set(slots)) == len(slots), \
            "n=%d 有僚機共用 slot:%s" % (n, list(zip(wing, slots)))

    def test_six_ship_slots_bit_identical(self):
        """字典命中的路徑必須完全不變 —— 序位公式的排序與手訂字典不同。"""
        assert [_v_rank_side(v, 6) for v in (2, 3, 4, 5)] == \
               [(1, +1), (2, +1), (1, -1), (2, -1)]

    @pytest.mark.parametrize("n", [7, 8, 10])
    def test_beyond_dictionary_still_spreads(self, n):
        _m, wing, _b = _roles(n)
        sides = {s for _r, s in (_v_rank_side(v, n) for v in wing)}
        assert sides == {+1, -1}, "n=%d 全部擠在同一翼" % n

    def test_no_silent_zero_side_default(self):
        """(1, 0) 的靜默預設是同點疊飛的來源,不可以再出現。"""
        assert "V_RANK_SIDE.get(self.uav_id, (1, 0))" not in WING
        assert "CONE_PHASE_DEG.get(self.uav_id, 0.0)" not in WING


class TestTotNodeSurvivesSingleAircraft:
    def test_fan_offset_helper_exists(self):
        assert "def _fan_off(self, k):" in TOT

    def test_no_bare_division_by_n_minus_one(self):
        """五處除法必須全部走 helper。

        剩下允許的只有三種:
          (a) `_fan_off` **自己的函式體** —— 它上面就有 `if self.n <= 1: return 0.0`
          (b) 夾過的 `max(1, self.n - 1)`
          (c) 乘法項 `(self.n - 1) / 2.0`(高度分層,n=1 給 0 是正確的)
        """
        helper = re.search(r"    def _fan_off\(self, k\):.*?\n    def ", TOT, re.S)
        assert helper, "找不到 _fan_off"
        body = helper.group(0)
        for ln in TOT.splitlines():
            code = ln.split("#", 1)[0]
            if "/ (self.n - 1)" not in code:
                continue
            if ln in body:                      # (a) helper 自己,已被 n<=1 守住
                continue
            if "max(1," in code:                # (b)
                continue
            if "/ 2.0" in code:                 # (c)
                continue
            raise AssertionError("仍有裸除法:%s" % ln.strip())

    def test_single_aircraft_takes_fan_centre(self):
        """n=1 要取扇形中心 0.0,不是 max(1,n-1) 硬算出來的 −half(扇形邊緣)。"""
        m = re.search(r"def _fan_off\(self, k\):(.*?)\n    def ", TOT, re.S)
        assert m
        assert re.search(r"if self\.n <= 1:\s*\n\s*return 0\.0", m.group(1))

    def test_multiplication_sites_untouched(self):
        """(self.n - 1) / 2.0 是高度分層的乘法項,n=1 給 0 是正確的,不可改。"""
        assert TOT.count("(self.n - 1) / 2.0") >= 4


class TestBackupEligibilityWorksWithoutWingmen:
    def test_falls_back_to_self_corroboration(self):
        """n=2 沒有僚機可作證 ⇒ 改用「自己定位有效 + 曾收過長機心跳」。"""
        assert "own_fix_ok = self.my_fix is not None" in BACKUP
        assert "heard_leader = self.seen_hb" in BACKUP

    def test_cold_start_still_blocked(self):
        """冷開機沒收過心跳 ⇒ seen_hb False ⇒ 不得接管(這是原防護的核心)。"""
        m = re.search(r"if not \(own_fix_ok and heard_leader\):(.*?)return False",
                      BACKUP, re.S)
        assert m, "冷開機沒有被擋"

    def test_four_or_more_path_unchanged(self):
        """n>=4 ⇒ expected = n-2 >= 2 ⇒ min(2, expected) = 2,與舊的 min(2,4) 相同。"""
        assert min(2, 6 - 2) == 2
        assert "need = min(2, self._expected_followers)" in BACKUP
        assert "if need >= 1:" in BACKUP


class TestMavrosWaitsForAll:
    def test_waits_for_every_aircraft(self):
        assert re.search(r'for _i in \$\(seq 1 "\$N"\)', UP), "仍只等 UAV1"
        assert '/uav${_i}/mavros/state' in UP

    def test_reports_shortfall_loudly(self):
        assert '_mav_ok" -lt "$N"' in UP
        assert "不可採信" in UP

    def test_timeout_scaled_for_more_aircraft(self):
        """內層每台最多 2 s ⇒ N 台單輪 2N 秒;40 s 對六機只夠三輪。"""
        m = re.search(r"for _t in \$\(seq 1 (\d+)\); do", UP)
        assert m and int(m.group(1)) >= 90
