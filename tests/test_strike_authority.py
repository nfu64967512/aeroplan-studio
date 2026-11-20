# -*- coding: utf-8 -*-
"""打擊期間的控制權交接 —— 編隊層讓權給 tot_node。

═══ 2026-08-07 事故(AP2,AeroPlan 互動航次)═══
批次路徑靠 `swarm_ros2_down.sh` **把編隊層整個收掉**來交棒(stot_trial.sh:425)。
互動路徑沒有那一步 ⇒ 兩層同時活著,同一架飛機被兩個控制器下相反的令:

    tot_node   : ALIGN,飛 48 km 向西、爬到 perch 400 m
    master_node: `master -> leg 2 (…) @ 80 m` + `leader slowing to 10 m/s`
    backup_node: `UAV2 -> V-tail slot -> (…) @ 50 m`(1 Hz 重發)

編隊層靠重發頻率贏 ⇒ 高度 122→74 m,飛機在起飛點繞圈,`d` 幾乎不減少。

修法(使用者定調):**軟交接** —— 訂閱 `/swarm/tot_status`,
phase 進 BUSY 就停止下令,但保持存活繼續發遙測;回 IDLE 或逾時就收回。
"""
import importlib.util
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
P = ROOT / "scripts" / "wsl" / "sar_swarm_patches"


def _load(name):
    """直接從檔案載入(不需要 ROS2 runtime —— 這個模組刻意零 rclpy 相依)。"""
    spec = importlib.util.spec_from_file_location(name, P / (name + ".py"))
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


sa = _load("strike_authority")

MASTER = (P / "master_node.py").read_text(encoding="utf-8")
BACKUP = (P / "backup_node.py").read_text(encoding="utf-8")
WING = (P / "wingman_node.py").read_text(encoding="utf-8")
DEPLOY = (ROOT / "scripts" / "wsl" / "deploy_build.sh").read_text(encoding="utf-8")


class TestPredicate:
    @pytest.mark.parametrize("phase", ["ALIGN", "PERCH", "DIVE"])
    def test_busy_phases_yield(self, phase):
        assert sa.should_yield(phase, 0.1) is True

    @pytest.mark.parametrize("phase", ["STANDBY", "INIT", "DONE"])
    def test_idle_phases_do_not_yield(self, phase):
        assert sa.should_yield(phase, 0.1) is False

    def test_never_heard_does_not_yield(self):
        """從沒收過狀態 ⇒ 沒有人在打擊 ⇒ 編隊層照常。"""
        assert sa.should_yield(None, 0.1) is False

    def test_stale_status_reclaims_control(self):
        """⚠️⚠️ 失效方向:逾時**收回**控制。
        反過來(逾時繼續讓權)會讓飛機在沒有任何控制器的情況下滑走。"""
        assert sa.should_yield("DIVE", 99.0) is False

    def test_boundary_is_inclusive_of_fresh(self):
        assert sa.should_yield("DIVE", 4.9, max_age=5.0) is True
        assert sa.should_yield("DIVE", 5.1, max_age=5.0) is False

    def test_kill_switch(self):
        """max_age<=0 ⇒ 完全關掉讓權(回退到 2026-08-07 之前的行為)。"""
        assert sa.should_yield("DIVE", 0.1, max_age=0.0) is False

    def test_default_max_age_above_standby_rate(self):
        """⚠️ tot_node 待命時只有 1 Hz;門檻小於 1 s 會一直在讓權/收回之間抖。"""
        assert sa.DEFAULT_MAX_AGE_S >= 2.0

    def test_unknown_phase_does_not_yield(self):
        """沒見過的相位當成「不在打擊」—— 安全方向與逾時一致。"""
        assert sa.should_yield("WAT", 0.1) is False


class TestParsing:
    def test_reads_phase(self):
        assert sa.parse_phase('{"phase": "DIVE", "T_arr": 1.0}') == "DIVE"

    @pytest.mark.parametrize("bad", [
        "", "not json", "[]", "null", "123", '{"T_arr": 1}', '{"phase": 3}',
    ])
    def test_bad_message_returns_none(self, bad):
        """壞訊息 ⇒ None ⇒ 不讓權(和逾時同一個安全方向)。"""
        assert sa.parse_phase(bad) is None

    def test_none_input(self):
        assert sa.parse_phase(None) is None

    def test_real_standby_payload(self):
        """實際的待命狀態(取自 AP2 的 log)。"""
        s = ('{"phase": "STANDBY", "T_arr": null, "uavs": {}, "standby": true,'
             ' "ready_for_target": true, "n_pos": 2, "n": 2}')
        assert sa.parse_phase(s) == "STANDBY"
        assert sa.should_yield(sa.parse_phase(s), 0.1) is False

    def test_real_align_payload(self):
        s = ('{"phase": "ALIGN", "T_arr": 570.3,'
             ' "uavs": {"1": {"mode": "straight", "d": 47893}}}')
        assert sa.should_yield(sa.parse_phase(s), 0.1) is True


class TestContractWithTheOtherSide:
    def test_busy_phases_match_the_ui_bridge(self):
        """⚠️ integrations/rclick_strike.py 有同一組常數,改一邊要改兩邊。"""
        src = (ROOT / "integrations" / "rclick_strike.py").read_text(encoding="utf-8")
        for ph in sa.BUSY_PHASES:
            assert 'PHASE_%s = "%s"' % (ph, ph) in src
        assert "PHASE_BUSY = (PHASE_ALIGN, PHASE_PERCH, PHASE_DIVE)" in src

    def test_topic_matches_tot_node(self):
        tot = (P / "tot_node.py").read_text(encoding="utf-8")
        assert '"/swarm/tot_status"' in tot
        assert sa.STATUS_TOPIC == "/swarm/tot_status"

    def test_phases_are_disjoint(self):
        assert not set(sa.BUSY_PHASES) & set(sa.IDLE_PHASES)


#: ⚠️ 不可以把原始碼本身當參數 —— pytest 會把它塞進測試 ID，
#:    而測試 ID 會寫進環境變數（PYTEST_CURRENT_TEST），
#:    Windows 上超過 32767 字元就 ValueError，整個測試檔收不起來。
SOURCES = {"master_node": MASTER, "backup_node": BACKUP, "wingman_node": WING}


class TestEveryCommandingNodeYields:
    @pytest.mark.parametrize("name", sorted(SOURCES))
    def test_imports_the_shared_module(self, name):
        assert ("from sar_swarm.strike_authority import StrikeAuthority"
                in SOURCES[name]), name

    @pytest.mark.parametrize("name", sorted(SOURCES))
    def test_constructs_it(self, name):
        assert "StrikeAuthority(self)" in SOURCES[name], name

    def test_master_gates_its_tick(self):
        i = MASTER.index("def _tick(self):")
        head = MASTER[i:i + 1400]
        assert "self._strike_auth.yielded()" in head

    def test_master_checks_before_the_foreign_leader_branch(self):
        """⚠️ 順序有意義:tot_node 讓權要在最前面,不然這一 tick 還是會下令。"""
        i = MASTER.index("def _tick(self):")
        body = MASTER[i:i + 2500]
        assert body.index("_strike_auth.yielded()") < body.index("_foreign_leader_t")

    def test_wingman_gates_its_fly(self):
        i = WING.index("def _fly(self):")
        assert "self._strike_auth.yielded()" in WING[i:i + 900]

    @pytest.mark.parametrize("fn", ["_fly_standby", "_acting_tick", "watchdog"])
    def test_backup_gates_all_three_entry_points(self, fn):
        """備長有三條路會下令/奪權:待命飛行、代理長機、接管判定。"""
        i = BACKUP.index("def %s(self)" % fn)
        assert "self._strike_auth.yielded()" in BACKUP[i:i + 1600], fn


class TestBackupSuspendsTakeover:
    def test_takeover_is_suspended_not_disabled(self):
        """⚠️ 讓權會自己過期 ⇒ tot_node 真的掛了,接管判定立刻恢復。
        這不是把防護關掉,是把它讓開。"""
        assert sa.should_yield("DIVE", 99.0) is False

    def test_heartbeat_clock_is_reset_on_resume(self):
        """⚠️ 收回控制的當下,last_hb 可能已經舊了 ⇒ 直接比較會誤判
        「心跳消失」而立刻接管。必須重置計時基準。"""
        assert "_hb_reset_on_resume" in BACKUP
        assert "self.last_hb = self._now()" in BACKUP

    def test_reset_flag_set_while_yielded(self):
        i = BACKUP.index("def watchdog(self):")
        body = BACKUP[i:i + 1800]
        assert "self._hb_reset_on_resume = True" in body


class TestObservability:
    def test_logs_both_directions(self):
        src = (P / "strike_authority.py").read_text(encoding="utf-8")
        assert "讓權給 tot_node" in src
        assert "收回控制" in src

    def test_says_what_keeps_running(self):
        """⚠️ 只停下令、不停發布 —— 訊息要講清楚,不然現場會以為節點死了。"""
        src = (P / "strike_authority.py").read_text(encoding="utf-8")
        assert "遙測" in src and "心跳" in src

    def test_only_logs_on_change(self):
        """0.5–1 Hz 的 tick,每次都印會把 log 洗掉。"""
        src = (P / "strike_authority.py").read_text(encoding="utf-8")
        assert "if y != self._yielded:" in src


class TestDeployed:
    def test_in_the_deploy_manifest(self):
        """⚠️ 這個清單漏過兩次(preflight_check.py、_kpi_parse.py),
        漏了的話三個節點會 ImportError 而**整層起不來**。"""
        assert "strike_authority.py" in DEPLOY

    def test_listed_under_nodes_not_scripts(self):
        i = DEPLOY.index("NODES=")
        j = DEPLOY.index("LAUNCHES=")
        assert "strike_authority.py" in DEPLOY[i:j]
