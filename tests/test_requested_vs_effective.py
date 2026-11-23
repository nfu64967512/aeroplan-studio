# -*- coding: utf-8 -*-
"""「要求的值 ≠ 生效值」比對器(2026-08-23)。

═══════════════════════════════════════════════════════════════════════════
 這一層在補什麼洞
═══════════════════════════════════════════════════════════════════════════
契約表原本的四類檢查問的都是「**這個生效值好不好**」:

    range      在不在宣告區間
    foam       是不是泡棉時代的禁區值
    invariant  跨參數關係成不成立
    missing    讀不讀得到

四類都無法回答「**這個值是不是你要的**」—— 因為意圖只存在於環境變數裡,
而環境變數從來沒有進到任何節點或驗證器看得到的地方。

⚠️⚠️ 代價是實際付過的。2026-08-23,WV1-3 三趟(約 2.5 小時 SITL)整批白跑:

    export WEAVE_TIME_SYNC=True WEAVE_RELATIVE=True RELEASE_AT_RING=True \\
           WAIT_GOTO_RADIUS=True DIVE_FROM_RUNUP=True

節點**五顆全部跑 0**,因為那條啟動路徑(`stot_trial.sh`)用一份手動維護的
`-p` 清單,55 個 tot_node 契約參數只傳得到 24 個(見
`tests/test_launcher_param_coverage.py`)。而五個退回值都在區間內、都不是
泡棉值 ⇒ 契約行照印、閘門說 OK、開跑前驗證也過。
三趟的結論本來會被寫成「蛇形通道無效」。

⇒ `requested_vs_effective()` 是**唯一**問「你要的 vs 你拿到的」的那一層。

═══════════════════════════════════════════════════════════════════════════
 ⭐ 本檔一律值層級
═══════════════════════════════════════════════════════════════════════════
餵一組 (生效值, 環境變數) 進去,對送出來的 Violation 下斷言。
沒有任何源碼字串比對([[value-locks-beat-source-locks]])。
"""
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "wsl" / "sar_swarm_patches"))

import contract_table as ct                                    # noqa: E402


def rve(values, environ):
    return ct.requested_vs_effective(values, environ)


def names(vs):
    return sorted(v.name for v in vs)


class TestItCatchesTheWv1Failure:
    """⭐⭐ 先把那三趟的實際形狀重現一遍,再談別的。"""

    #: WV1 的實況:五顆 export True,節點五顆全 0。
    WV1_REQUESTED = {
        "WEAVE_TIME_SYNC": "True", "WEAVE_RELATIVE": "True",
        "RELEASE_AT_RING": "True", "WAIT_GOTO_RADIUS": "True",
        "DIVE_FROM_RUNUP": "True",
    }
    WV1_EFFECTIVE = {
        "weave_time_sync": 0.0, "weave_relative": 0.0,
        "release_at_ring": 0.0, "wait_goto_radius": 0.0,
        "dive_from_runup": 0.0,
    }

    def test_all_five_dropped_switches_are_caught(self):
        got = names(rve(self.WV1_EFFECTIVE, self.WV1_REQUESTED))
        assert got == sorted(self.WV1_EFFECTIVE), (
            f"⚠️⚠️ 只抓到 {got} —— 這五顆就是 WV1-3 白跑的全部原因")

    def test_the_old_four_checks_would_all_have_passed(self):
        """⚠️⚠️ 這一條才是重點:證明**沒有這一層時真的沒人擋得住**。

        把同一組生效值餵給 evaluate(),五顆開關必須一項違規都不產生 ——
        它們在區間內、不是泡棉值、不變式也沒話說。
        """
        rep = ct.evaluate(self.WV1_EFFECTIVE, require=list(self.WV1_EFFECTIVE))
        assert rep.ok, (
            "⚠️ 既有四類檢查竟然抓得到?那本層的存在理由要重寫。\n"
            + rep.text())

    def test_the_violation_names_the_env_var_that_was_ignored(self):
        """⭐ 訊息要能直接把人帶到修法。只說「值不對」會讓人去改值,
        而真正的問題是那個值**根本沒送到**。"""
        v = rve({"weave_time_sync": 0.0}, {"WEAVE_TIME_SYNC": "True"})[0]
        assert "WEAVE_TIME_SYNC" in v.want
        assert v.kind == "requested"
        assert "stot_trial" in v.evidence, "反證要指向真正的機制(-p 清單漏了它)"


class TestItStaysQuietWhenItShould:
    """⚠️ 一個會亂叫的檢查等於沒有檢查 —— 它會被關掉。"""

    def test_an_unset_env_var_is_not_an_opinion(self):
        assert rve({"weave_time_sync": 0.0}, {}) == []

    @pytest.mark.parametrize("blank", ["", "   ", "\t"])
    def test_an_empty_env_var_is_not_an_opinion(self, blank):
        """⚠️ `export FOO   =1`(等號前有空格)在 bash 會產生**空字串**。
        那是另一條 lint 的事(test_launcher_param_coverage),
        這裡不重複報 —— 同一個錯誤報兩次會讓人只修一個地方。"""
        assert rve({"weave_time_sync": 0.0}, {"WEAVE_TIME_SYNC": blank}) == []

    def test_a_matching_value_is_silent(self):
        assert rve({"vmax": 150.0}, {"VMAX": "150.0"}) == []

    def test_an_unreadable_effective_value_is_left_to_missing(self):
        """讀不到 ⇒ 既有的 missing 會報。這裡再報一次只是噪音。"""
        assert rve({}, {"WEAVE_TIME_SYNC": "True"}) == []

    def test_a_parameter_with_no_env_mapping_is_skipped(self):
        """契約表上沒有 ENV 映射的參數,沒有「被要求」這個概念。"""
        no_map = [c.name for c in ct.TABLE if c.name not in ct.ENV_FOR]
        assert no_map, "每一項都有映射的話這條測試沒有意義,請改寫"
        assert rve({no_map[0]: 1.0}, {no_map[0].upper(): "999"}) == []


class TestBooleanSpellingsAreAllRecognised:
    """⚠️⚠️ shell 這一側歷來三種寫法都出現過。認漏一種 = 對那一種沒有意見,
    而那正是本層要治的病。"""

    @pytest.mark.parametrize("truthy", ["True", "true", "TRUE", "1", "yes", "on"])
    def test_true_spellings_flag_a_zero_node_value(self, truthy):
        assert names(rve({"weave_time_sync": 0.0},
                         {"WEAVE_TIME_SYNC": truthy})) == ["weave_time_sync"]

    @pytest.mark.parametrize("falsy", ["False", "false", "0", "no", "off"])
    def test_false_spellings_flag_a_one_node_value(self, falsy):
        assert names(rve({"ring_pace": 1.0},
                         {"RING_PACE": falsy})) == ["ring_pace"]

    @pytest.mark.parametrize("spelled", ["True", "1", "on"])
    def test_true_spellings_are_silent_when_the_node_agrees(self, spelled):
        assert rve({"weave_time_sync": 1.0}, {"WEAVE_TIME_SYNC": spelled}) == []

    def test_an_unparseable_value_does_not_guess(self):
        """⚠️ 認不出來就**不判定**。猜一個等價值會製造假違規,
        而假違規的代價是整層被關掉。"""
        assert rve({"vmax": 150.0}, {"VMAX": "很快"}) == []


class TestItCatchesBothDirections:
    """⭐ 「要開卻是關的」與「要關卻是開的」都是同一類錯誤。
    只抓一個方向的話,`RING_PACE=False` 而節點跑 1 就會漏掉 ——
    而那正好是 WX 這一批要證明的單變數。"""

    def test_requested_off_but_running_on(self):
        assert names(rve({"ring_pace": 1.0}, {"RING_PACE": "False"})) == ["ring_pace"]

    def test_a_numeric_mismatch_is_caught_too(self):
        """不只開關 —— WV1 的 vmax 也是 100 vs 要求的 150。"""
        assert names(rve({"vmax": 100.0}, {"VMAX": "150.0"})) == ["vmax"]

    def test_a_tiny_float_difference_is_not_flagged(self):
        """⚠️ 浮點往返(shell 印 %g、節點回 double)不可以變成違規。"""
        assert rve({"aero_cl_zero": 0.56381}, {"AERO_CL_ZERO": "0.56381"}) == []


class TestTheVerifierActuallyGatesOnIt:
    """⭐⭐ 比對器算得出違規、但沒有接進 contract_verify 的話,等於沒做。

    ⚠️ 這一條防的是本專案吃過三次的那一類:「註解說有鎖,實際沒有」
    ([[promised-guards-that-do-not-exist]])。判別法只有跑刪除實驗,
    而這條測試就是那個實驗的自動版。
    """

    def test_report_ok_turns_false_when_a_requested_value_was_ignored(self):
        rep = ct.evaluate({"weave_time_sync": 0.0}, require=["weave_time_sync"])
        assert rep.ok, "前提:四類檢查對它沒有意見"
        rep.violations.extend(
            ct.requested_vs_effective({"weave_time_sync": 0.0},
                                      {"WEAVE_TIME_SYNC": "True"}))
        assert not rep.ok, (
            "⚠️⚠️ Report.ok 沒有反映新加的違規 ⇒ 退出碼不會變 ⇒ 閘門形同虛設")

    def test_contract_verify_imports_and_calls_it(self):
        """⚠️ 這一條是**源碼層**的,而且刻意如此:值層級測不到「有沒有接線」。
        它只驗兩件事 —— 名字有被 import、而且在 evaluate 之後被呼叫。"""
        src = (ROOT / "scripts" / "wsl" / "contract_verify.py").read_text(
            encoding="utf-8", errors="replace")
        assert "requested_vs_effective" in src, (
            "⚠️⚠️ contract_verify.py 沒有用比對器 ⇒ 它算得再對也不會擋任何東西")
        i_eval = src.find("rep = evaluate(")
        i_req = src.find("requested_vs_effective(values")
        assert 0 < i_eval < i_req, (
            "⚠️ 比對器必須在 evaluate() **之後**呼叫並把結果併進同一份報告,"
            "否則違規不會進 rep.violations,退出碼也不會變。")
