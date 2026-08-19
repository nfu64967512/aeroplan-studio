# -*- coding: utf-8 -*-
"""高度閘門的死區:編隊層的判定門檻必須由爬升段的目標推導。

═══ 這個 bug 的形狀 ═══
兩個獨立寫死的數字,各自都合法,**關係**卻是錯的:

    彈射端(jet_catapult_launch.py)宣告「全員抵巡航高度」:
        min_alt >= CLIMB_TO_ALT − CLIMB_TO_TOL   = 3100 − 200 = 2900 m
    編隊層(wingman_node / master_node 的 cruise_alt_threshold)判定「已進巡航」:
        alt >= CRUISE_ALT_THRESHOLD              = 3000 m(先前寫死)

⇒ 2900–3000 m 這 **100 m 死區**裡,彈射端說「爬完了、交棒」,
  編隊層卻仍判定「還在爬升」而不做協同讓速 —— 而且完全靜默,兩邊各自都覺得自己對。

`CLIMB_TO_TOL` 與 `CLIMB_TO_TIMEOUT` 在四支驅動腳本裡**都沒設過**,全吃 python 端預設,
所以這個關係從來沒有被任何地方保證過。

⚠️ `preflight_check.py` 的檔頭自己寫著規則:
   「**檢查關係不只檢查值** —— 壞掉的從來不是單一參數,是參數之間的關係」。
   死區正是關係,所以只驗單一參數永遠抓不到。
"""
import pathlib
import re

import pytest

W = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "wsl"
PRE = (W / "preflight_check.py").read_text(encoding="utf-8")
WEST = (W / "run_west.sh").read_text(encoding="utf-8")


def deadzone(climb_to_alt, climb_tol, cruise_thr):
    """彈射端保證的下限之上還有多少是編隊層不認的。>0 就是死區。"""
    return cruise_thr - (climb_to_alt - climb_tol)


def derived_threshold(climb_to_alt, climb_tol, margin):
    return climb_to_alt - climb_tol - margin


class TestTheBugItself:
    def test_historical_values_have_a_deadzone(self):
        """3100 / 200 / 3000 —— 這組實際跑過,死區 100 m。"""
        assert deadzone(3100.0, 200.0, 3000.0) == pytest.approx(100.0)

    def test_derived_threshold_removes_it(self):
        thr = derived_threshold(3100.0, 200.0, 50.0)
        assert thr == pytest.approx(2850.0)
        assert deadzone(3100.0, 200.0, thr) <= 0.0

    @pytest.mark.parametrize("alt,tol,margin", [
        (3100.0, 200.0, 50.0), (3100.0, 100.0, 50.0),
        (4000.0, 200.0, 50.0), (3100.0, 200.0, 0.0),
        (1200.0, 50.0, 25.0),
    ])
    def test_derivation_never_creates_a_deadzone(self, alt, tol, margin):
        """推導式對任何合理輸入都不該產生死區 —— 這是它存在的理由。"""
        assert deadzone(alt, tol, derived_threshold(alt, tol, margin)) <= 0.0

    def test_changing_climb_altitude_moves_the_gate(self):
        """⭐ 使用者要的『動態更改』:改爬升高度,編隊閘門要跟著走。"""
        a = derived_threshold(3100.0, 200.0, 50.0)
        b = derived_threshold(4000.0, 200.0, 50.0)
        assert b - a == pytest.approx(900.0)


class TestPreflightAsserts:
    """兩條路徑(批次 / UI 右鍵)都會經過 preflight,所以檢查要放在那裡。"""

    def test_reads_all_three_quantities(self):
        for k in ("CLIMB_TO_ALT", "CLIMB_TO_TOL", "CRUISE_ALT_THRESHOLD"):
            assert 'env_f("%s"' % k in PRE, k

    def test_compares_the_relation_not_the_value(self):
        assert re.search(r"floor\s*=\s*c_alt\s*-\s*c_tol", PRE), "沒有算彈射端下限"
        assert re.search(r"if\s+c_thr\s*>\s*floor:", PRE), "沒有比較關係"

    def test_failure_message_is_actionable(self):
        m = re.search(r"高度閘門死區(.*?)\"\)", PRE, re.S)
        assert m, "找不到死區告警"
        b = m.group(1)
        for k in ("CLIMB_TO_ALT", "CLIMB_TO_TOL", "CRUISE_ALT_THRESHOLD"):
            assert k in b, "訊息沒告訴使用者可以調哪個:%s" % k

    def test_hysteresis_cannot_swallow_the_gate(self):
        """遲滯大於閘門 ⇒ 退出線為負 ⇒ 一旦進入就永遠不退出。"""
        assert "CRUISE_GATE_HYST" in PRE
        assert re.search(r"c_thr\s*-\s*hyst\s*<\s*0", PRE)

    def test_climb_to_tol_default_matches_the_launcher(self):
        """preflight 用的預設值必須和 jet_catapult_launch.py 的一致,
        否則檢查本身就建立在錯的假設上。"""
        launch = (W / "jet_catapult_launch.py").read_text(encoding="utf-8")
        m = re.search(r'os\.environ\.get\("CLIMB_TO_TOL",\s*"(\d+)"\)', launch)
        assert m, "彈射腳本沒有 CLIMB_TO_TOL 預設"
        assert 'env_f("CLIMB_TO_TOL", %s.0)' % m.group(1) in PRE, \
            "preflight 的 CLIMB_TO_TOL 預設與彈射腳本不一致"


class TestWestPanelDerives:
    def test_threshold_is_derived_not_hardcoded(self):
        assert "CRUISE_ALT_THRESHOLD:-3000.0" not in WEST, "還寫死 3000"
        assert re.search(r'CRUISE_ALT_THRESHOLD="\$\{CRUISE_ALT_THRESHOLD:-\$\(awk',
                         WEST), "沒有由 CLIMB_TO_ALT 推導"

    def test_knobs_are_still_overridable(self):
        """對照實驗要能覆寫,所以三個都必須是 ${VAR:-...} 形式。"""
        for k in ("CLIMB_TO_TOL", "CRUISE_GATE_MARGIN", "CRUISE_ALT_THRESHOLD"):
            assert re.search(r'export %s="\$\{%s:-' % (k, k), WEST), k

    def test_panel_asserts_before_launch(self):
        assert "高度閘門死區" in WEST, "面板沒有開跑前斷言"
