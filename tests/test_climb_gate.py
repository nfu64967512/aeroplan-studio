# -*- coding: utf-8 -*-
"""爬升未完成必須大聲失敗,不可以帶著爬到一半的機隊往下跑。

═══ 這條鎖的是什麼 ═══
`jet_catapult_launch.py` 的成功哨兵 `全員 GUIDED` 印在**爬升區塊之前**,
所以它只證明「都進 GUIDED 了」,**不證明爬到巡航高度**。而爬升迴圈原本是:

    while time.time() - t0 < budget:
        ...
        if lo >= climb_to - tol: ...; break
        time.sleep(10)
    # ← 預算用完就從這裡掉出來:沒有錯誤輸出、沒有非零退出、沒有失敗標記

⇒ `stot_trial.sh` 的 `grep -q "全員 GUIDED"` 照樣通過。

這正是 Q1 事故的形狀:只爬到 ~100 m ⇒ 末端誤差 **44 km**。
當時補的閘門只檢查「CLIMB_TO_ALT 這個**變數有沒有設**」,
漏的是「有沒有**真的爬到**」——所以同一個坑還在。

⚠️ 另外記一筆 100 m 的死區(本輪未改,先記著):
   `CLIMB_TO_TOL` 預設 200 ⇒ CLIMB_TO_ALT=3100 時 **2900 m 就算爬完**;
   而編隊層的 `CRUISE_ALT_THRESHOLD` 是 3000 ⇒ 彈射端宣告爬完、編隊端仍判定在爬升。
   `CLIMB_TO_TOL` / `CLIMB_TO_TIMEOUT` 在四支驅動腳本裡都沒設過,全吃 python 預設。
"""
import pathlib
import re

ROOT = pathlib.Path(__file__).resolve().parents[1]
LAUNCH = (ROOT / "scripts" / "wsl" / "jet_catapult_launch.py").read_text(encoding="utf-8")
TRIAL = (ROOT / "scripts" / "wsl" / "stot_trial.sh").read_text(encoding="utf-8")


class TestLauncherFailsLoudly:
    def test_tracks_whether_climb_completed(self):
        assert "ok_climb" in LAUNCH, "沒有追蹤爬升是否完成"

    def test_sets_flag_only_on_success_path(self):
        """旗標必須設在「達到高度」那一支裡,不能無條件設。"""
        m = re.search(r"if lo >= climb_to - tol:(.*?)\n            time\.sleep",
                      LAUNCH, re.S)
        assert m, "找不到爬升成功分支"
        assert "ok_climb = True" in m.group(1)

    def test_nonzero_exit_on_incomplete_climb(self):
        m = re.search(r"if not ok_climb:(.*?)sys\.exit\((\d+)\)", LAUNCH, re.S)
        assert m, "爬升未完成沒有非零退出"
        assert int(m.group(2)) != 0
        assert "✗ 爬升未完成" in m.group(1), "失敗訊息缺少可 grep 的標記"

    def test_failure_message_names_the_knobs(self):
        """訊息要指出怎麼放寬,否則下一個人只能猜。"""
        assert "CLIMB_TO_TIMEOUT" in LAUNCH
        assert "CLIMB_TO_TOL" in LAUNCH


class TestTrialGatesOnClimb:
    def test_captures_launch_rc(self):
        """rc 沒抓 ⇒ python 端的非零退出等於沒用。"""
        assert re.search(r"_launch_rc=\$\?", TRIAL)

    def test_gate_checks_all_three_signals(self):
        """rc、失敗標記、成功標記 —— 只認其中一種都會被漏掉。"""
        m = re.search(r'if \[ "\$\{CLIMB_TO_ALT:-0\}" != "0" \]; then(.*?)\nfi',
                      TRIAL, re.S)
        assert m, "找不到爬升閘門"
        b = m.group(1)
        assert '"$_launch_rc" != "0"' in b, "沒檢查 rc"
        assert "✗ 爬升未完成" in b, "沒檢查失敗標記"
        assert "全員抵巡航高度" in b, "沒檢查成功標記"

    def test_gate_is_conditional_on_climb_being_requested(self):
        """CLIMB_TO_ALT=0(不爬)時不該誤擋。"""
        assert '"${CLIMB_TO_ALT:-0}" != "0"' in TRIAL

    def test_records_a_distinct_failure_reason(self):
        """要和 launch_failed 分開,否則事後分不出是彈射壞還是爬升壞。"""
        assert "climb_failed" in TRIAL
        assert "launch_failed" in TRIAL

    def test_guided_sentinel_alone_is_not_the_climb_gate(self):
        """`全員 GUIDED` 印在爬升之前,不可以拿它當爬升判準。"""
        i_guided = TRIAL.index('grep -q "全員 GUIDED"')
        i_climb = TRIAL.index("全員抵巡航高度")
        assert i_guided < i_climb, "爬升閘門必須是**另一道**,排在 GUIDED 之後"
