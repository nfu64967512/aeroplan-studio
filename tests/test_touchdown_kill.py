# -*- coding: utf-8 -*-
"""觸水終結:SITL 不會因為落海而終結機體,不強制 disarm 就會浮起來再飛一趟。

實測(runs/tot_TWO1.csv,2026-08-06,N=2):
    UAV1  觸地 t=1379.37,agl 最低 −87.9 m,之後 79 筆爬回 **+15.7 m**
    UAV2  觸地 t=1371.70,agl 最低 −123.2 m,之後 233 筆(11.7 s)爬回 **+1103.2 m**
航次在 t=1383.4 結束才沒跑完第二趟。

要堵的是 tot_node **現場**的 impact 鎖存 —— 它只看水平距、完全不看高度,
而 miss 窗有 60 s(stot_trial.sh 的 MISS_WINDOW_S)。一架在紅心旁
> impact_reach 觸水、因此還沒鎖存的機,爬回來重穿就會被記成 impact,
污染節點摘要行與 stot_trial.sh 的 `grep -c "命中紅心 @"`。

⚠️ 修的不是計分器:stot_score.py 的「AGL≤0 無條件收窗」已於 2026-08-04 堵死那一側
   (TWO1 兩架的「收窗」欄都是 touchdown)。這裡修的是節點的現場判定與飛行行為。
"""
import math
import pathlib
import re

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
NODE = ROOT / "scripts" / "wsl" / "sar_swarm_patches" / "tot_node.py"
TRIAL = ROOT / "scripts" / "wsl" / "stot_trial.sh"
ARTI = ROOT / "scripts" / "wsl" / "make_artifacts.sh"

NODE_SRC = NODE.read_text(encoding="utf-8")
TRIAL_SRC = TRIAL.read_text(encoding="utf-8")


def should_kill(agl, n_lo, *, on=True, kill_agl=0.0, confirm_n=2):
    """把節點那條述詞獨立實作一次(不 import 節點,免得同源錯誤)。"""
    return on and agl <= kill_agl and n_lo >= confirm_n


class TestPredicate:
    def test_single_spike_does_not_kill(self):
        """單筆跳點不足以終結一架飛機(地形快取 / relative_alt 抖動都會製造單筆負值)。"""
        assert not should_kill(-5.0, 1)

    def test_two_consecutive_kills(self):
        """連續兩筆(20 Hz ⇒ 0.1 s)才動手。"""
        assert should_kill(-5.0, 2)

    def test_switch_off_never_kills(self):
        """KILL_ON_TOUCHDOWN=False 要能完全回到舊行為 —— A/B 只差這一個變數。"""
        assert not should_kill(-120.0, 99, on=False)

    def test_positive_agl_never_kills(self):
        """還在空中就不能終結,無論連續幾筆。"""
        assert not should_kill(0.5, 99)

    def test_exactly_zero_is_touchdown(self):
        """AGL == 0 就是觸地(邊界含在內),與 stot_score.py 的收窗規則同口徑。"""
        assert should_kill(0.0, 2)

    @pytest.mark.parametrize("agl", [-87.9, -123.2])
    def test_two1_measured_depths_would_kill(self, agl):
        """TWO1 兩架實測的最低 AGL 都應該早就觸發終結。"""
        assert should_kill(agl, 2)

    def test_climb_back_would_reset_counter_but_kill_already_latched(self):
        """爬回水面上時述詞為偽 —— 所以終結必須是**一次性鎖存**,不能每 tick 重評。

        UAV2 從 −123.2 m 爬回 +1103.2 m:若沒有 self.killed 的鎖存,
        飛機一浮上水面述詞就轉偽,等於從沒終結過。
        """
        assert should_kill(-123.2, 2)
        assert not should_kill(1103.2, 2)


class TestNodeWiring:
    def test_params_declared(self):
        for name in ("kill_on_touchdown", "kill_agl_m", "kill_confirm_n"):
            assert f'g("{name}"' in NODE_SRC, name

    def test_params_read_into_attributes(self):
        assert 'p("kill_on_touchdown").value' in NODE_SRC
        assert 'p("kill_agl_m").value' in NODE_SRC
        assert 'p("kill_confirm_n").value' in NODE_SRC

    def test_uses_force_disarm_magic(self):
        """param2 = 21196 是 force magic;少了它,飛行中的 disarm 會被飛控拒收。"""
        assert "MAV_CMD_COMPONENT_ARM_DISARM" in NODE_SRC
        assert "21196" in NODE_SRC

    def test_kill_is_latched_once(self):
        """i in self.killed 的早退 —— 見 test_climb_back...。"""
        assert re.search(r"if not self\.kill_on_touch or i in self\.killed:\s*\n\s*return",
                         NODE_SRC)

    def test_kill_has_exactly_one_call_site(self):
        """多個呼叫點 = 遲早有一個會被漏掉或放錯層級(見下一條)。"""
        assert NODE_SRC.count("self._kill_airframe(") == 1

    def test_kill_is_called_from_inside_row(self):
        """⚠️⚠️ KIL2(2026-08-06)實測的 bug:呼叫點放在 DIVE 迴圈的**外面**。

        原本放在和 `done = self._done_count()` 同一層 —— 那一層是「每個 tick 一次」,
        在逐機迴圈之外 ⇒ 每 tick 只用迴圈殘留的 `i` 跑一次 ⇒ **只有最後一架被評估**。
        後果:UAV1 有 72 筆連續 AGL≤0 卻從未終結,照樣爬回 +109.5 m;log 毫無異常。

        放進 _row 之後,三件事變成不變量而不是慣例:
          ① 一定在 writerow 之後(計分器要 (AGL>0, AGL≤0) 這對相鄰取樣做內插);
          ② DIVE 迴圈的三個 _row 呼叫點(missed / 未放行 / 主路徑)全部涵蓋 ——
             前兩個後面就 continue,放呼叫端一定漏;
          ③ ALIGN/PERCH 段撞海也會被終結。
        """
        m = re.search(r"def _row\(self.*?(?=\n    def )", NODE_SRC, re.S)
        assert m, "找不到 _row"
        body = m.group(0)
        assert "self._kill_airframe(" in body, "_kill_airframe 必須從 _row 內呼叫"
        assert body.index("self._cw.writerow(") < body.index("self._kill_airframe("), \
            "終結必須排在 writerow 之後"

    def test_kill_not_called_from_the_per_tick_level(self):
        """鎖住那個具體的錯誤形狀:呼叫點不可以和 _done_count() 同層。"""
        for line in NODE_SRC.splitlines():
            if "self._kill_airframe(" in line and "def " not in line:
                indent = len(line) - len(line.lstrip())
                assert indent == 8, (
                    "_kill_airframe 應該只在 _row 的方法體(8 空格)內被呼叫,"
                    "實際縮排 %d —— 放在迴圈外會只評估最後一架" % indent)

    def test_log_marker_is_greppable(self):
        """log 標記要能被 make_artifacts.sh 的事件列撈到,否則產物裡看不到。"""
        assert "觸水終結" in NODE_SRC
        assert "觸水終結" in ARTI.read_text(encoding="utf-8")


class TestTrialPlumbing:
    def test_defaults_present(self):
        assert 'KILL_ON_TOUCHDOWN="${KILL_ON_TOUCHDOWN:-True}"' in TRIAL_SRC
        assert 'KILL_AGL_M="${KILL_AGL_M:-0.0}"' in TRIAL_SRC
        assert 'KILL_CONFIRM_N="${KILL_CONFIRM_N:-2}"' in TRIAL_SRC

    def test_hold_radius_has_an_env_path(self):
        """⚠️ 節點的 _check_hold_radius() 每次開機都用 ERROR 吼 hold_radius 太小
        (噴射組態要 1252 m,預設 400),但先前**沒有任何環境變數通路**可以改它 ——
        一個吼得很大聲卻無法處置的守門員。這條鎖住通路本身。"""
        assert "-p hold_radius:=" in TRIAL_SRC
        assert "-p goto_radius:=" in TRIAL_SRC

    def test_hold_radius_default_is_actually_flyable(self):
        """⚠️⚠️ 2026-08-13:本條原本鎖死 `HOLD_RADIUS:-400.0`,理由寫
        「預設值刻意維持 400.0(本輪不改行為)」。

        **補了通路卻沒改值 = 沒修。** 節點的 ERROR 又繼續吼了八天,
        RPK/RPL 九趟全帶著 400 跑完 —— 而守環退化的後果正是
        「放行時距目標的距離在 ±迴轉半徑 之間隨機」,那極可能就是
        我們量到的交接高度 ±600 m 散布的來源。

        改鎖**物理條件**而不是某個字面值:預設必須 ≥ 迴轉半徑
        (vmin 95 EAS @ perch 3100、45° 傾角 ⇒ 1253 m)。
        """
        import pathlib as _p
        import sys as _s
        _s.path.insert(0, str(_p.Path(__file__).resolve().parents[1]
                              / "scripts" / "wsl" / "sar_swarm_patches"))
        from contract_table import turn_radius_m
        m = re.search(r'HOLD_RADIUS="\$\{HOLD_RADIUS:-([0-9.]+)\}"', TRIAL_SRC)
        assert m, "stot_trial.sh 找不到 HOLD_RADIUS 預設"
        need = turn_radius_m(95.0, 3100.0)
        assert float(m.group(1)) >= need, (
            f"HOLD_RADIUS 預設 {m.group(1)} < 迴轉半徑 {need:.0f} m ⇒ "
            f"守環退化成繞自己的最小圈,放行幾何逐機隨機")

    def test_class_defaults_are_safe(self):
        """樁節點(跳過 __init__)不該意外對任何機體送 disarm ⇒ class 層預設必須是 False。"""
        assert re.search(r"^    kill_on_touch = False$", NODE_SRC, re.M)

    def test_forwarded_to_node(self):
        """⚠️ 節點宣告了預設不代表跑得到:stot_trial.sh 把每一項都用 -p 明寫送出,
        沒列進 -p 清單的參數,節點端的預設值就是實際生效值(hold_radius 就是這樣
        沒有環境變數通路的)。這條鎖住通路本身。"""
        for name in ("kill_on_touchdown", "kill_agl_m", "kill_confirm_n"):
            assert f"-p {name}:=" in TRIAL_SRC, name


class TestAglIsImmuneToHomeDatumShift:
    """⚠️⚠️ 2026-08-07:觸水終結誤殺一架 150 m/s 巡航中的飛機。

    `agl = relative_alt − z_g` 的兩個量**基準不同步**:
      * relative_alt 每一筆都跟著飛控當下的 HOME 走
      * z_g 在 `i not in self.zg` 的保護下**只算一次就凍結**

    UAV2 收到目標後 9 秒,HOME 的 GPS 海拔從 +32.0 跳到 −0.31 m
    ⇒ relative_alt 立刻低 32 m,z_g 還停在 −31.99 ⇒ agl 變 −0.2
    ⇒ 觸水成立 ⇒ 強制 disarm。飛機當時在桃園上空 200+ m。

    `kill_confirm_n=2`(20 Hz ⇒ 0.1 s)對此毫無防護力:基準漂移是
    **持續偏移**,不是單筆跳點,連續多少筆都照樣成立。

    修法不是加否決條件,是**把基準對齊**:釘死 AMSL 的目標一律
    `agl = alt_amsl − target_amsl`,而 alt_amsl 與 relative_alt
    取自同一則 GLOBAL_POSITION_INT ⇒ HOME 一跳兩者反向等量,agl 恆定。
    """

    @staticmethod
    def agl(target_amsl, alt_amsl, rel_alt, zg, ground_alt=0.0):
        """把 `_agl` 的純邏輯抄過來(節點要 ROS2 runtime,測試不跑 __init__)。"""
        if not math.isnan(target_amsl) and alt_amsl is not None:
            return alt_amsl - target_amsl
        return rel_alt - (zg if zg is not None else ground_alt)

    def test_reproduces_the_false_kill_under_the_old_formula(self):
        """舊式:HOME 從 +32 掉到 −0.31,飛機**實際高度沒變**,agl 卻變負。"""
        true_amsl = 232.0                       # 飛機真的在 232 m AMSL
        # 基準跳動前:HOME=+32 ⇒ rel=200
        before = self.agl(float("nan"), None, true_amsl - 32.0, -31.99)
        # 基準跳動後:HOME=−0.31 ⇒ rel=232.31,但 z_g 凍在 −31.99
        after = self.agl(float("nan"), None, true_amsl - (-0.31), -31.99)
        assert before == pytest.approx(232.0 - 0.01, abs=0.05)
        # ⚠️ 舊式在**同一個真實高度**下給出兩個差 32 m 的答案 —— 這就是病灶
        assert abs(after - before) > 30.0

    def test_new_formula_is_invariant(self):
        """新式:同一個真實高度,基準怎麼跳都給同一個 agl。"""
        true_amsl = 232.0
        a = self.agl(0.0, true_amsl, true_amsl - 32.0, -31.99)
        b = self.agl(0.0, true_amsl, true_amsl - (-0.31), -31.99)
        assert a == pytest.approx(232.0)
        assert b == pytest.approx(232.0)
        assert a == pytest.approx(b)

    def test_the_actual_incident_no_longer_kills(self):
        """事故值:agl 應該是 +232,不是 −0.2。"""
        got = self.agl(0.0, 232.0, 232.31, -31.99)
        assert got > 100.0, "仍會誤判觸水"

    @pytest.mark.parametrize("home_amsl", [32.0, 0.0, -0.31, 120.0, -50.0])
    def test_invariant_across_arbitrary_datums(self, home_amsl):
        true_amsl = 15.0
        assert self.agl(0.0, true_amsl, true_amsl - home_amsl,
                        0.0 - 32.0) == pytest.approx(15.0)

    def test_genuine_touchdown_still_fires(self):
        """海面 0 AMSL、飛機在 −0.2 AMSL ⇒ agl 仍是 −0.2,照樣終結。"""
        assert self.agl(0.0, -0.2, 0.0, 0.0) == pytest.approx(-0.2)

    def test_bit_identical_when_datum_is_stable(self):
        """⚠️ 基準穩定時新舊兩式必須逐位元相同,否則既有海上資料全部要重評。"""
        for home in (32.0, 0.0, 100.0):
            for true_amsl in (-1.0, 0.0, 50.0, 400.0):
                zg = 0.0 - home                 # z_g ≡ target_amsl − home_amsl
                old = self.agl(float("nan"), None, true_amsl - home, zg)
                new = self.agl(0.0, true_amsl, true_amsl - home, zg)
                assert old == pytest.approx(new)

    def test_land_target_path_untouched(self):
        """沒釘 AMSL(陸上、走地形庫)⇒ 一律走舊式,一個位元都沒動。"""
        assert self.agl(float("nan"), 999.0, 120.0, 35.0) == pytest.approx(85.0)

    def test_falls_back_before_the_first_sample(self):
        """alt_amsl 還沒有(第一筆之前)⇒ 退回舊式,不可以 KeyError。"""
        assert self.agl(0.0, None, 120.0, -32.0) == pytest.approx(152.0)


class TestKillMessageIsDiagnosable:
    def test_ground_speed_is_logged(self):
        """⚠️ 沒有地速,log 上「agl=-0.2m ⇒ disarm」看起來完全合理,
        誤殺就查不出來 —— 這正是 2026-08-07 繞了很久的原因。"""
        assert "地速" in NODE_SRC
        assert "self.gs.get(i, float('nan'))" in NODE_SRC

    def test_speed_is_not_a_gate(self):
        """真正的觸水發生在 ~140 m/s 的俯衝末端。拿速度當閘門會把終結
        延後到「浮起來之後」—— 那是 second-pass contamination 的病灶。"""
        m = re.search(r"def _should_kill\(self, agl, n_lo\):(.*?)\n    def ", NODE_SRC, re.S)
        assert m
        assert "gs" not in m.group(1), "速度跑進閘門了 —— 只准印,不准擋"


class TestRowUsesTheHelper:
    def test_row_calls_agl_helper(self):
        assert "agl = self._agl(i)" in NODE_SRC

    def test_alt_amsl_recorded_at_both_pump_sites(self):
        """主鏈路與分流鏈路都要記,否則其中一條會退回舊式而靜默復發。"""
        assert NODE_SRC.count("self.alt_amsl[i] = m.alt / 1000.0") == 2

    def test_alt_amsl_comes_from_the_same_message(self):
        """⚠️ 免疫性的**唯一**來源就是「同一則訊息」。分開取樣就白做了。"""
        for m in re.finditer(r"self\.home_amsl\[i\] = m\.alt / 1000\.0 - "
                             r"m\.relative_alt / 1000\.0(.*?)self\.alt_amsl\[i\]",
                             NODE_SRC, re.S):
            assert "recv_match" not in m.group(1), "中間又收了一次訊息"


class TestKillNeedsProofItWasAirborne:
    """⚠️⚠️ 2026-08-10 REP1：起飛後 25 s 就下目標，兩機在 **6 秒內**全被觸水終結，
    而當下地速 95–106 m/s：

        t=0.0 uav=1 alt=-30.252 gs=100.4 agl= 1.8
        t=0.0 uav=2 alt=-36.089 gs=105.8 agl=-4.0

    真因**不是**基準不同步（那條 2026-08-07 已修），而是
    **飛控的高度估計還沒收斂** —— 剛 arm/起飛那幾十秒 EKF 原點在沉澱，
    relative_alt 回報 −30 m。`agl` 忠實地反映了一個**錯的輸入**。

    ⇒ 補上物理前提：**沒飛起來過就不可能觸水**。
    """

    @staticmethod
    def armed_kill(agl, n_lo, kill_agl=0.0, confirm=2, was_airborne=True):
        """複刻加上閃鎖之後的終結述詞。"""
        if not was_airborne:
            return False
        return agl <= kill_agl and n_lo >= confirm

    def test_rep1_false_kill_is_blocked(self):
        """事故值：還沒爬過 50 m 就回報 agl=−4 ⇒ **不終結**。"""
        assert self.armed_kill(-4.0, 5, was_airborne=False) is False

    def test_genuine_touchdown_still_fires(self):
        """飛過之後才觸水 ⇒ 照樣終結（AP4/AP5 那一類）。"""
        assert self.armed_kill(-4.0, 5, was_airborne=True) is True

    def test_latch_is_not_a_timer(self):
        """⚠️ 不可以用「起飛後 N 秒」這種會隨機體/風況漂的常數。"""
        m = re.search(r"def _kill_arm_update\(self\):(.*?)\n    def ", NODE_SRC, re.S)
        assert m, "找不到 _kill_arm_update"
        body = m.group(1)
        assert "self.kill_arm_agl" in body
        for bad in ("time.time() -", "elapsed", "起飛後"):
            assert bad not in body.split('"""')[-1], "閃鎖裡出現計時：%s" % bad

    def test_gate_is_before_the_counter(self):
        """⚠️ 要在計數**之前** —— 不然保險解除的當下計數已滿，等於沒擋。"""
        m = re.search(r"def _kill_airframe\(self, i, agl, now\):(.*?)\n    def ",
                      NODE_SRC, re.S)
        body = m.group(1)
        assert body.index("_kill_armed") < body.index("self._kill_lo[i] = self._kill_lo.get")

    def test_updated_in_both_phases(self):
        """飛機在 STANDBY 就已起飛爬升 ⇒ `_row` 與 `_standby_tick` 兩邊都要叫。"""
        assert NODE_SRC.count("self._kill_arm_update()") >= 2

    def test_standby_call_is_before_the_1hz_throttle(self):
        """1 Hz 節流之後才叫的話，爬升快時可能漏採。"""
        i = NODE_SRC.index("def _standby_tick(self):")
        body = NODE_SRC[i:i + 2600]
        assert body.index("_kill_arm_update()") < body.index("self._standby_stat_t < 1.0")

    def test_default_threshold_is_well_above_ekf_noise(self):
        """事故時回報 −36 m；門檻要遠高於那個量級。"""
        m = re.search(r'g\("kill_arm_agl", ([\d.]+)\)', NODE_SRC)
        assert m and 30.0 <= float(m.group(1)) <= 200.0

    def test_stub_default_never_arms(self):
        """⚠️ class 層預設要是**永遠解除不了** —— 樁節點不該意外終結機體。"""
        assert re.search(r"^    kill_arm_agl = 1e9$", NODE_SRC, re.M)

    def test_refusal_is_not_silent(self):
        """「不終結」也要出聲，否則下一個人會以為功能壞了。"""
        assert "但**不終結**" in NODE_SRC
        assert "REP1" in NODE_SRC


class TestHitAltSharesTheAglDatum:
    """⚠️⚠️ 命中高度鎖存與 CSV 的 agl 欄**必須同一個基準**。

    2026-08-07 把 `_agl` 改到 AMSL 空間（對 HOME 重設免疫），
    但 `hit_alt` 留在 `pos - _zg` 的 HOME 相對基準 ⇒ 兩套基準惄惄分岔。

    後果不是小數點：HOME 重設實測平移 **32.3 m**，
    而 `impact_alt_ok` 門檻只有 **30 m** ⇒
      * 一次真落地會被寫成「高空掠過」
      * 反向的 HOME 跳變會把 40 m 的掠過寫成「全員命中」
    而且 `_metrics_frozen` 之後**永不更新** —— 整趣的驗收結論就此定案。

    ⚠️ AP3/AP5 沒被污染是**運氣**：兩趣的 HOME 重設都發生在
       命中之後約 1 秒（撞海觸發的），不是設計上擋得住。
    """

    def test_hit_alt_uses_the_helper(self):
        assert "self.hit_alt[i] = self._agl(i)" in NODE_SRC

    def test_old_home_relative_form_is_gone(self):
        assert "self.hit_alt[i] = self.pos[i][2] - self._zg(i)" not in NODE_SRC

    def test_csv_and_hit_alt_are_the_same_call(self):
        """兩邊都走 `_agl(i)` ⇒ 不可能再分岔。"""
        assert "agl = self._agl(i)" in NODE_SRC
        assert NODE_SRC.count("self._agl(i)") >= 2

    def test_the_stale_comment_was_corrected(self):
        """那句「agl … 和 log 的『命中高度』同基準」曾經不成立。"""
        assert "2026-08-10 更正" in NODE_SRC
        assert "不要在只改其中一邊的情況下把這句留著" in NODE_SRC

    def test_threshold_is_smaller_than_the_measured_home_jump(self):
        """這就是為什麼它會翻盤：平移 32.3 m > 門檻 30 m。"""
        m = re.search(r'g\("impact_alt_ok",\s*([\d.]+)\)', NODE_SRC)
        assert m, "找不到 impact_alt_ok"
        assert float(m.group(1)) < 32.3

    def test_latch_is_still_frozen_once(self):
        """修基準不可以順手把「只定案一次」的性質改掉。"""
        assert "_metrics_frozen" in NODE_SRC
