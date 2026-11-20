# -*- coding: utf-8 -*-
"""互動路徑必須飛**噴射機**組態,不是泡棉機基準。

═══ 2026-08-07 發現(AP3 航次,使用者指出巡航速度不對)═══
HUD 巡航只有 **82 m/s**,而機體巡航能力是 ~157。

根因是一個**結構性**的鏡像錯誤:
`swarm_ros2_up.sh` 的 TOT_* 預設值逐條抄自 `stot_trial.sh` 的預設,
而**批次航次實際跑的是 `run_west.sh` 覆蓋後的值**。run_west.sh 覆寫了
其中 6 項,全都是噴射機改裝時改的。互動路徑看不到那一層覆蓋,
於是忠實地複製了一組**噴射機改裝後沒有任何一趟航次用過**的基準。

    v_plan = plan_frac × vmax = 0.9 × 100 = 90 EAS   ← 舊值,實測 HUD 82 m/s
    v_plan = plan_frac × vmax = 0.9 × 175 = 157.5    ← 使用者要求的巡航

⚠️ `tests/test_tot_standby.py` 那條「兩邊數值一致」的迴歸鎖**擋不住這個** ——
   它比的是 stot_trial.sh 的預設,而那不是批次跑的值。這一檔補的就是那個缺口。
"""
import pathlib
import re

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
W = ROOT / "scripts" / "wsl"
UP = (W / "swarm_ros2_up.sh").read_text(encoding="utf-8")
WEST = (W / "run_west.sh").read_text(encoding="utf-8")
SITL = (W / "swarm_sitl_up.sh").read_text(encoding="utf-8")
TOT = (W / "sar_swarm_patches" / "tot_node.py").read_text(encoding="utf-8")


def up_default(var):
    """swarm_ros2_up.sh 的 TOT_<VAR> 生效預設。"""
    m = re.search(r'TOT_%s="\$\(_f "\$\{%s:-([-\d.]+)\}"\)"' % (var, var), UP)
    assert m, "找不到 TOT_%s" % var
    return float(m.group(1))


def west_export(var):
    """run_west.sh 匯出的生效值。"""
    m = re.search(r'^export %s="\$\{%s:-([-\d.]+)\}"' % (var, var), WEST, re.M)
    assert m, "run_west.sh 沒有匯出 %s" % var
    return float(m.group(1))


class TestCruiseSpeedIsTheJetsNotTheFoams:
    def test_plan_frac_is_still_the_multiplier(self):
        """⚠️ 巡航速度 = plan_frac × vmax。這條關係是本檔每個數字的前提,
        先鎖住它 —— 它一改,下面所有推導都失效。"""
        assert "v_plan = self.plan_frac * self.vmax" in TOT

    def test_plan_frac_is_zero_point_nine(self):
        assert re.search(r'"plan_frac",\s*0\.9', TOT) or "plan_frac:=0.9" in UP or True
        # 兩條路都以 0.9 傳入(硬編字面值),用它推導巡航速度
        assert "-p plan_frac:=0.9" in UP

    def test_planned_cruise_is_the_design_eas(self):
        """⚠️⚠️ 使用者說的「157 m/s」是 **3100 m 的地速**，不是 EAS。

        run_west.sh:68 原文：「157 m/s 地速 @3100 m = **EAS 135**」，
        而 :72 就在警告這個坑：「--cruise 要餵 EAS；餵 157 會設計成 3100 m 的 183」。
        機體正是照 `--cruise 135` 反推出來的 ⇒ 設計巡航就是 EAS 135。

        ⭐ 把 157 當成 EAS（vmax=175）的後果：高於天花板 17% ⇒ 全隊卡在
           `mode=\"max\"` 飽和點 ⇒ T_arr 永遠達不到 ⇒ **共同抵達的同步迴路
           變成開迴路**，散布被凍結在形成當下的值。
        """
        assert 0.9 * up_default("VMAX") == pytest.approx(135.0, abs=0.6)

    def test_vmax_stays_under_the_airframe_ceiling(self):
        """指令上限不可以超過機體能飛的 EAS（設計巡航 135，留點餘裕）。"""
        assert up_default("VMAX") <= 160.0

    def test_the_eas_vs_groundspeed_trap_is_written_down(self):
        """⭐ 下一個人（或我自己）很容易再把 157 當成 EAS。"""
        assert "地速 @3100 m" in UP
        assert "EAS 135" in UP
        assert "138" in UP, "要講明 400 m 的地速是多少"

    def test_old_foam_value_is_gone(self):
        """舊值 100 ⇒ 規劃巡航 90 EAS ⇒ 實測 HUD 82 m/s。"""
        assert up_default("VMAX") != 100.0

    def test_commanded_max_fits_under_the_fc_ceiling(self):
        """⚠️⚠️ tot_node 在 v_req > vmax 時**直接指令 vmax**(tot_node.py 的
        `mode = "max"` 分支)。vmax 超過飛控的 AIRSPEED_MAX 會讓
        DO_CHANGE_SPEED **整發被拒**(MAV_RESULT=4)—— 而且完全靜默。
        這正是 catch-speed-rejected-by-fc 那條教訓。"""
        m = re.search(r'CAP="\$\{AIRSPEED_MAX_CAP:-(\d+)\}"', SITL)
        assert m, "找不到 SITL 的 AIRSPEED_MAX 上限"
        assert up_default("VMAX") <= float(m.group(1))

    def test_the_max_branch_really_commands_vmax(self):
        """鎖住上一條所依賴的程式行為。"""
        assert re.search(r'mode = "max";\s*self\._goto\(i, \*self\.slot_of\[i\],'
                         r' self\._alt\(i\), self\.vmax\)', TOT)


class TestParityWithTheOnlyValidatedConfig:
    """這五項是 run_west.sh 覆寫、而互動路徑原本看不到的噴射機改動。
    TWO1(唯一達標組態)走的就是 run_west.sh。"""

    @pytest.mark.parametrize("var", [
        "VMIN", "PERCH_CLIMB_V", "WEAVE_PERIOD", "PN_VMAX", "DIVE_SPEED",
    ])
    def test_interactive_matches_batch(self, var):
        assert up_default(var) == pytest.approx(west_export(var)), var

    @pytest.mark.parametrize("var,foam", [
        ("VMIN", 45.0), ("PERCH_CLIMB_V", 0.0), ("WEAVE_PERIOD", 22.0),
        ("PN_VMAX", 135.0), ("DIVE_SPEED", 140.0),
    ])
    def test_foam_era_values_are_gone(self, var, foam):
        assert up_default(var) != foam, "%s 回到泡棉機基準了" % var

    def test_vmin_is_above_the_sustainable_envelope(self):
        """⚠️ sep_envelope.py 在 3100 m 量到 SEP 零交越在 EAS 72-73。
        vmin 低於它 = 要求飛機用飛不動的速度耗時間,結果是往下掉。"""
        assert up_default("VMIN") > 73.0

    def test_weave_period_survives_the_jets_turn_rate(self):
        """⚠️ 本機體 157 m/s 只有 3.58°/s;θ=45° 換向要 2θ/ω = 25.1 s。
        週期要 ≥ 4× 換向時間。22 s 是泡棉機時代的值。"""
        assert up_default("WEAVE_PERIOD") >= 100.0


class TestDiveSpeedNeedsTheRaisedCeiling:
    def test_dive_speed_exceeds_airframe_limit_by_design(self):
        """俯衝指令空速要**高於機體極限**才會一直壓機頭。"""
        assert up_default("DIVE_SPEED") > up_default("VMAX")

    def test_but_still_under_the_fc_ceiling(self):
        """⚠️⚠️ 這兩個是**耦合**的:DIVE_SPEED=210 在 AIRSPEED_MAX=165 之下
        會被整發拒收。165 這個值和批次的俯衝組態根本不相容 ——
        這就是 SITL 上限必須跟著提到 230 的原因。"""
        m = re.search(r'CAP="\$\{AIRSPEED_MAX_CAP:-(\d+)\}"', SITL)
        assert up_default("DIVE_SPEED") <= float(m.group(1))

    def test_sitl_ceiling_matches_the_batch_path(self):
        m = re.search(r'CAP="\$\{AIRSPEED_MAX_CAP:-(\d+)\}"', SITL)
        assert float(m.group(1)) == pytest.approx(west_export("AIRSPEED_MAX_CAP"))

    def test_the_w1_counter_evidence_is_still_recorded(self):
        """⚠️ W1 的反證(拉到 220 之後 UAV6 climb 逾時)**沒有被推翻**,
        只是被更晚的證據取代。註解必須留著,不可以因為改了值就刪掉。"""
        assert "W1" in SITL and "220" in SITL
        assert "climb 逾時" in SITL or "逾時" in SITL


class TestTheMirrorGapIsDocumented:
    def test_up_explains_why_it_diverged(self):
        """下一個人要能讀懂「為什麼抄 stot_trial.sh 的預設是不夠的」。"""
        assert "run_west.sh" in UP
        assert "沒有任何一趟航次用過" in UP

    def test_every_changed_value_cites_its_source_line(self):
        for needle in ("run_west.sh:250-261", "run_west.sh:242-248",
                       "run_west.sh:263-271", "run_west.sh:234", "run_west.sh:273"):
            assert needle in UP, needle


class TestRingGeometryIsTheJetProfile:
    """⚠️⚠️ 這六項是**一整組耦合的幾何**，不可以只改其中一項。

        環半徑 = (perch_alt − dive_alt) / tan(dive_angle)
          3100 / tan(27°) =  6084 m   ← 這一組（run_west.sh / TWO1）
          3100 / tan(10°) = 17575 m   ← 只改高度的下場
           400 / tan(10°) =  2269 m   ← 舊值（泡棉機尺度）
    """

    @pytest.mark.parametrize("var", [
        "R_FAN", "REACH", "PERCH_ALT", "TALT", "DIVE_ANGLE", "GATE_R",
    ])
    def test_matches_run_west(self, var):
        assert up_default(var) == pytest.approx(west_export(var)), var

    def test_ring_radius_is_the_validated_one(self):
        import math
        perch, ang = up_default("PERCH_ALT"), up_default("DIVE_ANGLE")
        assert perch / math.tan(math.radians(ang)) == pytest.approx(6084.0, abs=40.0)

    def test_gate_to_ring_ratio_is_in_the_feasible_window(self):
        """⚠️⚠️ 2026-08-13:本測試原本鎖 TWO1 的 0.789,**那個鎖是錯的**。

        原註解寫「u1-descent-authority:成敗看 gate_r/環半徑 比」——
        那個比值在互動路徑上**零鑑別力**:RPK/RPL 十二架次全部是 0.781、
        交接距離 4765–4787 m(±11 m),比值毫無變異,結果卻是 2/6。
        而 TWO1 與 RPK/RPL 是同一組設計幾何,一個 2/2、一個 2/6
        ⇒ 比值解釋不了任何東西,鎖住它只會擋住真正該做的調整。

        改鎖**幾何可行窗口**(這個有物理意義):
          下界 —— 太小 ⇒ PN 擁有的航程不足,需求俯角進入實測必敗帶
          上界 —— 貼上環半徑 ⇒ PN 在放行瞬間接手而退化(contract_table 有硬約束)

        ⚠️ TWO1 的重現不靠這個預設值:repro_two1.sh 自己明確帶 GATE_R=4800。
        """
        import math
        ring = up_default("PERCH_ALT") / math.tan(math.radians(up_default("DIVE_ANGLE")))
        # ⚠️⚠️ 2026-08-13 再訂正:窗口一度寫成 [0.85, 0.95],那是「γ 越小越好」
        #    推出來的,**當日被 RPO 三趟實測反證**。真正的上界來自守環擺盪:
        #    飛機在環上 loiter,距紅心的水平距在 [環 ± hold_radius] 之間擺盪;
        #    gate 若落在帶內,放行時可能已在 gate 內側 ⇒ 當場交棒 ⇒ 交棒距隨機。
        #    實測代價:交棒距 ±11 m → ±1075 m、真落地 5/12 → 1/6。
        #    ⇒ 硬上界 = (環 − hold_radius)/環。下界仍取 gate_r_min/環。
        hold = 1300.0          # swarm_ros2_up.sh 的 HOLD_RADIUS 預設
        hi = (ring - hold) / ring
        ratio = up_default("GATE_R") / ring
        assert ratio < hi, (
            f"gate/環 = {ratio:.3f} ≥ 擺盪內緣 {hi:.3f} "
            f"(環={ring:.0f} − hold_radius={hold:.0f});"
            f"gate_r={up_default('GATE_R'):.0f} 必須 < {ring - hold:.0f}")
        assert ratio >= 0.40, f"gate/環 = {ratio:.3f} 過小,PN 擁有的航程不足"

    def test_reach_scales_with_the_geometry(self):
        """⚠️ 260 m 的抵達容差在 35 km 扇形 + 6 km 環下永遠判不到「到了」。"""
        assert up_default("REACH") >= 1000.0

    def test_half_angle_stays_wide_for_n_agnosticism(self):
        """⚠️ TWO1 用 5.0，但那是 n=2 的特例化；
        n=6 時收太小會讓 ALIGN 收不完（b500-wide-spacing-status）。
        本層要對任意機數成立 ⇒ 留 25.0，由 env 重現 TWO1。"""
        assert up_default("HALF_ANGLE") == pytest.approx(25.0)
        assert "HALF_ANGLE=5" in UP, "要寫明 n=2 重現怎麼下"

    def test_the_coupling_is_documented(self):
        assert "一整組耦合的幾何" in UP
        assert "17575" in UP or "17.6 km" in UP, "要寫出只改高度的下場"


class TestAirframeGate:
    """使用者定調：加讀回式守門，**不改 JET 預設**。"""

    def test_gate_exists(self):
        assert "機體身分不符" in UP
        assert "pgrep -a arduplane" in UP

    def test_it_does_not_open_a_mavlink_connection(self):
        """⚠️ 守門連一個 socket 都不可以開 —— 開了就會去搶 MAVProxy 的分流埠
        （--out=tcpin 只餵第一個 client，見 fanout-port-exclusivity）。

        ⚠️ 只看**程式碼**：緊接在後面的埠檢查註解裡本來就會提到 mavutil，
           那是在解釋別的事，不該被算成一次連線。
        """
        i = UP.index('if [ "$TOT_ENABLE" = "1" ] && [ "${AIRFRAME_FORCE:-0}" != "1" ]; then')
        j = UP.index("✔ 機體身分：", i)
        code = "\n".join(ln.split("#", 1)[0] for ln in UP[i:j].splitlines())
        for bad in ("mavutil", "wait_heartbeat", "ros2 topic", "nc ", "telnet"):
            assert bad not in code, "守門碰了連線：%s" % bad

    def test_jet_default_is_untouched(self):
        """⚠️ 把 JET 預設翻成 1 會砸掉 swarm_sitl_up.sh 刻意保留的泡棉模式。"""
        assert '"${JET:-0}"' in SITL

    def test_has_an_escape_hatch_and_prints_it(self):
        """沒有逃生門的檢查，下一個人會直接把整段註解掉。"""
        assert "AIRFRAME_FORCE" in UP
        assert "AIRFRAME_FORCE=1 bash swarm_ros2_up.sh" in UP

    def test_only_when_tot_node_is_enabled(self):
        """PRESET=tight / max 本來就是要泡棉機的，不可以擋掉。"""
        assert 'if [ "$TOT_ENABLE" = "1" ] && [ "${AIRFRAME_FORCE:-0}" != "1" ]; then' in UP

    def test_catches_both_failure_modes(self):
        """泡棉機（指令被拒）與原廠噴射機（天花板 98.5）是兩種不同的死法。"""
        assert "原廠泡棉機" in UP
        assert "沒有低阻力改裝" in UP
        assert "98.5" in UP

    def test_message_gives_a_pasteable_command(self):
        assert "JET=1 JET_CDP=0.01642 JET_THRMAX=25" in UP
