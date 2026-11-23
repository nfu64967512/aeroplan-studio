# -*- coding: utf-8 -*-
"""重複航次腳本的就緒閘門 —— **不可以拿系統做不到的事當門檻**。

═══ 2026-08-10,同一個錯我連犯三次 ═══

| 版本 | 我設的門檻 | 為什麼永遠達不到 |
|---|---|---|
| REP1 | `FIRE_DELAY=25s` 固定延遲 | 沒有門檻,25 s 時 EKF 還沒收斂 ⇒ 兩機 6 s 內被誤殺 |
| RPB1 | `rel_alt ≥ 120 m` | 編隊層 `patrol_alt=80 m`,飛不到 |
| RPC1 | `rel_alt ≥ 60 m`  | **編隊層根本爬不上去**(見下),多低都一樣 |

實測(measure_climb,起飛後逐 5 s 量 180 s,桃園跑道 +32 m AMSL):

    t(s)   UAV1_rel   UAV2_rel   UAV1_amsl   UAV2_amsl
       0     65.787     -2.003      105.27       47.52
      15    -24.891    -19.501       23.90       19.69
      60    -17.658    -22.113       32.83       29.46
     180     -8.265    -14.664       42.22       36.33

三分鐘下來兩機一直在跑道高度上下浮沉。真因有兩個,**都在編隊層**:

  (a) 編隊層下 NOMINAL 14 / WAIT 10 / CATCH 20 m/s,而噴射機的
      `AIRSPEED_MIN` 是 **25** ⇒ 每一發 `DO_CHANGE_SPEED` 都被 `MAV_RESULT=4`
      整發拒收。這是 catch-speed-rejected-by-fc 的**另一端**(那次太快,這次太慢)。
  (b) backup/wingman 以 1 Hz 重發 `DO_REPOSITION`,而每一發都會
      `set_target_altitude_current()` 把高度目標重錨在當下高度 ⇒ 高度需求
      永遠累積不起來。tot_node 的 `_goto` 用航點鎖存修過同一個病,編隊層沒有。

⇒ 正確的門檻量的是「**高度估計可不可信**」,不是「飛多高」。
   REP1 的壞輸入特徵是**劇烈跳動**(rel_alt −36 m 而地速 106 m/s),不是「低」。
"""
import pathlib
import re

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
W = ROOT / "scripts" / "wsl"
REP = (W / "rep_interactive.sh").read_text(encoding="utf-8")
UP = (W / "swarm_ros2_up.sh").read_text(encoding="utf-8")
SITL = (W / "swarm_sitl_up.sh").read_text(encoding="utf-8")
MASTER = (W / "sar_swarm_patches" / "master_node.py").read_text(encoding="utf-8")


def _sh_default(src, var):
    m = re.search(r'%s="\$\{%s:-([^}]*)\}"' % (var, var), src)
    assert m, "找不到 %s" % var
    return m.group(1)


def _sh_default_py(src, name, default=None):
    """讀 `self.declare_parameter("name", <數字>)` 的預設值。"""
    m = re.search(r'declare_parameter\(\s*"%s"\s*,\s*([0-9.]+)' % name, src)
    if m is None:
        assert default is not None, "找不到 declare_parameter(%r)" % name
        return default
    return float(m.group(1))


def _formation_hold_alt():
    """編隊層實際會把飛機**壓在**哪個高度 —— 閘門門檻的天花板。

    ⚠️ 取長機巡邏高與僚機編隊高的**最小值**:兩者預設不同
    (`patrol_alt` 走 `${FORM_ALT:-80.0}`,`formation_alt` 走 `${FORM_ALT:-50.0}`),
    而閘門要求的是**全員**都過門檻,所以受限於最低的那一個。
    實測 nodes.log:`UAV2 -> V-tail slot -> (…) @ 50 m`。
    """
    m_p = re.search(r'patrol_alt:="\$\{PATROL_ALT:-\$\{FORM_ALT:-([0-9.]+)\}\}"', UP)
    m_f = re.search(r'formation_alt:="\$\{FORM_ALT:-([0-9.]+)\}"', UP)
    assert m_p and m_f, "swarm_ros2_up.sh 的 patrol_alt / formation_alt 版面變了"
    return min(float(m_p.group(1)), float(m_f.group(1)))


def _gate_body():
    i = REP.index("_wait_stable()")
    return REP[i:REP.index("\nsource /opt/ros", i)]


class TestGateMeasuresTrustNotHeight:
    """⭐ 核心不變式:估計可不可信 **是必要條件**,任何高度門檻都不能取代它。

    ═══ 2026-08-10 補記:舊結論成立,但**不完整** ═══

    上面那張表(REP1/RPB1/RPC1)的結論是「不可以拿高度當門檻」。
    那在**它的條件下**是對的:當時 `JET_THRMAX=25` ⇒ THR_MAX=25 ⇒
    可用推力只有設計值的 1/4,飛機根本爬不動(見 thrmax-needs-climb-boost)。

    但只留「AMSL 穩定」這一個條件,在 RPG/RPH 兩批整個翻車:

        飛機**停在跑道上時 AMSL 當然穩定** ⇒ 閘門 4 秒就放行
        ⇒ 目標發出 ⇒ tot_node 用帶 CHANGE_MODE 的 DO_REPOSITION
          把飛機從 TAKEOFF 拉進 GUIDED ⇒ 地面上的 GUIDED 不會起飛
        ⇒ UAV1 gs_max=0.0 alt_max=0.1(樣本 6290 筆,遙測沒斷),整趟沒動

    ⇒ 現在是**兩個條件都要**:AMSL 穩定(估計可信)**且**連續 N 筆離地 ≥ 門檻。
      高度不再是「代替」可信度,而是「加在它後面」。

    為什麼這次高度門檻到得了(前三次到不了):
      爬升來自**飛控自己的 TAKEOFF 模式**,不是編隊層。
      `jet_fast.parm` 是 `TKOFF_ALT 300`,實測 RPH 的 UAV2 爬到 202.7 m。
    """

    def test_stability_parameters_exist(self):
        assert 'READY_BAND="' in REP
        assert 'READY_STABLE_N="' in REP

    def test_stability_is_still_required(self):
        """⭐ 高度門檻**不可以取代**穩定度 —— 兩者必須同時成立。"""
        body = _gate_body()
        assert "global_position/global" in body
        assert "^altitude:" in body, "AMSL 取樣不見了"
        assert "hi-lo <" in body, "極差判定不見了"

    def test_airborne_is_also_required(self):
        """擋住「又只剩穩定度」的回歸 —— 那正是 RPG/RPH 翻車的組態。"""
        body = _gate_body()
        assert "READY_AGL" in body, "閘門沒有離地條件,飛機停在跑道上也會放行"
        assert "rel_alt" in body, "離地高要走 rel_alt(相對 HOME),不要自己減地形"
        assert "airborne" in body

    def test_both_conditions_are_anded(self):
        """必須是 AND。任何一邊單獨成立都不可以放行。"""
        body = _gate_body()
        assert re.search(r'\[ "\$stable" = "1" \]\s*&&\s*\[ "\$airborne" = "1" \]', body), \
            "stable 與 airborne 沒有用 && 串起來"

    def test_airborne_needs_consecutive_samples_not_one_spike(self):
        """⚠️ 舊測試的反對理由:「不能拿一個壞量去判斷它自己好了沒」。
        回答方式是要求**連續 N 筆**都過門檻,單一尖峰不算。"""
        body = _gate_body()
        m = re.search(r'tail -n "\$READY_STABLE_N" "/tmp/rep_agl_', body)
        assert m, "離地判定只看最後一筆 —— 一個尖峰就能騙過閘門"

    def test_altitude_threshold_is_reachable_under_the_formation_layer(self):
        """⚠️⚠️ 門檻的天花板是**編隊層的巡邏高度**,不是 TKOFF_ALT。

        我一度設 250(理由是批次路徑 `jet_verify.py:220`「TKOFF_ALT 設 300,
        這裡等到 250」)—— **錯的**:批次路徑沒有編隊層,飛機可以一路在
        TAKEOFF 模式爬到 300;互動路徑的編隊層在 `guided_min_alt=15 m` 就
        接手 GUIDED,再把飛機帶去 `patrol_alt`(實際 50 m)。
        ⇒ 250 永遠到不了,必定逾時 —— 就是 RPB1/RPC1 那個「不可達門檻」。

        門檻必須夾在:接手高度(15) < READY_AGL < 巡邏高度(50)。
        """
        agl = float(_sh_default(REP, "READY_AGL"))
        guided_min = float(_sh_default_py(MASTER, "guided_min_alt", default=15.0))
        patrol = _formation_hold_alt()
        assert guided_min < agl < patrol, (
            f"READY_AGL={agl} 不在 (接手 {guided_min}, 巡邏 {patrol}) 之間 —— "
            "低於接手高度等於沒門檻,高於巡邏高度則永遠到不了"
        )

    def test_timeout_allows_time_to_actually_climb(self):
        """門檻從秒級(AMSL 穩定)變成分鐘級(爬到 250 m),逾時要跟著放大。"""
        assert float(_sh_default(REP, "READY_MAX_S")) >= 240.0

    def test_band_is_tighter_than_the_measured_ekf_swing(self):
        """實測 EKF 未收斂時 AMSL 在 19.7–105.3 之間跳(≈85 m);
        門檻要遠小於它才擋得住,但也不能小到永遠等不到。"""
        band = float(_sh_default(REP, "READY_BAND"))
        assert 5.0 <= band <= 30.0

    def test_looks_at_a_contiguous_tail(self):
        """⚠️ 「穩定 N 筆」必須是**連續的最後 N 筆**,不可以從歷史裡挑。
        用 `tail -n N` 讀常駐 echo 寫下的檔案 ⇒ 天然就是連續尾段;
        中間若有跳變,它一定落在視窗內、極差就超標。"""
        # ⚠️ 不要用固定長度切片:函式一長,要驗的東西就掉出視窗而**假性失敗**
        #    (2026-08-10 加離地條件時就撞到)。用函式邊界切。
        body = _gate_body()
        assert 'tail -n "$READY_STABLE_N"' in body
        assert "NR>=n" in body, "沒有要求視窗真的收滿 N 筆"


class TestTheLessonIsRecorded:
    def test_all_three_failed_attempts_are_written_down(self):
        """同一個錯犯三次,值得留在腳本裡。"""
        for needle in ("RPB1", "RPC1", "120 m", "60 m"):
            assert needle in REP, needle

    def test_root_cause_of_no_climb_is_recorded(self):
        assert "AIRSPEED_MIN" in REP
        assert "MAV_RESULT=4" in REP
        assert "set_target_altitude_current" in REP

    def test_measured_table_is_kept(self):
        """把實測數字留著 —— 下一個人不用再花 3 分鐘量一次。"""
        assert "105.27" in REP and "42.22" in REP


class TestFormationLayerSpeedMismatchIsReal:
    """這一條不是測腳本,是把**發現本身**鎖住:編隊層的速度低於飛控下限。"""

    def test_formation_speeds_are_below_the_jet_floor(self):
        nominal = float(_sh_default(UP, "NOMINAL_SPEED"))
        wait = float(_sh_default(UP, "WAIT_SPEED"))
        catch = float(_sh_default(UP, "CATCH_SPEED"))
        m = re.search(r"^AIRSPEED_MIN (\d+)", SITL, re.M)
        assert m, "找不到 jet_fast.parm 的 AIRSPEED_MIN"
        floor = float(m.group(1))
        below = [n for n, v in
                 (("NOMINAL", nominal), ("WAIT", wait), ("CATCH", catch))
                 if v < floor]
        assert below, "如果哪天全部都高於下限了,這條測試要改成正向鎖"
        # ⚠️ 這是**現況紀錄**,不是「應該如此」。修法要等受控實測。
        assert set(below) == {"NOMINAL", "WAIT", "CATCH"}, (
            "編隊層速度與飛控下限的關係變了,請重新評估 %s" % below)

    def test_patrol_alt_is_unreachable_in_practice(self):
        """編隊層命令 patrol_alt 80 m,但實測三分鐘只在跑道高度浮沉。"""
        m = re.search(r'declare_parameter\("patrol_alt",\s*([\d.]+)\)', MASTER)
        assert m and float(m.group(1)) == 80.0


class TestSingleVariableDiscipline:
    def test_approach_from_is_explicit(self):
        """⚠️ 不給的話走 auto_approach(由蜂群質心反推),每趟不一樣。"""
        assert "TGT_APPROACH" in REP
        assert '\\"approach_from\\": $TGT_APPROACH' in REP

    def test_amsl_is_pinned(self):
        assert '\\"amsl\\": $TGT_AMSL' in REP

    def test_failures_are_recorded_not_dropped(self):
        """⚠️ 只有失敗會消失的話,成功率會被灌高。"""
        assert REP.count("記為失敗") >= 4
        assert 'tee -a "$SUMMARY"' in REP

    def test_jet_conversion_is_explicit(self):
        assert "export JET=1" in REP and "JET_CDP" in REP


class TestNoDanglingReferences:
    """⚠️⚠️ RPD 實測：我把函式從 `_wait_airborne` 改名成 `_wait_stable`，
    **呼叫點忘了改**，而且訊息還引用已刪除的 `$READY_ALT`。
    症狀是 `等兩機離地(rel_alt ≥ ⟨空⟩ m)` 然後每趣直接記失敗。

    `bash -n` **只驗語法** —— 未定義的函式與空變數它都看不到。
    這一類造成的浪費是實際的：三趣 × 180 s。
    """

    def test_every_called_function_is_defined(self):
        import re as _re
        defined = set(_re.findall(r"^(\w+)\(\)\s*\{", REP, _re.M))
        # 取行首或 `if ! ` 之後的裸呼叫
        called = set(_re.findall(r"(?:^\s*|if !\s+)(_\w+)(?!\()", REP, _re.M))
        missing = {c for c in called if c.startswith("_wait") or c == "_down"} - defined
        assert not missing, "呼叫了不存在的函式：%s" % missing

    def test_removed_variable_is_not_referenced(self):
        """READY_ALT 已經拿掉，不可以還有**程式碼**引用它。

        ⚠️ 只看程式碼：註解裡本來就會提到 `$READY_ALT`（那是在解釋這個 bug），
           把說明文字也算成引用的話，這條測試會逼人把教訓刪掉。
        """
        code = "\n".join(ln.split("#", 1)[0] for ln in REP.splitlines())
        assert "READY_ALT" not in code, "程式碼仍引用已刪除的 READY_ALT"

    def test_runtime_selfchecks_exist(self):
        """静態鎖之外，腳本自己開跑前也要驗一次。"""
        assert "declare -F" in REP
        assert "內部錯誤:函式" in REP
        assert "內部錯誤:變數" in REP

    def test_selfcheck_runs_after_the_definitions(self):
        """⚠️ `declare -F` 放在定義之前的話永遠會失敗——我第一版就這樣。"""
        assert REP.index("_wait_ports() {") < REP.index("for _fn in")
        assert REP.index("_wait_stable() {") < REP.index("for _fn in")

    def test_selfcheck_runs_before_the_trial_loop(self):
        assert REP.index("for _fn in") < REP.index("for k in $(seq 1")


class TestThrMaxIsCoupledToClimbBoost:
    """⚠️⚠️⚠️ RPE1/RPE2：我把批次路徑的 `JET_THRMAX=25` 照抄過來，
    兩機起飛爬到 53/54 m 之後 **40 秒內沉回地面**。

        RPE2 t=43 s：UAV1 地速 11.5、UAV2 地速 0.1  —— 都在地上

    `swarm_sitl_up.sh:77-78` 原文：「**servo 上限就是 THR_MAX**
    ⇒ THR_MAX=19 等於可用推力只剩 19%。」
    25 是**巡航設計值**；批次路徑能用它，是因為它有
    `jet_catapult_launch.py` 在爬升段把 THR_MAX 拉到 35（CLIMB_THR_MAX）。
    **互動路徑沒有那一段。**

    ⭐ 順帶修正一個認知：AP3/AP5 飛得起來，是因為那次的臨時
       bring-up **漏了 JET_THRMAX** ⇒ THR_MAX 停在原廠的 100。
       也就是說 **AP5 不是在完整的噴射組態下飛出來的**。
    """

    def test_thrmax_is_not_set_by_the_repeat_script(self):
        code = "\n".join(ln.split("#", 1)[0] for ln in REP.splitlines())
        assert "export JET_THRMAX" not in code, (
            "又把巡航用的 THR_MAX 照抄過來了；"
            "互動路徑沒有爬升段的推力提升")
        assert "unset JET_THRMAX" in code

    def test_the_coupling_is_explained(self):
        assert "CLIMB_THR_MAX" in REP
        assert "jet_catapult_launch.py" in REP
        assert "servo 上限就是 THR_MAX" in REP

    def test_measured_sink_is_recorded(self):
        assert "RPE1/RPE2" in REP or "RPE2" in REP
        assert "地速 11.5" in REP

    def test_ap5_caveat_is_recorded(self):
        """不可以讓人以為 AP5 驗證過完整噴射組態。"""
        assert "THR_MAX=100" in REP or "THR_MAX 停在" in REP

    def test_climb_boost_exists_only_on_the_batch_path(self):
        west = (W / "run_west.sh").read_text(encoding="utf-8")
        assert "CLIMB_THR_MAX" in west
        up = (W / "swarm_ros2_up.sh").read_text(encoding="utf-8")
        assert "CLIMB_THR_MAX" not in up, (
            "互動路徑現在有爬升推力提升了？"
            "那 JET_THRMAX 的限制要重新評估")


class TestGateIsFastEnough:
    """⚠️ **閘門本身的延遲也是一種錯誤來源。**
    第一版每輪對每機各發 2 次 `ros2 topic echo --once`（每次 ~2 s）
    ⇒ 4 輪 ~40 s，而飛機在 ~40 s 內就沉下去了。
    """

    def test_uses_a_persistent_echo_not_per_round_once(self):
        """⚠️ 只看**程式碼** —— 註解裡本來就會提到 `--once`(那是在解釋舊寫法)。"""
        # ⚠️ 不要用固定長度切片:函式一長,要驗的東西就掉出視窗而**假性失敗**
        #    (2026-08-10 加離地條件時就撞到)。用函式邊界切。
        body = _gate_body()
        code = "\n".join(ln.split("#", 1)[0] for ln in body.splitlines())
        assert "--once" not in code, "又回到每輪重啟 python 的寫法"
        assert "ros2 topic echo" in code
        assert "fflush()" in code, "沒有 flush 的話樣本會卡在管線緩衝區"

    def test_cleans_up_the_background_readers(self):
        # ⚠️ 不要用固定長度切片:函式一長,要驗的東西就掉出視窗而**假性失敗**
        #    (2026-08-10 加離地條件時就撞到)。用函式邊界切。
        body = _gate_body()
        assert 'kill "$p"' in body, "背景 echo 沒收掉會累積"
