# -*- coding: utf-8 -*-
"""2026-08-14 大整改的通路與預設值鎖(S3–S8)。

═══ 這一檔要擋的是什麼 ═══

⚠️⚠️ **有參數沒有 `-p` 通路 = 純 no-op,而且零告警。**

前例的代價很具體:`pn_los_floor` 與 `pn_nz_min` 是「陡俯角唯一有效的開關」,
`run_west.sh` 給 1.0 / −1.0,但 `swarm_ros2_up.sh` **整份沒有這兩個 `-p`** ⇒
互動路徑 24 架次全部在關閉狀態下飛,而唯一達標的 TWO1 走的是另一條路徑。
於是「TWO1 與 RPK/RPL 同組態卻結果不同」這個比較從頭到尾就是無效的。

同族還有 `gate_*`(全 repo 只有 `stot_trial.sh` 有通路)。

⇒ 這一檔對本次新增的每一個參數,逐項驗證三件事:
   1. 節點宣告了它
   2. **啟動腳本有 `-p` 通路**
   3. 預設值 = 現行行為(否則「全部預設不變」這個承諾是假的)

═══ 為什麼「預設 = 現行行為」是硬性要求 ═══

2026-08-13 一次改了 `GATE_R` / `hold_radius` / `formation_alt` 三項,
結果真落地 5/12 → 1/6,而且**分不出是哪一項造成的**。
這一輪十二個參數若不是全部預設關閉,同樣的事會以十二倍的規模重演。
"""
import ast
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOT = ROOT / "scripts" / "wsl" / "sar_swarm_patches" / "tot_node.py"
UP = ROOT / "scripts" / "wsl" / "swarm_ros2_up.sh"
SRC = TOT.read_text(encoding="utf-8")
UPSRC = UP.read_text(encoding="utf-8")
TREE = ast.parse(SRC, filename=str(TOT))

#: (節點參數名, 預設值, 這個預設代表的現行行為)
NEW_PARAMS = [
    ("pn_alpha_tau", 1.75, "(1/50)/0.7 ≡ 0.05/1.75,α 濾波增益逐位元不變"),
    ("tecs_pitch_min", 0.0, "0 = 跟隨 dive_pitch_min,兩個俯仰限制仍綁在一起"),
    ("state_max_age", 0.0, "0 = 不做狀態新鮮度檢查"),
    ("att_require_fc", False, "False = 仍允許靜默降級到 6860 分流線"),
    ("param_verify", 0, "0 = 不做參數讀回驗證"),
    ("att_handback", False, "False = 釋放姿態時不做交棒重錨"),
    ("att_eas_floor", 0.0, "0 = 無 EAS 地板"),
    ("att_agl_floor", 0.0, "0 = 無 AGL 地板"),
    ("stall_prevention_glide", 0, "0 = 不動 .parm 的 STALL_PREVENTION"),
    ("att_require_guided", False, "False = 不檢查飛控模式"),
    ("hold_alt", 0.0, "0 = 等待段仍用 perch_of()"),
    ("hold_v", 0.0, "0 = 等待段仍用 vmin"),
    ("tgo_from_predictor", False, "False = t_go 仍用靜態幾何式 d/_pace_of"),
    ("tgo_predict_dt", 0.5, "積分節流;不影響是否啟用"),
    ("weave_time_sync", False, "False = 等待段不做蛇形(現行行為)"),
    ("weave_theta_max", 45.0, "上限;實際還受換向率夾限"),
    ("weave_deadband_s", 0.5, "死區,避免在雜訊上抖動"),
]


def _declared(name):
    for node in ast.walk(TREE):
        if not isinstance(node, ast.Call) or len(node.args) < 2:
            continue
        fn = getattr(node.func, "id", None) or getattr(node.func, "attr", None)
        if fn not in ("g", "declare_parameter"):
            continue
        k = node.args[0]
        if isinstance(k, ast.Constant) and k.value == name:
            v = node.args[1]
            if isinstance(v, ast.Constant):
                return v.value
    return None


class TestEveryNewParameterIsDeclared:
    @pytest.mark.parametrize("name,default,why", NEW_PARAMS,
                             ids=[p[0] for p in NEW_PARAMS])
    def test_declared(self, name, default, why):
        got = _declared(name)
        assert got is not None, f"tot_node 沒有宣告參數 {name}"


class TestEveryNewParameterHasAPlumbingPath:
    """⭐⭐ 本檔最重要的一組。

    ⚠️ 宣告了但沒有 `-p` = 節點吃宣告值 = **這個參數在產品路徑上不存在**,
       而且 log 不會有任何差別。`pn_los_floor` 就這樣白費了 24 架次。
    """

    @pytest.mark.parametrize("name,default,why", NEW_PARAMS,
                             ids=[p[0] for p in NEW_PARAMS])
    def test_has_ros_args_path(self, name, default, why):
        assert f"-p {name}:=" in UPSRC, (
            f"swarm_ros2_up.sh 沒有 `-p {name}:=` ⇒ 在互動/產品路徑上這個參數"
            f"是純 no-op,而且不會有任何訊息告訴你")

    GATE_PARAMS = ["gate_theta_max", "gate_phi", "gate_r_min",
                   "gate_r_max", "gate_hold_n"]

    @pytest.mark.parametrize("name", GATE_PARAMS)
    def test_the_dynamic_gate_switches_now_have_a_path(self, name):
        """⭐ 2026-08-14 補上的通路。

        ⚠️⚠️ 先前這五個**整份 swarm_ros2_up.sh 都沒有 `-p`** ⇒ 互動/產品路徑上
        `export GATE_THETA_MAX=35` 是純 no-op,節點照吃宣告值 0.0,
        而 log 仍印「(絕對 gate_r)」—— **開與沒開長得一模一樣**。
        全 repo 只有 `stot_trial.sh` 有,所以只有批次路徑用得到動態交棒。

        差一點就據此得出「動態交棒試過了,沒用」的錯誤結論。
        """
        assert f"-p {name}:=" in UPSRC, f"swarm_ros2_up.sh 缺 -p {name}:="

    def test_dynamic_gate_still_defaults_to_off(self):
        """⚠️ 補通路 ≠ 打開。打開是行為改動,要單獨一趟驗。"""
        assert _declared("gate_theta_max") == 0.0
        import re
        m = re.search(r'TOT_GATE_THETA_MAX=.*?:-([0-9.]+)', UPSRC)
        assert m and float(m.group(1)) == 0.0, "腳本把動態交棒預設打開了"


class TestDefaultsPreserveCurrentBehaviour:
    """⚠️⚠️ 「全部預設 = 現行行為」不是口號,是可驗證的斷言。

    2026-08-13 一次改三項導致無法歸因;這一輪有十二項,
    若不是全部預設關閉,同樣的事會以十二倍規模重演。
    """

    @pytest.mark.parametrize("name,default,why", NEW_PARAMS,
                             ids=[p[0] for p in NEW_PARAMS])
    def test_default_is_the_no_op_value(self, name, default, why):
        got = _declared(name)
        assert got == default or float(got) == float(default), (
            f"{name} 預設是 {got},應為 {default} —— {why}")

    @pytest.mark.parametrize("name,default,why", NEW_PARAMS,
                             ids=[p[0] for p in NEW_PARAMS])
    def test_shell_default_matches_the_node_default(self, name, default, why):
        """⚠️ 節點預設關閉、但腳本預設打開,等於「預設不變」是假的。
        兩邊都要是 no-op 值。"""
        import re
        env = re.search(rf'-p {name}:="\$(TOT_[A-Z_]+)"', UPSRC)
        assert env, f"抓不到 {name} 的 TOT_ 變數名"
        m = re.search(rf'{env.group(1)}=.*?:-([A-Za-z0-9.]+)', UPSRC)
        assert m, f"抓不到 {env.group(1)} 的預設值"
        v = m.group(1)
        if isinstance(default, bool):
            assert v == str(default), f"{name} 腳本預設 {v} ≠ 節點預設 {default}"
        else:
            assert float(v) == float(default), \
                f"{name} 腳本預設 {v} ≠ 節點預設 {default}"


class TestHoldAltIsDecoupledFromRingRadius:
    """⭐ S8 的核心設計決定,值得單獨鎖。

    ⚠️⚠️ **不能靠調低 `perch_alt` 來降低交接高度。**
    `auto_ring` 讓環半徑 = perch/tan(dive_angle),等比例縮小 ⇒
    γ = atan(perch/gate) **完全不變**。那是這個幾何的陷阱,而且很反直覺。

        perch 3100 → 環 6084、gate 上界 4784、γ_min 32.9°
        perch 2663 → 環 5226、gate 上界 3926、γ_min 34.2°  ← 反而更糟

    ⇒ 必須把「等待高度」與「環半徑」解耦:環仍由 perch_alt 定,等待飛更低。
    """

    def test_hold_alt_helper_exists_and_ring_does_not_use_it(self):
        assert "def hold_alt_of" in SRC, "S8 的 hold_alt_of() 不見了"
        # ring_of 必須仍然只吃 perch_of —— 用到 hold_alt 就會退回上面那個陷阱
        i = SRC.index("def ring_of")
        body = SRC[i:i + 700]
        assert "hold_alt" not in body, \
            "ring_of() 用到了 hold_alt ⇒ 環半徑會跟著縮 ⇒ γ 不變,整個 S8 白做"
        assert "perch_of" in body

    def test_hold_alt_never_raises_above_perch(self):
        """⚠️ 只准比 perch 低。高於它會把 perch_step 的高度階梯抹平,
        而那個階梯是「把同時性從油門搬到幾何」的既有機制。"""
        i = SRC.index("def hold_alt_of")
        body = SRC[i:i + 900]
        assert "min(" in body and "perch_of" in body, \
            "hold_alt_of 沒有用 min(perch_of, …) 夾住上界"

    def test_hold_v_is_clamped_into_the_flight_controller_envelope(self):
        """⚠️ 低於 vmin 會被飛控**整發拒收**(MAV_RESULT=4),不是夾住。"""
        i = SRC.index("def hold_v_of")
        body = SRC[i:i + 800]
        assert "self.vmin" in body and "self.vmax" in body, \
            "hold_v_of 沒有夾進 [vmin, vmax] ⇒ 指令會被飛控整發拒收"


class TestAttitudeGateFailsSafe:
    """S4/S5/S6 的兜底閘門。"""

    def test_gate_exists_and_is_consulted_before_sending(self):
        assert "def _att_gate" in SRC
        i = SRC.index("def _att_target")
        j = SRC.index("set_attitude_target_send", i)
        assert "_att_gate" in SRC[i:j], \
            "_att_target 在送出前沒有先問 _att_gate ⇒ 兜底形同虛設"

    def test_blocking_stops_sending_rather_than_sending_something_else(self):
        """⚠️⚠️ 擋下時必須**停發**,不是改送別的姿態。

        停發才會讓 GUIDED_TIMEOUT 觸發、把控制權還給飛控;
        繼續送(哪怕是「安全的」姿態)會讓逾時永遠不到,
        那正是 leader-state-freeze 記的「凍結沒有告警」那個形狀。
        """
        i = SRC.index("def _att_target")
        seg = SRC[i:i + 1400]
        assert "return" in seg.split("set_attitude_target_send")[0], \
            "_att_gate 擋下後沒有 return ⇒ 還是會送出去"

    def test_no_silent_fallback_when_required(self):
        """⚠️ 先前是 `self.fc.get(i) or self.link[i]` —— fc 缺席就靜默降級。"""
        i = SRC.index("def _att_target")
        seg = SRC[i:i + 1600]
        assert "att_require_fc" in seg, "_att_target 沒有引用 att_require_fc"

    def test_stale_state_uses_a_monotonic_clock(self):
        """⚠️ 牆鐘會被 NTP 拉動,新鮮度判斷必須用單調時鐘。"""
        assert "self.state_t[i] = time.monotonic()" in SRC


class TestNewStateIsResetBetweenTargets:
    """⚠️ 第二發目標不可以帶著第一發的累積。RESET_DICTS 有一條手寫對照鎖。"""

    @pytest.mark.parametrize("name", ["att_block", "att_block_n", "handback_t"])
    def test_in_reset_manifest(self, name):
        i = SRC.index("RESET_DICTS = (")
        manifest = SRC[i:SRC.index(")", i)]
        assert f'"{name}"' in manifest, f"{name} 沒進 RESET_DICTS"


class TestDynamicGateCannotLandInTheLoiterSwing:
    """⭐⭐ 對抗式驗證抓出來的致命問題,修在 `gate_of()` 裡。

    ⚠️⚠️ 直接打開 `gate_theta_max` **會重演 RPO 的 1/6**:
    算出來的動態 gate 落進守環擺盪帶 `[環 − hold_radius, 環 + hold_radius]`,
    於是放行當下飛機可能已在 gate 內側 ⇒ 當場交棒 ⇒ 交棒距離變成隨機量。

        RPO 實測(gate 5600 / hold 1300 / 環 6147):
            交棒距 ±11 m → ±1075 m,真落地 5/12 → 1/6

    ⚠️ **靜態** gate_r 由 contract_table 的同名不變式在起飛前擋;
       **動態** gate 每拍現算,契約層看不到 ⇒ 只能在 gate_of() 裡夾。
       少了這一道,補通路等於把已知會失敗的組態變得可以一鍵啟用。
    """

    def test_gate_of_clamps_against_the_swing(self):
        i = SRC.index("def gate_of")
        body = SRC[i:SRC.index("def _ring_points", i)]
        assert "hold_radius" in body, \
            "gate_of() 沒有引用 hold_radius ⇒ 動態 gate 可以落進守環擺盪帶"
        assert "swing_inner" in body, "缺少擺盪內緣的夾限"

    def test_the_clamp_is_applied_to_the_return_value(self):
        i = SRC.index("def gate_of")
        body = SRC[i:SRC.index("def _ring_points", i)]
        tail = body[body.rindex("return"):]
        assert "cap" in tail, "夾限算了但沒有用在回傳值上"

    def test_the_lower_bound_still_wins(self):
        """⚠️ 擺盪內緣可能比 gate_r_min 還小(環很小或 hold_radius 很大時)。
        那種情況下不可以讓 gate 掉到 gate_r_min 以下 —— PN 擁有的航程會不足。"""
        i = SRC.index("def gate_of")
        body = SRC[i:SRC.index("def _ring_points", i)]
        assert "swing_inner > self.gate_r_min" in body, \
            "沒有處理『擺盪內緣 < gate_r_min』的情況"


class TestHoldAltCoversEveryHoldingSegment:
    """⚠️⚠️ RPU 三趟燒出來的:`hold_alt` 只改一半等於沒改。

    第一版只把 `hold_ring` 與 `wait` 改成吃 `hold_alt_of()`,**漏了 `runup`**,
    而 runup 是三段裡最長的(實測 191–213 s):

        climb 爬到 2661–2832
        runup **再爬 +439~+894** → 3155–3719      ← 上游反向做功
        hold_ring 只剩 25–41 s 把它降回 2732–2772

    ⇒ 降多少取決於分到幾秒(逐機不同)⇒ 交接高度散布 916 m 毫無收斂。
      `hold_alt` 本身有效(每架確實降了 424–944 m),但被上游抵消。

    ⇒ 「等待高度」的控制必須覆蓋**每一個持有高度的段**,漏一段就前功盡棄。
    """

    #: (原始碼裡的**實際**錨點字串, 該段說明)。
    #: ⚠️ 不可以用 `mode = "<name>"` 一律組出來:`wait` 那一行寫的是
    #:    `mode = "runin" if far else "wait"`,組字串會抓不到 —— 第一版就這樣
    #:    讓一條測試看起來在檢查、實際上找錯位置。
    HOLDING_ANCHORS = [
        ('mode = "hold_ring"', "抵環後守在自己的環點等人"),
        ('mode = "runup"', "全員就緒後同步內切 —— 三段裡最長(實測 191–213 s)"),
        ('mode = "runin" if far else "wait"', "已在環上等待"),
    ]

    @pytest.mark.parametrize("anchor,why", HOLDING_ANCHORS,
                             ids=[a[0][:22] for a in HOLDING_ANCHORS])
    def test_segment_uses_hold_alt(self, anchor, why):
        i = SRC.index(anchor)
        # ⚠️ 視窗要夠寬:wait 段在 T3 之後多了蛇形分支,_goto 被推遠。
        #    視窗太窄會讓這條測試在**程式其實是對的**時候變紅 —— 假紅同樣有害,
        #    它會訓練人去忽略這條測試。
        seg = SRC[max(0, i - 200):i + 2000]
        assert "hold_alt_of" in seg, (
            f"{anchor}({why})沒有用 hold_alt_of() ⇒ 會用 perch_of() 把飛機"
            f"拉回 perch,抵消其他段的下降(RPU 實測 runup 反向爬升 439–894 m)")

    def test_climb_deliberately_does_not(self):
        """⚠️ 反向鎖:`climb` 段**不可以**吃 hold_alt ——
        那一段的任務就是爬到高度,在那裡壓低會讓「爬完了沒」的判定失去意義。"""
        i = SRC.index('mode = "climb"')
        line = SRC[i:SRC.index("\n", i)]
        assert "hold_alt_of" not in line, "climb 段不該用 hold_alt_of"
        assert "perch_alt" in line or "perch_of" in line


class TestObservationIsIndependentOfActuation:
    """⚠️⚠️ RPX 一整趟驗證航次白跑換來的。

    第一版把 `e_t` 寫在 `_weave_theta_of()` 裡,而那個函式在
    `weave_time_sync=False` 時第一行就 return ⇒ **T3 關著時 e_t 整欄是空的**。
    於是 T1(換估計器)的驗證航次跑完之後,完全沒有資料可以判斷估計準不準。

    「先量再致動」是這一整套增量落地的前提。觀測若綁在致動開關上,
    那個前提就不存在了 —— 而且失敗方式是**安靜的**:CSV 有欄位、只是空的。
    """

    def test_observation_has_its_own_function(self):
        assert "def _observe_timing" in SRC, "T2 的獨立觀測函式不見了"

    def test_observation_does_not_check_the_actuation_switch(self):
        """⚠️ 用 AST 取**真正的程式碼**,不做字串比對。

        本檔今天已經因為「錨點抓到註解/docstring」而假紅兩次:
        docstring 裡刻意引用 `weave_time_sync` 說明「這裡不看它」,
        而 substring 檢查分不出說明與程式碼。
        """
        fn = next(n for n in ast.walk(TREE)
                  if isinstance(n, ast.FunctionDef) and n.name == "_observe_timing")
        names = {a.attr for a in ast.walk(fn) if isinstance(a, ast.Attribute)}
        assert "weave_time_sync" not in names, (
            "_observe_timing 的程式碼讀了致動開關 ⇒ 關掉 T3 就量不到 e_t ⇒ "
            "「先量再致動」失效(RPX 就是這樣白跑的)")
        assert {"e_t", "tgo_obs"} <= names, "_observe_timing 沒有記錄 e_t / tgo_obs"

    def test_observation_is_hooked_to_the_recording_path(self):
        """掛在 _row ⇒ 保證「有記錄就有觀測」,兩者時間對齊。"""
        i = SRC.index("def _row")
        assert "_observe_timing" in SRC[i:i + 900], \
            "_observe_timing 沒有掛在 _row ⇒ 記錄與觀測可能不同步"

    def test_the_timing_columns_exist(self):
        # ⚠️ 欄名必須帶方向:`early_by_s` 一眼看得出正負代表什麼,
        #    `e_t_s` 不行。方向搞反會讓閉迴路往錯的方向蛇形而且看起來只像「沒效」。
        for col in ("secs_to_target", "early_by_s", "weave_yaw_deg",
                    "pred_residual_alt_m", "pred_over_limit_frac"):
            assert f'"{col}"' in SRC, f"CSV 缺 {col} 欄"
