# -*- coding: utf-8 -*-
"""節點端預測器 `sar_swarm_patches/predictor.py` 的迴歸鎖。

═══ 為什麼要逐點比對兩份實作 ═══

物理有兩份:
    scripts/research/predictor/sim.py       ← 用 RPK/RPL 十二架次校準並驗證過
    scripts/wsl/sar_swarm_patches/predictor.py  ← 節點要用的那一份

⚠️⚠️ 兩份分歧 = 有一邊改壞了,而且**症狀是「預測不準」而不是「程式壞掉」**——
   那正是本專案最貴的一類失敗:照跑、照出數字、結論卻是錯的。

═══ 這一檔還鎖住一個更重要的東西:誠實 ═══

預測器最危險的失效模式不是算錯,是**用一個飛機做不到的剖面宣告可達**。

實證:把 RPQ 的交接狀態(h=3100、d=4700、V=160 ⇒ 需求 γ=33.4°)代進
理想追蹤,預測器回報「會命中」;而 RPQ 六架實際殘餘 **416–3084 m,全部沒落地**。
差別不在積分器(重放實測 γ(t) 時 RMS 只有 5.4 m/s),而在
**「指令的 γ」與「做得到的 γ」是兩回事**。

⇒ `gamma_cap_deg` 與 `gamma_sat_frac` 是這份實作的良心,不可以被拿掉。
"""
import math
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "wsl" / "sar_swarm_patches"))
sys.path.insert(0, str(ROOT / "scripts" / "research" / "predictor"))

import predictor as P            # noqa: E402


def _sim():
    """研究版。⚠️ import 時不可觸發 traj.pkl 讀取(那是 validate() 才做的)。"""
    import sim
    return sim


class TestPhysicsMatchesTheValidatedImplementation:
    """⭐⭐ 兩份實作必須逐點相同。"""

    @pytest.mark.parametrize("h", [0.0, 1500.0, 3100.0, 6000.0])
    def test_density(self, h):
        assert P.air_density(h) == pytest.approx(_sim().rho(h), rel=0, abs=1e-12)

    @pytest.mark.parametrize("v", [80.0, 150.0, 210.0, 260.0])
    @pytest.mark.parametrize("h", [0.0, 3100.0])
    @pytest.mark.parametrize("gd", [0.0, 15.0, 27.0, 40.0])
    def test_drag(self, v, h, gd):
        g = math.radians(gd)
        assert P.drag_n(v, h, g) == pytest.approx(_sim().drag(v, h, g), abs=1e-9)

    @pytest.mark.parametrize("v", [80.0, 150.0, 210.0])
    @pytest.mark.parametrize("cap", [25.0, 100.0])
    def test_thrust(self, v, cap):
        assert P.thrust_n(v, cap) == pytest.approx(_sim().thrust(v, cap), abs=1e-9)

    @pytest.mark.parametrize("gd", [15.0, 27.0, 40.0])
    @pytest.mark.parametrize("cap", [25.0, 100.0])
    def test_equilibrium_speed(self, gd, cap):
        assert P.terminal_speed_on_ray(gd, 0.0, cap) == \
            pytest.approx(_sim().v_eq(gd, 0, cap), abs=0.05)

    def test_airframe_constants_are_identical(self):
        s = _sim()
        assert (P.MASS_KG, P.S_REF_M2, P.CD0) == (s.M, s.S, s.CD0)
        assert P.THRUST_FULL_N == pytest.approx(s.TFULL, abs=1e-9)


class TestThePredictorIsHonestAboutWhatTheAircraftCanDo:
    """⚠️⚠️ 本檔最重要的一組 —— 擋的是「用做不到的剖面宣告可達」。"""

    def test_the_cap_defaults_to_the_measured_capability(self):
        """預設必須是**實測**上限,不是理想值。

        ⚠️ 預設若是 None(理想追蹤),每一次呼叫都會系統性樂觀,
           而呼叫端不會知道 —— 那比沒有預測器更糟。
        """
        # ⚠️ 2026-08-14:33.0 → 36.0。舊值是**開關關閉時**量的分界(29.1/33.8),
        #    RPT 開啟後分界上移到 35.6/37.4。用舊值會誤拒可行的交棒。
        assert P.PN_GAMMA_CAP_DEG == pytest.approx(36.0)
        assert 35.6 < P.PN_GAMMA_CAP_DEG < 37.4, "上限必須落在實測全中/全敗之間"
        import inspect
        sig = inspect.signature(P.predict_terminal)
        assert sig.parameters["gamma_cap_deg"].default == P.PN_GAMMA_CAP_DEG

    def test_saturation_is_reported(self):
        """要求超過能力時必須**說出來**,不是默默夾掉。

        ⚠️ 夾掉而不回報 = 預測器自己製造一個「看起來可行」的答案。
        """
        # ⚠️ 取一個**確定超過能力**的幾何:γ_req = atan(3100/3000) = 46°,
        #    遠高於 cap 36。原本用 d=4700(γ_req 33.4°)是在 cap=33 時代寫的,
        #    cap 提到 36 之後它不再飽和 —— 測試的前提要跟著實測走。
        r = P.predict_terminal(3100.0, 3000.0, 160.0, gamma_cap_deg=36.0)
        assert r["gamma_sat_frac"] > 0.5, "需求遠超過能力,卻沒回報飽和"
        assert r["gamma_req_max_deg"] > 36.0

    def test_no_saturation_when_the_profile_is_within_reach(self):
        """反向鎖:做得到的剖面不可以誤報飽和,否則這個指標會被當成雜訊忽略。"""
        r = P.predict_terminal(2600.0, 4700.0, 160.0, gamma_cap_deg=36.0)
        assert r["gamma_sat_frac"] == 0.0
        assert r["gamma_req_max_deg"] < 36.0

    def test_crossing_altitude_is_reported_not_just_touchdown(self):
        """⚠️⚠️ 驗收量的是**水平最近通過時的殘餘高度**,不是觸地時的水平距。

        誤差 = 1.167 × 殘餘高度(R²=0.998)⇒ 殘餘高度才是 KPI③ 的直接上游。
        只看 `reached` 會把「從紅心上空 3 km 掠過然後落在 74 m 外」當成命中 ——
        那正是 hit-criterion-ignores-altitude 記過的坑。
        """
        r = P.predict_terminal(3100.0, 4700.0, 160.0)
        assert r["h_at_crossing"] is not None
        assert r["miss_pred_m"] is not None
        # 迴歸式必須被用上
        assert r["miss_pred_m"] == pytest.approx(
            max(0.0, 1.1668 * r["h_at_crossing"] - 2.62), abs=1e-6)


class TestThePredictionDependsOnTheConfigurationBeingPredicted:
    """⭐⭐ 預測器**必須**用「那一趟實際的組態」去夾,不能用預設值一體適用。

    ⚠️⚠️ 這是最容易犯又最難發現的錯:同一組交接狀態,用不同的 `gamma_cap_deg`
    會給出差 5–14 倍的殘餘高度,而兩個答案看起來都很合理。

        RPQ 六架次:pn_los_floor=0、pn_nz_min=0.05(開關**關閉**)
                    反解實得俯角 1.0–30.1°,中位 11.5°
        RPT 六架次:pn_los_floor=1.0、pn_nz_min=−1.0(開關**開啟**)
                    實測 γ ≤ 35.6° 全中、≥ 37.4° 全敗 ⇒ cap 36

    ⇒ 拿 cap=36(開啟)去預測 RPQ(關閉)會嚴重低估殘餘;
      拿 cap=11.5(關閉)去預測 RPT 會嚴重高估。
      **預設值只對「當前組態」有效**,預測歷史航次一定要顯式傳當時的 cap。
    """

    #: (交接h, 交接d, 實測殘餘AGL)。開關**關閉**時飛的。
    RPQ_CASES = [
        (3124, 4678, 416.6), (3299, 4684, 2448.0), (3113, 4395, 2378.4),
        (3146, 4539, 1750.2), (3054, 4567, 2030.8), (3084, 4410, 3084.1),
    ]
    #: 開關關閉時反解出來的等效俯角中位數。
    SWITCHES_OFF_CAP = 11.5

    def test_with_the_right_cap_rpq_is_predicted_to_fail(self):
        """用**當時的**能力去預測,六架都該被判成留下大量殘餘高度。"""
        for h, d, meas in self.RPQ_CASES:
            r = P.predict_terminal(float(h), float(d), 160.0,
                                   gamma_cap_deg=self.SWITCHES_OFF_CAP)
            assert r["h_at_crossing"] > 300.0, (
                f"h={h} d={d}:用關閉時的能力(cap {self.SWITCHES_OFF_CAP}°)預測"
                f"殘餘只有 {r['h_at_crossing']:.0f} m,而實測是 {meas:.0f} m")

    def test_using_the_wrong_cap_would_have_hidden_the_failure(self):
        """⚠️ 反面示範,鎖住「用錯 cap 會怎樣」——
        用開啟後的 cap 36 去預測 RPQ,殘餘會被低估一個量級。
        這條測試存在的意義是:**若哪天有人把顯式 cap 拿掉,這裡會紅。**"""
        h, d, meas = self.RPQ_CASES[-1]           # 實測殘餘 3084 m
        wrong = P.predict_terminal(float(h), float(d), 160.0)   # 用預設 cap=36
        right = P.predict_terminal(float(h), float(d), 160.0,
                                   gamma_cap_deg=self.SWITCHES_OFF_CAP)
        assert wrong["h_at_crossing"] < right["h_at_crossing"] / 3.0,             "用錯 cap 沒有造成明顯低估 —— 這條反面示範失去意義,請重新校準"
        assert right["h_at_crossing"] > 300.0

    def test_when_the_cap_binds_hard_the_prediction_is_insensitive_to_angle(self):
        """⭐⭐ cap 強約束時,預測殘餘對 γ **幾乎不敏感**。

        六架的 γ 需求跨 33.7°–35.3°,而預測殘餘全部擠在 2125–2346 m
        (帶寬只有 221 m,約 10%)。因為 cap 只有 11.5° 時飛機根本降不了多少,
        殘餘 ≈ 初始高度減掉一小段,與「要求多陡」幾乎無關。

        ⚠️⚠️ 對交接判據的直接後果:**能力飽和時「需求角度」不是可達性指標。**
           該看的是「還要卸多少高度」與「還有多少航程」的**絕對量**,
           不是兩者的比值 —— 比值只在能力有餘裕時才帶資訊。
           先前那條 29.1°/33.8° 的分界之所以看起來很乾淨,正是因為
           那批航次的能力全部飽和,分界其實是高度分界穿了角度的外衣。
        """
        res = [P.predict_terminal(float(h), float(d), 160.0,
                                  gamma_cap_deg=self.SWITCHES_OFF_CAP)["h_at_crossing"]
               for h, d, _ in self.RPQ_CASES]
        assert min(res) > 2000.0, f"預測殘餘不夠大:{[round(x) for x in res]}"
        spread = max(res) - min(res)
        assert spread < 0.15 * min(res), (
            f"預測殘餘帶寬 {spread:.0f} m 佔 {spread / min(res):.0%},"
            f"不像「對角度不敏感」—— 前提可能變了,請重新檢視")

    def test_rpt_cases_with_the_current_cap_are_predicted_to_land(self):
        """⭐ 開關開啟後的 RPT 實測(γ 28.0/33.0/34.9/35.6 全中,殘餘 ≤4.61 m)。
        用**現行**預設 cap 預測,這四架都該被判成幾乎零殘餘。"""
        for h, d in ((2483, 4678), (3041, 4692), (3267, 4678), (3000, 4188)):
            r = P.predict_terminal(float(h), float(d), 160.0)
            assert r["h_at_crossing"] < 60.0, (
                f"h={h} d={d}(γ={P.required_gamma_deg(h, d):.1f}°)"
                f"被預測成殘餘 {r['h_at_crossing']:.0f} m,而實測全部 ≤4.61 m")

    def test_lowering_the_handover_altitude_still_helps(self):
        r = P.predict_terminal(2600.0, 4700.0, 160.0)
        assert r["h_at_crossing"] < 10.0
        assert r["gs_end"] > 150.0


class TestFeasibleTimeWindow:
    """PINN-PCG 時間協同的輸入。

    ⚠️ `t_min` 在現行 tot_node 裡**完全不存在** —— 共識只取最慢的那架
    (`now + max_j(d_j/pace_j) + 2.0`),從不檢查最快的那架能不能等那麼久。
    """

    def test_window_is_ordered_and_nonempty(self):
        w = P.feasible_time_window(3100.0, 4700.0, 160.0)
        assert w["n"] > 0
        assert w["t_min"] <= w["t_max"]

    def test_the_minimum_time_is_altitude_limited_not_range_limited(self):
        """⭐⭐ 這條是量出來的、而且對時間協同有直接後果。

        d = 4000 與 d = 8000 的 `t_min` **完全相同**(26.9 s):
        在我們的幾何裡,最短飛行時間由「以 ≤cap 的俯角把 3100 m 卸完」決定,
        與水平距無關 —— 因為俯角被夾在 33°,降高才是瓶頸。

        ⚠️⚠️ 對 PINN-PCG 的時間協同這是壞消息:
           「壓平滑翔坡度來加速」在**下界**這一側沒有效果,
           因為下界不是距離造成的。能買到的時間只在**上界**那一側(拖延)。
           ⇒ 協同時刻 t_c = ½(max_i t_min_i + min_i t_max_i) 若落在
             某機的 t_min 之下,那一架**無論如何都趕不上**,而現行的
             共識式(只取最慢的那架)根本不會發現這件事。
        """
        near = P.feasible_time_window(3100.0, 4000.0, 160.0)
        far = P.feasible_time_window(3100.0, 8000.0, 160.0)
        assert near["t_min"] == pytest.approx(far["t_min"], abs=0.2), \
            "t_min 隨水平距改變了 —— 若真如此,代表已進入距離受限區,請更新本測試"

    def test_a_higher_aircraft_needs_more_time(self):
        """高度受限區裡,真正推高 t_min 的是**高度**。"""
        low = P.feasible_time_window(1500.0, 5000.0, 160.0)
        high = P.feasible_time_window(3100.0, 5000.0, 160.0)
        assert high["t_min"] > low["t_min"]
