# -*- coding: utf-8 -*-
"""ITCG(命中時間控制)的行為鎖(2026-08-15)。

═══ 為什麼要換掉純 PN ═══

使用者定調:**傳統 PN 是一打一,式子裡沒有時間這個量** ⇒ 它保證撞到,
不保證什麼時候撞到。多機同時到達本質上超出它的表達能力。

實測反證(RQA2,同一趟、同組態):
  兩架的放行幾何幾乎相同 —— 5120 m/31° 與 5590 m/29°、放行時刻差只有 −2.5 s
  —— 卻飛出 γ 中位 **−27.1° vs −32.4°**、俯衝耗時 **55 s vs 33 s**。
  上游已經對齊,PN 自己散開 21.2 s。它沒有任何一項在看時間。

═══ ⚠️⚠️ 誤差形式:文獻的等速形式在我們身上**完全失效** ═══

Kim & Shin 2023, Aerospace 10(3):308 式 5(逐字):

    e_t = V·(t_d − t) − R        **e_t > 0 ⇒ 太早到**

它的賣點是三個量全部直接可測、不需要估 t_go。我第一版就照抄了。

**RQC(2026-08-15)實測:整條律一次都沒出力。** itcg_accel 非零樣本 = 0,
八架全部,而契約顯示 itcg_on=1、gain=1.0 全部 OK ——「開了但沒效果」的
最糟版本,只因為我多記了一欄 itcg_accel 才發現。

根因:**該式假設等速**,而我們的俯衝是重力加速。放行瞬間實測瞬時地速
142.7 m/s,預測器算出的隱含平均速度是 **208 m/s** ⇒ 用瞬時速度乘剩餘時間
必然構不到斜距 ⇒ e_t 恆為負 ⇒ 單邊守衛(遲到不出力)把整條律關掉。

⇒ 改用秒形式 + 等效路徑餘裕:

    e_s     = (t_d − t) − t_go          ← t_go 走校準過的預測器(唯一入口)
    surplus = R · (e_s / t_go)          ← 公尺;要多繞多遠

⚠️ 兩種形式**都記錄**(early_by_m 舊式、itcg_surplus_m 新式),
   就是為了下次能一眼看出是不是又踩到同一類假設。

同篇式 6(逐字):

    de_t/dt = −V(1 − cos σ) ≤ 0

⇒ **這條律只能加長航程,不能縮短**。所以 e_t ≤ 0(已遲到)時不出力,
   而共同命中時刻必須取 `max_j`(取平均會讓最慢那架永遠追不上)。
"""
import ast
import math
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOT = ROOT / "scripts" / "wsl" / "sar_swarm_patches" / "tot_node.py"
SRC = TOT.read_text(encoding="utf-8")
TREE = ast.parse(SRC, filename=str(TOT))
GRAV = 9.80665


def _const(name):
    """讀模組層常數的**值**(不 import tot_node —— 它要 rclpy)。"""
    for n in TREE.body:
        if isinstance(n, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == name for t in n.targets):
            return ast.literal_eval(n.value)
    raise AssertionError(f"tot_node.py 找不到常數 {name}")


def _func(name):
    for n in ast.walk(TREE):
        if isinstance(n, ast.FunctionDef) and n.name == name:
            return n
    raise AssertionError(f"tot_node.py 找不到 {name}()")


class _Stub:
    itcg_on = True
    itcg_gain = 1.0
    itcg_a_max_g = 0.4
    fz_r = 100.0
    n_lat = 4.0
    # ⚠️ B1(2026-08-19):MPNG 預設 **False** ⇒ 底下每一條既有的鎖都仍然
    #    量的是**現行**的 surplus/σ_need 路徑。這正是「新開關預設 =
    #    逐位元同現行行為」的意思:舊鎖一條都不用改就該全綠。
    mpng_on = False
    mpng_k_eps = 0.0
    mpng_k_s = 0.0

    def __init__(self):
        self.itcg_e = {}
        self.itcg_s = {}
        self.itcg_side = {}
        self.mpng_k0 = {}
        self.itcg_dbg = {}
        self.dive_T = 1000.0
        self.tgo = 30.0            # 由測試控制的預測 t_go

    def _tgo_of(self, i, d):
        """⚠️ 樁:回傳可控的 t_go。真實版走校準過的點質量積分器 ——
        RQC 證明**必須**用它而不是瞬時速度,因為俯衝是重力加速的。"""
        return self.tgo

    def get_logger(self):
        return type("L", (), {"info": lambda *a, **k: None,
                              "warn": lambda *a, **k: None,
                              "error": lambda *a, **k: None})()


def _bind():
    # ⚠️ `_itcg_bias` 讀一個模組層常數(Jiang Eq.28/39 的 N′ 符號開關)。
    #    exec 的命名空間必須帶上它,否則是 NameError 而不是斷言失敗 ——
    #    後者看得懂,前者會被誤讀成「測試環境壞了」。
    ns = {"math": math,
          "MPNG_NPRIME_2_MINUS_N": _const("MPNG_NPRIME_2_MINUS_N")}
    src = ast.get_source_segment(SRC, _func("_itcg_bias"))
    exec("class _M:\n" + "\n".join("    " + l for l in src.splitlines()), ns)
    M = ns["_M"]
    s = _Stub()
    s.__class__ = type("S", (_Stub, M), {})
    return s


def _call(s, now, *, d=5000.0, R=5800.0, V=180.0, vh=160.0, sigma_deg=10.0):
    """以指定的前置角 σ 呼叫。目標放在正北方 (dn=d, de=0)。"""
    dn, de = d, 0.0
    a = math.radians(sigma_deg)
    # cross = dn·ve − de·vn = d·ve  ⇒ ve 的正負決定 σ 的正負
    vn, ve = vh * math.cos(a), vh * math.sin(a)
    return s._itcg_bias(1, now, dn, de, d, R, vn, ve, vh, V)


class TestTheLawCanOnlyDelay:
    """⭐⭐ de_t/dt = −V(1−cos σ) ≤ 0 ⇒ e_t 單調不增 ⇒ **遲到時無藥可救**。

    ⚠️ 這一條若破了,飛機會在已經遲到的時候還去繞路 —— 把遲到變得更嚴重,
       而且方向上完全反了。
    """

    def test_early_produces_a_bias(self):
        s = _bind()
        # t_rem = 1000−900 = 100 s;V·t_rem = 18000 ≫ R=5800 ⇒ 大幅早到
        a_t, _ = _call(s, 900.0)          # t_rem=100 ≫ t_go=30 ⇒ 大幅早到
        assert s.itcg_s[1] > 0.0, "等效路徑餘裕應為正(早到 ⇒ 要繞遠路)"
        assert abs(a_t) > 0.0, "早到必須出力繞路"

    def test_late_produces_no_bias(self):
        s = _bind()
        # t_rem = 1000−990 = 10 s;V·t_rem = 1800 < R=5800 ⇒ 遲到
        a_t, _ = _call(s, 990.0)          # t_rem=10 < t_go=30 ⇒ 遲到
        assert a_t == 0.0, (
            "⚠️⚠️ 已經遲到還在出力繞路 —— 這條律數學上只能加長航程,"
            "繞路只會讓遲到更嚴重。")

    def test_exactly_on_time_produces_no_bias(self):
        s = _bind()
        # t_rem 正好等於 t_go ⇒ e_s = 0
        a_t, _ = _call(s, 1000.0 - s.tgo)
        assert a_t == 0.0

    def test_past_the_impact_time_produces_no_bias(self):
        s = _bind()
        a_t, e_t = _call(s, 1001.0)     # t_rem < 0
        assert a_t == 0.0


class TestTheConstantSpeedFormWouldHaveStayedSilent:
    """⭐⭐ RQC 的迴歸鎖:用**真實量到的**數字,證明兩種形式的結果相反。

    RQC1-UAV1 放行瞬間(實測):斜距 6315 m、瞬時地速 142.7 m/s、
    預測 t_go 30.3 s ⇒ 隱含平均速度 **208 m/s**(俯衝在加速)。

    假設它早到 10 s(t_rem = 40 s):
      * 等速形式 e = V·t_rem − R = 142.7×40 − 6315 = **−607 m** ⇒ 判「遲到」⇒ 不出力
      * 秒形式   e_s = 40 − 30.3 = **+9.7 s** ⇒ 餘裕 2021 m ⇒ 出力 ✓

    ⚠️ 同一架飛機、同一瞬間,兩種形式給出**相反的結論**。
       等速形式的沉默不是「判斷它不用繞路」,是**假設不成立**。
    """

    R = 6315.0
    V_INST = 142.7
    T_GO = 30.3

    def test_the_naive_form_is_negative_even_when_early(self):
        s = _bind()
        s.tgo = self.T_GO
        s.dive_T = 1000.0
        a_t, e_naive = _call(s, 1000.0 - 40.0, d=self.R * 0.9, R=self.R,
                             V=self.V_INST, vh=self.V_INST * 0.9)
        assert e_naive < 0.0, (
            f"等速形式應為負({e_naive:.0f})—— 這正是 RQC 全批沉默的原因。"
            f"若它變成正的,代表這條迴歸鎖的前提沒了,請重新確認速度剖面。")

    def test_but_the_corrected_form_fires(self):
        s = _bind()
        s.tgo = self.T_GO
        s.dive_T = 1000.0
        a_t, _ = _call(s, 1000.0 - 40.0, d=self.R * 0.9, R=self.R,
                       V=self.V_INST, vh=self.V_INST * 0.9)
        assert s.itcg_s[1] > 0.0, "秒形式應判定早到 ⇒ 路徑餘裕為正"
        assert abs(a_t) > 0.0, (
            "⚠️⚠️ 修正後的形式**仍然**沒有出力 —— RQC 的失效原樣回來了。"
            "檢查 _itcg_bias 是不是又改回用瞬時速度。")

    def test_the_law_does_not_use_instantaneous_speed_for_the_decision(self):
        """結構鎖:判定分支必須看 `e_s`(秒形式),不可以看 `e_naive`。"""
        fn = ast.get_source_segment(SRC, _func("_itcg_bias"))
        assert "if e_s <= 0.0:" in fn, (
            "⚠️⚠️ 早/遲的判定沒有走秒形式。等速形式在加速俯衝上恆為負,"
            "拿它當判據等於把整條律關掉,而且完全無聲。")
        assert "self._tgo_of(" in fn, (
            "⚠️ 沒有走 t_go 的唯一入口 —— 速度剖面必須由校準過的預測器承擔。")


class TestSideIsLatched:
    """⭐⭐ 文獻的 `sign(σ)` 在 σ=0 有奇異點。σ 在零附近抖動 ⇒ 偏置左右翻面。

    ⚠️ 本專案已有同構的失效:[[stot-altitude-gate-chatter]] ——
       閘門沒遲滯 ⇒ 翻轉次數直接寫進末端誤差,UAV5 翻 20 次 → 35.6 km。
    """

    def test_side_survives_sigma_crossing_zero(self):
        s = _bind()
        a1, _ = _call(s, 900.0, sigma_deg=+10.0)
        assert s.itcg_side[1] == 1.0
        a2, _ = _call(s, 900.0, sigma_deg=-10.0)   # σ 翻到另一側
        assert s.itcg_side[1] == 1.0, "方向必須鎖存,不可以跟著 σ 翻面"
        assert a1 * a2 > 0.0, "⚠️ 偏置翻面了 —— 鎖存沒有生效"

    def test_no_latch_inside_the_deadband(self):
        """σ 還在死區內就不鎖 —— 否則會把雜訊鎖成方向。"""
        s = _bind()
        a_t, _ = _call(s, 900.0, sigma_deg=0.5)
        assert a_t == 0.0
        assert 1 not in s.itcg_side, "死區內不可以鎖定方向"

    def test_the_latch_is_in_the_reset_manifest(self):
        """⚠️ 留著上一發的方向 ⇒ 新目標第一拍就往上一發的方向繞。"""
        assert '"itcg_e", "itcg_a", "itcg_s", "itcg_side",' in SRC, (
            "itcg_side 必須在 RESET_DICTS 裡逐發清掉")


class TestTheBiasIsBounded:
    def test_cap_is_respected(self):
        s = _bind()
        s.itcg_gain = 1000.0              # 故意灌爆
        a_t, _ = _call(s, 900.0)
        assert abs(a_t) <= s.itcg_a_max_g * GRAV + 1e-9, (
            f"偏置 {a_t:.2f} 超出上限 {s.itcg_a_max_g * GRAV:.2f} m/s² —— "
            f"時間項會壓過 PN 的命中通道")

    def test_terminal_freeze_uses_the_existing_threshold(self):
        """⚠️ 末端不可以再橫向機動。而且必須沿用 PN 既有的凍結門檻,
        不另立一套 —— 兩套門檻會在中間那段互相打架且無人報告。"""
        s = _bind()
        a_t, _ = _call(s, 900.0, d=50.0)   # d < fz_r
        assert a_t == 0.0

    def test_zero_gain_means_zero_bias(self):
        """⚠️ 「開了但沒效果」—— 契約層有不變式擋,這裡鎖行為面。"""
        s = _bind()
        s.itcg_gain = 0.0
        a_t, _ = _call(s, 900.0)
        assert a_t == 0.0


class TestTheSwitchReallyDisables:
    def test_off_returns_zero_and_records_nothing(self):
        s = _bind()
        s.itcg_on = False
        a_t, e_t = _call(s, 900.0)
        assert a_t == 0.0 and e_t is None
        assert not s.itcg_e, "關閉時不應該留下任何觀測值(避免事後誤讀成有跑)"

    def test_no_dive_T_means_no_bias(self):
        """共識時刻還沒鎖定之前不能出力 —— 那時 e_t 沒有定義。"""
        s = _bind()
        s.dive_T = None
        a_t, e_t = _call(s, 900.0)
        assert a_t == 0.0 and e_t is None


class TestItIsWiredIntoTheLateralChannelOnly:
    """⭐ 縱向管角度、側向管時間(Sheng Yi 2021;Jiang 2021,兩篇獨立一致)。

    ⚠️ 混進縱向通道會直接動到**已經達標**的命中誤差(3.6–4.6 m)。
    """

    def test_bias_is_added_to_a_h_not_to_gdot(self):
        fn = ast.get_source_segment(SRC, _func("_pn_cmd"))
        assert "a_h += a_t" in fn, "ITCG 偏置沒有加進橫向通道"
        after = fn.split("a_h += a_t", 1)[1]
        assert "gdot += a_t" not in fn and "nz += a_t" not in fn, (
            "⚠️⚠️ 時間項混進了縱向通道 —— 那會動到命中誤差,"
            "而命中誤差是目前唯一已達標的 KPI。")
        assert "roll = math.atan2(a_h" in after, (
            "偏置必須在 roll 之前疊加,否則它根本沒有進到指令裡")
