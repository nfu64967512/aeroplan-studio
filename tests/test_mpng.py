# -*- coding: utf-8 -*-
"""B1:Jiang 2021 MPNG(Eq.41)導入的**值層級**迴歸鎖(2026-08-19)。

═══ 這一輪到底在做什麼 ═══

使用者 2026-08-19 裁決:把 Jiang, Ge, Xu, Yang《Impact Time Control
Cooperative Guidance Law Design Based on Modified Proportional Navigation》
(Aerospace 2021, 8(8):231,CC BY)的 **2D 指令式 Eq.(41)** 導入專案:

    a_M = N·V·λ̇  +  K_ε·N·V²·sin(σ)·ε_T  +  K_s·N·V²·sign(σ)·ε_T
          └ a_PNG ┘  └────── a_ε ───────┘   └────── a_s ────────┘
    ε_T ≡ t̄go − tgo   (Eq.49)  **ε_T > 0 = 還有多的時間 = 會太早到**

⭐ 我們的掛點結構上**已經就是** Eq.(41):`_pn_cmd` 的 `a_h = n_lat·vh·λ̇`
   就是 a_PNG,`a_h += a_t` 就是那個加號 ⇒ `_itcg_bias` 只需要回傳 a_ε + a_s,
   呼叫端一行都不用動。

═══ ⚠️⚠️ 這一輪的**首要目的不是讓它變好** ═══

三個必須先寫下來的硬事實,否則這批資料會被誤讀:

1. **權限差一個數量級。** 實測幾何重放(n=32,RRD1-8 + RRE1-8)給出的
   時間權限上界:`itcg_a_max_g=0.4`(RRE 契約快照值)⇒ **+1.45 s**;
   契約上限 `1.0` ⇒ **+10.59 s**。而 n=8 同時到達的需求是 **7.3–9.0 s**。
   ⇒ 這一輪要做的是把上界**量準**,不是假設它成立。

2. **那條通道從來沒有承載過訊號。** `itcg_accel` 80,760 個樣本全零
   (RRD+RRE 的 dive 段 18,567 筆有值、非零 0 筆)。
   「開關開著但那一段從來沒真的發生」是本專案的頭號失效類別,已經第五次
   (JET_CDP 空字串 / predict_hz=0 / pn_los_floor=0 / catch_speed 被 FC
   整發拒收 / itcg_accel 全零)。⇒ **觀測性必須先於功能。**

3. **論文的 t_go 估計在我們的工作點沒有優勢。** Eq.(29) 的優勢只在
   σ > 40°(原文行 1079 逐字:低於 40° 時三種方法 "similar"),而我們
   σ 中位 15.4°、32 架次裡 28 架 ≤ 23.8°。實測餵我們 32 架次:|誤差|
   中位從現行的 3.73 s **變差到 4.28 s**。⇒ 並列記錄一欄,**不取代**。

═══ 為什麼這些鎖是「值層級」不是「源碼字串比對」 ═══

實證:源碼層的 38 條鎖被 ≥25 種等價改寫繞過(改上游、改下游、換容器、
換命名都免疫);值層級的 26 條一次關掉 8 個。所以本檔的主體是
「**餵一組值進去,斷言送出來的數字**」,而不是「斷言那一行長什麼樣」。

⚠️ 唯一的例外是 `TestTheSwitchIsWiredEverywhere` —— 接線本身沒有可觀測的
   數值,只能對著檔案驗;而「環境變數名打錯 = 設了沒有用且零告警」正是
   effective-value-contract 記的那一整類。
"""
import ast
import itertools
import math
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
PATCHES = ROOT / "scripts" / "wsl" / "sar_swarm_patches"
TOT = PATCHES / "tot_node.py"
SRC = TOT.read_text(encoding="utf-8")
TREE = ast.parse(SRC, filename=str(TOT))
GRAV = 9.80665

sys.path.insert(0, str(PATCHES))
from contract_table import BY_NAME, BOOL_PARAMS, ENV_FOR, JET_REFERENCE  # noqa: E402
from contract_table import evaluate  # noqa: E402


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


# ── 把**真正在飛的那一份** `_itcg_bias` 綁進一個樁 ─────────────────────
# ⚠️ 用 exec 原始碼而不是 import:ROS2 在 Windows 開發機上不可用,而這些鎖
#    必須每次都跑得到。走原始碼也保證測的是「檔案裡的那一份」,不是複本。
_NS = {"math": math, "MPNG_NPRIME_2_MINUS_N": _const("MPNG_NPRIME_2_MINUS_N")}
exec("class _M:\n" + "\n".join(
    "    " + l for l in ast.get_source_segment(SRC, _func("_itcg_bias")).splitlines()),
    _NS)


class _Stub(_NS["_M"]):
    """最小節點樁。⚠️ 只提供 `_itcg_bias` 真的會碰到的東西 —— 多餘的屬性
    會讓「忘了初始化某個 dict」這類失效在測試裡被蓋掉。"""

    itcg_on = True
    itcg_gain = 1.0
    itcg_a_max_g = 0.4
    fz_r = 100.0
    n_lat = 4.0
    mpng_on = False
    mpng_k_eps = 0.0
    mpng_k_s = 0.0

    def __init__(self, tgo=41.3):
        self.itcg_e, self.itcg_s, self.itcg_side = {}, {}, {}
        self.mpng_k0, self.itcg_dbg = {}, {}
        self.dive_T = 1000.0
        self.tgo = tgo

    def _tgo_of(self, i, d):
        return self.tgo

    def get_logger(self):
        return type("L", (), {"info": lambda *a, **k: None,
                              "warn": lambda *a, **k: None})()


#: 實測工作點(RRD+RRE,n=32 的中位):放行斜距 6854 m、俯衝耗時 41.3 s、
#: 俯衝中位地速 161.7 m/s、放行 σ 15.3°。⚠️ 這些不是拍的,是量的。
R0, TGO0, V0, SIG0 = 6854.0, 41.3, 161.7, 15.3


def _call(s, *, early_s=3.7, d=6142.0, R=R0, V=V0, sigma_deg=SIG0, vh=None):
    """以「早到 early_s 秒」的幾何呼叫一次。目標在正北 (dn=d, de=0)。"""
    vh = V * 0.9 if vh is None else vh
    a = math.radians(sigma_deg)
    vn, ve = vh * math.cos(a), vh * math.sin(a)
    # ⚠️ dive_T 為 None 是**被測情境之一**(還沒有共同命中時刻),不是壞輸入 ——
    #    樁自己先炸掉的話,那一格反向鎖就變成「測到樁的 bug」而不是測到節點。
    now = 0.0 if s.dive_T is None else s.dive_T - (s.tgo + early_s)
    return s._itcg_bias(1, now, d, 0.0, d, R, vn, ve, vh, V)


# ══════════════════════════════════════════════════════════════════════════
#  §1 ⭐⭐ 新開關預設 False = **逐位元**同現行行為
# ══════════════════════════════════════════════════════════════════════════
class TestTheDefaultPathIsBitIdentical:
    """⭐⭐ 本專案的既有紀律(S1/S2 同樣寫法):新律必須能一顆開關退回舊行為。

    ⚠️ 沒有這一條,「開了之後變差」與「開之前就已經壞了」就分不開 ——
       而那正是 RRA 整批 8 趟 / 5 小時報廢後才學到的東西。

    這裡不比對源碼,**比對數字**:同一組輸入餵進新舊兩份實作,
    回傳值與所有副作用(itcg_e / itcg_s / itcg_side)都必須逐位元相同。
    """

    @staticmethod
    def _old(s, i, now, dn, de, d, R, vn, ve, vh, V):
        """2026-08-15 版 `_itcg_bias` 的**逐字**複本(surplus/σ_need 反解)。

        ⚠️ 這份複本是**故意**寫死在測試裡的:它是「舊行為」的唯一定義。
           若有人同時改了生產碼與這份複本,兩邊一起錯 —— 所以複本旁邊
           留著上面那段實測數字當現實錨點。
        """
        if not s.itcg_on or s.dive_T is None:
            return 0.0, None
        t_rem = s.dive_T - now
        if t_rem <= 0.0:
            return 0.0, None
        e_naive = V * t_rem - R
        s.itcg_e[i] = e_naive
        t_go = s._tgo_of(i, d)
        e_s = t_rem - t_go
        surplus = R * (e_s / max(t_go, 1e-3))
        s.itcg_s[i] = surplus
        if e_s <= 0.0:
            return 0.0, e_naive
        if d < s.fz_r:
            return 0.0, e_naive
        cross = dn * ve - de * vn
        dot = dn * vn + de * ve
        sigma = math.atan2(cross, dot)
        side = s.itcg_side.get(i)
        if side is None:
            if abs(sigma) < math.radians(2.0):
                return 0.0, e_naive
            side = 1.0 if sigma > 0.0 else -1.0
            s.itcg_side[i] = side
        frac = surplus / max(R, 1.0)
        cos_need = 1.0 / (1.0 + max(frac, 0.0))
        sig_need = math.acos(max(-1.0, min(1.0, cos_need)))
        a_t = s.itcg_gain * s.n_lat * vh * V * math.sin(sig_need) / max(R, 1.0)
        cap = s.itcg_a_max_g * 9.80665
        return max(-cap, min(cap, a_t * side)), e_naive

    #: 掃描格點刻意覆蓋每一條分支:遲到 / 準時 / 早到、凍結內外、
    #: 死區內外、σ 兩側、σ 到 81°(RRE 實測最大值)、增益 0 與飽和。
    GRID = list(itertools.product(
        [-11.3, 0.0, 3.7, 20.0, 60.0],          # early_s(負 = 遲到)
        [50.0, 3000.0, 6142.0],                 # d(50 < fz_r ⇒ 凍結)
        [5800.0, R0],                           # R
        [142.7, V0, 200.0],                     # V
        [0.5, -0.5, 10.0, -35.0, 81.0],         # σ(度)
        [20.0, TGO0, 60.0],                     # t_go
        [0.0, 1.0, 3.5]))                       # itcg_gain

    def test_every_grid_point_matches_the_old_implementation(self):
        bad = []
        for early_s, d, R, V, sd, tgo, gain in self.GRID:
            new, old = _Stub(tgo), _Stub(tgo)
            new.itcg_gain = old.itcg_gain = gain
            assert new.mpng_on is False, "開關預設必須是 False"
            vh = V * 0.9
            a = math.radians(sd)
            vn, ve = vh * math.cos(a), vh * math.sin(a)
            now = new.dive_T - (tgo + early_s)
            r_new = new._itcg_bias(1, now, d, 0.0, d, R, vn, ve, vh, V)
            r_old = self._old(old, 1, now, d, 0.0, d, R, vn, ve, vh, V)
            if (r_new != r_old or new.itcg_e != old.itcg_e
                    or new.itcg_s != old.itcg_s or new.itcg_side != old.itcg_side):
                bad.append((early_s, d, R, V, sd, tgo, gain, r_new, r_old))
        assert not bad, (
            f"⚠️⚠️ `mpng_on=False` 的路徑與 2026-08-15 版**不再逐位元相同** "
            f"({len(bad)}/{len(self.GRID)} 組不符)。\n"
            f"    前三組:{bad[:3]}\n"
            f"    ⇒ 這一批資料與 RRD/RRE 不可比,而且沒有任何一層會告訴你。")

    def test_the_grid_actually_exercises_both_outcomes(self):
        """⚠️ 反向鎖:格點若全部落在「不出力」上,上面那條會空轉通過。"""
        fired = 0
        for early_s, d, R, V, sd, tgo, gain in self.GRID:
            s = _Stub(tgo)
            s.itcg_gain = gain
            if abs(_call(s, early_s=early_s, d=d, R=R, V=V, sigma_deg=sd)[0]) > 0.0:
                fired += 1
        assert fired > 50, f"格點只有 {fired} 組真的出力 —— 比對等於沒測到致動路徑"


# ══════════════════════════════════════════════════════════════════════════
#  §2 ⭐⭐ 觀測性:這條通道 80,760 個樣本全零,先讓它有訊號
# ══════════════════════════════════════════════════════════════════════════
class TestObservabilityComesFirst:
    """⚠️⚠️ req/cmd 雙欄紀律:**判定量是 req**(每拍都寫、夾制前、凍結前),
    不是 cmd(hold-last-value ⇒ 假綠燈)。

    S2 那一輪的 `cmd_radius_m` 在開關開與關時同樣是 30 —— 拿它當「機制生效」
    的證據會給假綠燈。這一組鎖就是不讓同一件事再發生一次。
    """

    def test_the_law_actually_fires_at_the_measured_operating_point(self):
        """⭐ 上線第一件事:讓 `itcg_req_a` 非零。**再談效果。**"""
        s = _Stub()
        s.mpng_on = True
        a, _ = _call(s)
        assert abs(a) > 0.0, (
            "⚠️⚠️ 在實測工作點(R=6854 m、t_go=41.3 s、早到 3.7 s、σ=15.3°)"
            "MPNG 一點都沒出力 —— itcg_accel 全零的第六次。")
        assert abs(s.itcg_dbg[1]["req_a"]) > 0.0

    def test_late_still_records_a_requested_value(self):
        """⭐ 遲到時**不出力**(這條律只能加長航程),但 req 必須看得見。

        ⚠️ 沒有這一欄就分不出「律沒出力」是因為遲到、因為凍結、還是因為
           根本沒跑到那裡 —— 而三者的處置完全不同。
        """
        s = _Stub()
        s.mpng_on = True
        a, _ = _call(s, early_s=-11.3)
        assert a == 0.0, "遲到還去繞路會讓遲到更嚴重(de_t/dt = −V(1−cos σ) ≤ 0)"
        dbg = s.itcg_dbg[1]
        assert dbg["req_a"] < 0.0, "req 必須記下「它想反向出力」這件事"
        assert dbg["eps_t"] < 0.0, "ε_T < 0 = 已經遲到"

    def test_frozen_still_records_a_requested_value(self):
        s = _Stub()
        s.mpng_on = True
        a, _ = _call(s, d=50.0, R=60.0)      # d < fz_r
        assert a == 0.0, "末端凍結:不可以再橫向機動"
        assert s.itcg_dbg[1]["req_a"] > 0.0, "凍結是要被觀測的事件,不是沉默"

    def test_the_deadband_still_records(self):
        s = _Stub()
        s.mpng_on = True
        a, _ = _call(s, sigma_deg=0.5)
        assert a == 0.0 and 1 not in s.itcg_side, "死區內不鎖方向"
        assert s.itcg_dbg[1]["req_a"] != 0.0

    def test_nothing_is_recorded_before_the_channel_is_live(self):
        """⚠️ 反向鎖:總開關關著 / 還沒有共同命中時刻 ⇒ **不可以**留下任何值。
        留了就會讓事後判讀把「沒跑」讀成「跑了但剛好是 0」。"""
        for tweak in ("off", "no_dive_T", "past"):
            s = _Stub()
            s.mpng_on = True
            if tweak == "off":
                s.itcg_on = False
            elif tweak == "no_dive_T":
                s.dive_T = None
            a, e = (_call(s, early_s=-100.0) if tweak == "past" else _call(s))
            assert a == 0.0, tweak
            assert not s.itcg_dbg, f"{tweak}:通道沒跑卻寫了儀表"

    def test_the_six_columns_are_all_present(self):
        s = _Stub()
        s.mpng_on = True
        _call(s)
        assert set(s.itcg_dbg[1]) == {"req_a", "cmd_a", "eps_t", "tgo_ref",
                                      "tgo_jiang", "sat"}


# ══════════════════════════════════════════════════════════════════════════
#  §3 ⭐⭐ 權限:上界 +1.45 s(0.4 g) vs 需求 7.3–9.0 s
# ══════════════════════════════════════════════════════════════════════════
class TestTheClampIsTheRealVariable:
    """⭐⭐ 代進實測工作點,結論是「**增益完全不重要,只有夾制重要**」。

    r₀ = 6854 m、tgo₀ = 41.3 s ⇒ 論文建議式 K = 40/(r₀·tgo₀) = 1.413e−4 1/(m·s);
    N=4、V=161.7 m/s ⇒

        a_s = K·N·V²·ε_T = 14.78 · ε_T   m/s²

    ⇒ ε_T 只要 **0.27 s** 就撞上 0.4 g 的夾制(1.0 g 也只要 0.66 s),
      而我們的 ε_T 工作範圍是**數秒**級 ⇒ 這條律在我們身上整段貼在夾制上、
      退化成 bang-bang。K_ε/K_s 只決定「離開飽和的最後 0.3 秒長什麼樣」。

    ⚠️ 所以要掃的是 `itcg_a_max_g`(契約區間 0.1–1.0),不是增益。
       實測幾何重放(n=32):0.4 ⇒ **+1.45 s**、1.0 ⇒ **+10.59 s**,
       而 n=8 的需求是 **7.3–9.0 s**。
    ⚠️⚠️ 要跑 2.0 必須先由使用者裁決改契約 —— contract_verify 會整批中止,
       不可以在這一輪偷渡。
    """

    def test_the_paper_gain_saturates_immediately_at_our_operating_point(self):
        s = _Stub()
        s.mpng_on = True
        a, _ = _call(s, early_s=1.0)
        cap = s.itcg_a_max_g * GRAV
        dbg = s.itcg_dbg[1]
        assert dbg["sat"] == 1, (
            "⚠️ 在 ε_T = 1 s 就應該飽和 —— 若沒有,論文建議式的量級推算"
            "(a_s = 14.78·ε_T)與實作對不上,先查 K 的自動推導。")
        assert abs(a) == pytest.approx(cap), "飽和時輸出必須正好貼在夾制上"
        assert abs(dbg["req_a"]) > 3.0 * cap, (
            f"要求值 {dbg['req_a']:.1f} 只比夾制 {cap:.2f} 大一點 —— "
            f"與「整段 bang-bang」的推算不符")

    def test_the_auto_gain_matches_the_paper_formula(self):
        """K = 40/(r₀·tgo₀)。⚠️ 那是他們 330 m/s **等速**、r₀=10 km 的算例值,
        我們的工作點不同 ⇒ 這條只鎖「有照式子算」,不代表值是對的。"""
        s = _Stub()
        s.mpng_on = True
        _call(s)
        assert s.mpng_k0[1] == pytest.approx(40.0 / (R0 * TGO0), rel=1e-9)

    def test_the_gain_is_latched_not_recomputed_every_tick(self):
        """⚠️ 不鎖存 ⇒ 增益隨 R 一路飄(R 在俯衝段從 6854 m 掉到 0)⇒
        同一個 ε_T 在不同距離給出差三個數量級的指令。"""
        s = _Stub()
        s.mpng_on = True
        _call(s)
        k_first = s.mpng_k0[1]
        _call(s, d=1200.0, R=1300.0)
        assert s.mpng_k0[1] == k_first

    def test_an_explicit_gain_overrides_the_auto_formula(self):
        s = _Stub()
        s.mpng_on = True
        s.mpng_k_eps = s.mpng_k_s = 1e-6
        a, _ = _call(s)
        assert s.itcg_dbg[1]["sat"] == 0, "顯式小增益不該飽和"
        assert 1 not in s.mpng_k0, "顯式給值時不該去推論文的建議式"
        assert 0.0 < abs(a) < s.itcg_a_max_g * GRAV

    @pytest.mark.parametrize("cap_g", [0.1, 0.4, 1.0])
    def test_the_clamp_is_always_respected(self, cap_g):
        s = _Stub()
        s.mpng_on = True
        s.itcg_a_max_g = cap_g
        a, _ = _call(s, early_s=60.0)         # 故意灌爆
        assert abs(a) <= cap_g * GRAV + 1e-9, (
            "⚠️⚠️ 時間項壓過 PN 的命中通道 —— 而命中誤差是目前唯一達標的 KPI")

    def test_the_contract_band_still_tops_out_at_one_g(self):
        """⚠️ 反向鎖:有人把上限放寬就得有人看一眼。0.4 只買得到 +1.45 s,
        1.0 買到 +10.59 s,而需求是 7.3–9.0 s ⇒ 這個區間就是可行性本身。"""
        c = BY_NAME["itcg_a_max_g"]
        assert (c.lo, c.hi) == (0.1, 1.0), (
            f"itcg_a_max_g 的契約區間變成 [{c.lo}, {c.hi}] —— "
            f"時間權限的可行性結論綁在這個區間上,改它要有裁決。")


# ══════════════════════════════════════════════════════════════════════════
#  §4 ⭐⭐ 精度不在授權範圍內:ε_T → 0 必須退回純 PN
# ══════════════════════════════════════════════════════════════════════════
class TestPrecisionIsNotNegotiable:
    """原文行 735 逐字:ε_T = 0 時「the law proposed in this paper is
    equivalent to the PNG law」⇒ 終端退化成純 PN,**不動命中精度**。

    ⚠️ 但那是**漸近**的,不保證在 `fz_r` 之前歸零 ⇒ 末端凍結那道閘門
       必須留著,它是精度的保險絲。
    """

    def test_zero_time_error_means_zero_time_term(self):
        s = _Stub()
        s.mpng_on = True
        a, _ = _call(s, early_s=0.0)
        assert a == 0.0, (
            "⚠️⚠️ ε_T = 0 卻仍在出力 —— 那會在終端加一個沒有理由的橫向指令,"
            "直接動到已達標的命中誤差(0.92–5.36 m)。")

    def test_the_terminal_freeze_uses_the_existing_threshold(self):
        """⚠️ 必須沿用 PN 既有的 `fz_r`,不另立一套 —— 兩套門檻會在中間
        那段互相打架,而且沒有任何一層會報告。"""
        s = _Stub()
        s.mpng_on = True
        assert _call(s, d=s.fz_r - 1.0, R=s.fz_r)[0] == 0.0
        assert abs(_call(_stub_on(), d=s.fz_r + 1.0, R=s.fz_r + 10.0)[0]) > 0.0

    def test_the_time_term_never_touches_the_vertical_channel(self):
        """⭐ 縱向管角度、側向管時間(Sheng Yi 2021 與 Jiang 2021 兩篇獨立一致)。

        Jiang 的 3D 版 Eq.(68) 也只動 yaw 通道,pitch 是 `γ̇ = N·λ̇` 純 PN、
        **零時間項** —— 與我們的掛法一致。
        """
        fn = ast.get_source_segment(SRC, _func("_pn_cmd"))
        assert "a_h += a_t" in fn
        for forbidden in ("gdot += a_t", "nz += a_t", "gam_cmd += a_t",
                          "thr += a_t"):
            assert forbidden not in fn, (
                f"⚠️⚠️ 時間項混進縱向通道({forbidden})—— 命中誤差是目前"
                f"唯一達標的 KPI,而使用者明訂精度不在授權範圍內。")


def _stub_on(tgo=TGO0):
    s = _Stub(tgo)
    s.mpng_on = True
    return s


# ══════════════════════════════════════════════════════════════════════════
#  §5 sign(σ) 的抖振:鎖存比原文更必要
# ══════════════════════════════════════════════════════════════════════════
class TestTheSignTermIsTamed:
    """Eq.(42) 把 `sign(0)` 定義成 **+1**(不是 0),是**刻意**的:σ₀=0 時
    λ̇=0 ⇒ a_PNG=0、sin σ=0 ⇒ a_ε=0,只剩 a_s 能把 σ 推離零
    (原文行 843-844:"effective even if the initial leading angle is zero
    ... has no singularity")。

    ⚠️⚠️ 代價:a_s 的大小**與 σ 無關**,σ 過零時整項跳 2 倍幅值。
       而我們俯衝段的 σ 中位只有 **0.2°**(32 架次實測)⇒ 幾乎整段貼在
       零附近,正是它抖振最兇的區。
    ⇒ 我們用既有的 `itcg_side` 鎖存把不連續換成**一次性選擇**。
      ([[stot-altitude-gate-chatter]]:閘門沒遲滯 ⇒ 翻轉次數直接寫進末端
       誤差,UAV5 翻 20 次 → 35.6 km。同構的失效,已經發生過。)
    """

    def test_the_side_survives_sigma_crossing_zero(self):
        s = _stub_on()
        a1, _ = _call(s, sigma_deg=+10.0)
        assert s.itcg_side[1] == 1.0
        a2, _ = _call(s, sigma_deg=-10.0)
        assert s.itcg_side[1] == 1.0, "方向必須鎖存,不可以跟著 σ 翻面"
        assert a1 * a2 > 0.0, "⚠️ 偏置翻面了 —— a_s 的 2 倍跳變原樣回來了"

    def test_the_side_is_written_exactly_once_per_dive(self):
        s = _stub_on()
        writes = []
        real = s.itcg_side

        class _Watch(dict):
            def __setitem__(self, k, v):
                writes.append((k, v))
                super().__setitem__(k, v)

        s.itcg_side = _Watch(real)
        for sd in (+10.0, -10.0, +40.0, -80.0, +0.1):
            _call(s, sigma_deg=sd)
        assert len(writes) == 1, f"側向被寫了 {len(writes)} 次:{writes}"

    def test_the_sign_term_alone_can_start_from_zero_lead_angle(self):
        """⭐ a_s 存在的**唯一**理由:σ₀ = 0 的啟動問題。

        ⚠️ 我們的死區(|σ| < 2°)會先擋住第一拍 —— 這是刻意的取捨:
           寧可晚一點鎖方向,也不要把雜訊鎖成方向。這條鎖記錄的是
           「一旦離開死區,只靠 a_s 也推得動」。
        """
        s = _stub_on()
        s.mpng_k_eps = 0.0            # 只留 sign 項的自動增益
        a, _ = _call(s, sigma_deg=2.5)
        assert abs(a) > 0.0


# ══════════════════════════════════════════════════════════════════════════
#  §6 Eq.(29) 的 t_go:只記錄不使用
# ══════════════════════════════════════════════════════════════════════════
class TestTheJiangTgoIsRecordedButNeverUsed:
    """⚠️⚠️ **不要**用 Eq.(29) 取代現有的 t_go。

        tgo = (r/V)·(1 + N′·σ²)·(σ/sin σ)^{1/(N−1)}

    它的優勢只在 σ > 40°(原文行 1079 逐字:低於 40° 時三種方法 "similar"),
    而我們 σ 中位 15.4°、28/32 ≤ 23.8°。實測餵我們 32 架次:
    |誤差| 中位從現行的 3.73 s **變差到 4.28 s**。⇒ 並列記錄,讓資料說話。

    ⚠️ N′ 的符號在原文自相矛盾:Eq.(28) 印 `(2−N)/(6(N−1)(2N−1))`、
       Eq.(39) 印 `(N−2)/…`。這一輪照指派用 (2−N),用模組常數
       `MPNG_NPRIME_2_MINUS_N` 留一行可翻案的開關。
       反面證據(支持 Eq.39):Eq.(29) 展到二階必須退回 Eq.(30) 的
       1/(2(2N−1)),只有 (N−2) 成立。⚠️ 但這一欄**只記錄不致動**,而且
       在 σ=15.3° 兩式只差 0.24% 的 tgo(≈0.09 s)⇒ 不影響本輪結論。
    """

    def test_the_command_does_not_depend_on_the_jiang_tgo(self):
        """⭐⭐ 值層級:把 Eq.(29) 的結果換掉(改 N′ 的符號),**指令必須
        一模一樣**。這比「斷言某一行不出現」強 —— 換容器換命名都逃不掉。"""
        outs = {}
        for flag in (True, False):
            ns = dict(_NS)
            ns["MPNG_NPRIME_2_MINUS_N"] = flag
            exec("class _M2:\n" + "\n".join(
                "    " + l for l in
                ast.get_source_segment(SRC, _func("_itcg_bias")).splitlines()), ns)
            s = _Stub()
            # ⚠️⚠️ 基底順序是**這條鎖的成敗**:`_Stub` 自己就繼承了 `_M`
            #    (原始命名空間、原始 N′ 符號)⇒ 寫成 (_Stub, _M2) 時 MRO 先
            #    找到 `_M.` 的那一份,兩次迴圈跑的其實是**同一份程式碼**,
            #    tgo_jiang 當然相同 —— 鎖看起來綠燈但什麼都沒驗到。
            #    這正是本專案頭號失效類別在**測試側**的翻版。
            s.__class__ = type("S2", (ns["_M2"], _Stub), {})
            s.mpng_on = True
            outs[flag] = (_call(s, sigma_deg=60.0)[0], s.itcg_dbg[1]["tgo_jiang"])
        assert outs[True][0] == outs[False][0], (
            "⚠️⚠️ 換掉 Eq.(29) 的 N′ 符號竟然改變了指令 ⇒ 它**不是**只記錄"
            "不使用。而原文對這個符號自相矛盾 —— 讓一個有爭議的常數影響"
            "指令是不可接受的。")
        assert outs[True][1] != outs[False][1], "反向鎖:兩者應該真的算出不同的值"

    def test_it_is_recorded_and_is_in_the_right_ballpark(self):
        s = _stub_on()
        _call(s)
        dbg = s.itcg_dbg[1]
        assert dbg["tgo_ref"] == TGO0, "致動用的必須是我們現行的估計器"
        # σ=15.3°、r/V = 6854/161.7 = 42.4 s ⇒ Eq.(29) 應該略大於它
        assert R0 / V0 <= dbg["tgo_jiang"] <= R0 / V0 * 1.05

    def test_it_never_blows_up_the_guidance_law(self):
        """⚠️ 記錄層絕不可以因為算不出來就讓導引律爆掉(σ→180° 時 sin σ→0)。"""
        for sd in (0.0, 179.999, -179.999, 90.0):
            s = _stub_on()
            a, _ = _call(s, sigma_deg=sd)
            assert isinstance(a, float)


# ══════════════════════════════════════════════════════════════════════════
#  §7 接線:四處都要有,少一處就是「設了沒有用且零告警」
# ══════════════════════════════════════════════════════════════════════════
class TestTheSwitchIsWiredEverywhere:
    """⚠️⚠️ effective-value-contract 記的那一整類:環境變數名打錯 = 設了沒有
    用,而且**零告警**。所以四處都要對著檔案驗。"""

    def test_the_contract_table_owns_it(self):
        c = BY_NAME["mpng_on"]
        assert (c.lo, c.hi) == (0.0, 1.0)
        assert "mpng_on" in BOOL_PARAMS, (
            "⚠️ 不在 BOOL_PARAMS ⇒ 重跑提示會印成 `-p mpng_on:=1.0` 餵給 BOOL "
            "⇒ InvalidParameterTypeException ⇒ 節點整個起不來(同類坑第四次)")
        assert JET_REFERENCE["mpng_on"] == 0.0, "建議值必須是關閉(逐位元同現行)"
        assert ENV_FOR["mpng_on"] == "MPNG_ON"

    def test_the_two_gains_are_owned_and_read_back_too(self):
        """⚠️⚠️ 開關有不變式守著,**增益沒有**。`MPNG_K_EPS` 打錯一個字母 ⇒
        節點吃到 0.0 ⇒ 自動走論文建議式,而 log、契約行、CSV **全部看起來
        正常** ⇒ 掃增益的那一批會得到「怎麼調都一樣」,而那個結論其實是對的
        (因為根本沒調到),理由卻完全錯。這正是 effective-value-contract
        記的那一整類,所以增益必須(1)在表上、(2)被**讀回**。
        """
        for k in ("mpng_k_eps", "mpng_k_s"):
            c = BY_NAME[k]
            assert c.lo == 0.0, (
                f"{k} 的下界不是 0 —— 0.0 是**合法值**(走建議式 40/(r₀·tgo₀)),"
                f"擋掉它會讓自動推導這條生產路徑過不了契約")
            assert c.foam is None, f"{k} 不該有泡棉禁區:0.0 是合法的自動值"
            assert ENV_FOR[k] == k.upper()
        blob = (ROOT / "scripts" / "wsl" / "contract_verify.py").read_text(
            encoding="utf-8")
        for k in ("mpng_k_eps", "mpng_k_s"):
            assert f'"{k}"' in blob, (
                f"⚠️⚠️ {k} 沒進 contract_verify 的讀取映射 ⇒ 生效值從來不會被"
                f"讀回對照,設了沒有用也**零告警**")

    def test_the_node_declares_and_reads_it(self):
        assert 'g("mpng_on", False)' in SRC, "declare_parameter 預設必須是 False"
        assert 'self.mpng_on = bool(p("mpng_on").value)' in SRC

    def test_the_node_prints_it_into_the_contract_line(self):
        """⚠️ 不印進契約行 ⇒ contract_verify 一律判 missing,而 missing 的
        訊息長得像「節點沒起來」(RPZ1 就是這個形狀)。"""
        fn = ast.get_source_segment(SRC, _func("_contract_values"))
        for k in ("mpng_on", "mpng_k_eps", "mpng_k_s"):
            assert f'"{k}"' in fn, f"{k} 沒印進契約快照"

    def test_the_verifier_reads_it_back(self):
        blob = (ROOT / "scripts" / "wsl" / "contract_verify.py").read_text(
            encoding="utf-8")
        assert '"mpng_on"' in blob

    def test_the_launch_script_carries_all_three(self):
        sh = (ROOT / "scripts" / "wsl" / "swarm_ros2_up.sh").read_text(
            encoding="utf-8")
        assert 'TOT_MPNG_ON="${MPNG_ON:-False}"' in sh, (
            "⚠️ 預設值不可以是空字串 —— `-p name:=` 會讓 rcl 解析失敗、"
            "節點整個起不來,而症狀長得像埠號被搶走(RRA 8 趟報廢)。")
        for p in ("mpng_on", "mpng_k_eps", "mpng_k_s"):
            assert f'-p {p}:=' in sh, f"{p} 沒有真的餵給節點"


class TestTheInvariantCatchesTheSilentCombination:
    """⭐⭐ 值層級的守門:`mpng_on=1` 而 `itcg_on=0` 會讓整條律**一次都不會
    被呼叫到** —— `_itcg_bias` 第一行就 return,而契約快照仍然印「已啟用」。

    這正是本專案的頭號失效類別。源碼層擋不住(改上游/下游/旁路全免疫),
    值層級一次關掉整類。
    """

    BASE = {"mpng_on": 1.0, "itcg_on": 0.0, "itcg_gain": 1.0,
            "itcg_a_max_g": 0.4}

    def test_mpng_without_itcg_on_is_rejected(self):
        names = {v.name for v in evaluate(self.BASE, require=[]).violations}
        assert "MPNG 開著但 ITCG 總開關是關的" in names

    def test_turning_itcg_on_clears_it(self):
        cfg = dict(self.BASE, itcg_on=1.0)
        names = {v.name for v in evaluate(cfg, require=[]).violations}
        assert "MPNG 開著但 ITCG 總開關是關的" not in names

    def test_mpng_off_is_never_judged(self):
        cfg = dict(self.BASE, mpng_on=0.0)
        names = {v.name for v in evaluate(cfg, require=[]).violations}
        assert "MPNG 開著但 ITCG 總開關是關的" not in names
