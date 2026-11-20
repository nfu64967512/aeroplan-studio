# -*- coding: utf-8 -*-
"""B2:Jiang 2021 MPNG(Eq.41)的**值層級**迴歸鎖(M1–M9,2026-08-19)。

═══ 為什麼這一份不是 `tests/test_mpng.py` ═══

⚠️ 交付當下 `tests/test_mpng.py` 正在被**同一輪的另一支(B1)即時改寫**
   (實測 mtime 落在本檔動筆前 60 秒內,而且同一個檔案在我兩次讀取之間
   內容就變了一次)。兩支同時寫同一個檔 = 後寫的把先寫的整段吃掉,
   而且 git 看不到(那個檔還沒進版本庫)。
⇒ 本檔獨立成一份、**自帶完整的樁與參照實作**,不 import `test_mpng`、
  不依賴它的任何符號。兩份可以並存,也可以事後由人合併 —— 但合併前
  兩份都跑得過,不會有人因為合併失敗而少掉一整組鎖。

═══ 這一組鎖在守什麼 ═══

Jiang, Ge, Xu, Yang《Impact Time Control Cooperative Guidance Law Design
Based on Modified Proportional Navigation》(Aerospace 2021, 8(8):231)Eq.(41):

    a_M = N·V·λ̇  +  K_ε·N·V²·sin(σ)·ε_T  +  K_s·N·V²·sign(σ)·ε_T
          └ a_PNG ┘  └────── a_ε ───────┘   └────── a_s ────────┘
    ε_T ≡ t̄go − tgo (Eq.49)。**ε_T > 0 = 還有多的時間 = 會太早到。**

M1 開關關閉 ⇒ 逐位元同現行        M6 偏置**真的**疊進 a_h(不是算了沒接上)
M2 ε_T = 0 ⇒ 時間項恰為 0         M7 縱向通道零影響(保護 15/16 命中)
M3 ε_T 的符號方向                 M8 CSV 六個新欄的位置與空字串紀律
M4 σ = 0 時 sign 項仍出力          M9 Eq.(29) 的 t_go 只記錄不使用
M5 夾制真的咬 + req/cmd 語意

═══ ⭐⭐ 為什麼全部是「值層級」 ═══

本專案實證:源碼層的 38 條鎖被 **≥25 種等價改寫**繞過(改上游、改下游、
換容器、換命名、把式子搬進 helper —— 全部免疫);改成值層級之後 26 條
**一次關掉 8 個**。所以本檔一條字串比對都沒有:每一條都是
「餵一組數字進去 → 斷言送出來的數字」。

⚠️ 用 `exec` 原始碼而不是 `import tot_node`:ROS2 在 Windows 開發機上不可用,
   而這些鎖必須**每次都跑得到**(跑不到的鎖等於沒有鎖)。走原始碼也保證
   測的是「檔案裡真正在飛的那一份」,不是另外維護的複本。
"""
import ast
import csv
import itertools
import math
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOT = ROOT / "scripts" / "wsl" / "sar_swarm_patches" / "tot_node.py"
SRC = TOT.read_text(encoding="utf-8")
TREE = ast.parse(SRC, filename=str(TOT))

GRAV = 9.80665

#: 實測工作點(RRD1-8 + RRE1-8,n = 32 架次的中位)。⚠️ 這些不是拍的,是量的:
#: 放行斜距 6854 m、俯衝耗時 41.3 s、俯衝中位地速 161.7 m/s、放行 σ 15.3°。
#: 鎖住在這個工作點上的行為,才是鎖住「真的會發生的那一組數」。
R0, TGO0, V0, SIG0 = 6854.0, 41.3, 161.7, 15.3


def _func(name):
    for n in ast.walk(TREE):
        if isinstance(n, ast.FunctionDef) and n.name == name:
            return n
    raise AssertionError(f"tot_node.py 找不到 {name}()")


def _const(name):
    for n in TREE.body:
        if isinstance(n, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == name for t in n.targets):
            return ast.literal_eval(n.value)
    raise AssertionError(f"tot_node.py 找不到模組常數 {name}")


def _declared_default(name):
    """讀 `declare_parameter` 的**預設值**(不是它那一行長什麼樣)。

    節點用 `g("<名字>", <預設值>)` 宣告參數。這裡把那個字面值取出來,
    於是可以對「預設值是什麼」下值層級斷言 —— 換行、改排版、加註解、
    把 g() 換成別的 helper 都不影響(只要名字與字面值還在)。
    ⚠️ 不能 import tot_node(要 rclpy),所以走 AST。
    """
    for n in ast.walk(TREE):
        if (isinstance(n, ast.Call) and len(n.args) == 2
                and isinstance(n.args[0], ast.Constant)
                and n.args[0].value == name):
            return ast.literal_eval(n.args[1])
    raise AssertionError(f"tot_node.py 沒有宣告參數 {name}")


def _class_default(name):
    """讀節點類別上的**類別層級**預設值(樁與真節點共用的那一份)。"""
    for cls in (n for n in ast.walk(TREE) if isinstance(n, ast.ClassDef)):
        for st in cls.body:
            if isinstance(st, ast.Assign) and any(
                    isinstance(t, ast.Name) and t.id == name
                    for t in st.targets):
                return ast.literal_eval(st.value)
    raise AssertionError(f"tot_node.py 的類別上沒有 {name}")


def _block(name):
    """把一個方法的原始碼縮排成可以塞進 `class _M:` 的形狀。"""
    seg = ast.get_source_segment(SRC, _func(name))
    return "\n".join("    " + line for line in seg.splitlines())


def _mixin(names, nprime_2_minus_n=None):
    """把指定的方法**從檔案原始碼**綁成一個 mixin 類別。

    ⚠️ `nprime_2_minus_n` 用來在 M9 裡把模組常數換掉 —— 這是「值層級」與
       「源碼比對」的分水嶺:我們不是斷言某一行長什麼樣,而是**真的把那個
       常數換掉,再看指令有沒有動**。換容器、換命名、把式子搬進 helper
       都逃不掉這一條。
    """
    ns = {
        "math": math,
        "csv": csv,
        "EARTH": 111320.0,
        "mlon": lambda lat: 111320.0 * math.cos(math.radians(lat)),
        "MPNG_NPRIME_2_MINUS_N": (_const("MPNG_NPRIME_2_MINUS_N")
                                  if nprime_2_minus_n is None
                                  else nprime_2_minus_n),
    }
    exec("class _M:\n" + "\n\n".join(_block(n) for n in names), ns)
    return ns["_M"]


_GUID = _mixin(("_geom", "_itcg_bias", "_pn_cmd"))
_LOG = _mixin(("_open_csv", "_row"))


class _Log:
    """⚠️ 樁的 logger 不可以吞掉例外以外的東西 —— 它只是不印。"""

    def info(self, *a, **k):
        pass

    def warn(self, *a, **k):
        pass

    def error(self, *a, **k):
        pass


# ══════════════════════════════════════════════════════════════════════════
#  導引樁:只提供 `_geom` / `_itcg_bias` / `_pn_cmd` 真的會碰到的東西
# ══════════════════════════════════════════════════════════════════════════
class Guid(_GUID):
    """⚠️ 刻意**不提供**多餘屬性:多給一個,就會把「忘了初始化某個 dict」
    這一整類失效在測試裡蓋掉,而那正是本專案的頭號失效類別。"""

    # ── ITCG / MPNG ──(RRE 契約快照值:itcg_a_max_g = 0.4)
    itcg_on = True
    itcg_gain = 1.0
    itcg_a_max_g = 0.4
    mpng_on = False
    mpng_k_eps = 0.0
    mpng_k_s = 0.0
    # ── PN 三通道 ──
    n_lat = 4.0
    n_vert = 4.0
    pn_lead = 1.0
    pn_vmax = 210.0
    pn_los_floor = 0.0          # 0 = 關掉幾何前饋,讓縱向解只由 PN 決定
    pn_nz_min = -1.0
    pn_alpha_tau = 1.75
    tick_dt = 0.05
    dive_pitch_max = 0.0
    dive_pitch_min = -80.0
    pn_thr_trim = 0.36
    pn_thr_gain = 0.02
    pn_late_full = True
    dive_full_thr = False
    fz_r = 100.0
    fz_tgo = 0.5

    def __init__(self, tgo=TGO0, d=6142.0, h=3100.0, vh=145.0,
                 sigma_deg=SIG0, gam_deg=-30.0, alpha_deg=0.0):
        self.itcg_e, self.itcg_s, self.itcg_side = {}, {}, {}
        self.mpng_k0, self.itcg_dbg, self.itcg_a = {}, {}, {}
        self.alpha = {}
        # ⭐⭐ `alpha_deg = 0` 讓現場估計的攻角 α **恰好**是 0:
        #    α_raw = θ_meas − γ,而 α[i] 的初值就是 α_raw(`self.alpha.get(i, a_raw)`)
        #    ⇒ θ_meas = γ 時 α ≡ 0,於是 `pitch = gam_cmd + α/max(cos roll, 0.5)`
        #      退化成 `pitch == gam_cmd`。
        #    ⚠️ 這不是為了讓測試好過,是為了讓 M7 能對**真正回傳的 pitch**
        #      下逐位元斷言:α ≠ 0 時 pitch 透過 cos(roll) 與橫向耦合,
        #      `(a+b)−b == a` 在浮點下**不成立**,反解出來的 gam_cmd 會有
        #      1e-16 的殘差,那會讓「縱向被動到了」與「浮點捨入」分不開。
        #      需要有耦合的那一條鎖另外用 `alpha_deg != 0` 建樁。
        self.att = {1: (0.0, math.radians(gam_deg + alpha_deg), 0.0)}
        self.dive_T = 1000.0
        self.tgo = tgo
        self.T = (24.0, 121.0)
        self.pos = {1: (24.0 - d / 111320.0, 121.0, h)}
        a = math.radians(sigma_deg)
        hdot = -vh * math.tan(math.radians(-gam_deg))
        self.vel = {1: (vh * math.cos(a), vh * math.sin(a), -hdot)}

    def _zg(self, i):
        return 0.0

    def _tgo_of(self, i, d):
        return self.tgo

    def get_logger(self):
        return _Log()

    # ── 給測試用的取值器(不是被測碼的一部分)────────────────────────
    def now_for(self, early_s):
        """回傳「早到 early_s 秒」對應的牆鐘時刻。ε_T = early_s。"""
        return self.dive_T - (self.tgo + early_s)

    def bias(self, early_s=3.7):
        """直接呼叫 `_itcg_bias`,幾何由 `_geom` 供給(與飛行時同一條路)。"""
        dn, de, d, h, R, vn, ve, vh, hdot, V = self._geom(1)
        return self._itcg_bias(1, self.now_for(early_s), dn, de, d, R,
                               vn, ve, vh, V)


def _guid(**kw):
    """⚠️ 每一次都給**全新**的樁:`alpha` / `itcg_side` / `mpng_k0` 都是
    跨拍鎖存的狀態,重用會讓後一次呼叫吃到前一次的鎖存值。"""
    ctor = {k: kw.pop(k) for k in
            ("tgo", "d", "h", "vh", "sigma_deg", "gam_deg", "alpha_deg")
            if k in kw}
    s = Guid(**ctor)
    for k, v in kw.items():
        assert hasattr(Guid, k) or hasattr(s, k), f"樁沒有 {k},打錯字了"
        setattr(s, k, v)
    return s


def _pn(s, early_s=3.7):
    """跑一拍 `_pn_cmd`,把**中間量從輸出反解出來**。

    ⭐⭐ 這是本檔 M6/M7 的核心手法:`gam_cmd` / `a_h` 都是 `_pn_cmd` 的區域
       變數,外面看不到。但它們與回傳值之間是**代數可逆**的:

           roll    = atan2(a_h, g·max(cos γ, 0.25))     ⇒ a_h = tan(roll)·g·max(cos γ, 0.25)
           pitch   = gam_cmd + α/max(cos roll, 0.5)     ⇒ gam_cmd = pitch − α/max(cos roll, 0.5)

       ⇒ 不必碰源碼、不必開後門,就能對**縱向解**下逐位元斷言。
    ⚠️ 反解只在夾制沒有咬到時成立(roll 的 r_lim、pitch 的 ±dive_pitch_*)。
       下面的工作點刻意離兩個夾制都很遠(roll ≈ 2°、pitch ≈ −25°),
       `test_the_reconstruction_is_exact` 會在每次跑之前先驗證這件事。
    """
    roll, pitch, thr, R, V = s._pn_cmd(1, s.now_for(early_s))
    dn, de, d, h, R2, vn, ve, vh, hdot, Vv = s._geom(1)
    gam = math.atan2(hdot, max(vh, 1e-3))
    th_meas = s.att[1][1]
    return {
        "roll": roll, "pitch": pitch, "thr": thr, "R": R, "V": V,
        "a_h": math.tan(roll) * GRAV * max(math.cos(gam), 0.25),
        "gam_cmd": pitch - s.alpha[1] / max(math.cos(roll), 0.5),
        "alpha": s.alpha[1],
        "a_t": s.itcg_a[1],
        "gam": gam,
        # 純 PN 的橫向項 a_PNG = N·vh·λ̇(獨立算一次,不從被測碼拿)
        "a_pn": s.n_lat * vh * ((de * vn - dn * ve) / (d * d)),
        # ⚠️ 兩個夾制的貼邊旗標。反解只在**沒有貼邊**時成立 —— 貼邊時
        #    `pitch` 已經是常數,反解出來的 gam_cmd 是垃圾,而垃圾在兩組
        #    之間「剛好相同」會給假綠燈。
        "pitch_clamped": not (math.radians(s.dive_pitch_min) + 1e-9 < pitch
                              < math.radians(s.dive_pitch_max) - 1e-9),
        "roll_clamped": abs(roll) >= math.radians(80.0) * abs(
            math.cos(th_meas)) - 1e-9,
    }


# ══════════════════════════════════════════════════════════════════════════
#  M1 ⭐⭐ 開關關閉 ⇒ **逐位元**同現行
# ══════════════════════════════════════════════════════════════════════════
class TestM1DefaultOffIsBitIdentical:
    """⚠️⚠️ 這是**對照組不被污染的唯一保證**。

    沒有這一條,「開了之後變差」與「開之前就已經壞了」永遠分不開 ——
    而那正是 RRA 整批 8 趟 / 5 小時報廢之後才學到的東西。
    """

    @staticmethod
    def _reference(s, i, now, dn, de, d, R, vn, ve, vh, V):
        """2026-08-15 版 `_itcg_bias`(surplus / σ_need 反解)的**獨立參照**。

        ⚠️ 故意寫死在測試裡:它就是「舊行為」的定義。若有人同時改了生產碼
           與這一份,兩邊會一起錯 —— 所以旁邊釘著實測工作點(R0/TGO0/V0/SIG0)
           當現實錨點,而 `test_the_reference_reproduces_a_measured_number`
           會拿一個算得出來的數字對它。
        """
        if not s.itcg_on or s.dive_T is None:
            return 0.0, None
        t_rem = s.dive_T - now
        if t_rem <= 0.0:
            return 0.0, None
        e_naive = V * t_rem - R
        t_go = s._tgo_of(i, d)
        e_s = t_rem - t_go
        surplus = R * (e_s / max(t_go, 1e-3))
        if e_s <= 0.0 or d < s.fz_r:
            return 0.0, e_naive
        sigma = math.atan2(dn * ve - de * vn, dn * vn + de * ve)
        side = s.itcg_side.get(i)
        if side is None:
            if abs(sigma) < math.radians(2.0):
                return 0.0, e_naive
            side = 1.0 if sigma > 0.0 else -1.0
            s.itcg_side[i] = side
        frac = surplus / max(R, 1.0)
        sig_need = math.acos(max(-1.0, min(1.0, 1.0 / (1.0 + max(frac, 0.0)))))
        a_t = s.itcg_gain * s.n_lat * vh * V * math.sin(sig_need) / max(R, 1.0)
        cap = s.itcg_a_max_g * GRAV
        return max(-cap, min(cap, a_t * side)), e_naive

    #: 格點刻意覆蓋**每一條分支**:遲到 / 準時 / 早到、凍結內外、死區內外、
    #: σ 兩側、σ 到 81°(RRE 實測最大)、增益 0 與飽和、夾制三檔。
    GRID = list(itertools.product(
        [-11.3, 0.0, 3.7, 20.0, 60.0],     # early_s(= ε_T;負 = 已經遲到)
        [60.0, 3000.0, 6142.0],            # d(60 < fz_r ⇒ 末端凍結)
        [142.7, V0, 200.0],                # vh
        # σ(度)。⚠️ ±1.95 / ±2.05 是**死區邊界的探針**:突變測試證實
        #    少了它們,「把死區從 2.0° 改成 1.9°」這種改動會**整批綠燈通過**
        #    (2026-08-19 實測,20 個突變裡唯一活下來的那一個)。
        #    死區寬度直接決定「第幾拍才鎖定繞路方向」,而方向鎖定的時機
        #    就是側向抖振的開關 —— 不是無關緊要的常數。
        [0.5, -0.5, 1.95, -1.95, 2.05, 10.0, -35.0, 81.0],
        [20.0, TGO0, 60.0],                # t_go
        [0.0, 1.0, 3.5],                   # itcg_gain
        [0.1, 0.4, 1.0]))                  # itcg_a_max_g(契約區間 0.1–1.0)

    def test_the_switch_defaults_to_off_in_the_node_itself(self):
        """⚠️⚠️ 讀的是**節點真正宣告的預設值**,不是樁上的那一份 ——
        對著自己的樁斷言「預設是 False」是在測試自己,零價值。

        兩處都要:`declare_parameter` 的預設(節點起來時吃到的),
        以及類別層級的預設(參數還沒讀進來、或樁跳過 __init__ 時吃到的)。
        任何一處被改成 True,對照組就整批失效而且**零告警**。
        """
        assert _declared_default("mpng_on") is False, (
            "⚠️⚠️ declare_parameter 的預設不是 False ⇒ 沒有人指定 MPNG_ON 時"
            "節點就自己走新律了,而契約行看起來完全正常。")
        assert _class_default("mpng_on") is False
        assert _declared_default("mpng_k_eps") == 0.0, (
            "0 = 走論文建議式自動推;改成非零等於偷偷指定了一個沒飛過的增益")
        assert _declared_default("mpng_k_s") == 0.0
        assert Guid.mpng_on is _class_default("mpng_on"), (
            "樁的預設與節點不一致 ⇒ 下面整組比對測的不是真的預設路徑")

    def test_every_grid_point_is_bit_identical_to_the_old_law(self):
        bad = []
        for early, d, vh, sd, tgo, gain, cap in self.GRID:
            new = _guid(tgo=tgo, d=d, vh=vh, sigma_deg=sd,
                        itcg_gain=gain, itcg_a_max_g=cap)
            ref = _guid(tgo=tgo, d=d, vh=vh, sigma_deg=sd,
                        itcg_gain=gain, itcg_a_max_g=cap)
            assert new.mpng_on is False
            dn, de, dd, h, R, vn, ve, vhh, hdot, V = new._geom(1)
            now = new.now_for(early)
            got = new._itcg_bias(1, now, dn, de, dd, R, vn, ve, vhh, V)
            want = self._reference(ref, 1, now, dn, de, dd, R, vn, ve, vhh, V)
            if got != want or new.itcg_side != ref.itcg_side:
                bad.append((early, d, vh, sd, tgo, gain, cap, got, want))
        assert not bad, (
            f"⚠️⚠️ `mpng_on=False` 的路徑與 2026-08-15 版**不再逐位元相同**"
            f"({len(bad)}/{len(self.GRID)} 組不符)。前三組:{bad[:3]}\n"
            f"⇒ 這一批資料與 RRD/RRE 不可比,而且沒有任何一層會告訴你。")

    def test_the_grid_really_exercises_the_actuation_path(self):
        """⚠️ 反向鎖:格點若全部落在「不出力」上,上一條會**空轉通過**。

        本專案已經有過同構的假綠燈(S2 的 `cmd_radius_m` 在開與關時同樣是
        30)—— 一條永遠比較 0.0 == 0.0 的鎖比沒有鎖更糟,因為它會給人信心。
        """
        fired = sum(
            1 for early, d, vh, sd, tgo, gain, cap in self.GRID
            if abs(_guid(tgo=tgo, d=d, vh=vh, sigma_deg=sd, itcg_gain=gain,
                         itcg_a_max_g=cap).bias(early)[0]) > 0.0)
        assert fired > 100, f"格點只有 {fired} 組真的出力 —— 比對等於沒測到"

    def test_the_reference_reproduces_a_measured_number(self):
        """參照實作的現實錨點:在實測工作點上,舊律**必然飽和**。

        σ_need = acos(1/(1+surplus/R));surplus = R·e_s/t_go ⇒ 只看 e_s/t_go。
        early = 3.7 s、t_go = 41.3 s ⇒ frac = 0.0896、σ_need = 23.9°、
        a = 1.0·4·(145·0.9?)… 量級 ~4 m/s² > 0.4 g = 3.92 ⇒ 夾制咬住。
        """
        s = _guid()
        a, _ = s.bias(3.7)
        assert abs(a) == pytest.approx(0.4 * GRAV, rel=1e-12), (
            "舊律在實測工作點上不再飽和 —— 參照實作或工作點其中之一被改動了")


# ══════════════════════════════════════════════════════════════════════════
#  M2 ⭐⭐ ε_T = 0 ⇒ 時間項**恰為** 0(不是「很小」)
# ══════════════════════════════════════════════════════════════════════════
class TestM2ZeroTimeErrorMeansPurePN:
    """原文行 735 逐字:ε_T = 0 時「the law proposed in this paper is
    equivalent to the PNG law」。

    ⚠️⚠️ 使用者硬性要求:**精度不在授權範圍內**。目前命中誤差
       0.92–5.36 m / 15+ 架是唯一達標的 KPI,任何在終端多加一個橫向指令的
       行為都會直接動到它。所以這裡斷言的是 `== 0.0`,不是 `< ε`:
       「很小」會隨工作點漂,「恰為零」不會。
    """

    @pytest.mark.parametrize("sigma_deg", [0.0, 5.0, 15.3, -35.0, 81.0])
    @pytest.mark.parametrize("mpng_on", [False, True])
    def test_zero_epsilon_gives_exactly_zero(self, sigma_deg, mpng_on):
        s = _guid(sigma_deg=sigma_deg, mpng_on=mpng_on)
        a, _ = s.bias(0.0)
        assert a == 0.0, (
            f"⚠️⚠️ ε_T = 0(σ={sigma_deg}°、mpng_on={mpng_on})卻仍出力 {a} —— "
            f"那是在終端加一個沒有理由的橫向指令,直接動到已達標的命中誤差。")

    def test_the_command_collapses_onto_pure_pn_at_zero_epsilon(self):
        """⭐ 端到端:ε_T = 0 時 `_pn_cmd` 的橫向解必須**逐位元**等於純 PN。

        ⚠️ 這一條比上面那條強:上面鎖的是 `_itcg_bias` 的回傳值,這一條鎖的是
           「最後真正送出去的那個 roll」—— 中間任何一層偷加東西都會被抓到。
        """
        on = _pn(_guid(mpng_on=True), early_s=0.0)
        assert on["a_t"] == 0.0
        assert on["a_h"] == pytest.approx(on["a_pn"], rel=1e-12, abs=1e-12), (
            "ε_T = 0 時橫向指令不等於純 PN ⇒ Eq.(41) 沒有退化回 PNG")

    def test_epsilon_zero_is_not_a_trivially_dead_configuration(self):
        """⚠️ 反向鎖:同一組樁在 ε_T ≠ 0 時必須**真的**出力,否則上面那三條
        全部是在對一條本來就死掉的通道說「它是 0」。"""
        assert abs(_guid(mpng_on=True).bias(3.7)[0]) > 0.0


# ══════════════════════════════════════════════════════════════════════════
#  M3 ⭐⭐ ε_T 的符號方向:太早到 ⇒ 航程要**變長**
# ══════════════════════════════════════════════════════════════════════════
class TestM3TheSignOfEpsilonPointsTheRightWay:
    """⚠️⚠️ 搞反的後果不是「效果差」,是**越修越早**:飛機會為了「趕時間」
    而把航程縮短,而它本來就已經太早到了 —— 正回饋。

    ⭐ 怎麼在**值層級**判斷「航程變長」?看它與純 PN 的**相對符號**:
       PN 的工作就是把前置角 σ 收掉(走最短路徑)。要繞遠路 ⇔ 要把 σ 撐開
       ⇔ 時間項必須與 a_PNG **反號**。這個判準不依賴任何座標約定,
       換半球、換目標方位、把 dn/de 交換都成立。
    """

    @pytest.mark.parametrize("sigma_deg", [10.0, 35.0, -10.0, -35.0])
    def test_the_time_term_opposes_the_pn_term_when_early(self, sigma_deg):
        r = _pn(_guid(sigma_deg=sigma_deg, mpng_on=True), early_s=3.7)
        assert r["a_pn"] != 0.0 and r["a_t"] != 0.0
        assert r["a_t"] * r["a_pn"] < 0.0, (
            f"⚠️⚠️ σ={sigma_deg}°:時間項 {r['a_t']:+.3f} 與 PN 項 "
            f"{r['a_pn']:+.3f} **同號** ⇒ 它在幫 PN 把 σ 收掉 = 把航程"
            f"**縮短** —— 而 ε_T > 0 的意思是已經太早到了。方向反了。")

    @pytest.mark.parametrize("sigma_deg", [10.0, -10.0])
    def test_the_time_term_pushes_the_lateral_command_away_from_pn(self, sigma_deg):
        """端到端:疊加後的 a_h 必須比純 PN **更偏離**收斂方向。"""
        off = _pn(_guid(sigma_deg=sigma_deg, itcg_on=False), early_s=3.7)
        on = _pn(_guid(sigma_deg=sigma_deg, mpng_on=True), early_s=3.7)
        assert abs(on["a_h"]) < abs(off["a_h"]) or (
            on["a_h"] * off["a_h"] < 0.0), (
            "疊加後的橫向指令沒有往「少收一點 σ」的方向走")

    def test_being_late_never_produces_a_lateral_command(self):
        """ε_T < 0(已經遲到)⇒ 不出力。

        ⭐ 理由是幾何硬事實,不是保守:Kim & Shin 2023 eq.6
        `de_t/dt = −V(1 − cos σ) ≤ 0` —— 橫向機動**只能**加長航程。
        直線已經是最短路徑,遲到時再下橫向指令是純浪費(而且會讓遲到更嚴重)。
        ⚠️ 但 req 欄必須看得見它「想反向出力」,否則事後分不出
           「律沒出力」是因為遲到、因為凍結、還是因為根本沒跑到。
        """
        s = _guid(mpng_on=True)
        a, _ = s.bias(-11.3)
        assert a == 0.0
        dbg = s.itcg_dbg[1]
        assert dbg["eps_t"] < 0.0, "ε_T 必須記成負的(= 遲到)"
        assert dbg["req_a"] < 0.0, (
            "req 必須記下「它想反向出力」—— 那是判讀遲到分佈的唯一訊號")

    def test_the_requested_magnitude_grows_with_the_time_error(self):
        """⚠️ 單調性:ε_T 越大,要求的偏置越大。

        不是廢話 —— 它擋掉「把 ε_T 取絕對值」「把符號吃進 side」這一類
        會讓律對誤差大小失去反應的改寫。用 req(夾制**前**)才看得到,
        用 cmd 會整段貼在夾制上而看不出任何差別。
        """
        reqs = []
        for early in (0.5, 1.0, 2.0, 4.0, 8.0):
            s = _guid(mpng_on=True)
            s.bias(early)
            reqs.append(abs(s.itcg_dbg[1]["req_a"]))
        assert reqs == sorted(reqs) and reqs[0] < reqs[-1], (
            f"要求值對 ε_T 不單調:{reqs}")
        # 線性:Eq.(41) 的兩項都正比於 ε_T ⇒ 比值必須等於 ε_T 的比值
        assert reqs[-1] / reqs[0] == pytest.approx(8.0 / 0.5, rel=1e-9)


# ══════════════════════════════════════════════════════════════════════════
#  M4 ⭐⭐ σ = 0 時 sign 項仍出力(a_s 存在的**唯一**理由)
# ══════════════════════════════════════════════════════════════════════════
class TestM4TheSignTermSolvesTheZeroLeadAngleStart:
    """σ = 0 時 λ̇ = 0 ⇒ a_PNG = 0,sin σ = 0 ⇒ a_ε = 0 —— 整條律會卡死。
    Eq.(42) 把 `sign(0)` 定義成 **+1**(不是 0)就是為了讓
    a_s = K_s·N·V²·ε_T ≠ 0 把 σ 推離零(原文行 843-844:"effective even if
    the initial leading angle is zero ... has no singularity")。

    ⚠️⚠️ 這一條對我們**特別**要緊:32 架次實測俯衝段 σ 中位只有 **0.2°** ——
       我們幾乎整段都在 σ≈0 這個區,而那正是 a_ε 完全沒有出力的區。
       換句話說:在我們的工作點上,**出力的其實幾乎全部是 a_s**。
    """

    def test_at_exactly_zero_lead_angle_the_law_still_fires(self):
        s = _guid(sigma_deg=0.0, mpng_on=True)
        s.itcg_side[1] = 1.0            # 死區之外已經鎖過方向(見下一條)
        a, _ = s.bias(3.7)
        assert a != 0.0, (
            "⚠️⚠️ σ = 0 時整條律卡死 —— a_s 沒有接上。Eq.(41) 加這一項的"
            "唯一理由就是這個啟動問題,而我們俯衝段 σ 中位只有 0.2°。")

    def test_at_zero_lead_angle_the_epsilon_term_contributes_nothing(self):
        """⭐ 值層級地證明「σ=0 時出的力**全部**來自 a_s」:
        把 K_ε 放大 1000 倍,輸出必須**逐位元不變**(因為 sin 0 = 0)。"""
        outs = []
        for k_eps in (1e-9, 1e-6):
            s = _guid(sigma_deg=0.0, mpng_on=True,
                      mpng_k_eps=k_eps, mpng_k_s=2e-9)
            s.itcg_side[1] = 1.0
            outs.append(s.bias(3.7)[0])
        assert outs[0] == outs[1] != 0.0, (
            f"σ=0 時 K_ε 竟然影響了輸出 {outs} ⇒ a_ε 不是乘 sin(σ),"
            f"或者 σ 沒有真的是 0")

    def test_the_zero_lead_angle_output_equals_the_closed_form(self):
        """a_s = K_s·N·V²·ε_T。獨立算一次,逐位元比對(夾制前的 req)。"""
        s = _guid(sigma_deg=0.0, mpng_on=True, mpng_k_eps=1e-9, mpng_k_s=2e-9)
        s.itcg_side[1] = 1.0
        s.bias(3.7)
        V = s._geom(1)[9]
        want = 2e-9 * s.n_lat * V * V * 3.7
        assert s.itcg_dbg[1]["req_a"] == pytest.approx(want, rel=1e-9), (
            "σ=0 的閉式對不上 ⇒ a_s 的量綱或組成被改過")

    def test_the_deadband_still_refuses_to_latch_a_direction(self):
        """⚠️ 我們的死區(|σ| < 2°)會擋住**還沒鎖過方向**的第一拍 ——
        這是刻意的取捨:寧可晚一拍鎖方向,也不要把雜訊鎖成方向。

        ⇒ M4 的「σ=0 仍出力」是「**一旦方向定了**,σ=0 也推得動」,
          不是「σ=0 的第一拍就會動」。兩者混為一談會讓人以為鎖壞了。
        """
        s = _guid(sigma_deg=0.0, mpng_on=True)
        assert s.bias(3.7)[0] == 0.0 and 1 not in s.itcg_side
        assert s.itcg_dbg[1]["req_a"] != 0.0, "死區也要留下觀測值"

    def test_the_deadband_edge_sits_at_exactly_two_degrees(self):
        """⭐ 死區寬度是**側向抖振的開關**,不是一個無關緊要的常數:
        它決定「第幾拍才鎖定繞路方向」,而方向一旦鎖定就整段不再改。

        ⚠️ 突變測試(2026-08-19)實測:把它從 2.0° 改成 1.9°,20 個突變裡
           **只有這一個活下來** —— 因為原本的格點沒有任何一點落在 1.9–2.0°
           之間。所以這裡直接踩兩側邊界。
        """
        for sd, want_latch in ((1.95, False), (2.05, True),
                               (-1.95, False), (-2.05, True)):
            s = _guid(sigma_deg=sd, mpng_on=True)
            a, _ = s.bias(3.7)
            assert (1 in s.itcg_side) is want_latch, (
                f"σ={sd}° 的鎖定行為與 2.0° 死區不符 ⇒ 死區寬度被改動了")
            assert (a != 0.0) is want_latch

    def test_the_latched_side_never_flips_when_sigma_crosses_zero(self):
        """⚠️⚠️ Eq.(42) 的 sign(σ) 在 σ 過零時整項**跳 2 倍幅值**,而 a_s 的
        大小與 σ 無關 ⇒ 那是一個純粹的不連續。我們俯衝段 σ 中位 0.2°,
        整段都貼在這個不連續上。

        [[stot-altitude-gate-chatter]]:閘門沒遲滯 ⇒ 翻轉次數直接寫進末端
        誤差,UAV5 翻 20 次 → 35.6 km。同構的失效,已經發生過一次。
        """
        s = _guid(sigma_deg=10.0, mpng_on=True)
        a1, _ = s.bias(3.7)
        assert s.itcg_side[1] == 1.0
        for sd in (-10.0, +40.0, -81.0, 0.0):
            a = math.radians(sd)
            vh = math.hypot(s.vel[1][0], s.vel[1][1])
            s.vel[1] = (vh * math.cos(a), vh * math.sin(a), s.vel[1][2])
            ai, _ = s.bias(3.7)
            assert s.itcg_side[1] == 1.0, f"σ={sd}° 時方向鎖存被改寫"
            assert ai * a1 > 0.0, f"σ={sd}° 時偏置翻面 —— 2 倍跳變回來了"


# ══════════════════════════════════════════════════════════════════════════
#  M5 ⭐⭐ 夾制真的咬 + req/cmd 語意
# ══════════════════════════════════════════════════════════════════════════
class TestM5TheClampBitesAndReqIsPreClamp:
    """⚠️⚠️ **判定量是 req(每拍都寫、夾制前),不是 cmd(hold-last-value)。**

    S2 那一輪的 `cmd_radius_m` 在開關開與關時同樣是 30 —— 拿它當「機制生效」
    的證據會給**假綠燈**。這一組鎖就是不讓同一件事再發生第七次。

    ⭐⭐ 而且夾制是這一輪**真正的自變數**:實測幾何重放(n=32)
       itcg_a_max_g = 0.4 ⇒ 時間權限 **+1.45 s**;= 1.0 ⇒ **+10.59 s**;
       而 n=8 同時到達的需求是 **7.3–9.0 s**。
       ⇒ 這條律能不能用,不取決於 K_ε/K_s,取決於敢不敢把夾制開到接近契約上限。
    """

    @pytest.mark.parametrize("cap_g", [0.1, 0.4, 1.0])
    def test_the_clamp_is_never_exceeded(self, cap_g):
        s = _guid(mpng_on=True, itcg_a_max_g=cap_g)
        a, _ = s.bias(60.0)                       # 故意灌爆
        assert abs(a) <= cap_g * GRAV + 1e-12, (
            "⚠️⚠️ 時間項壓過 PN 的命中通道 —— 命中誤差是目前唯一達標的 KPI")
        assert abs(a) == pytest.approx(cap_g * GRAV, rel=1e-12), (
            "飽和時輸出必須**正好**貼在夾制上,不是「差不多」")

    def test_req_records_the_pre_clamp_value_and_cmd_the_post(self):
        s = _guid(mpng_on=True)
        a, _ = s.bias(60.0)
        dbg = s.itcg_dbg[1]
        cap = s.itcg_a_max_g * GRAV
        assert dbg["cmd_a"] == a, "cmd 欄必須就是真的疊進 a_h 的那個值"
        assert abs(dbg["cmd_a"]) == pytest.approx(cap, rel=1e-12)
        assert abs(dbg["req_a"]) > abs(dbg["cmd_a"]) * 10.0, (
            f"req {dbg['req_a']:.1f} 與 cmd {dbg['cmd_a']:.2f} 太接近 —— "
            f"req 很可能記成了夾制**後**的值,那樣就完全看不出權限不足。")
        assert dbg["sat"] == 1, "飽和旗標沒有升起"

    def test_req_and_cmd_agree_exactly_when_the_clamp_does_not_bite(self):
        """⚠️ 反向鎖:沒飽和時兩欄必須**相等**且 sat = 0。
        否則「req 一直比 cmd 大」會被誤讀成「一直在飽和」。"""
        s = _guid(mpng_on=True, mpng_k_eps=1e-9, mpng_k_s=1e-9)
        a, _ = s.bias(3.7)
        dbg = s.itcg_dbg[1]
        assert dbg["sat"] == 0
        assert dbg["req_a"] == dbg["cmd_a"] == a != 0.0

    def test_the_saturation_flag_tracks_the_clamp_not_the_gain(self):
        """整段 sat = 1 ⇒ 律退化成 bang-bang ⇒ **調增益無用,只能動夾制**。

        這條把那個結論鎖成可檢驗的:同一個 ε_T 下,把夾制放大 10 倍會讓
        cmd 放大 10 倍(因為兩邊都飽和),而把增益放大 10 倍**完全沒有反應**。
        """
        base = _guid(mpng_on=True, itcg_a_max_g=0.1)
        big_cap = _guid(mpng_on=True, itcg_a_max_g=1.0)
        big_gain = _guid(mpng_on=True, itcg_a_max_g=0.1,
                         mpng_k_eps=1e-2, mpng_k_s=1e-2)
        a0, a_cap, a_gain = (base.bias(3.7)[0], big_cap.bias(3.7)[0],
                             big_gain.bias(3.7)[0])
        assert a_cap == pytest.approx(a0 * 10.0, rel=1e-9), "夾制沒有線性放行"
        assert a_gain == a0, (
            "⚠️ 增益放大 100 倍竟然改變了輸出 —— 那表示這一點沒有飽和,"
            "與「整段 bang-bang」的推算不符,先查 K 的自動推導。")

    def test_the_paper_gain_is_latched_at_release_not_recomputed(self):
        """K = 40/(r₀·tgo₀)(原文行 1149-1153,N=3)。量綱 1/(m·s) ✓。

        ⚠️ 不鎖存 ⇒ 增益隨 R 一路飄,而 R 在俯衝段從 6854 m 掉到 0 ⇒
           同一個 ε_T 在不同距離會給出差三個數量級的指令。
        """
        s = _guid(mpng_on=True)
        s.bias(3.7)
        k_first = s.mpng_k0[1]
        R = s._geom(1)[4]
        assert k_first == pytest.approx(40.0 / (R * TGO0), rel=1e-9)
        s.pos = {1: (24.0 - 1200.0 / 111320.0, 121.0, 600.0)}   # 逼近紅心
        s.bias(3.7)
        assert s.mpng_k0[1] == k_first, "增益沒有鎖存,會隨距離飄"


# ══════════════════════════════════════════════════════════════════════════
#  M6 ⭐⭐ 偏置**真的**疊進 a_h(防「算了但沒接上」—— 已發生五次)
# ══════════════════════════════════════════════════════════════════════════
class TestM6TheBiasReallyReachesTheRollCommand:
    """⚠️⚠️ 本專案的頭號失效類別:「開關開著,但那一段從來沒真的發生」。
    已經**第五次**(JET_CDP 空字串 / predict_hz=0 / pn_los_floor=0 /
    catch_speed 被 FC 整發拒收 / itcg_accel 80,760 樣本全零)。

    ⇒ 只斷言 `_itcg_bias` 回傳非零是**不夠的** —— 那正是前五次都會通過的鎖。
      這一組一路跟到 `_pn_cmd` 真正回傳的 roll。
    """

    def test_the_reconstruction_is_exact(self):
        """先驗證反解本身:關掉時間項時,反解出來的 a_h 必須**逐位元**等於
        獨立算的 a_PNG = N·vh·λ̇。⚠️ 這條顧的是測試自己 —— 反解若被夾制
        污染,下面每一條都會變成在比較兩個垃圾。"""
        r = _pn(_guid(itcg_on=False))
        assert r["a_t"] == 0.0
        assert r["a_h"] == pytest.approx(r["a_pn"], rel=1e-12, abs=1e-12)
        assert abs(r["roll"]) < math.radians(70.0), "roll 已接近夾制,反解不可信"
        assert math.radians(-70.0) < r["pitch"] < math.radians(-5.0), (
            "pitch 已接近夾制,gam_cmd 的反解不可信")

    @pytest.mark.parametrize("cap_g", [0.1, 0.4, 1.0])
    def test_the_lateral_command_moves_by_exactly_the_bias(self, cap_g):
        """⭐⭐ 值層級的「接上了」:a_h(開) − a_h(關) 必須**逐位元**等於 a_t。

        ⚠️ 這比「roll 有變」強得多:roll 有變可能是任何東西造成的。
           要求差值**恰好**等於偏置,才排除掉「接上了但接錯地方 / 乘了係數 /
           被別的項吃掉一部分」。
        """
        off = _pn(_guid(itcg_on=False, itcg_a_max_g=cap_g))
        on = _pn(_guid(mpng_on=True, itcg_a_max_g=cap_g))
        assert on["a_t"] != 0.0, "偏置本身就是 0,這一條測不到東西"
        assert on["roll"] != off["roll"], (
            "⚠️⚠️ 偏置非零卻沒有改變 roll ⇒ **算了但沒接上**(第六次)")
        assert on["a_h"] - off["a_h"] == pytest.approx(
            on["a_t"], rel=1e-9, abs=1e-9), (
            f"a_h 只動了 {on['a_h'] - off['a_h']:+.4f},但偏置是 "
            f"{on['a_t']:+.4f} ⇒ 中間有人把它縮放或吃掉了一部分")

    def test_the_bias_is_also_published_for_the_csv(self):
        """`itcg_a[i]` 是 CSV 的 `itcg_accel` 欄。⚠️ 它必須是**真的送進 a_h**
        的那一份,不是另外算一次 —— 兩份會在重構時默默分岔。"""
        s = _guid(mpng_on=True)
        r = _pn(s)
        assert s.itcg_a[1] == r["a_t"]
        assert s.itcg_dbg[1]["cmd_a"] == s.itcg_a[1], (
            "CSV 的 itcg_accel 與 itcg_cmd_a 竟然不同源 ⇒ 兩欄會分岔")

    def test_turning_the_master_switch_off_removes_the_bias_entirely(self):
        """⚠️ `mpng_on=1` 而 `itcg_on=0` ⇒ `_itcg_bias` 第一行就 return,
        整條律**一次都不會被呼叫到**,而契約快照仍然印「MPNG 已啟用」。
        這裡把那個組合的**數值後果**釘住。"""
        r = _pn(_guid(mpng_on=True, itcg_on=False))
        assert r["a_t"] == 0.0
        assert r["a_h"] == pytest.approx(r["a_pn"], rel=1e-12, abs=1e-12)


# ══════════════════════════════════════════════════════════════════════════
#  M7 ⭐⭐ 縱向通道零影響(保護唯一達標的 KPI)
# ══════════════════════════════════════════════════════════════════════════
class TestM7TheVerticalChannelIsUntouched:
    """⭐ 縱向管角度、側向管時間 —— Sheng Yi 2021 與 Jiang 2021 兩篇獨立同結論。
    Jiang 的 3D 版 Eq.(68) 也只動 yaw 通道,pitch 是 `γ̇ = N·λ̇` 純 PN、
    **零時間項**,與我們的掛法一致。

    ⚠️⚠️ 命中誤差(0.92–5.36 m)是目前**唯一**達標的 KPI,而使用者明訂
       「精度不在授權範圍內」。縱向解只要動一個位元,這一輪的資料就不能
       用來回答「時間權限有多少」—— 因為精度會同時變,兩者分不開。
    """

    #: ⚠️⚠️ 這些格點刻意都讓 **α ≡ 0**(θ_meas = γ),理由見 `Guid.__init__`:
    #:    α ≠ 0 時 `pitch = gam_cmd + α/max(cos roll, 0.5)` 會透過 cos(roll)
    #:    與橫向耦合 —— 那是**座標轉換**,不是導引律在動縱向。
    #:    在 α = 0 的格點上 `pitch == gam_cmd`,於是可以對**真正回傳的 pitch**
    #:    下逐位元斷言,不必經過會掉精度的反解。
    #:    有耦合的情形由 `test_the_only_vertical_difference_is_the_known_roll_
    #:    coupling` 單獨鎖住,免得「pitch 不同是正常的」變成萬用免死金牌。
    #: ⚠️ γ 只取到 −40°:更陡時 PN 的推頭指令會把 pitch 頂到 `dive_pitch_max`
    #:    的夾制上,而夾制之後 pitch 是常數 ⇒ 兩組「剛好相同」是**假綠燈**。
    #:    `test_no_case_sits_on_a_clamp` 就是在守這件事。
    #: ⚠️ σ 只到 60°:RRE 實測最大的 81° 配上最快的剖面(vh 189.9、γ −35°)
    #:    會把 roll 頂到 r_lim = 80°·|cos θ| = 65.5° 的夾制上 —— 貼邊之後
    #:    兩組「剛好相同」是假綠燈。81° 由下面 `_EXTRA` 用較慢的剖面單獨覆蓋。
    CASES = [dict(sigma_deg=s, gam_deg=g, vh=v)
             for s in (5.0, 15.3, -35.0, 60.0)
             for g in (-15.0, -28.0, -35.0)
             for v in (142.7, 189.9)] + [
        dict(sigma_deg=81.0, gam_deg=-28.0, vh=142.7)]

    def test_no_case_sits_on_a_clamp(self):
        """⚠️ 顧測試自己:任何一組貼在 pitch/roll 夾制上,下面的逐位元比較
        就退化成「兩個常數相等」—— 通過但什麼都沒驗到。"""
        bad = [c for c in self.CASES
               for r in (_pn(_guid(mpng_on=True, **c)),)
               if r["pitch_clamped"] or r["roll_clamped"]]
        assert not bad, f"這些格點貼在夾制上,比較無效:{bad}"

    def test_the_alpha_is_exactly_zero_on_these_cases(self):
        """⚠️ 上面那段推理的前提。前提壞了要立刻知道,不要等到結論出錯。"""
        for c in self.CASES:
            r = _pn(_guid(mpng_on=True, **c))
            assert r["alpha"] == 0.0, c
            assert r["pitch"] == r["gam_cmd"], c

    def test_the_pitch_command_is_bit_identical(self):
        bad = []
        for c in self.CASES:
            off = _pn(_guid(itcg_on=False, **c))
            on = _pn(_guid(mpng_on=True, **c))
            assert on["a_t"] != 0.0, f"{c}:偏置是 0,這一組測不到東西"
            if (on["pitch"] != off["pitch"] or on["alpha"] != off["alpha"]
                    or on["gam"] != off["gam"]):
                bad.append((c, off["pitch"], on["pitch"]))
        assert not bad, (
            f"⚠️⚠️ 時間項動到了縱向解({len(bad)}/{len(self.CASES)} 組):"
            f"{bad[:3]}\n⇒ 命中誤差會跟著變,而這一輪要量的是**時間權限**,"
            f"兩者一起變就什麼都量不出來。")

    def test_the_normal_load_factor_is_bit_identical(self):
        """nz = cos γ + V·γ̇/g,而 γ̇ = (gam_cmd − γ)/pn_lead ⇒ nz 是 gam_cmd
        的雙射。獨立算一次而不是引用上一條 —— 它是 [[nz-clamp-is-the-dominant
        -predictor]] 裡與觸地誤差 spearman **+0.914** 的那個量,也就是說
        它動一點,觸地誤差就跟著動。"""
        for c in self.CASES:
            off, on = _pn(_guid(itcg_on=False, **c)), _pn(_guid(mpng_on=True, **c))
            nz = [math.cos(r["gam"]) + r["V"] * ((r["pitch"] - r["gam"]) / 1.0)
                  / GRAV for r in (off, on)]
            assert nz[0] == nz[1], f"{c}:nz 被時間項改動了 {nz}"

    def test_the_throttle_channel_is_bit_identical(self):
        """⚠️ 油門是**唯一的命中時刻通道**,而且與兩個轉向通道解耦。
        時間項若滲進油門,「橫向買時間」與「油門買時間」就疊在一起,
        權限上界會被高估。"""
        for c in self.CASES:
            assert _pn(_guid(itcg_on=False, **c))["thr"] == \
                   _pn(_guid(mpng_on=True, **c))["thr"], c

    def test_the_only_vertical_difference_is_the_known_roll_coupling(self):
        """⭐ α ≠ 0 時 pitch **會**不同 —— 把這件事本身也鎖住,並且鎖成
        **恰好**是那個已知的耦合項,不然「pitch 不同是正常的」會變成一張
        萬用免死金牌,真的動到縱向時也會被這句話蓋過去。

            pitch = gam_cmd + α/max(cos roll, 0.5)

        兩組的 α 相同、gam_cmd 相同,差別**只能**來自 cos(roll)。
        """
        off = _pn(_guid(itcg_on=False, alpha_deg=6.0))
        on = _pn(_guid(mpng_on=True, alpha_deg=6.0))
        assert on["alpha"] == off["alpha"] != 0.0, "α 本身不可以被時間項改動"
        assert on["pitch"] != off["pitch"], "反向鎖:這個工作點應該真的有耦合"
        # 把耦合項換算回去之後,兩組必須重合(容差只留給浮點捨入)
        assert on["gam_cmd"] == pytest.approx(
            off["gam_cmd"], rel=1e-12, abs=1e-12), (
            "扣掉已知的 cos(roll) 耦合之後縱向解仍然不同 ⇒ 真的被動到了")
        # 差值必須**完全**由 cos(roll) 解釋,沒有殘留
        want = on["alpha"] * (1.0 / max(math.cos(on["roll"]), 0.5)
                              - 1.0 / max(math.cos(off["roll"]), 0.5))
        assert on["pitch"] - off["pitch"] == pytest.approx(
            want, rel=1e-9, abs=1e-12), (
            "pitch 的差值超出 cos(roll) 耦合能解釋的範圍 ⇒ 有第二個來源")


# ══════════════════════════════════════════════════════════════════════════
#  M8 ⭐⭐ CSV 欄位契約:尾端追加、順序、未設值寫空字串
# ══════════════════════════════════════════════════════════════════════════
class _Cap:
    """把 `csv.writer` 換成攔截器 —— 我們要的是**送出去的那個 list**,
    不是它序列化之後的字串(序列化會把 '' 與 0 的差別藏進引號規則裡)。"""

    def __init__(self):
        self.rows = []

    def writerow(self, row):
        self.rows.append(list(row))


class Rec(_LOG):
    """記錄樁:只提供 `_open_csv` / `_row` 真的會碰到的東西。"""

    def __init__(self, tmpdir):
        self._dir = tmpdir
        self._csv, self._cw = None, _Cap()
        self.t0, self.phase = 0.0, "DIVE"
        self.ground_alt, self.target_amsl = 0.0, 0.0
        self._agl_datum_logged = True
        self.pos = {1: (24.0, 121.0, 3100.0)}
        self.vel = {1: (140.0, 10.0, 80.0)}
        self.gs, self.last_v = {1: 150.0}, {1: 160.0}
        self.pred, self.e_t, self.weave_theta, self.tgo_cache = {}, {}, {}, {}
        self.itcg_e, self.itcg_a, self.itcg_s, self.itcg_dbg = {}, {}, {}, {}
        self.req_radius, self.sent_radius = {}, {}
        self.wp_pt, self.tgo_obs = {}, {}

    # ── 與本組鎖無關的副作用一律做成 no-op ──
    def _predict_tick(self, i, now):
        pass

    def _observe_timing(self, i, now):
        pass

    def _kill_arm_update(self):
        pass

    def _kill_airframe(self, i, agl, now):
        pass

    def _agl(self, i):
        return 100.0

    def _zg(self, i):
        return 0.0

    def _lead_angle_deg(self, i):
        return 12.3

    def _csv_base(self):
        return str(pathlib.Path(self._dir) / "trial")

    def get_logger(self):
        return _Log()


class TestM8TheCsvColumnContract:
    """⚠️⚠️ **只能尾端追加。** `_tot_plot.py` / `_dive_plot.py` 用 `r[0..8]`
    位置解析,插在中間會把每一欄靜靜地平移一格 —— 圖照樣畫得出來,只是全錯。

    ⚠️⚠️ **未設值寫空字串不寫 0。** 0 是**合法的**偏置/誤差值,拿它當「沒跑」
       會讓事後判讀把「整條律沒啟動」誤讀成「剛好不需要出力」——
       而 `itcg_accel` 80,760 樣本全零那一批,就是這樣被看了很久才發現的。
    """

    #: B1 追加的六欄,順序即契約。
    NEW = ["itcg_req_a", "itcg_cmd_a", "itcg_eps_t",
           "itcg_tgo_ref", "itcg_tgo_jiang", "itcg_sat"]
    #: 位置解析的消費者吃的前九欄 —— 這九欄的**位置**是硬契約。
    HEAD9 = ["t", "sysid", "lat", "lon", "alt", "gs", "d_slot", "mode", "v_cmd"]

    @staticmethod
    def _header(tmp_path):
        s = Rec(tmp_path)
        s._open_csv()
        s._csv.close()
        with open(s._csv_base() + ".csv", newline="") as fh:
            return next(csv.reader(fh))

    def test_the_six_new_columns_are_the_last_six_in_order(self, tmp_path):
        hdr = self._header(tmp_path)
        assert hdr[-6:] == self.NEW, (
            f"⚠️⚠️ 六個新欄不在尾端或順序不對:{hdr[-8:]}")
        assert hdr[-10:-6] == ["req_radius_m", "cmd_radius_m", "wp_lat", "wp_lon"], (
            "⚠️ 新欄插到了 S2 那四欄的**前面** ⇒ S2 的判讀腳本會整批平移")

    def test_the_positional_prefix_is_untouched(self, tmp_path):
        assert self._header(tmp_path)[:9] == self.HEAD9, (
            "⚠️⚠️ 位置解析的前九欄被改動 ⇒ _tot_plot.py / _dive_plot.py "
            "會靜靜地畫出全錯的圖")

    def test_no_column_name_is_duplicated(self, tmp_path):
        hdr = self._header(tmp_path)
        dup = [c for c in set(hdr) if hdr.count(c) > 1]
        assert not dup, f"⚠️ 重複欄名 {dup} ⇒ DictReader 只留最後一個,前面的靜靜消失"

    def test_an_unrun_channel_writes_empty_strings_not_zeros(self, tmp_path):
        """⭐⭐ 這一輪最要緊的一條:通道沒跑時六欄必須全是 `''`。"""
        s = Rec(tmp_path)
        s._row(10.0, 1, 500.0, "dive", (24.0, 121.0))
        tail = s._cw.rows[0][-6:]
        assert tail == ["", "", "", "", "", ""], (
            f"⚠️⚠️ 通道沒跑卻寫了 {tail} —— 只要其中一個是 0,事後判讀就會把"
            f"「整條律沒啟動」讀成「剛好不需要出力」。itcg_accel 全零那一批"
            f"就是這樣被誤讀的。")
        assert 0 not in tail, "0 是合法值,不可以拿來表示『沒跑』"

    def test_a_live_channel_writes_the_recorded_values(self, tmp_path):
        s = Rec(tmp_path)
        s.itcg_dbg = {1: {"req_a": 12.3456789, "cmd_a": 3.92266,
                          "eps_t": 3.712345, "tgo_ref": 41.3,
                          "tgo_jiang": 44.371816, "sat": 1}}
        s._row(10.0, 1, 500.0, "dive", (24.0, 121.0))
        req, cmd, eps, ref, jiang, sat = s._cw.rows[0][-6:]
        assert req == pytest.approx(12.3457, abs=5e-5)
        assert cmd == pytest.approx(3.9227, abs=5e-5)
        assert eps == pytest.approx(3.712, abs=5e-4)
        assert ref == pytest.approx(41.3, abs=5e-3)
        assert jiang == pytest.approx(44.37, abs=5e-3)
        assert sat == 1
        assert req != cmd, (
            "⚠️ req 與 cmd 寫成同一個值 ⇒ 那兩欄的**全部價值**就是它們不同")

    def test_an_unavailable_jiang_tgo_is_empty_while_the_rest_are_numbers(self, tmp_path):
        """σ→180° 時 sin σ→0,Eq.(29) 算不出來 ⇒ 那**一欄**空著,
        其餘五欄照常。⚠️ 記錄層絕不可以因為一欄算不出來就整列消失。"""
        s = Rec(tmp_path)
        s.itcg_dbg = {1: {"req_a": 1.0, "cmd_a": 1.0, "eps_t": 2.0,
                          "tgo_ref": 41.3, "tgo_jiang": None, "sat": 0}}
        s._row(10.0, 1, 500.0, "dive", (24.0, 121.0))
        tail = s._cw.rows[0][-6:]
        assert tail[4] == "", "算不出來的 Eq.(29) 必須寫空字串"
        assert tail[:4] == [1.0, 1.0, 2.0, 41.3] and tail[5] == 0
        assert tail[5] == 0, "sat=0 是**合法值**,這一欄不可以被寫成空字串"

    def test_the_row_width_matches_the_header(self, tmp_path):
        s = Rec(tmp_path)
        s._open_csv()
        s._csv.close()
        with open(s._csv_base() + ".csv", newline="") as fh:
            hdr = next(csv.reader(fh))
        s._cw = _Cap()
        s._row(10.0, 1, 500.0, "dive", (24.0, 121.0))
        assert len(s._cw.rows[0]) == len(hdr), (
            f"⚠️⚠️ 表頭 {len(hdr)} 欄、資料列 {len(s._cw.rows[0])} 欄 ⇒ "
            f"DictReader 會靜靜地把尾巴丟掉或塞 None")


# ══════════════════════════════════════════════════════════════════════════
#  M9 ⭐⭐ Eq.(29) 的 t_go:只記錄,**不使用**
# ══════════════════════════════════════════════════════════════════════════
class TestM9TheJiangTgoIsRecordedButNeverActuates:
    """    tgo = (r/V)·(1 + N′·σ²)·(σ/sin σ)^{1/(N−1)}      (Eq.29)

    ⚠️⚠️ **不要**拿它取代現有的 t_go。它的優勢只在大前置角(原文行 1079
       逐字:σ 低於 40° 時三種估計法 "similar"),而我們 σ 中位 15.4°、
       32 架次裡 28 架 ≤ 23.8°。實測餵我們 32 架次:|誤差| 中位從現行的
       **3.73 s 變差到 4.28 s**。⇒ 並列記錄一欄,讓資料說話。

    ⚠️ 而且 N′ 的符號在原文**自相矛盾**:Eq.(28) 印 `(2−N)/(6(N−1)(2N−1))`、
       Eq.(39) 印 `(N−2)/…`。讓一個有爭議的常數影響指令是不可接受的 ——
       所以下面那條鎖直接**把常數換掉再看指令有沒有動**。
    """

    @staticmethod
    def _stub_with(flag):
        """把 `MPNG_NPRIME_2_MINUS_N` 換成 flag 的一份獨立實作。

        ⚠️⚠️ 基底順序是這條鎖的成敗:`Guid` 自己就繼承了預設常數的那一份,
           寫成 `(Guid, other)` 時 MRO 會先找到舊的,兩次迴圈跑的其實是
           **同一份程式碼** —— 鎖亮綠燈但什麼都沒驗到。
           (這正是本專案頭號失效類別在**測試側**的翻版。)
        """
        mix = _mixin(("_geom", "_itcg_bias", "_pn_cmd"), nprime_2_minus_n=flag)
        return type("GuidN", (mix, Guid), {})

    def test_flipping_the_disputed_constant_does_not_move_any_command(self):
        """⚠️⚠️ **必須在沒有飽和的工作點上驗。**

        實測(突變測試,2026-08-19):在自動增益的工作點上這條律整段貼在
        夾制上 ⇒ 就算真的把 `tgo_jiang` 接進致動路徑,輸出兩邊還是同樣等於
        cap,這條鎖會**亮綠燈放行**。夾制把訊號吃掉了。
        ⇒ 所以刻意給一組小增益讓它離開飽和,再比。σ=60° 是為了讓兩個 N′
          符號真的算出不同的 tgo(σ 小的時候兩式只差 0.24%)。
        """
        out = {}
        for flag in (True, False):
            s = self._stub_with(flag)(sigma_deg=60.0)
            s.mpng_on = True
            s.mpng_k_eps = s.mpng_k_s = 1e-9      # 遠離夾制,見上面
            a, _ = s.bias(3.7)
            roll = s._pn_cmd(1, s.now_for(3.7))[0]
            assert s.itcg_dbg[1]["sat"] == 0, (
                "⚠️ 這一點飽和了 ⇒ 夾制會把差異吃掉,這條鎖就形同虛設")
            out[flag] = (a, roll, s.itcg_dbg[1]["tgo_jiang"])
        assert out[True][:2] == out[False][:2], (
            f"⚠️⚠️ 換掉 Eq.(29) 的 N′ 符號竟然改變了指令 {out} ⇒ 它**不是**"
            f"只記錄不使用。原文對這個符號自相矛盾(Eq.28 印 (2−N)、"
            f"Eq.39 印 (N−2)),不可以讓它影響飛行。")
        assert out[True][2] != out[False][2], (
            "反向鎖:σ=60° 時兩個符號應該真的算出不同的 tgo,"
            "算出一樣代表這條鎖根本沒換到東西")

    def test_the_actuating_tgo_is_our_own_estimator(self):
        """⚠️ `tgo_ref` 才是**真的在用**的那個;`tgo_jiang` 只是並列。
        兩欄在同一列上,事後才有辦法量「換過去會變好還是變差」。"""
        s = _guid(mpng_on=True, tgo=37.0)
        s.bias(3.7)
        dbg = s.itcg_dbg[1]
        assert dbg["tgo_ref"] == 37.0
        assert dbg["tgo_jiang"] != 37.0, "反向鎖:兩者不該碰巧相同"

    def test_the_recorded_value_is_in_the_right_ballpark(self):
        """σ = 15.3° ⇒ (σ/sin σ)^{1/(N−1)} 只比 1 大一點點 ⇒ Eq.(29) 應該
        落在 r/V 與 r/V·1.05 之間。⚠️ 這是量綱與組成的體檢,不是精度斷言。"""
        s = _guid(mpng_on=True)
        s.bias(3.7)
        dn, de, d, h, R, vn, ve, vh, hdot, V = s._geom(1)
        assert R / V <= s.itcg_dbg[1]["tgo_jiang"] <= R / V * 1.05

    @pytest.mark.parametrize("sigma_deg", [0.0, 90.0, 179.999, -179.999])
    def test_it_never_blows_up_the_guidance_law(self, sigma_deg):
        """⚠️ 記錄層絕不可以因為算不出來就讓導引律爆掉 —— σ→180° 時
        sin σ→0。算不出來就是**沒有值**(寫空字串,見 M8)。"""
        s = _guid(sigma_deg=sigma_deg, mpng_on=True)
        a, _ = s.bias(3.7)
        assert isinstance(a, float) and math.isfinite(a)
        assert isinstance(s._pn_cmd(1, s.now_for(3.7))[0], float)
